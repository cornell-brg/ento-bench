#include <iostream>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include "FixedPoint.hh"
#include <Eigen/Dense>
#include "RawImage.cpp"
#include "ImagePyramid.cpp"

// g++ -std=c++11 $(pkg-config --cflags eigen3 --libs opencv4) lk_optical_flow_fp_gem5.cpp -o lk_optical_flow_fp_gem5  -I/opt/homebrew/include
// ./lk_optical_flow_fp_gem5 -MAIN_ITERATIONS=150 -NUM_LEVELS=1 -MAX_COUNT=5 -WIN_DIM=15 -FRAME_RATE=150 -TEST_FOLDER=5 -FIRST_IMG=32

using namespace cv;
using namespace std;

#define MAX_WIN_DIM 15
#define IMG_SAVE
// Algorithm parameters
// TODO: constexpr
int   NUM_POINTS        = 4;
int   WIN_DIM           = 15;
int   NUM_LEVELS        = 1;
int   MAX_COUNT         = 10;
int   POINT_REFRESH     = 32;
int   IMG_WIDTH         = 320;
int   IMG_HEIGHT        = 320;
int   DET_EPSILON       = (int)(1<<20);
float CRITERIA          = 0.01;

// Testing / visualizing params
int FIRST_IMG = 2;
int FRAME_RATE = 10;
int TEST_FOLDER = 4;
int MAIN_ITERATIONS = 200;
int PT_IND = 0;

RawImage* prevImg;
RawImage* nextImg;
ImagePyramid* prevPyramid;
ImagePyramid* nextPyramid;

const short scharr_x_arr[] = {-3, 0, 3, -10, 0, 10, -3, 0, 3};
const short scharr_y_arr[] = {-3, -10, -3, 0, 0, 0, 3, 10, 3};

// TODO: long vs longlong on cortex m4
const int decimal_bits = 20;
using fp_t = FixedPoint<64-decimal_bits, decimal_bits, int64_t>;
using PointFP = std::tuple<fp_t, fp_t>;

const fp_t fp_1(1);
const fp_t fp_2(2);

// -----------------------
// DEBUGGING FUNCTIONS
// -----------------------

Point2f fixed_tuple_to_point(PointFP fp) {
    Point2f point(get<0>(fp).to_float(), get<1>(fp).to_float());
    return point;
}

// -----------------------
// END DEBUGGING FUNCTIONS
// -----------------------

void calc_gradient(const PointFP & pt, const RawImage & src, short* Ix_arr, short* Iy_arr, const int dim, const int halfWin) {
    
    // Indices for top left corner of window in src
    int32_t x_i = get<0>(pt).to_int32()-halfWin;
    int32_t y_i = get<1>(pt).to_int32()-halfWin;

    for (int i = 0; i < dim; i++) { // row / height / y
        for (int j = 0; j < dim; j++) { // col / width / x
            // kernel loop
            int Ix = 0;
            int Iy = 0;
            for (int k = 0; k < 3; k++) { // row / height / y
                for (int l = 0; l < 3; l++) { // col / width / x
                    int y_index = i+k+y_i - 1;
                    if (y_index < 0) y_index = 0;
                    else if (y_index >= src.height) y_index = src.height-1;

                    int x_index = j+l+x_i - 1;
                    if (x_index < 0) x_index = 0;
                    else if (x_index >= src.width) x_index = src.width-1;

                    Ix += ((short) src.data[src.height*y_index + x_index])*scharr_x_arr[k*3+l];
                    Iy += ((short) src.data[src.height*y_index + x_index])*scharr_y_arr[k*3+l];
                }
            }
            Ix_arr[dim*i + j] = round(Ix / 32);
            Iy_arr[dim*i + j] = round(Iy / 32);
        }
    }
}

template <typename SrcT>
void interpolate(const PointFP & pt, const SrcT * src, Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> & dst, int src_width, int src_height) {
    int32_t x_i = get<0>(pt).to_int32();
    int32_t y_i = get<1>(pt).to_int32();
    fp_t a = get<0>(pt) - fp_t(x_i);
    fp_t b = get<1>(pt) - fp_t(y_i);

    fp_t iw00((fp_1 - a) * (fp_1 - b));
    fp_t iw01(a*(fp_1-b));
    fp_t iw10((fp_1-a)*b);
    fp_t iw11(fp_1- iw00 - iw01 - iw10);

    for (int k = 0; k < WIN_DIM; k++) { // row / height / y
        for (int l = 0; l < WIN_DIM; l++) { // col / width / x
            int l_plus = l+1;
            if (l_plus >= src_width) l_plus = src_width - 1;

            int k_plus = k+1;
            if (k_plus >= src_height) l_plus = src_height - 1;

            if (l_plus >= src_width) l_plus = src_width - 1;
            fp_t ival(iw00 * fp_t(src[src_width*(k)     + l    ]) + 
                      iw01 * fp_t(src[src_width*(k)     + l + 1]) +
                      iw10 * fp_t(src[src_width*(k + 1) + l    ]) + 
                      iw11 * fp_t(src[src_width*(k + 1) + l + 1]));
            dst(k, l) = ival;
        }
    }
}

void calcOpticalFlowPyrLKSimpleIter( const RawImage & prevImg, const RawImage & nextImg,
                               const PointFP* prevPts, PointFP* nextPts,
                               bool* status, int num_good_points )
{

    int area = WIN_DIM * WIN_DIM;
    int halfWin = WIN_DIM / 2;

    for ( int i = 0; i < num_good_points; i++ ) {

        // Get previous point (fixed point and int)
        fp_t x(get<0>(nextPts[i]));
        fp_t y(get<1>(nextPts[i]));
        int x_i = x.to_int32();
        int y_i = y.to_int32();
        
        PointFP prevPt = prevPts[i];

        // Initalize guess pt, uv, and success
        PointFP guessPt = make_tuple(x, y);
        fp_t u(0);
        fp_t v(0);
        bool success = false;

        // If point is out of bounds, status is failed
        if ( x_i >= prevImg.width || y_i >= prevImg.height || x_i < 0 || y_i < 0) {
            cout << "Point out of bounds " << "(" << x_i << ", " << y_i << ")" << endl;
            status[i] = 0;
            nextPts[i] = guessPt;
            continue;
        }

        // Calculate gradients
        short Ix_arr[(WIN_DIM+1)*(WIN_DIM+1)];
        short Iy_arr[(WIN_DIM+1)*(WIN_DIM+1)];
        Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Ix_win_square;
        Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> Iy_win_square;
        calc_gradient(prevPt, prevImg, Ix_arr, Iy_arr, WIN_DIM+1, halfWin);
        interpolate<short>(prevPt, Ix_arr, Ix_win_square, WIN_DIM+1, WIN_DIM+1);
        interpolate<short>(prevPt, Iy_arr, Iy_win_square, WIN_DIM+1, WIN_DIM+1);

        // Calculate ATA components and inverse
        fp_t IxIx(0);
        fp_t IyIy(0);
        fp_t IxIy(0);
        for (int j = 0; j < WIN_DIM; j++) {
            for (int k = 0; k < WIN_DIM; k++) {
                IxIx += (Ix_win_square(j, k) * Ix_win_square(j, k));
                IyIy += (Iy_win_square(j, k) * Iy_win_square(j, k));
                IxIy += (Ix_win_square(j, k) * Iy_win_square(j, k));
            }
        }

        // Separate int and decimal calculations because determinant needs 20 integer bits
        int IxIx_int = IxIx.to_int32();
        int IyIy_int = IyIy.to_int32();
        int IxIy_int = IxIy.to_int32();
        fp_t det = IxIx * IyIy - IxIy * IxIy;
        long det_int = ((long)IxIx_int) * ((long)IyIy_int) - ((long)IxIy_int) * ((long)IxIy_int);
        float full_det = ((float) det_int) + det.get_decimal(); // restore the integer bits
        det = fp_t(full_det); 

        // If matrix is nonsingular, mark status as failed
        if (det.to_int32() < DET_EPSILON && det.to_int32() > (-DET_EPSILON)) {
            cout << "Singular matrix at point (" << x << ", " << y << ")" << endl;
            status[i] = 0;
            nextPts[i] = guessPt;
            continue;
        }

        // Calculate ATA_inv since the determinant is valid
        Eigen::Matrix<fp_t, 2, 2> ATA;
        ATA << IxIx, IxIy, IxIy, IyIy;
        Eigen::Matrix<fp_t, 2, 2> ATA_inv;
        ATA_inv << IxIx / det, -IxIy / det, -IxIy / det, IyIy / det;

        // Calculate prev win
        Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> It_win_p;
        interpolate<uchar>(prevPt, ((u_char *)prevImg.data)+(y_i - halfWin)*prevImg.width+(x_i-halfWin), It_win_p, prevImg.width, prevImg.height-y_i);

        for (int j = 0; j < MAX_COUNT; j++) {

            // Calculate the time derivative
            Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> It_win_n;
            int guessPt_y_i = get<1>(guessPt).to_int32();
            int guessPt_x_i = get<0>(guessPt).to_int32();
            interpolate<uchar>(guessPt, ((u_char *)nextImg.data)+(guessPt_y_i - halfWin)*nextImg.width+(guessPt_x_i-halfWin), It_win_n, nextImg.width, nextImg.height-guessPt_y_i); 
            Eigen::Matrix<fp_t, MAX_WIN_DIM, MAX_WIN_DIM> It_win = It_win_n - It_win_p;

            // Compute b
            fp_t b0(0), b1(0);
            for (int k = 0; k < WIN_DIM; k++) {
                for (int l = 0; l < WIN_DIM; l++) {
                    b0 = b0 + Ix_win_square(k, l) * It_win(k, l);
                    b1 = b1 + Iy_win_square(k, l) * It_win(k, l);
                }
            }
            Eigen::Matrix<fp_t, 2, 1> b;
            b << b0 , b1;

            // Compute the least squares solution
            Eigen::Matrix<fp_t, 2, 1> soln = ATA_inv * (-b);
            u = soln(0, 0);
            v = soln(1, 0);
            success = true;
            get<0>(guessPt) += u;
            get<1>(guessPt) += v;

            if ((u * u + v * v) <= fp_t(CRITERIA)) break;        
        }

        status[i] = success;
        nextPts[i] = guessPt;
    }
}

void calcOpticalFlowPyrLKPyr(const PointFP* prevPts, PointFP* nextPts,
                               bool* status, int num_good_points)
{

    prevPyramid->create_pyramids();
    nextPyramid->create_pyramids();

    fp_t divFactor(1 << NUM_LEVELS);

    // Initialize highest layer points
    PointFP prevPyrPoints[num_good_points];
    for (int i = 0; i < num_good_points; i++) {
        prevPyrPoints[i] = PointFP(get<0>(prevPts[i]) / divFactor, get<1>(prevPts[i]) / divFactor);
        nextPts[i] = PointFP(get<0>(prevPts[i]) / divFactor, get<1>(prevPts[i]) / divFactor);
    }
    
    for (int i = NUM_LEVELS; i >= 0; i--) {
        calcOpticalFlowPyrLKSimpleIter(prevPyramid->get_level(i), nextPyramid->get_level(i), prevPyrPoints, nextPts, status, num_good_points);
        if (i != 0) {
            for (int j = 0; j < num_good_points; j ++) {
            prevPyrPoints[j] = PointFP(get<0>(prevPyrPoints[j]) * fp_2, get<1>(prevPyrPoints[j]) * fp_2);
            nextPts[j] = PointFP(get<0>(nextPts[j]) * fp_2, get<1>(nextPts[j]) * fp_2);
            }
        }
    }
}

int main(int argc, char **argv)
{

     const String keys =
    "{help h usage ? | | print this message }"
    "{NUM_POINTS | 1 | number of points to track }"
    "{WIN_DIM | 15 | shape of the window }"
    "{NUM_LEVELS | 1 | number of levels of pyramid }"
    "{CRITERIA | 0.01 | CRITERIA for iterative algorithm termination }"
    "{MAX_COUNT | 10 | maximum number of iterations }"
    "{MAIN_ITERATIONS | 200 | number of simulation iterations }"
    "{POINT_REFRESH | 32 | rate in frames to generate new points }"
    "{FIRST_IMG | 2 | first image in test data }"
    "{IMG_WIDTH | 320 | first image in test data }"
    "{IMG_HEIGHT | 320 | first image in test data }"
    "{FRAME_RATE | 10 | frame rate for display }"
    "{TEST_FOLDER | 4 | folder for test data }"
    "{SAVE_NAME | final.png | name for final image saved }"
    "{PT_IND | final.png | name for final image saved }"
    ;
    
    CommandLineParser parser(argc, argv, keys);
    parser.about("Application name v1.0.0");
    if (parser.has("help"))
    {
    parser.printMessage();
    return 0;
    }
    NUM_POINTS = parser.get<int>("NUM_POINTS");
    WIN_DIM = parser.get<int>("WIN_DIM");
    NUM_LEVELS = parser.get<int>("NUM_LEVELS");
    CRITERIA = parser.get<float>("CRITERIA");
    MAX_COUNT = parser.get<int>("MAX_COUNT");
    MAIN_ITERATIONS = parser.get<int>("MAIN_ITERATIONS");
    POINT_REFRESH = parser.get<int>("POINT_REFRESH");
    FIRST_IMG = parser.get<int>("FIRST_IMG");
    IMG_WIDTH = parser.get<int>("IMG_WIDTH");
    IMG_HEIGHT = parser.get<int>("IMG_HEIGHT");
    FRAME_RATE = parser.get<int>("FRAME_RATE");
    TEST_FOLDER = parser.get<int>("TEST_FOLDER");
    PT_IND = parser.get<int>("PT_IND");
    string SAVE_NAME = parser.get<string>("SAVE_NAME");
    string directory = "/Users/acui21/Documents/brg/FigureEight_test"+to_string(TEST_FOLDER)+"_images/";

    // initialize images and pyramids
    prevImg = new_image(IMG_WIDTH, IMG_HEIGHT);
    nextImg = new_image(IMG_WIDTH, IMG_HEIGHT);
    prevPyramid = new ImagePyramid(prevImg, IMG_HEIGHT, IMG_WIDTH, NUM_LEVELS);
    nextPyramid = new ImagePyramid(nextImg, IMG_HEIGHT, IMG_WIDTH, NUM_LEVELS);

    // Initialize points and status
    PointFP *prevPts = new PointFP[NUM_POINTS];
    PointFP *nextPts = new PointFP[NUM_POINTS];
    bool *status_simple = new bool[NUM_POINTS];

    // Create colors
    Scalar cv_color = Scalar(0, 255, 0);
    RNG rng;
    Scalar own_color = Scalar(0, 0, 255);

    Mat old_frame, old_gray;
    vector<Point2f> p0, p1;
    
    // Take first frame and find corners in it
    VideoCapture capture(directory);
    capture >> old_frame;
    int count = FIRST_IMG;
    old_frame = imread(directory + "image_" + to_string(count) + ".png", IMREAD_COLOR);
    if (old_frame.empty()) {
        std::cerr << "Error: Could not open or find the image." << std::endl;
        return -1;
    }

    // Initialize image
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    for (int i = 0; i < IMG_HEIGHT; i++) {
        for (int j = 0; j < IMG_WIDTH; j++) {
            prevImg->data[i*IMG_HEIGHT+j] = old_gray.at<u_char>(i, j);
        }
    }

    // Initialize points
    vector<Point2f> all_pts;
    goodFeaturesToTrack(old_gray, all_pts, 4, 0.3, 7, Mat(), 7, false, 0.04);
    p0.push_back(all_pts[PT_IND]);
    int num_good_points = p0.size();
    for (int i = 0; i < num_good_points; i++) {
        fp_t x(p0[i].x);
        fp_t y(p0[i].y);
        prevPts[i] = PointFP(x,y);
    }

    // Create a mask image for drawing purposes
    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
    
    float error = 0;

    while(count < FIRST_IMG + MAIN_ITERATIONS){ 
        count++;
        Mat frame, frame_gray;

        frame = imread(directory + "image_" + to_string(count) + ".png", IMREAD_COLOR);

        if (frame.empty())
            break;
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        
        // Update next image
        for (int i = 0; i < IMG_HEIGHT; i++) {
            for (int j = 0; j < IMG_WIDTH; j++) {
                nextImg->data[i*IMG_HEIGHT+j] = frame_gray.at<u_char>(i, j);
            }
        }

        // calculate optical flow
        vector<uchar> status;
        vector<float> err;
        TermCriteria CRITERIA = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15, 15), 2, CRITERIA);
        calcOpticalFlowPyrLKPyr(prevPts, nextPts, status_simple, num_good_points);

        // TODO: remove this error calc
        float opencv_flow_vector_x, opencv_flow_vector_y;
        float custom_flow_vector_x, custom_flow_vector_y;
        float diff_x, diff_y;
        float iter_error = 0;
        int num_pts = 0;

        for (int i = 0; i < 1; i++) {
            if (!status_simple[i]) {
                // opencv_flow_vector_x = p1[i].x - p0[i].x;
                // opencv_flow_vector_y = p1[i].y - p0[i].y;
                // custom_flow_vector_x = get<0>(nextPts[i]).to_float() - get<0>(prevPts[i]).to_float();
                // custom_flow_vector_y = get<1>(nextPts[i]).to_float() - get<1>(prevPts[i]).to_float();
                diff_x = p1[i].x - get<0>(nextPts[i]).to_float();
                diff_y = p1[i].y - get<1>(nextPts[i]).to_float();
                // iter_error += sqrt(diff_x*diff_x + diff_y*diff_y);
                iter_error++;
                num_pts ++;
            }
        }
        if (num_pts != 0) error += iter_error / num_pts;
        //TODO: remove this error calc

        vector<Point2f> good_new;
        vector<PointFP> good_new_simple;
        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                line(mask,p1[i], p0[i], cv_color, 1);
                circle(frame, p1[i], 1, cv_color, -1);
            }
        }

        for(uint i = 0; i < num_good_points; i++)
        {
            // Select good points
            if(status_simple[i]) {
                good_new_simple.push_back(nextPts[i]);
                // draw the tracks
                line(mask,fixed_tuple_to_point(nextPts[i]), fixed_tuple_to_point(prevPts[i]), own_color, 2);
                circle(frame, fixed_tuple_to_point(nextPts[i]), 1, own_color, -1);
            }
        }
        Mat img;
        add(frame, mask, img);
        imshow("Frame", img);

        int keyboard = waitKey(FRAME_RATE);
        if (keyboard == 'q' || keyboard == 27)
            break;

        // TODO: error calc
        // l2 norm of flow vectors (use opencv prev point)
        // l2 norm of coordinates for cumulative error
        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();
        if ((count-FIRST_IMG) % POINT_REFRESH == 0) {
            vector<Point2f> refresh_pts;
            goodFeaturesToTrack(old_gray, p0, NUM_POINTS, 0.3, 7, Mat(), 7, false, 0.04);
            // goodFeaturesToTrack(old_gray, refresh_pts, 4, 0.3, 7, Mat(), 7, false, 0.04);
            // double distance = 1000000; 
            // for (int i = 0; i < refresh_pts.size(); i++) {
            //     double res = norm(refresh_pts[i]-p0[0]);
            //     if (res < distance) {
            //         distance = res;
            //         p0[0] = refresh_pts[i];
            //     }
            // }
            num_good_points = p0.size();
            for (int i = 0; i < num_good_points; i++) {
                fp_t x(p0[i].x);
                fp_t y(p0[i].y);
                PointFP point(x,y);
                prevPts[i] = point;
            }

            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);

            own_color = Scalar(r, g, b);
        }

        else {
            p0 = good_new;

            num_good_points = good_new_simple.size();
            for (int i = 0; i < num_good_points; i ++) {
                prevPts[i] = good_new_simple[i];
            }

        }
        
        if (count == MAIN_ITERATIONS+FIRST_IMG) cv::imwrite("images/"+SAVE_NAME, img);
        
        // swap pyramids
        ImagePyramid* tempPyramid = prevPyramid;
        prevPyramid = nextPyramid;
        nextPyramid = tempPyramid;
        
        // swap images
        RawImage* tempImage = prevImg;
        prevImg = nextImg;
        nextImg = tempImage;
    }
    cout << error << endl;
    cout << error / MAIN_ITERATIONS << endl;

    del_raw_image(prevImg);
    del_raw_image(nextImg);
    delete prevPyramid;
    delete nextPyramid;
    delete [] prevPts;
    delete [] nextPts;
    delete [] status_simple;

}