#include <cstdio>
#include <array>
#include "image_io/Pixel.h"
#include "image_io/Image.h"
#include "feature2d/fast.h"
#include "ento-util/debug.h"

void checkCondition(bool condition, const char* testName) {
  if (condition) {
    printf("%s passed.\n", testName);
  } else {
    printf("%s not passed.\n", testName);
  }
}

// Test creation of FeatureDetectorOutput
void test_fdo_creation() {
  printf("Running test_fdo_creation...\n");

  FeatureDetectorOutput<FastKeypoint, 100> fdo;
  checkCondition(fdo.size() == 0, "Initial FDO size is zero");

  FastKeypoint kp1(10, 20, 9);
  fdo.add_keypoint(kp1);
  checkCondition(fdo.size() == 1, "FDO size after adding one keypoint");

  DPRINTF("[TEST_FDO_CREATION] Feature stored in fd[0]: Coord(%i, %i), Score(%i)\n",
          fdo[0].x, fdo[0].y, fdo[0].score);
  checkCondition(fdo[0].x == 10 && fdo[0].y == 20 && fdo[0].score == 9, "Correct keypoint stored in FDO");

  printf("test_fdo_creation finished.\n\n");
}

// Test threshold table creation
void test_threshold_table() {
  printf("Running test_threshold_table...\n");

  constexpr int threshold = 4;
  constexpr int bit_depth = 4;
  constexpr int tab_size = (1 << (bit_depth + 1)) + 1;
  constexpr int half = tab_size / 2;

  DPRINTF("[TEST_THRESHOLD_TABLE] tab_size: %i\n", tab_size);
  DPRINTF("[TEST_THRESHOLD_TABLE] half: %i\n", half);

  auto threshold_tab = ThresholdTable<bit_depth, threshold>::table;
  bool status = 1;
  for (int i = 0; i < tab_size; ++i)
  {
    int dist = i - half;
    //DPRINTF("[TEST_THRESHOLD_TABLE] threshold_tab[%i] = %i\n", i, threshold_tab[i]);
    //DPRINTF("[TEST_THRESHOLD_TABLE] dist: %i\n", dist);
    if (dist < -threshold)
    {
      status &= threshold_tab[i] == 1;
      checkCondition(threshold_tab[i] == 1, "Threshold table underflow test");
    }
    else if (dist > threshold)
    {
      status &= threshold_tab[i] == 2;
      checkCondition(threshold_tab[i] == 2, "Threshold table overflow test");
    }
    else
    {
      status &= threshold_tab[i] == 0;
      checkCondition(threshold_tab[i] == 0, "Threshold table middle test");
    }
    //DPRINTF("\n");
  }
  checkCondition(status, "Threshold table all tests");
  printf("test_threshold_table finished.\n\n");
}

// Test Bresenham circle creation
void test_bressenham_circle()
{
  printf("Running test_bressenham_circle...\n");

  constexpr int pattern_size = 16;
  constexpr int row_stride = 320;

  auto circle = generate_bresenham_circle<uint16_t, pattern_size, row_stride, 9>();

  checkCondition(circle.size() == pattern_size, "Circle size check");

  // Add checks for known values in the Bresenham circle
  printf("Circle offsets: ");
  for (const auto& offset : circle) {
    printf("%d ", offset);
  }
  printf("\n");

  printf("test_bressenham_circle finished.\n\n");
}

// Test fast algorithm on an image
void test_fast_algorithm_1()
{
  printf("Running test_fast_algorithm...\n");

  using TestImgType = Image<7, 7, uint8_t>;
  TestImgType img;
  FeatureDetectorOutput<FastKeypoint, 100> fdo;

  const char* pgm_path = "/home/ddo26/workspace/entomoton-bench/datasets/unittest/test_fast1.pgm";
  //const char* pgm_path = "/home/ddo26/workspace/entomoton-bench/datasets/unittest/test_pgm.pgm";
  bool img_opened = img.image_from_pgm(pgm_path);
  checkCondition(img_opened, "Load FAST Test 1 pgm");
  if (!img_opened) return;

  fast<TestImgType, 16, 10>(img, fdo);  // Run the FAST detector

  checkCondition(fdo.size() > 0, "FAST detector found features");
  printf("FAST detected %i features.\n", fdo.size());

  auto x = fdo[0].x;
  auto y = fdo[0].y;

  checkCondition((x == 3) & (y == 3), "Fast detector feature location");
  printf("FAST detected feature at (%i, %i)\n", fdo[0].x, fdo[0].y);

  printf("test_fast_algorithm finished.\n");
}

// Test fast algorithm on an image
void test_fast_algorithm_2()
{
  printf("Running test_fast_algorithm...\n");

  using TestImgType = Image<14, 14, uint8_t>;
  TestImgType img;
  FeatureDetectorOutput<FastKeypoint, 100> fdo;

  const char* pgm_path = "/home/ddo26/workspace/entomoton-bench/datasets/unittest/test_fast2.pgm";
  //const char* pgm_path = "/home/ddo26/workspace/entomoton-bench/datasets/unittest/test_pgm.pgm";
  bool img_opened = img.image_from_pgm(pgm_path);
  checkCondition(img_opened, "Load FAST Test 1 pgm");
  if (!img_opened) return;

  fast<TestImgType, 16, 10>(img, fdo);  // Run the FAST detector

  checkCondition(fdo.size() == 4, "FAST detector found 4 features");
  printf("FAST detected %i features.\n", fdo.size());


  uint8_t gtxs[4] = {3, 10,  3, 10};
  uint8_t gtys[4] = {3,  3, 10, 10};

  for (int i = 0; i < 4; i++)
  {
    auto x = fdo[i].x;  auto y = fdo[i].y;
    auto gtx = gtxs[i]; auto gty = gtys[i];
    checkCondition((x == gtx) & (y == gty), "Fast detector feature location");
    printf("FAST detected feature at (%i, %i)\n", x, y);

  }

  printf("test_fast_algorithm finished.\n");
}

int main() {
  test_fdo_creation();
  test_threshold_table();
  test_bressenham_circle();
  test_fast_algorithm_1();
  test_fast_algorithm_2();

  printf("All tests executed!\n");
  return 0;
}
