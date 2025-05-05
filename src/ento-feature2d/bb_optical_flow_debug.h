#include <iostream>

inline uint32_t usada8(uint32_t a, uint32_t b, uint32_t acc) {
    for (int i = 0; i < 4; ++i) {
        uint8_t byte_a = (a >> (8 * i)) & 0xFF;
        uint8_t byte_b = (b >> (8 * i)) & 0xFF;
        acc += std::abs((int)byte_a - (int)byte_b);
    }
    return acc;
}

inline uint32_t uhadd8(uint32_t a, uint32_t b) {
    uint32_t result = 0;
    for (int i = 0; i < 4; ++i) {
        uint8_t byte_a = (a >> (8 * i)) & 0xFF;
        uint8_t byte_b = (b >> (8 * i)) & 0xFF;

        // Unsigned halving add: average of the two bytes (rounding down)
        uint8_t avg = (byte_a + byte_b) >> 1;

        result |= (uint32_t(avg) << (8 * i));
    }
    return result;
}

inline int absdiff(const uint8_t* frame1, const uint8_t* frame2) {
    const int stride = 320; // assumed row size
    uint32_t result = 0;

    for (int row = 0; row < 8; row++) {
        int base = row * stride;

        // first 4 pixels in the row
        uint32_t a0 = *reinterpret_cast<const uint32_t*>(&frame1[base + 0]);
        uint32_t b0 = *reinterpret_cast<const uint32_t*>(&frame2[base + 0]);
        result = usada8(a0, b0, result);

        // second 4 pixels in the row
        uint32_t a1 = *reinterpret_cast<const uint32_t*>(&frame1[base + 4]);
        uint32_t b1 = *reinterpret_cast<const uint32_t*>(&frame2[base + 4]);
        result = usada8(a1, b1, result);
    }

    return result;
}

/**
 * @brief Compute SAD distances of subpixel shift of two 8x8 pixel patterns.
 *
 * @param image1  an array holding pixel data
 * @param image2  an array holding pixel data
 * @param off1X   x coordinate of upper left corner of pattern in image1
 * @param off1Y   y coordinate of upper left corner of pattern in image1
 * @param off2X   x coordinate of upper left corner of pattern in image2
 * @param off2Y   y coordinate of upper left corner of pattern in image2
 * @param acc     array to store SAD distances for shift in every direction
 *
 * @return        zero
 */
static inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for (uint16_t i = 0; i < 8; i++)
	{
		acc[i] = 0;
	}


	/*
	 * calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	 * every iteration is one line of the 8x8 field.
	 *
	 *  + - + - + - + - + - + - + - + - +
	 *  |   |   |   |   |   |   |   |   |
	 *  + - + - + - + - + - + - + - + - +
	 */

	for (uint16_t i = 0; i < 8; i++)
	{
		/*
		 * first column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  | x | x | x | x |   |   |   |   |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 * the 8 s values are from following positions for each pixel (X):
		 *  + - + - + - +
		 *  +   5   7   +
		 *  + - + 6 + - +
		 *  +   4 X 0   +
		 *  + - + 2 + - +
		 *  +   3   1   +
		 *  + - + - + - +
		 *
		 *  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		 */

		/* compute average of two pixel values */
		s0 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+0) * row_size])));
		s1 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+1) * row_size])));
		s2 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i+1) * row_size])));
		s3 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+1) * row_size])));
		s4 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+0) * row_size])));
		s5 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i-1) * row_size])));
		s6 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i-1) * row_size])));
		s7 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i-1) * row_size])));

		/* these 4 t values are from the corners around the center pixel */
		t1 = (uhadd8(s0, s1));
		t3 = (uhadd8(s3, s4));
		t5 = (uhadd8(s4, s5));
		t7 = (uhadd8(s7, s0));

		/*
		 * finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		 *  + - + - + - +
		 *  |   |   |   |
		 *  + - 5 6 7 - +
		 *  |   4 X 0   |
		 *  + - 3 2 1 - +
		 *  |   |   |   |
		 *  + - + - + - +
		 */

		/* fill accumulation vector */
		acc[0] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s0, acc[0]);
		acc[1] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t1, acc[1]);
		acc[2] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s2, acc[2]);
		acc[3] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t3, acc[3]);
		acc[4] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s4, acc[4]);
		acc[5] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t5, acc[5]);
		acc[6] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s6, acc[6]);
		acc[7] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t7, acc[7]);

		/*
		 * same for second column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  |   |   |   |   | x | x | x | x |
		 *  + - + - + - + - + - + - + - + - +
		 */

		s0 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+0) * row_size])));
		s1 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+1) * row_size])));
		s2 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i+1) * row_size])));
		s3 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+1) * row_size])));
		s4 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+0) * row_size])));
		s5 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i-1) * row_size])));
		s6 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i-1) * row_size])));
		s7 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i-1) * row_size])));

		t1 = (uhadd8(s0, s1));
		t3 = (uhadd8(s3, s4));
		t5 = (uhadd8(s4, s5));
		t7 = (uhadd8(s7, s0));

		acc[0] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s0, acc[0]);
		acc[1] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t1, acc[1]);
		acc[2] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s2, acc[2]);
		acc[3] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t3, acc[3]);
		acc[4] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s4, acc[4]);
		acc[5] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t5, acc[5]);
		acc[6] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s6, acc[6]);
		acc[7] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t7, acc[7]);
	}

	return 0;
}