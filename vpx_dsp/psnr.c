/*
*  Copyright (c) 2016 The WebM project authors. All Rights Reserved.
*
*  Use of this source code is governed by a BSD-style license
*  that can be found in the LICENSE file in the root of the source
*  tree. An additional intellectual property rights grant can be found
*  in the file PATENTS.  All contributing project authors may
*  be found in the AUTHORS file in the root of the source tree.
*/

#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "./vpx_dsp_rtcd.h"
#include "vpx_dsp/psnr.h"
#include "vpx_scale/yv12config.h"

double vpx_sse_to_psnr(double samples, double peak, double sse) {
  if (sse > 0.0) {
    const double psnr = 10.0 * log10(samples * peak * peak / sse);
    return psnr > MAX_PSNR ? MAX_PSNR : psnr;
  } else {
    return MAX_PSNR;
  }
}

/* TODO(yaowu): The block_variance calls the unoptimized versions of variance()
* and highbd_8_variance(). It should not.
*/
static void encoder_variance(const uint8_t *a, int a_stride, const uint8_t *b,
                             int b_stride, int w, int h, unsigned int *sse,
                             int *sum) {
  int i, j;

  *sum = 0;
  *sse = 0;

  for (i = 0; i < h; i++) {
    for (j = 0; j < w; j++) {
      const int diff = a[j] - b[j];
      *sum += diff;
      *sse += diff * diff;
    }

    a += a_stride;
    b += b_stride;
  }
}

#if CONFIG_VP9_HIGHBITDEPTH
static void encoder_highbd_variance64(const uint8_t *a8, int a_stride,
                                      const uint8_t *b8, int b_stride, int w,
                                      int h, uint64_t *sse, int64_t *sum) {
  int i, j;

  uint16_t *a = CONVERT_TO_SHORTPTR(a8);
  uint16_t *b = CONVERT_TO_SHORTPTR(b8);
  *sum = 0;
  *sse = 0;

  for (i = 0; i < h; i++) {
    for (j = 0; j < w; j++) {
      const int diff = a[j] - b[j];
      *sum += diff;
      *sse += diff * diff;
    }
    a += a_stride;
    b += b_stride;
  }
}

static void encoder_highbd_8_variance(const uint8_t *a8, int a_stride,
                                      const uint8_t *b8, int b_stride, int w,
                                      int h, unsigned int *sse, int *sum) {
  uint64_t sse_long = 0;
  int64_t sum_long = 0;
  encoder_highbd_variance64(a8, a_stride, b8, b_stride, w, h, &sse_long,
                            &sum_long);
  *sse = (unsigned int)sse_long;
  *sum = (int)sum_long;
}
#endif  // CONFIG_VP9_HIGHBITDEPTH

static int64_t get_sse(const uint8_t *a, int a_stride, const uint8_t *b,
                       int b_stride, int width, int height) {
  const int dw = width % 16;
  const int dh = height % 16;
  int64_t total_sse = 0;
  unsigned int sse = 0;
  int sum = 0;
  int x, y;

  if (dw > 0) {
    encoder_variance(&a[width - dw], a_stride, &b[width - dw], b_stride, dw,
                     height, &sse, &sum);
    total_sse += sse;
  }

  if (dh > 0) {
    encoder_variance(&a[(height - dh) * a_stride], a_stride,
                     &b[(height - dh) * b_stride], b_stride, width - dw, dh,
                     &sse, &sum);
    total_sse += sse;
  }

  for (y = 0; y < height / 16; ++y) {
    const uint8_t *pa = a;
    const uint8_t *pb = b;
    for (x = 0; x < width / 16; ++x) {
      vpx_mse16x16(pa, a_stride, pb, b_stride, &sse);
      total_sse += sse;

      pa += 16;
      pb += 16;
    }

    a += 16 * a_stride;
    b += 16 * b_stride;
  }

  return total_sse;
}

#if CONFIG_VP9_HIGHBITDEPTH
static int64_t highbd_get_sse_shift(const uint8_t *a8, int a_stride,
                                    const uint8_t *b8, int b_stride, int width,
                                    int height, unsigned int input_shift) {
  const uint16_t *a = CONVERT_TO_SHORTPTR(a8);
  const uint16_t *b = CONVERT_TO_SHORTPTR(b8);
  int64_t total_sse = 0;
  int x, y;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      int64_t diff;
      diff = (a[x] >> input_shift) - (b[x] >> input_shift);
      total_sse += diff * diff;
    }
    a += a_stride;
    b += b_stride;
  }
  return total_sse;
}

static int64_t highbd_get_sse(const uint8_t *a, int a_stride, const uint8_t *b,
                              int b_stride, int width, int height) {
  int64_t total_sse = 0;
  int x, y;
  const int dw = width % 16;
  const int dh = height % 16;
  unsigned int sse = 0;
  int sum = 0;
  if (dw > 0) {
    encoder_highbd_8_variance(&a[width - dw], a_stride, &b[width - dw],
                              b_stride, dw, height, &sse, &sum);
    total_sse += sse;
  }
  if (dh > 0) {
    encoder_highbd_8_variance(&a[(height - dh) * a_stride], a_stride,
                              &b[(height - dh) * b_stride], b_stride,
                              width - dw, dh, &sse, &sum);
    total_sse += sse;
  }
  for (y = 0; y < height / 16; ++y) {
    const uint8_t *pa = a;
    const uint8_t *pb = b;
    for (x = 0; x < width / 16; ++x) {
      vpx_highbd_8_mse16x16(pa, a_stride, pb, b_stride, &sse);
      total_sse += sse;
      pa += 16;
      pb += 16;
    }
    a += 16 * a_stride;
    b += 16 * b_stride;
  }
  return total_sse;
}
#endif  // CONFIG_VP9_HIGHBITDEPTH

int64_t vpx_get_y_sse(const YV12_BUFFER_CONFIG *a,
                      const YV12_BUFFER_CONFIG *b) {
  assert(a->y_crop_width == b->y_crop_width);
  assert(a->y_crop_height == b->y_crop_height);

  return get_sse(a->y_buffer, a->y_stride, b->y_buffer, b->y_stride,
                 a->y_crop_width, a->y_crop_height);
}

#if CONFIG_VP9_HIGHBITDEPTH
int64_t vpx_highbd_get_y_sse(const YV12_BUFFER_CONFIG *a,
                             const YV12_BUFFER_CONFIG *b) {
  assert(a->y_crop_width == b->y_crop_width);
  assert(a->y_crop_height == b->y_crop_height);
  assert((a->flags & YV12_FLAG_HIGHBITDEPTH) != 0);
  assert((b->flags & YV12_FLAG_HIGHBITDEPTH) != 0);

  return highbd_get_sse(a->y_buffer, a->y_stride, b->y_buffer, b->y_stride,
                        a->y_crop_width, a->y_crop_height);
}
#endif  // CONFIG_VP9_HIGHBITDEPTH

#if CONFIG_VP9_HIGHBITDEPTH
void vpx_calc_highbd_psnr(const YV12_BUFFER_CONFIG *a,
                          const YV12_BUFFER_CONFIG *b, PSNR_STATS *psnr,
                          uint32_t bit_depth, uint32_t in_bit_depth) {
  const int widths[3] = { a->y_crop_width, a->uv_crop_width, a->uv_crop_width };
  const int heights[3] = { a->y_crop_height, a->uv_crop_height,
                           a->uv_crop_height };
  const uint8_t *a_planes[3] = { a->y_buffer, a->u_buffer, a->v_buffer };
  const int a_strides[3] = { a->y_stride, a->uv_stride, a->uv_stride };
  const uint8_t *b_planes[3] = { b->y_buffer, b->u_buffer, b->v_buffer };
  const int b_strides[3] = { b->y_stride, b->uv_stride, b->uv_stride };
  int i;
  uint64_t total_sse = 0;
  uint32_t total_samples = 0;
  const double peak = (double)((1 << in_bit_depth) - 1);
  const unsigned int input_shift = bit_depth - in_bit_depth;

  for (i = 0; i < 3; ++i) {
    const int w = widths[i];
    const int h = heights[i];
    const uint32_t samples = w * h;
    uint64_t sse;
    if (a->flags & YV12_FLAG_HIGHBITDEPTH) {
      if (input_shift) {
        sse = highbd_get_sse_shift(a_planes[i], a_strides[i], b_planes[i],
                                   b_strides[i], w, h, input_shift);
      } else {
        sse = highbd_get_sse(a_planes[i], a_strides[i], b_planes[i],
                             b_strides[i], w, h);
      }
    } else {
      sse = get_sse(a_planes[i], a_strides[i], b_planes[i], b_strides[i], w, h);
    }
    psnr->sse[1 + i] = sse;
    psnr->samples[1 + i] = samples;
    psnr->psnr[1 + i] = vpx_sse_to_psnr(samples, peak, (double)sse);

    total_sse += sse;
    total_samples += samples;
  }

  psnr->sse[0] = total_sse;
  psnr->samples[0] = total_samples;
  psnr->psnr[0] =
      vpx_sse_to_psnr((double)total_samples, peak, (double)total_sse);
}

#endif  // !CONFIG_VP9_HIGHBITDEPTH

void vpx_calc_psnr(const YV12_BUFFER_CONFIG *a, const YV12_BUFFER_CONFIG *b,
                   PSNR_STATS *psnr) {
  static const double peak = 255.0;
  const int widths[3] = { a->y_crop_width, a->uv_crop_width, a->uv_crop_width };
  const int heights[3] = { a->y_crop_height, a->uv_crop_height,
                           a->uv_crop_height };
  const uint8_t *a_planes[3] = { a->y_buffer, a->u_buffer, a->v_buffer };
  const int a_strides[3] = { a->y_stride, a->uv_stride, a->uv_stride };
  const uint8_t *b_planes[3] = { b->y_buffer, b->u_buffer, b->v_buffer };
  const int b_strides[3] = { b->y_stride, b->uv_stride, b->uv_stride };
  int i;
  uint64_t total_sse = 0;
  uint32_t total_samples = 0;

  for (i = 0; i < 3; ++i) {
    const int w = widths[i];
    const int h = heights[i];
    const uint32_t samples = w * h;
    const uint64_t sse =
        get_sse(a_planes[i], a_strides[i], b_planes[i], b_strides[i], w, h);
    psnr->sse[1 + i] = sse;
    psnr->samples[1 + i] = samples;
    psnr->psnr[1 + i] = vpx_sse_to_psnr(samples, peak, (double)sse);

    total_sse += sse;
    total_samples += samples;
  }

  psnr->sse[0] = total_sse;
  psnr->samples[0] = total_samples;
  psnr->psnr[0] =
      vpx_sse_to_psnr((double)total_samples, peak, (double)total_sse);
}

double vp8_mse_2_psnr_tester(double samples, double peak, double mse)
{
	double psnr;

	if (mse > 0.0)
		psnr = 10.0 * log10(peak * peak * samples / mse);
	else
		psnr = MAX_PSNR;      // Limit to prevent / 0

	if (psnr > MAX_PSNR)
		psnr = MAX_PSNR;

	return psnr;
}

enum PotentialArtifact{
	kDontRunArtifactDetection = 0,
	kRunArtifactDetection = 1,
	kNoArtifactFound = 2,
	kPossibleArtifactFound = 3
};

double vp8_calcpsnr_tester(YV12_BUFFER_CONFIG *source, YV12_BUFFER_CONFIG *dest,
	double *ypsnr, double *upsnr, double *vpsnr, double *sq_error, int print_out,
	int possible_artifact)
{
	int i, j, z;
	int diff;
	double frame_psnr;
	double total;
	double grand_total;
	unsigned char *src = source->y_buffer;
	unsigned char *dst = dest->y_buffer;

	double max_psnr_1 = 0;
	double max_psnr_2 = 0;
	double max_psnr_3 = 0;
	double min_psnr = 61;

	int sub_frame_height = 1;
	int sub_frame_width = 1;

	double sub_frame_ypsnr[16][16] = { { 0 } }; // break the frame into 16 by 16
	double sub_frame_total[16][16] = { { 0 } }; // hold 16 by 16 frame total data

	// try to keep at least 64 pixel segments
	int width_segments = source->y_width / 64;
	int height_segments = width_segments;

	if (height_segments > 16)
		height_segments = 16;
	if (width_segments > 16)
		width_segments = 16;

	total = 0.0;
	grand_total = 0.0;

	// Loop throught the Y plane raw and reconstruction data summing
	// (square differences)
	for (i = 0; i < source->y_height; i++)
	{
		for (j = 0; j < source->y_width; j++)
		{
			diff = (int)(src[j]) - (int)(dst[j]);
			total += diff * diff;

			// gather totals for internal segments
			if (possible_artifact == kRunArtifactDetection)
				sub_frame_total[i / ((source->y_height / height_segments == 0)
				? 1 : ((height_segments - 1) + source->y_height) /
				height_segments)][j / ((source->y_width / width_segments == 0)
				? 1 : ((width_segments - 1) + source->y_width) /
				width_segments)] += diff * diff;
		}

		src += source->y_stride;
		dst += dest->y_stride;
	}

	// Work out Y PSNR
	*ypsnr = vp8_mse_2_psnr_tester(source->y_height * source->y_width, 255.0,
		total);

	if (possible_artifact == kRunArtifactDetection)
	{
		// Work out Y PSNRs for internal segments and find min and max
		for (i = 0; i < height_segments; i++){
			for (j = 0; j < width_segments; j++){

				sub_frame_height = 1;
				if (i == (height_segments - 1))
					sub_frame_height = source->y_height - ((height_segments - 1)
					* (((height_segments - 1) + source->y_height)
					/ height_segments));
				else
					sub_frame_height = ((height_segments - 1) + source->y_height
					) / height_segments;

				sub_frame_width = 1;
				if (j == (width_segments - 1))
					sub_frame_width = source->y_width - ((width_segments - 1) *
					(((width_segments - 1) + source->y_width) /
					width_segments));
				else
					sub_frame_width = (15 + source->y_width) / width_segments;

				sub_frame_ypsnr[i][j] = vp8_mse_2_psnr_tester(sub_frame_height *
					sub_frame_width, 255.0, sub_frame_total[i][j]);

				// Get min and top three max sub psnrs
				if (sub_frame_ypsnr[i][j] != 60 && sub_frame_ypsnr[i][j] >
					max_psnr_1){
					max_psnr_2 = max_psnr_1;
					max_psnr_3 = max_psnr_2;
					max_psnr_1 = sub_frame_ypsnr[i][j];
				}
				else if (sub_frame_ypsnr[i][j] != 60 && sub_frame_ypsnr[i][j] >
					max_psnr_2){
					max_psnr_3 = max_psnr_2;
					max_psnr_2 = sub_frame_ypsnr[i][j];
				}
				else if (sub_frame_ypsnr[i][j] != 60 && sub_frame_ypsnr[i][j] >
					max_psnr_3){
					max_psnr_3 = sub_frame_ypsnr[i][j];
				}
				if (sub_frame_ypsnr[i][j] < min_psnr)
					min_psnr = sub_frame_ypsnr[i][j];
			}
		}

		// if min sub psnr is not within ~57% of top three psnr
		// average then flag as potential artifact.
		if ((max_psnr_1 + max_psnr_2 + max_psnr_3) / 7 >= min_psnr) {
			possible_artifact = kPossibleArtifactFound;
		}
		else {
			possible_artifact = kNoArtifactFound;
		}

		if (possible_artifact == kPossibleArtifactFound && print_out)
		{
			printf("min: %.0f Max: %.0f %.0f %.0f", min_psnr,
				max_psnr_1, max_psnr_2, max_psnr_3);

			for (i = 0; i < height_segments; i++){
				printf("\n");
				for (z = 0; z < (width_segments * 3) + 1; z++){
					printf("-");
				}

				printf("\n|");
				for (j = 0; j < width_segments; j++){
					printf("%.0f|", sub_frame_ypsnr[i][j]);
				}
			}
			printf("\n");
			for (z = 0; z < (width_segments * 3) + 1; z++){
				printf("-");
			}
			printf("\n");
		}
	}

	grand_total += total;
	total = 0;

	// Loop through the U plane
	src = source->u_buffer;
	dst = dest->u_buffer;

	for (i = 0; i < source->uv_height; i++)
	{
		for (j = 0; j < source->uv_width; j++)
		{
			diff = (int)(src[j]) - (int)(dst[j]);
			total += diff * diff;
		}

		src += source->uv_stride;
		dst += dest->uv_stride;
	}

	// Work out U PSNR
	*upsnr = vp8_mse_2_psnr_tester(source->uv_height * source->uv_width, 255.0,
		total);
	grand_total += total;
	total = 0;

	// V PSNR
	src = source->v_buffer;
	dst = dest->v_buffer;

	for (i = 0; i < source->uv_height; i++)
	{
		for (j = 0; j < source->uv_width; j++)
		{
			diff = (int)(src[j]) - (int)(dst[j]);
			total += diff * diff;
		}

		src += source->uv_stride;
		dst += dest->uv_stride;
	}

	// Work out UV PSNR
	*vpsnr = vp8_mse_2_psnr_tester(source->uv_height * source->uv_width, 255.0,
		total);
	grand_total += total;
	total = 0;

	// Work out total PSNR
	frame_psnr = vp8_mse_2_psnr_tester(source->y_height * source->y_width *
		3 / 2, 255.0, grand_total);

	*sq_error = 1.0 * grand_total;

	return frame_psnr;
}