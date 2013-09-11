/*
 * Copyright (c) 2013 Dirk Farin <dirk.farin@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef NLM_H
#define NLM_H

#include <stdint.h>
#include "image.h"


typedef struct
{
  int    patch_size;
  int    range;
  double h_param;
  int    n_frames;
} DN_NLMeansParams;


#define DN_MAX_NLMeansImages 32

typedef struct
{
  DN_NLMeansParams param;

  DN_ColorImage images[DN_MAX_NLMeansImages];
  int           image_available[DN_MAX_NLMeansImages];

  void (*buildIntegralImage)(uint32_t* integral,   int integral_stride32,
			     const uint8_t* image, int stride,
			     int  w,int  h,
			     int dx,int dy);
} DN_NLMeansContext;


void NLMeans_mono_multi(uint8_t* out, int outStride,
			const DN_MonoImage*const * images, int nImages,
			const DN_NLMeansParams* param);

void NLMeans_mono_single(uint8_t* out, int outStride,
			 const uint8_t* in, int inStride,
			 int w, int h, int border,
			 const DN_NLMeansParams* param);

void NLMeans_color_auto(uint8_t** out, int* outStride,
			const DN_ColorImage* img, // function takes ownership
			DN_NLMeansContext* ctx);

void NLMeans_init_context(DN_NLMeansContext* ctx);
void NLMeans_free_context(DN_NLMeansContext* ctx);

#endif
