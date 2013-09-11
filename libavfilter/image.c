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

#include <string.h>
#include <stdlib.h>
#include "image.h"


void DN_alloc_copy_image(DN_MonoImage* ext_img, const uint8_t* img, int stride, int w,int h, int border)
{
  border = (border+15)/16*16;

  int inStride = (w+2*border);
  int inTotalHeight = (h+2*border);
  uint8_t* inputWithBorder = (uint8_t*)malloc(inStride*inTotalHeight);
  uint8_t* inWithBorderP = inputWithBorder+border+border*inStride;

  for (int y=0;y<h;y++) {
    memcpy(inWithBorderP+y*inStride, img+y*stride, w);
  }

  for (int k=0;k<border;k++) {
    memcpy(inWithBorderP-(k+1)*inStride, img, w);
    memcpy(inWithBorderP+(h+k)*inStride, img+(h-1)*stride, w);
  }

  for (int k=0;k<border;k++) {
    for (int y=-border;y<h+border;y++)
      {
	*(inWithBorderP  -k-1+y*inStride) = inWithBorderP[y*inStride];
	*(inWithBorderP+w+k  +y*inStride) = inWithBorderP[y*inStride+w-1];
      }
  }

  ext_img->img = inWithBorderP;
  ext_img->mem_start = inputWithBorder;
  ext_img->stride = inStride;
  ext_img->w = w;
  ext_img->h = h;
  ext_img->border = border;
}


void DN_free_image(DN_MonoImage* ext_img)
{
  if (ext_img->mem_start) {
    free(ext_img->mem_start);
    ext_img->mem_start=NULL;
    ext_img->img=NULL;
  }
}

void DN_free_color_image(DN_ColorImage* ext_img)
{
  for (int c=0;c<3;c++) {
    DN_free_image(&(ext_img->plane[c]));
  }
}
