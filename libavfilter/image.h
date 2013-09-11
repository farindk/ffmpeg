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

#ifndef DN_IMAGE_H
#define DN_IMAGE_H

#include <stdint.h>


typedef struct
{
  uint8_t* img;
  int stride;

  int w,h;
  int border;

  uint8_t* mem_start;
} DN_MonoImage;


typedef enum {
  DN_ImageFormat_Mono,
  DN_ImageFormat_YUV420,
  DN_ImageFormat_YUV422,
  DN_ImageFormat_YUV444,
  DN_ImageFormat_RGB
} DN_ImageFormat;


typedef struct
{
  DN_MonoImage   plane[3];
  DN_ImageFormat format;
} DN_ColorImage;


void DN_alloc_copy_image(DN_MonoImage* out_img,
			 const uint8_t* in_img, int in_stride,
			 int w,int h, int new_border);

void DN_free_image(DN_MonoImage* ext_img);
void DN_free_color_image(DN_ColorImage* ext_img);

#endif
