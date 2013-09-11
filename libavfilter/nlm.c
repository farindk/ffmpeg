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

#include <math.h>
#include <emmintrin.h>
#include <string.h>
#include <assert.h>
#include "nlm.h"

#include <stdio.h>


static void buildIntegralImage_scalar(uint32_t* integral,   int integral_stride32,
				      const uint8_t* currimage, int currstride,
				      const uint8_t* image, int stride,
				      int  w,int  h,
				      int dx,int dy)
{
  memset(integral-1-integral_stride32, 0, (w+1)*sizeof(uint32_t));

  for (int y=0;y<h;y++) {
    const uint8_t* p1 = currimage+y*currstride;
    const uint8_t* p2 = image+(y+dy)*stride+dx;
    uint32_t* out = integral+y*integral_stride32-1;

    *out++ = 0;

    for (int x=0;x<w;x++)
      {
        int diff = *p1++ - *p2++;
	diff= 0xFF;
        *out++ = *(out-1) + diff * diff;
      }

    if (y>0) {
      out = integral[y];

      for (int x=0;x<w;x++) {
        *out++ += *(out-integral_stride32);
      }
    }
  }
}



/* Input image must be large enough to have valid pixels for the offset (dx,dy).
   I.e., with (dx,dy)=(-10,8), x-value up to -10 and y-values up to (h-1)+8 will be accessed.
   The integral image will be access with (x,y) in [-1,w)x[-1,h).

   Note also that we use 32bit for the integral image even though the values may overflow
   that range. However, the modulo-arithmetic used when computing the block sums later
   will be still correct when the block size is not too large.
 */
static void buildIntegralImage_SSE(uint32_t* integral, int integral_stride32,
                                   const uint8_t* currimage, int currstride,
				   const uint8_t*  image,    int stride,
				   int  w,int  h,
				   int dx,int dy)
{
  memset(integral-1-integral_stride32, 0, (w+1)*sizeof(uint32_t));

  const __m128i zero = _mm_set1_epi8(0);

  for (int y=0;y<h;y++) {
    const uint8_t* p1 = currimage+y*currstride;
    const uint8_t* p2 = image+(y+dy)*stride+dx;
    uint32_t* out = integral+y*integral_stride32-1;

    *out++ = 0;
    __m128i prevadd = _mm_set1_epi32(0);

    const int nPix = 16;

    for (int x=0;x<w;x+=nPix)
      {
	__m128i pa = _mm_loadu_si128((__m128i*)p1);
	__m128i pb = _mm_loadu_si128((__m128i*)p2);

	__m128i pla = _mm_unpacklo_epi8(pa,zero);
	__m128i plb = _mm_unpacklo_epi8(pb,zero);

	__m128i ldiff = _mm_sub_epi16(pla,plb);
	ldiff = _mm_mullo_epi16(ldiff,ldiff);

	__m128i lldiff = _mm_unpacklo_epi16(ldiff,zero);
	__m128i lhdiff = _mm_unpackhi_epi16(ldiff,zero);

	__m128i ltmp = _mm_slli_si128(lldiff, 4);
	lldiff = _mm_add_epi32(lldiff, ltmp);
	ltmp = _mm_slli_si128(lldiff, 8);
	lldiff = _mm_add_epi32(lldiff, ltmp);
	lldiff = _mm_add_epi32(lldiff, prevadd);

	__m128i ladd = _mm_shuffle_epi32(lldiff, 0xff);

	__m128i htmp = _mm_slli_si128(lhdiff, 4);
	lhdiff = _mm_add_epi32(lhdiff, htmp);
	htmp = _mm_slli_si128(lhdiff, 8);
	lhdiff = _mm_add_epi32(lhdiff, htmp);
	lhdiff = _mm_add_epi32(lhdiff, ladd);

	prevadd = _mm_shuffle_epi32(lhdiff, 0xff);

	_mm_store_si128((__m128i*)(out),  lldiff);
	_mm_store_si128((__m128i*)(out+4),lhdiff);



	__m128i pha = _mm_unpackhi_epi8(pa,zero);
	__m128i phb = _mm_unpackhi_epi8(pb,zero);
	__m128i hdiff = _mm_sub_epi16(pha,phb);

	hdiff = _mm_mullo_epi16(hdiff,hdiff);

	__m128i hldiff = _mm_unpacklo_epi16(hdiff,zero);
	__m128i hhdiff = _mm_unpackhi_epi16(hdiff,zero);
	__m128i l2tmp = _mm_slli_si128(hldiff, 4);
	hldiff = _mm_add_epi32(hldiff, l2tmp);
	l2tmp = _mm_slli_si128(hldiff, 8);
	hldiff = _mm_add_epi32(hldiff, l2tmp);
	hldiff = _mm_add_epi32(hldiff, prevadd);
	__m128i hadd = _mm_shuffle_epi32(hldiff, 0xff);
	__m128i h2tmp = _mm_slli_si128(hhdiff, 4);
	hhdiff = _mm_add_epi32(hhdiff, h2tmp);
	h2tmp = _mm_slli_si128(hhdiff, 8);
	hhdiff = _mm_add_epi32(hhdiff, h2tmp);
	hhdiff = _mm_add_epi32(hhdiff, hadd);

	prevadd = _mm_shuffle_epi32(hhdiff, 0xff);

	_mm_store_si128((__m128i*)(out+8), hldiff);
	_mm_store_si128((__m128i*)(out+12),hhdiff);


	out+=nPix;
	p1+=nPix;
	p2+=nPix;
      }

    if (y>0) {
      out = integral+y*integral_stride32;

      for (int x=0;x<w;x+=16) {
	*((__m128i*)out) = _mm_add_epi32(*(__m128i*)(out-integral_stride32),
					 *(__m128i*)(out));

	*((__m128i*)(out+4)) = _mm_add_epi32(*(__m128i*)(out+4-integral_stride32),
					   *(__m128i*)(out+4));

	*((__m128i*)(out+8)) = _mm_add_epi32(*(__m128i*)(out+8-integral_stride32),
					   *(__m128i*)(out+8));
	
	*((__m128i*)(out+12)) = _mm_add_epi32(*(__m128i*)(out+12-integral_stride32),
					    *(__m128i*)(out+12));

	out += 4*4;
      }
    }
  }
}


struct PixelSum
{
  float weightSum;
  float pixelSum;
};


void NLMeans_mono_single(uint8_t* out, int outStride,
			 const uint8_t* in, int origInStride,
			 int w, int h, int border,
			 const DN_NLMeansParams* param)
{
  DN_MonoImage img;
  img.img = in;
  img.stride = origInStride;
  img.w = w;
  img.h = h;
  img.border = border;

  NLMeans_mono_multi(out,outStride,&img,1,param);
}


static int cnt=0;
static int ch;

void NLMeans_mono_multi(uint8_t* out, int outStride,
			const DN_MonoImage*const* images, int nImages,
			const DN_NLMeansParams* param)
{
  int w = images[0]->w;
  int h = images[0]->h;

  int n = (param->patch_size|1);
  int r = (param->range     |1);

  int n2 = (n-1)/2;
  int r2 = (r-1)/2;

  struct PixelSum* tmp_data = (struct PixelSum*)calloc(w*h,sizeof(struct PixelSum));


  float weightFact = 1.0/n/n / (param->h_param * param->h_param);

#define TABSIZE 128

  const int tabSize=TABSIZE;
  float exptab[TABSIZE];

  //const float stretch=10;
  const float stretch = tabSize/ (-log(0.0005));

  for (int i=0;i<tabSize;i++)
    exptab[i] = exp(-i/stretch);
  exptab[tabSize-1]=0;

  float weightFactTab = weightFact*stretch;
  int diff_max = tabSize/weightFactTab;

  int integral_stride32 = w+2*16;
  uint32_t* integral_mem = (uint32_t*)malloc( integral_stride32*(h+1)*sizeof(uint32_t) );
  uint32_t* integral = integral_mem + integral_stride32 + 16;


  for (int imageIdx=0; imageIdx<nImages; imageIdx++)
    {
      // copy input image

      const uint8_t* currentWithBorderP = images[0]->img;
      int currentStride = images[0]->stride;

      const uint8_t* inWithBorderP = images[imageIdx]->img;
      int inStride = images[imageIdx]->stride;

      // ...

      for (int dy=-r2;dy<=r2;dy++)
	for (int dx=-r2;dx<=r2;dx++)
	  {
	    // special case for no shift -> no difference -> weight 1
	    // (but it is not any faster than the full code...)

	    if (dx==0 && dy==0 && imageIdx==0 && 0) {
#pragma omp parallel for
	      for (int y=n2;y<h-n+n2;y++) {
		for (int x=n2;x<w-n+n2;x++) {
		  tmp_data[y*w+x].weightSum += 1;
		  tmp_data[y*w+x].pixelSum  += inWithBorderP[y*inStride+x];
		}
	      }

	      continue;
	    }

	    buildIntegralImage_SSE(integral,integral_stride32,
				   currentWithBorderP, currentStride,
				   inWithBorderP, inStride,
				   w,h,
				   dx,dy);

#pragma omp parallel for
	    for (int y=0;y<=h-n;y++) {
	      const uint32_t* iPtr1 = integral+(y  -1)*integral_stride32-1;
	      const uint32_t* iPtr2 = integral+(y+n-1)*integral_stride32-1;

	      for (int x=0;x<=w-n;x++) {
		const int xc = x+n2;
		const int yc = y+n2;

		// fast exp, see here: http://gruntthepeon.free.fr/ssemath/

		int diff = (uint32_t)(iPtr2[n] - iPtr2[0] - iPtr1[n] + iPtr1[0]);

		//int diffidx = diff*weightFactTab;
		//if (diffidx<tabSize) {

                float weight2;

		if (diff<diff_max) {
		  int diffidx = diff*weightFactTab;

		  //float weight = exp(-diff*weightFact);
		  weight2 = exptab[diffidx];
		  //float weight3 = 1/(1+diff*diff*weightFact*weightFact);
	    
		  tmp_data[yc*w+xc].weightSum += weight2;
		  tmp_data[yc*w+xc].pixelSum  += weight2 * inWithBorderP[(yc+dy)*inStride+xc+dx];
		}

                /*
                else {
                  weight2 = 0;
                }

                if (cnt==14 && xc==296/ch && yc==200/ch) {
                  printf("%d %d %d - %d %f - %d\n",imageIdx, dx,dy, diff, weight2, inWithBorderP[(yc+dy)*inStride+xc+dx]);
                }
                */

		iPtr1++;
		iPtr2++;
	      }
	    }
	  }
    }



  // --- fill output image ---

  // copy border area

  const uint8_t* in = images[0]->img;
  int origInStride  = images[0]->stride;

  for (int y=0;   y<n2;y++) { memcpy(out+y*outStride, in+y*origInStride, w); }
  for (int y=h-n2;y<h ;y++) { memcpy(out+y*outStride, in+y*origInStride, w); }
  for (int y=n2;y<h-n2;y++) {
    memcpy(out+y*outStride,      in+y*origInStride,      n2);
    memcpy(out+y*outStride+w-n2, in+y*origInStride+w-n2, n2);
  }


  // output main image

  for (int y=n2;y<h-n2;y++) {
    for (int x=n2;x<w-n2;x++) {
      *(out+y*outStride+x) = tmp_data[y*w+x].pixelSum / tmp_data[y*w+x].weightSum;
      /*
      if (cnt==14 && x==296/ch && y==200/ch) {
        printf("out: %d (%d)\n", *(out+y*outStride+x), *(in+y*origInStride+x));
      }
      */
    }
  }

  //free(inputWithBorder);
  free(tmp_data);
  free(integral_mem);

  /*
  if (cnt==14) {
    static FILE* fh = NULL;
    if (fh==NULL) fh = fopen("dump.yuv","wb");
    for (int y=0;y<h;y++)
      fwrite(out+y*outStride, w,1, fh);


    static FILE* fh1 = NULL;
    if (fh1==NULL) fh1 = fopen("in.yuv","wb");
    for (int y=0;y<h;y++)
      fwrite(in+y*origInStride, w,1, fh1);
  }
  */
}


void NLMeans_color_auto(uint8_t** out, int* outStride,
			const DN_ColorImage* img, // function takes ownership
			DN_NLMeansContext* ctx)
{
  cnt++;

  assert(ctx->param.n_frames >= 1);
  assert(ctx->param.n_frames <= DN_MAX_NLMeansImages);

  // free oldest image

  DN_free_color_image(&ctx->images[ctx->param.n_frames-1]);

  // shift old images one down and put new image into entry [0]

  for (int i=ctx->param.n_frames-1; i>0; i--) {
    ctx->images[i] = ctx->images[i-1];
    ctx->image_available[i] = ctx->image_available[i-1];
  }

  ctx->images[0] = *img;
  ctx->image_available[0] = 1;


  // process color planes separately

  for (int c=0;c<3;c++)
    if (ctx->images[0].plane[c].img != NULL)
      {
        ch= (c==0 ? 1 : 2);

	const DN_MonoImage* images[DN_MAX_NLMeansImages];
	int i;
	for (i=0; ctx->image_available[i]; i++) {
	  images[i] = &ctx->images[i].plane[c];
	}

	NLMeans_mono_multi(out[c], outStride[c],
			   images, i, &ctx->param);
      }
}



void NLMeans_init_context(DN_NLMeansContext* ctx)
{
  for (int i=0;i<DN_MAX_NLMeansImages;i++) {
    ctx->image_available[i] = 0;
  }


  // TODO: choose computation function based on CPU capabilities

  ctx->buildIntegralImage = buildIntegralImage_SSE;
  // buildIntegralImage_scalar // fallback
}


void NLMeans_free_context(DN_NLMeansContext* ctx)
{
  for (int i=0;i<DN_MAX_NLMeansImages;i++) {
    if (ctx->image_available[i]) {
      DN_free_color_image(&(ctx->images[i]));

      ctx->image_available[i] = 0;
    }
  }
}
