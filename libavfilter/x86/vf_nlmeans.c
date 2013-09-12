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


#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavfilter/vf_nlmeans.h"

#include <emmintrin.h>
#include <string.h>


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
    const __m128i zero = _mm_set1_epi8(0);


    memset(integral-1-integral_stride32, 0, (w+1)*sizeof(uint32_t));

    for (int y=0;y<h;y++) {
        const uint8_t* p1 = currimage+y*currstride;
        const uint8_t* p2 = image+(y+dy)*stride+dx;

        uint32_t* out = integral+y*integral_stride32-1;

        __m128i prevadd = _mm_set1_epi32(0);
        const int nPix = 16;

        *out++ = 0;

        for (int x=0;x<w;x+=nPix)
        {
            __m128i pa, pb;
            __m128i pla, plb;
            __m128i ldiff, lldiff, lhdiff;
            __m128i ltmp,htmp;
            __m128i ladd,hadd;
            __m128i pha,phb;
            __m128i hdiff,hldiff,hhdiff;
            __m128i l2tmp,h2tmp;



            pa = _mm_loadu_si128((__m128i*)p1);
            pb = _mm_loadu_si128((__m128i*)p2);

            pla = _mm_unpacklo_epi8(pa,zero);
            plb = _mm_unpacklo_epi8(pb,zero);

            ldiff = _mm_sub_epi16(pla,plb);
            ldiff = _mm_mullo_epi16(ldiff,ldiff);

            lldiff = _mm_unpacklo_epi16(ldiff,zero);
            lhdiff = _mm_unpackhi_epi16(ldiff,zero);

            ltmp = _mm_slli_si128(lldiff, 4);
            lldiff = _mm_add_epi32(lldiff, ltmp);
            ltmp = _mm_slli_si128(lldiff, 8);
            lldiff = _mm_add_epi32(lldiff, ltmp);
            lldiff = _mm_add_epi32(lldiff, prevadd);

            ladd = _mm_shuffle_epi32(lldiff, 0xff);

            htmp = _mm_slli_si128(lhdiff, 4);
            lhdiff = _mm_add_epi32(lhdiff, htmp);
            htmp = _mm_slli_si128(lhdiff, 8);
            lhdiff = _mm_add_epi32(lhdiff, htmp);
            lhdiff = _mm_add_epi32(lhdiff, ladd);

            prevadd = _mm_shuffle_epi32(lhdiff, 0xff);

            _mm_store_si128((__m128i*)(out),  lldiff);
            _mm_store_si128((__m128i*)(out+4),lhdiff);



            pha = _mm_unpackhi_epi8(pa,zero);
            phb = _mm_unpackhi_epi8(pb,zero);
            hdiff = _mm_sub_epi16(pha,phb);

            hdiff = _mm_mullo_epi16(hdiff,hdiff);

            hldiff = _mm_unpacklo_epi16(hdiff,zero);
            hhdiff = _mm_unpackhi_epi16(hdiff,zero);
            l2tmp = _mm_slli_si128(hldiff, 4);
            hldiff = _mm_add_epi32(hldiff, l2tmp);
            l2tmp = _mm_slli_si128(hldiff, 8);
            hldiff = _mm_add_epi32(hldiff, l2tmp);
            hldiff = _mm_add_epi32(hldiff, prevadd);
            hadd = _mm_shuffle_epi32(hldiff, 0xff);
            h2tmp = _mm_slli_si128(hhdiff, 4);
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


av_cold void ff_nlmeans_init_x86(NLMeansFunctions* func)
{
  int cpu_flags = av_get_cpu_flags();

  if (EXTERNAL_SSE2(cpu_flags)) {
    func->buildIntegralImage = buildIntegralImage_SSE;
  }
}
