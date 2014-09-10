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

/**
 * @file
 * Non-Local Means noise reduction filter.
 *
 * This implementation is an implementation of the paper below with
 * an extension to use several frames. The computation is slightly
 * simplified to reduce computation complexity.
 *
 * A. Buades, B. Coll, J.-M. Morel:
 * "A non-local algorithm for image denoising",
 * IEEE Computer Vision and Pattern Recognition 2005, Vol 2, pp: 60-65, 2005.
 */

#include <float.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <assert.h>

#include <stdio.h>

#include "config.h"
#include "libavutil/pixdesc.h"

#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"
#include "libavutil/opt.h"

#include "libavfilter/vf_nlmeans.h"



typedef struct
{
    uint8_t* img; // point to logical origin (0;0)
    int stride;

    int w,h;
    int border; // extra border on all four sides

    uint8_t* mem_start; // start of allocated memory
} MonoImage;


typedef enum {
    ImageFormat_Mono,
    ImageFormat_YUV420,
    ImageFormat_YUV422,
    ImageFormat_YUV444,
    ImageFormat_RGB
} ImageFormat;


typedef struct
{
    MonoImage   plane[3];
    ImageFormat format;
} ColorImage;


static void alloc_and_copy_image_with_border(MonoImage* out_img,
                                             const uint8_t* input_image, int input_stride,
                                             int w,int h, int req_border)
{
    const int border = (req_border+15)/16*16;
    const int out_stride = (w+2*border);
    const int out_total_height = (h+2*border);

    uint8_t* const memory_ptr   = (uint8_t*)malloc(out_stride*out_total_height);
    uint8_t* const output_image = memory_ptr + border + border*out_stride; // logical output image origin (0,0)


    // copy main image content

    for (int y=0;y<h;y++) {
        memcpy(output_image + y*out_stride, input_image+y*input_stride, w);
    }

    // top/bottom borders

    for (int k=0;k<border;k++) {
        memcpy(output_image-(k+1)*out_stride, input_image, w);
        memcpy(output_image+(h+k)*out_stride, input_image+(h-1)*input_stride, w);
    }

    // left/right borders

    for (int k=0;k<border;k++) {
        for (int y=-border;y<h+border;y++)
        {
            *(output_image  -k-1+y*out_stride) = output_image[y*out_stride];
            *(output_image+w+k  +y*out_stride) = output_image[y*out_stride+w-1];
        }
    }

    out_img->img = output_image;
    out_img->mem_start = memory_ptr;
    out_img->stride = out_stride;
    out_img->w = w;
    out_img->h = h;
    out_img->border = border;
}


static void free_mono_image(MonoImage* img)
{
    if (img->mem_start) {
        free(img->mem_start);
        img->mem_start=NULL;
        img->img=NULL;
    }
}

static void free_color_image(ColorImage* img)
{
    for (int c=0;c<3;c++) {
        free_mono_image(&(img->plane[c]));
    }
}





typedef struct
{
    double h_param;
    int    patch_size;
    int    range;      // search range (must be odd number)
    int    n_frames;   // temporal search depth
} NLMeansParams;


#define MAX_NLMeansImages 32

typedef struct {
    const AVClass *class;

    int hsub,vsub;

    NLMeansParams param;

    ColorImage images[MAX_NLMeansImages];
    int        image_available[MAX_NLMeansImages];

    NLMeansFunctions func;

} NLMContext;



static void buildIntegralImage_scalar(uint32_t* integral_image,   int integral_stride,
				      const uint8_t* current_image, int current_image_stride,
				      const uint8_t* compare_image, int compare_image_stride,
				      int  w,int  h,
				      int dx,int dy)
{
    memset(integral_image -1 -integral_stride, 0, (w+1)*sizeof(uint32_t));

    for (int y=0;y<h;y++) {
        const uint8_t* p1 = current_image +  y    *current_image_stride;
        const uint8_t* p2 = compare_image + (y+dy)*compare_image_stride + dx;
        uint32_t* out = integral_image + y*integral_stride -1;

        *out++ = 0;

        for (int x=0;x<w;x++)
        {
            int diff = *p1++ - *p2++;
            *out = *(out-1) + diff * diff;
            out++;
        }

        if (y>0) {
            out = integral_image + y*integral_stride;

            for (int x=0;x<w;x++) {
                *out += *(out - integral_stride);
                out++;
            }
        }
    }
}



struct PixelSum
{
    float weight_sum;
    float pixel_sum;
};



static void NLMeans_mono_multi(uint8_t* out, int out_stride,
			       const MonoImage*const* images, int n_images,
			       const NLMContext* ctx)
{
    const int w = images[0]->w;
    const int h = images[0]->h;

    const int n = (ctx->param.patch_size|1);
    const int r = (ctx->param.range     |1);

    const int n_half = (n-1)/2;
    const int r_half = (r-1)/2;


    // alloc memory for temporary pixel sums

    struct PixelSum* const tmp_data = (struct PixelSum*)calloc(w*h,sizeof(struct PixelSum));


    // allocate integral image

    const int integral_stride = w+2*16;
    uint32_t* const integral_mem = (uint32_t*)malloc( integral_stride*(h+1)*sizeof(uint32_t) );
    uint32_t* const integral = integral_mem + integral_stride + 16;


    // precompute exponential table

    const float weight_factor = 1.0/n/n / (ctx->param.h_param * ctx->param.h_param);

#define EXP_TABSIZE 128

    const int table_size=EXP_TABSIZE;
    const float min_weight_in_table = 0.0005;

    float exptable[EXP_TABSIZE];

    const float stretch = table_size/ (-log(min_weight_in_table));
    const float weight_fact_table = weight_factor*stretch;
    const int diff_max = table_size/weight_fact_table;

    for (int i=0;i<table_size;i++) {
        exptable[i] = exp(-i/stretch);
    }
    exptable[table_size-1]=0;



    for (int image_idx=0; image_idx<n_images; image_idx++)
    {
        // copy input image

        const uint8_t* current_image = images[0]->img;
        int current_image_stride = images[0]->stride;

        const uint8_t* compare_image = images[image_idx]->img;
        int compare_image_stride = images[image_idx]->stride;


        // --- iterate through all displacements ---

        for (int dy=-r_half ; dy<=r_half ; dy++)
            for (int dx=-r_half ; dx<=r_half ; dx++)
            {
                // special, simple implementation for no shift (no difference -> weight 1)

                if (dx==0 && dy==0 && image_idx==0) {
#pragma omp parallel for
                    for (int y=n_half;y<h-n+n_half;y++) {
                        for (int x=n_half;x<w-n+n_half;x++) {
                            tmp_data[y*w+x].weight_sum += 1;
                            tmp_data[y*w+x].pixel_sum  += current_image[y*current_image_stride+x];
                        }
                    }

                    continue;
                }


                // --- regular case ---

                ctx->func.buildIntegralImage(integral,integral_stride,
                                             current_image, current_image_stride,
                                             compare_image, compare_image_stride,
                                             w,h, dx,dy);

#pragma omp parallel for
                for (int y=0;y<=h-n;y++) {
                    const uint32_t* integral_ptr1 = integral+(y  -1)*integral_stride-1;
                    const uint32_t* integral_ptr2 = integral+(y+n-1)*integral_stride-1;

                    for (int x=0;x<=w-n;x++) {
                        const int xc = x+n_half;
                        const int yc = y+n_half;

                        // patch difference

                        int diff = (uint32_t)(integral_ptr2[n] - integral_ptr2[0] - integral_ptr1[n] + integral_ptr1[0]);


                        // sum pixel with weight

                        if (diff<diff_max) {
                            int diffidx = diff*weight_fact_table;

                            //float weight = exp(-diff*weightFact);
                            float weight = exptable[diffidx];

                            tmp_data[yc*w+xc].weight_sum += weight;
                            tmp_data[yc*w+xc].pixel_sum  += weight * compare_image[(yc+dy)*compare_image_stride+xc+dx];
                        }

                        integral_ptr1++;
                        integral_ptr2++;
                    }
                }
            }
    }



    // --- fill output image ---

    // copy border area

    {
        const uint8_t* in  = images[0]->img;
        int orig_in_stride = images[0]->stride;

        for (int y=0;       y<n_half  ;y++) { memcpy(out+y*out_stride, in+y*orig_in_stride, w); }
        for (int y=h-n_half;y<h       ;y++) { memcpy(out+y*out_stride, in+y*orig_in_stride, w); }
        for (int y=n_half  ;y<h-n_half;y++) {
            memcpy(out+y*out_stride,          in+y*orig_in_stride,          n_half);
            memcpy(out+y*out_stride+w-n_half, in+y*orig_in_stride+w-n_half, n_half);
        }
    }

    // output main image

    for (int y=n_half;y<h-n_half;y++) {
        for (int x=n_half;x<w-n_half;x++) {
            *(out+y*out_stride+x) = tmp_data[y*w+x].pixel_sum / tmp_data[y*w+x].weight_sum;
        }
    }

    free(tmp_data);
    free(integral_mem);
}


static void NLMeans_color_auto(uint8_t** out, int* out_stride,
			       const ColorImage* img, // function takes ownership
			       NLMContext* ctx)
{
    assert(ctx->param.n_frames >= 1);
    assert(ctx->param.n_frames <= MAX_NLMeansImages);

    // free oldest image

    free_color_image(&ctx->images[ctx->param.n_frames-1]);

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
            const MonoImage* images[MAX_NLMeansImages];
            int i;
            for (i=0; ctx->image_available[i]; i++) {
                images[i] = &ctx->images[i].plane[c];
            }

            NLMeans_mono_multi(out[c], out_stride[c],
                               images, i, ctx);
        }
}



static av_cold int init(AVFilterContext *ctx)
{
    NLMContext *nlm = ctx->priv;

    for (int i=0;i<MAX_NLMeansImages;i++) {
        nlm->image_available[i] = 0;
    }


    nlm->func.buildIntegralImage = buildIntegralImage_scalar;

    if (ARCH_X86) {
        ff_nlmeans_init_x86(&nlm->func);
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NLMContext *nlm = ctx->priv;

    for (int i=0;i<MAX_NLMeansImages;i++) {
        if (nlm->image_available[i]) {
            free_color_image(&(nlm->images[i]));

            nlm->image_available[i] = 0;
        }
    }
}

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUV410P,
        AV_PIX_FMT_YUV411P,
        AV_PIX_FMT_YUV440P,

        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUVJ422P,
        AV_PIX_FMT_YUVJ444P,
        AV_PIX_FMT_YUVJ440P,

        AV_PIX_FMT_NONE
    };

    ff_set_common_formats(ctx, ff_make_format_list(pix_fmts));

    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    NLMContext *nlm = inlink->dst->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);

    nlm->hsub  = desc->log2_chroma_w;
    nlm->vsub  = desc->log2_chroma_h;

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    NLMContext *nlm = inlink->dst->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];

    ColorImage bordered_image;

    AVFrame *out;
    int direct, c;

    if (av_frame_is_writable(in)) {
        direct = 1;
        out = in;
    } else {
        direct = 0;
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        av_frame_copy_props(out, in);
    }


    // extend image with border

    for (c = 0; c < 3; c++) {
        int w = FF_CEIL_RSHIFT(in->width,  (!!c * nlm->hsub));
        int h = FF_CEIL_RSHIFT(in->height, (!!c * nlm->vsub));
        int border = nlm->param.range/2;

        alloc_and_copy_image_with_border(&bordered_image.plane[c],
                                         in->data[c], in->linesize[c],
                                         w,h,border);
    }

    NLMeans_color_auto(out->data, out->linesize,
		       &bordered_image,
		       nlm);


    if (!direct)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

#define OFFSET(x) offsetof(NLMContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
static const AVOption options[] = {
    { "h",           "averaging weight decay parameter", OFFSET(param.h_param),    AV_OPT_TYPE_DOUBLE, { .dbl = 8.0 }, 0.1, 100.0, FLAGS },
    { "patchsize",   "patch width/height",               OFFSET(param.patch_size), AV_OPT_TYPE_INT,    { .i64 = 7   },   3, 255,   FLAGS },
    { "range",       "search range",                     OFFSET(param.range),      AV_OPT_TYPE_INT,    { .i64 = 3   },   1, 255,   FLAGS },
    { "temporal",    "temporal search range",            OFFSET(param.n_frames),   AV_OPT_TYPE_INT,    { .i64 = 2   },   1, MAX_NLMeansImages,   FLAGS },
    { NULL },
};


static const AVClass nlm_class = {
    .class_name = "nlm",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};


static const AVFilterPad avfilter_vf_nlm_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
    { NULL }
};


static const AVFilterPad avfilter_vf_nlm_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO
    },
    { NULL }
};

AVFilter ff_vf_nlmeans = {
    .name          = "nlmeans",
    .description   = NULL_IF_CONFIG_SMALL("Apply a Non-Local Means filter."),

    .priv_size     = sizeof(NLMContext),
    .priv_class    = &nlm_class,
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,

    .inputs    = avfilter_vf_nlm_inputs,
    .outputs   = avfilter_vf_nlm_outputs,
};
