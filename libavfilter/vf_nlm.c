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
 * See http://www.ipol.im/pub/art/2011/bcm_nlm/ for a description.
 */

#include <float.h>

#include "config.h"
#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/pixdesc.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"
#include "vf_nlm.h"
#include "nlm.h"


static av_cold int init(AVFilterContext *ctx)
{
    NLMContext *nlm = ctx->priv;

    NLMeans_init_context(&nlm->context);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NLMContext *nlm = ctx->priv;

    NLMeans_free_context(&nlm->context);
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

    DN_ColorImage borderedImg;

    for (c = 0; c < 3; c++) {
      int w = FF_CEIL_RSHIFT(in->width,  (!!c * nlm->hsub));
      int h = FF_CEIL_RSHIFT(in->height, (!!c * nlm->vsub));
      int border = nlm->context.param.range/2;

      DN_alloc_copy_image(&borderedImg.plane[c], in->data[c], in->linesize[c],
			  w,h,border);
    }

    NLMeans_color_auto(out->data, out->linesize,
		       &borderedImg,
		       &nlm->context);


    if (!direct)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

#define OFFSET(x) offsetof(NLMContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
static const AVOption options[] = {
  { "h",           "averaging weight decay parameter",  OFFSET(context.param.h_param), AV_OPT_TYPE_DOUBLE, { .dbl = 10.0 }, 0.1, 100.0, FLAGS },
  { "patchsize",   "patch width/height",        OFFSET(context.param.patch_size), AV_OPT_TYPE_INT,    { .i64 =  7   },   3, 255,   FLAGS },
  { "range",       "search range",            OFFSET(context.param.range), AV_OPT_TYPE_INT,    { .i64 = 21   },   3, 255,   FLAGS },
  { "temporal",    "temporal search range",            OFFSET(context.param.n_frames), AV_OPT_TYPE_INT,    { .i64 = 1   },   1, DN_MAX_NLMeansImages,   FLAGS },
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

AVFilter avfilter_vf_nlmeans = {
    .name          = "NLMeans",
    .description   = NULL_IF_CONFIG_SMALL("Apply a Non-Local Means filter."),

    .priv_size     = sizeof(NLMContext),
    .priv_class    = &nlm_class,
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,

    .inputs    = avfilter_vf_nlm_inputs,
    .outputs   = avfilter_vf_nlm_outputs,
};
