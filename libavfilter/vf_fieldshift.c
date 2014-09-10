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
 * Horizontally shift fields against each other.
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
#include "libavutil/intreadwrite.h"
#include "libavutil/imgutils.h"



typedef struct {
    const AVClass *class;

    int max_step[4];    ///< max pixel step for each plane, expressed as a number of bytes
    int hsub,vsub;

    int shift_amount;
} FieldShiftContext;



static av_cold int init(AVFilterContext *ctx)
{
    //FieldShiftContext *fieldshift = ctx->priv;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    //FieldShiftContext *fieldshift = ctx->priv;
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *pix_fmts = NULL;
    int fmt;

    for (fmt = 0; fmt < AV_PIX_FMT_NB; fmt++) {
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(fmt);
        if (!(desc->flags & AV_PIX_FMT_FLAG_HWACCEL ||
              desc->flags & AV_PIX_FMT_FLAG_BITSTREAM ||
              (desc->log2_chroma_w != desc->log2_chroma_h &&
               desc->comp[0].plane == desc->comp[1].plane)))
            ff_add_format(&pix_fmts, fmt);
    }

    ff_set_common_formats(ctx, pix_fmts);
    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    FieldShiftContext *fieldshift = inlink->dst->priv;
    const AVPixFmtDescriptor *pix_desc = av_pix_fmt_desc_get(inlink->format);

    av_image_fill_max_pixsteps(fieldshift->max_step, NULL, pix_desc);
    fieldshift->hsub  = pix_desc->log2_chroma_w;
    fieldshift->vsub  = pix_desc->log2_chroma_h;

    //printf("hsub %d %d\n",fieldshift->hsub,fieldshift->vsub);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    FieldShiftContext *fieldshift = inlink->dst->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];

    AVFrame *out;
    uint8_t *inrow, *outrow;

    int i, j, plane, step;


    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    /* copy palette if required */
    if (av_pix_fmt_desc_get(inlink->format)->flags & AV_PIX_FMT_FLAG_PAL)
        memcpy(out->data[1], in->data[1], AVPALETTE_SIZE);


    for (plane = 0; plane < 4 && in->data[plane] && in->linesize[plane]; plane++) {
        const int width  = (plane == 1 || plane == 2) ? FF_CEIL_RSHIFT(inlink->w, fieldshift->hsub) : inlink->w;
        const int height = (plane == 1 || plane == 2) ? FF_CEIL_RSHIFT(inlink->h, fieldshift->vsub) : inlink->h;

        int plane_shift;
        int shift_right;
        int shift_left;

        if ((plane==1 || plane==2) && fieldshift->vsub>0) {
            plane_shift=0;
        }
        else {
            plane_shift = fieldshift->shift_amount / fieldshift->hsub;
        }
        shift_right = plane_shift/2;
        shift_left  = plane_shift - shift_right;

        step = fieldshift->max_step[plane];

        //printf("step %d \n",step);

        outrow = out->data[plane];
        inrow  = in ->data[plane];
        for (i = 0; i < height; i++) {
            int shift = (i&1) ? shift_left : -shift_right;

            switch (step) {
            case 1:
                if (shift>=0) {
                    for (j = 0; j < shift; j++)
                        outrow[j] = inrow[0];
                    for (j = shift; j < width; j++)
                        outrow[j] = inrow[j-shift];
                }
                else {
                    for (j = 0; j < width+shift; j++)
                        outrow[j] = inrow[j-shift];
                    for (j = width+shift; j < width; j++)
                        outrow[j] = inrow[width-1];
                }
                break;

            case 2:
            {
                uint16_t *outrow16 = (uint16_t *)outrow;
                uint16_t * inrow16 = (uint16_t *) inrow;

                if (shift>=0) {
                    for (j = 0; j < shift; j++)
                        outrow16[j] = inrow16[0];
                    for (j = shift; j < width; j++)
                        outrow16[j] = inrow16[j-shift];
                }
                else {
                    for (j = 0; j < width+shift; j++)
                        outrow16[j] = inrow16[j-shift];
                    for (j = width+shift; j < width; j++)
                        outrow16[j] = inrow16[width-1];
                }
            }
            break;

            case 3:
            {
                uint8_t *in  =  inrow;
                uint8_t *out = outrow;

                if (shift>=0) {
                    int32_t v = AV_RB24(in);

                    for (j = 0; j < shift; j++, out+=3) {
                        AV_WB24(out, v);
                    }
                    for (j = shift; j < width; j++, out+=3, in+=3) {
                        v = AV_RB24(in);
                        AV_WB24(out, v);
                    }
                }
                else {
                    int32_t v = 0;

                    in -= shift*3;

                    for (j = 0; j < width+shift; j++, out+=3, in+=3)
                    {
                        v = AV_RB24(in);
                        AV_WB24(out, v);
                        out+=3;
                    }

                    v = AV_RB24(inrow+3*(width-1));

                    for (j = width+shift; j < width; j++, out+=3) {
                        AV_WB24(out, v);
                    }
                }
            }
            break;

            case 4:
            {
                uint32_t *outrow32 = (uint32_t *)outrow;
                uint32_t * inrow32 = (uint32_t *) inrow;

                if (shift>=0) {
                    for (j = 0; j < shift; j++)
                        outrow32[j] = inrow32[0];
                    for (j = shift; j < width; j++)
                        outrow32[j] = inrow32[j-shift];
                }
                else {
                    for (j = 0; j < width+shift; j++)
                        outrow32[j] = inrow32[j-shift];
                    for (j = width+shift; j < width; j++)
                        outrow32[j] = inrow32[width-1];
                }
            }
            break;

            default:
                if (shift>=0) {
                    for (j = 0; j < shift; j++)
                        memcpy(outrow + j*step, inrow, step);
                    for (j = shift; j < width; j++)
                        memcpy(outrow + j*step, inrow + (j-shift)*step, step);
                }
                else {
                    for (j = 0; j < width+shift; j++)
                        memcpy(outrow + j*step, inrow + (j-shift)*step, step);

                    for (j = width+shift; j < width; j++)
                        memcpy(outrow + j*step, inrow + (width-1)*step, step);
                }
                break;
            }

            inrow  += in ->linesize[plane];
            outrow += out->linesize[plane];
        }
    }


    av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

#define OFFSET(x) offsetof(FieldShiftContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
static const AVOption options[] = {
    { "offset", "shift offset", OFFSET(shift_amount),  AV_OPT_TYPE_INT, { .i64 = 2.0 }, -100, 100, FLAGS },
    { NULL },
};


static const AVClass fieldshift_class = {
    .class_name = "fieldshift",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};


static const AVFilterPad avfilter_vf_fieldshift_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
    { NULL }
};


static const AVFilterPad avfilter_vf_fieldshift_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO
    },
    { NULL }
};

AVFilter ff_vf_fieldshift = {
    .name          = "fieldshift",
    .description   = NULL_IF_CONFIG_SMALL("Shift fields against each other"),

    .priv_size     = sizeof(FieldShiftContext),
    .priv_class    = &fieldshift_class,
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,

    .inputs    = avfilter_vf_fieldshift_inputs,
    .outputs   = avfilter_vf_fieldshift_outputs,
};
