/*
 * H.265 decoder
 *
 * Copyright (c) 2013, Dirk Farin <dirk.farin@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * H.265 decoder based on libde265
 */

#include <libde265/de265.h>

#include "libavutil/common.h"
#include "libavutil/imgutils.h"
#include "avcodec.h"
#include "internal.h"


typedef struct DE265DecoderContext {
  de265_decoder_context* decoder;
} DE265Context;



static int de265_decode(AVCodecContext *avctx,
                        void *data, int *got_frame, AVPacket *avpkt)
{
    DE265Context *ctx = avctx->priv_data;
    AVFrame *picture = data;
    const struct de265_image *img;
    de265_error err;
    int ret;

    const uint8_t* src[4];
    int stride[4];


    err = de265_decode_data(ctx->decoder, avpkt->data, avpkt->size);
    if (err != DE265_OK) {
        const char *error  = de265_get_error_text(err);

        av_log(avctx, AV_LOG_ERROR, "Failed to decode frame: %s\n", error);
        return AVERROR_INVALIDDATA;
    }

    if (img = de265_get_next_picture(ctx->decoder)) {
      int width  = de265_get_image_width(img,0);
      int height = de265_get_image_height(img,0);
      if (width != avctx->width || height != avctx->height) {
            if (avctx->width != 0)
                av_log(avctx, AV_LOG_INFO, "dimension change! %dx%d -> %dx%d\n",
                       avctx->width, avctx->height, width, height);

            if (av_image_check_size(width, height, 0, avctx))
                return AVERROR_INVALIDDATA;

            avcodec_set_dimensions(avctx, width, height);
        }
        if ((ret = ff_get_buffer(avctx, picture, 0)) < 0)
            return ret;

        for (int i=0;i<4;i++) {
            src[i] = de265_get_image_plane(img,i, &stride[i]);
        }

        av_image_copy(picture->data, picture->linesize, src, stride,
                      avctx->pix_fmt, width, height);
        *got_frame = 1;
    }
    return avpkt->size;
}


static av_cold int de265_free(AVCodecContext *avctx)
{
    DE265Context *ctx = avctx->priv_data;
    de265_free_decoder(ctx->decoder);
    return 0;
}


static av_cold void de265_static_init(struct AVCodec *codec)
{
    de265_init();
}


#if CONFIG_LIBDE265_DECODER
static av_cold int de265_ctx_init(AVCodecContext *avctx)
{
    DE265Context *ctx = avctx->priv_data;
    ctx->decoder = de265_new_decoder();

    avctx->pix_fmt = AV_PIX_FMT_YUV420P;
    return 0;
}


AVCodec ff_libde265_decoder = {
    .name           = "libde265",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_H265,
    .priv_data_size = sizeof(DE265Context),
    .init_static_data = de265_static_init,
    .init           = de265_ctx_init,
    .close          = de265_free,
    .decode         = de265_decode,
    .capabilities   = CODEC_CAP_AUTO_THREADS | CODEC_CAP_DR1,
    .long_name      = NULL_IF_CONFIG_SMALL("libde265 H.265/HEVC decoder"),
};
#endif /* CONFIG_LIBDE265_DECODER */
