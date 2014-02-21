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
#include "libavutil/intreadwrite.h"


#define DE265_MAX_PTS_QUEUE 256
#define DE265_DEFAULT_NUMBER_OF_THREADS 4

typedef struct DE265DecoderContext {
  de265_decoder_context* decoder;

  int64_t pts_queue[DE265_MAX_PTS_QUEUE];
  int pts_queue_len;
} DE265Context;



static int libde265_decode(AVCodecContext *avctx,
			   void *data, int *got_frame, AVPacket *avpkt)
{
    DE265Context *ctx = avctx->priv_data;
    AVFrame *picture = data;
    const struct de265_image *img;
    de265_error err;
    int ret;

    const uint8_t* src[4];
    int stride[4];

    // insert input packet PTS into sorted queue
    if (ctx->pts_queue_len < DE265_MAX_PTS_QUEUE) {
      int pos=0;
      while (ctx->pts_queue[pos] < avpkt->pts &&
	     pos<ctx->pts_queue_len) {
	pos++;
      }

      if (pos < ctx->pts_queue_len) {
	memmove(&ctx->pts_queue[pos+1], &ctx->pts_queue[pos],
		sizeof(int64_t) * (ctx->pts_queue_len - pos));
      }

      ctx->pts_queue[pos] = avpkt->pts;
      ctx->pts_queue_len++;
    }

    // replace 4-byte length fields with NAL start codes
    uint8_t* avpkt_data = avpkt->data;
    uint8_t* avpkt_end = avpkt->data + avpkt->size;
    while (avpkt_data + 4 < avpkt_end) {
      int nal_size = AV_RB32(avpkt_data);
      AV_WB32(avpkt_data, 0x00000001);
      avpkt_data += 4 + nal_size;
    }

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


	// assign next PTS from queue
	if (ctx->pts_queue_len>0) {
	  picture->pkt_pts = ctx->pts_queue[0];

	  if (ctx->pts_queue_len>1) {
	    memmove(&ctx->pts_queue[0], &ctx->pts_queue[1],
		    sizeof(int64_t) * (ctx->pts_queue_len-1));
	  }

	  ctx->pts_queue_len--;
	}
    }

    return avpkt->size;
}


static av_cold int libde265_free(AVCodecContext *avctx)
{
    DE265Context *ctx = avctx->priv_data;
    de265_free_decoder(ctx->decoder);
    return 0;
}


#if CONFIG_LIBDE265_DECODER
static av_cold int libde265_ctx_init(AVCodecContext *avctx)
{
    DE265Context *ctx = avctx->priv_data;
    ctx->decoder = de265_new_decoder();
    de265_error err = de265_start_worker_threads(ctx->decoder, DE265_DEFAULT_NUMBER_OF_THREADS);

    avctx->pix_fmt = AV_PIX_FMT_YUV420P;
    return 0;
}


AVCodec ff_libde265_decoder = {
    .name           = "libde265",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_H265,
    .priv_data_size = sizeof(DE265Context),
    .init           = libde265_ctx_init,
    .close          = libde265_free,
    .decode         = libde265_decode,
    .capabilities   = CODEC_CAP_AUTO_THREADS | CODEC_CAP_DR1,
    .long_name      = NULL_IF_CONFIG_SMALL("libde265 H.265/HEVC decoder"),
};
#endif /* CONFIG_LIBDE265_DECODER */
