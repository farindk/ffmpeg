Personal FFMPEG fork
====================

This fork contains the following extensions to ffmpeg (each in its own branch):  
1. **nlmeans** - Non-Local Means noise reduction filter.  
2. **fieldshift** - Horizontally shift fields of interlaced images against each other (useful e.g. for VHS video recovery).  
3. **libde265** - Integration of libde265 HEVC video decoder.  



Documentation
=============

nlmeans
-------
Parameters:
* h - averaging weight decay parameter (larger values give smoother videos)
* range - spatial search range (default=3), should be odd number.
* temporal - number of frames to include into search (default=2)
* patchsize - pixel context region width (default=7, little need to change), should be odd number.

The defaults (h=8, range=3, temporal=2) is a good starting point for the
restoration of very noisy video (old VHS tapes). You may also try 10/5/3
for really noisy inputs or 6/3/1 for good quality inputs.

The NLMeans filter is under GPL, so be sure to configure with `--enable-gpl`.


fieldshift
----------

* offset - pixel offset between successive rows


libde265
--------

Get and install the latest version of libde265 from here: https://github.com/strukturag/libde265

Configure this ffmpeg fork with at least these options:

    ./configure --enable-libde265 --enable-decoder=libde265 --enable-gpl

The patched ffmpeg can play MKVs containing H.265 streams encoded with
the DIVX HEVC encoder. You can download an example stream from their
webpage:
    http://labs.divx.com/node/127909
