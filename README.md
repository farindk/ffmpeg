
personal FFMPEG fork
====================

This fork contains the following extensions to ffmpeg (each in its own branch):
1. **nlmeans** - Non-Local Means noise reduction filter.  
2. **fieldshift** - Horizontally shift fields of interlaced images against each other (useful e.g. for VHS video recovery).  
3. **libde265** - Integration of libde265 HEVC video decoder.  

Some branches are merged together into the master branch, _but not all of them_.


Documentation
=============

nlmeans
-------
Parameters:
* h - averaging weight decay parameter (larger values give smoother videos)
* range - spatial search range (default=3)
* temporal - number of frames to include into search (default=2)
* patchsize - pixel context region width (default=7, little need to change)

The defaults (h=8, range=3, temporal=2) is a good starting point for the
restoration of very noisy video (old VHS tapes). You may also try 10/5/3
for really noisy inputs or 6/3/1 for good quality inputs.


fieldshift
----------

* offset - pixel offset between successive rows
