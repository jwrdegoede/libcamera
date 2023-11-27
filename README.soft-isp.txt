The soft ISP only works with simple pipeline handler.
To build libcamera with soft ISP / soft IPA enabled use:
$ meson setup -Dprefix=$INSTALL_DIR -Dpipelines=simple/linaro -Dipas=simple/linaro $BUILD_DIR
$ ninja -C $BUILD_DIR
$ ninja -C $BUILD_DIR install

Notes on the SoftwareISP-v01 branch:
- it does work on RB5 board, but there are still a lot of things to implement
  or implement properly (soft ISP factory, dma_bufs vs memfd memory for the
  FrameBuffers, the mutex for the shared memory, etc etc)
- this is a WIP kind of a branch, so the commits structure is pretty crappy
- the soft ISP is no longer based on the Converter class
- either the Converter, or the soft ISP instance can be created. Not the both
- if the Converter isn't used for the particular platform (== video capture
  driver) the "linaro" implementation of soft ISP is instantiated
  unconditionally. Otherwise it should be possible to use:
    $ meson setup -Dprefix=$INSTALL_DIR -Dpipelines=simple $BUILD_DIR
  to get a build with simple pipeline handler and without the soft ISP / soft
  IPA
- debayering indirection is supported. What debayering function
  to call is decided based on the camera sensor format.
- each pair of soft ISP / soft IPA implementations can use their own data
  format for statistics
