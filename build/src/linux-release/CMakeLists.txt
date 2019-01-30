set(CMAKE_BUILD_TYPE Debug)
set(T "linux-release")
set(USE_MEMORY_CANVAS y)
add_definitions(-DHAVE_POSIX
                -DHAVE_STDINT_H
                -DHAVE_UNISTD_H
                -DHAVE_VASPRINTF
                -DUSE_MEMORY_CANVAS
#                -DENABLE_OPENGL
#                -DGL_GLEXT_PROTOTYPES
#                -DUSE_GLX
#                -DUSE_X11
                -DUSE_POLL_EVENT
                -DUSE_FREETYPE
                -DHAVE_VASPRINTF)
set(HAVE_POSIX TRUE)
include(../Include.cmake)

