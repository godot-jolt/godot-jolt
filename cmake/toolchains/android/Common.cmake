set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_SYSTEM_VERSION 21)
set(CMAKE_ANDROID_STL_TYPE c++_static)
set(CMAKE_ANDROID_EXCEPTIONS FALSE)

# HACK(mihe): CMake 3.31 dropped support for projects compatible with CMake versions older than
# 3.10, which breaks the Android NDK, since it declares 3.6 as its minimum version. So we just
# disable deprecation warnings altogether.
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
set(CMAKE_ERROR_DEPRECATED OFF CACHE BOOL "" FORCE)
