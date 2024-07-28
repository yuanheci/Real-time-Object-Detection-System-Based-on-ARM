set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(tools /usr/local/arm/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf)
set(CMAKE_C_COMPILER ${tools}/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER ${tools}/bin/arm-linux-gnueabihf-g++)


# 设置 ZLIB 和 PNG 库路径
set(ZLIB_INCLUDE_DIRS "/home/rsh/zlib/include")
set(PNG_INCLUDE_DIRS "/home/rsh/png/include")

set(ZLIB_LIBRARY "/home/rsh/zlib/lib/libz.so")
set(PNG_LIBRARY "/home/rsh/png/lib/libpng.so")
