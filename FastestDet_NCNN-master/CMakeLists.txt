cmake_minimum_required(VERSION 3.2)
add_compile_options(-std=c++11)
project(FastestDet)
SET(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads)
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIRS})

if(OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(FastestDet src/FastestDet.cpp)

# 设置 ZLIB 和 PNG 头文件路径
set(ZLIB_INCLUDE_DIRS "/home/rsh/zlib/include")
set(PNG_INCLUDE_DIRS "/home/rsh/png/include")

# 设置 ZLIB 和 PNG 库路径
set(ZLIB_LIBRARY "/home/rsh/zlib/lib/libz.so")
set(PNG_LIBRARY "/home/rsh/png/lib/libpng.so")

# 添加头文件路径
include_directories(${ZLIB_INCLUDE_DIRS})
include_directories(${PNG_INCLUDE_DIRS})

# 链接所需的库
target_link_libraries(FastestDet ${PNG_LIBRARY} ${ZLIB_LIBRARY} pthread ${OpenCV_LIBS} ncnn ${PROJECT_SOURCE_DIR}/lib/libncnn.a)

