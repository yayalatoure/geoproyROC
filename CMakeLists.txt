cmake_minimum_required(VERSION 3.5)
project(GeoProyectiva)

find_package(OpenCV REQUIRED)
find_package(Qt5Widgets REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

file(GLOB SOURCES *.h *.cpp)

set(SOURCE_FILES main.cpp geoproyectiva.cpp geoproyectiva.h foot.cpp foot.h genroc.cpp genroc.h)

add_executable(GeoProyectiva ${SOURCE_FILES} ${SOURCES})

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(GeoProyectiva ${OpenCV_LIBS})

target_link_libraries(GeoProyectiva Qt5::Widgets)