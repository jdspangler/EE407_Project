cmake_minimum_required (VERSION 3.5)

project(Pong_final_proj)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED) 
find_package(CUDA REQUIRED) 

pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4 
                               gstreamer-sdp-1.0>=1.4 
                               gstreamer-video-1.0>=1.4 
                               gstreamer-app-1.0>=1.4) 

configure_file( 
	"${PROJECT_SOURCE_DIR}/leastSquare.h"
	"${PROJECT_BINARY_DIR}/leastSquare.h"
	)

include_directories(${OpenCV_INCLUDE_DIRS}, ${GST_INCLUDE_DIRS}, ${CUDA_INCLUDE_DIRS}, "${PROJECT_BINARY_DIR}")
add_definitions(${GSTREAMER_DEFINITIONS})

add_executable(Pong_final_proj Pong_final.cpp leastSquare.cpp)

MESSAGE( STATUS "GST_INCLUDE_DIRS:         " ${GST_INCLUDE_DIRS} )
MESSAGE( STATUS "GST_LIBRARIES:         " ${GST_LIBRARIES} )

target_link_libraries(Pong_final_proj ${OpenCV_LIBS})
target_link_libraries(Pong_final_proj ${GST_LIBRARIES})
target_link_libraries(Pong_final_proj ${CUDA_LIBRARIES})
