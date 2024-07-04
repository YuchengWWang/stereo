cmake_minimum_required(VERSION 3.5.1)
project(stereo_project)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Define source files
set(STEREO_SOURCE stereo.cpp)
set(PHOTO_SOURCE take_photo.cpp)
set(INTRINSIC_SOURCE intrinsic.cpp)
set(UNDISTORT_SOURCE undistortion.cpp)
set(EPIPOLAR_SOURCE epipolar_calibration.cpp)


add_executable(stereo ${STEREO_SOURCE})
target_link_libraries(stereo ${OpenCV_LIBS})

add_executable(photo ${PHOTO_SOURCE})
target_link_libraries(photo ${OpenCV_LIBS})

add_executable(intrinsic ${INTRINSIC_SOURCE})
target_link_libraries(intrinsic ${OpenCV_LIBS})

add_executable(undistort ${UNDISTORT_SOURCE})
target_link_libraries(undistort ${OpenCV_LIBS})

add_executable(epipolar_calibration ${EPIPOLAR_SOURCE})
target_link_libraries(epipolar_calibration ${OpenCV_LIBS})

# Require C++11 standard for both executables
set_target_properties(stereo PROPERTIES CXX_STANDARD_REQUIRED ON CXX_STANDARD 11)
set_target_properties(photo PROPERTIES CXX_STANDARD_REQUIRED ON CXX_STANDARD 11)
