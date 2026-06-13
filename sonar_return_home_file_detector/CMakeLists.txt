cmake_minimum_required(VERSION 3.0.2)
project(sonar_return_home_file_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS common io)
find_package(OpenCV QUIET)
find_package(OpenMP QUIET)

option(SONAR_RETURN_HOME_FILE_ENABLE_FOURIER_SOFT2D
  "Enable the vendored Fourier-SOFT 2D matcher backend" ON)

if(SONAR_RETURN_HOME_FILE_ENABLE_FOURIER_SOFT2D)
  find_path(FFTW3_INCLUDE_DIR fftw3.h)
  find_library(FFTW3_LIBRARY fftw3)
  find_path(CGAL_INCLUDE_DIR CGAL/Simple_cartesian.h)
  find_library(GMP_LIBRARY gmp)
  find_library(MPFR_LIBRARY mpfr)
  if(NOT FFTW3_INCLUDE_DIR OR NOT FFTW3_LIBRARY OR
     NOT CGAL_INCLUDE_DIR OR NOT GMP_LIBRARY OR NOT MPFR_LIBRARY OR
     NOT OpenCV_FOUND OR NOT OpenMP_CXX_FOUND)
    message(WARNING
      "sonar_return_home_file_detector: Fourier-SOFT 2D backend disabled. "
      "Missing FFTW3, CGAL, GMP, MPFR, OpenCV, or OpenMP.")
    set(SONAR_RETURN_HOME_FILE_ENABLE_FOURIER_SOFT2D OFF)
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sonar_return_home_file_detector
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions
)

include_directories(
  include
  src
  src/core/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(FOURIER_SOFT2D_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/fourier-soft-2d/src)
set(FOURIER_SOFT2D_SOURCES
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/rotate_so3_fftw.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/rotate_so3_utils.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/s2_cospmls.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/s2_legendreTransforms.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/s2_semi_fly.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/s2_semi_memo.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/so3_correlate_fftw.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/soft_fftw.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/soft_fftw_nt.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/soft_fftw_pc.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/utils_vec_cx.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/wignerTransforms_fftw.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/wrap_s2_rotate_fftw.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/wrap_soft_fftw.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/lib1/wrap_soft_fftw_cor2.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/common/csecond.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/common/makeweights.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/common/makeWigner.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/common/s2_primitive.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/common/utils_so3.c
  ${FOURIER_SOFT2D_DIR}/soft20/src/common/wignerTransforms.c
  ${FOURIER_SOFT2D_DIR}/registration/src/softCorrelationClass.cpp
  ${FOURIER_SOFT2D_DIR}/registration/src/softDescriptorRegistration.cpp
  ${FOURIER_SOFT2D_DIR}/tools/src/PeakFinder.cpp
)

if(SONAR_RETURN_HOME_FILE_ENABLE_FOURIER_SOFT2D)
  add_library(sonar_return_home_file_fourier_soft2d_backend
    ${FOURIER_SOFT2D_SOURCES}
  )
  target_include_directories(sonar_return_home_file_fourier_soft2d_backend SYSTEM PUBLIC
    ${FFTW3_INCLUDE_DIR}
    ${CGAL_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${FOURIER_SOFT2D_DIR}/tools/include
    ${FOURIER_SOFT2D_DIR}/soft20/include
    ${FOURIER_SOFT2D_DIR}/soft20/include/soft20
    ${FOURIER_SOFT2D_DIR}/registration/include
  )
  target_link_libraries(sonar_return_home_file_fourier_soft2d_backend
    ${FFTW3_LIBRARY}
    OpenMP::OpenMP_CXX
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${GMP_LIBRARY}
    ${MPFR_LIBRARY}
  )
endif()

add_library(sonar_return_home_file_detector
  src/return_home_file_api.cpp
  src/core/return_home_detector.cpp
  src/core/fourier_soft2d_matcher.cpp
)
target_link_libraries(sonar_return_home_file_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(sonar_return_home_file_detector
  ${catkin_EXPORTED_TARGETS}
)

if(SONAR_RETURN_HOME_FILE_ENABLE_FOURIER_SOFT2D)
  target_compile_definitions(sonar_return_home_file_detector PRIVATE
    SONAR_RETURN_HOME_ENABLE_FOURIER_SOFT2D=1)
  target_include_directories(sonar_return_home_file_detector PRIVATE
    ${FOURIER_SOFT2D_DIR}/tools/include
    ${FOURIER_SOFT2D_DIR}/soft20/include
    ${FOURIER_SOFT2D_DIR}/soft20/include/soft20
    ${FOURIER_SOFT2D_DIR}/registration/include
    ${FFTW3_INCLUDE_DIR}
    ${CGAL_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(sonar_return_home_file_detector
    sonar_return_home_file_fourier_soft2d_backend)
else()
  target_compile_definitions(sonar_return_home_file_detector PRIVATE
    SONAR_RETURN_HOME_ENABLE_FOURIER_SOFT2D=0)
endif()

add_executable(return_home_directory_probe
  tools/return_home_directory_probe.cpp
)
target_link_libraries(return_home_directory_probe
  sonar_return_home_file_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(return_home_directory_probe
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
  sonar_return_home_file_detector
  return_home_directory_probe
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(SONAR_RETURN_HOME_FILE_ENABLE_FOURIER_SOFT2D)
  install(TARGETS sonar_return_home_file_fourier_soft2d_backend
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config docs scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
