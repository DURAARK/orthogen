CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(orthogen)
# ulrich.krispel@fraunhofer.at

#----------------------------------------------------------------------
# options and flags
#----------------------------------------------------------------------
IF(MSVC)
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ot /Oy /Ob2 /GS- /arch:SSE2 /fp:fast /openmp")
   ADD_DEFINITIONS(-DUNICODE)
   ADD_DEFINITIONS(-D_UNICODE)
   ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
ELSE(MSVC)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0 -fno-strict-aliasing -std=c++11 -fopenmp")
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2 -std=c++11 -mfpmath=sse -O3 -fopenmp")
   ADD_DEFINITIONS(-Wall)
   SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "$CMAKE_CXX_FLAGS_RELWITHDEBINFO$ -O3 -std=c++11 -g -fopenmp")
   SET(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -std=c++11 -fopenmp")
ENDIF(MSVC)

#----------------------------------------------------------------------
# Dependencies
#----------------------------------------------------------------------

###### EIGEN
IF (NOT EIGEN3_INCLUDE_DIR) 
  FIND_PATH(EIGEN3_INCLUDE_DIR "Eigen/Eigen" ENV EIGEN3_DIR )
ENDIF()
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_DIR to the base directory of your Eigen3 installation ($ENV{EIGEN3_DIR})")
ENDIF()

INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

###### LibJPEG

set(LIBJPEG_PATH ../lib/libjpeg CACHE FILEPATH "path to libjpeg src")
ADD_SUBDIRECTORY(${LIBJPEG_PATH})
include_directories(${LIBJPEG_PATH})
include_directories(${CMAKE_BINARY_DIR}/${LIBJPEG_PATH})

#----------------------------------------------------------------------
# Main Project
#----------------------------------------------------------------------
SET(SRC_FILES 
    ../src/image.h
    src/align.cpp
    )

ADD_EXECUTABLE(orthogen ${SRC_FILES})
TARGET_LINK_LIBRARIES( orthogen ${Boost_LIBRARIES} libjpeg)
