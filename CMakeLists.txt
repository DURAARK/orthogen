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
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0 -fno-strict-aliasing -std=c++11")
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2 -mfpmath=sse -O3")
   ADD_DEFINITIONS(-Wall)
   SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "$CMAKE_CXX_FLAGS_RELWITHDEBINFO$ -O3 -g")
   SET(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
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

###### Boost

FIND_PACKAGE(Boost 1.65 COMPONENTS program_options filesystem system REQUIRED )
option(Boost_USE_STATIC_LIBS "Use static libraries of Boost" OFF)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )

add_definitions(-D BOOST_ALL_DYN_LINK)
add_definitions(-DBOOST_USE_WINAPI_VERSION=0x0601) # otherwise we get boost log linker errors

###### LibJPEG

set(LIBJPEG_PATH lib/libjpeg CACHE FILEPATH "path to libjpeg src")
ADD_SUBDIRECTORY(${LIBJPEG_PATH})
include_directories(${LIBJPEG_PATH})
include_directories(${CMAKE_BINARY_DIR}/${LIBJPEG_PATH})

###### RapidJSON
IF (NOT RAPIDJSON_INCLUDE_DIR) 
  FIND_PATH(RAPIDJSON_INCLUDE_DIR "rapidjson/document.h" HINTS "lib/rapidjson/include")
ENDIF()
IF(NOT RAPIDJSON_INCLUDE_DIR)
    MESSAGE( FATAL_ERROR "Please set RAPIDJSON_INCLUDE_DIR")
ENDIF()
include_directories(${RAPIDJSON_INCLUDE_DIR})

#----------------------------------------------------------------------
# Main Project
#----------------------------------------------------------------------
SET(SRC_FILES 
    src/types.h
    src/ifs.h 
    src/image.h
    src/pose.h
    src/extraction.h
    src/projection.h
    src/projection.cpp
    src/quad.h
    src/meanshift.h
    src/main.cpp
    src/config.h
    )

ADD_EXECUTABLE(orthogen ${SRC_FILES})
TARGET_LINK_LIBRARIES(orthogen ${Boost_LIBRARIES} libjpeg)

#----------------------------------------------------------------------
# panoalign
#----------------------------------------------------------------------
SET(SRC_PANOALIGN
  src/meanshift.h
  src/image.h
  
  panoalign/align.cpp
  )
ADD_EXECUTABLE(panoalign ${SRC_PANOALIGN})
TARGET_LINK_LIBRARIES(panoalign ${Boost_LIBRARIES} libjpeg)

