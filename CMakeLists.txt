CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(orthogen)

#----------------------------------------------------------------------
# options and flags
#----------------------------------------------------------------------
IF(MSVC)
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ot /Oy /Ob2 /GS- /arch:SSE2 /fp:fast")
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

# Dependencies

# EIGEN
IF (NOT EIGEN3_INCLUDE_DIR) 
  SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
ENDIF()

IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

#find_package(PCL 1.3 REQUIRED COMPONENTS common io)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# use boost from PCL
FIND_PACKAGE(Boost 1.40 COMPONENTS program_options REQUIRED )
set(Boost_USE_STATIC_LIBS   ON)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )

# Libe57
#set(LIBE57_RELEASE ${CMAKE_SOURCE_DIR}/lib/E57RefImpl_src-1.1.312/MSVC2013/Release/E57RefImpl.lib CACHE FILEPATH "LibE57 release")
#set(LIBE57_DEBUG ${CMAKE_SOURCE_DIR}/lib/E57RefImpl_src-1.1.312/MSVC2013/Debug/E57RefImpl-d.lib CACHE FILEPATH "LibE57 debug")
#add_library(libe57 STATIC IMPORTED)
#set_property(TARGET libe57 PROPERTY IMPORTED_LOCATION_RELEASE ${LIBE57_RELEASE})
#set_property(TARGET libe57 PROPERTY IMPORTED_LOCATION_DEBUG   ${LIBE57_DEBUG})

# Xerces (libe57 dependency)
#set(XERCES_RELEASE I:/LIBS/xerces-c-3.1.1/MSVC2013/Release/Xerces.lib CACHE FILEPATH "desc")
#set(XERCES_DEBUG I:/LIBS/xerces-c-3.1.1/MSVC2013/Debug/Xerces.lib CACHE FILEPATH "desc")

#add_library(xerces STATIC IMPORTED)
#set_property(TARGET xerces PROPERTY IMPORTED_LOCATION_RELEASE ${XERCES_RELEASE})
#set_property(TARGET xerces PROPERTY IMPORTED_LOCATION_DEBUG   ${XERCES_DEBUG})
#set(LIBE57_LIBS libe57 xerces)

set(LIBJPEG_PATH lib/libjpeg CACHE FILEPATH "path to libjpeg src")
ADD_SUBDIRECTORY(${LIBJPEG_PATH})
include_directories(${LIBJPEG_PATH})
include_directories(${CMAKE_BINARY_DIR}/${LIBJPEG_PATH})
#----------------------------------------------------------------------
# Main Project
#----------------------------------------------------------------------
SET(SRC_FILES 
    src/types.h
    src/ifs.h 
    src/image.h
    src/pose.h
    src/projection.h
    src/projection.cpp
    src/quad.h
    src/meanshift.h
    src/main.cpp
    )
    
ADD_EXECUTABLE(orthogen ${SRC_FILES})

TARGET_LINK_LIBRARIES( orthogen ${Boost_LIBRARIES} ${LIBE57_LIBS} libjpeg)
