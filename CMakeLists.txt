CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(orthogen)

#----------------------------------------------------------------------
# options and flags
#----------------------------------------------------------------------
IF(MSVC)
   # /Ot optimize for time
   # /Oy omit frame pointer
   # /Ob2 more inlining
   # /GS- disable buffer check
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ot /Oy /Ob2 /GS- /arch:SSE2 /fp:fast")
   
   # no expensive range checking
   #ADD_DEFINITIONS(-D_SECURE_SCL=0)
   #ADD_DEFINITIONS(-D_ITERATOR_DEBUG_LEVEL=0)
   ADD_DEFINITIONS(-DUNICODE)
   ADD_DEFINITIONS(-D_UNICODE)
   
   # suppress MSVC security warnings
   # ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)   
ELSE(MSVC)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0 -fno-strict-aliasing")
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2 -mfpmath=sse -O3")
   ADD_DEFINITIONS(-std=c++0x -Wall)
   SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "$CMAKE_CXX_FLAGS_RELWITHDEBINFO$ -O3 -g")
   SET(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
ENDIF(MSVC)

# Dependencies

FIND_PACKAGE(Boost 1.40 COMPONENTS program_options REQUIRED )
set(Boost_USE_STATIC_LIBS   ON)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

#----------------------------------------------------------------------
# Main Project
#----------------------------------------------------------------------
SET(SRC_FILES 
    src/pnm.h src/pnm.cpp src/ifs.h src/image.h src/main.cpp)
    
ADD_EXECUTABLE(orthogen ${SRC_FILES})

TARGET_LINK_LIBRARIES( orthogen ${Boost_LIBRARIES} )
