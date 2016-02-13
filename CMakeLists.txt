cmake_minimum_required(VERSION 2.8)
project( more_t2 )
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /usr/local/lib)
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr/local/include)
find_package(OpenCV REQUIRED core imgproc highgui)
find_package(OpenGL)
find_package(GLUT)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../..//Release/moret2/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../../Release/moret2/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../../Release/moret2/bin)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -DUSE_GIGE -std=c++11 ")
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl" )

IF(UNIX) 
    SET(GSL_CONFIG_PREFER_PATH "$ENV{GSL_HOME}/bin" CACHE STRING "preferred path to GSL (gsl-config)")
    FIND_PROGRAM(GSL_CONFIG gsl-config
      ${GSL_CONFIG_PREFER_PATH}
      $ENV{LIB_DIR}/bin
      /usr/local/bin/
      /usr/bin/
      )
    # MESSAGE("DBG GSL_CONFIG ${GSL_CONFIG}")
    
    IF (GSL_CONFIG) 
      # set CXXFLAGS to be fed into CXX_FLAGS by the user:
      SET(GSL_CXX_FLAGS "`${GSL_CONFIG} --cflags`")
      
      # set INCLUDE_DIRS to prefix+include
      EXEC_PROGRAM(${GSL_CONFIG}
        ARGS --prefix
        OUTPUT_VARIABLE GSL_PREFIX)
      SET(GSL_INCLUDE_DIR ${GSL_PREFIX}/include CACHE STRING INTERNAL)

      # set link libraries and link flags
      EXEC_PROGRAM(${GSL_CONFIG}
          ARGS --libs
          OUTPUT_VARIABLE GSL_LIBRARIES)
      
      ## extract link dirs for rpath  
      EXEC_PROGRAM(${GSL_CONFIG}
        ARGS --libs
        OUTPUT_VARIABLE GSL_CONFIG_LIBS )

      ## split off the link dirs (for rpath)
      ## use regular expression to match wildcard equivalent "-L*<endchar>"
      ## with <endchar> is a space or a semicolon
      STRING(REGEX MATCHALL "[-][L]([^ ;])+" 
        GSL_LINK_DIRECTORIES_WITH_PREFIX 
        "${GSL_CONFIG_LIBS}" )
        #      MESSAGE("DBG  GSL_LINK_DIRECTORIES_WITH_PREFIX=${GSL_LINK_DIRECTORIES_WITH_PREFIX}")

      ## remove prefix -L because we need the pure directory for LINK_DIRECTORIES
      
      IF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
        STRING(REGEX REPLACE "[-][L]" "" GSL_LINK_DIRECTORIES ${GSL_LINK_DIRECTORIES_WITH_PREFIX} )
      ENDIF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
      SET(GSL_EXE_LINKER_FLAGS "-Wl,-rpath,${GSL_LINK_DIRECTORIES}" CACHE STRING INTERNAL)
      #      MESSAGE("DBG  GSL_LINK_DIRECTORIES=${GSL_LINK_DIRECTORIES}")
      #      MESSAGE("DBG  GSL_EXE_LINKER_FLAGS=${GSL_EXE_LINKER_FLAGS}")

      #      ADD_DEFINITIONS("-DHAVE_GSL")
      #      SET(GSL_DEFINITIONS "-DHAVE_GSL")
      MARK_AS_ADVANCED(
        GSL_CXX_FLAGS
        GSL_INCLUDE_DIR
        GSL_LIBRARIES
        GSL_LINK_DIRECTORIES
        GSL_DEFINITIONS
      )
      
    ELSE(GSL_CONFIG)

      IF (GSL_FIND_REQUIRED)
         MESSAGE(FATAL_ERROR "Could not find gsl-config. Please set it manually. GSL_CONFIG=${GSL_CONFIG}")
      ELSE (GSL_FIND_REQUIRED)
         MESSAGE(STATUS "Could not find GSL")
         # TODO: Avoid cmake complaints if GSL is not found
      ENDIF (GSL_FIND_REQUIRED)

    ENDIF(GSL_CONFIG)

  ENDIF(UNIX)

include_directories( include ${OpenCV_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/../moret2/packages/ARToolKitPlus-2.2.1/include ${GSL_INCLUDE_DIRS} ${GSLCBLAS_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}  ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS})

add_library(trackerhelper src/trackerhelper.cpp )
target_link_libraries(trackerhelper ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  ${CMAKE_BINARY_DIR}/../moret2/packages/ARToolKitPlus-2.2.1/lib/libARToolKitPlus.dylib  ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})


add_executable(adjustCamPose src/adjustCamPose.cpp)
target_link_libraries(adjustCamPose trackerhelper )

#include_directories( include ${OpenCV_INCLUDE_DIRS} )
#add_executable(getscalingfactor src/getscalingfactor.cpp)
#target_link_libraries(getscalingfactor trackerhelper )

add_executable(genpose src/genpose.cpp)
target_link_libraries(genpose trackerhelper )


add_executable(gencampose src/genCamPose.cpp)
target_link_libraries(gencampose trackerhelper)

add_executable(markerDetect src/markerDetect.cpp)
target_link_libraries(markerDetect trackerhelper )

add_executable(triggergen src/extra/triggergen.cpp)
target_link_libraries(triggergen trackerhelper )


add_custom_command(TARGET triggergen POST_BUILD
					COMMAND $(CMAKE_COMMAND) -E copy ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}/triggergen ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}/extra/)