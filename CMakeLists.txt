project(reconstructionFromHandObjectInteractions)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${PROJECT_SOURCE_DIR}/include)
include_directories( ${PROJECT_SOURCE_DIR}/3rd_party/cpu_tsdf/include)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
find_package (CPUTSDF REQUIRED)
include_directories(${CPUTSDF_INCLUDE_DIRS})
link_directories(${CPUTSDF_LIBRARY_DIRS})

# makes sure that it is compiled with 02 (O3 and up gives bad result)
  set ( CMAKE_CXX_FLAGS "-O2"
        CACHE STRING "g++ Compiler Flags for All Builds" FORCE)
  set ( CMAKE_CXX_FLAGS_DEBUG "-g"
        CACHE STRING "g++ Compiler Flags for Debug Builds" FORCE)
  set ( CMAKE_CXX_FLAGS_RELEASE  "-DNDEBUG"
        CACHE STRING "g++ Compiler Flags for Release Builds" FORCE)
  set ( CMAKE_CXX_FLAGS_RELWITHDEBINFO  "-DNDEBUG"
        CACHE STRING "g++ Compiler Flags for Release Builds" FORCE)
  set ( CMAKE_CXX_FLAGS_MINSIZEREL  "-DNDEBUG"
        CACHE STRING "g++ Compiler Flags for Release Builds" FORCE)

find_package(Boost COMPONENTS system thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions( -DBOOST_ALL_NO_LIB )

message("\n\nFound BOOST\n\n")

# Opencv required
find_package(OpenCV COMPONENTS core imgproc highgui features2d nonfree calib3d REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})

message("\n\nFound OpenCV\n\n")

find_package (Threads)

# QT4 required
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

message("\n\nFound QT4\n\n")


# LIBCONFIG required
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
find_package( LibConfig REQUIRED )
include_directories( ${LIBCONFIGPP_INCLUDE_DIR} )
#set(LIBS ${LIBS} ${LIBCONFIG_LIBRARIES})

message("\n Found LIBCONFIG \n")


find_package(PCL 1.7 REQUIRED COMPONENTS common features filters io registration sample_consensus search visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message("\n\nFound PCL\n\n")


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


file(GLOB PROJECT_SOURCES src/*.cpp)

file(GLOB PROJECT_INCLUDES include/*.h)

# create moc for QT
QT4_WRAP_CPP(QT_MOC ${QT_HEADERS})

# process ui
QT4_WRAP_UI(QT_FORMS_HEADERS ${QT_FORMS})

ADD_EXECUTABLE(reconstructionFromHandObjectInteractions	    ${PROJECT_SOURCES} ${PROJECT_INCLUDES} ${QT_INCLUDES}
							    ${QT_HEADERS}
							    ${QT_MOC}
							    ${QT_FORMS})

TARGET_LINK_LIBRARIES(reconstructionFromHandObjectInteractions ${QT_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES} ${LIBCONFIGPP_LIBRARIES} ${CPUTSDF_LIBRARIES})
set_property(TARGET reconstructionFromHandObjectInteractions PROPERTY COMPILE_DEFINITIONS QT_SHARED _REENTRANT)
