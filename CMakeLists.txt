cmake_minimum_required (VERSION 2.6)
project (SoftSLAM)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})  
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}) 

INCLUDE_DIRECTORIES("include/")
INCLUDE_DIRECTORIES("src/")

file(GLOB SOURCES "src/*.cpp" )
#add_library(${PROJECT_NAME} SHARED ${sources})

#file(GLOB SOURCES "tests/*.cpp")
add_subdirectory (tests)

#ADD_EXECUTABLE(main "main.cpp" ${SOURCES})


