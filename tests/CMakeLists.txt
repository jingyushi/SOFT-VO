find_package(CppUnitLite REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB SOURCES_TEST "*.cpp")


SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

foreach (testFile ${SOURCES_TEST})
    string(REPLACE ".cpp" "" testname ${testFile})
    string(REPLACE "/home/zephyr/Programming/soft_slam/tests/" "" testname ${testFile})
    ADD_EXECUTABLE(${testname} ${testFile} ${SOURCES})
    TARGET_LINK_LIBRARIES(${testname}  ${OpenCV_LIBS})
    TARGET_LINK_LIBRARIES(${testname} CppUnitLite)
    TARGET_LINK_LIBRARIES(${testname} Eigen3::Eigen)
endforeach(testFile $SOURCES_TEST$)
