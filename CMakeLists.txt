cmake_minimum_required(VERSION 3.5)

# Set the project name
project (spot_micro_kinematics)

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 REQUIRED NO_MODULE)

# Add an library for the example classes
add_library(spot_micro_kinematics 
    src/spot_micro_kinematics.cpp
)

target_include_directories(spot_micro_kinematics
	 PUBLIC 
		${PROJECT_SOURCE_DIR}/include
)

#target_link_libraries(spot_micro_kinematics
	#Eigen3::Eigen
#)

#############################################
# Unit tests

add_subdirectory(3rd_party/google-test)

# enable CTest testing
enable_testing()

# Add a testing executable
add_executable(test1 tests/test_1.cpp)

target_link_libraries(test1
    spot_micro_kinematics
    GTest::GTest 
    GTest::Main
)

add_test(test_all test1)
