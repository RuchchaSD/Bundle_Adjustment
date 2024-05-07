cmake_minimum_required(VERSION 3.10)
project(Generated_data_BA)

add_include_directories_recursively(${PROJECT_SOURCE_DIR}/include)
include_directories(../../Thirdparty)

# Define the executable to build from the provided source files
add_executable(Generated_data_BA main.cpp "Reprojection_Generated_dataset.h" Generated_dataSet_BA.cpp)

# If other cpp files are added, include them in the above executable definition

# Link the main library
target_link_libraries(Generated_data_BA Custom_BundleAdjustment)
