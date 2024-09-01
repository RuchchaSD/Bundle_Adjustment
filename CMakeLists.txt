# Specify the minimum version of CMake required to build this project.
cmake_minimum_required(VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  if (MSVC)
    # Set the debug information format for MSVC compilers depending on the configuration.
    set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue,ProgramDatabase>")
  endif()
endif()

# Define the project name, version, and description.
project(Custom_BundleAdjustment VERSION 1.0 DESCRIPTION "Custom library for bundle adjustment")

# Set the C++ standard. Use C++20 if CMake version is greater than 3.12, otherwise, use C++17.
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set(CMAKE_CXX_STANDARD 20)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()
# Ensure that the selected C++ standard is required and must be supported by the compiler.
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Platform-specific compiler settings.
if (WIN32)
    # Windows-specific compiler flags or settings.
    message(STATUS "Configuring for Windows")
    # Define a preprocessor macro for Windows platform.
    add_definitions(-DPLATFORM_WINDOWS)
elseif (UNIX)
    # Linux-specific compiler flags or settings.
    message(STATUS "Configuring for Linux")
    # Define a preprocessor macro for Linux platform.
    add_definitions(-DPLATFORM_LINUX)
endif()

# Function to recursively add all subdirectories that contain header files.
function(add_include_directories_recursively directory)
    # Add the current directory to the include path.
    include_directories(${directory})

    # Get all subdirectories in the specified directory.
    file(GLOB children RELATIVE ${directory} ${directory}/*)
    foreach(child ${children})
        if(IS_DIRECTORY ${directory}/${child})
            # Recursively add subdirectories to the include path.
            add_include_directories_recursively(${directory}/${child})
        endif()
    endforeach()
endfunction()

# Add all header file directories under the "include" directory to the include path.
add_include_directories_recursively(${PROJECT_SOURCE_DIR}/include)

# Add the "Thirdparty" directory to the include path for third-party dependencies.
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty)

# Recursively find all .cpp source files in the "src" directory and store them in the SOURCES variable.
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create a library named "Custom_BundleAdjustment" using the source files stored in SOURCES.
add_library(Custom_BundleAdjustment ${SOURCES})

# Automatically add directories that contain CMakeLists.txt under the "Examples" directory.
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/Examples ${PROJECT_SOURCE_DIR}/Examples/*)
foreach(child ${children})
  if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/${child})
    if(EXISTS "${PROJECT_SOURCE_DIR}/Examples/${child}/CMakeLists.txt")
      # Add subdirectories that contain CMakeLists.txt for building example projects or test cases.
      add_subdirectory(Examples/${child})
    endif()
  endif()
endforeach()
