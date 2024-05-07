cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project(Custom_BundleAdjustment VERSION 1.0 DESCRIPTION "Custom library for bundle adjustment")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set(CMAKE_CXX_STANDARD 20)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED True)


# Function to recursively add all subdirectories that contain header files
function(add_include_directories_recursively directory)
    # Add the current directory
    include_directories(${directory})

    # Get all subdirectories
    file(GLOB children RELATIVE ${directory} ${directory}/*)
    foreach(child ${children})
        if(IS_DIRECTORY ${directory}/${child})
            # Recursively add subdirectories
            add_include_directories_recursively(${directory}/${child})
        endif()
    endforeach()
endfunction()

add_include_directories_recursively(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_library(Custom_BundleAdjustment ${SOURCES})

# Automatically add directories that contain CMakeLists.txt
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/Examples ${PROJECT_SOURCE_DIR}/Examples/*)
foreach(child ${children})
  if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/${child})
    if(EXISTS "${PROJECT_SOURCE_DIR}/Examples/${child}/CMakeLists.txt")
      add_subdirectory(Examples/${child})
    endif()
  endif()
endforeach()