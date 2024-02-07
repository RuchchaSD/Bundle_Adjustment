cmake_minimum_required(VERSION 3.8)
# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

# Set the project name and version
project(Bundle_Adjustment VERSION 1.0.0)

# Specify the C++ standard, use C++20 if supported
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set(CMAKE_CXX_STANDARD 20)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories for header files
include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/Thirdparty
	${PROJECT_SOURCE_DIR}/Other
	${PROJECT_SOURCE_DIR}/ThirdParty/g2o
	)

link_directories("${PROJECT_SOURCE_DIR}/Thirdparty/g2o/build/g2o")

# Source directory for source files
add_subdirectory(src)
add_subdirectory(Other)

# If you plan on having tests, examples, or external libraries, include them here as well
# add_subdirectory(tests)
# add_subdirectory(examples)
# add_subdirectory(libs)

# TODO: Add install targets if needed
