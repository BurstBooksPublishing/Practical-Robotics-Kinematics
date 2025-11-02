# Minimal CMake snippet — add to CMakeLists.txt
cmake_minimum_required(VERSION 3.13)
project(robot_kinematics)

find_package(Python COMPONENTS Interpreter) # simple check
if(WIN32)
  set(PLATFORM_LIB_NAME "pinocchio.dll") # Windows DLL name
  # On Windows prefer static runtime or ensure matching MSVC toolset
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
elseif(APPLE)
  set(PLATFORM_LIB_NAME "libpinocchio.dylib") # macOS dylib name
  set(CMAKE_MACOSX_RPATH TRUE) # prefer rpath for macOS
else()
  set(PLATFORM_LIB_NAME "libpinocchio.so") # Linux SO name
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

# Link target and install rpath configuration
add_library(robot_kinematics SHARED src/kinematics.cpp)
target_link_libraries(robot_kinematics PRIVATE ${PLATFORM_LIB_NAME})
# Ensure runtime search path is correct after install
set_target_properties(robot_kinematics PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib"
)