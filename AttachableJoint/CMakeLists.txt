cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(AttachableJoint)
find_package(gz-physics7 REQUIRED)
set(GZ_MSGS_VER ${gz-physics7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED)
set(GZ_MSGS_VER ${gz-common5_VERSION_MAJOR})




find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
add_library(AttachableJoint SHARED AttachableJoint.cc AttachableJoint.hh)
set_property(TARGET AttachableJoint PROPERTY CXX_STANDARD 17)
target_link_libraries(AttachableJoint
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gz-common5::gz-common5
  PRIVATE gz-physics7::gz-physics7
  )


