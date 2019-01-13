cmake_minimum_required(VERSION 3.5)

project(unity_ros2 C)

find_package(ament_cmake REQUIRED)

find_package(rcldotnet_common REQUIRED)
foreach(_assembly_dep ${rcldotnet_common_ASSEMBLIES_DLL})
  list(APPEND _assembly_deps_dll "${_assembly_dep}")
endforeach()

find_package(rclcs REQUIRED)
foreach(_assembly_dep ${rclcs_ASSEMBLIES_DLL})
  list(APPEND _assembly_deps_dll "${_assembly_dep}")
endforeach()

find_package(rosidl_generator_cs REQUIRED)
foreach(_assembly_dep ${rosidl_generator_cs_ASSEMBLIES_DLL})
  list(APPEND _assembly_deps_dll "${_assembly_dep}")
endforeach()

install(
  FILES ${_assembly_deps_dll}
  DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Plugins"
)

ament_package()
