cmake_minimum_required(VERSION 3.5)

project(unity_ros2 C)

find_package(ament_cmake REQUIRED)

# -------------------- Utils --------------------------
# TODO(samaim): move to different file?
macro(dll_to_unity ros_package)
  find_package(${ros_package} REQUIRED)
  foreach(_assembly_dep "${${ros_package}_ASSEMBLIES_DLL}")
    list(APPEND _assembly_deps_dll "${_assembly_dep}")
  endforeach()
endmacro()

function(add_to_unity)
  cmake_parse_arguments(
    PARSED_ARGS
    ""
    ""
    "DEPENDS"
    ${ARGN}
  )
  foreach(DEPEND ${PARSED_ARGS_DEPENDS})
    dll_to_unity(${DEPEND})
  endforeach()

  install(
    FILES ${_assembly_deps_dll}
    DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Plugins"
  )
endfunction(add_to_unity)

# ------------------------------------------------------


# Add additional packages here...
add_to_unity(
  DEPENDS
    rcldotnet_common
    rclcs
    rosidl_generator_cs
    std_msgs
    geometry_msgs
    builtin_interfaces
)

ament_package()
