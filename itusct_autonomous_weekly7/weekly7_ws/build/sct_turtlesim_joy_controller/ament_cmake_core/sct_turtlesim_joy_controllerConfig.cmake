# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sct_turtlesim_joy_controller_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sct_turtlesim_joy_controller_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sct_turtlesim_joy_controller_FOUND FALSE)
  elseif(NOT sct_turtlesim_joy_controller_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sct_turtlesim_joy_controller_FOUND FALSE)
  endif()
  return()
endif()
set(_sct_turtlesim_joy_controller_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sct_turtlesim_joy_controller_FIND_QUIETLY)
  message(STATUS "Found sct_turtlesim_joy_controller: 0.0.0 (${sct_turtlesim_joy_controller_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sct_turtlesim_joy_controller' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sct_turtlesim_joy_controller_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sct_turtlesim_joy_controller_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sct_turtlesim_joy_controller_DIR}/${_extra}")
endforeach()
