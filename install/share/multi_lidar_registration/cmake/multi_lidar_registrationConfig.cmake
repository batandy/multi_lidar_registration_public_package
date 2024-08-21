# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_multi_lidar_registration_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED multi_lidar_registration_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(multi_lidar_registration_FOUND FALSE)
  elseif(NOT multi_lidar_registration_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(multi_lidar_registration_FOUND FALSE)
  endif()
  return()
endif()
set(_multi_lidar_registration_CONFIG_INCLUDED TRUE)

# output package information
if(NOT multi_lidar_registration_FIND_QUIETLY)
  message(STATUS "Found multi_lidar_registration: 0.0.0 (${multi_lidar_registration_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'multi_lidar_registration' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${multi_lidar_registration_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(multi_lidar_registration_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${multi_lidar_registration_DIR}/${_extra}")
endforeach()
