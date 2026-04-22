# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_topics_quiz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED topics_quiz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(topics_quiz_FOUND FALSE)
  elseif(NOT topics_quiz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(topics_quiz_FOUND FALSE)
  endif()
  return()
endif()
set(_topics_quiz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT topics_quiz_FIND_QUIETLY)
  message(STATUS "Found topics_quiz: 0.0.0 (${topics_quiz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'topics_quiz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${topics_quiz_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(topics_quiz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${topics_quiz_DIR}/${_extra}")
endforeach()
