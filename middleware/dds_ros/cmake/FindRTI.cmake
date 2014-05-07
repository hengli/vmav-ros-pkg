# FindRTI.cmake - Find RTI libraries & dependencies.
#
# This module defines the following variables which should be referenced
# by the caller to use the library.
#
# RTI_FOUND: TRUE iff RTI and all dependencies have been found.
# RTI_INCLUDE_DIR: Include directory for RTI.
# RTI_LIBRARIES: Libraries for RTI and all dependencies.
#
# The following variables control the behaviour of this module:
#
# RTI_INCLUDE_DIR_HINTS: List of additional directories in which to
#                        search for RTI includes,
#                        e.g: /timbuktu/include.
# RTI_LIBRARY_DIR_HINTS: List of additional directories in which to
#                        search for RTI libraries, e.g: /timbuktu/lib.
#
# The following variables are also defined by this module, but in line with
# CMake recommended FindPackage() module style should NOT be referenced directly
# by callers (use the plural variables detailed above instead).  These variables
# do however affect the behaviour of the module via FIND_[PATH/LIBRARY]() which
# are NOT re-called (i.e. search for library is not repeated) if these variables
# are set with valid values _in the CMake cache_. This means that if these
# variables are set directly in the cache, either by the user in the CMake GUI,
# or by the user passing -DVAR=VALUE directives to CMake when called (which
# explicitly defines a cache variable), then they will be used verbatim,
# bypassing the HINTS variables and other hard-coded search locations.
#
# RTI_INCLUDE_DIR: Include directory for RTI, not including the
#                  include directory of any dependencies.
# RTI_LIBRARY: RTI library, not including the libraries of any
#              dependencies.

# Called if we failed to find RTI or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
MACRO(RTI_REPORT_NOT_FOUND REASON_MSG)
  UNSET(RTI_FOUND)
  UNSET(RTI_INCLUDE_DIRS)
  UNSET(RTI_LIBRARIES)
  # Make results of search visible in the CMake GUI if RTI has not
  # been found so that user does not have to toggle to advanced view.
  MARK_AS_ADVANCED(CLEAR RTI_INCLUDE_DIR
                         RTI_LIBRARY)
  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
  # use the camelcase library name, not uppercase.
  IF (RTI_FIND_QUIETLY)
    MESSAGE(STATUS "Failed to find RTI - " ${REASON_MSG} ${ARGN})
  ELSEIF (RTI_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Failed to find RTI - " ${REASON_MSG} ${ARGN})
  ELSE()
    # Neither QUIETLY nor REQUIRED, use no priority which emits a message
    # but continues configuration and allows generation.
    MESSAGE("-- Failed to find RTI - " ${REASON_MSG} ${ARGN})
  ENDIF ()
ENDMACRO(RTI_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
#
# TODO: Add standard Windows search locations for RTI.
LIST(APPEND RTI_CHECK_INCLUDE_DIRS
  /opt/RTI/ndds.5.0.0/include
  /root/RTI/ndds.5.0.0/include)
LIST(APPEND RTI_CHECK_LIBRARY_DIRS
  /opt/RTI/ndds.5.0.0/lib/i86Linux2.6gcc4.4.5
  /opt/RTI/ndds.5.0.0/lib/x64Linux2.6gcc4.4.5
  /root/RTI/ndds.5.0.0/lib/armv7neonhfLinux3.xgcc4.6.3)

# Search supplied hint directories first if supplied.
FIND_PATH(RTI_INCLUDE_DIR
  NAMES ndds/ndds_cpp.h
  PATHS ${RTI_INCLUDE_DIR_HINTS}
  ${RTI_CHECK_INCLUDE_DIRS})
IF (NOT RTI_INCLUDE_DIR OR
    NOT EXISTS ${RTI_INCLUDE_DIR})
  RTI_REPORT_NOT_FOUND(
    "Could not find RTI include directory, set RTI_INCLUDE_DIR "
    "to directory containing ndds/ndds_cpp.h")
ENDIF (NOT RTI_INCLUDE_DIR OR
       NOT EXISTS ${RTI_INCLUDE_DIR})

SET(RTI_C_FOUND TRUE)
LIST(APPEND RTI_FOUND_REQUIRED_VARS RTI_C_FOUND)
FIND_LIBRARY(RTI_C_LIBRARY NAMES nddsc
  PATHS ${RTI_CHECK_LIBRARY_DIRS})
IF (EXISTS ${RTI_C_LIBRARY})
  MESSAGE(STATUS "Found RTI C library: ${RTI_C_LIBRARY}")
ELSE (EXISTS ${RTI_C_LIBRARY})
  RTI_REPORT_NOT_FOUND(
    "Did not find RTI C library (required RTI component).")
  SET(RTI_C_FOUND FALSE)
ENDIF (EXISTS ${RTI_C_LIBRARY})
MARK_AS_ADVANCED(RTI_C_LIBRARY)

SET(RTI_CPP_FOUND TRUE)
LIST(APPEND RTI_FOUND_REQUIRED_VARS RTI_CPP_FOUND)
FIND_LIBRARY(RTI_CPP_LIBRARY NAMES nddscpp
  PATHS ${RTI_CHECK_LIBRARY_DIRS})
IF (EXISTS ${RTI_CPP_LIBRARY})
  MESSAGE(STATUS "Found RTI CPP library: ${RTI_CPP_LIBRARY}")
ELSE (EXISTS ${RTI_CPP_LIBRARY})
  RTI_REPORT_NOT_FOUND(
    "Did not find RTI CPP library (required RTI component).")
  SET(RTI_CPP_FOUND FALSE)
ENDIF (EXISTS ${RTI_CPP_LIBRARY})
MARK_AS_ADVANCED(RTI_CPP_LIBRARY)

SET(RTI_CORE_FOUND TRUE)
LIST(APPEND RTI_FOUND_REQUIRED_VARS RTI_CORE_FOUND)
FIND_LIBRARY(RTI_CORE_LIBRARY NAMES nddscore
  PATHS ${RTI_CHECK_LIBRARY_DIRS})
IF (EXISTS ${RTI_CORE_LIBRARY})
  MESSAGE(STATUS "Found RTI CORE library: ${RTI_CORE_LIBRARY}")
ELSE (EXISTS ${RTI_CORE_LIBRARY})
  RTI_REPORT_NOT_FOUND(
    "Did not find RTI CORE library (required RTI component).")
  SET(RTI_CORE_FOUND FALSE)
ENDIF (EXISTS ${RTI_CORE_LIBRARY})
MARK_AS_ADVANCED(RTI_CORE_LIBRARY)

# Only mark RTI as found if all required components and dependencies
# have been found.
SET(RTI_FOUND TRUE)
FOREACH(REQUIRED_VAR ${RTI_FOUND_REQUIRED_VARS})
  IF (NOT ${REQUIRED_VAR})
    SET(RTI_FOUND FALSE)
  ENDIF (NOT ${REQUIRED_VAR})
ENDFOREACH(REQUIRED_VAR ${RTI_FOUND_REQUIRED_VARS})

IF (RTI_FOUND)
  # Important: The ordering of these libraries is *NOT* arbitrary, as these
  # could potentially be static libraries their link ordering is important.
  LIST(APPEND RTI_LIBRARIES
    ${RTI_CPP_LIBRARY}
    ${RTI_C_LIBRARIES}
    ${RTI_CORE_LIBRARIES})
ENDIF()

# Handle REQUIRED / QUIET optional arguments.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RTI DEFAULT_MSG
  RTI_INCLUDE_DIR RTI_LIBRARIES)
