# Ceres Solver - A fast non-linear least squares minimizer
# Copyright 2010, 2011, 2012 Google Inc. All rights reserved.
# http://code.google.com/p/ceres-solver/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Google Inc. nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: keir@google.com (Keir Mierle)

cmake_minimum_required(VERSION 2.8.3)
project(ceres)

find_package(catkin REQUIRED COMPONENTS cmake_modules)

# Make CMake aware of the cmake folder for local FindXXX scripts.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Important: Always bump the second number (e.g. 1.3.x to 1.4.0) for any
# release that changes the ABI. The ABI changes for almost any modification to
# include/ceres (e.g. the public API). If you are unsure about whether
# something is an ABI change, please ask on the list.
#
# For versions without ABI changes, bump the smallest number in CERES_VERSION,
# but leave the CERES_ABI_VERSION unchanged.
SET(CERES_VERSION_MAJOR 1)
SET(CERES_VERSION_MINOR 8)
SET(CERES_VERSION_PATCH 0)
SET(CERES_VERSION
    ${CERES_VERSION_MAJOR}.${CERES_VERSION_MINOR}.${CERES_VERSION_PATCH})
SET(CERES_ABI_VERSION 1.8.0)

find_package(Eigen REQUIRED)

OPTION(MINIGLOG "Use a stripped down version of glog." OFF)
OPTION(GFLAGS "Enable Google Flags." ON)
OPTION(SUITESPARSE "Enable SuiteSparse." ON)
OPTION(CXSPARSE "Enable CXSparse." ON)
OPTION(LAPACK "Enable use of LAPACK." ON)
# Template specializations for the Schur complement based solvers. If
# compile time, binary size or compiler performance is an issue, you
# may consider disabling this.
OPTION(SCHUR_SPECIALIZATIONS "Enable fixed-size schur specializations." ON)
OPTION(CUSTOM_BLAS
       "Use handcoded BLAS routines (usually faster) instead of Eigen."
       ON)
# Multithreading using OpenMP
OPTION(OPENMP "Enable threaded solving in Ceres (requires OpenMP)" ON)
# TODO(sameeragarwal): Replace this with a positive option instead?

# Line search minimizer is useful for large scale problems or when
# sparse linear algebra libraries are not available. If compile time,
# binary size or compiler performance is an issue, consider disabling
# this.
OPTION(LINE_SEARCH_MINIMIZER "Enable the line search minimizer." ON)
OPTION(BUILD_TESTING "Enable tests" ON)
OPTION(BUILD_DOCUMENTATION "Build User's Guide (html)" OFF)
OPTION(BUILD_EXAMPLES "Build examples" ON)
OPTION(BUILD_SHARED_LIBS "Build Ceres as a shared library." OFF)
IF (MSVC)
  OPTION(MSVC_USE_STATIC_CRT
    "MS Visual Studio: Use static C-Run Time Library in place of shared." OFF)
ENDIF (MSVC)

# Prior to October 2013, Ceres used some non-CMake standardised variables to
# hold user-specified (as opposed to FindPackage found) include directory and
# library paths for Ceres dependencies.  These were were of the form:
# <DEPENDENCY>_LIB / <DEPENDENCY>_INCLUDE.  Since then, Ceres now has
# FindPackage() scripts for all of its dependencies which obey the standard
# CMake variables: <DEPENDENCY>_LIBRARIES & <DEPENDENCY>_INCLUDE_DIRS.  In order
# to ensure backwards compatibility, we use convert any legacy variables to
# _directory_ hints for the FindPackage() scripts.
MACRO(HANDLE_LEGACY_INCLUDE_DEPENDENCY_HINT
    LEGACY_VAR DIRECTORY_HINT_VAR)
  IF (DEFINED ${LEGACY_VAR})
    # Get the dependency name (all caps) from the hint directory variable
    # for the warning message.
    STRING(REGEX MATCH "^[^_]*" DEPENDENCY_NAME ${DIRECTORY_HINT_VAR})
    MESSAGE(WARNING "You are defining a legacy variable ${LEGACY_VAR} "
      "to specify the include directory for ${DEPENDENCY_NAME}.  This is "
      "deprecated and support for it will be removed in a future release. "
      "Please use either the search directory hints variable: "
      "${DIRECTORY_HINT_VAR} or ${DEPENDENCY_NAME}_INCLUDE_DIR to specify "
      "exactly the directory used (no search performed), see: "
      "http://homes.cs.washington.edu/~sagarwal/ceres-solver/dev/building.html "
      "for more information.")
    LIST(APPEND ${DIRECTORY_HINT_VAR} ${${LEGACY_VAR}})
  ENDIF (DEFINED ${LEGACY_VAR})
ENDMACRO(HANDLE_LEGACY_INCLUDE_DEPENDENCY_HINT)

MACRO(HANDLE_LEGACY_LIBRARY_DEPENDENCY_HINT
    LEGACY_VAR DIRECTORY_HINT_VAR)
  IF (DEFINED ${LEGACY_VAR})
    # Get the dependency name (all caps) from the hint directory variable
    # for the warning message.
    STRING(REGEX MATCH "^[^_]*" DEPENDENCY_NAME ${DIRECTORY_HINT_VAR})
    MESSAGE(WARNING "You are defining a legacy variable ${LEGACY_VAR} "
      "to specify the library for ${DEPENDENCY_NAME}.  This is "
      "deprecated and support for it will be removed in a future release. "
      "Please use either the search directory hints variable: "
      "${DIRECTORY_HINT_VAR} or ${DEPENDENCY_NAME}_LIBRARY to specify "
      "exactly the library used (no search performed), see: "
      "http://homes.cs.washington.edu/~sagarwal/ceres-solver/dev/building.html "
      "for more information.")
    IF (EXISTS ${${LEGACY_VAR}} AND
        NOT IS_DIRECTORY ${${LEGACY_VAR}})
      # User specified an explicit (library) file using the legacy variable
      # interface, hints to FindPackage() scripts are directories so add the
      # parent directory of the specified file.
      GET_FILENAME_COMPONENT(DIR_HINT ${${LEGACY_VAR}} PATH)
      LIST(APPEND ${DIRECTORY_HINT_VAR} ${DIR_HINT})
    ELSEIF (EXISTS ${${LEGACY_VAR}} AND
            IS_DIRECTORY ${${LEGACY_VAR}})
      # User specified a directory hint using the legacy variable, use it.
      LIST(APPEND ${DIRECTORY_HINT_VAR} ${${LEGACY_VAR}})
    ENDIF()
  ENDIF (DEFINED ${LEGACY_VAR})
ENDMACRO(HANDLE_LEGACY_LIBRARY_DEPENDENCY_HINT)

# LAPACK (& BLAS).
IF (LAPACK)
  FIND_PACKAGE(LAPACK QUIET)
  IF (LAPACK_FOUND)
    MESSAGE("-- Found LAPACK library: ${LAPACK_LIBRARIES}")
  ELSE (LAPACK_FOUND)
    MESSAGE("-- Did not find LAPACK library, disabling LAPACK support.")
  ENDIF (LAPACK_FOUND)

  FIND_PACKAGE(BLAS QUIET)
  IF (BLAS_FOUND)
    MESSAGE("-- Found BLAS library: ${BLAS_LIBRARIES}")
  ELSE (BLAS_FOUND)
    MESSAGE("-- Did not find BLAS library, disabling LAPACK support.")
  ENDIF (BLAS_FOUND)

  IF (NOT (LAPACK_FOUND AND BLAS_FOUND))
    # Retain the help string associated with the LAPACK option
    # when updating it to disable use of LAPACK.
    GET_PROPERTY(HELP_STRING CACHE LAPACK PROPERTY HELPSTRING)
    SET(LAPACK OFF CACHE BOOL "${HELP_STRING}" FORCE)
    ADD_DEFINITIONS(-DCERES_NO_LAPACK)
  ENDIF (NOT (LAPACK_FOUND AND BLAS_FOUND))
ELSE (LAPACK)
  MESSAGE("-- Building without LAPACK.")
  ADD_DEFINITIONS(-DCERES_NO_LAPACK)
ENDIF (LAPACK)

# SuiteSparse.
IF (SUITESPARSE AND NOT LAPACK)
  # If user has disabled LAPACK, but left SUITESPARSE ON, turn it OFF,
  # LAPACK controls whether Ceres will be linked, directly or indirectly
  # via SuiteSparse to LAPACK.
  MESSAGE("-- Disabling SuiteSparse as use of LAPACK has been disabled, "
    "turn ON LAPACK to enable (optional) building with SuiteSparse.")
  # Retain the help string associated with the SUITESPARSE option
  # when updating it to disable use of SuiteSparse.
  GET_PROPERTY(HELP_STRING CACHE SUITESPARSE PROPERTY HELPSTRING)
  SET(SUITESPARSE OFF CACHE BOOL "${HELP_STRING}" FORCE)
ENDIF (SUITESPARSE AND NOT LAPACK)
IF (SUITESPARSE)
  # By default, if SuiteSparse and all dependencies are found, Ceres is
  # built with SuiteSparse support.

  # Check for SuiteSparse and dependencies.
  FIND_PACKAGE(SuiteSparse)
  IF (SUITESPARSE_FOUND)
    # On Ubuntu the system install of SuiteSparse (v3.4.0) up to at least
    # Ubuntu 13.10 cannot be used to link shared libraries.
    IF (BUILD_SHARED_LIBS AND
        SUITESPARSE_IS_BROKEN_SHARED_LINKING_UBUNTU_SYSTEM_VERSION)
      MESSAGE(FATAL_ERROR "You are attempting to build Ceres as a shared "
        "library on Ubuntu using a system package install of SuiteSparse "
        "3.4.0. This package is broken and does not support the "
        "construction of shared libraries (you can still build Ceres as "
        "a static library).  If you wish to build a shared version of Ceres "
        "you should uninstall the system install of SuiteSparse "
        "(libsuitesparse-dev) and perform a source install of SuiteSparse "
        "(we recommend that you use the latest version), "
        "see: http://homes.cs.washington.edu/~sagarwal"
        "/ceres-solver/dev/building.html for more information.")
    ENDIF (BUILD_SHARED_LIBS AND
      SUITESPARSE_IS_BROKEN_SHARED_LINKING_UBUNTU_SYSTEM_VERSION)

    # By default, if all of SuiteSparse's dependencies are found, Ceres is
    # built with SuiteSparse support.
    MESSAGE("-- Found SuiteSparse ${SUITESPARSE_VERSION}, "
            "building with SuiteSparse.")
  ELSE (SUITESPARSE_FOUND)
    # Disable use of SuiteSparse if it cannot be found and continue.
    MESSAGE("-- Did not find all SuiteSparse dependencies, disabling "
      "SuiteSparse support.")
    # Retain the help string associated with the SUITESPARSE option
    # when updating it to disable use of SuiteSparse.
    GET_PROPERTY(HELP_STRING CACHE SUITESPARSE PROPERTY HELPSTRING)
    SET(SUITESPARSE OFF CACHE BOOL "${HELP_STRING}" FORCE)
    ADD_DEFINITIONS(-DCERES_NO_SUITESPARSE)
  ENDIF (SUITESPARSE_FOUND)
ELSE (SUITESPARSE)
  MESSAGE("-- Building without SuiteSparse.")
  ADD_DEFINITIONS(-DCERES_NO_SUITESPARSE)
ENDIF (SUITESPARSE)

# CXSparse.
IF (CXSPARSE)
  # Don't search with REQUIRED as we can continue without CXSparse.
  FIND_PACKAGE(CXSparse)
  IF (CXSPARSE_FOUND)
    # By default, if CXSparse and all dependencies are found, Ceres is
    # built with CXSparse support.
    MESSAGE("-- Found CXSparse version: ${CXSPARSE_VERSION}, "
      "building with CXSparse.")
  ELSE (CXSPARSE_FOUND)
    # Disable use of CXSparse if it cannot be found and continue.
    MESSAGE("-- Did not find CXSparse, Building without CXSparse.")
    # Retain the help string associated with the CXSPARSE option
    # when updating it to disable use of CXSparse.
    GET_PROPERTY(HELP_STRING CACHE CXSPARSE PROPERTY HELPSTRING)
    SET(CXSPARSE OFF CACHE BOOL "${HELP_STRING}" FORCE)
    ADD_DEFINITIONS(-DCERES_NO_CXSPARSE)
  ENDIF (CXSPARSE_FOUND)
ELSE (CXSPARSE)
  MESSAGE("-- Building without CXSparse.")
  ADD_DEFINITIONS(-DCERES_NO_CXSPARSE)
  # Mark as advanced (remove from default GUI view) the CXSparse search
  # variables in case user enabled CXSPARSE, FindCXSparse did not find it, so
  # made search variables visible in GUI for user to set, but then user disables
  # CXSPARSE instead of setting them.
  MARK_AS_ADVANCED(FORCE CXSPARSE_INCLUDE_DIR
                         CXSPARSE_LIBRARY)
ENDIF (CXSPARSE)

# GFlags.
IF (GFLAGS)
  HANDLE_LEGACY_INCLUDE_DEPENDENCY_HINT(GFLAGS_INCLUDE GFLAGS_INCLUDE_DIR_HINTS)
  HANDLE_LEGACY_LIBRARY_DEPENDENCY_HINT(GFLAGS_LIB GFLAGS_LIBRARY_DIR_HINTS)

  # Don't search with REQUIRED as we can continue without gflags.
  FIND_PACKAGE(Gflags)
  IF (GFLAGS_FOUND)
    MESSAGE("-- Found Google Flags header in: ${GFLAGS_INCLUDE_DIRS}")
  ELSE (GFLAGS_FOUND)
    MESSAGE("-- Did not find Google Flags (gflags), Building without gflags "
      "- no tests or tools will be built!")
    # Retain the help string associated with the GFLAGS option
    # when updating it to disable use of gflags.
    GET_PROPERTY(HELP_STRING CACHE GFLAGS PROPERTY HELPSTRING)
    SET(GFLAGS OFF CACHE BOOL "${HELP_STRING}" FORCE)
    ADD_DEFINITIONS(-DCERES_NO_GFLAGS)
  ENDIF (GFLAGS_FOUND)
ELSE (GFLAGS)
  MESSAGE("-- Google Flags disabled; no tests or tools will be built!")
  ADD_DEFINITIONS(-DCERES_NO_GFLAGS)
  # Mark as advanced (remove from default GUI view) the gflags search
  # variables in case user enabled GFLAGS, FindGflags did not find it, so
  # made search variables visible in GUI for user to set, but then user disables
  # GFLAGS instead of setting them.
  MARK_AS_ADVANCED(FORCE GFLAGS_INCLUDE_DIR
                         GFLAGS_LIBRARY)
ENDIF (GFLAGS)

# MiniGLog.
IF (MINIGLOG)
  SET(GLOG_LIBRARIES miniglog)
  MESSAGE("-- Using minimal Glog substitute (library): ${GLOG_LIBRARIES}")
  SET(GLOG_INCLUDE_DIRS internal/ceres/miniglog)
  MESSAGE("-- Using minimal Glog substitute (include): ${GLOG_INCLUDE_DIRS}")

  # Mark as advanced (remove from default GUI view) the glog search
  # variables in case user disables MINIGLOG, FindGlog did not find it, so
  # made search variables visible in GUI for user to set, but then user enables
  # MINIGLOG instead of setting them.
  MARK_AS_ADVANCED(FORCE GLOG_INCLUDE_DIR
                         GLOG_LIBRARY)
ELSE (MINIGLOG)
  HANDLE_LEGACY_INCLUDE_DEPENDENCY_HINT(GLOG_INCLUDE GLOG_INCLUDE_DIR_HINTS)
  HANDLE_LEGACY_LIBRARY_DEPENDENCY_HINT(GLOG_LIB GLOG_LIBRARY_DIR_HINTS)

  # Don't search with REQUIRED so that configuration continues if not found and
  # we can output an error messages explaining MINIGLOG option.
  FIND_PACKAGE(Glog)
  IF (GLOG_FOUND)
    MESSAGE("-- Found Google Log header in: ${GLOG_INCLUDE_DIRS}")
  ELSE (GLOG_FOUND)
    MESSAGE(FATAL_ERROR "Can't find Google Log. Please set GLOG_INCLUDE_DIR & "
      "GLOG_LIBRARY or enable MINIGLOG option to use minimal glog "
      "implementation.")
  ENDIF (GLOG_FOUND)
ENDIF (MINIGLOG)

IF (NOT SCHUR_SPECIALIZATIONS)
  ADD_DEFINITIONS(-DCERES_RESTRICT_SCHUR_SPECIALIZATION)
  MESSAGE("-- Disabling Schur specializations (faster compiles)")
ENDIF (NOT SCHUR_SPECIALIZATIONS)

IF (NOT LINE_SEARCH_MINIMIZER)
  ADD_DEFINITIONS(-DCERES_NO_LINE_SEARCH_MINIMIZER)
  MESSAGE("-- Disabling line search minimizer")
ENDIF (NOT LINE_SEARCH_MINIMIZER)

IF (NOT CUSTOM_BLAS)
  ADD_DEFINITIONS(-DCERES_NO_CUSTOM_BLAS)
  MESSAGE("-- Disabling custom blas")
ENDIF (NOT CUSTOM_BLAS)

IF (OPENMP)
  # Clang does not (yet) support OpenMP.
  IF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Retain the help string associated with the OPENMP option
    # when updating it to disable use of OPENMP.
    GET_PROPERTY(HELP_STRING CACHE OPENMP PROPERTY HELPSTRING)
    SET(OPENMP OFF CACHE BOOL "${HELP_STRING}" FORCE)
    MESSAGE("-- Compiler is Clang, disabling OpenMP.")
    ADD_DEFINITIONS(-DCERES_NO_THREADS)
  ELSE (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Find quietly s/t as we can continue without OpenMP if it is not found.
    FIND_PACKAGE(OpenMP QUIET)
    IF (OPENMP_FOUND)
      MESSAGE("-- Building with OpenMP.")
      ADD_DEFINITIONS(-DCERES_USE_OPENMP)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      IF (UNIX)
        # At least on Linux, we need pthreads to be enabled for mutex to
        # compile.  This may not work on Windows or Android.
        FIND_PACKAGE(Threads REQUIRED)
        SET(STATIC_LIBRARY_FLAGS
          "${STATIC_LIBRARY_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
        SET(CMAKE_SHARED_LINKER_FLAGS
          "${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
        ADD_DEFINITIONS(-DCERES_HAVE_PTHREAD)
        ADD_DEFINITIONS(-DCERES_HAVE_RWLOCK)
      ENDIF (UNIX)
    ELSE (OPENMP_FOUND)
      MESSAGE("-- Failed to find OpenMP, disabling.")
      # Retain the help string associated with the OPENMP option
      # when updating it to disable use of OPENMP.
      GET_PROPERTY(HELP_STRING CACHE OPENMP PROPERTY HELPSTRING)
      SET(OPENMP OFF CACHE BOOL "${HELP_STRING}" FORCE)
      ADD_DEFINITIONS(-DCERES_NO_THREADS)
    ENDIF (OPENMP_FOUND)
  ENDIF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
ELSE (OPENMP)
  MESSAGE("-- Building without OpenMP (disabling multithreading).")
  ADD_DEFINITIONS(-DCERES_NO_THREADS)
ENDIF (OPENMP)

Include(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX(unordered_map UNORDERED_MAP_IN_STD_NAMESPACE)
If (UNORDERED_MAP_IN_STD_NAMESPACE)
  ADD_DEFINITIONS(-DCERES_STD_UNORDERED_MAP)
ELSE (UNORDERED_MAP_IN_STD_NAMESPACE)
  CHECK_INCLUDE_FILE_CXX("tr1/unordered_map" UNORDERED_MAP_IN_TR1_NAMESPACE)
  IF (UNORDERED_MAP_IN_TR1_NAMESPACE)
    ADD_DEFINITIONS(-DCERES_TR1_UNORDERED_MAP)
  ELSE (UNORDERED_MAP_IN_TR1_NAMESPACE)
    MESSAGE("-- Unable to find <unordered_map> or <tr1/unordered_map>. ")
    MESSAGE("-- Replacing unordered_map/set with map/set (warning: slower!)")
    ADD_DEFINITIONS(-DCERES_NO_UNORDERED_MAP)
  ENDIF (UNORDERED_MAP_IN_TR1_NAMESPACE)
ENDIF (UNORDERED_MAP_IN_STD_NAMESPACE)

include_directories(
  include
  internal
  internal/ceres
  ${GLOG_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

IF (SUITESPARSE)
  INCLUDE_DIRECTORIES(${SUITESPARSE_INCLUDE_DIRS})
ENDIF (SUITESPARSE)

IF (CXSPARSE)
  INCLUDE_DIRECTORIES(${CXSPARSE_INCLUDE_DIRS})
ENDIF (CXSPARSE)

IF (GFLAGS)
  INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})
ENDIF (GFLAGS)

# After the tweaks for the compile settings, disable some warnings on MSVC.
IF (MSVC)
  # Disable signed/unsigned int conversion warnings.
  ADD_DEFINITIONS("/wd4018")
  # Disable warning about using struct/class for the same symobl.
  ADD_DEFINITIONS("/wd4099")
  # Disable warning about the insecurity of using "std::copy".
  ADD_DEFINITIONS("/wd4996")
  # Disable performance warning about int-to-bool conversion.
  ADD_DEFINITIONS("/wd4800")
  # Disable performance warning about fopen insecurity.
  ADD_DEFINITIONS("/wd4996")
  # Disable warning about int64 to int32 conversion. Disabling
  # this warning may not be correct; needs investigation.
  # TODO(keir): Investigate these warnings in more detail.
  ADD_DEFINITIONS("/wd4244")
  # It's not possible to use STL types in DLL interfaces in a portable and
  # reliable way. However, that's what happens with Google Log and Google Flags
  # on Windows. MSVC gets upset about this and throws warnings that we can't do
  # much about. The real solution is to link static versions of Google Log and
  # Google Test, but that seems tricky on Windows. So, disable the warning.
  ADD_DEFINITIONS("/wd4251")

  # Google Flags doesn't have their DLL import/export stuff set up correctly,
  # which results in linker warnings. This is irrelevant for Ceres, so ignore
  # the warnings.
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ignore:4049")

  # Update the C/CXX flags for MSVC to use either the static or shared
  # C-Run Time (CRT) library based on the user option: MSVC_USE_STATIC_CRT.
  LIST(APPEND C_CXX_FLAGS
    CMAKE_CXX_FLAGS
    CMAKE_CXX_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE
    CMAKE_CXX_FLAGS_MINSIZEREL
    CMAKE_CXX_FLAGS_RELWITHDEBINFO)

  FOREACH(FLAG_VAR ${C_CXX_FLAGS})
    IF (MSVC_USE_STATIC_CRT)
      # Use static CRT.
      IF (${FLAG_VAR} MATCHES "/MD")
        STRING(REGEX REPLACE "/MD" "/MT" ${FLAG_VAR} "${${FLAG_VAR}}")
      ENDIF (${FLAG_VAR} MATCHES "/MD")
    ELSE (MSVC_USE_STATIC_CRT)
      # Use shared, not static, CRT.
      IF (${FLAG_VAR} MATCHES "/MT")
        STRING(REGEX REPLACE "/MT" "/MD" ${FLAG_VAR} "${${FLAG_VAR}}")
      ENDIF (${FLAG_VAR} MATCHES "/MT")
    ENDIF (MSVC_USE_STATIC_CRT)
  ENDFOREACH()

  # Tuple sizes of 10 are used by Gtest.
  ADD_DEFINITIONS("-D_VARIADIC_MAX=10")
ENDIF (MSVC)

IF (UNIX)
  # GCC is not strict enough by default, so enable most of the warnings.
  SET(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wno-unknown-pragmas -Wno-sign-compare -Wno-unused-parameter -Wno-missing-field-initializers")
ENDIF (UNIX)

# Use a larger inlining threshold for Clang, since it hobbles Eigen,
# resulting in an unreasonably slow version of the blas routines. The
# -Qunused-arguments is needed because CMake passes the inline
# threshold to the linker and clang complains about it and dies.
IF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Qunused-arguments -mllvm -inline-threshold=600")
  # Older versions of Clang (<= 2.9) do not support the 'return-type-c-linkage'
  # option, so check for its presence before adding it to the default flags set.
  INCLUDE(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-Wno-return-type-c-linkage"
                          HAVE_RETURN_TYPE_C_LINKAGE)
  IF (HAVE_RETURN_TYPE_C_LINKAGE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-return-type-c-linkage")
  ENDIF(HAVE_RETURN_TYPE_C_LINKAGE)
ENDIF ()

# Xcode 4.5.x used Clang 4.1 (Apple version), this has a bug that prevents
# compilation of Ceres.
IF (APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER}
    ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
    OUTPUT_VARIABLE CLANG_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  # Use version > 4.0 & < 4.2 to catch all 4.1(.x) versions.
  IF (CLANG_VERSION VERSION_GREATER 4.0 AND
      CLANG_VERSION VERSION_LESS 4.2)
    MESSAGE(FATAL_ERROR "You are attempting to build Ceres on OS X using Xcode "
      "4.5.x (Clang version: ${CLANG_VERSION}). This version of Clang has a "
      "bug that prevents compilation of Ceres, please update to "
      "Xcode >= 4.6.3.")
  ENDIF (CLANG_VERSION VERSION_GREATER 4.0 AND
    CLANG_VERSION VERSION_LESS 4.2)
ENDIF (APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ceres
  DEPENDS eigen suitesparse
)

add_subdirectory(internal/ceres)
