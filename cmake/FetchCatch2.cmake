# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Fetch catch2 
find_package(Catch2 3.0.1 QUIET)
cmake_dependent_option(USE_SYSTEM_Catch2 "Use system Catch2" ON "Catch2_FOUND" OFF)
if(NOT USE_SYSTEM_Catch2)
  include(FetchContent)
  FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.0.1)
  FetchContent_GetProperties(Catch2)
  if(NOT Catch2_POPULATED)
    message(STATUS "Fetching Catch2...")
    FetchContent_MakeAvailable(Catch2)
  endif()
endif()
