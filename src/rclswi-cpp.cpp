/* Copyright 2021 SWI-Prolog Solutions b.v.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http:  www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#define PROLOG_MODULE "ros"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <SWI-cpp.h>

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
This module provides functionality that  is   only  accessible using C++
code.
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

PREDICATE(ros_package_share_directory, 2)
{ std::string pkg((char *)A1);
  std::string dir;

  try
  { dir = ament_index_cpp::get_package_share_directory(pkg);
  } catch(...)				/* other errors? */
  { return PL_existence_error("ros_package", A1);
  }

  return PL_unify_chars(A2, PL_ATOM|REP_MB, (size_t)-1, dir.c_str());
}
