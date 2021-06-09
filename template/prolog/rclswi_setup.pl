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

:- module(rclswi_setup,
          []).
:- use_module(library(lists), [member/2]).

/** <module> Setup the environment to use rclswi

This module should be loaded as first from a Prolog file that is used as
a ROS2 executable.  Its  task  is  to   find  `rclswi`  and  add  it  to
SWI-Prolog's  library  path  as  well  as    to  setup  Prolog  specific
commandline arguments. Currently it allows for the following arguments:

  - ``--debug=Topic``
    Enable the SWI-Prolog debug topic Topic.  Topic is parsed
    as Prolog if it is valid syntax.  Otherwise it is used as
    an atom.
  - ``--spy=PI``
    Use tspy/1 to set a spy point on PI.  This requires the node to
    have access to graphics.
*/

:- initialization(rclswi_setup).

rclswi_setup :-
    add_ros_library_path.

user:file_search_path(ament_prefix, Dir) :-
    getenv('AMENT_PREFIX_PATH', Path),
    atomic_list_concat(Dirs, ':', Path),
    member(Dir, Dirs).

%!  add_ros_library_path
%
%   Make sure `ros.pl` is in the `library` search path.

add_ros_library_path :-
    absolute_file_name(library(ros), _,
                       [ file_type(prolog),
                         access(read),
                         file_errors(fail)
                       ]),
    !.
add_ros_library_path :-
    absolute_file_name(ament_prefix(prolog/ros), RosLib,
                       [ file_type(prolog),
                         access(read)
                       ]),
    file_directory_name(RosLib, Dir),
    debug(ros(path), 'Found rclswi library at ~p', [Dir]),
    asserta(user:file_search_path(library, Dir)).
