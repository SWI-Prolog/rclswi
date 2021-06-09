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

:- module(ros_pkg,
          [ ros_use_package/1
          ]).
:- use_module(library(ros)).

%!  ros_use_package(+Package)
%
%   Find the ROS package Package and extend  the Prolog file search with
%   Package such that Prolog files of the package may be loaded using:
%
%       :- use_module(Package(File)).

ros_use_package(Package) :-
    Term =.. [Package, '.'],
    absolute_file_name(Term, _,
                       [ file_errors(fail),
                         access(read)
                       ]),
    !.
ros_use_package(Package) :-
    ros_package_share_directory(Package, ShareDir),
    format(atom(Dir), '~w/prolog/~w', [ShareDir, Package]),
    asserta(user:file_search_path(Package, Dir)).
