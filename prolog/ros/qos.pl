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

:- module(ros_qos,
          [ ros_qos_profile/2,                  % +Name, +Dict
            ros_qos_profile/3,                  % +Name, +Dict, +Default
            ros_qos_profile_dict/2,             % ?QoSProfile, ?Dict
            ros_qos_object/2                    % +NameOrDict, -QoSProfile
          ]).
:- use_module(library(ros)).
:- use_module(library(apply)).
:- use_module(library(error)).

/** <module> Handle ROS QoS (Quality of Service) profiles

This library manages ROS Quality  of   Service  profiles. Normally, such
profiles are declared using e.g.

    :- use_module(library(ros/qos)).

    :- ros_qos_profile(vision, _{ history:keep_last,
                                  relability:best_effort}).

After which we can use the profile, for example to listen to a topic:

    ...,
    ros_subscribe('/camera', on_picture, [qos(vision]).
*/

:- dynamic qos_declaration/3 as multifile.      % Name, Dict, Default
:- dynamic qos_object/2 as volatile.            % Name, QoSProfile

qos_declaration(default, _{}, _).

%!  ros_qos_profile(+Name, +Dict) is det.
%!  ros_qos_profile(+Name, +Dict, +Default) is det.
%
%   Declare  a  ROS  QoS  Profile  to  be  associated  with  Name.  This
%   declaration itself does not create the  ROS QoS Profile object. This
%   task is left to ros_qos_object/2. Fields of the profile that are not
%   specified use the default value or  the value from the corresponding
%   Default profile.

ros_qos_profile(Name, Dict) :-
    ros_qos_profile(Name, Dict, _).

ros_qos_profile(Name, Dict, Default) :-
    check_qos_dict(Dict),
    retractall(qos_declaration(Name, _, _)),
    asserta(qos_declaration(Name, Dict, Default)).

check_qos_dict(Dict) :-
    dict_pairs(Dict, _, Pairs),
    maplist(check_qos_field, Pairs).

check_qos_field(Key-Value) :-
    type(Key, Type),
    !,
    check_qos_field_type(Type, Value).
check_qos_field(Key-_) :-
    existence_error(ros_qos_profile_field, Key).

check_qos_field_type(enum(Which), Value) =>
    ros:'$qos_enum_values'(Which, Domain),
    must_be(one_of(Domain), Value).
check_qos_field_type(duration, Value) =>
    must_be(number, Value),
    (   Value >= 0
    ->  true
    ;   domain_error(duration, Value)
    ).
check_qos_field_type(Type, Value) =>
    must_be(Type, Value).

type(history,                         enum(history)).
type(depth,                           nonneg).
type(reliability,                     enum(reliability)).
type(durability,                      enum(durability)).
type(deadline,                        duration).
type(lifespan,                        duration).
type(liveliness,                      enum(liveliness)).
type(liveliness_lease_duration,       duration).
type(avoid_ros_namespace_conventions, boolean).

user:term_expansion((:- ros_qos_profile(Name, Dict)),
                    ros_qos:qos_declaration(Name, Dict, _)) :-
    check_qos_dict(Dict).
user:term_expansion((:- ros_qos_profile(Name, Dict, Default)),
                    ros_qos:qos_declaration(Name, Dict, Default)) :-
    check_qos_dict(Dict).

%!  ros_qos_profile_dict(?QoSProfile, ?Dict) is det.
%
%   Bi-directional translation between a QoSProfile   handle  and a dict
%   representation of a QoS profile. The system defaults can be obtained
%   using
%
%   ```
%   ?- ros_qos_profile_dict(QoS, _{}),
%      ros_qos_profile_dict(QoS, Dict).
%   ```

ros_qos_profile_dict(QoSProfile, Dict) :-
    (   var(QoSProfile)
    ->  ros:'$ros_qos_profile_create'(Dict, QoSProfile, _)
    ;   ros:'$ros_qos_profile_to_prolog'(QoSProfile, Dict)
    ).

%!  ros_qos_object(+NameOrDict, -QoSProfile) is det.
%
%   Get a QoS Prolog handle to be passed  to the low-level ROS API. This
%   predicate is first of all for internal use.
%
%   @arg   NameOrDict   is   either    an     atom    registered   using
%   ros_qos_profile/2,3 or a dict that defines the QoS profile.

ros_qos_object(Name, QoSProfile) :-
    atom(Name),
    !,
    (   qos_object(Name, Object)
    ->  QoSProfile = Object
    ;   with_mutex(ros_qos,
                   create_ros_qos_object(Name, Object)),
        QoSProfile = Object
    ).
ros_qos_object(Dict, QoSProfile) :-
    ros:'$ros_qos_profile_create'(Dict, QoSProfile, _).

create_ros_qos_object(Name, QoSProfile) :-
    qos_object(Name, Object),
    !,
    QoSProfile = Object.
create_ros_qos_object(Name, QoSProfile) :-
    qos_declaration(Name, Decl, Default),
    (   var(Default)
    ->  ros:'$ros_qos_profile_create'(Decl, QoSProfile, _)
    ;   ros_qos_object(Default, DefaultObject),
        ros:'$ros_qos_profile_create'(Decl, QoSProfile, DefaultObject)
    ).
