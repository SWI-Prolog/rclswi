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

:- module(ros_clocks,
          [ ros_create_clock/2,     % +Type, -Clock,
            ros_clock_time/2,       % +Clock, -Type
            ros_clock_property/2    % +Clock, ?Property
          ]).
:- use_module(library(ros)).

/** <module> ROS clock interface

This library provides an interface to ROS clocks.

@tbd Most work. The current version merely supports creating action
servers.
*/

%!  ros_create_clock(+Type, -Clock) is det.
%
%   Create a ROS clock  object.  Type  is   one  of  `ros`,  `system` or
%   `steady`.

ros_create_clock(Type, Clock) :-
    ros_default_context(Context),
    ros_create_clock(Context, Type, Clock).

%!  ros_clock_property(+Clock, ?Property) is nondet.
%
%   True when Property is a property of Clock. Defined properties are:
%
%     - context(-Context)
%       Context in which the clock was created
%     - type(-Type)
%       Typed used to create the clock.
%     - time(-Stamp)
%       Time stamp for the clock.  Same as ros_clock_time/2.

ros_clock_property(Clock, Property) :-
    ros_clock_prop(Property, Clock).

ros_clock_prop(context(Context), Clock) :-
    '$ros_clock_prop'(Clock, type, Context).
ros_clock_prop(type(Type), Clock) :-
    '$ros_clock_prop'(Clock, type, Type).
ros_clock_prop(time(Time), Clock) :-
    ros_clock_time(Clock, Time).

%!  ros_clock_time(+Clock, -Stamp:float)
%
%   Get the time of Clock as a floating point time stamp.

