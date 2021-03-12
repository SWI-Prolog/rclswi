/*  This example code is public domain.
*/

:- module(actions,
          [ type/0,
            type/1                      % +ServiceName
          ]).
:- use_module(library(pprint)).
:- use_module(library(debug)).

:- reexport(library(ros)).

/** <module>

@see https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Actions.html
*/

:- ros_set_defaults(
       [ node(swi_action, [])
       ]).

%!  type is det.
%!  type(+ActionName) is det.
%
%   Fetch and show the goal, result and   feedback  message types for an
%   action.

type :-
    type('turtlesim/action/RotateAbsolute').

type(Name) :-
    ros_type_introspection(Name, Type),
    print_term(Type, []).
