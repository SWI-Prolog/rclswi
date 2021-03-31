/*  This example code is public domain.
*/

:- module(turtlesim_node,
          [ server/0
          ]).
:- use_module(library(ros)).
:- use_module(library(ros/action/server)).
:- use_module(library(ros/param/store)).
:- use_module(library(ros/logging)).

/** <module> CLI Turtlesim action server

This module implements the action server for   a CLI turtlesum node. The
server is started using

    swipl -p library=install/prolog src/rclswi/examples/turtlesim_node.pl

By default is it has a stepper motor   for the rotation that has only 42
positions and runs at 10 steps/sec. The   defaults  can be changed using
the node parameters, e.g, the  following   creates  a stepper motor with
3,600 steps running at 100Hz:

    ... --ros-args -p steps:=3600 -p step_hz:=100.0

The demo server may be operated   by the `turtlesim_teleop.pl` client or
using the ros2 turtlesim demo.
*/

:- ros_set_defaults(
       [ node(swi_turtlesim,
              [ parameters([ steps   - [ type(integer),
                                         default(42),
                                         comment("Number of steps our motor")
                                       ],
                             step_hz - [ type(double),
                                         default(10.0),
                                         comment("Steps per second")
                                       ]
                           ])
              ])
       ]).

:- initialization(server, main).

%!  server
%
%   Create and run the server.   The  ros_action_server/5 call registers
%   the action server. The ros_action_spin/1 call adds the server to the
%   waitable objects of the node.

server :-
    ros_action_server('/turtle1/rotate_absolute',
                      'turtlesim/action/RotateAbsolute',
                      turtlesim,
                      Server,
                      []),
    ros_action_spin(Server),
    ros_get_param(steps, Steps),
    ros_get_param(step_hz, StepHz),
    ros_log(info, 'Server ready.  ~D Steps, ~wHz', [Steps, StepHz]),
    ros_spin.

:- dynamic
    angle/1.

angle(0).

%!  turtlesim(+Goal, -Result) is det.
%
%   This is the callback from ros_action_server/5. Its first argument is
%   a dict representing the goal and its second argument must be unified
%   with   the   result.   During    the     execution    we   can   use
%   ros_action_feedback/1 to send feedback messages.
%
%   When a cancel request is  received  by   the  server  it  raises the
%   exception ros(cancel). If no action is required to cancel the action
%   than nothing needs to be done. Below, ros_action_on_cancel/3 is used
%   to trap the  exception,  This  calls   cancelled/2  below  and  then
%   re-throws the exception.

turtlesim(Goal, _{delta:Delta}) :-
    angle(Here),
    Target = Goal.theta,
    ros_get_param(steps, Steps),
    StepSize is (2*pi/Steps)*sign(Target-Here),
    ros_log(info, 'Goal to theta=~p; now ~p; StepSize=~p',
            [Target, Here, StepSize]),
    ros_action_on_cancel(
        step(Target, StepSize),
        Why,
        cancelled(Why, Target)),
    delta(Target, Delta),
    ros_log(info, 'Goal completed.  Delta = ~p', [Delta]).

%!  delta(+Target, -Delta) is det.
%
%   Computes the delta to the target.

delta(Target, Delta) :-
    angle(Here),
    Delta is Target-Here.

%!  step(+Target, +StepSize) is det.
%
%   Step to Target using steps of StepSize.

step(Target, StepSize) :-
    transaction(do_step(StepSize, New)),
    Remaining is Target-New,
    (   abs(Remaining) =< abs(StepSize)/2
    ->  true
    ;   ros_action_feedback(_{remaining:Remaining}),
        step(Target, StepSize)
    ).

do_step(StepSize, New) :-
    ros_get_param(step_hz, StepHz),
    Sleep is 1/StepHz,
    sleep(Sleep),
    (   retract(angle(Here))
    ->  New is Here + StepSize
    ;   New is StepSize
    ),
    asserta(angle(New)).

cancelled(Why, Target) :-
    delta(Target, Delta),
    ros_log(warn, 'Goal cancelled (~p) with Delta = ~p', [Why, Delta]),
    throw(ros(cancel)).
