/*  This example code is public domain.
*/

:- module(demo,
          [ spin/0,
            turtle_listener/0,
            py_listener/0,

            ros_current_topic/2,        % ?Topic,?Type
            ros_type_introspection/2,   % +Type, -Description
            py_talker/0
          ]).

/** <module> Demo rclswi

This file demonstrates the status of the  proof of concept of the rclswi
ROS2 Prolog client library.  To run, run from your ROS2 ws

    swipl -p foreign=install/rclswi/lib src/rclswi/examples/demo.pl

After which one normally starts spinning in a thread using

    ?- spin.

Now we can inspect the ROS2 network using

    ?- ros_current_topic(Topic, Type).
    Topic = '/parameter_events',
    Type = 'rcl_interfaces/msg/ParameterEvent' ;
    Topic = '/rosout',
    Type = 'rcl_interfaces/msg/Log' ;
    Topic = '/turtle1/cmd_vel',
    Type = 'geometry_msgs/msg/Twist' ;
    ...

And inspect a type  using   ros_type_introspection/2  (layout edited for
readability).

    ?- ros_type_introspection('rcl_interfaces/msg/Log', Type).
    Type = log{file:string,
               function:string,
               level:uint8,
               line:uint32,
               msg:string,
               name:string,
               stamp:time{nanosec:uint32,
                          sec:int32}}.
*/

% Load the ROS library and make it available at the toplevel

:- reexport(install/rclswi/prolog/ros).

% This can be used to define the default   node. If this is not done the
% node will be called  swi_prolog_NNN,  where   NNN  is  a  large random
% number.

:- ros_set_defaults(
       [ node(prolog, [])
       ]).

%!  spin
%
%   Spinning in a seperate thread. You may  also subscribe and then spin
%   in the main thread. Spinning in a   thread  is nicer for interactive
%   usage. The thread is _detached_,  which   causes  Prolog  to print a
%   warning in the event that it dies abnormally.

spin :-
    thread_create(ros_spin, _, [detached(true)]).

%!  turtle_listener
%
%   Listen to the turtlesim topic.  To run this demo:
%
%    - Start the turtlesim demo of ROS2 using (in two
%      different terminals):
%      - ros2 run turtlesim turtlesim_node
%      - ros2 run turtlesim turtle_teleop_key
%    - Load this file (see above) and run
%      - ``?- spin.``
%      - ``?- turtle_listener.``
%
%   Now type keys in the `turtle_teleop_key`   terminal. Tht should make
%   the system print messages such as (layout edited)
%
%   ```
%   Turtle asked to do twist{angular:vector3{x:0.0,y:0.0,z:0.0},
%                            linear:vector3{x:2.0,y:0.0,z:0.0}}
%   ```

turtle_listener :-
    ros_subscribe('/turtle1/cmd_vel', on_turtle,
                  [ message_type('geometry_msgs/msg/Twist')
                  ]).

on_turtle(Message) :-
    format('Turtle asked to do ~p~n', [Message]).

%!  py_listener
%
%   This demo does more or less the same as turtle_listener/0, but using
%   the very simple topic `/topic` from   the ROS2 minimal publisher and
%   subscriber example for Python (see link below).
%
%   It should print the same messages  as   the  Python listener of this
%   tutorial.
%
%   @see https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/

py_listener :-
    ros_subscribe('/topic', on_py,
                  [ message_type('std_msgs/msg/String')
                  ]).

on_py(Message) :-
    format('Heart ~p~n', [Message]).


		 /*******************************
		 *           PUBLISHING		*
		 *******************************/

%!  py_talker
%
%   Publish a string to `/topic` every second. This is the same as the
%   node below started from the Python pub/sub tutorial.
%
%       ros2 run py_pubsub talker

py_talker :-
    ros_publisher('/topic',
                  [ message_type('std_msgs/msg/String')
                  ]),
    between(1, infinite, I),
    format(string(Msg), 'Hello World ~d', [I]),
    ros_publish('/topic', _{data: Msg}),
    sleep(1),
    fail.
