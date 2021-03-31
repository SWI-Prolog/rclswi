/*  This example code is public domain.
*/

:- module(turtlesim_teleop,
          [ turtlesim_teleop/0
          ]).
:- use_module(library(ansi_term)).
:- use_module(library(broadcast)).
:- use_module(library(lists)).

:- use_module(library(ros)).
:- use_module(library(ros/action/client)).
:- use_module(library(ros/param/store)).
:- use_module(library(ros/logging)).

/** <module> SWI-Prolog implementation of turtlesim_teleop

This demo implements the same   functionality as the `turtle_teleop_key`
executable of the `turtlesim` and illustrates several aspects of the ROS
client API:

  - Declare and use parameters. The parameters can be viewed and changed
    using e.g. `rqt`.  The changes take immediate effect.
  - Publish on `/turtle1/cmd_vel` to rotate and move the turtle.
  - Run an action client to change the heading of the turtle that can
    be interrupted.
  - Log ROS events.

First, launch the turtlesim demo using

    ros2 run turtlesim turtlesim_node

Next, start this using

    swipl -p library=install/prolog src/rclswi/examples/turtlesim_teleop.pl
*/

% Create our default node and define the two parameters.

:- ros_set_defaults(
       [ node(swi_teleop,
              [ parameters([ scale_angular - [ type(double),
                                               default(2.0)
                                             ],
                             scale_linear  - [ type(double),
                                               default(2.0)
                                             ]
                           ])
              ])
       ]).

% Declare the action and publisher

:- ros_action(
       '/turtle1/rotate_absolute',
       'turtlesim/action/RotateAbsolute',
       []).
:- ros_publisher('/turtle1/cmd_vel',
                 [ message_type('geometry_msgs/msg/Twist')
                 ]).

% spin the node.  Actually this is only required for the parameter
% services.

:- initialization(ros_spin([thread(spinner)]), program).

% Start the program. You can also only _load_ the program usingm the
% `-l` option:

% swipl -p library=install/prolog -l src/rclswi/examples/turtlesim_teleop.pl

:- initialization(turtlesim_teleop, main).

%!  turtlesim_teleop
%
%   Runs the main loop, asking for user commands and executing them.

turtlesim_teleop :-
    with_tty_raw(teleop_raw).

teleop_raw :-
    format('~N'),
    format('Twist using the arrow keys~n'),
    format('Please type one of G,T,R,E,D,C,V,B or F to cancel~n'),
    format('Quit using Q.  Update code (make) using M~n'),
    poll_char(C, infinite),
    teleop_raw(C).

teleop_raw(q) =>
    true.
teleop_raw(m) =>
    make,
    teleop_raw.
teleop_raw(C), twist(C, Linear, Angular, Comment) =>
    ros_log(info, 'Ask turtle to ~s', [Comment]),
    ros_get_param(scale_angular, AScale),
    ros_get_param(scale_linear, LScale),
    L is LScale*Linear,
    A is AScale*Angular,
    ros_publish('/turtle1/cmd_vel', _{linear:_{x:L}, angular:_{z:A}}),
    teleop_raw.
teleop_raw(C), action_angle(C, Angle, Comment) =>
    Degrees is round(Angle*180/pi),
    ros_log(info, 'Ask turtle to head ~p (~p\u00b0)~n', [Comment, Degrees]),
    !,
    ros_action_run('/turtle1/rotate_absolute',
                   _{theta:Angle}, echo, []),
    teleop_raw.
teleop_raw(C) =>
    format('Unknown command: ~p~n', [C]),
    teleop_raw.

%!  echo//
%
%   This is the DCG (Grammar) called  by ros_action_run/4 to monitor the
%   ongoing goal. In this case we watch  for   the  'F' key to cancel by
%   making the grammar fail.  Otherwise  we   simple  echo  the event we
%   received.

echo -->
    [feedback(_)],
    { poll_char(f, 0),
      !,
      format('~NTyped F.  Cancelling ...~n'),
      fail
    }.
echo -->
    [Msg],
    !,
    { format(user_error, '\r\e[2KGot ~p', [Msg]) },
    echo.
echo -->
    [].

%!  action_angle(+Key, -Angle, -Comment)
%
%   Define the various headings that may   be selected using keys around
%   the `F` key.

action_angle(g,  0.0,    "east").
action_angle(t,  0.7854, "northeast").
action_angle(r,  1.5708, "north").
action_angle(e,  2.3562, "northwest").
action_angle(d,  3.1416, "west").
action_angle(c, -2.3562, "southwest").
action_angle(v, -1.5708, "south").
action_angle(b, -0.7854, "southeast").

%!  twist(+Arrow, -Linear, -Angular, -Comment)

twist(left,   0.0,  1.0, "turn clockwise").
twist(right,  0.0, -1.0, "turn anti-clockwise").
twist(up,     1.0,  0.0, "move forward").
twist(down,  -1.0,  0.0, "move backward").

%!  poll_char(-Char, +Timeout) is semidet.
%
%   Wait for a command.  Decodes the keyboard arrow keys.

poll_char(C, TimeOut) :-
    wait_for_input([user_input], Ready, TimeOut),
    Ready = [user_input],
    get_char(user_input, C0),
    read_arrow(C0, C).

read_arrow('\e', Arrow) =>
    get_char(user_input, '['),
    get_char(user_input, AD),
    arrow(AD, Arrow).
read_arrow(C, Char) => Char = C.

arrow('A', Arrow) => Arrow = up.
arrow('B', Arrow) => Arrow = down.
arrow('C', Arrow) => Arrow = right.
arrow('D', Arrow) => Arrow = left.

% watch for parameter events.  This  can  be   used  if  some  action is
% required on a parameter change. This is not the case for this demo and
% this code is therefore not required for this demo to work.

:- listen(ros_parameter_events(Node, Events),
          print_parameter_event(Node, Events)).

print_parameter_event(Node, Events) :-
    ansi_format([bold], '~NParameter events on node ~p~n', [Node]),
    forall(member(Event, Events),
           format('  ~p~n', [Event])).

