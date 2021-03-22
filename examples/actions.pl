/*  This example code is public domain.
*/

:- module(actions,
          [ type/0,
            type/1,                     % +ServiceName
            fib/1,
            fib_to/2,
            rotate/1,
            mock_cancel/0,
            server/0
          ]).
:- use_module(library(pprint)).
:- use_module(library(debug)).
:- use_module(library(apply)).
:- use_module(library(ansi_term)).

:- reexport(library(ros)).
:- reexport(library(ros/actions)).
:- reexport(library(ros/services)).

% :- debug(ros(spin)).

/** <module>

Getting to understand actions by using the low-level API.


@see https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Actions.html
*/

:- ros_set_defaults(
       [ node(swi_action, [])
       ]).
:- ros_action(
       '/fibonacci',
       'action_tutorials_interfaces/action/Fibonacci',
       []).
:- ros_action(
       '/turtle1/rotate_absolute',
       'turtlesim/action/RotateAbsolute',
       []).

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


fib(N) :-
    ros_action_run('/fibonacci', _{order:N}, echo, []).

rotate(To) :-
    with_tty_raw(ros_action_run('/turtle1/rotate_absolute', _{theta:To}, echo, [])).

echo -->
    [feedback(_)],
    { poll_char(q),
      !,
      format('Typed Q.  Cancelling ...~n'),
      fail
    }.
echo -->
    [Msg],
    !,
    { format('Got ~p~n', [Msg]) },
    echo.
echo -->
    [].

poll_char(C) :-
    wait_for_input([user_input], Ready, 0),
    Ready = [user_input],
    get_char(user_input, C).

fib_to(N, Max) :-
    ros_action_run('/fibonacci', _{order:N}, on_fib_to(Max), []).

on_fib_to(Max) -->
    [ feedback(Msg) ],
    { last(Msg.partial_sequence, Last),
      Last > Max,
      format('Oops, ~D is too big; requesting cancel~n', [Last]),
      !,
      fail
    }.
on_fib_to(Max) -->
    [Msg],
    !,
    { format('Got ~p~n', [Msg]) },
    on_fib_to(Max).
on_fib_to(_) -->
    [].



mock_cancel :-
    ros_client('/turtle1/rotate_absolute/_action/cancel',
               'action_msgs/srv/CancelGoal',
               Client, []),
    get_time(Now),
    ros_call(Client, _{goal_info:_{stamp:Now}}, Response),
    pp(Response).


		 /*******************************
		 *       SERVER EXPERIMENT	*
		 *******************************/

server :-
    ros_debug(10),
    debug(ros(_)),
    ros_default_context(Context),
    ros:ros_create_clock(Context, system, Clock),
    ros_action_server('/turtle1/rotate_absolute',
                      'turtlesim/action/RotateAbsolute',
                      Server,
                      [ clock(Clock) ]),
    server_loop(Server).

server_loop(Server) :-
    ros_wait([Server], infinite, Ready),
    Ready = [action_server(Server, Goal, Cancel, Result, Expired)],
    server_action(Server, Goal, Cancel, Result, Expired),
    server_loop(Server).

server_action(Server, Goal, Cancel, Result, Expired) :-
    server_action(Goal, Server, goal),
    server_action(Cancel, Server, cancel),
    server_action(Result, Server, result),
    server_action(Expired, Server, expired).

server_action(true, Server, Type) :-
    catch(server_action(Type, Server),
          E,
          print_message(warning, E)).
server_action(false, _, _).

server_action(goal, Server) :-
    ros:ros_action_take_goal_request(Server, Request, Info),
    debug(ros(action), 'Got goal request ~p, info ~p', [Request, Info]),
    get_time(Now),
    ros:ros_action_send_goal_response(Server, _{accepted:true, stamp:Now}, Info),
    run_goal(Request.goal, Server, Request.goal_id).
server_action(cancel, Server) :-
    ros:ros_action_take_cancel_request(Server, Request, Info),
    debug(ros(action), 'Got cancel request ~p, info ~p', [Request, Info]).
server_action(result, Server) :-
    ros:ros_action_take_result_request(Server, Request, Info),
    debug(ros(action), 'Got result request ~p, info ~p', [Request, Info]).
server_action(expired, Server) :-
    debug(ros(action), 'Got expired event', [Server]).

run_goal(Goal, Server, GoalID) :-
    Theta = Goal.theta,
    run(Theta, Server, GoalID).

run(Theta, Server, GoalID) :-
    Theta =< 0,
    set_succeeded(Server, GoalID).
run(Theta, Server, GoalID) :-
    ros:ros_action_publish_feedback(Server, _{feedback:_{remaining:Theta},
                                              goal_id:GoalID}),
    Theta2 is Theta - 1,
    run(Theta2, Server, GoalID).


