/*  This example code is public domain.
*/

:- module(actions,
          [ type/0,
            type/1,                     % +ServiceName
            client/2,
            rotate/2,
            echo_feedback/0,
            order/2,
            spin/1
          ]).
:- use_module(library(pprint)).
:- use_module(library(debug)).
:- use_module(library(apply)).
:- use_module(library(ansi_term)).

:- reexport(library(ros)).
:- reexport(library(ros/actions)).

:- debug(ros(spin)).

/** <module>

Getting to understand actions by using the low-level API.


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

%!  echo_feedback
%
%   This works.  Type info is
%     rotate_absolute__feedback_message{feedback:rotate_absolute__feedback{remaining:float},
%					goal_id:uuid{uuid:list(uint8, 16)}}

echo_feedback :-
    ros_subscribe('/turtle1/rotate_absolute/_action/feedback',
                  on_feedback,
                  [ message_type('turtlesim/action/RotateAbsolute_FeedbackMessage')
                  ]),
    ros_spin.

on_feedback(Feedback) :-
    ansi_format(comment, 'Feedback: ~p~n', [Feedback]).


:- table (client/2) as shared.

client(turtlesim, Client) :-
    ros_action_client('/turtle1/rotate_absolute',
                      'turtlesim/action/RotateAbsolute',
                      Client, []).

client(fibonacci, Client) :-
    ros_action_client('/fibonacci',
                      'action_tutorials_interfaces/action/Fibonacci',
                      Client, []).

rotate(To, SeqNum) :-
    client(turtlesim, Client),
    uuid(UUID),
    ros:ros_action_send_goal_request(Client,
                                     _{goal_id:UUID,
                                       goal:_{theta:To}},
                                     SeqNum).


order(N, SeqNum) :-
    client(fibonacci, Client),
    uuid(UUID),
    ros:ros_action_send_goal_request(Client,
                                     _{goal_id:UUID,
                                       goal:_{order:N}},
                                     SeqNum).

spin(Who) :-
    client(Who, Client),
    repeat,
       ros_wait([Client], 10, Ready),
       !,
    maplist(actions, Ready),
    spin(Who).

actions(action_client(Client, Feedback, Status, Goal, Cancel, Result)) :-
    action(Feedback, feedback, Client),
    action(Status,   status,   Client),
    action(Goal,     goal,     Client),
    action(Cancel,   cancel,   Client),
    action(Result,   result,   Client).

action(true, Type, Client) =>
    debug(ros(spin), 'Action client ~p ready for ~p', [Client,Type]),
    (   catch(action(Type, Client), E,
              print_message(warning, E))
    ->  true
    ;   true
    ).
action(false, _Type, _Client) =>
    true.

action(feedback, Client) =>
    ros:ros_action_take_feedback(Client, Feedback),
    ansi_format(comment, 'Feedback: ~p~n', [Feedback]).
action(status, Client) =>
    ros:ros_action_take_status(Client, Status),
    ansi_format(comment, 'Status: ~p~n', [Status]).
action(goal, Client) =>
    ros:ros_action_take_goal_response(Client, GoalResponse, MsgInfo),
    ansi_format(comment, 'GoalResponse: ~p; Info: ~p~n', [GoalResponse, MsgInfo]).
action(cancel, Client) =>
    ros:ros_action_take_cancel_response(Client, CancelResponse, MsgInfo),
    ansi_format(comment, 'CancelResponse: ~p; Info: ~p~n', [CancelResponse, MsgInfo]).
action(result, Client) =>
    ros:ros_action_take_result_response(Client, ResultResponse, MsgInfo),
    ansi_format(comment, 'ResultResponse: ~p; Info: ~p~n', [ResultResponse, MsgInfo]).

