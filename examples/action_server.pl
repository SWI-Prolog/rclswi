/*  This example code is public domain.
*/

:- module(action_server,
          [ server/0
          ]).
:- use_module(library(ros)).
:- use_module(library(ros/action/server)).

server :-
%   ros_debug(10),
    debug(ros(_)),
    ros_default_context(Context),
    ros:ros_create_clock(Context, system, Clock),
    ros_action_server('/turtle1/rotate_absolute',
                      'turtlesim/action/RotateAbsolute',
                      true,
                      Server,
                      [ clock(Clock) ]),
    server_loop(Server, []).

server_loop(Server, Goals) :-
    ros_wait([Server], infinite, Ready),
    Ready = [action_server(Server, Goal, Cancel, Result, Expired)],
    server_action(Server, Goal, Cancel, Result, Expired, Goals, NewGoals),
    server_loop(Server, NewGoals).

server_action(Server, Goal, Cancel, Result, Expired, Goals0, Goals) :-
    server_action(Goal, Server, goal, Goals0, Goals1),
    server_action(Cancel, Server, cancel, Goals1, Goals2),
    server_action(Result, Server, result, Goals2, Goals3),
    server_action(Expired, Server, expired, Goals3, Goals).

server_action(true, Server, Type, Goals0, Goals) :-
    catch(server_action1(Type, Server, Goals0, Goals),
          E,
          print_message(warning, E)).
server_action(false, _, _, Goals, Goals).

server_action1(goal, Server, Goals, [Handle|Goals]) :-
    ros:ros_action_take_goal_request(Server, Request, Info),
    debug(ros(action), 'Got goal request ~p, info ~p', [Request, Info]),
    get_time(Now),
    GoalID = Request.goal_id,
    ros:ros_action_accept_new_goal(Server, GoalID, Now, Handle),
    ros:ros_action_send_goal_response(Server, _{accepted:true, stamp:Now}, Info),
    run_goal(Request.goal, Server, GoalID, Handle).
server_action1(cancel, Server, Goals, Goals) :-
    ros:ros_action_take_cancel_request(Server, Request, Info),
    debug(ros(action), 'Got cancel request ~p, info ~p', [Request, Info]),
    cancel_goals(Server, Goals, Request.goal_info, Info).
server_action1(result, Server, Goals, Goals) :-
    ros:ros_action_take_result_request(Server, Request, Info),
    debug(ros(action), 'Got result request ~p, info ~p', [Request, Info]),
    ros:ros_action_send_result_response(Server,
                                        _{result:_{remaining:42},
                                          status:4}, Info).
server_action1(expired, Server, Goals, Goals) :-
    debug(ros(action), 'Got expired event', [Server]).

run_goal(Goal, Server, GoalID, Handle) :-
    Theta = Goal.theta,
    ros:ros_action_update_goal_state(Handle, execute),
    ros:ros_action_publish_status(Server),
    run(Theta, Server, GoalID, Handle).

run(Theta, Server, _GoalID, Handle) :-
    Theta =< 0,
    ros:ros_action_update_goal_state(Handle, succeed),
    ros:ros_action_publish_status(Server).
run(Theta, Server, GoalID, Handle) :-
    ros:ros_action_publish_feedback(Server, _{feedback:_{remaining:Theta},
                                              goal_id:GoalID}),
    Theta2 is Theta - 0.01,
    peek_cancel(Server, 0.1, Handle),
    run(Theta2, Server, GoalID, Handle).


%!  cancel_goals(+Server, +OurGoals, +CancelInfo, +MsgInfo)
%
%   Cancel all goals from OurGoals that match CancelInfo.

cancel_goals(Server, Goals, CancelInfo, MsgInfo) :-
    cancelling(Goals, CancelInfo, Cancelling),
    (   Cancelling == []
    ->  Status = unknown_goal_id
    ;   ros:ros_action_publish_status(Server),
        Status = none
    ),
    cancel_enum(Code, Status),
    ros:ros_action_send_cancel_response(Server,
                                        _{ return_code:Code,
                                           goals_canceling:Cancelling},
                                        MsgInfo).

cancelling([], _, []).
cancelling([H|T0], CancelInfo, [H|T]) :-
    ros:'$ros_action_goal_prop'(H, goal_info, ThisInfo),
    cancel_matches(ThisInfo, CancelInfo),
    !,
    debug(ros(action), 'Cancelling ~p', [ThisInfo]),
    ros:ros_action_update_goal_state(H, cancel_goal),
    cancelling(T0, CancelInfo, T).
cancelling([_|T0], CancelInfo, T) :-
    cancelling(T0, CancelInfo, T).

cancel_enum(0, none).
cancel_enum(1, rejected).
cancel_enum(1, unknown_goal_id).
cancel_enum(1, goal_terminated).


% TBD: include time in the matching
cancel_matches(ThisInfo, CancelInfo) :-
    ThisInfo.goal_id == CancelInfo.goal_id.

peek_cancel(Server, Time, Handle) :-
    (   ros_wait([Server], Time, Ready)
    ->  Ready = [action_server(Server, Goal, Cancel, Result, Expired)],
        server_action(Server, Goal, Cancel, Result, Expired, [Handle], _)
    ;   true                            % timeout
    ).
