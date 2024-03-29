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

:- module(ros_action_server,
          [ ros_action_server/5,          % +ActionName, +ActionType, :Goal,
                                          % -Server, +Options
            ros_action_server_property/2, % +ActionServer, ?Property
            ros_goal_handle_property/2,   % +GoalHandle, ?Property
            ros_action_spin/1,            % +Server
            ros_action_feedback/1,        % +Message
            ros_action_on_cancel/3        % :Goal,?Why,:Cleanup
          ]).
:- use_module(library(ros)).
:- use_module(library(ros/detail/options)).
:- use_module(library(ros/detail/actions)).

:- use_module(library(option)).
:- use_module(library(debug)).
:- use_module(library(apply)).
:- use_module(library(lists)).

:- meta_predicate
    ros_action_server(+,+,2,-,+),
    ros_action_on_cancel(0,?,0).


/** <module> Support ROS action servers

A ROS action server is a service that handles _goals_. This goes through
these phases:

  - A goal request is received.  We can act on this by
    - Rejecting the goal (e.g., if we are busy)
    - Accepting the goal
      - This may optionally abort other goals.
    In other words, we may run multiple goals.  If we can only
    manage one goal at a time we can choose between aborting
    the goal in progress and accept the new goal or reject the
    new goal.
  - The running goal may, at its sole discretion, decide to post
    _feedback_ messages.
  - A goal cancel request is received.  We should identify
    addressed goals and may decide to
    - Accept the request for one or more goals
    - Respond that the goals have already terminated
    - Respond that no goal matches the request
    - Reject the request.

Currently, an action server  is   registered  using ros_action_server/5.
This  server  is  associated  with   this    node   for  spinning  using
ros_action_spin/1.  The  spinner  thread  accepts  action  requests.  It
creates a new thread for running an action goal.

The action goal is a normal Prolog goal  that executes the action. If it
wants to send feedback it may call ros_action_feedback/1 using a message
that matches the feedback type  for   the  registered action. The action
goal must be prepared to handle   the exception ros_action(Request) that
is injected into the goal using thread_signal/2.
*/

%!  ros_action_server(+ActionName, +ActionType, :Goal,
%!                    -Server, +Options) is det
%
%   Register an action server for  ActionName that satisfies ActionType.
%   Options processed:
%
%     - node(+Node)
%       Node with which to associate this service
%     - qos_profile(+QoSProfile)
%       Dict holding QoS profiles for the various services related
%       to an action. Possible members are `goal`, `result`, `cancel`,
%       `feedback` or `status`.
%     - clock(+Clock)
%       Clock to use.  Default is the default clock of the node.  See
%       ros_node_property/2.
%     - result_timeout(+Seconds)
%       Expire the goal in Seconds.  Default is to not expire the goal.
%
%   @tbd limit number of ongoing goals
%   @tbd deal with goal expiring

ros_action_server(ActionName, ActionType, Goal, Server, Options) :-
    node_from_options(Node, Options),
    server_clock(Node, Clock, Options),
    option(result_timeout(ResultTimeOut), Options, 0.0),
    ros:ros_action_type_support(ActionType, TypeSupport),
    qos_profile_dict_from_options(server_qos, QoSProfile, Options),
    ros:'$ros_create_action_server'(Node, Clock, TypeSupport, ActionName,
                                    QoSProfile, ResultTimeOut, Goal, Server).

server_clock(_Node, Clock, Options) :-
    option(clock(Clock), Options),
    !.
server_clock(Node, Clock, _Options) :-
    ros_node_property(Node, clock(Clock)).

server_qos(goal).
server_qos(result).
server_qos(cancel).
server_qos(feedback).
server_qos(status).

%!  ros_action_spin(+Server) is det.
%
%   Register  an  action  server  as  waitable  object  with  its  node,
%   processing requests on ros_spin/1.

ros_action_spin(Server) :-
    ros_action_server_property(Server, name(Name)),
    ros_action_server_property(Server, node(Node)),
    ros:register_waitable(action_server(Name), Node, Server,
                          on_action_server_request).

:- multifile
    ros:ros_ready/1.

ros:ros_ready(action_server(Server, Goal, Cancel, Result, Expired)) :-
    server_action(Goal,    Server, goal),
    server_action(Cancel,  Server, cancel),
    server_action(Result,  Server, result),
    server_action(Expired, Server, expired).

server_action(true, Server, Type) :-
    catch(server_action1(Type, Server),
          E,
          print_message(warning, E)).
server_action(false, _, _).

server_action1(goal, Server) =>
    ros:ros_action_take_goal_request(Server, Request, Info),
    debug(ros(action), 'Goal request ~p', [Request.goal]),
    accept_goal(Server, Request, Info).
server_action1(result, Server) =>
    ros:ros_action_take_result_request(Server, Request, Info),
    GoalID = Request.goal_id,
    debug(ros(action), 'Result request for ~p', [GoalID]),
    answer_result_request(Server, GoalID, Info).
server_action1(cancel, Server) =>
    ros:ros_action_take_cancel_request(Server, Request, Info),
    GoalInfo = Request.goal_info,
    debug(ros(action), 'Cancel request for ~p', [GoalInfo]),
    cancel_goals(Server, GoalInfo, Info).
server_action1(expired, Server) =>
    debug(ros(action), 'Got expired event', [Server]),
    expire_goals(Server).

:- dynamic
    goal/4.                         % Server, Handle, GoalID, Thread

:- thread_local
    goal_data/2.                    % Handle, GoalID


%!  accept_goal(+Server, +Request, +Info) is det.
%
%   Accept (or reject) a new goal for this server.

accept_goal(Server, Request, Info) :-
    ros_action_server_property(Server, goal(CallBack)),
    GoalID = Request.goal_id,
    Goal = Request.goal,
    catch(thread_create(managed_goal(Server, GoalID, CallBack, Goal), Thread,
                        [ at_exit(done_goal) ]),
          E, true),
    (   var(E)
    ->  get_time(Now),
        ros:ros_action_accept_new_goal(Server, GoalID, Now, Handle),
        assertz(goal(Server, Handle, GoalID, Thread)),
        thread_send_message(Thread, ros_goal_handle(GoalID, Handle)),
        Accepted = true
    ;   Accepted = false
    ),
    ros:ros_action_send_goal_response(Server,
                                      _{accepted:Accepted, stamp:Now},
                                      Info).

:- public
    done_goal/0.

done_goal :-
    thread_self(Me),
    thread_property(Me, status(Status)),
    debug(ros(action), 'Goal thread completed with status ~p', [Status]),
    thread_detach(Me).              % Any reason to wait?


%!  managed_goal(+Server, +GoalID, :Callback, +Goal)
%
%   Run an action server goal inside a thread.

managed_goal(Server, GoalID, Callback, Goal) :-
    thread_self(Me),
    thread_get_message(Me, ros_goal_handle(GoalID, Handle), [timeout(10)]),
    asserta(goal_data(Handle, GoalID)),
    ros:ros_action_update_goal_state(Handle, execute),
    ros:ros_action_publish_status(Server),
    (   catch(call(Callback, Goal, Result), Error, true)
    ->  (   var(Error)
        ->  Success = true
        ;   true
        )
    ;   true
    ),
    (   Error == ros(cancel)
    ->  ros:ros_action_update_goal_state(Handle, canceled),
        ros:ros_action_publish_status(Server),
        throw(Error)
    ;   (   Success == true
        ->  NewState = succeed
        ;   NewState = abort
        ),
        ros:ros_action_update_goal_state(Handle, NewState),
        ros:ros_action_publish_status(Server),
        thread_get_message(Me, ros(send_result_request(Info)), [timeout(10)]),
        debug(ros(action), 'Asked to send result', []),
        (   Success = true
        ->  ros:ros_enum_goal_status(StatusCode, succeeded),
            ros:ros_action_send_result_response(
                    Server, _{result:Result, status:StatusCode}, Info)
        ;   ros:ros_enum_goal_status(StatusCode, aborted),
            ros:ros_action_send_result_response(
                    Server, _{status:StatusCode}, Info),
            nonvar(Error),
            throw(Error)
        )
    ).

%!  answer_result_request(+Server, +GoalID, -Info) is det.
%
%   Answer to a result request for GoalID.   Info is the service message
%   info we need for ros_action_send_result_response/3.

answer_result_request(Server, GoalID, Info) :-
    goal(Server, Handle, GoalID, Thread),
    ros_goal_handle_property(Handle, status(Status)),
    answer_result_request_(Status, Server, Thread, Info).

answer_result_request_(succeeded, _Server, Thread, Info) =>
    thread_send_message(Thread, ros(send_result_request(Info))).
answer_result_request_(Status, Server, _Thread, Info) =>
    ros:ros_enum_goal_status(StatusCode, Status),
    ros:ros_action_send_result_response(Server, _{status:StatusCode}, Info).

%!  cancel_goals(+Server, +GoalInfo, +MsgInfo)
%
%   Cancel Goals matching GoalInfo.

cancel_goals(Server, GoalInfo, MsgInfo) :-
    GoalID = GoalInfo.goal_id,
    time_msg_stamp(GoalInfo.stamp, Time),
    cancel_goals(Server, GoalID, Time, Response),
    ros:ros_action_send_cancel_response(Server, Response, MsgInfo).

cancel_goals(Server, '00000000-0000-0000-0000-000000000000', 0, Response) =>
    Before is 1<<63,
    cancel_goals_before(Server, Before, Response).
cancel_goals(Server, '00000000-0000-0000-0000-000000000000', Time, Response) =>
    cancel_goals_before(Server, Time, Response).
cancel_goals(Server, GoalID, 0, Response) =>
    cancel_goal(Server, GoalID, Response).
cancel_goals(Server, GoalID, Time, Response) =>
    cancel_goal(Server, GoalID, Response1),
    cancel_goals_before(Server, Time, Response2),
    join_cancel_response(Response1, Response2, Response).

join_cancel_response(_{return_code:_, goals_canceling:[]}, Response2, Response) =>
    Response = Response2.
join_cancel_response(Response1, _{return_code:_, goals_canceling:[]}, Response) =>
    Response = Response1.
join_cancel_response(_{return_code:_, goals_canceling:Cancelling1},
                     _{return_code:_, goals_canceling:Cancelling2}, Response) =>
    cancel_enum(StatusCode, none),
    append(Cancelling1, Cancelling2, Cancelling3),
    sort(Cancelling3, Cancelling),
    Response = _{return_code:StatusCode, goals_canceling:Cancelling}.


cancel_goal(Server, GoalID, Response) :-
    (   goal(Server, Handle, GoalID, Thread)
    ->  ros:ros_action_update_goal_state(Handle, cancel_goal),
        ros:ros_action_publish_status(Server),
        cancel_goal(Thread, Status),
        cancel_enum(StatusCode, Status),
        (   Status == none
        ->  ros_goal_handle_property(Handle, info(Info)),
            Cancelling = [Info]
        ;   Cancelling = []
        )
    ;   cancel_enum(StatusCode, unknown_goal_id),
        Cancelling = []
    ),
    Response = _{ return_code:StatusCode, goals_canceling:Cancelling}.

cancel_goals_before(Server, Time, Response) :-
    findall(Cancel, cancel_goal_before(Server, Time, Cancel), Cancelling),
    cancel_enum(StatusCode, none),
    Response = _{ return_code:StatusCode, goals_canceling:Cancelling}.

cancel_goal_before(Server, Time, Info) :-
    goal(Server, Handle, _GoalID, Thread),
    ros_goal_handle_property(Handle, info(Info)),
    time_msg_stamp(Info.stamp, Start),
    Start =< Time,
    cancel_goal(Thread, _Status).

%!  time_msg_stamp(?MsgTime, ?Stamp) is det.
%
%   Translate between time as represented in   a  message and the Prolog
%   representation as a float. The second   clause  anticipates on doing
%   this translation in the core message type translation.

time_msg_stamp(_{nanosec:NSec, sec:Sec}, Stamp) :-
    !,
    (   var(Stamp)
    ->  Stamp is Sec+NSec/1_000_000_000
    ;   Sec is floor(Stamp),
        NSec is round((Stamp-Sec)*1_000_000_000)
    ).
time_msg_stamp(Stamp, Stamp).

cancel_enum(0, none).
cancel_enum(1, rejected).
cancel_enum(1, unknown_goal_id).
cancel_enum(1, goal_terminated).

%!  cancel_goal(+Thread, -Status)
%
%   Cancel the goal that is executed by Thread.

cancel_goal(Thread, Status) :-
    E = error(_,_),
    catch(thread_signal(Thread, throw(ros(cancel))), E, true),
    (   var(E)
    ->  Status = none
    ;   Status = goal_terminated
    ).

%!  expire_goals(+Server) is det.
%
%   Deal with an expire event. I think  this   is  called if one or more
%   goal handles can be discarded. We reclaim   the  goal from our known
%   goals, which also removes the  last   reference  to the goal handle,
%   leaving the destruction to the garbage collector.

expire_goals(Server) :-
    predicate_property(goal(_,_,_,_), number_of_clauses(Max)),
    (   Max > 0
    ->  ros:ros_action_expire_goals(Server, Max, Expired),
        maplist(reclaim_goal, Expired)
    ;   true
    ).

reclaim_goal(GoalInfo) :-
    retractall(goal(_,_,GoalInfo.goal_id,_)).

%!  ros_action_feedback(+Message) is det.
%
%   Send a feedback message from an ROS action server.

ros_action_feedback(Message) :-
    goal_data(Handle, GoalID),
    ros_goal_handle_property(Handle, server(Server)),
    debug(ros(action), 'Sending feedback for ~p: ~p', [GoalID, Message]),
    ros:ros_action_publish_feedback(Server,
                                    _{feedback:Message,
                                      goal_id:GoalID}).

%!  ros_action_on_cancel(:Goal, ?Why, :Cleanup)
%
%   Run Goal, watching for a cancellation   exception. Why is the reason
%   for cancellation, which is normally `cancel`.

ros_action_on_cancel(Goal, Why, Cleanup) :-
    catch(Goal, ros(Why),
          call_cleanup(Cleanup, throw(Why))).


%!  ros_action_server_property(+ActionServer, ?Property)
%
%   True when Property is a property of ActionServer. Defined properties
%   are:
%
%     - node(-Node)
%     - name(-Name)
%     - clock(-Clock)
%     - goal(-Goal)

ros_action_server_property(ActionServer, Property) :-
    property_action_server(Property, ActionServer).

property_action_server(node(Node), ActionServer) :-
    ros:'$ros_action_server_prop'(ActionServer, node, Node).
property_action_server(name(Name), ActionServer) :-
    ros:'$ros_action_server_prop'(ActionServer, name, Name).
property_action_server(clock(Clock), ActionServer) :-
    ros:'$ros_action_server_prop'(ActionServer, clock, Clock).
property_action_server(goal(Goal), ActionServer) :-
    ros:'$ros_action_server_prop'(ActionServer, goal, Goal).


%!  ros_goal_handle_property(+GoalHandle, ?Property)
%
%   True when Property is a property   of GoalHandle. Defined properties
%   are:
%
%     - server(-Server)
%       Action server this handle belongs to.
%     - info(-Info)
%       ROS Goal info struct (dict) holding the goal UUID and time
%       stamp when it was created.
%     - active(-Bool)
%       Whether the goal is active
%     - status(-Status)
%       Status of the goal.  One of `unknown`, `accepted`, `executing`,
%       `canceling`, `succeeded`, `canceled` or `aborted`.

ros_goal_handle_property(GoalHandle, Property) :-
    property_goal_handle(Property, GoalHandle).

property_goal_handle(server(Server), GoalHandle) :-
    ros:'$ros_action_goal_prop'(GoalHandle, server, Server).
property_goal_handle(info(Info), GoalHandle) :-
    ros:'$ros_action_goal_prop'(GoalHandle, goal_info, Info).
property_goal_handle(active(Bool), GoalHandle) :-
    ros:'$ros_action_goal_prop'(GoalHandle, active, Bool).
property_goal_handle(status(Status), GoalHandle) :-
    ros:'$ros_action_goal_prop'(GoalHandle, status, Status).
