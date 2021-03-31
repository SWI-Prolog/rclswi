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

:- module(ros_action_client,
          [ ros_action/3,           % +ActionName, +ActionType, +Options
            ros_action_run/4,       % +ActionName, +Goal, :Grammar, +Options
            ros_action_client/4     % +ActionName, +ActionType, -Client, +Options
          ]).
:- use_module(library(ros)).
:- use_module(library(ros/detail/options)).
:- use_module(library(ros/detail/actions)).
:- use_module(library(debug)).
:- use_module(library(error)).
:- use_module(library(lazy_lists)).
:- use_module(library(option)).
:- use_module(library(uuid)).

:- meta_predicate
    ros_action_run(+, +, //, +).

/** <module> Client interface for ROS actions
*/


:- predicate_options(ros_action/3, 3,
                     [ pass_to(ros_action_client/4, 4)
                     ]).
:- predicate_options(ros_action_run/4, 4,
                     [ timeout(number),
                       client(-any)
                     ]).
:- predicate_options(ros_action_client/4, 4,
                     [ node(any),
                       qos_profile(any)
                     ]).

:- multifile
    action_decl/3.
:- dynamic
    action_decl/3.

%!  ros_action(+ActionName, +ActionType, +Options) is det.
%
%   Define the properties  for  an  action   client  or  server  without
%   creating the client or server object.   This  declaration is used to
%   create a client instance in ros_action_run/4 from the ActionName.

ros_action(ActionName, ActionType, Options) :-
    must_be(atom, ActionName),
    action_decl(ActionName, ActionType, Options),
    !.
ros_action(ActionName, ActionType, Options) :-
    retractall(action_decl(ActionName, _, _)),
    asserta(action_decl(ActionName, ActionType, Options)),
    !.

term_expansion(user:ros_action(ActionName, ActionType, Options),
               ros_actions:action_decl(ActionName, ActionType, Options)) :-
    must_be(atom, ActionName).

%!  action_client(+NameOrClient, -Client) is det.

:- dynamic
    ros_action_object/3.

action_client(Client0, Client) :-
    ros_object(Client0, action_client),
    !,
    Client = Client0.
action_client(ActionName, Client) :-
    with_mutex(ros_actions, create_action_client(ActionName, Client0)),
    Client = Client0.

create_action_client(ActionName, Client) :-
    action_decl(ActionName, ActionType, Options),
    (   ros_action_object(ActionName, Client, Options)
    ->  true
    ;   ros_action_client(ActionName, ActionType, Client, Options),
        asserta(ros_action_object(ActionName, Client, Options))
    ).

%!  ros_action_run(+ActionName, +Goal, :Gammar, +Options) is det.
%
%   Run ActionName started with message Goal and provide the response as
%   a lazy list of terms.  Options processed:
%
%     - timeout(+TimeOut)
%       Max time to wait for a next event from this client.
%     - client(-Client)
%       Get access to the dict that tracks progress of the action.

ros_action_run(ActionName, Goal, Grammar, Options) :-
    option(timeout(Timeout), Options, infinite),
    option(client(State), Options, State),
    action_client(ActionName, Client),
    uuid(GoalID),
    ros:ros_action_send_goal_request(Client, _{goal_id:GoalID, goal:Goal}, SeqN),
    State = action_client{ client:Client,
                           timeout:Timeout,
                           goal_id:GoalID,
                           sequence_number:SeqN,
                           result_sequence_number:0,
                           cancel_sequence_number:0,
                           status:request,
                           final:false
                         },
    lazy_list(action_state(State), Responses),
    (   phrase(Grammar, Responses)
    ->  true
    ;   ros_cancel_action(State, Options)
    ).

action_state(State, Items, Tail) :-
    ros_wait([State.client], State.timeout,
             [action_client(Client, Feedback, Status, Goal, Cancel, Result)]),
    action(Feedback, feedback, Client, State, Items, T0),
    action(Status,   status,   Client, State, T0, T1),
    action(Goal,     goal,     Client, State, T1, T2),
    action(Cancel,   cancel,   Client, State, T2, T3),
    action(Result,   result,   Client, State, T3, Tail),
    (   State.final == true
    ->  Tail = []
    ;   true
    ).

action(true, Type, Client, State, List, Tail) =>
    debug(ros(spin), 'Action client ~p ready for ~p', [Client,Type]),
    (   catch(action(Type, Client, State, Item), E,
              print_message(warning, E))
    ->  (   Item == []
        ->  List = Tail
        ;   List = [Item|Tail]
        )
    ;   List = Tail
    ).
action(false, _Type, _Client, _State, List, Tail) =>
    List = Tail.

action(feedback, Client, State, Item) =>
    ros:ros_action_take_feedback(Client, Message),
    debug(ros(action), 'Got feedback message ~p', [Message]),
    GoalID = State.goal_id,
    _{goal_id:GoalID, feedback:Feedback} :< Message,
    Item = feedback(Feedback).
action(status, Client, State, Item) =>
    ros:ros_action_take_status(Client, StatusArray),
    debug(ros(action), 'Got status message ~p', [StatusArray]),
    GoalID = State.goal_id,
    (   member(GoalStatus, StatusArray.status_list),
        _{goal_info: GoalInfo, status: StatusCode} :< GoalStatus,
        _{goal_id:GoalID} :< GoalInfo
    ->  ros:ros_enum_goal_status(StatusCode, Status),
        Item = status(Status),
        action_status(Status, State)
    ;   Item = []
    ).
action(goal, Client, State, Item) =>
    ros:ros_action_take_goal_response(Client, GoalResponse, MsgInfo),
    debug(ros(action), 'Got goal response ~p', [GoalResponse]),
    SeqNum = MsgInfo.sequence_number,
    assertion(State.sequence_number == SeqNum),
    Accepted = GoalResponse.accepted,
    (   Accepted == true
    ->  Item = accepted
    ;   Item = rejected
    ),
    nb_set_dict(status, State, Item).
action(cancel, Client, State, Item) =>
    ros:ros_action_take_cancel_response(Client, CancelResponse, MsgInfo),
    debug(ros(cancel), 'Got cancel response ~p (info = ~p)',
          [CancelResponse, MsgInfo]),
    SeqNum = MsgInfo.sequence_number,
    assertion(State.cancel_sequence_number == SeqNum),
    Item = cancel(CancelResponse, MsgInfo),
    nb_set_dict(final, State, true).
action(result, Client, State, Item) =>
    ros:ros_action_take_result_response(Client, ResultResponse, MsgInfo),
    debug(ros(action), 'Got result response ~p', [ResultResponse]),
    SeqNum = MsgInfo.sequence_number,
    assertion(State.result_sequence_number == SeqNum),
    _{result:Result, status:StatusCode} :< ResultResponse,
    Item = result(Result),
    ros:ros_enum_goal_status(StatusCode, Status),
    nb_set_dict(status, State, Status),
    nb_set_dict(final, State, true).

action_status(Status, State) :-
    nb_set_dict(status, State, Status),
    (   Status == succeeded
    ->  ros:ros_action_send_result_request(State.client,
                                           _{goal_id:State.goal_id}, SeqNum),
        nb_set_dict(result_sequence_number, State, SeqNum)
    ;   Status == aborted
    ->  nb_set_dict(final, State, true)
    ).

%!  ros_cancel_action(+State, +Options)
%
%   Cancel the indicated action. State is a   dict  holding at least the
%   action client handle in `client`  and   the  `goal_id`  that must be
%   cancelled.


ros_cancel_action(State, Options) :-
    _{client:Client, goal_id:GoalID} :< State,
    ros:ros_action_send_cancel_request(Client,
                                       _{goal_info:_{goal_id:GoalID}},
                                       SeqNum),
    debug(ros(cancel), 'Sent cancel for ~p -> seq_num = ~p', [GoalID, SeqNum]),
    nb_set_dict(cancel_sequence_number, State, SeqNum),
    lazy_list(action_state(State), Responses),
    phrase(cancel(Options), Responses).

cancel(Options) -->
    [_0H],
    { debug(ros(action), 'List holds ~p', [_0H]) },
    !,
    cancel(Options).
cancel(_Options) -->
    [].


%!  ros_action_client(+ActionName, +ActionType, -Client, +Options) is det
%
%   Register an action client for  ActionName that satisfies ActionType.
%   Options processed:
%
%     - node(+Node)
%       Node with which to associate this service
%     - qos_profile(+QoSProfile)
%       Dict or option list holding QoS profiles for the various
%       services related to an action. Possible members are
%       `goal`, `result`, `cancel`, `feedback` or `status`.

ros_action_client(ActionName, ActionType, Client, Options) :-
    node_from_options(Node, Options),
    ros:ros_action_type_support(ActionType, TypeSupport),
    qos_profile_dict_from_options(client_qos, QoSProfile, Options),
    ros:'$ros_create_action_client'(Node, TypeSupport, ActionName,
                                    QoSProfile, Client).

client_qos(goal).
client_qos(result).
client_qos(cancel).
client_qos(feedback).
client_qos(status).
