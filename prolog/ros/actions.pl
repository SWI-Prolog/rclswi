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

:- module(ros_actions,
          [ ros_action_client/4,    % +ActionName, +ActionType, -Client, +Options
            ros_action_server/4     % +ActionName, +ActionType, -Server, +Options
          ]).
:- use_module(library(ros)).

/** <module> Deal with ROS actions
*/

%!  ros_action_client(+ActionName, +ActionType, -Client, +Options) is det
%
%   Register an action client for  ActionName that satisfies ActionType.
%   Options processed:
%
%     - node(+Node)
%       Node with which to associate this service
%     - qos_profile(+QoSProfile)
%       Dict holding QoS profiles for the various services related
%       to an action.

ros_action_client(ActionName, ActionType, Client, Options) :-
    ros:node(Node, Options),
    ros:ros_action_type_support(ActionType, TypeSupport),
%   ros:qos_profile(QoSProfile, Options),
    ros:'$ros_create_action_client'(Node, TypeSupport, ActionName, _QoSProfile, Client).

%!  ros_action_server(+ActionName, +ActionType, -Client, +Options) is det
%
%   Register an action server for  ActionName that satisfies ActionType.
%   Options processed:
%
%     - node(+Node)
%       Node with which to associate this service
%     - qos_profile(+QoSProfile)
%       Dict holding QoS profiles for the various services related
%       to an action.

ros_action_server(ActionName, ActionType, Server, Options) :-
    ros:node(Node, Options),
    ros:ros_action_type_support(ActionType, TypeSupport),
%   ros:qos_profile(QoSProfile, Options),
    ros:'$ros_create_action_server'(Node, TypeSupport, ActionName, _QoSProfile, Server).

%!  init_goal_status is det.
%!  init_cancel_type is det.
%
%   Callbacks from rclswi.c  to  establish   the  type  information  for
%   actions.

:- public
    init_goal_status/0,
    init_cancel_type/0.

init_goal_status :-
    ros:ros_type_support('action_msgs/msg/GoalStatusArray', Type),
    ros:set_goal_status_type(Type).
init_cancel_type :-
    ros:ros_type_support('action_msgs/srv/CancelGoal', Type),
    ros:set_action_cancel_type(Type).
