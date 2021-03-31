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

:- module(ros_detail_actions,
          []).
:- use_module(library(ros)).

/** <module> Shared code to deal with ROS actions
*/

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
