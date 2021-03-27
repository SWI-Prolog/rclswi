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

:- module(ros_detail_options,
          [ node_from_options/2,        % -Node, +Options
            qos_profile_from_options/2  % -QoSProfile, +Options
          ]).
:- use_module(library(ros)).
:- autoload(library(ros/qos), [ros_qos_object/2]).
:- use_module(library(option)).

%!  node_from_opions(-Node, +Options) is det.
%
%   Get the ROS node to act from Options. The node may be specified as a
%   node object or node alias name.

node_from_options(Node, Options) :-
    option(node(NodeOrAlias), Options),
    !,
    (   ros_object(NodeOrAlias, ros_node)
    ->  Node = NodeOrAlias
    ;   ros_node(NodeOrAlias, Node)
    ).
node_from_options(Node, _) :-
    ros:ros_default_node(Node).

%!  qos_profile_from_options(-QoSProfile, +Options) is det.
%
%   Get a QoS profile instance from the option list.

qos_profile_from_options(QoSProfile, Options) :-
    (   option(qos(NameOrDict), Options)
    ->  ros_qos_object(NameOrDict, QoSProfile)
    ;   true
    ).
