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

:- module(ros_graph,
          [ ros_current_node/1,         % -NodeName
            ros_current_topic/2,        % ?Topic, ?Type
            ros_node_interface/5        % +NodeName, ?Kind, ?ClientServer, ?Path, ?Type
          ]).
:- use_module(library(ros)).
:- use_module(library(lists)).

/** <module> Query the ROS node graph

This library provides predicates to query   the  available nodes and how
they connect.
*/

%!  ros_current_node(-NodeName) is nondet.
%
%   True when NodeName is visible from our default node.  For example:
%
%       ?- ros_current_node(N).
%       N = '/swi_turtlesim' ;
%	N = '/prolog' ;
%	...

ros_current_node(NodeName) :-
    ros_default_node(Node),
    ros:ros_nodes(Node, Visible),
    member(_{name:Name, namespace:Namespace}, Visible),
    (   sub_atom(Namespace, _, _, 0, /)
    ->  atom_concat(Namespace, Name, NodeName)
    ;   atomic_list_concat([Namespace, /, Name], NodeName)
    ).

%!  ros_current_topic(?Topic, ?Type) is nondet.
%
%   True when Topic with Type is visible on the ROS network.

ros_current_topic(Topic, Type) :-
    ros_default_node(Node),
    ros:ros_topic_names_and_types(Node, TopicsAndTypes),
    member(Topic-[Type], TopicsAndTypes).

%!  ros_node_interface(?NodeName, ?Kind, ?ClientServer, ?Path, ?Type) is nondet.
%
%   True when the named node implements  the requested service or action
%   client or server. Note that these predicates   operate  on a node in
%   the network that is visible from our default node. For example:
%
%   ```
%   ?- ros_node_interface('/teleop_turtle', topic, client, Client, Type)
%   Client = '/turtle1/rotate_absolute/_action/cancel_goal',
%   Type = 'action_msgs/srv/CancelGoal' ;
%   Client = '/turtle1/rotate_absolute/_action/get_result',
%   Type = 'turtlesim/action/RotateAbsolute_GetResult' ;
%   ...
%   ```
%
%   @arg Kind is one of `topic`, `service` or `action`
%   @arg ClientServer is one of `client` or `server`

ros_node_interface(NodePath, Kind, ClientServer, Path, Type) :-
    (   var(NodePath)
    ->  ros_current_node(NodePath)
    ;   true
    ),
    node_path_name_domain(NodePath, NodeName, NameSpace),
    ros_default_node(Node),
    node_interface(Kind, ClientServer, Node, NodeName, NameSpace, NamesAndTypes),
    member(Path-[Type], NamesAndTypes).

node_interface(topic, client, Node, NodeName, NameSpace, NamesAndTypes) :-
    ros:ros_subscriber_names_and_types(Node, NodeName, NameSpace, false, NamesAndTypes).
node_interface(topic, server, Node, NodeName, NameSpace, NamesAndTypes) :-
    ros:ros_publisher_names_and_types(Node, NodeName, NameSpace, false, NamesAndTypes).
node_interface(service, client, Node, NodeName, NameSpace, NamesAndTypes) :-
    ros:ros_client_names_and_types(Node, NodeName, NameSpace, NamesAndTypes).
node_interface(service, server, Node, NodeName, NameSpace, NamesAndTypes) :-
    ros:ros_service_names_and_types(Node, NodeName, NameSpace, NamesAndTypes).
node_interface(action, client, Node, NodeName, NameSpace, NamesAndTypes) :-
    ros:ros_action_client_names_and_types(Node, NodeName, NameSpace, NamesAndTypes).
node_interface(action, server, Node, NodeName, NameSpace, NamesAndTypes) :-
    ros:ros_action_server_names_and_types(Node, NodeName, NameSpace, NamesAndTypes).

node_path_name_domain(Path, NodeName, NameSpace) :-
    file_base_name(Path, NodeName),
    file_directory_name(Path, NameSpace).
