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
          [ ros_current_topic/2,        % ?Topic, ?Type
            ros_node_current_client/3,  % +NodeName, -Client, -Type
            ros_node_current_service/3  % +NodeName, -Service, -Type
          ]).
:- use_module(library(ros)).
:- use_module(library(lists)).

/** <module> Query the ROS node graph

This library provides predicates to query   the  available nodes and how
they connect.
*/

%!  ros_current_topic(?Topic, ?Type) is nondet.
%
%   True when Topic with Type is visible on the ROS network.

ros_current_topic(Topic, Type) :-
    ros_default_node(Node),
    ros:ros_topic_names_and_types(Node, TopicsAndTypes),
    member(Topic-[Type], TopicsAndTypes).

%!  ros_node_current_client(+NodeName, -Client, -Type) is nondet.
%
%   True when the named node implements a service client that accepts
%   Type.  For example:
%
%   ```
%   ?- ros_node_current_client('/teleop_turtle', Client, Type)
%   Client = '/turtle1/rotate_absolute/_action/cancel_goal',
%   Type = 'action_msgs/srv/CancelGoal' ;
%   Client = '/turtle1/rotate_absolute/_action/get_result',
%   Type = 'turtlesim/action/RotateAbsolute_GetResult' ;
%   ...
%   ```

ros_node_current_client(NodePath, Client, Type) :-
    node_path_name_domain(NodePath, NodeName, NameSpace),
    ros_default_node(Node),
    ros:ros_client_names_and_types(Node, NodeName, NameSpace, NamesAndTypes),
    member(Client-[Type], NamesAndTypes).

node_path_name_domain(Path, NodeName, NameSpace) :-
    atomic_list_concat([''|Parts], '/', Path),
    (   Parts = [NodeName, NameSpace]
    ->  true
    ;   Parts = [NodeName],
        NameSpace = ''
    ).

%!  ros_node_current_service(+NodeName, -Service, -Type) is nondet.
%
%   Similar  to  ros_node_current_client/3,  enumerating   the  services
%   provided by a node. For example:
%
%   ```
%   ?- ros_node_current_service('/turtlesim', Client, Type).
%   Client = '/clear',
%   Type = 'std_srvs/srv/Empty' ;
%   Client = '/kill',
%   Type = 'turtlesim/srv/Kill' ;
%   ...
%   ```

ros_node_current_service(NodePath, Service, Type) :-
    node_path_name_domain(NodePath, NodeName, NameSpace),
    ros_default_node(Node),
    ros:ros_service_names_and_types(Node, NodeName, NameSpace, NamesAndTypes),
    member(Service-[Type], NamesAndTypes).
