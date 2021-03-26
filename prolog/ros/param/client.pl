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

:- module(ros_param_client,
          [ ros_node_parameter/3,        % +Node, -Name
            ros_node_get_parameter/4,    % +Node, +Name, -Value, +Options
            ros_node_get_parameters/3,   % +Node, +Dict, +Options
            ros_node_set_parameter/4,    % +Node, +Name, +Value, +Options
            ros_node_set_parameters/3,   % +Node, +Dict, +Options
            ros_parameter_types/3,       % +Node, +Dict, +Options
            ros_parameter_descriptions/3 % +Node, +Dict, +Options
          ]).
:- use_module(library(ros/services)).
:- use_module(library(ros/detail/param)).
:- use_module(library(debug)).
:- use_module(library(lists)).
:- use_module(library(option)).
:- use_module(library(apply)).
:- use_module(library(pairs)).

/** <module> Access ROS parameters on other nodes

This module is  a  client  library   for  the  ROS2  parameter  services
associated with a node. It allows querying   the names and properties of
parameters in a node as well as getting and setting parameter values.

@tbd How to deal with  client  handle   life  time?  As  this moment the
clients are left to (atom)  garbage   collection.  Alternatively  we can
cache the clients (optionally for a   limited  time) or actively destroy
them.
@tbd Current behaviour is synchronous.  We may also want an asynchronous
API, notably for setting parameters.
*/


%!  ros_node_parameter(+Node:atom, -Name, +Options) is nondet.
%
%   True when Name is a parameter in Node.  For example:
%
%       ?- ros_node_parameter('/turtlesim', Name, []).
%       Name = background_b ;
%	Name = background_g ;
%	Name = background_r ;
%	Name = use_sim_time.

ros_node_parameter(Node, Name, Options) :-
    option(depth(Depth), Options, 0),
    option(prefixes(Prefixes), Options, []),
    param_client(Node, list_parameters, Client, Options),
    Request = _{depth:Depth, prefixes:Prefixes},
    ros_call(Client, Request, Response, Options),
    member(NameS, Response.result.names),
    atom_string(Name, NameS).

%!  ros_parameter_types(+Node, +Dict, +Options) is det.
%
%   Given a dict with keys set to   parameter  names and unbound values,
%   bind all values to the parameter   types.  Defined types are `bool`,
%   `integer`,   `double`,   `string`,     `byte_array`,   `bool_array`.
%   `integer_array`, `double_array` and `string_array`. For example:
%
%   ```
%   ?- ros_parameter_types('/turtlesim', _{background_r:Type}, []).
%   Type = integer
%   ```

ros_parameter_types(Node, Dict, Options) :-
    dict_pairs(Dict, _, Pairs),
    pairs_keys_values(Pairs, Keys, Types),
    param_client(Node, get_parameter_types, Client, Options),
    Request = _{names:Keys},
    ros_call(Client, Request, Response, Options),
    maplist(ros:ros_enum_param_type, Response.types, Types).

%!  ros_parameter_descriptions(+Node, +Dict, +Options) is det.
%
%   Given a dict with keys set to   parameter  names and unbound values,
%   bind all values to the parameter description.  Currently returns the
%   raw response dict as description.
%
%   @tbd Future versions will provide a more Prolog minded description.

ros_parameter_descriptions(Node, Dict, Options) :-
    dict_pairs(Dict, _, Pairs),
    pairs_keys_values(Pairs, Keys, Descriptions),
    param_client(Node, describe_parameters, Client, Options),
    Request = _{names:Keys},
    ros_call(Client, Request, Response, Options),
    Descriptions = Response.descriptors.


%!  ros_node_get_parameter(+Node:atom, +Name, -Value, +Options) is det.
%!  ros_node_get_parameters(+Node:atom, +Dict, +Options) is det.
%
%   Get the values for the  parameters  on   Node.  The  first fetches a
%   single parameter. The second is passed  a dict with variable values.
%   The call fills these values. For example:
%
%      ?- ros_node_get_parameters('/turtlesim', _{background_b:Blue}, []).
%      Blue = 255.

ros_node_get_parameter(Node, Name, Value, Options) :-
    dict_create(Dict, _, [Name=Value]),
    ros_node_get_parameters(Node, Dict, Options).

ros_node_get_parameters(Node, Dict, Options) :-
    dict_pairs(Dict, _, Pairs),
    pairs_keys_values(Pairs, Keys, Values),
    param_client(Node, get_parameters, Client, Options),
    Request = _{names:Keys},
    ros_call(Client, Request, Response, Options),
    maplist(param_value, Response.values, Values).

param_value(Dict, Value) :-
    ros:ros_enum_param_type(Dict.type, Type),
    type_field(Type, Field),
    get_dict(Field, Dict, Value).

%!  ros_node_set_parameter(+Node:atom, +Name, +Value, +Options) is det.
%!  ros_node_set_parameters(+Node:atom, +Dict, +Options) is det.
%
%   Set one or more parameters  on   Node.  The  calling conventions are
%   similar to ros_node_get_parameter/4 and ros_node_get_parameters/3. A
%   Value is either  a  plain  Prolog  value   or  a  term  of  the type
%   Type(Value), e.g. string(true).   The call below makes the turtlesim
%   background yellow:
%
%   ```
%   ?- ros_node_set_parameters('/turtlesim', _{background_r:255,
%                                              background_g:255,
%                                              background_b:0},
%                              []).
%   ```
%
%   Options processed:
%
%     - atomic(+Bool)
%       Perform the setting _atomically_ or not.  In atomic mode either
%       all or none of the parameters is set.  Default is `true`.
%
%   @tbd  Errors  from  the   service    are   currently  printed  using
%   print_message/2. These should be mapped to an exception.

ros_node_set_parameter(Node, Name, Value, Options) :-
    dict_create(Dict, _, [Name=Value]),
    ros_node_set_parameters(Node, Dict, Options).

ros_node_set_parameters(Node, Dict, Options) :-
    option(atomic(true), Options, true),
    !,
    dict_pairs(Dict, _, Pairs),
    maplist(set_param, Pairs, Params),
    param_client(Node, set_parameters_atomically, Client, Options),
    ros_call(Client, _{parameters:Params}, Response, Options),
    maplist(set_param_result, Pairs, Response.results).
ros_node_set_parameters(Node, Dict, Options) :-
    dict_pairs(Dict, _, Pairs),
    maplist(set_param, Pairs, Params),
    param_client(Node, set_parameters, Client, Options),
    ros_call(Client, _{parameters:Params}, Response, Options),
    maplist(set_param_result, Pairs, Response.results).

set_param(Name-Value, _{name:Name, value:ValueDict}) :-
    value_dict(Value, ValueDict).

value_dict(TypedValue, Dict) :-
    compound(TypedValue),
    compound_name_arguments(TypedValue, Type, [Value]),
    type_field(Type, Field),
    ros:ros_enum_param_type(TypeCode, Type),
    !,
    dict_create(Dict, _, [type=TypeCode, Field=Value]).
value_dict(Value, Dict) :-
    value_type(Value, Type),
    type_field(Type, Field),
    ros:ros_enum_param_type(TypeCode, Type),
    !,
    dict_create(Dict, _, [type=TypeCode, Field=Value]).

set_param_result(_, Response) :-
    Response.successful == true,
    !.
set_param_result(Name-Value, Response) :-
    print_message(error,
                  ros(set_parameter(failed, Name, Value, Response.reason))).

%!  param_client(+Node, +Which, -Client, +Options)
%
%   Get a service client to access parameter service Which on Node.

param_client(Node, Which, Client, Options) :-
    atomic_list_concat([Node, Which], /, SrvName),
    srv_type(Which, SrvType),
    debug(ros(parameters), 'Createing client for ~p, type ~p',
          [SrvName, SrvType]),
    ros_client(SrvName, SrvType, Client, Options).



		 /*******************************
		 *             MESSAGES		*
		 *******************************/

:- multifile prolog:message//1.

prolog:message(ros(set_parameter(failed, Name, Value, Reason))) -->
    [ 'ROS: Failed to set parameter ~p to ~p: ~p'-[Name, Value, Reason] ].
