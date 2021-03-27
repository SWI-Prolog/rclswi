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

:- module(ros_param_detail_common,
          [ variant_value/2,            % +Dict, -Value
            value_type/2,               % +Value, -Type
            type_field/2,               % ?Type, ?Field
            srv_type/2,                 % ?ParamService, ?Type
            is_valid_param_type/1,	% @Type
            typecheck_parameter_value/2 % +Type, +Value
          ]).
:- use_module(library(apply)).
:- use_module(library(error)).

/** <module> Common functionality for dealing with ROS parameters

This library provides common  functionality   to  the  various parameter
related modules.
*/

%!  variant_value(?Dict, ?Value) is det.
%
%   Map      between      a       Prolog        value       and       an
%   ``rcl_interfaces/msg/ParameterValue`` term. When mapping from Prolog
%   to message, Value can  be  of   the  share  Type(Value),  to resolve
%   ambiguities. E.g., string(Atom) may be  used   to  ensure  `true` is
%   passed as a string rather than a boolean.

variant_value(Dict, Value) :-
    nonvar(Dict),
    !,
    ros:ros_enum_param_type(Dict.type, Type),
    type_field(Type, Field),
    get_dict(Field, Dict, Value).
variant_value(Dict, TypedValue) :-
    compound(TypedValue),
    compound_name_arguments(TypedValue, Type, [Value]),
    type_field(Type, Field),
    ros:ros_enum_param_type(TypeCode, Type),
    !,
    dict_create(Dict, _, [type=TypeCode, Field=Value]).
variant_value(Dict, Value) :-
    var(Value),
    !,
    ros:ros_enum_param_type(TypeCode, not_set),
    !,
    dict_create(Dict, _, [type=TypeCode]).
variant_value(Dict, Value) :-
    value_type(Value, Type),
    type_field(Type, Field),
    ros:ros_enum_param_type(TypeCode, Type),
    !,
    dict_create(Dict, _, [type=TypeCode, Field=Value]).

%!  value_type(+Value, -Type) is det.
%
%   Deduce the parameter type from a given value.

value_type(Value, Type), is_list(Value) =>
    maplist(value_type1, Value, Types),
    sort(Types, Unique),
    (   Unique = [One]
    ->  array_type(One, Type)
    ;   Unique = [bool,string]
    ->  Type = string_array
    ;   type_error(ros_parameter, Value)
    ).
value_type(Value, Type) =>
    value_type1(Value, Type).

value_type1(Value, Type), integer(Value)           => Type = integer.
value_type1(Value, Type), atom(Value), bool(Value) => Type = bool.
value_type1(Value, Type), float(Value)             => Type = double.
value_type1(Value, Type), string(Value)            => Type = string.
value_type1(Value, Type), atom(Value)              => Type = string.

bool(true).
bool(false).

array_type(bool,    bool_array).
array_type(integer, integer_array).
array_type(double,  double_array).
array_type(string,  string_array).

%!  typecheck_parameter_value(+Type, +Value)
%
%   Validate that Value satisfies type

typecheck_parameter_value(bool, Value) =>
    must_be(bool, Value).
typecheck_parameter_value(integer, Value) =>
    must_be(between(-9223372036854775808, 9223372036854775807), Value).
typecheck_parameter_value(integer(Low, High), Value) =>
    must_be(between(Low, High), Value).
typecheck_parameter_value(integer(Low, High, Step), Value) =>
    must_be(between(Low, High), Value),
    (   Step == 0
    ->  true
    ;   (Value-Low) mod Step =:= 0
    ->  true
    ;   domain_error(integer(Low, High, Step), Value)
    ).
typecheck_parameter_value(double, Value) =>
    must_be(float, Value).
typecheck_parameter_value(double(Low, High), Value) =>
    LowF is float(Low),
    HighF is float(High),
    must_be(between(LowF, HighF), Value).
typecheck_parameter_value(double(Low, High, Step), Value) =>
    LowF is float(Low),
    HighF is float(High),
    must_be(between(LowF, HighF), Value),
    (   Step =:= 0.0
    ->  true
    ;   float_fractional_part((Value-Low) / Step) < Step/1000.0
    ->  true
    ;   domain_error(float(Low, High, Step), Value)
    ).
typecheck_parameter_value(string, Value) =>
    must_be(text, Value).               % Do we really allow for codes/chars?
typecheck_parameter_value(byte_array, Value) =>
    must_be(list(between(0,255)), Value).
typecheck_parameter_value(bool_array, Value) =>
    must_be(list(bool), Value).
typecheck_parameter_value(integer_array, Value) =>
    must_be(list(between(-9223372036854775808, 9223372036854775807)), Value).
typecheck_parameter_value(double_array, Value) =>
    must_be(list(float), Value).
typecheck_parameter_value(string_array, Value) =>
    must_be(list(text), Value).
typecheck_parameter_value(Type, _) =>
    existence_error(ros_parameter_type, Type).

%!  is_valid_param_type(@Type) is det.
%
%   Succeeds if Type a valid parameter type.

is_valid_param_type(bool) => true.
is_valid_param_type(integer) => true.
is_valid_param_type(integer(Low,High)),
    integer(Low), integer(High), Low =< High => true.
is_valid_param_type(integer(Low,High,Step)),
    integer(Low), integer(High), Low =< High, integer(Step) => true.
is_valid_param_type(double) => true.
is_valid_param_type(double(Low,High)),
    number(Low), number(High), Low =< High => true.
is_valid_param_type(double(Low,High,Step)),
    number(Low), number(High), Low =< High, number(Step) => true.
is_valid_param_type(string) => true.
is_valid_param_type(byte_array) => true.
is_valid_param_type(bool_array) => true.
is_valid_param_type(integer_array) => true.
is_valid_param_type(double_array) => true.
is_valid_param_type(string_array) => true.
is_valid_param_type(X), var(X) => instantiation_error(X).
is_valid_param_type(X) => type_error(ros_parameter_type, X).

%!  type_field(?Type, ?Field)
%
%   Map between a parameter type and the   struct member where the value
%   can be found.

type_field(bool,          bool_value).
type_field(integer,       integer_value).
type_field(double,        double_value).
type_field(string,        string_value).
type_field(byte_array,    byte_array_value).
type_field(bool_array,    bool_array_value).
type_field(integer_array, integer_array_value).
type_field(double_array,  double_array_value).
type_field(string_array,  string_array_value).

%!  srv_type(?ParamService, ?Type)
%
%   Relates the various parameter services to their types.

srv_type(describe_parameters,       'rcl_interfaces/srv/DescribeParameters').
srv_type(get_parameter_types,       'rcl_interfaces/srv/GetParameterTypes').
srv_type(get_parameters,            'rcl_interfaces/srv/GetParameters').
srv_type(list_parameters,           'rcl_interfaces/srv/ListParameters').
srv_type(set_parameters,            'rcl_interfaces/srv/SetParameters').
srv_type(set_parameters_atomically, 'rcl_interfaces/srv/SetParametersAtomically').
