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

:- module(ros_types,
          [ ros_type_introspection/2,   % +Type, -Description
            ros_identifier_prolog/2,    % ?RosName, ?PrologName
            ros_import_type/1,          % :Type
            ros_constant/3              % :Type,?Constant,?Value
          ]).
:- use_module(library(ros/pkg),
              [ ros_package_share_directory/2
              ]).
:- use_module(library(dcg/basics)).
:- use_module(library(apply), [convlist/3]).
:- use_module(library(error),
              [existence_error/2, syntax_error/1, domain_error/2]).
:- use_module(library(lists), [member/2]).
:- use_module(library(readutil), [read_file_to_string/3]).

:- meta_predicate
    ros_import_type(:),
    ros_constant(:, ?, ?).

/** <module> Reason about ROS types

This library provides access to ROS   types and their conversion to/from
Prolog.
*/

%!  ros_type_introspection(+RosType, -Description) is det.
%
%   Describe a ros type using a Prolog term. RosType is either a message
%   or a service. The type description is   formatted as below. All type
%   names are mapped to lowercase,   replacing CamelCase word boundaries
%   with underscores, e.g., `CamelCase` -> `camel_case`.
%
%     - A structure is mapped to a SWI-Prolog dict.  The _tag_ is
%       the structure type name.  The _keys_ represent the fields
%       of the structure and the _values_ the types of the values.
%     - A dynamic array is represented as list(Type)
%     - A fixed array is represented as list(Type, Size)
%     - Primitives are `float`, `double`, `long_double`, `char`,
%       `wchar`, `boolean`, `octet`, `uint8`, `int8`, `uint16`,
%       `int16`, `uint32`, `int32`, `uint64`, `int64`, `string`
%       or `wstring`.
%
%    For example (output edited for readability):
%
%    ```
%    ?- os_type_introspection('rcl_interfaces/msg/Log', T).
%    log{ file:string,
%         function:string,
%         level:uint8,
%         line:uint32,
%         msg:string,
%         name:string,
%         stamp:time{nanosec:uint32,sec:int32}
%       }
%    ```
%
%    When an actual message  is  translated   to/from  Prolog  some  ROS
%    builtin types are handled special. Note  that when translating from
%    Prolot to ROS both the  canonical   representation  and the special
%    representation are accepted.  This list will probably extended, for
%    example for dealing with time.
%
%      - uuid{uuid:list(uint8,16)}
%        This is mapped to/from an atom using the hexadecimal UUID
%        notation that is also used by uuid/1.
%
%    If RosType is a service, Description is   a  dict of type `service`
%    with the keys `request` and `response`.   If  RosType is an action,
%    Description is a dict with keys `goal`, `result` and `feedback`.

ros_type_introspection(Type, Description) :-
    ros:ros_type_support(Type, TypeBlob),
    ros:'$ros_type_introspection'(TypeBlob, Description0),
    simplify_action_type(Description0, Description).

simplify_action_type(action{feedback:_{feedback:Feedback, goal_id:_},
                            goal_request:_{goal:Goal,goal_id:_},
                            goal_response:_,
                            result_request:_,
                            result_response:_{result:Result,status:_}},
                     Description) =>
    Description = action{goal:Goal,
                         result:Result,
                         feedback:Feedback}.
simplify_action_type(TypeIn, TypeOut) =>
    TypeOut = TypeIn.

%!  ros_identifier_prolog(?Ros, ?Prolog) is det.
%
%   Translate  between  ROS  CamelCase  (type)  identifiers  and  Prolog
%   underscore separated identifies.  At  least   one  argument  must be
%   instantiated to an atom. Currently only supports ASCII identifiers.

ros_identifier_prolog(Ros, Prolog) :-
    ros:ros_identifier_prolog(Ros, Prolog).


		 /*******************************
		 *       CONSTANT HANDLING	*
		 *******************************/

:- table ros_constant/3 as shared.

%!  ros_import_type(:Type) is det.
%
%   Import a ROS type into the current module. Type is a fully qualified
%   message type. For example:
%
%       :- ros_import_type('geometry_msgs/msg/Twist').

ros_import_type(Type) :-
    throw(error(context_error(nodirective, ros_import_type(Type)), _)).

:- multifile
    user:term_expansion/2.

user:term_expansion((:- ros_import_type(Type)),
                    [ (:- discontiguous(PI)),
                      Decl
                    ]) :-
    prolog_load_context(module, M),
    strip_module(M:Type, Module, Plain),
    file_base_name(Plain, Local),
    ros_identifier_prolog(Local, Prolog),
    qualify(Module, M, '$ros_type'/3, PI),
    qualify(Module, M, '$ros_type'(Type, Local, Prolog), Decl),
    abolish_table_subgoals(M:'$ros_type'(Type, _, _)).

qualify(M,M,T,T) :- !.
qualify(M,_,T,M:T).


%!  ros_constant(:Type, ?Constant, ?Value) is nondet.
%
%   True when Constant has Value in the message file referenced by Type.
%   The type is normally imported using ros_import_type/1. Alternatively
%   it may be specified as a fully qualified type.
%
%   This goal is subject to goal_expansion/2 if  Constant is an atom and
%   Value  is  unbound.  In  this  case  it  is  expanded  to  `Value  =
%   valueOfConstant`.
%
%   Type can be specified as a fully  qualified ROS type, just the final
%   type name or the Prolog friendly  lowercase version thereof. If Type
%   is unbound all imported types are enumerated.

ros_constant(M:Type, Constant, Value) :-
    full_ros_type(M, Type, FullType),
    ros_type_file(FullType, File),
    msg_file_constants(File, Constants),
    member(Constant=Value, Constants).

full_ros_type(_M, Type, FullType) :-
    atom(Type),
    split_string(Type, "/", "", [_Pkg, _Kind, _LType]),
    !,
    FullType = Type.
full_ros_type(M, Type, FullType) :-
    current_predicate(M:'$ros_type'/3),
    \+ predicate_property(M:'$ros_type'(_,_,_), imported_from(_)),
    (   var(Type)
    ->  FullType = Type,
        M:'$ros_type'(Type,_,_)
    ;   M:'$ros_type'(Type,_,_)
    ->  FullType = Type
    ;   M:'$ros_type'(FullType,Type,_)
    ->  true
    ;   M:'$ros_type'(FullType,_,Type)
    ->  true
    ;   existence_error(ros_type, Type)
    ).

%!  ros_type_file(+Type, -File) is det.
%
%   Get the file that describes a message, service or action.

ros_type_file(Type, File) :-
    split_string(Type, "/", "", [Pkg, Kind, LType]),
    ros_package_share_directory(Pkg, Dir),
    atomic_list_concat([Dir, /, Kind, /, LType, '.', Kind], File),
    (   exists_file(File)
    ->  true
    ;   existence_error(ros_type, Type)
    ).

msg_file_constants(File, Constants) :-
    read_file_to_string(File, String, [encoding(utf8)]),
    split_string(String, "\n\r", "\n\r", Lines),
    convlist(constant, Lines, Constants).

constant(String, Constant) :-
    string_codes(String, Codes),
    phrase(constant(Constant), Codes).

constant(Name=Value) -->
    whites,
    type(Type),
    whites,
    constant_name(Name),
    whites,
    "=",
    !,
    whites,
    (   value(Type, Value),
        end_of_line
    ->  []
    ;   { syntax_error(value_expected) }
    ).

type(Type) -->
    base_type(BaseType), !,
    derived_type(BaseType, Type).
type(extern(Type)) -->
    package(Pkg), "/", message(Msg),
    { atomic_list_concat([Pkg, /, Msg], Type) }.

base_type(bool)    --> "bool".
base_type(byte)    --> "byte".
base_type(float32) --> "float32".
base_type(float64) --> "float64".
base_type(int8)    --> "int8".
base_type(int16)   --> "int16".
base_type(int32)   --> "int32".
base_type(int64)   --> "int64".
base_type(uint8)   --> "uint8".
base_type(uint16)  --> "uint16".
base_type(uint32)  --> "uint32".
base_type(uint64)  --> "uint64".
base_type(string)  --> "string".

derived_type(string, string(Len)) -->
    "<=", uint(Len), !.
derived_type(Type, array(Type)) -->
    "[]", !.
derived_type(Type, array(Type, Len)) -->
    "[<=", uint(Len), "]", !.
derived_type(Type, Type) -->
    [].

package(Name) -->
    lower(C0),
    csymls(Cs),
    { atom_codes(Name, [C0|Cs]) }.

message(Name) -->
    upper(C0),
    csyms(Cs),
    { atom_codes(Name, [C0|Cs]) }.

constant_name(Name) -->
    upper(C0),
    csymus(Cs),
    { atom_codes(Name, [C0|Cs]) }.

lower(C) -->
    [C],
    { between(0'a,0'z,C) }.

csymls([H|T]) -->
    csyml(H), !,
    csymls(T).
csymls([]) --> [].

csyml(C) -->
    [C],
    { C == 0'_ -> true ; between(0'a,0'z,C) }.

csymus([H|T]) -->
    csymu(H), !,
    csymus(T).
csymus([]) --> [].

csymu(C) -->
    [C],
    { C == 0'_ -> true ; between(0'A,0'Z,C) }.

upper(C) -->
    [C],
    { between(0'A,0'Z,C) }.

csyms([H|T]) -->
    csym(H),
    !,
    csyms(T).
csyms([]) -->
    [].

csym(C) -->
    [C],
    { code_type(C, csym) }.

uint(Value) -->
    digit(D0),
    digits(D),
    { number_codes(Value, [D0|D]) }.

%!  value(+Type, -Value)// is semidet.

value(string, Value) -->
    !,
    quote(Q),
    string_chars(Q, Chars),
    quote(Q),
    !,
    { string_codes(Value, Chars) }.
value(string(MaxLen), Value) -->
    !,
    quote(Q),
    string_chars(Q, Chars),
    quote(Q),
    !,
    { string_codes(Value, Chars),
      string_length(Value, Len),
      (   Len =< MaxLen
      ->  true
      ;   domain_error(string(MaxLen), Value)
      )
    }.
value(array(Type), Values) -->
    !,
    "[", values(Type, Values), "]".
value(array(Type, MaxLen), Values) -->
    !,
    "[", values(Type, Values), "]",
    { length(Values, Len),
      (   Len =< MaxLen
      ->  true
      ;   domain_error(array(Type, MaxLen), Values)
      )
    }.
value(IntType, Int) -->
    { int_type(IntType, Bits, Signed) },
    !,
    (   {Signed == signed}
    ->  integer(Int),
        {   MinMax is 1<<(Bits-1),
            Int >= -MinMax,
            Int <  MinMax
        ->  true
        ;   domain_error(IntType, Int)
        }
    ;   uint(Int),
        {   Max is 1<<Bits,
            Int <  Max
        ->  true
        ;   domain_error(IntType, Int)
        }
    ).
value(float32, Float) -->
    float(Float).
value(float64, Float) -->
    float(Float).

values(_, []), "]" -->
    whites, "]".
values(Type, [H|T]) -->
    whites,
    value(Type, H),
    whites,
    (   ","
    ->  values(Type, T)
    ;   values(Type, T)
    ).

int_type(int8,   8,  signed).
int_type(int16,  16, signed).
int_type(int32,  32, signed).
int_type(int64,  64, signed).
int_type(uint8,  8,  unsigned).
int_type(uint16, 16, unsigned).
int_type(uint32, 32, unsigned).
int_type(uint64, 64, unsigned).

quote(0'\') --> "'".
quote(0'\") --> "\"".

string_chars(_, []) --> [].
string_chars(Q, [Q|T]) -->
    "\\", quote(Q), !,
    string_chars(Q, T).
string_chars(Q, [H|T]) -->
    [H],
    string_chars(Q, T).

end_of_line -->
    whites,
    (   "#"
    ->  remainder(_)
    ;   []
    ).

:- multifile
    user:goal_expansion/2.

user:goal_expansion(ros_constant(Type, Constant, Var),
                    Var = Value) :-
    var(Var),
    atom(Constant),
    prolog_load_context(module, M),
    findall(Type-Value, ros_constant(M:Type, Constant, Value), Pairs0),
    sort(2, @>, Pairs0, Pairs),
    (   Pairs = [_-Value]
    ->  true
    ;   Pairs == []
    ->  print_message(warning,
                      error(existence_error(ros_constant(Type), Constant),
                            _)),
        fail
    ;   pairs_keys(Pairs, Types),
        print_message(warning,
                      error(ambiguity_error(ros_constant(Constant), Types),
                            _)),
        fail
    ).

		 /*******************************
		 *            MESSAGES		*
		 *******************************/

:- multifile
    prolog:error_message//1.

prolog:error_message(ambiguity_error(ros_constant(Constant), Types)) -->
    [ 'ROS constant ~p is defined on multiple types.'-[Constant], nl,
      'Please specify the type.'-[]
    ],
    list_types(Types).

list_types([]) --> [].
list_types([H|T]) -->
    [ nl, '  ~p'-[H] ],
    list_types(T).
