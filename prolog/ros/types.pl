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
            ros_identifier_prolog/2     % ?RosName, ?PrologName
          ]).
:- use_module(library(ros), []).

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
