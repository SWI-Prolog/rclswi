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

:- module(ros,
          [ ros_set_defaults/1,         % +List
            ros_spin/0,
            ros_spin/1,                 % +Node
            ros_shutdown/0,

            ros_subscribe/3,            % +Topic, :CallBack, +Options
            ros_unsubscribe/1,          % +Topic

            ros_current_topic/2,        % ?Topic,?Type
            ros_type_introspection/2,   % +Type, -Description

            ros_create_context/1,       % -Context
            ros_init/3,                 % +Context, +Args, ?DomainID
            ros_create_node/4,          % +Context, +Name, -Node, +Options
            ros_node_property/2,        % +Node,?Property

            ros_publish/5,              % +Node, +MsgType, +Topic, +QoSProfile, -Publisher
            ros_subscribe/5,            % +Node, +MsgType, +Topic, +QoSProfile, -Subscription

            ros_wait/3,                 % +WaitSet, +TimeOut, -Ready

            ros_take/3,			% +Subscription, -Message, -MessageInfo

            ros_client_names_and_types_by_node/4, % +Node, +NodeName, +NameSpace, -NamesAndTypes
            ros_property/1,             % ?Property
            ros_debug/1,                % +Level
            ros_default_context/1,      % -Context
            ros_default_node/1          % -Node
          ]).

:- use_module(library(apply)).
:- use_module(library(error)).
:- use_module(library(lists)).
:- use_module(library(prolog_code)).
:- use_module(library(option)).

:- meta_predicate
    ros_subscribe(+, 1, +).

:- use_foreign_library(foreign(librclswi)).

/** <module> ROS2 client library

This module provides a ROS2 client   library for SWI-Prolog. The library
builds on top of the ROS2 low-level C libraries `rcl` and `rmw` and uses
type introspection to dynamically translate  ROS2 messages to SWI-Prolog
dict objects.
*/

type_support_function_prefix(
    rosidl_typesupport_c__get_message_type_support_handle).
type_introspection_function_prefix(
    rosidl_typesupport_introspection_c__get_message_type_support_handle).


		 /*******************************
		 *        HIGH LEVEL API	*
		 *******************************/

:- dynamic
    waitable/4.                     % Type, Node, Object, CallBack

%!  ros_subscribe(+Topic, :CallBack, +Options) is det.
%
%   Subscribe to the given ROS Topic. When   a  message arrives on Topic
%   call(CallBack, Message). The callback  is   executed  in  the thread
%   calling ros_spin/0,1, which can  be  a   different  thread  than the
%   thread that registers the subscription.  Options:
%
%     - message_type(+MessageType)
%       Provide the message type.  Default is to inspect the ROS graph
%       to obtain the MessageType.  See ros_current_topic/2.  Providing
%       the type explicitly can be used to subscribe while no publisher
%       exists (yet) as well as to be explicit about what you expect.
%
%     - node(+Node)
%       Specify the node to use.  Default is the node returned by
%       ros_default_node/1.
%
%     - qos_profile(+QoSProfile)
%       Quality of Service profile.  Not yet defined or implemented.

ros_subscribe(Topic, CallBack, Options) :-
    message_type(Topic, MsgType, Options),
    node(Node, Options),
    ros_type_support(MsgType, TypeSupport),
    qos_profile(QoSProfile, Options),
    '$ros_subscribe'(Node, TypeSupport, Topic, QoSProfile, Subscription),
    assert(waitable(subscription(Topic), Node, Subscription, CallBack)).

message_type(_Topic, MsgType, Options) :-
    option(message_type(MsgType), Options),
    !.
message_type(Topic, MsgType, _Options) :-
    ros_current_topic(Topic, MsgType),
    !.
message_type(Topic, _, _) :-
    existence_error(topic, Topic).

node(Node, Options) :-
    option(node(Node), Options),
    !.
node(Node, _) :-
    ros_default_node(Node).

qos_profile(_QoSProfile, _Options).

%!  ros_unsubscribe(+Topic) is semidet.
%
%   Unsubscribe from Topic. Fails silently if   no subscription on Topic
%   is known.

ros_unsubscribe(Topic) :-
    retract(waitable(subscription(Topic), _Node, Subscription, _CallBack)),
    !,
    '$ros_unsubscribe'(Subscription).

%!  ros_spin is det.
%!  ros_spin(+Node) is det.
%
%   Wait for all waitable objects  registered   with  Node  and call the
%   callbacks associated with the ready  objects.   If  no callbacks are
%   registered this call waits for new  registrations. Two use cases are
%   envisioned:
%
%     - Use a single thread that registers waitable objects
%       (subscriptions, timers, services, etc.) and then calls
%       ros_spin/0,1 to wait for ready objects and process there
%       callbacks.
%     - Start ros_spin/0,1 in a thread and register waitable objects
%       from different threads, for example, with the call below.
%       This option is notably practical for interactive usage.
%
%           ?- thread_create(ros_spin, _, [alias(ros_spinner)]).

ros_spin :-
    ros_default_node(Node),
    ros_spin(Node).

ros_spin(Node) :-
    ros_ok,
    !,
    (   ros_spin_once(Node, infinite)
    ->  true
    ;   thread_wait(waitables_on(Node),
                    [ wait_preds([waitable/4])
                    ])
    ),
    ros_spin(Node).
ros_spin(_).

waitables_on(Node) :-
    waitable(_Type, Node, _Obj, _Callback),
    !.


%!  ros_spin_once(+Node, +TimeOut) is semidet.
%
%   Wait for all waitable objects  registered   with  Node  and call the
%   callbacks associated with the ready objects.   Fails if there are no
%   objects to wait for or TimeOut is exceeded.

ros_spin_once(Node, TimeOut) :-
    findall(Obj, waitable(_Type, Node, Obj, _Callback), WaitFor),
    WaitFor \== [],
    ros_wait(WaitFor, TimeOut, Ready),
    maplist(ros_ready, Ready).

ros_ready(Obj) :-
    waitable(Type, _Node, Obj, CallBack),
    !,
    ros_ready(Type, Obj, CallBack).

ros_ready(subscription(_), Subscription, CallBack) :-
    ros_take(Subscription, Message, _MsgInfo),
    catch_with_backtrace(
        ignore(call(CallBack, Message)),
        Error,
        print_message(warning, Error)).


		 /*******************************
		 *        GLOBAL OBJECTS	*
		 *******************************/

%!  ros_set_defaults(+List)
%
%   Set global defaults for the ROS environment.  Defined defaults are
%
%     - init(List)
%       Set the arguments to initialize the default ROS context.
%     - domain(Integer)
%       Set the ROS domain
%     - node(Name, Options)
%       Set the name of the default node and the options to create it
%
%   @tbd Create default arguments from command line.

:- dynamic
    ros_default/1,                      % Term
    ros_context_store/1,                % Context
    ros_node_store/2,                   % Node, Context
    ros_property_store/1.               % Term

ros_set_defaults(List) :-
    must_be(list, List),
    maplist(set_ros_default, List).

set_ros_default(Term) :-
    check_default(Term),
    most_general_goal(Term, General),
    retractall(ros_default(General)),
    assertz(ros_default(Term)).

check_default(Term) :-
    most_general_goal(Term, General),
    (   ros_default(General, _Comment)
    ->  (   atom(General)
        ->  true
        ;   compound_name_arity(General, _, Arity),
            forall(between(1, Arity, ArgN),
                   check_arg(ArgN, Term, General))
        )
    ;   existence_error(ros_default, Term)
    ).

check_arg(I, Term, Types) :-
    arg(I, Term, Value),
    arg(I, Types, Type),
    must_be(Type, Value).

ros_default(init(list(atomic)),       "ROS context initialization arguments").
ros_default(domain(integer),          "ROS domain").
ros_default(node(atom, list(atomic)), "ROS default node initialization arguments").

%!  ros_default_context(-Context)
%
%   Default context to use for ROS operations.

ros_default_context(Context) :-
    ros_context_store(Context0),
    !,
    Context = Context0.
ros_default_context(Context) :-
    var(Context),
    with_mutex(ros, ros_default_context_sync(Context)).

ros_default_context_sync(Context) :-
    ros_context_store(Context),
    !.
ros_default_context_sync(Context) :-
    (   ros_default(init(Args))
    ->  true
    ;   Args = []
    ),
    ignore(ros_default(domain(Domain))),
    ros_create_context(Context),
    ros_init(Context, Args, Domain),
    asserta(ros_context_store(Context)),
    asserta(ros_property_store(domain(Domain))).

%!  ros_default_node(-Node)
%
%   Default node to use. The node is  created in the context provided by
%   ros_default_context/1. The name and arguments to create the node are
%   provided by ros_set_defaults/1. If  no   defaults  are  provided the
%   default node name is ``swip-prolog-<number>``, where ``<number>`` is
%   a large random number.

ros_default_node(Node) :-
    ros_node_store(Node0, _),
    !,
    Node = Node0.
ros_default_node(Node) :-
    var(Node),
    with_mutex(ros, ros_default_node_sync(Node)).

ros_default_node_sync(Node) :-
    ros_node_store(Node, _),
    !.
ros_default_node_sync(Node) :-
    default_node_name_and_arguments(Name, Args),
    ros_default_context(Context),
    ros_create_node(Context, Name, Node, Args),
    sleep(0.2),                     % TBD: Needed to make the node visible
    asserta(ros_node_store(Node, Context)).

default_node_name_and_arguments(Name, Args) :-
    ros_default(node(Name, Args)),
    !.
default_node_name_and_arguments(Name, []) :-
    Id is random(1<<63),
    atom_concat('swi_prolog_', Id, Name).

%!  ros_shutdown is det.
%!  ros_shutdown(+Context) is det.
%
%   Shut down all or a specific context.

ros_shutdown :-
    forall(ros_context_store(Context),
           ros_shutdown(Context)).

ros_shutdown(Context) :-
    forall(ros_node_store(Node, Context),
           ros_shutdown_node(Node)),
    forall(ros_context_store(Context),
           '$ros_shutdown'(Context)).

ros_shutdown_node(Node) :-
    forall(retract(waitable(Type, Node, Object, _CallBack)),
           shutdown_waitable(Type, Object)),
    ros_node_fini(Node).

shutdown_waitable(subscription(_Topic), Object) :-
    '$ros_unsubscribe'(Object).

:- at_halt(ros_shutdown).


		 /*******************************
		 *        INTROSPECTION		*
		 *******************************/

%!  ros_current_topic(?Topic, ?Type)
%
%   True when Topic with Type is visible on the ROS network.

ros_current_topic(Topic, Type) :-
    ros_default_node(Node),
    ros_topic_names_and_types(Node, TopicsAndTypes),
    member(Topic-[Type], TopicsAndTypes).


		 /*******************************
		 *            LOW LEVEL		*
		 *******************************/

%!  ros_create_context(-Context) is det.
%
%   Create a default ROS context.  The Context is subject to Prolog
%   garbage collection.

%!  ros_init(+Context, +Args, ?DomainID)
%
%   Initialize the ROS environment.
%
%   @arg Context is a context as produced by ros_create_context/1
%   @arg Args is a list of atoms or strings to initialise the context
%   @arg DomainID is the ROS domain.

%!  ros_ok is semidet.
%
%   True when ROS is initialized and not shut down.

ros_ok :-
    ros_context_store(Context),
    ros_ok(Context).


%!  ros_create_node(+Context, +Name, -Node, +Options)

%!  ros_node_property(+Node, ?Property) is nondet.
%
%   True when Property is a  property  of   the  ROS  node Node. Defined
%   properties are:
%
%     - name(Name)
%     - namespace(Namespace)

ros_node_property(Node, Property) :-
    ros_property_node(Property, Node).

ros_property_node(name(Name), Node) :-
    '$ros_node_prop'(Node, name, Name).
ros_property_node(namespace(Name), Node) :-
    '$ros_node_prop'(Node, namespace, Name).

%!  ros_publish(+Node, +MsgType, +Topic, +QoSProfile, -Publisher)
%
%   @tbd: Not yet working (lacking type support)

ros_publish(Node, MsgType, Topic, QoSProfile, Publisher) :-
    ros_type_support(MsgType, TypeSupport),
    '$ros_publish'(Node, TypeSupport, Topic, QoSProfile, Publisher).

%!  ros_subscribe(+Node, +MsgType, +Topic, +QoSProfile, -Subscription)
%
%   @tbd: Not yet working (lacking type support)

ros_subscribe(Node, MsgType, Topic, QoSProfile, Subscription) :-
    ros_type_support(MsgType, TypeSupport),
    '$ros_subscribe'(Node, TypeSupport, Topic, QoSProfile, Subscription).

%!  ros_type_support(+MsgType, -TypeSupport)
%
%   Get the RCL type support  description   from  MsgType. MsgType is an
%   atom or term of the form Package/Kind/Type. For example:
%
%      ?- ros_type_support('std_msgs/msg/Int64', Type).
%      Type = <rcl_type_support>(0x7fbc66896210).
%
%   @tbd We plan to make this more Prolog friendly by providing defaults
%   for the Package and Kind  components   and  case  conversion for the
%   actual type.

:- table ( ros_type_support/2,
           load_type_support_shared_object/3
         ) as shared.

ros_type_support(MsgType, MsgFunctions) :-
    type_support_function_prefix(TSPrefix),
    type_introspection_function_prefix(ISPrefix),
    type_support_function(MsgType, Package, FuncPostfix),
    load_type_support_shared_object(Package, rosidl_typesupport_c, _),
    load_type_support_shared_object(Package, rosidl_generator_c, _),
    load_type_support_shared_object(Package, rosidl_typesupport_introspection_c, _),
    atomic_list_concat([TSPrefix, FuncPostfix], '__', TSFunc),
    atomic_list_concat([ISPrefix, FuncPostfix], '__', ISFunc),
    '$ros_message_type'(ISFunc, TSFunc, FuncPostfix, MsgFunctions).

type_support_function(MsgType, Package, FuncPostfix) :-
    phrase(segments(MsgType), Segments),
    atomics_to_string(Segments, '__', FuncPostfix),
    func_package(FuncPostfix, Package).

segments(Var) -->
    { var(Var),
      instantiation_error(Var)
    }.
segments(A/B) -->
    !,
    segments(A),
    segments(B).
segments(A) -->
    { split_string(A, "/", "", Parts) },
    (   {Parts = [One]}
    ->  [One]
    ;   {atomic_list_concat(Parts, '__', Segment)},
        [Segment]
    ).

func_package(Func, Package) :-
    sub_atom(Func, Pre, _, _, '__'),
    !,
    sub_atom(Func, 0, Pre, _, Package).

load_type_support_shared_object(Package, Which, Handle) :-
    current_prolog_flag(shared_object_extension, SO),
    atomic_list_concat([lib,Package,'__',Which, '.', SO], File),
    open_shared_object(File, Handle, [global]).

%!  ros_type_introspection(+MsgType, -Description) is det.
%
%   Describe MsgType using a Prolog term.   Description  is formatted as
%   below. All type names are mapped to lowercase.
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

ros_type_introspection(MsgType, Description) :-
    ros_type_support(MsgType, MsgFunctions),
    '$ros_type_introspection'(MsgFunctions, Description).


%!  ros_client_names_and_types_by_node(+Node, +NodeName, +NameSpace, -NamesAndTypes)
%
%   Example:
%
%   ```
%   ?- ros_client_names_and_types_by_node(Node, "teleop_turtle", "", Types).
%   Types = [ '/turtle1/rotate_absolute/_action/cancel_goal'-
%                 ['action_msgs/srv/CancelGoal'],
%             '/turtle1/rotate_absolute/_action/get_result'-
%                 ['turtlesim/action/RotateAbsolute_GetResult'],
%             '/turtle1/rotate_absolute/_action/send_goal'-
%                 ['turtlesim/action/RotateAbsolute_SendGoal']
%           ].
%   ```


%!  ros_property(?Property)
%
%   Enumerate properties about the current   ROS  environment. Currently
%   defined:
%
%    - rmw_identifier(Id)

ros_property(rmw_identifier(Id)) :-
    ros_rwm_implementation(Id).
