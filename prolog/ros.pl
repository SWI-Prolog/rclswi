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
            ros_spin/1,                 % +Options
            ros_spin_once/1,            % Options
            ros_shutdown/0,

            ros_subscribe/3,            % +Topic, :CallBack, +Options
            ros_unsubscribe/2,          % +Topic, +Options

            ros_publish/2,              % +Topic, +Message
            ros_publish/3,              % +Topic, +Message, +Options
            ros_publisher/2,            % +Topic, +Options

            ros_create_node/3,          % +Name, -Node, +Options
            ros_node_property/2,        % +Node, ?Property

            ros_wait/3,                 % +WaitSet, +TimeOut, -Ready

            ros_property/1,             % ?Property
            ros_default_context/1,      % -Context
            ros_default_node/1,         % -Node
            ros_node/2,                 % +Alias, -Node

            ros_object/2,               % ?Object, ?Type
            ros_synchronized/2,         % +Object, :Goal
            ros_create_guard_condition/2,  % -Cond, +Options
            ros_trigger_guard_condition/1, % +Cond
            ros_debug/1                    % +Level
          ]).
:- autoload(library(error),
            [ must_be/2,
              existence_error/2,
              domain_error/2,
              instantiation_error/1,
              type_error/2,
              permission_error/3
            ]).
:- autoload(library(apply), [maplist/2]).
:- autoload(library(lists), [append/3]).
:- autoload(library(option), [option/2, select_option/3, option/3]).
:- autoload(library(prolog_code), [most_general_goal/2]).
:- use_module(library(filesex), [directory_file_path/3]).
:- use_module(library(debug), [debug/3]).

:- use_module(library(ros/detail/options)).
:- autoload(library(ros/param/services), [ros_param_services/1]).
:- autoload(library(ros/param/store), [ros_parameter/2, import_parameters/1]).
:- autoload(library(ros/graph), [ros_current_topic/2]).

:- meta_predicate
    ros_subscribe(+, 1, +),
    ros_synchronized(+, 0).

:- predicate_options(ros_spin/1, 1,
                     [ node(any),
                       thread(atom)
                     ]).
:- predicate_options(ros_spin_once/1, 1,
                     [ node(any),
                       timeout(any)
                     ]).
:- predicate_options(ros_subscribe/3, 3,
                     [ node(any),
                       message_type(atom),
                       qos_profile(any)
                     ]).
:- predicate_options(ros_publisher/2, 2,
                     [ node(any),
                       message_type(atom),
                       qos_profile(any)
                     ]).
:- predicate_options(ros_publish/3, 3,
                     [ node(any),
                       pass_to(ros_publisher/2, 2)
                     ]).


:- dynamic ros_lib_dir/1.
:- public unique_pred/0.

unique_pred.

user:file_search_path(foreign, LibDir) :-
    (   ros_lib_dir(LibDir)
    ->  true
    ;   source_file(unique_pred, ThisFile),
        file_directory_name(ThisFile, ThisDir),
        file_directory_name(ThisDir, RclSWIDir),
        directory_file_path(RclSWIDir, lib, LibDir),
        asserta(ros_lib_dir(LibDir))
    ).

:- use_foreign_library(foreign(librclswi)).

/** <module> ROS2 client library

This module provides a ROS2 client   library for SWI-Prolog. The library
builds on top of the ROS2 low-level C libraries `rcl` and `rmw` and uses
type introspection to dynamically translate  ROS2 messages to SWI-Prolog
dict objects. Because most applications need   only  part of this client
and/or server functionality, the ROS2  library   is  split into many sub
libraries:

  | Library      | Summary |
  |---|---|
  | library(ros) | This library |
  | library(ros/logging) | Send ROS log messages |
  | library(ros/graph) | Query nodes, topics, services and actions |
  | library(ros/types) | Query message types associated with topics, services and actions |
  | library(ros/qos) | Create and manage Quality of Service descriptions |
  | library(ros/clocks) | Deal with ROS clocks |
  | library(ros/services) | Client and server side for ROS services |
  | library(ros/param/client) | Read and set parameters in other nodes |
  | library(ros/param/store) | Access parameters of this node |
  | library(ros/param/services) | Provide remote access to our parameters |
  | library(ros/action/client) | Create and monitor a goal on an action server |
  | library(ros/action/server) | Create an action server |

This documentation does not provide an  overview on how these predicates
can be combined to build a ROS node.  For now we refer to the `examples`
directory in the `rclswi` source package.

__WARNING__

> The API should not be considered fully stable.  Changes that affect users
> will be tagged as __MODIFIED:__ in the Git commit log.
> If you decide to try this library, please discuss its usage, either on
> the ROS forum or on the [SWI-Prolog forum](https://swi-prolog.discourse.group/)
*/

type_support_function_prefix(msg,
    rosidl_typesupport_c__get_message_type_support_handle).
type_support_function_prefix(srv,
    rosidl_typesupport_c__get_service_type_support_handle).
type_support_function_prefix(action,
    rosidl_typesupport_c__get_action_type_support_handle).
type_introspection_function_prefix(
    rosidl_typesupport_introspection_c__get_message_type_support_handle).


		 /*******************************
		 *        HIGH LEVEL API	*
		 *******************************/

:- dynamic
    waitable/4,                     % Type, Node, Object, CallBack
    node_object/3.                  % Type, Node, Object

%!  ros_subscribe(+Topic, :CallBack, +Options) is det.
%
%   Subscribe to the given ROS Topic. When   a  message arrives on Topic
%   call(CallBack, Message). The callback  is   executed  in  the thread
%   calling ros_spin/0,1, which can  be  a   different  thread  than the
%   thread that registers the subscription.  Options:
%
%     - node(+Node)
%       Specify the node to use.  Default is the node returned by
%       ros_default_node/1.
%     - message_type(+MessageType)
%       Provide the message type.  Default is to inspect the ROS graph
%       to obtain the MessageType.  See ros_current_topic/2.  Providing
%       the type explicitly can be used to subscribe while no publisher
%       exists (yet) as well as to be explicit about what you expect.
%     - qos_profile(+QoSProfile)
%       Quality of Service profile.  Not yet defined or implemented.
%
%   If the specified subscription is already defined  this is a no-op. A
%   subscription is considered equal of the   Topic, `Node` and CallBack
%   match. Thus, one can subscribe  multiple   times  to  the same Topic
%   using different callbacks.

ros_subscribe(Topic, CallBack, Options) :-
    must_be(atom, Topic),
    node_from_options(Node, Options),
    ros_synchronized(Node, ros_subscribe_sync(Node, Topic, CallBack, Options)).

ros_subscribe_sync(Node, Topic, CallBack, _Options) :-
    waitable(subscription(Topic), Node, _Subscription, CallBackNow),
    CallBack =@= CallBackNow,
    !.
ros_subscribe_sync(Node, Topic, CallBack, Options) :-
    message_type(Topic, MsgType, Options),
    ros_msg_type_support(MsgType, TypeSupport),
    qos_profile_from_options(QoSProfile, Options),
    '$ros_subscribe'(Node, TypeSupport, Topic, QoSProfile, Subscription),
    register_waitable(subscription(Topic), Node, Subscription, CallBack).

message_type(_Topic, MsgType, Options) :-
    option(message_type(MsgType), Options),
    !.
message_type(Topic, MsgType, _Options) :-
    ros_current_topic(Topic, MsgType),
    !.
message_type(Topic, _, _) :-
    existence_error(topic, Topic).

%!  ros_unsubscribe(+Topic, +Options) is det.
%
%   Unsubscribe from Topic.  Options:
%
%     - node(+Node)
%       Only unsubscribe subscriptios on the specified node.  When
%       omitted the topic is unsubscribed from all nodes.

ros_unsubscribe(Topic, Options) :-
    must_be(atom, Topic),
    (   option(node(_), Options)
    ->  node_from_options(Node, Options)
    ;   true
    ),
    forall(retract(waitable(subscription(Topic), Node, Subscription, _CallBack)),
           '$ros_unsubscribe'(Subscription)).

%!  ros_publish(+Topic, +Message) is det.
%!  ros_publish(+Topic, +Message, +Options) is det.
%
%   Send a message to Topic. If no  publisher exists it it created based
%   on Options passed to ros_publisher/2. Other options processed:
%
%     - node(+Node)
%       Specify the node.  When omitted the first node for which
%       a publisher on Topic was registered is used.

ros_publish(Topic, Message) :-
    ros_publish(Topic, Message, []).

ros_publish(Topic, Message, Options) :-
    node_from_options(Node, Options),
    (   node_object(subscription(Topic), Node, Subscription)
    ->  true
    ;   ros_synchronized(Node,
                         ros_publisher_(Node, Topic, Subscription, Options))
    ),
    '$ros_publish'(Subscription, Message).


%!  ros_publisher(+Topic, +Options) is det.
%
%   Register a publisher for Topic. This is  a no-op if Topic is already
%   registered. Options:
%
%     - node(+Node)
%       Node to use.  Defaults to ros_default_node/1.
%     - message_type(+Type)
%       Message type.  When omitted ros_current_topic/2 is used to try
%       and infer the type.
%     - qos_profile(+QoSProfile)
%       Quality of Service profile.  Not yet defined or implemented.

ros_publisher(Topic, Options) :-
    node_from_options(Node, Options),
    ros_synchronized(Node, ros_publisher_(Node, Topic, _, Options)).

ros_publisher_(Node, Topic, Subscription, Options) :-
    (   node_object(subscription(Topic), Node, Subscription)
    ->  true
    ;   message_type(Topic, MsgType, Options),
        ros_msg_type_support(MsgType, TypeSupport),
        qos_profile_from_options(QoSProfile, Options),
        '$ros_publisher'(Node, TypeSupport, Topic, QoSProfile, Subscription),
        assert(node_object(subscription(Topic), Node, Subscription))
    ).

%!  register_waitable(+Type, +Node, +Object, +Callback) is det.
%
%   Register a waitable object with Node. If   we  run (_spin_) the node
%   this calls ros_ready(Type, Object, CallBack)

register_waitable(Type, Node, Object, Callback) :-
    assert(waitable(Type, Node, Object, Callback)).

%!  ros_spin is det.
%!  ros_spin(+Options) is det.
%
%   Wait for all waitable objects registered with   a  node and call the
%   callbacks associated with the ready  objects.   If  no callbacks are
%   registered this call waits for new  registrations.  Options:
%
%     - node(+Node)
%       The node to use
%     - thread(TrueOrAlias)
%       Run the spinner in a thread. If `true`, the thread is anonymous.
%       This is a no-op if the target node already has a spinning
%       thread.
%
%   @error permission_error(spin, ros_node, Node) if   another thread is
%   spinning on this node.

ros_spin :-
    ros_spin([]).

ros_spin(Options) :-
    node_from_options(Node, Options),
    (   option(thread(TrueAlias), Options)
    ->  (   TrueAlias == true
        ->  true
        ;   Alias = TrueAlias
        ),
        ros_synchronized(Node, create_spin_thread(Node, Alias))
    ;   ros_node_spin(Node)
    ).

create_spin_thread(Node, Alias) :-
    ros_node_property(Node, spinner(Alias)).
create_spin_thread(Node, Alias) :-
    var(Alias),
    !,
    thread_create(ros_node_spin(Node), _, [detached(true)]).
create_spin_thread(Node, Alias) :-
    thread_create(ros_node_spin(Node), _, [alias(Alias)]).


ros_node_spin(Node) :-
    setup_call_cleanup(
        '$ros_set_node'(Node, spinner(true)),
        ros_node_spin_loop(Node),
        '$ros_set_node'(Node, spinner(false))).

ros_node_spin_loop(Node) :-
    ros_node_store(_Name, Node, _Context, _Mutex, Guard),
    ros_ok,                             % TBD: wait on this node to be shut down
    !,
    (   ros_spin_once(Node, [Guard], infinite)
    ->  true
    ;   thread_wait(waitables_on(Node),
                    [ wait_preds([waitable/4])
                    ])
    ),
    ros_node_spin_loop(Node).
ros_node_spin_loop(_).

waitables_on(Node) :-
    waitable(_Type, Node, _Obj, _Callback),
    !.

:- prolog_listen(waitable/4, update_waitables, [name(ros)]).

update_waitables(Action, Clause) :-
    notify_spin_on(Action),
    clause(waitable(_Type, Node, _Obj, _Callback), true, Clause),
    debug(ros(spin), 'Change to waitables for ~p', [Node]),
    forall(ros_node_store(_Name, Node, _Context, _Mutex, Guard),
           ros_trigger_guard_condition(Guard)).

notify_spin_on(asserta).
notify_spin_on(assertz).
notify_spin_on(retract).

%!  ros_spin_once(+Options) is semidet.
%
%   Wait for all waitable objects  registered   with  Node  and call the
%   callbacks associated with the ready objects.   Fails if there are no
%   objects to wait for or TimeOut is exceeded.  Options:
%
%     - node(+Node)
%       Node to operate on
%     - timeout(+Seconds)
%       Time to wait.  Either a number or `infinite`.  Default is
%       0.1 seconds.

ros_spin_once(Options) :-
    node_from_options(Node, Options),
    option(timeout(TimeOut), Options, 0.1),
    setup_call_cleanup(
        '$ros_set_node'(Node, spinner(true)),
        ros_spin_once(Node, [], TimeOut),
        '$ros_set_node'(Node, spinner(false))).

%!  ros_spin_once(+Node, +ExtraWaitables, +TimeOut) is semidet.
%
%   Wait for all waitable objects  registered   with  Node  and call the
%   callbacks associated with the ready objects.   Fails if there are no
%   objects to wait for or TimeOut is exceeded.
%
%   @tbd Cache current set of waitables

:- multifile
    ros_ready/1,
    ros_ready/3.

ros_spin_once(Node, Extra, TimeOut) :-
    findall(Obj, waitable(_Type, Node, Obj, _Callback), WaitFor, Extra),
    WaitFor \== [],
    ros_wait(WaitFor, TimeOut, Ready),
    ros_ready_list(Ready).

ros_ready_list([]).
ros_ready_list([H|T]) :-
    (   catch_with_backtrace(
            ros_ready(H),
            Error,
            print_message(warning, Error))
    ->  true
    ;   debug(ros(spin), 'Ignoring ready object ~p', [H])
    ),
    ros_ready_list(T).

%!  ros_ready(+Obj)
%
%   Hookable  predicate  to  handle  a  ready   object  as  returned  by
%   ros_wait/3. This is  a  variation  on   ros_ready/3  below  that  is
%   intended for action  servers  and  action   clients  that  return  a
%   compound term rather than a simple object.

ros_ready(Obj) :-
    waitable(Type, _Node, Obj, CallBack),
    !,
    ros_ready_det(Type, Obj, CallBack).

ros_ready_det(Type, Obj, CallBack) :-
    ros_ready(Type, Obj, CallBack),
    !.
ros_ready_det(Type, Obj, CallBack) :-
    print_message(warning,
                  ros(callback_failed(ros_ready(Type, Obj, CallBack)))).

%!  ros_ready(+Type, +Object, :Callback)
%
%   Hookable predicate to deal with normal waitable objects. Its task is
%   to retrieve information from the  ready   Object  and  call Callback
%   using this information.

ros_ready(subscription(_), Subscription, CallBack) :-
    ros_take(Subscription, Message, _MsgInfo),
    call(CallBack, Message).


%!  ros_create_guard_condition(-Cond, +Options) is det.
%
%   Create a ROS guard condition.  Options processed:
%
%    - context(+Context)
%      Context to use.  Default is provided by ros_default_context/1.

ros_create_guard_condition(Cond, Options) :-
    ros_context_from_options(Context, Options),
    '$ros_create_guard_condition'(Context, Cond).

%!  ros_trigger_guard_condition(+Cond) is det.
%
%   Trigger a guard condition object.


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
%       Set the name of the default node and the options to create it.
%       Options are passed to ros_create_node/3.

:- dynamic
    ros_default/1,                      % Term
    ros_context_store/1,                % Context
    ros_node_store/5,                   % Name, Node, Context, Mutex, SpinGuard
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

ros_default(init(list(atomic)),         "ROS context initialization arguments").
ros_default(domain(integer),            "ROS domain").
ros_default(node(atom, list(compound)), "ROS default node initialization options").

%!  ros_default_context(-Context)
%
%   Default context to use for ROS   operations.  This creates a default
%   ROS  context,  either  from  the   `init`  arguments  provided  with
%   ros_set_defaults/1 or command line  arguments following `--ros-args`
%   passed to Prolog.

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
    (   ros_default(init(Argv))
    ->  true
    ;   ros_default_argv(Argv)
    ),
    ignore(ros_default(domain(Domain))),
    ros_create_context(Context),
    ros_init(Context, Argv, Domain),
    '$ros_logging_initialize',
    asserta(ros_context_store(Context)),
    asserta(ros_property_store(domain(Domain))).

ros_default_argv(ROSArgv) :-
    current_prolog_flag(argv, Argv),
    ROSArgv = ['--ros-args'|_],
    append(_, ROSArgv, Argv),
    !.
ros_default_argv([]).


%!  ros_default_node(-Node) is det.
%
%   Default node to use. The node is  created in the context provided by
%   ros_default_context/1. The name and arguments to create the node are
%   provided by ros_set_defaults/1. If  no   defaults  are  provided the
%   default node name is   ``swipl-prolog-<number>``, where ``<number>``
%   is a large random number.
%
%   This is the same  as  `ros_node(default,   Node)`,  except  that  is
%   creates the node if this has not already been done.

ros_default_node(Node) :-
    ros_node_store(default, Node0, _, _, _),
    !,
    Node = Node0.
ros_default_node(Node) :-
    var(Node),
    with_mutex(ros, ros_default_node_sync(Node)).

ros_default_node_sync(Node) :-
    ros_node_store(default, Node, _, _, _),
    !.
ros_default_node_sync(Node) :-
    default_node_name_and_arguments(NodeName, Args),
    ros_create_node(NodeName, Node, [alias(default)|Args]).

default_node_name_and_arguments(NodeName, Args) :-
    ros_default(node(NodeName, Args)),
    !.
default_node_name_and_arguments(NodeName, []) :-
    Id is random(1<<63),
    atom_concat('swi_prolog_', Id, NodeName).

%!  ros_node(+Alias, -Node) is det.
%
%   Get a ROS node from its alias name.
%
%   @see ros_default_node/1.  A node with alias name can be created
%   using ros_create_node/3 with the option alias(Alias).
%   @error existence_error(ros_node, Alias)

ros_node(Alias, Node), atom(Alias) =>
    (   ros_node_store(Alias, Node, _, _, _)
    ->  true
    ;   existence_error(ros_node, Alias)
    ).
ros_node(Alias, _) =>
    type_error(atom, Alias).

%!  ros_create_node(+NodeName, -Node, +Options) is det.
%
%   Create a ROS node.  Options processed:
%
%     - alias(+Atom)
%       Alias name that can be used with the node(Node) option of
%       many predicates.
%     - ros_args(+Arguments)
%       Arguments used to initialize the node.  Default come from the
%       global initialization arguments passed to ros_init/3.
%     - rosout(+Boolean)
%       Enable/disable ros logging to ``/rosout``. Default is to have
%       this enabled.
%     - parameters(+List)
%       Specify parameters for this node.  Each parameter is a pair
%       whose key is the parameter name and whose value is a list of
%       options passed to ros_parameter/2.
%     - parameter_services(+Bool)
%       If true, start the ROS parameter services that provide other
%       nodes with access to our parameters.  Default is `true` if
%       parameters(List) is supplied.
%     - context(+Context)
%       Context to use.  Default is provided by ros_default_context/1.

ros_create_node(Name, Node, Options) :-
    select_option(alias(Alias), Options, Options1),
    !,
    (   ros_node_store(Alias, _, _, _, _)
    ->  permission_error(create, ros_node, Node)
    ;   true
    ),
    ros_create_node(Name, Alias, Node, Options1).
ros_create_node(Name, Node, Options) :-
    ros_create_node(Name, Node, Node, Options).

ros_create_node(Name, Alias, Node, Options) :-
    ros_context_from_options(Context, Options),
    '$ros_create_node'(Context, Name, Node, Options),
    sleep(0.2),                     % TBD: Needed to make the node visible
    mutex_create(Mutex),
    ros_create_guard_condition(Guard, [context(Context)]),
    asserta(ros_node_store(Alias, Node, Context, Mutex, Guard)),
    init_node_parameters(Node, Options).

ros_context_from_options(Context, Options) :-
    option(context(Context0), Options),
    !,
    Context = Context0.
ros_context_from_options(Context, _) :-
    ros_default_context(Context).

%!  init_node_parameters(+Node, +Options) is det.
%
%   Initialize the node parameters from the options.

init_node_parameters(Node, Options) :-
    (   option(parameters(Params), Options)
    ->  StartServiceDefault = true,
        maplist(add_parameter(Node), Params),
        import_parameters([node(Node),publish(false)])
    ;   StartServiceDefault = false
    ),
    (   option(parameter_services(true), Options, StartServiceDefault)
    ->  ros_param_services([node(Node)])
    ;   true
    ).

add_parameter(Node, Name-Options) =>
    ros_parameter(Name, [node(Node),publish(false)|Options]).


%!  node_clock(+Node, -Clock) is det.
%
%   Get the clock for the given node.  Create   a  new clock if the node
%   does not have a clock yet.
%
%   @tbd: Define the type of clock to be created

node_clock(Node, Clock) :-
    node_object(clock, Node, Clock0),
    !,
    Clock = Clock0.
node_clock(Node, Clock) :-
    ros_node_property(Node, context(Context)),
    ros_synchronized(Node, create_node_clock(Node, Context, Clock0)),
    Clock = Clock0.

create_node_clock(Node, _Context, Clock) :-
    node_object(clock, Node, Clock),
    !.
create_node_clock(_Node, Context, Clock) :-
    ros_create_clock(Context, system, Clock).

%!  ros_synchronized(+Object, :Goal) is semidet.
%
%   Run Goal synchronized on the mutex associated with Object. Currently
%   Object must be a node instance

ros_synchronized(Node, Goal) :-
    ros_node_store(_, Node, _, Mutex, _),
    !,
    with_mutex(Mutex, Goal).

%!  ros_shutdown is det.
%!  ros_shutdown(+Context) is det.
%
%   Shut down all or a specific context.

ros_shutdown :-
    forall(ros_context_store(Context),
           ros_shutdown(Context)).

ros_shutdown(Context) :-
    forall(ros_node_store(_Name, Node, Context, _, _),
           ros_shutdown_node(Node)),
    '$ros_logging_shutdown',
    forall(ros_context_store(Context),
           '$ros_shutdown'(Context)).

ros_shutdown_node(Node) :-
    '$ros_node_shutdown'(Node),
    ros_node_store(_, Node, _, _, Guard),
    ros_trigger_guard_condition(Guard),
    forall(retract(waitable(Type, Node, Object, _CallBack)),
           shutdown_waitable(Type, Object)),
    retractall(ros_node_store(_, Node, _, _, _)),
    ros_node_fini(Node).

shutdown_waitable(subscription(_Topic), Object) :-
    '$ros_unsubscribe'(Object).
shutdown_waitable(_Type, Object) :-
    debug(ros(shutdown), 'Do not know how to shutdown ~p', [Object]).

:- at_halt(ros_shutdown).


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

%!  ros_node_property(+Node, ?Property) is nondet.
%
%   True when Property is a  property  of   the  ROS  node Node. Defined
%   properties are:
%
%     - name(Name)
%     - namespace(Namespace)
%     - qname(Name)
%       Fully qualified name (namespace + name)
%     - logger_name(LoggerName)
%       Name of the default logger for this node. See
%       library(ros/logging) for details.
%     - clock(-Clock)
%       Get the clock associated to this node.
%     - spinner(-Thread)
%       Thread that is spinning on this node using ros_spin/1 or
%       ros_spin_once/1.

ros_node_property(Node, Property) :-
    ros_property_node(Property, Node).

ros_property_node(name(Name), Node) :-
    '$ros_node_prop'(Node, name, Name).
ros_property_node(namespace(Name), Node) :-
    '$ros_node_prop'(Node, namespace, Name).
ros_property_node(qname(QName), Node) :-
    '$ros_node_prop'(Node, qname, QName).
ros_property_node(logger_name(Name), Node) :-
    '$ros_node_prop'(Node, logger_name, Name).
ros_property_node(clock(Clock), Node) :-
    node_clock(Node, Clock).
ros_property_node(context(Context), Node) :-
    '$ros_node_prop'(Node, context, Context).
ros_property_node(spinner(Thread), Node) :-
    '$ros_node_prop'(Node, spinner, TID),
    (   Thread == TID
    ->  true
    ;   catch(thread_property(TID, alias(Alias)), error(_,_), fail)
    ->  Thread = Alias
    ;   Thread = TID
    ).

%!  ros_publisher(+Node, +MsgType, +Topic, +QoSProfile, -Publisher)
%
%   Create a publisher for the given topic.

ros_publisher(Node, MsgType, Topic, QoSProfile, Publisher) :-
    ros_msg_type_support(MsgType, TypeSupport),
    '$ros_publisher'(Node, TypeSupport, Topic, QoSProfile, Publisher).

%!  ros_subscribe(+Node, +MsgType, +Topic, +QoSProfile, -Subscription)
%
%   Create a subscription for the given topic.

ros_subscribe(Node, MsgType, Topic, QoSProfile, Subscription) :-
    ros_msg_type_support(MsgType, TypeSupport),
    '$ros_subscribe'(Node, TypeSupport, Topic, QoSProfile, Subscription).

%!  ros_take(+Subscription, -Message, -MsgInfo) is det.
%
%   Read Message from Subscription (to a topic). The caller should first
%   use ros_wait/3 on Subscription to wait   for a message. Messages are
%   normally dealt with using a callback from ros_spin/1.


%!  ros_wait(+Objects, +Timeout, -Ready) is semidet.
%
%   Wait for any of the resources in  Objects to become ready and return
%   the ready objects in Ready.  Timeout  is   in  seconds  or  the atom
%   `infinite`. If waiting finishes  due  to   a  timeout  the predicate
%   fails. For most objects Ready simply   contains the object reference
%   (a _blob_). Action clients and servers   return a compound term that
%   both provides the ready object and the entities inside this compound
%   object that are ready.  Waitalble objects currently are:
%
%     - subscription
%     - publisher
%     - service
%       Ready contains handle
%     - action_client
%       Ready is a term as below.  Except for the `Handle`, all
%       values are booleans (`true` or `false`).
%
%           action_client(Handle, Feedback, Status, Goal, Cancel, Result)
%
%     - action_server
%       As for `action_client`, but the term is
%
%           action_server(Handle, Goal, Cancel, Result, Expired)
%
%   Waiting for message is  normally  achieved   using  more  high level
%   primitives such as ros_spin/1.


%!  ros_msg_type_support(+MsgType, -TypeSupport) is det.
%!  ros_srv_type_support(+SrvType, -TypeSupport) is det.
%
%   Get the RCL type support description for  a message type or service.
%   MsgType is an atom  or  term   of  the  form  Package/Kind/Type. For
%   example:
%
%      ?- ros_msg_type_support('std_msgs/msg/Int64', Type).
%      Type = <rcl_message_type_t>(0x7fbc66896210).
%
%   @tbd We plan to make this more Prolog friendly by providing defaults
%   for the Package and Kind  components   and  case  conversion for the
%   actual type.

:- table ( ros_msg_type_support/2,
           ros_srv_type_support/2,
           ros_action_type_support/2,
           load_type_support_shared_object/3
         ) as shared.

ros_type_support(Type, TypeBlob) :-
    type_support_function(Type, _Package, FuncPostfix),
    (   sub_atom(FuncPostfix, _, _, _, '__msg__')
    ->  ros_msg_type_support(Type, TypeBlob)
    ;   sub_atom(FuncPostfix, _, _, _, '__srv__')
    ->  ros_srv_type_support(Type, TypeBlob)
    ;   sub_atom(FuncPostfix, _, _, _, '__action__')
    ->  ros_action_type_support(Type, TypeBlob)
    ;   domain_error(ros_type, Type)
    ).

ros_msg_type_support(MsgType, MsgFunctions) :-
    type_support_function_prefix(msg, TSPrefix),
    type_introspection_function_prefix(ISPrefix),
    type_support_function(MsgType, Package, FuncPostfix),
    load_type_support(Package),
    atomic_list_concat([TSPrefix, FuncPostfix], '__', TSFunc),
    atomic_list_concat([ISPrefix, FuncPostfix], '__', ISFunc),
    '$ros_message_type'(ISFunc, TSFunc, FuncPostfix, MsgFunctions),
    !.
ros_msg_type_support(MsgType, _) :-
    existence_error(ros_message_type, MsgType).

ros_srv_type_support(SrvType, SrvFunctions) :-
    type_support_function_prefix(srv, TSPrefix),
    type_introspection_function_prefix(ISPrefix),
    type_support_function(SrvType, Package, FuncPostfix),
    load_type_support(Package),
    atom_concat(FuncPostfix, '_Request', FuncPostfixRequest),
    atom_concat(FuncPostfix, '_Response', FuncPostfixResponse),
    atomic_list_concat([TSPrefix, FuncPostfix], '__', TSFunc),
    atomic_list_concat([ISPrefix, FuncPostfixRequest], '__', ISFuncRequest),
    atomic_list_concat([ISPrefix, FuncPostfixResponse], '__', ISFuncResponse),
    '$ros_service_type'(ISFuncRequest, ISFuncResponse,
                        TSFunc,
                        FuncPostfixRequest, FuncPostfixResponse,
                        SrvFunctions),
    !.
ros_srv_type_support(SrvType, _) :-
    existence_error(ros_message_type, SrvType).

ros_action_type_support(ActType, ActFunctions) :-
    type_support_function_prefix(action, TSPrefix),
    type_introspection_function_prefix(ISPrefix),
    type_support_function(ActType, Package, FuncPostfix),
    load_type_support(Package),
    atom_concat(FuncPostfix, '_SendGoal_Request', FuncPostfixGoalRequest),
    atom_concat(FuncPostfix, '_SendGoal_Response', FuncPostfixGoalResponse),
    atom_concat(FuncPostfix, '_GetResult_Request', FuncPostfixResultRequest),
    atom_concat(FuncPostfix, '_GetResult_Response', FuncPostfixResultResponse),
    atom_concat(FuncPostfix, '_FeedbackMessage', FuncPostfixFeedback),
    atomic_list_concat([TSPrefix, FuncPostfix], '__', TSFunc),
    atomic_list_concat([ISPrefix, FuncPostfixGoalRequest], '__', ISFuncGoalRequest),
    atomic_list_concat([ISPrefix, FuncPostfixGoalResponse], '__', ISFuncGoalResponse),
    atomic_list_concat([ISPrefix, FuncPostfixResultRequest], '__', ISFuncResultRequest),
    atomic_list_concat([ISPrefix, FuncPostfixResultResponse], '__', ISFuncResultResponse),
    atomic_list_concat([ISPrefix, FuncPostfixFeedback], '__', ISFuncFeedback),
    '$ros_action_type'(ISFuncGoalRequest, ISFuncGoalResponse,
                       ISFuncResultRequest, ISFuncResultResponse, ISFuncFeedback,
                       TSFunc,
                       FuncPostfixGoalRequest, FuncPostfixGoalResponse,
                       FuncPostfixResultRequest, FuncPostfixResultResponse, FuncPostfixFeedback,
                       ActFunctions),
    !.
ros_action_type_support(ActType, _) :-
    existence_error(ros_message_type, ActType).

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

load_type_support(Package) :-
    rwm_c_identifier(Id),
    atom_concat(rosidl_typesupport_, Id, WM_TypeSupport),
    load_type_support_shared_object(Package, WM_TypeSupport, _),
    load_type_support_shared_object(Package, rosidl_typesupport_c, _),
    load_type_support_shared_object(Package, rosidl_generator_c, _),
    load_type_support_shared_object(Package, rosidl_typesupport_introspection_c, _).

load_type_support_shared_object(Package, Which, Handle) :-
    current_prolog_flag(shared_object_extension, SO),
    atomic_list_concat([lib,Package,'__',Which, '.', SO], File),
    debug(ros(type_support), 'Loading type support shared object ~p', [File]),
    open_shared_object(File, Handle, [global]).

rwm_c_identifier(Id) :-
    ros_property(rmw_identifier(RWM_ID)),
    split_string(RWM_ID, "_", "", [_,MW,_]),
    atom_concat(MW, '_c', Id).

%!  ros_object(?Object, ?Type) is nondet.
%
%   True when Object is a handle to a ROS object of type Type.

ros_object(Object, Type) :-
    nonvar(Object),
    !,
    '$ros_object_type'(Object, Type).
ros_object(Object, Type) :-
    current_blob(Object, c_ptr),
    '$ros_object_type'(Object, Type).


%!  ros_property(?Property)
%
%   Enumerate properties about the current   ROS  environment. Currently
%   defined:
%
%    - rmw_identifier(Id)

ros_property(rmw_identifier(Id)) :-
    ros_rwm_implementation(Id).


%!  ros_debug(+Level)
%
%   Print low level debug messages to   the  console. See ``DEBUG(Level,
%   Code)`` calls in the C source `src/rclswi.c`. Notably level 10 shows
%   all calls to the `rcl` library and their results.
