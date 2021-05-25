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

:- module(ros_services,
          [ ros_client/4,           % +ServiceName, +SrvType, -Client, +Options
            ros_call/3,             % +Client, +Request, -Response
            ros_call/4,             % +Client, +Request, -Response, +Options
            ros_service/4,          % +ServiceName, +SrvType, -Service, +Options
            ros_service_spin/2,     % +Service, :Callback
            ros_service_spin/3,     % +Service, :Callback, +Options
            ros_service_fold/4,     % +Service, :Callback, +StateStart, -StateEnd
            ros_service_property/2  % +Service, ?Property
          ]).
:- use_module(library(ros)).
:- use_module(library(ros/detail/options)).
:- use_module(library(debug)).
:- use_module(library(error)).
:- use_module(library(option)).
:- autoload(library(thread_pool), [thread_create_in_pool/4]).

:- meta_predicate
    ros_service_spin(+, 2),
    ros_service_spin(+, 2, +),
    ros_service_fold(+, 4, +, -).

/** <module> ROS services interface

@tbd Register clients with the node
*/

		 /*******************************
		 *       CLIENT INTERFACE	*
		 *******************************/

%!  ros_client(+ServiceName, +SrvType, -Client, +Options) is det.
%
%   Register a client for ServiceName   that  satisfies SrvType. Options
%   processed:
%
%     - node(+Node)
%       Node with which to associate this service
%     - qos_profile(+QoSProfile)
%       Quality of Service profile.

ros_client(ServiceName, SrvType, Client, Options) :-
    node_from_options(Node, Options),
    ros:ros_srv_type_support(SrvType, TypeSupport),
    qos_profile_from_options(QoSProfile, Options),
    ros:'$ros_create_client'(Node, TypeSupport, ServiceName, QoSProfile, Client).

%!  ros_call(+Client, +Request, -Response) is semidet.
%!  ros_call(+Client, +Request, -Response, +Options) is semidet.
%
%   Perform an synchronous call on  Client.   Request  and  Response are
%   dicts that must  conform  the   service  type  description.  Options
%   provided:
%
%     - deadline(+Stamp)
%       Do not wait beyond Stamp.  If `timeout` is also provided this
%       option takes preference.
%     - timeout(+Seconds)
%       Do not wait more than Seconds.  This is translated to the
%       deadline(Stamp) option.
%     - service_info(-Info)
%       If provided, Info is unified with the returned service info
%       represented as a Prolog dict.
%
%   @tbd: what do we do with out-of-sequence replies on the same
%   service?

ros_call(Client, Request, Response) :-
    ros_call(Client, Request, Response, []).

ros_call(Client, Request, Response, Options) :-
    deadline(State, Options),
    ros:ros_send_request(Client, Request, SeqReq),
    option(service_info(ServiceInfo), Options, _),
    must_be(var, ServiceInfo),
    repeat,
      (   deadline_timeout(State, TimeOut)
      ->  ros_wait([Client], TimeOut, Ready),
          Ready = [Client],
          ros:ros_take_response(Client, Response0, ServiceInfo),
          SeqRes = ServiceInfo.request_id.sequence_number,
          (   SeqReq == SeqRes
          ->  !, Response = Response0
          ;   debug(ros(client),
                    'Discarding out-of-sequence response (~p \\== ~p)',
                    [SeqRes, SeqReq]),
              fail
          )
      ;   !, fail
      ).

deadline(deadline(Deadline), Options) :-
    (   option(deadline(Deadline0), Options)
    ->  Deadline = Deadline0
    ;   option(timeout(Timeout), Options)
    ->  get_time(Now),
        Deadline is Now+Timeout
    ;   Deadline = infinite
    ).

deadline_timeout(deadline(Deadline), TimeOut) :-
    (   Deadline == infinite
    ->  TimeOut = infinite
    ;   get_time(Now),
        TimeOut is Deadline - Now,
        TimeOut > 0
    ).


		 /*******************************
		 *       SERVICE INTERFACE	*
		 *******************************/

%!  ros_service(+ServiceName, +SrvType, -Service, +Options)
%
%   Register a server for ServiceName   that  satisfies SrvType. Options
%   processed:
%
%     - node(+Node)
%       Node with which to associate this service
%     - qos_profile(+QoSProfile)
%       Quality of Service profile.

ros_service(ServiceName, SrvType, Service, Options) :-
    node_from_options(Node, Options),
    ros:ros_srv_type_support(SrvType, TypeSupport),
    qos_profile_from_options(QoS, Options),
    ros:'$ros_create_service'(Node, TypeSupport, ServiceName, QoS, Service).

%!  ros_service_spin(+Service, :Callback) is det.
%!  ros_service_spin(+Service, :Callback, +Options) is det.
%
%   Register Service to be processed by the  spin loop of the associated
%   node. The response is generated by calling
%
%       call(Callback, Query, Response)
%
%   Options provides additional proccesing options. Defined options are:
%
%     - threaded(+How)
%       Handle new requests in a thread such that multiple overlapping
%       requests can be processed concurrently.  How is one of
%       - true
%         Handle the request in a new _detached_ thread.  This is the
%         same as `[detached(true)]`.
%       - OptionList
%         Handle the request is a thread created using OptionList.
%         See thread_create/3 for details.
%       - pool(Name, OptionList)
%         Handle the request is a thread created using OptionList using
%         thread_create_in_pool/4.

ros_service_spin(Service, Callback) :-
    ros_service_spin(Service, Callback, []).

ros_service_spin(Service, Callback, Options) :-
    ros_service_property(Service, name(Name)),
    ros_service_property(Service, node(Node)),
    callback_options(Callback, TheCallback, Options),
    ros:register_waitable(service(Name), Node, Service, TheCallback).

callback_options(Callback, TheCallback, Options) :-
    option(threaded(How), Options),
    !,
    TheCallback = threaded(Callback, How).
callback_options(Callback, Callback, _).

%!  ros_service_fold(+Service, :Callback, +StateStart, -StateEnd) is det.
%
%   Process service request while collecting state. On each request this
%   calls
%
%       call(Callback, State, Request, NextState, Response)
%
%   @tbd: currently breaks the  loop  on   failure  or  error. Should we
%   ignore such messages?

ros_service_fold(Service, Callback, StateStart, StateEnd) :-
    repeat,
    ros_wait([Service], 10, Ready),
    Ready = [Service],
    !,
    ros:ros_take_request(Service, Request, Info),
    call(Callback, StateStart, Request, StateNext, Response),
    ros:ros_send_response(Service, Response, Info),
    ros_service_fold(Service, Callback, StateNext, StateEnd).


%!  ros_service_property(+Service, ?Property) is nondet.
%
%   True when Property is a property of Service.

ros_service_property(Service, Property) :-
    ros_service_prop(Property, Service).

ros_service_prop(node(Node), Service) :-
    ros:'$ros_service_prop'(Service, node, Node).
ros_service_prop(name(Name), Service) :-
    ros:'$ros_service_prop'(Service, name, Name).


:- multifile ros:ros_ready/3.

ros:ros_ready(service(_ServiceName), Service, CallBack) :-
    ros:ros_take_request(Service, Request, Info),
    handle_request(CallBack, Service, Request, Info).

handle_request(threaded(CallBack, How), Service, Request, Info) :-
    !,
    handle_threaded_request(How, CallBack, Service, Request, Info).
handle_request(CallBack, Service, Request, Info) :-
    process_request(CallBack, Service, Request, Info).

process_request(CallBack, Service, Request, Info) :-
    call(CallBack, Request, Response),
    ros:ros_send_response(Service, Response, Info).

handle_threaded_request(true, CallBack, Service, Request, Info) =>
    thread_create(process_request(CallBack, Service, Request, Info), _,
                  [ detached(true) ]).
handle_threaded_request(Options, CallBack, Service, Request, Info),
    is_list(Options) =>
    thread_create(
        process_request(CallBack, Service, Request, Info), _,
        Options).
handle_threaded_request(pool(Pool, Options), CallBack, Service, Request, Info) =>
    thread_create_in_pool(
        Pool,
        process_request(CallBack, Service, Request, Info), _,
        Options).

