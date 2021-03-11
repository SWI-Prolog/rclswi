/*  This example code is public domain.
*/

:- module(services,
          [ type/0,
            type/1,                     % +ServiceName
            add/3,                      % +A, +B, -Sum
            add_server/0,
            add_server_fold/0
          ]).
:- use_module(library(pprint)).
:- use_module(library(debug)).

:- reexport(library(ros/services)).
:- reexport(library(ros)).

/** <module>

@see https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html
*/

:- ros_set_defaults(
       [ node(swi_service, [])
       ]).

%!  type is det.
%!  type(+ServiceName) is det.
%
%   Fetch and show the request and response message types for a service.

type :-
    type('example_interfaces/srv/AddTwoInts').

type(Name) :-
    ros_type_introspection(Name, Type),
    print_term(Type, []).

%!  add(+A,+B,-Sum) is det.
%
%   Use the ``/add_two_ints`` service  to  add   two  integers  using  a
%   synchronous call.

add(A,B,Sum) :-
    client(Client),
    ros_call(Client, _{a:A,b:B}, Response),
    Sum = Response.sum.

:- table client/1 as shared.

client(Client) :-
    ros_client('/add_two_ints',
               'example_interfaces/srv/AddTwoInts',
               Client, []).

%!  add_server
%
%   Start and spin a server for ``/add_two_ints``

add_server :-
    service(Service),
    ros_service_spin(Service, add),
    ros_spin.

add(Request, _{sum:Sum}) :-
    Sum is Request.a + Request.b.

%!  add_server_fold
%
%   Start and spin a server for ``/add_two_ints``.  In contrast,
%   this server adds the two numbers to an accumulated total.

add_server_fold :-
    service(Service),
    ros_service_fold(Service, add, 0, _End).

add(State0, Request, Sum, _{sum:Sum}) :-
    Sum is Request.a + Request.b + State0.

:- table service/1 as shared.

service(Service) :-
    ros_service('/add_two_ints',
               'example_interfaces/srv/AddTwoInts',
                Service, []).

