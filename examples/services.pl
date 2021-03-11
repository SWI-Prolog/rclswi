:- module(services,
          [ type/0,
            type/1,
            add/3,
            add_server/0
          ]).
:- use_module(library(pprint)).
:- use_module(library(debug)).

:- reexport(library(ros)).

/** <module>

@see https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html
*/

:- ros_set_defaults(
       [ node(swi_service, [])
       ]).

type :-
    type('example_interfaces/srv/AddTwoInts').

type(Name) :-
    ros_type_introspection(Name, Type),
    print_term(Type, []).

:- table client/1 as shared.

client(Client) :-
    ros_default_node(Node),
    ros:ros_srv_type_support('example_interfaces/srv/AddTwoInts', Type),
    ros:'$ros_create_client'(Node, Type, '/add_two_ints', _QoS, Client).

add(A,B,Sum) :-
    client(Client),
    ros:ros_send_request(Client, _{a:A,b:B}, Seq),
    debug(client, 'Sent request ~p', [Seq]),
    ros_wait([Client], 10, Ready),
    debug(client, 'Ready: ~p', [Ready]),
    ros:ros_take_response(Client, Response, Seq2),
    debug(client, 'Got response ~p for request ~p', [Response, Seq2]),
    Sum = Response.sum.

:- table service/1 as shared.

service(Service) :-
    ros_default_node(Node),
    ros:ros_srv_type_support('example_interfaces/srv/AddTwoInts', Type),
    ros:'$ros_create_service'(Node, Type, '/add_two_ints', _QoS, Service).

add_server :-
    service(Service),
    repeat,
        ros_wait([Service], 10, Ready),
        debug(service, 'Ready: ~p', [Ready]),
        ros:ros_take_request(Service, Request, Info),
        Sum is Request.a + Request.b,
        debug(service, 'Sum is: ~p', [Sum]),
        ros:ros_send_response(Service, _{sum:Sum}, Info),
        fail.
