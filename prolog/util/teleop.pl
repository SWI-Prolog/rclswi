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

:- module(teleop,
          [ teleop/3,                   % :OnKey, +StateIn, -State
            poll_char/2                 % -Char, +Timeout
          ]).
:- autoload(library(make), [make/0]).

/** <module> Control applications using the keyboard

This library allows controlling an application using the keyboard.
*/

:- meta_predicate
    teleop(3, ?, ?).

%!  teleop(:OnKey, +State0, -State)
%
%   Run an application under keyboard control.   On  keyboard input this
%   calls:
%
%       call(OnKey, StateN, StateM)
%
%   Characters are read using get_char/2  and   (thus)  passes as single
%   character atoms. The arrow keys are decoded (assuming ANSI encoding)
%   to the atoms `up`, `down`, `right` or `left`.
%
%   Two keys are reserved:
%
%     - `q` terminates the loop
%     - `m` runs make/0, reloading the program when it was modified.
%
%   @tbd Other keys sending sequences, such   as  the function keys, are
%   not decoded.
%   @bug ANSI sequences are hard coded.
%   @bug Only works on POSIX systems where we can use wait_for_input/3
%   to wait for any file descriptor.

teleop(OnKey, State0, State) :-
    with_tty_raw(teleop_raw(OnKey, State0, State)).

teleop_raw(OnKey, State0, State) :-
    format('~N'),
    poll_char(C, infinite),
    (   C == q
    ->  State = State0
    ;   C == m
    ->  make,
        teleop_raw(OnKey, State0, State)
    ;   call(OnKey, C, State0, State1),
        teleop_raw(OnKey, State1, State)
    ).

%!  poll_char(-Char, +Timeout) is semidet.
%
%   Wait for a command. Decodes the keyboard  arrow keys. Timeout is the
%   max time to wait in seconds (a float).   A  value of `0` polls for a
%   ready  character  without  blocking.  The  value  `infinite`  blocks
%   indefinitely.

poll_char(C, TimeOut) :-
    wait_for_input([user_input], Ready, TimeOut),
    Ready = [user_input],
    get_char(user_input, C0),
    read_arrow(C0, C).

read_arrow('\e', Arrow) =>
    get_char(user_input, '['),
    get_char(user_input, AD),
    arrow(AD, Arrow).
read_arrow(C, Char) => Char = C.

arrow('A', Arrow) => Arrow = up.
arrow('B', Arrow) => Arrow = down.
arrow('C', Arrow) => Arrow = right.
arrow('D', Arrow) => Arrow = left.
