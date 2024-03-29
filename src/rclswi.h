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

#ifndef RCLSWI_H_INCLUDED
#define RCLSWI_H_INCLUDED

typedef struct rclswi_message_type_t
{ const rosidl_message_type_support_t *type_support;
  const rosidl_message_type_support_t *introspection;
  void* (*create)();
  void  (*init)(void*);
  void  (*destroy)(void*);
  void  (*fini)(void*);
} rclswi_message_type_t;

typedef struct rclswi_srv_type_t
{ const rosidl_service_type_support_t *type_support;
  rclswi_message_type_t request;
  rclswi_message_type_t response;
} rclswi_srv_type_t;

typedef struct rclswi_action_type_t
{ const rosidl_action_type_support_t *type_support;
  rclswi_srv_type_t goal;
  rclswi_srv_type_t result;
  rclswi_message_type_t feedback;
} rclswi_action_type_t;

#endif /*RCLSWI_H_INCLUDED*/
