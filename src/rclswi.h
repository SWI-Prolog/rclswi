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
  void* (*seq_create)();
  void  (*seq_init)(void*, size_t size);
  void  (*seq_destroy)(void*);
  void  (*seq_fini)(void*);
} rclswi_message_type_t;


#endif /*RCLSWI_H_INCLUDED*/
