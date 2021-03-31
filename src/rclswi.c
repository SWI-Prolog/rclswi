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

#include <SWI-Stream.h>
#include <SWI-Prolog.h>
#include "pointer.h"

#define _GNU_SOURCE
#include <dlfcn.h>
#include <math.h>

#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/graph.h>
#include <rcl/logging.h>
#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/rcl.h>
#include <rcl/remap.h>
#include <rcl/time.h>
#include <rcl/validate_topic_name.h>
#include <rcl/init_options.h>
#include <rcl/context.h>
#include <rcl_interfaces/msg/parameter_type.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcl_action/rcl_action.h>
#include <rcutils/allocator.h>
#include <rcutils/format_string.h>
#include <rcutils/macros.h>
#include <rcutils/strdup.h>
#include <rcutils/types.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/topic_endpoint_info_array.h>
#include <rmw/types.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/string_functions.h>

#include "common.h"
#include "rclswi.h"
#include "qos.h"

rcl_allocator_t rclswi_default_allocator;
static rcl_context_t  *rclswi_default_context_ptr = NULL;

static int	put_message(term_t Message, void *msg,
			    const rosidl_message_type_support_t *ts);
static int	fill_message(term_t Message, void *msg,
			     const rosidl_message_type_support_t *ts,
			     int noarray);
static int	fill_uuid(term_t t, void *msg);
static int	fill_time_stamp(term_t Stamp, builtin_interfaces__msg__Time *tp, int ex);
static int	get_timeout_nsec(term_t Timeout, int64_t *tmo);

static atom_t ATOM_action;
static atom_t ATOM_ros_args;
static atom_t ATOM_clock;
static atom_t ATOM_context;
static atom_t ATOM_feedback;
static atom_t ATOM_global;
static atom_t ATOM_goal;
static atom_t ATOM_goal_id;
static atom_t ATOM_goal_info;
static atom_t ATOM_goal_request;
static atom_t ATOM_goal_response;
static atom_t ATOM_inf;
static atom_t ATOM_infinite;
static atom_t ATOM_local;
static atom_t ATOM_logger_name;
static atom_t ATOM_name;
static atom_t ATOM_namespace;
static atom_t ATOM_node;
static atom_t ATOM_node_parameters;
static atom_t ATOM_parameters;
static atom_t ATOM_received_timestamp;
static atom_t ATOM_request;
static atom_t ATOM_request_id;
static atom_t ATOM_response;
static atom_t ATOM_result;
static atom_t ATOM_result_request;
static atom_t ATOM_result_response;
static atom_t ATOM_ros;
static atom_t ATOM_rosout;
static atom_t ATOM_sequence_number;
static atom_t ATOM_service;
static atom_t ATOM_service_info;
static atom_t ATOM_source_timestamp;
static atom_t ATOM_stamp;
static atom_t ATOM_steady;
static atom_t ATOM_system;
static atom_t ATOM_type;
static atom_t ATOM_writer_guid;
static atom_t ATOM_active;
static atom_t ATOM_status;
static atom_t ATOM_server;
static atom_t ATOM_cancel;

static functor_t FUNCTOR_error2;
static functor_t FUNCTOR_ros_error2;
static functor_t FUNCTOR_minus2;
static functor_t FUNCTOR_list1;
static functor_t FUNCTOR_list2;
static functor_t FUNCTOR_action_client6;
static functor_t FUNCTOR_action_server5;


		 /*******************************
		 *     ENCAPSULATED POINTERS	*
		 *******************************/

static void free_rcl_context(void*);
static void free_rcl_node(void*);
static void free_rcl_publisher(void*);
static void free_rcl_subscription(void*);
static void free_rcl_client(void*);
static void free_rcl_service(void*);
static void free_rcl_clock(void*);
static void free_rcl_action_client(void*);
static void free_rcl_action_server(void*);
static void free_rwm_service_info(void*);
static void free_goal_handle(void*);

static c_pointer_type context_type =
{ "ros_context",
  free_rcl_context
};

static c_pointer_type node_type =
{ "ros_node",
  free_rcl_node
};

static c_pointer_type publisher_type =
{ "ros_publisher",
  free_rcl_publisher
};

static c_pointer_type subscription_type =
{ "ros_subscription",
  free_rcl_subscription
};

static c_pointer_type rclswi_message_type_type =
{ "ros_message_type",
  NULL
};

static c_pointer_type client_type =
{ "ros_client",
  free_rcl_client
};

static c_pointer_type service_type =
{ "ros_service",
  free_rcl_service
};

static c_pointer_type clock_type =
{ "ros_clock",
  free_rcl_clock
};

static c_pointer_type action_client_type =
{ "ros_action_client",
  free_rcl_action_client
};

static c_pointer_type action_server_type =
{ "ros_action_server",
  free_rcl_action_server
};

static c_pointer_type rwm_service_info_type =
{ "ros_service_info",
  free_rwm_service_info
};

static c_pointer_type rmw_request_id_type =
{ "ros_request_id",
  free
};

static c_pointer_type rclswi_srv_type_type =
{ "ros_service_type",
  NULL
};

static c_pointer_type rclswi_action_type_type =
{ "ros_action_type",
  NULL
};

static c_pointer_type goal_handle_type =
{ "ros_goal_handle",
  free_goal_handle
};

		 /*******************************
		 *	      DEBUG		*
		 *******************************/

static int debuglevel = 0;

static foreign_t
ros_debug(term_t level)
{ return PL_get_integer_ex(level, &debuglevel);
}

int
ros_debug_level(void)
{ return debuglevel;
}


		 /*******************************
		 *	      ERRORS		*
		 *******************************/

int
set_error(rcl_ret_t ret)
{ term_t ex;
  const char *msg = rcl_get_error_string().str;
  int rc;

  rc = ( (ex=PL_new_term_ref()) &&
	 PL_unify_term(ex, PL_FUNCTOR, FUNCTOR_error2,
		             PL_FUNCTOR, FUNCTOR_ros_error2,
		               PL_INT, (int)ret,
		               PL_UTF8_CHARS, msg,
		             PL_VARIABLE) &&
	 PL_raise_exception(ex) );

  rcl_reset_error();
  return rc;
}

void
print_error(rcl_ret_t ret, const char *file, int line, const char *goal)
{ Sdprintf("% ERROR: [rclswi %s:%d] %s:\n"
	   "% ERROR: %s\n",
	   file, line, goal, rcl_get_error_string().str);
  rcl_reset_error();
}


		 /*******************************
		 *	      TYPES		*
		 *******************************/

/** '$ros_object_type'(@Term, -Type) is semidet.
 */

static foreign_t
ros_object_type(term_t Ptr, term_t Type)
{ c_pointer_type *type;
  void *obj;

  if ( get_pointer_type(Ptr, &obj, (const c_pointer_type**)&type) )
  { if ( !type->name )
      type->name = PL_new_atom(type->type);

    return PL_unify_atom(Type, type->name);
  }

  return FALSE;
}


		 /*******************************
		 *	      CONTEXT		*
		 *******************************/

rcl_context_t *
rclswi_default_context(void)
{ if ( !rclswi_default_context_ptr )
  { rcl_context_t *context;

    if ( (context=malloc(sizeof(*context))) )
    { *context = rcl_get_zero_initialized_context();
      rclswi_default_context_ptr = context;
    }
  }

  return rclswi_default_context_ptr;
}


static foreign_t
ros_create_context(term_t t)
{ rcl_context_t *context;

  if ( (context=malloc(sizeof(*context))) )
  { *context = rcl_get_zero_initialized_context();
    if ( !rclswi_default_context_ptr )
      rclswi_default_context_ptr = context;
    return unify_pointer(t, context, &context_type);
  }

  return PL_resource_error("memory");
}


static void
free_rcl_context(void *ptr)
{ rcl_context_t *context = ptr;

  if ( ptr == rclswi_default_context_ptr )
    rclswi_default_context_ptr = NULL;

  if ( context->impl )
  { if ( rcl_context_is_valid(context) )
      TRYVOID(rcl_shutdown(context));

    TRYVOID(rcl_context_fini(context));
  }
}

		 /*******************************
		 *	      INIT		*
		 *******************************/

static foreign_t
ros_init(term_t Context, term_t Args, term_t DomainId)
{ int64_t domain_id = RCL_DEFAULT_DOMAIN_ID;
  rcl_context_t *context;
  size_t num_args;

  if ( PL_is_variable(DomainId) )
  { if ( !PL_unify_int64(DomainId, domain_id) )
      return FALSE;
  } else
  { if ( !PL_get_int64_ex(DomainId, &domain_id) )
      return FALSE;
    if ( domain_id < 0 )
      return PL_domain_error("nonneg", DomainId);
  }

  if ( !get_pointer(Context, (void**)&context, &context_type) )
    return FALSE;

  if ( PL_skip_list(Args, 0, &num_args) == PL_LIST )
  { rcl_allocator_t allocator = rcl_get_default_allocator();
    char **arg_values = NULL;
    term_t tail = PL_copy_term_ref(Args);
    term_t head = PL_new_term_ref();
    int rc = TRUE;

    if ( num_args > 0 &&
	 !(arg_values=rcl_alloc(sizeof(*arg_values)*num_args, NULL)) )
      return FALSE;

    for(char **argp = arg_values; PL_get_list_ex(tail, head, tail); argp++)
    { if ( !PL_get_chars(head, argp, CVT_ATOMIC|CVT_EXCEPTION|REP_UTF8) )
	OUTFAIL;
    }
    if ( !PL_get_nil_ex(tail) )
      OUTFAIL;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    TRY(rcl_init_options_init(&init_options, allocator));
#ifdef HAVE_RCL_INIT_OPTIONS_SET_DOMAIN_ID
    TRY(rcl_init_options_set_domain_id(&init_options, (size_t)domain_id));
#endif
    TRY(rcl_init(num_args, (const char**)arg_values, &init_options, context));
    int unparsed = rcl_arguments_get_count_unparsed_ros(&context->global_arguments);
    if ( unparsed > 0 )
      Sdprintf("ROS: %d options were not parsed\n");

  out:
    rcl_free(arg_values, NULL);
    return rc;
  } else
  { return PL_type_error("list", Args);
  }
}


static foreign_t
ros_shutdown(term_t Context)
{ rcl_context_t *context;
  int rc = TRUE;

  if ( !get_pointer(Context, (void**)&context, &context_type) )
    return FALSE;

  TRY(rcl_shutdown(context));
  return rc;
}


static foreign_t
ros_ok(term_t Context)
{ rcl_context_t *context;

  if ( !get_pointer(Context, (void**)&context, &context_type) )
    return FALSE;

  return !!rcl_context_is_valid(context);
}



		 /*******************************
		 *	      ARGUMENTS		*
		 *******************************/

/**
 * Translate a Prolog list of strings into an RCL array managed
 * by allocator
 */

static void
free_rcl_arglist(char **arg_values, size_t len,
		 rcl_allocator_t *allocator)
{ size_t i;

  for(i=0; i<len; i++)
    rcl_free(arg_values[i], allocator);

  rcl_free(arg_values, allocator);
}


static int
rclswi_get_arglist(term_t list,
		   int *num_args, char ***arg_values,
		   rcl_allocator_t *allocator)
{ size_t len;
  int rc = TRUE;

  if ( PL_skip_list(list, 0, &len) != PL_LIST )
    return PL_type_error("list", list);
  len++;				/* for --ros-args */

  *num_args = len;
  *arg_values = NULL;

  if ( len > 1 )
  { term_t tail = PL_copy_term_ref(list);
    term_t head = PL_new_term_ref();
    char **argp;

    if ( !(*arg_values = rcl_alloc(sizeof(*arg_values)*len, allocator)) )
      return FALSE;
    argp = *arg_values;
    *argp++ = rcutils_strdup("--ros-args", *allocator);

    for( ; PL_get_list_ex(tail, head, tail); argp++)
    { char *s;

      if ( !PL_get_chars(head, &s, CVT_ATOMIC|CVT_EXCEPTION|REP_UTF8) )
	OUTFAIL;
      if ( !(*argp = rcutils_strdup(s, *allocator)) )
	OUTFAIL;
    }
    if ( !PL_get_nil_ex(tail) )
      OUTFAIL;

  out:
    if ( !rc )
    { free_rcl_arglist(*arg_values, argp-*arg_values, allocator);
      rc = PL_resource_error("memory");
    }
  }

  return rc;
}


static int
rclswi_parse_args(term_t list, rcl_arguments_t *parsed_args)
{ int num_args = 0;
  char **arg_values = NULL;
  int rc = TRUE;
  int unparsed;

  if ( !rclswi_get_arglist(list, &num_args, &arg_values, &rclswi_default_allocator) )
    return FALSE;

  TRY(rcl_parse_arguments(num_args, (const char**)arg_values, rclswi_default_allocator, parsed_args));
  if ( (unparsed=rcl_arguments_get_count_unparsed_ros(parsed_args)) > 0 )
  { Sdprintf("TBD: Unparsed arguments\n");
  }

  free_rcl_arglist(arg_values, num_args, &rclswi_default_allocator);
  return rc;
}


		 /*******************************
		 *	       NODE		*
		 *******************************/

typedef struct
{ rcl_node_t node;
  atom_t     context_symbol;
} rclswi_node_t;


static void
free_rcl_node(void* ptr)
{ rclswi_node_t *node = ptr;

  PL_unregister_atom(node->context_symbol);
  TRYVOID(rcl_node_fini(&node->node));
}


static foreign_t
ros_create_node(term_t Context, term_t Name, term_t Node, term_t Options)
{ rcl_context_t *context;
  rclswi_node_t *node;
  rcl_node_options_t options = rcl_node_get_default_options();
  rcl_arguments_t arguments = rcl_get_zero_initialized_arguments();
  int use_global_arguments = TRUE;
  int local_arguments = FALSE;
  int enable_rosout = TRUE;
  char *node_name;
  char *namespace = "";
  int rc = TRUE;
  term_t tail = PL_copy_term_ref(Options);
  term_t head = PL_new_term_ref();
  term_t arg = PL_new_term_ref();
  atom_t context_symbol;

  if ( !PL_get_chars(Name, &node_name, CVT_ATOMIC|CVT_EXCEPTION|REP_UTF8) )
    return FALSE;
  if ( !get_pointer_and_symbol(Context, (void**)&context, &context_symbol,
			       &context_type) )
    return FALSE;

  while(PL_get_list_ex(tail, head, tail))
  { atom_t name;
    size_t arity;

    if ( PL_get_name_arity(head, &name, &arity) && arity == 1)
    { _PL_get_arg(1, head, arg);

      if ( name == ATOM_ros_args )
      { if ( !rclswi_parse_args(arg, &arguments) )
	  OUTFAIL;
	local_arguments = TRUE;
      } else if ( name == ATOM_rosout )
      { if ( !PL_get_bool_ex(arg, &enable_rosout) )
	  OUTFAIL;
      }
    } else
    { return PL_type_error("option", head);
    }
  }
  if ( !PL_get_nil_ex(tail) )
    return FALSE;

  if ( rc && !local_arguments )
  { const char *av[] = {NULL};
    TRY(rcl_parse_arguments(0, av, rclswi_default_allocator,  &arguments));
  }

  if ( rc )
  { if ( !(node=malloc(sizeof(*node))) )
    { rc=PL_resource_error("memory");
      OUTFAIL;
    }
    node->context_symbol = context_symbol;
    node->node = rcl_get_zero_initialized_node();
    options.use_global_arguments = use_global_arguments;
    options.arguments = arguments;
    options.enable_rosout = enable_rosout;
  }

  TRY(rcl_node_init(&node->node, node_name, namespace, context, &options));

out:
  TRY_ANYWAY(rcl_arguments_fini(&arguments));

  if ( rc )
  { PL_register_atom(node->context_symbol);
    return unify_pointer(Node, node, &node_type);
  }

  return rc;
}


static foreign_t
ros_node_fini(term_t Node)
{ rclswi_node_t *node;
  int rc = TRUE;

  if ( !get_pointer(Node, (void**)&node, &node_type) )
    return FALSE;

  if ( rcl_node_is_valid(&node->node) )
    TRY(rcl_node_fini(&node->node));
  else
    rc = PL_existence_error("ros_node", Node);

  return rc;
}


static foreign_t
ros_node_prop(term_t Node, term_t Prop, term_t Value)
{ rclswi_node_t *node;
  atom_t prop;

  if ( !get_pointer(Node, (void**)&node, &node_type) ||
       !PL_get_atom_ex(Prop, &prop) )
    return FALSE;

  if ( prop == ATOM_name )
  { const char *node_name = rcl_node_get_name(&node->node);
    if ( node_name )
      return PL_unify_chars(Value, PL_ATOM|REP_UTF8, (size_t)-1, node_name);
  } else if ( prop == ATOM_namespace )
  { const char *namespace = rcl_node_get_namespace(&node->node);
    if ( namespace )
      return PL_unify_chars(Value, PL_ATOM|REP_UTF8, (size_t)-1, namespace);
  } else if ( prop == ATOM_logger_name )
  { const char *logger_name = rcl_node_get_logger_name(&node->node);
    if ( logger_name )
      return PL_unify_chars(Value, PL_ATOM|REP_UTF8, (size_t)-1, logger_name);
  } else if ( prop == ATOM_context )
  { return PL_unify_atom(Value, node->context_symbol);
  }

  return FALSE;
}


		 /*******************************
		 *	     PUBLISH		*
		 *******************************/

typedef struct
{
  // Important: a pointer to a structure is also a pointer to its first member.
  // The publisher must be first in the struct to compare pub.handle.pointer to an address
  // in a wait set.
  rcl_publisher_t publisher;
  rcl_node_t * node;
  rclswi_message_type_t *type_support;
} rclswi_publisher_t;

static void
free_rcl_publisher(void *ptr)
{ rclswi_publisher_t *pub = ptr;

  TRYVOID(rcl_publisher_fini(&pub->publisher, pub->node));
}


static foreign_t
ros_publisher(term_t Node, term_t MsgType, term_t Topic, term_t QoSProfile,
	      term_t Publisher)
{ rclswi_node_t *node;
  int rc = TRUE;
  char *topic;
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
  rclswi_publisher_t *pub = NULL;
  rclswi_message_type_t *msg_type;

  if ( !get_pointer(Node, (void**)&node, &node_type) )
    return FALSE;
  if ( !get_pointer(MsgType, (void**)&msg_type, &rclswi_message_type_type) )
    return FALSE;
  if ( !PL_get_chars(Topic, &topic, CVT_ATOM|CVT_STRING|CVT_EXCEPTION|REP_UTF8) )
    return FALSE;

  if ( !PL_is_variable(QoSProfile) )
  { rmw_qos_profile_t *qos_profile;
    if ( !get_qos_profile(QoSProfile, &qos_profile) )
      return FALSE;
    publisher_ops.qos = *qos_profile;
  }

  if ( !(pub = malloc(sizeof(*pub))) )
    return PL_resource_error("memory");
  pub->publisher = rcl_get_zero_initialized_publisher();
  pub->node = &node->node;
  pub->type_support = msg_type;

  TRY(rcl_publisher_init(&pub->publisher, &node->node, msg_type->type_support,
			 topic, &publisher_ops));

  if ( !rc )
  { free(pub);
  } else
  { rc = unify_pointer(Publisher, pub, &publisher_type);
  }

  return rc;
}

static foreign_t
ros_publish(term_t Publisher, term_t Message)
{ rclswi_publisher_t *pub;
  void *msg = NULL;
  int rc = TRUE;

  if ( !get_pointer(Publisher, (void**)&pub, &publisher_type) )
    return FALSE;

  msg = (*pub->type_support->create)();
  (*pub->type_support->init)(msg);

  if ( (rc=fill_message(Message, msg, pub->type_support->introspection, FALSE)) )
    TRY(rcl_publish(&pub->publisher, msg, NULL));

  (*pub->type_support->fini)(msg);
  (*pub->type_support->destroy)(msg);

  return rc;
}



		 /*******************************
		 *	   SUBSCRIPTION		*
		 *******************************/

typedef struct
{
  // Important: a pointer to a structure is also a pointer to its first member.
  // The subscription must be first in the struct to compare sub.handle.pointer to an address
  // in a wait set.
  rcl_subscription_t subscription;
  rcl_node_t * node;
  rclswi_message_type_t *type_support;
  int waiting;
  int deleted;
} rclswi_subscription_t;

static void
free_rcl_subscription(void *ptr)
{ rclswi_subscription_t *sub = ptr;

  TRYVOID(rcl_subscription_fini(&sub->subscription, sub->node));
}

static foreign_t
ros_subscribe(term_t Node, term_t MsgType, term_t Topic, term_t QoSProfile,
	      term_t Subscription)
{ rclswi_node_t *node;
  int rc = TRUE;
  char *topic;
  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rclswi_subscription_t *sub = NULL;
  rclswi_message_type_t *msg_type;

  if ( !get_pointer(Node, (void**)&node, &node_type) )
    return FALSE;
  if ( !get_pointer(MsgType, (void**)&msg_type, &rclswi_message_type_type) )
    return FALSE;
  if ( !get_utf8_name_ex(Topic, &topic) )
    return FALSE;

  if ( !PL_is_variable(QoSProfile) )
  { rmw_qos_profile_t *qos_profile;
    if ( !get_qos_profile(QoSProfile, &qos_profile) )
      return FALSE;
    subscription_ops.qos = *qos_profile;
  }

  if ( !(sub = malloc(sizeof(*sub))) )
    return PL_resource_error("memory");
  sub->subscription = rcl_get_zero_initialized_subscription();
  sub->node = &node->node;
  sub->type_support = msg_type;
  sub->waiting = FALSE;
  sub->deleted = FALSE;

  TRY(rcl_subscription_init(&sub->subscription, &node->node,
			    msg_type->type_support, topic, &subscription_ops));

  if ( !rc )
  { free(sub);
  } else
  { rc = unify_pointer(Subscription, sub, &subscription_type);
  }

  return rc;
}


static foreign_t
ros_unsubscribe(term_t Subscription)
{ rclswi_subscription_t *sub;
  int rc = TRUE;

  if ( !get_pointer(Subscription, (void**)&sub, &subscription_type) )
    return FALSE;

  sub->deleted = TRUE;			/* TBD: sync properly */
  if ( !sub->waiting )
    TRY(rcl_subscription_fini(&sub->subscription, sub->node));

  return rc;
}


static foreign_t
ros_take(term_t Subscription, term_t Message, term_t MsgInfo)
{ rclswi_subscription_t *sub;
  rmw_message_info_t msg_info;
  void *msg = NULL;
  int rc = TRUE;
  term_t result = PL_new_term_ref();

  if ( !get_pointer(Subscription, (void**)&sub, &subscription_type) )
    return FALSE;

  msg = (*sub->type_support->create)();
  (*sub->type_support->init)(msg);

  TRY(rcl_take(&sub->subscription, msg, &msg_info, NULL));
  rc = rc && put_message(result, msg, sub->type_support->introspection)
	  && PL_unify(Message, result);

  (*sub->type_support->fini)(msg);
  (*sub->type_support->destroy)(msg);

  return rc;
}


		 /*******************************
		 *       SERVICES (CLIENT)	*
		 *******************************/

typedef struct
{ rcl_client_t      client;			/* Must be first */
  rcl_node_t        *node;
  rclswi_srv_type_t *type_support;
  int		     waiting;
  int		     deleted;
} rclswi_client_t;


static void
free_rcl_client(void *ptr)
{ rclswi_client_t *client = ptr;

  TRYVOID(rcl_client_fini(&client->client, client->node));
}


static foreign_t
ros_create_client(term_t Node, term_t SrvType, term_t Name, term_t QoSProfile,
		  term_t Client)
{ rclswi_node_t *node;
  int rc = TRUE;
  char *service_name;
  rclswi_srv_type_t *srv_type;
  rclswi_client_t *client;
  rcl_client_options_t client_ops = rcl_client_get_default_options();

  if ( !get_pointer(Node, (void**)&node, &node_type) )
    return FALSE;
  if ( !get_pointer(SrvType, (void**)&srv_type, &rclswi_srv_type_type) )
    return FALSE;
  if ( !get_utf8_name_ex(Name, &service_name) )
    return FALSE;

  if ( !PL_is_variable(QoSProfile) )
  { rmw_qos_profile_t *qos_profile;
    if ( !get_qos_profile(QoSProfile, &qos_profile) )
      return FALSE;
    client_ops.qos = *qos_profile;
  }

  if ( !(client = malloc(sizeof(*client))) )
    return PL_resource_error("memory");
  client->client       = rcl_get_zero_initialized_client();
  client->node	       = &node->node;
  client->type_support = srv_type;
  client->waiting      = FALSE;
  client->deleted      = FALSE;

  TRY(rcl_client_init(&client->client, &node->node,
		      srv_type->type_support, service_name, &client_ops));

  if ( !rc )
    free(client);
  else
    rc = unify_pointer(Client, client, &client_type);

  return rc;
}


typedef struct
{ const rclswi_message_type_t	*msg_type;
  void			        *msg;
  int64_t			 seq_number;
  term_t			 Message;
  term_t			 SeqNumber;
} send_request_context_t;

static int
prepare_send_request(send_request_context_t *ctx)
{ if ( (ctx->msg = (*ctx->msg_type->create)()) )
  { (*ctx->msg_type->init)(ctx->msg);

    return fill_message(ctx->Message, ctx->msg, ctx->msg_type->introspection,
			FALSE);
  } else
    return PL_resource_error("memory");
}

static int
finish_send_request(send_request_context_t *ctx, int rc)
{ (*ctx->msg_type->fini)(ctx->msg);
  (*ctx->msg_type->destroy)(ctx->msg);

  return rc && PL_unify_int64(ctx->SeqNumber, ctx->seq_number);
}

static foreign_t
ros_send_request(term_t Client, term_t Message, term_t SeqNumber)
{ rclswi_client_t *client;
  int rc;

  if ( !get_pointer(Client, (void**)&client, &client_type) )
    return FALSE;

  send_request_context_t ctx = { .msg_type  = &client->type_support->request,
                                 .Message   = Message,
				 .SeqNumber = SeqNumber
			       };

  rc = prepare_send_request(&ctx);
  TRY(rcl_send_request(&client->client, ctx.msg, &ctx.seq_number));
  return finish_send_request(&ctx, rc);
}


static foreign_t
ros_service_is_ready(term_t Client)
{ rclswi_client_t *client;
  int rc = TRUE;
  bool is_ready;

  if ( !get_pointer(Client, (void**)&client, &client_type) )
    return FALSE;

  TRY(rcl_service_server_is_available(client->node, &client->client, &is_ready));

  return rc && is_ready;
}


static void
put_hex(char **out, uint8_t byte)
{ static const char hexd[] = "0123456789abcdef";

  *(*out)++ = hexd[(byte>>4)&0xf];
  *(*out)++ = hexd[(byte)&0xf];
}

static int
put_guid(term_t t, const uint8_t *guid)
{ char out[36];
  char *o = out;

  for(int i=0; i<16; i++)
  { put_hex(&o, guid[i]);
    if ( i == 3 || i == 5 || i == 7 || i == 9 )
      *o++ = '-';
  }

  return PL_put_atom_nchars(t, 36, out);
}

static int
put_time_stamp(term_t t, int64_t stamp)
{ double d = (double)stamp/1000000000.0;

  return PL_put_float(t, d);
}

static int
put_msg_time(term_t Stamp, const builtin_interfaces__msg__Time *tp)
{ double d = (double)tp->sec + (double)tp->nanosec/1000000000.0;

  return PL_put_float(Stamp, d);
}



static int
put_rwm_request_id(term_t t, const rmw_request_id_t *request_id)
{ const atom_t reqid_keys[] = { ATOM_writer_guid, ATOM_sequence_number };
  term_t reqid_values = PL_new_term_refs(2);

  return
  ( (reqid_values = PL_new_term_refs(2)) &&
    put_guid(reqid_values+0, (const uint8_t*)request_id->writer_guid) &&
    PL_put_int64(reqid_values+1, request_id->sequence_number) &&
    PL_put_dict(t, ATOM_request_id, 2, reqid_keys, reqid_values) &&
    (PL_reset_term_refs(reqid_values),TRUE)
  );
}

static int
put_rmw_service_info(term_t t, const rmw_service_info_t *header)
{ const atom_t hdr_keys[] = { ATOM_source_timestamp, ATOM_received_timestamp,
			      ATOM_request_id };
  term_t hdr_values = PL_new_term_refs(3);

  return
  ( (hdr_values = PL_new_term_refs(3)) &&
    put_time_stamp(hdr_values+0, header->source_timestamp) &&
    put_time_stamp(hdr_values+1, header->received_timestamp) &&
    put_rwm_request_id(hdr_values+2, &header->request_id) &&
    PL_put_dict(t, ATOM_service_info, 3, hdr_keys, hdr_values) &&
    (PL_reset_term_refs(hdr_values),TRUE)
  );
}

static foreign_t
ros_take_response(term_t Client, term_t Message, term_t MessageInfo)
{ rclswi_client_t *client;
  rmw_service_info_t header;
  void *msg = NULL;
  int rc = TRUE;
  term_t result = PL_new_term_ref();

  if ( !get_pointer(Client, (void**)&client, &client_type) )
    return FALSE;

  const rclswi_message_type_t *msg_type = &client->type_support->response;
  msg = (*msg_type->create)();
  (*msg_type->init)(msg);

  TRY(rcl_take_response_with_info(&client->client, &header, msg));
  rc = rc && put_message(result, msg, msg_type->introspection)
	  && PL_unify(Message, result)
          && put_rmw_service_info(result, &header)
	  && PL_unify(MessageInfo, result);

  (*msg_type->fini)(msg);
  (*msg_type->destroy)(msg);

  return rc;
}

		 /*******************************
		 *       SERVICES (SERVER)	*
		 *******************************/

typedef struct
{ rcl_service_t      service;			/* Must be first */
  rcl_node_t        *node;
  rclswi_srv_type_t *type_support;
  atom_t	     node_symbol;		/* Node blob */
  atom_t	     name;			/* Service name */
  int		     waiting;
  int		     deleted;
} rclswi_service_t;

static void
free_rcl_service(void *ptr)
{ rclswi_service_t *service = ptr;

  PL_unregister_atom(service->node_symbol);
  PL_unregister_atom(service->name);

  TRYVOID(rcl_service_fini(&service->service, service->node));
}

static foreign_t
ros_create_service(term_t Node, term_t SrvType, term_t Name, term_t QoSProfile,
		   term_t Service)
{ rclswi_node_t *node;
  int rc = TRUE;
  char *service_name;
  rclswi_srv_type_t *srv_type;
  rclswi_service_t *service;
  rcl_service_options_t service_ops = rcl_service_get_default_options();
  atom_t node_symbol;

  if ( !get_pointer_and_symbol(Node, (void**)&node, &node_symbol, &node_type) )
    return FALSE;
  if ( !get_pointer(SrvType, (void**)&srv_type, &rclswi_srv_type_type) )
    return FALSE;
  if ( !get_utf8_name_ex(Name, &service_name) )
    return FALSE;

  if ( !PL_is_variable(QoSProfile) )
  { rmw_qos_profile_t *qos_profile;
    if ( !get_qos_profile(QoSProfile, &qos_profile) )
      return FALSE;
    service_ops.qos = *qos_profile;
  }

  if ( !(service = malloc(sizeof(*service))) )
    return PL_resource_error("memory");
  service->service      = rcl_get_zero_initialized_service();
  service->node	        = &node->node;
  service->type_support = srv_type;
  service->name		= PL_new_atom(service_name);
  service->node_symbol  = node_symbol;
  service->waiting      = FALSE;
  service->deleted      = FALSE;

  TRY(rcl_service_init(&service->service, &node->node,
		      srv_type->type_support, service_name, &service_ops));

  if ( !rc )
  { PL_unregister_atom(service->name);
    free(service);
  } else
  { PL_register_atom(service->node_symbol);
    rc = unify_pointer(Service, service, &service_type);
  }

  return rc;
}

static foreign_t
ros_service_prop(term_t Service, term_t Key, term_t Value)
{ rclswi_service_t *service;
  atom_t key;

  if ( get_pointer(Service, (void**)&service, &service_type) &&
       PL_get_atom_ex(Key, &key) )
  { if ( key == ATOM_node )
      return PL_unify_atom(Value, service->node_symbol);
    else if ( key == ATOM_name )
      return PL_unify_atom(Value, service->name);
  }

  return FALSE;
}


static void
free_rwm_service_info(void* ptr)
{ rmw_service_info_t *header = ptr;

  free(header);
}

static foreign_t
ros_service_info_to_prolog(term_t Header, term_t Dict)
{ rmw_service_info_t *header;
  term_t tmp = PL_new_term_ref();

  return ( get_pointer(Header, (void**)&header, &rwm_service_info_type) &&
	   put_rmw_service_info(tmp, header) &&
	   PL_unify(tmp, Dict) );
}

static foreign_t
ros_send_response(term_t Service, term_t Message, term_t Header)
{ rclswi_service_t *service;
  void *msg = NULL;
  int rc = TRUE;
  rmw_service_info_t *header = NULL;

  if ( !get_pointer(Service, (void**)&service, &service_type) ||
       !get_pointer(Header,  (void**)&header,  &rwm_service_info_type) )
    return FALSE;

  const rclswi_message_type_t *msg_type = &service->type_support->response;
  msg = (*msg_type->create)();
  (*msg_type->init)(msg);

  if ( (rc=fill_message(Message, msg, msg_type->introspection, FALSE)) )
    TRY(rcl_send_response(&service->service, &header->request_id, msg));

  (*msg_type->fini)(msg);
  (*msg_type->destroy)(msg);

  return rc;
}

typedef struct
{ const rclswi_message_type_t	*msg_type;
  void			        *msg;
  rmw_service_info_t		*header;
  term_t			 result;
  term_t			 Message;
  term_t			 MessageInfo;
} take_request_context_t;

static int
prepare_take_request(take_request_context_t *ctx)
{ if ( (ctx->result = PL_new_term_ref()) &&
       (ctx->header = malloc(sizeof(*ctx->header))) &&
       (ctx->msg    = (*ctx->msg_type->create)()) )
  { (*ctx->msg_type->init)(ctx->msg);
    return TRUE;
  } else
    return PL_resource_error("memory");
}

static int
finish_take_request(take_request_context_t *ctx, int rc)
{ rc = rc && put_message(ctx->result, ctx->msg, ctx->msg_type->introspection)
	  && PL_unify(ctx->Message, ctx->result)
          && unify_pointer(ctx->MessageInfo, ctx->header, &rwm_service_info_type);

  (*ctx->msg_type->fini)(ctx->msg);
  (*ctx->msg_type->destroy)(ctx->msg);

  if ( !rc && ctx->header )
    free(ctx->header);

  return rc;
}


static foreign_t
ros_take_request(term_t Service, term_t Message, term_t MessageInfo)
{ rclswi_service_t *service;
  int rc;

  if ( !get_pointer(Service, (void**)&service, &service_type) )
    return FALSE;

  take_request_context_t ctx = { .msg_type = &service->type_support->request,
				 .Message = Message,
				 .MessageInfo = MessageInfo
			       };

  rc = prepare_take_request(&ctx);
  TRY(rcl_take_request_with_info(&service->service, ctx.header, ctx.msg));
  return finish_take_request(&ctx, rc);
}


		 /*******************************
		 *	      CLOCK		*
		 *******************************/

typedef struct
{ rcl_clock_t clock;
  atom_t context_symbol;
  atom_t type;
} rclswi_clock_t;

static void
free_rcl_clock(void *ptr)
{ rclswi_clock_t *clock = ptr;

  /* no need to unregister ->type as this is a global atom */
  TRYVOID(rcl_clock_fini(&clock->clock));
}

static foreign_t
ros_create_clock(term_t Context, term_t Type, term_t Clock)
{ rcl_context_t *context;
  rcl_clock_type_t type;
  int rc = TRUE;
  atom_t type_a;
  atom_t context_symbol;
  rclswi_clock_t *clock;

  if ( !get_pointer_and_symbol(Context, (void**)&context, &context_symbol, &context_type) )
    return FALSE;
  if ( PL_get_atom_ex(Type, &type_a) )
  { if ( type_a == ATOM_ros )
      type = RCL_ROS_TIME;
    else if ( type_a == ATOM_system )
      type = RCL_SYSTEM_TIME;
    else if ( type_a == ATOM_steady )
      type = RCL_STEADY_TIME;
    else
      return PL_domain_error("ros_clock_type", Type);
  }

  if ( !(clock = malloc(sizeof(*clock))) )
    return PL_resource_error("memory");
  clock->type = type_a;
  clock->context_symbol = context_symbol;

  TRY(rcl_clock_init(type, &clock->clock, &rclswi_default_allocator));

  if ( rc )
  { PL_register_atom(clock->context_symbol);
    rc = unify_pointer(Clock, clock, &clock_type);
  }

  return rc;
}

static int
unify_time_stamp(term_t t, int64_t stamp)
{ double d = (double)stamp/1000000000.0;

  return PL_unify_float(t, d);
}

static foreign_t
ros_clock_prop(term_t Clock, term_t Key, term_t Value)
{ rclswi_clock_t *clock;
  atom_t key;

  if ( get_pointer(Clock, (void**)&clock, &clock_type) &&
       PL_get_atom_ex(Key, &key) )
  { if ( key == ATOM_context )
      return PL_unify_atom(Value, clock->context_symbol);
    else if ( key == ATOM_type )
      return PL_unify_atom(Value, clock->type);
  }

  return FALSE;
}

static foreign_t
ros_clock_time(term_t Clock, term_t Time)
{ int rc = TRUE;
  rclswi_clock_t *clock;
  rcl_time_point_value_t now;

  if ( !get_pointer(Clock, (void**)&clock, &clock_type) )
    return FALSE;

  TRY(rcl_clock_get_now(&clock->clock, &now));

  return rc && unify_time_stamp(Time, now);
}


		 /*******************************
		 *	 ACTIONS (CLIENT)	*
		 *******************************/

/* Service type for action_msgs/srv/CancelGoal */
static rclswi_srv_type_t *cancel_type_ptr = NULL;

static foreign_t
set_action_cancel_type(term_t SrvType)
{ atom_t symbol;

  if ( !get_pointer_and_symbol(SrvType, (void**)&cancel_type_ptr, &symbol,
			       &rclswi_srv_type_type) )
    return FALSE;

  PL_register_atom(symbol);
  return TRUE;
}

static rclswi_srv_type_t *
goal_cancel_type_support(void)
{ if ( !cancel_type_ptr )
  { predicate_t pred = PL_predicate("init_cancel_type", 0, "ros_detail_actions");

    if ( !PL_call_predicate(NULL, PL_Q_NODEBUG|PL_Q_PASS_EXCEPTION, pred, 0) )
      return NULL;
  }

  return cancel_type_ptr;
}


typedef struct
{ rcl_action_client_t   action_client;			/* Must be first */
  rcl_node_t           *node;
  rclswi_action_type_t *type_support;
  atom_t		node_symbol;
  atom_t		name;
  int		        waiting;
  int		        deleted;
} rclswi_action_client_t;


static void
free_rcl_action_client(void *ptr)
{ rclswi_action_client_t *client = ptr;

  PL_unregister_atom(client->node_symbol);
  PL_unregister_atom(client->name);

  TRYVOID(rcl_action_client_fini(&client->action_client, client->node));
}


static int
copy_qos(term_t dict, atom_t key, rmw_qos_profile_t *into)
{ term_t tmp = PL_new_term_ref();

  if ( PL_get_dict_key(key, dict, tmp) )
  { rmw_qos_profile_t *qos_profile;

    if ( !get_qos_profile(tmp, &qos_profile) )
      return FALSE;
    *into = *qos_profile;
    return TRUE;
  }

  return TRUE;
}

static foreign_t
ros_create_action_client(term_t Node, term_t ActType, term_t Name,
			 term_t QoSDict,
			 term_t ActionClient)
{ rclswi_node_t *node;
  int rc = TRUE;
  char *action_name;
  rclswi_action_type_t *action_type;
  rclswi_action_client_t *action_client;
  rcl_action_client_options_t action_ops;
  atom_t node_symbol;

  if ( !get_pointer_and_symbol(Node, (void**)&node, &node_symbol, &node_type) )
    return FALSE;
  if ( !get_pointer(ActType, (void**)&action_type, &rclswi_action_type_type) )
    return FALSE;
  if ( !get_utf8_name_ex(Name, &action_name) )
    return FALSE;

  action_ops = rcl_action_client_get_default_options();
  if ( !copy_qos(QoSDict, ATOM_goal,     &action_ops.goal_service_qos)   ||
       !copy_qos(QoSDict, ATOM_result,   &action_ops.result_service_qos) ||
       !copy_qos(QoSDict, ATOM_cancel,   &action_ops.cancel_service_qos) ||
       !copy_qos(QoSDict, ATOM_feedback, &action_ops.feedback_topic_qos) ||
       !copy_qos(QoSDict, ATOM_status,   &action_ops.status_topic_qos) )
    return FALSE;

  if ( !(action_client = malloc(sizeof(*action_client))) )
    return PL_resource_error("memory");
  action_client->action_client = rcl_action_get_zero_initialized_client();
  action_client->node	       = &node->node;
  action_client->type_support  = action_type;
  action_client->name	       = PL_new_atom(action_name);
  action_client->node_symbol   = node_symbol;
  action_client->waiting       = FALSE;
  action_client->deleted       = FALSE;

  TRY(rcl_action_client_init(
	  &action_client->action_client, &node->node,
	  action_type->type_support, action_name, &action_ops));

  if ( !rc )
  { PL_unregister_atom(action_client->name);
    free(action_client);
  } else
  { PL_register_atom(action_client->node_symbol);
    rc = unify_pointer(ActionClient, action_client, &action_client_type);
  }

  return rc;
}


static foreign_t
ros_action_client_prop(term_t ActionClient, term_t Key, term_t Value)
{ rclswi_action_client_t *action_client;
  atom_t key;

  if ( get_pointer(ActionClient, (void**)&action_client, &action_client_type) &&
       PL_get_atom_ex(Key, &key) )
  { if ( key == ATOM_node )
      return PL_unify_atom(Value, action_client->node_symbol);
    else if ( key == ATOM_name )
      return PL_unify_atom(Value, action_client->name);
  }

  return FALSE;
}

		 /*******************************
		 *	 ACTIONS (SERVER)	*
		 *******************************/

typedef struct
{ rcl_action_server_t   action_server;			/* Must be first */
  rcl_node_t           *node;
  rclswi_action_type_t *type_support;
  atom_t		node_symbol;
  atom_t		clock_symbol;
  atom_t		name;
  record_t		goal;
  int		        waiting;
  int		        deleted;
} rclswi_action_server_t;


static void
free_rcl_action_server(void *ptr)
{ rclswi_action_server_t *server = ptr;

  PL_unregister_atom(server->node_symbol);
  PL_unregister_atom(server->clock_symbol);
  PL_unregister_atom(server->name);
  PL_erase(server->goal);

  TRYVOID(rcl_action_server_fini(&server->action_server, server->node));
}


static foreign_t
ros_create_action_server(term_t Node, term_t Clock, term_t ActType, term_t Name,
			 term_t QoSDict, term_t ResultTimeOut, term_t Goal,
			 term_t ActionServer)
{ rclswi_node_t *node;
  int rc = TRUE;
  char *action_name;
  rclswi_action_type_t *action_type;
  rclswi_action_server_t *action_server;
  rcl_action_server_options_t action_ops;
  atom_t node_symbol;
  atom_t clock_symbol;
  int64_t tmo_nsec;
  rclswi_clock_t *clock = NULL;

  if ( !get_pointer_and_symbol(Node, (void**)&node, &node_symbol, &node_type) )
    return FALSE;
  if ( !get_pointer_and_symbol(Clock, (void**)&clock, &clock_symbol, &clock_type) )
    return FALSE;
  if ( !get_pointer(ActType, (void**)&action_type, &rclswi_action_type_type) )
    return FALSE;
  if ( !get_utf8_name_ex(Name, &action_name) )
    return FALSE;
  if ( !get_timeout_nsec(ResultTimeOut, &tmo_nsec) )
    return FALSE;

  action_ops = rcl_action_server_get_default_options();
  /* TBD: Fill QoS options */
  action_ops.result_timeout.nanoseconds = tmo_nsec;

  if ( !(action_server = malloc(sizeof(*action_server))) )
    return PL_resource_error("memory");
  action_server->action_server = rcl_action_get_zero_initialized_server();
  action_server->node	       = &node->node;
  action_server->type_support  = action_type;
  action_server->name	       = PL_new_atom(action_name);
  action_server->node_symbol   = node_symbol;
  action_server->clock_symbol  = clock_symbol;
  action_server->waiting       = FALSE;
  action_server->deleted       = FALSE;
  action_server->goal          = PL_record(Goal);

  TRY(rcl_action_server_init(
	  &action_server->action_server, &node->node,
	  &clock->clock,
	  action_type->type_support, action_name, &action_ops));

  if ( !rc )
  { PL_unregister_atom(action_server->name);
    PL_erase(action_server->goal);
    free(action_server);
  } else
  { PL_register_atom(action_server->node_symbol);
    PL_register_atom(action_server->clock_symbol);
    rc = unify_pointer(ActionServer, action_server, &action_server_type);
  }

  return rc;
}


static foreign_t
ros_action_server_prop(term_t ActionServer, term_t Key, term_t Value)
{ rclswi_action_server_t *action_server;
  atom_t key;
  term_t tmp;

  if ( get_pointer(ActionServer, (void**)&action_server, &action_server_type) &&
       PL_get_atom_ex(Key, &key) )
  { if ( key == ATOM_node )
      return PL_unify_atom(Value, action_server->node_symbol);
    else if ( key == ATOM_name )
      return PL_unify_atom(Value, action_server->name);
    else if ( key == ATOM_clock )
      return PL_unify_atom(Value, action_server->clock_symbol);
    else if ( key == ATOM_goal )
      return ( (tmp=PL_new_term_ref()) &&
	       PL_recorded(action_server->goal, tmp) &&
	       PL_unify(tmp, Value));
  }

  return FALSE;
}


/* Sending action requests */

static foreign_t
ros_action_send_goal_request(term_t ActionClient, term_t Message, term_t SeqNum)
{ rclswi_action_client_t *action_client;
  int rc;

  if ( !get_pointer(ActionClient, (void**)&action_client, &action_client_type) )
    return FALSE;

  send_request_context_t ctx = { .msg_type  = &action_client->type_support->goal.request,
                                 .Message   = Message,
				 .SeqNumber = SeqNum
			       };

  rc = prepare_send_request(&ctx);
  TRY(rcl_action_send_goal_request(&action_client->action_client,
				   ctx.msg, &ctx.seq_number));
  return finish_send_request(&ctx, rc);
}


static foreign_t
ros_action_send_result_request(term_t ActionClient, term_t Message, term_t SeqNum)
{ rclswi_action_client_t *action_client;
  int rc;

  if ( !get_pointer(ActionClient, (void**)&action_client, &action_client_type) )
    return FALSE;

  send_request_context_t ctx = { .msg_type  = &action_client->type_support->result.request,
                                 .Message   = Message,
				 .SeqNumber = SeqNum
			       };

  rc = prepare_send_request(&ctx);
  TRY(rcl_action_send_result_request(&action_client->action_client,
				     ctx.msg, &ctx.seq_number));
  return finish_send_request(&ctx, rc);
}


static foreign_t
ros_action_send_cancel_request(term_t ActionClient, term_t Message, term_t SeqNum)
{ rclswi_action_client_t *action_client;
  int rc;
  rclswi_srv_type_t *cancel_type;

  if ( !get_pointer(ActionClient, (void**)&action_client, &action_client_type) )
    return FALSE;
  if ( !(cancel_type=goal_cancel_type_support()) )
    return FALSE;

  send_request_context_t ctx = { .msg_type  = &cancel_type->request,
                                 .Message   = Message,
				 .SeqNumber = SeqNum
			       };

  rc = prepare_send_request(&ctx);
  TRY(rcl_action_send_cancel_request(&action_client->action_client,
				     ctx.msg, &ctx.seq_number));
  return finish_send_request(&ctx, rc);
}


/* Taking action requests */

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
This is very similar to ros_take_request()  for services, but the action
variation thereof has a header if   type  `rmw_request_id_t` rather than
`rmw_service_info_t`.
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

typedef struct
{ const rclswi_message_type_t	*msg_type;
  void			        *msg;
  rmw_request_id_t		*header;
  term_t			 result;
  term_t			 Message;
  term_t			 MessageInfo;
} action_take_request_context_t;

static int
action_prepare_take_request(action_take_request_context_t *ctx)
{ if ( (ctx->result = PL_new_term_ref()) &&
       (ctx->header = malloc(sizeof(*ctx->header))) &&
       (ctx->msg    = (*ctx->msg_type->create)()) )
  { (*ctx->msg_type->init)(ctx->msg);
    return TRUE;
  } else
    return PL_resource_error("memory");
}

static int
action_finish_take_request(action_take_request_context_t *ctx, int rc)
{ rc = rc && put_message(ctx->result, ctx->msg, ctx->msg_type->introspection)
	  && PL_unify(ctx->Message, ctx->result)
          && unify_pointer(ctx->MessageInfo, ctx->header, &rmw_request_id_type);

  (*ctx->msg_type->fini)(ctx->msg);
  (*ctx->msg_type->destroy)(ctx->msg);

  if ( !rc && ctx->header )
    free(ctx->header);

  return rc;
}


static foreign_t
ros_action_take_goal_request(term_t ActionServer, term_t Msg, term_t MsgInfo)
{ rclswi_action_server_t *action_server;
  int rc;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;

  action_take_request_context_t ctx =
  { .msg_type    = &action_server->type_support->goal.request,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_take_request(&ctx);
  TRY(rcl_action_take_goal_request(&action_server->action_server,
				   ctx.header, ctx.msg));
  return action_finish_take_request(&ctx, rc);
}


static foreign_t
ros_action_take_result_request(term_t ActionServer, term_t Msg, term_t MsgInfo)
{ rclswi_action_server_t *action_server;
  int rc;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;

  action_take_request_context_t ctx =
  { .msg_type    = &action_server->type_support->result.request,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_take_request(&ctx);
  TRY(rcl_action_take_result_request(&action_server->action_server,
				     ctx.header, ctx.msg));
  return action_finish_take_request(&ctx, rc);
}


static foreign_t
ros_action_take_cancel_request(term_t ActionServer, term_t Msg, term_t MsgInfo)
{ rclswi_action_server_t *action_server;
  rclswi_srv_type_t *cancel_type;
  int rc;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;
  if ( !(cancel_type=goal_cancel_type_support()) )
    return FALSE;

  action_take_request_context_t ctx =
  { .msg_type    = &cancel_type->request,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_take_request(&ctx);
  TRY(rcl_action_take_cancel_request(&action_server->action_server,
				     ctx.header, ctx.msg));
  return action_finish_take_request(&ctx, rc);
}


/* Sending action responses */

typedef struct
{ const rclswi_message_type_t	*msg_type;
  void			        *msg;
  rmw_request_id_t		*header;
  term_t			 Message;
  term_t			 MessageInfo;
} action_send_response_context_t;


static int
action_prepare_send_response(action_send_response_context_t *ctx)
{ if ( !get_pointer(ctx->MessageInfo, (void**)&ctx->header,  &rmw_request_id_type) )
    return FALSE;

  if ( (ctx->msg = (*ctx->msg_type->create)()) )
  { (*ctx->msg_type->init)(ctx->msg);

    return fill_message(ctx->Message, ctx->msg, ctx->msg_type->introspection,
			FALSE);
  } else
    return PL_resource_error("memory");
}

static int
action_finish_send_response(action_send_response_context_t *ctx, int rc)
{ (*ctx->msg_type->fini)(ctx->msg);
  (*ctx->msg_type->destroy)(ctx->msg);

  return rc;
}


static foreign_t
ros_action_send_goal_response(term_t ActionServer, term_t Msg, term_t MsgInfo)
{ rclswi_action_server_t *action_server;
  int rc;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;

  action_send_response_context_t ctx =
  { .msg_type    = &action_server->type_support->goal.response,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_send_response(&ctx);
  TRY(rcl_action_send_goal_response(&action_server->action_server,
				    ctx.header, ctx.msg));
  return action_finish_send_response(&ctx, rc);
}


static foreign_t
ros_action_send_result_response(term_t ActionServer, term_t Msg, term_t MsgInfo)
{ rclswi_action_server_t *action_server;
  int rc;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;

  action_send_response_context_t ctx =
  { .msg_type    = &action_server->type_support->result.response,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_send_response(&ctx);
  TRY(rcl_action_send_result_response(&action_server->action_server,
				      ctx.header, ctx.msg));
  return action_finish_send_response(&ctx, rc);
}


static foreign_t
ros_action_send_cancel_response(term_t ActionServer, term_t Msg, term_t MsgInfo)
{ rclswi_action_server_t *action_server;
  rclswi_srv_type_t *cancel_type;
  int rc;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;
  if ( !(cancel_type=goal_cancel_type_support()) )
    return FALSE;

  action_send_response_context_t ctx =
  { .msg_type    = &cancel_type->response,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_send_response(&ctx);
  TRY(rcl_action_send_cancel_response(&action_server->action_server,
				      ctx.header, ctx.msg));
  return action_finish_send_response(&ctx, rc);
}


/* Taking action responses */

typedef struct
{ const rclswi_message_type_t	*msg_type;
  void			        *msg;
  rmw_request_id_t		 header;
  term_t			 result;
  term_t			 Message;
  term_t			 MessageInfo;
} action_take_response_context_t;


static int
action_prepare_take_response(action_take_response_context_t *ctx)
{ if ( (ctx->result = PL_new_term_ref()) &&
       (ctx->msg    = (*ctx->msg_type->create)()) )
  { (*ctx->msg_type->init)(ctx->msg);
    return TRUE;
  } else
    return PL_resource_error("memory");
}

static int
action_finish_take_response(action_take_response_context_t *ctx, int rc)
{ rc = rc && put_message(ctx->result, ctx->msg, ctx->msg_type->introspection)
	  && PL_unify(ctx->Message, ctx->result)
          && put_rwm_request_id(ctx->result, &ctx->header)
	  && PL_unify(ctx->MessageInfo, ctx->result);

  (*ctx->msg_type->fini)(ctx->msg);
  (*ctx->msg_type->destroy)(ctx->msg);

  return rc;
}


static foreign_t
ros_action_take_goal_response(term_t ActionClient, term_t Msg, term_t MsgInfo)
{ rclswi_action_client_t *action_client;
  int rc;

  if ( !get_pointer(ActionClient, (void**)&action_client, &action_client_type) )
    return FALSE;

  action_take_response_context_t ctx =
  { .msg_type    = &action_client->type_support->goal.response,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_take_response(&ctx);
  TRY(rcl_action_take_goal_response(&action_client->action_client,
				    &ctx.header, ctx.msg));
  return action_finish_take_response(&ctx, rc);
}


static foreign_t
ros_action_take_result_response(term_t ActionClient, term_t Msg, term_t MsgInfo)
{ rclswi_action_client_t *action_client;
  int rc;

  if ( !get_pointer(ActionClient, (void**)&action_client, &action_client_type) )
    return FALSE;

  action_take_response_context_t ctx =
  { .msg_type    = &action_client->type_support->result.response,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_take_response(&ctx);
  TRY(rcl_action_take_result_response(&action_client->action_client,
				      &ctx.header, ctx.msg));
  return action_finish_take_response(&ctx, rc);
}


static foreign_t
ros_action_take_cancel_response(term_t ActionClient, term_t Msg, term_t MsgInfo)
{ rclswi_action_client_t *action_client;
  rclswi_srv_type_t *cancel_type;
  int rc;

  if ( !get_pointer(ActionClient, (void**)&action_client, &action_client_type) )
    return FALSE;
  if ( !(cancel_type=goal_cancel_type_support()) )
    return FALSE;

  action_take_response_context_t ctx =
  { .msg_type    = &cancel_type->response,
    .Message     = Msg,
    .MessageInfo = MsgInfo
  };

  rc = action_prepare_take_response(&ctx);
  TRY(rcl_action_take_cancel_response(&action_client->action_client,
				      &ctx.header, ctx.msg));
  return action_finish_take_response(&ctx, rc);
}

/* Action feedback */

typedef struct
{ const rclswi_message_type_t	*msg_type;
  void			        *msg;
  term_t			 Message;
} action_send_message_context_t;

static int
action_prepare_send_message(action_send_message_context_t *ctx)
{ if ( (ctx->msg = (*ctx->msg_type->create)()) )
  { (*ctx->msg_type->init)(ctx->msg);
    return fill_message(ctx->Message, ctx->msg, ctx->msg_type->introspection,
			FALSE);
  } else
    return PL_resource_error("memory");
}

static int
action_finish_send_message(action_send_message_context_t *ctx, int rc)
{ (*ctx->msg_type->fini)(ctx->msg);
  (*ctx->msg_type->destroy)(ctx->msg);

  return rc;
}

static foreign_t
ros_action_publish_feedback(term_t ActionServer, term_t Message)
{ rclswi_action_server_t *action_server;
  int rc;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;

  action_send_message_context_t ctx =
  { .msg_type    = &action_server->type_support->feedback,
    .Message     = Message,
  };

  rc = action_prepare_send_message(&ctx);
  TRY(rcl_action_publish_feedback(&action_server->action_server, ctx.msg));
  return action_finish_send_message(&ctx, rc);
}


typedef struct
{ const rclswi_message_type_t	*msg_type;
  void			        *msg;
  term_t			 result;
  term_t			 Message;
} action_take_message_context_t;

static int
action_prepare_take_message(action_take_message_context_t *ctx)
{ if ( !(ctx->result = PL_new_term_ref()) )
    return FALSE;

  if ( (ctx->msg = (*ctx->msg_type->create)()) )
  { (*ctx->msg_type->init)(ctx->msg);
    return TRUE;
  } else
    return PL_resource_error("memory");
}

static int
action_finish_take_message(action_take_message_context_t *ctx, int rc)
{ rc = rc && put_message(ctx->result, ctx->msg, ctx->msg_type->introspection)
	  && PL_unify(ctx->Message, ctx->result);

  (*ctx->msg_type->fini)(ctx->msg);
  (*ctx->msg_type->destroy)(ctx->msg);

  return rc;
}

static foreign_t
ros_action_take_feedback(term_t ActionClient, term_t Message)
{ rclswi_action_client_t *action_client;
  int rc;

  if ( !get_pointer(ActionClient, (void**)&action_client, &action_client_type) )
    return FALSE;

  action_take_message_context_t ctx =
  { .msg_type    = &action_client->type_support->feedback,
    .Message     = Message,
  };

  rc = action_prepare_take_message(&ctx);
  TRY(rcl_action_take_feedback(&action_client->action_client, ctx.msg));
  return action_finish_take_message(&ctx, rc);
}

/* Publish/take action goal status */

/* Message type for action_msgs/msg/GoalStatusArray */
static rclswi_message_type_t *status_type_ptr = NULL;

static foreign_t
set_goal_status_type(term_t MsgType)
{ atom_t symbol;

  if ( !get_pointer_and_symbol(MsgType, (void**)&status_type_ptr, &symbol,
			       &rclswi_message_type_type) )
    return FALSE;

  PL_register_atom(symbol);
  return TRUE;
}

static rclswi_message_type_t *
goal_status_type(void)
{ if ( !status_type_ptr )
  { predicate_t pred = PL_predicate("init_goal_status", 0, "ros_detail_actions");

    if ( !PL_call_predicate(NULL, PL_Q_NODEBUG|PL_Q_PASS_EXCEPTION, pred, 0) )
      return NULL;
  }

  return status_type_ptr;
}

static foreign_t
ros_action_publish_status(term_t ActionServer)
{ rclswi_action_server_t *action_server;
  rcl_action_goal_status_array_t status_message =
    rcl_action_get_zero_initialized_goal_status_array();
  int rc = TRUE;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;

  TRY(rcl_action_get_goal_status_array(&action_server->action_server, &status_message));
  TRY(rcl_action_publish_status(&action_server->action_server, &status_message));

  return rc;
}


static foreign_t
ros_action_take_status(term_t ActionClient, term_t Message)
{ rclswi_action_client_t *action_client;
  rclswi_message_type_t *status_type;
  int rc;

  if ( !get_pointer(ActionClient, (void**)&action_client, &action_client_type) )
    return FALSE;
  if ( !(status_type=goal_status_type()) )
    return FALSE;

  action_take_message_context_t ctx =
  { .msg_type    = status_type,
    .Message     = Message,
  };

  rc = action_prepare_take_message(&ctx);
  TRY(rcl_action_take_status(&action_client->action_client, ctx.msg));
  return action_finish_take_message(&ctx, rc);
}


static foreign_t
ros_action_notify_goal_done(term_t ActionServer)
{ rclswi_action_server_t *action_server;
  int rc = TRUE;

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) )
    return FALSE;

  TRY(rcl_action_notify_goal_done(&action_server->action_server));

  return rc;
}


static int
put_msg_goal_info(term_t Info, const rcl_action_goal_info_t *goal_info)
{ atom_t keys[] = { ATOM_goal_id, ATOM_stamp };
  term_t values = 0;
  int rc;

  rc = ( (values = PL_new_term_refs(2)) &&
	 put_guid(values+0, goal_info->goal_id.uuid) &&
	 put_msg_time(values+1, &goal_info->stamp) &&
	 PL_put_dict(Info, ATOM_goal_info, 2, keys, values)
       );

  if ( values )
    PL_reset_term_refs(values);

  return rc;
}

static foreign_t
ros_action_expire_goals(term_t ActionServer, term_t MaxNumGoals, term_t Expired)
{ rclswi_action_server_t *action_server;
  size_t max_num_goals;
  size_t num_expired;
  int rc = TRUE;
  rcl_action_goal_info_t *expired_goals;
  term_t list = PL_new_term_ref();
  term_t head = PL_new_term_ref();

  if ( !get_pointer(ActionServer, (void**)&action_server, &action_server_type) ||
       !PL_get_size_ex(MaxNumGoals, &max_num_goals) )
    return FALSE;

  if ( !(expired_goals=malloc(sizeof(*expired_goals)*max_num_goals)) )
    return PL_resource_error("memory");

  TRY(rcl_action_expire_goals(&action_server->action_server,
			      expired_goals, max_num_goals, &num_expired));

  if ( rc )
  { PL_put_nil(list);
    for(size_t i=num_expired; --i>=0; )
    { if ( !put_msg_goal_info(head, &expired_goals[i]) ||
	   !PL_cons_list(list, head, list) )
	OUTFAIL;
    }
    rc = PL_unify(list, Expired);
  }

out:
  free(expired_goals);
  return rc;
}

typedef struct
{ rcl_action_goal_handle_t *handle;	 /* RCL Goal handle */
  atom_t		    server;	 /* Server it is associated with */
} rclswi_action_goal_handle_t;



static void
free_goal_handle(void *ptr)
{ rclswi_action_goal_handle_t *goal_handle = ptr;

  if ( goal_handle->handle )
    TRYVOID(rcl_action_goal_handle_fini(goal_handle->handle));
  if ( goal_handle->server )
    PL_unregister_atom(goal_handle->server);
}


/** ros_action_accept_new_goal(+Server, +GoalID, +Stamp, -GoalHandle)
 */

static foreign_t
ros_action_accept_new_goal(term_t ActionServer, term_t GoalID, term_t Stamp,
			   term_t GoalHandle)
{ rclswi_action_server_t *action_server;
  int rc = TRUE;
  rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
  rclswi_action_goal_handle_t *goal_handle = NULL;
  atom_t server_symbol;

  if ( !get_pointer_and_symbol(ActionServer, (void**)&action_server,
			       &server_symbol, &action_server_type) )
    return FALSE;

  if ( !fill_uuid(GoalID, &goal_info.goal_id.uuid) )
    return PL_type_error("UUID", GoalID);
  if ( !fill_time_stamp(Stamp, &goal_info.stamp, TRUE) )
    return FALSE;

  if ( !(goal_handle=malloc(sizeof(*goal_handle))) )
    return PL_resource_error("memory");
  goal_handle->server = server_symbol;
  PL_register_atom(goal_handle->server);

  if ( !(goal_handle->handle = rcl_action_accept_new_goal(&action_server->action_server,
							  &goal_info)) )
  { set_error(0);
    OUTFAIL;
  }

  rc = rc && unify_pointer(GoalHandle, goal_handle, &goal_handle_type);

out:
  if ( !rc && goal_handle )
    free_goal_handle(goal_handle);

  return rc;
}


static enum_decl enum_action_goal_event[] =
{ EN_DECL(GOAL_EVENT_EXECUTE,	  execute),
  EN_DECL(GOAL_EVENT_CANCEL_GOAL, cancel_goal),
  EN_DECL(GOAL_EVENT_SUCCEED,	  succeed),
  EN_DECL(GOAL_EVENT_ABORT,       abort),
  EN_DECL(GOAL_EVENT_CANCELED,    canceled),
  EN_END()
};

static foreign_t
ros_action_update_goal_state(term_t GoalHandle, term_t State)
{ rclswi_action_goal_handle_t *goal_handle;
  rcl_action_goal_event_t event;
  int rc = TRUE;

  if ( !get_pointer(GoalHandle, (void**)&goal_handle, &goal_handle_type) )
    return FALSE;
  if ( !get_enum(State, "action_goal_event", enum_action_goal_event, (int*)&event) )
    return FALSE;

  TRY(rcl_action_update_goal_state(goal_handle->handle, event));

  return rc;
}


static enum_decl enum_action_goal_status[] =
{ EN_DECL(GOAL_STATE_UNKNOWN,	unknown),
  EN_DECL(GOAL_STATE_ACCEPTED,  accepted),
  EN_DECL(GOAL_STATE_EXECUTING, executing),
  EN_DECL(GOAL_STATE_CANCELING, canceling),
  EN_DECL(GOAL_STATE_SUCCEEDED, succeeded),
  EN_DECL(GOAL_STATE_CANCELED,	canceled),
  EN_DECL(GOAL_STATE_ABORTED,   aborted),
  EN_END()
};

static foreign_t
ros_enum_goal_status(term_t AsInt, term_t AsAtom)
{ int val;

  if ( PL_get_integer(AsInt, &val) )
  { return unify_enum(AsAtom, enum_action_goal_status, val);
  } else if ( get_enum(AsAtom, "goal_status", enum_action_goal_status, &val) )
  { return PL_unify_integer(AsInt, val);
  }

  return FALSE;
}


static foreign_t
ros_action_goal_prop(term_t GoalHandle, term_t Prop, term_t Value)
{ rclswi_action_goal_handle_t *goal_handle;
  atom_t prop;
  int rc = TRUE;

  if ( !get_pointer(GoalHandle, (void**)&goal_handle, &goal_handle_type) ||
       !PL_get_atom_ex(Prop, &prop) )
    return FALSE;

  if ( prop == ATOM_goal_info )
  { rcl_action_goal_info_t info = rcl_action_get_zero_initialized_goal_info();
    term_t tmp = PL_new_term_ref();

    TRY(rcl_action_goal_handle_get_info(goal_handle->handle, &info));
    rc = rc && put_msg_goal_info(tmp, &info) &&
	       PL_unify(tmp, Value);
  } else if ( prop == ATOM_server )
  { return PL_unify_atom(Value, goal_handle->server);
  } else if ( prop == ATOM_active )
  { return PL_unify_bool(Value, rcl_action_goal_handle_is_active(goal_handle->handle));
  } else if ( prop == ATOM_status )
  { rcl_action_goal_state_t status;
    TRY(rcl_action_goal_handle_get_status(goal_handle->handle, &status));
    return rc && unify_enum(Value, enum_action_goal_status, status);
  } else
    rc = FALSE;

  return rc;
}


		 /*******************************
		 *	    PARAMETERS		*
		 *******************************/

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Delivers:

 - list of _{node:Name, params:Params}

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#define PUT_VARIANT_ARRAY(type, put) \
	do								   \
	{ term_t tmp;							   \
	  if ( (tmp=PL_new_term_ref()) && PL_put_nil(t) )                  \
	  { for(ssize_t i=variant->type ## _array_value->size; --i >= 0; ) \
	    { if ( !put(tmp, variant->type ## _array_value->values[i]) ||  \
		   !PL_cons_list(t, tmp, t) )				   \
		return FALSE;						   \
	    }                                                              \
	    return TRUE;                                                   \
	  }								   \
	  return FALSE;                                                    \
	} while(0)


static int
put_utf8_string(term_t t, const char *str)
{ return PL_put_chars(t, PL_ATOM|REP_UTF8, (size_t)-1, str);
}

static int
put_param_value(term_t t, const rcl_variant_t *variant)
{ if ( variant->bool_value )
  { return PL_put_bool(t, *variant->bool_value);
  } else if ( variant->integer_value )
  { return PL_put_int64(t, *variant->integer_value);
  } else if ( variant->double_value )
  { return PL_put_float(t, *variant->double_value);
  } else if ( variant->string_value )
  { return put_utf8_string(t, variant->string_value);
  } else if ( variant->byte_array_value )
  { PUT_VARIANT_ARRAY(byte, PL_put_integer);
  } else if ( variant->bool_array_value )
  { PUT_VARIANT_ARRAY(bool, PL_put_bool);
  } else if ( variant->integer_array_value )
  { PUT_VARIANT_ARRAY(integer, PL_put_int64);
  } else if ( variant->double_array_value )
  { PUT_VARIANT_ARRAY(double, PL_put_float);
  } else if ( variant->string_array_value )
  {
#define values data			/* different struct layout */
    PUT_VARIANT_ARRAY(string, put_utf8_string);
#undef values
  }

  assert(0);
  return FALSE;
}


static int
put_params(term_t Params, const rcl_node_params_t *params)
{ atom_t *keys;
  size_t i = 0;
  term_t values;
  int rc = TRUE;

  if ( !(keys=malloc(params->num_params*sizeof(*keys))) )
    return PL_resource_error("memory");
  if ( !(values = PL_new_term_refs(params->num_params)) )
    return FALSE;

  for(; i < params->num_params; i++)
  { if ( !(keys[i] = PL_new_atom_mbchars(REP_UTF8, (size_t)-1,
					 params->parameter_names[i])) ||
	 !put_param_value(values+i, &params->parameter_values[i]) )
      OUTFAIL;
  }
  rc = PL_put_dict(Params, ATOM_parameters, params->num_params, keys, values);

out:
  if ( keys )
  { for(size_t j=0; j < i; j++)
      PL_unregister_atom(keys[j]);
    free(keys);
  }

  return rc;
}



static int
node_parameters_from_rcl_params(term_t ParamsByNode,
				const rcl_params_t *params)
{ term_t Node;

  if ( !(Node=PL_new_term_ref()) )
    return FALSE;

  PL_put_nil(ParamsByNode);
  for(ssize_t i = params->num_nodes; --i >= 0; )
  { atom_t keys[] = { ATOM_node, ATOM_parameters };
    term_t values = PL_new_term_refs(2);

    if ( !(values = PL_new_term_refs(2)) ||
	 !put_utf8_string(values+0, params->node_names[i]) ||
	 !put_params(values+1, &params->params[i]) ||
	 !PL_put_dict(Node, ATOM_node_parameters, 2, keys, values) ||
	 !PL_cons_list(ParamsByNode, Node, ParamsByNode) )
      return FALSE;

    PL_reset_term_refs(values);
  }

  return TRUE;
}


static int
parse_param_overrides(term_t Params, const rcl_arguments_t *args)
{ rcl_params_t *params = NULL;
  int rc = TRUE;

  TRY(rcl_arguments_get_param_overrides(args, &params));
  if ( rc )
  { if ( params )
      rc = node_parameters_from_rcl_params(Params, params);
    else
      rc = PL_put_nil(Params);
  }

  if ( params )
    rcl_yaml_node_struct_fini(params);

  return rc;
}


static foreign_t
ros_get_node_parameters(term_t Node, term_t Which, term_t Params)
{ rclswi_node_t *node;
  int rc = TRUE;
  atom_t which;
  term_t Tmp = PL_new_term_ref();

  if ( !get_pointer(Node, (void**)&node, &node_type) ||
       !PL_get_atom_ex(Which, &which) )
    return FALSE;

  const rcl_node_options_t *node_options = rcl_node_get_options(&node->node);

  if ( which == ATOM_global )
  { if ( node_options->use_global_arguments )
      rc = parse_param_overrides(Tmp, &node->node.context->global_arguments);
    else
      rc = PL_put_nil(Tmp);
  } else if ( which == ATOM_local )
  { rc = parse_param_overrides(Tmp, &node_options->arguments);
  } else
    return PL_domain_error("ros_parameter_scope", Which);

  return rc && PL_unify(Params, Tmp);
}


static enum_decl enum_parameter_type[] =
{ EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_NOT_SET,       not_set),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_BOOL,	       bool),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER,       integer),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE,	       double),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_STRING,	       string),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_BYTE_ARRAY,    byte_array),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_BOOL_ARRAY,    bool_array),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER_ARRAY, integer_array),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE_ARRAY,  double_array),
  EN_DECL(rcl_interfaces__msg__ParameterType__PARAMETER_STRING_ARRAY,  string_array),
  EN_END()
};

static foreign_t
ros_enum_param_type(term_t AsInt, term_t AsAtom)
{ int val;

  if ( PL_get_integer(AsInt, &val) )
  { return unify_enum(AsAtom, enum_parameter_type, val);
  } else if ( get_enum(AsAtom, "parameter_type", enum_parameter_type, &val) )
  { return PL_unify_integer(AsInt, val);
  }

  return FALSE;
}



		 /*******************************
		 *	      TYPES		*
		 *******************************/

static int	put_message(term_t Message, void *msg,
			    const rosidl_message_type_support_t *ts);

static int
rclswi_names_and_types_fini(rcl_names_and_types_t *names_and_types)
{ int rc = TRUE;

  if ( names_and_types )
    TRY(rcl_names_and_types_fini(names_and_types));

  return rc;
}


/**
 * Unify an rcl `rcl_names_and_types_t` struct as a Prolog list of
 * Name-Types, where Types is a list of atoms.
 */

static int
unify_rcl_names_and_types(term_t t, rcl_names_and_types_t *names_and_types)
{ size_t i;
  term_t tail = PL_copy_term_ref(t);
  term_t head = PL_new_term_ref();
  term_t types = PL_new_term_ref();

  for(i = 0; i < names_and_types->names.size; i++)
  { if ( PL_unify_list(tail, head, tail) &&
	 PL_put_variable(types) &&
	 PL_unify_term(head, PL_FUNCTOR, FUNCTOR_minus2,
		               PL_UTF8_CHARS, names_and_types->names.data[i],
			       PL_TERM, types) )
    { size_t j;

      for(j=0; j < names_and_types->types[i].size; j++)
      { if ( !(PL_unify_list(types, head, types) &&
	       PL_unify_chars(head, PL_ATOM|REP_UTF8, (size_t)-1,
			      names_and_types->types[i].data[j])) )
	  return FALSE;
      }
      if ( !PL_unify_nil(types) )
	return FALSE;
    } else
      return FALSE;
  }
  if ( !PL_unify_nil(tail) )
    return FALSE;

  PL_reset_term_refs(tail);

  return TRUE;
}


#ifndef RTLD_DEFAULT
#define RTLD_DEFAULT NULL
#endif

static void *
msg_func(char *name, char *end, const char *postfix)
{ void *ret;

  strcpy(end, postfix);

  if ( !(ret=dlsym(RTLD_DEFAULT, name)) )
    DEBUG(1, Sdprintf("Cannot find symbol '%s'\n", name));

  return ret;
}

static const
void *
introspection_func(const char *name)
{ void *sym = dlsym(RTLD_DEFAULT, name);

  if ( sym )
  { const void*(*func)(void) = sym;

    return (*func)();
  } else
  { DEBUG(1, Sdprintf("Cannot find symbol '%s'\n", name));
  }

  return NULL;
}

static int
msg_type_functions(rclswi_message_type_t *type,
		   const char *prefix,
		   const char *introspection,
		   const char *type_support)
{ char buf[512];
  char *end;

  if ( strlen(prefix) > sizeof(buf)-16 )
    return PL_representation_error("symbol_name_length");

  strcpy(buf, prefix);
  end = buf+strlen(buf);

  if ( !(type->init	     = msg_func(buf, end, "__init"))      ||
       !(type->create        = msg_func(buf, end, "__create"))    ||
       !(type->fini          = msg_func(buf, end, "__fini"))      ||
       !(type->destroy       = msg_func(buf, end, "__destroy"))   ||
       !(type->introspection = introspection_func(introspection)) )
    return FALSE;
  if ( type_support )
  { if ( !(type->type_support  = introspection_func(type_support)) )
      return FALSE;
  } else
  { type->type_support = NULL;
  }

  return TRUE;
}


static foreign_t
ros_message_type(term_t Introspection, term_t TypeSupport,
		 term_t Prefix, term_t Functions)
{ char *prefix;
  char *introspection;
  char *type_support;

  if ( get_utf8_name_ex(Prefix, &prefix) &&
       get_utf8_name_ex(Introspection, &introspection) &&
       get_utf8_name_ex(TypeSupport, &type_support) )
  { rclswi_message_type_t *ret;

    if ( strlen(prefix) > 500 )
      return PL_representation_error("symbol_name_length");

    if ( (ret = malloc(sizeof(*ret))) )
    { if ( !msg_type_functions(ret, prefix, introspection, type_support) )
      { free(ret);
	return FALSE;
      }

      return unify_pointer(Functions, ret, &rclswi_message_type_type);
    }
  }

  return FALSE;
}


static foreign_t
ros_service_type(term_t IntrospectionRequest,
		 term_t IntrospectionResponse,
		 term_t TypeSupport,
		 term_t PrefixRequest,
		 term_t PrefixResponse,
		 term_t Functions)
{ char *prefix_req;
  char *prefix_res;
  char *introsp_req;
  char *introsp_res;
  char *type_support;

  if ( get_utf8_name_ex(PrefixRequest, &prefix_req) &&
       get_utf8_name_ex(PrefixResponse, &prefix_res) &&
       get_utf8_name_ex(IntrospectionRequest, &introsp_req) &&
       get_utf8_name_ex(IntrospectionResponse, &introsp_res) &&
       get_utf8_name_ex(TypeSupport, &type_support) )
  { rclswi_srv_type_t *ret;

    if ( (ret = malloc(sizeof(*ret))) )
    { if ( !(ret->type_support = introspection_func(type_support)) ||
	   !msg_type_functions(&ret->request,  prefix_req, introsp_req, NULL) ||
	   !msg_type_functions(&ret->response, prefix_res, introsp_res, NULL) )
      { free(ret);
	return FALSE;
      }

      return unify_pointer(Functions, ret, &rclswi_srv_type_type);
    }
  }

  return FALSE;
}


static foreign_t
ros_action_type(term_t a0)
{ char *prefix_goal_req;
  char *prefix_goal_res;
  char *prefix_res_req;
  char *prefix_res_res;
  char *prefix_fb;
  char *introsp_goal_req;
  char *introsp_goal_res;
  char *introsp_res_req;
  char *introsp_res_res;
  char *introsp_fb;
  char *type_support;

  term_t ISFuncGoalRequest	   = a0+0;
  term_t ISFuncGoalResponse	   = a0+1;
  term_t ISFuncResultRequest	   = a0+2;
  term_t ISFuncResultResponse	   = a0+3;
  term_t ISFuncFeedback		   = a0+4;
  term_t TSFunc			   = a0+5;
  term_t FuncPostfixGoalRequest	   = a0+6;
  term_t FuncPostfixGoalResponse   = a0+7;
  term_t FuncPostfixResultRequest  = a0+8;
  term_t FuncPostfixResultResponse = a0+9;
  term_t FuncPostfixFeedback	   = a0+10;
  term_t ActFunctions		   = a0+11;

  if ( get_utf8_name_ex(FuncPostfixGoalRequest,	   &prefix_goal_req) &&
       get_utf8_name_ex(FuncPostfixGoalResponse,   &prefix_goal_res) &&
       get_utf8_name_ex(FuncPostfixResultRequest,  &prefix_res_req) &&
       get_utf8_name_ex(FuncPostfixResultResponse, &prefix_res_res) &&
       get_utf8_name_ex(FuncPostfixFeedback,	   &prefix_fb) &&
       get_utf8_name_ex(ISFuncGoalRequest,	   &introsp_goal_req) &&
       get_utf8_name_ex(ISFuncGoalResponse,	   &introsp_goal_res) &&
       get_utf8_name_ex(ISFuncResultRequest,	   &introsp_res_req) &&
       get_utf8_name_ex(ISFuncResultResponse,	   &introsp_res_res) &&
       get_utf8_name_ex(ISFuncFeedback,	           &introsp_fb) &&
       get_utf8_name_ex(TSFunc,			   &type_support) )
  { rclswi_action_type_t *ret;

    if ( (ret = malloc(sizeof(*ret))) )
    { if ( !(ret->type_support = introspection_func(type_support)) ||
	   !msg_type_functions(&ret->goal.request,    prefix_goal_req, introsp_goal_req, NULL) ||
	   !msg_type_functions(&ret->goal.response,   prefix_goal_res, introsp_goal_res, NULL) ||
	   !msg_type_functions(&ret->result.request,  prefix_res_req,  introsp_res_req,  NULL) ||
	   !msg_type_functions(&ret->result.response, prefix_res_res,  introsp_res_res,  NULL) ||
	   !msg_type_functions(&ret->feedback,        prefix_fb,       introsp_fb,       NULL) )
      { free(ret);
	return FALSE;
      }

      return unify_pointer(ActFunctions, ret, &rclswi_action_type_type);
    }
  }

  return FALSE;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
rosidl_typesupport_introspection_c__get_message_type_support_handle__" + package + "__msg__" + name;
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#include <rosidl_typesupport_introspection_c/field_types.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <ctype.h>

typedef struct typesupport_type
{ uint8_t     type_id;
  const char *type_name;
  atom_t      type_atom;
} typesupport_type;

#define TS(name) { rosidl_typesupport_introspection_c__ROS_TYPE_ ## name, #name, 0 }

static typesupport_type types[] = {
  TS(FLOAT),
  TS(DOUBLE),
  TS(LONG_DOUBLE),
  TS(CHAR),
  TS(WCHAR),
  TS(BOOLEAN),
  TS(OCTET),
  TS(UINT8),
  TS(INT8),
  TS(UINT16),
  TS(INT16),
  TS(UINT32),
  TS(INT32),
  TS(UINT64),
  TS(INT64),
  TS(STRING),
  TS(WSTRING),
  TS(MESSAGE),
  { 0, NULL, 0 }
};

#define FAST_NAME_LEN 200

static atom_t
lwr_atom(const char *name)
{ size_t len = strlen(name)*2+1;
  char buf[FAST_NAME_LEN];
  char *lwr = (len > FAST_NAME_LEN ? malloc(len) : buf);

  if ( lwr )
  { const char *i=name;
    char *o = lwr;
    atom_t a;

    while (*i)
    { if ( o > lwr && !isupper(i[0]) && i[1] && isupper(i[1]) )
      { *o++ = i[0];
	*o++ = '_';
	*o++ = tolower(i[1]);
	i += 2;
      } else
      { *o++ = tolower(*i++);
      }
    }
    *o = '\0';

    a = PL_new_atom(lwr);
    if ( lwr != buf )
      free(lwr);

    return a;
  }

  return (atom_t)0;
}


static atom_t
camel_case_atom(const char *name)
{ size_t len = strlen(name)+1;
  char buf[FAST_NAME_LEN];
  char *camel = (len > FAST_NAME_LEN ? malloc(len) : buf);

  if ( camel )
  { const char *i=name;
    char *o = camel;
    atom_t a;

    if ( strncmp(i, "uint", 4) == 0 && isdigit(i[4]) )
    { strcpy(o, "UInt");
      i += 4;
      o += 4;
    }

    while (*i)
    { if ( o == camel )
      { *o++ = toupper(*i++);
      } else if ( *i == '_' )
      { *o++ = toupper(i[1]);
	i += 2;
      } else
      { *o++ = *i++;
      }
    }
    *o = '\0';

    a = PL_new_atom(camel);
    if ( camel != buf )
      free(camel);

    return a;
  }

  return (atom_t)0;
}


static foreign_t
ros_identifier_prolog(term_t Ros, term_t Prolog)
{ char *ros, *prolog;
  atom_t a;
  int rc;

  if ( PL_get_atom_chars(Ros, &ros) )
  { a = lwr_atom(ros);
    rc = PL_unify_atom(Prolog, a);
  } else if ( PL_get_atom_chars(Prolog, &prolog) )
  { a = camel_case_atom(prolog);
    rc = PL_unify_atom(Ros, a);
  } else
    return PL_type_error("atom", PL_is_variable(Ros) ? Prolog : Ros);

  PL_unregister_atom(a);

  return rc;
}



static atom_t
type_atom(uint8_t id)
{ typesupport_type *tst = &types[id-1];

  assert(tst->type_id == id);
  if ( !tst->type_atom )
    tst->type_atom = lwr_atom(tst->type_name);

  return tst->type_atom;
}


static int
put_type(term_t t, const rosidl_message_type_support_t *ts)
{ const rosidl_typesupport_introspection_c__MessageMembers *members = ts->data;
  atom_t tag    = lwr_atom(members->message_name_);
  atom_t *keys  = malloc(sizeof(*keys)*members->member_count_);
  term_t values = PL_new_term_refs(members->member_count_);
  int rc = TRUE;

  if ( !keys )
    return PL_resource_error("memory");

  for(uint32_t i=0; rc && i<members->member_count_; i++)
  { const rosidl_typesupport_introspection_c__MessageMember *mem;

    mem = members->members_+i;

    if ( (keys[i] = lwr_atom(mem->name_)) )
    { atom_t ta;

      if ( mem->type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE )
      { rc = put_type(values+i, mem->members_);
      } else if ( (ta=type_atom(mem->type_id_)) )
      { rc = PL_put_atom(values+i, ta);
      } else
	rc = FALSE;

      if ( rc && mem->is_array_ )
      { if ( mem->is_upper_bound_ || mem->array_size_ == 0 )
	{ rc = PL_cons_functor(values+i, FUNCTOR_list1, values+i);
	} else
	{ term_t nelms;

	  rc = ( (nelms = PL_new_term_ref()) &&
		 PL_put_integer(nelms, mem->array_size_) &&
		 PL_cons_functor(values+i, FUNCTOR_list2, values+i, nelms) );

	  if ( nelms )
	    PL_reset_term_refs(nelms);
	}
      }
    } else
      rc = FALSE;
  }

  rc = rc && PL_put_dict(t, tag, members->member_count_, keys, values);
  PL_reset_term_refs(values);
  free(keys);

  return rc;
}


static foreign_t
ros_type_introspection(term_t TypeBlob, term_t Type)
{ void *ptr;
  atom_t blob;
  const c_pointer_type *type;

  if ( get_pointer_ex(TypeBlob, &ptr, &blob, &type) )
  { term_t tmp = PL_new_term_ref();
    int rc;

    if ( type == &rclswi_message_type_type )
    { rclswi_message_type_t *msg_type = ptr;

      rc = put_type(tmp, msg_type->introspection);
    } else if ( type == &rclswi_srv_type_type )
    { rclswi_srv_type_t *srv_type = ptr;
      atom_t keys[] = {ATOM_request, ATOM_response};
      term_t values = PL_new_term_refs(2);

      rc = ( put_type(values+0, srv_type->request.introspection) &&
	     put_type(values+1, srv_type->response.introspection) &&
	     PL_put_dict(tmp, ATOM_service, 2, keys, values) );
    } else if ( type == &rclswi_action_type_type )
    { rclswi_action_type_t *action_type = ptr;
      atom_t keys[] = { ATOM_goal_request, ATOM_goal_response,
		        ATOM_result_request, ATOM_result_response,
			ATOM_feedback
		      };
      term_t values = PL_new_term_refs(5);

      rc = ( put_type(values+0, action_type->goal.request.introspection) &&
	     put_type(values+1, action_type->goal.response.introspection) &&
	     put_type(values+2, action_type->result.request.introspection) &&
	     put_type(values+3, action_type->result.response.introspection) &&
	     put_type(values+4, action_type->feedback.introspection) &&
	     PL_put_dict(tmp, ATOM_action, 5, keys, values) );
    } else
    { return PL_type_error("ros_type", TypeBlob);
    }

    return rc && PL_unify(Type, tmp);
  } else
    return FALSE;
}


static size_t
sizeof_primitive_type(uint8_t type_id)
{ switch (type_id)
  { case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      return sizeof(float);
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      return sizeof(double);
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      return sizeof(long double);
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
      return sizeof(uint16_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
      return sizeof(bool);
    case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
      return sizeof(int8_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
      return sizeof(uint16_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
      return sizeof(int16_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
      return sizeof(uint32_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
      return sizeof(int32_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
      return sizeof(uint64_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
      return sizeof(int64_t);
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      return sizeof(rosidl_runtime_c__String);
#ifdef HAVE_ROS_WSTRING
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
      return sizeof(rosidl_runtime_c__U16String);
#endif
    case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
      assert(0 && "Cannot get the size of a nested message");
      return 0;
    default:
      assert(0 && "Cannot get the size of an unknown message type");
      return 0;
  }
}

static int
put_primitive(term_t t, void *ptr, uint8_t type_id)
{ switch(type_id)
  { case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
    { float f = *(float*)ptr;
      return PL_put_float(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
    { double f = *(double*)ptr;
      return PL_put_float(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
    { long double f = *(long double*)ptr;
      return PL_put_float(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
    { unsigned char f = *(unsigned char*)ptr;
      return PL_put_atom_nchars(t, 1, (char *)&f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
    { wint_t f = *(wint_t*)ptr;		/* TBD: correct size? */
      wchar_t s[1];
      s[0] = f;
      return PL_unify_wchars(t, PL_ATOM, 1, s);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
    { uint8_t f = *(char*)ptr;		/* TBD: correct size? */
      return PL_put_bool(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
    { uint8_t f = *(uint8_t*)ptr;
      return PL_put_integer(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
    { int8_t f = *(int8_t*)ptr;
      return PL_put_integer(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
    { uint16_t f = *(uint16_t*)ptr;
      return PL_put_integer(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
    { int16_t f = *(int16_t*)ptr;
      return PL_put_integer(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
    { uint32_t f = *(uint32_t*)ptr;
      return PL_put_integer(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
    { int32_t f = *(int32_t*)ptr;
      return PL_put_integer(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
    { uint64_t f = *(uint64_t*)ptr;
      return PL_put_uint64(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
    { int64_t f = *(int64_t*)ptr;
      return PL_put_int64(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
    { rosidl_runtime_c__String *s = ptr;

      return PL_put_chars(t, PL_STRING|REP_UTF8, s->size, s->data);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
      Sdprintf("TBD: Strings support\n");
    default:
      assert(0);
      return FALSE;
  }
}

static int
put_array(term_t List, void *msg, size_t len,
	  const rosidl_typesupport_introspection_c__MessageMember *member)
{ term_t head = PL_new_term_ref();

  PL_put_nil(List);

  if ( member->type_id_ ==
       rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE )
  { const rosidl_message_type_support_t *ts = member->members_;
    const rosidl_typesupport_introspection_c__MessageMembers *members = ts->data;
    size_t elem_size = members->size_of_;

    for(ssize_t i=(ssize_t)len; --i >= 0; )
    { if ( !put_message(head,  (char*)msg+elem_size*i, ts) ||
	   !PL_cons_list(List, head, List) )
	return FALSE;
    }
  } else
  { size_t elem_size = sizeof_primitive_type(member->type_id_);

    for(ssize_t i=(ssize_t)len; --i >= 0; )
    { if ( !put_primitive(head, (char*)msg+elem_size*i, member->type_id_) ||
	   !PL_cons_list(List, head, List) )
	return FALSE;
    }
  }

  PL_reset_term_refs(head);

  return TRUE;
}

static int
put_dynamic_array(term_t Message, void *msg,
		  const rosidl_typesupport_introspection_c__MessageMember *member)
{ rosidl_runtime_c__octet__Sequence *seq = msg;

  return put_array(Message, seq->data, seq->size, member);
}

static int
put_fixed_array(term_t Message, void *msg,
		const rosidl_typesupport_introspection_c__MessageMember *member)
{ return put_array(Message, msg, member->array_size_, member);
}

static int
is_uuid_message(const rosidl_typesupport_introspection_c__MessageMembers *members)
{ if ( members->member_count_ == 1 )
  { const rosidl_typesupport_introspection_c__MessageMember *mem = members->members_;
    return ( mem->is_array_ &&
	     mem->array_size_ == 16 &&
	     mem->type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 &&
	     strcmp(mem->name_, "uuid") == 0 );
  }

  return FALSE;
}

static int
put_uuid_message(term_t Message, void *msg,
		 const rosidl_typesupport_introspection_c__MessageMembers *members)
{ const rosidl_typesupport_introspection_c__MessageMember *mem = members->members_;

  return put_guid(Message, (uint8_t*)msg+mem->offset_);
}

static int
is_time_message(const rosidl_typesupport_introspection_c__MessageMembers *members)
{ if ( members->member_count_ == 2 )
  { const rosidl_typesupport_introspection_c__MessageMember *mem = members->members_;
    return ( mem[0].type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_INT32 &&
	     mem[1].type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 &&
	     strcmp(mem[0].name_, "sec") == 0 &&
	     strcmp(mem[1].name_, "nanosec") == 0 );
  }

  return FALSE;
}


#define FAST_KEYS 32

static int
put_message(term_t Message, void *msg, const rosidl_message_type_support_t *ts)
{ const rosidl_typesupport_introspection_c__MessageMembers *members = ts->data;
  int rc = TRUE;

  if ( is_uuid_message(members) )
  { return put_uuid_message(Message, msg, members);
  } else
  { atom_t keybuf[FAST_KEYS];
    atom_t tag    = lwr_atom(members->message_name_);
    term_t values = PL_new_term_refs(members->member_count_);
    atom_t *keys  = malloc(sizeof(*keys)*members->member_count_);

    if ( members->member_count_ > FAST_KEYS )
    { if ( !(keys=malloc(sizeof(*keys)*members->member_count_)) )
	return PL_resource_error("memory");
    }

    for(uint32_t i=0; rc && i<members->member_count_; i++)
    { const rosidl_typesupport_introspection_c__MessageMember *mem
		= members->members_+i;

      keys[i] = lwr_atom(mem->name_);
      DEBUG(5, Sdprintf("Put %s -> ", mem->name_));
      if ( mem->is_array_ )
      { if ( mem->is_upper_bound_ || mem->array_size_ == 0 )
	  rc = put_dynamic_array(values+i, (char*)msg+mem->offset_, mem);
	else
	  rc = put_fixed_array(values+i, (char*)msg+mem->offset_, mem);
      } else if ( mem->type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE )
      { if ( !put_message(values+i, (char*)msg+mem->offset_, mem->members_) )
	  rc = FALSE;
      } else
      { if ( !put_primitive(values+i, (char*)msg+mem->offset_, mem->type_id_) )
	  rc = FALSE;
      }
      DEBUG(5, PL_write_term(Serror, values+i, 1200, PL_WRT_QUOTED|PL_WRT_NEWLINE));
    }

    rc = rc && PL_put_dict(Message, tag, members->member_count_, keys, values);
    PL_reset_term_refs(values);
    if ( keys != keybuf )
      free(keys);
  }

  return rc;
}


		 /*******************************
		 *	  PROLOG -> ROS		*
		 *******************************/

static int
get_xdigit(int chr, int *val)
{ if      ( chr >= '0' && chr <= '9' ) *val = chr-'0';
  else if ( chr >= 'a' && chr <= 'f' ) *val = chr-'a'+10;
  else if ( chr >= 'A' && chr <= 'F' ) *val = chr-'A'+10;
  else return FALSE;

  return TRUE;
}

static int
get_hex(const char *in, uint8_t *val)
{ int v1, v2;

  if ( get_xdigit(in[0], &v1) &&
       get_xdigit(in[1], &v2) )
  { *val = (v1<<4)+v2;
    return TRUE;
  }

  return FALSE;
}

static int
fill_uuid(term_t t, void *msg)
{ size_t len;
  char *s;

  if ( PL_get_nchars(t, &len, &s, CVT_ATOM|CVT_STRING) && len == 36 &&
       s[8] == '-' && s[13] == '-' && s[18] == '-' && s[23] == '-' )
  { uint8_t *out = msg;

    for(int i=0; i<16; i++)
    { if ( !get_hex(s, out) )
	return FALSE;
      s += 2;
      out++;
      if ( i == 3 || i == 5 || i == 7 || i == 9 )
      {	if ( *s == '-' )
	  s++;
      }
    }

    return TRUE;
  }

  return FALSE;
}

static int
fill_time_stamp(term_t Stamp, builtin_interfaces__msg__Time *tp, int ex)
{ double d;
  int rc;

  rc = ex ? PL_get_float_ex(Stamp, &d) : PL_get_float(Stamp, &d);
  if ( rc )
  { double ip;

    tp->nanosec = (uint32_t)(modf(d, &ip)*1000000000.0);
    tp->sec     = (int32_t)ip;
  }

  return rc;
}


#include <rosidl_runtime_c/primitives_sequence_functions.h>

typedef bool (*seq_init_function)(void *msg, size_t size);

static seq_init_function
primitive_seq_init_function(uint8_t type_id)
{ switch(type_id)
  { case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      return (seq_init_function)rosidl_runtime_c__float32__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      return (seq_init_function)rosidl_runtime_c__double__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      return (seq_init_function)rosidl_runtime_c__long_double__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
      return (seq_init_function)rosidl_runtime_c__char__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
      return (seq_init_function)rosidl_runtime_c__wchar__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
      return (seq_init_function)rosidl_runtime_c__bool__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
      return (seq_init_function)rosidl_runtime_c__octet__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
      return (seq_init_function)rosidl_runtime_c__uint8__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
      return (seq_init_function)rosidl_runtime_c__int8__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
      return (seq_init_function)rosidl_runtime_c__uint16__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
      return (seq_init_function)rosidl_runtime_c__int16__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
      return (seq_init_function)rosidl_runtime_c__uint32__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
      return (seq_init_function)rosidl_runtime_c__int32__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
      return (seq_init_function)rosidl_runtime_c__uint64__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
      return (seq_init_function)rosidl_runtime_c__int64__Sequence__init;
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      return (seq_init_function)rosidl_runtime_c__String__Sequence__init;
#ifdef HAVE_ROS_WSTRING
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
      return (seq_init_function)rosidl_runtime_c__U16String__Sequence__init;
#endif
    default:
      assert(0);
      return FALSE;
  }

}


static int
fill_primitive(term_t t, void *ptr, uint8_t type_id)
{ int rc;

  switch(type_id)
  { case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
    { rc = PL_cvt_i_single(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
    { rc = PL_get_float_ex(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
    { double d;

      if ( (rc=PL_get_float_ex(t, &d)) )
      { long double *p = ptr;
	*p = (long double)d;
      }

      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
    { rc = PL_cvt_i_char(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
    { int c;

      if ( (rc=PL_get_char_ex(t, &c, FALSE)) )
      {	wint_t *p = ptr;		/* TBD: correct size? */
	*p = c;
      }

      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
    { int v;

      if ( (rc=PL_get_bool_ex(t, &v)) )
      { uint8_t *p = ptr;		/* TBD: correct size? */
	*p = !!v;
      }

      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
    { rc = PL_cvt_i_uchar(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
    { rc = PL_cvt_i_char(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
    { rc = PL_cvt_i_ushort(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
    { rc = PL_cvt_i_short(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
    { rc = PL_cvt_i_uint(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
    { rc = PL_cvt_i_int(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
    { rc = PL_cvt_i_uint64(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
    { rc = PL_cvt_i_int64(t, ptr);
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
    { char *s;
      size_t len;

      if ( (rc=PL_get_nchars(t, &len, &s, CVT_ATOMIC|CVT_LIST|CVT_EXCEPTION|REP_UTF8)) )
      { rosidl_runtime_c__String *str = ptr;

	if ( strlen(s) < len )
	  return PL_domain_error("string_without_null", t);

	if ( !rosidl_runtime_c__String__assignn(str, s, len) )
	  rc = PL_resource_error("memory");
      }
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
    default:
      assert(0);
      return FALSE;
  }

  return rc;
}


static int
fill_primitive_array(term_t List, void *array, size_t len, uint8_t type_id)
{ term_t tail = PL_copy_term_ref(List);
  term_t head = PL_new_term_ref();
  size_t elemsize = sizeof_primitive_type(type_id);

  for(size_t i=0; i < len; i++)
  { if ( !PL_get_list(tail, head, tail) ||
	 !fill_primitive(head, (char*)array + elemsize*i, type_id) )
      return FALSE;
  }
  return PL_get_nil(tail);
}


static int
fill_static_struct_array(term_t List, void *array, size_t len,
			 const rosidl_message_type_support_t *ts)
{ const rosidl_typesupport_introspection_c__MessageMembers *members = ts->data;
  term_t tail = PL_copy_term_ref(List);
  term_t head = PL_new_term_ref();
  size_t elemsize = members->size_of_;

  for(size_t i=0; i < len; i++)
  { if ( !PL_get_list(tail, head, tail) ||
	 !fill_message(head, (char*)array + elemsize*i, ts, TRUE) )
      return FALSE;
  }
  return PL_get_nil(tail);
}

static int
fill_dynamic_struct_array(term_t List, void *array, size_t len,
			  const rosidl_typesupport_introspection_c__MessageMember *mem)
{ if ( (*mem->resize_function)(array, len) )
  { term_t tail = PL_copy_term_ref(List);
    term_t head = PL_new_term_ref();

    for(size_t i = 0; i < len; i++)
    { if ( !PL_get_list(tail, head, tail) ||
	   !fill_message(head, mem->get_function(array, i), mem->members_, TRUE) )
      return FALSE;
    }

    return PL_get_nil(tail);
  }

  return PL_resource_error("memmory");
}


static int
list_of_length(term_t t, size_t len, size_t expected, int fixed)
{ Sdprintf("TBD: Non-list length\n");

  if ( fixed )
    return PL_domain_error("fixed_list_length", t);
  else
    return PL_domain_error("bounded_list_length", t);
}


static int
fill_message(term_t Message, void *msg, const rosidl_message_type_support_t *ts,
	     int noarray)
{ const rosidl_typesupport_introspection_c__MessageMembers *members = ts->data;
  term_t Value;
  int rc;
  double time;

  if ( is_uuid_message(members) )
  { const rosidl_typesupport_introspection_c__MessageMember *mem = members->members_;
    if ( (rc=fill_uuid(Message, (uint8_t*)msg+mem->offset_)) ||
	 PL_exception(0) )
      return rc;
  } else if ( is_time_message(members) && PL_get_float(Message, &time) )
  { const rosidl_typesupport_introspection_c__MessageMember *mem = members->members_;
    int32_t  *secp  = (int32_t* )((char*)msg+mem[0].offset_);
    uint32_t *nsecp = (uint32_t*)((char*)msg+mem[1].offset_);
    double ip;

    *nsecp = (uint32_t)(modf(time, &ip)*1000000000.0);
    *secp  = (int32_t)ip;

    return TRUE;
  }

  if ( !(Value = PL_new_term_ref()) )
    return FALSE;
  rc = TRUE;

  for(uint32_t i=0; rc && i<members->member_count_; i++)
  { const rosidl_typesupport_introspection_c__MessageMember *mem
		= members->members_+i;
    atom_t key = lwr_atom(mem->name_);

    if ( PL_get_dict_key(key, Message, Value) )
    { if ( mem->is_array_ && !noarray )
      { size_t len;

	if ( PL_skip_list(Value, 0, &len) != PL_LIST )
	{ rc = PL_type_error("list", Value);
	  OUTFAIL;
	}

	if ( mem->is_upper_bound_ || mem->array_size_ == 0 )	/* Sequence */
	{ if ( mem->is_upper_bound_ && len > mem->array_size_ )
	  { rc = list_of_length(Value, len, mem->array_size_, TRUE);
	    OUTFAIL;
	  }

	  if ( mem->type_id_ ==
	       rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE )
	  { rc = fill_dynamic_struct_array(Value, (char*)msg+mem->offset_, len, mem);
	  } else
	  { rosidl_runtime_c__octet__Sequence *seq = (void*)(char*)msg+mem->offset_;
	    seq_init_function seq_init = primitive_seq_init_function(mem->type_id_);

	    rc = ( (*seq_init)(seq, len) &&
		   fill_primitive_array(Value, seq->data, len, mem->type_id_) );
	  }
	} else
	{ if ( len == mem->array_size_ )
	  { if ( mem->type_id_ ==
	       rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE )
	      rc = fill_static_struct_array(Value, (char*)msg+mem->offset_,
					    mem->array_size_, mem->members_);
	    else
	      rc = fill_primitive_array(Value, (char*)msg+mem->offset_,
					mem->array_size_, mem->type_id_);
	  } else
	  { rc = list_of_length(Value, len, mem->array_size_, TRUE);
	  }
	}
      } else if ( mem->type_id_ ==
		  rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE )
      { if ( !fill_message(Value, (char*)msg+mem->offset_, mem->members_, FALSE) )
	  OUTFAIL;
      } else
      { if ( !fill_primitive(Value, (char*)msg+mem->offset_, mem->type_id_) )
	  OUTFAIL;
      }
    }
  }

out:
  PL_reset_term_refs(Value);

  return rc;
}


		 /*******************************
		 *	       WAIT		*
		 *******************************/

typedef struct wait_obj
{ c_pointer_type *ctype;
  union
  { rclswi_subscription_t  *swi_subscription;
    rcl_guard_condition_t  *guard_condition;
    rcl_timer_t            *timer;
    rclswi_client_t        *client;
    rclswi_service_t	   *service;
    rclswi_action_client_t *action_client;
    rclswi_action_server_t *action_server;
    rcl_event_t		   *events;
    void                   *ptr;
  } obj;				/* The pointer */
  size_t index;
  atom_t symbol;			/* Prolog blob handle */
} wait_obj;


static int
unify_action_client_ready(term_t head, atom_t blob,
			  bool is_feedback_ready,
			  bool is_status_ready,
			  bool is_goal_response_ready,
			  bool is_cancel_response_ready,
			  bool is_result_response_ready)
{ return PL_unify_term(head,
		       PL_FUNCTOR, FUNCTOR_action_client6,
		         PL_ATOM, blob,
		         PL_BOOL, is_feedback_ready,
		         PL_BOOL, is_status_ready,
		         PL_BOOL, is_goal_response_ready,
		         PL_BOOL, is_cancel_response_ready,
		         PL_BOOL, is_result_response_ready);
}


static int
unify_action_server_ready(term_t head, atom_t blob,
			  bool is_goal_request_ready,
			  bool is_cancel_request_ready,
			  bool is_result_request_ready,
			  bool is_goal_expired)
{ return PL_unify_term(head,
		       PL_FUNCTOR, FUNCTOR_action_server5,
		         PL_ATOM, blob,
		         PL_BOOL, is_goal_request_ready,
		         PL_BOOL, is_cancel_request_ready,
		         PL_BOOL, is_result_request_ready,
		         PL_BOOL, is_goal_expired);
}



static rcl_ret_t
rclswi_wait(rcl_wait_set_t *wset, int64_t tmo, wait_obj *objs, size_t len)
{ int rc = TRUE;
  rcl_ret_t ret;
  wait_obj *obj;
  size_t i;

  TRY_SILENT(rcl_wait_set_clear(wset));
  for(i=0, obj=objs; i<len && rc; i++, obj++)
  { if ( obj->ctype == &subscription_type )
    { rclswi_subscription_t *sub = obj->obj.swi_subscription;
      if ( !sub->deleted )
      { sub->waiting = TRUE;
	TRY_SILENT(rcl_wait_set_add_subscription(wset, &sub->subscription, &obj->index));
      }
    } else if ( obj->ctype == &client_type )
    { rclswi_client_t *client = obj->obj.client;
      if ( !client->deleted )
      { client->waiting = TRUE;
	TRY_SILENT(rcl_wait_set_add_client(wset, &client->client, &obj->index));
      }
    } else if ( obj->ctype == &service_type )
    { rclswi_service_t *service = obj->obj.service;
      if ( !service->deleted )
      { service->waiting = TRUE;
	TRY_SILENT(rcl_wait_set_add_service(wset, &service->service, &obj->index));
      }
    } else if ( obj->ctype == &action_client_type )
    { rclswi_action_client_t *client = obj->obj.action_client;
      if ( !client->deleted )
      { client->waiting = TRUE;
	TRY_SILENT(rcl_action_wait_set_add_action_client(
		       wset, &client->action_client, NULL, NULL));
      }
    } else if ( obj->ctype == &action_server_type )
    { rclswi_action_server_t *server = obj->obj.action_server;
      if ( !server->deleted )
      { server->waiting = TRUE;
	TRY_SILENT(rcl_action_wait_set_add_action_server(
		       wset, &server->action_server, NULL));
      }
    } else
      assert(0);
  }

  if ( rc )
    ret = rcl_wait(wset, tmo);
  else
    assert(0);				/* TBD: pass Prolog exception */

  for(i=0, obj=objs; i<len && rc; i++, obj++)
  { if ( obj->ctype == &subscription_type )
    { rclswi_subscription_t *sub = obj->obj.swi_subscription;
      sub->waiting = FALSE;
      if ( sub->deleted )
	TRY_ANYWAY(rcl_subscription_fini(&sub->subscription, sub->node));
    } else if ( obj->ctype == &client_type )
    { rclswi_client_t *client = obj->obj.client;
      client->waiting = FALSE;
      if ( client->deleted )
	TRY_ANYWAY(rcl_client_fini(&client->client, client->node));
    } else if ( obj->ctype == &service_type )
    { rclswi_service_t *service = obj->obj.service;
      service->waiting = FALSE;
      if ( service->deleted )
	TRY_ANYWAY(rcl_service_fini(&service->service, service->node));
    } else if ( obj->ctype == &action_client_type )
    { rclswi_action_client_t *client = obj->obj.action_client;
      client->waiting = FALSE;
      if ( client->deleted )
	TRY_ANYWAY(rcl_action_client_fini(&client->action_client, client->node));
    } else if ( obj->ctype == &action_server_type )
    { rclswi_action_server_t *server = obj->obj.action_server;
      server->waiting = FALSE;
      if ( server->deleted )
	TRY_ANYWAY(rcl_action_server_fini(&server->action_server, server->node));
    } else
      assert(0);
  }

  return ret;
}


static int
get_timeout_nsec(term_t Timeout, int64_t *tmo)
{ double tmo_sec;

  if ( PL_get_float(Timeout, &tmo_sec) )
  { *tmo = (int64_t)(tmo_sec*1000000000.0);
    return TRUE;
  } else
  { atom_t inf;

    if ( PL_get_atom(Timeout, &inf) &&
	 (inf == ATOM_inf || inf == ATOM_infinite) )
    { *tmo = -1;
      return TRUE;
    } else
    { return PL_type_error("float", Timeout);
    }
  }
}


#define WAIT_POLL 250000000

static foreign_t
ros_wait(term_t For, term_t Timeout, term_t Ready)
{ int64_t tmo_nsec;
  term_t tail = PL_copy_term_ref(For);
  term_t head = PL_new_term_ref();
  int rc = TRUE;
  size_t len;
  wait_obj *objs = NULL;
  wait_obj *obj;
  int i;
  size_t nsubs=0, nguards=0, ntimers=0, nclients=0, nservices=0, nevents=0;

  if ( !get_timeout_nsec(Timeout, &tmo_nsec) )
    return FALSE;
  if ( PL_skip_list(For, 0, &len) != PL_LIST )
    return PL_type_error("list", For);
  if ( !(objs = malloc(sizeof(*objs)*len)) )
    return PL_resource_error("memory");

  for(i=0; PL_get_list_ex(tail, head, tail); i++)
  { if ( !get_pointer_ex(head, &objs[i].obj.ptr, &objs[i].symbol,
			 (const c_pointer_type**)&objs[i].ctype) )
      OUTFAIL;

    if ( objs[i].ctype == &subscription_type )
      nsubs++;
    else if ( objs[i].ctype == &client_type )
      nclients++;
    else if ( objs[i].ctype == &service_type )
      nservices++;
    else if ( objs[i].ctype == &action_client_type )
    { nclients += 3; nsubs += 2;	/* needed? */
    } else if ( objs[i].ctype == &action_server_type )
    { nservices += 3; ntimers += 1;
    } else
      assert(0);
  }
  if ( !PL_get_nil_ex(tail) )
    OUTFAIL;

  rcl_wait_set_t wset = rcl_get_zero_initialized_wait_set();
  DEBUG(3, Sdprintf("rcl_wait(): %d subscriptions; "
				"%d clients; "
				"%d services\n",
		    nsubs, nclients, nservices));
  TRY(rcl_wait_set_init(&wset,
			nsubs, nguards, ntimers, nclients, nservices, nevents,
			rclswi_default_context(), rcl_get_default_allocator()));
  int wset_initted = TRUE;

  rcl_ret_t ret = RCL_RET_TIMEOUT;
  while ( (tmo_nsec == -1 || tmo_nsec > WAIT_POLL) && ret == RCL_RET_TIMEOUT )
  { if ( tmo_nsec > WAIT_POLL )
      tmo_nsec -= WAIT_POLL;
    ret = rclswi_wait(&wset, WAIT_POLL, objs, len);
    if ( PL_handle_signals() < 0 )
      OUTFAIL;
  }
  if ( ret == RCL_RET_TIMEOUT )
  { ret = rclswi_wait(&wset, tmo_nsec, objs, len);
  }
  DEBUG(3, Sdprintf("rcl_wait() -> %d\n", ret));
  if ( ret == RCL_RET_TIMEOUT )
  { rc = FALSE;
  } else if ( ret == RCL_RET_WAIT_SET_EMPTY )
  { rc = PL_unify_nil_ex(Ready);
    goto out;
  } else
  { TRY_SILENT(ret);
  }

  tail = PL_copy_term_ref(Ready);
  for(i=0, obj=objs; i<len; i++, obj++)
  { int ready = FALSE;

    if ( objs->ctype == &subscription_type )
      ready = !!wset.subscriptions[obj->index];
    else if ( obj->ctype == &client_type )
      ready = !!wset.clients[obj->index];
    else if ( obj->ctype == &service_type )
      ready = !!wset.services[obj->index];
    else if ( obj->ctype == &action_client_type )
    { bool is_feedback_ready = false;
      bool is_status_ready = false;
      bool is_goal_response_ready = false;
      bool is_cancel_response_ready = false;
      bool is_result_response_ready = false;

      TRY(rcl_action_client_wait_set_get_entities_ready(
	      &wset, &obj->obj.action_client->action_client,
	      &is_feedback_ready,
	      &is_status_ready,
	      &is_goal_response_ready,
	      &is_cancel_response_ready,
	      &is_result_response_ready));

      DEBUG(3, Sdprintf("action client ready:%s%s%s%s%s\n",
			is_feedback_ready ? " feedback" : "",
			is_status_ready ? " status" : "",
			is_goal_response_ready ? " goal" : "",
			is_cancel_response_ready ? " cancel" : "",
			is_result_response_ready ? " result" : ""));

      if ( is_feedback_ready ||
	   is_status_ready ||
	   is_goal_response_ready ||
	   is_cancel_response_ready ||
	   is_result_response_ready )
      { if ( !PL_unify_list_ex(tail, head, tail) ||
	     !unify_action_client_ready(head, objs[i].symbol,
					is_feedback_ready,
					is_status_ready,
					is_goal_response_ready,
					is_cancel_response_ready,
					is_result_response_ready) )
	OUTFAIL;
      }

      continue;
    } else if ( obj->ctype == &action_server_type )
    { bool is_goal_request_ready = false;
      bool is_cancel_request_ready = false;
      bool is_result_request_ready = false;
      bool is_goal_expired = false;

      TRY(rcl_action_server_wait_set_get_entities_ready(
	      &wset, &obj->obj.action_server->action_server,
	      &is_goal_request_ready,
	      &is_cancel_request_ready,
	      &is_result_request_ready,
	      &is_goal_expired));

      if ( is_goal_request_ready ||
	   is_cancel_request_ready ||
	   is_result_request_ready ||
	   is_goal_expired )
      { if ( !PL_unify_list_ex(tail, head, tail) ||
	     !unify_action_server_ready(head, objs[i].symbol,
					is_goal_request_ready,
					is_cancel_request_ready,
					is_result_request_ready,
				        is_goal_expired) )
	OUTFAIL;
      }

      continue;
    }

    if ( ready )
    { if ( !PL_unify_list_ex(tail, head, tail) ||
	   !PL_unify_atom(head, objs[i].symbol) )
	OUTFAIL;
    }
  }
  if ( !PL_unify_nil_ex(tail) )
    OUTFAIL;

out:
  if ( wset_initted )
    TRY_ANYWAY(rcl_wait_set_fini(&wset));

  free(objs);

  return rc;
}


		 /*******************************
		 *    QUERY ROS NODE GRAPH	*
		 *******************************/

static foreign_t
ros_topic_names_and_types(term_t Node, term_t NamesAndTypes)
{ rclswi_node_t *node;
  int rc = TRUE;

  if ( !get_pointer(Node, (void**)&node, &node_type) )
    return FALSE;

  rcl_names_and_types_t nat = rcl_get_zero_initialized_names_and_types();
  TRY(rcl_get_topic_names_and_types(&node->node,
				    &rclswi_default_allocator, FALSE, &nat));

  if ( rc && !unify_rcl_names_and_types(NamesAndTypes, &nat) )
    OUTFAIL;

out:
  rc = rclswi_names_and_types_fini(&nat) && rc;
  return rc;
}

static foreign_t
ros_client_names_and_types(term_t Node,
			   term_t NodeName, term_t NameSpace,
			   term_t NamesAndTypes)
{ rclswi_node_t *node;
  char *node_name;
  char *namespace;
  rcl_names_and_types_t nat = rcl_get_zero_initialized_names_and_types();
  int rc = TRUE;

  if ( !get_pointer(Node, (void**)&node, &node_type) ||
       !get_utf8_name_ex(NodeName, &node_name) ||
       !get_utf8_name_ex(NameSpace, &namespace) )
    return FALSE;

  TRY(rcl_get_client_names_and_types_by_node(
	  &node->node, &rclswi_default_allocator, node_name, namespace,
	  &nat));

  if ( rc && !unify_rcl_names_and_types(NamesAndTypes, &nat) )
    OUTFAIL;

out:
  rc = rclswi_names_and_types_fini(&nat) && rc;
  return rc;
}

static foreign_t
ros_service_names_and_types(term_t Node,
			    term_t NodeName, term_t NameSpace,
			    term_t NamesAndTypes)
{ rclswi_node_t *node;
  char *node_name;
  char *namespace;
  rcl_names_and_types_t nat = rcl_get_zero_initialized_names_and_types();
  int rc = TRUE;

  if ( !get_pointer(Node, (void**)&node, &node_type) ||
       !get_utf8_name_ex(NodeName, &node_name) ||
       !get_utf8_name_ex(NameSpace, &namespace) )
    return FALSE;

  TRY(rcl_get_service_names_and_types_by_node(
	  &node->node, &rclswi_default_allocator, node_name, namespace,
	  &nat));

  if ( rc && !unify_rcl_names_and_types(NamesAndTypes, &nat) )
    OUTFAIL;

out:
  rc = rclswi_names_and_types_fini(&nat) && rc;
  return rc;
}






		 /*******************************
		 *	     META INFO		*
		 *******************************/

static foreign_t
ros_rwm_implementation(term_t Impl)
{ const char *impl = rmw_get_implementation_identifier();

  return PL_unify_chars(Impl, PL_ATOM|REP_UTF8, (size_t)-1, impl);
}

		 /*******************************
		 *	       UTIL		*
		 *******************************/

void *
rcl_alloc(size_t bytes, rcl_allocator_t *allocator)
{ void *ptr;

  if ( !allocator )
    allocator = &rclswi_default_allocator;

  if ( (ptr=allocator->allocate(bytes, allocator->state)) )
    return ptr;

  return PL_resource_error("memory"),NULL;
}

void
rcl_free(void *ptr, rcl_allocator_t *allocator)
{ if ( !allocator )
    allocator = &rclswi_default_allocator;

  allocator->deallocate(ptr, allocator->state);
}

int
get_utf8_name_ex(term_t t, char **name)
{ return PL_get_chars(t, name, CVT_ATOMIC|CVT_EXCEPTION|REP_UTF8);
}



		 /*******************************
		 *	      REGISTER		*
		 *******************************/

extern void	install_ros_logging(void);

install_t
install_librclswi(void)
{ MKFUNCTOR(error, 2);
  MKFUNCTOR(ros_error, 2);
  MKFUNCTOR(list, 1);
  MKFUNCTOR(list, 2);
  MKFUNCTOR(action_client, 6);
  MKFUNCTOR(action_server, 5);
  FUNCTOR_minus2 = PL_new_functor(PL_new_atom("-"), 2);

  MKATOM(action);
  MKATOM(ros_args);
  MKATOM(clock);
  MKATOM(context);
  MKATOM(feedback);
  MKATOM(global);
  MKATOM(goal);
  MKATOM(goal_id);
  MKATOM(goal_info);
  MKATOM(goal_request);
  MKATOM(goal_response);
  MKATOM(inf);
  MKATOM(infinite);
  MKATOM(local);
  MKATOM(logger_name);
  MKATOM(name);
  MKATOM(namespace);
  MKATOM(node);
  MKATOM(node_parameters);
  MKATOM(parameters);
  MKATOM(received_timestamp);
  MKATOM(request);
  MKATOM(request_id);
  MKATOM(response);
  MKATOM(result);
  MKATOM(result_request);
  MKATOM(result_response);
  MKATOM(ros);
  MKATOM(rosout);
  MKATOM(sequence_number);
  MKATOM(service);
  MKATOM(service_info);
  MKATOM(source_timestamp);
  MKATOM(stamp);
  MKATOM(steady);
  MKATOM(system);
  MKATOM(type);
  MKATOM(writer_guid);
  MKATOM(active);
  MKATOM(status);
  MKATOM(server);
  MKATOM(cancel);

  rclswi_default_allocator = rcl_get_default_allocator();
#define PRED(name, argc, func, flags) PL_register_foreign(name, argc, func, flags)

  PRED("ros_debug",		     1,	ros_debug,		    0);
  PRED("$ros_object_type",	     2, ros_object_type,	    0);

  PRED("ros_create_context",	     1,	ros_create_context,	    0);
  PRED("ros_init",		     3,	ros_init,		    0);
  PRED("$ros_shutdown",		     1,	ros_shutdown,		    0);
  PRED("ros_ok",		     1,	ros_ok,			    0);
  PRED("$ros_create_node",	     4,	ros_create_node,	    0);
  PRED("ros_node_fini",		     1,	ros_node_fini,		    0);
  PRED("$ros_node_prop",	     3,	ros_node_prop,		    0);
  PRED("$ros_publisher",	     5,	ros_publisher,		    0);
  PRED("$ros_subscribe",	     5,	ros_subscribe,		    0);
  PRED("$ros_unsubscribe",	     1,	ros_unsubscribe,	    0);
  PRED("$ros_create_client",	     5,	ros_create_client,	    0);
  PRED("$ros_create_service",	     5,	ros_create_service,	    0);
  PRED("$ros_create_action_client",  5,	ros_create_action_client,   0);
  PRED("$ros_create_action_server",  8,	ros_create_action_server,   0);

  PRED("ros_create_clock",	     3, ros_create_clock,	    0);
  PRED("ros_clock_time",	     2, ros_clock_time,		    0);

  PRED("$ros_message_type",	     4,	ros_message_type,	    0);
  PRED("$ros_service_type",	     6,	ros_service_type,	    0);
  PRED("$ros_action_type",	     12,ros_action_type, PL_FA_VARARGS);
  PRED("$ros_type_introspection",    2,	ros_type_introspection,	    0);
  PRED("$ros_service_prop",	     3,	ros_service_prop,	    0);
  PRED("$ros_action_client_prop",    3,	ros_action_client_prop,	    0);
  PRED("$ros_action_server_prop",    3,	ros_action_server_prop,	    0);
  PRED("$ros_action_goal_prop",      3,	ros_action_goal_prop,	    0);
  PRED("$ros_clock_prop",            3,	ros_clock_prop,	            0);

  PRED("ros_wait",		     3,	ros_wait,		    0);
  PRED("ros_service_is_ready",	     1,	ros_service_is_ready,	    0);

  PRED("ros_take",		     3,	ros_take,		    0);
  PRED("ros_take_request",	     3,	ros_take_request,	    0);
  PRED("ros_take_response",	     3,	ros_take_response,	    0);
  PRED("$ros_publish",		     2,	ros_publish,		    0);
  PRED("ros_send_request",	     3,	ros_send_request,	    0);
  PRED("ros_send_response",	     3,	ros_send_response,	    0);
  PRED("ros_action_send_goal_request", 3, ros_action_send_goal_request, 0);
  PRED("ros_action_send_result_request", 3, ros_action_send_result_request, 0);
  PRED("ros_action_send_cancel_request", 3, ros_action_send_cancel_request, 0);
  PRED("ros_action_take_goal_request", 3, ros_action_take_goal_request, 0);
  PRED("ros_action_take_result_request", 3, ros_action_take_result_request, 0);
  PRED("ros_action_take_cancel_request", 3, ros_action_take_cancel_request, 0);
  PRED("ros_action_send_goal_response", 3, ros_action_send_goal_response, 0);
  PRED("ros_action_send_result_response", 3, ros_action_send_result_response, 0);
  PRED("ros_action_send_cancel_response", 3, ros_action_send_cancel_response, 0);
  PRED("ros_action_take_goal_response", 3, ros_action_take_goal_response, 0);
  PRED("ros_action_take_result_response", 3, ros_action_take_result_response, 0);
  PRED("ros_action_take_cancel_response", 3, ros_action_take_cancel_response, 0);
  PRED("ros_action_publish_feedback", 2, ros_action_publish_feedback, 0);
  PRED("ros_action_take_feedback", 2, ros_action_take_feedback, 0);
  PRED("ros_action_publish_status", 1, ros_action_publish_status, 0);
  PRED("ros_action_take_status", 2, ros_action_take_status, 0);
  PRED("ros_action_notify_goal_done", 1, ros_action_notify_goal_done, 0);
  PRED("ros_action_accept_new_goal", 4, ros_action_accept_new_goal, 0);
  PRED("ros_action_update_goal_state", 2, ros_action_update_goal_state, 0);
  PRED("ros_action_expire_goals", 3, ros_action_expire_goals, 0);

  PRED("set_action_cancel_type",      1, set_action_cancel_type,      0);
  PRED("set_goal_status_type",	      1, set_goal_status_type,	      0);

  PRED("ros_enum_param_type",	      2, ros_enum_param_type,	      0);
  PRED("ros_enum_goal_status",	      2, ros_enum_goal_status,	      0);
  PRED("ros_get_node_parameters",     3, ros_get_node_parameters,     0);

  PRED("ros_rwm_implementation",      1, ros_rwm_implementation,      0);

  PRED("ros_identifier_prolog",	      2, ros_identifier_prolog,	      0);

  PRED("ros_service_info_to_prolog",  2, ros_service_info_to_prolog,  0);
  PRED("ros_topic_names_and_types",   2, ros_topic_names_and_types,   0);
  PRED("ros_client_names_and_types",  4, ros_client_names_and_types,  0);
  PRED("ros_service_names_and_types", 4, ros_service_names_and_types, 0);

					/* install helpers */
  install_ros_logging();
  install_ros_qos();
}
