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

#include "rclswi.h"

static rcl_allocator_t default_allocator;
static rcl_context_t  *default_context = NULL;

static void *	rcl_alloc(size_t bytes, rcl_allocator_t *allocator);
static void	rcl_free(void *ptr, rcl_allocator_t *allocator);
static int	get_utf8_name_ex(term_t t, char **name);
static int	put_message(term_t Message, void *msg,
			    const rosidl_message_type_support_t *ts);

static atom_t ATOM_cli_args;
static atom_t ATOM_inf;
static atom_t ATOM_infinite;
static atom_t ATOM_name;
static atom_t ATOM_namespace;

static functor_t FUNCTOR_error2;
static functor_t FUNCTOR_ros_error2;
static functor_t FUNCTOR_minus2;
static functor_t FUNCTOR_list1;
static functor_t FUNCTOR_list2;


		 /*******************************
		 *     ENCAPSULATED POINTERS	*
		 *******************************/

static void free_rcl_context(void*);
static void free_rcl_node(void*);
static void free_rcl_publisher(void*);
static void free_rcl_subscription(void*);

static const c_pointer_type context_type =
{ "rcl_context_t",
  free_rcl_context
};

static const c_pointer_type node_type =
{ "rcl_node_t",
  free_rcl_node
};

static const c_pointer_type publisher_type =
{ "rcl_publisher_t",
  free_rcl_publisher
};

static const c_pointer_type subscription_type =
{ "rcl_subscription_t",
  free_rcl_subscription
};

static const c_pointer_type rclswi_message_type_type =
{ "rclswi_message_type_t",
  NULL
};


		 /*******************************
		 *	      DEBUG		*
		 *******************************/

static int debuglevel = 0;

#define DEBUG(level, goal) \
	do				\
	{ if ( debuglevel >= level )	\
	    goal;			\
	} while(0)

static foreign_t
ros_debug(term_t level)
{ return PL_get_integer_ex(level, &debuglevel);
}



		 /*******************************
		 *	      ERRORS		*
		 *******************************/

static int
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

static void
print_error(rcl_ret_t ret, const char *file, int line, const char *goal)
{ Sdprintf("% ERROR: [rclswi %s:%d] %s:\n"
	   "% ERROR: %s\n",
	   file, line, goal, rcl_get_error_string().str);
  rcl_reset_error();
}

#define TRY(goal) \
	do				\
	{ if ( rc )			\
	  { DEBUG(9, Sdprintf("Running %s -> ", #goal)); \
	    rcl_ret_t __ret = (goal);	\
	    DEBUG(9, Sdprintf("%d\n", __ret)); \
	    if ( __ret != RCL_RET_OK )	\
	      rc = set_error(__ret);	\
	  }				\
	} while(0)

#define TRY_ANYWAY(goal) \
	do				\
	{ DEBUG(9, Sdprintf("Running (rc=%d) %s -> ", rc, #goal)); \
	  rcl_ret_t __ret = (goal);	\
          DEBUG(9, Sdprintf("%d\n", __ret)); \
	  if ( __ret != RCL_RET_OK )	\
	    rc = set_error(__ret);	\
	} while(0)

#define OUTFAIL \
	do				\
	{ rc = FALSE;			\
	  goto out;			\
	} while(0)

#define TRYVOID(goal) \
	do				\
	{ rcl_ret_t __ret = (goal);	\
	  if ( __ret != RCL_RET_OK )	\
	    print_error(__ret, __FILE__, __LINE__, #goal); \
	} while(0)


		 /*******************************
		 *	      CONTEXT		*
		 *******************************/

static rcl_context_t *default_context;

static foreign_t
ros_create_context(term_t t)
{ rcl_context_t *context;

  if ( (context=malloc(sizeof(*context))) )
  { *context = rcl_get_zero_initialized_context();
    if ( !default_context )
      default_context = context;
    return unify_pointer(t, context, &context_type);
  }

  return PL_resource_error("memory");
}


static void
free_rcl_context(void *ptr)
{ rcl_context_t *context = ptr;

  if ( ptr == default_context )
    default_context = NULL;

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
    const char **arg_values = NULL;
    term_t tail = PL_copy_term_ref(Args);
    term_t head = PL_new_term_ref();
    char **argp = (char**)arg_values;
    int rc = TRUE;

    if ( num_args > 0 &&
	 !(arg_values=rcl_alloc(sizeof(*arg_values)*num_args, NULL)) )
      return FALSE;

    for( ; PL_get_list_ex(tail, head, tail); argp++)
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
    TRY(rcl_init(num_args, arg_values, &init_options, context));

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

  *num_args = len;
  *arg_values = NULL;

  if ( len > 0 )
  { term_t tail = PL_copy_term_ref(list);
    term_t head = PL_new_term_ref();
    char **argp;

    if ( !(*arg_values = rcl_alloc(sizeof(*arg_values)*len, allocator)) )
      return FALSE;
    argp = *arg_values;

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

  if ( !rclswi_get_arglist(list, &num_args, &arg_values, &default_allocator) )
    return FALSE;

  TRY(rcl_parse_arguments(num_args, (const char**)arg_values, default_allocator, parsed_args));
  if ( (unparsed=rcl_arguments_get_count_unparsed_ros(parsed_args)) > 0 )
  { Sdprintf("TBD: Unparsed arguments\n");
  }

  free_rcl_arglist(arg_values, num_args, &default_allocator);
  return rc;
}


		 /*******************************
		 *	       NODE		*
		 *******************************/

static void
free_rcl_node(void* ptr)
{ rcl_node_t *node = ptr;

  TRYVOID(rcl_node_fini(node));
}


static foreign_t
ros_create_node(term_t Context, term_t Name, term_t Node, term_t Options)
{ rcl_context_t *context;
  rcl_node_t *node;
  rcl_node_options_t options = rcl_node_get_default_options();
  rcl_arguments_t arguments = rcl_get_zero_initialized_arguments();
  int use_global_arguments = TRUE;
  int local_arguments = FALSE;
  int enable_rosout = FALSE;
  char *node_name;
  char *namespace = "";
  int rc = TRUE;
  term_t tail = PL_copy_term_ref(Options);
  term_t head = PL_new_term_ref();
  term_t arg = PL_new_term_ref();

  if ( !PL_get_chars(Name, &node_name, CVT_ATOMIC|CVT_EXCEPTION|REP_UTF8) )
    return FALSE;
  if ( !get_pointer(Context, (void**)&context, &context_type) )
    return FALSE;

  while(PL_get_list_ex(tail, head, tail))
  { atom_t name;
    size_t arity;

    if ( PL_get_name_arity(head, &name, &arity) && arity == 1)
    { _PL_get_arg(1, head, arg);

      if ( name == ATOM_cli_args )
      { if ( !rclswi_parse_args(arg, &arguments) )
	  OUTFAIL;
	local_arguments = TRUE;
      }
    } else
    { return PL_type_error("option", head);
    }
  }
  if ( !PL_get_nil_ex(tail) )
    return FALSE;

  if ( !(node=malloc(sizeof(*node))) )
    return PL_resource_error("memory");
  *node = rcl_get_zero_initialized_node();
  options.use_global_arguments = use_global_arguments;
  options.arguments = arguments;
  options.enable_rosout = enable_rosout;

  TRY(rcl_node_init(node, node_name, namespace, context, &options));

out:
  if ( local_arguments )
    TRY_ANYWAY(rcl_arguments_fini(&arguments));

  if ( rc )
    return unify_pointer(Node, node, &node_type);

  return rc;
}


static foreign_t
ros_node_fini(term_t Node)
{ rcl_node_t *node;
  int rc = TRUE;

  if ( !get_pointer(Node, (void**)&node, &node_type) )
    return FALSE;

  if ( rcl_node_is_valid(node) )
    TRY(rcl_node_fini(node));
  else
    rc = PL_existence_error("ros_node", Node);

  return rc;
}


static foreign_t
ros_node_prop(term_t Node, term_t Prop, term_t Value)
{ rcl_node_t *node;
  atom_t prop;

  if ( !get_pointer(Node, (void**)&node, &node_type) ||
       !PL_get_atom_ex(Prop, &prop) )
    return FALSE;

  if ( prop == ATOM_name )
  { const char *node_name = rcl_node_get_name(node);
    if ( node_name )
      return PL_unify_chars(Value, PL_ATOM|REP_UTF8, (size_t)-1, node_name);
  } else if ( prop == ATOM_namespace )
  { const char *namespace = rcl_node_get_namespace(node);
    if ( namespace )
      return PL_unify_chars(Value, PL_ATOM|REP_UTF8, (size_t)-1, namespace);
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
} rclswi_publisher_t;

static void
free_rcl_publisher(void *ptr)
{ rclswi_publisher_t *pub = ptr;

  TRYVOID(rcl_publisher_fini(&pub->publisher, pub->node));
}


static foreign_t
ros_publish(term_t Node, term_t MsgType, term_t Topic, term_t QoSProfile,
	    term_t Publisher)
{ rcl_node_t *node;
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

  if ( !(pub = malloc(sizeof(*pub))) )
    return PL_resource_error("memory");
  pub->publisher = rcl_get_zero_initialized_publisher();
  pub->node = node;

  TRY(rcl_publisher_init(&pub->publisher, node, msg_type->type_support,
			 topic, &publisher_ops));

  if ( !rc )
  { free(pub);
  } else
  { rc = unify_pointer(Publisher, pub, &publisher_type);
  }

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
{ rcl_node_t *node;
  int rc = TRUE;
  char *topic;
  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rclswi_subscription_t *sub = NULL;
  rclswi_message_type_t *msg_type;

  if ( !get_pointer(Node, (void**)&node, &node_type) )
    return FALSE;
  if ( !get_pointer(MsgType, (void**)&msg_type, &rclswi_message_type_type) )
    return FALSE;
  if ( !PL_get_chars(Topic, &topic, CVT_ATOM|CVT_STRING|CVT_EXCEPTION|REP_UTF8) )
    return FALSE;

  if ( !(sub = malloc(sizeof(*sub))) )
    return PL_resource_error("memory");
  sub->subscription = rcl_get_zero_initialized_subscription();
  sub->node = node;
  sub->type_support = msg_type;
  sub->waiting = FALSE;
  sub->deleted = FALSE;

  TRY(rcl_subscription_init(&sub->subscription, node,
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
		 *	      TYPES		*
		 *******************************/

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
{ strcpy(end, postfix);

  return dlsym(RTLD_DEFAULT, name);
}

static const
rosidl_message_type_support_t *
introspection_func(char *name)
{ void *sym = dlsym(RTLD_DEFAULT, name);

  if ( sym )
  { const rosidl_message_type_support_t *(*func)(void) = sym;

    return (*func)();
  }

  return NULL;
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
    { char buf[512];
      char *end;

      strcpy(buf, prefix);
      end = buf+strlen(buf);

      if ( !(ret->init	        = msg_func(buf, end, "__init"))      ||
	   !(ret->create        = msg_func(buf, end, "__create"))    ||
	   !(ret->fini          = msg_func(buf, end, "__fini"))      ||
	   !(ret->destroy       = msg_func(buf, end, "__destroy"))   ||
	   !(ret->introspection = introspection_func(introspection)) ||
	   !(ret->type_support  = introspection_func(type_support)) )
      { free(ret);
	return FALSE;
      }

      return unify_pointer(Functions, ret, &rclswi_message_type_type);
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

#define FAST_NAME_LEN 64

static atom_t
lwr_atom(const char *name)
{ size_t len = strlen(name);
  char buf[FAST_NAME_LEN+1];
  char *lwr = (len > FAST_NAME_LEN ? malloc(len+1) : buf);

  if ( lwr )
  { const char *i=name;
    char *o = lwr;
    atom_t a;

    while (*i)
      *o++ = tolower(*i++);
    *o = '\0';

    a = PL_new_atom(lwr);
    if ( lwr != buf )
      free(lwr);

    return a;
  }

  return (atom_t)0;
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
		 PL_put_atom(values+i, ta) &&
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
ros_type_introspection(term_t MsgFuctions, term_t Type)
{ rclswi_message_type_t *msg_type;
  term_t tmp = PL_new_term_ref();

  if ( !get_pointer(MsgFuctions, (void**)&msg_type, &rclswi_message_type_type) )
    return FALSE;

  return ( put_type(tmp, msg_type->introspection) &&
	   PL_unify(Type, tmp) );
}


static size_t
size_of_member_type(uint8_t type_id)
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
#if 0					/* not in foxy */
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
    { wchar_t f = *(wchar_t*)ptr;
      return PL_unify_wchars(t, PL_ATOM, 1, &f);
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
    { uint64_t f = *(int64_t*)ptr;
      return PL_put_int64(t, f);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
    { rosidl_runtime_c__String *s = ptr;

      return PL_unify_chars(t, PL_STRING|REP_UTF8, s->size, s->data);
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
      Sdprintf("TBD: Strings support\n");
    default:
      assert(0);
      return FALSE;
  }
}


static int
put_dynamic_array(term_t Message, void *msg, const rosidl_typesupport_introspection_c__MessageMember *member)
{ Sdprintf("TBD: dynamic arrays\n");
  return FALSE;
}

static int
put_fixed_array(term_t Message, void *msg, const rosidl_typesupport_introspection_c__MessageMember *member)
{ term_t tail = PL_copy_term_ref(Message);
  term_t head = PL_new_term_ref();
  int rc;
  size_t elem_size = size_of_member_type(member->type_id_);

  for(size_t i=0; i<member->array_size_; i++)
  { if ( !PL_unify_list(tail, head, tail) ||
	 !put_primitive(head, (char*)msg+elem_size*i, member->type_id_) )
      return FALSE;
  }

  rc = PL_unify_nil(tail);
  PL_reset_term_refs(tail);

  return rc;
}


#define FAST_KEYS 32

static int
put_message(term_t Message, void *msg, const rosidl_message_type_support_t *ts)
{ const rosidl_typesupport_introspection_c__MessageMembers *members = ts->data;
  int rc = TRUE;

  if ( members->member_count_ == 1 && 0 )
  { const rosidl_typesupport_introspection_c__MessageMember *mem = members->members_;
    (void)mem;
    Sdprintf("TBD: non-struct support\n");
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
    { const rosidl_typesupport_introspection_c__MessageMember *mem = members->members_+i;

      keys[i] = lwr_atom(mem->name_);
      DEBUG(5, Sdprintf("Put %s -> ", mem->name_));
      if ( mem->type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE )
      { if ( !put_message(values+i, (char*)msg+mem->offset_, mem->members_) )
	  rc = FALSE;
      } else if ( mem->is_array_ )
      { if ( mem->is_upper_bound_ || mem->array_size_ == 0 )
	  return put_dynamic_array(Message, msg, mem);
	else
	  return put_fixed_array(Message, msg, mem);
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
		 *	       WAIT		*
		 *******************************/

typedef struct wait_obj
{ c_pointer_type *ctype;
  union
  { rcl_subscription_t    *subscription;
    rclswi_subscription_t *swi_subscription;
    rcl_guard_condition_t *guard_condition;
    rcl_timer_t           *timer;
    rcl_client_t          *client;
    rcl_service_t	  *service;
    rcl_event_t		  *events;
    void                  *ptr;
  } obj;				/* The pointer */
  atom_t symbol;			/* Prolog blob handle */
} wait_obj;

static int
add_to_ready(term_t tail, term_t head, size_t len,
	     wait_obj *objs, const void *ptr)
{ size_t i;

  for(i=0; i<len; i++)
  { if ( objs[i].obj.ptr == ptr )
    { return ( PL_unify_list_ex(tail, head, tail) &&
	       PL_unify_atom(head, objs[i].symbol) );
    }
  }

  assert(0);
  return FALSE;
}

static rcl_ret_t
rclswi_wait(rcl_wait_set_t *wset, int64_t tmo, wait_obj *objs, size_t len)
{ int rc = TRUE;
  rcl_ret_t ret;

  TRY(rcl_wait_set_clear(wset));
  for(size_t i=0; i<len && rc; i++)
  { if ( objs[i].ctype == &subscription_type )
    { rclswi_subscription_t *sub = objs[i].obj.swi_subscription;
      if ( !sub->deleted )
      { sub->waiting = TRUE;
	TRY(rcl_wait_set_add_subscription(wset, &sub->subscription, NULL));
      }
    } else
      assert(0);
  }

  if ( rc )
    ret = rcl_wait(wset, tmo);
  else
    assert(0);				/* TBD: pass Prolog exception */

  for(size_t i=0; i<len && rc; i++)
  { if ( objs[i].ctype == &subscription_type )
    { rclswi_subscription_t *sub = objs[i].obj.swi_subscription;
      sub->waiting = FALSE;
      if ( sub->deleted )
	TRY_ANYWAY(rcl_subscription_fini(&sub->subscription, sub->node));
    } else
      assert(0);
  }

  return ret;
}


#define WAIT_POLL 250000000

static foreign_t
ros_wait(term_t For, term_t Timeout, term_t Ready)
{ double tmo_sec;
  int64_t tmo_nsec;
  term_t tail = PL_copy_term_ref(For);
  term_t head = PL_new_term_ref();
  int rc = TRUE;
  size_t len;
  wait_obj *objs = NULL;
  int i;
  size_t nsubs=0, nguards=0, ntimers=0, nclients=0, nservices=0, nevents=0;

  if ( PL_get_float(Timeout, &tmo_sec) )
  { tmo_nsec = (int64_t)(tmo_sec*1000000000.0);
  } else
  { atom_t inf;

    if ( PL_get_atom(Timeout, &inf) &&
	 (inf == ATOM_inf || inf == ATOM_infinite) )
    { tmo_nsec = -1;
    } else
    { return PL_type_error("float", Timeout);
    }
  }

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
    else
      assert(0);
  }
  if ( !PL_get_nil_ex(tail) )
    OUTFAIL;

  rcl_wait_set_t wset = rcl_get_zero_initialized_wait_set();
  DEBUG(3, Sdprintf("rcl_wait(): %d subscriptions\n", nsubs));
  TRY(rcl_wait_set_init(&wset,
			nsubs, nguards, ntimers, nclients, nservices, nevents,
			default_context, rcl_get_default_allocator()));
  int wset_initted = TRUE;

  rcl_ret_t ret = RCL_RET_TIMEOUT;
  while ( tmo_nsec > WAIT_POLL && ret == RCL_RET_TIMEOUT )
  { tmo_nsec -= WAIT_POLL;
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
  { TRY(ret);
  }

  tail = PL_copy_term_ref(Ready);
  for(int i = 0; i < nsubs; i++)
  { if ( wset.subscriptions[i] &&
	 !add_to_ready(tail, head, len, objs, wset.subscriptions[i]) )
      OUTFAIL;
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
ros_client_names_and_types_by_node(term_t Node,
				   term_t NodeName, term_t NameSpace,
				   term_t NamesAndTypes)
{ rcl_node_t *node;
  char *node_name;
  char *namespace;
  rcl_names_and_types_t nat = rcl_get_zero_initialized_names_and_types();
  int rc = TRUE;

  if ( !get_pointer(Node, (void**)&node, &node_type) ||
       !get_utf8_name_ex(NodeName, &node_name) ||
       !get_utf8_name_ex(NameSpace, &namespace) )
    return FALSE;

  TRY(rcl_get_client_names_and_types_by_node(
	  node, &default_allocator, node_name, namespace,
	  &nat));

  if ( rc && !unify_rcl_names_and_types(NamesAndTypes, &nat) )
    OUTFAIL;

out:
  rc = rclswi_names_and_types_fini(&nat) && rc;
  return rc;
}


static foreign_t
ros_topic_names_and_types(term_t Node, term_t NamesAndTypes)
{ rcl_node_t *node;
  int rc = TRUE;

  if ( !get_pointer(Node, (void**)&node, &node_type) )
    return FALSE;

  rcl_names_and_types_t nat = rcl_get_zero_initialized_names_and_types();
  TRY(rcl_get_topic_names_and_types(node, &default_allocator, FALSE, &nat));

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

static void *
rcl_alloc(size_t bytes, rcl_allocator_t *allocator)
{ void *ptr;

  if ( !allocator )
    allocator = &default_allocator;

  if ( (ptr=allocator->allocate(bytes, allocator->state)) )
    return ptr;

  return PL_resource_error("memory"),NULL;
}

static void
rcl_free(void *ptr, rcl_allocator_t *allocator)
{ if ( !allocator )
    allocator = &default_allocator;

  allocator->deallocate(ptr, allocator->state);
}

static int
get_utf8_name_ex(term_t t, char **name)
{ return PL_get_chars(t, name, CVT_ATOMIC|CVT_EXCEPTION|REP_UTF8);
}



		 /*******************************
		 *	      REGISTER		*
		 *******************************/

#define MKFUNCTOR(n,a) \
        FUNCTOR_ ## n ## a = PL_new_functor(PL_new_atom(#n), a)
#define MKATOM(n) \
	ATOM_ ## n = PL_new_atom(#n);

install_t
install_librclswi(void)
{ MKFUNCTOR(error, 2);
  MKFUNCTOR(ros_error, 2);
  MKFUNCTOR(list, 1);
  MKFUNCTOR(list, 2);
  FUNCTOR_minus2 = PL_new_functor(PL_new_atom("-"), 2);

  MKATOM(cli_args);
  MKATOM(inf);
  MKATOM(infinite);
  MKATOM(name);
  MKATOM(namespace);

  default_allocator = rcl_get_default_allocator();

  PL_register_foreign("ros_debug",          1, ros_debug,          0);

  PL_register_foreign("ros_create_context", 1, ros_create_context, 0);
  PL_register_foreign("ros_init",           3, ros_init,	   0);
  PL_register_foreign("$ros_shutdown",	    1, ros_shutdown,       0);
  PL_register_foreign("ros_ok",             1, ros_ok,             0);
  PL_register_foreign("ros_create_node",    4, ros_create_node,    0);
  PL_register_foreign("ros_node_fini",      1, ros_node_fini,      0);
  PL_register_foreign("$ros_node_prop",     3, ros_node_prop,      0);
  PL_register_foreign("$ros_publish",       5, ros_publish,        0);
  PL_register_foreign("$ros_subscribe",     5, ros_subscribe,      0);
  PL_register_foreign("$ros_unsubscribe",   1, ros_unsubscribe,    0);

  PL_register_foreign("$ros_message_type",  4, ros_message_type,   0);
  PL_register_foreign("$ros_type_introspection", 2, ros_type_introspection,  0);

  PL_register_foreign("ros_client_names_and_types_by_node", 4,
		      ros_client_names_and_types_by_node, 0);
  PL_register_foreign("ros_topic_names_and_types", 2,
		      ros_topic_names_and_types, 0);

  PL_register_foreign("ros_wait",	    3, ros_wait,	   0);

  PL_register_foreign("ros_take",	    3, ros_take,           0);

  PL_register_foreign("ros_rwm_implementation", 1, ros_rwm_implementation, 0);
}
