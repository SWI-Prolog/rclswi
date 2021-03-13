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

#include "common.h"
#include "pointer.h"
#include "qos.h"
#include <math.h>

#include <rmw/qos_profiles.h>

static atom_t ATOM_qos_profile;

static atom_t ATOM_history;
static atom_t ATOM_depth;
static atom_t ATOM_reliability;
static atom_t ATOM_durability;
static atom_t ATOM_deadline;
static atom_t ATOM_lifespan;
static atom_t ATOM_liveliness;
static atom_t ATOM_liveliness_lease_duration;
static atom_t ATOM_avoid_ros_namespace_conventions;

static const c_pointer_type qos_profile_type =
{ "qos_profile",
  NULL
};


int
get_qos_profile(term_t t, rmw_qos_profile_t **profile)
{ return get_pointer(t, (void**)profile, &qos_profile_type);
}


typedef struct enum_decl
{ int		value;
  const char   *name;
  atom_t	atom;
} enum_decl;

#define EN_DECL(value, name) { (int)value, #name, 0 }
#define EN_END()	     { 0, NULL, 0 }

static enum_decl enum_history[] =
{ EN_DECL(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,     system_default),
  EN_DECL(RMW_QOS_POLICY_HISTORY_KEEP_LAST,          keep_last),
  EN_DECL(RMW_QOS_POLICY_HISTORY_KEEP_ALL,           keep_all),
  EN_DECL(RMW_QOS_POLICY_HISTORY_UNKNOWN,	     unknown),
  EN_END()
};

static enum_decl enum_reliability[] =
{ EN_DECL(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT, system_default),
  EN_DECL(RMW_QOS_POLICY_RELIABILITY_RELIABLE,       reliable),
  EN_DECL(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,    best_effort),
  EN_DECL(RMW_QOS_POLICY_RELIABILITY_UNKNOWN,        unknown),
  EN_END()
};

static enum_decl enum_durability[] =
{ EN_DECL(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,  system_default),
  EN_DECL(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, transient_local),
  EN_DECL(RMW_QOS_POLICY_DURABILITY_VOLATILE,        volatile),
  EN_DECL(RMW_QOS_POLICY_DURABILITY_UNKNOWN,         unknown),
  EN_END()
};

static enum_decl enum_liveliness[] =
{ EN_DECL(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,  system_default),
  EN_DECL(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,       automatic),
#if 0					/* deprecated */
  EN_DECL(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,  manual_by_node),
#endif
  EN_DECL(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, manual_by_topic),
  EN_DECL(RMW_QOS_POLICY_LIVELINESS_UNKNOWN,	     unknown),
  EN_END()
};


static int
get_enum(term_t t, const char *domain, enum_decl *decl, int *value)
{ atom_t a;

  if ( PL_get_atom_ex(t, &a) )
  { for(; decl->name; decl++)
    { if ( !decl->atom )
	decl->atom = PL_new_atom(decl->name);

      if ( decl->atom == a )
      { *value = decl->value;
	return TRUE;
      }
    }

    return PL_domain_error(domain,  t);
  } else
    return FALSE;
}


static int
get_rwm_time(term_t t, rmw_time_t *time)
{ double d;

  if ( PL_get_float(t, &d) )
  { double ip;

    time->nsec = (uint64_t)(modf(d, &ip)*1000000000.0);
    time->sec  = (uint64_t)ip;

    return TRUE;
  } else
    return PL_type_error("time", t);
}


static foreign_t
ros_qos_profile_create(term_t Dict, term_t Default, term_t QoSProfile)
{ rmw_qos_profile_t *profile, *qos_default = NULL;
  int rc = TRUE;

  if ( !PL_is_variable(Default) &&
       !get_pointer(Default, (void**)&qos_default, &qos_profile_type) )
    return FALSE;

  if ( (profile = malloc(sizeof(*profile))) )
  { term_t Value = PL_new_term_ref();

    if ( qos_default )
      *profile = *qos_default;
    else
      *profile = rmw_qos_profile_default;

    if ( PL_get_dict_key(ATOM_history, Dict, Value) )
    { int value;

      if ( get_enum(Value, "qos_history", enum_history, &value) )
	profile->history = value;
      else
	OUTFAIL;
    }

    if ( PL_get_dict_key(ATOM_depth, Dict, Value) )
    { size_t depth;

      if ( PL_get_size_ex(Value, &depth) )
	profile->depth = depth;
      else
	OUTFAIL;
    }

    if ( PL_get_dict_key(ATOM_reliability, Dict, Value) )
    { int value;

      if ( get_enum(Value, "qos_reliability", enum_reliability, &value) )
	profile->reliability = value;
      else
	OUTFAIL;
    }

    if ( PL_get_dict_key(ATOM_durability, Dict, Value) )
    { int value;

      if ( get_enum(Value, "qos_durability", enum_durability, &value) )
	profile->durability = value;
      else
	OUTFAIL;
    }

    if ( PL_get_dict_key(ATOM_deadline, Dict, Value) )
    { if ( !get_rwm_time(Value, &profile->deadline) )
	OUTFAIL;
    }

    if ( PL_get_dict_key(ATOM_lifespan, Dict, Value) )
    { if ( !get_rwm_time(Value, &profile->lifespan) )
	OUTFAIL;
    }

    if ( PL_get_dict_key(ATOM_liveliness, Dict, Value) )
    { int value;

      if ( get_enum(Value, "qos_liveliness", enum_liveliness, &value) )
	profile->liveliness = value;
      else
	OUTFAIL;
    }

    if ( PL_get_dict_key(ATOM_liveliness_lease_duration, Dict, Value) )
    { if ( !get_rwm_time(Value, &profile->liveliness_lease_duration) )
	OUTFAIL;
    }

    if ( PL_get_dict_key(ATOM_avoid_ros_namespace_conventions, Dict, Value) )
    { int val;

      if ( PL_get_bool_ex(Value, &val) )
	profile->avoid_ros_namespace_conventions = val;
      else
	OUTFAIL;
    }
  } else
    return PL_resource_error("memory");

out:
  if ( rc )
    return unify_pointer(QoSProfile, profile, &qos_profile_type);
  else
    free(profile);

  return rc;
}


static int
put_enum(term_t t, enum_decl *decl, int value)
{ for(; decl->name; decl++)
  { if ( decl->value == value )
    { if ( !decl->atom )
	decl->atom = PL_new_atom(decl->name);
      return PL_put_atom(t, decl->atom);
    }
  }

  return PL_unify_integer(t, value);
}


static int
put_rwm_time(term_t t, rmw_time_t *time)
{ double d = (double)time->sec+(double)time->nsec/1000000000.0;

  return PL_put_float(t, d);
}


static foreign_t
ros_qos_profile_to_prolog(term_t QoSProfile, term_t Dict)
{ rmw_qos_profile_t *profile;

  if ( get_pointer(QoSProfile, (void**)&profile, &qos_profile_type) )
  { const atom_t keys[] = { ATOM_history, ATOM_depth, ATOM_reliability,
			    ATOM_durability, ATOM_deadline, ATOM_lifespan,
			    ATOM_liveliness, ATOM_liveliness_lease_duration,
			    ATOM_avoid_ros_namespace_conventions };
    term_t tmp;
    term_t values;

    return ( (values=PL_new_term_refs(9)) &&
	     put_enum(values+0, enum_history, profile->history) &&
	     PL_put_uint64(values+1, profile->depth) &&
	     put_enum(values+2, enum_reliability, profile->reliability) &&
	     put_enum(values+3, enum_durability, profile->durability) &&
	     put_rwm_time(values+4, &profile->deadline) &&
	     put_rwm_time(values+5, &profile->lifespan) &&
	     put_enum(values+6, enum_liveliness, profile->liveliness) &&
	     put_rwm_time(values+7, &profile->liveliness_lease_duration) &&
	     PL_put_bool(values+8, profile->avoid_ros_namespace_conventions) &&
	     (tmp=PL_new_term_ref()) &&
	     PL_put_dict(tmp, ATOM_qos_profile, 9, keys, values) &&
	     PL_unify(Dict, tmp) );
  } else
    return FALSE;
}


static foreign_t
qos_enum_values(term_t name, term_t values)
{ atom_t a;

  if ( PL_get_atom_ex(name, &a) )
  { enum_decl *decl = NULL;

    if	    ( a == ATOM_history )     decl = enum_history;
    else if ( a == ATOM_reliability ) decl = enum_reliability;
    else if ( a == ATOM_durability )  decl = enum_durability;
    else if ( a == ATOM_liveliness )  decl = enum_liveliness;
    else return PL_domain_error("qos_enum", name);

    term_t tail = PL_copy_term_ref(values);
    term_t head = PL_new_term_ref();

    for(; decl->name; decl++)
    { if ( !decl->atom )
	decl->atom = PL_new_atom(decl->name);

      if ( !PL_unify_list(tail, head, tail) ||
	   !PL_unify_atom(head, decl->atom) )
	return FALSE;
    }

    return PL_unify_nil(tail);
  }

  return FALSE;
}


void
install_ros_qos(void)
{ MKATOM(history);
  MKATOM(depth);
  MKATOM(reliability);
  MKATOM(durability);
  MKATOM(deadline);
  MKATOM(lifespan);
  MKATOM(liveliness);
  MKATOM(liveliness_lease_duration);
  MKATOM(avoid_ros_namespace_conventions);
  MKATOM(qos_profile);

  PL_register_foreign("$ros_qos_profile_create", 3,
		      ros_qos_profile_create, 0);
  PL_register_foreign("$ros_qos_profile_to_prolog", 2,
		      ros_qos_profile_to_prolog, 0);
  PL_register_foreign("$qos_enum_values", 2, qos_enum_values, 0);
}
