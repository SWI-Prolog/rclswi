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
#include <pointer.h>

typedef struct c_ptr
{ const c_pointer_type *type;
  const void	       *ptr;
} c_ptr;

static int
write_c_ptr(IOSTREAM *s, atom_t aref, int flags)
{ c_ptr *ref = PL_blob_data(aref, NULL, NULL);
  (void)flags;

  Sfprintf(s, "<%s>(%p)", ref->type->type, ref->ptr);
  return TRUE;
}


static void
acquire_c_ptr(atom_t aref)
{ c_ptr *ref = PL_blob_data(aref, NULL, NULL);
  (void)ref;
}


static int
release_c_ptr(atom_t aref)
{ c_ptr *ref = PL_blob_data(aref, NULL, NULL);

  if ( ref->type->free )
    (*ref->type->free)((void*)ref->ptr);

  return TRUE;
}

static int
save_c_ptr(atom_t aref, IOSTREAM *fd)
{ c_ptr *ref = PL_blob_data(aref, NULL, NULL);
  (void)fd;

  return PL_warning("Cannot save reference to <%s>(%p)",
		    ref->type->type, ref->ptr);
}

static atom_t
load_c_ptr(IOSTREAM *fd)
{ (void)fd;

  return PL_new_atom("<C>");
}

static PL_blob_t c_ptr_blob =
{ PL_BLOB_MAGIC,
  0,
  "c_ptr",
  release_c_ptr,
  NULL,
  write_c_ptr,
  acquire_c_ptr,
  save_c_ptr,
  load_c_ptr
};

int
unify_pointer(term_t t, void *ptr, const c_pointer_type *type)
{ const c_ptr ref = { .type = type, .ptr = ptr };

  return PL_unify_blob(t, (void*)&ref, sizeof(ref), &c_ptr_blob);
}

int
get_pointer(term_t t, void **ptr, const c_pointer_type *type)
{ c_ptr *ref;
  PL_blob_t *btype;

  if ( PL_get_blob(t, (void**)&ref, NULL, &btype) &&
       btype == &c_ptr_blob &&
       ref->type == type )
  { *ptr = (void*)ref->ptr;

    return TRUE;
  }

  return PL_type_error(type->type, t);
}

int
get_pointer_and_symbol(term_t t, void **ptr, atom_t *symbol,
		       const c_pointer_type *type)
{ c_ptr *ref;
  PL_blob_t *btype;
  atom_t a;

  if ( PL_get_atom(t, &a) &&
       (ref=PL_blob_data(a, NULL, &btype)) &&
       btype == &c_ptr_blob &&
       ref->type == type )
  { *ptr = (void*)ref->ptr;
    *symbol = a;

    return TRUE;
  }

  return PL_type_error(type->type, t);
}

int
get_pointer_ex(term_t t, void **ptr, atom_t *symbol, const c_pointer_type **type)
{ c_ptr *ref;
  PL_blob_t *btype;
  atom_t a;

  if ( PL_get_atom(t, &a) &&
       (ref=PL_blob_data(a, NULL, &btype)) &&
       btype == &c_ptr_blob )
  { *ptr    = (void*)ref->ptr;
    *symbol = a;
    *type   = ref->type;

    return TRUE;
  }

  return PL_type_error("ros_object", t);
}

int
get_pointer_type(term_t t, void **ptr, const c_pointer_type **type)
{ c_ptr *ref;
  PL_blob_t *btype;
  atom_t a;

  if ( PL_get_atom(t, &a) &&
       (ref=PL_blob_data(a, NULL, &btype)) &&
       btype == &c_ptr_blob )
  { *ptr    = (void*)ref->ptr;
    *type   = ref->type;

    return TRUE;
  }

  return FALSE;
}
