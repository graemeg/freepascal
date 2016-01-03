// included by glib2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
   PGMemVTable = ^TGMemVTable;
   TGMemVTable = record
        malloc : function (n_bytes:gsize):gpointer; cdecl;
        realloc : function (mem:gpointer; n_bytes:gsize):gpointer; cdecl;
        free : procedure (mem:gpointer); cdecl;
        calloc : function (n_blocks:gsize; n_block_bytes:gsize):gpointer; cdecl;
        try_malloc : function (n_bytes:gsize):gpointer; cdecl;
        try_realloc : function (mem:gpointer; n_bytes:gsize):gpointer; cdecl;
     end;

   PGMemChunk = pointer; // internal structure of gmem.c
   PGAllocator = pointer; // internal structure of gmem.c
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
const
   G_MEM_ALIGN = GLIB_SIZEOF_VOID_P;

{ Memory allocation functions }

function g_malloc(n_bytes:gulong):gpointer; cdecl; external gliblib;
function g_malloc0(n_bytes:gulong):gpointer; cdecl; external gliblib;
function g_realloc(mem:gpointer; n_bytes:gulong):gpointer; cdecl; external gliblib;
procedure g_free(mem:gpointer); cdecl; external gliblib;
function g_try_malloc(n_bytes:gulong):gpointer; cdecl; external gliblib;
function g_try_realloc(mem:gpointer; n_bytes:gulong):gpointer; cdecl; external gliblib;

{ Convenience memory allocators }
function g_new(bytes_per_struct, n_structs: gsize): gpointer;
function g_new0(bytes_per_struct, n_structs: gsize): gpointer;
function g_renew(struct_size: gsize; OldMem: gpointer; n_structs : gsize) : gpointer;

{ Memory allocation virtualization for debugging purposes
   g_mem_set_vtable() has to be the very first GLib function called
   if being used
  }
{ optional  }

procedure g_mem_set_vtable(vtable:PGMemVTable); cdecl; external gliblib;
function g_mem_is_system_malloc:gboolean; cdecl; external gliblib;

{$IFNDEF KYLIX}
{ Memory profiler and checker, has to be enabled via g_mem_set_vtable() }
var
<<<<<<< HEAD
<<<<<<< HEAD
  {$IFDEF WINDOWS}
=======
  {$IFDEF WIN32}
>>>>>>> graemeg/fixes_2_2
=======
  {$IFDEF WIN32}
>>>>>>> origin/fixes_2_2
  glib_mem_profiler_table : PGMemVTable; external gliblib name 'glib_mem_profiler_table';
  {$ELSE}
  glib_mem_profiler_table : PGMemVTable;cvar;external;
  {$ENDIF}
{$ENDIF}

procedure g_mem_profile; cdecl; external gliblib;
{ Memchunk convenience functions }

{ c specific:
 #define g_mem_chunk_create(type, pre_alloc, alloc_type)        (
   g_mem_chunk_new (#type " mem chunks (" #pre_alloc ")",
                   sizeof (type), sizeof (type) * (pre_alloc), (alloc_type)) ) }

function g_chunk_new(chunk : Pointer) : Pointer;
function g_chunk_new0(chunk : Pointer) : Pointer;
procedure g_chunk_free(mem_chunk:PGMemChunk; mem:gpointer);
{ "g_mem_chunk_new" creates a new memory chunk.
   Memory chunks are used to allocate pieces of memory which are
    always the same size. Lists are a good example of such a data type.
   The memory chunk allocates and frees blocks of memory as needed.
    Just be sure to call "g_mem_chunk_free" and not "g_free" on data
    allocated in a mem chunk. ("g_free" will most likely cause a seg
    fault...somewhere).

   Oh yeah, GMemChunk is an opaque data type. (You don't really
    want to know what's going on inside do you?)
  }
{ ALLOC_ONLY MemChunk's can only allocate memory. The free operation
    is interpreted as a no op. ALLOC_ONLY MemChunk's save 4 bytes per
    atom. (They are also useful for lists which use MemChunk to allocate
    memory but are also part of the MemChunk implementation).
   ALLOC_AND_FREE MemChunk's can allocate and free memory.
  }

const
   G_ALLOC_ONLY = 1;
   G_ALLOC_AND_FREE = 2;

function g_mem_chunk_new(name:Pgchar; atom_size:gint; area_size:gulong; _type:gint):PGMemChunk; cdecl; external gliblib;
procedure g_mem_chunk_destroy(mem_chunk:PGMemChunk); cdecl; external gliblib;
function g_mem_chunk_alloc(mem_chunk:PGMemChunk):gpointer; cdecl; external gliblib;
function g_mem_chunk_alloc0(mem_chunk:PGMemChunk):gpointer; cdecl; external gliblib;
procedure g_mem_chunk_free(mem_chunk:PGMemChunk; mem:gpointer); cdecl; external gliblib;
procedure g_mem_chunk_clean(mem_chunk:PGMemChunk); cdecl; external gliblib;
procedure g_mem_chunk_reset(mem_chunk:PGMemChunk); cdecl; external gliblib;
procedure g_mem_chunk_print(mem_chunk:PGMemChunk); cdecl; external gliblib;
procedure g_mem_chunk_info; cdecl; external gliblib;

{ Ah yes...we have a "g_blow_chunks" function.
   "g_blow_chunks" simply compresses all the chunks. This operation
    consists of freeing every memory area that should be freed (but
    which we haven't gotten around to doing yet). And, no,
    "g_blow_chunks" doesn't follow the naming scheme, but it is a
    much better name than "g_mem_chunk_clean_all" or something
    similar.
  }
procedure g_blow_chunks; cdecl; external gliblib;

{ Generic allocators }
function g_allocator_new(name:Pgchar; n_preallocs:guint):PGAllocator; cdecl; external gliblib;
procedure g_allocator_free(allocator:PGAllocator); cdecl; external gliblib;

{ internal  }
const
   G_ALLOCATOR_LIST = 1;
   G_ALLOCATOR_SLIST = 2;
   G_ALLOCATOR_NODE = 3;

{$ENDIF read_interface_rest}
// included by glib2.pas

