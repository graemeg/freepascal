(*
 * Summary: interface for the memory allocator
 * Description: provides interfaces for the memory allocator,
 *              including debugging capabilities.
 *
 * Copy: See Copyright for the status of this software.
 *
 * Author: Daniel Veillard
 *)

(**
 * DEBUG_MEMORY:
 *
 * DEBUG_MEMORY replaces the allocator with a collect and debug
 * shell to the libc allocator.
 * DEBUG_MEMORY should only be activated when debugging
 * libxml i.e. if libxml has been configured with --with-debug-mem too.
 *)
{.$DEFINE DEBUG_MEMORY_FREED}
{.$DEFINE DEBUG_MEMORY_LOCATION}

{$IFDEF DEBUG}
{$IFNDEF DEBUG_MEMORY}
{$DEFINE DEBUG_MEMORY}
{$ENDIF}
{$ENDIF}

(**
 * DEBUG_MEMORY_LOCATION:
 *
 * DEBUG_MEMORY_LOCATION should be activated only when debugging
 * libxml i.e. if libxml has been configured with --with-debug-mem too.
 *)
{$IFDEF DEBUG_MEMORY_LOCATION}
{$ENDIF}

{$IFDEF TYPE}
(*
 * The XML memory wrapper support 4 basic overloadable functions.
 *)
(**
 * xmlFreeFunc:
 * @mem: an already allocated block of memory
 *
 * Signature for a free() implementation.
 *)
  xmlFreeFunc = procedure(mem: pointer); EXTDECL;
  {$IFDEF NO_EXTERNAL_VARS}
  PxmlFreeFunc = ^xmlFreeFunc;
  {$ENDIF}

(**
 * xmlMallocFunc:
 * @size:  the size requested in bytes
 *
 * Signature for a malloc() implementation.
 *
 * Returns a pointer to the newly allocated block or NULL in case of error.
 *)
  xmlMallocFunc = function(size: csize_t): pointer; EXTDECL;
  {$IFDEF NO_EXTERNAL_VARS}
  PxmlMallocFunc = ^xmlMallocFunc;
  {$ENDIF}

(**
 * xmlReallocFunc:
 * @mem: an already allocated block of memory
 * @size:  the new size requested in bytes
 *
 * Signature for a realloc() implementation.
 *
 * Returns a pointer to the newly reallocated block or NULL in case of error.
 *)
  xmlReallocFunc = function(mem: pointer; size: csize_t): pointer; EXTDECL;
  {$IFDEF NO_EXTERNAL_VARS}
  PxmlReallocFunc = ^xmlReallocFunc;
  {$ENDIF}

(**
 * xmlStrdupFunc:
 * @str: a zero terminated string
 *
 * Signature for an strdup() implementation.
 *
 * Returns the copy of the string or NULL in case of error.
 *)
  xmlStrdupFunc = function(str: pchar): pchar; EXTDECL;
  {$IFDEF NO_EXTERNAL_VARS}
  PxmlStrdupFunc = ^xmlStrdupFunc;
  {$ENDIF}

(*
 * The 4 interfaces used for all memory handling within libxml.
LIBXML_DLL_IMPORT extern xmlFreeFunc xmlFree;
LIBXML_DLL_IMPORT extern xmlMallocFunc xmlMalloc;
LIBXML_DLL_IMPORT extern xmlMallocFunc xmlMallocAtomic;
LIBXML_DLL_IMPORT extern xmlReallocFunc xmlRealloc;
LIBXML_DLL_IMPORT extern xmlStrdupFunc xmlMemStrdup;
 *)
{$ENDIF}

{$IFDEF FUNCTION}
(*
 * The way to overload the existing functions.
 * The xmlGc function have an extra entry for atomic block
 * allocations useful for garbage collected memory allocators
 *)
function xmlMemSetup(freeFunc: xmlFreeFunc; mallocFunc: xmlMallocFunc; reallocFunc: xmlReallocFunc; strdupFunc: xmlStrdupFunc): cint; EXTDECL; external xml2lib;
function xmlMemGet(var freeFunc: xmlFreeFunc; var mallocFunc: xmlMallocFunc; var reallocFunc: xmlReallocFunc; var strdupFunc: xmlStrdupFunc): cint; EXTDECL; external xml2lib;
function xmlGcMemSetup(freeFunc: xmlFreeFunc; mallocFunc: xmlMallocFunc; mallocAtomicFunc: xmlMallocFunc; reallocFunc: xmlReallocFunc; strdupFunc: xmlStrdupFunc): cint; EXTDECL; external xml2lib;
function xmlGcMemGet(var freeFunc: xmlFreeFunc; var mallocFunc: xmlMallocFunc; var mallocAtomicFunc: xmlMallocFunc; var reallocFunc: xmlReallocFunc; var strdupFunc: xmlStrdupFunc): cint; EXTDECL; external xml2lib;

(*
 * Initialization of the memory layer.
 *)
function xmlInitMemory(): cint; EXTDECL; external xml2lib;

(*
 * Cleanup of the memory layer.
 *)
procedure xmlCleanupMemory(); EXTDECL; external xml2lib;

(*
 * These are specific to the XML debug memory wrapper.
 *)
function xmlMemUsed(): cint; EXTDECL; external xml2lib;
function xmlMemBlocks(): cint; EXTDECL; external xml2lib;
procedure xmlMemDisplay(fp: PFILE); EXTDECL; external xml2lib;
procedure xmlMemShow(fp: PFILE; nr: cint); EXTDECL; external xml2lib;
procedure xmlMemoryDump(); EXTDECL; external xml2lib;
function xmlMemMalloc(size: csize_t): pointer; EXTDECL; external xml2lib;
function xmlMemRealloc(ptr: pointer; size: csize_t): pointer; EXTDECL; external xml2lib;
procedure xmlMemFree(ptr: pointer); EXTDECL; external xml2lib;
function xmlMemoryStrdup(str: pchar): pchar; EXTDECL; external xml2lib;

function xmlMallocLoc(size: csize_t; _file: pchar; line: cint): pointer; EXTDECL; external xml2lib;
function xmlReallocLoc(ptr: pointer; size: csize_t; _file: pchar; line: cint): pointer; EXTDECL; external xml2lib;
function xmlMallocAtomicLoc(size: csize_t; _file: pchar; line: cint): pointer; EXTDECL; external xml2lib;
function xmlMemStrdupLoc(str: pchar; _file: pchar; line: cint): pchar; EXTDECL; external xml2lib;

{$IFDEF DEBUG_MEMORY_LOCATION}
(**
 * xmlMalloc:
 * @size:  number of bytes to allocate
 *
 * Wrapper for the malloc() function used in the XML library.
 *
 * Returns the pointer to the allocated area or NULL in case of error.
 *)
//#define xmlMalloc(size) xmlMallocLoc((size), __FILE__, __LINE__)

(**
 * xmlMallocAtomic:
 * @size:  number of bytes to allocate
 *
 * Wrapper for the malloc() function used in the XML library for allocation
 * of block not containing pointers to other areas.
 *
 * Returns the pointer to the allocated area or NULL in case of error.
 *)
//#define xmlMallocAtomic(size) xmlMallocAtomicLoc((size), __FILE__, __LINE__)

(**
 * xmlRealloc:
 * @ptr:  pointer to the existing allocated area
 * @size:  number of bytes to allocate
 *
 * Wrapper for the realloc() function used in the XML library.
 *
 * Returns the pointer to the allocated area or NULL in case of error.
 *)
//#define xmlRealloc(ptr, size) xmlReallocLoc((ptr), (size), __FILE__, __LINE__)

(**
 * xmlMemStrdup:
 * @str:  pointer to the existing string
 *
 * Wrapper for the strdup() function, xmlStrdup() is usually preferred.
 *
 * Returns the pointer to the allocated area or NULL in case of error.
 *)
//#define xmlMemStrdup(str) xmlMemStrdupLoc((str), __FILE__, __LINE__)

{$ENDIF} (* DEBUG_MEMORY_LOCATION *)
{$ENDIF}
