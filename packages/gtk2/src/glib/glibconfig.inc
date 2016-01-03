// included by glib2.pas

{*
 * glibconfig.inc
 *
 * depends on gmacros.inc
 *}

{$IFDEF read_forward_definitions}
   Pgint8 = ^gint8;
   gint8 = shortint;

   Pguint8 = ^guint8;
   guint8 = byte;

   Pgint16 = ^gint16;
   gint16 = smallint;

   Pguint16 = ^guint16;
   guint16 = word;

   Pgint32 = ^gint32;
   gint32 = longint;

   Pguint32 = ^guint32;
   guint32 = dword;

   Pgint64 = ^gint64;
   gint64  = int64;

   Pguint64 = ^guint64;
   guint64  = qword;

   pgssize  = ^gssize;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   gssize   = SizeInt;
   pgsize   = ^gsize;
   gsize    = SizeUInt;
=======
   gssize   = longint;
   pgsize   = ^gsize;
   gsize    = dword;
>>>>>>> graemeg/fixes_2_2
=======
   gssize   = longint;
   pgsize   = ^gsize;
   gsize    = dword;
>>>>>>> origin/fixes_2_2
=======
   gssize   = SizeInt;
   pgsize   = ^gsize;
   gsize    = SizeUInt;
>>>>>>> origin/fixes_2.4

{$ENDIF read_forward_definitions}


//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
  PGSystemThread = ^TGSystemThread;
  TGSystemThread = record
    data           : array [0..3] of char;
    dummy_double   : double;
    dummy_pointer  : pointer;
    dummy_long     : longint;
  end;
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}

const
   G_MINFLOAT   = 5.0e-324;
   G_MAXFLOAT   = 1.7e308;
   G_MINDOUBLE  = G_MINFLOAT;  // since gdouble is the same
   G_MAXDOUBLE  = G_MAXFLOAT;  // as gfloat
   G_MAXSHORT   = 32767;
   G_MINSHORT   = -G_MAXSHORT-1;
   G_MAXUSHORT  = 2*G_MAXSHORT+1;
   G_MAXINT     = 2147483647;
   G_MININT     = -G_MAXINT-1;
   G_MAXUINT    = 4294967295;   //  2*G_MAXINT+1;
   G_MINLONG    = G_MININT;        // since glong is the same
   G_MAXLONG    = G_MAXINT;        // as gint
   G_MAXULONG   = G_MAXUINT;
   G_MAXINT64   = 9223372036854775807;
   G_MININT64   = -G_MAXINT64-1;
   {$IFNDEF KYLIX}
   G_MAXUINT64  = qword(2)*qword(G_MAXINT64)+1;
   {$ELSE}
   G_MAXUINT64  = G_MAXINT64;   //  Kylix doesn't support constants bigger than G_MAXINT64
   {$ENDIF}


const
   G_GINT16_FORMAT = 'hi';
   G_GUINT16_FORMAT = 'hu';

   G_GINT32_FORMAT = 'i';
   G_GUINT32_FORMAT = 'u';

   G_HAVE_GINT64 = 1;     { always true }

   G_GINT64_FORMAT  = 'I64i';
   G_GUINT64_FORMAT = 'I64u';

   GLIB_SIZEOF_VOID_P = SizeOf(Pointer);
   GLIB_SIZEOF_LONG   = SizeOf(longint);
   GLIB_SIZEOF_SIZE_T = SizeOf(longint);

const
   GLIB_SYSDEF_POLLIN    = 1;
   GLIB_SYSDEF_POLLOUT   = 4;
   GLIB_SYSDEF_POLLPRI   = 2;
   GLIB_SYSDEF_POLLERR   = 8;
   GLIB_SYSDEF_POLLHUP   = 16;
   GLIB_SYSDEF_POLLNVAL  = 32;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef windows}
=======
{$ifdef win32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef win32}
>>>>>>> origin/fixes_2_2
   G_MODULE_SUFFIX  = 'dll';
{$else}
   G_MODULE_SUFFIX  = 'so';
{$endif}

function GUINT_TO_POINTER(i: guint): pointer;

{$ENDIF read_interface_rest}

// included by glib2.pas

