// included by glib2.pas

{*
 * gtypes.inc
 *
 * depends on glibconfig.inc
 *}

{$IFDEF read_forward_definitions}
   {$IFDEF KYLIX}
     dword = Cardinal;
     qword = Int64;
   {$ENDIF}

   PPPchar  = ^PPchar;
   PPPgchar = ^PPgchar;
   PPgchar  = ^Pgchar;
   {$IFNDEF KYLIX}
   Pgchar   = ^gchar;
   {$ELSE}
   Pgchar   = PChar;
   {$ENDIF}
   gchar    = char;

   Pgshort = ^gshort;
   gshort = cshort;

   Pglong = ^glong;
   glong  = clong;

   Pgint = ^gint;
   PPgint = ^Pgint;
   gint  = cint;

   Pgboolean = ^gboolean;
   gboolean  = longbool;

   Pguchar = ^guchar;
   PPguchar = ^Pguchar;
   guchar = byte;

   Pgushort = ^gushort;
   gushort = cushort;

   Pgulong = ^gulong;
   gulong = culong;

   Pguint = ^guint;
   guint = cuint;

   Pgfloat = ^gfloat;
   gfloat = cfloat;

   Pgdouble = ^gdouble;
   gdouble = cdouble;

   pgpointer     = ^gpointer;
   gpointer      = pointer;
   gconstpointer = pointer;

   PGCompareFunc = ^TGCompareFunc;
   TGCompareFunc = function (a, b: gconstpointer): gint; cdecl;

   PGCompareDataFunc = ^TGCompareDataFunc;
   TGCompareDataFunc = function (a, b: gconstpointer;
                                 user_data: gpointer): gint; cdecl;

   PGEqualFunc   = ^TGEqualFunc;
   TGEqualFunc   = function (a, b: gconstpointer): gboolean; cdecl;

   PGDestroyNotify = ^TGDestroyNotify;
   TGDestroyNotify = procedure (data: gpointer);cdecl;

   PGFunc     = ^TGFunc;
   TGFunc     = procedure (data, userdata: gpointer;
                        key           : gconstpointer); cdecl;

   PGHashFunc = ^TGHashFunc;
   TGHashFunc = function (key : gconstpointer): guint; cdecl;


   PGHFunc    = ^TGHFunc;
   TGHFunc    = procedure (key, value, user_data: gpointer); cdecl;

   PGFreeFunc = procedure (data: gpointer); cdecl;

   PGTimeVal = ^TGTimeVal;
   TGTimeVal = record
                 tv_sec : glong;
                 tv_usec : glong;
               end;
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}

{ Mathematische Konstanten}

  const
       G_E     = 2.7182818284590452354E0;
       G_LN2   = 6.9314718055994530942E-1;
       G_LN10  = 2.3025850929940456840E0;
       G_PI    = 3.14159265358979323846E0;
       G_PI_2  = 1.57079632679489661923E0;
       G_PI_4  = 0.78539816339744830962E0;
       G_SQRT2 = 1.4142135623730950488E0;

{ byteorder}
       G_LITTLE_ENDIAN = 1234;
       G_BIG_ENDIAN    = 4321;
       G_PDP_ENDIAN    = 3412;          { unused, need specific PDP check }
{

 Basic bit swapping functions
}
function GUINT16_SWAP_LE_BE_CONSTANT(val: guint16): guint16;
function GUINT32_SWAP_LE_BE_CONSTANT(val: guint32): guint32;

{
 Portable versions of host-network order stuff
}
{ ToDo:
function g_ntohl(val): guint32;
// Result:=GUINT32_FROM_BE (val);
function g_ntohs(val): guint16;
// Result:=GUINT16_FROM_BE (val);
function g_htonl(val): guint32;
// Result:=GUINT32_TO_BE (val);
function g_htons(val): guint16;
// Result:=GUINT16_TO_BE (val);
}

{$ENDIF read_interface_rest}
// included by glib2.pas

