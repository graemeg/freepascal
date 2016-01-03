{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 Armin Diehl, member of the Free Pascal
    development team

    Interface to Netware libc

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}



{$ifndef INCLUDED_FROM_SYSTEM}
{$mode objfpc}
unit libc;
interface


{ Netware libc interface
  Translated from c ndk Armin Diehl 2004/09/02 }
{$endif}

const
  libc_nlm='libc';
  system_nlm='!netware';

{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
{ $ifndef INCLUDED_FROM_SYSTEM - (always available in 2.0.x and above - TH)
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;
$endif}
<<<<<<< HEAD
<<<<<<< HEAD
//  PPPChar   = ^PPChar;
=======
=======
>>>>>>> origin/fixes_2_2
  PPPChar   = ^PPChar;
>>>>>>> graemeg/fixes_2_2
  void      = pointer;
  cint      = longint;
  TNLMHandle = Pointer;

const
  NullNlmHandle = nil;

{$PACKRECORDS C}

// string.h
// limits.h

{ some limits wanted by POSIX...  }
{ exact-width signed types...  }
type

   Pint8_t = ^int8_t;
   int8_t = char;

   Pint16 = ^Tint16;
   Tint16 = smallint;

   Pint64_t = ^Tint64;
   Tint64 = int64;
{ exact-width unsigned types...  }

   Puint64 = ^Tuint64;
   Tuint64 = qword;
{ lower and upper bound of exact width integer types...  }
{ macros for minimum-width integer constants...  }
{ minimum-width signed integer types...  }

   Pint_least8_t = ^int_least8_t;
   int_least8_t = char;

   Pint_least16_t = ^int_least16_t;
   int_least16_t = smallint;

   Pint_least32_t = ^int_least32_t;
   int_least32_t = longint;

   Pint_least64_t = ^int_least64_t;
   int_least64_t = int64;
{ minimum-width unsigned integer types...  }

   Puint_least8_t = ^uint_least8_t;
   uint_least8_t = byte;

   Puint_least16_t = ^uint_least16_t;
   uint_least16_t = word;

   Puint_least32_t = ^uint_least32_t;
   uint_least32_t = dword;

   Puint_least64_t = ^uint_least64_t;
   uint_least64_t = qword;
{ lower and upper bound of exact width integer types...  }
{ fastest minimum-width signed integer types...  }

   Pint_fast8_t = ^int_fast8_t;
   int_fast8_t = char;

   Pint_fast16_t = ^int_fast16_t;
   int_fast16_t = smallint;

   Pint_fast32_t = ^int_fast32_t;
   int_fast32_t = longint;

   Pint_fast64_t = ^int_fast64_t;
   int_fast64_t = int64;
{ fastest minimum-width unsigned integer types...  }

   Puint_fast8_t = ^uint_fast8_t;
   uint_fast8_t = byte;

   Puint_fast16_t = ^uint_fast16_t;
   uint_fast16_t = word;

   Puint_fast32_t = ^uint_fast32_t;
   uint_fast32_t = dword;

   Puint_fast64_t = ^uint_fast64_t;
   uint_fast64_t = qword;
{ lower and upper bound of fastest minimum-width integer types...  }
{ integer types capable of holding object pointer...  }

   Pintptr_t = ^intptr_t;
   intptr_t = longint;

   Puintptr_t = ^uintptr_t;
   uintptr_t = dword;
{ limit of integer type capable of holding object pointer...  }
{ maximum-width integer types...  }

   Pintmax_t = ^intmax_t;
   intmax_t = int64;

   Puintmax_t = ^uintmax_t;
   uintmax_t = Tuint64;
{ macros for maximum-width integer constants...  }
{ limits for other integer types...  }

   Psize_t = ^size_t;
   size_t = dword;
{ type yielded by sizeof()  }

   Pssize_t = ^ssize_t;
   ssize_t = longint;
{ signed byte counts for file I/O  }

   Psize64_t = ^size64_t;
   size64_t = Tuint64;
{ used for 64-bit (long) file I/O  }

   Pmode_t = ^mode_t;
   mode_t = dword;
{ file attributes, permissions  }

   Poff_t = ^off_t;
   off_t = longint;
{ file offset value  }

   Poff64_t = ^off64_t;
   off64_t = int64;
{ 64-bit (long) file offset value  }

   Pino_t = ^ino_t;
   ino_t = Tuint64;

   Ppid_t = ^pid_t;
   pid_t = longint;
{ capable of holding a pointer or -1  }

   Puid_t = ^uid_t;
   uid_t = Tuint64;

   Pgid_t = ^gid_t;
   gid_t = Tuint64;

   Pblksize_t = ^blksize_t;
   blksize_t = Tuint64;

   Pblkcnt_t = ^blkcnt_t;
   blkcnt_t = Tuint64;

   Pdev_t = ^dev_t;
   dev_t = Tuint64;

   Pnlink_t = ^nlink_t;
   nlink_t = dword;

   Pptrdiff_t = ^ptrdiff_t;
   ptrdiff_t = longint;

   Pwchar_t = ^wchar_t;
   wchar_t = WideChar;
   PPwchar_t = ^Pwchar_t;


{ prototypes for functions standard and nonstandard...  }

function memchr(_para1:pointer; _para2:longint; _para3:size_t):pointer;cdecl;external libc_nlm name 'memchr';
function memcmp(_para1, _para2:pointer; _para3:size_t):longint;cdecl;external libc_nlm name 'memcmp';
function memcpy(__restrict, __restrict1:pointer; _para3:size_t):pointer;cdecl;external libc_nlm name 'memcpy';
function memmove(_para1, _para2:pointer; _para3:size_t):pointer;cdecl;external libc_nlm name 'memmove';
function memset(_para1:pointer; _para2:longint; _para3:size_t):pointer;cdecl;external libc_nlm name 'memset';
function strcasecmp(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'strcasecmp';
function strcat(dst,src:Pchar):Pchar;cdecl;external libc_nlm name 'strcat';
function strchr(_para1:Pchar; _para2:longint):Pchar;cdecl;external libc_nlm name 'strchr';
function strcmp(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'strcmp';
function strcoll(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'strcoll';
function strcpy(__restrict, __restrict1:Pchar):Pchar;cdecl;external libc_nlm name 'strcpy';
function strcspn(_para1, _para2:Pchar):size_t;cdecl;external libc_nlm name 'strcspn';
function strerror(_para1:longint):Pchar;cdecl;external libc_nlm name 'strerror';
function strlcat(__restrict, __restrict1:Pchar; _para3:size_t):size_t;cdecl;external libc_nlm name 'strlcat';
function strlcpy(__restrict, __restrict1:Pchar; _para3:size_t):size_t;cdecl;external libc_nlm name 'strlcpy';
function {$ifdef INCLUDED_FROM_SYSTEM}libc_strlen{$else}strlen{$endif}(_para1:Pchar):size_t;cdecl;external libc_nlm name 'strlen';
function strncasecmp(_para1, _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'strncasecmp';
function strncat(__restrict, __restrict1:Pchar; _para3:size_t):Pchar;cdecl;external libc_nlm name 'strncat';
function strncmp(_para1, _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'strncmp';
function strncoll(_para1, _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'strncoll';
function strncpy(__restrict, __restrict1:Pchar; _para3:size_t):Pchar;cdecl;external libc_nlm name 'strncpy';
function strnicmp(_para1, _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'strnicmp';
function strnset(_para1, _para2:longint; _para3:size_t):Pchar;cdecl;external libc_nlm name 'strnset';
function strpbrk(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'strpbrk';
function strrchr(_para1, _para2:longint):Pchar;cdecl;external libc_nlm name 'strrchr';
function strrev(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'strrev';
function strset(_para1:Pchar; _para2:longint):Pchar;cdecl;external libc_nlm name 'strset';
function strspn(_para1, _para2:Pchar):size_t;cdecl;external libc_nlm name 'strspn';
function strstr(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'strstr';
function strtok(__restrict, __restrict1:Pchar):Pchar;cdecl;external libc_nlm name 'strtok';
function strxfrm(__restrict, __restrict1:Pchar; _para3:size_t):size_t;cdecl;external libc_nlm name 'strxfrm';
{ POSIX and other functions...  }

function strtok_r(__restrict, __restrict1, __restrict2:PPchar):Pchar;cdecl;external libc_nlm name 'strtok_r';
function memicmp(_para1, _para2:pointer; _para3:size_t):longint;cdecl;external libc_nlm name 'memicmp';
function stpcpy(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'stpcpy';
function stricmp(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'stricmp';
function strdup(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'strdup';
function strecpy(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'strecpy';
function strerror_r(_para1:longint; _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'strerror_r';

{$ifndef DisableArrayOfConst}
function strlist(_para1, _para2:Pchar; args:array of const):Pchar;cdecl;external libc_nlm name 'strlist';
{$endif}
function strlist(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'strlist';
function strlwr(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'strlwr';
function strrindex(_para1:Pchar; _para2:size_t; _para3:longint):Pchar;cdecl;external libc_nlm name 'strrindex';
function strwhich(_para1:Pchar; _para2:longint; _para3:Pchar):Pchar;cdecl;external libc_nlm name 'strwhich';
function strupr(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'strupr';
procedure swab(_para1, _para2:pointer; _para3:size_t);cdecl;external libc_nlm name 'swab';
procedure swaw(_para1, _para2:pointer; _para3:size_t);cdecl;external libc_nlm name 'swaw';
procedure ungettok(__restrict, __restrict1:Pchar);cdecl;external libc_nlm name 'ungettok';
procedure ungettok_r(__restrict, __restrict1:Pchar; __restrict2:PPchar);cdecl;external libc_nlm name 'ungettok_r';
{ multibyte (double) interfaces for locale code page work...  }
function Lstrbcpy(__restrict, __restrict1:Pchar; _para3:size_t):Pchar;cdecl;external libc_nlm name 'Lstrbcpy';
function Lstrchr(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'Lstrchr';
function Lstrcmp(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'Lstrcmp';
function Lstrcoll(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'Lstrcoll';
function Lstrcspn(_para1, _para2:Pchar):size_t;cdecl;external libc_nlm name 'Lstrcspn';
function Lstricmp(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'Lstricmp';
function Lstrlen(_para1:Pchar):size_t;cdecl;external libc_nlm name 'Lstrlen';
function Lstrlwr(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'Lstrlwr';
function Lstrncat(__restrict:Pchar; __restrict1:Pchar; _para3:size_t):Pchar;cdecl;external libc_nlm name 'Lstrncat';
function Lstrncmp(_para1, _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'Lstrncmp';
function Lstrncoll(_para1, _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'Lstrncoll';
function Lstrncpy(__restrict, __restrict1:Pchar; _para3:size_t):Pchar;cdecl;external libc_nlm name 'Lstrncpy';
function Lstrnicmp(_para1, _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'Lstrnicmp';
function Lstrpbrk(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'Lstrpbrk';
function Lstrrchr(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'Lstrrchr';
function Lstrrev(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'Lstrrev';
function Lstrspn(_para1, _para2:Pchar):size_t;cdecl;external libc_nlm name 'Lstrspn';
function Lstrstr(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'Lstrstr';
function Lstrtok_r(__restrict:Pchar; __restrict1:Pchar; __restrict2:PPchar):Pchar;cdecl;external libc_nlm name 'Lstrtok_r';
function Lstrupr(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'Lstrupr';
function Lstrxfrm(__restrict:Pchar; __restrict1:Pchar; _para3:size_t):size_t;cdecl;external libc_nlm name 'Lstrxfrm';
{ length-preceeded string manipulation...  }
function ASCIIZToLenStr(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'ASCIIZToLenStr';
function ASCIIZToMaxLenStr(_para1, _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'ASCIIZToMaxLenStr';
function LenStrCat(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'LenStrCat';
function LenStrCmp(_para1, _para2:Pchar):longint;cdecl;external libc_nlm name 'LenStrCmp';
function LenStrCpy(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'LenStrCpy';
function LenToASCIIZStr(_para1,_para2:Pchar):longint;cdecl;external libc_nlm name 'LenToASCIIZStr';
function strindex(_para1, _para2:Pchar):Pchar;cdecl;external libc_nlm name 'strindex';


// stdarg.h

  const
     //EOF = -(1);
     BUFSIZ = 8096;           { default buffer size--change with setbuf()  }
     FOPEN_MAX = 1024;        { at least this many FILEs available         }
     FILENAME_MAX = 1024;     { maximum characters in any path name        }
                              { values for argument 'flags' to setvbuf()...}
     _IONBF = $0010;          { unbuffered (e.g.: stdout and stderr)       }
     _IOLBF = $0020;          { line buffered (e.g.: stdin)                }
     _IOFBF = $0040;          { fully buffered (most files)                }
                              { values for fseek()'s whence argument       }
     SEEK_SET = 0;            { add 'offset' to beginning of file          }
     SEEK_CUR = 1;            { add 'offset' to current position in file   }
     SEEK_END = 2;            { add 'offset' to end of file                }
                              { definitions for tmpnam() and tmpfil()      }
     TMP_MAX = 1000000;       { "T-000000.TMP" to "T-999999.TMP"           }
     L_tmpnam = 36;
     P_tmpdir = 'sys:/tmp';
  { FILE type definition (result is opaque)    }


type

   Pva_list = ^va_list;
   va_list = char;

   P_iobuf = ^_iobuf;
   _iobuf = record
        reserved : longint;
     end;
   TFILE = _iobuf;
   PFILE = ^TFILE;
   PPFILE = ^PFILE;

   Pfpos_t = ^fpos_t;
   fpos_t = longint;

   Pfpos64_t = ^fpos64_t;
   fpos64_t = off64_t;

procedure clearerr(_para1:PFILE);cdecl;external libc_nlm name 'clearerr';
function fclose(_para1:PFILE):longint;cdecl;external libc_nlm name 'fclose';
function feof(_para1:PFILE):longint;cdecl;external libc_nlm name 'feof';
function ferror(_para1:PFILE):longint;cdecl;external libc_nlm name 'ferror';
function fflush(_para1:PFILE):longint;cdecl;external libc_nlm name 'fflush';
function fgetc(_para1:PFILE):longint;cdecl;external libc_nlm name 'fgetc';
function fgetpos(_para1:PFILE; _para2:Pfpos_t):longint;cdecl;external libc_nlm name 'fgetpos';
function fgets(_para1:Pchar; _para2:longint; _para3:PFILE):Pchar;cdecl;external libc_nlm name 'fgets';


function fopen(__restrict:Pchar; __restrict1:Pchar):PFILE;cdecl;external libc_nlm name 'fopen';
{$ifndef DisableArrayOfConst}
function fprintf(__restrict:PFILE; __restrict1:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'fprintf';
{$endif}
function fprintf(__restrict:PFILE; __restrict1:Pchar):longint;cdecl;external libc_nlm name 'fprintf';
function fputc(_para1:longint; _para2:PFILE):longint;cdecl;external libc_nlm name 'fputc';

function fputs(__restrict:Pchar; __restrict1:PFILE):longint;cdecl;external libc_nlm name 'fputs';
function fread(__restrict:pointer; _para2:size_t; _para3:size_t; __restrict1:PFILE):size_t;cdecl;external libc_nlm name 'fread';


function freopen(__restrict:Pchar; __restrict1:Pchar; __restrict2:PFILE):PFILE;cdecl;external libc_nlm name 'freopen';
{$ifndef DisableArrayOfConst}
function fscanf(__restrict:PFILE; __restrict1:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'fscanf';
{$endif}
function fscanf(__restrict:PFILE; __restrict1:Pchar):longint;cdecl;external libc_nlm name 'fscanf';
function fseek(fp:PFILE; offset:longint; whence:longint):longint;cdecl;external libc_nlm name 'fseek';

function fsetpos(_para1:PFILE; _para2:Pfpos_t):longint;cdecl;external libc_nlm name 'fsetpos';
function ftell(_para1:PFILE):longint;cdecl;external libc_nlm name 'ftell';

function fwrite(__restrict:pointer; _para2:size_t; _para3:size_t; __restrict1:PFILE):size_t;cdecl;external libc_nlm name 'fwrite';
function getc(_para1:PFILE):longint;cdecl;external libc_nlm name 'getc';
function getchar:longint;cdecl;external libc_nlm name 'getchar';
function gets(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'gets';
procedure {$ifdef INCLUDED_FROM_SYSTEM}libc_perror{$else}perror{$endif}(_para1:Pchar);cdecl;external libc_nlm name 'perror';

{$ifndef DisableArrayOfConst}
function printf(__restrict:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'printf';
{$endif}
function printf(__restrict:Pchar):longint;cdecl;external libc_nlm name 'printf';
function putc(_para1:longint; _para2:PFILE):longint;cdecl;external libc_nlm name 'putc';
function putchar(_para1:longint):longint;cdecl;external libc_nlm name 'putchar';
function puts(_para1:Pchar):longint;cdecl;external libc_nlm name 'puts';
function remove(_para1:Pchar):longint;cdecl;external libc_nlm name 'remove';
function rename(_para1:Pchar; _para2:Pchar):longint;cdecl;external libc_nlm name 'rename';
procedure rewind(_para1:PFILE);cdecl;external libc_nlm name 'rewind';

{$ifndef DisableArrayOfConst}
function scanf(__restrict:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'scanf';
{$endif}
function scanf(__restrict:Pchar):longint;cdecl;external libc_nlm name 'scanf';
procedure setbuf(__restrict:PFILE; __restrict1:Pchar);cdecl;external libc_nlm name 'setbuf';
function setvbuf(__restrict:PFILE; __restrict1:Pchar; _para3:longint; _para4:size_t):longint;cdecl;external libc_nlm name 'setvbuf';
{$ifndef DisableArrayOfConst}
function sprintf(__restrict:Pchar; __restrict1:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'sprintf';
{$endif}
function sprintf(__restrict:Pchar; __restrict1:Pchar):longint;cdecl;external libc_nlm name 'sprintf';
{$ifndef DisableArrayOfConst}
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'snprintf';
{$endif}
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar):longint;cdecl;external libc_nlm name 'snprintf';
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar; p1:longint):longint;cdecl;external libc_nlm name 'snprintf';
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar; p1:longint; p2:pchar):longint;cdecl;external libc_nlm name 'snprintf';
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar; p1:pchar):longint;cdecl;external libc_nlm name 'snprintf';
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar; p1,p2:pchar):longint;cdecl;external libc_nlm name 'snprintf';
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar; p1,p2,p3:pchar):longint;cdecl;external libc_nlm name 'snprintf';
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar; p1,p2:longint):longint;cdecl;external libc_nlm name 'snprintf';
function snprintf(__restrict:Pchar; n:size_t; Format:Pchar; p1,p2,p3:longint):longint;cdecl;external libc_nlm name 'snprintf';

{$ifndef DisableArrayOfConst}
function sscanf(__restrict:Pchar; __restrict1:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'sscanf';
{$endif}
function sscanf(__restrict:Pchar; __restrict1:Pchar):longint;cdecl;external libc_nlm name 'sscanf';
function tmpfile:PFILE;cdecl;external libc_nlm name 'tmpfile';
function tmpnam(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'tmpnam';
function ungetc(_para1:longint; _para2:PFILE):longint;cdecl;external libc_nlm name 'ungetc';

function vfprintf(__restrict:PFILE; __restrict1:Pchar; _para3:va_list):longint;cdecl;external libc_nlm name 'vfprintf';
function vfscanf(__restrict:PFILE; __restrict1:Pchar; _para3:va_list):longint;cdecl;external libc_nlm name 'vfscanf';
function vprintf(__restrict:Pchar; _para2:va_list):longint;cdecl;external libc_nlm name 'vprintf';
function vscanf(__restrict:Pchar; _para2:va_list):longint;cdecl;external libc_nlm name 'vscanf';
function vsnprintf(__restrict:Pchar; n:size_t; __restrict1:Pchar; _para4:va_list):longint;cdecl;external libc_nlm name 'vsnprintf';
function vsprintf(__restrict:Pchar; __restrict1:Pchar; _para3:va_list):longint;cdecl;external libc_nlm name 'vsprintf';

function vsscanf(__restrict:Pchar; __restrict1:Pchar; _para3:va_list):longint;cdecl;external libc_nlm name 'vsscanf';
{ functions underlying macro support...  }
function ___stdin:PPFILE;cdecl;external libc_nlm name '___stdin';
function ___stdout:PPFILE;cdecl;external libc_nlm name '___stdout';
function ___stderr:PPFILE;cdecl;external libc_nlm name '___stderr';
function ___cin:PPFILE;cdecl;external libc_nlm name '___cin';
function ___cout:PPFILE;cdecl;external libc_nlm name '___cout';
{ POSIX-defined and other additions...  }

function fdopen(_para1:longint; __restrict:Pchar):PFILE;cdecl;external libc_nlm name 'fdopen';
function fileno(_para1:PFILE):longint;cdecl;external libc_nlm name 'fileno';
procedure flockfile(_para1:PFILE);cdecl;external libc_nlm name 'flockfile';
function ftrylockfile(_para1:PFILE):longint;cdecl;external libc_nlm name 'ftrylockfile';
procedure funlockfile(_para1:PFILE);cdecl;external libc_nlm name 'funlockfile';
function getc_unlocked(_para1:PFILE):longint;cdecl;external libc_nlm name 'getc_unlocked';
function getchar_unlocked:longint;cdecl;external libc_nlm name 'getchar_unlocked';
function getw(_para1:PFILE):longint;cdecl;external libc_nlm name 'getw';
function pclose(stream:PFILE):longint;cdecl;external libc_nlm name 'pclose';
function popen(command:Pchar; mode:Pchar):PFILE;cdecl;external libc_nlm name 'popen';
function putc_unlocked(c:longint; _para2:PFILE):longint;cdecl;external libc_nlm name 'putc_unlocked';
function putchar_unlocked(c:longint):longint;cdecl;external libc_nlm name 'putchar_unlocked';
function tempnam(dirpath:Pchar; prefix:Pchar):Pchar;cdecl;external libc_nlm name 'tempnam';
{ nonstandard (transitional) addtions for 64-bit file I/O...  }
function fgetpos64(_para1:PFILE; _para2:Pfpos64_t):longint;cdecl;external libc_nlm name 'fgetpos64';
function fseek64(fp:PFILE; offset:fpos64_t; whence:longint):longint;cdecl;external libc_nlm name 'fseek64';
function fsetpos64(_para1:PFILE; _para2:Pfpos64_t):longint;cdecl;external libc_nlm name 'fsetpos64';
function ftell64(_para1:PFILE):off64_t;cdecl;external libc_nlm name 'ftell64';
{ hard-wired console I/O support (cannot be redirected)...  }
function cgetc:longint;cdecl;external libc_nlm name 'cgetc';
function cgets(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'cgets';

{$ifndef DisableArrayOfConst}
function cprintf(_para1:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'cprintf';
{$endif}
function cprintf(_para1:Pchar):longint;cdecl;external libc_nlm name 'cprintf';
function cputc(_para1:longint):longint;cdecl;external libc_nlm name 'cputc';
function cputs(_para1:Pchar):longint;cdecl;external libc_nlm name 'cputs';
{$ifndef DisableArrayOfConst}
function cscanf(__restrict:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'cscanf';
{$endif}
function cscanf(__restrict:Pchar):longint;cdecl;external libc_nlm name 'cscanf';
function vcprintf(__restrict:Pchar; _para2:va_list):longint;cdecl;external libc_nlm name 'vcprintf';
function vcscanf(__restrict:Pchar; _para2:va_list):longint;cdecl;external libc_nlm name 'vcscanf';
function vasprintf(__restrict:PPchar; __restrict1:Pchar; _para3:va_list):longint;cdecl;external libc_nlm name 'vasprintf';
{ defined as macros in both C and C++...  }
{ UNIX 98/POSIX.1-2002 defined additions  }


// stdlib.h
{ pshpack1.h }
{ turn on 1-byte packing...  }

  const
     EXIT_FAILURE = -(1);
     EXIT_SUCCESS = 0;
     RAND_MAX = 32767;


type
   Pdiv_t = ^div_t;
   div_t = record
        quot : longint;
        rem : longint;
     end;

   Pldiv_t = ^ldiv_t;
   ldiv_t = record
        quot : longint;
        rem : longint;
     end;

   Plldiv_t = ^lldiv_t;
   lldiv_t = record
        quot : Tint64;
        rem : Tint64;
     end;

(** unsupported pragma#pragma pack()*)

type TCDeclProcedure = procedure; cdecl;

procedure abort;cdecl;external libc_nlm name 'abort';
// function {$ifdef INCLUDED_FROM_SYSTEM}libc_abs{$else}abs{$endif}(_para1:longint):longint;cdecl;external libc_nlm name 'abs';
function atexit(_para1:TCDeclProcedure ):longint;cdecl;external libc_nlm name 'atexit';
function atof(_para1:Pchar):double;cdecl;external libc_nlm name 'atof';
function atoi(_para1:Pchar):longint;cdecl;external libc_nlm name 'atoi';
function atol(_para1:Pchar):longint;cdecl;external libc_nlm name 'atol';
function atoll(_para1:Pchar):Tint64;cdecl;external libc_nlm name 'atoll';

//!! function bsearch(_para1:pointer; _para2:pointer; _para3:size_t; _para4:size_t; _para5:function (_para1:pointer; _para2:pointer):longint):pointer;cdecl;external libc_nlm name 'bsearch';
function calloc(_para1:size_t; _para2:size_t):pointer;cdecl;external libc_nlm name 'calloc';
function __CW_div(_para1:longint; _para2:longint):div_t;cdecl;external libc_nlm name '__CW_div';
procedure libc_exit(status:longint);cdecl;external libc_nlm name 'exit';
procedure free(_para1:pointer);cdecl;external libc_nlm name 'free';
function getenv(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'getenv';
function labs(_para1:longint):longint;cdecl;external libc_nlm name 'labs';
function __CW_ldiv(_para1:longint; _para2:longint):ldiv_t;cdecl;external libc_nlm name '__CW_ldiv';
function llabs(_para1:Tint64):Tint64;cdecl;external libc_nlm name 'llabs';
function __CW_lldiv(_para1:Tint64; _para2:Tint64):lldiv_t;cdecl;external libc_nlm name '__CW_lldiv';
function malloc(_para1:size_t):pointer;cdecl;external libc_nlm name 'malloc';
function mblen(_para1:Pchar; _para2:size_t):longint;cdecl;external libc_nlm name 'mblen';
function mbstowcs(_para1:Pwchar_t; _para2:Pchar; _para3:size_t):size_t;cdecl;external libc_nlm name 'mbstowcs';
function mbtowc(_para1:Pwchar_t; _para2:Pchar; _para3:size_t):longint;cdecl;external libc_nlm name 'mbtowc';

type TQSortFunc = function (_para1:pointer; _para2:pointer):longint; cdecl;
procedure qsort(_para1:pointer; _para2:size_t; _para3:size_t; _para4:TQSortFunc);cdecl;external libc_nlm name 'qsort';
function rand:longint;cdecl;external libc_nlm name 'rand';
function realloc(_para1:pointer; _para2:size_t):pointer;cdecl;external libc_nlm name 'realloc';
procedure srand(_para1:dword);cdecl;external libc_nlm name 'srand';
function strtod(__restrict:Pchar; __restrict1:PPchar):double;cdecl;external libc_nlm name 'strtod';
function strtol(__restrict:Pchar; __restrict1:PPchar; _para3:longint):longint;cdecl;external libc_nlm name 'strtol';
function strtoll(__restrict:Pchar; __restrict1:PPchar; _para3:longint):Tint64;cdecl;external libc_nlm name 'strtoll';
function strtoul(__restrict:Pchar; __restrict1:PPchar; _para3:longint):dword;cdecl;external libc_nlm name 'strtoul';
function strtoull(__restrict:Pchar; __restrict1:PPchar; _para3:longint):Tuint64;cdecl;external libc_nlm name 'strtoull';
function system(_para1:Pchar):longint;cdecl;external libc_nlm name 'system';
function wcstombs(__restrict:Pchar; __restrict1:Pwchar_t; _para3:size_t):size_t;cdecl;external libc_nlm name 'wcstombs';
function wctomb(_para1:Pchar; _para2:wchar_t):longint;cdecl;external libc_nlm name 'wctomb';
function alloca(_para1:size_t):pointer;cdecl;external libc_nlm name 'alloca';
function clearenv:longint;cdecl;external libc_nlm name 'clearenv';
function getcmd(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'getcmd';
function flushenv(_para1:Pchar; _para2:longint):longint;cdecl;external libc_nlm name 'flushenv';
function getbsize(_para1:Plongint; _para2:Plongint):Pchar;cdecl;external libc_nlm name 'getbsize';
function htol(_para1:Pchar):dword;cdecl;external libc_nlm name 'htol';
function itoa(_para1:longint; _para2:Pchar; _para3:longint):Pchar;cdecl;external libc_nlm name 'itoa';
function itoab(_para1:dword; _para2:Pchar):Pchar;cdecl;external libc_nlm name 'itoab';
function ltoa(_para1:longint; _para2:Pchar; _para3:longint):Pchar;cdecl;external libc_nlm name 'ltoa';
function lltoa(_para1:Tint64; _para2:Pchar; _para3:longint):Pchar;cdecl;external libc_nlm name 'lltoa';
function _lrotr(_para1:dword; _para2:dword):dword;cdecl;external libc_nlm name '_lrotr';
function _lrotl(_para1:dword; _para2:dword):dword;cdecl;external libc_nlm name '_lrotl';
function mkdtemp(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'mkdtemp';
function mkstemp(_para1:Pchar):longint;cdecl;external libc_nlm name 'mkstemp';
function mktemp(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'mktemp';
function msize(_para1:pointer):size_t;cdecl;external libc_nlm name 'msize';
function multibyte:longint;cdecl;external libc_nlm name 'multibyte';
function mvalidrange(_para1:pointer; _para2:size_t):longint;cdecl;external libc_nlm name 'mvalidrange';
function nextmb(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'nextmb';
function prevmb(_para1:Pchar):Pchar;cdecl;external libc_nlm name 'prevmb';
function putenv(_para1:Pchar):longint;cdecl;external libc_nlm name 'putenv';
function rand_r(_para1:Pdword):longint;cdecl;external libc_nlm name 'rand_r';
function _rotr(_para1:dword; _para2:dword):dword;cdecl;external libc_nlm name '_rotr';
function _rotl(_para1:dword; _para2:dword):dword;cdecl;external libc_nlm name '_rotl';
function rotl8(_para1:byte; _para2:longint):byte;cdecl;external libc_nlm name 'rotl8';
function rotl16(_para1:word; _para2:longint):word;cdecl;external libc_nlm name 'rotl16';
function rotl32(_para1:dword; _para2:longint):dword;cdecl;external libc_nlm name 'rotl32';
function rotl64(_para1:Tuint64; _para2:longint):Tuint64;cdecl;external libc_nlm name 'rotl64';
function rotr8(_para1:byte; _para2:longint):byte;cdecl;external libc_nlm name 'rotr8';
function rotr16(_para1:word; _para2:longint):word;cdecl;external libc_nlm name 'rotr16';
function rotr32(_para1:dword; _para2:longint):dword;cdecl;external libc_nlm name 'rotr32';
function rotr64(_para1:Tuint64; _para2:longint):Tuint64;cdecl;external libc_nlm name 'rotr64';
function setenv(_para1:Pchar; _para2:Pchar; _para3:longint):longint;cdecl;external libc_nlm name 'setenv';
procedure setkey(_para1:Pchar);cdecl;external libc_nlm name 'setkey';
function stackavail:size_t;cdecl;external libc_nlm name 'stackavail';
function stackbase:pointer;cdecl;external libc_nlm name 'stackbase';
function stackwatermark:size_t;cdecl;external libc_nlm name 'stackwatermark';
function strtoi(_para1:Pchar; _para2:longint):longint;cdecl;external libc_nlm name 'strtoi';
function truncmb(_para1:Pchar; _para2:size_t):Pchar;cdecl;external libc_nlm name 'truncmb';
function ultoa(_para1:dword; _para2:Pchar; _para3:longint):Pchar;cdecl;external libc_nlm name 'ultoa';
function ulltoa(_para1:Tuint64; _para2:Pchar; _para3:longint):Pchar;cdecl;external libc_nlm name 'ulltoa';
function unsetenv(name:Pchar):longint;cdecl;external libc_nlm name 'unsetenv';
function utoa(_para1:dword; _para2:Pchar; _para3:longint):Pchar;cdecl;external libc_nlm name 'utoa';
function valuemb(_para1:Pchar):longint;cdecl;external libc_nlm name 'valuemb';
{ atomic functions...  }
procedure atomic_add(addr:Pdword; value:dword);cdecl;external libc_nlm name 'atomic_add';
function atomic_bts(addr:Pdword; offset:dword):dword;cdecl;external libc_nlm name 'atomic_bts';
function atomic_btr(addr:Pdword; offset:dword):dword;cdecl;external libc_nlm name 'atomic_btr';
procedure atomic_dec(addr:Pdword);cdecl;external libc_nlm name 'atomic_dec';
procedure atomic_inc(addr:Pdword);cdecl;external libc_nlm name 'atomic_inc';
procedure atomic_sub(addr:Pdword; value:dword);cdecl;external libc_nlm name 'atomic_sub';
function atomic_xchg(addr:Pdword; value:dword):dword;cdecl;external libc_nlm name 'atomic_xchg';
procedure atomic_or(addr:Pdword; value:dword);cdecl;external libc_nlm name 'atomic_or';
procedure atomic_xor(addr:Pdword; value:dword);cdecl;external libc_nlm name 'atomic_xor';
procedure atomic_and(addr:Pdword; value:dword);cdecl;external libc_nlm name 'atomic_and';
function atomic_xchgadd(addr:Pdword; value:dword):dword;cdecl;external libc_nlm name 'atomic_xchgadd';
function atomic_cmpxchg(addr:Pdword; cmpvalue:dword; newvalue:dword):dword;cdecl;external libc_nlm name 'atomic_cmpxchg';
procedure atomic64_inc(addr:Puint64);cdecl;external libc_nlm name 'atomic64_inc';
procedure atomic64_dec(addr:Puint64);cdecl;external libc_nlm name 'atomic64_dec';
procedure atomic64_add(addr:Puint64; value:tuint64);cdecl;external libc_nlm name 'atomic64_add';
procedure atomic64_sub(addr:Puint64; value:tuint64);cdecl;external libc_nlm name 'atomic64_sub';
function atomic64_xchg(addr:Puint64; value:tuint64):Tuint64;cdecl;external libc_nlm name 'atomic64_xchg';
function atomic64_xchgadd(addr:Puint64; value:Tuint64):Tuint64;cdecl;external libc_nlm name 'atomic64_xchgadd';
function atomic64_cmpxchg(addr:Puint64; cmpvalue:Tuint64; newvalue:Tuint64):Tuint64;cdecl;external libc_nlm name 'atomic64_cmpxchg';
{ compiler-specific implementations of alloca()...  }
function max(a:longint; b:longint):longint;cdecl;external libc_nlm name 'max';
function min(a:longint; b:longint):longint;cdecl;external libc_nlm name 'min';


// stropts.h
// sys/ioctl.h
// unistd.h
// sys/types.h

type
   Puchar_t = ^uchar_t;
   uchar_t = byte;

   Pushort_t = ^ushort_t;
   ushort_t = word;

   Puint_t = ^uint_t;
   uint_t = dword;

   Pulong_t = ^ulong_t;
   ulong_t = dword;

   Pu_char = ^u_char;
   u_char = byte;

   Pu_short = ^u_short;
   u_short = word;

   Pu_int = ^u_int;
   u_int = dword;

   Pu_long = ^u_long;
   u_long = dword;

   Pcaddr_t = ^caddr_t;
   caddr_t = char;

   Puseconds_t = ^useconds_t;
   useconds_t = Tuint64;

   Psuseconds_t = ^suseconds_t;
   suseconds_t = Tint64;

   Pu_int8_t = ^u_int8_t;
   u_int8_t = byte;

   Pu_int16_t = ^u_int16_t;
   u_int16_t = word;

   Pu_int32_t = ^u_int32_t;
   u_int32_t = dword;

   Pu_int64_t = ^u_int64_t;
   u_int64_t = Tuint64;

function getpid:pid_t;cdecl;external libc_nlm name 'getpid';
function getppid:pid_t;cdecl;external libc_nlm name 'getppid';
function getuid:uid_t;cdecl;external libc_nlm name 'getuid';
function geteuid:uid_t;cdecl;external libc_nlm name 'geteuid';
function getgid:gid_t;cdecl;external libc_nlm name 'getgid';
function getegid:gid_t;cdecl;external libc_nlm name 'getegid';
function setuid(uid:uid_t):longint;cdecl;external libc_nlm name 'setuid';
function setgid(gid:gid_t):longint;cdecl;external libc_nlm name 'setgid';
function getpgrp:pid_t;cdecl;external libc_nlm name 'getpgrp';
function setsid:pid_t;cdecl;external libc_nlm name 'setsid';
function setpgid(pid:pid_t; pgid:pid_t):longint;cdecl;external libc_nlm name 'setpgid';

function FpGetpid:pid_t;cdecl;external libc_nlm name 'getpid';
function FpGetppid:pid_t;cdecl;external libc_nlm name 'getppid';
function FpGetuid:uid_t;cdecl;external libc_nlm name 'getuid';
function FpGeteuid:uid_t;cdecl;external libc_nlm name 'geteuid';
function FpGetgid:gid_t;cdecl;external libc_nlm name 'getgid';
function FpGetegid:gid_t;cdecl;external libc_nlm name 'getegid';
function FpSetuid(uid:uid_t):longint;cdecl;external libc_nlm name 'setuid';
function FpSetgid(gid:gid_t):longint;cdecl;external libc_nlm name 'setgid';
function FpSetpgrp:pid_t;cdecl;external libc_nlm name 'getpgrp';
function FpSetsid:pid_t;cdecl;external libc_nlm name 'setsid';
function FpSetpgid(pid:pid_t; pgid:pid_t):longint;cdecl;external libc_nlm name 'setpgid';


// sys/unistd.h

type
  TFilDes = Array [0..1] of cInt;
  pFilDes = ^TFilDes;

  const
     R_OK = 1;
  { test for write permission                  }
     W_OK = 2;
  { test for execute permission                }
     X_OK = 4;
  { test for existence of file                 }
     F_OK = 8;
  { values for 'whence' in lseek()...                                  }
  { set file pointer to 'offset'               }
    // SEEK_SET = 0;
  { set file pointer to current plus 'offset'  }
     // SEEK_CUR = 1;
  { set file pointer to EOF plus 'offset'      }
     // SEEK_END = 2;
  { old Berkeley names...                                              }
     L_SET = SEEK_SET;
     L_INCR = SEEK_CUR;
     L_XTND = SEEK_END;
  { test using effective ids                   }
     EFF_ONLY_OK = 8;
  { descriptor underlying 'stdin'              }
     STDIN_FILENO = 0;
  { ibid for 'stdout'                          }
     STDOUT_FILENO = 1;
  { ibid for 'stderr'                          }
     STDERR_FILENO = 2;
     _POSIX_THREAD_SAFE_FUNCTIONS = 1;
     _POSIX_THREAD_ATTR_STACKADDR = 1;
     _POSIX_THREAD_ATTR_STACKSIZE = 1;
     _POSIX_MAPPED_FILES = 1;
     _POSIX_MEMLOCK = 1;
     _POSIX_MEMLOCK_RANGE = 1;
     _POSIX_MEMORY_PROTECTION = 1;
     _POSIX_SYNCHRONIZED_IO = 1;
     _POSIX_FSYNC = 1;
     _POSIX_ASYNC_IO = -(1);
     _POSIX_CHOWN_RESTRICTED = -(1);
     _POSIX_NO_TRUNC = -(1);
     _POSIX_PRIO_IO = -(1);
     _POSIX_SYNC_IO = 1;
     _POSIX_VDISABLE = 0;


function access(path:Pchar; mode:longint):longint;cdecl;external libc_nlm name 'access';
function FpAccess(path:Pchar; mode:longint):longint;cdecl;external libc_nlm name 'access';
function alarm(seconds:dword):dword;cdecl;external libc_nlm name 'alarm';
function FpChdir(path:Pchar):longint;cdecl;external libc_nlm name 'chdir';
function {$ifdef INCLUDED_FROM_SYSTEM}libc_chsize{$else}chsize{$endif}(fildes:longint; size:size_t):longint;cdecl;external libc_nlm name 'chsize';
function FpChsize(fildes:longint; size:size_t):longint;cdecl;external libc_nlm name 'chsize';
function FpClose(fildes:longint):longint;cdecl;external libc_nlm name 'close';
function crypt(key:Pchar; salt:Pchar):Pchar;cdecl;external libc_nlm name 'crypt';
function dup(fildes:longint):longint;cdecl;external libc_nlm name 'dup';
function Fpdup(fildes:longint):longint;cdecl;external libc_nlm name 'dup';
function dup2(fildes1:longint; fildes2:longint):longint;cdecl;external libc_nlm name 'dup2';
function Fpdup2(fildes1:longint; fildes2:longint):longint;cdecl;external libc_nlm name 'dup2';
type TArr064char = array [0..63] of char;
procedure encrypt(block:TArr064char; edflag:longint);cdecl;external libc_nlm name 'encrypt';
procedure _exit(status:longint);cdecl;external libc_nlm name '_exit';
procedure FpExit(status:longint);cdecl;external libc_nlm name '_exit';
function fchdir(fildes:longint):longint;cdecl;external libc_nlm name 'fchdir';
function fdatasync(fildes:longint):longint;cdecl;external libc_nlm name 'fdatasync';
function fork:pid_t;cdecl;external libc_nlm name 'fork';
function fsync(fildes:longint):longint;cdecl;external libc_nlm name 'fsync';
function fpathconf(fildes:longint; name:longint):longint;cdecl;external libc_nlm name 'fpathconf';
function ftruncate(fildes:longint; len:off_t):longint;cdecl;external libc_nlm name 'ftruncate';
function getcwd(path:Pchar; len:size_t):Pchar;cdecl;external libc_nlm name 'getcwd';
function gethostid:longint;cdecl;external libc_nlm name 'gethostid';
(* Const before declarator ignored *)

function getopt(argc:longint; argv:array of Pchar; optstr:Pchar):longint;cdecl;external libc_nlm name 'getopt';
{$ifndef DisableArrayOfConst}
function Fpioctl(_para1:longint; _para2:longint; args:array of const):longint;cdecl;external libc_nlm name 'ioctl';
{$endif}
function Fpioctl(_para1:longint; _para2:longint):longint;cdecl;external libc_nlm name 'ioctl';
function Fpisatty(fildes:longint):longint;cdecl;external libc_nlm name 'isatty';
//function lseek(fildes:longint; offset:off_t; whence:longint):off_t;cdecl;external libc_nlm name 'lseek';
function fplseek(fildes:longint; offset:off_t; whence:longint):off_t;cdecl;external libc_nlm name 'lseek';

function pathconf(path:Pchar; name:longint):longint;cdecl;external libc_nlm name 'pathconf';
//!!function pipe(fildes:array[0..1] of longint):longint;cdecl;external libc_nlm name 'pipe';
function FpPipe(var fildes:TFilDes):cInt;cdecl;external libc_nlm name 'pipe';
function pread(fildes:longint; buf:pointer; nbytes:size_t; off:off_t):ssize_t;cdecl;external libc_nlm name 'pread';
function pwrite(fildes:longint; buf:pointer; nbytes:size_t; off:off_t):ssize_t;cdecl;external libc_nlm name 'pwrite';
function FpRead(fildes:longint; buf:pointer; nbytes:size_t):ssize_t;cdecl;external libc_nlm name 'read';
function FpRead(fildes:longint; var buf; nbytes:size_t):ssize_t;cdecl;external libc_nlm name 'read';
function Fprmdir(path:Pchar):longint;cdecl;external libc_nlm name 'rmdir';
procedure sync;cdecl;external libc_nlm name 'sync';
function sysconf(name:longint):longint;cdecl;external libc_nlm name 'sysconf';
function unlink(path:Pchar):longint;cdecl;external libc_nlm name 'unlink';
function FpUnlink(path:Pchar):longint;cdecl;external libc_nlm name 'unlink';
function FpWrite(fildes:longint; buf:pointer; nbytes:size_t):ssize_t;cdecl;external libc_nlm name 'write';
function FpWrite(fildes:longint; var buf; nbytes:size_t):ssize_t;cdecl;external libc_nlm name 'write';
{ appeared in BSD...  }
function brk(endds:pointer):longint;cdecl;external libc_nlm name 'brk';
function getdtablehi:longint;cdecl;external libc_nlm name 'getdtablehi';
function getdtablesize:longint;cdecl;external libc_nlm name 'getdtablesize';
function getpagesize:longint;cdecl;external libc_nlm name 'getpagesize';

function readlink(path:Pchar; buf:Pchar; bufsize:size_t):longint;cdecl;external libc_nlm name 'readlink';
function FpReadlink(path:Pchar; buf:Pchar; bufsize:size_t):longint;cdecl;external libc_nlm name 'readlink';
function sbrk(incr:intptr_t):pointer;cdecl;external libc_nlm name 'sbrk';
{ nonstandard additions (see also fsio.h)...  }
function cancel(t_id:longint):longint;cdecl;external libc_nlm name 'cancel';
function confstr(name:longint; buf:Pchar; len:size_t):size_t;cdecl;external libc_nlm name 'confstr';
function delay(milliseconds:dword):longint;cdecl;external libc_nlm name 'delay';
function _delay(milliseconds:dword):longint;cdecl;external libc_nlm name 'delay';
function sethostid(hostid:longint):longint;cdecl;external libc_nlm name 'sethostid';
function setmode(fildes:longint; oflag:longint):longint;cdecl;external libc_nlm name 'setmode';
function sleep(seconds:dword):dword;cdecl;external libc_nlm name 'sleep';
function FpSleep(seconds:dword):dword;cdecl;external libc_nlm name 'sleep';
function usleep(useconds:useconds_t):longint;cdecl;external libc_nlm name 'usleep';
{ nonstandard (transitional) addtions for 64-bit file I/O...  }
function chsize64(fildes:longint; size:size64_t):longint;cdecl;external libc_nlm name 'chsize64';
function Fpchsize64(fildes:longint; size:size64_t):longint;cdecl;external libc_nlm name 'chsize64';
function ftruncate64(fildes:longint; len:off64_t):longint;cdecl;external libc_nlm name 'ftruncate64';
function Fpftruncate64(fildes:longint; len:off64_t):longint;cdecl;external libc_nlm name 'ftruncate64';
function lseek64(fildes:longint; offset:off64_t; whence:longint):off64_t;cdecl;external libc_nlm name 'lseek64';
function Fplseek64(fildes:longint; offset:off64_t; whence:longint):off64_t;cdecl;external libc_nlm name 'lseek64';
function pread64(fildes:longint; buf:pointer; nbytes:size_t; off:off64_t):ssize_t;cdecl;external libc_nlm name 'pread64';
function pwrite64(fildes:longint; buf:pointer; nbytes:size_t; off:off64_t):ssize_t;cdecl;external libc_nlm name 'pwrite64';
function tell64(fildes:longint):off64_t;cdecl;external libc_nlm name 'tell64';
function Fptell64(fildes:longint):off64_t;cdecl;external libc_nlm name 'tell64';
function ____environ:PPPchar;cdecl;external libc_nlm name '____environ';
function ___optarg:PPchar;cdecl;external libc_nlm name '___optarg';
function ___optind:Plongint;cdecl;external libc_nlm name '___optind';
function ___opterr:Plongint;cdecl;external libc_nlm name '___opterr';
function ___optopt:Plongint;cdecl;external libc_nlm name '___optopt';
function ___optreset:Plongint;cdecl;external libc_nlm name '___optreset';
function want_posix_semantics(timestamp:longint):longint;cdecl;external libc_nlm name 'want_posix_semantics';
{
** Prototype for libraries writing their own start-up and shut-down code. This
** is not an interface, but only a prototype for code furnished by the UNIX-
** style NLM library. The presence of these is noted by the prelude object.
 }
function _init:longint;cdecl;external libc_nlm name '_init';
function _fini:longint;cdecl;external libc_nlm name '_fini';
{ globals for getopt() implementation...  }
{ the prototype for ioctl() is in unistd.h...  }


// sys/byteorder.h
function htonl(_para1:dword):dword;cdecl;external libc_nlm name 'htonl';
function htons(_para1:word):word;cdecl;external libc_nlm name 'htons';
function ntohl(_para1:dword):dword;cdecl;external libc_nlm name 'ntohl';
function ntohs(_para1:word):word;cdecl;external libc_nlm name 'ntohs';


// sys/cdefs.h
// sys/dir.h
// dirent.h
// sys/mode.h

{ POSIX file types...  }
{ POSIX file modes: owner (user) permission...  }
{ POSIX file modes: group permission...  }
{ POSIX file modes: other permission...  }
{ POSIX setuid(), setgid(), and sticky...  }
{ for use with stat(), readdir(), chmod(), mkdir(), etc.  }
{ NetWare-specific additions to the upper half of mode_t...  }
{ values for field 'd_type'...  }

{ pshpack1.h }
{ turn on 1-byte packing...  }

  const
     S_IFMT = $F000;
  { first-in/first-out (FIFO/pipe)            }
     S_IFIFO = $1000;
  { character-special file (tty/console)      }
     S_IFCHR = $2000;
  { directory                                 }
     S_IFDIR = $4000;
  { blocking device (unused)                  }
     S_IFBLK = $6000;
  { regular                                   }
     S_IFREG = $8000;
  { symbolic link (unused)                    }
     S_IFLNK = $A000;
  { Berkeley socket                           }
     S_IFSOCK = $C000;
     S_IRWXU = $01C0;
     S_IRUSR = $0100;
     S_IWUSR = $0080;
     S_IXUSR = $0040;
     S_IREAD = S_IRUSR;
     S_IWRITE = S_IWUSR;
     S_IEXEC = S_IXUSR;
  { POSIX file modes: group permission...  }
     S_IRWXG = $0038;
     S_IRGRP = $0020;
     S_IWGRP = $0010;
     S_IXGRP = $0008;
  { POSIX file modes: other permission...  }
     S_IRWXO = $0007;
     S_IROTH = $0004;
     S_IWOTH = $0002;
     S_IXOTH = $0001;
  { POSIX setuid(), setgid(), and sticky...  }
     S_ISUID = $0800;
     S_ISGID = $0400;
     S_ISVTX = $0200;
  { for use with stat(), readdir(), chmod(), mkdir(), etc.  }
  { NetWare-specific additions to the upper half of mode_t...  }
     M_A_RDONLY          = $00010000;    // read-only entry
     M_A_HIDDEN          = $00020000;    // hidden entry
     M_A_SYSTEM          = $00040000;    // system entry
     M_A_SUBDIR          = $00080000;    // is Subdir
     M_A_ARCH            = $00100000;    // file has been archived
     M_A_SHARE           = $00200000;    // file is shared
     M_A_TRANS           = $00400000;    // file transactions are tracked
     M_A_IMMPURG         = $00800000;    // purge deleted file immediately
     M_A_NORENAM         = $01000000;    // inhibit renaming
     M_A_NODELET         = $02000000;    // inhibit deletion
     M_A_NOCOPY          = $04000000;    // inhibit copying
     M_A_IMMCOMPRESS     = $08000000;    // compress immediately
     M_A_FILE_COMPRESSED = $10000000;    // file is compressed
     M_A_DONT_COMPRESS   = $20000000;    // inhibit compression
     M_A_CANT_COMPRESS   = $40000000;    // file cannot be compressed
     M_A_BITS_SIGNIFICANT= $80000000;    // these M_A_- bits are important

     DT_UNKNOWN = 0;
     DT_TTY = 1;           { console (won't occur) }
     DT_REG = S_IFREG;     { normal file }
     DT_DIR = S_IFDIR;     { subdirectory }
     DT_FIFO = S_IFIFO;    { first-in/first-out (FIFO/pipe) }
     DT_SOCK = S_IFSOCK;   { socket (won't occur) }
     DT_CHR = S_IFCHR;     { character-special file (unused)}
     DT_BLK = S_IFBLK;     { blocking device (unused) }
     DT_LNK = S_IFLNK;     { symbolic or hard link (won't occur) }

type
   Pdirent = ^Tdirent;
   Tdirent = record
        d_userspec : dword;
        d_flags    : dword;
        d_type     : mode_t;
        d_mode     : mode_t;
        d_ino      : ino_t;
        d_size     : off64_t;
        d_spare    : array[0..54] of dword;
        d_pad1     : byte;
        d_pad2     : byte;
        d_pad3     : byte;
        d_namelen  : byte;
        d_name     : array[0..(255 + 1)-1] of char;
     end;
    PPdirent = ^Pdirent;
   //DIR = dirent;
   //PDIR = ^DIR;
   //TDir = Dir;
{ sizeof(struct dirent) == 0x200 (512.)  }

(** unsupported pragma#pragma pack()*)


function closedir(dirp:Pdirent):longint;cdecl;external libc_nlm name 'closedir';
function opendir(pathName:Pchar):Pdirent;cdecl;external libc_nlm name 'opendir';
function readdir(dirp:Pdirent):Pdirent;cdecl;external libc_nlm name 'readdir';
function readdir_r(dirp:Pdirent; entry:Pdirent; result:PPdirent):longint;cdecl;external libc_nlm name 'readdir_r';
procedure rewinddir(dirp:Pdirent);cdecl;external libc_nlm name 'rewinddir';

function Fpclosedir(dirp:Pdirent):longint;cdecl;external libc_nlm name 'closedir';
function Fpopendir(pathName:Pchar):Pdirent;cdecl;external libc_nlm name 'opendir';
function Fpreaddir(dirp:Pdirent):Pdirent;cdecl;external libc_nlm name 'readdir';
function Fpreaddir_r(dirp:Pdirent; entry:Pdirent; result:PPdirent):longint;cdecl;external libc_nlm name 'readdir_r';
procedure Fprewinddir(dirp:Pdirent);cdecl;external libc_nlm name 'rewinddir';

// sys/file.h
// fcntl.h

{ 'cmd' values for fcntl()...                                               }
const
 F_GETFL      = 1;        // get file status flags
 F_SETFL      = 2;        // set file status flags
 F_DUPFD      = 3;        // duplicate file descriptor
 F_GETFD      = 4;        // get file descriptor flags
 F_SETFD      = 5;        // set file descriptor flags
 F_SETLK      = 6;        // set record locking info
 F_SETLK64    = 16;       // set record locking info (64-bit)
 F_GETLK      = 7;        // get record locking info
 F_GETLK64    = 17;       // get record locking info (64-bit)
 F_SETLKW     = 8;        // get record locking info; wait if blocked
 F_SETLKW64   = 18;       // get record locking info (64-bit)
 F_CLOEXEC    = 9;        // close on execute

// values for 'l_type' field of 'struct flock'...
 F_RDLCK      = 1;        // shared or read lock
 F_WRLCK      = 2;        // exclusive or write lock
 F_UNLCK      = 3;        // unlock

// values for 'oflag' in open()...
 O_RDONLY     =$00000000; // open for read only
 O_WRONLY     =$00000001; // open for write only
 O_RDWR       =$00000002; // open for read and write
 O_ACCMODE    =$00000003; // access flags mask
 O_reserved1  =$00000004; // reserved
 O_reserved2  =$00000008; // reserved
 O_APPEND     =$00000010; // writes done at end of file
 O_CREAT      =$00000020; // create new file
 O_TRUNC      =$00000040; // truncate existing file
 O_EXCL       =$00000080; // exclusive open
 O_NOCTTY     =$00000100; // no controlling terminal--unsupported
 O_BINARY     =$00000200; // binary file--all files
 O_NDELAY     =$00000400; // nonblocking flag
 O_reserved3  =$00000800; // reserved
 O_SYNC       =$00001000; // synchronized I/O file integrity
 O_DSYNC      =$00002000; // synchronized I/O data integrity
 O_RSYNC      =$00004000; // synchronized read I/O
 O_NONBLOCK   = O_NDELAY; // alias
 FD_CLOEXEC   =$00008000; // parent closes after call to process()
 O_UPDATE     =$00010000; // keep legacy files updated
 O_FIFO       =$00100000; // opening one end of a FIFO [non-standard]

// value for third argument when 'cmd' is F_SETFL in fcntl()...
 FNDELAY      = O_NDELAY;   // fcntl() non-blocking I/O

// 'shflag' values for sopen()...
 SH_DENYRW    = $00000010; // deny read/write mode
 SH_DENYWR    = $00000020; // deny write mode
 SH_DENYRD    = $00000030; // deny read mode
 SH_DENYNO    = $00000040; // deny none mode

type
   Pflock = ^flock;
   flock = record
        l_pid : pid_t;                { process ID of owner, get with F_GETLK       }
        l_tid : pid_t;                { thread ID of owner, get with F_GETLK        }
        l_type : smallint;            { F_RDLCK or F_WRLCK                          }
        l_whence : smallint;          { flag for starting offset                    }
        ________spare : longint;      { unused                                      }
        ________reserved1 : longint;  { padding to resemble 64-bit structure        }
        l_start : off_t;              { relative offset in bytes                    }
        ________reserved2 : longint;  { padding to resemble 64-bit structure        }
        l_len : off_t;                { size; if 0, then until EOF                  }
     end;

   Pflock64 = ^flock64;
   flock64 = record                   { with F_GETLK64, F_SETLK64, FSETLKW64        }
        l_pid : pid_t;                { process ID of owner, get with F_GETLK       }
        l_tid : pid_t;                { thread ID of owner, get with F_GETLK        }
        l_type : smallint;            { F_RDLCK or F_WRLCK                          }
        l_whence : smallint;          { flag for starting offset                    }
        ________spare : longint;      { unused                                      }
        l_start : off64_t;            { relative offset in bytes                    }
        l_len : off64_t;              { size; if 0, then until EOF                  }
     end;

function creat(path:Pchar; mode:mode_t):longint;cdecl;external libc_nlm name 'creat';
{$ifndef DisableArrayOfConst}
function fcntl(fildes:longint; cmd:longint; args:array of const):longint;cdecl;external libc_nlm name 'fcntl';
{$endif}
function fcntl(fildes:longint; cmd:longint):longint;cdecl;external libc_nlm name 'fcntl';
{$ifndef DisableArrayOfConst}
function open(path:Pchar; oflag:longint; args:array of const):longint;cdecl;external libc_nlm name 'open';
{$endif}
function open(path:Pchar; oflag:longint):longint;cdecl;external libc_nlm name 'open';
function open(path:Pchar; oflag,mode:longint):longint;cdecl;external libc_nlm name 'open';
function FpOpen(path:Pchar; oflag:longint):longint;cdecl;external libc_nlm name 'open';
function FpOpen(path:Pchar; oflag,mode:longint):longint;cdecl;external libc_nlm name 'open';

function pipe_open(path:Pchar; oflag:longint):longint;cdecl;external libc_nlm name 'pipe_open';
function pipe_open(path:Pchar; oflag,mode:longint):longint;cdecl;external libc_nlm name 'pipe_open';
{$ifndef DisableArrayOfConst}
function sopen(path:Pchar; oflag:longint; shflag:longint; args:array of const):longint;cdecl;external libc_nlm name 'sopen';
{$endif}
function sopen(path:Pchar; oflag:longint; shflag:longint):longint;cdecl;external libc_nlm name 'sopen';


// sys/filio.h
{ defines for ioctl()...  }
// sys/ipc.h

{ mode bits...  }
{ keys...  }
{ control commands for semctl() and shmctl()...  }
type
   Pkey_t = ^key_t;
   key_t = longint;
   Pipc_perm = ^ipc_perm;
   ipc_perm = record
        uid  : uid_t;    // owner
        gid  : gid_t;    // owner
        cuid : uid_t;    // creator
        cgid : gid_t;    // creator
        mode : mode_t;   // read write permission
     end;



function ftok(path:Pchar; id:longint):key_t;cdecl;external libc_nlm name 'ftok';


// sys/mman.h

{ return value of mmap() in case of error...  }
{ 'flags' values for mlockall()...  }
{ 'protection' values for mmap()...  }
{ 'flags' values for mmap()...  }
{ 'flags' values for msync()...  }
{ 'advice' to madvice()...  }
{ effectual dummies that will never do anything...  }

function mlock(addr:pointer; len:size_t):longint;cdecl;external libc_nlm name 'mlock';
function mlockall(flags:longint):longint;cdecl;external libc_nlm name 'mlockall';
function munlock(addr:pointer; len:size_t):longint;cdecl;external libc_nlm name 'munlock';
function munlockall:longint;cdecl;external libc_nlm name 'munlockall';
function mprotect(addr:pointer; len:size_t; prot:longint):longint;cdecl;external libc_nlm name 'mprotect';
function madvise(addr:pointer; len:size_t; advice:longint):longint;cdecl;external libc_nlm name 'madvise';



// sys/param.h
// sys/stat.h
// time.h
type

   Pclock_t = ^clock_t;
   clock_t = dword;
   TClock  = clock_t;

   Ptime_t = ^time_t;
   time_t = longint;
   Ttime_t  = time_t;

{ turn on 1-byte packing...  }

type
   Ptm = ^Ttm;
   Ttm = record                // ANSI/ISO 'broken-down' time
        tm_sec   : longint;    // seconds after the minute [0..59]
        tm_min   : longint;    // minutes after the hour [0..59]
        tm_hour  : longint;    // hours since midnight [0..23]
        tm_mday  : longint;    // days of the month [1..31]
        tm_mon   : longint;    // months since January [0..11]
        tm_year  : longint;    // years since 1900 [0..ì]
        tm_wday  : longint;    // days since Sunday [0..6]
        tm_yday  : longint;    // days since first of January [0..365]
        tm_isdst: longint;    // on summer time (-1 unknown, 0 no, !0 yes)
     end;

   Ptimespec = ^Ttimespec;
   Ttimespec = record        // time expressed in seconds and nanoseconds
        tv_sec  : time_t;    // seconds
        tv_nsec : longint;   // nanoseconds
     end;
   timespec_t = Ttimespec;
   Ptimespec_t = ^timespec_t;
   timestrc_t = Ttimespec;
   Ptimestrc_t = ^timestrc_t;

   Pitimerspec = ^Titimerspec;
   Titimerspec = record
        it_interval : Ttimespec;  // timer period
        it_value    : Ttimespec;  // expiration
     end;
   itimerspec_t = Titimerspec;
   Pitimerspec_t = ^itimerspec_t;
{ DOS 'broken-down' time                     }
{ two-second increments only                 }
{ 0-59                                       }
{ 0-23                                       }
{ 1-31                                       }
{ 1-12                                       }
{ years since 1980 (limit: 0-119)            }
   Pdos_tm = ^Tdos_tm;
   Tdos_tm = record
        flag0 : longint;
     end;


const
   bm_dos_tm_bisecond = $1F;
   bp_dos_tm_bisecond = 0;
   bm_dos_tm_minute = $7E0;
   bp_dos_tm_minute = 5;
   bm_dos_tm_hour = $F800;
   bp_dos_tm_hour = 11;
   bm_dos_tm_day = $1F0000;
   bp_dos_tm_day = 16;
   bm_dos_tm_month = $1E00000;
   bp_dos_tm_month = 21;
   bm_dos_tm_year = $FE000000;
   bp_dos_tm_year = 25;

{$ifndef INCLUDED_FROM_SYSTEM}
function bisecond(var a : Tdos_tm) : word;
procedure set_bisecond(var a : Tdos_tm; __bisecond : word);
function minute(var a : Tdos_tm) : word;
procedure set_minute(var a : Tdos_tm; __minute : word);
function hour(var a : Tdos_tm) : word;
procedure set_hour(var a : Tdos_tm; __hour : word);
function day(var a : Tdos_tm) : word;
procedure set_day(var a : Tdos_tm; __day : word);
function month(var a : Tdos_tm) : word;
procedure set_month(var a : Tdos_tm; __month : word);
function year(var a : Tdos_tm) : word;
procedure set_year(var a : Tdos_tm; __year : word);
{$endif}


type
   Pdos_d = ^Tdos_d;
   Tdos_d = packed record   // DOS date separated from time
        time : word;        // time fields
        date : word;        // date fields
     end;

{ utility for passing DOS time               }
{ scalar for passing as argument             }
{ separated-out DOS time and date            }
{ broken-down DOS time and date              }
   Pdos_tm_u = ^Tdos_tm_u;
   Tdos_tm_u = record
       case longint of
          0 : ( long_dt : dword );
          1 : ( struct_dt : Tdos_d );
          2 : ( struct_tm : Tdos_tm );
       end;

(** unsupported pragma#pragma pack()*)


function asctime(localtime:Ptm):Pchar;cdecl;external libc_nlm name 'asctime';
function asctime(var localtime:Ttm):Pchar;cdecl;external libc_nlm name 'asctime';
function clock:clock_t;cdecl;external libc_nlm name 'clock';
function ctime(calendar:Ptime_t):Pchar;cdecl;external libc_nlm name 'ctime';
function ctime(var calendar:Ttime_t):Pchar;cdecl;external libc_nlm name 'ctime';
function difftime(t1, t2:Ttime_t):double;cdecl;external libc_nlm name 'difftime';
function gmtime(calendar:Ptime_t):Ptm;cdecl;external libc_nlm name 'gmtime';
function gmtime(var calendar:Ttime_t):Ptm;cdecl;external libc_nlm name 'gmtime';
function localtime(calendar:Ptime_t):Ptm;cdecl;external libc_nlm name 'localtime';
function localtime(var calendar:Ttime_t):Ptm;cdecl;external libc_nlm name 'localtime';
function mktime(localtime:Ptm):time_t;cdecl;external libc_nlm name 'mktime';
function mktime(var localtime:Ttm):time_t;cdecl;external libc_nlm name 'mktime';

//size_t strftime ( char * __restrict s, size_t, const char * __restrict format,
//size_t strftime ( char * __restrict s, size_t, const char * __restrict format,
//                      const tm * __restrict localtime );

function time(calendar:Ptime_t):time_t;cdecl;external libc_nlm name 'time';
function time(var calendar:Ttime_t):time_t;cdecl;external libc_nlm name 'time';
function ___clocks_per_sec:longint;cdecl;external libc_nlm name '___clocks_per_sec';
{ POSIX data and helper functions...  }
function ___daylight:Plongint;cdecl;external libc_nlm name '___daylight';
function ___daylightOnOff:Plongint;cdecl;external libc_nlm name '___daylightOnOff';
function ___daylightOffset:Ptime_t;cdecl;external libc_nlm name '___daylightOffset';
function ___timezone:Ptime_t;cdecl;external libc_nlm name '___timezone';
function ___tzname:PPchar;cdecl;external libc_nlm name '___tzname';
function __isleap(year:longint):longint;cdecl;external libc_nlm name '__isleap';
procedure tzset;cdecl;external libc_nlm name 'tzset';
{ POSIX-defined reentrant additions...  }

function asctime_r(localtime:Ptm; timestr:Pchar):Pchar;cdecl;external libc_nlm name 'asctime_r';
function asctime_r(var localtime:Ttm; timestr:Pchar):Pchar;cdecl;external libc_nlm name 'asctime_r';
function ctime_r(calendar:Ptime_t; timestr:Pchar):Pchar;cdecl;external libc_nlm name 'ctime_r';
function ctime_r(var calendar:Ttime_t; timestr:Pchar):Pchar;cdecl;external libc_nlm name 'ctime_r';
function gmtime_r(calendar:Ptime_t; localtime:Ptm):Ptm;cdecl;external libc_nlm name 'gmtime_r';
function gmtime_r(var calendar:Ttime_t; localtime:Ptm):Ptm;cdecl;external libc_nlm name 'gmtime_r';
function localtime_r(calendar:Ptime_t; localtime:Ptm):Ptm;cdecl;external libc_nlm name 'localtime_r';
function localtime_r(var calendar:Ttime_t; var localtime:Ttm):Ptm;cdecl;external libc_nlm name 'localtime_r';
{ Single UNIX Specification additions...  }
function nanosleep(rqtp, rmtp:Ptimespec):longint;cdecl;external libc_nlm name 'nanosleep';
function nanosleep(var rqtp, rmtp:Ttimespec):longint;cdecl;external libc_nlm name 'nanosleep';
{ Novell-defined additions...  }
function ltime(calendar:Ptime_t):time_t;cdecl;external libc_nlm name 'ltime';
function ltime(var calendar:Ttime_t):time_t;cdecl;external libc_nlm name 'ltime';
function mkgmtime(gmtime:Ptm):time_t;cdecl;external libc_nlm name 'mkgmtime';
function mkgmtime(var gmtime:Ttm):time_t;cdecl;external libc_nlm name 'mkgmtime';
function dos2calendar(dostime:Tdos_d):time_t;cdecl;external libc_nlm name 'dos2calendar';
function calendar2dos(calendar:time_t):Tdos_d;cdecl;external libc_nlm name 'calendar2dos';


// sys/time.h
// sys/timeval.h
// sys/times.h

type
   Ptms = ^Ttms;
   Ttms = record                        { describes CPU time used by process, children  }
        tms_utime  : clock_t;           { user CPU time }
        tms_stime  : clock_t;           { system CPU time (identical to 'tms_utime')    }
        tms_cutime : clock_t;           { unimplemented }
        tms_cstime : clock_t;           { unimplemented }
     end;

   Ptimezone = ^Ttimezone;
   TTimezone = record                   { returned by gettimeofday() }
        tz_secondswest : time_t;        { seconds west of UTC }
        tz_minuteswest : longint;       { minutes west of UTC (GMT) }
        tz_dsttime     : longint;       { nonzero if DST is ever in effect }
     end;
   Timezone = TTimezone;

{
** Normally this functions fills struct tms with several time values. Most of
** the time it is very important to have only the return value, that is the
** realtime that has been elapsed.
 }

function times(__buffer:Ptms):clock_t;cdecl;external libc_nlm name 'times';
function times(var __buffer:Ttms):clock_t;cdecl;external libc_nlm name 'times';
{ turn on 1-byte packing...  }

{ this structure is returned by gettimeofday() and used in select()...  }
type
   Ptimeval = ^Ttimeval;
   Ttimeval = record
        tv_sec  : longint;
        tv_usec : longint;
     end;
   Timeval = TTimeval;

(** unsupported pragma#pragma pack()*)
{ operations on struct timeval; note timercmp() does not work for >= or <=  }

function gettimeofday(tp:Ptimeval; tpz:Ptimezone):longint;cdecl;external libc_nlm name 'gettimeofday';
function Fpgettimeofday(tp:Ptimeval; tpz:Ptimezone):longint;cdecl;external libc_nlm name 'gettimeofday';
function settimeofday(tp:Ptimeval; tpz:Ptimezone):longint;cdecl;external libc_nlm name 'settimeofday';
function gettimeofday(var tp:Ttimeval; var tpz:Ttimezone):longint;cdecl;external libc_nlm name 'gettimeofday';
function settimeofday(var tp:Ttimeval; var tpz:Ttimezone):longint;cdecl;external libc_nlm name 'settimeofday';
function Fpgettimeofday(var tp:Ttimeval; var tpz:Ttimezone):longint;cdecl;external libc_nlm name 'gettimeofday';
function Fpsettimeofday(var tp:Ttimeval; var tpz:Ttimezone):longint;cdecl;external libc_nlm name 'settimeofday';

{ turn on 1-byte packing...  }
type
   Pstat = ^Tstat;
   Tstat = record
        st_userspec: dword;                         // untouched by stat()
        st_flags   : dword;                         // flags for this entry
        st_mode    : mode_t;                        // emulated file mode
        st_spare1  : dword;
        st_gen     : Tuint64;                       // generation number of inode
        st_ino     : ino_t;                         // directory entry number
        st_dev     : dev_t;                         // volume number
        st_rdev    : dev_t;                         // device type (always 0)
        st_size    : off64_t;                       // total file size
        st_spare2  : Tuint64;
        st_blocks  : blkcnt_t;                      // count of blocks allocated to file
        st_blksize : blksize_t;                     // block size for allocation--files only
        st_nlink   : nlink_t;                       // count of hard links (always 1)
        st_spare3  : array[0..2] of dword;
        st_uid     : uid_t;                         // owner (object) identity
        st_gid     : gid_t;                         // group-id (always 0)
        st_bid     : uid_t;                         // identity of last archiver
        st_mid     : uid_t;                         // identity of last updator
        st_atim    : timespec_t;                    // last access date--files only
        st_mtim    : timespec_t;                    // last modify date and time
        st_ctim    : timespec_t;                    // last file attributes modification
        st_btim    : timespec_t;                    // last archived date and time
                                                    // NOT returned by stat() or fstat()...
        st_rights  : dword;                         // NetWare rights
        st_spare4  : array[0..2] of dword;
        st_name    : array[0..(255 + 1)-1] of char; // object name as if from readdir()
        st_spare5  : array[0..19] of dword;
     end;

{ sizeof(struct stat) == 0x200 (512.)      }

(** unsupported pragma#pragma pack()*)


function Fpchmod(path:Pchar; mode:mode_t):longint;cdecl;external libc_nlm name 'chmod';
function Fpfchmod(fildes:longint; mode:mode_t):longint;cdecl;external libc_nlm name 'fchmod';
function Fpfstat(fildes:longint; buf:Pstat):longint;cdecl;external libc_nlm name 'fstat';
function Fpfstat(fildes:longint; var buf:Tstat):longint;cdecl;external libc_nlm name 'fstat';
function Fplstat(path:Pchar; buf:Pstat):longint;cdecl;external libc_nlm name 'lstat';
function Fplstat(path:Pchar; var buf:Tstat):longint;cdecl;external libc_nlm name 'lstat';
function Fpmkdir(pathname:Pchar; mode:mode_t):longint;cdecl;external libc_nlm name 'mkdir';
function Fpmkfifo(pathname:Pchar; mode:mode_t):longint;cdecl;external libc_nlm name 'mkfifo';
function Fpmknod(path:Pchar; mode:mode_t; dev:dev_t):longint;cdecl;external libc_nlm name 'mknod';
function Fprealname(pathname:Pchar; name:Pchar):longint;cdecl;external libc_nlm name 'realname';
function Fpstat(path:Pchar; buf:Pstat):longint;cdecl;external libc_nlm name 'stat';
function Fpstat(path:Pchar; var buf:Tstat):longint;cdecl;external libc_nlm name 'stat';
function Fpumask(cmask:mode_t):mode_t;cdecl;external libc_nlm name 'umask';
{
** The following value is not really correct, but it is a value that has been
** used for a long time seems to be usable. Normally, NOFILE should not be used
** anyway.
 }
{ bit map related macros...   }
{ macros for counting and rounding...  }
{ supplementary macros for min/max...   }
{ unit of sys/stat.h `st_blocks'...   }


// sys/select.h

{
** Including file may make this bigger or smaller as long as the FD_- macros
** are faithfully used.
 }
type

   Pfd_set = ^Tfd_set;
   Tfd_set = record
        fd_count : longint;
        fd_array : array[0..63] of longint;
     end;
{ heterogeneous select calls (socket+pipe) unsupported on NetWare...  }


function Fppipe_select(nfds:longint; readfds, writefds, exceptfds:Pfd_set; timeout:Ptimeval):longint;cdecl;external libc_nlm name 'pipe_select';
function Fppipe_select(nfds:longint; var readfds, writefds, exceptfds:Tfd_set; var timeout:Ttimeval):longint;cdecl;external libc_nlm name 'pipe_select';

//function select(nfds:longint; readfds:Pfd_set; writefds:Pfd_set; exceptfds:Pfd_set; timeout:Ptimeval):longint;cdecl;external libc_nlm name 'select';
//function select(nfds:longint; var readfds, writefds, exceptfds:Tfd_set; var timeout:Ttimeval):longint;cdecl;external libc_nlm name 'select';
function FpSelect(nfds:longint; readfds:Pfd_set; writefds:Pfd_set; exceptfds:Pfd_set; timeout:Ptimeval):longint;cdecl;external libc_nlm name 'select';
function FpSelect(nfds:longint; var readfds, writefds, exceptfds:Tfd_set; var timeout:Ttimeval):longint;cdecl;external libc_nlm name 'select';
function ___fd_isset(fd:longint; _set:Pfd_set):longint;cdecl;external libc_nlm name '___fd_isset';
function ___fd_isset(fd:longint; var _set:Tfd_set):longint;cdecl;external libc_nlm name '___fd_isset';


// sys/sem.h

{ semctl() command definitions...  }
{ semaphore text map address        }
{ pid of last operation             }
{ count awaiting (semval > cval)    }
{ count awaiting (semval == 0)      }
type
   Psem = ^Tsem;
   Tsem = record
        semval  : ushort_t;
        semadj  : ushort_t;
        sempid  : pid_t;
        semncnt : ushort_t;
        semzcnt : ushort_t;
        semptr  : pointer;       // semaphore on which this is based
     end;
   TSemaphore = Tsem;
   PSemaphore = Psem;

   Psemid_ds = ^Tsemid_ds;
   Tsemid_ds = record
        sem_perm    : ipc_perm;
        sem_base    : Psem;       // pointer to first semaphore in set
        sem_nsems   : word;       // number of semaphores in set
        sem_otime   : time_t;     // last semop time
        sem_ctime   : time_t;     // last change time
        sem_ptr     : pointer;    // actual underlying semaphore
        sem_realkey : longint;    // 'real' semaphore key
     end;

   Psembuf = ^Tsembuf;
   Tsembuf = record
        sem_num : word;           // Number
        sem_op  : smallint;       // operation
        sem_flg : smallint;       // flags
     end;

   Psemun = ^Tsemun;
   Tsemun = record
       case longint of
          0 : ( val : longint );
          1 : ( buf : Psemid_ds );
          2 : ( _array : Pword );
       end;

{$ifndef DisableArrayOfConst}
function semctl(semid:longint; semnum:longint; cmd:longint; args:array of const):longint;cdecl;external libc_nlm name 'semctl';
{$endif}
function semctl(semid:longint; semnum:longint; cmd:longint):longint;cdecl;external libc_nlm name 'semctl';
function semget(key:key_t; nsems:longint; semflag:longint):longint;cdecl;external libc_nlm name 'semget';
function semop(semid:longint; sops:Psembuf; nsops:size_t):longint;cdecl;external libc_nlm name 'semop';
function semop(semid:longint; var sops:Tsembuf; nsops:size_t):longint;cdecl;external libc_nlm name 'semop';


// sys/sendfile.h

function sendfile(out_fd,in_fd:longint; offset:Poff_t; count:size_t):ssize_t;cdecl;external libc_nlm name 'sendfile';
function sendfile64(out_fd,in_fd:longint; offset:Poff64_t; count:size_t):ssize_t;cdecl;external libc_nlm name 'sendfile64';


// sys/shm.h

type

   Pshmatt_t = ^shmatt_t;
   shmatt_t = dword;
{ turn on 1-byte packing...  }

{ size of segment in bytes               }
{ process ID of last shared operation    }
{ process ID of creator                  }
{ number of current attaches             }
{ time of last shmat()                   }
{ time of last shmdt()                   }
{ time of last change by shmctl()        }
{ operation permission structure         }
type
   Pshmid_ds = ^shmid_ds;
   shmid_ds = record
        shm_segsz : size_t;
        shm_lpid : pid_t;
        shm_cpid : pid_t;
        shm_nattch : shmatt_t;
        shm_atime : time_t;
        shm_dtime : time_t;
        shm_ctime : time_t;
        shm_spare1 : longint;
        shm_perm : ipc_perm;
        shm_spare2 : array[0..2] of longint;
     end;

(** unsupported pragma#pragma pack()*)


function shmat(shmid:longint; shmaddr:pointer; shmflag:longint):pointer;cdecl;external libc_nlm name 'shmat';
function shmctl(shmid:longint; cmd:longint; buf:Pshmid_ds):longint;cdecl;external libc_nlm name 'shmctl';

function shmdt(shmaddr:pointer):longint;cdecl;external libc_nlm name 'shmdt';
function shmget(key:key_t; size:size_t; shmflag:longint):longint;cdecl;external libc_nlm name 'shmget';

// signal.h

    const
       SIGABRT = 1;
       SIGFPE = 2;
       SIGILL = 3;
       SIGINT = 4;
       SIGSEGV = 5;
       SIGTERM = 6;
       SIGPOLL = 7;
    { currently unimplemented POSIX-mandated signals  }
       SIGKILL  = 11;
       SIGSPARE = 12;
       SIGALRM  = 13;
       SIGCHILD = 14;
       SIGCHLD  = SIGCHILD;
       SIGCONT  = 15;
       SIGHUP   = 16;
       SIGPIPE  = 17;
       SIGQUIT  = 18;
       SIGSTOP  = 19;
       SIGTSTP  = 20;
       SIGTTIN  = 21;
       SIGTTOU  = 22;
       SIGUSR1  = 23;
       SIGUSR2  = 24;
       SIGUSR3  = 25;
       SIGUSR4  = 26;
       SIGUSR5  = 27;
       SIGUSR6  = 28;
       SIGUSR7  = 29;
    { Novell-defined signals  }
       SIG_FINI         = 30;
       SIG_LOCALECHANGE = 31;
       NSIG             = 32;
       SIG_BLOCK        = $00000000;
       SIG_UNBLOCK      = $00000001;
       SIG_SETMASK      = $FFFFFFFF;
//       SIGEMPTYSET      = $0000000000000000;
       SIGFULLSET       = $FFFFFFFFFFFFFFFF;


type
   Psig_atomic_t = ^sig_atomic_t;
   sig_atomic_t = longint;

   Psigset_t = ^sigset_t;
   sigset_t = Tuint64;
{ flags for sa_flags in struct sigaction  }
   Psigaction = ^sigaction;
   sigaction = record
        sa_handler : procedure (_para1:longint);cdecl;
        sa_mask    : sigset_t;
        sa_flags   : longint;
     end;

type TCDeclProc1LIntPara = procedure (_para1:longint); cdecl;
function Fpraise(_para1:longint):longint;cdecl;external libc_nlm name 'raise';
function Fpsignal(sig:longint; func:TCDeclProc1LIntPara):TCDeclProc1LIntPara;cdecl;external libc_nlm name 'signal';
function Fpsigwait(_set:Psigset_t; sig:Plongint):longint;cdecl;external libc_nlm name 'sigwait';
{ signal vector functions...  }

//!! function sigaction(sig:longint; act:Psigaction; oact:Psigaction):longint;cdecl;external libc_nlm name 'sigaction';

function Fpsigaddset(_para1:Psigset_t; _para2:longint):longint;cdecl;external libc_nlm name 'sigaddset';
function Fpsigdelset(_para1:Psigset_t; _para2:longint):longint;cdecl;external libc_nlm name 'sigdelset';
function Fpsigismember(_para1:Psigset_t; _para2:longint):longint;cdecl;external libc_nlm name 'sigismember';
function Fpsigfillset(_para1:Psigset_t):longint;cdecl;external libc_nlm name 'sigfillset';
function Fpsigemptyset(_para1:Psigset_t):longint;cdecl;external libc_nlm name 'sigemptyset';
function Fpsigpending(_set:Psigset_t):longint;cdecl;external libc_nlm name 'sigpending';
function Fpsigsuspend(mask:Psigset_t):longint;cdecl;external libc_nlm name 'sigsuspend';
function Fpsigprocmask(how:longint; act:Psigset_t; oldact:Psigset_t):longint;cdecl;external libc_nlm name 'sigprocmask';
function Fpkill(pid:pid_t; sig:longint):longint;cdecl;external libc_nlm name 'kill';


// sys/socket.h
// sys/uio.h
{ turn on 1-byte packing...  }

type
   Piovec = ^iovec;
   iovec = record
        iov_base : caddr_t;
        iov_len : longint;
     end;
   iovec_t = iovec;
   Piovec_t = ^iovec_t;

(** unsupported pragma#pragma pack()*)


function Fpreadv(fildes:longint; iov:Piovec; iovcnt:longint):ssize_t;cdecl;external libc_nlm name 'readv';
function Fpwritev(fildes:longint; iov:Piovec; iovcnt:longint):ssize_t;cdecl;external libc_nlm name 'writev';
{ socket types...  }
{ option flags per-socket...  }
{ additional options, not kept in so_options...  }
{ additional option to be used with level IPPROTO_TCP...  }
{ level number for get/setsockopt() to apply to socket itself...  }
{ address families  }
{ protocol families, same as address families for now...  }
{ values for shutdown() 'how'...  }

{
** Note:
**
** The Novell NDK headers for NKS/LibC contain structures that are explicitly
** hand-packed for best use on the platform in question (usually IA32). To
** avoid the impredictability encountered when compiling with different
** compilers, these headers rely on 1-byte packing.
 }
{ this header sets packing to 1 for different compilers  }
{ save off the previous packing directive in a compiler specific way...  }
{ turn on 1-byte packing...  }

{ structure used for manipulating linger option...  }
{ option on/off                          }
{ linger time                            }
type
   Plinger = ^linger;
   linger = record
        l_onoff : longint;
        l_linger : longint;
     end;

{ structure used to define addresses for bind(), connect(), etc...  }

   Psa_family_t = ^sa_family_t;
   sa_family_t = word;
{ address family                         }
{ up to 14 bytes of direct address       }
   Psockaddr = ^sockaddr;
   sockaddr = record
        sa_family : sa_family_t;
        sa_data : array[0..13] of char;
     end;

{ used by kernel to pass protocol info.  }
{    in raw sockets                      }
{ address family                         }
{ protocol                               }
   Psockproto = ^sockproto;
   sockproto = packed record
        sp_family : word;
        sp_protocol : word;
     end;


   Psocklen_t = ^socklen_t;
   socklen_t = dword;
{
** SUS' and BSD 4.4 message passing. struct msghdr has an additional field
** (msg_flags) and slightly different fieldnames over what we used to
** promote. msg_accrights(len) is done differently.
 }
{ optional address                       }
{ size of address                        }
{ scatter/gather array                   }
{ count of elements in msg_iov           }
{ access rights sent/received            }
   Pmsghdr = ^msghdr;
   msghdr = record
        msg_name : caddr_t;
        msg_namelen : socklen_t;
        msg_iov : Piovec;
        msg_iovlen : longint;
        msg_accrights : caddr_t;
        msg_accrightslen : socklen_t;
        msg_flags : longint;
     end;

{
** POSIX 1003.1g: Ancillary data object information consisting of a sequence
** of pairs of (cmsghdr, cmsg_data[1]).
 }
{ data byte count including header       }
{ originating protocol                   }
{ protocol-specific type                 }
   Pcmsghdr = ^cmsghdr;
   cmsghdr = record
        cmsg_len : socklen_t;
        cmsg_level : longint;
        cmsg_type : longint;
     end;


(** unsupported pragma#pragma pack()*)

function Fpaccept(s:longint; addr:Psockaddr; len:Psize_t):longint;cdecl;external libc_nlm name 'accept';
function Fpbind(s:longint; addr:Psockaddr; _para3:size_t):longint;cdecl;external libc_nlm name 'bind';
function Fpconnect(s:longint; addr:Psockaddr; len:size_t):longint;cdecl;external libc_nlm name 'connect';
function Fpgetpeername(s:longint; addr:Psockaddr; len:Psize_t):longint;cdecl;external libc_nlm name 'getpeername';
function Fpgetsockname(s:longint; addr:Psockaddr; len:Psize_t):longint;cdecl;external libc_nlm name 'getsockname';
function Fpgetsockopt(s:longint; level:longint; optname:longint; optval:pointer; optlen:Psize_t):longint;cdecl;external libc_nlm name 'getsockopt';
function Fplisten(s:longint; backlog:longint):longint;cdecl;external libc_nlm name 'listen';
function Fprecv(s:longint; buf:pointer; len:size_t; flags:longint):ssize_t;cdecl;external libc_nlm name 'recv';
function Fprecvfrom(s:longint; buf:pointer; len:size_t; flags:longint; from:Psockaddr;
           fromlen:Psize_t):ssize_t;cdecl;external libc_nlm name 'recvfrom';
function Fprecvmsg(s:longint; msg:Pmsghdr; flags:longint):ssize_t;cdecl;external libc_nlm name 'recvmsg';
function Fpsend(s:longint; msg:pointer; len:size_t; flags:longint):ssize_t;cdecl;external libc_nlm name 'send';
function Fpsendmsg(s:longint; _para2:Pmsghdr; flags:longint):ssize_t;cdecl;external libc_nlm name 'sendmsg';
function Fpsendto(s:longint; msg:pointer; len:size_t; flags:longint; _to:Psockaddr;
           tolen:size_t):ssize_t;cdecl;external libc_nlm name 'sendto';
function Fpsetsockopt(s:longint; level:longint; optname:longint; optval:pointer; optlen:size_t):longint;cdecl;external libc_nlm name 'setsockopt';
function Fpshutdown(s:longint; how:longint):longint;cdecl;external libc_nlm name 'shutdown';
function Fpsocket(domain:longint; _type:longint; protocol:longint):longint;cdecl;external libc_nlm name 'socket';


// sys/sockio.h
// sys/stat.h
{ turn on 1-byte packing...  }

{ file system type               }
{ fragment size                  }
{ block size                     }
{ total number of blocks         }
{ count of free blocks           }
{ total number of file nodes     }
{ count of free file nodes       }
{ server name                    }
{ volume name                    }
{ pack name                      }
type
   Pstatfs = ^Tstatfs;
   Tstatfs = record
        f_fstyp  : longint;
        f_frsize : size_t;
        f_bsize  : blksize_t;
        f_blocks : blkcnt_t;
        f_bfree,
        f_files,
        f_ffree  : Tuint64;
        f_fspare : array[0..1] of Tuint64;
        f_fserver: array[0..(48 + 4)-1] of char;
        f_fname  : array[0..(16 + 4)-1] of char;
        f_fpack  : array[0..19] of char;
     end;

(** unsupported pragma#pragma pack()*)


function Fpstatfs(path:Pchar; buf:Pstatfs):longint;cdecl;external libc_nlm name 'statfs';
function Fpstatfs(path:Pchar; var buf:Tstatfs):longint;cdecl;external libc_nlm name 'statfs';
function Fpfstatfs(fildes:longint; buf:Pstatfs):longint;cdecl;external libc_nlm name 'fstatfs';
function Fpfstatfs(fildes:longint; var buf:Tstatfs):longint;cdecl;external libc_nlm name 'fstatfs';

// sys/ttydefaults.h

{ system wide defaults for terminal state, mostly for porting help  }
{ defaults on "first" open...  }
{ control character defaults...  }
{ compatibility:  }
// sys/un.h

{ used in place of struct sockaddr_t to define addresses for UNIX domain...  }
type
   Psockaddr_un = ^sockaddr_un;
   sockaddr_un = record
        sun_family : sa_family_t;
        sun_path : array[0..510] of char;
     end;

// ======= sys/utsname.h ===============================================

{ turn on 1-byte packing...  }

// (request) bits for uname2()...
const
   UNAME_NLMMODULE      = $00000001;  // nlmmodule (if it can be gotten)
   UNAME_POSIXFIELDS    = $00000002;  // POSIX fields (see sys/utsname.h)
   UNAME_LIBVERSION     = $00000004;  // libminor/major/revision/threshold
   UNAME_FSVERSION      = $00000008;  // major/minor/revision/service pack
   UNAME_NETWAREVERSION = $00000010;  // netware_major/minor/revision
   UNAME_SERVERNAME     = $00000020;  // servername
   UNAME_CLUSTERNAME    = $00000040;  // clustername
   UNAME_LANGINFO       = $00000080;  // languagename/alt/id/altid/codepage
   UNAME_NLMMESSAGES    = $00000100;  // nlmmessagecount/table
   UNAME_NLMVERSION     = $00000200;  // nlmmajor/minor/revision
   UNAME_NLMINFO        = $00000400;  // nlmtimer/loadflags
   UNAME_NLMNAME        = $00000800;  // nlmname
   UNAME_NLMLOADPATH    = $00001000;  // nlmloadpath
   UNAME_NLMCOPYRIGHT   = $00002000;  // nlmcopyright
   UNAME_NLMDESCRIPTION = $00004000;  // nlmdescription
   UNAME_NLMCOMMANDLINE = $00008000;  // nlmcommandline
   UNAME_NDSTREENAME    = $00010000;  // treename
   UNAME_NLMCODEANDDATA = $00020000;  // code and datastart/-length

type
   Putsname = ^Tutsname;
   Tutsname = record
        userspec         : longint;     // untouched by uname()
                                        // Novell fields
                                        // Standard C Library implementation:
        libmajor         : longint;     //   major version number
        libminor         : longint;     //   minor version number
        librevision      : longint;     //   revision number
                                        // NetWare OS implementation
        major            : longint;     //   major version number
        minor            : longint;     //   minor version number
        revision         : longint;     //   revision number
                                        // NetWare C Library implementation
        libthreshold     : longint;     //   functionality and semantics timestamp
                                        // NetWare product distribution
        servicepack      : longint;
        netware_major    : longint;
        netware_minor    : longint;
        netware_revision : longint;
        servername       : array[0..63] of char;
        clustername      : array[0..63] of char;
        languagename     : array[0..31] of char;  // Server current language name
        altlanguagename  : array[0..31] of char;  // NLM's current language name
        languageid       : longint;
        altlanguageid    : longint;
        codepage         : longint;
        reserved1        : longint;
        reserved2        : array[0..3] of longint;
        nlmmodule        : pointer;              // NetWare-loadable module (NLM) handle
        nlmmajor         : longint;
        nlmminor         : longint;
        nlmrevision      : longint;
        nlmtimer         : time_t;               // module's date and time stamp in UTC
        nlmcommandline   : Pchar;
        nlmmessagecount  : dword;
        nlmmessagetable  : ^Pchar;
        nlmname          : array[0..35] of char;
        nlmloadpath      : array[0..255] of char;
        nlmcopyright     : array[0..255] of char;
        nlmdescription   : array[0..127] of char;
        nlmloadflags     : longint;
        reserved3        : longint;
        release          : array[0..15] of char;
        version          : array[0..15] of char;
        sysname          : array[0..15] of char;
        machine          : array[0..15] of char;
        nodename         : array[0..15] of char;
        treename         : array[0..95] of char; // name of NDS tree
        codeoffset       : pointer;
        codelength       : dword;
        dataoffset       : pointer;
        datalength       : dword;
        reserved4        : array[0..27] of longint;
     end;

(** unsupported pragma#pragma pack()*)

function Fpuname(name:Putsname):longint;cdecl;external libc_nlm name 'uname';
function Fpuname(var name:Tutsname):longint;cdecl;external libc_nlm name 'uname';
function Fpuname2(handle:pointer; info:Putsname; bits:dword):longint;cdecl;external libc_nlm name 'uname2';
function Fpuname2(handle:pointer; var info:Tutsname; bits:dword):longint;cdecl;external libc_nlm name 'uname2';


// sys/wait.h

function Fpwait(stat_loc:Plongint):pid_t;cdecl;external libc_nlm name 'wait';
function Fpwait(var stat_loc:longint):pid_t;cdecl;external libc_nlm name 'wait';
function Fpwaitpid(pid:pid_t; stat_loc:Plongint; options:longint):pid_t;cdecl;external libc_nlm name 'waitpid';
function Fpwaitpid(pid:pid_t; var stat_loc:longint; options:longint):pid_t;cdecl;external libc_nlm name 'waitpid';

// arpa/inet.h
// netinet/in.h

{ protocols...  }
{ port/socket numbers: network standard functions  }
{ port/socket numbers: host specific functions...  }
{ UNIX TCP sockets...   }
{ UNIX UDP sockets...  }
{
** Ports numbered less than IPPORT_RESERVED are reserved for privileged
** processes like 'root.'
 }
{ link numbers...  }
{
** Definitions of bits in Internet address integers. On subnets, the
** decomposition of addresses to host and net parts is done according
** to subnet mask, not the masks here.
 }
{ options for use with [gs]etsockopt at IP level...  }
{ macro to stuff the loopback address into an Internet address...  }
{ address testing macros...  }
{ type definitions...  }
type
   Pin_addr = ^in_addr;
   in_addr = record
        S_un : record
            case longint of
               0 : ( S_un_b : record
                    s_b1 : byte;
                    s_b2 : byte;
                    s_b3 : byte;
                    s_b4 : byte;
                 end );
               1 : ( S_un_w : record
                    s_w1 : word;
                    s_w2 : word;
                 end );
               2 : ( S_addr : dword );
            end;
     end;

   //!! in_addr = in_addr_t;
{ socket address, internet style    }
   Psockaddr_in = ^sockaddr_in;
   sockaddr_in = record
        sin_family : smallint;
        sin_port : u_short;
        sin_addr : in_addr;
        sin_zero : array[0..7] of char;
     end;

   //!! sockaddr_in = sockaddr_in_t;
   Pin6_addr = ^in6_addr;
   in6_addr = record
        in6a_u : record
            case longint of
               0 : ( bytes : array[0..15] of u_char );
               1 : ( shorts : array[0..7] of u_short );
               2 : ( words : array[0..3] of u_long );
            end;
     end;

   Psockaddr_in6 = ^sockaddr_in6;
   sockaddr_in6 = record
        sin6_family : smallint;
        sin6_port : u_short;
        sin6_flowinfo : u_long;
        sin6_addr : in6_addr;
        sin6_scope_id : u_long;
     end;

{ for IP_ADD_MEMBERSHIP and IP_DROP_MEMBERSHIP...  }
{ IP multicast address of group    }
{ local IP address of interface    }
   Pip_mreq = ^ip_mreq;
   ip_mreq = record
        imr_multiaddr : in_addr;
        imr_interface : in_addr;
     end;

{ data...  }

//  var
//     in6addr_any : in6_addr;cvar;external;
//     in6addr_loopback : in6_addr;cvar;external;


function inet_addr(_string:Pchar):dword;cdecl;external libc_nlm name 'inet_addr';
function inet_aton(cp:Pchar; addr:Pin_addr):longint;cdecl;external libc_nlm name 'inet_aton';
function inet_makeaddr(net_num:in_addr; loc_addr:in_addr):in_addr;cdecl;external libc_nlm name 'inet_makeaddr';
function inet_network(_string:Pchar):in_addr;cdecl;external libc_nlm name 'inet_network';
function inet_ntoa(addr:in_addr):Pchar;cdecl;external libc_nlm name 'inet_ntoa';
function inet_ntop(af:longint; src:pointer; dst:Pchar; size:size_t):Pchar;cdecl;external libc_nlm name 'inet_ntop';
function inet_pton(af:longint; cp:Pchar; ap:pointer):longint;cdecl;external libc_nlm name 'inet_pton';


// netinet/in.h

{ User-settable options (used with setsockopt). }


// nks/defs.h

  const
     NX_MAX_KEYS = 64;
     NX_MAX_OBJECT_NAME_LEN = 31;
     NX_INTR1 = 0;
     NX_INTR2 = 1;
     NX_INTR3 = 2;
     NX_INTR4 = 3;
     NX_INTR5 = 4;
     NX_INTR6 = 5;
     NX_INTR7 = 6;
     NX_INTR8 = 7;
     NX_INTR9 = 8;
     NX_INTR10 = 9;
     NX_INTR11 = 10;
     NX_INTR12 = 11;
     NX_INTR13 = 12;
     NX_INTR14 = 13;
     NX_INTR15 = 14;
     NX_INTR16 = 15;
     NX_INTR17 = 16;
     NX_INTR18 = 17;
     NX_INTR19 = 18;
     NX_INTR20 = 19;
     NX_INTR21 = 20;
     NX_INTR22 = 21;
     NX_INTR23 = 22;
     NX_INTR24 = 23;
     NX_INTR25 = 24;
     NX_INTR26 = 25;
     NX_INTR27 = 26;
     NX_INTR28 = 27;
     NX_INTR29 = 28;
     NX_INTR30 = 29;
     NX_INTR31 = 30;
     NX_INTR32 = 31;
     NX_INTR33 = 32;
     NX_INTR34 = 33;
     NX_INTR35 = 34;
     NX_INTR36 = 35;
     NX_INTR37 = 36;
     NX_INTR38 = 37;
     NX_INTR39 = 38;
     NX_INTR40 = 39;
     NX_INTR41 = 40;
     NX_INTR42 = 41;
     NX_INTR43 = 42;
     NX_INTR44 = 43;
     NX_INTR45 = 44;
     NX_INTR46 = 45;
     NX_INTR47 = 46;
     NX_INTR48 = 47;
     NX_INTR49 = 48;
     NX_INTR50 = 49;
     NX_INTR51 = 50;
     NX_INTR52 = 51;
     NX_INTR53 = 52;
     NX_INTR54 = 53;
     NX_INTR55 = 54;
     NX_INTR56 = 55;
     NX_INTR57 = 56;
     NX_INTR58 = 57;
     NX_INTR59 = 58;
     NX_INTR60 = 59;
     NX_INTR61 = 60;
     NX_INTR62 = 61;
     NX_INTR63 = 62;
     NX_INTR64 = 63;


type

   PNXBool_t = ^NXBool_t;
   NXBool_t = longint;

   PNXVmId_t = ^NXVmId_t;
   NXVmId_t = longint;
{ values for NXInterruptId_t...  }


// nks/dirio.h
// nks/fsio.h
// unilib.h

  const
     UNI_ERR_MEM_ALLOC = -(494);
  { nonexistant rule table handle      }
     UNI_ERR_BAD_HANDLE = -(496);
  { table corruption detected          }
     UNI_ERR_TABLE_CORRUPT = -(498);
  { insufficient room in string        }
     UNI_ERR_TOO_FEW_BYTES = -(500);
  { unable to open data file           }
     UNI_ERR_FILE_OPEN = -(501);
  {                                    }
     UNI_ERR_FILE_EXIST = -(502);
  { unable to read data file           }
     UNI_ERR_FILE_READ = -(504);
  { functional stub only               }
     UNI_ERR_UNIMPLEMENTED = -(505);
  { premature end-of-string            }
     UNI_ERR_PREMATURE_END = -(506);
  { discovered during translation      }
     UNI_ERR_UNMAPPABLE_CHAR = -(532);
  { invalid UTF-8 character sequence   }
     UNI_ERR_INVALID_UTF8_SEQ = $FFFF;
  { the local, default rule table for argument 'table' below...  }
  { respective to local codepage       }
     UNI_LOCAL_DEFAULT = -(1);
  { 'noMapFlag' values; when no mapping found...  }
  { return UNI_ERR_UNMAPPABLE_CHAR     }
     UNI_MAP_NO_CHAR = 0;
  { use value in 'noMapChar' unless 0  }
     UNI_MAP_CHAR = 1;
  { use 'noMapFunc' if non-nil         }
     UNI_MAP_BY_FUNC = 1;
  { use character itself               }
     UNI_MAP_SELF = 2;
  { no-map character if 'noMapChar' 0  }
     UNI_NOMAP_DEFAULT = '?';
  { character classification (UniClass_t)...  }
  { no classification                      }
     UNI_UNDEF = $00000000;
  { control character                      }
     UNI_CNTRL = $00000001;
  { non-printing space                     }
     UNI_SPACE = $00000002;
  { printing (visible) character           }
     UNI_PRINT = $00000004;
  { dingbats, special symbols, et al.      }
     UNI_SPECIAL = $00000008;
  { general punctuation                    }
     UNI_PUNCT = $00000010;
  { decimal digit                          }
     UNI_DIGIT = $00000020;
  { hexadecimal digit                      }
     UNI_XDIGIT = $00000040;
  { reserved for future use                }
     UNI_RESERVED1 = $00000080;
  { lower-case if applicable               }
     UNI_LOWER = $00000100;
  { upper-case if applicable               }
     UNI_UPPER = $00000200;
  { reserved for future use                }
     UNI_RESERVED2 = $00000400;
  { non-number, non-punctuation including: }
     UNI_ALPHA = $00000800;
  { Latin-based                            }
     UNI_LATIN = $00001000;
  { Greek                                  }
     UNI_GREEK = $00002000;
  { Cyrillic                               }
     UNI_CYRILLIC = $00004000;
  { Hebrew                                 }
     UNI_HEBREW = $00008000;
  { Arabic                                 }
     UNI_ARABIC = $00010000;
  { Chinese/Japanese/Korean characters     }
     UNI_CJK = $00020000;
  { Devanagari, Bengali, Tamil, et al.     }
     UNI_INDIAN = $00040000;
  { southeast Asia: Thai, Lao              }
     UNI_SEASIA = $00080000;
  { cent. Asia: Armenian Tibetain, Georg.  }
     UNI_CENASIA = $00100000;
  { none of the above                      }
     UNI_OTHER = $80000000;


type
   Punicode_t = ^unicode_t;
   unicode_t = wchar_t;
   PPunicode_t = ^Punicode_t;

   PUniRuleTable_t = ^UniRuleTable_t;
   UniRuleTable_t = longint;
{ more a cookie than anything else      }

   PUniClass_t = ^UniClass_t;
   UniClass_t = dword;
{ Unicode character classification       }
{ for uni2mono(), unicase(), et al.      }
{ default monocasing as implemented      }
{ character is not 'alphabetic'          }
{ character has no case                  }
{ emphatically upper case                }
{ emphatically lower case                }

   PUniCase_t = ^UniCase_t;
   UniCase_t =  Longint;
   Const
     UNI_CASE_DEFAULT = $FFFFFFFD;
     UNI_CASE_NONE = $FFFFFFFE;
     UNI_CASE_AMBIGUOUS = $FFFFFFFF;
     UNI_CASE_UPPER = $00000000;
     UNI_CASE_LOWER = $00000001;
     UNI_CASE_TITLE = $00000002;

{ unmappable character handling function types...  }

type

   Loc2UniNoMapFunc_t = function (dest:PPunicode_t; remaining:size_t; src:PPchar; userParm:pointer):longint;cdecl;


   Loc2Utf8NoMapFunc_t = function (dest:PPchar; remaining:size_t; src:PPchar; userParm:pointer):longint;cdecl;


   Utf82LocNoMapFunc_t = function (dest:PPchar; remaining:size_t; src:PPchar; userParm:pointer):longint;cdecl;


   Utf82UniNoMapFunc_t = function (dest:PPchar; remaining:size_t; src:PPunicode_t; userParm:pointer):longint;cdecl;


   Uni2LocNoMapFunc_t = function (dest:PPchar; remaining:size_t; src:PPunicode_t; userParm:pointer):longint;cdecl;


   Uni2Utf8NoMapFunc_t = function (dest:PPchar; remaining:size_t; src:PPunicode_t; userParm:pointer):longint;cdecl;
{ rule table management...  }

function UniGetTable(codePage:longint; table:PUniRuleTable_t):longint;cdecl;external libc_nlm name 'UniGetTable';

function UniGetMacintoshTable(name:Pchar; table:PUniRuleTable_t):longint;cdecl;external libc_nlm name 'UniGetMacintoshTable';
function UniSetDefault(table:UniRuleTable_t):longint;cdecl;external libc_nlm name 'UniSetDefault';
function UniDisposeTable(table:UniRuleTable_t):longint;cdecl;external libc_nlm name 'UniDisposeTable';
function UniGetHostCodePage:longint;cdecl;external libc_nlm name 'UniGetHostCodePage';
{ translation between local and other codepages, Unicode and UTF-8...  }

function loc2uni(table:UniRuleTable_t; dest:Punicode_t; src:Pchar; noMapCh:unicode_t; noMapFlag:longint):longint;cdecl;external libc_nlm name 'loc2uni';

function loc2unipath(table:UniRuleTable_t; dest:Punicode_t; src:Pchar; dryRunSize:Psize_t):longint;cdecl;external libc_nlm name 'loc2unipath';

function locn2uni(table:UniRuleTable_t; dest:Punicode_t; destLen:Psize_t; src:Pchar; srcLen:size_t;
           noMapCh:unicode_t; noMapFlag:longint):longint;cdecl;external libc_nlm name 'locn2uni';
function locnp2uni(table:UniRuleTable_t; target:PPunicode_t; destLen:Psize_t; source:PPchar; srcLen:size_t;
           noMapCh:unicode_t; noMapFlag:longint):longint;cdecl;external libc_nlm name 'locnp2uni';

function locnx2uni(table:UniRuleTable_t; dest:Punicode_t; destLen:Psize_t; src:Pchar; srcLen:size_t;
           noMapFunc:Loc2UniNoMapFunc_t; noMapFuncParm:pointer; noMapFlag:longint):longint;cdecl;external libc_nlm name 'locnx2uni';

function locnx2unipath(table:UniRuleTable_t; dest:Punicode_t; destLen:Psize_t; src:Pchar; srcLen:size_t;
           noMapFunc:Loc2UniNoMapFunc_t; noMapFuncParm:pointer; noMapFlag:longint; dryRunSize:Psize_t):longint;cdecl;external libc_nlm name 'locnx2unipath';

function locn2unispecial(handle:UniRuleTable_t; dest:Punicode_t; destLen:Psize_t; src:Pchar; srcLen:size_t):longint;cdecl;external libc_nlm name 'locn2unispecial';

function loc2utf8(handle:UniRuleTable_t; dest:Pchar; src:Pchar; noMapCh:char; noMapFlag:longint):longint;cdecl;external libc_nlm name 'loc2utf8';

function loc2utf8path(table:UniRuleTable_t; dest:Pchar; src:Pchar; dryRunSize:Psize_t):longint;cdecl;external libc_nlm name 'loc2utf8path';

function locn2utf8(table:UniRuleTable_t; dest:Pchar; destLen:Psize_t; src:Pchar; srcLen:size_t;
           noMapCh:char; noMapFlag:longint):longint;cdecl;external libc_nlm name 'locn2utf8';

function locnx2utf8(table:UniRuleTable_t; dest:Pchar; destLen:Psize_t; src:Pchar; srcLen:size_t;
           noMapFunc:Loc2Utf8NoMapFunc_t; noMapFuncParm:pointer; noMapFlag:longint):longint;cdecl;external libc_nlm name 'locnx2utf8';

function uni2loc(table:UniRuleTable_t; dest:Pchar; src:Punicode_t; noMapCh:char; noMapFlag:longint):longint;cdecl;external libc_nlm name 'uni2loc';

function uni2locpath(table:UniRuleTable_t; dest:Pchar; src:Punicode_t; dryRunSize:Psize_t):longint;cdecl;external libc_nlm name 'uni2locpath';

function unin2loc(table:UniRuleTable_t; dest:Pchar; destLen:Psize_t; src:Punicode_t; srcLen:size_t;
           noMapCh:char; noMapFlag:longint):longint;cdecl;external libc_nlm name 'unin2loc';
function uninp2loc(table:UniRuleTable_t; target:PPchar; destLen:Psize_t; source:PPunicode_t; srcLen:size_t;
           noMapCh:char; noMapFlag:longint):longint;cdecl;external libc_nlm name 'uninp2loc';

function uninx2loc(table:UniRuleTable_t; dest:Pchar; destLen:Psize_t; src:Punicode_t; srcLen:size_t;
           noMapFunc:Uni2LocNoMapFunc_t; noMapFuncParm:pointer; noMapFlag:longint):longint;cdecl;external libc_nlm name 'uninx2loc';

function uninx2locpath(table:UniRuleTable_t; dest:Pchar; destLen:Psize_t; src:Punicode_t; srcLen:size_t;
           noMapFunc:Uni2LocNoMapFunc_t; noMapFuncParm:pointer; noMapFlag:longint; dryRunSize:Psize_t):longint;cdecl;external libc_nlm name 'uninx2locpath';

function unin2locspecial(handle:UniRuleTable_t; dest:Pchar; destLen:Psize_t; src:Punicode_t; srcLen:size_t):longint;cdecl;external libc_nlm name 'unin2locspecial';

function uni2utf8(dest:Pchar; src:Punicode_t):longint;cdecl;external libc_nlm name 'uni2utf8';

function uni2utf8path(dest:Pchar; src:Punicode_t; dryRunSize:Psize_t):longint;cdecl;external libc_nlm name 'uni2utf8path';

function unin2utf8(dest:Pchar; destLen:Psize_t; src:Punicode_t; srcLen:size_t):longint;cdecl;external libc_nlm name 'unin2utf8';

function utf82loc(handle:UniRuleTable_t; dest:Pchar; src:Pchar; noMapCh:char; noMapFlag:longint):longint;cdecl;external libc_nlm name 'utf82loc';

function utf8n2loc(table:UniRuleTable_t; dest:Pchar; destLen:Psize_t; src:Pchar; srcLen:size_t;
           noMapCh:char; noMapFlag:longint):longint;cdecl;external libc_nlm name 'utf8n2loc';

function utf8nx2loc(table:UniRuleTable_t; dest:Pchar; destLen:Psize_t; src:Pchar; srcLen:size_t;
           noMapFunc:Utf82LocNoMapFunc_t; noMapFuncParm:pointer; noMapFlag:longint):longint;cdecl;external libc_nlm name 'utf8nx2loc';

function utf82uni(dest:Punicode_t; src:Pchar):longint;cdecl;external libc_nlm name 'utf82uni';

function utf8n2uni(dest:Punicode_t; destLen:Psize_t; src:Pchar; srcLen:size_t):longint;cdecl;external libc_nlm name 'utf8n2uni';
{ quick, 7-bit ASCII-capable translations--not preferred set...  }

function asc2uni(dest:Punicode_t; src:Pchar):Punicode_t;cdecl;external libc_nlm name 'asc2uni';

function ascn2uni(dest:Punicode_t; src:Pchar; nbytes:size_t):Punicode_t;cdecl;external libc_nlm name 'ascn2uni';

function uni2asc(dest:Pchar; src:Punicode_t):Pchar;cdecl;external libc_nlm name 'uni2asc';

function unin2asc(dest:Pchar; src:Punicode_t; nchars:size_t):Pchar;cdecl;external libc_nlm name 'unin2asc';
{ default 'noMapFunc' for X-translation to ensure round-trip conversion...  }

function LocToUniTagFunc(dest:PPunicode_t; remaining:size_t; src:PPchar; userParm:pointer):longint;cdecl;external libc_nlm name 'LocToUniTagFunc';

function UniToLocTagFunc(dest:PPchar; remaining:size_t; src:PPunicode_t; userParm:pointer):longint;cdecl;external libc_nlm name 'UniToLocTagFunc';
{ string size calculation...  }

function LocToUniSize(table:UniRuleTable_t; str:Pchar; unmappedCharSize:size_t; noMapFlag:longint; uniBufSize:Psize_t):longint;cdecl;external libc_nlm name 'LocToUniSize';

function LocToUtf8Size(table:UniRuleTable_t; str:Pchar; unmappedCharSize:size_t; noMapFlag:longint; utf8BufSize:Psize_t):longint;cdecl;external libc_nlm name 'LocToUtf8Size';

function UniToLocSize(table:UniRuleTable_t; str:Punicode_t; unmappedCharSize:size_t; noMapFlag:longint; locBufSize:Psize_t):longint;cdecl;external libc_nlm name 'UniToLocSize';

function UniToUtf8Size(str:Punicode_t; utf8BufSize:Psize_t):longint;cdecl;external libc_nlm name 'UniToUtf8Size';

function Utf8ToLocSize(table:UniRuleTable_t; str:Pchar; unmappedCharSize:size_t; noMapFlag:longint; locBufSize:Psize_t):longint;cdecl;external libc_nlm name 'Utf8ToLocSize';

function Utf8ToUniSize(str:Pchar; uniBufSize:Psize_t):longint;cdecl;external libc_nlm name 'Utf8ToUniSize';
{-----------------------------------------------------------------------------
** Little utility functions. These are not to be preferred over the interfaces
** from wchar.h.
 }
{ utility to measure width of a character in a codepage...  }

function dbcs_width(codepage:longint; str:Pchar):longint;cdecl;external libc_nlm name 'dbcs_width';
{ classification...  }
function unitype(ch:unicode_t):UniClass_t;cdecl;external libc_nlm name 'unitype';
{ collation...  }


function unicoll(s1:Punicode_t; s2:Punicode_t):longint;cdecl;external libc_nlm name 'unicoll';


function unincoll(s1:Punicode_t; s2:Punicode_t; n:size_t):longint;cdecl;external libc_nlm name 'unincoll';
{ casing...  }
function unicase(ch:unicode_t):UniCase_t;cdecl;external libc_nlm name 'unicase';

function uni2mono(dest:Punicode_t; src:Punicode_t; casing:UniCase_t):Punicode_t;cdecl;external libc_nlm name 'uni2mono';

function unin2mono(dest:Punicode_t; src:Punicode_t; casing:UniCase_t; destLen:size_t):Punicode_t;cdecl;external libc_nlm name 'unin2mono';
function chr2upr(ch:unicode_t):unicode_t;cdecl;external libc_nlm name 'chr2upr';
function chr2lwr(ch:unicode_t):unicode_t;cdecl;external libc_nlm name 'chr2lwr';
function chr2title(ch:unicode_t):unicode_t;cdecl;external libc_nlm name 'chr2title';
function unilwr(_string:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unilwr';
function uniupr(_string:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'uniupr';

function uni2lwr(dest:Punicode_t; src:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'uni2lwr';

function unin2lwr(dest:Punicode_t; src:Punicode_t; destLen:size_t):Punicode_t;cdecl;external libc_nlm name 'unin2lwr';

function uni2upr(dest:Punicode_t; src:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'uni2upr';

function unin2upr(dest:Punicode_t; src:Punicode_t; destLen:size_t):Punicode_t;cdecl;external libc_nlm name 'unin2upr';

function uni2title(dest:Punicode_t; src:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'uni2title';

function unin2title(dest:Punicode_t; src:Punicode_t; destLen:size_t):Punicode_t;cdecl;external libc_nlm name 'unin2title';
{ length...  }

function unilen(_string:Punicode_t):size_t;cdecl;external libc_nlm name 'unilen';

function uninlen(_string:Punicode_t; max:size_t):size_t;cdecl;external libc_nlm name 'uninlen';

function unisize(_string:Punicode_t):size_t;cdecl;external libc_nlm name 'unisize';
{ copying...  }

function unicpy(tgt:Punicode_t; src:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unicpy';

function unincpy(tgt:Punicode_t; src:Punicode_t; n:size_t):Punicode_t;cdecl;external libc_nlm name 'unincpy';
function uniset(base:Punicode_t; ch:unicode_t):Punicode_t;cdecl;external libc_nlm name 'uniset';
function uninset(base:Punicode_t; ch:unicode_t; n:size_t):Punicode_t;cdecl;external libc_nlm name 'uninset';
{ concatenation...  }

function unicat(tgt:Punicode_t; src:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unicat';

function unincat(tgt:Punicode_t; src:Punicode_t; n:size_t):Punicode_t;cdecl;external libc_nlm name 'unincat';

{$ifndef DisableArrayOfConst}
function unilist(tgt:Punicode_t; s1:Punicode_t; args:array of const):Punicode_t;cdecl;external libc_nlm name 'unilist';
{$endif}
function unilist(tgt:Punicode_t; s1:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unilist';
{ comparison...  }


function unicmp(s1:Punicode_t; s2:Punicode_t):longint;cdecl;external libc_nlm name 'unicmp';


function uniicmp(s1:Punicode_t; s2:Punicode_t):longint;cdecl;external libc_nlm name 'uniicmp';


function unincmp(s1:Punicode_t; s2:Punicode_t; n:size_t):longint;cdecl;external libc_nlm name 'unincmp';


function uninicmp(s1:Punicode_t; s2:Punicode_t; n:size_t):longint;cdecl;external libc_nlm name 'uninicmp';
{ character matching, indexing and miscellaneous...  }

function unichr(_string:Punicode_t; ch:unicode_t):Punicode_t;cdecl;external libc_nlm name 'unichr';
function unirchr(_string:Punicode_t; ch:unicode_t):Punicode_t;cdecl;external libc_nlm name 'unirchr';
function uniindex(_string:Punicode_t; search:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'uniindex';
function unistr(as1:Punicode_t; as2:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unistr';
function unirev(base:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unirev';
function unispn(_string:Punicode_t; charset:Punicode_t):size_t;cdecl;external libc_nlm name 'unispn';
function unicspn(_string:Punicode_t; charset:Punicode_t):size_t;cdecl;external libc_nlm name 'unicspn';
function unipbrk(s1:Punicode_t; s2:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unipbrk';
function unitok(_string:Punicode_t; sepset:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unitok';
function unitok_r(_string:Punicode_t; sepset:Punicode_t; lasts:PPunicode_t):Punicode_t;cdecl;external libc_nlm name 'unitok_r';
function unidup(s1:Punicode_t):Punicode_t;cdecl;external libc_nlm name 'unidup';
// nks/time.h

{ values for 'epoch' for NXGetTime()...  }
{ values for 'units' for NXGetTime()...  }
{ turn on 1-byte packing...  }

type

   PNXTime_t = ^NXTime_t;
   NXTime_t = Tuint64;

   PNXTimerVal_t = ^NXTimerVal_t;
   NXTimerVal_t = record
        tvPeriod : dword;
        tvCurrent : dword;
     end;

   PNXTimeOut_t = ^NXTimeOut_t;
   NXTimeOut_t = record
        reserved1 : array[0..3] of dword;
        toRoutine : procedure (_para1:pointer);cdecl;
        toArg : pointer;
        toTimerVal : NXTimerVal_t;
        reserved2 : array[0..15] of dword;
     end;

(** unsupported pragma#pragma pack()*)
{ Time and time-out...  }

function NXTimeOutCancel(tout:PNXTimeOut_t; wait:NXBool_t; status:PNXBool_t):longint;cdecl;external libc_nlm name 'NXTimeOutCancel';
function NXTimeOutSchedule(tout:PNXTimeOut_t):longint;cdecl;external libc_nlm name 'NXTimeOutSchedule';
function NXGetTime(epoch:longint; units:longint; time:PNXTime_t):longint;cdecl;external libc_nlm name 'NXGetTime';


// nks/thread.h
// nks/plat.h
type

   PNXCpuId_t = ^NXCpuId_t;
   NXCpuId_t = longint;
{ Platform-specific services...  }

function NXGetCacheLineSize:size_t;cdecl;external libc_nlm name 'NXGetCacheLineSize';
function NXGetCpuCount:dword;cdecl;external libc_nlm name 'NXGetCpuCount';
function NXGetCpuId:NXCpuId_t;cdecl;external libc_nlm name 'NXGetCpuId';
function NXGetPageSize:size_t;cdecl;external libc_nlm name 'NXGetPageSize';
function NXSeedRandom(width:size_t; seed:pointer):longint;cdecl;external libc_nlm name 'NXSeedRandom';
function NXGetSystemTick:dword;cdecl;external libc_nlm name 'NXGetSystemTick';
{ values for thread priority...  }
{ values for thread context flags...  }
{ values for thread flags...  }
{ value returned that specifies that the thread is unbound  }
{ value passed to NXThreadBind to bind it to the current CPU  }
{ return from NXThreadGetId indicating no-context or other error  }

{ turn on 1-byte packing...  }

type

   PNXContext_t = ^NXContext_t;
   NXContext_t = void;

   PNXWorkId_t = ^NXWorkId_t;
   NXWorkId_t = longint;

   PNXKey_t = ^NXKey_t;
   NXKey_t = longint;

   PNXThreadId_t = ^NXThreadId_t;
   NXThreadId_t = longint;

   PNXInterruptId_t = ^NXInterruptId_t;
   NXInterruptId_t = longint;

   PNXInterruptSet_t = ^NXInterruptSet_t;
   NXInterruptSet_t = Tuint64;

   PNXContextState_t = ^NXContextState_t;
   NXContextState_t =  Longint;
   Const
     NX_CTXSTATE_INIT = $FFFFFFFF;
     NX_CTXSTATE_BOUND = $00000001;
     NX_CTXSTATE_UNBOUND = $00000000;

type

   PNXContextInfo_t = ^NXContextInfo_t;
   NXContextInfo_t = record
        ciState : NXContextState_t;
        ciFunc : procedure (_para1:pointer);cdecl;
        ciArg : pointer;
        ciPriority : longint;
        ciStackSize : size_t;
        ciFlags : dword;
     end;

   PlwWork = ^lwWork;
   lwWork = record
        reserved : array[0..6] of pointer;
        lwWorkFunc : procedure (_para1:PlwWork; _para2:pointer);cdecl;
        lwAppRef : pointer;
     end;
   NXLwWork_t = lwWork;
   PNXLwWork_t = ^NXLwWork_t;

(** unsupported pragma#pragma pack()*)
{ Context management...  }

type TCDeclProc1PtrArg = procedure (_para1:pointer); cdecl;

function NXContextAlloc(start_routine:TCDeclProc1PtrArg; arg:pointer; priority:longint; stackSize:size_t; flags:dword;
           error:Plongint):NXContext_t;cdecl;external libc_nlm name 'NXContextAlloc';
function NXContextFree(ctx:NXContext_t):longint;cdecl;external libc_nlm name 'NXContextFree';
function NXContextGet:NXContext_t;cdecl;external libc_nlm name 'NXContextGet';
function NXContextGetInfo(ctx:NXContext_t; info:PNXContextInfo_t):longint;cdecl;external libc_nlm name 'NXContextGetInfo';
function NXContextGetName(ctx:NXContext_t; name:Pchar; len:size_t):longint;cdecl;external libc_nlm name 'NXContextGetName';
function NXContextReinit(ctx:NXContext_t; start_routine:TCDeclProc1PtrArg; arg:pointer; priority:longint; flags:dword):longint;cdecl;external libc_nlm name 'NXContextReinit';

function NXContextSetName(ctx:NXContext_t; name:Pchar):longint;cdecl;external libc_nlm name 'NXContextSetName';
{ Key-value pairs (per-context data)...  }
type TCdeclProc = procedure; cdecl;
function NXKeyCreate(_destructor: TCDeclProc; value:pointer; key:PNXKey_t):longint;cdecl;external libc_nlm name 'NXKeyCreate';
function NXKeyDelete(key:NXKey_t):longint;cdecl;external libc_nlm name 'NXKeyDelete';
function NXKeyGetValue(key:NXKey_t; value:Ppointer):longint;cdecl;external libc_nlm name 'NXKeyGetValue';
function NXKeySetValue(key:NXKey_t; value:pointer):longint;cdecl;external libc_nlm name 'NXKeySetValue';
{ Thread management...  }
function NXThreadBind(cpu_id:NXCpuId_t):longint;cdecl;external libc_nlm name 'NXThreadBind';
function NXThreadContinue(tid:NXThreadId_t):longint;cdecl;external libc_nlm name 'NXThreadContinue';
function NXThreadCreate(ctx:NXContext_t; flags:dword; idp:PNXThreadId_t):longint;cdecl;external libc_nlm name 'NXThreadCreate';
function NXThreadCreateSx(start_routine:TCDeclProc1PtrArg; arg:pointer; thread_flags:dword; ctx:PNXContext_t; thr:PNXThreadId_t):longint;cdecl;external libc_nlm name 'NXThreadCreateSx';
procedure NXThreadDelay(delay:dword);cdecl;external libc_nlm name 'NXThreadDelay';
function NXThreadDestroy(tid:NXThreadId_t):longint;cdecl;external libc_nlm name 'NXThreadDestroy';
function NXThreadDetach(tid:NXThreadId_t):longint;cdecl;external libc_nlm name 'NXThreadDetach';
procedure NXThreadExit(status:pointer);cdecl;external libc_nlm name 'NXThreadExit';
function NXThreadGetBinding:NXCpuId_t;cdecl;external libc_nlm name 'NXThreadGetBinding';
function NXThreadGetContext(tid:NXThreadId_t; ctx:PNXContext_t):longint;cdecl;external libc_nlm name 'NXThreadGetContext';
function NXThreadGetId:NXThreadId_t;cdecl;external libc_nlm name 'NXThreadGetId';
function NXThreadGetPriority(tid:NXThreadId_t; priority:Plongint):longint;cdecl;external libc_nlm name 'NXThreadGetPriority';
function NXThreadInterrupt(tid:NXThreadId_t; interId:NXInterruptId_t):longint;cdecl;external libc_nlm name 'NXThreadInterrupt';
function NXThreadIsInterrupted(_set:PNXInterruptSet_t):NXBool_t;cdecl;external libc_nlm name 'NXThreadIsInterrupted';
function NXThreadJoin(wait_for:NXThreadId_t; departed_thread:PNXThreadId_t; status:Ppointer):longint;cdecl;external libc_nlm name 'NXThreadJoin';
function NXThreadSetPriority(tid:NXThreadId_t; priority:longint):longint;cdecl;external libc_nlm name 'NXThreadSetPriority';
function NXThreadSuspend(tid:NXThreadId_t):longint;cdecl;external libc_nlm name 'NXThreadSuspend';
function NXThreadSwapContext(newctx:NXContext_t; prevctx:PNXContext_t):longint;cdecl;external libc_nlm name 'NXThreadSwapContext';
function NXThreadUnbind:longint;cdecl;external libc_nlm name 'NXThreadUnbind';
procedure NXThreadYield;cdecl;external libc_nlm name 'NXThreadYield';
function nxThreadCreate(start_routine:TCDeclProc1PtrArg; flags:dword; arg:pointer; ctxp:PNXContext_t; idp:PNXThreadId_t):longint;cdecl;external libc_nlm name 'nxThreadCreate';
{ Work-to-dos...  }
function NXDelayedWorkSchedule(ctx:NXContext_t; timerval:PNXTimerVal_t; bind:NXBool_t; wid:PNXWorkId_t):longint;cdecl;external libc_nlm name 'NXDelayedWorkSchedule';
function NXLwWorkCancel(work:PNXLwWork_t):longint;cdecl;external libc_nlm name 'NXLwWorkCancel';
function NXLwWorkSchedule(reserved:pointer; work:PNXLwWork_t; bind:NXBool_t):longint;cdecl;external libc_nlm name 'NXLwWorkSchedule';
function NXWorkCancel(wid:NXWorkId_t; wait:NXBool_t; status:PNXBool_t):longint;cdecl;external libc_nlm name 'NXWorkCancel';
function NXWorkSchedule(context:NXContext_t; bind:NXBool_t; wid:PNXWorkId_t):longint;cdecl;external libc_nlm name 'NXWorkSchedule';
{ Miscellaneous...  }
function NXProcessInterruptSet(_set:PNXInterruptSet_t; id:NXInterruptId_t; processed_id:PNXInterruptId_t):NXBool_t;cdecl;external libc_nlm name 'NXProcessInterruptSet';
{ values and masks for file modes...  }
{ values and masks for file operations...  }
{ for NXDeviceOpen() and NXConsoleOpen()...  }
{ aliases for above...  }
{ open flags...  }
{ sharing flags...  }
{ delegation type...  }
{ flush flags...  }
{ values and masks for access flags...  }
{ values for setting file length...  }
{ values for file byte-range locking flags...  }


{ turn on 1-byte packing...  }

  const
     NX_MAX_NAME_LEN = 255;
  { (big! --this isn't used yet)  }
{$define NX_MAX_FILESIZE}
  { values and masks for file modes...  }
     NX_O_RDONLY = $00000000;
     NX_O_WRONLY = $00000001;
     NX_O_RDWR = $00000002;
     NX_O_ACCMODE = $00000003;
  { values and masks for file operations...  }
     NX_O_APPEND = $00000010;
     NX_O_CREAT = $00000020;
     NX_O_TRUNC = $00000040;
     NX_O_EXCL = $00000080;
     NX_O_TRANS = $00000100;
     NX_O_NONBLOCK = $00000400;
     NX_O_OPMODE = $000005E0;
  { for NXDeviceOpen() and NXConsoleOpen()...  }
     NX_O_SCROLLABLE = $00000800;
  { aliases for above...  }
     NX_O_CREATE = NX_O_CREAT;
     NX_O_TRUNCATE = NX_O_TRUNC;
     NX_O_EXCLUSIVE = NX_O_EXCL;
  { open flags...  }
     NX_OFLAG_DIRECTIO = $00010000;
     NX_OFLAG_CRONLY = $00020000;
     NX_OFLAG_BACKUP = $00040000;
     NX_OFLAG_RESTORE = $00080000;
     NX_OFLAG_EXTEND = $00100000;
     NX_OFLAG_SYNCWR = $00200000;
     NX_OFLAG_ATOMIC_RW = $00400000;
     NX_OFLAG_NOTRAVERSE_LINK = $00800000;
     NX_OFLAG_MASK = $00FF0000;
  { sharing flags...  }
     NX_SHARE_DENYNO = $00000000;
     NX_SHARE_DENYRD = $00100000;
     NX_SHARE_DENYWR = $00200000;
     NX_SHARE_DENYALL = $00400000;
     NX_SHARE_MASK = $00700000;
  { delegation type...  }
     NX_DELEG_NONE = $00000000;
  { flush flags...  }
     NX_FLUSH_DATA = $00000001;
     NX_FLUSH_METADATA = $00000002;
     NX_FLUSH_ASYNC = $00000004;
  { values and masks for access flags...  }
     NX_R_OK = $00000001;
     NX_W_OK = $00000002;
     NX_X_OK = $00000004;
     NX_F_OK = $00000008;
  { values for setting file length...  }
     NX_FOP_RETURN_EXTEND = $00000001;
     NX_FOP_RETURN_TRUNC_FREE = $00000002;
     NX_FOP_RETURN_SPARSE = $00000004;
  { values for file byte-range locking flags...  }
     NX_RANGE_LOCK_SHARED = $00000001;
     NX_RANGE_LOCK_EXCL = $00000002;
     NX_RANGE_LOCK_CHECK = $00000004;
     NX_RANGE_LOCK_TRYLOCK = $00000008;
     NX_RANGE_LOCK_COURTESY = $00000010;
     NX_RANGE_LOCK_CANCEL = $00000020;
     NX_RANGE_LOCK_POSIX = $00000040;



type
   //Pfsio = ^fsio;
   //fsio = record
       {undefined structure}
    // end;


   PNXMode_t = ^NXMode_t;
   NXMode_t = dword;

   PNXOFlags_t = ^NXOFlags_t;
   NXOFlags_t = dword;

   PNXShareMode_t = ^NXShareMode_t;
   NXShareMode_t = dword;

   PNXOffset_t = ^NXOffset_t;
   NXOffset_t = Tuint64;
{ (file offsets and lengths)  }

   PNXSOffset_t = ^NXSOffset_t;
   NXSOffset_t = Tint64;

   PNXLockToken_t = ^NXLockToken_t;
   NXLockToken_t = Tuint64;

   PNXHandle_t = ^NXHandle_t;
   NXHandle_t = longint;

   PNXPathCtx_t = ^NXPathCtx_t;
   NXPathCtx_t = longint;

   PNXDelegType_t = ^NXDelegType_t;
   NXDelegType_t = longint;

   PNXAsyncId_t = ^NXAsyncId_t;
   NXAsyncId_t = void;
{ I/O objects supported...  }

   PNXObjType_t = ^NXObjType_t;
   NXObjType_t =  Longint;
   Const
     NX_OBJ_UNKNOWN = $FFFFFFFF;
     NX_OBJ_DEFAULT = $FFFFFFFE;
     NX_OBJ_FILE = $FFFFFFFD;
     NX_OBJ_DIR = $FFFFFFFC;
     NX_OBJ_FIFO = $FFFFFFFB;
     NX_OBJ_DEVICE = $FFFFFFFA;
     NX_OBJ_CONSOLE = $FFFFFFF9;
     NX_OBJ_SYMLINK = $FFFFFFF8;

type

   PNXGuid_t = ^NXGuid_t;
   NXGuid_t = record
       case longint of
          0 : ( guid_field : array[0..1] of Tuint64 );
       end;

   PNXFid_t = ^NXFid_t;
   NXFid_t = record
        fidFsId : NXGuid_t;
        fidFileId : array[0..1] of Tuint64;
     end;

   PNXUpCallReason_t = ^NXUpCallReason_t;
   NXUpCallReason_t =  Longint;
   Const
     NX_UPCALL_UNKNOWN = $FFFFFFFF;
     NX_UPCALL_DELEGRECALL = $00000000;
     NX_UPCALL_BACKUPIMMINENT = $00000001;

 Const
     NX_LOCK_RANGE_UNKNOWN = 2147483647;
     NX_LOCK_RANGE_FORWARD = 1;
     NX_LOCK_RANGE_BACKWARD = 2;

type

   NXFsUpCall_t = procedure (fileHandle:NXHandle_t; reason:NXUpCallReason_t; parm:pointer);cdecl;
{ generic filesystem name if used            }
{ Macintosh data stream (data or resource)   }

   PNXDataStream_t = ^NXDataStream_t;
   NXDataStream_t = record
       case longint of
          0 : ( name : pointer );
          1 : ( macintoshId : dword );
       end;
{ based on which 'fosPathname' is understood }

{ relative to 'fosPathCtx'                   }
{ NX_O_RDONLY, NX_O_RDWR, etc.               }
{ NX_OFLAG_SYNCWR, NX_OFLAG_DIRECTIO, etc.   }
{ NX_SHARE_DENYRD, NX_DENY_ALL, etc.         }
{ contiguous file block allocation hint      }
{ contiguous file block allocation hint      }
{ delegation type: NX_DELEG_NONE             }
{ data stream (if not part of fosPathname)   }
{ called to warn of delegation revocation    }
{ description of successful conclusion       }
{ actual grant of delegation                 }
{ returns FID of opened file                 }
{ time of last access                        }
{ time of last modification                  }
{ time of file creation                     }
{ time of last back-up                       }
{ length of file at open                     }
{ create/open operation results              }

   PNXFileOpenSpec_t = ^NXFileOpenSpec_t;
   NXFileOpenSpec_t = record
        fosPathCtx : NXPathCtx_t;
        fosPathname : pointer;
        fosMode : NXMode_t;
        fosOFlags : NXOFlags_t;
        fosShareMode : NXShareMode_t;
        fosExtentSize : size_t;
        reserved1 : dword;
        fosDelegType : NXDelegType_t;
        fosDataStream : NXDataStream_t;
        fosUpCallFunc : NXFsUpCall_t;
        fosResult : record
             actionTaken : NXMode_t;
             delegType : NXDelegType_t;
             fid : NXFid_t;
             accessTime : NXTime_t;
             modifyTime : NXTime_t;
             creationTime : NXTime_t;
             archiveTime : NXTime_t;
             length : NXOffset_t;
          end;
        reserved2 : dword;
        reserved3 : dword;
     end;
   Pfsio = ^fsio;
   NXIoComp_t = function (ioInfo:Pfsio):longint;cdecl;
{ length of I/O buffer                       }
{ pointer to data for I/O operation          }
{ application-maintained reference           }

   PNXIoVec_t = ^NXIoVec_t;
   NXIoVec_t = record
        ivLength : size_t;
        reserved : dword;
        ivBuffer : pointer;
        ivOpRef : pointer;
     end;
{ I/O objects supported...  }
{ For Internal Use Only                }

   PNXLockRange_t = ^NXLockRange_t;
   NXLockRange_t =  Longint;


{ application-maintained reference           }
{ file, FIFO, console, device, etc. handle   }
{ I/O flags and hints                        }
{ hint as to remaining number of bytes      }
{ from NXFileRangeLockEx()                   }
{ completion function if asynchronous        }
{ only for non-file system operations        }
{ offset at which to begin I/O               }
{ number of records in vector                }
{ IN/OUT: I/O operation record vector        }
{ OUT: asynchronous transaction ID           }
{ OUT: for asynchronous use                  }
{ OUT: total bytes written or read           }

   fsio = record
        ioAppRef : pointer;
        ioHandle : NXHandle_t;
        ioFlags : dword;
        reserved : dword;
        ioRemainingHint : NXOffset_t;
        ioLockToken : NXLockToken_t;
        ioCompletion : NXIoComp_t;
        ioTimeOut : dword;
        ioOffset : NXOffset_t;
        ioVecCount : longint;
        ioVector : PNXIoVec_t;
        ioAsyncID : NXAsyncId_t;
        ioStatus : longint;
        ioProcessed : NXOffset_t;
     end;
   NXIo_t = fsio;
   PNXIo_t = ^NXIo_t;
{ in file (0-based)                         }
{ magnitude of range                        }
{ NX_LOCK_RANGE_FORWARD, ...                }
{ usually 0; reserved to server apps        }
{ in use only with NX_RANGE_LOCK_NETWARE    }

   PNXFileLockDesc_t = ^NXFileLockDesc_t;
   NXFileLockDesc_t = record
        ldOffset : NXOffset_t;
        ldLength : NXOffset_t;
        ldDirection : NXLockRange_t;
        ldToken : NXLockToken_t;
        ldHandle : NXHandle_t;
        reserved : dword;
     end;

   NXLockUpCall_t = procedure (fileHandle:NXHandle_t; appRef:pointer; lockDesc:PNXFileLockDesc_t);cdecl;
{ application-maintained reference          }
{ handle on which file was opened           }
{ mandatory (TRUE) or merely advisory?     }
{ NX_RANGE_LOCK_EXCL, etc.                  }
{ maximum time to wait for lock             }
{ call-back by file system to release lock  }
{ info. on existing conflicting lock  }
{ info. for VM holding conflicting lock }
{ count of locks described in array         }
{ array of lock descriptions                }

   PNXFileRangeLockSpec_t = ^NXFileRangeLockSpec_t;
   NXFileRangeLockSpec_t = record
        rlsAppRef : pointer;
        rlsHandle : NXHandle_t;
        rlsMandatory : NXBool_t;
        rlsFlags : dword;
        rlsTimeOut : dword;
        rlsUpCall : NXLockUpCall_t;
        rlsConflictLockVm : NXVmId_t;
        rlsConflictLock : NXFileLockDesc_t;
        rlsVecCount : longint;
        rlsDescVec : PNXFileLockDesc_t;
     end;

(** unsupported pragma#pragma pack()*)
{ File I/O...  }

function NXClose(handle:NXHandle_t):longint;cdecl;external libc_nlm name 'NXClose';
function NXFileAllocExtentWithHandle(fileHandle:NXHandle_t; offset:NXOffset_t; length:NXOffset_t; flags:dword):longint;cdecl;external libc_nlm name 'NXFileAllocExtentWithHandle';
function NXFileCancelIo(async_id:NXAsyncId_t; wait:NXBool_t; status:PNXBool_t):longint;cdecl;external libc_nlm name 'NXFileCancelIo';
function NXFileFlushBuffers(fileHandle:NXHandle_t; flags:dword; offset:NXOffset_t; length:NXOffset_t):longint;cdecl;external libc_nlm name 'NXFileFlushBuffers';

function NXFileGetLength(pathCtx:NXPathCtx_t; pathname:pointer; length:PNXOffset_t):longint;cdecl;external libc_nlm name 'NXFileGetLength';
function NXFileGetLengthWithHandle(fileHandle:NXHandle_t; length:PNXOffset_t):longint;cdecl;external libc_nlm name 'NXFileGetLengthWithHandle';

function NXFileOpen(pathCtx:NXPathCtx_t; pathname:pointer; mode:NXMode_t; fileHandle:PNXHandle_t):longint;cdecl;external libc_nlm name 'NXFileOpen';
function NXFileOpenEx(openSpec:PNXFileOpenSpec_t; fileHandle:PNXHandle_t):longint;cdecl;external libc_nlm name 'NXFileOpenEx';
function NXFileRangeLock(fileHandle:NXHandle_t; flags:dword; offset:NXOffset_t; length:NXSOffset_t):longint;cdecl;external libc_nlm name 'NXFileRangeLock';
function NXFileRangeLockEx(lockSpec:NXFileRangeLockSpec_t):longint;cdecl;external libc_nlm name 'NXFileRangeLockEx';
function NXFileRangeUnlock(fileHandle:NXHandle_t; flags:dword; offset:NXOffset_t; length:NXSOffset_t):longint;cdecl;external libc_nlm name 'NXFileRangeUnlock';
function NXFileRangeUnlockEx(lockSpec:NXFileRangeLockSpec_t):longint;cdecl;external libc_nlm name 'NXFileRangeUnlockEx';
function NXFileRemoveWithHandle(handle:NXHandle_t):longint;cdecl;external libc_nlm name 'NXFileRemoveWithHandle';

function NXFileRenameWithHandle(fileHandle:NXHandle_t; targetPathCtx:NXPathCtx_t; newname:pointer; overwrite:NXBool_t):longint;cdecl;external libc_nlm name 'NXFileRenameWithHandle';
function NXFileSetLength(pathCtx:NXPathCtx_t; pathname:pointer; length:NXOffset_t; flags:dword):longint;cdecl;external libc_nlm name 'NXFileSetLength';
function NXFileSetLengthWithHandle(fileHandle:NXHandle_t; length:NXOffset_t; flags:dword):longint;cdecl;external libc_nlm name 'NXFileSetLengthWithHandle';
function NXRead(handle:NXHandle_t; offset:NXOffset_t; length:size_t; address:pointer; flags:dword;
           bytesRead:Psize_t):longint;cdecl;external libc_nlm name 'NXRead';
function NXReadEx(ioInfo:PNXIo_t; ioAsyncId:PNXAsyncId_t):longint;cdecl;external libc_nlm name 'NXReadEx';

function NXRemove(pathCtx:NXPathCtx_t; pathname:pointer):longint;cdecl;external libc_nlm name 'NXRemove';


function NXRename(sourcePathCtx:NXPathCtx_t; oldname:pointer; targetPathCtx:NXPathCtx_t; newname:pointer; overwrite:NXBool_t):longint;cdecl;external libc_nlm name 'NXRename';

function NXWrite(handle:NXHandle_t; offset:NXOffset_t; length:size_t; address:pointer; flags:dword;
           bytesWritten:Psize_t):longint;cdecl;external libc_nlm name 'NXWrite';
function NXWriteEx(ioInfo:PNXIo_t; ioAsyncId:PNXAsyncId_t):longint;cdecl;external libc_nlm name 'NXWriteEx';
{ Additional I/O (including FIFOs, devices, etc.)...  }

function NXDeviceOpen(pathCtx:NXPathCtx_t; name:pointer; mode:NXMode_t; shareMode:NXShareMode_t; flags:dword;
           ioBlockSize:Psize_t; deviceHandle:PNXHandle_t):longint;cdecl;external libc_nlm name 'NXDeviceOpen';

function NXFifoOpen(pathCtx:NXPathCtx_t; pathname:pointer; mode:NXMode_t; fifoSize:size_t; fifoHandle:PNXHandle_t):longint;cdecl;external libc_nlm name 'NXFifoOpen';
function NXIoSetBlockingState(handle:NXHandle_t; blocking:NXBool_t):longint;cdecl;external libc_nlm name 'NXIoSetBlockingState';
function NXIoGetOpenMode(handle:NXHandle_t; mode:PNXMode_t):longint;cdecl;external libc_nlm name 'NXIoGetOpenMode';


function NXLinkCreate(srcPathCtx:NXPathCtx_t; source:pointer; tgtPathCtx:PNXPathCtx_t; target:pointer; _type:longint):longint;cdecl;external libc_nlm name 'NXLinkCreate';


function NXLinkCreateSymbolic(srcPathCtx:NXPathCtx_t; linkname:pointer; target:pointer):longint;cdecl;external libc_nlm name 'NXLinkCreateSymbolic';
{ macro for useful alias...  }
{ basic change bits for NXSetAttr()...  }
{ flag values for NXDirAttr_t and NXDeEnum_t, etc...  }
{ deEffectiveRights bits  }

{ turn on 1-byte packing...  }

  { basic change bits for NXSetAttr()...  }

  const
     NX_DIRENT_EFFECTIVERIGHTS = $0000000000000001;
     NX_DIRENT_FLAGS = $0000000000000002;
     NX_DIRENT_OWNERID = $0000000000000004;
     NX_DIRENT_CREATETIME = $0000000000000008;
     NX_DIRENT_CHANGETIME = $0000000000000010;
     NX_DIRENT_ACCESSTIME = $0000000000000020;
     NX_DIRENT_MODIFYTIME = $0000000000000040;
     NX_DIRENT_ATTRIBUTES = $0000000000000080;
     NX_DIRENT_RESERVED1 = $0000000000000100;
     NX_DIRENT_RESERVED2 = $0000000000000200;
     NX_DIRENT_RESERVED3 = $0000000000000400;
     NX_DIRENT_RESERVED4 = $0000000000000800;
  { flag values for NXDirAttr_t and NXDeEnum_t, etc...  }
     NX_DEFLAGS_COMPRESSED = $00000001;
     NX_DEFLAGS_ENCRYPTED = $00000002;
     NX_DEFLAGS_TEMPORARY = $00000004;
     NX_DEFLAGS_MIGRATED = $00000008;
     NX_DEFLAGS_DELETED = $00000010;
     NX_DEFLAGS_SPARSE = $00000020;
  { deEffectiveRights bits  }
     NX_READ_EXISTING_FILE = $00000001;
     NX_WRITE_EXISTING_FILE = $00000002;
     NX_CREATE_NEW_ENTRY = $00000008;
     NX_DELETE_EXISTING_ENTRY = $00000010;
     NX_CHANGE_ACCESS_CONTROL = $00000020;
     NX_SEE_FILES = $00000040;
     NX_MODIFY_ENTRY = $00000080;
     NX_SUPERVISOR_PRIVILEGES = $00000100;
     NX_ACCESS_RIGHTS_MASK = $000001FB;



type

   PNXChangeBits_t = ^NXChangeBits_t;
   NXChangeBits_t = Tuint64;
   TNXChangeBits = NXChangeBits_t;
   PNXChangeBits = PNXChangeBits_t;
{ pathname format (PNF) enumeration         }
{ use PNF in effect at open                 }
{ generic NKS pathname format               }
{ DOS                                       }
{ Macintosh/AFP 3.0                         }
{ UNIX/NFS                                  }
{ Windows NT, OS/2                          }
{ Novell Storage Services (NSS)             }

   PNXPathFormat_t = ^NXPathFormat_t;
   NXPathFormat_t =  Longint;
   Const
     NX_PNF_DEFAULT = 2147483647;
     NX_PNF_NKS = 0;
     NX_PNF_DOS = 1;
     NX_PNF_MAC = 2;
     NX_PNF_UNIX = 4;
     NX_PNF_WIN = 5;
     NX_PNF_NSS = 7;

{ basic attribute information              }
{ filename                                 }
{ extended file system information         }
type

   PNXDeLevel_t = ^NXDeLevel_t;
   NXDeLevel_t =  Longint;
   Const
     NX_DELEVEL_BASIC = $7FFFFFFF;
     NX_DELEVEL_NAME_ONLY = $80000000;
     NX_DELEVEL_EXTENDED = $80000001;

{ marking place in directory enumeration    }
{ all private to implementation             }
type

   PNXDirMark_t = ^NXDirMark_t;
   NXDirMark_t = record
        mark : array[0..43] of byte;
        pathCtx : pointer;
        entryInfo : pointer;
     end;
{ structure size is 52 bytes on NetWare     }
{ information common to all PNF structures  }
{ total length of entry including any       }
{ type of entry (file, FIFO, etc.)          }
{ file system yielding information          }
{ level specified in NXDirEnumStart()       }

   PNXDeHeader_t = ^NXDeHeader_t;
   NXDeHeader_t = record
        length     : size_t;
        objectType : NXObjType_t;
        pathFormat : NXPathFormat_t;
        infoLevel  : NXDeLevel_t;
     end;
   PNXDeHeader = PNXDeHeader_t;
   TNDDeHeader = NXDeHeader_t;
{ common information                        }
{                                           }
{ length of file                            }
{ compressed, sparse, encrypted, etc.       }
{ timestamp of last attribute change        }
{ timestamp of last write+attribute change  }
{ timestamp of last write                   }
{ preferred I/O blocksize                   }

   PNXDirAttr_t = ^NXDirAttr_t;
   NXDirAttr_t = record
        deHeader : NXDeHeader_t;
        deFid : NXFid_t;
        deEffectiveRights : dword;
        deFileSize : NXOffset_t;
        deFlags : Tuint64;
        deAttrChangeTime : NXTime_t;
        deAccessTime : NXTime_t;
        deModifyTime : NXTime_t;
        deIoBlockSize : size_t;
     end;
{ plus name                                 }

   PNXDirAttrWithName_t = ^NXDirAttrWithName_t;
   NXDirAttrWithName_t = record
        deHeader : NXDeHeader_t;
        deName : pointer;
     end;
{ information fields                        }
{ NXDirEnum- navigational mark              }

   PNXDirEnum_t = ^NXDirEnum_t;
   NXDirEnum_t = record
        deDirAttr : NXDirAttr_t;
        deDirMark : NXDirMark_t;
        deName : pointer;
     end;
{ generic NKS (NX_PNF_NKS) attributes       }

   PNXDirAttrNks_t = ^NXDirAttrNks_t;
   NXDirAttrNks_t = record
        xdeHeader : NXDeHeader_t;
        xdeFid : NXFid_t;
        xdeEffectiveRights : dword;
        xdeFileSize : NXOffset_t;
        xdeFlags : Tuint64;
        xdeChangeTime : NXTime_t;
        xdeAccessTime : NXTime_t;
        xdeModifyTime : NXTime_t;
        xdeCreateTime : NXTime_t;
        xdeOwnerId : NXGuid_t;
     end;
{ generic NKS (NX_PNF_NKS) information...   }
{ ...including name                         }

   PNXDirEnumNks_t = ^NXDirEnumNks_t;
   NXDirEnumNks_t = record
        deNksDirAttr : NXDirAttrNks_t;
        deDirMark : NXDirMark_t;
        deName : pointer;
     end;

(** unsupported pragma#pragma pack()*)
{ Path context...  }

function NXFreePathContext(pathCtx:NXPathCtx_t):longint;cdecl;external libc_nlm name 'NXFreePathContext';

function NXCreatePathContext(pathCtx:NXPathCtx_t; pathname:pchar; format:NXPathFormat_t; securityBadge:pointer; newPathCtx:PNXPathCtx_t):longint;cdecl;external libc_nlm name 'NXCreatePathContext';
function NXCreatePathContext(pathCtx:NXPathCtx_t; pathname:pchar; format:NXPathFormat_t; securityBadge:pointer; var newPathCtx:NXPathCtx_t):longint;cdecl;external libc_nlm name 'NXCreatePathContext';
function NXCreatePathContextWithFid(fid:NXFid_t; format:NXPathFormat_t; securityBadge:pointer; newPathCtx:PNXPathCtx_t):longint;cdecl;external libc_nlm name 'NXCreatePathContextWithFid';
{ Directory I/O...  }

function NXDirCreate(pathCtx:NXPathCtx_t; pathname:pointer; reserved:pointer; newPathCtx:PNXPathCtx_t):longint;cdecl;external libc_nlm name 'NXDirCreate';

function NXDirRemove(pathCtx:NXPathCtx_t; pathname:pointer):longint;cdecl;external libc_nlm name 'NXDirRemove';
{ Directory entry attributes...  }

function NXGetAttr(pathCtx:NXPathCtx_t; pathname:pointer; level:NXDeLevel_t; buffer:pointer; length:size_t;
           flags:dword):longint;cdecl;external libc_nlm name 'NXGetAttr';
function NXGetAttrWithHandle(handle:NXHandle_t; format:NXPathFormat_t; level:NXDeLevel_t; buffer:pointer; length:size_t;
           flags:dword):longint;cdecl;external libc_nlm name 'NXGetAttrWithHandle';


function NXSetAttr(pathCtx:NXPathCtx_t; pathname:pointer; level:NXDeLevel_t; buffer:pointer; changeBits:NXChangeBits_t):longint;cdecl;external libc_nlm name 'NXSetAttr';

function NXSetAttrWithHandle(handle:NXHandle_t; format:NXPathFormat_t; level:NXDeLevel_t; buffer:pointer; changeBits:NXChangeBits_t):longint;cdecl;external libc_nlm name 'NXSetAttrWithHandle';
{ Subdirectory enumeration...  }
function NXDirMarkInit(handle:NXHandle_t; dirMark:PNXDirMark_t):longint;cdecl;external libc_nlm name 'NXDirMarkInit';
function NXDirMarkInit(handle:NXHandle_t; var dirMark:NXDirMark_t):longint;cdecl;external libc_nlm name 'NXDirMarkInit';
function NXDirEnumEnd(handle:NXHandle_t):longint;cdecl;external libc_nlm name 'NXDirEnumEnd';
function NXDirEnumGetEntries(handle:NXHandle_t; start:PNXDirMark_t; buffer:pointer; length:size_t; entriesReturned:Psize_t;
           next:PNXDirMark_t; sequenceGuarantee:PNXBool_t):longint;cdecl;external libc_nlm name 'NXDirEnumGetEntries';

function NXDirEnumStart(pathCtx:NXPathCtx_t; pathname:pchar; level:NXDeLevel_t; handle:PNXHandle_t):longint;cdecl;external libc_nlm name 'NXDirEnumStart';
function NXDirEnumStart(pathCtx:NXPathCtx_t; pathname:pchar; level:NXDeLevel_t; var handle:NXHandle_t):longint;cdecl;external libc_nlm name 'NXDirEnumStart';


// nks/doswin.h
// wchar.h
{ turn on 1-byte packing...  }

type

   Pwint_t = ^wint_t;
   wint_t = longint;

   Pwuchar_t = ^wuchar_t;
   wuchar_t = word;

   Pmbstate_t = ^mbstate_t;
   mbstate_t = longint;

(** unsupported pragma#pragma pack()*)


function mbsrtowcs(__restrict:Pwchar_t; __restrict1:PPchar; _para3:size_t; __restrict2:Pmbstate_t):size_t;cdecl;external libc_nlm name 'mbsrtowcs';

{$ifndef DisableArrayOfConst}
function fwprintf(__restrict:P_iobuf; __restrict1:Pwchar_t; args:array of const):longint;cdecl;external libc_nlm name 'fwprintf';
{$endif}
function fwprintf(__restrict:P_iobuf; __restrict1:Pwchar_t):longint;cdecl;external libc_nlm name 'fwprintf';

{$ifndef DisableArrayOfConst}
function fwscanf(__restrict:P_iobuf; __restrict1:Pwchar_t; args:array of const):longint;cdecl;external libc_nlm name 'fwscanf';
{$endif}
function fwscanf(__restrict:P_iobuf; __restrict1:Pwchar_t):longint;cdecl;external libc_nlm name 'fwscanf';

{$ifndef DisableArrayOfConst}
function swprintf(__restrict:Pwchar_t; _para2:size_t; __restrict1:Pwchar_t; args:array of const):longint;cdecl;external libc_nlm name 'swprintf';
{$endif}
function swprintf(__restrict:Pwchar_t; _para2:size_t; __restrict1:Pwchar_t):longint;cdecl;external libc_nlm name 'swprintf';

{$ifndef DisableArrayOfConst}
function swscanf(__restrict:Pwchar_t; __restrict1:Pwchar_t; args:array of const):longint;cdecl;external libc_nlm name 'swscanf';
{$endif}
function swscanf(__restrict:Pwchar_t; __restrict1:Pwchar_t):longint;cdecl;external libc_nlm name 'swscanf';

function vfwprintf(__restrict:P_iobuf; __restrict1:Pwchar_t; _para3:va_list):longint;cdecl;external libc_nlm name 'vfwprintf';

function vfwscanf(_para1:P_iobuf; _para2:Pwchar_t; _para3:va_list):longint;cdecl;external libc_nlm name 'vfwscanf';

function vwscanf(_para1:Pwchar_t; _para2:va_list):longint;cdecl;external libc_nlm name 'vwscanf';
function vswscanf(_para1:Pwchar_t; _para2:Pwchar_t; _para3:va_list):longint;cdecl;external libc_nlm name 'vswscanf';
function wcrtomb(__restrict:Pchar; _para2:wchar_t; __restrict1:Pmbstate_t):size_t;cdecl;external libc_nlm name 'wcrtomb';
function wcscat(__restrict:Pwchar_t; __restrict1:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcscat';
function wcschr(_para1:Pwchar_t; _para2:wchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcschr';
function wcscmp(_para1:Pwchar_t; _para2:Pwchar_t):longint;cdecl;external libc_nlm name 'wcscmp';
function wcscoll(_para1:Pwchar_t; _para2:Pwchar_t):longint;cdecl;external libc_nlm name 'wcscoll';
function wcscpy(__restrict:Pwchar_t; __restrict1:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcscpy';
function wcscspn(_para1:Pwchar_t; _para2:Pwchar_t):size_t;cdecl;external libc_nlm name 'wcscspn';
function wcsftime(__restrict:Pwchar_t; _para2:size_t; __restrict1:Pwchar_t; __restrict2:Ptm):size_t;cdecl;external libc_nlm name 'wcsftime';
function wcslen(_para1:Pwchar_t):size_t;cdecl;external libc_nlm name 'wcslen';
function wcsncat(__restrict:Pwchar_t; __restrict1:Pwchar_t; _para3:size_t):Pwchar_t;cdecl;external libc_nlm name 'wcsncat';
function wcsncmp(_para1:Pwchar_t; _para2:Pwchar_t; _para3:size_t):longint;cdecl;external libc_nlm name 'wcsncmp';
function wcsncpy(__restrict:Pwchar_t; __restrict1:Pwchar_t; _para3:size_t):Pwchar_t;cdecl;external libc_nlm name 'wcsncpy';
function wcspbrk(_para1:Pwchar_t; _para2:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcspbrk';
function wcsrchr(_para1:Pwchar_t; _para2:wchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcsrchr';
function wcsrtombs(__restrict:Pchar; __restrict1:PPwchar_t; _para3:size_t; __restrict2:Pmbstate_t):size_t;cdecl;external libc_nlm name 'wcsrtombs';
function wcsspn(_para1:Pwchar_t; _para2:Pwchar_t):size_t;cdecl;external libc_nlm name 'wcsspn';
function wcsstr(_para1:Pwchar_t; _para2:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcsstr';
function wcstod(__restrict:Pwchar_t; __restrict1:PPwchar_t):double;cdecl;external libc_nlm name 'wcstod';
function wcstof(__restrict:Pwchar_t; __restrict1:PPwchar_t):double;cdecl;external libc_nlm name 'wcstof';
function wcstok(__restrict:Pwchar_t; __restrict1:Pwchar_t; __restrict2:PPwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcstok';
function wcstok_r(__restrict:Pwchar_t; __restrict1:Pwchar_t; __restrict2:PPwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcstok_r';
function wcstol(__restrict:Pwchar_t; __restrict1:PPwchar_t; xx:longint):longint;cdecl;external libc_nlm name 'wcstol';

// long double wcstold(const wchar_t * __restrict1, wchar_t ** __restrict);
// long double wcstold(const wchar_t * __restrict1, wchar_t ** __restrict);

function wcstoll(__restrict:Pwchar_t; __restrict1:PPwchar_t; xx:longint):Tint64;cdecl;external libc_nlm name 'wcstoll';
function wcstoul(__restrict:Pwchar_t; __restrict1:PPwchar_t; _para3:longint):dword;cdecl;external libc_nlm name 'wcstoul';
function wcstoull(__restrict:Pwchar_t; __restrict1:PPwchar_t; _para3:longint):Tuint64;cdecl;external libc_nlm name 'wcstoull';
function wcsxfrm(__restrict:Pwchar_t; __restrict1:Pwchar_t; _para3:size_t):size_t;cdecl;external libc_nlm name 'wcsxfrm';
function wctob(_para1:wint_t):longint;cdecl;external libc_nlm name 'wctob';
function wmemchr(ws:Pwchar_t; wc:wchar_t; n:size_t):Pwchar_t;cdecl;external libc_nlm name 'wmemchr';
function wmemcmp(__restrict:Pwchar_t; __restrict1:Pwchar_t; _para3:size_t):longint;cdecl;external libc_nlm name 'wmemcmp';
function wmemcpy(__restrict:Pwchar_t; __restrict1:Pwchar_t; _para3:size_t):Pwchar_t;cdecl;external libc_nlm name 'wmemcpy';
function wmemmove(_para1:Pwchar_t; _para2:Pwchar_t; _para3:size_t):Pwchar_t;cdecl;external libc_nlm name 'wmemmove';
function wmemset(ws:Pwchar_t; wc:wchar_t; n:size_t):Pwchar_t;cdecl;external libc_nlm name 'wmemset';
function putwc(_para1:wchar_t; _para2:P_iobuf):wint_t;cdecl;external libc_nlm name 'putwc';
function putwchar(_para1:wchar_t):wint_t;cdecl;external libc_nlm name 'putwchar';
function fwide(_para1:P_iobuf; _para2:longint):longint;cdecl;external libc_nlm name 'fwide';
function fputwc(_para1:wchar_t; _para2:P_iobuf):wint_t;cdecl;external libc_nlm name 'fputwc';
function fputws(__restrict:Pwchar_t; __restrict1:P_iobuf):longint;cdecl;external libc_nlm name 'fputws';
function fgetwc(_para1:P_iobuf):wint_t;cdecl;external libc_nlm name 'fgetwc';
function fgetws(__restrict:Pwchar_t; _para2:longint; __restrict1:P_iobuf):Pwchar_t;cdecl;external libc_nlm name 'fgetws';
function getwc(_para1:P_iobuf):wint_t;cdecl;external libc_nlm name 'getwc';
function getwchar:wint_t;cdecl;external libc_nlm name 'getwchar';
function ungetwc(_para1:wint_t; __restrict:P_iobuf):wint_t;cdecl;external libc_nlm name 'ungetwc';

{$ifndef DisableArrayOfConst}
function wprintf(__restrict:Pwchar_t; args:array of const):longint;cdecl;external libc_nlm name 'wprintf';
{$endif}
function wprintf(__restrict:Pwchar_t):longint;cdecl;external libc_nlm name 'wprintf';

{$ifndef DisableArrayOfConst}
function wscanf(__restrict:Pwchar_t; args:array of const):longint;cdecl;external libc_nlm name 'wscanf';
{$endif}
function wscanf(__restrict:Pwchar_t):longint;cdecl;external libc_nlm name 'wscanf';
function vwprintf(__restrict:Pwchar_t; _para2:va_list):longint;cdecl;external libc_nlm name 'vwprintf';
function vswprintf(__restrict:Pwchar_t; _para2:size_t; __restrict1:Pwchar_t; _para4:va_list):longint;cdecl;external libc_nlm name 'vswprintf';
function wcscasecmp(_para1:Pwchar_t; _para2:Pwchar_t):longint;cdecl;external libc_nlm name 'wcscasecmp';
function wcserror(_para1:longint):Pwchar_t;cdecl;external libc_nlm name 'wcserror';
function wcsicmp(_para1:Pwchar_t; _para2:Pwchar_t):longint;cdecl;external libc_nlm name 'wcsicmp';
function wcsindex(_para1:Pwchar_t; _para2:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcsindex';

{$ifndef DisableArrayOfConst}
function wcslist(_para1:Pwchar_t; _para2:Pwchar_t; args:array of const):Pwchar_t;cdecl;external libc_nlm name 'wcslist';
{$endif}
function wcslist(_para1:Pwchar_t; _para2:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcslist';
function wcslwr(_para1:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcslwr';
function wcsmemcmp(__restrict:Pwchar_t; __restrict1:Pwchar_t; _para3:size_t):longint;cdecl;external libc_nlm name 'wcsmemcmp';
function wcsmemcpy(__restrict:Pwchar_t; __restrict1:Pwchar_t; _para3:size_t):Pwchar_t;cdecl;external libc_nlm name 'wcsmemcpy';
function wcsmemmove(_para1:Pwchar_t; _para2:Pwchar_t; _para3:size_t):Pwchar_t;cdecl;external libc_nlm name 'wcsmemmove';
function wcsncasecmp(_para1:Pwchar_t; _para2:Pwchar_t; _para3:size_t):longint;cdecl;external libc_nlm name 'wcsncasecmp';
function wcsnicmp(_para1:Pwchar_t; _para2:Pwchar_t; _para3:size_t):longint;cdecl;external libc_nlm name 'wcsnicmp';
function wcsnset(_para1:Pwchar_t; _para2:longint; _para3:size_t):Pwchar_t;cdecl;external libc_nlm name 'wcsnset';
function wcsrev(_para1:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcsrev';
function wcsrindex(_para1:Pwchar_t; _para2:size_t; _para3:longint):Pwchar_t;cdecl;external libc_nlm name 'wcsrindex';
function wcsset(_para1:Pwchar_t; _para2:longint):Pwchar_t;cdecl;external libc_nlm name 'wcsset';
function wcsupr(_para1:Pwchar_t):Pwchar_t;cdecl;external libc_nlm name 'wcsupr';
procedure wungettok(__restrict:Pwchar_t; __restrict1:Pwchar_t; __restrict2:PPwchar_t);cdecl;external libc_nlm name 'wungettok';
procedure wungettok_r(__restrict:Pwchar_t; __restrict1:Pwchar_t; __restrict2:PPwchar_t);cdecl;external libc_nlm name 'wungettok_r';
function btowc(_para1:longint):wint_t;cdecl;external libc_nlm name 'btowc';
function mbrlen(__restrict:Pchar; _para2:size_t; __restrict1:Pmbstate_t):longint;cdecl;external libc_nlm name 'mbrlen';
function mbrtowc(__restrict:Pwchar_t; __restrict1:Pchar; _para3:size_t; __restrict2:Pmbstate_t):longint;cdecl;external libc_nlm name 'mbrtowc';
function mbsinit(_para1:Pmbstate_t):longint;cdecl;external libc_nlm name 'mbsinit';
function wcwidth(_para1:wchar_t):longint;cdecl;external libc_nlm name 'wcwidth';
function wcswidth(_para1:Pwchar_t; _para2:size_t):longint;cdecl;external libc_nlm name 'wcswidth';
{ macros that overcome implementation's function call overhead...  }
{ attributes for NXDirAttrDos_t 'attrib' field...  }


// fsio.h
// netware.h
  const
     LD_MODULE_REENTRANT_BIT       = $00000001;
     LD_MODULE_MULTIPLE_LOAD       = $00000002;       { multiple load            }
     LD_SYNCHRONIZE_START          = $00000004;       { uses SynchronizeStart()  }
     LD_PSEUDOPREEMPTION_BIT       = $00000008;       { accept preemption        }
     LD_KERNEL_LOAD                = $00000010;       { ring 0-only              }
     LD_DONT_SHARE_CODE            = $00000020;
  { once dependencies gone   }
     LD_AUTO_UNLOAD                = $00000040;
     LD_HIDDEN_MODULE              = $00000080;
     LD_DIGITALLY_SIGNED_FILE      = $00000100;
     LD_PROTECTED_LOAD             = $00000200;       { protected address space  }
     LD_SHARED_LIBRARY             = $00000400;
     LD_RESTARTABLE                = $00000800;
     LD_MODULE_HAS_MPK_STUBS       = $00001000;
     LD_NOT_MULTIPROCESSOR_SAFE    = $00002000;
     LD_PREEMPTABLE                = $00004000;
     LD_HAS_SYSTEM_CALLS           = $00008000;
     LD_VIRTUAL_MEMORY             = $00010000;
     LD_ALL_EXPORTS_SAFE           = $00020000;
     LD_RESERVED_1                 = $00040000;
     LD_RESERVED_2                 = $00080000;
     LD_RESERVED_3                 = $00100000;
     LD_RESERVED_4                 = $00200000;
     LD_WANT_POSIX_SEMANTICS       = $00400000;       { assume POSIX semantics   }
     LD_UTF8_STRINGS               = $00800000;       { UTF-8 strings            }
     LD_TSR                        = $01000000;       { terminate-stay-resident  }
     LD_PROMPT_USER_AND_PASSWORD   = $02000000;       { with LD_WANT_POSIX...    }
     LD_HOTSWAP_DRIVER             = $04000000;
     LD_STARTUP_DEVICE_NLM_BIT     = $08000000;
     LD_BOUND_NLM_BIT              = $10000000;
     LD_DONT_UNLOAD_BIT            = $20000000;       { never unload             }
     LD_MODULE_BEING_DEBUGGED      = $40000000;       { (debugging)              }
     LD_MEMORY_ON_4K_BOUNDRIES_BIT = $80000000;       { (debugging)              }
  { Note LD_UNICODE_STRINGS mostly obsolete--used only by NKS applications:   }
     LD_UNICODE_STRINGS            = $02000000;       { Unicode strings          }


type
   Prtag_t = ^rtag_t;
   rtag_t = pointer;
   Trtag = rtag_t;
   Prtag = Prtag_t;

   {this is from the lan driver sdk () cmsm.h

    but be warned, like the ScanScreen function available in
    clib, there is no MP save way to access netware screens, a
    comment from Russell Bateman (libc developer @ novell) dated
    way to do screen discovery on NetWare. }

   PScreenStruct = ^TScreenStruct;
   TScreenStruct = packed record
     previousScreen       : PScreenStruct;
     nextScreen           : PScreenStruct;
     popUpOriginalScreen  : PScreenStruct;
     CLIBScreenStructure  : pdword;
     currentPalette       : byte;
     _Filler1             : byte;
     popUpCount           : byte;
     _Filler2             : byte;
     screenList           : byte;
     _Filler3             : byte;
     activeCount          : byte;
     _Filler4             : byte;
     resourceTag          : Prtag_t;
     screenName           : pchar;
     screenMemory         : pointer;
     flags                : dword;
     state                : dword;
     outputCursorPosition : word;
     inputCursorPosition  : word;
   end;

   Pscr_t = ^scr_t;
   scr_t = PScreenStruct;
   TScr = scr_t;
   PScr = Pscr_t;

// event.h
// screen.h

  const
     MAX_SCREEN_STRING_LEN = 255;
  { screen mode flags...  }
     SCR_NO_MODE           = $00000000;
     SCR_AUTOCLOSE_ON_EXIT = $00000001;   // default
     SCR_COLOR_ATTRS       = $00000002;
  { key types...  }
     NORMAL_KEY        = $00;
     FUNCTION_KEY      = $01;
     ENTER_KEY         = $02;
     ESCAPE_KEY        = $03;
     BACKSPACE_KEY     = $04;
     DELETE_KEY        = $05;
     INSERT_KEY        = $06;
     CURSOR_UP_KEY     = $07;
     CURSOR_DOWN_KEY   = $08;
     CURSOR_RIGHT_KEY  = $09;
     CURSOR_LEFT_KEY   = $0A;
     CURSOR_HOME_KEY   = $0B;
     CURSOR_END_KEY    = $0C;
     CURSOR_PUP_KEY    = $0D;
     CURSOR_PDOWN_KEY  = $0E;
  { some name equivalents...  }
     ENTER             = $0D;
     ESCAPE            = $1B;
     BACKSPACE         = $08;
  { modifier code constituents...  }
     SHIFT_KEY_HELD    = $01;
     CTRL_KEY_HELD     = $04;
     ALT_KEY_HELD      = $08;
     CAPS_LOCK_IS_ON   = $40;
     NUM_LOCK_IS_ON    = $20;
     SCROLL_LOCK_IS_ON = $10;
     _PASSWORD_LEN     = 128;  // suggested 'maxlen' argument for getpassword()...
  { string-embeddable color representations...  }
     COLOR_STR_BLACK   = '\x1B[0;30m';
     COLOR_STR_MAROON  = '\x1B[0;31m';
     COLOR_STR_GREEN   = '\x1B[0;32m';
     COLOR_STR_OLIVE   = '\x1B[0;33m';
     COLOR_STR_NAVY    = '\x1B[0;34m';
     COLOR_STR_PURPLE  = '\x1B[0;35m';
     COLOR_STR_TEAL    = '\x1B[0;36m';
     COLOR_STR_SILVER  = '\x1B[0;37m';
     COLOR_STR_GREY    = '\x1B[1;30m';
     COLOR_STR_RED     = '\x1B[1;31m';
     COLOR_STR_LIME    = '\x1B[1;32m';
     COLOR_STR_YELLOW  = '\x1B[1;33m';
     COLOR_STR_BLUE    = '\x1B[1;34m';
     COLOR_STR_MAGENTA = '\x1B[1;35m';
     COLOR_STR_CYAN    = '\x1B[1;36m';
     COLOR_STR_WHITE   = '\x1B[1;37m';

     COLOR_STR_NORMAL  = COLOR_STR_SILVER;  // dim/unhighlighted white
     COLOR_STR_GRAY    = COLOR_STR_GREY;
  { attributes for OutputToScreenWithAttributes(); cf. HTML color names  }

     COLOR_ATTR_NONE   = 0;            // black, no color at all
     COLOR_ATTR_NAVY   = 1;            // dim blue
     COLOR_ATTR_BLUE   = $01 or 8;
     COLOR_ATTR_GREEN  = 2;
     COLOR_ATTR_LIME   = 2 or 8;       // bright green
     COLOR_ATTR_TEAL   = 3;            // dim cyan
     COLOR_ATTR_CYAN   = 3 or 8;
     COLOR_ATTR_MAROON = 4;            // dim red
     COLOR_ATTR_RED    = 4 or 8;
     COLOR_ATTR_PURPLE = 5;
     COLOR_ATTR_MAGENTA= 5 or 8;       // bright purple
     COLOR_ATTR_OLIVE  = 6;            // brown, dim yellow
     COLOR_ATTR_YELLOW = 6 or 8;
     COLOR_ATTR_SILVER = 7;            // normal white, dim/unhighlighted
     COLOR_ATTR_GREY   = 8;            // dimmed white
     COLOR_ATTR_WHITE  = 15;           // bright, highlighted white

  const
//     OutputToScreenWithVaList = OutputToScreenWithPointer;
  { return and default values for Prompt functions...  }
     SCR_PROMPT_ANSWER_NO   = 0;
     SCR_PROMPT_ANSWER_YES  = 1;
     SCR_PROMPT_ANSWER_SKIP = 2;
     SCR_PROMPT_ANSWER_ALL  = 3;

  { cursor types...  }
     CURSOR_NORMAL = $0C0B;
     CURSOR_THICK  = $0C09;
     CURSOR_BLOCK  = $0C00;
     CURSOR_TOP    = $0400;
  { screen types...  }
     SCREEN_TYPE_TTY         = $00000000;
     SCREEN_TYPE_MONOCHROME  = $00000001;
     SCREEN_TYPE_DUAL_MODE   = $00000002;
     SCREEN_TYPE_CGA         = $00000003;
     SCREEN_TYPE_EGA         = $00000004;
     SCREEN_TYPE_VGA         = $00000005;
  { screen modes...  }
     SCREEN_MODE_TTY         = $00000000;
     SCREEN_MODE_80X25       = $00000001;
     SCREEN_MODE_80X43       = $00000002;
     SCREEN_MODE_80X50       = $00000003;
     SCREEN_MODE_D           = $0000000D;
     SCREEN_MODE_E           = $0000000E;
     SCREEN_MODE_F           = $0000000F;
     SCREEN_MODE_10          = $00000010;
     SCREEN_MODE_11          = $00000011;
     SCREEN_MODE_12          = $00000012;
     SCREEN_MODE_13          = $00000013;
  { voracious for memory!  }
     SCREEN_MODE_SCROLLABLE  = $80000000;
  { screen state...  }
     SCREEN_NON_SWITCHABLE   = $00000001;
     SCREEN_DUMMY            = $00000100;
     SCREEN_HIDDEN           = $00000200;

procedure clearscreen;cdecl;external libc_nlm name 'clearscreen';

{$ifndef DisableArrayOfConst}
function consoleprintf(txt:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'consoleprintf';
function consoleprintf(txt:Pchar):longint;cdecl;external libc_nlm name 'consoleprintf';
{$endif}
function __consoleprintfl1(txt:Pchar;p1:longint):longint;cdecl;external libc_nlm name 'consoleprintf';
function __consoleprintfl2(txt:Pchar;p1,p2:longint):longint;cdecl;external libc_nlm name 'consoleprintf';
function __consoleprintfl3(txt:Pchar;p1,p2,p3:longint):longint;cdecl;external libc_nlm name 'consoleprintf';
function getscreenmode(mode:Pdword):longint;cdecl;external libc_nlm name 'getscreenmode';
function getscreenmode(var mode:dword):longint;cdecl;external libc_nlm name 'getscreenmode';
function getcharacter:longint;cdecl;external libc_nlm name 'getcharacter';
function getkey(keytype,modifer,scancode:Plongint):longint;cdecl;external libc_nlm name 'getkey';
function getkey(var keytype, modifer, scancode:longint):longint;cdecl;external libc_nlm name 'getkey';
function getalternateconsole:scr_t;cdecl;external libc_nlm name 'getalternateconsole';
function getnetwareconsole:scr_t;cdecl;external libc_nlm name 'getnetwareconsole';
function getnetwarelogger:scr_t;cdecl;external libc_nlm name 'getnetwarelogger';

function getpassword(prompt,password:Pchar; maxlen:size_t):Pchar;cdecl;external libc_nlm name 'getpassword';
function getscreenhandle:scr_t;cdecl;external libc_nlm name 'getscreenhandle';
function getstring(_string:Pchar; max:size_t; display:longint):Pchar;cdecl;external libc_nlm name 'getstring';
function gotorowcol(row, col:longint):longint;cdecl;external libc_nlm name 'gotorowcol';
function kbhit:longint;cdecl;external libc_nlm name 'kbhit';
function pressanykey:longint;cdecl;external libc_nlm name 'pressanykey';
function pressanykeytocontinue:longint;cdecl;external libc_nlm name 'pressanykey';
function pressescape:longint;cdecl;external libc_nlm name 'pressescape';
function pressenter:longint;cdecl;external libc_nlm name 'pressenter';
function putcharacter(ch:longint):longint;cdecl;external libc_nlm name 'putcharacter';

function putstring(_string:Pchar):longint;cdecl;external libc_nlm name 'putstring';

{$ifndef DisableArrayOfConst}
function screenprintf(_para1:scr_t; _para2:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'screenprintf';
{$endif}
function screenprintf(_para1:scr_t; _para2:Pchar):longint;cdecl;external libc_nlm name 'screenprintf';
function screenprintf(_para1:scr_t; _para2:Pchar; l1:longint):longint;cdecl;external libc_nlm name 'screenprintf';
function screenprintf(_para1:scr_t; _para2:Pchar; l1,l2:longint):longint;cdecl;external libc_nlm name 'screenprintf';
function screenprintf(_para1:scr_t; _para2:Pchar; l1,l2,l3:longint):longint;cdecl;external libc_nlm name 'screenprintf';

function setscreenmode(mode:dword):longint;cdecl;external libc_nlm name 'setscreenmode';

function renamescreen(name:Pchar):longint;cdecl;external libc_nlm name 'renamescreen';
procedure ringbell;cdecl;external libc_nlm name 'ringbell';
function ungetcharacter(ch:longint):longint;cdecl;external libc_nlm name 'ungetcharacter';
function ungetkey(keytype:longint; modifer:longint; scancode:longint):longint;cdecl;external libc_nlm name 'ungetkey';

function vconsoleprintf(_para1:Pchar; _para2:va_list):longint;cdecl;external libc_nlm name 'vconsoleprintf';

function vscreenprintf(_para1:scr_t; _para2:Pchar; _para3:va_list):longint;cdecl;external libc_nlm name 'vscreenprintf';
function wherecol:longint;cdecl;external libc_nlm name 'wherecol';
function whererow:longint;cdecl;external libc_nlm name 'whererow';
function whererowcol(row,col:Plongint):longint;cdecl;external libc_nlm name 'whererowcol';
function whererowcol(var row,col:longint):longint;cdecl;external libc_nlm name 'whererowcol';
{ obsolete...  }
function getconsolehandle:scr_t;cdecl;external libc_nlm name 'getconsolehandle';
{ direct NetWare OS interfaces...  }
type

   Pscroll_t = ^scroll_t;
   scroll_t =  Longint;
   Const
     SCROLL_DOWN = 0;
     SCROLL_UP   = 1;
     SCROLL_NONE = 2147483647;


procedure ActivatePopUpScreen(scrID:scr_t);cdecl;external libc_nlm name 'ActivatePopUpScreen';
procedure ActivateScreen(scrID:scr_t);cdecl;external libc_nlm name 'ActivateScreen';
procedure ChangeToSystemConsoleScreen;cdecl;external libc_nlm name 'ChangeToSystemConsoleScreen';
function CheckIfScreenActive(scrID:scr_t; waitFlag:dword):longint;cdecl;external libc_nlm name 'CheckIfScreenActive';
function CheckKeyStatus(scrID:scr_t):longint;cdecl;external libc_nlm name 'CheckKeyStatus';
procedure ClearScreen(scrID:scr_t);cdecl;external libc_nlm name 'ClearScreen';
procedure CloseScreen(scrID:scr_t);cdecl;external libc_nlm name 'CloseScreen';
procedure ConsoleHungMenu;cdecl;external libc_nlm name 'ConsoleHungMenu';
procedure DisableInputCursor(scrID:scr_t);cdecl;external libc_nlm name 'DisableInputCursor';
function DisplayScreenLine(scrID:scr_t; line:dword; col:dword; length:dword; textAndAttr:Pbyte):longint;cdecl;external libc_nlm name 'DisplayScreenLine';
function DisplayScreenText(scrID:scr_t; line:dword; col:dword; length:dword; text:Pchar):longint;cdecl;external libc_nlm name 'DisplayScreenText';
function DisplayScreenTextWithAttribute(scrID:scr_t; line:dword; col:dword; length:dword; lineAttr:byte;
           text:Pchar):longint;cdecl;external libc_nlm name 'DisplayScreenTextWithAttribute';
procedure EnableInputCursor(scrID:scr_t);cdecl;external libc_nlm name 'EnableInputCursor';
procedure EndPopUpScreen(scr:scr_t);cdecl;external libc_nlm name 'EndPopUpScreen';
function FillScreenArea(scrID:scr_t; line:dword; col:dword; height:dword; width:dword;
           character:char; attr:byte):longint;cdecl;external libc_nlm name 'FillScreenArea';
function FillScreenAreaAttribute(scrID:scr_t; line:dword; col:dword; height:dword; width:dword;
           attr:byte):longint;cdecl;external libc_nlm name 'FillScreenAreaAttribute';
function GetActiveScreen:scr_t;cdecl;external system_nlm name 'GetActiveScreen';
function GetActualScreenSize(scrID:scr_t; height:Pdword; width:Pdword; bufferSize:Psize_t):longint;cdecl;external system_nlm name 'GetActualScreenSize';
function GetConsoleSecuredFlag:longint;cdecl;external libc_nlm name 'GetConsoleSecuredFlag';
procedure GetCursorStyle(scrID:scr_t; cursorStyle:Pword);cdecl;external system_nlm name 'GetCursorStyle';
procedure GetCursorStyle(scrID:scr_t; var cursorStyle:word);cdecl;external system_nlm name 'GetCursorStyle';
procedure GetInputCursorPosition(scrID:scr_t; row:Pword; col:Pword);cdecl;external system_nlm name 'GetInputCursorPosition';
procedure GetKey(scrID:scr_t; _type,value,status,scancode:Pbyte;linesToProtect:size_t);cdecl;external system_nlm name 'GetKey';
procedure GetKey(scrID:scr_t; var _type,value,status,scancode:byte;linesToProtect:size_t);cdecl;external system_nlm name 'GetKey';
procedure GetOutputCursorPosition(scrID:scr_t; row,col:Pword);cdecl;external system_nlm name 'GetOutputCursorPosition';
procedure GetOutputCursorPosition(scrID:scr_t; var row,col:word);cdecl;external system_nlm name 'GetOutputCursorPosition';
function GetRawKeyWithScreen(scrID:scr_t; _type,value,status,scancode:Pbyte):longint;cdecl;external libc_nlm name 'GetRawKeyWithScreen';
function GetRawKeyWithScreen(scrID:scr_t; var _type,value,status,scancode:byte):longint;cdecl;external libc_nlm name 'GetRawKeyWithScreen';
function GetScreenAddress:pointer;cdecl;external system_nlm name 'GetScreenAddress';  // not in protected mode
function GetScreenName(scrID:scr_t; nameBuffer:Pchar):longint;cdecl;external system_nlm name 'GetScreenName';
function GetScreenPhysicalAddress:pointer;cdecl;external libc_nlm name 'GetScreenPhysicalAddress';
procedure GetScreenSize(height,width:Pword);cdecl;external system_nlm name 'GetScreenSize';
procedure GetScreenSize(var height,width:word);cdecl;external system_nlm name 'GetScreenSize';

{$ifndef DisableArrayOfConst}
function InputFromScreen(scrID:scr_t; allowedCharacterSet:Pchar; bufferLength:size_t; editWidth:size_t; buffer:Pchar;
           linesToProtect:longint; hasDefaultString:longint; defaultString:Pchar; promptText:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'InputFromScreen';
{$endif}
function InputFromScreen(scrID:scr_t; allowedCharacterSet:Pchar; bufferLength:size_t; editWidth:size_t; buffer:Pchar;
           linesToProtect:longint; hasDefaultString:longint; defaultString:Pchar; promptText:Pchar):longint;cdecl;external libc_nlm name 'InputFromScreen';
function IsScreenModeSupported(screenMode:dword):longint;cdecl;external system_nlm name 'IsScreenModeSupported';

function OpenCustomScreen(name:Pchar; rTag:rtag_t; newScrID:Pscr_t; mode:longint):longint;cdecl;external libc_nlm name 'OpenCustomScreen';

function OpenPopUpScreen(name:Pchar; rTag:rtag_t; newScrID:Pscr_t):longint;cdecl;external libc_nlm name 'OpenPopUpScreen';

function OpenScreen(name:Pchar; rTag:rtag_t; newScrID:Pscr_t):longint;cdecl;external system_nlm name 'OpenScreen';

{$ifndef DisableArrayOfConst}
function OutputToScreen(scrID:scr_t; format:Pchar; args:array of const):longint;cdecl;external system_nlm name 'OutputToScreen';
{$endif}
function OutputToScreen(scrID:scr_t; format:Pchar):longint;cdecl;external system_nlm name 'OutputToScreen';

{$ifndef DisableArrayOfConst}
function OutputToScreenWithAttribute(scrID:scr_t; attr:byte; format:Pchar; args:array of const):longint;cdecl;external system_nlm name 'OutputToScreenWithAttribute';
{$endif}
function OutputToScreenWithAttribute(scrID:scr_t; attr:byte; format:Pchar):longint;cdecl;external system_nlm name 'OutputToScreenWithAttribute';


function OutputToScreenWithPointer(scrID:scr_t; format:Pchar; arguments:va_list):longint;cdecl;external system_nlm name 'OutputToScreenWithPointer';
procedure Pause(scrID:scr_t);cdecl;external system_nlm name 'Pause';
function PauseWithEscape(scrID:scr_t):longint;cdecl;external system_nlm name 'PauseWithEscape';
procedure PositionInputCursor(scrID:scr_t; row:word; col:word);cdecl;external system_nlm name 'PositionInputCursor';
function PositionOutputCursor(scrID:scr_t; row:word; col:word):longint;cdecl;external system_nlm name 'PositionOutputCursor';

function PromptForString(scr:scr_t; length:size_t; editWidth:size_t; _string:Pchar; hasDefaultValue:longint;
           defaultValue:Pchar; linesToProtect:longint; promptText:pointer):Pchar;cdecl;external libc_nlm name 'PromptForString';

function PromptForPassword(scr:scr_t; prompt:Pchar; blotOutChar:longint; password:Pchar; maxlen:size_t):Pchar;cdecl;external libc_nlm name 'PromptForPassword';

{$ifndef DisableArrayOfConst}
function PromptForUnsignedNumber(scrID:scr_t; result:Pdword; minValue:dword; maxValue:dword; radix:longint;
           linesToProtect:longint; hasDefaultValue:byte; defaultValue:dword; promptText:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'PromptForUnsignedNumber';
{$endif}
function PromptForUnsignedNumber(scrID:scr_t; result:Pdword; minValue:dword; maxValue:dword; radix:longint;
           linesToProtect:longint; hasDefaultValue:byte; defaultValue:dword; promptText:Pchar):longint;cdecl;external libc_nlm name 'PromptForUnsignedNumber';

{$ifndef DisableArrayOfConst}
function PromptForYesOrNo(scrID:scr_t; linesToProtect,defaultValue:longint; promptText:pointer; args:array of const):longint;cdecl;external libc_nlm name 'PromptForYesOrNo';
{$endif}
function PromptForYesOrNo(scrID:scr_t; linesToProtect,defaultValue:longint; promptText:pointer):longint;cdecl;external libc_nlm name 'PromptForYesOrNo';

{$ifndef DisableArrayOfConst}
function PromptForYesNoAllOrSkip(scrID:scr_t; linesToProtect:longint; defaultValue:dword; promptText:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'PromptForYesNoAllOrSkip';
{$endif}
function PromptForYesNoAllOrSkip(scrID:scr_t; linesToProtect:longint; defaultValue:dword; promptText:Pchar):longint;cdecl;external libc_nlm name 'PromptForYesNoAllOrSkip';
function ReadScreenCharacter(scrID:scr_t; line,col:dword; character:Pchar):longint;cdecl;external system_nlm name 'ReadScreenCharacter';

function RenameScreen(scrID:scr_t; name:Pchar):longint;cdecl;external system_nlm name 'RenameScreen';
function RestoreFullScreen(scrID:scr_t; buffer:pointer):longint;cdecl;external system_nlm name 'RestoreFullScreen';
function RestoreScreenArea(scrID:scr_t; line,col,height,width:dword; buffer:pointer):longint;cdecl;external system_nlm name 'RestoreScreenArea';
procedure ReturnScreenType(_type,colorFlag:Pdword);cdecl;external system_nlm name 'ReturnScreenType';
procedure ReturnScreenType(var _type,colorFlag:dword);cdecl;external system_nlm name 'ReturnScreenType';
procedure RingTheBell;cdecl;external system_nlm name 'RingTheBell';
function SaveFullScreen(scrID:scr_t; buffer:pointer):longint;cdecl;external system_nlm name 'SaveFullScreen';
function SaveScreenArea(scrID:scr_t; line,col,height,width:dword; buffer:pointer):longint;cdecl;external system_nlm name 'SaveScreenArea';
procedure SetConsoleSecuredFlag(value:byte);cdecl;external system_nlm name 'SetConsoleSecuredFlag';
procedure SetCursorStyle(scrID:scr_t; newStyle:word);cdecl;external system_nlm name 'SetCursorStyle';
procedure SetInputToOutputCursorPosition(scrID:scr_t);cdecl;external system_nlm name 'SetInputToOutputCursorPosition';
function ScrollScreenArea(scrID:scr_t; line,col,height,width,count:dword; newLineAttribute:byte; direction:scroll_t):longint;cdecl;external system_nlm name 'ScrollScreenArea';

procedure ShowTitleBarText(screenMemoryAddress:pointer; titleBarText:Pchar; textLength:size_t);cdecl;external system_nlm name 'ShowTitleBarText';
function UngetKey(scrID:scr_t; _type,value,status,scancode:byte):longint;cdecl;external system_nlm name 'UngetKey';
function ValidateScreenHandle(scrID:scr_t):longint;cdecl;external system_nlm name 'ValidateScreenHandle';
procedure WaitForKey(scrID:scr_t);cdecl;external system_nlm name 'WaitForKey';
function WriteScreenCharacter(scrID:scr_t; line,col:dword; character:char):longint;cdecl;external system_nlm name 'WriteScreenCharacter';
function WriteScreenCharacterAttribute(scrID:scr_t; line,col:dword; character:char; attr:byte):longint;cdecl;external system_nlm name 'WriteScreenCharacterAttribute';

{ turn on 1-byte packing...  }



{ file server event notification interfaces...  }
const
     EVENT_VOL_SYS_MOUNT = 0;
     EVENT_VOL_SYS_DISMOUNT = 1;
     EVENT_ANY_VOL_MOUNT = 2;
     EVENT_ANY_VOL_DISMOUNT = 3;
     EVENT_DOWN_SERVER = 4;        { (see EVENT_PRE_DOWN_SERVER)  }
     EVENT_EXIT_TO_DOS = 7;
     EVENT_MODULE_UNLOAD = 8;
     EVENT_MODULE_UNLOAD_POST_EXIT = 115;
     EVENT_CLEAR_CONNECTION = 9;
     EVENT_LOGIN_USER = 10;
     EVENT_CREATE_BINDERY_OBJ = 11;
     EVENT_DELETE_BINDERY_OBJ = 12;
     EVENT_CHANGE_SECURITY = 13;
     EVENT_CHANGE_SECURITY_ADD_EQUIVALENCE = 1;
     EVENT_ACTIVATE_SCREEN = 14;
     EVENT_UPDATE_SCREEN = 15;
     EVENT_UPDATE_CURSOR = 16;
     EVENT_KEY_WAS_PRESSED = 17;
     EVENT_DEACTIVATE_SCREEN = 18;
     EVENT_TRUSTEE_CHANGE = 19;
     EVENT_NO_RELINQUISH_CONTROL = 23;
     EVENT_THREAD_SWITCH = 25;
     EVENT_MODULE_LOAD = 27;
     EVENT_CREATE_PROCESS = 28;
     EVENT_DESTROY_PROCESS = 29;
     EVENT_NEW_PUBLIC = 32;
     EVENT_PROTOCOL_BIND = 33;
     EVENT_PROTOCOL_UNBIND = 34;
     EVENT_ALLOCATE_CONNECTION = 37;
     EVENT_LOGOUT_CONNECTION = 38;
     EVENT_MLID_REGISTER = 39;
     EVENT_MLID_DEREGISTER = 40;
     EVENT_DATA_MIGRATION = 41;
     EVENT_DATA_DEMIGRATION = 42;
     EVENT_CREATE_OBJECT = 46;
     EVENT_DELETE_OBJECT = 47;
     EVENT_RENAME_OBJECT = 48;
     EVENT_VALUE_CHANGE = 49;
     EVENT_MOVE_OBJECT = 52;
     EVENT_VALUE_ADD = 53;
     EVENT_VALUE_DEL = 54;
     EVENT_CHANGE_TIME = 51;
     EVENT_DM_KEY_MODIFIED = 55;
     EVENT_MODULE_UNLOADED = 56;
     EVENT_REMOVE_PUBLIC = 57;
     EVENT_UNICODE = 59;
     EVENT_SFT3_SERVER_STATE = 60;
     EVENT_SFT3_IMAGE_STATE = 61;
     EVENT_SFT3_PRESYNC_STATE = 62;
     EVENT_ALTERNATE_MOUNT_VOLUME = 63;
     EVENT_CONSOLE_CONFIG_COMMAND = 64;
     EVENT_CONSOLE_VERSION_COMMAND = 65;
     EVENT_PRE_LOAD_NLM = 66;
     EVENT_LOW_MEMORY = 67;
     EVENT_PRE_DOWN_SERVER = 129;     // called before NDS disappears
     EVENT_GET_KEY_INFORMATION = 148;
     EGKStructVersion =  $00000001;
     EVENT_PRIORITY_APPLICATION = 20;
     EVENT_PRIORITY_DEVICE = 40;      // lowest

  {** A consumer registering for an event sets this flag in the event type to
   ** denote that the consumer is multiprocessor safe. This allows us to
   ** distinguish multiprocessor safe consumers from unsafe ones. }
     EVENT_CONSUMER_MT_SAFE     = $40000000;
  { ** Values for fields of EventNetWareAlertStruct_t. }
     QueueThisAlertMask         = $00000001;
     AlertIDValidMask           = $00000002;
     AlertLocusValidMask        = $00000004;
     AlertEventNotifyOnlyMask   = $00000008;
     AlertNoEventNotifyMask     = $00000010;
     AlertMessageNumberValid    = $00010000;
     NoDisplayAlertUID          = $00200000;
     AlertNoRingBell            = $00400000;
     AlertIDNotUniqueBit        = $00800000;
     OldStyleSystemAlertMask    = $01000000;
     OldStyleINWSystemAlertMask = $02000000;
  { can be pointer or number  }
     OverloadMessageNumFieldBit = $04000000;
     NoDisplayLocusBit          = $10000000;
     NoDisplayAlertIDBit        = $20000000;
     OverrideNotificationBits   = $40000000;
     TargetStationIsAPointer    = $80000000;
     Alert300Mask = (AlertIDValidMask or AlertLocusValidMask) or OldStyleSystemAlertMask;
     Alert311Mask = AlertIDValidMask or OldStyleINWSystemAlertMask;
     //Alert320Mask = ((AlertIDValidMask or AlertMessageNumberValid) or AlertLocusValidMask) or NoDisp
     //QAlertMask = ((AlertIDValidMask or AlertLocusValidMask) or NoDisplayAlertIDBit) or QueueThisAle
     //QAlert320Mask = Alert320Mask or QueueThisAlertMask;
 { allotted ModuleNumbers for 'alertID' in EventNetWareAlertStruct_t...  }
     ALERT_BINDERY    = $01020000;   // Bindary
     ALERT_OS         = $01030000;   // OS Event Subject
     ALERT_LLC        = $01040000;   // LLC
     ALERT_SDLC       = $01050000;   // SDLC Stack
     ALERT_REMOTE     = $01060000;   // RConsole
     ALERT_MLID       = $01070000;   // MLID Lan Drivers
     ALERT_QLLC       = $01080000;   // QLLC
     ALERT_UPS        = $01090000;   // UPS Monitor
     ALERT_DS         = $010a0000;   // Directory Service
     ALERT_RSPX       = $010c0000;   // RSPX
     ALERT_R232       = $010d0000;   // Serial
     ALERT_TIME_SYNC  = $010e0000;   // TimeSync
     ALERT_CLIB       = $010f0000;   // Clib
     ALERT_PRINT      = $01100000;   // Print
     ALERT_NRS        = $01200000;   // Novell Replication Services
     ALERT_DNS        = $01300000;   // IP/Domain Name Services
     ALERT_DHCP       = $01400000;   // DHCP Services
     ALERT_MM         = $01500000;   // Media Manager
{
  ** OS-defined AlertNumber values for 'alertID' in EventNetWareAlertStruct_t
  ** structure.
   }
  { starting with NetWare 4...  }
     nmAllocFailed = 1;
     nmErrWrtExtDir = 2;
     nmSysErrWrtDSnoFN = 3;
     nmStaErrWrtDSnoFN = 4;
     nmSysErrWrtDSwithFN = 5;
     nmStaErrWrtDSwithFN = 6;
     nmSysErrRdDSnoFN = 7;
     nmStaErrRdDSnoFN = 8;
     nmSysErrRdDSwithFN = 9;
     nmStaErrRdDSwithFN = 10;
     nmSysWrtPreRDnoFN = 11;
     nmStaWrtPreRDnoFN = 12;
     nmSysWrtPreRDwithFN = 13;
     nmStaWrtPreRDwithFN = 14;
     nmCacheMemLimitExceded = 15;
     nmCacheMemOutOfMem = 16;
     nmCacheBufsGetLo = 17;
     nmDskSpcNoDelFiles = 18;
     nmDskSpcNoLimbo = 19;
     nmVolSpcAlmostGone = 20;
     nmFATWrtErr = 21;
     nmDirWrtErr = 22;
     nmDirCopyRdErr = 23;
     nmDirDblRdErr = 24;
     nmAllocDirWrtErr = 25;
     nmDirExpansionErr = 26;
     nmDirTooLarge = 27;
     nmErrExpandingDir = 28;
     nmErrExpandingMem = 29;
     nmErrDirGetTooLarge = 30;
     nmDskBottleneck = 31;
     nmWDClearedConn = 32;
     nmCpyrtViolation = 33;
     nmReadFault = 35;
     nmPktTooSmall = 36;
     nmCreatingVolLog = 37;
     nmWrtVolLog = 38;
     nmVolDmtDevDeact = 39;
     nmLoginDisabled = 40;
     nmLoginEnabled = 41;
     nmClrSta = 42;
     nmClrStaByUsr = 43;
     nmFSDownByUser = 44;
     nmRIPAlreadyOpn = 45;
     nmRouterConfigErr = 46;
     nmLANLoopbackErr = 47;
     nmRouterConfigErrNoInfo = 48;
     nmIPXUnreachable = 49;
     nmIPXUnbind = 50;
     nmSAPAlreadyOpn = 51;
     nmRouterConfigErrNameInfo = 52;
     nmSpuriousInt = 53;
     nmChecksumInvalidAlert = 54;
     nmPrimaryPicLostInt = 55;
     nmSecondaryPicLostInt = 56;
     nmCompErrHoleCountMismatch = 57;
     nmInvalidScreen = 58;
     nmRelinquishControl = 59;
     nmFSUserDeleted = 60;
     nmAccDelByUser = 61;
     nmInvalidRTag = 62;
     nmDeactUnknown = 63;
     nmDeactDriveUnld = 64;
     nmDeactDevFailure = 65;
     nmDeactUsrRequest = 66;
     nmDeactMediaDismount = 67;
     nmDeactMediaEject = 68;
     nmDeactServerDown = 69;
     nmDeactServerFailure = 70;
     nmResourceRelErr = 71;
     nmMirrorsNotSync = 72;
     nmMirrorsSyncUp = 73;
     nmPartMirrorSync = 74;
     nmPartMirrorNotSync = 75;
     nmReMirroringPart = 76;
     nmReMirroringPartAborted = 77;
     nmLogPartMirrorInconsist = 78;
     nmSysFileLockThresh = 79;
     nmStaFileLockThresh = 80;
     nmSysRecLockThresh = 81;
     nmStaRecLockThresh = 82;
     nmOpnNETACCTFailed = 83;
     nmNCPSearchLimitSys = 84;
     nmNCPSearchLimitSta = 85;
     nmInsMediaAck = 86;
     nmInsMediaAborted = 87;
     nmRemMediaAck = 88;
     nmRemMediaAbort = 89;
     nmInsMediaInto = 90;
     nmRemMediaFrom = 91;
     nmReDirectedBlockPart = 92;
     nmReDirectedBlockPartErr = 93;
     nmOutOfHotFixBlocks = 94;
     nmLowWarningHotFixBlocks = 95;
     nmReDirectInconsistNoFix = 96;
     nmReDirectInconsistFixed = 97;
     nmInvalidRTagHOptions = 98;
     nmCheckAndAddHWNoGetRTag = 99;
     nmRemHWBadPtr = 100;
     nmErrUnldNLM = 101;
     nmIvldRTagCrProc = 102;
     nmCrProcStkTooSmall = 103;
     nmCrProcNoPCB = 104;
     nmDelToLimboFileErr = 105;
     nmDelToLimboNoSpace = 106;
     nmMLIDResetLanBd = 107;
     nmRouterReset = 108;
     nmVolWrongDOSType = 109;
     nmNoOwnerNSfound = 110;
     nmRTDMDefSMchanged = 111;
     nmErrOpnTTSLOG = 112;
     nmErrWrtTTSLOG = 113;
     nmTTSdownVolDismount = 114;
     nmTTSdisableByStaUsr = 115;
     nmTTSdisByOp = 116;
     nmTTSdisErrRdBackFile = 117;
     nmTTSdisErrWrBackFile = 118;
     nmTTSdisTooManyDefVol = 119;
     nmTTSdisWrtVolDefInfo = 120;
     nmTTSdisErrRdBkFlRecGen = 121;
     nmTTSdisGrowMemTables = 122;
     nmTTSdisErrAllDiskSp = 123;
     nmTTSdisDirErrOnBkFile = 124;
     nmTTSEnableByStaUsr = 125;
     nmTTStransAbortedForSta = 126;
     nmTTStooManyTransDelaying = 127;
     nmTTSNoMemForExpTransNodes = 128;
     nmAuditEvent = 129;
     nmAuditDisNoAuditCfg = 130;
     nmInvldConnTypeToAllocConn = 131;
     nmInvldRTagToAllocConn = 132;
     nmOutOfServerConns = 133;
     nmConnTermAfter5Min = 134;
     nmUsrAccDisableBySta = 135;
     nmUnEncryptPwdNotAllowed = 136;
     nmSuperAccLockedByConsole = 137;
     nmSystemTimeChangedByCon = 138;
     nmSystemTimeChangedBySta = 139;
     nmVolStillActWithError = 140;
     nmRouterFalsehood = 141;
     nmServerAddressChanged = 142;
     nmExtFileNoOwnerCharge = 143;
     nmRouterConfigErrNode = 144;
     nmRouterConfigErrMyAddr = 145;
     nmNoMigratorLd = 146;
     nmNoSMLd = 147;
     nmNotEnoughRamForCompression = 148;
     nmDiskErrorCompressing = 149;
     nmUnknownErrorCompressing = 150;
     nmInsufficientSpaceForDeCompression = 151;
     nmDecompressUnknownCompressionVersion = 152;
     nmUnknownDecompressError = 153;
     nmInsufficientRAMToDecompress = 154;
     nmCompressedFileIsCorrupt = 155;
     nmStaAttemptedToUseBadPckt = 156;
     nmStaUsedABadPckt = 157;
     nmStaAttemptedToUseBadSFL = 158;
     nmStaUsedABadSFL = 159;
     nmCorruptCompFileWithName = 160;
     nmCorruptCompFileWithNameAndStation = 161;
     nmLowPriThreadsNotRun = 162;
     nmWorkToDoNotRun = 163;
     nmCompressErrorTempFileError = 164;
     nmCompressErrorLengthTotalsMismatch = 165;
     nmCompressErrorOffsetTotalsMismatch = 166;
     nmCompressErrorDataCodeCountMismatch = 167;
     nmCompressErrorLengthCountMismatch = 168;
     nmCompressErrorLargeLengthCountMismatch = 169;
     nmCompressErrorReadZeroBytesOrg = 170;
     nmCompressErrorTreeTooBig = 171;
     nmCompressErrorMatchSizeFail = 172;
     nmSignatureInvalidAlert = 173;
     nmLicenseIsInvalid = 174;
     nmDeactHotFixError = 175;
     nmUnknownDecompressErrorFN = 176;
     nmInsufficientRAMToDecompressFN = 177;
     nmDecompressUnderFreePercentage = 178;
     nmNegPktTriedLargeBuffer = 179;
     nmLoginDisabledByConsole = 180;
     nmLoginEnabledByConsole = 181;
     nmGrwStkNotAvail = 182;
     nmLicenseFileIsMissing = 183;
     nmFailedToDeletedMigratedFile = 184;
     nmNoMemForAuditing = 185;
     nmAuditFileWriteError = 186;
     nmAuditFileFull = 187;
     nmAuditFileThresholdOverflow = 188;
     nmCompressErrorReadZeroBytesInt = 189;
     nmEASpaceLimit = 190;
     nmThreadAreaNotEmpty = 191;
     nmErrMovingLogToMSEngine = 192;
     nmFaultInConsoleCmdHandler = 193;
     nmServerToServerComLinkActivated = 194;
     nmServerToServerComLinkFailure = 195;
     nmServerToServerComLinkDeact = 196;
     nmOtherServerAttemptedToSync = 197;
     nmServerToServerComLinkBrokeOK = 198;
     nmServerSyncStartingIAmSecondary = 199;
     nmBadSvrInitMsgFromOtherSvr = 200;
     nmSvrToSvrCommLinkInitFailed = 201;
     nmFailedDuringSyncWithReason = 202;
     nmCommDrvLdDuringActivateWait = 203;
     nmErrWritingStatusDump = 204;
     nmComDrvFailureOnPrimary = 205;
     nmComDrvFailureOnSecondary = 206;
     nmErrFinishingGenStatusDump = 207;
     nmSFTIIWhatToDoWithReasonString = 208;
     nmSFTIIErrorUnexpected = 209;
     nmSyncErrFromCustomServerNLM = 210;
     nmSvrLinkHasPluggedPacket = 211;
     nmSvrToBeRevived = 212;
     nmServersAreSyncPri = 213;
     nmSvrCantRouteIPXSec = 214;
     nmSrvIPXRouteInfoSec = 215;
     nmErrGivingRAMtoMS = 216;
     nmMoreRAMgivenToMS = 217;
     nmServersAreSyncSec = 218;
     nmSvrCantRouteIPXPri = 219;
     nmSrvIPXRouteInfoPri = 220;
     nmPriSvrFailedButSecDown = 221;
     nmPriSvrFailedNewPri = 222;
     nmNumMemSegsExceedLimit = 223;
     nmNumScreenExceedsLimit = 224;
     nmIOVersionMismatch = 225;
     nmOtherSvrProtectLvlNoMatch = 226;
     nmOtherSvrScrAddrMismatch = 227;
     nmIOEngNotAtSameAddr = 228;
     nmBothSvrHaveMSEng = 229;
     nmNoMSEngOnServers = 230;
     nmSecSvrMissingRAM = 231;
     nmBothSrvHaveSameIPXAddr = 232;
     nmIOEngIPXAddrMatchMSEng = 233;
     nmIOEngsMismatchRxSizes = 234;
     nmIOEngsHaveSameName = 235;
     nmNoMemForIOEngName = 236;
     nmSrvToSvrLinkBeginSync = 237;
     nmMSEngActivated = 238;
     nmMSEngActNowSyncOther = 239;
     nmIOtoMSComMisMatchUnload = 240;
     nmSFTIIIOutOfMsgCodes = 241;
     nmErrXferDumpToSystem = 242;
     nmFailureChkPrimary = 243;
     nmNoMemForOtherIOEngScr = 244;
     nmErrStarting2ndProc = 245;
     nmSrvFailureMsg = 246;
     nmSecIOEngSupModNotLd = 247;
     nmMSLBdNumHasConn = 248;
     nmSecSvrLANIsBetter = 249;
     nmIPXrtnStatusPckts = 250;
     nmIPXnotRtnStatChkPckts = 251;
     nmIPXnotRtnStatLANJam = 252;
     nmFailReasonByOtherSrv = 253;
     nmIPXMayBeTooSlowForSecSrv = 254;
     nmIPXToOtherSrvTooManyHops = 255;
     nmIPXappearsDown = 256;
     nmIPXFoundRouteToOtherSrv = 257;
     nmIPXLostRoute = 258;
     nmSecSrvGoingToDie = 259;
     nmPriSrcDyingTimerStart = 260;
     nmPriSrvDying = 261;
     nmIPXInternetIsJammed = 262;
     nmIPXNewRouteToSecSvr = 263;
     nmSrvsSyncing = 264;
     nmFSHookRegistered = 265;
     nmFSHookDeRegistered = 266;
     nmIOEngCantBorrowMemory = 267;
     nmDecompressNoCompressionOnVolume = 268;
     nmMkProcessUsingTooSmallStk = 269;
     nmQueueEventReportNoMemory = 270;
     nmServerPartMirrorNotSync = 271;
     nmStaWithoutRightsConsoleRPC = 272;
     nmAuditOverflowFileThreshold = 273;
     nmAuditOverflowFileFull = 274;
     nmSwitchStacksGrwStk = 275;
     nmConsoleCommandProcRestarted = 276;
     nmGrowableStackGrew = 278;
     nmOtherSvrIOLogSpaceNoMatch = 279;
     nmDFSLogicalStackRead = 280;
     nmDFSLogicalStackWrite = 281;
     nmSecureEraseFailure = 282;
     nmDropBadPktBurstConn = 283;
     nmOutOfIPXSockets = 284;
     nmVolumeObjectIDChanged = 285;
     nmAbendRecovery = 286;
     nmOpLockTimeout = 287;
     nmAbendRecovered = 288;
     nmUnknownSetCmd = 289;    { starting with NetWare 5...  }
     nmAddressSpaceProtectionFault = 290;
     nmAddressSpaceFailedToRestart = 291;
     nmAddressSpaceRestarted = 292;
     nmCorruptMemoryNodeDetected = 293;
     nmAddressSpaceCleanupFailure = 294;
     nmInvalidParameter = 295;
     nmInvalidObjectHandle = 296;
     nmNullPointer = 297;
     nmVolDmtMedDmt = 298;
     nmVolDmtmedChgd = 299;
     nmAccDelByUsrActConn = 300;
     nmResourcesRelErr = 301;
     nmDemoVersion = 302;
     nmDemoVersionTooLong = 303;
     nmLicenseReSellerFileIsMissing = 304;
     nmLicenseUpgradeIsMissing = 305;
     nmLicenseVersionInvalid = 306;
     nmLicenseProductInvalid = 307;
     nmLicenseNoMoreFiles = 308;
     nmLicensePIDInvalid = 309;
     nmLicenseContentInalid = 310;
     nmLicenseBadUpgrade = 311;
     nmLicensePrevMaxConnMisMatch = 312;
     nmLicenseContentResellerBad = 313;
     nmLicenseSNMisMatch = 314;
     nmLicenseUIDMisMatch = 315;
     nmLicenseOpenError = 316;
     nmLicenseCompanionErr = 317;
     nmLicenseSNUpgradeMisMatch = 318;
     nmLicenseUnableToRemMSL = 319;
     nmLicenseUnableToRemULF = 320;
     nmLicenseUnableToRemRLF = 321;
     nmLicenseUnableToGetFileSize = 322;
     nmLicenseUnkLicenseType = 323;
     nmLicenseReadErr = 324;
     nmLicenseFileSizeMisMatch = 325;
     nmLicenseDupServerLic = 326;
     nmLicenseNeedUpgrade = 327;
     nmLicenseMirrorNeedUpgrade = 328;
     nmLicenseDupLicDiscovered = 329;
     nmLicenseDupLicDiscoveredDel = 330;
     nmLicenseCpyRightViolated = 331;
     nmLicenseExpired = 332;
     nmVolDmtDevMedChgd = 333;
     nmVolDmtDevMedDmt = 334;
     nmInsMediaAckDS = 335;
     nmInsMediaAckMag = 336;
     nmInsMediaAbortedDS = 337;
     nmInsMediaAbortedMag = 338;
     nmRemMediaAckDS = 339;
     nmRemMediaAckMag = 340;
     nmRemMediaAbortDS = 341;
     nmRemMediaAbortMag = 342;
     nmInsMediaIntoDS = 343;
     nmInsMediaIntoMag = 344;
     nmRemMediaFromDS = 345;
     nmRemMediaFromMag = 346;
     nmServAddr = 347;
     nmSwapInError = 348;
     nmSwapOutError = 349;
     nmAveragePageInThresholdExceeded = 350;
     nmIllegalRequest = 351;
     nmTTSThrottleDelayError = 352;
     nmTTSLackOfResourcesError = 353;
     nmTTSLackOfResourcesNoReason = 354;
     nmDelayedWTDNotRunning = 355;
     nmInvalidCharacterInName = 356;
  { starting with NetWare 6...  }
     nmMPKBadThreadState = 357;
     nmPoolSeriousError = 358;
     nmPoolSeriousReadError = 359;
     nmVolSeriousError = 360;
     nmVolSeriousReadError = 361;
     nmVolDeactSeriousIOError = 362;
     nmVolDeactSeriousNonIOError = 363;
     nmPoolDeactSeriousIOError = 364;
     nmPoolDeactSeriousNonIOError = 365;
     nmTaskZeroCheck = 366;
  { values for 'alertLocus' in EventNetWareAlertStruct_t...  }
     LOCUS_UNKNOWN = 0;
     LOCUS_MEMORY = 1;
     LOCUS_FILESYSTEM = 2;
     LOCUS_DISKS = 3;
     LOCUS_LANBOARDS = 4;
     LOCUS_COMSTACKS = 5;
     LOCUS_TTS = 7;
     LOCUS_BINDERY = 8;
     LOCUS_STATION = 9;
     LOCUS_ROUTER = 10;
     LOCUS_LOCKS = 11;
     LOCUS_KERNEL = 12;
     LOCUS_UPS = 13;
     LOCUS_SERVICE_PROTOCOL = 14;
     LOCUS_SFT_III = 15;
     LOCUS_RESOURCE_TRACKING = 16;
     LOCUS_NLM = 17;
     LOCUS_OS_INFORMATION = 18;
     LOCUS_CACHE = 19;
  { values for 'alertClass' in EventNetWareAlertStruct_t...  }
     CLASS_UNKNOWN = 0;
     CLASS_OUT_OF_RESOURCE = 1;
     CLASS_TEMP_SITUATION = 2;
     CLASS_AUTHORIZATION_FAILURE = 3;
     CLASS_INTERNAL_ERROR = 4;
     CLASS_HARDWARE_FAILURE = 5;
     CLASS_SYSTEM_FAILURE = 6;
     CLASS_REQUEST_ERROR = 7;
     CLASS_NOT_FOUND = 8;
     CLASS_BAD_FORMAT = 9;
     CLASS_LOCKED = 10;
     CLASS_MEDIA_FAILURE = 11;
     CLASS_ITEM_EXISTS = 12;
     CLASS_STATION_FAILURE = 13;
     CLASS_LIMIT_EXCEEDED = 14;
     CLASS_CONFIGURATION_ERROR = 15;
     CLASS_LIMIT_ALMOST_EXCEEDED = 16;
     CLASS_SECURITY_AUDIT_INFO = 17;
     CLASS_DISK_INFORMATION = 18;
     CLASS_GENERAL_INFORMATION = 19;
     CLASS_FILE_COMPRESSION = 20;
     CLASS_PROTECTION_VIOLATION = 21;
     CLASS_VIRTUAL_MEMORY = 22;
  { values for 'alertSeverity' in EventNetWareAlertStruct_t...  }
  { counters reached thresholds          }
     SEVERITY_INFORMATIONAL = 0;
  { config errors, etc. no damage        }
     SEVERITY_WARNING = 1;
  { hot fix disk, etc. worked around     }
     SEVERITY_RECOVERABLE = 2;
  { disk mirror failure, etc. fix-up attempted  }
     SEVERITY_CRITICAL = 3;
  { resource fatally affected--shut down }
     SEVERITY_FATAL = 4;
  { cannot complete--result unknown      }
     SEVERITY_OPERATION_ABORTED = 5;
  { cannot complete--will not affect OS  }
     SEVERITY_NONOS_UNRECOVERABLE = 6;
  { values for 'targetNotificationBits' in EventNetWareAlertStruct_t...  }
     NOTIFY_CONNECTION_BIT = $00000001;
     NOTIFY_EVERYONE_BIT = $00000002;
     NOTIFY_ERROR_LOG_BIT = $00000004;
     NOTIFY_CONSOLE_BIT = $00000008;
  { use if alert to be queued  }
     NOTIFY_QUEUE_MESSAGE = $10000000;
     NOTIFY_DONT_NOTIFY_NMAGENT = $80000000;

type
  TnwAlertDataFreeProc = procedure (nwAlertDataPtr:pointer);cdecl;
  PNetWareAlertStructure  = ^TNetWareAlertStructure;
  TNetWareAlertStructure = record
    pNetworkManagementAttribute  : pointer;
    nwAlertFlags,
    nwTargetStation,
    nwTargetNotificationBits,
    nwAlertID,
    nwAlertLocus,
    nwAlertClass,
    nwAlertSeverity              : longint;
    nwAlertDataPtr               : pointer;
    nwAlertDataFree              : TnwAlertDataFreeProc;
    nwControlString              : pchar;
    nwControlStringMessageNumber : longint;
  end;

{$ifndef INCLUDED_FROM_SYSTEM}
procedure NetWareAlert(nlmHandle      : TNLMHandle;
                       nwAlert        : PNetWareAlertStructure;
                       parameterCount : longint;
                       args           : array of const); cdecl; external system_nlm name 'NetWareAlert';
{$endif}

procedure NetWareAlert(nlmHandle      : TNLMHandle;
                       nwAlert        : PNetWareAlertStructure;
                       parameterCount : longint); cdecl; external system_nlm name 'NetWareAlert';

type

   PEventSecurityChange_t = ^EventSecurityChange_t;
   EventSecurityChange_t = record
        objectID : dword;
        equivalentID : dword;
        changeFlags : dword;
     end;

   PEventTrusteeChange_t = ^EventTrusteeChange_t;
   EventTrusteeChange_t = record
        objectID : dword;
        entryID : dword;
        volumeNumber : longint;
        changeFlags : dword;
        newRights : dword;
     end;

   PEventModifyDirEntry_t = ^EventModifyDirEntry_t;
   EventModifyDirEntry_t = record
        primaryDirectoryEntry : dword;
        nameSpace : longint;
        modifyBits : dword;
        modifyVector : pointer;
        volumeNumber : longint;
        directoryEntry : dword;
     end;

   PEventProtocolBind_t = ^EventProtocolBind_t;
   EventProtocolBind_t = record
        boardNumber : longint;
        protocolNumber : longint;
     end;

   PEventDateMigrationInfo_t = ^EventDateMigrationInfo_t;
   EventDateMigrationInfo_t = record
        FileSystemTypeID : dword;
        Volume : longint;
        DOSDirEntry : dword;
        OwnerDirEntry : dword;
        OwnerNameSpace : longint;
        OwnerFileName : array[0..(1 + 255)-1] of char;
     end;

   PEventQueueNote_t = ^EventQueueNote_t;
   EventQueueNote_t = record
        QAction : longint;
        QID : dword;
        QName : array[0..(49 + 1)-1] of char;
     end;

   PEventNetWareAlert_t = ^EventNetWareAlert_t;
   EventNetWareAlert_t = record
        alertFlags : dword;
        alertId : dword;
        alertLocus : dword;
        alertClass : dword;
        alertSeverity : dword;
        targetStationCount : longint;
        targetStationList : array[0..31] of dword;
        targetNotificationBits : dword;
        alertParmCount : longint;
        alertDataPtr : pointer;
        NetWorkManagementAttributePointer : pointer;
        alertUnused : array[0..1] of dword;
        alertControlStringMessageNumber : longint;
        alertControlString : array[0..255] of char;
        alertParameters : array[0..(256 + 256)-1] of char;
        alertModuleName : array[0..35] of char;
        alertModuleMajorVersion : longint;
        alertModuleMinorVersion : longint;
        alertModuleRevision : longint;
     end;

   PEventBinderyObject_t = ^EventBinderyObject_t;
   EventBinderyObject_t = record
        EventObjectSignature : dword;
        ObjectID : dword;
        ObjectType : dword;
     end;

function DSAllocateEventTag(signature:dword):pointer;cdecl;external libc_nlm name 'DSAllocateEventTag';

type

   PEventDSObject_t = ^EventDSObject_t;
   EventDSObject_t = record
        EventObjectSignature : dword;
        EventType : dword;
        entry : pointer;
     end;

   PEventCloseFileInfo_t = ^EventCloseFileInfo_t;
   EventCloseFileInfo_t = record
        fileHandle : dword;
        station : longint;
        task : longint;
        fileHandleFlags : dword;
        completionCode : longint;
     end;

   PEventCheckKeyInfo_t = ^EventCheckKeyInfo_t;
   EventCheckKeyInfo_t = record
        structVersion : dword;
        keyData : dword;
        scrID : scr_t;
     end;
{
** A consumer registering for an event sets this flag in the event type to
** denote that the consumer is multiprocessor safe. This allows us to
** distinguish multiprocessor safe consumers from unsafe ones.
 }
{
** Values for fields of EventNetWareAlertStruct_t.
 }
{
** Values for 'alertID' in EventNetWareAlertStruct_t comprised of two parts,
** the ModuleNumber (upper 16 bits) and AlertNumber (lower 16 bits).
** AlertNumber is under the control of the module specified by ModuleNumber.
**
** ModuleNumber specifies which module is generating the alert and AlertNumber
** can then be used for a specific alert generated by that module.
**
** Any ModuleNumber in the range 0x8000xxxx to 0xFFFFxxxx. Range 0x0001xxxx
** through 0x7fffxxxx is reserved for Novell. Value 0x0000xxxx is considered
** invalid and is used for all legacy alerts.
 }
{ allotted ModuleNumbers for 'alertID' in EventNetWareAlertStruct_t...  }
{
** OS-defined AlertNumber values for 'alertID' in EventNetWareAlertStruct_t
** structure. }

   Pevent_handle_t = ^event_handle_t;
   event_handle_t = longint;

   Report_t = procedure (parm:pointer; userParm:pointer);cdecl;

   Warn_t = function (printf:pointer; parm:pointer; userParm:pointer):longint;cdecl;

function RegisterForEventNotification(rtag:rtag_t; _type:longint; priority:longint; warn:Warn_t; report:Report_t;
           userParm:pointer):event_handle_t;cdecl;external system_nlm name 'RegisterForEventNotification';
function UnRegisterEventNotification(handle:event_handle_t):longint;cdecl;external system_nlm name 'UnRegisterEventNotification';
function CanEventBlock(_type:longint):longint;cdecl;external system_nlm name 'CanEventBlock';
function CheckForRegisteredEvent(_type:longint):longint;cdecl;external system_nlm name 'CheckForRegisteredEvent';
function EventCheck(_type:longint; printf:pointer; parm:pointer):longint;cdecl;external system_nlm name 'EventCheck';
function EventReport(_type:longint; parm:pointer):longint;cdecl;external system_nlm name 'EventReport';


{ server interfaces...  }
{ ShutdownServer() 'flags'...  }

    const
       NW_POST65_MAX_CMDLINE_LEN = 2048 + 1;
       SHUTDOWN_RESET            = $00000001;   { ShutdownServer() 'flags'...  }
       SHUTDOWN_POWEROFF         = $00000002;   { unimplemented }


procedure RestartServer(commandLine:Pchar);cdecl;external system_nlm name 'RestartServer';
function ShutdownServer(scrID:scr_t; forceDown:byte; alternateMessage:Pchar; flags:dword):longint;cdecl;external system_nlm name 'ShutdownServer';
function ShutdownServer(scrID:scr_t; forceDown:boolean; alternateMessage:Pchar; flags:dword):longint;cdecl;external system_nlm name 'ShutdownServer';
function StopServer(scrID:scr_t; forceDown:byte; reserved1:dword; reserved2:pointer; alternateMessage:Pchar;
           reserved3:dword):longint;cdecl;external system_nlm name 'StopServer';
function StopServer(scrID:scr_t; forceDown:boolean; reserved1:dword; reserved2:pointer; alternateMessage:Pchar;
           reserved3:dword):longint;cdecl;external system_nlm name 'StopServer';

{ resource tag interfaces...  }
const
  AllocSignature               = $54524C41;
  BreakpointSignature          = $54504B42;
  ConsoleCommandSignature      = $4D4F4343;
  CommandLineServicesSignature = $5043574E;
  DebugCommandSignature        = $53504344;
  DebuggerSignature            = $47554244;
  EventSignature               = $544E5645;
  ScreenSignature              = $4E524353;
  SettableParameterSignature   = $4D505453;
  RTAG_DESC_LEN                = 63;


type
   Cleanup_t = procedure (rTag:rtag_t; forceFlag:longint);cdecl;

   Prtag_info = ^rtag_info;
   rtag_info = record
        tag         : rtag_t;
        signature   : dword;
        NLMHandle   : TNLMHandle;
        use_count   : longint;
        description : array[0..(63 + 1)-1] of char;
     end;
   rtag_info_t  = rtag_info;
   Prtag_info_t = ^rtag_info_t;


function AllocateResourceTag(NLMHandle:TNLMHandle; description:Pchar; signature:dword):rtag_t;cdecl;external system_nlm name 'AllocateResourceTag';
function GetModuleResourceTagInfo(rTag:rtag_t; NLMHandle:TNLMHandle; info:Prtag_info_t):longint;cdecl;external system_nlm name 'GetModuleResourceTagInfo';
function ReturnResourceTag(rTag:rtag_t; displayErrorsFlag:longint):longint;cdecl;external system_nlm name 'ReturnResourceTag';
function RegisterTrackedResource(NLMHandle:TNLMHandle; signature:dword; cleanup:Cleanup_t; description:Pchar):longint;cdecl;external system_nlm name 'RegisterTrackedResource';
function UnRegisterTrackedResource(NLMHandle:TNLMHandle; signature:dword):longint;cdecl;external system_nlm name 'UnRegisterTrackedResource';
function AddPollingProcedureRTag(proc:TCDeclProcedure ; rTag:rtag_t):longint;cdecl;external system_nlm name 'AddPollingProcedureRTag';
procedure RemovePollingProcedure(proc:TCDeclProcedure);cdecl;external system_nlm name 'RemovePollingProcedure';

{ NetWare-loader interfaces...  }
const
  MAX_SYMBOL_NAME_LEN = 80;

function ExportPublicObject(NLMHandle:TNLMHandle; name:Pchar; _object:pointer):longint;cdecl;external system_nlm name 'ExportPublicObject';
function ImportPublicObject(NLMHandle:TNLMHandle; name:Pchar):pointer;cdecl;external system_nlm name 'ImportPublicObject';

    const
       LO_NORMAL = $00000000;
       LO_PROTECT = $00000002;
       LO_LOAD_LOW = $00000020;
       LO_RETURN_HANDLE = $00000040;
       LO_LOAD_SILENT = $00000080;
       LO_RESTART = $00000200;
       LO_DONT_DISPLAY_ERROR = $00002000;
       LO_MEMORY_DEBUG = $00010000;          { debug only; no production use  }
       LO_RELAXED_MEMORY_DEBUG = $00020000;  { debug only; no production use  }
       ERR_LOADER_COULD_NOT_FIND_FILE = 1;   { error returns from LoadModule()  }
       ERR_LOADER_ERROR_READING_FILE = 2;
       ERR_LOADER_NOT_NLM_FILE_FORMAT = 3;
       ERR_LOADER_WRONG_NLM_FILE_VERSION = 4;
       ERR_LOADER_REENTRANT_INITIALIZE_FAILURE = 5;
       ERR_LOADER_CAN_NOT_LOAD_MULTIPLE_COPIES = 6;
       ERR_LOADER_ALREADY_IN_PROGRESS = 7;
       ERR_LOADER_NOT_ENOUGH_MEMORY = 8;
       ERR_LOADER_INITIALIZE_FAILURE = 9;
       ERR_LOADER_INCONSISTENT_FILE_FORMAT = 10;
       ERR_LOADER_CAN_NOT_LOAD_AT_STARTUP = 11;
       ERR_LOADER_AUTO_LOAD_MODULES_NOT_LOADED = 12;
       ERR_LOADER_UNRESOLVED_EXTERNAL = 13;
       ERR_LOADER_PUBLIC_ALREADY_DEFINED = 14;
       ERR_LOADER_XDC_DATA_ERROR = 15;
       ERR_LOADER_NOT_KERNEL = 16;
       ERR_LOADER_NIOS_ONLY_NLM = 17;
       ERR_LOADER_ADDRESS_SPACE_CREATION = 18;
       ERR_LOADER_INITIALIZE_FAULT = 19;
type
  TLoadModulePath = record
    case longint of
      0 : (NLMHandle : TNLMHandle);
      1 : (path      : array [0..1024] of char);
    end;


function GetNLMNames(NLMHandle:TNLMHandle; name:Pchar; description:Pchar):longint;cdecl;external system_nlm name 'GetNLMNames';
procedure KillMe(NLMHandle:TNLMHandle);cdecl;external system_nlm name 'KillMe';
function ReturnMessageInformation(NLMHandle:TNLMHandle; table:PPPchar; stringCount:Psize_t; languageID:Plongint; helpFile:pointer):longint;cdecl;external system_nlm name 'ReturnMessageInformation';
function SetAutoUnloadFlag(NLMHandle:TNLMHandle):longint;cdecl;external system_nlm name 'SetAutoUnloadFlag';

function UnImportPublicObject(NLMHandle:TNLMHandle; name:Pchar):longint;cdecl;external system_nlm name 'UnImportPublicObject';
function AddSearchPathAtEnd(scrID:scr_t; path:Pchar):longint;cdecl;external system_nlm name 'AddSearchPathAtEnd';
function DeleteSearchPath(scrID:scr_t; searchPathNumber:longint):longint;cdecl;external system_nlm name 'DeleteSearchPath';
function GetSearchPathElement(index:longint; isDOSFlag:Pdword; path:Pchar):longint;cdecl;external system_nlm name 'GetSearchPathElement';
function GetSearchPathElement(index:longint; var isDOSFlag:dword; path:Pchar):longint;cdecl;external system_nlm name 'GetSearchPathElement';
function GetSearchPathElement(index:longint; var isDOSFlag:longint; path:Pchar):longint;cdecl;external system_nlm name 'GetSearchPathElement';
function InsertSearchPath(scrID:scr_t; searchPathNumber:longint; path:Pchar):longint;cdecl;external system_nlm name 'InsertSearchPath';
function LoadModule(scrID:scr_t; path:Pchar; options:dword):longint;cdecl;external system_nlm name 'LoadModule';
function LoadModule(scrID:scr_t; var path:TLoadModulePath; options:dword):longint;cdecl;external system_nlm name 'LoadModule';
function UnloadModule(scrID:scr_t; commandline:Pchar):longint;cdecl;external system_nlm name 'UnloadModule';
{ memory management interfaces }
function _Alloc(size:size_t; rTag:rtag_t):pointer;cdecl;external system_nlm name 'Alloc';
function Alloc(size:size_t; rTag:rtag_t):pointer;cdecl;external system_nlm name 'Alloc';
function _AllocSleepOK(size:size_t; rTag:rtag_t; slept:Plongint):pointer;cdecl;external system_nlm name 'AllocSleepOK';
function AllocSleepOK(size:size_t; rTag:rtag_t; slept:Plongint):pointer;cdecl;external system_nlm name 'AllocSleepOK';
function _AllocSleepOK(size:size_t; rTag:rtag_t; var slept:longint):pointer;cdecl;external system_nlm name 'AllocSleepOK';
function AllocSleepOK(size:size_t; rTag:rtag_t; var slept:longint):pointer;cdecl;external system_nlm name 'AllocSleepOK';
procedure NWGarbageCollect(NLMHandle:TNLMHandle);cdecl;external system_nlm name 'NWGarbageCollect';
function NWGetAvailableMemory:size_t;cdecl;external system_nlm name 'NWGetAvailableMemory';
function NWGetPageSize:size_t;cdecl;external system_nlm name 'NWGetPageSize';
function NWMemorySizeAddressable(addr:pointer; size:size_t):longint;cdecl;external system_nlm name 'NWMemorySizeAddressable';
function _ReallocSleepOK(addr:pointer; size:size_t; rTag:rtag_t; slept:Plongint):pointer;cdecl;external system_nlm name 'ReallocSleepOK';
function _ReallocSleepOK(addr:pointer; size:size_t; rTag:rtag_t; var slept:longint):pointer;cdecl;external system_nlm name 'ReallocSleepOK';
function ReallocSleepOK(addr:pointer; size:size_t; rTag:rtag_t; slept:Plongint):pointer;cdecl;external system_nlm name 'ReallocSleepOK';
function ReallocSleepOK(addr:pointer; size:size_t; rTag:rtag_t; var slept:longint):pointer;cdecl;external system_nlm name 'ReallocSleepOK';
procedure _Free(addr:pointer);cdecl;external system_nlm name 'Free';

 const
       CMD_CONFIG_INFO     = $00000001;
       CMD_CONVERT_UPPER   = $00000002;   { convert command line to upper case?  }
       CMD_SERVER_RUNNING  = $00000004;   { server need to be running?  }
       CMD_LEGAL_SERVER    = $00000008;   { command legal on regular server?  }
       CMD_HIDDEN_CMD      = $00200000;   { don't display this command  }
       CMD_SUB_CMDS_AVAIL  = $00100000;   { command has sub commands  }
       CMD_NO_CMD_CHAIN    = $00010000;   { disallow chaining of command keyword  }
       CMD_PASS_ON_ERROR   = $00020000;   { ignore error  }
       CMD_ANY_PROCESS     = $80000000;   { can run on any process with keyboard  }
    { keyword flags for RegisterCommand()...  }
       CMD_MSG_NUMBER      = $80000000;   { pointer to keyword is number  }
       CMD_LENGTH_PREC     = $40000000;   { keyword is length-preceeded  }
    { insertion flags for RegisterCommand()...  }
       CMD_INSERT_AT_HEAD  = $00000001;   { insert at head  }
       CMD_INSERT_AT_TAIL  = $00000002;   { insert at tail  }
       CMD_PERM_POSITION   = $80000000;   { leave in place (head or tail)  }
    { valid error returns from command handler...  }
       CMD_CMD_EXECUTED    = $00000000;   { processed, go to next handler  }
       CMD_HALT_CMD_CHAIN  = $70000000;   { processed, but don't go on to next  }
       CMD_BAD_CMD_SYNTAX  = $70000010;   { not processed, syntax error  }
       CMD_NOT_RUNNING     = $70000011;   { won't execute until server up  }
       CMD_LINE_FAULT      = $70000012;   { command line fault--no error display  }
       CMD_BAD_MOD_HANDLE  = $70000021;   { NLM handle is invalid  }
       CMD_BAD_RTAG        = $70000022;   { invalid resource tag  }
       CMD_BAD_KEY         = $70000023;   { keyword is invalid  }
       CMD_RTAG_AND_MOD    = $70000024;   { resource tag doesn't match NLM  }
       CMD_NO_HANDLER      = $70000025;   { handler is missing  }
       CMD_KEY_TOO_LONG    = $70000026;   { keyword is too long  }
       CMD_INVAL_PERM      = $70000027;   { invalid syntax to make permanent  }
       CMD_NO_MEMORY       = $70000028;   { unable to allocate memory  }
       CMD_NOT_REGISTERED  = $70000029;   { unregistered command  }
       CMD_HAS_CHAIN       = $7000002A;   { command has a chain  }
       CMD_CANT_MAKE_HEAD  = $7000002B;   { cannot make command permanent head  }
       CMD_CANT_MAKE_TAIL  = $7000002C;   { cannot make command permanent tail  }
       CMD_PASS_TO_NEXT    = $7000002D;   { not processed, go to next handler  }
       CMD_PRIV_ON_ALT     = $7000002E;   { no privilege on alternate handler  }
       CMD_STOP_CMDS       = $7000002F;   { system no longer parsing commands  }
    { simplified versions of returns from command handler...  }
       CMD_PROCESSED_OK    = CMD_HALT_CMD_CHAIN;
       CMD_CHAIN_NEXT_CMD  = CMD_CMD_EXECUTED;
       CMD_SYNTAX_ERROR    = CMD_BAD_CMD_SYNTAX;
       CMD_NOT_MY_COMMAND  = CMD_PASS_TO_NEXT;
    { command handler function codes...  }
       CMD_HELP_ON_CMD     = $00000000;
       CMD_GET_SUB_CMDS    = $00000001;
       CMD_PROCESS_CMD     = $00000002;
       //CMD_MAX_HELP_CMDS = ProcessCommand;
    { error codes...  }
       ERR_INVALID_MODULE     = $00000001;
       ERR_INVALID_RTAG       = $00000002;
       ERR_INVALID_KEYWORD    = $00000003;
       ERR_MODULE_RTAG_MIX    = $00000004;
       ERR_MISSING_HANDLER    = $00000005;
       ERR_KEYWORD_TOO_LONG   = $00000006;
       ERR_INVALID_REQUEST    = $00000007;
       ERR_OUT_OF_MEMORY      = $00000008;
       ERR_FAILED_TO_REGISTER = $00000009;
       ERR_ALREADY_REGISTERED = $0000000A;
       ERR_CANT_GRANT_TOP     = $0000000B;
       ERR_CANT_GRANT_END     = $0000000C;

function SizeOfAllocBlock(addr:pointer):size_t;cdecl;external system_nlm name 'SizeOfAllocBlock';

type
   CommandHandler_t = function (funcCode:longint; scrID:pointer; command:Pchar; upperCaseCommand:Pchar; callerReference:pointer):longint;cdecl;


function DeRegisterCommand(NLMHandle:TNLMHandle; rTag:rtag_t; keywordFlags:dword; keyword:Pchar):longint;cdecl;external system_nlm name 'DeRegisterCommand';

function RegisterCommand(NLMHandle:TNLMHandle; rTag:rtag_t; keywordFlags:dword; keyword:Pchar; handlerFlags:dword;
           insertionFlags:dword; handler:CommandHandler_t; callerReference:pointer):longint;cdecl;external system_nlm name 'RegisterCommand';
{ legacy command parsing; uses ConsoleCommandSignature...  }

type
   TCommandParserFunc = function (scrID:scr_t; commandline:Pchar):longint;cdecl;

{ allocate with ConsoleCommandSignature  }

   PCommandParserStructure = ^TCommandParserStructure;
   TCommandParserStructure = record
        link   : pointer;
        case longint of
          0 : (parser : TCommandParserFunc; rTag : rtag_t);
          1 : (parseRoutine : TCommandParserFunc; rTag2 : rtag_t);
     end;
   TCommandParser = TCommandParserStructure;
   PCommandParser = PCommandParserStructure;

const
  HANDLEDCOMMAND  = 0;
  NOTMYCOMMAND    = 1;

function ParseCommand(commandLine:Pchar):longint;cdecl;external system_nlm name 'ParseCommand';
function RegisterConsoleCommand(cmdParser:PCommandParser):longint;cdecl;external system_nlm name 'RegisterConsoleCommand';
function RegisterConsoleCommand(var cmdParser:TCommandParser):longint;cdecl;external system_nlm name 'RegisterConsoleCommand';
function UnRegisterConsoleCommand(cmdParser:PCommandParser):longint;cdecl;external libc_nlm name 'UnRegisterConsoleCommand';
function UnRegisterConsoleCommand(var cmdParser:TCommandParser):longint;cdecl;external libc_nlm name 'UnRegisterConsoleCommand';

    const
       SP_TYPE_NUMBER = 0;
    { 'value' points to DWORD (0 or !0) }
       SP_TYPE_BOOLEAN = 1;
       SP_TYPE_TICKS = 2;
       SP_TYPE_BLOCK_SHIFT = 3;
    { [+|-]hh:mm:ss converted to seconds  }
       SP_TYPE_TIME_OFFSET = 4;
    { 'value' points to char buffer  }
       SP_TYPE_STRING = 5;
       SP_TYPE_TRIGGER = 6;
    { settable parameter flags...  }
       SP_STARTUP_ONLY = $01;
       SP_HIDE = $02;
       SP_ADVANCED = $04;
       SP_STARTUP_OR_LATER = $08;
    { can't be done on secured console  }
       SP_NOT_SECURED_CONSOLE = $10;
    { lock console RPC from changing value  }
       SP_RPC_LOCKOUT = $20;
    { settable parameter categories...  }
       SP_COMMUNICATIONS = 0;
       SP_MEMORY = 1;
       SP_FILE_CACHE = 2;
       SP_DIR_CACHE = 3;
       SP_FILE_SYSTEM = 4;
       SP_LOCKS = 5;
       SP_TRANS_TRACKING = 6;
       SP_DISK = 7;
       SP_TIME = 8;
       SP_NCP = 9;
    { recommended most common  }
       SP_MISCELLANEOUS = 10;
       SP_ERRORS = 11;
       SP_DIRECTORY_SERVICES = 12;
       SP_MULTIPROCESSOR = 13;
    { type depends on 'type' field  }
    { for parameter name  }
    { for parameter description  }


type

   Psettableparms_t = ^Tsettableparms;
   Tsettableparms = record
        link        : pointer;
        value       : pointer;
        rTag        : rtag_t;
        name        : Pchar;
        _type       : byte;
        flags       : byte;
        category    : byte;
        reserved    : byte;
        lower_limit : dword;
        upper_limit : dword;
        callback    : procedure (oldValue:dword);cdecl;
        description : Pchar;
        msg_namenum : word;
        msg_descnum : word;
     end;
    Psettableparms = Psettableparms_t;

function RegisterSetableParameter(setparms:Psettableparms_t):longint;cdecl;external system_nlm name 'RegisterSetableParameter';
function RegisterSetableParameter(var setparms:Tsettableparms):longint;cdecl;external system_nlm name 'RegisterSetableParameter';
function DeRegisterSetableParameter(setparms:Psettableparms_t):longint;cdecl;external system_nlm name 'DeRegisterSetableParameter';
function DeRegisterSetableParameter(var setparms:Tsettableparms):longint;cdecl;external system_nlm name 'DeRegisterSetableParameter';

function GetSetableParameterValue(slot:longint; name:Pchar; value:pointer):longint;cdecl;external system_nlm name 'GetSetableParameterValue';
function GetSetableParameterValue(slot:longint; name:Pchar; var value):longint;cdecl;external system_nlm name 'GetSetableParameterValue';

function ScanSetableParameters(scanCategory:longint; scanSequence:Pdword; name:Pchar; _type:Plongint; flags:Pdword;
           category:Plongint; description:pointer; value:pointer; lowerLimit:Plongint; upperLimit:Plongint):longint;cdecl;external system_nlm name 'ScanSetableParameters';
function ScanSetableParameters(scanCategory:longint; var scanSequence:dword; name:Pchar; var _type:longint; var flags:dword;
           var category:longint; var description, value; var lowerLimit,upperLimit:longint):longint;cdecl;external system_nlm name 'ScanSetableParameters';

function SetSetableParameterValue(slot:longint; name:Pchar; newValue:pointer):longint;cdecl;external system_nlm name 'SetSetableParameterValue';
function SetSetableParameterValue(slot:longint; name:Pchar; var newValue):longint;cdecl;external system_nlm name 'SetSetableParameterValue';
{ NLM start-up synchronization...  }
procedure SynchronizeStart;cdecl;external system_nlm name 'SynchronizeStart';
{ message table loading...  }
function LoadLanguageMessageTable(table:PPPchar; count:Plongint; languageID:Plongint):longint;cdecl;external system_nlm name 'LoadLanguageMessageTable';
{ timer interfaces...  }
function GetHighResolutionTimer:dword;cdecl;external system_nlm name 'GetHighResolutionTimer';
function GetSuperHighResolutionTimer:dword;cdecl;external system_nlm name 'GetSuperHighResolutionTimer';

{ spin locks for use in the kernel (not from a protected address space)...  }
type
   Pspinlock_t = ^spinlock_t;
   spinlock_t  = pointer;
   TSpinlock = spinlock_T;
   PSpinlock = Pspinlock_t;

function KernelSpinLockInit(lock:Pspinlock_t):longint;cdecl;external system_nlm name 'KernelSpinLockInit';
procedure KernelSpinLock(lock:Pspinlock_t);cdecl;external system_nlm name 'KernelSpinLock';
function KernelSpinTryLock(lock:Pspinlock_t):longint;cdecl;external system_nlm name 'KernelSpinTryLock';
procedure KernelSpinUnlock(lock:Pspinlock_t);cdecl;external system_nlm name 'KernelSpinUnlock';
function KernelSpinLockDisable(lock:Pspinlock_t):dword;cdecl;external system_nlm name 'KernelSpinLockDisable';
function KernelSpinTryLockDisable(lock:Pspinlock_t; flags:Pdword):longint;cdecl;external system_nlm name 'KernelSpinTryLockDisable';
procedure KernelSpinUnlockRestore(lock:Pspinlock_t; flags:dword);cdecl;external system_nlm name 'KernelSpinUnlockRestore';

function KernelSpinLockInit(var lock:spinlock_t):longint;cdecl;external system_nlm name 'KernelSpinLockInit';
procedure KernelSpinLock(var lock:spinlock_t);cdecl;external system_nlm name 'KernelSpinLock';
function KernelSpinTryLock(var lock:spinlock_t):longint;cdecl;external system_nlm name 'KernelSpinTryLock';
procedure KernelSpinUnlock(var lock:spinlock_t);cdecl;external system_nlm name 'KernelSpinUnlock';
function KernelSpinLockDisable(var lock:spinlock_t):dword;cdecl;external system_nlm name 'KernelSpinLockDisable';
function KernelSpinTryLockDisable(var lock:spinlock_t; flags:Pdword):longint;cdecl;external system_nlm name 'KernelSpinTryLockDisable';
procedure KernelSpinUnlockRestore(var lock:spinlock_t; flags:dword);cdecl;external system_nlm name 'KernelSpinUnlockRestore';


{ nonpreferred locale interfaces...  }
type
   Pcountryinfo_t = ^Tcountryinfo;
   Tcountryinfo = record
        infoID : byte;
        size : word;
        countryID : word;
        codePage : word;
        dateFormat : word;
        currencySymbol : array[0..4] of char;
        thousandSeparator : array[0..1] of char;
        decimalSeparator : array[0..1] of char;
        dateSeparator : array[0..1] of char;
        timeSeparator : array[0..1] of char;
        currencyFormatFlags : char;
        digitsInCurrency : char;
        timeFormat : char;
        UpperCase : procedure ;cdecl;
        dataListSeparator : array[0..1] of char;
        spare : array[0..9] of char;
     end;
   Pcountryinfo = Pcountryinfo_t;

function OSGetCodePage:longint;cdecl;external libc_nlm name 'OSGetCodePage';
procedure OSGetCountryInfo(_para1:Pcountryinfo_t);cdecl;external system_nlm name 'OSGetCountryInfo';
procedure OSGetCountryInfo(var _para1:Tcountryinfo);cdecl;external system_nlm name 'OSGetCountryInfo';

    const
       EXCEPTION_HANDLED     = 0;
       EXCEPTION_NOT_HANDLED = 1;     // chain to next handler
       { registered debugger parser returns:  }
       NEXT_DEBUG_PARSER     = -(2);  // call next debug parser
       NEXT_ALT_DEBUG_PARSER = -(1);  // call next registered debug parser
       COMMAND_HANDLED       = 0;     // call no other parser
       INTERNAL_DEBUGGER     = 1;     // pass to NetWare System Debugger


type
   Pexceptionframe = ^exceptionframe;
   exceptionframe = record
        xfReserved : array[0..6] of dword;
        xfCR3 : Pdword;
        xfEIP : dword;
        xfSystemFlags : dword;
        xfEAX : dword;
        xfECX : dword;
        xfEDX : dword;
        xfEBX : dword;
        xfESP : dword;
        xfEBP : dword;
        xfESI : dword;
        xfEDI : dword;
        xfES : array[0..1] of word;
        xfCS : array[0..1] of word;
        xfSS : array[0..1] of word;
        xfDS : array[0..1] of word;
        xfFS : array[0..1] of word;
        xfGS : array[0..1] of word;
        xfLDT : array[0..1] of word;
        xfSpecial : array[0..1] of word;
        xfNumber : dword;
        xfDescription : Pchar;
        xfFlags : dword;
        xfErrorCode : dword;
        xfPageFaultCR2 : dword;
        xfFPUState : dword;
        xfHistogram : dword;
        xfProcessorID : dword;
     end;
   xframe_t = exceptionframe;
   Pxframe_t = ^xframe_t;

   SoftBPHandler_t = function (number:longint; address:pointer; frame:Pxframe_t):longint;cdecl;
   DebugParser_t = function (scr:scr_t; command:Pchar; frame:Pxframe_t):longint;cdecl;


procedure Abend(message:Pchar);cdecl;external system_nlm name 'Abend';
function AddressOfSoftBreakpoint(number:longint):pointer;cdecl;external system_nlm name 'AddressOfSoftBreakpoint';
function AddSoftBreakpoint(addr:pointer; handler:SoftBPHandler_t):longint;cdecl;external system_nlm name 'AddSoftBreakpoint';
function CSetABreakpoint(number:longint; addr:pointer; _type:byte; length:byte):longint;cdecl;external system_nlm name 'CSetABreakpoint';
procedure EnterDebugger;cdecl;external system_nlm name 'EnterDebugger';
function GetDebuggerActiveCount:longint;cdecl;external system_nlm name 'GetDebuggerActiveCount';
function RegisterDebugCommandParser(parseRoutine:DebugParser_t; rTag:rtag_t):longint;cdecl;external system_nlm name 'RegisterDebugCommandParser';
function RemoveSoftBreakpoint(number:longint):longint;cdecl;external system_nlm name 'RemoveSoftBreakpoint';
function ReserveABreakpointRTag(_para1:rtag_t):longint;cdecl;external system_nlm name 'ReserveABreakpointRTag';
function UnRegisterDebugCommandParser(parseRoutine:DebugParser_t):longint;cdecl;external system_nlm name 'UnRegisterDebugCommandParser';
function UnReserveABreakpoint(_para1:longint):longint;cdecl;external system_nlm name 'UnReserveABreakpoint';


//?? dont know what this is: (exported by system)
//var preferredModule : pointer;cvar;external;

{ Prototypes for libraries and drivers writing their own start-up and shut-
  down code. (DllMain() is also part of this list and defined in windows.h.)
  These are not interfaces, but only prototypes for code furnished by the
  NLM application, library, driver, etc. }

function _NonAppCheckUnload:longint;cdecl;external libc_nlm name '_NonAppCheckUnload';

type TReadRoutine = function (conn:longint; fileHandle:pointer; offset,nbytes,bytesRead:Psize_t; buffer:pointer):longint; cdecl;
function _NonAppStart(NLMHandle:TNLMHandle; errorScreen:pointer; commandLine:Pchar; loadDirPath:Pchar; uninitializedDataLength:size_t;
           NLMFileHandle:pointer; readRoutineP:TReadRoutine; customDataOffset:size_t; customDataSize:size_t; messageCount:longint;
           messages:PPchar):longint;cdecl;external libc_nlm name '_NonAppStart';
procedure _NonAppStop;cdecl;external libc_nlm name '_NonAppStop';

  const
     CTX_ACTUAL_CWD = $01;
  { for set_pathname_format(), namespace appellations...  }
     SHORT_NAMES     = 0;    { 8.3 format  }
     MACINTOSH_NAMES = 1;
     NFS_NAMES       = 2;
     FTAM_NAMES      = 3;
     LONG_NAMES      = 4;    { default long-name format  }
     NT_NAMES        = 5;
  { definitions useful to fshooks.h (attribute-match) and others...  }

     ATTR_NORMAL             = $00000000;  { no read/write restrictions  }
     ATTR_READ_ONLY          = $00000001;  { read-only file }
     ATTR_HIDDEN             = $00000002;  { hidden file }
     ATTR_SYSTEM             = $00000004;  { system file }
     ATTR_EXECUTE            = $00000008;  { execute only file }
     ATTR_VOLUME_ID          = $00000008;  { file system label           }
     ATTR_DIRECTORY          = $00000010;  { subdirectory                }
     ATTR_ARCHIVE            = $00000020;  { archive file                }
     ATTR_SHARE              = $00000080;  { Sharable file               }
     ATTR_NO_SUBALLOC        = $00000800;  { don't sub allocate file     }
     ATTR_TRANS              = $00001000;  { trans'l file, TTS-usable    }
     ATTR_READAUD            = $00004000;  { read audit                  }
     ATTR_WRITAUD            = $00008000;  { write audit                 }
     ATTR_IMMPURG            = $00010000;  { immediate purge             }
     ATTR_NORENAM            = $00020000;  { rename inhibit              }
     ATTR_NODELET            = $00040000;  { delete inhibit              }
     ATTR_NOCOPY             = $00080000;  { copy inhibit                }

     ATTR_FILE_MIGRATED      = $00400000;  { file has been migrated      }
     ATTR_DONT_MIGRATE       = $00800000;  { don't migrate this file     }
     ATTR_IMMEDIATE_COMPRESS = $02000000;  { compress file immediately   }
     ATTR_FILE_COMPRESSED    = $04000000;  { file is compressed          }
     ATTR_DONT_COMPRESS      = $08000000;  { don't compress this file    }
     ATTR_CANT_COMPRESS      = $20000000;  { can't compress this file    }
     ATTR_ATTR_ARCHIVE       = $40000000;  { entry has been modified     }
  { Faster, better when getstat() or fgetstat() used with request bit map...  }
     ST_NONE        = $00000000;
     ST_FLAGS_BIT   = $00000001;
     ST_MODE_BIT    = $00000002;
     ST_GEN_BIT     = $00000004;
     ST_INO_BIT     = $00000008;
     ST_DEV_BIT     = $00000010;
     ST_RDEV_BIT    = $00000020;
     ST_SIZE_BIT    = $00000040;
     ST_BLOCKS_BIT  = $00000080;
     ST_BLKSIZE_BIT = $00000100;
     ST_NLINK_BIT   = $00000200;
     ST_UID_BIT     = $00000400;
     ST_GID_BIT     = $00000800;
     ST_BID_BIT     = $00001000;
     ST_MID_BIT     = $00002000;
     ST_ATIME_BIT   = $00004000;
     ST_MTIME_BIT   = $00008000;
     ST_CTIME_BIT   = $00010000;
     ST_BTIME_BIT   = $00020000;
     ST_STAT_BITS   = $0003FFFF;        { bits for normal stat call }
  { the following are not returned by a normal stat call }
     ST_RIGHTS_BIT  = $00040000;
     ST_NAME_BIT    = $00080000;
     ST_NS_BIT      = $00100000;        { return name in specified namespace }
  { path analysis/parsing 'type' and 'flags' arguments for [de]construct() }
     PATH_UNDEF     = $00000000;

     PATH_DOS       = PATH_UNDEF;       { indicates potential DOS path }
     PATH_UNC       = $00000001;        { double slash found at beginning }
     PATH_UNIX      = $00000002;        { forward slashes only }
     PATH_NETWARE   = $00000004;        { slash and colon followed by slashes }
     PATH_MACINTOSH = $00000008;        { only colons }
     PATH_ROOTED    = $00000010;        { starts with delimiter }
     PATH_VOLROOTED = $00000020;        { volume plus colon appears }
     PATH_EXTENSION = $00000040;        { contains period }
     PATH_HIERARCHY = $00000080;        { at least one subdirectory element }
     PATH_SHORTNAME = $00000100;        { 8.3 names only }
     PATH_LONGNAME  = $00000200;        { at least one element greater than 8.3 }
     PATH_ENDED     = $00000400;        { ends in delimiter }
     PATH_DOSDRIVE  = $00001000;        { single-letter drive, colon and path }
     PATH_MIXEDCASE = $00002000;        { at least one element in mixed case }
     PATH_DOTS      = $00004000;        { path contains dots }
     PATH_SLASH     = $00008000;        { path contains a slash }
     PATH_BACKSLASH = $00010000;        { path contains a backslash }
     PATH_COLON     = $00020000;        { path contains a colon }
     PATH_ILLEGAL   = $80000000;        { illegal character or combination }
     //PATH_MIXED = PATH_SHORT or PATH_LONG;
{     d_cdatetime = d_cdatetim.tv_sec;
     d_adatetime = d_adatetim.tv_sec;
     d_bdatetime = d_bdatetim.tv_sec;
     d_ddatetime = d_ddatetim.tv_sec;}
       FSKEY_NONE = -(1);
       FSKEY_TRAD = 0;
       FSKEY_NSS  = 1;
    { values returned by _fildes_type(): see sys/mode.h  }
    { values returned by _fs_type()...  }
    { 'fildes' is not a file                        }
       FS_NOT_FS  = $00000000;
       FS_LFS     = $00000100;  { file in local, traditional file system }
       FS_REMOTE  = $00000200;  { file in remote file system }
       FS_DOS     = $00000400;  { file in local, DOS file system }
       FS_NSS     = $00000800;  { file in Novell Storage Services }




{ for getcwdpath(), get NKS context for current working directory...  }
{ for set_pathname_format(), namespace appellations...  }
{ definitions useful to fshooks.h (attribute-match) and others...  }
{ Faster, better when getstat() or fgetstat() used with request bit map...  }
{ the following are not returned by a normal stat call                       }
{ path analysis/parsing 'type' and 'flags' arguments for [de]construct()     }
{ d_ddatetim & d_deletedID valid only in scanerasedfiles                     }
{ c.f. these fields in struct dirent...  }
{ untouched by scanerasedfiles()         }

type
   Pnwdirent = ^Tnwdirent;
   Tnwdirent = record
        d_userspec : dword;
        d_flags : dword;                         // flags for this entry
        d_type : mode_t;                         // type of entry
        d_mode : mode_t;                         // emulated file mode
        d_ino : ino_t;                           // directory entry number of d_name
        d_size : off64_t;                        // size of file
        d_spare : array[0..38] of dword;
        d_cdatetim : timespec_t;                 // creation date and time
        d_adatetim : timespec_t;                 // last access date--files only
        d_bdatetim : timespec_t;                 // last archive date and time
        d_ddatetim : timespec_t;                 // deleted date/time
        d_uid : uid_t;                           // owner id (object id)
        d_archivedID : uid_t;                    // object ID that last archived file
        d_updatedID : uid_t;                     // object ID that last updated file
        d_deletedID : uid_t;                     // deleted ID
        d_pad1 : byte;
        d_pad2 : byte;
        d_pad3 : byte;
        d_namelen : byte;                        // lenght of following name:
        d_name : array[0..(255 + 1)-1] of char;  // only portable field in this structure
     end;
   TNWDIR = Tnwdirent;
   PNWDIR = ^TNWDIR;
{ sizeof(struct nwdirent)==0x200 (512.)  }
{ extensions of unistd.h path parsing functions...  }


function deconstruct(path:Pchar; server:Pchar; volume:Pchar; directory:Pchar; name:Pchar;
           extension:Pchar; elements:Plongint; flags:Plongint):longint;cdecl;external libc_nlm name 'deconstruct';
function construct(path:Pchar; server:Pchar; volume:Pchar; directory:Pchar; name:Pchar;
           extension:Pchar; flags:longint):longint;cdecl;external libc_nlm name 'construct';
{ extensions of client.h identity functions...  }
function get_identity(pathctx:NXPathCtx_t; identity:Plongint):longint;cdecl;external libc_nlm name 'get_identity';
{ extensions of unistd.h current working directory I/O functions...  }
function getcwdpath(buf:Pchar; pathCtx:PNXPathCtx_t; flags:dword):Pchar;cdecl;external libc_nlm name 'getcwdpath';

function chdir2(path:Pchar):longint;cdecl;external libc_nlm name 'chdir2';
function setcwd(pathCtx:NXPathCtx_t):longint;cdecl;external libc_nlm name 'setcwd';
function setcwd2(pathCtx:NXPathCtx_t):longint;cdecl;external libc_nlm name 'setcwd2';
{ extensions of unistd.h file I/O functions...  }
function Fpeof(fildes:longint):longint;cdecl;external libc_nlm name 'eof';
function tell(fildes:longint):off_t;cdecl;external libc_nlm name 'tell';
function Fptell(fildes:longint):off_t;cdecl;external libc_nlm name 'tell';
{ extensions of sys/stat.h functions...  }
function fgetstat(fildes:longint; buf:Pstat; requestmap:dword):longint;cdecl;external libc_nlm name 'fgetstat';

function getstat(ctx:NXPathCtx_t; path:Pchar; buf:Pstat; requestmap:dword):longint;cdecl;external libc_nlm name 'getstat';
function fgetstat_with_namespace(fildes:longint; buf:Pstat; requestmap:dword; _namespace:longint):longint;cdecl;external libc_nlm name 'fgetstat_with_namespace';

function getstat_with_namespace(ctx:NXPathCtx_t; path:Pchar; buf:Pstat; requestmap:dword; _namespace:longint):longint;cdecl;external libc_nlm name 'getstat_with_namespace';
{ pathname format (namespace) state...  }
function set_pathname_format(newformat:longint; oldformat:Plongint):longint;cdecl;external libc_nlm name 'set_pathname_format';
{ for use with Novell Clustering...  }

function isclusteredvirtualserver(servername:Pchar):longint;cdecl;external libc_nlm name 'isclusteredvirtualserver';
{ equivalent to CLib's FileServerFileCopy()...  }
function fscopy(fildes1:longint; fildes2:longint; offset1:off64_t; offset2:off64_t; length:size_t;
           nbytes:Psize_t):longint;cdecl;external libc_nlm name 'fscopy';
{ equivalent to similarly named calls in CLib...  }

function purgeerasedfile(path:Pchar; sequence:longint):longint;cdecl;external libc_nlm name 'purgeerasedfile';


function salvageerasedfile(pathName:Pchar; sequence:longint; newFileName:Pchar):longint;cdecl;external libc_nlm name 'salvageerasedfile';

function scanerasedfiles(path:Pchar; nextEntryNumber:Plongint; deletedFileInfo:PNWDIR):longint;cdecl;external libc_nlm name 'scanerasedfiles';
function _fs_type(fildes:longint):longint;cdecl;external libc_nlm name '_fs_type';
function _fildes_type(fildes:longint):longint;cdecl;external libc_nlm name '_fildes_type';
function _fildes_from_nsskey(key:Tuint64; oflag:longint):longint;cdecl;external libc_nlm name '_fildes_from_nsskey';
function _key_from_fildes(fildes:longint; _type:Plongint; err:Plongint):Tuint64;cdecl;external libc_nlm name '_key_from_fildes';
{ fast type of a file descriptor--st_mode in fstat()...  }
{ equates to move between NKS file handles and POSIX descriptors...  }
{ derivation of POSIX descriptor from NSS open file key...  }
{ back-derivation of file system key...  }
{ types returned in back-derivation...  }
{ values returned by _fildes_type(): see sys/mode.h  }
{ values returned by _fs_type()...  }
{ attributes for NXDirAttrWin_t 'dwFileAttributes' field...  }


{ turn on 1-byte packing...  }

  const
     FILE_ATTRIBUTE_READONLY            = $00000001;
     FILE_ATTRIBUTE_HIDDEN              = $00000002;
     FILE_ATTRIBUTE_SYSTEM              = $00000004;
     FILE_ATTRIBUTE_DIRECTORY           = $00000010;
     FILE_ATTRIBUTE_ARCHIVE             = $00000020;
     FILE_ATTRIBUTE_ENCRYPTED           = $00000040;
     FILE_ATTRIBUTE_NORMAL              = $00000080;
     FILE_ATTRIBUTE_TEMPORARY           = $00000100;
     FILE_ATTRIBUTE_SPARSE_FILE         = $00000200;
     FILE_ATTRIBUTE_REPARSE_POINT       = $00000400;
     FILE_ATTRIBUTE_COMPRESSED          = $00000800;
     FILE_ATTRIBUTE_OFFLINE             = $00001000;
     FILE_ATTRIBUTE_NOT_CONTENT_INDEXED = $00002000;


type
   PNXDirAttrDos_t = ^NXDirAttrDos_t;
   NXDirAttrDos_t = record                    { DOS/FAT32 (NX_PNF_DOS) attributes    }
        xdeHeader : NXDeHeader_t;
        xdeFid : NXFid_t;
        xdeEffectiveRights : dword;
                                              { FAT32 File System Spec., p. 22       }
        xdeAttr : dword;
        xdeCrtDate : word;
        xdeCrtTime : word;
        xdeCrtTimeTenth : word;
        xdeLastAccDate : word;
        xdeWrtDate : word;
        xdeWrtTime : word;
        xdeFileSize : dword;
        Reserved1 : dword;
        Reserved2 : dword;
     end;
{ DOS/FAT32 (NX_PNF_DOS) information.. }
{ ...including name                    }
{ maximum length is 255 characters     }

   PNXDirEnumDos_t = ^NXDirEnumDos_t;
   NXDirEnumDos_t = record
        deDosDirAttr : NXDirAttrDos_t;
        deDirMark : NXDirMark_t;
        deShortName : array[0..15] of char;
        deName : pointer;
     end;
{ Win32 (NX_PNF_WIN) attributes        }
{ see WIN32_FIND_DATA...               }
{ FILE_ATTRIBUTE_NORMAL, etc.          }
{ date and time of creation or -1      }
{ date and time of last access         }
{ date and time of last modification   }
{ date and time of last attr change    }
{ see BY_HANDLE_FILE_INFORMATION...    }
{ always 1 for FAT                     }
{ unique identifier                    }
{ CIFS Specification 0.9, 4.2.16.8...  }
{ file system block size               }
{ (currently) Ramesh-invented fields... }
{ not in Win32 docs, but exists        }
{ Boolean flag fields for CIFS         }
{ total size of extended attributes    }
{ size of compressed file           }
{ format of of compression           }

   PNXDirAttrWin_t = ^NXDirAttrWin_t;
   NXDirAttrWin_t = record
        xdeHeader : NXDeHeader_t;
        xdeFid : NXFid_t;
        xdeEffectiveRights : dword;
        xdeFileAttributes : dword;
        xdeCreateTime : Tuint64;
        xdeLastAccessTime : Tuint64;
        xdeLastWriteTime : Tuint64;
        xdeLastChangeTime : Tuint64;
        xdeFileSize : Tuint64;
        Reserved0 : dword;
        Reserved1 : dword;
        Reserved2 : dword;
        Reserved3 : dword;
        xdeNumberOfLinks : dword;
        xdeVolumeSerialNumber : dword;
        Reserved4 : dword;
        Reserved5 : dword;
        xdeFileIndex : Tuint64;
        Reserved6 : Tuint64;
        xdeAllocationSize : dword;
        xdeAccessFlags : dword;
        xdeMiscFlags : dword;
        Reserved7 : dword;
        xdeEaSize : Tuint64;
        xdeCompressedFileSize : Tuint64;
        xdeCompressionFormat : word;
        Reserved8 : word;
        Reserved9 : dword;
        Reserved10 : Tuint64;
     end;
{ Win32 (NX_PNF_WIN) information...    }
{ ...including name                    }
{ see WIN32_FIND_DATA...               }
{ maximum length is 255 characters     }

   PNXDirEnumWin_t = ^NXDirEnumWin_t;
   NXDirEnumWin_t = record
        deWinDirAttr : NXDirAttrWin_t;
        deDirMark : NXDirMark_t;
        deAlternativeFileName : array[0..15] of char;
        deName : pointer;
     end;

// nks/errno.h

  const
     NX_EOF = -(1);
     NX_ENOENT = 1;        { no such file or directory                      }
     NX_E2BIG = 2;         { argument list too big                          }
     NX_ENOEXEC = 3;       { exec format error                              }
     NX_EBADF = 4;         { bad file number (descriptor or handle)         }
     NX_ENOMEM = 5;        { not enough memory                              }
     NX_EACCES = 6;        { permission denied                              }
     NX_EEXIST = 7;        { file exists                                    }
     NX_EXDEV = 8;         { cross-device link                              }
     NX_EINVAL = 9;        { invalid argument                               }
     NX_ENFILE = 10;       { file table overflow                            }
     NX_EMFILE = 11;       { too many open files                            }
     NX_ENOSPC = 12;       { no space left on device                        }
     NX_ERANGE = 14;       { result too large                               }
     NX_EDEADLK = 15;      { resource deadlock would occur                  }
     NX_EINUSE = 16;       { resource(s) in use                             }
     NX_ESERVER = 17;      { server error (memory out, I/O error, etc.)     }
     NX_ENOSERVR = 18;     { no server (queue server, file server, etc.)    }
     NX_EWRNGKND = 19;     { wrong kind--an operation is being...           }
                           {  ...attempted on the wrong kind of object      }
     NX_ETRNREST = 20;     { transaction restarted                          }
     NX_ERESOURCE = 21;    { resources unavailable (maybe permanently)      }
     NX_EBADHNDL = 22;     { bad non-file handle (screen, semaphore, etc.)  }
     NX_EAGAIN = 24;       { resource temporarily unavailable               }
     NX_EIO = 28;          { physical I/O error                             }
     NX_EPIPE = 32;        { broken pipe                                    }
     NX_EALREADY = 37;     { operation already in progress                  }
     NX_ETIMEDOUT = 60;    { connection timed out                           }
     NX_EBUSY = 62;        { resource busy                                  }
     NX_EINTR = 63;        { interrupted function call                      }
     NX_EISDIR = 64;       { is a directory (not a file)                    }
     NX_ENAMETOOLONG = 65; { filename too long                              }
     NX_ENOSYS = 66;       { function not implemented                       }
     NX_ENOTDIR = 67;      { not a directory                                }
     NX_ENOTEMPTY = 68;    { directory is not empty                         }
     NX_EPERM = 69;        { operation not permitted                        }
     NX_ECHILD = 70;       { no child process                               }
     NX_EFBIG = 71;        { file too large                                 }
     NX_EMLINK = 72;       { too many links                                 }
     NX_ENOLCK = 74;       { no locks available                             }
     NX_ESRCH = 77;        { no such object                                 }
     NX_ENOTSUP = 79;      { this optional functionality not supported      }
     NX_EBADTYPE = 80;     { bad type for operation                         }
     NX_EOVERFLOW = 81;    { operation would overflow                       }
     NX_EHOSTDOWN = 82;    { host is down                                   }
     NX_EHOSTUNREACH = 83; { no route to host                               }
     NX_EPROCLIM = 84;     { too many processes                             }
     NX_EUNKNOWN = 99;     { unknown error occurring                        }
     NX_ENLMDATA = 100;    { anomaly in NLM data structure                  }
     NX_EILSEQ = 101;      { illegal character sequence in multibyte        }
     NX_EINCONSIS = 102;   { internal library inconsistency                 }
     NX_EDOSTEXTEOL = 103; { DOS-text file inconsistency--no newline...     }
                           { ...after carriage return                       }
     NX_ENONEXTANT = 104;  { object doesn't exist                           }
     NX_ENOCONTEXT = 105;  { the caller is not an NKS thread                }
     NX_ENAMESPACE = 106;  { invalid namespace or namespace operation       }
     NX_EBADCONN = 107;    { invalid connection                             }
     NX_EEXHAUSTED = 108;  { end of search                                  }
     NX_EFILESYS = 111;    { generic file system error                      }
     NX_ESUFFICIENT = 112; { insufficient space for any operation result    }
     NX_EPARTONLY = 113;   { partial result only  for lack of space         }
     NX_EBADIDENT = 114;   { invalid user or other identity                 }
     { aliases...  }
     NX_ENOSUPPORT = NX_ENOTSUP;
     //NX_ENORESOURCE = NX_BADIDENT;


procedure NXGetNKSVersion(major:Plongint; minor:Plongint; revision:Plongint; platformName:pointer; maxNameLength:size_t);cdecl;external libc_nlm name 'NXGetNKSVersion';
procedure NXGetNKSVersion(var major, minor, revision:longint; platformName:pchar; maxNameLength:size_t);cdecl;external libc_nlm name 'NXGetNKSVersion';
function NXStrError(errornumber:longint):Pchar;cdecl;external libc_nlm name 'NXStrError';


// nks/mac.h
{ turn on 1-byte packing...  }

  { values for 'fdFlags' (Inside Macintosh IV-105)...  }

const
  fOnDesk    = 1;
  fHasBundle = 8192;
  fInvisible = 16384;
  { values for 'fdLocation' (Inside Macintosh IV-105)...  }
  fDisk      = 0;
  fDesktop   = -(2);
  fTrash     = -(3);
  { 'ioFlAttrib' values...  }
  flLocked   = $01;
  flResOpen  = $04;
  flDatOpen  = $08;
  flIsDir    = $10;
  flBothOpen = $80;

type
   POSType = ^OSType;
   OSType = char;
{ (Inside Macintosh II-373)  }
{ (Inside Macintosh I-139)  }

   PPoint = ^Point;
   Point = record
        v : word;
        h : word;
     end;
{ (Inside Macintosh I-141)  }

   PRect = ^Rect;
   Rect = record
       case longint of
          0 : ( corner : record
               topLeft : Point;
               botRight : Point;
            end );
          1 : ( point : record
               top : word;
               left : word;
               bottom : word;
               right : word;
            end );
       end;
{ (Inside Macintosh IV-104)  }
{ window  }

   PFInfo = ^FInfo;
   FInfo = record
        fdType : OSType;
        fdCreator : OSType;
        fdFlags : word;
        fdLocation : Point;
        fdFldr : word;
     end;
{ (Inside Macintosh IV-105)  }
{ Finder (Desktop) comment ID  }
{ home directory ID  }

   PFXInfo = ^FXInfo;
   FXInfo = record
        fdIconID : word;
        fdUnused : array[0..3] of word;
        fdComment : word;
        fdPutAway : dword;
     end;
{ (Inside Macintosh IV-105)  }
{ folder's rectangle  }

   PDInfo = ^DInfo;
   DInfo = record
        frRect : Rect;
        frFlags : word;
        frLocation : Point;
        frView : word;
     end;
{ scroll position                            }
{ directory ID chain of open folders         }
{ directory ID                               }

   PDXInfo = ^DXInfo;
   DXInfo = record
        frScroll : Point;
        frOpenChain : dword;
        frUnused : word;
        frComment : word;
        frPutAway : dword;
     end;
{ information used by the Finder             }
{ directory ID or file number                }
{ first allocation block of data fork        }
{ logical end-of-file of data fork           }
{ physical end-of-file of data fork          }
{ first allocation block of resource fork    }
{ logical end-of-file of resource fork       }
{ physical end-of-file of resource fork      }
{ date and time of creation                  }
{ date and time of last modification         }
{ date and time of last back-up              }
{ additional information used by the Finder  }
{ file's parent directory ID                 }
{ file's clump size                          }

   PhFileInfo = ^hFileInfo;
   hFileInfo = record
        ioFlFndrInfo : FInfo;
        ioDirID : dword;
        ioFlStBlk : word;
        ioFlLgLen : dword;
        ioFlPyLen : dword;
        ioFlRStBlk : word;
        ioFlRLgLen : dword;
        ioFlRPyLen : dword;
        ioFlCrDat : dword;
        ioFlMdDat : dword;
        ioFlBkDat : dword;
        ioFlXFndrInfo : FXInfo;
        ioFlParID : dword;
        ioFlClpSiz : dword;
     end;
{ information used by the Finder             }
{ number of files in directory               }
{ date and time of creation                  }
{ date and time of last modification         }
{ date and time of last backup               }
{ additional information used by the Finder  }
{ directory's parent directory ID            }

   PdirInfo = ^dirInfo;
   dirInfo = record
        ioDrUsrWds : DInfo;
        ioDrDirID : dword;
        ioDrNmFls : word;
        filler3 : array[0..8] of word;
        ioDrCrDat : dword;
        ioDrMdDat : dword;
        ioDrBkDat : dword;
        ioDrFndrInfo : DXInfo;
        ioDrParID : dword;
     end;

   PCInfoPBRec = ^CInfoPBRec;
   CInfoPBRec = record
       case longint of
          0 : ( _file : hFileInfo );
          1 : ( dir : dirInfo );
       end;
{ Macintosh (NX_PNF_MAC) attributes          }
{ (Inside Macintosh IV-125)  }
{ path reference number  }

   PNXDirAttrMac_t = ^NXDirAttrMac_t;
   NXDirAttrMac_t = record
        xdeHeader : NXDeHeader_t;
        xdeFid : NXFid_t;
        xdeEffectRights : dword;
        xdeIoFRefNum : word;
        xdeIofVersNum : int8_t;
        filler1 : int8_t;
        xdeIoFDirIndex : word;
        xdeIoFlAttrib : int8_t;
        filler2 : int8_t;
        filler3 : dword;
        xdeInfo : CInfoPBRec;
     end;
{ Macintosh (NX_PNF_MAC) information...      }
{ ...including                               }
{ maximum length is 255 characters     }

   PNXDirEnumMac_t = ^NXDirEnumMac_t;
   NXDirEnumMac_t = record
        deMacDirAttr : NXDirAttrMac_t;
        deDirMark : NXDirMark_t;
        deName : pointer;
     end;

(** unsupported pragma#pragma pack()*)

// nks/memory.h

  { deprecated values for memory control flags...  }

  const
     NX_PAGE_UNLOCK       = $00000000;
     NX_PAGE_LOCK         = $00000001;
     NX_PAGE_RESERVE      = $00000020;
     NX_PAGE_COMMIT       = $00000040;
     NX_PAGE_PHYSICAL     = $00000080;
     NX_PAGE_RESERVE_ONLY = $00000100;
  { final values for memory control flags...  }
     NX_MEM_UNLOCK        = $00000000;
     NX_MEM_LOCK          = $00000001;
     NX_MEM_DECOMMIT      = $00000020;
     NX_MEM_COMMIT        = $00000040;
     NX_MEM_RESERVE       = $00000100;

function NXMemAlloc(size:size_t; alignment:size_t):pointer;cdecl;external libc_nlm name 'NXMemAlloc';
function NXMemCtl(start:pointer; size:size_t; flags:dword):longint;cdecl;external libc_nlm name 'NXMemCtl';
procedure NXMemFree(memory:pointer);cdecl;external libc_nlm name 'NXMemFree';
function NXMemRealloc(old:pointer; newSize:size_t; alignment:size_t):pointer;cdecl;external libc_nlm name 'NXMemRealloc';
function NXPageAlloc(pageCount:size_t; flags:dword):pointer;cdecl;external libc_nlm name 'NXPageAlloc';
procedure NXPageFree(memory:pointer);cdecl;external libc_nlm name 'NXPageFree';


{ nks/netware.h =============================================================}
{ turn on 1-byte packing...  }

type

   PnxTrustees_t = ^nxTrustees_t;
   nxTrustees_t = record
        trObjectID,
        trRights   : dword;
     end;

(** unsupported pragma#pragma pack()*)
{----------------------------------------------------------------------------
 The following are unofficial NKS interfaces and can only be used on NetWare.
 ----------------------------------------------------------------------------}

procedure nxCancelCheck;cdecl;external libc_nlm name 'nxCancelCheck';
procedure nxCancelDisable;cdecl;external libc_nlm name 'nxCancelDisable';
procedure nxCancelEnable;cdecl;external libc_nlm name 'nxCancelEnable';
function nxContextFlushName(context:PNXContext_t):longint;cdecl;external libc_nlm name 'nxContextFlushName';

function nxExportInterface(funcAddr:pointer; funcName:Pchar):longint;cdecl;external libc_nlm name 'nxExportInterface';

function nxExportInterfaceWrapped(funcAddr:pointer; stackWords:longint; funcName:Pchar; reference:Ppointer):longint;cdecl;external libc_nlm name 'nxExportInterfaceWrapped';
function nxGetEnviron:PPchar;cdecl;external libc_nlm name 'nxGetEnviron';
function nxIsLoadedProtected:NXBool_t;cdecl;external libc_nlm name 'nxIsLoadedProtected';
function nxIsProtectedAddress(_para1:pointer):NXBool_t;cdecl;external libc_nlm name 'nxIsProtectedAddress';
function nxMemGetSize(block:pointer):size_t;cdecl;external libc_nlm name 'nxMemGetSize';
procedure nxUnexportInterfaceWrapped(reference:pointer);cdecl;external libc_nlm name 'nxUnexportInterfaceWrapped';
{ NetWare trustees...  }

function nxAddTrustee(pathCtx:NXPathCtx_t; pathname:Pchar; objectID:dword; rights:dword):longint;cdecl;external libc_nlm name 'nxAddTrustee';
function nxDeleteTrustee(pathCtx:NXPathCtx_t; pathname:Pchar; objectID:dword):longint;cdecl;external libc_nlm name 'nxDeleteTrustee';
function nxScanTrustees(pathCtx:NXPathCtx_t; pathname:Pchar; sequence:dword; count:Pdword; trusteeVector:PnxTrustees_t;
           nextSequence:Pdword):longint;cdecl;external libc_nlm name 'nxScanTrustees';
{ wrap/unwrap sobriquets...  }
// nks/synch.h

{ turn on 1-byte packing...  }

type
   PNXHierarchy_t = ^NXHierarchy_t;
   NXHierarchy_t = longint;

   PNXMutex_t = ^NXMutex_t;
   NXMutex_t = record
        reserved1 : Tuint64;
        reserved2 : array[0..9] of pointer;
     end;

   PNXRwLock_t = ^NXRwLock_t;
   NXRwLock_t = record
        reserved1 : Tuint64;
        reserved2 : array[0..9] of pointer;
     end;

   PNXSema_t = ^NXSema_t;
   NXSema_t = record
        reserved1 : Tuint64;
        reserved2 : array[0..4] of pointer;
     end;

   PNXCond_t = ^NXCond_t;
   NXCond_t = record
        reserved1 : Tuint64;
        reserved2 : array[0..4] of pointer;
     end;

   PNXLockInfo_t = ^NXLockInfo_t;
   NXLockInfo_t = record
        liName : array[0..(31 + 1)-1] of char;
        liFlags : dword;
        liPad : array[0..1] of dword;
     end;

(** unsupported pragma#pragma pack()*)
{ Mutexes...  }


function NXMutexAlloc(flags:dword; hierarchy:NXHierarchy_t; info:PNXLockInfo_t):PNXMutex_t;cdecl;external libc_nlm name 'NXMutexAlloc';
procedure NXMutexDeinit(mutex:PNXMutex_t);cdecl;external libc_nlm name 'NXMutexDeinit';
function NXMutexDepth(mutex:PNXMutex_t):longint;cdecl;external libc_nlm name 'NXMutexDepth';
procedure NXMutexFree(mutex:PNXMutex_t);cdecl;external libc_nlm name 'NXMutexFree';
function NXMutexInit(mutex:PNXMutex_t; flags:dword; hierarchy:NXHierarchy_t; info:PNXLockInfo_t):longint;cdecl;external libc_nlm name 'NXMutexInit';
function NXMutexIsOwned(mutex:PNXMutex_t):NXBool_t;cdecl;external libc_nlm name 'NXMutexIsOwned';
function NXMutexTestFlag(mutex:PNXMutex_t; flag:dword):NXBool_t;cdecl;external libc_nlm name 'NXMutexTestFlag';
function NXLock(mutex:PNXMutex_t):longint;cdecl;external libc_nlm name 'NXLock';
function NXTryLock(mutex:PNXMutex_t):NXBool_t;cdecl;external libc_nlm name 'NXTryLock';
function NXUnlock(mutex:PNXMutex_t):longint;cdecl;external libc_nlm name 'NXUnlock';

{ Reader-writer locks...  }
function NXRwLockAlloc(hierarchy:NXHierarchy_t; info:PNXLockInfo_t):PNXRwLock_t;cdecl;external libc_nlm name 'NXRwLockAlloc';
procedure NXRwLockDeinit(lock:PNXRwLock_t);cdecl;external libc_nlm name 'NXRwLockDeinit';
procedure NXRwLockFree(lock:PNXRwLock_t);cdecl;external libc_nlm name 'NXRwLockFree';
function NXRwLockInit(lock:PNXRwLock_t; hierarchy:NXHierarchy_t; info:PNXLockInfo_t):longint;cdecl;external libc_nlm name 'NXRwLockInit';
function NXRwLockIsOwned(lock:PNXRwLock_t; mode:dword):NXBool_t;cdecl;external libc_nlm name 'NXRwLockIsOwned';
procedure NXRdLock(lock:PNXRwLock_t);cdecl;external libc_nlm name 'NXRdLock';
procedure NXWrLock(lock:PNXRwLock_t);cdecl;external libc_nlm name 'NXWrLock';
function NXTryRdLock(lock:PNXRwLock_t):NXBool_t;cdecl;external libc_nlm name 'NXTryRdLock';
function NXTryWrLock(lock:PNXRwLock_t):NXBool_t;cdecl;external libc_nlm name 'NXTryWrLock';
procedure NXRwUnlock(lock:PNXRwLock_t);cdecl;external libc_nlm name 'NXRwUnlock';
function NXRwLockUpgrade(lock:PNXRwLock_t):longint;cdecl;external libc_nlm name 'NXRwLockUpgrade';
function NXRwLockDowngrade(lock:PNXRwLock_t):longint;cdecl;external libc_nlm name 'NXRwLockDowngrade';
{ Semaphores...  }
function NXSemaAlloc(count:dword; arg:pointer):PNXSema_t;cdecl;external libc_nlm name 'NXSemaAlloc';
procedure NXSemaDeinit(sema:PNXSema_t);cdecl;external libc_nlm name 'NXSemaDeinit';
procedure NXSemaFree(sema:PNXSema_t);cdecl;external libc_nlm name 'NXSemaFree';
function NXSemaInit(sema:PNXSema_t; count:dword; arg:pointer):longint;cdecl;external libc_nlm name 'NXSemaInit';
procedure NXSemaPost(sema:PNXSema_t);cdecl;external libc_nlm name 'NXSemaPost';
function NXSemaTryWait(sema:PNXSema_t):NXBool_t;cdecl;external libc_nlm name 'NXSemaTryWait';
procedure NXSemaWait(sema:PNXSema_t);cdecl;external libc_nlm name 'NXSemaWait';
{ Condition variables...  }
function NXCondAlloc(arg:pointer):PNXCond_t;cdecl;external libc_nlm name 'NXCondAlloc';
procedure NXCondBroadcast(cond:PNXCond_t);cdecl;external libc_nlm name 'NXCondBroadcast';
procedure NXCondDeinit(cond:PNXCond_t);cdecl;external libc_nlm name 'NXCondDeinit';
procedure NXCondFree(cond:PNXCond_t);cdecl;external libc_nlm name 'NXCondFree';
function NXCondInit(cond:PNXCond_t; arg:pointer):longint;cdecl;external libc_nlm name 'NXCondInit';
procedure NXCondSignal(cond:PNXCond_t);cdecl;external libc_nlm name 'NXCondSignal';
function NXCondWait(cond:PNXCond_t; mutex:PNXMutex_t):longint;cdecl;external libc_nlm name 'NXCondWait';
function NXCondTimedWait(cond:PNXCond_t; mutex:PNXMutex_t; interval:dword):longint;cdecl;external libc_nlm name 'NXCondTimedWait';




//  assert.h

procedure _assert(_para1,_para2, _para3:Pchar; ActionCode:longint);cdecl;external libc_nlm name '_assert';
procedure FpAssert(_para1,_para2, _para3:Pchar; ActionCode:longint);cdecl;external libc_nlm name '_assert';

type
   Taction_code =  Longint;
Const                         // modifications to behavior of assert()
  __IGNORE = -(1);            // assert() prints but returns -1
  __NOERR = 0;                // (value returned for no assertion)
  __ABORT = 1;                // assert() aborts (normal, default action)
  __DEBUGGER = 2;             // assert() prints and drops into the debugger

function assert_action(_para1:Taction_code):longint;cdecl;external libc_nlm name 'assert_action';
function _assert_expr(_para1:longint; _para2,_para3,_para4:Pchar; _para5:longint):longint;cdecl;external libc_nlm name '_assert_expr';

// nks/unix.h

{ turn on 1-byte packing...  }

{ UNIX (NX_PNF_UNIX) attributes           }
{ file mode                               }
{ number of links                         }
{ last access time (files only)           }
{ last modify time                        }
{ last archive time                       }
{ last attribute change time (or 0)       }
{ generation; bumped when file modified   }
{ entry serial number                     }
{ file system (device) containing entry   }
{ ID of raw device containing this entry  }
{ user ID of the owner of this entry      }
{ group ID of the group of this entry     }
{ length of file in bytes                 }
{ number of 512-byte blocks allocated     }
{ preferred I/O block size                }
type

   PNXDirAttrUnix_t = ^NXDirAttrUnix_t;
   NXDirAttrUnix_t = record
        xdeHeader : NXDeHeader_t;
        xdeFid : NXFid_t;
        xdeEffectiveRights : dword;
        xde_mode : dword;
        xde_nlink : dword;
        spare1 : dword;
        xde_atime : time_t;
        xde_mtime : time_t;
        xde_btime : time_t;
        xde_ctime : time_t;
        xde_change : Tuint64;
        xde_ino : Tuint64;
        xde_dev : array[0..1] of Tuint64;
        xde_rdev : array[0..1] of Tuint64;
        xde_uid : Tuint64;
        xde_gid : Tuint64;
        xde_size : Tuint64;
        xde_blocks : Tuint64;
        xde_blksize : dword;
        spare2 : dword;
        spare3 : Tuint64;
        spare4 : Tuint64;
        spare5 : Tuint64;
        spare6 : Tuint64;
        spare7 : Tuint64;
        spare8 : Tuint64;
     end;
{ UNIX (NX_PNF_UNIX) information...       }
{ ...including name                       }
{ maximum length is 255 characters        }

   PNXDirEnumUnix_t = ^NXDirEnumUnix_t;
   NXDirEnumUnix_t = record
        deUnixDirAttr : NXDirAttrUnix_t;
        deDirMark : NXDirMark_t;
        deName : pointer;
     end;

(** unsupported pragma#pragma pack()*)
//  nks/vm.h

{ value for 'wait_for' in NXVmJoin()...  }
{ values for 'flags' in NXVmSpawn()...  }
{ returned in newVm if NXVmSpawn() is passed NX_VM_DETACHED or if it fails  }

{ turn on 1-byte packing...  }

type
   PNXStrType_t = ^NXStrType_t;
   NXStrType_t  =  Longint;
Const
   NX_STR_ASCII   = $FFFFFFFF;
   NX_STR_UTF8    = $00000000;
   NX_STR_UNICODE = $00000001;

{ NX_OBJ_FILE, NX_OBJ_CONSOLE, NX_OBJ_FIFO, etc. }
{ set to 0                                       }
{ ancestor of 'ssPath'                           }
{ relative to 'ssPathCtx'                        }
type
   PNXNameSpec_t = ^NXNameSpec_t;
   NXNameSpec_t = record
        ssType : NXObjType_t;
        ssReserved : longint;
        ssPathCtx : NXPathCtx_t;
        ssPath : pointer;
     end;
   NXStreamSpec_t = NXNameSpec_t;
   PNXStreamSpec_t = ^NXStreamSpec_t;
{ unused; set to 0                               }
{ count of arguments in 'esArgv'                 }
{ command-line arguments to spawned VM           }
{ starting environment of spawned VM             }
{ wiring of standard input for spawned VM        }
{ wiring of standard output for spawned VM       }
{ wiring of standard error for spawned VM        }

   PNXExecEnvSpec_t = ^NXExecEnvSpec_t;
   NXExecEnvSpec_t = record
        esFlags : longint;
        esArgc : longint;
        esArgv : ^pointer;
        esEnv : ^pointer;
        esStdin : NXStreamSpec_t;
        esStdout : NXStreamSpec_t;
        esStderr : NXStreamSpec_t;
     end;
{ maxmimum number of threads in worker pool      }
{ implementation-reserved                        }

   PNXVmWorkerThreadConfig_t = ^NXVmWorkerThreadConfig_t;
   NXVmWorkerThreadConfig_t = record
        wtcThreads : size_t;
        reserved : array[0..4] of longint;
     end;

(** unsupported pragma#pragma pack()*)
{ Virtual machine management...  }

function  NXVmDestroy(id:NXVmId_t):longint;cdecl;external libc_nlm name 'NXVmDestroy';
procedure NXVmExit(status:longint);cdecl;external libc_nlm name 'NXVmExit';
function  NXVmGetId:NXVmId_t;cdecl;external libc_nlm name 'NXVmGetId';
function  NXVmGetWorkerThreadConfig(reserved:pointer; config:PNXVmWorkerThreadConfig_t):longint;cdecl;external libc_nlm name 'NXVmGetWorkerThreadConfig';
function  NXVmGetStringType(_type:PNXStrType_t):longint;cdecl;external libc_nlm name 'NXVmGetStringType';
function  NXVmJoin(wait_for:NXVmId_t; departed_vm:PNXVmId_t; status:Plongint):longint;cdecl;external libc_nlm name 'NXVmJoin';
function  NXVmRegisterExitHandler(cleanup:TCDeclProc1PtrArg; arg:pointer):longint;cdecl;external libc_nlm name 'NXVmRegisterExitHandler';
function  NXVmSetWorkerThreadConfig(reserved:pointer; config:PNXVmWorkerThreadConfig_t):longint;cdecl;external libc_nlm name 'NXVmSetWorkerThreadConfig';
function  NXVmSpawn(name:PNXNameSpec_t; envSpec:PNXExecEnvSpec_t; flags:dword; newVm:PNXVmId_t):longint;cdecl;external libc_nlm name 'NXVmSpawn';
function  NXVmUnregisterExitHandler(func:TCDeclProc1PtrArg; arg:pointer):longint;cdecl;external libc_nlm name 'NXVmUnregisterExitHandler';


// alloca.h
{ Non-standard functions from stdlib.h }
//** void   *alloca( size_t );


// client.h

// stdbool.h

{ origin flags...  }
{ address type flags (used in combination with ORIGIN_ADDRESS)...  }
{ transport type flags...  }
{ NMAS sequence passed...  }
{ string format flags...  }
{ suggested maximum lengths (in characters)...  }

{ turn on 1-byte packing...  }

  const
     ORIGIN_NAME    = $0010;
     ORIGIN_ADDRESS = $0020;    { interpret 'server' as 'netaddr_t'      }
  { address type flags (used in combination with ORIGIN_ADDRESS)...  }
  { address is IPX                         }
     ADDR_IPX = $0100;
     ADDR_IP = $0200;           { address is Internet Protocol           }
     ADDR_IPV6 = $0400;         { address is Internet Protocol version 6 }
  { transport type flags...  }
  { transport type unspecified             }
     XPORT_WILD = $0000;
  { prefer Novell IPX                      }
     XPORT_IPX = $0001;
  { prefer TCP                             }
     XPORT_TCP = $0002;
  { NMAS sequence passed...  }
  { alternative NMAS-based authentication  }
     NMAS_SEQUENCE = $8000;
  { string format flags...  }
     USERNAME_ASCII = $00000;
     USERNAME_UTF8 = $10000;
     USERNAME_UNICODE = $20000;
  { suggested maximum lengths (in characters)...  }
  { ASCII, Unicode or UTF-8                }
     MAX_USERNAME_LEN = 255;
  { (always in ASCII characters)           }
     MAX_PASSWORD_LEN = 255;
     MAX_TREENAME_LEN = 48;
     MAX_SERVERNAME_LEN = 48;


type

   Pnetaddr_t = ^netaddr_t;
   netaddr_t = record
        _type : longint;
        length : size_t;
        address : array[0..47] of byte;
     end;

   Pfrag_t = ^frag_t;
   frag_t = record
        data : pointer;
        length : size_t;
     end;

(** unsupported pragma#pragma pack()*)
{ managing user identity...  }


{$ifndef DisableArrayOfConst}
function build_username(max:size_t; flags:dword; username:Pchar; user:Pchar; args:array of const):longint;cdecl;external libc_nlm name 'build_username';
{$endif}
function build_username(max:size_t; flags:dword; username:Pchar; user:Pchar):longint;cdecl;external libc_nlm name 'build_username';


function create_identity(treename:Pchar; username:pointer; password:Pchar; nmas_sequence:pointer; flags:dword;
           identity:Plongint):longint;cdecl;external libc_nlm name 'create_identity';
function create_server_identity(identity:Plongint):longint;cdecl;external libc_nlm name 'create_server_identity';
function is_valid_identity(identity:longint; error:Plongint):longint;cdecl;external libc_nlm name 'is_valid_identity';
procedure delete_identity(identity:longint);cdecl;external libc_nlm name 'delete_identity';
{ managing NCP sessions with a remote server...  }

function open_ncp_session(identity:longint; flags:dword; servername:Pchar; session:Plongint):longint;cdecl;external libc_nlm name 'open_ncp_session';
function close_ncp_session(session:longint):longint;cdecl;external libc_nlm name 'close_ncp_session';
function send_ncp(session:longint; requestCode:longint; sendFragCount:longint; sendFrags:array of frag_t; replyFragCount:longint;
           replyFrags:array of frag_t; replyFragsUsed:Plongint; ncp_error:Plongint):longint;cdecl;external libc_nlm name 'send_ncp';

// complex.h

{ not presently supported  }


// ctype.h
//  var __ctype : array of byte;cvar;external;
{ standard prototypes...  }

function isalnum(_para1:longint):longint;cdecl;external libc_nlm name 'isalnum';
function isalpha(_para1:longint):longint;cdecl;external libc_nlm name 'isalpha';
function isblank(_para1:longint):longint;cdecl;external libc_nlm name 'isblank';
function iscntrl(_para1:longint):longint;cdecl;external libc_nlm name 'iscntrl';
function isdigit(_para1:longint):longint;cdecl;external libc_nlm name 'isdigit';
function isgraph(_para1:longint):longint;cdecl;external libc_nlm name 'isgraph';
function islower(_para1:longint):longint;cdecl;external libc_nlm name 'islower';
function isprint(_para1:longint):longint;cdecl;external libc_nlm name 'isprint';
function ispunct(_para1:longint):longint;cdecl;external libc_nlm name 'ispunct';
function isspace(_para1:longint):longint;cdecl;external libc_nlm name 'isspace';
function isupper(_para1:longint):longint;cdecl;external libc_nlm name 'isupper';
function isxdigit(_para1:longint):longint;cdecl;external libc_nlm name 'isxdigit';
function tolower(_para1:longint):longint;cdecl;external libc_nlm name 'tolower';
function toupper(_para1:longint):longint;cdecl;external libc_nlm name 'toupper';
function isascii(_para1:longint):longint;cdecl;external libc_nlm name 'isascii';
function toascii(_para1:longint):longint;cdecl;external libc_nlm name 'toascii';
function ismultibyte(_para1:Pchar):longint;cdecl;external libc_nlm name 'ismultibyte';
function Lisalnum(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisalnum';
function Lisalpha(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisalpha';
function Lisblank(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisblank';
function Liscntrl(_para1:Pchar):longint;cdecl;external libc_nlm name 'Liscntrl';
function Lisdigit(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisdigit';
function Lisgraph(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisgraph';
function Lislower(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lislower';
function Lisprint(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisprint';
function Lispunct(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lispunct';
function Lisspace(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisspace';
function Lisupper(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisupper';
function Lisxdigit(_para1:Pchar):longint;cdecl;external libc_nlm name 'Lisxdigit';


// dfs.h

{===========================================================
  Traditional direct file system
  These were interfaced by CLib in its NDK header, nwdfs.h
============================================================}

  const
     DFS_FAILURE = -(1);
     DFS_SUCCESS = 0;
     DFS_ERR_INSUFFICIENT_SPACE = 1;
     DFS_ERR_VOLUME_SEGMENT_DEACTIVATED = 4;
     DFS_ERR_TRUNCATION_FAILURE = 16;
     DFS_ERR_HOLE_IN_FILE = 17;
     DFS_ERR_PARAMETER = 18;
     DFS_ERR_OVERLAP = 19;
     DFS_ERR_SEGMENT = 20;
     DFS_ERR_BOUNDARY = 21;
     DFS_ERR_INSUFFICIENT_LIMBO_FILE_SPACE = 22;
     DFS_ERR_NOT_IN_DIRECT_FILE_MODE = 23;
     DFS_ERR_OPERATION_BEYOND_EOF = 24;
     DFS_ERR_OUT_OF_FILDES = 129;
     DFS_ERR_HARD_IO = 131;
     DFS_ERR_EBADF = 136;
     DFS_ERR_CANT_READ = 147;
     DFS_ERR_CANT_WRITE = 148;
     DFS_ERR_FILE_DETACHED = 149;
     DFS_ERR_ENOMEM = 150;
     DFS_ERR_INVALID_VOLUME = 152;
     DFS_ERR_IO_LOCK = 162;
  { setSizeFlags  }

     DFS_SETSIZE_NON_SPARSE_FILE = $00000001;    { to extend the file  }
     DFS_SETSIZE_NO_ZERO_FILL    = $00000002;    { do not zero fill  }
     DFS_SETSIZE_UNDO_ON_ERROR   = $00000004;    { truncate to original eof  }
     DFS_SETSIZE_PHYSICAL_ONLY   = $00000008;    { change physical EOF only  }
     DFS_SETSIZE_LOGICAL_ONLY    = $00000010;    { change only the logical EOF  }


{ type definitions...  }
type

   Pdfs_filemap_t = ^dfs_filemap_t;
   dfs_filemap_t = record
        FileBlock : longint;
        VolumeBlock : longint;
        NumberOfBlocks : longint;
     end;

   Pdfs_volinfo_t = ^dfs_volinfo_t;
   dfs_volinfo_t = record
        VolumeAllocationUnitSizeInBytes : size_t;
        VolumeSizeInAllocationUnits : size_t;
        VolumeSectorSize : size_t;
        AllocationUnitsUsed : longint;
        AllocationUnitsFreelyAvailable : longint;
        AllocationUnitsInDeletedFilesNotAvailable : longint;
        AllocationUnitsInAvailableDeletedFiles : longint;
        NumberOfPhysicalSegmentsInVolume : longint;
        PhysicalSegmentSizeInAllocationUnits : array[0..63] of size_t;
     end;

   Pdfs_cbparms_t = ^dfs_cbparms_t;
   dfs_cbparms_t = record
        sema : pointer;
        ccode : longint;
     end;
{  dlfcn.h
==============================================================================
=  Interfaces from the Single UNIX Specification of the Open Group for the
=  management of imported symbols and shared (dynamic) libraries. The imple-
=  mentation on NetWare is narrower as noted by the number of features that
=  are ignored or unsupported.
==============================================================================}
  const
     RTLD_LAZY = $01;
     RTLD_NOW = $02;               { ignored }
     RTLD_GLOBAL = $04;            { ignored }
     RTLD_LOCAL = $08;             { ignored }
     RTLD_MULTIPLE = $10;          { NetWare: force-load multiple times  }
     MAX_DLL_NAME_LEN = 8;         { special purpose value for 'handle' in dlsym()...  }
     MAX_SYMNAME_LEN = 255;        { maximim length of 'name' in dlsym()...  }


function dlopen(path:Pchar; mode:longint):pointer;cdecl;external libc_nlm name 'dlopen';
function dlsym(handle:pointer; name:Pchar):pointer;cdecl;external libc_nlm name 'dlsym';
function dlclose(handle:pointer):longint;cdecl;external libc_nlm name 'dlclose';
function dlerror:Pchar;cdecl;external libc_nlm name 'dlerror';


// endian.h
{ defines `__BYTE_ORDER' for the hosting environment...  }
{ some environments use a different "endianness" for floating point values  }

  const
     __LITTLE_ENDIAN = 1234;
     __BIG_ENDIAN = 4321;
     __PDP_ENDIAN = 3412;
  { defines `__BYTE_ORDER' for the hosting environment...  }
     __BYTE_ORDER = __LITTLE_ENDIAN;
     __FLOAT_WORD_ORDER = __BYTE_ORDER;
     LITTLE_ENDIAN = __LITTLE_ENDIAN;
     BIG_ENDIAN = __BIG_ENDIAN;
     PDP_ENDIAN = __PDP_ENDIAN;
     BYTE_ORDER = __BYTE_ORDER;



// err.h

{$ifndef DisableArrayOfConst}
procedure err(_para1:longint; _para2:Pchar; args:array of const);cdecl;external libc_nlm name 'err';
{$endif}
procedure err(_para1:longint; _para2:Pchar);cdecl;external libc_nlm name 'err';
procedure verr(_para1:longint; _para2:Pchar; _para3:va_list);cdecl;external libc_nlm name 'verr';

{$ifndef DisableArrayOfConst}
procedure errx(_para1:longint; _para2:Pchar; args:array of const);cdecl;external libc_nlm name 'errx';
{$endif}
procedure errx(_para1:longint; _para2:Pchar);cdecl;external libc_nlm name 'errx';
procedure verrx(_para1:longint; _para2:Pchar; _para3:va_list);cdecl;external libc_nlm name 'verrx';

{$ifndef DisableArrayOfConst}
procedure warn(_para1:Pchar; args:array of const);cdecl;external libc_nlm name 'warn';
{$endif}
procedure warn(_para1:Pchar);cdecl;external libc_nlm name 'warn';
procedure vwarn(_para1:Pchar; _para2:va_list);cdecl;external libc_nlm name 'vwarn';

{$ifndef DisableArrayOfConst}
procedure warnx(_para1:Pchar; args:array of const);cdecl;external libc_nlm name 'warnx';
{$endif}
procedure warnx(_para1:Pchar);cdecl;external libc_nlm name 'warnx';
procedure vwarnx(_para1:Pchar; _para2:va_list);cdecl;external libc_nlm name 'vwarnx';



// errno.h

  const
     ENOENT = 1;
     E2BIG = 2;           { arg list too big                               }
     ENOEXEC = 3;         { exec format error                              }
     EBADF = 4;           { bad file number                                }
     ENOMEM = 5;          { not enough memory                              }
     EACCES = 6;          { permission denied                              }
     EEXIST = 7;          { file exists                                    }
     EXDEV = 8;           { cross-device link                              }
     EINVAL = 9;          { invalid argument                               }
     ENFILE = 10;         { file table overflow                            }
     EMFILE = 11;         { too many open files                            }
     ENOSPC = 12;         { no space left on device                        }
     EDOM = 13;           { argument too large                             }
     ERANGE = 14;         { result too large                               }
     EDEADLK = 15;        { resource deadlock would occur                  }
  { -------------------------- Miscellaneous NLM Library constants ----------  }
     EINUSE = 16;         { resource(s) in use                             }
     ESERVER = 17;        { server error (memory out, I/O error, etc.)     }
     ENOSERVR = 18;       { no server (queue server, file server, etc.)    }
     EWRNGKND = 19;       { wrong kind--an operation is being...           }
                          {  ...attempted on the wrong kind of object      }
     ETRNREST = 20;       { transaction restarted                          }
     ERESOURCE = 21;      { resources unavailable (maybe permanently)      }
     EBADHNDL = 22;       { bad non-file handle (screen, semaphore, etc.)  }
     ENO_SCRNS = 23;      { screen I/O attempted when no screens           }
  { -------------------------- Additional POSIX / traditional UNIX constants   }
     EAGAIN = 24;         { resource temporarily unavailable               }
     ENXIO = 25;          { no such device or address                      }
     EBADMSG = 26;        { not a data message                             }
     EFAULT = 27;         { bad address                                    }
     EIO = 28;            { physical I/O error                             }
     ENODATA = 29;        { no data                                        }
     ENOSTRMS = 30;       { streams not available                          }
  { Berkeley sockets constants ------------------  }
     EPROTO = 31;         { fatal protocol error                           }
     EPIPE = 32;          { broken pipe                                    }
     ESPIPE = 33;         { illegal seek                                   }
  { Non-blocking and interrupt I/O constants ----  }
     ETIME = 34;          { ioctl acknowledge timeout                      }
     EWOULDBLOCK = 35;    { operation would block                          }
     EINPROGRESS = 36;    { operation now in progress                      }
     EALREADY = 37;       { operation already in progress                  }
  { IPC network argument constants --------------  }
     ENOTSOCK = 38;       { socket operation on non-socket                 }
     EDESTADDRREQ = 39;   { destination address required                   }
     EMSGSIZE = 40;       { message too long                               }
     EPROTOTYPE = 41;     { protocol wrong type for socket                 }
     ENOPROTOOPT = 42;    { protocol not available                         }
     EPROTONOSUPPORT = 43;{ protocol not supported                         }
     ESOCKTNOSUPPORT = 44;{ socket type not supported                      }
     EOPNOTSUPP = 45;     { operation not supported on socket              }
     EPFNOSUPPORT = 46;   { protocol family not supported                  }
     EAFNOSUPPORT = 47;   { address family unsupported by protocol family  }
     EADDRINUSE = 48;     { address already in use                         }
     EADDRNOTAVAIL = 49;  { can't assign requested address                 }
  { Operational constants -----------------------  }
     ENETDOWN = 50;       { network is down                                }
     ENETUNREACH = 51;    { network is unreachable                         }
     ENETRESET = 52;      { network dropped connection on reset            }
     ECONNABORTED = 53;   { software caused connection abort               }
     ECONNRESET = 54;     { connection reset by peer                       }
     ENOBUFS = 55;        { no buffer space available                      }
     EISCONN = 56;        { socket is already connected                    }
     ENOTCONN = 57;       { socket is not connected                        }
     ESHUTDOWN = 58;      { can't send after socket shutdown               }
     ETOOMANYREFS = 59;   { too many references: can't splice              }
     ETIMEDOUT = 60;      { connection timed out                           }
     ECONNREFUSED = 61;   { connection refused                             }
  { -------------------------- Additional POSIX-mandated constants ----------  }
     EBUSY = 62;          { resource busy                                  }
     EINTR = 63;          { interrupted function call                      }
     EISDIR = 64;         { is a directory                                 }
     ENAMETOOLONG = 65;   { filename too long                              }
     ENOSYS = 66;         { function not implemented                       }
     ENOTDIR = 67;        { not a directory                                }
     ENOTEMPTY = 68;      { directory not empty                            }
     EPERM = 69;          { operation not permitted                        }
     ECHILD = 70;         { no child process                               }
     EFBIG = 71;          { file too large                                 }
     EMLINK = 72;         { too many links                                 }
     ENODEV = 73;         { no such device                                 }
     ENOLCK = 74;         { no locks available                             }
     ENOTTY = 75;         { inappropriate I/O control operation            }
     EFTYPE = ENOTTY;     { inappropriate operation for file type       }
     EROFS = 76;          { read-only file system                          }
     ESRCH = 77;          { no such process                                }
     ECANCELED = 78;      { operation was cancelled                        }
     ENOTSUP = 79;        { this optional functionality not supported      }
     ECANCELLED = ECANCELED;
     EBADTYPE = 80;       { bad type for operation                         }
     EOVERFLOW = 81;      { operation would overflow                       }
     EHOSTDOWN = 82;      { host is down                                   }
     EHOSTUNREACH = 83;   { no route to host                               }
     EPROCLIM = 84;       { too many processes                             }
  { -------------------------- Additional POSIX / traditional UNIX constants   }
     ENOMSG = 90;         { message does not exist                         }
  { -------------------------- LibC-implementation-specific constants -------  }
     ENLMDATA = 100;      { anomaly in NLM data structure                  }
     EILSEQ = 101;        { illegal character sequence in multibyte        }
     EINCONSIS = 102;     { internal library inconsistency                 }
     EDOSTEXTEOL = 103;   { DOS-text file inconsistency--no newline...     }
                          { ...after carriage return                       }
     ENONEXTANT = 104;    { object doesn't exist                           }
     ENOCONTEXT = 105;    { no thread library context present              }
     ENAMESPACE = 106;    { invalid namespace or operation                 }
     EBADCONN = 107;      { invalid connection                             }
     ENAMEINVAL = 108;    { invalid NDS name                               }
     EPASSINVAL = 109;    { invalid password                               }
     ENCPINVAL = 110;     { invalid or erroneous NCP                       }
     EFILESYS = 111;      { generic file system error, see 'filesyserrno'  }
     ESUFFICIENT = 112;   { insufficient space for any operation result    }
     EPARTONLY = 113;     { partial result only  for lack of space         }
     EBADIDENT = 114;     { invalid user or other identity                 }
     ENDS = 115;          { generic eDirectory error, see 'h_errno'        }
     ENCP = 116;          { generic NCP error, see 'h_errno'               }
     ELOOKUP = 117;       { generic look-up error, see 'h_errno'           }
     ELASTERR = ELOOKUP;


function ___errno:Plongint;cdecl;external libc_nlm name '___errno';
function __errno_location:Plongint;cdecl;external libc_nlm name '___errno';
function ___lastClientErrno:Plongint;cdecl;external libc_nlm name '___lastClientErrno';
function ___lastFileSysErrno:Plongint;cdecl;external libc_nlm name '___lastFileSysErrno';


// esm.h

  const
     ERR_ESM_AVAL = 1;
     ERR_SIZE_ZERO = 2;           { ESMAlloc size requested is zero          }
     ERR_TABLE_FULL = 3;          { allocation table is full                 }
     ERR_NOT_CONTIGUOUS = 4;      { ESMAlloc request cannot be continguous   }
     ERR_INVAL_ADDRESS = 5;       { already free or out of range             }
     ERR_INVAL_SRC_ADDR = 6;      { bad source address (ESMCopy)             }
     ERR_INVAL_DEST_ADDR = 7;     { bad destination address(ESMCopy/ESMFill) }
     ERR_SRC_DEST_OVERLAP = 8;    { buffer overlap (ESMCopy)                 }
  { (will be obsolete when the overlapping buffer copy implemented...)         }
     ERR_MAP_4M_PAGE = 9;         { mapping particular 4M page failed        }
     ERR_BUFFER_SIZE = 10;        { buffer passed too small (ESMQuery)       }
     ERR_LOGICAL_SPACE = 11;      { adequate logical/window not available    }
     ERR_ACQUIRING_LOCK = 12;     { failed to acquire lock (please retry)    }

type
   Paddr64_t = ^addr64_t;
   addr64_t = Tuint64;

   PESMQueryInfo_t = ^ESMQueryInfo_t;
   ESMQueryInfo_t = record
        TotalExtendedMemory : size64_t;
        RemainingExtendedMemory : size64_t;
        TotalMemoryBelow4G : size_t;
     end;

function ESMAlloc(size:size64_t; options:dword; esmAddress:Paddr64_t):longint;cdecl;external libc_nlm name 'ESMAlloc';
function ESMAllocWindow(size:size_t; logicalAddress:Ppointer; callerID:pointer):longint;cdecl;external libc_nlm name 'ESMAllocWindow';
function ESMCopy(source:addr64_t; destination:addr64_t; length:size64_t):longint;cdecl;external libc_nlm name 'ESMCopy';
function ESMFill(pattern:dword; destination:addr64_t; length:size64_t):longint;cdecl;external libc_nlm name 'ESMFill';
function ESMFree(esmAddress:addr64_t):longint;cdecl;external libc_nlm name 'ESMFree';
function ESMFreeWindow(logicalAddress:pointer; callerID:pointer):longint;cdecl;external libc_nlm name 'ESMFreeWindow';
function ESMMapMemory(windowAddress:pointer; memoryAddress:addr64_t; size:size_t):longint;cdecl;external libc_nlm name 'ESMMapMemory';
function ESMQuery(bufferSize:size_t; buffer:PESMQueryInfo_t):longint;cdecl;external libc_nlm name 'ESMQuery';



// float.h
{ turn on 1-byte packing...  }

{
typedef union __fp_u

   unsigned char __uc[16];
   float         __f;
   double        __d;
   long double   __ld;
 __fp_u;
 }
{
typedef struct __fp_s

   int    __MANT_DIG;
   int    __DIG;
   int    __MIN_EXP;
   int    __MIN_10_EXP;
   int    __MAX_EXP;
   int    __MAX_10_EXP;
   __fp_u __EPSILON[2];
   __fp_u __MIN[2];
   __fp_u __MAX[2];
 __fp_s;
 }

(** unsupported pragma#pragma pack()*)
{ extern const __fp_s __fp_characteristics[3]; }


// fenv.h

{ these interfaces not presently supported!  }
{ floating-point exception bits for 'excepts' argument  }

{ turn on 1-byte packing...  }

  { these interfaces not presently supported!  }

 const
   _MAX_FPFLAGS  = 8;
  { floating-point exception bits for 'excepts' argument  }
   FE_DIVBYZERO  = $00000001;
   FE_INEXACT    = $00000002;
   FE_INVALID    = $00000004;
   FE_OVERFLOW   = $00000008;
   FE_UNDERFLOW  = $00000010;
   FE_ALL_EXCEPT = $00000020;
   FE_DOWNWARD   = $00000040;
   FE_TONEAREST  = $00000080;
   FE_TOWARDZERO = $00000100;
   FE_UPWARD     = $00000200;
   FE_DFL_ENV    = $00000400;


type
   Pfexcept_t = ^fexcept_t;
   fexcept_t = dword;

   Pfenv_t = ^fenv_t;
   fenv_t = record
        excepts : longint;
        flagp : fexcept_t;
     end;


(** unsupported pragma#pragma pack()*)
{ not presently supported...  }

procedure feclearexcept(excepts:longint);cdecl;external libc_nlm name 'feclearexcept';
procedure fegetexceptflag(flagp:Pfexcept_t; excepts:longint);cdecl;external libc_nlm name 'fegetexceptflag';
procedure feraiseexceptflag(flagp:Pfexcept_t; excepts:longint);cdecl;external libc_nlm name 'feraiseexceptflag';
procedure fesetexceptflag(flagp:Pfexcept_t; excepts:longint);cdecl;external libc_nlm name 'fesetexceptflag';
function fetestexcept(excepts:longint):longint;cdecl;external libc_nlm name 'fetestexcept';
function fegetround(round:longint):longint;cdecl;external libc_nlm name 'fegetround';
function fesetround:longint;cdecl;external libc_nlm name 'fesetround';
function fegetenv(envp:Pfenv_t):longint;cdecl;external libc_nlm name 'fegetenv';
function feholdexcept(envp:Pfenv_t):longint;cdecl;external libc_nlm name 'feholdexcept';
procedure fesetenv(envp:Pfenv_t);cdecl;external libc_nlm name 'fesetenv';
procedure feupdateenv(envp:Pfenv_t);cdecl;external libc_nlm name 'feupdateenv';



// fnmatch.h

  const
     FNM_NOMATCH = -(1);     { string fails to match pattern  }
  { values for field 'flags'...  }
     FNM_NOSYS       = $01;  { reserved (unused)                              }
     FNM_PATHNAME    = $02;  { slash in string must match in pattern          }
     FNM_PERIOD      = $04;  { leading period in string must match in pattern }
     FNM_NOESCAPE    = $08;  { disable backslash escaping                     }
     FNM_CASEFOLD    = $10;  { ignore case                                    }
     FNM_LEADING_DIR = $20;  { Ignore `/...' after a match                    }
     FNM_FILE_NAME   = FNM_PATHNAME;



function fnmatch(pattern, _string:Pchar; flags:longint):longint;cdecl;external libc_nlm name 'fnmatch';



// fshooks.h

  const
     FSHOOK_MAY_NOT_SLEEP_BIT    = $00000001;
     FSHOOK_SORT_LOW_TO_HIGH_BIT = $00000002;
     FSHOOK_CONSUMABLE_BIT       = $00000004;
     FSHOOK_NO_SA_BIT            = $00000008;
     FSHOOK_NO_AUDITOR_BIT       = $00000010;
     FSHOOK_NO_CHECK_BIT         = $00000020;
     FSHOOK_NESL_DATA_BIT        = $00000040;
     FSHOOK_NESL_SHIM_BIT        = $00000080;
     FSHOOK_DATA_FILTERED_BIT    = $80000000;

     FSHOOK_TYPE_DELETE_WARN               = 0;
     FSHOOK_TYPE_DELETE_REPORT             = 1;
     FSHOOK_TYPE_CREATE_WARN               = 2;
     FSHOOK_TYPE_CREATE_REPORT             = 3;
     FSHOOK_TYPE_OPEN_WARN                 = 4;
     FSHOOK_TYPE_OPEN_REPORT               = 5;
     FSHOOK_TYPE_CLOSE_WARN                = 6;
     FSHOOK_TYPE_CLOSE_REPORT              = 7;
     FSHOOK_TYPE_RENAME_WARN               = 8;
     FSHOOK_TYPE_RENAME_REPORT             = 9;
     FSHOOK_TYPE_MODIFYINFO_WARN           = 10;
     FSHOOK_TYPE_MODIFYINFO_REPORT         = 11;
     FSHOOK_TYPE_SETDATASIZE_WARN          = 12;
     FSHOOK_TYPE_SETDATASIZE_REPORT        = 13;
     FSHOOK_TYPE_ADDTRUSTEE_WARN           = 14;
     FSHOOK_TYPE_ADDTRUSTEE_REPORT         = 15;
     FSHOOK_TYPE_REMOVETRUSTEE_WARN        = 16;
     FSHOOK_TYPE_REMOVETRUSTEE_REPORT      = 17;
     FSHOOK_TYPE_SETINHERITEDRIGHTS_WARN   = 18;
     FSHOOK_TYPE_SETINHERITEDRIGHTS_REPORT = 19;
     FSHOOK_TYPE_CHANGEVOLSTATE_WARN       = 20;
     FSHOOK_TYPE_CHANGEVOLSTATE_REPORT     = 21;
     FSHOOK_TYPE_CHANGEPOOLSTATE_WARN      = 22;
     FSHOOK_TYPE_CHANGEPOOLSTATE_REPORT    = 23;


{==========================================================
  NSS file system hooks
  These are based on the Novell Event Bus, supported by NSS
==========================================================}
type
   Pfse_info = ^fse_info;
   fse_info = record
        version   : longint;                  // event block version
        reserved1 : longint;                  // do not modify
        reserved2 : pointer;                  // do not modify
        rtag      : rtag_t;                   // registerer's resource tag
        link      : Pfse_info;                // used by registerer to link blocks
        regID     : pointer;                  // registerer of the event
        regSpace  : pointer;                  // scratch space for registerer's use
        consID    : pointer;                  // consumer of event if relevant
        length    : size_t;                   // in bytes of event data
        data      : pointer;                  // pointer to data
        reserved3 : array[0..3] of pointer;   // do not modify
        _type     : longint;                  // one of NSS_FSTYPE_-...
        userParm  : pointer;                  // specified at time of registration
        parm0     : pointer;                  // value depends on event type
        parm1     : pointer;                  // ibid
        flags     : dword;                    // as noted above
     end;
   fsevent_info_t = fse_info;
   Pfsevent_info_t = ^fsevent_info_t;
   Pfsevent_info = Pfsevent_info_t;
   Tfsevent_info = fsevent_info_t;

   Pzkey_t = ^zkey_t;
   zkey_t = Tuint64;

   Pzid_t = ^zid_t;
   zid_t = Tuint64;

   Pvolid_t = ^volid_t;
   volid_t = record
        timeLow : dword;
        timeMid : word;
        timeHighAndVersion : word;
        clockSeqHighAndReserved : byte;
        clockSeqLow : byte;
        node : array[0..5] of byte;
     end;
   userid_t = volid_t;
   Puserid_t = ^userid_t;
   TVolId = volid_t;
   PVolId = pvolid_t;
   TUserId = userid_t;
   PUserId = puserid_t;

   Ptimeinfo_t = ^timeinfo_t;
   timeinfo_t = record
        accessedTime,                    // last time file was accessed
        createdTime,                     // time file was created
        modifiedTime,                    // last time data was changed
        metaDataModifiedTime : time_t;   // last time metadata was changed
     end;
   Ttimeinfo = timeinfo_t;
   Ptimeinfo = Ptimeinfo_t;

{ commonlity in call-back structures...  }
{ NSS (NetWare 6) filesystem hooks events and call-back data structures...  }

   Pdel_warn_t = ^del_warn_t;
   del_warn_t = record
        enterExitID,
        slotID,
        taskID : dword;
        zid    : zid_t;
        volID  : volid_t;
     end;
   Tdel_warn = del_warn_t;
   Pdel_warn = Pdel_warn_t;


   Pdel_report_t = ^del_report_t;
   del_report_t = record
        enterExitID    : dword;
        enterRetStatus,
        opRetCode      : longint;
     end;
   Tdel_report = del_report_t;
   Pdel_report = Pdel_report_t;

   Pcreate_warn_t = ^create_warn_t;
   create_warn_t = record
        enterExitID,
        slotID,
        taskID          : dword;
        zid             : zid_t;
        volID           : volid_t;
        name            : Punicode_t;
        fileType,
        fileAttributes,
        createFlags     : dword;
        createParms     : pointer;
        requestedRights,
        createAndOpen   : dword;
     end;
   Tcreate_warn = create_warn_t;
   Pcreate_warn = Pcreate_warn_t;

   Pcreate_report_t = ^create_report_t;
   create_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
        retOpenCreateAction : dword;
        retKey : zkey_t;
        retZid : zid_t;
        retVolID : volid_t;
        times : timeinfo_t;
     end;
   Tcreate_report = create_report_t;
   Pcreate_report = Pcreate_report_t;

   Popen_warn_t = ^open_warn_t;
   open_warn_t = record
        enterExitID : dword;
        slotID : dword;
        taskID : dword;
        zid : zid_t;
        volID : volid_t;
        requestedRights : dword;
        openParms : pointer;
     end;
   Topen_warn = open_warn_t;
   Popen_warn = Popen_warn_t;

   Popen_report_t = ^open_report_t;
   open_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
        retKey : zkey_t;
        retZid : zid_t;
        retVolID : volid_t;
        times : timeinfo_t;
     end;
   Topen_report = open_report_t;
   Popen_report = Popen_report_t;

   Pclose_warn_t = ^close_warn_t;
   close_warn_t = record
        enterExitID : dword;
        slotID : dword;
        key : zkey_t;
        fhState : dword;
        times : timeinfo_t;
     end;
   Tclose_warn = close_warn_t;
   Pclose_warn = Pclose_warn_t;

   Pclose_report_t = ^close_report_t;
   close_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
        fileDeleted : dword;
     end;
   Tclose_report = close_report_t;
   Pclose_report = Pclose_report_t;

   Pren_warn_t = ^ren_warn_t;
   ren_warn_t = record
        enterExitID : dword;
        slotID : dword;
        taskID : dword;
        zid : zid_t;
        volID : volid_t;
        destZid : zid_t;
        destName : Punicode_t;
        renameFlags : dword;
     end;
   Tren_warn = ren_warn_t;
   Pren_warn = Pren_warn_t;

   Pren_report_t = ^ren_report_t;
   ren_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
     end;
   Tren_report = ren_report_t;
   Pren_report = Pren_report_t;


   Pzinfo_t = ^zinfo_t;
   zinfo_t = record
        version : dword;
        totalBytes : size_t;
        nextByte : size_t;
        padding : dword;
        retMask : Tuint64;
        std : record
             zid : zid_t;
             dataStreamZid : zid_t;
             parentZid : zid_t;
             logicalEOF : off64_t;
             volumeID : volid_t;
             fileType : dword;
             fileAttributes : dword;
             fileAttributesModMask : dword;
             padding : dword;
          end;
        storageUsed : record
             physicalEOF : size64_t;
             dataBytes : size64_t;
             metaDataBytes : size64_t;
          end;
        primaryNameSpaceID : longint;
        nameStart : off_t;
        names : record
             numEntries : size_t;
             fileNameArray : off_t;
          end;
        time : record
             created : Tuint64;
             archived : Tuint64;
             modified : Tuint64;
             accessed : Tuint64;
             metaDataModified : Tuint64;
          end;
        id : record
             owner : userid_t;
             archiver : userid_t;
             modifier : userid_t;
             metaDataModifier : userid_t;
          end;
        blockSize : record
             size : size_t;
             sizeShift : off_t;
          end;
        count : record
             open : longint;
             hardLink : longint;
          end;
        dataStream : record
             count : longint;
             totalNameSize : size_t;
             totalDataSize : size64_t;
          end;
        extAttr : record
             count : size_t;
             totalNameSize : size_t;
             totalDataSize : size64_t;
          end;
        deleted : record
             time : Tuint64;
             id : userid_t;
          end;
        macNS : record
             finderInfo : record
                 case longint of
                    0 : ( generic : array[0..31] of byte );
                    1 : ( macintosh : record
                         FInfo : record
                              fdType : longint;
                              fdCreator : longint;
                              fdFlags : Tint16;
                              fdLocation : record
                                   v : Tint16;
                                   h : Tint16;
                                end;
                              fdFldr : Tint16;
                           end;
                         extended : record
                             case longint of
                                0 : ( FXInfo : record
                                     fdIconID : Tint16;
                                     fdUnused : Tint16;
                                     fdScript : int8_t;
                                     fdFlags : int8_t;
                                     fdComment : Tint16;
                                     fdPutAway : longint;
                                  end );
                                1 : ( DXInfo : record
                                     frScroll : record
                                          v : Tint16;
                                          h : Tint16;
                                       end;
                                     frOpenChain : longint;
                                     fdScript : int8_t;
                                     fdFlags : int8_t;
                                     fdComment : Tint16;
                                     fdPutAway : longint;
                                  end );
                             end;
                      end );
                 end;
             proDOSInfo : array[0..5] of byte;
             filler : array[0..1] of byte;
             dirRightsMask : dword;
          end;
        unixNS : record
             fMode : dword;
             rDev : dword;
             myFlags : dword;
             nfsUID : dword;
             nfsGID : dword;
             nwUID : dword;
             nwGID : dword;
             nwEveryone : dword;
             nwUIDRights : dword;
             nwGIDRights : dword;
             nwEveryoneRights : dword;
             acsFlags : byte;
             firstCreated : byte;
             variableSize : size_t;
             offsetToData : off_t;
          end;
        volumeID : volid_t;
        ndsObjectID : userid_t;
        volumeState : dword;
        nameSpaceMask : dword;
        features : record
             enabled : Tuint64;
             enableModMask : Tuint64;
             supported : Tuint64;
          end;
        maximumFileSize : size64_t;
        totalSpaceQuota : size64_t;
        numUsedBytes : size64_t;
        numObjects : size64_t;
        numFiles : size64_t;
        authModelID : dword;
        dataShreddingCount : size_t;
        salvage : record
             purgeableBytes : size64_t;
             nonPurgeableBytes : size64_t;
             numDeletedFiles : size64_t;
             oldestDeletedTime : Tuint64;
             minKeepSeconds : size_t;
             maxKeepSeconds : size_t;
             lowWaterMark : size_t;
             highWaterMark : size_t;
          end;
        comp : record
             numCompressedFiles : size64_t;
             numCompDelFiles : size64_t;
             numUncompressibleFiles : size64_t;
             numPreCompressedBytes : size64_t;
             numCompressedBytes : size64_t;
          end;
        pool : record
             poolID : volid_t;
             ndsObjectID : userid_t;
             poolState : dword;
             nameSpaceMask : dword;
             features : record
                  enabled : Tuint64;
                  enableModMask : Tuint64;
                  supported : Tuint64;
               end;
             totalSpace : size64_t;
             numUsedBytes : size64_t;
             purgeableBytes : size64_t;
             nonPurgeableBytes : size64_t;
          end;
        extAttrUserFlags : dword;
        variableData : array[0..0] of byte;
     end;
   Tzinfo = zinfo_t;
   Pzinfo = Pzinfo_t;

   Pmod_warn_t = ^mod_warn_t;
   mod_warn_t = record
        enterExitID : dword;
        slotID : dword;
        taskID : dword;
        zid : zid_t;
        volID : volid_t;
        modifyInfoMask : dword;
        modifyTypeInfoMask : dword;
        modifyInfo : Pzinfo_t;
        modifyTypeInfo : pointer;
     end;
   Tmod_warn = mod_warn_t;
   Pmod_warn = Pmod_warn_t;

   Pmod_report_t = ^mod_report_t;
   mod_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
     end;
   Tmod_report = mod_report_t;
   Pmod_report = Pmod_report_t;

   Psetsize_warn_t = ^setsize_warn_t;
   setsize_warn_t = record
        enterExitID : dword;
        slotID : dword;
        key : zkey_t;
        curEOF : Tuint64;
        newEOF : Tuint64;
        setSizeFlags : dword;
     end;
   Tsetsize_warn = setsize_warn_t;
   Psetsize_warn = Psetsize_warn_t;

   Psetsize_report_t = ^setsize_report_t;
   setsize_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
        newEOF : Tuint64;
     end;
   Tsetsize_report = setsize_report_t;
   Psetsize_report = Psetsize_report_t;

   Paddtrustee_warn_t = ^addtrustee_warn_t;
   addtrustee_warn_t = record
        enterExitID : dword;
        slotID : dword;
        taskID : dword;
        zid : zid_t;
        volID : volid_t;
        trusteeID : userid_t;
        rights : dword;
        attributes : dword;
     end;
   Taddtrustee_warn = addtrustee_warn_t;
   Paddtrustee_warn = Paddtrustee_warn_t;

   Paddtrustee_report_t = ^addtrustee_report_t;
   addtrustee_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
     end;

   Taddtrustee_report = addtrustee_report_t;
   Paddtrustee_report = Paddtrustee_report_t;

   Premtrustee_warn_t = ^remtrustee_warn_t;
   remtrustee_warn_t = record
        enterExitID : dword;
        slotID : dword;
        taskID : dword;
        zid : zid_t;
        volID : volid_t;
        trusteeID : userid_t;
        purgedFileFlag : dword;
     end;
   Tremtrustee_warn = remtrustee_warn_t;
   Premtrustee_warn = Premtrustee_warn_t;

   Premtrustee_report_t = ^remtrustee_report_t;
   remtrustee_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
     end;
   Tremtrustee_report = remtrustee_report_t;
   Premtrustee_report = Premtrustee_report_t;

   Psetrights_warn_t = ^setrights_warn_t;
   setrights_warn_t = record
        enterExitID : dword;
        slotID : dword;
        taskID : dword;
        zid : zid_t;
        volID : volid_t;
        inheritedRights : dword;
        authorizeFlag : longint;
     end;
   Tsetrights_warn = setrights_warn_t;
   Psetrights_warn = Psetrights_warn_t;

   Psetrights_report_t = ^setrights_report_t;
   setrights_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
     end;
   Tsetrights_report = setrights_report_t;
   Psetrights_report = Psetrights_report_t;

   Pxvolstate_warn_t = ^xvolstate_warn_t;
   xvolstate_warn_t = record
        enterExitID : dword;
        oldState : word;
        newState : word;
        mode : dword;
        volID : volid_t;
        poolID : volid_t;
     end;
   Txvolstate_warn = xvolstate_warn_t;
   Pxvolstate_warn = Pxvolstate_warn_t;

   Pxvolstate_report_t = ^xvolstate_report_t;
   xvolstate_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
        oldState : word;
        newState : word;
        mode : dword;
        volID : volid_t;
        poolID : volid_t;
     end;
   Txvolstate_report = xvolstate_report_t;
   Pxvolstate_report = Pxvolstate_report_t;

   Pxpoolstate_warn_t = ^xpoolstate_warn_t;
   xpoolstate_warn_t = record
        enterExitID : dword;
        oldState : word;
        newState : word;
        mode : dword;
        poolID : volid_t;
     end;
   Txpoolstate_warn = xpoolstate_warn_t;
   Pxpoolstate_warn = Pxpoolstate_warn_t;


   Pxpoolstate_report_t = ^xpoolstate_report_t;
   xpoolstate_report_t = record
        enterExitID : dword;
        enterRetStatus : longint;
        opRetCode : longint;
        oldState : word;
        newState : word;
        mode : dword;
        poolID : volid_t;
     end;
   Txpoolstate_report = xpoolstate_report_t;
   Pxpoolstate_report = Pxpoolstate_report_t;

{============================================================
 Traditional file system hooks
 These were interfaced by CLib in its NDK header, nwfshook.h.
============================================================}

   PEraseFileCallBackStruct = ^TEraseFileCallBackStruct;
   TEraseFileCallBackStruct = record
     case longint of
      0: (slot : longint);
      1: (connection : longint;
          task : longint;
          volume : longint;
          dirBase : longint;
          pathString : Pchar;
          pathComponentCount : longint;
          nameSpace : longint;
          attributeMatchBits : dword);
     end;
{ (see fsio.h)  }

   POpenFileCallBackStruct = ^TOpenFileCallBackStruct;
   TOpenFileCallBackStruct = record
        case longint of
          0: (slot : longint);
          1: (connection : longint;
              task : longint;
              volume : longint;
              dirBase : longint;
              pathString : Pchar;
              pathComponentCount : longint;
              nameSpace : longint;
              attributeMatchBits : dword;
              requestedAccessRights : dword;
              dataStreamNumber : longint;
              fileHandle : Plongint);
     end;

   PCreateFileCallBackStruct = ^TCreateFileCallBackStruct;
   TCreateFileCallBackStruct = record
      case integer of
        0: (connection : longint);
        1: (slot : longint;
            task : longint;
            volume : longint;
            dirBase : longint;
            pathString : Pchar;
            pathComponentCount : longint;
            nameSpace : longint;
            createAttributeBits : dword;
            createFlagBits : dword;
            dataStreamNumber : longint;
            fileHandle : Plongint);
     end;

   PCreateAndOpenCallBackStruct = ^TCreateAndOpenCallBackStruct;
   TCreateAndOpenCallBackStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           volume : longint;
           dirBase : longint;
           pathString : Pchar;
           pathComponentCount : longint;
           nameSpace : longint;
           createAttributeBits : dword;
           requestedAccessRights : dword;
           createFlagBits : dword;
           dataStreamNumber : longint;
           fileHandle : Plongint);
     end;

   PRenameMoveEntryCallBackStruct = ^TRenameMoveEntryCallBackStruct;
   TRenameMoveEntryCallBackStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           volume : longint;
           dirBase : longint;
           pathString : Pchar;
           pathComponentCount : longint;
           nameSpace : longint;
           attributeMatchBits : dword;
           subDirsOnlyFlag : longint;
           newDirBase : longint;
           newPathString : Pchar;
           originalNewCount : longint;
           compatibilityFlag : dword;
           allowRenamesToMyselfFlag : longint);
     end;

   PCloseFileCallBackStruct = ^TCloseFileCallBackStruct;
   TCloseFileCallBackStruct = record
     case integer of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           fileHandle : longint);
     end;

   PCreateDirCallBackStruct = ^TCreateDirCallBackStruct;
   TCreateDirCallBackStruct = record
     case integer of
       0: (connection : longint);
       1: (slot : longint;
           volume : longint;
           dirBase : longint;
           pathString : Pchar;
           pathComponentCount : longint;
           nameSpace : longint;
           directoryAccessMask : dword);
     end;

   PDeleteDirCallBackStruct = ^TDeleteDirCallBackStruct;
   TDeleteDirCallBackStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           volume : longint;
           dirBase : longint;
           pathString : Pchar;
           pathComponentCount : longint;
           nameSpace : longint);
     end;

   Tmodifyvector = record
             MModifyName : Pchar;
             MFileAttributes : dword;
             MFileAttributesMask : dword;
             MCreateDate : word;
             MCreateTime : word;
             MOwnerID : dword;
             MLastArchivedDate : word;
             MLastArchivedTime : word;
             MLastArchivedID : dword;
             MLastUpdatedDate : word;
             MLastUpdatedTime : word;
             MLastUpdatedID : dword;
             MLastAccessedDate : word;
             MInheritanceGrantMask : word;
             MInheritanceRevokeMask : word;
             MMaximumSpace : size_t;
             MLastUpdatedInSeconds : time_t;
          end;
   Pmodifyvector = ^Tmodifyvector;

   PModifyDirEntryCallBackStruct = ^TModifyDirEntryCallBackStruct;
   TModifyDirEntryCallBackStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           volume : longint;
           dirBase : longint;
           pathString : Pchar;
           pathComponentCount : longint;
           nameSpace : longint;
           attributeMatchBits : dword;
           targetNameSpace : longint;
           modifyVector : Pmodifyvector;
           modifyBits : dword;
           allowWildCardsFlag : longint);
     end;

   PSalvageDeletedCallBackStruct = ^TSalvageDeletedCallBackStruct;
   TSalvageDeletedCallBackStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           volume : longint;
           dirBase : longint;
           toBeSalvagedDirBase : longint;
           nameSpace : longint;
           newName : Pchar);
     end;

   PPurgeDeletedCallBackStruct = ^TPurgeDeletedCallBackStruct;
   TPurgeDeletedCallBackStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           volume : longint;
           dirBase : longint;
           toBePurgedDirBase : longint;
           nameSpace : longint);
     end;

   PRenameNSEntryCallBackStruct = ^TRenameNSEntryCallBackStruct;
   TRenameNSEntryCallBackStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           volume : longint;
           dirBase : longint;
           pathString : Pchar;
           pathComponentCount : longint;
           nameSpace : longint;
           matchBits : dword;
           newName : Pchar);
     end;

   PGenericSalvageDeletedCBStruct = ^TGenericSalvageDeletedCBStruct;
   TGenericSalvageDeletedCBStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           nameSpace : longint;
           sequence : longint;
           volume : longint;
           dirBase : longint;
           newName : Pchar);
     end;

   PGenericPurgeDeletedCBStruct = ^TGenericPurgeDeletedCBStruct;
   TGenericPurgeDeletedCBStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           nameSpace : longint;
           sequence : longint;
           volume : longint;
           dirBase : longint);
     end;

   PGenericOpenCreateCBStruct = ^TGenericOpenCreateCBStruct;
   TGenericOpenCreateCBStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           volume : longint;
           pathComponentCount : longint;
           dirBase : longint;
           pathString : Pchar;
           nameSpace : longint;
           dataStreamNumber : longint;
           openCreateFlags : dword;
           searchAttributes : dword;
           createAttributes : dword;
           requestedAccessRights : dword;
           returnInfoMask : dword;
           fileHandle : Plongint;
           openCreateAction : Pchar);
     end;

   PGenericRenameCBStruct = ^TGenericRenameCBStruct;
   TGenericRenameCBStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           nameSpace : longint;
           renameFlag : longint;
           searchAttributes : dword;
           srcVolume : longint;
           srcPathComponentCount : longint;
           srcDirBase : longint;
           srcPathString : Pchar;
           dstVolume : longint;
           dstPathComponentCount : longint;
           dstDirBase : longint;
           dstPathString : Pchar);
     end;

   PGenericEraseFileCBStruct = ^TGenericEraseFileCBStruct;
   TGenericEraseFileCBStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           volume : longint;
           pathComponentCount : longint;
           dirBase : longint;
           pathString : Pchar;
           nameSpace : longint;
           searchAttributes : dword);
     end;

   PGenericModifyDOSInfoCBStruct = ^TGenericModifyDOSInfoCBStruct;
   TGenericModifyDOSInfoCBStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           volume : longint;
           pathComponentCount : longint;
           dirBase : longint;
           pathString : Pchar;
           nameSpace : longint;
           searchAttributes : dword;
           modifyMask : dword;
           modifyInfo : pointer);
     end;

   PGenericModifyNSInfoCBStruct = ^TGenericModifyNSInfoCBStruct;
   TGenericModifyNSInfoCBStruct = record
     case longint of
       0: (connection : longint);
       1: (slot : longint;
           task : longint;
           dataLength : size_t;
           srcNameSpace : longint;
           dstNameSpace : longint;
           volume : longint;
           dirBase : longint;
           modifyMask : dword;
           modifyInfo : pointer);
     end;
{============================================================================
** NSS file system hook prototypes...
 }

//type TCdeclPfsEventFunc = function (info:Pfsevent_info_t):longint; cdecl;
type TCdeclPfsEventFunc = function (var info:Tfsevent_info):longint; cdecl;

function fs_register(_type:longint; cbFunc:TCdeclPfsEventFunc; userParm:pointer):longint;cdecl;external libc_nlm name 'fs_register';
function fs_unregister(_type:longint; cbFunc:TCdeclPfsEventFunc):longint;cdecl;external libc_nlm name 'fs_unregister';

function fs_mapkeytopath(key:zkey_t; path:Punicode_t; maxpathlen:Psize_t; want_volume:longint):longint;cdecl;external libc_nlm name 'fs_mapkeytopath';
function fs_mapkeytopath(key:zkey_t; path:Punicode_t; maxpathlen:Psize_t; want_volume:longbool):longint;cdecl;external libc_nlm name 'fs_mapkeytopath';
function fs_mapkeytopath(key:zkey_t; path:Punicode_t; var maxpathlen:longint; want_volume:longint):longint;cdecl;external libc_nlm name 'fs_mapkeytopath';
function fs_mapkeytopath(key:zkey_t; path:Punicode_t; var maxpathlen:longint; want_volume:longbool):longint;cdecl;external libc_nlm name 'fs_mapkeytopath';

function fs_mapzidtopath(zid:zid_t; volId:Pvolid_t; path:Punicode_t; maxpathlen:Psize_t; want_volume:longint):longint;cdecl;external libc_nlm name 'fs_mapzidtopath';
function fs_mapzidtopath(zid:zid_t; volId:Pvolid_t; path:Punicode_t; maxpathlen:Psize_t; want_volume:longbool):longint;cdecl;external libc_nlm name 'fs_mapzidtopath';
function fs_mapzidtopath(zid:zid_t; var volId:volid_t; path:Punicode_t; var maxpathlen:longint; want_volume:longint):longint;cdecl;external libc_nlm name 'fs_mapzidtopath';
function fs_mapzidtopath(zid:zid_t; var volId:volid_t; path:Punicode_t; var maxpathlen:longint; want_volume:longbool):longint;cdecl;external libc_nlm name 'fs_mapzidtopath';

function fs_read(key:zkey_t; buf:pointer; off:off64_t; len:size_t; bytes:Pssize_t):longint;cdecl;external libc_nlm name 'fs_read';
function fs_read(key:zkey_t; var buf; off:off64_t; len:size_t; var bytes:ssize_t):longint;cdecl;external libc_nlm name 'fs_read';

function fs_write(key:zkey_t; buf:pointer; off:off64_t; len:size_t; bytes:Pssize_t):longint;cdecl;external libc_nlm name 'fs_write';
function fs_write(key:zkey_t; var buf; off:off64_t; len:size_t; var bytes:ssize_t):longint;cdecl;external libc_nlm name 'fs_write';


{============================================================================
 Generic information (NEB and traditional) based on connection slot. This
 has little to do with file system hooks, but in LibC, there is no way to
 get a hold of a connection slot except through file system hook interfaces.}
function fs_getslotinfo(slot:longint;
                        name:Pchar;
                        objectType:PWord;
                        objectId:Pdword;
                        loginTime:Pointer):longint;cdecl;external libc_nlm name 'fs_getslotinfo';
function fs_getslotinfo(slot:longint;
                        name:Pchar;
                        var objectType:word;
                        var objectId:dword;
                        var loginTime):longint;cdecl;external libc_nlm name 'fs_getslotinfo';

// Clib compatible function name:
function GetConnectionInformation (connectionNumber:longint;
                                   objectName      :Pchar;
                                   objectType      :PWORD;
                                   objectID        :Plongint;
                                   loginTime       :pointer):longint;cdecl;external libc_nlm name 'fs_getslotinfo';
function GetConnectionInformation (connectionNumber:longint;
                                   objectName      :Pchar;
                               var objectType      :word;
                               var objectID        :longint;
                               var loginTime):longint;cdecl;external libc_nlm name 'fs_getslotinfo';



{==========================================
 Traditional file system hook prototypes... }

type TCDeclFunc1PtrArgLongint = function (info:pointer):longint; cdecl;

function fst_register(_type:longint; cbFunc:pointer):longint;cdecl;external libc_nlm name 'fst_register';
function fst_unregister(_type:longint; cbFunc:pointer):longint;cdecl;external libc_nlm name 'fst_unregister';
function fst_getvoldir(slot:longint; fileHandle:longint; namespace:longint; volNum:Plongint; dirBase:Plongint):longint;cdecl;external libc_nlm name 'fst_getvoldir';
function fst_getvoldir(slot, fileHandle, namespace:longint; var volNum, dirBase:longint):longint;cdecl;external libc_nlm name 'fst_getvoldir';
function fst_getorignamespace(volNum, dirBase:longint; namespace:Plongint):longint;cdecl;external libc_nlm name 'fst_getorignamespace';
function fst_getorignamespace(volNum, dirBase:longint; var namespace:longint):longint;cdecl;external libc_nlm name 'fst_getorignamespace';
function fst_mapvoldirtopath(volNum,dirBase,namespace:longint; path:Pchar; maxPathLen:longint):longint;cdecl;external libc_nlm name 'fst_mapvoldirtopath';
function fst_mapvoltoname(volNum:longint; name:Pchar):longint;cdecl;external libc_nlm name 'fst_mapvoltoname';
function fst_read(slot,fileHandle:longint; buffer:pointer; offset:off64_t; length:size_t;
           bytes:Plongint):longint;cdecl;external libc_nlm name 'fst_read';
function fst_read(slot,fileHandle:longint; buffer:pointer; offset:off64_t; length:size_t;
           var bytes:longint):longint;cdecl;external libc_nlm name 'fst_read';
function fst_write(slot, fileHandle:longint; buffer:pointer; offset:off64_t; length:size_t;
           bytes:Plongint):longint;cdecl;external libc_nlm name 'fst_write';
function fst_write(slot, fileHandle:longint; buffer:pointer; offset:off64_t; length:size_t;
           var bytes:longint):longint;cdecl;external libc_nlm name 'fst_write';
function fst_size(slot, fileHandle:longint; length:Poff64_t):longint;cdecl;external libc_nlm name 'fst_size';
function fst_flush(slot, fileHandle:longint):longint;cdecl;external libc_nlm name 'fst_flush';


type
   Pfst_info_t = ^fst_info_t;
   fst_info_t = record
        volNum,
        DosBase,
        dirBase,
        namespace,
        datastream : longint;
        flags : dword;
     end;
    Tfst_info = fst_info_t;
    Pfst_info = Pfst_info_t;

function fst_getinfo(slot,fileHandle:longint; info:Pfst_info_t):longint;cdecl;external libc_nlm name 'fst_getinfo';
function fst_getinfo(slot,fileHandle:longint; var info:fst_info_t):longint;cdecl;external libc_nlm name 'fst_getinfo';


// getopt.h

{ values for 'has_arg'...  }
const
  no_argument = 0;
  required_argument = 1;
  optional_argument = 2;

{ definition for getopt_long() and getopt_long_only()...  }

type
   Poption = ^option;
   option = record
        name    : Pchar;
        has_arg : longint;
        flag    : Plongint;
        val     : longint;
     end;

function getopt_long(argc:longint; argv:array of Pchar; optstring:Pchar; longopts:Poption; longindex:Plongint):longint;cdecl;external libc_nlm name 'getopt_long';
function getopt_long_only(argc:longint; argv:array of Pchar; optstring:Pchar; longopts:Poption; longindex:Plongint):longint;cdecl;external libc_nlm name 'getopt_long_only';

// err.h

  const
     GLOB_APPEND = $0001;
     GLOB_DOOFFS = $0002;        { use gl_offs                                }
     GLOB_ERR = $0004;           { return on error                            }
     GLOB_MARK = $0008;          { append / to matching directories           }
     GLOB_NOCHECK = $0010;       { return pattern itself if nothing matches   }
     GLOB_NOSORT = $0020;        { don't sort                                 }
     GLOB_ALTDIRFUNC = $0040;    { use alternately specified directory funcs  }
     GLOB_BRACE = $0080;         { expand braces ala csh                      }
     GLOB_MAGCHAR = $0100;       { pattern had globbing characters            }
     GLOB_NOMAGIC = $0200;       { GLOB_NOCHECK without magic chars (csh)     }
     GLOB_QUOTE = $0400;         { quote special chars with \                 }
     GLOB_TILDE = $0800;         { expand tilde names from the passwd file    }
     GLOB_NOESCAPE = $1000;      { disable backslash escaping                 }
     GLOB_LIMIT = $2000;         { limit pattern match output to ARG_MAX      }
  { error values returned by glob(3)  }
     GLOB_NOSPACE = -(1);        { malloc call failed                         }
     GLOB_ABORTED = -(2);        { unignored error                            }
     GLOB_NOMATCH = -(3);        { no match and GLOB_NOCHECK not set          }
  { function not supported }
     GLOB_NOSYS = -(4);
     GLOB_ABEND = GLOB_ABORTED;

type
   Pglob_t = ^glob_t;
   glob_t = record
        gl_pathc : longint;
        gl_matchc : longint;
        gl_offs : longint;
        gl_flags : longint;
        gl_pathv : ^Pchar;
        gl_errfunc : function (_para1:Pchar; _para2:longint):longint;cdecl;
        gl_closedir : procedure (_para1:pointer);
        gl_readdir : function (_para1:pointer):Pdirent;
        gl_opendir : function (_para1:Pchar):pointer;
        gl_lstat : function (_para1:Pchar; _para2:Pstat):longint;
        gl_stat : function (_para1:Pchar; _para2:Pstat):longint;
     end;

// grp.h

type
   Pgroup = ^group;
   group = record
        gr_name : Pchar;
        gr_passwd : Pchar;
        gr_gid : gid_t;
        gr_spare : gid_t;
        gr_mem : ^Pchar;
     end;

function getgrgid(gid:gid_t):Pgroup;cdecl;external libc_nlm name 'getgrgid';
function getgrnam(name:Pchar):Pgroup;cdecl;external libc_nlm name 'getgrnam';

// guid.h
// iconv.h

type
   Piconv_t = ^iconv_t;
   iconv_t = longint;

function iconv_open(tocode:Pchar; fromcode:Pchar):iconv_t;cdecl;external libc_nlm name 'iconv_open';
function iconv(cd:iconv_t; inbuf:PPchar; inbytesleft:Psize_t; outbuf:PPchar; outbytesleft:Psize_t):size_t;cdecl;external libc_nlm name 'iconv';
function iconv_close(cd:iconv_t):longint;cdecl;external libc_nlm name 'iconv_close';


// inttypes.h

{ printf-style macros for signed and unsigned integers...  }
{ scanf-style macros for signed and unsigned integers...  }


// iso646.h

// nl_types.h

  const
     NL_SETD = 1;
  { 'oflag' value for catopen()...  }
  { base on value of environment variable "LANG"  }
     NL_CAT_DEFAULT = 0;
  { base on LC_MESSAGES in effect  }
     NL_CAT_LOCALE = 1;
{$define _NL_ITEM}

type
   Pnl_item = ^nl_item;
   nl_item = longint;
   Pnl_catd = ^nl_catd;
   nl_catd = longint;

function catclose(catd:nl_catd):longint;cdecl;external libc_nlm name 'catclose';
function catgets(catd:nl_catd; set_id:longint; msg_id:longint; _string:Pchar):Pchar;cdecl;external libc_nlm name 'catgets';
function catopen(name:Pchar; oflag:longint):nl_catd;cdecl;external libc_nlm name 'catopen';

// langinfo.h

  const
     DAY_1 = 1;    // Sunday
     DAY_2 = 2;    // Monday
     DAY_3 = 3;    // Tuesday
     DAY_4 = 4;    // Wednesday
     DAY_5 = 5;    // Thursday
     DAY_6 = 6;    // Friday
     DAY_7 = 7;    // Saturday
     ABDAY_1 = 8;  // Sun
     ABDAY_2 = 9;  // Mon
     ABDAY_3 = 10; // Tue
     ABDAY_4 = 11; // Wed
     ABDAY_5 = 12; // Thu
     ABDAY_6 = 13; // Fri
     ABDAY_7 = 14; // Sat
     MON_1 = 15;   // January
     MON_2 = 16;   // February
     MON_3 = 17;   // March
     MON_4 = 18;   // April
     MON_5 = 19;   // May
     MON_6 = 20;   // June
     MON_7 = 21;   // July
     MON_8 = 22;   // August
     MON_9 = 23;   // September
     MON_10 = 24;  // October
     MON_11 = 25;  // November
     MON_12 = 26;  // December
     ABMON_1 = 27; // Jan
     ABMON_2 = 28; // Feb
     ABMON_3 = 29;     // Mar
     ABMON_4 = 30;     // Apr
     ABMON_5 = 31;     // May
     ABMON_6 = 32;     // Jun
     ABMON_7 = 33;     // Jul
     ABMON_8 = 34;     // Aug
     ABMON_9 = 35;     // Sep
     ABMON_10 = 36;    // Oct
     ABMON_11 = 37;    // Nov
     ABMON_12 = 38;    // Dec
     RADIXCHAR = 39;   // radix character (not supported)
     THOUSEP = 40;     // separator for thousand
     CRNCYSTR = 43;    // currency symbol
     D_T_FMT = 44;     // string for formatting date and time
     D_FMT = 45;       // date format
     T_FMT = 46;       // time format
     AM_STR = 47;      // am string
     PM_STR = 48;      // pm string
     CODESET = 49;     // code set name
     T_FMT_AMPM = 50;  // a.m. or p.m. time format string (not supported)
     ERA = 51;         //era description segments (not supported)
     ERA_D_FMT = 52;   // era date format string (not supported)
     ERA_D_T_FMT = 53; // era date and time format string (not supported)
     ERA_T_FMT = 54;   // era time format string (not supported)
     ALT_DIGITS = 55;  // alternative symbols for digits  (not supported)
     _MAXSTRMSG = 57;  // maximum number of strings in langinfo


function nl_langinfo(item:nl_item):Pchar;cdecl;external libc_nlm name 'nl_langinfo';


// libgen.h

function basename(path:Pchar):Pchar;cdecl;external libc_nlm name 'basename';
function dirname(path:Pchar):Pchar;cdecl;external libc_nlm name 'dirname';


// library.h

{ return flags for get_app_type()...  }
  const
     LIBRARY_UNKNOWN = $01;
     LIBRARY_LIBC    = $02;  { thread has specific NKS/LibC context    }
     LIBRARY_CLIB    = $04;  { thread has CLib context                 }
     LIBRARY_JAVA    = $08;  { thread belongs to Java Virtual Machine  }

type
   Paddrsp_t = ^addrsp_t;
   addrsp_t = void;
{ O_RDONLY, etc. from fcntl.h...  }

   Predirect_t = ^redirect_t;
   redirect_t = record
        pathname : Pchar;
        oflag : longint;
     end;
{ traditional NetWare solution for libraries...  }

function get_app_data(lib_id:longint):pointer;cdecl;external libc_nlm name 'get_app_data';
function get_app_type:longint;cdecl;external libc_nlm name 'get_app_type';
function register_library(cleanupFunc:TCDeclFunc1PtrArgLongint):longint;cdecl;external libc_nlm name 'register_library';
function register_destructor(libid:longint; cleanupFunc:TCDeclFunc1PtrArgLongint):longint;cdecl;external libc_nlm name 'register_destructor';
function set_app_data(lib_id:longint; data_area:pointer):longint;cdecl;external libc_nlm name 'set_app_data';
function unregister_library(lib_id:longint):longint;cdecl;external libc_nlm name 'unregister_library';
{ more prototypes for library creators, debugging and other uses...  }
function cleardontunloadflag(handle:pointer):longint;cdecl;external libc_nlm name 'cleardontunloadflag';
function findnlmhandle(name:Pchar; space:addrsp_t):TNLMHandle;cdecl;external libc_nlm name 'findnlmhandle';
function getaddressspace:addrsp_t;cdecl;external libc_nlm name 'getaddressspace';
function getaddressspacename(space:addrsp_t; name:Pchar):Pchar;cdecl;external libc_nlm name 'getaddressspacename';
function getallocresourcetag:rtag_t;cdecl;external libc_nlm name 'getallocresourcetag';
function getnativethread:pointer;cdecl;external libc_nlm name 'getnativethread';
{ (current process)  }
function getnlmhandle:TNLMHandle;cdecl;external libc_nlm name 'getnlmhandle';
function getnlmhandlefromthread(thread:pointer):TNLMHandle;cdecl;external libc_nlm name 'getnlmhandlefromthread';
function getnlmname(handle:TNLMHandle; name:Pchar):Pchar;cdecl;external libc_nlm name 'getnlmname';
function getnlmloadpath(loadpath:Pchar):Pchar;cdecl;external libc_nlm name 'getnlmloadpath';
function getthreadname(threadid:pointer; name:Pchar; maxlen:size_t):longint;cdecl;external libc_nlm name 'getthreadname';
function _getthreadid:pointer;cdecl;external libc_nlm name 'getthreadid';
function library_calloc(handle:pointer; size:size_t; count:size_t):pointer;cdecl;external libc_nlm name 'library_calloc';
procedure library_free(addr:pointer);cdecl;external libc_nlm name 'library_free';
function library_malloc(handle:pointer; size:size_t):pointer;cdecl;external libc_nlm name 'library_malloc';
function library_msize(addr:pointer):size_t;cdecl;external libc_nlm name 'library_msize';
function library_realloc(handle:pointer; old:pointer; size:size_t):pointer;cdecl;external libc_nlm name 'library_realloc';
function nlmisloadedprotected:longint;cdecl;external libc_nlm name 'nlmisloadedprotected';
function setdontunloadflag(handle:pointer):longint;cdecl;external libc_nlm name 'setdontunloadflag';
function setthreadname(threadid:pointer; name:Pchar):longint;cdecl;external libc_nlm name 'setthreadname';
//!! function uname2(handle:pointer; info:Putsname; bits:dword):longint;cdecl;external libc_nlm name 'uname2';
function validateaddressrange(addr:pointer; bytes:size_t):longint;cdecl;external libc_nlm name 'validateaddressrange';
function verifynlmhandle(handle:pointer):pointer;cdecl;external libc_nlm name 'verifynlmhandle';
function construct_argc_argv(command_line:Pchar; argv0:Pchar; argc:Plongint; argv:array of Pchar):longint;cdecl;external libc_nlm name 'construct_argc_argv';
type TRedirectSpecs = array [0..2] of redirect_t;
function detect_redirection(r:TRedirectSpecs; argc:Plongint; argv:array of Pchar):longint;cdecl;external libc_nlm name 'detect_redirection';
{ name-logical additions to library_malloc...  }


// locale.h

{ locale categories...  }
{ turn on 1-byte packing...  }

  { for use with LC_ALL                          }

const
   MAX_LOCNAME_LEN = 31 + 1;
  { locale categories...  }
   LC_CTYPE    = 0;  { character classification (unsupported) }
   LC_COLLATE  = 1;  { the locale's collation table (unsupported) }
   LC_NUMERIC  = 2;  { the numeric part of struct lconv }
   LC_MONETARY = 3;  { the monetary part of struct lconv }
   LC_TIME     = 4;  { the time and date part of struct lconv }
   LC_MESSAGES = 5;  { new starting in NetWare v4.11 (unsupported) }
   LC_ALL      = 6;



{ for the current locale...                    }
{ internal representations...                  }
{ ibid                                         }
{ as returned from setlocale(LC_ALL, NULL)     }
{ -------------------------- [Numeric Conventions] ---------------------  }
{ decimal point                                }
{ separator for digits left of decimal         }
{ digit grouping size                          }
{ -------------------------- [Monetary Conventions] --------------------  }
{ currency symbol                              }
{ decimal point                                }
{ separator for digits left of decimal         }
{ digit grouping sizes                         }
{ string indicating positive quantities        }
{ string indicating negative quantities        }
{ count of digits right of decimal             }
{ for positive monetary quantities:            }
{   currency symbol precedes quantity          }
{   currency symbol separated by blank         }
{   position of positive symbol                }
{ for negative monetary quantities:            }
{   currency symbol precedes quantity          }
{   currency symbol separated by blank         }
{   position of negative symbol                }
{ (reserved for future use)                    }
{ -------------------------- [International Monetary Conventions] ------  }
{ international currency symbol and separator  }
{ (international) digits right of decimal      }
{ -------------------------- [Time and Date Conventions] ---------------  }
{ always enforce 24-hour display (Boolean)     }
{ hour and seconds separator                   }
{ hour separator when no seconds displayed     }
{ month/day/year separator                     }
{ hours:minutes:seconds format (hh:mm:ss)      }
{ month/day/year format (mm/dd/yyyy)           }
{ weekday, month, day and year format          }
{ delimited string indicating am and pm        }
{ delimited string indicating AM and PM        }
{ delimited string of day names                }
{ delimited string of abbreviated day names    }
{ delimited string of month names              }
{ delimited string of abbreviated month names  }
type
   Plconv = ^lconv;
   lconv = record
        country : longint;//cdecl;
        language : longint;
        name : array[0..7] of char;
        decimal_point : array[0..3] of char;
        thousands_sep : array[0..3] of char;
        grouping : array[0..3] of char;
        currency_symbol : array[0..3] of char;
        mon_decimal_point : array[0..3] of char;
        mon_thousands_sep : array[0..3] of char;
        mon_grouping : array[0..7] of char;
        positive_sign : array[0..3] of char;
        negative_sign : array[0..3] of char;
        frac_digits : char;
        p_cs_precedes : char;
        p_sep_by_space : char;
        p_sign_posn : char;
        n_cs_precedes : char;
        n_sep_by_space : char;
        n_sign_posn : char;
        reserved : char;
        int_curr_symbol : array[0..14] of char;
        int_frac_digits : char;
        always_24 : longint;
        hour_sep : array[0..3] of char;
        hour_sans_sec_sep : array[0..3] of char;
        date_sep : array[0..3] of char;
        time_fmt : array[0..15] of char;
        date_fmt : array[0..15] of char;
        full_date_fmt : array[0..31] of char;
        ampm : array[0..31] of char;
        _AMPM : array[0..31] of char;
        days : array[0..159] of char;
        day_abbrevs : array[0..159] of char;
        months : array[0..159] of char;
        month_abbrevs : array[0..159] of char;
     end;

{ sizeof(struct lconv) == 0x360 (864.)         }


(** unsupported pragma#pragma pack()*)
{ prototypes for functions standard and nonstandard...  }

function localeconv:Plconv;cdecl;external libc_nlm name 'localeconv';
function setlocale(_para1:longint; _para2:Pchar):Pchar;cdecl;external libc_nlm name 'setlocale';
//!! function derivelocale(_para1:Pchar; _para2:Pchar; _para3:array[0..(31 + 1)-1] of char):Pchar;cdecl;external libc_nlm name 'derivelocale';
function setlocale_r(_para1:longint; _para2:Pchar; _para3:Plconv; _para4:Pchar):Pchar;cdecl;external libc_nlm name 'setlocale_r';


// malloc.h
{
** According to ISO/IEC (ANSI) 9899:1990 and 1999, memory allocation and
** management functions are properly the domain of stdlib.h:
** void   *calloc ( size_t, size_t );
** void    free   ( void * );
** void   *malloc ( size_t );
** void   *realloc( void *, size_t );
**
** Non-standard functions from stdlib.h (don't define __STDC__ or these
** disappear):
** void   *alloca        ( size_t );
** size_t  msize         ( void * );
** int     mvalidate     ( void * );
** size_t  stackavail    ( void );
** void   *stackbase     ( void );
** size_t  stackwatermark( void );
 }


// math.h

{ constants for type exception using matherr()       }
{ turn on 1-byte packing...  }

  const
     DOMAIN = 1;
     SING = 2;      { argument singularity }
     OVERFLOW = 3;  { overflow range error }
     UNDERFLOW = 4; { underflow range error  }
     TLOSS = 5;     { total loss of significance }
     PLOSS = 6;     { partial loss of significance  }


{ for C++, __fp_exception; for C, exception  }
type
   Pexception = ^Texception;
   Texception = record
        _type : longint;
        name : Pchar;
        arg1 : double;
        arg2 : double;
        retval : double;
     end;

{ for C++, __COMPLEX; for C, complex  }
   Pcomplex = ^complex;
   complex = record
        real : double;
        imag : double;
     end;

(** unsupported pragma#pragma pack()*)
//var
//  ___nan_float : double;cvar;external;
//  ___huge_float : double;cvar;external;
//  ___huge_double : double;cvar;external;
//  ___huge_long_double : double;cvar;external;

function acos(_para1:double):double;cdecl;external libc_nlm name 'acos';
function asin(_para1:double):double;cdecl;external libc_nlm name 'asin';
function atan(_para1:double):double;cdecl;external libc_nlm name 'atan';
function atan2(_para1:double; _para2:double):double;cdecl;external libc_nlm name 'atan2';
function cbrt(_para1:double):double;cdecl;external libc_nlm name 'cbrt';
function ceil(_para1:double):double;cdecl;external libc_nlm name 'ceil';
function cos(_para1:double):double;cdecl;external libc_nlm name 'cos';
function cosh(_para1:double):double;cdecl;external libc_nlm name 'cosh';
function exp(_para1:double):double;cdecl;external libc_nlm name 'exp';
function fabs(_para1:double):double;cdecl;external libc_nlm name 'fabs';
function floor(_para1:double):double;cdecl;external libc_nlm name 'floor';
function fmod(_para1:double; _para2:double):double;cdecl;external libc_nlm name 'fmod';
function frexp(_para1:double; _para2:Plongint):double;cdecl;external libc_nlm name 'frexp';
function hypot(_para1:double; _para2:double):double;cdecl;external libc_nlm name 'hypot';
function ldexp(_para1:double; _para2:longint):double;cdecl;external libc_nlm name 'ldexp';
function log(_para1:double):double;cdecl;external libc_nlm name 'log';
function log10(_para1:double):double;cdecl;external libc_nlm name 'log10';
function modf(_para1:double; _para2:Pdouble):double;cdecl;external libc_nlm name 'modf';
function pow(_para1:double; _para2:double):double;cdecl;external libc_nlm name 'pow';
function remainder(_para1:double; _para2:double):double;cdecl;external libc_nlm name 'remainder';
function rint(_para1:double):double;cdecl;external libc_nlm name 'rint';
function sin(_para1:double):double;cdecl;external libc_nlm name 'sin';
function sinh(_para1:double):double;cdecl;external libc_nlm name 'sinh';
function sqrt(_para1:double):double;cdecl;external libc_nlm name 'sqrt';
function tan(_para1:double):double;cdecl;external libc_nlm name 'tan';
function tanh(_para1:double):double;cdecl;external libc_nlm name 'tanh';
function cabs(_para1:complex):double;cdecl;external libc_nlm name 'cabs';
function finite(_para1:double):longint;cdecl;external libc_nlm name 'finite';
function j0(_para1:double):double;cdecl;external libc_nlm name 'j0';
function j1(_para1:double):double;cdecl;external libc_nlm name 'j1';
function jn(_para1:longint; _para2:double):double;cdecl;external libc_nlm name 'jn';
function y0(_para1:double):double;cdecl;external libc_nlm name 'y0';
function y1(_para1:double):double;cdecl;external libc_nlm name 'y1';
function yn(_para1:longint; _para2:double):double;cdecl;external libc_nlm name 'yn';
function matherr(_para1:Pexception):longint;cdecl;external libc_nlm name 'matherr';
type TmathErrHandlerFunc = function (_para1:Pexception):longint; cdecl;
function matherr_handler(_para1:TmathErrHandlerFunc):longint;cdecl;external libc_nlm name 'matherr_handler';
function ___fpclassify_f(_para1:double):longint;cdecl;external libc_nlm name '___fpclassify_f';
function ___fpclassify_d(_para1:double):longint;cdecl;external libc_nlm name '___fpclassify_d';
function ___fpclassify_ld(double:longint):longint;cdecl;external libc_nlm name '___fpclassify_ld';
function ___isfinite_f(_para1:double):longint;cdecl;external libc_nlm name '___isfinite_f';
function ___isfinite_d(_para1:double):longint;cdecl;external libc_nlm name '___isfinite_d';
function ___isfinite_ld(double:longint):longint;cdecl;external libc_nlm name '___isfinite_ld';
function ___signbit_f(_para1:double):longint;cdecl;external libc_nlm name '___signbit_f';
function ___signbit_d(_para1:double):longint;cdecl;external libc_nlm name '___signbit_d';
function ___signbit_ld(double:longint):longint;cdecl;external libc_nlm name '___signbit_ld';


// monitor.h
{ turn on 1-byte packing...  }

type
   Pconn_info = ^Tconn_info;
   Tconn_info = record
        codepage : longint;
        spares   : array[0..507] of byte;
     end;

{ additional processor information that may be available...  }
{ (may not equal 'ThreadsOnProcessor' when totalled:)  }
   Pcpu_info = ^Tcpu_info;
   Tcpu_info = record
        which : longint;
        CurrentProcessor : longint;
        ProcessorUtilization : dword;
        ThreadsOnProcessor : dword;
        reserved1 : dword;
        Family : dword;
        Model : dword;
        Stepping : dword;
        Revision : dword;
        FeatureFlags : dword;
        SerialNumber : Tuint64;
        Speed : dword;
        L1CacheSize : dword;
        L2CacheSize : dword;
        L3CacheSize : dword;
        ReadyThreads : dword;
        RunningThreads : dword;
        SuspendedThreads : dword;
        reserved2 : dword;
        ThreadCPUTime : Tuint64;
        reserved3 : Tuint64;
        reserved4 : array[0..7] of dword;
     end;

   Pfilesystem_info = ^Tfilesystem_info;
   Tfilesystem_info = record
        OpenFileCount : longint;
        CurrentDiskRequests : longint;
        reserved : array[0..61] of dword;
     end;

   Prestag_info = ^Trestag_info;
   Trestag_info = record                      { per-NLM resource-allocation information  }
        tag_count : longint;                  { count of discrete resource tags  }
        res_count : size_t;                   { total number of resources across tags  }
        reserved1 : longint;                  { used only by 'ALRT' tags  }
        signature : dword;                    { type of resource (see netware.h)  }
        description : array[0..79] of char;   { resource tag description string  }
     end;

   Pmem_restag_info = ^Tmem_restag_info;
   Tmem_restag_info = record                  { per-NLM memory information  }
        tag_count   : longint;                { discrete memory allocation resource tags  }
        total_bytes : size_t;                 { total number of bytes allocated across tags  }
        allocations : longint;                { total actual calls to allocator  }
        reserved2   : dword;                  { always signature ('ALRT')  }
        description : array[0..79] of char;   { resource tag description string  }
     end;

   Pmemory_info = ^Tmemory_info;
   Tmemory_info = record
        AllocatedMemoryPool    : size64_t;
        CacheBufferSize        : size64_t;
        CacheBufferMemory      : size64_t;
        CacheMoveableMemory    : size64_t;
        CacheNonmoveableMemory : size64_t;
        CodeMemory             : size64_t;
        DataMemory             : size64_t;
        TotalWorkMemory        : size64_t;
        OtherCachePagesMemory  : size64_t;
        reserved1              : size64_t;
        TotalKnownSystemMemoryUnder4Gb : size64_t;
        TotalKnownSystemMemory : size64_t;
        reserved               : array[0..11] of dword;
     end;

   Pvmemory_info = ^Tvmemory_info;
   Tvmemory_info = record
        PageInCount : Tuint64;
        PageOutCount : Tuint64;
        SwapResvCount : Tuint64;
        SwapPageCount : Tuint64;
        SwapFreeCount : Tuint64;
        PageFaultCount : Tuint64;
        freeCachePages : Tuint64;
        freeCleanPages : Tuint64;
        freeDirtyPages : Tuint64;
        VMPhysicalPageCount : Tuint64;
        reserved : array[0..19] of dword;
     end;

   Pnet_info = ^Tnet_info;
   Tnet_info = record
        MaximumConnections : longint;
        spare0 : array[0..11] of dword;
        IPXAddr : array[0..5] of byte;
        spare1 : array[0..1] of byte;
        MaximumBoards : dword;
        spare3 : array[0..11] of dword;
        IPAddrsBound : array[0..11] of dword;
        spare4 : array[0..23] of dword;
     end;

   Pos_info = ^Tos_info;
   Tos_info = record
        AbendedProcessCount    : longint;
        CurrentServerProcesses : longint;
        reserved               : array[0..61] of dword;
     end;

{ includes namespace list  }
   Pvolume_info = ^Tvolume_info;
   Tvolume_info = record
        which : longint;
        flags : dword;
        name : array[0..(31 + 1)-1] of char;
        SectorSize : dword;
        SectorsPerCluster : dword;
        VolumeSizeInClusters : dword;
        FreedClusters : dword;
        SubAllocFreeableClusters : dword;
        FreeableLimboSectors : dword;
        NonFreeableLimboSectors : dword;
        NonFreeableAvailableSubAllocSectors : dword;
        NotUsableSubAllocSectors : dword;
        SubAllocClusters : dword;
        DataStreamsCount : dword;
        LimboDataStreamsCount : dword;
        OldestDeletedFileAgeInTicks : dword;
        CompressedDataStreamsCount : dword;
        CompressedLimboDataStreamsCount : dword;
        UnCompressableDataStreamsCount : dword;
        PreCompressedSectors : dword;
        CompressedSectors : dword;
        MigratedFiles : dword;
        MigratedSectors : dword;
        ClustersUsedByFAT : dword;
        ClustersUsedByDirectories : dword;
        ClustersUsedByExtendedDirectories : dword;
        TotalDirectoryEntries : dword;
        UnUsedDirectoryEntries : dword;
        TotalExtendedDirectoryExtants : dword;
        UnUsedExtendedDirectoryExtants : dword;
        ExtendedAttributesDefined : dword;
        ExtendedAttributeExtantsUsed : dword;
        DirectoryServicesObjectID : dword;
        VolumeLastModifiedDateAndTime : dword;
        mounted : longint;
        BlockCount : Tuint64;
        BlocksFree : Tuint64;
        BlockSize : dword;
        reserved : array[0..57] of dword;
     end;

(** unsupported pragma#pragma pack()*)
{ prototypes...  }

function netware_conn_info(info:Pconn_info; sequence:Plongint):longint;cdecl;external libc_nlm name 'netware_conn_info';
function netware_conn_info_from_slot(info:Pconn_info; slot:longint):longint;cdecl;external libc_nlm name 'netware_conn_info_from_slot';
function netware_cpu_info(info:Pcpu_info; sequence:Plongint):longint;cdecl;external libc_nlm name 'netware_cpu_info';
function netware_fs_info(info:Pfilesystem_info):longint;cdecl;external libc_nlm name 'netware_fs_info';
function netware_net_info(info:Pnet_info):longint;cdecl;external libc_nlm name 'netware_net_info';
//!! function netware_net_macaddr(board:longint; macAddr:array[0..5] of byte):longint;cdecl;external libc_nlm name 'netware_net_macaddr';
function netware_mem_info(info:Pmemory_info):longint;cdecl;external libc_nlm name 'netware_mem_info';
function netware_mem_info_for_nlm(info:Pmem_restag_info; handle:pointer):longint;cdecl;external libc_nlm name 'netware_mem_info_for_nlm';
function netware_os_info(info:Pos_info):longint;cdecl;external libc_nlm name 'netware_os_info';
function netware_vmem_info(info:Pvmemory_info):longint;cdecl;external libc_nlm name 'netware_vmem_info';
function netware_vol_info(info:Pvolume_info; sequence:Plongint):longint;cdecl;external libc_nlm name 'netware_vol_info';
function netware_vol_info_from_number(info:Pvolume_info; volNum:longint):longint;cdecl;external libc_nlm name 'netware_vol_info_from_number';
function netware_vol_info_from_name(info:Pvolume_info; name:Pchar):longint;cdecl;external libc_nlm name 'netware_vol_info_from_name';
function netware_restag_info_for_nlm(info:Prestag_info; handle:pointer; signature:dword; which:longint):longint;cdecl;external libc_nlm name 'netware_restag_info_for_nlm';

function netware_conn_info(var info:Tconn_info; var sequence:longint):longint;cdecl;external libc_nlm name 'netware_conn_info';
function netware_conn_info_from_slot(var info:Tconn_info; slot:longint):longint;cdecl;external libc_nlm name 'netware_conn_info_from_slot';
function netware_cpu_info(var info:Tcpu_info; var sequence:longint):longint;cdecl;external libc_nlm name 'netware_cpu_info';
function netware_fs_info(var info:Tfilesystem_info):longint;cdecl;external libc_nlm name 'netware_fs_info';
function netware_net_info(var info:Tnet_info):longint;cdecl;external libc_nlm name 'netware_net_info';
//!! function netware_net_macaddr(board:longint; macAddr:array[0..5] of byte):longint;cdecl;external libc_nlm name 'netware_net_macaddr';
function netware_mem_info(var info:Tmemory_info):longint;cdecl;external libc_nlm name 'netware_mem_info';
function netware_mem_info_for_nlm(var info:Tmem_restag_info; handle:pointer):longint;cdecl;external libc_nlm name 'netware_mem_info_for_nlm';
function netware_os_info(var info:Tos_info):longint;cdecl;external libc_nlm name 'netware_os_info';
function netware_vmem_info(var info:Tvmemory_info):longint;cdecl;external libc_nlm name 'netware_vmem_info';
function netware_vol_info(var info:Tvolume_info; var sequence:longint):longint;cdecl;external libc_nlm name 'netware_vol_info';
function netware_vol_info_from_number(var info:Tvolume_info; volNum:longint):longint;cdecl;external libc_nlm name 'netware_vol_info_from_number';
function netware_vol_info_from_name(var info:Tvolume_info; name:Pchar):longint;cdecl;external libc_nlm name 'netware_vol_info_from_name';
function netware_restag_info_for_nlm(var info:Trestag_info; handle:pointer; signature:dword; which:longint):longint;cdecl;external libc_nlm name 'netware_restag_info_for_nlm';


// ncpx.h

const
     MAX_NCPX_NAMELEN = 33;
     NCPX_BEGIN_SCAN = $FFFFFFFF;
     NCPX_REPLY_IS_FRAGGED = $FFFFFFFF;
     NCPX_BEING_RESTARTED = $01101001;
     NCPX_BEING_KILLED = $02202002;
     NCPX_BEING_LOGGED_OUT = $03303003;
     NCPX_BEING_FREED = $04404004;

type

   Pncpx_id_t = ^ncpx_id_t;
   ncpx_id_t = dword;

   Pncpx_client_t = ^ncpx_client_t;
   ncpx_client_t = Tuint64;

   Pncpx_frag_element_t = ^ncpx_frag_element_t;
   ncpx_frag_element_t = record
        addr : pointer;
        size : size_t;
     end;

   Pncpx_msgfrag_t = ^ncpx_msgfrag_t;
   ncpx_msgfrag_t = record
        totalMessageSize : size_t;
        fragCount : longint;
        fragList : array[0..3] of ncpx_frag_element_t;
     end;

   Pncpx_vers_t = ^ncpx_vers_t;
   ncpx_vers_t = record
        major : longint;
        minor : longint;
        revision : longint;
     end;
{ the handler that implements the extended NCP service... }

   ncpx_handler_t = function (client:Pncpx_client_t; request:pointer; requestLen:size_t; reply:pointer; replyLen:Psize_t):longint;cdecl;
{ the call-back made when the session goes away for whatever reason... }

   sess_handler_t = procedure (session:longint; _type:longint);cdecl;
{ the handler that replies to extended NCP requests (if any)... }

   reply_mgr_t = procedure (client:Pncpx_client_t; repBuffer:pointer);cdecl;
{ server registering an extended NCP service...  }
function NcpxRegister(name:Pchar; ncpHandler:ncpx_handler_t; sessionHandler:sess_handler_t; replyManager:reply_mgr_t; version:ncpx_vers_t;
           queryData:Ppointer):longint;cdecl;external libc_nlm name 'NcpxRegister';
function NcpxRegisterWithId(id:ncpx_id_t; name:Pchar; ncpHandler:ncpx_handler_t; sessionHandler:sess_handler_t; replyManager:reply_mgr_t;
           version:ncpx_vers_t; queryData:Ppointer):longint;cdecl;external libc_nlm name 'NcpxRegisterWithId';
function NcpxDeregister(queryData:pointer):longint;cdecl;external libc_nlm name 'NcpxDeregister';
{ client getting information about extended NCP services...  }
function NcpxGetInfoByName(name:Pchar; id:Pncpx_id_t; version:ncpx_vers_t; queryData:pointer):longint;cdecl;external libc_nlm name 'NcpxGetInfoByName';
function NcpxGetInfoById(id:ncpx_id_t; name:Pchar; version:ncpx_vers_t; queryData:pointer):longint;cdecl;external libc_nlm name 'NcpxGetInfoById';
function NcpxScan(id:Pncpx_id_t; name:Pchar; version:ncpx_vers_t; queryData:pointer):longint;cdecl;external libc_nlm name 'NcpxScan';
{ for the client sending extended NCP packets to a service...  }

function NcpxSend(id:ncpx_id_t; request:pointer; requestLen:size_t; reply:pointer; replyLen:Psize_t):longint;cdecl;external libc_nlm name 'NcpxSend';
function NcpxSendFragged(id:ncpx_id_t; reqFrag:Pncpx_msgfrag_t; repFrag:Pncpx_msgfrag_t):longint;cdecl;external libc_nlm name 'NcpxSendFragged';

// ndkvers.h
{==============================================================================
=  This is a timestamp offered by the NDK. Calling libcthreshold() with the
=  defined value as first argument (the second argument returns that of the
=  currently loaded libc.nlm), will ensure at least the functionality and
=  semantics offered by the NDK this file accompanied or this function returns
=  an error (ENOTSUP). Whether or not this function fails, it always returns
=  the loaded library's threshold.
==============================================================================}

  const
     CURRENT_NDK_THRESHOLD = 406230000;
  { timestamps for known releases of LibC on NetWare...  }
     NETWARE_65_FCS = 306250000;        { 25 June 2003  }
     NETWARE_65_SP1 = 310090000;        { 9 October 2003  }
     NETWARE_CSP10  = 310070000;        { 7 October 2003  }
     NETWARE_51_SP7 = NETWARE_CSP10;
     NETWARE_60_SP4 = NETWARE_CSP10;
     NETWARE_CSP11  = 405260000;        { 26 May 2004  }
     NETWARE_60_SP5 = NETWARE_CSP11;
     NETWARE_65_SP2 = NETWARE_CSP11;
     NETWARE_CSP12  = 410310000;       { 31 October 2004 (just a guess)  }
     NETWARE_51_SP8 = NETWARE_CSP12;
     NETWARE_65_SP3 = NETWARE_CSP12;


function libcthreshold(desiredthreshold:longint; libthreshold:Plongint):longint;cdecl;external libc_nlm name 'libcthreshold';
function libcthreshold(desiredthreshold:longint; var libthreshold:longint):longint;cdecl;external libc_nlm name 'libcthreshold';

// netdb.h

  const
     NETDB_INTERNAL = -(1);
     NETDB_SUCCESS = 0;      { no problem  }
     HOST_NOT_FOUND = 1;     { authoritative answer host not found  }
     TRY_AGAIN = 2;          { non authoritative host not found or SERVERFAIL  }
     NO_RECOVERY = 3;        { non recoverable: FORMERR, REFUSED, NOTIMP  }
     NO_DATA = 4;            { valid name, no data record of requested type  }
     NO_ADDRESS = NO_DATA;   { no address, look for MX record  }


{ Addresses are supplied in host order and returned in network order. }
type
   Phostent = ^hostent;
   hostent = record
        h_name      : Pchar;     { official name of host }
        h_aliases   : PPchar;    { alias list }
        h_addrtype  : smallint;  { host address type }
        h_length    : smallint;  { length of address }
        h_addr_list : PPchar;    { list of addresses }
     end;

{ It is assumed here that a network number fits in 32 bits. }
   Pnetent = ^netent;
   netent = record
        n_name     : Pchar;      { official name of net }
        n_aliases  : PPchar;     { alias list }
        n_addrtype : smallint;   { net address type }
        n_net      : u_long;     { network number }
     end;

   Pservent = ^servent;
   servent = record
        s_name    : Pchar;       { official service name }
        s_aliases : PPchar;      { alias list }
        s_port    : smallint;    { port number }
        s_proto   : Pchar;       { protocol to use }
     end;

   Pprotoent = ^protoent;
   protoent = record
        p_name    : Pchar;       { official protocol name }
        p_aliases : PPchar;      { alias list }
        p_proto   : smallint;    { protocol number }
     end;


function gethostbyaddr(_para1:Pchar; _para2:longint; _para3:longint):Phostent;cdecl;external libc_nlm name 'gethostbyaddr';
function gethostbyname(_para1:Pchar):Phostent;cdecl;external libc_nlm name 'gethostbyname';
function gethostname(_para1:Pchar; _para2:longint):longint;cdecl;external libc_nlm name 'gethostname';
function getprotobyname(_para1:Pchar):Pprotoent;cdecl;external libc_nlm name 'getprotobyname';
function getprotobynumber(_para1:longint):Pprotoent;cdecl;external libc_nlm name 'getprotobynumber';
function getservbyname(_para1:Pchar; _para2:Pchar):Pservent;cdecl;external libc_nlm name 'getservbyname';
function getservbyport(_para1:longint; _para2:Pchar):Pservent;cdecl;external libc_nlm name 'getservbyport';
function ___h_errno:Plongint;cdecl;external libc_nlm name '___h_errno';


// nks/nksapi.h
// nlmformat.h

{ 0x002E --------------------------- }
{ 0x0032 --------------------------- }
{ 0x0036 --------------------------- }
{ 0x003A --------------------------- }
{ 0x003E --------------------------- }
{ 0x0042 --------------------------- }
{ 0x0046 --------------------------- }
{ 0x004A --------------------------- }
{ 0x004E --------------------------- }
{ 0x0052 --------------------------- }
{ 0x0056 --------------------------- }
{ 0x005A --------------------------- }
{ 0x005E --------------------------- }
{ 0x0062 --------------------------- }
{ 0x0066 --------------------------- }
{ 0x006A --------------------------- }
{ 0x006E --------------------------- }
{ 0x0072 --------------------------- }
{ 0x0076 --------------------------- }
{ 0x007A --------------------------- }
{ 0x007E --------------------------- }
{ 0x0082 --------------------------- }
{ -------------------------------------------------------------------------
   ** NB: Here begins the variable part of this structure's format; what is
   ** shown here are the maximums. Consequently, sizeof(NLM_HEADER) is almost
   ** always meaningless--the 400 bytes of 'otherData' actually referring to
   ** the original size of the scratch buffer in NLMLINK set up to hold the
   ** data before writing it to the binary file.
   ** -------------------------------------------------------------------------
    }

const
 MAX_DESCRIPTION_LENGTH   = 127;
 OLD_THREAD_NAME_LENGTH   = 5;    // (exactly " LONG")
 MAX_SCREEN_NAME_LENGTH   = 71;
 MAX_THREAD_NAME_LENGTH   = 71;


{ 0x0083 --------------------------- }
{ 0x0103 --------------------------- }
{ 0x0107 --------------------------- }
{ 0x010B --------------------------- }
{ 0x0110 --------------------------- }
{ 0x0111 --------------------------- }
{ 0x0158 --------------------------- }
{ 0x015C --------------------------- }
{ 0x01A3 --------------------------- }
type

   PNLM_HEADER = ^NLM_HEADER;
   NLM_HEADER = record                                       { offset in structure -------------- }
        signature                  : array[0..23] of char;   { "NetWare Loadable Modulex\1A" }
        version                    : dword;               { 0x0018 --------------------------- }
        moduleName                 : array[0..13] of char;   { 0x001C --------------------------- }
        codeImageOffset            : dword;               { 0x002A --------------------------- }
        codeImageSize              : dword;
        dataImageOffset            : dword;
        dataImageSize              : dword;
        uninitializedDataSize      : dword;
        customDataOffset           : dword;
        customDataSize             : dword;
        moduleDependencyOffset     : dword;
        numberOfModuleDependencies : dword;
        relocationFixupOffset      : dword;
        numberOfRelocationFixups   : dword;
        externalReferencesOffset   : dword;
        numberOfExternalReferences : dword;
        publicsOffset              : dword;
        numberOfPublics            : dword;
        debugInfoOffset            : dword;
        numberOfDebugRecords       : dword;
        codeStartOffset            : dword;
        exitProcedureOffset        : dword;
        checkUnloadProcedureOffset : dword;
        moduleType                 : dword;
        flags                      : dword;
        descriptionLength          : byte;
        descriptionText            : array[0..126] of char;
        stackSize                  : dword;
        reserved                   : dword;
        reserved2                  : array[0..4] of byte;
        screenNameLength           : byte;
        screenName                 : array[0..70] of char;
        threadNameLength           : byte;
        threadName                 : array[0..70] of char;
        otherData                  : array[0..399] of byte;
     end;
{ (note: length not actually 0x0333) }
{ starts 'otherData' of NLM_HEADER }
{ offset in structure -------------- }
{ 0x0000 (exactly "VeRsIoN#") }
{ 0x0008 --------------------------- }
{ 0x000C --------------------------- }
{ 0x0010 --------------------------- }
{ 0x0014 --------------------------- }
{ 0x0018 --------------------------- }
{ 0x001A --------------------------- }

   PVERSION_MASK = ^VERSION_MASK;
   VERSION_MASK = record
        VeRsIoN : array[0..7] of char;
        majorVersion : dword;
        minorVersion : dword;
        revision : dword;
        year : dword;
        month : dword;
        day : dword;
     end;                                           { 0x0020 (structure length) }


   PCOPYRIGHT_MASK = ^COPYRIGHT_MASK;               { immediately follows VERSION_MASK }
   COPYRIGHT_MASK = record                          { offset in structure -------------- }
        _CoPyRiGhT      : array[0..9] of char;      { 0x0000 (exactly "CoPyRiGhT=") }
        copyrightLength : byte;                  { 0x000A --------------------------- }
        copyright       : array[0..251] of char;    { 0x000B --------------------------- }
     end;
{ 0x0107 (structure length) }
{ immediately follows COPYRIGHT_MASK }
{ offset in structure -------------- }
{ 0x0000 (exactly "MeSsAgEs") }
{ 0x0008 --------------------------- }
{ 0x000C --------------------------- }
{ 0x0010 --------------------------- }
{ 0x0014 --------------------------- }
{ 0x0018 --------------------------- }
{ 0x001C --------------------------- }
{ 0x0020 --------------------------- }
{ 0x0024 --------------------------- }
{ 0x0028 (ignore to end of structure }
{ 0x0030     ...all these are either }
{ 0x0034     no longer used or never }
{ 0x0038     really have been) }
{ 0x003C --------------------------- }
{ 0x0040 --------------------------- }
{ 0x0044 --------------------------- }
{ 0x0048 --------------------------- }
{ 0x004C --------------------------- }
{ 0x0050 --------------------------- }
{ 0x0054 --------------------------- }
{ 0x0058 --------------------------- }
{ 0x0064 --------------------------- }
{ 0x0068 --------------------------- }
{ 0x006C --------------------------- }
{ 0x0070 --------------------------- }
{ 0x0074 --------------------------- }
{ 0x0078 --------------------------- }
{ 0x007C (* see CODEWARRIOR_LASTMOD) }

   PEXTENDED_HEADER = ^EXTENDED_HEADER;
   EXTENDED_HEADER = record
        MeSsAgEs : array[0..7] of char;
        languageID : dword;
        messageFileOffset : dword;
        messageFileLength : dword;
        messageCount : dword;
        helpFileOffset : dword;
        helpFileLength : dword;
        RPCDataOffset : dword;
        RPCDataLength : dword;
        sharedCodeOffset : dword;
        sharedCodeLength : dword;
        sharedDataOffset : dword;
        sharedDataLength : dword;
        sharedRelocationFixupOffset : dword;
        sharedRelocationFixupLength : dword;
        sharedExternalReferenceOffset : dword;
        sharedExternalReferenceCount : dword;
        sharedPublicsOffset : dword;
        sharedPublicsCount : dword;
        sharedDebugRecordOffset : dword;
        sharedDebugRecordCount : dword;
        sharedInitializationOffset : function :dword;cdecl;
        sharedExitProcedureOffset : procedure ;
        productID : dword;
        reserved0 : dword;
        reserved1 : dword;
        reserved2 : dword;
        reserved3 : dword;
        reserved4 : dword;
        reserved5 : dword;
     end;
{ 0x0080 (structure length) }
{
** The following syntagm appears only in NLMs linked with Metrowerks
** Code Warrior. In the hexadecimal dump, it appears starting at 'reserved5'
** in EXTENDED_HEADER above and appears thus (purely for example):
**
** 1C0: 4C 61 53 74  4D 6F 44 69  20 54 68 75  20 4F 63 74   "LaStMoDi Thu Oct"
** 1D0: 20 20 39 20  31 35 3A 30  33 3A 33 30  20 32 30 30   "  9 19:22:31 200"
** 1E0: 33 0A 00                                             "3..             "
**
** The start point shown (1C0) is variable, but LASTMOD_MASK will appear at
** very nearly this offset.
 }
{ at 'reserved5' in EXTENDED_HEADER }
{ offset in structure }
{ 0x0000 (exactly "LaStMoDi") }
{ 0x0009 --------------------------- }
{ 0x000A --------------------------- }
{ 0x000D --------------------------- }
{ 0x000E --------------------------- }
{ 0x0011 --------------------------- }
{ 0x0012 (blank-padded) ------------ }
{ 0x0014 --------------------------- }
{ 0x0015 --------------------------- }
{ 0x0017 --------------------------- }
{ 0x0018 --------------------------- }
{ 0x001A --------------------------- }
{ 0x001B --------------------------- }
{ 0x001D --------------------------- }
{ 0x001E --------------------------- }
{ 0x0022 --------------------------- }
{ 0x0023 --------------------------- }

   PLASTMOD_MASK = ^LASTMOD_MASK;
   LASTMOD_MASK = record
        LaStMoDi : array[0..7] of char;
        space : char;
        weekday : array[0..2] of char;
        space2 : char;
        month : array[0..2] of char;
        space3 : char;
        day : array[0..1] of char;
        space4 : char;
        hours24 : array[0..1] of char;
        colon1 : char;
        minutes : array[0..1] of char;
        colon2 : char;
        seconds : array[0..1] of char;
        space5 : char;
        year : array[0..3] of char;
        newline : char;
        null : char;
     end;


   PCUSTOM_HEADER_MASK = ^CUSTOM_HEADER_MASK;    { 0x0024 (structure length) }
   CUSTOM_HEADER_MASK = record                   { offset in structure -------------- }
        CuStHeAd : array[0..7] of byte;          { 0x0000 "CuStHeAd" }
     end;                                        { (note: length is custom) }

// nwieeefp.h

type fp_except = longint;
const
  FP_X_INV  = $01;  { invalid operation exception }
  FP_X_DNML = $02;  { denormal operation exception }
  FP_X_DZ   = $04;  { divide by zero exception }
  FP_X_OFL  = $08;  { overflow exception }
  FP_X_UFL  = $10;  { underflow exception }
  FP_X_IMP  = $20;  { inexact (precision) exception  }

type
   Pfp_rnd = ^fp_rnd;
   fp_rnd =  Longint;
Const
   FP_RN = 0;   { round to nearest representable number, tie -> even }
   FP_RM = 1;   { round toward minus infinity }
   FP_RP = 2;   { round toward plus infinity }
   FP_RZ = 3;   { round toward zero (truncate) }

(** unsupported pragma#pragma pack()*)

function fpgetmask:longint;cdecl;external libc_nlm name 'fpgetmask';
function fpgetround:fp_rnd;cdecl;external libc_nlm name 'fpgetround';
function fpgetsticky:longint;cdecl;external libc_nlm name 'fpgetsticky';
function fpsetmask(newmask:longint):longint;cdecl;external libc_nlm name 'fpsetmask';
function fpsetround(newround:fp_rnd):fp_rnd;cdecl;external libc_nlm name 'fpsetround';
function fpsetsticky(newsticky:longint):longint;cdecl;external libc_nlm name 'fpsetsticky';


(** unsupported pragma#pragma pack()*)


// paths.h

const
  _PATH_DEFPATH = '/system';
  _PATH_STDPATH = '/system';
  _PATH_DEVNULL = '/dev/null';
  _PATH_MAILDIR = '/mail';
  _PATH_DEV     = '/system/';
  _PATH_TMP     = '/tmp/';


// proc.h

const
  PROC_DETACHED = $00000001;
  PROC_CURRENT_SPACE = $00000004;    { use current address space    }
  PROC_MEMORY_DEBUG = $00000008;     { same as load -m              }
  PROC_LOAD_SILENT = $00000010;      { no console messages          }
  PROC_INHERIT_CWD = $00000020;      { child start in same CWD      }
  PROC_INHERIT_ID = $00000040;       { child inherits identity      }
  FD_UNUSED  = -1;                   { ignore: do not wire the console }

type
   Pwiring_t = ^wiring_t;    { standard console wiring for process()... }
   wiring_t = record
        infd  : longint;     { new process' standard input, etc. }
        outfd : longint;
        errfd : longint;
     end;
   TWiring = wiring_t;
   PWiring = Pwiring_t;

{$ifndef DisableArrayOfConst}
//function procle(path:Pchar; flags:dword; env:array of Pchar; wiring:Pwiring_t; fds:Pfd_set;
//           appdata:pointer; appdata_size:size_t; reserved:pointer; arg0:Pchar; args:array of const):pid_t;cdecl;external libc_nlm name 'procle';
{$endif}
{function procle(path:Pchar; flags:dword; env:array of Pchar; wiring:Pwiring_t; fds:Pfd_set;
           appdata:pointer; appdata_size:size_t; reserved:pointer; arg0:Pchar):pid_t;cdecl;external libc_nlm name 'procle';
function procve(path:Pchar; flags:dword; env:array of Pchar; wiring:Pwiring_t; fds:Pfd_set;
           appdata:pointer; appdata_size:size_t; reserved:pointer; argv:array of Pchar):pid_t;cdecl;external libc_nlm name 'procve';}
function procve(path:Pchar; flags:dword; env:pointer; wiring:Pwiring_t; fds:Pfd_set;
           appdata:pointer; appdata_size:size_t; reserved:pointer; argv:ppchar):pid_t;cdecl;external libc_nlm name 'procve';
function procle(path:Pchar; flags:dword; env:pointer; wiring:Pwiring_t; fds:Pfd_set;
           appdata:pointer; appdata_size:size_t; reserved:pointer; arg0:Pchar; args:ppchar):pid_t;cdecl;external libc_nlm name 'procle';

// pthread.h
// sched.h
{ turn on 1-byte packing...  }

const
  SCHED_UNKNOWN = 0;
  SCHED_FIFO = 1;         { first-in, first-out      (default NetWare MPK)  }
  SCHED_RR = 2;           { round-robin                    (unimplemented)  }
  SCHED_SPORADIC = 3;     { Single UNIX Specification      (unimplemented)  }
  SCHED_OTHER = 4;        { "other" policy                 (unimplemented)  }

type
   Psched_param = ^sched_param;
   sched_param = record
        sched_priority        : longint;    { for SPORADIC (unimplemented):   }
        sched_ss_low_priority : longint;    { low scheduling priority         }
        sched_ss_repl_period  : Ttimespec;  { replenishment period for        }
        sched_ss_init_budget  : Ttimespec;  { initial budget                  }
        sched_ss_max_repl     : longint;    { maximum pending replenishments  }
        sched_policy          : longint;
     end;

(** unsupported pragma#pragma pack()*)

  const
     PTHREAD_KEYS_MAX = 64;
  { no actual maximum   }
     PTHREAD_THREADS_MAX = 32767;
     PTHREAD_STACK_MIN = 16384;
  { pthread_setdetachstate, etc...  }
     PTHREAD_CREATE_DETACHED = $00000010;
     PTHREAD_CREATE_JOINABLE = 0;
  { values for field 'attr_scope'...  }
     PTHREAD_SCOPE_PROCESS = 0;
  { as yet unsupported  }
     PTHREAD_SCOPE_SYSTEM = 1;
  { values for field 'mattr_flags', 'cattr_flags' and 'rwattr_flags'...  }
     PTHREAD_PROCESS_PRIVATE = 0;
  { as yet unsupported  }
     PTHREAD_PROCESS_SHARED = 1;
  { pthread_setcancelstate, etc...  }
     PTHREAD_CANCELED = -(1);
     PTHREAD_CANCEL_ASYNCHRONOUS = 1;
     PTHREAD_CANCEL_DEFERRED = 2;
     PTHREAD_CANCEL_DISABLE = 0;
     PTHREAD_CANCEL_ENABLE = 1;
  { values for field 'attr_flags'...  }
     PTHREAD_EXPLICIT_SCHED = $0002;
  { as yet unsupported  }
     PTHREAD_INHERIT_SCHED = $0004;
  { values for field 'mattr_protocol'...  }
  { as yet unsupported  }
     PTHREAD_PRIO_PROTECT = -(1);
  { as yet unsupported  }
     PTHREAD_PRIO_INHERIT = 1;
  { as yet unsupported  }
     PTHREAD_PRIO_NONE = 0;
     PTHREAD_ONCE_INIT = 0;
  { values for field 'mattr_kind'...  }
     PTHREAD_MUTEX_NORMAL = $0000;
     PTHREAD_MUTEX_RECURSIVE = $0010;
  { as yet unsupported  }
     PTHREAD_MUTEX_ERRORCHECK = $0020;
  { as yet unsupported  }
     PTHREAD_MUTEX_DEFAULT = $0040;

type
   Ppthread_once_t = ^pthread_once_t;
   pthread_once_t = longint;

   Ppthread_key_t = ^pthread_key_t;
   pthread_key_t = longint;

   Ppthread_t = ^pthread_t;
   pthread_t = pointer; // longint;

{ turn on 1-byte packing...  }

   Ppthread_cond_t = ^pthread_cond_t;
   pthread_cond_t = record
        cond : pointer;
        spares : array[0..5] of longint;
     end;

   Ppthread_mutex_t = PRtlCriticalSection;
   pthread_mutex_t = TRtlCriticalSection;
   TpthreadMutex = TRtlCriticalSection;

   Ppthread_rwlock_t = ^pthread_rwlock_t;
   pthread_rwlock_t = record
        rwlock : pointer;
        reserved : array[0..2] of dword;
     end;

   Ppthread_attr_t = ^pthread_attr_t;
   pthread_attr_t = record
        attr_flags : dword;
        attr_scope : longint;
        attr_priority : longint;
        attr_detachstate : longint;
        attr_stackaddr : pointer;
        attr_stacksize : size_t;
        attr_policy : longint;
        attr_name : array[0..19] of char;
     end;

   Ppthread_condattr_t = ^pthread_condattr_t;
   pthread_condattr_t = record
        cattr_flags : dword;
        cattr_arg : pointer;
        cattr_spare1 : longint;
        cattr_spare2 : longint;
     end;

   Ppthread_mutexattr_t = ^pthread_mutexattr_t;
   pthread_mutexattr_t = record
        mattr_flags : dword;
        mattr_kind : longint;
        mattr_priority : longint;
        mattr_prioceiling : longint;
        mattr_protocol : longint;
        mattr_spares2 : array[0..5] of longint;
        mattr_name : array[0..31] of char;
     end;
   Ppthread_mutex_attr_t = Ppthread_mutexattr_t;
   pthread_mutex_attr_t = pthread_mutexattr_t;
   TMutexAttribute = pthread_mutex_attr_t;

   Ppthread_rwlockattr_t = ^pthread_rwlockattr_t;
   pthread_rwlockattr_t = record
        rwattr_flags : dword;
        rwattr_spare1 : longint;
        rwattr_priority : longint;
        rwattr_prioceiling : longint;
        rwattr_protocol : longint;
        rwattr_spares2 : array[0..5] of longint;
        rwattr_name : array[0..31] of char;
     end;

(** unsupported pragma#pragma pack()*)
{ pthread functions...  }


type TPTThreadStartFunction = function (_para1:pointer):pointer; cdecl;

function pthread_create(thread:Ppthread_t; attr:Ppthread_attr_t; start_routine:TPTThreadStartFunction; arg:pointer):longint;cdecl;external libc_nlm name 'pthread_create';
function pthread_cancel(thread:pthread_t):longint;cdecl;external libc_nlm name 'pthread_cancel';
function pthread_join(thread:pthread_t; status:Ppointer):longint;cdecl;external libc_nlm name 'pthread_join';
procedure pthread_exit(status:pointer);cdecl;external libc_nlm name 'pthread_exit';
function pthread_equal(t1:pthread_t; t2:pthread_t):longint;cdecl;external libc_nlm name 'pthread_equal';
procedure pthread_yield;cdecl;external libc_nlm name 'pthread_yield';
function pthread_kill(thread:pthread_t; sig:longint):longint;cdecl;external libc_nlm name 'pthread_kill';
function pthread_detach(thread:pthread_t):longint;cdecl;external libc_nlm name 'pthread_detach';

function pthread_once(once_control:Ppthread_once_t; init_routine:TCDeclProcedure ):longint;cdecl;external libc_nlm name 'pthread_once';
function pthread_self:pthread_t;cdecl;external libc_nlm name 'pthread_self';
function pthread_key_create(_para1:Ppthread_key_t; _destructor:TCDeclProc1PtrArg):longint;cdecl;external libc_nlm name 'pthread_key_create';
function pthread_key_delete(key:pthread_key_t):longint;cdecl;external libc_nlm name 'pthread_key_delete';
function pthread_getspecific(key:pthread_key_t):pointer;cdecl;external libc_nlm name 'pthread_getspecific';

function pthread_setspecific(key:pthread_key_t; value:pointer):longint;cdecl;external libc_nlm name 'pthread_setspecific';
function pthread_getschedparam(thread:pthread_t; policy:Plongint; param:Psched_param):longint;cdecl;external libc_nlm name 'pthread_getschedparam';

function pthread_setschedparam(thread:pthread_t; policy:longint; param:Psched_param):longint;cdecl;external libc_nlm name 'pthread_setschedparam';
function pthread_setcancelstate(state:longint; oldstate:Plongint):longint;cdecl;external libc_nlm name 'pthread_setcancelstate';
function pthread_setcanceltype(_type:longint; oldtype:Plongint):longint;cdecl;external libc_nlm name 'pthread_setcanceltype';
procedure pthread_testcancel;cdecl;external libc_nlm name 'pthread_testcancel';

function pthread_sigmask(how:longint; _set:Psigset_t; oset:Psigset_t):longint;cdecl;external libc_nlm name 'pthread_sigmask';
procedure pthread_cleanup_push(routine:TCDeclProc1PtrArg; arg:pointer);cdecl;external libc_nlm name 'pthread_cleanup_push';
procedure pthread_cleanup_pop(execute:longint);cdecl;external libc_nlm name 'pthread_cleanup_pop';
{ pthread attribute functions...  }
function pthread_attr_init(attr:Ppthread_attr_t):longint;cdecl;external libc_nlm name 'pthread_attr_init';
function pthread_attr_destroy(attr:Ppthread_attr_t):longint;cdecl;external libc_nlm name 'pthread_attr_destroy';

function pthread_attr_getdetachstate(attr:Ppthread_attr_t; detachstate:Plongint):longint;cdecl;external libc_nlm name 'pthread_attr_getdetachstate';
function pthread_attr_setdetachstate(attr:Ppthread_attr_t; detachstate:longint):longint;cdecl;external libc_nlm name 'pthread_attr_setdetachstate';

function pthread_attr_getinheritsched(attr:Ppthread_attr_t; inheritsched:Plongint):longint;cdecl;external libc_nlm name 'pthread_attr_getinheritsched';
function pthread_attr_setinheritsched(attr:Ppthread_attr_t; inheritsched:longint):longint;cdecl;external libc_nlm name 'pthread_attr_setinheritsched';
function pthread_attr_getschedparam(attr:Ppthread_attr_t; param:Psched_param):longint;cdecl;external libc_nlm name 'pthread_attr_getschedparam';
function pthread_attr_setschedparam(attr:Ppthread_attr_t; param:Psched_param):longint;cdecl;external libc_nlm name 'pthread_attr_setschedparam';
function pthread_attr_getschedpolicy(attr:Ppthread_attr_t; policy:Plongint):longint;cdecl;external libc_nlm name 'pthread_attr_getschedpolicy';
function pthread_attr_setschedpolicy(attr:Ppthread_attr_t; policy:longint):longint;cdecl;external libc_nlm name 'pthread_attr_setschedpolicy';
function pthread_attr_getscope(attr:Ppthread_attr_t; contentionscope:Plongint):longint;cdecl;external libc_nlm name 'pthread_attr_getscope';
function pthread_attr_setscope(attr:Ppthread_attr_t; contentionscope:longint):longint;cdecl;external libc_nlm name 'pthread_attr_setscope';
function pthread_attr_getstackaddr(attr:Ppthread_attr_t; stackaddr:Ppointer):longint;cdecl;external libc_nlm name 'pthread_attr_getstackaddr';
function pthread_attr_setstackaddr(attr:Ppthread_attr_t; stackaddr:pointer):longint;cdecl;external libc_nlm name 'pthread_attr_setstackaddr';
function pthread_attr_getstacksize(attr:Ppthread_attr_t; stacksize:Psize_t):longint;cdecl;external libc_nlm name 'pthread_attr_getstacksize';
function pthread_attr_setstacksize(attr:Ppthread_attr_t; stacksize:size_t):longint;cdecl;external libc_nlm name 'pthread_attr_setstacksize';
function pthread_attr_getname_np(attr:Ppthread_attr_t; name:Pchar; len:size_t; mbz:Ppointer):longint;cdecl;external libc_nlm name 'pthread_attr_getname_np';
function pthread_attr_setname_np(attr:Ppthread_attr_t; name:Pchar; mbz:pointer):longint;cdecl;external libc_nlm name 'pthread_attr_setname_np';
{ condition variable functions...  }

function pthread_cond_init(cond:Ppthread_cond_t; attr:Ppthread_condattr_t):longint;cdecl;external libc_nlm name 'pthread_cond_init';
function pthread_cond_destroy(cond:Ppthread_cond_t):longint;cdecl;external libc_nlm name 'pthread_cond_destroy';
function pthread_cond_signal(cond:Ppthread_cond_t):longint;cdecl;external libc_nlm name 'pthread_cond_signal';
function pthread_cond_broadcast(cond:Ppthread_cond_t):longint;cdecl;external libc_nlm name 'pthread_cond_broadcast';
function pthread_cond_wait(cond:Ppthread_cond_t; mutex:Ppthread_mutex_t):longint;cdecl;external libc_nlm name 'pthread_cond_wait';

function pthread_cond_timedwait(cond:Ppthread_cond_t; mutex:Ppthread_mutex_t; abstime:Ptimespec):longint;cdecl;external libc_nlm name 'pthread_cond_timedwait';
{ condition variable attribute functions...  }
function pthread_condattr_init(attr:Ppthread_condattr_t):longint;cdecl;external libc_nlm name 'pthread_condattr_init';
function pthread_condattr_destroy(attr:Ppthread_condattr_t):longint;cdecl;external libc_nlm name 'pthread_condattr_destroy';
function pthread_condattr_getpshared(attr:Ppthread_condattr_t; pshared:Plongint):longint;cdecl;external libc_nlm name 'pthread_condattr_getpshared';
function pthread_condattr_setpshared(attr:Ppthread_condattr_t; pshared:longint):longint;cdecl;external libc_nlm name 'pthread_condattr_setpshared';
{ mutex functions...  }

function pthread_mutex_init(mutex:Ppthread_mutex_t; attr:Ppthread_mutexattr_t):longint;cdecl;external libc_nlm name 'pthread_mutex_init';
function pthread_mutex_destroy(mutex:Ppthread_mutex_t):longint;cdecl;external libc_nlm name 'pthread_mutex_destroy';
function pthread_mutex_lock(mutex:Ppthread_mutex_t):longint;cdecl;external libc_nlm name 'pthread_mutex_lock';
function pthread_mutex_trylock(mutex:Ppthread_mutex_t):longint;cdecl;external libc_nlm name 'pthread_mutex_trylock';
function pthread_mutex_unlock(mutex:Ppthread_mutex_t):longint;cdecl;external libc_nlm name 'pthread_mutex_unlock';
{ mutex attribute functions...  }
function pthread_mutexattr_init(attr:Ppthread_mutexattr_t):longint;cdecl;external libc_nlm name 'pthread_mutexattr_init';
function pthread_mutexattr_destroy(attr:Ppthread_mutexattr_t):longint;cdecl;external libc_nlm name 'pthread_mutexattr_destroy';
function pthread_mutexattr_getprioceiling(attr:Ppthread_mutexattr_t; prioceiling:Plongint):longint;cdecl;external libc_nlm name 'pthread_mutexattr_getprioceiling';
function pthread_mutexattr_setprioceiling(attr:Ppthread_mutexattr_t; prioceiling:longint):longint;cdecl;external libc_nlm name 'pthread_mutexattr_setprioceiling';
function pthread_mutexattr_getprotocol(attr:Ppthread_mutexattr_t; protocol:Plongint):longint;cdecl;external libc_nlm name 'pthread_mutexattr_getprotocol';
function pthread_mutexattr_setprotocol(attr:Ppthread_mutexattr_t; protocol:longint):longint;cdecl;external libc_nlm name 'pthread_mutexattr_setprotocol';
function pthread_mutexattr_getpshared(attr:Ppthread_mutexattr_t; pshared:Plongint):longint;cdecl;external libc_nlm name 'pthread_mutexattr_getpshared';
function pthread_mutexattr_setpshared(attr:Ppthread_mutexattr_t; pshared:longint):longint;cdecl;external libc_nlm name 'pthread_mutexattr_setpshared';
function pthread_mutexattr_gettype(attr:Ppthread_mutexattr_t; kind:Plongint):longint;cdecl;external libc_nlm name 'pthread_mutexattr_gettype';
function pthread_mutexattr_settype(attr:Ppthread_mutexattr_t; kind:longint):longint;cdecl;external libc_nlm name 'pthread_mutexattr_settype';
{ reader-writer lock functions...  }
function pthread_rwlock_init(rwlp:Ppthread_rwlock_t; attr:Ppthread_rwlockattr_t):longint;cdecl;external libc_nlm name 'pthread_rwlock_init';
function pthread_rwlock_destroy(rwlp:Ppthread_rwlock_t):longint;cdecl;external libc_nlm name 'pthread_rwlock_destroy';
function pthread_rwlock_rdlock(rwlp:Ppthread_rwlock_t):longint;cdecl;external libc_nlm name 'pthread_rwlock_rdlock';
function pthread_rwlock_wrlock(rwlp:Ppthread_rwlock_t):longint;cdecl;external libc_nlm name 'pthread_rwlock_wrlock';
function pthread_rwlock_tryrdlock(rwlp:Ppthread_rwlock_t):longint;cdecl;external libc_nlm name 'pthread_rwlock_tryrdlock';
function pthread_rwlock_trywrlock(rwlp:Ppthread_rwlock_t):longint;cdecl;external libc_nlm name 'pthread_rwlock_trywrlock';
function pthread_rwlock_unlock(rwlp:Ppthread_rwlock_t):longint;cdecl;external libc_nlm name 'pthread_rwlock_unlock';
function pthread_rwlock_timedrdlock(rwlock:Ppthread_rwlock_t; abs_timeout:Ptimespec):longint;cdecl;external libc_nlm name 'pthread_rwlock_timedrdlock';
function pthread_rwlock_timedwrlock(rwlock:Ppthread_rwlock_t; abs_timeout:Ptimespec):longint;cdecl;external libc_nlm name 'pthread_rwlock_timedwrlock';
{ reader-writer lock attribute functions...  }
function pthread_rwlockattr_init(attr:Ppthread_rwlockattr_t):longint;cdecl;external libc_nlm name 'pthread_rwlockattr_init';
function pthread_rwlockattr_destroy(attr:Ppthread_rwlockattr_t):longint;cdecl;external libc_nlm name 'pthread_rwlockattr_destroy';
function pthread_rwlockattr_getpshared(attr:Ppthread_rwlockattr_t; pshared:Plongint):longint;cdecl;external libc_nlm name 'pthread_rwlockattr_getpshared';
function pthread_rwlockattr_setpshared(attr:Ppthread_rwlockattr_t; pshared:longint):longint;cdecl;external libc_nlm name 'pthread_rwlockattr_setpshared';
{ registering functions to execute at call to fork()...  }
function pthread_atfork(prepare, parent, child:TCDeclProcedure):longint;cdecl;external libc_nlm name 'pthread_atfork';


type
   Ppasswd = ^passwd;
   passwd = record
        pw_uid     : uid_t;     // user id
        pw_spare1  : uid_t;
        pw_gid     : gid_t;     // group id
        pw_spare2  : gid_t;
        pw_name    : Pchar;     // username
        pw_dir     : Pchar;     // home directory
        pw_shell   : Pchar;     // default shell
        pw_LDAPName: Pchar;     // real name
        pw_passwd  : Pchar;     // password (always nil)
        pw_gecos   : Pchar;     // general information
        pw_comment : Pchar;     // commend
        pw_change  : time_t;    // password change time
        pw_expire  : time_t;    // account expiration
        spare1 : array[0..2] of pointer;
        spare2 : array[0..3] of pointer;
     end;


//!! function geteuid:uid_t;cdecl;external libc_nlm name 'geteuid';
//!! function getuid:uid_t;cdecl;external libc_nlm name 'getuid';
function posixlogin(host:Pchar; port:longint; name:Pchar; pwd:Pchar; ctx:Pchar):longint;cdecl;external libc_nlm name 'posixlogin';
function posixlogout:longint;cdecl;external libc_nlm name 'posixlogout';

{$ifdef EnableLibcRegex}


// regex.h
type
   Pregoff_t = ^regoff_t;
   regoff_t = off_t;

   Preg_syntax_t = ^reg_syntax_t;
   reg_syntax_t = dword;

   Ps_reg_t = ^s_reg_t;
   s_reg_t = longint;

   Pactive_reg_t = ^active_reg_t;
   active_reg_t = dword;
{ number of parenthesized subexpressions  }

{ end pointer for REG_PEND                }
{ not visible                             }

   Pregex_t = ^regex_t;
   regex_t = record
        re_magic : longint;
        re_nsub : size_t;
        re_endp : Pchar;
        re_g : Pre_guts;
     end;

  const
     REG_BASIC = 0000;
     REG_EXTENDED = 0001;
     REG_ICASE = 0002;
     REG_NOSUB = 0004;
     REG_NEWLINE = 0010;
     REG_NOSPEC = 0020;
     REG_PEND = 0040;
     REG_DUMP = 0200;
  { regerror() flags  }
     REG_NOMATCH = 1;
     REG_BADPAT = 2;
     REG_ECOLLATE = 3;
     REG_ECTYPE = 4;
     REG_EESCAPE = 5;
     REG_ESUBREG = 6;
     REG_EBRACK = 7;
     REG_EPAREN = 8;
     REG_EBRACE = 9;
     REG_BADBR = 10;
     REG_ERANGE = 11;
     REG_ESPACE = 12;
     REG_BADRPT = 13;
     REG_EMPTY = 14;
     REG_ASSERT = 15;
     REG_INVARG = 16;
  { convert name to number (!)  }
     REG_ATOI = 255;
  { convert number to name (!)  }
     REG_ITOA = 0400;
  { regexec() flags  }
     REG_NOTBOL = 00001;
     REG_NOTEOL = 00002;
     REG_STARTEND = 00004;
  { tracing of execution        }
     REG_TRACE = 00400;
  { force large representation  }
     REG_LARGE = 01000;
  { force use of backref code   }
     REG_BACKR = 02000;


   Pregmatch_t = ^regmatch_t;
   regmatch_t = record
        rm_so : regoff_t;
        rm_eo : regoff_t;
     end;
{ regcomp() flags...  }
{ regerror() flags  }
{ regexec() flags  }

{$endif EnableLibcRegex}


// ringx.h

const PAGE_SIZE = 4096;


type RxCleanup_t = procedure (addr:pointer);cdecl;
{ preliminary (registration and clean-up)...  }

function RxIdentifyCode(startFuncAddr:pointer; endFuncAddrPlusOne:pointer; marshallingCodeReference:Plongint):longint;cdecl;external system_nlm name 'RxIdentifyCode';
function RxUnidentifyCode(marshallingCodeReference:longint):longint;cdecl;external system_nlm name 'RxUnidentifyCode';
function RxRegisterSysCall(marshalledFuncAddr:pointer; unmarshalledName:Pchar; argCount:longint):longint;cdecl;external system_nlm name 'RxRegisterSysCall';
function RxUnregisterSysCall(unmarshalledName:Pchar):longint;cdecl;external system_nlm name 'RxUnregisterSysCall';
{ mundane calls...  }
function RxLockMemory(addr:pointer; length:size_t):longint;cdecl;external system_nlm name 'RxLockMemory';
function RxUnlockMemory(addr:pointer; length:size_t):longint;cdecl;external system_nlm name 'RxUnlockMemory';
{ to handle address space fault and threads caught in kernel...  }
function RxRegisterKernelResource(_para1:pointer; _para2:RxCleanup_t):longint;cdecl;external system_nlm name 'RxRegisterKernelResource';
function RxRegisterThreadResource(_para1:pointer; _para2:RxCleanup_t):longint;cdecl;external system_nlm name 'RxRegisterThreadResource';
function RxUnregisterKernelResource(_para1:pointer):longint;cdecl;external system_nlm name 'RxUnregisterKernelResource';
function RxUnregisterThreadResource(_para1:pointer):longint;cdecl;external system_nlm name 'RxUnregisterThreadResource';
{ data; referenced to avoid compiler optimization of code including this  }
//??  var
//??     RxTmp : longint;cvar;public;
{ main working macros (buffers and structures, char and wide strings...  }


// semaphore.h

const
  SEM_VALUE_MAX = $7FFFFFFF;
// #define SEM_FAILED      ((sem_t *) 0)

type
   Psem_t = ^sem_t;
   sem_t = record
        sema : pointer;
        spares : array[0..5] of longint;
     end;

function sem_init(sem:Psem_t; pshared:longint; value:dword):longint;cdecl;external libc_nlm name 'sem_init';
function sem_destroy(sem:Psem_t):longint;cdecl;external libc_nlm name 'sem_destroy';
function sem_getvalue(sem:Psem_t; sval:Plongint):longint;cdecl;external libc_nlm name 'sem_getvalue';
function sem_post(sem:Psem_t):longint;cdecl;external libc_nlm name 'sem_post';
function sem_wait(sem:Psem_t):longint;cdecl;external libc_nlm name 'sem_wait';
function sem_trywait(sem:Psem_t):longint;cdecl;external libc_nlm name 'sem_trywait';
function sem_timedwait(sem:Psem_t; abstime:Ptimespec):longint;cdecl;external libc_nlm name 'sem_timedwait';


// setjmp.h

type
   _Pjmp_buf = ^_jmp_buf;
   _jmp_buf = double;

procedure __longjmp(_para1:_jmp_buf; _para2:longint);cdecl;external libc_nlm name '__longjmp';
function __setjmp(_para1:_jmp_buf):longint;cdecl;external libc_nlm name '__setjmp';

// synch.h

const
  USYNC_THREAD    = 0;
  USYNC_PROCESS   = 1;  { shared between processes (unsupported)}
  USYNC_DESTROYED = 2;  { deallocated object }

type
   //!! timespec = timestruc_t;
   Pbarrier = ^barrier;
   barrier = record
        reserved : array[0..7] of longint;
     end;
   barrier_t = barrier;
   Pbarrier_t = ^barrier_t;

   Pcond_t = ^cond_t;
   cond_t = pthread_cond_t;

   Pmutex_t = ^mutex_t;
   mutex_t = pthread_mutex_t;

   Prwlock_t = ^rwlock_t;
   rwlock_t = pthread_rwlock_t;

   Psema_t = ^sema_t;
   sema_t = sem_t;

function barrier_init(bp:Pbarrier_t; threads:longint):longint;cdecl;external libc_nlm name 'barrier_init';
procedure barrier_destroy(bp:Pbarrier_t);cdecl;external libc_nlm name 'barrier_destroy';
function barrier_wait(bp:Pbarrier_t):longint;cdecl;external libc_nlm name 'barrier_wait';
function barrier_inc(bp:Pbarrier_t):longint;cdecl;external libc_nlm name 'barrier_inc';
function barrier_dec(bp:Pbarrier_t):longint;cdecl;external libc_nlm name 'barrier_dec';
function cond_init(cvp:Pcond_t; _type:longint; arg:longint):longint;cdecl;external libc_nlm name 'cond_init';
function cond_destroy(cvp:Pcond_t):longint;cdecl;external libc_nlm name 'cond_destroy';
function cond_broadcast(cvp:Pcond_t):longint;cdecl;external libc_nlm name 'cond_broadcast';
function cond_signal(cvp:Pcond_t):longint;cdecl;external libc_nlm name 'cond_signal';
function cond_wait(cvp:Pcond_t; mp:Pmutex_t):longint;cdecl;external libc_nlm name 'cond_wait';
//!! function cond_timedwait(cvp:Pcond_t; mp:Pmutex_t; abstime:Ptimestruc_t):longint;cdecl;external libc_nlm name 'cond_timedwait';
function mutex_init(mp:Pmutex_t; _type:longint; arg:pointer):longint;cdecl;external libc_nlm name 'mutex_init';
function mutex_destroy(mp:Pmutex_t):longint;cdecl;external libc_nlm name 'mutex_destroy';
function mutex_lock(mp:Pmutex_t):longint;cdecl;external libc_nlm name 'mutex_lock';
function mutex_trylock(mp:Pmutex_t):longint;cdecl;external libc_nlm name 'mutex_trylock';
function mutex_unlock(mp:Pmutex_t):longint;cdecl;external libc_nlm name 'mutex_unlock';
function rwlock_init(rwlp:Prwlock_t; _type:longint; arg:pointer):longint;cdecl;external libc_nlm name 'rwlock_init';
function rwlock_destroy(rwlp:Prwlock_t):longint;cdecl;external libc_nlm name 'rwlock_destroy';
function rw_rdlock(rwlp:Prwlock_t):longint;cdecl;external libc_nlm name 'rw_rdlock';
function rw_wrlock(rwlp:Prwlock_t):longint;cdecl;external libc_nlm name 'rw_wrlock';
function rw_tryrdlock(rwlp:Prwlock_t):longint;cdecl;external libc_nlm name 'rw_tryrdlock';
function rw_trywrlock(rwlp:Prwlock_t):longint;cdecl;external libc_nlm name 'rw_trywrlock';
function rw_unlock(rwlp:Prwlock_t):longint;cdecl;external libc_nlm name 'rw_unlock';
function sema_init(sp:Psema_t; count:dword; _type:longint; arg:pointer):longint;cdecl;external libc_nlm name 'sema_init';
function sema_destroy(sp:Psema_t):longint;cdecl;external libc_nlm name 'sema_destroy';
function sema_post(sp:Psema_t):longint;cdecl;external libc_nlm name 'sema_post';
function sema_trywait(sp:Psema_t):longint;cdecl;external libc_nlm name 'sema_trywait';
function sema_wait(sp:Psema_t):longint;cdecl;external libc_nlm name 'sema_wait';

// syslog.h

{ log options for openlog()...  }
{ message facilities for openlog()...  }
{ for constructing 'maskpri' for setlogmask()...  }
{ values for priority argument of syslog()...  }
{ with no filename argument, syslogd.nlm uses this configuration file:  }
// termio.h
// termios.h
{
** Note: This is a very primitive and narrow adaptation of POSIX termios.h.
** Most of what is in this file is for completeness and to avoid locking
** termios.h interfaces off from advancing progressively as needed.}


  const
     NCCS = 32;
  { ^C  }
     INTR = $03;
  { ^D  }
     QUIT = $04;
  { ^Z  }
     SUSP = $1A;
  { currently unsupported                          }
     ECHO = $01;
  { currently unsupported                          }
     ECHOE = $02;
  { currently unsupported                          }
     ECHOK = $04;
  { currently unsupported                          }
     ECHONL = $08;
  { currently unsupported                          }
     ICANON = $10;
  { currently unsupported                          }
     IEXTEN = $20;
  { check against INTR, QUIT and SUSP (default)    }
     ISIG = $40;
  { currently unsupported                          }
     NOFLSH = $80;
  { c_iflag bits...  }
     IGNBRK = 0000001;
     BRKINT = 0000002;
     IGNPAR = 0000004;
     PARMRK = 0000010;
     INPCK = 0000020;
     ISTRIP = 0000040;
     INLCR = 0000100;
     IGNCR = 0000200;
     ICRNL = 0000400;
     IUCLC = 0001000;
     IXON = 0002000;
     IXANY = 0004000;
     IXOFF = 0010000;
     IMAXBEL = 0020000;
     CSIZE = 0000060;
     CS5 = 0000000;
     CS6 = 0000020;
     CS7 = 0000040;
     CS8 = 0000060;
     CSTOPB = 0000100;
     CREAD = 0000200;
     PARENB = 0000400;
     PARODD = 0001000;
     HUPCL = 0002000;
     CLOCAL = 0004000;
  { 'c_cc' control characters...  }
  { ^C                                             }
     VINTR = 1;
  { ^\   (unsupported)                             }
     VQUIT = 2;
  { del  (unsupported)                             }
     VERASE = 3;
  { ^D                                             }
     VEOF = 4;
  { @    (unsupported)                             }
     VKILL = 5;
  { currently unsupported                          }
     VTIME = 6;
  { currently unsupported                          }
     VMIN = 7;
  { ^q   (unsupported)                             }
     VSTART = 8;
  { ^s   (unsupported)                             }
     VSTOP = 9;
  { ^z                                             }
     VSUSP = 10;
  { '\0' (unsupported)                             }
     VEOL = 11;
  { 'optional_actions' causes action to be effectuated when:  }
  { immediately                                    }
     TCSANOW = 1;
  { output done                     (unsupported)  }
     TCSADRAIN = 2;
  { output done and input discarded (unsupported)  }
     TCSAFLUSH = 3;


{ c_iflag bits...  }
{ 'c_cc' control characters...  }
{ 'optional_actions' causes action to be effectuated when:  }
type
   Ptcflag_t = ^tcflag_t;
   tcflag_t = dword;

   Pcc_t = ^cc_t;
   cc_t = byte;

   Pspeed_t = ^speed_t;
   speed_t =  Longint;
   Const
     B0 = 0;
     B50 = 50;
     B75 = 75;
     B110 = 110;
     B134 = 134;
     B150 = 150;
     B200 = 200;
     B300 = 300;
     B600 = 600;
     B1200 = 1200;
     B1800 = 1800;
     B2400 = 2400;
     B4800 = 4800;
     B9600 = 9600;
     B19200 = 19200;
     B38400 = 38400;

{ ioctl() control packet...  }
{ input modes     --currently unused          }
{ output modes    --currently unused          }
{ control modes   --currently unused          }
{ local modes     --ISIG currently supported  }
{ input speed     --currently unused          }
{ output speed    --currently unused          }
{ line discipline --currently unused          }
{ control chars   --currently used            }
type
   Ptermios = ^termios;
   termios = record
        c_iflag : tcflag_t;
        c_oflag : tcflag_t;
        c_cflag : tcflag_t;
        c_lflag : tcflag_t;
        c_ispeed : speed_t;
        c_ospeed : speed_t;
        c_line : cc_t;
        c_spare1 : dword;
        c_cc : array[0..31] of cc_t;
        c_spare2 : dword;
        c_spare3 : dword;
        c_spare4 : dword;
        c_spare5 : dword;
     end;

{ POSIX-defined functions...  }

function tcgetattr(fildes:longint; tp:Ptermios):longint;cdecl;external libc_nlm name 'tcgetattr';
function tcsetattr(fildes:longint; optional_actions:longint; tp:Ptermios):longint;cdecl;external libc_nlm name 'tcsetattr';

// tgmath.h

{ syslog.h
  ==============================================================================}
  { log options for openlog()...  }
  { log the process ID with each message  }

  const
     LOG_PID = $00000001;
     LOG_CONS = $00000002;                   { log to the system console on error    }
     LOG_NDELAY = $00000004;                 { connect to syslog daemon immediately  }
     LOG_ODELAY = $00000008;                 { delay open until syslog() is called   }
     LOG_NOWAIT = $00000010;                 { do not wait for child processes       }
  { message facilities for openlog()...  }
     LOG_KERN = $00000001;                   { generated by system  }
     LOG_USER = $00000002;                   { generated by a process                }
     LOG_MAIL = $00000004;                   { generated by mail system              }
     LOG_NEWS = $00000008;                   { generated by news system              }
     LOG_UUCP = $00000010;                   { generated by UUCP system              }
     LOG_DAEMON = $00000020;                 { generated by daemon                   }
     LOG_AUTH = $00000040;                   { generated by auth. daemon             }
     LOG_CRON = $00000080;                   { generated by clock daemon             }
     LOG_LPR = $00000100;                    { generated by printer system           }
     LOG_LOCAL0 = $00000200;                 { local use                             }
     LOG_LOCAL1 = $00000400;
     LOG_LOCAL2 = $00000800;
     LOG_LOCAL3 = $00001000;
     LOG_LOCAL4 = $00002000;
     LOG_LOCAL5 = $00004000;
     LOG_LOCAL6 = $00008000;
     LOG_LOCAL7 = $00010000;
     LOG_UNUSED1 = $00020000;
     LOG_UNUSED2 = $00040000;
     LOG_UNUSED3 = $00080000;
     LOG_UNUSED4 = $00100000;
     LOG_UNUSED5 = $00200000;
     LOG_UNUSED6 = $00400000;                { unused                                }
     LOG_UNUSED7 = $00800000;                { unused                                }
  { for constructing 'maskpri' for setlogmask()...  }
  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }
//  function LOG_MASK(p : longint) : longint;
     LOG_EMERG = $01000000;
     LOG_ALERT = $02000000;                  { condition to correct immediately       }
     LOG_CRIT = $04000000;                   { critical condition                     }
     LOG_ERR = $08000000;                    { error message                          }
     LOG_WARNING = $10000000;                { warning message                        }
     LOG_NOTICE = $20000000;                 { condition requiring special handling   }
     LOG_INFO = $40000000;                   { general information message            }
     LOG_DEBUG = $80000000;                  { message useful for debugging programs  }
     P_cfgfile = 'sys:/etc/syslog.conf';     { with no filename argument, syslogd.nlm uses this configuration file:  }

  procedure closelog;cdecl;external libc_nlm name 'closelog';
  procedure openlog(ident:Pchar; logopt:longint; facility:longint);cdecl;external libc_nlm;
  function setlogmask(maskpri:longint):longint;cdecl;external libc_nlm name 'setlogmask';
{$ifndef DisableArrayOfConst}
  procedure syslog(priority:longint; message:Pchar; args:array of const);cdecl;external libc_nlm name 'syslog';
{$endif}
  procedure syslog(priority:longint; message:Pchar);cdecl;external libc_nlm name 'syslog';



// thread.h

  const
     THR_BOUND = $00000080;
     THR_DETACHED = PTHREAD_CREATE_DETACHED;
     THR_NEW_LWP = $FFFFFFFE;
     THR_SUSPENDED = $00000020;
     THR_DAEMON = $00000040;


{ type definitions...  }
type

//!!   Pthread_t = ^thread_t;
   thread_t = pthread_t;

//!!   Pthread_key_t = ^thread_key_t;
   thread_key_t = pthread_key_t;
{ prototypes...  }

type TThrStartRoutine = function (_para1:pointer):pointer; cdecl;
function thr_create(stack_based:pointer; stack_size:size_t;
           start_routine:TThrStartRoutine; arg:pointer; flags:longint;
           new_thr:Pthread_t):longint;cdecl;external libc_nlm name 'thr_create';
function thr_self:thread_t;cdecl;external libc_nlm name 'thr_self';
function thr_suspend(thr:thread_t):longint;cdecl;external libc_nlm name 'thr_suspend';
function thr_continue(thr:thread_t):longint;cdecl;external libc_nlm name 'thr_continue';
function thr_join(wait_for:thread_t; dead:Pthread_t; status:Ppointer):longint;cdecl;external libc_nlm name 'thr_join';
procedure thr_yield;cdecl;external libc_nlm name 'thr_yield';
procedure thr_exit(status:pointer);cdecl;external libc_nlm name 'thr_exit';
function thr_minstack:size_t;cdecl;external libc_nlm name 'thr_minstack';
function thr_kill(thr:thread_t; sig:longint):longint;cdecl;external libc_nlm name 'thr_kill';

function thr_sigsetmask(how:longint; _set:Psigset_t; oset:Psigset_t):longint;cdecl;external libc_nlm name 'thr_sigsetmask';
function thr_getconcurrency:longint;cdecl;external libc_nlm name 'thr_getconcurrency';
function thr_setconcurrency(new_level:longint):longint;cdecl;external libc_nlm name 'thr_setconcurrency';
function thr_getprio(thr:thread_t; pri:Plongint):longint;cdecl;external libc_nlm name 'thr_getprio';
function thr_setprio(thr:thread_t; pri:longint):longint;cdecl;external libc_nlm name 'thr_setprio';
function thr_keycreate(key:Pthread_key_t; _destructor:TCDeclProc1PtrArg):longint;cdecl;external libc_nlm name 'thr_keycreate';
function thr_getspecific(key:thread_key_t; value:Ppointer):longint;cdecl;external libc_nlm name 'thr_getspecific';
function thr_setspecific(key:thread_key_t; value:pointer):longint;cdecl;external libc_nlm name 'thr_setspecific';

// utf8.h

type
   Putf8_t = ^utf8_t;
   utf8_t = byte;
   PPutf8_t = ^Putf8_t;
// var __utf8width : array of byte;cvar;external;
{ prototypes...  }


function utf8width(ch:Putf8_t):size_t;cdecl;external libc_nlm name 'utf8width';
function utf8cat(tgt:Putf8_t; src:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8cat';
function utf8chr(_string:Putf8_t; ch:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8chr';
function utf8cmp(s1:Putf8_t; s2:Putf8_t):longint;cdecl;external libc_nlm name 'utf8cmp';
function utf8cpy(tgt:Putf8_t; src:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8cpy';
function utf8cspn(_string:Putf8_t; charset:Putf8_t):size_t;cdecl;external libc_nlm name 'utf8cspn';
function utf8dup(s:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8dup';
function utf8index(_string:Putf8_t; search:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8index';
function utf8len(_string:Putf8_t):size_t;cdecl;external libc_nlm name 'utf8len';
{$ifndef DisableArrayOfConst}
function utf8list(tgt:Putf8_t; s1:Putf8_t; args:array of const):Putf8_t;cdecl;external libc_nlm name 'utf8list';
{$endif}
function utf8list(tgt:Putf8_t; s1:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8list';
function utf8lwr(_string:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8lwr';
function utf8ncat(tgt:Putf8_t; src:Putf8_t; n:size_t):Putf8_t;cdecl;external libc_nlm name 'utf8ncat';
function utf8ncmp(s1:Putf8_t; s2:Putf8_t; n:size_t):longint;cdecl;external libc_nlm name 'utf8ncmp';
function utf8ncpy(tgt:Putf8_t; src:Putf8_t; n:size_t):Putf8_t;cdecl;external libc_nlm name 'utf8ncpy';
function utf8next(_string:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8next';
function utf8nlen(_string:Putf8_t; nbytes:size_t):size_t;cdecl;external libc_nlm name 'utf8nlen';
function utf8nset(base:Putf8_t; ch:Putf8_t; n:size_t):Putf8_t;cdecl;external libc_nlm name 'utf8nset';
function utf8pbrk(s1:Putf8_t; s2:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8pbrk';
function utf8prev(_string:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8prev';
function utf8rchr(_string:Putf8_t; ch:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8rchr';
function utf8rev(_string:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8rev';
function utf8size(_string:Putf8_t):size_t;cdecl;external libc_nlm name 'utf8size';
function utf8spn(_string:Putf8_t; charset:Putf8_t):size_t;cdecl;external libc_nlm name 'utf8spn';
function utf8str(as1:Putf8_t; as2:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8str';
function utf8tolower(_string:Putf8_t; dest:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8tolower';
function utf8tok_r(_string:Putf8_t; sepset:Putf8_t; lasts:PPutf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8tok_r';
function utf8toupper(_string:Putf8_t; dest:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8toupper';
function utf8upr(_string:Putf8_t):Putf8_t;cdecl;external libc_nlm name 'utf8upr';
function isutf8ascii(_string:Putf8_t):longint;cdecl;external libc_nlm name 'isutf8ascii';
function isutf8digit(_string:Putf8_t):longint;cdecl;external libc_nlm name 'isutf8digit';
function isutf8xdigit(_string:Putf8_t):longint;cdecl;external libc_nlm name 'isutf8xdigit';
function isutf8space(_string:Putf8_t):longint;cdecl;external libc_nlm name 'isutf8space';
function isutf8alnum(_string:Putf8_t):longint;cdecl;external libc_nlm name 'isutf8alnum';
function isutf8alpha(_string:Putf8_t):longint;cdecl;external libc_nlm name 'isutf8alpha';
function isutf8lower(_string:Putf8_t):longint;cdecl;external libc_nlm name 'isutf8lower';
function isutf8upper(_string:Putf8_t):longint;cdecl;external libc_nlm name 'isutf8upper';


// utime.h
{ turn on 1-byte packing...  }

{ access time  }
{ modification time  }
type
   Putimbuf = ^Tutimbuf;
   Tutimbuf = record
        actime  : time_t;
        modtime : time_t;
     end;
   utimbuf = Tutimbuf;

(** unsupported pragma#pragma pack()*)



function utime(path:Pchar; times:Putimbuf):longint;cdecl;external libc_nlm name 'utime';
function utime(path:Pchar; var times:Tutimbuf):longint;cdecl;external libc_nlm name 'utime';


// utsname.h
// wctype.h
type
   Pwctype_t = ^wctype_t;
   wctype_t =  Longint;
Const
   WCTYPE_UNDEF = 0;
   WCTYPE_ALNUM = 1;
   WCTYPE_ALPHA = 2;
   WCTYPE_BLANK = 3;
   WCTYPE_CNTRL = 4;
   WCTYPE_DIGIT = 5;
   WCTYPE_GRAPH = 6;
   WCTYPE_LOWER = 7;
   WCTYPE_PRINT = 8;
   WCTYPE_PUNCT = 9;
   WCTYPE_SPACE = 10;
   WCTYPE_UPPER = 11;
   WCTYPE_XDIGIT = 12;

type
   Pwctrans_t = ^wctrans_t;
   wctrans_t = wchar_t;

function iswalnum(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswalnum';
function iswalpha(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswalpha';
function iswblank(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswblank';
function iswcntrl(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswcntrl';
function iswdigit(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswdigit';
function iswgraph(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswgraph';
function iswlower(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswlower';
function iswprint(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswprint';
function iswpunct(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswpunct';
function iswspace(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswspace';
function iswupper(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswupper';
function iswxdigit(_para1:longint):longint;cdecl;external libc_nlm name 'iswxdigit';
function towlower(_para1:wint_t):wint_t;cdecl;external libc_nlm name 'towlower';
function towupper(_para1:wint_t):wint_t;cdecl;external libc_nlm name 'towupper';
function iswctype(_para1:wint_t; _para2:wctype_t):longint;cdecl;external libc_nlm name 'iswctype';

function wctype(_para1:Pchar):wctype_t;cdecl;external libc_nlm name 'wctype';
function towctrans(_para1:wint_t; _para2:wctrans_t):wint_t;cdecl;external libc_nlm name 'towctrans';

function wctrans(_para1:Pchar):wctrans_t;cdecl;external libc_nlm name 'wctrans';
function iswascii(_para1:wint_t):longint;cdecl;external libc_nlm name 'iswascii';


// windows.h


  const
     DLL_ACTUAL_DLLMAIN = 0;
     DLL_NLM_STARTUP = 1;        { start-up, 'lpvReserved' is NLM handle  }
     DLL_NLM_SHUTDOWN = 2;       { unload, 'lpvReserved' is NLM handle    }
     { standard DllMain() messages...  }
     DLL_PROCESS_ATTACH = 3;     { DLL "loaded" into application space    }
     DLL_THREAD_ATTACH = 4;      { application creating new thread        }
     DLL_THREAD_DETACH = 5;      { application thread exiting cleanly     }
     DLL_PROCESS_DETACH = 6;     { DLL "unloaded" from application space  }
     TLS_MINIMUM_AVAILABLE = 64; { minumum number of keys available       }


type

   PLPVOID = ^LPVOID;
   LPVOID = void;

   PBOOL = ^BOOL;
   BOOL = longint;

   PHMODULE = ^HMODULE;
   //HMODULE = void;

   PLPCTSTR = ^LPCTSTR;
   LPCTSTR = char;

   _PHINSTANCE = ^_HINSTANCE;
   _HINSTANCE = void;
{ Win32 DLL solutions for dynamic NLM libraries on NetWare...  }

function GetLastError:dword;cdecl;external libc_nlm name 'GetLastError';
procedure SetLastError(dwErrCode:dword);cdecl;external libc_nlm name 'SetLastError';
function FreeLibrary(hModule:HMODULE):BOOL;cdecl;external libc_nlm name 'FreeLibrary';
function LoadLibrary(lpFileName:LPCTSTR):HMODULE;cdecl;external libc_nlm name 'LoadLibrary';
{
** Prototype for libraries writing their own start-up and shut-down code.
** This is not an interface, but only a prototype for code furnished by the
** NLM library.
 }
function DllMain(hinstDLL:_HINSTANCE; fdwReason:dword; lpvReserve:LPVOID):BOOL;cdecl;external libc_nlm name 'DllMain';


// xmalloc.h

function xcalloc(_para1,_para2:size_t):pointer;cdecl;external libc_nlm name 'xcalloc';
procedure xfree(_para1:pointer);cdecl;external libc_nlm name 'xfree';
function xmalloc(_para1:size_t):pointer;cdecl;external libc_nlm name 'xmalloc';
function xrealloc(_para1:pointer; _para2:size_t):pointer;cdecl;external libc_nlm name 'xrealloc';


{$ifndef INCLUDED_FROM_SYSTEM}

implementation

function bisecond(var a : Tdos_tm) : word;
begin
   bisecond:=(a.flag0 and bm_dos_tm_bisecond) shr bp_dos_tm_bisecond;
end;

procedure set_bisecond(var a : Tdos_tm; __bisecond : word);
begin
   a.flag0:=a.flag0 or ((__bisecond shl bp_dos_tm_bisecond) and bm_dos_tm_bisecond);
end;

function minute(var a : Tdos_tm) : word;
begin
   minute:=(a.flag0 and bm_dos_tm_minute) shr bp_dos_tm_minute;
end;

procedure set_minute(var a : Tdos_tm; __minute : word);
begin
   a.flag0:=a.flag0 or ((__minute shl bp_dos_tm_minute) and bm_dos_tm_minute);
end;

function hour(var a : Tdos_tm) : word;
begin
   hour:=(a.flag0 and bm_dos_tm_hour) shr bp_dos_tm_hour;
end;

procedure set_hour(var a : Tdos_tm; __hour : word);
begin
   a.flag0:=a.flag0 or ((__hour shl bp_dos_tm_hour) and bm_dos_tm_hour);
end;

function day(var a : Tdos_tm) : word;
begin
   day:=(a.flag0 and bm_dos_tm_day) shr bp_dos_tm_day;
end;

procedure set_day(var a : Tdos_tm; __day : word);
begin
   a.flag0:=a.flag0 or ((__day shl bp_dos_tm_day) and bm_dos_tm_day);
end;

function month(var a : Tdos_tm) : word;
begin
   month:=(a.flag0 and bm_dos_tm_month) shr bp_dos_tm_month;
end;

procedure set_month(var a : Tdos_tm; __month : word);
begin
   a.flag0:=a.flag0 or ((__month shl bp_dos_tm_month) and bm_dos_tm_month);
end;

function year(var a : Tdos_tm) : word;
begin
   year:=(a.flag0 and bm_dos_tm_year) shr bp_dos_tm_year;
end;

procedure set_year(var a : Tdos_tm; __year : word);
begin
   a.flag0:=a.flag0 or ((__year shl bp_dos_tm_year) and bm_dos_tm_year);
end;

end.

{$endif}
