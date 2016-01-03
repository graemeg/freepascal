{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    Strings unit for PChar (asciiz/C compatible strings) handling

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit strings;
{$S-}
{$inline on}
interface

    { Implemented in System Unit }
    function strpas(p:pchar):shortstring;inline;

    function strlen(p:pchar):sizeint;external name 'FPC_PCHAR_LENGTH';

    { Converts a Pascal string to a null-terminated string }
    function strpcopy(d : pchar;const s : string) : pchar;

    { Copies source to dest, returns a pointer to dest }
    function strcopy(dest,source : pchar) : pchar; overload;

    { Copies at most maxlen bytes from source to dest. }
    { Returns a pointer to dest }
    function strlcopy(dest,source : pchar;maxlen : SizeInt) : pchar; overload;

    { Copies source to dest and returns a pointer to the terminating }
    { null character.    }
    function strecopy(dest,source : pchar) : pchar;

    { Returns a pointer tro the terminating null character of p }
    function strend(p : pchar) : pchar;

    { Appends source to dest, returns a pointer do dest}
    function strcat(dest,source : pchar) : pchar;

    { Compares str1 und str2, returns }
    { a value <0 if str1<str2;        }
    {  0 when str1=str2               }
    { and a value >0 if str1>str2     }
    function strcomp(str1,str2 : pchar) : SizeInt;

    { The same as strcomp, but at most l characters are compared  }
    function strlcomp(str1,str2 : pchar;l : SizeInt) : SizeInt;

    { The same as strcomp but case insensitive }
    function stricomp(str1,str2 : pchar) : SizeInt;

    { The same as stricomp, but at most l characters are compared }
    function strlicomp(str1,str2 : pchar;l : SizeInt) : SizeInt;

    { Copies l characters from source to dest, returns dest. }
    function strmove(dest,source : pchar;l : SizeInt) : pchar;

    { Appends at most l characters from source to dest }
    function strlcat(dest,source : pchar;l : SizeInt) : pchar;

    { Returns a pointer to the first occurrence of c in p }
    { If c doesn't occur, nil is returned }
    function strscan(p : pchar;c : char) : pchar;

    { The same as strscan but case insensitive }
    function striscan(p : pchar;c : char) : pchar;

    { Returns a pointer to the last occurrence of c in p }
    { If c doesn't occur, nil is returned }
    function strrscan(p : pchar;c : char) : pchar;

    { The same as strrscan but case insensitive }
    function strriscan(p : pchar;c : char) : pchar;

    { converts p to all-lowercase, returns p }
    function strlower(p : pchar) : pchar;

    { converts p to all-uppercase, returns p }
    function strupper(p : pchar) : pchar;

    { Returns a pointer to the first occurrence of str2 in }
    { str1 Otherwise returns nil }
    function strpos(str1,str2 : pchar) : pchar;

    { The same as strpos but case insensitive       }
    function stripos(str1,str2 : pchar) : pchar;

    { Makes a copy of p on the heap, and returns a pointer to this copy  }
    function strnew(p : pchar) : pchar;

    { Allocates L bytes on the heap, returns a pchar pointer to it }
    function stralloc(L : SizeInt) : pchar;

    { Releases a null-terminated string from the heap }
    procedure strdispose(p : pchar);

implementation

{$ifdef FPC_USE_LIBC}
{$i cgenstr.inc}
{$endif FPC_USE_LIBC}

{  Read Processor dependent part, shared with sysutils unit }
{$i strings.inc }

{ Read processor denpendent part, NOT shared with sysutils unit }
{$i stringss.inc }

{ Read generic string functions that are not implemented for the processor }
{$i genstr.inc}
{$i genstrs.inc}

{ Functions not in assembler, but shared with sysutils unit  }
{$i stringsi.inc}

{ Functions, different from the one in sysutils }

<<<<<<< HEAD
<<<<<<< HEAD

    procedure fpc_pchar_to_shortstr(var res : openstring;p:pchar);[external name 'FPC_PCHAR_TO_SHORTSTR'];


    function strpas(p:pchar):shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
      begin
        fpc_pchar_to_shortstr(strpas,p);
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_STRTOSHORTSTRINGPROC}

    { also define alias which can be used inside the system unit }
    function fpc_pchar_to_shortstr(p:pchar):shortstring;[external name 'FPC_PCHAR_TO_SHORTSTR'];

{$else FPC_STRTOSHORTSTRINGPROC}

    { also define alias which can be used inside the system unit }
    procedure fpc_pchar_to_shortstr(var res : openstring;p:pchar);[external name 'FPC_PCHAR_TO_SHORTSTR'];

{$endif FPC_STRTOSHORTSTRINGPROC}

    function strpas(p:pchar):shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
      begin
    {$ifndef FPC_STRTOSHORTSTRINGPROC}
        strpas:=fpc_pchar_to_shortstr(p);
    {$else FPC_STRTOSHORTSTRINGPROC}
        fpc_pchar_to_shortstr(strpas,p);
    {$endif FPC_STRTOSHORTSTRINGPROC}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;

    function stralloc(L : SizeInt) : pchar;

      begin
         StrAlloc:=Nil;
         GetMem (Stralloc,l);
      end;

    function strnew(p : pchar) : pchar;

      var
         len : SizeInt;

      begin
         strnew:=nil;
         if (p=nil) or (p^=#0) then
           exit;
         len:=strlen(p)+1;
         getmem(strnew,len);
         if strnew<>nil then
           move(p^,strnew^,len);
      end;

    procedure strdispose(p : pchar);

      begin
         if p<>nil then
           freemem(p);
      end;

end.
