{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements some support functions

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published
    by the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.


****************************************************************************
}
{# This unit contains some generic support functions which are used
   in the different parts of the compiler.
}
unit cutils;

{$i fpcdefs.inc}

interface

  uses
    constexp;

  type
    Tcharset=set of char;

  var
    internalerrorproc : procedure(i:longint);


    {# Returns the minimal value between @var(a) and @var(b) }
    function min(a,b : longint) : longint;{$ifdef USEINLINE}inline;{$endif}
    function min(a,b : int64) : int64;{$ifdef USEINLINE}inline;{$endif}
    function min(a,b : qword) : qword;{$ifdef USEINLINE}inline;{$endif}
    {# Returns the maximum value between @var(a) and @var(b) }
    function max(a,b : longint) : longint;{$ifdef USEINLINE}inline;{$endif}
    function max(a,b : int64) : int64;{$ifdef USEINLINE}inline;{$endif}
    function max(a,b : qword) : qword;{$ifdef USEINLINE}inline;{$endif}

    { These functions are intenionally put here and not in the constexp unit.
      Since Tconstexprint may be automatically converted to int, which causes
      loss of data and since there are already min and max functions for ints in
      this unit, we put min and max for Tconstexprint as well. This way we avoid
      potential bugs, caused by code unintentionally calling the int versions of
      min/max on Tconstexprint, because of only including cutils and forgetting
      the constexp unit in the uses clause. }
    function min(const a,b : Tconstexprint) : Tconstexprint;{$ifdef USEINLINE}inline;{$endif}
    function max(const a,b : Tconstexprint) : Tconstexprint;{$ifdef USEINLINE}inline;{$endif}

    {# Return value @var(i) aligned on @var(a) boundary }
    function align(i,a:longint):longint;{$ifdef USEINLINE}inline;{$endif}
    function align(i,a:int64):int64;{$ifdef USEINLINE}inline;{$endif}
    function align(i,a:qword):qword;{$ifdef USEINLINE}inline;{$endif}
    { if you have an address aligned using "oldalignment" and add an
      offset of (a multiple of) offset to it, this function calculates
      the new minimally guaranteed alignment
    }
    function newalignment(oldalignment: longint; offset: int64): longint;
    {# Return @var(b) with the bit order reversed }
    function reverse_byte(b: byte): byte;
    {# Return @var(w) with the bit order reversed }
    function reverse_word(w: word): word;
    {# Return @var(l) with the bit order reversed }
    function reverse_longword(l: longword): longword;

    function next_prime(l: longint): longint;

    function used_align(varalign,minalign,maxalign:longint):longint;
    function isbetteralignedthan(new, org, limit: cardinal): boolean;
    function packedbitsloadsize(bitlen: int64) : int64;
    procedure Replace(var s:string;s1:string;const s2:string);
    procedure Replace(var s:AnsiString;s1:string;const s2:AnsiString);
    procedure ReplaceCase(var s:string;const s1,s2:string);
    procedure ReplaceCase(var s:ansistring;const s1,s2:ansistring);
    Function MatchPattern(const pattern,what:string):boolean;
    function upper(const c : char) : char;
    function upper(const s : string) : string;
    function upper(const s : ansistring) : ansistring;
    function lower(const c : char) : char;
    function lower(const s : string) : string;
    function lower(const s : ansistring) : ansistring;
    function rpos(const needle: char; const haystack: shortstring): longint; overload;
    function rpos(const needle: shortstring; const haystack: shortstring): longint; overload;
    function trimbspace(const s:string):string;
    function trimspace(const s:string):string;
    function space (b : longint): string;
    function PadSpace(const s:string;len:longint):string;
    function GetToken(var s:string;endchar:char):string;
    procedure uppervar(var s : string);
    function realtostr(e:extended):string;{$ifdef USEINLINE}inline;{$endif}
    function tostr(i : qword) : string;{$ifdef USEINLINE}inline;{$endif}overload;
    function tostr(i : int64) : string;{$ifdef USEINLINE}inline;{$endif}overload;
    function tostr(i : longint) : string;{$ifdef USEINLINE}inline;{$endif}overload;
    function tostr_with_plus(i : int64) : string;{$ifdef USEINLINE}inline;{$endif}
    function DStr(l:longint):string;
    {# Returns true if the string s is a number }
    function is_number(const s : string) : boolean;{$ifdef USEINLINE}inline;{$endif}
    {# Returns true if value is a power of 2, the actual
       exponent value is returned in power.
    }
    function ispowerof2(value : int64;out power : longint) : boolean;
    function ispowerof2(const value : Tconstexprint;out power : longint) : boolean;
    {# Returns true if abs(value) is a power of 2, the actual
       exponent value is returned in power.
    }
    function isabspowerof2(const value : Tconstexprint;out power : longint) : boolean;
    function nextpowerof2(value : int64; out power: longint) : int64;
{$ifdef VER2_6}  { only 2.7.1+ has a popcnt function in the system unit }
    function PopCnt(AValue : Byte): Byte;
    function PopCnt(AValue : Word): Word;
    function PopCnt(AValue : DWord): DWord;
    function PopCnt(Const AValue : QWord): QWord;
{$endif VER2_6}

    function backspace_quote(const s:string;const qchars:Tcharset):string;
    function octal_quote(const s:string;const qchars:Tcharset):string;

    {# If the string is quoted, in accordance with pascal, it is
       dequoted and returned in s, and the function returns true.
       If it is not quoted, or if the quoting is bad, s is not touched,
       and false is returned.
    }
    function DePascalQuote(var s: ansistring): Boolean;
    function CompareStr(const S1, S2: string): Integer;
    function CompareText(S1, S2: string): integer;
    function CompareVersionStrings(s1,s2: string): longint;

    { releases the string p and assignes nil to p }
    { if p=nil then freemem isn't called          }
    procedure stringdispose(var p : pshortstring);{$ifdef USEINLINE}inline;{$endif}


    { allocates mem for a copy of s, copies s to this mem and returns }
    { a pointer to this mem                                           }
    function stringdup(const s : shortstring) : pshortstring;{$ifdef USEINLINE}inline;{$endif}
    function stringdup(const s : ansistring) : pshortstring;{$ifdef USEINLINE}inline;{$endif}

    {# Allocates memory for the string @var(s) and copies s as zero
       terminated string to that allocated memory and returns a pointer
       to that mem
    }
    function  strpnew(const s : string) : pchar;
    function  strpnew(const s : ansistring) : pchar;

    {# makes the character @var(c) lowercase, with spanish, french and german
       character set
    }
    function lowercase(c : char) : char;

    { makes zero terminated string to a pascal string }
    { the data in p is modified and p is returned     }
    function pchar2pshortstring(p : pchar) : pshortstring;

    { inverse of pchar2pshortstring }
    function pshortstring2pchar(p : pshortstring) : pchar;

    { allocate a new pchar with the contents of a}
    function ansistring2pchar(const a: ansistring) : pchar;

    { Ansistring (pchar+length) support }
    procedure ansistringdispose(var p : pchar;length : longint);
    function compareansistrings(p1,p2 : pchar;length1,length2 : longint) : longint;
    function concatansistrings(p1,p2 : pchar;length1,length2 : longint) : pchar;

    {Lzw encode/decode to compress strings -> save memory.}
    function minilzw_encode(const s:string):string;
    function minilzw_decode(const s:string):string;

    Function nextafter(x,y:double):double;

    function LengthUleb128(a: qword) : byte;
    function LengthSleb128(a: int64) : byte;
    function EncodeUleb128(a: qword;out buf;len: byte) : byte;
    function EncodeSleb128(a: int64;out buf;len: byte) : byte;

  { hide Sysutils.ExecuteProcess in units using this one after SysUtils}
  const
    ExecuteProcess = 'Do not use' deprecated 'Use cfileutil.RequotedExecuteProcess instead, ExecuteProcess cannot deal with single quotes as used by Unix command lines';

implementation

    uses
      SysUtils;

    var
      uppertbl,
      lowertbl  : array[char] of char;


    function min(a,b : longint) : longint;{$ifdef USEINLINE}inline;{$endif}
    {
      return the minimal of a and b
    }
      begin
         if a<=b then
           min:=a
         else
           min:=b;
      end;


    function min(a,b : int64) : int64;{$ifdef USEINLINE}inline;{$endif}
    {
      return the minimal of a and b
    }
      begin
         if a<=b then
           min:=a
         else
           min:=b;
      end;


    function min(const a,b : Tconstexprint) : Tconstexprint;{$ifdef USEINLINE}inline;{$endif}
    {
      return the minimal of a and b
    }
      begin
         if a<=b then
           min:=a
         else
           min:=b;
      end;


    function min(a,b : qword) : qword;
    {
      return the minimal of a and b
    }
      begin
         if a<=b then
           min:=a
         else
           min:=b;
      end;


    function max(a,b : longint) : longint;{$ifdef USEINLINE}inline;{$endif}
    {
      return the maximum of a and b
    }
      begin
         if a>=b then
           max:=a
         else
           max:=b;
      end;


    function max(a,b : int64) : int64;{$ifdef USEINLINE}inline;{$endif}
    {
      return the maximum of a and b
    }
      begin
         if a>=b then
           max:=a
         else
           max:=b;
      end;


    function max(a,b : qword) : qword;{$ifdef USEINLINE}inline;{$endif}
    {
      return the maximum of a and b
    }
      begin
         if a>=b then
           max:=a
         else
           max:=b;
      end;


    function max(const a,b : Tconstexprint) : Tconstexprint;{$ifdef USEINLINE}inline;{$endif}
    {
      return the maximum of a and b
    }
      begin
         if a>=b then
           max:=a
         else
           max:=b;
      end;


    function newalignment(oldalignment: longint; offset: int64): longint;
      var
        localoffset: longint;
      begin
        localoffset:=longint(offset);
        while (localoffset mod oldalignment)<>0 do
          oldalignment:=oldalignment div 2;
        newalignment:=oldalignment;
      end;


    function reverse_byte(b: byte): byte;
      const
        reverse_nible:array[0..15] of 0..15 =
          (%0000,%1000,%0100,%1100,%0010,%1010,%0110,%1110,
           %0001,%1001,%0101,%1101,%0011,%1011,%0111,%1111);
      begin
        reverse_byte:=(reverse_nible[b and $f] shl 4) or reverse_nible[b shr 4];
      end;

    function reverse_word(w: word): word;
      type
        TWordRec = packed record
          hi, lo: Byte;
        end;

      begin
        TWordRec(reverse_word).hi := reverse_byte(TWordRec(w).lo);
        TWordRec(reverse_word).lo := reverse_byte(TWordRec(w).hi);
      end;


    function reverse_longword(l: longword): longword;
      type
        TLongWordRec = packed record
          b: array[0..3] of Byte;
        end;

      begin
        TLongWordRec(reverse_longword).b[0] := reverse_byte(TLongWordRec(l).b[3]);
        TLongWordRec(reverse_longword).b[1] := reverse_byte(TLongWordRec(l).b[2]);
        TLongWordRec(reverse_longword).b[2] := reverse_byte(TLongWordRec(l).b[1]);
        TLongWordRec(reverse_longword).b[3] := reverse_byte(TLongWordRec(l).b[0]);
      end;


    function align(i,a:longint):longint;{$ifdef USEINLINE}inline;{$endif}
    {
      return value <i> aligned <a> boundary
    }
      begin
        { for 0 and 1 no aligning is needed }
        if a<=1 then
          result:=i
        else
          begin
            if i<0 then
              result:=((i+1-a) div a) * a
            else
              result:=((i-1+a) div a) * a;
          end;
      end;


    function align(i,a:int64):int64;{$ifdef USEINLINE}inline;{$endif}
    {
      return value <i> aligned <a> boundary
    }
      begin
        { for 0 and 1 no aligning is needed }
        if a<=1 then
          result:=i
        else
          begin
            if i<0 then
              result:=((i+1-a) div a) * a
            else
              result:=((i-1+a) div a) * a;
          end;
      end;


    function align(i,a:qword):qword;{$ifdef USEINLINE}inline;{$endif}
    {
      return value <i> aligned <a> boundary
    }
      begin
        { for 0 and 1 no aligning is needed }
        if (a<=1) or (i=0) then
          result:=i
        else
          result:=((i-1+a) div a) * a;
      end;


    function packedbitsloadsize(bitlen: int64) : int64;
      begin
         case bitlen of
           1,2,4,8:
             result := 1;
           { 10 bits can never be split over 3 bytes via 1-8-1, because it }
           { always starts at a multiple of 10 bits. Same for the others.  }
           3,5,6,7,9,10,12,16:
             result := 2;
  {$ifdef cpu64bitalu}
           { performance penalty for unaligned 8 byte access is much   }
           { higher than for unaligned 4 byte access, at least on ppc, }
           { so use 4 bytes even in some cases where a value could     }
           { always loaded using a single 8 byte load (e.g. in case of }
           { 28 bit values)                                            }
           11,13,14,15,17..32:
             result := 4;
           else
             result := 8;
  {$else cpu64bitalu}
           else
             result := 4;
  {$endif cpu64bitalu}
         end;
      end;


    function isbetteralignedthan(new, org, limit: cardinal): boolean;
      var
        cnt: cardinal;
      begin
        cnt:=2;
        while (cnt <= limit) do
          begin
            if (org and (cnt-1)) > (new and (cnt-1)) then
              begin
                result:=true;
                exit;
              end
            else if (org and (cnt-1)) < (new and (cnt-1)) then
              begin
                result:=false;
                exit;
              end;
            cnt:=cnt*2;
          end;
        result:=false;
      end;


    function next_prime(l: longint): longint;
      var
        check, checkbound: longint;
        ok: boolean;
      begin
        result:=l or 1;
        while l<high(longint) do
          begin
            ok:=true;
            checkbound:=trunc(sqrt(l));
            check:=3;
            while check<checkbound do
              begin
                if (l mod check) = 0 then
                  begin
                    ok:=false;
                    break;
                  end;
                inc(check,2);
              end;
            if ok then
              exit;
            inc(l);
          end;
      end;


    function used_align(varalign,minalign,maxalign:longint):longint;
      begin
        { varalign  : minimum alignment required for the variable
          minalign  : Minimum alignment of this structure, 0 = undefined
          maxalign  : Maximum alignment of this structure, 0 = undefined }
        if (minalign>0) and
           (varalign<minalign) then
         used_align:=minalign
        else
         begin
           if (maxalign>0) and
              (varalign>maxalign) then
            used_align:=maxalign
           else
            used_align:=varalign;
         end;
      end;


    procedure Replace(var s:string;s1:string;const s2:string);
      var
         last,
         i  : longint;
      begin
        s1:=upper(s1);
        last:=0;
        repeat
          i:=pos(s1,upper(s));
          if i=last then
           i:=0;
          if (i>0) then
           begin
             Delete(s,i,length(s1));
             Insert(s2,s,i);
             last:=i;
           end;
        until (i=0);
      end;


    procedure Replace(var s:AnsiString;s1:string;const s2:AnsiString);
      var
         last,
         i  : longint;
      begin
        s1:=upper(s1);
        last:=0;
        repeat
          i:=pos(s1,upper(s));
          if i=last then
           i:=0;
          if (i>0) then
           begin
             Delete(s,i,length(s1));
             Insert(s2,s,i);
             last:=i;
           end;
        until (i=0);
      end;


    procedure ReplaceCase(var s:string;const s1,s2:string);
      var
         last,
         i  : longint;
      begin
        last:=0;
        repeat
          i:=pos(s1,s);
          if i=last then
           i:=0;
          if (i>0) then
           begin
             Delete(s,i,length(s1));
             Insert(s2,s,i);
             last:=i;
           end;
        until (i=0);
      end;


    procedure ReplaceCase(var s: ansistring; const s1, s2: ansistring);
      var
         last,
         i  : longint;
      begin
        last:=0;
        repeat
          i:=pos(s1,s);
          if i=last then
           i:=0;
          if (i>0) then
           begin
             Delete(s,i,length(s1));
             Insert(s2,s,i);
             last:=i;
           end;
        until (i=0);
      end;


    Function MatchPattern(const pattern,what:string):boolean;
      var
        found : boolean;
        i1,i2 : longint;
      begin
        i1:=0;
        i2:=0;
        if pattern='' then
          begin
            result:=(what='');
            exit;
          end;
        found:=true;
        repeat
          inc(i1);
          if (i1>length(pattern)) then
            break;
          inc(i2);
          if (i2>length(what)) then
            break;
          case pattern[i1] of
            '?' :
              found:=true;
            '*' :
              begin
                found:=true;
                if (i1=length(pattern)) then
                 i2:=length(what)
                else
                 if (i1<length(pattern)) and (pattern[i1+1]<>what[i2]) then
                  begin
                    if i2<length(what) then
                     dec(i1)
                  end
                else
                 if i2>1 then
                  dec(i2);
              end;
            else
              found:=(pattern[i1]=what[i2]) or (what[i2]='?');
          end;
        until not found;
        if found then
          begin
            found:=(i2>=length(what)) and
                   (
                    (i1>length(pattern)) or
                    ((i1=length(pattern)) and
                     (pattern[i1]='*'))
                   );
          end;
        result:=found;
      end;


    function upper(const c : char) : char;
    {
      return uppercase of c
    }
      begin
        upper:=uppertbl[c];
      end;


    function upper(const s : string) : string;
    {
      return uppercased string of s
    }
      var
        i  : longint;
      begin
        for i:=1 to length(s) do
          upper[i]:=uppertbl[s[i]];
        upper[0]:=s[0];
      end;


    function upper(const s : ansistring) : ansistring;
    {
      return uppercased string of s
    }
      var
        i  : longint;
      begin
        Result:='';
        setlength(upper,length(s));
        for i:=1 to length(s) do
          upper[i]:=uppertbl[s[i]];
      end;


    function lower(const c : char) : char;
    {
      return lowercase of c
    }
      begin
        lower:=lowertbl[c];
      end;


    function lower(const s : string) : string;
    {
      return lowercased string of s
    }
      var
        i : longint;
      begin
        for i:=1 to length(s) do
          lower[i]:=lowertbl[s[i]];
        lower[0]:=s[0];
      end;


    function lower(const s : ansistring) : ansistring;
    {
      return lowercased string of s
    }
      var
        i : longint;
      begin
        Result:='';
        setlength(lower,length(s));
        for i:=1 to length(s) do
          lower[i]:=lowertbl[s[i]];
      end;


    procedure uppervar(var s : string);
    {
      uppercase string s
    }
      var
         i : longint;
      begin
         for i:=1 to length(s) do
          s[i]:=uppertbl[s[i]];
      end;


    procedure initupperlower;
      var
        c : char;
      begin
        for c:=#0 to #255 do
         begin
           lowertbl[c]:=c;
           uppertbl[c]:=c;
           case c of
             'A'..'Z' :
               lowertbl[c]:=char(byte(c)+32);
             'a'..'z' :
               uppertbl[c]:=char(byte(c)-32);
           end;
         end;
      end;


    function DStr(l:longint):string;
      var
        TmpStr : string[32];
        i : longint;
      begin
        Str(l,TmpStr);
        i:=Length(TmpStr);
        while (i>3) do
         begin
           dec(i,3);
           if TmpStr[i]<>'-' then
            insert('.',TmpStr,i+1);
         end;
        DStr:=TmpStr;
      end;


    function rpos(const needle: char; const haystack: shortstring): longint;
      begin
        result:=length(haystack);
        while (result>0) do
          begin
            if haystack[result]=needle then
              exit;
            dec(result);
          end;
      end;


    function rpos(const needle: shortstring; const haystack: shortstring): longint;
      begin
        result:=0;
        if (length(needle)=0) or
           (length(needle)>length(haystack)) then
          exit;
        result:=length(haystack)-length(needle);
        repeat
          if (haystack[result]=needle[1]) and
             (copy(haystack,result,length(needle))=needle) then
            exit;
          dec(result);
        until result=0;
      end;


    function trimbspace(const s:string):string;
    {
      return s with all leading spaces and tabs removed
    }
      var
        i,j : longint;
      begin
        j:=1;
        i:=length(s);
        while (j<i) and (s[j] in [#9,' ']) do
         inc(j);
        trimbspace:=Copy(s,j,i-j+1);
      end;



    function trimspace(const s:string):string;
    {
      return s with all leading and ending spaces and tabs removed
    }
      var
        i,j : longint;
      begin
        i:=length(s);
        while (i>0) and (s[i] in [#9,' ']) do
         dec(i);
        j:=1;
        while (j<i) and (s[j] in [#9,' ']) do
         inc(j);
        trimspace:=Copy(s,j,i-j+1);
      end;


    function space (b : longint): string;
      var
       s: string;
      begin
        space[0] := chr(b);
        s[0] := chr(b);
        FillChar (S[1],b,' ');
        space:=s;
      end;


    function PadSpace(const s:string;len:longint):string;
    {
      return s with spaces add to the end
    }
      begin
         if length(s)<len then
          PadSpace:=s+Space(len-length(s))
         else
          PadSpace:=s;
      end;


    function GetToken(var s:string;endchar:char):string;
      var
        i : longint;
        quote : char;
      begin
        GetToken:='';
        s:=TrimSpace(s);
        if (length(s)>0) and
           (s[1] in ['''','"']) then
         begin
           quote:=s[1];
           i:=1;
           while (i<length(s)) do
            begin
              inc(i);
              if s[i]=quote then
               begin
                 { Remove double quote }
                 if (i<length(s)) and
                    (s[i+1]=quote) then
                  begin
                    Delete(s,i,1);
                    inc(i);
                  end
                 else
                  begin
                    GetToken:=Copy(s,2,i-2);
                    Delete(s,1,i);
                    exit;
                  end;
               end;
            end;
           GetToken:=s;
           s:='';
         end
        else
         begin
           i:=pos(EndChar,s);
           if i=0 then
            begin
              GetToken:=s;
              s:='';
              exit;
            end
           else
            begin
              GetToken:=Copy(s,1,i-1);
              Delete(s,1,i);
              exit;
            end;
         end;
      end;


   function realtostr(e:extended):string;{$ifdef USEINLINE}inline;{$endif}
     begin
        str(e,result);
     end;


   function tostr(i : qword) : string;{$ifdef USEINLINE}inline;{$endif}overload;
   {
     return string of value i
   }
     begin
        str(i,result);
     end;


   function tostr(i : int64) : string;{$ifdef USEINLINE}inline;{$endif}overload;
   {
     return string of value i
   }
     begin
        str(i,result);
     end;


   function tostr(i : longint) : string;{$ifdef USEINLINE}inline;{$endif}overload;
   {
     return string of value i
   }
     begin
        str(i,result);
     end;


   function tostr_with_plus(i : int64) : string;{$ifdef USEINLINE}inline;{$endif}
   {
     return string of value i, but always include a + when i>=0
   }
     begin
        str(i,result);
        if i>=0 then
          result:='+'+result;
     end;


    function is_number(const s : string) : boolean;{$ifdef USEINLINE}inline;{$endif}
    {
      is string a correct number ?
    }
      var
         w : integer;
         l : longint;
      begin
         val(s,l,w);
         // remove warning
         l:=l;
         is_number:=(w=0);
      end;


    function ispowerof2(value : int64;out power : longint) : boolean;
    {
      return if value is a power of 2. And if correct return the power
    }
      begin
        if (value <= 0) or (value and (value - 1) <> 0) then
          exit(false);
        power:=BsfQWord(value);
        result:=true;
      end;


    function ispowerof2(const value: Tconstexprint; out power: longint): boolean;
      begin
        if value.signed or
           (value.uvalue<=high(int64)) then
          result:=ispowerof2(value.svalue,power)
        else if not value.signed and
            (value.svalue=low(int64)) then
          begin
            result:=true;
            power:=63;
          end
        else
          result:=false;
      end;


    function isabspowerof2(const value : Tconstexprint;out power : longint) : boolean;
      begin
        if ispowerof2(value,power) then
          result:=true
        else if value.signed and (value.svalue<0) and (value.svalue<>low(int64)) and ispowerof2(-value.svalue,power) then
          result:=true
        else
          result:=false;
      end;


    function nextpowerof2(value : int64; out power: longint) : int64;
    {
      returns the power of 2 >= value
    }
      var
        i : longint;
      begin
        result := 0;
        power := -1;
        if ((value <= 0) or
            (value >= $4000000000000000)) then
          exit;
        result := 1;
        for i:=0 to 63 do
          begin
            if result>=value then
              begin
                power := i;
                exit;
              end;
            result:=result shl 1;
          end;
      end;

{$ifdef VER2_6}
    const
      PopCntData : array[0..15] of byte = (0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4);

    function PopCnt(AValue : Byte): Byte;
      begin
        Result:=PopCntData[AValue and $f]+PopCntData[(AValue shr 4) and $f];
      end;


    function PopCnt(AValue : Word): Word;
      var
        i : SizeInt;
      begin
        Result:=0;
        for i:=0 to 3 do
          begin
            inc(Result,PopCntData[AValue and $f]);
            AValue:=AValue shr 4;
          end;
      end;


    function PopCnt(AValue : DWord): DWord;
      var
        i : SizeInt;
      begin
        Result:=0;
        for i:=0 to 7 do
          begin
            inc(Result,PopCntData[AValue and $f]);
            AValue:=AValue shr 4;
          end;
      end;


    function PopCnt(Const AValue : QWord): QWord;
      begin
        Result:=PopCnt(lo(AValue))+PopCnt(hi(AValue))
      end;
 {$endif VER2_6}


    function backspace_quote(const s:string;const qchars:Tcharset):string;

    var i:byte;

    begin
      backspace_quote:='';
      for i:=1 to length(s) do
        begin
          if (s[i]=#10) and (#10 in qchars) then
            backspace_quote:=backspace_quote+'\n'
          else if (s[i]=#13) and (#13 in qchars) then
            backspace_quote:=backspace_quote+'\r'
          else
            begin
              if s[i] in qchars then
                backspace_quote:=backspace_quote+'\';
              backspace_quote:=backspace_quote+s[i];
            end;
        end;
    end;


    function octal_quote(const s:string;const qchars:Tcharset):string;

    var i:byte;

    begin
      octal_quote:='';
      for i:=1 to length(s) do
        begin
          if s[i] in qchars then
            begin
              if ord(s[i])<64 then
                octal_quote:=octal_quote+'\'+octstr(ord(s[i]),3)
              else
                octal_quote:=octal_quote+'\'+octstr(ord(s[i]),4);
            end
          else
            octal_quote:=octal_quote+s[i];
        end;
    end;


    function DePascalQuote(var s: ansistring): Boolean;
      var
        destPos, sourcePos, len: Integer;
        t: string;
        ch: Char;
    begin
      t:='';
      DePascalQuote:= false;
      len:= length(s);
      if (len >= 1) and (s[1] = '''') then
        begin
          {Remove quotes, exchange '' against ' }
          destPos := 0;
          sourcepos:=1;
          while (sourcepos<len) do
            begin
              inc(sourcePos);
              ch := s[sourcePos];
              if ch = '''' then
                begin
                  inc(sourcePos);
                  if (sourcePos <= len) and (s[sourcePos] = '''') then
                    {Add the quote as part of string}
                  else
                    begin
                      SetLength(t, destPos);
                      s:= t;
                      Exit(true);
                    end;
                end;
              inc(destPos);
              t[destPos] := ch;
            end;
        end;
    end;


    function pchar2pshortstring(p : pchar) : pshortstring;
      var
         w,i : longint;
      begin
         w:=strlen(p);
         for i:=w-1 downto 0 do
           p[i+1]:=p[i];
         p[0]:=chr(w);
         pchar2pshortstring:=pshortstring(p);
      end;


    function pshortstring2pchar(p : pshortstring) : pchar;
      var
         w,i : longint;
      begin
         w:=length(p^);
         for i:=1 to w do
           p^[i-1]:=p^[i];
         p^[w]:=#0;
         pshortstring2pchar:=pchar(p);
      end;


    function ansistring2pchar(const a: ansistring) : pchar;
      var
        len: ptrint;
      begin
        len:=length(a);
        getmem(result,len+1);
        if (len<>0) then
          move(a[1],result[0],len);
        result[len]:=#0;
      end;


    function lowercase(c : char) : char;
       begin
          case c of
             #65..#90 : c := chr(ord (c) + 32);
             #154 : c:=#129;  { german }
             #142 : c:=#132;  { german }
             #153 : c:=#148;  { german }
             #144 : c:=#130;  { french }
             #128 : c:=#135;  { french }
             #143 : c:=#134;  { swedish/norge (?) }
             #165 : c:=#164;  { spanish }
             #228 : c:=#229;  { greek }
             #226 : c:=#231;  { greek }
             #232 : c:=#227;  { greek }
          end;
          lowercase := c;
       end;


    function strpnew(const s : string) : pchar;
      var
         p : pchar;
      begin
         getmem(p,length(s)+1);
         move(s[1],p^,length(s));
         p[length(s)]:=#0;
         result:=p;
      end;

    function strpnew(const s: ansistring): pchar;
      var
         p : pchar;
      begin
        getmem(p,length(s)+1);
        move(s[1],p^,length(s)+1);
        result:=p;
      end;


    procedure stringdispose(var p : pshortstring);{$ifdef USEINLINE}inline;{$endif}
      begin
         if assigned(p) then
           begin
             freemem(p);
             p:=nil;
           end;
      end;


    function stringdup(const s : shortstring) : pshortstring;{$ifdef USEINLINE}inline;{$endif}
      begin
         getmem(result,length(s)+1);
         result^:=s;
      end;


    function stringdup(const s : ansistring) : pshortstring;{$ifdef USEINLINE}inline;{$endif}
      begin
         getmem(result,length(s)+1);
         result^:=s;
      end;

    function CompareStr(const S1, S2: string): Integer;
      var
        count, count1, count2: integer;
      begin
        result := 0;
        Count1 := Length(S1);
        Count2 := Length(S2);
        if Count1>Count2 then
          Count:=Count2
        else
          Count:=Count1;
        result := CompareChar(S1[1],S2[1], Count);
        if result=0 then
          result:=Count1-Count2;
      end;


    function CompareText(S1, S2: string): integer;
      begin
        UpperVar(S1);
        UpperVar(S2);
        Result:=CompareStr(S1,S2);
      end;


    function CompareVersionStrings(s1,s2: string): longint;
      var
        start1, start2,
        i1, i2,
        num1,num2,
        res,
        err        : longint;
      begin
        i1:=1;
        i2:=1;
        repeat
          start1:=i1;
          start2:=i2;
          while (i1<=length(s1)) and
                (s1[i1] in ['0'..'9']) do
             inc(i1);
          while (i2<=length(s2)) and
                (s2[i2] in ['0'..'9']) do
             inc(i2);
          { one of the strings misses digits -> other is the largest version }
          if i1=start1 then
            if i2=start2 then
              exit(0)
            else
              exit(-1)
          else if i2=start2 then
            exit(1);
          { get version number part }
          val(copy(s1,start1,i1-start1),num1,err);
          val(copy(s2,start2,i2-start2),num2,err);
          { different -> done }
          res:=num1-num2;
          if res<>0 then
            exit(res);
          { if one of the two is at the end while the other isn't, add a '.0' }
          if (i1>length(s1)) and
             (i2<=length(s2)) then
            s1:=s1+'.0'
          else if i2>length(s2) then
            s2:=s2+'.0';
          { compare non-numerical characters normally }
          while (i1<=length(s1)) and
                not(s1[i1] in ['0'..'9']) and
                (i2<=length(s2)) and
                not(s2[i2] in ['0'..'9']) do
            begin
              res:=ord(s1[i1])-ord(s2[i2]);
              if res<>0 then
                exit(res);
              inc(i1);
              inc(i2);
            end;
          { both should be digits again now, otherwise pick the one with the
            digits as the largest (it more likely means that the input was
            ill-formatted though) }
          if (i1<=length(s1)) and
             not(s1[i1] in ['0'..'9']) then
            exit(-1);
          if (i2<=length(s2)) and
             not(s2[i2] in ['0'..'9']) then
            exit(1);
        until false;
      end;


{*****************************************************************************
                               Ansistring (PChar+Length)
*****************************************************************************}

    procedure ansistringdispose(var p : pchar;length : longint);
      begin
         if assigned(p) then
           begin
             freemem(p);
             p:=nil;
           end;
      end;


    { enable ansistring comparison }
    { 0 means equal }
    { 1 means p1 > p2 }
    { -1 means p1 < p2 }
    function compareansistrings(p1,p2 : pchar;length1,length2 :  longint) : longint;
      var
         i,j : longint;
      begin
         compareansistrings:=0;
         j:=min(length1,length2);
         i:=0;
         while (i<j) do
          begin
            if p1[i]>p2[i] then
             begin
               compareansistrings:=1;
               exit;
             end
            else
             if p1[i]<p2[i] then
              begin
                compareansistrings:=-1;
                exit;
              end;
            inc(i);
          end;
         if length1>length2 then
          compareansistrings:=1
         else
          if length1<length2 then
           compareansistrings:=-1;
      end;


    function concatansistrings(p1,p2 : pchar;length1,length2 : longint) : pchar;
      var
         p : pchar;
      begin
         getmem(p,length1+length2+1);
         move(p1[0],p[0],length1);
         move(p2[0],p[length1],length2+1);
         concatansistrings:=p;
      end;


{*****************************************************************************
                       Ultra basic KISS Lzw (de)compressor
*****************************************************************************}

    {This is an extremely basic implementation of the Lzw algorithm. It
     compresses 7-bit ASCII strings into 8-bit compressed strings.
     The Lzw dictionary is preinitialized with 0..127, therefore this
     part of the dictionary does not need to be stored in the arrays.
     The Lzw code size is allways 8 bit, so we do not need complex code
     that can write partial bytes.}

    function minilzw_encode(const s:string):string;

    var t,u,i:byte;
        c:char;
        data:array[128..255] of char;
        previous:array[128..255] of byte;
        lzwptr:byte;
        next_avail:set of 0..255;

    label l1;

    begin
      minilzw_encode:='';
      fillchar(data,sizeof(data),#0);
      fillchar(previous,sizeof(previous),#0);
      if s<>'' then
        begin
          lzwptr:=127;
          t:=byte(s[1]);
          i:=2;
          u:=128;
          next_avail:=[];
          while i<=length(s) do
            begin
              c:=s[i];
              if not(t in next_avail) or (u>lzwptr) then goto l1;
              while (previous[u]<>t) or (data[u]<>c) do
                begin
                  inc(u);
                  if u>lzwptr then goto l1;
                end;
              t:=u;
              inc(i);
              continue;
            l1:
              {It's a pity that we still need those awfull tricks
               with this modern compiler. Without this performance
               of the entire procedure drops about 3 times.}
              inc(minilzw_encode[0]);
              minilzw_encode[length(minilzw_encode)]:=char(t);
              if lzwptr=255 then
                begin
                  lzwptr:=127;
                  next_avail:=[];
                end
              else
                begin
                  inc(lzwptr);
                  data[lzwptr]:=c;
                  previous[lzwptr]:=t;
                  include(next_avail,t);
                end;
              t:=byte(c);
              u:=128;
              inc(i);
            end;
          inc(minilzw_encode[0]);
          minilzw_encode[length(minilzw_encode)]:=char(t);
        end;
    end;

    function minilzw_decode(const s:string):string;

    var oldc,newc,c:char;
        i,j:byte;
        data:array[128..255] of char;
        previous:array[128..255] of byte;
        lzwptr:byte;
        t:string;

    begin
      minilzw_decode:='';
      fillchar(data,sizeof(data),#0);
      fillchar(previous,sizeof(previous),#0);
      if s<>'' then
        begin
          lzwptr:=127;
          oldc:=s[1];
          c:=oldc;
          i:=2;
          minilzw_decode:=oldc;
          while i<=length(s) do
            begin
              newc:=s[i];
              if byte(newc)>lzwptr then
                begin
                  t:=c;
                  c:=oldc;
                end
              else
                begin
                  c:=newc;
                  t:='';
                end;
              while c>=#128 do
                begin
                  inc(t[0]);
                  t[length(t)]:=data[byte(c)];
                  byte(c):=previous[byte(c)];
                end;
              inc(minilzw_decode[0]);
              minilzw_decode[length(minilzw_decode)]:=c;
              for j:=length(t) downto 1 do
                begin
                  inc(minilzw_decode[0]);
                  minilzw_decode[length(minilzw_decode)]:=t[j];
                end;
              if lzwptr=255 then
                lzwptr:=127
              else
                begin
                  inc(lzwptr);
                  previous[lzwptr]:=byte(oldc);
                  data[lzwptr]:=c;
                end;
              oldc:=newc;
              inc(i);
            end;
        end;
    end;


    procedure defaulterror(i:longint);
      begin
        writeln('Internal error ',i);
        runerror(255);
      end;

    Function Nextafter(x,y:double):double;
    // Returns the double precision number closest to x in
    // the direction toward y.

    // Initial direct translation by Soeren Haastrup from
    // www.netlib.org/fdlibm/s_nextafter.c according to
    // ====================================================
    // Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
    // Developed at SunSoft, a Sun Microsystems, Inc. business.
    // Permission to use, copy, modify, and distribute this
    // software is freely granted, provided that this notice
    // is preserved.
    // ====================================================
    // and with all signaling policies preserved as is.

    type
      {$if defined(ENDIAN_LITTLE) and not defined(FPC_DOUBLE_HILO_SWAPPED)}
        twoword=record
                  lo,hi:longword; // Little Endian split of a double.
                end;
      {$else}
        twoword=record
                  hi,lo:longword; // Big Endian split of a double.
                end;
      {$endif}

    var
        hx,hy,ix,iy:longint;
        lx,ly:longword;

    Begin
    hx:=twoword(x).hi;    // high and low words of x and y
    lx:=twoword(x).lo;
    hy:=twoword(y).hi;
    ly:=twoword(y).lo;
    ix:=hx and $7fffffff;  // absolute values
    iy:=hy and $7fffffff;

    // Case x=NAN or y=NAN

    if ( (ix>=$7ff00000) and ((longword(ix-$7ff00000) or lx) <> 0) )
        or ( (iy>=$7ff00000) and ((longword(iy-$7ff00000) OR ly) <> 0) )
    then exit(x+y);

    // Case x=y

    if x=y then exit(x); // (implies Nextafter(0,-0) is 0 and not -0...)

    // Case x=0

    if (longword(ix) or lx)=0
    then begin
          twoword(x).hi:=hy and $80000000;  // return +-minimalSubnormal
          twoword(x).lo:=1;
          y:=x*x;    // set underflow flag (ignored in FPC as default)
          if y=x
          then exit(y)
          else exit(x);
        end;

    // all other cases

    if hx>=0  // x>0
    then begin
          if (hx>hy) or ( (hx=hy) and (lx>ly) ) // x>y , return x-ulp
          then begin
                if (lx=0) then hx:=hx-1;
                lx:=lx-1;
              end
          else begin                      // x<y, return x+ulp
                lx:=lx+1;
                if lx=0 then hx:=hx+1;
              end
        end
    else begin // x<0
          if (hy>=0) or (hx>=hy) or ( (hx=hy) and (lx>ly)) // x<y, return x-ulp
          then begin
                if (lx=0) then hx:=hx-1;
                lx:=lx-1;
              end
          else begin            // x>y , return x+ulp
                lx:=lx+1;
                if lx=0 then hx:=hx+1;
              end
        end;

    // finally check if overflow or underflow just happend

    hy:=hx and $7ff00000;
    if (hy>= $7ff00000) then exit(x+x); // overflow and signal
    if (hy<$0010000)                    // underflow
    then begin
          y:=x*x;              // raise underflow flag
          if y<>x
          then begin
                twoword(y).hi:=hx;
                twoword(y).lo:=lx;
                exit(y);
              end
        end;

    twoword(x).hi:=hx;
    twoword(x).lo:=lx;
    nextafter:=x;

    end;


    function LengthUleb128(a: qword) : byte;
      begin
        result:=0;
        repeat
          a := a shr 7;
          inc(result);
          if a=0 then
            break;
        until false;
      end;


    function LengthSleb128(a: int64) : byte;
      var
        b: byte;
        more: boolean;
      begin
        more := true;
        result:=0;
        repeat
          b := a and $7f;
          a := SarInt64(a, 7);

          if (
            ((a = 0) and (b and $40 = 0)) or
            ((a = -1) and (b and $40 <> 0))
          ) then
            more := false;
          inc(result);
          if not(more) then
            break;
        until false;
      end;


    function EncodeUleb128(a: qword;out buf;len : byte) : byte;
      var
        b: byte;
        pbuf : pbyte;
      begin
        result:=0;
        pbuf:=@buf;
        repeat
          b := a and $7f;
          a := a shr 7;
          if a<>0 then
            b := b or $80;
          pbuf^:=b;
          inc(pbuf);
          inc(result);
          if (a=0) and  (result>=len) then
            break;
        until false;
      end;


    function EncodeSleb128(a: int64;out buf;len : byte) : byte;
      var
        b: byte;
        more: boolean;
        pbuf : pbyte;
      begin
        more := true;
        result:=0;
        pbuf:=@buf;
        repeat
          b := a and $7f;
          a := SarInt64(a, 7);

          if (result+1>=len) and (
            ((a = 0) and (b and $40 = 0)) or
            ((a = -1) and (b and $40 <> 0))
          ) then
            more := false
          else
            b := b or $80;
          pbuf^:=b;
          inc(pbuf);
          inc(result);
        until not more;
      end;


initialization
  internalerrorproc:=@defaulterror;
  initupperlower;
end.
