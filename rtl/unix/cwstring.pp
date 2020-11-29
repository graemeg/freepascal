{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Florian Klaempfl,
    member of the Free Pascal development team.

    libc based wide string support

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 **********************************************************************}

{$mode objfpc}
{$inline on}
{$implicitexceptions off}

unit cwstring;

interface

procedure SetCWidestringManager;

implementation

{$linklib c}

// Linux (and maybe glibc platforms in general), have iconv in glibc.
{$if defined(linux) or defined(solaris)}
  {$define iconv_is_in_libc}
{$endif}

{$ifdef netbsd}
  {$ifndef DISABLE_ICONV_LIBC}
    {$define iconv_is_in_libc}
  {$endif}
{$endif}

{$ifndef iconv_is_in_libc}
 {$if defined(haiku)}
   {$linklib textencoding}
 {$else}
   {$linklib iconv}
 {$endif}
 {$define useiconv}
{$endif not iconv_is_in_libc}

{$i rtldefs.inc}

Uses
  BaseUnix,
  ctypes,
  unix,
  unixtype,
  initc,
  dynlibs,
  unixcp;

Const
{$ifndef useiconv}
    libiconvname='c';  // is in libc under Linux.
    libprefix='lib';
{$else}
  {$ifdef haiku}
    libiconvname='textencoding';  // is in libtextencoding under Haiku
    libprefix='lib';
  {$else}
    {$if defined(darwin) or defined(aix)}
      libiconvname='libiconv';
      libprefix='';
    {$else}
      libiconvname='iconv';
      libprefix='lib';
    {$endif}
  {$endif}
{$endif}

{ helper functions from libc }
function towlower(__wc:wint_t):wint_t;cdecl;external clib name 'towlower';
function towupper(__wc:wint_t):wint_t;cdecl;external clib name 'towupper';

function wcscoll (__s1:pwchar_t; __s2:pwchar_t):cint;cdecl;external clib name 'wcscoll';
function strcoll (__s1:pchar; __s2:pchar):cint;cdecl;external clib name 'strcoll';
{$ifdef netbsd}
  { NetBSD has a new setlocale function defined in /usr/include/locale.h
    that should be used }
function setlocale(category: cint; locale: pchar): pchar; cdecl; external clib name '__setlocale50';
{$else}
function setlocale(category: cint; locale: pchar): pchar; cdecl; external clib name 'setlocale';
{$endif}
{$if not(defined(beos) and not defined(haiku))}
function mbrtowc(pwc: pwchar_t; const s: pchar; n: size_t; ps: pmbstate_t): size_t; cdecl; external clib name 'mbrtowc';
function wcrtomb(s: pchar; wc: wchar_t; ps: pmbstate_t): size_t; cdecl; external clib name 'wcrtomb';
function mbrlen(const s: pchar; n: size_t; ps: pmbstate_t): size_t; cdecl; external clib name 'mbrlen';
{$else beos}
function mbtowc(pwc: pwchar_t; const s: pchar; n: size_t): size_t; cdecl; external clib name 'mbtowc';
function wctomb(s: pchar; wc: wchar_t): size_t; cdecl; external clib name 'wctomb';
function mblen(const s: pchar; n: size_t): size_t; cdecl; external clib name 'mblen';
{$endif beos}


const
{$if defined(linux)}
  __LC_CTYPE = 0;
  LC_ALL = 6;
  _NL_CTYPE_CLASS = (__LC_CTYPE shl 16);
  _NL_CTYPE_CODESET_NAME = (_NL_CTYPE_CLASS)+14;
  CODESET = _NL_CTYPE_CODESET_NAME;
{$elseif defined(darwin)}
  CODESET = 0;
  LC_ALL = 0;
{$elseif defined(FreeBSD)} // actually FreeBSD5. internationalisation is afaik not default on 4.
  __LC_CTYPE = 0;
  LC_ALL = 0;
  _NL_CTYPE_CLASS = (__LC_CTYPE shl 16);
  _NL_CTYPE_CODESET_NAME = (_NL_CTYPE_CLASS)+14;
  CODESET = 0; // _NL_CTYPE_CODESET_NAME;
{$elseif defined(solaris)}
  {$define ACCEPT_646}
  CODESET=49;
  LC_ALL = 6;
{$elseif defined(beos)}
  {$ifdef haiku}
  CODESET= 0; // Checked for Haiku
  LC_ALL = 0; // Checked for Haiku
  {$else}
  {$warning check correct value for BeOS}
  CODESET=49;
  LC_ALL = 6; // Checked for BeOS
  {$endif}
  ESysEILSEQ = EILSEQ;
{$elseif defined(OpenBSD)}
  {$define ACCEPT_646}
  CODESET = 51;
  LC_ALL = 0;
{$elseif defined(NetBSD)}
  {$define ACCEPT_646}
  CODESET = 51;
  LC_ALL = 0;
{$elseif defined(aix)}
  CODESET = 49;
  LC_ALL = -1;
{$elseif defined(dragonfly)}
  CODESET = 0;
  LC_ALL = 0;
  __LC_CTYPE = 0;
  _NL_CTYPE_CLASS = (__LC_CTYPE shl 16);
  _NL_CTYPE_CODESET_NAME = (_NL_CTYPE_CLASS)+14;
{$else not aix}
{$error lookup the value of CODESET in /usr/include/langinfo.h, and the value of LC_ALL in /usr/include/locale.h for your OS }
// and while doing it, check if iconv is in libc, and if the symbols are prefixed with iconv_ or libiconv_
{$endif}

{ unicode encoding name }
{$ifdef FPC_LITTLE_ENDIAN}
  unicode_encoding2 = 'UTF-16LE';
  unicode_encoding4 = 'UCS-4LE';
{$else  FPC_LITTLE_ENDIAN}
{$ifdef AIX}
  unicode_encoding2 = 'UTF-16';
{$else AIX}
  unicode_encoding2 = 'UTF-16BE';
  unicode_encoding4 = 'UCS-4BE';
{$endif AIX}
{$endif  FPC_LITTLE_ENDIAN}

{ en_US.UTF-8 needs maximally 6 chars, UCS-4/UTF-32 needs 4   }
{ -> 10 should be enough? Should actually use MB_CUR_MAX, but }
{ that's a libc macro mapped to internal functions/variables  }
{ and thus not a stable external API on systems where libc    }
{ breaks backwards compatibility every now and then           }
  MB_CUR_MAX = 10;

{ Requests for iconvctl }
  ICONV_TRIVIALP          = 0; // int *argument
  ICONV_GET_TRANSLITERATE = 1; // int *argument
  ICONV_SET_TRANSLITERATE = 2; // const int *argument
  ICONV_GET_DISCARD_ILSEQ = 3; // int *argument
  ICONV_SET_DISCARD_ILSEQ = 4; // const int *argument
  ICONV_SET_HOOKS         = 5; // const struct iconv_hooks *argument
  ICONV_SET_FALLBACKS     = 6; // const struct iconv_fallbacks *argument

type
  piconv_t = ^iconv_t;
  iconv_t = pointer;
  nl_item = cint;

{$ifdef haiku}
  function nl_langinfo(__item:nl_item):pchar;cdecl;external 'root' name 'nl_langinfo';
{$else}
  {$ifndef beos}
  function nl_langinfo(__item:nl_item):pchar;cdecl;external libiconvname name 'nl_langinfo';
  {$endif}
{$endif}

{$if (not defined(bsd) and not defined(beos)) or defined(iconv_is_in_libc) or (defined(darwin) and not defined(cpupowerpc32))}
function iconv_open(__tocode:pchar; __fromcode:pchar):iconv_t;cdecl;external libiconvname name 'iconv_open';
function iconv(__cd:iconv_t; __inbuf:ppchar; __inbytesleft:psize_t; __outbuf:ppchar; __outbytesleft:psize_t):size_t;cdecl;external libiconvname name 'iconv';
function iconv_close(__cd:iconv_t):cint;cdecl;external libiconvname name 'iconv_close';
const
  iconvctlname='iconvctl';
{$else}
function iconv_open(__tocode:pchar; __fromcode:pchar):iconv_t;cdecl;external libiconvname name 'libiconv_open';
function iconv(__cd:iconv_t; __inbuf:ppchar; __inbytesleft:psize_t; __outbuf:ppchar; __outbytesleft:psize_t):size_t;cdecl;external libiconvname name 'libiconv';
function iconv_close(__cd:iconv_t):cint;cdecl;external libiconvname name 'libiconv_close';
const
  iconvctlname='libiconvctl';
{$endif}
var
  iconvctl:function(__cd:iconv_t; __request:cint; __argument:pointer):cint;cdecl;

procedure fpc_rangeerror; [external name 'FPC_RANGEERROR'];


threadvar
  iconv_ansi2wide,
  iconv_wide2ansi : iconv_t;
  { since we cache the iconv_t converters, we have to do the same
    for the DefaultSystemCodePage variable since if it changes, we
    have to re-initialize the converters too. We can't do that via
    a callback in the widestring manager because DefaultSystemCodePage
    is not a threadvar and we can't automatically change this in all
    threads }
  current_DefaultSystemCodePage: TSystemCodePage;


procedure InitThread;
var
  transliterate: cint;
{$if not(defined(darwin) and (defined(cpuarm) or defined(cpuaarch64))) and not defined(iphonesim)}
  iconvindex: longint;
{$endif}
  iconvname, toencoding: rawbytestring;
begin
  current_DefaultSystemCodePage:=DefaultSystemCodePage;
{$if declared(iconvindex)}
  iconvindex:=GetCodepageData(DefaultSystemCodePage);
  if iconvindex<>-1 then
    iconvname:=UnixCpMap[iconvindex].name
  else
    { default to UTF-8 on Unix platforms }
    iconvname:='UTF-8';
{$else}
  { Unix locale settings are ignored on iPhoneOS/iPhoneSimulator }
  iconvname:='UTF-8';
{$endif}
  toencoding:=iconvname;
  if not assigned(iconvctl) then
    toencoding:=toencoding+'//TRANSLIT';
  iconv_wide2ansi:=iconv_open(pchar(toencoding),unicode_encoding2);
  iconv_ansi2wide:=iconv_open(unicode_encoding2,pchar(iconvname));
  if assigned(iconvctl) and
     (iconv_wide2ansi<>iconv_t(-1)) then
  begin
    transliterate:=1;
    iconvctl(iconv_wide2ansi,ICONV_SET_TRANSLITERATE,@transliterate);
  end;
end;


procedure FiniThread;
begin
  if (iconv_wide2ansi <> iconv_t(-1)) then
    iconv_close(iconv_wide2ansi);
  if (iconv_ansi2wide <> iconv_t(-1)) then
    iconv_close(iconv_ansi2wide);
end;


{$if defined(beos) and not defined(haiku)}
function nl_langinfo(__item:nl_item):pchar;
begin
  {$warning TODO BeOS nl_langinfo or more uptodate port of iconv...}
  // Now implement the minimum required to correctly initialize WideString support
  case __item of
    CODESET : Result := 'UTF-8'; // BeOS use UTF-8
    else
    begin
      Assert(False, 'nl_langinfo was called with an unknown nl_item value');
      Result := '';
    end;
  end;
end;
{$endif}


function open_iconv_for_cps(cp: TSystemCodePage; const otherencoding: pchar; cp_is_from: boolean): iconv_t;
  var
    iconvindex: longint;
    toencoding: rawbytestring;
    transliterate: cint;
  begin
    { TODO: add caching (then we also don't need separate code for
      the default system page and other ones)

      -- typecasting an ansistring function result to pchar is
        unsafe normally, but these are constant strings -> no
        problem }
    open_iconv_for_cps:=iconv_t(-1);
    iconvindex:=GetCodepageData(cp);
    if iconvindex=-1 then
      exit;
    repeat
      if cp_is_from then
        open_iconv_for_cps:=iconv_open(otherencoding,pchar(UnixCpMap[iconvindex].name))
      else
      begin
        toencoding:=UnixCpMap[iconvindex].name;
        if not assigned(iconvctl) then
          toencoding:=toencoding+'//TRANSLIT';
        open_iconv_for_cps:=iconv_open(pchar(toencoding),otherencoding);
      end;
      inc(iconvindex);
    until (open_iconv_for_cps<>iconv_t(-1)) or
          (iconvindex>high(UnixCpMap)) or
          (UnixCpMap[iconvindex].cp<>cp);
    if not cp_is_from and
      (open_iconv_for_cps<>iconv_t(-1)) and
      assigned(iconvctl) then
    begin
      transliterate:=1;
      iconvctl(open_iconv_for_cps,ICONV_SET_TRANSLITERATE,@transliterate);
    end;
  end;


{$ifdef aix}
{$i cwstraix.inc}
{$endif aix}

procedure Wide2AnsiMove(source:pwidechar; var dest:RawByteString; cp:TSystemCodePage; len:SizeInt);
  var
    outlength,
    outoffset,
    srclen,
    outleft : size_t;
    use_iconv: iconv_t;
    srcpos : pwidechar;
    destpos: pchar;
    mynil : pchar;
    my0 : size_t;
    err : longint;
    transliterate: cint;
    free_iconv: boolean;
{$ifdef aix}
    intermediate: rawbytestring;
{$endif aix}
  begin
{$ifdef aix}
    { AIX libiconv does not support converting cp866 to anything else except
      for iso-8859-5 -> always first convert to iso-8859-5, then to UTF-16 }
    if cp=866 then
      begin
        Wide2AnsiMove(source,intermediate,28595,len);
        if handle_aix_intermediate(pchar(intermediate),28595,cp,dest,len) then
          exit;
      end;
{$endif aix}
    if (cp=DefaultSystemCodePage) then
      begin
        { update iconv converter in case the DefaultSystemCodePage has been
          changed }
        if current_DefaultSystemCodePage<>DefaultSystemCodePage then
          begin
            FiniThread;
            InitThread;
          end;
        use_iconv:=iconv_wide2ansi;
        free_iconv:=false;
      end
    else
      begin
        use_iconv:=open_iconv_for_cps(cp,unicode_encoding2,false);
        if (use_iconv<>iconv_t(-1)) and
           assigned(iconvctl) then
        begin
          transliterate:=1;
          iconvctl(use_iconv,ICONV_SET_TRANSLITERATE,@transliterate);
        end;
        free_iconv:=true;
      end;
    { unsupported encoding -> default move }
    if use_iconv=iconv_t(-1) then
      begin
        DefaultUnicode2AnsiMove(source,dest,DefaultSystemCodePage,len);
        exit;
      end;
    mynil:=nil;
    my0:=0;
    { rought estimation }
    setlength(dest,len*3);
    outlength:=len*3;
    srclen:=len*2;
    srcpos:=source;
    destpos:=pchar(dest);
    outleft:=outlength;
    while iconv(use_iconv,ppchar(@srcpos),@srclen,@destpos,@outleft)=size_t(-1) do
      begin
        err:=fpgetCerrno;
        case err of
          { last character is incomplete sequence }
          ESysEINVAL,
          { incomplete sequence in the middle }
          ESysEILSEQ:
            begin
              { skip and set to '?' }
              inc(srcpos);
              dec(srclen,2);
              destpos^:='?';
              inc(destpos);
              dec(outleft);
              { reset }
              iconv(use_iconv,@mynil,@my0,@mynil,@my0);
              if err=ESysEINVAL then
                break;
            end;
          ESysE2BIG:
            begin
              outoffset:=destpos-pchar(dest);
              { extend }
              setlength(dest,outlength+len*3);
              inc(outleft,len*3);
              inc(outlength,len*3);
              { string could have been moved }
              destpos:=pchar(dest)+outoffset;
            end;
          else
            runerror(231);
        end;
      end;
    // truncate string
    setlength(dest,length(dest)-outleft);
    SetCodePage(dest,cp,false);
    if free_iconv then
      iconv_close(use_iconv);
  end;


procedure Ansi2WideMove(source:pchar; cp:TSystemCodePage; var dest:widestring; len:SizeInt);
  var
    outlength,
    outoffset,
    outleft : size_t;
    use_iconv: iconv_t;
    srcpos,
    destpos: pchar;
    mynil : pchar;
    my0 : size_t;
    err: cint;
    iconvindex: longint;
    free_iconv: boolean;
{$ifdef aix}
    intermediate: rawbytestring;
{$endif aix}
  begin
{$ifdef aix}
    { AIX libiconv does not support converting cp866 to anything else except
      for iso-8859-5 -> always first convert to iso-8859-5, then to UTF-16 }
    if cp=866 then
      begin
        if handle_aix_intermediate(source,cp,cp,intermediate,len) then
          source:=pchar(intermediate);
      end;
{$endif aix}
    if (cp=DefaultSystemCodePage) then
      begin
        { update iconv converter in case the DefaultSystemCodePage has been
          changed }
        if current_DefaultSystemCodePage<>DefaultSystemCodePage then
          begin
            FiniThread;
            InitThread;
          end;
        use_iconv:=iconv_ansi2wide;
        free_iconv:=false;
      end
    else
      begin
        { TODO: add caching (then we also don't need separate code for
          the default system page and other ones)

          -- typecasting an ansistring function result to pchar is
            unsafe normally, but these are constant strings -> no
            problem }
        use_iconv:=open_iconv_for_cps(cp,unicode_encoding2,true);
        free_iconv:=true;
      end;
    { unsupported encoding -> default move }
    if use_iconv=iconv_t(-1) then
      begin
        DefaultAnsi2UnicodeMove(source,DefaultSystemCodePage,dest,len);
        exit;
      end;
    mynil:=nil;
    my0:=0;
    // extra space
    outlength:=len+1;
    setlength(dest,outlength);
    srcpos:=source;
    destpos:=pchar(dest);
    outleft:=outlength*2;
    while iconv(use_iconv,@srcpos,psize(@len),@destpos,@outleft)=size_t(-1) do
      begin
        err:=fpgetCerrno;
        case err of
         ESysEINVAL,
         ESysEILSEQ:
            begin
              { skip and set to '?' }
              inc(srcpos);
              dec(len);
              pwidechar(destpos)^:='?';
              inc(destpos,2);
              dec(outleft,2);
              { reset }
              iconv(use_iconv,@mynil,@my0,@mynil,@my0);
              if err=ESysEINVAL then
                break;
            end;
          ESysE2BIG:
            begin
              outoffset:=destpos-pchar(dest);
              { extend }
              setlength(dest,outlength+len);
              inc(outleft,len*2);
              inc(outlength,len);
              { string could have been moved }
              destpos:=pchar(dest)+outoffset;
            end;
          else
            runerror(231);
        end;
      end;
    // truncate string
    setlength(dest,length(dest)-outleft div 2);
    if free_iconv then
      iconv_close(use_iconv);
  end;


function LowerWideString(const s : WideString) : WideString;
  var
    i : SizeInt;
  begin
    SetLength(result,length(s));
    for i:=0 to length(s)-1 do
      pwidechar(result)[i]:=WideChar(towlower(wint_t(s[i+1])));
  end;


function UpperWideString(const s : WideString) : WideString;
  var
    i : SizeInt;
  begin
    SetLength(result,length(s));
    for i:=0 to length(s)-1 do
      pwidechar(result)[i]:=WideChar(towupper(wint_t(s[i+1])));
  end;


procedure EnsureAnsiLen(var S: AnsiString; const len: SizeInt); inline;
begin
  if (len>length(s)) then
    if (length(s) < 10*256) then
      setlength(s,length(s)+10)
    else
      setlength(s,length(s)+length(s) shr 8);
end;


procedure ConcatCharToAnsiStr(const c: char; var S: AnsiString; var index: SizeInt);
begin
  EnsureAnsiLen(s,index);
  pchar(@s[index])^:=c;
  inc(index);
end;


{ concatenates an utf-32 char to a widestring. S *must* be unique when entering. }
{$if not(defined(beos) and not defined(haiku))}
procedure ConcatUTF32ToAnsiStr(const nc: wint_t; var S: AnsiString; var index: SizeInt; var mbstate: mbstate_t);
{$else not beos}
procedure ConcatUTF32ToAnsiStr(const nc: wint_t; var S: AnsiString; var index: SizeInt);
{$endif beos}
var
  p     : pchar;
  mblen : size_t;
begin
  { we know that s is unique -> avoid uniquestring calls}
  p:=@s[index];
  if (nc<=127) then
    ConcatCharToAnsiStr(char(nc),s,index)
  else
    begin
      EnsureAnsiLen(s,index+MB_CUR_MAX);
{$if not(defined(beos) and not defined(haiku))}
      mblen:=wcrtomb(p,wchar_t(nc),@mbstate);
{$else not beos}
      mblen:=wctomb(p,wchar_t(nc));
{$endif not beos}
      if (mblen<>size_t(-1)) then
        inc(index,mblen)
      else
        begin
          { invalid wide char }
          p^:='?';
          inc(index);
        end;
    end;
end;


function LowerAnsiString(const s : AnsiString) : AnsiString;
  var
    i, slen,
    resindex : SizeInt;
    mblen    : size_t;
{$if not(defined(beos) and not defined(haiku))}
    ombstate,
    nmbstate : mbstate_t;
{$endif beos}
    wc       : wchar_t;
  begin
{$if not(defined(beos) and not defined(haiku))}
    fillchar(ombstate,sizeof(ombstate),0);
    fillchar(nmbstate,sizeof(nmbstate),0);
{$endif beos}
    slen:=length(s);
    SetLength(result,slen+10);
    i:=1;
    resindex:=1;
    while (i<=slen) do
      begin
        if (s[i]<=#127) then
          begin
            wc:=wchar_t(s[i]);
            mblen:= 1;
          end
        else
{$if not(defined(beos) and not defined(haiku))}
          mblen:=mbrtowc(@wc, pchar(@s[i]), slen-i+1, @ombstate);
{$else not beos}
          mblen:=mbtowc(@wc, pchar(@s[i]), slen-i+1);
{$endif not beos}
        case mblen of
          size_t(-2):
            begin
              { partial invalid character, copy literally }
              while (i<=slen) do
                begin
                  ConcatCharToAnsiStr(s[i],result,resindex);
                  inc(i);
                end;
            end;
          size_t(-1), 0:
            begin
              { invalid or null character }
              ConcatCharToAnsiStr(s[i],result,resindex);
              inc(i);
            end;
          else
            begin
              { a valid sequence }
              { even if mblen = 1, the lowercase version may have a }
              { different length                                     }
              { We can't do anything special if wchar_t is 16 bit... }
{$if not(defined(beos) and not defined(haiku))}
              ConcatUTF32ToAnsiStr(towlower(wint_t(wc)),result,resindex,nmbstate);
{$else not beos}
              ConcatUTF32ToAnsiStr(towlower(wint_t(wc)),result,resindex);
{$endif not beos}
              inc(i,mblen);
            end;
          end;
      end;
    SetLength(result,resindex-1);
  end;


function UpperAnsiString(const s : AnsiString) : AnsiString;
  var
    i, slen,
    resindex : SizeInt;
    mblen    : size_t;
{$if not(defined(beos) and not defined(haiku))}
    ombstate,
    nmbstate : mbstate_t;
{$endif beos}
    wc       : wchar_t;
  begin
{$if not(defined(beos) and not defined(haiku))}
    fillchar(ombstate,sizeof(ombstate),0);
    fillchar(nmbstate,sizeof(nmbstate),0);
{$endif beos}
    slen:=length(s);
    SetLength(result,slen+10);
    i:=1;
    resindex:=1;
    while (i<=slen) do
      begin
        if (s[i]<=#127) then
          begin
            wc:=wchar_t(s[i]);
            mblen:= 1;
          end
        else
{$if not(defined(beos) and not defined(haiku))}
          mblen:=mbrtowc(@wc, pchar(@s[i]), slen-i+1, @ombstate);
{$else not beos}
          mblen:=mbtowc(@wc, pchar(@s[i]), slen-i+1);
{$endif beos}
        case mblen of
          size_t(-2):
            begin
              { partial invalid character, copy literally }
              while (i<=slen) do
                begin
                  ConcatCharToAnsiStr(s[i],result,resindex);
                  inc(i);
                end;
            end;
          size_t(-1), 0:
            begin
              { invalid or null character }
              ConcatCharToAnsiStr(s[i],result,resindex);
              inc(i);
            end;
          else
            begin
              { a valid sequence }
              { even if mblen = 1, the uppercase version may have a }
              { different length                                     }
              { We can't do anything special if wchar_t is 16 bit... }
{$if not(defined(beos) and not defined(haiku))}
              ConcatUTF32ToAnsiStr(towupper(wint_t(wc)),result,resindex,nmbstate);
{$else not beos}
              ConcatUTF32ToAnsiStr(towupper(wint_t(wc)),result,resindex);
{$endif not beos}
              inc(i,mblen);
            end;
          end;
      end;
    SetLength(result,resindex-1);
  end;

function WideStringToUCS4StringNoNulls(const s : WideString) : UCS4String;
  var
    i, slen,
    destindex : SizeInt;
    uch       : UCS4Char;
  begin
    slen:=length(s);
    setlength(result,slen+1);
    i:=1;
    destindex:=0;
    while (i<=slen) do
      begin
        uch:=UCS4Char(s[i]);
        if (uch=0) then
          result[destindex]:=32
        else if (uch<=$d7ff) or (uch>=$e000) then
          result[destindex]:=uch
        else if (uch<=$dbff) and
          (i<slen) and
          (s[i+1]>=#$dc00) and
          (s[i+1]<=#$dfff) then
          begin
            result[destindex]:=(UCS4Char(uch-$d7c0) shl 10)+(UCS4Char(s[i+1]) xor $dc00);
            inc(i);
          end
        else { invalid surrogate pair }
          result[destindex]:=uch;
        inc(i);
        inc(destindex);
      end;
    result[destindex]:=UCS4Char(0);
    { Trimming length in this particular case is just a waste of time,
      because result will be interpreted as null-terminated and discarded
      almost immediately }
  end;


function CompareWideString(const s1, s2 : WideString; Options : TCompareOptions) : PtrInt;
{$if not(defined (aix) and defined(cpupowerpc32))}
  var
    hs1,hs2 : UCS4String;
    us1,us2 : WideString;

  begin
    { wcscoll interprets null chars as end-of-string -> filter out }
    if coIgnoreCase in Options then
      begin
      us1:=UpperWideString(s1);
      us2:=UpperWideString(s2);
      end
    else
      begin
      us1:=s1;
      us2:=s2;
      end;
    hs1:=WideStringToUCS4StringNoNulls(us1);
    hs2:=WideStringToUCS4StringNoNulls(us2);
    result:=wcscoll(pwchar_t(hs1),pwchar_t(hs2));
  end;
{$else}
  { AIX/PPC32 has a 16 bit wchar_t }
  var
    i, len: longint;
    us1,us2 : WideString;
    hs1, hs2: array of widechar;
  begin
    if coIgnoreCase in Options then
      begin
      us1:=UpperWideString(s1);
      us2:=UpperWideString(s2);
      end
    else
      begin
      us1:=s1;
      us2:=s2;
      end;
    len:=length(us1);
    setlength(hs1,len+1);
    for i:=1 to len do
      if us1[i]<>#0 then
        hs1[i-1]:=us1[i]
      else
        hs1[i-1]:=#32;
    hs1[len]:=#0;

    len:=length(us2);
    setlength(hs2,len+1);
    for i:=1 to len do
      if us2[i]<>#0 then
        hs2[i-1]:=us2[i]
      else
        hs2[i-1]:=#32;
    hs2[len]:=#0;
    result:=wcscoll(pwchar_t(hs1),pwchar_t(hs2));
  end;
{$endif}



{ return value: number of code points in the string. Whenever an invalid
  code point is encountered, all characters part of this invalid code point
  are considered to form one "character" and the next character is
  considered to be the start of a new (possibly also invalid) code point }
function CharLengthPChar(const Str: PChar): PtrInt;
  var
    nextlen: ptrint;
    s: pchar;
{$if not(defined(beos) and not defined(haiku))}
    mbstate: mbstate_t;
{$endif not beos}
  begin
    result:=0;
    s:=str;
{$if not(defined(beos) and not defined(haiku))}
    fillchar(mbstate,sizeof(mbstate),0);
{$endif not beos}
    repeat
{$if defined(beos) and not defined(haiku)}
      nextlen:=ptrint(mblen(s,MB_CUR_MAX));
{$else beos}
      nextlen:=ptrint(mbrlen(s,MB_CUR_MAX,@mbstate));
{$endif beos}
      { skip invalid/incomplete sequences }
      if (nextlen<0) then
        nextlen:=1;
      inc(result,1);
      inc(s,nextlen);
    until (nextlen=0);
  end;


function CodePointLength(const Str: PChar; maxlookahead: ptrint): PtrInt;
{$if not(defined(beos) and not defined(haiku))}
  var
    mbstate: mbstate_t;
{$endif not beos}
  begin
{$if defined(beos) and not defined(haiku)}
    result:=ptrint(mblen(str,maxlookahead));
{$else beos}
    fillchar(mbstate,sizeof(mbstate),0);
    result:=ptrint(mbrlen(str,maxlookahead,@mbstate));
    { mbrlen can also return -2 for "incomplete but potially valid character
      and data has been processed" }
    if result<0 then
      result:=-1;
{$endif beos}
  end;


function StrCompAnsiIntern(s1,s2 : PChar; len1, len2: PtrInt; canmodifys1, canmodifys2: boolean): PtrInt;
  var
    a,b: pchar;
    i: PtrInt;
  begin
    if not(canmodifys1) then
      getmem(a,len1+1)
    else
      a:=s1;
    for i:=0 to len1-1 do
      if s1[i]<>#0 then
        a[i]:=s1[i]
      else
        a[i]:=#32;
    a[len1]:=#0;

    if not(canmodifys2) then
      getmem(b,len2+1)
    else
      b:=s2;
    for i:=0 to len2-1 do
      if s2[i]<>#0 then
        b[i]:=s2[i]
      else
        b[i]:=#32;
    b[len2]:=#0;
    result:=strcoll(a,b);
    if not(canmodifys1) then
      freemem(a);
    if not(canmodifys2) then
      freemem(b);
  end;


function CompareStrAnsiString(const s1, s2: ansistring): PtrInt;
  begin
    result:=StrCompAnsiIntern(pchar(s1),pchar(s2),length(s1),length(s2),false,false);
  end;


function StrCompAnsi(s1,s2 : PChar): PtrInt;
  begin
    result:=strcoll(s1,s2);
  end;


function AnsiCompareText(const S1, S2: ansistring): PtrInt;
  var
    a, b: AnsiString;
  begin
    a:=UpperAnsistring(s1);
    b:=UpperAnsistring(s2);
    result:=StrCompAnsiIntern(pchar(a),pchar(b),length(a),length(b),true,true);
  end;


function AnsiStrIComp(S1, S2: PChar): PtrInt;
  begin
    result:=AnsiCompareText(ansistring(s1),ansistring(s2));
  end;


function AnsiStrLComp(S1, S2: PChar; MaxLen: PtrUInt): PtrInt;
  var
    a, b: pchar;
begin
  if (maxlen=0) then
    exit(0);
  if (s1[maxlen]<>#0) then
    begin
      getmem(a,maxlen+1);
      move(s1^,a^,maxlen);
      a[maxlen]:=#0;
    end
  else
    a:=s1;
  if (s2[maxlen]<>#0) then
    begin
      getmem(b,maxlen+1);
      move(s2^,b^,maxlen);
      b[maxlen]:=#0;
    end
  else
    b:=s2;
  result:=StrCompAnsiIntern(a,b,maxlen,maxlen,a<>s1,b<>s2);
  if (a<>s1) then
    freemem(a);
  if (b<>s2) then
    freemem(b);
end;


function AnsiStrLIComp(S1, S2: PChar; MaxLen: PtrUInt): PtrInt;
  var
    a, b: ansistring;
begin
  if (maxlen=0) then
    exit(0);
  setlength(a,maxlen);
  move(s1^,a[1],maxlen);
  setlength(b,maxlen);
  move(s2^,b[1],maxlen);
  result:=AnsiCompareText(a,b);
end;


procedure ansi2pchar(const s: ansistring; const orgp: pchar; out p: pchar);
var
  newlen: sizeint;
begin
  newlen:=length(s);
  if newlen>strlen(orgp) then
    fpc_rangeerror;
  p:=orgp;
  if (newlen>0) then
    move(s[1],p[0],newlen);
  p[newlen]:=#0;
end;


function AnsiStrLower(Str: PChar): PChar;
var
  temp: ansistring;
begin
  temp:=loweransistring(str);
  ansi2pchar(temp,str,result);
end;


function AnsiStrUpper(Str: PChar): PChar;
var
  temp: ansistring;
begin
  temp:=upperansistring(str);
  ansi2pchar(temp,str,result);
end;


function envvarset(const varname: pchar): boolean;
var
  varval: pchar;
begin
  varval:=fpgetenv(varname);
  result:=
    assigned(varval) and
    (varval[0]<>#0);
end;


function GetStandardCodePage(const stdcp: TStandardCodePageEnum): TSystemCodePage;
var
  langinfo: pchar;
begin
{$ifdef FPCRTL_FILESYSTEM_UTF8}
  if stdcp=scpFileSystemSingleByte then
    begin
      result:=CP_UTF8;
      exit;
    end;
{$endif}
  { if none of the relevant LC_* environment variables are set, fall back to
    UTF-8 (this happens under some versions of OS X for GUI applications, which
    otherwise get CP_ASCII) }
  if envvarset('LC_ALL') or
     envvarset('LC_CTYPE') or
     envvarset('LANG') then
    begin
      langinfo:=nl_langinfo(CODESET);
      { there's a bug in the Mac OS X 10.5 libc (based on FreeBSD's)
        that causes it to return an empty string of UTF-8 locales
        -> patch up (and in general, UTF-8 is a good default on
        Unix platforms) }
      if not assigned(langinfo) or
         (langinfo^=#0) then
        langinfo:='UTF-8';
      Result:=GetCodepageByName(ansistring(langinfo));
    end
  else
    Result:=unixcp.GetSystemCodepage;
end;

{$ifdef FPC_HAS_CPSTRING}

procedure SetStdIOCodePage(var T: Text); inline;
begin
  case TextRec(T).Mode of
    fmInput:TextRec(T).CodePage:=GetStandardCodePage(scpConsoleInput);
    fmOutput:TextRec(T).CodePage:=GetStandardCodePage(scpConsoleOutput);
  end;
end;

procedure SetStdIOCodePages; inline;
begin
  SetStdIOCodePage(Input);
  SetStdIOCodePage(Output);
  SetStdIOCodePage(ErrOutput);
  SetStdIOCodePage(StdOut);
  SetStdIOCodePage(StdErr);
end;
{$endif FPC_HAS_CPSTRING}

var
  OrgWideStringManager: TUnicodeStringManager;

Procedure SetCWideStringManager;
Var
  CWideStringManager : TUnicodeStringManager;
begin
  GetUnicodeStringManager(OrgWideStringManager);
  CWideStringManager:=OrgWideStringManager;
  With CWideStringManager do
    begin
      Wide2AnsiMoveProc:=@Wide2AnsiMove;
      Ansi2WideMoveProc:=@Ansi2WideMove;

      UpperWideStringProc:=@UpperWideString;
      LowerWideStringProc:=@LowerWideString;

      CompareWideStringProc:=@CompareWideString;
//      CompareTextWideStringProc:=@CompareTextWideString;

      CharLengthPCharProc:=@CharLengthPChar;
      CodePointLengthProc:=@CodePointLength;

      UpperAnsiStringProc:=@UpperAnsiString;
      LowerAnsiStringProc:=@LowerAnsiString;
      CompareStrAnsiStringProc:=@CompareStrAnsiString;
      CompareTextAnsiStringProc:=@AnsiCompareText;
      StrCompAnsiStringProc:=@StrCompAnsi;
      StrICompAnsiStringProc:=@AnsiStrIComp;
      StrLCompAnsiStringProc:=@AnsiStrLComp;
      StrLICompAnsiStringProc:=@AnsiStrLIComp;
      StrLowerAnsiStringProc:=@AnsiStrLower;
      StrUpperAnsiStringProc:=@AnsiStrUpper;
      ThreadInitProc:=@InitThread;
      ThreadFiniProc:=@FiniThread;
      { Unicode }
      Unicode2AnsiMoveProc:=@Wide2AnsiMove;
      Ansi2UnicodeMoveProc:=@Ansi2WideMove;
      UpperUnicodeStringProc:=@UpperWideString;
      LowerUnicodeStringProc:=@LowerWideString;
      CompareUnicodeStringProc:=@CompareWideString;
      { CodePage }
      GetStandardCodePageProc:=@GetStandardCodePage;
    end;
  SetUnicodeStringManager(CWideStringManager);
end;

var
  iconvlib:TLibHandle;

initialization
  SetCWideStringManager;

  { you have to call setlocale(LC_ALL,'') to initialise the langinfo stuff  }
  { with the information from the environment variables according to POSIX  }
  { (some OSes do this automatically, but e.g. Darwin and Solaris don't)    }
  setlocale(LC_ALL,'');

  { load iconv library and iconvctl function }
  iconvlib:=LoadLibrary(libprefix+libiconvname+'.'+SharedSuffix);
  if iconvlib=0 then
    iconvlib:=LoadLibrary(libprefix+libiconvname+'.'+SharedSuffix+'.6');
  if iconvlib<>0 then
    pointer(iconvctl):=GetProcAddress(iconvlib,iconvctlname);

  { set the DefaultSystemCodePage }
  DefaultSystemCodePage:=GetStandardCodePage(scpAnsi);
  DefaultFileSystemCodePage:=GetStandardCodePage(scpFileSystemSingleByte);
  DefaultRTLFileSystemCodePage:=DefaultFileSystemCodePage;

  {$ifdef FPC_HAS_CPSTRING}
  SetStdIOCodePages;
  {$endif FPC_HAS_CPSTRING}

  { init conversion tables for main program }
  InitThread;
finalization
  { fini conversion tables for main program }
  FiniThread;
  { unload iconv library }
  if iconvlib<>0 then
    FreeLibrary(iconvlib);
  { restore previous (probably default) widestring manager so that subsequent calls
    into the widestring manager won't trigger the finalized functionality }
  SetWideStringManager(OrgWideStringManager);
end.
