{
    Copyright (c) 1998-2002 by the FPC Development Team

    Dumps the contents of a FPC unit file (PPU File)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************}
program ppudump;

{$mode objfpc}
{$H+}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  { do NOT add symconst or globtype to make merging easier }
  SysUtils,
  constexp,
=======
  SysUtils,
>>>>>>> graemeg/fixes_2_2
=======
  SysUtils,
>>>>>>> origin/fixes_2_2
  ppu,
  globals,
  tokens;

const
<<<<<<< HEAD
<<<<<<< HEAD
  Version   = 'Version 2.5.1';
  Title     = 'PPU-Analyser';
  Copyright = 'Copyright (c) 1998-2010 by the Free Pascal Development Team';
=======
  Version   = 'Version 2.2.0';
  Title     = 'PPU-Analyser';
  Copyright = 'Copyright (c) 1998-2007 by the Free Pascal Development Team';
>>>>>>> graemeg/fixes_2_2
=======
  Version   = 'Version 2.2.0';
  Title     = 'PPU-Analyser';
  Copyright = 'Copyright (c) 1998-2007 by the Free Pascal Development Team';
>>>>>>> origin/fixes_2_2

{ verbosity }
  v_none           = $0;
  v_header         = $1;
  v_defs           = $2;
  v_syms           = $4;
  v_interface      = $8;
  v_implementation = $10;
//  v_browser        = $20;
  v_all            = $ff;

type
  tprocinfoflag=(
    { procedure has at least one assembler block }
    pi_has_assembler_block,
    { procedure does a call }
    pi_do_call,
    { procedure has a try statement = no register optimization }
    pi_uses_exceptions,
    { procedure is declared as @var(assembler), don't optimize}
    pi_is_assembler,
    { procedure contains data which needs to be finalized }
    pi_needs_implicit_finally,
    { procedure has the implicit try..finally generated }
    pi_has_implicit_finally,
    { procedure uses fpu}
    pi_uses_fpu,
    { procedure uses GOT for PIC code }
    pi_needs_got,
    { references var/proc/type/const in static symtable,
      i.e. not allowed for inlining from other units }
    pi_uses_static_symtable,
    { set if the procedure has to push parameters onto the stack }
    pi_has_stackparameter,
    { set if the procedure has at least one got }
    pi_has_label,
    { calls itself recursive }
    pi_is_recursive,
    { stack frame optimization not possible (only on x86 probably) }
    pi_needs_stackframe,
    { set if the procedure has at least one register saved on the stack }
    pi_has_saved_regs,
    { dfa was generated for this proc }
    pi_dfaavailable
  );
  tprocinfoflags=set of tprocinfoflag;

  tspecialgenerictoken = (ST_LOADSETTINGS,ST_LINE,ST_COLUMN,ST_FILEINDEX);

  { Copied from systems.pas }
  tsystemcpu=
  (
        cpu_no,                       { 0 }
        cpu_i386,                     { 1 }
        cpu_m68k,                     { 2 }
        cpu_alpha,                    { 3 }
        cpu_powerpc,                  { 4 }
        cpu_sparc,                    { 5 }
        cpu_vm,                       { 6 }
        cpu_iA64,                     { 7 }
        cpu_x86_64,                   { 8 }
        cpu_mips,                     { 9 }
        cpu_arm,                      { 10 }
        cpu_powerpc64,                { 11 }
        cpu_avr                       { 12 }
  );

var
  ppufile     : tppufile;
  space       : string;
  verbose     : longint;
  derefdata   : pbyte;
  derefdatalen : longint;

{****************************************************************************
                          Helper Routines
****************************************************************************}

const has_errors : boolean = false;
Procedure Error(const S : string);
Begin
   Writeln(S);
   has_errors:=true;
End;


function ToStr(w:longint):String;
begin
  Str(w,ToStr);
end;

Function Target2Str(w:longint):string;
type
  { taken from systems.pas }
  ttarget =
  (
        target_none,               { 0 }
        obsolete_target_i386_GO32V1,{ 1 }
        target_i386_GO32V2,        { 2 }
        target_i386_linux,         { 3 }
        target_i386_OS2,           { 4 }
        target_i386_Win32,         { 5 }
        target_i386_freebsd,       { 6 }
        target_m68k_Amiga,         { 7 }
        target_m68k_Atari,         { 8 }
        target_m68k_Mac,           { 9 }
        target_m68k_linux,         { 10 }
        target_m68k_PalmOS,        { 11 }
        target_alpha_linux,        { 12 }
        target_powerpc_linux,      { 13 }
        target_powerpc_macos,      { 14 }
        target_i386_solaris,       { 15 }
        target_i386_beos,          { 16 }
        target_i386_netbsd,        { 17 }
        target_m68k_netbsd,        { 18 }
        target_i386_Netware,       { 19 }
        target_i386_qnx,           { 20 }
        target_i386_wdosx,         { 21 }
        target_sparc_solaris,      { 22 }
        target_sparc_linux,        { 23 }
        target_i386_openbsd,       { 24 }
        target_m68k_openbsd,       { 25 }
        target_x86_64_linux,       { 26 }
        target_powerpc_darwin,     { 27 }
        target_i386_emx,           { 28 }
        target_powerpc_netbsd,     { 29 }
        target_powerpc_openbsd,    { 30 }
        target_arm_linux,          { 31 }
        target_i386_watcom,        { 32 }
        target_powerpc_MorphOS,    { 33 }
        target_x86_64_freebsd,     { 34 }
        target_i386_netwlibc,      { 35 }
        target_powerpc_Amiga,      { 36 }
        target_x86_64_win64,       { 37 }
        target_arm_wince,          { 38 }
        target_ia64_win64,         { 39 }
        target_i386_wince,         { 40 }
        target_x86_6432_linux,     { 41 }
        target_arm_gba,            { 42 }
        target_powerpc64_linux,    { 43 }
        target_i386_darwin,        { 44 }
        target_arm_palmos,         { 45 }
        target_powerpc64_darwin,   { 46 }
        target_arm_nds,            { 47 }
        target_i386_embedded,      { 48 }
        target_m68k_embedded,      { 49 }
        target_alpha_embedded,     { 50 }
        target_powerpc_embedded,   { 51 }
        target_sparc_embedded,     { 52 }
        target_vm_embedded,        { 53 }
        target_iA64_embedded,      { 54 }
        target_x86_64_embedded,    { 55 }
        target_mips_embedded,      { 56 }
        target_arm_embedded,       { 57 }
        target_powerpc64_embedded, { 58 }
        target_i386_symbian,       { 59 }
        target_arm_symbian,        { 60 }
        target_x86_64_darwin,      { 61 }
        target_avr_embedded,       { 62 }
<<<<<<< HEAD
<<<<<<< HEAD
        target_i386_haiku,         { 63 }
        target_arm_darwin,         { 64 }
        target_x86_64_solaris,     { 65 }
        target_mips_linux,         { 66 }
        target_mipsel_linux,       { 67 }
        target_i386_nativent,      { 68 }
        target_i386_iphonesim      { 69 }
  );
=======
        target_i386_haiku          { 63 }             
  );

>>>>>>> graemeg/fixes_2_2
=======
        target_i386_haiku          { 63 }             
  );

>>>>>>> origin/fixes_2_2
const
  Targets : array[ttarget] of string[18]=(
  { 0 }   'none',
  { 1 }   'GO32V1 (obsolete)',
  { 2 }   'GO32V2',
  { 3 }   'Linux-i386',
  { 4 }   'OS/2',
  { 5 }   'Win32',
  { 6 }   'FreeBSD-i386',
  { 7 }   'Amiga',
  { 8 }   'Atari',
  { 9 }   'MacOS-m68k',
  { 10 }  'Linux-m68k',
  { 11 }  'PalmOS-m68k',
  { 12 }  'Linux-alpha',
  { 13 }  'Linux-ppc',
  { 14 }  'MacOS-ppc',
  { 15 }  'Solaris-i386',
  { 16 }  'BeOS-i386',
  { 17 }  'NetBSD-i386',
  { 18 }  'NetBSD-m68k',
  { 19 }  'Netware-i386-clib',
  { 20 }  'Qnx-i386',
  { 21 }  'WDOSX-i386',
  { 22 }  'Solaris-sparc',
  { 23 }  'Linux-sparc',
  { 24 }  'OpenBSD-i386',
  { 25 }  'OpenBSD-m68k',
  { 26 }  'Linux-x86-64',
  { 27 }  'MacOSX-ppc',
  { 28 }  'OS/2 via EMX',
  { 29 }  'NetBSD-powerpc',
  { 30 }  'OpenBSD-powerpc',
  { 31 }  'Linux-arm',
  { 32 }  'Watcom-i386',
  { 33 }  'MorphOS-powerpc',
  { 34 }  'FreeBSD-x86-64',
  { 35 }  'Netware-i386-libc',
  { 36 }  'Amiga-PowerPC',
  { 37 }  'Win64-x64',
  { 38 }  'WinCE-ARM',
  { 39 }  'Win64-iA64',
  { 40 }  'WinCE-i386',
  { 41 }  'Linux-x64',
  { 42 }  'GBA-arm',
  { 43 }  'Linux-powerpc64',
  { 44 }  'Darwin-i386',
  { 45 }  'PalmOS-arm',
  { 46 }  'MacOSX-powerpc64',
  { 47 }  'NDS-arm',
  { 48 }  'Embedded-i386',
  { 49 }  'Embedded-m68k',
  { 50 }  'Embedded-alpha',
  { 51 }  'Embedded-powerpc',
  { 52 }  'Embedded-sparc',
  { 53 }  'Embedded-vm',
  { 54 }  'Embedded-iA64',
  { 55 }  'Embedded-x64',
  { 56 }  'Embedded-mips',
  { 57 }  'Embedded-arm',
  { 58 }  'Embedded-powerpc64',
  { 59 }  'Symbian-i386',
  { 60 }  'Symbian-arm',
  { 61 }  'MacOSX-x64',
  { 62 }  'Embedded-avr',
<<<<<<< HEAD
<<<<<<< HEAD
  { 63 }  'Haiku-i386',
  { 64 }  'Darwin-ARM',
  { 65 }  'Solaris-x86-64',
  { 66 }  'Linux-MIPS',
  { 67 }  'Linux-MIPSel',
  { 68 }  'NativeNT-i386',
  { 69 }  'iPhoneSim-i386'
=======
  { 63 }  'Haiku-i386'        
>>>>>>> graemeg/fixes_2_2
=======
  { 63 }  'Haiku-i386'        
>>>>>>> origin/fixes_2_2
  );
begin
  if w<=ord(high(ttarget)) then
    Target2Str:=Targets[ttarget(w)]
  else
    Target2Str:='<!! Unknown target value '+tostr(w)+'>';
end;


Function Cpu2Str(w:longint):string;
const
  CpuTxt : array[tsystemcpu] of string[9]=
    ('none','i386','m68k','alpha','powerpc','sparc','vis','ia64',
     'x86_64','mips','arm','powerpc64','avr');
begin
  if w<=ord(high(tsystemcpu)) then
    Cpu2Str:=CpuTxt[tsystemcpu(w)]
  else
    Cpu2Str:='<!! Unknown cpu value '+tostr(w)+'>';
end;


Function Varspez2Str(w:longint):string;
const
  varspezstr : array[0..4] of string[6]=('Value','Const','Var','Out','Hidden');
begin
  if w<=ord(high(varspezstr)) then
    Varspez2Str:=varspezstr[w]
  else
    Varspez2Str:='<!! Unknown varspez value '+tostr(w)+'>';
end;

Function VarRegable2Str(w:longint):string;
const
  varregableStr : array[0..4] of string[6]=('None','IntReg','FPUReg','MMReg','Addr');
begin
  if w<=ord(high(varregablestr)) then
    Varregable2Str:=varregablestr[w]
  else
    Varregable2Str:='<!! Unknown regable value '+tostr(w)+'>';
end;


Function Visibility2Str(w:longint):string;
const
  visibilityName : array[0..6] of string[16] = (
    'hidden','strict private','private','strict protected','protected',
    'public','published'
  );
begin
  if w<=ord(high(visibilityName)) then
    result:=visibilityName[w]
  else
    result:='<!! Unknown visibility value '+tostr(w)+'>';
end;


function PPUFlags2Str(flags:longint):string;
type
  tflagopt=record
    mask : longint;
    str  : string[30];
  end;
const
  flagopts=17;
  flagopt : array[1..flagopts] of tflagopt=(
    (mask: $1    ;str:'init'),
    (mask: $2    ;str:'final'),
    (mask: $4    ;str:'big_endian'),
    (mask: $8    ;str:'dbx'),
//    (mask: $10   ;str:'browser'),
    (mask: $20   ;str:'in_library'),
    (mask: $40   ;str:'smart_linked'),
    (mask: $80   ;str:'static_linked'),
    (mask: $100  ;str:'shared_linked'),
//    (mask: $200  ;str:'local_browser'),
    (mask: $400  ;str:'no_link'),
    (mask: $800  ;str:'has_resources'),
    (mask: $1000  ;str:'little_endian'),
    (mask: $2000  ;str:'release'),
    (mask: $4000  ;str:'local_threadvars'),
    (mask: $8000  ;str:'fpu_emulation_on'),
    (mask: $10000  ;str:'has_debug_info'),
    (mask: $20000  ;str:'local_symtable'),
    (mask: $40000  ;str:'uses_variants')
  );
var
  i : longint;
  first  : boolean;
  s : string;
begin
  s:='';
  if flags<>0 then
   begin
     first:=true;
     for i:=1to flagopts do
      if (flags and flagopt[i].mask)<>0 then
       begin
         if first then
           first:=false
         else
           s:=s+', ';
         s:=s+flagopt[i].str;
       end;
   end
  else
   s:='none';
  PPUFlags2Str:=s;
end;


    Function L0(l:longint):string;
    {
      return the string of value l, if l<10 then insert a zero, so
      the string is always at least 2 chars '01','02',etc
    }
      var
        s : string;
      begin
        Str(l,s);
        if l<10 then
         s:='0'+s;
        L0:=s;
      end;


   function  filetimestring( t : longint) : string;
   {
     convert dos datetime t to a string YY/MM/DD HH:MM:SS
   }
     var
       DT : TDateTime;
       hsec : word;
       Year,Month,Day: Word;
       hour,min,sec : word;
     begin
       if t=-1 then
        begin
          Result := 'Not Found';
          exit;
        end;
       DT := FileDateToDateTime(t);
       DecodeTime(DT,hour,min,sec,hsec);
       DecodeDate(DT,year,month,day);
       Result := L0(Year)+'/'+L0(Month)+'/'+L0(Day)+' '+L0(Hour)+':'+L0(min)+':'+L0(sec);
     end;


{****************************************************************************
                             Read Routines
****************************************************************************}

Procedure ReadLinkContainer(const prefix:string);
{
  Read a serie of strings and write to the screen starting every line
  with prefix
}
  function maskstr(m:longint):string;
  const
    { link options }
    link_none    = $0;
    link_always  = $1;
    link_static  = $2;
    link_smart   = $4;
    link_shared  = $8;
  var
    s : string;
  begin
    s:='';
    if (m and link_always)<>0 then
     s:=s+'always ';
    if (m and link_static)<>0 then
     s:=s+'static ';
    if (m and link_smart)<>0 then
     s:=s+'smart ';
    if (m and link_shared)<>0 then
     s:=s+'shared ';
    maskstr:=s;
  end;

var
  s : string;
  m : longint;
begin
  while not ppufile.endofentry do
   begin
     s:=ppufile.getstring;
     m:=ppufile.getlongint;
     WriteLn(prefix,s,' (',maskstr(m),')');
   end;
end;


Procedure ReadContainer(const prefix:string);
{
  Read a serie of strings and write to the screen starting every line
  with prefix
}
begin
  while not ppufile.endofentry do
   WriteLn(prefix,ppufile.getstring);
end;


procedure ReadLoadUnit;
var
  ucrc,uintfcrc, indcrc : cardinal;
begin
  while not ppufile.EndOfEntry do
    begin
      write('Uses unit: ',ppufile.getstring);
      ucrc:=cardinal(ppufile.getlongint);
      uintfcrc:=cardinal(ppufile.getlongint);
      indcrc:=cardinal(ppufile.getlongint);
      writeln(' (Crc: ',hexstr(ucrc,8),', IntfcCrc: ',hexstr(uintfcrc,8),', IndCrc: ',hexstr(indcrc,8),')');
    end;
end;


Procedure ReadDerefmap;
var
  i,mapsize : longint;
begin
  mapsize:=ppufile.getlongint;
  writeln('DerefMapsize: ',mapsize);
  for i:=0 to mapsize-1 do
    writeln('DerefMap[',i,'] = ',ppufile.getstring);
end;


Procedure ReadImportSymbols;
var
  extlibname  : string;
  j,
  extsymcnt   : longint;
  extsymname  : string;
  extsymordnr : longint;
  extsymisvar : boolean;
begin
  while not ppufile.endofentry do
    begin
      extlibname:=ppufile.getstring;
      extsymcnt:=ppufile.getlongint;
      writeln('External Library: ',extlibname,' (',extsymcnt,' imports)');
      for j:=0 to extsymcnt-1 do
        begin
          extsymname:=ppufile.getstring;
          extsymordnr:=ppufile.getlongint;
          extsymisvar:=ppufile.getbyte<>0;
          writeln(' ',extsymname,' (OrdNr: ',extsymordnr,' IsVar: ',extsymisvar,')');
        end;
    end;
end;


Procedure ReadDerefdata;
begin
  derefdatalen:=ppufile.entrysize;
  if derefdatalen=0 then
    begin
      writeln('!! Error: derefdatalen=0');
      exit;
    end;
  Writeln('Derefdata length: ',derefdatalen);
  derefdata:=allocmem(derefdatalen);
  ppufile.getdata(derefdata^,derefdatalen);
end;


Procedure ReadWpoFileInfo;
begin
  Writeln('Compiled with input whole-program optimisation from ',ppufile.getstring,' ',filetimestring(ppufile.getlongint));
end;


Procedure ReadAsmSymbols;
type
  { Copied from aasmbase.pas }
  TAsmsymbind=(AB_NONE,AB_EXTERNAL,AB_COMMON,AB_LOCAL,AB_GLOBAL);
  TAsmsymtype=(AT_NONE,AT_FUNCTION,AT_DATA,AT_SECTION,AT_LABEL);
var
  s,
  bindstr,
  typestr  : string;
  i : longint;
begin
  writeln(space,'Number of AsmSymbols: ',ppufile.getlongint);
  i:=0;
  while (not ppufile.endofentry) and (not ppufile.error) do
   begin
     s:=ppufile.getstring;
     case tasmsymbind(ppufile.getbyte) of
       AB_EXTERNAL :
         bindstr:='External';
       AB_COMMON :
         bindstr:='Common';
       AB_LOCAL :
         bindstr:='Local';
       AB_GLOBAL :
         bindstr:='Global';
       else
         bindstr:='<Error !!>'
     end;
     case tasmsymtype(ppufile.getbyte) of
       AT_FUNCTION :
         typestr:='Function';
       AT_DATA :
         typestr:='Data';
       AT_SECTION :
         typestr:='Section';
       AT_LABEL :
         typestr:='Label';
       else
         typestr:='<Error !!>'
     end;
     Writeln(space,'  ',i,' : ',s,' [',bindstr,',',typestr,']');
     inc(i);
   end;
end;

function getexprint:Tconstexprint;

begin
  getexprint.overflow:=false;
  getexprint.signed:=boolean(ppufile.getbyte);
  getexprint.svalue:=ppufile.getint64;
end;

Procedure ReadPosInfo;
var
  info : byte;
  fileindex,line,column : longint;
begin
  with ppufile do
   begin
     {
       info byte layout in bits:
       0-1 - amount of bytes for fileindex
       2-3 - amount of bytes for line
       4-5 - amount of bytes for column
     }
     info:=getbyte;
     case (info and $03) of
      0 : fileindex:=getbyte;
      1 : fileindex:=getword;
      2 : fileindex:=(getbyte shl 16) or getword;
      3 : fileindex:=getlongint;
     end;
     case ((info shr 2) and $03) of
      0 : line:=getbyte;
      1 : line:=getword;
      2 : line:=(getbyte shl 16) or getword;
      3 : line:=getlongint;
     end;
     case ((info shr 4) and $03) of
      0 : column:=getbyte;
      1 : column:=getword;
      2 : column:=(getbyte shl 16) or getword;
      3 : column:=getlongint;
     end;
     Writeln(fileindex,' (',line,',',column,')');
   end;
end;


procedure readderef(const derefspace: string);
type
  tdereftype = (deref_nil,
    deref_unit,
    deref_symid,
    deref_defid
  );
var
  b : tdereftype;
  first : boolean;
  idx : longint;
  i,n : byte;
  pdata : pbyte;
begin
  if not assigned(derefdata) then
    exit;
  first:=true;
  idx:=ppufile.getlongint;
  if (idx>derefdatalen) then
    begin
      writeln('!! Error: Deref idx ',idx,' > ',derefdatalen);
      exit;
    end;
  write(derefspace,'(',idx,') ');
  pdata:=@derefdata[idx];
  i:=0;
  n:=pdata[i];
  inc(i);
  if n<1 then
    begin
      writeln('!! Error: Deref len < 1');
      exit;
    end;
  while (i<=n) do
   begin
     if not first then
      write(', ')
     else
      first:=false;
     b:=tdereftype(pdata[i]);
     inc(i);
     case b of
       deref_nil :
         write('Nil');
       deref_symid :
         begin
           idx:=pdata[i] shl 24 or pdata[i+1] shl 16 or pdata[i+2] shl 8 or pdata[i+3];
           inc(i,4);
           write('SymId ',idx);
         end;
       deref_defid :
         begin
           idx:=pdata[i] shl 24 or pdata[i+1] shl 16 or pdata[i+2] shl 8 or pdata[i+3];
           inc(i,4);
           write('DefId ',idx);
         end;
       deref_unit :
         begin
           idx:=pdata[i] shl 8 or pdata[i+1];
           inc(i,2);
           write('Unit ',idx);
         end;
       else
         begin
           writeln('!! unsupported dereftyp: ',ord(b));
           break;
         end;
     end;
   end;
  writeln;
end;


procedure readpropaccesslist(const s:string);
type
  tsltype = (sl_none,
    sl_load,
    sl_call,
    sl_subscript,
    sl_vec,
    sl_typeconv,
    sl_absolutetype
  );
const
  slstr : array[tsltype] of string[12] = ('',
    'load',
    'call',
    'subscript',
    'vec',
    'typeconv',
    'absolutetype'
  );
var
  sl : tsltype;
begin
  readderef('');
  repeat
    sl:=tsltype(ppufile.getbyte);
    if sl=sl_none then
     break;
    write(s,'(',slstr[sl],') ');
    case sl of
      sl_call,
      sl_load,
      sl_subscript :
        readderef('');
      sl_absolutetype,
      sl_typeconv :
        readderef('');
      sl_vec :
        begin
          writeln(ppufile.getlongint);
          readderef('');
        end;
    end;
  until false;
end;


procedure readsymoptions(space : string);
type
  { symbol options }
  tsymoption=(sp_none,
    sp_static,
    sp_hint_deprecated,
    sp_hint_platform,
    sp_hint_library,
    sp_hint_unimplemented,
    sp_hint_experimental,
    sp_has_overloaded,
    sp_internal,  { internal symbol, not reported as unused }
    sp_implicitrename,
    sp_generic_para,
    sp_has_deprecated_msg
  );
  tsymoptions=set of tsymoption;
  tsymopt=record
    mask : tsymoption;
    str  : string[30];
  end;
const
  symopts=11;
  symopt : array[1..symopts] of tsymopt=(
     (mask:sp_static;             str:'Static'),
     (mask:sp_hint_deprecated;    str:'Hint Deprecated'),
     (mask:sp_hint_platform;      str:'Hint Platform'),
     (mask:sp_hint_library;       str:'Hint Library'),
     (mask:sp_hint_unimplemented; str:'Hint Unimplemented'),
     (mask:sp_hint_experimental;  str:'Hint Experimental'),
     (mask:sp_has_overloaded;     str:'Has overloaded'),
     (mask:sp_internal;           str:'Internal'),
     (mask:sp_implicitrename;     str:'Implicit Rename'),
     (mask:sp_generic_para;       str:'Generic Parameter'),
     (mask:sp_has_deprecated_msg; str:'Has Deprecated Message')
  );
var
  symoptions : tsymoptions;
  i      : longint;
  first  : boolean;
begin
  ppufile.getsmallset(symoptions);
  if symoptions<>[] then
   begin
     first:=true;
     for i:=1to symopts do
      if (symopt[i].mask in symoptions) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(symopt[i].str);
       end;
   end;
  writeln;
  if sp_has_deprecated_msg in symoptions then
    writeln(space,'Deprecated : ', ppufile.getstring);
end;


procedure readcommonsym(const s:string);
begin
  writeln(space,'** Symbol Id ',ppufile.getlongint,' **');
  writeln(space,s,ppufile.getstring);
  write  (space,'     File Pos : ');
  readposinfo;
  writeln(space,'   Visibility : ',Visibility2Str(ppufile.getbyte));
  write  (space,'   SymOptions : ');
  readsymoptions(space+'   ');
end;



type
  { flags for a definition }
  tdefoption=(df_none,
    { type is unique, i.e. declared with type = type <tdef>; }
    df_unique,
    { type is a generic }
    df_generic,
    { type is a specialization of a generic type }
    df_specialization,
    { def has been copied from another def so symtable is not owned }
    df_copied_def
  );
  tdefoptions=set of tdefoption;


var
  { needed during tobjectdef parsing... }
  current_defoptions : tdefoptions;

procedure readcommondef(const s:string; out defoptions: tdefoptions);
type
  tdefstate=(ds_none,
    ds_vmt_written,
    ds_rtti_table_used,
    ds_init_table_used,
    ds_rtti_table_written,
    ds_init_table_written,
    ds_dwarf_dbg_info_used,
    ds_dwarf_dbg_info_written
  );
  tdefstates=set of tdefstate;
  tdefopt=record
    mask : tdefoption;
    str  : string[30];
  end;
  tdefstateinfo=record
    mask : tdefstate;
    str  : string[30];
  end;
  ptoken=^ttoken;
const
  defopt : array[1..ord(high(tdefoption))] of tdefopt=(
     (mask:df_unique;         str:'Unique Type'),
     (mask:df_generic;        str:'Generic'),
     (mask:df_specialization; str:'Specialization'),
     (mask:df_copied_def;     str:'Copied Typedef')
  );
  defstate : array[1..ord(high(tdefstate))] of tdefstateinfo=(
     (mask:ds_vmt_written;           str:'VMT Written'),
     (mask:ds_rtti_table_used;       str:'RTTITable Used'),
     (mask:ds_init_table_used;       str:'InitTable Used'),
     (mask:ds_rtti_table_written;    str:'RTTITable Written'),
     (mask:ds_init_table_written;    str:'InitTable Written'),
     (mask:ds_dwarf_dbg_info_used;   str:'Dwarf DbgInfo Used'),
     (mask:ds_dwarf_dbg_info_written;str:'Dwarf DbgInfo Written')
  );
var
<<<<<<< HEAD
=======
  defoptions : tdefoptions;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  defstates  : tdefstates;
  i      : longint;
  first  : boolean;
  tokenbufsize : longint;
  tokenbuf : pbyte;
  token : ttoken;
  len : sizeint;
  wstring : widestring;
  astring : ansistring;
begin
  writeln(space,'** Definition Id ',ppufile.getlongint,' **');
  writeln(space,s);
  write  (space,'      Type symbol : ');
  readderef('');
  write  (space,'       DefOptions : ');
  ppufile.getsmallset(defoptions);
  if defoptions<>[] then
    begin
      first:=true;
      for i:=1to high(defopt) do
       if (defopt[i].mask in defoptions) then
        begin
          if first then
            first:=false
          else
            write(', ');
          write(defopt[i].str);
        end;
    end;
  writeln;

  write  (space,'        DefStates : ');
  ppufile.getsmallset(defstates);
  if defstates<>[] then
    begin
      first:=true;
<<<<<<< HEAD
<<<<<<< HEAD
      for i:=1to high(defstate) do
=======
      for i:=1to defstateinfos do
>>>>>>> graemeg/fixes_2_2
=======
      for i:=1to defstateinfos do
>>>>>>> origin/fixes_2_2
       if (defstate[i].mask in defstates) then
        begin
          if first then
            first:=false
          else
            write(', ');
          write(defstate[i].str);
        end;
    end;
  writeln;

  if df_generic in defoptions then
    begin
      tokenbufsize:=ppufile.getlongint;
      writeln(space,' Tokenbuffer size : ',tokenbufsize);
      tokenbuf:=allocmem(tokenbufsize);
      ppufile.getdata(tokenbuf^,tokenbufsize);
      i:=0;
      write(space,' Tokens: ');
      while i<tokenbufsize do
        begin
          token:=ptoken(@tokenbuf[i])^;
          if token<>_GENERICSPECIALTOKEN then
            write(arraytokeninfo[token].str);
          inc(i,SizeOf(token));
          case token of
            _CWCHAR,
            _CWSTRING :
              begin
                len:=psizeint(@tokenbuf[i])^;
                inc(i,sizeof(sizeint));
                setlength(wstring,len);
                move(tokenbuf[i],wstring[1],len*2);
                write(' ',wstring);
                inc(i,len*2);
              end;
            _CSTRING:
              begin
                len:=psizeint(@tokenbuf[i])^;
                inc(i,sizeof(sizeint));
                setlength(astring,len);
                move(tokenbuf[i],astring[1],len);
                write(' ',astring);
                inc(i,len);
              end;
            _CCHAR,
            _INTCONST,
            _REALNUMBER :
              begin
                write(' ',pshortstring(@tokenbuf[i])^);
                inc(i,tokenbuf[i]+1);
              end;
            _ID :
              begin
                inc(i,SizeOf(ttoken)); // idtoken
                write(' ',pshortstring(@tokenbuf[i])^);
                inc(i,tokenbuf[i]+1);
              {
                replaytokenbuf.read(orgpattern[0],1);
                replaytokenbuf.read(orgpattern[1],length(orgpattern));
                pattern:=upper(orgpattern);
              }
              end;
            _GENERICSPECIALTOKEN:
              begin
                case tspecialgenerictoken(tokenbuf[i]) of
                  ST_LOADSETTINGS:
                    begin
                      inc(i);
                      write('Settings');
                      inc(i,sizeof(tsettings));
                    end;
                  ST_LINE:
                    begin
                      inc(i);
                      write('Line: ',pdword(@tokenbuf[i])^);
                      inc(i,4);
                    end;
                  ST_COLUMN:
                    begin
                      inc(i);
                      write('Col: ',pword(@tokenbuf[i])^);
                      inc(i,2);
                    end;
                  ST_FILEINDEX:
                    begin
                      inc(i);
                      write('File: ',pword(@tokenbuf[i])^);
                      inc(i,2);
                    end;
                end;
              {
                replaytokenbuf.read(specialtoken,1);
                case specialtoken of
                  ST_LOADSETTINGS:
                    begin
                      replaytokenbuf.read(current_settings,sizeof(current_settings));
                    end
                  else
                    internalerror(2006103010);
                end;
                continue;
              }
              end;
          end;

          if i<tokenbufsize then
            write(',');
        end;
      writeln;
      freemem(tokenbuf);
    end;
  if df_specialization in defoptions then
    begin
      write  (space,' Orig. GenericDef : ');
      readderef('');
    end;
  current_defoptions:=defoptions;
end;


{ Read abstract procdef and return if inline procdef }
type
  tproccalloption=(pocall_none,
    { procedure uses C styled calling }
    pocall_cdecl,
    { C++ calling conventions }
    pocall_cppdecl,
    { Far16 for OS/2 }
    pocall_far16,
    { Old style FPC default calling }
    pocall_oldfpccall,
    { Procedure has compiler magic}
    pocall_internproc,
    { procedure is a system call, applies e.g. to MorphOS and PalmOS }
    pocall_syscall,
    { pascal standard left to right }
    pocall_pascal,
    { procedure uses register (fastcall) calling }
    pocall_register,
    { safe call calling conventions }
    pocall_safecall,
    { procedure uses stdcall call }
    pocall_stdcall,
    { Special calling convention for cpus without a floating point
      unit. Floating point numbers are passed in integer registers
      instead of floating point registers. Depending on the other
      available calling conventions available for the cpu
      this replaces either pocall_fastcall or pocall_stdcall.
    }
    pocall_softfloat,
    { Metrowerks Pascal. Special case on Mac OS (X): passes all }
    { constant records by reference.                            }
    pocall_mwpascal
  );
  tproccalloptions = set of tproccalloption;
  tproctypeoption=(potype_none,
    potype_proginit,     { Program initialization }
    potype_unitinit,     { unit initialization }
    potype_unitfinalize, { unit finalization }
    potype_constructor,  { Procedure is a constructor }
    potype_destructor,   { Procedure is a destructor }
    potype_operator,     { Procedure defines an operator }
    potype_procedure,
    potype_function,
    potype_class_constructor, { class constructor }
    potype_class_destructor   { class destructor  }
  );
  tproctypeoptions=set of tproctypeoption;
  tprocoption=(po_none,
    po_classmethod,       { class method }
    po_virtualmethod,     { Procedure is a virtual method }
    po_abstractmethod,    { Procedure is an abstract method }
    po_finalmethod,       { Procedure is a final method }
    po_staticmethod,      { static method }
    po_overridingmethod,  { method with override directive }
    po_methodpointer,     { method pointer, only in procvardef, also used for 'with object do' }
    po_interrupt,         { Procedure is an interrupt handler }
    po_iocheck,           { IO checking should be done after a call to the procedure }
    po_assembler,         { Procedure is written in assembler }
    po_msgstr,            { method for string message handling }
    po_msgint,            { method for int message handling }
    po_exports,           { Procedure has export directive (needed for OS/2) }
    po_external,          { Procedure is external (in other object or lib)}
    po_overload,          { procedure is declared with overload directive }
    po_varargs,           { printf like arguments }
    po_internconst,       { procedure has constant evaluator intern }
    { flag that only the address of a method is returned and not a full methodpointer }
    po_addressonly,
    { procedure is exported }
    po_public,
    { calling convention is specified explicitly }
    po_hascallingconvention,
    { reintroduce flag }
    po_reintroduce,
    { location of parameters is given explicitly as it is necessary for some syscall
      conventions like that one of MorphOS }
    po_explicitparaloc,
    { no stackframe will be generated, used by lowlevel assembler like get_frame }
    po_nostackframe,
    po_has_mangledname,
    po_has_public_name,
    po_forward,
    po_global,
    po_has_inlininginfo,
    { The different kind of syscalls on MorphOS }
    po_syscall_legacy,
    po_syscall_sysv,
    po_syscall_basesysv,
    po_syscall_sysvbase,
    po_syscall_r12base,
    { Procedure can be inlined }
    po_inline,
    { Procedure is used for internal compiler calls }
    po_compilerproc,
    { importing }
    po_has_importdll,
    po_has_importname,
    po_kylixlocal,
    po_dispid,
    { weakly linked (i.e., may or may not exist at run time) }
    po_weakexternal,
    { Objective-C method }
    po_objc,
    { enumerator support }
    po_enumerator_movenext,
    { optional Objective-C protocol method }
    po_optional,
    { nested procedure that uses Delphi-style calling convention for passing
      the frame pointer (pushed on the stack, always the last parameter,
      removed by the caller). Required for nested procvar compatibility,
      because such procvars can hold both regular and nested procedures
      (when calling a regular procedure using the above convention, it will
       simply not see the frame pointer parameter, and since the caller cleans
       up the stack will also remain balanced) }
    po_delphi_nested_cc
  );
  tprocoptions=set of tprocoption;

procedure read_abstract_proc_def(var proccalloption:tproccalloption;var procoptions:tprocoptions);
type
  tproccallopt=record
    mask : tproccalloption;
    str  : string[30];
  end;
  tproctypeopt=record
    mask : tproctypeoption;
    str  : string[30];
  end;
  tprocopt=record
    mask : tprocoption;
    str  : string[30];
  end;
const
  proccalloptionStr : array[tproccalloption] of string[14]=('',
     'CDecl',
     'CPPDecl',
     'Far16',
     'OldFPCCall',
     'InternProc',
     'SysCall',
     'Pascal',
     'Register',
     'SafeCall',
     'StdCall',
     'SoftFloat',
     'MWPascal'
   );
  proctypeopt : array[1..ord(high(tproctypeoption))] of tproctypeopt=(
     (mask:potype_proginit;          str:'ProgInit'),
     (mask:potype_unitinit;          str:'UnitInit'),
     (mask:potype_unitfinalize;      str:'UnitFinalize'),
     (mask:potype_constructor;       str:'Constructor'),
     (mask:potype_destructor;        str:'Destructor'),
     (mask:potype_operator;          str:'Operator'),
     (mask:potype_procedure;         str:'Procedure'),
     (mask:potype_function;          str:'Function'),
     (mask:potype_class_constructor; str:'Class Constructor'),
     (mask:potype_class_destructor;  str:'Class Destructor')
  );
  procopt : array[1..ord(high(tprocoption))] of tprocopt=(
     (mask:po_classmethod;     str:'ClassMethod'),
     (mask:po_virtualmethod;   str:'VirtualMethod'),
     (mask:po_abstractmethod;  str:'AbstractMethod'),
     (mask:po_finalmethod;     str:'FinalMethod'),
     (mask:po_staticmethod;    str:'StaticMethod'),
     (mask:po_overridingmethod;str:'OverridingMethod'),
     (mask:po_methodpointer;   str:'MethodPointer'),
     (mask:po_interrupt;       str:'Interrupt'),
     (mask:po_iocheck;         str:'IOCheck'),
     (mask:po_assembler;       str:'Assembler'),
     (mask:po_msgstr;          str:'MsgStr'),
     (mask:po_msgint;          str:'MsgInt'),
     (mask:po_exports;         str:'Exports'),
     (mask:po_external;        str:'External'),
     (mask:po_overload;        str:'Overload'),
     (mask:po_varargs;         str:'VarArgs'),
     (mask:po_internconst;     str:'InternConst'),
     (mask:po_addressonly;     str:'AddressOnly'),
     (mask:po_public;          str:'Public'),
     (mask:po_hascallingconvention;str:'HasCallingConvention'),
     (mask:po_reintroduce;     str:'ReIntroduce'),
     (mask:po_explicitparaloc; str:'ExplicitParaloc'),
     (mask:po_nostackframe;    str:'NoStackFrame'),
     (mask:po_has_mangledname; str:'HasMangledName'),
     (mask:po_has_public_name; str:'HasPublicName'),
     (mask:po_forward;         str:'Forward'),
     (mask:po_global;          str:'Global'),
     (mask:po_has_inlininginfo;str:'HasInliningInfo'),
     (mask:po_syscall_legacy;  str:'SyscallLegacy'),
     (mask:po_syscall_sysv;    str:'SyscallSysV'),
     (mask:po_syscall_basesysv;str:'SyscallBaseSysV'),
     (mask:po_syscall_sysvbase;str:'SyscallSysVBase'),
     (mask:po_syscall_r12base; str:'SyscallR12Base'),
     (mask:po_inline;          str:'Inline'),
     (mask:po_compilerproc;    str:'CompilerProc'),
     (mask:po_has_importdll;   str:'HasImportDLL'),
     (mask:po_has_importname;  str:'HasImportName'),
     (mask:po_kylixlocal;      str:'KylixLocal'),
     (mask:po_dispid;          str:'DispId'),
     (mask:po_weakexternal;    str:'WeakExternal'),
     (mask:po_objc;            str:'ObjC'),
     (mask:po_enumerator_movenext; str:'EnumeratorMoveNext'),
     (mask:po_optional;        str: 'Optional'),
     (mask:po_delphi_nested_cc;str: 'Delphi-style nested frameptr')
  );
var
  proctypeoption  : tproctypeoption;
  i     : longint;
  first : boolean;
  tempbuf : array[0..255] of byte;
begin
  write(space,'      Return type : ');
  readderef('');
  writeln(space,'         Fpu used : ',ppufile.getbyte);
  proctypeoption:=tproctypeoption(ppufile.getbyte);
  write(space,'       TypeOption : ');
  first:=true;
  for i:=1 to high(proctypeopt) do
   if (proctypeopt[i].mask=proctypeoption) then
    begin
      if first then
        first:=false
      else
        write(', ');
      write(proctypeopt[i].str);
    end;
  writeln;
  proccalloption:=tproccalloption(ppufile.getbyte);
  writeln(space,'       CallOption : ',proccalloptionStr[proccalloption]);
  ppufile.getnormalset(procoptions);
  if procoptions<>[] then
   begin
     write(space,'          Options : ');
     first:=true;
     for i:=1 to high(procopt) do
      if (procopt[i].mask in procoptions) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(procopt[i].str);
       end;
     writeln;
   end;
  if (po_explicitparaloc in procoptions) then
    begin
      i:=ppufile.getbyte;
      ppufile.getdata(tempbuf,i);
    end;
end;


type
  tvaroption=(vo_none,
    vo_is_external,
    vo_is_dll_var,
    vo_is_thread_var,
    vo_has_local_copy,
    vo_is_const,  { variable is declared as const (parameter) and can't be written to }
    vo_is_public,
    vo_is_high_para,
    vo_is_funcret,
    vo_is_self,
    vo_is_vmt,
    vo_is_result,  { special result variable }
    vo_is_parentfp,
    vo_is_loop_counter, { used to detect assignments to loop counter }
    vo_is_hidden_para,
    vo_has_explicit_paraloc,
    vo_is_syscall_lib,
    vo_has_mangledname,
    vo_is_typed_const,
    vo_is_range_check,
    vo_is_overflow_check,
    vo_is_typinfo_para,
    vo_is_weak_external,
    vo_is_msgsel,
    vo_is_first_field
  );
  tvaroptions=set of tvaroption;
  { register variable }
  tvarregable=(vr_none,
    vr_intreg,
    vr_fpureg,
    vr_mmreg,
    { does not mean "needs address register", but "if it's a parameter which is }
    { passed by reference, then its address can be put in a register            }
    vr_addr
  );
procedure readabstractvarsym(const s:string;var varoptions:tvaroptions);
type
  tvaropt=record
    mask : tvaroption;
    str  : string[30];
  end;
const
  varopt : array[1..ord(high(tvaroption))] of tvaropt=(
     (mask:vo_is_external;     str:'External'),
     (mask:vo_is_dll_var;      str:'DLLVar'),
     (mask:vo_is_thread_var;   str:'ThreadVar'),
     (mask:vo_has_local_copy;  str:'HasLocalCopy'),
     (mask:vo_is_const;        str:'Constant'),
     (mask:vo_is_public;       str:'Public'),
     (mask:vo_is_high_para;    str:'HighValue'),
     (mask:vo_is_funcret;      str:'Funcret'),
     (mask:vo_is_self;         str:'Self'),
     (mask:vo_is_vmt;          str:'VMT'),
     (mask:vo_is_result;       str:'Result'),
     (mask:vo_is_parentfp;     str:'ParentFP'),
     (mask:vo_is_loop_counter; str:'LoopCounter'),
     (mask:vo_is_hidden_para;  str:'Hidden'),
     (mask:vo_has_explicit_paraloc;str:'ExplicitParaloc'),
     (mask:vo_is_syscall_lib;  str:'SysCallLib'),
     (mask:vo_has_mangledname; str:'HasMangledName'),
     (mask:vo_is_typed_const;  str:'TypedConst'),
     (mask:vo_is_range_check;  str:'RangeCheckSwitch'),
     (mask:vo_is_overflow_check; str:'OverflowCheckSwitch'),
     (mask:vo_is_typinfo_para; str:'TypeInfo'),
     (mask:vo_is_msgsel;str:'MsgSel'),
     (mask:vo_is_weak_external;str:'WeakExternal'),
     (mask:vo_is_first_field;str:'IsFirstField')
  );
var
  i : longint;
  first : boolean;
begin
  readcommonsym(s);
  writeln(space,'         Spez : ',Varspez2Str(ppufile.getbyte));
  writeln(space,'      Regable : ',Varregable2Str(ppufile.getbyte));
  writeln(space,'   Addr Taken : ',(ppufile.getbyte<>0));
  write  (space,'     Var Type : ');
  readderef('');
  ppufile.getsmallset(varoptions);
  if varoptions<>[] then
   begin
     write(space,'      Options : ');
     first:=true;
     for i:=1 to high(varopt) do
      if (varopt[i].mask in varoptions) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(varopt[i].str);
       end;
     writeln;
   end;
end;


procedure readobjectdefoptions;
type
  tobjectoption=(oo_none,
    oo_is_forward,         { the class is only a forward declared yet }
    oo_is_abstract,        { the class is abstract - only descendants can be used }
    oo_is_sealed,          { the class is sealed - can't have descendants }
    oo_has_virtual,        { the object/class has virtual methods }
    oo_has_private,
    oo_has_protected,
    oo_has_strictprivate,
    oo_has_strictprotected,
    oo_has_constructor,    { the object/class has a constructor }
    oo_has_destructor,     { the object/class has a destructor }
    oo_has_vmt,            { the object/class has a vmt }
    oo_has_msgstr,
    oo_has_msgint,
    oo_can_have_published,{ the class has rtti, i.e. you can publish properties }
    oo_has_default_property,
    oo_has_valid_guid,
    oo_has_enumerator_movenext,
    oo_has_enumerator_current,
    oo_is_external,       { the class is externally implemented (objcclass, cppclass) }
    oo_is_anonymous,      { the class is only formally defined in this module (objcclass x = class; external;) }
    oo_is_classhelper,    { objcclasses that represent categories, and Delpi-style class helpers, are marked like this }
    oo_has_class_constructor, { the object/class has a class constructor }
    oo_has_class_destructor   { the object/class has a class destructor  }
  );
  tobjectoptions=set of tobjectoption;
  tsymopt=record
    mask : tobjectoption;
    str  : string[30];
  end;
const
<<<<<<< HEAD
  symopt : array[1..ord(high(tobjectoption))] of tsymopt=(
     (mask:oo_is_forward;         str:'IsForward'),
     (mask:oo_is_abstract;        str:'IsAbstract'),
     (mask:oo_is_sealed;          str:'IsSealed'),
=======
  symopts=14;
  symopt : array[1..symopts] of tsymopt=(
     (mask:oo_is_forward;         str:'IsForward'),
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     (mask:oo_has_virtual;        str:'HasVirtual'),
     (mask:oo_has_private;        str:'HasPrivate'),
     (mask:oo_has_protected;      str:'HasProtected'),
     (mask:oo_has_strictprivate;  str:'HasStrictPrivate'),
     (mask:oo_has_strictprotected;str:'HasStrictProtected'),
     (mask:oo_has_constructor;    str:'HasConstructor'),
     (mask:oo_has_destructor;     str:'HasDestructor'),
     (mask:oo_has_vmt;            str:'HasVMT'),
     (mask:oo_has_msgstr;         str:'HasMsgStr'),
     (mask:oo_has_msgint;         str:'HasMsgInt'),
     (mask:oo_can_have_published; str:'CanHavePublished'),
     (mask:oo_has_default_property;str:'HasDefaultProperty'),
     (mask:oo_has_valid_guid;     str:'HasValidGUID'),
     (mask:oo_has_enumerator_movenext; str:'HasEnumeratorMoveNext'),
     (mask:oo_has_enumerator_current;  str:'HasEnumeratorCurrent'),
     (mask:oo_is_external;        str:'External'),
     (mask:oo_is_anonymous;       str:'Anonymous'),
     (mask:oo_is_classhelper;     str:'Class Helper/Category'),
     (mask:oo_has_class_constructor; str:'HasClassConstructor'),
     (mask:oo_has_class_destructor; str:'HasClassDestructor')
  );
var
  symoptions : tobjectoptions;
  i      : longint;
  first  : boolean;
begin
  ppufile.getsmallset(symoptions);
  if symoptions<>[] then
   begin
     first:=true;
     for i:=1 to high(symopt) do
      if (symopt[i].mask in symoptions) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(symopt[i].str);
       end;
   end;
  writeln;
end;


procedure readarraydefoptions;
type
  tarraydefoption=(ado_none,
    ado_IsConvertedPointer,
    ado_IsDynamicArray,
    ado_IsVariant,
    ado_IsConstructor,
    ado_IsArrayOfConst,
    ado_IsConstString,
    ado_IsBitPacked
  );
  tarraydefoptions=set of tarraydefoption;

  tsymopt=record
    mask : tarraydefoption;
    str  : string[30];
  end;
const
  symopt : array[1..ord(high(tarraydefoption))] of tsymopt=(
     (mask:ado_IsConvertedPointer;str:'ConvertedPointer'),
     (mask:ado_IsDynamicArray;    str:'IsDynamicArray'),
     (mask:ado_IsVariant;         str:'IsVariant'),
     (mask:ado_IsConstructor;     str:'IsConstructor'),
     (mask:ado_IsArrayOfConst;    str:'ArrayOfConst'),
     (mask:ado_IsConstString;     str:'ConstString'),
     (mask:ado_IsBitPacked;      str:'BitPacked')
  );
var
  symoptions : tarraydefoptions;
  i      : longint;
  first  : boolean;
begin
  ppufile.getsmallset(symoptions);
  if symoptions<>[] then
   begin
     first:=true;
     for i:=1 to high(symopt) do
      if (symopt[i].mask in symoptions) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(symopt[i].str);
       end;
   end;
  writeln;
end;


procedure readnodetree;
var
  l : longint;
  p : pointer;
begin
  with ppufile do
   begin
     if space<>'' then
      Writeln(space,'------ nodetree ------');
     if readentry=ibnodetree then
      begin
        l:=entrysize;
        Writeln(space,'Tree size : ',l);
        { Read data to prevent error that entry is not completly read }
        getmem(p,l);
        getdata(p^,l);
        freemem(p);
      end
     else
      begin
        Writeln('!! ibnodetree not found');
      end;
   end;
end;


procedure ReadCreatedObjTypes;
var
  i,j,
  len,
  bssize: longint;
  bs: pbyte;
begin
  if ppufile.readentry<>ibcreatedobjtypes then
    begin
      writeln('!! ibcreatedobjtypes entry not found');
      ppufile.skipdata(ppufile.entrysize);
      exit
    end;
  writeln;
  writeln(space,'WPO info');
  writeln(space,'--------');

  len:=ppufile.getlongint;
  writeln(space,'** Instantiated Object/Class types: ',len,' **');
  space:=space+'  ';
  for i:=0 to len-1 do
    readderef(space);
  setlength(space,length(space)-2);

  len:=ppufile.getlongint;
  writeln(space,'** Instantiated ClassRef types: ',len,' **');
  space:=space+'  ';
  for i:=0 to len-1 do
    readderef(space);
  setlength(space,length(space)-2);

  len:=ppufile.getlongint;
  writeln(space,'** Possibly instantiated ClassRef types : ',len,' **');
  space:=space+'  ';
  for i:=0 to len-1 do
    readderef(space);
  setlength(space,length(space)-2);

  len:=ppufile.getlongint;
  writeln(space,'** Class types with called virtual methods info : ',len,' **');
  space:=space+'  ';
  for i:=0 to len-1 do
    begin
      write(space,'Class def : ');
      readderef('');
      write(space+'  ','Called vmtentries : ');
      bssize:=ppufile.getlongint;
      getmem(bs,bssize);
      ppufile.readdata(bs^,bssize);
      for j:=0 to bssize*8-1 do
        if (((bs+j shr 3)^ shr (j and 7)) and 1) <> 0 then
          write(j,', ');
      writeln;
      freemem(bs);
    end;
  setlength(space,length(space)-2);
end;

{****************************************************************************
                             Read Symbols Part
****************************************************************************}

procedure readsymbols(const s:string);
type
  pguid = ^tguid;
  tguid = packed record
    D1: LongWord;
    D2: Word;
    D3: Word;
    D4: array[0..7] of Byte;
  end;

  absolutetyp = (tovar,toasm,toaddr);
  tconsttyp = (constnone,
    constord,conststring,constreal,
    constset,constpointer,constnil,
    constresourcestring,constwstring,constguid
  );
var
  b      : byte;
  pc     : pchar;
  i,j,len : longint;
  guid : tguid;
  tempbuf : array[0..127] of char;
  varoptions : tvaroptions;
begin
  with ppufile do
   begin
     if space<>'' then
      Writeln(space,'------ ',s,' ------');
     if readentry=ibstartsyms then
      begin
        Writeln(space,'Symtable datasize : ',getlongint);
        Writeln(space,'Symtable alignment: ',getlongint);
      end
     else
      Writeln('!! ibstartsym not found');
     repeat
       b:=readentry;
       case b of

         ibunitsym :
           readcommonsym('Unit symbol ');

         iblabelsym :
           readcommonsym('Label symbol ');

         ibtypesym :
           begin
             readcommonsym('Type symbol ');
             write(space,'  Result Type : ');
             readderef('');
           end;

         ibprocsym :
           begin
             readcommonsym('Procedure symbol ');
             len:=ppufile.getword;
             for i:=1 to len do
              begin
                write(space,'   Definition : ');
                readderef('');
              end;
           end;

         ibconstsym :
           begin
             readcommonsym('Constant symbol ');
             b:=getbyte;
             case tconsttyp(b) of
               constord :
                 begin
                   write  (space,'  OrdinalType : ');
                   readderef('');
                   writeln(space,'        Value : ',constexp.tostr(getexprint));
                 end;
               constpointer :
                 begin
                   write  (space,'  PointerType : ');
                   readderef('');
                   writeln(space,'        Value : ',getlongint)
                 end;
               conststring,
               constresourcestring :
                 begin
                   len:=getlongint;
                   getmem(pc,len+1);
                   getdata(pc^,len);
                   (pc+len)^:= #0;
                   writeln(space,'       Length : ',len);
                   writeln(space,'        Value : "',pc,'"');
                   freemem(pc,len+1);
                 end;
               constreal :
                 writeln(space,'        Value : ',getreal);
               constset :
                 begin
                   write (space,'      Set Type : ');
                   readderef('');
                   for i:=1to 4 do
                    begin
                      write (space,'        Value : ');
                      for j:=1to 8 do
                       begin
                         if j>1 then
                          write(',');
                         write(hexstr(getbyte,2));
                       end;
                      writeln;
                    end;
                 end;
               constwstring:
                 begin
                 end;
               constguid:
                 begin
                    getdata(guid,sizeof(guid));
                    write (space,'     IID String: {',hexstr(guid.d1,8),'-',hexstr(guid.d2,4),'-',hexstr(guid.d3,4),'-');
                    for i:=0 to 7 do
                      begin
                         write(hexstr(guid.d4[i],2));
                         if i=1 then write('-');
                      end;
                    writeln('}');
                 end
               else
                 Writeln ('!! Invalid unit format : Invalid const type encountered: ',b);
             end;
           end;

         ibabsolutevarsym :
           begin
             readabstractvarsym('Absolute variable symbol ',varoptions);
             Write (space,' Relocated to ');
             b:=getbyte;
             case absolutetyp(b) of
               tovar :
                 readpropaccesslist(space+'          Sym : ');
               toasm :
                 Writeln('Assembler name : ',getstring);
               toaddr :
                 begin
                   Write('Address : ',getlongint);
                   if tsystemcpu(ppufile.header.cpu)=cpu_i386 then
                     WriteLn(' (Far: ',getbyte<>0,')');
                 end;
               else
                 Writeln ('!! Invalid unit format : Invalid absolute type encountered: ',b);
             end;
           end;

         ibfieldvarsym :
           begin
             readabstractvarsym('Field Variable symbol ',varoptions);
             writeln(space,'      Address : ',getaint);
           end;

         ibstaticvarsym :
           begin
             readabstractvarsym('Global Variable symbol ',varoptions);
             write  (space,' DefaultConst : ');
             readderef('');
             if (vo_has_mangledname in varoptions) then
               writeln(space,' Mangledname : ',getstring);
           end;

         iblocalvarsym :
           begin
             readabstractvarsym('Local Variable symbol ',varoptions);
             write  (space,' DefaultConst : ');
             readderef('');
           end;

         ibparavarsym :
           begin
             readabstractvarsym('Parameter Variable symbol ',varoptions);
             write  (space,' DefaultConst : ');
             readderef('');
             writeln(space,'       ParaNr : ',getword);
             writeln(space,'        Univ  : ',boolean(getbyte));
             writeln(space,'     VarState : ',getbyte);
             if (vo_has_explicit_paraloc in varoptions) then
               begin
                 i:=getbyte;
                 getdata(tempbuf,i);
               end;
           end;

         ibenumsym :
           begin
             readcommonsym('Enumeration symbol ');
             write  (space,'   Definition : ');
             readderef('');
             writeln(space,'        Value : ',getlongint);
           end;

         ibsyssym :
           begin
             readcommonsym('Internal system symbol ');
             writeln(space,'  Internal Nr : ',getlongint);
           end;

         ibmacrosym :
           begin
             readcommonsym('Macro symbol ');
             writeln(space,'          Name: ',getstring);
             writeln(space,'       Defined: ',getbyte);
             writeln(space,'  Compiler var: ',getbyte);
             len:=getlongint;
             writeln(space,'  Value length: ',len);
             if len > 0 then
               begin
                 getmem(pc,len+1);
                 getdata(pc^,len);
                 (pc+len)^:= #0;
                 writeln(space,'         Value: "',pc,'"');
                 freemem(pc,len+1);
               end;
           end;

         ibpropertysym :
           begin
             readcommonsym('Property ');
             i:=getlongint;
             writeln(space,'  PropOptions : ',i);
             write  (space,' OverrideProp : ');
             readderef('');
             write  (space,'    Prop Type : ');
             readderef('');
             writeln(space,'        Index : ',getlongint);
             writeln(space,'      Default : ',getlongint);
             write  (space,'   Index Type : ');
             readderef('');
             { palt_none }
             readpropaccesslist('');
             write  (space,'   Readaccess : ');
             readpropaccesslist(space+'         Sym: ');
             write  (space,'  Writeaccess : ');
             readpropaccesslist(space+'         Sym: ');
             write  (space,' Storedaccess : ');
             readpropaccesslist(space+'         Sym: ');
           end;

         iberror :
           begin
             Writeln('!! Error in PPU');
             exit;
           end;

         ibendsyms :
           break;

         else
           WriteLn('!! Skipping unsupported PPU Entry in Symbols: ',b);
       end;
       if not EndOfEntry then
        Writeln('!! Entry has more information stored');
     until false;
   end;
end;


{****************************************************************************
                         Read defintions Part
****************************************************************************}

procedure readdefinitions(const s:string);
type
  tordtype = (
    uvoid,
    u8bit,u16bit,u32bit,u64bit,
    s8bit,s16bit,s32bit,s64bit,
    bool8bit,bool16bit,bool32bit,bool64bit,
    uchar,uwidechar,scurrency
  );
  tobjecttyp = (odt_none,
    odt_class,
    odt_object,
    odt_interfacecom,
    odt_interfacecom_property,
    odt_interfacecom_function,
    odt_interfacecorba,
    odt_cppclass,
    odt_dispinterface,
    odt_objcclass,
    odt_objcprotocol
  );
  tvarianttype = (
    vt_normalvariant,vt_olevariant
  );
var
  b : byte;
  l,j : longint;
  calloption : tproccalloption;
  procoptions : tprocoptions;
  procinfooptions : tprocinfoflag;
  defoptions: tdefoptions;
begin
  with ppufile do
   begin
     if space<>'' then
      Writeln(space,'------ ',s,' ------');
     if readentry<>ibstartdefs then
      Writeln('!! ibstartdefs not found');
     repeat
       b:=readentry;
       case b of

         ibpointerdef :
           begin
             readcommondef('Pointer definition',defoptions);
             write  (space,'     Pointed Type : ');
             readderef('');
             writeln(space,'           Is Far : ',(getbyte<>0));
             writeln(space,' Has Pointer Math : ',(getbyte<>0));
           end;

         iborddef :
           begin
             readcommondef('Ordinal definition',defoptions);
             write  (space,'        Base type : ');
             b:=getbyte;
             case tordtype(b) of
               uvoid     : writeln('uvoid');
               u8bit     : writeln('u8bit');
               u16bit    : writeln('u16bit');
               u32bit    : writeln('s32bit');
               u64bit    : writeln('u64bit');
               s8bit     : writeln('s8bit');
               s16bit    : writeln('s16bit');
               s32bit    : writeln('s32bit');
               s64bit    : writeln('s64bit');
               bool8bit  : writeln('bool8bit');
               bool16bit : writeln('bool16bit');
               bool32bit : writeln('bool32bit');
               bool64bit : writeln('bool64bit');
               uchar     : writeln('uchar');
               uwidechar : writeln('uwidechar');
               scurrency : writeln('ucurrency');
               else        writeln('!! Warning: Invalid base type ',b);
             end;
             writeln(space,'            Range : ',constexp.tostr(getexprint),' to ',constexp.tostr(getexprint));
           end;

         ibfloatdef :
           begin
             readcommondef('Float definition',defoptions);
             writeln(space,'       Float type : ',getbyte);
           end;

         ibarraydef :
           begin
             readcommondef('Array definition',defoptions);
             write  (space,'     Element type : ');
             readderef('');
             write  (space,'       Range Type : ');
             readderef('');
             writeln(space,'            Range : ',getaint,' to ',getaint);
             write  (space,'          Options : ');
             readarraydefoptions;
             readdefinitions('symbols');
             readsymbols('symbols');
           end;

         ibprocdef :
           begin
             readcommondef('Procedure definition',defoptions);
             read_abstract_proc_def(calloption,procoptions);
             if (po_has_mangledname in procoptions) then
               writeln(space,'     Mangled name : ',getstring);
             writeln(space,'           Number : ',getword);
             writeln(space,'            Level : ',getbyte);
             write  (space,'            Class : ');
             readderef('');
             write  (space,'          Procsym : ');
             readderef('');
             write  (space,'         File Pos : ');
             readposinfo;
             writeln(space,'       Visibility : ',Visibility2Str(ppufile.getbyte));
             write  (space,'       SymOptions : ');
             readsymoptions(space+'       ');
             if tsystemcpu(ppufile.header.cpu)=cpu_powerpc then
               begin
                 { library symbol for AmigaOS/MorphOS }
                 write  (space,'   Library symbol : ');
                 readderef('');
               end;
             if (po_has_importdll in procoptions) then
               writeln(space,'      Import DLL : ',getstring);
             if (po_has_importname in procoptions) then
               writeln(space,'      Import Name : ',getstring);
             writeln(space,'        Import Nr : ',getword);
             if (po_msgint in procoptions) then
               writeln(space,'           MsgInt : ',getlongint);
             if (po_msgstr in procoptions) then
               writeln(space,'           MsgStr : ',getstring);
             if (po_dispid in procoptions) then
               writeln(space,'      DispID: ',ppufile.getlongint);
             if (po_has_inlininginfo in procoptions) then
              begin
                write  (space,'       FuncretSym : ');
                readderef('');
                ppufile.getsmallset(procinfooptions);
                writeln(space,'  ProcInfoOptions : ',dword(procinfooptions));
              end;
             b:=ppufile.getbyte;
             if b<>0 then
               begin
                 write  (space,'       Alias names : ');
                 for j:=1 to b do
                   begin
                     write(ppufile.getstring);
                     if j<b then
                       write(', ');
                   end;
                 writeln;
               end;
             if not EndOfEntry then
              Writeln('!! Entry has more information stored');
             space:='    '+space;
             { parast }
             readdefinitions('parast');
             readsymbols('parast');
             { localst }
             if (po_has_inlininginfo in procoptions) then
              begin
                readdefinitions('localst');
                readsymbols('localst');
              end;
             if (po_has_inlininginfo in procoptions) then
               readnodetree;
             delete(space,1,4);
           end;

         ibprocvardef :
           begin
             readcommondef('Procedural type (ProcVar) definition',defoptions);
             read_abstract_proc_def(calloption,procoptions);
             writeln(space,'   Symtable level :',ppufile.getbyte);
             if not EndOfEntry then
              Writeln('!! Entry has more information stored');
             space:='    '+space;
             { parast }
             readdefinitions('parast');
             readsymbols('parast');
             delete(space,1,4);
           end;

         ibshortstringdef :
           begin
             readcommondef('ShortString definition',defoptions);
             writeln(space,'           Length : ',getbyte);
           end;

         ibwidestringdef :
           begin
             readcommondef('WideString definition',defoptions);
             writeln(space,'           Length : ',getlongint);
           end;
         
         ibunicodestringdef :
           begin
             readcommondef('UnicodeString definition',defoptions);
             writeln(space,'           Length : ',getlongint);
           end;

         ibansistringdef :
           begin
             readcommondef('AnsiString definition',defoptions);
             writeln(space,'           Length : ',getlongint);
           end;

         iblongstringdef :
           begin
             readcommondef('Longstring definition',defoptions);
             writeln(space,'           Length : ',getlongint);
           end;

         ibrecorddef :
           begin
             readcommondef('Record definition',defoptions);
             writeln(space,'   Name of Record : ',getstring);
             write  (space,'          Options : ');
             readobjectdefoptions;
             writeln(space,'       FieldAlign : ',getbyte);
             writeln(space,'      RecordAlign : ',getbyte);
             writeln(space,'         PadAlign : ',getbyte);
             writeln(space,'UseFieldAlignment : ',getbyte);
             writeln(space,'         DataSize : ',getaint);
             if not EndOfEntry then
              Writeln('!! Entry has more information stored');
             {read the record definitions and symbols}
             space:='    '+space;
             readdefinitions('fields');
             readsymbols('fields');
             Delete(space,1,4);
           end;

         ibobjectdef :
           begin
             readcommondef('Object/Class definition',defoptions);
             writeln(space,'    Name of Class : ',getstring);
             write  (space,'          Options : ');
             readobjectdefoptions;
             b:=getbyte;
             write  (space,'             Type : ');
             case tobjecttyp(b) of
               odt_class          : writeln('class');
               odt_object         : writeln('object');
               odt_interfacecom   : writeln('interfacecom');
               odt_interfacecorba : writeln('interfacecorba');
               odt_cppclass       : writeln('cppclass');
               odt_dispinterface  : writeln('dispinterface');
               odt_objcclass      : writeln('objcclass');
               odt_objcprotocol   : writeln('objcprotocol');
               else                 writeln('!! Warning: Invalid object type ',b);
             end;
             writeln(space,'    External name : ',getstring);
             writeln(space,'       Import lib : ',getstring);
             writeln(space,'         DataSize : ',getaint);
             writeln(space,'       FieldAlign : ',getbyte);
             writeln(space,'      RecordAlign : ',getbyte);
             writeln(space,'       Vmt offset : ',getlongint);
             write  (space,  '   Ancestor Class : ');
             readderef('');

             if tobjecttyp(b) in [odt_interfacecom,odt_interfacecorba,odt_dispinterface] then
               begin
                  { IIDGUID }
                  for j:=1to 16 do
                   getbyte;
                  writeln(space,'       IID String : ',getstring);
               end;

             l:=getlongint;
             writeln(space,'  VMT entries: ',l);
             for j:=1 to l do
               begin
                 write(space,'    ');
                 readderef('');
                 writeln(space,'      Visibility: ',Visibility2Str(getbyte));
               end;

             if tobjecttyp(b) in [odt_class,odt_interfacecorba,odt_objcclass,odt_objcprotocol] then
              begin
                l:=getlongint;
                writeln(space,'  Impl Intf Count : ',l);
                for j:=1 to l do
                 begin
                   write  (space,'  - Definition : ');
                   readderef('');
                   writeln(space,'       IOffset : ',getlongint);
                 end;
              end;

             if df_copied_def in current_defoptions then
               begin
                 writeln('  Copy of def: ');
                 readderef('');
               end;

             if not EndOfEntry then
              Writeln('!! Entry has more information stored');
             if not(df_copied_def in current_defoptions) then
               begin
                 {read the record definitions and symbols}
                 space:='    '+space;
                 readdefinitions('fields');
                 readsymbols('fields');
                 Delete(space,1,4);
              end;
           end;

         ibfiledef :
           begin
             ReadCommonDef('File definition',defoptions);
             write  (space,'             Type : ');
             case getbyte of
              0 : writeln('Text');
              1 : begin
                    writeln('Typed');
                    write  (space,'      File of Type : ');
                    readderef('');
                  end;
              2 : writeln('Untyped');
             end;
           end;

         ibformaldef :
           begin
<<<<<<< HEAD
<<<<<<< HEAD
             readcommondef('Generic definition (void-typ)',defoptions);
=======
             readcommondef('Generic definition (void-typ)');
>>>>>>> graemeg/fixes_2_2
=======
             readcommondef('Generic definition (void-typ)');
>>>>>>> origin/fixes_2_2
             writeln(space,'         Is Typed : ',(getbyte<>0));
           end;

         ibundefineddef :
           readcommondef('Undefined definition (generic parameter)',defoptions);

         ibenumdef :
           begin
             readcommondef('Enumeration type definition',defoptions);
             writeln(space,' Smallest element : ',getaint);
             writeln(space,'  Largest element : ',getaint);
             writeln(space,'             Size : ',getaint);
             if df_copied_def in defoptions then
               begin
                 write(space,'Base enumeration type : ');
                 readderef('');
               end
             else
               begin
                 space:='    '+space;
                 readdefinitions('elements');
                 readsymbols('elements');
                 delete(space,1,4);
               end;
           end;

         ibclassrefdef :
           begin
             readcommondef('Class reference definition',defoptions);
             write  (space,'    Pointed Type : ');
             readderef('');
           end;

         ibsetdef :
           begin
             readcommondef('Set definition',defoptions);
             write  (space,'     Element type : ');
             readderef('');
             writeln(space,'             Size : ',getaint);
             writeln(space,'         Set Base : ',getaint);
             writeln(space,'          Set Max : ',getaint);
           end;

         ibvariantdef :
           begin
             readcommondef('Variant definition',defoptions);
             write  (space,'      Varianttype : ');
             b:=getbyte;
             case tvarianttype(b) of
               vt_normalvariant :
                 writeln('Normal');
               vt_olevariant :
                 writeln('OLE');
               else
                 writeln('!! Warning: Invalid varianttype ',b);
             end;

           end;

         iberror :
           begin
             Writeln('!! Error in PPU');
             exit;
           end;

         ibenddefs :
           break;

         else
           WriteLn('!! Skipping unsupported PPU Entry in definitions: ',b);
       end;
       if not EndOfEntry then
        Writeln('!! Entry has more information stored');
     until false;
   end;
end;

procedure readmoduleoptions(space : string);
type
  tmoduleoption = (mo_none,
    mo_hint_deprecated,
    mo_hint_platform,
    mo_hint_library,
    mo_hint_unimplemented,
    mo_hint_experimental,
    mo_has_deprecated_msg
  );
  tmoduleoptions = set of tmoduleoption;
  tmoduleopt=record
    mask : tmoduleoption;
    str  : string[30];
  end;
const
  moduleopts=6;
  moduleopt : array[1..moduleopts] of tmoduleopt=(
     (mask:mo_hint_deprecated;    str:'Hint Deprecated'),
     (mask:mo_hint_platform;      str:'Hint Platform'),
     (mask:mo_hint_library;       str:'Hint Library'),
     (mask:mo_hint_unimplemented; str:'Hint Unimplemented'),
     (mask:mo_hint_experimental;  str:'Hint Experimental'),
     (mask:mo_has_deprecated_msg; str:'Has Deprecated Message')
  );
var
  moduleoptions : tmoduleoptions;
  i      : longint;
  first  : boolean;
begin
  ppufile.getsmallset(moduleoptions);
  if moduleoptions<>[] then
   begin
     first:=true;
     for i:=1to moduleopts do
      if (moduleopt[i].mask in moduleoptions) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(moduleopt[i].str);
       end;
   end;
  writeln;
  if mo_has_deprecated_msg in moduleoptions then
    writeln(space,'Deprecated : ', ppufile.getstring);
end;

{****************************************************************************
                           Read General Part
****************************************************************************}

procedure readinterface;
var
  b : byte;
  sourcenumber : longint;
begin
  with ppufile do
   begin
     repeat
       b:=readentry;
       case b of

         ibmodulename :
           Writeln('Module Name: ',getstring);

         ibmoduleoptions:
           readmoduleoptions('  ');

         ibsourcefiles :
           begin
             sourcenumber:=1;
             while not EndOfEntry do
              begin
                Writeln('Source file ',sourcenumber,' : ',getstring,' ',filetimestring(getlongint));
                inc(sourcenumber);
              end;
           end;
{$IFDEF MACRO_DIFF_HINT}
         ibusedmacros :
           begin
             while not EndOfEntry do
              begin
                Write('Conditional ',getstring);
                b:=getbyte;
                if boolean(b)=true then
                  write(' defined at startup')
                else
                  write(' not defined at startup');
                b:=getbyte;
                if boolean(b)=true then
                  writeln(' was used')
                else
                  writeln;
              end;
           end;
{$ENDIF}
         ibloadunit :
           ReadLoadUnit;

         iblinkunitofiles :
           ReadLinkContainer('Link unit object file: ');

         iblinkunitstaticlibs :
           ReadLinkContainer('Link unit static lib: ');

         iblinkunitsharedlibs :
           ReadLinkContainer('Link unit shared lib: ');

         iblinkotherofiles :
           ReadLinkContainer('Link other object file: ');

         iblinkotherstaticlibs :
           ReadLinkContainer('Link other static lib: ');

         iblinkothersharedlibs :
           ReadLinkContainer('Link other shared lib: ');

         iblinkotherframeworks:
           ReadLinkContainer('Link framework: ');

         ibmainname:
           Writeln('Specified main program symbol name: ',getstring);

         ibImportSymbols :
           ReadImportSymbols;

         ibderefdata :
           ReadDerefData;

         ibderefmap :
           ReadDerefMap;

         ibwpofile :
           ReadWpoFileInfo;

         ibresources :
           ReadLinkContainer('Resource file: ');

         iberror :
           begin
             Writeln('Error in PPU');
             exit;
           end;

         ibendinterface :
           break;

         else
           WriteLn('!! Skipping unsupported PPU Entry in General Part: ',b);
       end;
     until false;
   end;
end;



{****************************************************************************
                        Read Implementation Part
****************************************************************************}

procedure readimplementation;
var
  b : byte;
begin
  with ppufile do
   begin
     repeat
       b:=readentry;
       case b of
         ibasmsymbols :
           ReadAsmSymbols;

         ibloadunit :
           ReadLoadUnit;

         iberror :
           begin
             Writeln('Error in PPU');
             exit;
           end;
         ibendimplementation :
           break;
         else
           WriteLn('!! Skipping unsupported PPU Entry in Implementation: ',b);
       end;
     until false;
   end;
end;


procedure dofile (filename : string);
begin
{ reset }
  space:='';
{ fix filename }
  if pos('.',filename)=0 then
   filename:=filename+'.ppu';
  ppufile:=tppufile.create(filename);
  if not ppufile.openfile then
   begin
     writeln ('IO-Error when opening : ',filename,', Skipping');
     exit;
   end;
{ PPU File is open, check for PPU Id }
  if not ppufile.CheckPPUID then
   begin
     writeln(Filename,' : Not a valid PPU file, Skipping');
     exit;
   end;
{ Check PPU Version }
  Writeln('Analyzing ',filename,' (v',ppufile.GetPPUVersion,')');
  if ppufile.GetPPUVersion<16 then
   begin
     writeln(Filename,' : Old PPU Formats (<v16) are not supported, Skipping');
     exit;
   end;
{ Write PPU Header Information }
  if (verbose and v_header)<>0 then
   begin
     Writeln;
     Writeln('Header');
     Writeln('-------');
     with ppufile.header do
      begin
        Writeln('Compiler version        : ',ppufile.header.compiler shr 14,'.',
                                             (ppufile.header.compiler shr 7) and $7f,'.',
                                             ppufile.header.compiler and $7f);
        WriteLn('Target processor        : ',Cpu2Str(cpu));
        WriteLn('Target operating system : ',Target2Str(target));
        Writeln('Unit flags              : ',PPUFlags2Str(flags));
        Writeln('FileSize (w/o header)   : ',size);
        Writeln('Checksum                : ',hexstr(checksum,8));
        Writeln('Interface Checksum      : ',hexstr(interface_checksum,8));
        Writeln('Indirect Checksum       : ',hexstr(indirect_checksum,8));
        Writeln('Definitions stored      : ',tostr(deflistsize));
        Writeln('Symbols stored          : ',tostr(symlistsize));
      end;
   end;
{read the general stuff}
  if (verbose and v_interface)<>0 then
   begin
     Writeln;
     Writeln('Interface section');
     Writeln('------------------');
     readinterface;
   end
  else
   ppufile.skipuntilentry(ibendinterface);
{read the definitions}
  if (verbose and v_defs)<>0 then
   begin
     Writeln;
     Writeln('Interface definitions');
     Writeln('----------------------');
     readdefinitions('interface');
   end
  else
   ppufile.skipuntilentry(ibenddefs);
{read the symbols}
  if (verbose and v_syms)<>0 then
   begin
     Writeln;
     Writeln('Interface Symbols');
     Writeln('------------------');
     readsymbols('interface');
   end
  else
   ppufile.skipuntilentry(ibendsyms);

{read the macro symbols}
  if (verbose and v_syms)<>0 then
   begin
     Writeln;
     Writeln('Interface Macro Symbols');
     Writeln('-----------------------');
   end;
  if ppufile.readentry<>ibexportedmacros then
    begin
      Writeln('!! Error in PPU');
      exit;
    end;
  if boolean(ppufile.getbyte) then
    begin
      {skip the definition section for macros (since they are never used) }
      ppufile.skipuntilentry(ibenddefs);
      {read the macro symbols}
      if (verbose and v_syms)<>0 then
        readsymbols('interface macro')
      else
        ppufile.skipuntilentry(ibendsyms);
    end
  else
    Writeln('(no exported macros)');

{read the implementation stuff}
  if (verbose and v_implementation)<>0 then
   begin
     Writeln;
     Writeln('Implementation section');
     Writeln('-----------------------');
     readimplementation;
   end
  else
   ppufile.skipuntilentry(ibendimplementation);
  {read the static symtable}
  if (ppufile.header.flags and uf_local_symtable)<>0 then
   begin
     if (verbose and v_defs)<>0 then
      begin
        Writeln;
        Writeln('Static definitions');
        Writeln('----------------------');
        readdefinitions('implementation');
      end
     else
      ppufile.skipuntilentry(ibenddefs);
   {read the symbols}
     if (verbose and v_syms)<>0 then
      begin
        Writeln;
        Writeln('Static Symbols');
        Writeln('------------------');
        readsymbols('implementation');
      end
     else
      ppufile.skipuntilentry(ibendsyms);
   end;
  ReadCreatedObjTypes;
{shutdown ppufile}
  ppufile.closefile;
  ppufile.free;
  Writeln;
end;



procedure help;
begin
  writeln('usage: ppudump [options] <filename1> <filename2>...');
  writeln;
  writeln('[options] can be:');
  writeln('    -V<verbose>  Set verbosity to <verbose>');
  writeln('                   H - Show header info');
  writeln('                   I - Show interface');
  writeln('                   M - Show implementation');
  writeln('                   S - Show interface symbols');
  writeln('                   D - Show interface definitions');
//  writeln('                   B - Show browser info');
  writeln('                   A - Show all');
  writeln('    -h, -?       This helpscreen');
  halt;
end;

var
  startpara,
  nrfile,i  : longint;
  para      : string;
begin
  writeln(Title+' '+Version);
  writeln(Copyright);
  writeln;
  if paramcount<1 then
   begin
     writeln('usage: dumpppu [options] <filename1> <filename2>...');
     halt(1);
   end;
{ turn verbose on by default }
  verbose:=v_all;
{ read options }
  startpara:=1;
  while copy(paramstr(startpara),1,1)='-' do
   begin
     para:=paramstr(startpara);
     case upcase(para[2]) of
      'V' : begin
              verbose:=0;
              for i:=3 to length(para) do
               case upcase(para[i]) of
                'H' : verbose:=verbose or v_header;
                'I' : verbose:=verbose or v_interface;
                'M' : verbose:=verbose or v_implementation;
                'D' : verbose:=verbose or v_defs;
                'S' : verbose:=verbose or v_syms;
                'A' : verbose:=verbose or v_all;
               end;
            end;
      'H' : help;
      '?' : help;
     end;
     inc(startpara);
   end;
{ process files }
  for nrfile:=startpara to paramcount do
   dofile (paramstr(nrfile));
  if has_errors then
    Halt(1);
end.
