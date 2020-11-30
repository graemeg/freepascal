{
    Copyright (c) 1998-2020 by the FPC Development Team

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

{$i fpcdefs.inc}
{$H+}
{$packenum 1}

{$define IN_PPUDUMP}
uses
  { do NOT add symconst or globtype to make merging easier }
  { do include symconst and globtype now before splitting 2.5 PM 2011-06-15 }
  cutils,
  SysUtils,
  constexp,
  symconst,
  ppu,
  entfile,
  systems,
  cpuinfo,
  globals,
  globtype,
  widestr,
  tokens,
  version,
  ppuout,
  ppujson,
  ppuxml;

const
  Title     = 'PPU-Analyser';
  Copyright = 'Copyright (c) 1998-2020 by the Free Pascal Development Team';

{ verbosity }
  v_none           = $0;
  v_header         = $1;
  v_defs           = $2;
  v_syms           = $4;
  v_interface      = $8;
  v_implementation = $10;
//  v_browser        = $20;
  v_all            = $ff;

{ not needed anymore $i systems.inc }

{ List of all supported cpus }
const
  CpuTxt : array[tsystemcpu] of string[16]=
    (
    {  0 } 'none',
    {  1 } 'i386',
    {  2 } 'm68k',
    {  3 } 'alpha (obsolete)',
    {  4 } 'powerpc',
    {  5 } 'sparc',
    {  6 } 'vis (obsolete)',
    {  7 } 'ia64 (obsolete)',
    {  8 } 'x86_64',
    {  9 } 'mipseb',
    { 10 } 'arm',
    { 11 } 'powerpc64',
    { 12 } 'avr',
    { 13 } 'mipsel',
    { 14 } 'jvm',
    { 15 } 'i8086',
    { 16 } 'aarch64',
    { 17 } 'wasm',
    { 18 } 'sparc64',
    { 19 } 'riscv32',
    { 20 } 'riscv64',
    { 21 } 'xtensa',
    { 22 } 'z80'
    );

  CpuHasController : array[tsystemcpu] of boolean =
    (
    {  0 } false {'none'},
    {  1 } false {'i386'},
    {  2 } false {'m68k'},
    {  3 } false {'alpha (obsolete)'},
    {  4 } false {'powerpc'},
    {  5 } false {'sparc'},
    {  6 } false {'vis (obsolete)'},
    {  7 } false {'ia64 (obsolete)'},
    {  8 } false {'x86_64'},
    {  9 } false {'mipseb'},
    { 10 } true  {'arm'},
    { 11 } false {'powerpc64'},
    { 12 } true  {'avr'},
    { 13 } true  {'mipsel'},
    { 14 } false {'jvm'},
    { 15 } false {'i8086'},
    { 16 } false {'aarch64'},
    { 17 } false {'wasm'},
    { 18 } false {'sparc64'},
    { 19 } false {'riscv32'},
    { 20 } false {'riscv64'},
    { 21 } true  {'xtensa'},
    { 22 } true  {'z80'}
    );

{ List of all supported system-cpu couples }
const
  Targets : array[tsystem] of string[26]=(
  { 0 }   'none',
  { 1 }   'GO32V1 (obsolete)',
  { 2 }   'GO32V2',
  { 3 }   'Linux-i386',
  { 4 }   'OS/2',
  { 5 }   'Win32',
  { 6 }   'FreeBSD-i386',
  { 7 }   'Amiga',
  { 8 }   'Atari',
  { 9 }   'MacOSClassic-m68k',
  { 10 }  'Linux-m68k',
  { 11 }  'PalmOS-m68k',
  { 12 }  'Linux-alpha (obsolete)',
  { 13 }  'Linux-ppc',
  { 14 }  'MacOSClassic-ppc',
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
  { 25 }  'OpenBSD-m68k (obsolete)',
  { 26 }  'Linux-x86-64',
  { 27 }  'Darwin-ppc',
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
  { 39 }  'Win64-iA64 (obsolete)',
  { 40 }  'WinCE-i386',
  { 41 }  'Linux-x64',
  { 42 }  'GBA-arm',
  { 43 }  'Linux-powerpc64',
  { 44 }  'Darwin-i386',
  { 45 }  'PalmOS-arm',
  { 46 }  'Darwin-powerpc64',
  { 47 }  'NDS-arm',
  { 48 }  'Embedded-i386',
  { 49 }  'Embedded-m68k',
  { 50 }  'Embedded-alpha (obsolete)',
  { 51 }  'Embedded-powerpc',
  { 52 }  'Embedded-sparc',
  { 53 }  'Embedded-vm (obsolete)',
  { 54 }  'Embedded-iA64 (obsolete)',
  { 55 }  'Embedded-x64',
  { 56 }  'Embedded-mips',
  { 57 }  'Embedded-arm',
  { 58 }  'Embedded-powerpc64',
  { 59 }  'Symbian-i386',
  { 60 }  'Symbian-arm',
  { 61 }  'Darwin-x64',
  { 62 }  'Embedded-avr',
  { 63 }  'Haiku-i386',
  { 64 }  'iOS-ARM',
  { 65 }  'Solaris-x86-64',
  { 66 }  'Linux-MIPS',
  { 67 }  'Linux-MIPSel',
  { 68 }  'NativeNT-i386',
  { 69 }  'iPhoneSim-i386',
  { 70 }  'Wii-powerpc',
  { 71 }  'OpenBSD-x86-64',
  { 72 }  'NetBSD-x86-64',
  { 73 }  'AIX-powerpc',
  { 74 }  'AIX-powerpc64',
  { 75 }  'Java-JVM',
  { 76 }  'Android-JVM',
  { 77 }  'Android-arm',
  { 78 }  'Android-i386',
  { 79 }  'MSDOS-i8086',
  { 80 }  'Android-MIPSel',
  { 81 }  'Embedded-mipseb',
  { 82 }  'Embedded-mipsel',
  { 83 }  'AROS-i386',
  { 84 }  'AROS-x86-64',
  { 85 }  'DragonFly-x86-64',
  { 86 }  'iOS-AArch64',
  { 87 }  'iPhoneSim-x86-64',
  { 88 }  'Linux-AArch64',
  { 89 }  'Win16',
  { 90 }  'Embedded-i8086',
  { 91 }  'AROS-arm',
  { 92 }  'WebAssembly-wasm',
  { 93 }  'Linux-sparc64',
  { 94 }  'Solaris-sparc64',
  { 95 }  'NetBSD-arm',
  { 96 }  'Linux-RiscV32',
  { 97 }  'Linux-RiscV64',
  { 98 }  'Embedded-RiscV32',
  { 99 }  'Embedded-RiscV64',
  { 100 } 'Android-AArch64',
  { 101 } 'Android-x86-64',
  { 102 } 'Haiku-x86-64',
  { 103 } 'Embedded-Xtensa',
  { 104 } 'FreeRTos-Xtensa',
  { 105 } 'Linux-Xtensa',
  { 106 } 'FreeRTos-arm',
  { 107 } 'Win64-AArch64',
  { 108 } 'Embedded-Z80',
  { 109 } 'ZXSpectrum-Z80',
  { 110 } 'MSX-DOS-Z80',
  { 111 } 'Darwin-AArch64',
  { 112 } 'AmstradCPC-Z80',
  { 113 } 'SinclairQL-m68k'
  );

const
{ in widestr, we have the following definition
  type
       tcompilerwidechar = word;
  thus widecharsize seems to always be 2 bytes }

  widecharsize : longint = 2;
  cpu : tsystemcpu = cpu_no;

{ This type is defined in scanner.pas unit }
type
  tspecialgenerictoken = (
    ST_LOADSETTINGS,
    ST_LINE,
    ST_COLUMN,
    ST_FILEINDEX,
    ST_LOADMESSAGES,
    ST_INVALID);

type
  tcpu_i386 = (
       cpu_variant_i386_none,
       cpu_variant_386,
       cpu_variant_486,
       cpu_variant_Pentium,
       cpu_variant_Pentium2,
       cpu_variant_Pentium3,
       cpu_variant_Pentium4,
       cpu_variant_PentiumM,
       cpu_variant_core_i,
       cpu_variant_core_avx,
       cpu_variant_core_avx2);

  tcpu_m68k = (
       cpu_variant_m68k_none,
       cpu_variant_MC68000,
       cpu_variant_MC68020,
       cpu_variant_MC68040,
       cpu_variant_MC68060,
       cpu_variant_isa_a,
       cpu_variant_isa_a_p,
       cpu_variant_isa_b,
       cpu_variant_isa_c,
       cpu_variant_cfv4e
      );

  tcpu_powerpc = (
       cpu_variant_powerpc_none,
       cpu_variant_ppc604,
       cpu_variant_ppc750,
       cpu_variant_ppc7400,
       cpu_variant_ppc970
      );

  tcpu_sparc = (
    cpu_variant_sparc_none,
    cpu_variant_SPARC_V7,
    cpu_variant_SPARC_V8,
    cpu_variant_SPARC_V9
  );

  tcpu_x86_64 = (
       cpu_variant_x86_64_none,
       cpu_variant_athlon64,
       cpu_variant_x86_64_core_i,
       cpu_variant_x86_64_core_avx,
       cpu_variant_x86_64_core_avx2
      );

  tcpu_mipseb = (
       cpu_variant_mipseb_none,
       cpu_variant_mips1,
       cpu_variant_mips2,
       cpu_variant_mips3,
       cpu_variant_mips4,
       cpu_variant_mips5,
       cpu_variant_mips32,
       cpu_variant_mips32r2,
       cpu_variant_pic32mx
      );

  tcpu_arm = (
       cpu_variant_arm_none,
       cpu_variant_armv3,
       cpu_variant_armv4,
       cpu_variant_armv4t,
       cpu_variant_armv5,
       cpu_variant_armv5t,
       cpu_variant_armv5te,
       cpu_variant_armv5tej,
       cpu_variant_armv6,
       cpu_variant_armv6k,
       cpu_variant_armv6t2,
       cpu_variant_armv6z,
       cpu_variant_armv6m,
       cpu_variant_armv7,
       cpu_variant_armv7a,
       cpu_variant_armv7r,
       cpu_variant_armv7m,
       cpu_variant_armv7em
      );

  tcpu_powerpc64 = (
    cpu_variant_powerpc64_none,
    cpu_variant_powerpc64_ppc970
    );

  tcpu_avr = (
       cpu_variant_avr_none,
       cpu_variant_avr1,
       cpu_variant_avr2,
       cpu_variant_avr25,
       cpu_variant_avr3,
       cpu_variant_avr31,
       cpu_variant_avr35,
       cpu_variant_avr4,
       cpu_variant_avr5,
       cpu_variant_avr51,
       cpu_variant_avr6
      );

  tcpu_mipsel = tcpu_mipseb;

  tcpu_jvm = (
       cpu_variant_jvm_none,
       { jvm, same as cpu_none }
       cpu_variant_jvm,
       { jvm byte code to be translated into Dalvik bytecode: more type-
         sensitive }
       cpu_variant_dalvik
      );

  tcpu_i8086 = (
       cpu_variant_i8086_none,
       cpu_variant_8086,
       cpu_variant_186,
       cpu_variant_286,
       cpu_variant_i8086_386,
       cpu_variant_i8086_486,
       cpu_variant_i8086_Pentium,
       cpu_variant_i8086_Pentium2,
       cpu_variant_i8086_Pentium3,
       cpu_variant_i8086_Pentium4,
       cpu_variant_i8086_PentiumM
      );

  tcpu_aarch64 = (
       cpu_variant_aarch64_none,
       cpu_variant_armv8
      );

  tcpu_wasm = (
       cpu_variant_wasm_none);

  tcpu_sparc64 = (
    cpu_variant_sparc64_none,
    cpu_variant_SPARC64_V9
  );

  tcpu_riscv32 = (
       cpu_variant_riscv32_none,
       cpu_variant_rv32imafd,
       cpu_variant_rv32ima,
       cpu_variant_rv32im,
       cpu_variant_rv32i
      );

  tcpu_riscv64 = (
    cpu_variant_riscv64_none,
    cpu_variant_rv64imafdc,
    cpu_variant_rv64imafd,
    cpu_variant_rv64ima,
    cpu_variant_rv64im,
    cpu_variant_rv64i
  );

  tcpu_type = record
     case tsystemcpu of
       cpu_no:                      { 0 }
          ();
       cpu_i386:                    { 1 }
          (cpu_i386 : tcpu_i386;);
       cpu_m68k:                    { 2 }
          (cpu_m68k : tcpu_m68k;);
       obsolete_cpu_alpha:          { 3 }
          ();
       cpu_powerpc:                 { 4 }
          (cpu_powerpc : tcpu_powerpc;);
       cpu_sparc:                   { 5 }
          (cpu_sparc : tcpu_sparc;);
       obsolete_cpu_vm:             { 6 }
          ();
       obsolete_cpu_ia64:           { 7 }
          ();
       cpu_x86_64:                  { 8 }
          (cpu_x86_64 : tcpu_x86_64;);
       cpu_mipseb:                  { 9 }
          (cpu_mipseb : tcpu_mipseb;);
       cpu_arm:                     { 10 }
          (cpu_arm : tcpu_arm;);
       cpu_powerpc64:               { 11 }
          (cpu_powerpc64 : tcpu_powerpc64;);
       cpu_avr:                     { 12 }
          (cpu_avr : tcpu_avr;);
       cpu_mipsel:                  { 13 }
          (cpu_mipsel : tcpu_mipsel;);
       cpu_jvm:                     { 14 }
          (cpu_jvm : tcpu_jvm;);
       cpu_i8086:                   { 15 }
          (cpu_i8086 : tcpu_i8086;);
       cpu_aarch64:                 { 16 }
          (cpu_aarch64 : tcpu_aarch64;);
       cpu_wasm:                    { 17 }
          (cpu_wasm : tcpu_wasm;);
       cpu_sparc64:                 { 18 }
          (cpu_sparc64 : tcpu_sparc64;);
       cpu_riscv32:                 { 19 }
          (cpu_riscv32 : tcpu_riscv32;);
       cpu_riscv64:                 { 20 }
          (cpu_riscv64 : tcpu_riscv64;);
     end;


  TPpuModuleDef = class(TPpuUnitDef)
    ModuleFlags: tmoduleflags;
  end;

type
  tppudumpfile = class(tppufile)
  protected
    procedure RaiseAssertion(Code: Longint); override;
  end;

var
  ppufile     : tppudumpfile;
  ppuversion  : dword;
  space       : string;
  verbose     : longint;
  derefdata   : pbyte;
  derefdatalen : longint;
  pout: TPpuOutput;
  nostdout: boolean;
  UnitList: TPpuContainerDef;
  CurUnit: TPpuModuleDef;
  SkipVersionCheck: boolean;
  SymAnsiStr: boolean;

var
  { needed during tobjectdef parsing... }
  current_defoptions : tdefoptions;
  current_objectoptions : tobjectoptions;
  current_symtable_options : tsymtableoptions;

{****************************************************************************
                          Helper Routines
****************************************************************************}

{****************************************************************************
                          Routine to read 80-bit reals
****************************************************************************
}
{$PUSH}
{$WARN 6018 OFF} { Turn off unreachable code warning }
{ On platforms with sizeof(ext) <> 10 the code below will cause an unreachable
  code warning, which will cause compilation failures with -Sew (KB) }
type
  TSplit80bitReal = packed record
    case byte of
      0: (bytes: Array[0..9] of byte);
      1: (words: Array[0..4] of word);
{$ifdef FPC_LITTLE_ENDIAN}
      2: (cards: Array[0..1] of cardinal; w: word);
{$else not FPC_LITTLE_ENDIAN}
      2: (w:word; cards: Array[0..1] of cardinal);
{$endif not FPC_LITTLE_ENDIAN}
  end;
const
  maxDigits = 17;
  function Real80bitToStr(var e : TSplit80bitReal;var ext : extended) : string;
  var
    Temp : string;
    new : TSplit80bitReal;
    fraczero, expmaximal, sign, outside_double : boolean;
    exp : smallint;
    d : double;
    i : longint;
    mantval : qword;
  begin
    if ppufile.change_endian then
      begin
        for i:=0 to 9 do
          new.bytes[i]:=e.bytes[9-i];
        e:=new;
      end;
    if sizeof(ext)=10 then
      begin
        ext:=pextended(@e)^;
        str(ext,result);
        exit;
      end;
    { extended, format (MSB): 1 Sign bit, 15 bit exponent, 64 bit mantissa }
    sign := (e.w and $8000) <> 0;
    expMaximal := (e.w and $7fff) = 32767;
    exp:=(e.w and $7fff) - 16383 - 63;
    fraczero := (e.cards[0] = 0) and
                    ((e.cards[1] and $7fffffff) = 0);
{$ifdef FPC_LITTLE_ENDIAN}
    mantval := qword(e.cards[0]) or (qword(e.cards[1]) shl 32);
{$else not FPC_LITTLE_ENDIAN}
    mantval := (qword(e.cards[0]) shl 32) or qword(e.cards[1]);
{$endif not FPC_LITTLE_ENDIAN}
    if expMaximal then
      if fraczero then
        if sign then
          temp := '-Inf'
        else temp := '+Inf'
      else temp := 'Nan'
    else
      begin
        d:=double(mantval);
        if sign then
          d:=-d;
        outside_double:=false;
        Try
          if exp > 0 then
            begin
              for i:=1 to exp do
                d:=d *2.0;
            end
          else if exp < 0 then
            begin
              for i:=1 to -exp do
                d:=d /2.0;
            end;
        Except
          outside_double:=true;
        end;
      if (mantval<>0) and (d=0.0) then
        outside_double:=true;
      if outside_double then
        begin
          Temp:='Extended value outside double bound';
          ext:=0.0;
        end
      else
        begin
          ext:=d;
          system.str(d,temp);
        end;
      end;

    result:=temp;
  end;
{$POP}

const has_errors : boolean = false;
      has_warnings : boolean = false;
      has_more_infos : boolean = false;

procedure SetHasErrors;
begin
  has_errors:=true;
end;

Procedure WriteError(const S : string);
Begin
  system.Writeln(StdErr, S);
  SetHasErrors;
End;

procedure StrAppend(var st : string; const st2 : string);
begin
  st:=st+st2;
end;

procedure tppudumpfile.RaiseAssertion(Code: Longint);
begin
  WriteError('Internal Error ' + ToStr(Code));
  inherited RaiseAssertion(Code);
end;

Procedure WriteWarning(const S : string);
var
  ss: string;
Begin
  ss:='!! Warning: ' + S;
  if nostdout then
    system.Writeln(StdErr, ss)
  else
    system.Writeln(ss);
  has_warnings:=true;
End;

procedure Write(const s: string);
begin
  if nostdout then exit;
  system.write(s);
end;

procedure Write(const params: array of const);
var
  i: integer;
  { Last vtType define in rtl/inc/objpash.inc }
const
  max_vttype = vtUnicodeString;
begin
  if nostdout then exit;
  for i:=Low(params) to High(params) do
  { All vtType in
        vtInteger       = 0;
        vtBoolean       = 1;
        vtChar          = 2;
        vtExtended      = 3;
        vtString        = 4;
        vtPointer       = 5;
        vtPChar         = 6;
        vtObject        = 7;
        vtClass         = 8;
        vtWideChar      = 9;
        vtPWideChar     = 10;
        vtAnsiString32  = 11; called vtAnsiString in objpas unit
        vtCurrency      = 12;
        vtVariant       = 13;
        vtInterface     = 14;
        vtWideString    = 15;
        vtInt64         = 16;
        vtQWord         = 17;
        vtUnicodeString = 18;
        // vtAnsiString16  = 19; not yet used
        // vtAnsiString64  = 20; not yet used
    }
    with TVarRec(params[i]) do
      case VType of
        vtInteger: system.write(VInteger);
        vtBoolean: system.write(VBoolean);
        vtChar: system.write(VChar);
        vtExtended: system.write(VExtended^);
        vtString: system.write(VString^);
        vtPointer:
          begin
            { Not sure the display will be correct
              if sizeof pointer is not native }
            WriteWarning('Pointer constant');
          end;
        vtPChar: system.write(VPChar);
        vtObject:
          begin
            { Not sure the display will be correct
              if sizeof pointer is not native }
            WriteWarning('Object constant');
          end;
        vtClass:
          begin
            { Not sure the display will be correct
              if sizeof pointer is not native }
            WriteWarning('Class constant');
          end;
        vtWideChar: system.write(VWideChar);
        vtPWideChar:
          begin
            WriteWarning('PWideChar constant');
          end;
        vtAnsiString: system.write(ansistring(VAnsiString));
        vtCurrency : system.write(VCurrency^);
        vtVariant :
          begin
            { Not sure the display will be correct
              if sizeof pointer is not native }
            WriteWarning('Variant constant');
          end;
        vtInterface :
          begin
            { Not sure the display will be correct
              if sizeof pointer is not native }
            WriteWarning('Interface constant');
          end;
        vtWideString : system.write(widestring(VWideString));
        vtInt64: system.write(VInt64^);
        vtQWord: system.write(VQWord^);
        vtUnicodeString : system.write(unicodestring(VUnicodeString));
        else
          begin
            system.writeln;
            system.writeln('Unsupported var type: ', VType);
            Halt(10);
          end;
      end;
end;

procedure Writeln(const s: string = '');
begin
  if nostdout then exit;
  system.writeln(s);
end;

procedure Writeln(const params: array of const);
begin
  if nostdout then exit;
  Write(params);
  system.writeln;
end;

Procedure HasMoreInfos;
begin
  Writeln('!! Entry has more information stored');
  has_more_infos:=true;
end;

function Unknown(const st : string; val :longint) : string;
Begin
  Unknown:='<!! Unknown'+st+' value '+tostr(val)+'>';
  SetHasErrors;
end;

function ToStr(w:longint):String;
begin
  Str(w,ToStr);
end;

Function Target2Str(w:longint):string;
begin
  if w<=ord(high(tsystem)) then
    Target2Str:=Targets[tsystem(w)]
  else
    Target2Str:=Unknown('target',w);
end;


Function Cpu2Str(w:longint):string;
begin
  if w<=ord(high(tsystemcpu)) then
    begin
      cpu:=tsystemcpu(w);
      Cpu2Str:=CpuTxt[cpu];
    end
  else
    Cpu2Str:=Unknown('cpu',w);
end;


Function Varspez2Str(w:longint):string;
const
  { in symconst unit
    tvarspez = (vs_value,vs_const,vs_var,vs_out,vs_constref); }
  varspezstr : array[tvarspez] of string[8]=('Value','Const','Var','Out','ConstRef','Final');
begin
  if w<=ord(high(varspezstr)) then
    Varspez2Str:=varspezstr[tvarspez(w)]
  else
    Varspez2Str:=Unknown('varspez',w);
end;

Function VarRegable2Str(w:longint):string;
  { tvarregable type is defined in symconst unit }
const
  varregableStr : array[tvarregable] of string[6]=('None','IntReg','FPUReg','MMReg','Addr');
begin
  if w<=ord(high(varregablestr)) then
    Varregable2Str:=varregablestr[tvarregable(w)]
  else
    Varregable2Str:=Unknown('regable',w);
end;


Function Visibility2Str(w:longint):string;
const
  { tvisibility type is defined in symconst unit }

  visibilityName : array[tvisibility] of string[16] = (
    'hidden','strict private','private','strict protected','protected',
    'public','published','<none>'
  );
begin
  if w<=ord(high(visibilityName)) then
    result:=visibilityName[tvisibility(w)]
  else
    result:=Unknown('visibility',w);
end;


Function IntfEntryType2Str(w:longint):string;
const
  { tinterfaceentrytype type is defined in symconst unit }

  Name : array[tinterfaceentrytype] of string = (
    'standard','virtual method result','static method result','field value','virtual method class',
    'static method class','field value class'
  );
begin
  if w<=ord(high(Name)) then
    result:=Name[tinterfaceentrytype(w)]
  else
    result:=Unknown('entry type',w);
end;


function PPUFlags2Str(flags:dword):string;
type
  tflagopt=record
    mask : dword;
    str  : string[30];
  end;
const
  flagopts=8;
  flagopt : array[1..flagopts] of tflagopt=(
    (mask: $4    ;str:'big_endian'),
//    (mask: $10   ;str:'browser'),
    (mask: $20   ;str:'in_library'),
    (mask: $40   ;str:'smart_linked'),
    (mask: $80   ;str:'static_linked'),
    (mask: $100  ;str:'shared_linked'),
    (mask: $400  ;str:'no_link'),
    (mask: $1000  ;str:'little_endian'),
    (mask: $8000  ;str:'fpu_emulation_on')
  );
var
  i : longint;
  ntflags : dword;
  first  : boolean;
  s : string;
begin
  s:='';
  ntflags:=flags;
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
         ntflags:=ntflags and (not flagopt[i].mask);
       end;
   end
  else
   s:='none';
  if ntflags<>0 then
    begin
      s:=s+' unknown '+hexstr(ntflags,8);
      SetHasErrors;
    end;
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
          SetHasErrors;
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

function readsymstr(ppufile: tppufile): ansistring;
  begin
    if not(mf_symansistr in CurUnit.ModuleFlags) then
      result:=ppufile.getstring
    else
      result:=ppufile.getansistring;
  end;

function readmanagementoperatoroptions(const space : string;const name : string):tmanagementoperators;forward;

procedure readrecsymtableoptions;
var
  usefieldalignment : shortint;
begin
  if ppufile.readentry<>ibrecsymtableoptions then
    begin
      SetHasErrors;
      exit;
    end;
  writeln([space,' recordalignment: ',shortint(ppufile.getbyte)]);
  usefieldalignment:=shortint(ppufile.getbyte);
  writeln([space,' usefieldalignment: ',usefieldalignment]);
  writeln([space,' recordalignmin: ',shortint(ppufile.getbyte)]);
  if (usefieldalignment=C_alignment) then
    writeln([space,' fieldalignment: ',shortint(ppufile.getbyte)]);
  readmanagementoperatoroptions(space,'Fields have MOPs');
end;

function readsymtableoptions(const s: string) : tsymtableoptions;
type
  tsymtblopt=record
    mask : tsymtableoption;
    str  : string[30];
  end;
const
  symtblopts=ord(high(tsymtableoption))  + 1;
  symtblopt : array[1..symtblopts] of tsymtblopt=(
     (mask:sto_has_helper;   str:'Has helper'),
     (mask:sto_has_generic;  str:'Has generic'),
     (mask:sto_has_operator; str:'Has operator'),
     (mask:sto_needs_init_final;str:'Needs init final table'),
     (mask:sto_has_non_trivial_init;str:'Has non trivial init')
  );
var
  options : tsymtableoptions;
  first : boolean;
  i : integer;
begin
  if ppufile.readentry<>ibsymtableoptions then
    begin
      SetHasErrors;
      exit;
    end;
  ppufile.getset(tppuset1(options));
  if space<>'' then
   writeln([space,'------ ',s,' ------']);
  write([space,'Symtable options: ']);
  if options<>[] then
   begin
     first:=true;
     for i:=1 to symtblopts do
      if (symtblopt[i].mask in options) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(symtblopt[i].str);
       end;
   end
  else
   write('none');
  writeln;
  readsymtableoptions:=options;
end;

procedure readdefinitions(const s:string; ParentDef: TPpuContainerDef); forward;
procedure readsymbols(const s:string; ParentDef: TPpuContainerDef = nil); forward;

procedure readsymtable(const s: string; ParentDef: TPpuContainerDef = nil);
var
  stored_symtable_options : tsymtableoptions;
begin
  stored_symtable_options:=current_symtable_options;
  current_symtable_options:=readsymtableoptions(s);
  readdefinitions(s, ParentDef);
  readsymbols(s, ParentDef);
  current_symtable_options:=stored_symtable_options;
end;

procedure readrecordsymtable(const s: string; ParentDef: TPpuContainerDef = nil);
begin
  readrecsymtableoptions;
  readsymtable(s, ParentDef);
end;

Procedure ReadLinkContainer(const prefix:string);
{
  Read a serie of strings and write to the screen starting every line
  with prefix
}
  function maskstr(m:longint):string;
    { link options are in globtype unit
  const
    link_none    = $0;
    link_always  = $1;
    link_static  = $2;
    link_smart   = $4;
    link_shared  = $8; }
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
     WriteLn([prefix,s,' (',maskstr(m),')']);
   end;
end;


Procedure ReadContainer(const prefix:string);
{
  Read a series of strings and write to the screen starting every line
  with prefix
}
begin
  while not ppufile.endofentry do
   WriteLn([prefix,ppufile.getstring]);
end;


procedure ReadLoadUnit;
var
  ucrc,uintfcrc, indcrc : cardinal;
  un: TPpuUnitDef;
begin
  while not ppufile.EndOfEntry do
    begin
      un:=TPpuUnitDef.Create(CurUnit.UsedUnits);
      un.Name:=ppufile.getstring;
      write(['Uses unit: ',un.Name]);
      ucrc:=cardinal(ppufile.getlongint);
      uintfcrc:=cardinal(ppufile.getlongint);
      indcrc:=cardinal(ppufile.getlongint);
      writeln([' (Crc: ',hexstr(ucrc,8),', IntfcCrc: ',hexstr(uintfcrc,8),', IndCrc: ',hexstr(indcrc,8),')']);
      un.Crc:=ucrc;
      un.IntfCrc:=uintfcrc;
    end;
end;


Procedure ReadDerefmap;
var
  i,mapsize : longint;
  s: string;
begin
  mapsize:=ppufile.getlongint;
  writeln(['DerefMapsize: ',mapsize]);
  SetLength(CurUnit.RefUnits, mapsize);
  for i:=0 to mapsize-1 do
    begin
      s:=ppufile.getstring;
      writeln(['DerefMap[',i,'] = ',s]);
      CurUnit.RefUnits[i]:=LowerCase(s);
    end;
end;


Procedure ReadImportSymbols;
var
  extlibname  : string;
  j,
  extsymcnt   : longint;
  extsymname  : string;
  extsymmangledname  : string;
  extsymordnr : longint;
  extsymisvar : boolean;
begin
  while not ppufile.endofentry do
    begin
      extlibname:=ppufile.getstring;
      extsymcnt:=ppufile.getlongint;
      writeln(['External Library: ',extlibname,' (',extsymcnt,' imports)']);
      for j:=0 to extsymcnt-1 do
        begin
          extsymname:=ppufile.getstring;
          extsymmangledname:=ppufile.getstring;
          extsymordnr:=ppufile.getlongint;
          extsymisvar:=ppufile.getbyte<>0;
          writeln([' ',extsymname,' as ',extsymmangledname,
            '(OrdNr: ',extsymordnr,' IsVar: ',extsymisvar,')']);
        end;
    end;
end;


Procedure ReadDerefdata;
begin
  derefdatalen:=ppufile.entrysize;
  if derefdatalen=0 then
    begin
      Writeln(['No Derefdata length=0']);
      derefdata:=nil;
      exit;
    end;
  Writeln(['Derefdata length: ',derefdatalen]);
  derefdata:=allocmem(derefdatalen);
  ppufile.getdata(derefdata^,derefdatalen);
end;

Procedure FreeDerefdata;
begin
  if assigned(derefdata) then
    begin
      FreeMem(derefdata);
      derefdata:=nil;
      derefdatalen:=0;
    end;
end;


Procedure ReadWpoFileInfo;
begin
  Writeln(['Compiled with input whole-program optimisation from ',ppufile.getstring,' ',filetimestring(ppufile.getlongint)]);
end;


Procedure ReadAsmSymbols;
const
  unitasmlisttype: array[tunitasmlisttype] of string[6]=(
    'PUBLIC',
    'EXTERN'
  );
type
  { Copied from aasmbase.pas }
       TAsmsymbind=(
         AB_NONE,AB_EXTERNAL,AB_COMMON,AB_LOCAL,AB_GLOBAL,AB_WEAK_EXTERNAL,
         { global in the current program/library, but not visible outside it }
         AB_PRIVATE_EXTERN,AB_LAZY,AB_IMPORT,
         { a symbol that's internal to the compiler and used as a temp }
         AB_TEMP,
         { a global symbol that points to another global symbol and is only used
           to allow indirect loading in case of packages and indirect imports }
         AB_INDIRECT,AB_EXTERNAL_INDIRECT);

       TAsmsymtype=(
         AT_NONE,AT_FUNCTION,AT_DATA,AT_SECTION,AT_LABEL,
         {
           the address of this code label is taken somewhere in the code
           so it must be taken care of it when creating pic
         }
         AT_ADDR,
         { Label for debug or other non-program information }
         AT_METADATA,
         { label for data that must always be accessed indirectly, because it
           is handled explcitely in the system unit or (e.g. RTTI and threadvar
           tables) -- never seen in an assembler/assembler writer, always
           changed to AT_DATA }
         AT_DATA_FORCEINDIRECT,
         { don't generate an implicit indirect symbol as that might be provided
           by other means (e.g. the typed const builder) to ensure a correct
           section name }
         AT_DATA_NOINDIRECT,
         { Thread-local symbol (ELF targets) }
         AT_TLS,
         { GNU indirect function (ELF targets) }
         AT_GNU_IFUNC
         );

var
  s,
  bindstr,
  typestr  : string;
  i : longint;
  t: tunitasmlisttype;
begin
  writeln([space,'Assembler Symbols']);
  writeln([space,'-----------------']);
  t:=tunitasmlisttype(ppufile.getbyte);
  if (t>=Low(tunitasmlisttype)) and (t<=High(tunitasmlisttype)) then
    typestr:=unitasmlisttype[t]
  else
    typestr:='UNKNOWN';
  writeln([space,'Type: ',typestr]);
  writeln([space,'Count: ',ppufile.getlongint]);
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
       AB_WEAK_EXTERNAL :
         bindstr:='Weak external';
       AB_PRIVATE_EXTERN :
         bindstr:='Private extern';
       AB_LAZY :
         bindstr:='Lazy';
       AB_IMPORT :
         bindstr:='Import';
       AB_TEMP :
         bindstr:='Temp';
       AB_INDIRECT :
         bindstr:='Indirect';
       AB_EXTERNAL_INDIRECT :
         bindstr:='Indirect external';
       else
         begin
           bindstr:='<Error !!>';
           SetHasErrors;
         end;
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
       AT_ADDR :
         typestr:='Label (with address taken)';
       AT_METADATA :
         typestr:='Metadata';
       { this shouldn't appear in a PPU }
       AT_DATA_FORCEINDIRECT :
         typestr:='Data (ForceIndirect)';
       { this shouldn't appear in a PPU }
       AT_DATA_NOINDIRECT:
         typestr:='Data (NoIndirect)';
       AT_TLS :
         typestr:='TLS';
       AT_GNU_IFUNC :
         typestr:='GNU IFUNC';
       else
         begin
           typestr:='<Error !!>';
           SetHasErrors;
         end;
     end;
     Writeln([space,'  ',i,' : ',s,' [',bindstr,',',typestr,']']);
     inc(i);
   end;
  writeln([space]);
end;

function getexprint:Tconstexprint;

begin
  getexprint.overflow:=false;
  getexprint.signed:=ppufile.getboolean;
  getexprint.svalue:=ppufile.getint64;
end;

Procedure ReadPosInfo(Def: TPpuDef = nil);
var
  info : byte;
  fileindex,line,column : longint;
begin
  with ppufile do
   begin
     fileindex:=0;
     line:=0;
     column:=0;
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
     Writeln([fileindex,' (',line,',',column,')']);
     if Def <> nil then
       begin
         Def.FilePos.FileIndex:=fileindex;
         Def.FilePos.Line:=line;
         Def.FilePos.Col:=column;
       end;
   end;
end;


procedure readderef(const derefspace: string; Ref: TPpuRef = nil);
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
  if idx = -1 then
    begin
      writeln('Nil');
      exit;
    end;
  if (idx>derefdatalen) then
    begin
      WriteError('!! Error: Deref idx '+IntToStr(idx)+' > '+IntToStr(derefdatalen));
      exit;
    end;
  write([derefspace,'(',idx,') ']);
  pdata:=@derefdata[idx];
  i:=0;
  n:=pdata[i];
  inc(i);
  if n<1 then
    begin
      WriteError('!! Error: Deref len < 1');
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
           write(['SymId ',idx]);
           if Ref <> nil then
             Ref.Id:=idx;
         end;
       deref_defid :
         begin
           idx:=pdata[i] shl 24 or pdata[i+1] shl 16 or pdata[i+2] shl 8 or pdata[i+3];
           inc(i,4);
           write(['DefId ',idx]);
           if Ref <> nil then
             Ref.Id:=idx;
         end;
       deref_unit :
         begin
           idx:=pdata[i] shl 8 or pdata[i+1];
           inc(i,2);
           write(['Unit ',idx]);
           if Ref <> nil then
             Ref.UnitIndex:=idx;
         end;
       else
         begin
           WriteError('!! unsupported dereftyp: '+IntToStr(ord(b)));
           break;
         end;
     end;
   end;
  writeln;
end;


Procedure ReadUnitImportSyms;
var
  c,i : longint;
begin
  writeln([space,'Imported Symbols']);
  writeln([space,'----------------']);
  c:=ppufile.getlongint;
  for i:=0 to c-1 do
    readderef(space);
  writeln([space]);
end;


procedure readpropaccesslist(const s:string; Ref: TPpuRef = nil);
{ type tsltype is in symconst unit }
const
  slstr : array[tsltype] of string[12] = (
    '',
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
  readderef('',Ref);
  repeat
    sl:=tsltype(ppufile.getbyte);
    if sl=sl_none then
     break;
    write([s,'(',slstr[sl],') ']);
    case sl of
      sl_none : ;
      sl_call,
      sl_load,
      sl_subscript :
        if (Ref <> nil) and (Ref.IsNull) then
         begin
           readderef('',Ref);
           Ref.IsSymId:=True;
         end
        else
          readderef('');
      sl_absolutetype,
      sl_typeconv :
        readderef('');
      sl_vec :
        begin
          writeln([ppufile.getlongint]);
          readderef('');
        end;
    end;
  until false;
end;

(*
       talignmentinfo = packed record
         procalign,
         loopalign,
         jumpalign,
         constalignmin,
         constalignmax,
         varalignmin,
         varalignmax,
         localalignmin,
         localalignmax,
         recordalignmin,
         recordalignmax,
         maxCrecordalign : longint;
       end;


       tsettings = packed record
         alignment       : talignmentinfo;
         globalswitches  : tglobalswitches;
         moduleswitches  : tmoduleswitches;
         localswitches   : tlocalswitches;
         modeswitches    : tmodeswitches;
         optimizerswitches : toptimizerswitches;
         { generate information necessary to perform these wpo's during a subsequent compilation }
         genwpoptimizerswitches: twpoptimizerswitches;
         { perform these wpo's using information generated during a previous compilation }
         dowpoptimizerswitches: twpoptimizerswitches;
         debugswitches   : tdebugswitches;
         { 0: old behaviour for sets <=256 elements
           >0: round to this size }
         setalloc,
         packenum        : shortint;

         packrecords     : shortint;
         maxfpuregisters : shortint;

         cputype,
         optimizecputype : tcputype;
         fputype         : tfputype;
         asmmode         : tasmmode;
         interfacetype   : tinterfacetypes;
         defproccall     : tproccalloption;
         sourcecodepage  : tcodepagestring;

         minfpconstprec  : tfloattype;

         disabledircache : boolean;

        { CPU targets with microcontroller support can add a controller specific unit }
        controllertype   : tcontrollertype;
         { WARNING: this pointer cannot be written as such in record token }
         pmessage : pmessagestaterecord;
       end;

*)
procedure readprocinfooptions(space : string);
(*
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
         { set if the procedure has at least one label }
         pi_has_label,
         { calls itself recursive }
         pi_is_recursive,
         { stack frame optimization not possible (only on x86 probably) }
         pi_needs_stackframe,
         { set if the procedure has at least one register saved on the stack }
         pi_has_saved_regs,
         { dfa was generated for this proc }
         pi_dfaavailable,
         { subroutine contains interprocedural used labels }
         pi_has_interproclabel,
         { subroutine contains interprocedural gotos }
         pi_has_global_goto
       ); *)

type
  tprocinfoopt=record
    mask : tprocinfoflag;
    str  : string[81];
  end;
const
  procinfoopts=ord(high(tprocinfoflag)) - ord(low(tprocinfoflag));
  procinfoopt : array[0..procinfoopts] of tprocinfoopt=(
         (mask:pi_has_assembler_block;
         str:' has at least one assembler block'),
         (mask:pi_do_call;
         str:' does a call'),
         (mask:pi_uses_exceptions;
         str:' has a try statement = no register optimization '),
         (mask:pi_is_assembler;
         str:' is declared as @var(assembler), don''t optimize'),
         (mask:pi_needs_implicit_finally;
         str:' contains data which needs to be finalized '),
         (mask:pi_has_implicit_finally;
         str:' has the implicit try..finally generated '),
         (mask:pi_uses_fpu;
         str:' uses fpu'),
         (mask:pi_needs_got;
         str:' uses GOT for PIC code '),
         (mask:pi_uses_static_symtable;
         str:' references var/proc/type/const in static symtable'),
         (mask:pi_has_stackparameter;
         str:' set if the procedure has to push parameters onto the stack '),
         (mask:pi_has_label;
         str:' set if the procedure has at least one label '),
         (mask:pi_is_recursive;
         str:' calls itself recursive '),
         (mask:pi_needs_stackframe;
         str:' stack frame optimization not possible (only on x86 probably) '),
         (mask:pi_has_saved_regs;
         str:' set if the procedure has at least one register saved on the stack '),
         (mask:pi_dfaavailable;
         str:' dfa was generated for this proc '),
         (mask:pi_has_interproclabel;
         str:' subroutine contains interprocedural used labels '),
         (mask:pi_has_unwind_info;
         str:' unwinding info was generated for this proc '),
         (mask:pi_has_global_goto;
         str:' subroutine contains interprocedural goto '),
         (mask:pi_has_inherited;
         str:' subroutine contains inherited call '),
         (mask:pi_has_nested_exit;
         str:' subroutine contains a nested subroutine which calls the exit of the current one '),
         (mask:pi_has_stack_allocs;
         str:' allocates memory on stack, so stack may be unbalanced on exit '),
         (mask:pi_estimatestacksize;
         str:' stack size is estimated before subroutine is compiled '),
         (mask:pi_calls_c_varargs;
         str:' calls function with C-style varargs '),
         (mask:pi_has_open_array_parameter;
         str:' has open array parameter '),
         (mask:pi_uses_threadvar;
         str:' uses threadvars '),
         (mask:pi_has_except_table_data;
         str:' has except table data '),
         (mask:pi_needs_tls;
         str:' uses TLS data pointer '),
         (mask:pi_uses_get_frame;
         str:' uses get_frame'),
         (mask:pi_uses_ymm;
         str:' uses ymm register (x86 only)')
  );
var
  procinfooptions : tprocinfoflags;
  i      : longint;
  first  : boolean;
begin
  ppufile.getset(tppuset4(procinfooptions));
  if procinfooptions<>[] then
   begin
     first:=true;
     for i:=0 to procinfoopts do
      if (procinfoopt[i].mask in procinfooptions) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(procinfoopt[i].str);
       end;
   end;
  writeln;
end;

procedure readsymoptions(space : string; Def: TPpuDef = nil);
type
  tsymopt=record
    mask : tsymoption;
    str  : string[30];
  end;
const
  symopts=ord(high(tsymoption)) - ord(low(tsymoption));
  { sp_none = 0 corresponds to nothing }
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
     (mask:sp_has_deprecated_msg; str:'Has Deprecated Message'),
     (mask:sp_generic_dummy;      str:'Generic Dummy'),
     (mask:sp_explicitrename;     str:'Explicit Rename'),
     (mask:sp_generic_const;      str:'Generic Constant Parameter')
  );
var
  symoptions : tsymoptions;
  i      : longint;
  first  : boolean;
begin
  ppufile.getset(tppuset2(symoptions));
  if symoptions<>[] then
   begin
     if Def <> nil then
       if sp_internal in symoptions then
         Def.Visibility:=dvHidden;
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
    writeln([space,'Deprecated : ', ppufile.getstring]);
end;

procedure readvisibility(Def: TPpuDef = nil);
var
  i: byte;
begin
  i:=ppufile.getbyte;
  if Def <> nil then
    case tvisibility(i) of
      vis_public: Def.Visibility:=dvPublic;
      vis_published: Def.Visibility:=dvPublished;
      vis_protected, vis_strictprotected: Def.Visibility:=dvProtected;
      else Def.Visibility:=dvPrivate;
    end;
  writeln(Visibility2Str(i));
end;

procedure readattrs(def: TPpuDef);
var
  i,cnt,paras: longint;
begin
  cnt:=ppufile.getlongint;
  if cnt>0 then
    begin
      writeln([space,'   Attributes : ']);
      space:='    '+space;
      if assigned(def) then
        SetLength(def.Attrs,cnt);
      for i:=0 to cnt-1 do
        begin
          writeln([space,'** Custom Attribute ',i,' **']);
          write  ([space,'      Type symbol : ']);
          if assigned(def) then
            begin
              def.Attrs[i].TypeSym:=TPpuRef.Create;
              readderef('',def.Attrs[i].TypeSym);
            end
          else
            readderef('');
          write  ([space,' Type constructor : ']);
          if assigned(def) then
            begin
              def.Attrs[i].TypeConstr:=TPpuRef.Create;
              readderef('',def.Attrs[i].TypeConstr);
            end
          else
            readderef('');
          paras:=ppufile.getlongint;
          writeln([space,'       Parameters : ',paras]);
          if assigned(def) then
            def.Attrs[i].ParaCount:=paras;
        end;
      delete(space,1,4);
    end;
end;

procedure readnodetree; forward;

procedure readattrparas(def: TPpuDef);
var
  attr,para: LongInt;
begin
  if Length(def.Attrs) > 0 then
    writeln([space,'   Attr Paras : ']);
  space:='    '+space;
  for attr:=0 to High(def.Attrs) do
    begin
      writeln([space,'** Custom Attribute ',attr,' Arguments **']);
      space:='    '+space;
      for para:=0 to def.Attrs[attr].ParaCount-1 do
        begin
          readnodetree;
        end;
      delete(space,1,4);
    end;
  delete(space,1,4);
end;

procedure readdefsubentries(def: TPpuDef);
begin
  space:='    '+space;
  readattrparas(def);
  delete(space,1,4);
end;

procedure readsymsubentries(def: TPpuDef);
begin
  readattrparas(def);
end;

procedure readcommonsym(const s:string; Def: TPpuDef = nil);
var
  i: integer;
  n: ansistring;
begin
  n:=readsymstr(ppufile);
  if Def <> nil then
    Def.Name:=n;
  i:=ppufile.getlongint;
  if Def <> nil then
    Def.SetSymId(i);
  writeln([space,'** Symbol Id ',i,' **']);
  writeln([space,s,n]);
  write  ([space,'     File Pos : ']);
  readposinfo(Def);
  write  ([space,'   Visibility : ']);
  readvisibility(Def);
  write  ([space,'   SymOptions : ']);
  readsymoptions(space+'   ',Def);
  readattrs(Def);
end;


procedure readcgpara(const space:string);
{ this is originally in cgbase.pas }
type
  TCGLoc=(LOC_INVALID, LOC_VOID, LOC_CONSTANT, LOC_JUMP, LOC_FLAGS,
          LOC_REGISTER, LOC_CREGISTER, LOC_FPUREGISTER, LOC_CFPUREGISTER,
          LOC_MMXREGISTER, LOC_CMMXREGISTER, LOC_MMREGISTER, LOC_CMMREGISTER,
          LOC_SUBSETREG, LOC_CSUBSETREG, LOC_SUBSETREF, LOC_CSUBSETREF,
          LOC_CREFERENCE, LOC_REFERENCE);

const
  tcgloc2str : array[TCGLoc] of string[12] = (
         'LOC_INVALID', 'LOC_VOID', 'LOC_CONST', 'LOC_JUMP', 'LOC_FLAGS',
         'LOC_REG', 'LOC_CREG', 'LOC_FPUREG', 'LOC_CFPUREG',
         'LOC_MMXREG', 'LOC_CMMXREG', 'LOC_MMREG', 'LOC_CMMREG',
         'LOC_SSETREG', 'LOC_CSSETREG', 'LOC_SSETREF', 'LOC_CSSETREF',
         'LOC_CREF', 'LOC_REF');
var
  i: byte;
  ii: longint;
  np: byte;
  loc: tcgloc;
begin
  i:=ppufile.getbyte;
  writeln([space,'   Alignment : ',i]);
  i:=ppufile.getbyte;
  writeln([space,'        Size : ',i]);
  ii:=ppufile.getaint;
  writeln([space,'     IntSize : ',ii]);
  readderef(space+'  ');
  np:=ppufile.getbyte;
  writeln([space,'  NumParaloc : ',np]);
  while np > 0 do
    begin
      i:=ppufile.getbyte;
      writeln([space,'     Paraloc Size : ',i]);
      loc:=tcgloc(ppufile.getbyte);
      if loc > high(tcgloc) then
        begin
          WriteError('!! Location is out of range! '+IntToStr(ord(loc)));
          loc:=LOC_INVALID;
        end;
      writeln([space,'     Paraloc Loc  : (',ord(loc),') ',tcgloc2str[loc]]);
      case loc of
        LOC_REFERENCE:
          begin
            writeln([space,'     RegIndex : $',hexstr(ppufile.getdword,8)]);
            writeln([space,'       Offset : ',ppufile.getaint]);
          end;
        LOC_FPUREGISTER,
        LOC_CFPUREGISTER,
        LOC_MMREGISTER,
        LOC_CMMREGISTER,
        LOC_REGISTER,
        LOC_CREGISTER :
          begin
            writeln([space,'     ShiftVal : ',ppufile.getbyte]);
            writeln([space,'     Register : $',hexstr(ppufile.getdword,8)]);
          end;
        LOC_VOID:
          begin end
        else
          WriteError('!! Invalid location error')
      end;
      dec(np);
    end;
end;

procedure displaytokenbuffer(tokenbuf : pbyte;tokenbufsize : longint);
type
  ptoken=^ttoken;
  pmsgstate =^tmsgstate;
var
  tbi : longint;
  state : tmsgstate;
  prev_settings, new_settings : Tsettings;
  nb, msgvalue, mesgnb : longint;


  function readtoken: ttoken;
    var
      b,b2 : byte;
    begin
      b:=tokenbuf[tbi];
      inc(tbi);
      if (b and $80)<>0 then
        begin
          b2:=tokenbuf[tbi];
          inc(tbi);
          result:=ttoken(((b and $7f) shl 8) or b2);
        end
      else
        result:=ttoken(b);
    end;

  function gettokenbufdword : dword;
  var
    var32 : dword;
  begin
    var32:=unaligned(pdword(@tokenbuf[tbi])^);
    inc(tbi,sizeof(dword));
    if ppufile.change_endian then
      var32:=swapendian(var32);
    result:=var32;
  end;

  function gettokenbufword : word;
  var
    var16 : word;
  begin
    var16:=unaligned(pword(@tokenbuf[tbi])^);
    inc(tbi,sizeof(word));
    if ppufile.change_endian then
      var16:=swapendian(var16);
    result:=var16;
  end;

  function gettokenbuflongint : longint;
  var
    var32 : longint;
  begin
    var32:=unaligned(plongint(@tokenbuf[tbi])^);
    inc(tbi,sizeof(longint));
    if ppufile.change_endian then
      var32:=swapendian(var32);
    result:=var32;
  end;

  function gettokenbufshortint : shortint;
  var
    var8 : shortint;
  begin
    var8:=pshortint(@tokenbuf[tbi])^;
    inc(tbi,sizeof(shortint));
    result:=var8;
  end;

  procedure tokenreadset(var b;size : longint);
  var
    i : longint;
  begin
    move(tokenbuf[tbi],b,size);
    inc(tbi,size);
    if ppufile.change_endian then
      for i:=0 to size-1 do
        Pbyte(@b)[i]:=reverse_byte(Pbyte(@b)[i]);
  end;

  function gettokenbufbyte : byte;
  begin
    result:=pbyte(@tokenbuf[tbi])^;
    inc(tbi);
  end;

   function tokenreadenum(size : longint) : longword;
  begin
    if size=1 then
      result:=gettokenbufbyte
    else if size=2 then
      result:=gettokenbufword
    else if size=4 then
      result:=gettokenbufdword;
  end;



  function gettokenbufsizeint : int64;
  var
    var64 : int64;
    var32 : longint;
    var16 : smallint;

  begin
    if CpuAddrBitSize[cpu]=64 then
      begin
        var64:=unaligned(pint64(@tokenbuf[tbi])^);
        inc(tbi,sizeof(int64));
        if ppufile.change_endian then
          var64:=swapendian(var64);
        result:=var64;
      end
    else if CpuAddrBitSize[cpu]=32 then
      begin
        var32:=unaligned(plongint(@tokenbuf[tbi])^);
        inc(tbi,sizeof(longint));
        if ppufile.change_endian then
          var32:=swapendian(var32);
        result:=var32;
      end
    else if CpuAddrBitSize[cpu]=16 then
      begin
        { ASizeInt is still a longint, see globtype.pas unit }
        var32:=unaligned(plongint(@tokenbuf[tbi])^);
        inc(tbi,sizeof(longint));
        if ppufile.change_endian then
          var32:=swapendian(var32);
        result:=var32;
      end
    else
      begin
        WriteError('Wrong CpuAddrBitSize');
        result:=0;
      end;
  end;

  procedure tokenreadsettings(var asettings : tsettings; expected_size : asizeint);

    {    This procedure
       needs to be changed whenever
       globals.tsettings type is changed,
       the problem is that no error will appear
       before tests with generics are tested. PM }

       var
         startpos, endpos : longword;
      begin
        { WARNING all those fields need to be in the correct
        order otherwise cross_endian PPU reading will fail }
        startpos:=tbi;
        with asettings do
          begin
            alignment.procalign:=gettokenbuflongint;
            alignment.loopalign:=gettokenbuflongint;
            alignment.jumpalign:=gettokenbuflongint;
            alignment.jumpalignskipmax:=gettokenbuflongint;
            alignment.coalescealign:=gettokenbuflongint;
            alignment.coalescealignskipmax:=gettokenbuflongint;
            alignment.constalignmin:=gettokenbuflongint;
            alignment.constalignmax:=gettokenbuflongint;
            alignment.varalignmin:=gettokenbuflongint;
            alignment.varalignmax:=gettokenbuflongint;
            alignment.localalignmin:=gettokenbuflongint;
            alignment.localalignmax:=gettokenbuflongint;
            alignment.recordalignmin:=gettokenbuflongint;
            alignment.recordalignmax:=gettokenbuflongint;
            alignment.maxCrecordalign:=gettokenbuflongint;
            tokenreadset(globalswitches,sizeof(globalswitches));
            tokenreadset(targetswitches,sizeof(targetswitches));
            tokenreadset(moduleswitches,sizeof(moduleswitches));
            tokenreadset(localswitches,sizeof(localswitches));
            tokenreadset(modeswitches,sizeof(modeswitches));
            tokenreadset(optimizerswitches,sizeof(optimizerswitches));
            tokenreadset(genwpoptimizerswitches,sizeof(genwpoptimizerswitches));
            tokenreadset(dowpoptimizerswitches,sizeof(dowpoptimizerswitches));
            tokenreadset(debugswitches,sizeof(debugswitches));
            { 0: old behaviour for sets <=256 elements
              >0: round to this size }
            setalloc:=gettokenbufshortint;
            packenum:=gettokenbufshortint;

            packrecords:=gettokenbufshortint;
            maxfpuregisters:=gettokenbufshortint;

            cputype:=tcputype(tokenreadenum(sizeof(tcputype)));
            optimizecputype:=tcputype(tokenreadenum(sizeof(tcputype)));
            fputype:=tfputype(tokenreadenum(sizeof(tfputype)));
            asmmode:=tasmmode(tokenreadenum(sizeof(tasmmode)));
            interfacetype:=tinterfacetypes(tokenreadenum(sizeof(tinterfacetypes)));
            defproccall:=tproccalloption(tokenreadenum(sizeof(tproccalloption)));
            { tstringencoding is word type,
              thus this should be OK here }
            sourcecodepage:=tstringEncoding(gettokenbufword);

            minfpconstprec:=tfloattype(tokenreadenum(sizeof(tfloattype)));

            disabledircache:=boolean(gettokenbufbyte);

            tlsmodel:=ttlsmodel(tokenreadenum(sizeof(ttlsmodel)));
{ TH: Since the field was conditional originally, it was not stored in PPUs.  }
{ While adding ControllerSupport constant, I decided not to store ct_none     }
{ on targets not supporting controllers, but this might be changed here and   }
{ in tokenwritesettings in the future to unify the PPU structure and handling }
{ of this field in the compiler.                                              }
{$PUSH}
 {$WARN 6018 OFF} (* Unreachable code due to compile time evaluation *)
            if CpuHasController[cpu] then
             controllertype:=tcontrollertype(tokenreadenum(sizeof(tcontrollertype)))
            else
             ControllerType:=ct_none;
{$POP}
           endpos:=tbi;
           if endpos-startpos<>expected_size then
             Writeln(['Wrong size of Settings read-in: ',expected_size,' expected, but got ',endpos-startpos]);
         end;
     end;
  procedure dump_new_settings;
(*     tsettings = record
         alignment       : talignmentinfo;
         globalswitches  : tglobalswitches;
         targetswitches  : ttargetswitches;
         moduleswitches  : tmoduleswitches;
         localswitches   : tlocalswitches;
         modeswitches    : tmodeswitches;
         optimizerswitches : toptimizerswitches;
         { generate information necessary to perform these wpo's during a subsequent compilation }
         genwpoptimizerswitches: twpoptimizerswitches;
         { perform these wpo's using information generated during a previous compilation }
         dowpoptimizerswitches: twpoptimizerswitches;
         debugswitches   : tdebugswitches;
         { 0: old behaviour for sets <=256 elements
           >0: round to this size }
         setalloc,
         packenum        : shortint;

         packrecords     : shortint;
         maxfpuregisters : shortint;

         cputype,
         optimizecputype,
         asmcputype      : tcputype;
         fputype         : tfputype;
         asmmode         : tasmmode;
         interfacetype   : tinterfacetypes;
         defproccall     : tproccalloption;
         sourcecodepage  : tstringencoding;

         minfpconstprec  : tfloattype;

         disabledircache : boolean;

         tlsmodel : ttlsmodel;

{$if defined(i8086)}
         x86memorymodel  : tx86memorymodel;
{$endif defined(i8086)}

{$if defined(ARM)}
         instructionset : tinstructionset;
{$endif defined(ARM)}

{$if defined(LLVM) and not defined(GENERIC_CPU)}
         llvmversion: tllvmversion;
{$endif defined(LLVM) and not defined(GENERIC_CPU)}

        { CPU targets with microcontroller support can add a controller specific unit }
         controllertype   : tcontrollertype;

         { WARNING: this pointer cannot be written as such in record token }
         pmessage : pmessagestaterecord;
       end; *)

const
    targetswitchname : array[ttargetswitch] of string[30] =
       { global target-specific switches }
       ('Target None', {ts_none}
         { generate code that results in smaller TOCs than normal (AIX) }
        'Small TOC', {ts_small_toc}
         { for the JVM target: generate integer array initializations via string
           constants in order to reduce the generated code size (Java routines
           are limited to 64kb of bytecode) }
        'JVM compact int array init', {ts_compact_int_array_init}
         { for the JVM target: intialize enum fields in constructors with the
           enum class instance corresponding to ordinal value 0 (not done by
           default because this initialization can only be performed after the
           inherited constructors have run, and if they call a virtual method
           of the current class, then this virtual method may already have
           initialized that field with another value and the constructor
           initialization will result in data loss }
        'JVM enum field init', {ts_jvm_enum_field_init}
         { when automatically generating getters/setters for properties, use
           these strings as prefixes for the generated getters/setter names }
        'Auto getter prefix', {ts_auto_getter_prefix}
        'Auto setter prefix', {ts_auto_setter_predix}
        'Thumb interworking', {ts_thumb_interworking,}
         { lowercase the first character of routine names, used to generate
           names that are compliant with Java coding standards from code
           written according to Delphi coding standards }
        'LowerCase proc start', {ts_lowercase_proc_start,}
         { initialise local variables on the JVM target so you won't get
           accidental uses of uninitialised values }
        'Init locals', {ts_init_locals}
         { emit a CLD instruction before using the x86 string instructions }
        'Emit CLD instruction', {ts_cld}
         { increment BP before pushing it in the function prologue and decrement
           it after popping it in the function epilogue, iff the function is
           going to terminate with a far ret. Thus, the BP value pushed on the
           stack becomes odd if the function is far and even if the function is
           near. This allows walking the BP chain on the stack and e.g.
           obtaining a stack trace even if the program uses a mixture of near
           and far calls. This is also required for Win16 real mode, because it
           allows Windows to move code segments around (in order to defragment
           memory) and then walk through the stacks of all running programs and
           update the segment values of the segment that has moved. }
        'Use odd BP for far procs' {ts_x86_far_procs_push_odd_bp}
       );
    moduleswitchname : array[tmoduleswitch] of string[40] =
       ('Module None', {cs_modulenone,}
         { parser }
        'Floating Point Emulation',{ cs_fp_emulation}
        'Extended syntax', {cs_extsyntax}
        'Open string', {cs_openstring}
         { support }
        'Goto allowed', {cs_support_goto}
        'Macro support', {cs_support_macro}
        'C operator support', {cs_support_c_operators}
         { generation }
        'Profile', {cs_profile}
        'Debug information', {cs_debuginfo}
        'Compilation of System unit', {cs_compilesystem}
        'Line information', {cs_lineinfo}
        'Implicit exceptions', {cs_implicit_exceptions}
        'Explicit CodePage', {cs_explicit_codepage}
        'System CodePage', {cs_system_codepage}
         { linking }
        'Create smart units', {cs_create_smart}
        'Create dynamic', {cs_create_dynamic}
        'Create PIC code', {cs_create_pic}
         { browser switches are back }
        'Browser', {cs_browser}
        'Local Browser', {cs_local_browser}
         { target specific }
        'Executable Stack', {cs_executable_stack}
         { i8086 specific }
        'Huge code', {cs_huge_code}
        'Win16 smart callbacks', {cs_win16_smartcallbacks}
         { Record usage of checkpointer experimental feature }
        'CheckPointer used', {cs_checkpointer_called}
        'Supports LLVM Link-Time Optimization' {cs_lto}
       );
    globalswitchname : array[tglobalswitch] of string[50] =
       ('Global None',{cs_globalnone}
         { parameter switches }
        'Check unit name', {cs_check_unit_name}
        'Constructor name', {cs_constructor_name}
        'Support exceptions',{cs_support_exceptions}
        'Support Objective-C pas',{ cs_support_c_objectivepas}
        'Transparent file names', {cs_transparent_file_names}
         { units }
        'Load Objpas Unit', {cs_load_objpas_unit}
        'Load GPC unit', {cs_load_gpc_unit}
        'Load FPCKylix unit', {cs_load_fpcylix_unit}
        'Support Vectors', {cs_support_vectors}
         { debuginfo }
        'Use HeapTRc unit', {cs_use_heaptrc}
        'Use line information', {cs_use_lineinfo}
        'Use GDB Valgrind', {cs_gdb_valgrind}
        'No regalloc', {cs_no_regalloc}
        'Stabs preserve cases', {cs_stabs_preservecase}
         { assembling }
        'Leave assembler file', {cs_asm_leave}
        'Use external assembler', {cs_asm_extern}
        'Use pipes to call assembler', {cs_asm_pipe}
        'Add source infos into assembler files', {cs_asm_source}
        'Add register allocation into assembler files', {cs_asm_regalloc}
        'Add temporary  allocation into assmebler files', {cs_asm_tempalloc}
        'Add node information into assembler files', {cs_asm_nodes}
        'Adapt assembler call to GNU version <= 2.25', {cs_asm_pre_binutils_2_25}
         { linking }
        'Skip linking stage', {cs_link_nolink}
        'Link static', {cs_link_static}
        'Link smart', {cs_link_smart}
        'Link shared', {cs_link_shared}
        'Link deffile', {cs_link_deffile}
        'Strip after linking', {cs_link_strip}
        'Use linker static flag',{cs_link_staticflag}
        'Link on target OS',{cs_link_on_target}
        'Use external linker', {cs_link_extern}
        'Link opt vtable', {cs_link_opt_vtable}
        'Link opt used sections', {cs_link_opt_used_sections}
        'Link debug to separate file',{cs_link_separate_dbg_file}
        'Create linker map', {cs_link_map}
        'Link to pthread', {cs_link_pthread}
        'Link no default lib order', {cs_link_no_default_lib_order}
        'Link using native linker', {cs_link_native}
        'Link for GNU linker version <=2.19', {cs_link_pre_binutils_2_19}
        'Link using vlink', {cs_link_vlink}
        'Link-Time Optimization disabled for system unit', {cs_lto_nosystem}
        'Assemble on target OS', {cs_asemble_on_target}
        'Use a memory model to support >2GB static data on 64 Bit target' {cs_large}
       );
    localswitchname : array[tlocalswitch] of string[50] =
       { Switches which can be changed locally }
       ('Local None', {cs_localnone}
         { codegen }
        'Check overflow', {cs_check_overflow}
        'Check range', {cs_check_range}
        'Check object error', {cs_check_object}
        'Check I/O error', {cs_check_io}
        'Check stack', {cs_check_stack}
        'Check pointer', {cs_checkpointer}
        'Check ordinal size', {cs_check_ordinal_size}
        'Generate stackframes', {cs_generate_stackframes}
        'Do assertions', {cs_do_assertion}
        'Generate RTTI', {cs_generate_rtti}
        'Full boolean evaluaion', {cs_full_boolean_eval}
        'Typed constant are writable', {cs_typed_const_writable}
        'Allow calcuation on enum types', {cs_allow_enum_calc}
        'Do inline', {cs_do_inline}
        'Add FWAIT instruction for FPU 8087', {cs_fpu_fwait}
        'IEEE errors', {cs_ieee_errors}
        'Check low address loading', {cs_check_low_addr_load}
        'Imported data', {cs_imported_data}
        'Excess precision', {cs_excessprecision}
        'Check fpu exceptions', {cs_check_fpu_exceptions}
        'Check all case coverage', {cs_check_all_case_coverage}
         { mmx }
        'Allow MMX instructions', {cs_mmx}
        'Use MMX saturation', {cs_mmx_saturation}
         { parser }
        'Use typed addresses', {cs_typed_addresses}
        'Use strict var strings', {cs_strict_var_strings}
        'Use reference counted strings', {cs_refcountedstrings}
        'Use bit-packing', {cs_bitpacking}
        'Use var property setter', {cs_varpropsetter}
        'Use scoped enums',{cs_scopedenums}
        'Use pointer math', {cs_pointermath}
         { macpas specific}
        'MACPAS exteranl variable', {cs_external_var}
        'MACPAS externally visible', {cs_externally_visible}
         { jvm specific }
        'JVM check var copyout', {cs_check_var_copyout}
        'Zero based strings', {cs_zerobasedstrings}
         { i8086 specific }
        'i8086 force FAR calls', {cs_force_far_calls}
        'i8086 huge pointer arithmetic', {cs_hugeptr_arithmetic_normalization}
        'i8086 huge pointer comparison' {cs_hugeptr_comparison_normalization}
       );
       { Switches which can be changed by a mode (fpc,tp7,delphi) }
       modeswitchname : array[tmodeswitch] of string[50] =
        ('m_none',
         { generic }
         'm_fpc','m_objfpc','m_delphi','m_tp7','m_mac','m_iso','m_extpas',
         {$ifdef gpc_mode}'m_gpc',{$endif}
         { more specific }
         'm_class',               { delphi class model }
         'm_objpas',              { load objpas unit }
         'm_result',              { result in functions }
         'm_string_pchar',        { pchar 2 string conversion }
         'm_cvar_support',        { cvar variable directive }
         'm_nested_comment',      { nested comments }
         'm_tp_procvar',          { tp style procvars (no @ needed) }
         'm_mac_procvar',         { macpas style procvars }
         'm_repeat_forward',      { repeating forward declarations is needed }
         'm_pointer_2_procedure', { allows the assignement of pointers to
                                  procedure variables                     }
         'm_autoderef',           { does auto dereferencing of struct. vars }
         'm_initfinal',           { initialization/finalization for units }
         'm_default_ansistring',  { ansistring turned on by default }
         'm_out',                 { support the calling convention OUT }
         'm_default_para',        { support default parameters }
         'm_hintdirective',       { support hint directives }
         'm_duplicate_names',     { allow locals/paras to have duplicate names of globals }
         'm_property',            { allow properties }
         'm_default_inline',      { allow inline proc directive }
         'm_except',              { allow exception-related keywords }
         'm_objectivec1',         { support interfacing with Objective-C (1.0) }
         'm_objectivec2',         { support interfacing with Objective-C (2.0) }
         'm_nested_procvars',     { support nested procedural variables }
         'm_non_local_goto',      { support non local gotos (like iso pascal) }
         'm_advanced_records',    { advanced record syntax with visibility sections, methods and properties }
         'm_isolike_unary_minus', { unary minus like in iso pascal: same precedence level as binary minus/plus }
         'm_systemcodepage',      { use system codepage as compiler codepage by default, emit ansistrings with system codepage }
         'm_final_fields',        { allows declaring fields as "final", which means they must be initialised
                                  in the (class) constructor and are constant from then on (same as final
                                  fields in Java) }
         'm_default_unicodestring', { makes the default string type in $h+ mode unicodestring rather than
                                    ansistring; similarly, char becomes unicodechar rather than ansichar }
         'm_type_helpers',        { allows the declaration of "type helper" for all supported types
                                  (primitive types, records, classes, interfaces) }
         'm_blocks',              { support for http://en.wikipedia.org/wiki/Blocks_(C_language_extension) }
         'm_isolike_io',          { I/O as it required by an ISO compatible compiler }
         'm_isolike_program_para',{ program parameters as it required by an ISO compatible compiler }
         'm_isolike_mod',         { mod operation as it is required by an iso compatible compiler }
         'm_array_operators',     { use Delphi compatible array operators instead of custom ones ("+") }
         'm_multi_helpers',       { helpers can appear in multiple scopes simultaneously }
         'm_array2dynarray',      { regular arrays can be implicitly converted to dynamic arrays }
         'm_prefixed_attributes'  { enable attributes that are defined before the type they belong to }
       );
       { optimizer }
       optimizerswitchname : array[toptimizerswitch] of string[50] =
        ('cs_opt_none',
         'cs_opt_level1',
         'cs_opt_level2',
         'cs_opt_level3',
         'cs_opt_level4',
         'cs_opt_regvar',
         'cs_opt_uncertain',
         'cs_opt_size',
         'cs_opt_stackframe',
         'cs_opt_peephole',
         'cs_opt_loopunroll',
         'cs_opt_tailrecursion',
         'cs_opt_nodecse',
         'cs_opt_nodedfa',
         'cs_opt_loopstrength',
         'cs_opt_scheduler',
         'cs_opt_autoinline',
         'cs_useebp',
         'cs_userbp',
         'cs_opt_reorder_fields',
         'cs_opt_fastmath',
         { Allow removing expressions whose result is not used, even when this
           can change program behaviour (range check errors disappear',
           access violations due to invalid pointer derefences disappear, ...).
           Note: it does not (and must not) remove expressions that have
             explicit side-effects, only implicit side-effects (like the ones
             mentioned before) can disappear.
         }
         'cs_opt_dead_values',
         { compiler checks for empty procedures/methods and removes calls to them if possible }
         'cs_opt_remove_empty_proc',
         'cs_opt_constant_propagate',
         'cs_opt_dead_store_eliminate',
         'cs_opt_forcenostackframe',
         'cs_opt_use_load_modify_store',
         'cs_opt_unused_para'
       );
    var
         globalswitch  : tglobalswitch;
         targetswitch  : ttargetswitch;
         moduleswitch  : tmoduleswitch;
         localswitch   : tlocalswitch;
         modeswitch    : tmodeswitch;
         optimizerswitch : toptimizerswitch;
         globalswitches  : tglobalswitches;
         targetswitches  : ttargetswitches;
         moduleswitches  : tmoduleswitches;
         localswitches   : tlocalswitches;
         modeswitches    : tmodeswitches;
         optimizerswitches : toptimizerswitches;
    begin
       {alignment : talignmentinfo;}
       {talignmentinfo = packed record}
       writeln('Procedure alignment: '+tostr(new_settings.alignment.procalign));
       writeln('Loop alignment: '+tostr(new_settings.alignment.loopalign));
       { alignment for labels after unconditional jumps, this must be a power of two }
       writeln('Jump alignment: '+tostr(new_settings.alignment.jumpalign));
       { max. alignment for labels after unconditional jumps:
         the compiler tries to align jumpalign, however, to do so it inserts at maximum jumpalignskipmax bytes or uses
         the next smaller power of two of jumpalign }
       writeln('Jump skip max alignment: '+tostr(new_settings.alignment.jumpalignskipmax));
       { alignment for labels where two flows of the program flow coalesce, this must be a power of two }
       writeln('Coalescence alignment: '+tostr(new_settings.alignment.coalescealign));
       { max. alignment for labels where two flows of the program flow coalesce
         the compiler tries to align to coalescealign, however, to do so it inserts at maximum coalescealignskipmax bytes or uses
         the next smaller power of two of coalescealign }
       writeln('Coalescence skip max alignment: '+tostr(new_settings.alignment.coalescealignskipmax));
       writeln('Const min alignment: '+tostr(new_settings.alignment.constalignmin));
       writeln('Const max alignment: '+tostr(new_settings.alignment.constalignmax));
       writeln('Var min alignment: '+tostr(new_settings.alignment.varalignmin));
       writeln('Var max alignment: '+tostr(new_settings.alignment.varalignmax));
       writeln('Local min alignment: '+tostr(new_settings.alignment.localalignmin));
       writeln('Local max alignment: '+tostr(new_settings.alignment.localalignmax));
       writeln('Min record alignment: '+tostr(new_settings.alignment.recordalignmin));
       writeln('Max record alignment: '+tostr(new_settings.alignment.recordalignmax));
       writeln('Max C record alignment: '+tostr(new_settings.alignment.maxCrecordalign));
       globalswitches:=new_settings.globalswitches;
       for globalswitch:=low(tglobalswitch) to high(tglobalswitch) do
         if globalswitch in globalswitches then
           begin
             writeln('global switch: '+globalswitchname[globalswitch]);
             exclude(globalswitches,globalswitch);
           end;
       if (globalswitches <> []) then
         writeln('Unknown global switch');
       targetswitches:=new_settings.targetswitches;
       for targetswitch:=low(ttargetswitch) to high(ttargetswitch) do
         if targetswitch in targetswitches then
           begin
             writeln('target switch: '+targetswitchname[targetswitch]);
             exclude(targetswitches,targetswitch);
           end;
       if (targetswitches <> []) then
         writeln('Unknown target switch');
       moduleswitches:=new_settings.moduleswitches;
       for moduleswitch:=low(tmoduleswitch) to high(tmoduleswitch) do
         if moduleswitch in moduleswitches then
           begin
             writeln('module switch: '+moduleswitchname[moduleswitch]);
             exclude(moduleswitches,moduleswitch);
           end;
       if (moduleswitches <> []) then
         writeln('Unknown module switch');
       localswitches:=new_settings.localswitches;
       for localswitch:=low(tlocalswitch) to high(tlocalswitch) do
         if localswitch in localswitches then
           begin
             writeln('local switch: '+localswitchname[localswitch]);
             exclude(localswitches,localswitch);
           end;
       if (localswitches <> []) then
         writeln('Unknown local switch');
       modeswitches:=new_settings.modeswitches;
       for modeswitch:=low(tmodeswitch) to high(tmodeswitch) do
         if modeswitch in modeswitches then
           begin
             writeln(['mode switch: ',modeswitchname[modeswitch]]);
             exclude(modeswitches,modeswitch);
           end;
       if (modeswitches <> []) then
         writeln('Unknown mode switch');
       optimizerswitches:=new_settings.optimizerswitches;
       for optimizerswitch:=low(toptimizerswitch) to high(toptimizerswitch) do
         if optimizerswitch in optimizerswitches then
           begin
             writeln(['optimizer switch: ',optimizerswitchname[optimizerswitch]]);
             exclude(optimizerswitches,optimizerswitch);
           end;
       if (optimizerswitches <> []) then
         writeln('Unknown optimizer switch');
       writeln(['Set allocation size ',new_settings.setalloc]);
       writeln(['Pack enums ',new_settings.packenum]);
       writeln(['Pack records ',new_settings.packrecords]);
       writeln(['Max FPU registers ',new_settings.maxfpuregisters]);

       writeln(['CPU type ',new_settings.cputype]);
       writeln(['CPU optimize type ',new_settings.optimizecputype]);
       writeln(['FPU type ',new_settings.fputype]);
       writeln(['ASM mode ',new_settings.asmmode]);
    end;

var
  linestr,genstr : string;
  token : ttoken;
  copy_size, stbi, last_col, new_col : longint;
  last_line,new_line : dword;
  len : sizeint;
  wstring : widestring;
  astring : ansistring;
begin
  tbi:=0;
  last_line:=0;
  last_col:=0;
  linestr:='';
  genstr:='';
  fillchar(new_settings,sizeof(new_settings),#0);
  fillchar(prev_settings,sizeof(prev_settings),#0);
  write([space,' Tokens: ']);
  while tbi<tokenbufsize do
    begin
      token:=readtoken;
      if token<>_GENERICSPECIALTOKEN then
        begin
          if token <= high(ttoken) then
            begin
              write(arraytokeninfo[token].str);
              if not (token in [_CWCHAR, _CWSTRING, _CSTRING, _CCHAR,
                                _INTCONST,_REALNUMBER, _ID]) then
                StrAppend(linestr,lowercase(arraytokeninfo[token].str));
            end
          else
            begin
              HasMoreInfos;
              write('Error in Token List');
              break;
            end;
          {idtoken:=}readtoken;
        end;
      case token of
        _CWCHAR,
        _CWSTRING :
          begin
            len:=gettokenbufsizeint;
            setlength(wstring,len);
            move(tokenbuf[tbi],wstring[1],len*2);
            write([' ''',wstring,'''']);
            StrAppend(linestr,' ''');
            StrAppend(linestr,wstring);
            StrAppend(linestr,'''');
            inc(tbi,len*2);
          end;
        _CSTRING:
          begin
            len:=gettokenbufsizeint;
            setlength(astring,len);
            if len>0 then
              move(tokenbuf[tbi],astring[1],len);
            write([' ''',astring,'''']);
            StrAppend(linestr,' ''');
            StrAppend(linestr,astring);
            StrAppend(linestr,'''');
            inc(tbi,len);
          end;
        _CCHAR:
          begin
            write([' ''',unaligned(pshortstring(@tokenbuf[tbi])^),'''']);
            StrAppend(linestr,' ''');
            StrAppend(linestr,unaligned(pshortstring(@tokenbuf[tbi])^));
            StrAppend(linestr,'''');
            inc(tbi,tokenbuf[tbi]+1);
          end;
        _INTCONST,
        _REALNUMBER :
          begin
            write([' ',unaligned(pshortstring(@tokenbuf[tbi])^)]);
            StrAppend(linestr,unaligned(pshortstring(@tokenbuf[tbi])^));
            inc(tbi,tokenbuf[tbi]+1);
          end;
        _ID :
          begin
            write([' ',unaligned(pshortstring(@tokenbuf[tbi])^)]);
            StrAppend(linestr,unaligned(pshortstring(@tokenbuf[tbi])^));
            inc(tbi,tokenbuf[tbi]+1);
          end;
        _GENERICSPECIALTOKEN:
          begin
            { Short version of column change,
              byte or $80 used }
            if (tokenbuf[tbi] and $80)<>0 then
              begin
                new_col:=tokenbuf[tbi] and $7f;
                write(['Col: ',new_col]);
                if length(linestr)<new_col-1 then
                  StrAppend(linestr,StringOfChar(' ',new_col - 1 - length(linestr)));
                inc(tbi);
                last_col:=new_col;
              end
            else
              case tspecialgenerictoken(tokenbuf[tbi]) of
                ST_LOADSETTINGS:
                  begin
                    inc(tbi);
                    write([space,'Settings: ']);
                    fillchar(new_settings,sizeof(new_settings),#0);
                    { This does not load pmessage pointer }
                    new_settings.pmessage:=nil;
                    { TSettings size depends in target...
                      We first read the size of the copied part }
                    { Still not cross endian ready :( }
                    copy_size:=gettokenbufsizeint;
                    stbi:=tbi;
                    tokenreadsettings(new_settings, copy_size);
                    tbi:=stbi+copy_size;
                    if CompareByte(new_settings,prev_settings,sizeof(new_settings))<>0 then
                      begin
                        dump_new_settings;
                        writeln;
                      end
                    else
                      begin
                        writeln('Unchanged');
                      end;
                    prev_settings:=new_settings;
                  end;
                ST_LOADMESSAGES:
                  begin
                    inc(tbi);
                    mesgnb:=tokenbuf[tbi];
                    writeln([space,mesgnb,' messages: ']);
                    inc(tbi);
                    for nb:=1 to mesgnb do
                      begin
                        msgvalue:=gettokenbufsizeint;
                        //inc(tbi,sizeof(sizeint));
                        state:=tmsgstate(gettokenbufsizeint);
                        writeln(['#',msgvalue,' ',state]);
                      end;
                  end;
                ST_LINE:
                  begin
                    inc(tbi);
                    new_line:=gettokenbufdword;
                    if (new_line<>last_line) then
                      begin
                        StrAppend(genstr,linestr+LineEnding);
                        linestr:='';
                      end;
                    writeln(['Line: ',new_line]);
                    last_line:=new_line;
                  end;
                ST_COLUMN:
                  begin
                    inc(tbi);
                    new_col:=gettokenbufword;
                    write(['Col: ',new_col]);
                    if length(linestr)<new_col - 1 then
                      StrAppend(linestr,StringOfChar(' ',new_col - 1 - length(linestr)));
                    last_col:=new_col;
                  end;
                ST_FILEINDEX:
                  begin
                    inc(tbi);
                    StrAppend(genstr,linestr+LineEnding);
                    linestr:='';
                    write(['File: ',gettokenbufword]);
                  end;
                else
                  begin
                    HasMoreInfos;
                    write('Error in Token List');
                    break;
                  end;
              end;
          end;
        else ; { empty else to avoid warning }
      end;

      if tbi<tokenbufsize then
        write(',');
    end;
  writeln;
  StrAppend(genstr,linestr);
  writeln(['##',genstr,'##']);
end;

procedure readcommondef(const s:string; out defoptions: tdefoptions; Def: TPpuDef = nil);
type
  tdefopt=record
    mask : tdefoption;
    str  : string[30];
  end;
  tdefstateinfo=record
    mask : tdefstate;
    str  : string[30];
  end;
  tgenconstrflag=record
    mask : tgenericconstraintflag;
    str  : string[30];
  end;
const
  defopt : array[1..ord(high(tdefoption))] of tdefopt=(
     (mask:df_unique;         str:'Unique Type'),
     (mask:df_generic;        str:'Generic'),
     (mask:df_specialization; str:'Specialization'),
     (mask:df_copied_def;     str:'Copied Typedef'),
     (mask:df_genconstraint;  str:'Generic Constraint'),
     { this should never happen for defs stored to a ppu file }
     (mask:df_not_registered_no_free;  str:'Unregistered/No free (invalid)'),
     (mask:df_llvm_no_struct_packing;  str:'LLVM unpacked struct'),
     (mask:df_internal;       str:'Internal'),
     (mask:df_has_global_ref; str:'Has Global Ref'),
     (mask:df_has_generic_fields; str:'Has generic fields')
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
  genconstrflag : array[1..ord(high(tgenericconstraintflag))] of tgenconstrflag=(
     (mask:gcf_constructor; str:'Constructor'),
     (mask:gcf_class;       str:'Class'),
     (mask:gcf_record;      str:'Record')
  );

var
  defstates  : tdefstates;
  i, nb, len : longint;
  first  : boolean;
  min_size, tokenbufsize : longint;
  tokenbuf : pbyte;
  genconstr : tgenericconstraintflags;
begin
  i:=ppufile.getlongint;
  if Def <> nil then
    Def.Id:=i;
  writeln([space,'** Definition Id ',i,' **']);
  writeln([space,s]);
  write  ([space,'      Type symbol : ']);
  if Def <> nil then
    readderef('', Def.Ref)
  else
    readderef('');
  write  ([space,'       DefOptions : ']);
  ppufile.getset(tppuset2(defoptions));
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

  write  ([space,'        DefStates : ']);
  ppufile.getset(tppuset1(defstates));
  if defstates<>[] then
    begin
      first:=true;
      for i:=1 to high(defstate) do
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

  if df_genconstraint in defoptions then
    begin
      ppufile.getset(tppuset1(genconstr));
      write  ([space,'   GenConstraints : ']);
      if genconstr<>[] then
        begin
          first:=true;
          for i:=1 to high(genconstrflag) do
           if (genconstrflag[i].mask in genconstr) then
            begin
              if first then
                first:=false
              else
                write(', ');
              write(genconstrflag[i].str);
            end;
        end;
      writeln;

      len:=ppufile.getlongint;
      if len>0 then
        begin
          space:='    '+space;
          writeln([space,'------ constraint defs begin ------']);
          for i:=0 to len-1 do
            begin
              writeln([space,'------ constraint def ',i,' ------']);
              readderef(space);
            end;
          writeln([space,'------ constraint defs end ------']);
          delete(space,1,4);
        end;
    end;

  if [df_generic,df_specialization]*defoptions<>[] then
    begin
      nb:=ppufile.getlongint;
      writeln([space,'has ',nb,' parameters']);
      if nb>0 then
        begin
          for i:=0 to nb-1 do
            begin
              writeln([space,'parameter ',i,': ',ppufile.getstring]);
              readderef(space);
            end;
        end;
    end;
  if df_generic in defoptions then
    begin
      tokenbufsize:=ppufile.getlongint;
      writeln([space,' Tokenbuffer size : ',tokenbufsize]);
      tokenbuf:=allocmem(tokenbufsize);
      ppufile.getdata(tokenbuf^,tokenbufsize);
      displaytokenbuffer(tokenbuf,tokenbufsize);
      freemem(tokenbuf);
    end;
  if df_specialization in defoptions then
    begin
      write  ([space,' Orig. GenericDef : ']);
      readderef('');
    end;
  space:=space+'    ';
  readattrs(def);
  delete(space,1,4);
  current_defoptions:=defoptions;
end;


{ Read abstract procdef and return if inline procdef }
{ type tproccalloption is in globtype unit }
{ type tproctypeoption is in globtype unit }
{ type tprocoption is in globtype unit }

procedure read_abstract_proc_def(var proccalloption:tproccalloption;var procoptions:tprocoptions; ProcDef: TPpuProcDef);
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
    str  : string[34];
  end;
const
  {proccalloptionStr  is also in globtype unit }
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
     (mask:potype_class_destructor;  str:'Class Destructor'),
     { Dispinterface property accessors }
     (mask:potype_propgetter;        str:'Property Getter'),
     (mask:potype_propsetter;        str:'Property Setter'),
     (mask:potype_exceptfilter;      str:'SEH filter'),
     (mask:potype_mainstub;          str:'main stub'),
     (mask:potype_pkgstub;           str:'package stub'),
     (mask:potype_libmainstub;       str:'library main stub')
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
     (mask:po_syscall;         str:'Syscall'),
     (mask:po_syscall_legacy;  str:'SyscallLegacy'),
     (mask:po_syscall_basenone;str:'SyscallBaseNone'),
     (mask:po_syscall_basefirst;str:'SyscallBaseFirst'),
     (mask:po_syscall_baselast;str:'SyscallBaseLast'),
     (mask:po_syscall_basereg; str:'SyscallBaseReg'),
     (mask:po_syscall_has_libsym; str:'Has LibSym'),
     (mask:po_syscall_has_importnr; str:'Uses ImportNr'),
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
     (mask:po_delphi_nested_cc;str: 'Delphi-style nested frameptr'),
     (mask:po_java_nonvirtual; str: 'Java non-virtual method'),
     (mask:po_ignore_for_overload_resolution;str: 'Ignored for overload resolution'),
     (mask:po_rtlproc;         str: 'RTL procedure'),
     (mask:po_auto_raised_visibility; str: 'Visibility raised by compiler'),
     (mask:po_far;             str: 'Far'),
     (mask:po_hasnearfarcallmodel; str: 'Near/Far explicit'),
     (mask:po_noreturn;        str: 'No return'),
     (mask:po_is_function_ref; str: 'Function reference'),
     (mask:po_is_block;        str: 'C "Block"'),
     (mask:po_is_auto_getter;  str: 'Automatically generated getter'),
     (mask:po_is_auto_setter;  str: 'Automatically generated setter'),
     (mask:po_noinline;        str: 'Never inline'),
     (mask:po_variadic;        str: 'C VarArgs with array-of-const para'),
     (mask:po_objc_related_result_type; str: 'Objective-C related result type')
  );
var
  proctypeoption  : tproctypeoption;
  i     : longint;
  first : boolean;
begin
  write([space,'      Return type : ']);
  readderef('', ProcDef.ReturnType);
  proctypeoption:=tproctypeoption(ppufile.getbyte);
  case proctypeoption of
    potype_function: Include(ProcDef.Options, poFunction);
    potype_procedure: Include(ProcDef.Options, poProcedure);
    potype_constructor: Include(ProcDef.Options, poConstructor);
    potype_destructor: Include(ProcDef.Options, poDestructor);
    potype_operator: Include(ProcDef.Options, poOperator);
  end;
  write([space,'       TypeOption : ']);
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
  writeln([space,'       CallOption : ',proccalloptionStr[proccalloption]]);
  ppufile.getset(tppuset8(procoptions));
  if procoptions<>[] then
   begin
     if po_classmethod in procoptions then Include(ProcDef.Options, poClassMethod);
     if po_virtualmethod in procoptions then Include(ProcDef.Options, poVirtual);
     if po_abstractmethod in procoptions then Include(ProcDef.Options, poAbstract);
     if po_overridingmethod in procoptions then Include(ProcDef.Options, poOverriding);
     if po_overload in procoptions then Include(ProcDef.Options, poOverload);
     if po_inline in procoptions then Include(ProcDef.Options, poInline);
     if (po_methodpointer in procoptions) and (ProcDef.DefType = dtProcType) then
       TPpuProcTypeDef(ProcDef).MethodPtr:=True;

     write([space,'          Options : ']);
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
      readcgpara(space);
    end;
end;


{ type tvaroption is in unit symconst }
  { register variable }
{ type tvarregable is in unit symconst }
procedure readabstractvarsym(const s:string;var varoptions:tvaroptions; VarDef: TPpuVarDef = nil);
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
     (mask:vo_is_first_field;str:'IsFirstField'),
     (mask:vo_volatile;        str:'Volatile'),
     (mask:vo_has_section;     str:'HasSection'),
     (mask:vo_force_finalize;  str:'ForceFinalize'),
     (mask:vo_is_default_var;  str:'DefaultIntrinsicVar'),
     (mask:vo_is_far;          str:'IsFar'),
     (mask:vo_has_global_ref;  str:'HasGlobalRef')
  );
type
  tvaraccessdesc=record
    mask: tvarsymaccessflag;
    str: string[30];
  end;
const
  varaccessstr : array[ord(low(tvarsymaccessflag))..ord(high(tvarsymaccessflag))] of tvaraccessdesc=(
    (mask: vsa_addr_taken;         str:'Address taken'),
    (mask: vsa_different_scope;    str:'Accessed from different scope')
  );
var
  i : longint;
  accessflag: tvarsymaccessflag;
  varsymaccessflags: tvarsymaccessflags;
  first : boolean;
begin
  readcommonsym(s, VarDef);
  i:=ppufile.getbyte;
  if (VarDef <> nil) and (VarDef.DefType = dtParam) then
    with TPpuParamDef(VarDef) do
      case tvarspez(i) of
        vs_value: Spez:=psValue;
        vs_var: Spez:=psVar;
        vs_out: Spez:=psOut;
        vs_const: Spez:=psConst;
        vs_constref: Spez:=psConstRef;
      end;
  writeln([space,'         Spez : ',Varspez2Str(i)]);
  writeln([space,'      Regable : ',Varregable2Str(ppufile.getbyte)]);
  ppufile.getset(tppuset1(varsymaccessflags));
  write([space, ' Access Flags : ']);
  first:=true;
  for i:=low(varaccessstr) to high(varaccessstr) do
    begin
      if varaccessstr[i].mask in varsymaccessflags then
        begin
          if first then
            first:=false
          else
            write([', ']);
          write([varaccessstr[i].str]);
        end
    end;
  writeln;
  write  ([space,'     Var Type : ']);
  if VarDef <> nil then
    readderef('',VarDef.VarType)
  else
    readderef('');
  ppufile.getset(tppuset4(varoptions));
  if varoptions<>[] then
   begin
     if (VarDef <> nil) and (VarDef.DefType = dtParam) and (vo_is_hidden_para in varoptions) then
       TPpuParamDef(VarDef).Spez:=psHidden;
     write([space,'      Options : ']);
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


procedure readobjectdefoptions(ObjDef: TPpuObjectDef = nil);
type
  tsymopt=record
    mask : tobjectoption;
    str  : string[30];
  end;
const
  symopt : array[1..ord(high(tobjectoption))] of tsymopt=(
     (mask:oo_is_forward;         str:'IsForward'),
     (mask:oo_is_abstract;        str:'IsAbstract'),
     (mask:oo_is_sealed;          str:'IsSealed'),
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
     (mask:oo_is_formal;          str:'Formal'),
     (mask:oo_is_classhelper;     str:'Class Helper/Category'),
     (mask:oo_has_class_constructor; str:'HasClassConstructor'),
     (mask:oo_has_class_destructor; str:'HasClassDestructor'),
     (mask:oo_is_enum_class;      str:'JvmEnumClass'),
     (mask:oo_has_new_destructor; str:'HasNewDestructor')
  );
var
  i      : longint;
  first  : boolean;
begin
  ppufile.getset(tppuset4(current_objectoptions));
  if current_objectoptions<>[] then
   begin
     if ObjDef <> nil then
      begin
        if oo_is_abstract in current_objectoptions then
          Include(ObjDef.Options, ooIsAbstract);
      end;
     first:=true;
     for i:=1 to high(symopt) do
      if (symopt[i].mask in current_objectoptions) then
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


procedure readprocimploptions(const space: string; out implprocoptions: timplprocoptions);
type
  tpiopt=record
    mask : timplprocoption;
    str  : string[30];
  end;
const
  piopt : array[low(timplprocoption)..high(timplprocoption)] of tpiopt=(
    (mask:pio_empty; str:'IsEmpty'),
    (mask:pio_has_inlininginfo; str:'HasInliningInfo'),
    (mask:pio_inline_not_possible; str:'InlineNotPossible'),
    (mask:pio_nested_access; str:'NestedAccess'),
    (mask:pio_thunk; str:'Thunk'),
    (mask:pio_fastmath; str:'FastMath'),
    (mask:pio_inline_forbidden; str:'InlineForbidden')
  );
var
  i: timplprocoption;
  first: boolean;
begin
  ppufile.getset(tppuset1(implprocoptions));
  if implprocoptions<>[] then
    begin
      first:=true;
      write([space,'          Options : ']);
      for i:=low(piopt) to high(piopt) do
        begin
          if i in implprocoptions then
            begin
              if first then
                first:=false
              else
                write(', ');
              write(piopt[i].str);
            end;
        end;
      writeln;
    end;
end;

procedure readarraydefoptions(ArrayDef: TPpuArrayDef);
{ type tarraydefoption is in unit symconst }
const
  symopt : array[tarraydefoption] of string = (
   { ado_IsConvertedPointer } 'ConvertedPointer',
   { ado_IsDynamicArray     } 'IsDynamicArray',
   { ado_IsVariant          } 'IsVariant',
   { ado_IsConstructor      } 'IsConstructor',
   { ado_IsArrayOfConst     } 'ArrayOfConst',
   { ado_IsConstString      } 'ConstString',
   { ado_IsBitPacked        } 'BitPacked',
   { ado_IsVector           } 'Vector',
   { ado_IsGeneric          } 'Generic'
  );
var
  symoptions: tarraydefoptions;
  i: tarraydefoption;
  first: boolean;
begin
  ppufile.getset(tppuset2(symoptions));
  if symoptions<>[] then
   begin
     if ado_IsDynamicArray in symoptions then Include(ArrayDef.Options, aoDynamic);
     first:=true;
     for i:=Low(symopt) to high(symopt) do
      if (i in symoptions) then
       begin
         if first then
           first:=false
         else
           write(', ');
         write(symopt[i]);
       end;
   end;
  writeln;
end;

(* options for properties
  tpropertyoption=(ppo_none,
    ppo_indexed,
    ppo_defaultproperty,
    ppo_stored,
    ppo_hasparameters,
    ppo_implements,
    ppo_enumerator_current,
    ppo_overrides,
    ppo_dispid_write              { no longer used }
  );
  tpropertyoptions=set of tpropertyoption;
*)
function readpropertyoptions:tpropertyoptions;
{ type tarraydefoption is in unit symconst }
type
  tpropopt=record
    mask : tpropertyoption;
    str  : string[30];
  end;
const
  symopt : array[1..ord(high(tpropertyoption))] of tpropopt=(
    (mask:ppo_indexed;str:'indexed'),
    (mask:ppo_defaultproperty;str:'default'),
    (mask:ppo_stored;str:'stored'),
    (mask:ppo_hasparameters;str:'has parameters'),
    (mask:ppo_implements;str:'implements'),
    (mask:ppo_enumerator_current;str:'enumerator current'),
    (mask:ppo_overrides;str:'overrides'),
    (mask:ppo_dispid_write;str:'dispid write')  { no longer used }
  );
var
  i      : longint;
  first  : boolean;
begin
  ppufile.getset(tppuset2(result));
  if result<>[] then
   begin
     first:=true;
     for i:=1 to high(symopt) do
      if (symopt[i].mask in result) then
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



function readmanagementoperatoroptions(const space : string;const name : string):tmanagementoperators;
{ type is in unit symconst }
{ Management operator options
  tmanagementoperator=(
    mop_none,
    mop_initialize,
    mop_finalize,
    mop_addref,
    mop_copy);
}
type
  tmopopt=record
    mask : tmanagementoperator;
    str  : string[10];
  end;
const
  managementoperatoropt : array[1..ord(high(tmanagementoperator))] of tmopopt=(
    (mask:mop_initialize;str:'initialize'),
    (mask:mop_finalize;str:'finalize'),
    (mask:mop_addref;str:'addref'),
    (mask:mop_copy;str:'copy')
  );
var
  i      : longint;
  first  : boolean;
begin
  ppufile.getset(tppuset1(result));
  if result<>[] then
   begin
     first:=true;
     for i:=1 to high(managementoperatoropt) do
      if (managementoperatoropt[i].mask in result) then
       begin
         if first then
           begin
             write(space);
             write(name);
             write(': ');
             first:=false;
           end
         else
           write(', ');
         write(managementoperatoropt[i].str);
       end;
     if not first then
       writeln;
   end;
end;


procedure readnodetree;
var
  l : longint;
  p : pointer;
begin
  with ppufile do
   begin
     if space<>'' then
      Writeln([space,'------ nodetree ------']);
     if readentry=ibnodetree then
      begin
        l:=entrysize;
        Writeln([space,'Tree size : ',l]);
        { Read data to prevent error that entry is not completly read }
        getmem(p,l);
        getdata(p^,l);
        freemem(p);
      end
     else
      begin
        WriteError('!! ibnodetree not found');
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
      WriteError('!! ibcreatedobjtypes entry not found');
      ppufile.skipdata(ppufile.entrysize);
      exit
    end;
  writeln;
  writeln([space,'WPO info']);
  writeln([space,'--------']);

  len:=ppufile.getlongint;
  writeln([space,'** Instantiated Object/Class types: ',len,' **']);
  space:=space+'  ';
  for i:=0 to len-1 do
    readderef(space);
  setlength(space,length(space)-2);

  len:=ppufile.getlongint;
  writeln([space,'** Instantiated ClassRef types: ',len,' **']);
  space:=space+'  ';
  for i:=0 to len-1 do
    readderef(space);
  setlength(space,length(space)-2);

  len:=ppufile.getlongint;
  writeln([space,'** Possibly instantiated ClassRef types : ',len,' **']);
  space:=space+'  ';
  for i:=0 to len-1 do
    readderef(space);
  setlength(space,length(space)-2);

  len:=ppufile.getlongint;
  writeln([space,'** Class types with called virtual methods info : ',len,' **']);
  space:=space+'  ';
  for i:=0 to len-1 do
    begin
      write([space,'Class def : ']);
      readderef('');
      write([space+'  ','Called vmtentries : ']);
      bssize:=ppufile.getlongint;
      getmem(bs,bssize);
      ppufile.readdata(bs^,bssize);
      for j:=0 to bssize*8-1 do
        if (((bs+j shr 3)^ shr (j and 7)) and 1) <> 0 then
          write([j,', ']);
      writeln;
      freemem(bs);
    end;
  setlength(space,length(space)-2);
end;

{****************************************************************************
                             Read Symbols Part
****************************************************************************}

procedure readsymbols(const s:string; ParentDef: TPpuContainerDef = nil);

  function _finddef(symdef: TPpuDef): TPpuDef;
  begin
    Result:=nil;
    if symdef.Ref.IsCurUnit then
     begin;
       Result:=CurUnit.FindById(symdef.Ref.Id);
       if (Result <> nil) and (Result.Ref.Id = symdef.Id) then
        begin
          Result.Name:=symdef.Name;
          Result.FilePos:=symdef.FilePos;
        end
       else
         Result:=nil;
     end;
  end;

type
  pguid = ^tguid;
  tguid = packed record
    D1: LongWord;
    D2: Word;
    D3: Word;
    D4: array[0..7] of Byte;
  end;

var
  b      : byte;
  pc     : pchar;
  ch : dword;
  startnewline : boolean;
  i,j,len : longint;
  prettyname, ss : ansistring;
  ws: widestring;
  guid : tguid;
  realvalue : ppureal;
  doublevalue : double;
  singlevalue : single;
  realstr : shortstring;
  extended : TSplit80bitReal;
  pw : pcompilerwidestring;
  varoptions : tvaroptions;
  propoptions : tpropertyoptions;
  iexp: Tconstexprint;
  def: TPpuDef;
  constdef: TPpuConstDef absolute def;
begin
  with ppufile do
   begin
     if space<>'' then
      Writeln([space,'------ ',s,' ------']);
     if readentry=ibstartsyms then
      begin
        Writeln([space,'Symtable count: ',getlongint]);
      end
     else
      Writeln('!! ibstartsym not found');
     repeat
       def:=nil;
       b:=readentry;
       case b of

         ibunitsym :
           readcommonsym('Unit symbol ');

         ibnamespacesym :
           begin
             readcommonsym('NameSpace symbol ');
             write([space,'  Hidden Unit : ']);
             readderef('');
           end;

         iblabelsym :
           readcommonsym('Label symbol ');

         ibtypesym :
           begin
             def:=TPpuTypeRef.Create(nil);
             readcommonsym('Type symbol ',def);
             write([space,'  Result Type : ']);
             readderef('', def.Ref);
             if _finddef(def) = nil then
               def.Parent:=ParentDef;
             prettyname:=getansistring;
             if prettyname<>'' then
               begin
                 write([space,' Pretty Name : ']);
                 Writeln(prettyname);
               end;
           end;

         ibprocsym :
           begin
             def:=TPpuDef.Create(nil);
             readcommonsym('Procedure symbol ', def);
             len:=ppufile.getword;
             for i:=1 to len do
              begin
                write([space,'   Definition : ']);
                readderef('', def.Ref);
                _finddef(def);
              end;
           end;

         ibconstsym :
           begin
             constdef:=TPpuConstDef.Create(ParentDef);
             readcommonsym('Constant symbol ',constdef);
             b:=getbyte;
             case tconsttyp(b) of
               constord :
                 begin
                   write  ([space,'  OrdinalType : ']);
                   readderef('',constdef.TypeRef);
                   iexp:=getexprint;
                   constdef.ConstType:=ctInt;
                   constdef.VInt:=iexp.svalue;
                   writeln([space,'        Value : ',constexp.tostr(iexp)]);
                 end;
               constpointer :
                 begin
                   write  ([space,'  PointerType : ']);
                   readderef('',constdef.TypeRef);
                   constdef.ConstType:=ctInt;
                   constdef.VInt:=int64(getptruint);
                   writeln([space,'        Value : ',constdef.VInt])
                 end;
               conststring,
               constresourcestring :
               begin
                   write ([space,'   StringType : ']);
                   readderef('',constdef.TypeRef);
                   len:=getlongint;
                   getmem(pc,len+1);
                   getdata(pc^,len);
                   (pc+len)^:= #0;
                   writeln([space,'       Length : ',len]);
                   writeln([space,'        Value : "',pc,'"']);
                   constdef.ConstType:=ctStr;
                   SetString(constdef.VStr, pc, len);
                   constdef.VStr:=UTF8Encode(constdef.VStr);
                   freemem(pc,len+1);
                 end;
               constreal :
                 begin
                   constdef.ConstType:=ctFloat;
                   write  ([space,'     RealType : ']);
                   readderef('',constdef.TypeRef);
                   write([space,'        Value : ']);
                   if entryleft=sizeof(ppureal) then
                     begin
                       realvalue:=getrealsize(sizeof(ppureal));
                       constdef.VFloat:=realvalue;
                       system.str(realvalue,realstr);
                       writeln([realstr]);
                     end
                   else if entryleft=sizeof(double) then
                     begin
                       doublevalue:=getrealsize(sizeof(double));
                       constdef.VFloat:=doublevalue;
                       system.str(doublevalue,realstr);
                       writeln([realstr]);
                     end
                   else if entryleft=sizeof(single) then
                     begin
                       singlevalue:=getrealsize(sizeof(single));
                       constdef.VFloat:=singlevalue;
                       system.str(singlevalue,realstr);
                       writeln([realstr]);
                     end
                   else if entryleft=10 then
                     begin
                       getdata(extended,entryleft);
                       ss:=Real80bitToStr(extended,constdef.VFloat);
                       writeln(ss);
                     end
                   else
                     begin
                       realvalue:=0.0;
                       WriteError('Error reading real value');
                     end;
                 end;
               constset :
                 begin
                   constdef.ConstType:=ctSet;
                   write ([space,'      Set Type : ']);
                   readderef('',constdef.TypeRef);
                   for i:=1to 4 do
                    begin
                      write ([space,'        Value : ']);
                      for j:=1to 8 do
                       begin
                         if j>1 then
                          write(',');
                         b:=getbyte;
                         write(hexstr(b,2));
                         constdef.VSet[i*j-1]:=b;
                       end;
                      writeln;
                    end;
                 end;
               constnil:
                 begin
                   write([space,'   NIL pointer :']);
                   readderef('',constdef.TypeRef);
                   constdef.ConstType:=ctPtr;
                   constdef.VInt:=0;
                 end;
               constwstring :
                 begin
                   initwidestring(pw);
                   setlengthwidestring(pw,getlongint);
                   if widecharsize=2 then
                   { don't use getdata, because the compilerwidechars may have to
                     be byteswapped
                   }
                     begin
                       for i:=0 to pw^.len-1 do
                         pw^.data[i]:=ppufile.getword;
                       SetString(ws, PWideChar(pw^.data), pw^.len);
                       constdef.VStr:=UTF8Encode(ws);
                       constdef.ConstType:=ctStr;
                     end
                   else if widecharsize=4 then
                     begin
                       for i:=0 to pw^.len-1 do
                         pw^.data[i]:=cardinal(ppufile.getlongint);
                     end
                   else
                     begin
                       WriteError('Unsupported tcompilerwidechar size');
                     end;
                   Write([space,'Wide string type']);
                   startnewline:=true;
                   for i:=0 to pw^.len-1 do
                     begin
                       if startnewline then
                         begin
                           writeln;
                           write(space);
                           startnewline:=false;
                         end;
                       ch:=pw^.data[i];
                       if widecharsize=2 then
                         write(hexstr(ch,4))
                       else
                         write(hexstr(ch,8));
                       if ((i + 1) mod 8)= 0 then
                         startnewline:=true
                       else
                         if i <> pw^.len-1 then
                           write(', ');
                     end;
                   donewidestring(pw);
                   Writeln;
                 end;
               constguid:
                 begin
                    write ([space,'     IntfType : ']);
                    readderef('',constdef.TypeRef);
                    getdata(guid,sizeof(guid));
                    write ([space,'    IID String: {',hexstr(guid.d1,8),'-',hexstr(guid.d2,4),'-',hexstr(guid.d3,4),'-']);
                    for i:=0 to 7 do
                      begin
                         write(hexstr(guid.d4[i],2));
                         if i=1 then write('-');
                      end;
                    writeln('}');
                 end
               else
                 Writeln (['!! Invalid unit format : Invalid const type encountered: ',b]);
             end;
           end;

         ibabsolutevarsym :
           begin
             def:=TPpuVarDef.Create(ParentDef);
             readabstractvarsym('Absolute variable symbol ',varoptions,TPpuVarDef(def));
             Write ([space,' Relocated to ']);
             b:=getbyte;
             case absolutetyp(b) of
               tovar :
                 readpropaccesslist(space+'          Sym : ');
               toasm :
                 Writeln(['Assembler name : ',getstring]);
               toaddr :
                 begin
                   Write(['Address : ',getpuint]);
                   if tsystemcpu(ppufile.header.common.cpu)=cpu_i386 then
                     Write([' (Far: ',getbyte<>0,')']);
                   if tsystemcpu(ppufile.header.common.cpu)=cpu_i8086 then
                     if getbyte<>0 then
                       Write([' (Far: TRUE, Segment=',getaword,')'])
                     else
                       Write([' (Far: FALSE)']);
                   Writeln;
                 end;
               else
                 Writeln (['!! Invalid unit format : Invalid absolute type encountered: ',b]);
             end;
           end;

         ibfieldvarsym :
           begin
             def:=TPpuFieldDef.Create(ParentDef);
             readabstractvarsym('Field Variable symbol ',varoptions,TPpuVarDef(def));
             writeln([space,'      Address : ',getasizeint]);
             if vo_has_mangledname in varoptions then
               writeln([space,' Mangled name : ',getstring]);
           end;

         ibstaticvarsym :
           begin
             def:=TPpuVarDef.Create(ParentDef);
             readabstractvarsym('Global Variable symbol ',varoptions,TPpuVarDef(def));
             write  ([space,' DefaultConst : ']);
             readderef('');
             if (vo_has_mangledname in varoptions) then
               writeln([space,'Mangledname : ',readsymstr(ppufile)]);
             if vo_has_section in varoptions then
               writeln(['Section name:',ppufile.getansistring]);
             write  ([space,' FieldVarSymDeref: ']);
             readderef('');
           end;

         iblocalvarsym :
           begin
             readabstractvarsym('Local Variable symbol ',varoptions);
             write  ([space,' DefaultConst : ']);
             readderef('');
           end;

         ibparavarsym :
           begin
             def:=TPpuParamDef.Create(ParentDef);
             readabstractvarsym('Parameter Variable symbol ',varoptions,TPpuVarDef(def));
             write  ([space,' DefaultConst : ']);
             readderef('',TPpuParamDef(def).DefaultValue);
             writeln([space,'       ParaNr : ',getword]);
             writeln([space,'        Univ  : ',getboolean]);
             writeln([space,'     VarState : ',getbyte]);
             writeln([space,'         Refs : ',getbyte]);
             if (vo_has_explicit_paraloc in varoptions) then
               begin
                 readcgpara(space+'   ');
               end;
           end;

         ibenumsym :
           begin
             def:=TPpuConstDef.Create(nil);
             readcommonsym('Enumeration symbol ',def);
             write  ([space,'   Definition : ']);
             readderef('');
             TPpuConstDef(def).ConstType:=ctInt;
             TPpuConstDef(def).VInt:=getlongint;
             writeln([space,'        Value : ',TPpuConstDef(def).VInt]);
             if (ParentDef <> nil) and (ParentDef.DefType = dtEnum) then
               def.Parent:=ParentDef;
           end;

         ibsyssym :
           begin
             readcommonsym('Internal system symbol ');
             writeln([space,'  Internal Nr : ',getlongint]);
           end;

         ibmacrosym :
           begin
             readcommonsym('Macro symbol ');
             writeln([space,'       Defined: ',getboolean]);
             writeln([space,'  Compiler var: ',getboolean]);
             len:=getlongint;
             writeln([space,'  Value length: ',len]);
             if len > 0 then
               begin
                 getmem(pc,len+1);
                 getdata(pc^,len);
                 (pc+len)^:= #0;
                 writeln([space,'         Value: "',pc,'"']);
                 freemem(pc,len+1);
               end;
           end;

         ibpropertysym :
           begin
             def:=TPpuPropDef.Create(ParentDef);
             readcommonsym('Property ',def);
             write  ([space,' Prop Options : ']);
             propoptions:=readpropertyoptions;
             if ppo_overrides in propoptions then
               begin
                 write  ([space,' OverrideProp : ']);
                 readderef('');
               end;
             if ppo_defaultproperty in propoptions then
               Include(TPpuPropDef(def).Options, poDefault);
             write  ([space,'    Prop Type : ']);
             readderef('',TPpuPropDef(def).PropType);
             writeln([space,'        Index : ',getlongint]);
             writeln([space,'      Default : ',getlongint]);
             write  ([space,'   Index Type : ']);
             readderef('');
             { palt_none }
             write  ([space,'   Noneaccess : ']);
             readpropaccesslist('');
             write  ([space,'   Readaccess : ']);
             readpropaccesslist(space+'         Sym: ',TPpuPropDef(def).Getter);
             write  ([space,'  Writeaccess : ']);
             readpropaccesslist(space+'         Sym: ',TPpuPropDef(def).Setter);
             write  ([space,' Storedaccess : ']);
             readpropaccesslist(space+'         Sym: ');
             if [ppo_hasparameters,ppo_overrides]*propoptions=[ppo_hasparameters] then
               begin
                 space:='    '+space;
                 readsymtable('parast',TPpuPropDef(def));
                 delete(space,1,4);
               end;
           end;

         iberror :
           begin
             WriteError('!! Error in PPU');
             exit;
           end;

         ibendsyms :
           break;

         else
           begin
             WriteError('!! Skipping unsupported PPU Entry in Symbols: '+IntToStr(b));
           end;
       end;
       if assigned(def) then
         readsymsubentries(def);
       if (def <> nil) and (def.Parent = nil) then
         def.Free;
       if not EndOfEntry then
         HasMoreInfos;
     until false;
   end;
end;


{****************************************************************************
                         Read defintions Part
****************************************************************************}

procedure readdefinitions(const s:string; ParentDef: TPpuContainerDef);
{ type tordtype is in symconst unit }
{
    uvoid,
    u8bit,u16bit,u32bit,u64bit,u128bit,
    s8bit,s16bit,s32bit,s64bit,s128bit,
    bool8bit,bool16bit,bool32bit,bool64bit,
    uchar,uwidechar,scurrency,customint
  ); }

{ type tobjecttyp is in symconst unit }
{ type tvarianttype is in symconst unit }
{ type thelpertype is in symconst unit }
var
  b : byte;
  otb : byte; { Object Type byte, needed later again }
  l,j,tokenbufsize : longint;
  tokenbuf : pbyte;
  calloption : tproccalloption;
  procoptions : tprocoptions;
  implprocoptions: timplprocoptions;
  defoptions: tdefoptions;
  iexpr: Tconstexprint;
  def: TPpuDef;
  objdef: TPpuObjectDef absolute def;
  arrdef: TPpuArrayDef absolute def;
  enumdef: TPpuEnumDef absolute def;
  setdef: TPpuSetDef absolute def;
  orddef: TPpuOrdDef absolute def;
  floatdef: TPpuFloatDef absolute def;
  strdef: TPpuStringDef absolute def;
  filedef: TPpuFileDef absolute def;
begin
  with ppufile do
   begin
     if space<>'' then
      Writeln([space,'------ ',s,' ------']);
     if readentry<>ibstartdefs then
      Writeln('!! ibstartdefs not found');
     repeat
       def:=nil;
       b:=readentry;
       case b of

         ibpointerdef :
           begin
             def:=TPpuPointerDef.Create(ParentDef);
             readcommondef('Pointer definition',defoptions,def);
             write  ([space,'     Pointed Type : ']);
             readderef('',TPpuPointerDef(def).Ptr);
             writeln([space,' Has Pointer Math : ',(getbyte<>0)]);
             if tsystemcpu(ppufile.header.common.cpu) in [cpu_i8086,cpu_i386,cpu_x86_64] then
               begin
                 write([space,' X86 Pointer Type : ']);
                 b:=getbyte;
                 case tx86pointertyp(b) of
                   x86pt_near: writeln('Near');
                   x86pt_near_cs: writeln('Near ''CS''');
                   x86pt_near_ds: writeln('Near ''DS''');
                   x86pt_near_ss: writeln('Near ''SS''');
                   x86pt_near_es: writeln('Near ''ES''');
                   x86pt_near_fs: writeln('Near ''FS''');
                   x86pt_near_gs: writeln('Near ''GS''');
                   x86pt_far: writeln('Far');
                   x86pt_huge: writeln('Huge');
                   else
                     WriteWarning('Invalid x86 pointer type: ' + IntToStr(b));
                 end;
               end;
           end;

         iborddef :
           begin
             orddef:=TPpuOrdDef.Create(ParentDef);
             readcommondef('Ordinal definition',defoptions,orddef);
             write  ([space,'        Base type : ']);
             b:=getbyte;
             case tordtype(b) of
               uvoid:
                 begin
                   writeln('uvoid');
                   orddef.OrdType:=otVoid;
                 end;
               u8bit:
                 begin
                   writeln('u8bit');
                   orddef.OrdType:=otUInt;
                   orddef.Size:=1;
                 end;
               u16bit:
                 begin
                   writeln('u16bit');
                   orddef.OrdType:=otUInt;
                   orddef.Size:=2;
                 end;
               u32bit:
                 begin
                   writeln('u32bit');
                   orddef.OrdType:=otUInt;
                   orddef.Size:=4;
                 end;
               u64bit:
                 begin
                   writeln('u64bit');
                   orddef.OrdType:=otUInt;
                   orddef.Size:=8;
                 end;
               u128bit:
                 begin
                   writeln('u128bit');
                   orddef.OrdType:=otUInt;
                   orddef.Size:=16;
                 end;
               s8bit:
                 begin
                   writeln('s8bit');
                   orddef.OrdType:=otSInt;
                   orddef.Size:=1;
                 end;
               s16bit:
                 begin
                   writeln('s16bit');
                   orddef.OrdType:=otSInt;
                   orddef.Size:=2;
                 end;
               s32bit:
                 begin
                   writeln('s32bit');
                   orddef.OrdType:=otSInt;
                   orddef.Size:=4;
                 end;
               s64bit:
                 begin
                   writeln('s64bit');
                   orddef.OrdType:=otSInt;
                   orddef.Size:=8;
                 end;
               s128bit:
                 begin
                   writeln('s128bit');
                   orddef.OrdType:=otSInt;
                   orddef.Size:=16;
                 end;
               pasbool1:
                 begin
                   writeln('pasbool1');
                   orddef.OrdType:=otPasBool;
                   orddef.Size:=1;
                 end;
               pasbool8:
                 begin
                   writeln('pasbool8');
                   orddef.OrdType:=otPasBool;
                   orddef.Size:=1;
                 end;
               pasbool16:
                 begin
                   writeln('pasbool16');
                   orddef.OrdType:=otPasBool;
                   orddef.Size:=2;
                 end;
               pasbool32:
                 begin
                   writeln('pasbool32');
                   orddef.OrdType:=otPasBool;
                   orddef.Size:=4;
                 end;
               pasbool64:
                 begin
                   writeln('pasbool64');
                   orddef.OrdType:=otPasBool;
                   orddef.Size:=8;
                 end;
               bool8bit:
                 begin
                   writeln('bool8bit');
                   orddef.OrdType:=otBool;
                   orddef.Size:=1;
                 end;
               bool16bit:
                 begin
                   writeln('bool16bit');
                   orddef.OrdType:=otBool;
                   orddef.Size:=2;
                 end;
               bool32bit:
                 begin
                   writeln('bool32bit');
                   orddef.OrdType:=otBool;
                   orddef.Size:=4;
                 end;
               bool64bit:
                 begin
                   writeln('bool64bit');
                   orddef.OrdType:=otBool;
                   orddef.Size:=8;
                 end;
               uchar:
                 begin
                   writeln('uchar');
                   orddef.OrdType:=otChar;
                   orddef.Size:=1;
                 end;
               uwidechar:
                 begin
                   writeln('uwidechar');
                   orddef.OrdType:=otChar;
                   orddef.Size:=2;
                 end;
               scurrency:
                 begin
                   writeln('scurrency');
                   orddef.OrdType:=otCurrency;
                   orddef.Size:=8;
                 end;
               customint:
                 begin
                   writeln('customint');
                   orddef.OrdType:=otSint;
                   orddef.Size:=sizeof(ASizeInt);
                 end
               else
                 WriteWarning('Invalid base type: ' + IntToStr(b));
             end;
             iexpr:=getexprint;
             orddef.RangeLow:=iexpr.svalue;
             write([space,'            Range : ',constexp.tostr(iexpr)]);
             iexpr:=getexprint;
             orddef.RangeHigh:=iexpr.svalue;
             writeln([' to ',constexp.tostr(iexpr)]);
           end;

         ibfloatdef :
           begin
             floatdef:=TPpuFloatDef.Create(ParentDef);
             readcommondef('Float definition',defoptions,floatdef);
             write  ([space,'       Float type : ']);
             b:=getbyte;
             case b of
               ftSingle:
                 begin
                   writeln('Single');
                   floatdef.FloatType:=pftSingle;
                 end;
               ftDouble:
                 begin
                   writeln('Double');
                   floatdef.FloatType:=pftDouble;
                 end;
               ftExtended:
                 begin
                   writeln('Extended');
                   floatdef.FloatType:=pftExtended;
                 end;
               ftComp:
                 begin
                   writeln('Comp');
                   floatdef.FloatType:=pftComp;
                 end;
               ftCurr:
                 begin
                   writeln('Currency');
                   floatdef.FloatType:=pftCurrency;
                 end;
               ftFloat128:
                 begin
                   writeln('Float128');
                   floatdef.FloatType:=pftFloat128;
                 end;
               else
                 WriteWarning('Invalid float type: ' + IntToStr(b));
             end;
           end;

         ibarraydef :
           begin
             arrdef:=TPpuArrayDef.Create(ParentDef);
             readcommondef('Array definition',defoptions,arrdef);
             write  ([space,'     Element type : ']);
             readderef('',arrdef.ElType);
             write  ([space,'       Range Type : ']);
             readderef('',arrdef.RangeType);
             arrdef.RangeLow:=getasizeint;
             arrdef.RangeHigh:=getasizeint;
             writeln([space,'            Range : ',arrdef.RangeLow,' to ',arrdef.RangeHigh]);
             write  ([space,'          Options : ']);
             readarraydefoptions(arrdef);
             if tsystemcpu(ppufile.header.common.cpu)=cpu_i8086 then
               writeln([space,'             Huge : ',(getbyte<>0)]);
             readsymtable('symbols', arrdef);
           end;

         ibprocdef :
           begin
             def:=TPpuProcDef.Create(ParentDef);
             readcommondef('Procedure definition',defoptions,def);
             read_abstract_proc_def(calloption,procoptions,TPpuProcDef(def));
             if (po_has_mangledname in procoptions) then
               writeln([space,'     Mangled name : ',readsymstr(ppufile)]);
             writeln([space,'           Number : ',getword]);
             writeln([space,'            Level : ',getbyte]);
             write  ([space,'            Class : ']);
             readderef('');
             write  ([space,'          Procsym : ']);
             readderef('', def.Ref);
             write  ([space,'         File Pos : ']);
             readposinfo(def);
             write  ([space,'       Visibility : ']);
             readvisibility(def);
             write  ([space,'       SymOptions : ']);
             readsymoptions(space+'       ');
             if (po_has_importdll in procoptions) then
               writeln([space,'      Import DLL : ',getstring]);
             if (po_has_importname in procoptions) then
               writeln([space,'      Import Name : ',getstring]);
             writeln([space,'        Import Nr : ',getword]);
             if (po_msgint in procoptions) then
               writeln([space,'           MsgInt : ',getlongint]);
             if (po_msgstr in procoptions) then
               writeln([space,'           MsgStr : ',getstring]);
             if (po_dispid in procoptions) then
               writeln([space,'      DispID: ',ppufile.getlongint]);
             readprocimploptions(space,implprocoptions);
             if (pio_has_inlininginfo in implprocoptions) then
              begin
                write  ([space,'       FuncretSym : ']);
                readderef('');
                readprocinfooptions(space);
              end;
             b:=ppufile.getbyte;
             if b<>0 then
               begin
                 write  ([space,'      Alias names : ']);
                 for j:=1 to b do
                   begin
                     write(ppufile.getstring);
                     if j<b then
                       write(', ');
                   end;
                 writeln;
               end;
             tokenbufsize:=ppufile.getlongint;
             if tokenbufsize<>0 then
               begin
                 space:=space + '    ';
                 write  ([space,'Declaration token buffer : size = ',tokenbufsize]);
                 tokenbuf:=allocmem(tokenbufsize);
                 ppufile.getdata(tokenbuf^,tokenbufsize);
                 displaytokenbuffer(tokenbuf,tokenbufsize);
                 freemem(tokenbuf);
                 delete(space,1,4);
               end;
             if po_syscall_has_libsym in procoptions then
               begin
                 { library symbol for AmigaOS/MorphOS/AROS }
                 write  ([space,'   Library symbol : ']);
                 readderef('');
               end;
             if not EndOfEntry then
               HasMoreInfos;
             space:='    '+space;
             { parast }
             readsymtable('parast', TPpuProcDef(def));
             { localst }
             if (pio_has_inlininginfo in implprocoptions) then
                readsymtable('inline localst')
             else if (df_generic in defoptions) then
                readsymtable('generic localst');
             if (pio_has_inlininginfo in implprocoptions) then
               readnodetree;
             delete(space,1,4);
           end;

         ibprocvardef :
           begin
             def:=TPpuProcTypeDef.Create(ParentDef);
             readcommondef('Procedural type (ProcVar) definition',defoptions,def);
             read_abstract_proc_def(calloption,procoptions, TPpuProcDef(def));
             writeln([space,'   Symtable level :',ppufile.getbyte]);
             if tsystemcpu(ppufile.header.common.cpu)=cpu_jvm then
               readderef('');
             if not EndOfEntry then
               HasMoreInfos;
             space:='    '+space;
             { parast }
             readsymtable('parast',TPpuProcDef(def));
             delete(space,1,4);
           end;

         ibshortstringdef :
           begin
             strdef:=TPpuStringDef.Create(ParentDef);
             strdef.StrType:=stShort;
             readcommondef('ShortString definition',defoptions,strdef);
             strdef.Len:=getbyte;
             writeln([space,'           Length : ',strdef.Len]);
           end;

         ibwidestringdef :
           begin
             strdef:=TPpuStringDef.Create(ParentDef);
             strdef.StrType:=stWide;
             readcommondef('WideString definition',defoptions,strdef);
             strdef.Len:=getasizeint;
             writeln([space,'           Length : ',strdef.Len]);
           end;

         ibunicodestringdef :
           begin
             strdef:=TPpuStringDef.Create(ParentDef);
             strdef.StrType:=stUnicode;
             readcommondef('UnicodeString definition',defoptions,strdef);
             strdef.Len:=getasizeint;
             writeln([space,'           Length : ',strdef.Len]);
             writeln([space,'         Encoding : ',getword]);
           end;

         ibansistringdef :
           begin
             strdef:=TPpuStringDef.Create(ParentDef);
             strdef.StrType:=stAnsi;
             readcommondef('AnsiString definition',defoptions,strdef);
             strdef.Len:=getasizeint;
             writeln([space,'           Length : ',strdef.Len]);
             writeln([space,'         Encoding : ',getword]);
           end;

         iblongstringdef :
           begin
             strdef:=TPpuStringDef.Create(ParentDef);
             strdef.StrType:=stLong;
             readcommondef('Longstring definition',defoptions,strdef);
             strdef.Len:=getasizeint;
             writeln([space,'           Length : ',strdef.Len]);
           end;

         ibrecorddef :
           begin
             objdef:=TPpuRecordDef.Create(ParentDef);
             readcommondef('Record definition',defoptions, objdef);
             def.Name:=getstring;
             writeln([space,'   Name of Record : ',objdef.Name]);
             writeln([space,'   Import lib/pkg : ',getstring]);
             write  ([space,'          Options : ']);
             readobjectdefoptions(objdef);
             if (df_copied_def in defoptions) then
               begin
                 Include(TPpuRecordDef(def).Options, ooCopied);
                 write([space,'      Copied from : ']);
                 readderef('',objdef.Ancestor);
               end
             else
               begin
                 writeln([space,'       FieldAlign : ',shortint(getbyte)]);
                 writeln([space,'      RecordAlign : ',shortint(getbyte)]);
                 writeln([space,'         PadAlign : ',shortint(getbyte)]);
                 writeln([space,'UseFieldAlignment : ',shortint(getbyte)]);
                 writeln([space,'   RecordAlignMin : ',shortint(getbyte)]);
                 objdef.Size:=getasizeint;
                 writeln([space,'         DataSize : ',objdef.Size]);
                 writeln([space,'      PaddingSize : ',getword]);
                 readmanagementoperatoroptions(space,'Management operators');
               end;
             {read the record definitions and symbols}
             if not(df_copied_def in current_defoptions) then
               begin
                 space:='    '+space;
                 readrecordsymtable('fields',TPpuRecordDef(def));
                 Delete(space,1,4);
               end;
             if not EndOfEntry then
               HasMoreInfos;
           end;

         ibobjectdef :
           begin
             objdef:=TPpuObjectDef.Create(ParentDef);
             readcommondef('Object/Class definition',defoptions,objdef);
             objdef.Name:=getstring;
             writeln([space,'    Name of Class : ',objdef.Name]);
             writeln([space,'   Import lib/pkg : ',getstring]);
             write  ([space,'          Options : ']);
             readobjectdefoptions(objdef);
             otb:=getbyte;
             write  ([space,'             Type : ']);
             case tobjecttyp(otb) of
               odt_class          : writeln('class');
               odt_object         : writeln('object');
               odt_interfacecom   : writeln('interfacecom');
               odt_interfacecorba : writeln('interfacecorba');
               odt_cppclass       : writeln('cppclass');
               odt_dispinterface  : writeln('dispinterface');
               odt_objcclass      : writeln('objcclass');
               odt_objcprotocol   : writeln('objcprotocol');
               odt_helper         : writeln('helper');
               odt_objccategory   : writeln('objccategory');
               odt_javaclass      : writeln('Java class');
               odt_interfacejava  : writeln('Java interface');
               else                 WriteWarning('Invalid object type: ' + IntToStr(b));
             end;
             case tobjecttyp(otb) of
               odt_class, odt_cppclass, odt_objcclass, odt_javaclass:
                 objdef.ObjType:=otClass;
               odt_object:
                 objdef.ObjType:=otObject;
               odt_interfacecom, odt_interfacecorba, odt_interfacejava, odt_dispinterface:
                 objdef.ObjType:=otInterface;
               odt_helper:
                 objdef.ObjType:=otHelper;
             end;
             b:=getbyte;
             write  ([space,'      Helper Type : ']);
             case thelpertype(b) of
               ht_none   : writeln('none');
               ht_class  : writeln('class helper');
               ht_record : writeln('record helper');
               ht_type   : writeln('type helper');
               else        WriteWarning('Invalid helper type: ' + IntToStr(b));
             end;
             writeln([space,'    External name : ',getstring]);
             objdef.Size:=getasizeint;
             writeln([space,'         DataSize : ',objdef.Size]);
             writeln([space,'      PaddingSize : ',getword]);
             writeln([space,'       FieldAlign : ',shortint(getbyte)]);
             writeln([space,'      RecordAlign : ',shortint(getbyte)]);
             writeln([space,'   RecordAlignMin : ',shortint(getbyte)]);
             write  ([space,  '       VmtField : ']);
             readderef('',nil);
             write  ([space,  '   Ancestor Class : ']);
             readderef('',objdef.Ancestor);

             if tobjecttyp(otb) in [odt_interfacecom,odt_interfacecorba,odt_dispinterface] then
               begin
                  { IIDGUID }
                  for j:=1to 16 do
                   getbyte;
                  objdef.IID:=getstring;
                  writeln([space,'       IID String : ',objdef.IID]);
               end;

             l:=getlongint;
             if l > 0 then
               objdef.Options:=objdef.Options + [ooAbstractMethods];
             writeln([space,' Abstract methods : ',l]);

             if tobjecttyp(otb)=odt_helper then
               begin
                 write([space,'    Helper parent : ']);
                 readderef('',objdef.HelperParent);
               end;

             l:=getlongint;
             writeln([space,'  VMT entries: ',l]);
             for j:=1 to l do
               begin
                 write([space,'    ']);
                 readderef('');
                 write([space,'      Visibility: ']);
                 readvisibility;
               end;

             if tobjecttyp(otb) in [odt_class,odt_objcclass,odt_objcprotocol,odt_javaclass,odt_interfacejava] then
              begin
                l:=getlongint;
                writeln([space,'  Impl Intf Count : ',l]);
                for j:=1 to l do
                 begin
                   write  ([space,'  - Definition : ']);
                   readderef('');
                   write  ([space,'  - Getter Def : ']);
                   readderef('');
                   writeln([space,'       IOffset : ',getlongint]);
                   writeln([space,'    Entry type : ',IntfEntryType2Str(getbyte)]);
                 end;
              end;

             if df_copied_def in current_defoptions then
               begin
                 Include(objdef.Options, ooCopied);
                 writeln('  Copy of def: ');
                 readderef('',objdef.Ancestor);
               end
             else
               begin
                 {read the record definitions and symbols}
                 space:='    '+space;
                 readrecordsymtable('fields',objdef);
                 Delete(space,1,4);
              end;
             if not EndOfEntry then
               HasMoreInfos;
           end;

         ibfiledef :
           begin
             filedef:=TPpuFileDef.Create(ParentDef);
             ReadCommonDef('File definition',defoptions,filedef);
             write  ([space,'             Type : ']);
             case getbyte of
              0 : begin
                    writeln('Text');
                    filedef.FileType:=ftText;
                  end;
              1 : begin
                    writeln('Typed');
                    filedef.FileType:=ftTyped;
                    write  ([space,'      File of Type : ']);
                    readderef('',filedef.TypeRef);
                  end;
              2 : begin
                    writeln('Untyped');
                    filedef.FileType:=ftUntyped;
                  end;
             end;
           end;

         ibformaldef :
           begin
             def:=TPpuFormalDef.Create(ParentDef);
             readcommondef('Generic definition (void-typ)',defoptions,def);
             TPpuFormalDef(def).IsTyped:=(getbyte<>0);
             writeln([space,'         Is Typed : ',TPpuFormalDef(def).IsTyped]);
           end;

         ibundefineddef :
           begin
             def:=TPpuUndefinedDef.Create(ParentDef);
             readcommondef('Undefined definition (generic parameter)',defoptions,def);
           end;

         ibenumdef :
           begin
             enumdef:=TPpuEnumDef.Create(ParentDef);
             readcommondef('Enumeration type definition',defoptions,enumdef);
             enumdef.ElLow:=getaint;
             writeln([space,' Smallest element : ',enumdef.ElLow]);
             enumdef.ElHigh:=getaint;
             writeln([space,'  Largest element : ',enumdef.ElHigh]);
             enumdef.Size:=byte(getaint);
             writeln([space,'             Size : ',enumdef.Size]);
             if df_copied_def in defoptions then
               begin
                 write([space,'Base enumeration type : ']);
                 readderef('',enumdef.CopyFrom);
               end
             else
               begin
                 space:='    '+space;
                 readsymtable('elements',enumdef);
                 delete(space,1,4);
               end;
             if tsystemcpu(ppufile.header.common.cpu)=cpu_jvm then
               begin
                 write([space,'        Class def : ']);
                 readderef('');
               end;
           end;

         ibclassrefdef :
           begin
             def:=TPpuClassRefDef.Create(ParentDef);
             readcommondef('Class reference definition',defoptions,def);
             write  ([space,'    Pointed Type : ']);
             readderef('',TPpuClassRefDef(def).ClassRef);
           end;

         ibsetdef :
           begin
             setdef:=TPpuSetDef.Create(ParentDef);
             readcommondef('Set definition',defoptions,setdef);
             write  ([space,'     Element type : ']);
             readderef('',setdef.ElType);
             setdef.Size:=getasizeint;
             writeln([space,'             Size : ',setdef.Size]);
             setdef.SetBase:=getasizeint;
             writeln([space,'         Set Base : ',setdef.SetBase]);
             setdef.SetMax:=getasizeint;
             writeln([space,'          Set Max : ',setdef.SetMax]);
           end;

         ibvariantdef :
           begin
             def:=TPpuVariantDef.Create(ParentDef);
             readcommondef('Variant definition',defoptions,def);
             write  ([space,'      Varianttype : ']);
             b:=getbyte;
             case tvarianttype(b) of
               vt_normalvariant :
                 writeln('Normal');
               vt_olevariant :
                 begin
                   TPpuVariantDef(def).IsOLE:=True;
                   writeln('OLE');
                 end
               else
                 WriteWarning('Invalid varianttype: ' + IntToStr(b));
             end;
           end;

         iberror :
           begin
             WriteError('!! Error in PPU');
             exit;
           end;

         ibenddefs :
           break;

         else
           begin
             WriteError('!! Skipping unsupported PPU Entry in definitions: '+IntToStr(b));
           end;
       end;
       if assigned(def) then
         readdefsubentries(def);
       if (def <> nil) and (def.Parent = nil) then
         def.Free;
       if not EndOfEntry then
         HasMoreInfos;
     until false;
   end;
end;

procedure readmoduleoptions(space : string);
type
{ tmoduleoption type is in unit fmodule }
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
  moduleopts=ord(high(tmoduleoption));
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
  ppufile.getset(tppuset1(moduleoptions));
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
    writeln([space,'Deprecated : ', ppufile.getstring]);
end;

{****************************************************************************
                           Read General Part
****************************************************************************}

procedure readinterface(silent : boolean);
var
  b : byte;
  sourcenumber, i : longint;
  feature : tfeature;
  features : tfeatures;
  s : string;
begin
  with ppufile do
   begin
     repeat
       b:=readentry;
       case b of

         ibmodulename :
           begin
             CurUnit.Name:=getstring;
             if not silent then
               Writeln(['Module Name: ',CurUnit.Name]);
           end;

         ibfeatures :
           begin
             getset(tppuset4(features));
             Writeln('Features: ');
             for feature:=low(tfeatures) to high(tfeature) do
               if feature in features then
                 begin
                   str(feature,s);
                   s:=copy(s,3,255);
                   writeln([s]);
                 end;

           end;

         ibmoduleoptions:
           if not silent then
             readmoduleoptions('  ');

         ibsourcefiles :
           begin
             sourcenumber:=1;
             if not silent then
               while not EndOfEntry do
                 begin
                   with TPpuSrcFile.Create(CurUnit.SourceFiles) do begin
                     Name:=getstring;
                     i:=getlongint;
                     try
                       if i >= 0 then
                         FileTime:=FileDateToDateTime(i);
                       Writeln(['Source file ',sourcenumber,' : ',Name,' ',filetimestring(i)]);
                     except
                     end;
                   end;

                   inc(sourcenumber);
                 end;
           end;
{$IFDEF MACRO_DIFF_HINT}
         ibusedmacros :
           begin
             if not silent then
               while not EndOfEntry do
                 begin
                    Write('Conditional ',getstring);
                    if getboolean then
                      write(' defined at startup')
                    else
                      write(' not defined at startup');
                    if getboolean then
                      writeln(' was used')
                    else
                      writeln;
                  end;
                end;
{$ENDIF}
         ibloadunit :
           if not silent then
             ReadLoadUnit;

         iblinkunitofiles :
           if not silent then
             ReadLinkContainer('Link unit object file: ');

         iblinkunitstaticlibs :
           if not silent then
             ReadLinkContainer('Link unit static lib: ');

         iblinkunitsharedlibs :
           if not silent then
             ReadLinkContainer('Link unit shared lib: ');

         iblinkotherofiles :
           if not silent then
             ReadLinkContainer('Link other object file: ');

         iblinkotherstaticlibs :
           if not silent then
             ReadLinkContainer('Link other static lib: ');

         iblinkothersharedlibs :
           if not silent then
             ReadLinkContainer('Link other shared lib: ');

         iblinkotherframeworks:
           if not silent then
             ReadLinkContainer('Link framework: ');

         ibjvmnamespace:
            Writeln('JVM name space: '+getString);
         ibmainname:
           if not silent then
             Writeln(['Specified main program symbol name: ',getstring]);

         ibImportSymbols :
           if not silent then
             ReadImportSymbols;

         ibderefdata :
           ReadDerefData;

         ibderefmap :
           ReadDerefMap;

         ibwpofile :
           if not silent then
             ReadWpoFileInfo;

         ibresources :
           if not silent then
             ReadContainer('Resource file: ');

         iborderedsymbols:
           if not silent then
             ReadContainer('Ordered symbol: ');

         iberror :
           begin
             WriteError('Error in PPU');
             exit;
           end;

         ibendinterface :
           break;

         else
           begin
             WriteError('!! Skipping unsupported PPU Entry in General Part: '+IntToStr(b));
           end;
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

         ibunitimportsyms :
           ReadUnitImportSyms;

         iberror :
           begin
             WriteError('Error in PPU');
             exit;
           end;
         ibendimplementation :
           break;
         else
           begin
             WriteError('!! Skipping unsupported PPU Entry in Implementation: '+IntToStr(b));
           end;
       end;
     until false;
   end;
end;


function parseextraheader(module: TPpuModuleDef; ppufile: tppufile): boolean;
var
  b: byte;
begin
  result:=false;
  b:=ppufile.readentry;
  if b<>ibextraheader then
    exit;
  CurUnit.LongVersion:=cardinal(ppufile.getlongint);
  Writeln(['LongVersion: ',CurUnit.LongVersion]);
  ppufile.getset(tppuset4(CurUnit.ModuleFlags));
  result:=ppufile.EndOfEntry and (CurUnit.LongVersion=CurrentPPULongVersion);
  if mf_symansistr in CurUnit.ModuleFlags then
    SymAnsiStr:=true;
end;

procedure dofile (filename : string);
begin
{ reset }
  space:='';
{ fix filename }
  if pos('.',filename)=0 then
   filename:=filename+'.ppu';
  ppufile:=tppudumpfile.create(filename);
  if not ppufile.openfile then
   begin
     WriteError('IO-Error when opening : '+filename+', Skipping');
     exit;
   end;
{ PPU File is open, check for PPU Id }
  if not ppufile.CheckPPUID then
   begin
     WriteError(Filename+' : Not a valid PPU file, Skipping');
     exit;
   end;
{ Check PPU Version }
  ppuversion:=ppufile.getversion;

  Writeln(['Analyzing ',filename,' (v',PPUVersion,')']);

  if not SkipVersionCheck and (PPUVersion <> CurrentPPUVersion) then
   begin
     WriteError(Format('Unsupported PPU version %d. Expecting PPU version %d.', [PPUVersion, CurrentPPUVersion]));
     exit;
   end;

  CurUnit:=TPpuModuleDef.Create(UnitList);
  CurUnit.Version:=ppuversion;

  if not parseextraheader(CurUnit, ppufile) then
    begin
      WriteError(Format('Unsupported PPU sub-version %d. Expecting PPU sub-version %d.', [CurUnit.LongVersion, CurrentPPULongVersion]));
    end;

{ Write PPU Header Information }
  if (verbose and v_header)<>0 then
   begin
     Writeln;
     Writeln('Header');
     Writeln('-------');
     with ppufile.header do
      begin
        Writeln(['Compiler version        : ',ppufile.header.common.compiler shr 14,'.',
                                             (ppufile.header.common.compiler shr 7) and $7f,'.',
                                             ppufile.header.common.compiler and $7f]);
        WriteLn(['Target processor        : ',Cpu2Str(common.cpu)]);
        WriteLn(['Target operating system : ',Target2Str(common.target)]);
        Writeln(['Unit flags              : ',PPUFlags2Str(common.flags)]);
        Writeln(['FileSize (w/o header)   : ',common.size]);
        Writeln(['Checksum                : ',hexstr(checksum,8)]);
        Writeln(['Interface Checksum      : ',hexstr(interface_checksum,8)]);
        Writeln(['Indirect Checksum       : ',hexstr(indirect_checksum,8)]);
        Writeln(['Definitions stored      : ',tostr(deflistsize)]);
        Writeln(['Symbols stored          : ',tostr(symlistsize)]);
      end;
   end;

  with ppufile.header do
    begin
      CurUnit.Crc:=checksum;
      CurUnit.IntfCrc:=interface_checksum;
      CurUnit.TargetCPU:=Cpu2Str(common.cpu);
      CurUnit.TargetOS:=Target2Str(common.target);
    end;

{read the general stuff}
  if (verbose and v_interface)<>0 then
   begin
     Writeln;
     Writeln('Interface section');
     Writeln('------------------');
     readinterface(false);
   end
  { We need derefdata from Interface }
  else if verbose and (v_defs or v_syms or v_implementation)<>0 then
     readinterface(true)
  else
   ppufile.skipuntilentry(ibendinterface);
  Writeln;
  Writeln('Interface symtable');
  Writeln('----------------------');
  readsymtableoptions('interface');
{read the definitions}
  if (verbose and v_defs)<>0 then
   begin
     Writeln;
     Writeln('Interface definitions');
     Writeln('----------------------');
     readdefinitions('interface', CurUnit);
   end
  else
   ppufile.skipuntilentry(ibenddefs);
{read the symbols}
  if (verbose and v_syms)<>0 then
   begin
     Writeln;
     Writeln('Interface Symbols');
     Writeln('------------------');
     readsymbols('interface',CurUnit);
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
      WriteError('!! Error in PPU');
      exit;
    end;
  if ppufile.getboolean then
    begin
      readsymtableoptions('interface macro');
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
  Writeln;
  Writeln('Implementation symtable');
  Writeln('----------------------');
  readsymtableoptions('implementation');
  if (mf_local_symtable in CurUnit.ModuleFlags) then
   begin
     if (verbose and v_defs)<>0 then
      begin
        Writeln;
        Writeln('Static definitions');
        Writeln('----------------------');
        readdefinitions('implementation', nil);
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
  FreeDerefdata;
{shutdown ppufile}
  ppufile.closefile;
  ppufile.free;
  Writeln;
end;

procedure WriteLogo;
begin
  writeln(Title+' Version '+version_string);
  writeln(Copyright);
  writeln;
end;

procedure help;
begin
  WriteLogo;
  writeln('usage: ppudump [options] <filename1> <filename2>...');
  writeln;
  writeln('[options] can be:');
  writeln('    -F<format>  Set output format to <format>');
  writeln('                  t - text format (default)');
  writeln('                  j - JSON format');
  writeln('                  x - XML format');
  writeln('    -M Exit with ExitCode=2 if more information is available');
  writeln('    -S Skip PPU version check. May lead to reading errors');
  writeln('    -V<verbose>  Set verbosity to <verbose>');
  writeln('                   H - Show header info');
  writeln('                   I - Show interface');
  writeln('                   M - Show implementation');
  writeln('                   S - Show interface symbols');
  writeln('                   D - Show interface definitions');
  writeln('                   A - Show all');
  writeln('    -h, -?       This helpscreen');
  halt;
end;

var
  startpara,
  nrfile,i  : longint;
  para      : string;
const
  error_on_more : boolean = false;
begin
  if paramcount<1 then
    help;
{ turn verbose on by default }
  verbose:=v_all;
{ read options }
  startpara:=1;
  while copy(paramstr(startpara),1,1)='-' do
   begin
     para:=paramstr(startpara);
     case upcase(para[2]) of
      'F' : begin
              FreeAndNil(pout);
              if Length(para) > 2 then
                case upcase(para[3]) of
                  'T':
                    nostdout:=False;
                  'J':
                    begin
                      nostdout:=True;
                      pout:=TPpuJsonOutput.Create(StdOutputHandle);
                    end;
                  'X':
                    begin
                      nostdout:=True;
                      pout:=TPpuXmlOutput.Create(StdOutputHandle);
                    end;
                  else
                    begin
                      WriteError('Invalid output format: ' + para[3]);
                      Halt(1);
                    end;
                end;
            end;
      'M' : error_on_more:=true;
      'S' : SkipVersionCheck:=True;
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
      else
        begin
          WriteError('Invalid option: ' + para);
          Halt(1);
        end;
     end;
     inc(startpara);
   end;

  if not nostdout then
    WriteLogo;

  UnitList:=TPpuContainerDef.Create(nil);
  try
    UnitList.ItemsName:='';
    { process files }
    for nrfile:=startpara to paramcount do
      dofile (paramstr(nrfile));
    if not has_errors and (pout <> nil) then
     begin
       pout.Init;
       UnitList.Write(pout);
       pout.Done;
     end;
  finally
    UnitList.Free;
    pout.Free;
  end;
  if has_errors then
    Halt(1);
  if error_on_more and
    (has_more_infos or has_warnings) then
    Halt(2);
end.

