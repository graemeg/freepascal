{
    Copyright (c) 1998-2008 by Florian Klaempfl and Peter Vreman

    Reads command line options and config files

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

 ****************************************************************************
}
unit options;

{$i fpcdefs.inc}

interface

uses
  cfileutl,cclasses,
  globtype,globals,verbose,systems,cpuinfo,comprsrc;

Type
  TOption=class
    FirstPass,
    ParaLogo,
    NoPressEnter,
    FPCHelpLines,
    LogoWritten,
    ABISetExplicitly,
    FPUSetExplicitly,
    CPUSetExplicitly,
    OptCPUSetExplicitly: boolean;
    FileLevel : longint;
    QuickInfo : string;
    FPCBinaryPath: string;
    ParaIncludeCfgPath,
    ParaIncludePath,
    ParaUnitPath,
    ParaObjectPath,
    ParaLibraryPath,
    ParaFrameworkPath,
    parapackagepath : TSearchPathList;
    paranamespaces : TCmdStrList;
    ParaAlignment   : TAlignmentInfo;
    parapackages : tfphashobjectlist;
    paratarget        : tsystem;
    paratargetasm     : tasm;
    paratargetdbg     : tdbg;
    LinkTypeSetExplicitly : boolean;
    LinkerSetExplicitly : boolean;
    Constructor Create;
    Destructor Destroy;override;
    procedure WriteLogo;
    procedure WriteInfo (More: string);
    procedure WriteHelpPages;
    procedure WriteQuickInfo;
    procedure IllegalPara(const opt:TCmdStr);
    procedure UnsupportedPara(const opt:TCmdStr);
    procedure IgnoredPara(const opt:TCmdStr);
    function  Unsetbool(var Opts:TCmdStr; Pos: Longint; const FullPara: TCmdStr; RequireBoolPara: Boolean):boolean;
    procedure interpret_option(const opt :TCmdStr;ispara:boolean);
    procedure Interpret_envvar(const envname : TCmdStr);
    procedure Interpret_file(const filename : TPathStr);
    procedure Read_Parameters;
    procedure parsecmd(cmd:TCmdStr);
    procedure TargetOptions(def:boolean);
    procedure CheckOptionsCompatibility;
    procedure ForceStaticLinking;
   protected
    MacVersionSet: boolean;
    processorstr: TCmdStr;
    function ParseMacVersionMin(out minstr, emptystr: string; const compvarname, value: string; ios: boolean): boolean;
    procedure MaybeSetDefaultMacVersionMacro;
    procedure VerifyTargetProcessor;
  end;

  TOptionClass=class of toption;

var
  coption : TOptionClass;

procedure read_arguments(cmd:TCmdStr);

implementation

uses
  widestr,
  {$if FPC_FULLVERSION<20700}ccharset{$else}charset{$endif},
  SysUtils,
  version,
  cutils,cmsgs,
  comphook,
  symtable,scanner,rabase,
  symconst,
{$ifdef llvm}
  { override supported optimizer transformations at the compiler level }
  llvminfo,
{$endif llvm}
  dirparse,
  pkgutil,
  i_bsd;

const
  page_size = 24;
  page_width = 80;

var
  option     : toption;
  read_configfile,        { read config file, set when a cfgfile is found }
  disable_configfile : boolean;
  fpcdir,
  ppccfg,
  param_file    : string;   { file to compile specified on the commandline }


{****************************************************************************
                     Options not supported on all platforms
****************************************************************************}

const
  { gprof (requires implementation of g_profilecode in the code generator) }
  supported_targets_pg = [system_i386_linux,system_x86_64_linux,system_mipseb_linux,system_mipsel_linux,system_arm_linux]
                        + [system_i386_win32]
                        + [system_powerpc_darwin,system_x86_64_darwin]
                        + [system_i386_GO32V2]
                        + [system_i386_freebsd]
                        + [system_i386_netbsd]
                        + [system_i386_wdosx]
                        + [system_riscv32_linux,system_riscv64_linux];

  suppported_targets_x_smallr = systems_linux + systems_solaris + systems_android
                             + [system_i386_haiku,system_x86_64_haiku]
                             + [system_i386_beos]
                             + [system_m68k_amiga];

{****************************************************************************
                                 Defines
****************************************************************************}

procedure set_default_link_type;
begin
  undef_system_macro('FPC_LINK_SMART');
  def_system_macro('FPC_LINK_STATIC');
  undef_system_macro('FPC_LINK_DYNAMIC');
  init_settings.globalswitches:=init_settings.globalswitches+[cs_link_static];
  init_settings.globalswitches:=init_settings.globalswitches-[cs_link_shared,cs_link_smart];
{$ifdef AIX}
  init_settings.globalswitches:=init_settings.globalswitches+[cs_link_native];
{$endif}
end;


{****************************************************************************
                                 Toption
****************************************************************************}

procedure StopOptions(err:longint);
begin
  if assigned(Option) then
   begin
     Option.free;
     Option:=nil;
   end;
  raise ECompilerAbortSilent.Create;
end;


function is_identifier(const s: TCmdStr): boolean;
var
  i: longint;
begin
  result:=false;
  if (s='') or not (s[1] in ['A'..'Z','a'..'z','_']) then
    exit;
  for i:=2 to length(s) do
    if not (s[I] in ['A'..'Z','a'..'z','0'..'9','_']) then
      exit;
  result:=true;
end;


procedure Toption.WriteLogo;
var
  msg : TMsgStr;
  p : pchar;
begin
  if not LogoWritten then
    begin
      msg:=MessageStr(option_logo);
      p:=pchar(msg);
      while assigned(p) do
        Comment(V_Normal,GetMsgLine(p));
      LogoWritten:= true;
    end;
end;


procedure Toption.WriteInfo (More: string);
var
  msg_str: TMsgStr;
  p : pchar;
  hs,hs1,hs3,s : TCmdStr;
  J: longint;
const
  NewLineStr = '$\n';
  OSTargetsPlaceholder = '$OSTARGETS';
  CPUListPlaceholder = '$INSTRUCTIONSETS';
  FPUListPlaceholder = '$FPUINSTRUCTIONSETS';
  ABIListPlaceholder = '$ABITARGETS';
  OptListPlaceholder = '$OPTIMIZATIONS';
  WPOListPlaceholder = '$WPOPTIMIZATIONS';
  AsmModeListPlaceholder = '$ASMMODES';
  ControllerListPlaceholder = '$CONTROLLERTYPES';
  FeatureListPlaceholder = '$FEATURELIST';

  procedure SplitLine (var OrigString: TCmdStr; const Placeholder: TCmdStr;
                                                 var RemainderString: TCmdStr);
  var
    I: longint;
    HS2: TCmdStr;
  begin
    RemainderString := '';
    if OrigString = '' then
     Exit;
    repeat
     I := Pos (NewLineStr, OrigString);
     if I > 0 then
      begin
       HS2 := Copy (OrigString, 1, Pred (I));
{ Stop if this line contains the placeholder for list replacement }
       if Pos (Placeholder, HS2) > 0 then
        begin
         RemainderString := Copy (OrigString, I + Length (NewLineStr),
                                Length (OrigString) - I - Length (NewLineStr));
{ Special case - NewLineStr at the end of the line }
         if RemainderString = '' then
          RemainderString := NewLineStr;
         OrigString := HS2;
         Exit;
        end;
       Comment (V_Normal, HS2);
       Delete (OrigString, 1, Pred (I) + Length (NewLineStr));
      end;
    until I = 0;
    if (OrigString <> '') and (Pos (Placeholder, OrigString) = 0) then
     Comment (V_Normal, OrigString);
  end;

  procedure ListOSTargets (OrigString: TCmdStr);
  var
    target : tsystem;
  begin
    SplitLine (OrigString, OSTargetsPlaceholder, HS3);
    for target:=low(tsystem) to high(tsystem) do
    if assigned(targetinfos[target]) then
     begin
      hs1:=targetinfos[target]^.shortname;
      if OrigString = '' then
       WriteLn (hs1)
      else
       begin
        hs := OrigString;
        hs1:=hs1 + ': ' + targetinfos[target]^.name;
        if tf_under_development in targetinfos[target]^.flags then
         hs1:=hs1+' {*}';
        Replace(hs,OSTargetsPlaceholder,hs1);
        Comment(V_Normal,hs);
       end;
     end;
    OrigString := HS3;
    SplitLine (OrigString, OSTargetsPlaceholder, HS3);
  end;

  procedure ListCPUInstructionSets (OrigString: TCmdStr);
  var
    cpu : tcputype;
  begin
    SplitLine (OrigString, CPUListPlaceholder, HS3);
    hs1:='';
    for cpu:=low(tcputype) to high(tcputype) do
     begin
      if (OrigString = '') then
       begin
        if CPUTypeStr [CPU] <> '' then
         WriteLn (CPUTypeStr [CPU]);
       end
      else
       begin
        if length(hs1+cputypestr[cpu])>70 then
         begin
          hs:=OrigString;
          Replace(hs,CPUListPlaceholder,hs1);
          Comment(V_Normal,hs);
          hs1:=''
         end
        else if hs1<>'' then
         hs1:=hs1+',';
        if cputypestr[cpu]<>'' then
         hs1:=hs1+cputypestr[cpu];
       end;
     end;
    if (OrigString <> '') and (hs1 <> '') then
     begin
      hs:=OrigString;
      Replace(hs,CPUListPlaceholder,hs1);
      Comment(V_Normal,hs);
      hs1:=''
     end;
    OrigString := HS3;
    SplitLine (OrigString, CPUListPlaceholder, HS3);
  end;

  procedure ListFPUInstructionSets (OrigString: TCmdStr);
  var
    fpu : tfputype;
  begin
    SplitLine (OrigString, FPUListPlaceholder, HS3);
    hs1:='';
    for fpu:=low(tfputype) to high(tfputype) do
     begin
      if (OrigString = '') then
       begin
        if FPUTypeStr [FPU] <> '' then
         WriteLn (FPUTypeStr [FPU]);
       end
      else
       begin
        if length(hs1+fputypestr[fpu])>70 then
         begin
          hs:=OrigString;
          Replace(hs,FPUListPlaceholder,hs1);
          Comment(V_Normal,hs);
          hs1:=''
         end
        else if hs1<>'' then
         hs1:=hs1+',';
        if fputypestr[fpu]<>'' then
         hs1:=hs1+fputypestr[fpu];
       end;
     end;
    if (OrigString <> '') and (hs1 <> '') then
     begin
      hs:=OrigString;
      Replace(hs,FPUListPlaceholder,hs1);
      Comment(V_Normal,hs);
      hs1:=''
     end;
    OrigString := HS3;
    SplitLine (OrigString, FPUListPlaceholder, HS3);
  end;

  procedure ListABITargets (OrigString: TCmdStr);
  var
    abi : tabi;
  begin
    SplitLine (OrigString, ABIListPlaceholder, HS3);
    for abi:=low(abi) to high(abi) do
     begin
      if not abiinfo[abi].supported then
       continue;
      hs1:=abiinfo[abi].name;
      if hs1<>'' then
       begin
        if OrigString = '' then
         WriteLn (HS1)
        else
         begin
          hs:=OrigString;
          Replace(hs,ABIListPlaceholder,hs1);
          Comment(V_Normal,hs);
         end;
       end;
     end;
    OrigString := HS3;
    SplitLine (OrigString, ABIListPlaceholder, HS3);
  end;

  procedure ListOptimizations (OrigString: TCmdStr);
  var
    opt : toptimizerswitch;
  begin
    SplitLine (OrigString, OptListPlaceholder, HS3);
    for opt:=low(toptimizerswitch) to high(toptimizerswitch) do
     begin
      if opt in supported_optimizerswitches then
       begin
        hs1:=OptimizerSwitchStr[opt];
        if hs1<>'' then
         begin
          if OrigString = '' then
           WriteLn (hs1)
          else
           begin
            hs:=OrigString;
            Replace(hs,OptListPlaceholder,hs1);
            Comment(V_Normal,hs);
           end;
         end;
       end;
     end;
    OrigString := HS3;
    SplitLine (OrigString, OptListPlaceholder, HS3);
  end;

  procedure ListWPOptimizations (OrigString: TCmdStr);
  var
    wpopt: twpoptimizerswitch;
  begin
    SplitLine (OrigString, WPOListPlaceholder, HS3);
    for wpopt:=low(twpoptimizerswitch) to high(twpoptimizerswitch) do
     begin
{     currently all whole program optimizations are platform-independent
      if opt in supported_wpoptimizerswitches then
}
       begin
        hs1:=WPOptimizerSwitchStr[wpopt];
        if hs1<>'' then
         begin
          if OrigString = '' then
           WriteLn (hs1)
          else
           begin
            hs:=OrigString;
            Replace(hs,WPOListPlaceholder,hs1);
            Comment(V_Normal,hs);
           end;
         end;
       end;
     end;
    OrigString := HS3;
    SplitLine (OrigString, WPOListPlaceholder, HS3);
  end;

  procedure ListAsmModes (OrigString: TCmdStr);
  var
    asmmode : tasmmode;
  begin
    SplitLine (OrigString, AsmModeListPlaceholder, HS3);
    for asmmode:=low(tasmmode) to high(tasmmode) do
    if assigned(asmmodeinfos[asmmode]) then
     begin
      hs1:=asmmodeinfos[asmmode]^.idtxt;
      if hs1<>'' then
       begin
        if OrigString = '' then
         WriteLn (hs1)
        else
         begin
          hs:=OrigString;
          Replace(hs,AsmModeListPlaceholder,hs1);
          Comment(V_Normal,hs);
         end;
       end;
     end;
    OrigString := HS3;
    SplitLine (OrigString, AsmModeListPlaceholder, HS3);
  end;

  procedure ListControllerTypes (OrigString: TCmdStr);
  var
    controllertype : tcontrollertype;
  begin
{$PUSH}
 {$WARN 6018 OFF} (* Unreachable code due to compile time evaluation *)
    if (ControllerSupport) then
     begin
      SplitLine (OrigString, ControllerListPlaceholder, HS3);
      hs1:='';
      for controllertype:=low(tcontrollertype) to high(tcontrollertype) do
       begin
        if (OrigString = '') then
         begin
          if Embedded_Controllers [ControllerType].ControllerTypeStr <> '' then
           WriteLn (Embedded_Controllers [ControllerType].ControllerTypeStr);
         end
        else
         begin
          if length(hs1+embedded_controllers[controllertype].ControllerTypeStr)
                                                                       >70 then
           begin
            hs:=OrigString;
            Replace(hs,ControllerListPlaceholder,hs1);
            Comment(V_Normal,hs);
            hs1:=''
           end
          else if hs1<>'' then
           hs1:=hs1+',';
          if embedded_controllers[controllertype].ControllerTypeStr<>'' then
           hs1:=hs1+embedded_controllers[controllertype].ControllerTypeStr;
         end;
       end;
      if (OrigString <> '') and (hs1<>'') then
       begin
        hs:=OrigString;
        Replace(hs,ControllerListPlaceholder,hs1);
        Comment(V_Normal,hs);
        hs1:=''
       end;
      OrigString := HS3;
      SplitLine (OrigString, ControllerListPlaceholder, HS3);
     end;
{$POP}
  end;

  procedure ListFeatures (OrigString: TCmdStr);
  var
    Feature: TFeature;
  begin
    SplitLine (OrigString, FeatureListPlaceholder, HS3);
    HS1 := '';
    for Feature := Low (TFeature) to High (TFeature) do
     begin
      if (OrigString = '') then
       begin
        if FeatureStr [Feature] <> '' then
         WriteLn (FeatureStr [Feature]);
       end
      else
       begin
        if Length (HS1 + FeatureStr [Feature]) > 70 then
         begin
          HS := OrigString;
          Replace (HS, FeatureListPlaceholder, HS1);
          Comment (V_Normal, HS);
          HS1 := ''
         end
        else if HS1 <> '' then
         HS1 := HS1 + ',';
        if FeatureStr [Feature] <> '' then
         HS1 := HS1 + FeatureStr [Feature];
       end;
     end;
    if (OrigString <> '') and (HS1 <> '') then
     begin
      HS := OrigString;
      Replace (HS, FeatureListPlaceholder, HS1);
      Comment (V_Normal, HS);
      HS1 := ''
     end;
    OrigString := HS3;
    SplitLine (OrigString, FeatureListPlaceholder, HS3);
  end;

begin
  if More = '' then
   begin
    msg_str:=MessageStr(option_info);
    p:=pchar(msg_str);
    while assigned(p) do
     begin
      s:=GetMsgLine(p);
      { list permitted values for certain options }
      if pos(OSTargetsPlaceholder,s)>0 then
       ListOSTargets (S)
      else if pos(CPUListPlaceholder,s)>0 then
       ListCPUInstructionSets (S)
      else if pos(FPUListPlaceholder,s)>0 then
       ListFPUInstructionSets (S)
      else if pos(ABIListPlaceholder,s)>0 then
       ListABITargets (S)
      else if pos(OptListPlaceholder,s)>0 then
       ListOptimizations (S)
      else if pos(WPOListPlaceholder,s)>0 then
       ListWPOptimizations (S)
      else if pos(AsmModeListPlaceholder,s)>0 then
       ListAsmModes (S)
      else if pos(ControllerListPlaceholder,s)>0 then
       ListControllerTypes (S)
      else if pos(FeatureListPlaceholder,s)>0 then
       ListFeatures (S)
      else
       Comment(V_Normal,s);
     end;
   end
  else
   begin
    J := 1;
    while J <= Length (More) do
     begin
      if J > 1 then
       WriteLn;  (* Put empty line between multiple sections *)
      case More [J] of
       'a': ListABITargets ('');
       'c': ListCPUInstructionSets ('');
       'f': ListFPUInstructionSets ('');
       'i': ListAsmModes ('');
       'o': ListOptimizations ('');
       'r': ListFeatures ('');
       't': ListOSTargets ('');
       'u': ListControllerTypes ('');
       'w': ListWPOptimizations ('');
      else
       IllegalPara ('-i' + More);
      end;
      Inc (J);
     end;
   end;
  StopOptions(0);
end;


procedure Toption.WriteHelpPages;

  function PadEnd(s:string;i:longint):string;
  begin
    if length(s) >= i then
     S := S + ' '
    else
     while (length(s)<i) do
      s:=s+' ';
    PadEnd:=s;
  end;

var
  lastident,
  j,outline,
  ident,
  HelpLineHeight,
  lines : longint;
  show  : boolean;
  opt   : string[32];
  input,
  HelpLine,
  s     : string;
  p     : pchar;
  msg_str: TMsgStr;
begin
  WriteLogo;
  Lines:=4;
  if FPCHelpLines then
   Message1(option_usage,FixFileName(FPCBinaryPath))
  else
   Message1(option_usage,FixFileName(system.paramstr(0)));
  lastident:=0;
  msg_str:=MessageStr(option_help_pages);
  p:=pchar(msg_str);
  while assigned(p) do
   begin
   { get a line and reset }
     s:=GetMsgLine(p);
     ident:=0;
     show:=false;
   { parse options }
     case s[1] of
      'F': if FPCHelpLines then
            Show := true;
{$ifdef UNITALIASES}
      'a',
{$endif}
{$ifdef EXTDEBUG}
      'e',
{$endif EXTDEBUG}
{$ifdef i386}
      '3',
{$endif}
{$ifdef x86_64}
      '4',
{$endif}
{$ifdef m68k}
      '6',
{$endif}
{$ifdef i8086}
      '8',
{$endif}
{$ifdef aarch64}
      'a',
{$endif}
{$ifdef arm}
      'A',
{$endif}
{$ifdef mipsel}
      'm',
{$endif}
{$ifdef mipseb}
      'M',
{$endif}
{$ifdef powerpc}
      'P',
{$endif}
{$ifdef powerpc64}
      'p',
{$endif}
{$ifdef sparc}
      'S',
{$endif}
{$ifdef sparc64}
      's',
{$endif}
{$ifdef riscv32}
      'R',
{$endif}
{$ifdef riscv64}
      'r',
{$endif}
{$ifdef avr}
      'V',
{$endif}
{$ifdef jvm}
      'J',
{$endif}
{$ifdef llvm}
      'L',
{$endif}
      '*' : show:=true;
     end;
     if show then
      begin
        case s[2] of
         'g',
{$ifdef Unix}
         'L',
{$endif}
{$ifdef os2}
         'O',
{$endif}
         '*' : show:=true;
        else
         show:=false;
        end;
      end;
   { now we may show the message or not }
     if show then
      begin
        case s[3] of
         '0' : begin
                 ident:=0;
                 outline:=0;
               end;
         '1' : begin
                 ident:=2;
                 outline:=7;
               end;
         '2' : begin
                 ident:=6;
                 outline:=11;
               end;
         '3' : begin
                 ident:=9;
                 outline:=11;
               end;
         else
           internalerror(2013112906);
        end;
        j:=pos('_',s);
        opt:=Copy(s,4,j-4);
        if opt='*' then
         opt:=''
        else
        if (opt=' ') or (opt[1]='@') then
         opt:=PadEnd(opt,outline)
        else
         opt:=PadEnd('-'+opt,outline);
        if (ident=0) and (lastident<>0) then
         begin
           Comment(V_Normal,'');
           inc(Lines);
         end;
        HelpLine := PadEnd('',ident)+opt+Copy(s,j+1,255);
        if HelpLine = '' then
         HelpLineHeight := 1
        else
         HelpLineHeight := Succ (CharLength (HelpLine) div Page_Width);
      { page full ? }
        if (lines + HelpLineHeight >= page_size - 1) then
         begin
           if not NoPressEnter then
            begin
              Message(option_help_press_enter);
              readln(input);
              if upper(input)='Q' then
               StopOptions(0);
            end;
           lines:=0;
         end;
        Comment(V_Normal,HelpLine);
        LastIdent:=Ident;
        Inc (Lines, HelpLineHeight);
      end;
   end;
  StopOptions(0);
end;


procedure Toption.IllegalPara(const opt:TCmdStr);
begin
  Message1(option_illegal_para,opt);
  Message(option_help_pages_para);
  StopOptions(1);
end;


procedure toption.UnsupportedPara(const opt: TCmdStr);
begin
  Message1(option_unsupported_target,opt);
  StopOptions(1);
end;


procedure toption.IgnoredPara(const opt: TCmdStr);
begin
  Message1(option_ignored_target,opt);
end;


procedure toption.ForceStaticLinking;
begin
  def_system_macro('FPC_LINK_STATIC');
  undef_system_macro('FPC_LINK_SMART');
  undef_system_macro('FPC_LINK_DYNAMIC');
  include(init_settings.globalswitches,cs_link_static);
  exclude(init_settings.globalswitches,cs_link_smart);
  exclude(init_settings.globalswitches,cs_link_shared);
  LinkTypeSetExplicitly:=true;
end;


function toption.ParseMacVersionMin(out minstr, emptystr: string; const compvarname, value: string; ios: boolean): boolean;

  function subval(start,maxlen: longint; out stop: longint): string;
    var
      i: longint;
    begin
      result:='';
      i:=start;
      while (i<=length(value)) and
            (value[i] in ['0'..'9']) do
        inc(i);
      { sufficient amount of digits? }
      if (i=start) or
         (i-start>maxlen) then
        exit;
      result:=copy(value,start,i-start);
      stop:=i;
    end;

  var
    temp,
    compvarvalue: string[15];
    i: longint;
    osx_minor_two_digits: boolean;
  begin
    minstr:=value;
    emptystr:='';
    MacVersionSet:=false;
    { check whether the value is a valid version number }
    if value='' then
      begin
        undef_system_macro(compvarname);
        exit(true);
      end;
    { major version number }
    compvarvalue:=subval(1,2,i);
    { not enough digits -> invalid }
    if compvarvalue='' then
      exit(false);
    { already end of string -> invalid }
    if (i>=length(value)) or
       (value[i]<>'.') then
      exit(false);
    { minor version number }
    temp:=subval(i+1,2,i);
    if temp='' then
      exit(false);
    { on Mac OS X, the minor version number was originally limited to 1 digit;
      with 10.10 the format changed and two digits were also supported; on iOS,
      the minor version number always takes up two digits }
    osx_minor_two_digits:=false;
    if not ios then
      begin
        { if the minor version number is two digits on OS X (the case since
          OS X 10.10), we also have to add two digits for the patch level}
        if length(temp)=2 then
          osx_minor_two_digits:=true;
      end
    { the minor version number always takes up two digits on iOS }
    else if length(temp)=1 then
      temp:='0'+temp;
    compvarvalue:=compvarvalue+temp;
    { optional patch level }
    if i<=length(value) then
      begin
        if value[i]<>'.' then
          exit(false);
        temp:=subval(i+1,2,i);
        if temp='' then
          exit(false);
        { there's only room for a single digit patch level in the version macro
          for Mac OS X. gcc sets it to zero if there are more digits, but that
          seems worse than clamping to 9 (don't declare as invalid like with
          minor version number, because there is a precedent like 10.4.11).

          As of OS X 10.10 there are two digits for the patch level
        }
        if not ios and
           not osx_minor_two_digits then
          begin
            if length(temp)<>1 then
              temp:='9';
          end
        else
          begin
            { on iOS, the patch level is always two digits }
            if length(temp)=1 then
              temp:='0'+temp;
          end;
        compvarvalue:=compvarvalue+temp;
        { must be the end }
        if i<=length(value) then
          exit(false);
      end
    else if not ios and
       not osx_minor_two_digits then
      begin
        compvarvalue:=compvarvalue+'0';
        minstr:=minstr+'.0'
      end
    else
      begin
        compvarvalue:=compvarvalue+'00';
        { command line versions still only use one 0 though }
        minstr:=minstr+'.0'
      end;
    set_system_compvar(compvarname,compvarvalue);
    MacVersionSet:=true;
    result:=true;
  end;


procedure TOption.MaybeSetDefaultMacVersionMacro;
var
  envstr: ansistring;
begin
  if not(target_info.system in systems_darwin) then
    exit;
  if MacVersionSet then
    exit;
  { check for deployment target set via environment variable }
  if not(target_info.system in [system_i386_iphonesim,system_arm_darwin,system_aarch64_darwin,system_x86_64_iphonesim]) then
    begin
      envstr:=GetEnvironmentVariable('MACOSX_DEPLOYMENT_TARGET');
      if envstr<>'' then
        if not ParseMacVersionMin(MacOSXVersionMin,iPhoneOSVersionMin,'MAC_OS_X_VERSION_MIN_REQUIRED',envstr,false) then
          Message1(option_invalid_macosx_deployment_target,envstr)
        else
          begin
{$ifdef llvm}
             { We only support libunwind as part of libsystem, which happened in Mac OS X 10.6 }
            if CompareVersionStrings(MacOSXVersionMin,'10.6')<=0 then
              Message1(option_invalid_macosx_deployment_target,envstr);
{$endif}
            exit;
          end;
    end
  else
    begin
      envstr:=GetEnvironmentVariable('IPHONEOS_DEPLOYMENT_TARGET');
      if envstr<>'' then
        if not ParseMacVersionMin(iPhoneOSVersionMin,MacOSXVersionMin,'IPHONE_OS_VERSION_MIN_REQUIRED',envstr,true) then
          Message1(option_invalid_iphoneos_deployment_target,envstr)
        else
          exit;
    end;
  { nothing specified -> defaults }
  case target_info.system of
    system_powerpc_darwin:
      begin
        set_system_compvar('MAC_OS_X_VERSION_MIN_REQUIRED','1030');
        MacOSXVersionMin:='10.3';
      end;
    system_powerpc64_darwin:
      begin
        set_system_compvar('MAC_OS_X_VERSION_MIN_REQUIRED','1040');
        MacOSXVersionMin:='10.4';
      end;
    system_i386_darwin,
    system_x86_64_darwin:
      begin
        set_system_compvar('MAC_OS_X_VERSION_MIN_REQUIRED','1080');
        MacOSXVersionMin:='10.8';
      end;
    system_arm_darwin,
    system_i386_iphonesim:
      begin
        set_system_compvar('IPHONE_OS_VERSION_MIN_REQUIRED','30000');
        iPhoneOSVersionMin:='3.0';
      end;
    system_aarch64_darwin,
    system_x86_64_iphonesim:
      begin
        set_system_compvar('IPHONE_OS_VERSION_MIN_REQUIRED','70000');
        iPhoneOSVersionMin:='7.0';
      end
    else
      internalerror(2012031001);
  end;
end;

procedure TOption.VerifyTargetProcessor;
  begin
    { no custom target processor specified -> ok }
    if processorstr='' then
      exit;
    { custom target processor specified -> verify it's the one we support }
    if upcase(processorstr)<>upcase(target_cpu_string) then
      Message1(option_invalid_target_architecture,processorstr);
  end;


function Toption.Unsetbool(var Opts:TCmdStr; Pos: Longint; const FullPara: TCmdStr; RequireBoolPara: boolean):boolean;
{ checks if the character after pos in Opts is a + or a - and returns resp.
  false or true. If it is another character (or none), it also returns false }
begin
  UnsetBool := false;
  if Length(Opts)>Pos then
   begin
    inc(Pos);
    UnsetBool := Opts[Pos] = '-';
    if Opts[Pos] in ['-','+']then
     delete(Opts,Pos,1)
    else if RequireBoolPara then
     IllegalPara(FullPara);
   end;
end;

procedure TOption.interpret_option(const opt:TCmdStr;ispara:boolean);
var
  code : integer;
  c    : char;
  more : TCmdStr;
  major,minor : longint;
  error : integer;
  j,l   : longint;
  d,s   : TCmdStr;
  hs    : TCmdStr;
  unicodemapping : punicodemap;
{$ifdef llvm}
  disable: boolean;
{$endif}
begin
  if opt='' then
   exit;

  { only parse define,undef,target,verbosity,link etc options the firsttime
    -Us must now also be first-passed to avoid rejection of -Sf options
    earlier in command line }
  if firstpass and
     not(
         (opt[1]='-') and
         (
          ((length(opt)>1) and (opt[2] in ['i','d','v','T','u','n','X','l','U'])) or
          ((length(opt)>3) and (opt[2]='F') and (opt[3]='e')) or
          ((length(opt)>3) and (opt[2]='C') and (opt[3]='p')) or
          ((length(opt)>3) and (opt[2]='W') and (opt[3] in ['m','p']))
         )
        ) then
    exit;

  Message1(option_handling_option,opt);
  case opt[1] of
    '-' :
      begin
         more:=Copy(opt,3,2147483647);
         if firstpass then
           Message1(option_interpreting_firstpass_option,opt)
         else
           Message1(option_interpreting_option,opt);
         case opt[2] of
           '?' :
             begin
               if (More <> '') and (More [1] = 'F') then
                 begin
                   FPCHelpLines := true;
                   Delete (More, 1, 1);
                   FPCBinaryPath := More;
                 end;
               WriteHelpPages;
             end;

           'a' :
             begin
               include(init_settings.globalswitches,cs_asm_leave);
               j:=1;
               while j<=length(more) do
                begin
                  case more[j] of
                    '5' :
                      if (target_info.system in systems_all_windows+systems_nativent-[system_i8086_win16])
                         or (target_info.cpu in [cpu_mipseb, cpu_mipsel]) then
                        begin
                          if UnsetBool(More, j, opt, false) then
                            exclude(init_settings.globalswitches,cs_asm_pre_binutils_2_25)
                          else
                            include(init_settings.globalswitches,cs_asm_pre_binutils_2_25);
                        end
                      else
                        IllegalPara(opt);
                    'l' :
                      include(init_settings.globalswitches,cs_asm_source);
                    'r' :
                      include(init_settings.globalswitches,cs_asm_regalloc);
                    't' :
                      include(init_settings.globalswitches,cs_asm_tempalloc);
                    'n' :
                      include(init_settings.globalswitches,cs_asm_nodes);
                    { -ao option must be the last, everything behind it is passed directly to
                      external assembler, it is ignored if internal assembler is used. }
                    'o' :
                      begin
                        asmextraopt:=copy(more,j+1,length(more)-j);
                        break;
                      end;
                    'p' :
                      begin
                        exclude(init_settings.globalswitches,cs_asm_leave);
                        if UnsetBool(More, 0, opt, false) then
                          exclude(init_settings.globalswitches,cs_asm_pipe)
                        else
                          include(init_settings.globalswitches,cs_asm_pipe);
                      end;
                    '-' :
                      init_settings.globalswitches:=init_settings.globalswitches -
                          [cs_asm_leave, cs_asm_source,cs_asm_regalloc, cs_asm_tempalloc,
                           cs_asm_nodes, cs_asm_pipe];
                    else
                      IllegalPara(opt);
                  end;
                  inc(j);
                end;
             end;

           'A' :
             begin
               paratargetasm:=find_asm_by_string(More);
               if paratargetasm=as_none then
                 IllegalPara(opt);
             end;

           'b' :
             begin
               // Message1(option_obsolete_switch,'-b');
               if UnsetBool(More,0,opt,false) then
                 begin
                   init_settings.moduleswitches:=init_settings.moduleswitches-[cs_browser];
                   init_settings.moduleswitches:=init_settings.moduleswitches-[cs_local_browser];
                 end
               else
                 begin
                   init_settings.moduleswitches:=init_settings.moduleswitches+[cs_browser];
                 end;
               if More<>'' then
                 if (More='l') or (More='l+') then
                   init_settings.moduleswitches:=init_settings.moduleswitches+[cs_local_browser]
                 else if More='l-' then
                   init_settings.moduleswitches:=init_settings.moduleswitches-[cs_local_browser]
                 else
                   IllegalPara(opt);
             end;

           'B' :
             do_build:=not UnSetBool(more,0,opt,true);

           'C' :
             begin
               j:=1;
               while j<=length(more) do
                begin
                  case more[j] of
                    '3' :
                      If UnsetBool(More, j, opt, false) then
                        exclude(init_settings.localswitches,cs_ieee_errors)
                      Else
                        include(init_settings.localswitches,cs_ieee_errors);
                    'a' :
                      begin
                        s:=upper(copy(more,j+1,length(more)-j));
                        if not(SetAbiType(s,target_info.abi)) then
                          IllegalPara(opt);
                        ABISetExplicitly:=true;
                        break;
                      end;

                    'b' :
                       begin
                         if UnsetBool(More, j, opt, false) then
                           target_info.endian:=endian_little
                         else
                           target_info.endian:=endian_big;
                       end;

                    'c' :
                       begin
                         if not SetAktProcCall(upper(copy(more,j+1,length(more)-j)),init_settings.defproccall) then
                          IllegalPara(opt);
                         break;
                       end;
{$ifdef cpufpemu}
                    'e' :
                       begin
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.moduleswitches,cs_fp_emulation)
                         Else
                           include(init_settings.moduleswitches,cs_fp_emulation);
                       end;
{$endif cpufpemu}
                    'E' :
                      If UnsetBool(More, j, opt, false) then
                        exclude(init_settings.localswitches,cs_check_fpu_exceptions)
                      Else
                        include(init_settings.localswitches,cs_check_fpu_exceptions);
                    'f' :
                      begin
                        s:=upper(copy(more,j+1,length(more)-j));
                        if not(SetFpuType(s,init_settings.fputype)) then
                          IllegalPara(opt);
                        FPUSetExplicitly:=True;
                        break;
                      end;
                    'F' :
                       begin
                         if not SetMinFPConstPrec(copy(more,j+1,length(more)-j),init_settings.minfpconstprec) then
                           IllegalPara(opt);
                         break;
                       end;
                    'g' :
                       begin
                         if tf_no_pic_supported in target_info.flags then
                           begin
                             { consume a possible '-' coming after it }
                             UnsetBool(More, j, opt, false);
                             message(scan_w_pic_ignored);
                           end
                         else if UnsetBool(More, j, opt, false) then
                           exclude(init_settings.moduleswitches,cs_create_pic)
                         else
                           include(init_settings.moduleswitches,cs_create_pic);
                      end;
                    'h' :
                      begin
                         l:=pos(',',copy(more,j+1,length(more)-j));
                         if l=0 then
                           l:=length(more)-j+1;
                         val(copy(more,j+1,l-1),heapsize,code);
                         if (code<>0)
{$ifdef AVR}
                         or (heapsize<32)
{$else AVR}
                         or (heapsize<1024)
{$endif AVR}
                         then
                           IllegalPara(opt)
                         else if l<=length(more)-j then
                           begin
                             val(copy(more,j+l+1,length(more)),maxheapsize,code);
                             if code<>0 then
                               IllegalPara(opt)
                             else if (maxheapsize<heapsize) then
                               begin
                                 message(scan_w_heapmax_lessthan_heapmin);
                                 maxheapsize:=heapsize;
                               end;
                           end;
                         break;
                      end;
                    'i' :
                      If UnsetBool(More, j, opt, false) then
                        exclude(init_settings.localswitches,cs_check_io)
                      else
                        include(init_settings.localswitches,cs_check_io);
{$ifdef arm}
                    'I' :
                      begin
                        if (upper(copy(more,j+1,length(more)-j))='THUMB') and
                          { does selected CPU really understand thumb? }
                          (init_settings.cputype in cpu_has_thumb) then
                          init_settings.instructionset:=is_thumb
                        else if upper(copy(more,j+1,length(more)-j))='ARM' then
                          init_settings.instructionset:=is_arm
                        else
                          IllegalPara(opt);
                        break;
                      end;
{$endif arm}
{$ifdef llvm}
                    'L':
                      begin
                        l:=j+1;
                        while l<=length(More) do
                          begin
                            case More[l] of
                              'f':
                                begin
                                  More:=copy(More,l+1,length(More));
                                  disable:=Unsetbool(More,length(More)-1,opt,false);
                                  case More of
                                    'lto':
                                       begin
                                         if not disable then
                                           begin
                                             include(init_settings.moduleswitches,cs_lto);
                                             LTOExt:='.bc';
                                           end
                                         else
                                           exclude(init_settings.moduleswitches,cs_lto);
                                       end;
                                     'ltonosystem':
                                         begin
                                           if not disable then
                                             begin
                                               include(init_settings.globalswitches,cs_lto_nosystem);
                                             end
                                           else
                                             exclude(init_settings.globalswitches,cs_lto_nosystem);
                                         end;
                                    else
                                      begin
                                        IllegalPara(opt);
                                      end;
                                  end;
                                  l:=length(more)+1;
                                end;
                              'v':
                                begin
                                  init_settings.llvmversion:=llvmversion2enum(copy(More,l+1,length(More)));
                                  if init_settings.llvmversion=llvmver_invalid then
                                    begin
                                      IllegalPara(opt);
                                    end;
                                  l:=length(More)+1;
                                end
                              else
                                begin
                                  IllegalPara(opt);
                                end;
                            end;
                          end;
                        j:=l;
                      end;
{$endif llvm}
                    'n' :
                      If UnsetBool(More, j, opt, false) then
                        exclude(init_settings.globalswitches,cs_link_nolink)
                      Else
                        include(init_settings.globalswitches,cs_link_nolink);
                    'N' :
                      If UnsetBool(More, j, opt, false) then
                        exclude(init_settings.localswitches,cs_check_low_addr_load)
                      Else
                        include(init_settings.localswitches,cs_check_low_addr_load);
                    'o' :
                      If UnsetBool(More, j, opt, false) then
                        exclude(init_settings.localswitches,cs_check_overflow)
                      Else
                        include(init_settings.localswitches,cs_check_overflow);
                    'O' :
                      If UnsetBool(More, j, opt, false) then
                        exclude(init_settings.localswitches,cs_check_ordinal_size)
                      Else
                        include(init_settings.localswitches,cs_check_ordinal_size);
                    'p' :
                      begin
                        s:=upper(copy(more,j+1,length(more)-j));
                        if not(Setcputype(s,init_settings)) then
                          IllegalPara(opt);
                        CPUSetExplicitly:=true;
                        break;
                      end;
                    'P':
                      begin
                        delete(more,1,1);
                        case upper(copy(more,1,pos('=',more)-1)) of
                          'PACKSET':
                            begin
                              delete(more,1,pos('=',more));
                              case more of
                                '0','DEFAULT','NORMAL':
                                  init_settings.setalloc:=0;
                                '1','2','4','8':
                                  init_settings.setalloc:=StrToInt(more);
                                else
                                  IllegalPara(opt);
                              end
                            end;
                          'PACKENUM':
                            begin
                              delete(more,1,pos('=',more));
                              case more of
                                '0','DEFAULT','NORMAL':
                                  init_settings.packenum:=4;
                                '1','2','4':
                                  init_settings.packenum:=StrToInt(more);
                                else
                                  IllegalPara(opt);
                              end;
                            end;
                          'PACKRECORD':
                            begin
                              delete(more,1,pos('=',more));
                              case more of
                                '0','DEFAULT','NORMAL':
                                  init_settings.packrecords:=default_settings.packrecords;
                                '1','2','4','8','16','32':
                                  init_settings.packrecords:=StrToInt(more);
                                else
                                  IllegalPara(opt);
                              end;
                            end
                          else
                            IllegalPara(opt);
                        end;
                      end;
                    'r' :
                      If UnsetBool(More, j, opt, false) then
                        exclude(init_settings.localswitches,cs_check_range)
                      Else
                        include(init_settings.localswitches,cs_check_range);
                    'R' :
                      If UnsetBool(More, j, opt, false) then
                        begin
                          exclude(init_settings.localswitches,cs_check_range);
                          exclude(init_settings.localswitches,cs_check_object);
                        end
                      Else
                        begin
                          include(init_settings.localswitches,cs_check_range);
                          include(init_settings.localswitches,cs_check_object);
                        end;
                    's' :
                      begin
                         val(copy(more,j+1,length(more)-j),stacksize,code);
                         if (code<>0)
{$ifdef cpu16bitaddr}
                            or (stacksize>=65521)
{$else cpu16bitaddr}
                            or (stacksize>=67107840)
{$endif cpu16bitaddr}
                            or (stacksize<1024) then
                          IllegalPara(opt);
                         break;
                      end;
                    't' :
                       If UnsetBool(More, j, opt, false) then
                         exclude(init_settings.localswitches,cs_check_stack)
                       Else
                         include(init_settings.localswitches,cs_check_stack);
                    'D' :
                       If UnsetBool(More, j, opt, false) then
                         exclude(init_settings.moduleswitches,cs_create_dynamic)
                       Else
                         include(init_settings.moduleswitches,cs_create_dynamic);
                    'X' :
                       If UnsetBool(More, j, opt, false) then
                         exclude(init_settings.moduleswitches,cs_create_smart)
                       Else
                         include(init_settings.moduleswitches,cs_create_smart);
                    'T' :
                      begin
                        if not UpdateTargetSwitchStr(copy(more,j+1,length(more)),init_settings.targetswitches,true) then
                          IllegalPara(opt);
                        break;
                      end;
                    'v' :
                       If target_info.system in systems_jvm then
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.localswitches,cs_check_var_copyout)
                         Else
                           include(init_settings.localswitches,cs_check_var_copyout)
                       else
                         IllegalPara(opt);
                    'V':
                      begin
                        s:=upper(copy(more,j+1,length(more)-j));
                        if s='GLOBAL-DYNAMIC' then
                          init_settings.tlsmodel:=tlsm_global_dynamic
                        else if s='LOCAL-EXEC' then
                          init_settings.tlsmodel:=tlsm_local_exec
                        else
                          IllegalPara(opt);
                        break;
                      end;
                    else
                      IllegalPara(opt);
                  end;
                  inc(j);
                end;
             end;

           'd' :
             begin
               l:=Pos(':=',more);
               if l>0 then
                 hs:=copy(more,1,l-1)
               else
                 hs:=more;
               if (not is_identifier(hs)) then
                 begin
                   if hs='' then
                     Message1(option_missing_arg,'-d')
                   else
                     Message1(option_malformed_para,opt);
                   StopOptions(1);
                 end;
               if l>0 then
                 begin
                   if cs_support_macro in init_settings.moduleswitches then
                     set_system_macro(hs,Copy(more,l+2,255))
                   else
                     set_system_compvar(hs,Copy(more,l+2,255));
                 end
               else
                 def_system_macro(hs);
             end;
           'D' :
             begin
               include(init_settings.globalswitches,cs_link_deffile);
               j:=1;
               while j<=length(more) do
                begin
                  case more[j] of
                    'd' :
                      begin
                        description:=Copy(more,j+1,255);
                        break;
                      end;
                    'v' :
                      begin
                        dllversion:=Copy(more,j+1,255);
                        l:=pos('.',dllversion);
                        dllminor:=0;
                        error:=0;
                        if l>0 then
                         begin
                           val(copy(dllversion,l+1,255),minor,error);
                           if (error=0) and
                              (minor>=0) and (minor<=$ffff) then
                             dllminor:=minor
                           else
                             if error=0 then
                               error:=1;
                         end;
                        if l=0 then
                          l:=256;
                        dllmajor:=1;
                        major:=0;
                        if error=0 then
                          val(copy(dllversion,1,l-1),major,error);
                        if (error=0) and (major>=0) and (major<=$ffff) then
                          dllmajor:=major
                        else
                          if error=0 then
                            error:=1;
                        if error<>0 then
                          Message1(scan_w_wrong_version_ignored,dllversion);
                        break;
                      end;
                    'w' :
                      usewindowapi:=true;
                    '-' :
                      begin
                        exclude(init_settings.globalswitches,cs_link_deffile);
                        usewindowapi:=false;
                      end;
                    else
                      IllegalPara(opt);
                  end;
                  inc(j);
                end;
             end;

           'e' :
             exepath:=FixPath(More,true);

           'E' :
             begin
               if UnsetBool(More, 0, opt, true) then
                 exclude(init_settings.globalswitches,cs_link_nolink)
               else
                 include(init_settings.globalswitches,cs_link_nolink);
             end;

           'f' :
             begin
               if more='PIC' then
                 begin
                   if tf_no_pic_supported in target_info.flags then
                     message(scan_w_pic_ignored)
                   else
                     include(init_settings.moduleswitches,cs_create_pic)
                 end
               else
                 IllegalPara(opt);
             end;

           'F' :
             begin
               if more='' then
                 IllegalPara(opt);
               c:=more[1];
               Delete(more,1,1);
               DefaultReplacements(More);
               case c of
                 'a' :
                   autoloadunits:=more;
                 'c' :
                   begin
                     { if we first specify that the system code page should be
                       used and then explicitly specify a code page, unset the
                       flag that we're using the system code page again }
                     SetCompileModeSwitch('SYSTEMCODEPAGE-',true);
                     if (upper(more)='UTF8') or (upper(more)='UTF-8') then
                       init_settings.sourcecodepage:=CP_UTF8
                     else if not(cpavailable(more)) then
                       Message1(option_code_page_not_available,more)
                     else
                       init_settings.sourcecodepage:=codepagebyname(more);
                     include(init_settings.moduleswitches,cs_explicit_codepage);
                   end;
                 'C' :
                   RCCompiler:=More;
                 'd' :
                   if UnsetBool(more, 0, opt, true) then
                     init_settings.disabledircache:=false
                   else
                     init_settings.disabledircache:=true;
                 'D' :
                   utilsdirectory:=FixPath(More,true);
                 'e' :
                   SetRedirectFile(More);
                 'E' :
                   OutputExeDir:=FixPath(More,true);
                 'f' :
                     if (target_info.system in systems_darwin) then
                       if ispara then
                         ParaFrameworkPath.AddPath(More,false)
                       else
                         frameworksearchpath.AddPath(More,true)
                     else
                       IllegalPara(opt);
                 'i' :
                   begin
                     if ispara then
                       ParaIncludePath.AddPath(More,false)
                     else
                       includesearchpath.AddPath(More,true);
                   end;
                 'm' :
                   begin
                     s:=ExtractFileDir(more);
                     if TryStrToInt(ExtractFileName(more),j) then
                       begin
                         unicodemapping:=loadunicodemapping(More,More+'.txt',j);
                         if assigned(unicodemapping) then
                           registermapping(unicodemapping)
                         else
                           IllegalPara(opt);
                       end
                     else
                       IllegalPara(opt);
                   end;
                 'M' :
                   unicodepath:=FixPath(More,true);
                 'g' :
                   Message2(option_obsolete_switch_use_new,'-Fg','-Fl');
                 'l' :
                   begin
                     if ispara then
                       ParaLibraryPath.AddLibraryPath(sysrootpath,More,false)
                     else
                       LibrarySearchPath.AddLibraryPath(sysrootpath,More,true)
                   end;
                 'L' :
                   begin
                     if More<>'' then
                       ParaDynamicLinker:=More
                     else
                       IllegalPara(opt);
                   end;
                 'N' :
                   begin
                     if more<>'' then
                       paranamespaces.insert(more)
                     else
                       illegalpara(opt);
                   end;
                 'o' :
                   begin
                     if ispara then
                       ParaObjectPath.AddPath(More,false)
                     else
                       ObjectSearchPath.AddPath(More,true);
                   end;
                 'P' :
                   begin
                     if ispara then
                       parapackages.add(more,nil)
                     else
                       add_package(more,true,true);
                   end;
                 'p' :
                   begin
                     if ispara then
                       parapackagepath.AddPath(More,false)
                     else
                       packagesearchpath.AddPath(More,true);
                   end;
                 'r' :
                   Msgfilename:=More;
                 'R' :
                   ResCompiler:=More;
                 'u' :
                   begin
                     if ispara then
                       ParaUnitPath.AddPath(More,false)
                     else
                       unitsearchpath.AddPath(More,true);
                   end;
                 'U' :
                   OutputUnitDir:=FixPath(More,true);
                 'W',
                 'w':
                   begin
                     if More<>'' then
                       begin
                         DefaultReplacements(More);
                         D:=ExtractFilePath(More);
                         if (D<>'') then
                           D:=FixPath(D,True);
                         D:=D+ExtractFileName(More);
                         if (c='W') then
                           WpoFeedbackOutput:=D
                         else
                           WpoFeedbackInput:=D;
                       end
                     else
                       IllegalPara(opt);
                   end;
                 else
                   IllegalPara(opt);
               end;
             end;

           'g' :
             begin
               if UnsetBool(More, 0, opt, false) then
                begin
                  exclude(init_settings.moduleswitches,cs_debuginfo);
                  exclude(init_settings.globalswitches,cs_use_heaptrc);
                  exclude(init_settings.globalswitches,cs_use_lineinfo);
                  exclude(init_settings.localswitches,cs_checkpointer);
                  localvartrashing := -1;
                end
               else
                begin
                  include(init_settings.moduleswitches,cs_debuginfo);
                  if paratargetdbg=dbg_none then
                    paratargetdbg:=target_info.dbg;
                end;
               if not RelocSectionSetExplicitly then
                 RelocSection:=false;
               j:=1;
               while j<=length(more) do
                 begin
                   case more[j] of
                     'c' :
                       begin
                         if UnsetBool(More, j, opt, false) then
                           exclude(init_settings.localswitches,cs_checkpointer)
                         else if (target_info.system in systems_support_checkpointer) then
                           begin
                             if do_release then
                               Message(option_gc_incompatible_with_release_flag)
                             else
                               include(init_settings.localswitches,cs_checkpointer);
                           end
                         else
                           UnsupportedPara('-gc');
                       end;
                     'h' :
                       begin
                         if UnsetBool(More, j, opt, false) then
                           exclude(init_settings.globalswitches,cs_use_heaptrc)
                         else
                           include(init_settings.globalswitches,cs_use_heaptrc);
                       end;
                     'l' :
                       begin
                         if UnsetBool(More, j, opt, false) then
                           exclude(init_settings.globalswitches,cs_use_lineinfo)
                         else
                           include(init_settings.globalswitches,cs_use_lineinfo);
                       end;
                     'm' :
                       begin
                         paratargetdbg:=dbg_codeview;
                       end;
                     'o' :
                       begin
                         if not UpdateDebugStr(copy(more,j+1,length(more)),init_settings.debugswitches) then
                           IllegalPara(opt);
                         break;
                       end;
                     'p' :
                       begin
                         if UnsetBool(More, j, opt, false) then
                           exclude(init_settings.globalswitches,cs_stabs_preservecase)
                         else
                           include(init_settings.globalswitches,cs_stabs_preservecase);
                       end;
                     's' :
                       begin
                         paratargetdbg:=dbg_stabs;
                       end;
                     't' :
                       begin
                         if UnsetBool(More, j, opt, false) then
                            localvartrashing := -1
                         else
                           localvartrashing := (localvartrashing + 1) mod nroftrashvalues;
                       end;
                     'v' :
                       begin
                         if UnsetBool(More, j, opt, false) then
                           exclude(init_settings.globalswitches,cs_gdb_valgrind)
                         else
                           include(init_settings.globalswitches,cs_gdb_valgrind);
                       end;
                     'w' :
                       begin
                         if (j<length(more)) and (more[j+1] in ['2','3','4']) then
                           begin
                             case more[j+1] of
                               '2': paratargetdbg:=dbg_dwarf2;
                               '3': paratargetdbg:=dbg_dwarf3;
                               '4': paratargetdbg:=dbg_dwarf4;
                             end;
                             inc(j);
                           end
                         else
                           paratargetdbg:=dbg_dwarf2;
                       end;
                     else
                       IllegalPara(opt);
                   end;
                   inc(j);
                 end;
             end;

           'h' :
             begin
               NoPressEnter:=true;
               if (More <> '') and (More [1] = 'F') then
                 begin
                   FPCHelpLines := true;
                   Delete (More, 1, 1);
                   FPCBinaryPath := More;
                 end;
               WriteHelpPages;
             end;

           'i' :
             begin
               if (More='') or
                    (More [1] in ['a', 'c', 'f', 'i', 'o', 'r', 't', 'u', 'w']) then
                 WriteInfo (More)
               else
                 QuickInfo:=QuickInfo+More;
             end;

           'I' :
             begin
               if ispara then
                 ParaIncludePath.AddPath(More,false)
               else
                includesearchpath.AddPath(More,false);
             end;

           'k' :
             begin
               if more<>'' then
                 ParaLinkOptions:=ParaLinkOptions+' '+More
               else
                 IllegalPara(opt);
             end;

           'l' :
             ParaLogo:=not UnSetBool(more,0,opt,true);

{$ifdef PREPROCWRITE}
           'm' :
             parapreprocess:=not UnSetBool(more,0,opt,true);
{$endif PREPROCWRITE}

           'M' :
             begin
               more:=Upper(more);
               if not SetCompileMode(more, true) then
                 if not SetCompileModeSwitch(more, true) then
                   IllegalPara(opt);
             end;

           'n' :
             begin
               if More='' then
                 disable_configfile:=true
               else
                 IllegalPara(opt);
             end;

           'o' :
             begin
               if More<>'' then
                 begin
                   DefaultReplacements(More);
                   D:=ExtractFilePath(More);
                   if (D<>'') then
                     OutputExeDir:=FixPath(D,True);
                   OutputFileName:=ExtractFileName(More);
                 end
               else
                 IllegalPara(opt);
             end;

           'O' :
             begin
               j:=1;
               while j<=length(more) do
                begin
                  case more[j] of
                    '1' :
                      init_settings.optimizerswitches:=init_settings.optimizerswitches+level1optimizerswitches;
                    '2' :
                      init_settings.optimizerswitches:=init_settings.optimizerswitches+level2optimizerswitches;
                    '3' :
                      init_settings.optimizerswitches:=init_settings.optimizerswitches+level3optimizerswitches;
                    '4' :
                      init_settings.optimizerswitches:=init_settings.optimizerswitches+level4optimizerswitches;
                    'a' :
                      begin
                        if not(UpdateAlignmentStr(Copy(Opt,j+3,255),ParaAlignment)) then
                          IllegalPara(opt);
                        break;
                      end;
                    's' :
                      include(init_settings.optimizerswitches,cs_opt_size);
                    'p' :
                      begin
                        if not Setoptimizecputype(copy(more,j+1,length(more)),init_settings.optimizecputype) then
                          begin
                            OptCPUSetExplicitly:=true;
                            { Give warning for old i386 switches }
                            if (Length(More)-j=1) and
                               (More[j+1]>='1') and (More[j+1]<='5')then
                              Message2(option_obsolete_switch_use_new,'-Op<nr>','-Op<name>')
                            else
                              IllegalPara(opt);
                          end;
                        break;
                      end;
                    'o' :
                      begin
                        if not UpdateOptimizerStr(copy(more,j+1,length(more)),init_settings.optimizerswitches) then
                         IllegalPara(opt);
                        break;
                      end;
                    '-' :
                      begin
                        init_settings.optimizerswitches:=[];
                        FillChar(ParaAlignment,sizeof(ParaAlignment),0);
                      end;
                    { Obsolete switches }
                    'g' :
                      Message2(option_obsolete_switch_use_new,'-Og','-Os');
                    'G' :
                      Message1(option_obsolete_switch,'-OG');
                    'r' :
                      Message2(option_obsolete_switch_use_new,'-Or','-O2 or -Ooregvar');
                    'u' :
                      Message2(option_obsolete_switch_use_new,'-Ou','-Oouncertain');
                    'w' :
                      begin
                        if not UpdateWpoStr(copy(more,j+1,length(more)),init_settings.dowpoptimizerswitches) then
                          IllegalPara(opt);
                        break;
                      end;
                    'W' :
                      begin
                        if not UpdateWpoStr(copy(more,j+1,length(more)),init_settings.genwpoptimizerswitches) then
                          IllegalPara(opt);
                        break;
                      end;
                    else
                      IllegalPara(opt);
                  end;
                  inc(j);
                end;
             end;

           'p' :
             begin
               if UnsetBool(More, 0, opt, false) then
                 begin
                   init_settings.moduleswitches:=init_settings.moduleswitches-[cs_profile];
                   undef_system_macro('FPC_PROFILE');
                 end
               else
                 if Length(More)=0 then
                   IllegalPara(opt)
                 else
                 case more[1] of
                  'g' : if UnsetBool(more, 1, opt, false) then
                         begin
                           exclude(init_settings.moduleswitches,cs_profile);
                           undef_system_macro('FPC_PROFILE');
                         end
                        else if (target_info.system in supported_targets_pg) then
                         begin
                           include(init_settings.moduleswitches,cs_profile);
                           def_system_macro('FPC_PROFILE');
                         end
                        else
                          UnsupportedPara('-pg');
                 else
                   IllegalPara(opt);
                 end;
             end;

           'P' :
             begin
               { used to select the target processor with the "fpc" binary;
                 give an error if it's not the target architecture supported by
                 this compiler binary (will be verified after the target_info
                 is set) }
               processorstr:=More;
             end;

           'R' :
             begin
               if not SetAsmReadMode(More,init_settings.asmmode) then
                 IllegalPara(opt);
             end;

           's' :
             begin
               if UnsetBool(More, 0, opt, false) then
                 begin
                   init_settings.globalswitches:=init_settings.globalswitches-[cs_asm_extern,cs_link_extern,cs_link_nolink];
                   if more<>'' then
                     IllegalPara(opt);
                 end
               else
                 begin
                   init_settings.globalswitches:=init_settings.globalswitches+[cs_asm_extern,cs_link_extern,cs_link_nolink];
                   if more='h' then
                     init_settings.globalswitches:=init_settings.globalswitches-[cs_link_on_target]
                   else if more='t' then
                     init_settings.globalswitches:=init_settings.globalswitches+[cs_link_on_target]
                   else if more='r' then
                     init_settings.globalswitches:=init_settings.globalswitches+[cs_asm_leave,cs_no_regalloc]
                   else if more<>'' then
                     IllegalPara(opt);
                 end;
             end;

           'S' :
             begin
               if more='' then
                 IllegalPara(opt);
               if more[1]='I' then
                 begin
{$ifdef jvm}
                   UnsupportedPara('-SI');
{$endif}
                   if upper(more)='ICOM' then
                     init_settings.interfacetype:=it_interfacecom
                   else if upper(more)='ICORBA' then
                     init_settings.interfacetype:=it_interfacecorba
                   else
                     IllegalPara(opt);
                 end
               else
                begin
                  j:=1;
                  while j<=length(more) do
                   begin
                     case more[j] of
                       '2' : //an alternative to -Mobjfpc
                         SetCompileMode('OBJFPC',true);
                       'a' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.localswitches,cs_do_assertion)
                         else
                           include(init_settings.localswitches,cs_do_assertion);
                       'c' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.moduleswitches,cs_support_c_operators)
                         else
                           include(init_settings.moduleswitches,cs_support_c_operators);
                       'C':
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.localswitches,cs_check_all_case_coverage)
                         else
                           include(init_settings.localswitches,cs_check_all_case_coverage);
                       'd' : //an alternative to -Mdelphi
                         SetCompileMode('DELPHI',true);
                       'e' :
                         begin
                           SetErrorFlags(copy(more,j+1,length(more)));
                           break;
                         end;
                       'f' :
                         begin
                           if not(cs_compilesystem in init_settings.moduleswitches) then
                             Message(option_features_only_for_system_unit);
                           inc(j);
                           if more[j]='-' then
                             begin
                               if length(more)>j then
                                 IllegalPara(opt)
                               else
                                 features:=[];
                             end
                           else
                             begin
                               if (HandleFeature(upper(copy(more,j,length(more)-j+1)))) then
                                 j:=length(more)
                               else
                                 IllegalPara(opt);
                             end;
                         end;
                       'g' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.moduleswitches,cs_support_goto)
                         else
                           include(init_settings.moduleswitches,cs_support_goto);
                       'h' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.localswitches,cs_refcountedstrings)
                         else
                           include(init_settings.localswitches,cs_refcountedstrings);
                       'i' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.localswitches,cs_do_inline)
                         else
                           include(init_settings.localswitches,cs_do_inline);
                       'j' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.localswitches,cs_typed_const_writable)
                         else
                           include(init_settings.localswitches,cs_typed_const_writable);
                       'k' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.globalswitches,cs_load_fpcylix_unit)
                         else
                           include(init_settings.globalswitches,cs_load_fpcylix_unit);
                       'm' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.moduleswitches,cs_support_macro)
                         else
                           include(init_settings.moduleswitches,cs_support_macro);
                       'o' : //an alternative to -Mtp
                         SetCompileMode('TP',true);
                       'r' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.globalswitches,cs_transparent_file_names)
                         else
                           include(init_settings.globalswitches,cs_transparent_file_names);
{$ifdef gpc_mode}
                       'p' : //an alternative to -Mgpc
                         SetCompileMode('GPC',true);
{$endif}
                       's' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.globalswitches,cs_constructor_name)
                         else
                           include(init_settings.globalswitches,cs_constructor_name);
                       't' :
                         Message1(option_obsolete_switch,'-St');
                       'v' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.globalswitches,cs_support_vectors)
                         else
                           include(init_settings.globalswitches,cs_support_vectors);
                       'x' :
                         If UnsetBool(More, j, opt, false) then
                           SetCompileModeSwitch('EXCEPTIONS-',true)
                         else
                           SetCompileModeSwitch('EXCEPTIONS',true);
                       'y' :
                         If UnsetBool(More, j, opt, false) then
                           exclude(init_settings.localswitches,cs_typed_addresses)
                         else
                           include(init_settings.localswitches,cs_typed_addresses);
                       '-' :
                         begin
                           init_settings.globalswitches:=init_settings.globalswitches - [cs_constructor_name,cs_support_exceptions,
                                                                                         cs_support_vectors,cs_load_fpcylix_unit];

                           init_settings.localswitches:=init_settings.localswitches - [cs_do_assertion,cs_do_inline, cs_refcountedstrings,
                                                                                       cs_typed_addresses];

                           init_settings.moduleswitches:=init_settings.moduleswitches - [cs_support_c_operators, cs_support_goto,
                                                                                         cs_support_macro];
                         end;
                       else
                         IllegalPara(opt);
                     end;
                     inc(j);
                   end;
                end;
             end;

           'T' :
             begin
               more:=Upper(More);
               if paratarget=system_none then
                begin
                  { remove old target define }
                  TargetOptions(false);
                  { load new target }
                  paratarget:=find_system_by_string(More);
                  if paratarget<>system_none then
                    set_target(paratarget)
                  else
                    IllegalPara(opt);
                  { set new define }
                  TargetOptions(true);
                end
               else
                if More<>upper(target_info.shortname) then
                 Message1(option_target_is_already_set,target_info.shortname);
             end;

           'u' :
             if is_identifier(more) then
               undef_system_macro(more)
             else
               begin
                 if (more='') then
                   Message1(option_missing_arg,'-u')
                 else
                   Message1(option_malformed_para,opt);
                 StopOptions(1);
               end;
           'U' :
             begin
               j:=1;
               while j<=length(more) do
                begin
                  case more[j] of
{$ifdef UNITALIASES}
                    'a' :
                       begin
                         AddUnitAlias(Copy(More,j+1,255));
                         break;
                       end;
{$endif UNITALIASES}
                    'n' :
                      exclude(init_settings.globalswitches,cs_check_unit_name);
                    'p' :
                       begin
                         Message2(option_obsolete_switch_use_new,'-Up','-Fu');
                         break;
                       end;
                    'r' :
                      begin
                        do_release:=true;
                        if (cs_checkpointer in init_settings.localswitches) then
                          begin
                            Message(option_gc_incompatible_with_release_flag);
                            exclude(init_settings.localswitches,cs_checkpointer);
                          end;
                      end;
                    's' :
                      include(init_settings.moduleswitches,cs_compilesystem);
                    '-' :
                      begin
                        exclude(init_settings.moduleswitches,cs_compilesystem);
                        exclude(init_settings.globalswitches,cs_check_unit_name);
                      end;
                    else
                      IllegalPara(opt);
                  end;
                  inc(j);
                end;
             end;

           'v' :
             begin
               if not setverbosity(More) then
                 IllegalPara(opt);
             end;

           'V' : ; { Ignore used by fpc }

           'W' :
             begin
               j:=1;
               while j<=length(More) do
                begin
                  case More[j] of
                    'A':
                      begin
                        if target_info.system in systems_all_windows then
                          begin
                            if UnsetBool(More, j, opt, false) then
                              SetApptype(app_cui)
                            else
                              SetApptype(app_native);
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'b':
                      begin
                        if target_info.system in systems_darwin then
                          begin
                            if UnsetBool(More, j, opt, false) then
                              SetApptype(app_cui)
                            else
                              SetApptype(app_bundle)
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'B':
                      begin
                        if target_info.system in systems_all_windows+systems_symbian then
                          begin
                            {  -WB200000 means set trefered base address
                              to $200000, but does not change relocsection boolean
                              this way we can create both relocatble and
                              non relocatable DLL at a specific base address PM }
                            if (length(More)>j) then
                              begin
                                val('$'+Copy(More,j+1,255),imagebase,code);
                                if code<>0 then
                                  IllegalPara(opt);
                                ImageBaseSetExplicity:=true;
                              end
                            else
                              begin
                                RelocSection:=true;
                                RelocSectionSetExplicitly:=true;
                              end;
                            break;
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'C':
                      begin
                        if target_info.system in systems_all_windows+systems_os2+systems_macos then
                          begin
                            if UnsetBool(More, j, opt, false) then
                              SetApptype(app_gui)
                            else
                              SetApptype(app_cui);
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'D':
                      begin
                        if target_info.system in systems_all_windows then
                          begin
                            UseDeffileForExports:=not UnsetBool(More, j, opt, false);
                            UseDeffileForExportsSetExplicitly:=true;
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'e':
                      begin
                        if (target_info.system in systems_darwin) then
                          begin
                            RegisterRes(res_macosx_ext_info,TWinLikeResourceFile);
                            set_target_res(res_ext);
                            target_info.resobjext:='.fpcres';
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'F':
                      begin
                        if target_info.system in systems_os2 then
                          begin
                            if UnsetBool(More, j, opt, false) then
                              SetApptype(app_cui)
                            else
                              SetApptype(app_fs);
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'G':
                      begin
                        if target_info.system in systems_all_windows+systems_os2+systems_macos then
                          begin
                            if UnsetBool(More, j, opt, false) then
                              SetApptype(app_cui)
                            else
                              SetApptype(app_gui);
                          end
                        else
                          IllegalPara(opt);
                      end;
{$if defined(i8086)}
                    'h':
                      begin
                        if UnsetBool(More, j, opt, false) then
                          exclude(init_settings.moduleswitches,cs_huge_code)
                         else
                          include(init_settings.moduleswitches,cs_huge_code);
                      end;
{$endif defined(i8086)}
                    'I':
                      begin
                        if target_info.system in systems_all_windows then
                          begin
                            GenerateImportSection:=not UnsetBool(More,j,opt,false);
                            GenerateImportSectionSetExplicitly:=true;
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'i':
                      begin
                        if (target_info.system in systems_darwin) then
                          begin
                            set_target_res(res_macho);
                            target_info.resobjext:=
                              targetinfos[target_info.system]^.resobjext;
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'm':
                      begin
{$if defined(i8086)}
                        if (target_info.system in [system_i8086_msdos,system_i8086_win16,system_i8086_embedded]) then
                          begin
                            case Upper(Copy(More,j+1,255)) of
                              'TINY':    init_settings.x86memorymodel:=mm_tiny;
                              'SMALL':   init_settings.x86memorymodel:=mm_small;
                              'MEDIUM':  init_settings.x86memorymodel:=mm_medium;
                              'COMPACT': init_settings.x86memorymodel:=mm_compact;
                              'LARGE':   init_settings.x86memorymodel:=mm_large;
                              'HUGE':    init_settings.x86memorymodel:=mm_huge;
                              else
                                IllegalPara(opt);
                            end;
                            break;
                          end
                        else
{$endif defined(i8086)}
                          IllegalPara(opt);
                      end;
                    'M':
                      begin
                        if (target_info.system in (systems_darwin-[system_i386_iphonesim,system_arm_darwin,system_aarch64_darwin,system_x86_64_iphonesim])) and
                           ParseMacVersionMin(MacOSXVersionMin,iPhoneOSVersionMin,'MAC_OS_X_VERSION_MIN_REQUIRED',copy(More,2,255),false) then
                          begin
                            break;
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'N':
                      begin
                        if target_info.system in systems_all_windows then
                          begin
                            RelocSection:=UnsetBool(More,j,opt,false);
                            RelocSectionSetExplicitly:=true;
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'p':
                      begin
{$push}
{$warn 6018 off} { Unreachable code due to compile time evaluation }
                        if (target_info.system in systems_embedded) and
                          ControllerSupport then
                          begin
                            s:=upper(copy(more,j+1,length(more)-j));
                            if not(SetControllerType(s,init_settings.controllertype)) then
                              IllegalPara(opt)
                            else
                              begin
                                if init_settings.cputype<>embedded_controllers[init_settings.controllertype].cputype then
                                begin
                                  Message(scan_n_changecputype);
                                  init_settings.cputype:=embedded_controllers[init_settings.controllertype].cputype;
                                end;
                              end;
                            break;
                          end
                        else
                          IllegalPara(opt);
{$pop}
                      end;
                    'P':
                      begin
                        if (target_info.system in [system_i386_iphonesim,system_arm_darwin,system_aarch64_darwin,system_x86_64_iphonesim]) and
                           ParseMacVersionMin(iPhoneOSVersionMin,MacOSXVersionMin,'IPHONE_OS_VERSION_MIN_REQUIRED',copy(More,2,255),true) then
                          begin
                            break;
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'R':
                      begin
                        if target_info.system in systems_all_windows then
                          begin
                            { support -WR+ / -WR- as synonyms to -WR / -WN }
                            RelocSection:=not UnsetBool(More,j,opt,false);
                            RelocSectionSetExplicitly:=true;
                          end
                        else
                          IllegalPara(opt);
                      end;
                    't':
                      begin
{$if defined(i8086)}
                        if (target_info.system in [system_i8086_msdos,system_i8086_embedded]) then
                          begin
                            case Upper(Copy(More,j+1,255)) of
                              'EXE': SetAppType(app_cui);
                              'COM': SetAppType(app_com);
                              else
                                IllegalPara(opt);
                            end;
                            break;
                          end
                        else
{$endif defined(i8086)}
                          IllegalPara(opt);
                      end;
                    'T':
                      begin
                        if target_info.system in systems_macos then
                          begin
                            if UnsetBool(More, j, opt, false) then
                              SetApptype(app_cui)
                            else
                              SetApptype(app_tool);
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'X':
                      begin
                        if (target_info.system in systems_linux) then
                          begin
                            if UnsetBool(More, j, opt, false) then
                              exclude(init_settings.moduleswitches,cs_executable_stack)
                            else
                              include(init_settings.moduleswitches,cs_executable_stack)
                          end
                        else
                          IllegalPara(opt);
                      end;
                    else
                      IllegalPara(opt);
                  end;
                  inc(j);
                end;
             end;

           'X' :
             begin
               j:=1;
               while j<=length(more) do
                begin
                  case More[j] of
                    '9' :
                      begin
                        if target_info.system in systems_linux then
                          begin
                            if UnsetBool(More, j, opt, false) then
                              exclude(init_settings.globalswitches,cs_link_pre_binutils_2_19)
                            else
                              include(init_settings.globalswitches,cs_link_pre_binutils_2_19);
                          end
                        else
                          IllegalPara(opt);
                      end;
                    'c' : Cshared:=TRUE;
                    'd' : Dontlinkstdlibpath:=TRUE;
                    'e' :
                      begin
                        If UnsetBool(More, j, opt, false) then
                          exclude(init_settings.globalswitches,cs_link_extern)
                        else
                          include(init_settings.globalswitches,cs_link_extern);
                      end;
                    'f' :
                      include(init_settings.globalswitches,cs_link_pthread);
                    'g' :
                      begin
                        If UnsetBool(More, j, opt, false) then
                          exclude(init_settings.globalswitches,cs_link_separate_dbg_file)
                        else
                          include(init_settings.globalswitches,cs_link_separate_dbg_file);
                      end;
                    'i' :
                      begin
                        If UnsetBool(More, j, opt, false) then
                          include(init_settings.globalswitches,cs_link_extern)
                        else
                          exclude(init_settings.globalswitches,cs_link_extern);
                      end;
                    'n' :
                      begin
                        If UnsetBool(More, j, opt, false) then
                          exclude(init_settings.globalswitches,cs_link_native)
                        else
                          include(init_settings.globalswitches,cs_link_native);
                      end;

                    'm' :
                      begin
                        If UnsetBool(More, j, opt, false) then
                          exclude(init_settings.globalswitches,cs_link_map)
                        else
                          include(init_settings.globalswitches,cs_link_map);
                      end;
                    'p' : ; { Ignore used by fpc.pp }
                    'r' :
                      begin
                        if (target_info.system in suppported_targets_x_smallr) then
                          begin
                            rlinkpath:=Copy(more,2,length(More)-1);
                            DefaultReplacements(rlinkpath);
                          end
                        else
                          IgnoredPara('-Xr');
                        more:='';
                      end;
                    'R' :
                      begin
                        sysrootpath:=copy(more,2,length(more)-1);
                        defaultreplacements(sysrootpath);
                        more:='';
                      end;
                    's' :
                      begin
                        If UnsetBool(More, j, opt, false) then
                          exclude(init_settings.globalswitches,cs_link_strip)
                        else
                          include(init_settings.globalswitches,cs_link_strip);
                      end;
                    't' :
                      include(init_settings.globalswitches,cs_link_staticflag);
                    'v' :
                      begin
                        If UnsetBool(More, j, opt, false) then
                          exclude(init_settings.globalswitches,cs_link_opt_vtable)
                        else
                          include(init_settings.globalswitches,cs_link_opt_vtable);
                      end;
                    'D' :
                      begin
                        def_system_macro('FPC_LINK_DYNAMIC');
                        undef_system_macro('FPC_LINK_SMART');
                        undef_system_macro('FPC_LINK_STATIC');
                        exclude(init_settings.globalswitches,cs_link_static);
                        exclude(init_settings.globalswitches,cs_link_smart);
                        include(init_settings.globalswitches,cs_link_shared);
                        LinkTypeSetExplicitly:=true;
                      end;
                    'M' :
                      begin
                        mainaliasname:=Copy(more,2,length(More)-1);
                        More:='';
                      end;
                    'P' :
                      begin
                        utilsprefix:=Copy(more,2,length(More)-1);
                        DefaultReplacements(utilsprefix);
                        More:='';
                      end;
                    'L' : begin  // -XLO is link order -XLA is link alias. -XLD avoids load defaults.
                                 // these are not aggregable.
                            if (j=length(more)) or not (more[j+1] in ['O','A','D']) then
                              IllegalPara(opt)
                            else
                              begin
                                case more[j+1] of
                                 'A' : begin
                                        s:=Copy(more,3,length(More)-2);
                                        if not LinkLibraryAliases.AddDep(s) Then
                                           IllegalPara(opt);
                                       end;
                                 'O' : begin
                                        s:=Copy(more,3,length(More)-2);
                                        if not LinkLibraryOrder.AddWeight(s) Then
                                           IllegalPara(opt);
                                       end;
                                 'D' : include(init_settings.globalswitches,cs_link_no_default_lib_order)
                                else
                                  IllegalPara(opt);
                                 end; {case}
                                j:=length(more);
                              end; {else begin}
                          end;
                    'S' :
                      begin
                        ForceStaticLinking;
                      end;
                    'V' :
                      begin
                        if UnsetBool(More, j, opt, false) then
                          exclude(init_settings.globalswitches,cs_link_vlink)
                        else
                          begin
                            include(init_settings.globalswitches,cs_link_vlink);
                            include(init_settings.globalswitches,cs_link_extern);
                          end;
                        LinkerSetExplicitly:=true;
                      end;
                    'X' :
                      begin
                        def_system_macro('FPC_LINK_SMART');
                        undef_system_macro('FPC_LINK_STATIC');
                        undef_system_macro('FPC_LINK_DYNAMIC');
                        exclude(init_settings.globalswitches,cs_link_static);
                        include(init_settings.globalswitches,cs_link_smart);
                        exclude(init_settings.globalswitches,cs_link_shared);
                        LinkTypeSetExplicitly:=true;
                      end;
                    '-' :
                      begin
                        exclude(init_settings.globalswitches,cs_link_staticflag);
                        exclude(init_settings.globalswitches,cs_link_strip);
                        exclude(init_settings.globalswitches,cs_link_map);
                        set_default_link_type;
                      end;
                    else
                      IllegalPara(opt);
                  end;
                  inc(j);
                end;
             end;
           else
             IllegalPara(opt);
         end;
       end;

    '@' :
      begin
        Message(option_no_nested_response_file);
        StopOptions(1);
      end;

    else
      begin
        if (length(param_file)<>0) then
          Message2(option_only_one_source_support,param_file,opt);
        param_file:=opt;
        Message1(option_found_file,opt);
      end;
  end;
end;


procedure Toption.Interpret_file(const filename : TPathStr);

  procedure RemoveSep(var fn:TPathStr);
  var
    i : longint;
  begin
    i:=0;
    while (i<length(fn)) and (fn[i+1] in [',',' ',#9]) do
      inc(i);
    Delete(fn,1,i);
    i:=length(fn);
    while (i>0) and (fn[i] in [',',' ',#9]) do
      dec(i);
    fn:=copy(fn,1,i);
  end;

  function GetName(var fn:TPathStr):TPathStr;
  var
    i : longint;
  begin
    i:=0;
    while (i<length(fn)) and (fn[i+1] in ['a'..'z','A'..'Z','0'..'9','_','-']) do
     inc(i);
    GetName:=Copy(fn,1,i);
    Delete(fn,1,i);
  end;

const
  maxlevel = 15;
var
  f     : text;
  s, tmp,
  opts  : TCmdStr;
  skip  : array[0..maxlevel] of boolean;
  line,
  level : longint;
  option_read : boolean;
  oldfilemode : byte;
  ConfigFile: TPathStr;
begin
{ avoid infinite loop }
  Inc(FileLevel);
  Option_read:=false;
  If FileLevel>MaxLevel then
   Message(option_too_many_cfg_files);
  if not ParaIncludeCfgPath.FindFile(fileName,true,ConfigFile) then
    ConfigFile := ExpandFileName(filename);
{ Maybe It's Directory ?}   //Jaro Change:
  if PathExists(ConfigFile,false) then
    begin
       Message1(option_config_is_dir,filename);
       exit;
    end;
{ open file }
  Message1(option_using_file,filename);
  oldfilemode:=filemode;
  filemode:=0;
  assign(f,ConfigFile);
  {$push}{$I-}
   reset(f);
  {$pop}
  filemode:=oldfilemode;
  if ioresult<>0 then
   begin
     Message1(option_unable_open_file,filename);
     exit;
   end;
  Message1(option_start_reading_configfile,filename);
  fillchar(skip,sizeof(skip),0);
  level:=0;
  line:=0;
  while not eof(f) do
   begin
     readln(f,opts);
     inc(line);
     RemoveSep(opts);
     if (opts<>'') and (opts[1]<>';') then
      begin
        if opts[1]='#' then
         begin
           Message1(option_interpreting_file_option,opts);
           Delete(opts,1,1);
           s:=upper(GetName(opts));
           if (s='SECTION') then
            begin
              RemoveSep(opts);
              s:=upper(GetName(opts));
              if level=0 then
               skip[level]:=not defined_macro(s) or (s='COMMON');
            end
           else
            if (s='IFDEF') then
             begin
               RemoveSep(opts);
               if Level>=maxlevel then
                begin
                  Message2(option_too_many_ifdef,filename,tostr(line));
                  stopOptions(1);
                end;
               inc(Level);
               skip[level]:=(skip[level-1] or not defined_macro(upper(GetName(opts))));
             end
           else
            if (s='IFNDEF') then
             begin
               RemoveSep(opts);
               if Level>=maxlevel then
                begin
                  Message2(option_too_many_ifdef,filename,tostr(line));
                  stopOptions(1);
                end;
               inc(Level);
               skip[level]:=(skip[level-1] or defined_macro(upper(GetName(opts))));
             end
           else
            if (s='ELSE') then
              begin
                if Level=0 then
                  begin
                    Message2(option_else_without_if,filename,tostr(line));
                    stopOptions(1);
                  end
                else
                  skip[level]:=skip[level-1] or (not skip[level])
              end
           else
            if (s='ENDIF') then
             begin
               skip[level]:=false;
               if Level=0 then
                begin
                  Message2(option_too_many_endif,filename,tostr(line));
                  stopOptions(1);
                end;
               dec(level);
             end
           else
            if (not skip[level]) then
             begin
               if (s='DEFINE') then
                begin
                  RemoveSep(opts);
                  tmp:= GetName(opts);
                  if tmp <> '' then
                    def_system_macro(tmp);
                  Option_read:=true;
                end
              else
               if (s='UNDEF') then
                begin
                  RemoveSep(opts);
                  tmp:= GetName(opts);
                  if tmp <> '' then
                    undef_system_macro(tmp);
                  Option_read:=true;
                end
              else
               if (s='WRITE') then
                begin
                  Delete(opts,1,1);
                  DefaultReplacements(opts);
                  WriteLn(opts);
                  Option_read:=true;
                end
              else
               if (s='INCLUDE') then
                begin
                  Delete(opts,1,1);
                  DefaultReplacements(opts);
                  Interpret_file(opts);
                  Option_read:=true;
                end
              else
               if (s='CFGDIR') then
                begin
                  Delete(opts,1,1);
                  DefaultReplacements(opts);
                  ParaIncludeCfgPath.AddPath(opts,false);
                  Option_read:=true;
                end;
            end;
         end
        else
         begin
           if (opts[1]='-') or (opts[1]='@') then
            begin
              if (not skip[level]) then
                interpret_option(opts,false);
              Option_read:=true;
            end
           else
             Message1(option_illegal_para,opts);
         end;
      end;
   end;
  if Level>0 then
   Message(option_too_less_endif);
  if Not Option_read then
    Message1(option_no_option_found,filename)
  else
    Message1(option_end_reading_configfile,filename);
  Close(f);
  Dec(FileLevel);
end;


procedure Toption.Interpret_envvar(const envname : TCmdStr);
var
  argstart,
  env,
  pc     : pchar;
  arglen : longint;
  quote  : set of char;
  hs     : TCmdStr;
begin
  Message1(option_using_env,envname);
  env:=GetEnvPChar(envname);
  pc:=env;
  hs:='';
  if assigned(pc) then
   begin
     repeat
       { skip leading spaces }
       while pc^ in [' ',#9,#13] do
        inc(pc);
       case pc^ of
         #0 :
           break;
         '"' :
           begin
             quote:=['"'];
             inc(pc);
           end;
         '''' :
           begin
              quote:=[''''];
              inc(pc);
           end;
         else
           quote:=[' ',#9,#13];
       end;
     { scan until the end of the argument }
       argstart:=pc;
       while (pc^<>#0) and not(pc^ in quote) do
        inc(pc);
     { create argument }
       arglen:=pc-argstart;
{ TODO: FIXME: silent truncation of environment parameters }
       if (arglen > 255) then
         arglen := 255;
       setlength(hs,arglen);
       move(argstart^,hs[1],arglen);
       interpret_option(hs,true);
     { skip quote }
       if pc^ in quote then
        inc(pc);
     until false;
   end
  else
   Message1(option_no_option_found,'(env) '+envname);
  FreeEnvPChar(env);
end;


procedure toption.read_parameters;
var
  opts       : TCmdStr;
  paramindex : longint;
begin
  paramindex:=0;
  while paramindex<paramcount do
   begin
     inc(paramindex);
     opts:=objpas.paramstr(paramindex);
     if length(opts)>0 then
       case opts[1] of
         '@' :
           if not firstpass then
           begin
             Delete(opts,1,1);
             Message1(option_reading_further_from,opts);
             interpret_file(opts);
           end;
         '!' :
           if not firstpass then
           begin
             Delete(opts,1,1);
             Message1(option_reading_further_from,'(env) '+opts);
             interpret_envvar(opts);
           end;
         else
           interpret_option(opts,true);
       end;
   end;
end;


procedure toption.parsecmd(cmd:TCmdStr);
var
  i,ps  : longint;
  opts  : TCmdStr;
begin
  while (cmd<>'') do
   begin
     while cmd[1]=' ' do
      delete(cmd,1,1);
     i:=pos(' ',cmd);
     if i=0 then
       i:=2147483647;
     opts:=Copy(cmd,1,i-1);
     Delete(cmd,1,i);
     case opts[1] of
       '@' :
         if not firstpass then
         begin
           Delete(opts,1,1);
           Message1(option_reading_further_from,opts);
           interpret_file(opts);
         end;
       '!' :
         if not firstpass then
         begin
           Delete(opts,1,1);
           Message1(option_reading_further_from,'(env) '+opts);
           interpret_envvar(opts);
         end;
       '"' :
         begin
           Delete(opts,1,1);
           ps:=pos('"',cmd);
           if (i<>256) and (ps>0) then
             begin
               opts:=opts + ' '+ copy(cmd,1,ps-1);
               cmd:=copy(cmd,ps+1,255);
             end;
           interpret_option(opts,true);
         end;
       else
         interpret_option(opts,true);
     end;
   end;
end;


procedure toption.writequickinfo;
var
  s : string;
  i : longint;

  procedure addinfo(const hs:string);
  begin
    if s<>'' then
     s:=s+' '+hs
    else
     s:=hs;
  end;

begin
  s:='';
  i:=0;
  while (i<length(quickinfo)) do
   begin
     inc(i);
     case quickinfo[i] of
      'S' :
        begin
          inc(i);
          case quickinfo[i] of
           'O' :
             addinfo(lower(source_info.shortname));
           'P' :
             addinfo(source_cpu_string);
           else
             IllegalPara('-i'+QuickInfo);
          end;
        end;
      'T' :
        begin
          inc(i);
          case quickinfo[i] of
           'O' :
             addinfo(lower(target_info.shortname));
           'P' :
             AddInfo(target_cpu_string);
           else
             IllegalPara('-i'+QuickInfo);
          end;
        end;
      'V' :
        AddInfo(version_string);
      'W' :
        AddInfo(full_version_string);
      'D' :
        AddInfo(date_string);
      '_' :
        ;
      else
        IllegalPara('-i'+QuickInfo);
    end;
  end;
  if s<>'' then
   begin
     writeln(s);
     stopoptions(0);
   end;
end;


procedure TOption.TargetOptions(def:boolean);
var
  s : string;
  i : integer;
  target_unsup_features : tfeatures;
begin
  if def then
   def_system_macro(target_info.shortname)
  else
   undef_system_macro(target_info.shortname);

  s:=target_info.extradefines;
  while (s<>'') do
   begin
     i:=pos(';',s);
     if i=0 then
      i:=length(s)+1;
     if def then
      def_system_macro(Copy(s,1,i-1))
     else
      undef_system_macro(Copy(s,1,i-1));
     delete(s,1,i);
   end;

  if (tf_winlikewidestring in target_info.flags) then
    if def then
      def_system_macro('FPC_WINLIKEWIDESTRING')
    else
      undef_system_macro('FPC_WINLIKEWIDESTRING');

  if (tf_requires_proper_alignment in target_info.flags) then
    if def then
      def_system_macro('FPC_REQUIRES_PROPER_ALIGNMENT')
    else
      undef_system_macro('FPC_REQUIRES_PROPER_ALIGNMENT');

  if (tf_init_final_units_by_calls in target_info.flags) then
    if def then
      def_system_macro('FPC_INIT_FINAL_UNITS_BY_CALLS')
    else
      undef_system_macro('FPC_INIT_FINAL_UNITS_BY_CALLS');

  if source_info.system<>target_info.system then
    if def then
      def_system_macro('FPC_CROSSCOMPILING')
    else
      undef_system_macro('FPC_CROSSCOMPILING');

  if source_info.cpu<>target_info.cpu then
    if def then
      def_system_macro('FPC_CPUCROSSCOMPILING')
    else
      def_system_macro('FPC_CPUCROSSCOMPILING');

  if (tf_no_generic_stackcheck in target_info.flags) then
    if def then
      def_system_macro('FPC_NO_GENERIC_STACK_CHECK')
    else
      undef_system_macro('FPC_NO_GENERIC_STACK_CHECK');

  if (tf_section_threadvars in target_info.flags) then
    if def then
      def_system_macro('FPC_SECTION_THREADVARS')
    else
      undef_system_macro('FPC_SECTION_THREADVARS');

  if (tf_use_psabieh in target_info.flags) then
    if def then
      def_system_macro('FPC_USE_PSABIEH')
    else
      undef_system_macro('FPC_USE_PSABIEH');

  { Code generation flags }
  if (tf_pic_default in target_info.flags) then
    if def then
      include(init_settings.moduleswitches,cs_create_pic)
    else
      exclude(init_settings.moduleswitches,cs_create_pic);

  { Resources support }
  if (tf_has_winlike_resources in target_info.flags) then
    if def then
      def_system_macro('FPC_HAS_WINLIKERESOURCES')
    else
      undef_system_macro('FPC_HAS_WINLIKERESOURCES');

  { Features }
  case target_info.system of
    system_arm_gba:
      target_unsup_features:=[f_dynlibs];
    system_arm_nds:
      target_unsup_features:=[f_threading,f_commandargs,f_fileio,f_textio,f_consoleio,f_dynlibs];
    system_i386_nativent:
      // until these features are implemented, they are disabled in the compiler
      target_unsup_features:=[f_stackcheck];
    system_i8086_msdos:
      target_unsup_features:=[f_threading,f_dynlibs];
    system_i8086_win16:
      target_unsup_features:=[f_threading];
    system_jvm_java32,
    system_jvm_android32:
      target_unsup_features:=[f_heap,f_textio,f_consoleio,f_fileio,
         f_variants,f_objects,f_commandargs,
         f_processes,f_stackcheck,f_dynlibs,f_softfpu,f_objectivec1,f_resources];
    system_arm_palmos,
    system_m68k_palmos:
      target_unsup_features:=[f_threading];
    system_m68k_atari:
      target_unsup_features:=[f_threading];
    { classic amiga has dynamic libraries, but they cannot be integrated in the
      normal dynlibs infrastructure due to architectural differences, so therefore
      lets disable the feature. }
    system_m68k_amiga:
      target_unsup_features:=[f_dynlibs];
    else
      target_unsup_features:=[];
  end;
  if def then
    features:=features-target_unsup_features
  else
    features:=features+target_unsup_features;

{$if defined(atari) or defined(hasamiga)}
   { enable vlink as default linker on Atari, Amiga, and MorphOS, but not for cross compilers (for now) }
   if (target_info.system in [system_m68k_amiga,system_m68k_atari,
                              system_powerpc_amiga]) and
      not LinkerSetExplicitly then
     include(init_settings.globalswitches,cs_link_vlink);
{$endif}
end;

procedure TOption.checkoptionscompatibility;
begin
{$ifdef i8086}
  if (apptype=app_com) and (init_settings.x86memorymodel<>mm_tiny) then
    begin
      Message(option_com_files_require_tiny_model);
      StopOptions(1);
    end;
{$endif i8086}

{$ifndef i8086_link_intern_debuginfo}
  if (cs_debuginfo in init_settings.moduleswitches) and
     (target_info.system in [system_i8086_msdos,system_i8086_win16,system_i8086_embedded]) and
     not (cs_link_extern in init_settings.globalswitches) then
    begin
      Message(option_debug_info_requires_external_linker);
      include(init_settings.globalswitches,cs_link_extern);
    end;
{$endif i8086_link_intern_debuginfo}

  if (paratargetdbg in [dbg_dwarf2,dbg_dwarf3]) and
     not(target_info.system in (systems_darwin+[system_i8086_msdos,system_i8086_embedded])) then
    begin
      { smartlink creation does not yet work with DWARF
        debug info on most targets, but it works in internal assembler }
      if (cs_create_smart in init_settings.moduleswitches) and
         not (af_outputbinary in target_asm.flags) then
        begin
          Message(option_dwarf_smartlink_creation);
          exclude(init_settings.moduleswitches,cs_create_smart);
        end;

      { smart linking does not yet work with DWARF debug info on most targets }
      if (cs_link_smart in init_settings.globalswitches) then
        begin
          Message(option_dwarf_smart_linking);
          ForceStaticLinking;
        end;
    end;

  { external debug info is only supported for DWARF on darwin }
  if (target_info.system in systems_darwin) and
     (cs_link_separate_dbg_file in init_settings.globalswitches) and
     not(paratargetdbg in [dbg_dwarf2,dbg_dwarf3]) then
    begin
      Message(option_debug_external_unsupported);
      exclude(init_settings.globalswitches,cs_link_separate_dbg_file);
    end;
  { Also create a smartlinked version, on an assembler that
    does not support smartlink sections like nasm?
    This is not compatible with using internal linker. }
  if ((cs_link_smart in init_settings.globalswitches) or
      (cs_create_smart in init_settings.moduleswitches)) and
     (af_needar in target_asm.flags) and
     not (af_smartlink_sections in target_asm.flags) and
     not (cs_link_extern in init_settings.globalswitches) and
     (target_info.link<>ld_none) and
      not (cs_link_nolink in init_settings.globalswitches) then
    begin
      Message(option_smart_link_requires_external_linker);
      include(init_settings.globalswitches,cs_link_extern);
    end;
end;


constructor TOption.create;
begin
  LogoWritten:=false;
  NoPressEnter:=false;
  FirstPass:=false;
  ABISetExplicitly:=false;
  FPUSetExplicitly:=false;
  CPUSetExplicitly:=false;
  OptCPUSetExplicitly:=false;
  FileLevel:=0;
  Quickinfo:='';
  ParaIncludeCfgPath:=TSearchPathList.Create;
  ParaIncludePath:=TSearchPathList.Create;
  ParaObjectPath:=TSearchPathList.Create;
  ParaUnitPath:=TSearchPathList.Create;
  ParaLibraryPath:=TSearchPathList.Create;
  ParaFrameworkPath:=TSearchPathList.Create;
  parapackagepath:=TSearchPathList.Create;
  parapackages:=TFPHashObjectList.Create;
  paranamespaces:=TCmdStrList.Create;
  FillChar(ParaAlignment,sizeof(ParaAlignment),0);
  MacVersionSet:=false;
  paratarget:=system_none;
  paratargetasm:=as_none;
  paratargetdbg:=dbg_none;
  LinkTypeSetExplicitly:=false;
  LinkerSetExplicitly:=false;
end;


destructor TOption.destroy;
begin
  ParaIncludeCfgPath.Free;
  ParaIncludePath.Free;
  ParaObjectPath.Free;
  ParaUnitPath.Free;
  ParaLibraryPath.Free;
  ParaFrameworkPath.Free;
  parapackagepath.Free;
  ParaPackages.Free;
  paranamespaces.free;
end;


{****************************************************************************
                              Callable Routines
****************************************************************************}

function check_configfile(fn:string; var foundfn:string):boolean;

  function CfgFileExists(const fn:string):boolean;
  begin
    Comment(V_Tried,'Configfile search: '+fn);
    CfgFileExists:=FileExists(fn);
  end;

var
{$ifdef Unix}
  hs,
{$endif Unix}
  configpath : string;
begin
  foundfn:=fn;
  check_configfile:=true;
  { retrieve configpath }
  configpath:=FixPath(GetEnvironmentVariable('PPC_CONFIG_PATH'),false);
{$ifdef Unix}
  if configpath='' then
   configpath:=ExpandFileName(FixPath(exepath+'../etc/',false));
{$endif}
  {
    Order to read configuration file :
    try reading fpc.cfg in :
     1 - current dir
     2 - configpath
     3 - compiler path
  }
  if not FileExists(fn) then
   begin
{$ifdef Unix}
     hs:=GetEnvironmentVariable('HOME');
     if (hs<>'') and CfgFileExists(FixPath(hs,false)+'.'+fn) then
      foundfn:=FixPath(hs,false)+'.'+fn
     else
{$endif}
      if CfgFileExists(configpath+fn) then
       foundfn:=configpath+fn
     else
{$ifdef WINDOWS}
       if (GetEnvironmentVariable('USERPROFILE')<>'') and CfgFileExists(FixPath(GetEnvironmentVariable('USERPROFILE'),false)+fn) then
         foundfn:=FixPath(GetEnvironmentVariable('USERPROFILE'),false)+fn
     else
       if (GetEnvironmentVariable('ALLUSERSPROFILE')<>'') and CfgFileExists(FixPath(GetEnvironmentVariable('ALLUSERSPROFILE'),false)+fn) then
         foundfn:=FixPath(GetEnvironmentVariable('ALLUSERSPROFILE'),false)+fn
     else
{$endif WINDOWS}
{$ifndef Unix}
      if CfgFileExists(exepath+fn) then
       foundfn:=exepath+fn
     else
{$else}
      if CfgFileExists('/etc/'+fn) then
       foundfn:='/etc/'+fn
     else
{$endif}
      check_configfile:=false;
   end;
end;

procedure read_arguments(cmd:TCmdStr);

  procedure def_cpu_macros;
    var
      abi : tabi;
      fputype : tfputype;
      cputype : tcputype;
      controller: tcontrollertype;
      s: string;
    begin
{$ifdef llvm}
      def_system_macro('CPULLVM');
{$endif}
      for cputype:=low(tcputype) to high(tcputype) do
        undef_system_macro('CPU'+Cputypestr[cputype]);
      def_system_macro('CPU'+Cputypestr[init_settings.cputype]);

      for fputype:=low(tfputype) to high(tfputype) do
        undef_system_macro('FPU'+fputypestr[fputype]);
      def_system_macro('FPU'+fputypestr[init_settings.fputype]);

{$PUSH}
{$WARN 6018 OFF} { Unreachable code due to compile time evaluation }
      if ControllerSupport then
        begin
          for controller:=low(tcontrollertype) to high(tcontrollertype) do
            begin
              s:=embedded_controllers[controller].controllertypestr;
              if s<>'' then
                undef_system_macro('FPC_MCU_'+s);
            end;
          s:=embedded_controllers[init_settings.controllertype].controllertypestr;
          if s<>'' then
            def_system_macro('FPC_MCU_'+s);
        end;
{$POP}

      { define abi }
      for abi:=low(tabi) to high(tabi) do
        undef_system_macro('FPC_ABI_'+abiinfo[abi].name);
      def_system_macro('FPC_ABI_'+abiinfo[target_info.abi].name);


      { Define FPC_ABI_EABI in addition to FPC_ABI_EABIHF on EABI VFP hardfloat
        systems since most code needs to behave the same on both}
      if target_info.abi = abi_eabihf then
        def_system_macro('FPC_ABI_EABI');

      { using a case is pretty useless here (FK) }
      { some stuff for TP compatibility }
      {$ifdef i386}
        def_system_macro('CPU86');
        def_system_macro('CPU87');
        def_system_macro('CPU386');
      {$endif}

      { new processor stuff }
      {$ifdef i386}
        def_system_macro('CPUI386');
        def_system_macro('CPU32');
        def_system_macro('CPUX86');
        def_system_macro('FPC_HAS_TYPE_EXTENDED');
        def_system_macro('FPC_HAS_TYPE_DOUBLE');
        def_system_macro('FPC_HAS_TYPE_SINGLE');
      {$endif}

      {$ifdef m68k}
        def_system_macro('CPU68');
        def_system_macro('CPU68K');
        def_system_macro('CPUM68K');
        def_system_macro('CPU32');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif}

      {$ifdef powerpc}
        def_system_macro('CPUPOWERPC');
        def_system_macro('CPUPOWERPC32');
        def_system_macro('CPU32');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif}

      {$ifdef POWERPC64}
        def_system_macro('CPUPOWERPC');
        def_system_macro('CPUPOWERPC64');
        def_system_macro('CPU64');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif}

      {$ifdef x86_64}
        def_system_macro('CPUX86_64');
        def_system_macro('CPUAMD64');
        def_system_macro('CPU64');
        def_system_macro('CPUX64');
        { not supported for now, afaik (FK)
         def_system_macro('FPC_HAS_TYPE_FLOAT128'); }
      {$ifndef FPC_SUPPORT_X87_TYPES_ON_WIN64}
        { normally, win64 doesn't support the legacy fpu }
        if target_info.system=system_x86_64_win64 then
          begin
            def_system_macro('FPC_CURRENCY_IS_INT64');
            def_system_macro('FPC_COMP_IS_INT64');
          end;
      {$endif FPC_SUPPORT_X87_TYPES_ON_WIN64}
      {$endif}

      {$ifdef sparc}
        def_system_macro('CPUSPARCGEN');
        def_system_macro('CPUSPARC');
        def_system_macro('CPUSPARC32');
        def_system_macro('CPU32');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif}

      {$ifdef sparc64}
        def_system_macro('CPUSPARCGEN');
        def_system_macro('CPUSPARC64');
        def_system_macro('CPU64');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif}

      {$ifdef arm}
        def_system_macro('CPUARM');
        def_system_macro('CPU32');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif arm}

      {$ifdef avr}
        def_system_macro('CPUAVR');
        def_system_macro('CPU16');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif avr}

      {$ifdef jvm}
        def_system_macro('CPUJVM');
        def_system_macro('CPU32');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif jvm}

      {$ifdef mipsel}
        def_system_macro('CPUMIPS');
        def_system_macro('CPUMIPS32');
        def_system_macro('CPUMIPSEL');
        def_system_macro('CPUMIPSEL32');
        def_system_macro('CPU32');
        def_system_macro('FPC_HAS_TYPE_DOUBLE');
        def_system_macro('FPC_HAS_TYPE_SINGLE');
        def_system_macro('FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
        def_system_macro('FPC_REQUIRES_PROPER_ALIGNMENT');
        { On most systems, locals are accessed relative to base pointer,
          but for MIPS cpu, they are accessed relative to stack pointer.
          This needs adaptation for so low level routines,
          like MethodPointerLocal and related objects unit functions. }
        def_system_macro('FPC_LOCALS_ARE_STACK_REG_RELATIVE');
      {$endif mipsel}

      {$ifdef mipseb}
        def_system_macro('CPUMIPS');
        def_system_macro('CPUMIPS32');
        def_system_macro('CPUMIPSEB');
        def_system_macro('CPUMIPSEB32');
        def_system_macro('CPU32');
        def_system_macro('FPC_HAS_TYPE_DOUBLE');
        def_system_macro('FPC_HAS_TYPE_SINGLE');
        def_system_macro('FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
        def_system_macro('FPC_REQUIRES_PROPER_ALIGNMENT');
        { See comment above for mipsel }
        def_system_macro('FPC_LOCALS_ARE_STACK_REG_RELATIVE');
      {$endif}

      {$ifdef i8086}
        def_system_macro('CPU86');  { Borland compatibility }
        def_system_macro('CPU87');  { Borland compatibility }
        def_system_macro('CPUI8086');
        def_system_macro('CPU16');
        def_system_macro('FPC_HAS_TYPE_EXTENDED');
        def_system_macro('FPC_HAS_TYPE_DOUBLE');
        def_system_macro('FPC_HAS_TYPE_SINGLE');
        case init_settings.x86memorymodel of
          mm_tiny:    def_system_macro('FPC_MM_TINY');
          mm_small:   def_system_macro('FPC_MM_SMALL');
          mm_medium:  def_system_macro('FPC_MM_MEDIUM');
          mm_compact: def_system_macro('FPC_MM_COMPACT');
          mm_large:   def_system_macro('FPC_MM_LARGE');
          mm_huge:    def_system_macro('FPC_MM_HUGE');
        end;
      {$endif i8086}

      {$ifdef aarch64}
        def_system_macro('CPUAARCH64');
        def_system_macro('CPU64');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
      {$endif aarch64}

      {$ifdef riscv32}
        def_system_macro('CPURISCV');
        def_system_macro('CPURISCV32');
        def_system_macro('CPU32');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
        def_system_macro('FPC_REQUIRES_PROPER_ALIGNMENT');
      {$endif riscv32}
      {$ifdef riscv64}
        def_system_macro('CPURISCV');
        def_system_macro('CPURISCV64');
        def_system_macro('CPU64');
        def_system_macro('FPC_CURRENCY_IS_INT64');
        def_system_macro('FPC_COMP_IS_INT64');
        def_system_macro('FPC_REQUIRES_PROPER_ALIGNMENT');
      {$endif riscv64}

      {$if defined(cpu8bitalu)}
        def_system_macro('CPUINT8');
      {$elseif defined(cpu16bitalu)}
        def_system_macro('CPUINT16');
      {$elseif defined(cpu32bitalu)}
        def_system_macro('CPUINT32');
      {$elseif defined(cpu64bitalu)}
        def_system_macro('CPUINT64');
      {$endif defined(cpu64bitalu)}

      {$if defined(avr)}
        def_system_macro('FPC_HAS_INTERNAL_ABS_SHORTINT');
      {$endif}
      {$if defined(i8086) or defined(avr)}
        def_system_macro('FPC_HAS_INTERNAL_ABS_SMALLINT');
      {$endif i8086 or avr}
      { abs(long) is handled internally on all CPUs }
        def_system_macro('FPC_HAS_INTERNAL_ABS_LONG');
      {$if defined(i8086) or defined(i386) or defined(x86_64) or defined(powerpc64) or defined(aarch64)}
        def_system_macro('FPC_HAS_INTERNAL_ABS_INT64');
      {$endif i8086 or i386 or x86_64 or powerpc64 or aarch64}

        def_system_macro('FPC_HAS_UNICODESTRING');
        def_system_macro('FPC_RTTI_PACKSET1');
        def_system_macro('FPC_HAS_CPSTRING');
      {$ifdef x86_64}
        def_system_macro('FPC_HAS_RIP_RELATIVE');
      {$endif x86_64}
        def_system_macro('FPC_HAS_CEXTENDED');
        def_system_macro('FPC_HAS_RESSTRINITS');

      { these cpus have an inline rol/ror implementaion }
      {$ifdef cpurox}
      {$ifdef m68k}
        if CPUM68K_HAS_ROLROR in cpu_capabilities[init_settings.cputype] then
          def_system_macro('FPC_HAS_INTERNAL_ROX');
      {$else}
        def_system_macro('FPC_HAS_INTERNAL_ROX');
      {$endif}
      {$endif}

      {$ifdef powerpc64}
        def_system_macro('FPC_HAS_LWSYNC');
      {$endif}

      { currently, all supported CPUs have an internal sar implementation }
        def_system_macro('FPC_HAS_INTERNAL_SAR');
      {$ifdef SUPPORT_GET_FRAME}
        def_system_macro('INTERNAL_BACKTRACE');
      {$endif SUPPORT_GET_FRAME}
        def_system_macro('STR_CONCAT_PROCS');
      {$warnings off}
        if pocall_default = pocall_register then
          def_system_macro('REGCALL');
      {$warnings on}
    end;

var
  env: ansistring;
  i : tfeature;
  j : longint;
  tmplist : TCmdStrList;
  cmditem,
  tmpcmditem : TCmdStrListItem;
  cmdstr : TCmdStr;
{$if defined(cpucapabilities)}
  cpuflag : tcpuflags;
{$endif defined(cpucapabilities)}
{$if defined(fpucapabilities)}
  fpuflag : tfpuflags;
{$endif defined(fpucapabilities)}
{$if defined(cpucapabilities) or defined(fpucapabilities)}
  hs : string;
{$endif defined(cpucapabilities) or defined(fpucapabilities)}
begin
  option:=coption.create;
  disable_configfile:=false;

  { Non-core target defines }
  Option.TargetOptions(true);

{ get default messagefile }
  msgfilename:=GetEnvironmentVariable('PPC_ERROR_FILE');

{ default configfile can be specified on the commandline,
   remove it first }
  if (cmd<>'') and (cmd[1]='[') then
    begin
      ppccfg:=Copy(cmd,2,pos(']',cmd)-2);
      Delete(cmd,1,pos(']',cmd));
    end
  else
    ppccfg:='fpc.cfg';

{ first pass reading of parameters, only -i -v -T etc.}
  option.firstpass:=true;
  if cmd<>'' then
    option.parsecmd(cmd)
  else
    begin
      option.read_parameters;
      { Write only quickinfo }
      if option.quickinfo<>'' then
        option.writequickinfo;
    end;
  option.firstpass:=false;

  { redefine target options so all defines are written even if no -Txxx is passed on the command line }
  Option.TargetOptions(true);

{ target is set here, for wince the default app type is gui }
  if target_info.system in systems_wince then
    SetApptype(app_gui)
  else
    SetApptype(apptype);

{ default defines }
  def_system_macro(target_info.shortname);
  def_system_macro('FPC');
  def_system_macro('VER'+version_nr);
  def_system_macro('VER'+version_nr+'_'+release_nr);
  def_system_macro('VER'+version_nr+'_'+release_nr+'_'+patch_nr);

{ Temporary defines, until things settle down }
  def_system_macro('FPC_HAS_OPERATOR_ENUMERATOR');
  def_system_macro('FPC_HAS_CONSTREF');
  def_system_macro('FPC_STATICRIPFIXED');
  def_system_macro('FPC_VARIANTCOPY_FIXED');
  def_system_macro('FPC_DYNARRAYCOPY_FIXED');
  def_system_macro('FPC_HAS_MEMBAR');
  def_system_macro('FPC_SETBASE_USED');

  { don't remove this, it's also for fpdoc necessary (FK) }
  def_system_macro('FPC_HAS_FEATURE_SUPPORT');

  { make cpu makros available when reading the config files the second time }
  def_cpu_macros;

  if tf_cld in target_info.flags then
    if not UpdateTargetSwitchStr('CLD', init_settings.targetswitches, true) then
      InternalError(2013092801);
  if tf_x86_far_procs_push_odd_bp in target_info.flags then
    if not UpdateTargetSwitchStr('FARPROCSPUSHODDBP', init_settings.targetswitches, true) then
      InternalError(2013092801);

  { Use standard Android NDK prefixes when cross-compiling }
  if (source_info.system<>target_info.system) and (target_info.system in systems_android) then
    case target_info.system of
      system_arm_android:
        utilsprefix:='arm-linux-androideabi-';
      system_i386_android:
        utilsprefix:='i686-linux-android-';
      else
        utilsprefix:=target_cpu_string + '-linux-android-';
    end;

  { Set up default value for the heap }
  if target_info.system in systems_embedded then
    begin
      case target_info.system of
{$ifdef AVR}
        system_avr_embedded:
          if init_settings.controllertype=ct_avrsim then
            heapsize:=8192
          else
            heapsize:=128;
{$endif AVR}
        system_arm_embedded:
          heapsize:=256;
        system_mipsel_embedded:
          heapsize:=256;
        else
          heapsize:=256;
      end;
    end;

  { read configuration file }
  if (not disable_configfile) and
     (ppccfg<>'') then
    read_configfile:=check_configfile(ppccfg,ppccfg)
  else
    read_configfile := false;

{ Read commandline and configfile }
  param_file:='';

  { read configfile }
  if read_configfile then
    option.interpret_file(ppccfg);

  { read parameters again to override config file }
  if cmd<>'' then
    option.parsecmd(cmd)
  else
    begin
      { Write help pages if no parameters are passed }
      if (paramcount=0) then
        Option.WriteHelpPages;
      option.read_parameters;
      { Write only quickinfo }
      if option.quickinfo<>'' then
        option.writequickinfo;
    end;

  { check the compatibility of different options and adjust them if necessary
    (and print possible errors)
  }
  option.checkoptionscompatibility;

  { uses the CPUXXX-defines and target_info to determine whether the selected
    target processor, if any, is supported }
  Option.VerifyTargetProcessor;

  { Stop if errors in options }
  if ErrorCount>0 then
   StopOptions(1);

  { endian define }
  case target_info.endian of
    endian_little :
      begin
        def_system_macro('ENDIAN_LITTLE');
        def_system_macro('FPC_LITTLE_ENDIAN');
      end;
    endian_big :
      begin
        def_system_macro('ENDIAN_BIG');
        def_system_macro('FPC_BIG_ENDIAN');
      end;
  end;

  { Write logo }
  if option.ParaLogo then
    option.writelogo;

{ Check file to compile }
  if param_file='' then
   begin
     Message(option_no_source_found);
     StopOptions(1);
   end;
{$ifndef Unix}
  param_file:=FixFileName(param_file);
{$endif not unix}
  inputfilepath:=ExtractFilePath(param_file);
  inputfilename:=ExtractFileName(param_file);
  if ExtractFileExt(inputfilename)='' then
    begin
      if FileExists(inputfilepath+ChangeFileExt(inputfilename,sourceext)) then
        inputfilename:=ChangeFileExt(inputfilename,sourceext)
      else if FileExists(inputfilepath+ChangeFileExt(inputfilename,pasext)) then
        inputfilename:=ChangeFileExt(inputfilename,pasext)
      else if ((m_mac in current_settings.modeswitches) or
              (tf_p_ext_support in target_info.flags))
             and FileExists(inputfilepath+ChangeFileExt(inputfilename,pext)) then
        inputfilename:=ChangeFileExt(inputfilename,pext);
    end;

  { Check output dir }
  if (OutputExeDir<>'') and
     not PathExists(OutputExeDir,false) then
    begin
      Message1(general_e_path_does_not_exist,OutputExeDir);
      StopOptions(1);
    end;

  { Add paths specified with parameters to the searchpaths }
  UnitSearchPath.AddList(option.ParaUnitPath,true);
  ObjectSearchPath.AddList(option.ParaObjectPath,true);
  IncludeSearchPath.AddList(option.ParaIncludePath,true);
  LibrarySearchPath.AddList(option.ParaLibraryPath,true);
  FrameworkSearchPath.AddList(option.ParaFrameworkPath,true);
  packagesearchpath.addlist(option.parapackagepath,true);
  for j:=0 to option.parapackages.count-1 do
    add_package(option.parapackages.NameOfIndex(j),true,true);

  { add default namespaces }
  tmplist:=TCmdStrList.Create;
  cmditem:=TCmdStrListItem(option.paranamespaces.First);
  while assigned(cmditem) do
    begin
      { use a temporary list cause if ";" are involved we need to reverse the
        order due to how TCmdStrList behaves }
      cmdstr:=cmditem.str;
      repeat
        j:=Pos(';',cmdstr);
        if j>0 then
          begin
            tmplist.insert(copy(cmdstr,1,j-1));
            delete(cmdstr,1,j);
          end
        else
          tmplist.insert(cmdstr);
      until j=0;
      tmpcmditem:=TCmdStrListItem(tmplist.First);
      while assigned(tmpcmditem) do
        begin
          namespacelist.insert(tmpcmditem.Str);
          tmpcmditem:=TCmdStrListItem(tmpcmditem.Next);
        end;
      tmplist.clear;
      cmditem:=TCmdStrListItem(cmditem.Next);
    end;
  tmplist.Free;

  { add unit environment and exepath to the unit search path }
  if inputfilepath<>'' then
   Unitsearchpath.AddPath(inputfilepath,true);
  if not disable_configfile then
    begin
      env:=GetEnvironmentVariable(target_info.unit_env);
      if env<>'' then
        UnitSearchPath.AddPath(GetEnvironmentVariable(target_info.unit_env),false);
    end;

{$ifdef Unix}
  fpcdir:=FixPath(GetEnvironmentVariable('FPCDIR'),false);
  if fpcdir='' then
    begin
      if PathExists('/usr/local/lib/fpc/'+version_string,true) then
        fpcdir:='/usr/local/lib/fpc/'+version_string+'/'
      else
        fpcdir:='/usr/lib/fpc/'+version_string+'/';
    end;
{$else unix}
  fpcdir:=FixPath(GetEnvironmentVariable('FPCDIR'),false);
  if fpcdir='' then
    begin
      fpcdir:=ExePath+'../';
      if not(PathExists(fpcdir+'units',true)) and
         not(PathExists(fpcdir+'rtl',true)) then
        fpcdir:=fpcdir+'../';
    end;
{$endif unix}
  { first try development RTL, else use the default installation path }
  if not disable_configfile then
    begin
      if PathExists(FpcDir+'rtl',true) then
        if (tf_use_8_3 in Source_Info.Flags) or
           (tf_use_8_3 in Target_Info.Flags) then
          UnitSearchPath.AddPath(FpcDir+'rtl/'+target_os_string,false)
        else
          UnitSearchPath.AddPath(FpcDir+'rtl/'+target_full_string,false)
      else
        if (tf_use_8_3 in Source_Info.Flags) or
           (tf_use_8_3 in Target_Info.Flags) then
          UnitSearchPath.AddPath(FpcDir+'units/'+target_os_string+'/rtl',false)
        else
          UnitSearchPath.AddPath(FpcDir+'units/'+target_full_string+'/rtl',false);
    end;
  { Add exepath if the exe is not in the current dir, because that is always searched already.
    Do not add it when linking on the target because then we can maybe already find
    .o files that are not for the target }
  if (ExePath<>cfileutl.GetCurrentDir) and
     not(cs_link_on_target in init_settings.globalswitches) then
   UnitSearchPath.AddPath(ExePath,false);
  { Add unit dir to the object and library path }
  objectsearchpath.AddList(unitsearchpath,false);
  librarysearchpath.AddList(unitsearchpath,false);

{$ifdef llvm}
  { default to clang }
  if (option.paratargetasm=as_none) then
    begin
      option.paratargetasm:=as_llvm_clang;
    end;
{$endif llvm}
  { maybe override assembler }
  if (option.paratargetasm<>as_none) then
    begin
      if not set_target_asm(option.paratargetasm) then
        begin
          Message2(option_incompatible_asm,asminfos[option.paratargetasm]^.idtxt,target_info.name);
          set_target_asm(target_info.assemextern);
          Message1(option_asm_forced,target_asm.idtxt);
        end;
      if (af_no_debug in asminfos[option.paratargetasm]^.flags) and
         (option.paratargetdbg<>dbg_none) then
        begin
          Message1(option_confict_asm_debug,
            asminfos[option.paratargetasm]^.idtxt);
          option.paratargetdbg:=dbg_none;
          exclude(init_settings.moduleswitches,cs_debuginfo);
        end;
      { Some assemblers, like clang, do not support
        stabs debugging format, switch to dwardé in that case }
      if (af_no_stabs in asminfos[option.paratargetasm]^.flags) and
         (option.paratargetdbg=dbg_stabs) then
        begin
          option.paratargetdbg:=dbg_dwarf2;
        end;
    end;

  {TOptionheck a second time as we might have changed assembler just above }
  option.checkoptionscompatibility;

  { maybe override debug info format }
  if (option.paratargetdbg<>dbg_none) then
    if not set_target_dbg(option.paratargetdbg) then
      Message(option_w_unsupported_debug_format);

  { switch assembler if it's binary and we got -a on the cmdline }
  if ((cs_asm_leave in init_settings.globalswitches) and
     (af_outputbinary in target_asm.flags)) or
     { if -s is passed, we shouldn't call the internal assembler }
     (cs_asm_extern in init_settings.globalswitches) then
   begin
     Message(option_switch_bin_to_src_assembler);
{$ifdef llvm}
     set_target_asm(as_llvm_clang);
{$else}
     set_target_asm(target_info.assemextern);
{$endif}
     { At least i8086 needs that for nasm and -CX
       which is incompatible with internal linker }
     option.checkoptionscompatibility;
   end;

  { Force use of external linker if there is no
    internal linker or the linking is skipped }
  if not(cs_link_extern in init_settings.globalswitches) and
     ((target_info.link=ld_none) or
      (cs_link_nolink in init_settings.globalswitches)) then
    begin
      include(init_settings.globalswitches,cs_link_extern);
    end;

  { turn off stripping if compiling with debuginfo or profile }
  if (
      (cs_debuginfo in init_settings.moduleswitches) or
      (cs_profile in init_settings.moduleswitches)
     ) and
     not(cs_link_separate_dbg_file in init_settings.globalswitches) then
    exclude(init_settings.globalswitches,cs_link_strip);

  { choose a reasonable tls model }
  if (tf_section_threadvars in target_info.flags) and (init_settings.tlsmodel=tlsm_none) then
    begin
      if cs_create_pic in init_settings.moduleswitches then
        init_settings.tlsmodel:=tlsm_global_dynamic
      else
        init_settings.tlsmodel:=tlsm_local_exec;
    end;

  { set Mac OS X version default macros if not specified explicitly }
  option.MaybeSetDefaultMacVersionMacro;

  { force fpu emulation on arm/wince, arm/gba, arm/embedded and arm/nds
    if fpu type not explicitly set }
  if not(option.FPUSetExplicitly) and
     ((target_info.system in [system_arm_wince,system_arm_gba,
         system_m68k_amiga,system_m68k_atari,
         system_arm_nds,system_arm_embedded,
         system_riscv32_embedded,system_riscv64_embedded])
{$ifdef arm}
      or (target_info.abi=abi_eabi)
{$endif arm}
     )
{$if defined(arm) or defined(riscv32) or defined(riscv64) or defined (m68k)}
     or (init_settings.fputype=fpu_soft)
{$endif arm or m68k}
  then
    begin
{$ifdef cpufpemu}
      include(init_settings.moduleswitches,cs_fp_emulation);
      { cs_fp_emulation and fpu_soft are equal on arm and m68k }
      init_settings.fputype:=fpu_soft;
{$endif cpufpemu}
    end;

{$ifdef i386}
  case target_info.system of
    system_i386_android:
      begin
        { set default cpu type to PentiumM for Android unless specified otherwise }
        if not option.CPUSetExplicitly then
          init_settings.cputype:=cpu_PentiumM;
        if not option.OptCPUSetExplicitly then
          init_settings.optimizecputype:=cpu_PentiumM;
        { set default fpu type to SSSE3 for Android unless specified otherwise }
        if not option.FPUSetExplicitly then
          init_settings.fputype:=fpu_ssse3;
      end;
    else
      ;
  end;
{$endif i386}

{$ifdef arm}
  case target_info.system of
    system_arm_darwin:
      begin
        { set default cpu type to ARMv7 for Darwin unless specified otherwise, and fpu
          to VFPv3 (that's what all 32 bit ARM iOS devices use nowadays)
        }
        if not option.CPUSetExplicitly then
          init_settings.cputype:=cpu_armv7;
        if not option.OptCPUSetExplicitly then
          init_settings.optimizecputype:=cpu_armv7;
        if not option.FPUSetExplicitly then
          init_settings.fputype:=fpu_vfpv3;
      end;
    system_arm_android:
      begin
        { set default cpu type to ARMv5T for Android unless specified otherwise }
        if not option.CPUSetExplicitly then
          init_settings.cputype:=cpu_armv5t;
        if not option.OptCPUSetExplicitly then
          init_settings.optimizecputype:=cpu_armv5t;
      end;
    else
      ;
  end;

  { ARMHF defaults }
  if (target_info.abi = abi_eabihf) then
    { set default cpu type to ARMv7a for ARMHF unless specified otherwise }
    begin
    {$ifdef CPUARMV6}
      { if the compiler is built for armv6, then
        inherit this setting, e.g. Raspian is armhf but
        only armv6, this makes rebuilds of the compiler
        easier }
      if not option.CPUSetExplicitly then
        init_settings.cputype:=cpu_armv6;
      if not option.OptCPUSetExplicitly then
        init_settings.optimizecputype:=cpu_armv6;
    {$else CPUARMV6}
      if not option.CPUSetExplicitly then
        init_settings.cputype:=cpu_armv7a;
      if not option.OptCPUSetExplicitly then
        init_settings.optimizecputype:=cpu_armv7a;
    {$endif CPUARMV6}

      { Set FPU type }
      if not(option.FPUSetExplicitly) then
        begin
          if init_settings.cputype < cpu_armv7 then
            init_settings.fputype:=fpu_vfpv2
          else
            init_settings.fputype:=fpu_vfpv3_d16;
        end
      else
        begin
          if (not(FPUARM_HAS_VFP_EXTENSION in fpu_capabilities[init_settings.fputype]))
	     or (target_info.system = system_arm_darwin) then
            begin
              Message(option_illegal_fpu_eabihf);
              StopOptions(1);
            end;
        end;
    end;

  if (init_settings.instructionset=is_thumb) and not(CPUARM_HAS_THUMB2 in cpu_capabilities[init_settings.cputype]) then
    begin
      def_system_macro('CPUTHUMB');
      if not option.FPUSetExplicitly then
        init_settings.fputype:=fpu_soft;
{$if defined(FPC_ARMEL) or defined(FPC_ARMHF)}
      target_info.llvmdatalayout:='e-p:32:32:32-i1:8:32-i8:8:32-i16:16:32-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:64:128-a0:0:32-n32-S64';
{$else FPC_ARMAL or FPC_ARMHF}
      if target_info.endian=endian_little then
        target_info.llvmdatalayout:='e-p:32:32:32-i1:8:32-i8:8:32-i16:16:32-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:32:64-v128:32:128-a0:0:32-n32-S32';
{$endif FPC_ARMAL or FPC_ARMHF}
    end;

  if (init_settings.instructionset=is_thumb) and (CPUARM_HAS_THUMB2 in cpu_capabilities[init_settings.cputype]) then
    def_system_macro('CPUTHUMB2');
{$endif arm}

{$if defined(riscv32) or defined(riscv64)}
  { RISC-V defaults }
  if (target_info.abi = abi_riscv_hf) then
    begin
      {$ifdef riscv32}
      if not option.CPUSetExplicitly then
        init_settings.cputype:=cpu_rv32ima;
      if not option.OptCPUSetExplicitly then
        init_settings.optimizecputype:=cpu_rv32ima;
      {$else}
      if not option.CPUSetExplicitly then
        init_settings.cputype:=cpu_rv64imac;
      if not option.OptCPUSetExplicitly then
        init_settings.optimizecputype:=cpu_rv64imac;
      {$endif}

      { Set FPU type }
      if not(option.FPUSetExplicitly) then
        init_settings.fputype:=fpu_fd
      else
        begin
          if not (init_settings.fputype in [fpu_fd]) then
            begin
              Message(option_illegal_fpu_eabihf);
              StopOptions(1);
            end;
        end;
    end;
{$endif defined(riscv32) or defined(riscv64)}

{$ifdef jvm}
  { set default CPU type to Dalvik when targeting Android }
  if target_info.system=system_jvm_android32 then
    begin
      if not option.CPUSetExplicitly then
        init_settings.cputype:=cpu_dalvik;
    end;
{$endif jvm}

{$ifdef llvm}
  { standard extension for llvm bitcode files }
  target_info.asmext:='.ll';
  { don't generate dwarf cfi, llvm will do that }
  exclude(target_info.flags,tf_needs_dwarf_cfi);
{$endif llvm}
{$ifdef mipsel}
  case target_info.system of
    system_mipsel_android:
      begin
        { set default cpu type to MIPS32 rev. 1 and hard float for MIPS-Android unless specified otherwise }
        if not option.CPUSetExplicitly then
          init_settings.cputype:=cpu_mips32;
        if not option.OptCPUSetExplicitly then
          init_settings.optimizecputype:=cpu_mips32;
        if not option.FPUSetExplicitly then
          init_settings.fputype:=fpu_mips2;
      end;
    system_mipsel_embedded:
      begin
        { set default cpu type to PIC32MX and softfloat for MIPSEL-EMBEDDED target unless specified otherwise }
        if not option.CPUSetExplicitly then
          init_settings.cputype:=cpu_pic32mx;
        if not option.OptCPUSetExplicitly then
          init_settings.optimizecputype:=cpu_pic32mx;
        if not option.FPUSetExplicitly then
          init_settings.fputype:=fpu_soft;
      end;
    else
      ;
  end;
{$endif mipsel}
{$ifdef m68k}
  if init_settings.cputype in cpu_coldfire then
    def_system_macro('CPUCOLDFIRE');

  case target_info.system of
    system_m68k_linux,
    system_m68k_netbsd:
      begin
        if not (option.FPUSetExplicitly) and
           not (init_settings.cputype in cpu_coldfire) then
          begin
            { enable HW FPU for UNIX by default, but only for
              original 68k, not Coldfire }
            exclude(init_settings.moduleswitches,cs_fp_emulation);
            init_settings.fputype:=fpu_68881;
          end;
      end;
    system_m68k_palmos:
      begin
        if not option.CPUSetExplicitly then
          init_settings.cputype:=cpu_mc68000;
        if not (option.FPUSetExplicitly) then
          begin
            { No FPU for PalmOS by default }
            exclude(init_settings.moduleswitches,cs_fp_emulation);
            init_settings.fputype:=fpu_none;
          end;
      end;
    else
      ;
  end;
{$endif m68k}

  { now we can define cpu and fpu type }
  def_cpu_macros;

  { Use init_settings cpu type for asm cpu type,
    if asmcputype is cpu_none,
    at least as long as there is no explicit
    option to set it on command line PM }
  if init_settings.asmcputype = cpu_none then
    init_settings.asmcputype:=init_settings.cputype;

{$ifdef llvm}
  def_system_macro('CPULLVM');
{$endif llvm}

{$if defined(cpucapabilities)}
  for cpuflag:=low(cpuflag) to high(cpuflag) do
    begin
      str(cpuflag,hs);
      if cpuflag in cpu_capabilities[init_settings.cputype] then
        def_system_macro(hs)
      else
        undef_system_macro(hs);
    end;
{$endif defined(cpucapabilities)}
{$if defined(fpucapabilities)}
  for fpuflag:=low(fpuflag) to high(fpuflag) do
    begin
      str(fpuflag,hs);
      if fpuflag in fpu_capabilities[init_settings.fputype] then
        def_system_macro(hs)
      else
        undef_system_macro(hs);
    end;
{$endif defined(fpucapabilities)}

  if init_settings.fputype<>fpu_none then
    begin
{$if defined(i386) or defined(i8086)}
      def_system_macro('FPC_HAS_TYPE_EXTENDED');
{$endif}
      def_system_macro('FPC_HAS_TYPE_SINGLE');
      def_system_macro('FPC_HAS_TYPE_DOUBLE');
{$if not defined(i386) and not defined(x86_64) and not defined(i8086) and not defined(aarch64)}
      def_system_macro('FPC_INCLUDE_SOFTWARE_INT64_TO_DOUBLE');
{$endif}
{$if defined(m68k)}
      def_system_macro('FPC_INCLUDE_SOFTWARE_LONGWORD_TO_DOUBLE');
{$endif}
{$ifdef x86_64}
{$ifndef FPC_SUPPORT_X87_TYPES_ON_WIN64}
      { normally, win64 doesn't support the legacy fpu }
      if target_info.system=system_x86_64_win64 then
        undef_system_macro('FPC_HAS_TYPE_EXTENDED')
      else
{$endif FPC_SUPPORT_X87_TYPES_ON_WIN64}
        def_system_macro('FPC_HAS_TYPE_EXTENDED');
{$endif}
    end;
    { Enable now for testing }
{$ifndef DISABLE_TLS_DIRECTORY}
    if target_info.system in systems_windows then
      def_system_macro('FPC_USE_TLS_DIRECTORY');
{$endif not DISABLE_TLS_DIRECTORY}

{$ifndef DISABLE_WIN64_SEH}
    if target_info.system=system_x86_64_win64 then
      def_system_macro('FPC_USE_WIN64_SEH');
{$endif DISABLE_WIN64_SEH}

{$ifdef TEST_WIN32_SEH}
    if target_info.system=system_i386_win32 then
      def_system_macro('FPC_USE_WIN32_SEH');
{$endif TEST_WIN32_SEH}

{$ifdef ARM}
  { define FPC_DOUBLE_HILO_SWAPPED if needed to properly handle doubles in RTL }
  if (init_settings.fputype in [fpu_fpa,fpu_fpa10,fpu_fpa11]) and
    not(cs_fp_emulation in init_settings.moduleswitches) then
    def_system_macro('FPC_DOUBLE_HILO_SWAPPED');
{$endif ARM}

{ inline bsf/bsr implementation }
{$if defined(i386) or defined(x86_64) or defined(aarch64) or defined(powerpc) or defined(powerpc64)}
  def_system_macro('FPC_HAS_INTERNAL_BSF');
  def_system_macro('FPC_HAS_INTERNAL_BSR');
{$endif}

{ hardware FMA support }
{$if defined(i386) or defined(x86_64)}
  if (cpu_capabilities[current_settings.cputype]*[CPUX86_HAS_FMA,CPUX86_HAS_FMA4])<>[] then
    begin
      def_system_macro('FPC_HAS_FAST_FMA_SINGLE');
      def_system_macro('FPC_HAS_FAST_FMA_DOUBLE');
    end;
{$endif defined(i386) or defined(x86_64)}

{$if defined(arm)}
  { it is determined during system unit compilation if clz is used for bsf or not,
    this is not perfect but the current implementation bsf/bsr does not allow another
    solution }
  if (CPUARM_HAS_CLZ in cpu_capabilities[init_settings.cputype]) and
     ((init_settings.instructionset=is_arm) or
      (CPUARM_HAS_THUMB2 in cpu_capabilities[init_settings.cputype])) then
    begin
      def_system_macro('FPC_HAS_INTERNAL_BSR');
      if CPUARM_HAS_RBIT in cpu_capabilities[init_settings.cputype] then
        def_system_macro('FPC_HAS_INTERNAL_BSF');
    end;
{$endif}

{$if defined(powerpc64)}
  { on sysv targets, default to elfv2 for little endian and to elfv1 for
    big endian (unless specified otherwise). As the gcc man page says:
    "Overriding the default ABI requires special system support and is
     likely to fail in spectacular ways" }
  if not option.ABISetExplicitly then
    begin
      if (target_info.abi=abi_powerpc_sysv) and
         (target_info.endian=endian_little) then
        target_info.abi:=abi_powerpc_elfv2
      else
        if (target_info.abi=abi_powerpc_elfv2) and
         (target_info.endian=endian_big) then
        target_info.abi:=abi_powerpc_sysv
    end;
{$endif}

{$if defined(powerpc) or defined(powerpc64)}
  { define _CALL_ELF symbol like gcc }
  case target_info.abi of
    abi_powerpc_sysv:
      set_system_compvar('_CALL_ELF','1');
    abi_powerpc_elfv2:
      set_system_compvar('_CALL_ELF','2');
    else
      ;
    end;
{$endif}

  { Section smartlinking conflicts with import sections on Windows }
  if GenerateImportSection and
     (target_info.system in [system_i386_win32,system_x86_64_win64]) then
    exclude(target_info.flags,tf_smartlink_sections);

  if not option.LinkTypeSetExplicitly then
    set_default_link_type;

  { Default alignment settings,
    1. load the defaults for the target
    2. override with generic optimizer setting (little size)
    3. override with the user specified -Oa }
  UpdateAlignment(init_settings.alignment,target_info.alignment);
  if (cs_opt_size in init_settings.optimizerswitches) then
   begin
     init_settings.alignment.procalign:=1;
     init_settings.alignment.jumpalign:=1;
     init_settings.alignment.coalescealign:=1;
     init_settings.alignment.loopalign:=1;
{$ifdef x86}
     { constalignmax=1 keeps the executable and thus the memory foot print small but
       all processors except x86 are really hurt by this or might even crash }
     init_settings.alignment.constalignmax:=1;
{$endif x86}
   end;

  UpdateAlignment(init_settings.alignment,option.paraalignment);

  set_system_macro('FPC_VERSION',version_nr);
  set_system_macro('FPC_RELEASE',release_nr);
  set_system_macro('FPC_PATCH',patch_nr);
  set_system_macro('FPC_FULLVERSION',Format('%d%.02d%.02d',[StrToInt(version_nr),StrToInt(release_nr),StrToInt(patch_nr)]));

  if target_info.system in systems_indirect_entry_information then
    def_system_macro('FPC_HAS_INDIRECT_ENTRY_INFORMATION');

  if not (tf_winlikewidestring in target_info.flags) then
    def_system_macro('FPC_WIDESTRING_EQUAL_UNICODESTRING');

  if tf_supports_packages in target_info.flags then
    def_system_macro('FPC_HAS_DYNAMIC_PACKAGES');

  if target_info.system in systems_indirect_var_imports then
    def_system_macro('FPC_HAS_INDIRECT_VAR_ACCESS');

  if cs_compilesystem in init_settings.moduleswitches then
    for i:=low(tfeature) to high(tfeature) do
      if i in features then
        def_system_macro('FPC_HAS_FEATURE_'+featurestr[i]);

{$push}
{$warn 6018 off} { Unreachable code due to compile time evaluation }
  if ControllerSupport and (target_info.system in systems_embedded) and
    (init_settings.controllertype<>ct_none) then
    begin
      with embedded_controllers[init_settings.controllertype] do
        begin
          set_system_macro('FPC_FLASHBASE',tostr(flashbase));
          set_system_macro('FPC_FLASHSIZE',tostr(flashsize));
          set_system_macro('FPC_SRAMBASE',tostr(srambase));
          set_system_macro('FPC_SRAMSIZE',tostr(sramsize));
          set_system_macro('FPC_EEPROMBASE',tostr(eeprombase));
          set_system_macro('FPC_EEPROMSIZE',tostr(eepromsize));
          set_system_macro('FPC_BOOTBASE',tostr(bootbase));
          set_system_macro('FPC_BOOTSIZE',tostr(bootsize));
        end;
    end;
{$pop}
  { as stackalign is not part of the alignment record, we do not need to define the others alignments for symmetry yet }
  set_system_macro('FPC_STACKALIGNMENT',tostr(target_info.stackalign));

  option.free;
  Option:=nil;

  clearstack_pocalls := [pocall_cdecl,pocall_cppdecl,pocall_syscall,pocall_mwpascal,pocall_sysv_abi_cdecl,pocall_ms_abi_cdecl];
  cdecl_pocalls := [pocall_cdecl, pocall_cppdecl, pocall_mwpascal, pocall_sysv_abi_cdecl, pocall_ms_abi_cdecl];
  if (tf_safecall_clearstack in target_info.flags) then
    begin
      include (cdecl_pocalls, pocall_safecall);
      include (clearstack_pocalls, pocall_safecall)
    end;
end;


initialization
  coption:=toption;
finalization
  if assigned(option) then
   option.free;
end.
