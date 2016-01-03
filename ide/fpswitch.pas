{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998-2000 by Berczi Gabor

    Compiler switches routines for the IDE

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit FPSwitch;

interface

uses
  Objects,
  Systems,
  WUtils,
  FPConst;

const
     MinMemSize      = 1024;     { min. local heap and stack size }
     MaxMemSize      = 67107840; { max. local heap and stack size }

type
    TParamID =
      (idNone,idAlign,idRangeChecks,idStackChecks,idIOChecks,
       idOverflowChecks,idObjMethCallChecks,
       idAsmDirect,idAsmATT,idAsmIntel,idAsmMot,idAsmStandard,
       idSymInfNone,idSymInfGlobalOnly,idSymInfGlobalLocal,
       idStackSize,idHeapSize,idStrictVarStrings,idExtendedSyntax,
       idMMXOps,idTypedAddress,idPackRecords,idPackEnum,idStackFrames,
       idReferenceInfo,idDebugInfo,idBoolEval,
       idAnsiString,idTypeInfo);

    TSwitchMode = (om_Normal,om_Debug,om_Release);

    TSwitchItemTyp = (ot_Select,ot_Boolean,ot_String,ot_MultiString,ot_Longint);

    PSwitchItem = ^TSwitchItem;
    TSwitchItem = object(TObject)
      Typ       : TSwitchItemTyp;
      Name      : string[50];
      Param     : string[10];
      ParamID   : TParamID;
      constructor Init(const n,p:string; AID: TParamID);
      function  NeedParam:boolean;virtual;
      function  ParamValue(nr:sw_integer):string;virtual;
      function  ParamValueBool(SM: TSwitchMode):boolean;virtual;
      function  ParamCount:sw_integer;virtual;
      function  GetSwitchStr(SM: TSwitchMode): string; virtual;
      function  GetNumberStr(SM: TSwitchMode): string; virtual;
      function  GetOptionStr(SM: TSwitchMode): string; virtual;
      procedure Reset;virtual;
    end;

    PSelectItem = ^TSelectItem;
    TSelectItem = object(TSwitchItem)
      IsDefault : boolean;
      constructor Init(const n,p:string; AID: TParamID);
      { Select to avoid anything in config file }
      constructor InitDefault(const n:string);
    end;

    PBooleanItem = ^TBooleanItem;
    TBooleanItem = object(TSwitchItem)
      IsSet : array[TSwitchMode] of boolean;
      constructor Init(const n,p:string; AID: TParamID);
      function  NeedParam:boolean;virtual;
      procedure Reset;virtual;
      function  GetSwitchStr(SM: TSwitchMode): string; virtual;
      function  ParamValueBool(SM: TSwitchMode):boolean;virtual;
    end;

    PStringItem = ^TStringItem;
    TStringItem = object(TSwitchItem)
      Str : array[TSwitchMode] of string;
      multiple : boolean;
      SeparateSpaces : boolean;
      constructor Init(const n,p:string;AID: TParamID; mult,allowspaces:boolean);
      function  NeedParam:boolean;virtual;
      function  ParamValue(nr:sw_integer):string;virtual;
      procedure Reset;virtual;
    end;

    PMultiStringItem = ^TMultiStringItem;
    TMultiStringItem = object(TSwitchItem)
      MultiStr : array[TSwitchMode] of PunsortedStringCollection;
      constructor Init(const n,p:string;AID: TParamID);
      function  NeedParam:boolean;virtual;
      function  ParamValue(nr:sw_integer):string;virtual;
      function  ParamCount:sw_integer;virtual;
      procedure Reset;virtual;
      destructor done;virtual;
    end;


    PLongintItem = ^TLongintItem;
    TLongintItem = object(TSwitchItem)
      Val : array[TSwitchMode] of longint;
      constructor Init(const n,p:string; AID: TParamID);
      function  NeedParam:boolean;virtual;
      function  ParamValue(nr:sw_integer):string;virtual;
      function  GetNumberStr(SM: TSwitchMode): string; virtual;
      procedure Reset;virtual;
    end;

    PSwitches = ^TSwitches;
    TSwitches = object
      constructor Init(ch:char);
      constructor InitSelect(ch:char);
      destructor  Done;
      { general items }
      function  ItemCount:integer;
      function  ItemName(index:integer):string;
      function  ItemParam(index:integer):string;
      { type specific }
      procedure AddSelectItem(const name,param:string; AID: TParamID);
      procedure AddDefaultSelect(const name:string);
      procedure AddBooleanItem(const name,param:string; AID: TParamID);
      procedure AddLongintItem(const name,param:string; AID: TParamID);
      procedure AddStringItem(const name,param:string;AID: TParamID;mult,allowspaces:boolean);
      procedure AddMultiStringItem(const name,param:string;AID: TParamID);
      function  GetCurrSel:integer;
      function  GetCurrSelParam : String;
      function  GetBooleanItem(index:integer):boolean;
      function  GetLongintItem(index:integer):longint;
      function  GetStringItem(index:integer):string;
      function  GetMultiStringItem(index:integer):PunsortedStringCollection;
      function  GetItemTyp(index:integer):TSwitchItemTyp;
      procedure SetCurrSel(index:integer);
      function  SetCurrSelParam(const s:string) : boolean;
      procedure SetBooleanItem(index:integer;b:boolean);
      procedure SetLongintItem(index:integer;l:longint);
      procedure SetStringItem(index:integer;const s:string);
      { read / write to cfgfile which must be open }
      procedure WriteItemsCfg;
      function  ReadItemsCfg(const s:string):boolean;
    private
      IsSel  : boolean;
      Prefix : char;
      SelNr  : array[TSwitchMode] of integer;
      Items  : PCollection;
    end;

const
    SwitchesMode : TSwitchMode = om_Normal;

    SwitchesModeName : array[TSwitchMode] of string[10]=
      ('~N~ormal','~D~ebug','~R~elease');
    SwitchesModeStr : array[TSwitchMode] of string[8]=
      ('NORMAL','DEBUG','RELEASE');
    CustomArg : array[TSwitchMode] of string{$ifndef FPC}[128]{$endif}=
      ('','','');

var
    LibLinkerSwitches,
    OtherLinkerSwitches,
    DebugInfoSwitches,
    LinkAfterSwitches,
    ProfileInfoSwitches,
    {MemorySizeSwitches, doubled !! }
    SyntaxSwitches,
    CompilerModeSwitches,
    VerboseSwitches,
    CodegenSwitches,
    OptimizationSwitches,
    ProcessorCodeGenerationSwitches,
    ProcessorOptimizationSwitches,
    AsmReaderSwitches,
    AsmInfoSwitches,
    AsmOutputSwitches,
    TargetSwitches,
    ConditionalSwitches,
    MemorySwitches,
    BrowserSwitches,
    DirectorySwitches : PSwitches;

{ write/read the Switches to fpc.cfg file }
procedure WriteSwitches(const fn:string);
procedure ReadSwitches(const fn:string);

procedure UpdateAsmOutputSwitches;

{ initialize }
procedure InitSwitches;
procedure SetDefaultSwitches;
procedure DoneSwitches;
function  GetSourceDirectories : string;

procedure GetCompilerOptionLines(C: PUnsortedStringCollection);

implementation

uses
  Dos,
  GlobType,
  CpuInfo,
  FPVars,FPUtils;

var
  CfgFile : text;

{$ifdef useresstrings}
resourcestring
{$else}
const
{$endif}
      msg_automaticallycreateddontedit = 'Automatically created file, don''t edit.';

      { Compiler options }
      opt_objectpascal = 'Object pascal support';
      opt_clikeoperators = 'C-like operators';
      opt_stopafterfirsterror = 'Stop after first error';
      opt_allowlabelandgoto = 'Allow LABEL and GOTO';
      opt_cplusplusstyledinline = 'Allow inline';
      opt_globalcmacros = 'Enable macros';
      opt_allowstaticinobjects = 'Allow STATIC in objects';
      opt_assertions = 'Include assertion code';
      opt_kylix = 'Load Kylix compat. unit';
      opt_ansistring = 'Use Ansi Strings';
      opt_strictvarstrings = 'Strict var-strings';
      opt_extendedsyntax = 'Extended syntax';
      opt_allowmmxoperations = 'Allow MMX operations';

      opt_mode_freepascal = 'Free Pascal dialect';
      opt_mode_objectpascal = 'Object Pascal extension on';
      opt_mode_turbopascal = 'Turbo Pascal compatible';
      opt_mode_delphi = 'Delphi compatible';
      opt_mode_macpascal = 'Macintosh Pascal dialect';
      opt_mode_gnupascal = 'GNU Pascal';
      { Verbose options }
      opt_warnings = '~W~arnings';
      opt_notes = 'N~o~tes';
      opt_hints = '~H~ints';
      opt_generalinfo = 'General ~I~nfo';
      opt_usedtriedinfo = '~U~sed,tried info';
      opt_all = '~A~ll';
      opt_showallprocsonerror = 'Show all ~P~rocedures if error';
      { Checking options }
      opt_rangechecking = '~R~ange checking';
      opt_stackchecking = '~S~tack checking';
      opt_iochecking = '~I~/O checking';
      opt_overflowchecking = 'Integer ~o~verflow checking';
      opt_objmethcallvalid = 'Object ~m~ethod call checking';
      { Code generation }
      opt_pic = '~P~osition independent code';
      opt_smart = 'Create smart~l~inkable units';
      { Code options }
      //opt_generatefastercode = 'Generate ~f~aster code';
      opt_generatesmallercode = 'G~e~nerate smaller code';
      opt_useregistervariables = 'Use regis~t~er-variables';
      opt_uncertainoptimizations = '~U~ncertain optimizations';
      opt_level1optimizations = 'Level ~1~ optimizations';
      opt_level2optimizations = 'Level ~2~ optimizations';
      opt_level3optimizations = 'Level ~3~ optimizations';
      { optimization processor target }
      opt_i386486 = 'i~3~86/i486';
      opt_pentium = 'P~e~ntium (tm)';
      opt_pentiummmx = 'PentiumMM~X~ (tm)';
      opt_pentiumpro = '~P~entium2/PentiumM/AMD';
      opt_pentiumiv = 'Pentium~4~';
      opt_pentiumm = 'Pentium~M~';
      opt_m68000 = 'm~6~8000';
      opt_m68020 = 'm680~2~0';
      { Assembler options }
      opt_directassembler = '~D~irect assembler';
      opt_defaultassembler = '~D~efault style assembler';
      opt_attassembler = '~A~T&T style assembler';
      opt_intelassembler = '~I~ntel style assembler';
      opt_motassembler = '~M~otorola style assembler';
      opt_standardassembler = '~S~tandard style assembler';
      opt_listsource = '~L~ist source';
      opt_listregisterallocation = 'list ~r~egister allocation';
      opt_listtempallocation = 'list ~t~emp allocation';
      opt_listnodeallocation = 'list ~n~ode allocation';
      opt_useasmpipe = 'use ~p~ipe with assembler';
      { Assembler output selection }
      opt_usedefaultas = 'Use ~d~efault output';
      opt_usegnuas = 'Use ~G~NU as';
      { I386 assembler output selection }
      opt_usenasmcoff = 'Use ~N~ASM coff';
      opt_usenasmwin32 = 'Use NASM ~w~in32';
      opt_usenasmwdosx= 'Use ~N~ASM w~d~osx';
      opt_usenasmelf = 'Use NASM el~f~';
      opt_usenasmbeos = 'Use NASM ~b~eos';
      opt_usenasmobj = 'Use NASM ~o~bj';
      opt_usemasm = 'Use ~M~ASM';
      opt_usetasm = 'Use ~T~ASM';
      opt_usewasm = 'Use ~W~ASM';
      opt_usecoff = 'Use internal ~c~off';
      opt_usepecoff = 'Use internal ~p~ecoff';
      opt_usepecoffwdosx = 'Use internal pewdos~x~';
      opt_useelf= 'Use internal ~e~lf';

      { Browser options }
      opt_nobrowser = 'N~o~ browser';
      opt_globalonlybrowser = 'Only Glob~a~l browser';
      opt_localglobalbrowser = '~L~ocal and global browser';
      { Conditional defines }
      opt_conditionaldefines = 'Conditio~n~al defines';
      { Memory sizes }
      opt_stacksize = '~S~tack size';
      opt_heapsize = '~H~eap size';
      { Directory options }
      opt_unitdirectories = '~U~nit directories';
      opt_includedirectories = '~I~nclude directories';
      opt_librarydirectories = '~L~ibrary directories';
      opt_objectdirectories = '~O~bject directories';
      opt_exeppudirectories = '~E~XE output directory';
      opt_ppuoutputdirectory = '~P~PU output directory';
      opt_cross_tools_directory = '~C~ross tools directory';
      opt_dynamic_linker = '~D~ynamic linker path';
      { Library options }
      opt_librariesdefault = '~T~arget default';
      opt_dynamiclibraries = 'Link to ~D~ynamic libraries';
      opt_staticlibraries = 'Link to ~S~tatic libraries';
      opt_smartlibraries = 'Link to S~m~art libraries';
      opt_forcestaticlibs = 'Only link to st~a~tic libraries';
      { Symbol info options }
      opt_stripalldebugsymbols = '~S~trip all debug symbols from executable';
      opt_nogendebugsymbolinfo = 'Skip ~d~ebug information generation';
      opt_gendebugsymbolinfo = 'Generate ~d~ebug symbol information';
      opt_gensymbolandbacktraceinfo = 'Generate also backtrace ~l~ine information';
      opt_valgrindinfo = 'Generate ~v~algrind compatible debug info';
      { Link after options }
      opt_linkafter = 'Call ~l~inker after';
      { Profiling options }
      opt_noprofileinfo = '~N~o profile information';
      opt_gprofinfo = 'Generate profile code for g~p~rof';

{*****************************************************************************
            TSwitchItem
*****************************************************************************}

constructor TSwitchItem.Init(const n,p:string; AID: TParamID);
begin
  Inherited Init;
  Name:=n;
  Param:=p;
  ParamID:=AID;
end;


function TSwitchItem.NeedParam:boolean;
begin
  NeedParam:=false;
end;


function TSwitchItem.ParamValue(nr:sw_integer):string;
begin
  ParamValue:='';
end;

function TSwitchItem.ParamValueBool(SM: TSwitchMode):boolean;
begin
  Abstract;
  ParamValueBool:=false;
end;

function TSwitchItem.ParamCount:sw_integer;

begin
  ParamCount:=1;
end;

function TSwitchItem.GetSwitchStr(SM: TSwitchMode): string;
begin
  Abstract;
  GetSwitchStr:='';
end;

function TSwitchItem.GetNumberStr(SM: TSwitchMode): string;
begin
  Abstract;
  GetNumberStr:='';
end;

function TSwitchItem.GetOptionStr(SM: TSwitchMode): string;
begin
  Abstract;
  GetOptionStr:='';
end;

procedure TSwitchItem.Reset;
begin
end;


{*****************************************************************************
            TSelectItem
*****************************************************************************}

constructor TSelectItem.Init(const n,p:string; AID: TParamID);
begin
  Inherited Init(n,p,AID);
  Typ:=ot_Select;
  IsDefault:=false;
end;

constructor TSelectItem.InitDefault(const n:string);
begin
  Inherited Init(n,'',idNone);
  Typ:=ot_Select;
  IsDefault:=true;
end;

{*****************************************************************************
                TBooleanItem
*****************************************************************************}

constructor TBooleanItem.Init(const n,p:string; AID: TParamID);
begin
  Inherited Init(n,p,AID);
  Typ:=ot_Boolean;
  Reset;
end;


function TBooleanItem.NeedParam:boolean;
begin
  NeedParam:=IsSet[SwitchesMode];
end;


procedure TBooleanItem.Reset;
begin
  FillChar(IsSet,sizeof(IsSet),0);
end;

function TBooleanItem.ParamValueBool(SM: TSwitchMode):boolean;
begin
  ParamValueBool:=IsSet[SM];
end;

function TBooleanItem.GetSwitchStr(SM: TSwitchMode): string;
begin
  GetSwitchStr:=BoolToStr(IsSet[SM],'+','-');
end;


{*****************************************************************************
            TStringItem
*****************************************************************************}

constructor TStringItem.Init(const n,p:string; AID: TParamID; mult,allowspaces:boolean);
begin
  Inherited Init(n,p,AID);
  Typ:=ot_String;
  Multiple:=mult;
  SeparateSpaces:=not allowspaces;
  Reset;
end;


function TStringItem.NeedParam:boolean;
begin
  NeedParam:=(Str[SwitchesMode]<>'');
end;


function TStringItem.ParamValue(nr:sw_integer):string;
begin
  ParamValue:=Str[SwitchesMode];
end;


procedure TStringItem.Reset;
begin
  FillChar(Str,sizeof(Str),0);
end;

{*****************************************************************************
            TMultiStringItem
*****************************************************************************}

constructor TMultiStringItem.Init(const n,p:string;AID:TParamID);

var i:TSwitchMode;

begin
  inherited Init(n,p,AID);
  typ:=ot_MultiString;
  for i:=low(MultiStr) to high(MultiStr) do
    new(MultiStr[i],init(5,5));
{  Reset;}
end;

function TMultiStringItem.NeedParam:boolean;

begin
  NeedParam:=(multistr[SwitchesMode]^.count<>0);
end;

function TMultiStringItem.ParamValue(nr:sw_integer):string;

begin
  ParamValue:=MultiStr[SwitchesMode]^.at(nr)^;
end;

function TMultiStringItem.ParamCount:sw_integer;

begin
  ParamCount:=Multistr[SwitchesMode]^.count;
end;

procedure TMultiStringItem.Reset;

var i:TSwitchMode;

begin
  for i:=low(multiStr) to high(multiStr) do
    MultiStr[i]^.freeall;
end;

destructor TmultiStringItem.done;

var i:TSwitchMode;

begin
  for i:=low(MultiStr) to high(MultiStr) do
    dispose(MultiStr[i],done);
  inherited done;
end;

{*****************************************************************************
                TLongintItem
*****************************************************************************}

constructor TLongintItem.Init(const n,p:string; AID: TParamID);
begin
  Inherited Init(n,p,AID);
  Typ:=ot_Longint;
  Reset;
end;


function TLongintItem.NeedParam:boolean;
begin
  NeedParam:=(Val[SwitchesMode]<>0);
end;


function TLongintItem.ParamValue(nr:sw_integer):string;
var
  s : string;
begin
  Str(Val[SwitchesMode],s);
  ParamValue:=s;
end;

procedure TLongintItem.Reset;
begin
  FillChar(Val,sizeof(Val),0);
end;

function TLongintItem.GetNumberStr(SM: TSwitchMode): string;
begin
  GetNumberStr:=IntToStr(Val[SM]);
end;


{*****************************************************************************
               TSwitch
*****************************************************************************}

constructor TSwitches.Init(ch:char);
begin
  new(Items,Init(10,5));
  Prefix:=ch;
  FillChar(SelNr,SizeOf(SelNr),#0);
  IsSel:=false;
end;


constructor TSwitches.InitSelect(ch:char);
begin
  new(Items,Init(10,5));
  Prefix:=ch;
  FillChar(SelNr,SizeOf(SelNr),#0);
  IsSel:=true;
end;


destructor  TSwitches.Done;
begin
  dispose(Items,Done);
end;


procedure TSwitches.AddSelectItem(const name,param:string; AID: TParamID);
begin
  Items^.Insert(New(PSelectItem,Init(name,Param,AID)));
end;


procedure TSwitches.AddDefaultSelect(const name:string);
begin
  Items^.Insert(New(PSelectItem,InitDefault(name)));
end;


procedure TSwitches.AddBooleanItem(const name,param:string; AID: TParamID);
begin
  Items^.Insert(New(PBooleanItem,Init(name,Param,AID)));
end;


procedure TSwitches.AddLongintItem(const name,param:string; AID: TParamID);
begin
  Items^.Insert(New(PLongintItem,Init(name,Param,AID)));
end;


procedure TSwitches.AddStringItem(const name,param:string;AID:TParamID;mult,allowspaces:boolean);
begin
  Items^.Insert(New(PStringItem,Init(name,Param,AID,mult,allowspaces)));
end;

procedure TSwitches.AddMultiStringItem(const name,param:string;AID:TParamID);
begin
  Items^.Insert(New(PMultiStringItem,Init(name,Param,AID)));
end;

function TSwitches.ItemCount:integer;
begin
  ItemCount:=Items^.Count;
end;


function TSwitches.ItemName(index:integer):string;
var
  P : PSwitchItem;
begin
  if index<ItemCount then
    P:=Items^.At(Index)
  else
    P:=nil;
  if assigned(P) then
   ItemName:=P^.Name
  else
   ItemName:='';
end;


function TSwitches.ItemParam(index:integer):string;
var
  P : PSwitchItem;
begin
  if index<ItemCount then
    P:=Items^.At(Index)
  else
    P:=nil;
  if assigned(P) then
   ItemParam:='-'+Prefix+P^.Param
  else
   ItemParam:='';
end;


function TSwitches.GetBooleanItem(index:integer):boolean;
var
  P : PBooleanItem;
begin
  if index<ItemCount then
    P:=Items^.At(Index)
  else
    P:=nil;
  if assigned(P) and (P^.Typ=ot_boolean) then
   GetBooleanItem:=P^.IsSet[SwitchesMode]
  else
   GetBooleanItem:=false;
end;


function TSwitches.GetLongintItem(index:integer):longint;
var
  P : PLongintItem;
begin
  if index<ItemCount then
    P:=Items^.At(Index)
  else
    P:=nil;
  if assigned(P) and (P^.Typ=ot_longint) then
   GetLongintItem:=P^.Val[SwitchesMode]
  else
   GetLongintItem:=0;
end;


function TSwitches.GetStringItem(index:integer):string;
var
  P : PStringItem;
begin
  if index<ItemCount then
    P:=Items^.At(Index)
  else
    P:=nil;
  if assigned(P) and (P^.Typ=ot_string) then
   GetStringItem:=P^.Str[SwitchesMode]
  else
   GetStringItem:='';
end;

function TSwitches.GetMultiStringItem(index:integer):PUnsortedStringCollection;

var p:PMultiStringItem;

begin
  if index<ItemCount then
    p:=Items^.at(Index)
  else
    p:=nil;
  if (p<>nil) and (p^.typ=ot_multistring) then
    GetMultiStringItem:=p^.MultiStr[SwitchesMode]
  else
    GetMultiStringItem:=nil;
end;

function TSwitches.GetItemTyp(index:integer):TSwitchItemTyp;

var p:PSwitchItem;

begin
  assert(index<itemcount);
  GetItemTyp:=PSwitchItem(items^.at(index))^.typ;
end;

procedure TSwitches.SetBooleanItem(index:integer;b:boolean);
var
  P : PBooleanItem;
begin
  if index<ItemCount then
    P:=Items^.At(Index)
  else
    P:=nil;
  if assigned(P) and (P^.Typ=ot_boolean) then
   P^.IsSet[SwitchesMode]:=b;
end;


procedure TSwitches.SetLongintItem(index:integer;l:longint);
var
  P : PLongintItem;
begin
  if index<ItemCount then
    P:=Items^.At(Index)
  else
    P:=nil;
  if assigned(P) and (P^.Typ=ot_longint) then
   P^.Val[SwitchesMode]:=l;
end;


procedure TSwitches.SetStringItem(index:integer;const s:string);
var
  P : PStringItem;
begin
  if index<ItemCount then
    P:=Items^.At(Index)
  else
    P:=nil;
  if assigned(P) and (P^.Typ=ot_string) then
   P^.Str[SwitchesMode]:=s;
end;


function TSwitches.GetCurrSel:integer;
begin
  if IsSel then
   GetCurrSel:=SelNr[SwitchesMode]
  else
   GetCurrSel:=-1;
end;


function  TSwitches.GetCurrSelParam : String;
begin
  if IsSel then
    GetCurrSelParam:=PSwitchItem(Items^.At(SelNr[SwitchesMode]))^.Param
  else
    GetCurrSelParam:='';
end;

procedure TSwitches.SetCurrSel(index:integer);
begin
  if index<ItemCount then
   SelNr[SwitchesMode]:=index;
end;

function  TSwitches.SetCurrSelParam(const s : String) : boolean;
  function checkitem(P:PSwitchItem):boolean;
  begin
    { empty items are not equivalent to others !! }
    CheckItem:=((S='') and (P^.Param='')) or
               ((Length(S)>0) and (P^.Param=s));
  end;

var
  FoundP : PSwitchItem;
begin
  FoundP:=Items^.FirstThat(@CheckItem);
  if Assigned(FoundP) then
    begin
      SetCurrSelParam:=true;
      SelNr[SwitchesMode]:=Items^.IndexOf(FoundP);
    end
  else
    SetCurrSelParam:=false;
end;


procedure TSwitches.WriteItemsCfg;
var
  Pref : char;

  procedure writeitem(P:PSwitchItem);
  var
    s,s1 : string;
    i,j : integer;
  begin
    if P^.NeedParam then
     begin
       if (P^.Typ=ot_string) and (PStringItem(P)^.Multiple) then
         begin
           s:=PStringItem(P)^.Str[SwitchesMode];
           repeat
             i:=pos(';',s);
             if PStringItem(P)^.SeparateSpaces then
               j:=pos(' ',s)
             else
               j:=0;
             if i=0 then
              i:=256;
             if (j>0) and (j<i) then
               i:=j;
             s1:=Copy(s,1,i-1);
             if s1<>'' then
              writeln(CfgFile,' -'+Pref+P^.Param+s1);
             Delete(s,1,i);
           until s='';
         end
       else
         if P^.Param<>'/' then
           for i:=0 to p^.ParamCount-1 do
             Writeln(CfgFile,' -'+Pref+P^.Param+P^.ParamValue(i));
     end;
  end;

var
  P : PSelectItem;
begin
  Pref:=Prefix;
  if IsSel then
    begin
      { can be empty for some targets }
      If Items^.count>0 then
        begin
          P:=Items^.At(SelNr[SwitchesMode]);
          if not P^.IsDefault then
            writeln(CfgFile,' '+ItemParam(SelNr[SwitchesMode]));
        end;
    end
  else
    Items^.ForEach(@writeitem);
end;

procedure WriteCustom;
var
  s : string;
  i : longint;
begin
  s:=CustomArg[SwitchesMode];
  While s<>'' do
    begin
       i:=pos(' ',s);
       if i=0 then i:=256;
       writeln(CfgFile,' '+Copy(s,1,i-1));
       if i=256 then
         s:=''
       else
         s:=copy(s,i+1,255);
    end;
end;


function TSwitches.ReadItemsCfg(const s:string):boolean;

  function checkitem(P:PSwitchItem):boolean;
  begin
    { empty items are not equivalent to others !! }
    { but -dGDB didn't work because of this PM }
    CheckItem:=((P^.Param='') and ((S='') or (P^.typ in [ot_Boolean,ot_String]))) or
               ((Length(P^.Param)>0) and (upcase(P^.Param)=upcase(S)) and
                not (P^.typ in [ot_Boolean,ot_String])) or
               ((Length(P^.Param)>0) and (P^.typ<>ot_Select) and
                (P^.Param=Copy(s,1,length(P^.Param))));
  end;

var
  FoundP : PSwitchItem;
  code : integer;
begin
  FoundP:=Items^.FirstThat(@checkitem);
  if assigned(FoundP) then
   begin
     case FoundP^.Typ of
      ot_Select  : SelNr[SwitchesMode]:=Items^.IndexOf(FoundP);
      ot_Boolean : PBooleanItem(FoundP)^.IsSet[SwitchesMode]:=true;
      ot_String  : begin
           if (PStringItem(FoundP)^.Multiple) and (PStringItem(FoundP)^.Str[SwitchesMode]<>'') then
            PStringItem(FoundP)^.Str[SwitchesMode]:=PStringItem(FoundP)^.Str[SwitchesMode]+';'+
              Copy(s,length(FoundP^.Param)+1,255)
           else
            PStringItem(FoundP)^.Str[SwitchesMode]:=Copy(s,length(FoundP^.Param)+1,255);
         end;
      ot_MultiString :
        PMultiStringItem(foundP)^.MultiStr[SwitchesMode]^.insert(newstr(copy(s,length(foundP^.param)+1,255)));
      ot_Longint : Val(Copy(s,length(FoundP^.Param)+1,255),PLongintItem(FoundP)^.Val[SwitchesMode],code);
     end;
     ReadItemsCfg:=true;
   end
  else
   ReadItemsCfg:=false;
end;


{*****************************************************************************
             Read / Write
*****************************************************************************}

procedure WriteSwitches(const fn:string);
var
  OldSwitchesMode, SWM: TSwitchMode;
begin
{ create the switches }
  assign(CfgFile,fn);
  {$I-}
   rewrite(CfgFile);
  {$I+}
  if ioresult<>0 then
   exit;
  writeln(CfgFile,'# '+msg_automaticallycreateddontedit);
  OldSwitchesMode:=SwitchesMode;
  for SWM:=low(TSwitchMode) to high(TSwitchMode) do
   begin
     SwitchesMode := SWM;
     Writeln(CfgFile,'#IFDEF '+SwitchesModeStr[SwitchesMode]);
     TargetSwitches^.WriteItemsCfg;
     CompilerModeSwitches^.WriteItemsCfg;
     VerboseSwitches^.WriteItemsCfg;
     SyntaxSwitches^.WriteItemsCfg;
     CodegenSwitches^.WriteItemsCfg;
     OptimizationSwitches^.WriteItemsCfg;
     ProcessorCodeGenerationSwitches^.WriteItemsCfg;
     ProcessorOptimizationSwitches^.WriteItemsCfg;
     AsmReaderSwitches^.WriteItemsCfg;
     AsmInfoSwitches^.WriteItemsCfg;
     AsmOutputSwitches^.WriteItemsCfg;
     DirectorySwitches^.WriteItemsCfg;
     MemorySwitches^.WriteItemsCfg;
     ConditionalSwitches^.WriteItemsCfg;
     LibLinkerSwitches^.WriteItemsCfg;
     OtherLinkerSwitches^.WriteItemsCfg;
     DebugInfoSwitches^.WriteItemsCfg;
     ProfileInfoSwitches^.WriteItemsCfg;
     LinkAfterSwitches^.WriteItemsCfg;
     BrowserSwitches^.WriteItemsCfg;
     {MemorySizeSwitches^.WriteItemsCfg;}
     WriteCustom;
     Writeln(CfgFile,'#ENDIF');
     Writeln(CfgFile,'');
   end;
  close(CfgFile);
  SwitchesMode:=OldSwitchesMode;
end;


procedure ReadSwitches(const fn:string);
var
  c : char;
  s : string;
  res : boolean;
  OldSwitchesMode,i : TSwitchMode;
begin
  assign(CfgFile,fn);
  {$I-}
   reset(CfgFile);
  {$I+}
  if ioresult<>0 then
   begin
     SetDefaultSwitches;
     exit;
   end;
  OldSwitchesMode:=SwitchesMode;
  SwitchesMode:=om_Normal;
  while not eof(CfgFile) do
   begin
     readln(CfgFile,s);
     s:=LTrim(s);
     if (length(s)>=2) and (s[1]='-') then
      begin
      c:=s[2];
      res:=false;
      Delete(s,1,2);
      case c of
       'a' : res:=AsmInfoSwitches^.ReadItemsCfg(s);
       'A' : res:=AsmOutputSwitches^.ReadItemsCfg(s);
       'b' : res:=BrowserSwitches^.ReadItemsCfg(s);
       'C' : begin
               res:=CodegenSwitches^.ReadItemsCfg(s);
               if not res then
                 res:=MemorySwitches^.ReadItemsCfg(s);
               if not res then
                 res:=ProcessorCodeGenerationSwitches^.ReadItemsCfg(s);
             end;
       'd' : res:=ConditionalSwitches^.ReadItemsCfg(s);
       'F' : res:=DirectorySwitches^.ReadItemsCfg(s);
       'g' : res:=DebugInfoSwitches^.ReadItemsCfg(s);
       'O' : begin
               res:=OptimizationSwitches^.ReadItemsCfg(s);
               if not res then
                 res:=ProcessorOptimizationSwitches^.ReadItemsCfg(s);
             end;
       'M' : res:=CompilerModeSwitches^.ReadItemsCfg(s);
       'p' : res:=ProfileInfoSwitches^.ReadItemsCfg(s);
       's' : res:=LinkAfterSwitches^.ReadItemsCfg(s);
       'R' : res:=AsmReaderSwitches^.ReadItemsCfg(s);
       'S' : res:=SyntaxSwitches^.ReadItemsCfg(s);
       'T' : res:=TargetSwitches^.ReadItemsCfg(s);
       'v' : res:=VerboseSwitches^.ReadItemsCfg(s);
       'X' : begin
               res:=LibLinkerSwitches^.ReadItemsCfg(s);
               if not res then
                 res:=OtherLinkerSwitches^.ReadItemsCfg(s);
             end;
       end;
      { keep all others as a string }
      if not res then
       CustomArg[SwitchesMode]:=CustomArg[SwitchesMode]+' -'+c+s;
      end
     else
      if (Copy(s,1,7)='#IFDEF ') then
       begin
         Delete(s,1,7);
         for i:=low(TSwitchMode) to high(TSwitchMode) do
         if s=SwitchesModeStr[i] then
           begin
             SwitchesMode:=i;
             break;
           end;
       end
      else;
   end;
  close(CfgFile);
  SwitchesMode:=OldSwitchesMode;
end;


function  GetSourceDirectories : string;
var
  P : PStringItem;
  S : String;
  c : char;
  function checkitem(P:PSwitchItem):boolean;
  begin
    CheckItem:=(P^.Typ=ot_string) and (P^.Param=c);
  end;
begin
  GetSourceDirectories:='';
  c:='u';
  P:=DirectorySwitches^.Items^.FirstThat(@CheckItem);
  S:='';
  if assigned(P) then
    S:=P^.Str[SwitchesMode];
  c:='i';
  P:=DirectorySwitches^.Items^.FirstThat(@CheckItem);
  if assigned(P) then
    S:=P^.Str[SwitchesMode]+';'+S;
  if S='' then
    GetSourceDirectories:=SourceDirs+';'
  else
    GetSourceDirectories:=SourceDirs+';'+S+';';
end;

{*****************************************************************************
             AsmOutputInitialize
*****************************************************************************}

procedure UpdateAsmOutputSwitches;
var
  ta : tasm;
  st : string;
begin
  if assigned(AsmOutputSwitches) then
    dispose(AsmOutputSwitches,Done);
  New(AsmOutputSwitches,InitSelect('A'));
  with AsmOutputSwitches^ do
   begin

     AddDefaultSelect(opt_usedefaultas);
     for ta:=low(tasm) to high(tasm) do
       if assigned(asminfos[ta]) and
<<<<<<< HEAD
<<<<<<< HEAD
         ((target_info.system in asminfos[ta]^.supported_targets) or
         (system_any in asminfos[ta]^.supported_targets)) then
=======
         ((asminfos[ta]^.supported_target=target_info.system) or
         (asminfos[ta]^.supported_target=system_any)) then
>>>>>>> graemeg/fixes_2_2
=======
         ((asminfos[ta]^.supported_target=target_info.system) or
         (asminfos[ta]^.supported_target=system_any)) then
>>>>>>> origin/fixes_2_2
         begin
           st:='Asm '+asminfos[ta]^.idtxt;
           if asminfos[ta]^.idtxt='AS' then
             st:=opt_usegnuas;
{$ifdef I386}
           if asminfos[ta]^.idtxt='NASMCOFF' then
             st:=opt_usenasmcoff;
           if asminfos[ta]^.idtxt='NASMOBJ' then
             st:=opt_usenasmobj;
           if asminfos[ta]^.idtxt='NASMWIN32' then
             st:=opt_usenasmwin32;
           if asminfos[ta]^.idtxt='NASMWDOSX' then
             st:=opt_usenasmwdosx;
           if asminfos[ta]^.idtxt='NASMELF' then
             st:=opt_usenasmelf;
           if asminfos[ta]^.idtxt='NASMBEOS' then
             st:=opt_usenasmbeos;
           if asminfos[ta]^.idtxt='MASM' then
             st:=opt_usemasm;
           if asminfos[ta]^.idtxt='TASM' then
             st:=opt_usetasm;
           if asminfos[ta]^.idtxt='WASM' then
             st:=opt_usewasm;
           if asminfos[ta]^.idtxt='COFF' then
             st:=opt_usecoff;
           if asminfos[ta]^.idtxt='PECOFF' then
             st:=opt_usepecoff;
           if asminfos[ta]^.idtxt='PEWDOSX' then
             st:=opt_usepecoffwdosx;
           if asminfos[ta]^.idtxt='ELF' then
             st:=opt_useelf;
{$endif I386}
           AddSelectItem(st,asminfos[ta]^.idtxt,idNone);
         end;
   end;
end;

{*****************************************************************************
             Initialize
*****************************************************************************}

procedure InitSwitches;
var
  t : tsystem;
  cpu : tcputype;
  st : string;
begin
  New(SyntaxSwitches,Init('S'));
  with SyntaxSwitches^ do
   begin
//     AddBooleanItem(opt_objectpascal,'2',idNone);
     AddBooleanItem(opt_stopafterfirsterror,'e',idNone);
     AddBooleanItem(opt_allowlabelandgoto,'g',idNone);
     AddBooleanItem(opt_globalcmacros,'m',idNone);
     AddBooleanItem(opt_cplusplusstyledinline,'i',idNone);
//     AddBooleanItem(opt_tp7compatibility,'o',idNone);
//     AddBooleanItem(opt_delphicompatibility,'d',idNone);
     AddBooleanItem(opt_assertions,'a',idNone);
     AddBooleanItem(opt_ansistring,'h',idAnsiString);
     AddBooleanItem(opt_kylix,'k',idNone);
     AddBooleanItem(opt_allowstaticinobjects,'s',idNone);
     AddBooleanItem(opt_clikeoperators,'c',idNone);
     { Useless as they are not passed to the compiler PM
     AddBooleanItem(opt_strictvarstrings,'/',idStrictVarStrings);
     AddBooleanItem(opt_extendedsyntax,'/',idExtendedSyntax);
     AddBooleanItem(opt_allowmmxoperations,'/',idMMXOps);  }
   end;
  New(CompilerModeSwitches,InitSelect('M'));
  with CompilerModeSwitches^ do
    begin
       AddSelectItem(opt_mode_freepascal,'fpc',idNone);
       AddSelectItem(opt_mode_objectpascal,'objfpc',idNone);
       AddSelectItem(opt_mode_turbopascal,'tp',idNone);
       AddSelectItem(opt_mode_delphi,'delphi',idNone);
       AddSelectItem(opt_mode_macpascal,'macpas',idNone);
{      GNU Pascal mode doesn't do much, better disable it
       AddSelectItem(opt_mode_gnupascal,'gpc',idNone);}
    end;
  New(VerboseSwitches,Init('v'));
  with VerboseSwitches^ do
   begin
     AddBooleanItem(opt_warnings,'w',idNone);
     AddBooleanItem(opt_notes,'n',idNone);
     AddBooleanItem(opt_hints,'h',idNone);
     AddBooleanItem(opt_generalinfo,'i',idNone);
     AddBooleanItem(opt_usedtriedinfo,'ut',idNone);
     AddBooleanItem(opt_all,'a',idNone);
     AddBooleanItem(opt_showallprocsonerror,'b',idNone);
   end;
  New(CodegenSwitches,Init('C'));
  with CodegenSwitches^ do
   begin
     AddBooleanItem(opt_rangechecking,'r',idRangeChecks);
     AddBooleanItem(opt_stackchecking,'t',idStackChecks);
     AddBooleanItem(opt_iochecking,'i',idIOChecks);
     AddBooleanItem(opt_overflowchecking,'o',idOverflowChecks);
     AddBooleanItem(opt_objmethcallvalid,'R',idObjMethCallChecks);
     AddBooleanItem(opt_pic,'g',idNone);
     AddBooleanItem(opt_smart,'X',idNone);
   end;
  New(OptimizationSwitches,Init('O'));
  with OptimizationSwitches^ do
   begin
     AddBooleanItem(opt_generatesmallercode,'s',idNone);
{$ifdef I386}
     AddBooleanItem(opt_useregistervariables,'oregvar',idNone);
     AddBooleanItem(opt_uncertainoptimizations,'ouncertain',idNone);
     AddBooleanItem(opt_level1optimizations,'1',idNone);
     AddBooleanItem(opt_level2optimizations,'2',idNone);
     AddBooleanItem(opt_level3optimizations,'3',idNone);
{$else not I386}
 {$ifdef m68k}
     AddBooleanItem(opt_level1optimizations,'a',idNone);
     AddBooleanItem(opt_useregistervariables,'x',idNone);
 {$endif m68k}
{$endif I386}
   end;
  New(ProcessorOptimizationSwitches,InitSelect('O'));
  with ProcessorOptimizationSwitches^ do
   begin
     for cpu:=low(tcputype) to high(tcputype) do
       begin
         st:=cputypestr[cpu];
{$ifdef I386}
         if st='386' then
           st:=opt_i386486;
         if st='PENTIUM' then
           st:=opt_pentium;
         if st='PENTIUM2' then
           st:=opt_pentiummmx;
         if st='PENTIUM3' then
           st:=opt_pentiumpro;
         if st='PENTIUM4' then
           st:=opt_pentiumiv;
         if st='PENTIUMM' then
           st:=opt_pentiumM;
{$endif not I386}
{$ifdef m68k}
         if st='68000' then
           st:=opt_m68000;
         if st='68020' then
           st:=opt_m68020;
{$endif m68k}
         if st<>'' then
           AddSelectItem(st,'p'+cputypestr[cpu],idNone);
       end;
   end;
  New(ProcessorCodeGenerationSwitches,InitSelect('C'));
  with ProcessorCodeGenerationSwitches^ do
   begin
     for cpu:=low(tcputype) to high(tcputype) do
       begin
         st:=cputypestr[cpu];
{$ifdef I386}
         if st='386' then
           st:=opt_i386486;
         if st='PENTIUM' then
           st:=opt_pentium;
         if st='PENTIUM2' then
           st:=opt_pentiummmx;
         if st='PENTIUM3' then
           st:=opt_pentiumpro;
         if st='PENTIUM4' then
           st:=opt_pentiumiv;
         if st='PENTIUMM' then
           st:=opt_pentiumM;
{$endif not I386}
{$ifdef m68k}
         if st='68000' then
           st:=opt_m68000;
         if st='68020' then
           st:=opt_m68020;
{$endif m68k}
         { we use the string twice so kill duplicate highlights }
         while pos('~',st)<>0 do
           delete(st,pos('~',st),1);

         if st<>'' then
           AddSelectItem(st,'p'+cputypestr[cpu],idNone);
       end;
   end;
  New(TargetSwitches,InitSelect('T'));
  with TargetSwitches^ do
   begin
     { better, we've a correct target list without "tilded" names instead a wrong one }
     for t:=low(tsystem) to high(tsystem) do
       if assigned(targetinfos[t]) then
         AddSelectItem(targetinfos[t]^.name,targetinfos[t]^.shortname,idNone);
   end;
  New(AsmReaderSwitches,InitSelect('R'));
  with AsmReaderSwitches^ do
   begin
{$ifdef I386}
     AddSelectItem(opt_defaultassembler,'default',idNone);
{     AddSelectItem(opt_directassembler,'direct',idAsmDirect);}
     AddSelectItem(opt_attassembler,'att',idAsmATT);
     AddSelectItem(opt_intelassembler,'intel',idAsmIntel);
{$endif I386}
{$ifdef M68K}
     AddSelectItem(opt_defaultassembler,'default',idNone);
     //AddSelectItem(opt_standardassembler,'standard',idAsmStandard);
     AddSelectItem(opt_motassembler,'motorola',idAsmMot);
{$endif M68K}
   end;
  New(AsmInfoSwitches,Init('a'));
  with AsmInfoSwitches^ do
   begin
     AddBooleanItem(opt_listsource,'l',idNone);
     AddBooleanItem(opt_listregisterallocation,'r',idNone);
     AddBooleanItem(opt_listtempallocation,'t',idNone);
     AddBooleanItem(opt_listnodeallocation,'n',idNone);
     AddBooleanItem(opt_useasmpipe,'p',idNone);
   end;
  UpdateAsmOutputSwitches;
  New(BrowserSwitches,InitSelect('b'));
  with BrowserSwitches^ do
   begin
     AddSelectItem(opt_nobrowser,'-',idSymInfNone);
     AddSelectItem(opt_globalonlybrowser,'+',idSymInfGlobalOnly);
     AddSelectItem(opt_localglobalbrowser,'l',idSymInfGlobalLocal);
   end;
  New(ConditionalSwitches,Init('d'));
  with ConditionalSwitches^ do
   begin
     AddStringItem(opt_conditionaldefines,'',idNone,true,false);
   end;
  New(MemorySwitches,Init('C'));
  with MemorySwitches^ do
   begin
     AddLongintItem(opt_stacksize,'s',idStackSize);
     AddLongintItem(opt_heapsize,'h',idHeapSize);
   end;
  New(DirectorySwitches,Init('F'));
  with DirectorySwitches^ do
   begin
     AddMultiStringItem(opt_unitdirectories,'u',idNone);
     AddMultiStringItem(opt_includedirectories,'i',idNone);
     AddMultiStringItem(opt_librarydirectories,'l',idNone);
     AddMultiStringItem(opt_objectdirectories,'o',idNone);
     AddStringItem(opt_exeppudirectories,'E',idNone,true,true);
     AddStringItem(opt_ppuoutputdirectory,'U',idNone,true,true);
     AddStringItem(opt_cross_tools_directory,'D',idNone,true,true);
     AddStringItem(opt_dynamic_linker,'L',idNone,false,false);
   end;

  New(LibLinkerSwitches,InitSelect('X'));
  with LibLinkerSwitches^ do
   begin
     AddDefaultSelect(opt_librariesdefault);
     AddSelectItem(opt_dynamiclibraries,'D',idNone);
     AddSelectItem(opt_staticlibraries,'S',idNone);
     AddSelectItem(opt_smartlibraries,'X',idNone);
   end;
  New(OtherLinkerSwitches,Init('X'));
  with OtherLinkerSwitches^ do
   begin
     AddBooleanItem(opt_stripalldebugsymbols,'s',idNone);
     AddBooleanItem(opt_forcestaticlibs,'t',idNone);
   end;
  New(DebugInfoSwitches,InitSelect('g'));
  with DebugInfoSwitches^ do
   begin
     AddSelectItem(opt_nogendebugsymbolinfo,'-',idNone);
     AddSelectItem(opt_gendebugsymbolinfo,'',idNone);
     AddSelectItem(opt_gensymbolandbacktraceinfo,'l',idNone);
     AddSelectItem(opt_valgrindinfo,'v',idNone);
     { AddSelectItem('Generate ~d~bx symbol information','d');
       does not work anyhow (PM) }
   end;
  New(LinkAfterSwitches,Init('s'));
  LinkAfterSwitches^.AddBooleanItem(opt_linkafter,'',idNone);
  New(ProfileInfoSwitches,InitSelect('p'));
  with ProfileInfoSwitches^ do
   begin
     AddSelectItem(opt_noprofileinfo,'-',idNone);
     AddSelectItem(opt_gprofinfo,'g',idNone);
   end;
  {New(MemorySizeSwitches,Init('C'));
  with MemorySizeSwitches^ do
   begin
     AddLongIntItem('~S~tack size','s');
     AddLongIntItem('Local ~h~eap size','h');
   end;}
  SwitchesPath:=LocateFile(SwitchesName);
  if SwitchesPath='' then
    SwitchesPath:=SwitchesName;
  SwitchesPath:=FExpand(SwitchesPath);
end;

procedure SetDefaultSwitches;
var
   i,OldSwitchesMode : TSwitchMode;

begin
  { setup some useful defaults }
  OldSwitchesMode:=SwitchesMode;
  for i:=low(TSwitchMode) to high(TSwitchMode) do
    begin
       SwitchesMode:=i;

       { default is Pentium }
       ProcessorOptimizationSwitches^.SetCurrSel(1);
       { AT&T reader }
       AsmReaderSwitches^.SetCurrSel(1);

       { FPC mode}
       CompilerModeSwitches^.SetCurrSel(0);
(* Use platform defaults for memory switches. *)
       { 128k stack }
{       MemorySwitches^.SetLongintItem(0,65536*2);}
       MemorySwitches^.SetLongintItem(0,0);
       { 2 MB heap }
{       MemorySwitches^.SetLongintItem(1,1024*1024*2);}
       MemorySwitches^.SetLongintItem(1,0);
       { goto/lable allowed }
       SyntaxSwitches^.SetBooleanItem(1,true);
       { inline allowed }
       SyntaxSwitches^.SetBooleanItem(3,true);
       { Exe size complaints are louder than speed complaints: Optimize for size by default. }
       OptimizationSwitches^.SetBooleanItem(0,true);
       case i of
          om_debug:
            begin
               { debugging info on }
               DebugInfoSwitches^.SetCurrSel(1);
               { range checking }
               CodegenSwitches^.SetBooleanItem(0,true);
               { io checking }
               CodegenSwitches^.SetBooleanItem(2,true);
               { overflow checking }
               CodegenSwitches^.SetBooleanItem(3,true);
               { method call checking }
               CodegenSwitches^.SetBooleanItem(4,true);
               { assertions on }
               SyntaxSwitches^.SetBooleanItem(4,true);
            end;
          om_normal:
            begin
               {Register variables.}
               OptimizationSwitches^.SetBooleanItem(1,true);
               {Level 1 optimizations.}
               OptimizationSwitches^.SetBooleanItem(3,true);
            end;
          om_release:
            begin
               {Register variables.}
               OptimizationSwitches^.SetBooleanItem(1,true);
               {Level 2 optimizations.}
               OptimizationSwitches^.SetBooleanItem(4,true);
               {Smart linking.}
               LibLinkerSwitches^.SetCurrSel(3);
               CodegenSwitches^.SetBooleanItem(6,true);
               {Strip debug info}
               OtherLinkerSwitches^.SetBooleanItem(0,true);
            end;
       end;
       { set appriopriate default target }
       TargetSwitches^.SetCurrSelParam(target_info.shortname);
    end;
  SwitchesMode:=OldSwitchesMode;
end;

procedure DoneSwitches;

begin
  dispose(SyntaxSwitches,Done);
  dispose(CompilerModeSwitches,Done);
  dispose(VerboseSwitches,Done);
  dispose(CodegenSwitches,Done);
  dispose(OptimizationSwitches,Done);
  dispose(ProcessorOptimizationSwitches,Done);
  dispose(ProcessorCodeGenerationSwitches,Done);
  dispose(BrowserSwitches,Done);
  dispose(TargetSwitches,Done);
  dispose(AsmReaderSwitches,Done);
  dispose(AsmOutputSwitches,Done);
  dispose(AsmInfoSwitches,Done);
  dispose(ConditionalSwitches,Done);
  dispose(MemorySwitches,Done);
  {dispose(MemorySizeSwitches,Done);}
  dispose(DirectorySwitches,Done);
  dispose(DebugInfoSwitches,Done);
  dispose(LibLinkerSwitches,Done);
  dispose(LinkAfterSwitches,Done);
  dispose(OtherLinkerSwitches,Done);
  dispose(ProfileInfoSwitches,Done);
end;

procedure GetCompilerOptionLines(C: PUnsortedStringCollection);
procedure AddLine(const S: string);
begin
  C^.Insert(NewStr(S));
end;
procedure ConstructSwitchModeDirectives(SM: TSwitchMode; const IfDefSym: string);
var SwitchParams: PStringCollection;
    MiscParams  : PStringCollection;
procedure AddSwitch(const S: string);
begin
  SwitchParams^.Insert(NewStr(S));
end;
procedure AddParam(const S: string);
begin
  MiscParams^.Insert(NewStr(S));
end;

procedure EnumSwitches(P: PSwitches);

procedure HandleSwitch(P: PSwitchItem);
begin
  case P^.ParamID of
{    idAlign :}
    idRangeChecks    : AddSwitch('R'+P^.GetSwitchStr(SM));
    idStackChecks    : AddSwitch('S'+P^.GetSwitchStr(SM));
    idIOChecks       : AddSwitch('I'+P^.GetSwitchStr(SM));
    idOverflowChecks : AddSwitch('Q'+P^.GetSwitchStr(SM));
    idObjMethCallChecks: AddSwitch('OBJECTCHECKS'+P^.GetSwitchStr(SM));
{    idAsmDirect      : if P^.GetParamValueBool[SM] then AddParam('ASMMODE DIRECT');
    idAsmATT         : if P^.GetParamValueBool[SM] then AddParam('ASMMODE ATT');
    idAsmIntel       : if P^.GetParamValueBool[SM] then AddParam('ASMMODE INTEL');
    idAsmMot         : if P^.GetParamValueBool[SM] then AddParam('ASMMODE MOTOROLA');
    idAsmStandard    : if P^.GetParamValueBool[SM] then AddParam('ASMMODE STANDARD');}
{    idSymInfNone     : ;
    idSymInfGlobalOnly:;
    idSymInfGlobalLocal:if P^.ParamValueBool(SM) then AddSwitch('L+');}
{    idStackSize
    idHeapSize}
    idStrictVarStrings: AddSwitch('V'+P^.GetSwitchStr(SM));
    idExtendedSyntax  : AddSwitch('X'+P^.GetSwitchStr(SM));
    idMMXOps          : if P^.ParamValueBool(SM) then AddParam('MMX');
    idTypedAddress    : AddSwitch('T'+P^.GetSwitchStr(SM));
{    idPackRecords
    idPackEnum}
    idStackFrames     : AddSwitch('W'+P^.GetSwitchStr(SM));
    idReferenceInfo   : AddSwitch('Y'+P^.GetSwitchStr(SM));
    idDebugInfo       : AddSwitch('D'+P^.GetSwitchStr(SM));
    idBoolEval        : AddSwitch('B'+P^.GetSwitchStr(SM));
    idAnsiString      : AddSwitch('H'+P^.GetSwitchStr(SM));
    idTypeInfo        : AddSwitch('M'+P^.GetSwitchStr(SM));
   end;
end;
begin
  P^.Items^.ForEach(@HandleSwitch);
end;
var I: integer;
    S: string;
begin
  AddLine('{$IFDEF '+IfDefSym+'}');
  New(SwitchParams, Init(10,10));
  New(MiscParams, Init(10,10));
  EnumSwitches(LibLinkerSwitches);
  EnumSwitches(OtherLinkerSwitches);
  EnumSwitches(DebugInfoSwitches);
  EnumSwitches(ProfileInfoSwitches);
  EnumSwitches(SyntaxSwitches);
  EnumSwitches(CompilerModeSwitches);
  EnumSwitches(VerboseSwitches);
  EnumSwitches(CodegenSwitches);
  EnumSwitches(OptimizationSwitches);
  EnumSwitches(ProcessorOptimizationSwitches);
  EnumSwitches(ProcessorCodeGenerationSwitches);
  EnumSwitches(AsmReaderSwitches);
  EnumSwitches(AsmInfoSwitches);
  EnumSwitches(AsmOutputSwitches);
  EnumSwitches(TargetSwitches);
  EnumSwitches(ConditionalSwitches);
  EnumSwitches(MemorySwitches);
  EnumSwitches(BrowserSwitches);
  EnumSwitches(DirectorySwitches);
  S:='';
  for I:=0 to SwitchParams^.Count-1 do
  begin
    if I=0 then S:='{$' else S:=S+',';
    S:=S+PString(SwitchParams^.At(I))^;
  end;
  if S<>'' then S:=S+'}';
  if S<>'' then AddLine('  '+S);
  for I:=0 to MiscParams^.Count-1 do
    AddLine('  {$'+PString(MiscParams^.At(I))^+'}');
  Dispose(SwitchParams, Done); Dispose(MiscParams, Done);
  AddLine('{$ENDIF '+IfDefSym+'}');
end;
var SM: TSwitchMode;
begin
  for SM:=Low(TSwitchMode) to High(TSwitchMode) do
    ConstructSwitchModeDirectives(SM,SwitchesModeStr[SM]);
end;


end.
