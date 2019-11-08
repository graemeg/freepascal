{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002-2004 by Olle Raab

    FreePascal system unit for MacOS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System;

interface

{$define FPC_IS_SYSTEM}
{$DEFINE FPC_ANSI_TEXTFILEREC}

{ include system-independent routine headers }
{$I systemh.inc}

{$if defined(cpum68k) and defined(fpusoft)}
{$define fpc_softfpu_interface}
{$i softfpu.pp}
{$undef fpc_softfpu_interface}
{$endif defined(cpum68k) and defined(fpusoft)}

const
 LineEnding = #13;
 LFNSupport = true;
 DirectorySeparator = ':';
 DriveSeparator = ':';
 ExtensionSeparator = '.';
 PathSeparator = ',';  {Is used in MPW and OzTeX}
 AllowDirectorySeparators : set of char = [':'];
 AllowDriveSeparators : set of char = [':'];
 FileNameCaseSensitive = false;
 FileNameCasePreserving = true;
 CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)

 maxExitCode = 65535;
 MaxPathLen = 256;
 AllFilesMask = '*';

const
{ Default filehandles }
  UnusedHandle    : Longint = -1;
  StdInputHandle  : Longint = 0;
  StdOutputHandle : Longint = 1;
  StdErrorHandle  : Longint = 2;

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCR;



var
  argc : longint;
  argv : ppchar;
  envp : ppchar;

{*********************************}
{**  MacOS specific functions    **}
{*********************************}

{To be called at regular intervals, for lenghty tasks.
 Yield might give time for other tasks to run under the cooperative
 multitasked macos. For an MPW Tool, it also spinns the cursor.}

procedure Yield;

{To set mac file type and creator codes, to be used for files created
 by the FPC runtime library. They must be exactly 4 chars long.}

procedure SetDefaultMacOSFiletype(ftype: ShortString);
procedure SetDefaultMacOSCreator(creator: ShortString);

var
  {Whether unix and dos style paths should be translated. Default false}
  pathTranslation: Boolean;


{*********************************}
{**  Available features on macos **}
{*********************************}


  var
    macosHasGestalt: Boolean;
    macosHasWaitNextEvent: Boolean;
    macosHasColorQD: Boolean;
    macosHasFPU: Boolean;
    macosSystemVersion: Integer;
    macosHasSysDebugger: Boolean = false;
    macosHasCFM: Boolean;

    macosHasAppleEvents: Boolean;
    macosHasAliasMgr: Boolean;


    macosHasFSSpec: Boolean;
    macosHasFindFolder: Boolean;


    macosHasScriptMgr: Boolean;
    macosNrOfScriptsInstalled: Integer;

    macosHasAppearance: Boolean;
    macosHasAppearance101: Boolean;
    macosHasAppearance11: Boolean;

    macosBootVolumeVRefNum: Integer;
    macosBootVolumeName: String[31];

{
 MacOS paths
 ===========
 MacOS directory separator is a colon ":" which is the only character not
 allowed in filenames.
 A path containing no colon or which begins with a colon is a partial path.
 E g ":kalle:petter" ":kalle" "kalle"
 All other paths are full (absolute) paths. E g "HD:kalle:" "HD:"
 When generating paths, one is safe is one ensures that all partial paths
 begins with a colon, and all full paths ends with a colon.
 In full paths the first name (e g HD above) is the name of a mounted volume.
 These names are not unique, because, for instance, two diskettes with the
 same names could be inserted. This means that paths on MacOS is not
 waterproof. In case of equal names the first volume found will do.
 Two colons "::" are the relative path to the parent. Three is to the
 grandparent etc.
}

implementation

{
About the implementation
========================
A MacOS application is assembled and linked by MPW (Macintosh
Programmers Workshop), which nowadays is free to use. For info
and download of MPW and MacOS api, see www.apple.com

It can be linked to either a graphical user interface application,
a standalone text only application (using SIOW) or
to an MPW tool, this is entirely controlled by the linking step.

It requires system 7 and CFM, which is always the case for PowerPC.

If a m68k version would be implemented, it would save a lot
of efforts if it also uses CFM. This System.pp should, with
minor modifications, probably work with m68k.

Initial working directory is the directory of the application,
or for an MPWTool, the working directory as set by the
Directory command in MPW.

Note about working directory. There is a facility in MacOS which
manages a working directory for an application, initially set to
the applications directory, or for an MPWTool, the tool's directory.
However, this requires the application to have a unique application
signature (creator code), to distinguish its working directory
from working directories of other applications. Due to the fact
that casual applications are anonymous in this sense (without an
application signature), this facility will not work. Also, this
working directory facility is not present in Carbon. Hence we
will manage a working directory by our self.


Deviations
==========

In current implementation, working directory is stored as
directory id. This means there is a possibility the user moves the
working directory or a parent to it, while the application uses it.
Then the path to the wd suddenly changes. This is AFAIK not in
accordance with other OS's. Although this is a minor caveat,
it is mentioned here. To overcome this the wd could be stored
as a path instead, but this imposes translations from fullpath
to directory ID each time the filesystem is accessed.

The initial working directory for an MPWTool, as considered by
FPC, is different from the MacOS working directory facility,
see above.


Possible improvements:
=====================

Perhaps handle readonly filesystems, as in sysunix.inc

}

{$if defined(cpum68k) and defined(fpusoft)}

{$define fpc_softfpu_implementation}
{$define softfpu_compiler_mul32to64}
{$define softfpu_inline}
{$i softfpu.pp}
{$undef fpc_softfpu_implementation}

{ we get these functions and types from the softfpu code }
{$define FPC_SYSTEM_HAS_float64}
{$define FPC_SYSTEM_HAS_float32}
{$define FPC_SYSTEM_HAS_flag}
{$define FPC_SYSTEM_HAS_extractFloat64Frac0}
{$define FPC_SYSTEM_HAS_extractFloat64Frac1}
{$define FPC_SYSTEM_HAS_extractFloat64Exp}
{$define FPC_SYSTEM_HAS_extractFloat64Sign}
{$define FPC_SYSTEM_HAS_ExtractFloat32Frac}
{$define FPC_SYSTEM_HAS_extractFloat32Exp}
{$define FPC_SYSTEM_HAS_extractFloat32Sign}
{$endif defined(cpum68k) and defined(fpusoft)}


{******** include system independent routines **********}
{$I system.inc}


{*****************************************************************************
                              ParamStr/Randomize
*****************************************************************************}

{ number of args }
function paramcount : longint;
begin
  paramcount := argc - 1;
  //paramcount:=0;
end;

{ argument number l }
function paramstr(l : longint) : string;
begin
  if (l>=0) and (l+1<=argc) then
    paramstr:=strpas(argv[l])
  else
    paramstr:='';
end;

{ set randseed to a new pseudo random value }
procedure randomize;
begin
  randseed:= Cardinal(TickCount);
end;


{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

{$ifndef FPC_DARWIN_PASCALMAIN}
procedure pascalmain; external name 'PASCALMAIN';

{Main entry point in C style, needed to capture program parameters.
 For this to work, the system unit must be before the main program
 in the linking order.}
procedure main(argcparam: Longint; argvparam: ppchar; envpparam: ppchar); cdecl; [public];
{$else FPC_DARWIN_PASCALMAIN}
procedure FPC_SYSTEMMAIN(argcparam: Longint; argvparam: ppchar; envpparam: ppchar); cdecl; [public];
{$endif FPC_DARWIN_PASCALMAIN}

begin
  argc:= argcparam;
  argv:= argvparam;
  envp:= envpparam;
{$ifndef FPC_DARWIN_PASCALMAIN}
  pascalmain;  {run the pascal main program}
{$endif FPC_DARWIN_PASCALMAIN}
end;

procedure setup_arguments;
         begin
           {Nothing needs to be done here.}
         end;

procedure setup_environment;
         begin
         end;


{ FindSysFolder returns the (real) vRefNum, and the DirID of the current
system folder. It uses the Folder Manager if present, otherwise it falls
back to SysEnvirons. It returns zero on success, otherwise a standard
system error. }

function FindSysFolder(var foundVRefNum: Integer; var foundDirID: Longint): OSErr;

var
  gesResponse: Longint;
  envRec: SysEnvRec;
  myWDPB: WDPBRec;
  volName: String[34];
  err: OSErr;

begin
  foundVRefNum := 0;
  foundDirID := 0;
  if  macosHasGestalt
      and (Gestalt (FourCharCodeToLongword(gestaltFindFolderAttr), gesResponse) = noErr)
      and BitIsSet (gesResponse, gestaltFindFolderPresent) then
    begin { Does Folder Manager exist? }
       err := FindFolder (kOnSystemDisk, FourCharCodeToLongword(kSystemFolderType),
        kDontCreateFolder, foundVRefNum, foundDirID);
    end
  else
    begin
      { Gestalt can't give us the answer, so we resort to SysEnvirons }
      err := SysEnvirons (curSysEnvVers, envRec);
      if (err = noErr) then
        begin
          myWDPB.ioVRefNum := envRec.sysVRefNum;
          volName := '';
          myWDPB.ioNamePtr := @volName;
          myWDPB.ioWDIndex := 0;
          myWDPB.ioWDProcID := 0;
          err := PBGetWDInfoSync (@myWDPB);
          if (err = noErr) then
            begin
              foundVRefNum := myWDPB.ioWDVRefNum;
              foundDirID := myWDPB.ioWDDirID;
            end;
          end;
        end;
  FindSysFolder:= err;
end;

{$ifdef CPUM68K}
{$WARNING FIXME: Dummy TrapAvailable!}
function TrapAvailable(_trap: Word): Boolean;
begin
  TrapAvailable:=false;
end;
{$endif CPUM68K}

procedure InvestigateSystem;

  {$IFDEF CPUM68K}
  const
    _GestaltDispatch = $A0AD;
    _WaitNextEvent = $A860;
    _ScriptUtil = $A8B5;

    qdOffscreenTrap = $AB1D;
  {$ENDIF}

  var
    err: Integer;
    response: Longint;
    {$IFDEF CPUM68K}
    environs: SysEnvRec;
    {$ENDIF}

  {Vi rŠknar med att man kšr pŒ minst system 6.0.5.  DŒ finns bŒde Gestalt och GDevice med.}
  {Enligt Change Histrory Šr MacOS 6.0.5 mera konsistent mellan maskinmodellerna Šn fšregŒende system}

begin
  {$IFDEF CPUM68K}
  macosHasGestalt := TrapAvailable(_GestaltDispatch);
  {$ELSE}
  macosHasGestalt := true;  {There is always Gestalt on PowerPC}
  {$ENDIF}

  if not macosHasGestalt then    (* If we don't have Gestalt, then we can't have any System 7 features  *)
    begin
      {$IFDEF CPUM68K}
      {      Detta kan endast gŠlla pŒ en 68K maskin.}
      macosHasScriptMgr := TrapAvailable(_ScriptUtil);

      macosNrOfScriptsInstalled := 1; (* assume only Roman script, to start with  *)

      err := SysEnvirons(1, environs);
      if err = noErr then
        begin
          if environs.machineType < 0 then       { gammalt ROM}
            macosHasWaitNextEvent := FALSE
          else
            macosHasWaitNextEvent := TrapAvailable(_WaitNextEvent);
          macosHasColorQD := environs.hasColorQD;
          macosHasFPU := environs.hasFPU;
          macosSystemVersion := environs.systemVersion;
        end
      else
        begin
          macosHasWaitNextEvent := FALSE;
          macosHasColorQD := FALSE;
          macosHasFPU := FALSE;
          macosSystemVersion := 0;
        end;

      {$WARNING FIXME: MacJmp}
      //macosHasSysDebugger := (LongintPtr(MacJmp)^ <> 0);
      macosHasSysDebugger := false;

      macosHasCFM := false;
      macosHasAppleEvents := false;
      macosHasAliasMgr := false;

      macosHasFSSpec := false;
      macosHasFindFolder := false;

      macosHasAppearance := false;
      macosHasAppearance101 := false;
      macosHasAppearance11 := false;
      {$IFDEF THINK_PASCAL}
      if (macosHasScriptMgr) then
        macosNrOfScriptsInstalled := GetEnvirons(smEnabled);
      {$ELSE}
      {$WARNING FIXME: GetScriptManagerVariable and smEnabled}
      //if (macosHasScriptMgr) then
      //  macosNrOfScriptsInstalled := GetScriptManagerVariable(smEnabled);  {Gamla rutinnamnet var GetEnvirons.}
      {$ENDIF}
      {$ENDIF CPUM68K}
    end
  else
    begin
      macosHasScriptMgr := Gestalt(FourCharCodeToLongword(gestaltScriptMgrVersion), response) = noErr;  {Fšr att ta reda pŒ om script mgr finns.}
      macosNrOfScriptsInstalled := 1; (* assume only Roman script, to start with  *)
      macosHasWaitNextEvent := true;

      if Gestalt(FourCharCodeToLongword(gestaltSystemVersion), response) = noErr then
        macosSystemVersion := response
      else
        macosSystemVersion := 0;  {Borde inte kunna hŠnda.}

      if Gestalt(FourCharCodeToLongword(gestaltOSAttr), response) = noErr then
        macosHasSysDebugger := BitIsSet(response, gestaltSysDebuggerSupport)
      else
        macosHasSysDebugger := false;

      if Gestalt(FourCharCodeToLongword(gestaltQuickdrawVersion), response) = noErr then
        macosHasColorQD := (response >= $0100)
      else
        macosHasColorQD := false;

      if Gestalt(FourCharCodeToLongword(gestaltFPUType), response) = noErr then
        macosHasFPU := (response <> gestaltNoFPU)
      else
        macosHasFPU := false;

      if Gestalt(FourCharCodeToLongword(gestaltCFMAttr), response) = noErr then
        macosHasCFM := BitIsSet(response, gestaltCFMPresent)
      else
        macosHasCFM := false;

      macosHasAppleEvents := Gestalt(FourCharCodeToLongword(gestaltAppleEventsAttr), response) = noErr;
      macosHasAliasMgr := Gestalt(FourCharCodeToLongword(gestaltAliasMgrAttr), response) = noErr;

      if Gestalt(FourCharCodeToLongword(gestaltFSAttr), response) = noErr then
        macosHasFSSpec := BitIsSet(response, gestaltHasFSSpecCalls)
      else
        macosHasFSSpec := false;
      macosHasFindFolder := Gestalt(FourCharCodeToLongword(gestaltFindFolderAttr), response) = noErr;

      if macosHasScriptMgr then
        begin
          err := Gestalt(FourCharCodeToLongword(gestaltScriptCount), response);
          if (err = noErr) then
            macosNrOfScriptsInstalled := Integer(response);
        end;

      if (Gestalt(FourCharCodeToLongword(gestaltAppearanceAttr), response) = noErr) then
        begin
          macosHasAppearance := BitIsSet(response, gestaltAppearanceExists);
          if Gestalt(FourCharCodeToLongword(gestaltAppearanceVersion), response) = noErr then
            begin
              macosHasAppearance101 := (response >= $101);
              macosHasAppearance11 := (response >= $110);
            end
        end
      else
        begin
          macosHasAppearance := false;
          macosHasAppearance101 := false;
          macosHasAppearance11 := false;
        end;
    end;
end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

Procedure system_exit;
var
  s: ShortString;
begin
  if StandAlone <> 0 then
    if exitcode <> 0 then
        begin
          Str(exitcode,s);
          if IsConsole then
            Writeln( '### Program exited with exit code ' + s)
          else if macosHasSysDebugger then
            DebugStr('A possible error occurred, exit code: ' + s + '. Type "g" and return to continue.')
          else
            {Be quiet}
        end;

  {$ifndef MACOS_USE_STDCLIB}
  if StandAlone <> 0 then
    ExitToShell;
  {$else}
  c_exit(exitcode); {exitcode is only utilized by an MPW tool}
  {$endif}
end;

procedure SysInitStdIO;
begin
  { Setup stdin, stdout and stderr }
  {$ifdef MACOS_USE_STDCLIB}
     OpenStdIO(Input,fmInput,StdInputHandle);
     OpenStdIO(Output,fmOutput,StdOutputHandle);
     OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
     OpenStdIO(StdOut,fmOutput,StdOutputHandle);
     OpenStdIO(StdErr,fmOutput,StdErrorHandle);
  {$endif }
end;

function GetProcessID: SizeUInt;
begin
 GetProcessID := 1;
{$WARNING To be implemented - using GetProcessInformation???}
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

var
  resHdl: Mac_Handle;
  isFolder, hadAlias, leafIsAlias: Boolean;
  dirStr: string[2];
  err: OSErr;
  dummySysFolderDirID: Longint;

begin
  InvestigateSystem; {Must be first}

  {Check requred features for system.pp to work.}
  if not macosHasFSSpec then
    Halt(3);  //exit code 3 according to MPW

  if FindSysFolder(macosBootVolumeVRefNum, dummySysFolderDirID) <> noErr then
    Halt(3);  //exit code 3 according to MPW

  if GetVolumeName(macosBootVolumeVRefNum, macosBootVolumeName) <> noErr then
    Halt(3);  //exit code 3 according to MPW

  { To be set if this is a GUI or console application }
  if StandAlone = 0 then
    IsConsole := true {Its an MPW tool}
  else
    begin
      resHdl:= Get1Resource(FourCharCodeToLongword('siow'),0);
      IsConsole := (resHdl <> nil); {A SIOW app is also a console}
      ReleaseResource(resHdl);
    end;

  { To be set if this is a library and not a program  }
  IsLibrary := FALSE;

  StackLength := CheckInitialStkLen(InitialStkLen);
  StackBottom := SPtr - StackLength;
  pathTranslation:= false;

  { Setup working directory }
  if StandAlone <> 0 then
    begin
      if not GetAppFileLocation(workingDirectorySpec) then
        Halt(3);  //exit code 3 according to MPW
    end
  else
    begin
      { The fictive file x is used to make
        FSMakeFSSpec return a FSSpec to a file in the directory.
        Then by clearing the name, the FSSpec then
        points to the directory. It doesn't matter whether x exists or not.}
      dirStr:= ':x';
      err:= ResolveFolderAliases(0, 0, @dirStr, true,
           workingDirectorySpec, isFolder, hadAlias, leafIsAlias);
      workingDirectorySpec.name:='';
      if (err <> noErr) and (err <> fnfErr) then
        Halt(3);  //exit code 3 according to MPW
    end;

  { Setup heap }
  if StandAlone <> 0 then
    MaxApplZone;

  InitHeap;
  SysInitExceptions;
  initunicodestringmanager;
  SysInitStdIO;

  { Setup environment and arguments }
  Setup_Environment;
  setup_arguments;
  { Reset IO Error }
  InOutRes:=0;
  errno:=0;
  InitSystemThreads;

  if StandAlone = 0 then
    begin
      InitGraf(@qd.thePort);
      SetFScaleDisable(true);
      InitCursorCtl(nil);
    end;
end.
