{
    Copyright (c) 2001-2002 by Peter Vreman

    This unit implements support import,export,link routines for MacOS.

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
unit t_macos;

{$i fpcdefs.inc}

interface

  uses
     import,symsym,symdef,link;

  type
    timportlibmacos=class(timportlib)
      procedure generatelib;override;
    end;

    tlinkermpw=class(texternallinker)
    private
      Function  WriteResponseFile(isdll:boolean) : Boolean;
    public
      constructor Create;override;
      procedure SetDefaultInfo;override;
      function  MakeExecutable:boolean;override;
    end;

    { used for crosscompiling, depends on Retro68 GNU binutils }
    TLinkerMacOS = class(texternallinker)
    private
      function WriteResponseFile(isdll: boolean): boolean;
      procedure SetMacOS68kInfo;
      function MakeMacOSExe: boolean;
    public
      constructor Create; override;
      procedure SetDefaultInfo; override;
      {procedure InitSysInitUnitName; override;}
      function  MakeExecutable: boolean; override;
    end;


implementation

    uses
       SysUtils,
       cutils,cfileutl,cclasses,aasmbase,
       globtype,globals,systems,verbose,cscript,fmodule,i_macos,
       ogbase,
       symconst;

{*****************************************************************************
                               TIMPORTLIBMACOS
*****************************************************************************}

    procedure timportlibmacos.generatelib;
      var
        i : longint;
        ImportLibrary : TImportLibrary;
      begin
        for i:=0 to current_module.ImportLibraryList.Count-1 do
          begin
            ImportLibrary:=TImportLibrary(current_module.ImportLibraryList[i]);
            current_module.linkothersharedlibs.add(ImportLibrary.Name,link_always);
          end;
      end;

{*****************************************************************************
                                  TLINKERMPW
*****************************************************************************}

Constructor TLinkerMPW.Create;
begin
  Inherited Create;
  //LibrarySearchPath.AddLibraryPath(sysrootpath,'=/lib;=/usr/lib;=/usr/X11R6/lib',true);
end;


procedure TLinkerMPW.SetDefaultInfo;

begin
  with Info do
   begin
     ExeCmd[1]:='Execute $RES'; {The link.res file contains the whole link command.}
     //ExeCmd[1]:='PPCLink $OPT $DYNLINK $STATIC $STRIP -tocdataref off -dead on -o $EXE -@filelist $RES';
     //DllCmd[1]:='PPCLink $OPT $INIT $FINI $SONAME -shared -o $EXE -@filelist $RES';
   end;
end;


Function TLinkerMPW.WriteResponseFile(isdll:boolean) : Boolean;
Var
  linkres      : TLinkRes;
  s,heapsizestr: string;

begin
  WriteResponseFile:=False;
  { Open link.res file }
  linkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);

  with linkRes do
    begin
      {#182 is escape char in MPW (analog to backslash in unix). The space}
      {ensures there is whitespace separating items.}
      Add('PPCLink '#182);

      { Add MPW standard libraries}
      if apptype = app_cui then
          Add('"{PPCLibraries}PPCSIOW.o" '#182);

      {Even GUI apps must link to PPCToolLibs, because of the System unit
       which can be used by MPW tools as well as by GUI apps.}
      Add('"{PPCLibraries}PPCToolLibs.o" '#182);
      Add('"{SharedLibraries}InterfaceLib" '#182);
      Add('"{SharedLibraries}StdCLib" '#182);
      Add('"{SharedLibraries}MathLib" '#182);
      Add('"{PPCLibraries}StdCRuntime.o" '#182);
      Add('"{PPCLibraries}PPCCRuntime.o" '#182);

      {Add main objectfiles}
      while not ObjectFiles.Empty do
        begin
          s:=ObjectFiles.GetFirst;
          if s<>'' then
            Add(s+' '#182);
        end;

      {Add last lines of the link command}
      if apptype = app_tool then
        Add('-t "MPST" -c "MPS " '#182);

      if apptype = app_cui then {If SIOW, to avoid some warnings.}
        Add('-ignoredups __start -ignoredups .__start -ignoredups main -ignoredups .main -ignoredups qd '#182);

      Add('-tocdataref off -sym on -dead on -o '+ ScriptFixFileName(current_module.exefilename));

      Add('Exit If "{Status}" != 0');

      if heapsize = 0 then
        heapsizestr:= HexStr(384000, 8)
      else
        heapsizestr:= HexStr(heapsize, 8);

      {Add a SIZE resource on the fly. It controls:
         * backgrounding is enabled, to facilitate debuging with Power Mac Debugger
         * it is signaled it is a 32 bit app. (perhaps not nessecary on PowerPC)
         * heapsize  }
      if apptype <> app_tool then
        begin
          Add('Echo "data ''SIZE'' (-1) '#182'{ $'#182'"1080 ' + heapsizestr + ' ' + heapsizestr +
                                         #182'" '#182'};" | Rez -a -o ' + ScriptFixFileName(current_module.exefilename));
          Add('Exit If "{Status}" != 0');
        end;

      {Add mac resources}
      if apptype = app_cui then
        begin
          Add('Rez -a "{RIncludes}"SIOW.r -o ' + ScriptFixFileName(current_module.exefilename));
          Add('Exit If "{Status}" != 0');
        end;

      while not (current_module.ResourceFiles.Empty) do
        begin
          s := Current_module.ResourceFiles.GetFirst;
          if Copy(s,Length(s)-1,Length(s)) = '.r' then
            Add('Rez -a ' + s + ' -o ' + ScriptFixFileName(current_module.exefilename))
          else
            Add('DeRez ' + s + ' | Rez -a -o ' + ScriptFixFileName(current_module.exefilename));
          Add('Exit If "{Status}" != 0');
        end;

    end;

  { Write and Close response }
  linkres.writetodisk;
  linkres.Free;

  WriteResponseFile:=True;
end;


function TLinkerMPW.MakeExecutable:boolean;
var
  binstr,
  cmdstr  : TCmdStr;
  success : boolean;
  DynLinkStr : string[60];
  StaticStr,
  StripStr   : string[40];
begin
  //TODO Only external link in MPW is possible, otherwise yell.

  if not(cs_link_nolink in current_settings.globalswitches) then
    Message1(exec_i_linking,current_module.exefilename);

{ Create some replacements }
  StripStr:='';
  StaticStr:='';
  DynLinkStr:='';
(*
  if (cs_link_staticflag in current_settings.globalswitches) then
   StaticStr:='-static';
  if (cs_link_strip in current_settings.globalswitches) then
   StripStr:='-s';
  If (cs_profile in current_settings.moduleswitches) or
     ((Info.DynamicLinker<>'') and (not SharedLibFiles.Empty)) then
   DynLinkStr:='-dynamic-linker='+Info.DynamicLinker;
*)

{ Prepare linking }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(ScriptFixFileName(current_module.exefilename)));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$RES',maybequoted(ScriptFixFileName(outputexedir+Info.ResName)));
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);

        WriteResponseFile(false);

        success:= true;
        if cs_link_on_target in current_settings.globalswitches then
                success:=DoExec('SetFile', ' -c ''MPS '' -t ''TEXT'' ' +
                                                                 ScriptFixFileName(outputexedir+Info.ResName),true,false);

{ Call linker }
        if success then
                success:=DoExec('Execute',CmdStr,true,false);

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    DeleteFile(outputexedir+Info.ResName);

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


{*****************************************************************************
                                TLINKERMACOS
*****************************************************************************}

constructor TLinkerMacOS.Create;
begin
  Inherited Create;
  { allow duplicated libs (PM) }
  SharedLibFiles.doubles:=true;
  StaticLibFiles.doubles:=true;
end;


procedure TLinkerMacOS.SetMacOS68kInfo;
begin
  with Info do
    begin
      ExeCmd[1]:='ld $DYNLINK $GCSECTIONS $OPT -d -n -o $EXE $RES';
    end
end;


procedure TLinkerMacOS.SetDefaultInfo;
begin
  case (target_info.system) of
    system_m68k_macos:      SetMacOS68kInfo;
    else
      ;
  end;
end;


{procedure TLinkerMacOS.InitSysInitUnitName;
begin
  sysinitunit:='si_prc';
end;}


function TLinkerMacOS.WriteResponseFile(isdll: boolean): boolean;
var
  linkres  : TLinkRes;
  i        : longint;
  HPath    : TCmdStrListItem;
  s        : string;
  linklibc : boolean;
begin
  WriteResponseFile:=False;

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);

  { Write path to search libraries }
  HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
  while assigned(HPath) do
   begin
    s:=HPath.Str;
    if (cs_link_on_target in current_settings.globalswitches) then
     s:=ScriptFixFileName(s);
    LinkRes.Add('-L'+s);
    HPath:=TCmdStrListItem(HPath.Next);
   end;
  HPath:=TCmdStrListItem(LibrarySearchPath.First);
  while assigned(HPath) do
   begin
    s:=HPath.Str;
    if s<>'' then
     LinkRes.Add('SEARCH_DIR("'+s+'")');
    HPath:=TCmdStrListItem(HPath.Next);
   end;

  LinkRes.Add('INPUT (');
  { add objectfiles, start with prt0 always }
  if not (target_info.system in systems_internal_sysinit) then
    begin
      s:=FindObjectFile('prt0','',false);
      LinkRes.AddFileName(maybequoted(s));
    end;
  while not ObjectFiles.Empty do
   begin
    s:=ObjectFiles.GetFirst;
    if s<>'' then
     begin
      LinkRes.AddFileName(maybequoted(s));
     end;
   end;
  LinkRes.Add(')');

  { Write staticlibraries }
  if not StaticLibFiles.Empty then
   begin
    LinkRes.Add('GROUP(');
    while not StaticLibFiles.Empty do
     begin
      S:=StaticLibFiles.GetFirst;
      LinkRes.AddFileName(maybequoted(s));
     end;
    LinkRes.Add(')');
   end;

    { Write sharedlibraries like -l<lib>, also add the needed dynamic linker
      here to be sure that it gets linked this is needed for glibc2 systems (PFV) }
    linklibc:=false;
    while not SharedLibFiles.Empty do
     begin
      S:=SharedLibFiles.GetFirst;
      if s<>'c' then
       begin
        i:=Pos(target_info.sharedlibext,S);
        if i>0 then
         Delete(S,i,255);
        LinkRes.Add('-l'+s);
       end
      else
       begin
        LinkRes.Add('-l'+s);
        linklibc:=true;
       end;
     end;
    { be sure that libc&libgcc is the last lib }
    if linklibc then
     begin
      LinkRes.Add('-lc');
      LinkRes.Add('-lgcc');
     end;

{ Write and Close response }
  linkres.writetodisk;
  linkres.free;

  WriteResponseFile:=True;
end;


function TLinkerMacOS.MakeMacOSExe: boolean;
var
  BinStr,
  CmdStr  : TCmdStr;
  StripStr: string[40];
  DynLinkStr : string;
  GCSectionsStr : string;
  ExeName: string;
begin
  StripStr:='';
  GCSectionsStr:='';
  DynLinkStr:='';

  if (cs_link_strip in current_settings.globalswitches) then
    StripStr:='-s';
  if rlinkpath<>'' then
    DynLinkStr:='--rpath-link '+rlinkpath;
    if create_smartlink_sections then
      GCSectionsStr:='--gc-sections ';

  ExeName:=current_module.exefilename;

  { Call linker }
  SplitBinCmd(Info.ExeCmd[1],BinStr,CmdStr);
  binstr:=FindUtil(utilsprefix+BinStr);
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$EXE',maybequoted(ScriptFixFileName(ExeName)));
  Replace(cmdstr,'$RES',maybequoted(ScriptFixFileName(outputexedir+Info.ResName)));
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);

  MakeMacOSExe:=DoExec(BinStr,CmdStr,true,false);
end;


function TLinkerMacOS.MakeExecutable:boolean;
var
  success : boolean;
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
    Message1(exec_i_linking,current_module.exefilename);

  { Write used files and libraries }
  WriteResponseFile(false);

  success:=MakeMacOSExe;

  { Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    DeleteFile(outputexedir+Info.ResName);

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;



{*****************************************************************************
                                  Initialize
*****************************************************************************}

initialization
{$ifdef m68k}
{$ifndef macos}
  RegisterLinker(ld_mpw,TLinkerMacOS);
{$endif}
  RegisterTarget(system_m68k_macos_info);
  RegisterImport(system_m68k_macos,timportlibmacos);
{$endif m68k}
{$ifdef powerpc}
{$ifndef macos}
  RegisterLinker(ld_mpw,TLinkerMacOS);
{$else}
  RegisterLinker(ld_mpw,TLinkerMPW);
{$endif}
  RegisterTarget(system_powerpc_macos_info);
  RegisterImport(system_powerpc_macos,timportlibmacos);
{$endif powerpc}
end.
