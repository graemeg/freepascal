{
    Copyright (c) 1998-2002 by Peter Vreman

    This unit handles the writing of script files

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
unit cscript;

{$i fpcdefs.inc}

interface
{$H+}
uses
  sysutils,
  globtype,
  cclasses;

type
  TScript=class
    fn   : TCmdStr;
    data : TCmdStrList;
    executable : boolean;
    constructor Create(const s:TCmdStr);
    constructor CreateExec(const s:TCmdStr);
    destructor Destroy;override;
    procedure AddStart(const s:TCmdStr);
    procedure Add(const s:TCmdStr);
    Function  Empty:boolean;
    procedure WriteToDisk;virtual;
  end;

  TAsmScript = class (TScript)
    Constructor Create(Const ScriptName : TCmdStr); virtual;
    Procedure AddAsmCommand (Const Command, Options,FileName : TCmdStr);virtual;abstract;
    Procedure AddLinkCommand (Const Command, Options, FileName : TCmdStr);virtual;abstract;
    Procedure AddDeleteCommand (Const FileName : TCmdStr);virtual;abstract;
    Procedure AddDeleteDirCommand (Const FileName : TCmdStr);virtual;abstract;
  end;

  TAsmScriptDos = class (TAsmScript)
    Constructor Create (Const ScriptName : TCmdStr); override;
    Procedure AddAsmCommand (Const Command, Options,FileName : TCmdStr);override;
    Procedure AddLinkCommand (Const Command, Options, FileName : TCmdStr);override;
    Procedure AddDeleteCommand (Const FileName : TCmdStr);override;
    Procedure AddDeleteDirCommand (Const FileName : TCmdStr);override;
    Procedure WriteToDisk;override;
  end;

  TAsmScriptAmiga = class (TAsmScript)
    Constructor Create (Const ScriptName : TCmdStr); override;
    Procedure AddAsmCommand (Const Command, Options,FileName : TCmdStr);override;
    Procedure AddLinkCommand (Const Command, Options, FileName : TCmdStr);override;
    Procedure AddDeleteCommand (Const FileName : TCmdStr);override;
    Procedure AddDeleteDirCommand (Const FileName : TCmdStr);override;
    Procedure WriteToDisk;override;
  end;

  TAsmScriptUnix = class (TAsmScript)
    Constructor Create (Const ScriptName : TCmdStr);override;
    Procedure AddAsmCommand (Const Command, Options,FileName : TCmdStr);override;
    Procedure AddLinkCommand (Const Command, Options, FileName : TCmdStr);override;
    Procedure AddDeleteCommand (Const FileName : TCmdStr);override;
    Procedure AddDeleteDirCommand (Const FileName : TCmdStr);override;
    Procedure WriteToDisk;override;
  end;

  TAsmScriptMPW = class (TAsmScript)
    Constructor Create (Const ScriptName : TCmdStr); override;
    Procedure AddAsmCommand (Const Command, Options,FileName : TCmdStr);override;
    Procedure AddLinkCommand (Const Command, Options, FileName : TCmdStr);override;
    Procedure AddDeleteCommand (Const FileName : TCmdStr);override;
    Procedure AddDeleteDirCommand (Const FileName : TCmdStr);override;
    Procedure WriteToDisk;override;
  end;

  TLinkRes = Class (TScript)
    section: string[30];
    fRealResponseFile: Boolean;
    fForceUseForwardSlash: Boolean;
    constructor Create(const ScriptName : TCmdStr; RealResponseFile: Boolean);
    procedure Add(const s:TCmdStr);
    procedure AddFileName(const s:TCmdStr);
    procedure EndSection(const s:TCmdStr);
    procedure StartSection(const s:TCmdStr);
  end;

var
  AsmRes : TAsmScript;

Function ScriptFixFileName(const s:TCmdStr):TCmdStr;
Procedure GenerateAsmRes(const st : TCmdStr);
Function GenerateScript(const st : TCmdStr): TAsmScript;


implementation

uses
{$ifdef hasUnix}
  BaseUnix,
{$endif}
  cutils,cfileutl,
  globals,systems,verbose;


{****************************************************************************
                                   Helpers
****************************************************************************}

    Function ScriptFixFileName(const s:TCmdStr):TCmdStr;
     begin
       if cs_link_on_target in current_settings.globalswitches then
         ScriptFixFileName:=TargetFixFileName(s)
       else
         ScriptFixFileName:=FixFileName(s);
     end;

{****************************************************************************
                                  TScript
****************************************************************************}

constructor TScript.Create(const s: TCmdStr);
begin
  fn:=FixFileName(s);
  executable:=false;
  data:=TCmdStrList.Create;
end;


constructor TScript.CreateExec(const s:TCmdStr);
begin
  fn:=FixFileName(s);
  if cs_link_on_target in current_settings.globalswitches then
    fn:=ChangeFileExt(fn,target_info.scriptext)
  else
    fn:=ChangeFileExt(fn,source_info.scriptext);
  executable:=true;
  data:=TCmdStrList.Create;
end;


destructor TScript.Destroy;
begin
  data.Free;
end;


procedure TScript.AddStart(const s:TCmdStr);
begin
  data.Insert(s);
end;


procedure TScript.Add(const s:TCmdStr);
begin
   data.Concat(s);
end;


Function TScript.Empty:boolean;
begin
  Empty:=Data.Empty;
end;

procedure TScript.WriteToDisk;
var
  t : file;
  i : longint;
  s : TCmdStr;
  le: string[2];

begin
  Assign(t,fn);
  if cs_link_on_target in current_settings.globalswitches then
    le:= target_info.newline
  else
    le:= source_info.newline;

  {$push}{$I-}
  Rewrite(t,1);
  if ioresult<>0 then
    exit;
  while not data.Empty do
    begin
      s:=data.GetFirst;
      Blockwrite(t,s[1],length(s),i);
      Blockwrite(t,le[1],length(le),i);
    end;
  Close(t);
  {$pop}
  i:=ioresult;
{$ifdef hasUnix}
  if executable then
   fpchmod(fn,493);
{$endif}
end;

{****************************************************************************
                                  Asm Response
****************************************************************************}

Constructor TAsmScript.Create (Const ScriptName : TCmdStr);
begin
  Inherited CreateExec(ScriptName);
end;


{****************************************************************************
                                  DOS Asm Response
****************************************************************************}

Constructor TAsmScriptDos.Create (Const ScriptName : TCmdStr);
begin
  Inherited Create(ScriptName);
end;


Procedure TAsmScriptDos.AddAsmCommand (Const Command, Options,FileName : TCmdStr);
begin
  if FileName<>'' then
   begin
     Add('SET THEFILE='+ScriptFixFileName(FileName));
     Add('echo Assembling %THEFILE%');
   end;
  Add(maybequoted(command)+' '+Options);
  Add('if errorlevel 1 goto asmend');
end;


Procedure TAsmScriptDos.AddLinkCommand (Const Command, Options, FileName : TCmdStr);
begin
  if FileName<>'' then
   begin
     Add('SET THEFILE='+ScriptFixFileName(FileName));
     Add('echo Linking %THEFILE%');
   end;
  Add(maybequoted(command)+' '+Options);
  Add('if errorlevel 1 goto linkend');
end;


Procedure TAsmScriptDos.AddDeleteCommand (Const FileName : TCmdStr);
begin
 Add('Del ' + MaybeQuoted (ScriptFixFileName (FileName)));
end;


Procedure TAsmScriptDos.AddDeleteDirCommand (Const FileName : TCmdStr);
begin
 Add('Rmdir ' + MaybeQuoted (ScriptFixFileName (FileName)));
end;


Procedure TAsmScriptDos.WriteToDisk;
Begin
  AddStart('@echo off');
  Add('goto end');
  Add(':asmend');
  Add('echo An error occurred while assembling %THEFILE%');
  Add('goto end');
  Add(':linkend');
  Add('echo An error occurred while linking %THEFILE%');
  Add(':end');
  inherited WriteToDisk;
end;

{****************************************************************************
                                  Amiga Asm Response
****************************************************************************}


Constructor TAsmScriptAmiga.Create (Const ScriptName : TCmdStr);
begin
  Inherited Create(ScriptName);
end;


Procedure TAsmScriptAmiga.AddAsmCommand (Const Command, Options,FileName : TCmdStr);
begin
  if FileName<>'' then
   begin
     Add('SET THEFILE '+ScriptFixFileName(FileName));
     Add('echo Assembling $THEFILE');
   end;
  Add(maybequoted(command)+' '+Options);
  { There is a problem here,
    as always return with a non zero error value PM  }
  Add('if error');
  Add('why');
  Add('skip asmend');
  Add('endif');
end;


Procedure TAsmScriptAmiga.AddLinkCommand (Const Command, Options, FileName : TCmdStr);
begin
  if FileName<>'' then
   begin
     Add('SET THEFILE '+ScriptFixFileName(FileName));
     Add('echo Linking $THEFILE');
   end;
  Add(maybequoted(command)+' '+Options);
  Add('if error');
  Add('skip linkend');
  Add('endif');
end;


Procedure TAsmScriptAmiga.AddDeleteCommand (Const FileName : TCmdStr);
begin
 Add('Delete ' + Unix2AmigaPath(MaybeQuoted(ScriptFixFileName(FileName))) + ' Quiet');
end;


Procedure TAsmScriptAmiga.AddDeleteDirCommand (Const FileName : TCmdStr);
begin
 Add('Delete ' + Unix2AmigaPath(MaybeQuoted(ScriptFixFileName(FileName))) + ' All Quiet');
end;


Procedure TAsmScriptAmiga.WriteToDisk;
Begin
  Add('skip end');
  Add('lab asmend');
  Add('why');
  Add('echo An error occurred while assembling $THEFILE');
  Add('skip end');
  Add('lab linkend');
  Add('why');
  Add('echo An error occurred while linking $THEFILE');
  Add('lab end');
  inherited WriteToDisk;
end;


{****************************************************************************
                              Unix Asm Response
****************************************************************************}

Constructor TAsmScriptUnix.Create (Const ScriptName : TCmdStr);
begin
  Inherited Create(ScriptName);
end;


Procedure TAsmScriptUnix.AddAsmCommand (Const Command, Options,FileName : TCmdStr);
begin
  if FileName<>'' then
   Add('echo Assembling '+maybequoted(ScriptFixFileName(FileName)));
  Add(maybequoted(command)+' '+Options);
  Add('if [ $? != 0 ]; then DoExitAsm '+maybequoted(ScriptFixFileName(FileName))+'; fi');
end;


Procedure TAsmScriptUnix.AddLinkCommand (Const Command, Options, FileName : TCmdStr);
begin
  if FileName<>'' then
   Add('echo Linking '+ScriptFixFileName(FileName));
  Add('OFS=$IFS');
  Add('IFS="');
  Add('"');
  Add(maybequoted(command)+' '+Options);
  Add('if [ $? != 0 ]; then DoExitLink '+ScriptFixFileName(FileName)+'; fi');
  Add('IFS=$OFS');
end;


Procedure TAsmScriptUnix.AddDeleteCommand (Const FileName : TCmdStr);
begin
 Add('rm ' + MaybeQuoted (ScriptFixFileName(FileName)));
end;


Procedure TAsmScriptUnix.AddDeleteDirCommand (Const FileName : TCmdStr);
begin
 Add('rmdir ' + MaybeQuoted (ScriptFixFileName(FileName)));
end;


Procedure TAsmScriptUnix.WriteToDisk;
Begin
  AddStart('{ echo "An error occurred while linking $1"; exit 1; }');
  AddStart('DoExitLink ()');
  AddStart('{ echo "An error occurred while assembling $1"; exit 1; }');
  AddStart('DoExitAsm ()');
  {$ifdef BEOS}
   AddStart('#!/boot/beos/bin/sh');
  {$else}
   AddStart('#!/bin/sh');
  {$endif}
  inherited WriteToDisk;
end;


{****************************************************************************
                                  MPW (MacOS) Asm Response
****************************************************************************}

Constructor TAsmScriptMPW.Create (Const ScriptName : TCmdStr);
begin
  Inherited Create(ScriptName);
end;


Procedure TAsmScriptMPW.AddAsmCommand (Const Command, Options,FileName : TCmdStr);
begin
  if FileName<>'' then
    Add('Echo Assembling '+ScriptFixFileName(FileName));
  Add(maybequoted(command)+' '+Options);
  Add('Exit If "{Status}" != 0');
end;


Procedure TAsmScriptMPW.AddLinkCommand (Const Command, Options, FileName : TCmdStr);
begin
  if FileName<>'' then
    Add('Echo Linking '+ScriptFixFileName(FileName));
  Add(maybequoted(command)+' '+Options);
  Add('Exit If "{Status}" != 0');

  {Add resources}
  if apptype = app_cui then {If SIOW}
    begin
      Add('Rez -append "{RIncludes}"SIOW.r -o '+ ScriptFixFileName(FileName));
      Add('Exit If "{Status}" != 0');
    end;
end;


Procedure TAsmScriptMPW.AddDeleteCommand (Const FileName : TCmdStr);
begin
 Add('Delete ' + MaybeQuoted (ScriptFixFileName(FileName)));
end;


Procedure TAsmScriptMPW.AddDeleteDirCommand (Const FileName : TCmdStr);
begin
 Add('Delete ' + MaybeQuoted (ScriptFixFileName (FileName)));
end;


Procedure TAsmScriptMPW.WriteToDisk;
Begin
  AddStart('# Script for assembling and linking a FreePascal program on MPW (MacOS)');
  Add('Echo Done');
  inherited WriteToDisk;
end;



Procedure GenerateAsmRes(const st : TCmdStr);
begin
  AsmRes:=GenerateScript(st);
end;

function GenerateScript(const st: TCmdStr): TAsmScript;
  var
    scripttyp : tscripttype;
  begin
    if cs_link_on_target in current_settings.globalswitches then
      scripttyp := target_info.script
    else
      scripttyp := source_info.script;
    case scripttyp of
      script_unix :
        Result:=TAsmScriptUnix.Create(st);
      script_dos :
        Result:=TAsmScriptDos.Create(st);
      script_amiga :
        Result:=TAsmScriptAmiga.Create(st);
      script_mpw :
        Result:=TAsmScriptMPW.Create(st);
      else
        internalerror(2013112805);
    end;
  end;


{****************************************************************************
                                  Link Response
****************************************************************************}

constructor TLinkRes.Create(const ScriptName: TCmdStr; RealResponseFile: Boolean);
begin
  inherited Create(ScriptName);
  fRealResponseFile:=RealResponseFile;
  fForceUseForwardSlash:=false;
end;

procedure TLinkRes.Add(const s:TCmdStr);
begin
  if s<>'' then
   inherited Add(s);
end;

procedure TLinkRes.AddFileName(const s:TCmdStr);
var
  ls: TCmdStr;
  i: longint;
begin
  if section<>'' then
   begin
    inherited Add(section);
    section:='';
   end;
  if s<>'' then
   begin
     ls:=s;
     if fForceUseForwardSlash then
       { Fix separator }
       for i:=1 to length(ls) do
         if (ls[i]=source_info.dirsep) then
           ls[i]:='/';
     { GNU ld only supports double quotes in the response file. }
     if fRealResponseFile and
        (ls[1]='''') and
        (((cs_link_on_target in current_settings.globalswitches) and
          (target_info.script=script_unix)) or
         (not(cs_link_on_target in current_settings.globalswitches) and
          (source_info.script=script_unix))) then
       inherited add(UnixRequoteWithDoubleQuotes(s))
     else if not(ls[1] in ['a'..'z','A'..'Z','/','\','.','"']) then
      begin
        if fForceUseForwardSlash then
          inherited Add('./'+ls)
        else if (cs_link_on_target in current_settings.globalswitches) then
          inherited Add('.'+target_info.DirSep+ls)
        else
          inherited Add('.'+source_info.DirSep+ls);
      end
     else
      inherited Add(ls);
   end;
end;

procedure TLinkRes.EndSection(const s:TCmdStr);
begin
  { only terminate if we started the section }
  if section='' then
    inherited Add(s);
  section:='';
end;

procedure TLinkRes.StartSection(const s:TCmdStr);
begin
  section:=s;
end;

end.
