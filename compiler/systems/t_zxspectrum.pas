{
    Copyright (c) 2005-2020 by Free Pascal Compiler team

    This unit implements support import, export, link routines
    for the ZX Spectrum Target

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
unit t_zxspectrum;

{$i fpcdefs.inc}

interface


implementation

    uses
       SysUtils,
       cutils,cfileutl,cclasses,
       globtype,globals,systems,verbose,comphook,cscript,fmodule,i_zxspectrum,link,
       cpuinfo,ogbase,ogrel,owar;

    const
       DefaultOrigin=23800;

    type

       { sdld - the sdld linker from the SDCC project ( http://sdcc.sourceforge.net/ ) }
       { vlink - the vlink linker by Frank Wille (http://sun.hasenbraten.de/vlink/ ) }

       TLinkerZXSpectrum=class(texternallinker)
       private
          FOrigin: Word;
          Function  WriteResponseFile_Sdld: Boolean;
          Function  WriteResponseFile_Vlink: Boolean;

          procedure SetDefaultInfo_Sdld;
          procedure SetDefaultInfo_Vlink;
          function  MakeExecutable_Sdld: boolean;
          function  MakeExecutable_Vlink: boolean;
       public
          procedure SetDefaultInfo; override;
          function  MakeExecutable: boolean; override;
          procedure InitSysInitUnitName; override;

          function postprocessexecutable(const fn : string;isdll:boolean): boolean;
       end;

       { TInternalLinkerZXSpectrum }

       TInternalLinkerZXSpectrum=class(tinternallinker)
       private
         FOrigin: Word;
       protected
         procedure DefaultLinkScript;override;
       public
         constructor create;override;
         procedure InitSysInitUnitName;override;
         function MakeExecutable: boolean; override;
         function postprocessexecutable(const fn : string): boolean;
       end;


{*****************************************************************************
                          TLinkerZXSpectrum
*****************************************************************************}

function TLinkerZXSpectrum.WriteResponseFile_Sdld: Boolean;
  Var
    linkres  : TLinkRes;
    s        : TCmdStr;
    prtobj: string[80];
  begin
    result:=False;
    prtobj:='prt0';

    { Open link.res file }
    LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);

    { Write the origin (i.e. the program load address) }
    LinkRes.Add('-b _CODE='+tostr(FOrigin));

    if not (target_info.system in systems_internal_sysinit) and (prtobj <> '') then
      begin
        s:=FindObjectFile(prtobj,'',false);
        LinkRes.AddFileName(s);
      end;

    while not ObjectFiles.Empty do
     begin
      s:=ObjectFiles.GetFirst;
      if s<>'' then
       begin
        if not(cs_link_on_target in current_settings.globalswitches) then
         s:=FindObjectFile(s,'',false);
        LinkRes.AddFileName((maybequoted(s)));
       end;
     end;

    { Write staticlibraries }
    if not StaticLibFiles.Empty then
     begin
      while not StaticLibFiles.Empty do
       begin
        S:=StaticLibFiles.GetFirst;
        LinkRes.Add('-l'+maybequoted(s));
       end;
     end;

    { Write and Close response }
    linkres.writetodisk;
    linkres.free;

    result:=True;
  end;

function TLinkerZXSpectrum.WriteResponseFile_Vlink: Boolean;
  Var
    linkres  : TLinkRes;
    s        : TCmdStr;
    prtobj: string[80];
  begin
    result:=false;
    prtobj:='prt0';

    { Open link.res file }
    LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);

    LinkRes.Add('INPUT (');

    if not (target_info.system in systems_internal_sysinit) and (prtobj <> '') then
      begin
        s:=FindObjectFile(prtobj,'',false);
        LinkRes.AddFileName(maybequoted(s));
      end;

    while not ObjectFiles.Empty do
      begin
        s:=ObjectFiles.GetFirst;
        if s<>'' then
          begin
            s:=FindObjectFile(s,'',false);
            LinkRes.AddFileName(maybequoted(s));
          end;
      end;

    while not StaticLibFiles.Empty do
      begin
        S:=StaticLibFiles.GetFirst;
        LinkRes.AddFileName(maybequoted(s));
      end;

    LinkRes.Add(')');

    with LinkRes do
      begin
        Add('');
        Add('SECTIONS');
        Add('{');
        Add('  . = 0x'+hexstr(FOrigin,4)+';');
        Add('  .text : { *(.text .text.* _CODE _CODE.* ) }');
        Add('  .data : { *(.data .data.* .rodata .rodata.* .fpc.* ) }');
        Add('  .bss  : { *(_BSS _BSS.*) *(.bss .bss.*) *(_BSSEND _BSSEND.*) *(_HEAP _HEAP.*) *(.stack .stack.*) *(_STACK _STACK.*) }');
        Add('}');
      end;

    { Write and Close response }
    linkres.writetodisk;
    linkres.free;

    result:=true;
  end;

procedure TLinkerZXSpectrum.SetDefaultInfo_Sdld;
  const
    ExeName='sdldz80';
  begin
    if ImageBaseSetExplicity then
      FOrigin:=ImageBase
    else
      FOrigin:=DefaultOrigin;
    with Info do
     begin
       ExeCmd[1]:=ExeName+' -n $OPT -i $MAP $EXE -f $RES'
     end;
  end;

procedure TLinkerZXSpectrum.SetDefaultInfo_Vlink;
  const
    ExeName='vlink';
  begin
    if ImageBaseSetExplicity then
      FOrigin:=ImageBase
    else
      FOrigin:=DefaultOrigin;
    with Info do
     begin
       ExeCmd[1]:=ExeName+' -bihex $GCSECTIONS -e $STARTSYMBOL $STRIP $OPT $MAP -o $EXE -T $RES'
     end;
  end;

procedure TLinkerZXSpectrum.SetDefaultInfo;
  begin
    if not (cs_link_vlink in current_settings.globalswitches) then
      SetDefaultInfo_Sdld
    else
      SetDefaultInfo_Vlink;
  end;

function TLinkerZXSpectrum.MakeExecutable_Sdld: boolean;
  var
    binstr,
    cmdstr,
    mapstr: TCmdStr;
    success : boolean;
    StaticStr,
    //GCSectionsStr,
    DynLinkStr,
    StripStr,
    FixedExeFileName: string;
  begin
    { for future use }
    StaticStr:='';
    StripStr:='';
    mapstr:='';
    DynLinkStr:='';
    FixedExeFileName:=maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename,'.ihx')));

    if (cs_link_map in current_settings.globalswitches) then
     mapstr:='-mw';

  { Write used files and libraries }
    WriteResponseFile_Sdld();

  { Call linker }
    SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
    Replace(cmdstr,'$OPT',Info.ExtraOptions);

    Replace(cmdstr,'$EXE',FixedExeFileName);
    Replace(cmdstr,'$RES',(maybequoted(ScriptFixFileName(outputexedir+Info.ResName))));
    Replace(cmdstr,'$STATIC',StaticStr);
    Replace(cmdstr,'$STRIP',StripStr);
    Replace(cmdstr,'$MAP',mapstr);
    //Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
    Replace(cmdstr,'$DYNLINK',DynLinkStr);

    success:=DoExec(FindUtil(utilsprefix+BinStr),cmdstr,true,false);

  { Remove ReponseFile }
    if success and not(cs_link_nolink in current_settings.globalswitches) then
     DeleteFile(outputexedir+Info.ResName);

  { Post process }
    if success and not(cs_link_nolink in current_settings.globalswitches) then
      success:=PostProcessExecutable(FixedExeFileName,false);

    result:=success;   { otherwise a recursive call to link method }
  end;

function TLinkerZXSpectrum.MakeExecutable_Vlink: boolean;
  var
    binstr,
    cmdstr: TCmdStr;
    success: boolean;
    GCSectionsStr,
    StripStr,
    StartSymbolStr,
    MapStr,
    FixedExeFilename: string;
  begin
    GCSectionsStr:='-gc-all -mtype';
    StripStr:='';
    MapStr:='';
    StartSymbolStr:='start';
    FixedExeFileName:=maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename,'.ihx')));

    if (cs_link_map in current_settings.globalswitches) then
      MapStr:='-M'+maybequoted(ScriptFixFileName(current_module.mapfilename));

  { Write used files and libraries }
    WriteResponseFile_Vlink();

  { Call linker }
    SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
    Replace(cmdstr,'$OPT',Info.ExtraOptions);

    Replace(cmdstr,'$EXE',FixedExeFileName);
    Replace(cmdstr,'$RES',(maybequoted(ScriptFixFileName(outputexedir+Info.ResName))));
    Replace(cmdstr,'$MAP',MapStr);
    Replace(cmdstr,'$STRIP',StripStr);
    Replace(cmdstr,'$STARTSYMBOL',StartSymbolStr);
    Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);

    success:=DoExec(FindUtil(utilsprefix+BinStr),cmdstr,true,false);

  { Remove ReponseFile }
    if success and not(cs_link_nolink in current_settings.globalswitches) then
     DeleteFile(outputexedir+Info.ResName);

  { Post process }
    if success and not(cs_link_nolink in current_settings.globalswitches) then
      success:=PostProcessExecutable(FixedExeFileName,false);

    result:=success;
  end;

function TLinkerZXSpectrum.MakeExecutable: boolean;
  begin
    if not (cs_link_vlink in current_settings.globalswitches) then
      result:=MakeExecutable_Sdld
    else
      result:=MakeExecutable_Vlink;
  end;


procedure TLinkerZXSpectrum.InitSysInitUnitName;
begin
  sysinitunit:='si_prc';
end;

function TLinkerZXSpectrum.postprocessexecutable(const fn: string; isdll: boolean): boolean;
  begin
    result:=DoExec(FindUtil(utilsprefix+'ihxutil'),' '+fn,true,false);
  end;


{*****************************************************************************
                          TInternalLinkerZXSpectrum
*****************************************************************************}

procedure TInternalLinkerZXSpectrum.DefaultLinkScript;
  var
    s        : TCmdStr;
    prtobj: string[80];
  begin
    prtobj:='prt0';

    if not (target_info.system in systems_internal_sysinit) and (prtobj <> '') then
      LinkScript.Concat('READOBJECT ' + maybequoted(FindObjectFile(prtobj,'',false)));

    while not ObjectFiles.Empty do
      begin
        s:=ObjectFiles.GetFirst;
        if s<>'' then
          begin
            if not(cs_link_on_target in current_settings.globalswitches) then
              s:=FindObjectFile(s,'',false);
            LinkScript.Concat('READOBJECT ' + maybequoted(s));
          end;
      end;

    LinkScript.Concat('GROUP');
    { Write staticlibraries }
    if not StaticLibFiles.Empty then
      begin
        while not StaticLibFiles.Empty do
          begin
            S:=StaticLibFiles.GetFirst;
            if s<>'' then
              LinkScript.Concat('READSTATICLIBRARY '+MaybeQuoted(s));
          end;
      end;
    LinkScript.Concat('ENDGROUP');

    LinkScript.Concat('IMAGEBASE '+tostr(FOrigin));

    LinkScript.Concat('EXESECTION .text');
    LinkScript.Concat('  OBJSECTION _CODE');
    LinkScript.Concat('ENDEXESECTION');
    LinkScript.Concat('EXESECTION .data');
    LinkScript.Concat('  OBJSECTION _DATA');
    LinkScript.Concat('ENDEXESECTION');
    LinkScript.Concat('EXESECTION .bss');
    LinkScript.Concat('  OBJSECTION _BSS');
    LinkScript.Concat('  OBJSECTION _BSSEND');
    LinkScript.Concat('  OBJSECTION _HEAP');
    LinkScript.Concat('  OBJSECTION _STACK');
    LinkScript.Concat('ENDEXESECTION');

    LinkScript.Concat('ENTRYNAME start');
  end;

constructor TInternalLinkerZXSpectrum.create;
  begin
    inherited create;
    CArObjectReader:=TArObjectReader;
    CExeOutput:=TZXSpectrumIntelHexExeOutput;
    CObjInput:=TRelObjInput;
    if ImageBaseSetExplicity then
      FOrigin:=ImageBase
    else
      FOrigin:=DefaultOrigin;
  end;

procedure TInternalLinkerZXSpectrum.InitSysInitUnitName;
  begin
    sysinitunit:='si_prc';
  end;

function TInternalLinkerZXSpectrum.MakeExecutable: boolean;
  begin
    result:=inherited;
    { Post process }
    if result and not(cs_link_nolink in current_settings.globalswitches) then
      result:=PostProcessExecutable(current_module.exefilename);
  end;

function TInternalLinkerZXSpectrum.postprocessexecutable(const fn: string): boolean;
  var
    exitcode: longint;
    FoundBin: ansistring;
    Found: Boolean;
    utilexe: TCmdStr;
  begin
    result:=false;

    utilexe:=utilsprefix+'ihxutil';
    FoundBin:='';
    Found:=false;
    if utilsdirectory<>'' then
      Found:=FindFile(utilexe,utilsdirectory,false,Foundbin);
    if (not Found) then
      Found:=FindExe(utilexe,false,Foundbin);

    if Found then
      begin
        exitcode:=RequotedExecuteProcess(foundbin,' '+fn);
        result:=exitcode<>0;
      end;
  end;

{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
{$ifdef z80}
  RegisterLinker(ld_int_zxspectrum,TInternalLinkerZXSpectrum);
  RegisterLinker(ld_zxspectrum,TLinkerZXSpectrum);
  RegisterTarget(system_z80_zxspectrum_info);
{$endif z80}
end.
