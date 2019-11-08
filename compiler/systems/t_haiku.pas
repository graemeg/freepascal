{
    Copyright (c) 1998-2002 by Peter Vreman
    Copyright (c) 2008-2008 by Olivier Coursiere

    This unit implements support import,export,link routines
    for the (i386) Haiku target.

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
unit t_haiku;

{$i fpcdefs.inc}

interface

  uses
    symsym,symdef,
    import,export,link;

  type
    timportlibhaiku=class(timportlib)
      procedure generatelib;override;
    end;

    texportlibhaiku=class(texportlib)
      procedure preparelib(const s : string);override;
      procedure exportprocedure(hp : texported_item);override;
      procedure exportvar(hp : texported_item);override;
      procedure generatelib;override;
    end;

    tlinkerhaiku=class(texternallinker)
    private
      Function  WriteResponseFile(isdll:boolean;makelib:boolean) : Boolean;
    public
      constructor Create;override;
      procedure SetDefaultInfo;override;
      procedure InitSysInitUnitName;override;
      function  MakeExecutable:boolean;override;
      function  MakeSharedLibrary:boolean;override;
    end;


implementation

  uses
    SysUtils,
    cutils,cfileutl,cclasses,
    verbose,systems,globtype,globals,
    symconst,cscript,
    fmodule,aasmbase,aasmtai,aasmdata,aasmcpu,cpubase,i_haiku,ogbase;

{*****************************************************************************
                               TIMPORTLIBHAIKU
*****************************************************************************}

    procedure timportlibhaiku.generatelib;
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
                               TEXPORTLIBHAIKU
*****************************************************************************}

procedure texportlibhaiku.preparelib(const s:string);
begin
end;


procedure texportlibhaiku.exportprocedure(hp : texported_item);
var
  hp2 : texported_item;
begin
  { first test the index value }
  if eo_index in hp.options then
   begin
     Message1(parser_e_no_export_with_index_for_target,'haiku');
     exit;
   end;
  { now place in correct order }
  hp2:=texported_item(current_module._exports.first);
  while assigned(hp2) and
     (hp.name^>hp2.name^) do
    hp2:=texported_item(hp2.next);
  { insert hp there !! }
  if assigned(hp2) and (hp2.name^=hp.name^) then
    begin
      { this is not allowed !! }
      duplicatesymbol(hp.name^);
      exit;
    end;
  if hp2=texported_item(current_module._exports.first) then
    current_module._exports.concat(hp)
  else if assigned(hp2) then
    begin
       hp.next:=hp2;
       hp.previous:=hp2.previous;
       if assigned(hp2.previous) then
         hp2.previous.next:=hp;
       hp2.previous:=hp;
    end
  else
    current_module._exports.concat(hp);
end;


procedure texportlibhaiku.exportvar(hp : texported_item);
begin
  hp.is_var:=true;
  exportprocedure(hp);
end;


procedure texportlibhaiku.generatelib;
var
  hp2 : texported_item;
  pd  : tprocdef;
begin
  hp2:=texported_item(current_module._exports.first);
  while assigned(hp2) do
   begin
     if (not hp2.is_var) and
        (hp2.sym.typ=procsym) then
      begin
        { the manglednames can already be the same when the procedure
          is declared with cdecl }
        pd:=tprocdef(tprocsym(hp2.sym).ProcdefList[0]);
        if pd.mangledname<>hp2.name^ then
         begin
{$ifdef i386}
           { place jump in al_procedures }
           current_asmdata.asmlists[al_procedures].concat(Tai_align.Create_op(4,$90));
           current_asmdata.asmlists[al_procedures].concat(Tai_symbol.Createname_global(hp2.name^,AT_FUNCTION,0,pd));
           current_asmdata.asmlists[al_procedures].concat(Taicpu.Op_sym(A_JMP,S_NO,current_asmdata.RefAsmSymbol(pd.mangledname,AT_FUNCTION)));
           current_asmdata.asmlists[al_procedures].concat(Tai_symbol_end.Createname(hp2.name^));
{$endif i386}
         end;
      end
     else
      Message1(parser_e_no_export_of_variables_for_target,'haiku');
     hp2:=texported_item(hp2.next);
   end;
end;


{*****************************************************************************
                                  TLINKERHAIKU
*****************************************************************************}

Constructor TLinkerHaiku.Create;
const
  HomeNonPackagedDevLib = '=/boot/home/config/non-packaged/develop/lib';
  HomeDevLib = '=/boot/home/config/develop/lib';
  CommonNonPackagedDevLib = '=/boot/common/non-packaged/develop/lib';
  CommonDevLib = '=/boot/common/develop/lib';
  SystemDevLib = '=/boot/system/develop/lib';
var
  s : string;
  i : integer;
begin
  Inherited Create;
  s:=GetEnvironmentVariable('BELIBRARIES');
  { convert to correct format in case under unix system }
  for i:=1 to length(s) do
    if s[i] = ':' then
      s[i] := ';';
  { just in case we have a single path : add the ending ; }
  { since that is what the compiler expects.              }
  if pos(';',s) = 0 then
    s:=s+';';

  // Under Haiku with package management, BELIBRARIES is empty by default
  // We have to look at those system paths, in this order.
  // User can still customize BELIBRARIES. That is why it is looked at first.
  LibrarySearchPath.AddLibraryPath(sysrootpath,s,true); {format:'path1;path2;...'}

  LibrarySearchPath.AddLibraryPath(sysrootpath, HomeNonPackagedDevLib, false);
  LibrarySearchPath.AddLibraryPath(sysrootpath, HomeDevLib, false);
  LibrarySearchPath.AddLibraryPath(sysrootpath, CommonNonPackagedDevLib, false);
  LibrarySearchPath.AddLibraryPath(sysrootpath, CommonDevLib, false);
  LibrarySearchPath.AddLibraryPath(sysrootpath, SystemDevLib, false);
end;


procedure TLinkerHaiku.SetDefaultInfo;
begin
  with Info do
   begin
     ExeCmd[1]:='ld $OPT $DYNLINK $STATIC $STRIP -L. -o $EXE $CATRES';
     DllCmd[1]:='ld $OPT $INIT $FINI $SONAME -shared -L. -o $EXE $CATRES';
     DllCmd[2]:='strip --strip-unneeded $EXE';
(*
     ExeCmd[1]:='sh $RES $EXE $OPT $STATIC $STRIP -L.';
{     ExeCmd[1]:='sh $RES $EXE $OPT $DYNLINK $STATIC $STRIP -L.';}
      DllCmd[1]:='sh $RES $EXE $OPT -L.';

{     DllCmd[1]:='sh $RES $EXE $OPT -L. -g -nostart -soname=$EXE';
 }    DllCmd[2]:='strip --strip-unneeded $EXE';
{     DynamicLinker:='/lib/ld-beos.so.2';}
*)
   end;
end;


procedure TLinkerHaiku.InitSysInitUnitName;
const
  SysInitUnitNames: array[boolean] of string[15] = ( 'si_c', 'si_dllc' );
begin
  sysinitunit:=SysInitUnitNames[current_module.islibrary];
end;


function TLinkerHaiku.WriteResponseFile(isdll:boolean;makelib:boolean) : Boolean;
Var
  linkres  : TLinkRes;
  i        : integer;
  cprtobj,
  prtobj   : string[80];
  HPath    : TCmdStrListItem;
  s        : TCmdStr;
  linklibc : boolean;
begin
  WriteResponseFile:=False;
{ set special options for some targets }
  linklibc:=(SharedLibFiles.Find('root')<>nil);

  if (cs_profile in current_settings.moduleswitches) or
     (not SharedLibFiles.Empty) then
   begin
     AddSharedLibrary('root');
     linklibc:=true;
   end;

  prtobj:='';
  cprtobj:='';
  if not (target_info.system in systems_internal_sysinit) then
    begin
      prtobj:='prt0';
      cprtobj:='cprt0';

      if (not linklibc) and makelib then
        begin
          linklibc:=true;
          cprtobj:='dllprt.o';
        end
      else if makelib then
        begin
          // Making a dll with libc linking. Should be always the case under Haiku.
          cprtobj:='dllcprt0';
        end;
    end;

  if linklibc then
    prtobj:=cprtobj;

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,false);
  {
  if not isdll then
   LinkRes.Add('ld -o $1 $2 $3 $4 $5 $6 $7 $8 $9 \')
  else
   LinkRes.Add('ld -o $1 -e 0 $2 $3 $4 $5 $6 $7 $8 $9\');
  }
  LinkRes.Add('-m');
{$ifdef i386}
  LinkRes.Add('elf_i386_haiku');
{$else i386}
  LinkRes.Add('elf_x86_64_haiku');
{$endif i386}
  LinkRes.Add('-shared');
  LinkRes.Add('-Bsymbolic');

  { Write path to search libraries }
  HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
  while assigned(HPath) do
   begin
     LinkRes.Add('-L'+HPath.Str);
     HPath:=TCmdStrListItem(HPath.Next);
   end;
  HPath:=TCmdStrListItem(LibrarySearchPath.First);
  while assigned(HPath) do
   begin
     LinkRes.Add('-L'+HPath.Str);
     HPath:=TCmdStrListItem(HPath.Next);
   end;

  { try to add crti and crtbegin if linking to C }
  if linklibc then
   begin
     if librarysearchpath.FindFile('crti.o',false,s) then
      LinkRes.AddFileName(s);
     if librarysearchpath.FindFile('crtbegin.o',false,s) then
      LinkRes.AddFileName(s);
{      s:=librarysearchpath.FindFile('start_dyn.o',found)+'start_dyn.o';
     if found then LinkRes.AddFileName(s+' \');}

     if prtobj<>'' then
      LinkRes.AddFileName(FindObjectFile(prtobj,'',false));

//     if isdll then
//      LinkRes.AddFileName(FindObjectFile('func.o','',false));

     if librarysearchpath.FindFile('init_term_dyn.o',false,s) then
      LinkRes.AddFileName(s);
   end
  else
   begin
     if prtobj<>'' then
      LinkRes.AddFileName(FindObjectFile(prtobj,'',false));
   end;

  { main objectfiles }
  while not ObjectFiles.Empty do
   begin
     s:=ObjectFiles.GetFirst;
     if s<>'' then
      LinkRes.AddFileName(s);
   end;

{  LinkRes.Add('-lroot \');
  LinkRes.Add('/boot/develop/tools/gnupro/lib/gcc-lib/i586-beos/2.9-beos-991026/crtend.o \');
  LinkRes.Add('/boot/develop/lib/x86/crtn.o \');}

  { Write staticlibraries }
  if not StaticLibFiles.Empty then
   begin
     While not StaticLibFiles.Empty do
      begin
        S:=StaticLibFiles.GetFirst;
        LinkRes.AddFileName(s)
      end;
   end;

  { Write sharedlibraries like -l<lib> }
  if not SharedLibFiles.Empty then
   begin
     While not SharedLibFiles.Empty do
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
           linklibc:=true;
         end;
      end;
     { be sure that libc is the last lib }
{     if linklibc then
       LinkRes.Add('-lroot');}
{     if linkdynamic and (Info.DynamicLinker<>'') then
       LinkRes.AddFileName(Info.DynamicLinker);}
   end;
  if isdll then
   LinkRes.Add('-lroot');

  { objects which must be at the end }
  if linklibc then
   begin
     if librarysearchpath.FindFile('crtend.o',false,s) then
      LinkRes.AddFileName(s);
     if librarysearchpath.FindFile('crtn.o',false,s) then
      LinkRes.AddFileName(s);
   end;

{ Write and Close response }
  linkres.writetodisk;
  linkres.free;

  WriteResponseFile:=True;
end;


function TLinkerHaiku.MakeExecutable:boolean;
var
  binstr,
  cmdstr : TCmdStr;
  success,
  useshell : boolean;
  DynLinkStr : string[60];
  GCSectionsStr,
  StaticStr,
  StripStr   : string[40];
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

{ Create some replacements }
  StaticStr:='';
  StripStr:='';
  DynLinkStr:='';
  GCSectionsStr:='';
  if (cs_link_staticflag in current_settings.globalswitches) then
   StaticStr:='-static';
  if (cs_link_strip in current_settings.globalswitches) then
   StripStr:='-s';

  if (cs_link_smart in current_settings.globalswitches) and
     (tf_smartlink_sections in target_info.flags) then
      GCSectionsStr:='--gc-sections';

  If (cs_profile in current_settings.moduleswitches) or
     ((Info.DynamicLinker<>'') and (not SharedLibFiles.Empty)) then
   begin
     DynLinkStr:='-dynamic-linker='+Info.DynamicLinker;
     if cshared Then
       DynLinkStr:='--shared ' + DynLinkStr;
     if rlinkpath<>'' Then
       DynLinkStr:='--rpath-link '+rlinkpath + ' '+ DynLinkStr;
   End;

{ Write used files and libraries }
  WriteResponseFile(false,false);

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$CATRES',CatFileContent(outputexedir+Info.ResName));
  Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);
  useshell:=not (tf_no_backquote_support in source_info.flags);
  success:=DoExec(FindUtil(utilsprefix+BinStr),CmdStr,true,useshell);

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   DeleteFile(outputexedir+Info.ResName);

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


Function TLinkerHaiku.MakeSharedLibrary:boolean;
var
  binstr,
  cmdstr,
  SoNameStr : TCmdStr;
  success : boolean;
  DynLinkStr : string[60];
  StaticStr,
  StripStr   : string[40];

 begin
  MakeSharedLibrary:=false;
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.sharedlibfilename);

{ Create some replacements }
  StaticStr:='';
  StripStr:='';
  DynLinkStr:='';
  if (cs_link_staticflag in current_settings.globalswitches) then
   StaticStr:='-static';
  if (cs_link_strip in current_settings.globalswitches) then
   StripStr:='-s';
  If (cs_profile in current_settings.moduleswitches) or
     ((Info.DynamicLinker<>'') and (not SharedLibFiles.Empty)) then
   begin
     DynLinkStr:='-dynamic-linker='+Info.DynamicLinker;
     if cshared Then
       DynLinkStr:='--shared ' + DynLinkStr;
     if rlinkpath<>'' Then
       DynLinkStr:='--rpath-link '+rlinkpath + ' '+ DynLinkStr;
   End;
{ Write used files and libraries }
  WriteResponseFile(true,true);

  SoNameStr:='-soname '+ExtractFileName(current_module.sharedlibfilename);

{ Call linker }
  SplitBinCmd(Info.DllCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.sharedlibfilename));
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$CATRES',CatFileContent(outputexedir+Info.ResName));
  Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);
  Replace(cmdstr,'$SONAME',SoNameStr);

  success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,true,true);

{ Strip the library ? }
  if success and (cs_link_strip in current_settings.globalswitches) then
   begin
     SplitBinCmd(Info.DllCmd[2],binstr,cmdstr);
     Replace(cmdstr,'$EXE',maybequoted(current_module.sharedlibfilename));
     success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,true,false);
   end;

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   DeleteFile(outputexedir+Info.ResName);

  MakeSharedLibrary:=success;   { otherwise a recursive call to link method }
end;


{*****************************************************************************
                                  Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_haiku,TLinkerhaiku);
{$ifdef i386}
  RegisterImport(system_i386_haiku,timportlibhaiku);
  RegisterExport(system_i386_haiku,texportlibhaiku);
  RegisterTarget(system_i386_haiku_info);
{$endif i386}
{$ifdef x86_64}
  RegisterImport(system_x86_64_haiku,timportlibhaiku);
  RegisterExport(system_x86_64_haiku,texportlibhaiku);
  RegisterTarget(system_x86_64_haiku_info);
{$endif x86_64}
end.
