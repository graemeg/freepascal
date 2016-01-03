{
    Copyright (c) 1998-2002 by Peter Vreman

    This unit implements support import,export,link routines
    for the (i386) Netware target

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

    Currently generating NetWare-NLM's only work under Linux and win32.
    (see http://home.arcor.de/armin.diehl/fpcnw for binutils working
    with win32) while not included in fpc-releases.

    The following compiler-swiches are supported for NetWare:
    $DESCRIPTION    : NLM-Description, will be displayed at load-time
    $M              : For Stack-Size, Heap-Size will be ignored
                      32K is the accepted minimum
    $VERSION x.x.x  : Sets Major, Minor and Revision
    $SCREENNAME     : Sets the ScreenName
    $THREADNAME     : Sets current threadname

    Displaying copyright does not work with nlmconv from gnu bunutils
    version less that 2.13

    Additional parameters for the nlmvonv-inputfile can be passed with
    -k, i.e. -kREENTRANT will add the option REENTRANT to the nlmconv
    inputfile. A ; will be converted into a newline

    Exports will be handled like in win32:
    procedure bla;
    begin
    end;

    exports foo name 'Bar';

    The path to the import-Files must be specified by the library-path.
    All external modules are defined as autoload. (Note: the import-files have
    to be in unix-format for exe2nlm)
    By default, the most import files are included in freepascal.

    i.e. Procedure ConsolePrintf (p:pchar); cdecl; external 'clib.nlm';
    sets IMPORT @clib.imp and MODULE clib.

    Function simply defined as external work without generating autoload but
    you will get a warnung from nlmconv.

    If you dont have nlmconv, compile gnu-binutils with
       ./configure --enable-targets=i386-linux,i386-netware
       make all

    Debugging is possible with gdb and a converter from gdb to ndi available
    at http://home.arcor.de/armin.diehl/gdbnw

    A sample program:

    Program Hello;
    (*$DESCRIPTION HelloWorldNlm*)
    (*$VERSION 1.2.3*)
    (*$ScreenName Hello*)
    (*$M 60000,60000*)
    begin
      writeLn ('hello world');
    end.

    compile with:
    ppc386 -Tnetware hello

    ToDo:
      - No duplicate imports and autoloads
      - libc support (needs new target)

****************************************************************************
}
unit t_nwm;

{$i fpcdefs.inc}

interface


implementation

  uses
    SysUtils,
    cutils,cfileutl,
    verbose,systems,globtype,globals,
    symconst,script,
    fmodule,aasmbase,aasmtai,aasmdata,aasmcpu,cpubase,symsym,symdef,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    import,export,link,i_nwm,ogbase, ogcoff, ognlm, owar, cclasses
=======
    import,export,link,i_nwm,ogbase, ogcoff, ognlm, cclasses
>>>>>>> graemeg/cpstrnew
=======
    import,export,link,i_nwm,ogbase, ogcoff, ognlm, cclasses
>>>>>>> graemeg/cpstrnew
=======
    import,export,link,i_nwm,ogbase, ogcoff, ognlm, cclasses
>>>>>>> graemeg/cpstrnew
=======
    import,export,link,i_nwm,ogbase, ogcoff, ognlm, cclasses
>>>>>>> origin/cpstrnew
    {$ifdef netware} ,dos {$endif}
    ;

  type
    timportlibnetware=class(timportlib)
      procedure generatelib;override;
    end;

    texportlibnetware=class(texportlib)
      procedure preparelib(const s : string);override;
      procedure exportprocedure(hp : texported_item);override;
      procedure exportvar(hp : texported_item);override;
      procedure generatelib;override;
    end;

    tlinkernetware=class(texternallinker)
    private
      NLMConvLinkFile: TLinkRes;  {for second pass, fist pass is ld}
      Function  WriteResponseFile(isdll:boolean) : Boolean;
    public
      constructor Create;override;
      procedure SetDefaultInfo;override;
      function  MakeExecutable:boolean;override;
    end;

    TInternalLinkerNetware = class(TInternalLinker)
        prelude : string;
        constructor create;override;
        destructor destroy;override;
        procedure DefaultLinkScript;override;
        procedure InitSysInitUnitName;override;
        procedure ConcatEntryName; virtual;
        Function  MakeSharedLibrary:boolean;override;
      end;

Const tmpLinkFileName = 'link~tmp._o_';
      minStackSize = 32768;

{*****************************************************************************
                               TIMPORTLIBNETWARE
*****************************************************************************}

    procedure timportlibnetware.generatelib;
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
                               TEXPORTLIBNETWARE
*****************************************************************************}

procedure texportlibnetware.preparelib(const s:string);
begin
end;


procedure texportlibnetware.exportprocedure(hp : texported_item);
var
  hp2 : texported_item;
begin
  { first test the index value }
  if (hp.options and eo_index)<>0 then
   begin
     Comment(V_Error,'can''t export with index under netware');
     exit;
   end;
  { use pascal name is none specified }
  if (hp.options and eo_name)=0 then
    begin
       hp.name:=stringdup(hp.sym.name);
       hp.options:=hp.options or eo_name;
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
      Message1(parser_e_export_name_double,hp.name^);
      exit;
    end;
  if hp2=texported_item(current_module._exports.first) then
    current_module._exports.insert(hp)
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


procedure texportlibnetware.exportvar(hp : texported_item);
begin
  hp.is_var:=true;
  exportprocedure(hp);
end;


procedure texportlibnetware.generatelib;
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
           current_asmdata.asmlists[al_procedures].concat(Tai_symbol.Createname_global(hp2.name^,AT_FUNCTION,0));
           current_asmdata.asmlists[al_procedures].concat(Taicpu.Op_sym(A_JMP,S_NO,current_asmdata.RefAsmSymbol(pd.mangledname)));
           current_asmdata.asmlists[al_procedures].concat(Tai_symbol_end.Createname(hp2.name^));
{$endif i386}
         end;
      end
     else
      //Comment(V_Error,'Exporting of variables is not supported under netware');
      Message1(parser_e_no_export_of_variables_for_target,'netware');
     hp2:=texported_item(hp2.next);
   end;
end;


{*****************************************************************************
                                  TLINKERNETWARE
*****************************************************************************}

Constructor TLinkerNetware.Create;
begin
  Inherited Create;
end;


procedure TLinkerNetware.SetDefaultInfo;
begin
  with Info do
   begin
     {$ifndef netware}
     ExeCmd[1]:= FindUtil(utilsprefix+'ld') + ' -Ur -T $RES $STRIP -o $TMPOBJ';
     ExeCmd[2]:= FindUtil(utilsprefix+'nlmconv') + ' -T$RES';
     {$else}
     {for running on netware we need absolute pathes since ld has another working directory}
     ExeCmd[1]:= FindUtil(utilsprefix+'ld') + ' -Ur -T '+FExpand(outputexedir+Info.ResName)+' $STRIP -o '+Fexpand(outputexedir+tmpLinkFileName);
     ExeCmd[2]:= FindUtil(utilsprefix+'nlmconv') + ' -T'+FExpand(outputexedir+'n'+Info.ResName);
     {$endif}
   end;
end;


Function TLinkerNetware.WriteResponseFile(isdll:boolean) : Boolean;
Var
  linkres      : TLinkRes;
  i            : longint;
  s,s2,s3      : TCmdStr;
  ProgNam      : string [80];
  NlmNam       : string [80];
  hp2          : texported_item;  { for exports }
  p            : byte;
begin
  WriteResponseFile:=False;

  ProgNam := current_module.exefilename;
  i:=Pos(target_info.exeext,ProgNam);
  if i>0 then
    Delete(ProgNam,i,255);
  NlmNam := ProgNam + target_info.exeext;

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);             {for ld}
  NLMConvLinkFile:=TLinkRes.Create(outputexedir+'n'+Info.ResName,true); {for nlmconv, written in CreateExeFile}

  p := Pos ('"', Description);
  while (p > 0) do
  begin
    delete (Description,p,1);
    p := Pos ('"', Description);
  end;
  if Description <> '' then
    NLMConvLinkFile.Add('DESCRIPTION "' + Description + '"');
  NLMConvLinkFile.Add('VERSION '+tostr(dllmajor)+','+tostr(dllminor)+','+tostr(dllrevision));

  p := Pos ('"', nwscreenname);
  while (p > 0) do
  begin
    delete (nwscreenname,p,1);
    p := Pos ('"', nwscreenname);
  end;
  p := Pos ('"', nwthreadname);
  while (p > 0) do
  begin
    delete (nwthreadname,p,1);
    p := Pos ('"', nwthreadname);
  end;
  p := Pos ('"', nwcopyright);
  while (p > 0) do
  begin
    delete (nwcopyright,p,1);
    p := Pos ('"', nwcopyright);
  end;

  if nwscreenname <> '' then
    NLMConvLinkFile.Add('SCREENNAME "' + nwscreenname + '"');
  if nwthreadname <> '' then
    NLMConvLinkFile.Add('THREADNAME "' + nwthreadname + '"');
  if nwcopyright <> '' then
    NLMConvLinkFile.Add('COPYRIGHT "' + nwcopyright + '"');

  if stacksize < minStackSize then stacksize := minStackSize;
  str (stacksize, s);
  NLMConvLinkFile.Add ('STACKSIZE '+s);
  {$ifndef netware}
  NLMConvLinkFile.Add ('INPUT '+outputexedir+tmpLinkFileName);
  {$else}
  NLMConvLinkFile.Add ('INPUT '+FExpand(outputexedir+tmpLinkFileName));
  {$endif}

  { add objectfiles, start with nwpre always }
  LinkRes.Add ('INPUT(');
  if target_info.system = system_i386_netwlibc then
   begin
     s2 := FindObjectFile('nwplibc','',false);
     if s2 = '' then
       s2 := FindObjectFile('libcpre.gcc','',false);
   end else
     s2 := FindObjectFile('nwpre','',false);
  Comment (V_Debug,'adding Object File '+s2);
  {$ifndef netware} LinkRes.Add (s2); {$else} LinkRes.Add (FExpand(s2)); {$endif}

  if target_info.system = system_i386_netwlibc then
   begin
     if isDll then  {needed to provide main}
       s2 := FindObjectFile('nwl_dlle','',false)
     else
       s2 := FindObjectFile('nwl_main','',false);
     Comment (V_Debug,'adding Object File '+s2);
     {$ifndef netware} LinkRes.Add (s2); {$else} LinkRes.Add (FExpand(s2)); {$endif}
    end;

  { main objectfiles, add to linker input }
  while not ObjectFiles.Empty do
  begin
    s:=ObjectFiles.GetFirst;
    if s<>'' then
    begin
      s2 := FindObjectFile (s,'',false);
      Comment (V_Debug,'adding Object File '+s2);
      {$ifndef netware} LinkRes.Add (s2); {$else} LinkRes.Add (FExpand(s2)); {$endif}
    end;
  end;
  LinkRes.Add (')');

  { output file (nlm), add to nlmconv }
  {$ifndef netware}
  NLMConvLinkFile.Add ('OUTPUT ' + NlmNam);
  {$else}
  NLMConvLinkFile.Add ('OUTPUT ' + FExpand(NlmNam));
  {$endif}

  { start and stop-procedures }

  if target_info.system = system_i386_netwlibc then
    begin
      NLMConvLinkFile.Add ('START _LibCPrelude');
      NLMConvLinkFile.Add ('EXIT _LibCPostlude');
      NLMConvLinkFile.Add ('CHECK _LibCCheckUnload');
      NLMConvLinkFile.Add ('REENTRANT');            { needed by older libc versions }
    end else
    begin
      NLMConvLinkFile.Add ('START _Prelude');  { defined in rtl/netware/nwpre.as }
      NLMConvLinkFile.Add ('EXIT _Stop');                             { nwpre.as }
      NLMConvLinkFile.Add ('CHECK FPC_NW_CHECKFUNCTION');            { system.pp }
    end;


  if not (cs_link_strip in current_settings.globalswitches) then
  begin
    NLMConvLinkFile.Add ('DEBUG');
    Comment(V_Debug,'DEBUG');
  end;

  { Write staticlibraries }
  if not StaticLibFiles.Empty then
   begin
     LinkRes.Add ('GROUP(');
     While not StaticLibFiles.Empty do
      begin
        S:=lower (StaticLibFiles.GetFirst);
        if s<>'' then
        begin
          {ad: that's a hack !
           whith -XX we get the .a files as static libs (in addition to the
           imported libraries}
         if (pos ('.a',s) <> 0) OR (pos ('.A', s) <> 0) then
         begin
           S2 := FindObjectFile(s,'',false);
           {$ifndef netware} LinkRes.Add (s2); {$else} LinkRes.Add (FExpand(s2)); {$endif}
           Comment(V_Debug,'adding Object File (StaticLibFiles) '+S2);
         end else
         begin
           i:=Pos(target_info.staticlibext,S);
           if i>0 then
             Delete(S,i,255);
           S := S + '.imp'; S2 := '';
           librarysearchpath.FindFile(S,false,S2);
           {$ifdef netware}
           Comment(V_Debug,'IMPORT @'+s2);
           s2 := FExpand (S2);
           {$endif}
           NLMConvLinkFile.Add('IMPORT @'+S2);
           Comment(V_Debug,'IMPORT @'+s2);
         end;
        end
      end;
      LinkRes.Add (')');
   end;

  if not SharedLibFiles.Empty then
   begin
     While not SharedLibFiles.Empty do
      begin
        {because of upper/lower case mix, we may get duplicate
         names but nlmconv ignores that.
         Here we are setting the import-files for nlmconv. I.e. for
         the module clib or clib.nlm we add IMPORT @clib.imp and also
         the module clib.nlm (autoload)
         ? may it be better to set autoload's via StaticLibFiles ? }
        S:=lower (SharedLibFiles.GetFirst);
        if s<>'' then
         begin
           s2:=s;
           i:=Pos(target_info.sharedlibext,S);
           if i>0 then
             Delete(S,i,255);
           if s[1] = '!' then
           begin  // special, with ! only the imp will be included but no module is autoloaded, needed i.e. for netware.imp
             S := copy(S,2,255) + '.imp';
             librarysearchpath.FindFile(S,false,S3);
             {$ifdef netware}
             Comment(V_Debug,'IMPORT @'+S3);
             S3 := FExpand (S3);
             {$endif}
             NLMConvLinkFile.Add('IMPORT @'+S3);
             Comment(V_Debug,'IMPORT @'+S3);
           end else
           begin
             S := S + '.imp';
             librarysearchpath.FindFile(S,false,S3);
             {$ifdef netware}
             Comment(V_Debug,'IMPORT @'+S3);
             S3 := FExpand (S3);
             {$endif}
             NLMConvLinkFile.Add('IMPORT @'+S3);
             NLMConvLinkFile.Add('MODULE '+s2);
             Comment(V_Debug,'MODULE '+S2);
             Comment(V_Debug,'IMPORT @'+S3);
           end;
         end;
      end;
   end;

  { write exports }
  hp2:=texported_item(current_module._exports.first);
  while assigned(hp2) do
   begin
     if not hp2.is_var then
      begin
        { Export the Symbol }
        Comment(V_Debug,'EXPORT '+hp2.name^);
        NLMConvLinkFile.Add ('EXPORT '+hp2.name^);
      end
     else
      { really, i think it is possible }
      Message1(parser_e_no_export_of_variables_for_target,'netware');
     hp2:=texported_item(hp2.next);
   end;

{ Write and Close response for ld, response for nlmconv is in NLMConvLinkFile(not written) }
  linkres.writetodisk;
  LinkRes.Free;

{ pass options from -k to nlmconv, ; is interpreted as newline }
  s := ParaLinkOptions;
  while(Length(s) > 0) and (s[1] = ' ') do
    delete (s,1,1);
  p := pos ('"',s);
  while p > 0 do
  begin
    delete (s,p,1);
    p := pos ('"',s);
  end;

  p := pos (';',s);
  while p > 0 do
  begin
    s2 := copy(s,1,p-1);
    comment (V_Debug,'adding "'+s2+'" to nlmvonv input');
    NLMConvLinkFile.Add(s2);
    delete (s,1,p);
    p := pos (';',s);
  end;
  if s <> '' then
  begin
    comment (V_Debug,'adding "'+s+'" to nlmvonv input');
    NLMConvLinkFile.Add(s);
  end;

  WriteResponseFile:=True;
end;


function TLinkerNetware.MakeExecutable:boolean;
var
  binstr,
  cmdstr   : TCmdStr;
  success  : boolean;
  StripStr : string[2];
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

{ Create some replacements }
  StripStr:='';

  if (cs_link_strip in current_settings.globalswitches) then
   StripStr:='-s';

{ Write used files and libraries and create Headerfile for
  nlmconv in NLMConvLinkFile }
  WriteResponseFile(false);

{ Call linker, this will generate a new object file that will be passed
  to nlmconv. Otherwise we could not create nlms without debug info }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
  Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$TMPOBJ',maybequoted(outputexedir+tmpLinkFileName));
  Comment (v_debug,'Executing '+BinStr+' '+cmdstr);
  success:=DoExec(BinStr,CmdStr,true,false);

  { Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    DeleteFile(outputexedir+Info.ResName);

{ Call nlmconv }
  if success then
  begin
    NLMConvLinkFile.writetodisk;
    NLMConvLinkFile.Free;
    SplitBinCmd(Info.ExeCmd[2],binstr,cmdstr);
    Replace(cmdstr,'$RES',maybequoted(outputexedir+'n'+Info.ResName));
    Comment (v_debug,'Executing '+BinStr+' '+cmdstr);
    success:=DoExec(BinStr,CmdStr,true,false);
    if (success) and not(cs_link_nolink in current_settings.globalswitches) then
    begin
      DeleteFile(outputexedir+'n'+Info.ResName);
      DeleteFile(outputexedir+tmpLinkFileName);
    end;
  end;

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


{****************************************************************************
                            TInternalLinkerNetware
****************************************************************************}

    constructor TInternalLinkerNetware.Create;
      begin
        inherited Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        CArObjectReader:=TArObjectReader;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        CExeoutput:=TNLMexeoutput;
        CObjInput:=TNLMCoffObjInput;
        nlmSpecialSymbols_Segments := TFPHashList.create;
      end;

    destructor TInternalLinkerNetware.destroy;
      begin
        if assigned(nlmSpecialSymbols_Segments) then
          begin
            nlmSpecialSymbols_Segments.Free;
            nlmSpecialSymbols_Segments := nil;
          end;
        inherited destroy;
      end;

    procedure TInternalLinkerNetware.DefaultLinkScript;
      var
        s,s2 : TCmdStr;
        secname,
        secnames : string;
        hasCopyright,
        hasScreenname,
        hasThreadname,
        hasVersion,
        hasDescription,
        hasStacksize: boolean;
        t : text;



        procedure addLinkerOption(s : string);
        var op : string;
        begin
          if s = '' then exit;
          if s[1]  = '#' then exit;
          LinkScript.Concat(s);
          op := upper(GetToken(s,' '));
          {check for options via -k that can also be specified vie
           compiler directives in source, -k options will override
           options in source}
          if op = 'COPYRIGHT' then hasCopyright := true else
          if op = 'SCREENNAME' then hasScreenname := true else
          if op = 'THREADNAME' then hasThreadname := true else
          if op = 'VERSION' then hasVersion := true else
          if op = 'DESCRIPTION' then hasDescription := true else
          if (op = 'STACK') or (op = 'STACKSIZE') then hasStacksize := true;
        end;

        { add linker scropt specified by -k@FileName }
        procedure addLinkerOptionsFile (fileName : string);
        var
          t : text;
          option : string;
          fn : TCmdStr;
        begin
          fn := fileName;
          if not sysutils.fileExists(fn) then
            if not includesearchpath.FindFile(fileName,true,fn) then
            begin
              comment(v_error,'linker options file "'+fileName+'" not found');
              exit;
            end;
          assign(t,fn); reset(t);
          while not eof(t) do
            begin
              readln(t,option);
              addLinkerOption(option);
            end;
          close(t);
        end;

        { add  linker options specified by command line parameter -k }
        procedure addLinkerOptions;
        var
          s,option : string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
          p : integer;
>>>>>>> graemeg/cpstrnew
=======
          p : integer;
>>>>>>> graemeg/cpstrnew
=======
          p : integer;
>>>>>>> graemeg/cpstrnew
=======
          p : integer;
>>>>>>> origin/cpstrnew
        begin
          s := ParaLinkOptions;
          option := GetToken(s,';');
          while option <> '' do
          begin
            if copy(option,1,1)='@' then
            begin
              delete(option,1,1);
              addLinkerOptionsFile(option);
            end else
              addLinkerOption(option);
            option := GetToken(s,';');
          end;
        end;

        { default: nwpre but can be specified via linker options
          bacuse this has to be the first object, we have to scan
          linker options before adding other options }

        function findPreludeInFile (fileName : string):string;
        var
          t : text;
          option,s : string;
          fn : TCmdStr;
        begin
          result := '';
          fn := fileName;
          if not sysutils.fileExists(fn) then
            if not includesearchpath.FindFile(fileName,true,fn) then
            begin
              comment(v_error,'linker options file "'+fileName+'" not found');
              exit;
            end;
          assign(t,fn); reset(t);
          while not eof(t) do
            begin
              readln(t,option);
              option := upper(GetToken(s,' '));
              if option='PRELUDE' then
                begin
                  result := getToken(s,' ');
                  close(t);
                  exit;
                end;
            end;
          close(t);
        end;

        function findPrelude : string;
        var
          s,option,keyword : string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
          p : integer;
>>>>>>> graemeg/cpstrnew
=======
          p : integer;
>>>>>>> graemeg/cpstrnew
=======
          p : integer;
>>>>>>> graemeg/cpstrnew
=======
          p : integer;
>>>>>>> origin/cpstrnew
        begin
          s := ParaLinkOptions;
          option := GetToken(s,';');
          while option <> '' do
          begin
            if copy(option,1,1)='@' then
            begin
              delete(option,1,1);
              result := findPreludeInFile(option);
              if result <> '' then exit;
            end else
            begin
              keyword := GetToken(option,' ');
              if keyword = 'PRELUDE' then
                begin
                  result := GetToken(option,' ');
                  exit;
                end;
            end;
            option := GetToken(s,';');
          end;
          if target_info.system = system_i386_netwlibc then
            result := 'libcpre'
          else
            result := 'nwpre';
        end;

      begin
        with LinkScript do
          begin
            prelude := findPrelude;  // needs to be first object, can be specified by -k"PRELUDE ObjFileName"
            if prelude = '' then internalerror(201103271);
            if pos ('.',prelude) = 0 then prelude := prelude + '.o';
            s2 := FindObjectFile(prelude,'',false);
            Comment (V_Debug,'adding init Object File '+s2);
            Concat('READOBJECT '+MaybeQuoted(s2));
            while not ObjectFiles.Empty do
              begin
                s:=ObjectFiles.GetFirst;
                if s<>'' then
                begin
                  Concat('READOBJECT '+MaybeQuoted(s));
                  Comment (V_Debug,'adding Object File '+s);
                end;
              end;
            while not StaticLibFiles.Empty do
              begin
                s:=StaticLibFiles.GetFirst;
                if s<>'' then
                begin
                  Comment (V_Debug,'adding StaticLibFile '+s);
                  Concat('READSTATICLIBRARY '+MaybeQuoted(s));
                end;
              end;
           { While not SharedLibFiles.Empty do
              begin
                S:=SharedLibFiles.GetFirst;
                if FindLibraryFile(s,target_info.staticClibprefix,target_info.importlibext,s2) then
                begin
                  Comment (V_Debug,'adding LibraryFile '+s);
                  Concat('READSTATICLIBRARY '+MaybeQuoted(s2));
                end else
                  Comment(V_Error,'Import library not found for '+S);
              end;}
            if IsSharedLibrary then
              Concat('ISSHAREDLIBRARY');
            ConcatEntryName;
            Concat('IMAGEBASE $' + hexStr(0, SizeOf(imagebase)*2));
            Concat('HEADER');
            Concat('EXESECTION .text');
            Concat('  SYMBOL __text_start__');  nlmSpecialSymbols_Segments.Add('__text_start__',pointer(ptruint(Section_text)));
            Concat('  OBJSECTION .text*');
            Concat('  SYMBOL ___CTOR_LIST__');  nlmSpecialSymbols_Segments.Add('___CTOR_LIST__',pointer(ptruint(Section_text)));
            Concat('  SYMBOL __CTOR_LIST__');   nlmSpecialSymbols_Segments.Add('__CTOR_LIST__',pointer(ptruint(Section_text)));
            Concat('  LONG -1');
            Concat('  OBJSECTION .ctor*');
            Concat('  LONG 0');
            Concat('  SYMBOL ___DTOR_LIST__');  nlmSpecialSymbols_Segments.Add('___DTOR_LIST__',pointer(ptruint(Section_text)));
            Concat('  SYMBOL __DTOR_LIST__');   nlmSpecialSymbols_Segments.Add('__DTOR_LIST__',pointer(ptruint(Section_text)));
            Concat('  LONG -1');
            Concat('  OBJSECTION .dtor*');
            Concat('  LONG 0');
            Concat('  SYMBOL etext');           nlmSpecialSymbols_Segments.Add('etext',pointer(ptruint(Section_text)));
            Concat('ENDEXESECTION');

            Concat('EXESECTION .data');
            Concat('  SYMBOL __data_start__');  nlmSpecialSymbols_Segments.Add('__data_start__',pointer(ptruint(Section_data)));
            Concat('  OBJSECTION .data*');
            Concat('  OBJSECTION .fpc*');
            Concat('  SYMBOL edata');           nlmSpecialSymbols_Segments.Add('edata',pointer(ptruint(Section_data)));
            Concat('  SYMBOL __data_end__');    nlmSpecialSymbols_Segments.Add('__data_end__',pointer(ptruint(Section_data)));
            Concat('ENDEXESECTION');

            Concat('EXESECTION .bss');
            Concat('  SYMBOL __bss_start__');   nlmSpecialSymbols_Segments.Add('__bss_start__',pointer(ptruint(Section_data)));
            Concat('  OBJSECTION .bss*');
            Concat('  SYMBOL __bss_end__');     nlmSpecialSymbols_Segments.Add('__bss_end__',pointer(ptruint(Section_data)));
            Concat('ENDEXESECTION');

            Concat('EXESECTION .imports');
            Concat('  SYMBOL __imports_start__');
            Concat('  OBJSECTION .imports*');
            Concat('  SYMBOL __imports_end__');
            Concat('ENDEXESECTION');

            Concat('EXESECTION .modules');
            Concat('  SYMBOL __modules_start__');
            Concat('  OBJSECTION .modules*');
            Concat('  SYMBOL __modules_end__');
            Concat('ENDEXESECTION');

            Concat('EXESECTION .exports');
            Concat('  SYMBOL __exports_start__');
            Concat('  OBJSECTION .exports*');
            Concat('  SYMBOL __exports_end__');
            Concat('ENDEXESECTION');

            Concat('EXESECTION .reloc');
            Concat('  SYMBOL __reloc_start__');
            Concat('  OBJSECTION .reloc*');
            Concat('  SYMBOL __reloc_end__');
            Concat('ENDEXESECTION');

            Concat('EXESECTION .xdc');
            Concat('  OBJSECTION .xdc*');
            Concat('ENDEXESECTION');

            Concat('EXESECTION .custom');
            Concat('  OBJSECTION .custom*');
            Concat('ENDEXESECTION');

            Concat('EXESECTION .messages');
            Concat('  OBJSECTION .messages*');
            Concat('ENDEXESECTION');

            Concat('EXESECTION .help');
            Concat('  OBJSECTION .help*');
            Concat('ENDEXESECTION');

            Concat('EXESECTION .rdata');
            Concat('  SYMBOL ___RUNTIME_PSEUDO_RELOC_LIST__');
            Concat('  SYMBOL __RUNTIME_PSEUDO_RELOC_LIST__');
            Concat('  OBJSECTION .rdata_runtime_pseudo_reloc');
            Concat('  SYMBOL ___RUNTIME_PSEUDO_RELOC_LIST_END__');
            Concat('  SYMBOL __RUNTIME_PSEUDO_RELOC_LIST_END__');
            Concat('  OBJSECTION .rdata*');
            Concat('  OBJSECTION .rodata*');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .pdata');
            Concat('  OBJSECTION .pdata');
            Concat('ENDEXESECTION');
            secnames:='.edata,.rsrc,.gnu_debuglink,'+
                      '.debug_aranges,.debug_pubnames,.debug_info,.debug_abbrev,.debug_line,.debug_frame,.debug_str,.debug_loc,'+
                      '.debug_macinfo,.debug_weaknames,.debug_funcnames,.debug_typenames,.debug_varnames,.debug_ranges';
            repeat
              secname:=gettoken(secnames,',');
              if secname='' then
                break;
              Concat('EXESECTION '+secname);
              Concat('  OBJSECTION '+secname+'*');
              Concat('ENDEXESECTION');
            until false;
            { Can't use the generic rules, because that will add also .stabstr to .stab }
            Concat('EXESECTION .stab');
            Concat('  OBJSECTION .stab');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .stabstr');
            Concat('  OBJSECTION .stabstr');
            Concat('ENDEXESECTION');
            Concat('STABS');
            Concat('SYMBOLS');
            Concat('');

            hasCopyright := false;
            hasScreenname := false;
            hasThreadname := false;
            hasVersion := false;
            hasDescription := false;
            hasStacksize := false;
            addLinkerOptions;
            if not hasCopyright then
              if nwcopyright <> '' then
                Concat('COPYRIGHT "'+nwCopyright+'"');
            if not hasScreenname then
              if nwscreenname <> '' then
                Concat('SCREENNAME "'+nwscreenname+'"');
            if not hasThreadname then
              if nwthreadname <> '' then
                Concat('THREADNAME "'+nwthreadname+'"');
            if not hasVersion then
              Concat('VERSION '+tostr(dllmajor)+' '+tostr(dllminor)+' '+tostr(dllrevision));
            if not hasDescription then
              if description <> '' then
                Concat ('DESCRIPTION "'+description+'"');
            if not hasStacksize then
              if MaxStackSizeSetExplicity then
              begin
                if stacksize < minStackSize then stacksize := minStackSize;
                Concat ('STACKSIZE '+tostr(stacksize));
              end else
                Concat ('STACKSIZE '+tostr(minStackSize));
              if target_info.system = system_i386_netwlibc then
                Concat ('REENTRANT');            { needed by older libc versions }
          end;

        // add symbols needed by nwpre. We have not loaded the ppu,
        // therefore we do not know the externals so read it from nwpre.imp
        s := ChangeFileExt(prelude,'.imp');  // nwpre.imp
        if not librarysearchpath.FindFile(s,true,s2) then
          begin
            comment(v_error,s+' not found');
            exit;
          end;
        assign(t,s2); reset(t);
        while not eof(t) do
          begin
            readln(t,s);
            s := trimspace(s);
            if (length(s) > 0) then
              if copy(s,1,1) <> '#' then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                AddImportSymbol('!clib',s,s,0,false);
=======
                AddImportSymbol('!clib',s,0,false);
>>>>>>> graemeg/cpstrnew
=======
                AddImportSymbol('!clib',s,0,false);
>>>>>>> graemeg/cpstrnew
=======
                AddImportSymbol('!clib',s,0,false);
>>>>>>> graemeg/cpstrnew
=======
                AddImportSymbol('!clib',s,0,false);
>>>>>>> origin/cpstrnew
          end;
        close(t);
      end;


    procedure TInternalLinkerNetware.InitSysInitUnitName;
      begin
        //if target_info.system=system_i386_netware then
        //  GlobalInitSysInitUnitName(self);
      end;

    procedure TInternalLinkerNetware.ConcatEntryName;
      begin
        with LinkScript do
          begin
            if IsSharedLibrary then
              begin
                Concat('ISSHAREDLIBRARY');
                Concat('ENTRYNAME _Prelude')
              end
            else
              begin
                Concat('ENTRYNAME _Prelude')
              end;
          end;
      end;


    Function  TInternalLinkerNetware.MakeSharedLibrary:boolean;
    begin
      Comment(V_Error,'Make shared library not supported for netware');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      MakeSharedLibrary := false;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;

{*****************************************************************************
                                     Initialize
*****************************************************************************}


initialization
<<<<<<< HEAD
  RegisterLinker(ld_netware,TLinkerNetware);
  RegisterLinker(ld_int_netware,TInternalLinkerNetware);
=======
  RegisterExternalLinker(system_i386_netware_info,TLinkerNetware);
  RegisterInternalLinker(system_i386_netware_info,TInternalLinkerNetware);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  RegisterImport(system_i386_netware,TImportLibNetware);
  RegisterExport(system_i386_netware,TExportLibNetware);
  RegisterTarget(system_i386_netware_info);
end.
