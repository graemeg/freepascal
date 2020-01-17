{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the first loading and searching of the modules

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
unit fmodule;

{$i fpcdefs.inc}

{$ifdef go32v2}
  {$define shortasmprefix}
{$endif}
{$ifdef watcom}
  {$define shortasmprefix}
{$endif}
{$ifdef atari}
  {$define shortasmprefix}
{$endif}
{$ifdef OS2}
  { Allthough OS/2 supports long filenames I play it safe and
    use 8.3 filenames, because this allows the compiler to run
    on a FAT partition. (DM) }
  {$define shortasmprefix}
{$endif}

interface

    uses
       cutils,cclasses,cfileutl,
       globtype,finput,ogbase,fpkg,
       symbase,symsym,
       wpobase,
       aasmbase,aasmdata;


    const
      UNSPECIFIED_LIBRARY_NAME = '<none>';

    type
      trecompile_reason = (rr_unknown,
        rr_noppu,rr_sourcenewer,rr_build,rr_crcchanged
      );

      { unit options }
      tmoduleoption = (mo_none,
        mo_hint_deprecated,
        mo_hint_platform,
        mo_hint_library,
        mo_hint_unimplemented,
        mo_hint_experimental,
        mo_has_deprecated_msg
      );
      tmoduleoptions = set of tmoduleoption;

      tlinkcontaineritem=class(tlinkedlistitem)
      public
         data : TPathStr;
         needlink : cardinal;
         constructor Create(const s:TPathStr;m:cardinal);
      end;

      tlinkcontainer=class(tlinkedlist)
         procedure add(const s : TPathStr;m:cardinal);
         function get(var m:cardinal) : TPathStr;
         function getusemask(mask:cardinal) : TPathStr;
         function find(const s:TPathStr):boolean;
      end;

      tmodule = class;
      tused_unit = class;

      tunitmaprec = record
        u        : tmodule;
        { number of references }
        refs     : longint;
        { index in the derefmap }
        derefidx : longint;
      end;
      punitmap = ^tunitmaprec;

      tderefmaprec = record
        u           : tmodule;
        { modulename, used during ppu load }
        modulename  : pshortstring;
      end;
      pderefmap = ^tderefmaprec;

      { tmodule }

      tmodule = class(tmodulebase)
      private
        FImportLibraryList : TFPHashObjectList;
      public
        do_reload,                { force reloading of the unit }
        do_compile,               { need to compile the sources }
        sources_avail,            { if all sources are reachable }
        interface_compiled,       { if the interface section has been parsed/compiled/loaded }
        is_dbginfo_written,
        is_unit,
        in_interface,             { processing the implementation part? }
        { allow global settings }
        in_global     : boolean;
        { Whether a mode switch is still allowed at this point in the parsing.}
        mode_switch_allowed,
        { generate pic helper which loads eip in ecx (for leave procedures) }
        requires_ecx_pic_helper,
        { generate pic helper which loads eip in ebx (for non leave procedures) }
        requires_ebx_pic_helper : boolean;
        interface_only: boolean; { interface-only macpas unit; flag does not need saving/restoring to ppu }
        mainfilepos   : tfileposinfo;
        recompile_reason : trecompile_reason;  { the reason why the unit should be recompiled }
        crc,
        interface_crc,
        indirect_crc  : cardinal;
        headerflags   : cardinal;  { the PPU header flags }
        longversion   : cardinal;  { longer version than what fits in the ppu header }
        moduleflags   : tmoduleflags; { ppu flags that do not need to be known by just reading the ppu header }
        islibrary     : boolean;  { if it is a library (win32 dll) }
        IsPackage     : boolean;
        moduleid      : longint;
        unitmap       : punitmap; { mapping of all used units }
        unitmapsize   : longint;  { number of units in the map }
        derefmap      : pderefmap; { mapping of all units needed for deref }
        derefmapcnt   : longint;  { number of units in the map }
        derefmapsize  : longint;  { number of units in the map }
        derefdataintflen : longint;
        derefdata     : tdynamicarray;
        checkforwarddefs,
        deflist,
        symlist       : TFPObjectList;
        ptrdefs       : THashSet; { list of pointerdefs created in this module so we can reuse them (not saved/restored) }
        arraydefs     : THashSet; { list of single-element-arraydefs created in this module so we can reuse them (not saved/restored) }
        procaddrdefs  : THashSet; { list of procvardefs created when getting the address of a procdef (not saved/restored) }
{$ifdef llvm}
        llvmdefs      : THashSet; { defs added for llvm-specific reasons (not saved/restored) }
        llvmusedsyms  : TFPObjectList; { a list of asmsymbols and their defs that need to be added to llvm.used (so they're not removed by llvm optimisation passes nor by the linker) }
        llvmcompilerusedsyms : TFPObjectList; { a list of asmsymbols and their defs that need to be added to llvm.compiler.used (so they're not removed by llvm optimisation passes) }
        llvminitprocs,
        llvmfiniprocs : TFPList;
        llvmmetadatastrings: TFPHashList; { metadata strings (mapping string -> superregister) }
{$endif llvm}
        ansistrdef    : tobject; { an ansistring def redefined for the current module }
        wpoinfo       : tunitwpoinfobase; { whole program optimization-related information that is generated during the current run for this unit }
        globalsymtable,           { pointer to the global symtable of this unit }
        localsymtable : TSymtable;{ pointer to the local symtable of this unit }
        globalmacrosymtable,           { pointer to the global macro symtable of this unit }
        localmacrosymtable : TSymtable;{ pointer to the local macro symtable of this unit }
        scanner       : TObject;  { scanner object used }
        procinfo      : TObject;  { current procedure being compiled }
        asmdata       : TObject;  { Assembler data }
        asmprefix     : pshortstring;  { prefix for the smartlink asmfiles }
        publicasmsyms : TFPHashObjectList; { contains the assembler symbols which need to be exported from a package }
        externasmsyms : TFPHashObjectList; { contains the assembler symbols which are imported from another unit }
        unitimportsyms : tfpobjectlist; { list of symbols that are imported from other units }
        debuginfo     : TObject;
        loaded_from   : tmodule;
        _exports      : tlinkedlist;
        dllscannerinputlist : TFPHashList;
        resourcefiles,
        linkorderedsymbols : TCmdStrList;
        linkunitofiles,
        linkunitstaticlibs,
        linkunitsharedlibs,
        linkotherofiles,           { objects,libs loaded from the source }
        linkothersharedlibs,       { using $L or $LINKLIB or import lib (for linux) }
        linkotherstaticlibs,
        linkotherframeworks  : tlinkcontainer;
        mainname      : pshortstring; { alternate name for "main" procedure }
        package       : tpackage;

        used_units           : tlinkedlist;
        dependent_units      : tlinkedlist;

        localunitsearchpath,           { local searchpaths }
        localobjectsearchpath,
        localincludesearchpath,
        locallibrarysearchpath,
        localframeworksearchpath : TSearchPathList;

        moduleoptions: tmoduleoptions;
        deprecatedmsg: pshortstring;

        { contains a list of types that are extended by helper types; the key is
          the full name of the type and the data is a TFPObjectList of
          tobjectdef instances (the helper defs) }
        extendeddefs: TFPHashObjectList;
        { contains a list of the current topmost non-generic symbol for a
          typename of which at least one generic exists; the key is the
          non-generic typename and the data is a TFPObjectList of tgenericdummyentry
          instances whereby the last one is the current top most one }
        genericdummysyms: TFPHashObjectList;
        { contains a list of specializations for which the method bodies need
          to be generated }
        pendingspecializations : TFPHashObjectList;
        { list of attributes that are used and thus need their construction
          functions generated }
        used_rtti_attrs: tfpobjectlist;

        { this contains a list of units that needs to be waited for until the
          unit can be finished (code generated, etc.); this is needed to handle
          specializations in circular unit usages correctly }
        waitingforunit: tfpobjectlist;
        { this contains a list of all units that are waiting for this unit to be
          finished }
        waitingunits: tfpobjectlist;

        finishstate: pointer;
        globalstate: pointer;

        namespace: pshortstring; { for JVM target: corresponds to Java package name }

        { for targets that initialise typed constants via explicit assignments
          instead of by generating an initialised data section (holds typed
          constant assignments at the module level; does not have to be saved
          into the ppu file, because translated into code during compilation)
           -- actual type: tnode (but fmodule should not depend on node) }
         tcinitcode     : tobject;

        {create creates a new module which name is stored in 's'. LoadedFrom
        points to the module calling it. It is nil for the first compiled
        module. This allow inheritence of all path lists. MUST pay attention
        to that when creating link.res!!!!(mazen)}
        constructor create(LoadedFrom:TModule;const amodulename: string; const afilename:TPathStr;_is_unit:boolean);
        destructor destroy;override;
        procedure reset;virtual;
        procedure adddependency(callermodule:tmodule);
        procedure flagdependent(callermodule:tmodule);
        procedure addimportedsym(sym:TSymEntry);
        function  addusedunit(hp:tmodule;inuses:boolean;usym:tunitsym):tused_unit;
        procedure updatemaps;
        function  derefidx_unit(id:longint):longint;
        function  resolve_unit(id:longint):tmodule;
        procedure allunitsused;
        procedure end_of_parsing;virtual;
        procedure setmodulename(const s:string);
        procedure AddExternalImport(const libname,symname,symmangledname:string;OrdNr: longint;isvar:boolean;ImportByOrdinalOnly:boolean);
        procedure add_public_asmsym(sym:TAsmSymbol);
        procedure add_public_asmsym(const name:TSymStr;bind:TAsmsymbind;typ:Tasmsymtype);
        procedure add_extern_asmsym(sym:TAsmSymbol);
        procedure add_extern_asmsym(const name:TSymStr;bind:TAsmsymbind;typ:Tasmsymtype);
        property ImportLibraryList : TFPHashObjectList read FImportLibraryList;
      end;

       tused_unit = class(tlinkedlistitem)
          checksum,
          interface_checksum,
          indirect_checksum: cardinal;
          in_uses,
          in_interface    : boolean;
          u               : tmodule;
          unitsym         : tunitsym;
          constructor create(_u : tmodule;intface,inuses:boolean;usym:tunitsym);
          procedure check_hints;
       end;

       tdependent_unit = class(tlinkedlistitem)
          u : tmodule;
          constructor create(_u : tmodule);
       end;

    var
       main_module       : tmodule;     { Main module of the program }
       current_module    : tmodule;     { Current module which is compiled or loaded }
       compiled_module   : tmodule;     { Current module which is compiled }
       usedunits         : tlinkedlist; { Used units for this program }
       loaded_units      : tlinkedlist; { All loaded units }
       unloaded_units    : tlinkedlist; { Units removed from loaded_units, to be freed }
       SmartLinkOFiles   : TCmdStrList; { List of .o files which are generated,
                                          used to delete them after linking }


    procedure set_current_module(p:tmodule);
    function get_module(moduleindex : longint) : tmodule;
    function get_source_file(moduleindex,fileindex : longint) : tinputfile;
    procedure addloadedunit(hp:tmodule);
    function find_module_from_symtable(st:tsymtable):tmodule;


implementation

    uses
      SysUtils,globals,
      verbose,systems,
      scanner,ppu,dbgbase,
      procinfo,symdef;

{$ifdef MEMDEBUG}
    var
      memsymtable : TMemDebug;
{$endif}

{*****************************************************************************
                             Global Functions
*****************************************************************************}

    function find_module_from_symtable(st:tsymtable):tmodule;
      var
        hp : tmodule;
      begin
        result:=nil;
        hp:=tmodule(loaded_units.first);
        while assigned(hp) do
          begin
            if (hp.moduleid=st.moduleid) then
              begin
                result:=hp;
                exit;
              end;
            hp:=tmodule(hp.next);
         end;
      end;

    procedure set_current_module(p:tmodule);
      begin
        { save the state of the scanner }
        if assigned(current_scanner) then
          current_scanner.tempcloseinputfile;
        { set new module }
        current_module:=p;
        { restore previous module settings }
        Fillchar(current_filepos,0,sizeof(current_filepos));
        if assigned(current_module) then
          begin
            current_asmdata:=tasmdata(current_module.asmdata);
            current_debuginfo:=tdebuginfo(current_module.debuginfo);
            { restore scanner and file positions }
            current_scanner:=tscannerfile(current_module.scanner);
            if assigned(current_scanner) then
              begin
                current_scanner.tempopeninputfile;
                current_scanner.gettokenpos;
                parser_current_file:=current_scanner.inputfile.name;
              end
            else
              begin
                current_filepos.moduleindex:=current_module.unit_index;
                parser_current_file:='';
              end;
          end
        else
          begin
            current_asmdata:=nil;
            current_scanner:=nil;
            current_debuginfo:=nil;
          end;
      end;


    function get_module(moduleindex : longint) : tmodule;
      var
         hp : tmodule;
      begin
         result:=nil;
         if moduleindex=0 then
           exit;
         result:=current_module;
         if not(assigned(loaded_units)) then
           exit;
         hp:=tmodule(loaded_units.first);
         while assigned(hp) and (hp.unit_index<>moduleindex) do
           hp:=tmodule(hp.next);
         result:=hp;
      end;


    function get_source_file(moduleindex,fileindex : longint) : tinputfile;
      var
         hp : tmodule;
      begin
         hp:=get_module(moduleindex);
         if assigned(hp) then
          get_source_file:=hp.sourcefiles.get_file(fileindex)
         else
          get_source_file:=nil;
      end;


    procedure addloadedunit(hp:tmodule);
      begin
        hp.moduleid:=loaded_units.count;
        loaded_units.concat(hp);
      end;


{****************************************************************************
                             TLinkContainerItem
 ****************************************************************************}

    constructor TLinkContainerItem.Create(const s:TPathStr;m:cardinal);
      begin
        inherited Create;
        data:=s;
        needlink:=m;
      end;


{****************************************************************************
                           TLinkContainer
 ****************************************************************************}

    procedure TLinkContainer.add(const s : TPathStr;m:cardinal);
      begin
        inherited concat(TLinkContainerItem.Create(s,m));
      end;


    function TLinkContainer.get(var m:cardinal) : TPathStr;
      var
        p : tlinkcontaineritem;
      begin
        p:=tlinkcontaineritem(inherited getfirst);
        if p=nil then
         begin
           get:='';
           m:=0;
         end
        else
         begin
           get:=p.data;
           m:=p.needlink;
           p.free;
         end;
      end;


    function TLinkContainer.getusemask(mask:cardinal) : TPathStr;
      var
         p : tlinkcontaineritem;
         found : boolean;
      begin
        found:=false;
        repeat
          p:=tlinkcontaineritem(inherited getfirst);
          if p=nil then
           begin
             getusemask:='';
             exit;
           end;
          getusemask:=p.data;
          found:=(p.needlink and mask)<>0;
          p.free;
        until found;
      end;


    function TLinkContainer.find(const s:TPathStr):boolean;
      var
        newnode : tlinkcontaineritem;
      begin
        find:=false;
        newnode:=tlinkcontaineritem(First);
        while assigned(newnode) do
         begin
           if newnode.data=s then
            begin
              find:=true;
              exit;
            end;
           newnode:=tlinkcontaineritem(newnode.next);
         end;
      end;


{****************************************************************************
                              TUSED_UNIT
 ****************************************************************************}

    constructor tused_unit.create(_u : tmodule;intface,inuses:boolean;usym:tunitsym);
      begin
        u:=_u;
        in_interface:=intface;
        in_uses:=inuses;
        unitsym:=usym;
        if _u.state=ms_compiled then
         begin
           checksum:=u.crc;
           interface_checksum:=u.interface_crc;
           indirect_checksum:=u.indirect_crc;
         end
        else
         begin
           checksum:=0;
           interface_checksum:=0;
           indirect_checksum:=0;
         end;
      end;


    procedure tused_unit.check_hints;
      var
        uname: pshortstring;
      begin
        uname:=u.realmodulename;
        if mo_hint_deprecated in u.moduleoptions then
          if (mo_has_deprecated_msg in u.moduleoptions) and (u.deprecatedmsg <> nil) then
            MessagePos2(unitsym.fileinfo,sym_w_deprecated_unit_with_msg,uname^,u.deprecatedmsg^)
          else
            MessagePos1(unitsym.fileinfo,sym_w_deprecated_unit,uname^);
        if mo_hint_experimental in u.moduleoptions then
          MessagePos1(unitsym.fileinfo,sym_w_experimental_unit,uname^);
        if mo_hint_platform in u.moduleoptions then
          MessagePos1(unitsym.fileinfo,sym_w_non_portable_unit,uname^);
        if mo_hint_library in u.moduleoptions then
          MessagePos1(unitsym.fileinfo,sym_w_library_unit,uname^);
        if mo_hint_unimplemented in u.moduleoptions then
          MessagePos1(unitsym.fileinfo,sym_w_non_implemented_unit,uname^);
      end;


{****************************************************************************
                            TDENPENDENT_UNIT
 ****************************************************************************}

    constructor tdependent_unit.create(_u : tmodule);
      begin
         u:=_u;
      end;


{****************************************************************************
                                  TMODULE
 ****************************************************************************}

    constructor tmodule.create(LoadedFrom:TModule;const amodulename: string; const afilename:TPathStr;_is_unit:boolean);
      var
        n:string;
        fn:TPathStr;
      begin
        if amodulename='' then
          n:=ChangeFileExt(ExtractFileName(afilename),'')
        else
          n:=amodulename;
        if afilename='' then
          fn:=amodulename
        else
          fn:=afilename;
        { Programs have the name 'Program' to don't conflict with dup id's }
        if _is_unit then
         inherited create(amodulename)
        else
         inherited create('Program');
        mainsource:=fn;
        { Dos has the famous 8.3 limit :( }
{$ifdef shortasmprefix}
        asmprefix:=stringdup(FixFileName('as'));
{$else}
        asmprefix:=stringdup(FixFileName(n));
{$endif}
        setfilename(fn,true);
        localunitsearchpath:=TSearchPathList.Create;
        localobjectsearchpath:=TSearchPathList.Create;
        localincludesearchpath:=TSearchPathList.Create;
        locallibrarysearchpath:=TSearchPathList.Create;
        localframeworksearchpath:=TSearchPathList.Create;
        used_units:=TLinkedList.Create;
        dependent_units:=TLinkedList.Create;
        resourcefiles:=TCmdStrList.Create;
        linkorderedsymbols:=TCmdStrList.Create;
        linkunitofiles:=TLinkContainer.Create;
        linkunitstaticlibs:=TLinkContainer.Create;
        linkunitsharedlibs:=TLinkContainer.Create;
        linkotherofiles:=TLinkContainer.Create;
        linkotherstaticlibs:=TLinkContainer.Create;
        linkothersharedlibs:=TLinkContainer.Create;
        linkotherframeworks:=TLinkContainer.Create;
        mainname:=nil;
        FImportLibraryList:=TFPHashObjectList.Create(true);
        crc:=0;
        interface_crc:=0;
        indirect_crc:=0;
        headerflags:=0;
        longversion:=0;
        moduleflags:=[];
        scanner:=nil;
        unitmap:=nil;
        unitmapsize:=0;
        derefmap:=nil;
        derefmapsize:=0;
        derefmapcnt:=0;
        derefdata:=TDynamicArray.Create(1024);
        derefdataintflen:=0;
        deflist:=TFPObjectList.Create(false);
        symlist:=TFPObjectList.Create(false);
        ptrdefs:=THashSet.Create(64,true,false);
        arraydefs:=THashSet.Create(64,true,false);
        procaddrdefs:=THashSet.Create(64,true,false);
{$ifdef llvm}
        llvmdefs:=THashSet.Create(64,true,false);
        llvmusedsyms:=TFPObjectList.Create(true);
        llvmcompilerusedsyms:=TFPObjectList.Create(true);
        llvminitprocs:=TFPList.Create;
        llvmfiniprocs:=TFPList.Create;
        llvmmetadatastrings:=TFPHashList.Create;
{$endif llvm}
        ansistrdef:=nil;
        wpoinfo:=nil;
        checkforwarddefs:=TFPObjectList.Create(false);
        extendeddefs:=TFPHashObjectList.Create(true);
        genericdummysyms:=tfphashobjectlist.create(true);
        pendingspecializations:=tfphashobjectlist.create(false);
        waitingforunit:=tfpobjectlist.create(false);
        waitingunits:=tfpobjectlist.create(false);
        used_rtti_attrs:=tfpobjectlist.create(false);
        globalsymtable:=nil;
        localsymtable:=nil;
        globalmacrosymtable:=nil;
        localmacrosymtable:=nil;
        loaded_from:=LoadedFrom;
        do_reload:=false;
        do_compile:=false;
        sources_avail:=true;
        mainfilepos.line:=0;
        mainfilepos.column:=0;
        mainfilepos.fileindex:=0;
        recompile_reason:=rr_unknown;
        in_interface:=true;
        in_global:=true;
        is_unit:=_is_unit;
        islibrary:=false;
        ispackage:=false;
        is_dbginfo_written:=false;
        mode_switch_allowed:= true;
        moduleoptions:=[];
        deprecatedmsg:=nil;
        namespace:=nil;
        tcinitcode:=nil;
        _exports:=TLinkedList.Create;
        dllscannerinputlist:=TFPHashList.Create;
        asmdata:=casmdata.create(modulename);
        unitimportsyms:=TFPObjectList.Create(false);
        publicasmsyms:=TFPHashObjectList.Create(true);
        externasmsyms:=TFPHashObjectList.Create(true);
        InitDebugInfo(self,false);
      end;


    destructor tmodule.Destroy;
      var
        i : longint;
        current_debuginfo_reset : boolean;
      begin
        if assigned(unitmap) then
          freemem(unitmap);
        if assigned(derefmap) then
          begin
            for i:=0 to derefmapcnt-1 do
              stringdispose(derefmap[i].modulename);
            freemem(derefmap);
          end;
        if assigned(_exports) then
         _exports.free;
        if assigned(dllscannerinputlist) then
         dllscannerinputlist.free;
        if assigned(scanner) then
         begin
            { also update current_scanner if it was pointing
              to this module }
            if current_scanner=tscannerfile(scanner) then
             current_scanner:=nil;
            tscannerfile(scanner).free;
         end;
        if assigned(asmdata) then
          begin
            if current_asmdata=asmdata then
              current_asmdata:=nil;
             asmdata.free;
          end;
        if assigned(procinfo) then
          begin
            if current_procinfo=tprocinfo(procinfo) then
              begin
                current_procinfo:=nil;
                current_structdef:=nil;
                current_genericdef:=nil;
                current_specializedef:=nil;
              end;
            { release procinfo tree }
            tprocinfo(procinfo).destroy_tree;
          end;
        DoneDebugInfo(self,current_debuginfo_reset);
        used_units.free;
        dependent_units.free;
        resourcefiles.Free;
        linkorderedsymbols.Free;
        linkunitofiles.Free;
        linkunitstaticlibs.Free;
        linkunitsharedlibs.Free;
        linkotherofiles.Free;
        linkotherstaticlibs.Free;
        linkothersharedlibs.Free;
        linkotherframeworks.Free;
        stringdispose(mainname);
        externasmsyms.Free;
        publicasmsyms.Free;
        unitimportsyms.Free;
        FImportLibraryList.Free;
        extendeddefs.Free;
        genericdummysyms.free;
        pendingspecializations.free;
        waitingforunit.free;
        waitingunits.free;
        used_rtti_attrs.free;
        stringdispose(asmprefix);
        stringdispose(deprecatedmsg);
        stringdispose(namespace);
        tcinitcode.free;
        localunitsearchpath.Free;
        localobjectsearchpath.free;
        localincludesearchpath.free;
        locallibrarysearchpath.free;
        localframeworksearchpath.free;
{$ifdef MEMDEBUG}
        memsymtable.start;
{$endif}
        derefdata.free;
        deflist.free;
        symlist.free;
        ptrdefs.free;
        arraydefs.free;
        procaddrdefs.free;
{$ifdef llvm}
        llvmdefs.free;
        llvmusedsyms.free;
        llvmcompilerusedsyms.free;
        llvminitprocs.free;
        llvmfiniprocs.free;
        llvmmetadatastrings.free;
{$endif llvm}
        ansistrdef:=nil;
        wpoinfo.free;
        checkforwarddefs.free;
        globalsymtable.free;
        localsymtable.free;
        globalmacrosymtable.free;
        localmacrosymtable.free;
{$ifdef MEMDEBUG}
        memsymtable.stop;
{$endif}
        inherited Destroy;
      end;


    procedure tmodule.reset;
      var
        i   : longint;
        current_debuginfo_reset : boolean;
      begin
        if assigned(scanner) then
          begin
            { also update current_scanner if it was pointing
              to this module }
            if current_scanner=tscannerfile(scanner) then
             current_scanner:=nil;
            tscannerfile(scanner).free;
            scanner:=nil;
          end;
        if assigned(procinfo) then
          begin
            if current_procinfo=tprocinfo(procinfo) then
              begin
                current_procinfo:=nil;
                current_structdef:=nil;
                current_genericdef:=nil;
                current_specializedef:=nil;
              end;
            { release procinfo tree }
            tprocinfo(procinfo).destroy_tree;
          end;
        if assigned(asmdata) then
          begin
            if current_asmdata=asmdata then
             current_asmdata:=nil;
            asmdata.free;
            asmdata:=nil;
          end;
        DoneDebugInfo(self,current_debuginfo_reset);
        globalsymtable.free;
        globalsymtable:=nil;
        localsymtable.free;
        localsymtable:=nil;
        globalmacrosymtable.free;
        globalmacrosymtable:=nil;
        localmacrosymtable.free;
        localmacrosymtable:=nil;
        deflist.free;
        deflist:=TFPObjectList.Create(false);
        symlist.free;
        symlist:=TFPObjectList.Create(false);
        ptrdefs.free;
        ptrdefs:=THashSet.Create(64,true,false);
        arraydefs.free;
        arraydefs:=THashSet.Create(64,true,false);
        procaddrdefs.free;
        procaddrdefs:=THashSet.Create(64,true,false);
{$ifdef llvm}
        llvmdefs.free;
        llvmdefs:=THashSet.Create(64,true,false);
        llvmusedsyms.free;
        llvmusedsyms:=TFPObjectList.Create(true);
        llvmcompilerusedsyms.free;
        llvmcompilerusedsyms:=TFPObjectList.Create(true);
        llvminitprocs.free;
        llvminitprocs:=TFPList.Create;
        llvmfiniprocs.free;
        llvmfiniprocs:=TFPList.Create;
        llvmmetadatastrings.free;
        llvmmetadatastrings:=TFPHashList.Create;
{$endif llvm}
        wpoinfo.free;
        wpoinfo:=nil;
        checkforwarddefs.free;
        checkforwarddefs:=TFPObjectList.Create(false);
        publicasmsyms.free;
        publicasmsyms:=TFPHashObjectList.Create(true);
        externasmsyms.free;
        externasmsyms:=TFPHashObjectList.Create(true);
        unitimportsyms.free;
        unitimportsyms:=TFPObjectList.Create(false);
        derefdata.free;
        derefdata:=TDynamicArray.Create(1024);
        if assigned(unitmap) then
          begin
            freemem(unitmap);
            unitmap:=nil;
          end;
        if assigned(derefmap) then
          begin
            for i:=0 to derefmapcnt-1 do
              stringdispose(derefmap[i].modulename);
            freemem(derefmap);
            derefmap:=nil;
          end;
        unitmapsize:=0;
        derefmapsize:=0;
        derefmapcnt:=0;
        derefdataintflen:=0;
        sourcefiles.free;
        sourcefiles:=tinputfilemanager.create;
        asmdata:=casmdata.create(modulename);
        InitDebugInfo(self,current_debuginfo_reset);
        _exports.free;
        _exports:=tlinkedlist.create;
        dllscannerinputlist.free;
        dllscannerinputlist:=TFPHashList.create;
        used_units.free;
        used_units:=TLinkedList.Create;
        dependent_units.free;
        dependent_units:=TLinkedList.Create;
        resourcefiles.Free;
        resourcefiles:=TCmdStrList.Create;
        linkorderedsymbols.Free;
        linkorderedsymbols:=TCmdStrList.Create;;
        pendingspecializations.free;
        pendingspecializations:=tfphashobjectlist.create(false);
        if assigned(waitingforunit) and
          (waitingforunit.count<>0) then
          internalerror(2016070501);
        waitingforunit.free;
        waitingforunit:=tfpobjectlist.create(false);;
        linkunitofiles.Free;
        linkunitofiles:=TLinkContainer.Create;
        linkunitstaticlibs.Free;
        linkunitstaticlibs:=TLinkContainer.Create;
        linkunitsharedlibs.Free;
        linkunitsharedlibs:=TLinkContainer.Create;
        linkotherofiles.Free;
        linkotherofiles:=TLinkContainer.Create;
        linkotherstaticlibs.Free;
        linkotherstaticlibs:=TLinkContainer.Create;
        linkothersharedlibs.Free;
        linkothersharedlibs:=TLinkContainer.Create;
        linkotherframeworks.Free;
        linkotherframeworks:=TLinkContainer.Create;
        stringdispose(mainname);
        FImportLibraryList.Free;
        FImportLibraryList:=TFPHashObjectList.Create;
        do_compile:=false;
        do_reload:=false;
        interface_compiled:=false;
        in_interface:=true;
        in_global:=true;
        mode_switch_allowed:=true;
        stringdispose(deprecatedmsg);
        stringdispose(namespace);
        tcinitcode.free;
        tcinitcode:=nil;
        localunitsearchpath.Free;
        localunitsearchpath:=TSearchPathList.Create;
        localobjectsearchpath.free;
        localobjectsearchpath:=TSearchPathList.Create;
        localincludesearchpath.free;
        localincludesearchpath:=TSearchPathList.Create;
        locallibrarysearchpath.free;
        locallibrarysearchpath:=TSearchPathList.Create;
        localframeworksearchpath.free;
        localframeworksearchpath:=TSearchPathList.Create;
        moduleoptions:=[];
        is_dbginfo_written:=false;
        crc:=0;
        interface_crc:=0;
        indirect_crc:=0;
        headerflags:=0;
        longversion:=0;
        moduleflags:=[];
        mainfilepos.line:=0;
        mainfilepos.column:=0;
        mainfilepos.fileindex:=0;
        recompile_reason:=rr_unknown;
        {
          The following fields should not
          be reset:
           mainsource
           state
           loaded_from
           sources_avail
        }
      end;


    procedure tmodule.adddependency(callermodule:tmodule);
      begin
        { This is not needed for programs }
        if not callermodule.is_unit then
          exit;
        Message2(unit_u_add_depend_to,callermodule.modulename^,modulename^);
        dependent_units.concat(tdependent_unit.create(callermodule));
      end;


    procedure tmodule.flagdependent(callermodule:tmodule);
      var
        pm : tdependent_unit;
      begin
        { flag all units that depend on this unit for reloading }
        pm:=tdependent_unit(current_module.dependent_units.first);
        while assigned(pm) do
         begin
           { We do not have to reload the unit that wants to load
             this unit, unless this unit is already compiled during
             the loading }
           if (pm.u=callermodule) and
              (pm.u.state<>ms_compiled) then
             Message1(unit_u_no_reload_is_caller,pm.u.modulename^)
           else
            if pm.u.state=ms_second_compile then
              Message1(unit_u_no_reload_in_second_compile,pm.u.modulename^)
           else
            begin
              pm.u.do_reload:=true;
              Message1(unit_u_flag_for_reload,pm.u.modulename^);
            end;
           pm:=tdependent_unit(pm.next);
         end;
      end;


    procedure tmodule.addimportedsym(sym:TSymEntry);
      begin
        if unitimportsyms.IndexOf(sym)<0 then
          unitimportsyms.Add(sym);
      end;

    function tmodule.addusedunit(hp:tmodule;inuses:boolean;usym:tunitsym):tused_unit;
      var
        pu : tused_unit;
      begin
        pu:=tused_unit.create(hp,in_interface,inuses,usym);
        used_units.concat(pu);
        addusedunit:=pu;
      end;


    procedure tmodule.updatemaps;
      var
        oldmapsize : longint;
        hp  : tmodule;
        i   : longint;
      begin
        { Extend unitmap }
        oldmapsize:=unitmapsize;
        unitmapsize:=loaded_units.count;
        reallocmem(unitmap,unitmapsize*sizeof(tunitmaprec));
        fillchar(unitmap[oldmapsize],(unitmapsize-oldmapsize)*sizeof(tunitmaprec),0);

        { Extend Derefmap }
        oldmapsize:=derefmapsize;
        derefmapsize:=loaded_units.count;
        reallocmem(derefmap,derefmapsize*sizeof(tderefmaprec));
        fillchar(derefmap[oldmapsize],(derefmapsize-oldmapsize)*sizeof(tderefmaprec),0);

        { Add all units to unitmap }
        hp:=tmodule(loaded_units.first);
        i:=0;
        while assigned(hp) do
          begin
            if hp.moduleid>=unitmapsize then
              internalerror(200501151);
            { Verify old entries }
            if (i<oldmapsize) then
              begin
                if (hp.moduleid<>i) or
                   (unitmap[hp.moduleid].u<>hp) then
                  internalerror(200501156);
              end
            else
              begin
                unitmap[hp.moduleid].u:=hp;
                unitmap[hp.moduleid].derefidx:=-1;
              end;
            inc(i);
            hp:=tmodule(hp.next);
          end;
      end;



    function tmodule.derefidx_unit(id:longint):longint;
      begin
        if id>=unitmapsize then
          internalerror(2005011511);
        if unitmap[id].derefidx=-1 then
          begin
            unitmap[id].derefidx:=derefmapcnt;
            inc(derefmapcnt);
            derefmap[unitmap[id].derefidx].u:=unitmap[id].u;
          end;
        if unitmap[id].derefidx>=derefmapsize then
          internalerror(2005011514);
        result:=unitmap[id].derefidx;
      end;


    function tmodule.resolve_unit(id:longint):tmodule;
      var
        hp : tmodule;
      begin
        if id>=derefmapsize then
          internalerror(200306231);
        result:=derefmap[id].u;
        if not assigned(result) then
          begin
            if not assigned(derefmap[id].modulename) or
               (derefmap[id].modulename^='') then
              internalerror(200501159);
            hp:=tmodule(loaded_units.first);
            while assigned(hp) do
              begin
                { only check for units. The main program is also
                  as a unit in the loaded_units list. We simply need
                  to ignore this entry (PFV) }
                if hp.is_unit and
                   (hp.modulename^=derefmap[id].modulename^) then
                  break;
                hp:=tmodule(hp.next);
              end;
            if not assigned(hp) then
              internalerror(2005011510);
            derefmap[id].u:=hp;
            result:=hp;
          end;
      end;


    procedure tmodule.allunitsused;
      var
        pu : tused_unit;
      begin
        pu:=tused_unit(used_units.first);
        while assigned(pu) do
          begin
            if assigned(pu.u.globalsymtable) then
              begin
                if unitmap[pu.u.moduleid].u<>pu.u then
                  internalerror(200501157);
                { Give a note when the unit is not referenced, skip
                  this is for units with an initialization/finalization }
                if (unitmap[pu.u.moduleid].refs=0) and
                   pu.in_uses and
                   ((pu.u.moduleflags * [mf_init,mf_finalize])=[]) then
                  CGMessagePos2(pu.unitsym.fileinfo,sym_n_unit_not_used,pu.u.realmodulename^,realmodulename^);
              end;
            pu:=tused_unit(pu.next);
          end;
      end;

    procedure tmodule.end_of_parsing;
      begin
        { free asmdata }
        if assigned(asmdata) then
          begin
            asmdata.free;
            asmdata:=nil;
          end;

        { free scanner }
        if assigned(scanner) then
          begin
            if current_scanner=tscannerfile(scanner) then
              current_scanner:=nil;
            tscannerfile(scanner).free;
            scanner:=nil;
          end;

        { free symtable stack }
        if assigned(symtablestack) then
          begin
            symtablestack.free;
            symtablestack:=nil;
          end;
        if assigned(macrosymtablestack) then
          begin
            macrosymtablestack.free;
            macrosymtablestack:=nil;
          end;
        waitingforunit.free;
        waitingforunit:=nil;
        localmacrosymtable.free;
        localmacrosymtable:=nil;
        ptrdefs.free;
        ptrdefs:=nil;
        arraydefs.free;
        arraydefs:=nil;
        procaddrdefs.free;
        procaddrdefs:=nil;
{$ifdef llvm}
        llvmdefs.free;
        llvmdefs:=nil;
{$endif llvm}
        checkforwarddefs.free;
        checkforwarddefs:=nil;
        tcinitcode.free;
        tcinitcode:=nil;
        localunitsearchpath.free;
        localunitsearchpath:=nil;
        localobjectsearchpath.free;
        localobjectsearchpath:=nil;
        localincludesearchpath.free;
        localincludesearchpath:=nil;
        locallibrarysearchpath.free;
        locallibrarysearchpath:=nil;
        localframeworksearchpath.free;
        localframeworksearchpath:=nil;
      end;


    procedure tmodule.setmodulename(const s:string);
      begin
        stringdispose(modulename);
        stringdispose(realmodulename);
        modulename:=stringdup(upper(s));
        realmodulename:=stringdup(s);
        { also update asmlibrary names }
        current_asmdata.name:=modulename;
      end;


    procedure TModule.AddExternalImport(const libname,symname,symmangledname:string;
              OrdNr: longint;isvar:boolean;ImportByOrdinalOnly:boolean);
      var
        ImportLibrary,OtherIL : TImportLibrary;
        ImportSymbol  : TImportSymbol;
        i : longint;
      begin
        ImportLibrary:=TImportLibrary(ImportLibraryList.Find(libname));
        if not assigned(ImportLibrary) then
          ImportLibrary:=TImportLibrary.Create(ImportLibraryList,libname);
        ImportSymbol:=TImportSymbol(ImportLibrary.ImportSymbolList.Find(symname));
        if not assigned(ImportSymbol) then
          begin
            { Check that the same name does not exist in another library }
            { If it does and the same mangled name is used, issue a warning }
            if ImportLibraryList.Count>1 then
              for i:=0 To ImportLibraryList.Count-1 do
                begin
                  OtherIL:=TImportLibrary(ImportLibraryList.Items[i]);
                  ImportSymbol:=TImportSymbol(OtherIL.ImportSymbolList.Find(symname));
                  if assigned(ImportSymbol) then
                    begin
                      if ImportSymbol.MangledName=symmangledname then
                        begin
                          CGMessage3(sym_w_library_overload,symname,libname,OtherIL.Name);
                          break;
                        end;
                    end;
                end;
            if not ImportByOrdinalOnly then
              { negative ordinal number indicates import by name with ordinal number as hint }
              OrdNr:=-OrdNr;
            ImportSymbol:=TImportSymbol.Create(ImportLibrary.ImportSymbolList,
              symname,symmangledname,OrdNr,isvar);
          end;
      end;


    procedure tmodule.add_public_asmsym(sym:TAsmSymbol);
      begin
        add_public_asmsym(sym.name,sym.bind,sym.typ);
      end;


    procedure tmodule.add_public_asmsym(const name:TSymStr;bind:TAsmsymbind;typ:Tasmsymtype);
      var
        sym : tasmsymbol;
      begin
        { ToDo: check for AB_GLOBAL, AB_EXTERNAL? }
        sym:=tasmsymbol(publicasmsyms.find(name));
        if assigned(sym) then
          begin
            if (sym.bind<>bind) or (sym.typ<>typ) then
              internalerror(2016070101);
            exit;
          end;
        tasmsymbol.create(publicasmsyms,name,bind,typ);
      end;


    procedure tmodule.add_extern_asmsym(sym:TAsmSymbol);
      begin
        add_extern_asmsym(sym.name,sym.bind,sym.typ);
      end;


    procedure tmodule.add_extern_asmsym(const name:TSymStr;bind:TAsmsymbind;typ:Tasmsymtype);
      var
        sym : tasmsymbol;
      begin
        { ToDo: check for AB_EXTERNAL? }
        sym:=tasmsymbol(externasmsyms.find(name));
        if assigned(sym) then
          begin
            if (sym.bind<>bind) or (sym.typ<>typ) then
              internalerror(2016070102);
            exit;
          end;
        tasmsymbol.create(externasmsyms,name,bind,typ);
      end;


initialization
{$ifdef MEMDEBUG}
  memsymtable:=TMemDebug.create('Symtables');
  memsymtable.stop;
{$endif MEMDEBUG}

finalization
{$ifdef MEMDEBUG}
  memsymtable.free;
{$endif MEMDEBUG}

end.
