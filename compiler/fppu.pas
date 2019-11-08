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
unit fppu;

{$i fpcdefs.inc}

{ $define DEBUG_UNIT_CRC_CHANGES}

{ close ppufiles on system that are
  short on file handles like DOS system PM }
{$ifdef GO32V2}
  {$define SHORT_ON_FILE_HANDLES}
{$endif GO32V2}
{$ifdef WATCOM}
  {$define SHORT_ON_FILE_HANDLES}
{$endif WATCOM}

interface

    uses
      cmsgs,verbose,
      cutils,cclasses,cstreams,
      globtype,globals,finput,fmodule,
      symbase,ppu,symtype;

    type
       { tppumodule }

       tppumodule = class(tmodule)
          ppufile    : tcompilerppufile; { the PPU file }
          sourcefn   : TPathStr; { Source specified with "uses .. in '..'" }
          comments   : TCmdStrList;
          nsprefix   : TCmdStr; { Namespace prefix the unit was found with }
{$ifdef Test_Double_checksum}
          crc_array  : pointer;
          crc_size   : longint;
          crc_array2 : pointer;
          crc_size2  : longint;
{$endif def Test_Double_checksum}
          constructor create(LoadedFrom:TModule;const amodulename: string; const afilename:TPathStr;_is_unit:boolean);
          destructor destroy;override;
          procedure reset;override;
          function  openppufile:boolean;
          function  openppustream(strm:TCStream):boolean;
          procedure getppucrc;
          procedure writeppu;
          procedure loadppu;
          function  needrecompile:boolean;
          procedure setdefgeneration;
          procedure reload_flagged_units;
          procedure end_of_parsing;override;
       private
          unitimportsymsderefs : tfplist;
         { Each time a unit's defs are (re)created, its defsgeneration is
           set to the value of a global counter, and the global counter is
           increased. We only reresolve its dependent units' defs in case
           they have been resolved only for an older generation, in order to
           avoid endless resolving loops in case of cyclic dependencies. }
          defsgeneration : longint;

          function  openppu(ppufiletime:longint):boolean;
          function  search_unit_files(onlysource:boolean):boolean;
          function  search_unit(onlysource,shortname:boolean):boolean;
          function  loadfrompackage:boolean;
          procedure load_interface;
          procedure load_implementation;
          procedure load_usedunits;
          procedure printcomments;
          procedure queuecomment(const s:TMsgStr;v,w:longint);
          procedure buildderefunitimportsyms;
          procedure derefunitimportsyms;
          procedure freederefunitimportsyms;
          procedure writesourcefiles;
          procedure writeusedunit(intf:boolean);
          procedure writelinkcontainer(var p:tlinkcontainer;id:byte;strippath:boolean);
          procedure writederefmap;
          procedure writederefdata;
          procedure writeImportSymbols;
          procedure writeResources;
          procedure writeOrderedSymbols;
          procedure writeunitimportsyms;
          procedure writeasmsyms(kind:tunitasmlisttype;list:tfphashobjectlist);
          procedure writeextraheader;
          procedure readsourcefiles;
          procedure readloadunit;
          procedure readlinkcontainer(var p:tlinkcontainer);
          procedure readderefmap;
          procedure readderefdata;
          procedure readImportSymbols;
          procedure readResources;
          procedure readOrderedSymbols;
          procedure readwpofile;
          procedure readunitimportsyms;
          procedure readasmsyms;
          procedure readextraheader;
{$IFDEF MACRO_DIFF_HINT}
          procedure writeusedmacro(p:TNamedIndexItem;arg:pointer);
          procedure writeusedmacros;
          procedure readusedmacros;
{$ENDIF}
       end;

    function registerunit(callermodule:tmodule;const s : TIDString;const fn:string) : tppumodule;


implementation

uses
  SysUtils,
  cfileutl,
  systems,version,options,
  symtable, symsym,
  wpoinfo,
  scanner,
  aasmbase,ogbase,
  parser,
  comphook,
  entfile,fpkg,fpcp;


var
  currentdefgeneration: longint;

{****************************************************************************
                                TPPUMODULE
 ****************************************************************************}

    constructor tppumodule.create(LoadedFrom:TModule;const amodulename: string; const afilename:TPathStr;_is_unit:boolean);
      begin
        inherited create(LoadedFrom,amodulename,afilename,_is_unit);
        ppufile:=nil;
        sourcefn:=afilename;
        unitimportsymsderefs:=tfplist.create;
      end;


    destructor tppumodule.Destroy;
      begin
        if assigned(ppufile) then
         ppufile.free;
        ppufile:=nil;
        comments.free;
        comments:=nil;
        { all derefs allocated with new
          are dispose'd inside this method }
        freederefunitimportsyms;
        unitimportsymsderefs.free;
        unitimportsymsderefs:=nil;
        inherited Destroy;
      end;


    procedure tppumodule.reset;
      begin
        inc(currentdefgeneration);
        if assigned(ppufile) then
         begin
           ppufile.free;
           ppufile:=nil;
         end;
        freederefunitimportsyms;
        unitimportsymsderefs.free;
        unitimportsymsderefs:=tfplist.create;
        inherited reset;
      end;

    procedure tppumodule.queuecomment(const s:TMsgStr;v,w:longint);
    begin
      if comments = nil then
        comments := TCmdStrList.create;
      comments.insert(s);
    end;

    procedure tppumodule.printcomments;
    var
      comment: string;
    begin
      if comments = nil then
        exit;
      { comments are inserted in reverse order }
      repeat
        comment := comments.getlast;
        if length(comment) = 0 then
          exit;
        do_comment(v_normal, comment);
      until false;
    end;

    function tppumodule.openppufile:boolean;
      var
        ppufiletime : longint;
      begin
        openppufile:=false;
        Message1(unit_t_ppu_loading,ppufilename,@queuecomment);
      { Get ppufile time (also check if the file exists) }
        ppufiletime:=getnamedfiletime(ppufilename);
        if ppufiletime=-1 then
         exit;
      { Open the ppufile }
        Message1(unit_u_ppu_name,ppufilename);
        ppufile:=tcompilerppufile.create(ppufilename);
        if not ppufile.openfile then
         begin
           ppufile.free;
           ppufile:=nil;
           Message(unit_u_ppu_file_too_short);
           exit;
         end;
        result:=openppu(ppufiletime);
      end;


    function tppumodule.openppustream(strm:TCStream):boolean;
      begin
        result:=false;
      { Open the ppufile }
        Message1(unit_u_ppu_name,ppufilename);
        ppufile:=tcompilerppufile.create(ppufilename);
        if not ppufile.openstream(strm) then
         begin
           ppufile.free;
           ppufile:=nil;
           Message(unit_u_ppu_file_too_short);
           exit;
         end;
        result:=openppu(-1);
      end;


    function tppumodule.openppu(ppufiletime:longint):boolean;

      function checkheader: boolean;
        begin
          result:=false;
          { check for a valid PPU file }
            if not ppufile.CheckPPUId then
             begin
               Message(unit_u_ppu_invalid_header);
               exit;
             end;
          { check for allowed PPU versions }
            if not (ppufile.getversion = CurrentPPUVersion) then
             begin
               Message1(unit_u_ppu_invalid_version,tostr(ppufile.getversion),@queuecomment);
               exit;
             end;
          { check the target processor }
            if tsystemcpu(ppufile.header.common.cpu)<>target_cpu then
             begin
               Message(unit_u_ppu_invalid_processor,@queuecomment);
               exit;
             end;
          { check target }
            if tsystem(ppufile.header.common.target)<>target_info.system then
             begin
               Message(unit_u_ppu_invalid_target,@queuecomment);
               exit;
             end;
{$ifdef cpufpemu}
          { check if floating point emulation is on?
            fpu emulation isn't unit levelwise because it affects calling convention }
          if ((ppufile.header.common.flags and uf_fpu_emulation)<>0) <>
             (cs_fp_emulation in current_settings.moduleswitches) then
            begin
              Message(unit_u_ppu_invalid_fpumode,@queuecomment);
              exit;
            end;
{$endif cpufpemu}
           result:=true;
        end;

      function checkextraheader: boolean;
        begin
          result:=false;
          if ppufile.readentry<>ibextraheader then
            begin
              Message(unit_u_ppu_invalid_header);
              exit;
            end;
          readextraheader;
          if (longversion<>CurrentPPULongVersion) or
             not ppufile.EndOfEntry then
            begin
              Message(unit_u_ppu_invalid_header);
              exit;
            end;
{$ifdef i8086}
          { check i8086 memory model flags }
          if (mf_i8086_far_code in moduleflags) <>
             (current_settings.x86memorymodel in [mm_medium,mm_large,mm_huge]) then
            begin
              Message(unit_u_ppu_invalid_memory_model,@queuecomment);
              exit;
            end;
          if (mf_i8086_far_data in moduleflags) <>
             (current_settings.x86memorymodel in [mm_compact,mm_large]) then
            begin
              Message(unit_u_ppu_invalid_memory_model,@queuecomment);
              exit;
            end;
          if (mf_i8086_huge_data in moduleflags) <>
             (current_settings.x86memorymodel=mm_huge) then
            begin
              Message(unit_u_ppu_invalid_memory_model,@queuecomment);
              exit;
            end;
          if (mf_i8086_cs_equals_ds in moduleflags) <>
             (current_settings.x86memorymodel=mm_tiny) then
            begin
              Message(unit_u_ppu_invalid_memory_model,@queuecomment);
              exit;
            end;
          if (mf_i8086_ss_equals_ds in moduleflags) <>
             (current_settings.x86memorymodel in [mm_tiny,mm_small,mm_medium]) then
            begin
              Message(unit_u_ppu_invalid_memory_model,@queuecomment);
              exit;
            end;
{$endif i8086}
          if {$ifdef llvm}not{$endif}(mf_llvm in moduleflags) then
            begin
              Message(unit_u_ppu_llvm_mismatch,@queuecomment);
              exit;
            end;
          result:=true;
        end;

      begin
        openppu:=false;
        if not checkheader or
           not checkextraheader then
          begin
            ppufile.free;
            ppufile:=nil;
            exit;
          end;

      { Load values to be access easier }
        headerflags:=ppufile.header.common.flags;
        crc:=ppufile.header.checksum;
        interface_crc:=ppufile.header.interface_checksum;
        indirect_crc:=ppufile.header.indirect_checksum;
      { Show Debug info }
        if ppufiletime<>-1 then
          Message1(unit_u_ppu_time,filetimestring(ppufiletime))
        else
          Message1(unit_u_ppu_time,'unknown');
        Message1(unit_u_ppu_flags,tostr(headerflags));
        Message1(unit_u_ppu_crc,hexstr(ppufile.header.checksum,8));
        Message1(unit_u_ppu_crc,hexstr(ppufile.header.interface_checksum,8)+' (intfc)');
        Message1(unit_u_ppu_crc,hexstr(ppufile.header.indirect_checksum,8)+' (indc)');
        Comment(V_used,'Number of definitions: '+tostr(ppufile.header.deflistsize));
        Comment(V_used,'Number of symbols: '+tostr(ppufile.header.symlistsize));
        do_compile:=false;
        openppu:=true;
      end;


    function tppumodule.search_unit_files(onlysource:boolean):boolean;
      var
        found : boolean;
      begin
        found:=false;
        if search_unit(onlysource,false) then
          found:=true;
        if (not found) and
           (length(modulename^)>8) and
           search_unit(onlysource,true) then
          found:=true;
        search_unit_files:=found;
      end;


    function tppumodule.search_unit(onlysource,shortname:boolean):boolean;
      var
         singlepathstring,
         filename : TCmdStr;

         Function UnitExists(const ext:string;var foundfile:TCmdStr;const prefix:TCmdStr):boolean;
         var
           s : tcmdstr;
         begin
           if CheckVerbosity(V_Tried) then
             Message1(unit_t_unitsearch,Singlepathstring+filename+ext);
           s:=FileName+ext;
           if prefix<>'' then
             s:=prefix+'.'+s;
           UnitExists:=FindFile(s,Singlepathstring,true,foundfile);
         end;

         Function PPUSearchPath(const s,prefix:TCmdStr):boolean;
         var
           found : boolean;
           hs,
           newname : TCmdStr;
         begin
           Found:=false;
           singlepathstring:=FixPath(s,false);
         { Check for PPU file }
           Found:=UnitExists(target_info.unitext,hs,prefix);
           if Found then
            Begin
              SetFileName(hs,false);
              if prefix<>'' then
                begin
                  newname:=prefix+'.'+realmodulename^;
                  stringdispose(realmodulename);
                  realmodulename:=stringdup(newname);
                  stringdispose(modulename);
                  modulename:=stringdup(upper(newname));
                end;
              Found:=openppufile;
            End;
           PPUSearchPath:=Found;
         end;

         Function SourceSearchPath(const s,prefix:TCmdStr):boolean;
         var
           found   : boolean;
           hs,
           newname : TCmdStr;
         begin
           Found:=false;
           singlepathstring:=FixPath(s,false);
         { Check for Sources }
           ppufile:=nil;
           do_compile:=true;
           recompile_reason:=rr_noppu;
         {Check for .pp file}
           Found:=UnitExists(sourceext,hs,prefix);
           if not Found then
            begin
              { Check for .pas }
              Found:=UnitExists(pasext,hs,prefix);
            end;
           if not Found and
              ((m_mac in current_settings.modeswitches) or
               (tf_p_ext_support in target_info.flags)) then
            begin
              { Check for .p, if mode is macpas}
              Found:=UnitExists(pext,hs,prefix);
            end;
           mainsource:='';
           if Found then
            begin
              sources_avail:=true;
              { Load Filenames when found }
              mainsource:=hs;
              SetFileName(hs,false);
              if prefix<>'' then
                begin
                  newname:=prefix+'.'+realmodulename^;
                  stringdispose(realmodulename);
                  realmodulename:=stringdup(newname);
                  stringdispose(modulename);
                  modulename:=stringdup(upper(newname));
                end;
            end
           else
            sources_avail:=false;
           SourceSearchPath:=Found;
         end;

         Function SearchPath(const s,prefix:TCmdStr):boolean;
         var
           found : boolean;
         begin
           { First check for a ppu, then for the source }
           found:=false;
           if not onlysource then
            found:=PPUSearchPath(s,prefix);
           if not found then
            found:=SourceSearchPath(s,prefix);
           SearchPath:=found;
         end;

         Function SearchPathList(list:TSearchPathList;const prefix:TCmdStr):boolean;
         var
           hp : TCmdStrListItem;
           found : boolean;
         begin
           found:=false;
           hp:=TCmdStrListItem(list.First);
           while assigned(hp) do
            begin
              found:=SearchPath(hp.Str,prefix);
              if found then
               break;
              hp:=TCmdStrListItem(hp.next);
            end;
           SearchPathList:=found;
         end;

         function SearchPPUPaths(const prefix:TCmdStr):boolean;
         begin
           result:=PPUSearchPath('.',prefix);
           if (not result) and (outputpath<>'') then
            result:=PPUSearchPath(outputpath,prefix);
           if (not result) and Assigned(main_module) and (main_module.Path<>'')  then
            result:=PPUSearchPath(main_module.Path,prefix);
         end;

         function SearchSourcePaths(const prefix:TCmdStr):boolean;
         begin
           result:=SourceSearchPath('.',prefix);
           if (not result) and Assigned(main_module) and (main_module.Path<>'') then
             result:=SourceSearchPath(main_module.Path,prefix);
           if (not result) and Assigned(loaded_from) then
             result:=SearchPathList(loaded_from.LocalUnitSearchPath,prefix);
           if not result then
             result:=SearchPathList(UnitSearchPath,prefix);
         end;

       var
         fnd : boolean;
         hs  : TPathStr;
         nsitem : TCmdStrListItem;
       begin
         if shortname then
          filename:=FixFileName(Copy(realmodulename^,1,8))
         else
          filename:=FixFileName(realmodulename^);
         { try to find unit
            1. look for ppu in cwd
            2. look for ppu in outputpath if set, this is tp7 compatible (PFV)
            3. look for ppu in maindir
            4. look for the specified source file (from the uses line)
            5. look for source in cwd
            6. look for source in maindir
            7. local unit pathlist
            8. global unit pathlist
            9. for each default namespace:
                  repeat 1 - 3 and 5 - 8 with namespace as prefix }
         fnd:=false;
         if not onlysource then
            fnd:=SearchPPUPaths('');
         if (not fnd) and (sourcefn<>'') then
          begin
            { the full filename is specified so we can't use here the
              searchpath (PFV) }
            if CheckVerbosity(V_Tried) then
              Message1(unit_t_unitsearch,ChangeFileExt(sourcefn,sourceext));
            fnd:=FindFile(ChangeFileExt(sourcefn,sourceext),'',true,hs);
            if not fnd then
             begin
               if CheckVerbosity(V_Tried) then
                 Message1(unit_t_unitsearch,ChangeFileExt(sourcefn,pasext));
               fnd:=FindFile(ChangeFileExt(sourcefn,pasext),'',true,hs);
             end;
            if not fnd and
               ((m_mac in current_settings.modeswitches) or
                (tf_p_ext_support in target_info.flags)) then
             begin
               if CheckVerbosity(V_Tried) then
                 Message1(unit_t_unitsearch,ChangeFileExt(sourcefn,pext));
               fnd:=FindFile(ChangeFileExt(sourcefn,pext),'',true,hs);
             end;
            if fnd then
             begin
               sources_avail:=true;
               do_compile:=true;
               recompile_reason:=rr_noppu;
               mainsource:=hs;
               SetFileName(hs,false);
             end;
          end;
         if not fnd then
           begin
             fnd:=SearchSourcePaths('');
             if not fnd and (namespacelist.count>0) then
               begin
                 nsitem:=TCmdStrListItem(namespacelist.first);
                 while assigned(nsitem) do
                   begin
                     if not onlysource then
                       begin
                         fnd:=SearchPPUPaths(nsitem.str);
                         if fnd then
                           break;
                       end;
                     fnd:=SearchSourcePaths(nsitem.str);
                     if fnd then
                       break;

                     nsitem:=TCmdStrListItem(nsitem.next);
                   end;
                 if assigned(nsitem) then
                   nsprefix:=nsitem.str;
               end;
           end;
         search_unit:=fnd;
      end;

    function tppumodule.loadfrompackage: boolean;
      (*var
        singlepathstring,
        filename : TCmdStr;

        Function UnitExists(const ext:string;var foundfile:TCmdStr):boolean;
          begin
            if CheckVerbosity(V_Tried) then
              Message1(unit_t_unitsearch,Singlepathstring+filename);
            UnitExists:=FindFile(FileName,Singlepathstring,true,foundfile);
          end;

        Function PPUSearchPath(const s:TCmdStr):boolean;
          var
            found : boolean;
            hs    : TCmdStr;
          begin
            Found:=false;
            singlepathstring:=FixPath(s,false);
          { Check for PPU file }
            Found:=UnitExists(target_info.unitext,hs);
            if Found then
             Begin
               SetFileName(hs,false);
               //Found:=OpenPPU;
             End;
            PPUSearchPath:=Found;
          end;

        Function SearchPathList(list:TSearchPathList):boolean;
          var
            hp : TCmdStrListItem;
            found : boolean;
          begin
            found:=false;
            hp:=TCmdStrListItem(list.First);
            while assigned(hp) do
             begin
               found:=PPUSearchPath(hp.Str);
               if found then
                break;
               hp:=TCmdStrListItem(hp.next);
             end;
            SearchPathList:=found;
          end;*)

      var
        pkg : ppackageentry;
        pkgunit : pcontainedunit;
        i,idx : longint;
        strm : TCStream;
      begin
        result:=false;
        for i:=0 to packagelist.count-1 do
          begin
            pkg:=ppackageentry(packagelist[i]);
            if not assigned(pkg^.package) then
              internalerror(2013053103);
            idx:=pkg^.package.containedmodules.FindIndexOf(modulename^);
            if idx>=0 then
              begin
                { the unit is part of this package }
                pkgunit:=pcontainedunit(pkg^.package.containedmodules[idx]);
                if not assigned(pkgunit^.module) then
                  pkgunit^.module:=self;
                { ToDo: check whether we really don't need this anymore }
                {filename:=pkgunit^.ppufile;
                if not SearchPathList(unitsearchpath) then
                  exit};
                strm:=tpcppackage(pkg^.package).getmodulestream(self);
                if not assigned(strm) then
                  internalerror(2015103002);
                if not openppustream(strm) then
                  exit;
                package:=pkg^.package;
                Message2(unit_u_loading_from_package,modulename^,pkg^.package.packagename^);

                { now load the unit and all used units }
                load_interface;
                setdefgeneration;
                load_usedunits;
                Message1(unit_u_finished_loading_unit,modulename^);

                result:=true;
                break;
              end;
          end;
      end;


    procedure tppumodule.buildderefunitimportsyms;
      var
        i : longint;
        deref : pderef;
      begin
        for i:=0 to unitimportsyms.count-1 do
          begin
            new(deref);
            deref^.build(unitimportsyms[i]);
            unitimportsymsderefs.add(deref);
          end;
      end;


    procedure tppumodule.derefunitimportsyms;
      var
        i : longint;
        sym : tsym;
      begin
        for i:=0 to unitimportsymsderefs.count-1 do
          begin
            sym:=tsym(pderef(unitimportsymsderefs[i])^.resolve);
            unitimportsyms.add(sym);
          end;
      end;

    procedure tppumodule.freederefunitimportsyms;
      var
        i : longint;
        deref : pderef;
      begin
        for i:=0 to unitimportsymsderefs.count-1 do
          begin
            deref:=pderef(unitimportsymsderefs[i]);
            system.dispose(deref);
          end;
      end;

{**********************************
    PPU Reading/Writing Helpers
***********************************}

{$IFDEF MACRO_DIFF_HINT}
    var
      is_initial: Boolean;

    procedure tppumodule.writeusedmacro(p:TNamedIndexItem;arg:pointer);
      begin
        if tmacro(p).is_used or is_initial then
          begin
            ppufile.putstring(p.name);
            ppufile.putboolean(is_initial);
            ppufile.putboolean(tmacro(p).is_used);
          end;
      end;

    procedure tppumodule.writeusedmacros;
      begin
        ppufile.do_crc:=false;
        is_initial:= true;
        initialmacrosymtable.foreach(@writeusedmacro,nil);
        is_initial:= false;
        if assigned(globalmacrosymtable) then
          globalmacrosymtable.foreach(@writeusedmacro,nil);
        localmacrosymtable.foreach(@writeusedmacro,nil);
        ppufile.writeentry(ibusedmacros);
        ppufile.do_crc:=true;
      end;
{$ENDIF}

    procedure tppumodule.writesourcefiles;
      var
        hp  : tinputfile;
        i,j : longint;
      begin
      { second write the used source files }
        ppufile.do_crc:=false;
        hp:=sourcefiles.files;
      { write source files directly in good order }
        j:=0;
        while assigned(hp) do
          begin
            inc(j);
            hp:=hp.ref_next;
          end;
        while j>0 do
          begin
            hp:=sourcefiles.files;
            for i:=1 to j-1 do
              hp:=hp.ref_next;
            ppufile.putstring(hp.inc_path+hp.name);
            ppufile.putlongint(hp.getfiletime);
            dec(j);
         end;
        ppufile.writeentry(ibsourcefiles);
        ppufile.do_crc:=true;
      end;


    procedure tppumodule.writeusedunit(intf:boolean);
      var
        hp : tused_unit;
        oldcrc : boolean;
      begin
        { write a reference for each used unit }
        hp:=tused_unit(used_units.first);
        while assigned(hp) do
         begin
           if hp.in_interface=intf then
             begin
               ppufile.putstring(hp.u.realmodulename^);
               { the checksum should not affect the crc of this unit ! (PFV) }
               oldcrc:=ppufile.do_crc;
               ppufile.do_crc:=false;
               ppufile.putlongint(longint(hp.checksum));
               ppufile.putlongint(longint(hp.interface_checksum));
               ppufile.putlongint(longint(hp.indirect_checksum));
               ppufile.do_crc:=oldcrc;
               { combine all indirect checksums from units used by this unit }
               if intf then
                 ppufile.indirect_crc:=ppufile.indirect_crc xor hp.indirect_checksum;
             end;
           hp:=tused_unit(hp.next);
         end;
        ppufile.do_interface_crc:=true;
        ppufile.writeentry(ibloadunit);
      end;


    procedure tppumodule.writelinkcontainer(var p:tlinkcontainer;id:byte;strippath:boolean);
      var
        hcontainer : tlinkcontainer;
        s : TPathStr;
        mask : cardinal;
      begin
        hcontainer:=TLinkContainer.Create;
        while not p.empty do
         begin
           s:=p.get(mask);
           if strippath then
            ppufile.putstring(ExtractFileName(s))
           else
            ppufile.putstring(s);
           ppufile.putlongint(mask);
           hcontainer.add(s,mask);
         end;
        ppufile.writeentry(id);
        p.Free;
        p:=hcontainer;
      end;


    procedure tppumodule.writederefmap;
      var
        i : longint;
        oldcrc : boolean;
      begin
        { This does not influence crc }
        oldcrc:=ppufile.do_crc;
        ppufile.do_crc:=false;
        { The unit map used for resolving }
        ppufile.putlongint(derefmapcnt);
        for i:=0 to derefmapcnt-1 do
          begin
            if not assigned(derefmap[i].u) then
              internalerror(2005011512);
            ppufile.putstring(derefmap[i].u.modulename^)
          end;
        ppufile.writeentry(ibderefmap);
        ppufile.do_crc:=oldcrc;
      end;


    procedure tppumodule.writederefdata;
      var
        oldcrc : boolean;
        len,hlen : longint;
        buf : array[0..1023] of byte;
      begin
        if longword(derefdataintflen)>derefdata.size then
          internalerror(200310223);
        derefdata.seek(0);
        { Write interface data }
        len:=derefdataintflen;
        while (len>0) do
          begin
            if len>1024 then
              hlen:=1024
            else
              hlen:=len;
            derefdata.read(buf,hlen);
            ppufile.putdata(buf,hlen);
            dec(len,hlen);
          end;
        { Write implementation data, this does not influence crc }
        oldcrc:=ppufile.do_crc;
        ppufile.do_crc:=false;
        len:=derefdata.size-derefdataintflen;
        while (len>0) do
          begin
            if len>1024 then
              hlen:=1024
            else
              hlen:=len;
            derefdata.read(buf,hlen);
            ppufile.putdata(buf,hlen);
            dec(len,hlen);
          end;
        if derefdata.pos<>derefdata.size then
          internalerror(200310224);
        ppufile.do_crc:=oldcrc;
        ppufile.writeentry(ibderefdata);
      end;


    procedure tppumodule.writeImportSymbols;
      var
        i,j : longint;
        ImportLibrary : TImportLibrary;
        ImportSymbol  : TImportSymbol;
      begin
        for i:=0 to ImportLibraryList.Count-1 do
          begin
            ImportLibrary:=TImportLibrary(ImportLibraryList[i]);
            ppufile.putstring(ImportLibrary.Name);
            ppufile.putlongint(ImportLibrary.ImportSymbolList.Count);
            for j:=0 to ImportLibrary.ImportSymbolList.Count-1 do
              begin
                ImportSymbol:=TImportSymbol(ImportLibrary.ImportSymbolList[j]);
                ppufile.putstring(ImportSymbol.Name);
                ppufile.putstring(ImportSymbol.MangledName);
                ppufile.putlongint(ImportSymbol.OrdNr);
                ppufile.putbyte(byte(ImportSymbol.IsVar));
              end;
          end;
        ppufile.writeentry(ibImportSymbols);
      end;


    procedure tppumodule.writeResources;
      var
        res : TCmdStrListItem;
      begin
        res:=TCmdStrListItem(ResourceFiles.First);
        while res<>nil do
          begin
            ppufile.putstring(res.FPStr);
            res:=TCmdStrListItem(res.Next);
          end;
        ppufile.writeentry(ibresources);
      end;


    procedure tppumodule.writeOrderedSymbols;
      var
        res : TCmdStrListItem;
      begin
        res:=TCmdStrListItem(linkorderedsymbols.First);
        while res<>nil do
          begin
            ppufile.putstring(res.FPStr);
            res:=TCmdStrListItem(res.Next);
          end;
        ppufile.writeentry(iborderedsymbols);
      end;


    procedure tppumodule.writeunitimportsyms;
      var
        i : longint;
      begin
        ppufile.putlongint(unitimportsymsderefs.count);
        for i:=0 to unitimportsymsderefs.count-1 do
          ppufile.putderef(pderef(unitimportsymsderefs[i])^);
        ppufile.writeentry(ibunitimportsyms);
      end;


    procedure tppumodule.writeasmsyms(kind:tunitasmlisttype;list:tfphashobjectlist);
      var
        i : longint;
        sym : TAsmSymbol;
      begin
        ppufile.putbyte(ord(kind));
        ppufile.putlongint(list.count);
        for i:=0 to list.count-1 do
          begin
            sym:=TAsmSymbol(list[i]);
            ppufile.putstring(sym.Name);
            ppufile.putbyte(ord(sym.bind));
            ppufile.putbyte(ord(sym.typ));
          end;
        ppufile.writeentry(ibasmsymbols);
      end;

    procedure tppumodule.writeextraheader;
      var
        old_docrc: boolean;
      begin
        { create unit flags }
        if do_release then
          include(moduleflags,mf_release);
        if assigned(localsymtable) then
          include(moduleflags,mf_local_symtable);
        if cs_checkpointer_called in current_settings.moduleswitches then
          include(moduleflags,mf_checkpointer_called);
{$ifdef i8086}
        if current_settings.x86memorymodel in [mm_medium,mm_large,mm_huge] then
          include(moduleflags,mf_i8086_far_code);
        if current_settings.x86memorymodel in [mm_compact,mm_large] then
          include(moduleflags,mf_i8086_far_data);
        if current_settings.x86memorymodel=mm_huge then
          include(moduleflags,mf_i8086_huge_data);
        if current_settings.x86memorymodel=mm_tiny then
          include(moduleflags,mf_i8086_cs_equals_ds);
        if current_settings.x86memorymodel in [mm_tiny,mm_small,mm_medium] then
          include(moduleflags,mf_i8086_ss_equals_ds);
{$endif i8086}
{$ifdef llvm}
        include(moduleflags,mf_llvm);
{$endif}
{$ifdef symansistr}
        include(moduleflags,mf_symansistr);
{$endif}

        old_docrc:=ppufile.do_crc;
        ppufile.do_crc:=false;
        ppufile.putlongint(longint(CurrentPPULongVersion));
        ppufile.putset(tppuset4(moduleflags));
        ppufile.writeentry(ibextraheader);
        ppufile.do_crc:=old_docrc;
      end;


{$IFDEF MACRO_DIFF_HINT}

{
  Define MACRO_DIFF_HINT for the whole compiler (and ppudump)
  to turn this facility on. Also the hint messages defined
  below must be commented in in the msg/errore.msg file.

  There is some problems with this, thats why it is shut off:

  At the first compilation, consider a macro which is not initially
  defined, but it is used (e g the check that it is undefined is true).
  Since it do not exist, there is no macro object where the is_used
  flag can be set. Later on when the macro is defined, and the ppu
  is opened, the check cannot detect this.

  Also, in which macro object should this flag be set ? It cant be set
  for macros in the initialmacrosymboltable since this table is shared
  between different files.
}

    procedure tppumodule.readusedmacros;
      var
        hs : string;
        mac : tmacro;
        was_initial,
        was_used : boolean;
      {Reads macros which was defined or used when the module was compiled.
       This is done when a ppu file is open, before it possibly is parsed.}
      begin
        while not ppufile.endofentry do
         begin
           hs:=ppufile.getstring;
           was_initial:=ppufile.getboolean;
           was_used:=ppufile.getboolean;
           mac:=tmacro(initialmacrosymtable.Find(hs));
           if assigned(mac) then
             begin
{$ifndef EXTDEBUG}
           { if we don't have the sources why tell }
              if sources_avail then
{$endif ndef EXTDEBUG}
               if (not was_initial) and was_used then
                Message2(unit_h_cond_not_set_in_last_compile,hs,mainsource^);
             end
           else { not assigned }
             if was_initial and
                was_used then
              Message2(unit_h_cond_set_in_last_compile,hs,mainsource^);
         end;
      end;
{$ENDIF}

    procedure tppumodule.readsourcefiles;
      var
        temp,hs       : string;
        inc_path      : string;
        temp_dir      : TCmdStr;
        main_dir      : TCmdStr;
        found,
        is_main       : boolean;
        orgfiletime,
        source_time   : longint;
        hp            : tinputfile;
      begin
        sources_avail:=not(mf_release in moduleflags);
        is_main:=true;
        main_dir:='';
        while not ppufile.endofentry do
         begin
           hs:=SetDirSeparators(ppufile.getstring);
           inc_path:=ExtractFilePath(hs);
           orgfiletime:=ppufile.getlongint;
           temp_dir:='';
           if sources_avail then
             begin
               if (headerflags and uf_in_library)<>0 then
                begin
                  sources_avail:=false;
                  temp:=' library';
                end
               else if pos('Macro ',hs)=1 then
                begin
                  { we don't want to find this file }
                  { but there is a problem with file indexing !! }
                  temp:='';
                end
               else
                begin
                  { check the date of the source files:
                     1 path of ppu
                     2 path of main source
                     3 current dir
                     4 include/unit path }
                  Source_Time:=GetNamedFileTime(path+hs);
                  found:=false;
                  if Source_Time<>-1 then
                    hs:=path+hs
                  else
                   if not(is_main) then
                    begin
                      Source_Time:=GetNamedFileTime(main_dir+hs);
                      if Source_Time<>-1 then
                        hs:=main_dir+hs;
                    end;
                  if Source_Time=-1 then
                    Source_Time:=GetNamedFileTime(hs);
                  if (Source_Time=-1) then
                    begin
                      if is_main then
                        found:=unitsearchpath.FindFile(hs,true,temp_dir)
                      else
                        found:=includesearchpath.FindFile(hs,true,temp_dir);
                      if found then
                       begin
                         Source_Time:=GetNamedFileTime(temp_dir);
                         if Source_Time<>-1 then
                          hs:=temp_dir;
                       end;
                    end;
                  if Source_Time<>-1 then
                    begin
                      if is_main then
                        main_dir:=ExtractFilePath(hs);
                      temp:=' time '+filetimestring(source_time);
                      if (orgfiletime<>-1) and
                         (source_time<>orgfiletime) then
                        begin
                          do_compile:=true;
                          recompile_reason:=rr_sourcenewer;
                          Message2(unit_u_source_modified,hs,ppufilename,@queuecomment);
                          temp:=temp+' *';
                        end;
                    end
                  else
                    begin
                      sources_avail:=false;
                      temp:=' not found';
                    end;
                  hp:=tdosinputfile.create(hs);
                  hp.inc_path:=inc_path;
                  { the indexing is wrong here PM }
                  sourcefiles.register_file(hp);
                end;
             end
           else
             begin
               { still register the source module for proper error messages
                 since source_avail for the module is still false, this should not hurt }
               sourcefiles.register_file(tdosinputfile.create(hs));

               temp:=' not available';
             end;
           if is_main then
             begin
               mainsource:=hs;
             end;
           Message1(unit_u_ppu_source,hs+temp,@queuecomment);
           is_main:=false;
         end;
      { check if we want to rebuild every unit, only if the sources are
        available }
        if do_build and sources_avail then
          begin
             do_compile:=true;
             recompile_reason:=rr_build;
          end;
      end;


    procedure tppumodule.readloadunit;
      var
        hs : string;
        pu : tused_unit;
        hp : tppumodule;
        indchecksum,
        intfchecksum,
        checksum : cardinal;
      begin
        while not ppufile.endofentry do
         begin
           hs:=ppufile.getstring;
           checksum:=cardinal(ppufile.getlongint);
           intfchecksum:=cardinal(ppufile.getlongint);
           indchecksum:=cardinal(ppufile.getlongint);
           { set the state of this unit before registering, this is
             needed for a correct circular dependency check }
           hp:=registerunit(self,hs,'');
           pu:=addusedunit(hp,false,nil);
           pu.checksum:=checksum;
           pu.interface_checksum:=intfchecksum;
           pu.indirect_checksum:=indchecksum;
         end;
        in_interface:=false;
      end;


    procedure tppumodule.readlinkcontainer(var p:tlinkcontainer);
      var
        s : string;
        m : longint;
      begin
        while not ppufile.endofentry do
         begin
           s:=ppufile.getstring;
           m:=ppufile.getlongint;
           p.add(s,m);
         end;
      end;


    procedure tppumodule.readderefmap;
      var
        i : longint;
      begin
        { Load unit map used for resolving }
        derefmapsize:=ppufile.getlongint;
        derefmapcnt:=derefmapsize;
        getmem(derefmap,derefmapsize*sizeof(tderefmaprec));
        fillchar(derefmap^,derefmapsize*sizeof(tderefmaprec),0);
        for i:=0 to derefmapsize-1 do
          derefmap[i].modulename:=ppufile.getpshortstring;
      end;


    procedure tppumodule.readderefdata;
      var
        len,hlen : longint;
        buf : array[0..1023] of byte;
      begin
        len:=ppufile.entrysize;
        while (len>0) do
          begin
            if len>1024 then
              hlen:=1024
            else
              hlen:=len;
            ppufile.getdata(buf,hlen);
            derefdata.write(buf,hlen);
            dec(len,hlen);
          end;
      end;


    procedure tppumodule.readImportSymbols;
      var
        j,
        extsymcnt   : longint;
        ImportLibrary  : TImportLibrary;
        extsymname  : string;
        extsymmangledname : string;
        extsymordnr : longint;
        extsymisvar : boolean;
      begin
        while not ppufile.endofentry do
          begin
            ImportLibrary:=TImportLibrary.Create(ImportLibraryList,ppufile.getstring);
            extsymcnt:=ppufile.getlongint;
            for j:=0 to extsymcnt-1 do
              begin
                extsymname:=ppufile.getstring;
                extsymmangledname:=ppufile.getstring;
                extsymordnr:=ppufile.getlongint;
                extsymisvar:=(ppufile.getbyte<>0);
                TImportSymbol.Create(ImportLibrary.ImportSymbolList,extsymname,
                  extsymmangledname,extsymordnr,extsymisvar);
              end;
          end;
      end;


    procedure tppumodule.readResources;
      begin
        while not ppufile.endofentry do
          resourcefiles.Insert(ppufile.getstring);
      end;


    procedure tppumodule.readOrderedSymbols;
      begin
        while not ppufile.endofentry do
          linkorderedsymbols.Concat(ppufile.getstring);
      end;


    procedure tppumodule.readwpofile;
      var
        orgwpofilename: string;
        orgwpofiletime: longint;
      begin
        { check whether we are using the same wpo feedback input file as when
          this unit was compiled (same file name and file date)
        }
        orgwpofilename:=ppufile.getstring;
        orgwpofiletime:=ppufile.getlongint;
        if (extractfilename(orgwpofilename)<>extractfilename(wpofeedbackinput)) or
           (orgwpofiletime<>GetNamedFileTime(orgwpofilename)) then
          { make sure we don't throw away a precompiled unit if the user simply
            forgot to specify the right wpo feedback file
          }
          message3(unit_e_different_wpo_file,ppufilename,orgwpofilename,filetimestring(orgwpofiletime));
      end;


    procedure tppumodule.readunitimportsyms;
      var
        c,i : longint;
        deref : pderef;
      begin
        c:=ppufile.getlongint;
        for i:=0 to c-1 do
          begin
            new(deref);
            ppufile.getderef(deref^);
            unitimportsymsderefs.add(deref);
          end;
      end;


    procedure tppumodule.readasmsyms;
      var
        c,i : longint;
        name : TSymStr;
        bind : TAsmsymbind;
        typ : TAsmsymtype;
        list : tfphashobjectlist;
      begin
        case tunitasmlisttype(ppufile.getbyte) of
          ualt_public:
            list:=publicasmsyms;
          ualt_extern:
            list:=externasmsyms;
        end;
        c:=ppufile.getlongint;
        for i:=0 to c-1 do
          begin
            name:=ppufile.getstring;
            bind:=TAsmsymbind(ppufile.getbyte);
            typ:=TAsmsymtype(ppufile.getbyte);
            TAsmSymbol.Create(list,name,bind,typ);
          end;
      end;


    procedure tppumodule.readextraheader;
      begin
        longversion:=cardinal(ppufile.getlongint);
        ppufile.getset(tppuset4(moduleflags));
      end;


    procedure tppumodule.load_interface;
      var
        b : byte;
        newmodulename : string;
      begin
       { read interface part }
         repeat
           b:=ppufile.readentry;
           case b of
             ibjvmnamespace :
               begin
                 namespace:=ppufile.getpshortstring;
               end;
             ibmodulename :
               begin
                 newmodulename:=ppufile.getstring;
                 if (cs_check_unit_name in current_settings.globalswitches) and
                    (upper(newmodulename)<>modulename^) then
                   Message2(unit_f_unit_name_error,realmodulename^,newmodulename);
                 stringdispose(modulename);
                 stringdispose(realmodulename);
                 modulename:=stringdup(upper(newmodulename));
                 realmodulename:=stringdup(newmodulename);
               end;
             ibextraheader:
               begin
                 readextraheader;
               end;
             ibfeatures :
               begin
                 ppufile.getset(tppuset4(features));
               end;
             ibmoduleoptions:
               begin
                 ppufile.getset(tppuset1(moduleoptions));
                 if mo_has_deprecated_msg in moduleoptions then
                   begin
                     stringdispose(deprecatedmsg);
                     deprecatedmsg:=ppufile.getpshortstring;
                   end;
               end;
             ibsourcefiles :
               readsourcefiles;
{$IFDEF MACRO_DIFF_HINT}
             ibusedmacros :
               readusedmacros;
{$ENDIF}
             ibloadunit :
               readloadunit;
             iblinkunitofiles :
               readlinkcontainer(LinkUnitOFiles);
             iblinkunitstaticlibs :
               readlinkcontainer(LinkUnitStaticLibs);
             iblinkunitsharedlibs :
               readlinkcontainer(LinkUnitSharedLibs);
             iblinkotherofiles :
               readlinkcontainer(LinkotherOFiles);
             iblinkotherstaticlibs :
               readlinkcontainer(LinkotherStaticLibs);
             iblinkothersharedlibs :
               readlinkcontainer(LinkotherSharedLibs);
             iblinkotherframeworks :
               readlinkcontainer(LinkOtherFrameworks);
             ibmainname:
               begin
                 mainname:=ppufile.getpshortstring;
                 if (mainaliasname<>defaultmainaliasname) then
                   Message1(scan_w_multiple_main_name_overrides,mainaliasname);
                 mainaliasname:=mainname^;
               end;
             ibImportSymbols :
               readImportSymbols;
             ibderefmap :
               readderefmap;
             ibderefdata :
               readderefdata;
             ibresources:
               readResources;
             iborderedsymbols:
               readOrderedSymbols;
             ibwpofile:
               readwpofile;
             ibendinterface :
               break;
           else
             Message1(unit_f_ppu_invalid_entry,tostr(b));
           end;
           { we can already stop when we know that we must recompile }
           if do_compile then
             exit;
         until false;
      end;


    procedure tppumodule.load_implementation;
      var
        b : byte;
      begin
         { read implementation part }
         repeat
           b:=ppufile.readentry;
           case b of
             ibloadunit :
               readloadunit;
             ibasmsymbols :
               readasmsyms;
             ibunitimportsyms:
               readunitimportsyms;
             ibendimplementation :
               break;
           else
             Message1(unit_f_ppu_invalid_entry,tostr(b));
           end;
         until false;
      end;


    procedure tppumodule.writeppu;
      begin
         Message1(unit_u_ppu_write,realmodulename^);

         { create unit flags }
{$ifdef cpufpemu}
         if (cs_fp_emulation in current_settings.moduleswitches) then
           headerflags:=headerflags or uf_fpu_emulation;
{$endif cpufpemu}
{$ifdef Test_Double_checksum_write}
         Assign(CRCFile,ppufilename+'.IMP');
         Rewrite(CRCFile);
{$endif def Test_Double_checksum_write}

         { create new ppufile }
         ppufile:=tcompilerppufile.create(ppufilename);
         if not ppufile.createfile then
          Message(unit_f_ppu_cannot_write);

         { extra header (sub version, module flags) }
         writeextraheader;

         { first the (JVM) namespace }
         if assigned(namespace) then
           begin
             ppufile.putstring(namespace^);
             ppufile.writeentry(ibjvmnamespace);
           end;
         { the unitname }
         ppufile.putstring(realmodulename^);
         ppufile.writeentry(ibmodulename);

         ppufile.putset(tppuset1(moduleoptions));
         if mo_has_deprecated_msg in moduleoptions then
           ppufile.putstring(deprecatedmsg^);
         ppufile.writeentry(ibmoduleoptions);

         { write the alternate main procedure name if any }
         if assigned(mainname) then
           begin
             ppufile.putstring(mainname^);
             ppufile.writeentry(ibmainname);
           end;

         if cs_compilesystem in current_settings.moduleswitches then
           begin
             ppufile.putset(tppuset4(features));
             ppufile.writeentry(ibfeatures);
           end;

         writesourcefiles;
{$IFDEF MACRO_DIFF_HINT}
         writeusedmacros;
{$ENDIF}

         { write interface uses }
         writeusedunit(true);

         { write the objectfiles and libraries that come for this unit,
           preserve the containers because they are still needed to load
           the link.res.
            All doesn't depend on the crc! It doesn't matter
           if a unit is in a .o or .a file }
         ppufile.do_crc:=false;
         { write after source files, so that we know whether or not the compiler
           will recompile the unit when checking whether the correct wpo file is
           used (if it will recompile the unit anyway, it doesn't matter)
         }
         if (wpofeedbackinput<>'') then
           begin
             ppufile.putstring(wpofeedbackinput);
             ppufile.putlongint(getnamedfiletime(wpofeedbackinput));
             ppufile.writeentry(ibwpofile);
           end;
         writelinkcontainer(linkunitofiles,iblinkunitofiles,true);
         writelinkcontainer(linkunitstaticlibs,iblinkunitstaticlibs,true);
         writelinkcontainer(linkunitsharedlibs,iblinkunitsharedlibs,true);
         writelinkcontainer(linkotherofiles,iblinkotherofiles,false);
         writelinkcontainer(linkotherstaticlibs,iblinkotherstaticlibs,true);
         writelinkcontainer(linkothersharedlibs,iblinkothersharedlibs,true);
         writelinkcontainer(linkotherframeworks,iblinkotherframeworks,true);
         writeImportSymbols;
         writeResources;
         writeOrderedSymbols;
         ppufile.do_crc:=true;

         { generate implementation deref data, the interface deref data is
           already generated when calculating the interface crc }
         if (cs_compilesystem in current_settings.moduleswitches) then
           begin
             tstoredsymtable(globalsymtable).buildderef;
             derefdataintflen:=derefdata.size;
           end
         else
           { the unit may have been re-resolved, in which case the current
             position in derefdata is not necessarily at the end }
            derefdata.seek(derefdata.size);
         tstoredsymtable(globalsymtable).buildderefimpl;
         tunitwpoinfo(wpoinfo).buildderef;
         tunitwpoinfo(wpoinfo).buildderefimpl;

         if assigned(globalmacrosymtable) and (globalmacrosymtable.SymList.count > 0) then
            begin
              tstoredsymtable(globalmacrosymtable).buildderef;
              tstoredsymtable(globalmacrosymtable).buildderefimpl;
            end;

         if mf_local_symtable in moduleflags then
           tstoredsymtable(localsymtable).buildderef_registered;
         buildderefunitimportsyms;
         writederefmap;
         writederefdata;

         ppufile.writeentry(ibendinterface);

         { write the symtable entries }
         tstoredsymtable(globalsymtable).ppuwrite(ppufile);

         if assigned(globalmacrosymtable) and (globalmacrosymtable.SymList.count > 0) then
           begin
             ppufile.putbyte(byte(true));
             ppufile.writeentry(ibexportedmacros);
             tstoredsymtable(globalmacrosymtable).ppuwrite(ppufile);
           end
         else
           begin
             ppufile.putbyte(byte(false));
             ppufile.writeentry(ibexportedmacros);
           end;

         { everything after this doesn't affect the crc }
         ppufile.do_crc:=false;

         { write implementation uses }
         writeusedunit(false);

         { write all public assembler symbols }
         writeasmsyms(ualt_public,publicasmsyms);

         { write all external assembler symbols }
         writeasmsyms(ualt_extern,externasmsyms);

         { write all symbols imported from another unit }
         writeunitimportsyms;

         { end of implementation }
         ppufile.writeentry(ibendimplementation);

         { write static symtable
           needed for local debugging of unit functions }
         if mf_local_symtable in moduleflags then
           tstoredsymtable(localsymtable).ppuwrite(ppufile);

         { write whole program optimisation-related information }
         tunitwpoinfo(wpoinfo).ppuwrite(ppufile);

         { the last entry ibend is written automatically }

         { flush to be sure }
         ppufile.flush;
         { create and write header }
         ppufile.header.common.size:=ppufile.size;
         ppufile.header.checksum:=ppufile.crc;
         ppufile.header.interface_checksum:=ppufile.interface_crc;
         ppufile.header.indirect_checksum:=ppufile.indirect_crc;
         ppufile.header.common.compiler:=wordversion;
         ppufile.header.common.cpu:=word(target_cpu);
         ppufile.header.common.target:=word(target_info.system);
         ppufile.header.common.flags:=headerflags;
         ppufile.header.deflistsize:=current_module.deflist.count;
         ppufile.header.symlistsize:=current_module.symlist.count;
         ppufile.writeheader;

         { save crc in current module also }
         crc:=ppufile.crc;
         interface_crc:=ppufile.interface_crc;
         indirect_crc:=ppufile.indirect_crc;

{$ifdef Test_Double_checksum_write}
         close(CRCFile);
{$endif Test_Double_checksum_write}

         ppufile.closefile;
         ppufile.free;
         ppufile:=nil;
      end;


    procedure tppumodule.getppucrc;
      begin
{$ifdef Test_Double_checksum_write}
         Assign(CRCFile,ppufilename+'.INT');
         Rewrite(CRCFile);
{$endif def Test_Double_checksum_write}

         { create new ppufile }
         ppufile:=tcompilerppufile.create(ppufilename);
         ppufile.crc_only:=true;
         if not ppufile.createfile then
           Message(unit_f_ppu_cannot_write);

         { first the (JVM) namespace }
         if assigned(namespace) then
           begin
             ppufile.putstring(namespace^);
             ppufile.writeentry(ibjvmnamespace);
           end;
         { the unitname }
         ppufile.putstring(realmodulename^);
         ppufile.writeentry(ibmodulename);

         { extra header (sub version, module flags) }
         writeextraheader;

         ppufile.putset(tppuset1(moduleoptions));
         if mo_has_deprecated_msg in moduleoptions then
           ppufile.putstring(deprecatedmsg^);
         ppufile.writeentry(ibmoduleoptions);

         { the interface units affect the crc }
         writeusedunit(true);

         { deref data of interface that affect the crc }
         derefdata.reset;
         tstoredsymtable(globalsymtable).buildderef;
         derefdataintflen:=derefdata.size;
         writederefmap;
         writederefdata;

         ppufile.writeentry(ibendinterface);

         { write the symtable entries }
         tstoredsymtable(globalsymtable).ppuwrite(ppufile);

         if assigned(globalmacrosymtable) and (globalmacrosymtable.SymList.count > 0) then
           begin
             ppufile.putbyte(byte(true));
             ppufile.writeentry(ibexportedmacros);
             tstoredsymtable(globalmacrosymtable).ppuwrite(ppufile);
           end
         else
           begin
             ppufile.putbyte(byte(false));
             ppufile.writeentry(ibexportedmacros);
           end;

         { save crc  }
         crc:=ppufile.crc;
         interface_crc:=ppufile.interface_crc;
         indirect_crc:=ppufile.indirect_crc;

         { end of implementation, to generate a correct ppufile
           for ppudump when using INTFPPU define }
         ppufile.writeentry(ibendimplementation);

{$ifdef Test_Double_checksum}
         crc_array:=ppufile.crc_test;
         ppufile.crc_test:=nil;
         crc_size:=ppufile.crc_index2;
         crc_array2:=ppufile.crc_test2;
         ppufile.crc_test2:=nil;
         crc_size2:=ppufile.crc_index2;
{$endif Test_Double_checksum}

{$ifdef Test_Double_checksum_write}
         close(CRCFile);
{$endif Test_Double_checksum_write}

         { create and write header, this will only be used
           for debugging purposes }
         ppufile.header.common.size:=ppufile.size;
         ppufile.header.checksum:=ppufile.crc;
         ppufile.header.interface_checksum:=ppufile.interface_crc;
         ppufile.header.indirect_checksum:=ppufile.indirect_crc;
         ppufile.header.common.compiler:=wordversion;
         ppufile.header.common.cpu:=word(target_cpu);
         ppufile.header.common.target:=word(target_info.system);
         ppufile.header.common.flags:=headerflags;
         ppufile.writeheader;

         ppufile.closefile;
         ppufile.free;
         ppufile:=nil;
      end;


    procedure tppumodule.load_usedunits;
      var
        pu           : tused_unit;
      begin
        if current_module<>self then
         internalerror(200212284);

        { load the used units from interface }
        in_interface:=true;
        pu:=tused_unit(used_units.first);
        while assigned(pu) do
         begin
           if pu.in_interface then
            begin
              tppumodule(pu.u).loadppu;
              { if this unit is compiled we can stop }
              if state=ms_compiled then
               exit;
              { add this unit to the dependencies }
              pu.u.adddependency(self);
              { need to recompile the current unit, check the interface
                crc. And when not compiled with -Ur then check the complete
                crc }
              if (pu.u.interface_crc<>pu.interface_checksum) or
                 (pu.u.indirect_crc<>pu.indirect_checksum) or
                 (
                  (not(mf_release in moduleflags)) and
                  (pu.u.crc<>pu.checksum)
                 ) then
               begin
                 Message2(unit_u_recompile_crc_change,realmodulename^,pu.u.ppufilename,@queuecomment);
{$ifdef DEBUG_UNIT_CRC_CHANGES}
                 if (pu.u.interface_crc<>pu.interface_checksum) then
                   writeln('  intfcrc change: ',hexstr(pu.u.interface_crc,8),' <> ',hexstr(pu.interface_checksum,8))
                 else if (pu.u.indirect_crc<>pu.indirect_checksum) then
                   writeln('  indcrc change: ',hexstr(pu.u.indirect_crc,8),' <> ',hexstr(pu.indirect_checksum,8))
                 else
                   writeln('  implcrc change: ',hexstr(pu.u.crc,8),' <> ',hexstr(pu.checksum,8));
{$endif DEBUG_UNIT_CRC_CHANGES}
                 recompile_reason:=rr_crcchanged;
                 do_compile:=true;
                 exit;
               end;
            end;
           pu:=tused_unit(pu.next);
         end;

        { ok, now load the interface of this unit }
        if current_module<>self then
         internalerror(200208187);
        deflist.count:=ppufile.header.deflistsize;
        symlist.count:=ppufile.header.symlistsize;
        globalsymtable:=tglobalsymtable.create(modulename^,moduleid);
        tstoredsymtable(globalsymtable).ppuload(ppufile);

        if ppufile.readentry<>ibexportedmacros then
          Message(unit_f_ppu_read_error);
        if boolean(ppufile.getbyte) then
          begin
            globalmacrosymtable:=tmacrosymtable.Create(true);
            tstoredsymtable(globalmacrosymtable).ppuload(ppufile)
          end;

        interface_compiled:=true;

        { read the implementation part, containing
          the implementation uses and ObjData }
        in_interface:=false;
        load_implementation;

        { now only read the implementation uses }
        pu:=tused_unit(used_units.first);
        while assigned(pu) do
         begin
           if (not pu.in_interface) then
            begin
              tppumodule(pu.u).loadppu;
              { if this unit is compiled we can stop }
              if state=ms_compiled then
               exit;
              { add this unit to the dependencies }
              pu.u.adddependency(self);
              { need to recompile the current unit ? }
              if (pu.u.interface_crc<>pu.interface_checksum) or
                 (pu.u.indirect_crc<>pu.indirect_checksum) then
                begin
                  Message2(unit_u_recompile_crc_change,realmodulename^,pu.u.ppufilename+' {impl}',@queuecomment);
{$ifdef DEBUG_UNIT_CRC_CHANGES}
                  if (pu.u.interface_crc<>pu.interface_checksum) then
                    writeln('  intfcrc change (2): ',hexstr(pu.u.interface_crc,8),' <> ',hexstr(pu.interface_checksum,8))
                  else if (pu.u.indirect_crc<>pu.indirect_checksum) then
                    writeln('  indcrc change (2): ',hexstr(pu.u.indirect_crc,8),' <> ',hexstr(pu.indirect_checksum,8));
{$endif DEBUG_UNIT_CRC_CHANGES}
                  recompile_reason:=rr_crcchanged;
                  do_compile:=true;
                  exit;
                end;
            end;
           pu:=tused_unit(pu.next);
         end;

        { load implementation symtable }
        if mf_local_symtable in moduleflags then
          begin
            localsymtable:=tstaticsymtable.create(modulename^,moduleid);
            tstaticsymtable(localsymtable).ppuload(ppufile);
          end;

        { we can now derefence all pointers to the implementation parts }
        tstoredsymtable(globalsymtable).derefimpl(false);
        { we've just loaded the localsymtable from the ppu file, so everything
          in it was registered by definition (otherwise it wouldn't have been in
          there) }
        if assigned(localsymtable) then
          tstoredsymtable(localsymtable).derefimpl(false);

        derefunitimportsyms;

         { read whole program optimisation-related information }
         wpoinfo:=tunitwpoinfo.ppuload(ppufile);
         tunitwpoinfo(wpoinfo).deref;
         tunitwpoinfo(wpoinfo).derefimpl;
      end;


    function tppumodule.needrecompile:boolean;
      var
        pu : tused_unit;
      begin
        result:=false;
        pu:=tused_unit(used_units.first);
        while assigned(pu) do
         begin
           { need to recompile the current unit, check the interface
             crc. And when not compiled with -Ur then check the complete
             crc }
           if (pu.u.interface_crc<>pu.interface_checksum) or
              (pu.u.indirect_crc<>pu.indirect_checksum) or
              (
               (pu.in_interface) and
               (pu.u.crc<>pu.checksum)
              ) then
             begin
{$ifdef DEBUG_UNIT_CRC_CHANGES}
               if (pu.u.interface_crc<>pu.interface_checksum) then
                 writeln('  intfcrc change (3): ',hexstr(pu.u.interface_crc,8),' <> ',hexstr(pu.interface_checksum,8))
               else if (pu.u.indirect_crc<>pu.indirect_checksum) then
                 writeln('  indcrc change (3): ',hexstr(pu.u.indirect_crc,8),' <> ',hexstr(pu.indirect_checksum,8))
               else
                 writeln('  implcrc change (3): ',hexstr(pu.u.crc,8),' <> ',hexstr(pu.checksum,8));
{$endif DEBUG_UNIT_CRC_CHANGES}
               result:=true;
               exit;
             end;
           pu:=tused_unit(pu.next);
         end;
      end;


    procedure tppumodule.setdefgeneration;
      begin
        defsgeneration:=currentdefgeneration;
        inc(currentdefgeneration);
      end;


    procedure tppumodule.reload_flagged_units;
      var
        hp : tppumodule;
      begin
        { now reload all dependent units with outdated defs }
        hp:=tppumodule(loaded_units.first);
        while assigned(hp) do
         begin
           if hp.do_reload and
              (hp.defsgeneration<defsgeneration) then
             begin
               hp.defsgeneration:=defsgeneration;
               hp.loadppu
             end
           else
             hp.do_reload:=false;
           hp:=tppumodule(hp.next);
         end;
      end;

    procedure tppumodule.end_of_parsing;
      begin
        { module is now compiled }
        state:=ms_compiled;

        { free ppu }
        if assigned(ppufile) then
          begin
            ppufile.free;
            ppufile:=nil;
          end;

        inherited end_of_parsing;
      end;


    procedure tppumodule.loadppu;
      const
        ImplIntf : array[boolean] of string[15]=('implementation','interface');
      var
        do_load,
        second_time        : boolean;
        old_current_module : tmodule;
        pu : tused_unit;
      begin
        old_current_module:=current_module;
        Message3(unit_u_load_unit,old_current_module.modulename^,
                 ImplIntf[old_current_module.in_interface],
                 modulename^);

        { Update loaded_from to detect cycles }
        loaded_from:=old_current_module;

        { check if the globalsymtable is already available, but
          we must reload when the do_reload flag is set }
        if (not do_reload) and
           assigned(globalsymtable) then
           exit;

        { reset }
        do_load:=true;
        second_time:=false;
        set_current_module(self);

        { try to load it as a package unit first }
        if (packagelist.count>0) and loadfrompackage then
          begin
            do_load:=false;
            do_reload:=false;
            state:=ms_compiled;
            { PPU is not needed anymore }
            if assigned(ppufile) then
             begin
                ppufile.closefile;
                ppufile.free;
                ppufile:=nil;
             end;
            { add the unit to the used units list of the program }
            usedunits.concat(tused_unit.create(self,true,false,nil));
          end;

        { A force reload }
        if do_reload then
         begin
           Message(unit_u_forced_reload);
           do_reload:=false;
           { When the unit is already loaded or being loaded
             we can maybe skip a complete reload/recompile }
           if assigned(globalsymtable) and
              (not needrecompile) then
             begin
               { When we don't have any data stored yet there
                 is nothing to resolve }
               if interface_compiled then
                 begin
                   Message1(unit_u_reresolving_unit,modulename^);
                   tstoredsymtable(globalsymtable).deref(false);
                   tstoredsymtable(globalsymtable).derefimpl(false);
                   if assigned(localsymtable) then
                    begin
                      { we have only builderef(impl)'d the registered symbols of
                        the localsymtable -> also only deref those again }
                      tstoredsymtable(localsymtable).deref(true);
                      tstoredsymtable(localsymtable).derefimpl(true);
                    end;
                   if assigned(wpoinfo) then
                     begin
                       tunitwpoinfo(wpoinfo).deref;
                       tunitwpoinfo(wpoinfo).derefimpl;
                     end;

                   { We have to flag the units that depend on this unit even
                     though it didn't change, because they might also
                     indirectly depend on the unit that did change (e.g.,
                     in case rgobj, rgx86 and rgcpu have been compiled
                     already, and then rgobj is recompiled for some reason
                     -> rgx86 is re-reresolved, but the vmtentries of trgcpu
                     must also be re-resolved, because they will also contain
                     pointers to procdefs in the old trgobj (in case of a
                     recompile, all old defs are freed) }
                   flagdependent(old_current_module);
                   reload_flagged_units;
                 end
               else
                 Message1(unit_u_skipping_reresolving_unit,modulename^);
               do_load:=false;
             end;
         end;

        if do_load then
         begin
           { loading the unit for a second time? }
           if state=ms_registered then
            state:=ms_load
           else
            begin
              { try to load the unit a second time first }
              Message1(unit_u_second_load_unit,modulename^);
              Message2(unit_u_previous_state,modulename^,ModuleStateStr[state]);
              { Flag modules to reload }
              flagdependent(old_current_module);
              { Reset the module }
              reset;
              if state in [ms_compile,ms_second_compile] then
                begin
                  Message1(unit_u_second_compile_unit,modulename^);
                  state:=ms_second_compile;
                  do_compile:=true;
                end
              else
                state:=ms_second_load;
              second_time:=true;
            end;

           { close old_current_ppu on system that are
             short on file handles like DOS PM }
{$ifdef SHORT_ON_FILE_HANDLES}
           if old_current_module.is_unit and
              assigned(tppumodule(old_current_module).ppufile) then
             tppumodule(old_current_module).ppufile.tempclose;
{$endif SHORT_ON_FILE_HANDLES}

           { try to opening ppu, skip this when we already
             know that we need to compile the unit }
           if not do_compile then
            begin
              Message1(unit_u_loading_unit,modulename^);
              search_unit_files(false);
              if not do_compile then
               begin
                 load_interface;
                 setdefgeneration;
                 if not do_compile then
                  begin
                    load_usedunits;
                    if not do_compile then
                      Message1(unit_u_finished_loading_unit,modulename^);
                  end;
               end;
              { PPU is not needed anymore }
              if assigned(ppufile) then
               begin
                  ppufile.closefile;
                  ppufile.free;
                  ppufile:=nil;
               end;
            end;

           { Do we need to recompile the unit }
           if do_compile then
            begin
              { recompile the unit or give a fatal error if sources not available }
              if not(sources_avail) then
               begin
                 search_unit_files(true);
                 if not(sources_avail) then
                  begin
                    printcomments;
                    if recompile_reason=rr_noppu then
                      begin
                        pu:=tused_unit(loaded_from.used_units.first);
                        while assigned(pu) do
                          begin
                            if pu.u=self then
                              break;
                            pu:=tused_unit(pu.next);
                          end;
                        if assigned(pu) and assigned(pu.unitsym) then
                          MessagePos2(pu.unitsym.fileinfo,unit_f_cant_find_ppu,realmodulename^,loaded_from.realmodulename^)
                        else
                          Message2(unit_f_cant_find_ppu,realmodulename^,loaded_from.realmodulename^);
                      end
                    else
                      Message1(unit_f_cant_compile_unit,realmodulename^);
                  end;
               end;
              { we found the sources, we do not need the verbose messages anymore }
              if comments <> nil then
              begin
                comments.free;
                comments:=nil;
              end;
              { Flag modules to reload }
              flagdependent(old_current_module);
              { Reset the module }
              reset;
              { compile this module }
              if not(state in [ms_compile,ms_second_compile]) then
                state:=ms_compile;
              compile(mainsource);
              setdefgeneration;
            end
           else
            state:=ms_compiled;

           if current_module<>self then
             internalerror(200212282);

           if in_interface then
             internalerror(200212283);

           { for a second_time recompile reload all dependent units,
             for a first time compile register the unit _once_ }
           if second_time then
            reload_flagged_units
           else
            usedunits.concat(tused_unit.create(self,true,false,nil));

           { reopen the old module }
{$ifdef SHORT_ON_FILE_HANDLES}
           if old_current_module.is_unit and
              assigned(tppumodule(old_current_module).ppufile) then
             tppumodule(old_current_module).ppufile.tempopen;
{$endif SHORT_ON_FILE_HANDLES}
         end;

        { we are back, restore current_module }
        set_current_module(old_current_module);
      end;


{*****************************************************************************
                               RegisterUnit
*****************************************************************************}


    function registerunit(callermodule:tmodule;const s : TIDString;const fn:string) : tppumodule;
      var
        ups   : TIDString;
        hp    : tppumodule;
        hp2   : tmodule;
      begin
        { Info }
        ups:=upper(s);
        { search all loaded units }
        hp:=tppumodule(loaded_units.first);
        while assigned(hp) do
         begin
           if hp.modulename^=ups then
            begin
              { only check for units. The main program is also
                as a unit in the loaded_units list. We simply need
                to ignore this entry (PFV) }
              if hp.is_unit then
               begin
                 { both units in interface ? }
                 if callermodule.in_interface and
                    hp.in_interface then
                  begin
                    { check for a cycle }
                    hp2:=callermodule.loaded_from;
                    while assigned(hp2) and (hp2<>hp) do
                     begin
                       if hp2.in_interface then
                         hp2:=hp2.loaded_from
                       else
                         hp2:=nil;
                     end;
                    if assigned(hp2) then
                      Message2(unit_f_circular_unit_reference,callermodule.realmodulename^,hp.realmodulename^);
                  end;
                 break;
               end;
            end;
           { the next unit }
           hp:=tppumodule(hp.next);
         end;
        { the unit is not in the loaded units,
          we create an entry and register the unit }
        if not assigned(hp) then
         begin
           Message1(unit_u_registering_new_unit,Upper(s));
           hp:=tppumodule.create(callermodule,s,fn,true);
           hp.loaded_from:=callermodule;
           addloadedunit(hp);
         end;
        { return }
        registerunit:=hp;
      end;

end.
