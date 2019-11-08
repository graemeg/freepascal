{
    Copyright (c) 2009-2010 by Dmitry Boyarintsev

    Contains the binary mach-o writer

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
unit ogmacho;

{$i fpcdefs.inc}

interface

uses
  cclasses,
  globals, globtype, verbose,
  owbase, ogbase,
  aasmbase, assemble,
  macho, machoutils,
  systems,
  { assembler }
  cpuinfo,cpubase,aasmtai,aasmdata; {for system constants}

type

    { TMachoRawWriter }

    TMachoRawWriter=class(TRawWriter)
      private
        fwriter : tobjectwriter;
      public
        constructor Create(awriter: tobjectwriter);
        procedure WriteRaw(const data; datasize: Integer); override;
      end;

    { TmachoObjSection }

    TMachoSectionType=(mst_Normal, mst_ObjC, mst_Stabs, mst_Dwarf);

    TmachoObjSection=class(tObjSection)
      public
        nmsegment : string;  {mach-o segment name}
        nmsection : string;  {mach-o section name}

        inSegIdx  : Integer; {section index inside segment. one-based number}
        RelocOfs  : aword;   {file offset to the first relocation symbol}
        IndIndex  : Integer; {index in indirect table (see DysymTableCommand) for lazy and non-lazy symbol pointers}

        machoSec  : TMachoSectionType;
        function GetRelocCount: Integer;
        function FileSize: Integer;
        constructor create(AList:TFPHashObjectList; const Aname:string; Aalign:longint; Aoptions:TObjSectionOptions);override;
      end;

    { TmachoObjData }

    TmachoObjData=class(TObjData)
      public
        debugcount: Integer;
        constructor create(const n:string); override;
        procedure CreateDebugSections; override;
        function sectionname(atype:TAsmSectiontype; const aname:string; aorder:TAsmSectionOrder):string;override;
        function sectiontype2align(atype:TAsmSectiontype):longint;override;
        class function sectiontype2options(atype:TAsmSectiontype):TObjSectionOptions;override;
        procedure writereloc(data:aint; len:aword; p:TObjSymbol; reltype:TObjRelocationType);override;
      public
      end;

    { TMachoObjectOutput }

    TMachoSymbolLocation=(loc_Notused, loc_Local, loc_External, loc_Undef);

    TMachoObjectOutput=class(TObjOutput)
      private
        machoData   : TMachoObjData;
        mfile       : TMachOWriter;
        cputarget   : cpu_type_t;

        stabsec     : TmachoObjSection;
        strsec      : TmachoObjSection;

        sectionscnt : integer;
        memofs      : aword;
        fileofs     : aword;

        symstrofs   : aword;
        symlen      : aword;
        symCount    : aint;

        iLocal      : Integer;
        iExtern     : Integer;
        iUndef      : Integer;
        iIndir      : Integer;

        symList     : TFPObjectList;
        IndirIndex  : tdynamicarray;

        relcount : integer;
      protected
        procedure TrailZeros;
        function current_cpu_type: cpu_type_t;inline;

        {sections}
        procedure FixSectionRelocs(s: TMachoObjSection);
        procedure section_count_sections(p:TObject;arg:pointer);
        procedure section_set_datamempos(p:TObject;arg:pointer);
        procedure section_set_relocpos(p:TObject;arg:pointer);

        procedure section_write_data(p:TObject;arg:pointer);
        procedure section_write_relocdata(p:TObject;arg:pointer);
        procedure section_prepare_indirect(s: TObjSection);

        {symbols}
        procedure symbol_write_nlist(sym:TObjSymbol; symstr: tdynamicarray);
        function dysymbol_location(sym: TObjSymbol): TMachoSymbolLocation;

        function symWriteName(s: TObjSymbol): string;
        procedure InitSymbolIndexes(var sCount: aint; var symStrLen: aword);

        {mach-o file related}
        procedure writeSectionsHeader(s: TMachoObjSection);
        procedure writeSymTabCommand;
        procedure writeSymbols(symstr: tdynamicarray);
        procedure writeDySymTabCommand(IndOffset: aword; IndCount: Integer);
        procedure writeDysymbols;

        function writedata(data:TObjData):boolean;override;
      public
        constructor Create(AWriter:TObjectWriter);override;
      end;

    { TMachoAssembler }

    TMachoAssembler=class(TInternalAssembler)
      public
        constructor create(info: pasminfo; smart:boolean);override;
      end;


implementation

uses
  owar;

  { TmachoObjData }

  constructor TmachoObjData.create(const n: string);
    begin
      inherited create(n);
      CObjSection:=TmachoObjSection;
    end;


  { TmachoObjData.CreateDebugSections. }
  { note: mach-o file has specific symbol table command (not sections) to keep symbols and symbol string }
  procedure TmachoObjData.CreateDebugSections;
    begin
      inherited CreateDebugSections;
      if target_dbg.id=dbg_stabs then
        begin
          stabssec:=createsection(sec_stab);
          stabstrsec:=createsection(sec_stabstr);
        end;
    end;


  function TmachoObjData.sectionname(atype: TAsmSectiontype; const aname: string; aorder: TAsmSectionOrder): string;
    const
      DwarfSect : array [sec_debug_frame..sec_debug_ranges] of string
        = ('sec_debug_frame','__debug_info','__debug_line','__debug_abbrev','__debug_aranges','__debug_ranges');
    begin
      case atype of
        sec_user: Result:=aname;
        sec_bss:  Result:=MakeSectionName(seg_DATA, '__common');
        sec_stab: Result:='.stabs';
        sec_stabstr: Result:='.stabsstr';
        sec_fpc:  Result:=MakeSectionName(seg_TEXT, '.fpc');
        sec_stub: Result:=MakeSectionName(seg_IMPORT, '__jump_table');
        sec_code:
          if (aname='fpc_geteipasebx') or
             (aname='fpc_geteipasecx') then
            Result:=MakeSectionName(seg_TEXT, '__textcoal_nt')
          else
            Result:=MakeSectionName(seg_TEXT, '__text');
        sec_rodata_norel: Result:=MakeSectionName(seg_TEXT, '__const'); {.const}
        sec_rodata:       Result:=MakeSectionName(seg_DATA, '__const');
        sec_data:         Result:=MakeSectionName(seg_DATA, '__data');
        sec_data_nonlazy: Result:=MakeSectionName(seg_DATA, '__nl_symbol_ptr');
        sec_data_lazy:    Result:=MakeSectionName(seg_DATA, '__la_symbol_ptr');
        sec_init_func:    Result:=MakeSectionName(seg_DATA, '__mod_init_func');
        sec_term_func:    Result:=MakeSectionName(seg_DATA, '__mod_term_func');


        sec_objc_class:           Result:='__OBJC __class';
        sec_objc_meta_class:      Result:='__OBJC __meta_class';
        sec_objc_cat_cls_meth:    Result:='__OBJC __cat_cls_meth';
        sec_objc_cat_inst_meth:   Result:='__OBJC __cat_inst_meth';
        sec_objc_protocol:      Result:='__OBJC __protocol';
        sec_objc_string_object: Result:='__OBJC __cstring';
        sec_objc_cls_meth:        Result:='__OBJC __cls_meth';
        sec_objc_inst_meth:       Result:='__OBJC __inst_meth';
        sec_objc_cls_refs:        Result:='__OBJC __cls_refs';
        sec_objc_message_refs:    Result:='__OBJC __message_refs';
        sec_objc_symbols:         Result:='__OBJC __symbols';
        sec_objc_category:      Result:='__OBJC __categories';
        sec_objc_class_vars:    Result:='__OBJC __cls_vars';
        sec_objc_instance_vars: Result:='__OBJC __inst_vars';
        sec_objc_module_info:     Result := '__OBJC __module_info';
        sec_objc_class_names:     Result:='__TEXT __cstring';
        sec_objc_meth_var_types: Result:='__OBJC __var_types';
        sec_objc_meth_var_names:  Result:='__TEXT __cstring';
        sec_objc_selector_strs: Result:='__TEXT __cstring';
        sec_objc_protocol_ext:    Result:='__OBJC __protocol_ext';
        sec_objc_class_ext:       Result:='__OBJC __class_ext';
        sec_objc_property:        Result:='__OBJC __property';
        sec_objc_image_info:      Result:='__OBJC __image_info';
        sec_objc_cstring_object:  Result:='__OBJC __cstring_object';
        sec_objc_sel_fixup:       Result:='__OBJC __sel_fixup';
        { Objective-C non-fragile ABI }
        sec_objc_data:        Result:='__OBJC __data';
        sec_objc_const:       Result:='__OBJC __const';
        sec_objc_sup_refs:      Result:='__OBJC __supc_refs';
        sec_objc_classlist:     Result:='__OBJC __classlist';
        sec_objc_nlclasslist:   Result:='__OBJC __nlclasslist';
        sec_objc_catlist:       Result:='__OBJC __catlist';
        sec_objc_nlcatlist:     Result:='__OBJC __nlcatlist';
        sec_objc_protolist:     Result:='__OBJC __protolist';

        sec_debug_frame,
        sec_debug_info,
        sec_debug_line,
        sec_debug_abbrev,
        sec_debug_aranges,
        sec_debug_ranges:
          Result:=MakeSectionName(seg_DWARF, DwarfSect[atype])

      else
        Result:=MakeSectionName(seg_DATA, '__data');
      end;
    end;


  procedure TmachoObjData.writereloc(data: aint; len: aword; p: TObjSymbol; reltype: TObjRelocationType);
    var
      symaddr : longint;
    begin
      {stabs relocation}
      case TMachoObjSection(CurrObjSec).machoSec of

        mst_Stabs:
          begin
            if Assigned(p) then
              begin
                data:=p.address;
                CurrObjSec.addsymreloc(CurrObjSec.Size,p,reltype);
              end;
            CurrObjSec.write(data, len);
          end;

        mst_Dwarf:
          begin
            if Assigned(p) then
              begin
                CurrObjSec.addsectionReloc(CurrObjSec.Size,p.objsection,reltype);
                data:=p.address;
              end;
            CurrObjSec.write(data, len);
          end;

      else
        if assigned(p) then
          begin
            { real address of the symbol }
            symaddr:=p.address;
            { Local ObjSymbols can be resolved already or need a section reloc }
            if (p.bind=AB_LOCAL) and
               (reltype in [RELOC_RELATIVE,RELOC_ABSOLUTE{$ifdef x86_64},RELOC_ABSOLUTE32{$endif x86_64}]) then
              begin
                { For a reltype relocation in the same section the value can be calculated }
                if (p.objsection=CurrObjSec) and
                   (reltype=RELOC_RELATIVE) then
                  inc(data,symaddr-len-CurrObjSec.Size)
                else
                  begin
                    if (p.typ=AT_NONE) then
                      begin
                        {undefined symbol, using section}
                        CurrObjSec.addsectionreloc(CurrObjSec.Size,p.objsection,reltype);
                        data:=symaddr-len-CurrObjSec.Size;
                      end
                    else
                      begin
                        CurrObjSec.addsymreloc(CurrObjSec.Size,p,reltype);
                        if Assigned(p.objsection) and
                           (p.objsection.Name='__TEXT __textcoal_nt') then
                          data:=symaddr-len-CurrObjSec.Size
                        else
                          data:=p.objsection.Size;
                      end;
                  end;
              end
            else if (p.bind=AB_GLOBAL) and
                    not Assigned(p.indsymbol) and
                    (reltype<>RELOC_PIC_PAIR) then
              begin
                CurrObjSec.addsectionreloc(CurrObjSec.Size,p.objsection,reltype);
                data:=p.address;
              end
            else
              CurrObjSec.addsymreloc(CurrObjSec.Size,p,reltype);
          end; {if assigned(p) }

        CurrObjSec.write(data, len);
      end;
    end;


  function TmachoObjData.sectiontype2align(atype: TAsmSectiontype): longint;
    begin
      case atype of
        sec_bss:
          Result:=4;
        sec_stabstr, sec_stab:
          Result:=1;
        sec_stub, sec_data_lazy, sec_data_nonlazy:
          Result:=4;
      else
        Result:=inherited sectiontype2align(atype);
      end;
    end;


  class function TmachoObjData.sectiontype2options(atype: TAsmSectiontype): TObjSectionOptions;
    begin
      case atype of
        sec_objc_meth_var_names,
        sec_objc_class_names: Result:=[oso_data, oso_load];
      else
        Result:=inherited sectiontype2options(atype);
      end
    end;


  { TMachoAssembler }

  constructor TMachoAssembler.create(info: pasminfo; smart: boolean);
    begin
      inherited;
      CObjOutput:=TMachoObjectOutput;
      CInternalAr:=tarobjectwriter;
    end;


  { TMachoObjectOutput }

  procedure TMachoObjectOutput.FixSectionRelocs(s: TMachoObjSection);
    var
      i   : integer;
      ro  : TObjRelocation;
      dw  : aword;
    begin
      {todo: is it I386 only core}
      if not Assigned(s.Data) then
        Exit;

      for i:=0 to s.ObjRelocations.Count-1 do
        begin
          ro:=TObjRelocation(s.ObjRelocations[i]);

          if (Assigned(ro.objsection)) and
             (ro.objsection.Name='__TEXT __textcoal_nt') then
            Continue;

          if Assigned(ro.objsection) then
            begin
              s.Data.seek(ro.DataOffset);
              s.Data.read(dw, sizeof(aword));

              dw:=dw+ro.objsection.MemPos;

              s.Data.seek(ro.DataOffset);
              s.Data.write(dw, sizeof(aword));
            end
          else
            begin
              if ro.symbol.Name='fpc_geteipasebx' then
                Continue;
              if Assigned(ro.symbol.indsymbol) or
                 (ro.typ=RELOC_PIC_PAIR) then
                begin
                  s.Data.seek(ro.DataOffset);
                  s.Data.read(dw, sizeof(aword));
                  dw:=ro.symbol.address-dw;
                  s.Data.seek(ro.DataOffset);
                  s.Data.write(dw, sizeof(aword));
                end
              else if (ro.symbol.bind=AB_LOCAL) then
                begin
                  dw:=ro.symbol.address;
                  s.Data.seek(ro.DataOffset);
                  s.Data.write(dw, sizeof(aword));
                end;
            end;

        end;
      s.Data.seek(s.Data.Size);
    end;


  procedure TMachoObjectOutput.section_count_sections(p: TObject; arg: pointer);
    var
      s : TMachoObjSection;
    begin
      s:=TMachoObjSection(p);
      if s.machoSec=mst_Stabs then
        Exit;
      inc(sectionscnt);
      s.inSegIdx:=sectionscnt;
    end;


  procedure TMachoObjectOutput.section_set_datamempos(p: TObject; arg: pointer);
    var
      s : TMachoObjSection;
    begin
      s:=TMachoObjSection(p);
      if s.machoSec=mst_Stabs then
        Exit;

      s.setDataPos(fileofs);
      s.setMemPos(memofs);
      memofs:=Align(memofs+s.Size, s.SecAlign);

      fileofs:=AlignAddr(cputarget, fileofs);
    end;


  procedure TMachoObjectOutput.section_set_relocpos(p:TObject;arg:pointer);
    var
      s   : TMachoObjSection;
      sz  : Integer;
    begin
      s:=TMachoObjSection(p);
      if s.machoSec=mst_Stabs then
        Exit;

      sz:=s.GetRelocCount * sizeof(relocation_info);
      if sz > 0 then
        begin
          s.relocofs:=fileofs;
          inc(fileofs, sz);
          fileofs:=AlignAddr(cputarget, fileofs);
        end;
    end;


  procedure TMachoObjectOutput.section_write_data(p: TObject; arg: pointer);
    var
      s : TMachoObjSection;
    begin
      s:=TMachoObjSection(p);
      if s.machoSec=mst_Stabs then
        Exit;

      Writer.writezeros(s.DataAlignBytes);

      FixSectionRelocs(s);

      if s.Datapos<>FWriter.ObjSize then
        InternalError(200903101);
      if Assigned(s.data) then
        Writer.writearray(s.data);
      TrailZeros;
    end;


  procedure TMachoObjectOutput.section_write_relocdata(p: TObject; arg: pointer);
    var
      s       : TMachoObjSection;
      symsec  : TMachoObjSection;
      i       : Integer;
      dw      : aword;

      r       : relocation_info;
      sr      : scattered_relocation_info;
      ro      : TObjRelocation;
      symnum      : Integer;
      relpc       : Boolean;
      relextern   : Boolean;
      reltype     : Integer;

    begin
      s:=TMachoObjSection(p);

      {stabs relocation should not present in relocation table}
      if s.machoSec=mst_Stabs then
        Exit;
      {no relocation for the section}
      if s.relocofs=0 then
        Exit;
      {check file alignment}
      if s.relocofs<>FWriter.ObjSize then
        InternalError(200903102); {file misalignment}

      relcount:=s.ObjRelocations.Count;
      {the reversed order, is only to be alike Apple linker}
      for i:=s.ObjRelocations.Count-1 downto 0 do
      begin
        ro:=TObjRelocation(s.ObjRelocations[i]);

        {in-section relocation}
        if ro.symbol=nil then
          begin
            relextern:=false;
            relpc:=false;
            symnum:=TmachoObjSection(ro.objsection).inSegIdx;
            case ro.typ of
              RELOC_ABSOLUTE:
                begin
                  RelocInfo(ro.DataOffset, symnum, GENERIC_RELOC_VANILLA, ril_long, relpc, relextern, r);
                  mfile.WriteRelocation(r);
                end;
            else
              relpc:=ro.typ=RELOC_RELATIVE;
              RelocInfo(ro.DataOffset, symnum, GENERIC_RELOC_VANILLA, ril_long, relpc, relextern, r);
              mfile.WriteRelocation(r);
            end;

          end
        else
          begin
            symsec:=TMachoObjSection(ro.symbol.objsection);

            if Assigned(symsec) and
               (symsec.Name='__TEXT __textcoal_nt') then
              begin
                relextern:=true;
                symnum:=ro.symbol.symidx;
              end
            else if ro.symbol.bind=AB_EXTERNAL then
              begin
                relextern:=true;
                symnum:=ro.symbol.symidx;
              end
            else if Assigned(ro.symbol.objsection) and
                    (ro.symbol.bind=AB_LOCAL) and
                    (ro.symbol.typ in [AT_DATA,AT_METADATA]) then
              begin
                relextern:=false;
                symnum:=TMachoObjSection(ro.symbol.objsection).inSegIdx;
              end
            else if (ro.symbol.bind=AB_LOCAL) or
                    (ro.symbol.typ=AT_NONE) then
             begin
               relextern:=false;
                symnum:=s.inSegIdx
              end
            else
              begin
                relextern:=true;
                symnum:=ro.symbol.symidx;
              end;

            relpc:=false;
            relpc:=(ro.typ=RELOC_RELATIVE);
            if (ro.typ=RELOC_PIC_PAIR) then
              begin
                if ro.symbol.bind=AB_LOCAL then
                  reltype:=GENERIC_RELOC_LOCAL_SECTDIFF
                else
                  reltype:=GENERIC_RELOC_SECTDIFF;
                ScatterRelocInfo(ro.symbol.address, ro.DataOffset, reltype, ril_long, false, sr);
                mfile.WriteScatterReloc(sr);

                { the section data is already fixed to:   ro.SymbolOffset - Label.Offset }
                s.Data.seek(ro.DataOffset);
                s.Data.read(dw, sizeof(aword));
                dw:=ro.symbol.address-dw;
                ScatterRelocInfo(dw, 0, GENERIC_RELOC_PAIR, ril_long, false, sr);
                mfile.WriteScatterReloc(sr);
              end
            else
              begin
                RelocInfo(ro.DataOffset, symnum, GENERIC_RELOC_VANILLA, ril_long, relpc, relextern, r);
                mfile.WriteRelocation(r);
              end
          end;
        if Assigned(s.Data) then
          s.Data.seek(s.Data.size);
      end;
      TrailZeros;
    end;


    procedure TMachoObjectOutput.section_prepare_indirect(s: TObjSection);
      var
        t       : TObjSymbol;
        i       : Integer;
        anysym  : Boolean;
      begin
        if TmachoObjSection(s).machoSec=mst_Stabs then
          Exit;

        anysym:=false;
        for i:=0 to machoData.ObjSymbolList.Count-1 do
          begin
            t:=TObjSymbol(machoData.ObjSymbolList[i]);
            if (t.objsection=s) and Assigned(t.indsymbol) then
              begin
                if not anysym then
                  begin
                    {remember the index of the first indirect symbol. Will be used later at section header writting}
                    TmachoObjSection(s).indIndex:=IndirIndex.size div SizeOf(Integer);
                    anysym:=true;
                  end;
                IndirIndex.write(t.symidx, sizeof(Integer));
              end;
          end;

      end;


    procedure TMachoObjectOutput.symbol_write_nlist(sym:TObjSymbol; symstr: tdynamicarray);
      var
        n       : nlist_64;
        sec     : TmachoObjSection;
      begin
        sec:=TMachoObjSection(sym.objsection);
        FillChar(n, sizeof(n), 0);
        n.n_un.n_strx:=symstr.size;
        symstr.writestr(sym.Name+#0);

        if assigned(sec) and
           (sec.machoSec=mst_ObjC) and
           (sec.nmsection='__module_info') then
          begin
            n.n_type:=N_ABS or N_EXT;
            mfile.WriteNList(n);
            Exit;
          end;

        if (sym.typ=AT_NONE) then
          begin
            n.n_value:=0;
            if sym.bind<>AB_EXTERNAL then
              n.n_desc:=n.n_desc or REFERENCE_FLAG_UNDEFINED_LAZY;
            n.n_type:=n.n_type or N_EXT;
          end
        else if sym.bind=AB_LAZY then
          begin
            n.n_value:=0;
            n.n_type:=N_ABS or N_EXT;
            n.n_sect:=NO_SECT;
          end
        else
          begin
            n.n_value:=sym.address;

            if Assigned(sec) then
              begin
                n.n_sect:=sec.inSegIdx;
                n.n_type:=n.n_type or N_SECT;

                if (sym.typ=AT_FUNCTION) and
                   (sym.bind=AB_LOCAL) then
                  begin
                    n.n_type:=N_PEXT or N_EXT or N_SECT;
                    n.n_desc:=n.n_desc or N_WEAK_DEF;
                  end;
              end;
          end;

        if (sym.bind=AB_GLOBAL) and
           (n.n_type and N_PEXT=0) then
          n.n_type:=n.n_type or N_EXT;

        if (sym.typ=AT_FUNCTION) and
           (sym.bind=AB_GLOBAL) then
          n.n_desc:=n.n_desc or N_NO_DEAD_STRIP;

        if Assigned(sec) then
          begin
            if (sec.nmsection='__nl_symbol_ptr') then
              n.n_desc:=n.n_desc or REFERENCE_FLAG_UNDEFINED_NON_LAZY;
            if (sec.nmsegment=seg_Data) and (sec.nmsection='__const') then
              n.n_desc:=n.n_desc or N_NO_DEAD_STRIP;
          end;

        mfile.WriteNList(n);
      end;


    function TMachoObjectOutput.dysymbol_location(sym: TObjSymbol): TMachoSymbolLocation;
      begin
        if Assigned(sym.objsection) and
           (TMachoObjSection(sym.objsection).machoSec=mst_Stabs) then
          Result:=loc_Local
        else
          case sym.typ of
            AT_NONE:  Result:=loc_Undef;
            AT_LABEL: Result:=loc_Notused;
          else
            Result:=loc_External;
          end;
      end;


  procedure TMachoObjectOutput.writeSectionsHeader(s: TMachoObjSection);
    var
      sc      : TMachoSection;
    begin
      section_prepare_indirect(s);

      fillChar(sc, sizeof(sc), 0);
      sc.segname:=s.nmsegment;
      sc.sectname:=s.nmsection;
      sc.size:=s.Size;
      if s.FileSize>0 then
        sc.offset:=s.DataPos
      else
        sc.offset:=0;
      sc.addr:=s.MemPos;
      sc.nreloc:=s.GetRelocCount;
      sc.reloff:=s.relocofs;
      sc.flags:=GetSectionFlags(s.nmsegment, s.nmsection);
      sc.align:=MachoAlign(s.SecAlign);
      sc.indirectIndex:=s.indIndex;

      if (sc.flags and SECTION_TYPE)=S_SYMBOL_STUBS then
        sc.stubSize:=GetStubSize(cputarget, false);
      mfile.WriteSection(sc);
    end;


  procedure TMachoObjectOutput.writeSymTabCommand;
    begin
      mfile.WriteLoadCommand(LC_SYMTAB, sizeof(symtab_command));
      mfile.WriteUint32(fileofs); {symoff}
      mfile.WriteUint32(symCount); {nsyms}
      inc(fileofs, symCount*sizeNList(cputarget));
      fileofs:=AlignAddr(cputarget, fileofs);

      symstrofs:=fileofs;
      mfile.WriteUint32(fileofs); {stroff}
      mfile.WriteUint32(symlen); {strsize}

      inc(fileofs, symlen);
      fileofs:=AlignAddr(cputarget, fileofs);
    end;


    function TMachoObjectOutput.symWriteName(s: TObjSymbol): string;
      begin
        if not Assigned(s.indsymbol) then
          Result:=s.Name
        else
          Result:=s.indsymbol.Name;
      end;


{    function getSymWriteNameLength(s: TObjSymbol): Integer; inline;
      begin
        Result:=length(symWriteName(s))+1;
      end;}


    procedure TMachoObjectOutput.InitSymbolIndexes(var sCount: aint; var symStrLen: aword);
      var
        i         : integer;
        s         : TObjSymbol;
        stabcount : Integer;
      begin
        sCount:=0;
        symStrLen:=0;

        iIndir:=0;
        for i:=0 to machoData.ObjSymbolList.Count-1 do
          begin
            s:=TObjSymbol(machoData.ObjSymbolList[i]);
            if (s.typ=AT_LABEL) then
              Continue;

            if Assigned(s.indsymbol) then
              inc(iIndir);
          end;

        iLocal:=0;
        iExtern:=0;
        iUndef:=0;

        for i:=0 to machoData.ObjSymbolList.Count-1 do
          begin
            s:=TObjSymbol(machoData.ObjSymbolList[i]);
            if (s.typ=AT_LABEL) or
               Assigned(s.indsymbol) then
                 Continue;
            if (s.bind=AB_LOCAL) and
               (s.Name <> 'fpc_geteipasebx') then
              Continue;

            case dysymbol_location(s) of
              loc_Local:
                begin
                  symList.Insert(iLocal, s);
                  inc(iLocal); inc(iExtern); inc(iUndef);
                end;
              loc_External:
                begin
                  symList.Insert(iExtern, s);
                  inc(iExtern); inc(iUndef);
                end;
              loc_Undef:
                begin
                  symList.Insert(iUndef, s);
                  inc(iUndef);
                end;
              loc_Notused:
                ;
            end;
            inc(symStrLen, length(s.Name)+1 );
          end;

        if Assigned(stabsec) then
          {skipping hdrsym! (added by ogbase) }
          stabcount:=stabsec.Size div sizeof(TObjStabEntry) - 1
        else
          stabcount:=0;

        for i:=0 to symList.Count-1 do
          TObjSymbol(symList[i]).symidx:=i+stabcount;
        sCount:=symList.Count+stabcount;

        for i:=0 to machoData.ObjSymbolList.Count-1 do
          with TObjSymbol(machoData.ObjSymbolList[i]) do
            if Assigned(indsymbol) then
              symidx:=indsymbol.symidx;

        if Assigned(strsec) then
          // 1 byte of zero name (that stands in the end of table, not at zero pos)
          inc(symlen, strsec.Size + 1)
        else
          inc(symlen); {the first zero byte}

        dec(iUndef, iExtern); { iUndef is count of undefined symbols (for dysymtable command) }
        dec(iExtern, iLocal); { iExtern is count of external symbols (for dysymtable command) }
        inc(iLocal, stabcount);
      end;


    procedure TMachoObjectOutput.writeSymbols(symstr: tdynamicarray);
      var
        i       : integer;
        s       : TObjSymbol;
        b       : byte;
        stab    : TObjStabEntry;
        ro      : TObjRelocation;
        sym     : TObjSymbol;
        addr    : aword;
        text    : TmachoObjSection;
        funofs  : AWord;
      begin
        if Assigned(stabsec) then
          begin
            for i:=0 to stabsec.ObjRelocations.Count - 1 do
              begin
                ro:=TObjRelocation(stabsec.ObjRelocations[i]);
                sym:=ro.symbol;
                addr:=sym.address;
                if Assigned(sym.objsection) then
                  begin
                    stabsec.Data.seek(ro.DataOffset-3);
                    b:=TmachoObjSection(sym.objsection).inSegIdx;
                    stabsec.Data.write(b, sizeof(b));
                  end;
                stabsec.Data.seek(ro.DataOffset);
                stabsec.Data.write(addr, sizeof(addr));
              end;

            stabsec.Data.seek(sizeof(TObjStabEntry));
            funofs:=0;
            text:=TmachoObjSection(machoData.ObjSectionList.Find(MakeSectionName(seg_TEXT, '__text')));
            for i:=1 to stabsec.Data.size div SizeOf(TObjStabEntry) - 1 do
              begin
                stabsec.Data.read(stab, sizeof(stab));
                case stab.ntype of
                  N_FUN:
                    begin
                      if stab.strpos=0 then
                        funofs:=0
                      else
                        funofs:=stab.nvalue;
                    end;
                  N_SLINE,N_RBRAC,N_LBRAC:
                    begin
                      if Assigned(text) then
                        begin
                          { SLINE are expected to be in  __TEXT __text only }
                          stab.nother:=text.inSegIdx;
                          inc(stab.nvalue, funofs);
                        end;
                    end;
                  N_OSO:
                    begin
                      { null-terminated string is the first in the list         }
                      { apple-gdb doesn't recognize it as zero-string for N_OSO }
                      { another zero-string should be added to the list         }
                      if stab.strpos=0 then
                        stab.strpos:=symstr.Size;
                    end;
                end;
                FWriter.write(stab, sizeof(stab));
              end;
          end;

        symstr.Seek(symStr.size);
        b:=0;
        symstr.Write(b,1);

        for i:=0 to symList.Count-1 do
          begin
            s:=TObjSymbol(symList[i]);
            symbol_write_nlist(s, symstr);
          end;
      end;


  procedure TMachoObjectOutput.writeDySymTabCommand(IndOffset: aword; IndCount: Integer);
    begin
      mfile.WriteLoadCommand(LC_DYSYMTAB, sizeof(dysymtab_command));

      mfile.WriteUint32(0); {ilocalsym}
      mfile.WriteUint32(iLocal); {nlocalsym}

      mfile.WriteUint32(iLocal); {iextdefsym}
      mfile.WriteUint32(iExtern); {nextdefsym}

      mfile.WriteUint32(iLocal + iExtern); {iundefsym}
      mfile.WriteUint32(iUndef); {nundefsym}

      mfile.WriteUint32(0); {tocoff}
      mfile.WriteUint32(0); {ntoc}
      mfile.WriteUint32(0); {modtaboff}
      mfile.WriteUint32(0); {nmodtab}
      mfile.WriteUint32(0); {extrefsymoff}
      mfile.WriteUint32(0); {nextrefsyms}
      mfile.WriteUint32(IndOffset);  {indirectsymoff}
      mfile.WriteUint32(IndCount);   {nindirectsyms}
      mfile.WriteUint32(0); {extreloff}
      mfile.WriteUint32(0); {nextrel}
      mfile.WriteUint32(0); {locreloff}
      mfile.WriteUint32(0); {nlocrel}
    end;


  procedure TMachoObjectOutput.writeDysymbols;
    var
      i   : integer;
      idx : LongWord;
    begin
      IndirIndex.seek(0);
      for i:=0 to (IndirIndex.size div sizeof(Integer))-1 do
        begin
          IndirIndex.read(idx, sizeof(idx));
          mfile.WriteUint32(idx);
        end;
    end;


  function AddSectionToSegment(var segment: TMachoSegment; section : TMachoObjSection): boolean;
    begin
      { sections must be attached one-by-one to the segment }
      if segment.fileoff=0 then
        segment.fileoff:=section.DataPos;

      if (segment.fileoff+segment.filesize)<(section.FileSize+section.DataPos) then
        segment.filesize:=section.FileSize+section.DataPos;

      inc(segment.nsects);
      inc(segment.vmsize, section.size);
      Result:=true;
   end;


  procedure TMachoObjectOutput.TrailZeros;
    var
      sz : LongWord;
    begin
      sz:=AlignAddr(cputarget, FWriter.Size);
      if sz - FWriter.Size>0 then
        FWriter.WriteZeros(sz-FWriter.Size);
    end;


  function TMachoObjectOutput.current_cpu_type: cpu_type_t;
    begin
{$if defined(powerpc)}
      result:=CPU_TYPE_POWERPC;
{$elseif defined(powerpc64)}
      result:=CPU_TYPE_POWERPC64;
{$elseif defined(i386)}
      result:=CPU_TYPE_I386;
{$elseif defined(x86_64)}
      result:=CPU_TYPE_X86_64;
{$elseif defined(arm)}
      result:=CPU_TYPE_ARM;
{$elseif defined(aarch64)}
      result:=CPU_TYPE_ARM64;
{$else}
      result:=CPU_TYPE_ANY;
{$endif}
    end;

  function TMachoObjectOutput.writedata(data: TObjData): boolean;
    var
      header  : TMachHeader;
      seg     : TMachoSegment;
      secobj  : TMachoObjSection;
      i       : Integer;

      symstr  : tdynamicarray;
      segSize : integer; {size of a segment command - platform dependant}
      sctSize : integer; {size of a single section header - platform dependant}

      indOfs: aword;   {indirect symbol offset}

    begin
      symList:=TFPObjectList.Create(false);
      IndirIndex:=tdynamicarray.Create(1024);

      result:=false;
      machoData:=TMachoObjData(data);

      cputarget:=current_cpu_type;
      segSize:=sizeSegment(cputarget);
      sctSize:=sizeSection(cputarget);

      sectionscnt:=0;
      stabsec:=TMachoObjSection(machoData.ObjSectionList.Find('.stabs'));
      strsec:=TMachoObjSection(machoData.ObjSectionList.Find('.stabsstr'));

      {count number of sections}
      machoData.ObjSectionList.ForEachCall(@section_count_sections, nil);

      {sections data is written after machheader,load-commands.   }
      {   basic loadcommands for MH_OBJECT are                    }
      {   single LC_SEGMENT, containing all sections headers      }
      {   symbol linking information at LC_SYMTAB and LC_DYSYMTAB }
      header.cputype:=cputarget;
      header.cpusubtype:=CPU_SUBTYPE_i386_ALL;
      header.filetype:=MH_OBJECT;
      header.ncmds:=3;
      header.sizeofcmds:=segSize+sctSize*sectionscnt+sizeof(symtab_command)+sizeof(dysymtab_command);
      header.flags:=0;

      {setting sections data and memory pos}
      fileofs:=sizeMachHeader(cputarget)+header.sizeofcmds;
      fileofs:=AlignAddr(cputarget, fileofs);
      memofs:=0;

      machoData.ObjSectionList.ForEachCall(@section_set_datamempos, nil);
      fileofs:=AlignAddr(cputarget, fileofs);

      {setting sections relocation offsets}
      machoData.ObjSectionList.ForEachCall(@section_set_relocpos, nil);
      fileofs:=AlignAddr(cputarget, fileofs);

      {creating actual mach-o file writer}
      mfile:=AllocMachoWriter(cputarget, TMachoRawWriter.Create(writer), true);
      {writing macho-o header}
      mfile.WriteHeader(header);

      {starting the first segment command}
      InitSegment(seg);

      {initialze symbols. some sections (non_lazy, lazy pointers) are effected}
      InitSymbolIndexes(symCount, symlen);

      for i:=0 to machoData.ObjSectionList.Count-1 do
        begin
          secobj:=TmachoObjSection(machoData.ObjSectionList[i]);
          if secobj.machoSec=mst_Stabs then
            Continue;
          AddSectionToSegment(seg, secobj);
        end;

      {writting segment command}
      {for MH_OBJECT, all sections are stored in the single segment}
      mfile.WriteSegmentCmd(seg, segSize+(seg.nsects)*sctSize);

      {section headers are written inside segment command}
      for i:=0 to machoData.ObjSectionlist.Count - 1 do
        begin
          secobj:=TmachoObjSection(machoData.ObjSectionList[i]);
          if secobj.machoSec=mst_Stabs then
            Continue;
          writeSectionsHeader(secobj);
        end;
      TrailZeros;

      if IndirIndex.size div sizeof(Integer)<>iIndir then
        InternalError(2009121001);
      if iIndir>0 then
        indOfs:=fileOfs
      else
        indOfs:=0;
      inc(fileofs, IndirIndex.size);

      {write symtab command}
      {initilize symbos order. local first, extern second, undef last}
      writeSymTabCommand;
      TrailZeros;

      {write dysymtab command}
      writeDySymTabCommand(indofs, iIndir);
      TrailZeros;

      {writting sections data, to precalculated offsets}
      {if precalculated offsets, doesn't match actual written offsets, internal error is risen}
      machoData.ObjSectionList.ForEachCall(@section_write_data, nil);

      {writting relocation offsets}
      machoData.ObjSectionList.ForEachCall(@section_write_relocdata, nil);

      {writting dyn symbol tables (indirect symbols arrays)}
      writeDysymbols;

      {writting symbol table}
      if Assigned(strsec) then
        symstr:=strsec.Data
      else
        symstr:=tdynamicarray.create(1024);

      writeSymbols(symstr);
      TrailZeros;

      {writting symbol table strings}
      FWriter.writearray(symstr);
      // terminating null name
      TrailZeros;

      if not Assigned(strsec) then
        symstr.Free;

      TrailZeros;

      mfile.Free;
      symList.Free;
      IndirIndex.Free;
    end;


  constructor TMachoObjectOutput.Create(AWriter: TObjectWriter);
    begin
      inherited Create(AWriter);
      CObjData:=TMachoObjData;
    end;


  { TMachoRawWriter }

  constructor TMachoRawWriter.Create(awriter: tobjectwriter);
    begin
      inherited Create;
      fwriter:=awriter;
    end;


  procedure TMachoRawWriter.WriteRaw(const data; datasize: Integer);
    begin
      fwriter.Write(data, datasize);
    end;


  { TmachoObjSection }

  function TmachoObjSection.GetRelocCount: Integer;
    var
      i: integer;
      r: TObjRelocation;
    begin
      Result:=ObjRelocations.Count;
      for i:=0 to ObjRelocations.Count-1 do
        begin
          r:=TObjRelocation(ObjRelocations[i]);
          if (r.typ=RELOC_PIC_PAIR) then
            inc(Result);
        end;
    end;


  function TmachoObjSection.FileSize: Integer;
    begin
      if Assigned(data) then
        Result:=data.size
      else
        Result:=0;
    end;


  constructor TmachoObjSection.create(AList: TFPHashObjectList;
    const Aname: string; Aalign: longint; Aoptions: TObjSectionOptions);
    begin
      if Aname = '__TEXT __textcoal_nt' then
        Aalign:=4;

      inherited create(AList, Aname, Aalign, Aoptions);
      GetSegmentSectionName(aName, nmsegment, nmsection);
      if (aname='.stabs') or
         (aname='.stabsstr') then
        machoSec:=mst_Stabs
      else if nmsegment=seg_DWARF then
        machoSec:=mst_Dwarf
      else if nmsegment=seg_OBJC then
        machoSec:=mst_ObjC
      else
        machoSec:=mst_Normal;
    end;


  const
    as_i386_darwin_info : tasminfo =
      (
        id     : as_i386_macho;
        idtxt  : 'MACHO';
        asmbin : '';
        asmcmd : '';
        supported_targets : [system_i386_darwin,system_i386_iphonesim];
        flags : [af_outputbinary,af_smartlink_sections,af_supports_dwarf{, af_stabs_use_function_absolute_addresses}];
        labelprefix : '.L';
        comment : '#';
        dollarsign: '$';
      );

initialization
{$ifdef i386}
  RegisterAssembler(as_i386_darwin_info,TMachoAssembler);
{$endif i386}

end.

