{
    Copyright (c) 1998-2006 by Florian Klaempfl

    This unit implements an abstract asmoutput class for all processor types

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
{ @abstract(This unit implements an abstract asm output class for all processor types)
  This unit implements an abstract assembler output class for all processors, these
  are then overridden for each assembler writer to actually write the data in these
  classes to an assembler file.
}

unit aasmdata;

{$i fpcdefs.inc}

interface

    uses
       cutils,cclasses,
       globtype,systems,
       cgbase,
       symtype,
       aasmbase;

    type
      { Type of AsmLists. The order is important for the layout of the
        information in the .o file. The stabs for the types must be defined
        before they can be referenced and therefor they need to be written
        first (PFV) }
      TAsmListType=(
        al_start,
        al_stabs,
        { pure assembler routines }
        al_pure_assembler,
        al_procedures,
        al_globals,
        al_const,
        al_typedconsts,
        al_rotypedconsts,
        al_threadvars,
        al_imports,
        al_exports,
        al_resources,
        al_rtti,
        al_dwarf_frame,
        al_dwarf_info,
        al_dwarf_abbrev,
        al_dwarf_line,
        al_picdata,
        al_indirectpicdata,
        al_resourcestrings,
<<<<<<< HEAD
        { Objective-C related sections }
        al_objc_data,
        { keep pool data separate, so we can generate new pool entries
          while emitting other data }
        al_objc_pools,
=======
>>>>>>> origin/fixes_2.4
        al_end
      );

      { Type of constant 'pools'. Mostly for string types, but usable for
        floating point and large set constants, too. }
      
      TConstPoolType = (
         sp_invalid,
         sp_conststr,
         sp_shortstr,
         sp_longstr,
         sp_ansistr,
         sp_widestr,
<<<<<<< HEAD
         sp_unicodestr,
         sp_objcclassnamerefs,
         sp_varnamerefs,
         sp_objcclassnames,
         sp_objcvarnames,
         sp_objcvartypes,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         sp_objcprotocolrefs,
         sp_varsets,
         sp_floats,
         sp_guids
=======
         sp_objcprotocolrefs
>>>>>>> graemeg/cpstrnew
=======
         sp_objcprotocolrefs
>>>>>>> graemeg/cpstrnew
=======
         sp_objcprotocolrefs
>>>>>>> graemeg/cpstrnew
=======
         sp_objcprotocolrefs
>>>>>>> origin/cpstrnew
=======
         sp_unicodestr
>>>>>>> origin/fixes_2.4
      );
      
    const
      AsmListTypeStr : array[TAsmListType] of string[24] =(
        'al_begin',
        'al_stabs',
        'al_pure_assembler',
        'al_procedures',
        'al_globals',
        'al_const',
        'al_typedconsts',
        'al_rotypedconsts',
        'al_threadvars',
        'al_imports',
        'al_exports',
        'al_resources',
        'al_rtti',
        'al_dwarf_frame',
        'al_dwarf_info',
        'al_dwarf_abbrev',
        'al_dwarf_line',
        'al_picdata',
        'al_indirectpicdata',
        'al_resourcestrings',
<<<<<<< HEAD
        'al_objc_data',
        'al_objc_pools',
=======
>>>>>>> origin/fixes_2.4
        'al_end'
      );

    type
      TAsmList = class(tlinkedlist)
         constructor create;
         function  getlasttaifilepos : pfileposinfo;
      end;

      TAsmCFI=class
      public
        constructor create;virtual;
        destructor destroy;override;
        procedure generate_code(list:TAsmList);virtual;
        procedure start_frame(list:TAsmList);virtual;
        procedure end_frame(list:TAsmList);virtual;
        procedure cfa_offset(list:TAsmList;reg:tregister;ofs:longint);virtual;
        procedure cfa_restore(list:TAsmList;reg:tregister);virtual;
        procedure cfa_def_cfa_register(list:TAsmList;reg:tregister);virtual;
        procedure cfa_def_cfa_offset(list:TAsmList;ofs:longint);virtual;
      end;
      TAsmCFIClass=class of TAsmCFI;

      { TAsmData }

      TAsmData = class
      private
        { Symbols }
        FAsmSymbolDict : TFPHashObjectList;
        FAltSymbolList : TFPObjectList;
        FNextAltNr     : longint;
        FNextLabelNr   : array[TAsmLabeltype] of longint;
        { Call Frame Information for stack unwinding}
        FAsmCFI        : TAsmCFI;
        FConstPools    : array[TConstPoolType] of THashSet;
        function GetConstPools(APoolType: TConstPoolType): THashSet;
      public
        name          : pshortstring;       { owned by tmodule }
        NextVTEntryNr : longint;
        { Assembler lists }
        AsmLists      : array[TAsmListType] of TAsmList;
        CurrAsmList   : TAsmList;
        WideInits     : TLinkedList;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        ResStrInits   : TLinkedList;
        constructor create(n: pshortstring);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        { hash tables for reusing constant storage }
        ConstPools    : array[TConstPoolType] of THashSet;
        constructor create(const n:string);
>>>>>>> graemeg/cpstrnew
        destructor  destroy;override;
        { asmsymbol }
        function  DefineAsmSymbolByClass(symclass: TAsmSymbolClass; const s : TSymStr;_bind:TAsmSymBind;_typ:Tasmsymtype) : TAsmSymbol;
        function  DefineAsmSymbol(const s : TSymStr;_bind:TAsmSymBind;_typ:Tasmsymtype) : TAsmSymbol;
        function  WeakRefAsmSymbol(const s : TSymStr;_typ:Tasmsymtype=AT_NONE) : TAsmSymbol;
        function  RefAsmSymbol(const s : TSymStr;_typ:Tasmsymtype=AT_NONE) : TAsmSymbol;
        function  GetAsmSymbol(const s : TSymStr) : TAsmSymbol;
        { create new assembler label }
        procedure getlabel(out l : TAsmLabel;alt:TAsmLabeltype);
        procedure getjumplabel(out l : TAsmLabel);
        procedure getglobaljumplabel(out l : TAsmLabel);
        procedure getaddrlabel(out l : TAsmLabel);
        { visible from outside current object }
        procedure getglobaldatalabel(out l : TAsmLabel);
        { visible only inside current object, but doesn't start with
          target_asm.label_prefix (treated the Darwin linker as the start of a
          dead-strippable data block) }
        procedure getstaticdatalabel(out l : TAsmLabel);
        { visible only inside the current object and does start with
          target_asm.label_prefix (not treated by the Darwin linker as the start
          of a dead-strippable data block, and references to such labels are
          also ignored to determine whether a data block should be live) }
        procedure getlocaldatalabel(out l : TAsmLabel);
        { generate an alternative (duplicate) symbol }
        procedure GenerateAltSymbol(p:TAsmSymbol);
        procedure ResetAltSymbols;
        property AsmSymbolDict:TFPHashObjectList read FAsmSymbolDict;
        property AsmCFI:TAsmCFI read FAsmCFI;
        { hash tables for reusing constant storage }
        property ConstPools[APoolType:TConstPoolType]: THashSet read GetConstPools;
      end;
      TAsmDataClass = class of TAsmData;

      TTCInitItem = class(TLinkedListItem)
        sym: tsym;
        offset: aint;
        datalabel: TAsmSymbol;
        constructor Create(asym: tsym; aoffset: aint; alabel: TAsmSymbol);
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    const
      casmdata: TAsmDataClass = TAsmData;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      TTCInitItem = class(TLinkedListItem)
        sym: tsym;
        offset: aint;
        datalabel: TAsmLabel;
        constructor Create(asym: tsym; aoffset: aint; alabel: TAsmLabel);
      end;
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

    var
      CAsmCFI : TAsmCFIClass;
      current_asmdata : TAsmData;


implementation

    uses
      verbose,
      aasmtai;

{$ifdef MEMDEBUG}
    var
      memasmsymbols,
      memasmcfi,
      memasmlists : TMemDebug;
{$endif MEMDEBUG}


{*****************************************************************************
                                 TAsmCFI
*****************************************************************************}

    constructor TAsmCFI.create;
      begin
      end;


    destructor TAsmCFI.destroy;
      begin
      end;


    procedure TAsmCFI.generate_code(list:TAsmList);
      begin
      end;


    procedure TAsmCFI.start_frame(list:TAsmList);
      begin
      end;


    procedure TAsmCFI.end_frame(list:TAsmList);
      begin
      end;


    procedure TAsmCFI.cfa_offset(list:TAsmList;reg:tregister;ofs:longint);
      begin
      end;


    procedure TAsmCFI.cfa_restore(list:TAsmList;reg:tregister);
      begin
      end;


    procedure TAsmCFI.cfa_def_cfa_register(list:TAsmList;reg:tregister);
      begin
      end;


    procedure TAsmCFI.cfa_def_cfa_offset(list:TAsmList;ofs:longint);
      begin
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{*****************************************************************************
                                 TTCInitItem
*****************************************************************************}


    constructor TTCInitItem.Create(asym: tsym; aoffset: aint; alabel: TAsmLabel);
      begin
        inherited Create;
        sym:=asym;
        offset:=aoffset;
        datalabel:=alabel;
      end;

>>>>>>> graemeg/cpstrnew
{*****************************************************************************
                                 TTCInitItem
*****************************************************************************}


    constructor TTCInitItem.Create(asym: tsym; aoffset: aint; alabel: TAsmSymbol);
      begin
        inherited Create;
        sym:=asym;
        offset:=aoffset;
        datalabel:=alabel;
      end;

{*****************************************************************************
                                 TAsmList
*****************************************************************************}

    constructor TAsmList.create;
      begin
        inherited create;
      end;


    function TAsmList.getlasttaifilepos : pfileposinfo;
      var
       hp : tlinkedlistitem;
      begin
         getlasttaifilepos := nil;
         if assigned(last) then
           begin
              { find the last file information record }
              if not (tai(last).typ in SkipLineInfo) then
                getlasttaifilepos:=@tailineinfo(last).fileinfo
              else
               { go through list backwards to find the first entry
                 with line information
               }
               begin
                 hp:=tai(last);
                 while assigned(hp) and (tai(hp).typ in SkipLineInfo) do
                    hp:=hp.Previous;
                 { found entry }
                 if assigned(hp) then
                   getlasttaifilepos:=@tailineinfo(hp).fileinfo
               end;
           end;
      end;


{****************************************************************************
                                TAsmData
****************************************************************************}

    function TAsmData.GetConstPools(APoolType: TConstPoolType): THashSet;
      begin
        if FConstPools[APoolType] = nil then
          case APoolType of
            sp_ansistr: FConstPools[APoolType] := TTagHashSet.Create(64, True, False);
          else
            FConstPools[APoolType] := THashSet.Create(64, True, False);
          end;
        Result := FConstPools[APoolType];
      end;

    constructor TAsmData.create(n:pshortstring);
      var
        alt : TAsmLabelType;
        hal : TAsmListType;
      begin
        inherited create;
        name:=n;
        { symbols }
        FAsmSymbolDict:=TFPHashObjectList.create(true);
        FAltSymbolList:=TFPObjectList.Create(false);
        { labels }
        FNextAltNr:=1;
        for alt:=low(TAsmLabelType) to high(TAsmLabelType) do
          FNextLabelNr[alt]:=1;
        { AsmLists }
        CurrAsmList:=TAsmList.create;
        for hal:=low(TAsmListType) to high(TAsmListType) do
          AsmLists[hal]:=TAsmList.create;
<<<<<<< HEAD
        WideInits :=TLinkedList.create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        ResStrInits:=TLinkedList.create;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
        { PIC data }
        if (target_info.system in systems_darwin) then
          AsmLists[al_picdata].concat(tai_directive.create(asd_non_lazy_symbol_pointer,''));
>>>>>>> graemeg/fixes_2_2
        { CFI }
        FAsmCFI:=CAsmCFI.Create;
      end;


    destructor TAsmData.destroy;
      var
        hal : TAsmListType;
        hp  : TConstPoolType;
      begin
        { Symbols }
{$ifdef MEMDEBUG}
        memasmsymbols.start;
{$endif}
        FAltSymbolList.free;
        FAsmSymbolDict.free;
{$ifdef MEMDEBUG}
        memasmsymbols.stop;
{$endif}
        { CFI }
{$ifdef MEMDEBUG}
        memasmcfi.start;
{$endif}
        FAsmCFI.free;
{$ifdef MEMDEBUG}
        memasmcfi.stop;
{$endif}
        { Lists }
{$ifdef MEMDEBUG}
         memasmlists.start;
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        ResStrInits.free;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        WideInits.free;
         for hal:=low(TAsmListType) to high(TAsmListType) do
           AsmLists[hal].free;
         CurrAsmList.free;
{$ifdef MEMDEBUG}
         memasmlists.stop;
{$endif}
         for hp := low(TConstPoolType) to high(TConstPoolType) do
           FConstPools[hp].Free;
      end;


    function TAsmData.DefineAsmSymbolByClass(symclass: TAsmSymbolClass; const s : TSymStr;_bind:TAsmSymBind;_typ:Tasmsymtype) : TAsmSymbol;
      var
        hp : TAsmSymbol;
      begin
        hp:=TAsmSymbol(FAsmSymbolDict.Find(s));
        if assigned(hp) then
         begin
           { Redefine is allowed, but the types must be the same. The redefine
             is needed for Darwin where the labels are first allocated }
           if not(hp.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL]) then
             begin
               if (hp.bind<>_bind) and
                  (hp.typ<>_typ) then
                 internalerror(200603261);
             end;
           hp.typ:=_typ;
           { Changing bind from AB_GLOBAL to AB_LOCAL is wrong
             if bind is already AB_GLOBAL or AB_EXTERNAL,
             GOT might have been used, so change might be harmful. }
           if (_bind<>hp.bind) and (hp.getrefs>0) then
             begin
{$ifdef extdebug}
               { the changes that matter must become internalerrors, the rest
                 should be ignored; a used cannot change anything about this,
                 so printing a warning/hint is not useful }
               if (_bind=AB_LOCAL) then
                 Message3(asmw_w_changing_bind_type,s,asmsymbindname[hp.bind],asmsymbindname[_bind])
               else
                 Message3(asmw_h_changing_bind_type,s,asmsymbindname[hp.bind],asmsymbindname[_bind]);
{$endif extdebug}
             end;
           hp.bind:=_bind;
         end
        else
         begin
           { Not found, insert it. }
           hp:=symclass.create(AsmSymbolDict,s,_bind,_typ);
         end;
        result:=hp;
      end;


    function TAsmData.DefineAsmSymbol(const s : TSymStr;_bind:TAsmSymBind;_typ:Tasmsymtype) : TAsmSymbol;
      begin
        result:=DefineAsmSymbolByClass(TAsmSymbol,s,_bind,_typ);
      end;


    function TAsmData.RefAsmSymbol(const s : TSymStr;_typ:Tasmsymtype=AT_NONE) : TAsmSymbol;
      begin
        result:=TAsmSymbol(FAsmSymbolDict.Find(s));
        if not assigned(result) then
          result:=TAsmSymbol.create(AsmSymbolDict,s,AB_EXTERNAL,_typ)
        { one normal reference removes the "weak" character of a symbol }
        else if (result.bind=AB_WEAK_EXTERNAL) then
          result.bind:=AB_EXTERNAL;
      end;


    function TAsmData.WeakRefAsmSymbol(const s : TSymStr;_typ:Tasmsymtype=AT_NONE) : TAsmSymbol;
      begin
        result:=TAsmSymbol(FAsmSymbolDict.Find(s));
        if not assigned(result) then
          result:=TAsmSymbol.create(AsmSymbolDict,s,AB_WEAK_EXTERNAL,_typ);
      end;


    function TAsmData.GetAsmSymbol(const s : TSymStr) : TAsmSymbol;
      begin
        result:=TAsmSymbol(FAsmSymbolDict.Find(s));
      end;


    procedure TAsmData.GenerateAltSymbol(p:TAsmSymbol);
      begin
        if not assigned(p.altsymbol) then
         begin
           p.altsymbol:=p.getaltcopy(AsmSymbolDict,FNextAltNr);
           FAltSymbolList.Add(p);
         end;
      end;


    procedure TAsmData.ResetAltSymbols;
      var
        i  : longint;
      begin
        for i:=0 to FAltSymbolList.Count-1 do
          TAsmSymbol(FAltSymbolList[i]).altsymbol:=nil;
        FAltSymbolList.Clear;
      end;


    procedure TAsmData.getlabel(out l : TAsmLabel;alt:TAsmLabeltype);
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if (target_info.system in (systems_linux + systems_bsd + systems_android)) and
           { the next condition was
             (cs_create_smart in current_settings.moduleswitches) and
             but if we create_smartlink_sections, this is useless }
           (create_smartlink_library) and
           (alt = alt_dbgline) then
          l:=TAsmLabel.createglobal(AsmSymbolDict,name^,FNextLabelNr[alt],alt)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if (target_info.system in systems_linux) and
           (cs_link_smart in current_settings.globalswitches) and
           (alt = alt_dbgline) then
          l:=TAsmLabel.createglobal(AsmSymbolDict,name,FNextLabelNr[alt],alt)
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
        else
          l:=TAsmLabel.createlocal(AsmSymbolDict,FNextLabelNr[alt],alt);
        inc(FNextLabelNr[alt]);
      end;


    procedure TAsmData.getjumplabel(out l : TAsmLabel);
      begin
        l:=TAsmLabel.createlocal(AsmSymbolDict,FNextLabelNr[alt_jump],alt_jump);
        inc(FNextLabelNr[alt_jump]);
      end;

    procedure TAsmData.getglobaljumplabel(out l : TAsmLabel);
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        l:=TAsmLabel.createglobal(AsmSymbolDict,name^,FNextLabelNr[alt_jump],alt_jump);
        inc(FNextLabelNr[alt_jump]);
      end;


    procedure TAsmData.getglobaldatalabel(out l : TAsmLabel);
      begin
        l:=TAsmLabel.createglobal(AsmSymbolDict,name^,FNextLabelNr[alt_data],alt_data);
        inc(FNextLabelNr[alt_data]);
      end;


    procedure TAsmData.getstaticdatalabel(out l : TAsmLabel);
      begin
        l:=TAsmLabel.createstatic(AsmSymbolDict,FNextLabelNr[alt_data],alt_data);
        inc(FNextLabelNr[alt_data]);
      end;

=======
        l:=TAsmLabel.createglobal(AsmSymbolDict,name,FNextLabelNr[alt_jump],alt_jump);
        inc(FNextLabelNr[alt_jump]);
      end;
>>>>>>> graemeg/fixes_2_2
=======
        l:=TAsmLabel.createglobal(AsmSymbolDict,name,FNextLabelNr[alt_jump],alt_jump);
        inc(FNextLabelNr[alt_jump]);
      end;
>>>>>>> origin/fixes_2_2

    procedure TAsmData.getlocaldatalabel(out l: TAsmLabel);
      begin
        l:=TAsmLabel.createlocal(AsmSymbolDict,FNextLabelNr[alt_data],alt_data);
        inc(FNextLabelNr[alt_data]);
      end;


    procedure TAsmData.getaddrlabel(out l : TAsmLabel);
      begin
        l:=TAsmLabel.createlocal(AsmSymbolDict,FNextLabelNr[alt_addr],alt_addr);
        inc(FNextLabelNr[alt_addr]);
      end;

initialization
{$ifdef MEMDEBUG}
  memasmsymbols:=TMemDebug.create('AsmSymbols');
  memasmsymbols.stop;
  memasmcfi:=TMemDebug.create('AsmCFI');
  memasmcfi.stop;
  memasmlists:=TMemDebug.create('AsmLists');
  memasmlists.stop;
{$endif MEMDEBUG}
  CAsmCFI:=TAsmCFI;

finalization
{$ifdef MEMDEBUG}
  memasmsymbols.free;
  memasmcfi.free;
  memasmlists.free;
{$endif MEMDEBUG}

end.
