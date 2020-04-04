{
    Copyright (c) 20011 by Jonas Maebe

    LLVM version of some node tree helper routines

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
unit nllvmutil;

{$i fpcdefs.inc}

interface

  uses
    globtype,cclasses,
    aasmbase,aasmdata,aasmllvmmetadata, ngenutil,
    symtype,symconst,symsym,symdef;


  type
    tllvmnodeutils = class(tnodeutils)
     strict protected
      class procedure insertbsssym(list: tasmlist; sym: tstaticvarsym; size: asizeint; varalign: shortint; _typ:Tasmsymtype); override;
      class procedure InsertUsedList(var usedsyms: tfpobjectlist; const usedsymsname: TSymStr);
      class procedure InsertInitFiniList(var procdefs: tfplist; const initfinisymsname: TSymStr);
     public
      class procedure InsertObjectInfo; override;
      class procedure RegisterUsedAsmSym(sym: TAsmSymbol; def: tdef; compileronly: boolean); override;
      class procedure GenerateObjCImageInfo; override;
      class procedure RegisterModuleInitFunction(pd: tprocdef); override;
      class procedure RegisterModuleFiniFunction(pd: tprocdef); override;
    end;


implementation

    uses
      verbose,cutils,globals,fmodule,systems,
      aasmtai,cpubase,llvmbase,aasmllvm,
      aasmcnst,nllvmtcon,
      symbase,symtable,defutil,
      llvmtype,llvmdef,
      objcasm;

  class procedure tllvmnodeutils.insertbsssym(list: tasmlist; sym: tstaticvarsym; size: asizeint; varalign: shortint; _typ:Tasmsymtype);
    var
      asmsym: tasmsymbol;
      field1, field2: tsym;
    begin
      if sym.globalasmsym then
        asmsym:=current_asmdata.DefineAsmSymbol(sym.mangledname,AB_GLOBAL,_typ,sym.vardef)
      else if tf_supports_hidden_symbols in target_info.flags then
        asmsym:=current_asmdata.DefineAsmSymbol(sym.mangledname,AB_PRIVATE_EXTERN,_typ,sym.vardef)
      else
        asmsym:=current_asmdata.DefineAsmSymbol(sym.mangledname,AB_LOCAL,_typ,sym.vardef);
      if not(vo_is_thread_var in sym.varoptions) then
        list.concat(taillvmdecl.createdef(asmsym,sym.vardef,nil,sec_data,varalign))
      else if tf_section_threadvars in target_info.flags then
        list.concat(taillvmdecl.createtls(asmsym,sym.vardef,varalign))
      else
        list.concat(taillvmdecl.createdef(asmsym,
          get_threadvar_record(sym.vardef,field1,field2),
          nil,sec_data,varalign));
    end;


  type
    TTypedAsmSym = class
      sym: TAsmSymbol;
      def: tdef;
      constructor Create(s: TAsmSymbol; d: tdef);
    end;


  constructor TTypedAsmSym.Create(s: TAsmSymbol; d: tdef);
    begin
      sym:=s;
      def:=d;
    end;


  function TypedAsmSymComparer(p1, p2: Pointer): Integer;
    var
      sym1: TTypedAsmSym absolute p1;
      sym2: TTypedAsmSym absolute p2;
    begin
      result:=CompareStr(sym1.sym.Name,sym2.sym.Name);
    end;


  class procedure tllvmnodeutils.InsertUsedList(var usedsyms: tfpobjectlist; const usedsymsname: TSymstr);
    var
      useddef: tdef;
      tcb: ttai_typedconstbuilder;
      prevasmsym: TAsmSymbol;
      typedsym: TTypedAsmSym;
      uniquesyms, i: longint;
    begin
      if usedsyms.count<>0 then
        begin
          { a symbol can appear multiple times -> sort the list so we can filter out doubles }
          usedsyms.Sort(@TypedAsmSymComparer);
          { count uniques }
          prevasmsym:=nil;
          uniquesyms:=0;
          for i:=0 to usedsyms.count-1 do
            begin
              typedsym:=TTypedAsmSym(usedsyms[i]);
              if (prevasmsym<>typedsym.sym) and
                { even though we already filter on pure assembler routines when adding the symbols,
                  some may slip through because of forward definitions that are not yet resolved }
                 not((typedsym.def.typ=procdef) and
                     (po_assembler in tprocdef(typedsym.def).procoptions)) then
                inc(uniquesyms);
              prevasmsym:=typedsym.sym;
              end;
          { emit uniques }
          prevasmsym:=nil;
          tcb:=ctai_typedconstbuilder.create([tcalo_new_section]);
          tllvmtai_typedconstbuilder(tcb).appendingdef:=true;
          useddef:=carraydef.getreusable(voidpointertype,uniquesyms);
          tcb.maybe_begin_aggregate(useddef);
          for i:=0 to usedsyms.count-1 do
            begin
              typedsym:=TTypedAsmSym(usedsyms[i]);
              if (prevasmsym<>typedsym.sym) and
                 not((typedsym.def.typ=procdef) and
                     (po_assembler in tprocdef(typedsym.def).procoptions)) then
                begin
                  tcb.queue_init(voidpointertype);
                  tcb.queue_emit_asmsym(typedsym.sym,typedsym.def);
                  prevasmsym:=typedsym.sym;
                end;
            end;
          tcb.maybe_end_aggregate(useddef);
          current_asmdata.AsmLists[al_globals].concatlist(
            tcb.get_final_asmlist(
              current_asmdata.DefineAsmSymbol(
                usedsymsname,AB_GLOBAL,AT_DATA,useddef),useddef,sec_user,
                'llvm.metadata',0
            )
          );
          tcb.free;
        end;
      usedsyms.free;
      usedsyms:=nil;
    end;


  class procedure tllvmnodeutils.InsertInitFiniList(var procdefs: tfplist; const initfinisymsname: TSymStr);
    var
      itemdef: trecorddef;
      arraydef: tarraydef;
      pd: tprocdef;
      fields: array[0..2] of tdef;
      tcb: ttai_typedconstbuilder;
      i: longint;
    begin
      if procdefs.count<>0 then
        begin
          pd:=tprocdef(procdefs[0]);
          fields[0]:=s32inttype;
          fields[1]:=cprocvardef.getreusableprocaddr(pd,pc_address_only);
          fields[2]:=voidpointertype;
          itemdef:=llvmgettemprecorddef(fields,C_alignment,
            targetinfos[target_info.system]^.alignment.recordalignmin);
          include(itemdef.defoptions,df_llvm_no_struct_packing);
          tcb:=ctai_typedconstbuilder.create([tcalo_new_section]);
          tllvmtai_typedconstbuilder(tcb).appendingdef:=true;
          arraydef:=carraydef.getreusable(itemdef,procdefs.Count);
          tcb.maybe_begin_aggregate(arraydef);
          for i:=0 to procdefs.count-1 do
            begin
              tcb.maybe_begin_aggregate(itemdef);
              tcb.emit_ord_const(65535,s32inttype);
              tcb.emit_procdef_const(tprocdef(procdefs[i]));
              tcb.emit_tai(Tai_const.Create_sym(nil),voidpointertype);
              tcb.maybe_end_aggregate(itemdef);
            end;
          tcb.maybe_end_aggregate(arraydef);
          current_asmdata.AsmLists[al_globals].concatlist(
            tcb.get_final_asmlist(
              current_asmdata.DefineAsmSymbol(
                initfinisymsname,AB_GLOBAL,AT_DATA,arraydef),arraydef,sec_data,
                initfinisymsname,voidpointertype.alignment
            )
          );
          tcb.free;
        end;
    end;


  class procedure tllvmnodeutils.InsertObjectInfo;
    begin
      inherited;

      { insert newly created defs in the implementation rather than interface symtable
        (the interface symtable is sealed at this point) }
      symtablestack.push(current_module.localsymtable);

      { add the llvm.compiler.used array }
      InsertUsedList(current_module.llvmcompilerusedsyms,'llvm.compiler.used');
      { add the llvm.used array }
      InsertUsedList(current_module.llvmusedsyms,'llvm.used');
      { add the llvm.global_ctors array }
      InsertInitFiniList(current_module.llvminitprocs,'llvm.global_ctors');
      { add the llvm.global_dtors array }
      InsertInitFiniList(current_module.llvmfiniprocs,'llvm.global_dtors');

      { add "type xx = .." statements for all used recorddefs }
      with TLLVMTypeInfo.Create do
        begin
          inserttypeinfo;
          free;
        end;

      symtablestack.pop(current_module.localsymtable);
    end;


  class procedure tllvmnodeutils.RegisterUsedAsmSym(sym: TAsmSymbol; def: tdef; compileronly: boolean);
    var
      last: TTypedAsmSym;
    begin
      if compileronly then
        begin
          { filter multiple adds in succession here already }
          last:=TTypedAsmSym(current_module.llvmcompilerusedsyms.Last);
          if not assigned(last) or
             (last.sym<>sym) then
            current_module.llvmcompilerusedsyms.Add(TTypedAsmSym.Create(sym,def))
        end
      else
        begin
          last:=TTypedAsmSym(current_module.llvmusedsyms.Last);
          if not assigned(last) or
             (last.sym<>sym) then
          current_module.llvmusedsyms.Add(TTypedAsmSym.Create(sym,def))
        end;
    end;


  class procedure tllvmnodeutils.GenerateObjCImageInfo;
    var
      llvmmoduleflags,
       objcmoduleflag: tai_llvmbasemetadatanode;
      objcabiversion: longint;
    begin
      llvmmoduleflags:=tai_llvmnamedmetadatanode.create('llvm.module.flags');
      current_asmdata.AsmLists[al_rotypedconsts].Concat(llvmmoduleflags);

      { Objective-C ABI version }
      if not(target_info.system in [system_powerpc_darwin,system_powerpc64_darwin,system_i386_darwin,system_x86_64_darwin]) or
         (CompareVersionStrings(MacOSXVersionMin,'10.5')>=0) then
        objcabiversion:=2
      else
        objcabiversion:=1;
      objcmoduleflag:=tai_llvmunnamedmetadatanode.create;
      objcmoduleflag.addvalue(tai_simpletypedconst.create(s32inttype,tai_const.Create_32bit(1)));
      objcmoduleflag.addvalue(tai_simpletypedconst.create(charpointertype,tai_string.Create('Objective-C Version')));
      objcmoduleflag.addvalue(tai_simpletypedconst.create(s32inttype,tai_const.Create_32bit(objcabiversion)));
      llvmmoduleflags.addvalue(llvm_getmetadatareftypedconst(objcmoduleflag));
      current_asmdata.AsmLists[al_rotypedconsts].Concat(objcmoduleflag);

      { image info version }
      objcmoduleflag:=tai_llvmunnamedmetadatanode.create;
      objcmoduleflag.addvalue(tai_simpletypedconst.create(s32inttype,tai_const.Create_32bit(1)));
      objcmoduleflag.addvalue(tai_simpletypedconst.create(charpointertype,tai_string.Create('Objective-C Image Info Version')));
      objcmoduleflag.addvalue(tai_simpletypedconst.create(s32inttype,tai_const.Create_32bit(0)));
      llvmmoduleflags.addvalue(llvm_getmetadatareftypedconst(objcmoduleflag));
      current_asmdata.AsmLists[al_rotypedconsts].Concat(objcmoduleflag);

      { image info section }
      objcmoduleflag:=tai_llvmunnamedmetadatanode.create;
      objcmoduleflag.addvalue(tai_simpletypedconst.create(s32inttype,tai_const.Create_32bit(1)));
      objcmoduleflag.addvalue(tai_simpletypedconst.create(charpointertype,tai_string.Create('Objective-C Image Info Section')));
      objcmoduleflag.addvalue(tai_simpletypedconst.create(charpointertype,tai_string.Create(objc_section_name(sec_objc_image_info))));
      llvmmoduleflags.addvalue(llvm_getmetadatareftypedconst(objcmoduleflag));
      current_asmdata.AsmLists[al_rotypedconsts].Concat(objcmoduleflag);

      { garbage collection }
      objcmoduleflag:=tai_llvmunnamedmetadatanode.create;
      objcmoduleflag.addvalue(tai_simpletypedconst.create(s32inttype,tai_const.Create_32bit(1)));
      objcmoduleflag.addvalue(tai_simpletypedconst.create(charpointertype,tai_string.Create('Objective-C Garbage Collection')));
      objcmoduleflag.addvalue(tai_simpletypedconst.create(s32inttype,tai_const.Create_32bit(0)));
      llvmmoduleflags.addvalue(llvm_getmetadatareftypedconst(objcmoduleflag));
      current_asmdata.AsmLists[al_rotypedconsts].Concat(objcmoduleflag);
    end;


  class procedure tllvmnodeutils.RegisterModuleInitFunction(pd: tprocdef);
    begin
      current_module.llvminitprocs.add(pd);
    end;


  class procedure tllvmnodeutils.RegisterModuleFiniFunction(pd: tprocdef);
    begin
      current_module.llvmfiniprocs.add(pd);
    end;


begin
  cnodeutils:=tllvmnodeutils;
end.

