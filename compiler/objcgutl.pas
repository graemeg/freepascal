{
    Copyright (c) 2009 by Jonas Maebe

    This unit implements some Objective-C helper routines at the code generator
    level.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License,or
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

{$i fpcdefs.inc}

unit objcgutl;

interface

  uses
    cclasses,
    aasmbase,aasmdata,
    symbase,symdef;

  procedure objcfinishstringrefpoolentry(entry: phashsetitem; stringpool: tconstpooltype; refsec, stringsec: tasmsectiontype);
  procedure objcfinishclassrefnfpoolentry(entry: phashsetitem; classdef: tobjectdef);

  procedure MaybeGenerateObjectiveCImageInfo(globalst, localst: tsymtable);


implementation

  uses
    globtype,globals,fmodule,
    systems,
    aasmtai,
    cgbase,
    objcdef,objcutil,
    symconst,symtype,symsym,symtable,
    verbose;

  type
    tobjcabi = (oa_fragile, oa_nonfragile);
(*    tivarlayouttype = (il_weak,il_strong); *)

    tobjcrttiwriter = class
     protected
      fabi: tobjcabi;
      classdefs,
      catdefs: tfpobjectlist;
      classsyms,
      catsyms: tfpobjectlist;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      procedure gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmsymbol; classmethods, iscategory: Boolean);
=======
      procedure gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmlabel; classmethods, iscategory: Boolean);
>>>>>>> graemeg/cpstrnew
=======
      procedure gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmlabel; classmethods, iscategory: Boolean);
>>>>>>> graemeg/cpstrnew
=======
      procedure gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmlabel; classmethods, iscategory: Boolean);
>>>>>>> graemeg/cpstrnew
=======
      procedure gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmlabel; classmethods, iscategory: Boolean);
>>>>>>> origin/cpstrnew
      procedure gen_objc_protocol_elements(list: tasmlist; protocol: tobjectdef; out reqinstsym, optinstsym, reqclssym, optclssym: TAsmLabel);
      procedure gen_objc_protocol_list(list:TAsmList; protolist: TFPObjectList; out protolistsym: TAsmLabel);
      procedure gen_objc_cat_methods(list:TAsmList; items: TFPObjectList; section: tasmsectiontype;const sectname: string; out listsym: TAsmLabel);

      procedure gen_objc_protocol(list:TAsmList; protocol: tobjectdef; out protocollabel: TAsmSymbol);virtual;abstract;
      procedure gen_objc_category_sections(list:TAsmList; objccat: tobjectdef; out catlabel: TAsmSymbol);virtual;abstract;
      procedure gen_objc_classes_sections(list:TAsmList; objclss: tobjectdef; out classlabel: TAsmSymbol);virtual;abstract;
      procedure gen_objc_info_sections(list: tasmlist);virtual;abstract;
     public
      constructor create(_abi: tobjcabi);
      destructor destroy;override;
      procedure gen_objc_rtti_sections(list:TAsmList; st:TSymtable);
      property abi: tobjcabi read fabi;
    end;


    { Used by by PowerPC/32 and i386 }
    tobjcrttiwriter_fragile = class(tobjcrttiwriter)
     protected
      function gen_objc_protocol_ext(list: TAsmList; optinstsym, optclssym: TAsmLabel): TAsmLabel;
      procedure gen_objc_ivars(list: TAsmList; objccls: tobjectdef; out ivarslabel: TAsmLabel);
      procedure gen_objc_protocol(list:TAsmList; protocol: tobjectdef; out protocollabel: TAsmSymbol);override;
      procedure gen_objc_category_sections(list:TAsmList; objccat: tobjectdef; out catlabel: TAsmSymbol);override;
      procedure gen_objc_classes_sections(list:TAsmList; objclss: tobjectdef; out classlabel: TAsmSymbol);override;
      procedure gen_objc_info_sections(list: tasmlist);override;
     public
      constructor create;
    end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    { Used by PowerPC/64, ARM, x86_64 and AArch64 }
=======
    { Used by PowerPC/64, ARM, and x86_64 }
>>>>>>> graemeg/cpstrnew
=======
    { Used by PowerPC/64, ARM, and x86_64 }
>>>>>>> graemeg/cpstrnew
=======
    { Used by PowerPC/64, ARM, and x86_64 }
>>>>>>> graemeg/cpstrnew
=======
    { Used by PowerPC/64, ARM, and x86_64 }
>>>>>>> origin/cpstrnew
    tobjcrttiwriter_nonfragile = class(tobjcrttiwriter)
     protected
      ObjCEmptyCacheVar,
      ObjCEmptyVtableVar: TAsmSymbol;

      procedure gen_objc_class_ro_part(list: TAsmList; objclss: tobjectdef; protolistsym: TAsmSymbol; out classrolabel: TAsmSymbol; metaclass: boolean);
      procedure addclasslist(list: tasmlist; section: tasmsectiontype; const symname: string; classes: tfpobjectlist);

      procedure gen_objc_ivars(list: TAsmList; objccls: tobjectdef; out ivarslabel: TAsmLabel);
      procedure gen_objc_protocol(list:TAsmList; protocol: tobjectdef; out protocollabel: TAsmSymbol);override;
      procedure gen_objc_category_sections(list:TAsmList; objccat: tobjectdef; out catlabel: TAsmSymbol);override;
      procedure gen_objc_classes_sections(list:TAsmList; objclss: tobjectdef; out classlabel: TAsmSymbol);override;
      procedure gen_objc_info_sections(list: tasmlist);override;
     public
      constructor create;
    end;



{******************************************************************
                    Protocol declaration helpers
*******************************************************************}

function objcfindprotocolentry(const p: shortstring): TAsmSymbol;
  var
    item  : PHashSetItem;
  begin
    result:=nil;
    if not assigned(current_asmdata.ConstPools[sp_objcprotocolrefs]) then
      exit;
    item:=current_asmdata.constpools[sp_objcprotocolrefs].Find(@p[1], length(p));
    if not assigned(item) then
      exit;
    result:=TAsmSymbol(item^.Data);
  end;


function objcaddprotocolentry(const p: shortstring; ref: TAsmSymbol): Boolean;
  var
    item  : PHashSetItem;
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    if current_asmdata.ConstPools[sp_objcprotocolrefs]=nil then
      current_asmdata.ConstPools[sp_objcprotocolrefs]:=THashSet.Create(64, True, False);

>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[sp_objcprotocolrefs]=nil then
      current_asmdata.ConstPools[sp_objcprotocolrefs]:=THashSet.Create(64, True, False);

>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[sp_objcprotocolrefs]=nil then
      current_asmdata.ConstPools[sp_objcprotocolrefs]:=THashSet.Create(64, True, False);

>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[sp_objcprotocolrefs]=nil then
      current_asmdata.ConstPools[sp_objcprotocolrefs]:=THashSet.Create(64, True, False);

>>>>>>> origin/cpstrnew
    item:=current_asmdata.constpools[sp_objcprotocolrefs].FindOrAdd(@p[1], length(p));
    Result:=(item^.Data=nil);
    if Result then
      item^.Data:=ref;
  end;

{******************************************************************
                       Pool section helpers
*******************************************************************}

function objcreatestringpoolentryintern(p: pchar; len: longint; pooltype: tconstpooltype; stringsec: tasmsectiontype): TAsmSymbol;
  var
    entry  : PHashSetItem;
    strlab : tasmlabel;
    pc     : pchar;
    pool   : THashSet;
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    if current_asmdata.ConstPools[pooltype]=nil then
       current_asmdata.ConstPools[pooltype]:=THashSet.Create(64, True, False);
>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[pooltype]=nil then
       current_asmdata.ConstPools[pooltype]:=THashSet.Create(64, True, False);
>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[pooltype]=nil then
       current_asmdata.ConstPools[pooltype]:=THashSet.Create(64, True, False);
>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[pooltype]=nil then
       current_asmdata.ConstPools[pooltype]:=THashSet.Create(64, True, False);
>>>>>>> origin/cpstrnew
    pool := current_asmdata.constpools[pooltype];

    entry:=pool.FindOrAdd(p,len);
    if not assigned(entry^.data) then
      begin
        { create new entry }
        current_asmdata.getlabel(strlab,alt_data);
        entry^.Data:=strlab;
        getmem(pc,entry^.keylength+1);
        move(entry^.key^,pc^,entry^.keylength);
        pc[entry^.keylength]:=#0;

        { add the string to the approriate section }
        new_section(current_asmdata.asmlists[al_objc_pools],stringsec,strlab.name,0);
        current_asmdata.asmlists[al_objc_pools].concat(Tai_label.Create(strlab));
        current_asmdata.asmlists[al_objc_pools].concat(Tai_string.Create_pchar(pc,entry^.keylength+1));
        Result := strlab;
      end
    else
      Result := TAsmLabel(Entry^.Data);
  end;


procedure objcfinishstringrefpoolentry(entry: phashsetitem; stringpool: tconstpooltype; refsec, stringsec: tasmsectiontype);
  var
    reflab : tasmlabel;
    strlab : tasmsymbol;
    classname: string;
  begin
    { have we already generated a reference for this string entry? }
    if not assigned(entry^.Data) then
      begin
        { no, add the string to the associated strings section }
        strlab:=objcreatestringpoolentryintern(pchar(entry^.key),entry^.keylength,stringpool,stringsec);

        { and now finish the reference }
        current_asmdata.getlabel(reflab,alt_data);
        entry^.Data:=reflab;

        { add a pointer to the string in the string references section }
        new_section(current_asmdata.asmlists[al_objc_pools],refsec,reflab.name,sizeof(pint));
        current_asmdata.asmlists[al_objc_pools].concat(Tai_label.Create(reflab));
        current_asmdata.asmlists[al_objc_pools].concat(Tai_const.Create_sym(strlab));

        { in case of a class reference, also add a lazy symbol reference for
          the class (the linker requires this for the fragile ABI). }
        if (refsec=sec_objc_cls_refs) and
           not(target_info.system in systems_objc_nfabi) then
          begin
            setlength(classname,entry^.keylength);
            move(entry^.key^,classname[1],entry^.keylength);
            current_asmdata.asmlists[al_objc_pools].concat(tai_directive.Create(asd_lazy_reference,'.objc_class_name_'+classname));
          end;
      end;
  end;


function objcreatestringpoolentry(const s: string; pooltype: tconstpooltype; stringsec: tasmsectiontype): TAsmSymbol;
  begin
    result:=objcreatestringpoolentryintern(@s[1],length(s),pooltype,stringsec);
  end;


procedure objcfinishclassrefnfpoolentry(entry: phashsetitem; classdef: tobjectdef);
  var
    reflab: TAsmLabel;
    classym: TasmSymbol;
  begin
    { have we already generated a reference for this class ref entry? }
    if not assigned(entry^.Data) then
      begin
        { no, add the classref to the sec_objc_cls_refs section }
        current_asmdata.getlabel(reflab,alt_data);
        entry^.Data:=reflab;

        { add a pointer to the class }
        new_section(current_asmdata.asmlists[al_objc_pools],sec_objc_cls_refs,reflab.name,sizeof(pint));
        current_asmdata.asmlists[al_objc_pools].concat(Tai_label.Create(reflab));
        classym:=current_asmdata.RefAsmSymbol(classdef.rtti_mangledname(objcclassrtti));
        current_asmdata.asmlists[al_objc_pools].concat(Tai_const.Create_sym(classym));
      end;
  end;

{******************************************************************
                    RTTI generation -- Helpers
*******************************************************************}

procedure ConcatSymOrNil(list: tasmlist; sym: TAsmSymbol); inline;
begin
  if Assigned(sym) then
    list.Concat(tai_const.Create_sym(sym))
  else
    list.Concat(tai_const.Create_pint(0));
end;


{******************************************************************
                 RTTI generation -- Common
*******************************************************************}

{ generate a method list, either of class methods or of instance methods,
  and both for obj-c classes and categories. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure tobjcrttiwriter.gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmsymbol; classmethods, iscategory: Boolean);
=======
procedure tobjcrttiwriter.gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmlabel; classmethods, iscategory: Boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure tobjcrttiwriter.gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmlabel; classmethods, iscategory: Boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure tobjcrttiwriter.gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmlabel; classmethods, iscategory: Boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure tobjcrttiwriter.gen_objc_methods(list: tasmlist; objccls: tobjectdef; out methodslabel: tasmlabel; classmethods, iscategory: Boolean);
>>>>>>> origin/cpstrnew
  const
    clsSectType : array [Boolean] of tasmsectiontype = (sec_objc_inst_meth, sec_objc_cls_meth);
    clsSectName : array [Boolean] of string = ('_OBJC_INST_METH','_OBJC_CLS_METH');
    catSectType : array [Boolean] of tasmsectiontype = (sec_objc_cat_inst_meth, sec_objc_cat_cls_meth);
    catSectName : array [Boolean] of string = ('_OBJC_CAT_INST_METH','_OBJC_CAT_CLS_METH');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    instclsName : array [Boolean] of string = ('INSTANCE','CLASS');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  type
    method_data = record
      def     : tprocdef;
      selsym  : TAsmSymbol;
      encsym  : TAsmSymbol;
    end;
  var
    i     : Integer;
    def   : tprocdef;
    defs  : array of method_data;
    mcnt  : integer;
    sym   : tasmsymbol;
    mtype : tdef;
  begin
    methodslabel:=nil;
    mcnt:=0;
    { collect all instance/class methods }
    SetLength(defs,objccls.vmtentries.count);
    for i:=0 to objccls.vmtentries.count-1 do
      begin
        def:=pvmtentry(objccls.vmtentries[i])^.procdef;
        if (def.owner.defowner=objccls) and
           (classmethods = (po_classmethod in def.procoptions)) then
          begin
            defs[mcnt].def:=def;
            defs[mcnt].selsym:=objcreatestringpoolentry(def.messageinf.str^,sp_objcvarnames,sec_objc_meth_var_names);
            defs[mcnt].encsym:=objcreatestringpoolentry(objcencodemethod(def),sp_objcvartypes,sec_objc_meth_var_types);
            inc(mcnt);
          end;
      end;
    if mcnt=0 then
      exit;

    if iscategory then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      begin
        new_section(list,catSectType[classmethods],catSectName[classmethods],sizeof(ptrint));
        methodslabel:=current_asmdata.DefineAsmSymbol('l_OBJC_$_CATEGORY_'+instclsName[classmethods]+'_METHODS_'+objccls.objextname^+'_$_'+objccls.childof.objextname^,AB_LOCAL,AT_DATA);
      end
    else
      begin
        new_section(list,clsSectType[classmethods],clsSectName[classmethods],sizeof(ptrint));
        methodslabel:=current_asmdata.DefineAsmSymbol('l_OBJC_$_'+instclsName[classmethods]+'_METHODS_'+objccls.objextname^,AB_LOCAL,AT_DATA);
      end;

    list.Concat(tai_symbol.Create(methodslabel,0));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      new_section(list,catSectType[classmethods],catSectName[classmethods],sizeof(ptrint))
    else
      new_section(list,clsSectType[classmethods],clsSectName[classmethods],sizeof(ptrint));

    current_asmdata.getlabel(methodslabel,alt_data);
    list.Concat(tai_label.Create(methodslabel));
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

    if (abi=oa_fragile) then
      { not used, always zero }
      list.Concat(tai_const.Create_32bit(0))
    else
      begin
        { size of each entry -- always 32 bit value }
        mtype:=search_named_unit_globaltype('OBJC','OBJC_METHOD',true).typedef;
        list.Concat(tai_const.Create_32bit(mtype.size));
      end;
    { number of objc_method entries in the method_list array -- always 32 bit}
    list.Concat(tai_const.Create_32bit(mcnt));
    for i:=0 to mcnt-1 do
      begin
        { reference to the selector name }
        list.Concat(tai_const.Create_sym(defs[i].selsym));
        { reference to the obj-c encoded function parameters (signature) }
        list.Concat(tai_const.Create_sym(defs[i].encsym));
        { mangled name of the method }
        sym:=current_asmdata.GetAsmSymbol(defs[i].def.mangledname);
        if not assigned(sym) then
          internalerror(2009091601);
        list.Concat(tai_const.Create_sym(sym));
      end;
  end;


{ generate method (and in the future also property) info for protocols }
procedure tobjcrttiwriter.gen_objc_protocol_elements(list: tasmlist; protocol: tobjectdef; out reqinstsym, optinstsym, reqclssym, optclssym: TAsmLabel);
  var
    proc          : tprocdef;
    reqinstmlist,
    reqclsmlist,
    optinstmlist,
    optclsmlist   : TFPObjectList;
    i             : ptrint;
  begin
    reqinstmlist:=TFPObjectList.Create(false);
    reqclsmlist:=TFPObjectList.Create(false);
    optinstmlist:=TFPObjectList.Create(false);
    optclsmlist:=TFPObjectList.Create(false);
    for i:=0 to protocol.vmtentries.Count-1 do
      begin
        proc:=pvmtentry(protocol.vmtentries[i])^.procdef;
        if (po_classmethod in proc.procoptions) then
          if not(po_optional in proc.procoptions) then
            reqclsmlist.Add(proc)
          else
            optclsmlist.Add(proc)
        else if not(po_optional in proc.procoptions) then
          reqinstmlist.Add(proc)
        else
          optinstmlist.Add(proc);
      end;
    if reqinstmlist.Count > 0 then
      gen_objc_cat_methods(list,reqinstmlist,sec_objc_cat_inst_meth,'_OBJC_CAT_INST_METH',reqinstsym)
    else
      reqinstsym:=nil;
    if optinstmlist.Count > 0 then
      gen_objc_cat_methods(list,optinstmlist,sec_objc_cat_inst_meth,'_OBJC_CAT_INST_METH',optinstsym)
    else
      optinstsym:=nil;

    if reqclsmlist.Count>0 then
      gen_objc_cat_methods(list,reqclsmlist,sec_objc_cat_cls_meth,'_OBJC_CAT_CLS_METH',reqclssym)
    else
      reqclssym:=nil;
    if optclsmlist.Count>0 then
      gen_objc_cat_methods(list,optclsmlist,sec_objc_cat_cls_meth,'_OBJC_CAT_CLS_METH',optclssym)
    else
      optclssym:=nil;

    reqinstmlist.Free;
    reqclsmlist.Free;
    optinstmlist.Free;
    optclsmlist.Free;
end;


(*
From CLang:

  struct objc_protocol_list
  {
#ifdef FRAGILE_ABI
      struct objc_protocol_list *next;
      int count;
#else
      long count;
#endif
      Protocol *list[1];
  };
*)
procedure tobjcrttiwriter.gen_objc_protocol_list(list: tasmlist; protolist: tfpobjectlist; out protolistsym: tasmlabel);
  var
    i         : Integer;
    protosym  : TAsmSymbol;
    protodef  : tobjectdef;
  begin
    if not Assigned(protolist) or
       (protolist.Count=0) then
      begin
        protolistsym:=nil;
        Exit;
      end;

    for i:=0 to protolist.Count-1 do
      begin
        protodef:=TImplementedInterface(protolist[i]).IntfDef;
        protosym:=objcfindprotocolentry(protodef.objextname^);
        if not assigned(protosym) then
          begin
            gen_objc_protocol(list,protodef,protosym);
            objcaddprotocolentry(protodef.objextname^,protosym);
          end;
      end;

    { protocol lists are stored in .objc_cat_cls_meth section }
    new_section(list,sec_objc_cat_cls_meth,'_OBJC_PROTOCOLLIST',sizeof(pint));
    current_asmdata.getlabel(protolistsym, alt_data);
    list.Concat(tai_label.Create(protolistsym));

    if (abi=oa_fragile) then
      { From Clang: next, always nil}
      list.Concat(tai_const.Create_pint(0));
    { From Clang: protocols count}
    list.Concat(Tai_const.Create_pint(protolist.Count));
    for i:=0 to protolist.Count-1 do
      begin
        protodef:=(protolist[i] as TImplementedInterface).IntfDef;
        protosym:=objcfindprotocolentry(protodef.objextname^);
        if not Assigned(protosym) then
          begin
            { For some reason protosym is not declared, though must be!
              Probably gen_obcj1_protocol returned wrong protosym
            }
            InternalError(2009091602);
          end;
        list.Concat(tai_const.Create_sym(protosym));
      end;
  end;


{ Generate rtti for an Objective-C methods (methods without implementation) }
{ items : TFPObjectList of Tprocdef }
procedure tobjcrttiwriter.gen_objc_cat_methods(list:TAsmList; items: TFPObjectList; section: tasmsectiontype;
  const sectname: string; out listsym: TAsmLabel);
var
  i     : integer;
  m     : tprocdef;
  mtype : tdef;
begin
  if not assigned(items) or
     (items.count=0) then
    exit;

  new_section(list, section, sectname, sizeof(pint));
  current_asmdata.getlabel(listsym,alt_data);
  list.Concat(tai_label.Create(listsym));
  if (abi=oa_nonfragile) then
    begin
      { size of each entry -- always 32 bit value }
      mtype:=search_named_unit_globaltype('OBJC','OBJC_METHOD',true).typedef;
      list.Concat(tai_const.Create_32bit(mtype.size));
    end;
  list.Concat(Tai_const.Create_32bit(items.count));
  for i:=0 to items.Count-1 do
    begin
      m:=tprocdef(items[i]);
      list.Concat(Tai_const.Create_sym(
        objcreatestringpoolentry(m.messageinf.str^,sp_objcvarnames,sec_objc_meth_var_names)));
      list.Concat(Tai_const.Create_sym(
        objcreatestringpoolentry(objcencodemethod(m),sp_objcvartypes,sec_objc_meth_var_types)));
      { placeholder for address of implementation? }
      if (abi=oa_nonfragile) then
        list.Concat(Tai_const.Create_pint(0));
    end;
end;


{ Generate the rtti sections for all obj-c classes defined in st, and return
  these classes in the classes list. }
procedure tobjcrttiwriter.gen_objc_rtti_sections(list:TAsmList; st:TSymtable);
  var
    i: longint;
    def: tdef;
    sym : TAsmSymbol;
  begin
    if not Assigned(st) then
      exit;

    for i:=0 to st.DefList.Count-1 do
      begin
        def:=tdef(st.DefList[i]);
        { check whether all types used in Objective-C class/protocol/category
          declarations can be used with the Objective-C run time (can only be
          done now, because at parse-time some of these types can still be
          forwarddefs) }
        if is_objc_class_or_protocol(def) then
          if not tobjectdef(def).check_objc_types then
            continue;
        if is_objcclass(def) and
           not(oo_is_external in tobjectdef(def).objectoptions) then
          begin
            if not(oo_is_classhelper in tobjectdef(def).objectoptions) then
              begin
                gen_objc_classes_sections(list,tobjectdef(def),sym);
                classsyms.add(sym);
                classdefs.add(def);
              end
            else
              begin
                gen_objc_category_sections(list,tobjectdef(def),sym);
                catsyms.add(sym);
                catdefs.add(def);
              end
          end;
      end;
  end;


constructor tobjcrttiwriter.create(_abi: tobjcabi);
  begin
    fabi:=_abi;
    classdefs:=tfpobjectlist.create(false);
    classsyms:=tfpobjectlist.create(false);
    catdefs:=tfpobjectlist.create(false);
    catsyms:=tfpobjectlist.create(false);
  end;


destructor tobjcrttiwriter.destroy;
  begin
    classdefs.free;
    classsyms.free;
    catdefs.free;
    catsyms.free;
    inherited destroy;
  end;

{******************************************************************
                 RTTI generation -- Fragile ABI
*******************************************************************}

{ generate an instance variables list for an obj-c class. }
procedure tobjcrttiwriter_fragile.gen_objc_ivars(list: TAsmList; objccls: tobjectdef; out ivarslabel: TAsmLabel);
  type
    ivar_data = record
      vf      : tfieldvarsym;
      namesym : TAsmSymbol;
      typesym : TAsmSymbol;
    end;
  var
    i     : integer;
    vf    : tfieldvarsym;
    vars  : array of ivar_data;
    vcnt  : Integer;
    enctype : ansistring;
    encerr  : tdef;
  begin
    ivarslabel:=nil;

    vcnt:=0;
    setLength(vars,objccls.symtable.SymList.Count);

    for i:=0 to objccls.symtable.SymList.Count-1 do
      if tsym(objccls.symtable.SymList[i]).typ=fieldvarsym then
        begin
          vf:=tfieldvarsym(objccls.symtable.SymList[i]);
          if objctryencodetype(vf.vardef,enctype,encerr) then
            begin
              vars[vcnt].vf:=vf;
              vars[vcnt].namesym:=objcreatestringpoolentry(vf.RealName,sp_objcvarnames,sec_objc_meth_var_names);
              vars[vcnt].typesym:=objcreatestringpoolentry(enctype,sp_objcvartypes,sec_objc_meth_var_types);
              inc(vcnt);
            end
          else
            { Should be caught during parsing }
            internalerror(2009090601);
        end;
    if vcnt=0 then
      exit;

    new_section(list,sec_objc_instance_vars,'_OBJC_INSTANCE_VARS',sizeof(pint));

    current_asmdata.getlabel(ivarslabel,alt_data);
    list.Concat(tai_label.Create(ivarslabel));

    { objc_ivar_list: first the number of elements }
    list.Concat(tai_const.Create_32bit(vcnt));

    for i:=0 to vcnt-1 do
      begin
        { reference to the instance variable name }
        list.Concat(tai_const.Create_sym(vars[i].namesym));
        { reference to the encoded type }
        list.Concat(tai_const.Create_sym(vars[i].typesym));
        { and the offset of the field }
        list.Concat(tai_const.Create_32bit(vars[i].vf.fieldoffset));
      end;
  end;


(* From GCC:

  struct _objc_protocol_extension
    {
      uint32_t size;	// sizeof (struct _objc_protocol_extension)
      struct objc_method_list	*optional_instance_methods;
      struct objc_method_list   *optional_class_methods;
      struct objc_prop_list	*instance_properties;
    }
*)
function tobjcrttiwriter_fragile.gen_objc_protocol_ext(list: TAsmList; optinstsym, optclssym: TAsmLabel): TAsmLabel;
  begin
    if assigned(optinstsym) or
       assigned(optclssym) then
      begin
        new_section(list, sec_objc_protocol_ext,'_OBJC_PROTOCOLEXT',sizeof(pint));
        current_asmdata.getlabel(Result,alt_data);
        list.Concat(tai_label.Create(Result));
        { size of this structure }
        list.Concat(Tai_const.Create_32bit(16));
        { optional instance methods }
        ConcatSymOrNil(list,optinstsym);
        { optional class methods }
        ConcatSymOrNil(list,optclssym);
        { optional properties (todo) }
        ConcatSymOrNil(list,nil);
      end
    else
      Result:=nil;
  end;


{ Generate rtti for an Objective-C protocol  }
procedure tobjcrttiwriter_fragile.gen_objc_protocol(list:TAsmList; protocol: tobjectdef; out protocollabel: TAsmSymbol);
  var
    namesym     : TAsmSymbol;
    protolist   : TAsmLabel;
    reqinstsym,
    optinstsym,
    reqclssym,
    optclssym,
    protoext,
    lbl          : TAsmLabel;
  begin
    gen_objc_protocol_list(list,protocol.ImplementedInterfaces,protolist);
    gen_objc_protocol_elements(list,protocol,reqinstsym,optinstsym,reqclssym,optclssym);
    protoext:=gen_objc_protocol_ext(list,optinstsym,optclssym);

    new_section(list, sec_objc_protocol,'_OBJC_PROTOCOL',sizeof(pint));
    current_asmdata.getlabel(lbl,alt_data);
    list.Concat(tai_label.Create(lbl));
    protocollabel:=lbl;

    { protocol's isa - points to information about optional methods/properties }
    ConcatSymOrNil(list,protoext);
    { name }
    namesym:=objcreatestringpoolentry(protocol.objextname^,sp_objcclassnames,sec_objc_class_names);
    list.Concat(Tai_const.Create_sym(namesym));
    { protocol's list }
    ConcatSymOrNil(list,protolist);
    { instance methods, in __cat_inst_meth }
    ConcatSymOrNil(list,reqinstsym);
    { class methods, in __cat_cls_meth }
    ConcatSymOrNil(list,reqclssym);
  end;


(*
From Clang:

  struct _objc_category {
  char *category_name;
  char *class_name;
  struct _objc_method_list *instance_methods;
  struct _objc_method_list *class_methods;
  struct _objc_protocol_list *protocols;
  uint32_t size; // <rdar://4585769>
  struct _objc_property_list *instance_properties;
  };
*)

{ Generate rtti for an Objective-C class and its meta-class. }
procedure tobjcrttiwriter_fragile.gen_objc_category_sections(list:TAsmList; objccat: tobjectdef; out catlabel: TAsmSymbol);
  var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    protolistsym  : TAsmLabel;
    instmthdlist,
    clsmthdlist,
=======
    instmthdlist,
    clsmthdlist,
    protolistsym  : TAsmLabel;
>>>>>>> graemeg/cpstrnew
=======
    instmthdlist,
    clsmthdlist,
    protolistsym  : TAsmLabel;
>>>>>>> graemeg/cpstrnew
=======
    instmthdlist,
    clsmthdlist,
    protolistsym  : TAsmLabel;
>>>>>>> graemeg/cpstrnew
=======
    instmthdlist,
    clsmthdlist,
    protolistsym  : TAsmLabel;
>>>>>>> origin/cpstrnew
    catstrsym,
    clsstrsym,
    catsym        : TAsmSymbol;
  begin
    { the category name }
    catstrsym:=objcreatestringpoolentry(objccat.objextname^,sp_objcclassnames,sec_objc_class_names);

    { the name of the class it extends }
    clsstrsym:=objcreatestringpoolentry(objccat.childof.objextname^,sp_objcclassnames,sec_objc_class_names);

    { generate the methods lists }
    gen_objc_methods(list,objccat,instmthdlist,false,true);
    gen_objc_methods(list,objccat,clsmthdlist,true,true);

    { generate implemented protocols list }
    gen_objc_protocol_list(list,objccat.ImplementedInterfaces,protolistsym);

    { category declaration section }
    new_section(list,sec_objc_category,'_OBJC_CATEGORY',sizeof(pint));
    catsym:=current_asmdata.DefineAsmSymbol(objccat.rtti_mangledname(objcclassrtti),AB_LOCAL,AT_DATA);
    list.Concat(tai_symbol.Create(catsym,0));

    list.Concat(Tai_const.Create_sym(catstrsym));
    list.Concat(Tai_const.Create_sym(clsstrsym));
    ConcatSymOrNil(list,instmthdlist);
    ConcatSymOrNil(list,clsmthdlist);
    ConcatSymOrNil(list,protolistsym);
    { size of this structure }
    list.Concat(Tai_const.Create_32bit(28));
    { properties, not yet supported }
    list.Concat(Tai_const.Create_32bit(0));

    catlabel:=catsym;
  end;

(*
From Clang:

  struct _objc_class {
    Class isa;
    Class super_class;
    const char *name;
    long version;
    long info;
    long instance_size;
    struct _objc_ivar_list *ivars;
    struct _objc_method_list *methods;
    struct _objc_cache *cache;
    struct _objc_protocol_list *protocols;
    // Objective-C 1.0 extensions (<rdr://4585769>) -- for garbage collection
    const char *ivar_layout;
    struct _objc_class_ext *ext;
  };
*)



{ Generate rtti for an Objective-C class and its meta-class. }
procedure tobjcrttiwriter_fragile.gen_objc_classes_sections(list:TAsmList; objclss: tobjectdef; out classlabel: TAsmSymbol);
  const
    CLS_CLASS  = 1;
    CLS_META   = 2;
    CLS_HIDDEN = $20000;
    META_INST_SIZE = 40+8; // sizeof(objc_class) + 8
  var
    root          : tobjectdef;
    superStrSym,
    classStrSym,
    metaisaStrSym,
    metasym,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    mthdlist,
    clssym        : TAsmSymbol;
=======
    clssym        : TAsmSymbol;
    mthdlist,
>>>>>>> graemeg/cpstrnew
=======
    clssym        : TAsmSymbol;
    mthdlist,
>>>>>>> graemeg/cpstrnew
=======
    clssym        : TAsmSymbol;
    mthdlist,
>>>>>>> graemeg/cpstrnew
=======
    clssym        : TAsmSymbol;
    mthdlist,
>>>>>>> origin/cpstrnew
    ivarslist,
    protolistsym  : TAsmLabel;
    hiddenflag    : cardinal;
  begin
    { generate the class methods list }
    gen_objc_methods(list,objclss,mthdlist,true,false);

    { generate implemented protocols list }
    gen_objc_protocol_list(list,objclss.ImplementedInterfaces,protolistsym);

    { register necessary names }
    { 1) the superclass }
    if assigned(objclss.childof) then
      superStrSym:=objcreatestringpoolentry(objclss.childof.objextname^,sp_objcclassnames,sec_objc_class_names)
    else
      { not empty string, but nil! }
      superStrSym:=nil;

    { 2) the current class }
    classStrSym:=objcreatestringpoolentry(objclss.objextname^,sp_objcclassnames,sec_objc_class_names);
    { 3) the isa }
    { From Clang: The isa for the meta-class is the root of the hierarchy. }
    root:=objclss;
    while assigned(root.childof) do
      root:=root.childof;
    metaisaStrSym:=objcreatestringpoolentry(root.objextname^,sp_objcclassnames,sec_objc_class_names);

    { 4) the flags }
    { consider every class declared in the implementation section of a unit
      as "hidden"
    }
    hiddenflag:=0;
    if (objclss.owner.symtabletype=staticsymtable) and
       current_module.is_unit then
      hiddenflag:=CLS_HIDDEN;

    { class declaration section }
    new_section(list,sec_objc_meta_class,'_OBJC_META_CLASS',sizeof(pint));

    { 1) meta-class declaration  }
    metasym:=current_asmdata.DefineAsmSymbol(objclss.rtti_mangledname(objcmetartti),AB_LOCAL,AT_DATA);
    list.Concat(tai_symbol.Create(metasym,0));

    list.Concat(Tai_const.Create_sym(metaisaStrSym));
    { pointer to the superclass name if any, otherwise nil }
    if assigned(superstrsym) then
      list.Concat(Tai_const.Create_sym(superStrSym))
    else
      list.concat(tai_const.create_32bit(0));
    { pointer to the class name }
    list.Concat(Tai_const.Create_sym(classStrSym));

    { version is always 0 currently }
    list.Concat(Tai_const.Create_32bit(0));
    { CLS_META for meta-classes }
    list.Concat(Tai_const.Create_32bit(hiddenflag or CLS_META));
    { size of the meta-class instance: sizeof(objc_class) + 8 bytes }
    list.Concat(Tai_const.Create_32bit(META_INST_SIZE) );
    { meta-classes don't have ivars list (=0) }
    list.Concat(Tai_const.Create_32bit(0));
    { class methods list (stored in "__cls_meth" section) }
    if Assigned(mthdlist) then
      list.Concat(Tai_const.Create_sym(mthdlist))
    else
      list.Concat(Tai_const.Create_32bit(0));
    { From Clang: cache is always nil }
    list.Concat(Tai_const.Create_32bit(0));
    { protocols }
    ConcatSymOrNil(list, protolistsym);
    { From Clang: ivar_layout for meta-class is always NULL. }
    list.Concat(Tai_const.Create_32bit(0));
    { From Clang: The class extension is always unused for meta-classes. }
    list.Concat(Tai_const.Create_32bit(0));

    { 2) regular class declaration }

    { generate the instance methods list }
    gen_objc_methods(list,objclss,mthdlist,false,false);
    { generate the instance variables list }
    gen_objc_ivars(list,objclss,ivarslist);

    new_section(list,sec_objc_class,'_OBJC_CLASS',sizeof(pint));

    clssym:=current_asmdata.DefineAsmSymbol(objclss.rtti_mangledname(objcclassrtti),AB_LOCAL,AT_DATA);
    list.Concat(tai_symbol.Create(clssym,0));

    { for class declaration: the isa points to the meta-class declaration }
    list.Concat(Tai_const.Create_sym(metasym));
    { pointer to the super_class name if any, nil otherwise }
    if assigned(superStrSym) then
      list.Concat(Tai_const.Create_sym(superStrSym))
    else
      list.Concat(Tai_const.Create_32bit(0));
    { pointer to the class name }
    list.Concat(Tai_const.Create_sym(classStrSym));
    { version is always 0 currently }
    list.Concat(Tai_const.Create_32bit(0));
    { CLS_CLASS for classes }
    list.Concat(Tai_const.Create_32bit(hiddenflag or CLS_CLASS));
    { size of instance: total size of instance variables }
    list.Concat(Tai_const.Create_32bit(tobjectsymtable(objclss.symtable).datasize));
    { objc_ivar_list (stored in "__instance_vars" section) }
    if assigned(ivarslist) then
      list.Concat(Tai_const.Create_sym(ivarslist))
    else
      list.Concat(tai_const.create_32bit(0));
    { instance methods list (stored in "__inst_meth" section) }
    if Assigned(mthdlist) then
      list.Concat(Tai_const.Create_sym(mthdlist))
    else
      list.Concat(Tai_const.Create_32bit(0));
    { From Clang: cache is always NULL }
    list.Concat(Tai_const.Create_32bit(0));
    { protocols, protolistsym has been created for meta-class, no need to create another one}
    ConcatSymOrNil(list, protolistsym);
    { TODO: From Clang: strong ivar_layout, necessary for garbage collection support }
    list.Concat(Tai_const.Create_32bit(0));
    { TODO: From Clang: weak ivar_layout, necessary for garbage collection support }
    list.Concat(Tai_const.Create_32bit(0));

    classlabel:=clssym;
  end;


{ Generate the global information sections (objc_symbols and objc_module_info)
  for this module. }
procedure tobjcrttiwriter_fragile.gen_objc_info_sections(list: tasmlist);
  var
    i: longint;
    sym : TAsmSymbol;
    parent: tobjectdef;
    superclasses: tfpobjectlist;
  begin
    if (classsyms.count<>0) or
       (catsyms.count<>0) then
      begin
        new_section(list,sec_objc_symbols,'_OBJC_SYMBOLS',sizeof(pint));
        sym := current_asmdata.DefineAsmSymbol(target_asm.labelprefix+'_OBJC_SYMBOLS_$',AB_LOCAL,AT_DATA);

        { symbol to refer to this information }
        list.Concat(tai_symbol.Create(sym,0));
        { ??? (always 0 in Clang) }
        list.Concat(Tai_const.Create_pint(0));
        { ??? (From Clang: always 0, pointer to some selector) }
        list.Concat(Tai_const.Create_pint(0));
        { From Clang: number of defined classes }
        list.Concat(Tai_const.Create_16bit(classsyms.count));
        { From Clang: number of defined categories }
        list.Concat(Tai_const.Create_16bit(catsyms.count));
        { first all classes }
        for i:=0 to classsyms.count-1 do
          list.Concat(Tai_const.Create_sym(tasmsymbol(classsyms[i])));
        { then all categories }
        for i:=0 to catsyms.count-1 do
          list.Concat(Tai_const.Create_sym(tasmsymbol(catsyms[i])));
     end
    else
      sym:=nil;

    new_section(list,sec_objc_module_info,'_OBJC_MODULE_INFO',4);
    { version number = 7 (always, both for gcc and clang) }
    list.Concat(Tai_const.Create_pint(7));
    { sizeof(objc_module): 4 pointer-size entities }
    list.Concat(Tai_const.Create_pint(sizeof(pint)*4));
    { used to be file name, now unused (points to empty string) }
    list.Concat(Tai_const.Create_sym(objcreatestringpoolentry('',sp_objcclassnames,sec_objc_class_names)));
    { pointer to classes/categories list declared in this module }
    if assigned(sym) then
      list.Concat(Tai_const.Create_sym(sym))
    else
      list.concat(tai_const.create_pint(0));

    { Add lazy references to parent classes of all classes defined in this unit }
    superclasses:=tfpobjectlist.create(false);
    for i:=0 to classdefs.count-1 do
      begin
        parent:=tobjectdef(classdefs[i]).childof;
        { warning: linear search, performance hazard if large number of subclasses }
        if assigned(parent) and
           (superclasses.indexof(parent)=-1) then
          begin
            list.concat(tai_directive.create(asd_lazy_reference,'.objc_class_name_'+parent.objextname^));
            superclasses.add(parent);
          end;
      end;
    for i:=0 to catdefs.count-1 do
      begin
        parent:=tobjectdef(catdefs[i]).childof;
        { warning: linear search, performance hazard if large number of subclasses }
        if assigned(parent) and
           (superclasses.indexof(parent)=-1) then
          begin
            list.concat(tai_directive.create(asd_lazy_reference,'.objc_class_name_'+parent.objextname^));
            superclasses.add(parent);
          end;
      end;
    superclasses.free;
    { reference symbols for all classes and categories defined in this unit }
    for i:=0 to classdefs.count-1 do
      list.concat(tai_symbol.Createname_global_value('.objc_class_name_'+tobjectdef(classdefs[i]).objextname^,AT_DATA,0,0));
    for i:=0 to catdefs.count-1 do
      list.concat(tai_symbol.Createname_global_value('.objc_category_name_'+
        tobjectdef(catdefs[i]).childof.objextname^+'_'+
        tobjectdef(catdefs[i]).objextname^,AT_DATA,0,0));
  end;


constructor tobjcrttiwriter_fragile.create;
  begin
    inherited create(oa_fragile);
  end;


{******************************************************************
                 RTTI generation -- Non-Fragile ABI
*******************************************************************}

(*
From Clang:
/// EmitIvarList - Emit the ivar list for the given
/// implementation. The return value has type
/// IvarListnfABIPtrTy.
///  struct _ivar_t {
///   unsigned long int *offset;  // pointer to ivar offset location
///   char *name;
///   char *type;
///   uint32_t alignment;
///   uint32_t size;
/// }
/// struct _ivar_list_t {
///   uint32 entsize;  // sizeof(struct _ivar_t)
///   uint32 count;
///   struct _iver_t list[count];
/// }
///
*)
procedure tobjcrttiwriter_nonfragile.gen_objc_ivars(list: tasmlist; objccls: tobjectdef; out ivarslabel: tasmlabel);
  type
    ivar_data = record
      vf      : tfieldvarsym;
      namesym : TAsmSymbol;
      typesym : TAsmSymbol;
      offssym : TAsmSymbol;
    end;
  var
    ivtype: tdef;
    vf    : tfieldvarsym;
    vars  : array of ivar_data;
    i     : integer;
    vcnt  : integer;
    enctype : ansistring;
    encerr  : tdef;
    prefix  : shortstring;
    vis     : TAsmsymbind;
  begin
    ivarslabel:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    prefix:='';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    vcnt:=0;
    setLength(vars,objccls.symtable.SymList.Count);

    for i:=0 to objccls.symtable.SymList.Count-1 do
      if tsym(objccls.symtable.SymList[i]).typ=fieldvarsym then
        begin
          vf:=tfieldvarsym(objccls.symtable.SymList[i]);
          if objctryencodetype(vf.vardef,enctype,encerr) then
            begin
              vars[vcnt].vf:=vf;
              vars[vcnt].namesym:=objcreatestringpoolentry(vf.RealName,sp_objcvarnames,sec_objc_meth_var_names);
              vars[vcnt].typesym:=objcreatestringpoolentry(enctype,sp_objcvartypes,sec_objc_meth_var_types);
              if (vcnt=0) then
                begin
                  new_section(list,sec_objc_const,'_OBJC_IVAR_OFFSETS',sizeof(pint));
                  prefix:=target_info.cprefix+'OBJC_IVAR_$_'+objccls.objextname^+'.';
                end;
              { This matches gcc/Clang, but is strange: I would expect private
                fields to be local symbols rather than private_extern (which
                is "package-global") (JM)
              }
              if not(vf.visibility in [vis_public,vis_protected,vis_strictprotected]) then
                vis:=AB_PRIVATE_EXTERN
              else
                vis:=AB_GLOBAL;
              vars[vcnt].offssym:=current_asmdata.DefineAsmSymbol(prefix+vf.RealName,vis,AT_DATA);
              list.concat(tai_symbol.Create_Global(vars[vcnt].offssym,0));
              list.concat(tai_const.create_pint(vf.fieldoffset));
              inc(vcnt);
            end
          else
            { must be caught during parsing }
            internalerror(2009092301);
        end;
    if vcnt=0 then
      exit;

    new_section(list,sec_objc_instance_vars,'_OBJC_INSTANCE_VARS',sizeof(pint));

    current_asmdata.getlabel(ivarslabel,alt_data);
    list.Concat(tai_label.Create(ivarslabel));

    { size of each entry -- always 32 bit value }
    ivtype:=search_named_unit_globaltype('OBJC','OBJC_IVAR',true).typedef;
    list.concat(tai_const.Create_32bit(ivtype.size));
    { number of entries -- always 32 bit value }
    list.Concat(tai_const.Create_32bit(vcnt));

    for i:=0 to vcnt-1 do
      begin
        { reference to the offset }
        list.Concat(tai_const.Create_sym(vars[i].offssym));
        { reference to the instance variable name }
        list.Concat(tai_const.Create_sym(vars[i].namesym));
        { reference to the encoded type }
        list.Concat(tai_const.Create_sym(vars[i].typesym));
        { alignment -- always 32 bit value }
        list.Concat(tai_const.create_32bit(vars[i].vf.vardef.alignment));
        { size -- always 32 bit value }
        list.Concat(tai_const.Create_32bit(vars[i].vf.vardef.size));
      end;
  end;


(*
From Clang:
/// GetOrEmitProtocol - Generate the protocol meta-data:
/// @code
/// struct _protocol_t {
///   id isa;  // NULL
///   const char * const protocol_name;
///   const struct _protocol_list_t * protocol_list; // super protocols
///   const struct method_list_t * const instance_methods;
///   const struct method_list_t * const class_methods;
///   const struct method_list_t *optionalInstanceMethods;
///   const struct method_list_t *optionalClassMethods;
///   const struct _prop_list_t * properties;
///   const uint32_t size;  // sizeof(struct _protocol_t)
///   const uint32_t flags;  // = 0
/// }
/// @endcode
*)
procedure tobjcrttiwriter_nonfragile.gen_objc_protocol(list: tasmlist; protocol: tobjectdef; out protocollabel: tasmsymbol);
  var
    lbl,
    namesym,
    listsym       : TAsmSymbol;
    protolist     : TAsmLabel;
    reqinstsym,
    reqclssym,
    optinstsym,
    optclssym     : TAsmLabel;
    prottype      : tdef;
  begin
    gen_objc_protocol_list(list,protocol.ImplementedInterfaces,protolist);
    gen_objc_protocol_elements(list,protocol,reqinstsym,optinstsym,reqclssym,optclssym);

    new_section(list, sec_data_coalesced,'_OBJC_PROTOCOL',sizeof(pint));
    { label for the protocol needs to be
        a) in a coalesced section (so multiple definitions of the same protocol
           can be merged by the linker)
        b) private_extern (should only be merged within the same module)
        c) weakly defined (so multiple definitions don't cause errors)
    }
    lbl:=current_asmdata.DefineAsmSymbol(protocol.rtti_mangledname(objcclassrtti),AB_PRIVATE_EXTERN,AT_DATA);
    list.Concat(tai_symbol.Create_Global(lbl,0));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    list.Concat(tai_directive.Create(asd_weak_definition,lbl.name));
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    protocollabel:=lbl;

    { protocol's isa - always nil }
    list.Concat(Tai_const.Create_pint(0));
    { name }
    namesym:=objcreatestringpoolentry(protocol.objextname^,sp_objcclassnames,sec_objc_class_names);
    list.Concat(Tai_const.Create_sym(namesym));
    { parent protocols list }
    ConcatSymOrNil(list,protolist);
    { required instance methods }
    ConcatSymOrNil(list,reqinstsym);
    { required class methods }
    ConcatSymOrNil(list,reqclssym);
    { optional instance methods }
    ConcatSymOrNil(list,optinstsym);
    { optional class methods }
    ConcatSymOrNil(list,optclssym);
    { TODO: properties }
    list.Concat(Tai_const.Create_pint(0));
    { size of this type }
    prottype:=search_named_unit_globaltype('OBJC','OBJC_PROTOCOL',true).typedef;
    list.concat(tai_const.Create_32bit(prottype.size));
    { flags }
    list.concat(tai_const.Create_32bit(0));

    { also add an entry to the __DATA, __objc_protolist section, required to
      register the protocol with the runtime }
    new_section(list, sec_objc_protolist,'_OBJC_PROTOLIST',sizeof(pint));
    listsym:=current_asmdata.DefineAsmSymbol(protocol.rtti_mangledname(objcmetartti),AB_PRIVATE_EXTERN,AT_DATA);
    list.Concat(tai_symbol.Create_Global(listsym,0));
    list.Concat(tai_const.Create_sym(lbl));
    list.Concat(tai_directive.Create(asd_weak_definition,listsym.name));
  end;

(*
From Clang:
/// struct _category_t {
///   const char * const name;
///   struct _class_t *const cls;
///   const struct _method_list_t * const instance_methods;
///   const struct _method_list_t * const class_methods;
///   const struct _protocol_list_t * const protocols;
///   const struct _prop_list_t * const properties;
/// }
*)
procedure tobjcrttiwriter_nonfragile.gen_objc_category_sections(list:TAsmList; objccat: tobjectdef; out catlabel: TAsmSymbol);
  var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    protolistsym  : TAsmLabel;
    instmthdlist,
    clsmthdlist,
=======
    instmthdlist,
    clsmthdlist,
    protolistsym  : TAsmLabel;
>>>>>>> graemeg/cpstrnew
=======
    instmthdlist,
    clsmthdlist,
    protolistsym  : TAsmLabel;
>>>>>>> graemeg/cpstrnew
=======
    instmthdlist,
    clsmthdlist,
    protolistsym  : TAsmLabel;
>>>>>>> graemeg/cpstrnew
=======
    instmthdlist,
    clsmthdlist,
    protolistsym  : TAsmLabel;
>>>>>>> origin/cpstrnew
    catstrsym,
    clssym,
    catsym        : TAsmSymbol;
  begin
    { the category name }
    catstrsym:=objcreatestringpoolentry(objccat.objextname^,sp_objcclassnames,sec_objc_class_names);

    { the class it extends }
    clssym:=current_asmdata.RefAsmSymbol(objccat.childof.rtti_mangledname(objcclassrtti));

    { generate the methods lists }
    gen_objc_methods(list,objccat,instmthdlist,false,true);
    gen_objc_methods(list,objccat,clsmthdlist,true,true);

    { generate implemented protocols list }
    gen_objc_protocol_list(list,objccat.ImplementedInterfaces,protolistsym);

    { category declaration section }
    new_section(list,sec_objc_const,'_OBJC_CATEGORY',sizeof(pint));
    catsym:=current_asmdata.DefineAsmSymbol(objccat.rtti_mangledname(objcclassrtti),AB_LOCAL,AT_DATA);
    list.Concat(tai_symbol.Create(catsym,0));

    list.Concat(Tai_const.Create_sym(catstrsym));
    list.Concat(Tai_const.Create_sym(clssym));
    ConcatSymOrNil(list,instmthdlist);
    ConcatSymOrNil(list,clsmthdlist);
    ConcatSymOrNil(list,protolistsym);
    { properties, not yet supported }
    list.Concat(Tai_const.Create_pint(0));

    catlabel:=catsym;
  end;


(*
From Clang:
/// BuildIvarLayout - Builds ivar layout bitmap for the class
/// implementation for the __strong or __weak case.
/// The layout map displays which words in ivar list must be skipped
/// and which must be scanned by GC (see below). String is built of bytes.
/// Each byte is divided up in two nibbles (4-bit each). Left nibble is count
/// of words to skip and right nibble is count of words to scan. So, each
/// nibble represents up to 15 workds to skip or scan. Skipping the rest is
/// represented by a 0x00 byte which also ends the string.
/// 1. when ForStrongLayout is true, following ivars are scanned:
/// - id, Class
/// - object *  // note: this "object" means "Objective-C object" (JM)
/// - __strong anything
///
/// 2. When ForStrongLayout is false, following ivars are scanned:
/// - __weak anything
*)
(*

Only required when supporting garbage collection

procedure tobjcrttiwriter_nonfragile.gen_objc_ivargc_recursive(st: tabstractrecordsymtable; ptrbset: tbitset; startoffset: puint; il: tivarlayouttype);
var
  i: longint;
  fs: tfieldvarsym;
  includelen: longint;
begin
  for i:=0 to st.SymList.count-1 do
    if (tsym(st.symlist[i]).typ=fieldvarsym) then
      begin
        fs:=tfieldvarsym(st.symlist[i]);
        includelen:=0;
        case fs.vardef.typ of
          pointerdef,
          classrefdef:
            if (fs.vardef=objc_idtype) or
               (fs.vardef=objc_metaclasstype) then
              includelen:=1;
          recorddef:
            TODO: bitpacking -> offset differences
            gen_objc_ivargc_recursive(tabstractrecordsymtable(trecorddef(fs.vardef).symtable),ptrbset,startoffset+fs.fieldoffset,il);
          arraydef:
            begin
              if not is_special_
            end;
          objectdef :
            begin
              case tobjectdef(fs.vardef).objecttype of
                odt_objcclass,
                odt_objcprotocol:
                  includelen:=1;
                odt_object:
                  gen_objc_ivargc_recursive(tabstractrecordsymtable(tobjectdef(fs.vardef).symtable),ptrbset,startoffset+fs.fieldoffset,il);
              end;
            end;
        end;
      end;
end;


function tobjcrttiwriter_nonfragile.gen_objc_ivargcstring(objclss: tobjectdef; il: tivarlayouttype): ansistring;
  var
    ptrbset: tbitset;
    parent: tobjectdef;
    size,
    startoffset: puint;
    i: longint;
  begin
    size:=tObjectSymtable(objclss.symtable).datasize;
    if assigned(objclss.childof) then
      startoffset:=tObjectSymtable(objclss.childof.symtable).datasize
    else
      startoffset:=0;
    size:=size-startoffset;
    ptrbset:=tbitset.create_bytesize((size+sizeof(ptruint)-1) div sizeof(ptruint));
    { has to include info for this class' fields and those of all parent
      classes as well
    }
    parent:=obclss;
    repeat
      gen_objc_ivargc_recursive(parent.symtable,ptrbset,0,il);
      parent:=parent.childof;
    until not assigned(parent);
    { convert bits set to encoded string }
  end;
*)

(*
From Clang:
/// struct _class_ro_t {
///   uint32_t const flags;
///   uint32_t const instanceStart;
///   uint32_t const instanceSize;
///   uint32_t const reserved;  // only when building for 64bit targets
///   const uint8_t * const ivarLayout;
///   const char *const name;
///   const struct _method_list_t * const baseMethods;
///   const struct _protocol_list_t *const baseProtocols;
///   const struct _ivar_list_t *const ivars;
///   const uint8_t * const weakIvarLayout;
///   const struct _prop_list_t * const properties;
/// }
*)

procedure tobjcrttiwriter_nonfragile.gen_objc_class_ro_part(list: tasmlist; objclss: tobjectdef; protolistsym: TAsmSymbol; out classrolabel: tasmsymbol; metaclass: boolean);
  const
    CLS_CLASS        = 0;
    CLS_META         = 1;
    CLS_ROOT         = 2;
    OBJC2_CLS_HIDDEN = $10;
    CLS_EXCEPTION    = $20;
  var
    classStrSym,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    methodssym,
    rosym        : TAsmSymbol;
=======
    rosym        : TAsmSymbol;
    methodslab,
>>>>>>> graemeg/cpstrnew
=======
    rosym        : TAsmSymbol;
    methodslab,
>>>>>>> graemeg/cpstrnew
=======
    rosym        : TAsmSymbol;
    methodslab,
>>>>>>> graemeg/cpstrnew
=======
    rosym        : TAsmSymbol;
    methodslab,
>>>>>>> origin/cpstrnew
    ivarslab     : TAsmLabel;
    class_type   : tdef;
    start,
    size,
    flags        : cardinal;
    rttitype     : trttitype;
    firstfield   : tfieldvarsym;
    i            : longint;
  begin
    { consider every class declared in the implementation section of a unit
      as "hidden"
    }
    flags:=0;
    if (objclss.owner.symtabletype=staticsymtable) and
       current_module.is_unit then
      flags:=OBJC2_CLS_HIDDEN;
    if metaclass then
      begin
        flags:=flags or CLS_META;
        rttitype:=objcmetarortti;
        { metaclass size/start: always size of objc_object }
        class_type:=search_named_unit_globaltype('OBJC','OBJC_OBJECT',true).typedef;
        start:=class_type.size;
        size:=start;
      end
    else
      begin
        flags:=flags or CLS_CLASS;
        rttitype:=objcclassrortti;
        size:=tObjectSymtable(objclss.symtable).datasize;
        { can't simply use childof's datasize, because alignment may cause the
          first field to skip a couple of bytes after the previous end }
        firstfield:=nil;
        for i:=0 to objclss.symtable.SymList.Count-1 do
          if (tsym(objclss.symtable.SymList[i]).typ=fieldvarsym) then
            begin
              firstfield:=tfieldvarsym(objclss.symtable.SymList[i]);
              break;
            end;
        if assigned(firstfield) then
          start:=firstfield.fieldoffset
        else
          { no extra fields -> start = size }
          start:=size;
      end;
    if not assigned(objclss.childof) then
      flags:=flags or CLS_ROOT;

    classStrSym:=objcreatestringpoolentry(objclss.objextname^,sp_objcclassnames,sec_objc_class_names);
    { generate methods list }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    gen_objc_methods(list,objclss,methodssym,metaclass,false);
=======
    gen_objc_methods(list,objclss,methodslab,metaclass,false);
>>>>>>> graemeg/cpstrnew
=======
    gen_objc_methods(list,objclss,methodslab,metaclass,false);
>>>>>>> graemeg/cpstrnew
=======
    gen_objc_methods(list,objclss,methodslab,metaclass,false);
>>>>>>> graemeg/cpstrnew
=======
    gen_objc_methods(list,objclss,methodslab,metaclass,false);
>>>>>>> origin/cpstrnew
    { generate ivars (nil for metaclass) }
    if metaclass then
      ivarslab:=nil
    else
      gen_objc_ivars(list,objclss,ivarslab);

    { class declaration section }
    new_section(list,sec_objc_const,'_OBJC_META_CLASS',sizeof(pint));

    rosym:=current_asmdata.DefineAsmSymbol(objclss.rtti_mangledname(rttitype),AB_LOCAL,AT_DATA);
    classrolabel:=rosym;
    list.Concat(tai_symbol.create(rosym,0));
    list.Concat(tai_const.Create_32bit(longint(flags)));
    list.Concat(tai_const.Create_32bit(longint(start)));
    list.Concat(tai_const.Create_32bit(longint(size)));
{$ifdef cpu64bitaddr}
    { alignment }
    list.Concat(tai_const.Create_32bit(0));
{$endif}
    { TODO: strong ivar layout for garbage collection }
    list.concat(tai_const.Create_pint(0));
    list.concat(tai_const.Create_sym(classStrSym));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ConcatSymOrNil(list,methodssym);
=======
    ConcatSymOrNil(list,methodslab);
>>>>>>> graemeg/cpstrnew
=======
    ConcatSymOrNil(list,methodslab);
>>>>>>> graemeg/cpstrnew
=======
    ConcatSymOrNil(list,methodslab);
>>>>>>> graemeg/cpstrnew
=======
    ConcatSymOrNil(list,methodslab);
>>>>>>> origin/cpstrnew
    ConcatSymOrNil(list,protolistsym);
    ConcatSymOrNil(list,ivarslab);
    { TODO: weak ivar layout for garbage collection }
    list.concat(tai_const.Create_pint(0));
    { TODO: properties }
    list.concat(tai_const.Create_pint(0));
  end;


(*
From Clang:

/// struct _class_t {
///   struct _class_t *isa;
///   struct _class_t * const superclass;
///   void *cache;
///   IMP *vtable;
///   struct class_ro_t *ro;
/// }
///
*)

{ Generate rtti for an Objective-C class and its meta-class. }
procedure tobjcrttiwriter_nonfragile.gen_objc_classes_sections(list:TAsmList; objclss: tobjectdef; out classlabel: TAsmSymbol);
  var
    root          : tobjectdef;
    superSym,
    superMetaSym,
    metaisaSym,
    metasym,
    clssym,
    metarosym,
    rosym         : TAsmSymbol;
    protolistsym  : TAsmLabel;
    vis           : TAsmsymbind;
  begin
    { A) Register necessary names }

    { 1) the current class and metaclass }
    if (objclss.owner.symtabletype=globalsymtable) then
      vis:=AB_GLOBAL
    else
      vis:=AB_PRIVATE_EXTERN;
    clssym:=current_asmdata.DefineAsmSymbol(objclss.rtti_mangledname(objcclassrtti),vis,AT_DATA);
    metasym:=current_asmdata.DefineAsmSymbol(objclss.rtti_mangledname(objcmetartti),vis,AT_DATA);
    { 2) the superclass and meta superclass }
    if assigned(objclss.childof) then
      begin
        superSym:=current_asmdata.RefAsmSymbol(objclss.childof.rtti_mangledname(objcclassrtti));
        superMetaSym:=current_asmdata.RefAsmSymbol(objclss.childof.rtti_mangledname(objcmetartti));
      end
    else
      begin
        superSym:=nil;
        { the class itself }
        superMetaSym:=clssym;
      end;

    { 3) the isa }
    { From Clang: The isa for the meta-class is the root of the hierarchy. }
    root:=objclss;
    while assigned(root.childof) do
      root:=root.childof;
    metaisaSym:=current_asmdata.RefAsmSymbol(root.rtti_mangledname(objcmetartti));

    { 4) the implemented protocols (same for metaclass and regular class) }
    gen_objc_protocol_list(list,objclss.ImplementedInterfaces,protolistsym);

    { 5) the read-only parts of the class definitions }
    gen_objc_class_ro_part(list,objclss,protolistsym,metarosym,true);
    gen_objc_class_ro_part(list,objclss,protolistsym,rosym,false);

    { B) Class declaration section }
    { both class and metaclass are in the objc_data section for obj-c 2 }
    new_section(list,sec_objc_data,'_OBJC_CLASS',sizeof(pint));

    { 1) meta-class declaration }
    list.Concat(tai_symbol.Create_Global(metasym,0));

    { the isa }
    list.Concat(Tai_const.Create_sym(metaisaSym));
    { the superclass }
    list.Concat(Tai_const.Create_sym(superMetaSym));
    { pointer to cache }
    if not assigned(ObjCEmptyCacheVar) then
      ObjCEmptyCacheVar:=current_asmdata.RefAsmSymbol(target_info.Cprefix+'_objc_empty_cache');
    list.Concat(Tai_const.Create_sym(ObjCEmptyCacheVar));
    { pointer to vtable }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if not assigned(ObjCEmptyVtableVar) and
       not(target_info.system in [system_arm_darwin,system_aarch64_darwin,system_i386_iphonesim,system_x86_64_iphonesim]) then
      ObjCEmptyVtableVar:=current_asmdata.RefAsmSymbol(target_info.Cprefix+'_objc_empty_vtable');
    ConcatSymOrNil(list,ObjCEmptyVtableVar);
=======
    if not assigned(ObjCEmptyVtableVar) then
      ObjCEmptyVtableVar:=current_asmdata.RefAsmSymbol(target_info.Cprefix+'_objc_empty_vtable');
    list.Concat(Tai_const.Create_sym(ObjCEmptyVtableVar));
>>>>>>> graemeg/cpstrnew
=======
    if not assigned(ObjCEmptyVtableVar) then
      ObjCEmptyVtableVar:=current_asmdata.RefAsmSymbol(target_info.Cprefix+'_objc_empty_vtable');
    list.Concat(Tai_const.Create_sym(ObjCEmptyVtableVar));
>>>>>>> graemeg/cpstrnew
=======
    if not assigned(ObjCEmptyVtableVar) then
      ObjCEmptyVtableVar:=current_asmdata.RefAsmSymbol(target_info.Cprefix+'_objc_empty_vtable');
    list.Concat(Tai_const.Create_sym(ObjCEmptyVtableVar));
>>>>>>> graemeg/cpstrnew
=======
    if not assigned(ObjCEmptyVtableVar) then
      ObjCEmptyVtableVar:=current_asmdata.RefAsmSymbol(target_info.Cprefix+'_objc_empty_vtable');
    list.Concat(Tai_const.Create_sym(ObjCEmptyVtableVar));
>>>>>>> origin/cpstrnew
    { the read-only part }
    list.Concat(Tai_const.Create_sym(metarosym));

    { 2) regular class declaration }
    list.Concat(tai_symbol.Create_Global(clssym,0));

    { the isa }
    list.Concat(Tai_const.Create_sym(metasym));
    { the superclass }
    list.Concat(Tai_const.Create_sym(superSym));
    { pointer to cache }
    list.Concat(Tai_const.Create_sym(ObjCEmptyCacheVar));
    { pointer to vtable }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ConcatSymOrNil(list,ObjCEmptyVtableVar);
=======
    list.Concat(Tai_const.Create_sym(ObjCEmptyVtableVar));
>>>>>>> graemeg/cpstrnew
=======
    list.Concat(Tai_const.Create_sym(ObjCEmptyVtableVar));
>>>>>>> graemeg/cpstrnew
=======
    list.Concat(Tai_const.Create_sym(ObjCEmptyVtableVar));
>>>>>>> graemeg/cpstrnew
=======
    list.Concat(Tai_const.Create_sym(ObjCEmptyVtableVar));
>>>>>>> origin/cpstrnew
    { the read-only part }
    list.Concat(Tai_const.Create_sym(rosym));

    classlabel:=clssym;
  end;


procedure tobjcrttiwriter_nonfragile.addclasslist(list: tasmlist; section: tasmsectiontype; const symname: string; classes: tfpobjectlist);
  var
    i: longint;
    sym: TAsmSymbol;
  begin
    if classes.count=0 then
      exit;
    new_section(list,section,symname,sizeof(pint));
    sym:=current_asmdata.DefineAsmSymbol(symname,AB_LOCAL,AT_DATA);
    list.concat(tai_symbol.Create(sym,0));
    for i:=0 to classes.count-1 do
      list.concat(tai_const.Create_sym(current_asmdata.RefAsmSymbol(tobjectdef(classes[i]).rtti_mangledname(objcclassrtti))));
  end;


procedure tobjcrttiwriter_nonfragile.gen_objc_info_sections(list: tasmlist);

  function collectnonlazyclasses(classes: tfpobjectlist): tfpobjectlist;
    var
      symentry : tsym;
      procdef  : tprocdef;
      i,j      : longint;
    begin
      { non-lazy classes are all classes that define a class method with the
        selector called "load" (simply inheriting this class method is not enough,
        they have to implement it themselves)

        -- TODO: this currently only works if the Pascal identifier is also 'load'! }
      result:=tfpobjectlist.create(false);
      for i:=0 to classes.count-1 do
        begin
          symentry:=tsym(tobjectsymtable(tobjectdef(classes[i]).symtable).find('LOAD'));
          if assigned(symentry) and
             (symentry.typ=procsym) then
            begin
              for j:=0 to tprocsym(symentry).ProcdefList.count do
                begin
                  procdef:=tprocdef(tprocsym(symentry).ProcdefList[0]);
                  if ((po_classmethod in procdef.procoptions) and
                      (procdef.messageinf.str^='load')) then
                    begin
                      result.add(classes[i]);
                      break;
                    end;
                end;
            end;
        end;
    end;

  var
    nonlazyclasses,
    nonlazycategories : tfpobjectlist;
  begin
    if (classdefs.count=0) and
       (catdefs.count=0) then
      exit;

    nonlazyclasses:=collectnonlazyclasses(classdefs);
    nonlazycategories:=collectnonlazyclasses(catdefs);

    { this list has to include all classes, also the non-lazy ones }
    addclasslist(list,sec_objc_classlist,target_asm.labelprefix+'_OBJC_LABEL_CLASS_$',classdefs);
    addclasslist(list,sec_objc_nlclasslist,target_asm.labelprefix+'_OBJC_LABEL_NONLAZY_CLASS_$',nonlazyclasses);
    { category and non-lazy category lists }
    addclasslist(list,sec_objc_catlist,target_asm.labelprefix+'_OBJC_LABEL_CATEGORY_$',catdefs);
    addclasslist(list,sec_objc_nlcatlist,target_asm.labelprefix+'_OBJC_LABEL_NONLAZY_CATEGORY_$',nonlazycategories);

    nonlazyclasses.free;
    nonlazycategories.free;
    { the non-fragile abi doesn't have any module info, nor lazy references
      to used classes or to parent classes }
  end;


constructor tobjcrttiwriter_nonfragile.create;
  begin
    inherited create(oa_nonfragile);
  end;


{******************************************************************
                 RTTI generation -- Main function
*******************************************************************}

procedure MaybeGenerateObjectiveCImageInfo(globalst, localst: tsymtable);
  var
    objcrttiwriter: tobjcrttiwriter;
  begin
    if (m_objectivec1 in current_settings.modeswitches) then
      begin
        { first 4 bytes contain version information about this section (currently version 0),
          next 4 bytes contain flags (currently only regarding whether the code in the object
          file supports or requires garbage collection)
        }
        new_section(current_asmdata.asmlists[al_objc_data],sec_objc_image_info,'_OBJC_IMAGE_INFO',sizeof(pint));
        current_asmdata.asmlists[al_objc_data].concat(Tai_symbol.Createname(target_asm.labelprefix+'_OBJC_IMAGE_INFO',AT_LABEL,sizeof(pint)));
        current_asmdata.asmlists[al_objc_data].concat(Tai_const.Create_64bit(0));

        { generate rtti for all obj-c classes, protocols and categories
          defined in this module. }
        if not(target_info.system in systems_objc_nfabi) then
          objcrttiwriter:=tobjcrttiwriter_fragile.create
        else
          objcrttiwriter:=tobjcrttiwriter_nonfragile.create;
        objcrttiwriter.gen_objc_rtti_sections(current_asmdata.asmlists[al_objc_data],globalst);
        objcrttiwriter.gen_objc_rtti_sections(current_asmdata.asmlists[al_objc_data],localst);
        objcrttiwriter.gen_objc_info_sections(current_asmdata.asmlists[al_objc_data]);
        objcrttiwriter.free;
      end;
  end;


end.
