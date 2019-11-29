{
    Copyright (c) 2014 by Florian Klaempfl

    Symbol table overrides for JVM

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
unit symcpu;

{$i fpcdefs.inc}

interface

uses
  globtype,
  aasmdata,
  symtype,
  symdef,symsym;

type
  { defs }
  tcpufiledef = class(tfiledef)
  end;
  tcpufiledefclass = class of tcpufiledef;

  tcpuvariantdef = class(tvariantdef)
  end;
  tcpuvariantdefclass = class of tcpuvariantdef;

  tcpuformaldef = class(tformaldef)
  end;
  tcpuformaldefclass = class of tcpuformaldef;

  tcpuforwarddef = class(tforwarddef)
  end;
  tcpuforwarddefclass = class of tcpuforwarddef;

  tcpuundefineddef = class(tundefineddef)
  end;
  tcpuundefineddefclass = class of tcpuundefineddef;

  tcpuerrordef = class(terrordef)
  end;
  tcpuerrordefclass = class of tcpuerrordef;

  tcpupointerdef = class(tpointerdef)
  end;
  tcpupointerdefclass = class of tcpupointerdef;

  tcpurecorddef = class(trecorddef)
  end;
  tcpurecorddefclass = class of tcpurecorddef;

  tcpuimplementedinterface = class(timplementedinterface)
  end;
  tcpuimplementedinterfaceclass = class of tcpuimplementedinterface;

  tcpuobjectdef = class(tobjectdef)
  end;
  tcpuobjectdefclass = class of tcpuobjectdef;

  tcpuclassrefdef = class(tclassrefdef)
  end;
  tcpuclassrefdefclass = class of tcpuclassrefdef;

  tcpuarraydef = class(tarraydef)
  end;
  tcpuarraydefclass = class of tcpuarraydef;

  tcpuorddef = class(torddef)
  end;
  tcpuorddefclass = class of tcpuorddef;

  tcpufloatdef = class(tfloatdef)
  end;
  tcpufloatdefclass = class of tcpufloatdef;

  tcpuprocvardef = class(tprocvardef)
   protected
    procedure ppuwrite_platform(ppufile: tcompilerppufile); override;
    procedure ppuload_platform(ppufile: tcompilerppufile); override;
   public
    { class representing this procvar on the Java side }
    classdef  : tobjectdef;
    classdefderef : tderef;
    procedure buildderef;override;
    procedure deref;override;
    function getcopy: tstoreddef; override;
    function generate_safecall_wrapper: boolean; override;
  end;
  tcpuprocvardefclass = class of tcpuprocvardef;

  tcpuprocdef = class(tprocdef)
    { generated assembler code; used by JVM backend so it can afterwards
      easily write out all methods grouped per class }
    exprasmlist      : TAsmList;
    function  jvmmangledbasename(signature: boolean): TSymStr;
    function mangledname: TSymStr; override;
    function get_funcretsym_info(out ressym: tsym; out resdef: tdef): boolean; override;
    function generate_safecall_wrapper: boolean; override;
    destructor destroy; override;
  end;
  tcpuprocdefclass = class of tcpuprocdef;

  tcpustringdef = class(tstringdef)
  end;
  tcpustringdefclass = class of tcpustringdef;

  tcpuenumdef = class(tenumdef)
   protected
     procedure ppuload_platform(ppufile: tcompilerppufile); override;
     procedure ppuwrite_platform(ppufile: tcompilerppufile); override;
   public
    { class representing this enum on the Java side }
    classdef  : tobjectdef;
    classdefderef : tderef;
    function getcopy: tstoreddef; override;
    procedure buildderef; override;
    procedure deref; override;
  end;
  tcpuenumdefclass = class of tcpuenumdef;

  tcpusetdef = class(tsetdef)
  end;
  tcpusetdefclass = class of tcpusetdef;

  { syms }
  tcpulabelsym = class(tlabelsym)
  end;
  tcpulabelsymclass = class of tcpulabelsym;

  tcpuunitsym = class(tunitsym)
  end;
  tcpuunitsymclass = class of tcpuunitsym;

  tcpuprogramparasym = class(tprogramparasym)
  end;
  tcpuprogramparasymclass = class(tprogramparasym);

  tcpunamespacesym = class(tnamespacesym)
  end;
  tcpunamespacesymclass = class of tcpunamespacesym;

  tcpuprocsym = class(tprocsym)
    procedure check_forward; override;
  end;
  tcpuprocsymclass = class of tcpuprocsym;

  tcputypesym = class(ttypesym)
  end;
  tcpuypesymclass = class of tcputypesym;

  tcpufieldvarsym = class(tfieldvarsym)
    procedure set_externalname(const s: string); override;
    function mangledname: TSymStr; override;
  end;
  tcpufieldvarsymclass = class of tcpufieldvarsym;

  tcpulocalvarsym = class(tlocalvarsym)
  end;
  tcpulocalvarsymclass = class of tcpulocalvarsym;

  tcpuparavarsym = class(tparavarsym)
  end;
  tcpuparavarsymclass = class of tcpuparavarsym;

  tcpustaticvarsym = class(tstaticvarsym)
    procedure set_mangledname(const s: TSymStr); override;
    function mangledname: TSymStr; override;
  end;
  tcpustaticvarsymclass = class of tcpustaticvarsym;

  tcpuabsolutevarsym = class(tabsolutevarsym)
  end;
  tcpuabsolutevarsymclass = class of tcpuabsolutevarsym;

  tcpupropertysym = class(tpropertysym)
   protected
    { when a private/protected field is exposed via a property with a higher
      visibility, then we have to create a getter and/or setter with that same
      higher visibility to make sure that using the property does not result
      in JVM verification errors }
    procedure create_getter_or_setter_for_property(orgaccesspd: tprocdef; getter: boolean);
    procedure finalize_getter_or_setter_for_sym(getset: tpropaccesslisttypes; sym: tsym; fielddef: tdef; accessordef: tprocdef); override;
    procedure maybe_create_overridden_getter_or_setter(getset: tpropaccesslisttypes);
   public
    procedure inherit_accessor(getset: tpropaccesslisttypes); override;
  end;
  tcpupropertysymclass = class of tcpupropertysym;

  tcpuconstsym = class(tconstsym)
  end;
  tcpuconstsymclass = class of tcpuconstsym;

  tcpuenumsym = class(tenumsym)
  end;
  tcpuenumsymclass = class of tcpuenumsym;

  tcpusyssym = class(tsyssym)
  end;
  tcpusyssymclass = class of tcpusyssym;


const
  pbestrealtype : ^tdef = @s64floattype;


implementation

  uses
    verbose,cutils,cclasses,globals,
    symconst,symbase,symtable,symcreat,jvmdef,
    pdecsub,pparautl,pjvm,
    paramgr;


  {****************************************************************************
                               tcpuproptertysym
  ****************************************************************************}

  procedure tcpupropertysym.create_getter_or_setter_for_property(orgaccesspd: tprocdef; getter: boolean);
    var
      obj: tabstractrecorddef;
      ps: tprocsym;
      pvs: tparavarsym;
      sym: tsym;
      pd, parentpd, accessorparapd: tprocdef;
      tmpaccesslist: tpropaccesslist;
      callthroughpropname,
      accessorname: string;
      callthroughprop: tpropertysym;
      accesstyp: tpropaccesslisttypes;
      accessortyp: tprocoption;
      procoptions: tprocoptions;
      paranr: word;
      explicitwrapper: boolean;
    begin
      obj:=current_structdef;
      { if someone gets the idea to add a property to an external class
        definition, don't try to wrap it since we cannot add methods to
        external classes }
      if oo_is_external in obj.objectoptions then
        exit;
      symtablestack.push(obj.symtable);

      try
        if getter then
          accesstyp:=palt_read
        else
          accesstyp:=palt_write;

        { we can't use str_parse_method_dec here because the type of the field
          may not be visible at the Pascal level }

        explicitwrapper:=
          { private methods are not visibile outside the current class, so
            no use in making life harder for us by introducing potential
            (future or current) naming conflicts }
          (visibility<>vis_private) and
          (getter and
           (prop_auto_getter_prefix<>'')) or
          (not getter and
           (prop_auto_setter_prefix<>''));
        sym:=nil;
        if getter then
          accessortyp:=po_is_auto_getter
        else
          accessortyp:=po_is_auto_setter;
        procoptions:=[accessortyp];
        if explicitwrapper then
          begin
            if getter then
              accessorname:=prop_auto_getter_prefix+realname
            else
              accessorname:=prop_auto_setter_prefix+realname;
            sym:=search_struct_member_no_helper(obj,upper(accessorname));
            if assigned(sym) then
              begin
                if ((sym.typ<>procsym) or
                    (tprocsym(sym).procdeflist.count<>1) or
                    not(accessortyp in tprocdef(tprocsym(sym).procdeflist[0]).procoptions)) and
                   (not assigned(orgaccesspd) or
                    (sym<>orgaccesspd.procsym)) then
                  begin
                    MessagePos2(fileinfo,parser_e_cannot_generate_property_getter_setter,accessorname,FullTypeName(tdef(sym.owner.defowner),nil)+'.'+accessorname);
                    exit;
                  end
                else
                  begin
                    if accessorname<>sym.realname then
                      MessagePos2(fileinfo,parser_w_case_difference_auto_property_getter_setter_prefix,sym.realname,accessorname);
                    { is the specified getter/setter defined in the current
                      struct and was it originally specified as the getter/
                      setter for this property? If so, simply adjust its
                      visibility if necessary.
                    }
                    if assigned(orgaccesspd) then
                      parentpd:=orgaccesspd
                    else
                      parentpd:=tprocdef(tprocsym(sym).procdeflist[0]);
                    if parentpd.owner.defowner=owner.defowner then
                      begin
                        if parentpd.visibility<visibility then
                          begin
                            parentpd.visibility:=visibility;
                            include(parentpd.procoptions,po_auto_raised_visibility);
                          end;
                        { we are done, no need to create a wrapper }
                        exit
                      end
                    { a parent already included this getter/setter -> try to
                      override it }
                    else if parentpd.visibility<>vis_private then
                      begin
                        if po_virtualmethod in parentpd.procoptions then
                          begin
                            procoptions:=procoptions+[po_virtualmethod,po_overridingmethod];
                            if not(parentpd.synthetickind in [tsk_field_getter,tsk_field_setter]) then
                              Message2(parser_w_overriding_property_getter_setter,accessorname,FullTypeName(tdef(parentpd.owner.defowner),nil));
                          end;
                        { otherwise we can't do anything, and
                          proc_add_definition will give an error }
                      end;
                    { add method with the correct visibility }
                    pd:=tprocdef(parentpd.getcopyas(procdef,pc_normal_no_hidden,''));
                    { get rid of the import accessorname for inherited virtual class methods,
                      it has to be regenerated rather than amended }
                    if [po_classmethod,po_virtualmethod]<=pd.procoptions then
                      begin
                        stringdispose(pd.import_name);
                        exclude(pd.procoptions,po_has_importname);
                      end;
                    pd.visibility:=visibility;
                    pd.procoptions:=pd.procoptions+procoptions;
                    { ignore this artificially added procdef when looking for overloads }
                    include(pd.procoptions,po_ignore_for_overload_resolution);
                    finish_copied_procdef(pd,parentpd.procsym.realname,obj.symtable,obj);
                    exclude(pd.procoptions,po_external);
                    pd.synthetickind:=tsk_anon_inherited;
                    { set the accessor in the property }
                    propaccesslist[accesstyp].clear;
                    propaccesslist[accesstyp].addsym(sl_call,pd.procsym);
                    propaccesslist[accesstyp].procdef:=pd;
                    exit;
                  end;
              end;
            { make the artificial getter/setter virtual so we can override it in
              children if necessary }
            if not(sp_static in symoptions) and
               (obj.typ=objectdef) then
              include(procoptions,po_virtualmethod);
            { prevent problems in Delphi mode }
            include(procoptions,po_overload);
          end
        else
          begin
            { construct procsym accessorname (unique for this access; reusing the same
              helper for multiple accesses to the same field is hard because the
              propacesslist can contain subscript nodes etc) }
            accessorname:=visibilityName[visibility];
            replace(accessorname,' ','_');
            if getter then
              accessorname:=accessorname+'$getter'
            else
              accessorname:=accessorname+'$setter';
          end;

        { create procdef }
        if not assigned(orgaccesspd) then
          begin
            pd:=cprocdef.create(normal_function_level,true);
            if df_generic in obj.defoptions then
              include(pd.defoptions,df_generic);
            { method of this objectdef }
            pd.struct:=obj;
            { can only construct the artificial accessorname now, because it requires
              pd.unique_id_str }
            if not explicitwrapper then
              accessorname:='$'+obj.symtable.realname^+'$'+realname+'$'+accessorname+'$'+pd.unique_id_str;
          end
        else
          begin
            { getter/setter could have parameters in case of indexed access
              -> copy original procdef }
            pd:=tprocdef(orgaccesspd.getcopyas(procdef,pc_normal_no_hidden,''));
            exclude(pd.procoptions,po_abstractmethod);
            exclude(pd.procoptions,po_overridingmethod);
            { can only construct the artificial accessorname now, because it requires
              pd.unique_id_str }
            if not explicitwrapper then
              accessorname:='$'+obj.symtable.realname^+'$'+realname+'$'+accessorname+'$'+pd.unique_id_str;
            finish_copied_procdef(pd,accessorname,obj.symtable,obj);
            sym:=pd.procsym;
          end;
        { add previously collected procoptions }
        pd.procoptions:=pd.procoptions+procoptions;
        { visibility }
        pd.visibility:=visibility;

        { new procsym? }
        if not assigned(sym) or
           (sym.owner<>owner)  then
          begin
            ps:=cprocsym.create(accessorname);
            obj.symtable.insert(ps);
          end
        else
          ps:=tprocsym(sym);
        { associate procsym with procdef}
        pd.procsym:=ps;

        { function/procedure }
        accessorparapd:=nil;
        if getter then
          begin
            pd.proctypeoption:=potype_function;
            pd.synthetickind:=tsk_field_getter;
            { result type }
            pd.returndef:=propdef;
            if (ppo_hasparameters in propoptions) and
               not assigned(orgaccesspd) then
              accessorparapd:=pd;
          end
        else
          begin
            pd.proctypeoption:=potype_procedure;
            pd.synthetickind:=tsk_field_setter;
            pd.returndef:=voidtype;
            if not assigned(orgaccesspd) then
              begin
                { parameter with value to set }
                pvs:=cparavarsym.create('__fpc_newval__',10,vs_const,propdef,[]);
                pd.parast.insert(pvs);
              end;
            if (ppo_hasparameters in propoptions) and
               not assigned(orgaccesspd) then
              accessorparapd:=pd;
          end;

        { create a property for the old symaccesslist with a new accessorname, so that
          we can reuse it in the implementation (rather than having to
          translate the symaccesslist back to Pascal code) }
        callthroughpropname:='__fpc__'+realname;
        if getter then
          callthroughpropname:=callthroughpropname+'__getter_wrapper'
        else
          callthroughpropname:=callthroughpropname+'__setter_wrapper';
        callthroughprop:=cpropertysym.create(callthroughpropname);
        callthroughprop.visibility:=visibility;

        if getter then
          makeduplicate(callthroughprop,accessorparapd,nil,paranr)
        else
          makeduplicate(callthroughprop,nil,accessorparapd,paranr);

        callthroughprop.default:=longint($80000000);
        callthroughprop.default:=0;
        callthroughprop.propoptions:=callthroughprop.propoptions-[ppo_stored,ppo_enumerator_current,ppo_overrides,ppo_defaultproperty];
        if sp_static in symoptions then
          include(callthroughprop.symoptions, sp_static);
        { copy original property target to callthrough property (and replace
          original one with the new empty list; will be filled in later) }
        tmpaccesslist:=callthroughprop.propaccesslist[accesstyp];
        callthroughprop.propaccesslist[accesstyp]:=propaccesslist[accesstyp];
        propaccesslist[accesstyp]:=tmpaccesslist;
        owner.insert(callthroughprop);

        pd.skpara:=callthroughprop;
        { needs to be exported }
        include(pd.procoptions,po_global);
        { class property -> static class method }
        if sp_static in symoptions then
          pd.procoptions:=pd.procoptions+[po_classmethod,po_staticmethod];

        { in case we made a copy of the original accessor, this has all been
          done already }
        if not assigned(orgaccesspd) then
          begin
            { calling convention }
            handle_calling_convention(pd,hcc_default_actions_intf_struct);
            { register forward declaration with procsym }
            proc_add_definition(pd);
          end;

        { make the property call this new function }
        propaccesslist[accesstyp].addsym(sl_call,ps);
        propaccesslist[accesstyp].procdef:=pd;
      finally
        symtablestack.pop(obj.symtable);
      end;
    end;


  procedure tcpupropertysym.finalize_getter_or_setter_for_sym(getset: tpropaccesslisttypes; sym: tsym; fielddef: tdef; accessordef: tprocdef);
    var
      orgaccesspd: tprocdef;
      pprefix: pshortstring;
      wrongvisibility: boolean;
    begin
      inherited;
      if getset=palt_read then
        pprefix:=@prop_auto_getter_prefix
      else
        pprefix:=@prop_auto_setter_prefix;
      case sym.typ of
        procsym:
          begin
            orgaccesspd:=tprocdef(propaccesslist[getset].procdef);
            wrongvisibility:=tprocdef(propaccesslist[getset].procdef).visibility<visibility;
            { if the visibility of the accessor is lower than
              the visibility of the property, wrap it so that
              we can call it from all contexts in which the
              property is visible }
            if wrongvisibility or
               ((pprefix^<>'') and
                (sym.RealName<>pprefix^+RealName)) then
              create_getter_or_setter_for_property(orgaccesspd,getset=palt_read)
          end;
        fieldvarsym:
          begin
            { if the visibility of the field is lower than the
              visibility of the property, wrap it in a getter
              so that we can access it from all contexts in
              which the property is visibile }
            if (pprefix^<>'') or
               (tfieldvarsym(sym).visibility<visibility) then
              create_getter_or_setter_for_property(nil,getset=palt_read);
          end;
        else
          internalerror(2014061101);
      end;
    end;


  procedure tcpupropertysym.maybe_create_overridden_getter_or_setter(getset: tpropaccesslisttypes);
    var
      sym: tsym;
      accessordef: tprocdef;
      psym: tpropertysym;
    begin
      { find the last defined getter/setter/field accessed by an inherited
        property }
      psym:=overriddenpropsym;
      while not assigned(psym.propaccesslist[getset].firstsym) do
        begin
          psym:=psym.overriddenpropsym;
          { if there is simply no getter/setter for this property, we're done }
          if not assigned(psym) then
            exit;
        end;
      sym:=psym.propaccesslist[getset].firstsym^.sym;
      case sym.typ of
        procsym:
          begin
            accessordef:=tprocdef(psym.propaccesslist[getset].procdef);
            if accessordef.visibility>=visibility then
              exit;
          end;
        fieldvarsym:
          begin
            if sym.visibility>=visibility then
              exit;
            accessordef:=nil;
          end;
        else
          internalerror(2014061102);
      end;
      propaccesslist[getset]:=psym.propaccesslist[getset].getcopy;
      finalize_getter_or_setter_for_sym(getset,sym,propdef,accessordef);
    end;


  procedure tcpupropertysym.inherit_accessor(getset: tpropaccesslisttypes);
    begin
      inherited;
      { new property has higher visibility than previous one -> maybe override
        the getters/setters }
      if assigned(overriddenpropsym) and
         (overriddenpropsym.visibility<visibility) then
        maybe_create_overridden_getter_or_setter(getset);
    end;


{****************************************************************************
                             tcpuenumdef
****************************************************************************}

  procedure tcpuenumdef.ppuload_platform(ppufile: tcompilerppufile);
    begin
      inherited;
      ppufile.getderef(classdefderef);
    end;


  procedure tcpuenumdef.ppuwrite_platform(ppufile: tcompilerppufile);
    begin
      inherited;
      ppufile.putderef(classdefderef);
    end;


  function tcpuenumdef.getcopy: tstoreddef;
    begin
      result:=inherited;
      tcpuenumdef(result).classdef:=classdef;
    end;


  procedure tcpuenumdef.buildderef;
    begin
      inherited;
      classdefderef.build(classdef);
    end;


  procedure tcpuenumdef.deref;
    begin
      inherited;
      classdef:=tobjectdef(classdefderef.resolve);
    end;

{****************************************************************************
                             tcpuprocdef
****************************************************************************}

  function tcpuprocdef.jvmmangledbasename(signature: boolean): TSymStr;
  var
    vs: tparavarsym;
    i: longint;
    founderror: tdef;
    tmpresult: TSymStr;
    container: tsymtable;
  begin
    { format:
        * method definition (in Jasmin):
            (private|protected|public) [static] method(parametertypes)returntype
        * method invocation
            package/class/method(parametertypes)returntype
      -> store common part: method(parametertypes)returntype and
         adorn as required when using it.
    }
    if not signature then
      begin
        { method name }
        { special names for constructors and class constructors }
        if proctypeoption=potype_constructor then
          tmpresult:='<init>'
        else if proctypeoption in [potype_class_constructor,potype_unitinit] then
          tmpresult:='<clinit>'
        else if po_has_importname in procoptions then
          begin
            if assigned(import_name) then
              tmpresult:=import_name^
            else
              internalerror(2010122608);
          end
        else
          begin
            tmpresult:=procsym.realname;
            if tmpresult[1]='$' then
              tmpresult:=copy(tmpresult,2,length(tmpresult)-1);
            { nested functions }
            container:=owner;
            while container.symtabletype=localsymtable do
              begin
                tmpresult:='$'+tprocdef(owner.defowner).procsym.realname+'$$'+tprocdef(owner.defowner).unique_id_str+'$'+tmpresult;
                container:=container.defowner.owner;
              end;
          end;
      end
    else
      tmpresult:='';
    { parameter types }
    tmpresult:=tmpresult+'(';
    { not the case for the main program (not required for defaultmangledname
      because setmangledname() is called for the main program; in case of
      the JVM, this only sets the importname, however) }
    if assigned(paras) then
      begin
        for i:=0 to paras.count-1 do
          begin
            vs:=tparavarsym(paras[i]);
            { function result is not part of the mangled name }
            if vo_is_funcret in vs.varoptions then
              continue;
            { self pointer neither, except for class methods (the JVM only
              supports static class methods natively, so the self pointer
              here is a regular parameter as far as the JVM is concerned }
            if not(po_classmethod in procoptions) and
               (vo_is_self in vs.varoptions) then
              continue;
            { passing by reference is emulated by passing an array of one
              element containing the value; for types that aren't pointers
              in regular Pascal, simply passing the underlying pointer type
              does achieve regular call-by-reference semantics though;
              formaldefs always have to be passed like that because their
              contents can be replaced }
            if paramanager.push_copyout_param(vs.varspez,vs.vardef,proccalloption) then
              tmpresult:=tmpresult+'[';
            { Add the parameter type.  }
            if not jvmaddencodedtype(vs.vardef,false,tmpresult,signature,founderror) then
              { an internalerror here is also triggered in case of errors in the source code }
              tmpresult:='<error>';
          end;
      end;
    tmpresult:=tmpresult+')';
    { And the type of the function result (void in case of a procedure and
      constructor). }
    if (proctypeoption in [potype_constructor,potype_class_constructor]) then
      jvmaddencodedtype(voidtype,false,tmpresult,signature,founderror)
    else if not jvmaddencodedtype(returndef,false,tmpresult,signature,founderror) then
      { an internalerror here is also triggered in case of errors in the source code }
      tmpresult:='<error>';
    result:=tmpresult;
  end;


  function tcpuprocdef.mangledname: TSymStr;
    begin
      if _mangledname='' then
        begin
          result:=jvmmangledbasename(false);
          if (po_has_importdll in procoptions) then
            begin
              { import_dll comes from "external 'import_dll_name' name 'external_name'" }
              if assigned(import_dll) then
                result:=import_dll^+'/'+result
              else
                internalerror(2010122607);
            end
          else
            jvmaddtypeownerprefix(owner,mangledname);
          _mangledname:=result;
        end
      else
        result:=_mangledname;
    end;


  function tcpuprocdef.get_funcretsym_info(out ressym: tsym; out resdef: tdef): boolean;
    begin
      { constructors don't have a result on the JVM platform }
      if proctypeoption<>potype_constructor then
        result:=inherited
      else
        result:=false;
    end;


  function tcpuprocdef.generate_safecall_wrapper: boolean;
    begin
      result:=false;
    end;


  destructor tcpuprocdef.destroy;
    begin
      exprasmlist.free;
      inherited destroy;
    end;

{****************************************************************************
                             tcpuprocvardef
****************************************************************************}

  procedure tcpuprocvardef.ppuwrite_platform(ppufile: tcompilerppufile);
    begin
      inherited;
      ppufile.putderef(classdefderef);
    end;


  procedure tcpuprocvardef.ppuload_platform(ppufile: tcompilerppufile);
    begin
      inherited;
      ppufile.getderef(classdefderef);
    end;


  procedure tcpuprocvardef.buildderef;
    begin
      inherited buildderef;
      classdefderef.build(classdef);
    end;


  procedure tcpuprocvardef.deref;
    begin
      inherited deref;
      classdef:=tobjectdef(classdefderef.resolve);
    end;

  function tcpuprocvardef.getcopy: tstoreddef;
    begin
      result:=inherited;
      tcpuprocvardef(result).classdef:=classdef;
    end;


  function tcpuprocvardef.generate_safecall_wrapper: boolean;
    begin
      result:=false;
    end;


{****************************************************************************
                             tcpuprocsym
****************************************************************************}

  procedure tcpuprocsym.check_forward;
    var
      curri, checki: longint;
      currpd, checkpd: tprocdef;
    begin
      inherited;
      { check for conflicts based on mangled name, because several FPC
        types/constructs map to the same JVM mangled name }
      for curri:=0 to FProcdefList.Count-2 do
        begin
          currpd:=tprocdef(FProcdefList[curri]);
          if (po_external in currpd.procoptions) or
             (currpd.proccalloption=pocall_internproc) then
            continue;
          for checki:=curri+1 to FProcdefList.Count-1 do
            begin
              checkpd:=tprocdef(FProcdefList[checki]);
              if po_external in checkpd.procoptions then
                continue;
              if currpd.mangledname=checkpd.mangledname then
                begin
                  MessagePos(checkpd.fileinfo,parser_e_overloaded_have_same_mangled_name);
                  MessagePos1(currpd.fileinfo,sym_e_param_list,currpd.customprocname([pno_mangledname]));
                  MessagePos1(checkpd.fileinfo,sym_e_param_list,checkpd.customprocname([pno_mangledname]));
                end;
            end;
        end;
      inherited;
    end;


{****************************************************************************
                             tcpustaticvarsym
****************************************************************************}

  procedure tcpustaticvarsym.set_mangledname(const s: TSymStr);
    begin
      inherited;
      _mangledname:=jvmmangledbasename(self,s,false);
      jvmaddtypeownerprefix(owner,_mangledname);
    end;


  function tcpustaticvarsym.mangledname: TSymStr;
    begin
      if _mangledname='' then
        begin
          if _mangledbasename='' then
            _mangledname:=jvmmangledbasename(self,false)
          else
            _mangledname:=jvmmangledbasename(self,_mangledbasename,false);
          jvmaddtypeownerprefix(owner,_mangledname);
        end;
      result:=_mangledname;
    end;


{****************************************************************************
                             tcpufieldvarsym
****************************************************************************}

  procedure tcpufieldvarsym.set_externalname(const s: string);
    begin
      { make sure it is recalculated }
      cachedmangledname:='';
      if is_java_class_or_interface(tdef(owner.defowner)) then
        begin
          externalname:=stringdup(s);
          include(varoptions,vo_has_mangledname);
        end
      else
        internalerror(2011031201);
    end;


  function tcpufieldvarsym.mangledname: TSymStr;
    begin
      if is_java_class_or_interface(tdef(owner.defowner)) or
         (tdef(owner.defowner).typ=recorddef) then
        begin
          if cachedmangledname<>'' then
            result:=cachedmangledname
          else
            begin
              result:=jvmmangledbasename(self,false);
              jvmaddtypeownerprefix(owner,result);
              cachedmangledname:=result;
            end;
        end
      else
        result:=inherited;
    end;

begin
  { used tdef classes }
  cfiledef:=tcpufiledef;
  cvariantdef:=tcpuvariantdef;
  cformaldef:=tcpuformaldef;
  cforwarddef:=tcpuforwarddef;
  cundefineddef:=tcpuundefineddef;
  cerrordef:=tcpuerrordef;
  cpointerdef:=tcpupointerdef;
  crecorddef:=tcpurecorddef;
  cimplementedinterface:=tcpuimplementedinterface;
  cobjectdef:=tcpuobjectdef;
  cclassrefdef:=tcpuclassrefdef;
  carraydef:=tcpuarraydef;
  corddef:=tcpuorddef;
  cfloatdef:=tcpufloatdef;
  cprocvardef:=tcpuprocvardef;
  cprocdef:=tcpuprocdef;
  cstringdef:=tcpustringdef;
  cenumdef:=tcpuenumdef;
  csetdef:=tcpusetdef;

  { used tsym classes }
  clabelsym:=tcpulabelsym;
  cunitsym:=tcpuunitsym;
  cprogramparasym:=tcpuprogramparasym;
  cnamespacesym:=tcpunamespacesym;
  cprocsym:=tcpuprocsym;
  ctypesym:=tcputypesym;
  cfieldvarsym:=tcpufieldvarsym;
  clocalvarsym:=tcpulocalvarsym;
  cparavarsym:=tcpuparavarsym;
  cstaticvarsym:=tcpustaticvarsym;
  cabsolutevarsym:=tcpuabsolutevarsym;
  cpropertysym:=tcpupropertysym;
  cconstsym:=tcpuconstsym;
  cenumsym:=tcpuenumsym;
  csyssym:=tcpusyssym;
end.

