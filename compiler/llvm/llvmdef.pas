{
    Copyright (c) 2013 by Jonas Maebe

    This unit implements some LLVM type helper routines.

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

{$i fpcdefs.inc}

unit llvmdef;

interface

    uses
      cclasses,globtype,
      aasmbase,
      parabase,
      symconst,symbase,symtype,symdef,
      llvmbase;

   type
     { there are three different circumstances in which procdefs are used:
        a) definition of a procdef that's implemented in the current module
        b) declaration of an external routine that's called in the current one
        c) alias declaration of a procdef implemented in the current module
        d) defining a procvar type
       The main differences between the contexts are:
        a) information about sign extension of result type, proc name, parameter names & sign-extension info & types
        b) information about sign extension of result type, proc name, no parameter names, with parameter sign-extension info & types
        c) no information about sign extension of result type, proc name, no parameter names, no information about sign extension of parameters, parameter types
        d) no information about sign extension of result type, no proc name, no parameter names, no information about sign extension of parameters, parameter types
      }
     tllvmprocdefdecltype = (lpd_def,lpd_decl,lpd_alias,lpd_procvar);

    { returns the identifier to use as typename for a def in llvm (llvm only
      allows naming struct types) -- only supported for defs with a typesym, and
      only for tabstractrecorddef descendantds and complex procvars }
    function llvmtypeidentifier(def: tdef): TSymStr;

    { encode a type into the internal format used by LLVM (for a type
      declaration) }
    function llvmencodetypedecl(def: tdef): TSymStr;

    { same as above, but use a type name if possible (for any use) }
    function llvmencodetypename(def: tdef): TSymStr;

    { encode a procdef/procvardef into the internal format used by LLVM }
    function llvmencodeproctype(def: tabstractprocdef; const customname: TSymStr; pddecltype: tllvmprocdefdecltype): TSymStr;
    { incremental version of the above }
    procedure llvmaddencodedproctype(def: tabstractprocdef; const customname: TSymStr; pddecltype: tllvmprocdefdecltype; var encodedstr: TSymStr);

    { function result types may have to be represented differently, e.g. a
      record consisting of 4 longints must be returned as a record consisting of
      two int64's on x86-64. This function is used to create (and reuse)
      temporary recorddefs for such purposes.}
    function llvmgettemprecorddef(const fieldtypes: array of tdef; packrecords, recordalignmin: shortint): trecorddef;

    { get the llvm type corresponding to a parameter, e.g. a record containing
      two integer int64 for an arbitrary record split over two individual int64
      parameters, or an int32 for an int16 parameter on a platform that requires
      such parameters to be zero/sign extended. The second parameter can be used
      to get the type before zero/sign extension, as e.g. required to generate
      function declarations. }
    function llvmgetcgparadef(const cgpara: tcgpara; beforevalueext: boolean; callercallee: tcallercallee): tdef;

    { can be used to extract the value extension info from acgpara. Pass in
      the def of the cgpara as first parameter and a local variable holding
      a copy of the def of the location (value extension only makes sense for
      ordinal parameters that are smaller than a single location). The routine
      will return the def of the location without sign extension (if applicable)
      and the kind of sign extension that was originally performed in the
      signext parameter }
    procedure llvmextractvalueextinfo(paradef: tdef; var paralocdef: tdef; out signext: tllvmvalueextension);

    { returns whether a paraloc should be translated into an llvm "byval"
      parameter. These are declared as pointers to a particular type, but
      usually turned into copies onto the stack. The exact behaviour for
      parameters that should be passed in registers is undefined and depends on
      the platform, and furthermore this modifier sometimes inhibits
      optimizations.  As a result,we only use it for aggregate parameters of
      which we know that they should be passed on the stack }
    function llvmbyvalparaloc(paraloc: pcgparalocation): boolean;

    { returns whether a def is representated by an aggregate type in llvm
      (struct, array) }
    function llvmaggregatetype(def: tdef): boolean;

    function llvmconvop(var fromsize, tosize: tdef; inregs: boolean): tllvmop;

    { mangle a global identifier so that it's recognised by LLVM as a global
      (in the sense of module-global) label and so that it won't mangle the
      name further according to platform conventions (we already did that) }
    function llvmmangledname(const s: TSymStr): TSymStr;

    function llvmasmsymname(const sym: TAsmSymbol): TSymStr;


implementation

  uses
    globals,cutils,constexp,
    verbose,systems,
    fmodule,
    symtable,symsym,
    llvmsym,hlcgobj,
    defutil,blockutl,cgbase,paramgr,
    cpubase;


{******************************************************************
                          Type encoding
*******************************************************************}

  function llvmtypeidentifier(def: tdef): TSymStr;
    begin
      if not assigned(def.typesym) then
        internalerror(2015041901);
      result:='%"typ.'+def.fullownerhierarchyname(false)+def.typesym.realname+'"'
    end;


  function llvmaggregatetype(def: tdef): boolean;
    begin
      result:=
        (def.typ in [recorddef,filedef,variantdef]) or
        ((def.typ=arraydef) and
         not is_dynamic_array(def)) or
        ((def.typ=setdef) and
         not is_smallset(def)) or
        is_shortstring(def) or
        is_object(def) or
        ((def.typ=procvardef) and
         not tprocvardef(def).is_addressonly)
    end;


  function llvmconvop(var fromsize, tosize: tdef; inregs: boolean): tllvmop;
    var
      fromregtyp,
      toregtyp: tregistertype;
      frombytesize,
      tobytesize: asizeint;
    begin
      fromregtyp:=chlcgobj.def2regtyp(fromsize);
      toregtyp:=chlcgobj.def2regtyp(tosize);
      { int to pointer or vice versa }
      if fromregtyp=R_ADDRESSREGISTER then
        begin
          case toregtyp of
            R_INTREGISTER:
              result:=la_ptrtoint;
            R_ADDRESSREGISTER:
              result:=la_bitcast;
            else
              result:=la_ptrtoint_to_x;
            end;
        end
      else if toregtyp=R_ADDRESSREGISTER then
        begin
          case fromregtyp of
            R_INTREGISTER:
              result:=la_inttoptr;
            R_ADDRESSREGISTER:
              result:=la_bitcast;
            else
              result:=la_x_to_inttoptr;
            end;
        end
      else
        begin
          { treat comp and currency as extended in registers (see comment at start
            of thlgcobj.a_loadfpu_ref_reg) }
          if inregs and
             (fromsize.typ=floatdef) then
            begin
              if tfloatdef(fromsize).floattype in [s64comp,s64currency] then
                fromsize:=sc80floattype;
              { at the value level, s80real and sc80real are the same }
              if tfloatdef(fromsize).floattype<>s80real then
                frombytesize:=fromsize.size
              else
                frombytesize:=sc80floattype.size;
            end
          else
            frombytesize:=fromsize.size;

          if inregs and
             (tosize.typ=floatdef) then
            begin
              if tfloatdef(tosize).floattype in [s64comp,s64currency] then
                tosize:=sc80floattype;
              if tfloatdef(tosize).floattype<>s80real then
                tobytesize:=tosize.size
              else
                tobytesize:=sc80floattype.size;
            end
          else
            tobytesize:=tosize.size;

          { need zero/sign extension, float truncation or plain bitcast? }
          if tobytesize<>frombytesize then
            begin
              case fromregtyp of
                R_FPUREGISTER,
                R_MMREGISTER:
                  begin
                    { todo: update once we support vectors }
                    if not(toregtyp in [R_FPUREGISTER,R_MMREGISTER]) then
                      internalerror(2014062203);
                    if tobytesize<frombytesize then
                      result:=la_fptrunc
                    else
                      result:=la_fpext
                  end;
                else
                  begin
                    if tobytesize<frombytesize then
                      result:=la_trunc
                    else if is_signed(fromsize) then
                      { fromsize is signed -> sign extension }
                      result:=la_sext
                    else
                      result:=la_zext;
                  end;
              end;
            end
          else if (fromsize=llvmbool1type) and
                  (tosize<>llvmbool1type) then
            begin
              if is_cbool(tosize) then
                result:=la_sext
              else
                result:=la_zext
            end
          else if (tosize=llvmbool1type) and
                  (fromsize<>llvmbool1type) then
            begin
              { would have to compare with 0, can't just take the lowest bit }
              if is_cbool(fromsize) then
                internalerror(2016052001)
              else
                result:=la_trunc
            end
          else
            result:=la_bitcast;
        end;
    end;


  function llvmmangledname(const s: TSymStr): TSymStr;
    begin
      if copy(s,1,length('llvm.'))<>'llvm.' then
        if s[1]<>'"' then
          result:='@"\01'+s+'"'
        else
          begin
            { already quoted -> insert \01 and prepend @ }
            result:='@'+s;
            insert('\01',result,3);
          end
      else
        result:='@'+s
    end;


  function llvmasmsymname(const sym: TAsmSymbol): TSymStr;
    begin
      { AT_ADDR and AT_LABEL represent labels in the code, which have
        a different type in llvm compared to (global) data labels }
      if sym.bind=AB_TEMP then
        result:='%'+sym.name
      else if not(sym.typ in [AT_LABEL,AT_ADDR]) then
        result:=llvmmangledname(sym.name)
      else
        result:='label %'+sym.name;
    end;


  function llvmbyvalparaloc(paraloc: pcgparalocation): boolean;
    begin
      { "byval" is broken for register paras on several platforms in llvm
        (search for "byval" in llvm's bug tracker). Additionally, it should only
        be used to pass aggregate parameters on the stack, because it reportedly
        inhibits llvm's midlevel optimizers.

        Exception (for now?): parameters that have special shifting
          requirements, because modelling those in llvm is not easy (and clang
          nor llvm-gcc seem to do so either) }
      result:=
        ((paraloc^.loc=LOC_REFERENCE) and
         llvmaggregatetype(paraloc^.def)) or
        ((paraloc^.loc in [LOC_REGISTER,LOC_CREGISTER]) and
         (paraloc^.shiftval<>0))
    end;


  procedure llvmaddencodedabstractrecordtype(def: tabstractrecorddef; var encodedstr: TSymStr); forward;

  type
    tllvmencodeflag = (lef_inaggregate, lef_noimplicitderef, lef_typedecl);
    tllvmencodeflags = set of tllvmencodeflag;

    procedure llvmaddencodedtype_intern(def: tdef; const flags: tllvmencodeflags; var encodedstr: TSymStr);
      begin
        case def.typ of
          stringdef :
            begin
              case tstringdef(def).stringtype of
                st_widestring,
                st_unicodestring:
                  { the variable does not point to the header, but to a
                    null-terminated string/array with undefined bounds }
                  encodedstr:=encodedstr+'i16*';
                st_ansistring:
                  encodedstr:=encodedstr+'i8*';
                st_shortstring:
                  { length byte followed by string bytes }
                  if tstringdef(def).len>0 then
                    encodedstr:=encodedstr+'['+tostr(tstringdef(def).len+1)+' x i8]'
                  else
                    encodedstr:=encodedstr+'[0 x i8]';
                else
                  internalerror(2013100201);
              end;
            end;
          enumdef:
            begin
              encodedstr:=encodedstr+'i'+tostr(def.size*8);
            end;
          orddef :
            begin
              if is_void(def) then
                encodedstr:=encodedstr+'void'
              { mainly required because comparison operations return i1, and
                we need a way to represent the i1 type in Pascal. We don't
                reuse pasbool1type, because putting an i1 in a record or
                passing it as a parameter may result in unexpected behaviour }
              else if def=llvmbool1type then
                encodedstr:=encodedstr+'i1'
              else if torddef(def).ordtype<>customint then
                encodedstr:=encodedstr+'i'+tostr(def.size*8)
              else
                encodedstr:=encodedstr+'i'+tostr(def.packedbitsize);
            end;
          pointerdef :
            begin
              if is_voidpointer(def) then
                encodedstr:=encodedstr+'i8*'
              else
                begin
                  llvmaddencodedtype_intern(tpointerdef(def).pointeddef,[],encodedstr);
                  encodedstr:=encodedstr+'*';
                end;
            end;
          floatdef :
            begin
              case tfloatdef(def).floattype of
                s32real:
                  encodedstr:=encodedstr+'float';
                s64real:
                  encodedstr:=encodedstr+'double';
                { necessary to be able to force our own size/alignment }
                s80real:
                  { prevent llvm from allocating the standard ABI size for
                    extended }
                  if lef_inaggregate in flags then
                    encodedstr:=encodedstr+'[10 x i8]'
                  else
                    encodedstr:=encodedstr+'x86_fp80';
                sc80real:
                  encodedstr:=encodedstr+'x86_fp80';
                s64comp,
                s64currency:
                  encodedstr:=encodedstr+'i64';
                s128real:
{$if defined(powerpc) or defined(powerpc128)}
                  encodedstr:=encodedstr+'ppc_fp128';
{$else}
                  encodedstr:=encodedstr+'fp128';
{$endif}
              end;
            end;
          filedef :
            begin
              case tfiledef(def).filetyp of
                ft_text    :
                  llvmaddencodedtype_intern(search_system_type('TEXTREC').typedef,[lef_inaggregate]+[lef_typedecl]*flags,encodedstr);
                ft_typed   :
                  begin
                    { in case of ISO-like I/O, the typed file def includes a
                      get/put buffer of the size of the file's elements }
                    if (m_isolike_io in current_settings.modeswitches) and
                       not is_void(tfiledef(def).typedfiledef) then
                      encodedstr:=encodedstr+'<{';
                    llvmaddencodedtype_intern(search_system_type('FILEREC').typedef,[lef_inaggregate]+[lef_typedecl]*flags,encodedstr);
                    if (m_isolike_io in current_settings.modeswitches) and
                       not is_void(tfiledef(def).typedfiledef) then
                      begin
                        encodedstr:=encodedstr+',[';
                        encodedstr:=encodedstr+tostr(tfiledef(def).typedfiledef.size);
                        encodedstr:=encodedstr+' x i8]}>'
                      end;
                  end;
                ft_untyped :
                  llvmaddencodedtype_intern(search_system_type('FILEREC').typedef,[lef_inaggregate]+[lef_typedecl]*flags,encodedstr);
              end;
            end;
          recorddef :
            begin
              { avoid endlessly recursive definitions }
              if assigned(def.typesym) and
                 ((lef_inaggregate in flags) or
                  not(lef_typedecl in flags)) then
                encodedstr:=encodedstr+llvmtypeidentifier(def)
              else
                llvmaddencodedabstractrecordtype(trecorddef(def),encodedstr);
            end;
          variantdef :
            begin
              llvmaddencodedtype_intern(search_system_type('TVARDATA').typedef,[lef_inaggregate]+[lef_typedecl]*flags,encodedstr);
            end;
          classrefdef :
            begin
              if is_class(tclassrefdef(def).pointeddef) then
                begin
                  llvmaddencodedtype_intern(tobjectdef(tclassrefdef(def).pointeddef).vmt_def,flags,encodedstr);
                  encodedstr:=encodedstr+'*';
                end
              else if is_objcclass(tclassrefdef(def).pointeddef) then
                llvmaddencodedtype_intern(objc_idtype,flags,encodedstr)
              else
                encodedstr:=encodedstr+'i8*'
            end;
          setdef :
            begin
              { just an array as far as llvm is concerned; don't use a "packed
                array of i1" or so, this requires special support in backends
                and guarantees nothing about the internal format }
              if is_smallset(def) then
                llvmaddencodedtype_intern(cgsize_orddef(def_cgsize(def)),[lef_inaggregate],encodedstr)
              else
                encodedstr:=encodedstr+'['+tostr(tsetdef(def).size)+' x i8]';
            end;
          formaldef :
            begin
              { var/const/out x (always treated as "pass by reference" -> don't
                add extra "*" here) }
              encodedstr:=encodedstr+'i8';
            end;
          arraydef :
            begin
              if tarraydef(def).is_hwvector then
                begin
                  encodedstr:=encodedstr+'<'+tostr(tarraydef(def).elecount)+' x ';
                  llvmaddencodedtype_intern(tarraydef(def).elementdef,[lef_inaggregate],encodedstr);
                  encodedstr:=encodedstr+'>';
                end
              else if is_array_of_const(def) then
                begin
                  encodedstr:=encodedstr+'[0 x ';
                  llvmaddencodedtype_intern(search_system_type('TVARREC').typedef,[lef_inaggregate],encodedstr);
                  encodedstr:=encodedstr+']';
                end
              else if is_open_array(def) then
                begin
                  encodedstr:=encodedstr+'[0 x ';
                  llvmaddencodedtype_intern(tarraydef(def).elementdef,[lef_inaggregate],encodedstr);
                  encodedstr:=encodedstr+']';
                end
              else if is_dynamic_array(def) then
                begin
                  llvmaddencodedtype_intern(tarraydef(def).elementdef,[lef_inaggregate],encodedstr);
                  encodedstr:=encodedstr+'*';
                end
              else if is_packed_array(def) and
                      (tarraydef(def).elementdef.typ in [enumdef,orddef]) then
                begin
                  { encode as an array of bytes rather than as an array of
                    packedbitsloadsize(elesize), because even if the load size
                    is e.g. 2 bytes, the array may only be 1 or 3 bytes long
                    (and if this array is inside a record, it must not be
                     encoded as a type that is too long) }
                  encodedstr:=encodedstr+'['+tostr(tarraydef(def).size)+' x ';
                  llvmaddencodedtype_intern(u8inttype,[lef_inaggregate],encodedstr);
                  encodedstr:=encodedstr+']';
                end
              else
                begin
                  encodedstr:=encodedstr+'['+tostr(tarraydef(def).elecount)+' x ';
                  llvmaddencodedtype_intern(tarraydef(def).elementdef,[lef_inaggregate],encodedstr);
                  encodedstr:=encodedstr+']';
                end;
            end;
          procdef,
          procvardef :
            begin
              if (def.typ=procdef) or
                 tprocvardef(def).is_addressonly then
                begin
                  llvmaddencodedproctype(tabstractprocdef(def),'',lpd_procvar,encodedstr);
                  if def.typ=procvardef then
                    encodedstr:=encodedstr+'*';
                end
              else if ((lef_inaggregate in flags) or
                  not(lef_typedecl in flags)) and
                 assigned(tprocvardef(def).typesym) then
                begin
                  { in case the procvardef recursively references itself, e.g.
                    via a pointer }
                  encodedstr:=encodedstr+llvmtypeidentifier(def);
                  { blocks are implicit pointers }
                  if is_block(def) then
                    encodedstr:=encodedstr+'*'
                end
              else if is_block(def) then
                begin
                  llvmaddencodedtype_intern(get_block_literal_type_for_proc(tabstractprocdef(def)),flags,encodedstr);
                end
              else
                begin
                  encodedstr:=encodedstr+'<{';
                  { code pointer }
                  llvmaddencodedproctype(tabstractprocdef(def),'',lpd_procvar,encodedstr);
                  { data pointer (maybe todo: generate actual layout if
                    available) }
                  encodedstr:=encodedstr+'*, i8*}>';
                end;
            end;
          objectdef :
            case tobjectdef(def).objecttype of
              odt_class,
              odt_objcclass,
              odt_object,
              odt_cppclass:
                begin
                  if not(lef_typedecl in flags) and
                     assigned(def.typesym) then
                    encodedstr:=encodedstr+llvmtypeidentifier(def)
                  else
                    llvmaddencodedabstractrecordtype(tabstractrecorddef(def),encodedstr);
                  if ([lef_typedecl,lef_noimplicitderef]*flags=[]) and
                     is_implicit_pointer_object_type(def) then
                    encodedstr:=encodedstr+'*'
                end;
              odt_interfacecom,
              odt_interfacecorba,
              odt_dispinterface:
                begin
                  { type is a pointer to a pointer to the vmt }
                  llvmaddencodedtype_intern(tobjectdef(def).vmt_def,flags,encodedstr);
                  if ([lef_typedecl,lef_noimplicitderef]*flags=[]) then
                    encodedstr:=encodedstr+'**';
                end;
              odt_interfacecom_function,
              odt_interfacecom_property,
              odt_objcprotocol:
                begin
                  { opaque for now }
                  encodedstr:=encodedstr+'i8*'
                end;
              odt_helper:
                llvmaddencodedtype_intern(tobjectdef(def).extendeddef,flags,encodedstr);
              else
                internalerror(2013100601);
            end;
          undefineddef,
          errordef :
            internalerror(2013100604);
        else
          internalerror(2013100603);
        end;
      end;


    function llvmencodetypename(def: tdef): TSymStr;
      begin
        result:='';
        llvmaddencodedtype_intern(def,[],result);
      end;


    procedure llvmaddencodedtype(def: tdef; inaggregate: boolean; var encodedstr: TSymStr);
      var
        flags: tllvmencodeflags;
      begin
        if inaggregate then
          flags:=[lef_inaggregate]
        else
          flags:=[];
        llvmaddencodedtype_intern(def,flags,encodedstr);
      end;


    procedure llvmaddencodedabstractrecordtype(def: tabstractrecorddef; var encodedstr: TSymStr);
      var
        st: tllvmshadowsymtable;
        symdeflist: tfpobjectlist;
        i: longint;
        nopacked: boolean;
      begin
        st:=tabstractrecordsymtable(def.symtable).llvmst;
        symdeflist:=st.symdeflist;

        nopacked:=df_llvm_no_struct_packing in def.defoptions;
        if nopacked then
          encodedstr:=encodedstr+'{ '
        else
          encodedstr:=encodedstr+'<{ ';
        if symdeflist.count>0 then
          begin
            i:=0;
            if (def.typ=objectdef) and
               assigned(tobjectdef(def).childof) and
               is_class_or_interface_or_dispinterface(tllvmshadowsymtableentry(symdeflist[0]).def) then
              begin
                { insert the struct for the class rather than a pointer to the struct }
                if (tllvmshadowsymtableentry(symdeflist[0]).def.typ<>objectdef) then
                  internalerror(2008070601);
                llvmaddencodedtype_intern(tllvmshadowsymtableentry(symdeflist[0]).def,[lef_inaggregate,lef_noimplicitderef],encodedstr);
                inc(i);
              end;
            while i<symdeflist.count do
              begin
                if i<>0 then
                  encodedstr:=encodedstr+', ';
                llvmaddencodedtype_intern(tllvmshadowsymtableentry(symdeflist[i]).def,[lef_inaggregate],encodedstr);
                inc(i);
              end;
          end;
        if nopacked then
          encodedstr:=encodedstr+' }'
        else
          encodedstr:=encodedstr+' }>';
      end;


    procedure llvmextractvalueextinfo(paradef: tdef; var paralocdef: tdef; out signext: tllvmvalueextension);
      begin
        { implicit zero/sign extension for ABI compliance? (yes, if the size
          of a paraloc is larger than the size of the entire parameter) }
        if is_ordinal(paradef) and
           is_ordinal(paralocdef) and
           (paradef.size<paralocdef.size) then
          begin
            paralocdef:=paradef;
            if is_signed(paradef) then
              signext:=lve_signext
            else
              signext:=lve_zeroext
          end
        else
          signext:=lve_none;
      end;


    procedure llvmaddencodedparaloctype(hp: tparavarsym; proccalloption: tproccalloption; withparaname, withattributes: boolean; var first: boolean; var encodedstr: TSymStr);
      var
        para: PCGPara;
        paraloc: PCGParaLocation;
        side: tcallercallee;
        signext: tllvmvalueextension;
        usedef: tdef;
        firstloc: boolean;
      begin
        if (proccalloption in cdecl_pocalls) and
           is_array_of_const(hp.vardef) then
          begin
            if not first then
               encodedstr:=encodedstr+', '
            else
              first:=false;
            encodedstr:=encodedstr+'...';
            exit
          end;
        if not withparaname then
          side:=callerside
        else
          side:=calleeside;
        { don't add parameters that don't take up registers or stack space;
          clang doesn't either and some LLVM backends don't support them }
        if hp.paraloc[side].isempty then
          exit;
        para:=@hp.paraloc[side];
        paraloc:=para^.location;
        firstloc:=true;
        repeat
          usedef:=paraloc^.def;
          llvmextractvalueextinfo(hp.vardef,usedef,signext);
          { implicit zero/sign extension for ABI compliance? }
          if not first then
             encodedstr:=encodedstr+', ';
          llvmaddencodedtype_intern(usedef,[],encodedstr);
          { in case signextstr<>'', there should be only one paraloc -> no need
            to clear (reason: it means that the paraloc is larger than the
            original parameter) }
          if withattributes then
            encodedstr:=encodedstr+llvmvalueextension2str[signext];
          { sret: hidden pointer for structured function result }
          if vo_is_funcret in hp.varoptions then
            begin
              { "sret" is only valid for the firstparameter, while in FPC this
                can sometimes be second one (self comes before). In general,
                this is not a problem: we can just leave out sret, which means
                the result will be a bit less well optimised), but it is for
                AArch64: there, the sret parameter must be passed in a different
                register (-> paranr_result is smaller than paranr_self for that
                platform in symconst) }
{$ifdef aarch64}
              if not first then
                internalerror(2015101404);
{$endif aarch64}
              if withattributes then
                 if first then
                   encodedstr:=encodedstr+' sret noalias nocapture'
                 else
                   encodedstr:=encodedstr+' noalias nocapture';
            end
          else if not paramanager.push_addr_param(hp.varspez,hp.vardef,proccalloption) and
             llvmbyvalparaloc(paraloc) then
            begin
              if withattributes then
                begin
                  encodedstr:=encodedstr+'* byval';
                  if firstloc and
                     (para^.alignment<>std_param_align) then
                    begin
                      encodedstr:=encodedstr+' align '+tostr(para^.alignment);
                    end;
                end
              else
                encodedstr:=encodedstr+'*';
            end
          else if withattributes and
             paramanager.push_addr_param(hp.varspez,hp.vardef,proccalloption) then
            begin
              { it's not valid to take the address of a parameter and store it for
                use past the end of the function call (since the address can always
                be on the stack and become invalid later) }
              encodedstr:=encodedstr+' nocapture';
              { open array/array of const/variant array may be a valid pointer but empty }
              if not is_special_array(hp.vardef) and
                 { e.g. empty records }
                 (hp.vardef.size<>0) then
                begin
                  case hp.varspez of
                    vs_value,
                    vs_const:
                      begin
                        encodedstr:=encodedstr+' readonly dereferenceable('
                      end;
                    vs_var,
                    vs_out:
                      begin
                        { while normally these are not nil, it is technically possible
                          to pass nil via ptrtype(nil)^ }
                        encodedstr:=encodedstr+' dereferenceable_or_null(';
                      end;
                    vs_constref:
                      begin
                        encodedstr:=encodedstr+' readonly dereferenceable_or_null(';
                      end;
                    else
                      internalerror(2018120801);
                  end;
                  if hp.vardef.typ<>formaldef then
                    encodedstr:=encodedstr+tostr(hp.vardef.size)+')'
                  else
                    encodedstr:=encodedstr+'1)';
                end;
            end;
          if withparaname then
            begin
              if paraloc^.llvmloc.loc<>LOC_REFERENCE then
                internalerror(2014010803);
              encodedstr:=encodedstr+' '+llvmasmsymname(paraloc^.llvmloc.sym);
            end;
          paraloc:=paraloc^.next;
          firstloc:=false;
          first:=false;
        until not assigned(paraloc);
      end;


    function llvmencodeproctype(def: tabstractprocdef; const customname: TSymStr; pddecltype: tllvmprocdefdecltype): TSymStr;
      begin
        result:='';
        llvmaddencodedproctype(def,customname,pddecltype,result);
      end;


    procedure llvmaddencodedproctype(def: tabstractprocdef; const customname: TSymStr; pddecltype: tllvmprocdefdecltype; var encodedstr: TSymStr);
      var
        callingconv: ansistring;
        usedef: tdef;
        paranr: longint;
        hp: tparavarsym;
        signext: tllvmvalueextension;
        useside: tcallercallee;
        first: boolean;
      begin
        if not(pddecltype in [lpd_alias,lpd_procvar]) then
          begin
            callingconv:=llvm_callingconvention_name(def.proccalloption);
            if callingconv<>'' then
              encodedstr:=encodedstr+' '+callingconv;
          end;
        { when writing a definition, we have to write the parameter names, and
          those are only available on the callee side. In all other cases,
          we are at the callerside }
        if pddecltype=lpd_def then
          useside:=calleeside
        else
          useside:=callerside;
        def.init_paraloc_info(useside);
        first:=true;
        { function result (return-by-ref is handled explicitly) }
        if not paramanager.ret_in_param(def.returndef,def) or
           def.generate_safecall_wrapper then
          begin
            if not def.generate_safecall_wrapper then
              usedef:=llvmgetcgparadef(def.funcretloc[useside],false,useside)
            else
              usedef:=ossinttype;
            llvmextractvalueextinfo(def.returndef,usedef,signext);
            { specifying result sign extention information for an alias causes
              an error for some reason }
            if pddecltype in [lpd_decl,lpd_def] then
              encodedstr:=encodedstr+llvmvalueextension2str[signext];
            encodedstr:=encodedstr+' ';
            llvmaddencodedtype_intern(usedef,[],encodedstr);
          end
        else
          begin
            encodedstr:=encodedstr+' ';
            llvmaddencodedtype(voidtype,false,encodedstr);
          end;
        encodedstr:=encodedstr+' ';
        { add procname? }
        if (pddecltype in [lpd_decl,lpd_def]) and
           (def.typ=procdef) then
          if customname='' then
            encodedstr:=encodedstr+llvmmangledname(tprocdef(def).mangledname)
          else
            encodedstr:=encodedstr+llvmmangledname(customname);
        encodedstr:=encodedstr+'(';
        { parameters }
        first:=true;
        for paranr:=0 to def.paras.count-1 do
          begin
            hp:=tparavarsym(def.paras[paranr]);
            llvmaddencodedparaloctype(hp,def.proccalloption,pddecltype in [lpd_def],not(pddecltype in [lpd_procvar,lpd_alias]),first,encodedstr);
          end;
        if po_varargs in def.procoptions then
          begin
            if not first then
              encodedstr:=encodedstr+', ';
            encodedstr:=encodedstr+'...';
          end;
        encodedstr:=encodedstr+')'
      end;


    function llvmgettemprecorddef(const fieldtypes: array of tdef; packrecords, recordalignmin: shortint): trecorddef;
      var
        i: longint;
        res: PHashSetItem;
        oldsymtablestack: tsymtablestack;
        hrecst: trecordsymtable;
        hdef: tdef;
        hrecdef: trecorddef;
        sym: tfieldvarsym;
        typename: string;
      begin
        typename:=internaltypeprefixName[itp_llvmstruct];
        for i:=low(fieldtypes) to high(fieldtypes) do
          begin
            hdef:=fieldtypes[i];
            case hdef.typ of
              orddef:
                case torddef(hdef).ordtype of
                  s8bit,
                  u8bit,
                  pasbool1,
                  pasbool8:
                    typename:=typename+'i8';
                  s16bit,
                  u16bit:
                    typename:=typename+'i16';
                  s32bit,
                  u32bit:
                    typename:=typename+'i32';
                  s64bit,
                  u64bit:
                    typename:=typename+'i64';
                  customint:
                    typename:=typename+'i'+tostr(torddef(hdef).packedbitsize);
                  else
                    { other types should not appear currently, add as needed }
                    internalerror(2014012001);
                end;
              floatdef:
                case tfloatdef(hdef).floattype of
                  s32real:
                    typename:=typename+'f32';
                  s64real:
                    typename:=typename+'f64';
                  else
                    { other types should not appear currently, add as needed }
                    internalerror(2014012008);
                end;
              else
                typename:=typename+'d'+hdef.unique_id_str;
            end;
          end;
        if not assigned(current_module) then
          internalerror(2014012002);
        res:=current_module.llvmdefs.FindOrAdd(@typename[1],length(typename));
        if not assigned(res^.Data) then
          begin
            res^.Data:=crecorddef.create_global_internal(typename,packrecords,
              recordalignmin);
            for i:=low(fieldtypes) to high(fieldtypes) do
              trecorddef(res^.Data).add_field_by_def('F'+tostr(i),fieldtypes[i]);
          end;
        trecordsymtable(trecorddef(res^.Data).symtable).addalignmentpadding;
        result:=trecorddef(res^.Data);
      end;


    function llvmgetcgparadef(const cgpara: tcgpara; beforevalueext: boolean; callercallee: tcallercallee): tdef;
      var
        retdeflist: array[0..9] of tdef;
        retloc: pcgparalocation;
        usedef: tdef;
        valueext: tllvmvalueextension;
        paraslots,
        i: longint;
        sizeleft: asizeint;
      begin
        { single location }
        if not assigned(cgpara.location^.next) then
          begin
            { def of the location, except in case of zero/sign-extension and
              zero-sized records }
            if not is_special_array(cgpara.def) and
               (cgpara.def.size=0) then
              usedef:=cgpara.def
            else
              usedef:=cgpara.location^.def;
            if beforevalueext then
              llvmextractvalueextinfo(cgpara.def,usedef,valueext);
            { comp and currency are handled by the x87 in this case. They cannot
              be represented directly in llvm, and llvmdef translates them into
              i64 (since that's their storage size and internally they also are
              int64). Solve this by changing the type to s80real in the
              returndef/parameter declaration. }
            if (usedef.typ=floatdef) and
               (tfloatdef(usedef).floattype in [s64comp,s64currency]) then
              usedef:=s80floattype;
            result:=usedef;
            exit
          end;
        { multiple locations -> create temp record }
        retloc:=cgpara.location;
        i:=0;
        sizeleft:=cgpara.Def.size;
        repeat
          if i>high(retdeflist) then
            internalerror(2016121801);
          if assigned(retloc^.next) then
            begin
              retdeflist[i]:=retloc^.def;
              dec(sizeleft,retloc^.def.size);
            end
          { on the callerside, "byval" parameter locations have the implicit
            pointer in their type -> remove if we wish to create a record
            containing all actual parameter data }
          else if (callercallee=callerside) and
             not retloc^.llvmvalueloc then
            begin
              if retloc^.def.typ<>pointerdef then
                internalerror(2019020201);
              retdeflist[i]:=tpointerdef(retloc^.def).pointeddef
            end
          else if retloc^.def.size<>sizeleft then
            begin
              case sizeleft of
                1:
                  retdeflist[i]:=u8inttype;
                2:
                  retdeflist[i]:=u16inttype;
                3:
                  retdeflist[i]:=u24inttype;
                4:
                  retdeflist[i]:=u32inttype;
                5:
                  retdeflist[i]:=u40inttype;
                6:
                  retdeflist[i]:=u48inttype;
                7:
                  retdeflist[i]:=u56inttype;
                else
                  retdeflist[i]:=retloc^.def;
              end
            end
          else
            begin
              if retloc^.def.typ<>floatdef then
                begin
                  paraslots:=sizeleft div cgpara.Alignment;
                  if (paraslots>1) and
                     ((paraslots*cgpara.Alignment)=sizeleft) then
                    retdeflist[i]:=carraydef.getreusable(cgsize_orddef(int_cgsize(cgpara.Alignment)),paraslots)
                  else
                    retdeflist[i]:=retloc^.def;
                end
              else
                retdeflist[i]:=retloc^.def;
            end;
          inc(i);
          retloc:=retloc^.next;
        until not assigned(retloc);
        result:=llvmgettemprecorddef(slice(retdeflist,i),C_alignment,
          targetinfos[target_info.system]^.alignment.recordalignmin);
        include(result.defoptions,df_llvm_no_struct_packing);
      end;


    function llvmencodetypedecl(def: tdef): TSymStr;
      begin
        result:='';
        llvmaddencodedtype_intern(def,[lef_typedecl],result);
      end;


end.
