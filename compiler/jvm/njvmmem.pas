{
    Copyright (c) 2011 by Jonas Maebe

    Generate JVM byetcode for in memory related nodes

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
unit njvmmem;

{$i fpcdefs.inc}

interface

    uses
      globtype,
      cgbase,cpubase,
      node,nmem,ncgmem,ncgnstmm;

    type
       tjvmaddrnode = class(tcgaddrnode)
        protected
         function isrefparaload: boolean;
         function isarrayele0load: boolean;
         function isdererence: boolean;
        public
         function pass_typecheck: tnode; override;
         procedure pass_generate_code; override;
       end;

       tjvmderefnode = class(tcgderefnode)
          function pass_typecheck: tnode; override;
          procedure pass_generate_code; override;
       end;

       tjvmsubscriptnode = class(tcgsubscriptnode)
        protected
         function handle_platform_subscript: boolean; override;
       end;

       tjvmloadvmtaddrnode = class(tcgloadvmtaddrnode)
         function pass_1: tnode; override;
         procedure pass_generate_code; override;
       end;

       tjvmvecnode = class(tcgvecnode)
        protected
          function gen_array_rangecheck: tnode; override;
        public
         function pass_1: tnode; override;
         procedure pass_generate_code;override;
       end;

implementation

    uses
      systems,globals,procinfo,
      cutils,verbose,constexp,
      aasmbase,
      symconst,symtype,symtable,symsym,symdef,symcpu,defutil,jvmdef,
      htypechk,paramgr,
      nadd,ncal,ncnv,ncon,nld,nutils,
      pass_1,njvmcon,
      aasmdata,aasmcpu,pass_2,
      cgutils,hlcgobj,hlcgcpu;

{*****************************************************************************
                              TJVMDEREFNODE
*****************************************************************************}

    function tjvmderefnode.pass_typecheck: tnode;
      begin
        result:=inherited pass_typecheck;
        if assigned(result) then
          exit;
        { don't allow dereferencing untyped pointers, because how this has to
          be done depends on whether it's a pointer to an implicit pointer type
          or not }
        if is_voidpointer(left.resultdef) then
          CGMessage(parser_e_illegal_expression);
      end;


    procedure tjvmderefnode.pass_generate_code;
      var
        implicitptr: boolean;
      begin
        secondpass(left);
        implicitptr:=jvmimplicitpointertype(resultdef);
        if implicitptr then
          begin
            { this is basically a typecast: the left node is a regular
              'pointer', and we typecast it to an implicit pointer }
            location_copy(location,left.location);
            { these implicit pointer types (records, sets, shortstrings, ...)
              cannot be located in registers on native targets (since
              they're not pointers there) -> force into memory to avoid
              confusing the compiler; this can happen when typecasting a
              Java class type into a pshortstring and then dereferencing etc
            }
            if location.loc in [LOC_REGISTER,LOC_CREGISTER] then
              hlcg.location_force_mem(current_asmdata.CurrAsmList,location,left.resultdef);
          end
        else
          begin
            { these are always arrays (used internally for pointers to var
              parameters stored in nestedfpstructs, and by programmers for any
              kind of pointers) }
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
            location_reset_ref(location,LOC_REFERENCE,def_cgsize(resultdef),4,[]);
            reference_reset_base(location.reference,left.location.register,0,ctempposinvalid,4,[]);
            location.reference.arrayreftype:=art_indexconst;
            if (left.nodetype<>addrn) and
               not(resultdef.typ in [orddef,floatdef]) and
               not is_voidpointer(resultdef) and
               ((resultdef.typ<>objectdef) or
                (find_real_class_definition(tobjectdef(resultdef),false)<>java_jlobject)) then
              location.reference.checkcast:=true;
          end
      end;


{*****************************************************************************
                            TJVMSUBSCRIPTNODE
*****************************************************************************}

    function tjvmsubscriptnode.handle_platform_subscript: boolean;
      begin
        result:=false;
        if is_java_class_or_interface(left.resultdef) or
           (left.resultdef.typ=recorddef) then
          begin
            if (location.loc<>LOC_REFERENCE) or
               (location.reference.index<>NR_NO) or
               assigned(location.reference.symbol) then
              internalerror(2011011301);
            location.reference.symbol:=current_asmdata.RefAsmSymbol(vs.mangledname,AT_METADATA);
            result:=true;
          end
      end;


{*****************************************************************************
                              TJVMADDRNODE
*****************************************************************************}

    function tjvmaddrnode.isrefparaload: boolean;
      begin
        result:=
         (left.nodetype=loadn) and
         (tloadnode(left).symtableentry.typ=paravarsym) and
         paramanager.push_copyout_param(tparavarsym(tloadnode(left).symtableentry).varspez,
           left.resultdef,
           tabstractprocdef(tloadnode(left).symtableentry.owner.defowner).proccalloption);
      end;


    function tjvmaddrnode.isarrayele0load: boolean;
      begin
        result:=
          (left.nodetype=vecn) and
          (tvecnode(left).left.resultdef.typ=arraydef) and
          (tvecnode(left).right.nodetype=ordconstn) and
          (tordconstnode(tvecnode(left).right).value=tarraydef(tvecnode(left).left.resultdef).lowrange);
      end;


    function tjvmaddrnode.isdererence: boolean;
      begin
        result:=
          (left.nodetype=derefn);
      end;


    function tjvmaddrnode.pass_typecheck: tnode;
      var
        fsym: tsym;
      begin
        result:=nil;
        typecheckpass(left);
        if codegenerror then
         exit;

        make_not_regable(left,[ra_addr_regable,ra_addr_taken]);

        { in TP/Delphi, @procvar = contents of procvar and @@procvar =
          address of procvar. In case of a procedure of object, this works
          by letting the first addrnode typecast the procvar into a tmethod
          record followed by subscripting its "code" field (= first field),
          and if there's a second addrnode then it takes the address of
          this code field (which is hence also the address of the procvar).

          In Java, such ugly hacks don't work -> replace first addrnode
          with getting procvar.method.code, and second addrnode with
          the class for procedure of object}
        if not(nf_internal in flags) and
           ((m_tp_procvar in current_settings.modeswitches) or
            (m_mac_procvar in current_settings.modeswitches)) and
           (((left.nodetype=addrn) and
             (taddrnode(left).left.resultdef.typ=procvardef)) or
            (left.resultdef.typ=procvardef)) then
          begin
            if (left.nodetype=addrn) and
               (taddrnode(left).left.resultdef.typ=procvardef) then
              begin
                { double address -> pointer that is the address of the
                  procvardef (don't allow for non-object procvars, as they
                  aren't implicitpointerdefs) }
                if not jvmimplicitpointertype(taddrnode(left).left.resultdef) then
                  CGMessage(parser_e_illegal_expression)
                else
                  begin
                    { an internal address node will observe "normal" address
                      operator semantics (= take the actual address!) }
                    result:=caddrnode.create_internal(taddrnode(left).left);
                    result:=ctypeconvnode.create_explicit(result,tcpuprocvardef(taddrnode(left).left.resultdef).classdef);
                    taddrnode(left).left:=nil;
                 end;
              end
            else if left.resultdef.typ=procvardef then
              begin
                if not tprocvardef(left.resultdef).is_addressonly then
                  begin
                    { the "code" field from the procvar }
                    result:=caddrnode.create_internal(left);
                    result:=ctypeconvnode.create_explicit(result,tcpuprocvardef(left.resultdef).classdef);
                    { procvarclass.method }
                    fsym:=search_struct_member(tcpuprocvardef(left.resultdef).classdef,'METHOD');
                    if not assigned(fsym) or
                       (fsym.typ<>fieldvarsym) then
                      internalerror(2011072501);
                    result:=csubscriptnode.create(fsym,result);
                    { procvarclass.method.code }
                    fsym:=search_struct_member(trecorddef(tfieldvarsym(fsym).vardef),'CODE');
                    if not assigned(fsym) or
                       (fsym.typ<>fieldvarsym) then
                      internalerror(2011072502);
                    result:=csubscriptnode.create(fsym,result);
                    left:=nil
                  end
                else
                  { convert contents to plain pointer }
                  begin
                    result:=ctypeconvnode.create_explicit(left,java_jlobject);
                    include(result.flags,nf_load_procvar);
                    left:=nil;
                  end;
              end
            else
              internalerror(2011072503);
          end
        else if (left.resultdef.typ=procdef) then
          begin
            result:=inherited;
            exit;
          end
        else
          begin
            if not jvmimplicitpointertype(left.resultdef) then
              begin
                { allow taking the address of a copy-out parameter (it's an
                  array reference), of the first element of an array and of a
                  pointer derefence }
                if not isrefparaload and
                   not isarrayele0load and
                   not isdererence then
                  begin
                    CGMessage(parser_e_illegal_expression);
                    exit
                  end;
              end;
            result:=inherited;
          end;
      end;


    procedure tjvmaddrnode.pass_generate_code;
      var
        implicitptr: boolean;
      begin
        secondpass(left);
        implicitptr:=jvmimplicitpointertype(left.resultdef);
        if implicitptr then
          { this is basically a typecast: the left node is an implicit
            pointer, and we typecast it to a regular 'pointer'
            (java.lang.Object) }
          location_copy(location,left.location)
        else
          begin
            { these are always arrays (used internally for pointers to var
              parameters stored in nestedfpstructs) -> get base pointer to
              array }
            if (left.location.loc<>LOC_REFERENCE) or
               (left.location.reference.arrayreftype<>art_indexconst) or
               (left.location.reference.base=NR_NO) or
               (left.location.reference.indexoffset<>0) or
               assigned(left.location.reference.symbol) then
              internalerror(2011060701);
            location_reset(location,LOC_REGISTER,OS_ADDR);
            location.register:=left.location.reference.base;
          end;
      end;

{*****************************************************************************
                         TJVMLOADVMTADDRNODE
*****************************************************************************}

    function tjvmloadvmtaddrnode.pass_1: tnode;
      var
        vs: tsym;
      begin
        result:=nil;
        if is_javaclass(left.resultdef) and
           (left.nodetype<>typen) and
           (left.resultdef.typ<>classrefdef) then
          begin
            { call java.lang.Object.getClass() }
            vs:=search_struct_member(tobjectdef(left.resultdef),'GETCLASS');
            if not assigned(vs) or
               (tsym(vs).typ<>procsym) then
              internalerror(2011041903);
            result:=ccallnode.create(nil,tprocsym(vs),vs.owner,left,[],nil);
            inserttypeconv_explicit(result,resultdef);
            { reused }
            left:=nil;
          end;
      end;


    procedure tjvmloadvmtaddrnode.pass_generate_code;
      begin
        current_asmdata.CurrAsmList.concat(taicpu.op_sym(a_ldc,current_asmdata.RefAsmSymbol(
          tabstractrecorddef(tclassrefdef(resultdef).pointeddef).jvm_full_typename(true),AT_METADATA)));
        thlcgjvm(hlcg).incstack(current_asmdata.CurrAsmList,1);
        location_reset(location,LOC_REGISTER,OS_ADDR);
        location.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,resultdef);
        thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,resultdef,location.register);
      end;


{*****************************************************************************
                             TJVMVECNODE
*****************************************************************************}

    function tjvmvecnode.gen_array_rangecheck: tnode;
      begin
        { JVM does the range checking for us }
        result:=nil;
      end;


    function tjvmvecnode.pass_1: tnode;
      var
        psym: tsym;
        stringclass: tdef;
      begin
        if (left.resultdef.typ=stringdef) then
          begin
            case tstringdef(left.resultdef).stringtype of
              st_ansistring:
                stringclass:=java_ansistring;
              st_unicodestring,
              st_widestring:
                stringclass:=java_jlstring;
              st_shortstring:
                begin
                  stringclass:=java_shortstring;
                  left:=caddrnode.create_internal(left);
                  { avoid useless typecheck when casting to shortstringclass }
                  include(taddrnode(left).addrnodeflags,anf_typedaddr);
                end
              else
                internalerror(2011052407);
            end;
            psym:=search_struct_member(tabstractrecorddef(stringclass),'CHARAT');
            if not assigned(psym) or
               (psym.typ<>procsym) then
              internalerror(2011031502);
            { Pascal strings are 1-based, Java strings 0-based }
            result:=ccallnode.create(ccallparanode.create(
              caddnode.create(subn,right,genintconstnode(1)),nil),tprocsym(psym),
              psym.owner,ctypeconvnode.create_explicit(left,stringclass),[],nil);
            left:=nil;
            right:=nil;
            exit;
          end
        else
          begin
            { keep indices that are enum constants that way, rather than
              transforming them into a load of the class instance that
              represents this constant (since we then would have to extract
              the int constant value again at run time anyway) }
            if right.nodetype=ordconstn then
              tjvmordconstnode(right).enumconstok:=true;
            result:=inherited;
          end;
      end;


    procedure tjvmvecnode.pass_generate_code;
      var
        psym: tsym;
        newsize: tcgsize;
      begin
        if left.resultdef.typ=stringdef then
          internalerror(2011052702);

        { This routine is not used for Strings, as they are a class type and
          you have to use charAt() there to load a character (and you cannot
          change characters; you have to create a new string in that case)

          As far as arrays are concerned: we have to create a trefererence
          with arrayreftype in [art_indexreg,art_indexref], and ref.base =
          pointer to the array (i.e., left.location.register) }
        secondpass(left);
        newsize:=def_cgsize(resultdef);
        if left.location.loc=LOC_CREFERENCE then
          location_reset_ref(location,LOC_CREFERENCE,newsize,left.location.reference.alignment,left.location.reference.volatility)
        else
          location_reset_ref(location,LOC_REFERENCE,newsize,left.location.reference.alignment,left.location.reference.volatility);
        { don't use left.resultdef, because it may be an open or regular array,
          and then asking for the size doesn't make any sense }
        hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,java_jlobject,java_jlobject,true);
        location.reference.base:=left.location.register;
        secondpass(right);
        if (right.expectloc=LOC_JUMP)<>
           (right.location.loc=LOC_JUMP) then
          internalerror(2011090501);

        { simplify index location if necessary, since array references support
          an index in memory, but not an another array index }
        if (right.location.loc=LOC_JUMP) or
           ((right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and
            (right.location.reference.arrayreftype<>art_none)) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);

        { replace enum class instance with the corresponding integer value }
        if (right.resultdef.typ=enumdef) then
          begin
           if (right.location.loc<>LOC_CONSTANT) then
             begin
               psym:=search_struct_member(tcpuenumdef(tenumdef(right.resultdef).getbasedef).classdef,'FPCORDINAL');
               if not assigned(psym) or
                  (psym.typ<>procsym) or
                  (tprocsym(psym).ProcdefList.count<>1) then
                 internalerror(2011062607);
               thlcgjvm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,right.resultdef,right.location);
               hlcg.a_call_name(current_asmdata.CurrAsmList,tprocdef(tprocsym(psym).procdeflist[0]),tprocdef(tprocsym(psym).procdeflist[0]).mangledname,[],nil,false);
               { call replaces self parameter with longint result -> no stack
                 height change }
               location_reset(right.location,LOC_REGISTER,OS_S32);
               right.location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,s32inttype);
               thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,s32inttype,right.location.register);
             end;
           { always force to integer location, because enums are handled as
             object instances (since that's what they are in Java) }
           right.resultdef:=s32inttype;
           right.location.size:=OS_S32;
          end
        else if (right.location.loc<>LOC_CONSTANT) and
                ((right.resultdef.typ<>orddef) or
                 (torddef(right.resultdef).ordtype<>s32bit)) then
          begin
            { Java array indices are always 32 bit signed integers }
            hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,s32inttype,true);
            right.resultdef:=s32inttype;
          end;

        { adjust index if necessary }
        if not is_special_array(left.resultdef) and
           (tarraydef(left.resultdef).lowrange<>0) and
           (right.location.loc<>LOC_CONSTANT) then
          begin
            thlcgjvm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,right.resultdef,right.location);
            thlcgjvm(hlcg).a_op_const_stack(current_asmdata.CurrAsmList,OP_SUB,right.resultdef,tarraydef(left.resultdef).lowrange);
            location_reset(right.location,LOC_REGISTER,def_cgsize(right.resultdef));
            right.location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,right.resultdef);
            thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,right.resultdef,right.location.register);
          end;

        { create array reference }
        case right.location.loc of
          LOC_REGISTER,LOC_CREGISTER:
            begin
              location.reference.arrayreftype:=art_indexreg;
              location.reference.index:=right.location.register;
            end;
          LOC_REFERENCE,LOC_CREFERENCE:
            begin
              location.reference.arrayreftype:=art_indexref;
              location.reference.indexbase:=right.location.reference.base;
              location.reference.indexsymbol:=right.location.reference.symbol;
              location.reference.indexoffset:=right.location.reference.offset;
            end;
          LOC_CONSTANT:
            begin
              location.reference.arrayreftype:=art_indexconst;
              location.reference.indexoffset:=right.location.value-tarraydef(left.resultdef).lowrange;
            end
          else
            internalerror(2011012002);
        end;
      end;


begin
   cderefnode:=tjvmderefnode;
   csubscriptnode:=tjvmsubscriptnode;
   caddrnode:=tjvmaddrnode;
   cvecnode:=tjvmvecnode;
   cloadvmtaddrnode:=tjvmloadvmtaddrnode;
end.
