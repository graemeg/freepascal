{
    Copyright (c) 1998-2011 by Florian Klaempfl and Jonas Maebe

    Generate JVM inline nodes

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
unit njvminl;

{$i fpcdefs.inc}

interface

    uses
       cpubase,
       node,ninl,ncginl;

    type
       tjvminlinenode = class(tcginlinenode)
         protected
          function typecheck_length(var handled: boolean): tnode;
          function typecheck_high(var handled: boolean): tnode;
          function typecheck_new(var handled: boolean): tnode;

          function first_copy: tnode; override;
          function first_assigned: tnode; override;
          function first_get_frame: tnode; override;

          function first_box: tnode; override;
          function first_unbox: tnode; override;

          function first_setlength_array: tnode;
         public
          { typecheck override to intercept handling }
          function pass_typecheck: tnode; override;

          { first pass override
            so that the code generator will actually generate
            these nodes.
          }
          function first_sqr_real: tnode; override;
          function first_trunc_real: tnode; override;
          function first_new: tnode; override;
          function first_IncludeExclude: tnode; override;
          function first_setlength: tnode; override;
          function first_length: tnode; override;

          procedure second_length; override;
          procedure second_sqr_real; override;
          procedure second_trunc_real; override;
          procedure second_new; override;
          procedure second_setlength; override;
       protected
          procedure load_fpu_location;
       end;

implementation

    uses
      cutils,globals,verbose,globtype,constexp,fmodule,compinnr,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      symtype,symconst,symdef,symsym,symcpu,symtable,jvmdef,
      defutil,
      nadd,nbas,ncon,ncnv,nmat,nmem,ncal,nld,nflw,nutils,
      cgbase,pass_1,pass_2,
      cpuinfo,ncgutil,
      cgutils,hlcgobj,hlcgcpu;


{*****************************************************************************
                              tjvminlinenode
*****************************************************************************}

    function tjvminlinenode.typecheck_length(var handled: boolean): tnode;
      begin
        result:=nil;
        typecheckpass(left);
        if is_open_array(left.resultdef) or
           is_dynamic_array(left.resultdef) or
           is_array_of_const(left.resultdef) then
          begin
            resultdef:=s32inttype;
            handled:=true;
          end;
      end;


    function tjvminlinenode.typecheck_high(var handled: boolean): tnode;
      begin
        result:=nil;
        typecheckpass(left);
        if is_dynamic_array(left.resultdef) or
           is_open_array(left.resultdef) or
           is_array_of_const(left.resultdef) then
          begin
            { replace with pred(length(arr)) }
            result:=cinlinenode.create(in_pred_x,false,
              cinlinenode.create(in_length_x,false,left));
            left:=nil;
            handled:=true;
          end;
      end;


    function tjvminlinenode.typecheck_new(var handled: boolean): tnode;
      var
        para: tcallparanode;
        elemdef: tdef;
      begin
        result:=nil;
        { normally never exists; used by the JVM backend to create new
          arrays because it requires special opcodes }
        tcallparanode(left).get_paratype;
        if is_dynamic_array(left.resultdef) then
          begin
            para:=tcallparanode(left);
            { need at least one extra parameter in addition to the
              array }
            if not assigned(para.right) then
              internalerror(2011012206);
            elemdef:=tarraydef(left.resultdef).elementdef;
            while elemdef.typ=arraydef do
              begin
                { if we have less length specifiers than dimensions, make
                  the last array an array of length 0 }
                if not assigned(para.right) then
                  begin
                    para.right:=ccallparanode.create(
                      cordconstnode.create(0,s32inttype,false),nil);
                    tcallparanode(para.right).get_paratype;
                    break;
                  end
                else
                  begin
                    inserttypeconv(tcallparanode(para.right).left,s32inttype);
                    tcallparanode(para.right).get_paratype;
                  end;
                para:=tcallparanode(para.right);
                elemdef:=tarraydef(elemdef).elementdef;
              end;
            resultdef:=left.resultdef;
            handled:=true;
          end;
      end;


    function tjvminlinenode.first_copy: tnode;
      var
        ppn: tcallparanode;
        arr, len, start: tnode;
        eledef: tdef;
        counter, ndims: longint;
        finaltype: char;
      begin
        if is_dynamic_array(resultdef) then
          begin
            ppn:=tcallparanode(left);
            counter:=1;
            while assigned(ppn.right) do
              begin
                inc(counter);
                ppn:=tcallparanode(ppn.right);
              end;
            if (counter=3) then
              begin
                len:=tcallparanode(left).left;
                tcallparanode(left).left:=nil;
                start:=tcallparanode(tcallparanode(left).right).left;
                tcallparanode(tcallparanode(left).right).left:=nil;
                { free the original start/len paras and remove them }
                ppn:=tcallparanode(left);
                left:=tcallparanode(tcallparanode(left).right).right;
                tcallparanode(ppn.right).right:=nil;
                ppn.free;
              end
            else
              begin
                { use special -1,-1 argument to copy the whole array }
                len:=genintconstnode(-1);
                start:=genintconstnode(-1);
              end;
            { currently there is one parameter left: the array itself }
            arr:=tcallparanode(left).left;
            tcallparanode(left).left:=nil;
            { in case it's a dynamic array of static arrays, get the dimensions
              of the static array components }
            eledef:=tarraydef(resultdef).elementdef;
            ndims:=1;
            while (eledef.typ=arraydef) and
                  not is_dynamic_array(eledef) do
              begin
                inc(ndims);
                eledef:=tarraydef(eledef).elementdef;
              end;
            { get the final element kind }
            finaltype:=jvmarrtype_setlength(eledef);
            { construct the call to
                fpc_dynarray_copy(src: JLObject; start, len: longint; ndim: longint; eletype: jchar) }
            result:=ccallnode.createintern('FPC_DYNARRAY_COPY',
              ccallparanode.create(cordconstnode.create(ord(finaltype),cwidechartype,false),
                ccallparanode.create(genintconstnode(ndims),
                  ccallparanode.create(len,
                    ccallparanode.create(start,
                      ccallparanode.create(ctypeconvnode.create_explicit(arr,java_jlobject),nil)
                    )
                  )
                )
              )
            );
            inserttypeconv_explicit(result,resultdef);
          end
        else
          result:=inherited first_copy;
      end;


    function tjvminlinenode.first_assigned: tnode;
      begin
        { on the JVM target, empty arrays can also be <> nil but have length 0
          instead. Since assigned(dynarray) is only used to determine whether
          the length is <> 0 on other targets, replace this expression here }
        if is_dynamic_array(tcallparanode(left).left.resultdef) then
          begin
            result:=caddnode.create(unequaln,cinlinenode.create(
              in_length_x,false,tcallparanode(left).left),genintconstnode(0));
            tcallparanode(left).left:=nil;
          end
        else
          result:=inherited;
      end;


    function tjvminlinenode.first_get_frame: tnode;
      begin
        { no frame pointer on the JVM target }
        result:=cnilnode.create;
      end;


    function tjvminlinenode.first_box: tnode;
      var
        boxdef,
        boxparadef: tdef;
      begin
        { get class wrapper type }
        jvmgetboxtype(left.resultdef,boxdef,boxparadef,true);
        { created wrapped instance }
        inserttypeconv_explicit(tcallparanode(left).left,boxparadef);
        result:=ccallnode.createinternmethod(
          cloadvmtaddrnode.create(ctypenode.create(tobjectdef(boxdef))),'CREATE',left);
        { reused }
        left:=nil;
      end;


    function tjvminlinenode.first_unbox: tnode;
      var
        val: tnode;
        boxdef,
        boxparadef: tdef;
      begin
        jvmgetboxtype(resultdef,boxdef,boxparadef,true);
        val:=tcallparanode(tcallparanode(left).right).left;
        tcallparanode(tcallparanode(left).right).left:=nil;
        { typecast to the boxing type }
        val:=ctypeconvnode.create_explicit(val,boxdef);
        { call the unboxing method }
        val:=ccallnode.createinternmethod(val,jvmgetunboxmethod(resultdef),nil);
        { add type conversion for shortint -> byte etc }
        inserttypeconv_explicit(val,resultdef);
        result:=val;
      end;


    function tjvminlinenode.pass_typecheck: tnode;
      var
        handled: boolean;
      begin
         result:=nil;
         handled:=false;
         case inlinenumber of
           in_length_x:
             begin
               result:=typecheck_length(handled);
             end;
           in_high_x:
             begin
               result:=typecheck_high(handled);
             end;
           in_new_x:
             begin
               result:=typecheck_new(handled);
             end;
           in_sizeof_x:
             begin
               { can't get the size of the data of a class/object }
               if left.resultdef.typ in [objectdef,classrefdef] then
                 Message(parser_e_illegal_expression);
             end;
           else
             ;
         end;
        if not handled then
          result:=inherited pass_typecheck;
      end;


(*
    function tjvminlinenode.first_sqrt_real : tnode;
      begin
        if (current_settings.cputype >= cpu_PPC970) then
          begin
            expectloc:=LOC_FPUREGISTER;
            first_sqrt_real := nil;
          end
        else
          result:=inherited first_sqrt_real;
      end;
*)

     function tjvminlinenode.first_sqr_real : tnode;
      begin
        expectloc:=LOC_FPUREGISTER;
        first_sqr_real:=nil;
      end;


     function tjvminlinenode.first_trunc_real : tnode;
      begin
        expectloc:=LOC_REGISTER;
        first_trunc_real:=nil;
      end;


    function tjvminlinenode.first_new: tnode;
      begin
        { skip the array; it's a type node }
        tcallparanode(tcallparanode(left).right).firstcallparan;
        expectloc:=LOC_REGISTER;
        result:=nil;
      end;


    function tjvminlinenode.first_IncludeExclude: tnode;
      var
        setpara: tnode;
        valuepara: tcallparanode;
        seteledef: tdef;
        procname: string[6];
      begin
        setpara:=tcallparanode(left).left;
        tcallparanode(left).left:=nil;
        valuepara:=tcallparanode(tcallparanode(left).right);
        tcallparanode(left).right:=nil;
        seteledef:=tsetdef(setpara.resultdef).elementdef;
        setpara:=caddrnode.create_internal(setpara);
        include(taddrnode(setpara).addrnodeflags,anf_typedaddr);
        if seteledef.typ=enumdef then
          begin
            inserttypeconv_explicit(setpara,java_juenumset);
            inserttypeconv_explicit(valuepara.left,tcpuenumdef(tenumdef(seteledef).getbasedef).classdef);
          end
        else
          begin
            inserttypeconv_explicit(setpara,java_jubitset);
            inserttypeconv_explicit(valuepara.left,s32inttype);
          end;
        if inlinenumber=in_include_x_y then
          procname:='ADD'
        else
          procname:='REMOVE';
        result:=ccallnode.createinternmethod(setpara,procname,valuepara);
      end;


    function tjvminlinenode.first_setlength_array: tnode;
      var
        assignmenttarget,
        ppn,
        newparas: tnode;
        newnode: tnode;
        eledef,
        objarraydef: tdef;
        ndims: longint;
        finaltype: char;
        setlenroutine: string;
        lefttemp: ttempcreatenode;
        newblock: tblocknode;
        newstatement: tstatementnode;
      begin
        { first parameter is the array, the rest are the dimensions }
        newparas:=tcallparanode(left).right;
        tcallparanode(left).right:=nil;
        { count the number of specified dimensions, and determine the type of
          the final one }
        ppn:=newparas;
        eledef:=tarraydef(left.resultdef).elementdef;
        { ppn already points to the first dimension }
        ndims:=1;
        while assigned(tcallparanode(ppn).right) do
          begin
            inc(ndims);
            eledef:=tarraydef(eledef).elementdef;
            ppn:=tcallparanode(ppn).right;
          end;
        { in case it's a dynamic array of static arrays, we must also allocate
          the static arrays! }
        while (eledef.typ=arraydef) and
              not is_dynamic_array(eledef) do
          begin
            inc(ndims);
            tcallparanode(ppn).right:=
              ccallparanode.create(
                genintconstnode(tarraydef(eledef).elecount),nil);
            ppn:=tcallparanode(ppn).right;
            eledef:=tarraydef(eledef).elementdef;
          end;
        { prepend type parameter for the array }
        newparas:=ccallparanode.create(ctypenode.create(left.resultdef),newparas);
        ttypenode(tcallparanode(newparas).left).allowed:=true;
        { node to create the new array }
        newnode:=cinlinenode.create(in_new_x,false,newparas);
        { Common parameters for setlength helper }
        { start with org (save assignmenttarget itself to assign the result back to) }
        { store left into a temp in case it may contain a function call
          (which must not be evaluated twice) }
        newblock:=nil;
        newstatement:=nil;
        lefttemp:=maybereplacewithtempref(tcallparanode(left).left,newblock,newstatement,tcallparanode(left).left.resultdef.size,false);
        if assigned(lefttemp) then
          begin
            assignmenttarget:=ctemprefnode.create(lefttemp);
            typecheckpass(tnode(assignmenttarget));
          end
        else
          assignmenttarget:=tcallparanode(left).left.getcopy;
        newparas:=left;
        left:=nil;
        finaltype:=jvmarrtype_setlength(eledef);
        { since the setlength prototypes require certain types, insert
          explicit type conversions where necessary }
        objarraydef:=nil;
        if (ndims>1) then
          begin
            { expects array of JLObject }
            setlenroutine:='FPC_SETLENGTH_DYNARR_MULTIDIM';
            objarraydef:=search_system_type('TJOBJECTARRAY').typedef
          end
        else
          begin
            case finaltype of
              'R':
                begin
                  { expects array of FpcBaseRecord}
                  setlenroutine:='FPC_SETLENGTH_DYNARR_JRECORD';
                  objarraydef:=search_system_type('TJRECORDARRAY').typedef;
                end;
              'T':
                begin
                  { expects array of ShortstringClass}
                  setlenroutine:='FPC_SETLENGTH_DYNARR_JSHORTSTRING';
                  objarraydef:=search_system_type('TSHORTSTRINGARRAY').typedef;
                end;
              else
                begin
                  { expects JLObject }
                  setlenroutine:='FPC_SETLENGTH_DYNARR_GENERIC';
                  objarraydef:=java_jlobject;
                end
              end;
          end;
        tcallparanode(newparas).left:=ctypeconvnode.create_explicit(tcallparanode(newparas).left,objarraydef);
        newnode:=ctypeconvnode.create_explicit(newnode,objarraydef);
        { prepend new }
        newparas:=ccallparanode.create(newnode,newparas);
        { prepend deepcopy }
        newparas:=ccallparanode.create(cordconstnode.create(0,pasbool1type,false),newparas);
        { call the right setlenght helper }
        if ndims>1 then
          begin
            { create proper parameters, from right to left:
               eletype=finaltype, ndim=ndims, deepcopy=false, new=newnode,
               assignmenttarget=tcallparanode(left).left }
            { prepend ndim }
            newparas:=ccallparanode.create(cordconstnode.create(ndims,s32inttype,false),newparas);
            { prepend eletype }
            newparas:=ccallparanode.create(cordconstnode.create(ord(finaltype),cwidechartype,false),newparas);
          end
        else
          begin
            { create proper parameters, from right to left:
               deepcopy=false, new=newnode, assignmenttarget=tcallparnode(left).left
              -> already done in common part above }
          end;
        result:=ccallnode.createintern(setlenroutine,newparas);
        { assign result back to org (no call-by-reference for Java) }
        result:=cassignmentnode.create(assignmenttarget,
          ctypeconvnode.create_explicit(result,assignmenttarget.resultdef));
        if assigned(lefttemp) then
          begin
            addstatement(newstatement,result);
            addstatement(newstatement,ctempdeletenode.create(lefttemp));
            result:=newblock;
          end;
      end;


    function tjvminlinenode.first_setlength: tnode;
      begin
        { reverse the parameter order so we can process them more easily }
        reverseparameters(tcallparanode(left));
        { treat setlength(x,0) specially: used to init uninitialised locations }
        if not is_shortstring(left.resultdef) and
           not assigned(tcallparanode(tcallparanode(left).right).right) and
           is_constintnode(tcallparanode(tcallparanode(left).right).left) and
           (tordconstnode(tcallparanode(tcallparanode(left).right).left).value=0) then
          begin
            result:=nil;
            expectloc:=LOC_VOID;
            exit;
          end;
        { strings are handled the same as on other platforms }
        if left.resultdef.typ=stringdef then
          begin
            reverseparameters(tcallparanode(left));
            result:=inherited first_setlength;
            exit;
          end;
        case left.resultdef.typ of
          arraydef:
            result:=first_setlength_array;
          else
            internalerror(2011031204);
        end;
      end;


    function tjvminlinenode.first_length: tnode;
      var
        newblock: tblocknode;
        newstatement: tstatementnode;
        lefttemp,
        lentemp: ttempcreatenode;
        ifcond,
        stringtemp,
        stringnonnull,
        stringnull: tnode;
        psym: tsym;
        stringclass: tdef;
      begin
        if is_wide_or_unicode_string(left.resultdef) or
           is_ansistring(left.resultdef) then
          begin
            { if assigned(stringclass(left)) then
                lentemp:=stringclass(left).length()
              else
                lentemp:=0;
              --> return lentemp
            }
            if is_ansistring(left.resultdef) then
              stringclass:=java_ansistring
            else
              stringclass:=java_jlstring;
            newblock:=internalstatements(newstatement);
            { store left into a temp since it may contain a function call
              (which must not be evaluated twice) }
            if node_complexity(left)>4 then
              begin
                lefttemp:=ctempcreatenode.create_value(stringclass,stringclass.size,tt_persistent,true,ctypeconvnode.create_explicit(left,stringclass));
                addstatement(newstatement,lefttemp);
                stringtemp:=ctemprefnode.create(lefttemp)
              end
            else
              begin
                lefttemp:=nil;
                stringtemp:=ctypeconvnode.create_explicit(left,stringclass);
              end;
            left:=nil;
            lentemp:=ctempcreatenode.create(s32inttype,s32inttype.size,tt_persistent,true);
            addstatement(newstatement,lentemp);
            { if-condition: assigned(stringclass(stringvar))? }
            ifcond:=cinlinenode.create(in_assigned_x,false,
              ccallparanode.create(stringtemp.getcopy,nil));
            { then-path: call length() method }
            psym:=search_struct_member(tabstractrecorddef(stringclass),'LENGTH');
            if not assigned(psym) or
               (psym.typ<>procsym) then
              internalerror(2011031403);
            stringnonnull:=cassignmentnode.create(
              ctemprefnode.create(lentemp),
              ccallnode.create(nil,tprocsym(psym),psym.owner,stringtemp,[],nil));
            { else-path: length is 0 }
            stringnull:=cassignmentnode.create(
              ctemprefnode.create(lentemp),
              genintconstnode(0));
            { complete if-statement }
            addstatement(newstatement,cifnode.create(ifcond,stringnonnull,stringnull));
            { free lefttemp }
            if assigned(lefttemp) then
              addstatement(newstatement,ctempdeletenode.create(lefttemp));
            { return len temp }
            addstatement(newstatement,ctempdeletenode.create_normal_temp(lentemp));
            addstatement(newstatement,ctemprefnode.create(lentemp));
            result:=newblock;
          end
        else if is_shortstring(left.resultdef) then
          begin
            psym:=search_struct_member(tabstractrecorddef(java_shortstring),'LENGTH');
            if not assigned(psym) or
               (psym.typ<>procsym) then
              internalerror(2011052402);
            result:=
              ccallnode.create(nil,tprocsym(psym),psym.owner,
                ctypeconvnode.create_explicit(caddrnode.create_internal(left),java_shortstring),[],nil);
            { reused }
            left:=nil;
          end
        { should be no other string types }
        else if left.resultdef.typ=stringdef then
          internalerror(2011052403)
       else
         result:=inherited first_length;
      end;


    procedure tjvminlinenode.second_length;
      begin
        if is_dynamic_array(left.resultdef) or
           is_open_array(left.resultdef) or
           is_array_of_const(left.resultdef) then
          begin
            location_reset(location,LOC_REGISTER,OS_S32);
            location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,s32inttype);
            secondpass(left);
            thlcgjvm(hlcg).g_getarraylen(current_asmdata.CurrAsmList,left.location);
            thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,resultdef,location.register);
          end
        else
          internalerror(2011012004);
      end;

(*
     function tjvminlinenode.first_round_real : tnode;
      begin
       if (current_settings.cputype >= cpu_PPC970) then
          begin
            expectloc:=LOC_REFERENCE;
            first_round_real := nil;
          end
        else
          result:=inherited first_round_real;
      end;
*)

     { load the FPU value on the evaluation stack }
     procedure tjvminlinenode.load_fpu_location;
       begin
         secondpass(left);
         thlcgjvm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,left.resultdef,left.location);
       end;

(*
    procedure tjvminlinenode.second_sqrt_real;
      begin
        if (current_settings.cputype < cpu_PPC970) then
          internalerror(2007020910);
        location.loc:=LOC_FPUREGISTER;
        load_fpu_location;
        case left.location.size of
          OS_F32:
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FSQRTS,location.register,
              left.location.register));
          OS_F64:
            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FSQRT,location.register,
              left.location.register));
          else
            inherited;
        end;
      end;
*)

     procedure tjvminlinenode.second_sqr_real;
       begin
         load_fpu_location;
         location_reset(location,LOC_FPUREGISTER,location.size);
         location.register:=hlcg.getfpuregister(current_asmdata.CurrAsmList,resultdef);
         case left.location.size of
           OS_F32:
             begin
               current_asmdata.CurrAsmList.concat(taicpu.op_none(a_dup));
               thlcgjvm(hlcg).incstack(current_asmdata.CurrAsmList,1);
               current_asmdata.CurrAsmList.concat(taicpu.op_none(a_fmul));
               thlcgjvm(hlcg).decstack(current_asmdata.CurrAsmList,1);
             end;
           OS_F64:
             begin
               current_asmdata.CurrAsmList.concat(taicpu.op_none(a_dup2));
               thlcgjvm(hlcg).incstack(current_asmdata.CurrAsmList,2);
               current_asmdata.CurrAsmList.concat(taicpu.op_none(a_dmul));
               thlcgjvm(hlcg).decstack(current_asmdata.CurrAsmList,2);
             end;
           else
             internalerror(2011010804);
         end;
         thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,resultdef,location.register);
       end;


    procedure tjvminlinenode.second_trunc_real;
      begin
         load_fpu_location;
         location_reset(location,LOC_REGISTER,left.location.size);
         location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
         case left.location.size of
           OS_F32:
             begin
               current_asmdata.CurrAsmList.concat(taicpu.op_none(a_f2l));
               { 32 bit float -> 64 bit int: +1 stack slot }
               thlcgjvm(hlcg).incstack(current_asmdata.CurrAsmList,1);
             end;
           OS_F64:
             begin
               { 64 bit float -> 64 bit int: same number of stack slots }
               current_asmdata.CurrAsmList.concat(taicpu.op_none(a_d2l));
             end;
           else
             internalerror(2011010805);
         end;
         thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,resultdef,location.register);
      end;


    procedure tjvminlinenode.second_new;
      var
        arr: tnode;
        hp: tcallparanode;
        paracount: longint;
      begin
        hp:=tcallparanode(left);
        { we don't second pass this one, it's only a type node }
        arr:=hp.left;
        if not is_dynamic_array(arr.resultdef) then
          internalerror(2011012204);
        hp:=tcallparanode(hp.right);
        if not assigned(hp) then
          internalerror(2011012205);
        paracount:=0;
        { put all the dimensions on the stack }
        repeat
          inc(paracount);
          secondpass(hp.left);
          thlcgjvm(hlcg).a_load_loc_stack(current_asmdata.CurrAsmList,hp.left.resultdef,hp.left.location);
          hp:=tcallparanode(hp.right);
        until not assigned(hp);
        { create the array }
        thlcgjvm(hlcg).g_newarray(current_asmdata.CurrAsmList,arr.resultdef,paracount);
        location_reset(location,LOC_REGISTER,OS_ADDR);
        location.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,resultdef);
        thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,arr.resultdef,location.register);
      end;


    procedure tjvminlinenode.second_setlength;
      var
        target: tnode;
        lenpara: tnode;
        emptystr: ansichar;
        tmpreg: tregister;
      begin
        target:=tcallparanode(left).left;
        lenpara:=tcallparanode(tcallparanode(left).right).left;
        if assigned(tcallparanode(tcallparanode(left).right).right) or
           not is_constintnode(lenpara) or
           (tordconstnode(lenpara).value<>0) then
          internalerror(2011031801);

        secondpass(target);
        { can't directly load from stack to destination, because if target is
          a reference then its address must be placed on the stack before the
          value }
        tmpreg:=hlcg.getaddressregister(current_asmdata.CurrAsmList,target.resultdef);
        if is_wide_or_unicode_string(target.resultdef) then
          begin
            emptystr:=#0;
            current_asmdata.CurrAsmList.concat(taicpu.op_string(a_ldc,0,@emptystr));
            thlcgjvm(hlcg).incstack(current_asmdata.CurrAsmList,1);
          end
        else if is_ansistring(target.resultdef) then
          thlcgjvm(hlcg).a_load_const_stack(current_asmdata.CurrAsmList,java_jlobject,0,R_ADDRESSREGISTER)
        else if is_dynamic_array(target.resultdef) then
          begin
            thlcgjvm(hlcg).a_load_const_stack(current_asmdata.CurrAsmList,s32inttype,0,R_INTREGISTER);
            thlcgjvm(hlcg).g_newarray(current_asmdata.CurrAsmList,target.resultdef,1);
          end
        else
          internalerror(2011031401);
        thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,target.resultdef,tmpreg);
        thlcgjvm(hlcg).a_load_reg_loc(current_asmdata.CurrAsmList,target.resultdef,target.resultdef,tmpreg,target.location);
      end;


begin
   cinlinenode:=tjvminlinenode;
end.
