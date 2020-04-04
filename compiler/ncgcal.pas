{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate assembler for call nodes

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
unit ncgcal;

{$i fpcdefs.inc}

interface

    uses
      cpubase,
      globtype,
      parabase,cgutils,
      aasmdata,cgbase,
      symdef,node,ncal;

    type
       tcgcallparanode = class(tcallparanode)
       protected
          function push_zero_sized_value_para: boolean; virtual;

          procedure push_addr_para;
          procedure push_value_para;virtual;
          procedure push_formal_para;virtual;
          procedure push_copyout_para;virtual;abstract;
       public
          tempcgpara : tcgpara;

          constructor create(expr,next : tnode);override;
          destructor destroy;override;
          procedure secondcallparan;override;
       end;

       { tcgcallnode }

       tcgcallnode = class(tcallnode)
       private

          procedure handle_return_value;
          procedure release_unused_return_value;
          procedure copy_back_paras;
          procedure release_para_temps;
          procedure reorder_parameters;
          procedure freeparas;
       protected
          retloc: tcgpara;
          paralocs: array of pcgpara;

          framepointer_paraloc : tcgpara;
          {# This routine is used to push the current frame pointer
             on the stack. This is used in nested routines where the
             value of the frame pointer is always pushed as an extra
             parameter.

             The default handling is the standard handling used on
             most stack based machines, where the frame pointer is
             the first invisible parameter.
          }
          procedure pop_parasize(pop_size:longint);virtual;
          procedure extra_interrupt_code;virtual;
          procedure extra_pre_call_code;virtual;
          procedure extra_call_code;virtual;
          procedure extra_post_call_code;virtual;

          function get_syscall_libbase_paraloc: pcgparalocation;virtual;
          procedure get_syscall_call_ref(out tmpref: treference; reg: tregister);virtual;
          procedure do_syscall;virtual;abstract;

          { The function result is returned in a tcgpara. This tcgpara has to
            be translated into a tlocation so the rest of the code generator
            can work with it. This routine decides what the most appropriate
            tlocation is and sets self.location based on that. }
          procedure set_result_location(realresdef: tstoreddef);virtual;

          { if an unused return value is in another location than a
            LOC_REFERENCE, this method will be called to perform the necessary
            cleanups. By default it does not do anything }
          procedure do_release_unused_return_value;virtual;

          { Override the following three methods to support calls to address in
            'ref' without loading it into register (only x86 targets probably).
            If can_call_ref returns true, it should do required simplification
            on ref. }
          function can_call_ref(var ref: treference):boolean;virtual;
          procedure extra_call_ref_code(var ref: treference);virtual;
          function do_call_ref(ref: treference): tcgpara;virtual;

          { store all the parameters in the temporary paralocs in their final
            location, and create the paralocs array that will be passed to
            hlcg.a_call_* }
          procedure pushparas;virtual;

          { loads the code pointer of a complex procvar (one with a self/
            parentfp/... and a procedure address) into a register and returns it }
          procedure load_complex_procvar_codeptr(out reg: tregister; out callprocdef: tabstractprocdef); virtual;
          { loads the procvar code pointer into a register with type def }
          procedure load_procvar_codeptr(out reg: tregister; out callprocdef: tabstractprocdef);

          procedure load_block_invoke(out toreg: tregister; out callprocdef: tabstractprocdef);

          function get_call_reg(list: TAsmList): tregister; virtual;
          procedure unget_call_reg(list: TAsmList; reg: tregister); virtual;
       public
          procedure pass_generate_code;override;
          destructor destroy;override;
       end;


implementation

    uses
      systems,
      verbose,globals,
      symconst,symtable,symtype,symsym,defutil,paramgr,
      pass_2,
      nld,ncnv,
      ncgutil,blockutl,
      cgobj,tgobj,hlcgobj,
      procinfo,
      wpobase;


{*****************************************************************************
                             TCGCALLPARANODE
*****************************************************************************}

    constructor tcgcallparanode.create(expr,next : tnode);
      begin
        inherited create(expr,next);
        tempcgpara.init;
      end;


    destructor tcgcallparanode.destroy;
      begin
        tempcgpara.done;
        inherited destroy;
      end;


    function tcgcallparanode.push_zero_sized_value_para: boolean;
      begin
        { nothing to push by default }
        result:=false;
      end;


    procedure tcgcallparanode.push_addr_para;
      var
        valuedef: tdef;
      begin
        if not(left.location.loc in [LOC_CREFERENCE,LOC_REFERENCE]) then
          internalerror(200304235);
        { see the call to keep_para_array_range in ncal: if that call returned
          true, we overwrite the resultdef of left with its original resultdef
          (to keep track of the range of the original array); we inserted a type
          conversion to parasym.vardef, so that is the type this value actually
          has }
        if is_dynamic_array(left.resultdef) and
           is_open_array(parasym.vardef) then
          valuedef:=parasym.vardef
        else
          valuedef:=left.resultdef;
        hlcg.a_loadaddr_ref_cgpara(current_asmdata.CurrAsmList,valuedef,left.location.reference,tempcgpara);
      end;


    procedure tcgcallnode.reorder_parameters;
      var
        hpcurr,hpprev,hpnext,hpreversestart : tcgcallparanode;
      begin
        { All parameters are now in temporary locations. If we move them to
          their regular locations in the same order, then we get the
          following pattern for register parameters:
            mov para1, tempreg1
            mov para2, tempreg2
            mov para3, tempreg3
            mov tempreg1, parareg1
            mov tempreg2, parareg2
            mov tempreg3, parareg3

          The result is that all tempregs conflict with all pararegs.
          A better solution is to use:
            mov para1, tempreg1
            mov para2, tempreg2
            mov para3, tempreg3
            mov tempreg3, parareg3
            mov tempreg2, parareg2
            mov tempreg1, parareg1
          This way, tempreg2 can be the same as parareg1 etc.

          To achieve this, we invert the order of all LOC_XREGISTER
          paras (JM).
        }
        hpcurr:=tcgcallparanode(left);
        { assume all LOC_REFERENCE parameters come first
          (see tcallnode.order_parameters)
        }
        hpreversestart:=nil;
        while assigned(hpcurr) do
          begin
            if not(hpcurr.parasym.paraloc[callerside].location^.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
              hpreversestart:=hpcurr;
            hpcurr:=tcgcallparanode(hpcurr.right);
          end;

        { since all register tempparalocs have basically a complexity of 1,
          (unless there are large stack offsets that require a temp register on
          some architectures, but that's minor), we don't have to care about
          the internal relative order of different register type parameters
        }
        hpprev:=nil;
        hpcurr:=tcgcallparanode(left);
        while (hpcurr<>hpreversestart) do
          begin
            hpnext:=tcgcallparanode(hpcurr.right);
            if not(hpcurr.parasym.paraloc[callerside].location^.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
              begin
                { remove hpcurr from chain }
                if assigned(hpprev) then
                  hpprev.right:=hpnext
                else
                  left:=hpnext;
                { insert right after hpreversestart, so every element will
                  be inserted right before the previously moved one ->
                  reverse order; hpreversestart itself is the last register
                  parameter }
                hpcurr.right:=hpreversestart.right;
                hpreversestart.right:=hpcurr;
              end
            else
              hpprev:=hpcurr;
            hpcurr:=hpnext;
          end;
      end;


    procedure tcgcallparanode.push_value_para;
      begin
        { we've nothing to push when the size of the parameter is 0
          -- except on platforms where the parameters are part of the signature
             and checked by the runtime/backend compiler (e.g. JVM, LLVM) }
        if (left.resultdef.size=0) and
           not push_zero_sized_value_para then
          exit;

        { Move flags and jump in register to make it less complex }
        if left.location.loc in [LOC_FLAGS,LOC_JUMP,LOC_SUBSETREG,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF] then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);

        { load the parameter's tlocation into its cgpara }
        hlcg.gen_load_loc_cgpara(current_asmdata.CurrAsmList,left.resultdef,left.location,tempcgpara)
      end;


    procedure tcgcallparanode.push_formal_para;
      begin
        { allow passing of a constant to a const formaldef }
        if (parasym.varspez=vs_const) and
           not(left.location.loc in [LOC_CREFERENCE,LOC_REFERENCE]) then
          hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);
        push_addr_para;
      end;


    procedure tcgcallparanode.secondcallparan;
      var
         pushaddr: boolean;
      begin
         if not(assigned(parasym)) then
           internalerror(200304242);

         { Skip nothingn nodes which are used after disabling
           a parameter }
         if (left.nodetype<>nothingn) then
           begin
             if assigned(fparainit) then
               secondpass(fparainit);
             secondpass(left);

             hlcg.maybe_change_load_node_reg(current_asmdata.CurrAsmList,left,true);

             paramanager.createtempparaloc(current_asmdata.CurrAsmList,aktcallnode.procdefinition.proccalloption,parasym,not followed_by_stack_tainting_call_cached,tempcgpara);

             { handle varargs first, because parasym is not valid }
             if (cpf_varargs_para in callparaflags) then
               begin
                 if paramanager.push_addr_param(vs_value,left.resultdef,
                        aktcallnode.procdefinition.proccalloption) then
                   push_addr_para
                 else
                   push_value_para;
               end
             { hidden parameters }
             else if (vo_is_hidden_para in parasym.varoptions) then
               begin
                 { don't push a node that already generated a pointer type
                   by address for implicit hidden parameters }
                 pushaddr:=(vo_is_funcret in parasym.varoptions) or
                   { pass "this" in C++ classes explicitly as pointer
                     because push_addr_param might not be true for them }
                   (is_cppclass(parasym.vardef) and (vo_is_self in parasym.varoptions)) or
                    (
                      (
                        not(left.resultdef.typ in [pointerdef,classrefdef]) or
                        (
                          { allow pointerdefs (as self) to be passed as addr
                            param if the method is part of a type helper which
                            extends a pointer type }
                          (vo_is_self in parasym.varoptions) and
                          (aktcallnode.procdefinition.owner.symtabletype=objectsymtable) and
                          (is_objectpascal_helper(tdef(aktcallnode.procdefinition.owner.defowner))) and
                          (tobjectdef(aktcallnode.procdefinition.owner.defowner).extendeddef.typ=pointerdef)
                        )
                      ) and
                     paramanager.push_addr_param(parasym.varspez,parasym.vardef,
                         aktcallnode.procdefinition.proccalloption));

                 if pushaddr then
                   begin
                     { objects or advanced records could be located in registers if they are the result of a type case, see e.g. webtbs\tw26075.pp }
                     if not(left.location.loc in [LOC_CREFERENCE,LOC_REFERENCE]) then
                       hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);
                     push_addr_para
                   end
                 else
                   push_value_para;
               end
             { formal def }
             else if (parasym.vardef.typ=formaldef) then
               push_formal_para
             { Normal parameter }
             else if paramanager.push_copyout_param(parasym.varspez,parasym.vardef,
                         aktcallnode.procdefinition.proccalloption) then
               push_copyout_para
             else
               begin
                 { don't push a node that already generated a pointer type
                   by address for implicit hidden parameters }
                 if (not(
                         (vo_is_hidden_para in parasym.varoptions) and
                         (left.resultdef.typ in [pointerdef,classrefdef])
                        ) and
                     paramanager.push_addr_param(parasym.varspez,parasym.vardef,
                         aktcallnode.procdefinition.proccalloption)) and
                     { dyn. arrays passed to an array of const must be passed by value, see tests/webtbs/tw4219.pp }
                     not(
                         is_array_of_const(parasym.vardef) and
                         is_dynamic_array(left.resultdef)
                        ) then
                   begin
                      { Passing a var parameter to a var parameter, we can
                        just push the address transparently }
                      if (left.nodetype=loadn) and
                         (tloadnode(left).is_addr_param_load) then
                        begin
                          if (left.location.reference.index<>NR_NO) or
                             (left.location.reference.offset<>0) then
                            internalerror(200410107);
                          hlcg.a_load_reg_cgpara(current_asmdata.CurrAsmList,cpointerdef.getreusable(left.resultdef),left.location.reference.base,tempcgpara)
                        end
                      else
                        begin
                          { Force to be in memory }
                          if not(left.location.loc in [LOC_CREFERENCE,LOC_REFERENCE]) then
                            hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);
                          push_addr_para;
                        end;
                   end
                 else
                   push_value_para;
               end;

             { update return location in callnode when this is the function
               result }
             if assigned(parasym) and
                (
                  { for type helper/record constructor check that it is self parameter }
                  (
                    (vo_is_self in parasym.varoptions) and
                    (aktcallnode.procdefinition.proctypeoption=potype_constructor) and
                    (parasym.vardef.typ<>objectdef)
                  ) or
                  (vo_is_funcret in parasym.varoptions)
                ) then
               location_copy(aktcallnode.location,left.location);
           end;

         { next parameter }
         if assigned(right) then
           tcallparanode(right).secondcallparan;
      end;


{*****************************************************************************
                             TCGCALLNODE
*****************************************************************************}

{$if first_mm_imreg = 0}
  {$WARN 4044 OFF} { Comparison might be always false ... }
{$endif}

    procedure tcgcallnode.extra_interrupt_code;
      begin
      end;


    procedure tcgcallnode.extra_pre_call_code;
      begin
      end;


    procedure tcgcallnode.extra_call_code;
      begin
      end;


    procedure tcgcallnode.extra_post_call_code;
      begin
      end;

    function tcgcallnode.get_syscall_libbase_paraloc: pcgparalocation;
      var
        hsym: tsym;
      begin
        hsym:=tsym(procdefinition.parast.Find('syscalllib'));
        if not assigned(hsym) then
          internalerror(2016110605);
        result:=tparavarsym(hsym).paraloc[callerside].location;
        if not assigned(result) then
          internalerror(2016110604);
      end;

    procedure tcgcallnode.get_syscall_call_ref(out tmpref: treference; reg: tregister);
      var
        libparaloc: pcgparalocation;
      begin
        libparaloc:=get_syscall_libbase_paraloc;

        case libparaloc^.loc of
          LOC_REGISTER:
            reference_reset_base(tmpref,libparaloc^.register,-tprocdef(procdefinition).extnumber,ctempposinvalid,sizeof(pint),[]);
          LOC_REFERENCE:
            begin
              reference_reset_base(tmpref,libparaloc^.reference.index,libparaloc^.reference.offset,ctempposinvalid,sizeof(pint),[]);
              cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,tmpref,reg);
              reference_reset_base(tmpref,reg,-tprocdef(procdefinition).extnumber,ctempposinvalid,sizeof(pint),[]);
            end;
          else
            begin
              reference_reset(tmpref,0,[]);
              internalerror(2016090202);
            end;
        end;
      end;

    function tcgcallnode.can_call_ref(var ref: treference): boolean;
      begin
        result:=false;
      end;


    procedure tcgcallnode.extra_call_ref_code(var ref: treference);
      begin
        { no action by default }
      end;


    function tcgcallnode.do_call_ref(ref: treference): tcgpara;
      begin
        InternalError(2014012901);
        { silence warning }
        result.init;
      end;


    procedure tcgcallnode.load_block_invoke(out toreg: tregister; out callprocdef: tabstractprocdef);
      var
        href: treference;
        literaldef: trecorddef;
      begin
        literaldef:=get_block_literal_type_for_proc(tabstractprocdef(right.resultdef));
        hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,cpointerdef.getreusable(literaldef),true);
        { load the invoke pointer }
        hlcg.reference_reset_base(href,right.resultdef,right.location.register,0,ctempposinvalid,right.resultdef.alignment,[]);
        callprocdef:=cprocvardef.getreusableprocaddr(procdefinition,pc_address_only);
        toreg:=hlcg.getaddressregister(current_asmdata.CurrAsmList,callprocdef);
        hlcg.g_load_field_reg_by_name(current_asmdata.CurrAsmList,literaldef,callprocdef,'INVOKE',href,toreg);
     end;


    function tcgcallnode.get_call_reg(list: TAsmList): tregister;
      begin
        result:=hlcg.getaddressregister(current_asmdata.CurrAsmList,procdefinition.address_type);
      end;


    procedure tcgcallnode.unget_call_reg(list: TAsmList; reg: tregister);
      begin
        { nothing to do by default }
      end;


    procedure tcgcallnode.set_result_location(realresdef: tstoreddef);
      begin
        if realresdef.is_intregable or
           realresdef.is_fpuregable or
           { avoid temporarily storing pointer-sized entities that can't be
             regvars, such as reference-counted pointers, to memory --
             no exception can occur right now (except in case of existing
             memory corruption), and we'd store them to a regular temp
             anyway and that is not safer than keeping them in a register }
           ((realresdef.size=sizeof(aint)) and
            (retloc.location^.loc=LOC_REGISTER) and
            not assigned(retloc.location^.next)) then
          location_allocate_register(current_asmdata.CurrAsmList,location,realresdef,false)
        else
          begin
            location_reset_ref(location,LOC_REFERENCE,def_cgsize(realresdef),0,[]);
            tg.gethltemp(current_asmdata.CurrAsmList,realresdef,retloc.intsize,tt_normal,location.reference);
          end;
      end;


    procedure tcgcallnode.do_release_unused_return_value;
      begin
        case location.loc of
          LOC_REFERENCE :
            begin
              if is_managed_type(resultdef) then
                 hlcg.g_finalize(current_asmdata.CurrAsmList,resultdef,location.reference);
               tg.ungetiftemp(current_asmdata.CurrAsmList,location.reference);
            end;
          else
            ;
        end;
      end;


    procedure tcgcallnode.pop_parasize(pop_size:longint);
      begin
      end;


    procedure tcgcallnode.handle_return_value;
      var
        realresdef: tstoreddef;
      begin
        { Check that the return location is set when the result is passed in
          a parameter }
        if paramanager.ret_in_param(resultdef,procdefinition) then
          begin
            { self.location is set near the end of secondcallparan so it
              refers to the implicit result parameter }
            if location.loc<>LOC_REFERENCE then
              internalerror(200304241);
            exit;
          end;

        if not assigned(typedef) then
          realresdef:=tstoreddef(resultdef)
        else
          realresdef:=tstoreddef(typedef);

          { get a tlocation that can hold the return value that's currently in
            the return value's tcgpara }
          set_result_location(realresdef);

          { Do not move the physical register to a virtual one in case
            the return value is not used, because if the virtual one is
            then mapped to the same register as the physical one, we will
            end up with two deallocs of this register (one inserted here,
            one inserted by the register allocator), which unbalances the
            register allocation information.  The return register(s) will
            be freed by location_free() in release_unused_return_value
            (mantis #13536).  }
          if (cnf_return_value_used in callnodeflags) or
             assigned(funcretnode) then
            hlcg.gen_load_cgpara_loc(current_asmdata.CurrAsmList,realresdef,retloc,location,false);

        { copy value to the final location if this was already provided to the
          callnode. This must be done after the call node, because the location can
          also be used as parameter and may not be finalized yet }
        if assigned(funcretnode) then
          begin
            funcretnode.pass_generate_code;
            { Decrease refcount for refcounted types, this can be skipped when
              we have used a temp, because then it is already done from tempcreatenode.
              Also no finalize is needed, because there is no risk of exceptions from the
              function since this is code is only executed after the function call has returned }
            if is_managed_type(funcretnode.resultdef) and
               (funcretnode.nodetype<>temprefn) then
              hlcg.g_finalize(current_asmdata.CurrAsmList,funcretnode.resultdef,funcretnode.location.reference);

            case location.loc of
              LOC_REGISTER :
                begin
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
                  if location.size in [OS_64,OS_S64] then
                    cg64.a_load64_reg_loc(current_asmdata.CurrAsmList,location.register64,funcretnode.location)
                  else
{$endif}
                    hlcg.a_load_reg_loc(current_asmdata.CurrAsmList,resultdef,resultdef,location.register,funcretnode.location);
                  location_free(current_asmdata.CurrAsmList,location);
                end;
              LOC_REFERENCE:
                begin
                  case funcretnode.location.loc of
                    LOC_REGISTER:
                      hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,resultdef,resultdef,location.reference,funcretnode.location.register);
                    LOC_REFERENCE:
                      hlcg.g_concatcopy(current_asmdata.CurrAsmList,resultdef,location.reference,funcretnode.location.reference);
                    else
                      internalerror(200802121);
                  end;
                  location_freetemp(current_asmdata.CurrAsmList,location);
                end;
              else
                internalerror(200709085);
            end;
            location := funcretnode.location;
          end;
      end;


    procedure tcgcallnode.release_unused_return_value;
      begin
        { When the result is not used we need to finalize the result and
          can release the temp. This need to be after the callcleanupblock
          tree is generated, because that converts the temp from persistent to normal }
        if not(cnf_return_value_used in callnodeflags) then
          begin
            do_release_unused_return_value;
            if (retloc.intsize<>0) then
              paramanager.freecgpara(current_asmdata.CurrAsmList,retloc);
            location_reset(location,LOC_VOID,OS_NO);
         end;
      end;


    procedure tcgcallnode.copy_back_paras;
      var
        ppn : tcallparanode;
      begin
        ppn:=tcallparanode(left);
        while assigned(ppn) do
          begin
             if assigned(ppn.paracopyback) then
               secondpass(ppn.paracopyback);
             ppn:=tcallparanode(ppn.right);
          end;
      end;


    procedure tcgcallnode.release_para_temps;
      var
        hp,
        hp2 : tnode;
        ppn : tcallparanode;
      begin
        { Release temps from parameters }
        ppn:=tcallparanode(left);
        while assigned(ppn) do
          begin
             if assigned(ppn.left) then
               begin
                 { don't release the funcret temp }
                 if not(assigned(ppn.parasym)) or
                    not(
                      (vo_is_funcret in ppn.parasym.varoptions) or
                      (
                        (vo_is_self in ppn.parasym.varoptions) and
                        (procdefinition.proctypeoption=potype_constructor) and
                        (ppn.parasym.vardef.typ<>objectdef)
                      )
                    )then
                   location_freetemp(current_asmdata.CurrAsmList,ppn.left.location);
                 { process also all nodes of an array of const }
                 hp:=ppn.left;
                 while (hp.nodetype=typeconvn) do
                   hp:=ttypeconvnode(hp).left;
                 if (hp.nodetype=arrayconstructorn) and
                    assigned(tarrayconstructornode(hp).left) then
                   begin
                     while assigned(hp) do
                       begin
                         hp2:=tarrayconstructornode(hp).left;
                         { ignore typeconvs and addrn inserted by arrayconstructn for
                           passing a shortstring }
                         if (hp2.nodetype=typeconvn) and
                            (tunarynode(hp2).left.nodetype=addrn) then
                           hp2:=tunarynode(tunarynode(hp2).left).left
                         else if hp2.nodetype=addrn then
                           hp2:=tunarynode(hp2).left;
                         location_freetemp(current_asmdata.CurrAsmList,hp2.location);
                         hp:=tarrayconstructornode(hp).right;
                       end;
                   end;
               end;
             ppn:=tcallparanode(ppn.right);
          end;
        setlength(paralocs,0);
      end;


     procedure tcgcallnode.pushparas;
       var
         ppn : tcgcallparanode;
         callerparaloc,
         tmpparaloc : pcgparalocation;
         sizeleft: aint;
         htempref,
         href : treference;
         calleralignment,
         tmpalignment, i: longint;
         skipiffinalloc: boolean;
       begin
         { copy all resources to the allocated registers }
         ppn:=tcgcallparanode(left);
         while assigned(ppn) do
           begin
             if (ppn.left.nodetype<>nothingn) then
               begin
                 { better check for the real location of the parameter here, when stack passed parameters
                   are saved temporary in registers, checking for the tmpparaloc.loc is wrong
                 }
                 paramanager.freecgpara(current_asmdata.CurrAsmList,ppn.tempcgpara);
                 tmpparaloc:=ppn.tempcgpara.location;
                 sizeleft:=ppn.tempcgpara.intsize;
                 calleralignment:=ppn.parasym.paraloc[callerside].alignment;
                 tmpalignment:=ppn.tempcgpara.alignment;
                 if (tmpalignment=0) or
                    (calleralignment=0) then
                   internalerror(2009020701);
                 callerparaloc:=ppn.parasym.paraloc[callerside].location;
                 skipiffinalloc:=
                   not paramanager.use_fixed_stack or
                   not(ppn.followed_by_stack_tainting_call_cached);
                 while assigned(callerparaloc) do
                   begin
                     { Every paraloc must have a matching tmpparaloc }
                     if not assigned(tmpparaloc) then
                       internalerror(200408224);
                     if callerparaloc^.size<>tmpparaloc^.size then
                       internalerror(200408225);
                     case callerparaloc^.loc of
                       LOC_REGISTER:
                         begin
                           if tmpparaloc^.loc<>LOC_REGISTER then
                             internalerror(200408221);
                           if getsupreg(callerparaloc^.register)<first_int_imreg then
                             cg.getcpuregister(current_asmdata.CurrAsmList,callerparaloc^.register);
                           cg.a_load_reg_reg(current_asmdata.CurrAsmList,tmpparaloc^.size,tmpparaloc^.size,
                               tmpparaloc^.register,callerparaloc^.register);
                         end;
                       LOC_FPUREGISTER:
                         begin
                           if tmpparaloc^.loc<>LOC_FPUREGISTER then
                             internalerror(200408222);
                           if getsupreg(callerparaloc^.register)<first_fpu_imreg then
                             cg.getcpuregister(current_asmdata.CurrAsmList,callerparaloc^.register);
                           cg.a_loadfpu_reg_reg(current_asmdata.CurrAsmList,tmpparaloc^.size,tmpparaloc^.size,tmpparaloc^.register,callerparaloc^.register);
                         end;
                       LOC_MMREGISTER:
                         begin
                           if tmpparaloc^.loc<>LOC_MMREGISTER then
                             internalerror(200408223);
                           if getsupreg(callerparaloc^.register)<first_mm_imreg then
                             cg.getcpuregister(current_asmdata.CurrAsmList,callerparaloc^.register);
                           cg.a_loadmm_reg_reg(current_asmdata.CurrAsmList,tmpparaloc^.size,tmpparaloc^.size,
                             tmpparaloc^.register,callerparaloc^.register,mms_movescalar);
                         end;
                       LOC_REFERENCE:
                         begin
                           if not(skipiffinalloc and
                                  paramanager.is_stack_paraloc(callerparaloc)) then
                             begin
                               { Can't have a data copied to the stack, every location
                                 must contain a valid size field }

                              if (tmpparaloc^.size=OS_NO) and
                                 ((tmpparaloc^.loc<>LOC_REFERENCE) or
                                  assigned(tmpparaloc^.next)) then
                                internalerror(200501281);
                                reference_reset_base(href,callerparaloc^.reference.index,callerparaloc^.reference.offset,ctempposinvalid,calleralignment,[]);
                              { copy parameters in case they were moved to a temp. location because we've a fixed stack }
                              case tmpparaloc^.loc of
                              LOC_REFERENCE:
                                  begin
                                    reference_reset_base(htempref,tmpparaloc^.reference.index,tmpparaloc^.reference.offset,ctempposinvalid,tmpalignment,[]);
                                    { use concatcopy, because it can also be a float which fails when
                                      load_ref_ref is used }
                                    if (ppn.tempcgpara.size <> OS_NO) then
                                      cg.g_concatcopy(current_asmdata.CurrAsmList,htempref,href,tcgsize2size[tmpparaloc^.size])
                                    else
                                      cg.g_concatcopy(current_asmdata.CurrAsmList,htempref,href,sizeleft)
                                  end;
                                LOC_REGISTER:
                                  cg.a_load_reg_ref(current_asmdata.CurrAsmList,tmpparaloc^.size,tmpparaloc^.size,tmpparaloc^.register,href);
                                LOC_FPUREGISTER:
                                  cg.a_loadfpu_reg_ref(current_asmdata.CurrAsmList,tmpparaloc^.size,tmpparaloc^.size,tmpparaloc^.register,href);
                                LOC_MMREGISTER:
                                  cg.a_loadmm_reg_ref(current_asmdata.CurrAsmList,tmpparaloc^.size,tmpparaloc^.size,tmpparaloc^.register,href,mms_movescalar);
                                else
                                  internalerror(200402081);
                             end;
                           end;
                         end;
                       LOC_VOID:
                         ;
                       else
                         internalerror(2019050707);
                     end;
                     dec(sizeleft,tcgsize2size[tmpparaloc^.size]);
                     callerparaloc:=callerparaloc^.next;
                     tmpparaloc:=tmpparaloc^.next;
                   end;
               end;
             ppn:=tcgcallparanode(ppn.right);
           end;
         setlength(paralocs,procdefinition.paras.count);
         for i:=0 to procdefinition.paras.count-1 do
           paralocs[i]:=@tparavarsym(procdefinition.paras[i]).paraloc[callerside];
       end;


     procedure tcgcallnode.load_complex_procvar_codeptr(out reg: tregister; out callprocdef: tabstractprocdef);
       var
         srcreg: tregister;
       begin
         { this is safe even on i8086, because procvardef code pointers are
           always far there (so the current state of far calls vs the state
           of far calls where the procvardef was defined does not matter,
           even though the procvardef constructor called by getcopyas looks at
           it) }
         callprocdef:=cprocvardef.getreusableprocaddr(procdefinition,pc_address_only);
         reg:=hlcg.getaddressregister(current_asmdata.CurrAsmList,callprocdef);
         { in case we have a method pointer on a big endian target in registers,
           the method address is stored in registerhi (it's the first field
           in the tmethod record) }
         if (right.location.loc in [LOC_REGISTER,LOC_CREGISTER]) then
           begin
             if not(right.location.size in [OS_PAIR,OS_SPAIR]) then
               internalerror(2014081401);
             if (target_info.endian=endian_big) then
               srcreg:=right.location.registerhi
             else
               srcreg:=right.location.register;
             hlcg.a_load_reg_reg(current_asmdata.CurrAsmList,callprocdef,callprocdef,srcreg,reg)
           end
         else
           begin
             hlcg.location_force_mem(current_asmdata.CurrAsmList,right.location,procdefinition);
             hlcg.g_ptrtypecast_ref(current_asmdata.CurrAsmList,cpointerdef.getreusable(procdefinition),cpointerdef.getreusable(callprocdef),right.location.reference);
             hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,callprocdef,callprocdef,right.location.reference,reg);
           end;
       end;


     procedure tcgcallnode.load_procvar_codeptr(out reg: tregister; out callprocdef: tabstractprocdef);
       begin
         if po_is_block in procdefinition.procoptions then
           load_block_invoke(reg,callprocdef)
         else if not(procdefinition.is_addressonly) then
           load_complex_procvar_codeptr(reg,callprocdef)
         else
           begin
             reg:=hlcg.getaddressregister(current_asmdata.CurrAsmList,procdefinition);
             hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,procdefinition,procdefinition,right.location,reg);
             callprocdef:=procdefinition;
           end;
         callprocdef.init_paraloc_info(callerside);
       end;


     procedure tcgcallnode.freeparas;
       var
         ppn : tcgcallparanode;
       begin
         { free the resources allocated for the parameters }
         ppn:=tcgcallparanode(left);
         while assigned(ppn) do
           begin
             if (ppn.left.nodetype<>nothingn) then
               begin
                 if (ppn.parasym.paraloc[callerside].location^.loc <> LOC_REFERENCE) then
                   paramanager.freecgpara(current_asmdata.CurrAsmList,ppn.parasym.paraloc[callerside]);
               end;
             ppn:=tcgcallparanode(ppn.right);
           end;
       end;



    procedure tcgcallnode.pass_generate_code;
      var
        name_to_call: TSymStr;
        regs_to_save_int,
        regs_to_save_address,
        regs_to_save_fpu,
        regs_to_save_mm   : Tcpuregisterset;
        href : treference;
        pop_size : longint;
        pvreg : tregister;
        oldaktcallnode : tcallnode;
        retlocitem: pcgparalocation;
        callpvdef: tabstractprocdef;
        pd : tprocdef;
        callref: boolean;
{$ifdef vtentry}
        sym : tasmsymbol;
        vmtoffset : aint;
{$endif vtentry}
        cgpara : tcgpara;
        tmploc: tlocation;
      begin
         if not assigned(procdefinition) or
            not(procdefinition.has_paraloc_info in [callerside,callbothsides]) then
           internalerror(200305264);

         extra_pre_call_code;

         if assigned(callinitblock) then
           secondpass(tnode(callinitblock));

         regs_to_save_int:=paramanager.get_volatile_registers_int(procdefinition.proccalloption);
         regs_to_save_address:=paramanager.get_volatile_registers_address(procdefinition.proccalloption);
         regs_to_save_fpu:=paramanager.get_volatile_registers_fpu(procdefinition.proccalloption);
         regs_to_save_mm:=paramanager.get_volatile_registers_mm(procdefinition.proccalloption);

         { Include Function result registers }
         if (not is_void(resultdef)) then
          begin
            { The forced returntype may have a different size than the one
              declared for the procdef }
            retloc:=hlcg.get_call_result_cgpara(procdefinition,typedef);
            retlocitem:=retloc.location;
            while assigned(retlocitem) do
              begin
                case retlocitem^.loc of
                  LOC_REGISTER:
                    case getregtype(retlocitem^.register) of
                      R_INTREGISTER:
                        include(regs_to_save_int,getsupreg(retlocitem^.register));
                      R_ADDRESSREGISTER:
                        include(regs_to_save_address,getsupreg(retlocitem^.register));
                      R_TEMPREGISTER:
                        ;
                      else
                        internalerror(2014020102);
                      end;
                  LOC_FPUREGISTER:
                    include(regs_to_save_fpu,getsupreg(retlocitem^.register));
                  LOC_MMREGISTER:
                    include(regs_to_save_mm,getsupreg(retlocitem^.register));
                  LOC_REFERENCE,
                  LOC_VOID:
                    ;
                  else
                    internalerror(2004110213);
                end;
                retlocitem:=retlocitem^.next;
              end;
          end;

         { Process parameters, register parameters will be loaded
           in imaginary registers. The actual load to the correct
           register is done just before the call }
         oldaktcallnode:=aktcallnode;
         aktcallnode:=self;
         if assigned(left) then
           tcallparanode(left).secondcallparan;
         aktcallnode:=oldaktcallnode;

         { procedure variable or normal function call ? }
         if (right=nil) then
           begin
             { register call for WPO (must be done before wpo test below,
               otherwise optimised called methods are no longer registered)
             }
             if (po_virtualmethod in procdefinition.procoptions) and
                not is_objectpascal_helper(tprocdef(procdefinition).struct) and
                assigned(methodpointer) and
                (methodpointer.nodetype<>typen) and
                (not assigned(current_procinfo) or
                 wpoinfomanager.symbol_live(current_procinfo.procdef.mangledname)) then
               tobjectdef(tprocdef(procdefinition).struct).register_vmt_call(tprocdef(procdefinition).extnumber);
{$ifdef vtentry}
             if not is_interface(tprocdef(procdefinition)._class) then
               begin
                 inc(current_asmdata.NextVTEntryNr);
                 current_asmdata.CurrAsmList.Concat(tai_symbol.CreateName('VTREF'+tostr(current_asmdata.NextVTEntryNr)+'_'+tprocdef(procdefinition).struct.vmt_mangledname+'$$'+tostr(vmtoffset div sizeof(pint)),AT_FUNCTION,0,voidpointerdef));
               end;
{$endif vtentry}

             name_to_call:=forcedprocname;
             { When methodpointer is typen we don't need (and can't) load
               a pointer. We can directly call the correct procdef (PFV) }
             if (name_to_call='') and
                (po_virtualmethod in procdefinition.procoptions) and
                not is_objectpascal_helper(tprocdef(procdefinition).struct) and
                assigned(methodpointer) and
                (methodpointer.nodetype<>typen) and
                not wpoinfomanager.can_be_devirtualized(methodpointer.resultdef,procdefinition,name_to_call) then
               begin
                 { virtual methods require an index }
                 if tprocdef(procdefinition).extnumber=$ffff then
                   internalerror(200304021);

                 { load the VMT entry (address of the virtual method) }
                 secondpass(vmt_entry);

                 { register call for WPO }
                 if (not assigned(current_procinfo) or
                     wpoinfomanager.symbol_live(current_procinfo.procdef.mangledname)) then
                   tobjectdef(tprocdef(procdefinition).struct).register_vmt_call(tprocdef(procdefinition).extnumber);

                 if not(vmt_entry.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                   internalerror(2015052502);
                 href:=vmt_entry.location.reference;
                 pvreg:=NR_NO;

                 callref:=can_call_ref(href);
                 if not callref then
                   begin
                     pvreg:=get_call_reg(current_asmdata.CurrAsmList);
                     hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,
                       vmt_entry.resultdef,vmt_entry.resultdef,
                       href,pvreg);
                   end;

                 { Load parameters that are in temporary registers in the
                   correct parameter register }
                 if assigned(left) then
                   begin
                     reorder_parameters;
                     pushparas;
                   end;

                 if callref then
                   extra_call_ref_code(href);

                 cg.alloccpuregisters(current_asmdata.CurrAsmList,R_INTREGISTER,regs_to_save_int);
                 if cg.uses_registers(R_ADDRESSREGISTER) then
                   cg.alloccpuregisters(current_asmdata.CurrAsmList,R_ADDRESSREGISTER,regs_to_save_address);
                 if cg.uses_registers(R_FPUREGISTER) then
                   cg.alloccpuregisters(current_asmdata.CurrAsmList,R_FPUREGISTER,regs_to_save_fpu);
                 if cg.uses_registers(R_MMREGISTER) then
                   cg.alloccpuregisters(current_asmdata.CurrAsmList,R_MMREGISTER,regs_to_save_mm);

                 { call method }
                 extra_call_code;
                 retloc.resetiftemp;
                 if callref then
                   retloc:=do_call_ref(href)
                 else
                   begin
                     hlcg.g_ptrtypecast_reg(current_asmdata.CurrAsmList,vmt_entry.resultdef,cpointerdef.getreusable(procdefinition),pvreg);
                     retloc:=hlcg.a_call_reg(current_asmdata.CurrAsmList,tabstractprocdef(procdefinition),pvreg,paralocs);
                     unget_call_reg(current_asmdata.CurrAsmList,pvreg);
                   end;

                 extra_post_call_code;
               end
             else
               begin
                  { Load parameters that are in temporary registers in the
                    correct parameter register }
                  if assigned(left) then
                    begin
                      reorder_parameters;
                      pushparas;
                    end;

                  cg.alloccpuregisters(current_asmdata.CurrAsmList,R_INTREGISTER,regs_to_save_int);
                  if cg.uses_registers(R_ADDRESSREGISTER) then
                    cg.alloccpuregisters(current_asmdata.CurrAsmList,R_ADDRESSREGISTER,regs_to_save_address);
                  if cg.uses_registers(R_FPUREGISTER) then
                    cg.alloccpuregisters(current_asmdata.CurrAsmList,R_FPUREGISTER,regs_to_save_fpu);
                  if cg.uses_registers(R_MMREGISTER) then
                    cg.alloccpuregisters(current_asmdata.CurrAsmList,R_MMREGISTER,regs_to_save_mm);

                  if procdefinition.proccalloption=pocall_syscall then
                    do_syscall
                  else
                    begin
                      { Calling interrupt from the same code requires some
                        extra code }
                      if (po_interrupt in procdefinition.procoptions) then
                        extra_interrupt_code;
                      extra_call_code;
                      retloc.resetiftemp;
                      if (name_to_call='') then
                        name_to_call:=tprocdef(procdefinition).mangledname;
                      if cnf_inherited in callnodeflags then
                        retloc:=hlcg.a_call_name_inherited(current_asmdata.CurrAsmList,tprocdef(procdefinition),name_to_call,paralocs)
                      { under certain conditions, a static call (i.e. without PIC) can be generated }
                      else if ((procdefinition.owner=current_procinfo.procdef.owner) or
                         (procdefinition.owner.symtabletype in [localsymtable,staticsymtable])
                        ) and ((procdefinition.procoptions*[po_weakexternal,po_external])=[]) then
                        retloc:=hlcg.a_call_name_static(current_asmdata.CurrAsmList,tprocdef(procdefinition),name_to_call,paralocs,typedef)
                      else
                        retloc:=hlcg.a_call_name(current_asmdata.CurrAsmList,tprocdef(procdefinition),name_to_call,paralocs,typedef,po_weakexternal in procdefinition.procoptions);
                      extra_post_call_code;
                    end;
               end;
           end
         else
           { now procedure variable case }
           begin
              secondpass(right);

              { can we directly call the procvar in a memory location? }
              callref:=false;
              if not(po_is_block in procdefinition.procoptions) and
                 (right.location.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                begin
                  href:=right.location.reference;
                  callref:=can_call_ref(href);
                end;

              if not callref then
                load_procvar_codeptr(pvreg,callpvdef)
              else
                begin
                  pvreg:=NR_INVALID;
                  callpvdef:=nil;
                end;
              location_freetemp(current_asmdata.CurrAsmList,right.location);

              { Load parameters that are in temporary registers in the
                correct parameter register }
              if assigned(left) then
                begin
                  reorder_parameters;
                  pushparas;
                end;

              if callref then
                extra_call_ref_code(href);

              cg.alloccpuregisters(current_asmdata.CurrAsmList,R_INTREGISTER,regs_to_save_int);
              if cg.uses_registers(R_ADDRESSREGISTER) then
                cg.alloccpuregisters(current_asmdata.CurrAsmList,R_ADDRESSREGISTER,regs_to_save_address);
              if cg.uses_registers(R_FPUREGISTER) then
                cg.alloccpuregisters(current_asmdata.CurrAsmList,R_FPUREGISTER,regs_to_save_fpu);
              if cg.uses_registers(R_MMREGISTER) then
                cg.alloccpuregisters(current_asmdata.CurrAsmList,R_MMREGISTER,regs_to_save_mm);

              { Calling interrupt from the same code requires some
                extra code }
              if (po_interrupt in procdefinition.procoptions) then
                extra_interrupt_code;
              extra_call_code;

              retloc.resetiftemp;
              if callref then
                retloc:=do_call_ref(href)
              else
                retloc:=hlcg.a_call_reg(current_asmdata.CurrAsmList,callpvdef,pvreg,paralocs);
              extra_post_call_code;
           end;

         { free the resources allocated for the parameters }
         if assigned(left) then
           freeparas;

         { Need to remove the parameters from the stack? }
         if procdefinition.proccalloption in clearstack_pocalls then
           begin
             pop_size:=pushedparasize;
             { for Cdecl functions we don't need to pop the funcret when it
               was pushed by para. Except for safecall functions with
               safecall-exceptions enabled. In that case the funcret is always
               returned as a para which is considered a normal para on the
               c-side, so the funcret has to be pop'ed normally. }
             if not ((procdefinition.proccalloption=pocall_safecall) and
                     (tf_safecall_exceptions in target_info.flags)) and
                paramanager.ret_in_param(procdefinition.returndef,procdefinition) then
               dec(pop_size,sizeof(pint));
             { Remove parameters/alignment from the stack }
             pop_parasize(pop_size);
           end
         { in case we use a fixed stack, we did not push anything, if the stack is
           really adjusted because a ret xxx was done, depends on
           pop_parasize which uses pushedparasize to determine this

           This does not apply to interrupt procedures, their ret statment never clears any stack parameters }
         else if paramanager.use_fixed_stack and
                 not(po_interrupt in procdefinition.procoptions) and
                 (target_info.abi=abi_i386_dynalignedstack) then
           begin
             { however, a delphi style frame pointer for a nested subroutine
               is not cleared by the callee, so we have to compensate for this
               by passing 4 as pushedparasize does include it }
             if po_delphi_nested_cc in procdefinition.procoptions then
               pop_parasize(sizeof(pint))
             else
               pop_parasize(0);
           end
         { frame pointer parameter is popped by the caller when it's passed the
           Delphi way }
         else if (po_delphi_nested_cc in procdefinition.procoptions) and
           not paramanager.use_fixed_stack then
           pop_parasize(sizeof(pint));
         { Release registers, but not the registers that contain the
           function result }
         if (not is_void(resultdef)) then
           begin
             retlocitem:=retloc.location;
             while assigned(retlocitem) do
               begin
                 case retlocitem^.loc of
                   LOC_REGISTER:
                     case getregtype(retlocitem^.register) of
                       R_INTREGISTER:
                         exclude(regs_to_save_int,getsupreg(retlocitem^.register));
                       R_ADDRESSREGISTER:
                         exclude(regs_to_save_address,getsupreg(retlocitem^.register));
                       R_TEMPREGISTER:
                         ;
                       else
                         internalerror(2014020103);
                     end;
                   LOC_FPUREGISTER:
                     exclude(regs_to_save_fpu,getsupreg(retlocitem^.register));
                   LOC_MMREGISTER:
                     exclude(regs_to_save_mm,getsupreg(retlocitem^.register));
                   LOC_REFERENCE,
                   LOC_VOID:
                     ;
                   else
                     internalerror(2004110214);
                 end;
                 retlocitem:=retlocitem^.next;
               end;
           end;

         if cg.uses_registers(R_MMREGISTER) then
           cg.dealloccpuregisters(current_asmdata.CurrAsmList,R_MMREGISTER,regs_to_save_mm);
         if cg.uses_registers(R_FPUREGISTER) then
           cg.dealloccpuregisters(current_asmdata.CurrAsmList,R_FPUREGISTER,regs_to_save_fpu);
         if cg.uses_registers(R_ADDRESSREGISTER) then
           cg.dealloccpuregisters(current_asmdata.CurrAsmList,R_ADDRESSREGISTER,regs_to_save_address);
         cg.dealloccpuregisters(current_asmdata.CurrAsmList,R_INTREGISTER,regs_to_save_int);

         if procdefinition.generate_safecall_wrapper then
           begin
             pd:=search_system_proc('fpc_safecallcheck');
             cgpara.init;
             { fpc_safecallcheck returns its parameter value (= function result of function we just called) }
             paramanager.getcgtempparaloc(current_asmdata.CurrAsmList,pd,1,cgpara);
             location_reset(tmploc,LOC_REGISTER,def_cgsize(retloc.Def));
             tmploc.register:=hlcg.getregisterfordef(current_asmdata.CurrAsmList,retloc.Def);
             hlcg.gen_load_cgpara_loc(current_asmdata.CurrAsmList,retloc.Def,retloc,tmploc,true);
             paramanager.freecgpara(current_asmdata.CurrAsmList,cgpara);
             hlcg.a_load_loc_cgpara(current_asmdata.CurrAsmList,retloc.Def,tmploc,cgpara);
             retloc.resetiftemp;
             retloc:=hlcg.g_call_system_proc(current_asmdata.CurrAsmList,pd,[@cgpara],nil);
             cgpara.done;
           end;

         { handle function results }
         if (not is_void(resultdef)) then
           handle_return_value
         else
           location_reset(location,LOC_VOID,OS_NO);

         { convert persistent temps for parameters and function result to normal temps }
         if assigned(callcleanupblock) then
           secondpass(tnode(callcleanupblock));

         { copy back copy-out parameters if any }
         copy_back_paras;

         { release temps and finalize unused return values, must be
           after the callcleanupblock because that converts temps
           from persistent to normal }
         release_unused_return_value;

         { release temps of paras }
         release_para_temps;

         { check for fpu exceptions }
         if cnf_check_fpu_exceptions in callnodeflags then
           cg.maybe_check_for_fpu_exception(current_asmdata.CurrAsmList);

         { perhaps i/o check ? }
         if (cs_check_io in current_settings.localswitches) and
            (po_iocheck in procdefinition.procoptions) and
            not(po_iocheck in current_procinfo.procdef.procoptions) and
            { no IO check for methods and procedure variables }
            (right=nil) and
            not(po_virtualmethod in procdefinition.procoptions) then
           begin
              hlcg.g_call_system_proc(current_asmdata.CurrAsmList,'fpc_iocheck',[],nil).resetiftemp;
           end;
      end;


    destructor tcgcallnode.destroy;
      begin
        retloc.resetiftemp;
        inherited destroy;
      end;


begin
   ccallparanode:=tcgcallparanode;
   ccallnode:=tcgcallnode;
end.
