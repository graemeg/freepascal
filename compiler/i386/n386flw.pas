{
    Copyright (c) 2011 by Free Pascal development team

    Generate Win32-specific exception handling code

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
unit n386flw;

{$i fpcdefs.inc}

interface

  uses
    node,nflw,ncgflw,psub;

  type
    ti386raisenode=class(tcgraisenode)
      function pass_1 : tnode;override;
    end;

    ti386onnode=class(tcgonnode)
      procedure pass_generate_code;override;
    end;

    ti386tryexceptnode=class(tcgtryexceptnode)
      procedure pass_generate_code;override;
    end;

    ti386tryfinallynode=class(tcgtryfinallynode)
      finalizepi: tcgprocinfo;
      constructor create(l,r:TNode);override;
      constructor create_implicit(l,r:TNode);override;
      function pass_1: tnode;override;
      function simplify(forinline: boolean): tnode;override;
      procedure pass_generate_code;override;
    end;

implementation

  uses
    cutils,globtype,globals,verbose,systems,fmodule,
    nbas,ncal,nmem,nutils,
    symconst,symbase,symtable,symsym,symdef,
    cgbase,cgobj,cgcpu,cgutils,tgobj,
    cpubase,htypechk,
    parabase,paramgr,pass_1,pass_2,ncgutil,cga,
    aasmbase,aasmtai,aasmdata,aasmcpu,procinfo,cpupi;

  var
    endexceptlabel: tasmlabel;


{ ti386raisenode }

function ti386raisenode.pass_1 : tnode;
  var
    statements : tstatementnode;
    raisenode : tcallnode;
  begin
    { difference from generic code is that address stack is not popped on reraise }
    if (target_info.system<>system_i386_win32) or assigned(left) then
      result:=inherited pass_1
    else
      begin
        result:=internalstatements(statements);
        raisenode:=ccallnode.createintern('fpc_reraise',nil);
        include(raisenode.callnodeflags,cnf_call_never_returns);
        addstatement(statements,raisenode);
      end;
end;

{ ti386onnode }

procedure ti386onnode.pass_generate_code;
  var
    oldflowcontrol : tflowcontrol;
    exceptvarsym : tlocalvarsym;
  begin
    if (target_info.system<>system_i386_win32) then
      begin
        inherited pass_generate_code;
        exit;
      end;

    location_reset(location,LOC_VOID,OS_NO);

    oldflowcontrol:=flowcontrol;
    flowcontrol:=[fc_inflowcontrol];

    { RTL will put exceptobject into EAX when jumping here }
    cg.a_reg_alloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);

    { Retrieve exception variable }
    if assigned(excepTSymtable) then
      exceptvarsym:=tlocalvarsym(excepTSymtable.SymList[0])
    else
      exceptvarsym:=nil;

    if assigned(exceptvarsym) then
      begin
        exceptvarsym.localloc.loc:=LOC_REFERENCE;
        exceptvarsym.localloc.size:=OS_ADDR;
        tg.GetLocal(current_asmdata.CurrAsmList,sizeof(pint),voidpointertype,exceptvarsym.localloc.reference);
        cg.a_load_reg_ref(current_asmdata.CurrAsmList,OS_ADDR,OS_ADDR,NR_FUNCTION_RESULT_REG,exceptvarsym.localloc.reference);
      end;
    cg.a_reg_dealloc(current_asmdata.CurrAsmList,NR_FUNCTION_RESULT_REG);

    if assigned(right) then
      secondpass(right);

    { deallocate exception symbol }
    if assigned(exceptvarsym) then
      begin
        tg.UngetLocal(current_asmdata.CurrAsmList,exceptvarsym.localloc.reference);
        exceptvarsym.localloc.loc:=LOC_INVALID;
      end;
    cg.g_call(current_asmdata.CurrAsmList,'FPC_DONEEXCEPTION');
    cg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);

    flowcontrol:=oldflowcontrol+(flowcontrol-[fc_inflowcontrol]);
  end;

{ ti386tryfinallynode }

function reset_regvars(var n: tnode; arg: pointer): foreachnoderesult;
  begin
    case n.nodetype of
      temprefn:
        make_not_regable(n,[]);
      calln:
        include(tprocinfo(arg).flags,pi_do_call);
      else ;
    end;
    result:=fen_true;
  end;

function copy_parasize(var n: tnode; arg: pointer): foreachnoderesult;
  begin
    if n.nodetype=calln then
        tcgprocinfo(arg).allocate_push_parasize(tcallnode(n).pushed_parasize);
    result:=fen_true;
  end;

constructor ti386tryfinallynode.create(l, r: TNode);
  begin
    inherited create(l,r);
    if (target_info.system<>system_i386_win32) or
      { Don't create child procedures for generic methods, their nested-like
        behavior causes compilation errors because real nested procedures
        aren't allowed for generics. Not creating them doesn't harm because
        generic node tree is discarded without generating code. }
      (df_generic in current_procinfo.procdef.defoptions)
      then
      exit;
    finalizepi:=tcgprocinfo(current_procinfo.create_for_outlining('$fin$',current_procinfo.procdef.struct,potype_exceptfilter,voidtype,r));
    { Regvar optimization for symbols is suppressed when using exceptions, but
      temps may be still placed into registers. This must be fixed. }
    foreachnodestatic(r,@reset_regvars,finalizepi);
    include(finalizepi.flags,pi_has_assembler_block);
    include(finalizepi.flags,pi_do_call);
    include(finalizepi.flags,pi_uses_exceptions);
  end;

constructor ti386tryfinallynode.create_implicit(l, r: TNode);
  begin
    inherited create_implicit(l, r);
    if (target_info.system<>system_i386_win32) then
      exit;

    { safecall procedures can handle implicit finalization as part of "except" flow }
    if implicitframe and (current_procinfo.procdef.proccalloption=pocall_safecall) then
      exit;

    if df_generic in current_procinfo.procdef.defoptions then
      InternalError(2013012501);

    finalizepi:=tcgprocinfo(current_procinfo.create_for_outlining('$fin$',current_procinfo.procdef.struct,potype_exceptfilter,voidtype,r));
    include(finalizepi.flags,pi_has_assembler_block);
    include(finalizepi.flags,pi_do_call);
    include(finalizepi.flags,pi_uses_exceptions);
  end;


function ti386tryfinallynode.pass_1: tnode;
  var
    selfsym: tparavarsym;
  begin
    result:=inherited pass_1;
    if (target_info.system=system_i386_win32) then
      begin
        { safecall method will access 'self' from except block -> make it non-regable }
        if implicitframe and (current_procinfo.procdef.proccalloption=pocall_safecall) and
          is_class(current_procinfo.procdef.struct) then
          begin
            selfsym:=tparavarsym(current_procinfo.procdef.parast.Find('self'));
            if (selfsym=nil) or (selfsym.typ<>paravarsym) then
              InternalError(2011123101);
            selfsym.varregable:=vr_none;
          end;
      end;
  end;


function ti386tryfinallynode.simplify(forinline: boolean): tnode;
  begin
    result:=inherited simplify(forinline);
    if (target_info.system<>system_i386_win32) then
      exit;

    if (result=nil) and assigned(finalizepi) then
      begin
        finalizepi.code:=right;
        foreachnodestatic(right,@copy_parasize,finalizepi);
        right:=ccallnode.create(nil,tprocsym(finalizepi.procdef.procsym),nil,nil,[],nil);
        firstpass(right);
        { For implicit frames, no actual code is available at this time,
          it is added later in assembler form. So store the nested procinfo
          for later use. }
        if implicitframe then
          begin
            current_procinfo.finalize_procinfo:=finalizepi;
          end;
      end;
  end;


procedure emit_scope_start(handler,data: TAsmSymbol);
  var
    href: treference;
    hreg: tregister;
  begin
    hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
    reference_reset_base(href,hreg,0,ctempposinvalid,sizeof(pint),[]);
    href.segment:=NR_FS;
    emit_reg_reg(A_XOR,S_L,hreg,hreg);
    emit_sym(A_PUSH,S_L,data);
    emit_reg(A_PUSH,S_L,NR_FRAME_POINTER_REG);
    emit_sym(A_PUSH,S_L,handler);
    emit_ref(A_PUSH,S_L,href);
    emit_reg_ref(A_MOV,S_L,NR_ESP,href);
  end;

procedure emit_scope_end;
  var
    href: treference;
    hreg,hreg2: tregister;
  begin
    hreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
    hreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_ADDR);
    reference_reset_base(href,hreg,0,ctempposinvalid,sizeof(pint),[]);
    href.segment:=NR_FS;
    emit_reg_reg(A_XOR,S_L,hreg,hreg);
    emit_reg(A_POP,S_L,hreg2);
    emit_const_reg(A_ADD,S_L,3*sizeof(pint),NR_ESP);
    emit_reg_ref(A_MOV,S_L,hreg2,href);
  end;

procedure ti386tryfinallynode.pass_generate_code;
  var
    finallylabel,
    exceptlabel,
    safecalllabel,
    endfinallylabel,
    exitfinallylabel,
    continuefinallylabel,
    breakfinallylabel,
    oldCurrExitLabel,
    oldContinueLabel,
    oldBreakLabel : tasmlabel;
    oldflowcontrol,tryflowcontrol : tflowcontrol;
    is_safecall: boolean;
    sym : tasmsymbol;
  begin
    if (target_info.system<>system_i386_win32) then
      begin
        inherited pass_generate_code;
        exit;
      end;
    location_reset(location,LOC_VOID,OS_NO);
    tryflowcontrol:=[];
    oldBreakLabel:=nil;
    oldContinueLabel:=nil;
    continuefinallylabel:=nil;
    breakfinallylabel:=nil;
    exceptlabel:=nil;
    safecalllabel:=nil;
    is_safecall:=implicitframe and (current_procinfo.procdef.proccalloption=pocall_safecall);

    { check if child nodes do a break/continue/exit }
    oldflowcontrol:=flowcontrol;
    flowcontrol:=[fc_inflowcontrol];
    current_asmdata.getjumplabel(finallylabel);
    current_asmdata.getjumplabel(endfinallylabel);

    { the finally block must catch break, continue and exit }
    { statements                                            }
    oldCurrExitLabel:=current_procinfo.CurrExitLabel;
    if implicitframe then
      exitfinallylabel:=finallylabel
    else
      current_asmdata.getjumplabel(exitfinallylabel);
    current_procinfo.CurrExitLabel:=exitfinallylabel;
    if assigned(current_procinfo.CurrBreakLabel) then
      begin
        oldContinueLabel:=current_procinfo.CurrContinueLabel;
        oldBreakLabel:=current_procinfo.CurrBreakLabel;
        if implicitframe then
          begin
            breakfinallylabel:=finallylabel;
            continuefinallylabel:=finallylabel;
          end
        else
          begin
            current_asmdata.getjumplabel(breakfinallylabel);
            current_asmdata.getjumplabel(continuefinallylabel);
          end;
        current_procinfo.CurrContinueLabel:=continuefinallylabel;
        current_procinfo.CurrBreakLabel:=breakfinallylabel;
      end;

    { Start of scope }
    if is_safecall then
      begin
        with cg.rg[R_INTREGISTER] do
          used_in_proc:=used_in_proc+[RS_EBX,RS_ESI,RS_EDI];

        current_asmdata.getjumplabel(exceptlabel);
        sym:=current_asmdata.RefAsmSymbol('__FPC_except_safecall',AT_FUNCTION);
        emit_scope_start(
          sym,
          exceptlabel
        );
        current_module.add_extern_asmsym(sym);
      end
    else
      begin
        sym:=current_asmdata.RefAsmSymbol('__FPC_finally_handler',AT_FUNCTION);
        emit_scope_start(
          sym,
          current_asmdata.RefAsmSymbol(finalizepi.procdef.mangledname,AT_FUNCTION)
        );
        current_module.add_extern_asmsym(sym);
      end;

    { try code }
    if assigned(left) then
      begin
        secondpass(left);
        tryflowcontrol:=flowcontrol;
        if codegenerror then
          exit;
      end;

    { don't generate line info for internal cleanup }
    current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

    cg.a_label(current_asmdata.CurrAsmList,finallylabel);
    emit_scope_end;
    if is_safecall then
      begin
        current_asmdata.getjumplabel(safecalllabel);
        cg.a_jmp_always(current_asmdata.CurrAsmList,safecalllabel);
        { RTL handler will jump here on exception }
        cg.a_label(current_asmdata.CurrAsmList,exceptlabel);
        handle_safecall_exception;
        cg.a_label(current_asmdata.CurrAsmList,safecalllabel);
      end;

    { end cleanup }
    current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

    { generate finally code as a separate procedure }
    { !!! this resets flowcontrol, how to check flow away? }
    if not implicitframe then
      tcgprocinfo(current_procinfo).generate_exceptfilter(finalizepi);

    flowcontrol:=[fc_inflowcontrol];
    { right is a call to finalizer procedure }
    secondpass(right);

    { goto is allowed if it stays inside the finally block,
      this is checked using the exception block number }
    if (flowcontrol-[fc_gotolabel])<>[fc_inflowcontrol] then
      CGMessage(cg_e_control_flow_outside_finally);
    if codegenerror then
      exit;

    { don't generate line info for internal cleanup }
    current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoStart));

    if not implicitframe then
      begin
        if tryflowcontrol*[fc_exit,fc_break,fc_continue]<>[] then
          cg.a_jmp_always(current_asmdata.CurrAsmList,endfinallylabel);
        { do some magic for exit,break,continue in the try block }
        if fc_exit in tryflowcontrol then
          begin
            cg.a_label(current_asmdata.CurrAsmList,exitfinallylabel);
            cg.g_call(current_asmdata.CurrAsmList,'_FPC_leave');
            cg.a_jmp_always(current_asmdata.CurrAsmList,oldCurrExitLabel);
          end;
        if fc_break in tryflowcontrol then
          begin
            cg.a_label(current_asmdata.CurrAsmList,breakfinallylabel);
            cg.g_call(current_asmdata.CurrAsmList,'_FPC_leave');
            cg.a_jmp_always(current_asmdata.CurrAsmList,oldBreakLabel);
          end;
        if fc_continue in tryflowcontrol then
          begin
            cg.a_label(current_asmdata.CurrAsmList,continuefinallylabel);
            cg.g_call(current_asmdata.CurrAsmList,'_FPC_leave');
            cg.a_jmp_always(current_asmdata.CurrAsmList,oldContinueLabel);
          end;
      end;
    cg.a_label(current_asmdata.CurrAsmList,endfinallylabel);

    { end cleanup }
    current_asmdata.CurrAsmList.concat(tai_marker.create(mark_NoLineInfoEnd));

    current_procinfo.CurrExitLabel:=oldCurrExitLabel;
    if assigned(current_procinfo.CurrBreakLabel) then
      begin
        current_procinfo.CurrContinueLabel:=oldContinueLabel;
        current_procinfo.CurrBreakLabel:=oldBreakLabel;
      end;
    flowcontrol:=oldflowcontrol+(tryflowcontrol-[fc_inflowcontrol]);
  end;

{ ti386tryexceptnode }

procedure ti386tryexceptnode.pass_generate_code;
  var
    exceptlabel,oldendexceptlabel,
    lastonlabel,
    exitexceptlabel,
    continueexceptlabel,
    breakexceptlabel,
    exittrylabel,
    continuetrylabel,
    breaktrylabel,
    oldCurrExitLabel,
    oldContinueLabel,
    oldBreakLabel : tasmlabel;
    onlabel,
    filterlabel: tasmlabel;
    oldflowcontrol,tryflowcontrol,
    exceptflowcontrol : tflowcontrol;
    hnode : tnode;
    hlist : tasmlist;
    onnodecount : tai_const;
    sym : tasmsymbol;
  label
    errorexit;
  begin
    if (target_info.system<>system_i386_win32) then
      begin
        inherited pass_generate_code;
        exit;
      end;
    location_reset(location,LOC_VOID,OS_NO);

    exceptflowcontrol:=[];
    breakexceptlabel:=nil;
    continueexceptlabel:=nil;
    breaktrylabel:=nil;
    continuetrylabel:=nil;

    oldflowcontrol:=flowcontrol;
    flowcontrol:=[fc_inflowcontrol];
    { this can be called recursivly }
    oldBreakLabel:=nil;
    oldContinueLabel:=nil;
    oldendexceptlabel:=endexceptlabel;

    { Win32 SEH unwinding does not preserve registers. Indicate that they are
      going to be destroyed. }
    cg.alloccpuregisters(current_asmdata.CurrAsmList,R_INTREGISTER,[RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESI,RS_EDI]);
    cg.dealloccpuregisters(current_asmdata.CurrAsmList,R_INTREGISTER,[RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESI,RS_EDI]);

    { save the old labels for control flow statements }
    oldCurrExitLabel:=current_procinfo.CurrExitLabel;
    if assigned(current_procinfo.CurrBreakLabel) then
      begin
        oldContinueLabel:=current_procinfo.CurrContinueLabel;
        oldBreakLabel:=current_procinfo.CurrBreakLabel;
      end;

    { get new labels for the control flow statements }
    current_asmdata.getjumplabel(exittrylabel);
    current_asmdata.getjumplabel(exitexceptlabel);
    if assigned(current_procinfo.CurrBreakLabel) then
      begin
        current_asmdata.getjumplabel(breaktrylabel);
        current_asmdata.getjumplabel(continuetrylabel);
        current_asmdata.getjumplabel(breakexceptlabel);
        current_asmdata.getjumplabel(continueexceptlabel);
      end;

    current_asmdata.getjumplabel(exceptlabel);
    current_asmdata.getjumplabel(endexceptlabel);
    current_asmdata.getjumplabel(lastonlabel);
    filterlabel:=nil;

    { start of scope }
    if assigned(right) then
      begin
        current_asmdata.getaddrlabel(filterlabel);
        sym:=current_asmdata.RefAsmSymbol('__FPC_on_handler',AT_FUNCTION);
        emit_scope_start(
          sym,
          filterlabel);
        current_module.add_extern_asmsym(sym);
      end
    else
      begin
        sym:=current_asmdata.RefAsmSymbol('__FPC_except_handler',AT_FUNCTION);
        emit_scope_start(
          sym,
          exceptlabel);
        current_module.add_extern_asmsym(sym);
      end;

    { set control flow labels for the try block }
    current_procinfo.CurrExitLabel:=exittrylabel;
    if assigned(oldBreakLabel) then
      begin
        current_procinfo.CurrContinueLabel:=continuetrylabel;
        current_procinfo.CurrBreakLabel:=breaktrylabel;
      end;

    secondpass(left);
    tryflowcontrol:=flowcontrol;
    if codegenerror then
      goto errorexit;

    emit_scope_end;
    { jump over except handlers }
    cg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);

    if fc_exit in tryflowcontrol then
      begin
        cg.a_label(current_asmdata.CurrAsmList,exittrylabel);
        emit_scope_end;
        cg.a_jmp_always(current_asmdata.CurrAsmList,oldCurrExitLabel);
      end;
    if fc_break in tryflowcontrol then
      begin
        cg.a_label(current_asmdata.CurrAsmList,breaktrylabel);
        emit_scope_end;
        cg.a_jmp_always(current_asmdata.CurrAsmList,oldBreakLabel);
      end;
    if fc_continue in tryflowcontrol then
      begin
        cg.a_label(current_asmdata.CurrAsmList,continuetrylabel);
        emit_scope_end;
        cg.a_jmp_always(current_asmdata.CurrAsmList,oldContinueLabel);
      end;

    { target for catch-all handler }
    cg.a_label(current_asmdata.CurrAsmList,exceptlabel);

    { set control flow labels for the except block }
    { and the on statements                        }
    current_procinfo.CurrExitLabel:=exitexceptlabel;
    if assigned(oldBreakLabel) then
      begin
        current_procinfo.CurrContinueLabel:=continueexceptlabel;
        current_procinfo.CurrBreakLabel:=breakexceptlabel;
      end;

    flowcontrol:=[fc_inflowcontrol];
    { on statements }
    if assigned(right) then
      begin
        { emit filter table to a temporary asmlist }
        hlist:=TAsmList.Create;
        new_section(hlist,sec_rodata,filterlabel.name,4);
        cg.a_label(hlist,filterlabel);
        onnodecount:=tai_const.create_32bit(0);
        hlist.concat(onnodecount);

        hnode:=right;
        while assigned(hnode) do
          begin
            if hnode.nodetype<>onn then
              InternalError(2011103101);
            current_asmdata.getjumplabel(onlabel);
            sym:=current_asmdata.RefAsmSymbol(tonnode(hnode).excepttype.vmt_mangledname,AT_DATA,true);
            hlist.concat(tai_const.create_sym(sym));
            hlist.concat(tai_const.create_sym(onlabel));
            current_module.add_extern_asmsym(sym);
            cg.a_label(current_asmdata.CurrAsmList,onlabel);
            secondpass(hnode);
            inc(onnodecount.value);
            hnode:=tonnode(hnode).left;
          end;
        { add 'else' node to the filter list, too }
        if assigned(t1) then
          begin
            hlist.concat(tai_const.create_32bit(-1));
            hlist.concat(tai_const.create_sym(lastonlabel));
            inc(onnodecount.value);
          end;
        { now move filter table to permanent list all at once }
        current_procinfo.aktlocaldata.concatlist(hlist);
        hlist.free;
      end;

    cg.a_label(current_asmdata.CurrAsmList,lastonlabel);
    if assigned(t1) then
      begin
        { here we don't have to reset flowcontrol           }
        { the default and on flowcontrols are handled equal }
        secondpass(t1);
        cg.g_call(current_asmdata.CurrAsmList,'FPC_DONEEXCEPTION');
        if (flowcontrol*[fc_exit,fc_break,fc_continue]<>[]) then
          cg.a_jmp_always(current_asmdata.CurrAsmList,endexceptlabel);
      end;
    exceptflowcontrol:=flowcontrol;

    if fc_exit in exceptflowcontrol then
      begin
        { do some magic for exit in the try block }
        cg.a_label(current_asmdata.CurrAsmList,exitexceptlabel);
        cg.g_call(current_asmdata.CurrAsmList,'FPC_DONEEXCEPTION');
        cg.a_jmp_always(current_asmdata.CurrAsmList,oldCurrExitLabel);
      end;

    if fc_break in exceptflowcontrol then
      begin
        cg.a_label(current_asmdata.CurrAsmList,breakexceptlabel);
        cg.g_call(current_asmdata.CurrAsmList,'FPC_DONEEXCEPTION');
        cg.a_jmp_always(current_asmdata.CurrAsmList,oldBreakLabel);
      end;

    if fc_continue in exceptflowcontrol then
      begin
        cg.a_label(current_asmdata.CurrAsmList,continueexceptlabel);
        cg.g_call(current_asmdata.CurrAsmList,'FPC_DONEEXCEPTION');
        cg.a_jmp_always(current_asmdata.CurrAsmList,oldContinueLabel);
      end;

    cg.a_label(current_asmdata.CurrAsmList,endexceptlabel);

errorexit:
    { restore all saved labels }
    endexceptlabel:=oldendexceptlabel;

    { restore the control flow labels }
    current_procinfo.CurrExitLabel:=oldCurrExitLabel;
    if assigned(oldBreakLabel) then
      begin
        current_procinfo.CurrContinueLabel:=oldContinueLabel;
        current_procinfo.CurrBreakLabel:=oldBreakLabel;
      end;

    { return all used control flow statements }
    flowcontrol:=oldflowcontrol+(exceptflowcontrol +
      tryflowcontrol - [fc_inflowcontrol]);
  end;

initialization
  craisenode:=ti386raisenode;
  connode:=ti386onnode;
  ctryexceptnode:=ti386tryexceptnode;
  ctryfinallynode:=ti386tryfinallynode;
end.
