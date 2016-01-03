{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Type checking and register allocation for nodes that influence
    the flow

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
unit nflw;

{$i fpcdefs.inc}

interface

    uses
      cclasses,
      node,cpubase,
      symtype,symbase,symdef,symsym,
      optloop;

    type
       { flags used by loop nodes }
       tloopflag = (
         { set if it is a for ... downto ... do loop }
         lnf_backward,
         { Do we need to parse childs to set var state? }
         lnf_varstate,
         { Do a test at the begin of the loop?}
         lnf_testatbegin,
         { Negate the loop test? }
         lnf_checknegate,
         { Should the value of the loop variable on exit be correct. }
         lnf_dont_mind_loopvar_on_exit,
         { Loop simplify flag }
         lnf_simplify_processing);
       tloopflags = set of tloopflag;

    const
         { loop flags which must match to consider loop nodes equal regarding the flags }
         loopflagsequal = [lnf_backward];

    type
       tlabelnode = class;

       tloopnode = class(tbinarynode)
          t1,t2 : tnode;
          loopflags : tloopflags;
          constructor create(tt : tnodetype;l,r,_t1,_t2 : tnode);virtual;
          destructor destroy;override;
          function dogetcopy : tnode;override;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure derefnode;override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          procedure insertintolist(l : tnodelist);override;
          procedure printnodetree(var t:text);override;
          function docompare(p: tnode): boolean; override;
       end;

       twhilerepeatnode = class(tloopnode)
          constructor create(l,r:Tnode;tab,cn:boolean);virtual;reintroduce;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
{$ifdef state_tracking}
          function track_state_pass(exec_known:boolean):boolean;override;
{$endif}
       end;
       twhilerepeatnodeclass = class of twhilerepeatnode;

       tifnode = class(tloopnode)
          constructor create(l,r,_t1 : tnode);virtual;reintroduce;
          constructor create_internal(l,r,_t1 : tnode);virtual;reintroduce;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
          function simplify(forinline : boolean) : tnode;override;
         private
          function internalsimplify(warn: boolean) : tnode;
       end;
       tifnodeclass = class of tifnode;

       tfornode = class(tloopnode)
          { if count isn divisable by unrolls then
            the for loop must jump to this label to get the correct
            number of executions }
          entrylabel,
          { this is a dummy node used by the dfa to store life information for the loop iteration }
          loopiteration : tnode;
          loopvar_notid:cardinal;
          constructor create(l,r,_t1,_t2 : tnode;back : boolean);virtual;reintroduce;
          function wrap_to_value:tnode;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
          function simplify(forinline : boolean) : tnode;override;
       end;
       tfornodeclass = class of tfornode;

       texitnode = class(tunarynode)
          constructor create(l:tnode);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
          property resultexpr : tnode read left write left;
       end;
       texitnodeclass = class of texitnode;

       tbreaknode = class(tnode)
          constructor create;virtual;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
       end;
       tbreaknodeclass = class of tbreaknode;

       tcontinuenode = class(tnode)
          constructor create;virtual;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
       end;
       tcontinuenodeclass = class of tcontinuenode;

       tgotonode = class(tnode)
<<<<<<< HEAD
<<<<<<< HEAD
       private
          labelnodeidx : longint;
       public
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          labelsym : tlabelsym;
          labelnode : tlabelnode;
          exceptionblock : integer;
          constructor create(p : tlabelsym);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          procedure resolveppuidx;override;
          function dogetcopy : tnode;override;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
          function docompare(p: tnode): boolean; override;
       end;
       tgotonodeclass = class of tgotonode;

       tlabelnode = class(tunarynode)
          exceptionblock : integer;
          { when copying trees, this points to the newly created copy of a label }
          copiedto : tlabelnode;
          labsym : tlabelsym;
          constructor create(l:tnode;alabsym:tlabelsym);virtual;
          destructor destroy;override;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function dogetcopy : tnode;override;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
          function docompare(p: tnode): boolean; override;
       end;
       tlabelnodeclass = class of tlabelnode;

       traisenode = class(ttertiarynode)
          constructor create(l,taddr,tframe:tnode);virtual;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
       end;
       traisenodeclass = class of traisenode;

       ttryexceptnode = class(tloopnode)
          constructor create(l,r,_t1 : tnode);virtual;reintroduce;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
          function simplify(forinline: boolean): tnode; override;
         protected
          procedure adjust_estimated_stack_size; virtual;
       end;
       ttryexceptnodeclass = class of ttryexceptnode;

       ttryfinallynode = class(tloopnode)
          implicitframe : boolean;
          constructor create(l,r:tnode);virtual;reintroduce;
          constructor create_implicit(l,r,_t1:tnode);virtual;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
          function simplify(forinline:boolean): tnode;override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       protected
          function create_finalizer_procdef: tprocdef;
          procedure adjust_estimated_stack_size; virtual;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       end;
       ttryfinallynodeclass = class of ttryfinallynode;

       tonnode = class(tbinarynode)
          excepTSymtable : TSymtable;
          excepttype : tobjectdef;
          constructor create(l,r:tnode);virtual;
          destructor destroy;override;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          function pass_typecheck:tnode;override;
          function pass_1 : tnode;override;
          function dogetcopy : tnode;override;
          function docompare(p: tnode): boolean; override;
       end;
       tonnodeclass = class of tonnode;

    var
       cwhilerepeatnode : twhilerepeatnodeclass=twhilerepeatnode;
       cifnode : tifnodeclass = tifnode;
       cfornode : tfornodeclass = tfornode;
       cexitnode : texitnodeclass = texitnode;
       cgotonode : tgotonodeclass = tgotonode;
       clabelnode : tlabelnodeclass = tlabelnode;
       craisenode : traisenodeclass = traisenode;
       ctryexceptnode : ttryexceptnodeclass = ttryexceptnode;
       ctryfinallynode : ttryfinallynodeclass = ttryfinallynode;
       connode : tonnodeclass = tonnode;
       cbreaknode : tbreaknodeclass = tbreaknode;
       ccontinuenode : tcontinuenodeclass = tcontinuenode;

    // for-in loop helpers
    function create_type_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
    function create_string_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
    function create_array_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
    function create_set_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
    function create_enumerator_for_in_loop(hloopvar, hloopbody, expr: tnode;
       enumerator_get, enumerator_move: tprocdef; enumerator_current: tpropertysym): tnode;
    function create_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;

implementation

    uses
      globtype,systems,constexp,
      cutils,verbose,globals,
      symconst,symtable,paramgr,defcmp,defutil,htypechk,pass_1,
      ncal,nadd,ncon,nmem,nld,ncnv,nbas,cgobj,nutils,ninl,nset,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      pdecsub,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    {$ifdef state_tracking}
      nstate,
    {$endif}
    {$ifdef i8086}
      cpuinfo,
    {$endif i8086}
      cgbase,procinfo
      ;


    // for-in loop helpers

    function create_type_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
      begin
        result:=cfornode.create(hloopvar,
          cinlinenode.create(in_low_x,false,expr.getcopy),
          cinlinenode.create(in_high_x,false,expr.getcopy),
          hloopbody,
          false);
      end;


    function create_objc_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
      var
        mainstatement, outerloopbodystatement, innerloopbodystatement, tempstatement: tstatementnode;
        state, mutationcheck, currentamount, innerloopcounter, items, expressiontemp: ttempcreatenode;
        outerloop, innerloop, hp: tnode;
        itemsarraydef: tarraydef;
        sym: tsym;
      begin
        { Objective-C enumerators require Objective-C 2.0 }
        if not(m_objectivec2 in current_settings.modeswitches) then
          begin
            result:=cerrornode.create;
            MessagePos(expr.fileinfo,parser_e_objc_enumerator_2_0);
            exit;
          end;
        { Requires the NSFastEnumeration protocol and NSFastEnumerationState
          record }
        maybeloadcocoatypes;
        if not assigned(objc_fastenumeration) or
           not assigned(objc_fastenumerationstate) then
          begin
            result:=cerrornode.create;
            MessagePos(expr.fileinfo,parser_e_objc_missing_enumeration_defs);
            exit;
          end;

        (* Original code:
            for hloopvar in expression do
              <hloopbody>

          Pascal code equivalent into which it has to be transformed
          (sure would be nice if the compiler had some kind of templates ;) :
            var
              state: NSFastEnumerationState;
              expressiontemp: NSFastEnumerationProtocol;
              mutationcheck,
              currentamount,
              innerloopcounter: culong;
              { size can be increased/decreased if desired }
              items: array[1..16] of id;
            begin
              fillchar(state,sizeof(state),0);
              expressiontemp:=expression;
              repeat
                currentamount:=expressiontemp.countByEnumeratingWithState_objects_count(@state,@items,length(items));
                if currentamount=0 then
                  begin
                    { "The iterating variable is set to nil when the loop ends by
                      exhausting the source pool of objects" }
                    hloopvar:=nil;
                    break;
                  end;
                mutationcheck:=state.mutationsptr^;
                innerloopcounter:=culong(-1);
                repeat
                  { at the start so that "continue" in <loopbody> works correctly }
                  { don't use for-loop, because then the value of the iteration
                    counter is undefined on exit and we have to check it in the
                    outer repeat/until condition }
                  {$push}
                  {$r-,q-}
                  inc(innerloopcounter);
                  {$pop}
                  if innerloopcounter=currentamount then
                    break;
                  if mutationcheck<>state.mutationsptr^ then
                    { raises Objective-C exception... }
                    objc_enumerationMutation(expressiontemp);
                  hloopvar:=state.itemsPtr[innerloopcounter];
                  { if continue in loopbody -> jumps to start, increases count and checks }
                  { if break in loopbody: goes to outer repeat/until and innerloopcount
                    will be < currentamount -> stops }
                  <hloopbody>
                until false;
              { if the inner loop terminated early, "break" was used and we have
                to stop }
              { "If the loop is terminated early, the iterating variable is left
                pointing to the last iteration item." }
              until innerloopcounter<currentamount;
            end;
         *)

         result:=internalstatements(mainstatement);
         { the fast enumeration state }
         state:=ctempcreatenode.create(objc_fastenumerationstate,objc_fastenumerationstate.size,tt_persistent,false);
         typecheckpass(tnode(state));
         addstatement(mainstatement,state);
         { the temporary items array }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         itemsarraydef:=carraydef.create(1,16,u32inttype);
=======
         itemsarraydef:=tarraydef.create(1,16,u32inttype);
>>>>>>> graemeg/cpstrnew
=======
         itemsarraydef:=tarraydef.create(1,16,u32inttype);
>>>>>>> graemeg/cpstrnew
=======
         itemsarraydef:=tarraydef.create(1,16,u32inttype);
>>>>>>> graemeg/cpstrnew
=======
         itemsarraydef:=tarraydef.create(1,16,u32inttype);
>>>>>>> origin/cpstrnew
         itemsarraydef.elementdef:=objc_idtype;
         items:=ctempcreatenode.create(itemsarraydef,itemsarraydef.size,tt_persistent,false);
         addstatement(mainstatement,items);
         typecheckpass(tnode(items));
         { temp for the expression/collection through which we iterate }
         expressiontemp:=ctempcreatenode.create(objc_fastenumeration,objc_fastenumeration.size,tt_persistent,true);
         addstatement(mainstatement,expressiontemp);
         { currentamount temp (not really clean: we use ptruint instead of
           culong) }
         currentamount:=ctempcreatenode.create(ptruinttype,ptruinttype.size,tt_persistent,true);
         typecheckpass(tnode(currentamount));
         addstatement(mainstatement,currentamount);
         { mutationcheck temp (idem) }
         mutationcheck:=ctempcreatenode.create(ptruinttype,ptruinttype.size,tt_persistent,true);
         typecheckpass(tnode(mutationcheck));
         addstatement(mainstatement,mutationcheck);
         { innerloopcounter temp (idem) }
         innerloopcounter:=ctempcreatenode.create(ptruinttype,ptruinttype.size,tt_persistent,true);
         typecheckpass(tnode(innerloopcounter));
         addstatement(mainstatement,innerloopcounter);
         { initialise the state with 0 }
         addstatement(mainstatement,ccallnode.createinternfromunit('SYSTEM','FILLCHAR',
           ccallparanode.create(genintconstnode(0),
             ccallparanode.create(genintconstnode(objc_fastenumerationstate.size),
               ccallparanode.create(ctemprefnode.create(state),nil)
             )
           )
         ));
         { this will also check whether the expression (potentially) conforms
           to the NSFastEnumeration protocol (use expr.getcopy, because the
           caller will free expr) }
         addstatement(mainstatement,cassignmentnode.create(ctemprefnode.create(expressiontemp),expr.getcopy));

         { we add the "repeat..until" afterwards, now just create the body }
         outerloop:=internalstatements(outerloopbodystatement);
         { the countByEnumeratingWithState_objects_count call }
         hp:=ccallparanode.create(cinlinenode.create(in_length_x,false,ctypenode.create(itemsarraydef)),
               ccallparanode.create(caddrnode.create(ctemprefnode.create(items)),
                 ccallparanode.create(caddrnode.create(ctemprefnode.create(state)),nil)
               )
             );
         sym:=search_struct_member(objc_fastenumeration,'COUNTBYENUMERATINGWITHSTATE_OBJECTS_COUNT');
         if not assigned(sym) or
            (sym.typ<>procsym) then
           internalerror(2010061901);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         hp:=ccallnode.create(hp,tprocsym(sym),sym.owner,ctemprefnode.create(expressiontemp),[],nil);
=======
         hp:=ccallnode.create(hp,tprocsym(sym),sym.owner,ctemprefnode.create(expressiontemp),[]);
>>>>>>> graemeg/cpstrnew
=======
         hp:=ccallnode.create(hp,tprocsym(sym),sym.owner,ctemprefnode.create(expressiontemp),[]);
>>>>>>> graemeg/cpstrnew
=======
         hp:=ccallnode.create(hp,tprocsym(sym),sym.owner,ctemprefnode.create(expressiontemp),[]);
>>>>>>> graemeg/cpstrnew
=======
         hp:=ccallnode.create(hp,tprocsym(sym),sym.owner,ctemprefnode.create(expressiontemp),[]);
>>>>>>> origin/cpstrnew
         addstatement(outerloopbodystatement,cassignmentnode.create(
           ctemprefnode.create(currentamount),hp));
         { if currentamount = 0, bail out (use copy of hloopvar, because we
           have to use it again below) }
         hp:=internalstatements(tempstatement);
         addstatement(tempstatement,cassignmentnode.create(
             hloopvar.getcopy,cnilnode.create));
         addstatement(tempstatement,cbreaknode.create);
         addstatement(outerloopbodystatement,cifnode.create(
           caddnode.create(equaln,ctemprefnode.create(currentamount),genintconstnode(0)),
           hp,nil));
        { initial value of mutationcheck }
        hp:=ctemprefnode.create(state);
        typecheckpass(hp);
        hp:=cderefnode.create(genloadfield(hp,'MUTATIONSPTR'));
        addstatement(outerloopbodystatement,cassignmentnode.create(
          ctemprefnode.create(mutationcheck),hp));
        { initialise innerloopcounter }
        addstatement(outerloopbodystatement,cassignmentnode.create(
          ctemprefnode.create(innerloopcounter),cordconstnode.create(-1,ptruinttype,false)));

        { and now the inner loop, again adding the repeat/until afterwards }
        innerloop:=internalstatements(innerloopbodystatement);
        { inc(innerloopcounter) without range/overflowchecking (because
          we go from culong(-1) to 0 during the first iteration }
        hp:=cinlinenode.create(
          in_inc_x,false,ccallparanode.create(
            ctemprefnode.create(innerloopcounter),nil));
        hp.localswitches:=hp.localswitches-[cs_check_range,cs_check_overflow];
        addstatement(innerloopbodystatement,hp);
        { if innerloopcounter=currentamount then break to the outer loop }
        addstatement(innerloopbodystatement,cifnode.create(
          caddnode.create(equaln,
            ctemprefnode.create(innerloopcounter),
            ctemprefnode.create(currentamount)),
          cbreaknode.create,
          nil));
        { verify that the collection didn't change in the mean time }
        hp:=ctemprefnode.create(state);
        typecheckpass(hp);
        addstatement(innerloopbodystatement,cifnode.create(
          caddnode.create(unequaln,
            ctemprefnode.create(mutationcheck),
            cderefnode.create(genloadfield(hp,'MUTATIONSPTR'))
          ),
          ccallnode.createinternfromunit('OBJC','OBJC_ENUMERATIONMUTATION',
            ccallparanode.create(ctemprefnode.create(expressiontemp),nil)),
          nil));
        { finally: actually get the next element }
        hp:=ctemprefnode.create(state);
        typecheckpass(hp);
        hp:=genloadfield(hp,'ITEMSPTR');
        typecheckpass(hp);
        { don't simply use a vecn, because indexing a pointer won't work in
          non-FPC modes }
        if hp.resultdef.typ<>pointerdef then
          internalerror(2010061904);
        inserttypeconv(hp,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          carraydef.create_from_pointer(tpointerdef(hp.resultdef)));
=======
          tarraydef.create_from_pointer(tpointerdef(hp.resultdef).pointeddef));
>>>>>>> graemeg/cpstrnew
=======
          tarraydef.create_from_pointer(tpointerdef(hp.resultdef).pointeddef));
>>>>>>> graemeg/cpstrnew
=======
          tarraydef.create_from_pointer(tpointerdef(hp.resultdef).pointeddef));
>>>>>>> graemeg/cpstrnew
=======
          tarraydef.create_from_pointer(tpointerdef(hp.resultdef).pointeddef));
>>>>>>> origin/cpstrnew
        hp:=cvecnode.create(hp,ctemprefnode.create(innerloopcounter));
        addstatement(innerloopbodystatement,
          cassignmentnode.create(hloopvar,hp));
        { the actual loop body! }
        addstatement(innerloopbodystatement,hloopbody);

        { create the inner repeat/until and add it to the body of the outer
          one }
        hp:=cwhilerepeatnode.create(
          { repeat .. until false }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          cordconstnode.create(0,pasbool8type,false),innerloop,false,true);
=======
          cordconstnode.create(0,booltype,false),innerloop,false,true);
>>>>>>> graemeg/cpstrnew
=======
          cordconstnode.create(0,booltype,false),innerloop,false,true);
>>>>>>> graemeg/cpstrnew
=======
          cordconstnode.create(0,booltype,false),innerloop,false,true);
>>>>>>> graemeg/cpstrnew
=======
          cordconstnode.create(0,booltype,false),innerloop,false,true);
>>>>>>> origin/cpstrnew
        addstatement(outerloopbodystatement,hp);

        { create the outer repeat/until and add it to the the main body }
        hp:=cwhilerepeatnode.create(
          { repeat .. until innerloopcounter<currentamount }
          caddnode.create(ltn,
            ctemprefnode.create(innerloopcounter),
            ctemprefnode.create(currentamount)),
          outerloop,false,true);
        addstatement(mainstatement,hp);

        { release the temps }
        addstatement(mainstatement,ctempdeletenode.create(state));
        addstatement(mainstatement,ctempdeletenode.create(mutationcheck));
        addstatement(mainstatement,ctempdeletenode.create(currentamount));
        addstatement(mainstatement,ctempdeletenode.create(innerloopcounter));
        addstatement(mainstatement,ctempdeletenode.create(items));
        addstatement(mainstatement,ctempdeletenode.create(expressiontemp));
      end;


    function create_string_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
      var
        loopstatement, loopbodystatement: tstatementnode;
        loopvar, stringvar: ttempcreatenode;
        stringindex, loopbody, forloopnode: tnode;
      begin
        { result is a block of statements }
        result:=internalstatements(loopstatement);

        { create a temp variable for expression }
        stringvar := ctempcreatenode.create(
          expr.resultdef,
          expr.resultdef.size,
          tt_persistent,true);

        addstatement(loopstatement,stringvar);
        addstatement(loopstatement,cassignmentnode.create(ctemprefnode.create(stringvar),expr.getcopy));

        { create a loop counter: signed integer with size of string length }
        loopvar := ctempcreatenode.create(
          sinttype,
          sinttype.size,
          tt_persistent,true);

        addstatement(loopstatement,loopvar);

        stringindex:=ctemprefnode.create(loopvar);

        loopbody:=internalstatements(loopbodystatement);
        // for-in loop variable := string_expression[index]
        addstatement(loopbodystatement,
          cassignmentnode.create(hloopvar, cvecnode.create(ctemprefnode.create(stringvar),stringindex)));

        { add the actual statement to the loop }
        addstatement(loopbodystatement,hloopbody);

        forloopnode:=cfornode.create(ctemprefnode.create(loopvar),
          genintconstnode(1),
          cinlinenode.create(in_length_x,false,ctemprefnode.create(stringvar)),
          loopbody,
          false);

        addstatement(loopstatement,forloopnode);
        { free the loop counter }
        addstatement(loopstatement,ctempdeletenode.create(loopvar));
        { free the temp variable for expression }
        addstatement(loopstatement,ctempdeletenode.create(stringvar));
      end;


    function create_array_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
      var
        loopstatement, loopbodystatement: tstatementnode;
        loopvar, arrayvar: ttempcreatenode;
        arrayindex, lowbound, highbound, loopbody, forloopnode, expression: tnode;
        is_string: boolean;
        tmpdef, convertdef: tdef;
        elementcount: aword;
      begin
        expression := expr;

        { result is a block of statements }
        result:=internalstatements(loopstatement);

        is_string:=ado_IsConstString in tarraydef(expr.resultdef).arrayoptions;

        // if array element type <> loovar type then create a conversion if possible
        if compare_defs(tarraydef(expression.resultdef).elementdef,hloopvar.resultdef,nothingn)=te_incompatible then
          begin
            tmpdef:=expression.resultdef;
            elementcount:=1;
            while assigned(tmpdef) and (tmpdef.typ=arraydef) and
                  (tarraydef(tmpdef).arrayoptions = []) and
                  (compare_defs(tarraydef(tmpdef).elementdef,hloopvar.resultdef,nothingn)=te_incompatible) do
              begin
                elementcount:=elementcount*tarraydef(tmpdef).elecount;
                tmpdef:=tarraydef(tmpdef).elementdef;
              end;
            if assigned(tmpdef) and (tmpdef.typ=arraydef) and (tarraydef(tmpdef).arrayoptions = []) then
              begin
                elementcount:=elementcount*tarraydef(tmpdef).elecount;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                convertdef:=carraydef.create(0,elementcount-1,s32inttype);
=======
                convertdef:=tarraydef.create(0,elementcount-1,s32inttype);
>>>>>>> graemeg/cpstrnew
=======
                convertdef:=tarraydef.create(0,elementcount-1,s32inttype);
>>>>>>> graemeg/cpstrnew
=======
                convertdef:=tarraydef.create(0,elementcount-1,s32inttype);
>>>>>>> graemeg/cpstrnew
=======
                convertdef:=tarraydef.create(0,elementcount-1,s32inttype);
>>>>>>> origin/cpstrnew
                tarraydef(convertdef).elementdef:=tarraydef(tmpdef).elementdef;
                expression:=expr.getcopy;
                expression:=ctypeconvnode.create_internal(expression,convertdef);
                typecheckpass(expression);
                addstatement(loopstatement,expression);
              end;
          end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if (node_complexity(expression) > 1) and
          not(is_open_array(expression.resultdef)) and not(is_array_of_const(expression.resultdef)) then
=======
        if (node_complexity(expression) > 1) and not is_open_array(expression.resultdef) then
>>>>>>> graemeg/cpstrnew
=======
        if (node_complexity(expression) > 1) and not is_open_array(expression.resultdef) then
>>>>>>> graemeg/cpstrnew
=======
        if (node_complexity(expression) > 1) and not is_open_array(expression.resultdef) then
>>>>>>> graemeg/cpstrnew
=======
        if (node_complexity(expression) > 1) and not is_open_array(expression.resultdef) then
>>>>>>> origin/cpstrnew
          begin
            { create a temp variable for expression }
            arrayvar := ctempcreatenode.create(
              expression.resultdef,
              expression.resultdef.size,
              tt_persistent,true);

            if is_string then
              begin
                lowbound:=genintconstnode(1);
                highbound:=cinlinenode.create(in_length_x,false,ctemprefnode.create(arrayvar))
              end
            else
              begin
                lowbound:=cinlinenode.create(in_low_x,false,ctemprefnode.create(arrayvar));
                highbound:=cinlinenode.create(in_high_x,false,ctemprefnode.create(arrayvar));
              end;

            addstatement(loopstatement,arrayvar);
            addstatement(loopstatement,cassignmentnode.create(ctemprefnode.create(arrayvar),expression.getcopy));
          end
        else
          begin
            arrayvar:=nil;
            if is_string then
              begin
                lowbound:=genintconstnode(1);
                highbound:=cinlinenode.create(in_length_x,false,expression.getcopy);
              end
            else
              begin
                lowbound:=cinlinenode.create(in_low_x,false,expression.getcopy);
                highbound:=cinlinenode.create(in_high_x,false,expression.getcopy);
              end;
          end;

        { create a loop counter }
        loopvar := ctempcreatenode.create(
          tarraydef(expression.resultdef).rangedef,
          tarraydef(expression.resultdef).rangedef.size,
          tt_persistent,true);

        addstatement(loopstatement,loopvar);

        arrayindex:=ctemprefnode.create(loopvar);

        loopbody:=internalstatements(loopbodystatement);
        // for-in loop variable := array_expression[index]
        if assigned(arrayvar) then
          addstatement(loopbodystatement,
            cassignmentnode.create(hloopvar,cvecnode.create(ctemprefnode.create(arrayvar),arrayindex)))
        else
          addstatement(loopbodystatement,
            cassignmentnode.create(hloopvar,cvecnode.create(expression.getcopy,arrayindex)));

        { add the actual statement to the loop }
        addstatement(loopbodystatement,hloopbody);

        forloopnode:=cfornode.create(ctemprefnode.create(loopvar),
          lowbound,
          highbound,
          loopbody,
          false);

        addstatement(loopstatement,forloopnode);
        { free the loop counter }
        addstatement(loopstatement,ctempdeletenode.create(loopvar));
        { free the temp variable for expression if needed }
        if arrayvar<>nil then
          addstatement(loopstatement,ctempdeletenode.create(arrayvar));
      end;


    function create_set_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
      var
        loopstatement, loopbodystatement: tstatementnode;
        loopvar, setvar: ttempcreatenode;
        loopbody, forloopnode: tnode;
      begin
        // first check is set is empty and if it so then skip other processing
        if not Assigned(tsetdef(expr.resultdef).elementdef) then
          begin
            result:=cnothingnode.create;
            // free unused nodes
            hloopvar.free;
            hloopbody.free;
            exit;
          end;
        { result is a block of statements }
        result:=internalstatements(loopstatement);

        { create a temp variable for expression }
        setvar := ctempcreatenode.create(
          expr.resultdef,
          expr.resultdef.size,
          tt_persistent,true);

        addstatement(loopstatement,setvar);
        addstatement(loopstatement,cassignmentnode.create(ctemprefnode.create(setvar),expr.getcopy));

        { create a loop counter }
        loopvar := ctempcreatenode.create(
          tsetdef(expr.resultdef).elementdef,
          tsetdef(expr.resultdef).elementdef.size,
          tt_persistent,true);

        addstatement(loopstatement,loopvar);

        // if loopvar in set then
        // begin
        //   hloopvar := loopvar
        //   for-in loop body
        // end

        loopbody:=cifnode.create(
          cinnode.create(ctemprefnode.create(loopvar),ctemprefnode.create(setvar)),
          internalstatements(loopbodystatement),
          nil);

        addstatement(loopbodystatement,cassignmentnode.create(hloopvar,ctemprefnode.create(loopvar)));
        { add the actual statement to the loop }
        addstatement(loopbodystatement,hloopbody);

        forloopnode:=cfornode.create(ctemprefnode.create(loopvar),
          cinlinenode.create(in_low_x,false,ctemprefnode.create(setvar)),
          cinlinenode.create(in_high_x,false,ctemprefnode.create(setvar)),
          loopbody,
          false);

        addstatement(loopstatement,forloopnode);
        { free the loop counter }
        addstatement(loopstatement,ctempdeletenode.create(loopvar));
        { free the temp variable for expression }
        addstatement(loopstatement,ctempdeletenode.create(setvar));
      end;


    function create_enumerator_for_in_loop(hloopvar, hloopbody, expr: tnode;
       enumerator_get, enumerator_move: tprocdef; enumerator_current: tpropertysym): tnode;
      var
        loopstatement, loopbodystatement: tstatementnode;
        enumvar: ttempcreatenode;
        loopbody, whileloopnode,
        enum_get, enum_move, enum_current, enum_get_params: tnode;
        propaccesslist: tpropaccesslist;
        enumerator_is_class: boolean;
        enumerator_destructor: tprocdef;
      begin
        { result is a block of statements }
        result:=internalstatements(loopstatement);

        enumerator_is_class := is_class(enumerator_get.returndef);

        { create a temp variable for enumerator }
        enumvar := ctempcreatenode.create(
          enumerator_get.returndef,
          enumerator_get.returndef.size,
          tt_persistent,true);

        addstatement(loopstatement,enumvar);

        if enumerator_get.proctypeoption=potype_operator then
          begin
            enum_get_params:=ccallparanode.create(expr.getcopy,nil);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            enum_get:=ccallnode.create(enum_get_params, tprocsym(enumerator_get.procsym), nil, nil, [],nil);
=======
            enum_get:=ccallnode.create(enum_get_params, tprocsym(enumerator_get.procsym), nil, nil, []);
>>>>>>> graemeg/cpstrnew
=======
            enum_get:=ccallnode.create(enum_get_params, tprocsym(enumerator_get.procsym), nil, nil, []);
>>>>>>> graemeg/cpstrnew
=======
            enum_get:=ccallnode.create(enum_get_params, tprocsym(enumerator_get.procsym), nil, nil, []);
>>>>>>> graemeg/cpstrnew
=======
            enum_get:=ccallnode.create(enum_get_params, tprocsym(enumerator_get.procsym), nil, nil, []);
>>>>>>> origin/cpstrnew
            tcallnode(enum_get).procdefinition:=enumerator_get;
            addsymref(enumerator_get.procsym);
          end
        else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          enum_get:=ccallnode.create(nil, tprocsym(enumerator_get.procsym), enumerator_get.owner, expr.getcopy, [],nil);
=======
          enum_get:=ccallnode.create(nil, tprocsym(enumerator_get.procsym), enumerator_get.owner, expr.getcopy, []);
>>>>>>> graemeg/cpstrnew
=======
          enum_get:=ccallnode.create(nil, tprocsym(enumerator_get.procsym), enumerator_get.owner, expr.getcopy, []);
>>>>>>> graemeg/cpstrnew
=======
          enum_get:=ccallnode.create(nil, tprocsym(enumerator_get.procsym), enumerator_get.owner, expr.getcopy, []);
>>>>>>> graemeg/cpstrnew
=======
          enum_get:=ccallnode.create(nil, tprocsym(enumerator_get.procsym), enumerator_get.owner, expr.getcopy, []);
>>>>>>> origin/cpstrnew

        addstatement(loopstatement,
          cassignmentnode.create(
            ctemprefnode.create(enumvar),
            enum_get
          ));

        loopbody:=internalstatements(loopbodystatement);
        { for-in loop variable := enumerator.current }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if enumerator_current.getpropaccesslist(palt_read,propaccesslist) then
=======
        if getpropaccesslist(enumerator_current,palt_read,propaccesslist) then
>>>>>>> graemeg/cpstrnew
=======
        if getpropaccesslist(enumerator_current,palt_read,propaccesslist) then
>>>>>>> graemeg/cpstrnew
=======
        if getpropaccesslist(enumerator_current,palt_read,propaccesslist) then
>>>>>>> graemeg/cpstrnew
=======
        if getpropaccesslist(enumerator_current,palt_read,propaccesslist) then
>>>>>>> origin/cpstrnew
          begin
             case propaccesslist.firstsym^.sym.typ of
               fieldvarsym :
                 begin
                   { generate access code }
                   enum_current:=ctemprefnode.create(enumvar);
                   propaccesslist_to_node(enum_current,enumerator_current.owner,propaccesslist);
                   include(enum_current.flags,nf_isproperty);
                 end;
               procsym :
                 begin
                   { generate the method call }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   enum_current:=ccallnode.create(nil,tprocsym(propaccesslist.firstsym^.sym),enumerator_current.owner,ctemprefnode.create(enumvar),[],nil);
=======
                   enum_current:=ccallnode.create(nil,tprocsym(propaccesslist.firstsym^.sym),enumerator_current.owner,ctemprefnode.create(enumvar),[]);
>>>>>>> graemeg/cpstrnew
=======
                   enum_current:=ccallnode.create(nil,tprocsym(propaccesslist.firstsym^.sym),enumerator_current.owner,ctemprefnode.create(enumvar),[]);
>>>>>>> graemeg/cpstrnew
=======
                   enum_current:=ccallnode.create(nil,tprocsym(propaccesslist.firstsym^.sym),enumerator_current.owner,ctemprefnode.create(enumvar),[]);
>>>>>>> graemeg/cpstrnew
=======
                   enum_current:=ccallnode.create(nil,tprocsym(propaccesslist.firstsym^.sym),enumerator_current.owner,ctemprefnode.create(enumvar),[]);
>>>>>>> origin/cpstrnew
                   include(enum_current.flags,nf_isproperty);
                 end
               else
                 begin
                   enum_current:=cerrornode.create;
                   Message(type_e_mismatch);
                 end;
            end;
          end
        else
          enum_current:=cerrornode.create;

        addstatement(loopbodystatement,
          cassignmentnode.create(hloopvar, enum_current));

        { add the actual statement to the loop }
        addstatement(loopbodystatement,hloopbody);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        enum_move:=ccallnode.create(nil, tprocsym(enumerator_move.procsym), enumerator_move.owner, ctemprefnode.create(enumvar), [],nil);
=======
        enum_move:=ccallnode.create(nil, tprocsym(enumerator_move.procsym), enumerator_move.owner, ctemprefnode.create(enumvar), []);
>>>>>>> graemeg/cpstrnew
=======
        enum_move:=ccallnode.create(nil, tprocsym(enumerator_move.procsym), enumerator_move.owner, ctemprefnode.create(enumvar), []);
>>>>>>> graemeg/cpstrnew
=======
        enum_move:=ccallnode.create(nil, tprocsym(enumerator_move.procsym), enumerator_move.owner, ctemprefnode.create(enumvar), []);
>>>>>>> graemeg/cpstrnew
=======
        enum_move:=ccallnode.create(nil, tprocsym(enumerator_move.procsym), enumerator_move.owner, ctemprefnode.create(enumvar), []);
>>>>>>> origin/cpstrnew
        whileloopnode:=cwhilerepeatnode.create(enum_move,loopbody,true,false);

        if enumerator_is_class then
          begin
            { insert a try-finally and call the destructor for the enumerator in the finally section }
            enumerator_destructor:=tobjectdef(enumerator_get.returndef).find_destructor;
            if assigned(enumerator_destructor) then
              begin
                whileloopnode:=ctryfinallynode.create(
                  whileloopnode, // try node
                  ccallnode.create(nil,tprocsym(enumerator_destructor.procsym), // finally node
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[],nil));
=======
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[]));
>>>>>>> graemeg/cpstrnew
=======
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[]));
>>>>>>> graemeg/cpstrnew
=======
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[]));
>>>>>>> graemeg/cpstrnew
=======
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[]));
>>>>>>> origin/cpstrnew
              end;
            { if getenumerator <> nil then do the loop }
            whileloopnode:=cifnode.create(
              caddnode.create(unequaln, ctemprefnode.create(enumvar), cnilnode.create),
              whileloopnode,
              nil);
          end;

        addstatement(loopstatement, whileloopnode);

        if is_object(enumerator_get.returndef) then
          begin
            // call the object destructor too
            enumerator_destructor:=tobjectdef(enumerator_get.returndef).find_destructor;
            if assigned(enumerator_destructor) then
              begin
                addstatement(loopstatement,
                  ccallnode.create(nil,tprocsym(enumerator_destructor.procsym),
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[],nil));
=======
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[]));
>>>>>>> graemeg/cpstrnew
=======
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[]));
>>>>>>> graemeg/cpstrnew
=======
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[]));
>>>>>>> graemeg/cpstrnew
=======
                    enumerator_destructor.procsym.owner,ctemprefnode.create(enumvar),[]));
>>>>>>> origin/cpstrnew
              end;
          end;

        { free the temp variable for enumerator }
        addstatement(loopstatement,ctempdeletenode.create(enumvar));
      end;


    function create_for_in_loop(hloopvar, hloopbody, expr: tnode): tnode;
      var
        pd, movenext: tprocdef;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        helperdef: tobjectdef;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        current: tpropertysym;
        storefilepos: tfileposinfo;
      begin
        storefilepos:=current_filepos;
        current_filepos:=hloopvar.fileinfo;
        if expr.nodetype=typen then
          begin
            if (expr.resultdef.typ=enumdef) and tenumdef(expr.resultdef).has_jumps then
              begin
                result:=cerrornode.create;
                hloopvar.free;
                hloopbody.free;
                MessagePos1(expr.fileinfo,parser_e_for_in_loop_cannot_be_used_for_the_type,expr.resultdef.typename);
              end
            else
              result:=create_type_for_in_loop(hloopvar, hloopbody, expr);
          end
        else
          begin
            { loop is made for an expression }
            // Objective-C uses different conventions (and it's only supported for Objective-C 2.0)
            if is_objc_class_or_protocol(hloopvar.resultdef) or
               is_objc_class_or_protocol(expr.resultdef) then
              begin
                result:=create_objc_for_in_loop(hloopvar,hloopbody,expr);
                if result.nodetype=errorn then
                  begin
                    hloopvar.free;
                    hloopbody.free;
                  end;
              end
            else
              begin
                // search for operator first
                pd:=search_enumerator_operator(expr.resultdef, hloopvar.resultdef);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                // if there is no operator then search for class/object enumerator method
                if (pd=nil) and (expr.resultdef.typ in [objectdef,recorddef]) then
                  begin
                    { first search using the helper hierarchy }
                    if search_last_objectpascal_helper(tabstractrecorddef(expr.resultdef),nil,helperdef) then
                      repeat
                        pd:=helperdef.search_enumerator_get;
                        helperdef:=helperdef.childof;
                      until (pd<>nil) or (helperdef=nil);
                    { we didn't find an enumerator in a helper, so search in the
                      class/record/object itself }
                    if pd=nil then
                      pd:=tabstractrecorddef(expr.resultdef).search_enumerator_get;
                  end;
=======
                // if there is no operator then search for class/object/record enumerator method
                if (pd=nil) and (expr.resultdef.typ in [objectdef,recorddef]) then
                  pd:=tabstractrecorddef(expr.resultdef).search_enumerator_get;
>>>>>>> graemeg/cpstrnew
=======
                // if there is no operator then search for class/object/record enumerator method
                if (pd=nil) and (expr.resultdef.typ in [objectdef,recorddef]) then
                  pd:=tabstractrecorddef(expr.resultdef).search_enumerator_get;
>>>>>>> graemeg/cpstrnew
=======
                // if there is no operator then search for class/object/record enumerator method
                if (pd=nil) and (expr.resultdef.typ in [objectdef,recorddef]) then
                  pd:=tabstractrecorddef(expr.resultdef).search_enumerator_get;
>>>>>>> graemeg/cpstrnew
=======
                // if there is no operator then search for class/object/record enumerator method
                if (pd=nil) and (expr.resultdef.typ in [objectdef,recorddef]) then
                  pd:=tabstractrecorddef(expr.resultdef).search_enumerator_get;
>>>>>>> origin/cpstrnew
                if pd<>nil then
                  begin
                    // seach movenext and current symbols
                    movenext:=tabstractrecorddef(pd.returndef).search_enumerator_move;
                    if movenext = nil then
                      begin
                        result:=cerrornode.create;
                        hloopvar.free;
                        hloopbody.free;
                        MessagePos1(expr.fileinfo,sym_e_no_enumerator_move,pd.returndef.typename);
                      end
                    else
                      begin
                        current:=tpropertysym(tabstractrecorddef(pd.returndef).search_enumerator_current);
                        if current = nil then
                          begin
                            result:=cerrornode.create;
                            hloopvar.free;
                            hloopbody.free;
                            MessagePos1(expr.fileinfo,sym_e_no_enumerator_current,pd.returndef.typename);
                          end
                        else
                          result:=create_enumerator_for_in_loop(hloopvar, hloopbody, expr, pd, movenext, current);
                      end;
                  end
                else
                  begin
                    case expr.resultdef.typ of
                      stringdef: result:=create_string_for_in_loop(hloopvar, hloopbody, expr);
                      arraydef: result:=create_array_for_in_loop(hloopvar, hloopbody, expr);
                      setdef: result:=create_set_for_in_loop(hloopvar, hloopbody, expr);
                    else
                      begin
                        result:=cerrornode.create;
                        hloopvar.free;
                        hloopbody.free;
                        MessagePos1(expr.fileinfo,sym_e_no_enumerator,expr.resultdef.typename);
                      end;
                    end;
                  end;
              end;
          end;
        current_filepos:=storefilepos;
      end;

{****************************************************************************
                                 TLOOPNODE
*****************************************************************************}

    constructor tloopnode.create(tt : tnodetype;l,r,_t1,_t2 : tnode);

      begin
         inherited create(tt,l,r);
         t1:=_t1;
         t2:=_t2;
         fileinfo:=l.fileinfo;
      end;

    destructor tloopnode.destroy;

      begin
         t1.free;
         t2.free;
         inherited destroy;
      end;


    constructor tloopnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        t1:=ppuloadnode(ppufile);
        t2:=ppuloadnode(ppufile);
        ppufile.getsmallset(loopflags);
      end;


    procedure tloopnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppuwritenode(ppufile,t1);
        ppuwritenode(ppufile,t2);
        ppufile.putsmallset(loopflags);
      end;


    procedure tloopnode.derefnode;
      begin
        inherited derefnode;
        if assigned(t1) then
          t1.derefnode;
        if assigned(t2) then
          t2.derefnode;
      end;

    procedure tloopnode.buildderefimpl;
      begin
        inherited buildderefimpl;
        if assigned(t1) then
          t1.buildderefimpl;
        if assigned(t2) then
          t2.buildderefimpl;
      end;


    procedure tloopnode.derefimpl;
      begin
        inherited derefimpl;
        if assigned(t1) then
          t1.derefimpl;
        if assigned(t2) then
          t2.derefimpl;
      end;


    function tloopnode.dogetcopy : tnode;

      var
         p : tloopnode;

      begin
         p:=tloopnode(inherited dogetcopy);
         if assigned(t1) then
           p.t1:=t1.dogetcopy
         else
           p.t1:=nil;
         if assigned(t2) then
           p.t2:=t2.dogetcopy
         else
           p.t2:=nil;
         p.loopflags:=loopflags;
         dogetcopy:=p;
      end;

    procedure tloopnode.insertintolist(l : tnodelist);

      begin
      end;


    procedure tloopnode.printnodetree(var t:text);
      begin
        write(t,printnodeindention,'(');
        printnodeindent;
        printnodeinfo(t);
        writeln(t);
        printnode(t,left);
        printnode(t,right);
        printnode(t,t1);
        printnode(t,t2);
        printnodeunindent;
        writeln(t,printnodeindention,')');
      end;


    function tloopnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (loopflags*loopflagsequal=tloopnode(p).loopflags*loopflagsequal) and
          t1.isequal(tloopnode(p).t1) and
          t2.isequal(tloopnode(p).t2);
      end;

{****************************************************************************
                               TWHILEREPEATNODE
*****************************************************************************}

    constructor Twhilerepeatnode.create(l,r:Tnode;tab,cn:boolean);
      begin
          inherited create(whilerepeatn,l,r,nil,nil);
          if tab then
              include(loopflags, lnf_testatbegin);
          if cn then
              include(loopflags,lnf_checknegate);
      end;

    function twhilerepeatnode.pass_typecheck:tnode;
      var
         t:Tunarynode;
      begin
         result:=nil;
         resultdef:=voidtype;

         typecheckpass(left);

         { tp procvar support }
         maybe_call_procvar(left,true);

         {A not node can be removed.}
         if left.nodetype=notn then
           begin
             t:=Tunarynode(left);
             left:=Tunarynode(left).left;
             t.left:=nil;
             t.destroy;
             {Symdif operator, in case you are wondering:}
             loopflags:=loopflags >< [lnf_checknegate];
           end;
         { loop instruction }
         if assigned(right) then
           typecheckpass(right);
         set_varstate(left,vs_read,[vsf_must_be_valid]);
         if codegenerror then
           exit;

         if not(is_boolean(left.resultdef)) and
           not(is_typeparam(left.resultdef)) then
             inserttypeconv(left,pasbool8type);

         { Give warnings for code that will never be executed for
           while false do }
         if (lnf_testatbegin in loopflags) and
            (left.nodetype=ordconstn) and
            (tordconstnode(left).value.uvalue=0) and
            assigned(right) then
           CGMessagePos(right.fileinfo,cg_w_unreachable_code);
      end;


{$ifdef prefetchnext}
    type
      passignmentquery = ^tassignmentquery;
      tassignmentquery = record
        towhat: tnode;
        source: tassignmentnode;
        statementcount: cardinal;
      end;

    function checkassignment(var n: tnode; arg: pointer): foreachnoderesult;
      var
        query: passignmentquery absolute arg;
        temp, prederef: tnode;
      begin
        result := fen_norecurse_false;
        if (n.nodetype in [assignn,inlinen,forn,calln,whilerepeatn,casen,ifn]) then
          inc(query^.statementcount);
        { make sure there's something else in the loop besides going to the }
        { next item                                                         }
        if (query^.statementcount > 1) and
           (n.nodetype = assignn) then
          begin
            { skip type conversions of assignment target }
            temp := tassignmentnode(n).left;
            while (temp.nodetype = typeconvn) do
              temp := ttypeconvnode(temp).left;

            { assignment to x of the while assigned(x) check? }
            if not(temp.isequal(query^.towhat)) then
              exit;

            { right hand side of assignment dereferenced field of }
            { x? (no derefn in case of class)                     }
            temp := tassignmentnode(n).right;
            while (temp.nodetype = typeconvn) do
              temp := ttypeconvnode(temp).left;
            if (temp.nodetype <> subscriptn) then
              exit;

            prederef := tsubscriptnode(temp).left;
            temp := prederef;
            while (temp.nodetype = typeconvn) do
              temp := ttypeconvnode(temp).left;

            { see tests/test/prefetch1.pp }
            if (temp.nodetype = derefn) then
              temp := tderefnode(temp).left
            else
              temp := prederef;

            if temp.isequal(query^.towhat) then
              begin
                query^.source := tassignmentnode(n);
                result := fen_norecurse_true;
               end
          end
        { don't check nodes which can't contain an assignment or whose }
        { final assignment can vary a lot                              }
        else if not(n.nodetype in [calln,inlinen,casen,whilerepeatn,forn]) then
          result := fen_false;
      end;


    function findassignment(where: tnode; towhat: tnode): tassignmentnode;
      var
        query: tassignmentquery;
      begin
        query.towhat := towhat;
        query.source := nil;
        query.statementcount := 0;
        if foreachnodestatic(where,@checkassignment,@query) then
          result := query.source
        else
           result := nil;
      end;
{$endif prefetchnext}


    function twhilerepeatnode.pass_1 : tnode;
{$ifdef prefetchnext}
      var
         runnernode, prefetchcode: tnode;
         assignmentnode: tassignmentnode;
         prefetchstatements: tstatementnode;
{$endif prefetchnext}
      begin
         result:=nil;
         expectloc:=LOC_VOID;

         firstpass(left);
         if codegenerror then
           exit;

         { loop instruction }
         if assigned(right) then
           begin
              firstpass(right);
              if codegenerror then
                exit;
           end;

{$ifdef prefetchnext}
         { do at the end so all complex typeconversions are already }
         { converted to calln's                                     }
         if (cs_opt_level1 in current_settings.optimizerswitches) and
            (lnf_testatbegin in loopflags) then
           begin
             { get first component of the while check }
             runnernode := left;
             while (runnernode.nodetype in [andn,orn,notn,xorn,typeconvn]) do
               runnernode := tunarynode(runnernode).left;
             { is it an assigned(x) check? }
             if ((runnernode.nodetype = inlinen) and
                 (tinlinenode(runnernode).inlinenumber = in_assigned_x)) or
                ((runnernode.nodetype = unequaln) and
                 (taddnode(runnernode).right.nodetype = niln)) then
               begin
                 runnernode := tunarynode(runnernode).left;
                 { in case of in_assigned_x, there's a callparan in between }
                 if (runnernode.nodetype = callparan) then
                   runnernode := tcallparanode(runnernode).left;
                 while (runnernode.nodetype = typeconvn) do
                   runnernode := ttypeconvnode(runnernode).left;
                 { is there an "x := x(^).somefield"? }
                 assignmentnode := findassignment(right,runnernode);
                 if assigned(assignmentnode) then
                   begin
                     prefetchcode := internalstatements(prefetchstatements);
                     addstatement(prefetchstatements,geninlinenode(in_prefetch_var,false,
                       cderefnode.create(ctypeconvnode.create(assignmentnode.right.getcopy,voidpointertype))));
                     addstatement(prefetchstatements,right);
                     right := prefetchcode;
                     typecheckpass(right);
                   end;
               end;
           end;
{$endif prefetchnext}
      end;

{$ifdef state_tracking}
    function Twhilerepeatnode.track_state_pass(exec_known:boolean):boolean;

    var condition:Tnode;
        code:Tnode;
        done:boolean;
        value:boolean;
        change:boolean;
        firsttest:boolean;
        factval:Tnode;

    begin
        track_state_pass:=false;
        done:=false;
        firsttest:=true;
        {For repeat until statements, first do a pass through the code.}
        if not(lnf_testatbegin in flags) then
            begin
                code:=right.getcopy;
                if code.track_state_pass(exec_known) then
                    track_state_pass:=true;
                code.destroy;
            end;
        repeat
            condition:=left.getcopy;
            code:=right.getcopy;
            change:=condition.track_state_pass(exec_known);
            factval:=aktstate.find_fact(left);
            if factval<>nil then
                begin
                    condition.destroy;
                    condition:=factval.getcopy;
                    change:=true;
                end;
            if change then
                begin
                    track_state_pass:=true;
                    {Force new resultdef pass.}
                    condition.resultdef:=nil;
                    do_typecheckpass(condition);
                end;
            if is_constboolnode(condition) then
                begin
                    {Try to turn a while loop into a repeat loop.}
                    if firsttest then
                        exclude(flags,testatbegin);
                    value:=(Tordconstnode(condition).value<>0) xor checknegate;
                    if value then
                        begin
                            if code.track_state_pass(exec_known) then
                                track_state_pass:=true;
                        end
                    else
                        done:=true;
                end
            else
                begin
                    {Remove any modified variables from the state.}
                    code.track_state_pass(false);
                    done:=true;
                end;
            code.destroy;
            condition.destroy;
            firsttest:=false;
        until done;
        {The loop condition is also known, for example:
         while i<10 do
            begin
                ...
            end;

         When the loop is done, we do know that i<10 = false.
        }
        condition:=left.getcopy;
        if condition.track_state_pass(exec_known) then
            begin
                track_state_pass:=true;
                {Force new resultdef pass.}
                condition.resultdef:=nil;
                do_typecheckpass(condition);
            end;
        if not is_constboolnode(condition) then
            aktstate.store_fact(condition,
             cordconstnode.create(byte(checknegate),pasbool8type,true))
        else
            condition.destroy;
    end;
{$endif}

{*****************************************************************************
                               TIFNODE
*****************************************************************************}

    constructor tifnode.create(l,r,_t1 : tnode);
      begin
         inherited create(ifn,l,r,_t1,nil);
      end;


    constructor tifnode.create_internal(l,r,_t1 : tnode);
      begin
        create(l,r,_t1);
        include(flags,nf_internal);
      end;


    function tifnode.internalsimplify(warn: boolean) : tnode;
      begin
        result:=nil;
        { optimize constant expressions }
        if (left.nodetype=ordconstn) then
          begin
             if tordconstnode(left).value.uvalue<>0 then
               begin
                  if assigned(right) then
                    result:=right
                  else
                    result:=cnothingnode.create;
                  right:=nil;
                  if warn and assigned(t1) then
                    CGMessagePos(t1.fileinfo,cg_w_unreachable_code);
               end
             else
               begin
                  if assigned(t1) then
                    result:=t1
                  else
                    result:=cnothingnode.create;
                  t1:=nil;
                  if warn and assigned(right) then
                    CGMessagePos(right.fileinfo,cg_w_unreachable_code);
               end;
          end;
      end;


    function tifnode.simplify(forinline : boolean) : tnode;
      begin
        result:=internalsimplify(false);
      end;


    function tifnode.pass_typecheck:tnode;
      begin
         result:=nil;
         resultdef:=voidtype;

         typecheckpass(left);

         { tp procvar support }
         maybe_call_procvar(left,true);

         { if path }
         if assigned(right) then
           typecheckpass(right);
         { else path }
         if assigned(t1) then
           typecheckpass(t1);
         set_varstate(left,vs_read,[vsf_must_be_valid]);
         if codegenerror then
           exit;

         if not(is_boolean(left.resultdef)) and
           not(is_typeparam(left.resultdef)) then
             inserttypeconv(left,pasbool8type);

         result:=internalsimplify(not(nf_internal in flags));
      end;


    function tifnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_VOID;
         firstpass(left);

         { if path }
         if assigned(right) then
           firstpass(right);

         { else path }
         if assigned(t1) then
           firstpass(t1);

         { leave if we've got an error in one of the paths }

         if codegenerror then
           exit;
      end;


{*****************************************************************************
                              TFORNODE
*****************************************************************************}

    constructor tfornode.create(l,r,_t1,_t2 : tnode;back : boolean);

      begin
         inherited create(forn,l,r,_t1,_t2);
         if back then
           include(loopflags,lnf_backward);
         include(loopflags,lnf_testatbegin);
      end;

<<<<<<< HEAD
=======
    procedure Tfornode.loop_var_access(not_type:Tnotification_flag;
                                       symbol:Tsym);

    begin
      {If there is a read access, the value of the loop counter is important;
       at the end of the loop the loop variable should contain the value it
       had in the last iteration.}
      if not_type=vn_onwrite then
        begin
          writeln('Loopvar does not matter on exit');
        end
      else
        begin
          exclude(loopflags,lnf_dont_mind_loopvar_on_exit);
          writeln('Loopvar does matter on exit');
        end;
      Tabstractvarsym(symbol).unregister_notification(loopvar_notid);
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
    function tfornode.simplify(forinline : boolean) : tnode;
      begin
        result:=nil;
         { Can we spare the first comparision? }
         if (t1.nodetype=ordconstn) and
            (right.nodetype=ordconstn) and
            (
             (
              (lnf_backward in loopflags) and
              (Tordconstnode(right).value>=Tordconstnode(t1).value)
             ) or
             (
               not(lnf_backward in loopflags) and
               (Tordconstnode(right).value<=Tordconstnode(t1).value)
             )
            ) then
           exclude(loopflags,lnf_testatbegin);

        if (t1.nodetype=ordconstn) and
           (right.nodetype=ordconstn) and
           (
            (
             (lnf_backward in loopflags) and
             (tordconstnode(right).value<tordconstnode(t1).value)
            ) or
            (
              not(lnf_backward in loopflags) and
              (tordconstnode(right).value>tordconstnode(t1).value)
            )
           ) then
        result:=cnothingnode.create;
      end;


    function tfornode.wrap_to_value:tnode;
      var
        statements: tstatementnode;
        temp: ttempcreatenode;
      begin
        result:=internalstatements(statements);
        temp:=ctempcreatenode.create(t1.resultdef,t1.resultdef.size,tt_persistent,true);
        addstatement(statements,temp);
        addstatement(statements,cassignmentnode.create(
          ctemprefnode.create(temp),
          t1));
        { create a new for node, it is cheaper than cloning entire loop body }
        addstatement(statements,cfornode.create(
          left,right,ctemprefnode.create(temp),t2,lnf_backward in loopflags));
        addstatement(statements,ctempdeletenode.create(temp));
        { all child nodes are reused }
        left:=nil;
        right:=nil;
        t1:=nil;
        t2:=nil;
      end;


    function tfornode.pass_typecheck:tnode;
      var
        res : tnode;
      begin
         result:=nil;
         resultdef:=voidtype;

         { process the loopvar, from and to, varstates are already set }
         typecheckpass(left);
         typecheckpass(right);
         typecheckpass(t1);

         set_varstate(left,vs_written,[]);

         { loop unrolling }
         if (cs_opt_loopunroll in current_settings.optimizerswitches) and
           { statements must be error free }
           not(nf_error in t2.flags) then
           begin
             typecheckpass(t2);
             res:=t2.simplify(false);
             if assigned(res) then
               t2:=res;
             res:=unroll_loop(self);
             if assigned(res) then
               begin
                 typecheckpass(res);
                 result:=res;
                 exit;
               end;
           end;

         { Make sure that the loop var and the
           from and to values are compatible types }
         check_ranges(right.fileinfo,right,left.resultdef);
         inserttypeconv(right,left.resultdef);

         check_ranges(t1.fileinfo,t1,left.resultdef);
         inserttypeconv(t1,left.resultdef);

         if assigned(t2) then
           typecheckpass(t2);
      end;


    function tfornode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_VOID;

         firstpass(left);
         firstpass(right);
         firstpass(t1);

         if assigned(t2) then
           firstpass(t2);
         if codegenerror then
           exit;

         { 'to' value must be evaluated once before loop, so its possible modifications
           inside loop body do not affect the number of iterations (see webtbs/tw8883). }
         if not (t1.nodetype in [ordconstn,temprefn]) then
           result:=wrap_to_value;
      end;


{*****************************************************************************
                             TEXITNODE
*****************************************************************************}

    constructor texitnode.create(l:tnode);
      begin
        inherited create(exitn,l);
        if assigned(left) then
          begin
            { add assignment to funcretsym }
            left:=ctypeconvnode.create(left,current_procinfo.procdef.returndef);
            left:=cassignmentnode.create(
              cloadnode.create(current_procinfo.procdef.funcretsym,current_procinfo.procdef.funcretsym.owner),
              left);
          end;
      end;


    constructor texitnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
      end;


    procedure texitnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
      end;


    function texitnode.pass_typecheck:tnode;
      var
        newstatement : tstatementnode;
      begin
        result:=nil;
        if assigned(left) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          begin
             result:=internalstatements(newstatement);
             addstatement(newstatement,left);
             left:=nil;
             addstatement(newstatement,self.getcopy);
          end;
=======
          typecheckpass(left);
>>>>>>> graemeg/cpstrnew
=======
          typecheckpass(left);
>>>>>>> graemeg/cpstrnew
=======
          typecheckpass(left);
>>>>>>> graemeg/cpstrnew
=======
          typecheckpass(left);
>>>>>>> origin/cpstrnew
        resultdef:=voidtype;
      end;


    function texitnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_VOID;
         if assigned(left) then
           internalerror(2011052801);
      end;


{*****************************************************************************
                             TBREAKNODE
*****************************************************************************}

    constructor tbreaknode.create;

      begin
        inherited create(breakn);
      end;


    function tbreaknode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=voidtype;
      end;


    function tbreaknode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_VOID;
      end;


{*****************************************************************************
                             TCONTINUENODE
*****************************************************************************}

    constructor tcontinuenode.create;
      begin
        inherited create(continuen);
      end;


    function tcontinuenode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=voidtype;
      end;


    function tcontinuenode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_VOID;
      end;


{*****************************************************************************
                             TGOTONODE
*****************************************************************************}

    constructor tgotonode.create(p : tlabelsym);
      begin
        inherited create(goton);
<<<<<<< HEAD
        exceptionblock:=current_exceptblock;
=======
        exceptionblock:=aktexceptblock;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        labelnode:=nil;
        labelsym:=p;
      end;


    constructor tgotonode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        labelnodeidx:=ppufile.getlongint;
        exceptionblock:=ppufile.getbyte;
      end;


    procedure tgotonode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        labelnodeidx:=labelnode.ppuidx;
        ppufile.putlongint(labelnodeidx);
        ppufile.putbyte(exceptionblock);
      end;


    procedure tgotonode.buildderefimpl;
      begin
        inherited buildderefimpl;
      end;


    procedure tgotonode.derefimpl;
      begin
        inherited derefimpl;
      end;


    procedure tgotonode.resolveppuidx;
      begin
        labelnode:=tlabelnode(nodeppuidxget(labelnodeidx));
        if labelnode.nodetype<>labeln then
          internalerror(200809021);
      end;


    function tgotonode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=voidtype;
      end;


    function tgotonode.pass_1 : tnode;
      var
        p2 : tprocinfo;
      begin
        result:=nil;
        expectloc:=LOC_VOID;

        { The labelnode can already be set when
          this node was copied }
<<<<<<< HEAD
<<<<<<< HEAD
        if not(assigned(labelnode)) then
          begin
            { inner procedure goto? }
            if assigned(labelsym.code) and
              ((assigned(labelsym.owner) and (current_procinfo.procdef.parast.symtablelevel=labelsym.owner.symtablelevel)) or
              { generated by the optimizer? }
               not(assigned(labelsym.owner))) then
=======
        if not assigned(labelnode) then
          begin
            if assigned(labelsym.code) then
>>>>>>> graemeg/fixes_2_2
=======
        if not assigned(labelnode) then
          begin
            if assigned(labelsym.code) then
>>>>>>> origin/fixes_2_2
              labelnode:=tlabelnode(labelsym.code)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            else if ((m_non_local_goto in current_settings.modeswitches) and
              assigned(labelsym.owner)) or
              { nested exits don't need the non local goto switch }
              (labelsym.realname='$nestedexit') then
=======
            else if (m_non_local_goto in current_settings.modeswitches) and
              assigned(labelsym.owner) then
>>>>>>> graemeg/cpstrnew
=======
            else if (m_non_local_goto in current_settings.modeswitches) and
              assigned(labelsym.owner) then
>>>>>>> graemeg/cpstrnew
=======
            else if (m_non_local_goto in current_settings.modeswitches) and
              assigned(labelsym.owner) then
>>>>>>> graemeg/cpstrnew
=======
            else if (m_non_local_goto in current_settings.modeswitches) and
              assigned(labelsym.owner) then
>>>>>>> origin/cpstrnew
              begin
                if current_procinfo.procdef.parast.symtablelevel>labelsym.owner.symtablelevel then
                  begin
                    { don't mess with the exception blocks, global gotos in/out side exception blocks are not allowed }
                    if exceptionblock>0 then
                      CGMessage(cg_e_goto_inout_of_exception_block);

                    { goto across procedures using exception?
                      this is not allowed because we cannot
                      easily unwind the exception frame
                      stack
                    }
                    p2:=current_procinfo;
                    while true do
                      begin
                        if (p2.flags*[pi_needs_implicit_finally,pi_uses_exceptions,pi_has_implicit_finally])<>[] then
                          Message(cg_e_goto_across_procedures_with_exceptions_not_allowed);
                        if labelsym.owner=p2.procdef.localst then
                          break;
                        p2:=p2.parent
                      end;

                    if assigned(labelsym.jumpbuf) then
                      begin
                        labelsym.nonlocal:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        exclude(current_procinfo.procdef.procoptions,po_inline);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                        result:=ccallnode.createintern('fpc_longjmp',
                          ccallparanode.create(cordconstnode.create(1,sinttype,true),
                          ccallparanode.create(cloadnode.create(labelsym.jumpbuf,labelsym.jumpbuf.owner),
                        nil)));
                      end
                    else
                      CGMessage1(cg_e_goto_label_not_found,labelsym.realname);
                  end
                else
                  CGMessage(cg_e_interprocedural_goto_only_to_outer_scope_allowed);
              end
            else
              CGMessage1(cg_e_goto_label_not_found,labelsym.realname);
          end;

        { check if we don't mess with exception blocks }
        if assigned(labelnode) and
           (exceptionblock<>labelnode.exceptionblock) then
          CGMessage(cg_e_goto_inout_of_exception_block);
      end;


   function tgotonode.dogetcopy : tnode;
     var
       p : tgotonode;
     begin
        p:=tgotonode(inherited dogetcopy);
        p.exceptionblock:=exceptionblock;

        { generate labelnode if not done yet }
        if not(assigned(labelnode)) then
          begin
            if assigned(labelsym) and assigned(labelsym.code) then
              labelnode:=tlabelnode(labelsym.code)
          end;

        p.labelsym:=labelsym;
        if assigned(labelnode) then
          p.labelnode:=tlabelnode(labelnode.dogetcopy)
        else
          begin
            { don't trigger IE when there was already an error, i.e. the
              label is not defined. See tw11763 (PFV) }
            if (errorcount=0) and
            { don't trigger IE if it's a global goto }
               ((assigned(labelsym.owner) and (current_procinfo.procdef.parast.symtablelevel=labelsym.owner.symtablelevel)) or
               not(assigned(labelsym.owner))) then
              internalerror(200610291);
          end;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

        p.labelsym:=labelsym;
        p.labelnode:=tlabelnode(labelnode.dogetcopy);
>>>>>>> graemeg/fixes_2_2
        result:=p;
     end;


    function tgotonode.docompare(p: tnode): boolean;
      begin
        docompare := false;
      end;


{*****************************************************************************
                             TLABELNODE
*****************************************************************************}

    constructor tlabelnode.create(l:tnode;alabsym:tlabelsym);
      begin
        inherited create(labeln,l);
<<<<<<< HEAD
        exceptionblock:=current_exceptblock;
=======
        exceptionblock:=aktexceptblock;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        labsym:=alabsym;
        { Register labelnode in labelsym }
        labsym.code:=self;
      end;


    constructor tlabelnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        exceptionblock:=ppufile.getbyte;
      end;


    destructor tlabelnode.destroy;
      begin
        { Remove reference in labelsym, this is to prevent
          goto's to this label }
        if assigned(labsym) and (labsym.code=pointer(self)) then
          labsym.code:=nil;
        inherited destroy;
      end;


    procedure tlabelnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(exceptionblock);
      end;


    procedure tlabelnode.buildderefimpl;
      begin
        inherited buildderefimpl;
      end;


    procedure tlabelnode.derefimpl;
      begin
        inherited derefimpl;
      end;


    function tlabelnode.pass_typecheck:tnode;
      begin
        result:=nil;
        { left could still be unassigned }
        if assigned(left) then
         typecheckpass(left);
        resultdef:=voidtype;
      end;


    function tlabelnode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_VOID;

        include(current_procinfo.flags,pi_has_label);

        if assigned(labsym) and labsym.nonlocal then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          begin
            include(current_procinfo.flags,pi_has_interproclabel);
            exclude(current_procinfo.procdef.procoptions,po_inline);
          end;
=======
          include(current_procinfo.flags,pi_has_interproclabel);
>>>>>>> graemeg/cpstrnew
=======
          include(current_procinfo.flags,pi_has_interproclabel);
>>>>>>> graemeg/cpstrnew
=======
          include(current_procinfo.flags,pi_has_interproclabel);
>>>>>>> graemeg/cpstrnew
=======
          include(current_procinfo.flags,pi_has_interproclabel);
>>>>>>> origin/cpstrnew

        if assigned(left) then
          firstpass(left);
        if (m_non_local_goto in current_settings.modeswitches) and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            { the owner can be Nil for internal labels }
            assigned(labsym.owner) and
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          (current_procinfo.procdef.parast.symtablelevel<>labsym.owner.symtablelevel) then
          CGMessage(cg_e_labels_cannot_defined_outside_declaration_scope)
      end;


   function tlabelnode.dogetcopy : tnode;
     begin
       if not(assigned(copiedto)) then
         copiedto:=tlabelnode(inherited dogetcopy);
       copiedto.exceptionblock:=exceptionblock;

       result:=copiedto;
     end;


    function tlabelnode.docompare(p: tnode): boolean;
      begin
        docompare := false;
      end;


{*****************************************************************************
                            TRAISENODE
*****************************************************************************}

    constructor traisenode.create(l,taddr,tframe:tnode);
      begin
         inherited create(raisen,l,taddr,tframe);
      end;


    function traisenode.pass_typecheck:tnode;
      begin
         result:=nil;
         resultdef:=voidtype;
         if assigned(left) then
           begin
             { first para must be a _class_ }
             typecheckpass(left);
             set_varstate(left,vs_read,[vsf_must_be_valid]);
             if codegenerror then
              exit;
             if not is_class(left.resultdef) and
                not is_javaclass(left.resultdef) then
               CGMessage1(type_e_class_type_expected,left.resultdef.typename);
             { insert needed typeconvs for addr,frame }
             if assigned(right) then
               begin
                 { addr }
                 typecheckpass(right);
                 inserttypeconv(right,voidcodepointertype);
                 { frame }
                 if assigned(third) then
                  begin
                    typecheckpass(third);
                    inserttypeconv(third,voidpointertype);
                  end;
               end;
           end;
      end;


    function traisenode.pass_1 : tnode;
      var
        statements : tstatementnode;
        current_addr : tlabelnode;
        raisenode : tcallnode;
      begin
        result:=internalstatements(statements);

        if assigned(left) then
          begin
            { first para must be a class }
            firstpass(left);
            { insert needed typeconvs for addr,frame }
            if assigned(right) then
              begin
                { addr }
                firstpass(right);
                { frame }
                if assigned(third) then
                  firstpass(third)
                else
                  third:=cpointerconstnode.Create(0,voidpointertype);
              end
            else
              begin
                third:=cinlinenode.create(in_get_frame,false,nil);
                current_addr:=clabelnode.create(cnothingnode.create,clabelsym.create('$raiseaddr'));
                addstatement(statements,current_addr);
                right:=caddrnode.create(cloadnode.create(current_addr.labsym,current_addr.labsym.owner));
              end;

            raisenode:=ccallnode.createintern('fpc_raiseexception',
              ccallparanode.create(third,
              ccallparanode.create(right,
              ccallparanode.create(left,nil)))
              );
            include(raisenode.callnodeflags,cnf_call_never_returns);
            addstatement(statements,raisenode);
          end
        else
          begin
            addstatement(statements,ccallnode.createintern('fpc_popaddrstack',nil));
            raisenode:=ccallnode.createintern('fpc_reraise',nil);
            include(raisenode.callnodeflags,cnf_call_never_returns);
            addstatement(statements,raisenode);
          end;
        left:=nil;
        right:=nil;
        third:=nil;
      end;

{*****************************************************************************
                             TTRYEXCEPTNODE
*****************************************************************************}

    constructor ttryexceptnode.create(l,r,_t1 : tnode);
      begin
         inherited create(tryexceptn,l,r,_t1,nil);
      end;


    function ttryexceptnode.pass_typecheck:tnode;
      begin
        result:=nil;
        typecheckpass(left);
        { on statements }
        if assigned(right) then
          typecheckpass(right);
        { else block }
        if assigned(t1) then
          typecheckpass(t1);
        resultdef:=voidtype;
      end;


    function ttryexceptnode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_VOID;
        firstpass(left);
        { on statements }
        if assigned(right) then
          firstpass(right);
        { else block }
        if assigned(t1) then
          firstpass(t1);

        include(current_procinfo.flags,pi_do_call);
        include(current_procinfo.flags,pi_uses_exceptions);

        adjust_estimated_stack_size;
      end;


    function ttryexceptnode.simplify(forinline: boolean): tnode;
      begin
        result:=nil;
        { empty try -> can never raise exception -> do nothing }
        if has_no_code(left) then
          result:=cnothingnode.create;
      end;

    procedure ttryexceptnode.adjust_estimated_stack_size;
      begin
        inc(current_procinfo.estimatedtempsize,rec_jmp_buf.size*2);
      end;


{*****************************************************************************
                           TTRYFINALLYNODE
*****************************************************************************}

    constructor ttryfinallynode.create(l,r:tnode);
      begin
        inherited create(tryfinallyn,l,r,nil,nil);
        implicitframe:=false;
      end;


    constructor ttryfinallynode.create_implicit(l,r,_t1:tnode);
      begin
        inherited create(tryfinallyn,l,r,_t1,nil);
        implicitframe:=true;
      end;


    function ttryfinallynode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=voidtype;

        typecheckpass(left);
        // "try block" is "used"? (JM)
        set_varstate(left,vs_readwritten,[vsf_must_be_valid]);

        typecheckpass(right);
        // "except block" is "used"? (JM)
        set_varstate(right,vs_readwritten,[vsf_must_be_valid]);

        { special finally block only executed when there was an exception }
        if assigned(t1) then
          begin
            typecheckpass(t1);
            // "finally block" is "used"? (JM)
            set_varstate(t1,vs_readwritten,[vsf_must_be_valid]);
          end;
      end;


    function ttryfinallynode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_VOID;
        firstpass(left);

        firstpass(right);

        if assigned(t1) then
          firstpass(t1);

        include(current_procinfo.flags,pi_do_call);

        { pi_uses_exceptions is an information for the optimizer and it
          is only interested in exceptions if they appear inside the body,
          so ignore implicit frames when setting the flag }
        if not(implicitframe) then
          include(current_procinfo.flags,pi_uses_exceptions);

        adjust_estimated_stack_size;
      end;


   function ttryfinallynode.simplify(forinline : boolean): tnode;
     begin
       result:=nil;
       { if the try contains no code, we can kill
         the try and except and return only the
         finally part }
       if has_no_code(left) then
         begin
           result:=right;
           right:=nil;
         end;
     end;


    var
      seq: longint=0;

    function ttryfinallynode.create_finalizer_procdef: tprocdef;
      var
        st:TSymTable;
        checkstack: psymtablestackitem;
        oldsymtablestack: tsymtablestack;
        sym:tprocsym;
      begin
        { get actual procedure symtable (skip withsymtables, etc.) }
        st:=nil;
        checkstack:=symtablestack.stack;
        while assigned(checkstack) do
          begin
            st:=checkstack^.symtable;
            if st.symtabletype in [staticsymtable,globalsymtable,localsymtable] then
              break;
            checkstack:=checkstack^.next;
          end;
        { Create a nested procedure, even from main_program_level.
          Furthermore, force procdef and procsym into the same symtable
          (by default, defs are registered with symtablestack.top which may be
          something temporary like exceptsymtable - in that case, procdef can be
          destroyed before procsym, leaving invalid pointers). }
        oldsymtablestack:=symtablestack;
        symtablestack:=nil;
        result:=cprocdef.create(max(normal_function_level,st.symtablelevel)+1,true);
        symtablestack:=oldsymtablestack;
        st.insertdef(result);
        result.struct:=current_procinfo.procdef.struct;
        { tabstractprocdef constructor sets po_delphi_nested_cc whenever
          nested procvars modeswitch is active. We must be independent of this switch. }
        exclude(result.procoptions,po_delphi_nested_cc);
        result.proctypeoption:=potype_exceptfilter;
        handle_calling_convention(result);
        sym:=cprocsym.create('$fin$'+tostr(seq));
        st.insert(sym);
        inc(seq);

        result.procsym:=sym;
        proc_add_definition(result);
        result.forwarddef:=false;
        result.aliasnames.insert(result.mangledname);
      end;


    procedure ttryfinallynode.adjust_estimated_stack_size;
      begin
        inc(current_procinfo.estimatedtempsize,rec_jmp_buf.size);
      end;


{*****************************************************************************
                                TONNODE
*****************************************************************************}

    constructor tonnode.create(l,r:tnode);
      begin
         inherited create(onn,l,r);
         excepTSymtable:=nil;
         excepttype:=nil;
      end;


    destructor tonnode.destroy;
      begin
        { copied nodes don't need to release the symtable }
        if assigned(excepTSymtable) then
         excepTSymtable.free;
        inherited destroy;
      end;


    constructor tonnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        excepTSymtable:=nil;
        excepttype:=nil;
      end;


    function tonnode.dogetcopy : tnode;
      var
         n : tonnode;
      begin
         n:=tonnode(inherited dogetcopy);
         if assigned(exceptsymtable) then
           n.exceptsymtable:=exceptsymtable.getcopy
         else
           n.exceptsymtable:=nil;
         n.excepttype:=excepttype;
         result:=n;
      end;


    function tonnode.pass_typecheck:tnode;
      begin
         result:=nil;
         resultdef:=voidtype;
         if not is_class(excepttype) and
            not is_javaclass(excepttype) then
           CGMessage1(type_e_class_type_expected,excepttype.typename);
         if assigned(left) then
           typecheckpass(left);
         if assigned(right) then
           typecheckpass(right);
      end;


    function tonnode.pass_1 : tnode;
      begin
         result:=nil;
         include(current_procinfo.flags,pi_do_call);
         { Loads exception class VMT, therefore may need GOT
           (generic code only; descendants may need to avoid this check) }
         if (cs_create_pic in current_settings.moduleswitches) and
           (tf_pic_uses_got in target_info.flags) then
           include(current_procinfo.flags,pi_needs_got);
         expectloc:=LOC_VOID;
         if assigned(left) then
           firstpass(left);

         if assigned(right) then
           firstpass(right);
      end;


    function tonnode.docompare(p: tnode): boolean;
      begin
        docompare := false;
      end;

end.
