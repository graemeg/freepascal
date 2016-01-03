{
    Common subexpression elimination on base blocks

    Copyright (c) 2005-2012 by Florian Klaempfl

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
unit optcse;

{$i fpcdefs.inc}

{ $define csedebug}
<<<<<<< HEAD
<<<<<<< HEAD
{ $define csestats}
=======
{$define csestats}
>>>>>>> graemeg/fixes_2_2
=======
{$define csestats}
>>>>>>> origin/fixes_2_2

  interface

    uses
      node;

<<<<<<< HEAD
<<<<<<< HEAD
    {
      the function  creates non optimal code so far:
      - call para nodes are cse barriers because they can be reordered and thus the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        temp. creation could be done too late
      - the cse knows nothing about register pressure. In case of high register pressure, cse might
        have a negative impact
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        temp. creation can be done too late
      - cse's in chained expressions are not recognized: the common subexpression
        in (a1 and b and c) vs. (a2 and b and c) is not recognized because there is no common
        subtree b and c
      - the cse knows nothing about register pressure. In case of high register pressure, cse might
        have a negative impact
      - assignment nodes are currently cse borders: things like a[i,j]:=a[i,j]+1; are not improved
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
      - the list of cseinvariant node types and inline numbers is not complete yet

      Further, it could be done probably in a faster way though the complexity can't probably not reduced
    }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function do_optcse(var rootnode : tnode) : tnode;

  implementation

    uses
<<<<<<< HEAD
<<<<<<< HEAD
      globtype,globals,
      cutils,cclasses,
      nutils,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      nbas,nld,ninl,ncal,nadd,nmem,
      pass_1,
      symconst,symdef,symsym,
=======
      procinfo,
      nbas,nld,ninl,ncal,ncnv,nadd,
      pass_1,
      symconst,symtype,symdef,symsym,
>>>>>>> graemeg/cpstrnew
=======
      procinfo,
      nbas,nld,ninl,ncal,ncnv,nadd,
      pass_1,
      symconst,symtype,symdef,symsym,
>>>>>>> graemeg/cpstrnew
=======
      procinfo,
      nbas,nld,ninl,ncal,ncnv,nadd,
      pass_1,
      symconst,symtype,symdef,symsym,
>>>>>>> graemeg/cpstrnew
=======
      procinfo,
      nbas,nld,ninl,ncal,ncnv,nadd,
      pass_1,
      symconst,symtype,symdef,symsym,
>>>>>>> origin/cpstrnew
      defutil,
      optbase;

    const
      cseinvariant : set of tnodetype = [addn,muln,subn,divn,slashn,modn,andn,orn,xorn,notn,vecn,
        derefn,equaln,unequaln,ltn,gtn,lten,gten,typeconvn,subscriptn,
<<<<<<< HEAD
        inn,symdifn,shrn,shln,ordconstn,realconstn,unaryminusn,pointerconstn,stringconstn,setconstn,niln,
        setelementn,{arrayconstructorn,arrayconstructorrangen,}
        isn,asn,starstarn,nothingn,temprefn,loadparentfpn {,callparan},assignn,addrn];
=======
        inn,symdifn,shrn,shln,ordconstn,realconstn,unaryminusn,pointerconstn,stringconstn,setconstn,
        isn,asn,starstarn,nothingn,temprefn,loadparentfpn {,callparan}];
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

    function searchsubdomain(var n:tnode; arg: pointer) : foreachnoderesult;
      begin
        if (n.nodetype in cseinvariant) or
          ((n.nodetype=inlinen) and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           (tinlinenode(n).inlinenumber in [in_length_x,in_assigned_x,in_sqr_real,in_sqrt_real,in_sin_real,in_cos_real,in_abs_long,
             in_abs_real,in_exp_real,in_ln_real,in_pi_real,in_popcnt_x,in_arctan_real,in_round_real,in_trunc_real,
             { cse on fma will still not work because it would require proper handling of call nodes
               with more than one parameter }
             in_fma_single,in_fma_double,in_fma_extended,in_fma_float128])
          ) or
          ((n.nodetype=callparan) and not(assigned(tcallparanode(n).right))) or
=======
           (tinlinenode(n).inlinenumber in [in_assigned_x])
          ) or
>>>>>>> graemeg/cpstrnew
=======
           (tinlinenode(n).inlinenumber in [in_assigned_x])
          ) or
>>>>>>> graemeg/cpstrnew
=======
           (tinlinenode(n).inlinenumber in [in_assigned_x])
          ) or
>>>>>>> graemeg/cpstrnew
=======
           (tinlinenode(n).inlinenumber in [in_assigned_x])
          ) or
>>>>>>> origin/cpstrnew
          ((n.nodetype=loadn) and
            not((tloadnode(n).symtableentry.typ in [staticvarsym,localvarsym,paravarsym]) and
                (vo_volatile in tabstractvarsym(tloadnode(n).symtableentry).varoptions))
          ) then
          result:=fen_true
        else
          begin
            pboolean(arg)^:=false;
            result:=fen_norecurse_true;
          end;
=======
=======
>>>>>>> origin/fixes_2_2
      globtype,
      cclasses,
      verbose,
      nutils,
      nbas,nld,
      pass_1,
      symtype,symdef;

    const
      cseinvariant : set of tnodetype = [loadn,addn,muln,subn,divn,slashn,modn,andn,orn,xorn,notn,vecn,
        derefn,equaln,unequaln,ltn,gtn,lten,gten,typeconvn,subscriptn,
        inn,symdifn,shrn,shln,ordconstn,realconstn,unaryminusn,pointerconstn,stringconstn,setconstn,
        isn,asn,starstarn,nothingn];

    function searchsubdomain(var n:tnode; arg: pointer) : foreachnoderesult;
      begin
        if not(n.nodetype in cseinvariant) then
          begin
            pboolean(arg)^:=false;
            result:=fen_norecurse_true;
          end
        else
          result:=fen_true;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;

    type
      tlists = record
        nodelist : tfplist;
        locationlist : tfplist;
<<<<<<< HEAD
<<<<<<< HEAD
        equalto : tfplist;
        refs : tfplist;
        avail : TDFASet;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;

      plists = ^tlists;

<<<<<<< HEAD
<<<<<<< HEAD
    { collectnodes needs the address of itself to call foreachnodestatic,
      so we need a wrapper because @<func> inside <func doesn't work }

    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;forward;

    function collectnodes2(var n:tnode; arg: pointer) : foreachnoderesult;
      begin
        result:=collectnodes(n,arg);
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;

      { when compiling a tree like
            and
            / \
          and  C
          / \
         A   B
        all expressions of B are available during evaluation of C. However considerung the whole expression,
        values of B and C might not be available due to short boolean evaluation.

        So recurseintobooleanchain detectes such chained and/or expressions and makes sub-expressions of B
        available during the evaluation of C

        firstleftend is later used to remove all sub expressions of B and C by storing the expression count
        in the cse table after handling A
      }
      var
        firstleftend : longint;
      procedure recurseintobooleanchain(t : tnodetype;n : tnode);
        begin
          if (tbinarynode(n).left.nodetype=t) and is_boolean(tbinarynode(n).left.resultdef) then
            recurseintobooleanchain(t,tbinarynode(n).left)
          else
            foreachnodestatic(pm_postprocess,tbinarynode(n).left,@collectnodes2,arg);
          firstleftend:=min(plists(arg)^.nodelist.count,firstleftend);
          foreachnodestatic(pm_postprocess,tbinarynode(n).right,@collectnodes2,arg);
        end;

      var
        i : longint;
=======
    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;
      var
        i,j : longint;
>>>>>>> graemeg/cpstrnew
=======
    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;
      var
        i,j : longint;
>>>>>>> graemeg/cpstrnew
=======
    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;
      var
        i,j : longint;
>>>>>>> graemeg/cpstrnew
=======
    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;
      var
        i,j : longint;
>>>>>>> origin/cpstrnew
      begin
        result:=fen_false;
        { don't add the tree below an untyped const parameter: there is
          no information available that this kind of tree actually needs
          to be addresable, this could be improved }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        { the nodes below a type conversion node created for an absolute
          reference cannot be handled separately, because the absolute reference
          may have special requirements (no regability, must be in memory, ...)
        }
        if (((n.nodetype=callparan) and
             (tcallparanode(n).left.resultdef.typ=formaldef) and
             (tcallparanode(n).parasym.varspez=vs_const)) or
            ((n.nodetype=typeconvn) and
             (nf_absolute in n.flags))
           ) then
=======
        if ((n.nodetype=callparan) and
          (tcallparanode(n).left.resultdef.typ=formaldef) and
          (tcallparanode(n).parasym.varspez=vs_const)) then
>>>>>>> graemeg/cpstrnew
=======
        if ((n.nodetype=callparan) and
          (tcallparanode(n).left.resultdef.typ=formaldef) and
          (tcallparanode(n).parasym.varspez=vs_const)) then
>>>>>>> graemeg/cpstrnew
=======
        if ((n.nodetype=callparan) and
          (tcallparanode(n).left.resultdef.typ=formaldef) and
          (tcallparanode(n).parasym.varspez=vs_const)) then
>>>>>>> graemeg/cpstrnew
=======
        if ((n.nodetype=callparan) and
          (tcallparanode(n).left.resultdef.typ=formaldef) and
          (tcallparanode(n).parasym.varspez=vs_const)) then
>>>>>>> origin/cpstrnew
          begin
            result:=fen_norecurse_false;
            exit;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if
          { node possible to add? }
          assigned(n.resultdef) and
          (
            { regable expressions }
            (actualtargetnode(@n)^.flags*[nf_write,nf_modify,nf_address_taken]=[]) and
            ((((tstoreddef(n.resultdef).is_intregable or tstoreddef(n.resultdef).is_fpuregable or tstoreddef(n.resultdef).is_const_intregable) and
            { is_int/fpuregable allows arrays and records to be in registers, cse cannot handle this }
            (not(n.resultdef.typ in [arraydef,recorddef]))) or is_dynamic_array(n.resultdef)) and
            { same for voiddef }
            not(is_void(n.resultdef)) and
            { adding tempref and callpara nodes itself is worthless but
              their complexity is probably <= 1 anyways

              neither add setelementn nodes because the compiler sometimes depends on the fact
              that a certain node stays a setelementn, this does not hurt either because
              setelementn nodes itself generate no real code (except moving data into register) }
            not(n.nodetype in [temprefn,callparan,setelementn]) and
=======
    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;
      begin
        { node worth to add? }
        if (node_complexity(n)>1) and (tstoreddef(n.resultdef).is_intregable or tstoreddef(n.resultdef).is_fpuregable) and
          { adding tempref nodes is worthless but there complexity is probably <= 1 anyways }
          not(n.nodetype in [temprefn]) then
          begin
            plists(arg)^.nodelist.Add(n);
            plists(arg)^.locationlist.Add(@n);
            result:=fen_false;
          end
        else
          result:=fen_norecurse_false;
      end;


    function searchcsedomain(var n: tnode; arg: pointer) : foreachnoderesult;
      var
        csedomain : boolean;
        lists : tlists;
        templist : tfplist;
        i,j : longint;
        def : tstoreddef;
        nodes : tblocknode;
        creates,
        statements : tstatementnode;
        hp : ttempcreatenode;
      begin
        result:=fen_false;
        if n.nodetype in cseinvariant then
          begin
            csedomain:=true;
            foreachnodestatic(pm_postprocess,n,@searchsubdomain,@csedomain);
            { found a cse domain }
            if csedomain then
              begin
                statements:=nil;
                result:=fen_norecurse_true;
{$ifdef csedebug}
                writeln('============ cse domain ==================');
                printnode(output,n);
{$endif csedebug}

                lists.nodelist:=tfplist.create;
                lists.locationlist:=tfplist.create;
                foreachnodestatic(pm_postprocess,n,@collectnodes,@lists);

                templist:=tfplist.create;
                templist.count:=lists.nodelist.count;

                { this is poorly coded, just comparing every node with all other nodes }
                for i:=0 to lists.nodelist.count-1 do
                  for j:=i+1 to lists.nodelist.count-1 do
                    begin
                      if not(tnode(lists.nodelist[i]).nodetype in [tempcreaten,temprefn]) and
                        tnode(lists.nodelist[i]).isequal(tnode(lists.nodelist[j])) then
                        begin
                          if not(assigned(statements)) then
                            begin
                              nodes:=internalstatements(statements);
                              addstatement(statements,internalstatements(creates));
                            end;
{$if defined(csedebug) or defined(csestats)}
                          writeln('    ====     ');
                          printnode(output,tnode(lists.nodelist[i]));
                          writeln('    equals   ');
                          printnode(output,tnode(lists.nodelist[j]));
                          writeln('    ====     ');
{$endif defined(csedebug) or defined(csestats)}

                          def:=tstoreddef(tnode(lists.nodelist[i]).resultdef);
                          if assigned(def) then
                            begin
                              if assigned(templist[i])  then
                                begin
                                  templist[j]:=templist[i];
                                  pnode(lists.locationlist[j])^.free;
                                  pnode(lists.locationlist[j])^:=ctemprefnode.create(ttempcreatenode(templist[j]));
                                  do_firstpass(pnode(lists.locationlist[j])^);
                                end
                              else
                                begin
                                  templist[i]:=ctempcreatenode.create(def,def.size,tt_persistent,
                                    def.is_intregable or def.is_fpuregable);
                                  addstatement(creates,tnode(templist[i]));

                                  { properties can't be passed by var }
                                  hp:=ttempcreatenode(templist[i]);
                                  do_firstpass(tnode(hp));

                                  addstatement(statements,cassignmentnode.create(ctemprefnode.create(ttempcreatenode(templist[i])),
                                    tnode(lists.nodelist[i])));
                                  pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[i]));
                                  do_firstpass(pnode(lists.locationlist[i])^);

                                  templist[j]:=templist[i];

                                  pnode(lists.locationlist[j])^.free;
                                  pnode(lists.locationlist[j])^:=ctemprefnode.create(ttempcreatenode(templist[j]));
                                  do_firstpass(pnode(lists.locationlist[j])^);
{$ifdef csedebug}
                                  printnode(output,statements);
{$endif csedebug}
                                end;
                              end
                            else
                              internalerror(2007091701);
                        end;
                    end;
                if assigned(statements) then
                  begin
                    addstatement(statements,n);
                    n:=nodes;
                    do_firstpass(n);
{$ifdef csedebug}
                    printnode(output,nodes);
{$endif csedebug}
                  end;
{$ifdef csedebug}
                writeln('nodes: ',lists.nodelist.count);
                writeln('==========================================');
{$endif csedebug}
                lists.nodelist.free;
                lists.locationlist.free;
                templist.free;
              end
          end;
      end;


    function do_optcse(var rootnode : tnode) : tnode;
      begin
=======
    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;
      begin
        { node worth to add? }
        if (node_complexity(n)>1) and (tstoreddef(n.resultdef).is_intregable or tstoreddef(n.resultdef).is_fpuregable) and
          { adding tempref nodes is worthless but there complexity is probably <= 1 anyways }
          not(n.nodetype in [temprefn]) then
          begin
            plists(arg)^.nodelist.Add(n);
            plists(arg)^.locationlist.Add(@n);
            result:=fen_false;
          end
        else
          result:=fen_norecurse_false;
      end;


    function searchcsedomain(var n: tnode; arg: pointer) : foreachnoderesult;
      var
        csedomain : boolean;
        lists : tlists;
        templist : tfplist;
        i,j : longint;
        def : tstoreddef;
        nodes : tblocknode;
        creates,
        statements : tstatementnode;
        hp : ttempcreatenode;
      begin
        result:=fen_false;
        if n.nodetype in cseinvariant then
          begin
            csedomain:=true;
            foreachnodestatic(pm_postprocess,n,@searchsubdomain,@csedomain);
            { found a cse domain }
            if csedomain then
              begin
                statements:=nil;
                result:=fen_norecurse_true;
{$ifdef csedebug}
                writeln('============ cse domain ==================');
                printnode(output,n);
{$endif csedebug}

                lists.nodelist:=tfplist.create;
                lists.locationlist:=tfplist.create;
                foreachnodestatic(pm_postprocess,n,@collectnodes,@lists);

                templist:=tfplist.create;
                templist.count:=lists.nodelist.count;

                { this is poorly coded, just comparing every node with all other nodes }
                for i:=0 to lists.nodelist.count-1 do
                  for j:=i+1 to lists.nodelist.count-1 do
                    begin
                      if not(tnode(lists.nodelist[i]).nodetype in [tempcreaten,temprefn]) and
                        tnode(lists.nodelist[i]).isequal(tnode(lists.nodelist[j])) then
                        begin
                          if not(assigned(statements)) then
                            begin
                              nodes:=internalstatements(statements);
                              addstatement(statements,internalstatements(creates));
                            end;
{$if defined(csedebug) or defined(csestats)}
                          writeln('    ====     ');
                          printnode(output,tnode(lists.nodelist[i]));
                          writeln('    equals   ');
                          printnode(output,tnode(lists.nodelist[j]));
                          writeln('    ====     ');
{$endif defined(csedebug) or defined(csestats)}

                          def:=tstoreddef(tnode(lists.nodelist[i]).resultdef);
                          if assigned(def) then
                            begin
                              if assigned(templist[i])  then
                                begin
                                  templist[j]:=templist[i];
                                  pnode(lists.locationlist[j])^.free;
                                  pnode(lists.locationlist[j])^:=ctemprefnode.create(ttempcreatenode(templist[j]));
                                  do_firstpass(pnode(lists.locationlist[j])^);
                                end
                              else
                                begin
                                  templist[i]:=ctempcreatenode.create(def,def.size,tt_persistent,
                                    def.is_intregable or def.is_fpuregable);
                                  addstatement(creates,tnode(templist[i]));

                                  { properties can't be passed by var }
                                  hp:=ttempcreatenode(templist[i]);
                                  do_firstpass(tnode(hp));

                                  addstatement(statements,cassignmentnode.create(ctemprefnode.create(ttempcreatenode(templist[i])),
                                    tnode(lists.nodelist[i])));
                                  pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[i]));
                                  do_firstpass(pnode(lists.locationlist[i])^);

                                  templist[j]:=templist[i];

                                  pnode(lists.locationlist[j])^.free;
                                  pnode(lists.locationlist[j])^:=ctemprefnode.create(ttempcreatenode(templist[j]));
                                  do_firstpass(pnode(lists.locationlist[j])^);
{$ifdef csedebug}
                                  printnode(output,statements);
{$endif csedebug}
                                end;
                              end
                            else
                              internalerror(2007091701);
                        end;
                    end;
                if assigned(statements) then
                  begin
                    addstatement(statements,n);
                    n:=nodes;
                    do_firstpass(n);
{$ifdef csedebug}
                    printnode(output,nodes);
{$endif csedebug}
                  end;
{$ifdef csedebug}
                writeln('nodes: ',lists.nodelist.count);
                writeln('==========================================');
{$endif csedebug}
                lists.nodelist.free;
                lists.locationlist.free;
                templist.free;
              end
          end;
      end;


    function do_optcse(var rootnode : tnode) : tnode;
      begin
>>>>>>> origin/fixes_2_2
        foreachnodestatic(pm_postprocess,rootnode,@searchcsedomain,nil);
        result:=nil;
(*
        { create a linear list of nodes }
>>>>>>> graemeg/fixes_2_2

            { node worth to add?

              We consider almost every node because even loading a variables from
              a register instead of memory is more beneficial. This behaviour should
              not increase register pressure because if a variable is already
              in a register, the reg. allocator can merge the nodes. If a variable
              is loaded from memory, loading this variable and spilling another register
              should not add a speed penalty.
            }
            {
              load nodes are not considered if they load para or local symbols from the
              current stack frame, those are in registers anyways if possible
            }
            (not(actualtargetnode(@n)^.nodetype=loadn) or
             not(tloadnode(actualtargetnode(@n)^).symtableentry.typ in [paravarsym,localvarsym,staticvarsym]) or
             { apply cse on non-regable variables }
             ((tloadnode(actualtargetnode(@n)^).symtableentry.typ in [paravarsym,localvarsym,staticvarsym]) and
               not(tabstractvarsym(tloadnode(actualtargetnode(@n)^).symtableentry).is_regvar(false)) and
               not(vo_volatile in tabstractvarsym(tloadnode(actualtargetnode(@n)^).symtableentry).varoptions)) or
             (node_complexity(n)>1)
            ) and

            {
              Const nodes however are only considered if their complexity is >1
              This might be the case for the risc architectures if they need
              more than one instruction to load this particular value
            }
            (not(is_constnode(n)) or (node_complexity(n)>1)))
{$if not(defined(i386)) and not(defined(i8086))}
            or
            { store reference of expression? }

            { loading the address of a global symbol takes typically more than
              one instruction on every platform except i8086/i386
              so consider in this case loading the address of the data
            }
            (((n.resultdef.typ in [arraydef,recorddef]) or is_object(n.resultdef)) and not(is_dynamic_array(n.resultdef)) and
             (n.nodetype=loadn) and
             (tloadnode(n).symtableentry.typ=staticvarsym)
            )
{$endif not(defined(i386)) and not(defined(i8086))}
          ) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        { so far, we can handle only nodes being read }
        if (n.flags*[nf_write,nf_modify]=[]) and
          { node possible to add? }
          assigned(n.resultdef) and
          (tstoreddef(n.resultdef).is_intregable or tstoreddef(n.resultdef).is_fpuregable) and
          { is_int/fpuregable allows arrays and records to be in registers, cse cannot handle this }
          not(n.resultdef.typ in [arraydef,recorddef]) and
          { same for voiddef }
          not(is_void(n.resultdef)) and
          { adding tempref nodes is worthless but their complexity is probably <= 1 anyways }
          not(n.nodetype in [temprefn]) and

          { node worth to add?

            We consider almost every node because even loading a variables from
            a register instead of memory is more beneficial. This behaviour should
            not increase register pressure because if a variable is already
            in a register, the reg. allocator can merge the nodes. If a variable
            is loaded from memory, loading this variable and spilling another register
            should not add a speed penalty.
          }
          {
            load nodes are not considered if they load para or local symbols from the
            current stack frame, those are in registers anyways if possible
          }
          (not(n.nodetype=loadn) or
           not(tloadnode(n).symtableentry.typ in [paravarsym,localvarsym]) or
           (tloadnode(n).symtable.symtablelevel<>current_procinfo.procdef.parast.symtablelevel)
          ) and

          {
            Const nodes however are only considered if their complexity is >1
            This might be the case for the risc architectures if they need
            more than one instruction to load this particular value
          }
          (not(is_constnode(n)) or (node_complexity(n)>1)) then
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
          begin
            plists(arg)^.nodelist.Add(n);
            plists(arg)^.locationlist.Add(@n);
            plists(arg)^.refs.Add(nil);
            plists(arg)^.equalto.Add(pointer(-1));

            DFASetInclude(plists(arg)^.avail,plists(arg)^.nodelist.count-1);

            for i:=0 to plists(arg)^.nodelist.count-2 do
              begin
                if tnode(plists(arg)^.nodelist[i]).isequal(n) and DFASetIn(plists(arg)^.avail,i) then
                  begin
                    { use always the first occurence }
                    if plists(arg)^.equalto[i]<>pointer(-1) then
                      plists(arg)^.equalto[plists(arg)^.nodelist.count-1]:=plists(arg)^.equalto[i]
                    else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                      plists(arg)^.equalto[plists(arg)^.nodelist.count-1]:=pointer(ptrint(i));
                    plists(arg)^.refs[i]:=pointer(plists(arg)^.refs[i])+1;
                    { tree has been found, no need to search further,
                      sub-trees have been added by the first occurence of
                      the tree already }
                    result:=fen_norecurse_false;
                    break;
                  end;
              end;
          end;

        { boolean and/or require a special handling: after evaluating the and/or node,
          the expressions of the right side might not be available due to short boolean
          evaluation, so after handling the right side, mark those expressions
          as unavailable }
        if (n.nodetype in [orn,andn]) and is_boolean(taddnode(n).left.resultdef) then
          begin
<<<<<<< HEAD
            firstleftend:=high(longint);
            recurseintobooleanchain(n.nodetype,n);
            for i:=firstleftend to plists(arg)^.nodelist.count-1 do
              DFASetExclude(plists(arg)^.avail,i);
            result:=fen_norecurse_false;
          end;
{$ifdef cpuhighleveltarget}
          { The high level targets use the functionality from ncgnstld for
            nested accesses, and that one stores the complete location of the
            nested variable in tloadnode.left rather than only the location of
            the parent context containing it. This causes problems with the
            CSE in case the nested variable is used as an lvalue, so disable
            CSE in that case
          }
          if (n.nodetype=loadn) and assigned(tloadnode(n).left) then
            result:=fen_norecurse_false;
{$endif}
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                      plists(arg)^.equalto[plists(arg)^.nodelist.count-1]:=pointer(i);
                    plists(arg)^.refs[i]:=pointer(plists(arg)^.refs[i])+1;
                    break;
                  end;
              end;

            { boolean and/or require a special handling: after evaluating the and/or node,
              the expressions of the right side might not be available due to short boolean
              evaluation, so after handling the right side, mark those expressions
              as unavailable }
            if (n.nodetype in [orn,andn]) and is_boolean(taddnode(n).left.resultdef) then
=======
            { and then search backward so we get always the largest equal trees }
            j:=i+1;
            { collect equal nodes }
            while (j<=nodelist.length-1) and
              nodelist[i].isequal(nodelist[j]) do
              inc(j);
            dec(j);
            if j>i then
>>>>>>> graemeg/fixes_2_2
              begin
                foreachnodestatic(pm_postprocess,taddnode(n).left,@collectnodes2,arg);
                j:=plists(arg)^.nodelist.count;
                foreachnodestatic(pm_postprocess,taddnode(n).right,@collectnodes2,arg);
                for i:=j to plists(arg)^.nodelist.count-1 do
                  DFASetExclude(plists(arg)^.avail,i);
                result:=fen_norecurse_false;
              end;
          end;
*)
<<<<<<< HEAD
      end;
>>>>>>> graemeg/cpstrnew


    function searchcsedomain(var n: tnode; arg: pointer) : foreachnoderesult;
      var
        csedomain : boolean;
        lists : tlists;
        templist : tfplist;
        i : longint;
        def : tstoreddef;
        nodes : tblocknode;
        creates,
        statements : tstatementnode;
        hp : ttempcreatenode;
        addrstored : boolean;
        hp2 : tnode;
      begin
        result:=fen_false;
        nodes:=nil;
        if n.nodetype in cseinvariant then
          begin
            csedomain:=true;
            foreachnodestatic(pm_postprocess,n,@searchsubdomain,@csedomain);
            if not(csedomain) then
              begin
                { try to transform the tree to get better cse domains, consider:
                       +
                      / \
                     +   C
                    / \
                   A   B

                  if A is not cse'able but B and C are, then the compiler cannot do cse so the tree is transformed into
                       +
                      / \
                     A   +
                        / \
                       B   C
                  Because A could be another tree of this kind, the whole process is done in a while loop
                }
                if (n.nodetype in [andn,orn,addn,muln]) and
                  (n.nodetype=tbinarynode(n).left.nodetype) and
                  { do is optimizations only for integers, reals (no currency!), vectors, sets or booleans }
                  (is_integer(n.resultdef) or is_real(n.resultdef) or is_vector(n.resultdef) or is_set(n.resultdef) or
                   is_boolean(n.resultdef)) and
                  { either if fastmath is on }
                  ((cs_opt_fastmath in current_settings.optimizerswitches) or
                   { or for the logical operators, they cannot overflow }
                   (n.nodetype in [andn,orn]) or
                   { or for integers if range checking is off }
                   ((is_integer(n.resultdef) and
                    (n.localswitches*[cs_check_range,cs_check_overflow]=[]) and
                    (tbinarynode(n).left.localswitches*[cs_check_range,cs_check_overflow]=[]))) or
                   { for sets, we can do this always }
                   (is_set(n.resultdef))
                   ) then
                  while (n.nodetype=tbinarynode(n).left.nodetype) and
                        { the resulttypes of the operands we'll swap must be equal,
                          required in case of a 32x32->64 multiplication, then we
                          cannot swap out one of the 32 bit operands for a 64 bit one
                        }
                        (tbinarynode(tbinarynode(n).left).left.resultdef=tbinarynode(n).left.resultdef) and
                        (tbinarynode(n).left.resultdef=tbinarynode(n).right.resultdef) do
                    begin
                      csedomain:=true;
                      foreachnodestatic(pm_postprocess,tbinarynode(n).right,@searchsubdomain,@csedomain);
                      if csedomain then
                        begin
                          csedomain:=true;
                          foreachnodestatic(pm_postprocess,tbinarynode(tbinarynode(n).left).right,@searchsubdomain,@csedomain);
                          if csedomain then
                            begin
                              hp2:=tbinarynode(tbinarynode(n).left).left;
                              tbinarynode(tbinarynode(n).left).left:=tbinarynode(tbinarynode(n).left).right;
                              tbinarynode(tbinarynode(n).left).right:=tbinarynode(n).right;
                              tbinarynode(n).right:=tbinarynode(n).left;
                              tbinarynode(n).left:=hp2;

                              { the transformed tree could result in new possibilities to fold constants
                                so force a firstpass on the root node }
                              exclude(tbinarynode(n).right.flags,nf_pass1_done);
                              do_firstpass(tbinarynode(n).right);
                            end
                          else
                            break;
                        end
                      else
                        break;
                    end;
              end
            else
              begin
                statements:=nil;
                result:=fen_norecurse_true;
{$ifdef csedebug}
                writeln('============ cse domain ==================');
                printnode(output,n);
                writeln('Complexity: ',node_complexity(n));
{$endif csedebug}
                lists.nodelist:=tfplist.create;
                lists.locationlist:=tfplist.create;
                lists.equalto:=tfplist.create;
                lists.refs:=tfplist.create;
                foreachnodestatic(pm_postprocess,n,@collectnodes,@lists);

                templist:=tfplist.create;
                templist.count:=lists.nodelist.count;

                { check all nodes if one is used more than once }
                for i:=0 to lists.nodelist.count-1 do
                  begin
                    { current node used more than once? }
                    if assigned(lists.refs[i]) then
                      begin
                        if not(assigned(statements)) then
                          begin
                            nodes:=internalstatements(statements);
                            addstatement(statements,internalstatements(creates));
                          end;

                        def:=tstoreddef(tnode(lists.nodelist[i]).resultdef);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        { we cannot handle register stored records or array in CSE yet
                          but we can store their reference }
                        addrstored:=((def.typ in [arraydef,recorddef]) or is_object(def)) and not(is_dynamic_array(def));

                        if addrstored then
                          templist[i]:=ctempcreatenode.create_value(cpointerdef.getreusable(def),voidpointertype.size,tt_persistent,
                            true,caddrnode.create_internal(tnode(lists.nodelist[i])))
                        else
                          templist[i]:=ctempcreatenode.create_value(def,def.size,tt_persistent,
                            def.is_intregable or def.is_fpuregable or def.is_const_intregable,tnode(lists.nodelist[i]));

                        { the value described by the temp. is immutable and the temp. can be always in register

                          ttempcreatenode.create normally takes care of the register location but it does not
                          know about immutability so it cannot take care of managed types }
                        include(ttempcreatenode(templist[i]).tempinfo^.flags,ti_const);
                        include(ttempcreatenode(templist[i]).tempinfo^.flags,ti_may_be_in_reg);

=======
                        templist[i]:=ctempcreatenode.create_value(def,def.size,tt_persistent,
                          def.is_intregable or def.is_fpuregable,tnode(lists.nodelist[i]));
>>>>>>> graemeg/cpstrnew
=======
                        templist[i]:=ctempcreatenode.create_value(def,def.size,tt_persistent,
                          def.is_intregable or def.is_fpuregable,tnode(lists.nodelist[i]));
>>>>>>> graemeg/cpstrnew
=======
                        templist[i]:=ctempcreatenode.create_value(def,def.size,tt_persistent,
                          def.is_intregable or def.is_fpuregable,tnode(lists.nodelist[i]));
>>>>>>> graemeg/cpstrnew
=======
                        templist[i]:=ctempcreatenode.create_value(def,def.size,tt_persistent,
                          def.is_intregable or def.is_fpuregable,tnode(lists.nodelist[i]));
>>>>>>> origin/cpstrnew
                        { make debugging easier and set temp. location to the original location }
                        tnode(templist[i]).fileinfo:=tnode(lists.nodelist[i]).fileinfo;

                        addstatement(creates,tnode(templist[i]));
                        { make debugging easier and set temp. location to the original location }
                        creates.fileinfo:=tnode(lists.nodelist[i]).fileinfo;

                        hp:=ttempcreatenode(templist[i]);
                        do_firstpass(tnode(hp));
                        templist[i]:=hp;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        if addrstored then
                          pnode(lists.locationlist[i])^:=cderefnode.Create(ctemprefnode.create(ttempcreatenode(templist[i])))
                        else
                          pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[i]));
=======
                        pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[i]));
>>>>>>> graemeg/cpstrnew
=======
                        pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[i]));
>>>>>>> graemeg/cpstrnew
=======
                        pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[i]));
>>>>>>> graemeg/cpstrnew
=======
                        pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[i]));
>>>>>>> origin/cpstrnew
                        { make debugging easier and set temp. location to the original location }
                        pnode(lists.locationlist[i])^.fileinfo:=tnode(lists.nodelist[i]).fileinfo;

                        do_firstpass(pnode(lists.locationlist[i])^);
{$ifdef csedebug}
                        printnode(output,statements);
{$endif csedebug}
                      end
                    { current node reference to another node? }
                    else if lists.equalto[i]<>pointer(-1) then
                      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        def:=tstoreddef(tnode(lists.nodelist[i]).resultdef);
                        { we cannot handle register stored records or array in CSE yet
                          but we can store their reference }
                        addrstored:=((def.typ in [arraydef,recorddef]) or is_object(def)) and not(is_dynamic_array(def));

{$if defined(csedebug) or defined(csestats)}
                        writeln;
                        writeln('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
                        writeln('Complexity: ',node_complexity(tnode(lists.nodelist[i])),'  Node ',i,' equals Node ',ptrint(lists.equalto[i]));
                        printnode(output,tnode(lists.nodelist[i]));
                        printnode(output,tnode(lists.nodelist[ptrint(lists.equalto[i])]));
                        writeln('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
                        writeln;
{$endif defined(csedebug) or defined(csestats)}
                        templist[i]:=templist[ptrint(lists.equalto[i])];
                        if addrstored then
                          pnode(lists.locationlist[i])^:=cderefnode.Create(ctemprefnode.create(ttempcreatenode(templist[ptrint(lists.equalto[i])])))
                        else
                          pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[ptrint(lists.equalto[i])]));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$if defined(csedebug) or defined(csestats)}
                        printnode(output,tnode(lists.nodelist[i]));
                        writeln(i,'    equals   ',ptrint(lists.equalto[i]));
                        printnode(output,tnode(lists.nodelist[ptrint(lists.equalto[i])]));
{$endif defined(csedebug) or defined(csestats)}
                        templist[i]:=templist[ptrint(lists.equalto[i])];
                        pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[ptrint(lists.equalto[i])]));
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

                        { make debugging easier and set temp. location to the original location }
                        pnode(lists.locationlist[i])^.fileinfo:=tnode(lists.nodelist[i]).fileinfo;

                        do_firstpass(pnode(lists.locationlist[i])^);
                      end;
                  end;
                { clean up unused trees }
                for i:=0 to lists.nodelist.count-1 do
                  if lists.equalto[i]<>pointer(-1) then
                    tnode(lists.nodelist[i]).free;
{$ifdef csedebug}
                writeln('nodes: ',lists.nodelist.count);
                writeln('==========================================');
{$endif csedebug}
                lists.nodelist.free;
                lists.locationlist.free;
                lists.equalto.free;
                lists.refs.free;
                templist.free;

                if assigned(statements) then
                  begin
                    { call para nodes need a special handling because
                      they can be only children nodes of call nodes
                      so the initialization code is inserted below the
                      call para node
                    }
                    if n.nodetype=callparan then
                      begin
                        addstatement(statements,tcallparanode(n).left);
                        tcallparanode(n).left:=nodes;
                        do_firstpass(tcallparanode(n).left);
                      end
                    else
                      begin
                        addstatement(statements,n);
                        n:=nodes;
                        do_firstpass(n);
                      end;
{$ifdef csedebug}
                    printnode(output,nodes);
{$endif csedebug}
                  end;
              end
          end;
      end;


    function do_optcse(var rootnode : tnode) : tnode;
      begin
{$ifdef csedebug}
         writeln('====================================================================================');
         writeln('CSE optimization pass started');
         writeln('====================================================================================');
         printnode(rootnode);
         writeln('====================================================================================');
         writeln;
{$endif csedebug}
        foreachnodestatic(pm_postprocess,rootnode,@searchcsedomain,nil);
        result:=nil;
=======
>>>>>>> origin/fixes_2_2
      end;

end.
