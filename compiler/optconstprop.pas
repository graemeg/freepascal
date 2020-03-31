{
    Constant propagation across statements

    Copyright (c) 2005-2012 by Jeppe Johansen and Florian Klaempfl

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
unit optconstprop;

{$i fpcdefs.inc}

{ $define DEBUG_CONSTPROP}

  interface

    uses
      node;

    { does constant propagation for rootnode

      The approach is simple: It search for constant assignment statements. As soon as such
      a statement is found, the following statements are search if they contain
      a usage of the assigned variable. If this is a the case, the variable is
      replaced by the constant. This does not work across points where the
      program flow joins so e.g.

      if ... then
        ...
        a:=1;
        ...
      else
        ...
        a:=1;
        ...
      writeln(a);

      will not result in any constant propagation.
    }
    function do_optconstpropagate(var rootnode : tnode) : tnode;

  implementation

    uses
      pass_1,procinfo,compinnr,
      symsym, symconst,
      nutils, nbas, ncnv, nld, nflw, ncal, ninl;

    function check_written(var n: tnode; arg: pointer): foreachnoderesult;
      begin
        result:=fen_false;

        if n.isequal(tnode(arg)) and
           ((n.flags*[nf_write,nf_modify])<>[]) then
          begin
            result:=fen_norecurse_true;
          end;
      end;


    { propagates the constant assignment passed in arg into n }
    function replaceBasicAssign(var n: tnode; arg: tnode; var tree_modified: boolean): boolean;
      var
        st2, oldnode: tnode;
        old: pnode;
        changed, tree_modified2, tree_modified3: boolean;
        written, tree_modified4, tree_modified5: Boolean;
      begin
        result:=true;

        if n = nil then
          exit;

        tree_modified:=false;
        tree_modified2:=false;
        tree_modified3:=false;
        tree_modified4:=false;
        tree_modified5:=false;

        { while it might be usefull, to use foreach to iterate all nodes, it is safer to
          iterate manually here so we have full controll how all nodes are processed }

        { We cannot analyze beyond those nodes, so we terminate to be on the safe side }
        if (n.nodetype in [addrn,derefn,asmn,casen,whilerepeatn,labeln,continuen,breakn,
                           tryexceptn,raisen,tryfinallyn,onn,loadparentfpn,loadvmtaddrn,guidconstn,rttin,addoptn,asn,goton,
                           objcselectorn,objcprotocoln,
                           arrayconstructorn,arrayconstructorrangen]) then
          exit(false)
        else if n.nodetype=assignn then
          begin
            tree_modified:=false;

            { we can propage the constant in both branches because the evaluation order is not defined }
            result:=replaceBasicAssign(tassignmentnode(n).right, arg, tree_modified);
            { do not use the intuitive way result:=result and replace... because this would prevent
              replaceBasicAssign being called if the result is already false }
            result:=replaceBasicAssign(tassignmentnode(n).left, arg, tree_modified2) and result;
            tree_modified:=tree_modified or tree_modified2;

            { but we have to check if left writes to the currently searched variable ... }
            written:=foreachnodestatic(pm_postprocess, tassignmentnode(n).left, @check_written, tassignmentnode(arg).left);
            { ... if this is the case, we have to stop searching }
            result:=result and not(written);
          end
        else if n.isequal(tassignmentnode(arg).left) and ((n.flags*[nf_write,nf_modify])=[]) then
          begin
            n.Free;
            n:=tassignmentnode(arg).right.getcopy;
            inserttypeconv_internal(n, tassignmentnode(arg).left.resultdef);

            tree_modified:=true;
          end
        else if n.nodetype=statementn then
          result:=replaceBasicAssign(tstatementnode(n).left, arg, tree_modified)
        else if n.nodetype=forn then
          begin
            result:=replaceBasicAssign(tfornode(n).right, arg, tree_modified);
            if result then
              replaceBasicAssign(tfornode(n).t1, arg, tree_modified2);
            tree_modified:=tree_modified or tree_modified2;
            { after a for node we cannot continue with our simple approach }
            result:=false;
          end
        else if n.nodetype=blockn then
          begin
            changed:=false;

            st2:=tstatementnode(tblocknode(n).statements);
            old:=@tblocknode(n).statements;
            while assigned(st2) do
              begin
                repeat
                  oldnode:=st2;

                  tree_modified2:=false;
                  if not replaceBasicAssign(st2, arg, tree_modified2) then
                    begin
                      old^:=st2;
                      oldnode:=nil;
                      changed:=changed or tree_modified2;
                      result:=false;
                      break;
                    end
                  else
                    old^:=st2;
                  changed:=changed or tree_modified2;
                until oldnode=st2;

                if oldnode = nil then
                  break;

                old:=@tstatementnode(st2).next;
                st2:=tstatementnode(st2).next;
              end;

            tree_modified:=changed;
          end
        else if n.nodetype=ifn then
          begin
            result:=replaceBasicAssign(tifnode(n).left, arg, tree_modified);

            if result then
              begin
                if assigned(tifnode(n).t1) then
                  begin
                    { we can propagate the constant in both branches of an if statement
                      because even if the the branch writes to it, the else branch gets the
                      unmodified value }
                    result:=replaceBasicAssign(tifnode(n).right, arg, tree_modified2);

                    { do not use the intuitive way result:=result and replace... because this would prevent
                      replaceBasicAssign being called if the result is already false }
                    result:=replaceBasicAssign(tifnode(n).t1, arg, tree_modified3) and result;

                    tree_modified:=tree_modified or tree_modified2 or tree_modified3;
                  end
                else
                  begin
                    result:=replaceBasicAssign(tifnode(n).right, arg, tree_modified2);
                    tree_modified:=tree_modified or tree_modified2;
                  end;
              end;
          end
        else if n.nodetype=inlinen then
          begin
            { constant inc'ed/dec'ed? }
            if (tinlinenode(n).inlinenumber=in_dec_x) or (tinlinenode(n).inlinenumber=in_inc_x) then
              begin
                if tnode(tassignmentnode(arg).left).isequal(tcallparanode(tinlinenode(n).left).left) and
                   (not(assigned(tcallparanode(tinlinenode(n).left).right)) or
                       (tcallparanode(tcallparanode(tinlinenode(n).left).right).left.nodetype=ordconstn)) then
                  begin
                    { if the node just being searched is inc'ed/dec'ed then replace the inc/dec
                      by add/sub and force a second replacement pass }
                    oldnode:=n;
                    n:=tinlinenode(n).getaddsub_for_incdec;
                    oldnode.free;
                    tree_modified:=true;
                    { do not continue, value changed, if further const. propagations are possible, this is done
                      by the next pass }
                    result:=false;
                    exit;
                  end;
              end
            else if might_have_sideeffects(n) then
              exit(false);

            replaceBasicAssign(tunarynode(n).left, arg, tree_modified);
            result:=false;
          end
        else if n.nodetype=calln then
          begin
            { only propagate simply variables which are regable, this means that their address is not
              taken }
            if (tassignmentnode(arg).left.nodetype=loadn) and
              (tabstractvarsym(tloadnode(tassignmentnode(arg).left).symtableentry).varregable in [vr_fpureg,vr_mmreg,vr_intreg]) then
              begin
                result:=replaceBasicAssign(tnode(tcallnode(n).callinitblock), arg, tree_modified);
                result:=result and replaceBasicAssign(tcallnode(n).left, arg, tree_modified2);
                result:=result and replaceBasicAssign(tcallnode(n).vmt_entry, arg, tree_modified3);
                result:=result and replaceBasicAssign(tcallnode(n).right, arg, tree_modified4);
                result:=result and replaceBasicAssign(tnode(tcallnode(n).callcleanupblock), arg, tree_modified5);
                tree_modified:=tree_modified or tree_modified2 or tree_modified3 or tree_modified4 or tree_modified5;
              end
            else
              result:=false;
            exit;
          end
        else if n.InheritsFrom(tbinarynode) then
          begin
            result:=replaceBasicAssign(tbinarynode(n).left, arg, tree_modified);
            if result then
              result:=replaceBasicAssign(tbinarynode(n).right, arg, tree_modified2);
            tree_modified:=tree_modified or tree_modified2;
          end
        else if n.InheritsFrom(tunarynode) then
          begin
            result:=replaceBasicAssign(tunarynode(n).left, arg, tree_modified);
          end;

        if n.nodetype<>callparan then
          begin
            if tree_modified then
              exclude(n.flags,nf_pass1_done);

            do_firstpass(n);
          end;
      end;


    function propagate(var n: tnode; arg: pointer): foreachnoderesult;
      var
        l,
        st, st2, oldnode: tnode;
        old: pnode;
        a: tassignmentnode;
        tree_mod, changed: boolean;
      begin
        result:=fen_true;

        changed:=false;
        PBoolean(arg)^:=false;

        if not assigned(n) then
          exit;

        if n.nodetype in [calln] then
          exit(fen_norecurse_true);

        if n.nodetype=blockn then
          begin
            st:=tblocknode(n).statements;

            while assigned(st) and
                  (st.nodetype=statementn) and
                  assigned(tstatementnode(st).statement) do
              begin
                if tstatementnode(st).statement.nodetype=assignn then
                  begin
                    a:=tassignmentnode(tstatementnode(st).statement);

                    l:=a.left;

                    if ((((l.nodetype=loadn) and
                         { its address cannot have escaped the current routine }
                         not(tabstractvarsym(tloadnode(l).symtableentry).addr_taken)) and
                         ((
                           (tloadnode(l).symtableentry.typ=localvarsym) and
                           (tloadnode(l).symtable=current_procinfo.procdef.localst)
                          ) or
                          ((tloadnode(l).symtableentry.typ=paravarsym) and
                           (tloadnode(l).symtable=current_procinfo.procdef.parast)
                          ) or
                          ((tloadnode(l).symtableentry.typ=staticvarsym) and
                           (tloadnode(l).symtable.symtabletype=staticsymtable)
                          )
                         )) or
                        (l.nodetype = temprefn)) and
                       (is_constintnode(a.right) or
                        is_constboolnode(a.right) or
                        is_constcharnode(a.right) or
                        is_constenumnode(a.right) or
                        is_conststringnode(a.right)) then
                      begin
                        st2:=tstatementnode(tstatementnode(st).right);
                        old:=@tstatementnode(st).right;
                        while assigned(st2) do
                          begin
                            repeat
                              oldnode:=st2;

                              { Simple assignment of constant found }
                              tree_mod:=false;
                              if not replaceBasicAssign(st2, a, tree_mod) then
                                begin
                                  old^:=st2;
                                  oldnode:=nil;
                                  changed:=changed or tree_mod;
                                  break;
                                end
                              else
                                old^:=st2;
                              changed:=changed or tree_mod;
                            until oldnode=st2;

                            if oldnode = nil then
                              break;

                            old:=@tstatementnode(st2).next;
                            st2:=tstatementnode(st2).next;
                          end;
                      end;
                  end;

                st:=tstatementnode(st).next;
              end;
          end;

        PBoolean(arg)^:=changed;
      end;


    function do_optconstpropagate(var rootnode: tnode): tnode;
      var
        changed: boolean;
        runsimplify : Boolean;
      begin
{$ifdef DEBUG_CONSTPROP}
        writeln('************************ before constant propagation ***************************');
        printnode(rootnode);
{$endif DEBUG_CONSTPROP}
        runsimplify:=false;
        repeat
          changed:=false;
          foreachnodestatic(pm_postandagain, rootnode, @propagate, @changed);
          runsimplify:=runsimplify or changed;
        until changed=false;
        if runsimplify then
          doinlinesimplify(rootnode);
{$ifdef DEBUG_CONSTPROP}
        writeln('************************ after constant propagation ***************************');
        printnode(rootnode);
        writeln('*******************************************************************************');
{$endif DEBUG_CONSTPROP}
        result:=rootnode;
      end;

end.

