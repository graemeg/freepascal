{
    DFA

    Copyright (c) 2007 by Florian Klaempfl

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

{ $define DEBUG_DFA}
{ $define EXTDEBUG_DFA}

{ this unit implements routines to perform dfa }
unit optdfa;

{$i fpcdefs.inc}

  interface

    uses
      node,optutils;

    type
      TDFABuilder = class
      protected
        procedure CreateLifeInfo(node : tnode;map : TIndexedNodeSet);
      public
        resultnode : tnode;
        nodemap : TIndexedNodeSet;
        { reset all dfa info, this is required before creating dfa info
          if the tree has been changed without updating dfa }
        procedure resetdfainfo(node : tnode);

        procedure createdfainfo(node : tnode);
        destructor destroy;override;
      end;

    procedure CheckAndWarn(code : tnode;nodetosearch : tnode);

  implementation

    uses
      globtype,constexp,
      verbose,
      symconst,symdef,symsym,
      defutil,
      procinfo,
      nutils,htypechk,
      nbas,nflw,ncal,nset,nld,nadd,
      optbase;


    (*
    function initnodes(var n:tnode; arg: pointer) : foreachnoderesult;
      begin
        { node worth to add? }
        if (node_complexity(n)>1) and (tstoreddef(n.resultdef).is_intregable or tstoreddef(n.resultdef).is_fpuregable) then
          begin
            plists(arg)^.nodelist.Add(n);
            plists(arg)^.locationlist.Add(@n);
            result:=fen_false;
          end
        else
          result:=fen_norecurse_false;
      end;
    *)

    {
      x:=f;         read: [f]

      while x do    read: []

        a:=b;       read: [a,b,d]  def: [a]       life:  read*def=[a]
          c:=d;     read: [a,d]    def: [a,c]     life:  read*def=[a]
            e:=a;   read: [a]      def: [a,c,e]   life:  read*def=[a]


      function f(b,d,x : type) : type;

        begin
          while x do        alive: b,d,x
            begin
              a:=b;         alive: b,d,x
              c:=d;         alive: a,d,x
              e:=a+c;       alive: a,c,x
              dec(x);       alive: c,e,x
            end;
          result:=c+e;      alive: c,e
        end;                alive: result

    }

    type
      tdfainfo = record
        use : PDFASet;
        def : PDFASet;
        map : TIndexedNodeSet
      end;
      pdfainfo = ^tdfainfo;

    function AddDefUse(var n: tnode; arg: pointer): foreachnoderesult;
      begin
        case n.nodetype of
          tempcreaten:
            begin
              if assigned(ttempcreatenode(n).tempinfo^.tempinitcode) then
                begin
                  pdfainfo(arg)^.map.Add(n);
                  DFASetInclude(pdfainfo(arg)^.def^,n.optinfo^.index);
                end;
            end;
          temprefn,
          loadn:
            begin
              pdfainfo(arg)^.map.Add(n);
              if nf_modify in n.flags then
                begin
                  DFASetInclude(pdfainfo(arg)^.use^,n.optinfo^.index);
                  DFASetInclude(pdfainfo(arg)^.def^,n.optinfo^.index)
                end
              else if nf_write in n.flags then
                DFASetInclude(pdfainfo(arg)^.def^,n.optinfo^.index)
              else
                DFASetInclude(pdfainfo(arg)^.use^,n.optinfo^.index);
            end;
          else
            ;
        end;
        result:=fen_false;
      end;


    function ResetProcessing(var n: tnode; arg: pointer): foreachnoderesult;
      begin
        exclude(n.flags,nf_processing);
        { dfa works only on normalized trees, so do not recurse into expressions, because
          ResetProcessing eats a signififcant amount of time of CheckAndWarn

          the following set contains (hopefully) most of the expression nodes }
        if n.nodetype in [calln,inlinen,assignn,callparan,andn,addn,orn,subn,muln,divn,slashn,notn,equaln,unequaln,gtn,ltn,lten,gten,loadn,
          typeconvn,vecn,subscriptn,addrn,derefn] then
          result:=fen_norecurse_false
        else
          result:=fen_false;
      end;


    function ResetDFA(var n: tnode; arg: pointer): foreachnoderesult;
      begin
        if assigned(n.optinfo) then
          begin
            with n.optinfo^ do
              begin
                life:=nil;
                def:=nil;
                use:=nil;
                defsum:=nil;
              end;
          end;
        result:=fen_false;
      end;


    procedure TDFABuilder.CreateLifeInfo(node : tnode;map : TIndexedNodeSet);

      var
        changed : boolean;

      procedure CreateInfo(node : tnode);

        { update life entry of a node with l, set changed if this changes
          life info for the node
        }
        procedure updatelifeinfo(n : tnode;l : TDFASet);
          var
            b : boolean;
          begin
            b:=DFASetNotEqual(l,n.optinfo^.life);
{$ifdef DEBUG_DFA}
            if not(changed) and b then
              begin
                writeln('Another DFA pass caused by: ',nodetype2str[n.nodetype],'(',n.fileinfo.line,',',n.fileinfo.column,')');
                write('  Life info set was:     ');PrintDFASet(Output,n.optinfo^.life);writeln;
                write('  Life info set will be: ');PrintDFASet(Output,l);writeln;
              end;
{$endif DEBUG_DFA}

            changed:=changed or b;
            n.optinfo^.life:=l;
          end;

        procedure calclife(n : tnode);
          var
            l : TDFASet;
          begin
            if assigned(n.successor) then
              begin
                { ensure we can access optinfo }
                DFASetDiff(l,n.successor.optinfo^.life,n.optinfo^.def);
                DFASetIncludeSet(l,n.optinfo^.use);
                DFASetIncludeSet(l,n.optinfo^.life);
              end
            else
              begin
                l:=n.optinfo^.use;
                DFASetIncludeSet(l,n.optinfo^.life);
              end;
            updatelifeinfo(n,l);
          end;

        var
          dfainfo : tdfainfo;
          l : TDFASet;
          save: TDFASet;
          lv, hv: TConstExprInt;
          i : longint;
          counteruse_after_loop : boolean;
        begin
          if node=nil then
            exit;

          { ensure we've already optinfo set }
          node.allocoptinfo;

          if nf_processing in node.flags then
            exit;
          include(node.flags,nf_processing);

          if assigned(node.successor) then
            CreateInfo(node.successor);

{$ifdef EXTDEBUG_DFA}
          writeln('Handling: ',nodetype2str[node.nodetype],'(',node.fileinfo.line,',',node.fileinfo.column,')');
{$endif EXTDEBUG_DFA}
          { life:=succesorlive-definition+use }

          case node.nodetype of
            whilerepeatn:
              begin
                { analyze the loop condition }
                if not(assigned(node.optinfo^.def)) and
                   not(assigned(node.optinfo^.use)) then
                  begin
                    dfainfo.use:=@node.optinfo^.use;
                    dfainfo.def:=@node.optinfo^.def;
                    dfainfo.map:=map;
                    foreachnodestatic(pm_postprocess,twhilerepeatnode(node).left,@AddDefUse,@dfainfo);
                  end;

                { NB: this node should typically have empty def set }
                if assigned(node.successor) then
                  DFASetDiff(l,node.successor.optinfo^.life,node.optinfo^.def)
                else if assigned(resultnode) then
                  DFASetDiff(l,resultnode.optinfo^.life,node.optinfo^.def)
                else
                  l:=nil;

                { for repeat..until, node use set in included at the end of loop }
                if not (lnf_testatbegin in twhilerepeatnode(node).loopflags) then
                  DFASetIncludeSet(l,node.optinfo^.use);

                DFASetIncludeSet(l,node.optinfo^.life);

                save:=node.optinfo^.life;
                { to process body correctly, we need life info in place (because
                  whilerepeatnode is successor of its body). }
                node.optinfo^.life:=l;

                { now process the body }
                CreateInfo(twhilerepeatnode(node).right);

                { restore, to prevent infinite recursion via changed flag }
                node.optinfo^.life:=save;

                { for while loops, node use set is included at the beginning of loop }
                l:=twhilerepeatnode(node).right.optinfo^.life;
                if lnf_testatbegin in twhilerepeatnode(node).loopflags then
                  DFASetIncludeSet(l,node.optinfo^.use);

                UpdateLifeInfo(node,l);

                { ... and a second iteration for fast convergence }
                CreateInfo(twhilerepeatnode(node).right);
              end;

            forn:
              begin
                {
                  left: loopvar
                  right: from
                  t1: to
                  t2: body
                }
                node.allocoptinfo;
                tfornode(node).loopiteration.allocoptinfo;
                if not(assigned(node.optinfo^.def)) and
                   not(assigned(node.optinfo^.use)) then
                  begin
                    dfainfo.use:=@node.optinfo^.use;
                    dfainfo.def:=@node.optinfo^.def;
                    dfainfo.map:=map;
                    foreachnodestatic(pm_postprocess,tfornode(node).left,@AddDefUse,@dfainfo);
                    foreachnodestatic(pm_postprocess,tfornode(node).right,@AddDefUse,@dfainfo);
                    foreachnodestatic(pm_postprocess,tfornode(node).t1,@AddDefUse,@dfainfo);
                  end;

                { create life for the body }
                CreateInfo(tfornode(node).t2);

                { is the counter living after the loop?

                  if left is a record element, it might not be tracked by dfa, so
                  optinfo might not be assigned
                }
                counteruse_after_loop:=assigned(tfornode(node).left.optinfo) and assigned(node.successor) and
                  DFASetIn(node.successor.optinfo^.life,tfornode(node).left.optinfo^.index);

                if counteruse_after_loop then
                  begin
                    { if yes, then we should warn }
                    { !!!!!! }
                  end
                else
                  Include(tfornode(node).loopflags,lnf_dont_mind_loopvar_on_exit);

                { first update the dummy node }

                { get the life of the loop block }
                l:=copy(tfornode(node).t2.optinfo^.life);

                { take care of the sucessor }
                if assigned(node.successor) then
                  DFASetIncludeSet(l,node.successor.optinfo^.life);

                { the counter variable is living as well inside the for loop

                  if left is a record element, it might not be tracked by dfa, so
                  optinfo might not be assigned
                }
                if assigned(tfornode(node).left.optinfo) then
                  DFASetInclude(l,tfornode(node).left.optinfo^.index);

                { force block node life info }
                UpdateLifeInfo(tfornode(node).loopiteration,l);

                { now update the for node itself }

                { get the life of the loop block }
                l:=copy(tfornode(node).t2.optinfo^.life);

                { take care of the sucessor as it's possible that we don't have one execution of the body }
                if (not(tfornode(node).right.nodetype=ordconstn) or not(tfornode(node).t1.nodetype=ordconstn)) and
                  assigned(node.successor) then
                  DFASetIncludeSet(l,node.successor.optinfo^.life);

                {
                  the counter variable is not living at the entry of the for node

                  if left is a record element, it might not be tracked by dfa, so
                    optinfo might not be assigned
                }
                if assigned(tfornode(node).left.optinfo) then
                  DFASetExclude(l,tfornode(node).left.optinfo^.index);

                { ... but it could be that left/right use it, so do this after
                  removing the def of the counter variable }
                DFASetIncludeSet(l,node.optinfo^.use);

                UpdateLifeInfo(node,l);

                { ... and a second iteration for fast convergence }
                CreateInfo(tfornode(node).t2);
              end;

            temprefn,
            loadn,
            typeconvn,
            derefn,
            assignn:
              begin
                if not(assigned(node.optinfo^.def)) and
                  not(assigned(node.optinfo^.use)) then
                  begin
                    dfainfo.use:=@node.optinfo^.use;
                    dfainfo.def:=@node.optinfo^.def;
                    dfainfo.map:=map;
                    foreachnodestatic(pm_postprocess,node,@AddDefUse,@dfainfo);
                  end;
                calclife(node);
              end;

            statementn:
              begin
                { nested statement }
                CreateInfo(tstatementnode(node).statement);
                { inherit info }
                node.optinfo^.life:=tstatementnode(node).statement.optinfo^.life;
              end;

            blockn:
              begin
                CreateInfo(tblocknode(node).statements);
                { ensure that we don't remove life info }
                l:=node.optinfo^.life;
                if assigned(node.successor) then
                  DFASetIncludeSet(l,node.successor.optinfo^.life);
                UpdateLifeInfo(node,l);
              end;

            ifn:
              begin
                { get information from cond. expression }
                if not(assigned(node.optinfo^.def)) and
                   not(assigned(node.optinfo^.use)) then
                  begin
                    dfainfo.use:=@node.optinfo^.use;
                    dfainfo.def:=@node.optinfo^.def;
                    dfainfo.map:=map;
                    foreachnodestatic(pm_postprocess,tifnode(node).left,@AddDefUse,@dfainfo);
                  end;

                { create life info for then and else node }
                CreateInfo(tifnode(node).right);
                CreateInfo(tifnode(node).t1);

                { ensure that we don't remove life info }
                l:=node.optinfo^.life;

                { get life info from then branch }
                if assigned(tifnode(node).right) then
                  DFASetIncludeSet(l,tifnode(node).right.optinfo^.life);

                { get life info from else branch }
                if assigned(tifnode(node).t1) then
                  DFASetIncludeSet(l,tifnode(node).t1.optinfo^.life)
                else if assigned(node.successor) then
                  DFASetIncludeSet(l,node.successor.optinfo^.life);

                { remove def info from the cond. expression }
                DFASetExcludeSet(l,tifnode(node).optinfo^.def);

                { add use info from the cond. expression }
                DFASetIncludeSet(l,tifnode(node).optinfo^.use);

                { finally, update the life info of the node }
                UpdateLifeInfo(node,l);
              end;

            casen:
              begin
                { get information from "case" expression }
                if not(assigned(node.optinfo^.def)) and
                   not(assigned(node.optinfo^.use)) then
                  begin
                    dfainfo.use:=@node.optinfo^.use;
                    dfainfo.def:=@node.optinfo^.def;
                    dfainfo.map:=map;
                    foreachnodestatic(pm_postprocess,tcasenode(node).left,@AddDefUse,@dfainfo);
                  end;

                { create life info for block and else nodes }
                for i:=0 to tcasenode(node).blocks.count-1 do
                  CreateInfo(pcaseblock(tcasenode(node).blocks[i])^.statement);

                CreateInfo(tcasenode(node).elseblock);

                { ensure that we don't remove life info }
                l:=node.optinfo^.life;

                { get life info from case branches }
                for i:=0 to tcasenode(node).blocks.count-1 do
                  DFASetIncludeSet(l,pcaseblock(tcasenode(node).blocks[i])^.statement.optinfo^.life);

                { get life info from else branch or the succesor }
                if assigned(tcasenode(node).elseblock) then
                  DFASetIncludeSet(l,tcasenode(node).elseblock.optinfo^.life)
                else if assigned(node.successor) then
                  begin
                    if is_ordinal(tcasenode(node).left.resultdef) then
                      begin
                        getrange(tcasenode(node).left.resultdef,lv,hv);
                        if tcasenode(node).labelcoverage<(hv-lv) then
                          DFASetIncludeSet(l,node.successor.optinfo^.life);
                      end
                    else
                      DFASetIncludeSet(l,node.successor.optinfo^.life);
                  end;

                { add use info from the "case" expression }
                DFASetIncludeSet(l,tcasenode(node).optinfo^.use);

                { finally, update the life info of the node }
                UpdateLifeInfo(node,l);
              end;

            exitn:
              begin
                if not(is_void(current_procinfo.procdef.returndef)) then
                  begin
                    if not(assigned(node.optinfo^.def)) and
                       not(assigned(node.optinfo^.use)) then
                      begin
                        if assigned(texitnode(node).left) then
                          begin
                            node.optinfo^.def:=resultnode.optinfo^.def;

                            dfainfo.use:=@node.optinfo^.use;
                            dfainfo.def:=@node.optinfo^.def;
                            dfainfo.map:=map;
                            foreachnodestatic(pm_postprocess,texitnode(node).left,@AddDefUse,@dfainfo);
                            calclife(node);
                          end
                        else
                          begin
                            { get info from faked resultnode }
                            node.optinfo^.use:=resultnode.optinfo^.use;
                            node.optinfo^.life:=node.optinfo^.use;
                            changed:=true;
                          end;
                      end;
                  end;
              end;

{$ifdef JVM}
            { all other platforms except jvm translate raise nodes into call nodes during pass_1 }
            raisen,
{$endif JVM}
            asn,
            inlinen,
            calln:
              begin
                if not(assigned(node.optinfo^.def)) and
                  not(assigned(node.optinfo^.use)) then
                  begin
                    dfainfo.use:=@node.optinfo^.use;
                    dfainfo.def:=@node.optinfo^.def;
                    dfainfo.map:=map;
                    foreachnodestatic(pm_postprocess,node,@AddDefUse,@dfainfo);
                  end;
                calclife(node);
              end;

            labeln:
              begin
                calclife(node);

                if assigned(tlabelnode(node).left) then
                  begin
                    l:=node.optinfo^.life;
                    DFASetIncludeSet(l,tlabelnode(node).optinfo^.life);
                    UpdateLifeInfo(node,l);
                  end;
              end;
            tempcreaten,
            tempdeleten,
            nothingn,
            continuen,
            goton,
            breakn:
              begin
                calclife(node);
              end;
            else
              internalerror(2007050502);
          end;
        end;

      var
        runs : integer;
      begin
        runs:=0;
        repeat
          inc(runs);
          changed:=false;
          CreateInfo(node);
          foreachnodestatic(pm_postprocess,node,@ResetProcessing,nil);
          { the result node is not reached by foreachnodestatic }
          exclude(resultnode.flags,nf_processing);
{$ifdef DEBUG_DFA}
          PrintIndexedNodeSet(output,map);
          PrintDFAInfo(output,node);
{$endif DEBUG_DFA}
        until not(changed);
{$ifdef DEBUG_DFA}
        writeln('DFA solver iterations: ',runs);
{$endif DEBUG_DFA}
      end;


    { reset all dfa info, this is required before creating dfa info
      if the tree has been changed without updating dfa }
    procedure TDFABuilder.resetdfainfo(node : tnode);
      begin
        nodemap.Free;
        nodemap:=nil;
        resultnode.Free;
        resultnode:=nil;
        foreachnodestatic(pm_postprocess,node,@ResetDFA,nil);
      end;


    procedure TDFABuilder.createdfainfo(node : tnode);
      var
        dfarec : tdfainfo;
      begin
        if not(assigned(nodemap)) then
          nodemap:=TIndexedNodeSet.Create;

        { create a fake node using the result which will be the last node }
        if not(is_void(current_procinfo.procdef.returndef)) then
          begin
            if current_procinfo.procdef.proctypeoption=potype_constructor then
              resultnode:=load_self_node
            else
              resultnode:=load_result_node;
            resultnode.allocoptinfo;
            dfarec.use:=@resultnode.optinfo^.use;
            dfarec.def:=@resultnode.optinfo^.def;
            dfarec.map:=nodemap;
            AddDefUse(resultnode,@dfarec);
            resultnode.optinfo^.life:=resultnode.optinfo^.use;
          end
        else
          begin
            resultnode:=cnothingnode.create;
            resultnode.allocoptinfo;
          end;

        { add controll flow information }
        SetNodeSucessors(node,resultnode);

        { now, collect life information }
        CreateLifeInfo(node,nodemap);
      end;


    destructor TDFABuilder.Destroy;
      begin
        Resultnode.free;
        nodemap.free;
        inherited destroy;
      end;

    var
      { we have to pass the address of SearchNode in a call inside of SearchNode:
        @SearchNode does not work because the compiler thinks we take the address of the result
        so store the address from outside }
      SearchNodeProcPointer : function(var n: tnode; arg: pointer): foreachnoderesult;

    type
      { helper structure to be able to pass more than one variable to the iterator function }
      TSearchNodeInfo = record
        nodetosearch : tnode;
        { this contains a list of all file locations where a warning was thrown already,
          the same location might appear multiple times because nodes might have been copied }
        warnedfilelocs : array of tfileposinfo;
      end;

      PSearchNodeInfo = ^TSearchNodeInfo;

    { searches for a given node n and warns if the node is found as being uninitialized. If a node is
      found, searching is stopped so each call issues only one warning/hint }
    function SearchNode(var n: tnode; arg: pointer): foreachnoderesult;

      function WarnedForLocation(f : tfileposinfo) : boolean;
        var
          i : longint;
        begin
          result:=true;
          for i:=0 to high(PSearchNodeInfo(arg)^.warnedfilelocs) do
            with PSearchNodeInfo(arg)^.warnedfilelocs[i] do
              begin
                if (f.column=column) and (f.fileindex=fileindex) and (f.line=line) and (f.moduleindex=moduleindex) then
                  exit;
              end;
          result:=false;
        end;


      procedure AddFilepos(const f : tfileposinfo);
        begin
          Setlength(PSearchNodeInfo(arg)^.warnedfilelocs,length(PSearchNodeInfo(arg)^.warnedfilelocs)+1);
          PSearchNodeInfo(arg)^.warnedfilelocs[high(PSearchNodeInfo(arg)^.warnedfilelocs)]:=f;
        end;


      { Checks if the symbol is a candidate for a warning.
        Emit warning/note for living locals, result and parameters, but only about the current
        symtables }
      function SymbolCandidateForWarningOrHint(sym : tabstractnormalvarsym) : Boolean;
        begin
          Result:=(((sym.owner=current_procinfo.procdef.localst) and
                    (current_procinfo.procdef.localst.symtablelevel=sym.owner.symtablelevel)
                   ) or
                   ((sym.owner=current_procinfo.procdef.parast) and
                    (sym.typ=paravarsym) and
                    (current_procinfo.procdef.parast.symtablelevel=sym.owner.symtablelevel) and
                    { all parameters except out parameters are initialized by the caller }
                    (tparavarsym(sym).varspez=vs_out)
                   ) or
                   ((vo_is_funcret in sym.varoptions) and
                    (current_procinfo.procdef.parast.symtablelevel=sym.owner.symtablelevel)
                   )
                  ) and not(vo_is_external in sym.varoptions)
        end;

      var
        varsym : tabstractnormalvarsym;
        methodpointer,
        hpt : tnode;
      begin
        result:=fen_false;
        case n.nodetype of
          callparan:
            begin
              { do not warn about variables passed by var, just issue a hint, this
                is a workaround for old code e.g. using fillchar }
              if assigned(tcallparanode(n).parasym) and (tcallparanode(n).parasym.varspez in [vs_var,vs_out]) then
                begin
                  hpt:=tcallparanode(n).left;
                  while assigned(hpt) and (hpt.nodetype in [subscriptn,vecn,typeconvn]) do
                    hpt:=tunarynode(hpt).left;
                  if assigned(hpt) and (hpt.nodetype=loadn) and not(WarnedForLocation(hpt.fileinfo)) and
                    SymbolCandidateForWarningOrHint(tabstractnormalvarsym(tloadnode(hpt).symtableentry)) and
                    PSearchNodeInfo(arg)^.nodetosearch.isequal(hpt) then
                    begin
                      { issue only a hint for var, when encountering the node passed as out, we need only to stop searching }
                      if tcallparanode(n).parasym.varspez=vs_var then
                        UninitializedVariableMessage(hpt.fileinfo,false,
                          tloadnode(hpt).symtable.symtabletype=localsymtable,
                          is_managed_type(tloadnode(hpt).resultdef),
                          tloadnode(hpt).symtableentry.RealName);
                      AddFilepos(hpt.fileinfo);
                      result:=fen_norecurse_true;
                    end
                end;
            end;
          orn,
          andn:
            begin
              { take care of short boolean evaluation: if the expression to be search is found in left,
                we do not need to search right }
              if foreachnodestatic(pm_postprocess,taddnode(n).left,SearchNodeProcPointer,arg) or
                foreachnodestatic(pm_postprocess,taddnode(n).right,SearchNodeProcPointer,arg) then
                result:=fen_norecurse_true
              else
                result:=fen_norecurse_false;
            end;
          calln:
            begin
              methodpointer:=tcallnode(n).methodpointer;
              if assigned(methodpointer) and (methodpointer.nodetype<>typen) then
               begin
                  { Remove all postfix operators }
                  hpt:=methodpointer;
                  while assigned(hpt) and (hpt.nodetype in [subscriptn,vecn]) do
                    hpt:=tunarynode(hpt).left;

                 { skip (absolute and other simple) type conversions -- only now,
                   because the checks above have to take type conversions into
                   e.g. class reference types account }
                 hpt:=actualtargetnode(@hpt)^;

                  { R.Init then R will be initialized by the constructor,
                    Also allow it for simple loads }
                  if (tcallnode(n).procdefinition.proctypeoption=potype_constructor) or
                     (PSearchNodeInfo(arg)^.nodetosearch.isequal(hpt) and
                      (((methodpointer.resultdef.typ=objectdef) and
                        not(oo_has_virtual in tobjectdef(methodpointer.resultdef).objectoptions)) or
                       (methodpointer.resultdef.typ=recorddef)
                      )
                     ) then
                    begin
                      { don't warn about the method pointer }
                      AddFilepos(hpt.fileinfo);

                      if not(foreachnodestatic(pm_postprocess,tcallnode(n).left,SearchNodeProcPointer,arg)) then
                        foreachnodestatic(pm_postprocess,tcallnode(n).right,SearchNodeProcPointer,arg);
                      result:=fen_norecurse_true
                    end;
                 end;
            end;
          loadn:
            begin
              if (tloadnode(n).symtableentry.typ in [localvarsym,paravarsym,staticvarsym]) and
                PSearchNodeInfo(arg)^.nodetosearch.isequal(n) and ((nf_modify in n.flags) or not(nf_write in n.flags)) then
                begin
                  varsym:=tabstractnormalvarsym(tloadnode(n).symtableentry);

                  if assigned(varsym.owner) and SymbolCandidateForWarningOrHint(varsym) then
                    begin
                      if (vo_is_funcret in varsym.varoptions) and not(WarnedForLocation(n.fileinfo)) then
                        begin
                          if is_managed_type(varsym.vardef) then
                            MessagePos(n.fileinfo,sym_w_managed_function_result_uninitialized)
                          else
                            MessagePos(n.fileinfo,sym_w_function_result_uninitialized);
                          AddFilepos(n.fileinfo);
                          result:=fen_norecurse_true;
                        end
                      else
                        begin
                          { typed consts are initialized, further, warn only once per location }
                          if not (vo_is_typed_const in varsym.varoptions) and not(WarnedForLocation(n.fileinfo)) then
                            begin
                              UninitializedVariableMessage(n.fileinfo,true,varsym.typ=localvarsym,is_managed_type(varsym.vardef),varsym.realname);
                              AddFilepos(n.fileinfo);
                              result:=fen_norecurse_true;
                            end;
                        end;
                    end
{$ifdef dummy}
                  { if a the variable we are looking for is passed as a var parameter, we stop searching }
                  else if assigned(varsym.owner) and
                     (varsym.owner=current_procinfo.procdef.parast) and
                     (varsym.typ=paravarsym) and
                     (current_procinfo.procdef.parast.symtablelevel=varsym.owner.symtablelevel) and
                     (tparavarsym(varsym).varspez=vs_var) then
                    result:=fen_norecurse_true;
{$endif dummy}
                end;
            end;
          else
            ;
        end;
      end;


    procedure CheckAndWarn(code : tnode;nodetosearch : tnode);

      var
        SearchNodeInfo : TSearchNodeInfo;

      function DoCheck(node : tnode) : boolean;
        var
          i : longint;
          touchesnode : Boolean;

        procedure MaybeDoCheck(n : tnode);inline;
          begin
            Result:=Result or DoCheck(n);
          end;

        procedure MaybeSearchIn(n : tnode);
          begin
            if touchesnode then
              Result:=Result or foreachnodestatic(pm_postprocess,n,@SearchNode,@SearchNodeInfo);
          end;

        begin
          result:=false;

          if node=nil then
            exit;

          if nf_processing in node.flags then
            exit;
          include(node.flags,nf_processing);

          if not(DFASetIn(node.optinfo^.life,nodetosearch.optinfo^.index)) then
            exit;

          { we do not need this info always, so try to safe some time here, CheckAndWarn
            takes a lot of time anyways }
          if not(node.nodetype in [statementn,blockn]) then
            touchesnode:=DFASetIn(node.optinfo^.use,nodetosearch.optinfo^.index) or
              DFASetIn(node.optinfo^.def,nodetosearch.optinfo^.index)
          else
            touchesnode:=false;

          case node.nodetype of
            whilerepeatn:
              begin
                MaybeSearchIn(twhilerepeatnode(node).left);
                MaybeDoCheck(twhilerepeatnode(node).right);
              end;

            forn:
              begin
                MaybeSearchIn(tfornode(node).right);
                MaybeSearchIn(tfornode(node).t1);
                MaybeDoCheck(tfornode(node).t2);
              end;

            statementn:
              MaybeDoCheck(tstatementnode(node).statement);

            blockn:
              MaybeDoCheck(tblocknode(node).statements);

            ifn:
              begin
                MaybeSearchIn(tifnode(node).left);
                MaybeDoCheck(tifnode(node).right);
                MaybeDoCheck(tifnode(node).t1);
              end;

            casen:
              begin
                MaybeSearchIn(tcasenode(node).left);
                for i:=0 to tcasenode(node).blocks.count-1 do
                  MaybeDoCheck(pcaseblock(tcasenode(node).blocks[i])^.statement);

                MaybeDoCheck(tcasenode(node).elseblock);
              end;

            labeln:
              MaybeDoCheck(tlabelnode(node).left);

            { we are aware of the following nodes so if new node types are added to the compiler
              and pop up in the search, the ie below kicks in as a reminder }
            exitn:
              begin
                MaybeSearchIn(texitnode(node).left);
                { exit uses the resultnode implicitly, so searching for a matching node is
                  useless, if we reach the exit node and found the living node not in left, then
                  it can be only the resultnode  }
                if not(Result) and not(is_void(current_procinfo.procdef.returndef)) and
                  not(assigned(texitnode(node).resultexpr)) and
                  { don't warn about constructors }
                  not(current_procinfo.procdef.proctypeoption in [potype_class_constructor,potype_constructor]) then
                  begin
                    if is_managed_type(current_procinfo.procdef.returndef) then
                      MessagePos(node.fileinfo,sym_w_managed_function_result_uninitialized)
                    else
                      MessagePos(node.fileinfo,sym_w_function_result_uninitialized);

                    Setlength(SearchNodeInfo.warnedfilelocs,length(SearchNodeInfo.warnedfilelocs)+1);
                    SearchNodeInfo.warnedfilelocs[high(SearchNodeInfo.warnedfilelocs)]:=node.fileinfo;
                  end
              end;
            { could be the implicitly generated load node for the result }
{$ifdef JVM}
            { all other platforms except jvm translate raise nodes into call nodes during pass_1 }
            raisen,
{$endif JVM}
            loadn,
            assignn,
            calln,
            temprefn,
            typeconvn,
            inlinen,
            tempcreaten,
            tempdeleten:
              MaybeSearchIn(node);
            nothingn,
            continuen,
            goton,
            breakn:
              ;
            else
              internalerror(2013111301);
          end;

          { if already a warning has been issued, then stop }
          if Result then
            exit;

          if assigned(node.successor) then
            MaybeDoCheck(node.successor);
        end;

      begin
        SearchNodeInfo.nodetosearch:=nodetosearch;
        DoCheck(code);
        foreachnodestatic(pm_postprocess,code,@ResetProcessing,nil);
      end;


begin
  SearchNodeProcPointer:=@SearchNode;
end.
