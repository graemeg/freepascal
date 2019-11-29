{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Does the parsing of the statements

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
unit pstatmnt;

{$i fpcdefs.inc}

interface

    uses
      tokens,node;


    function statement_block(starttoken : ttoken) : tnode;

    { reads an assembler block }
    function assembler_block : tnode;


implementation

    uses
       { common }
       cutils,cclasses,
       { global }
       globtype,globals,verbose,constexp,
       systems,
       { aasm }
       cpubase,aasmtai,aasmdata,
       { symtable }
       symconst,symbase,symtype,symdef,symsym,symtable,defutil,defcmp,
       paramgr,
       { pass 1 }
       pass_1,htypechk,
       nutils,ngenutil,nbas,ncal,nmem,nset,ncnv,ncon,nld,nflw,
       { parser }
       scanner,
       pbase,ptype,pexpr,
       { codegen }
       procinfo,cgbase,
       { assembler reader }
       rabase;


    function statement : tnode;forward;


    function if_statement : tnode;
      var
         ex,if_a,else_a : tnode;
      begin
         consume(_IF);
         ex:=comp_expr([ef_accept_equal]);
         consume(_THEN);
         if not(token in endtokens) then
           if_a:=statement
         else
           if_a:=nil;

         if try_to_consume(_ELSE) then
            else_a:=statement
         else
           else_a:=nil;
         result:=cifnode.create(ex,if_a,else_a);
      end;

    { creates a block (list) of statements, til the next END token }
    function statements_til_end : tnode;

      var
         first,last : tstatementnode;

      begin
         first:=nil;
         last:=nil;
         while token<>_END do
           begin
              if first=nil then
                begin
                   last:=cstatementnode.create(statement,nil);
                   first:=last;
                end
              else
                begin
                   last.right:=cstatementnode.create(statement,nil);
                   last:=tstatementnode(last.right);
                end;
              if not try_to_consume(_SEMICOLON) then
                break;
              consume_emptystats;
           end;
         consume(_END);
         statements_til_end:=cblocknode.create(first);
         if assigned(first) then
           statements_til_end.fileinfo:=first.fileinfo;
      end;


    function case_statement : tnode;
      var
         casedef : tdef;
         caseexpr,p : tnode;
         blockid : longint;
         hl1,hl2 : TConstExprInt;
         sl1,sl2 : tstringconstnode;
         casedeferror, caseofstring : boolean;
         casenode : tcasenode;
      begin
         consume(_CASE);
         caseexpr:=comp_expr([ef_accept_equal]);
         { determines result type }
         do_typecheckpass(caseexpr);
         { variants must be accepted, but first they must be converted to integer }
         if caseexpr.resultdef.typ=variantdef then
           begin
             caseexpr:=ctypeconvnode.create_internal(caseexpr,sinttype);
             do_typecheckpass(caseexpr);
           end;
         set_varstate(caseexpr,vs_read,[vsf_must_be_valid]);
         casedeferror:=false;
         casedef:=caseexpr.resultdef;
         { case of string must be rejected in delphi-, }
         { tp7/bp7-, mac-compatibility modes.          }
         caseofstring :=
           ([m_delphi, m_mac, m_tp7] * current_settings.modeswitches = []) and
           is_string(casedef);

         if (not assigned(casedef)) or
            ( not(is_ordinal(casedef)) and (not caseofstring) ) then
          begin
            CGMessage(type_e_ordinal_or_string_expr_expected);
            { create a correct tree }
            caseexpr.free;
            caseexpr:=cordconstnode.create(0,u32inttype,false);
            { set error flag so no rangechecks are done }
            casedeferror:=true;
          end;
         { Create casenode }
         casenode:=ccasenode.create(caseexpr);
         consume(_OF);
         { Parse all case blocks }
         blockid:=0;
         repeat
           { maybe an instruction has more case labels }
           repeat
             p:=expr(true);
             if is_widechar(casedef) then
               begin
                  if (p.nodetype=rangen) then
                    begin
                       trangenode(p).left:=ctypeconvnode.create(trangenode(p).left,cwidechartype);
                       trangenode(p).right:=ctypeconvnode.create(trangenode(p).right,cwidechartype);
                       do_typecheckpass(trangenode(p).left);
                       do_typecheckpass(trangenode(p).right);
                    end
                  else
                    begin
                       p:=ctypeconvnode.create(p,cwidechartype);
                       do_typecheckpass(p);
                    end;
               end
             else
               begin
                 if is_char(casedef) and is_widechar(p.resultdef) then
                   begin
                      if (p.nodetype=ordconstn) then
                        begin
                           p:=ctypeconvnode.create(p,cansichartype);
                           do_typecheckpass(p);
                        end
                      else if (p.nodetype=rangen) then
                        begin
                           trangenode(p).left:=ctypeconvnode.create(trangenode(p).left,cansichartype);
                           trangenode(p).right:=ctypeconvnode.create(trangenode(p).right,cansichartype);
                           do_typecheckpass(trangenode(p).left);
                           do_typecheckpass(trangenode(p).right);
                        end;
                   end;
               end;
             hl1:=0;
             hl2:=0;
             sl1:=nil;
             sl2:=nil;
             if (p.nodetype=rangen) then
               begin
                 { type check for string case statements }
                 if caseofstring and
                   is_conststring_or_constcharnode(trangenode(p).left) and
                   is_conststring_or_constcharnode(trangenode(p).right) then
                 begin
                   { we need stringconstnodes, even if expression contains single chars }
                   sl1 := get_string_value(trangenode(p).left, tstringdef(casedef));
                   sl2 := get_string_value(trangenode(p).right, tstringdef(casedef));
                   if sl1.fullcompare(sl2) > 0 then
                     CGMessage(parser_e_case_lower_less_than_upper_bound);
                 end
                 { type checking for ordinal case statements }
                 else if (not caseofstring) and
                   is_subequal(casedef, trangenode(p).left.resultdef) and
                   is_subequal(casedef, trangenode(p).right.resultdef) then
                   begin
                     hl1:=get_ordinal_value(trangenode(p).left);
                     hl2:=get_ordinal_value(trangenode(p).right);
                     if hl1>hl2 then
                       CGMessage(parser_e_case_lower_less_than_upper_bound);
                     if not casedeferror then
                       begin
                         adaptrange(casedef,hl1,false,false,cs_check_range in current_settings.localswitches);
                         adaptrange(casedef,hl2,false,false,cs_check_range in current_settings.localswitches);
                       end;
                   end
                 else
                   CGMessage(parser_e_case_mismatch);

                 if caseofstring then
                   casenode.addlabel(blockid,sl1,sl2)
                 else
                   casenode.addlabel(blockid,hl1,hl2);
               end
             else
               begin
                 { type check for string case statements }
                 if (caseofstring and (not is_conststring_or_constcharnode(p))) or
                 { type checking for ordinal case statements }
                   ((not caseofstring) and (not is_subequal(casedef, p.resultdef))) then
                   CGMessage(parser_e_case_mismatch);

                 if caseofstring then
                   begin
                     sl1:=get_string_value(p, tstringdef(casedef));
                     casenode.addlabel(blockid,sl1,sl1);
                   end
                 else
                   begin
                     hl1:=get_ordinal_value(p);
                     if not casedeferror then
                       adaptrange(casedef,hl1,false,false,cs_check_range in current_settings.localswitches);
                     casenode.addlabel(blockid,hl1,hl1);
                   end;
               end;
             p.free;
             sl1.free;
             sl2.free;

             if token=_COMMA then
               consume(_COMMA)
             else
               break;
           until false;
           consume(_COLON);

           { add instruction block }
           casenode.addblock(blockid,statement);

           { next block }
           inc(blockid);

           if not(token in [_ELSE,_OTHERWISE,_END]) then
             consume(_SEMICOLON);
         until (token in [_ELSE,_OTHERWISE,_END]);

         if (token in [_ELSE,_OTHERWISE]) then
           begin
              if not try_to_consume(_ELSE) then
                consume(_OTHERWISE);
              casenode.addelseblock(statements_til_end);
           end
         else
           consume(_END);

         result:=casenode;
      end;


    function repeat_statement : tnode;

      var
         first,last,p_e : tnode;

      begin
         consume(_REPEAT);

         first:=nil;
         last:=nil;
         while token<>_UNTIL do
           begin
              if first=nil then
                begin
                   last:=cstatementnode.create(statement,nil);
                   first:=last;
                end
              else
                begin
                   tstatementnode(last).right:=cstatementnode.create(statement,nil);
                   last:=tstatementnode(last).right;
                end;
              if not try_to_consume(_SEMICOLON) then
                break;
              consume_emptystats;
           end;
         consume(_UNTIL);

         first:=cblocknode.create(first);
         p_e:=comp_expr([ef_accept_equal]);
         result:=cwhilerepeatnode.create(p_e,first,false,true);
      end;


    function while_statement : tnode;

      var
         p_e,p_a : tnode;

      begin
         consume(_WHILE);
         p_e:=comp_expr([ef_accept_equal]);
         consume(_DO);
         p_a:=statement;
         result:=cwhilerepeatnode.create(p_e,p_a,true,false);
      end;

    { a helper function which is used both by "with" and "for-in loop" nodes }
    function skip_nodes_before_load(p: tnode): tnode;
      begin
        { ignore nodes that don't add instructions in the tree }
        while assigned(p) and
           { equal type conversions }
           (
            (p.nodetype=typeconvn) and
            (ttypeconvnode(p).convtype=tc_equal)
           ) or
           { constant array index }
           (
            (p.nodetype=vecn) and
            (tvecnode(p).right.nodetype=ordconstn)
           ) do
          p:=tunarynode(p).left;
        result:=p;
      end;

    function for_statement : tnode;

        procedure check_range(hp:tnode; fordef: tdef);
          begin
            if (hp.nodetype=ordconstn) and
               (fordef.typ<>errordef) then
              adaptrange(fordef,tordconstnode(hp).value,false,false,true);
          end;

        function for_loop_create(hloopvar: tnode): tnode;
          var
             hp,
             hblock,
             hto,hfrom : tnode;
             backward : boolean;
             loopvarsym : tabstractvarsym;
          begin
             { Check loop variable }
             loopvarsym:=nil;

             { variable must be an ordinal, int64 is not allowed for 32bit targets }
             if (
                 not(is_ordinal(hloopvar.resultdef))
    {$ifndef cpu64bitaddr}
                 or is_64bitint(hloopvar.resultdef)
    {$endif not cpu64bitaddr}
               ) and
               (hloopvar.resultdef.typ<>undefineddef)
               then
               MessagePos(hloopvar.fileinfo,type_e_ordinal_expr_expected);

             hp:=hloopvar;
             while assigned(hp) and
                   (
                    { record/object fields and array elements are allowed }
                    { in tp7 mode only                                    }
                    (
                     (m_tp7 in current_settings.modeswitches) and
                     (
                      ((hp.nodetype=subscriptn) and
                       ((tsubscriptnode(hp).left.resultdef.typ=recorddef) or
                        is_object(tsubscriptnode(hp).left.resultdef))
                      ) or
                      { constant array index }
                      (
                       (hp.nodetype=vecn) and
                       is_constintnode(tvecnode(hp).right)
                      )
                     )
                    ) or
                    { equal typeconversions }
                    (
                     (hp.nodetype=typeconvn) and
                     (ttypeconvnode(hp).convtype=tc_equal)
                    )
                   ) do
               begin
                 { Use the recordfield for loopvarsym }
                 if not assigned(loopvarsym) and
                    (hp.nodetype=subscriptn) then
                   loopvarsym:=tsubscriptnode(hp).vs;
                 hp:=tunarynode(hp).left;
               end;

             if assigned(hp) and
                (hp.nodetype=loadn) then
               begin
                 case tloadnode(hp).symtableentry.typ of
                   staticvarsym,
                   localvarsym,
                   paravarsym :
                     begin
                       { we need a simple loadn:
                           1. The load must be in a global symtable or
                               in the same level as the para of the current proc.
                           2. value variables (no const,out or var)
                           3. No threadvar, readonly or typedconst
                       }
                       if (
                           (tloadnode(hp).symtable.symtablelevel=main_program_level) or
                           (tloadnode(hp).symtable.symtablelevel=current_procinfo.procdef.parast.symtablelevel)
                          ) and
                          (tabstractvarsym(tloadnode(hp).symtableentry).varspez=vs_value) and
                          ([vo_is_thread_var,vo_is_typed_const] * tabstractvarsym(tloadnode(hp).symtableentry).varoptions=[]) then
                         begin
                           { Assigning for-loop variable is only allowed in tp7 and macpas }
                           if ([m_tp7,m_mac] * current_settings.modeswitches = []) then
                             begin
                               if not assigned(loopvarsym) then
                                 loopvarsym:=tabstractvarsym(tloadnode(hp).symtableentry);
                               include(loopvarsym.varoptions,vo_is_loop_counter);
                             end;
                         end
                       else
                         begin
                           { Typed const is allowed in tp7 }
                           if not(m_tp7 in current_settings.modeswitches) or
                              not(vo_is_typed_const in tabstractvarsym(tloadnode(hp).symtableentry).varoptions) then
                             MessagePos(hp.fileinfo,type_e_illegal_count_var);
                         end;
                     end;
                   else
                     MessagePos(hp.fileinfo,type_e_illegal_count_var);
                 end;
               end
             else
               MessagePos(hloopvar.fileinfo,type_e_illegal_count_var);

             hfrom:=comp_expr([ef_accept_equal]);

             if try_to_consume(_DOWNTO) then
               backward:=true
             else
               begin
                 consume(_TO);
                 backward:=false;
               end;

             hto:=comp_expr([ef_accept_equal]);
             consume(_DO);

             { Check if the constants fit in the range }
             check_range(hfrom,hloopvar.resultdef);
             check_range(hto,hloopvar.resultdef);

             { first set the varstate for from and to, so
               uses of loopvar in those expressions will also
               trigger a warning when it is not used yet. This
               needs to be done before the instruction block is
               parsed to have a valid hloopvar }
             typecheckpass(hfrom);
             set_varstate(hfrom,vs_read,[vsf_must_be_valid]);
             typecheckpass(hto);
             set_varstate(hto,vs_read,[vsf_must_be_valid]);
             typecheckpass(hloopvar);
             { in two steps, because vs_readwritten may turn on vsf_must_be_valid }
             { for some subnodes                                                  }
             set_varstate(hloopvar,vs_written,[]);
             set_varstate(hloopvar,vs_read,[vsf_must_be_valid]);

             { ... now the instruction block }
             hblock:=statement;

             { variable is not used for loop counter anymore }
             if assigned(loopvarsym) then
               exclude(loopvarsym.varoptions,vo_is_loop_counter);

             result:=cfornode.create(hloopvar,hfrom,hto,hblock,backward);
          end;


          function for_in_loop_create(hloopvar: tnode): tnode;
            var
              expr,hloopbody,hp: tnode;
              loopvarsym: tabstractvarsym;
            begin
              hp:=skip_nodes_before_load(hloopvar);
              if assigned(hp)and(hp.nodetype=loadn) then
                begin
                  loopvarsym:=tabstractvarsym(tloadnode(hp).symtableentry);
                  include(loopvarsym.varoptions,vo_is_loop_counter);
                end
              else
                loopvarsym:=nil;

              expr:=comp_expr([ef_accept_equal]);

              consume(_DO);

              set_varstate(hloopvar,vs_written,[]);
              set_varstate(hloopvar,vs_read,[vsf_must_be_valid]);

              hloopbody:=statement;
              if assigned(loopvarsym) then
                exclude(loopvarsym.varoptions,vo_is_loop_counter);
              result:=create_for_in_loop(hloopvar,hloopbody,expr);

              expr.free;
            end;


      var
         hloopvar: tnode;
      begin
         { parse loop header }
         consume(_FOR);

         hloopvar:=factor(false,[]);
         valid_for_loopvar(hloopvar,true);

         if try_to_consume(_ASSIGNMENT) then
           result:=for_loop_create(hloopvar)
         else if try_to_consume(_IN) then
           result:=for_in_loop_create(hloopvar)
         else
           begin
             consume(_ASSIGNMENT); // fail
             result:=cerrornode.create;
           end;
      end;


    function _with_statement : tnode;

      var
         p   : tnode;
         i   : longint;
         st  : TSymtable;
         newblock : tblocknode;
         newstatement : tstatementnode;
         calltempnode,
         tempnode : ttempcreatenode;
         valuenode,
         hp,
         refnode  : tnode;
         hdef : tdef;
         helperdef : tobjectdef;
         hasimplicitderef : boolean;
         withsymtablelist : TFPObjectList;

         procedure pushobjchild(withdef,obj:tobjectdef);
         var
           parenthelperdef : tobjectdef;
         begin
           if not assigned(obj) then
             exit;
           pushobjchild(withdef,obj.childof);
           { we need to look for helpers that were defined for the parent
             class as well }
           search_last_objectpascal_helper(obj,current_structdef,parenthelperdef);
           { push the symtables of the helper's parents in reverse order }
           if assigned(parenthelperdef) then
             pushobjchild(withdef,parenthelperdef.childof);
           { keep the original tobjectdef as owner, because that is used for
             visibility of the symtable }
           st:=twithsymtable.create(withdef,obj.symtable.SymList,refnode.getcopy);
           symtablestack.push(st);
           withsymtablelist.add(st);
           { push the symtable of the helper }
           if assigned(parenthelperdef) then
             begin
               st:=twithsymtable.create(withdef,parenthelperdef.symtable.SymList,refnode.getcopy);
               symtablestack.push(st);
               withsymtablelist.add(st);
             end;
         end;


      begin
         calltempnode:=nil;
         p:=comp_expr([ef_accept_equal]);
         do_typecheckpass(p);

         if (p.nodetype=vecn) and
            (nf_memseg in p.flags) then
           CGMessage(parser_e_no_with_for_variable_in_other_segments);

         { "with procvar" can never mean anything, so always try
           to call it in case it returns a record/object/... }
         maybe_call_procvar(p,false);

         if (p.resultdef.typ in [objectdef,recorddef,classrefdef]) or
           ((p.resultdef.typ=undefineddef) and (df_generic in current_procinfo.procdef.defoptions)) then
          begin
            newblock:=nil;
            valuenode:=nil;
            tempnode:=nil;

            hp:=skip_nodes_before_load(p);
            if (hp.nodetype=loadn) and
               (
                (tloadnode(hp).symtable=current_procinfo.procdef.localst) or
                (tloadnode(hp).symtable=current_procinfo.procdef.parast) or
                (tloadnode(hp).symtable.symtabletype in [staticsymtable,globalsymtable])
               ) and
               { MacPas objects are mapped to classes, and the MacPas compilers
                 interpret with-statements with MacPas objects the same way
                 as records (the object referenced by the with-statement
                 must remain constant)
               }
               not(is_class(hp.resultdef) and
                   (m_mac in current_settings.modeswitches)) then
              begin
                { simple load, we can reference direct }
                refnode:=p;
              end
            else
              begin
                { complex load, load in temp first }
                newblock:=internalstatements(newstatement);
                { when we can't take the address of p, load it in a temp }
                { since we may need its address later on                 }
                if not valid_for_addr(p,false) then
                  begin
                    calltempnode:=ctempcreatenode.create(p.resultdef,p.resultdef.size,tt_persistent,true);
                    addstatement(newstatement,calltempnode);
                    addstatement(newstatement,cassignmentnode.create(
                        ctemprefnode.create(calltempnode),
                        p));
                    p:=ctemprefnode.create(calltempnode);
                    typecheckpass(p);
                  end;
                { several object types have implicit dereferencing }
                { is_implicit_pointer_object_type() returns true for records
                  on the JVM target because they are implemented as classes
                  there, but we definitely have to take their address here
                  since otherwise a deep copy is made and changes are made to
                  this copy rather than to the original one }
                hasimplicitderef:=
                  (is_implicit_pointer_object_type(p.resultdef) or
                   (p.resultdef.typ=classrefdef)) and
                  not((target_info.system in systems_jvm) and
                      ((p.resultdef.typ=recorddef) or
                       is_object(p.resultdef)));
                if hasimplicitderef then
                  hdef:=p.resultdef
                else
                  hdef:=cpointerdef.create(p.resultdef);
                { load address of the value in a temp }
                tempnode:=ctempcreatenode.create_withnode(hdef,sizeof(pint),tt_persistent,true,p);
                typecheckpass(tnode(tempnode));
                valuenode:=p;
                refnode:=ctemprefnode.create(tempnode);
                fillchar(refnode.fileinfo,sizeof(tfileposinfo),0);
                { add address call for valuenode and deref for refnode if this
                  is not done implicitly }
                if not hasimplicitderef then
                  begin
                    valuenode:=caddrnode.create_internal_nomark(valuenode);
                    include(taddrnode(valuenode).addrnodeflags,anf_typedaddr);
                    refnode:=cderefnode.create(refnode);
                    fillchar(refnode.fileinfo,sizeof(tfileposinfo),0);
                  end;
                addstatement(newstatement,tempnode);
                addstatement(newstatement,cassignmentnode.create(
                    ctemprefnode.create(tempnode),
                    valuenode));
                typecheckpass(refnode);
              end;
            { Note: the symtable of the helper is pushed after the following
                    "case", the symtables of the helper's parents are passed in
                    the "case" branches }
            withsymtablelist:=TFPObjectList.create(true);
            case p.resultdef.typ of
              objectdef :
                begin
                   { do we have a helper for this type? }
                   search_last_objectpascal_helper(tabstractrecorddef(p.resultdef),current_structdef,helperdef);
                   { push symtables of all parents in reverse order }
                   pushobjchild(tobjectdef(p.resultdef),tobjectdef(p.resultdef).childof);
                   { push symtables of all parents of the helper in reverse order }
                   if assigned(helperdef) then
                     pushobjchild(helperdef,helperdef.childof);
                   { push object symtable }
                   st:=twithsymtable.Create(tobjectdef(p.resultdef),tobjectdef(p.resultdef).symtable.SymList,refnode);
                   symtablestack.push(st);
                   withsymtablelist.add(st);
                 end;
              classrefdef :
                begin
                   { do we have a helper for this type? }
                   search_last_objectpascal_helper(tobjectdef(tclassrefdef(p.resultdef).pointeddef),current_structdef,helperdef);
                   { push symtables of all parents in reverse order }
                   pushobjchild(tobjectdef(tclassrefdef(p.resultdef).pointeddef),tobjectdef(tclassrefdef(p.resultdef).pointeddef).childof);
                   { push symtables of all parents of the helper in reverse order }
                   if assigned(helperdef) then
                     pushobjchild(helperdef,helperdef.childof);
                   { push object symtable }
                   st:=twithsymtable.Create(tobjectdef(tclassrefdef(p.resultdef).pointeddef),tobjectdef(tclassrefdef(p.resultdef).pointeddef).symtable.SymList,refnode);
                   symtablestack.push(st);
                   withsymtablelist.add(st);
                end;
              recorddef :
                begin
                   { do we have a helper for this type? }
                   search_last_objectpascal_helper(tabstractrecorddef(p.resultdef),current_structdef,helperdef);
                   { push symtables of all parents of the helper in reverse order }
                   if assigned(helperdef) then
                     pushobjchild(helperdef,helperdef.childof);
                   { push record symtable }
                   st:=twithsymtable.create(trecorddef(p.resultdef),trecorddef(p.resultdef).symtable.SymList,refnode);
                   symtablestack.push(st);
                   withsymtablelist.add(st);
                end;
              undefineddef :
                begin
                   if not(df_generic in current_procinfo.procdef.defoptions) then
                     internalerror(2012122802);
                   helperdef:=nil;
                   { push record symtable }
                   st:=twithsymtable.create(p.resultdef,nil,refnode);
                   symtablestack.push(st);
                   withsymtablelist.add(st);
                end;
              else
                internalerror(200601271);
            end;

            { push helper symtable }
            if assigned(helperdef) then
              begin
                st:=twithsymtable.Create(helperdef,helperdef.symtable.SymList,refnode.getcopy);
                symtablestack.push(st);
                withsymtablelist.add(st);
              end;

            if try_to_consume(_COMMA) then
              p:=_with_statement()
            else
              begin
                consume(_DO);
                if token<>_SEMICOLON then
                  p:=statement
                else
                  p:=cnothingnode.create;
              end;

            { remove symtables in reverse order from the stack }
            for i:=withsymtablelist.count-1 downto 0 do
              symtablestack.pop(TSymtable(withsymtablelist[i]));
            withsymtablelist.free;

            { Finalize complex withnode with destroy of temp }
            if assigned(newblock) then
             begin
               addstatement(newstatement,p);
               if assigned(tempnode) then
                 addstatement(newstatement,ctempdeletenode.create(tempnode));
               if assigned(calltempnode) then
                 addstatement(newstatement,ctempdeletenode.create(calltempnode));
               p:=newblock;
             end;
            result:=p;
          end
         else
          begin
            p.free;
            Message1(parser_e_false_with_expr,p.resultdef.GetTypeName);
            { try to recover from error }
            if try_to_consume(_COMMA) then
             begin
               hp:=_with_statement();
               if (hp=nil) then; { remove warning about unused }
             end
            else
             begin
               consume(_DO);
               { ignore all }
               if token<>_SEMICOLON then
                statement;
             end;
            result:=nil;
          end;
      end;


    function with_statement : tnode;
      begin
         consume(_WITH);
         with_statement:=_with_statement();
      end;


    function raise_statement : tnode;
      var
         p,pobj,paddr,pframe : tnode;
      begin
         pobj:=nil;
         paddr:=nil;
         pframe:=nil;
         consume(_RAISE);
         if not(token in endtokens) then
           begin
              { object }
              pobj:=comp_expr([ef_accept_equal]);
              if try_to_consume(_AT) then
                begin
                   paddr:=comp_expr([ef_accept_equal]);
                   if try_to_consume(_COMMA) then
                     pframe:=comp_expr([ef_accept_equal]);
                end;
           end
         else
           begin
              if (block_type<>bt_except) then
                Message(parser_e_no_reraise_possible);
           end;
         p:=craisenode.create(pobj,paddr,pframe);
         raise_statement:=p;
      end;


    function try_statement : tnode;

      procedure check_type_valid(var def: tdef);
        begin
           if not (is_class(def) or is_javaclass(def) or
              { skip showing error message the second time }
              (def.typ=errordef)) then
             begin
               Message1(type_e_class_type_expected,def.typename);
               def:=generrordef;
             end;
        end;

      var
         p_try_block,p_finally_block,first,last,
         p_default,p_specific,hp : tnode;
         ot : tDef;
         sym : tlocalvarsym;
         old_block_type : tblock_type;
         excepTSymtable : TSymtable;
         objname,objrealname : TIDString;
         srsym : tsym;
         srsymtable : TSymtable;
         t:ttoken;
         unit_found:boolean;
         oldcurrent_exceptblock: integer;
         filepostry : tfileposinfo;
      begin
         p_default:=nil;
         p_specific:=nil;
         excepTSymtable:=nil;
         last:=nil;

         { read statements to try }
         consume(_TRY);
         filepostry:=current_filepos;
         first:=nil;
         inc(exceptblockcounter);
         oldcurrent_exceptblock := current_exceptblock;
         current_exceptblock := exceptblockcounter;
         old_block_type := block_type;
         block_type := bt_body;

         while (token<>_FINALLY) and (token<>_EXCEPT) do
           begin
              if first=nil then
                begin
                   last:=cstatementnode.create(statement,nil);
                   first:=last;
                end
              else
                begin
                   tstatementnode(last).right:=cstatementnode.create(statement,nil);
                   last:=tstatementnode(last).right;
                end;
              if not try_to_consume(_SEMICOLON) then
                break;
              consume_emptystats;
           end;
         p_try_block:=cblocknode.create(first);

         if try_to_consume(_FINALLY) then
           begin
              inc(exceptblockcounter);
              current_exceptblock := exceptblockcounter;
              p_finally_block:=statements_til_end;
              try_statement:=ctryfinallynode.create(p_try_block,p_finally_block);
              try_statement.fileinfo:=filepostry;
           end
         else
           begin
              consume(_EXCEPT);
              block_type:=bt_except;
              inc(exceptblockcounter);
              current_exceptblock := exceptblockcounter;
              ot:=generrordef;
              p_specific:=nil;
              if (idtoken=_ON) then
                { catch specific exceptions }
                begin
                   repeat
                     consume(_ON);
                     if token=_ID then
                       begin
                          objname:=pattern;
                          objrealname:=orgpattern;
                          { can't use consume_sym here, because we need already
                            to check for the colon }
                          searchsym(objname,srsym,srsymtable);
                          consume(_ID);
                          { is a explicit name for the exception given ? }
                          if try_to_consume(_COLON) then
                            begin
                              single_type(ot,[]);
                              check_type_valid(ot);
                              sym:=clocalvarsym.create(objrealname,vs_value,ot,[]);
                            end
                          else
                            begin
                               { check if type is valid, must be done here because
                                 with "e: Exception" the e is not necessary }

                               { support unit.identifier }
                               unit_found:=try_consume_unitsym_no_specialize(srsym,srsymtable,t,[],objname);
                               if srsym=nil then
                                 begin
                                   identifier_not_found(orgpattern);
                                   srsym:=generrorsym;
                                 end;
                               if unit_found then
                                 consume(t);
                               { check if type is valid, must be done here because
                                 with "e: Exception" the e is not necessary }
                               if (srsym.typ=typesym) then
                                 begin
                                   ot:=ttypesym(srsym).typedef;
                                   parse_nested_types(ot,false,false,nil);
                                   check_type_valid(ot);
                                 end
                               else
                                 begin
                                   Message(type_e_type_id_expected);
                                   ot:=generrordef;
                                 end;

                                 { create dummy symbol so we don't need a special
                                 case in ncgflw, and so that we always know the
                                 type }
                               sym:=clocalvarsym.create('$exceptsym',vs_value,ot,[]);
                            end;
                          excepTSymtable:=tstt_excepTSymtable.create;
                          excepTSymtable.insert(sym);
                          symtablestack.push(excepTSymtable);
                       end
                     else
                       consume(_ID);
                     consume(_DO);
                     hp:=connode.create(nil,statement);
                     if ot.typ=errordef then
                       begin
                          hp.free;
                          hp:=cerrornode.create;
                       end;
                     if p_specific=nil then
                       begin
                          last:=hp;
                          p_specific:=last;
                       end
                     else
                       begin
                          tonnode(last).left:=hp;
                          last:=tonnode(last).left;
                       end;
                     { set the informations }
                     { only if the creation of the onnode was succesful, it's possible }
                     { that last and hp are errornodes (JM)                            }
                     if last.nodetype = onn then
                       begin
                         tonnode(last).excepttype:=tobjectdef(ot);
                         tonnode(last).excepTSymtable:=excepTSymtable;
                       end;
                     { remove exception symtable }
                     if assigned(excepTSymtable) then
                       begin
                         symtablestack.pop(excepTSymtable);
                         if last.nodetype <> onn then
                           excepTSymtable.free;
                       end;
                     if not try_to_consume(_SEMICOLON) then
                        break;
                     consume_emptystats;
                   until (token in [_END,_ELSE]);
                   if try_to_consume(_ELSE) then
                     begin
                       { catch the other exceptions }
                       p_default:=statements_til_end;
                     end
                   else
                     consume(_END);
                end
              else
                begin
                   { catch all exceptions }
                   p_default:=statements_til_end;
                end;

              try_statement:=ctryexceptnode.create(p_try_block,p_specific,p_default);
           end;
         block_type:=old_block_type;
         current_exceptblock := oldcurrent_exceptblock;
      end;


    function _asm_statement : tnode;
      var
        asmstat : tasmnode;
        reg     : tregister;
        asmreader : tbaseasmreader;
        entrypos : tfileposinfo;
        hl : TAsmList;
      begin
         Inside_asm_statement:=true;
         asmstat:=nil;
         hl:=nil;
         if assigned(asmmodeinfos[current_settings.asmmode]) then
           begin
             asmreader:=asmmodeinfos[current_settings.asmmode]^.casmreader.create;
             entrypos:=current_filepos;
             hl:=asmreader.assemble as TAsmList;
             if (not hl.empty) then
               begin
                 { mark boundaries of assembler block, this is necessary for optimizer }
                 hl.insert(tai_marker.create(mark_asmblockstart));
                 hl.concat(tai_marker.create(mark_asmblockend));
               end;
             asmstat:=casmnode.create(hl);
             asmstat.fileinfo:=entrypos;
             asmreader.free;
           end
         else
           Message(parser_f_assembler_reader_not_supported);

         { Mark procedure that it has assembler blocks }
         include(current_procinfo.flags,pi_has_assembler_block);

         { Read first the _ASM statement }
         consume(_ASM);

         { Force an empty register list for pure assembler routines,
           so that pass2 won't allocate volatile registers for them. }
         asmstat.has_registerlist:=(po_assembler in current_procinfo.procdef.procoptions);

         { END is read, got a list of changed registers? }
         if try_to_consume(_LECKKLAMMER) then
           begin
             if token<>_RECKKLAMMER then
              begin
                if po_assembler in current_procinfo.procdef.procoptions then
                  Message(parser_w_register_list_ignored);
                repeat
                  { it's possible to specify the modified registers }
                  reg:=std_regnum_search(lower(cstringpattern));
                  if reg<>NR_NO then
                    begin
                      if not(po_assembler in current_procinfo.procdef.procoptions) and assigned(hl) then
                        begin
                          hl.Insert(tai_regalloc.alloc(reg,nil));
                          hl.Insert(tai_regalloc.markused(reg));
                          hl.Concat(tai_regalloc.dealloc(reg,nil));
                        end;
                    end
                  else
                    Message(asmr_e_invalid_register);
                  consume(_CSTRING);
                  if not try_to_consume(_COMMA) then
                    break;
                until false;
                asmstat.has_registerlist:=true;
              end;
             consume(_RECKKLAMMER);
           end;

         Inside_asm_statement:=false;
         _asm_statement:=asmstat;
      end;


    function statement : tnode;
      var
         p,
         code       : tnode;
         filepos    : tfileposinfo;
         srsym      : tsym;
         srsymtable : TSymtable;
         s          : TIDString;
      begin
         filepos:=current_tokenpos;
         code:=nil;
         case token of
           _GOTO :
             begin
                if not(cs_support_goto in current_settings.moduleswitches) then
                  Message(sym_e_goto_and_label_not_supported);
                consume(_GOTO);
                if (token<>_INTCONST) and (token<>_ID) then
                  begin
                    Message(sym_e_label_not_found);
                    code:=cerrornode.create;
                  end
                else
                  begin
                     if token=_ID then
                       consume_sym(srsym,srsymtable)
                     else
                      begin
                        if token<>_INTCONST then
                          internalerror(201008021);

                        { strip leading 0's in iso mode }
                        if (([m_iso,m_extpas]*current_settings.modeswitches)<>[]) then
                          while pattern[1]='0' do
                            delete(pattern,1,1);

                        searchsym(pattern,srsym,srsymtable);
                        if srsym=nil then
                          begin
                            identifier_not_found(pattern);
                            srsym:=generrorsym;
                            srsymtable:=nil;
                          end;
                        consume(token);
                      end;

                     if srsym.typ<>labelsym then
                       begin
                          Message(sym_e_id_is_no_label_id);
                          code:=cerrornode.create;
                       end
                     else
                       begin
                         { goto outside the current scope? }
                         if srsym.owner<>current_procinfo.procdef.localst then
                           begin
                             { allowed? }
                             if not(m_non_local_goto in current_settings.modeswitches) then
                               Message(parser_e_goto_outside_proc);
                             include(current_procinfo.flags,pi_has_global_goto);
                           end;
                         code:=cgotonode.create(tlabelsym(srsym));
                         tgotonode(code).labelsym:=tlabelsym(srsym);
                         { set flag that this label is used }
                         tlabelsym(srsym).used:=true;
                       end;
                  end;
             end;
           _BEGIN :
             code:=statement_block(_BEGIN);
           _IF :
             code:=if_statement;
           _CASE :
             code:=case_statement;
           _REPEAT :
             code:=repeat_statement;
           _WHILE :
             code:=while_statement;
           _FOR :
             code:=for_statement;
           _WITH :
             code:=with_statement;
           _TRY :
             code:=try_statement;
           _RAISE :
             code:=raise_statement;
           { semicolons,else until and end are ignored }
           _SEMICOLON,
           _ELSE,
           _UNTIL,
           _END:
             code:=cnothingnode.create;
           _FAIL :
             begin
                if (current_procinfo.procdef.proctypeoption<>potype_constructor) then
                  Message(parser_e_fail_only_in_constructor);
                consume(_FAIL);
                code:=cnodeutils.call_fail_node;
             end;
           _ASM :
             begin
               if parse_generic then
                 Message(parser_e_no_assembler_in_generic);
               code:=_asm_statement;
             end;
           _PLUS:
             begin
               Message(parser_e_syntax_error);
               consume(_PLUS);
             end;
           _EOF :
             Message(scan_f_end_of_file);
         else
           begin
             { don't typecheck yet, because that will also simplify, which may
               result in not detecting certain kinds of syntax errors --
               see mantis #15594 }
             p:=expr(false);
             { save the pattern here for latter usage, the label could be "000",
               even if we read an expression, the pattern is still valid if it's really
               a label (FK)
               if you want to mess here, take care of
               tests/webtbs/tw3546.pp
             }
             s:=pattern;

             { When a colon follows a intconst then transform it into a label }
             if (p.nodetype=ordconstn) and
                try_to_consume(_COLON) then
              begin
                { in iso mode, 0003: is equal to 3: }
                if (([m_iso,m_extpas]*current_settings.modeswitches)<>[]) then
                  searchsym(tostr(tordconstnode(p).value),srsym,srsymtable)
                else
                  searchsym(s,srsym,srsymtable);
                p.free;

                if assigned(srsym) and
                   (srsym.typ=labelsym) then
                 begin
                   if tlabelsym(srsym).defined then
                     Message(sym_e_label_already_defined);
                   if symtablestack.top.symtablelevel<>srsymtable.symtablelevel then
                     begin
                       tlabelsym(srsym).nonlocal:=true;
                       include(current_procinfo.flags,pi_has_interproclabel);
                     end;
                   if tlabelsym(srsym).nonlocal and
                     (current_procinfo.procdef.proctypeoption in [potype_unitinit,potype_unitfinalize]) then
                     Message(sym_e_interprocgoto_into_init_final_code_not_allowed);

                   tlabelsym(srsym).defined:=true;
                   p:=clabelnode.create(nil,tlabelsym(srsym));
                   tlabelsym(srsym).code:=p;
                 end
                else
                 begin
                   Message1(sym_e_label_used_and_not_defined,s);
                   p:=cnothingnode.create;
                 end;
              end;

             if p.nodetype=labeln then
               begin
                 { the pointer to the following instruction }
                 { isn't a very clean way                   }
                 if token in endtokens then
                   tlabelnode(p).left:=cnothingnode.create
                 else
                   tlabelnode(p).left:=statement();
                 { be sure to have left also typecheckpass }
                 typecheckpass(tlabelnode(p).left);
               end
             else

             { change a load of a procvar to a call. this is also
               supported in fpc mode }
             if p.nodetype in [vecn,derefn,typeconvn,subscriptn,loadn] then
               maybe_call_procvar(p,false);

             { blockn support because a read/write is changed into a blocknode
               with a separate statement for each read/write operation (JM)
               the same is true for val() if the third parameter is not 32 bit

               goto nodes are created by the compiler for non local exit statements, so
               include them as well
             }
             if not(p.nodetype in [nothingn,errorn,calln,ifn,assignn,breakn,inlinen,
                                   continuen,labeln,blockn,exitn,goton]) or
                ((p.nodetype=inlinen) and
                 not is_void(p.resultdef)) or
                ((p.nodetype=calln) and
                 (assigned(tcallnode(p).procdefinition)) and
                 (tcallnode(p).procdefinition.proctypeoption=potype_operator)) then
               Message(parser_e_illegal_expression);

             if not assigned(p.resultdef) then
               do_typecheckpass(p);

             { Specify that we don't use the value returned by the call.
               This is used for :
                - dispose of temp stack space
                - dispose on FPU stack
                - extended syntax checking }
             if (p.nodetype=calln) then
               begin
                 exclude(tcallnode(p).callnodeflags,cnf_return_value_used);

                 { in $x- state, the function result must not be ignored }
                 if not(cs_extsyntax in current_settings.moduleswitches) and
                    not(is_void(p.resultdef)) and
                    { can be nil in case there was an error in the expression }
                    assigned(tcallnode(p).procdefinition) and
                    { allow constructor calls to drop the result if they are
                      called as instance methods instead of class methods }
                    not(
                      (tcallnode(p).procdefinition.proctypeoption=potype_constructor) and
                      is_class_or_object(tprocdef(tcallnode(p).procdefinition).struct) and
                      assigned(tcallnode(p).methodpointer) and
                      (tnode(tcallnode(p).methodpointer).resultdef.typ=objectdef)
                    ) then
                   Message(parser_e_illegal_expression);
               end;

             code:=p;
           end;
         end;
         if assigned(code) then
           begin
             typecheckpass(code);
             code.fileinfo:=filepos;
           end;
         statement:=code;
      end;


    function statement_block(starttoken : ttoken) : tnode;

      var
         first,last : tnode;
         filepos : tfileposinfo;

      begin
         first:=nil;
         last:=nil;
         filepos:=current_tokenpos;
         consume(starttoken);

         while not((token=_END) or (token=_FINALIZATION)) do
           begin
              if first=nil then
                begin
                   last:=cstatementnode.create(statement,nil);
                   first:=last;
                end
              else
                begin
                   tstatementnode(last).right:=cstatementnode.create(statement,nil);
                   last:=tstatementnode(last).right;
                end;
              if ((token=_END) or (token=_FINALIZATION)) then
                break
              else
                begin
                   { if no semicolon, then error and go on }
                   if token<>_SEMICOLON then
                     begin
                        consume(_SEMICOLON);
                        consume_all_until(_SEMICOLON);
                     end;
                   consume(_SEMICOLON);
                end;
              consume_emptystats;
           end;

         { don't consume the finalization token, it is consumed when
           reading the finalization block, but allow it only after
           an initalization ! }
         if (starttoken<>_INITIALIZATION) or (token<>_FINALIZATION) then
           consume(_END);

         last:=cblocknode.create(first);
         last.fileinfo:=filepos;
         statement_block:=last;
      end;


    function assembler_block : tnode;
      var
        p : tnode;
        {$if not(defined(sparcgen)) and not(defined(arm)) and not(defined(avr)) and not(defined(mips))}
        locals : longint;
        {$endif not(defined(sparcgen)) and not(defined(arm)) and not(defined(avr)) and not(defined(mips))}
        srsym : tsym;
      begin
         if parse_generic then
           message(parser_e_no_assembler_in_generic);

         { Rename the funcret so that recursive calls are possible }
         if not is_void(current_procinfo.procdef.returndef) then
           begin
             srsym:=TSym(current_procinfo.procdef.localst.Find(current_procinfo.procdef.procsym.name));
             if assigned(srsym) then
               srsym.realname:='$hiddenresult';
           end;

         { delphi uses register calling for assembler methods }
         if (m_delphi in current_settings.modeswitches) and
            (po_assembler in current_procinfo.procdef.procoptions) and
            not(po_hascallingconvention in current_procinfo.procdef.procoptions) then
           current_procinfo.procdef.proccalloption:=pocall_register;

         { force the asm statement }
         if token<>_ASM then
           consume(_ASM);
         include(current_procinfo.flags,pi_is_assembler);
         p:=_asm_statement;

{$if not(defined(sparcgen)) and not(defined(arm)) and not(defined(avr)) and not(defined(mips))}
         if (po_assembler in current_procinfo.procdef.procoptions) then
           begin
             { set the framepointer to esp for assembler functions when the
               following conditions are met:
               - if the are no local variables and parameters (except the allocated result)
               - no reference to the result variable (refcount<=1)
               - result is not stored as parameter
               - target processor has optional frame pointer save
                 (vm, i386, vm only currently)
             }
             locals:=tabstractlocalsymtable(current_procinfo.procdef.parast).count_locals;
             if (current_procinfo.procdef.localst.symtabletype=localsymtable) then
               inc(locals,tabstractlocalsymtable(current_procinfo.procdef.localst).count_locals);
             if (locals=0) and
                not (current_procinfo.procdef.owner.symtabletype in [ObjectSymtable,recordsymtable]) and
                (not assigned(current_procinfo.procdef.funcretsym) or
                 (tabstractvarsym(current_procinfo.procdef.funcretsym).refs<=1)) and
                not (df_generic in current_procinfo.procdef.defoptions) and
                not(paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef)) then
               begin
                 { Only need to set the framepointer, the locals will
                   be inserted with the correct reference in tcgasmnode.pass_generate_code }
                 current_procinfo.framepointer:=NR_STACK_POINTER_REG;
               end;
           end;
{$endif not(defined(sparcgen)) and not(defined(arm)) and not(defined(avr)) not(defined(mipsel))}

        { Flag the result as assigned when it is returned in a
          register.
        }
        if assigned(current_procinfo.procdef.funcretsym) and
            not (df_generic in current_procinfo.procdef.defoptions) and
           (not paramanager.ret_in_param(current_procinfo.procdef.returndef,current_procinfo.procdef)) then
          tabstractvarsym(current_procinfo.procdef.funcretsym).varstate:=vs_initialised;

        { because the END is already read we need to get the
          last_endtoken_filepos here (PFV) }
        last_endtoken_filepos:=current_tokenpos;

        assembler_block:=p;
      end;

end.
