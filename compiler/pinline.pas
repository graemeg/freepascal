{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generates nodes for routines that need compiler support

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
unit pinline;

{$i fpcdefs.inc}

interface

    uses
      symtype,
      node,
      globals,
      cpuinfo;

    function new_dispose_statement(is_new:boolean) : tnode;
    function new_function : tnode;

    function inline_setlength : tnode;
    function inline_setstring : tnode;
    function inline_initialize : tnode;
    function inline_finalize : tnode;
    function inline_copy : tnode;


implementation

    uses
       { common }
       cutils,
       { global }
       globtype,tokens,verbose,constexp,
       systems,
       { symtable }
       symbase,symconst,symdef,symsym,symtable,defutil,
       { pass 1 }
       pass_1,htypechk,
       nmat,nadd,ncal,nmem,nset,ncnv,ninl,ncon,nld,nflw,nbas,nutils,ngenutil,
       { parser }
       scanner,
       pbase,pexpr,
       { codegen }
       cgbase
       ;


    function new_dispose_statement(is_new:boolean) : tnode;
      var
        newstatement : tstatementnode;
        temp         : ttempcreatenode;
        para         : tcallparanode;
        p,p2     : tnode;
        again    : boolean; { dummy for do_proc_call }
        destructorname : TIDString;
        sym      : tsym;
        classh   : tobjectdef;
        callflag : tcallnodeflag;
        destructorpos,
        storepos : tfileposinfo;
        variantdesc : pvariantrecdesc;
        found : boolean;
        j,i : longint;
        variantselectsymbol : tfieldvarsym;
      begin
        if target_info.system in systems_managed_vm then
          message(parser_e_feature_unsupported_for_vm);
        consume(_LKLAMMER);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        p:=comp_expr([ef_accept_equal]);
=======
        p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
        p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
        p:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
        p:=comp_expr(true,false);
>>>>>>> origin/cpstrnew
        { calc return type }
        if is_new then
          begin
            set_varstate(p,vs_written,[]);
            valid_for_var(p,true);
          end
        else
          set_varstate(p,vs_readwritten,[vsf_must_be_valid]);
        if (m_mac in current_settings.modeswitches) and
           is_class(p.resultdef) then
          begin
            classh:=tobjectdef(p.resultdef);

            { make sure we call ObjPas.TObject.Create/Free and not a random }
            { create/free method in a macpas descendent object (since those }
            { are not supposed to be called automatically when you call     }
            { new/dispose)                                                  }
            while assigned(classh.childof) do
              classh := classh.childof;
            if is_new then
              begin
<<<<<<< HEAD
                sym:=search_struct_member(classh,'CREATE');
=======
                sym:=search_class_member(classh,'CREATE');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                p2 := cloadvmtaddrnode.create(ctypenode.create(p.resultdef));
              end
            else
              begin
                sym:=search_struct_member(classh,'FREE');
                p2 := p;
             end;

            if not(assigned(sym)) then
              begin
                 p.free;
                 if is_new then
                   p2.free;
                 new_dispose_statement := cerrornode.create;
                 consume_all_until(_RKLAMMER);
                 consume(_RKLAMMER);
                 exit;
              end;

            do_member_read(classh,false,sym,p2,again,[],nil);

            { we need the real called method }
            do_typecheckpass(p2);

            if (p2.nodetype=calln) and
               assigned(tcallnode(p2).procdefinition) then
              begin
                if is_new then
                  begin
                    if (tcallnode(p2).procdefinition.proctypeoption<>potype_constructor) then
                      Message(parser_e_expr_have_to_be_constructor_call);
                    p2.resultdef:=p.resultdef;
                    p2:=cassignmentnode.create(p,p2);
                    typecheckpass(p2);
                  end
                else
                  begin
                   { Free is not a destructor
                    if (tcallnode(p2).procdefinition.proctypeoption<>potype_destructor) then
                      Message(parser_e_expr_have_to_be_destructor_call);
                   }
                  end
              end
            else
              internalerror(2005061202);
            new_dispose_statement := p2;
          end
        { constructor,destructor specified }
        else if (([m_mac,m_iso,m_extpas]*current_settings.modeswitches)=[]) and
                try_to_consume(_COMMA) then
          begin
            { extended syntax of new and dispose }
            { function styled new is handled in factor }
            { destructors have no parameters }
            destructorname:=pattern;
            destructorpos:=current_tokenpos;
            consume(_ID);

            if is_typeparam(p.resultdef) then
              begin
                 p.free;
                 p:=factor(false,[]);
                 p.free;
                 consume(_RKLAMMER);
                 new_dispose_statement:=cnothingnode.create;
                 exit;
              end;

            if (p.resultdef.typ<>pointerdef) then
              begin
                 Message1(type_e_pointer_type_expected,p.resultdef.typename);
                 p.free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                 p:=factor(false,[]);
=======
                 p:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 p:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 p:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 p:=factor(false,false);
>>>>>>> origin/cpstrnew
                 p.free;
                 consume(_RKLAMMER);
                 new_dispose_statement:=cerrornode.create;
                 exit;
              end;
            { first parameter must be an object or class }
            if tpointerdef(p.resultdef).pointeddef.typ<>objectdef then
              begin
                 Message(parser_e_pointer_to_class_expected);
                 p.free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                 new_dispose_statement:=factor(false,[]);
=======
                 new_dispose_statement:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 new_dispose_statement:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 new_dispose_statement:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 new_dispose_statement:=factor(false,false);
>>>>>>> origin/cpstrnew
                 consume_all_until(_RKLAMMER);
                 consume(_RKLAMMER);
                 exit;
              end;
            { check, if the first parameter is a pointer to a _class_ }
            classh:=tobjectdef(tpointerdef(p.resultdef).pointeddef);
            if is_class(classh) then
              begin
                 Message(parser_e_no_new_or_dispose_for_classes);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                 new_dispose_statement:=factor(false,[]);
=======
                 new_dispose_statement:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 new_dispose_statement:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 new_dispose_statement:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
                 new_dispose_statement:=factor(false,false);
>>>>>>> origin/cpstrnew
                 consume_all_until(_RKLAMMER);
                 consume(_RKLAMMER);
                 exit;
              end;
            { search cons-/destructor, also in parent classes }
            storepos:=current_tokenpos;
            current_tokenpos:=destructorpos;
            sym:=search_struct_member(classh,destructorname);
            current_tokenpos:=storepos;

            { the second parameter of new/dispose must be a call }
            { to a cons-/destructor                              }
            if (not assigned(sym)) or (sym.typ<>procsym) then
              begin
                 if is_new then
                  Message(parser_e_expr_have_to_be_constructor_call)
                 else
                  Message(parser_e_expr_have_to_be_destructor_call);
                 p.free;
                 new_dispose_statement:=cerrornode.create;
              end
            else
              begin
                { For new(var,constructor) we need to take a copy because
                  p is also used in the assignmentn below }
                if is_new then
                  begin
                    p2:=cderefnode.create(p.getcopy);
                    include(p2.flags,nf_no_checkpointer);
                  end
                else
                  p2:=cderefnode.create(p);
                do_typecheckpass(p2);
                if is_new then
                  callflag:=cnf_new_call
                else
                  callflag:=cnf_dispose_call;
                if is_new then
                  do_member_read(classh,false,sym,p2,again,[callflag],nil)
                else
                  begin
                    if not(m_fpc in current_settings.modeswitches) then
                      do_member_read(classh,false,sym,p2,again,[callflag],nil)
                    else
                      begin
                        p2:=ccallnode.create(nil,tprocsym(sym),sym.owner,p2,[callflag],nil);
                        { support dispose(p,done()); }
                        if try_to_consume(_LKLAMMER) then
                          begin
                            if not try_to_consume(_RKLAMMER) then
                              begin
                                Message(parser_e_no_paras_for_destructor);
                                consume_all_until(_RKLAMMER);
                                consume(_RKLAMMER);
                              end;
                          end;
                      end;
                  end;

                { we need the real called method }
                do_typecheckpass(p2);

                if (p2.nodetype=calln) and
                   assigned(tcallnode(p2).procdefinition) then
                  begin
                    if is_new then
                     begin
                       if (tcallnode(p2).procdefinition.proctypeoption<>potype_constructor) then
                         Message(parser_e_expr_have_to_be_constructor_call);
                       p2.resultdef:=p.resultdef;
                       p2:=cassignmentnode.create(p,p2);
                     end
                    else
                     begin
                       if (tcallnode(p2).procdefinition.proctypeoption<>potype_destructor) then
                         Message(parser_e_expr_have_to_be_destructor_call);
                     end;
                  end
                else
                  begin
                    if is_new then
                      CGMessage(parser_e_expr_have_to_be_constructor_call)
                    else
                      CGMessage(parser_e_expr_have_to_be_destructor_call);
                  end;

                result:=p2;
              end;
          end
        else
          begin
             if (p.resultdef.typ<>pointerdef) then
               Begin
                 if is_typeparam(p.resultdef) then
                   begin
                      p.free;
                      consume(_RKLAMMER);
                      new_dispose_statement:=cnothingnode.create;
                      exit;
                   end
                 else
                   begin
                     Message1(type_e_pointer_type_expected,p.resultdef.typename);
                     new_dispose_statement:=cerrornode.create;
                   end;
               end
             else
               begin
                  if (tpointerdef(p.resultdef).pointeddef.typ=objectdef) and
                     (oo_has_vmt in tobjectdef(tpointerdef(p.resultdef).pointeddef).objectoptions) then
                    Message(parser_w_use_extended_syntax_for_objects);
                  if (tpointerdef(p.resultdef).pointeddef.typ=orddef) and
                     (torddef(tpointerdef(p.resultdef).pointeddef).ordtype=uvoid) then
                    begin
                      if (m_tp7 in current_settings.modeswitches) or
                         (m_delphi in current_settings.modeswitches) then
                       Message(parser_w_no_new_dispose_on_void_pointers)
                      else
                       Message(parser_e_no_new_dispose_on_void_pointers);
                    end;

                  { create statements with call to getmem+initialize or
                    finalize+freemem }
                  new_dispose_statement:=internalstatements(newstatement);

                  if is_new then
                   begin
                     { create temp for result }
                     temp := ctempcreatenode.create(p.resultdef,p.resultdef.size,tt_persistent,true);
                     addstatement(newstatement,temp);

                     { create call to fpc_getmem }
                     para := ccallparanode.create(cordconstnode.create
                         (tpointerdef(p.resultdef).pointeddef.size,s32inttype,true),nil);
                     addstatement(newstatement,cassignmentnode.create(
                         ctemprefnode.create(temp),
                         ccallnode.createintern('fpc_getmem',para)));

                     { create call to fpc_initialize }
                     if is_managed_type(tpointerdef(p.resultdef).pointeddef) or
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       ((m_isolike_io in current_settings.modeswitches) and (tpointerdef(p.resultdef).pointeddef.typ=filedef)) then
                       addstatement(newstatement,cnodeutils.initialize_data_node(cderefnode.create(ctemprefnode.create(temp)),false));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                       ((m_iso in current_settings.modeswitches) and (tpointerdef(p.resultdef).pointeddef.typ=filedef)) then
                       addstatement(newstatement,initialize_data_node(cderefnode.create(ctemprefnode.create(temp))));
>>>>>>> graemeg/cpstrnew

                     { copy the temp to the destination }
                     addstatement(newstatement,cassignmentnode.create(
                         p,
                         ctemprefnode.create(temp)));

                     if (([m_iso,m_extpas]*current_settings.modeswitches)<>[]) and (is_record(tpointerdef(p.resultdef).pointeddef)) then
                       begin
                         variantdesc:=trecorddef(tpointerdef(p.resultdef).pointeddef).variantrecdesc;
                         while (token=_COMMA) and assigned(variantdesc) do
                           begin
                             consume(_COMMA);
                             p2:=factor(false,[]);
                             do_typecheckpass(p2);
                             if p2.nodetype=ordconstn then
                               begin
                                 found:=false;
                                 { we do not have dynamic dfa, so avoid warning on variantselectsymbol below }
                                 variantselectsymbol:=nil;
                                 for i:=0 to high(variantdesc^.branches) do
                                   begin
                                     for j:=0 to high(variantdesc^.branches[i].values) do
                                       if variantdesc^.branches[i].values[j]=tordconstnode(p2).value then
                                         begin
                                           found:=true;
                                           variantselectsymbol:=tfieldvarsym(variantdesc^.variantselector);
                                           variantdesc:=variantdesc^.branches[i].nestedvariant;
                                           break;
                                         end;
                                     if found then
                                       break;
                                   end;
                                 if found then
                                   begin
                                     { if no tag-field is given, do not create an assignment statement for it }
                                     if assigned(variantselectsymbol) then
                                       { setup variant selector }
                                       addstatement(newstatement,cassignmentnode.create(
                                           csubscriptnode.create(variantselectsymbol,
                                             cderefnode.create(ctemprefnode.create(temp))),
                                           p2));
                                   end
                                 else
                                   Message(parser_e_illegal_expression);
                               end
                             else
                               Message(parser_e_illegal_expression);
                           end;
                       end;
                     { release temp }
                     addstatement(newstatement,ctempdeletenode.create(temp));
                   end
                  else
                   begin
                     { create call to fpc_finalize }
                     if is_managed_type(tpointerdef(p.resultdef).pointeddef) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       addstatement(newstatement,cnodeutils.finalize_data_node(cderefnode.create(p.getcopy)));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                       addstatement(newstatement,finalize_data_node(cderefnode.create(p.getcopy)));
>>>>>>> graemeg/cpstrnew

                     { create call to fpc_freemem }
                     para := ccallparanode.create(p,nil);
                     addstatement(newstatement,ccallnode.createintern('fpc_freemem',para));
                   end;
               end;
          end;
        consume(_RKLAMMER);
      end;


    function new_function : tnode;
      var
        p1,p2  : tnode;
        classh : tobjectdef;
        srsym    : tsym;
        srsymtable : TSymtable;
        again  : boolean; { dummy for do_proc_call }
      begin
        if target_info.system in systems_managed_vm then
          message(parser_e_feature_unsupported_for_vm);
        consume(_LKLAMMER);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        p1:=factor(false,[]);
=======
        p1:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
        p1:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
        p1:=factor(false,false);
>>>>>>> graemeg/cpstrnew
=======
        p1:=factor(false,false);
>>>>>>> origin/cpstrnew
        if p1.nodetype<>typen then
         begin
           Message(type_e_type_id_expected);
           consume_all_until(_RKLAMMER);
           consume(_RKLAMMER);
           p1.destroy;
           new_function:=cerrornode.create;
           exit;
         end;

        if (p1.resultdef.typ<>pointerdef) then
         begin
           Message1(type_e_pointer_type_expected,p1.resultdef.typename);
           consume_all_until(_RKLAMMER);
           consume(_RKLAMMER);
           p1.destroy;
           new_function:=cerrornode.create;
           exit;
         end;

        if try_to_consume(_RKLAMMER) then
          begin
            if (tpointerdef(p1.resultdef).pointeddef.typ=objectdef) and
               (oo_has_vmt in tobjectdef(tpointerdef(p1.resultdef).pointeddef).objectoptions)  then
              Message(parser_w_use_extended_syntax_for_objects);

            if p1.nodetype=typen then
              ttypenode(p1).allowed:=true;

<<<<<<< HEAD
            p1:=cinlinenode.create(in_new_x,false,p1);
=======
            { create call to fpc_initialize }
            if is_managed_type(tpointerdef(p1.resultdef).pointeddef) then
             begin
               para := ccallparanode.create(caddrnode.create_internal(crttinode.create
                          (tstoreddef(tpointerdef(p1.resultdef).pointeddef),initrtti,rdt_normal)),
                       ccallparanode.create(ctemprefnode.create
                          (temp),nil));
               addstatement(newstatement,ccallnode.createintern('fpc_initialize',para));
             end;

            { the last statement should return the value as
              location and type, this is done be referencing the
              temp and converting it first from a persistent temp to
              normal temp }
            addstatement(newstatement,ctempdeletenode.create_normal_temp(temp));
            addstatement(newstatement,ctemprefnode.create(temp));

            p1.destroy;
            p1:=newblock;
>>>>>>> graemeg/cpstrnew
          end
        else
          begin
            consume(_COMMA);
            if tpointerdef(p1.resultdef).pointeddef.typ<>objectdef then
             begin
               Message(parser_e_pointer_to_class_expected);
               consume_all_until(_RKLAMMER);
               consume(_RKLAMMER);
               p1.destroy;
               new_function:=cerrornode.create;
               exit;
             end;
            classh:=tobjectdef(tpointerdef(p1.resultdef).pointeddef);
            { use the objectdef for loading the VMT }
            p2:=p1;
            p1:=ctypenode.create(tpointerdef(p1.resultdef).pointeddef);
            do_typecheckpass(p1);
            { search the constructor also in the symbol tables of
              the parents }
            afterassignment:=false;
            searchsym_in_class(classh,classh,pattern,srsym,srsymtable,[ssf_search_helper]);
            consume(_ID);
            do_member_read(classh,false,srsym,p1,again,[cnf_new_call],nil);
            { we need to know which procedure is called }
            do_typecheckpass(p1);
            if not(
                   (p1.nodetype=calln) and
                   assigned(tcallnode(p1).procdefinition) and
                   (tcallnode(p1).procdefinition.proctypeoption=potype_constructor)
                  ) then
              Message(parser_e_expr_have_to_be_constructor_call);
            { constructors return boolean, update resultdef to return
              the pointer to the object }
            p1.resultdef:=p2.resultdef;
            p2.free;
            consume(_RKLAMMER);
          end;
        new_function:=p1;
      end;


    function inline_setlength : tnode;
      var
        paras: tnode;
      begin
        consume(_LKLAMMER);
        paras:=parse_paras(false,false,_RKLAMMER);
        consume(_RKLAMMER);
        if not assigned(paras) then
         begin
           result:=cerrornode.create;
           CGMessage1(parser_e_wrong_parameter_size,'SetLength');
           exit;
         end;
<<<<<<< HEAD
        result:=cinlinenode.create(in_setlength_x,false,paras);
      end;


    function inline_setstring : tnode;
      var
        paras, strpara, pcharpara: tnode;
        procname: string;
      begin
        consume(_LKLAMMER);
        paras:=parse_paras(false,false,_RKLAMMER);
        consume(_RKLAMMER);
        procname:='';
        if assigned(paras) and
           assigned(tcallparanode(paras).right) and
           assigned(tcallparanode(tcallparanode(paras).right).right) then
          begin
            do_typecheckpass(tcallparanode(tcallparanode(paras).right).left);
            do_typecheckpass(tcallparanode(tcallparanode(tcallparanode(paras).right).right).left);
            pcharpara:=tcallparanode(tcallparanode(paras).right).left;
            strpara:=tcallparanode(tcallparanode(tcallparanode(paras).right).right).left;
            if strpara.resultdef.typ=stringdef then
              begin
                { if there are three parameters and the first parameter
                  ( = paras.right.right) is an ansistring, add a codepage
                  parameter }
                if is_ansistring(strpara.resultdef) then
                  paras:=ccallparanode.create(genintconstnode(tstringdef(strpara.resultdef).encoding),paras);
                procname:='fpc_setstring_'+tstringdef(strpara.resultdef).stringtypname;
                { decide which version to call based on the second parameter }
                if not is_shortstring(strpara.resultdef) then
                  if is_pwidechar(pcharpara.resultdef) or
                     is_widechar(pcharpara.resultdef) or
                     ((pcharpara.resultdef.typ=arraydef) and
                      is_widechar(tarraydef(pcharpara.resultdef).elementdef)) then
                    procname:=procname+'_pwidechar'
                  else
                    procname:=procname+'_pansichar';
              end;
          end;
        { default version (for error message) in case of missing or wrong
          parameters }
        if procname='' then
          if m_default_unicodestring in current_settings.modeswitches then
            procname:='fpc_setstring_unicodestr_pwidechar'
          else if m_default_ansistring in current_settings.modeswitches then
            procname:='fpc_setstring_ansistr_pansichar'
          else
            procname:='fpc_setstring_shortstr';
        result:=ccallnode.createintern(procname,paras)
      end;


=======

        dims:=0;
        if assigned(paras) then
         begin
           { check type of lengths }
           ppn:=tcallparanode(paras);
           while assigned(ppn.right) do
            begin
              set_varstate(ppn.left,vs_read,[vsf_must_be_valid]);
              inserttypeconv(ppn.left,sinttype);
              inc(dims);
              ppn:=tcallparanode(ppn.right);
            end;
         end;
        if dims=0 then
         begin
           CGMessage1(parser_e_wrong_parameter_size,'SetLength');
           paras.free;
           exit;
         end;
        { last param must be var }
        destppn:=ppn.left;
        valid_for_var(destppn,true);
        set_varstate(destppn,vs_written,[]);
        { first param must be a string or dynamic array ...}
        isarray:=is_dynamic_array(destppn.resultdef);
        if not((destppn.resultdef.typ=stringdef) or
               isarray) then
         begin
           CGMessage(type_e_mismatch);
           paras.free;
           exit;
         end;

        { only dynamic arrays accept more dimensions }
        if (dims>1) then
         begin
           if (not isarray) then
            CGMessage(type_e_mismatch)
           else
            begin
              { check if the amount of dimensions is valid }
              def := tarraydef(destppn.resultdef).elementdef;
              counter:=dims;
              while counter > 1 do
                begin
                  if not(is_dynamic_array(def)) then
                    begin
                      CGMessage1(parser_e_wrong_parameter_size,'SetLength');
                      break;
                    end;
                  dec(counter);
                  def := tarraydef(def).elementdef;
                end;
            end;
         end;

        if isarray then
         begin
            { create statements with call initialize the arguments and
              call fpc_dynarr_setlength }
            newblock:=internalstatements(newstatement);

            { get temp for array of lengths }
            temp := ctempcreatenode.create(sinttype,dims*sinttype.size,tt_persistent,false);
            addstatement(newstatement,temp);

            { load array of lengths }
            ppn:=tcallparanode(paras);
            counter:=0;
            while assigned(ppn.right) do
             begin
               addstatement(newstatement,cassignmentnode.create(
                   ctemprefnode.create_offset(temp,counter*sinttype.size),
                   ppn.left));
               ppn.left:=nil;
               inc(counter);
               ppn:=tcallparanode(ppn.right);
             end;
            { destppn is also reused }
            ppn.left:=nil;

            { create call to fpc_dynarr_setlength }
            npara:=ccallparanode.create(caddrnode.create_internal
                      (ctemprefnode.create(temp)),
                   ccallparanode.create(cordconstnode.create
                      (counter,s32inttype,true),
                   ccallparanode.create(caddrnode.create_internal
                      (crttinode.create(tstoreddef(destppn.resultdef),initrtti,rdt_normal)),
                   ccallparanode.create(ctypeconvnode.create_internal(destppn,voidpointertype),nil))));
            addstatement(newstatement,ccallnode.createintern('fpc_dynarray_setlength',npara));
            addstatement(newstatement,ctempdeletenode.create(temp));

            { we don't need original the callparanodes tree }
            paras.free;
         end
        else if is_ansistring(destppn.resultdef) then
         begin
            newblock:=ccallnode.createintern(
              'fpc_'+tstringdef(destppn.resultdef).stringtypname+'_setlength',
              ccallparanode.create(
                cordconstnode.create(tstringdef(destppn.resultdef).encoding,u16inttype,true),
                paras
              )
            );           
         end
        else
         begin
            { we can reuse the supplied parameters }
            newblock:=ccallnode.createintern(
               'fpc_'+tstringdef(destppn.resultdef).stringtypname+'_setlength',paras);
         end;

        result.free;
        result:=newblock;
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
    function inline_initfinal(isinit: boolean): tnode;
      var
        newblock,
        paras   : tnode;
        npara,
        destppn,
        ppn     : tcallparanode;
      begin
        { for easy exiting if something goes wrong }
        result := cerrornode.create;

        consume(_LKLAMMER);
        paras:=parse_paras(false,false,_RKLAMMER);
        consume(_RKLAMMER);
        ppn:=tcallparanode(paras);

        if not assigned(paras) or
           (assigned(ppn.right) and
            assigned(tcallparanode(ppn.right).right)) then
         begin
           if isinit then
             CGMessage1(parser_e_wrong_parameter_size,'Initialize')
           else
             CGMessage1(parser_e_wrong_parameter_size,'Finalize');
           exit;
         end;

        { 2 arguments? }
        if assigned(ppn.right) then
         begin
           destppn:=tcallparanode(ppn.right);
           { create call to fpc_initialize/finalize_array }
           npara:=ccallparanode.create(ctypeconvnode.create
                     (ppn.left,s32inttype),
                  ccallparanode.create(caddrnode.create_internal
                     (crttinode.create(tstoreddef(destppn.left.resultdef),initrtti,rdt_normal)),
                  ccallparanode.create(caddrnode.create_internal
                     (destppn.left),nil)));
           if isinit then
             newblock:=ccallnode.createintern('fpc_initialize_array',npara)
           else
             newblock:=ccallnode.createintern('fpc_finalize_array',npara);
           destppn.left:=nil;
         end
        else
         begin
           if isinit then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             newblock:=cnodeutils.initialize_data_node(ppn.left,true)
           else
             newblock:=cnodeutils.finalize_data_node(ppn.left);
=======
             newblock:=initialize_data_node(ppn.left)
           else
             newblock:=finalize_data_node(ppn.left);
>>>>>>> graemeg/cpstrnew
=======
             newblock:=initialize_data_node(ppn.left)
           else
             newblock:=finalize_data_node(ppn.left);
>>>>>>> graemeg/cpstrnew
=======
             newblock:=initialize_data_node(ppn.left)
           else
             newblock:=finalize_data_node(ppn.left);
>>>>>>> graemeg/cpstrnew
=======
             newblock:=initialize_data_node(ppn.left)
           else
             newblock:=finalize_data_node(ppn.left);
>>>>>>> origin/cpstrnew
         end;
        ppn.left:=nil;
        paras.free;
        result.free;
        result:=newblock;
      end;


    function inline_initialize : tnode;
      begin
        result:=inline_initfinal(true);
      end;


    function inline_finalize : tnode;
      begin
        result:=inline_initfinal(false);
      end;


    function inline_copy : tnode;
      var
        paras   : tnode;
        { for easy exiting if something goes wrong }
      begin
        result := cerrornode.create;

        consume(_LKLAMMER);
        paras:=parse_paras(false,false,_RKLAMMER);
        consume(_RKLAMMER);
        if not assigned(paras) then
<<<<<<< HEAD
=======
         begin
           CGMessage1(parser_e_wrong_parameter_size,'Copy');
           exit;
         end;

        { determine copy function to use based on the first argument,
          also count the number of arguments in this loop }
        counter:=1;
        ppn:=tcallparanode(paras);
        while assigned(ppn.right) do
         begin
           inc(counter);
           ppn:=tcallparanode(ppn.right);
         end;
        paradef:=ppn.left.resultdef;
        if is_ansistring(paradef) or
           (is_chararray(paradef) and
            (paradef.size>255)) or
           ((cs_ansistrings in current_settings.localswitches) and
            is_pchar(paradef)) then
          copynode:=ccallnode.createintern('fpc_ansistr_copy',paras)
        else
         if is_widestring(paradef) then
           copynode:=ccallnode.createintern('fpc_widestr_copy',paras)
        else
         if is_unicodestring(paradef) or
            is_widechararray(paradef) or
            is_pwidechar(paradef) then
           copynode:=ccallnode.createintern('fpc_unicodestr_copy',paras)
        else
         if is_char(paradef) then
           copynode:=ccallnode.createintern('fpc_char_copy',paras)
        else
         if is_dynamic_array(paradef) then
>>>>>>> graemeg/cpstrnew
          begin
            CGMessage1(parser_e_wrong_parameter_size,'Copy');
            exit;
          end;
        result.free;
        result:=cinlinenode.create(in_copy_x,false,paras);
      end;

end.
