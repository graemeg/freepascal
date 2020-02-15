{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Does parsing of expression for Free Pascal

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
unit pexpr;

{$i fpcdefs.inc}

interface

    uses
      symtype,symdef,symbase,
      node,ncal,compinnr,
      tokens,globtype,globals,constexp,
      pgentype;

    type
      texprflag = (
        ef_accept_equal,
        ef_type_only,
        ef_had_specialize,
        ef_check_attr_suffix
      );
      texprflags = set of texprflag;

    { reads a whole expression }
    function expr(dotypecheck:boolean) : tnode;

    { reads an expression without assignements and .. }
    function comp_expr(flags:texprflags):tnode;

    { reads a single factor }
    function factor(getaddr:boolean;flags:texprflags) : tnode;

    procedure string_dec(var def: tdef; allowtypedef: boolean);

    function parse_paras(__colon,__namedpara : boolean;end_of_paras : ttoken) : tnode;

    { the ID token has to be consumed before calling this function }
    procedure do_member_read(structh:tabstractrecorddef;getaddr:boolean;sym:tsym;var p1:tnode;var again:boolean;callflags:tcallnodeflags;spezcontext:tspecializationcontext);

    function get_intconst:TConstExprInt;
    function get_stringconst:string;

    { Does some postprocessing for a generic type (especially when nested types
      of the specialization are used) }
    procedure post_comp_expr_gendef(var def: tdef);

implementation

    uses
       { common }
       cutils,cclasses,
       { global }
       verbose,
       systems,widestr,
       { symtable }
       symconst,symtable,symsym,symcpu,defutil,defcmp,
       { module }
       fmodule,ppu,
       { pass 1 }
       pass_1,
       nmat,nadd,nmem,nset,ncnv,ninl,ncon,nld,nflw,nbas,nutils,
       { parser }
       scanner,
       pbase,pinline,ptype,pgenutil,procinfo,cpuinfo
       ;

    function sub_expr(pred_level:Toperator_precedence;flags:texprflags;factornode:tnode):tnode;forward;

    const
       { true, if the inherited call is anonymous }
       anon_inherited : boolean = false;
       { last def found, only used by anon. inherited calls to insert proper type casts }
       srdef : tdef = nil;

    procedure string_dec(var def:tdef; allowtypedef: boolean);
    { reads a string type with optional length }
    { and returns a pointer to the string      }
    { definition                               }
      var
         p : tnode;
      begin
         def:=cshortstringtype;
         consume(_STRING);
         if token=_LECKKLAMMER then
           begin
             if not(allowtypedef) then
               Message(parser_e_no_local_para_def);
             consume(_LECKKLAMMER);
             p:=comp_expr([ef_accept_equal]);
             if not is_constintnode(p) then
               begin
                 Message(parser_e_illegal_expression);
                 { error recovery }
                 consume(_RECKKLAMMER);
               end
             else
               begin
                if (tordconstnode(p).value<=0) then
                  begin
                     Message(parser_e_invalid_string_size);
                     tordconstnode(p).value:=255;
                  end;
                if tordconstnode(p).value>255 then
                  begin
                    { longstring is currently unsupported (CEC)! }
{                    t:=cstringdef.createlong(tordconstnode(p).value))}
                    Message(parser_e_invalid_string_size);
                    tordconstnode(p).value:=255;
                    def:=cstringdef.createshort(int64(tordconstnode(p).value),true);
                  end
                else
                  if tordconstnode(p).value<>255 then
                    def:=cstringdef.createshort(int64(tordconstnode(p).value),true);
                consume(_RECKKLAMMER);
              end;
             p.free;
           end
          else
            begin
              if cs_refcountedstrings in current_settings.localswitches then
                begin
                  if m_default_unicodestring in current_settings.modeswitches then
                    def:=cunicodestringtype
                  else
                    def:=cansistringtype
                end
              else
                def:=cshortstringtype;
            end;
       end;


    function parse_paras(__colon,__namedpara : boolean;end_of_paras : ttoken) : tnode;
      var
         p1,p2,argname : tnode;
         prev_in_args,
         old_named_args_allowed : boolean;
      begin
         if token=end_of_paras then
           begin
              parse_paras:=nil;
              exit;
           end;
         { save old values }
         prev_in_args:=in_args;
         old_named_args_allowed:=named_args_allowed;
         { set para parsing values }
         in_args:=true;
         named_args_allowed:=false;
         p2:=nil;
         repeat
           if __namedpara then
             begin
               if token=_COMMA then
                 begin
                   { empty parameter }
                   p2:=ccallparanode.create(cnothingnode.create,p2);
                 end
               else
                 begin
                   named_args_allowed:=true;
                   p1:=comp_expr([ef_accept_equal]);
                   named_args_allowed:=false;
                   if found_arg_name then
                     begin
                       argname:=p1;
                       p1:=comp_expr([ef_accept_equal]);
                       p2:=ccallparanode.create(p1,p2);
                       tcallparanode(p2).parametername:=argname;
                     end
                   else
                     p2:=ccallparanode.create(p1,p2);
                   found_arg_name:=false;
                 end;
             end
           else
             begin
               p1:=comp_expr([ef_accept_equal]);
               p2:=ccallparanode.create(p1,p2);
             end;
           { it's for the str(l:5,s); }
           if __colon and (token=_COLON) then
             begin
               consume(_COLON);
               p1:=comp_expr([ef_accept_equal]);
               p2:=ccallparanode.create(p1,p2);
               include(tcallparanode(p2).callparaflags,cpf_is_colon_para);
               if try_to_consume(_COLON) then
                 begin
                   p1:=comp_expr([ef_accept_equal]);
                   p2:=ccallparanode.create(p1,p2);
                   include(tcallparanode(p2).callparaflags,cpf_is_colon_para);
                 end
             end;
         until not try_to_consume(_COMMA);
         in_args:=prev_in_args;
         named_args_allowed:=old_named_args_allowed;
         parse_paras:=p2;
      end;


     function gen_c_style_operator(ntyp:tnodetype;p1,p2:tnode) : tnode;
       var
         hp    : tnode;
         hdef  : tdef;
         temp  : ttempcreatenode;
         newstatement : tstatementnode;
       begin
         { Properties are not allowed, because the write can
           be different from the read }
         if (nf_isproperty in p1.flags) then
           begin
             Message(type_e_variable_id_expected);
             { We can continue with the loading,
               it'll not create errors. Only the expected
               result can be wrong }
           end;

         hp:=p1;
         while assigned(hp) and
               (hp.nodetype in [derefn,subscriptn,vecn,typeconvn]) do
           hp:=tunarynode(hp).left;
         if not assigned(hp) then
           internalerror(200410121);
         if (hp.nodetype=calln) then
           begin
             typecheckpass(p1);
             result:=internalstatements(newstatement);
             hdef:=cpointerdef.getreusable(p1.resultdef);
             temp:=ctempcreatenode.create(hdef,sizeof(pint),tt_persistent,false);
             addstatement(newstatement,temp);
             addstatement(newstatement,cassignmentnode.create(ctemprefnode.create(temp),caddrnode.create_internal(p1)));
             addstatement(newstatement,cassignmentnode.create(
                 cderefnode.create(ctemprefnode.create(temp)),
                 caddnode.create(ntyp,
                     cderefnode.create(ctemprefnode.create(temp)),
                     p2)));
             addstatement(newstatement,ctempdeletenode.create(temp));
           end
         else
           result:=cassignmentnode.create(p1,caddnode.create(ntyp,p1.getcopy,p2));
       end;


     function statement_syssym(l : tinlinenumber) : tnode;
      var
        p1,p2,paras  : tnode;
        err,
        prev_in_args : boolean;
        def : tdef;
        exit_procinfo: tprocinfo;
      begin
        prev_in_args:=in_args;
        case l of

          in_new_x :
            begin
              if afterassignment or in_args then
               statement_syssym:=new_function
              else
               statement_syssym:=new_dispose_statement(true);
            end;

          in_dispose_x :
            begin
              statement_syssym:=new_dispose_statement(false);
            end;

          in_ord_x :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              consume(_RKLAMMER);
              p1:=geninlinenode(in_ord_x,false,p1);
              statement_syssym := p1;
            end;

          in_exit :
            begin
              statement_syssym:=nil;
              if try_to_consume(_LKLAMMER) then
                begin
                  if not (m_mac in current_settings.modeswitches) then
                    begin
                      if not(try_to_consume(_RKLAMMER)) then
                        begin
                          p1:=comp_expr([ef_accept_equal]);
                          consume(_RKLAMMER);
                          if not assigned(current_procinfo) or
                             (current_procinfo.procdef.proctypeoption in [potype_constructor,potype_destructor]) or
                             is_void(current_procinfo.procdef.returndef) then
                            begin
                              Message(parser_e_void_function);
                              { recovery }
                              p1.free;
                              p1:=nil;
                            end;
                        end
                      else
                        p1:=nil;
                    end
                  else
                    begin
                      { non local exit ? }
                      if current_procinfo.procdef.procsym.name<>pattern then
                        begin
                          exit_procinfo:=current_procinfo.parent;
                          while assigned(exit_procinfo) do
                            begin
                              if exit_procinfo.procdef.procsym.name=pattern then
                                break;
                              exit_procinfo:=exit_procinfo.parent;
                            end;
                          if assigned(exit_procinfo) then
                            begin
                              if not(assigned(exit_procinfo.nestedexitlabel)) then
                                begin
                                  include(current_procinfo.flags,pi_has_nested_exit);
                                  exclude(current_procinfo.procdef.procoptions,po_inline);

                                  exit_procinfo.nestedexitlabel:=clabelsym.create('$nestedexit');

                                  { the compiler is responsible to define this label }
                                  exit_procinfo.nestedexitlabel.defined:=true;
                                  exit_procinfo.nestedexitlabel.used:=true;

                                  exit_procinfo.nestedexitlabel.jumpbuf:=clocalvarsym.create('LABEL$_'+exit_procinfo.nestedexitlabel.name,vs_value,rec_jmp_buf,[]);
                                  exit_procinfo.procdef.localst.insert(exit_procinfo.nestedexitlabel);
                                  exit_procinfo.procdef.localst.insert(exit_procinfo.nestedexitlabel.jumpbuf);
                                end;

                              statement_syssym:=cgotonode.create(exit_procinfo.nestedexitlabel);
                              tgotonode(statement_syssym).labelsym:=exit_procinfo.nestedexitlabel;
                            end
                          else
                            Message(parser_e_macpas_exit_wrong_param);
                        end;
                      consume(_ID);
                      consume(_RKLAMMER);
                      p1:=nil;
                    end
                end
              else
                p1:=nil;
              if not assigned(statement_syssym) then
                statement_syssym:=cexitnode.create(p1);
            end;

          in_break :
            begin
              statement_syssym:=cbreaknode.create
            end;

          in_continue :
            begin
              statement_syssym:=ccontinuenode.create
            end;

          in_leave :
            begin
              if m_mac in current_settings.modeswitches then
                statement_syssym:=cbreaknode.create
              else
                begin
                  Message1(sym_e_id_not_found, orgpattern);
                  statement_syssym:=cerrornode.create;
                end;
            end;

          in_cycle :
            begin
              if m_mac in current_settings.modeswitches then
                statement_syssym:=ccontinuenode.create
              else
                begin
                  Message1(sym_e_id_not_found, orgpattern);
                  statement_syssym:=cerrornode.create;
                end;
            end;

          in_typeof_x :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              consume(_RKLAMMER);
              if p1.nodetype=typen then
                ttypenode(p1).allowed:=true;
              { Allow classrefdef, which is required for
                Typeof(self) in static class methods }
              if not(is_objc_class_or_protocol(p1.resultdef)) and
                 not(is_java_class_or_interface(p1.resultdef)) and
                 ((p1.resultdef.typ = objectdef) or
                  (assigned(current_procinfo) and
                   ((po_classmethod in current_procinfo.procdef.procoptions) or
                    (po_staticmethod in current_procinfo.procdef.procoptions)) and
                   (p1.resultdef.typ=classrefdef))) then
               statement_syssym:=geninlinenode(in_typeof_x,false,p1)
              else
               begin
                 Message(parser_e_class_id_expected);
                 p1.destroy;
                 statement_syssym:=cerrornode.create;
               end;
            end;

          in_sizeof_x,
          in_bitsizeof_x :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              consume(_RKLAMMER);
              if ((p1.nodetype<>typen) and

                 (
                  (is_object(p1.resultdef) and
                   (oo_has_constructor in tobjectdef(p1.resultdef).objectoptions)) or
                  is_open_array(p1.resultdef) or
                  is_array_of_const(p1.resultdef) or
                  is_open_string(p1.resultdef)
                 )) or
                 { keep the function call if it is a type parameter to avoid arithmetic errors due to constant folding }
                 (p1.resultdef.typ=undefineddef) then
                begin
                  statement_syssym:=geninlinenode(in_sizeof_x,false,p1);
                  { no packed bit support for these things }
                  if l=in_bitsizeof_x then
                    statement_syssym:=caddnode.create(muln,statement_syssym,cordconstnode.create(8,sizesinttype,true));
                end
              else
               begin
                 { allow helpers for SizeOf and BitSizeOf }
                 if p1.nodetype=typen then
                   ttypenode(p1).helperallowed:=true;
                 if (p1.resultdef.typ=forwarddef) then
                   Message1(type_e_type_is_not_completly_defined,tforwarddef(p1.resultdef).tosymname^);
                 if (l = in_sizeof_x) or
                    (not((p1.nodetype = vecn) and
                         is_packed_array(tvecnode(p1).left.resultdef)) and
                     not((p1.nodetype = subscriptn) and
                         is_packed_record_or_object(tsubscriptnode(p1).left.resultdef))) then
                   begin
                     statement_syssym:=cordconstnode.create(p1.resultdef.size,sizesinttype,true);
                     if (l = in_bitsizeof_x) then
                       statement_syssym:=caddnode.create(muln,statement_syssym,cordconstnode.create(8,sizesinttype,true));
                   end
                 else
                   statement_syssym:=cordconstnode.create(p1.resultdef.packedbitsize,sizesinttype,true);
                 { p1 not needed !}
                 p1.destroy;
               end;
            end;

          in_typeinfo_x,
          in_objc_encode_x,
          in_gettypekind_x,
          in_ismanagedtype_x:
            begin
              if (l in [in_typeinfo_x,in_gettypekind_x,in_ismanagedtype_x]) or
                 (m_objectivec1 in current_settings.modeswitches) then
                begin
                  consume(_LKLAMMER);
                  in_args:=true;
                  p1:=comp_expr([ef_accept_equal]);
                  { When reading a class type it is parsed as loadvmtaddrn,
                    typeinfo only needs the type so we remove the loadvmtaddrn }
                  if p1.nodetype=loadvmtaddrn then
                    begin
                      p2:=tloadvmtaddrnode(p1).left;
                      tloadvmtaddrnode(p1).left:=nil;
                      p1.free;
                      p1:=p2;
                    end;
                  if p1.nodetype=typen then
                  begin
                    ttypenode(p1).allowed:=true;
                    { allow helpers for TypeInfo }
                    if l in [in_typeinfo_x,in_gettypekind_x,in_ismanagedtype_x] then
                      ttypenode(p1).helperallowed:=true;
                  end;
    {              else
                    begin
                       p1.destroy;
                       p1:=cerrornode.create;
                       Message(parser_e_illegal_parameter_list);
                    end;}
                  consume(_RKLAMMER);
                  p2:=geninlinenode(l,false,p1);
                  statement_syssym:=p2;
                end
              else
                begin
                  Message1(sym_e_id_not_found, orgpattern);
                  statement_syssym:=cerrornode.create;
                end;
            end;

          in_aligned_x,
          in_unaligned_x,
          in_volatile_x:
            begin
              err:=false;
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              p2:=ccallparanode.create(p1,nil);
              p2:=geninlinenode(l,false,p2);
              consume(_RKLAMMER);
              statement_syssym:=p2;
            end;

          in_assigned_x :
            begin
              err:=false;
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              { When reading a class type it is parsed as loadvmtaddrn,
                typeinfo only needs the type so we remove the loadvmtaddrn }
              if p1.nodetype=loadvmtaddrn then
                begin
                  p2:=tloadvmtaddrnode(p1).left;
                  tloadvmtaddrnode(p1).left:=nil;
                  p1.free;
                  p1:=p2;
                end;
              if not codegenerror then
               begin
                 case p1.resultdef.typ of
                   procdef, { procvar }
                   pointerdef,
                   procvardef,
                   classrefdef : ;
                   objectdef :
                     if not is_implicit_pointer_object_type(p1.resultdef) then
                       begin
                         Message(parser_e_illegal_parameter_list);
                         err:=true;
                       end;
                   arraydef :
                     if not is_dynamic_array(p1.resultdef) then
                       begin
                         Message(parser_e_illegal_parameter_list);
                         err:=true;
                       end;
                   else
                     if p1.resultdef.typ<>undefineddef then
                       begin
                         Message(parser_e_illegal_parameter_list);
                         err:=true;
                       end;
                 end;
               end
              else
               err:=true;
              if not err then
               begin
                 p2:=ccallparanode.create(p1,nil);
                 p2:=geninlinenode(in_assigned_x,false,p2);
               end
              else
               begin
                 p1.free;
                 p2:=cerrornode.create;
               end;
              consume(_RKLAMMER);
              statement_syssym:=p2;
            end;

          in_addr_x :
            begin
              consume(_LKLAMMER);
              got_addrn:=true;
              p1:=factor(true,[]);
              { inside parentheses a full expression is allowed, see also tests\webtbs\tb27517.pp }
              if token<>_RKLAMMER then
                p1:=sub_expr(opcompare,[ef_accept_equal],p1);
              p1:=caddrnode.create(p1);
              got_addrn:=false;
              consume(_RKLAMMER);
              statement_syssym:=p1;
            end;

{$ifdef i8086}
          in_faraddr_x :
            begin
              consume(_LKLAMMER);
              got_addrn:=true;
              p1:=factor(true,[]);
              { inside parentheses a full expression is allowed, see also tests\webtbs\tb27517.pp }
              if token<>_RKLAMMER then
                p1:=sub_expr(opcompare,[ef_accept_equal],p1);
              p1:=geninlinenode(in_faraddr_x,false,p1);
              got_addrn:=false;
              consume(_RKLAMMER);
              statement_syssym:=p1;
            end;
{$endif i8086}

          in_ofs_x :
            begin
              if target_info.system in systems_managed_vm then
                message(parser_e_feature_unsupported_for_vm);
              consume(_LKLAMMER);
              got_addrn:=true;
              p1:=factor(true,[]);
              { inside parentheses a full expression is allowed, see also tests\webtbs\tb27517.pp }
              if token<>_RKLAMMER then
                p1:=sub_expr(opcompare,[ef_accept_equal],p1);
              p1:=caddrnode.create(p1);
              include(taddrnode(p1).addrnodeflags,anf_ofs);
              got_addrn:=false;
              { Ofs() returns a cardinal/qword, not a pointer }
              inserttypeconv_internal(p1,uinttype);
              consume(_RKLAMMER);
              statement_syssym:=p1;
            end;

          in_seg_x :
            begin
              consume(_LKLAMMER);
              got_addrn:=true;
              p1:=factor(true,[]);
              { inside parentheses a full expression is allowed, see also tests\webtbs\tb27517.pp }
              if token<>_RKLAMMER then
                p1:=sub_expr(opcompare,[ef_accept_equal],p1);
              p1:=geninlinenode(in_seg_x,false,p1);
              got_addrn:=false;
              consume(_RKLAMMER);
              statement_syssym:=p1;
            end;

          in_high_x,
          in_low_x :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              p2:=geninlinenode(l,false,p1);
              consume(_RKLAMMER);
              statement_syssym:=p2;
            end;

          in_succ_x,
          in_pred_x :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              p2:=geninlinenode(l,false,p1);
              consume(_RKLAMMER);
              statement_syssym:=p2;
            end;

          in_inc_x,
          in_dec_x :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              if try_to_consume(_COMMA) then
                p2:=ccallparanode.create(comp_expr([ef_accept_equal]),nil)
              else
                p2:=nil;
              p2:=ccallparanode.create(p1,p2);
              statement_syssym:=geninlinenode(l,false,p2);
              consume(_RKLAMMER);
            end;

          in_slice_x:
            begin
              if not(in_args) then
                begin
                  message(parser_e_illegal_slice);
                  consume(_LKLAMMER);
                  in_args:=true;
                  comp_expr([ef_accept_equal]).free;
                  if try_to_consume(_COMMA) then
                    comp_expr([ef_accept_equal]).free;
                  statement_syssym:=cerrornode.create;
                  consume(_RKLAMMER);
                end
              else
                begin
                  consume(_LKLAMMER);
                  in_args:=true;
                  p1:=comp_expr([ef_accept_equal]);
                  Consume(_COMMA);
                  if not(codegenerror) then
                    p2:=ccallparanode.create(comp_expr([ef_accept_equal]),nil)
                  else
                    p2:=cerrornode.create;
                  p2:=ccallparanode.create(p1,p2);
                  statement_syssym:=geninlinenode(l,false,p2);
                  consume(_RKLAMMER);
                end;
            end;

          in_initialize_x:
            begin
              statement_syssym:=inline_initialize;
            end;

          in_finalize_x:
            begin
              statement_syssym:=inline_finalize;
            end;

          in_copy_x:
            begin
              statement_syssym:=inline_copy;
            end;

          in_concat_x :
            begin
              statement_syssym:=inline_concat;
            end;

          in_read_x,
          in_readln_x,
          in_readstr_x:
            begin
              if try_to_consume(_LKLAMMER) then
               begin
                 paras:=parse_paras(false,false,_RKLAMMER);
                 consume(_RKLAMMER);
               end
              else
               paras:=nil;
              p1:=geninlinenode(l,false,paras);
              statement_syssym := p1;
            end;

          in_setlength_x:
            begin
              statement_syssym := inline_setlength;
            end;

          in_objc_selector_x:
            begin
              if (m_objectivec1 in current_settings.modeswitches) then
                begin
                  consume(_LKLAMMER);
                  in_args:=true;
                  { don't turn procsyms into calls (getaddr = true) }
                  p1:=factor(true,[]);
                  p2:=geninlinenode(l,false,p1);
                  consume(_RKLAMMER);
                  statement_syssym:=p2;
                end
              else
                begin
                  Message1(sym_e_id_not_found, orgpattern);
                  statement_syssym:=cerrornode.create;
                end;
            end;

          in_length_x:
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              p2:=geninlinenode(l,false,p1);
              consume(_RKLAMMER);
              statement_syssym:=p2;
            end;

          in_write_x,
          in_writeln_x,
          in_writestr_x :
            begin
              if try_to_consume(_LKLAMMER) then
               begin
                 paras:=parse_paras(true,false,_RKLAMMER);
                 consume(_RKLAMMER);
               end
              else
               paras:=nil;
              p1 := geninlinenode(l,false,paras);
              statement_syssym := p1;
            end;

          in_str_x_string :
            begin
              consume(_LKLAMMER);
              paras:=parse_paras(true,false,_RKLAMMER);
              consume(_RKLAMMER);
              p1 := geninlinenode(l,false,paras);
              statement_syssym := p1;
            end;

          in_val_x:
            Begin
              consume(_LKLAMMER);
              in_args := true;
              p1:= ccallparanode.create(comp_expr([ef_accept_equal]), nil);
              consume(_COMMA);
              p2 := ccallparanode.create(comp_expr([ef_accept_equal]),p1);
              if try_to_consume(_COMMA) then
                p2 := ccallparanode.create(comp_expr([ef_accept_equal]),p2);
              consume(_RKLAMMER);
              p2 := geninlinenode(l,false,p2);
              statement_syssym := p2;
            End;

          in_include_x_y,
          in_exclude_x_y :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              consume(_COMMA);
              p2:=comp_expr([ef_accept_equal]);
              statement_syssym:=geninlinenode(l,false,ccallparanode.create(p1,ccallparanode.create(p2,nil)));
              consume(_RKLAMMER);
            end;

          in_pack_x_y_z,
          in_unpack_x_y_z :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              consume(_COMMA);
              p2:=comp_expr([ef_accept_equal]);
              consume(_COMMA);
              paras:=comp_expr([ef_accept_equal]);
              statement_syssym:=geninlinenode(l,false,ccallparanode.create(p1,ccallparanode.create(p2,ccallparanode.create(paras,nil))));
              consume(_RKLAMMER);
            end;

          in_assert_x_y :
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              if try_to_consume(_COMMA) then
                 p2:=comp_expr([ef_accept_equal])
              else
               begin
                 { then insert an empty string }
                 p2:=cstringconstnode.createstr('');
               end;
              statement_syssym:=geninlinenode(l,false,ccallparanode.create(p1,ccallparanode.create(p2,nil)));
              consume(_RKLAMMER);
            end;
          in_get_frame:
            begin
              statement_syssym:=geninlinenode(l,false,nil);
            end;
(*
          in_get_caller_frame:
            begin
              if try_to_consume(_LKLAMMER) then
                begin
                  {You used to call get_caller_frame as get_caller_frame(get_frame),
                   however, as a stack frame may not exist, it does more harm than
                   good, so ignore it.}
                  in_args:=true;
                  p1:=comp_expr([ef_accept_equal]);
                  p1.destroy;
                  consume(_RKLAMMER);
                end;
              statement_syssym:=geninlinenode(l,false,nil);
            end;
*)
          in_default_x:
            begin
              consume(_LKLAMMER);
              in_args:=true;
              def:=nil;
              single_type(def,[stoAllowSpecialization]);
              statement_syssym:=cerrornode.create;
              if def<>generrordef then
                { "type expected" error is already done by single_type }
                if def.typ=forwarddef then
                  Message1(type_e_type_is_not_completly_defined,tforwarddef(def).tosymname^)
                else
                  begin
                    statement_syssym.free;
                    statement_syssym:=geninlinenode(in_default_x,false,ctypenode.create(def));
                  end;
              { consume the right bracket here for a nicer error position }
              consume(_RKLAMMER);
            end;

          in_setstring_x_y_z:
            begin
              statement_syssym := inline_setstring;
            end;

          in_delete_x_y_z:
            begin
              statement_syssym:=inline_delete;
            end;

          in_insert_x_y_z:
            begin
              statement_syssym:=inline_insert;
            end;
          in_const_eh_return_data_regno:
            begin
              consume(_LKLAMMER);
              in_args:=true;
              p1:=comp_expr([ef_accept_equal]);
              p2:=geninlinenode(l,true,p1);
              consume(_RKLAMMER);
              statement_syssym:=p2;
            end;
          else
            internalerror(15);

        end;
        in_args:=prev_in_args;
      end;


    function maybe_load_methodpointer(st:TSymtable;var p1:tnode):boolean;
      var
        pd: tprocdef;
      begin
        maybe_load_methodpointer:=false;
        if not assigned(p1) then
         begin
           case st.symtabletype of
             withsymtable :
               begin
                 if (st.defowner.typ=objectdef) then
                   p1:=tnode(twithsymtable(st).withrefnode).getcopy;
               end;
             ObjectSymtable,
             recordsymtable:
               begin
                 { Escape nested procedures }
                 if assigned(current_procinfo) then
                   begin
                     pd:=current_procinfo.get_normal_proc.procdef;
                     { We are calling from the static class method which has no self node }
                     if assigned(pd) and pd.no_self_node then
                       if st.symtabletype=recordsymtable then
                         p1:=ctypenode.create(pd.struct)
                       else
                         p1:=cloadvmtaddrnode.create(ctypenode.create(pd.struct))
                     else
                       p1:=load_self_node;
                   end
                 else
                   p1:=load_self_node;
                 { We are calling a member }
                 maybe_load_methodpointer:=true;
               end;
             else
               ;
           end;
         end;
      end;


    { reads the parameter for a subroutine call }
    procedure do_proc_call(sym:tsym;st:TSymtable;obj:tabstractrecorddef;getaddr:boolean;var again : boolean;var p1:tnode;callflags:tcallnodeflags;spezcontext:tspecializationcontext);
      var
         membercall,
         prevafterassn : boolean;
         i        : integer;
         para,p2  : tnode;
         currpara : tparavarsym;
         aprocdef : tprocdef;
      begin
         prevafterassn:=afterassignment;
         afterassignment:=false;
         membercall:=false;
         aprocdef:=nil;

         { when it is a call to a member we need to load the
           methodpointer first
         }
         membercall:=maybe_load_methodpointer(st,p1);

         { When we are expecting a procvar we also need
           to get the address in some cases }
         if assigned(getprocvardef) then
          begin
            if (block_type=bt_const) or
               getaddr then
             begin
               aprocdef:=Tprocsym(sym).Find_procdef_byprocvardef(getprocvardef);
               getaddr:=true;
             end
            else
             if ((m_tp_procvar in current_settings.modeswitches) or
                 (m_mac_procvar in current_settings.modeswitches)) and
                not(token in [_CARET,_POINT,_LKLAMMER]) then
              begin
                aprocdef:=Tprocsym(sym).Find_procdef_byprocvardef(getprocvardef);
                if assigned(aprocdef) then
                 getaddr:=true;
              end;
          end;

         { only need to get the address of the procedure? Check token because
           in the case of opening parenthesis is possible to get pointer to
           function result (lack of checking for token was the reason of
           tw10933.pp test failure) }
         if getaddr and (token<>_LKLAMMER) then
           begin
             { for now we don't support pointers to generic functions, but since
               this is only temporary we use a non translated message }
             if assigned(spezcontext) then
               begin
                 comment(v_error, 'Pointers to generics functions not implemented');
                 p1:=cerrornode.create;
                 spezcontext.free;
                 exit;
               end;

             { Retrieve info which procvar to call. For tp_procvar the
               aprocdef is already loaded above so we can reuse it }
             if not assigned(aprocdef) and
                assigned(getprocvardef) then
               aprocdef:=Tprocsym(sym).Find_procdef_byprocvardef(getprocvardef);

             { generate a methodcallnode or proccallnode }
             { we shouldn't convert things like @tcollection.load }
             p2:=cloadnode.create_procvar(sym,aprocdef,st);
             if assigned(p1) then
              begin
                { for loading methodpointer of an inherited function
                  we use self as instance and load the address of
                  the function directly and not through the vmt (PFV) }
                if (cnf_inherited in callflags) then
                  begin
                    include(tloadnode(p2).loadnodeflags,loadnf_inherited);
                    p1.free;
                    p1:=load_self_node;
                  end;
                if (p1.nodetype<>typen) then
                  tloadnode(p2).set_mp(p1)
                else
                  begin
                    typecheckpass(p1);
                    if (p1.resultdef.typ=objectdef) then
                      { so we can create the correct  method pointer again in case
                        this is a "objectprocvar:=@classname.method" expression }
                      tloadnode(p2).symtable:=tobjectdef(p1.resultdef).symtable
                    else
                      p1.free;
                  end;
              end;
             p1:=p2;

             { no postfix operators }
             again:=false;
           end
         else
           begin
             para:=nil;
             if anon_inherited then
              begin
                if not assigned(current_procinfo) then
                  internalerror(200305054);
                for i:=0 to current_procinfo.procdef.paras.count-1 do
                  begin
                    currpara:=tparavarsym(current_procinfo.procdef.paras[i]);
                    if not(vo_is_hidden_para in currpara.varoptions) then
                      begin
                        { inheritance by msgint? }
                        if assigned(srdef) then
                          { anonymous inherited via msgid calls only require a var parameter for
                            both methods, so we need some type casting here }
                          para:=ccallparanode.create(ctypeconvnode.create_internal(ctypeconvnode.create_internal(
                            cloadnode.create(currpara,currpara.owner),cformaltype),tparavarsym(tprocdef(srdef).paras[i]).vardef),
                          para)
                        else
                          para:=ccallparanode.create(cloadnode.create(currpara,currpara.owner),para);
                      end;
                 end;
              end
             else
              begin
                if try_to_consume(_LKLAMMER) then
                 begin
                   para:=parse_paras(false,false,_RKLAMMER);
                   consume(_RKLAMMER);
                 end;
              end;
             { indicate if this call was generated by a member and
               no explicit self is used, this is needed to determine
               how to handle a destructor call (PFV) }
             if membercall then
               include(callflags,cnf_member_call);
             if assigned(obj) then
               begin
                 if not (st.symtabletype in [ObjectSymtable,recordsymtable]) then
                   internalerror(200310031);
                 p1:=ccallnode.create(para,tprocsym(sym),obj.symtable,p1,callflags,spezcontext);
               end
             else
               p1:=ccallnode.create(para,tprocsym(sym),st,p1,callflags,spezcontext);
           end;
         afterassignment:=prevafterassn;
      end;


    procedure handle_procvar(pv : tprocvardef;var p2 : tnode);
      var
        hp,hp2 : tnode;
        hpp    : ^tnode;
        currprocdef : tprocdef;
      begin
        if not assigned(pv) then
         internalerror(200301121);
        if (m_tp_procvar in current_settings.modeswitches) or
           (m_mac_procvar in current_settings.modeswitches) then
         begin
           hp:=p2;
           hpp:=@p2;
           while assigned(hp) and
                 (hp.nodetype=typeconvn) do
            begin
              hp:=ttypeconvnode(hp).left;
              { save orignal address of the old tree so we can replace the node }
              hpp:=@hp;
            end;
           if (hp.nodetype=calln) and
              { a procvar can't have parameters! }
              not assigned(tcallnode(hp).left) then
            begin
              currprocdef:=tcallnode(hp).symtableprocentry.Find_procdef_byprocvardef(pv);
              if assigned(currprocdef) then
               begin
                 hp2:=cloadnode.create_procvar(tprocsym(tcallnode(hp).symtableprocentry),currprocdef,tcallnode(hp).symtableproc);
                 if (po_methodpointer in pv.procoptions) then
                   tloadnode(hp2).set_mp(tcallnode(hp).methodpointer.getcopy);
                 hp.free;
                 { replace the old callnode with the new loadnode }
                 hpp^:=hp2;
               end;
            end;
         end;
      end;


    { the following procedure handles the access to a property symbol }
    procedure handle_propertysym(propsym : tpropertysym;st : TSymtable;var p1 : tnode);
      var
         paras : tnode;
         p2    : tnode;
         membercall : boolean;
         callflags  : tcallnodeflags;
         propaccesslist : tpropaccesslist;
         sym: tsym;
      begin
         { property parameters? read them only if the property really }
         { has parameters                                             }
         paras:=nil;
         if (ppo_hasparameters in propsym.propoptions) then
           begin
             if try_to_consume(_LECKKLAMMER) then
               begin
                 paras:=parse_paras(false,false,_RECKKLAMMER);
                 consume(_RECKKLAMMER);
               end;
           end;
         { indexed property }
         if (ppo_indexed in propsym.propoptions) then
           begin
             p2:=cordconstnode.create(propsym.index,propsym.indexdef,true);
             paras:=ccallparanode.create(p2,paras);
           end;
         { we need only a write property if a := follows }
         { if not(afterassignment) and not(in_args) then }
         if token=_ASSIGNMENT then
           begin
              if propsym.getpropaccesslist(palt_write,propaccesslist) then
                begin
                   sym:=propaccesslist.firstsym^.sym;
                   case sym.typ of
                     procsym :
                       begin
                         callflags:=[];
                         { generate the method call }
                         membercall:=maybe_load_methodpointer(st,p1);
                         if membercall then
                           include(callflags,cnf_member_call);
                         p1:=ccallnode.create(paras,tprocsym(sym),st,p1,callflags,nil);
                         addsymref(sym);
                         paras:=nil;
                         consume(_ASSIGNMENT);
                         { read the expression }
                         if propsym.propdef.typ=procvardef then
                           getprocvardef:=tprocvardef(propsym.propdef);
                         p2:=comp_expr([ef_accept_equal]);
                         if assigned(getprocvardef) then
                           handle_procvar(getprocvardef,p2);
                         tcallnode(p1).left:=ccallparanode.create(p2,tcallnode(p1).left);
                         { mark as property, both the tcallnode and the real call block }
                         include(p1.flags,nf_isproperty);
                         getprocvardef:=nil;
                       end;
                     fieldvarsym :
                       begin
                         { generate access code }
                         if not handle_staticfield_access(sym,p1) then
                           propaccesslist_to_node(p1,st,propaccesslist);
                         include(p1.flags,nf_isproperty);
                         consume(_ASSIGNMENT);
                         { read the expression }
                         p2:=comp_expr([ef_accept_equal]);
                         p1:=cassignmentnode.create(p1,p2);
                      end
                    else
                      begin
                        p1:=cerrornode.create;
                        Message(parser_e_no_procedure_to_access_property);
                      end;
                  end;
                end
              else
                begin
                   p1:=cerrornode.create;
                   Message(parser_e_no_procedure_to_access_property);
                end;
           end
         else
           begin
              if propsym.getpropaccesslist(palt_read,propaccesslist) then
                begin
                   sym := propaccesslist.firstsym^.sym;
                   case sym.typ of
                     fieldvarsym :
                       begin
                         { generate access code }
                         if not handle_staticfield_access(sym,p1) then
                           propaccesslist_to_node(p1,st,propaccesslist);
                         include(p1.flags,nf_isproperty);
                         { catch expressions like "(propx):=1;" }
                         include(p1.flags,nf_no_lvalue);
                       end;
                     procsym :
                       begin
                          callflags:=[];
                          { generate the method call }
                          membercall:=maybe_load_methodpointer(st,p1);
                          if membercall then
                            include(callflags,cnf_member_call);
                          p1:=ccallnode.create(paras,tprocsym(sym),st,p1,callflags,nil);
                          paras:=nil;
                          include(p1.flags,nf_isproperty);
                          include(p1.flags,nf_no_lvalue);
                       end
                     else
                       begin
                          p1:=cerrornode.create;
                          Message(type_e_mismatch);
                       end;
                  end;
                end
              else
                begin
                   { error, no function to read property }
                   p1:=cerrornode.create;
                   Message(parser_e_no_procedure_to_access_property);
                end;
           end;
        { release paras if not used }
        if assigned(paras) then
         paras.free;
      end;


    { the ID token has to be consumed before calling this function }
    procedure do_member_read(structh:tabstractrecorddef;getaddr:boolean;sym:tsym;var p1:tnode;var again:boolean;callflags:tcallnodeflags;spezcontext:tspecializationcontext);
      var
        isclassref:boolean;
        isrecordtype:boolean;
        isobjecttype:boolean;
      begin
         if sym=nil then
           begin
              { pattern is still valid unless
              there is another ID just after the ID of sym }
              Message1(sym_e_id_no_member,orgpattern);
              p1.free;
              p1:=cerrornode.create;
              { try to clean up }
              spezcontext.free;
              again:=false;
           end
         else
           begin
              if assigned(p1) then
               begin
                 if not assigned(p1.resultdef) then
                   do_typecheckpass(p1);
                 isclassref:=(p1.resultdef.typ=classrefdef);
                 isrecordtype:=(p1.nodetype=typen) and (p1.resultdef.typ=recorddef);
                 isobjecttype:=(p1.nodetype=typen) and is_object(p1.resultdef);
               end
              else
                begin
                  isclassref:=false;
                  isrecordtype:=false;
                  isobjecttype:=false;
                end;

              if assigned(spezcontext) and not (sym.typ=procsym) then
                internalerror(2015091801);

              { we assume, that only procsyms and varsyms are in an object }
              { symbol table, for classes, properties are allowed          }
              case sym.typ of
                 procsym:
                   begin
                      do_proc_call(sym,sym.owner,structh,
                                   (getaddr and not(token in [_CARET,_POINT])),
                                   again,p1,callflags,spezcontext);
                      { we need to know which procedure is called }
                      do_typecheckpass(p1);
                      { calling using classref? }
                      if (
                            isclassref or
                            (
                              (isobjecttype or
                               isrecordtype) and
                              not (cnf_inherited in callflags)
                            )
                          ) and
                         (p1.nodetype=calln) and
                         assigned(tcallnode(p1).procdefinition) then
                        begin
                          if not isobjecttype then
                            begin
                              if not(po_classmethod in tcallnode(p1).procdefinition.procoptions) and
                                 not(tcallnode(p1).procdefinition.proctypeoption=potype_constructor) then
                                Message(parser_e_only_class_members_via_class_ref);
                            end
                          else
                            begin
                              { with objects, you can also do this:
                                  type
                                    tparent = object
                                      procedure test;
                                    end;

                                    tchild = object(tchild)
                                      procedure test;
                                    end;

                                    procedure tparent.test;
                                      begin
                                      end;

                                    procedure tchild.test;
                                      begin
                                        tparent.test;
                                      end;
                              }
                              if (tcallnode(p1).procdefinition.proctypeoption<>potype_constructor) and
                                 not(po_staticmethod in tcallnode(p1).procdefinition.procoptions) and
                                 (not assigned(current_structdef) or
                                  not def_is_related(current_structdef,structh)) then
                                begin
                                  p1.free;
                                  p1:=cerrornode.create;
                                  Message(parser_e_only_static_members_via_object_type);
                                  exit;
                                end;
                            end;
                          { in Java, constructors are not automatically inherited
                            -> calling a constructor from a parent type will create
                               an instance of that parent type! }
                          if is_javaclass(structh) and
                             (tcallnode(p1).procdefinition.proctypeoption=potype_constructor) and
                             (tcallnode(p1).procdefinition.owner.defowner<>find_real_class_definition(tobjectdef(structh),false)) then
                            Message(parser_e_java_no_inherited_constructor);
                          { Provide a warning if we try to create an instance of a
                            abstract class using the type name of that class. We
                            must not provide a warning if we use a "class of"
                            variable of that type though as we don't know the
                            type of the class }
                          if (tcallnode(p1).procdefinition.proctypeoption=potype_constructor) and
                              (oo_is_abstract in structh.objectoptions) and
                              assigned(tcallnode(p1).methodpointer) and
                              (tcallnode(p1).methodpointer.nodetype=loadvmtaddrn) then
                            Message1(type_w_instance_abstract_class,structh.RttiName);
                        end
                   end;
                 fieldvarsym:
                   begin
                      if not handle_staticfield_access(sym,p1) then
                        begin
                          if isclassref then
                            if assigned(p1) and
                              (
                                is_self_node(p1) or
                                (assigned(current_procinfo) and (current_procinfo.get_normal_proc.procdef.no_self_node) and
                                (current_procinfo.procdef.struct=structh))) then
                              Message(parser_e_only_class_members)
                            else
                              Message(parser_e_only_class_members_via_class_ref);
                          p1:=csubscriptnode.create(sym,p1);
                        end;
                   end;
                 propertysym:
                   begin
                      if isclassref and not (sp_static in sym.symoptions) then
                        Message(parser_e_only_class_members_via_class_ref);
                      handle_propertysym(tpropertysym(sym),sym.owner,p1);
                   end;
                 typesym:
                   begin
                     p1.free;
                     if try_to_consume(_LKLAMMER) then
                      begin
                        p1:=comp_expr([ef_accept_equal]);
                        consume(_RKLAMMER);
                        p1:=ctypeconvnode.create_explicit(p1,ttypesym(sym).typedef);
                      end
                     else
                       begin
                         p1:=ctypenode.create(ttypesym(sym).typedef);
                         if (is_class(ttypesym(sym).typedef) or
                             is_objcclass(ttypesym(sym).typedef) or
                             is_javaclass(ttypesym(sym).typedef)) and
                            not(block_type in [bt_type,bt_const_type,bt_var_type]) then
                           p1:=cloadvmtaddrnode.create(p1);
                       end;
                   end;
                 constsym:
                   begin
                     p1.free;
                     p1:=genconstsymtree(tconstsym(sym));
                   end;
                 staticvarsym:
                   begin
                     { typed constant is a staticvarsym
                       now they are absolutevarsym }
                     p1.free;
                     p1:=cloadnode.create(sym,sym.Owner);
                   end;
                 absolutevarsym:
                   begin
                     p1.free;
                     p1:=nil;
                     { typed constants are absolutebarsyms now to handle storage properly }
                     propaccesslist_to_node(p1,nil,tabsolutevarsym(sym).ref);
                   end
                 else
                   internalerror(16);
              end;
           end;
      end;


    function handle_specialize_inline_specialization(var srsym:tsym;out srsymtable:tsymtable;out spezcontext:tspecializationcontext):boolean;
      var
        spezdef : tdef;
        symname : tsymstr;
      begin
        result:=false;
        spezcontext:=nil;
        srsymtable:=nil;
        if not assigned(srsym) then
          message1(sym_e_id_no_member,orgpattern)
        else
          if not (srsym.typ in [typesym,procsym]) then
            message(type_e_type_id_expected)
          else
            begin
              if srsym.typ=typesym then
                spezdef:=ttypesym(srsym).typedef
              else
                spezdef:=tdef(tprocsym(srsym).procdeflist[0]);
              if (spezdef.typ=errordef) and (sp_generic_dummy in srsym.symoptions) then
                symname:=srsym.RealName
              else
                symname:='';
              spezdef:=generate_specialization_phase1(spezcontext,spezdef,symname);
              case spezdef.typ of
                errordef:
                  begin
                    spezcontext.free;
                    spezcontext:=nil;
                    srsym:=generrorsym;
                  end;
                procdef:
                  begin
                    if block_type<>bt_body then
                      begin
                        message(parser_e_illegal_expression);
                        spezcontext.free;
                        spezcontext:=nil;
                        srsym:=generrorsym;
                      end
                    else
                      begin
                        srsym:=tprocdef(spezdef).procsym;
                        srsymtable:=srsym.owner;
                        result:=true;
                      end;
                  end;
                objectdef,
                recorddef,
                arraydef,
                procvardef:
                  begin
                    spezdef:=generate_specialization_phase2(spezcontext,tstoreddef(spezdef),false,'');
                    spezcontext.free;
                    spezcontext:=nil;
                    if spezdef<>generrordef then
                      begin
                        srsym:=spezdef.typesym;
                        srsymtable:=srsym.owner;
                        check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg);
                        result:=true;
                      end;
                  end;
                else
                  internalerror(2015070302);
              end;
            end;
      end;


    function handle_factor_typenode(hdef:tdef;getaddr:boolean;var again:boolean;sym:tsym;typeonly:boolean):tnode;
      var
        srsym : tsym;
        srsymtable : tsymtable;
        erroroutresult,
        isspecialize : boolean;
        spezcontext : tspecializationcontext;
        savedfilepos : tfileposinfo;
      begin
         spezcontext:=nil;
         if sym=nil then
           sym:=hdef.typesym;
         { allow Ordinal(Value) for type declarations since it
           can be an enummeration declaration or a set lke:
           (OrdinalType(const1)..OrdinalType(const2) }
         if (not typeonly or is_ordinal(hdef)) and
            try_to_consume(_LKLAMMER) then
          begin
            result:=comp_expr([ef_accept_equal]);
            consume(_RKLAMMER);
            { type casts to class helpers aren't allowed }
            if is_objectpascal_helper(hdef) then
              Message(parser_e_no_category_as_types)
              { recovery by not creating a conversion node }
            else
              result:=ctypeconvnode.create_explicit(result,hdef);
          end
         { not LKLAMMER }
         else if (token=_POINT) and
            (is_object(hdef) or is_record(hdef)) then
           begin
             consume(_POINT);
             { handles calling methods declared in parent objects
               using "parentobject.methodname()" }
             if assigned(current_structdef) and
                not(getaddr) and
                def_is_related(current_structdef,hdef) then
               begin
                 result:=ctypenode.create(hdef);
                 ttypenode(result).typesym:=sym;
                 if not (m_delphi in current_settings.modeswitches) and
                     (block_type in inline_specialization_block_types) and
                     (token=_ID) and
                     (idtoken=_SPECIALIZE) then
                   begin
                     consume(_ID);
                     if token<>_ID then
                       message(type_e_type_id_expected);
                     isspecialize:=true;
                   end
                 else
                   isspecialize:=false;
                 { search also in inherited methods }
                 searchsym_in_class(tobjectdef(hdef),tobjectdef(current_structdef),pattern,srsym,srsymtable,[ssf_search_helper]);
                 if isspecialize then
                   begin
                     consume(_ID);
                     if not handle_specialize_inline_specialization(srsym,srsymtable,spezcontext) then
                       begin
                         result.free;
                         result:=cerrornode.create;
                       end;
                   end
                 else
                   begin
                     if assigned(srsym) then
                       check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg);
                     consume(_ID);
                   end;
                 if result.nodetype<>errorn then
                   do_member_read(tabstractrecorddef(hdef),false,srsym,result,again,[],spezcontext)
                 else
                   spezcontext.free;
               end
             else
              begin
                { handles:
                    * @TObject.Load
                    * static methods and variables }
                result:=ctypenode.create(hdef);
                ttypenode(result).typesym:=sym;
                if not (m_delphi in current_settings.modeswitches) and
                    (block_type in inline_specialization_block_types) and
                    (token=_ID) and
                    (idtoken=_SPECIALIZE) then
                  begin
                    consume(_ID);
                    if token<>_ID then
                      message(type_e_type_id_expected);
                    isspecialize:=true;
                  end
                else
                  isspecialize:=false;
                erroroutresult:=true;
                { TP allows also @TMenu.Load if Load is only }
                { defined in an anchestor class              }
                srsym:=search_struct_member(tabstractrecorddef(hdef),pattern);
                if isspecialize and assigned(srsym) then
                  begin
                    consume(_ID);
                    if handle_specialize_inline_specialization(srsym,srsymtable,spezcontext) then
                      erroroutresult:=false;
                  end
                else
                  begin
                    if assigned(srsym) then
                      begin
                        savedfilepos:=current_filepos;
                        consume(_ID);
                        if not (sp_generic_dummy in srsym.symoptions) or
                            not (token in [_LT,_LSHARPBRACKET]) then
                          check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg,savedfilepos)
                        else
                          result:=cspecializenode.create(result,getaddr,srsym);
                        erroroutresult:=false;
                      end
                    else
                      Message1(sym_e_id_no_member,orgpattern);
                  end;
                if erroroutresult then
                  begin
                    result.free;
                    result:=cerrornode.create;
                  end
                else
                  if result.nodetype<>specializen then
                    do_member_read(tabstractrecorddef(hdef),getaddr,srsym,result,again,[],spezcontext);
              end;
           end
         else
          begin
            { Normally here would be the check against the usage
              of "TClassHelper.Something", but as that might be
              used inside of system symbols like sizeof and
              typeinfo this check is put into ttypenode.pass_1
              (for "TClassHelper" alone) and tcallnode.pass_1
              (for "TClassHelper.Something") }
            { class reference ? }
            if is_class(hdef) or
               is_objcclass(hdef) or
               { Java interfaces also can have loadvmtaddrnodes,
                 e.g. for expressions such as JLClass(intftype) }
               is_java_class_or_interface(hdef) then
             begin
               if getaddr and (token=_POINT) and
                  not is_javainterface(hdef) then
                begin
                  consume(_POINT);
                  { allows @Object.Method }
                  { also allows static methods and variables }
                  result:=ctypenode.create(hdef);
                  ttypenode(result).typesym:=sym;
                  { TP allows also @TMenu.Load if Load is only }
                  { defined in an anchestor class              }
                  srsym:=search_struct_member(tobjectdef(hdef),pattern);
                  if assigned(srsym) then
                   begin
                     check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg);
                     consume(_ID);
                     { in case of @Object.Method1.Method2, we have to call
                       Method1 -> create a loadvmtaddr node as self instead of
                       a typen (the typenode would be changed to self of the
                       current method in case Method1 is a constructor, see
                       mantis #24844) }
                     if not(block_type in [bt_type,bt_const_type,bt_var_type]) and
                        (srsym.typ=procsym) and
                        (token in [_CARET,_POINT]) then
                       result:=cloadvmtaddrnode.create(result);
                     do_member_read(tabstractrecorddef(hdef),getaddr,srsym,result,again,[],nil);
                   end
                  else
                   begin
                     Message1(sym_e_id_no_member,orgpattern);
                     consume(_ID);
                   end;
                end
               else
                begin
                  result:=ctypenode.create(hdef);
                  ttypenode(result).typesym:=sym;
                  { For a type block we simply return only
                    the type. For all other blocks we return
                    a loadvmt node }
                  if not(block_type in [bt_type,bt_const_type,bt_var_type]) then
                    result:=cloadvmtaddrnode.create(result);
                end;
             end
            else
              begin
                result:=ctypenode.create(hdef);
                ttypenode(result).typesym:=sym;
              end;
          end;
      end;

{****************************************************************************
                               Factor
****************************************************************************}


    function real_const_node_from_pattern(s:string):tnode;
      var
        d : bestreal;
        code : integer;
        cur : currency;
      begin
        val(s,d,code);
        if code<>0 then
         begin
           Message(parser_e_error_in_real);
           d:=1.0;
         end;
        if current_settings.fputype=fpu_none then
          begin
            Message(parser_e_unsupported_real);
            result:=cerrornode.create;
            exit;
          end;
        if (current_settings.minfpconstprec=s32real) and
           (d = single(d)) then
          result:=crealconstnode.create(d,s32floattype)
        else if (current_settings.minfpconstprec=s64real) and
                (d = double(d)) then
          result:=crealconstnode.create(d,s64floattype)
        else
          result:=crealconstnode.create(d,pbestrealtype^);
        val(pattern,cur,code);
        if code=0 then
          trealconstnode(result).value_currency:=cur;
      end;

{---------------------------------------------
               PostFixOperators
---------------------------------------------}

    { returns whether or not p1 has been changed }
    function postfixoperators(var p1:tnode;var again:boolean;getaddr:boolean): boolean;

      { tries to avoid syntax errors after invalid qualifiers }
      procedure recoverconsume_postfixops;
       begin
         repeat
           if not try_to_consume(_CARET) then
             if try_to_consume(_POINT) then
               try_to_consume(_ID)
             else if try_to_consume(_LECKKLAMMER) then
               begin
                 repeat
                   comp_expr([ef_accept_equal]);
                 until not try_to_consume(_COMMA);
                 consume(_RECKKLAMMER);
               end
             else if try_to_consume(_LKLAMMER) then
               begin
                 repeat
                   comp_expr([ef_accept_equal]);
                 until not try_to_consume(_COMMA);
                 consume(_RKLAMMER);
               end
             else
               break;
         until false;
       end;


      procedure handle_variantarray;
       var
         p4 : tnode;
         newstatement : tstatementnode;
         tempresultvariant,
         temp    : ttempcreatenode;
         paras : tcallparanode;
         newblock : tnode;
         countindices : longint;
         elements: tfplist;
         arraydef: tdef;
       begin
         { create statements with call initialize the arguments and
           call fpc_dynarr_setlength }
         newblock:=internalstatements(newstatement);

         { store all indices in a temporary array }
         countindices:=0;
         elements:=tfplist.Create;
         repeat
           p4:=comp_expr([ef_accept_equal]);
           elements.add(p4);
         until not try_to_consume(_COMMA);

         arraydef:=carraydef.getreusable(s32inttype,elements.count);
         temp:=ctempcreatenode.create(arraydef,arraydef.size,tt_persistent,false);
         addstatement(newstatement,temp);
         for countindices:=0 to elements.count-1 do
           begin
             addstatement(newstatement,
               cassignmentnode.create(
                 cvecnode.create(
                   ctemprefnode.create(temp),
                   genintconstnode(countindices)
                 ),
                 tnode(elements[countindices])
               )
             );
           end;
         countindices:=elements.count;
         elements.free;

         consume(_RECKKLAMMER);

         { we need only a write access if a := follows }
         if token=_ASSIGNMENT then
           begin
             consume(_ASSIGNMENT);
             p4:=comp_expr([ef_accept_equal]);

             { create call to fpc_vararray_put }
             paras:=ccallparanode.create(cordconstnode.create
                   (countindices,s32inttype,true),
                ccallparanode.create(caddrnode.create_internal
               (cvecnode.create(ctemprefnode.create(temp),genintconstnode(0))),
                ccallparanode.create(ctypeconvnode.create_internal(p4,cvarianttype),
                ccallparanode.create(ctypeconvnode.create_internal(p1,cvarianttype)
                  ,nil))));

             addstatement(newstatement,ccallnode.createintern('fpc_vararray_put',paras));
             addstatement(newstatement,ctempdeletenode.create(temp));
           end
         else
           begin
             { create temp for result }
             tempresultvariant:=ctempcreatenode.create(cvarianttype,cvarianttype.size,tt_persistent,true);
             addstatement(newstatement,tempresultvariant);

             { create call to fpc_vararray_get }
             paras:=ccallparanode.create(cordconstnode.create
                   (countindices,s32inttype,true),
                ccallparanode.create(caddrnode.create_internal
               (ctemprefnode.create(temp)),
                ccallparanode.create(p1,
                ccallparanode.create(
                    ctemprefnode.create(tempresultvariant)
                  ,nil))));

             addstatement(newstatement,ccallnode.createintern('fpc_vararray_get',paras));
             addstatement(newstatement,ctempdeletenode.create(temp));
             { the last statement should return the value as
               location and type, this is done be referencing the
               temp and converting it first from a persistent temp to
               normal temp }
             addstatement(newstatement,ctempdeletenode.create_normal_temp(tempresultvariant));
             addstatement(newstatement,ctemprefnode.create(tempresultvariant));
           end;
         p1:=newblock;
       end;

      function parse_array_constructor(arrdef:tarraydef): tnode;
        var
          newstatement,assstatement:tstatementnode;
          arrnode:ttempcreatenode;
          temp2:ttempcreatenode;
          assnode:tnode;
          paracount:integer;
        begin
          result:=internalstatements(newstatement);
          { create temp for result }
          arrnode:=ctempcreatenode.create(arrdef,arrdef.size,tt_persistent,true);
          addstatement(newstatement,arrnode);

          paracount:=0;
          { check arguments and create an assignment calls }
          if try_to_consume(_LKLAMMER) then
            begin
              assnode:=internalstatements(assstatement);
              repeat
                { arr[i] := param_i }
                addstatement(assstatement,
                  cassignmentnode.create(
                    cvecnode.create(
                      ctemprefnode.create(arrnode),
                      cordconstnode.create(paracount,arrdef.rangedef,false)),
                    comp_expr([ef_accept_equal])));
                inc(paracount);
              until not try_to_consume(_COMMA);
              consume(_RKLAMMER);
            end
          else
            assnode:=nil;

          { get temp for array of lengths }
          temp2:=ctempcreatenode.create(sinttype,sinttype.size,tt_persistent,false);
          addstatement(newstatement,temp2);

          { one dimensional }
          addstatement(newstatement,cassignmentnode.create(
              ctemprefnode.create(temp2),
              cordconstnode.create
                 (paracount,s32inttype,true)));
          { create call to fpc_dynarr_setlength }
          addstatement(newstatement,ccallnode.createintern('fpc_dynarray_setlength',
              ccallparanode.create(caddrnode.create_internal
                    (ctemprefnode.create(temp2)),
                 ccallparanode.create(cordconstnode.create
                    (1,s32inttype,true),
                 ccallparanode.create(caddrnode.create_internal
                    (crttinode.create(tstoreddef(arrdef),initrtti,rdt_normal)),
                 ccallparanode.create(
                   ctypeconvnode.create_internal(
                     ctemprefnode.create(arrnode),voidpointertype),
                   nil))))

            ));
          { add assignment statememnts }
          addstatement(newstatement,ctempdeletenode.create(temp2));
          if assigned(assnode) then
            addstatement(newstatement,assnode);
          { the last statement should return the value as
            location and type, this is done be referencing the
            temp and converting it first from a persistent temp to
            normal temp }
          addstatement(newstatement,ctempdeletenode.create_normal_temp(arrnode));
          addstatement(newstatement,ctemprefnode.create(arrnode));
        end;

      function try_type_helper(var node:tnode;def:tdef):boolean;
        var
          srsym : tsym;
          srsymtable : tsymtable;
          n : tnode;
          newstatement : tstatementnode;
          temp : ttempcreatenode;
          extdef : tdef;
        begin
          result:=false;
          if (token=_ID) and (block_type in [bt_body,bt_general,bt_except,bt_const]) then
            begin
              if not assigned(def) then
                if node.nodetype=addrn then
                  { always use the pointer type for addr nodes as otherwise
                    we'll have an anonymous pointertype with no name }
                  def:=voidpointertype
                else
                  def:=node.resultdef;
              result:=search_objectpascal_helper(def,nil,pattern,srsym,srsymtable);
              if result then
                begin
                  if not (srsymtable.symtabletype=objectsymtable) or
                      not is_objectpascal_helper(tdef(srsymtable.defowner)) then
                    internalerror(2013011401);
                  { convert const node to temp node of the extended type }
                  if node.nodetype in (nodetype_const+[addrn]) then
                    begin
                      extdef:=tobjectdef(srsymtable.defowner).extendeddef;
                      newstatement:=nil;
                      n:=internalstatements(newstatement);
                      temp:=ctempcreatenode.create(extdef,extdef.size,tt_persistent,false);
                      addstatement(newstatement,temp);
                      addstatement(newstatement,cassignmentnode.create(ctemprefnode.create(temp),node));
                      addstatement(newstatement,ctempdeletenode.create_normal_temp(temp));
                      addstatement(newstatement,ctemprefnode.create(temp));
                      node:=n;
                      do_typecheckpass(node)
                    end;
                  check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg);
                  consume(_ID);
                  do_member_read(nil,getaddr,srsym,node,again,[],nil);
                end;
            end;
        end;

    var
     protsym  : tpropertysym;
     p2,p3  : tnode;
     srsym  : tsym;
     srsymtable : TSymtable;
     structh    : tabstractrecorddef;
     { shouldn't be used that often, so the extra overhead is ok to save
       stack space }
     dispatchstring : ansistring;
     autoderef,
     erroroutp1,
     allowspecialize,
     isspecialize,
     found,
     haderror,
     nodechanged    : boolean;
     calltype: tdispcalltype;
     valstr,expstr : string;
     intval : qword;
     code : integer;
     strdef : tdef;
     spezcontext : tspecializationcontext;
     old_current_filepos : tfileposinfo;
    label
     skipreckklammercheck,
     skippointdefcheck;
    begin
     result:=false;
     again:=true;
     while again do
      begin
        spezcontext:=nil;
        { we need the resultdef }
        do_typecheckpass_changed(p1,nodechanged);
        result:=result or nodechanged;

        if codegenerror then
         begin
           recoverconsume_postfixops;
           exit;
         end;
        { handle token }
        case token of
          _CARET:
             begin
               consume(_CARET);

               { support tp/mac procvar^ if the procvar returns a
                 pointer type }
               if ((m_tp_procvar in current_settings.modeswitches) or
                   (m_mac_procvar in current_settings.modeswitches)) and
                  (p1.resultdef.typ=procvardef) and
                  (tprocvardef(p1.resultdef).returndef.typ=pointerdef) then
                 begin
                   p1:=ccallnode.create_procvar(nil,p1);
                   typecheckpass(p1);
                 end;

               { iso file buf access? }
               if (m_isolike_io in current_settings.modeswitches) and
                 (p1.resultdef.typ=filedef) then
                 begin
                   case tfiledef(p1.resultdef).filetyp of
                     ft_text:
                       begin
                         p1:=cderefnode.create(ccallnode.createintern('fpc_getbuf_text',ccallparanode.create(p1,nil)));
                         typecheckpass(p1);
                       end;
                     ft_typed:
                       begin
                         p1:=cderefnode.create(ctypeconvnode.create_internal(ccallnode.createintern('fpc_getbuf_typedfile',ccallparanode.create(p1,nil)),
                           cpointerdef.getreusable(tfiledef(p1.resultdef).typedfiledef)));
                         typecheckpass(p1);
                       end;
                     else
                       internalerror(2019050530);
                   end;
                 end
               else if not(p1.resultdef.typ in [pointerdef,undefineddef]) then
                 begin
                    { ^ as binary operator is a problem!!!! (FK) }
                    again:=false;
                    Message(parser_e_invalid_qualifier);
                    recoverconsume_postfixops;
                    p1.destroy;
                    p1:=cerrornode.create;
                 end
               else
                 p1:=cderefnode.create(p1);
             end;

          _LECKKLAMMER:
             begin
               if is_class_or_interface_or_object(p1.resultdef) or
                  is_dispinterface(p1.resultdef) or
                  is_record(p1.resultdef) or
                  is_javaclass(p1.resultdef) then
                 begin
                   { default property }
                   protsym:=search_default_property(tabstractrecorddef(p1.resultdef));
                   if not(assigned(protsym)) then
                     begin
                        p1.destroy;
                        p1:=cerrornode.create;
                        again:=false;
                        message(parser_e_no_default_property_available);
                     end
                   else
                     begin
                       { The property symbol is referenced indirect }
                       protsym.IncRefCount;
                       handle_propertysym(protsym,protsym.owner,p1);
                     end;
                 end
               else
                 begin
                   consume(_LECKKLAMMER);
                   repeat
                     { in all of the cases below, p1 is changed }
                     case p1.resultdef.typ of
                       pointerdef:
                         begin
                            { support delphi autoderef }
                            if (tpointerdef(p1.resultdef).pointeddef.typ=arraydef) and
                               (m_autoderef in current_settings.modeswitches) then
                              p1:=cderefnode.create(p1);
                            p2:=comp_expr([ef_accept_equal]);
                            { Support Pbytevar[0..9] which returns array [0..9].}
                            if try_to_consume(_POINTPOINT) then
                              p2:=crangenode.create(p2,comp_expr([ef_accept_equal]));
                            p1:=cvecnode.create(p1,p2);
                         end;
                       variantdef:
                         begin
                           handle_variantarray;
                           { the RECKKLAMMER is already read }
                           goto skipreckklammercheck;
                         end;
                       stringdef :
                         begin
                           p2:=comp_expr([ef_accept_equal]);
                           { Support string[0..9] which returns array [0..9] of char.}
                           if try_to_consume(_POINTPOINT) then
                             p2:=crangenode.create(p2,comp_expr([ef_accept_equal]));
                           p1:=cvecnode.create(p1,p2);
                         end;
                       arraydef:
                         begin
                           p2:=comp_expr([ef_accept_equal]);
                           { support SEG:OFS for go32v2/msdos Mem[] }
                           if (target_info.system in [system_i386_go32v2,system_i386_watcom,system_i8086_msdos,system_i8086_win16,system_i8086_embedded]) and
                              (p1.nodetype=loadn) and
                              assigned(tloadnode(p1).symtableentry) and
                              assigned(tloadnode(p1).symtableentry.owner.name) and
                              (tloadnode(p1).symtableentry.owner.name^='SYSTEM') and
                              ((tloadnode(p1).symtableentry.name='MEM') or
                               (tloadnode(p1).symtableentry.name='MEMW') or
                               (tloadnode(p1).symtableentry.name='MEML')) then
                             begin
{$if defined(i8086)}
                               consume(_COLON);
                               inserttypeconv(p2,u16inttype);
                               inserttypeconv_internal(p2,u32inttype);
                               p3:=cshlshrnode.create(shln,p2,cordconstnode.create($10,s16inttype,false));
                               p2:=comp_expr([ef_accept_equal]);
                               inserttypeconv(p2,u16inttype);
                               inserttypeconv_internal(p2,u32inttype);
                               p2:=caddnode.create(addn,p2,p3);
                               case tloadnode(p1).symtableentry.name of
                                 'MEM': p2:=ctypeconvnode.create_internal(p2,bytefarpointertype);
                                 'MEMW': p2:=ctypeconvnode.create_internal(p2,wordfarpointertype);
                                 'MEML': p2:=ctypeconvnode.create_internal(p2,longintfarpointertype);
                                 else
                                   internalerror(2013053102);
                               end;
                               p1:=cderefnode.create(p2);
{$elseif defined(i386)}
                               if try_to_consume(_COLON) then
                                begin
                                  p3:=caddnode.create(muln,cordconstnode.create($10,s32inttype,false),p2);
                                  p2:=comp_expr([ef_accept_equal]);
                                  p2:=caddnode.create(addn,p2,p3);
                                  if try_to_consume(_POINTPOINT) then
                                    { Support mem[$a000:$0000..$07ff] which returns array [0..$7ff] of memtype.}
                                    p2:=crangenode.create(p2,caddnode.create(addn,comp_expr([ef_accept_equal]),p3.getcopy));
                                  p1:=cvecnode.create(p1,p2);
                                  include(tvecnode(p1).flags,nf_memseg);
                                  include(tvecnode(p1).flags,nf_memindex);
                                end
                               else
                                begin
                                  if try_to_consume(_POINTPOINT) then
                                    { Support mem[$80000000..$80000002] which returns array [0..2] of memtype.}
                                    p2:=crangenode.create(p2,comp_expr([ef_accept_equal]));
                                  p1:=cvecnode.create(p1,p2);
                                  include(tvecnode(p1).flags,nf_memindex);
                                end;
{$else}
                               internalerror(2013053101);
{$endif}
                             end
                           else
                             begin
                               if try_to_consume(_POINTPOINT) then
                                 { Support arrayvar[0..9] which returns array [0..9] of arraytype.}
                                 p2:=crangenode.create(p2,comp_expr([ef_accept_equal]));
                               p1:=cvecnode.create(p1,p2);
                             end;
                         end;
                       else
                         begin
                           if p1.resultdef.typ<>undefineddef then
                             Message(parser_e_invalid_qualifier);
                           p1.destroy;
                           p1:=cerrornode.create;
                           comp_expr([ef_accept_equal]);
                           again:=false;
                         end;
                     end;
                     do_typecheckpass(p1);
                   until not try_to_consume(_COMMA);
                   consume(_RECKKLAMMER);
                   { handle_variantarray eats the RECKKLAMMER and jumps here }
                 skipreckklammercheck:
                 end;
             end;

          _POINT :
             begin
               consume(_POINT);
               allowspecialize:=not (m_delphi in current_settings.modeswitches) and (block_type in inline_specialization_block_types);
               if allowspecialize and (token=_ID) and (idtoken=_SPECIALIZE) then
                 begin
                   //consume(_ID);
                   isspecialize:=true;
                 end
               else
                 isspecialize:=false;
               autoderef:=false;
               if (p1.resultdef.typ=pointerdef) and
                  (m_autoderef in current_settings.modeswitches) and
                  { don't auto-deref objc.id, because then the code
                    below for supporting id.anyobjcmethod isn't triggered }
                  (p1.resultdef<>objc_idtype) then
                 begin
                   p1:=cderefnode.create(p1);
                   do_typecheckpass(p1);
                   autoderef:=true;
                 end;
               { procvar.<something> can never mean anything so always
                 try to call it in case it returns a record/object/... }
               maybe_call_procvar(p1,false);

               if (p1.nodetype=ordconstn) and
                   not is_boolean(p1.resultdef) and
                   not is_enum(p1.resultdef) then
                 begin
                   { type helpers are checked first }
                   if (token=_ID) and try_type_helper(p1,nil) then
                     goto skippointdefcheck;
                   { only an "e" or "E" can follow an intconst with a ".", the
                     other case (another intconst) is handled by the scanner }
                   if (token=_ID) and (pattern[1]='E') then
                     begin
                       haderror:=false;
                       if length(pattern)>1 then
                         begin
                           expstr:=copy(pattern,2,length(pattern)-1);
                           val(expstr,intval,code);
                           if code<>0 then
                             begin
                               haderror:=true;
                               intval:=intval; // Hackfix the "var assigned but never used" note.
                             end;
                         end
                       else
                         expstr:='';
                       consume(token);
                       if tordconstnode(p1).value.signed then
                         str(tordconstnode(p1).value.svalue,valstr)
                       else
                         str(tordconstnode(p1).value.uvalue,valstr);
                       valstr:=valstr+'.0E';
                       if expstr='' then
                         case token of
                           _MINUS:
                             begin
                               consume(token);
                               if token=_INTCONST then
                                 begin
                                   valstr:=valstr+'-'+pattern;
                                   consume(token);
                                 end
                               else
                                 haderror:=true;
                             end;
                           _PLUS:
                             begin
                               consume(token);
                               if token=_INTCONST then
                                 begin
                                   valstr:=valstr+pattern;
                                   consume(token);
                                 end
                               else
                                 haderror:=true;
                             end;
                           _INTCONST:
                             begin
                               valstr:=valstr+pattern;
                               consume(_INTCONST);
                             end;
                           else
                             haderror:=true;
                         end
                       else
                         valstr:=valstr+expstr;
                       if haderror then
                         begin
                           Message(parser_e_error_in_real);
                           p2:=cerrornode.create;
                         end
                       else
                         p2:=real_const_node_from_pattern(valstr);
                       p1.free;
                       p1:=p2;
                       again:=false;
                       goto skippointdefcheck;
                     end
                   else
                     begin
                       { just convert the ordconst to a realconst }
                       p2:=crealconstnode.create(tordconstnode(p1).value,pbestrealtype^);
                       p1.free;
                       p1:=p2;
                       again:=false;
                       goto skippointdefcheck;
                     end;
                 end;

               if (p1.nodetype=stringconstn) and (token=_ID) then
                 begin
                   { the def of a string const is an array }
                   case tstringconstnode(p1).cst_type of
                     cst_conststring:
                       if cs_refcountedstrings in current_settings.localswitches then
                         if m_default_unicodestring in current_settings.modeswitches then
                           strdef:=cunicodestringtype
                         else
                           strdef:=cansistringtype
                       else
                         strdef:=cshortstringtype;
                     cst_shortstring:
                       strdef:=cshortstringtype;
                     cst_ansistring:
                       { use getansistringdef? }
                       strdef:=cansistringtype;
                     cst_widestring:
                       strdef:=cwidestringtype;
                     cst_unicodestring:
                       strdef:=cunicodestringtype;
                     cst_longstring:
                       { let's see when someone stumbles upon this...}
                       internalerror(201301111);
                   end;
                   if try_type_helper(p1,strdef) then
                     goto skippointdefcheck;
                 end;

               { this is skipped if label skippointdefcheck is used }
               case p1.resultdef.typ of
                 recorddef:
                   begin
                     if isspecialize or (token=_ID) then
                       begin
                         erroroutp1:=true;
                         srsym:=nil;
                         structh:=tabstractrecorddef(p1.resultdef);
                         if isspecialize then
                           begin
                             { consume the specialize }
                             consume(_ID);
                             if token<>_ID then
                               consume(_ID)
                             else
                               begin
                                 searchsym_in_record(structh,pattern,srsym,srsymtable);
                                 consume(_ID);
                                 if handle_specialize_inline_specialization(srsym,srsymtable,spezcontext) then
                                   erroroutp1:=false;
                               end;
                           end
                         else
                           begin
                             searchsym_in_record(structh,pattern,srsym,srsymtable);
                             if assigned(srsym) then
                               begin
                                 old_current_filepos:=current_filepos;
                                 consume(_ID);
                                 if not (sp_generic_dummy in srsym.symoptions) or
                                     not (token in [_LT,_LSHARPBRACKET]) then
                                   check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg,old_current_filepos)
                                 else
                                   p1:=cspecializenode.create(p1,getaddr,srsym);
                                 erroroutp1:=false;
                               end
                             else
                               begin
                                 Message1(sym_e_id_no_member,orgpattern);
                                 { try to clean up }
                                 consume(_ID);
                               end;
                           end;
                         if erroroutp1 then
                           begin
                             p1.free;
                             p1:=cerrornode.create;
                           end
                         else
                           if p1.nodetype<>specializen then
                             do_member_read(structh,getaddr,srsym,p1,again,[],spezcontext);
                       end
                     else
                     consume(_ID);
                   end;
                 enumdef:
                   begin
                     if token=_ID then
                       begin
                         srsym:=tsym(tenumdef(p1.resultdef).symtable.Find(pattern));
                         if assigned(srsym) and (srsym.typ=enumsym) and (p1.nodetype=typen) then
                           begin
                             p1.destroy;
                             check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg);
                             p1:=genenumnode(tenumsym(srsym));
                             consume(_ID);
                           end
                         else
                           if not try_type_helper(p1,nil) then
                             begin
                               p1.destroy;
                               Message1(sym_e_id_no_member,orgpattern);
                               p1:=cerrornode.create;
                               consume(_ID);
                             end;
                       end;
                   end;
                 arraydef:
                   begin
                     if is_dynamic_array(p1.resultdef) then
                       begin
                         if token=_ID then
                           begin
                             if not try_type_helper(p1,nil) then
                               begin
                                 if pattern='CREATE' then
                                   begin
                                     consume(_ID);
                                     p2:=parse_array_constructor(tarraydef(p1.resultdef));
                                     p1.destroy;
                                     p1:=p2;
                                   end
                                 else
                                   begin
                                     Message2(scan_f_syn_expected,'CREATE',pattern);
                                     p1.destroy;
                                     p1:=cerrornode.create;
                                     consume(_ID);
                                   end;
                               end;
                           end
                         else
                           begin
                             Message(parser_e_invalid_qualifier);
                             p1.destroy;
                             p1:=cerrornode.create;
                             consume(_ID);
                           end;
                       end
                     else
                       if (token<>_ID) or not try_type_helper(p1,nil) then
                         begin
                           Message(parser_e_invalid_qualifier);
                           p1.destroy;
                           p1:=cerrornode.create;
                           consume(_ID);
                         end;
                   end;
                  variantdef:
                    begin
                      { dispatch call? }
                      { lhs := v.ident[parameters] -> property get
                        lhs := v.ident(parameters) -> method call
                        v.ident[parameters] := rhs -> property put
                        v.ident(parameters) := rhs -> also property put }
                      if token=_ID then
                        begin
                          if not try_type_helper(p1,nil) then
                            begin
                              dispatchstring:=orgpattern;
                              consume(_ID);
                              calltype:=dct_method;
                              if try_to_consume(_LKLAMMER) then
                                begin
                                  p2:=parse_paras(false,true,_RKLAMMER);
                                  consume(_RKLAMMER);
                                end
                              else if try_to_consume(_LECKKLAMMER) then
                                begin
                                  p2:=parse_paras(false,true,_RECKKLAMMER);
                                  consume(_RECKKLAMMER);
                                  calltype:=dct_propget;
                                end
                              else
                                p2:=nil;
                              { property setter? }
                              if (token=_ASSIGNMENT) and not(afterassignment) then
                                begin
                                  consume(_ASSIGNMENT);
                                  { read the expression }
                                  p3:=comp_expr([ef_accept_equal]);
                                  { concat value parameter too }
                                  p2:=ccallparanode.create(p3,p2);
                                  p1:=translate_disp_call(p1,p2,dct_propput,dispatchstring,0,voidtype);
                                end
                              else
                              { this is only an approximation
                                setting useresult if not necessary is only a waste of time, no more, no less (FK) }
                              if afterassignment or in_args or (token<>_SEMICOLON) then
                                p1:=translate_disp_call(p1,p2,calltype,dispatchstring,0,cvarianttype)
                              else
                                p1:=translate_disp_call(p1,p2,calltype,dispatchstring,0,voidtype);
                            end;
                        end
                      else { Error }
                        Consume(_ID);
                     end;
                  classrefdef:
                    begin
                      erroroutp1:=true;
                      if token=_ID then
                        begin
                          srsym:=nil;
                          structh:=tobjectdef(tclassrefdef(p1.resultdef).pointeddef);
                          if isspecialize then
                            begin
                              { consume the specialize }
                              consume(_ID);
                              if token<>_ID then
                                consume(_ID)
                              else
                                begin
                                  searchsym_in_class(tobjectdef(structh),tobjectdef(structh),pattern,srsym,srsymtable,[ssf_search_helper]);
                                  consume(_ID);
                                  if handle_specialize_inline_specialization(srsym,srsymtable,spezcontext) then
                                    erroroutp1:=false;
                                end;
                            end
                          else
                            begin
                              searchsym_in_class(tobjectdef(structh),tobjectdef(structh),pattern,srsym,srsymtable,[ssf_search_helper]);
                              if assigned(srsym) then
                                begin
                                  old_current_filepos:=current_filepos;
                                  consume(_ID);
                                  if not (sp_generic_dummy in srsym.symoptions) or
                                      not (token in [_LT,_LSHARPBRACKET]) then
                                    check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg,old_current_filepos)
                                  else
                                    p1:=cspecializenode.create(p1,getaddr,srsym);
                                  erroroutp1:=false;
                                end
                              else
                                begin
                                  Message1(sym_e_id_no_member,orgpattern);
                                  { try to clean up }
                                  consume(_ID);
                                end;
                            end;
                          if erroroutp1 then
                            begin
                              p1.free;
                              p1:=cerrornode.create;
                            end
                          else
                            if p1.nodetype<>specializen then
                              do_member_read(structh,getaddr,srsym,p1,again,[],spezcontext);
                        end
                      else { Error }
                        Consume(_ID);
                    end;
                  objectdef:
                    begin
                      if isspecialize or (token=_ID) then
                        begin
                          erroroutp1:=true;
                          srsym:=nil;
                          structh:=tobjectdef(p1.resultdef);
                          if isspecialize then
                            begin
                              { consume the "specialize" }
                              consume(_ID);
                              if token<>_ID then
                                consume(_ID)
                              else
                                begin
                                  searchsym_in_class(tobjectdef(structh),tobjectdef(structh),pattern,srsym,srsymtable,[ssf_search_helper]);
                                  consume(_ID);
                                  if handle_specialize_inline_specialization(srsym,srsymtable,spezcontext) then
                                    erroroutp1:=false;
                                end;
                            end
                          else
                            begin
                              searchsym_in_class(tobjectdef(structh),tobjectdef(structh),pattern,srsym,srsymtable,[ssf_search_helper]);
                              if assigned(srsym) then
                                begin
                                   old_current_filepos:=current_filepos;
                                   consume(_ID);
                                   if not (sp_generic_dummy in srsym.symoptions) or
                                       not (token in [_LT,_LSHARPBRACKET]) then
                                     check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg,old_current_filepos)
                                   else
                                     p1:=cspecializenode.create(p1,getaddr,srsym);
                                   erroroutp1:=false;
                                end
                              else
                                begin
                                   Message1(sym_e_id_no_member,orgpattern);
                                   { try to clean up }
                                   consume(_ID);
                                end;
                            end;
                          if erroroutp1 then
                            begin
                              p1.free;
                              p1:=cerrornode.create;
                            end
                          else
                            if p1.nodetype<>specializen then
                              do_member_read(structh,getaddr,srsym,p1,again,[],spezcontext);
                        end
                      else { Error }
                        Consume(_ID);
                    end;
                  pointerdef:
                    begin
                      if (p1.resultdef=objc_idtype) then
                        begin
                          { objc's id type can be used to call any
                            Objective-C method of any Objective-C class
                            type that's currently in scope }
                          if search_objc_method(pattern,srsym,srsymtable) then
                            begin
                              consume(_ID);
                              do_proc_call(srsym,srsymtable,nil,
                                (getaddr and not(token in [_CARET,_POINT])),
                                again,p1,[cnf_objc_id_call],nil);
                              { we need to know which procedure is called }
                              do_typecheckpass(p1);
                            end
                          else
                            begin
                              consume(_ID);
                              Message(parser_e_methode_id_expected);
                            end;
                        end
                      else
                        begin
                          if not try_type_helper(p1,nil) then
                            begin
                              Message(parser_e_invalid_qualifier);
                              if tpointerdef(p1.resultdef).pointeddef.typ in [recorddef,objectdef,classrefdef] then
                                Message(parser_h_maybe_deref_caret_missing);
                            end;
                        end
                    end;
                  else
                    begin
                      if autoderef then
                        begin
                          { always try with the not dereferenced node }
                          p2:=tderefnode(p1).left;
                          found:=try_type_helper(p2,nil);
                          if found then
                            begin
                              tderefnode(p1).left:=nil;
                              p1.destroy;
                              p1:=p2;
                            end;
                        end
                      else
                        found:=try_type_helper(p1,nil);
                      if not found then
                        begin
                          if p1.resultdef.typ<>undefineddef then
                            Message(parser_e_invalid_qualifier);
                          p1.destroy;
                          p1:=cerrornode.create;
                          { Error }
                          consume(_ID);
                        end;
                    end;
               end;
               { processing an ordconstnode avoids the resultdef check }
               skippointdefcheck:
             end;

          else
            begin
              { is this a procedure variable ? }
              if assigned(p1.resultdef) and
                 (p1.resultdef.typ=procvardef) then
                begin
                  { Typenode for typecasting or expecting a procvar }
                  if (p1.nodetype=typen) or
                     (
                      assigned(getprocvardef) and
                      equal_defs(p1.resultdef,getprocvardef)
                     ) then
                    begin
                      if try_to_consume(_LKLAMMER) then
                        begin
                          p1:=comp_expr([ef_accept_equal]);
                          consume(_RKLAMMER);
                          p1:=ctypeconvnode.create_explicit(p1,p1.resultdef);
                        end
                      else
                        again:=false
                    end
                  else
                    begin
                      if try_to_consume(_LKLAMMER) then
                        begin
                          p2:=parse_paras(false,false,_RKLAMMER);
                          consume(_RKLAMMER);
                          p1:=ccallnode.create_procvar(p2,p1);
                          { proc():= is never possible }
                          if token=_ASSIGNMENT then
                            begin
                              Message(parser_e_illegal_expression);
                              p1.free;
                              p1:=cerrornode.create;
                              again:=false;
                            end;
                        end
                      else
                        again:=false;
                    end;
                end
              else
                again:=false;
             end;
        end;

        { we only try again if p1 was changed }
        if again or
           (p1.nodetype=errorn) then
          result:=true;
      end; { while again }
    end;

    function is_member_read(sym: tsym; st: tsymtable; var p1: tnode;
                            out memberparentdef: tdef): boolean;
      var
        hdef : tdef;
      begin
        result:=true;
        memberparentdef:=nil;

        case st.symtabletype of
          ObjectSymtable,
          recordsymtable:
            begin
              memberparentdef:=tdef(st.defowner);
              exit;
            end;
          WithSymtable:
            begin
              if assigned(p1) then
               internalerror(2007012002);

              hdef:=tnode(twithsymtable(st).withrefnode).resultdef;
              p1:=tnode(twithsymtable(st).withrefnode).getcopy;

              if not(hdef.typ in [objectdef,classrefdef]) then
                exit;

              if (hdef.typ=classrefdef) then
                hdef:=tclassrefdef(hdef).pointeddef;
              memberparentdef:=hdef;
            end;
          else
            result:=false;
        end;
      end;

  {$maxfpuregisters 0}

    function factor(getaddr:boolean;flags:texprflags) : tnode;

         {---------------------------------------------
                         Factor_read_id
         ---------------------------------------------}

       procedure factor_read_id(out p1:tnode;out again:boolean);

         function findwithsymtable : boolean;
           var
             hp : psymtablestackitem;
           begin
             result:=true;
             hp:=symtablestack.stack;
             while assigned(hp) do
               begin
                 if hp^.symtable.symtabletype=withsymtable then
                   exit;
                 hp:=hp^.next;
               end;
             result:=false;
           end;

         var
           srsym: tsym;
           srsymtable: TSymtable;
           hdef: tdef;
           pd: tprocdef;
           orgstoredpattern,
           storedpattern: string;
           callflags: tcallnodeflags;
           t : ttoken;
           consumeid,
           wasgenericdummy,
           allowspecialize,
           isspecialize,
           unit_found, tmpgetaddr: boolean;
           dummypos,
           tokenpos: tfileposinfo;
           spezcontext : tspecializationcontext;
           cufflags : tconsume_unitsym_flags;
         begin
           { allow post fix operators }
           again:=true;

           { preinitalize tokenpos }
           tokenpos:=current_filepos;
           p1:=nil;
           spezcontext:=nil;

           { avoid warning }
           fillchar(dummypos,sizeof(dummypos),0);

           allowspecialize:=not (m_delphi in current_settings.modeswitches) and
                            not (ef_had_specialize in flags) and
                            (block_type in inline_specialization_block_types);
           if allowspecialize and (token=_ID) and (idtoken=_SPECIALIZE) then
             begin
               consume(_ID);
               isspecialize:=true;
             end
           else
             isspecialize:=ef_had_specialize in flags;

           { first check for identifier }
           if token<>_ID then
             begin
               srsym:=generrorsym;
               srsymtable:=nil;
               consume(_ID);
               unit_found:=false;
             end
           else
             begin
               storedpattern:=pattern;
               orgstoredpattern:=orgpattern;
               { store the position of the token before consuming it }
               tokenpos:=current_filepos;
               consumeid:=true;
               srsym:=nil;
               if ef_check_attr_suffix in flags then
                 begin
                   if not (ef_type_only in flags) then
                     internalerror(2019063001);
                   consume(_ID);
                   consumeid:=false;
                   if token<>_POINT then
                     searchsym_type(storedpattern+custom_attribute_suffix,srsym,srsymtable);
                 end;
               if not assigned(srsym) then
                 begin
                   if ef_type_only in flags then
                     searchsym_type(storedpattern,srsym,srsymtable)
                   else
                     searchsym(storedpattern,srsym,srsymtable);
                 end;
               { handle unit specification like System.Writeln }
               if not isspecialize then
                 begin
                   cufflags:=[];
                   if consumeid then
                     include(cufflags,cuf_consume_id);
                   if allowspecialize then
                     include(cufflags,cuf_allow_specialize);
                   if ef_check_attr_suffix in flags then
                     include(cufflags,cuf_check_attr_suffix);
                   unit_found:=try_consume_unitsym(srsym,srsymtable,t,cufflags,isspecialize,pattern);
                   if unit_found then
                     consumeid:=true;
                 end
               else
                 begin
                   unit_found:=false;
                   t:=_ID;
                 end;
               if consumeid then
                 begin
                   storedpattern:=pattern;
                   orgstoredpattern:=orgpattern;
                   { store the position of the token before consuming it }
                   tokenpos:=current_filepos;
                   consume(t);
                 end;
               { named parameter support }
               found_arg_name:=false;

               if not(unit_found) and
                   not isspecialize and
                  named_args_allowed and
                  (token=_ASSIGNMENT) then
                  begin
                    found_arg_name:=true;
                    p1:=cstringconstnode.createstr(storedpattern);
                    consume(_ASSIGNMENT);
                    exit;
                  end;

               if isspecialize then
                 begin
                   if not assigned(srsym) then
                     begin
                       identifier_not_found(orgstoredpattern,tokenpos);
                       srsym:=generrorsym;
                       srsymtable:=nil;
                     end
                   else
                     begin
                       {$push}
                       {$warn 5036 off}
                       hdef:=generate_specialization_phase1(spezcontext,nil,nil,orgstoredpattern,dummypos);
                       {$pop}
                       if hdef=generrordef then
                         begin
                           spezcontext.free;
                           spezcontext:=nil;
                           srsym:=generrorsym;
                           srsymtable:=nil;
                         end
                       else
                         begin
                           if hdef.typ in [objectdef,recorddef,procvardef,arraydef] then
                             begin
                               hdef:=generate_specialization_phase2(spezcontext,tstoreddef(hdef),false,'');
                               spezcontext.free;
                               spezcontext:=nil;
                               if hdef<>generrordef then
                                 begin
                                   srsym:=hdef.typesym;
                                   srsymtable:=srsym.owner;
                                 end
                               else
                                 begin
                                   srsym:=generrorsym;
                                   srsymtable:=nil;
                                 end;
                             end
                           else
                             if hdef.typ=procdef then
                               begin
                                 if block_type<>bt_body then
                                   begin
                                     message(parser_e_illegal_expression);
                                     srsym:=generrorsym;
                                     srsymtable:=nil;
                                   end
                                 else
                                   begin
                                     srsym:=tprocdef(hdef).procsym;
                                     srsymtable:=srsym.owner;
                                   end;
                               end
                             else
                               internalerror(2015061204);
                         end;
                     end;
                 end;

               wasgenericdummy:=false;
               if assigned(srsym) and
                   (sp_generic_dummy in srsym.symoptions) and
                   (srsym.typ=typesym) and
                   (
                     (
                       (m_delphi in current_settings.modeswitches) and
                       not (token in [_LT, _LSHARPBRACKET]) and
                       (ttypesym(srsym).typedef.typ=undefineddef)
                     )
                     or
                     (
                       not (m_delphi in current_settings.modeswitches) and
                       not isspecialize and
                       (
                         not parse_generic or
                         not (
                           assigned(current_structdef) and
                           assigned(get_generic_in_hierarchy_by_name(srsym,current_structdef))
                         )
                       )
                     )
                   ) then
                 begin
                   srsym:=resolve_generic_dummysym(srsym.name);
                   if assigned(srsym) then
                     srsymtable:=srsym.owner
                   else
                     begin
                       srsymtable:=nil;
                       wasgenericdummy:=true;
                     end;
                 end;

               { check hints, but only if it isn't a potential generic symbol;
                 that is checked in sub_expr if it isn't a generic }
               if assigned(srsym) and
                   not (
                     (srsym.typ=typesym) and
                     (
                       (ttypesym(srsym).typedef.typ in [recorddef,objectdef,arraydef,procvardef,undefineddef]) or
                       (
                         (ttypesym(srsym).typedef.typ=errordef) and
                         (sp_generic_dummy in srsym.symoptions)
                       )
                     ) and
                     not (sp_generic_para in srsym.symoptions) and
                     (token in [_LT, _LSHARPBRACKET])
                   ) then
                 check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg);

               { if nothing found give error and return errorsym }
               if not assigned(srsym) or
                   { is this a generic dummy symbol? }
                   ((srsym.typ=typesym) and
                   assigned(ttypesym(srsym).typedef) and
                   (ttypesym(srsym).typedef.typ=undefineddef) and
                   not (sp_generic_para in srsym.symoptions) and
                   not (token in [_LT, _LSHARPBRACKET]) and
                   not (
                     { in non-Delphi modes the generic class' name without a
                       "specialization" or "<T>" may be used to identify the
                       current class }
                     (sp_generic_dummy in srsym.symoptions) and
                     assigned(current_structdef) and
                     (df_generic in current_structdef.defoptions) and
                     not (m_delphi in current_settings.modeswitches) and
                     assigned(get_generic_in_hierarchy_by_name(srsym,current_structdef))
                   )) and
                   { it could be a rename of a generic para }
                   { Note: if this generates false positives we'll need to
                           include a "basesym" to tsym to track the original
                           symbol }
                   not (sp_explicitrename in srsym.symoptions) then
                 begin
                   { if a generic is parsed and when we are inside an with block,
                     a symbol might not be defined }
                   if assigned(current_procinfo) and (df_generic in current_procinfo.procdef.defoptions) and
                      findwithsymtable then
                     begin
                       { create dummy symbol, it will be freed later on }
                       srsym:=tstoredsym.create(undefinedsym,'$undefinedsym');
                       srsymtable:=nil;
                     end
                   else
                     begin
                       if wasgenericdummy then
                         messagepos(tokenpos,parser_e_no_generics_as_types)
                       else
                         identifier_not_found(orgstoredpattern,tokenpos);
                       srsym:=generrorsym;
                       srsymtable:=nil;
                     end;
                 end;
             end;

           { Access to funcret or need to call the function? }
           if (srsym.typ in [absolutevarsym,localvarsym,paravarsym]) and
              (vo_is_funcret in tabstractvarsym(srsym).varoptions) and
              { result(x) is not allowed }
              not(vo_is_result in tabstractvarsym(srsym).varoptions) and
              (
               (token=_LKLAMMER) or
               (
                (([m_tp7,m_delphi,m_mac,m_iso,m_extpas] * current_settings.modeswitches) <> []) and
                (afterassignment or in_args)
               )
              ) then
            begin
              hdef:=tdef(srsym.owner.defowner);
              if assigned(hdef) and
                 (hdef.typ=procdef) then
                srsym:=tprocdef(hdef).procsym
              else
                begin
                  Message(parser_e_illegal_expression);
                  srsym:=generrorsym;
                end;
              srsymtable:=srsym.owner;
            end;

            begin
              case srsym.typ of
                absolutevarsym :
                  begin
                    if (tabsolutevarsym(srsym).abstyp=tovar) then
                      begin
                        p1:=nil;
                        propaccesslist_to_node(p1,nil,tabsolutevarsym(srsym).ref);
                        p1:=ctypeconvnode.create(p1,tabsolutevarsym(srsym).vardef);
                        include(p1.flags,nf_absolute);
                      end
                    else
                      p1:=cloadnode.create(srsym,srsymtable);
                  end;

                staticvarsym,
                localvarsym,
                paravarsym,
                fieldvarsym :
                  begin
                    { check if we are reading a field of an object/class/   }
                    { record. is_member_read() will deal with withsymtables }
                    { if needed.                                            }
                    p1:=nil;
                    if is_member_read(srsym,srsymtable,p1,hdef) then
                      begin
                        { if the field was originally found in an     }
                        { objectsymtable, it means it's part of self  }
                        { if only method from which it was called is  }
                        { not class static                            }
                        if (srsymtable.symtabletype in [ObjectSymtable,recordsymtable]) then
                          { if we are accessing a owner procsym from the nested }
                          { class we need to call it as a class member          }
                          if assigned(current_structdef) and
                              (((current_structdef<>hdef) and is_owned_by(current_structdef,hdef)) or
                               (sp_static in srsym.symoptions)) then
                            if srsymtable.symtabletype=recordsymtable then
                              p1:=ctypenode.create(hdef)
                            else
                              p1:=cloadvmtaddrnode.create(ctypenode.create(hdef))
                          else
                            begin
                              if assigned(current_procinfo) then
                                begin
                                  pd:=current_procinfo.get_normal_proc.procdef;
                                  if assigned(pd) and pd.no_self_node then
                                    p1:=cloadvmtaddrnode.create(ctypenode.create(pd.struct))
                                  else
                                    p1:=load_self_node;
                                end
                              else
                                p1:=load_self_node;
                            end;
                        { now, if the field itself is part of an objectsymtab }
                        { (it can be even if it was found in a withsymtable,  }
                        {  e.g., "with classinstance do field := 5"), then    }
                        { let do_member_read handle it                        }
                        if (srsym.owner.symtabletype in [ObjectSymtable,recordsymtable]) then
                          do_member_read(tabstractrecorddef(hdef),getaddr,srsym,p1,again,[],nil)
                        else
                          { otherwise it's a regular record subscript }
                          p1:=csubscriptnode.create(srsym,p1);
                      end
                    else
                      { regular non-field load }
                      p1:=cloadnode.create(srsym,srsymtable);
                  end;

                syssym :
                  begin
                    p1:=statement_syssym(tsyssym(srsym).number);
                  end;

                typesym :
                  begin
                    hdef:=ttypesym(srsym).typedef;
                    if not assigned(hdef) then
                     begin
                       again:=false;
                     end
                    else
                     begin
                       if (m_delphi in current_settings.modeswitches) and
                           (sp_generic_dummy in srsym.symoptions) and
                           (token in [_LT,_LSHARPBRACKET]) then
                         begin
                           if block_type in [bt_type,bt_const_type,bt_var_type] then
                             begin
                               if not handle_specialize_inline_specialization(srsym,srsymtable,spezcontext) or (srsym.typ=procsym) then
                                 begin
                                   spezcontext.free;
                                   p1:=cerrornode.create;
                                   if try_to_consume(_LKLAMMER) then
                                    begin
                                      parse_paras(false,false,_RKLAMMER);
                                      consume(_RKLAMMER);
                                    end;
                                 end
                               else
                                 begin
                                   if srsym.typ<>typesym then
                                     internalerror(2015071705);
                                   hdef:=ttypesym(srsym).typedef;
                                   p1:=handle_factor_typenode(hdef,getaddr,again,srsym,ef_type_only in flags);
                                 end;
                             end
                           else
                             p1:=cspecializenode.create(nil,getaddr,srsym)
                         end
                       else
                         begin
                           { We need to know if this unit uses Variants }
                           if ((hdef=cvarianttype) or (hdef=colevarianttype)) and
                              not(cs_compilesystem in current_settings.moduleswitches) then
                             include(current_module.moduleflags,mf_uses_variants);
                           p1:=handle_factor_typenode(hdef,getaddr,again,srsym,ef_type_only in flags);
                         end;
                     end;
                  end;

                enumsym :
                  begin
                    p1:=genenumnode(tenumsym(srsym));
                  end;

                constsym :
                  begin
                    if tconstsym(srsym).consttyp=constresourcestring then
                      begin
                        p1:=cloadnode.create(srsym,srsymtable);
                        do_typecheckpass(p1);
                        p1.resultdef:=getansistringdef;
                      end
                    else
                      p1:=genconstsymtree(tconstsym(srsym));
                  end;

                procsym :
                  begin
                    p1:=nil;
                    { check if it's a method/class method }
                    if is_member_read(srsym,srsymtable,p1,hdef) then
                      begin
                        { if we are accessing a owner procsym from the nested }
                        { class we need to call it as a class member          }
                        if (srsymtable.symtabletype in [ObjectSymtable,recordsymtable]) and
                          assigned(current_structdef) and (current_structdef<>hdef) and is_owned_by(current_structdef,hdef) then
                          p1:=cloadvmtaddrnode.create(ctypenode.create(hdef));
                        { not srsymtable.symtabletype since that can be }
                        { withsymtable as well                          }
                        if (srsym.owner.symtabletype in [ObjectSymtable,recordsymtable]) then
                          begin
                            do_member_read(tabstractrecorddef(hdef),getaddr,srsym,p1,again,[],spezcontext);
                            spezcontext:=nil;
                          end
                        else
                          { no procsyms in records (yet) }
                          internalerror(2007012006);
                      end
                    else
                      begin
                        { regular procedure/function call }
                        if not unit_found then
                          callflags:=[]
                        else
                          callflags:=[cnf_unit_specified];
                        { TP7 uglyness: @proc^ is parsed as (@proc)^,
                          but @notproc^ is parsed as @(notproc^) }
                        if m_tp_procvar in current_settings.modeswitches then
                          tmpgetaddr:=getaddr and not(token in [_POINT,_LECKKLAMMER])
                        else
                          tmpgetaddr:=getaddr and not(token in [_CARET,_POINT,_LECKKLAMMER]);
                        do_proc_call(srsym,srsymtable,nil,tmpgetaddr,
                                     again,p1,callflags,spezcontext);
                        spezcontext:=nil;
                      end;
                  end;

                propertysym :
                  begin
                    p1:=nil;
                    { property of a class/object? }
                    if is_member_read(srsym,srsymtable,p1,hdef) then
                      begin
                        if (srsymtable.symtabletype in [ObjectSymtable,recordsymtable]) then
                          { if we are accessing a owner procsym from the nested }
                          { class or from a static class method we need to call }
                          { it as a class member                                }
                          if (assigned(current_structdef) and (current_structdef<>hdef) and is_owned_by(current_structdef,hdef)) or
                             (assigned(current_procinfo) and current_procinfo.get_normal_proc.procdef.no_self_node) then
                            begin
                              p1:=ctypenode.create(hdef);
                              if not is_record(hdef) then
                                p1:=cloadvmtaddrnode.create(p1);
                            end
                          else
                            p1:=load_self_node;
                        { not srsymtable.symtabletype since that can be }
                        { withsymtable as well                          }
                        if (srsym.owner.symtabletype in [ObjectSymtable,recordsymtable]) then
                          do_member_read(tabstractrecorddef(hdef),getaddr,srsym,p1,again,[],nil)
                        else
                          { no propertysyms in records (yet) }
                          internalerror(2009111510);
                      end
                    else
                    { no method pointer }
                      begin
                        handle_propertysym(tpropertysym(srsym),srsymtable,p1);
                      end;
                  end;

                labelsym :
                  begin
                    { Support @label }
                    if getaddr then
                      begin
                        if srsym.owner<>current_procinfo.procdef.localst then
                          CGMessage(parser_e_label_outside_proc);
                        p1:=cloadnode.create(srsym,srsym.owner)
                      end
                    else
                      begin
                        consume(_COLON);
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
                        p1:=clabelnode.create(nil,tlabelsym(srsym));
                        tlabelsym(srsym).code:=p1;
                      end;
                  end;

                undefinedsym :
                  begin
                    p1:=cnothingnode.Create;
                    p1.resultdef:=cundefineddef.create(true);
                    { clean up previously created dummy symbol }
                    srsym.free;
                  end;

                errorsym :
                  begin
                    p1:=cerrornode.create;
                    if try_to_consume(_LKLAMMER) then
                     begin
                       parse_paras(false,false,_RKLAMMER);
                       consume(_RKLAMMER);
                     end;
                  end;

                else
                  begin
                    p1:=cerrornode.create;
                    Message(parser_e_illegal_expression);
                  end;
              end; { end case }

              if assigned(spezcontext) then
                internalerror(2015061207);

              if assigned(p1) and (p1.nodetype<>errorn) then
                p1.fileinfo:=tokenpos;
            end;
         end;

         {---------------------------------------------
                         Factor_Read_Set
         ---------------------------------------------}

         { Read a set between [] }
         function factor_read_set:tnode;
         var
           p1,p2 : tnode;
           lastp,
           buildp : tarrayconstructornode;
         begin
           buildp:=nil;
           lastp:=nil;
         { be sure that a least one arrayconstructn is used, also for an
           empty [] }
           if token=_RECKKLAMMER then
             buildp:=carrayconstructornode.create(nil,buildp)
           else
            repeat
              p1:=comp_expr([ef_accept_equal]);
              if try_to_consume(_POINTPOINT) then
                begin
                  p2:=comp_expr([ef_accept_equal]);
                  p1:=carrayconstructorrangenode.create(p1,p2);
                end;
               { insert at the end of the tree, to get the correct order }
             if not assigned(buildp) then
               begin
                 buildp:=carrayconstructornode.create(p1,nil);
                 lastp:=buildp;
               end
             else
               begin
                 lastp.right:=carrayconstructornode.create(p1,nil);
                 lastp:=tarrayconstructornode(lastp.right);
               end;
           { there could be more elements }
           until not try_to_consume(_COMMA);
           buildp.allow_array_constructor:=block_type in [bt_body,bt_except];
           factor_read_set:=buildp;
         end;

         function can_load_self_node: boolean;
         begin
           result:=false;
           if (block_type in [bt_const,bt_type,bt_const_type,bt_var_type]) or
              not assigned(current_structdef) or
              not assigned(current_procinfo) then
             exit;
           result:=not current_procinfo.get_normal_proc.procdef.no_self_node;
         end;


      {---------------------------------------------
                      Factor (Main)
      ---------------------------------------------}

      var
         l          : longint;
         ic         : int64;
         qc         : qword;
         p1         : tnode;
         code       : integer;
         srsym      : tsym;
         srsymtable : TSymtable;
         pd         : tprocdef;
         hclassdef  : tobjectdef;
         d          : bestreal;
         hs,hsorg   : string;
         hdef       : tdef;
         filepos    : tfileposinfo;
         callflags  : tcallnodeflags;
         idstr      : tidstring;
         spezcontext : tspecializationcontext;
         isspecialize,
         mightbegeneric,
         useself,
         dopostfix,
         again,
         updatefpos,
         nodechanged  : boolean;
      begin
        { can't keep a copy of p1 and compare pointers afterwards, because
          p1 may be freed and reallocated in the same place!  }
        dopostfix:=true;
        updatefpos:=false;
        p1:=nil;
        filepos:=current_tokenpos;
        again:=false;
        pd:=nil;
        isspecialize:=false;
        if token=_ID then
         begin
           again:=true;
           { Handle references to self }
           if (idtoken=_SELF) and can_load_self_node then
             begin
               p1:=load_self_node;
               consume(_ID);
               again:=true;
             end
           else
             factor_read_id(p1,again);

           if assigned(p1) then
            begin
              { factor_read_id will set the filepos to after the id,
                and in case of _SELF the filepos will already be the
                same as filepos (so setting it again doesn't hurt).  }
              p1.fileinfo:=filepos;
              filepos:=current_tokenpos;
            end;
           { handle post fix operators }
           if (p1.nodetype=specializen) then
             { post fix operators are handled after specialization }
             dopostfix:=false
           else
             if (m_delphi in current_settings.modeswitches) and
                 (block_type=bt_body) and
                 (token in [_LT,_LSHARPBRACKET]) then
               begin
                 if p1.nodetype=typen then
                   idstr:=ttypenode(p1).typesym.name
                 else
                   if (p1.nodetype=loadvmtaddrn) and
                       (tloadvmtaddrnode(p1).left.nodetype=typen) then
                     idstr:=ttypenode(tloadvmtaddrnode(p1).left).typesym.name
                   else
                     if (p1.nodetype=loadn) then
                       idstr:=tloadnode(p1).symtableentry.name
                     else
                       idstr:='';
                 { if this is the case then the postfix handling is done in
                   sub_expr if necessary }
                 dopostfix:=not could_be_generic(idstr);
               end;
           { TP7 uglyness: @proc^ is parsed as (@proc)^, but @notproc^ is parsed
             as @(notproc^) }
           if (m_tp_procvar in current_settings.modeswitches) and (token=_CARET) and
              getaddr and (p1.nodetype=loadn) and (tloadnode(p1).symtableentry.typ=procsym) then
             dopostfix:=false;
           { maybe an additional parameter instead of misusing hadspezialize? }
           if dopostfix and not (ef_had_specialize in flags) then
             updatefpos:=postfixoperators(p1,again,getaddr);
         end
        else
         begin
           updatefpos:=true;
           case token of
             _RETURN :
                begin
                  consume(_RETURN);
                  p1:=nil;
                  if not(token in [_SEMICOLON,_ELSE,_END]) then
                    begin
                      p1:=comp_expr([ef_accept_equal]);
                      if not assigned(current_procinfo) or
                         (current_procinfo.procdef.proctypeoption in [potype_constructor,potype_destructor]) or
                         is_void(current_procinfo.procdef.returndef) then
                        begin
                          Message(parser_e_void_function);
                          { recovery }
                          p1.free;
                          p1:=nil;
                        end;
                    end;
                  p1 := cexitnode.create(p1);
                end;
             _INHERITED :
               begin
                 again:=true;
                 consume(_INHERITED);
                 if assigned(current_procinfo) and
                    assigned(current_structdef) and
                    ((current_structdef.typ=objectdef) or
                     ((target_info.system in systems_jvm) and
                      (current_structdef.typ=recorddef)))then
                  begin
                    { for record helpers in mode Delphi "inherited" is not
                      allowed }
                    if is_objectpascal_helper(current_structdef) and
                        (m_delphi in current_settings.modeswitches) and
                        (tobjectdef(current_structdef).helpertype=ht_record) then
                      Message(parser_e_inherited_not_in_record);
                    if (current_structdef.typ=objectdef) then
                      begin
                        hclassdef:=tobjectdef(current_structdef).childof;
                        { Objective-C categories *replace* methods in the class
                          they extend, or add methods to it. So calling an
                          inherited method always calls the method inherited from
                          the parent of the extended class }
                        if is_objccategory(current_structdef) then
                          hclassdef:=hclassdef.childof;
                      end
                    else if target_info.system in systems_jvm then
                      hclassdef:=java_fpcbaserecordtype
                    else
                      internalerror(2012012401);
                    spezcontext:=nil;
                    { if inherited; only then we need the method with
                      the same name }
                    if token <> _ID then
                     begin
                       hs:=current_procinfo.procdef.procsym.name;
                       hsorg:=current_procinfo.procdef.procsym.realname;
                       anon_inherited:=true;
                       { For message methods we need to search using the message
                         number or string }
                       pd:=tprocdef(tprocsym(current_procinfo.procdef.procsym).ProcdefList[0]);
                       srdef:=nil;
                       if (po_msgint in pd.procoptions) then
                         searchsym_in_class_by_msgint(hclassdef,pd.messageinf.i,srdef,srsym,srsymtable)
                       else
                        if (po_msgstr in pd.procoptions) then
                          searchsym_in_class_by_msgstr(hclassdef,pd.messageinf.str^,srsym,srsymtable)
                       else
                       { helpers have their own ways of dealing with inherited }
                       if is_objectpascal_helper(current_structdef) then
                         searchsym_in_helper(tobjectdef(current_structdef),tobjectdef(current_structdef),hs,srsym,srsymtable,[ssf_has_inherited])
                       else
                         searchsym_in_class(hclassdef,current_structdef,hs,srsym,srsymtable,[ssf_search_helper]);
                     end
                    else
                     begin
                       if not (m_delphi in current_settings.modeswitches) and
                           (block_type in inline_specialization_block_types) and
                           (token=_ID) and
                           (idtoken=_SPECIALIZE) then
                         begin
                           consume(_ID);
                           if token<>_ID then
                             message(parser_e_methode_id_expected);
                           isspecialize:=true;
                         end
                       else
                         isspecialize:=false;
                       hs:=pattern;
                       hsorg:=orgpattern;
                       consume(_ID);
                       anon_inherited:=false;
                       { helpers have their own ways of dealing with inherited }
                       if is_objectpascal_helper(current_structdef) then
                         searchsym_in_helper(tobjectdef(current_structdef),tobjectdef(current_structdef),hs,srsym,srsymtable,[ssf_has_inherited])
                       else
                         searchsym_in_class(hclassdef,current_structdef,hs,srsym,srsymtable,[ssf_search_helper]);
                       if isspecialize and assigned(srsym) then
                         begin
                           if not handle_specialize_inline_specialization(srsym,srsymtable,spezcontext) then
                             srsym:=nil;
                         end;
                     end;
                    if assigned(srsym) then
                     begin
                       mightbegeneric:=(m_delphi in current_settings.modeswitches) and
                                         (token in [_LT,_LSHARPBRACKET]) and
                                         (sp_generic_dummy in srsym.symoptions);
                       { load the procdef from the inherited class and
                         not from self }
                       case srsym.typ of
                         typesym,
                         procsym:
                           begin
                             { typesym is only a valid choice if we're dealing
                               with a potential generic }
                             if (srsym.typ=typesym) and not mightbegeneric then
                               begin
                                 Message(parser_e_methode_id_expected);
                                 p1:=cerrornode.create;
                               end
                             else
                               begin
                                 useself:=false;
                                 if is_objectpascal_helper(current_structdef) then
                                   begin
                                     { for a helper load the procdef either from the
                                       extended type, from the parent helper or from
                                       the extended type of the parent helper
                                       depending on the def the found symbol belongs
                                       to }
                                     if (srsym.Owner.defowner.typ=objectdef) and
                                         is_objectpascal_helper(tobjectdef(srsym.Owner.defowner)) then
                                       if def_is_related(current_structdef,tdef(srsym.Owner.defowner)) and
                                           assigned(tobjectdef(current_structdef).childof) then
                                         hdef:=tobjectdef(current_structdef).childof
                                       else
                                         begin
                                           hdef:=tobjectdef(srsym.Owner.defowner).extendeddef;
                                           useself:=true;
                                         end
                                     else
                                       begin
                                         hdef:=tdef(srsym.Owner.defowner);
                                         useself:=true;
                                       end;
                                   end
                                 else
                                   hdef:=hclassdef;
                                 if (po_classmethod in current_procinfo.procdef.procoptions) or
                                    (po_staticmethod in current_procinfo.procdef.procoptions) then
                                   hdef:=cclassrefdef.create(hdef);
                                 if useself then
                                   begin
                                     p1:=ctypeconvnode.create_internal(load_self_node,hdef);
                                   end
                                 else
                                   begin
                                     p1:=ctypenode.create(hdef);
                                     { we need to allow helpers here }
                                     ttypenode(p1).helperallowed:=true;
                                   end;
                               end;
                           end;
                         propertysym:
                           ;
                         else
                           begin
                             Message(parser_e_methode_id_expected);
                             p1:=cerrornode.create;
                           end;
                       end;
                       if mightbegeneric then
                         begin
                           p1:=cspecializenode.create_inherited(p1,getaddr,srsym,hclassdef);
                         end
                       else
                         begin
                           if not isspecialize then
                             check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg);
                           callflags:=[cnf_inherited];
                           include(current_procinfo.flags,pi_has_inherited);
                           if anon_inherited then
                             include(callflags,cnf_anon_inherited);
                           do_member_read(hclassdef,getaddr,srsym,p1,again,callflags,spezcontext);
                         end;
                       if p1.nodetype=errorn then
                         spezcontext.free;
                     end
                    else
                     begin
                       if anon_inherited then
                        begin
                          { For message methods we need to call DefaultHandler }
                          if (po_msgint in pd.procoptions) or
                             (po_msgstr in pd.procoptions) then
                            begin
                              searchsym_in_class(hclassdef,hclassdef,'DEFAULTHANDLER',srsym,srsymtable,[ssf_search_helper]);
                              if not assigned(srsym) or
                                 (srsym.typ<>procsym) then
                                internalerror(200303171);
                              p1:=nil;
                              do_proc_call(srsym,srsym.owner,hclassdef,false,again,p1,[],nil);
                            end
                          else
                            begin
                              { we need to ignore the inherited; }
                              p1:=cnothingnode.create;
                            end;
                        end
                       else
                        begin
                          Message1(sym_e_id_no_member,hsorg);
                          p1:=cerrornode.create;
                        end;
                       again:=false;
                     end;
                    { turn auto inheriting off }
                    anon_inherited:=false;
                  end
                 else
                   begin
                     { in case of records we use a more clear error message }
                     if assigned(current_structdef) and
                         (current_structdef.typ=recorddef) then
                       Message(parser_e_inherited_not_in_record)
                     else
                       Message(parser_e_generic_methods_only_in_methods);
                     again:=false;
                     p1:=cerrornode.create;
                   end;
                 if p1.nodetype<>specializen then
                   postfixoperators(p1,again,getaddr);
               end;

             _INTCONST :
               begin
                 {Try first wether the value fits in an int64.}
                 val(pattern,ic,code);
                 if code=0 then
                   begin
                      consume(_INTCONST);
                      int_to_type(ic,hdef);
                      p1:=cordconstnode.create(ic,hdef,true);
                   end
                 else
                   begin
                     { try qword next }
                     val(pattern,qc,code);
                     if code=0 then
                       begin
                          consume(_INTCONST);
                          int_to_type(qc,hdef);
                          p1:=cordconstnode.create(qc,hdef,true);
                       end;
                   end;
                 if code<>0 then
                   begin
                     { finally float }
                     val(pattern,d,code);
                     if code<>0 then
                       begin
                          Message(parser_e_invalid_integer);
                          consume(_INTCONST);
                          l:=1;
                          p1:=cordconstnode.create(l,sinttype,true);
                       end
                     else
                       begin
                          consume(_INTCONST);
                          p1:=crealconstnode.create(d,pbestrealtype^);
                       end;
                   end
                 else
                   { the necessary range checking has already been done by val }
                   tordconstnode(p1).rangecheck:=false;
                 if token=_POINT then
                   begin
                     again:=true;
                     postfixoperators(p1,again,getaddr);
                   end;
               end;

             _REALNUMBER :
               begin
                 p1:=real_const_node_from_pattern(pattern);
                 consume(_REALNUMBER);
                 if token=_POINT then
                   begin
                     again:=true;
                     postfixoperators(p1,again,getaddr);
                   end;
               end;

             _STRING :
               begin
                 string_dec(hdef,true);
                 { STRING can be also a type cast }
                 if try_to_consume(_LKLAMMER) then
                  begin
                    p1:=comp_expr([ef_accept_equal]);
                    consume(_RKLAMMER);
                    p1:=ctypeconvnode.create_explicit(p1,hdef);
                    { handle postfix operators here e.g. string(a)[10] }
                    again:=true;
                    postfixoperators(p1,again,getaddr);
                  end
                 else
                   begin
                     p1:=ctypenode.create(hdef);
                     if token=_POINT then
                       begin
                         again:=true;
                         { handle type helpers here }
                         postfixoperators(p1,again,getaddr);
                       end;
                   end;
               end;

             _FILE :
               begin
                 hdef:=cfiletype;
                 consume(_FILE);
                 { FILE can be also a type cast }
                 if try_to_consume(_LKLAMMER) then
                  begin
                    p1:=comp_expr([ef_accept_equal]);
                    consume(_RKLAMMER);
                    p1:=ctypeconvnode.create_explicit(p1,hdef);
                    { handle postfix operators here e.g. string(a)[10] }
                    again:=true;
                    postfixoperators(p1,again,getaddr);
                  end
                 else
                  begin
                    p1:=ctypenode.create(hdef);
                  end;
               end;

             _CSTRING :
               begin
                 p1:=cstringconstnode.createpchar(ansistring2pchar(cstringpattern),length(cstringpattern),nil);
                 consume(_CSTRING);
                 if token in postfixoperator_tokens then
                   begin
                     again:=true;
                     postfixoperators(p1,again,getaddr);
                   end;
               end;

             _CCHAR :
               begin
                 p1:=cordconstnode.create(ord(pattern[1]),cansichartype,true);
                 consume(_CCHAR);
                 if token=_POINT then
                   begin
                     again:=true;
                     postfixoperators(p1,again,getaddr);
                   end;
               end;

             _CWSTRING:
               begin
                 if getlengthwidestring(patternw)=1 then
                   p1:=cordconstnode.create(ord(getcharwidestring(patternw,0)),cwidechartype,true)
                 else
                   p1:=cstringconstnode.createunistr(patternw);
                 consume(_CWSTRING);
                 if token in postfixoperator_tokens then
                   begin
                     again:=true;
                     postfixoperators(p1,again,getaddr);
                   end;
               end;

             _CWCHAR:
               begin
                 p1:=cordconstnode.create(ord(getcharwidestring(patternw,0)),cwidechartype,true);
                 consume(_CWCHAR);
                 if token=_POINT then
                   begin
                     again:=true;
                     postfixoperators(p1,again,getaddr);
                   end;
               end;

             _KLAMMERAFFE :
               begin
                 consume(_KLAMMERAFFE);
                 got_addrn:=true;
                 { support both @<x> and @(<x>) }
                 if try_to_consume(_LKLAMMER) then
                  begin
                    p1:=factor(true,[]);
                    { inside parentheses a full expression is allowed, see also tests\webtbs\tb27517.pp }
                    if token<>_RKLAMMER then
                      p1:=sub_expr(opcompare,[ef_accept_equal],p1);
                    consume(_RKLAMMER);
                  end
                 else
                  p1:=factor(true,[]);
                 if (token in postfixoperator_tokens) and
                   { TP7 uglyness: @proc^ is parsed as (@proc)^, but @notproc^
                     is parsed as @(notproc^) }
                    not
                    (
                     (m_tp_procvar in current_settings.modeswitches) and
                     (token=_CARET) and (p1.nodetype=loadn) and (tloadnode(p1).symtableentry.typ=procsym)
                    )
                   then
                  begin
                    again:=true;
                    postfixoperators(p1,again,getaddr);
                  end;
                 got_addrn:=false;
                 p1:=caddrnode.create(p1);
                 p1.fileinfo:=filepos;
                 if cs_typed_addresses in current_settings.localswitches then
                   include(taddrnode(p1).addrnodeflags,anf_typedaddr);
                 { Store the procvar that we are expecting, the
                   addrn will use the information to find the correct
                   procdef or it will return an error }
                 if assigned(getprocvardef) and
                    (taddrnode(p1).left.nodetype = loadn) then
                   taddrnode(p1).getprocvardef:=getprocvardef;
                 if (token in postfixoperator_tokens) then
                  begin
                    again:=true;
                    postfixoperators(p1,again,getaddr);
                  end;
               end;

             _LKLAMMER :
               begin
                 consume(_LKLAMMER);
                 p1:=comp_expr([ef_accept_equal]);
                 consume(_RKLAMMER);
                 { it's not a good solution
                   but (a+b)^ makes some problems  }
                 if token in postfixoperator_tokens then
                  begin
                    again:=true;
                    postfixoperators(p1,again,getaddr);
                  end;
               end;

             _LECKKLAMMER :
               begin
                 consume(_LECKKLAMMER);
                 p1:=factor_read_set;
                 consume(_RECKKLAMMER);
               end;

             _PLUS :
               begin
                 consume(_PLUS);
                 p1:=factor(false,[]);
                 p1:=cunaryplusnode.create(p1);
               end;

             _MINUS :
               begin
                 consume(_MINUS);
                 if (token = _INTCONST) and not(m_isolike_unary_minus in current_settings.modeswitches) then
                    begin
                      { ugly hack, but necessary to be able to parse }
                      { -9223372036854775808 as int64 (JM)           }
                      pattern := '-'+pattern;
                      p1:=sub_expr(oppower,[],nil);
                      {  -1 ** 4 should be - (1 ** 4) and not
                         (-1) ** 4
                         This was the reason of tw0869.pp test failure PM }
                      if p1.nodetype=starstarn then
                        begin
                          if tbinarynode(p1).left.nodetype=ordconstn then
                            begin
                              tordconstnode(tbinarynode(p1).left).value:=-tordconstnode(tbinarynode(p1).left).value;
                              p1:=cunaryminusnode.create(p1);
                            end
                          else if tbinarynode(p1).left.nodetype=realconstn then
                            begin
                              trealconstnode(tbinarynode(p1).left).value_real:=-trealconstnode(tbinarynode(p1).left).value_real;
                              trealconstnode(tbinarynode(p1).left).value_currency:=-trealconstnode(tbinarynode(p1).left).value_currency;
                              p1:=cunaryminusnode.create(p1);
                            end
                          else
                            internalerror(20021029);
                        end;
                    end
                 else
                   begin
                     if m_isolike_unary_minus in current_settings.modeswitches then
                       p1:=sub_expr(opmultiply,[],nil)
                     else
                       p1:=sub_expr(oppower,[],nil);

                     p1:=cunaryminusnode.create(p1);
                   end;
               end;

             _OP_NOT :
               begin
                 consume(_OP_NOT);
                 p1:=factor(false,[]);
                 p1:=cnotnode.create(p1);
               end;

             _NIL :
               begin
                 consume(_NIL);
                 p1:=cnilnode.create;
                 { It's really ugly code nil^, but delphi allows it }
                 if token in [_CARET,_POINT] then
                  begin
                    again:=true;
                    postfixoperators(p1,again,getaddr);
                  end;
               end;
             _OBJCPROTOCOL:
               begin
                 { The @protocol keyword is used in two ways in Objective-C:
                     1) to declare protocols (~ Object Pascal interfaces)
                     2) to obtain the metaclass (~ Object Pascal) "class of")
                        of a declared protocol
                   This code is for handling the second case. Because of 1),
                   we cannot simply use a system unit symbol.
                 }
                 consume(_OBJCPROTOCOL);
                 consume(_LKLAMMER);
                 p1:=factor(false,[]);
                 consume(_RKLAMMER);
                 p1:=cinlinenode.create(in_objc_protocol_x,false,p1);
               end;

             else
               begin
                 Message(parser_e_illegal_expression);
                 p1:=cerrornode.create;
                 { recover }
                 consume(token);
               end;
           end;
        end;

        { generate error node if no node is created }
        if not assigned(p1) then
         begin
{$ifdef EXTDEBUG}
           Comment(V_Warning,'factor: p1=nil');
{$endif}
           p1:=cerrornode.create;
           updatefpos:=true;
         end;

        { get the resultdef for the node if nothing stops us }
        if (not assigned(p1.resultdef)) and dopostfix then
          begin
            do_typecheckpass_changed(p1,nodechanged);
            updatefpos:=updatefpos or nodechanged;
          end;

        if assigned(p1) and
           updatefpos then
          p1.fileinfo:=filepos;
        factor:=p1;
      end;
  {$maxfpuregisters default}

    procedure post_comp_expr_gendef(var def: tdef);
      var
        p1 : tnode;
        again : boolean;
      begin
        if not assigned(def) then
          internalerror(2011053001);
        again:=false;
        { handle potential typecasts, etc }
        p1:=handle_factor_typenode(def,false,again,nil,false);
        { parse postfix operators }
        postfixoperators(p1,again,false);
        if assigned(p1) and (p1.nodetype=typen) then
          def:=ttypenode(p1).typedef
        else
          def:=generrordef;
      end;

{****************************************************************************
                             Sub_Expr
****************************************************************************}
    function sub_expr(pred_level:Toperator_precedence;flags:texprflags;factornode:tnode):tnode;
    {Reads a subexpression while the operators are of the current precedence
     level, or any higher level. Replaces the old term, simpl_expr and
     simpl2_expr.}

      function istypenode(n:tnode):boolean;inline;
      { Checks whether the given node is a type node or a VMT node containing a
        typenode. This is used in the code for inline specializations in the
        _LT branch below }
        begin
          result:=assigned(n) and
                    (
                      (n.nodetype=typen) or
                      (
                        (n.nodetype=loadvmtaddrn) and
                        (tloadvmtaddrnode(n).left.nodetype=typen)
                      )
                    );
        end;

      function gettypedef(n:tnode):tdef;inline;
      { This returns the typedef that belongs to the given typenode or
        loadvmtaddrnode. n must not be Nil! }
        begin
          if n.nodetype=typen then
            result:=ttypenode(n).typedef
          else
            result:=ttypenode(tloadvmtaddrnode(n).left).typedef;
        end;

      function gettypedef(sym:tsym):tdef;inline;
        begin
          result:=nil;
          case sym.typ of
            typesym:
              result:=ttypesym(sym).typedef;
            procsym:
              result:=tdef(tprocsym(sym).procdeflist[0]);
            else
              internalerror(2015092701);
          end;
        end;

      function getgenericsym(n:tnode;out srsym:tsym):boolean;
        var
          srsymtable : tsymtable;
        begin
          srsym:=nil;
          case n.nodetype of
            typen:
              srsym:=ttypenode(n).typedef.typesym;
            loadvmtaddrn:
              srsym:=ttypenode(tloadvmtaddrnode(n).left).typedef.typesym;
            loadn:
              if not searchsym_with_symoption(tloadnode(n).symtableentry.Name,srsym,srsymtable,sp_generic_dummy) then
                srsym:=nil;
            specializen:
              srsym:=tspecializenode(n).sym;
            { TODO : handle const nodes }
            else
              ;
          end;
          result:=assigned(srsym);
        end;

      function generate_inline_specialization(gendef:tdef;n:tnode;filepos:tfileposinfo;parseddef:tdef;gensym:tsym;p2:tnode):tnode;
        var
          again,
          getaddr : boolean;
          pload : tnode;
          spezcontext : tspecializationcontext;
          structdef,
          inheriteddef : tabstractrecorddef;
          callflags : tcallnodeflags;
        begin
          if n.nodetype=specializen then
            begin
              getaddr:=tspecializenode(n).getaddr;
              pload:=tspecializenode(n).left;
              inheriteddef:=tabstractrecorddef(tspecializenode(n).inheriteddef);
              tspecializenode(n).left:=nil;
            end
          else
            begin
              getaddr:=false;
              pload:=nil;
              inheriteddef:=nil;
            end;

          if assigned(parseddef) and assigned(gensym) and assigned(p2) then
            gendef:=generate_specialization_phase1(spezcontext,gendef,parseddef,gensym.realname,p2.fileinfo)
          else
            gendef:=generate_specialization_phase1(spezcontext,gendef);
          case gendef.typ of
            errordef:
              begin
                spezcontext.free;
                spezcontext:=nil;
                gensym:=generrorsym;
              end;
            objectdef,
            recorddef,
            procvardef,
            arraydef:
              begin
                gendef:=generate_specialization_phase2(spezcontext,tstoreddef(gendef),false,'');
                spezcontext.free;
                spezcontext:=nil;
                gensym:=gendef.typesym;
              end;
            procdef:
              begin
                if block_type<>bt_body then
                  begin
                    message(parser_e_illegal_expression);
                    gensym:=generrorsym;
                  end
                else
                  begin
                    gensym:=tprocdef(gendef).procsym;
                  end;
              end;
            else
              internalerror(2015092702);
          end;

          { in case of a class or a record the specialized generic
            is always a classrefdef }
          again:=false;

          if assigned(pload) then
            begin
              result:=pload;
              typecheckpass(result);
              structdef:=inheriteddef;
              if not assigned(structdef) then
                case result.resultdef.typ of
                  objectdef,
                  recorddef:
                    begin
                      structdef:=tabstractrecorddef(result.resultdef);
                    end;
                  classrefdef:
                    begin
                      structdef:=tabstractrecorddef(tclassrefdef(result.resultdef).pointeddef);
                    end;
                  else
                    internalerror(2015092703);
                end;
              if not (structdef.typ in [recorddef,objectdef]) then
                internalerror(2018092101);
              if assigned(inheriteddef) then
                begin
                  callflags:=[cnf_inherited];
                  include(current_procinfo.flags,pi_has_inherited);
                end
              else
                callflags:=[];
              do_member_read(structdef,getaddr,gensym,result,again,callflags,spezcontext);
              spezcontext:=nil;
            end
          else
            begin
              if gensym.typ=procsym then
                begin
                  result:=nil;
                  { check if it's a method/class method }
                  if is_member_read(gensym,gensym.owner,result,parseddef) then
                    begin
                      { if we are accessing a owner procsym from the nested }
                      { class we need to call it as a class member }
                      if (gensym.owner.symtabletype in [ObjectSymtable,recordsymtable]) and
                          assigned(current_structdef) and (current_structdef<>parseddef) and is_owned_by(current_structdef,parseddef) then
                        result:=cloadvmtaddrnode.create(ctypenode.create(parseddef));
                      { not srsymtable.symtabletype since that can be }
                      { withsymtable as well                          }
                      if (gensym.owner.symtabletype in [ObjectSymtable,recordsymtable]) then
                        begin
                          do_member_read(tabstractrecorddef(parseddef),getaddr,gensym,result,again,[],spezcontext);
                          spezcontext:=nil;
                        end
                      else
                        { no procsyms in records (yet) }
                        internalerror(2015092704);
                    end
                  else
                    begin
                      { regular procedure/function call }
                      do_proc_call(gensym,gensym.owner,nil,
                                   (getaddr and not(token in [_CARET,_POINT,_LECKKLAMMER])),
                                   again,result,[],spezcontext);
                      spezcontext:=nil;
                    end;
                  end
                else
                  { handle potential typecasts, etc }
                  result:=handle_factor_typenode(gendef,false,again,nil,false);
            end;

          { parse postfix operators }
          if postfixoperators(result,again,false) then
            if assigned(result) then
              result.fileinfo:=filepos
            else
              result:=cerrornode.create;

          spezcontext.free;
        end;

      label
        SubExprStart;
      var
        p1,p2,ptmp : tnode;
        oldt    : Ttoken;
        filepos : tfileposinfo;
        gendef,parseddef : tdef;
        gensym : tsym;
      begin
        SubExprStart:
        if pred_level=highest_precedence then
          begin
            if factornode=nil then
              p1:=factor(false,flags)
            else
              p1:=factornode;
          end
        else
          p1:=sub_expr(succ(pred_level),flags+[ef_accept_equal],factornode);
        repeat
          if (token in [NOTOKEN..last_operator]) and
             (token in operator_levels[pred_level]) and
             ((token<>_EQ) or (ef_accept_equal in flags)) then
           begin
             oldt:=token;
             filepos:=current_tokenpos;
             consume(token);
             if pred_level=highest_precedence then
               p2:=factor(false,[])
             else
               p2:=sub_expr(succ(pred_level),flags+[ef_accept_equal],nil);
             case oldt of
               _PLUS :
                 p1:=caddnode.create(addn,p1,p2);
               _MINUS :
                 p1:=caddnode.create(subn,p1,p2);
               _STAR :
                 p1:=caddnode.create(muln,p1,p2);
               _SLASH :
                 p1:=caddnode.create(slashn,p1,p2);
               _EQ:
                 p1:=caddnode.create(equaln,p1,p2);
               _GT :
                 p1:=caddnode.create(gtn,p1,p2);
               _LT :
                 begin
                   { we need to decice whether we have an inline specialization
                     (type nodes to the left and right of "<", mode Delphi and
                     ">" or "," following) or a normal "<" comparison }
                   { TODO : p1 could be a non type if e.g. a variable with the
                            same name is defined in the same unit where the
                            generic is defined (though "same unit" is not
                            necessarily needed) }
                   if getgenericsym(p1,gensym) and
                      { Attention: when nested specializations are supported
                                   p2 could be a loadn if a "<" follows }
                      istypenode(p2) and
                       (m_delphi in current_settings.modeswitches) and
                       { TODO : add _LT, _LSHARPBRACKET for nested specializations }
                       (token in [_GT,_RSHARPBRACKET,_COMMA]) then
                     begin
                       { this is an inline specialization }

                       { retrieve the defs of two nodes }
                       if p1.nodetype=specializen then
                         gendef:=gettypedef(tspecializenode(p1).sym)
                       else
                         gendef:=nil;
                       parseddef:=gettypedef(p2);

                       { check the hints for parseddef }
                       check_hints(parseddef.typesym,parseddef.typesym.symoptions,parseddef.typesym.deprecatedmsg,p1.fileinfo);

                       ptmp:=generate_inline_specialization(gendef,p1,filepos,parseddef,gensym,p2);

                       { we don't need these nodes anymore }
                       p1.free;
                       p2.free;

                       p1:=ptmp;

                       { with p1 now set we are in reality directly behind the
                         call to "factor" thus we need to call down to that
                         again }
                       { This is disabled until specializations on the right
                         hand side work as well, because
                         "not working expressions" is better than "half working
                         expressions" }
                       {factornode:=p1;
                       goto SubExprStart;}
                     end
                   else
                     begin
                       { this is a normal "<" comparison }

                       { potential generic types that are followed by a "<": }

                       { a) might not have their resultdef set }
                       if not assigned(p1.resultdef) then
                         do_typecheckpass(p1);

                       { b) are not checked whether they are an undefined def,
                            but not a generic parameter }
                       if (p1.nodetype=typen) and
                           (ttypenode(p1).typedef.typ=undefineddef) and
                           assigned(ttypenode(p1).typedef.typesym) and
                           not (sp_generic_para in ttypenode(p1).typedef.typesym.symoptions) then
                         begin
                           identifier_not_found(ttypenode(p1).typedef.typesym.RealName);
                           p1.Free;
                           p1:=cerrornode.create;
                         end;

                       { c) don't have their hints checked }
                       if istypenode(p1) then
                         begin
                           gendef:=gettypedef(p1);
                           if gendef.typ in [objectdef,recorddef,arraydef,procvardef] then
                             check_hints(gendef.typesym,gendef.typesym.symoptions,gendef.typesym.deprecatedmsg);
                         end;

                       { Note: the second part of the expression will be needed
                               for nested specializations }
                       if istypenode(p2) {and
                           not (token in [_LT, _LSHARPBRACKET])} then
                         begin
                           gendef:=gettypedef(p2);
                           if gendef.typ in [objectdef,recorddef,arraydef,procvardef] then
                             check_hints(gendef.typesym,gendef.typesym.symoptions,gendef.typesym.deprecatedmsg);
                         end;

                       { create the comparison node for "<" }
                       p1:=caddnode.create(ltn,p1,p2)
                     end;
                 end;
               _GTE :
                 p1:=caddnode.create(gten,p1,p2);
               _LTE :
                 p1:=caddnode.create(lten,p1,p2);
               _SYMDIF :
                 p1:=caddnode.create(symdifn,p1,p2);
               _STARSTAR :
                 p1:=caddnode.create(starstarn,p1,p2);
               _OP_AS,
               _OP_IS :
                 begin
                   if (m_delphi in current_settings.modeswitches) and
                       (token in [_LT, _LSHARPBRACKET]) and
                       getgenericsym(p2,gensym) then
                     begin
                       { for now we're handling this as a generic declaration;
                         there could be cases though (because of operator
                         overloading) where this is the wrong decision... }
                       if gensym.typ=typesym then
                         gendef:=ttypesym(gensym).typedef
                       else
                         if gensym.typ=procsym then
                           gendef:=tdef(tprocsym(gensym).procdeflist[0])
                         else
                           internalerror(2015072401);

                       ptmp:=generate_inline_specialization(gendef,p2,filepos,nil,nil,nil);

                       { we don't need the old p2 anymore }
                       p2.Free;

                       p2:=ptmp;

                       { here we don't need to call back down to "factor", thus
                         no "goto" }
                     end;

                   { now generate the "is" or "as" node }
                   case oldt of
                     _OP_AS:
                       p1:=casnode.create(p1,p2);
                     _OP_IS:
                       p1:=cisnode.create(p1,p2);
                     else
                       internalerror(2019050528);
                   end;
                 end;
               _OP_IN :
                 p1:=cinnode.create(p1,p2);
               _OP_OR,
               _PIPE {macpas only} :
                 begin
                   p1:=caddnode.create(orn,p1,p2);
                   if (oldt = _PIPE) then
                     include(p1.flags,nf_short_bool);
                 end;
               _OP_AND,
               _AMPERSAND {macpas only} :
                 begin
                   p1:=caddnode.create(andn,p1,p2);
                   if (oldt = _AMPERSAND) then
                     include(p1.flags,nf_short_bool);
                 end;
               _OP_DIV :
                 p1:=cmoddivnode.create(divn,p1,p2);
               _OP_NOT :
                 p1:=cnotnode.create(p1);
               _OP_MOD :
                 begin
                   p1:=cmoddivnode.create(modn,p1,p2);
                   if m_isolike_mod in current_settings.modeswitches then
                     include(p1.flags,nf_isomod);
                 end;
               _OP_SHL :
                 p1:=cshlshrnode.create(shln,p1,p2);
               _OP_SHR :
                 p1:=cshlshrnode.create(shrn,p1,p2);
               _OP_XOR :
                 p1:=caddnode.create(xorn,p1,p2);
               _ASSIGNMENT :
                 p1:=cassignmentnode.create(p1,p2);
               _NE :
                 p1:=caddnode.create(unequaln,p1,p2);
               else
                 internalerror(2019050529);
             end;
             p1.fileinfo:=filepos;
           end
          else
           break;
        until false;
        sub_expr:=p1;
      end;


    function comp_expr(flags:texprflags):tnode;
      var
         oldafterassignment : boolean;
         p1 : tnode;
      begin
         oldafterassignment:=afterassignment;
         afterassignment:=true;
         p1:=sub_expr(opcompare,flags,nil);
         { get the resultdef for this expression }
         if not assigned(p1.resultdef) then
          do_typecheckpass(p1);
         afterassignment:=oldafterassignment;
         comp_expr:=p1;
      end;


    function expr(dotypecheck : boolean) : tnode;

      var
         p1,p2 : tnode;
         filepos : tfileposinfo;
         oldafterassignment,
         updatefpos          : boolean;

      begin
         oldafterassignment:=afterassignment;
         p1:=sub_expr(opcompare,[ef_accept_equal],nil);
         { get the resultdef for this expression }
         if not assigned(p1.resultdef) and
            dotypecheck then
          do_typecheckpass(p1);
         filepos:=current_tokenpos;
         if token in [_ASSIGNMENT,_PLUSASN,_MINUSASN,_STARASN,_SLASHASN] then
           afterassignment:=true;
         updatefpos:=true;
         case token of
           _POINTPOINT :
             begin
                consume(_POINTPOINT);
                p2:=sub_expr(opcompare,[ef_accept_equal],nil);
                p1:=crangenode.create(p1,p2);
             end;
           _ASSIGNMENT :
             begin
                consume(_ASSIGNMENT);
                if assigned(p1.resultdef) and (p1.resultdef.typ=procvardef) then
                  getprocvardef:=tprocvardef(p1.resultdef);
                p2:=sub_expr(opcompare,[ef_accept_equal],nil);
                if assigned(getprocvardef) then
                  handle_procvar(getprocvardef,p2);
                getprocvardef:=nil;
                p1:=cassignmentnode.create(p1,p2);
             end;
           _PLUSASN :
             begin
               consume(_PLUSASN);
               p2:=sub_expr(opcompare,[ef_accept_equal],nil);
               p1:=gen_c_style_operator(addn,p1,p2);
            end;
          _MINUSASN :
            begin
               consume(_MINUSASN);
               p2:=sub_expr(opcompare,[ef_accept_equal],nil);
               p1:=gen_c_style_operator(subn,p1,p2);
            end;
          _STARASN :
            begin
               consume(_STARASN  );
               p2:=sub_expr(opcompare,[ef_accept_equal],nil);
               p1:=gen_c_style_operator(muln,p1,p2);
            end;
          _SLASHASN :
            begin
               consume(_SLASHASN  );
               p2:=sub_expr(opcompare,[ef_accept_equal],nil);
               p1:=gen_c_style_operator(slashn,p1,p2);
            end;
          else
            updatefpos:=false;
         end;
         { get the resultdef for this expression }
         if not assigned(p1.resultdef) and
            dotypecheck then
          do_typecheckpass(p1);
         afterassignment:=oldafterassignment;
         if updatefpos then
           p1.fileinfo:=filepos;
         expr:=p1;
      end;

    function get_intconst:TConstExprInt;
    {Reads an expression, tries to evalute it and check if it is an integer
     constant. Then the constant is returned.}
    var
      p:tnode;
    begin
      result:=0;
      p:=comp_expr([ef_accept_equal]);
      if not codegenerror then
       begin
         if (p.nodetype<>ordconstn) or
            not(is_integer(p.resultdef)) then
          Message(parser_e_illegal_expression)
         else
          result:=tordconstnode(p).value;
       end;
      p.free;
    end;


    function get_stringconst:string;
    {Reads an expression, tries to evaluate it and checks if it is a string
     constant. Then the constant is returned.}
    var
      p:tnode;
    begin
      get_stringconst:='';
      p:=comp_expr([ef_accept_equal]);
      if p.nodetype<>stringconstn then
        begin
          if (p.nodetype=ordconstn) and is_char(p.resultdef) then
            get_stringconst:=char(int64(tordconstnode(p).value))
          else
            Message(parser_e_illegal_expression);
        end
      else
        get_stringconst:=strpas(tstringconstnode(p).value_str);
      p.free;
    end;

end.
