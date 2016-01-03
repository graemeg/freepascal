{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Parses variable declarations. Used for var statement and record
    definitions

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
unit pdecvar;

{$i fpcdefs.inc}

interface

    uses
      cclasses,
      symtable,symsym,symdef;

    type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      tvar_dec_option=(vd_record,vd_object,vd_threadvar,vd_class,vd_final,vd_canreorder,vd_check_generic);
=======
      tvar_dec_option=(vd_record,vd_object,vd_threadvar,vd_class);
>>>>>>> graemeg/cpstrnew
=======
      tvar_dec_option=(vd_record,vd_object,vd_threadvar,vd_class);
>>>>>>> graemeg/cpstrnew
=======
      tvar_dec_option=(vd_record,vd_object,vd_threadvar,vd_class);
>>>>>>> graemeg/cpstrnew
=======
      tvar_dec_option=(vd_record,vd_object,vd_threadvar,vd_class);
>>>>>>> origin/cpstrnew
      tvar_dec_options=set of tvar_dec_option;

    function  read_property_dec(is_classproperty:boolean;astruct:tabstractrecorddef):tpropertysym;

    procedure read_var_decls(options:Tvar_dec_options;out had_generic:boolean);

    procedure read_record_fields(options:Tvar_dec_options; reorderlist: TFPObjectList; variantdesc: ppvariantrecdesc;out had_generic:boolean);

    procedure read_public_and_external(vs: tabstractvarsym);

<<<<<<< HEAD
<<<<<<< HEAD
    procedure try_consume_sectiondirective(var asection: ansistring);
=======
    procedure read_public_and_external(vs: tabstractvarsym);
>>>>>>> graemeg/fixes_2_2
=======
    procedure read_public_and_external(vs: tabstractvarsym);
>>>>>>> origin/fixes_2_2

implementation

    uses
       SysUtils,
       { common }
       cutils,
       { global }
       globtype,globals,tokens,verbose,constexp,
       systems,
       { symtable }
<<<<<<< HEAD
       symconst,symbase,symtype,defutil,defcmp,symcreat,
{$if defined(i386) or defined(i8086)}
       symcpu,
{$endif}
=======
       symconst,symbase,symtype,symtable,defutil,defcmp,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
       fmodule,htypechk,
       { pass 1 }
       node,pass_1,aasmdata,
       ncon,nmat,nadd,ncal,nset,ncnv,ninl,nld,nflw,nmem,nutils,
       { codegen }
       ncgutil,ngenutil,
       { parser }
       scanner,
       pbase,pexpr,ptype,ptconst,pdecsub,
       { link }
       import
       ;


    function read_property_dec(is_classproperty:boolean;astruct:tabstractrecorddef):tpropertysym;

        { convert a node tree to symlist and return the last
          symbol }
        function parse_symlist(pl:tpropaccesslist;out def:tdef):boolean;
          var
            idx : longint;
            sym : tsym;
            srsymtable : TSymtable;
            st  : TSymtable;
            p   : tnode;
          begin
            result:=true;
            def:=nil;
            if token=_ID then
             begin
               if assigned(astruct) then
                 sym:=search_struct_member(astruct,pattern)
               else
                 searchsym(pattern,sym,srsymtable);
               if assigned(sym) then
                begin
                  if assigned(astruct) and
                     not is_visible_for_object(sym,astruct) then
                    Message(parser_e_cant_access_private_member);
                  case sym.typ of
                    fieldvarsym :
                      begin
                        addsymref(sym);
                        pl.addsym(sl_load,sym);
                        def:=tfieldvarsym(sym).vardef;
                      end;
                    procsym :
                      begin
                        addsymref(sym);
                        pl.addsym(sl_call,sym);
                      end;
                    else
                      begin
                        Message1(parser_e_illegal_field_or_method,orgpattern);
                        def:=generrordef;
                        result:=false;
                      end;
                  end;
                end
               else
                begin
                  Message1(parser_e_illegal_field_or_method,orgpattern);
                  def:=generrordef;
                  result:=false;
                end;
               consume(_ID);
               repeat
                 case token of
                   _ID,
                   _SEMICOLON :
                     begin
                       break;
                     end;
                   _POINT :
                     begin
                       consume(_POINT);
                       if assigned(def) then
                        begin
                          st:=def.GetSymtable(gs_record);
                          if assigned(st) then
                           begin
                             sym:=tsym(st.Find(pattern));
                             if not(assigned(sym)) and is_object(def) then
<<<<<<< HEAD
<<<<<<< HEAD
                               sym:=search_struct_member(tobjectdef(def),pattern);
=======
                               sym:=search_class_member(tobjectdef(def),pattern);
>>>>>>> graemeg/fixes_2_2
=======
                               sym:=search_class_member(tobjectdef(def),pattern);
>>>>>>> origin/fixes_2_2
                             if assigned(sym) then
                              begin
                                pl.addsym(sl_subscript,sym);
                                case sym.typ of
                                  fieldvarsym :
                                    def:=tfieldvarsym(sym).vardef;
                                  else
                                    begin
                                      Message1(sym_e_illegal_field,orgpattern);
                                      result:=false;
                                    end;
                                end;
                              end
                             else
                              begin
                                Message1(sym_e_illegal_field,orgpattern);
                                result:=false;
                              end;
                           end
                          else
                           begin
                             Message(parser_e_invalid_qualifier);
                             result:=false;
                           end;
                        end
                       else
                        begin
                          Message(parser_e_invalid_qualifier);
                          result:=false;
                        end;
                       consume(_ID);
                     end;
                   _LECKKLAMMER :
                     begin
                       consume(_LECKKLAMMER);
                       repeat
                         if def.typ=arraydef then
                          begin
                            idx:=0;
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
                            if (not codegenerror) then
                             begin
                               if (p.nodetype=ordconstn) then
                                 begin
                                   { type/range checking }
                                   inserttypeconv(p,tarraydef(def).rangedef);
<<<<<<< HEAD
<<<<<<< HEAD
                                   if (Tordconstnode(p).value<int64(low(longint))) or
                                      (Tordconstnode(p).value>int64(high(longint))) then
                                     message(parser_e_array_range_out_of_bounds)
                                   else
                                     idx:=Tordconstnode(p).value.svalue
=======
                                   idx:=tordconstnode(p).value
>>>>>>> graemeg/fixes_2_2
=======
                                   idx:=tordconstnode(p).value
>>>>>>> origin/fixes_2_2
                                 end
                               else
                                Message(type_e_ordinal_expr_expected)
                             end;
                            pl.addconst(sl_vec,idx,p.resultdef);
                            p.free;
                            def:=tarraydef(def).elementdef;
                          end
                         else
                          begin
                            Message(parser_e_invalid_qualifier);
                            result:=false;
                          end;
                       until not try_to_consume(_COMMA);
                       consume(_RECKKLAMMER);
                     end;
                   else
                     begin
                       Message(parser_e_ill_property_access_sym);
                       result:=false;
                       break;
                     end;
                 end;
               until false;
             end
            else
             begin
               Message(parser_e_ill_property_access_sym);
               result:=false;
             end;
          end;

          function allow_default_property(p : tpropertysym) : boolean;

          begin
             allow_default_property:=
               (is_ordinal(p.propdef) or
{$ifndef cpu64bitaddr}
               is_64bitint(p.propdef) or
{$endif cpu64bitaddr}
               is_class(p.propdef) or
               is_single(p.propdef) or
               (p.propdef.typ in [classrefdef,pointerdef]) or
                 is_smallset(p.propdef)
               ) and not
               (
                (p.propdef.typ=arraydef) and
                (ppo_indexed in p.propoptions)
               ) and not
               (ppo_hasparameters in p.propoptions);
          end;

<<<<<<< HEAD
<<<<<<< HEAD
          procedure create_accessor_procsym(p: tpropertysym; pd: tprocdef; const prefix: string;
              accesstype: tpropaccesslisttypes);
            var
              sym: tprocsym;
            begin
              handle_calling_convention(pd);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              sym:=cprocsym.create(prefix+lower(p.realname));
=======
              sym:=tprocsym.create(prefix+lower(p.realname));
>>>>>>> graemeg/cpstrnew
=======
              sym:=tprocsym.create(prefix+lower(p.realname));
>>>>>>> graemeg/cpstrnew
=======
              sym:=tprocsym.create(prefix+lower(p.realname));
>>>>>>> graemeg/cpstrnew
=======
              sym:=tprocsym.create(prefix+lower(p.realname));
>>>>>>> origin/cpstrnew
              symtablestack.top.insert(sym);
              pd.procsym:=sym;
              include(pd.procoptions,po_dispid);
              include(pd.procoptions,po_global);
              pd.visibility:=vis_private;
              proc_add_definition(pd);
              p.propaccesslist[accesstype].addsym(sl_call,sym);
              p.propaccesslist[accesstype].procdef:=pd;
            end;

          procedure parse_dispinterface(p : tpropertysym; readpd,writepd: tprocdef;
              var paranr: word);
            var
              hasread, haswrite: boolean;
              pt: tnode;
              hdispid: longint;
              hparavs: tparavarsym;
            begin
              p.propaccesslist[palt_read].clear;
              p.propaccesslist[palt_write].clear;

              hasread:=true;
              haswrite:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              hdispid:=0;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

              if try_to_consume(_READONLY) then
                haswrite:=false
              else if try_to_consume(_WRITEONLY) then
                hasread:=false;

              if try_to_consume(_DISPID) then
                begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  pt:=comp_expr([ef_accept_equal]);
                  if is_constintnode(pt) then
                    if (Tordconstnode(pt).value<int64(low(longint))) or (Tordconstnode(pt).value>int64(high(longint))) then
                      message3(type_e_range_check_error_bounds,tostr(Tordconstnode(pt).value),tostr(low(longint)),tostr(high(longint)))
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  pt:=comp_expr(true,false);
                  if is_constintnode(pt) then
                    if (Tordconstnode(pt).value<int64(low(longint))) or (Tordconstnode(pt).value>int64(high(longint))) then
                      message(parser_e_range_check_error)
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
                    else
                      hdispid:=Tordconstnode(pt).value.svalue
                  else
                    Message(parser_e_dispid_must_be_ord_const);
                  pt.free;
                end
              else
                hdispid:=tobjectdef(astruct).get_next_dispid;

              { COM property is simply a pair of methods, tagged with 'propertyget'
                and 'propertyset' flags (or a single method if access is restricted).
                Creating these implicit accessor methods also allows the rest of compiler
                to handle dispinterface properties the same way as regular ones. }
              if hasread then
                begin
                  readpd.returndef:=p.propdef;
                  readpd.dispid:=hdispid;
                  readpd.proctypeoption:=potype_propgetter;
                  create_accessor_procsym(p,readpd,'get$',palt_read);
                end;
              if haswrite then
                begin
                  { add an extra parameter, a placeholder of the value to set }
                  inc(paranr);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  hparavs:=cparavarsym.create('$value',10*paranr,vs_value,p.propdef,[]);
=======
                  hparavs:=tparavarsym.create('$value',10*paranr,vs_value,p.propdef,[]);
>>>>>>> graemeg/cpstrnew
=======
                  hparavs:=tparavarsym.create('$value',10*paranr,vs_value,p.propdef,[]);
>>>>>>> graemeg/cpstrnew
=======
                  hparavs:=tparavarsym.create('$value',10*paranr,vs_value,p.propdef,[]);
>>>>>>> graemeg/cpstrnew
=======
                  hparavs:=tparavarsym.create('$value',10*paranr,vs_value,p.propdef,[]);
>>>>>>> origin/cpstrnew
                  writepd.parast.insert(hparavs);

                  writepd.proctypeoption:=potype_propsetter;
                  writepd.dispid:=hdispid;
                  create_accessor_procsym(p,writepd,'put$',palt_write);
                end;
            end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          procedure add_index_parameter(var paranr: word; p: tpropertysym; readprocdef, writeprocdef: tprocdef);
            var
              hparavs: tparavarsym;
            begin
              inc(paranr);
              hparavs:=tparavarsym.create('$index',10*paranr,vs_value,p.indexdef,[]);
              readprocdef.parast.insert(hparavs);
              hparavs:=tparavarsym.create('$index',10*paranr,vs_value,p.indexdef,[]);
              writeprocdef.parast.insert(hparavs);
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
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      var
         sym : tsym;
         srsymtable: tsymtable;
         p : tpropertysym;
         overridden : tsym;
         varspez : tvarspez;
         hdef : tdef;
         arraytype : tdef;
         def : tdef;
         pt : tnode;
         sc : TFPObjectList;
         paranr : word;
         i      : longint;
         ImplIntf     : TImplementedInterface;
         found,
         gotreadorwrite: boolean;
         hreadparavs,
         hparavs      : tparavarsym;
         storedprocdef: tprocvardef;
         readprocdef,
         writeprocdef : tprocdef;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         result:=nil;
         { Generate temp procdefs to search for matching read/write
           procedures. the readprocdef will store all definitions }
         paranr:=0;
         readprocdef:=cprocdef.create(normal_function_level,true);
         writeprocdef:=cprocdef.create(normal_function_level,true);
=======
=======
>>>>>>> graemeg/cpstrnew
         { Generate temp procdefs to search for matching read/write
           procedures. the readprocdef will store all definitions }
         paranr:=0;
         readprocdef:=tprocdef.create(normal_function_level);
         writeprocdef:=tprocdef.create(normal_function_level);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

         readprocdef.struct:=astruct;
         writeprocdef.struct:=astruct;

=======
=======
>>>>>>> origin/cpstrnew
         { Generate temp procdefs to search for matching read/write
           procedures. the readprocdef will store all definitions }
         paranr:=0;
         readprocdef:=tprocdef.create(normal_function_level);
         writeprocdef:=tprocdef.create(normal_function_level);

         readprocdef.struct:=astruct;
         writeprocdef.struct:=astruct;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         if assigned(astruct) and is_classproperty then
           begin
             readprocdef.procoptions:=[po_staticmethod,po_classmethod];
             writeprocdef.procoptions:=[po_staticmethod,po_classmethod];
           end;

         if token<>_ID then
           begin
              consume(_ID);
              consume(_SEMICOLON);
              exit;
           end;
         { Generate propertysym and insert in symtablestack }
<<<<<<< HEAD
         p:=cpropertysym.create(orgpattern);
         p.visibility:=symtablestack.top.currentvisibility;
         p.default:=longint($80000000);
         if is_classproperty then
           include(p.symoptions, sp_static);
=======
         p:=tpropertysym.create(orgpattern);
         p.default:=longint($80000000);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         symtablestack.top.insert(p);
         consume(_ID);
         { property parameters ? }
         if try_to_consume(_LECKKLAMMER) then
           begin
              if (p.visibility=vis_published) and
                not (m_delphi in current_settings.modeswitches) then
                Message(parser_e_cant_publish_that_property);
              { create a list of the parameters }
              p.parast:=tparasymtable.create(nil,0);
              symtablestack.push(p.parast);
              sc:=TFPObjectList.create(false);
              repeat
                if try_to_consume(_VAR) then
                  varspez:=vs_var
                else if try_to_consume(_CONST) then
                  varspez:=vs_const
                else if try_to_consume(_CONSTREF) then
                  varspez:=vs_constref
                else if (m_out in current_settings.modeswitches) and try_to_consume(_OUT) then
                  varspez:=vs_out
                else
                  varspez:=vs_value;
                sc.clear;
                repeat
                  inc(paranr);
                  hreadparavs:=cparavarsym.create(orgpattern,10*paranr,varspez,generrordef,[]);
                  p.parast.insert(hreadparavs);
                  sc.add(hreadparavs);
                  consume(_ID);
                until not try_to_consume(_COMMA);
                if try_to_consume(_COLON) then
                  begin
                    if try_to_consume(_ARRAY) then
                      begin
                        consume(_OF);
                        { define range and type of range }
                        hdef:=carraydef.create(0,-1,s32inttype);
                        { define field type }
                        single_type(arraytype,[]);
                        tarraydef(hdef).elementdef:=arraytype;
                      end
                    else
                      single_type(hdef,[]);
                  end
                else
                  hdef:=cformaltype;
                for i:=0 to sc.count-1 do
                  tparavarsym(sc[i]).vardef:=hdef;
              until not try_to_consume(_SEMICOLON);
              sc.free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              symtablestack.pop(p.parast);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              symtablestack.pop(readprocdef.parast);
>>>>>>> graemeg/cpstrnew
              consume(_RECKKLAMMER);

              { the parser need to know if a property has parameters, the
                index parameter doesn't count (PFV) }
              if paranr>0 then
                begin
                  p.add_accessor_parameters(readprocdef,writeprocdef);
                  include(p.propoptions,ppo_hasparameters);
                end;
           end;
         { overridden property ?                                 }
         { force property interface
             there is a property parameter
             a global property }
         if (token=_COLON) or (paranr>0) or (astruct=nil) then
           begin
              consume(_COLON);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              single_type(p.propdef,[stoAllowSpecialization]);
=======
              single_type(p.propdef,[]);
>>>>>>> graemeg/cpstrnew
=======
              single_type(p.propdef,[]);
>>>>>>> graemeg/cpstrnew
=======
              single_type(p.propdef,[]);
>>>>>>> graemeg/cpstrnew
=======
              single_type(p.propdef,[]);
>>>>>>> origin/cpstrnew

              if is_dispinterface(astruct) and not is_automatable(p.propdef) then
                Message1(type_e_not_automatable,p.propdef.typename);

              if (idtoken=_INDEX) then
                begin
                   consume(_INDEX);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   pt:=comp_expr([ef_accept_equal]);
=======
                   pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                   pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                   pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                   pt:=comp_expr(true,false);
>>>>>>> origin/cpstrnew
                   { Only allow enum and integer indexes. Convert all integer
                     values to objpas.integer (s32int on 32- and 64-bit targets,
                     s16int on 16- and 8-bit) to be compatible with delphi,
                     because the procedure matching requires equal parameters }
                   if is_constnode(pt) and
                      is_ordinal(pt.resultdef)
                      and (not is_64bitint(pt.resultdef))
{$if defined(cpu8bitalu) or defined(cpu16bitalu)}
                      and (not is_32bitint(pt.resultdef))
{$endif}
                      then
                     begin
                       if is_integer(pt.resultdef) then
{$if defined(cpu8bitalu) or defined(cpu16bitalu)}
                         inserttypeconv_internal(pt,s16inttype);
{$else}
                         inserttypeconv_internal(pt,s32inttype);
{$endif}
                       p.index:=tordconstnode(pt).value.svalue;
                     end
                   else
                     begin
                       Message(parser_e_invalid_property_index_value);
                       p.index:=0;
                     end;
                   p.indexdef:=pt.resultdef;
                   include(p.propoptions,ppo_indexed);
                   { concat a longint to the para templates }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   p.add_index_parameter(paranr,readprocdef,writeprocdef);
=======
                   add_index_parameter(paranr,p,readprocdef,writeprocdef);
>>>>>>> graemeg/cpstrnew
=======
                   add_index_parameter(paranr,p,readprocdef,writeprocdef);
>>>>>>> graemeg/cpstrnew
=======
                   add_index_parameter(paranr,p,readprocdef,writeprocdef);
>>>>>>> graemeg/cpstrnew
=======
                   add_index_parameter(paranr,p,readprocdef,writeprocdef);
>>>>>>> origin/cpstrnew
                   pt.free;
                end;
           end
         else
           begin
              { do an property override }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              if (astruct.typ=objectdef) and assigned(tobjectdef(astruct).childof) then
=======
              if (astruct.typ=objectdef) then
>>>>>>> graemeg/cpstrnew
=======
              if (astruct.typ=objectdef) then
>>>>>>> graemeg/cpstrnew
=======
              if (astruct.typ=objectdef) then
>>>>>>> graemeg/cpstrnew
=======
              if (astruct.typ=objectdef) then
>>>>>>> origin/cpstrnew
                overridden:=search_struct_member(tobjectdef(astruct).childof,p.name)
              else
                overridden:=nil;
              if assigned(overridden) and
                 (overridden.typ=propertysym) and
                 not(is_dispinterface(astruct)) then
                begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  tpropertysym(overridden).makeduplicate(p,readprocdef,writeprocdef,paranr);
                  p.register_override(tpropertysym(overridden));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  p.overriddenpropsym:=tpropertysym(overridden);
                  { inherit all type related entries }
                  p.indexdef:=tpropertysym(overridden).indexdef;
                  p.propdef:=tpropertysym(overridden).propdef;
                  p.index:=tpropertysym(overridden).index;
                  p.default:=tpropertysym(overridden).default;
                  p.propoptions:=tpropertysym(overridden).propoptions;
                  if ppo_indexed in p.propoptions then
                    add_index_parameter(paranr,p,readprocdef,writeprocdef);
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
                end
              else
                begin
                  p.propdef:=generrordef;
                  message(parser_e_no_property_found_to_override);
                end;
           end;
         if ((p.visibility=vis_published) or is_dispinterface(astruct)) and
            (not(p.propdef.is_publishable) or (sp_static in p.symoptions)) then
           begin
             Message(parser_e_cant_publish_that_property);
             p.visibility:=vis_public;
           end;

         if not(is_dispinterface(astruct)) then
           begin
             gotreadorwrite:=false;
             { parse accessors }
             if try_to_consume(_READ) then
               begin
                 gotreadorwrite:=true;
                 p.propaccesslist[palt_read].clear;
                 if parse_symlist(p.propaccesslist[palt_read],def) then
                  begin
                    sym:=p.propaccesslist[palt_read].firstsym^.sym;
<<<<<<< HEAD
                    { getter is a function returning the type of the property }
                    if sym.typ=procsym then
                      begin
                        readprocdef.returndef:=p.propdef;
                        { Insert hidden parameters }
                        handle_calling_convention(readprocdef);
                      end;
                    p.add_getter_or_setter_for_sym(palt_read,sym,def,readprocdef);
=======
                    case sym.typ of
                      procsym :
                        begin
                          { read is function returning the type of the property }
                          readprocdef.returndef:=p.propdef;
                          { Insert hidden parameters }
                          handle_calling_convention(readprocdef);
                          { search procdefs matching readprocdef }
                          { we ignore hidden stuff here because the property access symbol might have
                            non default calling conventions which might change the hidden stuff;
                            see tw3216.pp (FK) }
                          p.propaccesslist[palt_read].procdef:=Tprocsym(sym).Find_procdef_bypara(readprocdef.paras,p.propdef,[cpo_allowdefaults,cpo_ignorehidden]);
                          if not assigned(p.propaccesslist[palt_read].procdef) or
                            { because of cpo_ignorehidden we need to compare if it is a static class method and we have a class property }
                            ((sp_static in p.symoptions) <> tprocdef(p.propaccesslist[palt_read].procdef).no_self_node) then
                            Message(parser_e_ill_property_access_sym);
                        end;
                      fieldvarsym :
                        begin
                          if not assigned(def) then
                            internalerror(200310071);
                          if compare_defs(def,p.propdef,nothingn)>=te_equal then
                           begin
                             { property parameters are allowed if this is
                               an indexed property, because the index is then
                               the parameter.
                               Note: In the help of Kylix it is written
                               that it isn't allowed, but the compiler accepts it (PFV) }
                             if (ppo_hasparameters in p.propoptions) or
                                ((sp_static in p.symoptions) <> (sp_static in sym.symoptions)) then
                               Message(parser_e_ill_property_access_sym);
                           end
                          else
                           IncompatibleTypes(def,p.propdef);
                        end;
                      else
                        Message(parser_e_ill_property_access_sym);
                    end;
>>>>>>> graemeg/cpstrnew
                  end;
               end
             else
               p.inherit_accessor(palt_read);
             if try_to_consume(_WRITE) then
               begin
                 gotreadorwrite:=true;
                 p.propaccesslist[palt_write].clear;
                 if parse_symlist(p.propaccesslist[palt_write],def) then
                  begin
                    sym:=p.propaccesslist[palt_write].firstsym^.sym;
<<<<<<< HEAD
                    if sym.typ=procsym then
                      begin
                        { settter is a procedure with an extra value parameter
                          of the of the property }
                        writeprocdef.returndef:=voidtype;
                        inc(paranr);
                        hparavs:=cparavarsym.create('$value',10*paranr,vs_value,p.propdef,[]);
                        writeprocdef.parast.insert(hparavs);
                        { Insert hidden parameters }
                        handle_calling_convention(writeprocdef);
                      end;
                    p.add_getter_or_setter_for_sym(palt_write,sym,def,writeprocdef);
=======
                    case sym.typ of
                      procsym :
                        begin
                          { write is a procedure with an extra value parameter
                            of the of the property }
                          writeprocdef.returndef:=voidtype;
                          inc(paranr);
                          hparavs:=tparavarsym.create('$value',10*paranr,vs_value,p.propdef,[]);
                          writeprocdef.parast.insert(hparavs);
                          { Insert hidden parameters }
                          handle_calling_convention(writeprocdef);
                          { search procdefs matching writeprocdef }
                          if cs_varpropsetter in current_settings.localswitches then
                            p.propaccesslist[palt_write].procdef:=Tprocsym(sym).Find_procdef_bypara(writeprocdef.paras,writeprocdef.returndef,[cpo_allowdefaults,cpo_ignorevarspez])
                          else
                            p.propaccesslist[palt_write].procdef:=Tprocsym(sym).Find_procdef_bypara(writeprocdef.paras,writeprocdef.returndef,[cpo_allowdefaults]);
                          if not assigned(p.propaccesslist[palt_write].procdef) then
                            Message(parser_e_ill_property_access_sym);
                        end;
                      fieldvarsym :
                        begin
                          if not assigned(def) then
                            internalerror(200310072);
                          if compare_defs(def,p.propdef,nothingn)>=te_equal then
                           begin
                             { property parameters are allowed if this is
                               an indexed property, because the index is then
                               the parameter.
                               Note: In the help of Kylix it is written
                               that it isn't allowed, but the compiler accepts it (PFV) }
                             if (ppo_hasparameters in p.propoptions) or
                                ((sp_static in p.symoptions) <> (sp_static in sym.symoptions)) then
                              Message(parser_e_ill_property_access_sym);
                           end
                          else
                           IncompatibleTypes(def,p.propdef);
                        end;
                      else
                        Message(parser_e_ill_property_access_sym);
                    end;
>>>>>>> graemeg/cpstrnew
                  end;
               end
             else
               p.inherit_accessor(palt_write);
             { a new property (needs to declare a getter or setter, except in
               an interface }
             if not(ppo_overrides in p.propoptions) and
                not is_interface(astruct) and
                not gotreadorwrite then
               Consume(_READ);
           end
         else
           parse_dispinterface(p,readprocdef,writeprocdef,paranr);

         { stored is not allowed for dispinterfaces, records or class properties }
         if assigned(astruct) and not(is_dispinterface(astruct) or is_record(astruct)) and not is_classproperty then
           begin
             { ppo_stored is default on for not overridden properties }
             if not assigned(p.overriddenpropsym) then
               include(p.propoptions,ppo_stored);
             if try_to_consume(_STORED) then
              begin
                include(p.propoptions,ppo_stored);
                p.propaccesslist[palt_stored].clear;
<<<<<<< HEAD
                if token=_ID then
                  begin
                    { in the case that idtoken=_DEFAULT }
                    { we have to do nothing except      }
                    { setting ppo_stored, it's the same }
                    { as stored true                    }
                    if idtoken<>_DEFAULT then
                     begin
                       { parse_symlist cannot deal with constsyms, and
                         we also don't want to put constsyms in symlists
                         since they have to be evaluated immediately rather
                         than each time the property is accessed

                         The proper fix would be to always create a parse tree
                         and then convert that one, if appropriate, to a symlist.
                         Currently, we e.g. don't support any constant expressions
                         yet either here, while Delphi does.

                       }
                       { make sure we don't let constants mask class fields/
                         methods
                       }
                       sym:=nil;
                       if (not assigned(astruct) or
                           (search_struct_member(astruct,pattern)=nil)) and
                          searchsym(pattern,sym,srsymtable) and
                          (sym.typ = constsym) then
                         begin
                            addsymref(sym);
                            if not is_boolean(tconstsym(sym).constdef) then
                              Message(parser_e_stored_property_must_be_boolean)
                            else if (tconstsym(sym).value.valueord=0) then
                              { same as for _FALSE }
                              exclude(p.propoptions,ppo_stored)
                            else
                              { same as for _TRUE }
                              p.default:=longint($80000000);
                            consume(_ID);
                          end
                       else if parse_symlist(p.propaccesslist[palt_stored],def) then
                        begin
                          sym:=p.propaccesslist[palt_stored].firstsym^.sym;
                          case sym.typ of
                            procsym :
                              begin
                                 { Create a temporary procvardef to handle parameters }
                                 storedprocdef:=cprocvardef.create(normal_function_level);
                                 include(storedprocdef.procoptions,po_methodpointer);
                                 { Return type must be boolean }
                                 storedprocdef.returndef:=pasbool8type;
                                 { Add index parameter if needed }
                                 if ppo_indexed in p.propoptions then
                                   begin
                                     hparavs:=cparavarsym.create('$index',10,vs_value,p.indexdef,[]);
                                     storedprocdef.parast.insert(hparavs);
                                   end;

                                 { Insert hidden parameters }
                                 handle_calling_convention(storedprocdef);
                                 p.propaccesslist[palt_stored].procdef:=Tprocsym(sym).Find_procdef_bypara(storedprocdef.paras,storedprocdef.returndef,[cpo_allowdefaults,cpo_ignorehidden]);
                                 if not assigned(p.propaccesslist[palt_stored].procdef) then
                                   message(parser_e_ill_property_storage_sym);
                                 { Not needed anymore }
                                 storedprocdef.owner.deletedef(storedprocdef);
                              end;
                            fieldvarsym :
                              begin
                                if not assigned(def) then
                                  internalerror(200310073);
                                if (ppo_hasparameters in p.propoptions) or
                                   not(is_boolean(def)) then
                                 Message(parser_e_stored_property_must_be_boolean);
                              end;
                            else
                              Message(parser_e_ill_property_access_sym);
=======
                case token of
                  _ID:
                    begin
                      { in the case that idtoken=_DEFAULT }
                      { we have to do nothing except      }
                      { setting ppo_stored, it's the same }
                      { as stored true                    }
                      if idtoken<>_DEFAULT then
                       begin
                         { parse_symlist cannot deal with constsyms, and
                           we also don't want to put constsyms in symlists
                           since they have to be evaluated immediately rather
                           than each time the property is accessed

                           The proper fix would be to always create a parse tree
                           and then convert that one, if appropriate, to a symlist.
                           Currently, we e.g. don't support any constant expressions
                           yet either here, while Delphi does.

                         }
                         { make sure we don't let constants mask class fields/
                           methods
                         }
                         if (not assigned(astruct) or
                             (search_struct_member(astruct,pattern)=nil)) and
                            searchsym(pattern,sym,srsymtable) and
                            (sym.typ = constsym) then
                           begin
                              addsymref(sym);
                              if not is_boolean(tconstsym(sym).constdef) then
                                Message(parser_e_stored_property_must_be_boolean)
                              else if (tconstsym(sym).value.valueord=0) then
                                { same as for _FALSE }
                                exclude(p.propoptions,ppo_stored)
                              else
                                { same as for _TRUE }
                                p.default:=longint($80000000);
                              consume(_ID);
                            end
                         else if parse_symlist(p.propaccesslist[palt_stored],def) then
                          begin
                            sym:=p.propaccesslist[palt_stored].firstsym^.sym;
                            case sym.typ of
                              procsym :
                                begin
                                   { Create a temporary procvardef to handle parameters }
                                   storedprocdef:=tprocvardef.create(normal_function_level);
                                   include(storedprocdef.procoptions,po_methodpointer);
                                   { Return type must be boolean }
                                   storedprocdef.returndef:=booltype;
                                   { Add index parameter if needed }
                                   if ppo_indexed in p.propoptions then
                                     begin
                                       hparavs:=tparavarsym.create('$index',10,vs_value,p.indexdef,[]);
                                       storedprocdef.parast.insert(hparavs);
                                     end;

                                   { Insert hidden parameters }
                                   handle_calling_convention(storedprocdef);
                                   p.propaccesslist[palt_stored].procdef:=Tprocsym(sym).Find_procdef_bypara(storedprocdef.paras,storedprocdef.returndef,[cpo_allowdefaults,cpo_ignorehidden]);
                                   if not assigned(p.propaccesslist[palt_stored].procdef) then
                                     message(parser_e_ill_property_storage_sym);
                                   { Not needed anymore }
                                   storedprocdef.owner.deletedef(storedprocdef);
                                end;
                              fieldvarsym :
                                begin
                                  if not assigned(def) then
                                    internalerror(200310073);
                                  if (ppo_hasparameters in p.propoptions) or
                                     not(is_boolean(def)) then
                                   Message(parser_e_stored_property_must_be_boolean);
                                end;
                              else
                                Message(parser_e_ill_property_access_sym);
                            end;
>>>>>>> graemeg/cpstrnew
                          end;
                        end;
                     end;
                  end;
              end;
           end;
         if not is_record(astruct) and try_to_consume(_DEFAULT) then
           begin
              if not allow_default_property(p) then
                begin
                  Message(parser_e_property_cant_have_a_default_value);
                  { Error recovery }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  pt:=comp_expr([ef_accept_equal]);
=======
                  pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                  pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                  pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                  pt:=comp_expr(true,false);
>>>>>>> origin/cpstrnew
                  pt.free;
                end
              else
                begin
                  { Get the result of the default, the firstpass is
                    needed to support values like -1 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  pt:=comp_expr([ef_accept_equal]);
=======
                  pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                  pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                  pt:=comp_expr(true,false);
>>>>>>> graemeg/cpstrnew
=======
                  pt:=comp_expr(true,false);
>>>>>>> origin/cpstrnew
                  if (p.propdef.typ=setdef) and
                     (pt.nodetype=arrayconstructorn) then
                    begin
                      arrayconstructor_to_set(pt);
                      do_typecheckpass(pt);
                    end;
                  inserttypeconv(pt,p.propdef);
                  if not(is_constnode(pt)) then
                    Message(parser_e_property_default_value_must_const);
                  { Set default value }
                  case pt.nodetype of
                    setconstn :
                      p.default:=plongint(tsetconstnode(pt).value_set)^;
                    ordconstn :
                      if (Tordconstnode(pt).value<int64(low(longint))) or
                         (Tordconstnode(pt).value>int64(high(cardinal))) then
                        message3(type_e_range_check_error_bounds,tostr(Tordconstnode(pt).value),tostr(low(longint)),tostr(high(cardinal)))
                      else
                        p.default:=longint(tordconstnode(pt).value.svalue);
                    niln :
                      p.default:=0;
                    realconstn:
                      p.default:=longint(single(trealconstnode(pt).value_real));
                  end;
                  pt.free;
                end;
           end
         else if not is_record(astruct) and try_to_consume(_NODEFAULT) then
           begin
              p.default:=longint($80000000);
           end;
(*
         else {if allow_default_property(p) then
           begin
              p.default:=longint($80000000);
           end;
<<<<<<< HEAD
=======
(*
         else {if allow_default_property(p) then
           begin
              p.default:=longint($80000000);
           end;
>>>>>>> origin/fixes_2_2
*)
         { Parse possible "implements" keyword }
         if not is_record(astruct) and try_to_consume(_IMPLEMENTS) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           repeat
=======
           begin
>>>>>>> graemeg/cpstrnew
=======
           begin
>>>>>>> graemeg/cpstrnew
=======
           begin
>>>>>>> graemeg/cpstrnew
=======
           begin
>>>>>>> origin/cpstrnew
             single_type(def,[]);

             if not(is_interface(def)) then
               message(parser_e_class_implements_must_be_interface);

             if is_interface(p.propdef) then
               begin
                 { an interface type may delegate itself or one of its ancestors }
                 if not def_is_related(p.propdef,def) then
                   begin
                     message2(parser_e_implements_must_have_correct_type,def.typename,p.propdef.typename);
                     exit;
                   end;
               end
             else if is_class(p.propdef) then
               begin
                 ImplIntf:=find_implemented_interface(tobjectdef(p.propdef),tobjectdef(def));
                 if assigned(ImplIntf) then
                   begin
                     if compare_defs(ImplIntf.IntfDef,def,nothingn)<te_equal then
                       begin
                         message2(parser_e_implements_must_have_correct_type,ImplIntf.IntfDef.typename,def.typename);
                         exit;
                       end;
                   end
                 else
                   begin
                     message2(parser_e_class_doesnt_implement_interface,p.propdef.typename,def.typename);
                     exit;
                   end;
               end
             else
               begin
                 message(parser_e_implements_must_be_class_or_interface);
                 exit;
               end;


             if not assigned(p.propaccesslist[palt_read].firstsym) then
               begin
                 message(parser_e_implements_must_read_specifier);
                 exit;
               end;
             if assigned(p.propaccesslist[palt_read].procdef) and
                (tprocdef(p.propaccesslist[palt_read].procdef).proccalloption<>pocall_default) then
               message(parser_e_implements_getter_not_default_cc);
             if assigned(p.propaccesslist[palt_write].firstsym) then
               begin
                 message(parser_e_implements_must_not_have_write_specifier);
                 exit;
               end;
             if assigned(p.propaccesslist[palt_stored].firstsym) then
               begin
                 message(parser_e_implements_must_not_have_stored_specifier);
                 exit;
               end;
             found:=false;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             ImplIntf:=nil;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
             for i:=0 to tobjectdef(astruct).ImplementedInterfaces.Count-1 do
               begin
                 ImplIntf:=TImplementedInterface(tobjectdef(astruct).ImplementedInterfaces[i]);

                 if compare_defs(def,ImplIntf.IntfDef,nothingn)>=te_equal then
                   begin
                     found:=true;
                     break;
                   end;
               end;
             if found then
               begin
                 { An interface may not be delegated by more than one property,
                   it also may not have method mappings. }
                 if Assigned(ImplIntf.ImplementsGetter) then
                   message1(parser_e_duplicate_implements_clause,ImplIntf.IntfDef.typename);
                 if Assigned(ImplIntf.NameMappings) then
                   message2(parser_e_mapping_no_implements,ImplIntf.IntfDef.typename,astruct.objrealname^);

                 ImplIntf.ImplementsGetter:=p;
                 ImplIntf.VtblImplIntf:=ImplIntf;
                 case p.propaccesslist[palt_read].firstsym^.sym.typ of
                   procsym :
                     begin
                       if (po_virtualmethod in tprocdef(p.propaccesslist[palt_read].procdef).procoptions) and
                           not is_objectpascal_helper(tprocdef(p.propaccesslist[palt_read].procdef).struct) then
                         ImplIntf.IType:=etVirtualMethodResult
                       else
                         ImplIntf.IType:=etStaticMethodResult;
                     end;
                   fieldvarsym :
                     begin
                       ImplIntf.IType:=etFieldValue;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       { this must be done in a more robust way. Can't read the
                         fieldvarsym's fieldoffset yet, because it may not yet
                         be set }
                       ImplIntf.ImplementsField:=p.propaccesslist[palt_read].firstsym^.sym;
=======
                       { this must be done more sophisticated, here is also probably the wrong place }
                       ImplIntf.IOffset:=tfieldvarsym(p.propaccesslist[palt_read].firstsym^.sym).fieldoffset;
>>>>>>> graemeg/cpstrnew
=======
                       { this must be done more sophisticated, here is also probably the wrong place }
                       ImplIntf.IOffset:=tfieldvarsym(p.propaccesslist[palt_read].firstsym^.sym).fieldoffset;
>>>>>>> graemeg/cpstrnew
=======
                       { this must be done more sophisticated, here is also probably the wrong place }
                       ImplIntf.IOffset:=tfieldvarsym(p.propaccesslist[palt_read].firstsym^.sym).fieldoffset;
>>>>>>> graemeg/cpstrnew
=======
                       { this must be done more sophisticated, here is also probably the wrong place }
                       ImplIntf.IOffset:=tfieldvarsym(p.propaccesslist[palt_read].firstsym^.sym).fieldoffset;
>>>>>>> origin/cpstrnew
=======
                       { this must be done more sophisticated, here is also probably the wrong place }
                       ImplIntf.IOffset:=tfieldvarsym(p.propaccesslist[palt_read].firstsym^.sym).fieldoffset;
>>>>>>> origin/fixes_2.4
                     end
                   else
                     internalerror(200802161);
                 end;
                 if not is_interface(p.propdef) then
                   case ImplIntf.IType of
                     etVirtualMethodResult: ImplIntf.IType := etVirtualMethodClass;
                     etStaticMethodResult:  ImplIntf.IType := etStaticMethodClass;
                     etFieldValue:          ImplIntf.IType := etFieldValueClass;
                   else
                     internalerror(200912101);
                   end;
               end
             else
               message1(parser_e_implements_uses_non_implemented_interface,def.typename);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           until not try_to_consume(_COMMA);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         end;
>>>>>>> graemeg/cpstrnew

         { remove unneeded procdefs }
         if readprocdef.proctypeoption<>potype_propgetter then
           readprocdef.owner.deletedef(readprocdef);
         if writeprocdef.proctypeoption<>potype_propsetter then
           writeprocdef.owner.deletedef(writeprocdef);

         result:=p;
      end;


     function maybe_parse_proc_directives(def:tdef):boolean;
       var
         newtype : ttypesym;
       begin
         result:=false;
         { Process procvar directives before = and ; }
         if (def.typ=procvardef) and
            (def.typesym=nil) and
            check_proc_directive(true) then
           begin
              newtype:=ctypesym.create('unnamed',def,true);
              parse_var_proc_directives(tsym(newtype));
              newtype.typedef:=nil;
              def.typesym:=nil;
              newtype.free;
              result:=true;
           end;
       end;


    const
       variantrecordlevel : longint = 0;


    procedure read_public_and_external_sc(sc:TFPObjectList);
    var
      vs: tabstractvarsym;
    begin
      { only allowed for one var }
      vs:=tabstractvarsym(sc[0]);
      if sc.count>1 then
<<<<<<< HEAD
<<<<<<< HEAD
        Message1(parser_e_directive_only_one_var,arraytokeninfo[idtoken].str);
=======
        Message(parser_e_absolute_only_one_var);
>>>>>>> graemeg/fixes_2_2
=======
        Message(parser_e_absolute_only_one_var);
>>>>>>> origin/fixes_2_2
      read_public_and_external(vs);
    end;


    procedure read_public_and_external(vs: tabstractvarsym);
    var
      is_dll,
      is_cdecl,
      is_external_var,
<<<<<<< HEAD
<<<<<<< HEAD
      is_weak_external,
      is_public_var  : boolean;
      dll_name,section_name,
      C_name,mangledname      : string;
=======
      is_public_var  : boolean;
      dll_name,
      C_name      : string;
>>>>>>> graemeg/fixes_2_2
=======
      is_public_var  : boolean;
      dll_name,
      C_name      : string;
>>>>>>> origin/fixes_2_2
    begin
      { only allowed for one var }
      { only allow external and public on global symbols }
      if vs.typ<>staticvarsym then
        begin
          Message(parser_e_no_local_var_external);
          exit;
        end;
      { defaults }
      is_dll:=false;
      is_cdecl:=false;
      is_external_var:=false;
      is_public_var:=false;
<<<<<<< HEAD
<<<<<<< HEAD
      section_name := '';
      dll_name := '';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      C_name:=vs.realname;

      { macpas specific handling due to some switches}
      if (m_mac in current_settings.modeswitches) then
        begin
          if (cs_external_var in current_settings.localswitches) then
            begin {The effect of this is the same as if cvar; external; has been given as directives.}
              is_cdecl:=true;
              is_external_var:=true;
            end
          else if (cs_externally_visible in current_settings.localswitches) then
            begin {The effect of this is the same as if cvar has been given as directives and it's made public.}
              is_cdecl:=true;
              is_public_var:=true;
            end;
        end;

      { cdecl }
      if try_to_consume(_CVAR) then
        begin
          consume(_SEMICOLON);
          is_cdecl:=true;
        end;

      { external }
<<<<<<< HEAD
<<<<<<< HEAD
      is_weak_external:=try_to_consume(_WEAKEXTERNAL);
      if is_weak_external or
         try_to_consume(_EXTERNAL) then
        begin
          is_external_var:=true;
          if (idtoken<>_NAME) and (token<>_SEMICOLON) then
            begin
              is_dll:=true;
              dll_name:=get_stringconst;
              if ExtractFileExt(dll_name)='' then
                dll_name:=ChangeFileExt(dll_name,target_info.sharedlibext);
            end;
          if not(is_cdecl) and try_to_consume(_NAME) then
            C_name:=get_stringconst;
=======
=======
>>>>>>> origin/fixes_2_2
      if try_to_consume(_EXTERNAL) then
        begin
          is_external_var:=true;
          if not is_cdecl then
            begin
              if idtoken<>_NAME then
                begin
                  is_dll:=true;
                  dll_name:=get_stringconst;
                  if ExtractFileExt(dll_name)='' then
                    dll_name:=ChangeFileExt(dll_name,target_info.sharedlibext);
                end;
              if try_to_consume(_NAME) then
                C_name:=get_stringconst;
            end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          consume(_SEMICOLON);
        end;

      { export or public }
      if idtoken in [_EXPORT,_PUBLIC] then
        begin
          consume(_ID);
          if is_external_var then
            Message(parser_e_not_external_and_export)
          else
            is_public_var:=true;
          if try_to_consume(_NAME) then
            C_name:=get_stringconst;
<<<<<<< HEAD
<<<<<<< HEAD
          if (target_info.system in systems_allow_section_no_semicolon) and
             (vs.typ=staticvarsym) and
             try_to_consume (_SECTION) then
            section_name:=get_stringconst;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          consume(_SEMICOLON);
        end;

      { Windows uses an indirect reference using import tables }
      if is_dll and
<<<<<<< HEAD
<<<<<<< HEAD
         (target_info.system in systems_all_windows) then
        include(vs.varoptions,vo_is_dll_var);

      { This can only happen if vs.typ=staticvarsym }
      if section_name<>'' then
        begin
          tstaticvarsym(vs).section:=section_name;
          include(vs.varoptions,vo_has_section);
        end;


=======
         (target_info.system in system_all_windows) then
        include(vs.varoptions,vo_is_dll_var);

>>>>>>> graemeg/fixes_2_2
=======
         (target_info.system in system_all_windows) then
        include(vs.varoptions,vo_is_dll_var);

>>>>>>> origin/fixes_2_2
      { Add C _ prefix }
      if is_cdecl or
         (
          is_dll and
          (target_info.system in systems_darwin)
         ) then
        C_Name := target_info.Cprefix+C_Name;

      if is_public_var then
        begin
          include(vs.varoptions,vo_is_public);
          vs.varregable := vr_none;
          { mark as referenced }
          inc(vs.refs);
        end;

<<<<<<< HEAD
<<<<<<< HEAD
      mangledname:=C_name;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      { now we can insert it in the import lib if its a dll, or
        add it to the externals }
      if is_external_var then
        begin
          if vo_is_typed_const in vs.varoptions then
            Message(parser_e_initialized_not_for_external);
          include(vs.varoptions,vo_is_external);
<<<<<<< HEAD
<<<<<<< HEAD
          if (is_weak_external) then
            begin
              if not(target_info.system in systems_weak_linking) then
                message(parser_e_weak_external_not_supported);
              include(vs.varoptions,vo_is_weak_external);
            end;
          vs.varregable := vr_none;
          if is_dll then
            begin
              if target_info.system in (systems_all_windows + systems_nativent +
                                       [system_i386_emx, system_i386_os2]) then
                mangledname:=make_dllmangledname(dll_name,C_name,0,pocall_none);

              current_module.AddExternalImport(dll_name,C_Name,mangledname,0,true,false);
            end
=======
          vs.varregable := vr_none;
          if is_dll then
            current_module.AddExternalImport(dll_name,C_Name,0,true,false)
>>>>>>> graemeg/fixes_2_2
=======
          vs.varregable := vr_none;
          if is_dll then
            current_module.AddExternalImport(dll_name,C_Name,0,true,false)
>>>>>>> origin/fixes_2_2
          else
            if tf_has_dllscanner in target_info.flags then
              current_module.dllscannerinputlist.Add(vs.mangledname,vs);
        end;

      { Set the assembler name }
<<<<<<< HEAD
<<<<<<< HEAD
      tstaticvarsym(vs).set_mangledbasename(mangledname);
      tstaticvarsym(vs).set_mangledname(mangledname);
    end;


    procedure try_consume_sectiondirective(var asection: ansistring);
      begin
        if idtoken=_SECTION then
          begin
            consume(_ID);
            asection:=get_stringconst;
            consume(_SEMICOLON);
          end;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure try_read_field_external(vs: tabstractvarsym);
      var
        extname: string;
      begin
        if try_to_consume(_EXTERNAL) then
          begin
            consume(_NAME);
            extname:=get_stringconst;
            tfieldvarsym(vs).set_externalname(extname);
            consume(_SEMICOLON);
          end;
      end;


    procedure try_read_field_external_sc(sc:TFPObjectList);
    var
      vs: tabstractvarsym;
    begin
      { only allowed for one var }
      vs:=tabstractvarsym(sc[0]);
      if sc.count>1 then
        Message1(parser_e_directive_only_one_var,arraytokeninfo[idtoken].str);
      try_read_field_external(vs);
    end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure read_var_decls(options:Tvar_dec_options;out had_generic:boolean);
=======
=======
>>>>>>> graemeg/cpstrnew
    procedure try_consume_sectiondirective(var asection: ansistring);
      begin
        if idtoken=_SECTION then
          begin
            consume(_ID);
            asection:=get_stringconst;
            consume(_SEMICOLON);
          end;
      end;


<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
      tstaticvarsym(vs).set_mangledname(C_Name);
    end;


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure read_var_decls(options:Tvar_dec_options);
>>>>>>> graemeg/cpstrnew

        procedure read_default_value(sc : TFPObjectList);
        var
          vs : tabstractnormalvarsym;
          tcsym : tstaticvarsym;
        begin
          vs:=tabstractnormalvarsym(sc[0]);
          if sc.count>1 then
            Message(parser_e_initialized_only_one_var);
          if vo_is_thread_var in vs.varoptions then
            Message(parser_e_initialized_not_for_threadvar);
          consume(_EQ);
          case vs.typ of
            localvarsym :
              begin
                tcsym:=cstaticvarsym.create('$default'+vs.realname,vs_const,vs.vardef,[],true);
                include(tcsym.symoptions,sp_internal);
                vs.defaultconstsym:=tcsym;
                symtablestack.top.insert(tcsym);
                read_typed_const(current_asmdata.asmlists[al_typedconsts],tcsym,false);
              end;
            staticvarsym :
              begin
                read_typed_const(current_asmdata.asmlists[al_typedconsts],tstaticvarsym(vs),false);
              end;
            else
              internalerror(200611051);
          end;
          vs.varstate:=vs_initialised;
        end;

{$ifdef gpc_mode}
        procedure read_gpc_name(sc : TFPObjectList);
        var
          vs : tabstractnormalvarsym;
          C_Name : string;
        begin
          consume(_ID);
          C_Name:=get_stringconst;
          vs:=tabstractnormalvarsym(sc[0]);
          if sc.count>1 then
            Message(parser_e_directive_only_one_var,'ABSOLUTE');
          if vs.typ=staticvarsym then
            begin
              tstaticvarsym(vs).set_mangledname(C_Name);
              include(vs.varoptions,vo_is_external);
            end
          else
            Message(parser_e_no_local_var_external);
        end;
{$endif}

        procedure read_absolute(sc : TFPObjectList);
        var
          vs     : tabstractvarsym;
          abssym : tabsolutevarsym;
          pt,hp  : tnode;
          st     : tsymtable;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          {$if defined(i386) or defined(i8086)}
          tmpaddr : int64;
          {$endif defined(i386) or defined(i8086)}
=======
          {$ifdef i386}
          tmpaddr : int64;
          {$endif}
>>>>>>> graemeg/cpstrnew
=======
          {$ifdef i386}
          tmpaddr : int64;
          {$endif}
>>>>>>> graemeg/cpstrnew
=======
          {$ifdef i386}
          tmpaddr : int64;
          {$endif}
>>>>>>> graemeg/cpstrnew
=======
          {$ifdef i386}
          tmpaddr : int64;
          {$endif}
>>>>>>> origin/cpstrnew
=======
          {$ifdef i386}
          tmpaddr : int64;
          {$endif}
>>>>>>> origin/fixes_2.4
        begin
          abssym:=nil;
          { only allowed for one var }
          vs:=tabstractvarsym(sc[0]);
          if sc.count>1 then
            Message1(parser_e_directive_only_one_var,'ABSOLUTE');
          if vo_is_typed_const in vs.varoptions then
            Message(parser_e_initialized_not_for_external);
          { parse the rest }
          pt:=expr(true);
          { check allowed absolute types }
          if (pt.nodetype=stringconstn) or
            (is_constcharnode(pt)) then
            begin
              abssym:=cabsolutevarsym.create(vs.realname,vs.vardef);
              abssym.fileinfo:=vs.fileinfo;
              if pt.nodetype=stringconstn then
                abssym.asmname:=stringdup(strpas(tstringconstnode(pt).value_str))
              else
                abssym.asmname:=stringdup(chr(tordconstnode(pt).value.svalue));
              consume(token);
              abssym.abstyp:=toasm;
            end
          { address }
          else if is_constintnode(pt) then
            begin
              abssym:=cabsolutevarsym.create(vs.realname,vs.vardef);
              abssym.fileinfo:=vs.fileinfo;
              abssym.abstyp:=toaddr;
{$ifndef cpu64bitaddr}
              { on 64 bit systems, abssym.addroffset is a qword and hence this
                test is useless (value is a 64 bit entity) and will always fail
                for positive values (since int64(high(abssym.addroffset))=-1
              }
              if (Tordconstnode(pt).value<int64(low(abssym.addroffset))) or
                 (Tordconstnode(pt).value>int64(high(abssym.addroffset))) then
                message3(type_e_range_check_error_bounds,tostr(Tordconstnode(pt).value),tostr(low(abssym.addroffset)),tostr(high(abssym.addroffset)))
             else
{$endif}
                abssym.addroffset:=Tordconstnode(pt).value.svalue;
{$if defined(i386) or defined(i8086)}
              tcpuabsolutevarsym(abssym).absseg:=false;
              if (target_info.system in [system_i386_go32v2,system_i386_watcom,system_i8086_msdos,system_i8086_win16]) and
                  try_to_consume(_COLON) then
                begin
                  pt.free;
                  pt:=expr(true);
                  if is_constintnode(pt) then
                    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                      {$if defined(i8086)}
                        tcpuabsolutevarsym(abssym).addrsegment:=abssym.addroffset;
                        tmpaddr:=tordconstnode(pt).value.svalue;
                        if (tmpaddr<int64(low(abssym.addroffset))) or
                           (tmpaddr>int64(high(abssym.addroffset))) then
                          message3(type_e_range_check_error_bounds,tostr(Tordconstnode(pt).value),tostr(low(abssym.addroffset)),tostr(high(abssym.addroffset)))
                        else
                          abssym.addroffset:=tmpaddr;
                      {$elseif defined(i386)}
                        tmpaddr:=abssym.addroffset shl 4+tordconstnode(pt).value.svalue;
                        if (tmpaddr<int64(low(abssym.addroffset))) or
                           (tmpaddr>int64(high(abssym.addroffset))) then
                          message3(type_e_range_check_error_bounds,tostr(Tordconstnode(pt).value),tostr(low(abssym.addroffset)),tostr(high(abssym.addroffset)))
                        else
                          abssym.addroffset:=tmpaddr;
                      {$endif}
                      tcpuabsolutevarsym(abssym).absseg:=true;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                      tmpaddr:=abssym.addroffset shl 4+tordconstnode(pt).value.svalue;
                      if (tmpaddr<int64(low(abssym.addroffset))) or
                         (tmpaddr>int64(high(abssym.addroffset))) then
                        message(parser_e_range_check_error)
                      else
                        abssym.addroffset:=tmpaddr;
                      abssym.absseg:=true;
>>>>>>> graemeg/cpstrnew
                    end
                  else
                    Message(type_e_ordinal_expr_expected);
                end;
{$endif i386 or i8086}
            end
          { variable }
          else
            begin
              { we have to be able to take the address of the absolute
                expression
              }
              valid_for_addr(pt,true);
              { remove subscriptn before checking for loadn }
              hp:=pt;
              while (hp.nodetype in [subscriptn,typeconvn,vecn]) do
                begin
                  { check for implicit dereferencing and reject it }
                  if (hp.nodetype in [subscriptn,vecn]) then
                    begin
                      if (tunarynode(hp).left.resultdef.typ in [pointerdef,classrefdef]) then
                        break;
                      { catch, e.g., 'var b: char absolute pchar_var[5];"
                        (pchar_var[5] is a pchar_2_string typeconv ->
                         the vecn only sees an array of char)
                        I don't know if all of these type conversions are
                        possible, but they're definitely all bad.
                      }
                      if (tunarynode(hp).left.nodetype=typeconvn) and
                         (ttypeconvnode(tunarynode(hp).left).convtype in
                           [tc_pchar_2_string,tc_pointer_2_array,
                            tc_intf_2_string,tc_intf_2_guid,
                            tc_dynarray_2_variant,tc_interface_2_variant,
                            tc_array_2_dynarray]) then
                        break;

                      if (tunarynode(hp).left.resultdef.typ=stringdef) and
                         not(tstringdef(tunarynode(hp).left.resultdef).stringtype in [st_shortstring,st_longstring]) then
                        break;
                      if (tunarynode(hp).left.resultdef.typ=objectdef) and
                         (tobjectdef(tunarynode(hp).left.resultdef).objecttype<>odt_object) then
                        break;
                      if is_dynamic_array(tunarynode(hp).left.resultdef) then
                        break;
                    end;
                  hp:=tunarynode(hp).left;
                end;
              if (hp.nodetype=loadn) then
                begin
                  { we should check the result type of loadn }
                  if not (tloadnode(hp).symtableentry.typ in [fieldvarsym,staticvarsym,localvarsym,paravarsym]) then
                    Message(parser_e_absolute_only_to_var_or_const);
                  abssym:=cabsolutevarsym.create(vs.realname,vs.vardef);
                  abssym.fileinfo:=vs.fileinfo;
                  abssym.abstyp:=tovar;
                  abssym.ref:=node_to_propaccesslist(pt);

                  { if the sizes are different, can't be a regvar since you }
                  { can't be "absolute upper 8 bits of a register" (except  }
                  { if its a record field of the same size of a record      }
                  { regvar, but in that case pt.resultdef.size will have    }
                  { the same size since it refers to the field and not to   }
                  { the whole record -- which is why we use pt and not hp)  }

                  { we can't take the size of an open array }
                  if is_open_array(pt.resultdef) or
                     (vs.vardef.size <> pt.resultdef.size) then
                    make_not_regable(pt,[ra_addr_regable]);
                end
              else
                Message(parser_e_absolute_only_to_var_or_const);
            end;
          pt.free;
          { replace old varsym with the new absolutevarsym }
          if assigned(abssym) then
            begin
              st:=vs.owner;
              vs.owner.Delete(vs);
              st.insert(abssym);
              sc[0]:=abssym;
            end;
        end;

<<<<<<< HEAD
=======
        procedure read_public_and_external(sc:TFPObjectList);
        var
          vs          : tabstractvarsym;
          is_dll,
          is_cdecl,
          is_external_var,
          is_public_var  : boolean;
          dll_name,
          C_name      : string;
        begin
          { only allowed for one var }
          vs:=tabstractvarsym(sc[0]);
          if sc.count>1 then
            Message(parser_e_absolute_only_one_var);
          { only allow external and public on global symbols }
          if vs.typ<>staticvarsym then
            begin
              Message(parser_e_no_local_var_external);
              exit;
            end;
          { defaults }
          is_dll:=false;
          is_cdecl:=false;
          is_external_var:=false;
          is_public_var:=false;
          C_name:=vs.realname;

          { macpas specific handling due to some switches}
          if (m_mac in current_settings.modeswitches) then
            begin
              if (cs_external_var in current_settings.localswitches) then
                begin {The effect of this is the same as if cvar; external; has been given as directives.}
                  is_cdecl:=true;
                  is_external_var:=true;
                end
              else if (cs_externally_visible in current_settings.localswitches) then
                begin {The effect of this is the same as if cvar has been given as directives and it's made public.}
                  is_cdecl:=true;
                  is_public_var:=true;
                end;
            end;

          { cdecl }
          if try_to_consume(_CVAR) then
            begin
              consume(_SEMICOLON);
              is_cdecl:=true;
            end;

          { external }
          if try_to_consume(_EXTERNAL) then
            begin
              is_external_var:=true;
              if not is_cdecl then
                begin
                  if idtoken<>_NAME then
                    begin
                      is_dll:=true;
                      dll_name:=get_stringconst;
                      if ExtractFileExt(dll_name)='' then
                        dll_name:=ChangeFileExt(dll_name,target_info.sharedlibext);
                    end;
                  if try_to_consume(_NAME) then
                    C_name:=get_stringconst;
                end;
              consume(_SEMICOLON);
            end;

          { export or public }
          if idtoken in [_EXPORT,_PUBLIC] then
            begin
              consume(_ID);
              if is_external_var then
                Message(parser_e_not_external_and_export)
              else
                is_public_var:=true;
              if try_to_consume(_NAME) then
                C_name:=get_stringconst;
              consume(_SEMICOLON);
            end;

          { Windows uses an indirect reference using import tables }
          if is_dll and
             (target_info.system in system_all_windows) then
            include(vs.varoptions,vo_is_dll_var);

          { Add C _ prefix }
          if is_cdecl or
             (
              is_dll and
              (target_info.system in systems_darwin)
             ) then
            C_Name := target_info.Cprefix+C_Name;

          if is_public_var then
            begin
              include(vs.varoptions,vo_is_public);
              vs.varregable := vr_none;
              { mark as referenced }
              inc(vs.refs);
            end;

          { now we can insert it in the import lib if its a dll, or
            add it to the externals }
          if is_external_var then
            begin
              if vo_is_typed_const in vs.varoptions then
                Message(parser_e_initialized_not_for_external);
              include(vs.varoptions,vo_is_external);
              vs.varregable := vr_none;
              if is_dll then
                current_module.AddExternalImport(dll_name,C_Name,0,true,false)
              else
                if tf_has_dllscanner in target_info.flags then
                  current_module.dllscannerinputlist.Add(vs.mangledname,vs);
            end;

          { Set the assembler name }
          tstaticvarsym(vs).set_mangledname(C_Name);
        end;

>>>>>>> graemeg/fixes_2_2
      var
         sc   : TFPObjectList;
         vs   : tabstractvarsym;
         hdef : tdef;
         i    : longint;
         first,
         isgeneric,
         semicoloneaten,
         allowdefaultvalue,
         hasdefaultvalue : boolean;
         hintsymoptions  : tsymoptions;
         deprecatedmsg   : pshortstring;
         old_block_type  : tblock_type;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         sectionname : ansistring;
         tmp_filepos,
         old_current_filepos     : tfileposinfo;
=======
         section : ansistring;
>>>>>>> graemeg/cpstrnew
=======
         section : ansistring;
>>>>>>> graemeg/cpstrnew
=======
         section : ansistring;
>>>>>>> graemeg/cpstrnew
=======
         section : ansistring;
>>>>>>> origin/cpstrnew
      begin
         old_block_type:=block_type;
         block_type:=bt_var;
         { Force an expected ID error message }
         if not (token in [_ID,_CASE,_END]) then
           consume(_ID);
         { read vars }
         sc:=TFPObjectList.create(false);
         first:=true;
         had_generic:=false;
         vs:=nil;
         fillchar(tmp_filepos,sizeof(tmp_filepos),0);
         while (token=_ID) do
           begin
             semicoloneaten:=false;
             hasdefaultvalue:=false;
             allowdefaultvalue:=true;
             sc.clear;
             repeat
               if (token = _ID) then
                 begin
                   isgeneric:=(vd_check_generic in options) and
                                not (m_delphi in current_settings.modeswitches) and
                                (idtoken=_GENERIC);
                   case symtablestack.top.symtabletype of
                     localsymtable :
                       vs:=clocalvarsym.create(orgpattern,vs_value,generrordef,[],false);
                     staticsymtable,
                     globalsymtable :
                       begin
                         vs:=cstaticvarsym.create(orgpattern,vs_value,generrordef,[],false);
                         if vd_threadvar in options then
                           include(vs.varoptions,vo_is_thread_var);
                       end;
                     else
                       internalerror(200411064);
                   end;
                   sc.add(vs);
                   if isgeneric then
                     tmp_filepos:=current_filepos;
                 end
               else
                 isgeneric:=false;
               consume(_ID);
               { when the first variable had been read the next declaration could be
                 a "generic procedure", "generic function" or
                 "generic class (function/procedure)" }
               if not first
                   and isgeneric
                   and (sc.count=1)
                   and (token in [_PROCEDURE,_FUNCTION,_CLASS]) then
                 begin
                   vs.free;
                   sc.clear;
                   had_generic:=true;
                   break;
                 end
               else
                 begin
                   vs.register_sym;
                   symtablestack.top.insert(vs);
                   if isgeneric then
                     begin
                       { ensure correct error position }
                       old_current_filepos:=current_filepos;
                       current_filepos:=tmp_filepos;
                       symtablestack.top.insert(vs);
                       current_filepos:=old_current_filepos;
                     end;
                 end;
             until not try_to_consume(_COMMA);

             if had_generic then
               break;

             { read variable type def }
             block_type:=bt_var_type;
             consume(_COLON);

{$ifdef gpc_mode}
             if (m_gpc in current_settings.modeswitches) and
                (token=_ID) and
                (orgpattern='__asmname__') then
               read_gpc_name(sc);
{$endif}

             read_anon_type(hdef,false);
             maybe_guarantee_record_typesym(hdef,symtablestack.top);
             for i:=0 to sc.count-1 do
               begin
                 vs:=tabstractvarsym(sc[i]);
                 vs.vardef:=hdef;
               end;
             block_type:=bt_var;

             { Process procvar directives }
             if maybe_parse_proc_directives(hdef) then
               semicoloneaten:=true;

             { check for absolute }
             if try_to_consume(_ABSOLUTE) then
               begin
                 read_absolute(sc);
                 allowdefaultvalue:=false;
               end;

             { Check for EXTERNAL etc directives before a semicolon }
             if (idtoken in [_EXPORT,_EXTERNAL,_WEAKEXTERNAL,_PUBLIC,_CVAR]) then
               begin
                 read_public_and_external_sc(sc);
                 allowdefaultvalue:=false;
                 semicoloneaten:=true;
               end;

             { try to parse the hint directives }
             hintsymoptions:=[];
             deprecatedmsg:=nil;
             try_consume_hintdirective(hintsymoptions,deprecatedmsg);
             for i:=0 to sc.count-1 do
               begin
                 vs:=tabstractvarsym(sc[i]);
                 vs.symoptions := vs.symoptions + hintsymoptions;
                 if deprecatedmsg<>nil then
                   vs.deprecatedmsg:=stringdup(deprecatedmsg^);
               end;
             stringdispose(deprecatedmsg);

             { Handling of Delphi typed const = initialized vars }
             if allowdefaultvalue and
                (token=_EQ) and
                not(m_tp7 in current_settings.modeswitches) and
                (symtablestack.top.symtabletype<>parasymtable) then
               begin
                 { Add calling convention for procvar }
                 if (hdef.typ=procvardef) and
                    (hdef.typesym=nil) then
                   handle_calling_convention(tprocvardef(hdef));
                 read_default_value(sc);
                 hasdefaultvalue:=true;
               end
             else
               begin
                 if not(semicoloneaten) then
                   consume(_SEMICOLON);
               end;

             { Support calling convention for procvars after semicolon }
             if not(hasdefaultvalue) and
                (hdef.typ=procvardef) and
                (hdef.typesym=nil) then
               begin
                 { Parse procvar directives after ; }
                 maybe_parse_proc_directives(hdef);
                 { Add calling convention for procvar }
                 handle_calling_convention(tprocvardef(hdef));
                 { Handling of Delphi typed const = initialized vars }
                 if (token=_EQ) and
                    not(m_tp7 in current_settings.modeswitches) and
                    (symtablestack.top.symtabletype<>parasymtable) then
                   begin
                     read_default_value(sc);
                     hasdefaultvalue:=true;
                   end;
               end;

             { Check for EXTERNAL etc directives or, in macpas, if cs_external_var is set}
             if (
                 (
                  (idtoken in [_EXPORT,_EXTERNAL,_WEAKEXTERNAL,_PUBLIC,_CVAR]) and
                  (m_cvar_support in current_settings.modeswitches)
                 ) or
                 (
                  (m_mac in current_settings.modeswitches) and
                  (
                   (cs_external_var in current_settings.localswitches) or
                   (cs_externally_visible in current_settings.localswitches)
                  )
                 )
                ) then
               read_public_and_external_sc(sc);
<<<<<<< HEAD
<<<<<<< HEAD

             { try to parse a section directive }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             if (target_info.system in systems_allow_section) and
                (symtablestack.top.symtabletype in [staticsymtable,globalsymtable]) and
                (idtoken=_SECTION) then
               begin
                 try_consume_sectiondirective(sectionname);
                 if sectionname<>'' then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
             if (target_info.system in systems_embedded) and (idtoken=_SECTION) then
               begin
                 try_consume_sectiondirective(section);
                 if section<>'' then
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
                     for i:=0 to sc.count-1 do
                       begin
                         vs:=tabstractvarsym(sc[i]);
                         if (vs.varoptions *[vo_is_external,vo_is_weak_external])<>[] then
                           Message(parser_e_externals_no_section);
                         if vs.typ<>staticvarsym then
                           Message(parser_e_section_no_locals);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                         tstaticvarsym(vs).section:=sectionname;
                         include(vs.varoptions, vo_has_section);
=======
                         tstaticvarsym(vs).section:=section;
>>>>>>> graemeg/cpstrnew
=======
                         tstaticvarsym(vs).section:=section;
>>>>>>> graemeg/cpstrnew
=======
                         tstaticvarsym(vs).section:=section;
>>>>>>> graemeg/cpstrnew
=======
                         tstaticvarsym(vs).section:=section;
>>>>>>> origin/cpstrnew
                       end;
                   end;
               end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

             { allocate normal variable (non-external and non-typed-const) staticvarsyms }
             for i:=0 to sc.count-1 do
               begin
                 vs:=tabstractvarsym(sc[i]);
                 if (vs.typ=staticvarsym) and
                    not(vo_is_typed_const in vs.varoptions) and
                    not(vo_is_external in vs.varoptions) then
                   cnodeutils.insertbssdata(tstaticvarsym(vs));
               end;

             first:=false;
           end;
         block_type:=old_block_type;
         { free the list }
         sc.free;
      end;


    procedure read_record_fields(options:Tvar_dec_options; reorderlist: TFPObjectList; variantdesc : ppvariantrecdesc;out had_generic:boolean);
      var
         sc : TFPObjectList;
         i  : longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         hs,sorg : string;
         hdef,casetype,tmpdef : tdef;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         hs,sorg,static_name : string;
         hdef,casetype : tdef;
>>>>>>> graemeg/cpstrnew
         { maxsize contains the max. size of a variant }
         { startvarrec contains the start of the variant part of a record }
         maxsize, startvarrecsize : longint;
         usedalign,
         maxalignment,startvarrecalign,
         maxpadalign, startpadalign: shortint;
         stowner : tdef;
         pt : tnode;
         fieldvs   : tfieldvarsym;
         hstaticvs : tstaticvarsym;
         vs    : tabstractvarsym;
         srsym : tsym;
         srsymtable : TSymtable;
         visibility : tvisibility;
         recst : tabstractrecordsymtable;
         recstlist : tfpobjectlist;
         unionsymtable : trecordsymtable;
         offset : longint;
         uniondef : trecorddef;
         hintsymoptions : tsymoptions;
         deprecatedmsg : pshortstring;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         hadgendummy,
         semicoloneaten,
         removeclassoption: boolean;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         semicoloneaten: boolean;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
{$if defined(powerpc) or defined(powerpc64)}
         tempdef: tdef;
         is_first_type: boolean;
{$endif powerpc or powerpc64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
         sl: tpropaccesslist;
>>>>>>> graemeg/cpstrnew
=======
         sl: tpropaccesslist;
>>>>>>> graemeg/cpstrnew
=======
         sl: tpropaccesslist;
>>>>>>> graemeg/cpstrnew
=======
         sl: tpropaccesslist;
>>>>>>> origin/cpstrnew
         old_block_type: tblock_type;
      begin
=======
=======
>>>>>>> origin/fixes_2_2
{$if defined(powerpc) or defined(powerpc64)}
         tempdef: tdef;
         is_first_field: boolean;
{$endif powerpc or powerpc64}
      begin
         recst:=tabstractrecordsymtable(symtablestack.top);
{$if defined(powerpc) or defined(powerpc64)}
         is_first_field := true;
{$endif powerpc or powerpc64}
         old_current_object_option:=current_object_option;
         { all variables are public if not in a object declaration }
         if not(vd_object in options) then
          current_object_option:=[sp_public];
>>>>>>> graemeg/fixes_2_2
         old_block_type:=block_type;
         block_type:=bt_var;
         recst:=tabstractrecordsymtable(symtablestack.top);
{$if defined(powerpc) or defined(powerpc64)}
         is_first_type:=true;
{$endif powerpc or powerpc64}
         { Force an expected ID error message }
         if not (token in [_ID,_CASE,_END]) then
           consume(_ID);
         { read vars }
         sc:=TFPObjectList.create(false);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         removeclassoption:=false;
         had_generic:=false;
         while (token=_ID) and
            not(((vd_object in options) or
                 ((vd_record in options) and (m_advanced_records in current_settings.modeswitches))) and
                ((idtoken in [_PUBLIC,_PRIVATE,_PUBLISHED,_PROTECTED,_STRICT]) or
                 ((m_final_fields in current_settings.modeswitches) and
                  (idtoken=_FINAL)))) do
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         recstlist:=TFPObjectList.create(false);;
         while (token=_ID) and
            not(((vd_object in options) or
                 ((vd_record in options) and (m_advanced_records in current_settings.modeswitches))) and
                (idtoken in [_PUBLIC,_PRIVATE,_PUBLISHED,_PROTECTED,_STRICT])) do
>>>>>>> graemeg/cpstrnew
           begin
<<<<<<< HEAD
<<<<<<< HEAD
             visibility:=symtablestack.top.currentvisibility;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             semicoloneaten:=false;
             sc.clear;
             repeat
               sorg:=orgpattern;
               if token=_ID then
<<<<<<< HEAD
<<<<<<< HEAD
                 begin
                   vs:=cfieldvarsym.create(sorg,vs_value,generrordef,[],false);

                   { normally the visibility is set via addfield, but sometimes
                     we collect symbols so we can add them in a batch of
                     potentially mixed visibility, and then the individual
                     symbols need to have their visibility already set }
                   vs.visibility:=visibility;
                   if (vd_check_generic in options) and (idtoken=_GENERIC) then
                     had_generic:=true;
                 end
               else
                 vs:=nil;
               consume(_ID);
               if assigned(vs) and
                  (
                    not had_generic or
                    not (token in [_PROCEDURE,_FUNCTION,_CLASS])
                  ) then
                 begin
                   vs.register_sym;
                   sc.add(vs);
                   recst.insert(vs);
                   had_generic:=false;
                 end
               else
                 vs.free;
=======
=======
>>>>>>> origin/fixes_2_2
                 begin
                   vs:=tfieldvarsym.create(sorg,vs_value,generrordef,[]);
                   sc.add(vs);
                   recst.insert(vs);
                 end;
               consume(_ID);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             until not try_to_consume(_COMMA);
             if m_delphi in current_settings.modeswitches then
               block_type:=bt_var_type
             else
               block_type:=old_block_type;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             if had_generic and (sc.count=0) then
               break;
             consume(_COLON);

             read_anon_type(hdef,false);
             maybe_guarantee_record_typesym(hdef,symtablestack.top);
             block_type:=bt_var;
             { allow only static fields reference to struct where they are declared }
             if not (vd_class in options) then
               begin
                 stowner:=tdef(recst.defowner);
                 while assigned(stowner) and (stowner.typ in [objectdef,recorddef]) do
                   begin
                     if hdef.typ=arraydef then
                       begin
                         tmpdef:=hdef;
                         while (tmpdef.typ=arraydef) do
                           begin
                             { dynamic arrays are allowed }
                             if ado_IsDynamicArray in tarraydef(tmpdef).arrayoptions then
                               begin
                                 tmpdef:=nil;
                                 break;
                               end;
                             tmpdef:=tarraydef(tmpdef).elementdef;
                           end;
                       end
                     else
                       tmpdef:=hdef;
                     if assigned(tmpdef) and
                         (is_object(tmpdef) or is_record(tmpdef)) and
                         is_owned_by(tabstractrecorddef(stowner),tabstractrecorddef(tmpdef)) then
                       begin
                         Message1(type_e_type_is_not_completly_defined, tabstractrecorddef(tmpdef).RttiName);
                         { for error recovery or compiler will crash later }
                         hdef:=generrordef;
                       end;
                     stowner:=tdef(stowner.owner.defowner);
                   end;
=======
             consume(_COLON);

             { Don't search for types where they can't be:
               types can be only in objects, classes and records.
               This just speedup the search a bit. }
             recstlist.count:=0;
             if not is_class_or_object(tdef(recst.defowner)) and
                not is_record(tdef(recst.defowner)) then
               begin
                 recstlist.add(recst);
                 symtablestack.pop(recst);
               end;
             read_anon_type(hdef,false);
=======
=======
>>>>>>> graemeg/cpstrnew
             consume(_COLON);

             { Don't search for types where they can't be:
               types can be only in objects, classes and records.
               This just speedup the search a bit. }
             recstlist.count:=0;
             if not is_class_or_object(tdef(recst.defowner)) and
                not is_record(tdef(recst.defowner)) then
               begin
                 recstlist.add(recst);
                 symtablestack.pop(recst);
               end;
             read_anon_type(hdef,false);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
             consume(_COLON);

             { Don't search for types where they can't be:
               types can be only in objects, classes and records.
               This just speedup the search a bit. }
             recstlist.count:=0;
             if not is_class_or_object(tdef(recst.defowner)) and
                not is_record(tdef(recst.defowner)) then
               begin
                 recstlist.add(recst);
                 symtablestack.pop(recst);
               end;
             read_anon_type(hdef,false);
>>>>>>> origin/cpstrnew
             block_type:=bt_var;
             { allow only static fields reference to struct where they are declared }
             if not (vd_class in options) and
               (is_object(hdef) or is_record(hdef)) and
               is_owned_by(tabstractrecorddef(recst.defowner),tabstractrecorddef(hdef)) then
               begin
                 Message1(type_e_type_is_not_completly_defined, tabstractrecorddef(hdef).RttiName);
                 { for error recovery or compiler will crash later }
                 hdef:=generrordef;
               end;
             { restore stack }
             for i:=recstlist.count-1 downto 0 do
               begin
                 recst:=tabstractrecordsymtable(recstlist[i]);
                 symtablestack.push(recst);
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
               end;

             { Process procvar directives }
             if maybe_parse_proc_directives(hdef) then
               semicoloneaten:=true;

{$if defined(powerpc) or defined(powerpc64)}
             { from gcc/gcc/config/rs6000/rs6000.h:
              /* APPLE LOCAL begin Macintosh alignment 2002-1-22 ff */
              /* Return the alignment of a struct based on the Macintosh PowerPC
                 alignment rules.  In general the alignment of a struct is
                 determined by the greatest alignment of its elements.  However, the
                 PowerPC rules cause the alignment of a struct to peg at word
                 alignment except when the first field has greater than word
                 (32-bit) alignment, in which case the alignment is determined by
                 the alignment of the first field.  */
             }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             { TODO: check whether this is also for AIX }
             if (target_info.abi in [abi_powerpc_aix,abi_powerpc_darwin]) and
=======
             if (target_info.system in [system_powerpc_darwin, system_powerpc_macos, system_powerpc64_darwin]) and
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
                is_first_type and
=======
             if (target_info.system in [system_powerpc_darwin, system_powerpc_macos, system_powerpc64_darwin]) and
                is_first_field and
>>>>>>> graemeg/fixes_2_2
=======
             if (target_info.system in [system_powerpc_darwin, system_powerpc_macos, system_powerpc64_darwin]) and
                is_first_field and
>>>>>>> origin/fixes_2_2
                (symtablestack.top.symtabletype=recordsymtable) and
                (trecordsymtable(symtablestack.top).usefieldalignment=C_alignment) then
               begin
                 tempdef:=hdef;
                 while tempdef.typ=arraydef do
                   tempdef:=tarraydef(tempdef).elementdef;
                 if tempdef.typ<>recorddef then
                   maxpadalign:=tempdef.alignment
                 else
                   maxpadalign:=trecorddef(tempdef).padalignment;

                 if (maxpadalign>4) and
                    (maxpadalign>trecordsymtable(symtablestack.top).padalignment) then
                   trecordsymtable(symtablestack.top).padalignment:=maxpadalign;
<<<<<<< HEAD
<<<<<<< HEAD
                 is_first_type:=false;
=======
                 is_first_field:=false;
>>>>>>> graemeg/fixes_2_2
=======
                 is_first_field:=false;
>>>>>>> origin/fixes_2_2
               end;
{$endif powerpc or powerpc64}

             { types that use init/final are not allowed in variant parts, but
               classes are allowed }
             if (variantrecordlevel>0) then
               if is_managed_type(hdef) then
                 Message(parser_e_cant_use_inittable_here)
               else
               if hdef.typ=undefineddef then
                 Message(parser_e_cant_use_type_parameters_here);

             { try to parse the hint directives }
             hintsymoptions:=[];
             deprecatedmsg:=nil;
             try_consume_hintdirective(hintsymoptions,deprecatedmsg);

             { update variable type and hints }
             for i:=0 to sc.count-1 do
               begin
                 fieldvs:=tfieldvarsym(sc[i]);
                 fieldvs.vardef:=hdef;
                 { insert any additional hint directives }
                 fieldvs.symoptions := fieldvs.symoptions + hintsymoptions;
                 if deprecatedmsg<>nil then
                   fieldvs.deprecatedmsg:=stringdup(deprecatedmsg^);
               end;
               stringdispose(deprecatedmsg);

             { Records and objects can't have default values }
             { for a record there doesn't need to be a ; before the END or )    }
             if not(token in [_END,_RKLAMMER]) and
                not(semicoloneaten) then
               consume(_SEMICOLON);

             { Parse procvar directives after ; }
             maybe_parse_proc_directives(hdef);

             { Add calling convention for procvar }
             if (hdef.typ=procvardef) and
                (hdef.typesym=nil) then
               handle_calling_convention(tprocvardef(hdef));

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             if (vd_object in options) then
=======
             { check if it is a class field }
             if (vd_object in options) then
=======
             { check if it is a class field }
             if (vd_object in options) then
=======
             { check if it is a class field }
             if (vd_object in options) then
=======
             { check if it is a class field }
             if (vd_object in options) then
               begin
                 { if it is not a class var section and token=STATIC then it is a class field too }
                 if not (vd_class in options) and try_to_consume(_STATIC) then
                   begin
                     consume(_SEMICOLON);
                     include(options, vd_class);
                   end;
               end;
             if vd_class in options then
>>>>>>> origin/cpstrnew
               begin
                 { if it is not a class var section and token=STATIC then it is a class field too }
                 if not (vd_class in options) and try_to_consume(_STATIC) then
                   begin
                     consume(_SEMICOLON);
                     include(options, vd_class);
                   end;
               end;
             if vd_class in options then
>>>>>>> graemeg/cpstrnew
               begin
                 { if it is not a class var section and token=STATIC then it is a class field too }
                 if not (vd_class in options) and try_to_consume(_STATIC) then
                   begin
                     consume(_SEMICOLON);
                     include(options, vd_class);
                   end;
               end;
             if vd_class in options then
>>>>>>> graemeg/cpstrnew
               begin
                 { if it is not a class var section and token=STATIC then it is a class field too }
                 if not (vd_class in options) and try_to_consume(_STATIC) then
                   begin
                     consume(_SEMICOLON);
                     include(options, vd_class);
                   end;
               end;
             if vd_class in options then
>>>>>>> graemeg/cpstrnew
               begin
                 { if it is not a class var section and token=STATIC then it is a class field too }
                 if not (vd_class in options) and try_to_consume(_STATIC) then
                   begin
<<<<<<< HEAD
                     consume(_SEMICOLON);
                     include(options,vd_class);
                     removeclassoption:=true;
                   end;
                 { Fields in Java classes/interfaces can have a separately
                   specified external name }
                 if is_java_class_or_interface(tdef(recst.defowner)) and
                    (oo_is_external in tobjectdef(recst.defowner).objectoptions) then
                   try_read_field_external_sc(sc);
=======
                     fieldvs:=tfieldvarsym(sc[i]);
                     include(fieldvs.symoptions,sp_static);
                     { generate the symbol which reserves the space }
                     static_name:=lower(generate_nested_name(recst,'_'))+'_'+fieldvs.name;
                     hstaticvs:=tstaticvarsym.create('$_static_'+static_name,vs_value,hdef,[]);
                     include(hstaticvs.symoptions,sp_internal);
                     recst.get_unit_symtable.insert(hstaticvs);
                     insertbssdata(hstaticvs);
                     { generate the symbol for the access }
                     sl:=tpropaccesslist.create;
                     sl.addsym(sl_load,hstaticvs);
                     recst.insert(tabsolutevarsym.create_ref('$'+static_name,hdef,sl));
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
               end;
             if (visibility=vis_published) and
                not(is_class(hdef)) then
               begin
                 MessagePos(tfieldvarsym(sc[0]).fileinfo,parser_e_cant_publish_that);
                 visibility:=vis_public;
               end;

             if (visibility=vis_published) and
                not(oo_can_have_published in tobjectdef(hdef).objectoptions) and
                not(m_delphi in current_settings.modeswitches) then
               begin
                 MessagePos(tfieldvarsym(sc[0]).fileinfo,parser_e_only_publishable_classes_can_be_published);
                 visibility:=vis_public;
               end;
             if vd_class in options then
               begin
                 { add static flag and staticvarsyms }
                 for i:=0 to sc.count-1 do
                   begin
                     fieldvs:=tfieldvarsym(sc[i]);
                     fieldvs.visibility:=visibility;
                     hstaticvs:=make_field_static(recst,fieldvs);
                     if not parse_generic then
                       cnodeutils.insertbssdata(hstaticvs);
                     if vd_final in options then
                       hstaticvs.varspez:=vs_final;
                   end;
                 if removeclassoption then
                   begin
                     exclude(options,vd_class);
                     removeclassoption:=false;
                   end;
<<<<<<< HEAD
=======
               end
             else
              if (sp_published in current_object_option) and
                 not(oo_can_have_published in tobjectdef(hdef).objectoptions) and
                 not(m_delphi in current_settings.modeswitches) then
               begin
                 Message(parser_e_only_publishable_classes_can_be_published);
                 exclude(current_object_option,sp_published);
>>>>>>> graemeg/fixes_2_2
               end;
             if vd_final in options then
               begin
                 { add final flag }
                 for i:=0 to sc.count-1 do
                   begin
                     fieldvs:=tfieldvarsym(sc[i]);
                     fieldvs.varspez:=vs_final;
                   end;
               end;

             if not(vd_canreorder in options) then
               { add field(s) to the recordsymtable }
               recst.addfieldlist(sc,false)
             else
               { we may reorder the fields before adding them to the symbol
                 table }
               reorderlist.concatlistcopy(sc)
           end;
          recstlist.free;

         if m_delphi in current_settings.modeswitches then
           block_type:=bt_var_type
         else
           block_type:=old_block_type;
         { Check for Case }
         if (vd_record in options) and
            try_to_consume(_CASE) then
           begin
              maxsize:=0;
              maxalignment:=0;
              maxpadalign:=0;

              { already inside a variant record? if not, setup a new variantdesc chain }
              if not(assigned(variantdesc)) then
                variantdesc:=@trecorddef(trecordsymtable(recst).defowner).variantrecdesc;

              { else just concat the info to the given one }
              new(variantdesc^);
              fillchar(variantdesc^^,sizeof(tvariantrecdesc),0);

              { including a field declaration? }
              fieldvs:=nil;
              sorg:=orgpattern;
              hs:=pattern;
              searchsym(hs,srsym,srsymtable);
              if not(assigned(srsym) and (srsym.typ in [typesym,unitsym])) then
                begin
                  consume(_ID);
                  consume(_COLON);
                  fieldvs:=cfieldvarsym.create(sorg,vs_value,generrordef,[],true);
                  variantdesc^^.variantselector:=fieldvs;
                  symtablestack.top.insert(fieldvs);
                end;
              read_anon_type(casetype,true);
              block_type:=bt_var;
              if assigned(fieldvs) then
                begin
                  fieldvs.vardef:=casetype;
                  recst.addfield(fieldvs,recst.currentvisibility);
                end;
              if not(is_ordinal(casetype))
{$ifndef cpu64bitaddr}
                 or is_64bitint(casetype)
{$endif cpu64bitaddr}
                 then
                Message(type_e_ordinal_expr_expected);
              consume(_OF);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              UnionSymtable:=trecordsymtable.create('',current_settings.packrecords,current_settings.alignment.recordalignmin,current_settings.alignment.maxCrecordalign);
              UnionDef:=crecorddef.create('',unionsymtable);
=======
              UnionSymtable:=trecordsymtable.create('',current_settings.packrecords);
              UnionDef:=trecorddef.create('',unionsymtable);
>>>>>>> graemeg/cpstrnew
=======
              UnionSymtable:=trecordsymtable.create('',current_settings.packrecords);
              UnionDef:=trecorddef.create('',unionsymtable);
>>>>>>> graemeg/cpstrnew
=======
              UnionSymtable:=trecordsymtable.create('',current_settings.packrecords);
              UnionDef:=trecorddef.create('',unionsymtable);
>>>>>>> graemeg/cpstrnew
=======
              UnionSymtable:=trecordsymtable.create('',current_settings.packrecords);
              UnionDef:=trecorddef.create('',unionsymtable);
>>>>>>> origin/cpstrnew
              uniondef.isunion:=true;

              startvarrecsize:=UnionSymtable.datasize;
              { align the bitpacking to the next byte }
              UnionSymtable.datasize:=startvarrecsize;
              startvarrecalign:=UnionSymtable.fieldalignment;
              startpadalign:=Unionsymtable.padalignment;
              symtablestack.push(UnionSymtable);
              repeat
                SetLength(variantdesc^^.branches,length(variantdesc^^.branches)+1);
                fillchar(variantdesc^^.branches[high(variantdesc^^.branches)],
                  sizeof(variantdesc^^.branches[high(variantdesc^^.branches)]),0);
                repeat
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  pt:=comp_expr([ef_accept_equal]);
                  if not(pt.nodetype=ordconstn) then
                    Message(parser_e_illegal_expression);
                  { iso pascal does not support ranges in variant record definitions }
                  if (([m_iso,m_extpas]*current_settings.modeswitches)=[]) and try_to_consume(_POINTPOINT) then
                    pt:=crangenode.create(pt,comp_expr([ef_accept_equal]))
                  else
                    begin
                      with variantdesc^^.branches[high(variantdesc^^.branches)] do
                        begin
                          SetLength(values,length(values)+1);
                          values[high(values)]:=tordconstnode(pt).value;
                        end;
                    end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  pt:=comp_expr(true,false);
                  if not(pt.nodetype=ordconstn) then
                    Message(parser_e_illegal_expression);
                  if try_to_consume(_POINTPOINT) then
                    pt:=crangenode.create(pt,comp_expr(true,false));
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
                  pt.free;
                  if token=_COMMA then
                    consume(_COMMA)
                  else
                    break;
                until false;
                if m_delphi in current_settings.modeswitches then
                  block_type:=bt_var_type
                else
                  block_type:=old_block_type;
                consume(_COLON);
                { read the vars }
                consume(_LKLAMMER);
                inc(variantrecordlevel);
                if token<>_RKLAMMER then
                  read_record_fields([vd_record],nil,@variantdesc^^.branches[high(variantdesc^^.branches)].nestedvariant,hadgendummy);
                dec(variantrecordlevel);
                consume(_RKLAMMER);

                { calculates maximal variant size }
                maxsize:=max(maxsize,unionsymtable.datasize);
                maxalignment:=max(maxalignment,unionsymtable.fieldalignment);
                maxpadalign:=max(maxpadalign,unionsymtable.padalignment);
                { the items of the next variant are overlayed }
                unionsymtable.datasize:=startvarrecsize;
                unionsymtable.fieldalignment:=startvarrecalign;
                unionsymtable.padalignment:=startpadalign;
                if (token<>_END) and (token<>_RKLAMMER) then
                  consume(_SEMICOLON)
                else
                  break;
              until (token=_END) or (token=_RKLAMMER);
              symtablestack.pop(UnionSymtable);
              { at last set the record size to that of the biggest variant }
              unionsymtable.datasize:=maxsize;
              unionsymtable.fieldalignment:=maxalignment;
              unionsymtable.addalignmentpadding;
{$if defined(powerpc) or defined(powerpc64)}
              { parent inherits the alignment padding if the variant is the first "field" of the parent record/variant }
              if (target_info.system in [system_powerpc_darwin, system_powerpc_macos, system_powerpc64_darwin]) and
<<<<<<< HEAD
<<<<<<< HEAD
                 is_first_type and
=======
                 is_first_field and
>>>>>>> graemeg/fixes_2_2
=======
                 is_first_field and
>>>>>>> origin/fixes_2_2
                 (recst.usefieldalignment=C_alignment) and
                 (maxpadalign>recst.padalignment) then
                recst.padalignment:=maxpadalign;
{$endif powerpc or powerpc64}
              { Align the offset where the union symtable is added }
<<<<<<< HEAD
<<<<<<< HEAD
              case recst.usefieldalignment of
                { allow the unionsymtable to be aligned however it wants }
                { (within the global min/max limits)                     }
                0, { default }
                C_alignment:
                  usedalign:=used_align(unionsymtable.recordalignment,current_settings.alignment.recordalignmin,current_settings.alignment.maxCrecordalign);
                { 1 byte alignment if we are bitpacked }
                bit_alignment:
                  usedalign:=1;
                mac68k_alignment:
                  usedalign:=2;
                { otherwise alignment at the packrecords alignment of the }
                { current record                                          }
                else
                  usedalign:=used_align(recst.fieldalignment,current_settings.alignment.recordalignmin,current_settings.alignment.recordalignmax);
              end;
=======
=======
>>>>>>> origin/fixes_2_2
              if (recst.usefieldalignment=C_alignment) then
                usedalign:=used_align(unionsymtable.recordalignment,current_settings.alignment.recordalignmin,current_settings.alignment.maxCrecordalign)
              else
                usedalign:=used_align(unionsymtable.recordalignment,current_settings.alignment.recordalignmin,current_settings.alignment.recordalignmax);

>>>>>>> graemeg/fixes_2_2
              offset:=align(recst.datasize,usedalign);
              recst.datasize:=offset+unionsymtable.datasize;

              if unionsymtable.recordalignment>recst.fieldalignment then
                recst.fieldalignment:=unionsymtable.recordalignment;

              trecordsymtable(recst).insertunionst(Unionsymtable,offset);
              uniondef.owner.deletedef(uniondef);
           end;
         { free the list }
         sc.free;
{$ifdef powerpc}
         is_first_type := false;
{$endif powerpc}
         block_type:=old_block_type;
      end;

end.
