{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Reads typed constants

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
unit ptconst;

{$i fpcdefs.inc}

interface

   uses symtype,symsym,aasmdata;

    procedure read_typed_const(list:tasmlist;sym:tstaticvarsym;in_structure:boolean);


implementation

    uses
       globtype,systems,globals,verbose,cutils,tokens,
       aasmbase,aasmtai,
       fmodule,
       scanner,pbase,pdecvar,
       node,ngtcon,
       symconst,symbase,symdef
       ;

    procedure read_typed_const(list:tasmlist;sym:tstaticvarsym;in_structure:boolean);
      var
        storefilepos : tfileposinfo;
        section      : ansistring;
        tcbuilder    : ttypedconstbuilder;
        reslist,
        datalist     : tasmlist;
        restree,
        previnit     : tnode;
      begin
        { mark the staticvarsym as typedconst }
        include(sym.varoptions,vo_is_typed_const);
        { The variable has a value assigned }
        sym.varstate:=vs_initialised;
        { the variable can't be placed in a register }
        sym.varregable:=vr_none;

        { generate data for typed const }
        storefilepos:=current_filepos;
        current_filepos:=sym.fileinfo;

        if not(target_info.system in systems_typed_constants_node_init) then
          begin
            maybe_new_object_file(list);
            tcbuilder:=tasmlisttypedconstbuilderclass(ctypedconstbuilder).create(sym);
            tasmlisttypedconstbuilder(tcbuilder).parse_into_asmlist;
          end
        else
          begin
            if assigned(current_structdef) then
              previnit:=current_structdef.tcinitcode
            else
              previnit:=tnode(current_module.tcinitcode);
            tcbuilder:=tnodetreetypedconstbuilderclass(ctypedconstbuilder).create(sym,previnit);
            restree:=tnodetreetypedconstbuilder(tcbuilder).parse_into_nodetree;
            if assigned(current_structdef) then
              current_structdef.tcinitcode:=restree
            else
              current_module.tcinitcode:=restree;
          end;

        { Parse hints }
        try_consume_hintdirective(sym.symoptions,sym.deprecatedmsg);

        consume(_SEMICOLON);

        { parse public/external/export/... }
        if not in_structure and
           (
            (
             (token = _ID) and
             ((idtoken in [_EXPORT,_EXTERNAL,_PUBLIC,_CVAR]) or (idtoken = _WEAKEXTERNAL)) and
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
          read_public_and_external(sym);


        { try to parse a section directive }
        if not in_structure and (target_info.system in systems_allow_section) and
           (symtablestack.top.symtabletype in [staticsymtable,globalsymtable]) and
           (idtoken=_SECTION) then
          begin
            try_consume_sectiondirective(section);
            if section<>'' then
              begin
                if (sym.varoptions *[vo_is_external,vo_is_weak_external])<>[] then
                  Message(parser_e_externals_no_section);
                if sym.typ<>staticvarsym then
                  Message(parser_e_section_no_locals);
                tstaticvarsym(sym).section:=section;
                include(sym.varoptions, vo_has_section);
              end;
          end;

        if not parse_generic then
          begin
            if vo_is_public in sym.varoptions then
              current_module.add_public_asmsym(sym.mangledname,AB_GLOBAL,AT_DATA);

            if not(target_info.system in systems_typed_constants_node_init) then
              begin
                { only now get the final asmlist, because inserting the symbol
                  information depends on potential section information set above }
                tasmlisttypedconstbuilder(tcbuilder).get_final_asmlists(reslist,datalist);
                 { add the parsed value }
                list.concatlist(reslist);
                { and pointed data, if any }
                current_asmdata.asmlists[al_const].concatlist(datalist);
                { the (empty) lists themselves are freed by tcbuilder }
              end
            else
              begin
                { nothing to do }
              end;
          end;

        tcbuilder.free;
        current_filepos:=storefilepos;
      end;

end.
