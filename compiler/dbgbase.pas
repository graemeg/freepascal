{
    Copyright (c) 2003-2006 by Peter Vreman and Florian Klaempfl

    This units contains the base class for debug info generation

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
unit dbgbase;

{$i fpcdefs.inc}

interface

    uses
      cclasses,
      systems,
      parabase,
      symconst,symbase,symdef,symtype,symsym,symtable,
      fmodule,
      aasmtai,aasmdata;

    type
      TDebugInfo=class
      protected
        { definitions }
<<<<<<< HEAD
<<<<<<< HEAD
        { collect all defs in one list so we can reset them easily }
        defnumberlist      : TFPObjectList;
        deftowritelist     : TFPObjectList;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        procedure appenddef(list:TAsmList;def:tdef);
        procedure beforeappenddef(list:TAsmList;def:tdef);virtual;
        procedure afterappenddef(list:TAsmList;def:tdef);virtual;
        procedure appenddef_ord(list:TAsmList;def:torddef);virtual;
        procedure appenddef_float(list:TAsmList;def:tfloatdef);virtual;
        procedure appenddef_file(list:TAsmList;def:tfiledef);virtual;
        procedure appenddef_enum(list:TAsmList;def:tenumdef);virtual;
        procedure appenddef_array(list:TAsmList;def:tarraydef);virtual;
        procedure appenddef_record(list:TAsmList;def:trecorddef);virtual;
        procedure appenddef_object(list:TAsmList;def:tobjectdef);virtual;
<<<<<<< HEAD
<<<<<<< HEAD
        procedure appenddef_classref(list:TAsmList;def: tclassrefdef);virtual;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        procedure appenddef_pointer(list:TAsmList;def:tpointerdef);virtual;
        procedure appenddef_string(list:TAsmList;def:tstringdef);virtual;
        procedure appenddef_procvar(list:TAsmList;def:tprocvardef);virtual;
        procedure appenddef_variant(list:TAsmList;def:tvariantdef);virtual;
        procedure appenddef_set(list:TAsmList;def:tsetdef);virtual;
        procedure appenddef_formal(list:TAsmList;def:tformaldef);virtual;
        procedure appenddef_undefined(list:TAsmList;def: tundefineddef);virtual;
        procedure appendprocdef(list:TAsmList;def:tprocdef);virtual;
<<<<<<< HEAD
<<<<<<< HEAD
        procedure write_remaining_defs_to_write(list:TAsmList);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        { symbols }
        procedure appendsym(list:TAsmList;sym:tsym);
        procedure beforeappendsym(list:TAsmList;sym:tsym);virtual;
        procedure afterappendsym(list:TAsmList;sym:tsym);virtual;
        procedure appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);virtual;
        procedure appendsym_paravar(list:TAsmList;sym:tparavarsym);virtual;
        procedure appendsym_localvar(list:TAsmList;sym:tlocalvarsym);virtual;
        procedure appendsym_fieldvar(list:TAsmList;sym:tfieldvarsym);virtual;
        procedure appendsym_unit(list:TAsmList;sym:tunitsym);virtual;
        procedure appendsym_const(list:TAsmList;sym:tconstsym);virtual;
        procedure appendsym_type(list:TAsmList;sym:ttypesym);virtual;
        procedure appendsym_label(list:TAsmList;sym:tlabelsym);virtual;
        procedure appendsym_absolute(list:TAsmList;sym:tabsolutevarsym);virtual;
        procedure appendsym_property(list:TAsmList;sym:tpropertysym);virtual;
        { symtable }
<<<<<<< HEAD
<<<<<<< HEAD
        procedure write_symtable_parasyms(list:TAsmList;paras: tparalist);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        procedure write_symtable_syms(list:TAsmList;st:TSymtable);
        procedure write_symtable_defs(list:TAsmList;st:TSymtable);
        procedure write_symtable_procdefs(list:TAsmList;st:TSymtable);
        procedure reset_unit_type_info;
        procedure write_used_unit_type_info(list:TAsmList;hp:tmodule);
      public
        constructor Create;virtual;
        procedure inserttypeinfo;virtual;
        procedure insertmoduleinfo;virtual;
        procedure insertlineinfo(list:TAsmList);virtual;
        procedure referencesections(list:TAsmList);virtual;
      end;
      TDebugInfoClass=class of TDebugInfo;

    var
      CDebugInfo : array[tdbg] of TDebugInfoClass;
      current_debuginfo : tdebuginfo;

<<<<<<< HEAD
<<<<<<< HEAD
    procedure InitDebugInfo(hp:tmodule; restore_current_debuginfo : boolean);
    procedure DoneDebugInfo(hp:tmodule;var current_debuginfo_reset : boolean);
=======
    procedure InitDebugInfo(hp:tmodule);
    procedure DoneDebugInfo(hp:tmodule);
>>>>>>> graemeg/fixes_2_2
=======
    procedure InitDebugInfo(hp:tmodule);
    procedure DoneDebugInfo(hp:tmodule);
>>>>>>> origin/fixes_2_2
    procedure RegisterDebugInfo(const r:tdbginfo;c:TDebugInfoClass);


implementation

    uses
      cutils,
      verbose;


    constructor TDebugInfo.Create;
      begin
      end;


    procedure TDebugInfo.insertmoduleinfo;
      begin
      end;


    procedure TDebugInfo.inserttypeinfo;
      begin
      end;


    procedure TDebugInfo.insertlineinfo(list:TAsmList);
      begin
      end;


    procedure TDebugInfo.referencesections(list:TAsmList);
      begin
      end;


{**************************************
          Definition
**************************************}

    procedure TDebugInfo.appendprocdef(list:TAsmList;def:tprocdef);
<<<<<<< HEAD
=======
      begin
      end;


    procedure TDebugInfo.beforeappenddef(list:TAsmList;def:tdef);
      begin
      end;


    procedure TDebugInfo.afterappenddef(list:TAsmList;def:tdef);
      begin
      end;


    procedure TDebugInfo.appenddef_ord(list:TAsmList;def:torddef);
      begin
      end;


    procedure TDebugInfo.appenddef_float(list:TAsmList;def:tfloatdef);
      begin
      end;


    procedure TDebugInfo.appenddef_formal(list:TAsmList;def: tformaldef);
      begin
      end;


    procedure TDebugInfo.appenddef_undefined(list:TAsmList;def: tundefineddef);
>>>>>>> origin/fixes_2_2
      begin
      end;


<<<<<<< HEAD
    procedure TDebugInfo.beforeappenddef(list:TAsmList;def:tdef);
=======
    procedure TDebugInfo.appenddef_set(list:TAsmList;def: tsetdef);
>>>>>>> origin/fixes_2_2
      begin
      end;


<<<<<<< HEAD
    procedure TDebugInfo.afterappenddef(list:TAsmList;def:tdef);
=======
    procedure TDebugInfo.appenddef_object(list:TAsmList;def: tobjectdef);
>>>>>>> origin/fixes_2_2
      begin
      end;


<<<<<<< HEAD
    procedure TDebugInfo.appenddef_ord(list:TAsmList;def:torddef);
=======
    procedure TDebugInfo.appenddef_variant(list:TAsmList;def: tvariantdef);
>>>>>>> origin/fixes_2_2
      begin
      end;


<<<<<<< HEAD
    procedure TDebugInfo.appenddef_float(list:TAsmList;def:tfloatdef);
=======
    procedure TDebugInfo.appenddef_enum(list:TAsmList;def:tenumdef);
>>>>>>> origin/fixes_2_2
      begin
      end;


<<<<<<< HEAD
    procedure TDebugInfo.appenddef_formal(list:TAsmList;def: tformaldef);
      begin
      end;


    procedure TDebugInfo.appenddef_undefined(list:TAsmList;def: tundefineddef);
<<<<<<< HEAD
      begin
      end;


    procedure TDebugInfo.appenddef_set(list:TAsmList;def: tsetdef);
      begin
      end;


    procedure TDebugInfo.appenddef_object(list:TAsmList;def: tobjectdef);
      begin
      end;


    procedure TDebugInfo.appenddef_classref(list:TAsmList;def: tclassrefdef);
      begin
        appenddef_pointer(list,tpointerdef(pvmttype));
      end;


    procedure TDebugInfo.appenddef_variant(list:TAsmList;def: tvariantdef);
=======
>>>>>>> graemeg/fixes_2_2
      begin
      end;


<<<<<<< HEAD
    procedure TDebugInfo.appenddef_enum(list:TAsmList;def:tenumdef);
=======
    procedure TDebugInfo.appenddef_set(list:TAsmList;def: tsetdef);
>>>>>>> graemeg/fixes_2_2
      begin
      end;


<<<<<<< HEAD
    procedure TDebugInfo.appenddef_file(list:TAsmList;def: tfiledef);
=======
    procedure TDebugInfo.appenddef_object(list:TAsmList;def: tobjectdef);
>>>>>>> graemeg/fixes_2_2
=======
    procedure TDebugInfo.appenddef_file(list:TAsmList;def: tfiledef);
>>>>>>> origin/fixes_2_2
      begin
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure TDebugInfo.appenddef_array(list:TAsmList;def:tarraydef);
=======
    procedure TDebugInfo.appenddef_variant(list:TAsmList;def: tvariantdef);
>>>>>>> graemeg/fixes_2_2
=======
    procedure TDebugInfo.appenddef_array(list:TAsmList;def:tarraydef);
>>>>>>> origin/fixes_2_2
      begin
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure TDebugInfo.appenddef_record(list:TAsmList;def:trecorddef);
=======
    procedure TDebugInfo.appenddef_enum(list:TAsmList;def:tenumdef);
>>>>>>> graemeg/fixes_2_2
      begin
      end;


<<<<<<< HEAD
=======
    procedure TDebugInfo.appenddef_file(list:TAsmList;def: tfiledef);
      begin
      end;


    procedure TDebugInfo.appenddef_array(list:TAsmList;def:tarraydef);
=======
    procedure TDebugInfo.appenddef_record(list:TAsmList;def:trecorddef);
>>>>>>> origin/fixes_2_2
      begin
      end;


<<<<<<< HEAD
    procedure TDebugInfo.appenddef_record(list:TAsmList;def:trecorddef);
      begin
      end;


>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure TDebugInfo.appenddef_pointer(list:TAsmList;def:tpointerdef);
      begin
      end;


    procedure TDebugInfo.appenddef_string(list:TAsmList;def:tstringdef);
      begin
      end;


    procedure TDebugInfo.appenddef_procvar(list:TAsmList;def:tprocvardef);
      begin
      end;


    procedure TDebugInfo.appenddef(list:TAsmList;def:tdef);
      begin
        if (def.dbg_state in [dbg_state_writing,dbg_state_written]) then
          exit;
        { never write generic template defs }
        if df_generic in def.defoptions then
          begin
            def.dbg_state:=dbg_state_written;
            exit;
          end;
        { to avoid infinite loops }
        def.dbg_state := dbg_state_writing;
        beforeappenddef(list,def);
        { queued defs have to be written later }
        if (def.dbg_state=dbg_state_queued) then
          exit;
        case def.typ of
          stringdef :
            appenddef_string(list,tstringdef(def));
          enumdef :
            appenddef_enum(list,tenumdef(def));
          orddef :
            appenddef_ord(list,torddef(def));
          pointerdef :
            appenddef_pointer(list,tpointerdef(def));
          floatdef :
            appenddef_float(list,tfloatdef(def));
          filedef :
            appenddef_file(list,tfiledef(def));
          recorddef :
            appenddef_record(list,trecorddef(def));
          variantdef :
            appenddef_variant(list,tvariantdef(def));
          classrefdef :
<<<<<<< HEAD
<<<<<<< HEAD
            appenddef_classref(list,tclassrefdef(def));
=======
            appenddef_pointer(list,tpointerdef(pvmttype));
>>>>>>> graemeg/fixes_2_2
=======
            appenddef_pointer(list,tpointerdef(pvmttype));
>>>>>>> origin/fixes_2_2
          setdef :
            appenddef_set(list,tsetdef(def));
          formaldef :
            appenddef_formal(list,tformaldef(def));
          arraydef :
            appenddef_array(list,tarraydef(def));
          procvardef :
            appenddef_procvar(list,tprocvardef(def));
          objectdef :
            appenddef_object(list,tobjectdef(def));
          undefineddef :
            appenddef_undefined(list,tundefineddef(def));
          procdef :
            begin
              { procdefs are already written in a separate step. procdef
                support in appenddef is only needed for beforeappenddef to
                write all local type defs }
            end;
        else
          internalerror(200601281);
        end;
        afterappenddef(list,def);
        def.dbg_state := dbg_state_written;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure TDebugInfo.write_remaining_defs_to_write(list:TAsmList);
      var
        n       : integer;
        looplist,
        templist: TFPObjectList;
        def     : tdef;
      begin
        templist := TFPObjectList.Create(False);
        looplist := deftowritelist;
        while looplist.count > 0 do
          begin
            deftowritelist := templist;
            for n := 0 to looplist.count - 1 do
              begin
                def := tdef(looplist[n]);
                case def.dbg_state of
                  dbg_state_written:
                    continue;
                  dbg_state_writing:
                    internalerror(200610052);
                  dbg_state_unused:
                    internalerror(200610053);
                  dbg_state_used:
                    appenddef(list,def);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  dbg_state_queued:
                    begin
                      { can happen in case an objectdef was used from another
                        unit that was compiled without debug info, and we are
                        using Stabs (which means that parent types have to be
                        written before child types). In this case, the child
                        objectdef will be queued and never written, because its
                        definition is not inside the current unit and hence will
                        not be encountered }
                      if def.typ<>objectdef then
                        internalerror(2012072401);
                      if not assigned(tobjectdef(def).childof) or
                         (tobjectdef(def).childof.dbg_state=dbg_state_written) then
                        appenddef(list,def)
                      else if tobjectdef(def).childof.dbg_state=dbg_state_queued then
                        deftowritelist.add(def)
                      else if tobjectdef(def).childof.dbg_state=dbg_state_used then
                        { comes somewhere after the current def in the looplist
                          and will be written at that point, so we will have to
                          wait until the next iteration }
                        deftowritelist.add(def)
                      else
                        internalerror(2012072402);
                    end;
                  else
                    internalerror(200610054);
=======
                else
                  internalerror(200610054);
>>>>>>> graemeg/cpstrnew
=======
                else
                  internalerror(200610054);
>>>>>>> graemeg/cpstrnew
=======
                else
                  internalerror(200610054);
>>>>>>> graemeg/cpstrnew
=======
                else
                  internalerror(200610054);
>>>>>>> origin/cpstrnew
                end;
              end;
            looplist.clear;
            templist := looplist;
            looplist := deftowritelist;
          end;
        templist.free;
      end;


{**************************************
          Symbols
**************************************}

    procedure TDebugInfo.beforeappendsym(list:TAsmList;sym:tsym);
      begin
      end;


    procedure TDebugInfo.afterappendsym(list:TAsmList;sym:tsym);
      begin
      end;


    procedure TDebugInfo.appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_paravar(list:TAsmList;sym: tparavarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_localvar(list:TAsmList;sym: tlocalvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_fieldvar(list:TAsmList;sym: tfieldvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_const(list:TAsmList;sym:tconstsym);
      begin
      end;


    procedure TDebugInfo.appendsym_label(list:TAsmList;sym: tlabelsym);
      begin
      end;


    procedure TDebugInfo.appendsym_property(list:TAsmList;sym: tpropertysym);
      begin
      end;


    procedure TDebugInfo.appendsym_type(list:TAsmList;sym: ttypesym);
      begin
      end;


    procedure TDebugInfo.appendsym_unit(list:TAsmList;sym: tunitsym);
      begin
      end;


    procedure TDebugInfo.appendsym_absolute(list:TAsmList;sym:tabsolutevarsym);
      begin
      end;


    procedure TDebugInfo.appendsym(list:TAsmList;sym:tsym);
      begin
        if sym.isdbgwritten then
          exit;
        beforeappendsym(list,sym);
        case sym.typ of
          staticvarsym :
            if not assigned(tstaticvarsym(sym).fieldvarsym) or
               not(df_generic in tdef(tstaticvarsym(sym).fieldvarsym.owner.defowner).defoptions) then
              appendsym_staticvar(list,tstaticvarsym(sym));
          unitsym:
            appendsym_unit(list,tunitsym(sym));
          labelsym :
            appendsym_label(list,tlabelsym(sym));
          localvarsym :
            appendsym_localvar(list,tlocalvarsym(sym));
          paravarsym :
            appendsym_paravar(list,tparavarsym(sym));
          constsym :
            appendsym_const(list,tconstsym(sym));
          typesym :
            appendsym_type(list,ttypesym(sym));
          enumsym :
            { ignore enum syms, they are written by the owner }
            ;
          syssym :
            { ignore sys syms, they are only of internal use }
            ;
          procsym :
            { ignore proc syms, they are written by procdefs }
            ;
          absolutevarsym :
            appendsym_absolute(list,tabsolutevarsym(sym));
          propertysym :
            appendsym_property(list,tpropertysym(sym));
          namespacesym :
            { ignore namespace syms, they are only of internal use }
            ;
          else
            internalerror(200601242);
        end;
        afterappendsym(list,sym);
        sym.isdbgwritten:=true;
      end;


{**************************************
          Symtables
**************************************}

    procedure TDebugInfo.write_symtable_defs(list:TAsmList;st:TSymtable);
      var
        def : tdef;
        i   : longint;
        nonewadded : boolean;
      begin
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - Begin Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - Begin unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
        repeat
          nonewadded:=true;
          for i:=0 to st.DefList.Count-1 do
            begin
              def:=tdef(st.DefList[i]);
              if (def.dbg_state in [dbg_state_used,dbg_state_queued]) then
                begin
                  appenddef(list,def);
                  nonewadded:=false;
                end;
            end;
        until nonewadded;
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - End Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - End unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
      end;


    procedure TDebugInfo.write_symtable_parasyms(list:TAsmList;paras: tparalist);
      var
        i   : longint;
        sym : tsym;
      begin
        for i:=0 to paras.Count-1 do
          begin
            sym:=tsym(paras[i]);
            if (sym.visibility<>vis_hidden) then
              begin
                appendsym(list,sym);
                { if we ever write this procdef again for some reason (this
                  can happen with DWARF), then we want to write all the
                  parasyms again as well. }
                sym.isdbgwritten:=false;
              end;
=======
{**************************************
          Symbols
**************************************}

    procedure TDebugInfo.beforeappendsym(list:TAsmList;sym:tsym);
      begin
      end;


    procedure TDebugInfo.afterappendsym(list:TAsmList;sym:tsym);
      begin
      end;


    procedure TDebugInfo.appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_paravar(list:TAsmList;sym: tparavarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_localvar(list:TAsmList;sym: tlocalvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_fieldvar(list:TAsmList;sym: tfieldvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_const(list:TAsmList;sym:tconstsym);
      begin
      end;


    procedure TDebugInfo.appendsym_label(list:TAsmList;sym: tlabelsym);
      begin
      end;


    procedure TDebugInfo.appendsym_property(list:TAsmList;sym: tpropertysym);
      begin
      end;


    procedure TDebugInfo.appendsym_type(list:TAsmList;sym: ttypesym);
      begin
      end;


    procedure TDebugInfo.appendsym_unit(list:TAsmList;sym: tunitsym);
      begin
      end;


    procedure TDebugInfo.appendsym_absolute(list:TAsmList;sym:tabsolutevarsym);
      begin
      end;


    procedure TDebugInfo.appendsym(list:TAsmList;sym:tsym);
      begin
        if sym.isdbgwritten then
          exit;
        beforeappendsym(list,sym);
        case sym.typ of
          staticvarsym :
            appendsym_staticvar(list,tstaticvarsym(sym));
          unitsym:
            appendsym_unit(list,tunitsym(sym));
          labelsym :
            appendsym_label(list,tlabelsym(sym));
          localvarsym :
            appendsym_localvar(list,tlocalvarsym(sym));
          paravarsym :
            appendsym_paravar(list,tparavarsym(sym));
          constsym :
            appendsym_const(list,tconstsym(sym));
          typesym :
            appendsym_type(list,ttypesym(sym));
          enumsym :
            { ignore enum syms, they are written by the owner }
            ;
          syssym :
            { ignore sys syms, they are only of internal use }
            ;
          procsym :
            { ignore proc syms, they are written by procdefs }
            ;
          absolutevarsym :
            appendsym_absolute(list,tabsolutevarsym(sym));
          propertysym :
            appendsym_property(list,tpropertysym(sym));
          else
            internalerror(200601242);
        end;
        afterappendsym(list,sym);
        sym.isdbgwritten:=true;
      end;


{**************************************
          Symtables
**************************************}

    procedure TDebugInfo.write_symtable_defs(list:TAsmList;st:TSymtable);
      var
        def : tdef;
        i   : longint;
      begin
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - Begin Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - Begin unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
        for i:=0 to st.DefList.Count-1 do
          begin
            def:=tdef(st.DefList[i]);
            if (def.dbg_state in [dbg_state_used,dbg_state_queued]) then
              appenddef(list,def);
          end;
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - End Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - End unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
      end;


    procedure TDebugInfo.write_symtable_syms(list:TAsmList;st:TSymtable);
      var
        i   : longint;
        sym : tsym;
      begin
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - Begin Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - Begin unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if not(sp_hidden in sym.symoptions) and
               (not sym.isdbgwritten) then
              appendsym(list,sym);
          end;
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - End Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - End unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
      end;


    procedure TDebugInfo.write_symtable_procdefs(list:TAsmList;st:TSymtable);
      var
        i   : longint;
        def : tdef;
      begin
=======
{**************************************
          Symbols
**************************************}

    procedure TDebugInfo.beforeappendsym(list:TAsmList;sym:tsym);
      begin
      end;


    procedure TDebugInfo.afterappendsym(list:TAsmList;sym:tsym);
      begin
      end;


    procedure TDebugInfo.appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_paravar(list:TAsmList;sym: tparavarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_localvar(list:TAsmList;sym: tlocalvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_fieldvar(list:TAsmList;sym: tfieldvarsym);
      begin
      end;


    procedure TDebugInfo.appendsym_const(list:TAsmList;sym:tconstsym);
      begin
      end;


    procedure TDebugInfo.appendsym_label(list:TAsmList;sym: tlabelsym);
      begin
      end;


    procedure TDebugInfo.appendsym_property(list:TAsmList;sym: tpropertysym);
      begin
      end;


    procedure TDebugInfo.appendsym_type(list:TAsmList;sym: ttypesym);
      begin
      end;


    procedure TDebugInfo.appendsym_unit(list:TAsmList;sym: tunitsym);
      begin
      end;


    procedure TDebugInfo.appendsym_absolute(list:TAsmList;sym:tabsolutevarsym);
      begin
      end;


    procedure TDebugInfo.appendsym(list:TAsmList;sym:tsym);
      begin
        if sym.isdbgwritten then
          exit;
        beforeappendsym(list,sym);
        case sym.typ of
          staticvarsym :
            appendsym_staticvar(list,tstaticvarsym(sym));
          unitsym:
            appendsym_unit(list,tunitsym(sym));
          labelsym :
            appendsym_label(list,tlabelsym(sym));
          localvarsym :
            appendsym_localvar(list,tlocalvarsym(sym));
          paravarsym :
            appendsym_paravar(list,tparavarsym(sym));
          constsym :
            appendsym_const(list,tconstsym(sym));
          typesym :
            appendsym_type(list,ttypesym(sym));
          enumsym :
            { ignore enum syms, they are written by the owner }
            ;
          syssym :
            { ignore sys syms, they are only of internal use }
            ;
          procsym :
            { ignore proc syms, they are written by procdefs }
            ;
          absolutevarsym :
            appendsym_absolute(list,tabsolutevarsym(sym));
          propertysym :
            appendsym_property(list,tpropertysym(sym));
          else
            internalerror(200601242);
        end;
        afterappendsym(list,sym);
        sym.isdbgwritten:=true;
      end;


{**************************************
          Symtables
**************************************}

    procedure TDebugInfo.write_symtable_defs(list:TAsmList;st:TSymtable);
      var
        def : tdef;
        i   : longint;
      begin
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - Begin Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - Begin unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
        for i:=0 to st.DefList.Count-1 do
          begin
            def:=tdef(st.DefList[i]);
            if (def.dbg_state in [dbg_state_used,dbg_state_queued]) then
              appenddef(list,def);
          end;
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - End Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Defs - End unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
      end;


    procedure TDebugInfo.write_symtable_syms(list:TAsmList;st:TSymtable);
      var
        i   : longint;
        sym : tsym;
      begin
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - Begin Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - Begin unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if not(sp_hidden in sym.symoptions) and
               (not sym.isdbgwritten) then
              appendsym(list,sym);
          end;
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - End Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - End unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
      end;


    procedure TDebugInfo.write_symtable_procdefs(list:TAsmList;st:TSymtable);
      var
        i   : longint;
        def : tdef;
      begin
>>>>>>> origin/fixes_2_2
        for i:=0 to st.DefList.Count-1 do
          begin
            def:=tdef(st.DefList[i]);
            case def.typ of
              procdef :
                begin
                  appendprocdef(list,tprocdef(def));
                  if assigned(tprocdef(def).localst) then
                    write_symtable_procdefs(list,tprocdef(def).localst);
                end;
              objectdef :
                begin
                  write_symtable_procdefs(list,tobjectdef(def).symtable);
                end;
            end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
    procedure TDebugInfo.write_symtable_syms(list:TAsmList;st:TSymtable);
      var
        i   : longint;
        sym : tsym;
      begin
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - Begin Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - Begin unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
        for i:=0 to st.SymList.Count-1 do
          begin
            sym:=tsym(st.SymList[i]);
            if (sym.visibility<>vis_hidden) and
               (not sym.isdbgwritten) and
               { avoid all generic symbols }
               not (sp_generic_dummy in sym.symoptions) and
               not ((sym.typ=typesym) and assigned(ttypesym(sym).typedef) and
                    (df_generic in ttypesym(sym).typedef.defoptions)) then
              appendsym(list,sym);
          end;
        case st.symtabletype of
          staticsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - End Staticsymtable')));
          globalsymtable :
            list.concat(tai_comment.Create(strpnew('Syms - End unit '+st.name^+' has index '+tostr(st.moduleid))));
        end;
      end;


    procedure TDebugInfo.write_symtable_procdefs(list:TAsmList;st:TSymtable);
      var
        i   : longint;
        def : tdef;
      begin
        for i:=0 to st.DefList.Count-1 do
          begin
            def:=tdef(st.DefList[i]);
            case def.typ of
              procdef :
                begin
                  appendprocdef(list,tprocdef(def));
                  if assigned(tprocdef(def).localst) then
                    write_symtable_procdefs(list,tprocdef(def).localst);
                end;
              objectdef,recorddef :
                begin
                  write_symtable_procdefs(list,tabstractrecorddef(def).symtable);
                end;
            end;
          end;
      end;


    procedure TDebugInfo.reset_unit_type_info;
      var
        hp : tmodule;
      begin
        hp:=tmodule(loaded_units.first);
        while assigned(hp) do
          begin
=======
    procedure TDebugInfo.reset_unit_type_info;
      var
        hp : tmodule;
      begin
        hp:=tmodule(loaded_units.first);
        while assigned(hp) do
          begin
>>>>>>> graemeg/fixes_2_2
=======
    procedure TDebugInfo.reset_unit_type_info;
      var
        hp : tmodule;
      begin
        hp:=tmodule(loaded_units.first);
        while assigned(hp) do
          begin
>>>>>>> origin/fixes_2_2
            hp.is_dbginfo_written:=false;
            hp:=tmodule(hp.next);
          end;
      end;


    procedure TDebugInfo.write_used_unit_type_info(list:TAsmList;hp:tmodule);
      var
        pu : tused_unit;
      begin
        pu:=tused_unit(hp.used_units.first);
        while assigned(pu) do
          begin
            if not pu.u.is_dbginfo_written then
              begin
                { prevent infinte loop for circular dependencies }
                pu.u.is_dbginfo_written:=true;
                { write type info from used units, use a depth first
                  strategy to reduce the recursion in writing all
                  dependent stabs }
                write_used_unit_type_info(list,pu.u);
                if assigned(pu.u.globalsymtable) then
                  write_symtable_defs(list,pu.u.globalsymtable);
              end;
            pu:=tused_unit(pu.next);
          end;
      end;


{****************************************************************************
                           Init / Done
****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
    procedure InitDebugInfo(hp:tmodule; restore_current_debuginfo : boolean);
=======
    procedure InitDebugInfo(hp:tmodule);
>>>>>>> graemeg/fixes_2_2
=======
    procedure InitDebugInfo(hp:tmodule);
>>>>>>> origin/fixes_2_2
      begin
        if not assigned(CDebugInfo[target_dbg.id]) then
          begin
            Comment(V_Fatal,'cg_f_debuginfo_output_not_supported');
            exit;
          end;
        hp.DebugInfo:=CDebugInfo[target_dbg.id].Create;
<<<<<<< HEAD
<<<<<<< HEAD
        if restore_current_debuginfo then
          begin
            if current_debuginfo=nil then
              current_debuginfo:=tdebuginfo(hp.DebugInfo)
            else
              internalerror(2012032101);
          end;
      end;


    procedure DoneDebugInfo(hp:tmodule;var current_debuginfo_reset : boolean);
      begin
        current_debuginfo_reset:=false;
        if assigned(hp.DebugInfo) then
          begin
            if hp.DebugInfo=current_debuginfo then
              begin
                current_debuginfo:=nil;
                current_debuginfo_reset:=true;
              end;
=======
=======
>>>>>>> origin/fixes_2_2
      end;


    procedure DoneDebugInfo(hp:tmodule);
      begin
        if assigned(hp.DebugInfo) then
          begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            hp.DebugInfo.Free;
            hp.DebugInfo:=nil;
          end;
      end;


    procedure RegisterDebugInfo(const r:tdbginfo;c:TDebugInfoClass);
      var
        t : tdbg;
      begin
        t:=r.id;
        if assigned(dbginfos[t]) then
          writeln('Warning: DebugInfo is already registered!')
        else
          Getmem(dbginfos[t],sizeof(tdbginfo));
        dbginfos[t]^:=r;
        CDebugInfo[t]:=c;
      end;


    const
      dbg_none_info : tdbginfo =
         (
           id     : dbg_none;
           idtxt  : 'NONE';
         );

initialization
  RegisterDebugInfo(dbg_none_info,TDebugInfo);
end.
