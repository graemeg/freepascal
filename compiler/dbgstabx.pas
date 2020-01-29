{
    Copyright (c) 2012 by Jonas Maebe

    This units contains support for STABX debug info generation

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
unit dbgstabx;

{$i fpcdefs.inc}

interface

  uses
    cclasses,globtype,
    dbgbase,dbgstabs,cgbase,
    symtype,symdef,symsym,symtable,symbase,
    aasmtai,aasmdata;

  type
    TDebugInfoStabx = class(TDebugInfoStabs)
     protected
      function staticvarsym_mangled_name(sym: tstaticvarsym): string; override;
      procedure maybe_add_vmt_sym(list: TAsmList; def: tobjectdef); override;
      procedure write_def_stabstr(list:TAsmList;def:tdef;const ss:ansistring);override;
      function  base_stabs_str(const typ, other, desc, value: ansistring): ansistring;overload;override;
      function  gen_procdef_startsym_stabs(def: tprocdef): TAsmList; override;
      function  gen_procdef_endsym_stabs(def: tprocdef): TAsmList; override;
      procedure appendsym_label(list: TAsmList; sym: tlabelsym); override;
      procedure appendsym_staticvar(list: TAsmList; sym: tstaticvarsym); override;
     public
      procedure insertlineinfo(list:TAsmList);override;
      procedure insertmoduleinfo; override;
      procedure referencesections(list: TAsmList); override;

      constructor create;override;
    end;

implementation

  uses
    globals,cutils,cfileutl,verbose,
    systems,finput,fmodule,
    aasmbase,
    symconst;

  const
    STABX_N_GSYM = $80;
    STABX_N_LSYM = $81;
    STABX_N_PSYM = $82;
    STABX_N_RSYM = $83;
    STABX_N_RPSYM = $84;
    STABX_N_STSYM = $85;
    STABX_N_LCSYM = 255;
    STABX_N_Function = $8e;
    STABX_N_TextLine = 255;
    STABX_N_DataLine = 255;
    STABX_N_BssLine = 255;
    STABX_N_DECL = $8c;
    STABX_N_tsym = $86;
    STABX_N_SourceFile = 255;
    STABX_N_OSO = 255;
    STABX_N_IncludeFile = 255;
    STABX_N_BINCL = 255;
    STABX_N_EINCL = 255;
    STABX_N_LBRAC = 255;
    STABX_N_EXCL = 255;
    STABX_N_RBRAC = 255;


{ TDebugInfoStabx }

  function TDebugInfoStabx.base_stabs_str(const typ, other, desc, value: ansistring): ansistring;
    begin
      { no other/desc }
      result:=value+','+typ+',0';
    end;


  function TDebugInfoStabx.staticvarsym_mangled_name(sym: tstaticvarsym): string;
    begin
      { create reference to the local symbol at the same address as the global
        symbol (with same name as unmangled symbol, so GDB can find it) }
      Result:=ReplaceForbiddenAsmSymbolChars(sym.name);
    end;


  procedure TDebugInfoStabx.maybe_add_vmt_sym(list: TAsmList; def: tobjectdef);
    begin
(*
      if assigned(def.owner) and
         def.owner.iscurrentunit then
        begin
          if (oo_has_vmt in def.objectoptions) and
             assigned(def.owner.name) then
            list.concat(Tai_stab.create_ansistr(stabsdir,ansistring('"vmt_')+GetSymTableName(def.owner)+tobjectdef(def).objname^+':S'+
                   def_stab_number(vmttype)+'",'+
                   base_stabs_str(globalvarsym_inited_stab,'0','0',ReplaceForbiddenAsmSymbolChars(tobjectdef(def).vmt_mangledname)+'.')));
        end;
*)
      { do nothing, because creating debug information for a global symbol
        defined in another unit is not possible for stabx given the FPC
        constraints (namely that the name of the global symbol does not match
        the name of the variable). If it's in the same unit, we have to add an
        extra symbol for the vmt with the same variable name as what we have
        here (ansistring('"vmt_')+GetSymTableName(def.owner)+tobjectdef(def).objname^).
        We'd have to do that when that symbol is created, in generic code,
        which is not very clean, and moreover these symbols are not really
        used for anything currently, afaik }
    end;


  procedure TDebugInfoStabx.write_def_stabstr(list:TAsmList;def:tdef;const ss:ansistring);
    var
      stabchar,
      symname,
      declstabnr,
      st    : ansistring;
    begin
      { type prefix }
      if use_tag_prefix(def) then
        stabchar := tagtypeprefix
      else
        stabchar := 't';
      { in case of writing the class record structure, we always have to
        use the class name (so it refers both to the struct and the
        pointer to the struct), otherwise gdb crashes (see tests/webtbs/tw9766.pp) }

      if is_class(def) and
         tobjectdef(def).writing_class_record_dbginfo then
        begin
          declstabnr:=def_stab_classnumber(tobjectdef(def));
          symname:='${sym_name}'
        end
      else
        begin
          { Type names for types defined in the current unit are already written in
            the typesym }
          if (def.owner.symtabletype=globalsymtable) and
             not(def.owner.iscurrentunit) then
            symname:='${sym_name}'
          else
            symname:='';
          declstabnr:=def_stab_number(def)
        end;
      if (symname='') or
         not(use_tag_prefix(def)) then
        begin
          st:=def_stabstr_evaluate(def,':$1$2=',[stabchar,declstabnr]);
          st:='"'+def_stabstr_evaluate(def,symname,[])+st+ss;
          { line info is set to 0 for all defs, because the def can be in another
            unit and then the linenumber is invalid in the current sourcefile }
          st:=st+'",'+base_stabs_str(def_stab,'0','0','0');
          { add to list }
          list.concat(Tai_stab.create_ansistr(stabsdir,st));
        end
      else
        begin
          { first tag, then type decl }
          inc(global_stab_number);
          st:=def_stabstr_evaluate(def,':$1$2=',[stabchar,tostr(global_stab_number)]);
          st:='"'+st+ss;
          st:=st+'",'+base_stabs_str(def_stab,'0','0','0');
          list.concat(Tai_stab.create_ansistr(stabsdir,st));
          st:='"'+def_stabstr_evaluate(def,symname+':t$1=$2',[declstabnr,tostr(global_stab_number)]);
          st:=st+'",'+base_stabs_str(def_stab,'0','0','0');
          list.concat(Tai_stab.create_ansistr(stabsdir,st));
        end;
    end;


  function TDebugInfoStabx.gen_procdef_startsym_stabs(def: tprocdef): TAsmList;
    var
      mangledname: ansistring;
      hp, hpp, inclstart: tai;
    begin
      result:=inherited;
      { can happen for procdefs defined in other units, this code is only for
        the place where it is defined }
      if not assigned(def.procstarttai) then
        exit;
      mangledname:=ReplaceForbiddenAsmSymbolChars(def.mangledname);
      if target_info.system in systems_dotted_function_names then
        mangledname:='.'+mangledname;
      result.concat(tai_stab.create(stabx_function,
        strpnew(mangledname+','+mangledname+',16,044,LT.'+mangledname+'-'+mangledname)));
      { hoist the already generated ".bf" up right after the function
        definition so that all parameter and local variable definitions come
        after it -- we have to generate it during lineinfo generation and not
        here to make sure it takes into account include files opened right after
        the function definition but before the code starts
        -- also move include file start if any}
      hp:=def.procstarttai;
      inclstart:=nil;
      while (hp.typ<>ait_symbol_end) and
            ((hp.typ<>ait_stab) or
             (tai_stab(hp).stabtype<>stabx_bf)) do
        begin
          if (hp.typ=ait_stab) and
             (tai_stab(hp).stabtype=stabx_bi) then
            inclstart:=hp;
          hp:=tai(hp.next);
        end;
      { happens for implicit unit init routines and the like, they don't get
        line info }
      if hp.typ=ait_symbol_end then
        exit;
      if assigned(inclstart) then
        begin
          current_asmdata.asmlists[al_procedures].Remove(inclstart);
          result.concat(inclstart);
        end;
      current_asmdata.asmlists[al_procedures].Remove(hp);
      result.concat(hp);
      { also hoist up the function start symbol(s) }
      hp:=def.procstarttai;
      while assigned(hp) and
            (hp.typ<>ait_symbol_end) do
        begin
          if (hp.typ=ait_symbol) and
             (tai_symbol(hp).sym.typ=AT_FUNCTION) then
            begin
              hpp:=tai(hp.next);
              if hp=def.procstarttai then
                def.procstarttai:=hpp;
              current_asmdata.asmlists[al_procedures].Remove(hp);
              result.insert(hp);
              hp:=hpp;
            end
          else
            hp:=tai(hp.next);
        end;
    end;


  function TDebugInfoStabx.gen_procdef_endsym_stabs(def: tprocdef): TAsmList;
    var
      procendsymbol: tasmsymbol;
    begin
      result:=inherited gen_procdef_endsym_stabs(def);
      if not assigned(def.procstarttai) then
        exit;
      procendsymbol:=current_asmdata.DefineAsmSymbol('LT..'+ReplaceForbiddenAsmSymbolChars(def.mangledname),AB_LOCAL,AT_ADDR,voidpointertype);
      current_asmdata.asmlists[al_procedures].insertbefore(tai_symbol.create(procendsymbol,0),def.procendtai);
    end;


  procedure TDebugInfoStabx.appendsym_label(list: TAsmList; sym: tlabelsym);
    begin
      // ignore, not sure what kind of debug information we could generate for
      // this
    end;


  procedure TDebugInfoStabx.appendsym_staticvar(list: TAsmList; sym: tstaticvarsym);
    var
      ismem,
      isglobal: boolean;
    begin
      if vo_is_external in sym.varoptions then
        exit;
      ismem:=not(sym.localloc.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_MMREGISTER,LOC_CMMREGISTER,LOC_FPUREGISTER,LOC_CFPUREGISTER]);
      isglobal:=false;
      if ismem then
        isglobal:=current_asmdata.RefAsmSymbol(sym.mangledname,AT_DATA).bind=AB_GLOBAL;

      { put extra ss/es markers in place }
      if ismem then
        if isglobal then
          list.concat(tai_stab.Create_ansistr(stabx_bs,'.data[RW]'))
        else
          list.concat(tai_stab.Create_ansistr(stabx_bs,'_data.bss_'));
      inherited;
      if ismem then
        list.concat(tai_stab.Create_ansistr(stabx_es,''));
    end;


  procedure TDebugInfoStabx.insertlineinfo(list: TAsmList);
    var
      currfileinfo,
      lastfileinfo,
      curincludefileinfo,
      curfunstartfileinfo: tfileposinfo;
      currsectype  : TAsmSectiontype;
      hp, inclinsertpos, last : tai;
      infile : tinputfile;
      linenr, stabx_func_level,
      nolineinfolevel: longint;
      nextlineisfunstart: boolean;
    begin
      FillChar(currfileinfo,sizeof(currfileinfo),0);
      FillChar(lastfileinfo,sizeof(lastfileinfo),0);
      FillChar(curincludefileinfo,sizeof(curincludefileinfo),0);
      FillChar(curfunstartfileinfo,sizeof(curfunstartfileinfo),0);
      currsectype:=sec_code;
      hp:=Tai(list.first);
      nextlineisfunstart:=false;
      nolineinfolevel:=0;
      stabx_func_level:=0;
      last:=nil;
      while assigned(hp) do
        begin
          case hp.typ of
            ait_section :
              currsectype:=tai_section(hp).sectype;
            ait_force_line :
              lastfileinfo.line:=-1;
            ait_symbol:
              if tai_symbol(hp).sym.typ = AT_FUNCTION then
                nextlineisfunstart:=true;
            ait_symbol_end:
              if tai_symbol_end(hp).sym.typ = AT_FUNCTION then
                begin
                  { end of function }
                  if stabx_func_level > 0 then
                    begin
                      list.insertbefore(Tai_stab.Create_str(stabx_ef,tostr(currfileinfo.line)),hp);
                      dec(stabx_func_level);
                    end;
                end;
            ait_marker :
              begin
                case tai_marker(hp).kind of
                  mark_NoLineInfoStart:
                    inc(nolineinfolevel);
                  mark_NoLineInfoEnd:
                    dec(nolineinfolevel);
                  else
                    ;
                end;
              end;
            else
              ;
          end;

          if (currsectype=sec_code) and
             (hp.typ=ait_instruction) then
            begin
              currfileinfo:=tailineinfo(hp).fileinfo;

              inclinsertpos:=hp;
              while assigned(inclinsertpos.previous) and
                    (tai(inclinsertpos.previous).typ in (SkipInstr+[ait_marker])) do
                inclinsertpos:=tai(inclinsertpos.previous);

              { file changed ? (must be before line info) }
              if (currfileinfo.fileindex<>0) and
                 ((lastfileinfo.fileindex<>currfileinfo.fileindex) or
                  (lastfileinfo.moduleindex<>currfileinfo.moduleindex)) then
                begin
                  if curincludefileinfo.fileindex<>0 then
                    begin
                      infile:=get_module(curincludefileinfo.moduleindex).sourcefiles.get_file(curincludefileinfo.fileindex);
                      list.insertbefore(Tai_stab.Create_str(stabx_ei,'"'+FixFileName(infile.name)+'"'),inclinsertpos);
                      curincludefileinfo.fileindex:=0;
                    end;
                  if currfileinfo.fileindex<>1 then
                    begin
                      infile:=get_module(currfileinfo.moduleindex).sourcefiles.get_file(currfileinfo.fileindex);
                      if assigned(infile) then
                        begin
                          list.insertbefore(Tai_stab.Create_str(stabx_bi,'"'+FixFileName(infile.name)+'"'),inclinsertpos);
                          curincludefileinfo:=currfileinfo;
                          { force new line info }
                          lastfileinfo.line:=-1;
                        end;
                    end
                  else
                    lastfileinfo.line:=-1;
                  if nextlineisfunstart then
                    begin
                      curfunstartfileinfo:=currfileinfo;
                      { insert here rather than via procdef, because the procdef
                        may have been created in another file in case the body
                        is completely declared in an include file }
                      list.insertbefore(Tai_stab.Create_str(stabx_bf,tostr(currfileinfo.line)),hp);
                      inc(stabx_func_level);
                      { -1 to avoid outputting a relative line 0 in the
                        function, because that means something different }
                      dec(curfunstartfileinfo.line);
                      nextlineisfunstart:=false;
                    end;

                end;

              { implicit functions have no file information }
              if nextlineisfunstart then
                begin
                  list.insertbefore(Tai_stab.Create_str(stabx_bf,tostr(currfileinfo.line)),hp);
                  inc(stabx_func_level);
                  nextlineisfunstart:=false;
                end;
              if nolineinfolevel=0 then
                begin
                  { line changed ? }
                  if (currfileinfo.line>lastfileinfo.line) and
                     (currfileinfo.line<>0) then
                    begin
                      linenr:=currfileinfo.line;
                      { line numbers in AIX are relative to the function start line
                        (except if they are in a different file then where the
                         function started!) }
                      if (currfileinfo.fileindex=curfunstartfileinfo.fileindex) and
                         (currfileinfo.moduleindex=curfunstartfileinfo.moduleindex) then
                        dec(linenr,curfunstartfileinfo.line);
                      { can be < 0 in case of bugs in the compiler }
                      if (linenr > 0)
{$ifndef cpu64bitaddr}
                         { line numbers are unsigned short in 32 bit xcoff }
                         and (linenr<=high(word))
{$endif}
                        then
                         list.insertbefore(Tai_stab.Create_str(stabx_line,tostr(linenr)),hp);
                    end;
                  lastfileinfo:=currfileinfo;
                end;
            end;

          last:=hp;
          hp:=tai(hp.next);
        end;
      { close include file if still open }
      if curincludefileinfo.fileindex<>0 then
        begin
          infile:=get_module(curincludefileinfo.moduleindex).sourcefiles.get_file(curincludefileinfo.fileindex);
          list.insertbefore(Tai_stab.Create_str(stabx_ei,'"'+FixFileName(infile.name)+'"'),last);
          curincludefileinfo.fileindex:=0;
        end;
    end;


  procedure TDebugInfoStabx.insertmoduleinfo;
    begin
      // do nothing
    end;


  procedure TDebugInfoStabx.referencesections(list: TAsmList);
    begin
      // do nothing
    end;


  constructor TDebugInfoStabx.create;
    begin
      inherited create;
      dbgtype:=dbg_stabx;
      stabsdir:=stab_stabx;

      def_stab:=STABX_N_DECL;
      regvar_stab:=STABX_N_RPSYM;
      procdef_stab:=STABX_N_Function;
      constsym_stab:=STABX_N_GSYM;
      typesym_stab:=STABX_N_DECL;
      globalvarsym_uninited_stab:=STABX_N_STSYM;
      globalvarsym_inited_stab:=STABX_N_STSYM;
      staticvarsym_uninited_stab:=STABX_N_STSYM;
      staticvarsym_inited_stab:=STABX_N_STSYM;
      localvarsymref_stab:=STABX_N_LSYM;
      paravarsymref_stab:=STABX_N_PSYM;

      tagtypeprefix:='T';
    end;

  const
    dbg_stabx_info : tdbginfo =
       (
         id     : dbg_stabx;
         idtxt  : 'STABX';
       );

initialization
  RegisterDebugInfo(dbg_stabx_info,TDebugInfoStabx);
end.
