{
    Copyright (c) 2003-2004 by Peter Vreman and Florian Klaempfl

    This units contains support for STABS debug info generation

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
unit dbgstabs;

{$i fpcdefs.inc}

interface

    uses
      cclasses,
      systems,dbgbase,cgbase,
      symconst,symtype,symdef,symsym,symtable,symbase,
      aasmtai,aasmdata;

    const
      { stab types }
      STABS_N_GSYM = $20;
      STABS_N_STSYM = 38;     { initialized const }
      STABS_N_LCSYM = 40;     { non initialized variable}
      STABS_N_Function = $24; { function or const }
      STABS_N_TextLine = $44;
      STABS_N_DataLine = $46;
      STABS_N_BssLine = $48;
      STABS_N_RSYM = $40;     { register variable }
      STABS_N_LSYM = $80;
      STABS_N_DECL = $8c;
      STABS_N_RPSYM = $8e;
      STABS_N_tsym = 160;
      STABS_N_SourceFile = $64;
{ APPLE LOCAL N_OSO: This is the stab that associated the .o file with the
   N_SO stab, in the case where debug info is mostly stored in the .o file.  }
      STABS_N_OSO        = $66;
      STABS_N_IncludeFile = $84;
      STABS_N_BINCL = $82;
      STABS_N_EINCL = $A2;
      STABS_N_LBRAC = $C0;
      STABS_N_EXCL  = $C2;
      STABS_N_RBRAC = $E0;

    type
      TDebugInfoStabs=class(TDebugInfo)
      protected
        dbgtype: tdbg;
        stabsdir: TStabType;
        def_stab,
        regvar_stab,
        procdef_stab,
        constsym_stab,
        typesym_stab,
        globalvarsym_uninited_stab,
        globalvarsym_inited_stab,
        staticvarsym_uninited_stab,
        staticvarsym_inited_stab,
        localvarsymref_stab,
        paravarsymref_stab: byte;
        writing_def_stabs  : boolean;
        global_stab_number : word;
        vardatadef: trecorddef;
        tagtypeprefix: ansistring;
        function use_tag_prefix(def : tdef) : boolean;
        { tsym writing }
        function  sym_var_value(const s:string;arg:pointer):string;
        function  sym_stabstr_evaluate(sym:tsym;const s:string;const vars:array of string):ansistring;
        procedure write_sym_stabstr(list:TAsmList;sym:tsym;const ss:ansistring);
        function  staticvarsym_mangled_name(sym: tstaticvarsym):string;virtual;
        procedure maybe_add_vmt_sym(list:TAsmList;def: tobjectdef);virtual;
        { tdef writing }
        function  def_stab_number(def:tdef):string;
        function  def_stab_classnumber(def:tabstractrecorddef):string;
        function  def_var_value(const s:string;arg:pointer):string;
        function  def_stabstr_evaluate(def:tdef;const s:string;const vars:array of string):ansistring;
        procedure write_def_stabstr(list:TAsmList;def:tdef;const ss:ansistring);virtual;
        procedure field_add_stabstr(p:TObject;arg:pointer);
        procedure method_add_stabstr(p:TObject;arg:pointer);
        procedure field_write_defs(p:TObject;arg:pointer);
        function  get_enum_defstr(def: tenumdef; lowerbound: longint): ansistring;
        function  get_appendsym_paravar_reg(sym:tparavarsym;const typ,stabstr:string;reg: tregister): ansistring;
        function  base_stabs_str(typ: longint; const other, desc, value: ansistring): ansistring;overload;
        function  base_stabs_str(const typ, other, desc, value: ansistring): ansistring;overload;virtual;
        function  gen_procdef_startsym_stabs(def: tprocdef): TAsmList;virtual;
        function  gen_procdef_endsym_stabs(def: tprocdef): TAsmList;virtual;
      protected
        procedure appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);override;
        procedure appendsym_paravar(list:TAsmList;sym:tparavarsym);override;
        procedure appendsym_localvar(list:TAsmList;sym:tlocalvarsym);override;
        procedure appendsym_fieldvar(list:TAsmList;sym:tfieldvarsym);override;
        procedure appendsym_const(list:TAsmList;sym:tconstsym);override;
        procedure appendsym_type(list:TAsmList;sym:ttypesym);override;
        procedure appendsym_label(list:TAsmList;sym:tlabelsym);override;
        procedure beforeappenddef(list:TAsmList;def:tdef);override;
        procedure appenddef_ord(list:TAsmList;def:torddef);override;
        procedure appenddef_float(list:TAsmList;def:tfloatdef);override;
        procedure appenddef_file(list:TAsmList;def:tfiledef);override;
        procedure appenddef_enum(list:TAsmList;def:tenumdef);override;
        procedure appenddef_array(list:TAsmList;def:tarraydef);override;
        procedure appenddef_record(list:TAsmList;def:trecorddef);override;
        procedure appenddef_object(list:TAsmList;def:tobjectdef);override;
        procedure appenddef_pointer(list:TAsmList;def:tpointerdef);override;
        procedure appenddef_string(list:TAsmList;def:tstringdef);override;
        procedure appenddef_procvar(list:TAsmList;def:tprocvardef);override;
        procedure appenddef_variant(list:TAsmList;def:tvariantdef);override;
        procedure appenddef_set(list:TAsmList;def:tsetdef);override;
        procedure appenddef_formal(list:TAsmList;def:tformaldef);override;
        procedure appenddef_undefined(list:TAsmList;def: tundefineddef);override;
        procedure appendprocdef(list:TAsmList;def:tprocdef);override;
      public
        procedure inserttypeinfo;override;
        procedure insertmoduleinfo;override;
        procedure insertlineinfo(list:TAsmList);override;
        procedure referencesections(list:TAsmList);override;

        constructor Create;override;
      end;


    function GetSymTableName(SymTable : TSymTable) : string;

    const
      tagtypes = [
        recorddef,
        variantdef,
        enumdef,
        stringdef,
        filedef,
        objectdef
      ];


implementation

    uses
{$ifdef MIPS}
      { we need taicpu definition to add .set nomips16 pseudo-instruction
        before any procedure/function reference }
      aasmcpu,
{$endif}
      SysUtils,cutils,cfileutl,
      globals,globtype,verbose,constexp,
      defutil, cgutils, parabase,
      cpuinfo,cpubase,cpupi,paramgr,
      aasmbase,procinfo,
      finput,fmodule,ppu,
      symutil;


    const
      current_procdef : tprocdef = nil;

    function GetOffsetStr(reference : TReference) : string;
    begin
{$ifdef MIPS}
      if (reference.index=NR_STACK_POINTER_REG) or
         (reference.base=NR_STACK_POINTER_REG)  then
        GetOffsetStr:=tostr(reference.offset
          - mips_extra_offset(current_procdef))
      else
{$endif MIPS}
      GetOffsetStr:=tostr(reference.offset);
    end;

    function GetParaOffsetStr(reference : TCGParaReference) : string;
    begin
{$ifdef MIPS}
      if reference.index=NR_STACK_POINTER_REG then
        GetParaOffsetStr:=tostr(reference.offset
          - mips_extra_offset(current_procdef))
      else
{$endif MIPS}
      GetParaOffsetStr:=tostr(reference.offset);
    end;

    function GetSymName(Sym : TSymEntry) : string;
    begin
      if Not (cs_stabs_preservecase in current_settings.globalswitches) then
        result := Sym.Name
      else
        result := Sym.RealName;
      if (Sym.typ=typesym) and (ttypesym(Sym).Fprettyname<>'') then
        result:=ttypesym(Sym).FPrettyName;
      if target_asm.dollarsign<>'$' then
        result:=ReplaceForbiddenAsmSymbolChars(result);
    end;

    function GetSymTableName(SymTable : TSymTable) : string;
    begin
      if Not (cs_stabs_preservecase in current_settings.globalswitches) then
        result := SymTable.Name^
      else
        result := SymTable.RealName^;
      if target_asm.dollarsign<>'$' then
        result:=ReplaceForbiddenAsmSymbolChars(result);
    end;

    const
      memsizeinc = 512;

    type
       get_var_value_proc=function(const s:string;arg:pointer):string of object;


    function string_evaluate(s:string;get_var_value:get_var_value_proc;get_var_value_arg:pointer;const vars:array of string):ansistring;
    (*
     S contains a prototype of a result. Stabstr_evaluate will expand
     variables and parameters.

     Output is s in ASCIIZ format, with the following expanded:

     ${varname}   - The variable name is expanded.
     $n           - The parameter n is expanded.
     $$           - Is expanded to $
    *)

    const maxvalue=9;
          maxdata=1023;

    var i,j:byte;
        varname:string[63];
        varno,varcounter:byte;
        varvalues:array[0..9] of pshortstring;
        {1 kb of parameters is the limit. 256 extra bytes are allocated to
         ensure buffer integrity.}
        varvaluedata:array[0..maxdata+256] of char;
        varptr:Pchar;
        varidx : byte;
        len:longint;
        r:Pchar;

    begin
      {Two pass approach, first, calculate the length and receive variables.}
      i:=1;
      len:=0;
      varcounter:=0;
      varptr:=@varvaluedata[0];
      varvalues[0]:=nil;
      result:='';
      while i<=length(s) do
        begin
          if (s[i]='$') and (i<length(s)) then
            begin
             if s[i+1]='$' then
               begin
                 inc(len);
                 inc(i);
               end
             else if (s[i+1]='{') and (length(s)>2) and (i<length(s)-2) then
               begin
                 varname:='';
                 inc(i,2);
                 repeat
                   inc(varname[0]);
                   varname[length(varname)]:=s[i];
                   s[i]:=char(varcounter);
                   inc(i);
                 until s[i]='}';
                 varvalues[varcounter]:=pshortstring(varptr);
                 if varptr>@varvaluedata[maxdata] then
                   internalerrorproc(200411152);
                 pshortstring(varptr)^:=get_var_value(varname,get_var_value_arg);
                 inc(len,length(pshortstring(varptr)^));
                 inc(varptr,length(pshortstring(varptr)^)+1);
                 inc(varcounter);
               end
             else if s[i+1] in ['1'..'9'] then
               begin
                 varidx:=byte(s[i+1])-byte('1');
                 if varidx>high(vars) then
                   internalerror(200509263);
                 inc(len,length(vars[varidx]));
                 inc(i);
               end;
            end
          else
            inc(len);
          inc(i);
        end;

      {Second pass, writeout result.}
      setlength(result,len);
      r:=pchar(result);
      i:=1;
      while i<=length(s) do
        begin
          if (s[i]='$') and (i<length(s)) then
            begin
             if s[i+1]='$' then
               begin
                 r^:='$';
                 inc(r);
                 inc(i);
               end
             else if (s[i+1]='{') and (length(s)>2) and (i<length(s)-2) then
               begin
                 varname:='';
                 inc(i,2);
                 varno:=byte(s[i]);
                 repeat
                   inc(i);
                 until s[i]='}';
                 for j:=1 to length(varvalues[varno]^) do
                   begin
                     r^:=varvalues[varno]^[j];
                     inc(r);
                   end;
               end
             else if s[i+1] in ['0'..'9'] then
               begin
                 for j:=1 to length(vars[byte(s[i+1])-byte('1')]) do
                   begin
                     r^:=vars[byte(s[i+1])-byte('1')][j];
                     inc(r);
                   end;
                 inc(i);
               end
            end
          else
            begin
              r^:=s[i];
              inc(r);
            end;
          inc(i);
        end;
      { verify that the length was correct }
      if r^<>#0 then
        internalerror(200802031);
    end;


{****************************************************************************
                               TDef support
****************************************************************************}

    function TDebugInfoStabs.def_stab_number(def:tdef):string;
      begin
        { procdefs only need a number, mark them as already written
          so they won't be written implicitly }
        if (def.typ=procdef) then
          def.dbg_state:=dbg_state_written;

        { Stab must already be written, or we must be busy writing it }
        if writing_def_stabs and
           not(def.dbg_state in [dbg_state_writing,dbg_state_written,dbg_state_queued]) then
          internalerror(200403091);

        { Keep track of used stabs, this info is only useful for stabs
          referenced by the symbols. Definitions will always include all
          required stabs }
        if def.dbg_state=dbg_state_unused then
          begin
            def.dbg_state:=dbg_state_used;
            deftowritelist.Add(def);
          end;
        { Need a new number? }
        if def.stab_number=0 then
          begin
            inc(global_stab_number);
            { classes require 2 numbers }
            if is_class(def) then
              inc(global_stab_number);
            def.stab_number:=global_stab_number;
            if global_stab_number>=defnumberlist.count then
              defnumberlist.count:=global_stab_number+250;
            defnumberlist[global_stab_number]:=def;
          end;
        result:=tostr(def.stab_number);
      end;


    function TDebugInfoStabs.def_stab_classnumber(def:tabstractrecorddef):string;
      begin
        if def.stab_number=0 then
          def_stab_number(def);
        if (def.typ=objectdef) and (tobjectdef(def).objecttype=odt_class) then
          result:=tostr(def.stab_number-1)
        else
          result:=tostr(def.stab_number);
      end;


    function TDebugInfoStabs.def_var_value(const s:string;arg:pointer):string;
      var
        def : tdef;
      begin
        def:=tdef(arg);
        result:='';
        if s='numberstring' then
          result:=def_stab_number(def)
        else if s='sym_name' then
          begin
            if assigned(def.typesym) then
               result:=GetSymName(Ttypesym(def.typesym));
          end
        else if s='savesize' then
          result:=tostr(def.size);
      end;


    function TDebugInfoStabs.def_stabstr_evaluate(def:tdef;const s:string;const vars:array of string):ansistring;
      begin
        result:=string_evaluate(s,@def_var_value,def,vars);
      end;


    procedure TDebugInfoStabs.field_add_stabstr(p:TObject;arg:pointer);
      var
        spec    : string[3];
        varsize : asizeint;
        newss   : ansistring;
        ss      : pansistring absolute arg;
      begin
        if (tsym(p).visibility=vis_hidden) then
          exit;
        { static variables from objects are like global objects }
        if is_normal_fieldvarsym(Tsym(p)) then
          begin
           case tsym(p).visibility of
             vis_private,
             vis_strictprivate :
               spec:='/0';
             vis_protected,
             vis_strictprotected :
               spec:='/1';
             else
               spec:='';
           end;
           if (tabstractrecordsymtable(tsym(p).owner).usefieldalignment<>bit_alignment) then
              begin
                varsize:=tfieldvarsym(p).vardef.size;
                { open arrays made overflows !! }
                { how can a record/object/class contain an open array? (JM) }
{$ifdef cpu16bitaddr}
                if varsize>$fff then
                  varsize:=$fff;
{$else cpu16bitaddr}
                if varsize>$fffffff then
                  varsize:=$fffffff;
{$endif cpu16bitaddr}
                newss:=def_stabstr_evaluate(nil,'$1:$2,$3,$4;',[GetSymName(tfieldvarsym(p)),
                                     spec+def_stab_number(tfieldvarsym(p).vardef),
                                     tostr(TConstExprInt(tfieldvarsym(p).fieldoffset)*8),tostr(varsize*8)])
              end
            else
              newss:=def_stabstr_evaluate(nil,'$1:$2,$3,$4;',[GetSymName(tfieldvarsym(p)),
                                   spec+def_stab_number(tfieldvarsym(p).vardef),
                                   tostr(TConstExprInt(tfieldvarsym(p).fieldoffset)),tostr(tfieldvarsym(p).vardef.packedbitsize)]);
            ss^:=ss^+newss;
          end;
      end;


    procedure TDebugInfoStabs.method_add_stabstr(p:TObject;arg:pointer);
      var
        virtualind,argnames : string;
        pd     : tprocdef;
        lindex : longint;
        arglength : byte;
        sp : char;
        i : integer;
        parasym : tparavarsym;
        newss   : ansistring;
        ss      : pansistring absolute arg;
      begin
        if tsym(p).typ = procsym then
         begin
           pd :=tprocdef(tprocsym(p).ProcdefList[0]);
           if (po_virtualmethod in pd.procoptions) and
               not is_objectpascal_helper(pd.struct) then
             begin
               lindex := pd.extnumber;
               {doesnt seem to be necessary
               lindex := lindex or $80000000;}
               virtualind := '*'+tostr(lindex)+';'+def_stab_classnumber(pd.struct)+';'
             end
            else
             virtualind := '.';

            { used by gdbpas to recognize constructor and destructors }
            if (pd.proctypeoption=potype_constructor) then
              argnames:='__ct__'
            else if (pd.proctypeoption=potype_destructor) then
              argnames:='__dt__'
            else
              argnames := '';

           { arguments are not listed here }
           {we don't need another definition}
            for i:=0 to pd.paras.count-1 do
              begin
                parasym:=tparavarsym(pd.paras[i]);
                if Parasym.vardef.typ = formaldef then
                  begin
                    case Parasym.varspez of
                      vs_var :
                        argnames := argnames+'3var';
                      vs_const :
                        argnames:=argnames+'5const';
                      vs_out :
                        argnames:=argnames+'3out';
                      vs_constref :
                        argnames:=argnames+'8constref';
                      vs_value :
                        ;
                      vs_final:
                        internalerror(2019050911);
                    end;
                  end
                else
                  begin
                    { if the arg definition is like (v: ^byte;..
                    there is no sym attached to data !!! }
                    if assigned(Parasym.vardef.typesym) then
                      begin
                        arglength := length(GetSymName(Parasym.vardef.typesym));
                        argnames := argnames + tostr(arglength)+GetSymName(Parasym.vardef.typesym);
                      end
                    else
                      argnames:=argnames+'11unnamedtype';
                  end;
              end;
           { here 2A must be changed for private and protected }
           { 0 is private 1 protected and 2 public }
           case tsym(p).visibility of
             vis_private,
             vis_strictprivate :
               sp:='0';
             vis_protected,
             vis_strictprotected :
               sp:='1'
             else
               sp:='2';
           end;
           newss:=def_stabstr_evaluate(nil,'$1::$2=##$3;:$4;$5A$6;',[GetSymName(tsym(p)),def_stab_number(pd),
                                    def_stab_number(pd.returndef),argnames,sp,
                                    virtualind]);
           ss^:=ss^+newss;
         end;
      end;


    procedure TDebugInfoStabs.field_write_defs(p:TObject;arg:pointer);
      begin
        if is_normal_fieldvarsym(Tsym(p)) then
          appenddef(TAsmList(arg),tfieldvarsym(p).vardef);
      end;

    function TDebugInfoStabs.use_tag_prefix(def : tdef) : boolean;
      begin
        { stringdefs are not all considered as 'taggable',
          because ansi, unicode and wide strings are
          just associated to pointer types }
        use_tag_prefix:=(def.typ in tagtypes) and
                      ((def.typ<>stringdef) or
                       (tstringdef(tdef).stringtype in [st_shortstring,st_longstring]));
      end;


    procedure TDebugInfoStabs.write_def_stabstr(list:TAsmList;def:tdef;const ss:ansistring);
      var
        stabchar : string[2];
        symname  : string[20];
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
          st:=def_stabstr_evaluate(def,'"${sym_name}:$1$2=',[stabchar,def_stab_classnumber(tobjectdef(def))])
        else
          begin
            { Type names for types defined in the current unit are already written in
              the typesym }
            if (def.owner.symtabletype=globalsymtable) and
               not(def.owner.iscurrentunit) then
              symname:='${sym_name}'
            else
              symname:='';
            st:=def_stabstr_evaluate(def,'"'+symname+':$1$2=',[stabchar,def_stab_number(def)]);
          end;
        st:=st+ss;
        { line info is set to 0 for all defs, because the def can be in another
          unit and then the linenumber is invalid in the current sourcefile }
        st:=st+def_stabstr_evaluate(def,'",'+base_stabs_str(def_stab,'0','0','0'),[]);
        { add to list }
        list.concat(Tai_stab.create_ansistr(stabsdir,st));
      end;


    procedure TDebugInfoStabs.appenddef_string(list:TAsmList;def:tstringdef);
      var
        bytest,charst,longst : string;
        ss : ansistring;
        slen : longint;
      begin
        ss:='';
        case def.stringtype of
          st_shortstring:
            begin
              { fix length of openshortstring }
              slen:=def.len;
              if slen=0 then
                slen:=255;
              charst:=def_stab_number(cansichartype);
              bytest:=def_stab_number(u8inttype);
              ss:=def_stabstr_evaluate(def,'s$1length:$2,0,8;st:ar$2;1;$3;$4,8,$5;;',
                          [tostr(slen+1),bytest,tostr(slen),charst,tostr(slen*8)]);
            end;
          st_longstring:
            begin
              charst:=def_stab_number(cansichartype);
              bytest:=def_stab_number(u8inttype);
              longst:=def_stab_number(u32inttype);
              ss:=def_stabstr_evaluate(def,'s$1length:$2,0,32;dummy:$6,32,8;st:ar$2;1;$3;$4,40,$5;;',
                          [tostr(def.len+5),longst,tostr(def.len),charst,tostr(def.len*8),bytest]);
           end;
         st_ansistring:
           begin
             { looks like a pchar }
             ss:='*'+def_stab_number(cansichartype);
           end;
         st_unicodestring,
         st_widestring:
           begin
             { looks like a pwidechar }
             ss:='*'+def_stab_number(cwidechartype);
           end;
        end;
        write_def_stabstr(list,def,ss);
      end;


    function TDebugInfoStabs.get_enum_defstr(def: tenumdef; lowerbound: longint): ansistring;
      var
        i: longint;
        p: tenumsym;
      begin
        { we can specify the size with @s<size>; prefix PM }
        if def.size <> std_param_align then
          result:='@s'+tostr(def.size*8)+';e'
        else
          result:='e';
        { the if-test is required because pred(def.minval) might overflow;
          the longint() typecast should be safe because stabs is not
          supported for 64 bit targets }
        if (def.minval<>lowerbound) then
          for i:=lowerbound to pred(longint(def.minval)) do
            result:=result+'<invalid>:'+tostr(i)+',';

        for i := 0 to def.symtable.SymList.Count - 1 do
          begin
            p := tenumsym(def.symtable.SymList[i]);
            if p.value<def.minval then
              continue
            else
            if p.value>def.maxval then
              break;
            result:=result+GetSymName(p)+':'+tostr(p.value)+',';
          end;
        { the final ',' is required to have a valid stabs }
        result:=result+';';
      end;

    procedure TDebugInfoStabs.appenddef_enum(list:TAsmList;def:tenumdef);
      begin
        write_def_stabstr(list,def,get_enum_defstr(def,def.minval));
      end;


    procedure TDebugInfoStabs.appenddef_ord(list:TAsmList;def:torddef);
      var
        ss : ansistring;
      begin
        ss:='';
        if cs_gdb_valgrind in current_settings.globalswitches then
          begin
            case def.ordtype of
              uvoid :
                ss:=def_stab_number(def);
              pasbool1,
              pasbool8,
              pasbool16,
              pasbool32,
              pasbool64,
              bool8bit,
              bool16bit,
              bool32bit,
              bool64bit :
                ss:=def_stabstr_evaluate(def,'r${numberstring};0;255;',[]);
              u32bit,
              s64bit,
              u64bit,
              s128bit,
              u128bit:
                ss:=def_stabstr_evaluate(def,'r${numberstring};0;-1;',[]);
              else
                ss:=def_stabstr_evaluate(def,'r${numberstring};$1;$2;',[tostr(longint(def.low.svalue)),tostr(longint(def.high.svalue))]);
            end;
          end
        else
          begin
            case def.ordtype of
              uvoid :
                ss:=def_stab_number(def);
              uchar :
                ss:='-20;';
              uwidechar :
                ss:='-30;';
              pasbool1,
              pasbool8,
              bool8bit :
                ss:='-21;';
              pasbool16,
              bool16bit :
                ss:='-22;';
              pasbool32,
              bool32bit :
                ss:='-23;';
              pasbool64,
              bool64bit :
                { no clue if this is correct (FK) }
                ss:='-23;';
              u64bit :
                ss:='-32;';
              s64bit :
                ss:='-31;';
              {u32bit : result:=def_stab_number(s32inttype)+';0;-1;'); }
              else
                begin
                  if def.size <> std_param_align then
                    ss:='@s'+tostr(def.size*8)+';'
                  else
                    ss:='';
                  ss:=ss+def_stabstr_evaluate(def,'r${numberstring};$1;$2;',[tostr(longint(def.low.svalue)),tostr(longint(def.high.svalue))]);
                end;
            end;
         end;
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_float(list:TAsmList;def:tfloatdef);
      var
        ss : ansistring;
      begin
        ss:='';
        case def.floattype of
          s32real,
          s64real,
          s80real,
          sc80real:
            ss:=def_stabstr_evaluate(def,'r$1;${savesize};0;',[def_stab_number(s32inttype)]);
          s64currency,
          s64comp:
            ss:=def_stabstr_evaluate(def,'r$1;-${savesize};0;',[def_stab_number(s32inttype)]);
          else
            internalerror(200509261);
        end;
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_file(list:TAsmList;def:tfiledef);
      var
        ss : ansistring;
      begin
{$ifdef cpu64bitaddr}
        ss:=def_stabstr_evaluate(def,'s${savesize}HANDLE:$1,0,32;MODE:$1,32,32;RECSIZE:$2,64,64;'+
                                 '_PRIVATE:ar$1;1;64;$3,128,256;USERDATA:ar$1;1;32;$3,384,256;'+
                                 'NAME:ar$1;0;255;$4,640,2048;;',[def_stab_number(s32inttype),
                                 def_stab_number(s64inttype),
                                 def_stab_number(u8inttype),
                                 def_stab_number(cansichartype)]);
{$else cpu64bitaddr}
        ss:=def_stabstr_evaluate(def,'s${savesize}HANDLE:$1,0,32;MODE:$1,32,32;RECSIZE:$1,64,32;'+
                                 '_PRIVATE:ar$1;1;32;$3,96,256;USERDATA:ar$1;1;32;$2,352,256;'+
                                 'NAME:ar$1;0;255;$3,608,2048;;',[def_stab_number(s32inttype),
                                 def_stab_number(u8inttype),
                                 def_stab_number(cansichartype)]);
{$endif cpu64bitaddr}
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_record(list:TAsmList;def:trecorddef);
      var
        ss : ansistring;
      begin
        ss:='s'+tostr(def.size);
        def.symtable.SymList.ForEachCall(@field_add_stabstr,@ss);
        ss[length(ss)]:=';';
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_object(list:TAsmList;def:tobjectdef);

        procedure do_write_object(list:TAsmList;def:tobjectdef);
        var
          ss : ansistring;
          anc    : tobjectdef;
        begin
          ss:='';
          { Write the invisible pointer for the class? }
          if (def.objecttype=odt_class) and
             (not def.writing_class_record_dbginfo) then
            begin
              ss:='*'+def_stab_classnumber(def);
              write_def_stabstr(list,def,ss);
              exit;
            end;

          ss:='s'+tostr(tobjecTSymtable(def.symtable).datasize);
          if assigned(def.childof) then
            begin
              {only one ancestor not virtual, public, at base offset 0 }
              {       !1           ,    0       2         0    ,       }
              ss:=ss+'!1,020,'+def_stab_classnumber(def.childof)+';';
            end;

          {virtual table to implement yet}
          def.symtable.symList.ForEachCall(@field_add_stabstr,@ss);

          if (oo_has_vmt in def.objectoptions) and
             (
              not assigned(def.childof) or
              not(oo_has_vmt in def.childof.objectoptions)
             ) then
            ss:=ss+'$vf'+def_stab_classnumber(def)+':'+def_stab_number(vmtarraytype)+','+tostr(def.vmt_offset*8)+';';
          def.symtable.symList.ForEachCall(@method_add_stabstr,@ss);
          if (oo_has_vmt in def.objectoptions) then
            begin
               anc := def;
               while assigned(anc.childof) and (oo_has_vmt in anc.childof.objectoptions) do
                 anc := anc.childof;
               { just in case anc = self }
               ss:=ss+';~%'+def_stab_classnumber(anc)+';';
            end
          else
            ss:=ss+';';
          write_def_stabstr(list,def,ss);
        end;

      var
        oldtypesym : tsym;
      begin
        tobjectdef(def).symtable.symList.ForEachCall(@field_write_defs,list);
        { classes require special code to write the record and the invisible pointer }
        if is_class(def) then
          begin
            { Write the record class itself }
            tobjectdef(def).writing_class_record_dbginfo:=true;
            do_write_object(list,def);
            tobjectdef(def).writing_class_record_dbginfo:=false;
            { Write the invisible pointer class }
            oldtypesym:=def.typesym;
            def.typesym:=nil;
            do_write_object(list,def);
            def.typesym:=oldtypesym;
          end
        else
          do_write_object(list,def);
        { VMT symbol }
        maybe_add_vmt_sym(list,def);
      end;


    procedure TDebugInfoStabs.appenddef_variant(list:TAsmList;def:tvariantdef);
      var
        ss : ansistring;
      begin
        if not assigned(vardatadef) then
          exit;

        ss:='s'+tostr(vardatadef.size);
        vardatadef.symtable.SymList.ForEachCall(@field_add_stabstr,@ss);
        ss[length(ss)]:=';';
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_pointer(list:TAsmList;def:tpointerdef);
      var
        ss : ansistring;
      begin
        ss:='*'+def_stab_number(tpointerdef(def).pointeddef);
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_set(list:TAsmList;def:tsetdef);
      var
        st,
        ss : ansistring;
        elementdefstabnr: string;
      begin
        { ugly hack: create a temporary subrange type if the lower bound of
          the set's element type is not a multiple of 8 (because we store them
          as if the lower bound is a multiple of 8) }
        if (def.setbase<>get_min_value(def.elementdef)) then
          begin
            { allocate a def number }
            inc(global_stab_number);
            elementdefstabnr:=tostr(global_stab_number);
            { anonymous subrange def }
            st:='":t'+elementdefstabnr+'=';
            if (def.elementdef.typ = enumdef) then
              st:=st+get_enum_defstr(tenumdef(def.elementdef),def.setbase)
            else
              st:=st+def_stabstr_evaluate(def.elementdef,'r'+elementdefstabnr+';$1;$2;',[tostr(longint(def.setbase)),tostr(longint(get_max_value(def.elementdef).svalue))]);
            st:=st+'",'+base_stabs_str(def_stab,'0','0','0');
            { add to list }
            list.concat(Tai_stab.create_ansistr(stabsdir,st));
          end
        else
          elementdefstabnr:=def_stab_number(def.elementdef);
        ss:=def_stabstr_evaluate(def,'@s$1;S$2',[tostr(def.size*8),elementdefstabnr]);
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_formal(list:TAsmList;def:tformaldef);
      var
        ss : ansistring;
      begin
        ss:=def_stabstr_evaluate(def,'${numberstring};',[]);
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_array(list:TAsmList;def:tarraydef);
      var
        tempstr: shortstring;
        ss : ansistring;
      begin
        if not is_packed_array(def) then
          begin
            { Try to used P if ememlent size is smaller than
              usual integer }
            if def.elesize <> std_param_align then
              tempstr:='ar@s'+tostr(def.elesize*8)+';$1;$2;$3;$4'
            else
              tempstr:='ar$1;$2;$3;$4';
            if is_dynamic_array(def) then
              tempstr:='*'+tempstr;
            ss:=def_stabstr_evaluate(def,tempstr,[def_stab_number(tarraydef(def).rangedef),
                     tostr(tarraydef(def).lowrange),tostr(tarraydef(def).highrange),def_stab_number(tarraydef(def).elementdef)])
          end
        else
          begin
            // the @P seems to be ignored by gdb
            tempstr:=def_stabstr_evaluate(tarraydef(def).rangedef,'r${numberstring};$1;$2;',
              [tostr(tarraydef(def).lowrange),tostr(tarraydef(def).highrange)]);
            // will only show highrange-lowrange+1 bits in gdb
            ss:=def_stabstr_evaluate(def,'@s$1;@S;S$2',
              [tostr(TConstExprInt(tarraydef(def).elepackedbitsize) * tarraydef(def).elecount),tempstr]);
          end;
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_procvar(list:TAsmList;def:tprocvardef);
      var
        ss : ansistring;
      begin
        ss:='*f'+def_stab_number(tprocvardef(def).returndef);
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.appenddef_undefined(list:TAsmList;def:tundefineddef);
      var
        ss : ansistring;
      begin
        ss:=def_stabstr_evaluate(def,'${numberstring};',[]);
        write_def_stabstr(list,def,ss);
      end;


    procedure TDebugInfoStabs.beforeappenddef(list:TAsmList;def:tdef);
      var
        anc : tobjectdef;
        i : longint;
      begin
        { write dependencies first }
        case def.typ of
          stringdef :
            begin
              if tstringdef(def).stringtype in [st_widestring,st_unicodestring] then
                appenddef(list,cwidechartype)
              else
                begin
                  appenddef(list,cansichartype);
                  appenddef(list,u8inttype);
                end;
            end;
          floatdef :
            appenddef(list,s32inttype);
          filedef :
            begin
              appenddef(list,s32inttype);
{$ifdef cpu64bitaddr}
              appenddef(list,s64inttype);
{$endif cpu64bitaddr}
              appenddef(list,u8inttype);
              appenddef(list,cansichartype);
            end;
          classrefdef :
            appenddef(list,pvmttype);
          pointerdef :
            appenddef(list,tpointerdef(def).pointeddef);
          setdef :
            appenddef(list,tsetdef(def).elementdef);
          procvardef :
            begin
              appenddef(list,tprocvardef(def).returndef);
              if assigned(tprocvardef(def).parast) then
                write_symtable_defs(list,tprocvardef(def).parast);
            end;
          procdef :
            begin
              appenddef(list,tprocdef(def).returndef);
              if assigned(tprocdef(def).parast) then
                write_symtable_defs(list,tprocdef(def).parast);
              if assigned(tprocdef(def).localst) and
                 (tprocdef(def).localst.symtabletype=localsymtable) then
                write_symtable_defs(list,tprocdef(def).localst);
            end;
          arraydef :
            begin
              appenddef(list,tarraydef(def).rangedef);
              appenddef(list,tarraydef(def).elementdef);
            end;
          recorddef :
            trecorddef(def).symtable.symList.ForEachCall(@field_write_defs,list);
          enumdef :
            if assigned(tenumdef(def).basedef) then
              appenddef(list,tenumdef(def).basedef);
          objectdef :
            begin
              { make sure we don't write child classdefs before their parent }
              { classdefs, because this crashes gdb                          }
              anc:=tobjectdef(def);
              while assigned(anc.childof) do
                begin
                  anc:=anc.childof;
                  case anc.dbg_state of
                    dbg_state_writing:
                      { happens in case a field of a parent is of the (forward
                        defined) child type
                      }
                      begin
                        { We don't explicitly requeue it, but the fact that
                          a child type was used in a parent before the child
                          type was fully defined means that it was forward
                          declared, and will still be encountered later.
                          Setting the state to queued however allows us to
                          get the def number already without an IE
                        }
                        def.dbg_state:=dbg_state_queued;
                        break;
                      end;
                    else
                      ;
                  end;
                end;
              appenddef(list,vmtarraytype);
              if assigned(tobjectdef(def).ImplementedInterfaces) then
                for i:=0 to tobjectdef(def).ImplementedInterfaces.Count-1 do
                  appenddef(list,TImplementedInterface(tobjectdef(def).ImplementedInterfaces[i]).IntfDef);
              { first the parents }
              anc:=tobjectdef(def);
              while assigned(anc.childof) do
                begin
                  anc:=anc.childof;
                  { in case this is an object family declared in another unit
                    that was compiled without debug info, this ancestor may not
                    yet have a stabs number and not yet be added to defstowrite
                    -> take care of that now, while its dbg_state is still
                    dbg_state_unused in case the aforementioned things haven't
                    happened yet (afterwards it will become dbg_state_writing,
                    and then def_stab_number() won't do anything anymore because
                    it assumes it's already happened
                  }
                  def_stab_number(anc);
                  appenddef(list,anc);
                  if assigned(anc.ImplementedInterfaces) then
                    for i:=0 to anc.ImplementedInterfaces.Count-1 do
                      appenddef(list,TImplementedInterface(anc.ImplementedInterfaces[i]).IntfDef);
                end;
            end;
          else
            ;
        end;
      end;


    procedure TDebugInfoStabs.appendprocdef(list:TAsmList;def:tprocdef);
      var
        hs : ansistring;
        templist : TAsmList;
        prev_procdef : tprocdef;
      begin
        if not(def.in_currentunit) or
           { happens for init procdef of units without init section }
           not assigned(def.procstarttai) then
          exit;

        { mark as used so the local type defs also be written }
        def.dbg_state:=dbg_state_used;
        prev_procdef:=current_procdef;
        current_procdef:=def;

        templist:=gen_procdef_endsym_stabs(def);
        current_asmdata.asmlists[al_procedures].insertlistafter(def.procendtai,templist);

        { FUNC stabs }
        templist.free;
        templist:=gen_procdef_startsym_stabs(def);
        current_asmdata.asmlists[al_procedures].insertlistbefore(def.procstarttai,templist);

        { para types }
        if assigned(def.parast) then
          write_symtable_syms(templist,def.parast);
        { local type defs and vars should not be written
          inside the main proc stab }
        if assigned(def.localst) and
           (def.localst.symtabletype=localsymtable) then
          write_symtable_syms(templist,def.localst);

        if assigned(def.funcretsym) and
           (tabstractnormalvarsym(def.funcretsym).refs>0) then
          begin
            if tabstractnormalvarsym(def.funcretsym).localloc.loc=LOC_REFERENCE then
              begin
{ TODO: Need to add gdb support for ret in param register calling}
                if paramanager.ret_in_param(def.returndef,def) then
                  hs:='X*'
                else
                  hs:='X';
                templist.concat(Tai_stab.create(stabsdir,strpnew(
                   '"'+GetSymName(def.procsym)+':'+hs+def_stab_number(def.returndef)+'",'+
                   base_stabs_str(localvarsymref_stab,'0','0',getoffsetstr(tabstractnormalvarsym(def.funcretsym).localloc.reference)))));
                if (m_result in current_settings.modeswitches) then
                  templist.concat(Tai_stab.create(stabsdir,strpnew(
                     '"RESULT:'+hs+def_stab_number(def.returndef)+'",'+
                     base_stabs_str(localvarsymref_stab,'0','0',getoffsetstr(tabstractnormalvarsym(def.funcretsym).localloc.reference)))));
              end;
          end;


        current_asmdata.asmlists[al_procedures].insertlistbefore(def.procstarttai,templist);

        templist.free;
        current_procdef:=prev_procdef;
      end;


{****************************************************************************
                               TSym support
****************************************************************************}

    function TDebugInfoStabs.sym_var_value(const s:string;arg:pointer):string;
      var
        sym : tsym absolute arg;
      begin
        result:='';
        if s='name' then
          result:=GetSymName(sym)
        else if s='mangledname' then
          result:=ReplaceForbiddenAsmSymbolChars(sym.mangledname)
        else if s='ownername' then
          result:=GetSymTableName(sym.owner)
        else if s='line' then
          result:=tostr(sym.fileinfo.line)
        else
          internalerror(200401152);
      end;


    function TDebugInfoStabs.sym_stabstr_evaluate(sym:tsym;const s:string;const vars:array of string):ansistring;
      begin
        result:=string_evaluate(s,@sym_var_value,sym,vars);
      end;


    procedure TDebugInfoStabs.write_sym_stabstr(list:TAsmList;sym:tsym;const ss:ansistring);
      begin
        if ss='' then
          exit;
        { add to list }
        list.concat(Tai_stab.create_ansistr(stabsdir,ss));
      end;


    function TDebugInfoStabs.staticvarsym_mangled_name(sym: tstaticvarsym): string;
      begin
        result:=ReplaceForbiddenAsmSymbolChars(sym.mangledname);
      end;


    procedure TDebugInfoStabs.maybe_add_vmt_sym(list: TAsmList; def: tobjectdef);
      begin
        if (oo_has_vmt in def.objectoptions) and
           assigned(def.owner) and
           assigned(def.owner.name) then
          list.concat(Tai_stab.create_ansistr(stabsdir,ansistring('"vmt_')+GetSymTableName(def.owner)+tobjectdef(def).objname^+':S'+
                 def_stab_number(vmttype)+'",'+
                 base_stabs_str(globalvarsym_inited_stab,'0','0',ReplaceForbiddenAsmSymbolChars(tobjectdef(def).vmt_mangledname))));
      end;


    procedure TDebugInfoStabs.appendsym_fieldvar(list:TAsmList;sym:tfieldvarsym);
      var
        ss : ansistring;
      begin
        ss:='';
        if (sym.owner.symtabletype in [ObjectSymtable,recordsymtable]) and
           (sp_static in sym.symoptions) then
          ss:=sym_stabstr_evaluate(sym,'"${ownername}__${name}:S$1",'+base_stabs_str(globalvarsym_uninited_stab,'0','${line}','${mangledname}'),
              [def_stab_number(sym.vardef)]);
        write_sym_stabstr(list,sym,ss);
      end;


    procedure TDebugInfoStabs.appendsym_staticvar(list:TAsmList;sym:tstaticvarsym);
      var
        ss : ansistring;
        st : string;
        threadvaroffset : string;
        regidx : Tregisterindex;
        nsym : byte;
      begin
        { external symbols can't be resolved at link time, so we
          can't generate stabs for them }
        if vo_is_external in sym.varoptions then
          exit;
        ss:='';
        st:=def_stab_number(sym.vardef);
        case sym.localloc.loc of
          LOC_REGISTER,
          LOC_CREGISTER,
          LOC_MMREGISTER,
          LOC_CMMREGISTER,
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
              regidx:=findreg_by_number(sym.localloc.register);
              { "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi", "eip", "ps", "cs", "ss", "ds", "es", "fs", "gs", }
              { this is the register order for GDB}
              if regidx<>0 then
                ss:=sym_stabstr_evaluate(sym,'"${name}:r$1",'+base_stabs_str(regvar_stab,'0','${line}','$2'),[st,tostr(regstabs_table[regidx])]);
            end;
          else
            begin
              if (vo_is_thread_var in sym.varoptions) then
                threadvaroffset:='+'+tostr(sizeof(pint))
              else
                threadvaroffset:='';
              if (vo_is_typed_const in sym.varoptions) then
                if vo_is_public in sym.varoptions then
                  nsym:=globalvarsym_inited_stab
                else
                  nsym:=staticvarsym_inited_stab
              else if vo_is_public in sym.varoptions then
                nsym:=globalvarsym_uninited_stab
              else
                nsym:=staticvarsym_uninited_stab;
              { Here we used S instead of
                because with G GDB doesn't look at the address field
                but searches the same name or with a leading underscore
                but these names don't exist in pascal !}
              st:='S'+st;
              ss:=sym_stabstr_evaluate(sym,'"${name}:$1",'+base_stabs_str(nsym,'0','${line}','$2$3'),[st,staticvarsym_mangled_name(sym),threadvaroffset]);
            end;
        end;
        write_sym_stabstr(list,sym,ss);
      end;


    procedure TDebugInfoStabs.appendsym_localvar(list:TAsmList;sym:tlocalvarsym);
      var
        ss : ansistring;
        st : string;
        regidx : Tregisterindex;
      begin
        { There is no space allocated for not referenced locals }
        if (sym.owner.symtabletype=localsymtable) and (sym.refs=0) then
          exit;

        ss:='';
        st:=def_stab_number(sym.vardef);
        case sym.localloc.loc of
          LOC_REGISTER,
          LOC_CREGISTER,
          LOC_MMREGISTER,
          LOC_CMMREGISTER,
          LOC_FPUREGISTER,
          LOC_CFPUREGISTER :
            begin
              regidx:=findreg_by_number(sym.localloc.register);
              { "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi", "eip", "ps", "cs", "ss", "ds", "es", "fs", "gs", }
              { this is the register order for GDB}
              if regidx<>0 then
                ss:=sym_stabstr_evaluate(sym,'"${name}:r$1",'+base_stabs_str(regvar_stab,'0','${line}','$2'),[st,tostr(regstabs_table[regidx])]);
            end;
          LOC_REFERENCE :
            { offset to ebp => will not work if the framepointer is esp
              so some optimizing will make things harder to debug }
            ss:=sym_stabstr_evaluate(sym,'"${name}:$1",'+base_stabs_str(localvarsymref_stab,'0','${line}','$2'),[st,getoffsetstr(sym.localloc.reference)])
          else
            internalerror(2003091814);
        end;
        write_sym_stabstr(list,sym,ss);
      end;


    function TDebugInfoStabs.get_appendsym_paravar_reg(sym:tparavarsym;const typ,stabstr:string;reg: tregister): ansistring;
      var
        ltyp: string[1];
        regidx : Tregisterindex;
      begin
        result:='';
        if typ='p' then
          ltyp:='R'
        else
          ltyp:='a';
        regidx:=findreg_by_number(reg);
        { "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi", "eip", "ps", "cs", "ss", "ds", "es", "fs", "gs", }
        { this is the register order for GDB}
        if regidx<>0 then
          result:=sym_stabstr_evaluate(sym,'"${name}:$1",'+base_stabs_str(regvar_stab,'0','${line}','$2'),[ltyp+stabstr,tostr(longint(regstabs_table[regidx]))]);
      end;


    function TDebugInfoStabs.base_stabs_str(typ: longint; const other, desc, value: ansistring): ansistring;
      begin
        result:=base_stabs_str(tostr(typ),other,desc,value);
      end;


    function TDebugInfoStabs.base_stabs_str(const typ, other, desc, value: ansistring): ansistring;
      begin
        result:=typ+','+other+','+desc+','+value
      end;


    function TDebugInfoStabs.gen_procdef_startsym_stabs(def: tprocdef): TAsmList;
      var
        RType : Char;
        Obj,Info,
        mangledname: ansistring;
      begin
        result:=TAsmList.create;
        { "The stab representing a procedure is located immediately
          following the code of the procedure. This stab is in turn
          directly followed by a group of other stabs describing
          elements of the procedure. These other stabs describe the
          procedure's parameters, its block local variables, and its
          block structure." (stab docs)                               }
        { this is however incorrect in case "include source" statements }
        { appear in the block, in that case the procedure stab must     }
        { appear before this include stabs (and we generate such an     }
        { stabs for all functions) (JM)                                 }

        obj := GetSymName(def.procsym);
        info := '';
        if (po_global in def.procoptions) then
          RType := 'F'
        else
          RType := 'f';
        if assigned(def.owner) then
          begin
            if (def.owner.symtabletype in [ObjectSymtable,recordsymtable]) then
              obj := GetSymTableName(def.owner)+'__'+GetSymName(def.procsym);
            if not(cs_gdb_valgrind in current_settings.globalswitches) and
               (def.owner.symtabletype=localsymtable) and
               assigned(def.owner.defowner) and
               assigned(tprocdef(def.owner.defowner).procsym) then
              info := ','+GetSymName(def.procsym)+','+GetSymName(tprocdef(def.owner.defowner).procsym);
          end;
        mangledname:=ReplaceForbiddenAsmSymbolChars(def.mangledname);
        if target_info.system in systems_dotted_function_names then
          mangledname:='.'+mangledname;
        result.concat(Tai_stab.Create_ansistr(stabsdir,'"'+obj+':'+RType+def_stab_number(def.returndef)+info+'",'+
          base_stabs_str(procdef_stab,'0',tostr(def.fileinfo.line),mangledname)));
      end;


    function TDebugInfoStabs.gen_procdef_endsym_stabs(def: tprocdef): TAsmList;
      var
        ss, mangledname: ansistring;
        stabsendlabel: tasmlabel;
      begin
        result:=TAsmList.create;

        { end of procedure }
        current_asmdata.getlabel(stabsendlabel,alt_dbgtype);

        if dbgtype<>dbg_stabx then
          begin
            mangledname:=def.mangledname;
            if target_info.system in systems_dotted_function_names then
              mangledname:='.'+mangledname;
            // LBRAC
            if af_stabs_use_function_absolute_addresses in target_asm.flags then
              ss:=tostr(STABS_N_LBRAC)+',0,0,'+mangledname
            else
              ss:=tostr(STABS_N_LBRAC)+',0,0,0';
            result.concat(Tai_stab.Create_ansistr(stab_stabn,ss));

            // RBRAC
            ss:=tostr(STABS_N_RBRAC)+',0,0,'+stabsendlabel.name;
            if not(af_stabs_use_function_absolute_addresses in target_asm.flags) then
              ss:=ss+'-'+mangledname;
            result.concat(Tai_stab.Create_ansistr(stab_stabn,ss));

            { the stabsendlabel must come after all other stabs for this }
            { function                                                   }
            result.concat(tai_label.create(stabsendlabel));

            { Add a "size" stab as described in the last paragraph of 2.5 at  }
            { http://sourceware.org/gdb/current/onlinedocs/stabs_2.html#SEC12 }
            { This works at least on Darwin (and is needed on Darwin to get   }
            { correct smartlinking of stabs), but I don't know which binutils }
            { version is required on other platforms                          }
            { This stab must come after all other stabs for the procedure,    }
            { including the LBRAC/RBRAC ones                                  }
            if (target_info.system in systems_darwin) then
              result.concat(Tai_stab.create(stabsdir,
                strpnew('"",'+base_stabs_str(procdef_stab,'0','0',stabsendlabel.name+'-'+mangledname))));
          end;
      end;


    procedure TDebugInfoStabs.appendsym_paravar(list:TAsmList;sym:tparavarsym);
      var
        ss : ansistring;
        c  : string[1];
        st : string;
        regidx : Tregisterindex;
      begin
        ss:='';
        { set loc to LOC_REFERENCE to get somewhat usable debugging info for -Or }
        { while stabs aren't adapted for regvars yet                             }
        if (vo_is_self in sym.varoptions) then
          begin
            case sym.localloc.loc of
              LOC_REGISTER,
              LOC_CREGISTER:
                regidx:=findreg_by_number(sym.localloc.register);
              LOC_REFERENCE: ;
              else
                internalerror(2003091815);
            end;
            if (po_classmethod in tabstractprocdef(sym.owner.defowner).procoptions) or
               (po_staticmethod in tabstractprocdef(sym.owner.defowner).procoptions) then
              begin
                if (sym.localloc.loc=LOC_REFERENCE) then
                  ss:=sym_stabstr_evaluate(sym,'"pvmt:p$1",'+base_stabs_str(localvarsymref_stab,'0','0','$2'),
                    [def_stab_number(pvmttype),getoffsetstr(sym.localloc.reference)])
                else
                  begin
                    regidx:=findreg_by_number(sym.localloc.register);
                    ss:=sym_stabstr_evaluate(sym,'"pvmt:r$1",'+base_stabs_str(regvar_stab,'0','0','$2'),
                      [def_stab_number(pvmttype),tostr(regstabs_table[regidx])]);
                  end
                end
            else
              begin
                if not(is_class(tprocdef(sym.owner.defowner).struct)) then
                  c:='v'
                else
                  c:='p';
                if (sym.localloc.loc=LOC_REFERENCE) then
                  ss:=sym_stabstr_evaluate(sym,'"$$t:$1",'+base_stabs_str(localvarsymref_stab,'0','0','$2'),
                        [c+def_stab_number(tprocdef(sym.owner.defowner).struct),getoffsetstr(sym.localloc.reference)])
                else
                  begin
                    if (c='p') then
                      c:='R'
                    else
                      c:='a';
                    regidx:=findreg_by_number(sym.localloc.register);
                    ss:=sym_stabstr_evaluate(sym,'"$$t:$1",'+base_stabs_str(regvar_stab,'0','0','$2'),
                        [c+def_stab_number(tprocdef(sym.owner.defowner).struct),tostr(regstabs_table[regidx])]);
                  end
              end;
          end
        else
          begin
            st:=def_stab_number(sym.vardef);

            if paramanager.push_addr_param(sym.varspez,sym.vardef,tprocdef(sym.owner.defowner).proccalloption) and
               not(vo_has_local_copy in sym.varoptions) and
               not is_open_string(sym.vardef) then
              c:='v' { should be 'i' but 'i' doesn't work }
            else
              c:='p';
            case sym.localloc.loc of
              LOC_REGISTER,
              LOC_CREGISTER,
              LOC_MMREGISTER,
              LOC_CMMREGISTER,
              LOC_FPUREGISTER,
              LOC_CFPUREGISTER :
                begin
                  ss:=get_appendsym_paravar_reg(sym,c,st,sym.localloc.register);
                end;
              LOC_REFERENCE :
                begin
                  { When the *value* of a parameter (so not its address!) is
                    copied into a local variable, you have to generate two
                    stabs: one for the parmeter, and one for the local copy.
                    Not doing this breaks debugging under e.g. SPARC. Doc:
                    http://sourceware.org/gdb/current/onlinedocs/stabs_4.html#SEC26
                  }
                  if (target_dbg.id<>dbg_stabx) and
                     (c='p') and
                     not is_open_string(sym.vardef) and
                     ((sym.paraloc[calleeside].location^.loc<>sym.localloc.loc) or
                      ((sym.localloc.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and
                       ((sym.paraloc[calleeside].location^.reference.index<>sym.localloc.reference.base) or
                        (sym.paraloc[calleeside].location^.reference.offset<>sym.localloc.reference.offset))) or
                      ((sym.localloc.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_MMREGISTER,LOC_CMMREGISTER,LOC_FPUREGISTER,LOC_CFPUREGISTER]) and
                       (sym.localloc.register<>sym.paraloc[calleeside].location^.register))) then
                    begin
                      if not(sym.paraloc[calleeside].location^.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
                        ss:=get_appendsym_paravar_reg(sym,c,st,sym.paraloc[calleeside].location^.register)
                      else
                        ss:=sym_stabstr_evaluate(sym,'"${name}:$1",'+base_stabs_str(localvarsymref_stab,'0','${line}','$2'),
                              [c+st,getparaoffsetstr(sym.paraloc[calleeside].location^.reference)]);
                      write_sym_stabstr(list,sym,ss);
                      { second stab has no parameter specifier }
                      c:='';
                    end;
                  { offset to ebp => will not work if the framepointer is esp
                    so some optimizing will make things harder to debug }
                  ss:=sym_stabstr_evaluate(sym,'"${name}:$1",'+base_stabs_str(paravarsymref_stab,'0','${line}','$2'),[c+st,getoffsetstr(sym.localloc.reference)])
                end;
              else
                internalerror(2003091814);
            end;
          end;
        write_sym_stabstr(list,sym,ss);
      end;


    function stabx_quote_const(const s: string): string;
      var
        i:byte;
      begin
        stabx_quote_const:='';
        for i:=1 to length(s) do
          begin
            case s[i] of
              #10:
                stabx_quote_const:=stabx_quote_const+'\n';
              #13:
                stabx_quote_const:=stabx_quote_const+'\r';
              { stabx strings cannot deal with embedded quotes }
              '"':
                stabx_quote_const:=stabx_quote_const+' ';
              else
                stabx_quote_const:=stabx_quote_const+s[i];
            end;
          end;
      end;


    procedure TDebugInfoStabs.appendsym_const(list:TAsmList;sym:tconstsym);
      var
        st : string;
        ss : ansistring;
      begin
        ss:='';
        { Don't write info for default parameter values, the N_Func breaks
          the N_Func for the function itself.
          Valgrind does not support constants }
        if (sym.owner.symtabletype=parasymtable) or
           (cs_gdb_valgrind in current_settings.globalswitches) then
          exit;
        case sym.consttyp of
          conststring:
            begin
              if sym.value.len<200 then
                if target_dbg.id=dbg_stabs then
                  st:='s'''+backspace_quote(octal_quote(strpas(pchar(sym.value.valueptr)),[#0..#9,#11,#12,#14..#31,'''']),['"','\',#10,#13])+''''
                else
                  st:='s'''+stabx_quote_const(octal_quote(strpas(pchar(sym.value.valueptr)),[#0..#9,#11,#12,#14..#31,'''']))+''''
              else
                st:='<constant string too long>';
            end;
          constord:
            st:='i'+tostr(sym.value.valueord);
          constpointer:
            st:='i'+tostr(sym.value.valueordptr);
          constreal:
            begin
              system.str(pbestreal(sym.value.valueptr)^,st);
              st := 'r'+st;
            end;
          else
            begin
              { if we don't know just put zero !! }
              st:='i0';
            end;
        end;
        ss:=sym_stabstr_evaluate(sym,'"${name}:c=$1;",'+base_stabs_str(constsym_stab,'0','${line}','0'),[st]);
        write_sym_stabstr(list,sym,ss);
      end;


    procedure TDebugInfoStabs.appendsym_type(list:TAsmList;sym:ttypesym);
      var
        ss : ansistring;
        stabchar : string[2];
      begin
        ss:='';
        if not assigned(sym.typedef) then
          internalerror(200509262);
        if use_tag_prefix(sym.typedef) then
          stabchar:=tagtypeprefix
        else
          stabchar:='t';
        ss:=sym_stabstr_evaluate(sym,'"${name}:$1$2",'+base_stabs_str(typesym_stab,'0','${line}','0'),[stabchar,def_stab_number(sym.typedef)]);
        write_sym_stabstr(list,sym,ss);
      end;


    procedure TDebugInfoStabs.appendsym_label(list:TAsmList;sym:tlabelsym);
      var
        ss : ansistring;
      begin
        ss:=sym_stabstr_evaluate(sym,'"${name}",'+base_stabs_str(localvarsymref_stab,'0','${line}','0'),[]);
        write_sym_stabstr(list,sym,ss);
      end;


{****************************************************************************
                             Proc/Module support
****************************************************************************}

    procedure TDebugInfoStabs.inserttypeinfo;
      var
        stabsvarlist,
        stabstypelist : TAsmList;
        storefilepos  : tfileposinfo;
        i  : longint;
        vardatatype : ttypesym;
      begin
        storefilepos:=current_filepos;
        current_filepos:=current_module.mainfilepos;

        global_stab_number:=0;
        defnumberlist:=TFPObjectlist.create(false);
        deftowritelist:=TFPObjectlist.create(false);
        stabsvarlist:=TAsmList.create;
        stabstypelist:=TAsmList.create;

        vardatatype:=try_search_system_type('TVARDATA');
        if assigned(vardatatype) then
          vardatadef:=trecorddef(vardatatype.typedef);

        { include symbol that will be referenced from the main to be sure to
          include this debuginfo .o file }
        include(current_module.moduleflags,mf_has_stabs_debuginfo);
        if not(target_info.system in systems_darwin) then
          begin
            new_section(current_asmdata.asmlists[al_stabs],sec_data,GetSymTableName(current_module.localsymtable),sizeof(pint));
            current_asmdata.asmlists[al_stabs].concat(tai_symbol.Createname_global(make_mangledname('DEBUGINFO',current_module.localsymtable,''),AT_METADATA,0,voidpointertype));
          end
        else
          new_section(current_asmdata.asmlists[al_stabs],sec_code,GetSymTableName(current_module.localsymtable),sizeof(pint));

        { write all global/local variables. This will flag all required tdefs  }
        if assigned(current_module.globalsymtable) then
          write_symtable_syms(stabsvarlist,current_module.globalsymtable);
        if assigned(current_module.localsymtable) then
          write_symtable_syms(stabsvarlist,current_module.localsymtable);

        { write all procedures and methods. This will flag all required tdefs }
        if assigned(current_module.globalsymtable) then
          write_symtable_procdefs(stabsvarlist,current_module.globalsymtable);
        if assigned(current_module.localsymtable) then
          write_symtable_procdefs(stabsvarlist,current_module.localsymtable);

        { reset unit type info flag }
        reset_unit_type_info;

        { write used types from the used units }
        write_used_unit_type_info(stabstypelist,current_module);
        { last write the types from this unit }
        if assigned(current_module.globalsymtable) then
          write_symtable_defs(stabstypelist,current_module.globalsymtable);
        if assigned(current_module.localsymtable) then
          write_symtable_defs(stabstypelist,current_module.localsymtable);

        write_remaining_defs_to_write(stabstypelist);

        current_asmdata.asmlists[al_stabs].concatlist(stabstypelist);
        current_asmdata.asmlists[al_stabs].concatlist(stabsvarlist);

        { reset stab numbers }
        for i:=0 to defnumberlist.count-1 do
          begin
            if assigned(defnumberlist[i]) then
              begin
                tdef(defnumberlist[i]).stab_number:=0;
                tdef(defnumberlist[i]).dbg_state:=dbg_state_unused;
              end;
          end;

        defnumberlist.free;
        defnumberlist:=nil;
        deftowritelist.free;
        deftowritelist:=nil;

        stabsvarlist.free;
        stabstypelist.free;
        current_filepos:=storefilepos;
      end;


    procedure TDebugInfoStabs.insertlineinfo(list: TAsmList);
      var
        currfileinfo,
        lastfileinfo : tfileposinfo;
        currfuncname : pshortstring;
        currsectype  : TAsmSectiontype;
        hlabel       : tasmlabel;
        hp : tai;
        infile : tinputfile;
      begin
        FillChar(lastfileinfo,sizeof(lastfileinfo),0);
        currfuncname:=nil;
        currsectype:=sec_code;
        hp:=Tai(list.first);
        while assigned(hp) do
          begin
            case hp.typ of
              ait_section :
                currsectype:=tai_section(hp).sectype;
              ait_function_name :
                currfuncname:=tai_function_name(hp).funcname;
              ait_force_line :
                lastfileinfo.line:=-1;
              else
                ;
            end;

            if (currsectype=sec_code) and
               (hp.typ=ait_instruction) then
              begin
                currfileinfo:=tailineinfo(hp).fileinfo;
                { file changed ? (must be before line info) }
                if (currfileinfo.fileindex<>0) and
                   ((lastfileinfo.fileindex<>currfileinfo.fileindex) or
                    (lastfileinfo.moduleindex<>currfileinfo.moduleindex)) then
                  begin
                    infile:=get_module(currfileinfo.moduleindex).sourcefiles.get_file(currfileinfo.fileindex);
                    if assigned(infile) then
                      begin
                        current_asmdata.getlabel(hlabel,alt_dbgfile);
                        { emit stabs }
                        if not(ds_stabs_abs_include_files in current_settings.debugswitches) or
                           path_absolute(infile.path) then
                          list.insertbefore(Tai_stab.Create_str(stabsdir,'"'+BsToSlash(FixPath(infile.path,false))+FixFileName(infile.name)+'",'+tostr(stabs_n_includefile)+
                                            ',0,0,'+hlabel.name),hp)
                        else
                          list.insertbefore(Tai_stab.Create_str(stabsdir,'"'+BsToSlash(FixPath(getcurrentdir,false)+FixPath(infile.path,false))+FixFileName(infile.name)+'",'+tostr(stabs_n_includefile)+
                                            ',0,0,'+hlabel.name),hp);
                        list.insertbefore(tai_label.create(hlabel),hp);
                        { force new line info }
                        lastfileinfo.line:=-1;
                      end;
                  end;

                { line changed ? }
                if (currfileinfo.line>lastfileinfo.line) and (currfileinfo.line<>0) then
                  begin
                     if assigned(currfuncname) and
                        not(af_stabs_use_function_absolute_addresses in target_asm.flags) then
                      begin
                        current_asmdata.getlabel(hlabel,alt_dbgline);
                        list.insertbefore(Tai_stab.Create_str(stab_stabn,tostr(stabs_n_textline)+',0,'+tostr(currfileinfo.line)+','+
                                          hlabel.name+' - '+{$IFDEF POWERPC64}'.'+{$ENDIF POWERPC64}currfuncname^),hp);
                        list.insertbefore(tai_label.create(hlabel),hp);
                      end
                     else
                      list.insertbefore(Tai_stab.Create_str(stab_stabd,tostr(stabs_n_textline)+',0,'+tostr(currfileinfo.line)),hp);
                  end;
                lastfileinfo:=currfileinfo;
              end;

            hp:=tai(hp.next);
          end;
      end;


    procedure TDebugInfoStabs.insertmoduleinfo;
      var
        hlabel : tasmlabel;
        infile : tinputfile;
      begin
        { emit main source n_sourcefile for start of module }
        current_asmdata.getlabel(hlabel,alt_dbgfile);
        infile:=current_module.sourcefiles.get_file(1);
        new_section(current_asmdata.asmlists[al_start],sec_code,make_mangledname('DEBUGSTART',current_module.localsymtable,''),sizeof(pint),secorder_begin);
        if not(target_info.system in systems_darwin) then
          current_asmdata.asmlists[al_start].concat(tai_symbol.Createname_global(make_mangledname('DEBUGSTART',current_module.localsymtable,''),AT_METADATA,0,voidpointertype));
{$ifdef MIPS}
       { at least mipsel needs an explicit '.set nomips16' before any reference to
         procedure/function, see bug report 32138 }
        current_asmdata.asmlists[al_start].concat(Taicpu.op_none(A_P_SET_NOMIPS16));
{$endif MIPS}
        current_asmdata.asmlists[al_start].concat(Tai_stab.Create_str(stabsdir,'"'+BsToSlash(FixPath(getcurrentdir,false))+'",'+
          base_stabs_str(stabs_n_sourcefile,'0','0',hlabel.name)));
        current_asmdata.asmlists[al_start].concat(Tai_stab.Create_str(stabsdir,'"'+BsToSlash(FixPath(infile.path,false))+FixFileName(infile.name)+'",'+
          base_stabs_str(stabs_n_sourcefile,'0','0',hlabel.name)));
        current_asmdata.asmlists[al_start].concat(tai_label.create(hlabel));
        { for darwin, you need a "module marker" too to work around      }
        { either some assembler or gdb bug (radar 4386531 according to a }
        { comment in dbxout.c of Apple's gcc)                            }
        if (target_info.system in systems_darwin) then
          current_asmdata.asmlists[al_end].concat(Tai_stab.Create_str(stabsdir,'"",'+base_stabs_str(STABS_N_OSO,'0','0','0')));
        { emit empty n_sourcefile for end of module }
        current_asmdata.getlabel(hlabel,alt_dbgfile);
        new_section(current_asmdata.asmlists[al_end],sec_code,make_mangledname('DEBUGEND',current_module.localsymtable,''),sizeof(pint),secorder_end);
        if not(target_info.system in systems_darwin) then
          current_asmdata.asmlists[al_end].concat(tai_symbol.Createname_global(make_mangledname('DEBUGEND',current_module.localsymtable,''),AT_METADATA,0,voidpointertype));
        current_asmdata.asmlists[al_end].concat(Tai_stab.Create_str(stabsdir,'"",'+base_stabs_str(stabs_n_sourcefile,'0','0',hlabel.name)));
        current_asmdata.asmlists[al_end].concat(tai_label.create(hlabel));
      end;


        procedure TDebugInfoStabs.referencesections(list: TAsmList);
      var
        hp : tmodule;
        dbgtable : tai_symbol;
      begin
        { Reference all DEBUGINFO sections from the main .fpc section }
        if (target_info.system in ([system_powerpc_macos]+systems_darwin)) then
          exit;
        new_section(list,sec_fpc,'links',0);
        { make sure the debuginfo doesn't get stripped out }
        if (target_info.system in systems_darwin) then
          begin
            dbgtable:=tai_symbol.createname('DEBUGINFOTABLE',AT_METADATA,0,voidpointertype);
            list.concat(tai_directive.create(asd_no_dead_strip,dbgtable.sym.name));
            list.concat(dbgtable);
          end;
        { include reference to all debuginfo sections of used units }
        hp:=tmodule(loaded_units.first);
        while assigned(hp) do
          begin
            If (mf_has_stabs_debuginfo in hp.moduleflags) and not assigned(hp.package) then
              begin
                list.concat(Tai_const.Createname(make_mangledname('DEBUGINFO',hp.localsymtable,''),0));
                list.concat(Tai_const.Createname(make_mangledname('DEBUGSTART',hp.localsymtable,''),0));
                list.concat(Tai_const.Createname(make_mangledname('DEBUGEND',hp.localsymtable,''),0));
              end;
            hp:=tmodule(hp.next);
          end;
      end;

    constructor TDebugInfoStabs.Create;
      begin
        inherited Create;
        dbgtype:=dbg_stabs;
        stabsdir:=stab_stabs;

        def_stab:=STABS_N_LSYM;
        regvar_stab:=STABS_N_RSYM;
        procdef_stab:=STABS_N_Function;
        constsym_stab:=STABS_N_Function;
        typesym_stab:=STABS_N_LSYM;
        globalvarsym_uninited_stab:=STABS_N_STSYM;
        globalvarsym_inited_stab:=STABS_N_LCSYM;
        staticvarsym_uninited_stab:=STABS_N_STSYM;
        staticvarsym_inited_stab:=STABS_N_LCSYM;
        localvarsymref_stab:=STABS_N_TSYM;
        paravarsymref_stab:=STABS_N_TSYM;
        tagtypeprefix:='Tt';

        vardatadef:=nil;
      end;

    const
      dbg_stabs_info : tdbginfo =
         (
           id     : dbg_stabs;
           idtxt  : 'STABS';
         );

initialization
  RegisterDebugInfo(dbg_stabs_info,TDebugInfoStabs);
end.
