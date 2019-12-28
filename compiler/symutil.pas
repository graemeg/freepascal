{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit provides some help routines for symbol handling

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
unit symutil;

{$i fpcdefs.inc}

interface

    uses
       symconst,symbase,symtype,symsym;

    function is_funcret_sym(p:TSymEntry):boolean;

    function equal_constsym(sym1,sym2:tconstsym; nanequal: boolean):boolean;

    function get_first_proc_str(Options: TProcOptions): ShortString;

    procedure maybe_guarantee_record_typesym(var def: tdef; st: tsymtable);

    function is_normal_fieldvarsym(sym: tsym): boolean; inline;


implementation

    uses
       systems,
       globtype,cpuinfo,constexp,verbose,
       widestr,
       symdef;


    function is_funcret_sym(p:TSymEntry):boolean;
      begin
        is_funcret_sym:=(p.typ in [absolutevarsym,localvarsym,paravarsym]) and
                        (vo_is_funcret in tabstractvarsym(p).varoptions);
      end;


    function equal_constsym(sym1,sym2:tconstsym; nanequal: boolean):boolean;
      var
        p1,p2,pend : pchar;
      begin
        equal_constsym:=false;
        if sym1.consttyp<>sym2.consttyp then
         exit;
        case sym1.consttyp of
           constord :
             equal_constsym:=(sym1.value.valueord=sym2.value.valueord);
           constpointer :
             equal_constsym:=(sym1.value.valueordptr=sym2.value.valueordptr);
           conststring,constresourcestring :
             begin
               if sym1.value.len=sym2.value.len then
                begin
                  p1:=pchar(sym1.value.valueptr);
                  p2:=pchar(sym2.value.valueptr);
                  pend:=p1+sym1.value.len;
                  while (p1<pend) do
                   begin
                     if p1^<>p2^ then
                      break;
                     inc(p1);
                     inc(p2);
                   end;
                  if (p1=pend) then
                   equal_constsym:=true;
                end;
             end;
           constwstring :
             begin
               if (sym1.value.len=sym2.value.len) and
                  (comparewidestrings(sym1.value.valueptr,sym2.value.valueptr)=0) then
                 equal_constsym:=true;
             end;
           constreal :
             if nanequal then
               equal_constsym:=CompareByte(pbestreal(sym1.value.valueptr)^,pbestreal(sym2.value.valueptr)^,sizeof(pbestreal^))=0
             else
               equal_constsym:=pbestreal(sym1.value.valueptr)^=pbestreal(sym2.value.valueptr)^;
           constset :
             equal_constsym:=(pnormalset(sym1.value.valueptr)^=pnormalset(sym2.value.valueptr)^);
           constnil :
             equal_constsym:=true;
           else
             ;
        end;
      end;


    { get_first_proc_str - returns the token string of the first option that
      appears in the list }
    function get_first_proc_str(Options: TProcOptions): ShortString;
      var
        X: TProcOption;
      begin
        if Options = [] then
          InternalError(2018051700);

        get_first_proc_str := '';

        for X in Options do
          begin
            get_first_proc_str := ProcOptionKeywords[X];
            Exit;
          end;
      end;


    procedure maybe_guarantee_record_typesym(var def: tdef; st: tsymtable);
      var
        ts: ttypesym;
      begin
        { create a dummy typesym for the JVM target, because the record
          has to be wrapped by a class }
        if (target_info.system in systems_jvm) and
           (def.typ=recorddef) and
           not assigned(def.typesym) then
          begin
            ts:=ctypesym.create(trecorddef(def).symtable.realname^,def);
            st.insert(ts);
            ts.visibility:=vis_strictprivate;
            { this typesym can't be used by any Pascal code, so make sure we don't
              print a hint about it being unused }
            include(ts.symoptions,sp_internal);
          end;
      end;


    function is_normal_fieldvarsym(sym: tsym): boolean; inline;
      begin
        result:=
           (sym.typ=fieldvarsym) and
           not(sp_static in sym.symoptions);
      end;


end.

