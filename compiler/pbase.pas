{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Contains some helper routines for the parser

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
unit pbase;

{$i fpcdefs.inc}

interface

    uses
       cutils,cclasses,
       tokens,globtype,
       symconst,symbase,symtype,symdef,symsym,symtable
       ;

    const
       { tokens that end a block or statement. And don't require
         a ; on the statement before }
       endtokens = [_SEMICOLON,_END,_ELSE,_UNTIL,_EXCEPT,_FINALLY];

       { true, if we are after an assignement }
       afterassignment : boolean = false;

       { true, if we are parsing arguments }
       in_args : boolean = false;

       { true, if we are parsing arguments allowing named parameters }
       named_args_allowed : boolean = false;

       { true, if we got an @ to get the address }
       got_addrn  : boolean = false;

       { special for handling procedure vars }
       getprocvardef : tprocvardef = nil;

    var
       { for operators }
       optoken : ttoken;

       { true, if only routine headers should be parsed }
       parse_only : boolean;

       { true, if we found a name for a named arg }
       found_arg_name : boolean;

       { true, if we are parsing generic declaration }
       parse_generic : boolean;

    procedure identifier_not_found(const s:string);
    procedure identifier_not_found(const s:string;const filepos:tfileposinfo);

{    function tokenstring(i : ttoken):string;}

    { consumes token i, if the current token is unequal i }
    { a syntax error is written                           }
    procedure consume(i : ttoken);

    {Tries to consume the token i, and returns true if it was consumed:
     if token=i.}
    function try_to_consume(i:Ttoken):boolean;

    { consumes all tokens til atoken (for error recovering }
    procedure consume_all_until(atoken : ttoken);

    { consumes tokens while they are semicolons }
    procedure consume_emptystats;

    { reads a list of identifiers into a string list }
    { consume a symbol, if not found give an error and
      and return an errorsym }
    function consume_sym(var srsym:tsym;var srsymtable:TSymtable):boolean;
    function consume_sym_orgid(var srsym:tsym;var srsymtable:TSymtable;var s : string):boolean;

<<<<<<< HEAD
<<<<<<< HEAD
    function try_consume_unitsym(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume:ttoken;consume_id,allow_specialize:boolean;out is_specialize:boolean):boolean;
    function try_consume_unitsym_no_specialize(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume:ttoken;consume_id:boolean):boolean;
=======
    function try_consume_unitsym(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume : ttoken):boolean;
>>>>>>> graemeg/fixes_2_2
=======
    function try_consume_unitsym(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume : ttoken):boolean;
>>>>>>> origin/fixes_2_2

    function try_consume_hintdirective(var symopt:tsymoptions; var deprecatedmsg:pshortstring):boolean;

    { just for an accurate position of the end of a procedure (PM) }
    var
       last_endtoken_filepos: tfileposinfo;


implementation

    uses
<<<<<<< HEAD
<<<<<<< HEAD
       globals,htypechk,scanner,systems,verbose,fmodule;
=======
       globtype,htypechk,scanner,systems,verbose,fmodule;
>>>>>>> graemeg/fixes_2_2
=======
       globtype,htypechk,scanner,systems,verbose,fmodule;
>>>>>>> origin/fixes_2_2

{****************************************************************************
                               Token Parsing
****************************************************************************}

     procedure identifier_not_found(const s:string);
       begin
         Message1(sym_e_id_not_found,s);
         { show a fatal that you need -S2 or -Sd, but only
           if we just parsed the a token that has m_class }
         if not(m_class in current_settings.modeswitches) and
            (Upper(s)=pattern) and
            (m_class in tokeninfo^[idtoken].keyword) then
           Message(parser_f_need_objfpc_or_delphi_mode);
       end;


<<<<<<< HEAD
<<<<<<< HEAD
     procedure identifier_not_found(const s:string;const filepos:tfileposinfo);
       begin
         MessagePos1(filepos,sym_e_id_not_found,s);
         { show a fatal that you need -S2 or -Sd, but only
           if we just parsed the a token that has m_class }
         if not(m_class in current_settings.modeswitches) and
            (Upper(s)=pattern) and
            (m_class in tokeninfo^[idtoken].keyword) then
           MessagePos(filepos,parser_f_need_objfpc_or_delphi_mode);
       end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { consumes token i, write error if token is different }
    procedure consume(i : ttoken);
      begin
        if (token<>i) and (idtoken<>i) then
          if token=_id then
            Message2(scan_f_syn_expected,tokeninfo^[i].str,'identifier '+pattern)
          else
            Message2(scan_f_syn_expected,tokeninfo^[i].str,tokeninfo^[token].str)
        else
          begin
            if token=_END then
              last_endtoken_filepos:=current_tokenpos;
            current_scanner.readtoken(true);
          end;
      end;


    function try_to_consume(i:Ttoken):boolean;
      begin
        try_to_consume:=false;
        if (token=i) or (idtoken=i) then
         begin
           try_to_consume:=true;
           if token=_END then
            last_endtoken_filepos:=current_tokenpos;
           current_scanner.readtoken(true);
         end;
      end;


    procedure consume_all_until(atoken : ttoken);
      begin
         while (token<>atoken) and (idtoken<>atoken) do
          begin
            Consume(token);
            if token=_EOF then
             begin
               Consume(atoken);
               Message(scan_f_end_of_file);
               exit;
             end;
          end;
      end;


    procedure consume_emptystats;
      begin
         repeat
         until not try_to_consume(_SEMICOLON);
      end;


    { check if a symbol contains the hint directive, and if so gives out a hint
      if required.

      If this code is changed, it's likly that consume_sym_orgid and factor_read_id
      must be changed as well (FK)
    }
    function consume_sym(var srsym:tsym;var srsymtable:TSymtable):boolean;
      var
        t : ttoken;
      begin
        { first check for identifier }
        if token<>_ID then
          begin
            consume(_ID);
            srsym:=generrorsym;
            srsymtable:=nil;
            result:=false;
            exit;
          end;
        searchsym(pattern,srsym,srsymtable);
        { handle unit specification like System.Writeln }
<<<<<<< HEAD
<<<<<<< HEAD
        try_consume_unitsym_no_specialize(srsym,srsymtable,t,true);
=======
        try_consume_unitsym(srsym,srsymtable,t);
>>>>>>> graemeg/fixes_2_2
=======
        try_consume_unitsym(srsym,srsymtable,t);
>>>>>>> origin/fixes_2_2
        { if nothing found give error and return errorsym }
        if assigned(srsym) then
          check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg)
        else
          begin
            identifier_not_found(orgpattern);
            srsym:=generrorsym;
            srsymtable:=nil;
          end;
        consume(t);
        result:=assigned(srsym);
      end;


    { check if a symbol contains the hint directive, and if so gives out a hint
      if required and returns the id with it's original casing
    }
    function consume_sym_orgid(var srsym:tsym;var srsymtable:TSymtable;var s : string):boolean;
      var
        t : ttoken;
      begin
        { first check for identifier }
        if token<>_ID then
          begin
            consume(_ID);
            srsym:=generrorsym;
            srsymtable:=nil;
            result:=false;
            exit;
          end;
        searchsym(pattern,srsym,srsymtable);
        { handle unit specification like System.Writeln }
<<<<<<< HEAD
<<<<<<< HEAD
        try_consume_unitsym_no_specialize(srsym,srsymtable,t,true);
=======
        try_consume_unitsym(srsym,srsymtable,t);
>>>>>>> graemeg/fixes_2_2
=======
        try_consume_unitsym(srsym,srsymtable,t);
>>>>>>> origin/fixes_2_2
        { if nothing found give error and return errorsym }
        if assigned(srsym) then
          check_hints(srsym,srsym.symoptions,srsym.deprecatedmsg)
        else
          begin
            identifier_not_found(orgpattern);
            srsym:=generrorsym;
            srsymtable:=nil;
          end;
        s:=orgpattern;
        consume(t);
        result:=assigned(srsym);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function try_consume_unitsym(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume:ttoken;consume_id,allow_specialize:boolean;out is_specialize:boolean):boolean;
      var
        hmodule: tmodule;
        ns:ansistring;
        nssym:tsym;
=======
    function try_consume_unitsym(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume : ttoken):boolean;
      var
        hmodule: tmodule;
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
        result:=false;
        tokentoconsume:=_ID;
        is_specialize:=false;

        if assigned(srsym) and (srsym.typ in [unitsym,namespacesym]) then
=======
    function try_consume_unitsym(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume : ttoken):boolean;
      begin
        result:=false;
        tokentoconsume:=_ID;
=======
    function try_consume_unitsym(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume : ttoken):boolean;
      begin
        result:=false;
        tokentoconsume:=_ID;
>>>>>>> origin/fixes_2_2
        if assigned(srsym) and
           (srsym.typ=unitsym) then
>>>>>>> graemeg/fixes_2_2
          begin
            if not(srsym.owner.symtabletype in [staticsymtable,globalsymtable]) then
              internalerror(200501154);
            { only allow unit.symbol access if the name was
              found in the current module
              we can use iscurrentunit because generic specializations does not
              change current_unit variable }
            hmodule:=find_module_from_symtable(srsym.Owner);
            if not Assigned(hmodule) then
              internalerror(201001120);
            if hmodule.unit_index=current_filepos.moduleindex then
              begin
                if consume_id then
                  consume(_ID);
                consume(_POINT);
<<<<<<< HEAD
<<<<<<< HEAD
                if srsym.typ=namespacesym then
                  begin
                    ns:=srsym.name;
                    nssym:=srsym;
                    while assigned(srsym) and (srsym.typ=namespacesym) do
                      begin
                        { we have a namespace. the next identifier should be either a namespace or a unit }
                        searchsym_in_module(hmodule,ns+'.'+pattern,srsym,srsymtable);
                        if assigned(srsym) and (srsym.typ in [namespacesym,unitsym]) then
                          begin
                            ns:=ns+'.'+pattern;
                            nssym:=srsym;
                            consume(_ID);
                            consume(_POINT);
                          end;
                      end;
                    { check if there is a hidden unit with this pattern in the namespace }
                    if not assigned(srsym) and
                       assigned(nssym) and (nssym.typ=namespacesym) and assigned(tnamespacesym(nssym).unitsym) then
                      srsym:=tnamespacesym(nssym).unitsym;
                    if assigned(srsym) and (srsym.typ<>unitsym) then
                      internalerror(201108260);
                    if not assigned(srsym) then
                      begin
                        result:=true;
                        srsymtable:=nil;
                        exit;
                      end;
                  end;
                case token of
                  _ID:
                    { system.char? (char=widechar comes from the implicit
                      uuchar unit -> override) }
                    if (pattern='CHAR') and
                       (tmodule(tunitsym(srsym).module).globalsymtable=systemunit) then
                      begin
                        if m_default_unicodestring in current_settings.modeswitches then
                          searchsym_in_module(tunitsym(srsym).module,'WIDECHAR',srsym,srsymtable)
                        else
                          searchsym_in_module(tunitsym(srsym).module,'ANSICHAR',srsym,srsymtable)
                      end
                    else
                      if allow_specialize and (idtoken=_SPECIALIZE) then
                        begin
                          consume(_ID);
                          is_specialize:=true;
                          if token=_ID then
                            searchsym_in_module(tunitsym(srsym).module,pattern,srsym,srsymtable);
                        end
                      else
                        searchsym_in_module(tunitsym(srsym).module,pattern,srsym,srsymtable);
=======
                case token of
                  _ID:
                     searchsym_in_module(tunitsym(srsym).module,pattern,srsym,srsymtable);
>>>>>>> graemeg/fixes_2_2
=======
                case token of
                  _ID:
                     searchsym_in_module(tunitsym(srsym).module,pattern,srsym,srsymtable);
>>>>>>> origin/fixes_2_2
                  _STRING:
                    begin
                      { system.string? }
                      if tmodule(tunitsym(srsym).module).globalsymtable=systemunit then
                        begin
<<<<<<< HEAD
<<<<<<< HEAD
                          if cs_refcountedstrings in current_settings.localswitches then
                            begin
                              if m_default_unicodestring in current_settings.modeswitches then
                                searchsym_in_module(tunitsym(srsym).module,'UNICODESTRING',srsym,srsymtable)
                              else
                                searchsym_in_module(tunitsym(srsym).module,'ANSISTRING',srsym,srsymtable)
                            end
=======
                          if cs_ansistrings in current_settings.localswitches then
                            searchsym_in_module(tunitsym(srsym).module,'ANSISTRING',srsym,srsymtable)
>>>>>>> graemeg/fixes_2_2
=======
                          if cs_ansistrings in current_settings.localswitches then
                            searchsym_in_module(tunitsym(srsym).module,'ANSISTRING',srsym,srsymtable)
>>>>>>> origin/fixes_2_2
                          else
                            searchsym_in_module(tunitsym(srsym).module,'SHORTSTRING',srsym,srsymtable);
                          tokentoconsume:=_STRING;
                        end;
                    end
                  end;
              end
            else
              begin
                srsym:=nil;
                srsymtable:=nil;
              end;
            result:=true;
          end;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function try_consume_unitsym_no_specialize(var srsym:tsym;var srsymtable:TSymtable;var tokentoconsume:ttoken;consume_id:boolean):boolean;
      var
        dummy: Boolean;
      begin
        result:=try_consume_unitsym(srsym,srsymtable,tokentoconsume,consume_id,false,dummy);
      end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function try_consume_hintdirective(var symopt:tsymoptions; var deprecatedmsg:pshortstring):boolean;
      var
        last_is_deprecated:boolean;
      begin
        try_consume_hintdirective:=false;
        if not(m_hintdirective in current_settings.modeswitches) then
          exit;
        repeat
          last_is_deprecated:=false;
          case idtoken of
            _LIBRARY:
              begin
                if sp_hint_library in symopt then
                  Message1(parser_e_dir_not_allowed,arraytokeninfo[idtoken].str)
                else
                  include(symopt,sp_hint_library);
                try_consume_hintdirective:=true;
              end;
            _DEPRECATED:
              begin
                if sp_hint_deprecated in symopt then
                  Message1(parser_e_dir_not_allowed,arraytokeninfo[idtoken].str)
                else
                  include(symopt,sp_hint_deprecated);
                try_consume_hintdirective:=true;
                last_is_deprecated:=true;
              end;
            _EXPERIMENTAL:
              begin
                if sp_hint_experimental in symopt then
                  Message1(parser_e_dir_not_allowed,arraytokeninfo[idtoken].str)
                else
                  include(symopt,sp_hint_experimental);
                try_consume_hintdirective:=true;
              end;
            _PLATFORM:
              begin
                if sp_hint_platform in symopt then
                  Message1(parser_e_dir_not_allowed,arraytokeninfo[idtoken].str)
                else
                  include(symopt,sp_hint_platform);
                try_consume_hintdirective:=true;
              end;
            _UNIMPLEMENTED:
              begin
                if sp_hint_unimplemented in symopt then
                  Message1(parser_e_dir_not_allowed,arraytokeninfo[idtoken].str)
                else
                  include(symopt,sp_hint_unimplemented);
                try_consume_hintdirective:=true;
              end;
            else
              break;
          end;
          consume(Token);
          { handle deprecated message }
          if ((token=_CSTRING) or (token=_CCHAR)) and last_is_deprecated then
            begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              if not assigned(deprecatedmsg) then
                begin
                  if token=_CSTRING then
                    deprecatedmsg:=stringdup(cstringpattern)
                  else
                    deprecatedmsg:=stringdup(pattern);
                end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              if deprecatedmsg<>nil then
                internalerror(200910181);
              if token=_CSTRING then
                deprecatedmsg:=stringdup(cstringpattern)
              else
                deprecatedmsg:=stringdup(pattern);
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
              consume(token);
              include(symopt,sp_has_deprecated_msg);
            end;
        until false;
      end;

end.
