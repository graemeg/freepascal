
(* Yacc parser template (TP Yacc V3.0), V1.2 6-17-91 AG *)

(* global definitions: *)

program h2pas;
{$H+}
(*
    Copyright (c) 1998-2000 by Florian Klaempfl

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

 ****************************************************************************)

{$message TODO: warning Unit types is only needed due to issue 7910}

   uses
     SysUtils,types, classes,
     h2poptions,scan,converu,h2plexlib,h2pyacclib;

   type
     YYSTYPE = presobject;

   const
     SHORT_STR = 'shortint';
     USHORT_STR = 'byte';
     //C++ SHORT types usually map to the small types
     SMALL_STR  = 'smallint';
     USMALL_STR = 'word';
     INT_STR    = 'longint';
     UINT_STR   = 'dword';
     CHAR_STR   = 'char';
     UCHAR_STR  = USHORT_STR; { should we use byte or char for 'unsigned char' ?? }

     INT64_STR  = 'int64';
     QWORD_STR  = 'qword';
     FLOAT_STR  = 'single';
     WCHAR_STR  = 'widechar';

  {ctypes strings}
  const
    cint8_STR       = 'cint8';
    cuint8_STR      = 'cuint8';
    cchar_STR       = 'cchar';
    cschar_STR      = 'cschar';
    cuchar_STR      = 'cuchar';

    cint16_STR      = 'cint16';
    cuint16_STR     = 'cuint16';
    cshort_STR      = 'cshort';
    csshort_STR     = 'csshort';
    cushort_STR     = 'cushort';

    cint32_STR      = 'cint32';
    cuint32_STR     = 'cuint32';
    cint_STR        = 'cint';
    csint_STR       = 'csint';
    cuint_STR       = 'cuint';

    csigned_STR     = 'csigned';
    cunsigned_STR   = 'cunsigned';

    cint64_STR      = 'cint64';
    cuint64_STR     = 'cuint64';
    clonglong_STR   = 'clonglong';
    cslonglong_STR  = 'cslonglong';
    culonglong_STR  = 'culonglong';

    cbool_STR       = 'cbool';

    clong_STR       = 'clong';
    cslong_STR      = 'cslong';
    culong_STR      = 'culong';

    cfloat_STR      = 'cfloat';
    cdouble_STR     = 'cdouble';
    clongdouble_STR = 'clongdouble';

  const
    MAX_CTYPESARRAY = 25;
    CTypesArray : array [0..MAX_CTYPESARRAY] of string =
      (cint8_STR,     cuint8_STR,
       cchar_STR,     cschar_STR,     cuchar_STR,
       cint16_STR,    cuint16_STR,
       cshort_STR,    csshort_STR,    cushort_STR,
       csigned_STR,   cunsigned_STR,
       cint32_STR,    cuint32_STR,    cint_STR,
       csint_STR,     cuint_STR,
       cint64_STR,    cuint64_STR,
       clonglong_STR, cslonglong_STR, culonglong_STR,

       cbool_STR,
       clong_STR,      cslong_STR,    culong_STR);


  var
     hp,ph    : presobject;
     implemfile  : text;  (* file for implementation headers extern procs *)
     IsExtern : boolean;
     NeedEllipsisOverload : boolean;
     must_write_packed_field : boolean;
     tempfile : text;
     No_pop   : boolean;
     s,TN,PN  : String;
     pointerprefix: boolean;
     freedynlibproc,
     loaddynlibproc : tstringlist;


(* $ define yydebug
 compile with -dYYDEBUG to get debugging info *)

  const
     (* number of a?b:c construction in one define *)
     if_nb : longint = 0;
     is_packed : boolean = false;
     is_procvar : boolean = false;

  var space_array : array [0..255] of byte;
      space_index : byte;

      { Used when PPointers is used - pointer type definitions }
      PTypeList : TStringList;


        procedure shift(space_number : byte);
          var
             i : byte;
          begin
             space_array[space_index]:=space_number;
             inc(space_index);
             for i:=1 to space_number do
               aktspace:=aktspace+' ';
          end;

        procedure popshift;
          begin
             dec(space_index);
             if space_index<0 then
               internalerror(20);
             delete(aktspace,1,space_array[space_index]);
          end;

    function str(i : longint) : string;
      var
         s : string;
      begin
         system.str(i,s);
         str:=s;
      end;

    function hexstr(i : cardinal) : string;

    const
      HexTbl : array[0..15] of char='0123456789ABCDEF';
    var
      str : string;
    begin
      str:='';
      while i<>0 do
        begin
           str:=hextbl[i and $F]+str;
           i:=i shr 4;
        end;
      if str='' then str:='0';
      hexstr:='$'+str;
    end;

    function uppercase(s : string) : string;
      var
         i : byte;
      begin
         for i:=1 to length(s) do
           s[i]:=UpCase(s[i]);
         uppercase:=s;
      end;

    procedure write_type_specifier(var outfile:text; p : presobject);forward;
    procedure write_p_a_def(var outfile:text; p,simple_type : presobject);forward;
    procedure write_ifexpr(var outfile:text; p : presobject);forward;
    procedure write_funexpr(var outfile:text; p : presobject);forward;

    procedure yymsg(const msg : string);
      begin
         writeln('line ',line_no,': ',msg);
      end;


    { This converts pascal reserved words to
      the correct syntax.
    }
    function FixId(const s:string):string;
    const
     maxtokens = 14;
     reservedid: array[1..maxtokens] of string[14] =
       (
         'CLASS',
         'DISPOSE',
         'FUNCTION',
         'FALSE',
         'LABEL',
         'NEW',
         'PROPERTY',
         'PROCEDURE',
         'RECORD',
         'REPEAT',
         'STRING',
         'TYPE',
         'TRUE',
         'UNTIL'
       );
      var
        b : boolean;
        up : string;
        i: integer;
      begin
        if s='' then
         begin
           FixId:='';
           exit;
         end;
        b:=false;
        up:=Uppercase(s);
        for i:=1 to maxtokens do
          begin
            if up=reservedid[i] then
               begin
                  b:=true;
                  break;
                end;
          end;
        if b then
         FixId:='_'+s
        else
         FixId:=s;
      end;


    function TypeName(const s:string):string;
      var
        i : longint;
      begin
        i:=1;
        if RemoveUnderScore and (length(s)>1) and (s[1]='_') then
         i:=2;
        if PrependTypes then
         TypeName:='T'+Copy(s,i,255)
        else
         TypeName:=Copy(s,i,255);
      end;

    function IsACType(const s : String) : Boolean;
    var i : Integer;
    begin
      IsACType := True;
      for i := 0 to MAX_CTYPESARRAY do
      begin
        if s = CTypesArray[i] then
        begin
          Exit;
        end;
      end;
      IsACType := False;
    end;

    function PointerName(const s:string):string;
      var
        i : longint;
      begin
        if UseCTypesUnit then
        begin
          if IsACType(s) then
          begin
            PointerName := 'p'+s;
            exit;
          end;
        end;
        i:=1;
        if RemoveUnderScore and (length(s)>1) and (s[1]='_') then
         i:=2;
        if UsePPointers then
        begin
         PointerName:='P'+Copy(s,i,255);
         PTypeList.Add(PointerName);
        end
        else
         PointerName:=Copy(s,i,255);
        if PointerPrefix then
           PTypeList.Add('P'+s);
      end;

    procedure write_packed_fields_info(var outfile:text; p : presobject; ph : string);
      var
         hp1,hp2,hp3 : presobject;
         is_sized : boolean;
         line : string;
         flag_index : longint;
         name : pchar;
         ps : byte;

      begin
         { write out the tempfile created }
         close(tempfile);
         reset(tempfile);
         is_sized:=false;
         flag_index:=0;
         writeln(outfile);
         writeln(outfile,aktspace,'const');
         shift(2);
         while not eof(tempfile) do
           begin
              readln(tempfile,line);
              ps:=pos('&',line);
              if ps>0 then
                line:=copy(line,1,ps-1)+ph+'_'+copy(line,ps+1,255);
              writeln(outfile,aktspace,line);
           end;
         writeln(outfile);
         close(tempfile);
         rewrite(tempfile);
         popshift;
         (* walk through all members *)
         hp1 := p^.p1;
         while assigned(hp1) do
           begin
              (* hp2 is t_memberdec *)
              hp2:=hp1^.p1;
              (*  hp3 is t_declist *)
              hp3:=hp2^.p2;
              while assigned(hp3) do
                begin
                   if assigned(hp3^.p1^.p3) and
                      (hp3^.p1^.p3^.typ = t_size_specifier) then
                     begin
                        is_sized:=true;
                        name:=hp3^.p1^.p2^.p;
                        { get function in interface }
                        write(outfile,aktspace,'function ',name);
                        write(outfile,'(var a : ',ph,') : ');
                        shift(2);
                        write_p_a_def(outfile,hp3^.p1^.p1,hp2^.p1);
                        writeln(outfile,';');
                        popshift;
                        { get function in implementation }
                        write(implemfile,aktspace,'function ',name);
                        write(implemfile,'(var a : ',ph,') : ');
                        if not compactmode then
                         shift(2);
                        write_p_a_def(implemfile,hp3^.p1^.p1,hp2^.p1);
                        writeln(implemfile,';');
                        writeln(implemfile,aktspace,'begin');
                        shift(2);
                        write(implemfile,aktspace,name,':=(a.flag',flag_index);
                        writeln(implemfile,' and bm_',ph,'_',name,') shr bp_',ph,'_',name,';');
                        popshift;
                        writeln(implemfile,aktspace,'end;');
                        if not compactmode then
                         popshift;
                        writeln(implemfile,'');
                        { set function in interface }
                        write(outfile,aktspace,'procedure set_',name);
                        write(outfile,'(var a : ',ph,'; __',name,' : ');
                        shift(2);
                        write_p_a_def(outfile,hp3^.p1^.p1,hp2^.p1);
                        writeln(outfile,');');
                        popshift;
                        { set function in implementation }
                        write(implemfile,aktspace,'procedure set_',name);
                        write(implemfile,'(var a : ',ph,'; __',name,' : ');
                        if not compactmode then
                         shift(2);
                        write_p_a_def(implemfile,hp3^.p1^.p1,hp2^.p1);
                        writeln(implemfile,');');
                        writeln(implemfile,aktspace,'begin');
                        shift(2);
                        write(implemfile,aktspace,'a.flag',flag_index,':=');
                        write(implemfile,'a.flag',flag_index,' or ');
                        writeln(implemfile,'((__',name,' shl bp_',ph,'_',name,') and bm_',ph,'_',name,');');
                        popshift;
                        writeln(implemfile,aktspace,'end;');
                        if not compactmode then
                         popshift;
                        writeln(implemfile,'');
                     end
                   else if is_sized then
                     begin
                        is_sized:=false;
                        inc(flag_index);
                     end;
                   hp3:=hp3^.next;
                end;
              hp1:=hp1^.next;
           end;
         must_write_packed_field:=false;
         block_type:=bt_no;
      end;


    procedure write_expr(var outfile:text; p : presobject);
      begin
      if assigned(p) then
        begin
         case p^.typ of
            t_id,
            t_ifexpr :
              write(outfile,FixId(p^.p));
            t_funexprlist :
              write_funexpr(outfile,p);
             t_exprlist :
               begin
                 if assigned(p^.p1) then
                   write_expr(outfile,p^.p1);
                 if assigned(p^.next) then
                   begin
                     write(', ');
                     write_expr(outfile,p^.next);
                   end;
               end;
            t_preop : begin
                         write(outfile,p^.p,'(');
                         write_expr(outfile,p^.p1);
                         write(outfile,')');
                         flush(outfile);
                      end;
            t_typespec : begin
                         write_type_specifier(outfile,p^.p1);
                         write(outfile,'(');
                         write_expr(outfile,p^.p2);
                         write(outfile,')');
                         flush(outfile);
                      end;
            t_bop : begin
                       if p^.p1^.typ<>t_id then
                         write(outfile,'(');
                       write_expr(outfile,p^.p1);
                       if p^.p1^.typ<>t_id then
                       write(outfile,')');
                       write(outfile,p^.p);
                       if p^.p2^.typ<>t_id then
                         write(outfile,'(');
                       write_expr(outfile,p^.p2);
                       if p^.p2^.typ<>t_id then
                         write(outfile,')');
                       flush(outfile);
                    end;
            t_arrayop :
                    begin
                      write_expr(outfile,p^.p1);
                      write(outfile,p^.p,'[');
                      write_expr(outfile,p^.p2);
                      write(outfile,']');
                      flush(outfile);
                    end;
            t_callop :
                    begin
                      write_expr(outfile,p^.p1);
                      write(outfile,p^.p,'(');
                      write_expr(outfile,p^.p2);
                      write(outfile,')');
                      flush(outfile);
                    end;
            else
              begin
                writeln(ord(p^.typ));
                internalerror(2);
              end;
            end;
         end;
      end;


    procedure write_ifexpr(var outfile:text; p : presobject);
      begin
         flush(outfile);
         write(outfile,'if ');
         write_expr(outfile,p^.p1);
         writeln(outfile,' then');
         write(outfile,aktspace,'  ');
         write(outfile,p^.p);
         write(outfile,':=');
         write_expr(outfile,p^.p2);
         writeln(outfile);
         writeln(outfile,aktspace,'else');
         write(outfile,aktspace,'  ');
         write(outfile,p^.p);
         write(outfile,':=');
         write_expr(outfile,p^.p3);
         writeln(outfile,';');
         write(outfile,aktspace);
         flush(outfile);
      end;


    procedure write_all_ifexpr(var outfile:text; p : presobject);
      begin
      if assigned(p) then
        begin
           case p^.typ of
             t_id :;
             t_preop :
               write_all_ifexpr(outfile,p^.p1);
             t_callop,
             t_arrayop,
             t_bop :
               begin
                  write_all_ifexpr(outfile,p^.p1);
                  write_all_ifexpr(outfile,p^.p2);
               end;
             t_ifexpr :
               begin
                  write_all_ifexpr(outfile,p^.p1);
                  write_all_ifexpr(outfile,p^.p2);
                  write_all_ifexpr(outfile,p^.p3);
                  write_ifexpr(outfile,p);
               end;
             t_typespec :
                  write_all_ifexpr(outfile,p^.p2);
             t_funexprlist,
             t_exprlist :
               begin
                 if assigned(p^.p1) then
                   write_all_ifexpr(outfile,p^.p1);
                 if assigned(p^.next) then
                   write_all_ifexpr(outfile,p^.next);
               end
             else
               internalerror(6);
           end;
        end;
      end;

    procedure write_funexpr(var outfile:text; p : presobject);
      var
         i : longint;

      begin
      if assigned(p) then
        begin
           case p^.typ of
             t_ifexpr :
               write(outfile,p^.p);
             t_exprlist :
               begin
                  write_expr(outfile,p^.p1);
                  if assigned(p^.next) then
                    begin
                      write(outfile,',');
                      write_funexpr(outfile,p^.next);
                    end
               end;
             t_funcname :
               begin
                  if if_nb>0 then
                    begin
                       writeln(outfile,aktspace,'var');
                       write(outfile,aktspace,'   ');
                       for i:=1 to if_nb do
                         begin
                            write(outfile,'if_local',i);
                            if i<if_nb then
                              write(outfile,', ')
                            else
                              writeln(outfile,' : longint;');
                         end;
                       writeln(outfile,aktspace,'(* result types are not known *)');
                       if_nb:=0;
                    end;
                  writeln(outfile,aktspace,'begin');
                  shift(2);
                  write(outfile,aktspace);
                  write_all_ifexpr(outfile,p^.p2);
                  write_expr(outfile,p^.p1);
                  write(outfile,':=');
                  write_funexpr(outfile,p^.p2);
                  writeln(outfile,';');
                  popshift;
                  writeln(outfile,aktspace,'end;');
                  if not compactmode then
                   popshift;
                  flush(outfile);
               end;
             t_funexprlist :
               begin
                  if assigned(p^.p3) then
                    begin
                       write_type_specifier(outfile,p^.p3);
                       write(outfile,'(');
                    end;
                  if assigned(p^.p1) then
                    write_funexpr(outfile,p^.p1);
                  if assigned(p^.p2) then
                    begin
                      write(outfile,'(');
                      write_funexpr(outfile,p^.p2);
                      write(outfile,')');
                    end;
                  if assigned(p^.p3) then
                    write(outfile,')');
               end
             else internalerror(5);
           end;
        end;
      end;

     function ellipsisarg : presobject;
       begin
          ellipsisarg:=new(presobject,init_two(t_arg,nil,nil));
       end;

    const
       (* if in args *dname is replaced by pdname *)
       in_args : boolean = false;
       typedef_level : longint = 0;
       old_in_args : boolean = false;

    (* writes an argument list, where p is t_arglist *)

    procedure write_args(var outfile:text; p : presobject);
      var
         len,para : longint;
         old_in_args : boolean;
         varpara : boolean;
         lastp : presobject;
         hs : string;
      begin
         NeedEllipsisOverload:=false;
         para:=1;
         len:=0;
         lastp:=nil;
         old_in_args:=in_args;
         in_args:=true;
         write(outfile,'(');
         shift(2);

         (* walk through all arguments *)
         (* p must be of type t_arglist *)
         while assigned(p) do
           begin
              if p^.typ<>t_arglist then
                internalerror(10);
              (* is ellipsis ? *)
              if not assigned(p^.p1^.p1) and
                 not assigned(p^.p1^.next) then
                begin
                   write(outfile,'args:array of const');
                   (* if variable number of args we must allways pop *)
                   no_pop:=false;
                   (* Needs 2 declarations, also one without args, because
                      in C you can omit the second parameter. Default parameter
                      doesn't help as that isn't possible with array of const *)
                   NeedEllipsisOverload:=true;
                   (* Remove this para *)
                   if assigned(lastp) then
                    lastp^.next:=nil;
                   dispose(p,done);
                   (* leave the loop as p isnot valid anymore *)
                   break;
                end
              (* we need to correct this in the pp file after *)
              else
                begin
                   (* generate a call by reference parameter ?       *)

//                   varpara:=usevarparas and
//                            assigned(p^.p1^.p2^.p1) and
//                            (p^.p1^.p2^.p1^.typ in [t_addrdef,t_pointerdef]) and
//                            assigned(p^.p1^.p2^.p1^.p1) and
//                            (p^.p1^.p2^.p1^.p1^.typ<>t_procdef);
                   varpara:=usevarparas and
                            assigned(p^.p1^.p1) and
                            (p^.p1^.p1^.typ in [t_addrdef,t_pointerdef]) and
                            assigned(p^.p1^.p1^.p1) and
                            (p^.p1^.p1^.p1^.typ<>t_procdef);
                   (* do not do it for char pointer !!               *)
                   (* para : pchar; and var para : char; are         *)
                   (* completely different in pascal                 *)
                   (* here we exclude all typename containing char   *)
                   (* is this a good method ??                       *)
                   if varpara and
                      (p^.p1^.p1^.typ=t_pointerdef) and
                      (p^.p1^.p1^.p1^.typ=t_id) and
                      (pos('CHAR',uppercase(p^.p1^.p1^.p1^.str))<>0) then
                     varpara:=false;
                   if varpara then
                     begin
                        write(outfile,'var ');
                        inc(len,4);
                     end;

                   (* write new parameter name *)
                   if assigned(p^.p1^.p2^.p2) then
                     begin
                        hs:=FixId(p^.p1^.p2^.p2^.p);
                        write(outfile,hs);
                        inc(len,length(hs));
                     end
                   else
                     begin
                       If removeUnderscore then
                         begin
                           Write (outfile,'para',para);
                           inc(Len,5);
                         end
                       else
                         begin
                           write(outfile,'_para',para);
                           inc(Len,6);
                         end;
                     end;
                   write(outfile,':');
                   if varpara then
                   begin
                     write_p_a_def(outfile,p^.p1^.p2^.p1,p^.p1^.p1^.p1);
                   end
                   else
                     write_p_a_def(outfile,p^.p1^.p2^.p1,p^.p1^.p1);

                end;
              lastp:=p;
              p:=p^.next;
              if assigned(p) then
                begin
                   write(outfile,'; ');
                   { if len>40 then : too complicated to compute }
                   if (para mod 5) = 0 then
                     begin
                        writeln(outfile);
                        write(outfile,aktspace);
                     end;
                end;
              inc(para);
           end;
         write(outfile,')');
         flush(outfile);
         in_args:=old_in_args;
         popshift;
      end;



    procedure write_p_a_def(var outfile:text; p,simple_type : presobject);
      var
         i : longint;
         error : integer;
         pointerwritten,
         constant : boolean;
         old_in_args : boolean;
      begin
         if not(assigned(p)) then
           begin
              write_type_specifier(outfile,simple_type);
              exit;
           end;
         case p^.typ of
            t_pointerdef :
              begin
                (* procedure variable ? *)
                if assigned(p^.p1) and (p^.p1^.typ=t_procdef) then
                  begin
                     is_procvar:=true;
                     (* distinguish between procedure and function *)
                     if (simple_type^.typ=t_void) and (p^.p1^.p1=nil) then
                       begin
                          write(outfile,'procedure ');

                          shift(10);
                          (* write arguments *)
                          if assigned(p^.p1^.p2) then
                            write_args(outfile,p^.p1^.p2);
                          flush(outfile);
                          popshift;
                       end
                     else
                       begin
                          write(outfile,'function ');
                          shift(9);
                          (* write arguments *)
                          if assigned(p^.p1^.p2) then
                            write_args(outfile,p^.p1^.p2);
                          write(outfile,':');
                          flush(outfile);

                          old_in_args:=in_args;
                          (* write pointers as P.... instead of ^.... *)
                          in_args:=true;
                          write_p_a_def(outfile,p^.p1^.p1,simple_type);
                          in_args:=old_in_args;
                          popshift;
                       end
                  end
                else
                  begin
                     (* generate "pointer" ? *)
                     if (simple_type^.typ=t_void) and (p^.p1=nil) then
                       begin
                         write(outfile,'pointer');
                         flush(outfile);
                       end
                     else
                       begin
                         pointerwritten:=false;
                         if (p^.p1=nil) and UsePPointers then
                          begin
                            if (simple_type^.typ=t_id) then
                             begin
                               write(outfile,PointerName(simple_type^.p));
                               pointerwritten:=true;
                             end
                            { structure }
                            else if (simple_type^.typ in [t_uniondef,t_structdef]) and
                                    (simple_type^.p1=nil) and (simple_type^.p2^.typ=t_id) then
                             begin
                               write(outfile,PointerName(simple_type^.p2^.p));
                               pointerwritten:=true;
                             end;
                          end;
                        if not pointerwritten then
                         begin
                           if in_args then
                           begin
                            write(outfile,'P');
                            pointerprefix:=true;
                           end
                           else
                            write(outfile,'^');
                           write_p_a_def(outfile,p^.p1,simple_type);
                           pointerprefix:=false;
                         end;
                       end;
                  end;
              end;
            t_arraydef :
              begin
                constant:=false;
                if assigned(p^.p2) then
                 begin
                   if p^.p2^.typ=t_id then
                    begin
                      val(p^.p2^.str,i,error);
                      if error=0 then
                       begin
                         dec(i);
                         constant:=true;
                       end;
                    end;
                   if not constant then
                    begin
                      write(outfile,'array[0..(');
                      write_expr(outfile,p^.p2);
                      write(outfile,')-1] of ');
                    end
                   else
                    begin
                      write(outfile,'array[0..',i,'] of ');
                    end;
                 end
                else
                 begin
                   (* open array *)
                   write(outfile,'array of ');
                 end;
                flush(outfile);
                write_p_a_def(outfile,p^.p1,simple_type);
              end;
            else internalerror(1);
         end;
      end;

    procedure write_type_specifier(var outfile:text; p : presobject);
      var
         hp1,hp2,hp3,lastexpr : presobject;
         i,l,w : longint;
         error : integer;
         current_power,
         mask : cardinal;
         flag_index : longint;
         current_level : byte;
         pointerwritten,
         is_sized : boolean;

      begin
         case p^.typ of
            t_id :
              begin
                if pointerprefix then
                  if UseCtypesUnit then
                  begin
                    if not IsACType(p^.p) then
                    begin
                      PTypeList.Add('P'+p^.str);
                    end;
                  end
                  else
                   PTypeList.Add('P'+p^.str);
                if p^.intname then
                 write(outfile,p^.p)
                else
                 write(outfile,TypeName(p^.p));
              end;
            { what can we do with void defs  ? }
            t_void :
              write(outfile,'pointer');
            t_pointerdef :
              begin
                 pointerwritten:=false;
                 if (p^.p1^.typ=t_void) then
                  begin
                    write(outfile,'pointer');
                    pointerwritten:=true;
                  end
                 else
                  if UsePPointers then
                   begin
                     if (p^.p1^.typ=t_id) then
                      begin
                        write(outfile,PointerName(p^.p1^.p));
                        pointerwritten:=true;
                      end
                     { structure }
                     else if (p^.p1^.typ in [t_uniondef,t_structdef]) and
                             (p^.p1^.p1=nil) and (p^.p1^.p2^.typ=t_id) then
                      begin
                        write(outfile,PointerName(p^.p1^.p2^.p));
                        pointerwritten:=true;
                      end;
                   end;
                 if not pointerwritten then
                  begin
                    if in_args then
                    begin
                      if UseCTypesUnit and IsACType(p^.p1^.p) then
                        write(outfile,'p')
                      else
                        write(outfile,'P');
                      pointerprefix:=true;
                    end
                    else
                    begin
                      if UseCTypesUnit and (IsACType(p^.p1^.p)=False) then
                        write(outfile,'^')
                      else
                        write(outfile,'p');
                    end;
                    write_type_specifier(outfile,p^.p1);
                    pointerprefix:=false;
                  end;
              end;
            t_enumdef :
              begin
                 if (typedef_level>1) and (p^.p1=nil) and
                    (p^.p2^.typ=t_id) then
                   begin
                      if pointerprefix then
                        if UseCTypesUnit and (IsACType( p^.p2^.p )=False) then
                          PTypeList.Add('P'+p^.p2^.str);
                      write(outfile,p^.p2^.p);
                   end
                 else
                 if not EnumToConst then
                   begin
                      write(outfile,'(');
                      hp1:=p^.p1;
                      w:=length(aktspace);
                      while assigned(hp1) do
                        begin
                           write(outfile,hp1^.p1^.p);
                           if assigned(hp1^.p2) then
                             begin
                                write(outfile,' := ');
                                write_expr(outfile,hp1^.p2);
                                w:=w+6;(* strlen(hp1^.p); *)
                             end;
                           w:=w+length(hp1^.p1^.str);
                           hp1:=hp1^.next;
                           if assigned(hp1) then
                             write(outfile,',');
                           if w>40 then
                             begin
                                 writeln(outfile);
                                 write(outfile,aktspace);
                                 w:=length(aktspace);
                             end;
                           flush(outfile);
                        end;
                      write(outfile,')');
                      flush(outfile);
                   end
                 else
                   begin
                      Writeln (outfile,' Longint;');
                      hp1:=p^.p1;
                      l:=0;
                      lastexpr:=nil;
                      Writeln (outfile,copy(aktspace,1,length(aktspace)-2),'Const');
                      while assigned(hp1) do
                        begin
                           write (outfile,aktspace,hp1^.p1^.p,' = ');
                           if assigned(hp1^.p2) then
                             begin
                                write_expr(outfile,hp1^.p2);
                                writeln(outfile,';');
                                lastexpr:=hp1^.p2;
                                if lastexpr^.typ=t_id then
                                  begin
                                     val(lastexpr^.str,l,error);
                                     if error=0 then
                                       begin
                                          inc(l);
                                          lastexpr:=nil;
                                       end
                                     else
                                       l:=1;
                                  end
                                else
                                  l:=1;
                             end
                           else
                             begin
                                if assigned(lastexpr) then
                                  begin
                                     write(outfile,'(');
                                     write_expr(outfile,lastexpr);
                                     writeln(outfile,')+',l,';');
                                  end
                                else
                                  writeln (outfile,l,';');
                                inc(l);
                             end;
                           hp1:=hp1^.next;
                           flush(outfile);
                        end;
                      block_type:=bt_const;
                  end;
               end;
            t_structdef :
              begin
                 inc(typedef_level);
                 flag_index:=-1;
                 is_sized:=false;
                 current_level:=0;
                 if ((in_args) or (typedef_level>1)) and
                    (p^.p1=nil) and (p^.p2^.typ=t_id) then
                   begin
                      if pointerprefix then
                        if UseCTypesUnit and (IsACType(p^.p2^.str)=false) then
                          PTypeList.Add('P'+p^.p2^.str);
                     write(outfile,TypeName(p^.p2^.p));
                   end
                 else
                   begin
                      if packrecords then
                        writeln(outfile,'packed record')
                      else
                        writeln(outfile,'record');
                      shift(2);
                      hp1:=p^.p1;

                      (* walk through all members *)
                      while assigned(hp1) do
                        begin
                           (* hp2 is t_memberdec *)
                           hp2:=hp1^.p1;
                           (*  hp3 is t_declist *)
                           hp3:=hp2^.p2;
                           while assigned(hp3) do
                             begin
                                if assigned(hp3^.p1) and
                                   (not assigned(hp3^.p1^.p3) or
                                   (hp3^.p1^.p3^.typ <> t_size_specifier)) then
                                  begin
                                     if is_sized then
                                       begin
                                          if current_level <= 16 then
                                            writeln(outfile,'word;')
                                          else if current_level <= 32 then
                                            writeln(outfile,'longint;')
                                          else
                                            internalerror(11);
                                          is_sized:=false;
                                       end;

                                     write(outfile,aktspace,FixId(hp3^.p1^.p2^.p));
                                     write(outfile,' : ');
                                     shift(2);
                                     write_p_a_def(outfile,hp3^.p1^.p1,hp2^.p1);
                                     popshift;
                                  end;
                                { size specifier  or default value ? }
                                if assigned(hp3^.p1) and
                                   assigned(hp3^.p1^.p3) then
                                  begin
                                     { we could use mask to implement this }
                                     { because we need to respect the positions }
                                     if hp3^.p1^.p3^.typ = t_size_specifier then
                                       begin
                                          if not is_sized then
                                            begin
                                               current_power:=1;
                                               current_level:=0;
                                               inc(flag_index);
                                               write(outfile,aktspace,'flag',flag_index,' : ');
                                            end;
                                          must_write_packed_field:=true;
                                          is_sized:=true;
                                          { can it be something else than a constant ? }
                                          { it can be a macro !! }
                                          if hp3^.p1^.p3^.p1^.typ=t_id then
                                            begin
                                              val(hp3^.p1^.p3^.p1^.str,l,error);
                                              if error=0 then
                                                begin
                                                   mask:=0;
                                                   for i:=1 to l do
                                                     begin
                                                        inc(mask,current_power);
                                                        current_power:=current_power*2;
                                                     end;
                                                   write(tempfile,'bm_&',hp3^.p1^.p2^.p);
                                                   writeln(tempfile,' = ',hexstr(mask),';');
                                                   write(tempfile,'bp_&',hp3^.p1^.p2^.p);
                                                   writeln(tempfile,' = ',current_level,';');
                                                   current_level:=current_level + l;
                                                   { go to next flag if 31 }
                                                   if current_level = 32 then
                                                     begin
                                                        write(outfile,'longint');
                                                        is_sized:=false;
                                                     end;
                                                end;
                                            end;

                                       end
                                     else if hp3^.p1^.p3^.typ = t_default_value then
                                       begin
                                          write(outfile,'{=');
                                          write_expr(outfile,hp3^.p1^.p3^.p1);
                                          write(outfile,' ignored}');
                                       end;
                                  end;
                                if not is_sized then
                                  begin
                                     if is_procvar then
                                       begin
                                          if not no_pop then
                                            write(outfile,';cdecl');
                                          is_procvar:=false;
                                       end;
                                     writeln(outfile,';');
                                  end;
                                hp3:=hp3^.next;
                             end;
                           hp1:=hp1^.next;
                        end;
                      if is_sized then
                        begin
                           if current_level <= 16 then
                             writeln(outfile,'word;')
                           else if current_level <= 32 then
                             writeln(outfile,'longint;')
                           else
                             internalerror(11);
                           is_sized:=false;
                        end;
                      popshift;
                      write(outfile,aktspace,'end');
                      flush(outfile);
                   end;
                 dec(typedef_level);
              end;
            t_uniondef :
              begin
                 inc(typedef_level);
                 if (typedef_level>1) and (p^.p1=nil) and
                    (p^.p2^.typ=t_id) then
                   begin
                      write(outfile,p^.p2^.p);
                   end
                 else
                   begin
                      inc(typedef_level);
                      if packrecords then
                        writeln(outfile,'packed record')
                      else
                        writeln(outfile,'record');
                      shift(2);
                      writeln(outfile,aktspace,'case longint of');
                      shift(2);
                      l:=0;
                      hp1:=p^.p1;

                      (* walk through all members *)
                      while assigned(hp1) do
                        begin
                           (* hp2 is t_memberdec *)
                           hp2:=hp1^.p1;
                           (* hp3 is t_declist *)
                           hp3:=hp2^.p2;
                           while assigned(hp3) do
                             begin
                                write(outfile,aktspace,l,' : ( ');
                                write(outfile,FixId(hp3^.p1^.p2^.p),' : ');
                                shift(2);
                                write_p_a_def(outfile,hp3^.p1^.p1,hp2^.p1);
                                popshift;
                                writeln(outfile,' );');
                                hp3:=hp3^.next;
                                inc(l);
                             end;
                           hp1:=hp1^.next;
                        end;
                      popshift;
                      write(outfile,aktspace,'end');
                      popshift;
                      flush(outfile);
                      dec(typedef_level);
                   end;
                 dec(typedef_level);
              end;
            else
              internalerror(3);
         end;
      end;

    procedure write_def_params(var outfile:text; p : presobject);
      var
         hp1 : presobject;
      begin
         case p^.typ of
            t_enumdef : begin
                           hp1:=p^.p1;
                           while assigned(hp1) do
                             begin
                                write(outfile,FixId(hp1^.p1^.p));
                                hp1:=hp1^.next;
                                if assigned(hp1) then
                                  write(outfile,',')
                                else
                                  write(outfile);
                                flush(outfile);
                             end;
                           flush(outfile);
                        end;
         else internalerror(4);
         end;
      end;


    procedure write_statement_block(var outfile:text; p : presobject);
      begin
        writeln(outfile,aktspace,'begin');
        while assigned(p) do
          begin
            shift(2);
            if assigned(p^.p1) then
              begin
                case p^.p1^.typ of
                  t_whilenode:
                    begin
                      write(outfile,aktspace,'while ');
                      write_expr(outfile,p^.p1^.p1);
                      writeln(outfile,' do');
                      shift(2);
                      write_statement_block(outfile,p^.p1^.p2);
                      popshift;
                    end;
                  else
                    begin
                      write(outfile,aktspace);
                      write_expr(outfile,p^.p1);
                      writeln(outfile,';');
                    end;
                end;
              end;
            p:=p^.next;
            popshift;
          end;
        writeln(outfile,aktspace,'end;');
      end;


const _WHILE = 257;
const _FOR = 258;
const _DO = 259;
const _GOTO = 260;
const _CONTINUE = 261;
const _BREAK = 262;
const TYPEDEF = 263;
const DEFINE = 264;
const COLON = 265;
const SEMICOLON = 266;
const COMMA = 267;
const LKLAMMER = 268;
const RKLAMMER = 269;
const LECKKLAMMER = 270;
const RECKKLAMMER = 271;
const LGKLAMMER = 272;
const RGKLAMMER = 273;
const STRUCT = 274;
const UNION = 275;
const ENUM = 276;
const ID = 277;
const NUMBER = 278;
const CSTRING = 279;
const SHORT = 280;
const UNSIGNED = 281;
const LONG = 282;
const INT = 283;
const FLOAT = 284;
const _CHAR = 285;
const VOID = 286;
const _CONST = 287;
const _FAR = 288;
const _HUGE = 289;
const _NEAR = 290;
const NEW_LINE = 291;
const SPACE_DEFINE = 292;
const EXTERN = 293;
const STDCALL = 294;
const CDECL = 295;
const CALLBACK = 296;
const PASCAL = 297;
const WINAPI = 298;
const APIENTRY = 299;
const WINGDIAPI = 300;
const SYS_TRAP = 301;
const _PACKED = 302;
const ELLIPSIS = 303;
const _ASSIGN = 304;
const R_AND = 305;
const EQUAL = 306;
const UNEQUAL = 307;
const GT = 308;
const LT = 309;
const GTE = 310;
const LTE = 311;
const QUESTIONMARK = 312;
const _OR = 313;
const _AND = 314;
const _PLUS = 315;
const MINUS = 316;
const _SHR = 317;
const _SHL = 318;
const STAR = 319;
const _SLASH = 320;
const _NOT = 321;
const PSTAR = 322;
const P_AND = 323;
const POINT = 324;
const DEREF = 325;
const STICK = 326;
const SIGNED = 327;
const INT8 = 328;
const INT16 = 329;
const INT32 = 330;
const INT64 = 331;

var yylval : YYSType;

function yylex : Integer; forward;

function yyparse : Integer;

var yystate, yysp, yyn : Integer;
    yys : array [1..yymaxdepth] of Integer;
    yyv : array [1..yymaxdepth] of YYSType;
    yyval : YYSType;

procedure yyaction ( yyruleno : Integer );
  (* local definitions: *)
begin
  (* actions: *)
  case yyruleno of
   1 : begin
         yyval := yyv[yysp-0];
       end;
   2 : begin
         
         yyval:=nil;
         
       end;
   3 : begin
         
         yyval:=nil;
         
       end;
   4 : begin
         
         writeln(outfile,'(* error ');
         writeln(outfile,yyline);
         
       end;
   5 : begin
         if yydebug then writeln('declaration reduced at line ',line_no);
         if yydebug then writeln(outfile,'(* declaration reduced *)');
         
       end;
   6 : begin
         if yydebug then writeln('define declaration reduced at line ',line_no);
         if yydebug then writeln(outfile,'(* define declaration reduced *)');
         
       end;
   7 : begin
         if yydebug then writeln('declaration reduced at line ',line_no);
         
       end;
   8 : begin
         if yydebug then writeln('define declaration reduced at line ',line_no);
         
       end;
   9 : begin
         yyval:=new(presobject,init_id('extern')); 
       end;
  10 : begin
         yyval:=new(presobject,init_id('intern')); 
       end;
  11 : begin
         yyval:=new(presobject,init_id('no_pop')); 
       end;
  12 : begin
         yyval:=new(presobject,init_id('cdecl')); 
       end;
  13 : begin
         yyval:=new(presobject,init_id('no_pop')); 
       end;
  14 : begin
         yyval:=new(presobject,init_id('no_pop')); 
       end;
  15 : begin
         yyval:=new(presobject,init_id('no_pop')); 
       end;
  16 : begin
         yyval:=new(presobject,init_id('no_pop')); 
       end;
  17 : begin
         yyval:=new(presobject,init_id('no_pop')); 
       end;
  18 : begin
         yyval:=nil 
       end;
  19 : begin
         yyval:=yyv[yysp-1]; 
       end;
  20 : begin
         yyval:=nil; 
       end;
  21 : begin
         yyval:=yyv[yysp-1]; 
       end;
  22 : begin
         yyval:=new(presobject,init_two(t_whilenode,yyv[yysp-2],yyv[yysp-0])); 
       end;
  23 : begin
         
         yyval:=new(presobject,init_one(t_statement_list,yyv[yysp-1]));
         yyval^.next:=yyv[yysp-0];
         
       end;
  24 : begin
         
         yyval:=new(presobject,init_one(t_statement_list,yyv[yysp-0]));
         
       end;
  25 : begin
         
         yyval:=new(presobject,init_one(t_statement_list,nil));
         
       end;
  26 : begin
         
         yyval:=new(presobject,init_one(t_statement_list,nil));
         
       end;
  27 : begin
         yyval:=yyv[yysp-1]; 
       end;
  28 : begin
         
         IsExtern:=false;
         (* by default we must pop the args pushed on stack *)
         no_pop:=false;
         if (assigned(yyv[yysp-1])and assigned(yyv[yysp-1]^.p1)and assigned(yyv[yysp-1]^.p1^.p1))
         and (yyv[yysp-1]^.p1^.p1^.typ=t_procdef) then
         begin
         repeat
         If UseLib then
         IsExtern:=true
         else
         IsExtern:=assigned(yyv[yysp-4])and(yyv[yysp-4]^.str='extern');
         no_pop:=assigned(yyv[yysp-2]) and (yyv[yysp-2]^.str='no_pop');
         
         if (block_type<>bt_func) and not(createdynlib) then
         begin
         writeln(outfile);
         block_type:=bt_func;
         end;
         
         (* dyn. procedures must be put into a var block *)
         if createdynlib then
         begin
         if (block_type<>bt_var) then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'var');
         block_type:=bt_var;
         end;
         shift(2);
         end;
         if not CompactMode then
         begin
         write(outfile,aktspace);
         if not IsExtern then
         write(implemfile,aktspace);
         end;
         (* distinguish between procedure and function *)
         if assigned(yyv[yysp-3]) then
         if (yyv[yysp-3]^.typ=t_void) and (yyv[yysp-1]^.p1^.p1^.p1=nil) then
         begin
         if createdynlib then
         begin
         write(outfile,yyv[yysp-1]^.p1^.p2^.p,' : procedure');
         end
         else
         begin
         shift(10);
         write(outfile,'procedure ',yyv[yysp-1]^.p1^.p2^.p);
         end;
         if assigned(yyv[yysp-1]^.p1^.p1^.p2) then
         write_args(outfile,yyv[yysp-1]^.p1^.p1^.p2);
         if createdynlib then
         begin
         loaddynlibproc.add('pointer('+yyv[yysp-1]^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+yyv[yysp-1]^.p1^.p2^.p+''');');
         freedynlibproc.add(yyv[yysp-1]^.p1^.p2^.p+':=nil;');
         end
         else if not IsExtern then
         begin
         write(implemfile,'procedure ',yyv[yysp-1]^.p1^.p2^.p);
         if assigned(yyv[yysp-1]^.p1^.p1^.p2) then
         write_args(implemfile,yyv[yysp-1]^.p1^.p1^.p2);
         end;
         end
         else
         begin
         if createdynlib then
         begin
         write(outfile,yyv[yysp-1]^.p1^.p2^.p,' : function');
         end
         else
         begin
         shift(9);
         write(outfile,'function ',yyv[yysp-1]^.p1^.p2^.p);
         end;
         
         if assigned(yyv[yysp-1]^.p1^.p1^.p2) then
         write_args(outfile,yyv[yysp-1]^.p1^.p1^.p2);
         write(outfile,':');
         old_in_args:=in_args;
         (* write pointers as P.... instead of ^.... *)
         in_args:=true;
         write_p_a_def(outfile,yyv[yysp-1]^.p1^.p1^.p1,yyv[yysp-3]);
         in_args:=old_in_args;
         if createdynlib then
         begin
         loaddynlibproc.add('pointer('+yyv[yysp-1]^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+yyv[yysp-1]^.p1^.p2^.p+''');');
         freedynlibproc.add(yyv[yysp-1]^.p1^.p2^.p+':=nil;');
         end
         else if not IsExtern then
         begin
         write(implemfile,'function ',yyv[yysp-1]^.p1^.p2^.p);
         if assigned(yyv[yysp-1]^.p1^.p1^.p2) then
         write_args(implemfile,yyv[yysp-1]^.p1^.p1^.p2);
         write(implemfile,':');
         
         old_in_args:=in_args;
         (* write pointers as P.... instead of ^.... *)
         in_args:=true;
         write_p_a_def(implemfile,yyv[yysp-1]^.p1^.p1^.p1,yyv[yysp-3]);
         in_args:=old_in_args;
         end;
         end;
         (* No CDECL in interface for Uselib *)
         if IsExtern and (not no_pop) then
         write(outfile,';cdecl');
         popshift;
         if createdynlib then
         begin
         writeln(outfile,';');
         end
         else if UseLib then
         begin
         if IsExtern then
         begin
         write (outfile,';external');
         If UseName then
         Write(outfile,' External_library name ''',yyv[yysp-1]^.p1^.p2^.p,'''');
         end;
         writeln(outfile,';');
         end
         else
         begin
         writeln(outfile,';');
         if not IsExtern then
         begin
         writeln(implemfile,';');
         shift(2);
         if yyv[yysp-0]^.typ=t_statement_list then
         write_statement_block(implemfile,yyv[yysp-0]);
         popshift;
         end;
         end;
         IsExtern:=false;
         if not(compactmode) and not(createdynlib) then
         writeln(outfile);
         until not NeedEllipsisOverload;
         end
         else (* yyv[yysp-1]^.p1^.p1^.typ=t_procdef *)
         if assigned(yyv[yysp-1])and assigned(yyv[yysp-1]^.p1) then
         begin
         shift(2);
         if block_type<>bt_var then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'var');
         end;
         block_type:=bt_var;
         
         shift(2);
         
         IsExtern:=assigned(yyv[yysp-4])and(yyv[yysp-4]^.str='extern');
         (* walk through all declarations *)
         hp:=yyv[yysp-1];
         while assigned(hp) and assigned(hp^.p1) do
         begin
         (* write new var name *)
         if assigned(hp^.p1^.p2) and assigned(hp^.p1^.p2^.p) then
         write(outfile,aktspace,hp^.p1^.p2^.p);
         write(outfile,' : ');
         shift(2);
         (* write its type *)
         write_p_a_def(outfile,hp^.p1^.p1,yyv[yysp-3]);
         if assigned(hp^.p1^.p2)and assigned(hp^.p1^.p2^.p)then
         begin
         if isExtern then
         write(outfile,';cvar;external')
         else
         write(outfile,';cvar;public');
         end;
         writeln(outfile,';');
         popshift;
         hp:=hp^.p2;
         end;
         popshift;
         popshift;
         end;
         if assigned(yyv[yysp-4]) then
         dispose(yyv[yysp-4],done);
         if assigned(yyv[yysp-3]) then
         dispose(yyv[yysp-3],done);
         if assigned(yyv[yysp-2]) then
         dispose(yyv[yysp-2],done);
         if assigned(yyv[yysp-1]) then
         dispose(yyv[yysp-1],done);
         if assigned(yyv[yysp-0]) then
         dispose(yyv[yysp-0],done);
         
       end;
  29 : begin
         
         IsExtern:=false;
         (* by default we must pop the args pushed on stack *)
         no_pop:=false;
         if (assigned(yyv[yysp-2])and assigned(yyv[yysp-2]^.p1)and assigned(yyv[yysp-2]^.p1^.p1))
         and (yyv[yysp-2]^.p1^.p1^.typ=t_procdef) then
         begin
         repeat
         If UseLib then
         IsExtern:=true
         else
         IsExtern:=assigned(yyv[yysp-5])and(yyv[yysp-5]^.str='extern');
         no_pop:=assigned(yyv[yysp-3]) and (yyv[yysp-3]^.str='no_pop');
         
         if (block_type<>bt_func) and not(createdynlib) then
         begin
         writeln(outfile);
         block_type:=bt_func;
         end;
         
         (* dyn. procedures must be put into a var block *)
         if createdynlib then
         begin
         if (block_type<>bt_var) then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'var');
         block_type:=bt_var;
         end;
         shift(2);
         end;
         if not CompactMode then
         begin
         write(outfile,aktspace);
         if not IsExtern then
         write(implemfile,aktspace);
         end;
         (* distinguish between procedure and function *)
         if assigned(yyv[yysp-4]) then
         if (yyv[yysp-4]^.typ=t_void) and (yyv[yysp-2]^.p1^.p1^.p1=nil) then
         begin
         if createdynlib then
         begin
         write(outfile,yyv[yysp-2]^.p1^.p2^.p,' : procedure');
         end
         else
         begin
         shift(10);
         write(outfile,'procedure ',yyv[yysp-2]^.p1^.p2^.p);
         end;
         if assigned(yyv[yysp-2]^.p1^.p1^.p2) then
         write_args(outfile,yyv[yysp-2]^.p1^.p1^.p2);
         if createdynlib then
         begin
         loaddynlibproc.add('pointer('+yyv[yysp-2]^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+yyv[yysp-2]^.p1^.p2^.p+''');');
         freedynlibproc.add(yyv[yysp-2]^.p1^.p2^.p+':=nil;');
         end
         else if not IsExtern then
         begin
         write(implemfile,'procedure ',yyv[yysp-2]^.p1^.p2^.p);
         if assigned(yyv[yysp-2]^.p1^.p1^.p2) then
         write_args(implemfile,yyv[yysp-2]^.p1^.p1^.p2);
         end;
         end
         else
         begin
         if createdynlib then
         begin
         write(outfile,yyv[yysp-2]^.p1^.p2^.p,' : function');
         end
         else
         begin
         shift(9);
         write(outfile,'function ',yyv[yysp-2]^.p1^.p2^.p);
         end;
         
         if assigned(yyv[yysp-2]^.p1^.p1^.p2) then
         write_args(outfile,yyv[yysp-2]^.p1^.p1^.p2);
         write(outfile,':');
         write_p_a_def(outfile,yyv[yysp-2]^.p1^.p1^.p1,yyv[yysp-4]);
         if createdynlib then
         begin
         loaddynlibproc.add('pointer('+yyv[yysp-2]^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+yyv[yysp-2]^.p1^.p2^.p+''');');
         freedynlibproc.add(yyv[yysp-2]^.p1^.p2^.p+':=nil;');
         end
         else if not IsExtern then
         begin
         write(implemfile,'function ',yyv[yysp-2]^.p1^.p2^.p);
         if assigned(yyv[yysp-2]^.p1^.p1^.p2) then
         write_args(implemfile,yyv[yysp-2]^.p1^.p1^.p2);
         write(implemfile,':');
         
         old_in_args:=in_args;
         (* write pointers as P.... instead of ^.... *)
         in_args:=true;
         write_p_a_def(implemfile,yyv[yysp-2]^.p1^.p1^.p1,yyv[yysp-4]);
         in_args:=old_in_args;
         end;
         end;
         if assigned(yyv[yysp-1]) then
         write(outfile,';systrap ',yyv[yysp-1]^.p);
         (* No CDECL in interface for Uselib *)
         if IsExtern and (not no_pop) then
         write(outfile,';cdecl');
         popshift;
         if createdynlib then
         begin
         writeln(outfile,';');
         end
         else if UseLib then
         begin
         if IsExtern then
         begin
         write (outfile,';external');
         If UseName then
         Write(outfile,' External_library name ''',yyv[yysp-2]^.p1^.p2^.p,'''');
         end;
         writeln(outfile,';');
         end
         else
         begin
         writeln(outfile,';');
         if not IsExtern then
         begin
         writeln(implemfile,';');
         writeln(implemfile,aktspace,'begin');
         writeln(implemfile,aktspace,'  { You must implement this function }');
         writeln(implemfile,aktspace,'end;');
         end;
         end;
         IsExtern:=false;
         if not(compactmode) and not(createdynlib) then
         writeln(outfile);
         until not NeedEllipsisOverload;
         end
         else (* yyv[yysp-2]^.p1^.p1^.typ=t_procdef *)
         if assigned(yyv[yysp-2])and assigned(yyv[yysp-2]^.p1) then
         begin
         shift(2);
         if block_type<>bt_var then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'var');
         end;
         block_type:=bt_var;
         
         shift(2);
         
         IsExtern:=assigned(yyv[yysp-5])and(yyv[yysp-5]^.str='extern');
         (* walk through all declarations *)
         hp:=yyv[yysp-2];
         while assigned(hp) and assigned(hp^.p1) do
         begin
         (* write new var name *)
         if assigned(hp^.p1^.p2) and assigned(hp^.p1^.p2^.p) then
         write(outfile,aktspace,hp^.p1^.p2^.p);
         write(outfile,' : ');
         shift(2);
         (* write its type *)
         write_p_a_def(outfile,hp^.p1^.p1,yyv[yysp-4]);
         if assigned(hp^.p1^.p2)and assigned(hp^.p1^.p2^.p)then
         begin
         if isExtern then
         write(outfile,';cvar;external')
         else
         write(outfile,';cvar;public');
         end;
         writeln(outfile,';');
         popshift;
         hp:=hp^.p2;
         end;
         popshift;
         popshift;
         end;
         if assigned(yyv[yysp-5])then  dispose(yyv[yysp-5],done);
         if assigned(yyv[yysp-4])then  dispose(yyv[yysp-4],done);
         if assigned(yyv[yysp-2])then  dispose(yyv[yysp-2],done);
         
       end;
  30 : begin
         
         if block_type<>bt_type then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'type');
         block_type:=bt_type;
         end;
         shift(2);
         if ( yyv[yysp-1]^.p2  <> nil ) then
         begin
         (* write new type name *)
         TN:=TypeName(yyv[yysp-1]^.p2^.p);
         PN:=PointerName(yyv[yysp-1]^.p2^.p);
         (* define a Pointer type also for structs *)
         if UsePPointers and (Uppercase(tn)<>Uppercase(pn)) and
         assigned(yyv[yysp-1]) and (yyv[yysp-1]^.typ in [t_uniondef,t_structdef]) then
         writeln(outfile,aktspace,PN,' = ^',TN,';');
         write(outfile,aktspace,TN,' = ');
         shift(2);
         hp:=yyv[yysp-1];
         write_type_specifier(outfile,hp);
         popshift;
         (* enum_to_const can make a switch to const *)
         if block_type=bt_type then
         writeln(outfile,';');
         writeln(outfile);
         flush(outfile);
         popshift;
         if must_write_packed_field then
         write_packed_fields_info(outfile,hp,TN);
         if assigned(hp) then
         dispose(hp,done)
         end
         else
         begin
         TN:=TypeName(yyv[yysp-1]^.str);
         PN:=PointerName(yyv[yysp-1]^.str);
         if UsePPointers then writeln(outfile,aktspace,PN,' = ^',TN,';');
         if PackRecords then
         writeln(outfile, aktspace, TN, ' = packed record')
         else
         writeln(outfile, aktspace, TN, ' = record');
         writeln(outfile, aktspace, '    {undefined structure}');
         writeln(outfile, aktspace, '  end;');
         writeln(outfile);
         popshift;
         end;
         
       end;
  31 : begin
         
         (* TYPEDEF STRUCT dname dname SEMICOLON *)
         if block_type<>bt_type then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'type');
         block_type:=bt_type;
         end;
         PN:=TypeName(yyv[yysp-2]^.p);
         TN:=TypeName(yyv[yysp-1]^.p);
         if Uppercase(tn)<>Uppercase(pn) then
         begin
         shift(2);
         writeln(outfile,aktspace,PN,' = ',TN,';');
         popshift;
         end;
         if assigned(yyv[yysp-2]) then
         dispose(yyv[yysp-2],done);
         if assigned(yyv[yysp-1]) then
         dispose(yyv[yysp-1],done);
         
       end;
  32 : begin
         
         (* TYPEDEF type_specifier LKLAMMER dec_modifier declarator RKLAMMER maybe_space LKLAMMER argument_declaration_list RKLAMMER SEMICOLON *)
         if block_type<>bt_type then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'type');
         block_type:=bt_type;
         end;
         no_pop:=assigned(yyv[yysp-7]) and (yyv[yysp-7]^.str='no_pop');
         shift(2);
         (* walk through all declarations *)
         hp:=yyv[yysp-6];
         if assigned(hp) then
         begin
         hp:=yyv[yysp-6];
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_two(t_procdef,nil,yyv[yysp-2]));
         hp:=yyv[yysp-6];
         if assigned(hp^.p1) and assigned(hp^.p1^.p1) then
         begin
         writeln(outfile);
         (* write new type name *)
         write(outfile,aktspace,TypeName(hp^.p2^.p),' = ');
         shift(2);
         write_p_a_def(outfile,hp^.p1,yyv[yysp-9]);
         popshift;
         (* if no_pop it is normal fpc calling convention *)
         if is_procvar and
         (not no_pop) then
         write(outfile,';cdecl');
         writeln(outfile,';');
         flush(outfile);
         end;
         end;
         popshift;
         if assigned(yyv[yysp-9])then
         dispose(yyv[yysp-9],done);
         if assigned(yyv[yysp-7])then
         dispose(yyv[yysp-7],done);
         if assigned(yyv[yysp-6])then (* disposes also yyv[yysp-2] *)
         dispose(yyv[yysp-6],done);
         
       end;
  33 : begin
         
         (* TYPEDEF type_specifier dec_modifier declarator_list SEMICOLON *)
         if block_type<>bt_type then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'type');
         block_type:=bt_type;
         end
         else
         writeln(outfile);
         no_pop:=assigned(yyv[yysp-2]) and (yyv[yysp-2]^.str='no_pop');
         shift(2);
         (* Get the name to write the type definition for, try
         to use the tag name first *)
         if assigned(yyv[yysp-3]^.p2) then
         begin
         ph:=yyv[yysp-3]^.p2;
         end
         else
         begin
         if not assigned(yyv[yysp-1]^.p1^.p2) then
         internalerror(4444);
         ph:=yyv[yysp-1]^.p1^.p2;
         end;
         (* write type definition *)
         is_procvar:=false;
         TN:=TypeName(ph^.p);
         PN:=PointerName(ph^.p);
         if UsePPointers and (Uppercase(tn)<>Uppercase(pn)) and
         assigned(yyv[yysp-3]) and (yyv[yysp-3]^.typ<>t_procdef) then
         writeln(outfile,aktspace,PN,' = ^',TN,';');
         (* write new type name *)
         write(outfile,aktspace,TN,' = ');
         shift(2);
         write_p_a_def(outfile,yyv[yysp-1]^.p1^.p1,yyv[yysp-3]);
         popshift;
         (* if no_pop it is normal fpc calling convention *)
         if is_procvar and
         (not no_pop) then
         write(outfile,';cdecl');
         writeln(outfile,';');
         flush(outfile);
         (* write alias names, ph points to the name already used *)
         hp:=yyv[yysp-1];
         while assigned(hp) do
         begin
         if (hp<>ph) and assigned(hp^.p1^.p2) then
         begin
         PN:=TypeName(ph^.p);
         TN:=TypeName(hp^.p1^.p2^.p);
         if Uppercase(TN)<>Uppercase(PN) then
         begin
         write(outfile,aktspace,TN,' = ');
         write_p_a_def(outfile,hp^.p1^.p1,ph);
         writeln(outfile,';');
         PN:=PointerName(hp^.p1^.p2^.p);
         if UsePPointers and (Uppercase(tn)<>Uppercase(pn)) and
         assigned(yyv[yysp-3]) and (yyv[yysp-3]^.typ<>t_procdef) then
         writeln(outfile,aktspace,PN,' = ^',TN,';');
         end;
         end;
         hp:=hp^.next;
         end;
         popshift;
         if must_write_packed_field then
         if assigned(ph) then
         write_packed_fields_info(outfile,yyv[yysp-3],ph^.str)
         else if assigned(yyv[yysp-3]^.p2) then
         write_packed_fields_info(outfile,yyv[yysp-3],yyv[yysp-3]^.p2^.str);
         if assigned(yyv[yysp-3])then
         dispose(yyv[yysp-3],done);
         if assigned(yyv[yysp-2])then
         dispose(yyv[yysp-2],done);
         if assigned(yyv[yysp-1])then
         dispose(yyv[yysp-1],done);
         
       end;
  34 : begin
         
         if block_type<>bt_type then
         begin
         if not(compactmode) then
         writeln(outfile);
         writeln(outfile,aktspace,'type');
         block_type:=bt_type;
         end
         else
         writeln(outfile);
<<<<<<< HEAD
         shift(2);
=======
         shift(3);
>>>>>>> origin/fixes_2.4
         (* write as pointer *)
         writeln(outfile,'(* generic typedef  *)');
         writeln(outfile,aktspace,yyv[yysp-1]^.p,' = pointer;');
         flush(outfile);
         popshift;
         if assigned(yyv[yysp-1]) then
         dispose(yyv[yysp-1],done);
         
       end;
  35 : begin
         writeln(outfile,'in declaration at line ',line_no,' *)');
         aktspace:='';
         in_space_define:=0;
         in_define:=false;
         arglevel:=0;
         if_nb:=0;
         aktspace:='    ';
         space_index:=1;
         yyerrok;
       end;
  36 : begin
         
         (* DEFINE dname LKLAMMER enum_list RKLAMMER para_def_expr NEW_LINE *)
         if not stripinfo then
         begin
         writeln (outfile,aktspace,'{ was #define dname(params) para_def_expr }');
         writeln (implemfile,aktspace,'{ was #define dname(params) para_def_expr }');
         if assigned(yyv[yysp-3]) then
         begin
         writeln (outfile,aktspace,'{ argument types are unknown }');
         writeln (implemfile,aktspace,'{ argument types are unknown }');
         end;
         if not assigned(yyv[yysp-1]^.p3) then
         begin
         writeln(outfile,aktspace,'{ return type might be wrong }   ');
         writeln(implemfile,aktspace,'{ return type might be wrong }   ');
         end;
         end;
         if block_type<>bt_func then
         writeln(outfile);
         
         block_type:=bt_func;
         write(outfile,aktspace,'function ',yyv[yysp-5]^.p);
         write(implemfile,aktspace,'function ',yyv[yysp-5]^.p);
         
         if assigned(yyv[yysp-3]) then
         begin
         write(outfile,'(');
         write(implemfile,'(');
         ph:=new(presobject,init_one(t_enumdef,yyv[yysp-3]));
         write_def_params(outfile,ph);
         write_def_params(implemfile,ph);
         if assigned(ph) then dispose(ph,done);
         ph:=nil;
         (* types are unknown *)
         write(outfile,' : longint)');
         write(implemfile,' : longint)');
         end;
         if not assigned(yyv[yysp-1]^.p3) then
         begin
         writeln(outfile,' : longint;',aktspace,commentstr);
         writeln(implemfile,' : longint;');
         flush(outfile);
         end
         else
         begin
         write(outfile,' : ');
         write_type_specifier(outfile,yyv[yysp-1]^.p3);
         writeln(outfile,';',aktspace,commentstr);
         flush(outfile);
         write(implemfile,' : ');
         write_type_specifier(implemfile,yyv[yysp-1]^.p3);
         writeln(implemfile,';');
         end;
         writeln(outfile);
         flush(outfile);
         hp:=new(presobject,init_two(t_funcname,yyv[yysp-5],yyv[yysp-1]));
         write_funexpr(implemfile,hp);
         writeln(implemfile);
         flush(implemfile);
         if assigned(hp)then dispose(hp,done);
         
       end;
  37 : begin
         
         (* DEFINE dname SPACE_DEFINE NEW_LINE *)
         writeln(outfile,'{$define ',yyv[yysp-2]^.p,'}',aktspace,commentstr);
         flush(outfile);
         if assigned(yyv[yysp-2])then
         dispose(yyv[yysp-2],done);
         
       end;
  38 : begin
         
         writeln(outfile,'{$define ',yyv[yysp-1]^.p,'}',aktspace,commentstr);
         flush(outfile);
         if assigned(yyv[yysp-1])then
         dispose(yyv[yysp-1],done);
         
       end;
  39 : begin
         
         (* DEFINE dname SPACE_DEFINE def_expr NEW_LINE *)
         if (yyv[yysp-1]^.typ=t_exprlist) and
         yyv[yysp-1]^.p1^.is_const and
         not assigned(yyv[yysp-1]^.next) then
         begin
         if block_type<>bt_const then
         begin
         if block_type<>bt_func then
         writeln(outfile);
         writeln(outfile,aktspace,'const');
         end;
         block_type:=bt_const;
         shift(2);
         write(outfile,aktspace,yyv[yysp-3]^.p);
         write(outfile,' = ');
         flush(outfile);
         write_expr(outfile,yyv[yysp-1]^.p1);
         writeln(outfile,';',aktspace,commentstr);
         popshift;
         if assigned(yyv[yysp-3]) then
         dispose(yyv[yysp-3],done);
         if assigned(yyv[yysp-1]) then
         dispose(yyv[yysp-1],done);
         end
         else
         begin
         if block_type<>bt_func then
         writeln(outfile);
         if not stripinfo then
         begin
         writeln (outfile,aktspace,'{ was #define dname def_expr }');
         writeln (implemfile,aktspace,'{ was #define dname def_expr }');
         end;
         block_type:=bt_func;
         write(outfile,aktspace,'function ',yyv[yysp-3]^.p);
         write(implemfile,aktspace,'function ',yyv[yysp-3]^.p);
         shift(2);
         if not assigned(yyv[yysp-1]^.p3) then
         begin
         writeln(outfile,' : longint; { return type might be wrong }');
         flush(outfile);
         writeln(implemfile,' : longint; { return type might be wrong }');
         end
         else
         begin
         write(outfile,' : ');
         write_type_specifier(outfile,yyv[yysp-1]^.p3);
         writeln(outfile,';',aktspace,commentstr);
         flush(outfile);
         write(implemfile,' : ');
         write_type_specifier(implemfile,yyv[yysp-1]^.p3);
         writeln(implemfile,';');
         end;
         writeln(outfile);
         flush(outfile);
         hp:=new(presobject,init_two(t_funcname,yyv[yysp-3],yyv[yysp-1]));
         write_funexpr(implemfile,hp);
         popshift;
         dispose(hp,done);
         writeln(implemfile);
         flush(implemfile);
         end;
         
       end;
  40 : begin
         writeln(outfile,'in define line ',line_no,' *)');
         aktspace:='';
         in_space_define:=0;
         in_define:=false;
         arglevel:=0;
         if_nb:=0;
         aktspace:='    ';
         space_index:=1;
         
         yyerrok;
       end;
  41 : begin
         yyval:=yyv[yysp-1];
       end;
  42 : begin
         writeln(outfile,' in member_list *)');
         yyerrok;
         yyval:=nil;
         
       end;
  43 : begin
         yyval:=yyv[yysp-1];
       end;
  44 : begin
         writeln(outfile,' in enum_list *)');
         yyerrok;
         yyval:=nil;
         
       end;
  45 : begin
         
         if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
         is_packed:=true;
         yyval:=new(presobject,init_two(t_structdef,yyv[yysp-1],yyv[yysp-2]));
         
       end;
  46 : begin
         
         if (is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 4}');
         is_packed:=false;
         yyval:=new(presobject,init_two(t_structdef,yyv[yysp-0],yyv[yysp-1]));
         
       end;
  47 : begin
         
         if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
         is_packed:=true;
         yyval:=new(presobject,init_two(t_uniondef,yyv[yysp-1],yyv[yysp-2]));
         
       end;
  48 : begin
         
         yyval:=new(presobject,init_two(t_uniondef,yyv[yysp-0],yyv[yysp-1]));
         
       end;
  49 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
  50 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
  51 : begin
         
         yyval:=new(presobject,init_two(t_enumdef,yyv[yysp-0],yyv[yysp-1]));
         
       end;
  52 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
  53 : begin
         
         if not stripinfo then
         writeln(outfile,'(* Const before type ignored *)');
         yyval:=yyv[yysp-0];
         
       end;
  54 : begin
         
         if (not is_packed) and (not packrecords)then
         writeln(outfile,'{$PACKRECORDS 1}');
         is_packed:=true;
         yyval:=new(presobject,init_one(t_uniondef,yyv[yysp-1]));
         
       end;
  55 : begin
         
         yyval:=new(presobject,init_one(t_uniondef,yyv[yysp-0]));
         
       end;
  56 : begin
         
         if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
         is_packed:=true;
         yyval:=new(presobject,init_one(t_structdef,yyv[yysp-1]));
         
       end;
  57 : begin
         
         if (is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 4}');
         is_packed:=false;
         yyval:=new(presobject,init_one(t_structdef,yyv[yysp-0]));
         
       end;
  58 : begin
         
         yyval:=new(presobject,init_one(t_enumdef,yyv[yysp-0]));
         
       end;
  59 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
  60 : begin
         yyval:=yyv[yysp-0]; 
       end;
  61 : begin
         
         yyval:=new(presobject,init_one(t_memberdeclist,yyv[yysp-1]));
         yyval^.next:=yyv[yysp-0];
         
       end;
  62 : begin
         
         yyval:=new(presobject,init_one(t_memberdeclist,yyv[yysp-0]));
         
       end;
  63 : begin
         
         yyval:=new(presobject,init_two(t_memberdec,yyv[yysp-2],yyv[yysp-1]));
         
       end;
  64 : begin
         (*dname*)
         yyval:=new(presobject,init_id(act_token));
         
       end;
  65 : begin
         
         hp:=yyv[yysp-0];
         yyval:=hp;
         if assigned(hp) then
         begin
         s:=strpas(hp^.p);
         if UseCTypesUnit then
         begin
         if s=cint_STR then
         s:=csint_STR
         else if s=cshort_STR then
         s:=csshort_STR
         else if s=cchar_STR then
         s:=cschar_STR
         else if s=clong_STR then
         s:=cslong_STR
         else if s=clonglong_STR then
         s:=cslonglong_STR
         else if s=cint8_STR then
         s:=cint8_STR
         else if s=cint16_STR then
         s:=cint16_STR
         else if s=cint32_STR then
         s:=cint32_STR
         else if s=cint64_STR then
         s:=cint64_STR
         else
         s:='';
         end
         else
         begin
         if s=UINT_STR then
         s:=INT_STR
         else if s=USHORT_STR then
         s:=SHORT_STR
         else if s=USMALL_STR then
         s:=SMALL_STR
         else if s=UCHAR_STR then
         s:=CHAR_STR
         else if s=QWORD_STR then
         s:=INT64_STR
         else
         s:='';
         end;
         if s<>'' then
         hp^.setstr(s);
         end;
         
       end;
  66 : begin
         
         hp:=yyv[yysp-0];
         yyval:=hp;
         if assigned(hp) then
         begin
         s:=strpas(hp^.p);
         if UseCTypesUnit then
         begin
         if s=cint_STR then
         s:=cuint_STR
         else if s=cshort_STR then
         s:=cushort_STR
         else if s=cchar_STR then
         s:=cuchar_STR
         else if s=clong_STR then
         s:=culong_STR
         else if s=clonglong_STR then
         s:=culonglong_STR
         else if s=cint8_STR then
         s:=cuint8_STR
         else if s=cint16_STR then
         s:=cuint16_STR
         else if s=cint32_STR then
         s:=cuint32_STR
         else if s=cint64_STR then
         s:=cuint64_STR
         else
         s:='';
         end
         else
         begin
         if s=INT_STR then
         s:=UINT_STR
         else if s=SHORT_STR then
         s:=USHORT_STR
         else if s=SMALL_STR then
         s:=USMALL_STR
         else if s=CHAR_STR then
         s:=UCHAR_STR
         else if s=INT64_STR then
         s:=QWORD_STR
         else
         s:='';
         end;
         if s<>'' then
         hp^.setstr(s);
         end;
         
       end;
  67 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cint_STR))
         else
         yyval:=new(presobject,init_intid(INT_STR));
         
       end;
  68 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(clong_STR))
         else
         yyval:=new(presobject,init_intid(INT_STR));
         
       end;
  69 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(clong_STR))
         else
         yyval:=new(presobject,init_intid(INT_STR));
         
       end;
  70 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(clonglong_STR))
         else
         yyval:=new(presobject,init_intid(INT64_STR));
         
       end;
  71 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(clonglong_STR))
         else
         yyval:=new(presobject,init_intid(INT64_STR));
         
       end;
  72 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cshort_STR))
         else
         yyval:=new(presobject,init_intid(SMALL_STR));
         
       end;
  73 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cshort_STR))
         else
         yyval:=new(presobject,init_intid(SMALL_STR));
         
       end;
  74 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cint8_STR))
         else
         yyval:=new(presobject,init_intid(SHORT_STR));
         
       end;
  75 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cint16_STR))
         else
         yyval:=new(presobject,init_intid(SMALL_STR));
         
       end;
  76 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cint32_STR))
         else
         yyval:=new(presobject,init_intid(INT_STR));
         
       end;
  77 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cint64_STR))
         else
         yyval:=new(presobject,init_intid(INT64_STR));
         
       end;
  78 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cfloat_STR))
         else
         yyval:=new(presobject,init_intid(FLOAT_STR));
         
       end;
  79 : begin
         
         yyval:=new(presobject,init_no(t_void));
         
       end;
  80 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cchar_STR))
         else
         yyval:=new(presobject,init_intid(CHAR_STR));
         
       end;
  81 : begin
         
         if UseCTypesUnit then
         yyval:=new(presobject,init_id(cunsigned_STR))
         else
         yyval:=new(presobject,init_intid(UINT_STR));
         
       end;
  82 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
  83 : begin
         
         yyval:=yyv[yysp-0];
         tn:=yyval^.str;
         if removeunderscore and
         (length(tn)>1) and (tn[1]='_') then
         yyval^.setstr(Copy(tn,2,length(tn)-1));
         
       end;
  84 : begin
         
         yyval:=yyv[yysp-2];
         hp:=yyv[yysp-2];
         while assigned(hp^.next) do
         hp:=hp^.next;
         hp^.next:=new(presobject,init_one(t_declist,yyv[yysp-0]));
         
       end;
  85 : begin
         
         writeln(outfile,' in declarator_list *)');
         yyval:=yyv[yysp-0];
         yyerrok;
         
       end;
  86 : begin
         
         writeln(outfile,' in declarator_list *)');
         yyerrok;
         
       end;
  87 : begin
         
         yyval:=new(presobject,init_one(t_declist,yyv[yysp-0]));
         
       end;
  88 : begin
         
         yyval:=new(presobject,init_two(t_arg,yyv[yysp-1],yyv[yysp-0]));
         
       end;
  89 : begin
         
         (* type_specifier STAR declarator *)
         hp:=new(presobject,init_one(t_pointerdef,yyv[yysp-2]));
         yyval:=new(presobject,init_two(t_arg,hp,yyv[yysp-0]));
         
       end;
  90 : begin
         
         yyval:=new(presobject,init_two(t_arg,yyv[yysp-1],yyv[yysp-0]));
         
       end;
  91 : begin
         
         yyval:=new(presobject,init_two(t_arglist,yyv[yysp-0],nil));
         
       end;
  92 : begin
         
         yyval:=new(presobject,init_two(t_arglist,yyv[yysp-2],nil));
         yyval^.next:=yyv[yysp-0];
         
       end;
  93 : begin
         
         yyval:=new(presobject,init_two(t_arglist,ellipsisarg,nil));
         
       end;
  94 : begin
         
         yyval:=nil;
         
       end;
  95 : begin
         yyval:=new(presobject,init_id('far'));
       end;
  96 : begin
         yyval:=new(presobject,init_id('near'));
       end;
  97 : begin
         yyval:=new(presobject,init_id('huge'));
       end;
  98 : begin
         
         if not stripinfo then
         writeln(outfile,'(* Const before declarator ignored *)');
         yyval:=yyv[yysp-0];
         
       end;
  99 : begin
         
         if not stripinfo then
         writeln(outfile,aktspace,'(* ',yyv[yysp-2]^.p,' ignored *)');
         dispose(yyv[yysp-2],done);
         hp:=yyv[yysp-0];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
         
       end;
 100 : begin
         
         (* %prec PSTAR this was wrong!! *)
         hp:=yyv[yysp-0];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
         
       end;
 101 : begin
         
         hp:=yyv[yysp-0];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_one(t_addrdef,nil));
         
       end;
 102 : begin
         
         (*  size specifier supported *)
         hp:=new(presobject,init_one(t_size_specifier,yyv[yysp-0]));
         yyval:=new(presobject,init_three(t_dec,nil,yyv[yysp-2],hp));
         
       end;
 103 : begin
         
         if not stripinfo then
         writeln(outfile,'(* Warning : default value for ',yyv[yysp-2]^.p,' ignored *)');
         hp:=new(presobject,init_one(t_default_value,yyv[yysp-0]));
         yyval:=new(presobject,init_three(t_dec,nil,yyv[yysp-2],hp));
         
       end;
 104 : begin
         
         yyval:=new(presobject,init_two(t_dec,nil,yyv[yysp-0]));
         
       end;
 105 : begin
         
         hp:=yyv[yysp-3];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_two(t_procdef,nil,yyv[yysp-1]));
         
       end;
 106 : begin
         
         hp:=yyv[yysp-1];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_two(t_procdef,nil,nil));
         
       end;
 107 : begin
         
         hp:=yyv[yysp-3];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_two(t_arraydef,nil,yyv[yysp-1]));
         
       end;
 108 : begin
         
         (* this is translated into a pointer *)
         hp:=yyv[yysp-2];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
         
       end;
 109 : begin
         
         yyval:=yyv[yysp-1];
         
       end;
 110 : begin
         yyval := yyv[yysp-1];
       end;
 111 : begin
         yyval := yyv[yysp-2];
       end;
 112 : begin
         
         if not stripinfo then
         writeln(outfile,'(* Const before abstract_declarator ignored *)');
         yyval:=yyv[yysp-0];
         
       end;
 113 : begin
         
         if not stripinfo then
         writeln(outfile,aktspace,'(* ',yyv[yysp-2]^.p,' ignored *)');
         dispose(yyv[yysp-2],done);
         hp:=yyv[yysp-0];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
         
       end;
 114 : begin
         
         hp:=yyv[yysp-0];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
         
       end;
 115 : begin
         
         hp:=yyv[yysp-3];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_two(t_procdef,nil,yyv[yysp-1]));
         
       end;
 116 : begin
         
         hp:=yyv[yysp-1];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_two(t_procdef,nil,nil));
         
       end;
 117 : begin
         
         hp:=yyv[yysp-3];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_two(t_arraydef,nil,yyv[yysp-1]));
         
       end;
 118 : begin
         
         (* this is translated into a pointer *)
         hp:=yyv[yysp-2];
         yyval:=hp;
         while assigned(hp^.p1) do
         hp:=hp^.p1;
         hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
         
       end;
 119 : begin
         
         yyval:=yyv[yysp-1];
         
       end;
 120 : begin
         
         yyval:=new(presobject,init_two(t_dec,nil,nil));
         
       end;
 121 : begin
         yyval:=yyv[yysp-0]; 
       end;
 122 : begin
         yyval:=new(presobject,init_bop(':=',yyv[yysp-2],yyv[yysp-0])); 
       end;
 123 : begin
         yyval:=new(presobject,init_bop('=',yyv[yysp-2],yyv[yysp-0]));
       end;
 124 : begin
         yyval:=new(presobject,init_bop('<>',yyv[yysp-2],yyv[yysp-0]));
       end;
 125 : begin
         yyval:=new(presobject,init_bop('>',yyv[yysp-2],yyv[yysp-0]));
       end;
 126 : begin
         yyval:=new(presobject,init_bop('>=',yyv[yysp-2],yyv[yysp-0]));
       end;
 127 : begin
         yyval:=new(presobject,init_bop('<',yyv[yysp-2],yyv[yysp-0]));
       end;
 128 : begin
         yyval:=new(presobject,init_bop('<=',yyv[yysp-2],yyv[yysp-0]));
       end;
 129 : begin
         yyval:=new(presobject,init_bop('+',yyv[yysp-2],yyv[yysp-0]));
       end;
 130 : begin
         yyval:=new(presobject,init_bop('-',yyv[yysp-2],yyv[yysp-0]));
       end;
 131 : begin
         yyval:=new(presobject,init_bop('*',yyv[yysp-2],yyv[yysp-0]));
       end;
 132 : begin
         yyval:=new(presobject,init_bop('/',yyv[yysp-2],yyv[yysp-0]));
       end;
 133 : begin
         yyval:=new(presobject,init_bop(' or ',yyv[yysp-2],yyv[yysp-0]));
       end;
 134 : begin
         yyval:=new(presobject,init_bop(' and ',yyv[yysp-2],yyv[yysp-0]));
       end;
 135 : begin
         yyval:=new(presobject,init_bop(' not ',yyv[yysp-2],yyv[yysp-0]));
       end;
 136 : begin
         yyval:=new(presobject,init_bop(' shl ',yyv[yysp-2],yyv[yysp-0]));
       end;
 137 : begin
         yyval:=new(presobject,init_bop(' shr ',yyv[yysp-2],yyv[yysp-0]));
       end;
 138 : begin
         
         yyv[yysp-0]^.p1:=yyv[yysp-2];
         yyval:=yyv[yysp-0];
         inc(if_nb);
         yyval^.p:=strpnew('if_local'+str(if_nb));
         
       end;
 139 : begin
         yyval:=yyv[yysp-0];
       end;
 140 : begin
         (* if A then B else C *)
         yyval:=new(presobject,init_three(t_ifexpr,nil,yyv[yysp-2],yyv[yysp-0]));
       end;
 141 : begin
         yyval:=yyv[yysp-0]; 
       end;
 142 : begin
         yyval:=nil;
       end;
 143 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
 144 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
 145 : begin
         
         (* remove L prefix for widestrings *)
         s:=act_token;
         if Win32headers and (s[1]='L') then
         delete(s,1,1);
         yyval:=new(presobject,init_id(''''+copy(s,2,length(s)-2)+''''));
         
       end;
 146 : begin
         
         yyval:=new(presobject,init_id(act_token));
         
       end;
 147 : begin
         
         yyval:=new(presobject,init_bop('.',yyv[yysp-2],yyv[yysp-0]));
         
       end;
 148 : begin
         
         yyval:=new(presobject,init_bop('^.',yyv[yysp-2],yyv[yysp-0]));
         
       end;
 149 : begin
         
         yyval:=new(presobject,init_preop('-',yyv[yysp-0]));
         
       end;
 150 : begin
         
         yyval:=new(presobject,init_preop('+',yyv[yysp-0]));
         
       end;
 151 : begin
         
         yyval:=new(presobject,init_preop('@',yyv[yysp-0]));
         
       end;
 152 : begin
         
         yyval:=new(presobject,init_preop(' not ',yyv[yysp-0]));
         
       end;
 153 : begin
         
         if assigned(yyv[yysp-0]) then
         yyval:=new(presobject,init_two(t_typespec,yyv[yysp-2],yyv[yysp-0]))
         else
         yyval:=yyv[yysp-2];
         
       end;
 154 : begin
         
         yyval:=new(presobject,init_two(t_typespec,yyv[yysp-2],yyv[yysp-0]));
         
       end;
 155 : begin
         
         hp:=new(presobject,init_one(t_pointerdef,yyv[yysp-3]));
         yyval:=new(presobject,init_two(t_typespec,hp,yyv[yysp-0]));
         
       end;
 156 : begin
         
         if not stripinfo then
         writeln(outfile,aktspace,'(* ',yyv[yysp-3]^.p,' ignored *)');
         dispose(yyv[yysp-3],done);
         write_type_specifier(outfile,yyv[yysp-4]);
         writeln(outfile,' ignored *)');
         hp:=new(presobject,init_one(t_pointerdef,yyv[yysp-4]));
         yyval:=new(presobject,init_two(t_typespec,hp,yyv[yysp-0]));
         
       end;
 157 : begin
         
         hp:=new(presobject,init_one(t_exprlist,yyv[yysp-3]));
         yyval:=new(presobject,init_three(t_funexprlist,hp,yyv[yysp-1],nil));
         
       end;
 158 : begin
         
         yyval:=yyv[yysp-1];
         
       end;
 159 : begin
         
         yyval:=new(presobject,init_two(t_callop,yyv[yysp-5],yyv[yysp-1]));
         
       end;
 160 : begin
         
         yyval:=new(presobject,init_two(t_arrayop,yyv[yysp-3],yyv[yysp-1]));
         
       end;
 161 : begin
         (*enum_element COMMA enum_list *)
         yyval:=yyv[yysp-2];
         yyval^.next:=yyv[yysp-0];
         
       end;
 162 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
 163 : begin
         (* empty enum list *)
         yyval:=nil;
       end;
 164 : begin
         begin (*enum_element: dname _ASSIGN expr *)
         yyval:=new(presobject,init_two(t_enumlist,yyv[yysp-2],yyv[yysp-0]));
         end;
         
       end;
 165 : begin
         
         begin (*enum_element: dname*)
         yyval:=new(presobject,init_two(t_enumlist,yyv[yysp-0],nil));
         end;
         
       end;
 166 : begin
         
         if yyv[yysp-0]^.typ=t_funexprlist then
         yyval:=yyv[yysp-0]
         else
         yyval:=new(presobject,init_two(t_exprlist,yyv[yysp-0],nil));
         (* if here is a type specifier
         we know the return type *)
         if (yyv[yysp-0]^.typ=t_typespec) then
         yyval^.p3:=yyv[yysp-0]^.p1^.get_copy;
         
       end;
 167 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
 168 : begin
         
         yyval:=yyv[yysp-1]
         
       end;
 169 : begin
         (*exprlist COMMA expr*)
         yyval:=yyv[yysp-2];
         yyv[yysp-2]^.next:=yyv[yysp-0];
         
       end;
 170 : begin
         
         yyval:=yyv[yysp-0];
         
       end;
 171 : begin
         (* empty expression list *)
         yyval:=nil; 
       end;
 172 : begin
         
         yyval:=new(presobject,init_one(t_exprlist,yyv[yysp-0]));
         
       end;
  end;
end(*yyaction*);

(* parse table: *)

type YYARec = record
                sym, act : Integer;
              end;
     YYRRec = record
                len, sym : Integer;
              end;

const

yynacts   = 3055;
yyngotos  = 424;
yynstates = 313;
yynrules  = 172;

yya : array [1..yynacts] of YYARec = (
{ 0: }
  ( sym: 256; act: 7 ),
  ( sym: 263; act: 8 ),
  ( sym: 264; act: 9 ),
  ( sym: 274; act: 10 ),
  ( sym: 275; act: 11 ),
  ( sym: 276; act: 12 ),
  ( sym: 293; act: 13 ),
  ( sym: 277; act: -10 ),
  ( sym: 280; act: -10 ),
  ( sym: 281; act: -10 ),
  ( sym: 282; act: -10 ),
  ( sym: 283; act: -10 ),
  ( sym: 284; act: -10 ),
  ( sym: 285; act: -10 ),
  ( sym: 286; act: -10 ),
  ( sym: 287; act: -10 ),
  ( sym: 327; act: -10 ),
  ( sym: 328; act: -10 ),
  ( sym: 329; act: -10 ),
  ( sym: 330; act: -10 ),
  ( sym: 331; act: -10 ),
{ 1: }
  ( sym: 266; act: 14 ),
{ 2: }
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 287; act: 31 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 3: }
{ 4: }
{ 5: }
  ( sym: 256; act: 7 ),
  ( sym: 263; act: 8 ),
  ( sym: 264; act: 9 ),
  ( sym: 274; act: 10 ),
  ( sym: 275; act: 11 ),
  ( sym: 276; act: 12 ),
  ( sym: 293; act: 13 ),
  ( sym: 0; act: -1 ),
  ( sym: 277; act: -10 ),
  ( sym: 280; act: -10 ),
  ( sym: 281; act: -10 ),
  ( sym: 282; act: -10 ),
  ( sym: 283; act: -10 ),
  ( sym: 284; act: -10 ),
  ( sym: 285; act: -10 ),
  ( sym: 286; act: -10 ),
  ( sym: 287; act: -10 ),
  ( sym: 327; act: -10 ),
  ( sym: 328; act: -10 ),
  ( sym: 329; act: -10 ),
  ( sym: 330; act: -10 ),
  ( sym: 331; act: -10 ),
{ 6: }
  ( sym: 0; act: 0 ),
{ 7: }
{ 8: }
  ( sym: 274; act: 42 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 287; act: 31 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 9: }
  ( sym: 277; act: 23 ),
{ 10: }
  ( sym: 277; act: 23 ),
{ 11: }
  ( sym: 277; act: 23 ),
{ 12: }
  ( sym: 277; act: 23 ),
{ 13: }
{ 14: }
{ 15: }
{ 16: }
{ 17: }
{ 18: }
  ( sym: 294; act: 48 ),
  ( sym: 295; act: 49 ),
  ( sym: 296; act: 50 ),
  ( sym: 297; act: 51 ),
  ( sym: 298; act: 52 ),
  ( sym: 299; act: 53 ),
  ( sym: 300; act: 54 ),
  ( sym: 256; act: -18 ),
  ( sym: 268; act: -18 ),
  ( sym: 277; act: -18 ),
  ( sym: 287; act: -18 ),
  ( sym: 288; act: -18 ),
  ( sym: 289; act: -18 ),
  ( sym: 290; act: -18 ),
  ( sym: 314; act: -18 ),
  ( sym: 319; act: -18 ),
{ 19: }
{ 20: }
  ( sym: 256; act: 56 ),
  ( sym: 272; act: 57 ),
  ( sym: 277; act: 23 ),
{ 21: }
  ( sym: 256; act: 56 ),
  ( sym: 272; act: 57 ),
  ( sym: 277; act: 23 ),
{ 22: }
  ( sym: 256; act: 60 ),
  ( sym: 272; act: 61 ),
  ( sym: 277; act: 23 ),
{ 23: }
{ 24: }
  ( sym: 283; act: 62 ),
  ( sym: 256; act: -72 ),
  ( sym: 265; act: -72 ),
  ( sym: 266; act: -72 ),
  ( sym: 267; act: -72 ),
  ( sym: 268; act: -72 ),
  ( sym: 269; act: -72 ),
  ( sym: 270; act: -72 ),
  ( sym: 271; act: -72 ),
  ( sym: 272; act: -72 ),
  ( sym: 273; act: -72 ),
  ( sym: 277; act: -72 ),
  ( sym: 287; act: -72 ),
  ( sym: 288; act: -72 ),
  ( sym: 289; act: -72 ),
  ( sym: 290; act: -72 ),
  ( sym: 291; act: -72 ),
  ( sym: 294; act: -72 ),
  ( sym: 295; act: -72 ),
  ( sym: 296; act: -72 ),
  ( sym: 297; act: -72 ),
  ( sym: 298; act: -72 ),
  ( sym: 299; act: -72 ),
  ( sym: 300; act: -72 ),
  ( sym: 301; act: -72 ),
  ( sym: 304; act: -72 ),
  ( sym: 306; act: -72 ),
  ( sym: 307; act: -72 ),
  ( sym: 308; act: -72 ),
  ( sym: 309; act: -72 ),
  ( sym: 310; act: -72 ),
  ( sym: 311; act: -72 ),
  ( sym: 312; act: -72 ),
  ( sym: 313; act: -72 ),
  ( sym: 314; act: -72 ),
  ( sym: 315; act: -72 ),
  ( sym: 316; act: -72 ),
  ( sym: 317; act: -72 ),
  ( sym: 318; act: -72 ),
  ( sym: 319; act: -72 ),
  ( sym: 320; act: -72 ),
  ( sym: 321; act: -72 ),
  ( sym: 324; act: -72 ),
  ( sym: 325; act: -72 ),
{ 25: }
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 256; act: -81 ),
  ( sym: 265; act: -81 ),
  ( sym: 266; act: -81 ),
  ( sym: 267; act: -81 ),
  ( sym: 268; act: -81 ),
  ( sym: 269; act: -81 ),
  ( sym: 270; act: -81 ),
  ( sym: 271; act: -81 ),
  ( sym: 272; act: -81 ),
  ( sym: 273; act: -81 ),
  ( sym: 277; act: -81 ),
  ( sym: 287; act: -81 ),
  ( sym: 288; act: -81 ),
  ( sym: 289; act: -81 ),
  ( sym: 290; act: -81 ),
  ( sym: 291; act: -81 ),
  ( sym: 294; act: -81 ),
  ( sym: 295; act: -81 ),
  ( sym: 296; act: -81 ),
  ( sym: 297; act: -81 ),
  ( sym: 298; act: -81 ),
  ( sym: 299; act: -81 ),
  ( sym: 300; act: -81 ),
  ( sym: 301; act: -81 ),
  ( sym: 304; act: -81 ),
  ( sym: 306; act: -81 ),
  ( sym: 307; act: -81 ),
  ( sym: 308; act: -81 ),
  ( sym: 309; act: -81 ),
  ( sym: 310; act: -81 ),
  ( sym: 311; act: -81 ),
  ( sym: 312; act: -81 ),
  ( sym: 313; act: -81 ),
  ( sym: 314; act: -81 ),
  ( sym: 315; act: -81 ),
  ( sym: 316; act: -81 ),
  ( sym: 317; act: -81 ),
  ( sym: 318; act: -81 ),
  ( sym: 319; act: -81 ),
  ( sym: 320; act: -81 ),
  ( sym: 321; act: -81 ),
  ( sym: 324; act: -81 ),
  ( sym: 325; act: -81 ),
{ 26: }
  ( sym: 282; act: 64 ),
  ( sym: 283; act: 65 ),
  ( sym: 256; act: -68 ),
  ( sym: 265; act: -68 ),
  ( sym: 266; act: -68 ),
  ( sym: 267; act: -68 ),
  ( sym: 268; act: -68 ),
  ( sym: 269; act: -68 ),
  ( sym: 270; act: -68 ),
  ( sym: 271; act: -68 ),
  ( sym: 272; act: -68 ),
  ( sym: 273; act: -68 ),
  ( sym: 277; act: -68 ),
  ( sym: 287; act: -68 ),
  ( sym: 288; act: -68 ),
  ( sym: 289; act: -68 ),
  ( sym: 290; act: -68 ),
  ( sym: 291; act: -68 ),
  ( sym: 294; act: -68 ),
  ( sym: 295; act: -68 ),
  ( sym: 296; act: -68 ),
  ( sym: 297; act: -68 ),
  ( sym: 298; act: -68 ),
  ( sym: 299; act: -68 ),
  ( sym: 300; act: -68 ),
  ( sym: 301; act: -68 ),
  ( sym: 304; act: -68 ),
  ( sym: 306; act: -68 ),
  ( sym: 307; act: -68 ),
  ( sym: 308; act: -68 ),
  ( sym: 309; act: -68 ),
  ( sym: 310; act: -68 ),
  ( sym: 311; act: -68 ),
  ( sym: 312; act: -68 ),
  ( sym: 313; act: -68 ),
  ( sym: 314; act: -68 ),
  ( sym: 315; act: -68 ),
  ( sym: 316; act: -68 ),
  ( sym: 317; act: -68 ),
  ( sym: 318; act: -68 ),
  ( sym: 319; act: -68 ),
  ( sym: 320; act: -68 ),
  ( sym: 321; act: -68 ),
  ( sym: 324; act: -68 ),
  ( sym: 325; act: -68 ),
{ 27: }
{ 28: }
{ 29: }
{ 30: }
{ 31: }
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 287; act: 31 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 32: }
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 33: }
{ 34: }
{ 35: }
{ 36: }
{ 37: }
{ 38: }
{ 39: }
  ( sym: 266; act: 68 ),
  ( sym: 291; act: 69 ),
{ 40: }
  ( sym: 268; act: 71 ),
  ( sym: 294; act: 48 ),
  ( sym: 295; act: 49 ),
  ( sym: 296; act: 50 ),
  ( sym: 297; act: 51 ),
  ( sym: 298; act: 52 ),
  ( sym: 299; act: 53 ),
  ( sym: 300; act: 54 ),
  ( sym: 256; act: -18 ),
  ( sym: 277; act: -18 ),
  ( sym: 287; act: -18 ),
  ( sym: 288; act: -18 ),
  ( sym: 289; act: -18 ),
  ( sym: 290; act: -18 ),
  ( sym: 314; act: -18 ),
  ( sym: 319; act: -18 ),
{ 41: }
  ( sym: 266; act: 72 ),
  ( sym: 256; act: -83 ),
  ( sym: 268; act: -83 ),
  ( sym: 277; act: -83 ),
  ( sym: 287; act: -83 ),
  ( sym: 288; act: -83 ),
  ( sym: 289; act: -83 ),
  ( sym: 290; act: -83 ),
  ( sym: 294; act: -83 ),
  ( sym: 295; act: -83 ),
  ( sym: 296; act: -83 ),
  ( sym: 297; act: -83 ),
  ( sym: 298; act: -83 ),
  ( sym: 299; act: -83 ),
  ( sym: 300; act: -83 ),
  ( sym: 314; act: -83 ),
  ( sym: 319; act: -83 ),
{ 42: }
  ( sym: 256; act: 56 ),
  ( sym: 272; act: 57 ),
  ( sym: 277; act: 23 ),
{ 43: }
  ( sym: 268; act: 74 ),
  ( sym: 291; act: 75 ),
  ( sym: 292; act: 76 ),
{ 44: }
  ( sym: 256; act: 56 ),
  ( sym: 272; act: 57 ),
  ( sym: 266; act: -50 ),
  ( sym: 267; act: -50 ),
  ( sym: 268; act: -50 ),
  ( sym: 269; act: -50 ),
  ( sym: 270; act: -50 ),
  ( sym: 277; act: -50 ),
  ( sym: 287; act: -50 ),
  ( sym: 288; act: -50 ),
  ( sym: 289; act: -50 ),
  ( sym: 290; act: -50 ),
  ( sym: 294; act: -50 ),
  ( sym: 295; act: -50 ),
  ( sym: 296; act: -50 ),
  ( sym: 297; act: -50 ),
  ( sym: 298; act: -50 ),
  ( sym: 299; act: -50 ),
  ( sym: 300; act: -50 ),
  ( sym: 314; act: -50 ),
  ( sym: 319; act: -50 ),
{ 45: }
  ( sym: 256; act: 56 ),
  ( sym: 272; act: 57 ),
  ( sym: 266; act: -49 ),
  ( sym: 267; act: -49 ),
  ( sym: 268; act: -49 ),
  ( sym: 269; act: -49 ),
  ( sym: 270; act: -49 ),
  ( sym: 277; act: -49 ),
  ( sym: 287; act: -49 ),
  ( sym: 288; act: -49 ),
  ( sym: 289; act: -49 ),
  ( sym: 290; act: -49 ),
  ( sym: 294; act: -49 ),
  ( sym: 295; act: -49 ),
  ( sym: 296; act: -49 ),
  ( sym: 297; act: -49 ),
  ( sym: 298; act: -49 ),
  ( sym: 299; act: -49 ),
  ( sym: 300; act: -49 ),
  ( sym: 314; act: -49 ),
  ( sym: 319; act: -49 ),
{ 46: }
  ( sym: 256; act: 60 ),
  ( sym: 272; act: 61 ),
  ( sym: 266; act: -52 ),
  ( sym: 267; act: -52 ),
  ( sym: 268; act: -52 ),
  ( sym: 269; act: -52 ),
  ( sym: 270; act: -52 ),
  ( sym: 277; act: -52 ),
  ( sym: 287; act: -52 ),
  ( sym: 288; act: -52 ),
  ( sym: 289; act: -52 ),
  ( sym: 290; act: -52 ),
  ( sym: 294; act: -52 ),
  ( sym: 295; act: -52 ),
  ( sym: 296; act: -52 ),
  ( sym: 297; act: -52 ),
  ( sym: 298; act: -52 ),
  ( sym: 299; act: -52 ),
  ( sym: 300; act: -52 ),
  ( sym: 314; act: -52 ),
  ( sym: 319; act: -52 ),
{ 47: }
  ( sym: 256; act: 84 ),
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 48: }
{ 49: }
{ 50: }
{ 51: }
{ 52: }
{ 53: }
{ 54: }
{ 55: }
  ( sym: 302; act: 92 ),
  ( sym: 256; act: -57 ),
  ( sym: 267; act: -57 ),
  ( sym: 268; act: -57 ),
  ( sym: 269; act: -57 ),
  ( sym: 270; act: -57 ),
  ( sym: 277; act: -57 ),
  ( sym: 287; act: -57 ),
  ( sym: 288; act: -57 ),
  ( sym: 289; act: -57 ),
  ( sym: 290; act: -57 ),
  ( sym: 294; act: -57 ),
  ( sym: 295; act: -57 ),
  ( sym: 296; act: -57 ),
  ( sym: 297; act: -57 ),
  ( sym: 298; act: -57 ),
  ( sym: 299; act: -57 ),
  ( sym: 300; act: -57 ),
  ( sym: 314; act: -57 ),
  ( sym: 319; act: -57 ),
{ 56: }
{ 57: }
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 287; act: 31 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 58: }
  ( sym: 302; act: 97 ),
  ( sym: 256; act: -55 ),
  ( sym: 267; act: -55 ),
  ( sym: 268; act: -55 ),
  ( sym: 269; act: -55 ),
  ( sym: 270; act: -55 ),
  ( sym: 277; act: -55 ),
  ( sym: 287; act: -55 ),
  ( sym: 288; act: -55 ),
  ( sym: 289; act: -55 ),
  ( sym: 290; act: -55 ),
  ( sym: 294; act: -55 ),
  ( sym: 295; act: -55 ),
  ( sym: 296; act: -55 ),
  ( sym: 297; act: -55 ),
  ( sym: 298; act: -55 ),
  ( sym: 299; act: -55 ),
  ( sym: 300; act: -55 ),
  ( sym: 314; act: -55 ),
  ( sym: 319; act: -55 ),
{ 59: }
{ 60: }
{ 61: }
  ( sym: 277; act: 23 ),
  ( sym: 273; act: -163 ),
{ 62: }
{ 63: }
{ 64: }
  ( sym: 283; act: 102 ),
  ( sym: 256; act: -70 ),
  ( sym: 265; act: -70 ),
  ( sym: 266; act: -70 ),
  ( sym: 267; act: -70 ),
  ( sym: 268; act: -70 ),
  ( sym: 269; act: -70 ),
  ( sym: 270; act: -70 ),
  ( sym: 271; act: -70 ),
  ( sym: 272; act: -70 ),
  ( sym: 273; act: -70 ),
  ( sym: 277; act: -70 ),
  ( sym: 287; act: -70 ),
  ( sym: 288; act: -70 ),
  ( sym: 289; act: -70 ),
  ( sym: 290; act: -70 ),
  ( sym: 291; act: -70 ),
  ( sym: 294; act: -70 ),
  ( sym: 295; act: -70 ),
  ( sym: 296; act: -70 ),
  ( sym: 297; act: -70 ),
  ( sym: 298; act: -70 ),
  ( sym: 299; act: -70 ),
  ( sym: 300; act: -70 ),
  ( sym: 301; act: -70 ),
  ( sym: 304; act: -70 ),
  ( sym: 306; act: -70 ),
  ( sym: 307; act: -70 ),
  ( sym: 308; act: -70 ),
  ( sym: 309; act: -70 ),
  ( sym: 310; act: -70 ),
  ( sym: 311; act: -70 ),
  ( sym: 312; act: -70 ),
  ( sym: 313; act: -70 ),
  ( sym: 314; act: -70 ),
  ( sym: 315; act: -70 ),
  ( sym: 316; act: -70 ),
  ( sym: 317; act: -70 ),
  ( sym: 318; act: -70 ),
  ( sym: 319; act: -70 ),
  ( sym: 320; act: -70 ),
  ( sym: 321; act: -70 ),
  ( sym: 324; act: -70 ),
  ( sym: 325; act: -70 ),
{ 65: }
{ 66: }
{ 67: }
{ 68: }
{ 69: }
{ 70: }
  ( sym: 256; act: 84 ),
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 71: }
  ( sym: 294; act: 48 ),
  ( sym: 295; act: 49 ),
  ( sym: 296; act: 50 ),
  ( sym: 297; act: 51 ),
  ( sym: 298; act: 52 ),
  ( sym: 299; act: 53 ),
  ( sym: 300; act: 54 ),
  ( sym: 268; act: -18 ),
  ( sym: 277; act: -18 ),
  ( sym: 287; act: -18 ),
  ( sym: 288; act: -18 ),
  ( sym: 289; act: -18 ),
  ( sym: 290; act: -18 ),
  ( sym: 314; act: -18 ),
  ( sym: 319; act: -18 ),
{ 72: }
{ 73: }
  ( sym: 256; act: 56 ),
  ( sym: 272; act: 57 ),
  ( sym: 277; act: 23 ),
  ( sym: 268; act: -50 ),
  ( sym: 287; act: -50 ),
  ( sym: 288; act: -50 ),
  ( sym: 289; act: -50 ),
  ( sym: 290; act: -50 ),
  ( sym: 294; act: -50 ),
  ( sym: 295; act: -50 ),
  ( sym: 296; act: -50 ),
  ( sym: 297; act: -50 ),
  ( sym: 298; act: -50 ),
  ( sym: 299; act: -50 ),
  ( sym: 300; act: -50 ),
  ( sym: 314; act: -50 ),
  ( sym: 319; act: -50 ),
{ 74: }
  ( sym: 277; act: 23 ),
  ( sym: 269; act: -163 ),
{ 75: }
{ 76: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 291; act: 114 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 77: }
  ( sym: 302; act: 119 ),
  ( sym: 256; act: -46 ),
  ( sym: 266; act: -46 ),
  ( sym: 267; act: -46 ),
  ( sym: 268; act: -46 ),
  ( sym: 269; act: -46 ),
  ( sym: 270; act: -46 ),
  ( sym: 277; act: -46 ),
  ( sym: 287; act: -46 ),
  ( sym: 288; act: -46 ),
  ( sym: 289; act: -46 ),
  ( sym: 290; act: -46 ),
  ( sym: 294; act: -46 ),
  ( sym: 295; act: -46 ),
  ( sym: 296; act: -46 ),
  ( sym: 297; act: -46 ),
  ( sym: 298; act: -46 ),
  ( sym: 299; act: -46 ),
  ( sym: 300; act: -46 ),
  ( sym: 314; act: -46 ),
  ( sym: 319; act: -46 ),
{ 78: }
  ( sym: 302; act: 120 ),
  ( sym: 256; act: -48 ),
  ( sym: 266; act: -48 ),
  ( sym: 267; act: -48 ),
  ( sym: 268; act: -48 ),
  ( sym: 269; act: -48 ),
  ( sym: 270; act: -48 ),
  ( sym: 277; act: -48 ),
  ( sym: 287; act: -48 ),
  ( sym: 288; act: -48 ),
  ( sym: 289; act: -48 ),
  ( sym: 290; act: -48 ),
  ( sym: 294; act: -48 ),
  ( sym: 295; act: -48 ),
  ( sym: 296; act: -48 ),
  ( sym: 297; act: -48 ),
  ( sym: 298; act: -48 ),
  ( sym: 299; act: -48 ),
  ( sym: 300; act: -48 ),
  ( sym: 314; act: -48 ),
  ( sym: 319; act: -48 ),
{ 79: }
{ 80: }
  ( sym: 319; act: 121 ),
{ 81: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 124 ),
  ( sym: 266; act: -87 ),
  ( sym: 267; act: -87 ),
  ( sym: 272; act: -87 ),
  ( sym: 301; act: -87 ),
{ 82: }
  ( sym: 267; act: 127 ),
  ( sym: 272; act: 128 ),
  ( sym: 301; act: 129 ),
  ( sym: 266; act: -20 ),
{ 83: }
  ( sym: 265; act: 131 ),
  ( sym: 266; act: -104 ),
  ( sym: 267; act: -104 ),
  ( sym: 268; act: -104 ),
  ( sym: 269; act: -104 ),
  ( sym: 270; act: -104 ),
  ( sym: 272; act: -104 ),
  ( sym: 301; act: -104 ),
{ 84: }
{ 85: }
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 86: }
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 87: }
{ 88: }
{ 89: }
{ 90: }
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 91: }
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 92: }
{ 93: }
  ( sym: 273; act: 137 ),
{ 94: }
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 287; act: 31 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 273; act: -62 ),
{ 95: }
  ( sym: 273; act: 139 ),
{ 96: }
  ( sym: 256; act: 84 ),
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 97: }
{ 98: }
  ( sym: 273; act: 141 ),
{ 99: }
  ( sym: 267; act: 142 ),
  ( sym: 269; act: -162 ),
  ( sym: 273; act: -162 ),
{ 100: }
  ( sym: 273; act: 143 ),
{ 101: }
  ( sym: 304; act: 144 ),
  ( sym: 267; act: -165 ),
  ( sym: 269; act: -165 ),
  ( sym: 273; act: -165 ),
{ 102: }
{ 103: }
  ( sym: 266; act: 145 ),
  ( sym: 267; act: 127 ),
{ 104: }
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 105: }
  ( sym: 266; act: 147 ),
{ 106: }
  ( sym: 269; act: 148 ),
{ 107: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 269; act: -166 ),
  ( sym: 291; act: -166 ),
{ 108: }
{ 109: }
  ( sym: 291; act: 151 ),
{ 110: }
  ( sym: 268; act: 152 ),
  ( sym: 270; act: 153 ),
  ( sym: 265; act: -143 ),
  ( sym: 266; act: -143 ),
  ( sym: 267; act: -143 ),
  ( sym: 269; act: -143 ),
  ( sym: 271; act: -143 ),
  ( sym: 272; act: -143 ),
  ( sym: 273; act: -143 ),
  ( sym: 291; act: -143 ),
  ( sym: 301; act: -143 ),
  ( sym: 304; act: -143 ),
  ( sym: 306; act: -143 ),
  ( sym: 307; act: -143 ),
  ( sym: 308; act: -143 ),
  ( sym: 309; act: -143 ),
  ( sym: 310; act: -143 ),
  ( sym: 311; act: -143 ),
  ( sym: 312; act: -143 ),
  ( sym: 313; act: -143 ),
  ( sym: 314; act: -143 ),
  ( sym: 315; act: -143 ),
  ( sym: 316; act: -143 ),
  ( sym: 317; act: -143 ),
  ( sym: 318; act: -143 ),
  ( sym: 319; act: -143 ),
  ( sym: 320; act: -143 ),
  ( sym: 321; act: -143 ),
  ( sym: 324; act: -143 ),
  ( sym: 325; act: -143 ),
{ 111: }
  ( sym: 268; act: 111 ),
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 287; act: 31 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 319; act: 160 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 112: }
{ 113: }
{ 114: }
{ 115: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 116: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 117: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 118: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 119: }
{ 120: }
{ 121: }
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 122: }
{ 123: }
  ( sym: 269; act: 169 ),
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 170 ),
  ( sym: 287; act: 31 ),
  ( sym: 303; act: 171 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 124: }
  ( sym: 268; act: 111 ),
  ( sym: 271; act: 174 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 125: }
{ 126: }
  ( sym: 266; act: 175 ),
{ 127: }
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 128: }
  ( sym: 257; act: 180 ),
  ( sym: 266; act: 181 ),
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 273; act: -26 ),
{ 129: }
  ( sym: 268; act: 182 ),
{ 130: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 131: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 132: }
  ( sym: 267; act: 185 ),
  ( sym: 266; act: -86 ),
  ( sym: 272; act: -86 ),
  ( sym: 301; act: -86 ),
{ 133: }
  ( sym: 268; act: 123 ),
  ( sym: 269; act: 186 ),
  ( sym: 270; act: 124 ),
{ 134: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 124 ),
  ( sym: 266; act: -98 ),
  ( sym: 267; act: -98 ),
  ( sym: 269; act: -98 ),
  ( sym: 272; act: -98 ),
  ( sym: 301; act: -98 ),
{ 135: }
  ( sym: 270; act: 124 ),
  ( sym: 266; act: -101 ),
  ( sym: 267; act: -101 ),
  ( sym: 268; act: -101 ),
  ( sym: 269; act: -101 ),
  ( sym: 272; act: -101 ),
  ( sym: 301; act: -101 ),
{ 136: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 124 ),
  ( sym: 266; act: -100 ),
  ( sym: 267; act: -100 ),
  ( sym: 269; act: -100 ),
  ( sym: 272; act: -100 ),
  ( sym: 301; act: -100 ),
{ 137: }
{ 138: }
{ 139: }
{ 140: }
  ( sym: 266; act: 187 ),
  ( sym: 267; act: 127 ),
{ 141: }
{ 142: }
  ( sym: 277; act: 23 ),
  ( sym: 269; act: -163 ),
  ( sym: 273; act: -163 ),
{ 143: }
{ 144: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 145: }
{ 146: }
  ( sym: 268; act: 123 ),
  ( sym: 269; act: 190 ),
  ( sym: 270; act: 124 ),
{ 147: }
{ 148: }
  ( sym: 292; act: 193 ),
  ( sym: 268; act: -3 ),
{ 149: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 150: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 151: }
{ 152: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 269; act: -171 ),
{ 153: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 271; act: -171 ),
{ 154: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -139 ),
  ( sym: 266; act: -139 ),
  ( sym: 267; act: -139 ),
  ( sym: 268; act: -139 ),
  ( sym: 269; act: -139 ),
  ( sym: 270; act: -139 ),
  ( sym: 271; act: -139 ),
  ( sym: 272; act: -139 ),
  ( sym: 273; act: -139 ),
  ( sym: 291; act: -139 ),
  ( sym: 301; act: -139 ),
  ( sym: 304; act: -139 ),
  ( sym: 306; act: -139 ),
  ( sym: 307; act: -139 ),
  ( sym: 308; act: -139 ),
  ( sym: 309; act: -139 ),
  ( sym: 310; act: -139 ),
  ( sym: 311; act: -139 ),
  ( sym: 312; act: -139 ),
  ( sym: 313; act: -139 ),
  ( sym: 314; act: -139 ),
  ( sym: 315; act: -139 ),
  ( sym: 316; act: -139 ),
  ( sym: 317; act: -139 ),
  ( sym: 318; act: -139 ),
  ( sym: 319; act: -139 ),
  ( sym: 320; act: -139 ),
  ( sym: 321; act: -139 ),
{ 155: }
  ( sym: 269; act: 200 ),
  ( sym: 304; act: -121 ),
  ( sym: 306; act: -121 ),
  ( sym: 307; act: -121 ),
  ( sym: 308; act: -121 ),
  ( sym: 309; act: -121 ),
  ( sym: 310; act: -121 ),
  ( sym: 311; act: -121 ),
  ( sym: 312; act: -121 ),
  ( sym: 313; act: -121 ),
  ( sym: 314; act: -121 ),
  ( sym: 315; act: -121 ),
  ( sym: 316; act: -121 ),
  ( sym: 317; act: -121 ),
  ( sym: 318; act: -121 ),
  ( sym: 319; act: -121 ),
  ( sym: 320; act: -121 ),
  ( sym: 321; act: -121 ),
{ 156: }
  ( sym: 269; act: -82 ),
  ( sym: 288; act: -82 ),
  ( sym: 289; act: -82 ),
  ( sym: 290; act: -82 ),
  ( sym: 319; act: -82 ),
  ( sym: 304; act: -144 ),
  ( sym: 306; act: -144 ),
  ( sym: 307; act: -144 ),
  ( sym: 308; act: -144 ),
  ( sym: 309; act: -144 ),
  ( sym: 310; act: -144 ),
  ( sym: 311; act: -144 ),
  ( sym: 312; act: -144 ),
  ( sym: 313; act: -144 ),
  ( sym: 314; act: -144 ),
  ( sym: 315; act: -144 ),
  ( sym: 316; act: -144 ),
  ( sym: 317; act: -144 ),
  ( sym: 318; act: -144 ),
  ( sym: 320; act: -144 ),
  ( sym: 321; act: -144 ),
  ( sym: 324; act: -144 ),
  ( sym: 325; act: -144 ),
{ 157: }
  ( sym: 269; act: 202 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 319; act: 203 ),
{ 158: }
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
{ 159: }
  ( sym: 268; act: 152 ),
  ( sym: 269; act: 221 ),
  ( sym: 270; act: 153 ),
  ( sym: 288; act: -83 ),
  ( sym: 289; act: -83 ),
  ( sym: 290; act: -83 ),
  ( sym: 319; act: -83 ),
  ( sym: 304; act: -143 ),
  ( sym: 306; act: -143 ),
  ( sym: 307; act: -143 ),
  ( sym: 308; act: -143 ),
  ( sym: 309; act: -143 ),
  ( sym: 310; act: -143 ),
  ( sym: 311; act: -143 ),
  ( sym: 312; act: -143 ),
  ( sym: 313; act: -143 ),
  ( sym: 314; act: -143 ),
  ( sym: 315; act: -143 ),
  ( sym: 316; act: -143 ),
  ( sym: 317; act: -143 ),
  ( sym: 318; act: -143 ),
  ( sym: 320; act: -143 ),
  ( sym: 321; act: -143 ),
  ( sym: 324; act: -143 ),
  ( sym: 325; act: -143 ),
{ 160: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 161: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -151 ),
  ( sym: 266; act: -151 ),
  ( sym: 267; act: -151 ),
  ( sym: 268; act: -151 ),
  ( sym: 269; act: -151 ),
  ( sym: 270; act: -151 ),
  ( sym: 271; act: -151 ),
  ( sym: 272; act: -151 ),
  ( sym: 273; act: -151 ),
  ( sym: 291; act: -151 ),
  ( sym: 301; act: -151 ),
  ( sym: 304; act: -151 ),
  ( sym: 306; act: -151 ),
  ( sym: 307; act: -151 ),
  ( sym: 308; act: -151 ),
  ( sym: 309; act: -151 ),
  ( sym: 310; act: -151 ),
  ( sym: 311; act: -151 ),
  ( sym: 312; act: -151 ),
  ( sym: 313; act: -151 ),
  ( sym: 314; act: -151 ),
  ( sym: 315; act: -151 ),
  ( sym: 316; act: -151 ),
  ( sym: 317; act: -151 ),
  ( sym: 318; act: -151 ),
  ( sym: 319; act: -151 ),
  ( sym: 320; act: -151 ),
  ( sym: 321; act: -151 ),
{ 162: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -150 ),
  ( sym: 266; act: -150 ),
  ( sym: 267; act: -150 ),
  ( sym: 268; act: -150 ),
  ( sym: 269; act: -150 ),
  ( sym: 270; act: -150 ),
  ( sym: 271; act: -150 ),
  ( sym: 272; act: -150 ),
  ( sym: 273; act: -150 ),
  ( sym: 291; act: -150 ),
  ( sym: 301; act: -150 ),
  ( sym: 304; act: -150 ),
  ( sym: 306; act: -150 ),
  ( sym: 307; act: -150 ),
  ( sym: 308; act: -150 ),
  ( sym: 309; act: -150 ),
  ( sym: 310; act: -150 ),
  ( sym: 311; act: -150 ),
  ( sym: 312; act: -150 ),
  ( sym: 313; act: -150 ),
  ( sym: 314; act: -150 ),
  ( sym: 315; act: -150 ),
  ( sym: 316; act: -150 ),
  ( sym: 317; act: -150 ),
  ( sym: 318; act: -150 ),
  ( sym: 319; act: -150 ),
  ( sym: 320; act: -150 ),
  ( sym: 321; act: -150 ),
{ 163: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -149 ),
  ( sym: 266; act: -149 ),
  ( sym: 267; act: -149 ),
  ( sym: 268; act: -149 ),
  ( sym: 269; act: -149 ),
  ( sym: 270; act: -149 ),
  ( sym: 271; act: -149 ),
  ( sym: 272; act: -149 ),
  ( sym: 273; act: -149 ),
  ( sym: 291; act: -149 ),
  ( sym: 301; act: -149 ),
  ( sym: 304; act: -149 ),
  ( sym: 306; act: -149 ),
  ( sym: 307; act: -149 ),
  ( sym: 308; act: -149 ),
  ( sym: 309; act: -149 ),
  ( sym: 310; act: -149 ),
  ( sym: 311; act: -149 ),
  ( sym: 312; act: -149 ),
  ( sym: 313; act: -149 ),
  ( sym: 314; act: -149 ),
  ( sym: 315; act: -149 ),
  ( sym: 316; act: -149 ),
  ( sym: 317; act: -149 ),
  ( sym: 318; act: -149 ),
  ( sym: 319; act: -149 ),
  ( sym: 320; act: -149 ),
  ( sym: 321; act: -149 ),
{ 164: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -152 ),
  ( sym: 266; act: -152 ),
  ( sym: 267; act: -152 ),
  ( sym: 268; act: -152 ),
  ( sym: 269; act: -152 ),
  ( sym: 270; act: -152 ),
  ( sym: 271; act: -152 ),
  ( sym: 272; act: -152 ),
  ( sym: 273; act: -152 ),
  ( sym: 291; act: -152 ),
  ( sym: 301; act: -152 ),
  ( sym: 304; act: -152 ),
  ( sym: 306; act: -152 ),
  ( sym: 307; act: -152 ),
  ( sym: 308; act: -152 ),
  ( sym: 309; act: -152 ),
  ( sym: 310; act: -152 ),
  ( sym: 311; act: -152 ),
  ( sym: 312; act: -152 ),
  ( sym: 313; act: -152 ),
  ( sym: 314; act: -152 ),
  ( sym: 315; act: -152 ),
  ( sym: 316; act: -152 ),
  ( sym: 317; act: -152 ),
  ( sym: 318; act: -152 ),
  ( sym: 319; act: -152 ),
  ( sym: 320; act: -152 ),
  ( sym: 321; act: -152 ),
{ 165: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 124 ),
  ( sym: 266; act: -99 ),
  ( sym: 267; act: -99 ),
  ( sym: 269; act: -99 ),
  ( sym: 272; act: -99 ),
  ( sym: 301; act: -99 ),
{ 166: }
  ( sym: 267; act: 223 ),
  ( sym: 269; act: -91 ),
{ 167: }
  ( sym: 269; act: 224 ),
{ 168: }
  ( sym: 268; act: 228 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 229 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 230 ),
  ( sym: 267; act: -120 ),
  ( sym: 269; act: -120 ),
  ( sym: 270; act: -120 ),
{ 169: }
{ 170: }
  ( sym: 269; act: 231 ),
  ( sym: 267; act: -79 ),
  ( sym: 268; act: -79 ),
  ( sym: 270; act: -79 ),
  ( sym: 277; act: -79 ),
  ( sym: 287; act: -79 ),
  ( sym: 288; act: -79 ),
  ( sym: 289; act: -79 ),
  ( sym: 290; act: -79 ),
  ( sym: 314; act: -79 ),
  ( sym: 319; act: -79 ),
{ 171: }
{ 172: }
{ 173: }
  ( sym: 271; act: 232 ),
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
{ 174: }
{ 175: }
{ 176: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 124 ),
  ( sym: 266; act: -84 ),
  ( sym: 267; act: -84 ),
  ( sym: 272; act: -84 ),
  ( sym: 301; act: -84 ),
{ 177: }
  ( sym: 273; act: 233 ),
{ 178: }
  ( sym: 266; act: 234 ),
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
{ 179: }
  ( sym: 257; act: 180 ),
  ( sym: 266; act: 181 ),
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 273; act: -24 ),
{ 180: }
  ( sym: 268; act: 236 ),
{ 181: }
{ 182: }
  ( sym: 277; act: 23 ),
{ 183: }
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 266; act: -103 ),
  ( sym: 267; act: -103 ),
  ( sym: 268; act: -103 ),
  ( sym: 269; act: -103 ),
  ( sym: 270; act: -103 ),
  ( sym: 272; act: -103 ),
  ( sym: 301; act: -103 ),
{ 184: }
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 266; act: -102 ),
  ( sym: 267; act: -102 ),
  ( sym: 268; act: -102 ),
  ( sym: 269; act: -102 ),
  ( sym: 270; act: -102 ),
  ( sym: 272; act: -102 ),
  ( sym: 301; act: -102 ),
{ 185: }
  ( sym: 256; act: 84 ),
  ( sym: 268; act: 85 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 86 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 91 ),
{ 186: }
{ 187: }
{ 188: }
{ 189: }
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 267; act: -164 ),
  ( sym: 269; act: -164 ),
  ( sym: 273; act: -164 ),
{ 190: }
  ( sym: 292; act: 240 ),
  ( sym: 268; act: -3 ),
{ 191: }
  ( sym: 291; act: 241 ),
{ 192: }
  ( sym: 268; act: 242 ),
{ 193: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 194: }
{ 195: }
{ 196: }
  ( sym: 267; act: 244 ),
  ( sym: 269; act: -170 ),
  ( sym: 271; act: -170 ),
{ 197: }
  ( sym: 269; act: 245 ),
{ 198: }
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 267; act: -172 ),
  ( sym: 269; act: -172 ),
  ( sym: 271; act: -172 ),
{ 199: }
  ( sym: 271; act: 246 ),
{ 200: }
{ 201: }
  ( sym: 319; act: 247 ),
{ 202: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 203: }
  ( sym: 269; act: 249 ),
{ 204: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 205: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 206: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 207: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 208: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 209: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 210: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 211: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 212: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 213: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 214: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 215: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 216: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 217: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 218: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 219: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 220: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 221: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 265; act: -142 ),
  ( sym: 266; act: -142 ),
  ( sym: 267; act: -142 ),
  ( sym: 269; act: -142 ),
  ( sym: 270; act: -142 ),
  ( sym: 271; act: -142 ),
  ( sym: 272; act: -142 ),
  ( sym: 273; act: -142 ),
  ( sym: 291; act: -142 ),
  ( sym: 301; act: -142 ),
  ( sym: 304; act: -142 ),
  ( sym: 306; act: -142 ),
  ( sym: 307; act: -142 ),
  ( sym: 308; act: -142 ),
  ( sym: 309; act: -142 ),
  ( sym: 310; act: -142 ),
  ( sym: 311; act: -142 ),
  ( sym: 312; act: -142 ),
  ( sym: 313; act: -142 ),
  ( sym: 317; act: -142 ),
  ( sym: 318; act: -142 ),
  ( sym: 319; act: -142 ),
  ( sym: 320; act: -142 ),
  ( sym: 324; act: -142 ),
  ( sym: 325; act: -142 ),
{ 222: }
  ( sym: 269; act: 270 ),
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
{ 223: }
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 287; act: 31 ),
  ( sym: 303; act: 171 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 269; act: -94 ),
{ 224: }
{ 225: }
  ( sym: 319; act: 272 ),
{ 226: }
  ( sym: 268; act: 274 ),
  ( sym: 270; act: 275 ),
  ( sym: 267; act: -90 ),
  ( sym: 269; act: -90 ),
{ 227: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 276 ),
  ( sym: 267; act: -88 ),
  ( sym: 269; act: -88 ),
{ 228: }
  ( sym: 268; act: 228 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 229 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 279 ),
  ( sym: 269; act: -120 ),
  ( sym: 270; act: -120 ),
{ 229: }
  ( sym: 268; act: 228 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 229 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 279 ),
  ( sym: 267; act: -120 ),
  ( sym: 269; act: -120 ),
  ( sym: 270; act: -120 ),
{ 230: }
  ( sym: 268; act: 228 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 229 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 279 ),
  ( sym: 267; act: -120 ),
  ( sym: 269; act: -120 ),
  ( sym: 270; act: -120 ),
{ 231: }
{ 232: }
{ 233: }
{ 234: }
{ 235: }
{ 236: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 237: }
  ( sym: 269; act: 285 ),
{ 238: }
{ 239: }
  ( sym: 268; act: 286 ),
{ 240: }
{ 241: }
{ 242: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 243: }
{ 244: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 269; act: -171 ),
  ( sym: 271; act: -171 ),
{ 245: }
{ 246: }
{ 247: }
  ( sym: 269; act: 289 ),
{ 248: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -154 ),
  ( sym: 266; act: -154 ),
  ( sym: 267; act: -154 ),
  ( sym: 268; act: -154 ),
  ( sym: 269; act: -154 ),
  ( sym: 270; act: -154 ),
  ( sym: 271; act: -154 ),
  ( sym: 272; act: -154 ),
  ( sym: 273; act: -154 ),
  ( sym: 291; act: -154 ),
  ( sym: 301; act: -154 ),
  ( sym: 304; act: -154 ),
  ( sym: 306; act: -154 ),
  ( sym: 307; act: -154 ),
  ( sym: 308; act: -154 ),
  ( sym: 309; act: -154 ),
  ( sym: 310; act: -154 ),
  ( sym: 311; act: -154 ),
  ( sym: 312; act: -154 ),
  ( sym: 313; act: -154 ),
  ( sym: 314; act: -154 ),
  ( sym: 315; act: -154 ),
  ( sym: 316; act: -154 ),
  ( sym: 317; act: -154 ),
  ( sym: 318; act: -154 ),
  ( sym: 319; act: -154 ),
  ( sym: 320; act: -154 ),
  ( sym: 321; act: -154 ),
{ 249: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 250: }
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -122 ),
  ( sym: 266; act: -122 ),
  ( sym: 267; act: -122 ),
  ( sym: 268; act: -122 ),
  ( sym: 269; act: -122 ),
  ( sym: 270; act: -122 ),
  ( sym: 271; act: -122 ),
  ( sym: 272; act: -122 ),
  ( sym: 273; act: -122 ),
  ( sym: 291; act: -122 ),
  ( sym: 301; act: -122 ),
  ( sym: 324; act: -122 ),
  ( sym: 325; act: -122 ),
{ 251: }
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -123 ),
  ( sym: 266; act: -123 ),
  ( sym: 267; act: -123 ),
  ( sym: 268; act: -123 ),
  ( sym: 269; act: -123 ),
  ( sym: 270; act: -123 ),
  ( sym: 271; act: -123 ),
  ( sym: 272; act: -123 ),
  ( sym: 273; act: -123 ),
  ( sym: 291; act: -123 ),
  ( sym: 301; act: -123 ),
  ( sym: 304; act: -123 ),
  ( sym: 306; act: -123 ),
  ( sym: 307; act: -123 ),
  ( sym: 308; act: -123 ),
  ( sym: 309; act: -123 ),
  ( sym: 310; act: -123 ),
  ( sym: 311; act: -123 ),
  ( sym: 324; act: -123 ),
  ( sym: 325; act: -123 ),
{ 252: }
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -124 ),
  ( sym: 266; act: -124 ),
  ( sym: 267; act: -124 ),
  ( sym: 268; act: -124 ),
  ( sym: 269; act: -124 ),
  ( sym: 270; act: -124 ),
  ( sym: 271; act: -124 ),
  ( sym: 272; act: -124 ),
  ( sym: 273; act: -124 ),
  ( sym: 291; act: -124 ),
  ( sym: 301; act: -124 ),
  ( sym: 304; act: -124 ),
  ( sym: 306; act: -124 ),
  ( sym: 307; act: -124 ),
  ( sym: 308; act: -124 ),
  ( sym: 309; act: -124 ),
  ( sym: 310; act: -124 ),
  ( sym: 311; act: -124 ),
  ( sym: 324; act: -124 ),
  ( sym: 325; act: -124 ),
{ 253: }
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -125 ),
  ( sym: 266; act: -125 ),
  ( sym: 267; act: -125 ),
  ( sym: 268; act: -125 ),
  ( sym: 269; act: -125 ),
  ( sym: 270; act: -125 ),
  ( sym: 271; act: -125 ),
  ( sym: 272; act: -125 ),
  ( sym: 273; act: -125 ),
  ( sym: 291; act: -125 ),
  ( sym: 301; act: -125 ),
  ( sym: 304; act: -125 ),
  ( sym: 306; act: -125 ),
  ( sym: 307; act: -125 ),
  ( sym: 308; act: -125 ),
  ( sym: 309; act: -125 ),
  ( sym: 310; act: -125 ),
  ( sym: 311; act: -125 ),
  ( sym: 324; act: -125 ),
  ( sym: 325; act: -125 ),
{ 254: }
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -127 ),
  ( sym: 266; act: -127 ),
  ( sym: 267; act: -127 ),
  ( sym: 268; act: -127 ),
  ( sym: 269; act: -127 ),
  ( sym: 270; act: -127 ),
  ( sym: 271; act: -127 ),
  ( sym: 272; act: -127 ),
  ( sym: 273; act: -127 ),
  ( sym: 291; act: -127 ),
  ( sym: 301; act: -127 ),
  ( sym: 304; act: -127 ),
  ( sym: 306; act: -127 ),
  ( sym: 307; act: -127 ),
  ( sym: 308; act: -127 ),
  ( sym: 309; act: -127 ),
  ( sym: 310; act: -127 ),
  ( sym: 311; act: -127 ),
  ( sym: 324; act: -127 ),
  ( sym: 325; act: -127 ),
{ 255: }
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -126 ),
  ( sym: 266; act: -126 ),
  ( sym: 267; act: -126 ),
  ( sym: 268; act: -126 ),
  ( sym: 269; act: -126 ),
  ( sym: 270; act: -126 ),
  ( sym: 271; act: -126 ),
  ( sym: 272; act: -126 ),
  ( sym: 273; act: -126 ),
  ( sym: 291; act: -126 ),
  ( sym: 301; act: -126 ),
  ( sym: 304; act: -126 ),
  ( sym: 306; act: -126 ),
  ( sym: 307; act: -126 ),
  ( sym: 308; act: -126 ),
  ( sym: 309; act: -126 ),
  ( sym: 310; act: -126 ),
  ( sym: 311; act: -126 ),
  ( sym: 324; act: -126 ),
  ( sym: 325; act: -126 ),
{ 256: }
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -128 ),
  ( sym: 266; act: -128 ),
  ( sym: 267; act: -128 ),
  ( sym: 268; act: -128 ),
  ( sym: 269; act: -128 ),
  ( sym: 270; act: -128 ),
  ( sym: 271; act: -128 ),
  ( sym: 272; act: -128 ),
  ( sym: 273; act: -128 ),
  ( sym: 291; act: -128 ),
  ( sym: 301; act: -128 ),
  ( sym: 304; act: -128 ),
  ( sym: 306; act: -128 ),
  ( sym: 307; act: -128 ),
  ( sym: 308; act: -128 ),
  ( sym: 309; act: -128 ),
  ( sym: 310; act: -128 ),
  ( sym: 311; act: -128 ),
  ( sym: 324; act: -128 ),
  ( sym: 325; act: -128 ),
{ 257: }
{ 258: }
  ( sym: 265; act: 291 ),
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
{ 259: }
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -133 ),
  ( sym: 266; act: -133 ),
  ( sym: 267; act: -133 ),
  ( sym: 268; act: -133 ),
  ( sym: 269; act: -133 ),
  ( sym: 270; act: -133 ),
  ( sym: 271; act: -133 ),
  ( sym: 272; act: -133 ),
  ( sym: 273; act: -133 ),
  ( sym: 291; act: -133 ),
  ( sym: 301; act: -133 ),
  ( sym: 304; act: -133 ),
  ( sym: 306; act: -133 ),
  ( sym: 307; act: -133 ),
  ( sym: 308; act: -133 ),
  ( sym: 309; act: -133 ),
  ( sym: 310; act: -133 ),
  ( sym: 311; act: -133 ),
  ( sym: 312; act: -133 ),
  ( sym: 313; act: -133 ),
  ( sym: 324; act: -133 ),
  ( sym: 325; act: -133 ),
{ 260: }
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -134 ),
  ( sym: 266; act: -134 ),
  ( sym: 267; act: -134 ),
  ( sym: 268; act: -134 ),
  ( sym: 269; act: -134 ),
  ( sym: 270; act: -134 ),
  ( sym: 271; act: -134 ),
  ( sym: 272; act: -134 ),
  ( sym: 273; act: -134 ),
  ( sym: 291; act: -134 ),
  ( sym: 301; act: -134 ),
  ( sym: 304; act: -134 ),
  ( sym: 306; act: -134 ),
  ( sym: 307; act: -134 ),
  ( sym: 308; act: -134 ),
  ( sym: 309; act: -134 ),
  ( sym: 310; act: -134 ),
  ( sym: 311; act: -134 ),
  ( sym: 312; act: -134 ),
  ( sym: 313; act: -134 ),
  ( sym: 314; act: -134 ),
  ( sym: 324; act: -134 ),
  ( sym: 325; act: -134 ),
{ 261: }
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -129 ),
  ( sym: 266; act: -129 ),
  ( sym: 267; act: -129 ),
  ( sym: 268; act: -129 ),
  ( sym: 269; act: -129 ),
  ( sym: 270; act: -129 ),
  ( sym: 271; act: -129 ),
  ( sym: 272; act: -129 ),
  ( sym: 273; act: -129 ),
  ( sym: 291; act: -129 ),
  ( sym: 301; act: -129 ),
  ( sym: 304; act: -129 ),
  ( sym: 306; act: -129 ),
  ( sym: 307; act: -129 ),
  ( sym: 308; act: -129 ),
  ( sym: 309; act: -129 ),
  ( sym: 310; act: -129 ),
  ( sym: 311; act: -129 ),
  ( sym: 312; act: -129 ),
  ( sym: 313; act: -129 ),
  ( sym: 314; act: -129 ),
  ( sym: 315; act: -129 ),
  ( sym: 316; act: -129 ),
  ( sym: 324; act: -129 ),
  ( sym: 325; act: -129 ),
{ 262: }
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -130 ),
  ( sym: 266; act: -130 ),
  ( sym: 267; act: -130 ),
  ( sym: 268; act: -130 ),
  ( sym: 269; act: -130 ),
  ( sym: 270; act: -130 ),
  ( sym: 271; act: -130 ),
  ( sym: 272; act: -130 ),
  ( sym: 273; act: -130 ),
  ( sym: 291; act: -130 ),
  ( sym: 301; act: -130 ),
  ( sym: 304; act: -130 ),
  ( sym: 306; act: -130 ),
  ( sym: 307; act: -130 ),
  ( sym: 308; act: -130 ),
  ( sym: 309; act: -130 ),
  ( sym: 310; act: -130 ),
  ( sym: 311; act: -130 ),
  ( sym: 312; act: -130 ),
  ( sym: 313; act: -130 ),
  ( sym: 314; act: -130 ),
  ( sym: 315; act: -130 ),
  ( sym: 316; act: -130 ),
  ( sym: 324; act: -130 ),
  ( sym: 325; act: -130 ),
{ 263: }
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -137 ),
  ( sym: 266; act: -137 ),
  ( sym: 267; act: -137 ),
  ( sym: 268; act: -137 ),
  ( sym: 269; act: -137 ),
  ( sym: 270; act: -137 ),
  ( sym: 271; act: -137 ),
  ( sym: 272; act: -137 ),
  ( sym: 273; act: -137 ),
  ( sym: 291; act: -137 ),
  ( sym: 301; act: -137 ),
  ( sym: 304; act: -137 ),
  ( sym: 306; act: -137 ),
  ( sym: 307; act: -137 ),
  ( sym: 308; act: -137 ),
  ( sym: 309; act: -137 ),
  ( sym: 310; act: -137 ),
  ( sym: 311; act: -137 ),
  ( sym: 312; act: -137 ),
  ( sym: 313; act: -137 ),
  ( sym: 314; act: -137 ),
  ( sym: 315; act: -137 ),
  ( sym: 316; act: -137 ),
  ( sym: 317; act: -137 ),
  ( sym: 318; act: -137 ),
  ( sym: 324; act: -137 ),
  ( sym: 325; act: -137 ),
{ 264: }
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -136 ),
  ( sym: 266; act: -136 ),
  ( sym: 267; act: -136 ),
  ( sym: 268; act: -136 ),
  ( sym: 269; act: -136 ),
  ( sym: 270; act: -136 ),
  ( sym: 271; act: -136 ),
  ( sym: 272; act: -136 ),
  ( sym: 273; act: -136 ),
  ( sym: 291; act: -136 ),
  ( sym: 301; act: -136 ),
  ( sym: 304; act: -136 ),
  ( sym: 306; act: -136 ),
  ( sym: 307; act: -136 ),
  ( sym: 308; act: -136 ),
  ( sym: 309; act: -136 ),
  ( sym: 310; act: -136 ),
  ( sym: 311; act: -136 ),
  ( sym: 312; act: -136 ),
  ( sym: 313; act: -136 ),
  ( sym: 314; act: -136 ),
  ( sym: 315; act: -136 ),
  ( sym: 316; act: -136 ),
  ( sym: 317; act: -136 ),
  ( sym: 318; act: -136 ),
  ( sym: 324; act: -136 ),
  ( sym: 325; act: -136 ),
{ 265: }
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -131 ),
  ( sym: 266; act: -131 ),
  ( sym: 267; act: -131 ),
  ( sym: 268; act: -131 ),
  ( sym: 269; act: -131 ),
  ( sym: 270; act: -131 ),
  ( sym: 271; act: -131 ),
  ( sym: 272; act: -131 ),
  ( sym: 273; act: -131 ),
  ( sym: 291; act: -131 ),
  ( sym: 301; act: -131 ),
  ( sym: 304; act: -131 ),
  ( sym: 306; act: -131 ),
  ( sym: 307; act: -131 ),
  ( sym: 308; act: -131 ),
  ( sym: 309; act: -131 ),
  ( sym: 310; act: -131 ),
  ( sym: 311; act: -131 ),
  ( sym: 312; act: -131 ),
  ( sym: 313; act: -131 ),
  ( sym: 314; act: -131 ),
  ( sym: 315; act: -131 ),
  ( sym: 316; act: -131 ),
  ( sym: 317; act: -131 ),
  ( sym: 318; act: -131 ),
  ( sym: 319; act: -131 ),
  ( sym: 320; act: -131 ),
  ( sym: 324; act: -131 ),
  ( sym: 325; act: -131 ),
{ 266: }
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -132 ),
  ( sym: 266; act: -132 ),
  ( sym: 267; act: -132 ),
  ( sym: 268; act: -132 ),
  ( sym: 269; act: -132 ),
  ( sym: 270; act: -132 ),
  ( sym: 271; act: -132 ),
  ( sym: 272; act: -132 ),
  ( sym: 273; act: -132 ),
  ( sym: 291; act: -132 ),
  ( sym: 301; act: -132 ),
  ( sym: 304; act: -132 ),
  ( sym: 306; act: -132 ),
  ( sym: 307; act: -132 ),
  ( sym: 308; act: -132 ),
  ( sym: 309; act: -132 ),
  ( sym: 310; act: -132 ),
  ( sym: 311; act: -132 ),
  ( sym: 312; act: -132 ),
  ( sym: 313; act: -132 ),
  ( sym: 314; act: -132 ),
  ( sym: 315; act: -132 ),
  ( sym: 316; act: -132 ),
  ( sym: 317; act: -132 ),
  ( sym: 318; act: -132 ),
  ( sym: 319; act: -132 ),
  ( sym: 320; act: -132 ),
  ( sym: 324; act: -132 ),
  ( sym: 325; act: -132 ),
{ 267: }
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -135 ),
  ( sym: 266; act: -135 ),
  ( sym: 267; act: -135 ),
  ( sym: 268; act: -135 ),
  ( sym: 269; act: -135 ),
  ( sym: 270; act: -135 ),
  ( sym: 271; act: -135 ),
  ( sym: 272; act: -135 ),
  ( sym: 273; act: -135 ),
  ( sym: 291; act: -135 ),
  ( sym: 301; act: -135 ),
  ( sym: 304; act: -135 ),
  ( sym: 306; act: -135 ),
  ( sym: 307; act: -135 ),
  ( sym: 308; act: -135 ),
  ( sym: 309; act: -135 ),
  ( sym: 310; act: -135 ),
  ( sym: 311; act: -135 ),
  ( sym: 312; act: -135 ),
  ( sym: 313; act: -135 ),
  ( sym: 314; act: -135 ),
  ( sym: 315; act: -135 ),
  ( sym: 316; act: -135 ),
  ( sym: 317; act: -135 ),
  ( sym: 318; act: -135 ),
  ( sym: 319; act: -135 ),
  ( sym: 320; act: -135 ),
  ( sym: 324; act: -135 ),
  ( sym: 325; act: -135 ),
{ 268: }
{ 269: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -141 ),
  ( sym: 266; act: -141 ),
  ( sym: 267; act: -141 ),
  ( sym: 268; act: -141 ),
  ( sym: 269; act: -141 ),
  ( sym: 270; act: -141 ),
  ( sym: 271; act: -141 ),
  ( sym: 272; act: -141 ),
  ( sym: 273; act: -141 ),
  ( sym: 291; act: -141 ),
  ( sym: 301; act: -141 ),
  ( sym: 304; act: -141 ),
  ( sym: 306; act: -141 ),
  ( sym: 307; act: -141 ),
  ( sym: 308; act: -141 ),
  ( sym: 309; act: -141 ),
  ( sym: 310; act: -141 ),
  ( sym: 311; act: -141 ),
  ( sym: 312; act: -141 ),
  ( sym: 313; act: -141 ),
  ( sym: 314; act: -141 ),
  ( sym: 315; act: -141 ),
  ( sym: 316; act: -141 ),
  ( sym: 317; act: -141 ),
  ( sym: 318; act: -141 ),
  ( sym: 319; act: -141 ),
  ( sym: 320; act: -141 ),
  ( sym: 321; act: -141 ),
{ 270: }
  ( sym: 292; act: 240 ),
  ( sym: 268; act: -3 ),
{ 271: }
{ 272: }
  ( sym: 268; act: 228 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 229 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 279 ),
  ( sym: 267; act: -120 ),
  ( sym: 269; act: -120 ),
  ( sym: 270; act: -120 ),
{ 273: }
{ 274: }
  ( sym: 269; act: 169 ),
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 170 ),
  ( sym: 287; act: 31 ),
  ( sym: 303; act: 171 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 275: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 276: }
  ( sym: 268; act: 111 ),
  ( sym: 271; act: 297 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 277: }
  ( sym: 268; act: 274 ),
  ( sym: 269; act: 298 ),
  ( sym: 270; act: 275 ),
{ 278: }
  ( sym: 268; act: 123 ),
  ( sym: 269; act: 186 ),
  ( sym: 270; act: 276 ),
{ 279: }
  ( sym: 268; act: 228 ),
  ( sym: 277; act: 23 ),
  ( sym: 287; act: 229 ),
  ( sym: 288; act: 87 ),
  ( sym: 289; act: 88 ),
  ( sym: 290; act: 89 ),
  ( sym: 314; act: 90 ),
  ( sym: 319; act: 279 ),
  ( sym: 267; act: -120 ),
  ( sym: 269; act: -120 ),
  ( sym: 270; act: -120 ),
{ 280: }
  ( sym: 268; act: 274 ),
  ( sym: 270; act: 275 ),
  ( sym: 267; act: -112 ),
  ( sym: 269; act: -112 ),
{ 281: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 276 ),
  ( sym: 267; act: -98 ),
  ( sym: 269; act: -98 ),
{ 282: }
  ( sym: 270; act: 275 ),
  ( sym: 267; act: -114 ),
  ( sym: 268; act: -114 ),
  ( sym: 269; act: -114 ),
{ 283: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 276 ),
  ( sym: 267; act: -89 ),
  ( sym: 269; act: -89 ),
{ 284: }
  ( sym: 269; act: 300 ),
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
{ 285: }
{ 286: }
  ( sym: 274; act: 20 ),
  ( sym: 275; act: 21 ),
  ( sym: 276; act: 22 ),
  ( sym: 277; act: 23 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 287; act: 31 ),
  ( sym: 303; act: 171 ),
<<<<<<< HEAD
=======
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 269; act: -94 ),
{ 287: }
  ( sym: 269; act: 302 ),
{ 288: }
{ 289: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 290: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -155 ),
  ( sym: 266; act: -155 ),
  ( sym: 267; act: -155 ),
  ( sym: 268; act: -155 ),
  ( sym: 269; act: -155 ),
  ( sym: 270; act: -155 ),
  ( sym: 271; act: -155 ),
  ( sym: 272; act: -155 ),
  ( sym: 273; act: -155 ),
  ( sym: 291; act: -155 ),
  ( sym: 301; act: -155 ),
  ( sym: 304; act: -155 ),
  ( sym: 306; act: -155 ),
  ( sym: 307; act: -155 ),
  ( sym: 308; act: -155 ),
  ( sym: 309; act: -155 ),
  ( sym: 310; act: -155 ),
  ( sym: 311; act: -155 ),
  ( sym: 312; act: -155 ),
  ( sym: 313; act: -155 ),
  ( sym: 314; act: -155 ),
  ( sym: 315; act: -155 ),
  ( sym: 316; act: -155 ),
  ( sym: 317; act: -155 ),
  ( sym: 318; act: -155 ),
  ( sym: 319; act: -155 ),
  ( sym: 320; act: -155 ),
  ( sym: 321; act: -155 ),
{ 291: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
>>>>>>> origin/fixes_2.4
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
<<<<<<< HEAD
  ( sym: 269; act: -94 ),
{ 287: }
  ( sym: 269; act: 302 ),
{ 288: }
{ 289: }
=======
{ 292: }
  ( sym: 268; act: 305 ),
{ 293: }
  ( sym: 268; act: 274 ),
  ( sym: 270; act: 275 ),
  ( sym: 267; act: -113 ),
  ( sym: 269; act: -113 ),
{ 294: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 276 ),
  ( sym: 267; act: -99 ),
  ( sym: 269; act: -99 ),
{ 295: }
  ( sym: 269; act: 306 ),
{ 296: }
  ( sym: 271; act: 307 ),
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
{ 297: }
{ 298: }
{ 299: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 276 ),
  ( sym: 267; act: -100 ),
  ( sym: 269; act: -100 ),
{ 300: }
  ( sym: 257; act: 180 ),
  ( sym: 266; act: 181 ),
>>>>>>> origin/fixes_2.4
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
<<<<<<< HEAD
{ 290: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -155 ),
  ( sym: 266; act: -155 ),
  ( sym: 267; act: -155 ),
  ( sym: 268; act: -155 ),
  ( sym: 269; act: -155 ),
  ( sym: 270; act: -155 ),
  ( sym: 271; act: -155 ),
  ( sym: 272; act: -155 ),
  ( sym: 273; act: -155 ),
  ( sym: 291; act: -155 ),
  ( sym: 301; act: -155 ),
  ( sym: 304; act: -155 ),
  ( sym: 306; act: -155 ),
  ( sym: 307; act: -155 ),
  ( sym: 308; act: -155 ),
  ( sym: 309; act: -155 ),
  ( sym: 310; act: -155 ),
  ( sym: 311; act: -155 ),
  ( sym: 312; act: -155 ),
  ( sym: 313; act: -155 ),
  ( sym: 314; act: -155 ),
  ( sym: 315; act: -155 ),
  ( sym: 316; act: -155 ),
  ( sym: 317; act: -155 ),
  ( sym: 318; act: -155 ),
  ( sym: 319; act: -155 ),
  ( sym: 320; act: -155 ),
  ( sym: 321; act: -155 ),
{ 291: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
{ 292: }
  ( sym: 268; act: 305 ),
{ 293: }
  ( sym: 268; act: 274 ),
  ( sym: 270; act: 275 ),
  ( sym: 267; act: -113 ),
  ( sym: 269; act: -113 ),
{ 294: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 276 ),
  ( sym: 267; act: -99 ),
  ( sym: 269; act: -99 ),
{ 295: }
  ( sym: 269; act: 306 ),
{ 296: }
  ( sym: 271; act: 307 ),
  ( sym: 304; act: 204 ),
  ( sym: 306; act: 205 ),
  ( sym: 307; act: 206 ),
  ( sym: 308; act: 207 ),
  ( sym: 309; act: 208 ),
  ( sym: 310; act: 209 ),
  ( sym: 311; act: 210 ),
  ( sym: 312; act: 211 ),
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
{ 297: }
{ 298: }
{ 299: }
  ( sym: 268; act: 123 ),
  ( sym: 270; act: 276 ),
  ( sym: 267; act: -100 ),
  ( sym: 269; act: -100 ),
{ 300: }
  ( sym: 257; act: 180 ),
  ( sym: 266; act: 181 ),
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
=======
>>>>>>> origin/fixes_2.4
  ( sym: 273; act: -26 ),
{ 301: }
  ( sym: 269; act: 309 ),
{ 302: }
{ 303: }
  ( sym: 324; act: 149 ),
  ( sym: 325; act: 150 ),
  ( sym: 265; act: -156 ),
  ( sym: 266; act: -156 ),
  ( sym: 267; act: -156 ),
  ( sym: 268; act: -156 ),
  ( sym: 269; act: -156 ),
  ( sym: 270; act: -156 ),
  ( sym: 271; act: -156 ),
  ( sym: 272; act: -156 ),
  ( sym: 273; act: -156 ),
  ( sym: 291; act: -156 ),
  ( sym: 301; act: -156 ),
  ( sym: 304; act: -156 ),
  ( sym: 306; act: -156 ),
  ( sym: 307; act: -156 ),
  ( sym: 308; act: -156 ),
  ( sym: 309; act: -156 ),
  ( sym: 310; act: -156 ),
  ( sym: 311; act: -156 ),
  ( sym: 312; act: -156 ),
  ( sym: 313; act: -156 ),
  ( sym: 314; act: -156 ),
  ( sym: 315; act: -156 ),
  ( sym: 316; act: -156 ),
  ( sym: 317; act: -156 ),
  ( sym: 318; act: -156 ),
  ( sym: 319; act: -156 ),
  ( sym: 320; act: -156 ),
  ( sym: 321; act: -156 ),
{ 304: }
  ( sym: 313; act: 212 ),
  ( sym: 314; act: 213 ),
  ( sym: 315; act: 214 ),
  ( sym: 316; act: 215 ),
  ( sym: 317; act: 216 ),
  ( sym: 318; act: 217 ),
  ( sym: 319; act: 218 ),
  ( sym: 320; act: 219 ),
  ( sym: 321; act: 220 ),
  ( sym: 265; act: -140 ),
  ( sym: 266; act: -140 ),
  ( sym: 267; act: -140 ),
  ( sym: 268; act: -140 ),
  ( sym: 269; act: -140 ),
  ( sym: 270; act: -140 ),
  ( sym: 271; act: -140 ),
  ( sym: 272; act: -140 ),
  ( sym: 273; act: -140 ),
  ( sym: 291; act: -140 ),
  ( sym: 301; act: -140 ),
  ( sym: 304; act: -140 ),
  ( sym: 306; act: -140 ),
  ( sym: 307; act: -140 ),
  ( sym: 308; act: -140 ),
  ( sym: 309; act: -140 ),
  ( sym: 310; act: -140 ),
  ( sym: 311; act: -140 ),
  ( sym: 312; act: -140 ),
  ( sym: 324; act: -140 ),
  ( sym: 325; act: -140 ),
{ 305: }
  ( sym: 268; act: 111 ),
  ( sym: 277; act: 23 ),
  ( sym: 278; act: 112 ),
  ( sym: 279; act: 113 ),
  ( sym: 280; act: 24 ),
  ( sym: 281; act: 25 ),
  ( sym: 282; act: 26 ),
  ( sym: 283; act: 27 ),
  ( sym: 284; act: 28 ),
  ( sym: 285; act: 29 ),
  ( sym: 286; act: 30 ),
  ( sym: 314; act: 115 ),
  ( sym: 315; act: 116 ),
  ( sym: 316; act: 117 ),
  ( sym: 321; act: 118 ),
  ( sym: 327; act: 32 ),
  ( sym: 328; act: 33 ),
  ( sym: 329; act: 34 ),
  ( sym: 330; act: 35 ),
  ( sym: 331; act: 36 ),
  ( sym: 269; act: -171 ),
{ 306: }
{ 307: }
{ 308: }
{ 309: }
  ( sym: 266; act: 311 ),
{ 310: }
  ( sym: 269; act: 312 )
{ 311: }
{ 312: }
);

yyg : array [1..yyngotos] of YYARec = (
{ 0: }
  ( sym: -18; act: 1 ),
  ( sym: -8; act: 2 ),
  ( sym: -7; act: 3 ),
  ( sym: -6; act: 4 ),
  ( sym: -3; act: 5 ),
  ( sym: -2; act: 6 ),
{ 1: }
{ 2: }
  ( sym: -29; act: 15 ),
  ( sym: -27; act: 16 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 18 ),
  ( sym: -11; act: 19 ),
{ 3: }
{ 4: }
{ 5: }
  ( sym: -18; act: 1 ),
  ( sym: -8; act: 2 ),
  ( sym: -7; act: 37 ),
  ( sym: -6; act: 38 ),
{ 6: }
{ 7: }
  ( sym: -5; act: 39 ),
{ 8: }
  ( sym: -29; act: 15 ),
  ( sym: -27; act: 16 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 40 ),
  ( sym: -11; act: 41 ),
{ 9: }
  ( sym: -11; act: 43 ),
{ 10: }
  ( sym: -11; act: 44 ),
{ 11: }
  ( sym: -11; act: 45 ),
{ 12: }
  ( sym: -11; act: 46 ),
{ 13: }
{ 14: }
{ 15: }
{ 16: }
{ 17: }
{ 18: }
  ( sym: -9; act: 47 ),
{ 19: }
{ 20: }
  ( sym: -24; act: 55 ),
  ( sym: -11; act: 44 ),
{ 21: }
  ( sym: -24; act: 58 ),
  ( sym: -11; act: 45 ),
{ 22: }
  ( sym: -26; act: 59 ),
  ( sym: -11; act: 46 ),
{ 23: }
{ 24: }
{ 25: }
  ( sym: -29; act: 63 ),
{ 26: }
{ 27: }
{ 28: }
{ 29: }
{ 30: }
{ 31: }
  ( sym: -29; act: 15 ),
  ( sym: -27; act: 16 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 66 ),
  ( sym: -11; act: 19 ),
{ 32: }
  ( sym: -29; act: 67 ),
{ 33: }
{ 34: }
{ 35: }
{ 36: }
{ 37: }
{ 38: }
{ 39: }
{ 40: }
  ( sym: -9; act: 70 ),
{ 41: }
{ 42: }
  ( sym: -24; act: 55 ),
  ( sym: -11; act: 73 ),
{ 43: }
{ 44: }
  ( sym: -24; act: 77 ),
{ 45: }
  ( sym: -24; act: 78 ),
{ 46: }
  ( sym: -26; act: 79 ),
{ 47: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 81 ),
  ( sym: -17; act: 82 ),
  ( sym: -11; act: 83 ),
{ 48: }
{ 49: }
{ 50: }
{ 51: }
{ 52: }
{ 53: }
{ 54: }
{ 55: }
{ 56: }
  ( sym: -5; act: 93 ),
{ 57: }
  ( sym: -29; act: 15 ),
  ( sym: -28; act: 94 ),
  ( sym: -27; act: 16 ),
  ( sym: -25; act: 95 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 96 ),
  ( sym: -11; act: 19 ),
{ 58: }
{ 59: }
{ 60: }
  ( sym: -5; act: 98 ),
{ 61: }
  ( sym: -40; act: 99 ),
  ( sym: -21; act: 100 ),
  ( sym: -11; act: 101 ),
{ 62: }
{ 63: }
{ 64: }
{ 65: }
{ 66: }
{ 67: }
{ 68: }
{ 69: }
{ 70: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 81 ),
  ( sym: -17; act: 103 ),
  ( sym: -11; act: 83 ),
{ 71: }
  ( sym: -9; act: 104 ),
{ 72: }
{ 73: }
  ( sym: -24; act: 77 ),
  ( sym: -11; act: 105 ),
{ 74: }
  ( sym: -40; act: 99 ),
  ( sym: -21; act: 106 ),
  ( sym: -11; act: 101 ),
{ 75: }
{ 76: }
  ( sym: -37; act: 107 ),
  ( sym: -29; act: 108 ),
  ( sym: -23; act: 109 ),
  ( sym: -11; act: 110 ),
{ 77: }
{ 78: }
{ 79: }
{ 80: }
{ 81: }
  ( sym: -34; act: 122 ),
{ 82: }
  ( sym: -15; act: 125 ),
  ( sym: -10; act: 126 ),
{ 83: }
  ( sym: -33; act: 130 ),
{ 84: }
  ( sym: -5; act: 132 ),
{ 85: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 133 ),
  ( sym: -11; act: 83 ),
{ 86: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 134 ),
  ( sym: -11; act: 83 ),
{ 87: }
{ 88: }
{ 89: }
{ 90: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 135 ),
  ( sym: -11; act: 83 ),
{ 91: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 136 ),
  ( sym: -11; act: 83 ),
{ 92: }
{ 93: }
{ 94: }
  ( sym: -29; act: 15 ),
  ( sym: -28; act: 94 ),
  ( sym: -27; act: 16 ),
  ( sym: -25; act: 138 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 96 ),
  ( sym: -11; act: 19 ),
{ 95: }
{ 96: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 81 ),
  ( sym: -17; act: 140 ),
  ( sym: -11; act: 83 ),
{ 97: }
{ 98: }
{ 99: }
{ 100: }
{ 101: }
{ 102: }
{ 103: }
{ 104: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 146 ),
  ( sym: -11; act: 83 ),
{ 105: }
{ 106: }
{ 107: }
{ 108: }
{ 109: }
{ 110: }
{ 111: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 155 ),
  ( sym: -29; act: 156 ),
  ( sym: -27; act: 16 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 157 ),
  ( sym: -13; act: 158 ),
  ( sym: -11; act: 159 ),
{ 112: }
{ 113: }
{ 114: }
{ 115: }
  ( sym: -37; act: 161 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 116: }
  ( sym: -37; act: 162 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 117: }
  ( sym: -37; act: 163 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 118: }
  ( sym: -37; act: 164 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 119: }
{ 120: }
{ 121: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 165 ),
  ( sym: -11; act: 83 ),
{ 122: }
{ 123: }
  ( sym: -30; act: 166 ),
  ( sym: -29; act: 15 ),
  ( sym: -27; act: 16 ),
  ( sym: -20; act: 167 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 168 ),
  ( sym: -11; act: 19 ),
{ 124: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 173 ),
  ( sym: -11; act: 110 ),
{ 125: }
{ 126: }
{ 127: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 176 ),
  ( sym: -11; act: 83 ),
{ 128: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -14; act: 177 ),
  ( sym: -13; act: 178 ),
  ( sym: -12; act: 179 ),
  ( sym: -11; act: 110 ),
{ 129: }
{ 130: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 183 ),
  ( sym: -11; act: 110 ),
{ 131: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 184 ),
  ( sym: -11; act: 110 ),
{ 132: }
{ 133: }
  ( sym: -34; act: 122 ),
{ 134: }
  ( sym: -34; act: 122 ),
{ 135: }
  ( sym: -34; act: 122 ),
{ 136: }
  ( sym: -34; act: 122 ),
{ 137: }
{ 138: }
{ 139: }
{ 140: }
{ 141: }
{ 142: }
  ( sym: -40; act: 99 ),
  ( sym: -21; act: 188 ),
  ( sym: -11; act: 101 ),
{ 143: }
{ 144: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 189 ),
  ( sym: -11; act: 110 ),
{ 145: }
{ 146: }
  ( sym: -34; act: 122 ),
{ 147: }
{ 148: }
  ( sym: -22; act: 191 ),
  ( sym: -4; act: 192 ),
{ 149: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 194 ),
  ( sym: -11; act: 110 ),
{ 150: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 195 ),
  ( sym: -11; act: 110 ),
{ 151: }
{ 152: }
  ( sym: -41; act: 196 ),
  ( sym: -39; act: 197 ),
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 198 ),
  ( sym: -11; act: 110 ),
{ 153: }
  ( sym: -41; act: 196 ),
  ( sym: -39; act: 199 ),
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 198 ),
  ( sym: -11; act: 110 ),
{ 154: }
{ 155: }
{ 156: }
{ 157: }
  ( sym: -32; act: 201 ),
{ 158: }
{ 159: }
{ 160: }
  ( sym: -37; act: 222 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 161: }
{ 162: }
{ 163: }
{ 164: }
{ 165: }
  ( sym: -34; act: 122 ),
{ 166: }
{ 167: }
{ 168: }
  ( sym: -32; act: 225 ),
  ( sym: -31; act: 226 ),
  ( sym: -19; act: 227 ),
  ( sym: -11; act: 83 ),
{ 169: }
{ 170: }
{ 171: }
{ 172: }
{ 173: }
{ 174: }
{ 175: }
{ 176: }
  ( sym: -34; act: 122 ),
{ 177: }
{ 178: }
{ 179: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -14; act: 235 ),
  ( sym: -13; act: 178 ),
  ( sym: -12; act: 179 ),
  ( sym: -11; act: 110 ),
{ 180: }
{ 181: }
{ 182: }
  ( sym: -11; act: 237 ),
{ 183: }
{ 184: }
{ 185: }
  ( sym: -32; act: 80 ),
  ( sym: -19; act: 81 ),
  ( sym: -17; act: 238 ),
  ( sym: -11; act: 83 ),
{ 186: }
{ 187: }
{ 188: }
{ 189: }
{ 190: }
  ( sym: -4; act: 239 ),
{ 191: }
{ 192: }
{ 193: }
  ( sym: -37; act: 107 ),
  ( sym: -29; act: 108 ),
  ( sym: -23; act: 243 ),
  ( sym: -11; act: 110 ),
{ 194: }
{ 195: }
{ 196: }
{ 197: }
{ 198: }
{ 199: }
{ 200: }
{ 201: }
{ 202: }
  ( sym: -37; act: 248 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 203: }
{ 204: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 250 ),
  ( sym: -11; act: 110 ),
{ 205: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 251 ),
  ( sym: -11; act: 110 ),
{ 206: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 252 ),
  ( sym: -11; act: 110 ),
{ 207: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 253 ),
  ( sym: -11; act: 110 ),
{ 208: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 254 ),
  ( sym: -11; act: 110 ),
{ 209: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 255 ),
  ( sym: -11; act: 110 ),
{ 210: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 256 ),
  ( sym: -11; act: 110 ),
{ 211: }
  ( sym: -37; act: 154 ),
  ( sym: -36; act: 257 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 258 ),
  ( sym: -11; act: 110 ),
{ 212: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 259 ),
  ( sym: -11; act: 110 ),
{ 213: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 260 ),
  ( sym: -11; act: 110 ),
{ 214: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 261 ),
  ( sym: -11; act: 110 ),
{ 215: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 262 ),
  ( sym: -11; act: 110 ),
{ 216: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 263 ),
  ( sym: -11; act: 110 ),
{ 217: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 264 ),
  ( sym: -11; act: 110 ),
{ 218: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 265 ),
  ( sym: -11; act: 110 ),
{ 219: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 266 ),
  ( sym: -11; act: 110 ),
{ 220: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 267 ),
  ( sym: -11; act: 110 ),
{ 221: }
  ( sym: -38; act: 268 ),
  ( sym: -37; act: 269 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 222: }
{ 223: }
  ( sym: -30; act: 166 ),
  ( sym: -29; act: 15 ),
  ( sym: -27; act: 16 ),
  ( sym: -20; act: 271 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 168 ),
  ( sym: -11; act: 19 ),
{ 224: }
{ 225: }
{ 226: }
  ( sym: -34; act: 273 ),
{ 227: }
  ( sym: -34; act: 122 ),
{ 228: }
  ( sym: -32; act: 225 ),
  ( sym: -31; act: 277 ),
  ( sym: -19; act: 278 ),
  ( sym: -11; act: 83 ),
{ 229: }
  ( sym: -32; act: 225 ),
  ( sym: -31; act: 280 ),
  ( sym: -19; act: 281 ),
  ( sym: -11; act: 83 ),
{ 230: }
  ( sym: -32; act: 225 ),
  ( sym: -31; act: 282 ),
  ( sym: -19; act: 283 ),
  ( sym: -11; act: 83 ),
{ 231: }
{ 232: }
{ 233: }
{ 234: }
{ 235: }
{ 236: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 284 ),
  ( sym: -11; act: 110 ),
{ 237: }
{ 238: }
{ 239: }
{ 240: }
{ 241: }
{ 242: }
  ( sym: -37; act: 107 ),
  ( sym: -29; act: 108 ),
  ( sym: -23; act: 287 ),
  ( sym: -11; act: 110 ),
{ 243: }
{ 244: }
  ( sym: -41; act: 196 ),
  ( sym: -39; act: 288 ),
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 198 ),
  ( sym: -11; act: 110 ),
{ 245: }
{ 246: }
{ 247: }
{ 248: }
{ 249: }
  ( sym: -37; act: 290 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 250: }
{ 251: }
{ 252: }
{ 253: }
{ 254: }
{ 255: }
{ 256: }
{ 257: }
{ 258: }
{ 259: }
{ 260: }
{ 261: }
{ 262: }
{ 263: }
{ 264: }
{ 265: }
{ 266: }
{ 267: }
{ 268: }
{ 269: }
{ 270: }
  ( sym: -4; act: 292 ),
{ 271: }
{ 272: }
  ( sym: -32; act: 225 ),
  ( sym: -31; act: 293 ),
  ( sym: -19; act: 294 ),
  ( sym: -11; act: 83 ),
{ 273: }
{ 274: }
  ( sym: -30; act: 166 ),
  ( sym: -29; act: 15 ),
  ( sym: -27; act: 16 ),
  ( sym: -20; act: 295 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 168 ),
  ( sym: -11; act: 19 ),
{ 275: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 296 ),
  ( sym: -11; act: 110 ),
{ 276: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 173 ),
  ( sym: -11; act: 110 ),
{ 277: }
  ( sym: -34; act: 273 ),
{ 278: }
  ( sym: -34; act: 122 ),
{ 279: }
  ( sym: -32; act: 225 ),
  ( sym: -31; act: 282 ),
  ( sym: -19; act: 299 ),
  ( sym: -11; act: 83 ),
{ 280: }
  ( sym: -34; act: 273 ),
{ 281: }
  ( sym: -34; act: 122 ),
{ 282: }
  ( sym: -34; act: 273 ),
{ 283: }
  ( sym: -34; act: 122 ),
{ 284: }
{ 285: }
{ 286: }
  ( sym: -30; act: 166 ),
  ( sym: -29; act: 15 ),
  ( sym: -27; act: 16 ),
  ( sym: -20; act: 301 ),
  ( sym: -18; act: 17 ),
  ( sym: -16; act: 168 ),
  ( sym: -11; act: 19 ),
{ 287: }
{ 288: }
{ 289: }
  ( sym: -37; act: 303 ),
  ( sym: -29; act: 108 ),
  ( sym: -11; act: 110 ),
{ 290: }
{ 291: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 304 ),
  ( sym: -11; act: 110 ),
{ 292: }
{ 293: }
  ( sym: -34; act: 273 ),
{ 294: }
  ( sym: -34; act: 122 ),
{ 295: }
{ 296: }
{ 297: }
{ 298: }
{ 299: }
  ( sym: -34; act: 122 ),
{ 300: }
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -14; act: 308 ),
  ( sym: -13; act: 178 ),
  ( sym: -12; act: 179 ),
  ( sym: -11; act: 110 ),
{ 301: }
{ 302: }
{ 303: }
{ 304: }
{ 305: }
  ( sym: -41; act: 196 ),
  ( sym: -39; act: 310 ),
  ( sym: -37; act: 154 ),
  ( sym: -35; act: 172 ),
  ( sym: -29; act: 108 ),
  ( sym: -13; act: 198 ),
  ( sym: -11; act: 110 )
{ 306: }
{ 307: }
{ 308: }
{ 309: }
{ 310: }
{ 311: }
{ 312: }
);

yyd : array [0..yynstates-1] of Integer = (
{ 0: } 0,
{ 1: } 0,
{ 2: } 0,
{ 3: } -8,
{ 4: } -7,
{ 5: } 0,
{ 6: } 0,
{ 7: } -4,
{ 8: } 0,
{ 9: } 0,
{ 10: } 0,
{ 11: } 0,
{ 12: } 0,
{ 13: } -9,
{ 14: } -30,
{ 15: } -82,
{ 16: } -60,
{ 17: } -59,
{ 18: } 0,
{ 19: } -83,
{ 20: } 0,
{ 21: } 0,
{ 22: } 0,
{ 23: } -64,
{ 24: } 0,
{ 25: } 0,
{ 26: } 0,
{ 27: } -67,
{ 28: } -78,
{ 29: } -80,
{ 30: } -79,
{ 31: } 0,
{ 32: } 0,
{ 33: } -74,
{ 34: } -75,
{ 35: } -76,
{ 36: } -77,
{ 37: } -6,
{ 38: } -5,
{ 39: } 0,
{ 40: } 0,
{ 41: } 0,
{ 42: } 0,
{ 43: } 0,
{ 44: } 0,
{ 45: } 0,
{ 46: } 0,
{ 47: } 0,
{ 48: } -11,
{ 49: } -12,
{ 50: } -13,
{ 51: } -14,
{ 52: } -15,
{ 53: } -16,
{ 54: } -17,
{ 55: } 0,
{ 56: } -4,
{ 57: } 0,
{ 58: } 0,
{ 59: } -58,
{ 60: } -4,
{ 61: } 0,
{ 62: } -73,
{ 63: } -66,
{ 64: } 0,
{ 65: } -69,
{ 66: } -53,
{ 67: } -65,
{ 68: } -35,
{ 69: } -40,
{ 70: } 0,
{ 71: } 0,
{ 72: } -34,
{ 73: } 0,
{ 74: } 0,
{ 75: } -38,
{ 76: } 0,
{ 77: } 0,
{ 78: } 0,
{ 79: } -51,
{ 80: } 0,
{ 81: } 0,
{ 82: } 0,
{ 83: } 0,
{ 84: } -4,
{ 85: } 0,
{ 86: } 0,
{ 87: } -95,
{ 88: } -97,
{ 89: } -96,
{ 90: } 0,
{ 91: } 0,
{ 92: } -56,
{ 93: } 0,
{ 94: } 0,
{ 95: } 0,
{ 96: } 0,
{ 97: } -54,
{ 98: } 0,
{ 99: } 0,
{ 100: } 0,
{ 101: } 0,
{ 102: } -71,
{ 103: } 0,
{ 104: } 0,
{ 105: } 0,
{ 106: } 0,
{ 107: } 0,
{ 108: } -144,
{ 109: } 0,
{ 110: } 0,
{ 111: } 0,
{ 112: } -146,
{ 113: } -145,
{ 114: } -37,
{ 115: } 0,
{ 116: } 0,
{ 117: } 0,
{ 118: } 0,
{ 119: } -45,
{ 120: } -47,
{ 121: } 0,
{ 122: } -106,
{ 123: } 0,
{ 124: } 0,
{ 125: } -28,
{ 126: } 0,
{ 127: } 0,
{ 128: } 0,
{ 129: } 0,
{ 130: } 0,
{ 131: } 0,
{ 132: } 0,
{ 133: } 0,
{ 134: } 0,
{ 135: } 0,
{ 136: } 0,
{ 137: } -42,
{ 138: } -61,
{ 139: } -41,
{ 140: } 0,
{ 141: } -44,
{ 142: } 0,
{ 143: } -43,
{ 144: } 0,
{ 145: } -33,
{ 146: } 0,
{ 147: } -31,
{ 148: } 0,
{ 149: } 0,
{ 150: } 0,
{ 151: } -39,
{ 152: } 0,
{ 153: } 0,
{ 154: } 0,
{ 155: } 0,
{ 156: } 0,
{ 157: } 0,
{ 158: } 0,
{ 159: } 0,
{ 160: } 0,
{ 161: } 0,
{ 162: } 0,
{ 163: } 0,
{ 164: } 0,
{ 165: } 0,
{ 166: } 0,
{ 167: } 0,
{ 168: } 0,
{ 169: } -110,
{ 170: } 0,
{ 171: } -93,
{ 172: } -121,
{ 173: } 0,
{ 174: } -108,
{ 175: } -29,
{ 176: } 0,
{ 177: } 0,
{ 178: } 0,
{ 179: } 0,
{ 180: } 0,
{ 181: } -25,
{ 182: } 0,
{ 183: } 0,
{ 184: } 0,
{ 185: } 0,
{ 186: } -109,
{ 187: } -63,
{ 188: } -161,
{ 189: } 0,
{ 190: } 0,
{ 191: } 0,
{ 192: } 0,
{ 193: } 0,
{ 194: } -147,
{ 195: } -148,
{ 196: } 0,
{ 197: } 0,
{ 198: } 0,
{ 199: } 0,
{ 200: } -158,
{ 201: } 0,
{ 202: } 0,
{ 203: } 0,
{ 204: } 0,
{ 205: } 0,
{ 206: } 0,
{ 207: } 0,
{ 208: } 0,
{ 209: } 0,
{ 210: } 0,
{ 211: } 0,
{ 212: } 0,
{ 213: } 0,
{ 214: } 0,
{ 215: } 0,
{ 216: } 0,
{ 217: } 0,
{ 218: } 0,
{ 219: } 0,
{ 220: } 0,
{ 221: } 0,
{ 222: } 0,
{ 223: } 0,
{ 224: } -105,
{ 225: } 0,
{ 226: } 0,
{ 227: } 0,
{ 228: } 0,
{ 229: } 0,
{ 230: } 0,
{ 231: } -111,
{ 232: } -107,
{ 233: } -27,
{ 234: } -21,
{ 235: } -23,
{ 236: } 0,
{ 237: } 0,
{ 238: } -85,
{ 239: } 0,
{ 240: } -2,
{ 241: } -36,
{ 242: } 0,
{ 243: } -167,
{ 244: } 0,
{ 245: } -157,
{ 246: } -160,
{ 247: } 0,
{ 248: } 0,
{ 249: } 0,
{ 250: } 0,
{ 251: } 0,
{ 252: } 0,
{ 253: } 0,
{ 254: } 0,
{ 255: } 0,
{ 256: } 0,
{ 257: } -138,
{ 258: } 0,
{ 259: } 0,
{ 260: } 0,
{ 261: } 0,
{ 262: } 0,
{ 263: } 0,
{ 264: } 0,
{ 265: } 0,
{ 266: } 0,
{ 267: } 0,
{ 268: } -153,
{ 269: } 0,
{ 270: } 0,
{ 271: } -92,
{ 272: } 0,
{ 273: } -116,
{ 274: } 0,
{ 275: } 0,
{ 276: } 0,
{ 277: } 0,
{ 278: } 0,
{ 279: } 0,
{ 280: } 0,
{ 281: } 0,
{ 282: } 0,
{ 283: } 0,
{ 284: } 0,
{ 285: } -19,
{ 286: } 0,
{ 287: } 0,
{ 288: } -169,
{ 289: } 0,
{ 290: } 0,
{ 291: } 0,
{ 292: } 0,
{ 293: } 0,
{ 294: } 0,
{ 295: } 0,
{ 296: } 0,
{ 297: } -108,
{ 298: } -119,
{ 299: } 0,
{ 300: } 0,
{ 301: } 0,
{ 302: } -168,
{ 303: } 0,
{ 304: } 0,
{ 305: } 0,
{ 306: } -115,
{ 307: } -117,
{ 308: } -22,
{ 309: } 0,
{ 310: } 0,
{ 311: } -32,
{ 312: } -159
);

yyal : array [0..yynstates-1] of Integer = (
{ 0: } 1,
{ 1: } 22,
{ 2: } 23,
{ 3: } 40,
{ 4: } 40,
{ 5: } 40,
{ 6: } 62,
{ 7: } 63,
{ 8: } 63,
{ 9: } 80,
{ 10: } 81,
{ 11: } 82,
{ 12: } 83,
{ 13: } 84,
{ 14: } 84,
{ 15: } 84,
{ 16: } 84,
{ 17: } 84,
{ 18: } 84,
{ 19: } 100,
{ 20: } 100,
{ 21: } 103,
{ 22: } 106,
{ 23: } 109,
{ 24: } 109,
{ 25: } 153,
{ 26: } 208,
{ 27: } 253,
{ 28: } 253,
{ 29: } 253,
{ 30: } 253,
{ 31: } 253,
{ 32: } 270,
{ 33: } 282,
{ 34: } 282,
{ 35: } 282,
{ 36: } 282,
{ 37: } 282,
{ 38: } 282,
{ 39: } 282,
{ 40: } 284,
{ 41: } 300,
{ 42: } 317,
{ 43: } 320,
{ 44: } 323,
{ 45: } 344,
{ 46: } 365,
{ 47: } 386,
{ 48: } 395,
{ 49: } 395,
{ 50: } 395,
{ 51: } 395,
{ 52: } 395,
{ 53: } 395,
{ 54: } 395,
{ 55: } 395,
{ 56: } 415,
{ 57: } 415,
{ 58: } 432,
{ 59: } 452,
{ 60: } 452,
{ 61: } 452,
{ 62: } 454,
{ 63: } 454,
{ 64: } 454,
{ 65: } 498,
{ 66: } 498,
{ 67: } 498,
{ 68: } 498,
{ 69: } 498,
{ 70: } 498,
{ 71: } 507,
{ 72: } 522,
{ 73: } 522,
{ 74: } 539,
{ 75: } 541,
{ 76: } 541,
{ 77: } 562,
{ 78: } 583,
{ 79: } 604,
{ 80: } 604,
{ 81: } 605,
{ 82: } 611,
{ 83: } 615,
{ 84: } 623,
{ 85: } 623,
{ 86: } 631,
{ 87: } 639,
{ 88: } 639,
{ 89: } 639,
{ 90: } 639,
{ 91: } 647,
{ 92: } 655,
{ 93: } 655,
{ 94: } 656,
{ 95: } 674,
{ 96: } 675,
{ 97: } 684,
{ 98: } 684,
{ 99: } 685,
{ 100: } 688,
{ 101: } 689,
{ 102: } 693,
{ 103: } 693,
{ 104: } 695,
{ 105: } 703,
{ 106: } 704,
{ 107: } 705,
{ 108: } 709,
{ 109: } 709,
{ 110: } 710,
{ 111: } 740,
{ 112: } 765,
{ 113: } 765,
{ 114: } 765,
{ 115: } 765,
{ 116: } 785,
{ 117: } 805,
{ 118: } 825,
{ 119: } 845,
{ 120: } 845,
{ 121: } 845,
{ 122: } 853,
{ 123: } 853,
{ 124: } 872,
{ 125: } 893,
{ 126: } 893,
{ 127: } 894,
{ 128: } 902,
{ 129: } 925,
{ 130: } 926,
{ 131: } 946,
{ 132: } 966,
{ 133: } 970,
{ 134: } 973,
{ 135: } 980,
{ 136: } 987,
{ 137: } 994,
{ 138: } 994,
{ 139: } 994,
{ 140: } 994,
{ 141: } 996,
{ 142: } 996,
{ 143: } 999,
{ 144: } 999,
{ 145: } 1019,
{ 146: } 1019,
{ 147: } 1022,
{ 148: } 1022,
{ 149: } 1024,
{ 150: } 1044,
{ 151: } 1064,
{ 152: } 1064,
{ 153: } 1085,
{ 154: } 1106,
{ 155: } 1136,
{ 156: } 1154,
{ 157: } 1177,
{ 158: } 1182,
{ 159: } 1199,
{ 160: } 1224,
{ 161: } 1244,
{ 162: } 1274,
{ 163: } 1304,
{ 164: } 1334,
{ 165: } 1364,
{ 166: } 1371,
{ 167: } 1373,
{ 168: } 1374,
{ 169: } 1385,
{ 170: } 1385,
{ 171: } 1396,
{ 172: } 1396,
{ 173: } 1396,
{ 174: } 1414,
{ 175: } 1414,
{ 176: } 1414,
{ 177: } 1420,
{ 178: } 1421,
{ 179: } 1439,
{ 180: } 1462,
{ 181: } 1463,
{ 182: } 1463,
{ 183: } 1464,
{ 184: } 1488,
{ 185: } 1512,
{ 186: } 1521,
{ 187: } 1521,
{ 188: } 1521,
{ 189: } 1521,
{ 190: } 1541,
{ 191: } 1543,
{ 192: } 1544,
{ 193: } 1545,
{ 194: } 1565,
{ 195: } 1565,
{ 196: } 1565,
{ 197: } 1568,
{ 198: } 1569,
{ 199: } 1589,
{ 200: } 1590,
{ 201: } 1590,
{ 202: } 1591,
{ 203: } 1611,
{ 204: } 1612,
{ 205: } 1632,
{ 206: } 1652,
{ 207: } 1672,
{ 208: } 1692,
{ 209: } 1712,
{ 210: } 1732,
{ 211: } 1752,
{ 212: } 1772,
{ 213: } 1792,
{ 214: } 1812,
{ 215: } 1832,
{ 216: } 1852,
{ 217: } 1872,
{ 218: } 1892,
{ 219: } 1912,
{ 220: } 1932,
{ 221: } 1952,
{ 222: } 1997,
{ 223: } 2000,
{ 224: } 2019,
{ 225: } 2019,
{ 226: } 2020,
{ 227: } 2024,
{ 228: } 2028,
{ 229: } 2038,
{ 230: } 2049,
{ 231: } 2060,
{ 232: } 2060,
{ 233: } 2060,
{ 234: } 2060,
{ 235: } 2060,
{ 236: } 2060,
{ 237: } 2080,
{ 238: } 2081,
{ 239: } 2081,
{ 240: } 2082,
{ 241: } 2082,
{ 242: } 2082,
{ 243: } 2102,
{ 244: } 2102,
{ 245: } 2124,
{ 246: } 2124,
{ 247: } 2124,
{ 248: } 2125,
{ 249: } 2155,
{ 250: } 2175,
{ 251: } 2205,
{ 252: } 2235,
{ 253: } 2265,
{ 254: } 2295,
{ 255: } 2325,
{ 256: } 2355,
{ 257: } 2385,
{ 258: } 2385,
{ 259: } 2403,
{ 260: } 2433,
{ 261: } 2463,
{ 262: } 2493,
{ 263: } 2523,
{ 264: } 2553,
{ 265: } 2583,
{ 266: } 2613,
{ 267: } 2643,
{ 268: } 2673,
{ 269: } 2673,
{ 270: } 2703,
{ 271: } 2705,
{ 272: } 2705,
{ 273: } 2716,
{ 274: } 2716,
{ 275: } 2735,
{ 276: } 2755,
{ 277: } 2776,
{ 278: } 2779,
{ 279: } 2782,
{ 280: } 2793,
{ 281: } 2797,
{ 282: } 2801,
{ 283: } 2805,
{ 284: } 2809,
{ 285: } 2827,
{ 286: } 2827,
{ 287: } 2846,
{ 288: } 2847,
{ 289: } 2847,
{ 290: } 2867,
{ 291: } 2897,
{ 292: } 2917,
{ 293: } 2918,
{ 294: } 2922,
{ 295: } 2926,
{ 296: } 2927,
{ 297: } 2945,
{ 298: } 2945,
{ 299: } 2945,
{ 300: } 2949,
{ 301: } 2972,
{ 302: } 2973,
{ 303: } 2973,
{ 304: } 3003,
{ 305: } 3033,
{ 306: } 3054,
{ 307: } 3054,
{ 308: } 3054,
{ 309: } 3054,
{ 310: } 3055,
{ 311: } 3056,
{ 312: } 3056
);

yyah : array [0..yynstates-1] of Integer = (
{ 0: } 21,
{ 1: } 22,
{ 2: } 39,
{ 3: } 39,
{ 4: } 39,
{ 5: } 61,
{ 6: } 62,
{ 7: } 62,
{ 8: } 79,
{ 9: } 80,
{ 10: } 81,
{ 11: } 82,
{ 12: } 83,
{ 13: } 83,
{ 14: } 83,
{ 15: } 83,
{ 16: } 83,
{ 17: } 83,
{ 18: } 99,
{ 19: } 99,
{ 20: } 102,
{ 21: } 105,
{ 22: } 108,
{ 23: } 108,
{ 24: } 152,
{ 25: } 207,
{ 26: } 252,
{ 27: } 252,
{ 28: } 252,
{ 29: } 252,
{ 30: } 252,
{ 31: } 269,
{ 32: } 281,
{ 33: } 281,
{ 34: } 281,
{ 35: } 281,
{ 36: } 281,
{ 37: } 281,
{ 38: } 281,
{ 39: } 283,
{ 40: } 299,
{ 41: } 316,
{ 42: } 319,
{ 43: } 322,
{ 44: } 343,
{ 45: } 364,
{ 46: } 385,
{ 47: } 394,
{ 48: } 394,
{ 49: } 394,
{ 50: } 394,
{ 51: } 394,
{ 52: } 394,
{ 53: } 394,
{ 54: } 394,
{ 55: } 414,
{ 56: } 414,
{ 57: } 431,
{ 58: } 451,
{ 59: } 451,
{ 60: } 451,
{ 61: } 453,
{ 62: } 453,
{ 63: } 453,
{ 64: } 497,
{ 65: } 497,
{ 66: } 497,
{ 67: } 497,
{ 68: } 497,
{ 69: } 497,
{ 70: } 506,
{ 71: } 521,
{ 72: } 521,
{ 73: } 538,
{ 74: } 540,
{ 75: } 540,
{ 76: } 561,
{ 77: } 582,
{ 78: } 603,
{ 79: } 603,
{ 80: } 604,
{ 81: } 610,
{ 82: } 614,
{ 83: } 622,
{ 84: } 622,
{ 85: } 630,
{ 86: } 638,
{ 87: } 638,
{ 88: } 638,
{ 89: } 638,
{ 90: } 646,
{ 91: } 654,
{ 92: } 654,
{ 93: } 655,
{ 94: } 673,
{ 95: } 674,
{ 96: } 683,
{ 97: } 683,
{ 98: } 684,
{ 99: } 687,
{ 100: } 688,
{ 101: } 692,
{ 102: } 692,
{ 103: } 694,
{ 104: } 702,
{ 105: } 703,
{ 106: } 704,
{ 107: } 708,
{ 108: } 708,
{ 109: } 709,
{ 110: } 739,
{ 111: } 764,
{ 112: } 764,
{ 113: } 764,
{ 114: } 764,
{ 115: } 784,
{ 116: } 804,
{ 117: } 824,
{ 118: } 844,
{ 119: } 844,
{ 120: } 844,
{ 121: } 852,
{ 122: } 852,
{ 123: } 871,
{ 124: } 892,
{ 125: } 892,
{ 126: } 893,
{ 127: } 901,
{ 128: } 924,
{ 129: } 925,
{ 130: } 945,
{ 131: } 965,
{ 132: } 969,
{ 133: } 972,
{ 134: } 979,
{ 135: } 986,
{ 136: } 993,
{ 137: } 993,
{ 138: } 993,
{ 139: } 993,
{ 140: } 995,
{ 141: } 995,
{ 142: } 998,
{ 143: } 998,
{ 144: } 1018,
{ 145: } 1018,
{ 146: } 1021,
{ 147: } 1021,
{ 148: } 1023,
{ 149: } 1043,
{ 150: } 1063,
{ 151: } 1063,
{ 152: } 1084,
{ 153: } 1105,
{ 154: } 1135,
{ 155: } 1153,
{ 156: } 1176,
{ 157: } 1181,
{ 158: } 1198,
{ 159: } 1223,
{ 160: } 1243,
{ 161: } 1273,
{ 162: } 1303,
{ 163: } 1333,
{ 164: } 1363,
{ 165: } 1370,
{ 166: } 1372,
{ 167: } 1373,
{ 168: } 1384,
{ 169: } 1384,
{ 170: } 1395,
{ 171: } 1395,
{ 172: } 1395,
{ 173: } 1413,
{ 174: } 1413,
{ 175: } 1413,
{ 176: } 1419,
{ 177: } 1420,
{ 178: } 1438,
{ 179: } 1461,
{ 180: } 1462,
{ 181: } 1462,
{ 182: } 1463,
{ 183: } 1487,
{ 184: } 1511,
{ 185: } 1520,
{ 186: } 1520,
{ 187: } 1520,
{ 188: } 1520,
{ 189: } 1540,
{ 190: } 1542,
{ 191: } 1543,
{ 192: } 1544,
{ 193: } 1564,
{ 194: } 1564,
{ 195: } 1564,
{ 196: } 1567,
{ 197: } 1568,
{ 198: } 1588,
{ 199: } 1589,
{ 200: } 1589,
{ 201: } 1590,
{ 202: } 1610,
{ 203: } 1611,
{ 204: } 1631,
{ 205: } 1651,
{ 206: } 1671,
{ 207: } 1691,
{ 208: } 1711,
{ 209: } 1731,
{ 210: } 1751,
{ 211: } 1771,
{ 212: } 1791,
{ 213: } 1811,
{ 214: } 1831,
{ 215: } 1851,
{ 216: } 1871,
{ 217: } 1891,
{ 218: } 1911,
{ 219: } 1931,
{ 220: } 1951,
{ 221: } 1996,
{ 222: } 1999,
{ 223: } 2018,
{ 224: } 2018,
{ 225: } 2019,
{ 226: } 2023,
{ 227: } 2027,
{ 228: } 2037,
{ 229: } 2048,
{ 230: } 2059,
{ 231: } 2059,
{ 232: } 2059,
{ 233: } 2059,
{ 234: } 2059,
{ 235: } 2059,
{ 236: } 2079,
{ 237: } 2080,
{ 238: } 2080,
{ 239: } 2081,
{ 240: } 2081,
{ 241: } 2081,
{ 242: } 2101,
{ 243: } 2101,
{ 244: } 2123,
{ 245: } 2123,
{ 246: } 2123,
{ 247: } 2124,
{ 248: } 2154,
{ 249: } 2174,
{ 250: } 2204,
{ 251: } 2234,
{ 252: } 2264,
{ 253: } 2294,
{ 254: } 2324,
{ 255: } 2354,
{ 256: } 2384,
{ 257: } 2384,
{ 258: } 2402,
{ 259: } 2432,
{ 260: } 2462,
{ 261: } 2492,
{ 262: } 2522,
{ 263: } 2552,
{ 264: } 2582,
{ 265: } 2612,
{ 266: } 2642,
{ 267: } 2672,
{ 268: } 2672,
{ 269: } 2702,
{ 270: } 2704,
{ 271: } 2704,
{ 272: } 2715,
{ 273: } 2715,
{ 274: } 2734,
{ 275: } 2754,
{ 276: } 2775,
{ 277: } 2778,
{ 278: } 2781,
{ 279: } 2792,
{ 280: } 2796,
{ 281: } 2800,
{ 282: } 2804,
{ 283: } 2808,
{ 284: } 2826,
{ 285: } 2826,
{ 286: } 2845,
{ 287: } 2846,
{ 288: } 2846,
{ 289: } 2866,
{ 290: } 2896,
{ 291: } 2916,
{ 292: } 2917,
{ 293: } 2921,
{ 294: } 2925,
{ 295: } 2926,
{ 296: } 2944,
{ 297: } 2944,
{ 298: } 2944,
{ 299: } 2948,
{ 300: } 2971,
{ 301: } 2972,
{ 302: } 2972,
{ 303: } 3002,
{ 304: } 3032,
{ 305: } 3053,
{ 306: } 3053,
{ 307: } 3053,
{ 308: } 3053,
{ 309: } 3054,
{ 310: } 3055,
{ 311: } 3055,
{ 312: } 3055
);

yygl : array [0..yynstates-1] of Integer = (
{ 0: } 1,
{ 1: } 7,
{ 2: } 7,
{ 3: } 12,
{ 4: } 12,
{ 5: } 12,
{ 6: } 16,
{ 7: } 16,
{ 8: } 17,
{ 9: } 22,
{ 10: } 23,
{ 11: } 24,
{ 12: } 25,
{ 13: } 26,
{ 14: } 26,
{ 15: } 26,
{ 16: } 26,
{ 17: } 26,
{ 18: } 26,
{ 19: } 27,
{ 20: } 27,
{ 21: } 29,
{ 22: } 31,
{ 23: } 33,
{ 24: } 33,
{ 25: } 33,
{ 26: } 34,
{ 27: } 34,
{ 28: } 34,
{ 29: } 34,
{ 30: } 34,
{ 31: } 34,
{ 32: } 39,
{ 33: } 40,
{ 34: } 40,
{ 35: } 40,
{ 36: } 40,
{ 37: } 40,
{ 38: } 40,
{ 39: } 40,
{ 40: } 40,
{ 41: } 41,
{ 42: } 41,
{ 43: } 43,
{ 44: } 43,
{ 45: } 44,
{ 46: } 45,
{ 47: } 46,
{ 48: } 50,
{ 49: } 50,
{ 50: } 50,
{ 51: } 50,
{ 52: } 50,
{ 53: } 50,
{ 54: } 50,
{ 55: } 50,
{ 56: } 50,
{ 57: } 51,
{ 58: } 58,
{ 59: } 58,
{ 60: } 58,
{ 61: } 59,
{ 62: } 62,
{ 63: } 62,
{ 64: } 62,
{ 65: } 62,
{ 66: } 62,
{ 67: } 62,
{ 68: } 62,
{ 69: } 62,
{ 70: } 62,
{ 71: } 66,
{ 72: } 67,
{ 73: } 67,
{ 74: } 69,
{ 75: } 72,
{ 76: } 72,
{ 77: } 76,
{ 78: } 76,
{ 79: } 76,
{ 80: } 76,
{ 81: } 76,
{ 82: } 77,
{ 83: } 79,
{ 84: } 80,
{ 85: } 81,
{ 86: } 84,
{ 87: } 87,
{ 88: } 87,
{ 89: } 87,
{ 90: } 87,
{ 91: } 90,
{ 92: } 93,
{ 93: } 93,
{ 94: } 93,
{ 95: } 100,
{ 96: } 100,
{ 97: } 104,
{ 98: } 104,
{ 99: } 104,
{ 100: } 104,
{ 101: } 104,
{ 102: } 104,
{ 103: } 104,
{ 104: } 104,
{ 105: } 107,
{ 106: } 107,
{ 107: } 107,
{ 108: } 107,
{ 109: } 107,
{ 110: } 107,
{ 111: } 107,
{ 112: } 115,
{ 113: } 115,
{ 114: } 115,
{ 115: } 115,
{ 116: } 118,
{ 117: } 121,
{ 118: } 124,
{ 119: } 127,
{ 120: } 127,
{ 121: } 127,
{ 122: } 130,
{ 123: } 130,
{ 124: } 137,
{ 125: } 142,
{ 126: } 142,
{ 127: } 142,
{ 128: } 145,
{ 129: } 152,
{ 130: } 152,
{ 131: } 157,
{ 132: } 162,
{ 133: } 162,
{ 134: } 163,
{ 135: } 164,
{ 136: } 165,
{ 137: } 166,
{ 138: } 166,
{ 139: } 166,
{ 140: } 166,
{ 141: } 166,
{ 142: } 166,
{ 143: } 169,
{ 144: } 169,
{ 145: } 174,
{ 146: } 174,
{ 147: } 175,
{ 148: } 175,
{ 149: } 177,
{ 150: } 182,
{ 151: } 187,
{ 152: } 187,
{ 153: } 194,
{ 154: } 201,
{ 155: } 201,
{ 156: } 201,
{ 157: } 201,
{ 158: } 202,
{ 159: } 202,
{ 160: } 202,
{ 161: } 205,
{ 162: } 205,
{ 163: } 205,
{ 164: } 205,
{ 165: } 205,
{ 166: } 206,
{ 167: } 206,
{ 168: } 206,
{ 169: } 210,
{ 170: } 210,
{ 171: } 210,
{ 172: } 210,
{ 173: } 210,
{ 174: } 210,
{ 175: } 210,
{ 176: } 210,
{ 177: } 211,
{ 178: } 211,
{ 179: } 211,
{ 180: } 218,
{ 181: } 218,
{ 182: } 218,
{ 183: } 219,
{ 184: } 219,
{ 185: } 219,
{ 186: } 223,
{ 187: } 223,
{ 188: } 223,
{ 189: } 223,
{ 190: } 223,
{ 191: } 224,
{ 192: } 224,
{ 193: } 224,
{ 194: } 228,
{ 195: } 228,
{ 196: } 228,
{ 197: } 228,
{ 198: } 228,
{ 199: } 228,
{ 200: } 228,
{ 201: } 228,
{ 202: } 228,
{ 203: } 231,
{ 204: } 231,
{ 205: } 236,
{ 206: } 241,
{ 207: } 246,
{ 208: } 251,
{ 209: } 256,
{ 210: } 261,
{ 211: } 266,
{ 212: } 272,
{ 213: } 277,
{ 214: } 282,
{ 215: } 287,
{ 216: } 292,
{ 217: } 297,
{ 218: } 302,
{ 219: } 307,
{ 220: } 312,
{ 221: } 317,
{ 222: } 321,
{ 223: } 321,
{ 224: } 328,
{ 225: } 328,
{ 226: } 328,
{ 227: } 329,
{ 228: } 330,
{ 229: } 334,
{ 230: } 338,
{ 231: } 342,
{ 232: } 342,
{ 233: } 342,
{ 234: } 342,
{ 235: } 342,
{ 236: } 342,
{ 237: } 347,
{ 238: } 347,
{ 239: } 347,
{ 240: } 347,
{ 241: } 347,
{ 242: } 347,
{ 243: } 351,
{ 244: } 351,
{ 245: } 358,
{ 246: } 358,
{ 247: } 358,
{ 248: } 358,
{ 249: } 358,
{ 250: } 361,
{ 251: } 361,
{ 252: } 361,
{ 253: } 361,
{ 254: } 361,
{ 255: } 361,
{ 256: } 361,
{ 257: } 361,
{ 258: } 361,
{ 259: } 361,
{ 260: } 361,
{ 261: } 361,
{ 262: } 361,
{ 263: } 361,
{ 264: } 361,
{ 265: } 361,
{ 266: } 361,
{ 267: } 361,
{ 268: } 361,
{ 269: } 361,
{ 270: } 361,
{ 271: } 362,
{ 272: } 362,
{ 273: } 366,
{ 274: } 366,
{ 275: } 373,
{ 276: } 378,
{ 277: } 383,
{ 278: } 384,
{ 279: } 385,
{ 280: } 389,
{ 281: } 390,
{ 282: } 391,
{ 283: } 392,
{ 284: } 393,
{ 285: } 393,
{ 286: } 393,
{ 287: } 400,
{ 288: } 400,
{ 289: } 400,
{ 290: } 403,
{ 291: } 403,
{ 292: } 408,
{ 293: } 408,
{ 294: } 409,
{ 295: } 410,
{ 296: } 410,
{ 297: } 410,
{ 298: } 410,
{ 299: } 410,
{ 300: } 411,
{ 301: } 418,
{ 302: } 418,
{ 303: } 418,
{ 304: } 418,
{ 305: } 418,
{ 306: } 425,
{ 307: } 425,
{ 308: } 425,
{ 309: } 425,
{ 310: } 425,
{ 311: } 425,
{ 312: } 425
);

yygh : array [0..yynstates-1] of Integer = (
{ 0: } 6,
{ 1: } 6,
{ 2: } 11,
{ 3: } 11,
{ 4: } 11,
{ 5: } 15,
{ 6: } 15,
{ 7: } 16,
{ 8: } 21,
{ 9: } 22,
{ 10: } 23,
{ 11: } 24,
{ 12: } 25,
{ 13: } 25,
{ 14: } 25,
{ 15: } 25,
{ 16: } 25,
{ 17: } 25,
{ 18: } 26,
{ 19: } 26,
{ 20: } 28,
{ 21: } 30,
{ 22: } 32,
{ 23: } 32,
{ 24: } 32,
{ 25: } 33,
{ 26: } 33,
{ 27: } 33,
{ 28: } 33,
{ 29: } 33,
{ 30: } 33,
{ 31: } 38,
{ 32: } 39,
{ 33: } 39,
{ 34: } 39,
{ 35: } 39,
{ 36: } 39,
{ 37: } 39,
{ 38: } 39,
{ 39: } 39,
{ 40: } 40,
{ 41: } 40,
{ 42: } 42,
{ 43: } 42,
{ 44: } 43,
{ 45: } 44,
{ 46: } 45,
{ 47: } 49,
{ 48: } 49,
{ 49: } 49,
{ 50: } 49,
{ 51: } 49,
{ 52: } 49,
{ 53: } 49,
{ 54: } 49,
{ 55: } 49,
{ 56: } 50,
{ 57: } 57,
{ 58: } 57,
{ 59: } 57,
{ 60: } 58,
{ 61: } 61,
{ 62: } 61,
{ 63: } 61,
{ 64: } 61,
{ 65: } 61,
{ 66: } 61,
{ 67: } 61,
{ 68: } 61,
{ 69: } 61,
{ 70: } 65,
{ 71: } 66,
{ 72: } 66,
{ 73: } 68,
{ 74: } 71,
{ 75: } 71,
{ 76: } 75,
{ 77: } 75,
{ 78: } 75,
{ 79: } 75,
{ 80: } 75,
{ 81: } 76,
{ 82: } 78,
{ 83: } 79,
{ 84: } 80,
{ 85: } 83,
{ 86: } 86,
{ 87: } 86,
{ 88: } 86,
{ 89: } 86,
{ 90: } 89,
{ 91: } 92,
{ 92: } 92,
{ 93: } 92,
{ 94: } 99,
{ 95: } 99,
{ 96: } 103,
{ 97: } 103,
{ 98: } 103,
{ 99: } 103,
{ 100: } 103,
{ 101: } 103,
{ 102: } 103,
{ 103: } 103,
{ 104: } 106,
{ 105: } 106,
{ 106: } 106,
{ 107: } 106,
{ 108: } 106,
{ 109: } 106,
{ 110: } 106,
{ 111: } 114,
{ 112: } 114,
{ 113: } 114,
{ 114: } 114,
{ 115: } 117,
{ 116: } 120,
{ 117: } 123,
{ 118: } 126,
{ 119: } 126,
{ 120: } 126,
{ 121: } 129,
{ 122: } 129,
{ 123: } 136,
{ 124: } 141,
{ 125: } 141,
{ 126: } 141,
{ 127: } 144,
{ 128: } 151,
{ 129: } 151,
{ 130: } 156,
{ 131: } 161,
{ 132: } 161,
{ 133: } 162,
{ 134: } 163,
{ 135: } 164,
{ 136: } 165,
{ 137: } 165,
{ 138: } 165,
{ 139: } 165,
{ 140: } 165,
{ 141: } 165,
{ 142: } 168,
{ 143: } 168,
{ 144: } 173,
{ 145: } 173,
{ 146: } 174,
{ 147: } 174,
{ 148: } 176,
{ 149: } 181,
{ 150: } 186,
{ 151: } 186,
{ 152: } 193,
{ 153: } 200,
{ 154: } 200,
{ 155: } 200,
{ 156: } 200,
{ 157: } 201,
{ 158: } 201,
{ 159: } 201,
{ 160: } 204,
{ 161: } 204,
{ 162: } 204,
{ 163: } 204,
{ 164: } 204,
{ 165: } 205,
{ 166: } 205,
{ 167: } 205,
{ 168: } 209,
{ 169: } 209,
{ 170: } 209,
{ 171: } 209,
{ 172: } 209,
{ 173: } 209,
{ 174: } 209,
{ 175: } 209,
{ 176: } 210,
{ 177: } 210,
{ 178: } 210,
{ 179: } 217,
{ 180: } 217,
{ 181: } 217,
{ 182: } 218,
{ 183: } 218,
{ 184: } 218,
{ 185: } 222,
{ 186: } 222,
{ 187: } 222,
{ 188: } 222,
{ 189: } 222,
{ 190: } 223,
{ 191: } 223,
{ 192: } 223,
{ 193: } 227,
{ 194: } 227,
{ 195: } 227,
{ 196: } 227,
{ 197: } 227,
{ 198: } 227,
{ 199: } 227,
{ 200: } 227,
{ 201: } 227,
{ 202: } 230,
{ 203: } 230,
{ 204: } 235,
{ 205: } 240,
{ 206: } 245,
{ 207: } 250,
{ 208: } 255,
{ 209: } 260,
{ 210: } 265,
{ 211: } 271,
{ 212: } 276,
{ 213: } 281,
{ 214: } 286,
{ 215: } 291,
{ 216: } 296,
{ 217: } 301,
{ 218: } 306,
{ 219: } 311,
{ 220: } 316,
{ 221: } 320,
{ 222: } 320,
{ 223: } 327,
{ 224: } 327,
{ 225: } 327,
{ 226: } 328,
{ 227: } 329,
{ 228: } 333,
{ 229: } 337,
{ 230: } 341,
{ 231: } 341,
{ 232: } 341,
{ 233: } 341,
{ 234: } 341,
{ 235: } 341,
{ 236: } 346,
{ 237: } 346,
{ 238: } 346,
{ 239: } 346,
{ 240: } 346,
{ 241: } 346,
{ 242: } 350,
{ 243: } 350,
{ 244: } 357,
{ 245: } 357,
{ 246: } 357,
{ 247: } 357,
{ 248: } 357,
{ 249: } 360,
{ 250: } 360,
{ 251: } 360,
{ 252: } 360,
{ 253: } 360,
{ 254: } 360,
{ 255: } 360,
{ 256: } 360,
{ 257: } 360,
{ 258: } 360,
{ 259: } 360,
{ 260: } 360,
{ 261: } 360,
{ 262: } 360,
{ 263: } 360,
{ 264: } 360,
{ 265: } 360,
{ 266: } 360,
{ 267: } 360,
{ 268: } 360,
{ 269: } 360,
{ 270: } 361,
{ 271: } 361,
{ 272: } 365,
{ 273: } 365,
{ 274: } 372,
{ 275: } 377,
{ 276: } 382,
{ 277: } 383,
{ 278: } 384,
{ 279: } 388,
{ 280: } 389,
{ 281: } 390,
{ 282: } 391,
{ 283: } 392,
{ 284: } 392,
{ 285: } 392,
{ 286: } 399,
{ 287: } 399,
{ 288: } 399,
{ 289: } 402,
{ 290: } 402,
{ 291: } 407,
{ 292: } 407,
{ 293: } 408,
{ 294: } 409,
{ 295: } 409,
{ 296: } 409,
{ 297: } 409,
{ 298: } 409,
{ 299: } 410,
{ 300: } 417,
{ 301: } 417,
{ 302: } 417,
{ 303: } 417,
{ 304: } 417,
{ 305: } 424,
{ 306: } 424,
{ 307: } 424,
{ 308: } 424,
{ 309: } 424,
{ 310: } 424,
{ 311: } 424,
{ 312: } 424
);

yyr : array [1..yynrules] of YYRRec = (
{ 1: } ( len: 1; sym: -2 ),
{ 2: } ( len: 1; sym: -4 ),
{ 3: } ( len: 0; sym: -4 ),
{ 4: } ( len: 0; sym: -5 ),
{ 5: } ( len: 2; sym: -3 ),
{ 6: } ( len: 2; sym: -3 ),
{ 7: } ( len: 1; sym: -3 ),
{ 8: } ( len: 1; sym: -3 ),
{ 9: } ( len: 1; sym: -8 ),
{ 10: } ( len: 0; sym: -8 ),
{ 11: } ( len: 1; sym: -9 ),
{ 12: } ( len: 1; sym: -9 ),
{ 13: } ( len: 1; sym: -9 ),
{ 14: } ( len: 1; sym: -9 ),
{ 15: } ( len: 1; sym: -9 ),
{ 16: } ( len: 1; sym: -9 ),
{ 17: } ( len: 1; sym: -9 ),
{ 18: } ( len: 0; sym: -9 ),
{ 19: } ( len: 4; sym: -10 ),
{ 20: } ( len: 0; sym: -10 ),
{ 21: } ( len: 2; sym: -12 ),
{ 22: } ( len: 5; sym: -12 ),
{ 23: } ( len: 2; sym: -14 ),
{ 24: } ( len: 1; sym: -14 ),
{ 25: } ( len: 1; sym: -14 ),
{ 26: } ( len: 0; sym: -14 ),
{ 27: } ( len: 3; sym: -15 ),
{ 28: } ( len: 5; sym: -6 ),
{ 29: } ( len: 6; sym: -6 ),
{ 30: } ( len: 2; sym: -6 ),
{ 31: } ( len: 5; sym: -6 ),
{ 32: } ( len: 11; sym: -6 ),
{ 33: } ( len: 5; sym: -6 ),
{ 34: } ( len: 3; sym: -6 ),
{ 35: } ( len: 3; sym: -6 ),
{ 36: } ( len: 7; sym: -7 ),
{ 37: } ( len: 4; sym: -7 ),
{ 38: } ( len: 3; sym: -7 ),
{ 39: } ( len: 5; sym: -7 ),
{ 40: } ( len: 3; sym: -7 ),
{ 41: } ( len: 3; sym: -24 ),
{ 42: } ( len: 3; sym: -24 ),
{ 43: } ( len: 3; sym: -26 ),
{ 44: } ( len: 3; sym: -26 ),
{ 45: } ( len: 4; sym: -18 ),
{ 46: } ( len: 3; sym: -18 ),
{ 47: } ( len: 4; sym: -18 ),
{ 48: } ( len: 3; sym: -18 ),
{ 49: } ( len: 2; sym: -18 ),
{ 50: } ( len: 2; sym: -18 ),
{ 51: } ( len: 3; sym: -18 ),
{ 52: } ( len: 2; sym: -18 ),
{ 53: } ( len: 2; sym: -16 ),
{ 54: } ( len: 3; sym: -16 ),
{ 55: } ( len: 2; sym: -16 ),
{ 56: } ( len: 3; sym: -16 ),
{ 57: } ( len: 2; sym: -16 ),
{ 58: } ( len: 2; sym: -16 ),
{ 59: } ( len: 1; sym: -16 ),
{ 60: } ( len: 1; sym: -16 ),
{ 61: } ( len: 2; sym: -25 ),
{ 62: } ( len: 1; sym: -25 ),
{ 63: } ( len: 3; sym: -28 ),
{ 64: } ( len: 1; sym: -11 ),
{ 65: } ( len: 2; sym: -29 ),
{ 66: } ( len: 2; sym: -29 ),
{ 67: } ( len: 1; sym: -29 ),
{ 68: } ( len: 1; sym: -29 ),
{ 69: } ( len: 2; sym: -29 ),
{ 70: } ( len: 2; sym: -29 ),
{ 71: } ( len: 3; sym: -29 ),
{ 72: } ( len: 1; sym: -29 ),
{ 73: } ( len: 2; sym: -29 ),
{ 74: } ( len: 1; sym: -29 ),
{ 75: } ( len: 1; sym: -29 ),
{ 76: } ( len: 1; sym: -29 ),
{ 77: } ( len: 1; sym: -29 ),
{ 78: } ( len: 1; sym: -29 ),
{ 79: } ( len: 1; sym: -29 ),
{ 80: } ( len: 1; sym: -29 ),
{ 81: } ( len: 1; sym: -29 ),
{ 82: } ( len: 1; sym: -27 ),
{ 83: } ( len: 1; sym: -27 ),
{ 84: } ( len: 3; sym: -17 ),
{ 85: } ( len: 4; sym: -17 ),
{ 86: } ( len: 2; sym: -17 ),
{ 87: } ( len: 1; sym: -17 ),
{ 88: } ( len: 2; sym: -30 ),
{ 89: } ( len: 3; sym: -30 ),
{ 90: } ( len: 2; sym: -30 ),
{ 91: } ( len: 1; sym: -20 ),
{ 92: } ( len: 3; sym: -20 ),
{ 93: } ( len: 1; sym: -20 ),
{ 94: } ( len: 0; sym: -20 ),
{ 95: } ( len: 1; sym: -32 ),
{ 96: } ( len: 1; sym: -32 ),
{ 97: } ( len: 1; sym: -32 ),
{ 98: } ( len: 2; sym: -19 ),
{ 99: } ( len: 3; sym: -19 ),
{ 100: } ( len: 2; sym: -19 ),
{ 101: } ( len: 2; sym: -19 ),
{ 102: } ( len: 3; sym: -19 ),
{ 103: } ( len: 3; sym: -19 ),
{ 104: } ( len: 1; sym: -19 ),
{ 105: } ( len: 4; sym: -19 ),
{ 106: } ( len: 2; sym: -19 ),
{ 107: } ( len: 4; sym: -19 ),
{ 108: } ( len: 3; sym: -19 ),
{ 109: } ( len: 3; sym: -19 ),
{ 110: } ( len: 2; sym: -34 ),
{ 111: } ( len: 3; sym: -34 ),
{ 112: } ( len: 2; sym: -31 ),
{ 113: } ( len: 3; sym: -31 ),
{ 114: } ( len: 2; sym: -31 ),
{ 115: } ( len: 4; sym: -31 ),
{ 116: } ( len: 2; sym: -31 ),
{ 117: } ( len: 4; sym: -31 ),
{ 118: } ( len: 3; sym: -31 ),
{ 119: } ( len: 3; sym: -31 ),
{ 120: } ( len: 0; sym: -31 ),
{ 121: } ( len: 1; sym: -13 ),
{ 122: } ( len: 3; sym: -35 ),
{ 123: } ( len: 3; sym: -35 ),
{ 124: } ( len: 3; sym: -35 ),
{ 125: } ( len: 3; sym: -35 ),
{ 126: } ( len: 3; sym: -35 ),
{ 127: } ( len: 3; sym: -35 ),
{ 128: } ( len: 3; sym: -35 ),
{ 129: } ( len: 3; sym: -35 ),
{ 130: } ( len: 3; sym: -35 ),
{ 131: } ( len: 3; sym: -35 ),
{ 132: } ( len: 3; sym: -35 ),
{ 133: } ( len: 3; sym: -35 ),
{ 134: } ( len: 3; sym: -35 ),
{ 135: } ( len: 3; sym: -35 ),
{ 136: } ( len: 3; sym: -35 ),
{ 137: } ( len: 3; sym: -35 ),
{ 138: } ( len: 3; sym: -35 ),
{ 139: } ( len: 1; sym: -35 ),
{ 140: } ( len: 3; sym: -36 ),
{ 141: } ( len: 1; sym: -38 ),
{ 142: } ( len: 0; sym: -38 ),
{ 143: } ( len: 1; sym: -37 ),
{ 144: } ( len: 1; sym: -37 ),
{ 145: } ( len: 1; sym: -37 ),
{ 146: } ( len: 1; sym: -37 ),
{ 147: } ( len: 3; sym: -37 ),
{ 148: } ( len: 3; sym: -37 ),
{ 149: } ( len: 2; sym: -37 ),
{ 150: } ( len: 2; sym: -37 ),
{ 151: } ( len: 2; sym: -37 ),
{ 152: } ( len: 2; sym: -37 ),
{ 153: } ( len: 4; sym: -37 ),
{ 154: } ( len: 4; sym: -37 ),
{ 155: } ( len: 5; sym: -37 ),
{ 156: } ( len: 6; sym: -37 ),
{ 157: } ( len: 4; sym: -37 ),
{ 158: } ( len: 3; sym: -37 ),
{ 159: } ( len: 8; sym: -37 ),
{ 160: } ( len: 4; sym: -37 ),
{ 161: } ( len: 3; sym: -21 ),
{ 162: } ( len: 1; sym: -21 ),
{ 163: } ( len: 0; sym: -21 ),
{ 164: } ( len: 3; sym: -40 ),
{ 165: } ( len: 1; sym: -40 ),
{ 166: } ( len: 1; sym: -23 ),
{ 167: } ( len: 2; sym: -22 ),
{ 168: } ( len: 4; sym: -22 ),
{ 169: } ( len: 3; sym: -39 ),
{ 170: } ( len: 1; sym: -39 ),
{ 171: } ( len: 0; sym: -39 ),
{ 172: } ( len: 1; sym: -41 )
);


const _error = 256; (* error token *)

function yyact(state, sym : Integer; var act : Integer) : Boolean;
  (* search action table *)
  var k : Integer;
  begin
    k := yyal[state];
    while (k<=yyah[state]) and (yya[k].sym<>sym) do inc(k);
    if k>yyah[state] then
      yyact := false
    else
      begin
        act := yya[k].act;
        yyact := true;
      end;
  end(*yyact*);

function yygoto(state, sym : Integer; var nstate : Integer) : Boolean;
  (* search goto table *)
  var k : Integer;
  begin
    k := yygl[state];
    while (k<=yygh[state]) and (yyg[k].sym<>sym) do inc(k);
    if k>yygh[state] then
      yygoto := false
    else
      begin
        nstate := yyg[k].act;
        yygoto := true;
      end;
  end(*yygoto*);

label parse, next, error, errlab, shift, reduce, accept, abort;

begin(*yyparse*)

  (* initialize: *)

  yystate := 0; yychar := -1; yynerrs := 0; yyerrflag := 0; yysp := 0;

{$ifdef yydebug}
  yydebug := true;
{$else}
  yydebug := false;
{$endif}

parse:

  (* push state and value: *)

  inc(yysp);
  if yysp>yymaxdepth then
    begin
      yyerror('yyparse stack overflow');
      goto abort;
    end;
  yys[yysp] := yystate; yyv[yysp] := yyval;

next:

  if (yyd[yystate]=0) and (yychar=-1) then
    (* get next symbol *)
    begin
      yychar := yylex; if yychar<0 then yychar := 0;
    end;

  if yydebug then writeln('state ', yystate, ', char ', yychar);

  (* determine parse action: *)

  yyn := yyd[yystate];
  if yyn<>0 then goto reduce; (* simple state *)

  (* no default action; search parse table *)

  if not yyact(yystate, yychar, yyn) then goto error
  else if yyn>0 then                      goto shift
  else if yyn<0 then                      goto reduce
  else                                    goto accept;

error:

  (* error; start error recovery: *)

  if yyerrflag=0 then yyerror('syntax error');

errlab:

  if yyerrflag=0 then inc(yynerrs);     (* new error *)

  if yyerrflag<=2 then                  (* incomplete recovery; try again *)
    begin
      yyerrflag := 3;
      (* uncover a state with shift action on error token *)
      while (yysp>0) and not ( yyact(yys[yysp], _error, yyn) and
                               (yyn>0) ) do
        begin
          if yydebug then
            if yysp>1 then
              writeln('error recovery pops state ', yys[yysp], ', uncovers ',
                      yys[yysp-1])
            else
              writeln('error recovery fails ... abort');
          dec(yysp);
        end;
      if yysp=0 then goto abort; (* parser has fallen from stack; abort *)
      yystate := yyn;            (* simulate shift on error *)
      goto parse;
    end
  else                                  (* no shift yet; discard symbol *)
    begin
      if yydebug then writeln('error recovery discards char ', yychar);
      if yychar=0 then goto abort; (* end of input; abort *)
      yychar := -1; goto next;     (* clear lookahead char and try again *)
    end;

shift:

  (* go to new state, clear lookahead character: *)

  yystate := yyn; yychar := -1; yyval := yylval;
  if yyerrflag>0 then dec(yyerrflag);

  goto parse;

reduce:

  (* execute action, pop rule from stack, and go to next state: *)

  if yydebug then writeln('reduce ', -yyn);

  yyflag := yyfnone; yyaction(-yyn);
  dec(yysp, yyr[-yyn].len);
  if yygoto(yys[yysp], yyr[-yyn].sym, yyn) then yystate := yyn;

  (* handle action calls to yyaccept, yyabort and yyerror: *)

  case yyflag of
    yyfaccept : goto accept;
    yyfabort  : goto abort;
    yyferror  : goto errlab;
  end;

  goto parse;

accept:

  yyparse := 0; exit;

abort:

  yyparse := 1; exit;

end(*yyparse*);


function yylex : Integer;
begin
  yylex:=scan.yylex;
  line_no:=yylineno;
end;

procedure WriteFileHeader(var headerfile: Text);
var
 i: integer;
 originalstr: string;
begin
{ write unit header }
  if not includefile then
   begin
     if createdynlib then
       writeln(headerfile,'{$mode objfpc}');
     writeln(headerfile,'unit ',unitname,';');
     writeln(headerfile,'interface');
     writeln(headerfile);
     if UseCTypesUnit then
     begin
       writeln(headerfile,'uses');
       writeln(headerfile,'  ctypes;');
       writeln(headerfile);
     end;
     writeln(headerfile,'{');
     writeln(headerfile,'  Automatically converted by H2Pas ',version,' from ',inputfilename);
     writeln(headerfile,'  The following command line parameters were used:');
     for i:=1 to paramcount do
       writeln(headerfile,'    ',paramstr(i));
     writeln(headerfile,'}');
     writeln(headerfile);
   end;
  if UseName then
   begin
     writeln(headerfile,aktspace,'const');
     writeln(headerfile,aktspace,'  External_library=''',libfilename,'''; {Setup as you need}');
     writeln(headerfile);
   end;
  if UsePPointers then
   begin
     Writeln(headerfile,aktspace,'{ Pointers to basic pascal types, inserted by h2pas conversion program.}');
     Writeln(headerfile,aktspace,'Type');
     Writeln(headerfile,aktspace,'  PLongint  = ^Longint;');
     Writeln(headerfile,aktspace,'  PSmallInt = ^SmallInt;');
     Writeln(headerfile,aktspace,'  PByte     = ^Byte;');
     Writeln(headerfile,aktspace,'  PWord     = ^Word;');
     Writeln(headerfile,aktspace,'  PDWord    = ^DWord;');
     Writeln(headerfile,aktspace,'  PDouble   = ^Double;');
     Writeln(headerfile);
   end;
  if PTypeList.count <> 0 then
   Writeln(headerfile,aktspace,'Type');
  for i:=0 to (PTypeList.Count-1) do
   begin
     originalstr:=copy(PTypelist[i],2,length(PTypeList[i]));
     Writeln(headerfile,aktspace,PTypeList[i],'  = ^',originalstr,';');
   end;
  if not packrecords then
   begin
      writeln(headerfile,'{$IFDEF FPC}');
      writeln(headerfile,'{$PACKRECORDS C}');
      writeln(headerfile,'{$ENDIF}');
   end;
  writeln(headerfile);
end;


var
  SS : string;
  i : longint;
  headerfile: Text;
  finaloutfile: Text;
begin
  pointerprefix:=false;
{ Initialize }
  PTypeList:=TStringList.Create;
  PTypeList.Sorted := true;
  PTypeList.Duplicates := dupIgnore;
  freedynlibproc:=TStringList.Create;
  loaddynlibproc:=TStringList.Create;
  yydebug:=true;
  aktspace:='';
  block_type:=bt_no;
  IsExtern:=false;
{ Read commandline options }
  ProcessOptions;
  if not CompactMode then
   aktspace:='  ';
{ open input and output files }
  assign(yyinput, inputfilename);
  {$I-}
   reset(yyinput);
  {$I+}
  if ioresult<>0 then
   begin
     writeln('file ',inputfilename,' not found!');
     halt(1);
   end;
  { This is the intermediate output file }
  assign(outfile, 'ext3.tmp');
  {$I-}
  rewrite(outfile);
  {$I+}
  if ioresult<>0 then
   begin
     writeln('file ext3.tmp could not be created!');
     halt(1);
   end;
  writeln(outfile);
{ Open tempfiles }
  { This is where the implementation section of the unit shall be stored }
  Assign(implemfile,'ext.tmp');
  rewrite(implemfile);
  Assign(tempfile,'ext2.tmp');
  rewrite(tempfile);
{ Parse! }
  yyparse;
{ Write implementation if needed }
   if not(includefile) then
    begin
      writeln(outfile);
      writeln(outfile,'implementation');
      writeln(outfile);
    end;
   { here we have a problem if a line is longer than 255 chars !! }
   reset(implemfile);
   while not eof(implemfile) do
    begin
      readln(implemfile,SS);
      writeln(outfile,SS);
    end;

  if createdynlib then
    begin
      writeln(outfile,'  uses');
      writeln(outfile,'    SysUtils, dynlibs;');
      writeln(outfile);
      writeln(outfile,'  var');
      writeln(outfile,'    hlib : tlibhandle;');
      writeln(outfile);
      writeln(outfile);
      writeln(outfile,'  procedure Free',unitname,';');
      writeln(outfile,'    begin');
      writeln(outfile,'      FreeLibrary(hlib);');

      for i:=0 to (freedynlibproc.Count-1) do
        Writeln(outfile,'      ',freedynlibproc[i]);

      writeln(outfile,'    end;');
      writeln(outfile);
      writeln(outfile);
      writeln(outfile,'  procedure Load',unitname,'(lib : pchar);');
      writeln(outfile,'    begin');
      writeln(outfile,'      Free',unitname,';');
      writeln(outfile,'      hlib:=LoadLibrary(lib);');
      writeln(outfile,'      if hlib=0 then');
      writeln(outfile,'        raise Exception.Create(format(''Could not load library: %s'',[lib]));');
      writeln(outfile);
      for i:=0 to (loaddynlibproc.Count-1) do
        Writeln(outfile,'      ',loaddynlibproc[i]);
      writeln(outfile,'    end;');

      writeln(outfile);
      writeln(outfile);

      writeln(outfile,'initialization');
      writeln(outfile,'  Load',unitname,'(''',unitname,''');');
      writeln(outfile,'finalization');
      writeln(outfile,'  Free',unitname,';');
    end;

   { write end of file }
   writeln(outfile);
   if not(includefile) then
     writeln(outfile,'end.');
   { close and erase tempfiles }
  close(implemfile);
  erase(implemfile);
  close(tempfile);
  erase(tempfile);
  flush(outfile);

  {**** generate full file ****}
  assign(headerfile, 'ext4.tmp');
  {$I-}
  rewrite(headerfile);
  {$I+}
  if ioresult<>0 then
    begin
      writeln('file ext4.tmp could not be created!');
      halt(1);
  end;
  WriteFileHeader(HeaderFile);

  { Final output filename }
  assign(finaloutfile, outputfilename);
  {$I-}
  rewrite(finaloutfile);
  {$I+}
  if ioresult<>0 then
  begin
     writeln('file ',outputfilename,' could not be created!');
     halt(1);
  end;
  writeln(finaloutfile);

  { Read unit header file }
  reset(headerfile);
  while not eof(headerfile) do
    begin
      readln(headerfile,SS);
      writeln(finaloutfile,SS);
    end;
  { Read interface and implementation file }
  reset(outfile);
  while not eof(outfile) do
    begin
      readln(outfile,SS);
      writeln(finaloutfile,SS);
    end;

  close(HeaderFile);
  close(outfile);
  close(finaloutfile);
  erase(outfile);
  erase(headerfile);

  PTypeList.Free;
  freedynlibproc.free;
  loaddynlibproc.free;
end.