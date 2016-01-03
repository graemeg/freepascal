<<<<<<< HEAD
<<<<<<< HEAD
%{
program h2pas;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$H+}
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
=======
%{
program h2pas;

>>>>>>> origin/fixes_2.4
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


<<<<<<< HEAD
=======

>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
         shift(2);
=======
         shift(3);
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
                        shift(2);
=======
                        shift(3);
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
                        shift(2);
=======
                        shift(3);
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
=======
                  if not compactmode then
                   shift(2);
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
                  shift(2);
=======
                  shift(3);
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
       old_in_args : boolean = false;
=======
>>>>>>> origin/fixes_2.4

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
                   (* Needs 2 declarations, also one without args, becuase
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
<<<<<<< HEAD
         old_in_args : boolean;
=======

>>>>>>> origin/fixes_2.4
      begin
         if not(assigned(p)) then
           begin
              write_type_specifier(outfile,simple_type);
              exit;
           end;
         case p^.typ of
<<<<<<< HEAD
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
=======
            t_pointerdef : begin
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
                                        write_p_a_def(outfile,p^.p1^.p1,simple_type);
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
            t_arraydef : begin
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
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
                      if UseCTypesUnit and IsACType(p^.p1^.p) then
                        write(outfile,'p')
                      else
                        write(outfile,'P');
=======
                      if UseCTypesUnit and (IsACType(p^.p1^.p)=False) then
                        write(outfile,'P')
                      else
                        write(outfile,'p');
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
                      shift(2);
=======
                      shift(3);
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                if assigned(hp3^.p1) and
                                   (not assigned(hp3^.p1^.p3) or
                                   (hp3^.p1^.p3^.typ <> t_size_specifier)) then
=======
                                if not assigned(hp3^.p1^.p3) or
                                   (hp3^.p1^.p3^.typ <> t_size_specifier) then
>>>>>>> graemeg/cpstrnew
=======
                                if not assigned(hp3^.p1^.p3) or
                                   (hp3^.p1^.p3^.typ <> t_size_specifier) then
>>>>>>> graemeg/cpstrnew
=======
                                if not assigned(hp3^.p1^.p3) or
                                   (hp3^.p1^.p3^.typ <> t_size_specifier) then
>>>>>>> graemeg/cpstrnew
=======
                                if not assigned(hp3^.p1^.p3) or
                                   (hp3^.p1^.p3^.typ <> t_size_specifier) then
>>>>>>> origin/cpstrnew
=======
                                if not assigned(hp3^.p1^.p3) or
                                   (hp3^.p1^.p3^.typ <> t_size_specifier) then
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                if assigned(hp3^.p1) and
                                   assigned(hp3^.p1^.p3) then
=======
                                if assigned(hp3^.p1^.p3) then
>>>>>>> graemeg/cpstrnew
=======
                                if assigned(hp3^.p1^.p3) then
>>>>>>> graemeg/cpstrnew
=======
                                if assigned(hp3^.p1^.p3) then
>>>>>>> graemeg/cpstrnew
=======
                                if assigned(hp3^.p1^.p3) then
>>>>>>> origin/cpstrnew
=======
                                if assigned(hp3^.p1^.p3) then
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
                                            write(outfile,';cdecl');
=======
                                            begin
                                               write(outfile,';cdecl');
                                               no_pop:=true;
                                            end;
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
                      shift(2);
=======
                      shift(3);
>>>>>>> origin/fixes_2.4
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

<<<<<<< HEAD

=======
>>>>>>> origin/fixes_2.4
%}

%token _WHILE _FOR _DO _GOTO _CONTINUE _BREAK
%token TYPEDEF DEFINE
%token COLON SEMICOLON COMMA
%token LKLAMMER RKLAMMER LECKKLAMMER RECKKLAMMER
%token LGKLAMMER RGKLAMMER
%token STRUCT UNION ENUM
%token ID NUMBER CSTRING
%token SHORT UNSIGNED LONG INT FLOAT _CHAR
%token VOID _CONST
%token _FAR _HUGE _NEAR
%token NEW_LINE SPACE_DEFINE
%token EXTERN STDCALL CDECL CALLBACK PASCAL WINAPI APIENTRY WINGDIAPI SYS_TRAP
%token _PACKED
%token ELLIPSIS
%right _ASSIGN
%right R_AND
%left EQUAL UNEQUAL GT LT GTE LTE
%left QUESTIONMARK COLON
%left _OR
%left _AND
%left _PLUS MINUS
%left _SHR _SHL
%left STAR _SLASH
%right _NOT
%right LKLAMMER
%right PSTAR
%right P_AND
%right LECKKLAMMER
%left POINT DEREF
%left COMMA
%left STICK
%token SIGNED
%token INT8 INT16 INT32 INT64
%%

file : declaration_list
     ;

maybe_space :
     SPACE_DEFINE
     {
       $$:=nil;
     } |
     {
       $$:=nil;
     }
     ;

error_info : {
                  writeln(outfile,'(* error ');
                  writeln(outfile,yyline);
             };

declaration_list : declaration_list  declaration
     {  if yydebug then writeln('declaration reduced at line ',line_no);
        if yydebug then writeln(outfile,'(* declaration reduced *)');
     }
     | declaration_list define_dec
     {  if yydebug then writeln('define declaration reduced at line ',line_no);
        if yydebug then writeln(outfile,'(* define declaration reduced *)');
     }
     | declaration
     {  if yydebug then writeln('declaration reduced at line ',line_no);
     }
     | define_dec
     {  if yydebug then writeln('define declaration reduced at line ',line_no);
     }
     ;

dec_specifier :
     EXTERN { $$:=new(presobject,init_id('extern')); }
     |{ $$:=new(presobject,init_id('intern')); }
     ;

dec_modifier :
     STDCALL { $$:=new(presobject,init_id('no_pop')); }
     | CDECL { $$:=new(presobject,init_id('cdecl')); }
     | CALLBACK { $$:=new(presobject,init_id('no_pop')); }
     | PASCAL { $$:=new(presobject,init_id('no_pop')); }
     | WINAPI { $$:=new(presobject,init_id('no_pop')); }
     | APIENTRY { $$:=new(presobject,init_id('no_pop')); }
     | WINGDIAPI { $$:=new(presobject,init_id('no_pop')); }
     | { $$:=nil }
     ;

systrap_specifier:
     SYS_TRAP LKLAMMER dname RKLAMMER { $$:=$3; }
     | { $$:=nil; }
     ;

statement :
     expr SEMICOLON { $$:=$1; } |
     _WHILE LKLAMMER expr RKLAMMER statement_list { $$:=new(presobject,init_two(t_whilenode,$3,$5)); }
     ;


statement_list : statement statement_list
     {
       $$:=new(presobject,init_one(t_statement_list,$1));
       $$^.next:=$2;
     } |
     statement
     {
       $$:=new(presobject,init_one(t_statement_list,$1));
     } |
     SEMICOLON
     {
       $$:=new(presobject,init_one(t_statement_list,nil));
     } |
     {
       $$:=new(presobject,init_one(t_statement_list,nil));
     }
     ;

statement_block :
     LGKLAMMER statement_list RGKLAMMER { $$:=$2; }
     ;

declaration :
     dec_specifier type_specifier dec_modifier declarator_list statement_block
     {
       IsExtern:=false;
       (* by default we must pop the args pushed on stack *)
       no_pop:=false;
       if (assigned($4)and assigned($4^.p1)and assigned($4^.p1^.p1))
         and ($4^.p1^.p1^.typ=t_procdef) then
          begin
             repeat
             If UseLib then
               IsExtern:=true
             else
               IsExtern:=assigned($1)and($1^.str='extern');
             no_pop:=assigned($3) and ($3^.str='no_pop');

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
             if assigned($2) then
              if ($2^.typ=t_void) and ($4^.p1^.p1^.p1=nil) then
               begin
                 if createdynlib then
                   begin
                     write(outfile,$4^.p1^.p2^.p,' : procedure');
                   end
                 else
                   begin
                     shift(10);
                     write(outfile,'procedure ',$4^.p1^.p2^.p);
                   end;
                 if assigned($4^.p1^.p1^.p2) then
                   write_args(outfile,$4^.p1^.p1^.p2);
                 if createdynlib then
                    begin
                      loaddynlibproc.add('pointer('+$4^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+$4^.p1^.p2^.p+''');');
                      freedynlibproc.add($4^.p1^.p2^.p+':=nil;');
                    end
                  else if not IsExtern then
                  begin
                    write(implemfile,'procedure ',$4^.p1^.p2^.p);
                    if assigned($4^.p1^.p1^.p2) then
                     write_args(implemfile,$4^.p1^.p1^.p2);
                  end;
               end
             else
               begin
                 if createdynlib then
                   begin
                     write(outfile,$4^.p1^.p2^.p,' : function');
                   end
                 else
                   begin
                     shift(9);
                     write(outfile,'function ',$4^.p1^.p2^.p);
                   end;

                  if assigned($4^.p1^.p1^.p2) then
                    write_args(outfile,$4^.p1^.p1^.p2);
                  write(outfile,':');
<<<<<<< HEAD
                  old_in_args:=in_args;
                  (* write pointers as P.... instead of ^.... *)
                  in_args:=true;
                  write_p_a_def(outfile,$4^.p1^.p1^.p1,$2);
                  in_args:=old_in_args;
=======
                  write_p_a_def(outfile,$4^.p1^.p1^.p1,$2);
>>>>>>> origin/fixes_2.4
                  if createdynlib then
                    begin
                      loaddynlibproc.add('pointer('+$4^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+$4^.p1^.p2^.p+''');');
                      freedynlibproc.add($4^.p1^.p2^.p+':=nil;');
                    end
                  else if not IsExtern then
                   begin
                     write(implemfile,'function ',$4^.p1^.p2^.p);
                     if assigned($4^.p1^.p1^.p2) then
<<<<<<< HEAD
                       write_args(implemfile,$4^.p1^.p1^.p2);
                     write(implemfile,':');

                     old_in_args:=in_args;
                     (* write pointers as P.... instead of ^.... *)
                     in_args:=true;
                     write_p_a_def(implemfile,$4^.p1^.p1^.p1,$2);
                     in_args:=old_in_args;
=======
                      write_args(implemfile,$4^.p1^.p1^.p2);
                     write(implemfile,':');
                     write_p_a_def(implemfile,$4^.p1^.p1^.p1,$2);
>>>>>>> origin/fixes_2.4
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
                     Write(outfile,' External_library name ''',$4^.p1^.p2^.p,'''');
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
                    if $5^.typ=t_statement_list then
                      write_statement_block(implemfile,$5);
                    popshift;
                  end;
               end;
             IsExtern:=false;
             if not(compactmode) and not(createdynlib) then
              writeln(outfile);
            until not NeedEllipsisOverload;
          end
        else (* $4^.p1^.p1^.typ=t_procdef *)
        if assigned($4)and assigned($4^.p1) then
          begin
             shift(2);
             if block_type<>bt_var then
               begin
                  if not(compactmode) then
                    writeln(outfile);
                  writeln(outfile,aktspace,'var');
               end;
             block_type:=bt_var;

<<<<<<< HEAD
             shift(2);
=======
             shift(3);
>>>>>>> origin/fixes_2.4

             IsExtern:=assigned($1)and($1^.str='extern');
             (* walk through all declarations *)
             hp:=$4;
             while assigned(hp) and assigned(hp^.p1) do
               begin
                  (* write new var name *)
                  if assigned(hp^.p1^.p2) and assigned(hp^.p1^.p2^.p) then
                    write(outfile,aktspace,hp^.p1^.p2^.p);
                  write(outfile,' : ');
                  shift(2);
                  (* write its type *)
                  write_p_a_def(outfile,hp^.p1^.p1,$2);
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
        if assigned($1) then
          dispose($1,done);
        if assigned($2) then
          dispose($2,done);
        if assigned($3) then
          dispose($3,done);
        if assigned($4) then
          dispose($4,done);
        if assigned($5) then
          dispose($5,done);
     }
     | dec_specifier type_specifier dec_modifier declarator_list systrap_specifier SEMICOLON
     {
       IsExtern:=false;
       (* by default we must pop the args pushed on stack *)
       no_pop:=false;
       if (assigned($4)and assigned($4^.p1)and assigned($4^.p1^.p1))
         and ($4^.p1^.p1^.typ=t_procdef) then
          begin
             repeat
             If UseLib then
               IsExtern:=true
             else
               IsExtern:=assigned($1)and($1^.str='extern');
             no_pop:=assigned($3) and ($3^.str='no_pop');

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
             if assigned($2) then
              if ($2^.typ=t_void) and ($4^.p1^.p1^.p1=nil) then
               begin
                 if createdynlib then
                   begin
                     write(outfile,$4^.p1^.p2^.p,' : procedure');
                   end
                 else
                   begin
                     shift(10);
                     write(outfile,'procedure ',$4^.p1^.p2^.p);
                   end;
                 if assigned($4^.p1^.p1^.p2) then
                   write_args(outfile,$4^.p1^.p1^.p2);
                 if createdynlib then
                    begin
                      loaddynlibproc.add('pointer('+$4^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+$4^.p1^.p2^.p+''');');
                      freedynlibproc.add($4^.p1^.p2^.p+':=nil;');
                    end
                  else if not IsExtern then
                  begin
                    write(implemfile,'procedure ',$4^.p1^.p2^.p);
                    if assigned($4^.p1^.p1^.p2) then
                     write_args(implemfile,$4^.p1^.p1^.p2);
                  end;
               end
             else
               begin
                 if createdynlib then
                   begin
                     write(outfile,$4^.p1^.p2^.p,' : function');
                   end
                 else
                   begin
                     shift(9);
                     write(outfile,'function ',$4^.p1^.p2^.p);
                   end;

                  if assigned($4^.p1^.p1^.p2) then
                    write_args(outfile,$4^.p1^.p1^.p2);
                  write(outfile,':');
                  write_p_a_def(outfile,$4^.p1^.p1^.p1,$2);
                  if createdynlib then
                    begin
                      loaddynlibproc.add('pointer('+$4^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+$4^.p1^.p2^.p+''');');
                      freedynlibproc.add($4^.p1^.p2^.p+':=nil;');
                    end
                  else if not IsExtern then
                   begin
                     write(implemfile,'function ',$4^.p1^.p2^.p);
                     if assigned($4^.p1^.p1^.p2) then
                      write_args(implemfile,$4^.p1^.p1^.p2);
                     write(implemfile,':');
<<<<<<< HEAD

                     old_in_args:=in_args;
                     (* write pointers as P.... instead of ^.... *)
                     in_args:=true;
                     write_p_a_def(implemfile,$4^.p1^.p1^.p1,$2);
                     in_args:=old_in_args;
=======
                     write_p_a_def(implemfile,$4^.p1^.p1^.p1,$2);
>>>>>>> origin/fixes_2.4
                   end;
               end;
             if assigned($5) then
               write(outfile,';systrap ',$5^.p);
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
                     Write(outfile,' External_library name ''',$4^.p1^.p2^.p,'''');
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
        else (* $4^.p1^.p1^.typ=t_procdef *)
        if assigned($4)and assigned($4^.p1) then
          begin
             shift(2);
             if block_type<>bt_var then
               begin
                  if not(compactmode) then
                    writeln(outfile);
                  writeln(outfile,aktspace,'var');
               end;
             block_type:=bt_var;

<<<<<<< HEAD
             shift(2);
=======
             shift(3);
>>>>>>> origin/fixes_2.4

             IsExtern:=assigned($1)and($1^.str='extern');
             (* walk through all declarations *)
             hp:=$4;
             while assigned(hp) and assigned(hp^.p1) do
               begin
                  (* write new var name *)
                  if assigned(hp^.p1^.p2) and assigned(hp^.p1^.p2^.p) then
                    write(outfile,aktspace,hp^.p1^.p2^.p);
                  write(outfile,' : ');
                  shift(2);
                  (* write its type *)
                  write_p_a_def(outfile,hp^.p1^.p1,$2);
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
        if assigned($1)then  dispose($1,done);
        if assigned($2)then  dispose($2,done);
        if assigned($4)then  dispose($4,done);
     } |
     special_type_specifier SEMICOLON
     {
       if block_type<>bt_type then
         begin
            if not(compactmode) then
              writeln(outfile);
            writeln(outfile,aktspace,'type');
            block_type:=bt_type;
         end;
<<<<<<< HEAD
       shift(2);
=======
       shift(3);
>>>>>>> origin/fixes_2.4
       if ( yyv[yysp-1]^.p2  <> nil ) then
         begin
           (* write new type name *)
           TN:=TypeName($1^.p2^.p);
           PN:=PointerName($1^.p2^.p);
           (* define a Pointer type also for structs *)
           if UsePPointers and (Uppercase(tn)<>Uppercase(pn)) and
              assigned($1) and ($1^.typ in [t_uniondef,t_structdef]) then
            writeln(outfile,aktspace,PN,' = ^',TN,';');
           write(outfile,aktspace,TN,' = ');
           shift(2);
           hp:=$1;
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
     } |
     TYPEDEF STRUCT dname dname SEMICOLON
     {
       (* TYPEDEF STRUCT dname dname SEMICOLON *)
       if block_type<>bt_type then
         begin
            if not(compactmode) then
              writeln(outfile);
            writeln(outfile,aktspace,'type');
            block_type:=bt_type;
         end;
       PN:=TypeName($3^.p);
       TN:=TypeName($4^.p);
       if Uppercase(tn)<>Uppercase(pn) then
        begin
<<<<<<< HEAD
          shift(2);
=======
          shift(3);
>>>>>>> origin/fixes_2.4
          writeln(outfile,aktspace,PN,' = ',TN,';');
          popshift;
        end;
       if assigned($3) then
        dispose($3,done);
       if assigned($4) then
        dispose($4,done);
     } |
     TYPEDEF type_specifier LKLAMMER dec_modifier declarator RKLAMMER maybe_space LKLAMMER argument_declaration_list RKLAMMER SEMICOLON
     {
       (* TYPEDEF type_specifier LKLAMMER dec_modifier declarator RKLAMMER maybe_space LKLAMMER argument_declaration_list RKLAMMER SEMICOLON *)
       if block_type<>bt_type then
         begin
            if not(compactmode) then
              writeln(outfile);
            writeln(outfile,aktspace,'type');
            block_type:=bt_type;
         end;
       no_pop:=assigned($4) and ($4^.str='no_pop');
<<<<<<< HEAD
       shift(2);
=======
       shift(3);
>>>>>>> origin/fixes_2.4
       (* walk through all declarations *)
       hp:=$5;
       if assigned(hp) then
        begin
          hp:=$5;
          while assigned(hp^.p1) do
           hp:=hp^.p1;
          hp^.p1:=new(presobject,init_two(t_procdef,nil,$9));
          hp:=$5;
          if assigned(hp^.p1) and assigned(hp^.p1^.p1) then
           begin
             writeln(outfile);
             (* write new type name *)
             write(outfile,aktspace,TypeName(hp^.p2^.p),' = ');
             shift(2);
             write_p_a_def(outfile,hp^.p1,$2);
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
       if assigned($2)then
       dispose($2,done);
       if assigned($4)then
       dispose($4,done);
       if assigned($5)then (* disposes also $9 *)
       dispose($5,done);
     } |
     TYPEDEF type_specifier dec_modifier declarator_list SEMICOLON
     {
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
       no_pop:=assigned($3) and ($3^.str='no_pop');
<<<<<<< HEAD
       shift(2);
=======
       shift(3);
>>>>>>> origin/fixes_2.4
       (* Get the name to write the type definition for, try
          to use the tag name first *)
       if assigned($2^.p2) then
        begin
          ph:=$2^.p2;
        end
       else
        begin
          if not assigned($4^.p1^.p2) then
           internalerror(4444);
          ph:=$4^.p1^.p2;
        end;
       (* write type definition *)
       is_procvar:=false;
       TN:=TypeName(ph^.p);
       PN:=PointerName(ph^.p);
       if UsePPointers and (Uppercase(tn)<>Uppercase(pn)) and
          assigned($2) and ($2^.typ<>t_procdef) then
         writeln(outfile,aktspace,PN,' = ^',TN,';');
       (* write new type name *)
       write(outfile,aktspace,TN,' = ');
       shift(2);
       write_p_a_def(outfile,$4^.p1^.p1,$2);
       popshift;
       (* if no_pop it is normal fpc calling convention *)
       if is_procvar and
          (not no_pop) then
         write(outfile,';cdecl');
       writeln(outfile,';');
       flush(outfile);
       (* write alias names, ph points to the name already used *)
       hp:=$4;
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
                  assigned($2) and ($2^.typ<>t_procdef) then
                 writeln(outfile,aktspace,PN,' = ^',TN,';');
              end;
           end;
          hp:=hp^.next;
        end;
       popshift;
       if must_write_packed_field then
         if assigned(ph) then
           write_packed_fields_info(outfile,$2,ph^.str)
         else if assigned($2^.p2) then
           write_packed_fields_info(outfile,$2,$2^.p2^.str);
       if assigned($2)then
       dispose($2,done);
       if assigned($3)then
       dispose($3,done);
       if assigned($4)then
       dispose($4,done);
     } |
     TYPEDEF dname SEMICOLON
     {
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
       writeln(outfile,aktspace,$2^.p,' = pointer;');
       flush(outfile);
       popshift;
       if assigned($2) then
        dispose($2,done);
     }
     | error  error_info SEMICOLON
      { writeln(outfile,'in declaration at line ',line_no,' *)');
        aktspace:='';
        in_space_define:=0;
        in_define:=false;
        arglevel:=0;
        if_nb:=0;
        aktspace:='    ';
        space_index:=1;
        yyerrok;}
     ;

define_dec :
     DEFINE dname LKLAMMER enum_list RKLAMMER para_def_expr NEW_LINE
     {
       (* DEFINE dname LKLAMMER enum_list RKLAMMER para_def_expr NEW_LINE *)
       if not stripinfo then
        begin
          writeln (outfile,aktspace,'{ was #define dname(params) para_def_expr }');
          writeln (implemfile,aktspace,'{ was #define dname(params) para_def_expr }');
          if assigned($4) then
           begin
             writeln (outfile,aktspace,'{ argument types are unknown }');
             writeln (implemfile,aktspace,'{ argument types are unknown }');
           end;
          if not assigned($6^.p3) then
           begin
             writeln(outfile,aktspace,'{ return type might be wrong }   ');
             writeln(implemfile,aktspace,'{ return type might be wrong }   ');
           end;
        end;
<<<<<<< HEAD
       if block_type<>bt_func then
         writeln(outfile);

=======
>>>>>>> origin/fixes_2.4
       block_type:=bt_func;
       write(outfile,aktspace,'function ',$2^.p);
       write(implemfile,aktspace,'function ',$2^.p);

       if assigned($4) then
         begin
            write(outfile,'(');
            write(implemfile,'(');
            ph:=new(presobject,init_one(t_enumdef,$4));
            write_def_params(outfile,ph);
            write_def_params(implemfile,ph);
            if assigned(ph) then dispose(ph,done);
            ph:=nil;
            (* types are unknown *)
            write(outfile,' : longint)');
            write(implemfile,' : longint)');
         end;
       if not assigned($6^.p3) then
         begin
            writeln(outfile,' : longint;',aktspace,commentstr);
            writeln(implemfile,' : longint;');
            flush(outfile);
         end
       else
         begin
            write(outfile,' : ');
            write_type_specifier(outfile,$6^.p3);
            writeln(outfile,';',aktspace,commentstr);
            flush(outfile);
            write(implemfile,' : ');
            write_type_specifier(implemfile,$6^.p3);
            writeln(implemfile,';');
         end;
       writeln(outfile);
       flush(outfile);
       hp:=new(presobject,init_two(t_funcname,$2,$6));
       write_funexpr(implemfile,hp);
       writeln(implemfile);
       flush(implemfile);
       if assigned(hp)then dispose(hp,done);
     }|
     DEFINE dname SPACE_DEFINE NEW_LINE
     {
       (* DEFINE dname SPACE_DEFINE NEW_LINE *)
       writeln(outfile,'{$define ',$2^.p,'}',aktspace,commentstr);
       flush(outfile);
       if assigned($2)then
        dispose($2,done);
     }|
     DEFINE dname NEW_LINE
     {
       writeln(outfile,'{$define ',$2^.p,'}',aktspace,commentstr);
       flush(outfile);
       if assigned($2)then
        dispose($2,done);
     } |
     DEFINE dname SPACE_DEFINE def_expr NEW_LINE
     {
       (* DEFINE dname SPACE_DEFINE def_expr NEW_LINE *)
       if ($4^.typ=t_exprlist) and
          $4^.p1^.is_const and
          not assigned($4^.next) then
         begin
            if block_type<>bt_const then
              begin
<<<<<<< HEAD
                if block_type<>bt_func then
                  writeln(outfile);
                writeln(outfile,aktspace,'const');
              end;
            block_type:=bt_const;
            shift(2);
=======
                 writeln(outfile);
                 writeln(outfile,aktspace,'const');
              end;
            block_type:=bt_const;
            shift(3);
>>>>>>> origin/fixes_2.4
            write(outfile,aktspace,$2^.p);
            write(outfile,' = ');
            flush(outfile);
            write_expr(outfile,$4^.p1);
            writeln(outfile,';',aktspace,commentstr);
            popshift;
            if assigned($2) then
            dispose($2,done);
            if assigned($4) then
            dispose($4,done);
         end
       else
         begin
<<<<<<< HEAD
            if block_type<>bt_func then
              writeln(outfile);
=======
>>>>>>> origin/fixes_2.4
            if not stripinfo then
             begin
               writeln (outfile,aktspace,'{ was #define dname def_expr }');
               writeln (implemfile,aktspace,'{ was #define dname def_expr }');
             end;
            block_type:=bt_func;
            write(outfile,aktspace,'function ',$2^.p);
            write(implemfile,aktspace,'function ',$2^.p);
            shift(2);
            if not assigned($4^.p3) then
              begin
<<<<<<< HEAD
                 writeln(outfile,' : longint; { return type might be wrong }');
                 flush(outfile);
                 writeln(implemfile,' : longint; { return type might be wrong }');
=======
                 writeln(outfile,' : longint;');
                 writeln(outfile,aktspace,'  { return type might be wrong }');
                 flush(outfile);
                 writeln(implemfile,' : longint;');
                 writeln(implemfile,aktspace,'  { return type might be wrong }');
>>>>>>> origin/fixes_2.4
              end
            else
              begin
                 write(outfile,' : ');
                 write_type_specifier(outfile,$4^.p3);
                 writeln(outfile,';',aktspace,commentstr);
                 flush(outfile);
                 write(implemfile,' : ');
                 write_type_specifier(implemfile,$4^.p3);
                 writeln(implemfile,';');
              end;
            writeln(outfile);
            flush(outfile);
            hp:=new(presobject,init_two(t_funcname,$2,$4));
            write_funexpr(implemfile,hp);
            popshift;
            dispose(hp,done);
            writeln(implemfile);
            flush(implemfile);
         end;
     }
     | error error_info NEW_LINE
      { writeln(outfile,'in define line ',line_no,' *)');
        aktspace:='';
        in_space_define:=0;
        in_define:=false;
        arglevel:=0;
        if_nb:=0;
        aktspace:='    ';
        space_index:=1;

        yyerrok;}
     ;

closed_list : LGKLAMMER member_list RGKLAMMER
            {$$:=$2;} |
            error  error_info RGKLAMMER
            { writeln(outfile,' in member_list *)');
            yyerrok;
            $$:=nil;
            }
            ;

closed_enum_list : LGKLAMMER enum_list RGKLAMMER
            {$$:=$2;} |
            error  error_info  RGKLAMMER
            { writeln(outfile,' in enum_list *)');
            yyerrok;
            $$:=nil;
            }
            ;

special_type_specifier :
     STRUCT dname closed_list _PACKED
     {
       if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
       is_packed:=true;
       $$:=new(presobject,init_two(t_structdef,$3,$2));
     } |
     STRUCT dname closed_list
     {
       if (is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 4}');
       is_packed:=false;
       $$:=new(presobject,init_two(t_structdef,$3,$2));
     } |
     UNION dname closed_list _PACKED
     {
       if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
       is_packed:=true;
       $$:=new(presobject,init_two(t_uniondef,$3,$2));
     } |
     UNION dname closed_list
     {
       $$:=new(presobject,init_two(t_uniondef,$3,$2));
     } |
     UNION dname
     {
       $$:=$2;
     } |
     STRUCT dname
     {
       $$:=$2;
     } |
     ENUM dname closed_enum_list
     {
       $$:=new(presobject,init_two(t_enumdef,$3,$2));
     } |
     ENUM dname
     {
       $$:=$2;
     };

type_specifier :
      _CONST type_specifier
      {
        if not stripinfo then
         writeln(outfile,'(* Const before type ignored *)');
        $$:=$2;
        } |
     UNION closed_list  _PACKED
     {
       if (not is_packed) and (not packrecords)then
         writeln(outfile,'{$PACKRECORDS 1}');
       is_packed:=true;
       $$:=new(presobject,init_one(t_uniondef,$2));
     } |
     UNION closed_list
     {
       $$:=new(presobject,init_one(t_uniondef,$2));
     } |
     STRUCT closed_list _PACKED
     {
       if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
       is_packed:=true;
       $$:=new(presobject,init_one(t_structdef,$2));
     } |
     STRUCT closed_list
     {
       if (is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 4}');
       is_packed:=false;
       $$:=new(presobject,init_one(t_structdef,$2));
     } |
     ENUM closed_enum_list
     {
       $$:=new(presobject,init_one(t_enumdef,$2));
     } |
     special_type_specifier
     {
       $$:=$1;
     } |
     simple_type_name { $$:=$1; }
     ;

member_list : member_declaration member_list
     {
       $$:=new(presobject,init_one(t_memberdeclist,$1));
       $$^.next:=$2;
     } |
     member_declaration
     {
       $$:=new(presobject,init_one(t_memberdeclist,$1));
     }
     ;

member_declaration :
     type_specifier declarator_list SEMICOLON
     {
       $$:=new(presobject,init_two(t_memberdec,$1,$2));
     }
     ;

dname : ID { (*dname*)
           $$:=new(presobject,init_id(act_token));
           }
     ;
special_type_name :
     SIGNED special_type_name
     {
       hp:=$2;
       $$:=hp;
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
     } |
     UNSIGNED special_type_name
     {
       hp:=$2;
       $$:=hp;
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
     } |
     INT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint_STR))
     else
       $$:=new(presobject,init_intid(INT_STR));
     } |
     LONG
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(clong_STR))
     else
       $$:=new(presobject,init_intid(INT_STR));
     } |
     LONG INT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(clong_STR))
     else
       $$:=new(presobject,init_intid(INT_STR));
     } |
     LONG LONG
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(clonglong_STR))
     else
       $$:=new(presobject,init_intid(INT64_STR));
     } |
     LONG LONG INT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(clonglong_STR))
     else
       $$:=new(presobject,init_intid(INT64_STR));
     } |
     SHORT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cshort_STR))
     else
       $$:=new(presobject,init_intid(SMALL_STR));
     } |
     SHORT INT
     {
     if UseCTypesUnit then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       $$:=new(presobject,init_id(cshort_STR))
=======
       $$:=new(presobject,init_id(csint_STR))
>>>>>>> graemeg/cpstrnew
=======
       $$:=new(presobject,init_id(csint_STR))
>>>>>>> graemeg/cpstrnew
=======
       $$:=new(presobject,init_id(csint_STR))
>>>>>>> graemeg/cpstrnew
=======
       $$:=new(presobject,init_id(csint_STR))
>>>>>>> origin/cpstrnew
=======
       $$:=new(presobject,init_id(csint_STR))
>>>>>>> origin/fixes_2.4
     else
       $$:=new(presobject,init_intid(SMALL_STR));
     } |
     INT8
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint8_STR))
     else
       $$:=new(presobject,init_intid(SHORT_STR));
     } |
     INT16
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint16_STR))
     else
       $$:=new(presobject,init_intid(SMALL_STR));
     } |
     INT32
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint32_STR))
     else
       $$:=new(presobject,init_intid(INT_STR));
     } |
     INT64
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint64_STR))
     else
       $$:=new(presobject,init_intid(INT64_STR));
     } |
     FLOAT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cfloat_STR))
     else
       $$:=new(presobject,init_intid(FLOAT_STR));
     } |
     VOID
     {
       $$:=new(presobject,init_no(t_void));
     } |
     _CHAR
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cchar_STR))
     else
       $$:=new(presobject,init_intid(CHAR_STR));
     } |
     UNSIGNED
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cunsigned_STR))
     else
       $$:=new(presobject,init_intid(UINT_STR));
     }
     ;

simple_type_name :
     special_type_name
     {
     $$:=$1;
     }
     |
     dname
     {
     $$:=$1;
     tn:=$$^.str;
     if removeunderscore and
        (length(tn)>1) and (tn[1]='_') then
      $$^.setstr(Copy(tn,2,length(tn)-1));
     }
     ;

declarator_list :
     declarator_list COMMA declarator
     {
     $$:=$1;
     hp:=$1;
     while assigned(hp^.next) do
       hp:=hp^.next;
     hp^.next:=new(presobject,init_one(t_declist,$3));
     }|
     error error_info COMMA declarator_list
     {
     writeln(outfile,' in declarator_list *)');
     $$:=$4;
     yyerrok;
     }|
     error error_info
     {
     writeln(outfile,' in declarator_list *)');
     yyerrok;
     }|
     declarator
     {
     $$:=new(presobject,init_one(t_declist,$1));
     }
     ;

argument_declaration : type_specifier declarator
     {
       $$:=new(presobject,init_two(t_arg,$1,$2));
     } |
     type_specifier STAR declarator
     {
       (* type_specifier STAR declarator *)
       hp:=new(presobject,init_one(t_pointerdef,$1));
       $$:=new(presobject,init_two(t_arg,hp,$3));
     } |
     type_specifier abstract_declarator
     {
       $$:=new(presobject,init_two(t_arg,$1,$2));
     }
     ;

argument_declaration_list : argument_declaration
     {
       $$:=new(presobject,init_two(t_arglist,$1,nil));
     } |
     argument_declaration COMMA argument_declaration_list
     {
       $$:=new(presobject,init_two(t_arglist,$1,nil));
       $$^.next:=$3;
     } |
     ELLIPSIS
     {
       $$:=new(presobject,init_two(t_arglist,ellipsisarg,nil));
     } |
     {
       $$:=nil;
     }
     ;

size_overrider :
       _FAR
       { $$:=new(presobject,init_id('far'));}
       | _NEAR
       { $$:=new(presobject,init_id('near'));}
       | _HUGE
       { $$:=new(presobject,init_id('huge'));}
       ;

declarator :
      _CONST declarator
      {
        if not stripinfo then
         writeln(outfile,'(* Const before declarator ignored *)');
        $$:=$2;
        } |
     size_overrider STAR declarator
     {
       if not stripinfo then
        writeln(outfile,aktspace,'(* ',$1^.p,' ignored *)');
       dispose($1,done);
       hp:=$3;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
     } |
     STAR declarator
     {
       (* %prec PSTAR this was wrong!! *)
       hp:=$2;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
     } |
     _AND declarator %prec P_AND
     {
       hp:=$2;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_addrdef,nil));
     } |
     dname COLON expr
       {
         (*  size specifier supported *)
         hp:=new(presobject,init_one(t_size_specifier,$3));
         $$:=new(presobject,init_three(t_dec,nil,$1,hp));
        }|
     dname ASSIGN expr
       {
         if not stripinfo then
          writeln(outfile,'(* Warning : default value for ',$1^.p,' ignored *)');
         hp:=new(presobject,init_one(t_default_value,$3));
         $$:=new(presobject,init_three(t_dec,nil,$1,hp));
        }|
     dname
       {
         $$:=new(presobject,init_two(t_dec,nil,$1));
        }|
     declarator LKLAMMER argument_declaration_list RKLAMMER
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_procdef,nil,$3));
     } |
     declarator no_arg
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_procdef,nil,nil));
     } |
     declarator LECKKLAMMER expr RECKKLAMMER
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_arraydef,nil,$3));
     } |
     declarator LECKKLAMMER RECKKLAMMER
     {
       (* this is translated into a pointer *)
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
<<<<<<< HEAD
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
=======
       hp^.p1:=new(presobject,init_two(t_arraydef,nil,nil));
>>>>>>> origin/fixes_2.4
     } |
     LKLAMMER declarator RKLAMMER
     {
       $$:=$2;
     }
     ;

no_arg : LKLAMMER RKLAMMER |
        LKLAMMER VOID RKLAMMER;

abstract_declarator :
      _CONST abstract_declarator
      {
        if not stripinfo then
         writeln(outfile,'(* Const before abstract_declarator ignored *)');
        $$:=$2;
        } |
     size_overrider STAR abstract_declarator
     {
       if not stripinfo then
        writeln(outfile,aktspace,'(* ',$1^.p,' ignored *)');
       dispose($1,done);
       hp:=$3;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
     } |
     STAR abstract_declarator %prec PSTAR
     {
       hp:=$2;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
     } |
     abstract_declarator LKLAMMER argument_declaration_list RKLAMMER
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_procdef,nil,$3));
     } |
     abstract_declarator no_arg
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_procdef,nil,nil));
     } |
     abstract_declarator LECKKLAMMER expr RECKKLAMMER
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_arraydef,nil,$3));
     } |
     declarator LECKKLAMMER RECKKLAMMER
     {
       (* this is translated into a pointer *)
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
<<<<<<< HEAD
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
=======
       hp^.p1:=new(presobject,init_two(t_arraydef,nil,nil));
>>>>>>> origin/fixes_2.4
     } |
     LKLAMMER abstract_declarator RKLAMMER
     {
       $$:=$2;
     } |
     {
       $$:=new(presobject,init_two(t_dec,nil,nil));
     }
     ;

expr    : shift_expr
          { $$:=$1; }
          ;

shift_expr :
          expr _ASSIGN expr
          { $$:=new(presobject,init_bop(':=',$1,$3)); }
          | expr EQUAL expr
          { $$:=new(presobject,init_bop('=',$1,$3));}
          | expr UNEQUAL expr
          { $$:=new(presobject,init_bop('<>',$1,$3));}
          | expr GT expr
          { $$:=new(presobject,init_bop('>',$1,$3));}
          | expr GTE expr
          { $$:=new(presobject,init_bop('>=',$1,$3));}
          | expr LT expr
          { $$:=new(presobject,init_bop('<',$1,$3));}
          | expr LTE expr
          { $$:=new(presobject,init_bop('<=',$1,$3));}
          | expr _PLUS expr
          { $$:=new(presobject,init_bop('+',$1,$3));}
          | expr MINUS expr
          { $$:=new(presobject,init_bop('-',$1,$3));}
               | expr STAR expr
          { $$:=new(presobject,init_bop('*',$1,$3));}
               | expr _SLASH expr
          { $$:=new(presobject,init_bop('/',$1,$3));}
               | expr _OR expr
          { $$:=new(presobject,init_bop(' or ',$1,$3));}
               | expr _AND expr
          { $$:=new(presobject,init_bop(' and ',$1,$3));}
               | expr _NOT expr
          { $$:=new(presobject,init_bop(' not ',$1,$3));}
               | expr _SHL expr
          { $$:=new(presobject,init_bop(' shl ',$1,$3));}
               | expr _SHR expr
          { $$:=new(presobject,init_bop(' shr ',$1,$3));}
          | expr QUESTIONMARK colon_expr
          {
            $3^.p1:=$1;
            $$:=$3;
            inc(if_nb);
            $$^.p:=strpnew('if_local'+str(if_nb));
          } |
          unary_expr {$$:=$1;}
          ;

colon_expr : expr COLON expr
       { (* if A then B else C *)
       $$:=new(presobject,init_three(t_ifexpr,nil,$1,$3));}
       ;

maybe_empty_unary_expr :
                  unary_expr
                  { $$:=$1; }
                  |
                  { $$:=nil;}
                  ;

unary_expr:
     dname
     {
     $$:=$1;
     } |
     special_type_name
     {
     $$:=$1;
     } |
     CSTRING
     {
     (* remove L prefix for widestrings *)
     s:=act_token;
     if Win32headers and (s[1]='L') then
       delete(s,1,1);
     $$:=new(presobject,init_id(''''+copy(s,2,length(s)-2)+''''));
     } |
     NUMBER
     {
     $$:=new(presobject,init_id(act_token));
     } |
     unary_expr POINT expr
     {
     $$:=new(presobject,init_bop('.',$1,$3));
     } |
     unary_expr DEREF expr
     {
     $$:=new(presobject,init_bop('^.',$1,$3));
     } |
     MINUS unary_expr
     {
     $$:=new(presobject,init_preop('-',$2));
     }|
     _PLUS unary_expr
     {
     $$:=new(presobject,init_preop('+',$2));
     }|
     _AND unary_expr %prec R_AND
     {
     $$:=new(presobject,init_preop('@',$2));
     }|
     _NOT unary_expr
     {
     $$:=new(presobject,init_preop(' not ',$2));
     } |
     LKLAMMER dname RKLAMMER maybe_empty_unary_expr
     {
     if assigned($4) then
       $$:=new(presobject,init_two(t_typespec,$2,$4))
     else
       $$:=$2;
     } |
     LKLAMMER type_specifier RKLAMMER unary_expr
     {
     $$:=new(presobject,init_two(t_typespec,$2,$4));
     } |
     LKLAMMER type_specifier STAR RKLAMMER unary_expr
     {
     hp:=new(presobject,init_one(t_pointerdef,$2));
     $$:=new(presobject,init_two(t_typespec,hp,$5));
     } |
     LKLAMMER type_specifier size_overrider STAR RKLAMMER unary_expr
     {
     if not stripinfo then
      writeln(outfile,aktspace,'(* ',$3^.p,' ignored *)');
     dispose($3,done);
     write_type_specifier(outfile,$2);
     writeln(outfile,' ignored *)');
     hp:=new(presobject,init_one(t_pointerdef,$2));
     $$:=new(presobject,init_two(t_typespec,hp,$6));
     } |
     dname LKLAMMER exprlist RKLAMMER
     {
     hp:=new(presobject,init_one(t_exprlist,$1));
     $$:=new(presobject,init_three(t_funexprlist,hp,$3,nil));
     } |
     LKLAMMER shift_expr RKLAMMER
     {
     $$:=$2;
     } |
     LKLAMMER STAR unary_expr RKLAMMER maybe_space LKLAMMER exprlist RKLAMMER
     {
       $$:=new(presobject,init_two(t_callop,$3,$7));
     } |
     dname LECKKLAMMER exprlist RECKKLAMMER
     {
       $$:=new(presobject,init_two(t_arrayop,$1,$3));
     }
     ;

enum_list :
     enum_element COMMA enum_list
     { (*enum_element COMMA enum_list *)
       $$:=$1;
       $$^.next:=$3;
      } |
      enum_element {
       $$:=$1;
      } |
      {(* empty enum list *)
       $$:=nil;};

enum_element :
     dname _ASSIGN expr
     { begin (*enum_element: dname _ASSIGN expr *)
        $$:=new(presobject,init_two(t_enumlist,$1,$3));
       end;
     } |
     dname
     {
       begin (*enum_element: dname*)
       $$:=new(presobject,init_two(t_enumlist,$1,nil));
       end;
     };


def_expr :
     unary_expr
     {
         if $1^.typ=t_funexprlist then
           $$:=$1
         else
           $$:=new(presobject,init_two(t_exprlist,$1,nil));
         (* if here is a type specifier
            we know the return type *)
         if ($1^.typ=t_typespec) then
           $$^.p3:=$1^.p1^.get_copy;
     }
     ;

para_def_expr :
     SPACE_DEFINE def_expr
     {
     $$:=$2;
     } |
     maybe_space LKLAMMER def_expr RKLAMMER
     {
     $$:=$3
     }
     ;

exprlist : exprelem COMMA exprlist
    { (*exprlist COMMA expr*)
       $$:=$1;
       $1^.next:=$3;
     } |
     exprelem
     {
       $$:=$1;
     } |
     { (* empty expression list *)
       $$:=nil; };

exprelem :
           expr
           {
             $$:=new(presobject,init_one(t_exprlist,$1));
           };

%%

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
<<<<<<< HEAD
=======
%{
program h2pas;

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
     options,scan,converu,lexlib,yacclib;

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
         shift(3);
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
                        shift(3);
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
                        shift(3);
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
                  if not compactmode then
                   shift(2);
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
                  shift(3);
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
                   (* Needs 2 declarations, also one without args, becuase
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

      begin
         if not(assigned(p)) then
           begin
              write_type_specifier(outfile,simple_type);
              exit;
           end;
         case p^.typ of
            t_pointerdef : begin
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
                                        write_p_a_def(outfile,p^.p1^.p1,simple_type);
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
            t_arraydef : begin
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
                      if UseCTypesUnit and (IsACType(p^.p1^.p)=False) then
                        write(outfile,'P')
                      else
                        write(outfile,'p');
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
                      shift(3);
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
                                if not assigned(hp3^.p1^.p3) or
                                   (hp3^.p1^.p3^.typ <> t_size_specifier) then
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
                                if assigned(hp3^.p1^.p3) then
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
                                            begin
                                               write(outfile,';cdecl');
                                               no_pop:=true;
                                            end;
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
                      shift(3);
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

%}

%token _WHILE _FOR _DO _GOTO _CONTINUE _BREAK
%token TYPEDEF DEFINE
%token COLON SEMICOLON COMMA
%token LKLAMMER RKLAMMER LECKKLAMMER RECKKLAMMER
%token LGKLAMMER RGKLAMMER
%token STRUCT UNION ENUM
%token ID NUMBER CSTRING
%token SHORT UNSIGNED LONG INT FLOAT _CHAR
%token VOID _CONST
%token _FAR _HUGE _NEAR
%token NEW_LINE SPACE_DEFINE
%token EXTERN STDCALL CDECL CALLBACK PASCAL WINAPI APIENTRY WINGDIAPI SYS_TRAP
%token _PACKED
%token ELLIPSIS
%right _ASSIGN
%right R_AND
%left EQUAL UNEQUAL GT LT GTE LTE
%left QUESTIONMARK COLON
%left _OR
%left _AND
%left _PLUS MINUS
%left _SHR _SHL
%left STAR _SLASH
%right _NOT
%right LKLAMMER
%right PSTAR
%right P_AND
%right LECKKLAMMER
%left POINT DEREF
%left COMMA
%left STICK
%token SIGNED
%token INT8 INT16 INT32 INT64
%%

file : declaration_list
     ;

maybe_space :
     SPACE_DEFINE
     {
       $$:=nil;
     } |
     {
       $$:=nil;
     }
     ;

error_info : {
                  writeln(outfile,'(* error ');
                  writeln(outfile,yyline);
             };

declaration_list : declaration_list  declaration
     {  if yydebug then writeln('declaration reduced at line ',line_no);
        if yydebug then writeln(outfile,'(* declaration reduced *)');
     }
     | declaration_list define_dec
     {  if yydebug then writeln('define declaration reduced at line ',line_no);
        if yydebug then writeln(outfile,'(* define declaration reduced *)');
     }
     | declaration
     {  if yydebug then writeln('declaration reduced at line ',line_no);
     }
     | define_dec
     {  if yydebug then writeln('define declaration reduced at line ',line_no);
     }
     ;

dec_specifier :
     EXTERN { $$:=new(presobject,init_id('extern')); }
     |{ $$:=new(presobject,init_id('intern')); }
     ;

dec_modifier :
     STDCALL { $$:=new(presobject,init_id('no_pop')); }
     | CDECL { $$:=new(presobject,init_id('cdecl')); }
     | CALLBACK { $$:=new(presobject,init_id('no_pop')); }
     | PASCAL { $$:=new(presobject,init_id('no_pop')); }
     | WINAPI { $$:=new(presobject,init_id('no_pop')); }
     | APIENTRY { $$:=new(presobject,init_id('no_pop')); }
     | WINGDIAPI { $$:=new(presobject,init_id('no_pop')); }
     | { $$:=nil }
     ;

systrap_specifier:
     SYS_TRAP LKLAMMER dname RKLAMMER { $$:=$3; }
     | { $$:=nil; }
     ;

statement :
     expr SEMICOLON { $$:=$1; } |
     _WHILE LKLAMMER expr RKLAMMER statement_list { $$:=new(presobject,init_two(t_whilenode,$3,$5)); }
     ;


statement_list : statement statement_list
     {
       $$:=new(presobject,init_one(t_statement_list,$1));
       $$^.next:=$2;
     } |
     statement
     {
       $$:=new(presobject,init_one(t_statement_list,$1));
     } |
     SEMICOLON
     {
       $$:=new(presobject,init_one(t_statement_list,nil));
     } |
     {
       $$:=new(presobject,init_one(t_statement_list,nil));
     }
     ;

statement_block :
     LGKLAMMER statement_list RGKLAMMER { $$:=$2; }
     ;

declaration :
     dec_specifier type_specifier dec_modifier declarator_list statement_block
     {
       IsExtern:=false;
       (* by default we must pop the args pushed on stack *)
       no_pop:=false;
       if (assigned($4)and assigned($4^.p1)and assigned($4^.p1^.p1))
         and ($4^.p1^.p1^.typ=t_procdef) then
          begin
             repeat
             If UseLib then
               IsExtern:=true
             else
               IsExtern:=assigned($1)and($1^.str='extern');
             no_pop:=assigned($3) and ($3^.str='no_pop');

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
             if assigned($2) then
              if ($2^.typ=t_void) and ($4^.p1^.p1^.p1=nil) then
               begin
                 if createdynlib then
                   begin
                     write(outfile,$4^.p1^.p2^.p,' : procedure');
                   end
                 else
                   begin
                     shift(10);
                     write(outfile,'procedure ',$4^.p1^.p2^.p);
                   end;
                 if assigned($4^.p1^.p1^.p2) then
                   write_args(outfile,$4^.p1^.p1^.p2);
                 if createdynlib then
                    begin
                      loaddynlibproc.add('pointer('+$4^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+$4^.p1^.p2^.p+''');');
                      freedynlibproc.add($4^.p1^.p2^.p+':=nil;');
                    end
                  else if not IsExtern then
                  begin
                    write(implemfile,'procedure ',$4^.p1^.p2^.p);
                    if assigned($4^.p1^.p1^.p2) then
                     write_args(implemfile,$4^.p1^.p1^.p2);
                  end;
               end
             else
               begin
                 if createdynlib then
                   begin
                     write(outfile,$4^.p1^.p2^.p,' : function');
                   end
                 else
                   begin
                     shift(9);
                     write(outfile,'function ',$4^.p1^.p2^.p);
                   end;

                  if assigned($4^.p1^.p1^.p2) then
                    write_args(outfile,$4^.p1^.p1^.p2);
                  write(outfile,':');
                  write_p_a_def(outfile,$4^.p1^.p1^.p1,$2);
                  if createdynlib then
                    begin
                      loaddynlibproc.add('pointer('+$4^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+$4^.p1^.p2^.p+''');');
                      freedynlibproc.add($4^.p1^.p2^.p+':=nil;');
                    end
                  else if not IsExtern then
                   begin
                     write(implemfile,'function ',$4^.p1^.p2^.p);
                     if assigned($4^.p1^.p1^.p2) then
                      write_args(implemfile,$4^.p1^.p1^.p2);
                     write(implemfile,':');
                     write_p_a_def(implemfile,$4^.p1^.p1^.p1,$2);
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
                     Write(outfile,' External_library name ''',$4^.p1^.p2^.p,'''');
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
                    if $5^.typ=t_statement_list then
                      write_statement_block(implemfile,$5);
                    popshift;
                  end;
               end;
             IsExtern:=false;
             if not(compactmode) and not(createdynlib) then
              writeln(outfile);
            until not NeedEllipsisOverload;
          end
        else (* $4^.p1^.p1^.typ=t_procdef *)
        if assigned($4)and assigned($4^.p1) then
          begin
             shift(2);
             if block_type<>bt_var then
               begin
                  if not(compactmode) then
                    writeln(outfile);
                  writeln(outfile,aktspace,'var');
               end;
             block_type:=bt_var;

             shift(3);

             IsExtern:=assigned($1)and($1^.str='extern');
             (* walk through all declarations *)
             hp:=$4;
             while assigned(hp) and assigned(hp^.p1) do
               begin
                  (* write new var name *)
                  if assigned(hp^.p1^.p2) and assigned(hp^.p1^.p2^.p) then
                    write(outfile,aktspace,hp^.p1^.p2^.p);
                  write(outfile,' : ');
                  shift(2);
                  (* write its type *)
                  write_p_a_def(outfile,hp^.p1^.p1,$2);
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
        if assigned($1) then
          dispose($1,done);
        if assigned($2) then
          dispose($2,done);
        if assigned($3) then
          dispose($3,done);
        if assigned($4) then
          dispose($4,done);
        if assigned($5) then
          dispose($5,done);
     }
     | dec_specifier type_specifier dec_modifier declarator_list systrap_specifier SEMICOLON
     {
       IsExtern:=false;
       (* by default we must pop the args pushed on stack *)
       no_pop:=false;
       if (assigned($4)and assigned($4^.p1)and assigned($4^.p1^.p1))
         and ($4^.p1^.p1^.typ=t_procdef) then
          begin
             repeat
             If UseLib then
               IsExtern:=true
             else
               IsExtern:=assigned($1)and($1^.str='extern');
             no_pop:=assigned($3) and ($3^.str='no_pop');

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
             if assigned($2) then
              if ($2^.typ=t_void) and ($4^.p1^.p1^.p1=nil) then
               begin
                 if createdynlib then
                   begin
                     write(outfile,$4^.p1^.p2^.p,' : procedure');
                   end
                 else
                   begin
                     shift(10);
                     write(outfile,'procedure ',$4^.p1^.p2^.p);
                   end;
                 if assigned($4^.p1^.p1^.p2) then
                   write_args(outfile,$4^.p1^.p1^.p2);
                 if createdynlib then
                    begin
                      loaddynlibproc.add('pointer('+$4^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+$4^.p1^.p2^.p+''');');
                      freedynlibproc.add($4^.p1^.p2^.p+':=nil;');
                    end
                  else if not IsExtern then
                  begin
                    write(implemfile,'procedure ',$4^.p1^.p2^.p);
                    if assigned($4^.p1^.p1^.p2) then
                     write_args(implemfile,$4^.p1^.p1^.p2);
                  end;
               end
             else
               begin
                 if createdynlib then
                   begin
                     write(outfile,$4^.p1^.p2^.p,' : function');
                   end
                 else
                   begin
                     shift(9);
                     write(outfile,'function ',$4^.p1^.p2^.p);
                   end;

                  if assigned($4^.p1^.p1^.p2) then
                    write_args(outfile,$4^.p1^.p1^.p2);
                  write(outfile,':');
                  write_p_a_def(outfile,$4^.p1^.p1^.p1,$2);
                  if createdynlib then
                    begin
                      loaddynlibproc.add('pointer('+$4^.p1^.p2^.p+'):=GetProcAddress(hlib,'''+$4^.p1^.p2^.p+''');');
                      freedynlibproc.add($4^.p1^.p2^.p+':=nil;');
                    end
                  else if not IsExtern then
                   begin
                     write(implemfile,'function ',$4^.p1^.p2^.p);
                     if assigned($4^.p1^.p1^.p2) then
                      write_args(implemfile,$4^.p1^.p1^.p2);
                     write(implemfile,':');
                     write_p_a_def(implemfile,$4^.p1^.p1^.p1,$2);
                   end;
               end;
             if assigned($5) then
               write(outfile,';systrap ',$5^.p);
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
                     Write(outfile,' External_library name ''',$4^.p1^.p2^.p,'''');
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
        else (* $4^.p1^.p1^.typ=t_procdef *)
        if assigned($4)and assigned($4^.p1) then
          begin
             shift(2);
             if block_type<>bt_var then
               begin
                  if not(compactmode) then
                    writeln(outfile);
                  writeln(outfile,aktspace,'var');
               end;
             block_type:=bt_var;

             shift(3);

             IsExtern:=assigned($1)and($1^.str='extern');
             (* walk through all declarations *)
             hp:=$4;
             while assigned(hp) and assigned(hp^.p1) do
               begin
                  (* write new var name *)
                  if assigned(hp^.p1^.p2) and assigned(hp^.p1^.p2^.p) then
                    write(outfile,aktspace,hp^.p1^.p2^.p);
                  write(outfile,' : ');
                  shift(2);
                  (* write its type *)
                  write_p_a_def(outfile,hp^.p1^.p1,$2);
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
        if assigned($1)then  dispose($1,done);
        if assigned($2)then  dispose($2,done);
        if assigned($4)then  dispose($4,done);
     } |
     special_type_specifier SEMICOLON
     {
       if block_type<>bt_type then
         begin
            if not(compactmode) then
              writeln(outfile);
            writeln(outfile,aktspace,'type');
            block_type:=bt_type;
         end;
       shift(3);
       if ( yyv[yysp-1]^.p2  <> nil ) then
         begin
         (* write new type name *)
         TN:=TypeName($1^.p2^.p);
         PN:=PointerName($1^.p2^.p);
         (* define a Pointer type also for structs *)
         if UsePPointers and (Uppercase(tn)<>Uppercase(pn)) and
            assigned($1) and ($1^.typ in [t_uniondef,t_structdef]) then
          writeln(outfile,aktspace,PN,' = ^',TN,';');
         write(outfile,aktspace,TN,' = ');
         shift(2);
         hp:=$1;
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
     } |
     TYPEDEF STRUCT dname dname SEMICOLON
     {
       (* TYPEDEF STRUCT dname dname SEMICOLON *)
       if block_type<>bt_type then
         begin
            if not(compactmode) then
              writeln(outfile);
            writeln(outfile,aktspace,'type');
            block_type:=bt_type;
         end;
       PN:=TypeName($3^.p);
       TN:=TypeName($4^.p);
       if Uppercase(tn)<>Uppercase(pn) then
        begin
          shift(3);
          writeln(outfile,aktspace,PN,' = ',TN,';');
          popshift;
        end;
       if assigned($3) then
        dispose($3,done);
       if assigned($4) then
        dispose($4,done);
     } |
     TYPEDEF type_specifier LKLAMMER dec_modifier declarator RKLAMMER maybe_space LKLAMMER argument_declaration_list RKLAMMER SEMICOLON
     {
       (* TYPEDEF type_specifier LKLAMMER dec_modifier declarator RKLAMMER maybe_space LKLAMMER argument_declaration_list RKLAMMER SEMICOLON *)
       if block_type<>bt_type then
         begin
            if not(compactmode) then
              writeln(outfile);
            writeln(outfile,aktspace,'type');
            block_type:=bt_type;
         end;
       no_pop:=assigned($4) and ($4^.str='no_pop');
       shift(3);
       (* walk through all declarations *)
       hp:=$5;
       if assigned(hp) then
        begin
          hp:=$5;
          while assigned(hp^.p1) do
           hp:=hp^.p1;
          hp^.p1:=new(presobject,init_two(t_procdef,nil,$9));
          hp:=$5;
          if assigned(hp^.p1) and assigned(hp^.p1^.p1) then
           begin
             writeln(outfile);
             (* write new type name *)
             write(outfile,aktspace,TypeName(hp^.p2^.p),' = ');
             shift(2);
             write_p_a_def(outfile,hp^.p1,$2);
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
       if assigned($2)then
       dispose($2,done);
       if assigned($4)then
       dispose($4,done);
       if assigned($5)then (* disposes also $9 *)
       dispose($5,done);
     } |
     TYPEDEF type_specifier dec_modifier declarator_list SEMICOLON
     {
       (* TYPEDEF type_specifier dec_modifier declarator_list SEMICOLON *)
       if block_type<>bt_type then
         begin
            if not(compactmode) then
              writeln(outfile);
            writeln(outfile,aktspace,'type');
            block_type:=bt_type;
         end;
       no_pop:=assigned($3) and ($3^.str='no_pop');
       shift(3);
       (* Get the name to write the type definition for, try
          to use the tag name first *)
       if assigned($2^.p2) then
        begin
          ph:=$2^.p2;
        end
       else
        begin
          if not assigned($4^.p1^.p2) then
           internalerror(4444);
          ph:=$4^.p1^.p2;
        end;
       (* write type definition *)
       is_procvar:=false;
       writeln(outfile);
       TN:=TypeName(ph^.p);
       PN:=PointerName(ph^.p);
       if UsePPointers and (Uppercase(tn)<>Uppercase(pn)) and
          assigned($2) and ($2^.typ<>t_procdef) then
         writeln(outfile,aktspace,PN,' = ^',TN,';');
       (* write new type name *)
       write(outfile,aktspace,TN,' = ');
       shift(2);
       write_type_specifier(outfile,$2);
       popshift;
       (* if no_pop it is normal fpc calling convention *)
       if is_procvar and
          (not no_pop) then
         write(outfile,';cdecl');
       writeln(outfile,';');
       flush(outfile);
       (* write alias names, ph points to the name already used *)
       hp:=$4;
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
                  assigned($2) and ($2^.typ<>t_procdef) then
                 writeln(outfile,aktspace,PN,' = ^',TN,';');
              end;
           end;
          hp:=hp^.next;
        end;
       popshift;
       if must_write_packed_field then
         if assigned(ph) then
           write_packed_fields_info(outfile,$2,ph^.str)
         else if assigned($2^.p2) then
           write_packed_fields_info(outfile,$2,$2^.p2^.str);
       if assigned($2)then
       dispose($2,done);
       if assigned($3)then
       dispose($3,done);
       if assigned($4)then
       dispose($4,done);
     } |
     TYPEDEF dname SEMICOLON
     {
       if block_type<>bt_type then
         begin
            if not(compactmode) then
              writeln(outfile);
            writeln(outfile,aktspace,'type');
            block_type:=bt_type;
         end;
       shift(3);
       (* write as pointer *)
       writeln(outfile);
       writeln(outfile,'(* generic typedef  *)');
       writeln(outfile,aktspace,$2^.p,' = pointer;');
       flush(outfile);
       popshift;
       if assigned($2) then
        dispose($2,done);
     }
     | error  error_info SEMICOLON
      { writeln(outfile,'in declaration at line ',line_no,' *)');
        aktspace:='';
        in_space_define:=0;
        in_define:=false;
        arglevel:=0;
        if_nb:=0;
        aktspace:='    ';
        space_index:=1;
        yyerrok;}
     ;

define_dec :
     DEFINE dname LKLAMMER enum_list RKLAMMER para_def_expr NEW_LINE
     {
       (* DEFINE dname LKLAMMER enum_list RKLAMMER para_def_expr NEW_LINE *)
       if not stripinfo then
        begin
          writeln (outfile,aktspace,'{ was #define dname(params) para_def_expr }');
          writeln (implemfile,aktspace,'{ was #define dname(params) para_def_expr }');
          if assigned($4) then
           begin
             writeln (outfile,aktspace,'{ argument types are unknown }');
             writeln (implemfile,aktspace,'{ argument types are unknown }');
           end;
          if not assigned($6^.p3) then
           begin
             writeln(outfile,aktspace,'{ return type might be wrong }   ');
             writeln(implemfile,aktspace,'{ return type might be wrong }   ');
           end;
        end;
       block_type:=bt_func;
       write(outfile,aktspace,'function ',$2^.p);
       write(implemfile,aktspace,'function ',$2^.p);

       if assigned($4) then
         begin
            write(outfile,'(');
            write(implemfile,'(');
            ph:=new(presobject,init_one(t_enumdef,$4));
            write_def_params(outfile,ph);
            write_def_params(implemfile,ph);
            if assigned(ph) then dispose(ph,done);
            ph:=nil;
            (* types are unknown *)
            write(outfile,' : longint)');
            write(implemfile,' : longint)');
         end;
       if not assigned($6^.p3) then
         begin
            writeln(outfile,' : longint;',aktspace,commentstr);
            writeln(implemfile,' : longint;');
            flush(outfile);
         end
       else
         begin
            write(outfile,' : ');
            write_type_specifier(outfile,$6^.p3);
            writeln(outfile,';',aktspace,commentstr);
            flush(outfile);
            write(implemfile,' : ');
            write_type_specifier(implemfile,$6^.p3);
            writeln(implemfile,';');
         end;
       writeln(outfile);
       flush(outfile);
       hp:=new(presobject,init_two(t_funcname,$2,$6));
       write_funexpr(implemfile,hp);
       writeln(implemfile);
       flush(implemfile);
       if assigned(hp)then dispose(hp,done);
     }|
     DEFINE dname SPACE_DEFINE NEW_LINE
     {
       (* DEFINE dname SPACE_DEFINE NEW_LINE *)
       writeln(outfile,'{$define ',$2^.p,'}',aktspace,commentstr);
       flush(outfile);
       if assigned($2)then
        dispose($2,done);
     }|
     DEFINE dname NEW_LINE
     {
       writeln(outfile,'{$define ',$2^.p,'}',aktspace,commentstr);
       flush(outfile);
       if assigned($2)then
        dispose($2,done);
     } |
     DEFINE dname SPACE_DEFINE def_expr NEW_LINE
     {
       (* DEFINE dname SPACE_DEFINE def_expr NEW_LINE *)
       if ($4^.typ=t_exprlist) and
          $4^.p1^.is_const and
          not assigned($4^.next) then
         begin
            if block_type<>bt_const then
              begin
                 writeln(outfile);
                 writeln(outfile,aktspace,'const');
              end;
            block_type:=bt_const;
            shift(3);
            write(outfile,aktspace,$2^.p);
            write(outfile,' = ');
            flush(outfile);
            write_expr(outfile,$4^.p1);
            writeln(outfile,';',aktspace,commentstr);
            popshift;
            if assigned($2) then
            dispose($2,done);
            if assigned($4) then
            dispose($4,done);
         end
       else
         begin
            if not stripinfo then
             begin
               writeln (outfile,aktspace,'{ was #define dname def_expr }');
               writeln (implemfile,aktspace,'{ was #define dname def_expr }');
             end;
            block_type:=bt_func;
            write(outfile,aktspace,'function ',$2^.p);
            write(implemfile,aktspace,'function ',$2^.p);
            shift(2);
            if not assigned($4^.p3) then
              begin
                 writeln(outfile,' : longint;');
                 writeln(outfile,aktspace,'  { return type might be wrong }');
                 flush(outfile);
                 writeln(implemfile,' : longint;');
                 writeln(implemfile,aktspace,'  { return type might be wrong }');
              end
            else
              begin
                 write(outfile,' : ');
                 write_type_specifier(outfile,$4^.p3);
                 writeln(outfile,';',aktspace,commentstr);
                 flush(outfile);
                 write(implemfile,' : ');
                 write_type_specifier(implemfile,$4^.p3);
                 writeln(implemfile,';');
              end;
            writeln(outfile);
            flush(outfile);
            hp:=new(presobject,init_two(t_funcname,$2,$4));
            write_funexpr(implemfile,hp);
            popshift;
            dispose(hp,done);
            writeln(implemfile);
            flush(implemfile);
         end;
     }
     | error error_info NEW_LINE
      { writeln(outfile,'in define line ',line_no,' *)');
        aktspace:='';
        in_space_define:=0;
        in_define:=false;
        arglevel:=0;
        if_nb:=0;
        aktspace:='    ';
        space_index:=1;

        yyerrok;}
     ;

closed_list : LGKLAMMER member_list RGKLAMMER
            {$$:=$2;} |
            error  error_info RGKLAMMER
            { writeln(outfile,' in member_list *)');
            yyerrok;
            $$:=nil;
            }
            ;

closed_enum_list : LGKLAMMER enum_list RGKLAMMER
            {$$:=$2;} |
            error  error_info  RGKLAMMER
            { writeln(outfile,' in enum_list *)');
            yyerrok;
            $$:=nil;
            }
            ;

special_type_specifier :
     STRUCT dname closed_list _PACKED
     {
       if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
       is_packed:=true;
       $$:=new(presobject,init_two(t_structdef,$3,$2));
     } |
     STRUCT dname closed_list
     {
       if (is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 4}');
       is_packed:=false;
       $$:=new(presobject,init_two(t_structdef,$3,$2));
     } |
     UNION dname closed_list _PACKED
     {
       if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
       is_packed:=true;
       $$:=new(presobject,init_two(t_uniondef,$3,$2));
     } |
     UNION dname closed_list
     {
       $$:=new(presobject,init_two(t_uniondef,$3,$2));
     } |
     UNION dname
     {
       $$:=$2;
     } |
     STRUCT dname
     {
       $$:=$2;
     } |
     ENUM dname closed_enum_list
     {
       $$:=new(presobject,init_two(t_enumdef,$3,$2));
     } |
     ENUM dname
     {
       $$:=$2;
     };

type_specifier :
      _CONST type_specifier
      {
        if not stripinfo then
         writeln(outfile,'(* Const before type ignored *)');
        $$:=$2;
        } |
     UNION closed_list  _PACKED
     {
       if (not is_packed) and (not packrecords)then
         writeln(outfile,'{$PACKRECORDS 1}');
       is_packed:=true;
       $$:=new(presobject,init_one(t_uniondef,$2));
     } |
     UNION closed_list
     {
       $$:=new(presobject,init_one(t_uniondef,$2));
     } |
     STRUCT closed_list _PACKED
     {
       if (not is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 1}');
       is_packed:=true;
       $$:=new(presobject,init_one(t_structdef,$2));
     } |
     STRUCT closed_list
     {
       if (is_packed) and (not packrecords) then
         writeln(outfile,'{$PACKRECORDS 4}');
       is_packed:=false;
       $$:=new(presobject,init_one(t_structdef,$2));
     } |
     ENUM closed_enum_list
     {
       $$:=new(presobject,init_one(t_enumdef,$2));
     } |
     special_type_specifier
     {
       $$:=$1;
     } |
     simple_type_name { $$:=$1; }
     ;

member_list : member_declaration member_list
     {
       $$:=new(presobject,init_one(t_memberdeclist,$1));
       $$^.next:=$2;
     } |
     member_declaration
     {
       $$:=new(presobject,init_one(t_memberdeclist,$1));
     }
     ;

member_declaration :
     type_specifier declarator_list SEMICOLON
     {
       $$:=new(presobject,init_two(t_memberdec,$1,$2));
     }
     ;

dname : ID { (*dname*)
           $$:=new(presobject,init_id(act_token));
           }
     ;
special_type_name :
     SIGNED special_type_name
     {
       hp:=$2;
       $$:=hp;
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
     } |
     UNSIGNED special_type_name
     {
       hp:=$2;
       $$:=hp;
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
     } |
     INT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint_STR))
     else
       $$:=new(presobject,init_intid(INT_STR));
     } |
     LONG
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(clong_STR))
     else
       $$:=new(presobject,init_intid(INT_STR));
     } |
     LONG INT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(clong_STR))
     else
       $$:=new(presobject,init_intid(INT_STR));
     } |
     LONG LONG
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(clonglong_STR))
     else
       $$:=new(presobject,init_intid(INT64_STR));
     } |
     LONG LONG INT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(clonglong_STR))
     else
       $$:=new(presobject,init_intid(INT64_STR));
     } |
     SHORT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cshort_STR))
     else
       $$:=new(presobject,init_intid(SMALL_STR));
     } |
     SHORT INT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(csint_STR))
     else
       $$:=new(presobject,init_intid(SMALL_STR));
     } |
     INT8
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint8_STR))
     else
       $$:=new(presobject,init_intid(SHORT_STR));
     } |
     INT16
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint16_STR))
     else
       $$:=new(presobject,init_intid(SMALL_STR));
     } |
     INT32
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint32_STR))
     else
       $$:=new(presobject,init_intid(INT_STR));
     } |
     INT64
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cint64_STR))
     else
       $$:=new(presobject,init_intid(INT64_STR));
     } |
     FLOAT
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cfloat_STR))
     else
       $$:=new(presobject,init_intid(FLOAT_STR));
     } |
     VOID
     {
       $$:=new(presobject,init_no(t_void));
     } |
     _CHAR
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cchar_STR))
     else
       $$:=new(presobject,init_intid(CHAR_STR));
     } |
     UNSIGNED
     {
     if UseCTypesUnit then
       $$:=new(presobject,init_id(cunsigned_STR))
     else
       $$:=new(presobject,init_intid(UINT_STR));
     }
     ;

simple_type_name :
     special_type_name
     {
     $$:=$1;
     }
     |
     dname
     {
     $$:=$1;
     tn:=$$^.str;
     if removeunderscore and
        (length(tn)>1) and (tn[1]='_') then
      $$^.setstr(Copy(tn,2,length(tn)-1));
     }
     ;

declarator_list :
     declarator_list COMMA declarator
     {
     $$:=$1;
     hp:=$1;
     while assigned(hp^.next) do
       hp:=hp^.next;
     hp^.next:=new(presobject,init_one(t_declist,$3));
     }|
     error error_info COMMA declarator_list
     {
     writeln(outfile,' in declarator_list *)');
     $$:=$4;
     yyerrok;
     }|
     error error_info
     {
     writeln(outfile,' in declarator_list *)');
     yyerrok;
     }|
     declarator
     {
     $$:=new(presobject,init_one(t_declist,$1));
     }
     ;

argument_declaration : type_specifier declarator
     {
       $$:=new(presobject,init_two(t_arg,$1,$2));
     } |
     type_specifier STAR declarator
     {
       (* type_specifier STAR declarator *)
       hp:=new(presobject,init_one(t_pointerdef,$1));
       $$:=new(presobject,init_two(t_arg,hp,$3));
     } |
     type_specifier abstract_declarator
     {
       $$:=new(presobject,init_two(t_arg,$1,$2));
     }
     ;

argument_declaration_list : argument_declaration
     {
       $$:=new(presobject,init_two(t_arglist,$1,nil));
     } |
     argument_declaration COMMA argument_declaration_list
     {
       $$:=new(presobject,init_two(t_arglist,$1,nil));
       $$^.next:=$3;
     } |
     ELLIPSIS
     {
       $$:=new(presobject,init_two(t_arglist,ellipsisarg,nil));
     } |
     {
       $$:=nil;
     }
     ;

size_overrider :
       _FAR
       { $$:=new(presobject,init_id('far'));}
       | _NEAR
       { $$:=new(presobject,init_id('near'));}
       | _HUGE
       { $$:=new(presobject,init_id('huge'));}
       ;

declarator :
      _CONST declarator
      {
        if not stripinfo then
         writeln(outfile,'(* Const before declarator ignored *)');
        $$:=$2;
        } |
     size_overrider STAR declarator
     {
       if not stripinfo then
        writeln(outfile,aktspace,'(* ',$1^.p,' ignored *)');
       dispose($1,done);
       hp:=$3;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
     } |
     STAR declarator
     {
       (* %prec PSTAR this was wrong!! *)
       hp:=$2;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
     } |
     _AND declarator %prec P_AND
     {
       hp:=$2;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_addrdef,nil));
     } |
     dname COLON expr
       {
         (*  size specifier supported *)
         hp:=new(presobject,init_one(t_size_specifier,$3));
         $$:=new(presobject,init_three(t_dec,nil,$1,hp));
        }|
     dname ASSIGN expr
       {
         if not stripinfo then
          writeln(outfile,'(* Warning : default value for ',$1^.p,' ignored *)');
         hp:=new(presobject,init_one(t_default_value,$3));
         $$:=new(presobject,init_three(t_dec,nil,$1,hp));
        }|
     dname
       {
         $$:=new(presobject,init_two(t_dec,nil,$1));
        }|
     declarator LKLAMMER argument_declaration_list RKLAMMER
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_procdef,nil,$3));
     } |
     declarator no_arg
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_procdef,nil,nil));
     } |
     declarator LECKKLAMMER expr RECKKLAMMER
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_arraydef,nil,$3));
     } |
     declarator LECKKLAMMER RECKKLAMMER
     {
       (* this is translated into a pointer *)
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_arraydef,nil,nil));
     } |
     LKLAMMER declarator RKLAMMER
     {
       $$:=$2;
     }
     ;

no_arg : LKLAMMER RKLAMMER |
        LKLAMMER VOID RKLAMMER;

abstract_declarator :
      _CONST abstract_declarator
      {
        if not stripinfo then
         writeln(outfile,'(* Const before abstract_declarator ignored *)');
        $$:=$2;
        } |
     size_overrider STAR abstract_declarator
     {
       if not stripinfo then
        writeln(outfile,aktspace,'(* ',$1^.p,' ignored *)');
       dispose($1,done);
       hp:=$3;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
     } |
     STAR abstract_declarator %prec PSTAR
     {
       hp:=$2;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_one(t_pointerdef,nil));
     } |
     abstract_declarator LKLAMMER argument_declaration_list RKLAMMER
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_procdef,nil,$3));
     } |
     abstract_declarator no_arg
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_procdef,nil,nil));
     } |
     abstract_declarator LECKKLAMMER expr RECKKLAMMER
     {
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_arraydef,nil,$3));
     } |
     declarator LECKKLAMMER RECKKLAMMER
     {
       (* this is translated into a pointer *)
       hp:=$1;
       $$:=hp;
       while assigned(hp^.p1) do
         hp:=hp^.p1;
       hp^.p1:=new(presobject,init_two(t_arraydef,nil,nil));
     } |
     LKLAMMER abstract_declarator RKLAMMER
     {
       $$:=$2;
     } |
     {
       $$:=new(presobject,init_two(t_dec,nil,nil));
     }
     ;

expr    : shift_expr
          { $$:=$1; }
          ;

shift_expr :
          expr _ASSIGN expr
          { $$:=new(presobject,init_bop(':=',$1,$3)); }
          | expr EQUAL expr
          { $$:=new(presobject,init_bop('=',$1,$3));}
          | expr UNEQUAL expr
          { $$:=new(presobject,init_bop('<>',$1,$3));}
          | expr GT expr
          { $$:=new(presobject,init_bop('>',$1,$3));}
          | expr GTE expr
          { $$:=new(presobject,init_bop('>=',$1,$3));}
          | expr LT expr
          { $$:=new(presobject,init_bop('<',$1,$3));}
          | expr LTE expr
          { $$:=new(presobject,init_bop('<=',$1,$3));}
          | expr _PLUS expr
          { $$:=new(presobject,init_bop('+',$1,$3));}
          | expr MINUS expr
          { $$:=new(presobject,init_bop('-',$1,$3));}
               | expr STAR expr
          { $$:=new(presobject,init_bop('*',$1,$3));}
               | expr _SLASH expr
          { $$:=new(presobject,init_bop('/',$1,$3));}
               | expr _OR expr
          { $$:=new(presobject,init_bop(' or ',$1,$3));}
               | expr _AND expr
          { $$:=new(presobject,init_bop(' and ',$1,$3));}
               | expr _NOT expr
          { $$:=new(presobject,init_bop(' not ',$1,$3));}
               | expr _SHL expr
          { $$:=new(presobject,init_bop(' shl ',$1,$3));}
               | expr _SHR expr
          { $$:=new(presobject,init_bop(' shr ',$1,$3));}
          | expr QUESTIONMARK colon_expr
          {
            $3^.p1:=$1;
            $$:=$3;
            inc(if_nb);
            $$^.p:=strpnew('if_local'+str(if_nb));
          } |
          unary_expr {$$:=$1;}
          ;

colon_expr : expr COLON expr
       { (* if A then B else C *)
       $$:=new(presobject,init_three(t_ifexpr,nil,$1,$3));}
       ;

maybe_empty_unary_expr :
                  unary_expr
                  { $$:=$1; }
                  |
                  { $$:=nil;}
                  ;

unary_expr:
     dname
     {
     $$:=$1;
     } |
     special_type_name
     {
     $$:=$1;
     } |
     CSTRING
     {
     (* remove L prefix for widestrings *)
     s:=act_token;
     if Win32headers and (s[1]='L') then
       delete(s,1,1);
     $$:=new(presobject,init_id(''''+copy(s,2,length(s)-2)+''''));
     } |
     NUMBER
     {
     $$:=new(presobject,init_id(act_token));
     } |
     unary_expr POINT expr
     {
     $$:=new(presobject,init_bop('.',$1,$3));
     } |
     unary_expr DEREF expr
     {
     $$:=new(presobject,init_bop('^.',$1,$3));
     } |
     MINUS unary_expr
     {
     $$:=new(presobject,init_preop('-',$2));
     }|
     _AND unary_expr %prec R_AND
     {
     $$:=new(presobject,init_preop('@',$2));
     }|
     _NOT unary_expr
     {
     $$:=new(presobject,init_preop(' not ',$2));
     } |
     LKLAMMER dname RKLAMMER maybe_empty_unary_expr
     {
     if assigned($4) then
       $$:=new(presobject,init_two(t_typespec,$2,$4))
     else
       $$:=$2;
     } |
     LKLAMMER type_specifier RKLAMMER unary_expr
     {
     $$:=new(presobject,init_two(t_typespec,$2,$4));
     } |
     LKLAMMER type_specifier STAR RKLAMMER unary_expr
     {
     hp:=new(presobject,init_one(t_pointerdef,$2));
     $$:=new(presobject,init_two(t_typespec,hp,$5));
     } |
     LKLAMMER type_specifier size_overrider STAR RKLAMMER unary_expr
     {
     if not stripinfo then
      writeln(outfile,aktspace,'(* ',$3^.p,' ignored *)');
     dispose($3,done);
     write_type_specifier(outfile,$2);
     writeln(outfile,' ignored *)');
     hp:=new(presobject,init_one(t_pointerdef,$2));
     $$:=new(presobject,init_two(t_typespec,hp,$6));
     } |
     dname LKLAMMER exprlist RKLAMMER
     {
     hp:=new(presobject,init_one(t_exprlist,$1));
     $$:=new(presobject,init_three(t_funexprlist,hp,$3,nil));
     } |
     LKLAMMER shift_expr RKLAMMER
     {
     $$:=$2;
     } |
     LKLAMMER STAR unary_expr RKLAMMER maybe_space LKLAMMER exprlist RKLAMMER
     {
       $$:=new(presobject,init_two(t_callop,$3,$7));
     } |
     dname LECKKLAMMER exprlist RECKKLAMMER
     {
       $$:=new(presobject,init_two(t_arrayop,$1,$3));
     }
     ;

enum_list :
     enum_element COMMA enum_list
     { (*enum_element COMMA enum_list *)
       $$:=$1;
       $$^.next:=$3;
      } |
      enum_element {
       $$:=$1;
      } |
      {(* empty enum list *)
       $$:=nil;};

enum_element :
     dname _ASSIGN expr
     { begin (*enum_element: dname _ASSIGN expr *)
        $$:=new(presobject,init_two(t_enumlist,$1,$3));
       end;
     } |
     dname
     {
       begin (*enum_element: dname*)
       $$:=new(presobject,init_two(t_enumlist,$1,nil));
       end;
     };


def_expr :
     unary_expr
     {
         if $1^.typ=t_funexprlist then
           $$:=$1
         else
           $$:=new(presobject,init_two(t_exprlist,$1,nil));
         (* if here is a type specifier
            we know the return type *)
         if ($1^.typ=t_typespec) then
           $$^.p3:=$1^.p1^.get_copy;
     }
     ;

para_def_expr :
     SPACE_DEFINE def_expr
     {
     $$:=$2;
     } |
     maybe_space LKLAMMER def_expr RKLAMMER
     {
     $$:=$3
     }
     ;

exprlist : exprelem COMMA exprlist
    { (*exprlist COMMA expr*)
       $$:=$1;
       $1^.next:=$3;
     } |
     exprelem
     {
       $$:=$1;
     } |
     { (* empty expression list *)
       $$:=nil; };

exprelem :
           expr
           {
             $$:=new(presobject,init_one(t_exprlist,$1));
           };

%%

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
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
