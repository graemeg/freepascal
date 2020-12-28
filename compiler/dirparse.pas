{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements some support functions for the parsing of directives
    and option strings

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

unit dirparse;

{$i fpcdefs.inc}

interface

    uses
      globtype,
      systems;

    function UpdateAlignmentStr(s:string;var a:talignmentinfo):boolean;
    function UpdateOptimizerStr(s:string;var a:toptimizerswitches):boolean;
    function UpdateWpoStr(s: string; var a: twpoptimizerswitches): boolean;
    function UpdateDebugStr(s:string;var a:tdebugswitches):boolean;
    function UpdateTargetSwitchStr(s: string; var a: ttargetswitches; global: boolean): boolean;

implementation

    uses
      globals,
      cutils,
      symtable;

    function UpdateAlignmentStr(s:string;var a:talignmentinfo):boolean;
      var
        tok  : string;
        vstr : string;
        l    : longint;
        code : integer;
        b    : talignmentinfo;
      begin
        UpdateAlignmentStr:=true;
        uppervar(s);
        fillchar(b,sizeof(b),0);
        repeat
          tok:=GetToken(s,'=');
          if tok='' then
           break;
          vstr:=GetToken(s,',');
          val(vstr,l,code);
          if tok='PROC' then
           b.procalign:=l
          else if tok='JUMP' then
           b.jumpalign:=l
          else if tok='LOOP' then
           b.loopalign:=l
          else if tok='CONSTMIN' then
           begin
             b.constalignmin:=l;
             if l>b.constalignmax then
               b.constalignmax:=l;
           end
          else if tok='CONSTMAX' then
           b.constalignmax:=l
          else if tok='VARMIN' then
           begin
             b.varalignmin:=l;
             if l>b.varalignmax then
               b.varalignmax:=l;
           end
          else if tok='VARMAX' then
           b.varalignmax:=l
          else if tok='LOCALMIN' then
           begin
             b.localalignmin:=l;
             if l>b.localalignmax then
               b.localalignmax:=l;
           end
          else if tok='LOCALMAX' then
           b.localalignmax:=l
          else if tok='RECORDMIN' then
           begin
             b.recordalignmin:=l;
             if l>b.recordalignmax then
               b.recordalignmax:=l;
           end
          else if tok='RECORDMAX' then
           b.recordalignmax:=l

{          disabled for now as - as Jonas pointed out - this
           is a matter of abi

           else if tok='MAXCRECORD' then
           b.maxCrecordalign:=l }
          else { Error }
           UpdateAlignmentStr:=false;
        until false;
        Result:=Result and UpdateAlignment(a,b);
      end;


    function UpdateOptimizerStr(s:string;var a:toptimizerswitches):boolean;
      var
        tok   : string;
        doset,
        found : boolean;
        opt   : toptimizerswitch;
      begin
        result:=true;
        uppervar(s);
        repeat
          tok:=GetToken(s,',');
          if tok='' then
           break;
          if Copy(tok,1,2)='NO' then
            begin
              delete(tok,1,2);
              doset:=false;
            end
          else
            doset:=true;
          found:=false;
          for opt:=low(toptimizerswitch) to high(toptimizerswitch) do
            begin
              if OptimizerSwitchStr[opt]=tok then
                begin
                  found:=true;
                  break;
                end;
            end;
          if found then
            begin
{$ifdef llvm}
             { -Ooregvar is not supported, llvm will take care of that }
             if opt<>cs_opt_regvar then
{$endif llvm}
              if doset then
                include(a,opt)
              else
                exclude(a,opt);
            end
          else
            result:=false;
        until false;
      end;


    function UpdateWpoStr(s: string; var a: twpoptimizerswitches): boolean;
      var
        tok   : string;
        doset,
        found : boolean;
        opt   : twpoptimizerswitch;
      begin
        result:=true;
        uppervar(s);
        repeat
          tok:=GetToken(s,',');
          if tok='' then
           break;
          if Copy(tok,1,2)='NO' then
            begin
              delete(tok,1,2);
              doset:=false;
            end
          else
            doset:=true;
          found:=false;
          if (tok = 'ALL') then
            begin
              for opt:=low(twpoptimizerswitch) to high(twpoptimizerswitch) do
                if doset then
                  include(a,opt)
                else
                  exclude(a,opt);
            end
          else
            begin
              for opt:=low(twpoptimizerswitch) to high(twpoptimizerswitch) do
                begin
                  if WPOptimizerSwitchStr[opt]=tok then
                    begin
                      found:=true;
                      break;
                    end;
                end;
              if found then
                begin
                  if doset then
                    include(a,opt)
                  else
                    exclude(a,opt);
                end
              else
                result:=false;
            end;
        until false;
      end;


    function UpdateDebugStr(s:string;var a:tdebugswitches):boolean;
      var
        tok   : string;
        doset,
        found : boolean;
        opt   : tdebugswitch;
      begin
        result:=true;
        uppervar(s);
        repeat
          tok:=GetToken(s,',');
          if tok='' then
           break;
          if Copy(tok,1,2)='NO' then
            begin
              delete(tok,1,2);
              doset:=false;
            end
          else
            doset:=true;
          found:=false;
          for opt:=low(tdebugswitch) to high(tdebugswitch) do
            begin
              if DebugSwitchStr[opt]=tok then
                begin
                  found:=true;
                  break;
                end;
            end;
          if found then
            begin
              if doset then
                include(a,opt)
              else
                exclude(a,opt);
            end
          else
            result:=false;
        until false;
      end;


    function UpdateTargetSwitchStr(s: string; var a: ttargetswitches; global: boolean): boolean;
      var
        tok,
        value : string;
        setstr: string[2];
        equalspos: longint;
        doset,
        gotvalue,
        found : boolean;
        opt   : ttargetswitch;
      begin
        result:=true;
        value:='';
        repeat
          tok:=GetToken(s,',');
          if tok='' then
           break;
          setstr:=upper(copy(tok,length(tok),1));
          if setstr='-' then
            begin
              setlength(tok,length(tok)-1);
              doset:=false;
            end
          else
            doset:=true;
          { value specified? }
          gotvalue:=false;
          equalspos:=pos('=',tok);
          if equalspos<>0 then
            begin
              value:=copy(tok,equalspos+1,length(tok));
              delete(tok,equalspos,length(tok));
              gotvalue:=true;
            end;
          found:=false;
          uppervar(tok);
          for opt:=low(ttargetswitch) to high(ttargetswitch) do
            begin
              if TargetSwitchStr[opt].name=tok then
                begin
                  found:=true;
                  break;
                end;
            end;
          if found then
            begin
              if not global and
                 TargetSwitchStr[opt].isglobal then
                result:=false
              else if not TargetSwitchStr[opt].hasvalue then
                begin
                  if gotvalue then
                    result:=false;
                  if (TargetSwitchStr[opt].define<>'') and (doset xor (opt in a)) then
                    if doset then
                      def_system_macro(TargetSwitchStr[opt].define)
                    else
                      undef_system_macro(TargetSwitchStr[opt].define);
                  if doset then
                    include(a,opt)
                  else
                    exclude(a,opt)
                end
              else
                begin
                  if not gotvalue or
                     not doset then
                    result:=false
                  else
                    begin
                      case opt of
                        ts_auto_getter_prefix:
                          prop_auto_getter_prefix:=value;
                        ts_auto_setter_predix:
                          prop_auto_setter_prefix:=value;
                        else
                          begin
                            writeln('Internalerror 2012053001');
                            halt(1);
                          end;
                      end;
                    end;
                end;
            end
          else
            result:=false;
        until false;
      end;

end.
