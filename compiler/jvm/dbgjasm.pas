{
    Copyright (c) 2003-2006 by Peter Vreman, Florian Klaempfl, and Jonas Maebe

    This units contains support for Jasmin debug info generation

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
unit dbgjasm;

{$i fpcdefs.inc}

interface

    uses
      cclasses,globtype,
      aasmbase,aasmtai,aasmdata,
      symbase,symconst,symtype,symdef,symsym,
      finput,
      DbgBase;

    type
      { TDebugInfoJasmin }

      TDebugInfoJasmin=class(TDebugInfo)
      protected
        fcurrprocstart,
        fcurrprocafterstart,
        fcurrprocend: tasmsymbol;

        procedure appendsym_localsym(list: TAsmList; sym: tabstractnormalvarsym; startlab: tasmsymbol);

        procedure appendsym_paravar(list:TAsmList;sym:tparavarsym);override;
        procedure appendsym_localvar(list:TAsmList;sym:tlocalvarsym);override;
        procedure beforeappenddef(list:TAsmList;def:tdef);override;
        procedure appendprocdef(list:TAsmList;def:tprocdef);override;
      public
        procedure inserttypeinfo;override;
        procedure insertlineinfo(list:TAsmList);override;
      end;

implementation

    uses
      sysutils,cutils,cfileutl,constexp,
      version,globals,verbose,systems,
      cpubase,cpuinfo,cgbase,paramgr,
      fmodule,
      defutil,symtable,symcpu,jvmdef,ppu
      ;

{****************************************************************************
                              TDebugInfoJasmin
****************************************************************************}

  procedure TDebugInfoJasmin.appendsym_localsym(list: TAsmList; sym: tabstractnormalvarsym; startlab: tasmsymbol);
    var
      jvar: tai_jvar;
      proc: tprocdef;
    begin
      if tdef(sym.owner.defowner).typ<>procdef then
        exit;
      if not(sym.localloc.loc in [LOC_REFERENCE,LOC_CREFERENCE]) then
        exit;
      proc:=tprocdef(sym.owner.defowner);
      jvar:=tai_jvar.create(sym.localloc.reference.offset,jvmmangledbasename(sym,true),startlab,fcurrprocend);
      tcpuprocdef(proc).exprasmlist.InsertAfter(jvar,proc.procstarttai);
    end;


  procedure TDebugInfoJasmin.appendsym_paravar(list: TAsmList; sym: tparavarsym);
    begin
      appendsym_localsym(list,sym,fcurrprocstart);
    end;


  procedure TDebugInfoJasmin.appendsym_localvar(list: TAsmList; sym: tlocalvarsym);
    begin
      appendsym_localsym(list,sym,fcurrprocafterstart);
    end;


  procedure TDebugInfoJasmin.beforeappenddef(list: TAsmList; def: tdef);
    begin
    end;


  procedure TDebugInfoJasmin.appendprocdef(list: TAsmList; def: tprocdef);
    var
      procstartlabel,
      procendlabel,
      afterprocstartlabel : tasmlabel;
      hp,
      afterproccodestart  : tai;
    begin
      { insert debug information for local variables and parameters, but only
        for routines implemented in the Pascal code }
      if not assigned(def.procstarttai) then
        exit;

      current_asmdata.getlabel(procstartlabel,alt_dbgtype);
      current_asmdata.getlabel(procendlabel,alt_dbgtype);
      tcpuprocdef(def).exprasmlist.insertafter(tai_label.create(procstartlabel),def.procstarttai);
      tcpuprocdef(def).exprasmlist.insertbefore(tai_label.create(procendlabel),def.procendtai);

      fcurrprocstart:=procstartlabel;
      { set the start label for local variables after the first instruction,
        because javac's code completion support assumes that all info at
        bytecode position 0 is for parameters }
      afterproccodestart:=def.procstarttai;
      while assigned(afterproccodestart.next) do
        begin
          afterproccodestart:=tai(afterproccodestart.next);
          if (afterproccodestart.typ=ait_instruction) then
            break;
        end;
      { must be followed by at least one more instruction }
      hp:=tai(afterproccodestart.next);
      while assigned(hp) do
        begin
          if hp.typ=ait_instruction then
            break;
          hp:=tai(hp.next);
        end;
      if assigned(hp) then
        begin
          current_asmdata.getlabel(afterprocstartlabel,alt_dbgtype);
          tcpuprocdef(def).exprasmlist.insertafter(tai_label.create(afterprocstartlabel),afterproccodestart);
          fcurrprocafterstart:=afterprocstartlabel;
        end
      else
        fcurrprocafterstart:=procstartlabel;
      fcurrprocend:=procendlabel;

      write_symtable_parasyms(list,def.paras);
      { not assigned for unit init }
      if assigned(def.localst) then
        write_symtable_syms(list,def.localst);
    end;


  procedure TDebugInfoJasmin.inserttypeinfo;
    begin
      { write all procedures and methods }
      if assigned(current_module.globalsymtable) then
        write_symtable_procdefs(nil,current_module.globalsymtable);
      if assigned(current_module.localsymtable) then
        write_symtable_procdefs(nil,current_module.localsymtable);
    end;

  procedure TDebugInfoJasmin.insertlineinfo(list: TAsmList);
    var
      currfileinfo,
      lastfileinfo : tfileposinfo;
      nolineinfolevel : Integer;
      currfuncname : pshortstring;
      hp : tai;
    begin
      FillChar(lastfileinfo,sizeof(lastfileinfo),0);
      hp:=Tai(list.first);
      nolineinfolevel:=0;
      while assigned(hp) do
        begin
          case hp.typ of
            ait_function_name :
              begin
                currfuncname:=tai_function_name(hp).funcname;
                list.concat(tai_comment.Create(strpnew('function: '+currfuncname^)));
              end;
            ait_force_line :
              begin
                lastfileinfo.line:=-1;
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

          { Java does not support multiple source files }
          if (hp.typ=ait_instruction) and
             (nolineinfolevel=0) and
             (tailineinfo(hp).fileinfo.fileindex=main_module.unit_index) then
            begin
              currfileinfo:=tailineinfo(hp).fileinfo;

              { line changed ? }
              if (lastfileinfo.line<>currfileinfo.line) and (currfileinfo.line<>0) then
                begin
                  { line directive }
                  list.insertbefore(tai_directive.Create(asd_jline,tostr(currfileinfo.line)),hp);
                end;
              lastfileinfo:=currfileinfo;
            end;

          hp:=tai(hp.next);
        end;
    end;


{****************************************************************************
****************************************************************************}
    const
      dbg_jasmin_info : tdbginfo =
         (
           id     : dbg_jasmin;
           idtxt  : 'JASMIN';
         );


initialization
  RegisterDebugInfo(dbg_jasmin_info,TDebugInfoJasmin);

end.
