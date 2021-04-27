{
    Copyright (c) 2016 by the Free Pascal development team

    This unit is the VASM assembler writer for 68k

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

unit ag68kvasm;

{$i fpcdefs.inc}

  interface

    uses
       aasmbase,systems,
       aasmtai,aasmdata,
       assemble,aggas,ag68kgas,
       cpubase,cgutils,
       globtype;

  type
    tm68kvasm = class(Tm68kGNUassembler)
    protected
      function sectionattrs(atype:TAsmSectiontype):string; override;
    public
      constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      function MakeCmdLine: TCmdStr; override;
    end;

  implementation

    uses
       cutils,cfileutl,globals,verbose,
       cgbase,
       cscript,
       itcpugas,cpuinfo,
       aasmcpu;


{****************************************************************************}
{                         VASM m68k Assembler writer                         }
{****************************************************************************}


    constructor tm68kvasm.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := Tm68kInstrWriter.create(self);
      end;

    function tm68kvasm.sectionattrs(atype:TAsmSectiontype):string;
      begin
        case atype of
          sec_code, sec_fpc, sec_init, sec_fini:
            result:='acrx';
          { map sec_rodata as read-write, otherwise the linker (vlink) complains if it
            has to write into the relocations in a rodata section. (KB) }
          sec_data, sec_rodata:
            result:='adrw';
          sec_rodata_norel:
            case target_info.system of
              { stop vlink from complaining when it merges ro sections into rw ones (KB) }
              system_m68k_atari: result:='adrw';
              system_m68k_amiga: result:='adrw';
            else
              result:='adr';
            end;
          sec_bss, sec_threadvar:
            result:='aurw';
          sec_stab, sec_stabstr:
            result:='dr';
          else
            result:='';
        end;
      end;

    function tm68kvasm.MakeCmdLine: TCmdStr;
      var
        objtype: string;
      begin
        result:=asminfo^.asmcmd;

        case target_info.system of
          { a.out doesn't support named sections, lets use ELF for interoperability }
          system_m68k_amiga,
          system_m68k_atari,
          system_m68k_sinclairql: objtype:='-Felf';
        else
          internalerror(2016052601);
        end;

        if (target_info.system = system_m68k_amiga) then 
          begin
            Replace(result,'$ASM',maybequoted(ScriptFixFileName(Unix2AmigaPath(AsmFileName))));
            Replace(result,'$OBJ',maybequoted(ScriptFixFileName(Unix2AmigaPath(ObjFileName))));
          end
        else
          begin
            Replace(result,'$ASM',maybequoted(ScriptFixFileName(AsmFileName)));
            Replace(result,'$OBJ',maybequoted(ScriptFixFileName(ObjFileName)));
          end;
        Replace(result,'$ARCH','-m'+GasCpuTypeStr[current_settings.cputype]);
        Replace(result,'$OTYPE',objtype);
        Replace(result,'$EXTRAOPT',asmextraopt);
      end;



{*****************************************************************************
                                  Initialize
*****************************************************************************}

  const
    as_m68k_vasm_info : tasminfo =
       (
         id     : as_m68k_vasm;

         idtxt  : 'VASM';
         asmbin : 'vasmm68k_std';
         asmcmd:  '-quiet -elfregs -gas $OTYPE $ARCH -o $OBJ $EXTRAOPT $ASM';
         supported_targets : [system_m68k_amiga,system_m68k_atari,system_m68k_sinclairql];
         flags : [af_needar,af_smartlink_sections];
         labelprefix : '.L';
         labelmaxlen : -1;
         comment : '# ';
         dollarsign: '$';
       );

begin
  RegisterAssembler(as_m68k_vasm_info,tm68kvasm);
end.
