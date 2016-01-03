{
    Copyright (c) 1998-2006 by the Free Pascal development team

    This unit implements an asmoutput class for m68k GAS syntax

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
unit ag68kgas;

{$i fpcdefs.inc}

interface

    uses
      cclasses,cpubase,systems,
      globals,globtype,
      aasmbase,aasmtai,aasmdata,aasmcpu,assemble,aggas;

    type
      Tm68kGNUAssembler=class(TGNUassembler)
        constructor create(info: pasminfo; smart: boolean); override;
        function MakeCmdLine : TCmdStr; override;
      end;

    type
      Tm68kInstrWriter=class(TCPUInstrWriter)
        procedure WriteInstruction(hp: tai);override;
      end;

    const
      gas_opsize2str : array[topsize] of string[2] =
        ('','.b','.w','.l','.s','.d','.x','');


  implementation

    uses
      cutils,
      cgbase,cgutils,cpuinfo,
      verbose,itcpugas;


 {****************************************************************************}
 {                         GNU m68k Assembler writer                          }
 {****************************************************************************}

 constructor Tm68kGNUAssembler.create(info: pasminfo; smart: boolean);
   begin
     inherited;
     InstrWriter := Tm68kInstrWriter.create(self);
   end;

 function Tm68kGNUAssembler.MakeCmdLine: TCmdStr;
   begin
     result:=inherited MakeCmdLine;
     // Use old -m option for Amiga system
     if target_info.system=system_m68k_amiga then
       Replace(result,'$ARCH','-m'+GasCpuTypeStr[current_settings.cputype])
     else
       Replace(result,'$ARCH','-march='+GasCpuTypeStr[current_settings.cputype]);
   end;

    function getreferencestring(var ref : treference) : string;
      var
         s,basestr,indexstr : string;

      begin
         s:='';
         with ref do
           begin
             basestr:=gas_regname(base);
             indexstr:=gas_regname(index);
             if assigned(symbol) then
               s:=s+symbol.name;

             if offset<0 then s:=s+tostr(offset)
              else if (offset>0) then
                begin
                  if (symbol=nil) then s:=tostr(offset)
                       else s:=s+'+'+tostr(offset);
                    end
                  else if (index=NR_NO) and (base=NR_NO) and not assigned(symbol) then
                    s:=s+'0';

               if (index<>NR_NO) and (base=NR_NO) and (direction=dir_none) then
                begin
                  if (scalefactor = 1) or (scalefactor = 0) then
                    s:=s+'('+indexstr+'.l)'
                  else
                    s:=s+'('+indexstr+'.l*'+tostr(scalefactor)+')'
                end
                else if (index=NR_NO) and (base<>NR_NO) and (direction=dir_inc) then
                begin
                  if (scalefactor = 1) or (scalefactor = 0) then
                      s:=s+'('+basestr+')+'
                  else
                   InternalError(10002);
                end
                else if (index=NR_NO) and (base<>NR_NO) and (direction=dir_dec) then
                begin
                  if (scalefactor = 1) or (scalefactor = 0) then
                      s:=s+'-('+basestr+')'
                  else
                   InternalError(10003);
                end
                  else if (index=NR_NO) and (base<>NR_NO) and (direction=dir_none) then
                begin
                  s:=s+'('+basestr+')'
                end
                  else if (index<>NR_NO) and (base<>NR_NO) and (direction=dir_none) then
                begin
                  if (scalefactor = 1) or (scalefactor = 0) then
                    s:=s+'('+basestr+','+indexstr+'.l)'
                  else
                    s:=s+'('+basestr+','+indexstr+'.l*'+tostr(scalefactor)+')';
                end;
          end;
         getreferencestring:=s;
      end;


    function getopstr(const o:toper) : string;
      var
        hs : string;
        i : tsuperregister;
      begin
        case o.typ of
          top_reg:
            getopstr:=gas_regname(o.reg);
          top_ref:
            if o.ref^.refaddr=addr_full then
              begin
                if assigned(o.ref^.symbol) then
                  hs:=o.ref^.symbol.name
                else
                  hs:='#';
                if o.ref^.offset>0 then
                  hs:=hs+'+'+tostr(o.ref^.offset)
                else
                  if o.ref^.offset<0 then
                    hs:=hs+tostr(o.ref^.offset)
                  else
                    if not(assigned(o.ref^.symbol)) then
                      hs:=hs+'0';
                getopstr:=hs;
              end
            else
              getopstr:=getreferencestring(o.ref^);
          top_regset:
            begin
              hs:='';
              for i:=RS_D0 to RS_D7 do
                begin
                  if i in o.dataregset^ then
                   hs:=hs+gas_regname(newreg(R_INTREGISTER,i,R_SUBWHOLE))+'/';
                end;
              for i:=RS_A0 to RS_SP do
                begin
                  if i in o.addrregset^ then
                   hs:=hs+gas_regname(newreg(R_ADDRESSREGISTER,i,R_SUBWHOLE))+'/';
                end;
              for i:=RS_FP0 to RS_FP7 do
                begin
                  if i in o.fpuregset^ then
                   hs:=hs+gas_regname(newreg(R_FPUREGISTER,i,R_SUBNONE))+'/';
                end;
              delete(hs,length(hs),1);
              getopstr := hs;
            end;
          top_const:
            getopstr:='#'+tostr(longint(o.val));
          else internalerror(200405021);
        end;
      end;


    function getopstr_jmp(const o:toper) : string;
      var
        hs : string;
      begin
        case o.typ of
          top_reg:
            getopstr_jmp:=gas_regname(o.reg);
          top_ref:
            if o.ref^.refaddr=addr_no then
              getopstr_jmp:=getreferencestring(o.ref^)
            else
              begin
                if assigned(o.ref^.symbol) then
                  hs:=o.ref^.symbol.name
                else
                  hs:='';
                if o.ref^.offset>0 then
                  hs:=hs+'+'+tostr(o.ref^.offset)
                else
                  if o.ref^.offset<0 then
                    hs:=hs+tostr(o.ref^.offset)
                  else
                    if not(assigned(o.ref^.symbol)) then
                      hs:=hs+'0';
                getopstr_jmp:=hs;
              end;
          top_const:
            getopstr_jmp:=tostr(o.val);
          else 
            internalerror(200405022);
        end;
      end;

{****************************************************************************
                            TM68kASMOUTPUT
 ****************************************************************************}

    { returns the opcode string }
    function getopcodestring(hp : tai) : string;
      var
        op : tasmop;
      begin
        op:=taicpu(hp).opcode;
        { old versions of GAS don't like PEA.L and LEA.L }
        if (op in [
          A_LEA,A_PEA,A_ABCD,A_BCHG,A_BCLR,A_BSET,A_BTST,
          A_EXG,A_NBCD,A_SBCD,A_SWAP,A_TAS,A_SCC,A_SCS,
          A_SEQ,A_SGE,A_SGT,A_SHI,A_SLE,A_SLS,A_SLT,A_SMI,
          A_SNE,A_SPL,A_ST,A_SVC,A_SVS,A_SF]) then
          result:=gas_op2str[op]
        else
        { Scc/FScc is always BYTE, DBRA/DBcc is always WORD, doesn't need opsize (KB) }
        if op in [A_SXX, A_FSXX, A_DBXX, A_DBRA] then
          result:=gas_op2str[op]+cond2str[taicpu(hp).condition]
        else
        if op in [a_bxx,a_fbxx] then
          result:=gas_op2str[op]+cond2str[taicpu(hp).condition]+gas_opsize2str[taicpu(hp).opsize]
        else
<<<<<<< HEAD
          result:=gas_op2str[op]+gas_opsize2str[taicpu(hp).opsize];
=======
        if op in [a_dbxx,a_bxx,a_fbxx] then
         s:=gas_op2str[op]+cond2str[taicpu(hp).condition]+gas_opsize2str[taicpu(hp).opsize]
        else
         s:=gas_op2str[op]+gas_opsize2str[taicpu(hp).opsize];
        if op = A_FMOVE then
          begin
{$ifdef DEBUG_CHARLIE}
            writeln('fmove! opsize:',dword(taicpu(hp).opsize));
{$endif DEBUG_CHARLIE}
          end;
        getopcodestring:=s;
>>>>>>> graemeg/fixes_2_2
      end;


    procedure Tm68kInstrWriter.WriteInstruction(hp: tai);
      var
        op       : tasmop;
        s        : string;
        sep      : char;
        calljmp  : boolean;
        i        : integer;
       begin
         if hp.typ <> ait_instruction then exit;
         op:=taicpu(hp).opcode;
         calljmp:=is_calljmp(op);
         { call maybe not translated to call }
         s:=#9+getopcodestring(hp);
         { process operands }
         if taicpu(hp).ops<>0 then
           begin
             { call and jmp need an extra handling                          }
             { this code is only called if jmp isn't a labeled instruction  }
             { quick hack to overcome a problem with manglednames=255 chars }
             if calljmp then
                begin
                  owner.writer.AsmWrite(s+#9);
                  s:=getopstr_jmp(taicpu(hp).oper[0]^);
                  { dbcc dx,<sym> has two operands! (KB) }
                  if (taicpu(hp).ops>1) then
                    s:=s+','+getopstr_jmp(taicpu(hp).oper[1]^);
                  if (taicpu(hp).ops>2) then
                    internalerror(2006120501);
                end
              else
                begin
                  for i:=0 to taicpu(hp).ops-1 do
                    begin
                      if i=0 then
                        sep:=#9
                      else
                      if ((op = A_DIVSL) or
                         (op = A_DIVUL) or
                         (op = A_MULU) or
                         (op = A_MULS) or
                         (op = A_DIVS) or
                         (op = A_DIVU)) and (i=2) then
                      begin
                        sep:=':'
                      end else
                        sep:=',';
                      s:=s+sep+getopstr(taicpu(hp).oper[i]^)
                    end;
                end;
           end;
           owner.writer.AsmWriteLn(s);
       end;


{*****************************************************************************
                                  Initialize
*****************************************************************************}

    const
       as_m68k_as_info : tasminfo =
          (
            id     : as_gas;
            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '$ARCH -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_m68k_Amiga,system_m68k_Atari,system_m68k_Mac,system_m68k_linux,system_m68k_PalmOS,system_m68k_netbsd,system_m68k_openbsd,system_m68k_embedded];
            flags : [af_needar,af_smartlink_sections];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );

initialization
  RegisterAssembler(as_m68k_as_info,Tm68kGNUAssembler);
end.
