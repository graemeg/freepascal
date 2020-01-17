{
    Copyright (c) 2003,2014 by Florian Klaempfl and Jonas Maebe

    This unit implements an asm for AArch64

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
{ This unit implements the GNU Assembler writer for AArch64
}

unit agcpugas;

{$i fpcdefs.inc}

  interface

    uses
       globtype,systems,
       aasmtai,
       assemble,aggas,
       cpubase,cpuinfo;

    type
      TAArch64InstrWriter=class(TCPUInstrWriter)
        procedure WriteInstruction(hp : tai);override;
      end;

      TAArch64Assembler=class(TGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      end;

      TAArch64AppleAssembler=class(TAppleGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      end;


    const
      gas_shiftmode2str : array[tshiftmode] of string[4] = (
        '','lsl','lsr','asr','ror',
        'uxtb','uxth','uxtw','uxtx',
        'sxtb','sxth','sxtw','sxtx');

    const 
      cputype_to_gas_march : array[tcputype] of string = (
        '', // cpu_none
        'armv8'
      );

  implementation

    uses
       cutils,globals,verbose,
       aasmcpu,
       itcpugas,
       cgbase,cgutils;


{****************************************************************************}
{                      AArch64 Assembler writer                              }
{****************************************************************************}

    constructor TAArch64Assembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TAArch64InstrWriter.create(self);
      end;

{****************************************************************************}
{                      Apple AArch64 Assembler writer                        }
{****************************************************************************}

    constructor TAArch64AppleAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TAArch64InstrWriter.create(self);
      end;


{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

    function getreferencestring(asminfo: pasminfo; var ref : treference) : string;
      const
        darwin_addrpage2str: array[addr_page..addr_gotpageoffset] of string[11] =
           ('@PAGE','@PAGEOFF','@GOTPAGE','@GOTPAGEOFF');
        linux_addrpage2str: array[addr_page..addr_gotpageoffset] of string[10] =
           ('',':lo12:',':got:',':got_lo12:');
      begin
        if ref.base=NR_NO then
          begin
            case ref.refaddr of
              addr_gotpage,
              addr_page,
              addr_gotpageoffset,
              addr_pageoffset:
                begin
                  if not assigned(ref.symbol) or
                     (ref.base<>NR_NO) or
                     (ref.index<>NR_NO) or
                     (ref.shiftmode<>SM_None) or
                     (ref.offset<>0) then
                    internalerror(2014121501);
                  if target_info.system in systems_darwin then
                    result:=ref.symbol.name+darwin_addrpage2str[ref.refaddr]
                  else
                    result:=linux_addrpage2str[ref.refaddr]+ref.symbol.name
                end;
              addr_pic,
              { for locals replaced by temp symbols on LLVM }
              addr_no:
                result:=ref.symbol.name;
              else
                internalerror(2015022302);
            end
          end
        else
          begin
            result:='['+gas_regname(ref.base);
            if ref.addressmode=AM_POSTINDEXED then
              result:=result+']';
            if ref.index<>NR_NO then
              begin
                if (ref.offset<>0) or
                   assigned(ref.symbol) then
                  internalerror(2014121504);
                result:=result+', '+gas_regname(ref.index);
                case ref.shiftmode of
                  SM_None: ;
                  SM_LSL,
                  SM_UXTW, SM_UXTX, SM_SXTW, SM_SXTX:
                    begin
                      result:=result+', '+gas_shiftmode2str[ref.shiftmode];
                      if (ref.shiftmode=SM_LSL) or
                         (ref.shiftimm<>0) then
                        result:=result+' #'+tostr(ref.shiftimm);
                    end
                  else
                    internalerror(2014121505);
                end;
              end
            else
              begin
                if assigned(ref.symbol) then
                  begin
                    case ref.refaddr of
                      addr_gotpageoffset,
                      addr_pageoffset:
                        begin
                          if target_info.system in systems_darwin then
                            result:=result+', '+ref.symbol.name+darwin_addrpage2str[ref.refaddr]
                          else
                            result:=result+', '+linux_addrpage2str[ref.refaddr]+ref.symbol.name
                        end
                      else
                        { todo: not yet generated/don't know syntax }
                        internalerror(2014121506);
                    end;
                  end
                else
                  begin
                    if ref.refaddr<>addr_no then
                      internalerror(2014121506);
                    if (ref.offset<>0) then
                      result:=result+', #'+tostr(ref.offset);
                  end;
              end;
            case ref.addressmode of
              AM_OFFSET:
                result:=result+']';
              AM_PREINDEXED:
                result:=result+']!';
              else
                ;
            end;
          end;
      end;


    function getopstr(asminfo: pasminfo; hp: taicpu; opnr: longint; const o: toper): string;
      begin
        case o.typ of
          top_reg:
            { we cannot yet represent "umov w0, v4.s[0]" or "ins v4.d[0], x1",
              so for now we use "s4" or "d4" instead -> translate here }
            if ((hp.opcode=A_INS) or
                (hp.opcode=A_UMOV)) and
               (getregtype(hp.oper[opnr]^.reg)=R_MMREGISTER) then
              begin
                case getsubreg(hp.oper[opnr]^.reg) of
                  R_SUBMMS:
                    getopstr:='v'+tostr(getsupreg(hp.oper[opnr]^.reg))+'.S[0]';
                  R_SUBMMD:
                    getopstr:='v'+tostr(getsupreg(hp.oper[opnr]^.reg))+'.D[0]';
                  else
                    internalerror(2014122907);
                end;
              end
            else
              getopstr:=gas_regname(o.reg);
          top_shifterop:
            begin
              getopstr:=gas_shiftmode2str[o.shifterop^.shiftmode];
              if o.shifterop^.shiftimm<>0 then
                getopstr:=getopstr+' #'+tostr(o.shifterop^.shiftimm)
            end;
          top_const:
            if o.val>=0 then
              getopstr:='#'+tostr(o.val)
            else
              getopstr:='#0x'+hexStr(o.val,16);
          top_conditioncode:
            getopstr:=cond2str[o.cc];
          top_ref:
            if is_calljmp(hp.opcode) then
              begin
                if o.ref^.refaddr<>addr_full then
                  internalerror(2014122220);
                if not assigned(o.ref^.symbol) or
                   assigned(o.ref^.relsymbol) or
                   (o.ref^.base<>NR_NO) or
                   (o.ref^.index<>NR_NO) or
                   (o.ref^.offset<>0) then
                  internalerror(2014122221);
                getopstr:=o.ref^.symbol.name;
              end
            else
              getopstr:=getreferencestring(asminfo,o.ref^);
          top_realconst:
            begin
              str(o.val_real,Result);
              Result:='#'+Result;
            end
          else
            internalerror(2014121507);
        end;
      end;


    procedure TAArch64InstrWriter.WriteInstruction(hp : tai);
      var
        op: TAsmOp;
        s: string;
        i: byte;
        sep: string[3];
      begin
        op:=taicpu(hp).opcode;
        s:=#9+gas_op2str[op]+oppostfix2str[taicpu(hp).oppostfix];
        if taicpu(hp).condition<>C_NONE then
          s:=s+'.'+cond2str[taicpu(hp).condition];
        if taicpu(hp).ops<>0 then
          begin
            sep:=#9;
            for i:=0 to taicpu(hp).ops-1 do
              begin
                 // debug code
                 // writeln(s);
                 // writeln(taicpu(hp).fileinfo.line);
                 s:=s+sep+getopstr(owner.asminfo,taicpu(hp),i,taicpu(hp).oper[i]^);
                 sep:=',';
              end;
          end;
        owner.writer.AsmWriteLn(s);
      end;


    const
       as_aarch64_gas_info : tasminfo =
          (
            id     : as_gas;
            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_aarch64_linux,system_aarch64_android];
            flags : [af_needar,af_smartlink_sections];
            labelprefix : '.L';
            comment : '// ';
            dollarsign: '$';
          );

       as_aarch64_clang_darwin_info : tasminfo =
          (
            id     : as_clang;
            idtxt  : 'CLANG';
            asmbin : 'clang';
            asmcmd : '-c -o $OBJ $EXTRAOPT -arch arm64 $DARWINVERSION -x assembler $ASM';
            supported_targets : [system_aarch64_darwin];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : 'L';
            comment : '# ';
            dollarsign: '$';
          );


begin
  RegisterAssembler(as_aarch64_gas_info,TAArch64Assembler);
  RegisterAssembler(as_aarch64_clang_darwin_info,TAArch64AppleAssembler);
end.
