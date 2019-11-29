{
    Copyright (c) 1999-2009 by Florian Klaempfl and David Zhang

    This unit implements an asmoutput class for MIPS assembly syntax

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
unit cpugas;

{$i fpcdefs.inc}

  interface

    uses
      cpubase, aasmbase, globtype, systems,
      aasmtai, aasmcpu, assemble, aggas;

    type
      TMIPSGNUAssembler = class(TGNUassembler)
        nomacro, noreorder, noat : boolean;
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
        {# Constructs the command line for calling the assembler }
        function MakeCmdLine: TCmdStr; override;
        procedure WriteExtraHeader; override;
      end;

      TMIPSInstrWriter = class(TCPUInstrWriter)
        procedure WriteInstruction(hp : tai);override;
      end;

    const
      use_std_regnames : boolean =
      {$ifndef USE_MIPS_GAS_REGS}
      true;
      {$else}
      false;
      {$endif}

  implementation

    uses
      cutils, cpuinfo,
      globals, verbose, itcpugas, cgbase, cgutils;


      function asm_regname(reg : TRegister) : string;

        begin
          if use_std_regnames then
            asm_regname:='$'+std_regname(reg)
          else
            asm_regname:=gas_regname(reg);
        end;

{****************************************************************************}
{                         GNU MIPS  Assembler writer                           }
{****************************************************************************}

    constructor TMIPSGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter:=TMIPSInstrWriter.create(self);
        nomacro:=false;
        noreorder:=false;
        noat:=false;
      end;

    function TMIPSGNUAssembler.MakeCmdLine: TCmdStr;
      begin
         result := Inherited MakeCmdLine;
         { ABI selection }
         Replace(result,'$ABI','-mabi='+abitypestr[mips_abi]);
         { ARCH selection }
         Replace(result,'$ARCH','-march='+lower(cputypestr[current_settings.cputype]));
//          Replace(result,'$ARCH','-march=pic32mx -mtune=pic32mx');      
      end;

    procedure TMIPSGNUAssembler.WriteExtraHeader;
      var
        i : longint;
      begin
        if not (cs_asm_pre_binutils_2_25 in current_settings.globalswitches) then
          writer.AsmWriteln(#9'.module nomips16');
      end;


{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

    function GetReferenceString(var ref: TReference): string;
      var
        reg: TRegister;
        regstr: string;
      begin
        result:='';
        if assigned(ref.symbol) then
          result:=ref.symbol.name;
        if (ref.offset<0) then
          result:=result+tostr(ref.offset)
        else if (ref.offset>0) then
          begin
            if assigned(ref.symbol) then
              result:=result+'+';
            result:=result+tostr(ref.offset);
          end
        { asmreader appears to treat literal numbers as references }
        else if (ref.symbol=nil) and (ref.base=NR_NO) and (ref.index=NR_NO) then
          result:='0';

        { either base or index may be present, but not both }
        reg:=ref.base;
        if (reg=NR_NO) then
          reg:=ref.index
        else if (ref.index<>NR_NO) then
          InternalError(2013013001);

        if (reg=NR_NO) then
          regstr:=''
        else
          regstr:='('+asm_regname(reg)+')';

        case ref.refaddr of
          addr_no,
          addr_full:
            if assigned(ref.symbol) and (reg<>NR_NO) then
              InternalError(2013013002)
            else
              begin
                result:=result+regstr;
                exit;
              end;
          addr_pic:
            result:='%got('+result;
          addr_high:
            result:='%hi('+result;
          addr_low:
            result:='%lo('+result;
          addr_pic_call16:
            result:='%call16('+result;
          addr_low_pic:
            result:='%got_lo('+result;
          addr_high_pic:
            result:='%got_hi('+result;
          addr_low_call:
            result:='%call_lo('+result;
          addr_high_call:
            result:='%call_hi('+result;
        else
          InternalError(2013013003);
        end;

        result:=result+')'+regstr;
      end;


    function getopstr(const Oper: TOper): string;
      begin
        with Oper do
          case typ of
            top_reg:
              getopstr := asm_regname(reg);
            top_const:
              getopstr := tostr(longint(val));
            top_ref:
              getopstr := getreferencestring(ref^);
            else
              internalerror(10001);
          end;
      end;


    procedure TMIPSInstrWriter.WriteInstruction(hp: Tai);
      var
        Op: TAsmOp;
        s:  string;
        i:  integer;
      begin
        if hp.typ <> ait_instruction then
          exit;
        op := taicpu(hp).opcode;

        case op of
          A_P_SET_NOMIPS16:
            begin
              owner.writer.AsmWriteLn(#9'.set'#9'nomips16');
            end;
          A_P_MASK,
          A_P_FMASK:
            begin
              s := #9 + gas_op2str[op] + #9'0x' + hexstr(taicpu(hp).oper[0]^.val,8)+ ',' + getopstr(taicpu(hp).oper[1]^) ;
              owner.writer.AsmWriteLn(s);
            end;
          A_P_SET_MACRO:
            begin
              owner.writer.AsmWriteLn(#9'.set'#9'macro');
              TMIPSGNUAssembler(owner).nomacro:=false;
            end;
          A_P_SET_REORDER:
            begin
              owner.writer.AsmWriteLn(#9'.set'#9'reorder');
              TMIPSGNUAssembler(owner).noreorder:=false;
            end;
          A_P_SET_NOMACRO:
            begin
              owner.writer.AsmWriteLn(#9'.set'#9'nomacro');
              TMIPSGNUAssembler(owner).nomacro:=true;
            end;
          A_P_SET_NOREORDER:
            begin
              owner.writer.AsmWriteLn(#9'.set'#9'noreorder');
              TMIPSGNUAssembler(owner).noreorder:=true;
            end;
          A_P_SET_NOAT:
            begin
              owner.writer.AsmWriteln(#9'.set'#9'noat');
              TMIPSGNUAssembler(owner).noat:=true;
            end;
          A_P_SET_AT:
            begin
              owner.writer.AsmWriteln(#9'.set'#9'at');
              TMIPSGNUAssembler(owner).noat:=false;
            end;
          else
            begin
              if taicpu(hp).is_macro and TMIPSGNUAssembler(owner).nomacro then
                owner.writer.AsmWriteln(#9'.set'#9'macro');
              s := #9 + gas_op2str[op] + cond2str[taicpu(hp).condition];
              if taicpu(hp).ops > 0 then
              begin
                s := s + #9 + getopstr(taicpu(hp).oper[0]^);
                for i := 1 to taicpu(hp).ops - 1 do
                  s := s + ',' + getopstr(taicpu(hp).oper[i]^);
              end;
              owner.writer.AsmWriteLn(s);
              if taicpu(hp).is_macro and TMIPSGNUAssembler(owner).nomacro then
                owner.writer.AsmWriteln(#9'.set'#9'nomacro');
            end;
        end;
      end;


    const
{$ifdef MIPSEL}
      as_MIPSEL_as_info: tasminfo =
        (
        id: as_gas;
        idtxt: 'AS';
        asmbin: 'as';
        asmcmd: '$ABI $ARCH $NOWARN -EL $PIC -o $OBJ $EXTRAOPT $ASM';
        supported_targets: [system_mipsel_linux,system_mipsel_android,system_mipsel_embedded];
        flags: [ af_needar, af_smartlink_sections];
        labelprefix: '.L';
        comment: '# ';
        dollarsign: '$';
        );
{$else MIPSEL}
      as_MIPSEB_as_info: tasminfo =
        (
        id: as_gas;
        idtxt: 'AS';
        asmbin: 'as';
        asmcmd: '$ABI $ARCH $NOWARN -EB $PIC -o $OBJ $EXTRAOPT $ASM';
        supported_targets: [system_mipseb_linux];
        flags: [ af_needar, af_smartlink_sections];
        labelprefix: '.L';
        comment: '# ';
        dollarsign: '$';
        );
{$endif MIPSEL}

begin
{$ifdef MIPSEL}
  RegisterAssembler(as_MIPSEL_as_info, TMIPSGNUAssembler);
{$else MIPSEL}
  RegisterAssembler(as_MIPSEB_as_info, TMIPSGNUAssembler);
{$endif MIPSEL}
end.
