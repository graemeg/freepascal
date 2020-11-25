{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit the GAS asm writers for Risc-V32/Risc-V64

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

{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

unit agrvgas;

{$i fpcdefs.inc}

  interface
  
    uses
       systems,aasmbase,
       aasmtai,aasmdata,
       aggas,
       assemble,
       cpubase,cgutils,
       globtype;

  type
    TRVInstrWriter=class(TCPUInstrWriter)
       procedure WriteInstruction(hp : tai);override;
    end;

    TRVGNUAssembler=class(TGNUassembler)
      constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      function MakeCmdLine: TCmdStr; override;
    end;

  implementation

    uses
       cutils,globals,verbose,
       cgbase,
       itcpugas,cpuinfo,
       aasmcpu;


    function getreferencestring(asminfo: pasminfo; var ref : treference) : string;
    var
      s : string;
    begin
       with ref do
        begin
          if ((offset < -2048) or (offset > 2047)) and
             (refaddr = addr_no) then
            internalerror(2006052501);
          case refaddr of
            addr_no:
              s := '';
            addr_pic_no_got:
              internalerror(2016060501);
            else
              begin
                s :='';
                s := s+'(';
                if assigned(symbol) then
                  begin
                    if asminfo^.dollarsign<>'$' then
                      begin
                        s:=s+ApplyAsmSymbolRestrictions(symbol.name);
                        if assigned(relsymbol) then
                          s:=s+'-'+ApplyAsmSymbolRestrictions(relsymbol.name)
                      end
                    else
                      begin
                        s:=s+symbol.name;
                        if assigned(relsymbol) then
                          s:=s+'-'+relsymbol.name;
                      end;
                  end;
              end;
          end;
          if offset<0 then
           s:=s+tostr(offset)
          else
           if (offset>0) then
            begin
              if assigned(symbol) then
                s:=s+'+'+tostr(offset)
              else
                s:=s+tostr(offset);
            end;

           if not(refaddr in [addr_no,addr_pic_no_got]) then
             begin
               s := s+')';
             end;
{$ifdef cpu64bitaddr}
           if (refaddr=addr_pic) then
             s := s + '@got';
{$endif cpu64bitaddr}

           if (index=NR_NO) then
             begin
                if offset=0 then
                  begin
                    if not (assigned(symbol)) then
                      s:=s+'0';
                  end;
                if (base<>NR_NO) then
                  s:=s+'('+gas_regname(base)+')'
                else if not assigned(symbol) then
                  s:=s+'(0)';
             end
           else if (index<>NR_NO) and (base<>NR_NO) then
             begin
               if (offset=0) then
                 s:=s+gas_regname(base)+','+gas_regname(index)
               else
                 internalerror(2006052502);
             end;

           case refaddr of
             addr_lo12: s:='%lo'+s;
             addr_hi20: s:='%hi'+s;
             addr_pcrel_lo12: s:='%pcrel_lo'+s;
             addr_pcrel_hi20: s:='%pcrel_hi'+s;
             else
               ;
           end;
        end;
      getreferencestring:=s;
    end;


    function getopstr(asminfo: pasminfo; const o:toper) : string;
    var
      hs : string;
    begin
      case o.typ of
        top_reg:
          getopstr:=gas_regname(o.reg);
        top_const:
          getopstr:=tostr(longint(o.val));
        top_ref:
          if o.ref^.refaddr=addr_full then
            begin
              hs:=o.ref^.symbol.name;
              if asminfo^.dollarsign<>'$' then
                hs:=ApplyAsmSymbolRestrictions(hs);
              if o.ref^.offset>0 then
               hs:=hs+'+'+tostr(o.ref^.offset)
              else
               if o.ref^.offset<0 then
                hs:=hs+tostr(o.ref^.offset);
              getopstr:=hs;
            end
          else
            getopstr:=getreferencestring(asminfo,o.ref^);
        top_fenceflags:
          begin
            getopstr:='';
            if ffI in o.fenceflags then getopstr:=getopstr+'i';
            if ffO in o.fenceflags then getopstr:=getopstr+'o';
            if ffR in o.fenceflags then getopstr:=getopstr+'r';
            if ffW in o.fenceflags then getopstr:=getopstr+'w';
          end;
        top_roundingmode:
          getopstr:=roundingmode2str[o.roundingmode];
        else
          internalerror(2002070604);
      end;
    end;


    Procedure TRVInstrWriter.WriteInstruction(hp : tai);
    var op: TAsmOp;
        s: string;
        i: byte;
        sep: string[3];
    begin
      s:=#9+gas_op2str[taicpu(hp).opcode];

      if taicpu(hp).condition<>C_None then
        s:=s+cond2str[taicpu(hp).condition];

      if taicpu(hp).memoryordering<>[] then
        begin
          s:=s+'.';
          if moAq in taicpu(hp).memoryordering then s:=s+'aq';
          if moRl in taicpu(hp).memoryordering then s:=s+'rl';
        end;

      if taicpu(hp).ops<>0 then
        begin
          sep:=#9;
          for i:=0 to taicpu(hp).ops-1 do
            begin
               s:=s+sep+getopstr(owner.asminfo,taicpu(hp).oper[i]^);
               sep:=',';
            end;
        end;

      owner.writer.AsmWriteLn(s);
    end;


{****************************************************************************}
{                         GNU RiscV Assembler writer                           }
{****************************************************************************}

    constructor TRVGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TRVInstrWriter.create(self);
      end;


    function TRVGNUAssembler.MakeCmdLine: TCmdStr;
      const
        arch_str: array[boolean,tcputype] of string[10] = (
{$ifdef RISCV32}
          ('','rv32imac','rv32ima','rv32im','rv32i'),
          ('','rv32imafdc','rv32imafd','rv32imfd','rv32ifd')
{$endif RISCV32}
{$ifdef RISCV64}
          ('','rv64imac','rv64ima','rv64im','rv64i'),
          ('','rv64imafdc','rv64imafd','rv64imfd','rv64ifd')
{$endif RISCV64}
        );
      begin
        result := inherited MakeCmdLine;
        Replace(result,'$ARCH',arch_str[current_settings.fputype=fpu_fd,current_settings.cputype]);
{$ifdef RISCV32}
        Replace(result,'$ABI','ilp32');
{$endif RISCV32}
{$ifdef RISCV64}
        if target_info.abi=abi_riscv_hf then
          Replace(result,'$ABI','lp64d')
        else
          Replace(result,'$ABI','lp64');
{$endif RISCV64}
      end;


  const
    as_riscv_gas_info : tasminfo =
       (
         id     : as_gas;

         idtxt  : 'AS';
         asmbin : 'as';
         asmcmd : '-o $OBJ $EXTRAOPT -march=$ARCH -mabi=$ABI $ASM';
         supported_targets : [system_riscv32_linux,system_riscv64_linux];
         flags : [af_needar,af_smartlink_sections];
         labelprefix : '.L';
         labelmaxlen : -1;
         comment : '# ';
         dollarsign: '$';
       );

begin
  RegisterAssembler(as_riscv_gas_info,TRVGNUAssembler);
end.
