{
    Copyright (c) 2003 by Florian Klaempfl

    This unit implements an asm for the Xtensa

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
{ This unit implements the GNU Assembler writer for the Xtensa
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
      TXtensaInstrWriter=class(TCPUInstrWriter)
        procedure WriteInstruction(hp : tai);override;
      end;

      TXtensaGNUAssembler=class(TGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
        function MakeCmdLine: TCmdStr; override;
      end;

  implementation

    uses
       cutils,globals,verbose,
       aasmcpu,
       itcpugas,
       cgbase,cgutils;

{****************************************************************************}
{                         GNU Xtensa Assembler writer                           }
{****************************************************************************}

    constructor TXtensaGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TXtensaInstrWriter.create(self);
      end;


    function TXtensaGNUAssembler.MakeCmdLine: TCmdStr;
      begin
        result:=inherited MakeCmdLine;
      end;

{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

    function getreferencestring(var ref : treference) : string;
      var
        s : string;
      begin
         with ref do
          begin
            if assigned(symbol) then
              begin
                s:=symbol.name;
                if offset<>0 then
                  s:=s+tostr_with_plus(offset);
                case refaddr of
                  addr_pic:
                    s:=s+'(PLT)';
                  addr_full,
                  addr_no:
                    ;
                  else
                    Internalerror(2020112403);
                end;
              end
            else
              begin
                s:=gas_regname(base);
                if index<>NR_NO then
                  begin
                    s:=s+','+gas_regname(index);
                    if offset<>0 then
                      Internalerror(2020112402);
                  end
                else
                  s:=s+','+tostr(offset);
              end;
          end;
        getreferencestring:=s;
      end;


    function getopstr(const o:toper) : string;
      var
        hs : string;
        first : boolean;
        r, rs : tsuperregister;
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
                if o.ref^.offset>0 then
                 hs:=hs+'+'+tostr(o.ref^.offset)
                else
                 if o.ref^.offset<0 then
                  hs:=hs+tostr(o.ref^.offset);
                getopstr:=hs;
              end
            else
              getopstr:=getreferencestring(o.ref^);
          else
            internalerror(2020030705);
        end;
      end;


    Procedure TXtensaInstrWriter.WriteInstruction(hp : tai);
    var op: TAsmOp;
        s: string;
        i: byte;
        sep: string[3];
    begin
      op:=taicpu(hp).opcode;
      if taicpu(hp).opIsPrefixed then
        s:=#9'_'+gas_op2str[op]
      else
        s:=#9+gas_op2str[op];
      if taicpu(hp).condition<>C_None then
        s:=s+cond2str[taicpu(hp).condition];
      if taicpu(hp).oppostfix <> PF_None then
        s:=s+'.'+oppostfix2str[taicpu(hp).oppostfix];
      if taicpu(hp).ops<>0 then
        begin
          if length(s)<5 then
            sep:=#9#9
          else
            sep:=#9;
          for i:=0 to taicpu(hp).ops-1 do
            begin
               s:=s+sep+getopstr(taicpu(hp).oper[i]^);
               sep:=',';
            end;
        end;
      owner.writer.AsmWriteLn(s);
    end;


    const
       as_xtensa_gas_info : tasminfo =
          (
            id     : as_gas;

            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM --longcalls';
            supported_targets : [system_xtensa_embedded,system_xtensa_linux,system_xtensa_freertos];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );


begin
  RegisterAssembler(as_xtensa_gas_info,TXtensaGNUAssembler);
end.

