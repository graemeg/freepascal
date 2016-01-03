{
    Copyright (c) 2003 by Florian Klaempfl

    This unit implements an asm for the ARM

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
{ This unit implements the GNU Assembler writer for the ARM
}

unit agavrgas;

{$i fpcdefs.inc}

  interface

    uses
       globtype,systems,
       aasmtai,aasmdata,
       aggas,
       cpubase;

    type

      { TAVRGNUAssembler }

      TAVRGNUAssembler=class(TGNUassembler)
        constructor create(info: pasminfo; smart: boolean); override;
       function MakeCmdLine: TCmdStr; override;
      end;

     TAVRInstrWriter=class(TCPUInstrWriter)
        procedure WriteInstruction(hp : tai);override;
     end;


  implementation

    uses
       cutils,globals,verbose,
       assemble,
       aasmbase,aasmcpu,
       itcpugas,
       cpuinfo,
       cgbase,cgutils;

{****************************************************************************}
{                         GNU Arm Assembler writer                           }
{****************************************************************************}

    constructor TAVRGNUAssembler.create(info: pasminfo; smart: boolean);
      begin
        inherited;
        InstrWriter := TAVRInstrWriter.create(self);
      end;


{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

<<<<<<< HEAD
=======
    function getreferencestring(var ref : treference) : string;
      var
        s : string;
      begin
         with ref do
          begin
{$ifdef extdebug}
            // if base=NR_NO then
            //   internalerror(200308292);

            // if ((index<>NR_NO) or (shiftmode<>SM_None)) and ((offset<>0) or (symbol<>nil)) then
            //   internalerror(200308293);
{$endif extdebug}

            if assigned(symbol) then
              begin
                s:=symbol.name;
                if offset<0 then
                  s:=s+tostr(offset)
                else if offset>0 then
                  s:=s+'+'+tostr(offset);
              end
            else
              begin
                s:=gas_regname(base);
              end;
>>>>>>> graemeg/cpstrnew

    Procedure TAVRInstrWriter.WriteInstruction(hp : tai);

      function getreferencestring(var ref : treference) : string;
        var
          s : string;
        begin
           s:='';
           with ref do
            begin
  {$ifdef extdebug}
              // if base=NR_NO then
              //   internalerror(200308292);

              // if ((index<>NR_NO) or (shiftmode<>SM_None)) and ((offset<>0) or (symbol<>nil)) then
              //   internalerror(200308293);
  {$endif extdebug}
              if index<>NR_NO then
                internalerror(2011021701)
              else if base<>NR_NO then
                begin
                  if addressmode=AM_PREDRECEMENT then
                    s:='-';

                  case base of
                    NR_R26:
                      s:=s+'X';
                    NR_R28:
                      s:=s+'Y';
                    NR_R30:
                      s:=s+'Z';
                    else
                      s:=gas_regname(base);
                  end;
                  if addressmode=AM_POSTINCREMENT then
                    s:=s+'+';

                  if offset>0 then
                    s:=s+'+'+tostr(offset)
                  else if offset<0 then
                    s:=s+tostr(offset)
                end
              else if assigned(symbol) or (offset<>0) then
                begin
                  if assigned(symbol) then
                    s:=ReplaceForbiddenAsmSymbolChars(symbol.name);

                  if offset<0 then
                    s:=s+tostr(offset)
                  else if offset>0 then
                    s:=s+'+'+tostr(offset);
                  case refaddr of
                    addr_hi8:
                      s:='hi8('+s+')';
                    addr_hi8_gs:
                      s:='hi8(gs('+s+'))';
                    addr_lo8:
                      s:='lo8('+s+')';
                    addr_lo8_gs:
                      s:='lo8(gs('+s+'))';
                    else
                      s:='('+s+')';
                  end;
                end;
            end;
          getreferencestring:=s;
        end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

      function getopstr(const o:toper) : string;
        var
          hs : string;
          first : boolean;
          r : tsuperregister;
        begin
          case o.typ of
            top_reg:
              getopstr:=gas_regname(o.reg);
            top_const:
              getopstr:=tostr(longint(o.val));
            top_ref:
              if o.ref^.refaddr=addr_full then
                begin
                  hs:=ReplaceForbiddenAsmSymbolChars(o.ref^.symbol.name);
                  if o.ref^.offset>0 then
                   hs:=hs+'+'+tostr(o.ref^.offset)
                  else
                   if o.ref^.offset<0 then
                    hs:=hs+tostr(o.ref^.offset);
                  getopstr:=hs;
                end
              else
                getopstr:=getreferencestring(o.ref^);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function getopstr(const o:toper) : string;
      var
        hs : string;
        first : boolean;
        r : tsuperregister;
      begin
        case o.typ of
          top_reg:
            getopstr:=gas_regname(o.reg);
          top_const:
            getopstr:='#'+tostr(longint(o.val));
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
>>>>>>> graemeg/cpstrnew
            else
              internalerror(2002070604);
          end;
        end;

    var op: TAsmOp;
        s: string;
        i: byte;
        sep: string[3];
    begin
      op:=taicpu(hp).opcode;
      s:=#9+gas_op2str[op]+cond2str[taicpu(hp).condition];
      if taicpu(hp).ops<>0 then
        begin
          sep:=#9;
          for i:=0 to taicpu(hp).ops-1 do
            begin
              s:=s+sep+getopstr(taicpu(hp).oper[i]^);
              sep:=',';
            end;
        end;
      owner.writer.AsmWriteLn(s);
    end;


    function TAVRGNUAssembler.MakeCmdLine: TCmdStr;
      begin
        result := '-mmcu='+lower(cputypestr[current_settings.cputype])+' '+inherited MakeCmdLine;
      end;


    const
       as_avr_gas_info : tasminfo =
          (
            id     : as_gas;

            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_avr_embedded];
            flags : [af_needar,af_smartlink_sections];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: 's';
          );


begin
  RegisterAssembler(as_avr_gas_info,TAVRGNUAssembler);
end.
