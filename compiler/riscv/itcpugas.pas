{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit contains the RiscV GAS instruction tables

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
unit itcpugas;

{$I fpcdefs.inc}

  interface

    uses
      cpubase, cgbase;

    const
      gas_op2str: array[tasmop] of string[14] = ('<none>',
        'nop','call',
        'lui','auipc','jal','jalr',
        'b','lb','lh','lw','lbu','lhu',
        'sb','sh','sw',
        'addi','slti','sltiu',
        'xori','ori','andi',
        'slli','srli','srai',
        'add','sub','sll','slt','sltu',
        'xor','srl','sra','or','and',
        'fence','fence.i',
        'ecall','ebreak',
        'csrrw','csrrs','csrrc','csrrwi','csrrsi','csrrci',
{$ifdef RISCV64}
        { 64-bit }
        'addiw','slliw','srliw','sraiw',
        'addw','sllw','srlw','subw','sraw',
        'ld','sd','lwu',
{$endif RISCV64}

        { m-extension }
        'mul','mulh','mulhsu','mulhu',
        'div','divu','rem','remu',
{$ifdef RISCV64}
        { 64-bit }
        'mulw',
        'divw','divuw','remw','remuw',
{$endif RISCV64}

        { a-extension }
        'lr.w','sc.w','amoswap.w','amoadd.w','amoxor.w','amoand.w',
        'amoor.w','amomin.w','amomax.w','amominu.w','amomaxu.w',

{$ifdef RISCV64}
        { 64-bit }
        'lr.d','sc.d','amoswap.d','amoadd.d','amoxor.d','amoand.d',
        'amoor.d','amomin.d','amomax.d','amominu.d','amomaxu.d',
{$endif RISCV64}

        { f-extension }
        'flw','fsw',
        'fmadd.s','fmsub.s','fnmsub.s','fnmadd.s',
        'fadd.s','fsub.s','fmul.s','fdiv.s',
        'fsqrt.s','fsgnj.s','fsgnjn.s','fsgnjx.s',
        'fmin.s','fmax.s',
        'fmv.x.s','feq.s','flt.s','fle.s','fclass.s',
        'fcvt.w.s','fcvt.wu.s','fcvt.s.w','fcvt.s.wu',
        'fmv.s.x',
        'frcsr','frrm','frflags','fscsr','fsrm',
        'fsflags','fsrmi','fsflagsi',

{$ifdef RISCV64}
        { 64-bit }
        'fcvt.l.s','fcvt.lu.s',
        'fcvt.s.l','fcvt.s.lu',
{$endif RISCV64}

        { d-extension }
        'fld','fsd',
        'fmadd.d','fmsub.d','fnmsub.d','fnmadd.d',
        'fadd.d','fsub.d','fmul.d','fdiv.d',
        'fsqrt.d','fsgnj.d','fsgnjn.d','fsgnjx.d',
        'fmin.d','fmax.d',
        'feq.d','flt.d','fle.d','fclass.d',
        'fcvt.d.s','fcvt.s.d',
        'fcvt.w.d','fcvt.wu.d','fcvt.d.w','fcvt.d.wu',

{$ifdef RISCV64}
        { 64-bit }
        'fcvt.l.d','fcvt.lu.d','fmv.x.d',
        'fcvt.d.l','fcvt.d.lu','fmv.d.x',
{$endif RISCV64}

        { Machine mode }
        'mret','hret','sret','uret',
        'wfi',

        { Supervisor mode }
        'sfence.vm'
        );

    function gas_regnum_search(const s: string): Tregister;
    function gas_regname(r: Tregister): string;

  implementation

    uses
      globtype,globals,aasmbase,
      cutils,verbose, systems,
      rgbase;

    const
      gas_regname_table : TRegNameTable = (
        {$i rrv32std.inc}
      );

      gas_regname_index : array[tregisterindex] of tregisterindex = (
        {$i rrv32sri.inc}
      );


    function findreg_by_gasname(const s:string):tregisterindex;
      var
        i,p : tregisterindex;
      begin
        {Binary search.}
        p:=0;
        i:=regnumber_count_bsstart;
        repeat
          if (p+i<=high(tregisterindex)) and (gas_regname_table[gas_regname_index[p+i]]<=s) then
            p:=p+i;
          i:=i shr 1;
        until i=0;
        if gas_regname_table[gas_regname_index[p]]=s then
          findreg_by_gasname:=gas_regname_index[p]
        else
          findreg_by_gasname:=0;
      end;


    function gas_regnum_search(const s:string):Tregister;
      begin
        result:=regnumber_table[findreg_by_gasname(s)];
      end;


    function gas_regname(r:Tregister):string;
      var
        p : tregisterindex;
      begin
        p:=findreg_by_number(r);
        if p<>0 then
          result:=gas_regname_table[p]
        else
          result:=generic_regname(r);
      end;

end.

