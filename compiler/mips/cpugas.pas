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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      cpubase, aasmbase, globtype, systems,
      aasmtai, aasmcpu, assemble, aggas;

    type
      TMIPSGNUAssembler = class(TGNUassembler)
        nomacro, noreorder, noat : boolean;
        constructor create(info: pasminfo; smart: boolean); override;
        {# Constructs the command line for calling the assembler }
        function MakeCmdLine: TCmdStr; override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      cpubase,
       aasmtai, aasmcpu, assemble, aggas;

    type
      TMIPSGNUAssembler = class(TGNUassembler)
        constructor create(smart: boolean); override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;

      TMIPSInstrWriter = class(TCPUInstrWriter)
        procedure WriteInstruction(hp : tai);override;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  implementation

    uses
      cutils, systems,
      verbose, itcpugas, cgbase, cgutils;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{****************************************************************************}
{                         GNU MIPS  Assembler writer                           }
{****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    constructor TMIPSGNUAssembler.create(info: pasminfo; smart: boolean);
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    constructor TMIPSGNUAssembler.create(smart: boolean);
      begin
        inherited create(smart);
        InstrWriter := TMIPSInstrWriter.create(self);
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

    function GetReferenceString(var ref: TReference): string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin
        GetReferenceString := '';
        with ref do
        begin
          if (base = NR_NO) and (index = NR_NO) then
          begin
            if assigned(symbol) then
              GetReferenceString := symbol.Name;
            if offset > 0 then
              GetReferenceString := GetReferenceString + '+' + ToStr(offset)
            else if offset < 0 then
              GetReferenceString := GetReferenceString + ToStr(offset);
            case refaddr of
              addr_high:
                GetReferenceString := '%hi(' + GetReferenceString + ')';
              addr_low:
                GetReferenceString := '%lo(' + GetReferenceString + ')';
            end;
          end
          else
          begin
      {$ifdef extdebug}
            if assigned(symbol) and
              not(refaddr in [addr_pic,addr_lo]) then
              internalerror(2003052601);
      {$endif extdebug}
            if base <> NR_NO then
              GetReferenceString := GetReferenceString + '(' + gas_regname(base) + ')';
            if index = NR_NO then
            begin
              if offset <> 0 then
                GetReferenceString := ToStr(offset) + GetReferenceString;
              if assigned(symbol) then
              begin
                if refaddr = addr_low then
                  GetReferenceString := '%lo(' + symbol.Name + ')' + GetReferenceString
                else
                  GetReferenceString := symbol.Name + {'+' +} GetReferenceString;
              end;
            end
            else
            begin
  {$ifdef extdebug}
              if (Offset<>0) or assigned(symbol) then
                internalerror(2003052603);
  {$endif extdebug}
              GetReferenceString := GetReferenceString + '(' + gas_regname(index) + ')';

            end;
          end;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;


    function getopstr(const Oper: TOper): string;
      begin
        with Oper do
          case typ of
            top_reg:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              getopstr := asm_regname(reg);
            top_const:
              getopstr := tostr(longint(val));
            top_ref:
              getopstr := getreferencestring(ref^);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              getopstr := gas_regname(reg);
            top_const:
              getopstr := tostr(longint(val));
            top_ref:
              if (oper.ref^.refaddr in [addr_no, addr_pic]) or ((oper.ref^.refaddr = addr_low) and ((oper.ref^.base <> NR_NO) or
                (oper.ref^.index <> NR_NO))) then
                getopstr := getreferencestring(ref^)
              else
                getopstr := getreferencestring(ref^);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            else
              internalerror(10001);
          end;
      end;

      function getopstr_4(const Oper: TOper): string;
      var
        tmpref: treference;
      begin
        with Oper do
          case typ of
            top_ref:
            begin
              tmpref := ref^;
              Inc(tmpref.offset, 4);
              getopstr_4 := getreferencestring(tmpref);
            end;
            else
              internalerror(2007050403);
          end;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
     function getnextfpreg(tmpfpu : shortstring) : shortstring;
     begin
       case length(tmpfpu) of
       3:
        if (tmpfpu[3] = '9') then
          tmpfpu:='$f10'
        else
          tmpfpu[3] := succ(tmpfpu[3]);
       4:
        if (tmpfpu[4] = '9') then
          tmpfpu:='$f20'
        else
          tmpfpu[4] := succ(tmpfpu[4]);
        else
          internalerror(20120531);
        end;
        getnextfpreg := tmpfpu;
     end;
}

    function is_macro_instruction(ai : taicpu) : boolean;
      var
        op: tasmop;
      begin
        op:=ai.opcode;
        is_macro_instruction :=
        { 'seq', 'sge', 'sgeu', 'sgt', 'sgtu', 'sle', 'sleu', 'sne', }
          (op=A_SEQ) or (op = A_SGE) or (op=A_SGEU) or (op=A_SGT) or
          (op=A_SGTU) or (op=A_SLE) or (op=A_SLEU) or (op=A_SNE)
          { JAL is not here! See comments in TCGMIPS.a_call_name. }
          or (op=A_LA) or ((op=A_BC) and
            not (ai.condition in [C_EQ,C_NE,C_GTZ,C_GEZ,C_LTZ,C_LEZ,C_COP1TRUE,C_COP1FALSE])) {or (op=A_JAL)}
          or (op=A_REM) or (op=A_REMU)
          { DIV and DIVU are normally macros, but use $zero as first arg to generate a CPU instruction. }
          or (((op=A_DIV) or (op=A_DIVU)) and
            ((ai.ops<>3) or (ai.oper[0]^.typ<>top_reg) or (ai.oper[0]^.reg<>NR_R0)))
          or (op=A_MULO) or (op=A_MULOU)
          { A_LI is only a macro if the immediate is not in thez 16-bit range }
          or (op=A_LI);
      end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    procedure TMIPSInstrWriter.WriteInstruction(hp: Tai);
      var
        Op: TAsmOp;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        s:  string;
        i:  integer;
        tmpfpu: string;
        //tmpfpu_len: longint;
        r: TRegister;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        s,s1:  string;
        i:  integer;
        tmpfpu: string;
        tmpfpu_len: longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin
        if hp.typ <> ait_instruction then
          exit;
        op := taicpu(hp).opcode;

        case op of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
          A_LDC1:
            begin
              if (target_info.endian = endian_big) then
                begin
                  s := #9 + gas_op2str[A_LDC1] + #9 + getopstr(taicpu(hp).oper[0]^)
                       + ',' + getopstr(taicpu(hp).oper[1]^);
                end
              else
                begin
                  tmpfpu := getopstr(taicpu(hp).oper[0]^);
                  s := #9 + gas_op2str[A_LWC1] + #9 + tmpfpu + ',' + getopstr(taicpu(hp).oper[1]^); // + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
                  owner.writer.AsmWriteLn(s);

{ bug if $f9/$f19
              tmpfpu_len := length(tmpfpu);
              tmpfpu[tmpfpu_len] := succ(tmpfpu[tmpfpu_len]);

}
                  r := taicpu(hp).oper[0]^.reg;
                  setsupreg(r, getsupreg(r) + 1);
                  tmpfpu := asm_regname(r);
                  s := #9 + gas_op2str[A_LWC1] + #9 + tmpfpu + ',' + getopstr_4(taicpu(hp).oper[1]^); // + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
                end;
              owner.writer.AsmWriteLn(s);
            end;
          A_SDC1:
            begin
              if (target_info.endian = endian_big) then
                begin
                  s := #9 + gas_op2str[A_SDC1] + #9 + getopstr(taicpu(hp).oper[0]^)
                       + ',' + getopstr(taicpu(hp).oper[1]^);
                end
              else
                begin
                  tmpfpu := getopstr(taicpu(hp).oper[0]^);
                  s := #9 + gas_op2str[A_SWC1] + #9 + tmpfpu + ',' + getopstr(taicpu(hp).oper[1]^); //+ ',' + getopstr(taicpu(hp).oper[2]^) + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
                  owner.writer.AsmWriteLn(s);

{
              tmpfpu_len := length(tmpfpu);
              tmpfpu[tmpfpu_len] := succ(tmpfpu[tmpfpu_len]);
}
                  r := taicpu(hp).oper[0]^.reg;
                  setsupreg(r, getsupreg(r) + 1);
                  tmpfpu := asm_regname(r);
                  s := #9 + gas_op2str[A_SWC1] + #9 + tmpfpu + ',' + getopstr_4(taicpu(hp).oper[1]^); //+ ',' + getopstr(taicpu(hp).oper[2]^) + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
                end;
              owner.writer.AsmWriteLn(s);
            end;
          else
            begin
              if is_macro_instruction(taicpu(hp)) and TMIPSGNUAssembler(owner).nomacro then
                owner.writer.AsmWriteln(#9'.set'#9'macro');
              s := #9 + gas_op2str[op] + cond2str[taicpu(hp).condition];
              if taicpu(hp).delayslot_annulled then
                s := s + ',a';
              if taicpu(hp).ops > 0 then
              begin
                s := s + #9 + getopstr(taicpu(hp).oper[0]^);
                for i := 1 to taicpu(hp).ops - 1 do
                  s := s + ',' + getopstr(taicpu(hp).oper[i]^);
              end;
              owner.writer.AsmWriteLn(s);
              if is_macro_instruction(taicpu(hp)) and TMIPSGNUAssembler(owner).nomacro then
                owner.writer.AsmWriteln(#9'.set'#9'nomacro');
            end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          A_P_STK2:
          begin
            s1 := getopstr(taicpu(hp).oper[2]^);
            STK2_LocalSize := align(STK2_LocalSize, 8);
            if s1[1] = '-' then
              str(-STK2_LocalSize, s1)
            else
              str(STK2_LocalSize, s1);
            s := #9 + gas_op2str[A_ADDIU] + #9 + getopstr(taicpu(hp).oper[0]^)+ ',' + getopstr(taicpu(hp).oper[1]^) + ',' + s1;
            owner.AsmWriteLn(s);
          end;
          A_P_FRAME:
          begin
          end;
          A_P_SET_MACRO:
          begin
            s := #9 + '.set' + #9 + 'macro';
            owner.AsmWriteLn(s);
          end;
          A_P_SET_REORDER:
          begin
            s := #9 + '.set' + #9 + 'reorder';
            owner.AsmWriteLn(s);
          end;
          A_P_SET_NOMACRO:
          begin
            s := #9 + '.set' + #9 + 'nomacro';
            owner.AsmWriteLn(s);
          end;
          A_P_SET_NOREORDER:
          begin
            s := #9 + '.set' + #9 + 'noreorder';
            owner.AsmWriteLn(s);
          end;
          A_P_SW:
          begin
            s := #9 + gas_op2str[A_SW] + #9 + getopstr(taicpu(hp).oper[0]^)+ ',' + getopstr(taicpu(hp).oper[2]^) + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
            owner.AsmWriteLn(s);
          end;
          A_P_LW:
          begin
            s := #9 + gas_op2str[A_LW] + #9 + getopstr(taicpu(hp).oper[0]^)+ ',' + getopstr(taicpu(hp).oper[2]^) + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
            owner.AsmWriteLn(s);
          end;
          A_LDC1:
          begin
            tmpfpu := getopstr(taicpu(hp).oper[0]^);
            s := #9 + gas_op2str[A_LWC1] + #9 + tmpfpu + ',' + getopstr(taicpu(hp).oper[1]^); // + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
            owner.AsmWriteLn(s);

            tmpfpu_len := length(tmpfpu);
            tmpfpu[tmpfpu_len] := succ(tmpfpu[tmpfpu_len]);
            s := #9 + gas_op2str[A_LWC1] + #9 + tmpfpu + ',' + getopstr_4(taicpu(hp).oper[1]^); // + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
            owner.AsmWriteLn(s);
          end;
          A_SDC1:
          begin
            tmpfpu := getopstr(taicpu(hp).oper[0]^);
            s := #9 + gas_op2str[A_SWC1] + #9 + tmpfpu + ',' + getopstr(taicpu(hp).oper[1]^); //+ ',' + getopstr(taicpu(hp).oper[2]^) + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
            owner.AsmWriteLn(s);

            tmpfpu_len := length(tmpfpu);
            tmpfpu[tmpfpu_len] := succ(tmpfpu[tmpfpu_len]);
            s := #9 + gas_op2str[A_SWC1] + #9 + tmpfpu + ',' + getopstr_4(taicpu(hp).oper[1]^); //+ ',' + getopstr(taicpu(hp).oper[2]^) + '(' + getopstr(taicpu(hp).oper[1]^) + ')';
            owner.AsmWriteLn(s);
          end;
          else
          begin
            s := #9 + gas_op2str[op] + cond2str[taicpu(hp).condition];
            if taicpu(hp).delayslot_annulled then
              s := s + ',a';
            if taicpu(hp).ops > 0 then
            begin
              s := s + #9 + getopstr(taicpu(hp).oper[0]^);
              for i := 1 to taicpu(hp).ops - 1 do
                s := s + ',' + getopstr(taicpu(hp).oper[i]^);
            end;
            owner.AsmWriteLn(s);
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        end;
      end;


    const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef MIPSEL}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      as_MIPSEL_as_info: tasminfo =
        (
        id: as_gas;
        idtxt: 'AS';
        asmbin: 'as';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        asmcmd: '-mips2 -W -EL -o $OBJ $ASM';
        supported_targets: [system_mips_linux,system_mipsel_linux];
        flags: [af_allowdirect, af_needar, af_smartlink_sections];
        labelprefix: '.L';
        comment: '# ';
        );

begin
  RegisterAssembler(as_MIPSEL_as_info, TMIPSGNUAssembler);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end.
