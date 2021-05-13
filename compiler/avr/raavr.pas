{
    Copyright (c) 1998-2003 by Carl Eric Codere and Peter Vreman

    Handles the common arm assembler reader routines

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
unit raavr;

{$i fpcdefs.inc}

  interface

    uses
      cpubase,
      aasmtai,aasmdata,
      rautils,
      globtype;

    type
       TAVROperand=class(TOperand)
      end;

      TAVRInstruction=class(TInstruction)
        function ConcatInstruction(p:TAsmList) : tai;override;
      end;

      TRegType = (rt_all, rt_even, rt_16_31, rt_even_24_30, rt_16_23, rt_XYZ, rt_YZ, rt_X, rt_Y, rt_Z);

      TAVROpConstraint = record
        case typ : toptype of
          top_none   : ();
          top_reg    : (rt: TRegType; am: set of taddressmode; minconst, maxconst: tcgint);
          top_ref,
          top_const  : (max, min: tcgint);
      end;

      // Constraints of operands per opcode
      // Encode variable number of operands by bit position, e.g.
      // 2 operands = 1 shl 2 = 4
      // 1 or 2 operands = 1 shl 1 + 1 shl 2 = 6
      TAVRInstrConstraint = record
        numOperands: byte;
        Operands: array[0..max_operands-1] of TAVROpConstraint;
      end;

{$PUSH}
{$WARN 3177 off : Some fields coming after "$1" were not initialized}
  const
    AVRInstrConstraint: array[TAsmOp] of TAVRInstrConstraint =
      // A_NONE
      ((numOperands: 0; Operands: ((typ: top_none), (typ: top_none))),
      // A_ADD
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_ADC
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_ADIW
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_even_24_30), (typ: top_const; max: 63; min: 0))),
       // A_SUB
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_SUBI
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_const; max: 255; min: -128))),
       // A_SBC
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_SBCI
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_const; max: 255; min: -128))),
       // A_SBRC
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_const; max: 7; min: 0))),
       // A_SBRS
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_const; max: 7; min: 0))),
       // A_SBIW
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_even_24_30), (typ: top_const; max: 63; min: 0))),
       // A_AND
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_ANDI
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_const; max: 255; min: -128))),
       // A_OR
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_ORI
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_const; max: 255; min: -128))),
       // A_EOR
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_COM
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_NEG
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_SBR
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_const; max: 255; min: -128))),
       // A_CBR
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_const; max: 255; min: -128))),
       // A_INC
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_DEC
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_TST
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_MUL
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_MULS
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_reg; rt: rt_16_31))),
       // A_MULSU
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_23), (typ: top_reg; rt: rt_16_23))),
       // A_FMUL
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_23), (typ: top_reg; rt: rt_16_23))),
       // A_FMULS
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_23), (typ: top_reg; rt: rt_16_23))),
       // A_FMULSU
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_23), (typ: top_reg; rt: rt_16_23))),
       // A_RJMP
       (numOperands: (1 shl 1); Operands: ((typ: top_const; max: 2047; min: -2048), (typ: top_reg; rt: rt_all))),
       // A_IJMP
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_EIJMP
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_JMP, max size depends on size op PC
       (numOperands: (1 shl 1); Operands: ((typ: top_const; max: (1 shl 22 - 1); min: 0), (typ: top_none))),
       // A_RCALL
       (numOperands: (1 shl 1); Operands: ((typ: top_const; max: 2047; min: -2048), (typ: top_none))),
       // A_ICALL
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_EICALL
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CALL, max size depends on size op PC
       (numOperands: (1 shl 1); Operands: ((typ: top_const; max: (1 shl 22 - 1); min: 0), (typ: top_none))),
       // A_RET
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_IRET
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CPSE
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_CP
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_CPC
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_CPI
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_const; max: 255; min: -128))),
       // A_SBIC
       (numOperands: (1 shl 2); Operands: ((typ: top_const; max: 31; min: 0), (typ: top_const; max: 7; min: 0))),
       // A_SBIS
       (numOperands: (1 shl 2); Operands: ((typ: top_const; max: 31; min: 0), (typ: top_const; max: 7; min: 0))),
       // A_BRxx
       (numOperands: (1 shl 1); Operands: ((typ: top_const; max: 63; min: -64), (typ: top_none))),
       // A_MOV
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_all))),
       // A_MOVW
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_even), (typ: top_reg; rt: rt_even))),
       // A_LDI
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_const; max: 255; min: -128))),
       // A_LDS TODO: There are 2 versions with different machine codes and constant ranges. Could possibly distinguish based on size of PC?
       // Perhaps handle separately with a check on sub-architecture? Range check only important if smaller instruction code selected on larger arch
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_const; max: 65535; min: 0))),
       // A_LD
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_XYZ; am: [AM_UNCHANGED, AM_POSTINCREMENT, AM_PREDRECEMENT]))),
       // A_LDD
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_YZ; am: [AM_UNCHANGED]; minconst: 0; maxconst: 63))),
       // A_STS TODO: See LDS above
       (numOperands: (1 shl 2); Operands: ((typ: top_const; max: 65535; min: 0), (typ: top_reg; rt: rt_all))),
       // A_ST
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_XYZ; am: [AM_UNCHANGED, AM_POSTINCREMENT, AM_PREDRECEMENT]), (typ: top_reg; rt: rt_all))),
       // A_STD
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_YZ; am: [AM_UNCHANGED]; minconst: 0; maxconst: 63), (typ: top_reg; rt: rt_all))),
       // A_LPM
       (numOperands: (1 shl 0 + 1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_Z; am: [AM_UNCHANGED, AM_POSTINCREMENT]))),
       // A_ELPM
       (numOperands: (1 shl 0 + 1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_reg; rt: rt_Z; am: [AM_UNCHANGED, AM_POSTINCREMENT]))),
       // A_SPM
       (numOperands: (1 shl 0 + 1 shl 1); Operands: ((typ: top_reg; rt: rt_Z; am: [AM_UNCHANGED, AM_POSTINCREMENT]), (typ: top_none))),
       // A_IN
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_const; max: 63; min: 0))),
       // A_OUT
       (numOperands: (1 shl 2); Operands: ((typ: top_const; max: 63; min: 0), (typ: top_reg; rt: rt_all))),
       // A_PUSH
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_POP
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_LSL
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_LSR
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_ROL
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_ROR
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_ASR
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_SWAP
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_BSET
       (numOperands: (1 shl 1); Operands: ((typ: top_const; max: 7; min: 0), (typ: top_none))),
       // A_BCLR
       (numOperands: (1 shl 1); Operands: ((typ: top_const; max: 7; min: 0), (typ: top_none))),
       // A_SBI
       (numOperands: (1 shl 2); Operands: ((typ: top_const; max: 32; min: 0), (typ: top_const; max: 7; min: 0))),
       // A_CBI
       (numOperands: (1 shl 2); Operands: ((typ: top_const; max: 32; min: 0), (typ: top_const; max: 7; min: 0))),
       // A_SEC
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_SEH
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_SEI
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_SEN
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_SER
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_16_31), (typ: top_none))),
       // A_SES
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_SET
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_SEV
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_SEZ
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CLC
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CLH
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CLI
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CLN
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CLR
       (numOperands: (1 shl 1); Operands: ((typ: top_reg; rt: rt_all), (typ: top_none))),
       // A_CLS
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CLT
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CLV
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_CLZ
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_BST
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_const; max: 7; min: 0))),
       // A_BLD
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_all), (typ: top_const; max: 7; min: 0))),
       // A_BREAK
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_NOP
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_SLEEP
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_WDR
       (numOperands: (1 shl 0); Operands: ((typ: top_none), (typ: top_none))),
       // A_XCH
       (numOperands: (1 shl 2); Operands: ((typ: top_reg; rt: rt_Z), (typ: top_reg; rt: rt_all))),
       // A_DES
       (numOperands: (1 shl 1); Operands: ((typ: top_const; max: 15; min: 0), (typ: top_none)))
       );
{$POP}


  implementation
    uses
      aasmcpu, verbose, cgbase, itcpugas;

    function TAVRInstruction.ConcatInstruction(p:TAsmList) : tai;
      var
        i: integer;
        err, opregasref: boolean;
        s1, s2: string;
        reg: tregister;
      begin
        result:=inherited ConcatInstruction(p);

        if Result.typ = ait_instruction then
          begin
            // check if number of operands fall inside the allowed set
            if ((1 shl taicpu(Result).ops) and AVRInstrConstraint[opcode].numOperands = 0)  then
              Message(asmr_e_invalid_opcode_and_operand)
            else
              begin
                for i := 0 to taicpu(Result).ops-1 do
                  begin
                    err := true;
                    opregasref := false;
                    // Type check
                    if (taicpu(Result).oper[i]^.typ = AVRInstrConstraint[opcode].Operands[i].typ) then
                      err := false

                    // if label was specified check if const is expected
                    else if (AVRInstrConstraint[opcode].Operands[i].typ = top_const) and (taicpu(Result).oper[i]^.typ = top_ref) then
                      err := false

                    // Also expressions such as X+, -Z and X+8 are classified as TOP_REF
                    else if (AVRInstrConstraint[opcode].Operands[i].typ = top_reg) and (taicpu(Result).oper[i]^.typ = top_ref) and
                             (int64(taicpu(Result).oper[i]^.ref^.base) and $01030000 > 0) then
                      begin
                        err := false;
                        opregasref := true;
                        reg := taicpu(Result).oper[i]^.ref^.base;
                      end
                    // LDD r1, a or STD a, r1 where a is local variable => type is set to top_local
                    // also mov r10, TypeCastToByte(procParam) returns top_local for typecasted param
                    // but no further information is available at this point to check
                    else if (AVRInstrConstraint[opcode].Operands[i].typ = top_reg) and (taicpu(Result).oper[i]^.typ = top_local) and
                            (opcode in [A_LDD, A_STD, A_MOV]) then
                      begin
                        err := false;
                      end;

                    if err then
                      begin
                        WriteStr(s1, taicpu(Result).oper[i]^.typ);
                        WriteStr(s2, AVRInstrConstraint[opcode].Operands[i].typ);
                        Message2(type_e_incompatible_types, s1, s2);
                      end
                    else if (taicpu(Result).oper[i]^.typ = top_reg) or opregasref then
                      begin
                        err := false;
                        if not opregasref then
                          reg := taicpu(Result).oper[i]^.reg;

                        case AVRInstrConstraint[opcode].Operands[i].rt of
                          rt_all: err := int64(reg) > $1030000;  // excluding pointer registers X, Y, Z
                          rt_even:    err := int64(reg) mod 2 = 1;
                          rt_even_24_30:  err := not(word(reg) in [24, 26, 28, 30]);
                          rt_16_31: err := not(word(reg) in [16..31]);
                          rt_16_23: err := not(word(reg) in [16..23]);
                          rt_Z: err := (int64(reg) <> $0103001e);
                          rt_YZ: err := not((int64(reg) = $0103001c) or
                                         (int64(reg) =$0103001e));
                          rt_XYZ: err := not((int64(reg) = $0103001a) or
                                         (int64(reg) = $0103001c) or
                                         (int64(reg) = $0103001e));
                        end;

                        // Catch erroneous z if it should be z+k or something similar
                        // By checking if .am is not empty (implying a reference is expected)

                        // ldd R20, z should not pass  => opregasref = true
                        // but ldd R20, z+1 should pass => opregasref = false
                        if not (err) and (AVRInstrConstraint[opcode].Operands[i].am = [AM_UNCHANGED]) then
                          err := not opregasref;

                        if not (err) and not(AM_UNCHANGED in AVRInstrConstraint[opcode].Operands[i].am) and
                          ((AM_POSTINCREMENT in AVRInstrConstraint[opcode].Operands[i].am) or
                           (AM_PREDRECEMENT in AVRInstrConstraint[opcode].Operands[i].am)) then
                          err := not opregasref;

                        if not(err) and opregasref then
                          begin
                            if (taicpu(Result).oper[i]^.ref^.addressmode = AM_UNCHANGED) and
                                  (AM_UNCHANGED in AVRInstrConstraint[opcode].Operands[i].am) then
                               err := ((taicpu(Result).oper[i]^.ref^.offset > AVRInstrConstraint[opcode].Operands[i].maxconst) or
                                      (taicpu(Result).oper[i]^.ref^.offset < AVRInstrConstraint[opcode].Operands[i].minconst))

                            else if (taicpu(Result).oper[i]^.ref^.addressmode in AVRInstrConstraint[opcode].Operands[i].am) then
                              err := false

                            else
                              err := true;
                          end;

                          if err then
                            Message1(asmr_e_invalid_ref_register, gas_regname(reg));
                        end
                      else if taicpu(Result).oper[i]^.typ = top_const then
                        begin
                          // Need to check if original value was signed or simply sign overflowed in 16 bit?
                          if ((taicpu(Result).oper[i]^.val > AVRInstrConstraint[opcode].Operands[i].max) or
                              (taicpu(Result).oper[i]^.val < AVRInstrConstraint[opcode].Operands[i].min)) then
                            Message(asmr_e_constant_out_of_bounds);
                        end;
                  end;
              end;
          end;
      end;

end.
