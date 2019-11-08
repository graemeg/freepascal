{
    Copyright (c) 1998-2002 by Florian Klaempfl and Jonas Maebe

    This unit contains the peephole optimizer.

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
unit aoptx86;

{$i fpcdefs.inc}

{$define DEBUG_AOPTCPU}

  interface

    uses
      globtype,
      cpubase,
      aasmtai,aasmcpu,
      cgbase,cgutils,
      aopt,aoptobj;

    type
      TX86AsmOptimizer = class(TAsmOptimizer)
        function RegLoadedWithNewValue(reg : tregister; hp : tai) : boolean; override;
        function InstructionLoadsFromReg(const reg : TRegister; const hp : tai) : boolean; override;
        function RegReadByInstruction(reg : TRegister; hp : tai) : boolean;
        function RegInInstruction(Reg: TRegister; p1: tai): Boolean;override;
        function GetNextInstructionUsingReg(Current: tai; out Next: tai; reg: TRegister): Boolean;
      protected
        { checks whether loading a new value in reg1 overwrites the entirety of reg2 }
        function Reg1WriteOverwritesReg2Entirely(reg1, reg2: tregister): boolean;
        { checks whether reading the value in reg1 depends on the value of reg2. This
          is very similar to SuperRegisterEquals, except it takes into account that
          R_SUBH and R_SUBL are independendent (e.g. reading from AL does not
          depend on the value in AH). }
        function Reg1ReadDependsOnReg2(reg1, reg2: tregister): boolean;

        procedure DebugMsg(const s : string; p : tai);inline;

        class function IsExitCode(p : tai) : boolean;
        class function isFoldableArithOp(hp1 : taicpu; reg : tregister) : boolean;
        procedure RemoveLastDeallocForFuncRes(p : tai);

        function DoSubAddOpt(var p : tai) : Boolean;

        function PrePeepholeOptSxx(var p : tai) : boolean;
        function PrePeepholeOptIMUL(var p : tai) : boolean;

        function OptPass1AND(var p : tai) : boolean;
        function OptPass1_V_MOVAP(var p : tai) : boolean;
        function OptPass1VOP(var p : tai) : boolean;
        function OptPass1MOV(var p : tai) : boolean;
        function OptPass1Movx(var p : tai) : boolean;
        function OptPass1MOVXX(var p : tai) : boolean;
        function OptPass1OP(var p : tai) : boolean;
        function OptPass1LEA(var p : tai) : boolean;
        function OptPass1Sub(var p : tai) : boolean;
        function OptPass1SHLSAL(var p : tai) : boolean;
        function OptPass1SETcc(var p: tai): boolean;
        function OptPass1FSTP(var p: tai): boolean;
        function OptPass1FLD(var p: tai): boolean;

        function OptPass2MOV(var p : tai) : boolean;
        function OptPass2Imul(var p : tai) : boolean;
        function OptPass2Jmp(var p : tai) : boolean;
        function OptPass2Jcc(var p : tai) : boolean;
        function OptPass2Lea(var p: tai): Boolean;

        function PostPeepholeOptMov(var p : tai) : Boolean;
{$ifdef x86_64} { These post-peephole optimisations only affect 64-bit registers. [Kit] }
        function PostPeepholeOptMovzx(var p : tai) : Boolean;
        function PostPeepholeOptXor(var p : tai) : Boolean;
{$endif}
        function PostPeepholeOptCmp(var p : tai) : Boolean;
        function PostPeepholeOptTestOr(var p : tai) : Boolean;
        function PostPeepholeOptCall(var p : tai) : Boolean;
        function PostPeepholeOptLea(var p : tai) : Boolean;

        procedure OptReferences;
      end;

    function MatchInstruction(const instr: tai; const op: TAsmOp; const opsize: topsizes): boolean;
    function MatchInstruction(const instr: tai; const op1,op2: TAsmOp; const opsize: topsizes): boolean;
    function MatchInstruction(const instr: tai; const op1,op2,op3: TAsmOp; const opsize: topsizes): boolean;
    function MatchInstruction(const instr: tai; const ops: array of TAsmOp; const opsize: topsizes): boolean;

    function MatchOperand(const oper: TOper; const reg: TRegister): boolean; inline;
    function MatchOperand(const oper: TOper; const a: tcgint): boolean; inline;
    function MatchOperand(const oper1: TOper; const oper2: TOper): boolean;

    function RefsEqual(const r1, r2: treference): boolean;

    function MatchReference(const ref : treference;base,index : TRegister) : Boolean;

    { returns true, if ref is a reference using only the registers passed as base and index
      and having an offset }
    function MatchReferenceWithOffset(const ref : treference;base,index : TRegister) : Boolean;

{$ifdef DEBUG_AOPTCPU}
  const
    SPeepholeOptimization: shortstring = 'Peephole Optimization: ';
{$else DEBUG_AOPTCPU}
  { Empty strings help the optimizer to remove string concatenations that won't
    ever appear to the user on release builds. [Kit] }
  const
    SPeepholeOptimization = '';
{$endif DEBUG_AOPTCPU}

  implementation

    uses
      cutils,verbose,
      globals,
      cpuinfo,
      procinfo,
      aasmbase,
      aoptutils,
      symconst,symsym,
      cgx86,
      itcpugas;

    function MatchInstruction(const instr: tai; const op: TAsmOp; const opsize: topsizes): boolean;
      begin
        result :=
          (instr.typ = ait_instruction) and
          (taicpu(instr).opcode = op) and
          ((opsize = []) or (taicpu(instr).opsize in opsize));
      end;


    function MatchInstruction(const instr: tai; const op1,op2: TAsmOp; const opsize: topsizes): boolean;
      begin
        result :=
          (instr.typ = ait_instruction) and
          ((taicpu(instr).opcode = op1) or
           (taicpu(instr).opcode = op2)
          ) and
          ((opsize = []) or (taicpu(instr).opsize in opsize));
      end;


    function MatchInstruction(const instr: tai; const op1,op2,op3: TAsmOp; const opsize: topsizes): boolean;
      begin
        result :=
          (instr.typ = ait_instruction) and
          ((taicpu(instr).opcode = op1) or
           (taicpu(instr).opcode = op2) or
           (taicpu(instr).opcode = op3)
          ) and
          ((opsize = []) or (taicpu(instr).opsize in opsize));
      end;


    function MatchInstruction(const instr : tai;const ops : array of TAsmOp;
     const opsize : topsizes) : boolean;
      var
        op : TAsmOp;
      begin
        result:=false;
        for op in ops do
          begin
            if (instr.typ = ait_instruction) and
               (taicpu(instr).opcode = op) and
               ((opsize = []) or (taicpu(instr).opsize in opsize)) then
               begin
                 result:=true;
                 exit;
               end;
          end;
      end;


    function MatchOperand(const oper: TOper; const reg: TRegister): boolean; inline;
      begin
        result := (oper.typ = top_reg) and (oper.reg = reg);
      end;


    function MatchOperand(const oper: TOper; const a: tcgint): boolean; inline;
      begin
        result := (oper.typ = top_const) and (oper.val = a);
      end;


    function MatchOperand(const oper1: TOper; const oper2: TOper): boolean;
      begin
        result := oper1.typ = oper2.typ;

        if result then
          case oper1.typ of
            top_const:
              Result:=oper1.val = oper2.val;
            top_reg:
              Result:=oper1.reg = oper2.reg;
            top_ref:
              Result:=RefsEqual(oper1.ref^, oper2.ref^);
            else
              internalerror(2013102801);
          end
      end;


    function RefsEqual(const r1, r2: treference): boolean;
      begin
        RefsEqual :=
          (r1.offset = r2.offset) and
          (r1.segment = r2.segment) and (r1.base = r2.base) and
          (r1.index = r2.index) and (r1.scalefactor = r2.scalefactor) and
          (r1.symbol=r2.symbol) and (r1.refaddr = r2.refaddr) and
          (r1.relsymbol = r2.relsymbol) and
          (r1.volatility=[]) and
          (r2.volatility=[]);
      end;


    function MatchReference(const ref : treference;base,index : TRegister) : Boolean;
      begin
       Result:=(ref.offset=0) and
         (ref.scalefactor in [0,1]) and
         (ref.segment=NR_NO) and
         (ref.symbol=nil) and
         (ref.relsymbol=nil) and
         ((base=NR_INVALID) or
          (ref.base=base)) and
         ((index=NR_INVALID) or
          (ref.index=index)) and
         (ref.volatility=[]);
      end;


    function MatchReferenceWithOffset(const ref : treference;base,index : TRegister) : Boolean;
      begin
       Result:=(ref.scalefactor in [0,1]) and
         (ref.segment=NR_NO) and
         (ref.symbol=nil) and
         (ref.relsymbol=nil) and
         ((base=NR_INVALID) or
          (ref.base=base)) and
         ((index=NR_INVALID) or
          (ref.index=index)) and
         (ref.volatility=[]);
      end;


    function InstrReadsFlags(p: tai): boolean;
      begin
        InstrReadsFlags := true;
        case p.typ of
          ait_instruction:
            if InsProp[taicpu(p).opcode].Ch*
               [Ch_RCarryFlag,Ch_RParityFlag,Ch_RAuxiliaryFlag,Ch_RZeroFlag,Ch_RSignFlag,Ch_ROverflowFlag,
                Ch_RWCarryFlag,Ch_RWParityFlag,Ch_RWAuxiliaryFlag,Ch_RWZeroFlag,Ch_RWSignFlag,Ch_RWOverflowFlag,
                Ch_RFlags,Ch_RWFlags,Ch_RFLAGScc,Ch_All]<>[] then
              exit;
          ait_label:
            exit;
          else
            ;
        end;
        InstrReadsFlags := false;
      end;


  function TX86AsmOptimizer.GetNextInstructionUsingReg(Current: tai; out Next: tai; reg: TRegister): Boolean;
    begin
      Next:=Current;
      repeat
        Result:=GetNextInstruction(Next,Next);
      until not (Result) or
            not(cs_opt_level3 in current_settings.optimizerswitches) or
            (Next.typ<>ait_instruction) or
            RegInInstruction(reg,Next) or
            is_calljmp(taicpu(Next).opcode);
    end;


  function TX86AsmOptimizer.InstructionLoadsFromReg(const reg: TRegister;const hp: tai): boolean;
    begin
      Result:=RegReadByInstruction(reg,hp);
    end;


  function TX86AsmOptimizer.RegReadByInstruction(reg: TRegister; hp: tai): boolean;
    var
      p: taicpu;
      opcount: longint;
    begin
      RegReadByInstruction := false;
      if hp.typ <> ait_instruction then
        exit;
      p := taicpu(hp);
      case p.opcode of
        A_CALL:
          regreadbyinstruction := true;
        A_IMUL:
          case p.ops of
            1:
              regReadByInstruction := RegInOp(reg,p.oper[0]^) or
                 (
                  ((getregtype(reg)=R_INTREGISTER) and (getsupreg(reg)=RS_EAX)) and
                  ((getsubreg(reg)<>R_SUBH) or (p.opsize<>S_B))
                 );
            2,3:
              regReadByInstruction :=
                reginop(reg,p.oper[0]^) or
                reginop(reg,p.oper[1]^);
          end;
        A_MUL:
          begin
            regReadByInstruction := RegInOp(reg,p.oper[0]^) or
               (
                ((getregtype(reg)=R_INTREGISTER) and (getsupreg(reg)=RS_EAX)) and
                ((getsubreg(reg)<>R_SUBH) or (p.opsize<>S_B))
               );
          end;
        A_IDIV,A_DIV:
          begin
            regReadByInstruction := RegInOp(reg,p.oper[0]^) or
               (
                 (getregtype(reg)=R_INTREGISTER) and
                 (
                   (getsupreg(reg)=RS_EAX) or ((getsupreg(reg)=RS_EDX) and (p.opsize<>S_B))
                 )
               );
          end;
        else
          begin
            if (p.opcode=A_LEA) and is_segment_reg(reg) then
              begin
                RegReadByInstruction := false;
                exit;
              end;
            for opcount := 0 to p.ops-1 do
              if (p.oper[opCount]^.typ = top_ref) and
                 RegInRef(reg,p.oper[opcount]^.ref^) then
                begin
                  RegReadByInstruction := true;
                  exit
                end;
            { special handling for SSE MOVSD }
            if (p.opcode=A_MOVSD) and (p.ops>0) then
              begin
                if p.ops<>2 then
                  internalerror(2017042702);
                regReadByInstruction := reginop(reg,p.oper[0]^) or
                  (
                   (p.oper[1]^.typ=top_reg) and (p.oper[0]^.typ=top_reg) and reginop(reg, p.oper[1]^)
                  );
                exit;
              end;
            with insprop[p.opcode] do
              begin
                if getregtype(reg)=R_INTREGISTER then
                  begin
                    case getsupreg(reg) of
                      RS_EAX:
                        if [Ch_REAX,Ch_RWEAX,Ch_MEAX]*Ch<>[] then
                          begin
                            RegReadByInstruction := true;
                            exit
                          end;
                      RS_ECX:
                        if [Ch_RECX,Ch_RWECX,Ch_MECX]*Ch<>[] then
                          begin
                            RegReadByInstruction := true;
                            exit
                          end;
                      RS_EDX:
                        if [Ch_REDX,Ch_RWEDX,Ch_MEDX]*Ch<>[] then
                          begin
                            RegReadByInstruction := true;
                            exit
                          end;
                      RS_EBX:
                        if [Ch_REBX,Ch_RWEBX,Ch_MEBX]*Ch<>[] then
                          begin
                            RegReadByInstruction := true;
                            exit
                          end;
                      RS_ESP:
                        if [Ch_RESP,Ch_RWESP,Ch_MESP]*Ch<>[] then
                          begin
                            RegReadByInstruction := true;
                            exit
                          end;
                      RS_EBP:
                        if [Ch_REBP,Ch_RWEBP,Ch_MEBP]*Ch<>[] then
                          begin
                            RegReadByInstruction := true;
                            exit
                          end;
                      RS_ESI:
                        if [Ch_RESI,Ch_RWESI,Ch_MESI]*Ch<>[] then
                          begin
                            RegReadByInstruction := true;
                            exit
                          end;
                      RS_EDI:
                        if [Ch_REDI,Ch_RWEDI,Ch_MEDI]*Ch<>[] then
                          begin
                            RegReadByInstruction := true;
                            exit
                          end;
                    end;
                  end;
                if SuperRegistersEqual(reg,NR_DEFAULTFLAGS) then
                  begin
                    if (Ch_RFLAGScc in Ch) and not(getsubreg(reg) in [R_SUBW,R_SUBD,R_SUBQ]) then
                      begin
                        case p.condition of
                          C_A,C_NBE,       { CF=0 and ZF=0  }
                          C_BE,C_NA:       { CF=1 or ZF=1   }
                            RegReadByInstruction:=getsubreg(reg) in [R_SUBFLAGCARRY,R_SUBFLAGZERO];
                          C_AE,C_NB,C_NC,  { CF=0           }
                          C_B,C_NAE,C_C:   { CF=1           }
                            RegReadByInstruction:=getsubreg(reg) in [R_SUBFLAGCARRY];
                          C_NE,C_NZ,       { ZF=0           }
                          C_E,C_Z:         { ZF=1           }
                            RegReadByInstruction:=getsubreg(reg) in [R_SUBFLAGZERO];
                          C_G,C_NLE,       { ZF=0 and SF=OF }
                          C_LE,C_NG:       { ZF=1 or SF<>OF }
                            RegReadByInstruction:=getsubreg(reg) in [R_SUBFLAGZERO,R_SUBFLAGSIGN,R_SUBFLAGOVERFLOW];
                          C_GE,C_NL,       { SF=OF          }
                          C_L,C_NGE:       { SF<>OF         }
                            RegReadByInstruction:=getsubreg(reg) in [R_SUBFLAGSIGN,R_SUBFLAGOVERFLOW];
                          C_NO,            { OF=0           }
                          C_O:             { OF=1           }
                            RegReadByInstruction:=getsubreg(reg) in [R_SUBFLAGOVERFLOW];
                          C_NP,C_PO,       { PF=0           }
                          C_P,C_PE:        { PF=1           }
                            RegReadByInstruction:=getsubreg(reg) in [R_SUBFLAGPARITY];
                          C_NS,            { SF=0           }
                          C_S:             { SF=1           }
                            RegReadByInstruction:=getsubreg(reg) in [R_SUBFLAGSIGN];
                          else
                            internalerror(2017042701);
                        end;
                        if RegReadByInstruction then
                          exit;
                      end;
                    case getsubreg(reg) of
                      R_SUBW,R_SUBD,R_SUBQ:
                        RegReadByInstruction :=
                          [Ch_RCarryFlag,Ch_RParityFlag,Ch_RAuxiliaryFlag,Ch_RZeroFlag,Ch_RSignFlag,Ch_ROverflowFlag,
                           Ch_RWCarryFlag,Ch_RWParityFlag,Ch_RWAuxiliaryFlag,Ch_RWZeroFlag,Ch_RWSignFlag,Ch_RWOverflowFlag,
                           Ch_RDirFlag,Ch_RFlags,Ch_RWFlags,Ch_RFLAGScc]*Ch<>[];
                      R_SUBFLAGCARRY:
                        RegReadByInstruction:=[Ch_RCarryFlag,Ch_RWCarryFlag,Ch_RFlags,Ch_RWFlags]*Ch<>[];
                      R_SUBFLAGPARITY:
                        RegReadByInstruction:=[Ch_RParityFlag,Ch_RWParityFlag,Ch_RFlags,Ch_RWFlags]*Ch<>[];
                      R_SUBFLAGAUXILIARY:
                        RegReadByInstruction:=[Ch_RAuxiliaryFlag,Ch_RWAuxiliaryFlag,Ch_RFlags,Ch_RWFlags]*Ch<>[];
                      R_SUBFLAGZERO:
                        RegReadByInstruction:=[Ch_RZeroFlag,Ch_RWZeroFlag,Ch_RFlags,Ch_RWFlags]*Ch<>[];
                      R_SUBFLAGSIGN:
                        RegReadByInstruction:=[Ch_RSignFlag,Ch_RWSignFlag,Ch_RFlags,Ch_RWFlags]*Ch<>[];
                      R_SUBFLAGOVERFLOW:
                        RegReadByInstruction:=[Ch_ROverflowFlag,Ch_RWOverflowFlag,Ch_RFlags,Ch_RWFlags]*Ch<>[];
                      R_SUBFLAGINTERRUPT:
                        RegReadByInstruction:=[Ch_RFlags,Ch_RWFlags]*Ch<>[];
                      R_SUBFLAGDIRECTION:
                        RegReadByInstruction:=[Ch_RDirFlag,Ch_RFlags,Ch_RWFlags]*Ch<>[];
                      else
                        internalerror(2017042601);
                    end;
                    exit;
                  end;
                if (Ch_NoReadIfEqualRegs in Ch) and (p.ops=2) and
                   (p.oper[0]^.typ=top_reg) and (p.oper[1]^.typ=top_reg) and
                   (p.oper[0]^.reg=p.oper[1]^.reg) then
                  exit;
                if ([CH_RWOP1,CH_ROP1,CH_MOP1]*Ch<>[]) and reginop(reg,p.oper[0]^) then
                  begin
                    RegReadByInstruction := true;
                    exit
                  end;
                if ([Ch_RWOP2,Ch_ROP2,Ch_MOP2]*Ch<>[]) and reginop(reg,p.oper[1]^) then
                  begin
                    RegReadByInstruction := true;
                    exit
                  end;
                if ([Ch_RWOP3,Ch_ROP3,Ch_MOP3]*Ch<>[]) and reginop(reg,p.oper[2]^) then
                  begin
                    RegReadByInstruction := true;
                    exit
                  end;
                if ([Ch_RWOP4,Ch_ROP4,Ch_MOP4]*Ch<>[]) and reginop(reg,p.oper[3]^) then
                  begin
                    RegReadByInstruction := true;
                    exit
                  end;
              end;
          end;
      end;
    end;


  function TX86AsmOptimizer.RegInInstruction(Reg: TRegister; p1: tai): Boolean;
    begin
      result:=false;
      if p1.typ<>ait_instruction then
        exit;

      if (Ch_All in insprop[taicpu(p1).opcode].Ch) then
        exit(true);

      if (getregtype(reg)=R_INTREGISTER) and
        { change information for xmm movsd are not correct }
        ((taicpu(p1).opcode<>A_MOVSD) or (taicpu(p1).ops=0)) then
        begin
          case getsupreg(reg) of
            { RS_EAX = RS_RAX on x86-64 }
            RS_EAX:
              result:=([Ch_REAX,Ch_RRAX,Ch_WEAX,Ch_WRAX,Ch_RWEAX,Ch_RWRAX,Ch_MEAX,Ch_MRAX]*insprop[taicpu(p1).opcode].Ch)<>[];
            RS_ECX:
              result:=([Ch_RECX,Ch_RRCX,Ch_WECX,Ch_WRCX,Ch_RWECX,Ch_RWRCX,Ch_MECX,Ch_MRCX]*insprop[taicpu(p1).opcode].Ch)<>[];
            RS_EDX:
              result:=([Ch_REDX,Ch_RRDX,Ch_WEDX,Ch_WRDX,Ch_RWEDX,Ch_RWRDX,Ch_MEDX,Ch_MRDX]*insprop[taicpu(p1).opcode].Ch)<>[];
            RS_EBX:
              result:=([Ch_REBX,Ch_RRBX,Ch_WEBX,Ch_WRBX,Ch_RWEBX,Ch_RWRBX,Ch_MEBX,Ch_MRBX]*insprop[taicpu(p1).opcode].Ch)<>[];
            RS_ESP:
              result:=([Ch_RESP,Ch_RRSP,Ch_WESP,Ch_WRSP,Ch_RWESP,Ch_RWRSP,Ch_MESP,Ch_MRSP]*insprop[taicpu(p1).opcode].Ch)<>[];
            RS_EBP:
              result:=([Ch_REBP,Ch_RRBP,Ch_WEBP,Ch_WRBP,Ch_RWEBP,Ch_RWRBP,Ch_MEBP,Ch_MRBP]*insprop[taicpu(p1).opcode].Ch)<>[];
            RS_ESI:
              result:=([Ch_RESI,Ch_RRSI,Ch_WESI,Ch_WRSI,Ch_RWESI,Ch_RWRSI,Ch_MESI,Ch_MRSI,Ch_RMemEDI]*insprop[taicpu(p1).opcode].Ch)<>[];
            RS_EDI:
              result:=([Ch_REDI,Ch_RRDI,Ch_WEDI,Ch_WRDI,Ch_RWEDI,Ch_RWRDI,Ch_MEDI,Ch_MRDI,Ch_WMemEDI]*insprop[taicpu(p1).opcode].Ch)<>[];
            else
              ;
          end;
          if result then
            exit;
        end
      else if SuperRegistersEqual(reg,NR_DEFAULTFLAGS) then
        begin
          if ([Ch_RFlags,Ch_WFlags,Ch_RWFlags,Ch_RFLAGScc]*insprop[taicpu(p1).opcode].Ch)<>[] then
            exit(true);
          case getsubreg(reg) of
            R_SUBFLAGCARRY:
              Result:=([Ch_RCarryFlag,Ch_RWCarryFlag,Ch_W0CarryFlag,Ch_W1CarryFlag,Ch_WCarryFlag,Ch_WUCarryFlag]*insprop[taicpu(p1).opcode].Ch)<>[];
            R_SUBFLAGPARITY:
              Result:=([Ch_RParityFlag,Ch_RWParityFlag,Ch_W0ParityFlag,Ch_W1ParityFlag,Ch_WParityFlag,Ch_WUParityFlag]*insprop[taicpu(p1).opcode].Ch)<>[];
            R_SUBFLAGAUXILIARY:
              Result:=([Ch_RAuxiliaryFlag,Ch_RWAuxiliaryFlag,Ch_W0AuxiliaryFlag,Ch_W1AuxiliaryFlag,Ch_WAuxiliaryFlag,Ch_WUAuxiliaryFlag]*insprop[taicpu(p1).opcode].Ch)<>[];
            R_SUBFLAGZERO:
              Result:=([Ch_RZeroFlag,Ch_RWZeroFlag,Ch_W0ZeroFlag,Ch_W1ZeroFlag,Ch_WZeroFlag,Ch_WUZeroFlag]*insprop[taicpu(p1).opcode].Ch)<>[];
            R_SUBFLAGSIGN:
              Result:=([Ch_RSignFlag,Ch_RWSignFlag,Ch_W0SignFlag,Ch_W1SignFlag,Ch_WSignFlag,Ch_WUSignFlag]*insprop[taicpu(p1).opcode].Ch)<>[];
            R_SUBFLAGOVERFLOW:
              Result:=([Ch_ROverflowFlag,Ch_RWOverflowFlag,Ch_W0OverflowFlag,Ch_W1OverflowFlag,Ch_WOverflowFlag,Ch_WUOverflowFlag]*insprop[taicpu(p1).opcode].Ch)<>[];
            R_SUBFLAGINTERRUPT:
              Result:=([Ch_W0IntFlag,Ch_W1IntFlag,Ch_WFlags]*insprop[taicpu(p1).opcode].Ch)<>[];
            R_SUBFLAGDIRECTION:
              Result:=([Ch_RDirFlag,Ch_W0DirFlag,Ch_W1DirFlag,Ch_WFlags]*insprop[taicpu(p1).opcode].Ch)<>[];
            else
              ;
          end;
          if result then
            exit;
        end
      else if (getregtype(reg)=R_FPUREGISTER) and (Ch_FPU in insprop[taicpu(p1).opcode].Ch) then
        exit(true);
      Result:=inherited RegInInstruction(Reg, p1);
    end;


{$ifdef DEBUG_AOPTCPU}
    procedure TX86AsmOptimizer.DebugMsg(const s: string;p : tai);
      begin
        asml.insertbefore(tai_comment.Create(strpnew(s)), p);
      end;

    function debug_tostr(i: tcgint): string; inline;
      begin
        Result := tostr(i);
      end;

    function debug_regname(r: TRegister): string; inline;
      begin
        Result := '%' + std_regname(r);
      end;

    { Debug output function - creates a string representation of an operator }
    function debug_operstr(oper: TOper): string;
      begin
        case oper.typ of
          top_const:
            Result := '$' + debug_tostr(oper.val);
          top_reg:
            Result := debug_regname(oper.reg);
          top_ref:
            begin
              if oper.ref^.offset <> 0 then
                Result := debug_tostr(oper.ref^.offset) + '('
              else
                Result := '(';

              if (oper.ref^.base <> NR_INVALID) and (oper.ref^.base <> NR_NO) then
                begin
                  Result := Result + debug_regname(oper.ref^.base);
                  if (oper.ref^.index <> NR_INVALID) and (oper.ref^.index <> NR_NO) then
                    Result := Result + ',' + debug_regname(oper.ref^.index);
                end
              else
                if (oper.ref^.index <> NR_INVALID) and (oper.ref^.index <> NR_NO) then
                  Result := Result + debug_regname(oper.ref^.index);

              if (oper.ref^.scalefactor > 1) then
                Result := Result + ',' + debug_tostr(oper.ref^.scalefactor) + ')'
              else
                Result := Result + ')';
            end;
          else
            Result := '[UNKNOWN]';
        end;
      end;

    function debug_op2str(opcode: tasmop): string; inline;
      begin
        Result := std_op2str[opcode];
      end;

    function debug_opsize2str(opsize: topsize): string; inline;
      begin
        Result := gas_opsize2str[opsize];
      end;

{$else DEBUG_AOPTCPU}
    procedure TX86AsmOptimizer.DebugMsg(const s: string;p : tai);inline;
      begin
      end;

    function debug_tostr(i: tcgint): string; inline;
      begin
        Result := '';
      end;

    function debug_regname(r: TRegister): string; inline;
      begin
        Result := '';
      end;

    function debug_operstr(oper: TOper): string; inline;
      begin
        Result := '';
      end;

    function debug_op2str(opcode: tasmop): string; inline;
      begin
        Result := '';
      end;

    function debug_opsize2str(opsize: topsize): string; inline;
      begin
        Result := '';
      end;
{$endif DEBUG_AOPTCPU}

    function TX86AsmOptimizer.Reg1WriteOverwritesReg2Entirely(reg1, reg2: tregister): boolean;
      begin
        if not SuperRegistersEqual(reg1,reg2) then
          exit(false);
        if getregtype(reg1)<>R_INTREGISTER then
          exit(true);  {because SuperRegisterEqual is true}
        case getsubreg(reg1) of
          { A write to R_SUBL doesn't change R_SUBH and if reg2 is R_SUBW or
            higher, it preserves the high bits, so the new value depends on
            reg2's previous value. In other words, it is equivalent to doing:

            reg2 := (reg2 and $ffffff00) or byte(reg1); }
          R_SUBL:
            exit(getsubreg(reg2)=R_SUBL);
          { A write to R_SUBH doesn't change R_SUBL and if reg2 is R_SUBW or
            higher, it actually does a:

            reg2 := (reg2 and $ffff00ff) or (reg1 and $ff00); }
          R_SUBH:
            exit(getsubreg(reg2)=R_SUBH);
          { If reg2 is R_SUBD or larger, a write to R_SUBW preserves the high 16
            bits of reg2:

            reg2 := (reg2 and $ffff0000) or word(reg1); }
          R_SUBW:
            exit(getsubreg(reg2) in [R_SUBL,R_SUBH,R_SUBW]);
          { a write to R_SUBD always overwrites every other subregister,
            because it clears the high 32 bits of R_SUBQ on x86_64 }
          R_SUBD,
          R_SUBQ:
            exit(true);
          else
            internalerror(2017042801);
        end;
      end;


    function TX86AsmOptimizer.Reg1ReadDependsOnReg2(reg1, reg2: tregister): boolean;
      begin
        if not SuperRegistersEqual(reg1,reg2) then
          exit(false);
        if getregtype(reg1)<>R_INTREGISTER then
          exit(true);  {because SuperRegisterEqual is true}
        case getsubreg(reg1) of
          R_SUBL:
            exit(getsubreg(reg2)<>R_SUBH);
          R_SUBH:
            exit(getsubreg(reg2)<>R_SUBL);
          R_SUBW,
          R_SUBD,
          R_SUBQ:
            exit(true);
          else
            internalerror(2017042802);
        end;
      end;


    function TX86AsmOptimizer.PrePeepholeOptSxx(var p : tai) : boolean;
      var
        hp1 : tai;
        l : TCGInt;
      begin
        result:=false;
        { changes the code sequence
          shr/sar const1, x
          shl     const2, x

          to

          either "sar/and", "shl/and" or just "and" depending on const1 and const2 }
        if GetNextInstruction(p, hp1) and
          MatchInstruction(hp1,A_SHL,[]) and
          (taicpu(p).oper[0]^.typ = top_const) and
          (taicpu(hp1).oper[0]^.typ = top_const) and
          (taicpu(hp1).opsize = taicpu(p).opsize) and
          (taicpu(hp1).oper[1]^.typ = taicpu(p).oper[1]^.typ) and
          OpsEqual(taicpu(hp1).oper[1]^, taicpu(p).oper[1]^) then
          begin
            if (taicpu(p).oper[0]^.val > taicpu(hp1).oper[0]^.val) and
              not(cs_opt_size in current_settings.optimizerswitches) then
              begin
                { shr/sar const1, %reg
                  shl     const2, %reg
                  with const1 > const2 }
                taicpu(p).loadConst(0,taicpu(p).oper[0]^.val-taicpu(hp1).oper[0]^.val);
                taicpu(hp1).opcode := A_AND;
                l := (1 shl (taicpu(hp1).oper[0]^.val)) - 1;
                case taicpu(p).opsize Of
                  S_B: taicpu(hp1).loadConst(0,l Xor $ff);
                  S_W: taicpu(hp1).loadConst(0,l Xor $ffff);
                  S_L: taicpu(hp1).loadConst(0,l Xor tcgint($ffffffff));
                  S_Q: taicpu(hp1).loadConst(0,l Xor tcgint($ffffffffffffffff));
                  else
                    Internalerror(2017050703)
                end;
              end
            else if (taicpu(p).oper[0]^.val<taicpu(hp1).oper[0]^.val) and
              not(cs_opt_size in current_settings.optimizerswitches) then
              begin
                { shr/sar const1, %reg
                  shl     const2, %reg
                  with const1 < const2 }
                taicpu(hp1).loadConst(0,taicpu(hp1).oper[0]^.val-taicpu(p).oper[0]^.val);
                taicpu(p).opcode := A_AND;
                l := (1 shl (taicpu(p).oper[0]^.val))-1;
                case taicpu(p).opsize Of
                  S_B: taicpu(p).loadConst(0,l Xor $ff);
                  S_W: taicpu(p).loadConst(0,l Xor $ffff);
                  S_L: taicpu(p).loadConst(0,l Xor tcgint($ffffffff));
                  S_Q: taicpu(p).loadConst(0,l Xor tcgint($ffffffffffffffff));
                  else
                    Internalerror(2017050702)
                end;
              end
            else if (taicpu(p).oper[0]^.val = taicpu(hp1).oper[0]^.val) then
              begin
                { shr/sar const1, %reg
                  shl     const2, %reg
                  with const1 = const2 }
                taicpu(p).opcode := A_AND;
                l := (1 shl (taicpu(p).oper[0]^.val))-1;
                case taicpu(p).opsize Of
                  S_B: taicpu(p).loadConst(0,l Xor $ff);
                  S_W: taicpu(p).loadConst(0,l Xor $ffff);
                  S_L: taicpu(p).loadConst(0,l Xor tcgint($ffffffff));
                  S_Q: taicpu(p).loadConst(0,l Xor tcgint($ffffffffffffffff));
                  else
                    Internalerror(2017050701)
                end;
                asml.remove(hp1);
                hp1.free;
              end;
          end;
      end;


    function TX86AsmOptimizer.PrePeepholeOptIMUL(var p : tai) : boolean;
      var
        opsize : topsize;
        hp1 : tai;
        tmpref : treference;
        ShiftValue : Cardinal;
        BaseValue : TCGInt;
      begin
        result:=false;
        opsize:=taicpu(p).opsize;
        { changes certain "imul const, %reg"'s to lea sequences }
        if (MatchOpType(taicpu(p),top_const,top_reg) or
            MatchOpType(taicpu(p),top_const,top_reg,top_reg)) and
           (opsize in [S_L{$ifdef x86_64},S_Q{$endif x86_64}]) then
          if (taicpu(p).oper[0]^.val = 1) then
            if (taicpu(p).ops = 2) then
             { remove "imul $1, reg" }
              begin
                hp1 := tai(p.Next);
                asml.remove(p);
                DebugMsg(SPeepholeOptimization + 'Imul2Nop done',p);
                p.free;
                p := hp1;
                result:=true;
              end
            else
             { change "imul $1, reg1, reg2" to "mov reg1, reg2" }
              begin
                hp1 := taicpu.Op_Reg_Reg(A_MOV, opsize, taicpu(p).oper[1]^.reg,taicpu(p).oper[2]^.reg);
                InsertLLItem(p.previous, p.next, hp1);
                DebugMsg(SPeepholeOptimization + 'Imul2Mov done',p);
                p.free;
                p := hp1;
              end
          else if
           ((taicpu(p).ops <= 2) or
            (taicpu(p).oper[2]^.typ = Top_Reg)) and
           not(cs_opt_size in current_settings.optimizerswitches) and
           (not(GetNextInstruction(p, hp1)) or
             not((tai(hp1).typ = ait_instruction) and
                 ((taicpu(hp1).opcode=A_Jcc) and
                  (taicpu(hp1).condition in [C_O,C_NO])))) then
            begin
              {
                imul X, reg1, reg2 to
                  lea (reg1,reg1,Y), reg2
                  shl ZZ,reg2
                imul XX, reg1 to
                  lea (reg1,reg1,YY), reg1
                  shl ZZ,reg2

                This optimziation makes sense for pretty much every x86, except the VIA Nano3000: it has IMUL latency 2, lea/shl pair as well,
                it does not exist as a separate optimization target in FPC though.

                This optimziation can be applied as long as only two bits are set in the constant and those two bits are separated by
                at most two zeros
              }
              reference_reset(tmpref,1,[]);
              if (PopCnt(QWord(taicpu(p).oper[0]^.val))=2) and (BsrQWord(taicpu(p).oper[0]^.val)-BsfQWord(taicpu(p).oper[0]^.val)<=3) then
                begin
                  ShiftValue:=BsfQWord(taicpu(p).oper[0]^.val);
                  BaseValue:=taicpu(p).oper[0]^.val shr ShiftValue;
                  TmpRef.base := taicpu(p).oper[1]^.reg;
                  TmpRef.index := taicpu(p).oper[1]^.reg;
                  if not(BaseValue in [3,5,9]) then
                    Internalerror(2018110101);
                  TmpRef.ScaleFactor := BaseValue-1;
                  if (taicpu(p).ops = 2) then
                    hp1 := taicpu.op_ref_reg(A_LEA, opsize, TmpRef, taicpu(p).oper[1]^.reg)
                  else
                    hp1 := taicpu.op_ref_reg(A_LEA, opsize, TmpRef, taicpu(p).oper[2]^.reg);
                  AsmL.InsertAfter(hp1,p);
                  DebugMsg(SPeepholeOptimization + 'Imul2LeaShl done',p);
                  AsmL.Remove(p);
                  taicpu(hp1).fileinfo:=taicpu(p).fileinfo;
                  p.free;
                  p := hp1;
                  if ShiftValue>0 then
                    AsmL.InsertAfter(taicpu.op_const_reg(A_SHL, opsize, ShiftValue, taicpu(hp1).oper[1]^.reg),hp1);
              end;
            end;
      end;


    function TX86AsmOptimizer.RegLoadedWithNewValue(reg: tregister; hp: tai): boolean;
      var
        p: taicpu;
      begin
        if not assigned(hp) or
           (hp.typ <> ait_instruction) then
         begin
           Result := false;
           exit;
         end;
        p := taicpu(hp);
        if SuperRegistersEqual(reg,NR_DEFAULTFLAGS) then
          with insprop[p.opcode] do
            begin
              case getsubreg(reg) of
                R_SUBW,R_SUBD,R_SUBQ:
                  Result:=
                    RegLoadedWithNewValue(NR_CARRYFLAG,hp) and
                    RegLoadedWithNewValue(NR_PARITYFLAG,hp) and
                    RegLoadedWithNewValue(NR_AUXILIARYFLAG,hp) and
                    RegLoadedWithNewValue(NR_ZEROFLAG,hp) and
                    RegLoadedWithNewValue(NR_SIGNFLAG,hp) and
                    RegLoadedWithNewValue(NR_OVERFLOWFLAG,hp);
                R_SUBFLAGCARRY:
                  Result:=[Ch_W0CarryFlag,Ch_W1CarryFlag,Ch_WCarryFlag,Ch_WUCarryFlag,Ch_WFlags]*Ch<>[];
                R_SUBFLAGPARITY:
                  Result:=[Ch_W0ParityFlag,Ch_W1ParityFlag,Ch_WParityFlag,Ch_WUParityFlag,Ch_WFlags]*Ch<>[];
                R_SUBFLAGAUXILIARY:
                  Result:=[Ch_W0AuxiliaryFlag,Ch_W1AuxiliaryFlag,Ch_WAuxiliaryFlag,Ch_WUAuxiliaryFlag,Ch_WFlags]*Ch<>[];
                R_SUBFLAGZERO:
                  Result:=[Ch_W0ZeroFlag,Ch_W1ZeroFlag,Ch_WZeroFlag,Ch_WUZeroFlag,Ch_WFlags]*Ch<>[];
                R_SUBFLAGSIGN:
                  Result:=[Ch_W0SignFlag,Ch_W1SignFlag,Ch_WSignFlag,Ch_WUSignFlag,Ch_WFlags]*Ch<>[];
                R_SUBFLAGOVERFLOW:
                  Result:=[Ch_W0OverflowFlag,Ch_W1OverflowFlag,Ch_WOverflowFlag,Ch_WUOverflowFlag,Ch_WFlags]*Ch<>[];
                R_SUBFLAGINTERRUPT:
                  Result:=[Ch_W0IntFlag,Ch_W1IntFlag,Ch_WFlags]*Ch<>[];
                R_SUBFLAGDIRECTION:
                  Result:=[Ch_W0DirFlag,Ch_W1DirFlag,Ch_WFlags]*Ch<>[];
                else
                  begin
                  writeln(getsubreg(reg));
                  internalerror(2017050501);
                  end;
              end;
              exit;
            end;
        Result :=
          (((p.opcode = A_MOV) or
            (p.opcode = A_MOVZX) or
            (p.opcode = A_MOVSX) or
            (p.opcode = A_LEA) or
            (p.opcode = A_VMOVSS) or
            (p.opcode = A_VMOVSD) or
            (p.opcode = A_VMOVAPD) or
            (p.opcode = A_VMOVAPS) or
            (p.opcode = A_VMOVQ) or
            (p.opcode = A_MOVSS) or
            (p.opcode = A_MOVSD) or
            (p.opcode = A_MOVQ) or
            (p.opcode = A_MOVAPD) or
            (p.opcode = A_MOVAPS) or
{$ifndef x86_64}
            (p.opcode = A_LDS) or
            (p.opcode = A_LES) or
{$endif not x86_64}
            (p.opcode = A_LFS) or
            (p.opcode = A_LGS) or
            (p.opcode = A_LSS)) and
           (p.ops=2) and  { A_MOVSD can have zero operands, so this check is needed }
           (p.oper[1]^.typ = top_reg) and
           (Reg1WriteOverwritesReg2Entirely(p.oper[1]^.reg,reg)) and
           ((p.oper[0]^.typ = top_const) or
            ((p.oper[0]^.typ = top_reg) and
             not(Reg1ReadDependsOnReg2(p.oper[0]^.reg,reg))) or
            ((p.oper[0]^.typ = top_ref) and
             not RegInRef(reg,p.oper[0]^.ref^)))) or
          ((p.opcode = A_POP) and
           (Reg1WriteOverwritesReg2Entirely(p.oper[0]^.reg,reg))) or
          ((p.opcode = A_IMUL) and
           (p.ops=3) and
           (Reg1WriteOverwritesReg2Entirely(p.oper[2]^.reg,reg)) and
           (((p.oper[1]^.typ=top_reg) and not(Reg1ReadDependsOnReg2(p.oper[1]^.reg,reg))) or
            ((p.oper[1]^.typ=top_ref) and not(RegInRef(reg,p.oper[1]^.ref^))))) or
          ((((p.opcode = A_IMUL) or
             (p.opcode = A_MUL)) and
            (p.ops=1)) and
           (((p.oper[0]^.typ=top_reg) and not(Reg1ReadDependsOnReg2(p.oper[0]^.reg,reg))) or
            ((p.oper[0]^.typ=top_ref) and not(RegInRef(reg,p.oper[0]^.ref^)))) and
           (((p.opsize=S_B) and Reg1WriteOverwritesReg2Entirely(NR_AX,reg) and not(Reg1ReadDependsOnReg2(NR_AL,reg))) or
            ((p.opsize=S_W) and Reg1WriteOverwritesReg2Entirely(NR_DX,reg)) or
            ((p.opsize=S_L) and Reg1WriteOverwritesReg2Entirely(NR_EDX,reg))
{$ifdef x86_64}
         or ((p.opsize=S_Q) and Reg1WriteOverwritesReg2Entirely(NR_RDX,reg))
{$endif x86_64}
           )) or
          ((p.opcode = A_CWD) and Reg1WriteOverwritesReg2Entirely(NR_DX,reg)) or
          ((p.opcode = A_CDQ) and Reg1WriteOverwritesReg2Entirely(NR_EDX,reg)) or
{$ifdef x86_64}
          ((p.opcode = A_CQO) and Reg1WriteOverwritesReg2Entirely(NR_RDX,reg)) or
{$endif x86_64}
          ((p.opcode = A_CBW) and Reg1WriteOverwritesReg2Entirely(NR_AX,reg) and not(Reg1ReadDependsOnReg2(NR_AL,reg))) or
{$ifndef x86_64}
          ((p.opcode = A_LDS) and (reg=NR_DS) and not(RegInRef(reg,p.oper[0]^.ref^))) or
          ((p.opcode = A_LES) and (reg=NR_ES) and not(RegInRef(reg,p.oper[0]^.ref^))) or
{$endif not x86_64}
          ((p.opcode = A_LFS) and (reg=NR_FS) and not(RegInRef(reg,p.oper[0]^.ref^))) or
          ((p.opcode = A_LGS) and (reg=NR_GS) and not(RegInRef(reg,p.oper[0]^.ref^))) or
          ((p.opcode = A_LSS) and (reg=NR_SS) and not(RegInRef(reg,p.oper[0]^.ref^))) or
{$ifndef x86_64}
          ((p.opcode = A_AAM) and Reg1WriteOverwritesReg2Entirely(NR_AH,reg)) or
{$endif not x86_64}
          ((p.opcode = A_LAHF) and Reg1WriteOverwritesReg2Entirely(NR_AH,reg)) or
          ((p.opcode = A_LODSB) and Reg1WriteOverwritesReg2Entirely(NR_AL,reg)) or
          ((p.opcode = A_LODSW) and Reg1WriteOverwritesReg2Entirely(NR_AX,reg)) or
          ((p.opcode = A_LODSD) and Reg1WriteOverwritesReg2Entirely(NR_EAX,reg)) or
{$ifdef x86_64}
          ((p.opcode = A_LODSQ) and Reg1WriteOverwritesReg2Entirely(NR_RAX,reg)) or
{$endif x86_64}
          ((p.opcode = A_SETcc) and (p.oper[0]^.typ=top_reg) and Reg1WriteOverwritesReg2Entirely(p.oper[0]^.reg,reg)) or
          (((p.opcode = A_FSTSW) or
            (p.opcode = A_FNSTSW)) and
           (p.oper[0]^.typ=top_reg) and
           Reg1WriteOverwritesReg2Entirely(p.oper[0]^.reg,reg)) or
          (((p.opcode = A_XOR) or (p.opcode = A_SUB) or (p.opcode = A_SBB)) and
           (p.oper[0]^.typ=top_reg) and (p.oper[1]^.typ=top_reg) and
           (p.oper[0]^.reg=p.oper[1]^.reg) and
           Reg1WriteOverwritesReg2Entirely(p.oper[1]^.reg,reg));
      end;


    class function TX86AsmOptimizer.IsExitCode(p : tai) : boolean;
      var
        hp2,hp3 : tai;
      begin
        { some x86-64 issue a NOP before the real exit code }
        if MatchInstruction(p,A_NOP,[]) then
          GetNextInstruction(p,p);
        result:=assigned(p) and (p.typ=ait_instruction) and
        ((taicpu(p).opcode = A_RET) or
         ((taicpu(p).opcode=A_LEAVE) and
          GetNextInstruction(p,hp2) and
          MatchInstruction(hp2,A_RET,[S_NO])
         ) or
         (((taicpu(p).opcode=A_LEA) and
           MatchOpType(taicpu(p),top_ref,top_reg) and
           (taicpu(p).oper[0]^.ref^.base=NR_STACK_POINTER_REG) and
           (taicpu(p).oper[1]^.reg=NR_STACK_POINTER_REG)
           ) and
          GetNextInstruction(p,hp2) and
          MatchInstruction(hp2,A_RET,[S_NO])
         ) or
         ((((taicpu(p).opcode=A_MOV) and
           MatchOpType(taicpu(p),top_reg,top_reg) and
           (taicpu(p).oper[0]^.reg=current_procinfo.framepointer) and
           (taicpu(p).oper[1]^.reg=NR_STACK_POINTER_REG)) or
           ((taicpu(p).opcode=A_LEA) and
           MatchOpType(taicpu(p),top_ref,top_reg) and
           (taicpu(p).oper[0]^.ref^.base=current_procinfo.framepointer) and
           (taicpu(p).oper[1]^.reg=NR_STACK_POINTER_REG)
           )
          ) and
          GetNextInstruction(p,hp2) and
          MatchInstruction(hp2,A_POP,[reg2opsize(current_procinfo.framepointer)]) and
          MatchOpType(taicpu(hp2),top_reg) and
          (taicpu(hp2).oper[0]^.reg=current_procinfo.framepointer) and
          GetNextInstruction(hp2,hp3) and
          MatchInstruction(hp3,A_RET,[S_NO])
         )
        );
      end;


    class function TX86AsmOptimizer.isFoldableArithOp(hp1: taicpu; reg: tregister): boolean;
      begin
        isFoldableArithOp := False;
        case hp1.opcode of
          A_ADD,A_SUB,A_OR,A_XOR,A_AND,A_SHL,A_SHR,A_SAR:
            isFoldableArithOp :=
              ((taicpu(hp1).oper[0]^.typ = top_const) or
               ((taicpu(hp1).oper[0]^.typ = top_reg) and
                (taicpu(hp1).oper[0]^.reg <> reg))) and
              (taicpu(hp1).oper[1]^.typ = top_reg) and
              (taicpu(hp1).oper[1]^.reg = reg);
          A_INC,A_DEC,A_NEG,A_NOT:
            isFoldableArithOp :=
              (taicpu(hp1).oper[0]^.typ = top_reg) and
              (taicpu(hp1).oper[0]^.reg = reg);
          else
            ;
        end;
      end;


    procedure TX86AsmOptimizer.RemoveLastDeallocForFuncRes(p: tai);

      procedure DoRemoveLastDeallocForFuncRes( supreg: tsuperregister);
        var
          hp2: tai;
        begin
          hp2 := p;
          repeat
            hp2 := tai(hp2.previous);
            if assigned(hp2) and
               (hp2.typ = ait_regalloc) and
               (tai_regalloc(hp2).ratype=ra_dealloc) and
               (getregtype(tai_regalloc(hp2).reg) = R_INTREGISTER) and
               (getsupreg(tai_regalloc(hp2).reg) = supreg) then
              begin
                asml.remove(hp2);
                hp2.free;
                break;
              end;
          until not(assigned(hp2)) or regInInstruction(newreg(R_INTREGISTER,supreg,R_SUBWHOLE),hp2);
        end;

      begin
          case current_procinfo.procdef.returndef.typ of
            arraydef,recorddef,pointerdef,
               stringdef,enumdef,procdef,objectdef,errordef,
               filedef,setdef,procvardef,
               classrefdef,forwarddef:
              DoRemoveLastDeallocForFuncRes(RS_EAX);
            orddef:
              if current_procinfo.procdef.returndef.size <> 0 then
                begin
                  DoRemoveLastDeallocForFuncRes(RS_EAX);
                  { for int64/qword }
                  if current_procinfo.procdef.returndef.size = 8 then
                    DoRemoveLastDeallocForFuncRes(RS_EDX);
                end;
            else
              ;
          end;
      end;


    function TX86AsmOptimizer.OptPass1_V_MOVAP(var p : tai) : boolean;
      var
        hp1,hp2 : tai;
      begin
        result:=false;
        if MatchOpType(taicpu(p),top_reg,top_reg) then
          begin
            { vmova* reg1,reg1
              =>
              <nop> }
            if MatchOperand(taicpu(p).oper[0]^,taicpu(p).oper[1]^) then
              begin
                GetNextInstruction(p,hp1);
                asml.Remove(p);
                p.Free;
                p:=hp1;
                result:=true;
              end
            else if GetNextInstruction(p,hp1) then
              begin
                if MatchInstruction(hp1,[taicpu(p).opcode],[S_NO]) and
                  MatchOpType(taicpu(hp1),top_reg,top_reg) and
                  MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[0]^) then
                  begin
                    { vmova* reg1,reg2
                      vmova* reg2,reg3
                      dealloc reg2
                      =>
                      vmova* reg1,reg3 }
                    TransferUsedRegs(TmpUsedRegs);
                    UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                    if not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp1,TmpUsedRegs)) then
                      begin
                        taicpu(p).loadoper(1,taicpu(hp1).oper[1]^);
                        asml.Remove(hp1);
                        hp1.Free;
                        result:=true;
                      end
                    { special case:
                      vmova* reg1,reg2
                      vmova* reg2,reg1
                      =>
                      vmova* reg1,reg2 }
                    else if MatchOperand(taicpu(p).oper[0]^,taicpu(hp1).oper[1]^) then
                      begin
                        asml.Remove(hp1);
                        hp1.Free;
                        result:=true;
                      end
                  end
                else if MatchInstruction(hp1,[A_VFMADDPD,
                                              A_VFMADD132PD,
                                              A_VFMADD132PS,
                                              A_VFMADD132SD,
                                              A_VFMADD132SS,
                                              A_VFMADD213PD,
                                              A_VFMADD213PS,
                                              A_VFMADD213SD,
                                              A_VFMADD213SS,
                                              A_VFMADD231PD,
                                              A_VFMADD231PS,
                                              A_VFMADD231SD,
                                              A_VFMADD231SS,
                                              A_VFMADDSUB132PD,
                                              A_VFMADDSUB132PS,
                                              A_VFMADDSUB213PD,
                                              A_VFMADDSUB213PS,
                                              A_VFMADDSUB231PD,
                                              A_VFMADDSUB231PS,
                                              A_VFMSUB132PD,
                                              A_VFMSUB132PS,
                                              A_VFMSUB132SD,
                                              A_VFMSUB132SS,
                                              A_VFMSUB213PD,
                                              A_VFMSUB213PS,
                                              A_VFMSUB213SD,
                                              A_VFMSUB213SS,
                                              A_VFMSUB231PD,
                                              A_VFMSUB231PS,
                                              A_VFMSUB231SD,
                                              A_VFMSUB231SS,
                                              A_VFMSUBADD132PD,
                                              A_VFMSUBADD132PS,
                                              A_VFMSUBADD213PD,
                                              A_VFMSUBADD213PS,
                                              A_VFMSUBADD231PD,
                                              A_VFMSUBADD231PS,
                                              A_VFNMADD132PD,
                                              A_VFNMADD132PS,
                                              A_VFNMADD132SD,
                                              A_VFNMADD132SS,
                                              A_VFNMADD213PD,
                                              A_VFNMADD213PS,
                                              A_VFNMADD213SD,
                                              A_VFNMADD213SS,
                                              A_VFNMADD231PD,
                                              A_VFNMADD231PS,
                                              A_VFNMADD231SD,
                                              A_VFNMADD231SS,
                                              A_VFNMSUB132PD,
                                              A_VFNMSUB132PS,
                                              A_VFNMSUB132SD,
                                              A_VFNMSUB132SS,
                                              A_VFNMSUB213PD,
                                              A_VFNMSUB213PS,
                                              A_VFNMSUB213SD,
                                              A_VFNMSUB213SS,
                                              A_VFNMSUB231PD,
                                              A_VFNMSUB231PS,
                                              A_VFNMSUB231SD,
                                              A_VFNMSUB231SS],[S_NO]) and
                  { we mix single and double opperations here because we assume that the compiler
                    generates vmovapd only after double operations and vmovaps only after single operations }
                  MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[2]^) and
                  GetNextInstruction(hp1,hp2) and
                  MatchInstruction(hp2,[A_VMOVAPD,A_VMOVAPS,A_MOVAPD,A_MOVAPS],[S_NO]) and
                  MatchOperand(taicpu(p).oper[0]^,taicpu(hp2).oper[1]^) then
                  begin
                    TransferUsedRegs(TmpUsedRegs);
                    UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                    UpdateUsedRegs(TmpUsedRegs, tai(hp1.next));
                    if not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp2,TmpUsedRegs))
                     then
                      begin
                        taicpu(hp1).loadoper(2,taicpu(p).oper[0]^);
                        asml.Remove(p);
                        p.Free;
                        asml.Remove(hp2);
                        hp2.Free;
                        p:=hp1;
                      end;
                  end
                else if (hp1.typ = ait_instruction) and
                  GetNextInstruction(hp1, hp2) and
                  MatchInstruction(hp2,taicpu(p).opcode,[]) and
                  OpsEqual(taicpu(hp2).oper[1]^, taicpu(p).oper[0]^) and
                  MatchOpType(taicpu(hp2),top_reg,top_reg) and
                  MatchOperand(taicpu(hp2).oper[0]^,taicpu(p).oper[1]^) and
                  (((taicpu(p).opcode=A_MOVAPS) and
                    ((taicpu(hp1).opcode=A_ADDSS) or (taicpu(hp1).opcode=A_SUBSS) or
                     (taicpu(hp1).opcode=A_MULSS) or (taicpu(hp1).opcode=A_DIVSS))) or
                   ((taicpu(p).opcode=A_MOVAPD) and
                    ((taicpu(hp1).opcode=A_ADDSD) or (taicpu(hp1).opcode=A_SUBSD) or
                     (taicpu(hp1).opcode=A_MULSD) or (taicpu(hp1).opcode=A_DIVSD)))
                  ) then
                  { change
                             movapX    reg,reg2
                             addsX/subsX/... reg3, reg2
                             movapX    reg2,reg
                    to
                             addsX/subsX/... reg3,reg
                  }
                  begin
                    TransferUsedRegs(TmpUsedRegs);
                    UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                    UpdateUsedRegs(TmpUsedRegs, tai(hp1.next));
                    If not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp2,TmpUsedRegs)) then
                      begin
                        DebugMsg(SPeepholeOptimization + 'MovapXOpMovapX2Op ('+
                              debug_op2str(taicpu(p).opcode)+' '+
                              debug_op2str(taicpu(hp1).opcode)+' '+
                              debug_op2str(taicpu(hp2).opcode)+') done',p);
                        { we cannot eliminate the first move if
                          the operations uses the same register for source and dest }
                        if not(OpsEqual(taicpu(hp1).oper[1]^,taicpu(hp1).oper[0]^)) then
                          begin
                            asml.remove(p);
                            p.Free;
                          end;
                        taicpu(hp1).loadoper(1, taicpu(hp2).oper[1]^);
                        asml.remove(hp2);
                        hp2.Free;
                        p:=hp1;
                        result:=true;
                      end;
                  end;
              end;
          end;
      end;


    function TX86AsmOptimizer.OptPass1VOP(var p : tai) : boolean;
      var
        hp1 : tai;
      begin
        result:=false;
        { replace
            V<Op>X   %mreg1,%mreg2,%mreg3
            VMovX    %mreg3,%mreg4
            dealloc  %mreg3

            by
            V<Op>X   %mreg1,%mreg2,%mreg4
          ?
        }
        if GetNextInstruction(p,hp1) and
          { we mix single and double operations here because we assume that the compiler
            generates vmovapd only after double operations and vmovaps only after single operations }
          MatchInstruction(hp1,A_VMOVAPD,A_VMOVAPS,[S_NO]) and
          MatchOperand(taicpu(p).oper[2]^,taicpu(hp1).oper[0]^) and
          (taicpu(hp1).oper[1]^.typ=top_reg) then
          begin
            TransferUsedRegs(TmpUsedRegs);
            UpdateUsedRegs(TmpUsedRegs, tai(p.next));
            if not(RegUsedAfterInstruction(taicpu(hp1).oper[0]^.reg,hp1,TmpUsedRegs)
             ) then
              begin
                taicpu(p).loadoper(2,taicpu(hp1).oper[1]^);
                DebugMsg(SPeepholeOptimization + 'VOpVmov2VOp done',p);
                asml.Remove(hp1);
                hp1.Free;
                result:=true;
              end;
          end;
      end;


    function TX86AsmOptimizer.OptPass1MOV(var p : tai) : boolean;
      var
        hp1, hp2: tai;
        GetNextInstruction_p: Boolean;
        PreMessage, RegName1, RegName2, InputVal, MaskNum: string;
        NewSize: topsize;
      begin
        Result:=false;

        GetNextInstruction_p:=GetNextInstruction(p, hp1);

        {  remove mov reg1,reg1? }
        if MatchOperand(taicpu(p).oper[0]^,taicpu(p).oper[1]^)
        then
          begin
            DebugMsg(SPeepholeOptimization + 'Mov2Nop done',p);
            { take care of the register (de)allocs following p }
            UpdateUsedRegs(tai(p.next));
            asml.remove(p);
            p.free;
            p:=hp1;
            Result:=true;
            exit;
          end;

        if GetNextInstruction_p and
          MatchInstruction(hp1,A_AND,[]) and
          (taicpu(p).oper[1]^.typ = top_reg) and
          MatchOpType(taicpu(hp1),top_const,top_reg) then
          begin
            if MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[1]^) then
              begin
                case taicpu(p).opsize of
                  S_L:
                    if (taicpu(hp1).oper[0]^.val = $ffffffff) then
                      begin
                        { Optimize out:
                            mov x, %reg
                            and ffffffffh, %reg
                        }
                        DebugMsg(SPeepholeOptimization + 'MovAnd2Mov 1 done',p);
                        asml.remove(hp1);
                        hp1.free;
                        Result:=true;
                        exit;
                      end;
                  S_Q: { TODO: Confirm if this is even possible }
                    if (taicpu(hp1).oper[0]^.val = $ffffffffffffffff) then
                      begin
                        { Optimize out:
                            mov x, %reg
                            and ffffffffffffffffh, %reg
                        }
                        DebugMsg(SPeepholeOptimization + 'MovAnd2Mov 2 done',p);
                        asml.remove(hp1);
                        hp1.free;
                        Result:=true;
                        exit;
                      end;
                  else
                    ;
                end;
              end
            else if (taicpu(p).oper[1]^.typ = top_reg) and (taicpu(hp1).oper[1]^.typ = top_reg) and
              (taicpu(p).oper[0]^.typ <> top_const) and { MOVZX only supports registers and memory, not immediates (use MOV for that!) }
              (getsupreg(taicpu(p).oper[1]^.reg) = getsupreg(taicpu(hp1).oper[1]^.reg))
              then
              begin
                InputVal := debug_operstr(taicpu(p).oper[0]^);
                MaskNum := debug_tostr(taicpu(hp1).oper[0]^.val);

                case taicpu(p).opsize of
                  S_B:
                    if (taicpu(hp1).oper[0]^.val = $ff) then
                      begin
                        { Convert:
                            movb x, %regl        movb x, %regl
                            andw ffh, %regw      andl ffh, %regd
                          To:
                            movzbw x, %regd      movzbl x, %regd

                          (Identical registers, just different sizes)
                        }
                        RegName1 := debug_regname(taicpu(p).oper[1]^.reg); { 8-bit register name }
                        RegName2 := debug_regname(taicpu(hp1).oper[1]^.reg); { 16/32-bit register name }

                        case taicpu(hp1).opsize of
                          S_W: NewSize := S_BW;
                          S_L: NewSize := S_BL;
{$ifdef x86_64}
                          S_Q: NewSize := S_BQ;
{$endif x86_64}
                          else
                            InternalError(2018011510);
                        end;
                      end
                    else
                      NewSize := S_NO;
                  S_W:
                    if (taicpu(hp1).oper[0]^.val = $ffff) then
                      begin
                        { Convert:
                            movw x, %regw
                            andl ffffh, %regd
                          To:
                            movzwl x, %regd

                          (Identical registers, just different sizes)
                        }
                        RegName1 := debug_regname(taicpu(p).oper[1]^.reg); { 16-bit register name }
                        RegName2 := debug_regname(taicpu(hp1).oper[1]^.reg); { 32-bit register name }

                        case taicpu(hp1).opsize of
                          S_L: NewSize := S_WL;
{$ifdef x86_64}
                          S_Q: NewSize := S_WQ;
{$endif x86_64}
                          else
                            InternalError(2018011511);
                        end;
                      end
                    else
                      NewSize := S_NO;
                  else
                    NewSize := S_NO;
                end;

                if NewSize <> S_NO then
                  begin
                    PreMessage := 'mov' + debug_opsize2str(taicpu(p).opsize) + ' ' + InputVal + ',' + RegName1;

                    { The actual optimization }
                    taicpu(p).opcode := A_MOVZX;
                    taicpu(p).changeopsize(NewSize);
                    taicpu(p).oper[1]^ := taicpu(hp1).oper[1]^;

                    { Safeguard if "and" is followed by a conditional command }
                    TransferUsedRegs(TmpUsedRegs);
                    UpdateUsedRegs(TmpUsedRegs,tai(p.next));

                    if (RegUsedAfterInstruction(NR_DEFAULTFLAGS, hp1, TmpUsedRegs)) then
                      begin
                        { At this point, the "and" command is effectively equivalent to
                          "test %reg,%reg". This will be handled separately by the
                          Peephole Optimizer. [Kit] }

                        DebugMsg(SPeepholeOptimization + PreMessage +
                          ' -> movz' + debug_opsize2str(NewSize) + ' ' + InputVal + ',' + RegName2, p);
                      end
                    else
                      begin
                        DebugMsg(SPeepholeOptimization + PreMessage + '; and' + debug_opsize2str(taicpu(hp1).opsize) + ' $' + MaskNum + ',' + RegName2 +
                          ' -> movz' + debug_opsize2str(NewSize) + ' ' + InputVal + ',' + RegName2, p);

                        asml.Remove(hp1);
                        hp1.Free;
                      end;

                    Result := True;
                    Exit;

                  end;
              end;
          end;
        { Next instruction is also a MOV ? }
        if GetNextInstruction_p and
          MatchInstruction(hp1,A_MOV,[taicpu(p).opsize]) then
          begin
            if (taicpu(p).oper[1]^.typ = top_reg) and
              MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[0]^) then
              begin
                TransferUsedRegs(TmpUsedRegs);
                UpdateUsedRegs(TmpUsedRegs, tai(p.Next));
                { we have
                    mov x, %treg
                    mov %treg, y
                }
                if not(RegInOp(taicpu(p).oper[1]^.reg,taicpu(hp1).oper[1]^)) and
                   not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg, hp1, TmpUsedRegs)) then
                  { we've got

                    mov x, %treg
                    mov %treg, y

                    with %treg is not used after }
                  case taicpu(p).oper[0]^.typ Of
                    top_reg:
                      begin
                        { change
                            mov %reg, %treg
                            mov %treg, y

                            to

                            mov %reg, y
                        }
                        if taicpu(hp1).oper[1]^.typ=top_reg then
                          AllocRegBetween(taicpu(hp1).oper[1]^.reg,p,hp1,usedregs);
                        taicpu(p).loadOper(1,taicpu(hp1).oper[1]^);
                        DebugMsg(SPeepholeOptimization + 'MovMov2Mov 2 done',p);
                        asml.remove(hp1);
                        hp1.free;
                        Result:=true;
                        Exit;
                      end;
                    top_const:
                      begin
                        { change
                            mov const, %treg
                            mov %treg, y

                            to

                            mov const, y
                        }
                        if (taicpu(hp1).oper[1]^.typ=top_reg) or
                          ((taicpu(p).oper[0]^.val>=low(longint)) and (taicpu(p).oper[0]^.val<=high(longint))) then
                          begin
                            if taicpu(hp1).oper[1]^.typ=top_reg then
                              AllocRegBetween(taicpu(hp1).oper[1]^.reg,p,hp1,usedregs);
                            taicpu(p).loadOper(1,taicpu(hp1).oper[1]^);
                            DebugMsg(SPeepholeOptimization + 'MovMov2Mov 5 done',p);
                            asml.remove(hp1);
                            hp1.free;
                            Result:=true;
                            Exit;
                          end;
                      end;
                    top_ref:
                      if (taicpu(hp1).oper[1]^.typ = top_reg) then
                        begin
                          { change
                               mov mem, %treg
                               mov %treg, %reg

                               to

                               mov mem, %reg"
                          }
                          taicpu(p).loadoper(1,taicpu(hp1).oper[1]^);
                          DebugMsg(SPeepholeOptimization + 'MovMov2Mov 3 done',p);
                          asml.remove(hp1);
                          hp1.free;
                          Result:=true;
                          Exit;
                        end;
                    else
                      ;
                  end;
              end;
            if (taicpu(hp1).oper[0]^.typ = taicpu(p).oper[1]^.typ) and
               (taicpu(hp1).oper[1]^.typ = taicpu(p).oper[0]^.typ) then
                {  mov reg1, mem1     or     mov mem1, reg1
                   mov mem2, reg2            mov reg2, mem2}
              begin
                if OpsEqual(taicpu(hp1).oper[1]^,taicpu(p).oper[0]^) then
                  { mov reg1, mem1     or     mov mem1, reg1
                    mov mem2, reg1            mov reg2, mem1}
                  begin
                    if OpsEqual(taicpu(hp1).oper[0]^,taicpu(p).oper[1]^) then
                      { Removes the second statement from
                        mov reg1, mem1/reg2
                        mov mem1/reg2, reg1 }
                      begin
                        if taicpu(p).oper[0]^.typ=top_reg then
                          AllocRegBetween(taicpu(p).oper[0]^.reg,p,hp1,usedregs);
                        DebugMsg(SPeepholeOptimization + 'MovMov2Mov 1',p);
                        asml.remove(hp1);
                        hp1.free;
                        Result:=true;
                        exit;
                      end
                    else
                      begin
                        TransferUsedRegs(TmpUsedRegs);
                        UpdateUsedRegs(TmpUsedRegs, tai(hp1.next));
                        if (taicpu(p).oper[1]^.typ = top_ref) and
                          { mov reg1, mem1
                            mov mem2, reg1 }
                           (taicpu(hp1).oper[0]^.ref^.refaddr = addr_no) and
                           GetNextInstruction(hp1, hp2) and
                           MatchInstruction(hp2,A_CMP,[taicpu(p).opsize]) and
                           OpsEqual(taicpu(p).oper[1]^,taicpu(hp2).oper[0]^) and
                           OpsEqual(taicpu(p).oper[0]^,taicpu(hp2).oper[1]^) and
                           not(RegUsedAfterInstruction(taicpu(p).oper[0]^.reg, hp2, TmpUsedRegs)) then
                           { change                   to
                             mov reg1, mem1           mov reg1, mem1
                             mov mem2, reg1           cmp reg1, mem2
                             cmp mem1, reg1
                           }
                          begin
                            asml.remove(hp2);
                            hp2.free;
                            taicpu(hp1).opcode := A_CMP;
                            taicpu(hp1).loadref(1,taicpu(hp1).oper[0]^.ref^);
                            taicpu(hp1).loadreg(0,taicpu(p).oper[0]^.reg);
                            AllocRegBetween(taicpu(p).oper[0]^.reg,p,hp1,UsedRegs);
                            DebugMsg(SPeepholeOptimization + 'MovMovCmp2MovCmp done',hp1);
                          end;
                      end;
                  end
                else if (taicpu(p).oper[1]^.typ=top_ref) and
                  OpsEqual(taicpu(hp1).oper[0]^,taicpu(p).oper[1]^) then
                  begin
                    AllocRegBetween(taicpu(p).oper[0]^.reg,p,hp1,UsedRegs);
                    taicpu(hp1).loadreg(0,taicpu(p).oper[0]^.reg);
                    DebugMsg(SPeepholeOptimization + 'MovMov2MovMov1 done',p);
                  end
                else
                  begin
                    TransferUsedRegs(TmpUsedRegs);
                    if GetNextInstruction(hp1, hp2) and
                      MatchOpType(taicpu(p),top_ref,top_reg) and
                      MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[0]^) and
                      (taicpu(hp1).oper[1]^.typ = top_ref) and
                      MatchInstruction(hp2,A_MOV,[taicpu(p).opsize]) and
                      MatchOpType(taicpu(hp2),top_ref,top_reg) and
                      RefsEqual(taicpu(hp2).oper[0]^.ref^, taicpu(hp1).oper[1]^.ref^)  then
                      if not RegInRef(taicpu(hp2).oper[1]^.reg,taicpu(hp2).oper[0]^.ref^) and
                         not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp1,tmpUsedRegs)) then
                        {   mov mem1, %reg1
                            mov %reg1, mem2
                            mov mem2, reg2
                         to:
                            mov mem1, reg2
                            mov reg2, mem2}
                        begin
                          AllocRegBetween(taicpu(hp2).oper[1]^.reg,p,hp2,usedregs);
                          DebugMsg(SPeepholeOptimization + 'MovMovMov2MovMov 1 done',p);
                          taicpu(p).loadoper(1,taicpu(hp2).oper[1]^);
                          taicpu(hp1).loadoper(0,taicpu(hp2).oper[1]^);
                          asml.remove(hp2);
                          hp2.free;
                        end
{$ifdef i386}
                      { this is enabled for i386 only, as the rules to create the reg sets below
                        are too complicated for x86-64, so this makes this code too error prone
                        on x86-64
                      }
                      else if (taicpu(p).oper[1]^.reg <> taicpu(hp2).oper[1]^.reg) and
                        not(RegInRef(taicpu(p).oper[1]^.reg,taicpu(p).oper[0]^.ref^)) and
                        not(RegInRef(taicpu(hp2).oper[1]^.reg,taicpu(hp2).oper[0]^.ref^)) then
                        {   mov mem1, reg1         mov mem1, reg1
                            mov reg1, mem2         mov reg1, mem2
                            mov mem2, reg2         mov mem2, reg1
                         to:                    to:
                            mov mem1, reg1         mov mem1, reg1
                            mov mem1, reg2         mov reg1, mem2
                            mov reg1, mem2

                         or (if mem1 depends on reg1
                      and/or if mem2 depends on reg2)
                         to:
                             mov mem1, reg1
                             mov reg1, mem2
                             mov reg1, reg2
                        }
                        begin
                          taicpu(hp1).loadRef(0,taicpu(p).oper[0]^.ref^);
                          taicpu(hp1).loadReg(1,taicpu(hp2).oper[1]^.reg);
                          taicpu(hp2).loadRef(1,taicpu(hp2).oper[0]^.ref^);
                          taicpu(hp2).loadReg(0,taicpu(p).oper[1]^.reg);
                          AllocRegBetween(taicpu(p).oper[1]^.reg,p,hp2,usedregs);
                          if (taicpu(p).oper[0]^.ref^.base <> NR_NO) and
                             (getsupreg(taicpu(p).oper[0]^.ref^.base) in [RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESI,RS_EDI]) then
                            AllocRegBetween(taicpu(p).oper[0]^.ref^.base,p,hp2,usedregs);
                          if (taicpu(p).oper[0]^.ref^.index <> NR_NO) and
                             (getsupreg(taicpu(p).oper[0]^.ref^.index) in [RS_EAX,RS_EBX,RS_ECX,RS_EDX,RS_ESI,RS_EDI]) then
                            AllocRegBetween(taicpu(p).oper[0]^.ref^.index,p,hp2,usedregs);
                        end
                      else if (taicpu(hp1).Oper[0]^.reg <> taicpu(hp2).Oper[1]^.reg) then
                        begin
                          taicpu(hp2).loadReg(0,taicpu(hp1).Oper[0]^.reg);
                          AllocRegBetween(taicpu(p).oper[1]^.reg,p,hp2,usedregs);
                        end
                      else
                        begin
                          asml.remove(hp2);
                          hp2.free;
                        end
{$endif i386}
                        ;
                  end;
              end;
(*          { movl [mem1],reg1
              movl [mem1],reg2

              to

              movl [mem1],reg1
              movl reg1,reg2
             }
             else if (taicpu(p).oper[0]^.typ = top_ref) and
                (taicpu(p).oper[1]^.typ = top_reg) and
                (taicpu(hp1).oper[0]^.typ = top_ref) and
                (taicpu(hp1).oper[1]^.typ = top_reg) and
                (taicpu(p).opsize = taicpu(hp1).opsize) and
                RefsEqual(TReference(taicpu(p).oper[0]^^),taicpu(hp1).oper[0]^^.ref^) and
                (taicpu(p).oper[1]^.reg<>taicpu(hp1).oper[0]^^.ref^.base) and
                (taicpu(p).oper[1]^.reg<>taicpu(hp1).oper[0]^^.ref^.index) then
                taicpu(hp1).loadReg(0,taicpu(p).oper[1]^.reg)
              else*)

            {   movl const1,[mem1]
                movl [mem1],reg1

                to

                movl const1,reg1
                movl reg1,[mem1]
            }
            if MatchOpType(Taicpu(p),top_const,top_ref) and
                 MatchOpType(Taicpu(hp1),top_ref,top_reg) and
                 (taicpu(p).opsize = taicpu(hp1).opsize) and
                 RefsEqual(taicpu(hp1).oper[0]^.ref^,taicpu(p).oper[1]^.ref^) and
                 not(RegInRef(taicpu(hp1).oper[1]^.reg,taicpu(hp1).oper[0]^.ref^)) then
              begin
                AllocRegBetween(taicpu(hp1).oper[1]^.reg,p,hp1,usedregs);
                taicpu(hp1).loadReg(0,taicpu(hp1).oper[1]^.reg);
                taicpu(hp1).loadRef(1,taicpu(p).oper[1]^.ref^);
                taicpu(p).loadReg(1,taicpu(hp1).oper[0]^.reg);
                taicpu(hp1).fileinfo := taicpu(p).fileinfo;
                DebugMsg(SPeepholeOptimization + 'MovMov2MovMov 1',p);
                Result:=true;
                exit;
              end;
            {
              mov*  x,reg1
              mov*  y,reg1

              to

              mov*  y,reg1
            }
            if (taicpu(p).oper[1]^.typ=top_reg) and
              MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[1]^) and
              not(RegInOp(taicpu(p).oper[1]^.reg,taicpu(hp1).oper[0]^)) then
              begin
                DebugMsg(SPeepholeOptimization + 'MovMov2Mov 4 done',p);
                { take care of the register (de)allocs following p }
                UpdateUsedRegs(tai(p.next));
                asml.remove(p);
                p.free;
                p:=hp1;
                Result:=true;
                exit;
              end;
          end;
        { search further than the next instruction for a mov }
        if (cs_opt_level3 in current_settings.optimizerswitches) and
          { check as much as possible before the expensive GetNextInstructionUsingReg call }
          (taicpu(p).oper[1]^.typ = top_reg) and
          (taicpu(p).oper[0]^.typ in [top_reg,top_const]) and
          { we work with hp2 here, so hp1 can be still used later on when
            checking for GetNextInstruction_p }
          GetNextInstructionUsingReg(p,hp2,taicpu(p).oper[1]^.reg) and
          MatchInstruction(hp2,A_MOV,[]) and
          MatchOperand(taicpu(p).oper[1]^,taicpu(hp2).oper[0]^) and
          ((taicpu(p).oper[0]^.typ=top_const) or
           ((taicpu(p).oper[0]^.typ=top_reg) and
            not(RegUsedBetween(taicpu(p).oper[0]^.reg, p, hp2))
           )
          ) then
          begin
            TransferUsedRegs(TmpUsedRegs);
            { we have
                mov x, %treg
                mov %treg, y
            }
            if not(RegInOp(taicpu(p).oper[1]^.reg,taicpu(hp2).oper[1]^)) and
               not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg, hp2, TmpUsedRegs)) then
              { we've got

                mov x, %treg
                mov %treg, y

                with %treg is not used after }
              case taicpu(p).oper[0]^.typ Of
                top_reg:
                  begin
                    { change
                        mov %reg, %treg
                        mov %treg, y

                        to

                        mov %reg, y
                    }
                    AllocRegBetween(taicpu(p).oper[0]^.reg,p,hp2,usedregs);
                    taicpu(hp2).loadOper(0,taicpu(p).oper[0]^);
                    DebugMsg(SPeepholeOptimization + 'MovMov2Mov 6 done',p);
                    { take care of the register (de)allocs following p }
                    UpdateUsedRegs(tai(p.next));
                    asml.remove(p);
                    p.free;
                    p:=hp1;
                    Result:=true;
                    Exit;
                  end;
                top_const:
                  begin
                    { change
                        mov const, %treg
                        mov %treg, y

                        to

                        mov const, y
                    }
                    if (taicpu(hp2).oper[1]^.typ=top_reg) or
                      ((taicpu(p).oper[0]^.val>=low(longint)) and (taicpu(p).oper[0]^.val<=high(longint))) then
                      begin
                        taicpu(hp2).loadOper(0,taicpu(p).oper[0]^);
                        DebugMsg(SPeepholeOptimization + 'MovMov2Mov 7 done',p);
                        { take care of the register (de)allocs following p }
                        UpdateUsedRegs(tai(p.next));
                        asml.remove(p);
                        p.free;
                        p:=hp1;
                        Result:=true;
                        Exit;
                      end;
                  end;
                else
                  Internalerror(2019103001);
              end;
          end;
        { Change
           mov %reg1, %reg2
           xxx %reg2, ???

           to

           mov %reg1, %reg2
           xxx %reg1, ???

           to avoid a write/read penalty
        }
        if GetNextInstruction_p and
           MatchOpType(taicpu(p),top_reg,top_reg) and
           MatchInstruction(hp1,A_OR,A_AND,A_TEST,[]) and
           MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[0]^) and
           (taicpu(hp1).oper[1]^.typ = top_reg) and
           (taicpu(hp1).oper[0]^.reg = taicpu(hp1).oper[1]^.reg) then
           {  we have

              mov %reg1, %reg2
              test/or/and %reg2, %reg2
           }
           begin
             TransferUsedRegs(TmpUsedRegs);
             { reg1 will be used after the first instruction,
               so update the allocation info                  }
             AllocRegBetween(taicpu(p).oper[0]^.reg,p,hp1,usedregs);
             if GetNextInstruction(hp1, hp2) and
                (hp2.typ = ait_instruction) and
                taicpu(hp2).is_jmp and
                not(RegUsedAfterInstruction(taicpu(hp1).oper[0]^.reg, hp1, TmpUsedRegs)) then
                 { change

                   mov %reg1, %reg2
                   test/or/and %reg2, %reg2
                   jxx

                   to

                   test %reg1, %reg1
                   jxx
                 }
                 begin
                   taicpu(hp1).loadoper(0,taicpu(p).oper[0]^);
                   taicpu(hp1).loadoper(1,taicpu(p).oper[0]^);
                   DebugMsg(SPeepholeOptimization + 'MovTestJxx2TestMov done',p);
                   asml.remove(p);
                   p.free;
                   p := hp1;
                   Exit;
                 end
               else
                 { change

                   mov %reg1, %reg2
                   test/or/and %reg2, %reg2

                   to

                   mov %reg1, %reg2
                   test/or/and %reg1, %reg1

                   }
                 begin
                   taicpu(hp1).loadoper(0,taicpu(p).oper[0]^);
                   taicpu(hp1).loadoper(1,taicpu(p).oper[0]^);
                   DebugMsg(SPeepholeOptimization + 'MovTestJxx2MovTestJxx done',p);
                 end;
           end;
        { leave out the mov from "mov reg, x(%frame_pointer); leave/ret" (with
          x >= RetOffset) as it doesn't do anything (it writes either to a
          parameter or to the temporary storage room for the function
          result)
        }
        if GetNextInstruction_p and
          IsExitCode(hp1) and
          MatchOpType(taicpu(p),top_reg,top_ref) and
          (taicpu(p).oper[1]^.ref^.base = current_procinfo.FramePointer) and
          not(assigned(current_procinfo.procdef.funcretsym) and
             (taicpu(p).oper[1]^.ref^.offset < tabstractnormalvarsym(current_procinfo.procdef.funcretsym).localloc.reference.offset)) and
          (taicpu(p).oper[1]^.ref^.index = NR_NO) then
          begin
            asml.remove(p);
            p.free;
            p:=hp1;
            DebugMsg(SPeepholeOptimization + 'removed deadstore before leave/ret',p);
            RemoveLastDeallocForFuncRes(p);
            Result:=true;
            exit;
          end;
        if GetNextInstruction_p and
          MatchOpType(taicpu(p),top_reg,top_ref) and
          MatchInstruction(hp1,A_CMP,A_TEST,[taicpu(p).opsize]) and
          (taicpu(hp1).oper[1]^.typ = top_ref) and
           RefsEqual(taicpu(p).oper[1]^.ref^, taicpu(hp1).oper[1]^.ref^) then
          begin
            { change
                mov reg1, mem1
                test/cmp x, mem1

                to

                mov reg1, mem1
                test/cmp x, reg1
            }
            taicpu(hp1).loadreg(1,taicpu(p).oper[0]^.reg);
            DebugMsg(SPeepholeOptimization + 'MovTestCmp2MovTestCmp 1',hp1);
            AllocRegBetween(taicpu(p).oper[0]^.reg,p,hp1,usedregs);
            exit;
          end;
        if GetNextInstruction_p and
          (taicpu(p).oper[1]^.typ = top_reg) and
          (hp1.typ = ait_instruction) and
          GetNextInstruction(hp1, hp2) and
          MatchInstruction(hp2,A_MOV,[]) and
          (SuperRegistersEqual(taicpu(hp2).oper[0]^.reg,taicpu(p).oper[1]^.reg)) and
          (IsFoldableArithOp(taicpu(hp1), taicpu(p).oper[1]^.reg) or
           ((taicpu(p).opsize=S_L) and (taicpu(hp1).opsize=S_Q) and (taicpu(hp2).opsize=S_L) and
            IsFoldableArithOp(taicpu(hp1), newreg(R_INTREGISTER,getsupreg(taicpu(p).oper[1]^.reg),R_SUBQ)))
          ) then
          begin
            if OpsEqual(taicpu(hp2).oper[1]^, taicpu(p).oper[0]^) and
              (taicpu(hp2).oper[0]^.typ=top_reg) then
              { change   movsX/movzX    reg/ref, reg2
                         add/sub/or/... reg3/$const, reg2
                         mov            reg2 reg/ref
                         dealloc        reg2
                to
                         add/sub/or/... reg3/$const, reg/ref      }
              begin
                TransferUsedRegs(TmpUsedRegs);
                UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                UpdateUsedRegs(TmpUsedRegs, tai(hp1.next));
                If not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp2,TmpUsedRegs)) then
                  begin
                    { by example:
                        movswl  %si,%eax        movswl  %si,%eax      p
                        decl    %eax            addl    %edx,%eax     hp1
                        movw    %ax,%si         movw    %ax,%si       hp2
                      ->
                        movswl  %si,%eax        movswl  %si,%eax      p
                        decw    %eax            addw    %edx,%eax     hp1
                        movw    %ax,%si         movw    %ax,%si       hp2
                    }
                    DebugMsg(SPeepholeOptimization + 'MovOpMov2Op ('+
                          debug_op2str(taicpu(p).opcode)+debug_opsize2str(taicpu(p).opsize)+' '+
                          debug_op2str(taicpu(hp1).opcode)+debug_opsize2str(taicpu(hp1).opsize)+' '+
                          debug_op2str(taicpu(hp2).opcode)+debug_opsize2str(taicpu(hp2).opsize),p);
                    taicpu(hp1).changeopsize(taicpu(hp2).opsize);
                    {
                      ->
                        movswl  %si,%eax        movswl  %si,%eax      p
                        decw    %si             addw    %dx,%si       hp1
                        movw    %ax,%si         movw    %ax,%si       hp2
                    }
                    case taicpu(hp1).ops of
                      1:
                        begin
                          taicpu(hp1).loadoper(0, taicpu(hp2).oper[1]^);
                          if taicpu(hp1).oper[0]^.typ=top_reg then
                            setsubreg(taicpu(hp1).oper[0]^.reg,getsubreg(taicpu(hp2).oper[0]^.reg));
                        end;
                      2:
                        begin
                          taicpu(hp1).loadoper(1, taicpu(hp2).oper[1]^);
                          if (taicpu(hp1).oper[0]^.typ=top_reg) and
                            (taicpu(hp1).opcode<>A_SHL) and
                            (taicpu(hp1).opcode<>A_SHR) and
                            (taicpu(hp1).opcode<>A_SAR) then
                            setsubreg(taicpu(hp1).oper[0]^.reg,getsubreg(taicpu(hp2).oper[0]^.reg));
                        end;
                      else
                        internalerror(2008042701);
                    end;
                    {
                      ->
                        decw    %si             addw    %dx,%si       p
                    }
                    asml.remove(p);
                    asml.remove(hp2);
                    p.Free;
                    hp2.Free;
                    p := hp1;
                  end;
              end;
            if MatchOpType(taicpu(hp2),top_reg,top_reg) and
              not(SuperRegistersEqual(taicpu(hp1).oper[0]^.reg,taicpu(hp2).oper[1]^.reg)) and
              ((topsize2memsize[taicpu(hp1).opsize]<= topsize2memsize[taicpu(hp2).opsize]) or
               { opsize matters for these opcodes, we could probably work around this, but it is not worth the effort }
               ((taicpu(hp1).opcode<>A_SHL) and (taicpu(hp1).opcode<>A_SHR) and (taicpu(hp1).opcode<>A_SAR))
              )
{$ifdef i386}
              { byte registers of esi, edi, ebp, esp are not available on i386 }
              and ((taicpu(hp2).opsize<>S_B) or not(getsupreg(taicpu(hp1).oper[0]^.reg) in [RS_ESI,RS_EDI,RS_EBP,RS_ESP]))
              and ((taicpu(hp2).opsize<>S_B) or not(getsupreg(taicpu(p).oper[0]^.reg) in [RS_ESI,RS_EDI,RS_EBP,RS_ESP]))
{$endif i386}
              then
              { change   movsX/movzX    reg/ref, reg2
                         add/sub/or/... regX/$const, reg2
                         mov            reg2, reg3
                         dealloc        reg2
                to
                         movsX/movzX    reg/ref, reg3
                         add/sub/or/... reg3/$const, reg3
              }
              begin
                TransferUsedRegs(TmpUsedRegs);
                UpdateUsedRegs(TmpUsedRegs, tai(p.next));
                UpdateUsedRegs(TmpUsedRegs, tai(hp1.next));
                If not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp2,TmpUsedRegs)) then
                  begin
                    { by example:
                        movswl  %si,%eax        movswl  %si,%eax      p
                        decl    %eax            addl    %edx,%eax     hp1
                        movw    %ax,%si         movw    %ax,%si       hp2
                      ->
                        movswl  %si,%eax        movswl  %si,%eax      p
                        decw    %eax            addw    %edx,%eax     hp1
                        movw    %ax,%si         movw    %ax,%si       hp2
                    }
                    DebugMsg(SPeepholeOptimization + 'MovOpMov2MovOp ('+
                          debug_op2str(taicpu(p).opcode)+debug_opsize2str(taicpu(p).opsize)+' '+
                          debug_op2str(taicpu(hp1).opcode)+debug_opsize2str(taicpu(hp1).opsize)+' '+
                          debug_op2str(taicpu(hp2).opcode)+debug_opsize2str(taicpu(hp2).opsize)+')',p);
                    { limit size of constants as well to avoid assembler errors, but
                      check opsize to avoid overflow when left shifting the 1 }
                    if (taicpu(p).oper[0]^.typ=top_const) and (topsize2memsize[taicpu(hp2).opsize]<=4) then
                      taicpu(p).oper[0]^.val:=taicpu(p).oper[0]^.val and ((qword(1) shl (topsize2memsize[taicpu(hp2).opsize]*8))-1);
                    taicpu(hp1).changeopsize(taicpu(hp2).opsize);
                    taicpu(p).changeopsize(taicpu(hp2).opsize);
                    if taicpu(p).oper[0]^.typ=top_reg then
                      setsubreg(taicpu(p).oper[0]^.reg,getsubreg(taicpu(hp2).oper[0]^.reg));
                    taicpu(p).loadoper(1, taicpu(hp2).oper[1]^);
                    AllocRegBetween(taicpu(p).oper[1]^.reg,p,hp1,usedregs);
                    {
                      ->
                        movswl  %si,%eax        movswl  %si,%eax      p
                        decw    %si             addw    %dx,%si       hp1
                        movw    %ax,%si         movw    %ax,%si       hp2
                    }
                    case taicpu(hp1).ops of
                      1:
                        begin
                          taicpu(hp1).loadoper(0, taicpu(hp2).oper[1]^);
                          if taicpu(hp1).oper[0]^.typ=top_reg then
                            setsubreg(taicpu(hp1).oper[0]^.reg,getsubreg(taicpu(hp2).oper[0]^.reg));
                        end;
                      2:
                        begin
                          taicpu(hp1).loadoper(1, taicpu(hp2).oper[1]^);
                          if (taicpu(hp1).oper[0]^.typ=top_reg) and
                            (taicpu(hp1).opcode<>A_SHL) and
                            (taicpu(hp1).opcode<>A_SHR) and
                            (taicpu(hp1).opcode<>A_SAR) then
                            setsubreg(taicpu(hp1).oper[0]^.reg,getsubreg(taicpu(hp2).oper[0]^.reg));
                        end;
                      else
                        internalerror(2018111801);
                    end;
                    {
                      ->
                        decw    %si             addw    %dx,%si       p
                    }
                    asml.remove(hp2);
                    hp2.Free;
                  end;
              end;
          end;
        if GetNextInstruction_p and
          MatchInstruction(hp1,A_BTS,A_BTR,[Taicpu(p).opsize]) and
          GetNextInstruction(hp1, hp2) and
          MatchInstruction(hp2,A_OR,[Taicpu(p).opsize]) and
          MatchOperand(Taicpu(p).oper[0]^,0) and
          (Taicpu(p).oper[1]^.typ = top_reg) and
          MatchOperand(Taicpu(p).oper[1]^,Taicpu(hp1).oper[1]^) and
          MatchOperand(Taicpu(p).oper[1]^,Taicpu(hp2).oper[1]^) then
          { mov reg1,0
            bts reg1,operand1             -->      mov reg1,operand2
            or  reg1,operand2                      bts reg1,operand1}
          begin
            Taicpu(hp2).opcode:=A_MOV;
            asml.remove(hp1);
            insertllitem(hp2,hp2.next,hp1);
            asml.remove(p);
            p.free;
            p:=hp1;
            Result:=true;
            exit;
          end;

        if GetNextInstruction_p and
           MatchInstruction(hp1,A_LEA,[S_L]) and
           MatchOpType(Taicpu(p),top_ref,top_reg) and
           ((MatchReference(Taicpu(hp1).oper[0]^.ref^,Taicpu(hp1).oper[1]^.reg,Taicpu(p).oper[1]^.reg) and
             (Taicpu(hp1).oper[0]^.ref^.base<>Taicpu(p).oper[1]^.reg)
            ) or
            (MatchReference(Taicpu(hp1).oper[0]^.ref^,Taicpu(p).oper[1]^.reg,Taicpu(hp1).oper[1]^.reg) and
             (Taicpu(hp1).oper[0]^.ref^.index<>Taicpu(p).oper[1]^.reg)
            )
           ) then
           { mov reg1,ref
             lea reg2,[reg1,reg2]

             to

             add reg2,ref}
          begin
            TransferUsedRegs(TmpUsedRegs);
            { reg1 may not be used afterwards }
            if not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg, hp1, TmpUsedRegs)) then
              begin
                Taicpu(hp1).opcode:=A_ADD;
                Taicpu(hp1).oper[0]^.ref^:=Taicpu(p).oper[0]^.ref^;
                DebugMsg(SPeepholeOptimization + 'MovLea2Add done',hp1);
                asml.remove(p);
                p.free;
                p:=hp1;
                result:=true;
                exit;
              end;
          end;
      end;


   function TX86AsmOptimizer.OptPass1MOVXX(var p : tai) : boolean;
      var
        hp1 : tai;
      begin
        Result:=false;
        if taicpu(p).ops <> 2 then
          exit;
        if GetNextInstruction(p,hp1) and
          MatchInstruction(hp1,taicpu(p).opcode,[taicpu(p).opsize]) and
          (taicpu(hp1).ops = 2) then
          begin
            if (taicpu(hp1).oper[0]^.typ = taicpu(p).oper[1]^.typ) and
               (taicpu(hp1).oper[1]^.typ = taicpu(p).oper[0]^.typ) then
                {  movXX reg1, mem1     or     movXX mem1, reg1
                   movXX mem2, reg2            movXX reg2, mem2}
              begin
                if OpsEqual(taicpu(hp1).oper[1]^,taicpu(p).oper[0]^) then
                  { movXX reg1, mem1     or     movXX mem1, reg1
                    movXX mem2, reg1            movXX reg2, mem1}
                  begin
                    if OpsEqual(taicpu(hp1).oper[0]^,taicpu(p).oper[1]^) then
                      begin
                        { Removes the second statement from
                          movXX reg1, mem1/reg2
                          movXX mem1/reg2, reg1
                        }
                        if taicpu(p).oper[0]^.typ=top_reg then
                          AllocRegBetween(taicpu(p).oper[0]^.reg,p,hp1,usedregs);
                        { Removes the second statement from
                          movXX mem1/reg1, reg2
                          movXX reg2, mem1/reg1
                        }
                        if (taicpu(p).oper[1]^.typ=top_reg) and
                          not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp1,UsedRegs)) then
                          begin
                            asml.remove(p);
                            p.free;
                            GetNextInstruction(hp1,p);
                            DebugMsg(SPeepholeOptimization + 'MovXXMovXX2Nop 1 done',p);
                          end
                        else
                          DebugMsg(SPeepholeOptimization + 'MovXXMovXX2MoVXX 1 done',p);
                        asml.remove(hp1);
                        hp1.free;
                        Result:=true;
                        exit;
                      end
                end;
            end;
        end;
      end;


    function TX86AsmOptimizer.OptPass1OP(var p : tai) : boolean;
      var
        hp1 : tai;
      begin
        result:=false;
        { replace
            <Op>X    %mreg1,%mreg2  // Op in [ADD,MUL]
            MovX     %mreg2,%mreg1
            dealloc  %mreg2

            by
            <Op>X    %mreg2,%mreg1
          ?
        }
        if GetNextInstruction(p,hp1) and
          { we mix single and double opperations here because we assume that the compiler
            generates vmovapd only after double operations and vmovaps only after single operations }
          MatchInstruction(hp1,A_MOVAPD,A_MOVAPS,[S_NO]) and
          MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[0]^) and
          MatchOperand(taicpu(p).oper[0]^,taicpu(hp1).oper[1]^) and
          (taicpu(p).oper[0]^.typ=top_reg) then
          begin
            TransferUsedRegs(TmpUsedRegs);
            UpdateUsedRegs(TmpUsedRegs, tai(p.next));
            if not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp1,TmpUsedRegs)) then
              begin
                taicpu(p).loadoper(0,taicpu(hp1).oper[0]^);
                taicpu(p).loadoper(1,taicpu(hp1).oper[1]^);
                DebugMsg(SPeepholeOptimization + 'OpMov2Op done',p);
                asml.Remove(hp1);
                hp1.Free;
                result:=true;
              end;
          end;
      end;


    function TX86AsmOptimizer.OptPass1LEA(var p : tai) : boolean;
      var
        hp1, hp2, hp3: tai;
        l : ASizeInt;
      begin
        Result:=false;
        { removes seg register prefixes from LEA operations, as they
          don't do anything}
        taicpu(p).oper[0]^.ref^.Segment:=NR_NO;
        { changes "lea (%reg1), %reg2" into "mov %reg1, %reg2" }
        if (taicpu(p).oper[0]^.ref^.base <> NR_NO) and
           (taicpu(p).oper[0]^.ref^.index = NR_NO) and
           { do not mess with leas acessing the stack pointer }
           (taicpu(p).oper[1]^.reg <> NR_STACK_POINTER_REG) and
           (not(Assigned(taicpu(p).oper[0]^.ref^.Symbol))) then
          begin
            if (taicpu(p).oper[0]^.ref^.base <> taicpu(p).oper[1]^.reg) and
               (taicpu(p).oper[0]^.ref^.offset = 0) then
              begin
                hp1:=taicpu.op_reg_reg(A_MOV,taicpu(p).opsize,taicpu(p).oper[0]^.ref^.base,
                  taicpu(p).oper[1]^.reg);
                InsertLLItem(p.previous,p.next, hp1);
                DebugMsg(SPeepholeOptimization + 'Lea2Mov done',hp1);
                p.free;
                p:=hp1;
                Result:=true;
                exit;
              end
            else if (taicpu(p).oper[0]^.ref^.offset = 0) then
              begin
                hp1:=taicpu(p.Next);
                DebugMsg(SPeepholeOptimization + 'Lea2Nop done',p);
                asml.remove(p);
                p.free;
                p:=hp1;
                Result:=true;
                exit;
              end
            { continue to use lea to adjust the stack pointer,
              it is the recommended way, but only if not optimizing for size }
            else if (taicpu(p).oper[1]^.reg<>NR_STACK_POINTER_REG) or
              (cs_opt_size in current_settings.optimizerswitches) then
              with taicpu(p).oper[0]^.ref^ do
                if (base = taicpu(p).oper[1]^.reg) then
                  begin
                    l:=offset;
                    if (l=1) and UseIncDec then
                      begin
                        taicpu(p).opcode:=A_INC;
                        taicpu(p).loadreg(0,taicpu(p).oper[1]^.reg);
                        taicpu(p).ops:=1;
                        DebugMsg(SPeepholeOptimization + 'Lea2Inc done',p);
                      end
                    else if (l=-1) and UseIncDec then
                      begin
                        taicpu(p).opcode:=A_DEC;
                        taicpu(p).loadreg(0,taicpu(p).oper[1]^.reg);
                        taicpu(p).ops:=1;
                        DebugMsg(SPeepholeOptimization + 'Lea2Dec done',p);
                      end
                    else
                      begin
                        if (l<0) and (l<>-2147483648) then
                          begin
                            taicpu(p).opcode:=A_SUB;
                            taicpu(p).loadConst(0,-l);
                            DebugMsg(SPeepholeOptimization + 'Lea2Sub done',p);
                          end
                        else
                          begin
                            taicpu(p).opcode:=A_ADD;
                            taicpu(p).loadConst(0,l);
                            DebugMsg(SPeepholeOptimization + 'Lea2Add done',p);
                          end;
                      end;
                    Result:=true;
                    exit;
                  end;
          end;
        if GetNextInstruction(p,hp1) and
          MatchInstruction(hp1,A_MOV,[taicpu(p).opsize]) and
          MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[0]^) and
          MatchOpType(Taicpu(hp1),top_reg,top_reg) and
          (taicpu(p).oper[1]^.reg<>NR_STACK_POINTER_REG) then
          begin
            TransferUsedRegs(TmpUsedRegs);
            UpdateUsedRegs(TmpUsedRegs, tai(p.next));
            if not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp1,TmpUsedRegs)) then
              begin
                taicpu(p).loadoper(1,taicpu(hp1).oper[1]^);
                DebugMsg(SPeepholeOptimization + 'LeaMov2Lea done',p);
                asml.Remove(hp1);
                hp1.Free;
                result:=true;
              end;
          end;
        { changes
            lea offset1(regX), reg1
            lea offset2(reg1), reg1
            to
            lea offset1+offset2(regX), reg1 }
        if GetNextInstructionUsingReg(p,hp1,taicpu(p).oper[1]^.reg) and
          MatchInstruction(hp1,A_LEA,[S_L]) and
          MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[1]^) and
          (taicpu(hp1).oper[0]^.ref^.base=taicpu(p).oper[1]^.reg) and
          (taicpu(p).oper[0]^.ref^.index=NR_NO) and
          (taicpu(p).oper[0]^.ref^.relsymbol=nil) and
          (taicpu(p).oper[0]^.ref^.scalefactor in [0,1]) and
          (taicpu(p).oper[0]^.ref^.segment=NR_NO) and
          (taicpu(p).oper[0]^.ref^.symbol=nil) and
          (taicpu(p).oper[0]^.ref^.index=taicpu(hp1).oper[0]^.ref^.index) and
          (taicpu(p).oper[0]^.ref^.relsymbol=taicpu(hp1).oper[0]^.ref^.relsymbol) and
          (taicpu(p).oper[0]^.ref^.scalefactor=taicpu(hp1).oper[0]^.ref^.scalefactor) and
          (taicpu(p).oper[0]^.ref^.segment=taicpu(hp1).oper[0]^.ref^.segment) and
          (taicpu(p).oper[0]^.ref^.symbol=taicpu(hp1).oper[0]^.ref^.symbol) then
          begin
            DebugMsg(SPeepholeOptimization + 'LeaLea2Lea done',p);
            inc(taicpu(hp1).oper[0]^.ref^.offset,taicpu(p).oper[0]^.ref^.offset);
            taicpu(hp1).oper[0]^.ref^.base:=taicpu(p).oper[0]^.ref^.base;
            asml.Remove(p);
            p.Free;
            p:=hp1;
            result:=true;
          end;
        { replace
            lea    x(stackpointer),stackpointer
            call   procname
            lea    -x(stackpointer),stackpointer
            ret
          by
            jmp    procname

          this should never hurt except when pic is used, not sure
          how to handle it then

          but do it only on level 4 because it destroys stack back traces
        }
        if (cs_opt_level4 in current_settings.optimizerswitches) and
          not(cs_create_pic in current_settings.moduleswitches) and
          (taicpu(p).oper[1]^.reg=NR_STACK_POINTER_REG) and
          (taicpu(p).oper[0]^.ref^.base=NR_STACK_POINTER_REG) and
          (taicpu(p).oper[0]^.ref^.index=NR_NO) and
          (taicpu(p).oper[0]^.ref^.relsymbol=nil) and
          (taicpu(p).oper[0]^.ref^.scalefactor in [0,1]) and
          (taicpu(p).oper[0]^.ref^.segment=NR_NO) and
          (taicpu(p).oper[0]^.ref^.symbol=nil) and
          GetNextInstruction(p, hp1) and
          MatchInstruction(hp1,A_CALL,[S_NO]) and
          GetNextInstruction(hp1, hp2) and
          MatchInstruction(hp2,A_LEA,[taicpu(p).opsize]) and
          (taicpu(hp2).oper[1]^.reg=NR_STACK_POINTER_REG) and
          (taicpu(p).oper[0]^.ref^.base=taicpu(hp2).oper[0]^.ref^.base) and
          (taicpu(p).oper[0]^.ref^.index=taicpu(hp2).oper[0]^.ref^.index) and
          (taicpu(p).oper[0]^.ref^.offset=-taicpu(hp2).oper[0]^.ref^.offset) and
          (taicpu(p).oper[0]^.ref^.relsymbol=taicpu(hp2).oper[0]^.ref^.relsymbol) and
          (taicpu(p).oper[0]^.ref^.scalefactor=taicpu(hp2).oper[0]^.ref^.scalefactor) and
          (taicpu(p).oper[0]^.ref^.segment=taicpu(hp2).oper[0]^.ref^.segment) and
          (taicpu(p).oper[0]^.ref^.symbol=taicpu(hp2).oper[0]^.ref^.symbol) and
          GetNextInstruction(hp2, hp3) and
          MatchInstruction(hp3,A_RET,[S_NO]) and
          (taicpu(hp3).ops=0) then
          begin
            DebugMsg(SPeepholeOptimization + 'LeaCallLeaRet2Jmp done',p);
            taicpu(hp1).opcode:=A_JMP;
            taicpu(hp1).is_jmp:=true;
            asml.remove(p);
            asml.remove(hp2);
            asml.remove(hp3);
            p.free;
            hp2.free;
            hp3.free;
            p:=hp1;
            Result:=true;
          end;
      end;


    function TX86AsmOptimizer.DoSubAddOpt(var p: tai): Boolean;
      var
        hp1 : tai;
      begin
        DoSubAddOpt := False;
        if GetLastInstruction(p, hp1) and
           (hp1.typ = ait_instruction) and
           (taicpu(hp1).opsize = taicpu(p).opsize) then
          case taicpu(hp1).opcode Of
            A_DEC:
              if (taicpu(hp1).oper[0]^.typ = top_reg) and
                MatchOperand(taicpu(hp1).oper[0]^,taicpu(p).oper[1]^) then
                begin
                  taicpu(p).loadConst(0,taicpu(p).oper[0]^.val+1);
                  asml.remove(hp1);
                  hp1.free;
                end;
             A_SUB:
               if MatchOpType(taicpu(hp1),top_const,top_reg) and
                 MatchOperand(taicpu(hp1).oper[1]^,taicpu(p).oper[1]^) then
                 begin
                   taicpu(p).loadConst(0,taicpu(p).oper[0]^.val+taicpu(hp1).oper[0]^.val);
                   asml.remove(hp1);
                   hp1.free;
                 end;
             A_ADD:
               begin
                 if MatchOpType(taicpu(hp1),top_const,top_reg) and
                   MatchOperand(taicpu(hp1).oper[1]^,taicpu(p).oper[1]^) then
                   begin
                     taicpu(p).loadConst(0,taicpu(p).oper[0]^.val-taicpu(hp1).oper[0]^.val);
                     asml.remove(hp1);
                     hp1.free;
                     if (taicpu(p).oper[0]^.val = 0) then
                       begin
                         hp1 := tai(p.next);
                         asml.remove(p);
                         p.free;
                         if not GetLastInstruction(hp1, p) then
                           p := hp1;
                         DoSubAddOpt := True;
                       end
                   end;
               end;
             else
               ;
           end;
      end;


    function TX86AsmOptimizer.OptPass1Sub(var p : tai) : boolean;
{$ifdef i386}
      var
        hp1 : tai;
{$endif i386}
      begin
        Result:=false;
        { * change "subl $2, %esp; pushw x" to "pushl x"}
        { * change "sub/add const1, reg" or "dec reg" followed by
            "sub const2, reg" to one "sub ..., reg" }
        if MatchOpType(taicpu(p),top_const,top_reg) then
          begin
{$ifdef i386}
            if (taicpu(p).oper[0]^.val = 2) and
               (taicpu(p).oper[1]^.reg = NR_ESP) and
               { Don't do the sub/push optimization if the sub }
               { comes from setting up the stack frame (JM)    }
               (not(GetLastInstruction(p,hp1)) or
               not(MatchInstruction(hp1,A_MOV,[S_L]) and
                 MatchOperand(taicpu(hp1).oper[0]^,NR_ESP) and
                 MatchOperand(taicpu(hp1).oper[0]^,NR_EBP))) then
              begin
                hp1 := tai(p.next);
                while Assigned(hp1) and
                      (tai(hp1).typ in [ait_instruction]+SkipInstr) and
                      not RegReadByInstruction(NR_ESP,hp1) and
                      not RegModifiedByInstruction(NR_ESP,hp1) do
                  hp1 := tai(hp1.next);
                if Assigned(hp1) and
                  MatchInstruction(hp1,A_PUSH,[S_W]) then
                  begin
                    taicpu(hp1).changeopsize(S_L);
                    if taicpu(hp1).oper[0]^.typ=top_reg then
                      setsubreg(taicpu(hp1).oper[0]^.reg,R_SUBWHOLE);
                    hp1 := tai(p.next);
                    asml.remove(p);
                    p.free;
                    p := hp1;
                    Result:=true;
                    exit;
                  end;
              end;
{$endif i386}
            if DoSubAddOpt(p) then
              Result:=true;
          end;
      end;


    function TX86AsmOptimizer.OptPass1SHLSAL(var p : tai) : boolean;
      var
        TmpBool1,TmpBool2 : Boolean;
        tmpref : treference;
        hp1,hp2: tai;
      begin
        Result:=false;
        if MatchOpType(taicpu(p),top_const,top_reg) and
           (taicpu(p).opsize in [S_L{$ifdef x86_64},S_Q{$endif x86_64}]) and
           (taicpu(p).oper[0]^.val <= 3) then
          { Changes "shl const, %reg32; add const/reg, %reg32" to one lea statement }
          begin
            { should we check the next instruction? }
            TmpBool1 := True;
            { have we found an add/sub which could be
              integrated in the lea? }
            TmpBool2 := False;
            reference_reset(tmpref,2,[]);
            TmpRef.index := taicpu(p).oper[1]^.reg;
            TmpRef.scalefactor := 1 shl taicpu(p).oper[0]^.val;
            while TmpBool1 and
                  GetNextInstruction(p, hp1) and
                  (tai(hp1).typ = ait_instruction) and
                  ((((taicpu(hp1).opcode = A_ADD) or
                     (taicpu(hp1).opcode = A_SUB)) and
                    (taicpu(hp1).oper[1]^.typ = Top_Reg) and
                    (taicpu(hp1).oper[1]^.reg = taicpu(p).oper[1]^.reg)) or
                   (((taicpu(hp1).opcode = A_INC) or
                     (taicpu(hp1).opcode = A_DEC)) and
                    (taicpu(hp1).oper[0]^.typ = Top_Reg) and
                    (taicpu(hp1).oper[0]^.reg = taicpu(p).oper[1]^.reg))) and
                  (not GetNextInstruction(hp1,hp2) or
                   not instrReadsFlags(hp2)) Do
              begin
                TmpBool1 := False;
                if (taicpu(hp1).oper[0]^.typ = Top_Const) then
                  begin
                    TmpBool1 := True;
                    TmpBool2 := True;
                    case taicpu(hp1).opcode of
                      A_ADD:
                        inc(TmpRef.offset, longint(taicpu(hp1).oper[0]^.val));
                      A_SUB:
                        dec(TmpRef.offset, longint(taicpu(hp1).oper[0]^.val));
                      else
                        internalerror(2019050536);
                    end;
                    asml.remove(hp1);
                    hp1.free;
                  end
                else
                  if (taicpu(hp1).oper[0]^.typ = Top_Reg) and
                     (((taicpu(hp1).opcode = A_ADD) and
                       (TmpRef.base = NR_NO)) or
                      (taicpu(hp1).opcode = A_INC) or
                      (taicpu(hp1).opcode = A_DEC)) then
                    begin
                      TmpBool1 := True;
                      TmpBool2 := True;
                      case taicpu(hp1).opcode of
                        A_ADD:
                          TmpRef.base := taicpu(hp1).oper[0]^.reg;
                        A_INC:
                          inc(TmpRef.offset);
                        A_DEC:
                          dec(TmpRef.offset);
                        else
                          internalerror(2019050535);
                      end;
                      asml.remove(hp1);
                      hp1.free;
                    end;
              end;
            if TmpBool2
{$ifndef x86_64}
               or
               ((current_settings.optimizecputype < cpu_Pentium2) and
               (taicpu(p).oper[0]^.val <= 3) and
               not(cs_opt_size in current_settings.optimizerswitches))
{$endif x86_64}
              then
              begin
                if not(TmpBool2) and
                    (taicpu(p).oper[0]^.val = 1) then
                  begin
                    hp1 := taicpu.Op_reg_reg(A_ADD,taicpu(p).opsize,
                      taicpu(p).oper[1]^.reg, taicpu(p).oper[1]^.reg)
                  end
                else
                  hp1 := taicpu.op_ref_reg(A_LEA, taicpu(p).opsize, TmpRef,
                              taicpu(p).oper[1]^.reg);
                InsertLLItem(p.previous, p.next, hp1);
                p.free;
                p := hp1;
              end;
          end
{$ifndef x86_64}
        else if (current_settings.optimizecputype < cpu_Pentium2) and
          MatchOpType(taicpu(p),top_const,top_reg) then
          begin
            { changes "shl $1, %reg" to "add %reg, %reg", which is the same on a 386,
              but faster on a 486, and Tairable in both U and V pipes on the Pentium
              (unlike shl, which is only Tairable in the U pipe) }
            if taicpu(p).oper[0]^.val=1 then
                begin
                  hp1 := taicpu.Op_reg_reg(A_ADD,taicpu(p).opsize,
                            taicpu(p).oper[1]^.reg, taicpu(p).oper[1]^.reg);
                  InsertLLItem(p.previous, p.next, hp1);
                  p.free;
                  p := hp1;
                end
           { changes "shl $2, %reg" to "lea (,%reg,4), %reg"
             "shl $3, %reg" to "lea (,%reg,8), %reg }
           else if (taicpu(p).opsize = S_L) and
                   (taicpu(p).oper[0]^.val<= 3) then
             begin
               reference_reset(tmpref,2,[]);
               TmpRef.index := taicpu(p).oper[1]^.reg;
               TmpRef.scalefactor := 1 shl taicpu(p).oper[0]^.val;
               hp1 := taicpu.Op_ref_reg(A_LEA,S_L,TmpRef, taicpu(p).oper[1]^.reg);
               InsertLLItem(p.previous, p.next, hp1);
               p.free;
               p := hp1;
             end;
          end
{$endif x86_64}
          ;
      end;


    function TX86AsmOptimizer.OptPass1SETcc(var p: tai): boolean;
      var
        hp1,hp2,next: tai; SetC, JumpC: TAsmCond;
      begin
        Result:=false;

        if MatchOpType(taicpu(p),top_reg) and
          GetNextInstruction(p, hp1) and
          MatchInstruction(hp1, A_TEST, [S_B]) and
          MatchOpType(taicpu(hp1),top_reg,top_reg) and
          (taicpu(p).oper[0]^.reg = taicpu(hp1).oper[0]^.reg) and
          (taicpu(hp1).oper[0]^.reg = taicpu(hp1).oper[1]^.reg) and
          GetNextInstruction(hp1, hp2) and
          MatchInstruction(hp2, A_Jcc, []) then
          { Change from:             To:

            set(C) %reg              j(~C) label
            test   %reg,%reg
            je     label


            set(C) %reg              j(C)  label
            test   %reg,%reg
            jne    label
          }
          begin
            next := tai(p.Next);

            TransferUsedRegs(TmpUsedRegs);
            UpdateUsedRegs(TmpUsedRegs, next);
            UpdateUsedRegs(TmpUsedRegs, tai(hp1.next));

            asml.Remove(hp1);
            hp1.Free;

            JumpC := taicpu(hp2).condition;

            if conditions_equal(JumpC, C_E) then
              SetC := inverse_cond(taicpu(p).condition)
            else if conditions_equal(JumpC, C_NE) then
              SetC := taicpu(p).condition
            else
              InternalError(2018061400);

            if SetC = C_NONE then
              InternalError(2018061401);

            taicpu(hp2).SetCondition(SetC);

            if not RegUsedAfterInstruction(taicpu(p).oper[0]^.reg, hp2, TmpUsedRegs) then
              begin
                asml.Remove(p);
                UpdateUsedRegs(next);
                p.Free;
                Result := True;
                p := hp2;
              end;

            DebugMsg(SPeepholeOptimization + 'SETcc/TEST/Jcc -> Jcc',p);
          end;
      end;


    function TX86AsmOptimizer.OptPass1FSTP(var p: tai): boolean;
      { returns true if a "continue" should be done after this optimization }
      var
        hp1, hp2: tai;
      begin
        Result := false;
        if MatchOpType(taicpu(p),top_ref) and
           GetNextInstruction(p, hp1) and
           (hp1.typ = ait_instruction) and
           (((taicpu(hp1).opcode = A_FLD) and
             (taicpu(p).opcode = A_FSTP)) or
            ((taicpu(p).opcode = A_FISTP) and
             (taicpu(hp1).opcode = A_FILD))) and
           MatchOpType(taicpu(hp1),top_ref) and
           (taicpu(hp1).opsize = taicpu(p).opsize) and
           RefsEqual(taicpu(p).oper[0]^.ref^, taicpu(hp1).oper[0]^.ref^) then
          begin
            { replacing fstp f;fld f by fst f is only valid for extended because of rounding }
            if (taicpu(p).opsize=S_FX) and
               GetNextInstruction(hp1, hp2) and
               (hp2.typ = ait_instruction) and
               IsExitCode(hp2) and
               (taicpu(p).oper[0]^.ref^.base = current_procinfo.FramePointer) and
               not(assigned(current_procinfo.procdef.funcretsym) and
                   (taicpu(p).oper[0]^.ref^.offset < tabstractnormalvarsym(current_procinfo.procdef.funcretsym).localloc.reference.offset)) and
               (taicpu(p).oper[0]^.ref^.index = NR_NO) then
              begin
                asml.remove(p);
                asml.remove(hp1);
                p.free;
                hp1.free;
                p := hp2;
                RemoveLastDeallocForFuncRes(p);
                Result := true;
              end
            (* can't be done because the store operation rounds
            else
              { fst can't store an extended value! }
              if (taicpu(p).opsize <> S_FX) and
                 (taicpu(p).opsize <> S_IQ) then
                begin
                  if (taicpu(p).opcode = A_FSTP) then
                    taicpu(p).opcode := A_FST
                  else taicpu(p).opcode := A_FIST;
                  asml.remove(hp1);
                  hp1.free;
                end
            *)
          end;
      end;


     function TX86AsmOptimizer.OptPass1FLD(var p : tai) : boolean;
      var
       hp1, hp2: tai;
      begin
        result:=false;
        if MatchOpType(taicpu(p),top_reg) and
           GetNextInstruction(p, hp1) and
           (hp1.typ = Ait_Instruction) and
           MatchOpType(taicpu(hp1),top_reg,top_reg) and
           (taicpu(hp1).oper[0]^.reg = NR_ST) and
           (taicpu(hp1).oper[1]^.reg = NR_ST1) then
           { change                        to
               fld      reg               fxxx reg,st
               fxxxp    st, st1 (hp1)
             Remark: non commutative operations must be reversed!
           }
          begin
              case taicpu(hp1).opcode Of
                A_FMULP,A_FADDP,
                A_FSUBP,A_FDIVP,A_FSUBRP,A_FDIVRP:
                  begin
                    case taicpu(hp1).opcode Of
                      A_FADDP: taicpu(hp1).opcode := A_FADD;
                      A_FMULP: taicpu(hp1).opcode := A_FMUL;
                      A_FSUBP: taicpu(hp1).opcode := A_FSUBR;
                      A_FSUBRP: taicpu(hp1).opcode := A_FSUB;
                      A_FDIVP: taicpu(hp1).opcode := A_FDIVR;
                      A_FDIVRP: taicpu(hp1).opcode := A_FDIV;
                      else
                        internalerror(2019050534);
                    end;
                    taicpu(hp1).oper[0]^.reg := taicpu(p).oper[0]^.reg;
                    taicpu(hp1).oper[1]^.reg := NR_ST;
                    asml.remove(p);
                    p.free;
                    p := hp1;
                    Result:=true;
                    exit;
                  end;
                else
                  ;
              end;
          end
        else
          if MatchOpType(taicpu(p),top_ref) and
             GetNextInstruction(p, hp2) and
             (hp2.typ = Ait_Instruction) and
             MatchOpType(taicpu(hp2),top_reg,top_reg) and
             (taicpu(p).opsize in [S_FS, S_FL]) and
             (taicpu(hp2).oper[0]^.reg = NR_ST) and
             (taicpu(hp2).oper[1]^.reg = NR_ST1) then
            if GetLastInstruction(p, hp1) and
              MatchInstruction(hp1,A_FLD,A_FST,[taicpu(p).opsize]) and
              MatchOpType(taicpu(hp1),top_ref) and
              RefsEqual(taicpu(p).oper[0]^.ref^, taicpu(hp1).oper[0]^.ref^) then
              if ((taicpu(hp2).opcode = A_FMULP) or
                  (taicpu(hp2).opcode = A_FADDP)) then
              { change                      to
                  fld/fst   mem1  (hp1)       fld/fst   mem1
                  fld       mem1  (p)         fadd/
                  faddp/                       fmul     st, st
                  fmulp  st, st1 (hp2) }
                begin
                  asml.remove(p);
                  p.free;
                  p := hp1;
                  if (taicpu(hp2).opcode = A_FADDP) then
                    taicpu(hp2).opcode := A_FADD
                  else
                    taicpu(hp2).opcode := A_FMUL;
                  taicpu(hp2).oper[1]^.reg := NR_ST;
                end
              else
              { change              to
                  fld/fst mem1 (hp1)   fld/fst mem1
                  fld     mem1 (p)     fld      st}
                begin
                  taicpu(p).changeopsize(S_FL);
                  taicpu(p).loadreg(0,NR_ST);
                end
            else
              begin
                case taicpu(hp2).opcode Of
                  A_FMULP,A_FADDP,A_FSUBP,A_FDIVP,A_FSUBRP,A_FDIVRP:
            { change                        to
                fld/fst  mem1    (hp1)      fld/fst    mem1
                fld      mem2    (p)        fxxx       mem2
                fxxxp    st, st1 (hp2)                      }

                    begin
                      case taicpu(hp2).opcode Of
                        A_FADDP: taicpu(p).opcode := A_FADD;
                        A_FMULP: taicpu(p).opcode := A_FMUL;
                        A_FSUBP: taicpu(p).opcode := A_FSUBR;
                        A_FSUBRP: taicpu(p).opcode := A_FSUB;
                        A_FDIVP: taicpu(p).opcode := A_FDIVR;
                        A_FDIVRP: taicpu(p).opcode := A_FDIV;
                        else
                          internalerror(2019050533);
                      end;
                      asml.remove(hp2);
                      hp2.free;
                    end
                  else
                    ;
                end
              end
      end;


   function TX86AsmOptimizer.OptPass2MOV(var p : tai) : boolean;
      var
       hp1,hp2: tai;
{$ifdef x86_64}
       hp3: tai;
{$endif x86_64}
      begin
        Result:=false;
        if MatchOpType(taicpu(p),top_reg,top_reg) and
          GetNextInstruction(p, hp1) and
{$ifdef x86_64}
          MatchInstruction(hp1,A_MOVZX,A_MOVSX,A_MOVSXD,[]) and
{$else x86_64}
          MatchInstruction(hp1,A_MOVZX,A_MOVSX,[]) and
{$endif x86_64}
          MatchOpType(taicpu(hp1),top_reg,top_reg) and
          (taicpu(hp1).oper[0]^.reg = taicpu(p).oper[1]^.reg) then
          { mov reg1, reg2                mov reg1, reg2
            movzx/sx reg2, reg3      to   movzx/sx reg1, reg3}
          begin
            taicpu(hp1).oper[0]^.reg := taicpu(p).oper[0]^.reg;
            DebugMsg(SPeepholeOptimization + 'mov %reg1,%reg2; movzx/sx %reg2,%reg3 -> mov %reg1,%reg2;movzx/sx %reg1,%reg3',p);

            { Don't remove the MOV command without first checking that reg2 isn't used afterwards,
              or unless supreg(reg3) = supreg(reg2)). [Kit] }

            TransferUsedRegs(TmpUsedRegs);
            UpdateUsedRegs(TmpUsedRegs, tai(p.next));

            if (getsupreg(taicpu(p).oper[1]^.reg) = getsupreg(taicpu(hp1).oper[1]^.reg)) or
              not RegUsedAfterInstruction(taicpu(p).oper[1]^.reg, hp1, TmpUsedRegs)
            then
              begin
                asml.remove(p);
                p.free;
                p := hp1;
                Result:=true;
              end;

            exit;
          end
        else if MatchOpType(taicpu(p),top_reg,top_reg) and
          GetNextInstruction(p, hp1) and
{$ifdef x86_64}
          MatchInstruction(hp1,[A_MOV,A_MOVZX,A_MOVSX,A_MOVSXD],[]) and
{$else x86_64}
          MatchInstruction(hp1,A_MOV,A_MOVZX,A_MOVSX,[]) and
{$endif x86_64}
          MatchOpType(taicpu(hp1),top_ref,top_reg) and
          ((taicpu(hp1).oper[0]^.ref^.base = taicpu(p).oper[1]^.reg)
           or
           (taicpu(hp1).oper[0]^.ref^.index = taicpu(p).oper[1]^.reg)
            ) and
          (getsupreg(taicpu(hp1).oper[1]^.reg) = getsupreg(taicpu(p).oper[1]^.reg)) then
          { mov reg1, reg2
            mov/zx/sx (reg2, ..), reg2      to   mov/zx/sx (reg1, ..), reg2}
          begin
            if (taicpu(hp1).oper[0]^.ref^.base = taicpu(p).oper[1]^.reg) then
              taicpu(hp1).oper[0]^.ref^.base := taicpu(p).oper[0]^.reg;
            if (taicpu(hp1).oper[0]^.ref^.index = taicpu(p).oper[1]^.reg) then
              taicpu(hp1).oper[0]^.ref^.index := taicpu(p).oper[0]^.reg;
            DebugMsg(SPeepholeOptimization + 'MovMovXX2MoVXX 1 done',p);
            asml.remove(p);
            p.free;
            p := hp1;
            Result:=true;
            exit;
          end
        else if (taicpu(p).oper[0]^.typ = top_ref) and
          GetNextInstruction(p,hp1) and
          (hp1.typ = ait_instruction) and
          { while the GetNextInstruction(hp1,hp2) call could be factored out,
            doing it separately in both branches allows to do the cheap checks
            with low probability earlier }
          ((IsFoldableArithOp(taicpu(hp1),taicpu(p).oper[1]^.reg) and
            GetNextInstruction(hp1,hp2) and
            MatchInstruction(hp2,A_MOV,[])
           ) or
           ((taicpu(hp1).opcode=A_LEA) and
             GetNextInstruction(hp1,hp2) and
             MatchInstruction(hp2,A_MOV,[]) and
            ((MatchReference(taicpu(hp1).oper[0]^.ref^,taicpu(p).oper[1]^.reg,NR_INVALID) and
             (taicpu(hp1).oper[0]^.ref^.index<>taicpu(p).oper[1]^.reg)
              ) or
             (MatchReference(taicpu(hp1).oper[0]^.ref^,NR_INVALID,
              taicpu(p).oper[1]^.reg) and
             (taicpu(hp1).oper[0]^.ref^.base<>taicpu(p).oper[1]^.reg)) or
             (MatchReferenceWithOffset(taicpu(hp1).oper[0]^.ref^,taicpu(p).oper[1]^.reg,NR_NO)) or
             (MatchReferenceWithOffset(taicpu(hp1).oper[0]^.ref^,NR_NO,taicpu(p).oper[1]^.reg))
            ) and
            ((MatchOperand(taicpu(p).oper[1]^,taicpu(hp2).oper[0]^)) or not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,hp1,UsedRegs)))
           )
          ) and
          MatchOperand(taicpu(hp1).oper[taicpu(hp1).ops-1]^,taicpu(hp2).oper[0]^) and
          (taicpu(hp2).oper[1]^.typ = top_ref) then
          begin
            TransferUsedRegs(TmpUsedRegs);
            UpdateUsedRegs(TmpUsedRegs,tai(p.next));
            UpdateUsedRegs(TmpUsedRegs,tai(hp1.next));
            if (RefsEqual(taicpu(hp2).oper[1]^.ref^,taicpu(p).oper[0]^.ref^) and
              not(RegUsedAfterInstruction(taicpu(hp2).oper[0]^.reg,hp2,TmpUsedRegs))) then
              { change   mov            (ref), reg
                         add/sub/or/... reg2/$const, reg
                         mov            reg, (ref)
                         # release reg
                to       add/sub/or/... reg2/$const, (ref)    }
              begin
                case taicpu(hp1).opcode of
                  A_INC,A_DEC,A_NOT,A_NEG :
                    taicpu(hp1).loadRef(0,taicpu(p).oper[0]^.ref^);
                  A_LEA :
                    begin
                      taicpu(hp1).opcode:=A_ADD;
                      if (taicpu(hp1).oper[0]^.ref^.index<>taicpu(p).oper[1]^.reg) and (taicpu(hp1).oper[0]^.ref^.index<>NR_NO) then
                        taicpu(hp1).loadreg(0,taicpu(hp1).oper[0]^.ref^.index)
                      else if (taicpu(hp1).oper[0]^.ref^.base<>taicpu(p).oper[1]^.reg) and (taicpu(hp1).oper[0]^.ref^.base<>NR_NO) then
                        taicpu(hp1).loadreg(0,taicpu(hp1).oper[0]^.ref^.base)
                      else
                        taicpu(hp1).loadconst(0,taicpu(hp1).oper[0]^.ref^.offset);
                      taicpu(hp1).loadRef(1,taicpu(p).oper[0]^.ref^);
                      DebugMsg(SPeepholeOptimization + 'FoldLea done',hp1);
                    end
                  else
                    taicpu(hp1).loadRef(1,taicpu(p).oper[0]^.ref^);
                end;
                asml.remove(p);
                asml.remove(hp2);
                p.free;
                hp2.free;
                p := hp1
              end;
            Exit;
{$ifdef x86_64}
          end
        else if (taicpu(p).opsize = S_L) and
          (taicpu(p).oper[1]^.typ = top_reg) and
          (
            GetNextInstruction(p, hp1) and
            MatchInstruction(hp1, A_MOV,[]) and
            (taicpu(hp1).opsize = S_L) and
            (taicpu(hp1).oper[1]^.typ = top_reg)
          ) and (
            GetNextInstruction(hp1, hp2) and
            (tai(hp2).typ=ait_instruction) and
            (taicpu(hp2).opsize = S_Q) and
            (
              (
                MatchInstruction(hp2, A_ADD,[]) and
                (taicpu(hp2).opsize = S_Q) and
                (taicpu(hp2).oper[0]^.typ = top_reg) and (taicpu(hp2).oper[1]^.typ = top_reg) and
                (
                  (
                    (getsupreg(taicpu(hp2).oper[0]^.reg) = getsupreg(taicpu(p).oper[1]^.reg)) and
                    (getsupreg(taicpu(hp2).oper[1]^.reg) = getsupreg(taicpu(hp1).oper[1]^.reg))
                  ) or (
                    (getsupreg(taicpu(hp2).oper[0]^.reg) = getsupreg(taicpu(hp1).oper[1]^.reg)) and
                    (getsupreg(taicpu(hp2).oper[1]^.reg) = getsupreg(taicpu(p).oper[1]^.reg))
                  )
                )
              ) or (
                MatchInstruction(hp2, A_LEA,[]) and
                (taicpu(hp2).oper[0]^.ref^.offset = 0) and
                (taicpu(hp2).oper[0]^.ref^.scalefactor <= 1) and
                (
                  (
                    (getsupreg(taicpu(hp2).oper[0]^.ref^.base) = getsupreg(taicpu(p).oper[1]^.reg)) and
                    (getsupreg(taicpu(hp2).oper[0]^.ref^.index) = getsupreg(taicpu(hp1).oper[1]^.reg))
                  ) or (
                    (getsupreg(taicpu(hp2).oper[0]^.ref^.base) = getsupreg(taicpu(hp1).oper[1]^.reg)) and
                    (getsupreg(taicpu(hp2).oper[0]^.ref^.index) = getsupreg(taicpu(p).oper[1]^.reg))
                  )
                ) and (
                  (
                    (getsupreg(taicpu(hp2).oper[1]^.reg) = getsupreg(taicpu(hp1).oper[1]^.reg))
                  ) or (
                    (getsupreg(taicpu(hp2).oper[1]^.reg) = getsupreg(taicpu(p).oper[1]^.reg))
                  )
                )
              )
            )
          ) and (
            GetNextInstruction(hp2, hp3) and
            MatchInstruction(hp3, A_SHR,[]) and
            (taicpu(hp3).opsize = S_Q) and
            (taicpu(hp3).oper[0]^.typ = top_const) and (taicpu(hp2).oper[1]^.typ = top_reg) and
            (taicpu(hp3).oper[0]^.val = 1) and
            (taicpu(hp3).oper[1]^.reg = taicpu(hp2).oper[1]^.reg)
          ) then
          begin
            { Change   movl    x,    reg1d         movl    x,    reg1d
                       movl    y,    reg2d         movl    y,    reg2d
                       addq    reg2q,reg1q   or    leaq    (reg1q,reg2q),reg1q
                       shrq    $1,   reg1q         shrq    $1,   reg1q

            ( reg1d and reg2d can be switched around in the first two instructions )

              To       movl    x,    reg1d
                       addl    y,    reg1d
                       rcrl    $1,   reg1d

              This corresponds to the common expression (x + y) shr 1, where
              x and y are Cardinals (replacing "shr 1" with "div 2" produces
              smaller code, but won't account for x + y causing an overflow). [Kit]
            }

            if (getsupreg(taicpu(hp2).oper[1]^.reg) = getsupreg(taicpu(hp1).oper[1]^.reg)) then
              { Change first MOV command to have the same register as the final output }
              taicpu(p).oper[1]^.reg := taicpu(hp1).oper[1]^.reg
            else
              taicpu(hp1).oper[1]^.reg := taicpu(p).oper[1]^.reg;

            { Change second MOV command to an ADD command. This is easier than
              converting the existing command because it means we don't have to
              touch 'y', which might be a complicated reference, and also the
              fact that the third command might either be ADD or LEA. [Kit] }
            taicpu(hp1).opcode := A_ADD;

            { Delete old ADD/LEA instruction }
            asml.remove(hp2);
            hp2.free;

            { Convert "shrq $1, reg1q" to "rcr $1, reg1d" }
            taicpu(hp3).opcode := A_RCR;
            taicpu(hp3).changeopsize(S_L);
            setsubreg(taicpu(hp3).oper[1]^.reg, R_SUBD);
{$endif x86_64}
          end;
      end;


    function TX86AsmOptimizer.OptPass2Imul(var p : tai) : boolean;
      var
        hp1 : tai;
      begin
        Result:=false;
        if (taicpu(p).ops >= 2) and
           ((taicpu(p).oper[0]^.typ = top_const) or
            ((taicpu(p).oper[0]^.typ = top_ref) and (taicpu(p).oper[0]^.ref^.refaddr=addr_full))) and
           (taicpu(p).oper[1]^.typ = top_reg) and
           ((taicpu(p).ops = 2) or
            ((taicpu(p).oper[2]^.typ = top_reg) and
             (taicpu(p).oper[2]^.reg = taicpu(p).oper[1]^.reg))) and
           GetLastInstruction(p,hp1) and
           MatchInstruction(hp1,A_MOV,[]) and
           MatchOpType(taicpu(hp1),top_reg,top_reg) and
           ((taicpu(hp1).oper[1]^.reg = taicpu(p).oper[1]^.reg) or
            ((taicpu(hp1).opsize=S_L) and (taicpu(p).opsize=S_Q) and SuperRegistersEqual(taicpu(hp1).oper[1]^.reg,taicpu(p).oper[1]^.reg))) then
          begin
            TransferUsedRegs(TmpUsedRegs);
            if not(RegUsedAfterInstruction(taicpu(p).oper[1]^.reg,p,TmpUsedRegs)) then
              { change
                  mov reg1,reg2
                  imul y,reg2 to imul y,reg1,reg2 }
              begin
                taicpu(p).ops := 3;
                taicpu(p).loadreg(1,taicpu(hp1).oper[0]^.reg);
                taicpu(p).loadreg(2,taicpu(hp1).oper[1]^.reg);
                DebugMsg(SPeepholeOptimization + 'MovImul2Imul done',p);
                asml.remove(hp1);
                hp1.free;
                result:=true;
              end;
          end;
      end;


    function TX86AsmOptimizer.OptPass2Jmp(var p : tai) : boolean;
      var
        hp1 : tai;
      begin
        {
          change
                 jmp .L1
                 ...
             .L1:
                 ret
          into
                 ret
        }
        result:=false;
        if (taicpu(p).oper[0]^.typ=top_ref) and (taicpu(p).oper[0]^.ref^.refaddr=addr_full) and (taicpu(p).oper[0]^.ref^.base=NR_NO) and
          (taicpu(p).oper[0]^.ref^.index=NR_NO) then
          begin
            hp1:=getlabelwithsym(tasmlabel(taicpu(p).oper[0]^.ref^.symbol));
            if (taicpu(p).condition=C_None) and assigned(hp1) and SkipLabels(hp1,hp1) and
              MatchInstruction(hp1,A_RET,[S_NO]) then
              begin
                tasmlabel(taicpu(p).oper[0]^.ref^.symbol).decrefs;
                taicpu(p).opcode:=A_RET;
                taicpu(p).is_jmp:=false;
                taicpu(p).ops:=taicpu(hp1).ops;
                case taicpu(hp1).ops of
                  0:
                    taicpu(p).clearop(0);
                  1:
                    taicpu(p).loadconst(0,taicpu(hp1).oper[0]^.val);
                  else
                    internalerror(2016041301);
                end;
                result:=true;
              end;
          end;
      end;


    function CanBeCMOV(p : tai) : boolean;
      begin
         CanBeCMOV:=assigned(p) and
           MatchInstruction(p,A_MOV,[S_W,S_L,S_Q]) and
           { we can't use cmov ref,reg because
             ref could be nil and cmov still throws an exception
             if ref=nil but the mov isn't done (FK)
            or ((taicpu(p).oper[0]^.typ = top_ref) and
             (taicpu(p).oper[0]^.ref^.refaddr = addr_no))
           }
           (MatchOpType(taicpu(p),top_reg,top_reg) or
            { allow references, but only pure symbols or got rel. addressing with RIP as based,
              it is not expected that this can cause a seg. violation }
            (MatchOpType(taicpu(p),top_ref,top_reg) and
             (((taicpu(p).oper[0]^.ref^.base=NR_NO) and (taicpu(p).oper[0]^.ref^.refaddr=addr_no)){$ifdef x86_64} or
              ((taicpu(p).oper[0]^.ref^.base=NR_RIP) and (taicpu(p).oper[0]^.ref^.refaddr=addr_pic)){$endif x86_64}
             ) and
             (taicpu(p).oper[0]^.ref^.index=NR_NO) and
             (taicpu(p).oper[0]^.ref^.offset=0)
            )
           );
      end;


    function TX86AsmOptimizer.OptPass2Jcc(var p : tai) : boolean;
      var
        hp1,hp2,hp3,hp4,hpmov2: tai;
        carryadd_opcode : TAsmOp;
        l : Longint;
        condition : TAsmCond;
        symbol: TAsmSymbol;
      begin
        result:=false;
        symbol:=nil;
        if GetNextInstruction(p,hp1) then
          begin
            symbol := TAsmLabel(taicpu(p).oper[0]^.ref^.symbol);

            if (hp1.typ=ait_instruction) and
               GetNextInstruction(hp1,hp2) and (hp2.typ=ait_label) and
               (Tasmlabel(symbol) = Tai_label(hp2).labsym) then
                 { jb @@1                            cmc
                   inc/dec operand           -->     adc/sbb operand,0
                   @@1:

                   ... and ...

                   jnb @@1
                   inc/dec operand           -->     adc/sbb operand,0
                   @@1: }
              begin
                carryadd_opcode:=A_NONE;
                if Taicpu(p).condition in [C_NAE,C_B] then
                  begin
                    if Taicpu(hp1).opcode=A_INC then
                      carryadd_opcode:=A_ADC;
                    if Taicpu(hp1).opcode=A_DEC then
                      carryadd_opcode:=A_SBB;
                    if carryadd_opcode<>A_NONE then
                      begin
                        Taicpu(p).clearop(0);
                        Taicpu(p).ops:=0;
                        Taicpu(p).is_jmp:=false;
                        Taicpu(p).opcode:=A_CMC;
                        Taicpu(p).condition:=C_NONE;
                        Taicpu(hp1).ops:=2;
                        Taicpu(hp1).loadoper(1,Taicpu(hp1).oper[0]^);
                        Taicpu(hp1).loadconst(0,0);
                        Taicpu(hp1).opcode:=carryadd_opcode;
                        result:=true;
                        exit;
                      end;
                  end;
                if Taicpu(p).condition in [C_AE,C_NB] then
                  begin
                    if Taicpu(hp1).opcode=A_INC then
                      carryadd_opcode:=A_ADC;
                    if Taicpu(hp1).opcode=A_DEC then
                      carryadd_opcode:=A_SBB;
                    if carryadd_opcode<>A_NONE then
                      begin
                        asml.remove(p);
                        p.free;
                        Taicpu(hp1).ops:=2;
                        Taicpu(hp1).loadoper(1,Taicpu(hp1).oper[0]^);
                        Taicpu(hp1).loadconst(0,0);
                        Taicpu(hp1).opcode:=carryadd_opcode;
                        p:=hp1;
                        result:=true;
                        exit;
                      end;
                  end;
              end;

            if ((hp1.typ = ait_label) and (symbol = tai_label(hp1).labsym))
                or ((hp1.typ = ait_align) and GetNextInstruction(hp1, hp2) and (hp2.typ = ait_label) and (symbol = tai_label(hp2).labsym)) then
              begin
                { If Jcc is immediately followed by the label that it's supposed to jump to, remove it }
                DebugMsg(SPeepholeOptimization + 'Removed conditional jump whose destination was immediately after it', p);
                UpdateUsedRegs(hp1);

                TAsmLabel(symbol).decrefs;
                { if the label refs. reach zero, remove any alignment before the label }
                if (hp1.typ = ait_align) then
                  begin
                    UpdateUsedRegs(hp2);
                    if (TAsmLabel(symbol).getrefs = 0) then
                    begin
                      asml.Remove(hp1);
                      hp1.Free;
                    end;
                    hp1 := hp2; { Set hp1 to the label }
                  end;

                asml.remove(p);
                p.free;

                if (TAsmLabel(symbol).getrefs = 0) then
                  begin
                    GetNextInstruction(hp1, p); { Instruction following the label }
                    asml.remove(hp1);
                    hp1.free;

                    UpdateUsedRegs(p);
                    Result := True;
                  end
                else
                  begin
                    { We don't need to set the result to True because we know hp1
                      is a label and won't trigger any optimisation routines. [Kit] }
                    p := hp1;
                  end;

                Exit;
              end;
          end;

{$ifndef i8086}
        if CPUX86_HAS_CMOV in cpu_capabilities[current_settings.cputype] then
          begin
             { check for
                    jCC   xxx
                    <several movs>
                 xxx:
             }
             l:=0;
             GetNextInstruction(p, hp1);
             while assigned(hp1) and
               CanBeCMOV(hp1) and
               { stop on labels }
               not(hp1.typ=ait_label) do
               begin
                  inc(l);
                  GetNextInstruction(hp1,hp1);
               end;
             if assigned(hp1) then
               begin
                  if FindLabel(tasmlabel(symbol),hp1) then
                    begin
                      if (l<=4) and (l>0) then
                        begin
                          condition:=inverse_cond(taicpu(p).condition);
                          GetNextInstruction(p,hp1);
                          repeat
                            if not Assigned(hp1) then
                              InternalError(2018062900);

                            taicpu(hp1).opcode:=A_CMOVcc;
                            taicpu(hp1).condition:=condition;
                            UpdateUsedRegs(hp1);
                            GetNextInstruction(hp1,hp1);
                          until not(CanBeCMOV(hp1));

                          { Don't decrement the reference count on the label yet, otherwise
                            GetNextInstruction might skip over the label if it drops to
                            zero. }
                          GetNextInstruction(hp1,hp2);

                          { if the label refs. reach zero, remove any alignment before the label }
                          if (hp1.typ = ait_align) and (hp2.typ = ait_label) then
                            begin
                              { Ref = 1 means it will drop to zero }
                              if (tasmlabel(symbol).getrefs=1) then
                                begin
                                  asml.Remove(hp1);
                                  hp1.Free;
                                end;
                            end
                          else
                            hp2 := hp1;

                          if not Assigned(hp2) then
                            InternalError(2018062910);

                          if (hp2.typ <> ait_label) then
                            begin
                              { There's something other than CMOVs here.  Move the original jump
                                to right before this point, then break out.

                                Originally this was part of the above internal error, but it got
                                triggered on the bootstrapping process sometimes. Investigate. [Kit] }
                              asml.remove(p);
                              asml.insertbefore(p, hp2);
                              DebugMsg('Jcc/CMOVcc drop-out', p);
                              UpdateUsedRegs(p);
                              Result := True;
                              Exit;
                            end;

                          { Now we can safely decrement the reference count }
                          tasmlabel(symbol).decrefs;

                          { Remove the original jump }
                          asml.Remove(p);
                          p.Free;

                          GetNextInstruction(hp2, p); { Instruction after the label }

                          { Remove the label if this is its final reference }
                          if (tasmlabel(symbol).getrefs=0) then
                            begin
                              asml.remove(hp2);
                              hp2.free;
                            end;

                          if Assigned(p) then
                            begin
                              UpdateUsedRegs(p);
                              result:=true;
                            end;
                          exit;
                        end;
                    end
                  else
                    begin
                       { check further for
                              jCC   xxx
                              <several movs 1>
                              jmp   yyy
                      xxx:
                              <several movs 2>
                      yyy:
                       }
                      { hp2 points to jmp yyy }
                      hp2:=hp1;
                      { skip hp1 to xxx (or an align right before it) }
                      GetNextInstruction(hp1, hp1);

                      if assigned(hp2) and
                        assigned(hp1) and
                        (l<=3) and
                        (hp2.typ=ait_instruction) and
                        (taicpu(hp2).is_jmp) and
                        (taicpu(hp2).condition=C_None) and
                        { real label and jump, no further references to the
                          label are allowed }
                        (tasmlabel(symbol).getrefs=1) and
                        FindLabel(tasmlabel(symbol),hp1) then
                         begin
                           l:=0;
                           { skip hp1 to <several moves 2> }
                           if (hp1.typ = ait_align) then
                             GetNextInstruction(hp1, hp1);

                           GetNextInstruction(hp1, hpmov2);

                           hp1 := hpmov2;
                           while assigned(hp1) and
                             CanBeCMOV(hp1) do
                             begin
                               inc(l);
                               GetNextInstruction(hp1, hp1);
                             end;
                           { hp1 points to yyy (or an align right before it) }
                           hp3 := hp1;
                           if assigned(hp1) and
                             FindLabel(tasmlabel(taicpu(hp2).oper[0]^.ref^.symbol),hp1) then
                             begin
                                condition:=inverse_cond(taicpu(p).condition);
                                GetNextInstruction(p,hp1);
                                repeat
                                  taicpu(hp1).opcode:=A_CMOVcc;
                                  taicpu(hp1).condition:=condition;
                                  UpdateUsedRegs(hp1);
                                  GetNextInstruction(hp1,hp1);
                                until not(assigned(hp1)) or
                                  not(CanBeCMOV(hp1));

                                condition:=inverse_cond(condition);
                                hp1 := hpmov2;
                                { hp1 is now at <several movs 2> }
                                while Assigned(hp1) and CanBeCMOV(hp1) do
                                  begin
                                    taicpu(hp1).opcode:=A_CMOVcc;
                                    taicpu(hp1).condition:=condition;
                                    UpdateUsedRegs(hp1);
                                    GetNextInstruction(hp1,hp1);
                                  end;

                                hp1 := p;

                                { Get first instruction after label }
                                GetNextInstruction(hp3, p);

                                if assigned(p) and (hp3.typ = ait_align) then
                                  GetNextInstruction(p, p);

                                { Don't dereference yet, as doing so will cause
                                  GetNextInstruction to skip the label and
                                  optional align marker. [Kit] }
                                GetNextInstruction(hp2, hp4);

                                { remove jCC }
                                asml.remove(hp1);
                                hp1.free;

                                { Remove label xxx (it will have a ref of zero due to the initial check }
                                if (hp4.typ = ait_align) then
                                  begin
                                    { Account for alignment as well }
                                    GetNextInstruction(hp4, hp1);
                                    asml.remove(hp1);
                                    hp1.free;
                                  end;

                                asml.remove(hp4);
                                hp4.free;

                                { Now we can safely decrement it }
                                tasmlabel(symbol).decrefs;

                                { remove jmp }
                                symbol := taicpu(hp2).oper[0]^.ref^.symbol;

                                asml.remove(hp2);
                                hp2.free;

                                { Remove label yyy (and the optional alignment) if its reference will fall to zero }
                                if tasmlabel(symbol).getrefs = 1 then
                                  begin
                                    if (hp3.typ = ait_align) then
                                      begin
                                        { Account for alignment as well }
                                        GetNextInstruction(hp3, hp1);
                                        asml.remove(hp1);
                                        hp1.free;
                                      end;

                                    asml.remove(hp3);
                                    hp3.free;

                                    { As before, now we can safely decrement it }
                                    tasmlabel(symbol).decrefs;
                                  end;

                                if Assigned(p) then
                                  begin
                                    UpdateUsedRegs(p);
                                    result:=true;
                                  end;
                                exit;
                             end;
                         end;
                    end;
               end;
          end;
{$endif i8086}
      end;


    function TX86AsmOptimizer.OptPass1Movx(var p : tai) : boolean;
      var
        hp1,hp2: tai;
      begin
        result:=false;
        if (taicpu(p).oper[1]^.typ = top_reg) and
           GetNextInstruction(p,hp1) and
           (hp1.typ = ait_instruction) and
           IsFoldableArithOp(taicpu(hp1),taicpu(p).oper[1]^.reg) and
           GetNextInstruction(hp1,hp2) and
           MatchInstruction(hp2,A_MOV,[]) and
           (taicpu(hp2).oper[0]^.typ = top_reg) and
           OpsEqual(taicpu(hp2).oper[1]^,taicpu(p).oper[0]^) and
{$ifdef i386}
           { not all registers have byte size sub registers on i386 }
           ((taicpu(hp2).opsize<>S_B) or (getsupreg(taicpu(hp1).oper[0]^.reg) in [RS_EAX, RS_EBX, RS_ECX, RS_EDX])) and
{$endif i386}
           (((taicpu(hp1).ops=2) and
             (getsupreg(taicpu(hp2).oper[0]^.reg)=getsupreg(taicpu(hp1).oper[1]^.reg))) or
             ((taicpu(hp1).ops=1) and
             (getsupreg(taicpu(hp2).oper[0]^.reg)=getsupreg(taicpu(hp1).oper[0]^.reg)))) and
           not(RegUsedAfterInstruction(taicpu(hp2).oper[0]^.reg,hp2,UsedRegs)) then
          begin
            { change   movsX/movzX    reg/ref, reg2
                       add/sub/or/... reg3/$const, reg2
                       mov            reg2 reg/ref
              to       add/sub/or/... reg3/$const, reg/ref      }

            { by example:
                movswl  %si,%eax        movswl  %si,%eax      p
                decl    %eax            addl    %edx,%eax     hp1
                movw    %ax,%si         movw    %ax,%si       hp2
              ->
                movswl  %si,%eax        movswl  %si,%eax      p
                decw    %eax            addw    %edx,%eax     hp1
                movw    %ax,%si         movw    %ax,%si       hp2
            }
            taicpu(hp1).changeopsize(taicpu(hp2).opsize);
            {
              ->
                movswl  %si,%eax        movswl  %si,%eax      p
                decw    %si             addw    %dx,%si       hp1
                movw    %ax,%si         movw    %ax,%si       hp2
            }
            case taicpu(hp1).ops of
              1:
               taicpu(hp1).loadoper(0,taicpu(hp2).oper[1]^);
              2:
                begin
                  taicpu(hp1).loadoper(1,taicpu(hp2).oper[1]^);
                  if (taicpu(hp1).oper[0]^.typ = top_reg) then
                    setsubreg(taicpu(hp1).oper[0]^.reg,getsubreg(taicpu(hp2).oper[0]^.reg));
                end;
              else
                internalerror(2008042701);
            end;
            {
              ->
                decw    %si             addw    %dx,%si       p
            }
            DebugMsg(SPeepholeOptimization + 'var3',p);
            asml.remove(p);
            asml.remove(hp2);
            p.free;
            hp2.free;
            p:=hp1;
          end
        else if taicpu(p).opcode=A_MOVZX then
          begin
            { removes superfluous And's after movzx's }
            if (taicpu(p).oper[1]^.typ = top_reg) and
              GetNextInstruction(p, hp1) and
              (tai(hp1).typ = ait_instruction) and
              (taicpu(hp1).opcode = A_AND) and
              (taicpu(hp1).oper[0]^.typ = top_const) and
              (taicpu(hp1).oper[1]^.typ = top_reg) and
              (taicpu(hp1).oper[1]^.reg = taicpu(p).oper[1]^.reg) then
              begin
                case taicpu(p).opsize Of
                  S_BL, S_BW{$ifdef x86_64}, S_BQ{$endif x86_64}:
                    if (taicpu(hp1).oper[0]^.val = $ff) then
                      begin
                        DebugMsg(SPeepholeOptimization + 'var4',p);
                        asml.remove(hp1);
                        hp1.free;
                      end;
                    S_WL{$ifdef x86_64}, S_WQ{$endif x86_64}:
                      if (taicpu(hp1).oper[0]^.val = $ffff) then
                        begin
                          DebugMsg(SPeepholeOptimization + 'var5',p);
                          asml.remove(hp1);
                          hp1.free;
                        end;
{$ifdef x86_64}
                    S_LQ:
                      if (taicpu(hp1).oper[0]^.val = $ffffffff) then
                        begin
                          if (cs_asm_source in current_settings.globalswitches) then
                            asml.insertbefore(tai_comment.create(strpnew(SPeepholeOptimization + 'var6')),p);
                          asml.remove(hp1);
                          hp1.Free;
                        end;
{$endif x86_64}
                  else
                    ;
                end;
              end;
            { changes some movzx constructs to faster synonims (all examples
              are given with eax/ax, but are also valid for other registers)}
            if (taicpu(p).oper[1]^.typ = top_reg) then
              if (taicpu(p).oper[0]^.typ = top_reg) then
                case taicpu(p).opsize of
                  S_BW:
                    begin
                      if (getsupreg(taicpu(p).oper[0]^.reg)=getsupreg(taicpu(p).oper[1]^.reg)) and
                         not(cs_opt_size in current_settings.optimizerswitches) then
                        {Change "movzbw %al, %ax" to "andw $0x0ffh, %ax"}
                        begin
                          taicpu(p).opcode := A_AND;
                          taicpu(p).changeopsize(S_W);
                          taicpu(p).loadConst(0,$ff);
                          DebugMsg(SPeepholeOptimization + 'var7',p);
                        end
                      else if GetNextInstruction(p, hp1) and
                        (tai(hp1).typ = ait_instruction) and
                        (taicpu(hp1).opcode = A_AND) and
                        (taicpu(hp1).oper[0]^.typ = top_const) and
                        (taicpu(hp1).oper[1]^.typ = top_reg) and
                        (taicpu(hp1).oper[1]^.reg = taicpu(p).oper[1]^.reg) then
                      { Change "movzbw %reg1, %reg2; andw $const, %reg2"
                        to "movw %reg1, reg2; andw $(const1 and $ff), %reg2"}
                        begin
                          DebugMsg(SPeepholeOptimization + 'var8',p);
                          taicpu(p).opcode := A_MOV;
                          taicpu(p).changeopsize(S_W);
                          setsubreg(taicpu(p).oper[0]^.reg,R_SUBW);
                          taicpu(hp1).loadConst(0,taicpu(hp1).oper[0]^.val and $ff);
                        end;
                    end;
                  S_BL:
                    begin
                      if (getsupreg(taicpu(p).oper[0]^.reg)=getsupreg(taicpu(p).oper[1]^.reg)) and
                         not(cs_opt_size in current_settings.optimizerswitches) then
                        { Change "movzbl %al, %eax" to "andl $0x0ffh, %eax" }
                        begin
                          taicpu(p).opcode := A_AND;
                          taicpu(p).changeopsize(S_L);
                          taicpu(p).loadConst(0,$ff)
                        end
                      else if GetNextInstruction(p, hp1) and
                        (tai(hp1).typ = ait_instruction) and
                        (taicpu(hp1).opcode = A_AND) and
                        (taicpu(hp1).oper[0]^.typ = top_const) and
                        (taicpu(hp1).oper[1]^.typ = top_reg) and
                        (taicpu(hp1).oper[1]^.reg = taicpu(p).oper[1]^.reg) then
                        { Change "movzbl %reg1, %reg2; andl $const, %reg2"
                          to "movl %reg1, reg2; andl $(const1 and $ff), %reg2"}
                        begin
                          DebugMsg(SPeepholeOptimization + 'var10',p);
                          taicpu(p).opcode := A_MOV;
                          taicpu(p).changeopsize(S_L);
                          { do not use R_SUBWHOLE
                            as movl %rdx,%eax
                            is invalid in assembler PM }
                          setsubreg(taicpu(p).oper[0]^.reg, R_SUBD);
                          taicpu(hp1).loadConst(0,taicpu(hp1).oper[0]^.val and $ff);
                        end
                    end;
{$ifndef i8086}
                  S_WL:
                    begin
                      if (getsupreg(taicpu(p).oper[0]^.reg)=getsupreg(taicpu(p).oper[1]^.reg)) and
                        not(cs_opt_size in current_settings.optimizerswitches) then
                        { Change "movzwl %ax, %eax" to "andl $0x0ffffh, %eax" }
                        begin
                          DebugMsg(SPeepholeOptimization + 'var11',p);
                          taicpu(p).opcode := A_AND;
                          taicpu(p).changeopsize(S_L);
                          taicpu(p).loadConst(0,$ffff);
                        end
                      else if GetNextInstruction(p, hp1) and
                        (tai(hp1).typ = ait_instruction) and
                        (taicpu(hp1).opcode = A_AND) and
                        (taicpu(hp1).oper[0]^.typ = top_const) and
                        (taicpu(hp1).oper[1]^.typ = top_reg) and
                        (taicpu(hp1).oper[1]^.reg = taicpu(p).oper[1]^.reg) then
                        { Change "movzwl %reg1, %reg2; andl $const, %reg2"
                          to "movl %reg1, reg2; andl $(const1 and $ffff), %reg2"}
                        begin
                          DebugMsg(SPeepholeOptimization + 'var12',p);
                          taicpu(p).opcode := A_MOV;
                          taicpu(p).changeopsize(S_L);
                          { do not use R_SUBWHOLE
                            as movl %rdx,%eax
                            is invalid in assembler PM }
                          setsubreg(taicpu(p).oper[0]^.reg, R_SUBD);
                          taicpu(hp1).loadConst(0,taicpu(hp1).oper[0]^.val and $ffff);
                        end;
                    end;
{$endif i8086}
                  else
                    ;
                end
              else if (taicpu(p).oper[0]^.typ = top_ref) then
                  begin
                    if GetNextInstruction(p, hp1) and
                      (tai(hp1).typ = ait_instruction) and
                      (taicpu(hp1).opcode = A_AND) and
                      MatchOpType(taicpu(hp1),top_const,top_reg) and
                      (taicpu(hp1).oper[1]^.reg = taicpu(p).oper[1]^.reg) then
                      begin
                        //taicpu(p).opcode := A_MOV;
                        case taicpu(p).opsize Of
                          S_BL:
                            begin
                              DebugMsg(SPeepholeOptimization + 'var13',p);
                              taicpu(hp1).changeopsize(S_L);
                              taicpu(hp1).loadConst(0,taicpu(hp1).oper[0]^.val and $ff);
                            end;
                          S_WL:
                            begin
                              DebugMsg(SPeepholeOptimization + 'var14',p);
                              taicpu(hp1).changeopsize(S_L);
                              taicpu(hp1).loadConst(0,taicpu(hp1).oper[0]^.val and $ffff);
                            end;
                          S_BW:
                            begin
                              DebugMsg(SPeepholeOptimization + 'var15',p);
                              taicpu(hp1).changeopsize(S_W);
                              taicpu(hp1).loadConst(0,taicpu(hp1).oper[0]^.val and $ff);
                            end;
{$ifdef x86_64}
                          S_BQ:
                            begin
                              DebugMsg(SPeepholeOptimization + 'var16',p);
                              taicpu(hp1).changeopsize(S_Q);
                              taicpu(hp1).loadConst(
                                0, taicpu(hp1).oper[0]^.val and $ff);
                            end;
                          S_WQ:
                            begin
                              DebugMsg(SPeepholeOptimization + 'var17',p);
                              taicpu(hp1).changeopsize(S_Q);
                              taicpu(hp1).loadConst(0, taicpu(hp1).oper[0]^.val and $ffff);
                            end;
                          S_LQ:
                            begin
                              DebugMsg(SPeepholeOptimization + 'var18',p);
                              taicpu(hp1).changeopsize(S_Q);
                              taicpu(hp1).loadConst(
                                0, taicpu(hp1).oper[0]^.val and $ffffffff);
                            end;
{$endif x86_64}
                          else
                            Internalerror(2017050704)
                        end;
                      end;
                  end;
          end;
      end;


    function TX86AsmOptimizer.OptPass1AND(var p : tai) : boolean;
      var
        hp1 : tai;
        MaskLength : Cardinal;
      begin
        Result:=false;

        if GetNextInstruction(p, hp1) then
          begin
            if MatchOpType(taicpu(p),top_const,top_reg) and
              MatchInstruction(hp1,A_AND,[]) and
              MatchOpType(taicpu(hp1),top_const,top_reg) and
              (getsupreg(taicpu(p).oper[1]^.reg) = getsupreg(taicpu(hp1).oper[1]^.reg)) and
              { the second register must contain the first one, so compare their subreg types }
              (getsubreg(taicpu(p).oper[1]^.reg)<=getsubreg(taicpu(hp1).oper[1]^.reg)) and
              (abs(taicpu(p).oper[0]^.val and taicpu(hp1).oper[0]^.val)<$80000000) then
              { change
                  and const1, reg
                  and const2, reg
                to
                  and (const1 and const2), reg
              }
              begin
                taicpu(hp1).loadConst(0, taicpu(p).oper[0]^.val and taicpu(hp1).oper[0]^.val);
                DebugMsg(SPeepholeOptimization + 'AndAnd2And done',hp1);
                asml.remove(p);
                p.Free;
                p:=hp1;
                Result:=true;
                exit;
              end
            else if MatchOpType(taicpu(p),top_const,top_reg) and
              MatchInstruction(hp1,A_MOVZX,[]) and
              (taicpu(hp1).oper[0]^.typ = top_reg) and
              MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[1]^) and
              (getsupreg(taicpu(hp1).oper[0]^.reg)=getsupreg(taicpu(hp1).oper[1]^.reg)) and
               (((taicpu(p).opsize=S_W) and
                 (taicpu(hp1).opsize=S_BW)) or
                ((taicpu(p).opsize=S_L) and
                 (taicpu(hp1).opsize in [S_WL,S_BL]))
{$ifdef x86_64}
                  or
                 ((taicpu(p).opsize=S_Q) and
                  (taicpu(hp1).opsize in [S_BQ,S_WQ]))
{$endif x86_64}
                ) then
                  begin
                    if (((taicpu(hp1).opsize) in [S_BW,S_BL{$ifdef x86_64},S_BQ{$endif x86_64}]) and
                        ((taicpu(p).oper[0]^.val and $ff)=taicpu(p).oper[0]^.val)
                         ) or
                       (((taicpu(hp1).opsize) in [S_WL{$ifdef x86_64},S_WQ{$endif x86_64}]) and
                        ((taicpu(p).oper[0]^.val and $ffff)=taicpu(p).oper[0]^.val))
                    then
                      begin
                        { Unlike MOVSX, MOVZX doesn't actually have a version that zero-extends a
                          32-bit register to a 64-bit register, or even a version called MOVZXD, so
                          code that tests for the presence of AND 0xffffffff followed by MOVZX is
                          wasted, and is indictive of a compiler bug if it were triggered. [Kit]

                          NOTE: To zero-extend from 32 bits to 64 bits, simply use the standard MOV.
                        }
                        DebugMsg(SPeepholeOptimization + 'AndMovzToAnd done',p);

                        asml.remove(hp1);
                        hp1.free;
                        Exit;
                      end;
                  end
            else if MatchOpType(taicpu(p),top_const,top_reg) and
              MatchInstruction(hp1,A_SHL,[]) and
              MatchOpType(taicpu(hp1),top_const,top_reg) and
              (getsupreg(taicpu(p).oper[1]^.reg)=getsupreg(taicpu(hp1).oper[1]^.reg)) then
              begin
{$ifopt R+}
{$define RANGE_WAS_ON}
{$R-}
{$endif}
                { get length of potential and mask }
                MaskLength:=SizeOf(taicpu(p).oper[0]^.val)*8-BsrQWord(taicpu(p).oper[0]^.val)-1;

                { really a mask? }
{$ifdef RANGE_WAS_ON}
{$R+}
{$endif}
                if (((QWord(1) shl MaskLength)-1)=taicpu(p).oper[0]^.val) and
                  { unmasked part shifted out? }
                  ((MaskLength+taicpu(hp1).oper[0]^.val)>=topsize2memsize[taicpu(hp1).opsize]) then
                  begin
                    DebugMsg(SPeepholeOptimization + 'AndShlToShl done',p);

                    { take care of the register (de)allocs following p }
                    UpdateUsedRegs(tai(p.next));
                    asml.remove(p);
                    p.free;
                    p:=hp1;
                    Result:=true;
                    exit;
                  end;
              end
            else if MatchOpType(taicpu(p),top_const,top_reg) and
              MatchInstruction(hp1,A_MOVSX{$ifdef x86_64},A_MOVSXD{$endif x86_64},[]) and
              (taicpu(hp1).oper[0]^.typ = top_reg) and
              MatchOperand(taicpu(p).oper[1]^,taicpu(hp1).oper[1]^) and
              (getsupreg(taicpu(hp1).oper[0]^.reg)=getsupreg(taicpu(hp1).oper[1]^.reg)) and
               (((taicpu(p).opsize=S_W) and
                 (taicpu(hp1).opsize=S_BW)) or
                ((taicpu(p).opsize=S_L) and
                 (taicpu(hp1).opsize in [S_WL,S_BL]))
{$ifdef x86_64}
                 or
                 ((taicpu(p).opsize=S_Q) and
                 (taicpu(hp1).opsize in [S_BQ,S_WQ,S_LQ]))
{$endif x86_64}
                ) then
                  begin
                    if (((taicpu(hp1).opsize) in [S_BW,S_BL{$ifdef x86_64},S_BQ{$endif x86_64}]) and
                        ((taicpu(p).oper[0]^.val and $7f)=taicpu(p).oper[0]^.val)
                         ) or
                       (((taicpu(hp1).opsize) in [S_WL{$ifdef x86_64},S_WQ{$endif x86_64}]) and
                        ((taicpu(p).oper[0]^.val and $7fff)=taicpu(p).oper[0]^.val))
{$ifdef x86_64}
                       or
                       (((taicpu(hp1).opsize)=S_LQ) and
                        ((taicpu(p).oper[0]^.val and $7fffffff)=taicpu(p).oper[0]^.val)
                       )
{$endif x86_64}
                       then
                       begin
                         DebugMsg(SPeepholeOptimization + 'AndMovsxToAnd',p);
                         asml.remove(hp1);
                         hp1.free;
                         Exit;
                       end;
                  end
            else if (taicpu(p).oper[1]^.typ = top_reg) and
              (hp1.typ = ait_instruction) and
              (taicpu(hp1).is_jmp) and
              (taicpu(hp1).opcode<>A_JMP) and
              not(RegInUsedRegs(taicpu(p).oper[1]^.reg,UsedRegs)) then
              begin
                { change
                    and x, reg
                    jxx
                  to
                    test x, reg
                    jxx
                  if reg is deallocated before the
                  jump, but only if it's a conditional jump (PFV)
                }
                taicpu(p).opcode := A_TEST;
                Exit;
              end;
          end;

        { Lone AND tests }
        if MatchOpType(taicpu(p),top_const,top_reg) then
          begin
            {
              - Convert and $0xFF,reg to and reg,reg if reg is 8-bit
              - Convert and $0xFFFF,reg to and reg,reg if reg is 16-bit
              - Convert and $0xFFFFFFFF,reg to and reg,reg if reg is 32-bit
            }
            if ((taicpu(p).oper[0]^.val = $FF) and (taicpu(p).opsize = S_B)) or
              ((taicpu(p).oper[0]^.val = $FFFF) and (taicpu(p).opsize = S_W)) or
              ((taicpu(p).oper[0]^.val = $FFFFFFFF) and (taicpu(p).opsize = S_L)) then
              begin
                taicpu(p).loadreg(0, taicpu(p).oper[1]^.reg)
              end;
          end;

      end;


    function TX86AsmOptimizer.OptPass2Lea(var p : tai) : Boolean;
      begin
        Result:=false;
        if not (RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) and
          MatchReference(taicpu(p).oper[0]^.ref^,taicpu(p).oper[1]^.reg,NR_INVALID) and
          (taicpu(p).oper[0]^.ref^.index<>NR_NO) then
          begin
            taicpu(p).loadreg(1,taicpu(p).oper[0]^.ref^.base);
            taicpu(p).loadreg(0,taicpu(p).oper[0]^.ref^.index);
            taicpu(p).opcode:=A_ADD;
            DebugMsg(SPeepholeOptimization + 'Lea2AddBase done',p);
            result:=true;
          end

        else if not (RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) and
          MatchReference(taicpu(p).oper[0]^.ref^,NR_INVALID,taicpu(p).oper[1]^.reg) and
          (taicpu(p).oper[0]^.ref^.base<>NR_NO) then
          begin
            taicpu(p).loadreg(1,taicpu(p).oper[0]^.ref^.index);
            taicpu(p).loadreg(0,taicpu(p).oper[0]^.ref^.base);
            taicpu(p).opcode:=A_ADD;
            DebugMsg(SPeepholeOptimization + 'Lea2AddIndex done',p);
            result:=true;
          end;
      end;


    function TX86AsmOptimizer.PostPeepholeOptLea(var p : tai) : Boolean;

      function SkipSimpleInstructions(var hp1 : tai) : Boolean;
        begin
          { we can skip all instructions not messing with the stack pointer }
          while assigned(hp1) and {MatchInstruction(taicpu(hp1),[A_LEA,A_MOV,A_MOVQ,A_MOVSQ,A_MOVSX,A_MOVSXD,A_MOVZX,
            A_AND,A_OR,A_XOR,A_ADD,A_SHR,A_SHL,A_IMUL,A_SETcc,A_SAR,A_SUB,A_TEST,A_CMOVcc,
            A_MOVSS,A_MOVSD,A_MOVAPS,A_MOVUPD,A_MOVAPD,A_MOVUPS,
            A_VMOVSS,A_VMOVSD,A_VMOVAPS,A_VMOVUPD,A_VMOVAPD,A_VMOVUPS],[]) and}
            ({(taicpu(hp1).ops=0) or }
             ({(MatchOpType(taicpu(hp1),top_reg,top_reg) or MatchOpType(taicpu(hp1),top_const,top_reg) or
               (MatchOpType(taicpu(hp1),top_ref,top_reg))
              ) and }
              not(RegInInstruction(NR_STACK_POINTER_REG,hp1)) { and not(RegInInstruction(NR_FRAME_POINTER_REG,hp1))}
             )
            ) do
            GetNextInstruction(hp1,hp1);
          Result:=assigned(hp1);
        end;

      var
        hp1, hp2, hp3: tai;
      begin
        Result:=false;
        { replace
            leal(q) x(<stackpointer>),<stackpointer>
            call   procname
            leal(q) -x(<stackpointer>),<stackpointer>
            ret
          by
            jmp    procname

          but do it only on level 4 because it destroys stack back traces
        }
        if (cs_opt_level4 in current_settings.optimizerswitches) and
          MatchOpType(taicpu(p),top_ref,top_reg) and
          (taicpu(p).oper[0]^.ref^.base=NR_STACK_POINTER_REG) and
          (taicpu(p).oper[0]^.ref^.index=NR_NO) and
          { the -8 or -24 are not required, but bail out early if possible,
            higher values are unlikely }
          ((taicpu(p).oper[0]^.ref^.offset=-8) or
           (taicpu(p).oper[0]^.ref^.offset=-24))  and
          (taicpu(p).oper[0]^.ref^.symbol=nil) and
          (taicpu(p).oper[0]^.ref^.relsymbol=nil) and
          (taicpu(p).oper[0]^.ref^.segment=NR_NO) and
          (taicpu(p).oper[1]^.reg=NR_STACK_POINTER_REG) and
          GetNextInstruction(p, hp1) and
          { trick to skip label }
          ((hp1.typ=ait_instruction) or GetNextInstruction(hp1, hp1)) and
          SkipSimpleInstructions(hp1) and
          MatchInstruction(hp1,A_CALL,[S_NO]) and
          GetNextInstruction(hp1, hp2) and
          MatchInstruction(hp2,A_LEA,[taicpu(p).opsize]) and
          MatchOpType(taicpu(hp2),top_ref,top_reg) and
          (taicpu(hp2).oper[0]^.ref^.offset=-taicpu(p).oper[0]^.ref^.offset) and
          (taicpu(hp2).oper[0]^.ref^.base=NR_STACK_POINTER_REG) and
          (taicpu(hp2).oper[0]^.ref^.index=NR_NO) and
          (taicpu(hp2).oper[0]^.ref^.symbol=nil) and
          (taicpu(hp2).oper[0]^.ref^.relsymbol=nil) and
          (taicpu(hp2).oper[0]^.ref^.segment=NR_NO) and
          (taicpu(hp2).oper[1]^.reg=NR_STACK_POINTER_REG) and
          GetNextInstruction(hp2, hp3) and
          { trick to skip label }
          ((hp3.typ=ait_instruction) or GetNextInstruction(hp3, hp3)) and
          MatchInstruction(hp3,A_RET,[S_NO]) and
          (taicpu(hp3).ops=0) then
          begin
            taicpu(hp1).opcode := A_JMP;
            taicpu(hp1).is_jmp := true;
            DebugMsg(SPeepholeOptimization + 'LeaCallLeaRet2Jmp done',p);
            RemoveCurrentP(p);
            AsmL.Remove(hp2);
            hp2.free;
            AsmL.Remove(hp3);
            hp3.free;
            Result:=true;
          end;
      end;


    function TX86AsmOptimizer.PostPeepholeOptMov(var p : tai) : Boolean;
      var
        Value, RegName: string;
      begin
        Result:=false;
        if (taicpu(p).oper[1]^.typ = top_reg) and (taicpu(p).oper[0]^.typ = top_const) then
          begin

            case taicpu(p).oper[0]^.val of
            0:
              { Don't make this optimisation if the CPU flags are required, since XOR scrambles them }
              if not (RegInUsedRegs(NR_DEFAULTFLAGS,UsedRegs)) then
                begin
                  { change "mov $0,%reg" into "xor %reg,%reg" }
                  taicpu(p).opcode := A_XOR;
                  taicpu(p).loadReg(0,taicpu(p).oper[1]^.reg);
                  Result := True;
                end;
            $1..$FFFFFFFF:
              begin
                { Code size reduction by J. Gareth "Kit" Moreton }
                { change 64-bit register to 32-bit register to reduce code size (upper 32 bits will be set to zero) }
                case taicpu(p).opsize of
                  S_Q:
                    begin
                      RegName := debug_regname(taicpu(p).oper[1]^.reg); { 64-bit register name }
                      Value := debug_tostr(taicpu(p).oper[0]^.val);

                      { The actual optimization }
                      setsubreg(taicpu(p).oper[1]^.reg, R_SUBD);
                      taicpu(p).changeopsize(S_L);

                      DebugMsg(SPeepholeOptimization + 'movq $' + Value + ',' + RegName + ' -> movl $' + Value + ',' + debug_regname(taicpu(p).oper[1]^.reg) + ' (immediate can be represented with just 32 bits)', p);
                      Result := True;
                    end;
                  else
                    ;
                end;
              end;
            end;
          end;
      end;


    function TX86AsmOptimizer.PostPeepholeOptCmp(var p : tai) : Boolean;
      begin
        Result:=false;
        { change "cmp $0, %reg" to "test %reg, %reg" }
        if MatchOpType(taicpu(p),top_const,top_reg) and
           (taicpu(p).oper[0]^.val = 0) then
          begin
            taicpu(p).opcode := A_TEST;
            taicpu(p).loadreg(0,taicpu(p).oper[1]^.reg);
            Result:=true;
          end;
      end;


    function TX86AsmOptimizer.PostPeepholeOptTestOr(var p : tai) : Boolean;
      var
        IsTestConstX : Boolean;
        hp1,hp2 : tai;
      begin
        Result:=false;
        { removes the line marked with (x) from the sequence
          and/or/xor/add/sub/... $x, %y
          test/or %y, %y  | test $-1, %y    (x)
          j(n)z _Label
             as the first instruction already adjusts the ZF
             %y operand may also be a reference }
        IsTestConstX:=(taicpu(p).opcode=A_TEST) and
          MatchOperand(taicpu(p).oper[0]^,-1);
        if (OpsEqual(taicpu(p).oper[0]^,taicpu(p).oper[1]^) or IsTestConstX) and
           GetLastInstruction(p, hp1) and
           (tai(hp1).typ = ait_instruction) and
           GetNextInstruction(p,hp2) and
           MatchInstruction(hp2,A_SETcc,A_Jcc,A_CMOVcc,[]) then
          case taicpu(hp1).opcode Of
            A_ADD, A_SUB, A_OR, A_XOR, A_AND:
              begin
                if OpsEqual(taicpu(hp1).oper[1]^,taicpu(p).oper[1]^) and
                  { does not work in case of overflow for G(E)/L(E)/C_O/C_NO }
                  { and in case of carry for A(E)/B(E)/C/NC                  }
                   ((taicpu(hp2).condition in [C_Z,C_NZ,C_E,C_NE]) or
                    ((taicpu(hp1).opcode <> A_ADD) and
                     (taicpu(hp1).opcode <> A_SUB))) then
                  begin
                    hp1 := tai(p.next);
                    asml.remove(p);
                    p.free;
                    p := tai(hp1);
                    Result:=true;
                  end;
              end;
            A_SHL, A_SAL, A_SHR, A_SAR:
              begin
                if OpsEqual(taicpu(hp1).oper[1]^,taicpu(p).oper[1]^) and
                  { SHL/SAL/SHR/SAR with a value of 0 do not change the flags }
                  { therefore, it's only safe to do this optimization for     }
                  { shifts by a (nonzero) constant                            }
                   (taicpu(hp1).oper[0]^.typ = top_const) and
                   (taicpu(hp1).oper[0]^.val <> 0) and
                  { does not work in case of overflow for G(E)/L(E)/C_O/C_NO }
                  { and in case of carry for A(E)/B(E)/C/NC                  }
                   (taicpu(hp2).condition in [C_Z,C_NZ,C_E,C_NE]) then
                  begin
                    hp1 := tai(p.next);
                    asml.remove(p);
                    p.free;
                    p := tai(hp1);
                    Result:=true;
                  end;
              end;
            A_DEC, A_INC, A_NEG:
              begin
                if OpsEqual(taicpu(hp1).oper[0]^,taicpu(p).oper[1]^) and
                  { does not work in case of overflow for G(E)/L(E)/C_O/C_NO }
                  { and in case of carry for A(E)/B(E)/C/NC                  }
                  (taicpu(hp2).condition in [C_Z,C_NZ,C_E,C_NE]) then
                  begin
                    case taicpu(hp1).opcode of
                      A_DEC, A_INC:
                        { replace inc/dec with add/sub 1, because inc/dec doesn't set the carry flag }
                        begin
                          case taicpu(hp1).opcode Of
                            A_DEC: taicpu(hp1).opcode := A_SUB;
                            A_INC: taicpu(hp1).opcode := A_ADD;
                            else
                              ;
                          end;
                          taicpu(hp1).loadoper(1,taicpu(hp1).oper[0]^);
                          taicpu(hp1).loadConst(0,1);
                          taicpu(hp1).ops:=2;
                        end;
                      else
                        ;
                    end;
                    hp1 := tai(p.next);
                    asml.remove(p);
                    p.free;
                    p := tai(hp1);
                    Result:=true;
                  end;
              end
          else
            { change "test  $-1,%reg" into "test %reg,%reg" }
            if IsTestConstX and (taicpu(p).oper[1]^.typ=top_reg) then
              taicpu(p).loadoper(0,taicpu(p).oper[1]^);
          end { case }
        { change "test  $-1,%reg" into "test %reg,%reg" }
        else if IsTestConstX and (taicpu(p).oper[1]^.typ=top_reg) then
          taicpu(p).loadoper(0,taicpu(p).oper[1]^);
      end;


    function TX86AsmOptimizer.PostPeepholeOptCall(var p : tai) : Boolean;
      var
        hp1 : tai;
{$ifndef x86_64}
        hp2 : taicpu;
{$endif x86_64}
      begin
        Result:=false;
{$ifndef x86_64}
        { don't do this on modern CPUs, this really hurts them due to
          broken call/ret pairing }
        if (current_settings.optimizecputype < cpu_Pentium2) and
           not(cs_create_pic in current_settings.moduleswitches) and
           GetNextInstruction(p, hp1) and
           MatchInstruction(hp1,A_JMP,[S_NO]) and
           MatchOpType(taicpu(hp1),top_ref) and
           (taicpu(hp1).oper[0]^.ref^.refaddr=addr_full) then
          begin
            hp2 := taicpu.Op_sym(A_PUSH,S_L,taicpu(hp1).oper[0]^.ref^.symbol);
            InsertLLItem(p.previous, p, hp2);
            taicpu(p).opcode := A_JMP;
            taicpu(p).is_jmp := true;
            asml.remove(hp1);
            hp1.free;
            Result:=true;
          end
        else
{$endif x86_64}
        { replace
            call   procname
            ret
          by
            jmp    procname

          but do it only on level 4 because it destroys stack back traces
        }
        if (cs_opt_level4 in current_settings.optimizerswitches) and
          GetNextInstruction(p, hp1) and
          MatchInstruction(hp1,A_RET,[S_NO]) and
          (taicpu(hp1).ops=0) then
          begin
            taicpu(p).opcode := A_JMP;
            taicpu(p).is_jmp := true;
            DebugMsg(SPeepholeOptimization + 'CallRet2Jmp done',p);
            asml.remove(hp1);
            hp1.free;
            Result:=true;
          end;
      end;


{$ifdef x86_64}
    function TX86AsmOptimizer.PostPeepholeOptMovzx(var p : tai) : Boolean;
      var
        PreMessage: string;
      begin
        Result := False;
        { Code size reduction by J. Gareth "Kit" Moreton }
        { Convert MOVZBQ and MOVZWQ to MOVZBL and MOVZWL respectively if it removes the REX prefix }
        if (taicpu(p).opsize in [S_BQ, S_WQ]) and
          (getsupreg(taicpu(p).oper[1]^.reg) in [RS_RAX, RS_RCX, RS_RDX, RS_RBX, RS_RSI, RS_RDI, RS_RBP, RS_RSP])
        then
          begin
            { Has 64-bit register name and opcode suffix }
            PreMessage := 'movz' + debug_opsize2str(taicpu(p).opsize) + ' ' + debug_operstr(taicpu(p).oper[0]^) + ',' + debug_regname(taicpu(p).oper[1]^.reg) + ' -> movz';

            { The actual optimization }
            setsubreg(taicpu(p).oper[1]^.reg, R_SUBD);
            if taicpu(p).opsize = S_BQ then
              taicpu(p).changeopsize(S_BL)
            else
              taicpu(p).changeopsize(S_WL);

            DebugMsg(SPeepholeOptimization + PreMessage +
              debug_opsize2str(taicpu(p).opsize) + ' ' + debug_operstr(taicpu(p).oper[0]^) + ',' + debug_regname(taicpu(p).oper[1]^.reg) + ' (removes REX prefix)', p);
          end;
      end;


    function TX86AsmOptimizer.PostPeepholeOptXor(var p : tai) : Boolean;
      var
        PreMessage, RegName: string;
      begin
        { Code size reduction by J. Gareth "Kit" Moreton }
        { change "xorq %reg,%reg" to "xorl %reg,%reg" for %rax, %rcx, %rdx, %rbx, %rsi, %rdi, %rbp and %rsp,
          as this removes the REX prefix }

        Result := False;
        if not OpsEqual(taicpu(p).oper[0]^,taicpu(p).oper[1]^) then
          Exit;

        if taicpu(p).oper[0]^.typ <> top_reg then
          { Should be impossible if both operands were equal, since one of XOR's operands must be a register }
          InternalError(2018011500);

        case taicpu(p).opsize of
          S_Q:
            begin
              if (getsupreg(taicpu(p).oper[0]^.reg) in [RS_RAX, RS_RCX, RS_RDX, RS_RBX, RS_RSI, RS_RDI, RS_RBP, RS_RSP]) then
                begin
                  RegName := debug_regname(taicpu(p).oper[0]^.reg); { 64-bit register name }
                  PreMessage := 'xorq ' + RegName + ',' + RegName + ' -> xorl ';

                  { The actual optimization }
                  setsubreg(taicpu(p).oper[0]^.reg, R_SUBD);
                  setsubreg(taicpu(p).oper[1]^.reg, R_SUBD);
                  taicpu(p).changeopsize(S_L);

                  RegName := debug_regname(taicpu(p).oper[0]^.reg); { 32-bit register name }

                  DebugMsg(SPeepholeOptimization + PreMessage + RegName + ',' + RegName + ' (removes REX prefix)', p);
                end;
            end;
          else
            ;
        end;
      end;
{$endif}


    procedure TX86AsmOptimizer.OptReferences;
      var
        p: tai;
        i: Integer;
      begin
        p := BlockStart;
        while (p <> BlockEnd) Do
          begin
            if p.typ=ait_instruction then
              begin
                for i:=0 to taicpu(p).ops-1 do
                  if taicpu(p).oper[i]^.typ=top_ref then
                    optimize_ref(taicpu(p).oper[i]^.ref^,false);
              end;
            p:=tai(p.next);
          end;
      end;

end.

