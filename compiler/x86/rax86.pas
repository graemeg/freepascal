{
    Copyright (c) 1998-2002 by Carl Eric Codere and Peter Vreman

    Handles the common x86 assembler reader routines

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
{
  Contains the common x86 (i386 and x86-64) assembler reader routines.
}
unit rax86;

{$i fpcdefs.inc}

interface

uses
  aasmbase,aasmtai,aasmdata,aasmcpu,
  cpubase,rautils,cclasses;

{ Parser helpers }
function is_prefix(t:tasmop):boolean;
function is_override(t:tasmop):boolean;
Function CheckPrefix(prefixop,op:tasmop): Boolean;
Function CheckOverride(overrideop,op:tasmop): Boolean;
Procedure FWaitWarning;

type
  Tx86Operand=class(TOperand)
    opsize  : topsize;
    vopext  : smallint;      // bitmask: vector-operand extention AVX512 (e.g. vaddps xmm0 {k1} {z})
    vbcst   : byte;
    Procedure SetSize(_size:longint;force:boolean);override;
    Procedure SetCorrectSize(opcode:tasmop);override;
    Function CheckOperand: boolean; override;
    { handles the @Code symbol }
    Procedure SetupCode;
    { handles the @Data symbol }
    Procedure SetupData;

    constructor create; override;
  end;

  { Operands are always in AT&T order.
    Intel reader attaches them right-to-left, then shifts to start with 1 }

  { Tx86Instruction }

  Tx86Instruction=class(TInstruction)
    opsize  : topsize;
    constructor Create(optype : tcoperand);override;
    { Operand sizes }
    procedure AddReferenceSizes; virtual;
    procedure SetInstructionOpsize;
    procedure CheckOperandSizes;
    procedure CheckNonCommutativeOpcodes;
    { Additional actions required by specific reader }
    procedure FixupOpcode;virtual;
    { opcode adding }
    function ConcatInstruction(p : TAsmList) : tai;override;
    function getstring: string;
    { returns true, if the opcode might have an extension as used by AVX512 }
    function MightHaveExtension : boolean;
  end;

const
  AsmPrefixes = 8{$ifdef i8086}+2{$endif i8086};
  AsmPrefix : array[0..AsmPrefixes-1] of TasmOP =(
    A_LOCK,A_REP,A_REPE,A_REPNE,A_REPNZ,A_REPZ,A_XACQUIRE,A_XRELEASE{$ifdef i8086},A_REPC,A_REPNC{$endif i8086}
  );

  AsmOverrides = 6;
  AsmOverride : array[0..AsmOverrides-1] of TasmOP =(
    A_SEGCS,A_SEGES,A_SEGDS,A_SEGFS,A_SEGGS,A_SEGSS
  );

  CondAsmOps=3;
  CondAsmOp:array[0..CondAsmOps-1] of TasmOp=(
    A_CMOVcc, A_Jcc, A_SETcc
  );
  CondAsmOpStr:array[0..CondAsmOps-1] of string[4]=(
    'CMOV','J','SET'
  );

implementation

uses
  globtype,globals,systems,verbose,
  procinfo,
  cgbase,cgutils,
  itcpugas,cgx86, cutils;


{*****************************************************************************
                              Parser Helpers
*****************************************************************************}

function is_prefix(t:tasmop):boolean;
var
  i : longint;
Begin
  is_prefix:=false;
  for i:=1 to AsmPrefixes do
   if t=AsmPrefix[i-1] then
    begin
      is_prefix:=true;
      exit;
    end;
end;


function is_override(t:tasmop):boolean;
var
  i : longint;
Begin
  is_override:=false;
  for i:=1 to AsmOverrides do
   if t=AsmOverride[i-1] then
    begin
      is_override:=true;
      exit;
    end;
end;


Function CheckPrefix(prefixop,op:tasmop): Boolean;
{ Checks if the prefix is valid with the following opcode }
{ return false if not, otherwise true                          }
Begin
  CheckPrefix := TRUE;
(*  Case prefix of
    A_REP,A_REPNE,A_REPE:
      Case opcode Of
        A_SCASB,A_SCASW,A_SCASD,
        A_INS,A_OUTS,A_MOVS,A_CMPS,A_LODS,A_STOS:;
        Else
          Begin
            CheckPrefix := FALSE;
            exit;
          end;
      end; { case }
    A_LOCK:
      Case opcode Of
        A_BT,A_BTS,A_BTR,A_BTC,A_XCHG,A_ADD,A_OR,A_ADC,A_SBB,A_AND,A_SUB,
        A_XOR,A_NOT,A_NEG,A_INC,A_DEC:;
        Else
          Begin
            CheckPrefix := FALSE;
            Exit;
          end;
      end; { case }
    A_NONE: exit; { no prefix here }
    else
      CheckPrefix := FALSE;
   end; { end case } *)
end;


Function CheckOverride(overrideop,op:tasmop): Boolean;
{ Check if the override is valid, and if so then }
{ update the instr variable accordingly.         }
Begin
  CheckOverride := true;
{     Case instr.getinstruction of
    A_MOVS,A_XLAT,A_CMPS:
      Begin
        CheckOverride := TRUE;
        Message(assem_e_segment_override_not_supported);
      end
  end }
end;


Procedure FWaitWarning;
begin
  if (target_info.system=system_i386_GO32V2) and (cs_fp_emulation in current_settings.moduleswitches) then
   Message(asmr_w_fwait_emu_prob);
end;

{*****************************************************************************
                              TX86Operand
*****************************************************************************}

Procedure Tx86Operand.SetSize(_size:longint;force:boolean);
begin
  inherited SetSize(_size,force);
  { OS_64 will be set to S_L and be fixed later
    in SetCorrectSize }

  // multimedia register
  case _size of
    16: size := OS_M128;
    32: size := OS_M256;
    64: size := OS_M512;
  end;

{$ifdef i8086}
  { allows e.g. using 32-bit registers in i8086 inline asm }
  if size in [OS_32,OS_S32] then
    opsize:=S_L
  else
{$endif i8086}
    opsize:=TCGSize2Opsize[size];
end;


Procedure Tx86Operand.SetCorrectSize(opcode:tasmop);
begin
  if gas_needsuffix[opcode]=attsufFPU then
    begin
     case size of
       OS_32 : opsize:=S_FS;
       OS_64 : opsize:=S_FL;
       else
         ;
     end;
    end
  else if gas_needsuffix[opcode]=attsufFPUint then
    begin
      case size of
        OS_16 : opsize:=S_IS;
        OS_32 : opsize:=S_IL;
        OS_64 : opsize:=S_IQ;
        else
          ;
      end;
    end
  else if gas_needsuffix[opcode]=AttSufMM then
  begin
    if (opr.typ=OPR_Reference) then
    begin
      case size of
        OS_32 : size := OS_M32;
        OS_64 : size := OS_M64;
        else
          ;
      end;
    end;
  end
  else
    begin
      if size=OS_64 then
        opsize:=S_Q;
    end;
end;

Function Tx86Operand.CheckOperand: boolean;
var
  ErrorRefStr: string;
begin
  result:=true;
  if (opr.typ=OPR_Reference) then
    begin
      if not hasvar then
        begin
          if (getsupreg(opr.ref.base)=RS_EBP) and (opr.ref.offset>0) then
            begin
              if current_settings.asmmode in asmmodes_x86_intel then
                begin
                  case getsubreg(opr.ref.base) of
                    R_SUBW:
                      ErrorRefStr:='[BP+offset]';
                    R_SUBD:
                      ErrorRefStr:='[EBP+offset]';
                    R_SUBQ:
                      ErrorRefStr:='[RBP+offset]';
                    else
                      internalerror(2019061001);
                  end;
                end
              else
                begin
                  case getsubreg(opr.ref.base) of
                    R_SUBW:
                      ErrorRefStr:='+offset(%bp)';
                    R_SUBD:
                      ErrorRefStr:='+offset(%ebp)';
                    R_SUBQ:
                      ErrorRefStr:='+offset(%rbp)';
                    else
                      internalerror(2019061002);
                  end;
                end;
              if current_procinfo.procdef.proccalloption=pocall_register then
                message1(asmr_w_no_direct_ebp_for_parameter,ErrorRefStr)
              else
                message1(asmr_w_direct_ebp_for_parameter_regcall,ErrorRefStr);
            end
          else if (getsupreg(opr.ref.base)=RS_EBP) and (opr.ref.offset<0) then
            begin
              if current_settings.asmmode in asmmodes_x86_intel then
                begin
                  case getsubreg(opr.ref.base) of
                    R_SUBW:
                      ErrorRefStr:='[BP-offset]';
                    R_SUBD:
                      ErrorRefStr:='[EBP-offset]';
                    R_SUBQ:
                      ErrorRefStr:='[RBP-offset]';
                    else
                      internalerror(2019061003);
                  end;
                end
              else
                begin
                  case getsubreg(opr.ref.base) of
                    R_SUBW:
                      ErrorRefStr:='-offset(%bp)';
                    R_SUBD:
                      ErrorRefStr:='-offset(%ebp)';
                    R_SUBQ:
                      ErrorRefStr:='-offset(%rbp)';
                    else
                      internalerror(2019061004);
                  end;
                end;
              message1(asmr_w_direct_ebp_neg_offset,ErrorRefStr);
            end
          else if (getsupreg(opr.ref.base)=RS_ESP) and (getsubreg(opr.ref.base)<>R_SUBW) and (opr.ref.offset<0) then
            begin
              if current_settings.asmmode in asmmodes_x86_intel then
                begin
                  case getsubreg(opr.ref.base) of
                    R_SUBD:
                      ErrorRefStr:='[ESP-offset]';
                    R_SUBQ:
                      ErrorRefStr:='[RSP-offset]';
                    else
                      internalerror(2019061005);
                  end;
                end
              else
                begin
                  case getsubreg(opr.ref.base) of
                    R_SUBD:
                      ErrorRefStr:='-offset(%esp)';
                    R_SUBQ:
                      ErrorRefStr:='-offset(%rsp)';
                    else
                      internalerror(2019061006);
                  end;
                end;
              message1(asmr_w_direct_esp_neg_offset,ErrorRefStr);
            end;
        end;
      if (cs_create_pic in current_settings.moduleswitches) and
         assigned(opr.ref.symbol) and
         not assigned(opr.ref.relsymbol) then
        begin
          if not(opr.ref.refaddr in [addr_pic,addr_pic_no_got]) then
            begin
              if (opr.ref.symbol.name <> '_GLOBAL_OFFSET_TABLE_') then
                begin
                  message(asmr_e_need_pic_ref);
                  result:=false;
                end
              else
                opr.ref.refaddr:=addr_pic;
            end
          else
            begin
{$ifdef x86_64}
              { should probably be extended to i386, but there the situation
                is more complex and ELF-style PIC still need to be
                tested/debugged }
              if (opr.ref.symbol.bind in [AB_LOCAL,AB_PRIVATE_EXTERN]) and
                 (opr.ref.refaddr=addr_pic) then
                message(asmr_w_useless_got_for_local)
              else if (opr.ref.symbol.bind in [AB_GLOBAL,AB_EXTERNAL,AB_COMMON,AB_WEAK_EXTERNAL]) and
                 (opr.ref.refaddr=addr_pic_no_got) then
                message(asmr_w_global_access_without_got);
{$endif x86_64}
            end;
        end;
    end;
end;


procedure Tx86Operand.SetupCode;
begin
{$ifdef i8086}
  opr.typ:=OPR_SYMBOL;
  opr.symofs:=0;
  opr.symbol:=current_asmdata.RefAsmSymbol(current_procinfo.procdef.mangledname,AT_FUNCTION);
  opr.symseg:=true;
  opr.sym_farproc_entry:=false;
{$else i8086}
  Message(asmr_w_CODE_and_DATA_not_supported);
{$endif i8086}
end;


procedure Tx86Operand.SetupData;
begin
{$ifdef i8086}
  InitRef;
  if current_settings.x86memorymodel=mm_huge then
    opr.ref.refaddr:=addr_fardataseg
  else
    opr.ref.refaddr:=addr_dgroup;
{$else i8086}
  Message(asmr_w_CODE_and_DATA_not_supported);
{$endif i8086}
end;

constructor Tx86Operand.create;
begin
  inherited;

  vopext  := 0;
  vbcst   := 0;
end;

{*****************************************************************************
                              T386Instruction
*****************************************************************************}

constructor Tx86Instruction.Create(optype : tcoperand);
begin
  inherited Create(optype);
  Opsize:=S_NO;
end;

procedure Tx86Instruction.AddReferenceSizes;
{ this will add the sizes for references like [esi] which do not
  have the size set yet, it will take only the size if the other
  operand is a register }
var
  operand2,i,j,k : longint;
  t: topsize;
  s : tasmsymbol;
  so : aint;
  ExistsMemRefNoSize: boolean;
  ExistsMemRef: boolean;
  ExistsConstNoSize: boolean;
  ExistConst: boolean;
  ExistsLocalSymSize: boolean;
  ExistsBCST: boolean;
  memrefsize: integer;
  memopsize: integer;
  memoffset: asizeint;
  vbcst: byte;
  mmregs: Set of TSubregister;
  multiplicator: integer;
  bcst1,bcst2: string;
begin
  ExistsMemRefNoSize := false;
  ExistsMemRef       := false;
  ExistsConstNoSize  := false;
  ExistsLocalSymSize := false;
  ExistsBCST         := false;

  // EXIST A MEMORY- OR CONSTANT-OPERAND WITHOUT SIZE ?
  for i := 1 to ops do
  begin
    if operands[i].Opr.Typ in [OPR_REFERENCE, OPR_LOCAL] then
    begin
      ExistsMemRef := true;

      ExistsBCST := (MemRefInfo(opcode).ExistsSSEAVX) and
                    (tx86operand(operands[i]).vbcst <> 0);

      if (tx86operand(operands[i]).opsize = S_NO) then
      begin
        ExistsMemRefNoSize := true;

        case operands[i].opr.Typ of
              OPR_LOCAL: ExistsLocalSymSize := tx86operand(operands[i]).opr.localsym.getsize > 0;
          OPR_REFERENCE: ExistsLocalSymSize := true;
          else
            ;
        end;
      end;
    end
    else if operands[i].Opr.Typ in [OPR_CONSTANT] then
    begin
      ExistsConstNoSize := tx86operand(operands[i]).opsize = S_NO;
    end;
  end;

  // ONLY SUPPORTED OPCODES WITH SSE- OR AVX-REGISTERS
  if (ExistsMemRef) and
     (MemRefInfo(opcode).ExistsSSEAVX) then
  begin
    // 1. WE HAVE AN SSE- OR AVX-OPCODE WITH MEMORY OPERAND
    if (not(ExistsMemRefNoSize)) or
       (ExistsLocalSymSize) then
    begin
      // 2. WE KNOWN THE MEMORYSIZE OF THE MEMORY-OPERAND OR WE CAN
      //    CALC THE MEMORYSIZE

      // 3. CALC THE SIZE OF THE MEMORYOPERAND BY OPCODE-DEFINITION
      // 4. COMPARE THE SIZE FROM OPCODE-DEFINITION AND THE REAL MEMORY-OPERAND-SIZE

      // - validate memory-reference-size
      for i := 1 to ops do
      begin
        if (operands[i].Opr.Typ in [OPR_REFERENCE, OPR_LOCAL]) then
        begin
          memrefsize := -1;

          if ExistsBCST then
          begin
            case MemRefInfo(opcode).MemRefSizeBCST of
              msbBCST32: memrefsize := 32;
              msbBCST64: memrefsize := 64;
              else
                Internalerror(2019081005);
            end;
          end
          else
          case MemRefInfo(opcode).MemRefSize of
            msiMem8: memrefsize := 8;
            msiMem16: memrefsize := 16;
            msiMem32: memrefsize := 32;
            msiMem64: memrefsize := 64;
            msiMem128: memrefsize := 128;
            msiMem256: memrefsize := 256;
            msiMem512: memrefsize := 512;
            msiMemRegx16y32:
              begin
                for j := 1 to ops do
                begin
                  if operands[j].Opr.Typ = OPR_REGISTER then
                  begin
                    case getsubreg(operands[j].opr.reg) of
                      R_SUBMMX: memrefsize := 16;
                      R_SUBMMY: memrefsize := 32;
                      else Message(asmr_e_unable_to_determine_reference_size);
                    end;
                  end;
                end;
              end;
            msiMemRegx32y64:
              begin
                for j := 1 to ops do
                begin
                  if operands[j].Opr.Typ = OPR_REGISTER then
                  begin
                    case getsubreg(operands[j].opr.reg) of
                      R_SUBMMX: memrefsize := 32;
                      R_SUBMMY: memrefsize := 64;
                      else Message(asmr_e_unable_to_determine_reference_size);
                    end;
                  end;
                end;
              end;
           msiMemRegx64y128:
              begin
                for j := 1 to ops do
                begin
                  if operands[j].Opr.Typ = OPR_REGISTER then
                  begin
                    case getsubreg(operands[j].opr.reg) of
                      R_SUBMMX: memrefsize := 64;
                      R_SUBMMY: memrefsize := 128;
                      else Message(asmr_e_unable_to_determine_reference_size);
                    end;
                  end;
                end;
              end;
           msiMemRegx64y256:
              begin
                for j := 1 to ops do
                begin
                  if operands[j].Opr.Typ = OPR_REGISTER then
                  begin
                    case getsubreg(operands[j].opr.reg) of
                      R_SUBMMX: memrefsize := 64;
                      R_SUBMMY: memrefsize := 256;
                      else Message(asmr_e_unable_to_determine_reference_size);
                    end;
                  end;
                end;
              end;
            msiMemRegx64y128z256:
              begin
                begin
                  for j := 1 to ops do
                  begin
                    if operands[j].Opr.Typ = OPR_REGISTER then
                    begin
                      case getsubreg(operands[j].opr.reg) of
                        R_SUBMMX: memrefsize := 64;
                        R_SUBMMY: memrefsize := 128;
                        R_SUBMMZ: memrefsize := 256;
                        else Message(asmr_e_unable_to_determine_reference_size);
                      end;
                    end;
                  end;
                end;
              end;
            msiMemRegx64y256z512:
              begin
                begin
                  for j := 1 to ops do
                  begin
                    if operands[j].Opr.Typ = OPR_REGISTER then
                    begin
                      case getsubreg(operands[j].opr.reg) of
                        R_SUBMMX: memrefsize := 64;
                        R_SUBMMY: memrefsize := 256;
                        R_SUBMMZ: memrefsize := 512;
                        else Message(asmr_e_unable_to_determine_reference_size);
                      end;
                    end;
                  end;
                end;
              end;
            msiMemRegSize:
              begin
                for j := 1 to ops do
                   begin
                     if operands[j].Opr.Typ = OPR_REGISTER then
                     begin
                       if (tx86operand(operands[j]).opsize <> S_NO) and
                          (tx86operand(operands[j]).size <> OS_NO) then
                       begin
                         case tx86operand(operands[j]).opsize of
                           S_B   : memrefsize := 8;
                           S_W   : memrefsize := 16;
                           S_L   : memrefsize := 32;
                           S_Q   : memrefsize := 64;
                           S_XMM : memrefsize := 128;
                           S_YMM : memrefsize := 256;
                           S_ZMM : memrefsize := 512;
                           else Internalerror(2019081001);
                         end;
                         break;
                       end;
                     end;
                   end;
              end;
            msiMemRegConst128,
            msiMemRegConst256,
            msiMemRegConst512:
              begin
                for j := 1 to ops do
                begin
                  if operands[j].Opr.Typ = OPR_CONSTANT then
                  begin
                    for k := 1 to ops do
                     begin
                       if operands[k].Opr.Typ = OPR_REGISTER then
                       begin
                         if (tx86operand(operands[k]).opsize <> S_NO) and
                            (tx86operand(operands[k]).size <> OS_NO) then
                         begin
                           case tx86operand(operands[k]).opsize of
                             S_B   : memrefsize := 8;
                             S_W   : memrefsize := 16;
                             S_L   : memrefsize := 32;
                             S_Q   : memrefsize := 64;
                             S_XMM : memrefsize := 128;
                             S_YMM : memrefsize := 256;
                             S_ZMM : memrefsize := 512;
                             else Internalerror(777200);
                           end;
                           break;
                         end;
                       end;
                     end;

                    break;
                  end;
                end;

                // no exists const-operand
                if memrefsize = -1 then
                begin
                  case MemRefInfo(opcode).MemRefSize of
                    msiMemRegConst128: memrefsize := 128;
                    msiMemRegConst256: memrefsize := 256;
                    msiMemRegConst512: memrefsize := 512;
                    else Internalerror(2019081002);
                  end;
                end;
              end;
            msiXMem32,
            msiYMem32,
            msiZMem32,
            msiXMem64,
            msiYMem64,
            msiZMem64: ; // ignore;  gather/scatter opcodes haven a fixed element-size, not a fixed memory-size
                         // the vector-register have indices with base of the memory-address in the memory-operand
            msiMultipleMinSize8,
            msiMultipleMinSize16,
            msiMultipleMinSize32,
            msiMultipleMinSize64,
            msiMultipleMinSize128,
            msiMultipleMinSize256,
            msiMultipleMinSize512: ; // ignore
            msiNoSize,
            msiNoMemRef,
            msiUnknown,
            msiUnsupported,
            msiVMemMultiple,
            msiVMemRegSize,
            msiMultiple:
              ;
            else
              Internalerror(2019081005);
          end;

          if memrefsize > -1 then
          begin
            // CALC REAL-MEMORY-OPERAND-SIZE AND A POSSIBLE OFFSET

            // OFFSET:
            // e.g. PAND  XMM0, [RAX + 16] =>> OFFSET = 16 BYTES
            //      PAND  XMM0, [RAX + a.b + 10] =>> OFFSET = 10 BYTES   (a = record-variable)

            memopsize := 0;
            case operands[i].opr.typ of
                  OPR_LOCAL: memopsize := operands[i].opr.localvarsize * 8;
              OPR_REFERENCE:
                  if operands[i].opr.ref.refaddr = addr_pic then
                    memopsize := sizeof(pint) * 8
                  else
                    memopsize := operands[i].opr.varsize * 8;
              else
                ;
            end;

            //if memopsize = 0 then memopsize := topsize2memsize[tx86operand(operands[i]).opsize];

            if memopsize = 0 then
            begin
              {$ifdef i386}
                { 64-bit operands are allowed for SSE and AVX instructions, so
                  go by the byte size instead for these families of opcodes }
                if (MemRefInfo(opcode).ExistsSSEAVX) then
                begin
                  memopsize := tx86operand(operands[i]).typesize * 8;
                  if tx86operand(operands[i]).typesize = 8 then
                  { Will be S_L otherwise and won't be corrected in time }
                   tx86operand(operands[i]).opsize := S_Q;
                end
                else
              {$endif i386}
                memopsize := topsize2memsize[tx86operand(operands[i]).opsize];
            end;


            if (memopsize > 0) and
               (memrefsize > 0) then
            begin
              memoffset := 0;

              case operands[i].opr.typ of
                OPR_LOCAL:
                   memoffset := operands[i].opr.localconstoffset;
                OPR_REFERENCE:
                   memoffset := operands[i].opr.constoffset;
                else
                  ;
              end;

              if memoffset < 0 then
              begin
                Message2(asmr_w_check_mem_operand_negative_offset,
                         std_op2str[opcode],
                         ToStr(memoffset));
              end
              else if ((tx86operand(operands[i]).hastype) and (memopsize < memrefsize)) or
                      (memopsize < (memrefsize + memoffset * 8)) then
              begin
                if memopsize < memrefsize then
                begin
                  if memoffset = 0 then
                  begin
                    Message3(asmr_w_check_mem_operand_size3,
                             std_op2str[opcode],
                             ToStr(memopsize),
                             ToStr(memrefsize)
                             );
                  end
                  else
                  begin
                    Message4(asmr_w_check_mem_operand_size_offset,
                             std_op2str[opcode],
                             ToStr(memopsize),
                             ToStr(memrefsize),
                             ToStr(memoffset)
                             );
                  end;
                end;
              end;
            end;
          end;
        end;
      end;
    end;
  end;

  if (ExistsMemRefNoSize or ExistsConstNoSize) and
     (MemRefInfo(opcode).ExistsSSEAVX) then
  begin
    for i := 1 to ops do
    begin
      if (tx86operand(operands[i]).opsize = S_NO) then
      begin
        case operands[i].Opr.Typ of
          OPR_REFERENCE:
                begin
                  if ExistsBCST then
                  begin
                    case MemRefInfo(opcode).MemRefSizeBCST of
                      msbBCST32: begin
                                   tx86operand(operands[i]).opsize := S_L;
                                   tx86operand(operands[i]).size   := OS_32;
                                 end;
                      msbBCST64: begin
                                   tx86operand(operands[i]).opsize := S_Q;
                                   tx86operand(operands[i]).size   := OS_M64;
                                 end;
                      else
                        Internalerror(2019081006);
                    end;
                  end
                  else

                  case MemRefInfo(opcode).MemRefSize of
                      msiMem8:
                              begin
                                tx86operand(operands[i]).opsize := S_B;
                                tx86operand(operands[i]).size   := OS_8;
                              end;
                      msiMultipleMinSize8:
                              begin
                                tx86operand(operands[i]).opsize := S_B;
                                tx86operand(operands[i]).size   := OS_8;

                                Message2(asmr_w_check_mem_operand_automap_multiple_size, std_op2str[opcode], '"8 bit memory operand"');
                              end;
                      msiMem16:
                              begin
                                tx86operand(operands[i]).opsize := S_W;
                                tx86operand(operands[i]).size   := OS_16;
                              end;
                      msiMultipleMinSize16:
                               begin
                                 tx86operand(operands[i]).opsize := S_W;
                                 tx86operand(operands[i]).size   := OS_16;

                                 Message2(asmr_w_check_mem_operand_automap_multiple_size, std_op2str[opcode], '"16 bit memory operand"');
                               end;
                      msiMem32:
                               begin
                                 tx86operand(operands[i]).opsize := S_L;
                                 tx86operand(operands[i]).size   := OS_32;
                               end;
                      msiMultipleMinSize32:
                               begin
                                 tx86operand(operands[i]).opsize := S_L;
                                 tx86operand(operands[i]).size   := OS_32;

                                 Message2(asmr_w_check_mem_operand_automap_multiple_size, std_op2str[opcode], '"32 bit memory operand"');
                               end;
                      msiMem64:
                               begin
                                 tx86operand(operands[i]).opsize := S_Q;
                                 tx86operand(operands[i]).size   := OS_M64;
                               end;
                      msiMultipleMinSize64:
                               begin
                                 tx86operand(operands[i]).opsize := S_Q;
                                 tx86operand(operands[i]).size   := OS_M64;

                                 Message2(asmr_w_check_mem_operand_automap_multiple_size, std_op2str[opcode], '"64 bit memory operand"');
                               end;
                      msiMem128:
                               begin
                                 tx86operand(operands[i]).opsize := S_XMM;
                                 tx86operand(operands[i]).size   := OS_M128;
                               end;
                      msiMultipleMinSize128:
                               begin
                                 tx86operand(operands[i]).opsize := S_XMM;
                                 tx86operand(operands[i]).size   := OS_M128;

                                 Message2(asmr_w_check_mem_operand_automap_multiple_size, std_op2str[opcode], '"128 bit memory operand"');
                               end;
                      msiMem256:
                               begin
                                 tx86operand(operands[i]).opsize := S_YMM;
                                 tx86operand(operands[i]).size   := OS_M256;
                                 opsize := S_YMM;
                               end;
                      msiMultipleMinSize256:
                               begin
                                 tx86operand(operands[i]).opsize := S_YMM;
                                 tx86operand(operands[i]).size   := OS_M256;
                                 opsize := S_YMM;

                                 Message2(asmr_w_check_mem_operand_automap_multiple_size, std_op2str[opcode], '"256 bit memory operand"');
                               end;
                      msiMem512:
                               begin
                                 tx86operand(operands[i]).opsize := S_ZMM;
                                 tx86operand(operands[i]).size   := OS_M512;
                                 opsize := S_ZMM;
                               end;
                      msiMultipleMinSize512:
                               begin
                                 tx86operand(operands[i]).opsize := S_ZMM;
                                 tx86operand(operands[i]).size   := OS_M512;
                                 opsize := S_ZMM;

                                 Message2(asmr_w_check_mem_operand_automap_multiple_size, std_op2str[opcode], '"512 bit memory operand"');
                               end;

                    msiMemRegSize:
                               begin
                                 // mem-ref-size = register size
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_REGISTER then
                                   begin
                                     if (tx86operand(operands[j]).opsize <> S_NO) and
                                        (tx86operand(operands[j]).size <> OS_NO) then
                                     begin
                                       tx86operand(operands[i]).opsize := tx86operand(operands[j]).opsize;
                                       tx86operand(operands[i]).size   := tx86operand(operands[j]).size;
                                       break;
                                     end
                                     else Message(asmr_e_unable_to_determine_reference_size);
                                   end;
                                 end;
                               end;
                      msiMemRegx16y32:
                               begin
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_REGISTER then
                                   begin
                                     case getsubreg(operands[j].opr.reg) of
                                       R_SUBMMX: begin
                                                   tx86operand(operands[i]).opsize := S_W;
                                                   tx86operand(operands[i]).size   := OS_M16;
                                                   break;
                                                 end;
                                       R_SUBMMY: begin
                                                   tx86operand(operands[i]).opsize := S_L;
                                                   tx86operand(operands[i]).size   := OS_M32;
                                                   break;
                                                 end;
                                            else Message(asmr_e_unable_to_determine_reference_size);
                                     end;
                                   end;
                                 end;
                               end;
                      msiMemRegx16y32z64:
                               begin
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_REGISTER then
                                   begin
                                     case getsubreg(operands[j].opr.reg) of
                                       R_SUBMMX: begin
                                                   tx86operand(operands[i]).opsize := S_W;
                                                   tx86operand(operands[i]).size   := OS_M16;
                                                   break;
                                                 end;
                                       R_SUBMMY: begin
                                                   tx86operand(operands[i]).opsize := S_L;
                                                   tx86operand(operands[i]).size   := OS_M32;
                                                   break;
                                                 end;
                                       R_SUBMMZ: begin
                                                   tx86operand(operands[i]).opsize := S_Q;
                                                   tx86operand(operands[i]).size   := OS_M64;
                                                   break;
                                                 end;
                                            else Message(asmr_e_unable_to_determine_reference_size);
                                     end;
                                   end;
                                 end;
                               end;
                      msiMemRegx32y64:
                               begin
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_REGISTER then
                                   begin
                                     case getsubreg(operands[j].opr.reg) of
                                       R_SUBMMX: begin
                                                   tx86operand(operands[i]).opsize := S_L;
                                                   tx86operand(operands[i]).size   := OS_M32;
                                                   break;
                                                 end;
                                       R_SUBMMY: begin
                                                   tx86operand(operands[i]).opsize := S_Q;
                                                   tx86operand(operands[i]).size   := OS_M64;
                                                   break;
                                                 end;
                                            else Message(asmr_e_unable_to_determine_reference_size);
                                     end;
                                   end;
                                 end;
                               end;
                     msiMemRegx32y64z128:
                               for j := 1 to ops do
                               begin
                                 if operands[j].Opr.Typ = OPR_REGISTER then
                                 begin
                                   case getsubreg(operands[j].opr.reg) of
                                     R_SUBMMX: begin
                                                 tx86operand(operands[i]).opsize := S_L;
                                                 tx86operand(operands[i]).size   := OS_M32;
                                                 break;
                                               end;
                                     R_SUBMMY: begin
                                                 tx86operand(operands[i]).opsize := S_Q;
                                                 tx86operand(operands[i]).size   := OS_M64;
                                                 break;
                                               end;
                                     R_SUBMMZ: begin
                                                 tx86operand(operands[i]).opsize := S_XMM;
                                                 tx86operand(operands[i]).size   := OS_M128;
                                                 break;
                                               end;
                                          else Message(asmr_e_unable_to_determine_reference_size);
                                   end;
                                 end;
                               end;
                     msiMemRegx64y128:
                               begin
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_REGISTER then
                                   begin
                                     case getsubreg(operands[j].opr.reg) of
                                       R_SUBMMX: begin
                                                   tx86operand(operands[i]).opsize := S_Q;
                                                   tx86operand(operands[i]).size   := OS_M64;
                                                   break;
                                                 end;
                                       R_SUBMMY: begin
                                                   tx86operand(operands[i]).opsize := S_XMM;
                                                   tx86operand(operands[i]).size   := OS_M128;
                                                   break;
                                                 end;
                                            else Message(asmr_e_unable_to_determine_reference_size);
                                     end;
                                   end;
                                 end;
                               end;
                     msiMemRegx64y128z256:
                               begin
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_REGISTER then
                                   begin
                                     case getsubreg(operands[j].opr.reg) of
                                       R_SUBMMX: begin
                                                   tx86operand(operands[i]).opsize := S_Q;
                                                   tx86operand(operands[i]).size   := OS_M64;
                                                   break;
                                                 end;
                                       R_SUBMMY: begin
                                                   tx86operand(operands[i]).opsize := S_XMM;
                                                   tx86operand(operands[i]).size   := OS_M128;
                                                   break;
                                                 end;
                                       R_SUBMMZ: begin
                                                   tx86operand(operands[i]).opsize := S_YMM;
                                                   tx86operand(operands[i]).size   := OS_M256;
                                                   break;
                                                 end;

                                            else Message(asmr_e_unable_to_determine_reference_size);
                                     end;
                                   end;
                                 end;
                               end;
                     msiMemRegx64y256:
                               begin
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_REGISTER then
                                   begin
                                     case getsubreg(operands[j].opr.reg) of
                                       R_SUBMMX: begin
                                                   tx86operand(operands[i]).opsize := S_Q;
                                                   tx86operand(operands[i]).size   := OS_M64;
                                                   break;
                                                 end;
                                       R_SUBMMY: begin
                                                   tx86operand(operands[i]).opsize := S_YMM;
                                                   tx86operand(operands[i]).size   := OS_M256;
                                                   break;
                                                 end;
                                            else Message(asmr_e_unable_to_determine_reference_size);
                                     end;
                                   end;
                                 end;
                               end;
                     msiMemRegx64y256z512:
                               begin
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_REGISTER then
                                   begin
                                     case getsubreg(operands[j].opr.reg) of
                                       R_SUBMMX: begin
                                                   tx86operand(operands[i]).opsize := S_Q;
                                                   tx86operand(operands[i]).size   := OS_M64;
                                                   break;
                                                 end;
                                       R_SUBMMY: begin
                                                   tx86operand(operands[i]).opsize := S_YMM;
                                                   tx86operand(operands[i]).size   := OS_M256;
                                                   break;
                                                 end;
                                       R_SUBMMZ: begin
                                                   tx86operand(operands[i]).opsize := S_ZMM;
                                                   tx86operand(operands[i]).size   := OS_M512;
                                                   break;
                                                 end;
                                            else Message(asmr_e_unable_to_determine_reference_size);
                                     end;
                                   end;
                                 end;
                               end;

                     msiMemRegConst128,
                     msiMemRegConst256,
                     msiMemRegConst512:
                               begin
                                 ExistConst := false;
                                 for j := 1 to ops do
                                 begin
                                   if operands[j].Opr.Typ = OPR_CONSTANT then
                                   begin
                                     ExistConst := true;
                                     break;
                                   end;
                                 end;

                                 if ExistConst then
                                 begin
                                   for j := 1 to ops do
                                   begin
                                     if operands[j].Opr.Typ = OPR_REGISTER then
                                     begin
                                       if (tx86operand(operands[j]).opsize <> S_NO) and
                                          (tx86operand(operands[j]).size <> OS_NO) then
                                       begin
                                         tx86operand(operands[i]).opsize := tx86operand(operands[j]).opsize;
                                         tx86operand(operands[i]).size   := tx86operand(operands[j]).size;
                                         break;
                                       end
                                       else Message(asmr_e_unable_to_determine_reference_size);
                                     end;
                                   end;
                                 end
                                 else
                                 begin
                                   case MemRefInfo(opcode).MemRefSize  of
                                     msiMemRegConst128: begin
                                                          tx86operand(operands[i]).opsize := S_XMM;
                                                          tx86operand(operands[i]).size   := OS_M128;
                                                          break;
                                                        end;
                                     msiMemRegConst256: begin
                                                          tx86operand(operands[i]).opsize := S_YMM;
                                                          tx86operand(operands[i]).size   := OS_M256;
                                                          break;
                                                        end;
                                     msiMemRegConst512: begin
                                                          tx86operand(operands[i]).opsize := S_ZMM;
                                                          tx86operand(operands[i]).size   := OS_M512;
                                                          break;
                                                        end;
                                     else
                                       Internalerror(2019081007);
                                   end;
                                 end;
                               end;

                   msiXMem32,
                   msiYMem32,
                   msiZMem32,
                   msiXMem64,
                   msiYMem64,
                   msiZMem64: ; // ignore;  gather/scatter opcodes haven a fixed element-size, not a fixed memory-size
                                // the vector-register have indices with base of the memory-address in the memory-operand
                   msiNoSize: ; //  all memory-sizes are ok
                   msiNoMemRef:; // ignore;
                   msiVMemMultiple,
                   msiVMemRegSize: ;  // ignore
                   msiUnknown,
                   msiUnsupported,
                   msiMultiple: Message(asmr_e_unable_to_determine_reference_size); // TODO individual message
                  else
                    Internalerror(2019081008);
                end;
              end;
          OPR_CONSTANT:
                case MemRefInfo(opcode).ConstSize of
                   csiMem8: begin
                              tx86operand(operands[i]).opsize := S_B;
                              tx86operand(operands[i]).size   := OS_8;
                            end;
                  csiMem16: begin
                              tx86operand(operands[i]).opsize := S_W;
                              tx86operand(operands[i]).size   := OS_16;
                            end;
                  csiMem32: begin
                              tx86operand(operands[i]).opsize := S_L;
                              tx86operand(operands[i]).size   := OS_32;
                            end;
{$ifdef x86_64}
                  csiMem64: begin
                              tx86operand(operands[i]).opsize := S_Q;
                              tx86operand(operands[i]).size   := OS_64;
                            end;
{$else}
                  csiMem64: begin
                              internalerror(2019050910);
                            end;
{$endif}
                  csiUnknown, csiMultiple, csiNoSize:
                    ;
                end;
          else
            ;
        end;
      end;
    end;
  end;


  for i:=1 to ops do
    begin
      operands[i].SetCorrectSize(opcode);
      if tx86operand(operands[i]).opsize=S_NO then
        begin
{$ifdef x86_64}
          if (opcode=A_MOVQ) and
             (ops=2) and
             (operands[1].opr.typ=OPR_CONSTANT) then
             opsize:=S_Q
          else
{$endif x86_64}
            case operands[i].Opr.Typ of
              OPR_LOCAL,
              OPR_REFERENCE :
                begin
                  { for 3-operand opcodes, operand #1 (in ATT order) is always an immediate,
                    don't consider it. }
                  if i=ops then
                    operand2:=i-1
                  else
                    operand2:=i+1;
                  if operand2>0 then
                   begin
                     { Only allow register as operand to take the size from }
                     if operands[operand2].opr.typ=OPR_REGISTER then
                       begin
                         if ((opcode<>A_MOVD) and
                             (opcode<>A_CVTSI2SS)) then
                      begin
                        //tx86operand(operands[i]).opsize:=tx86operand(operands[operand2]).opsize;

                        // torsten - 31.01.2012
                        // old: xmm/ymm-register operands have a opsize = "S_NO"
                        // new: xmm/ymm-register operands have a opsize = "S_XMM/S_YMM"

                        // any SSE- and AVX-opcodes have mixed operand sizes (e.g. cvtsd2ss xmmreg, xmmreg/m32)
                        // in this case is we need the old handling ("S_NO")
                        // =>> ignore
                        if (tx86operand(operands[operand2]).opsize <> S_XMM) and
                           (tx86operand(operands[operand2]).opsize <> S_YMM) and
                           (tx86operand(operands[operand2]).opsize <> S_ZMM) then

                          tx86operand(operands[i]).opsize:=tx86operand(operands[operand2]).opsize
                        else tx86operand(operands[operand2]).opsize := S_NO;
                      end;
                    end
                     else
                      begin
                        { if no register then take the opsize (which is available with ATT),
                          if not availble then give an error }
                        if opsize<>S_NO then
                          tx86operand(operands[i]).opsize:=opsize
                        else
                         begin
                           if (m_delphi in current_settings.modeswitches) then
                             Message(asmr_w_unable_to_determine_reference_size_using_dword)
                           else
                             Message(asmr_e_unable_to_determine_reference_size);
                           { recovery }
                           tx86operand(operands[i]).opsize:=S_L;
                         end;
                      end;
                   end
                  else
                   begin
                     if opsize<>S_NO then
                       tx86operand(operands[i]).opsize:=opsize
                   end;
                end;
              OPR_SYMBOL :
                begin
                  { Fix lea which need a reference }
                  if opcode=A_LEA then
                   begin
                     s:=operands[i].opr.symbol;
                     so:=operands[i].opr.symofs;
                     operands[i].opr.typ:=OPR_REFERENCE;
                     Fillchar(operands[i].opr.ref,sizeof(treference),0);
                     operands[i].opr.ref.symbol:=s;
                     operands[i].opr.ref.offset:=so;
                   end;
  {$if defined(x86_64)}
                  tx86operand(operands[i]).opsize:=S_Q;
  {$elseif defined(i386)}
                  tx86operand(operands[i]).opsize:=S_L;
  {$elseif defined(i8086)}
                  tx86operand(operands[i]).opsize:=S_W;
  {$endif}
                end;
              else
                ;
            end;
        end;
    end;

  if MemRefInfo(opcode).ExistsSSEAVX then
  begin
    // validate broadcast-memory-operands
    vbcst := 0;
    mmregs := [];

    for i := 1 to ops do
     if operands[i].Opr.Typ in [OPR_REFERENCE, OPR_LOCAL] then vbcst := tx86operand(operands[i]).vbcst
      else if operands[i].Opr.Typ = OPR_REGISTER then
      begin
        if getsubreg(operands[i].opr.reg) in [R_SUBMMX, R_SUBMMY, R_SUBMMZ] then
        begin
          include(mmregs, getsubreg(operands[i].opr.reg));
        end;
      end;

    if vbcst <> 0 then
    begin
      // found broadcast-memory-operand  (e.g. "{1to8}")
      // check is correct

      multiplicator := 0;
      if mmregs = [R_SUBMMX] then multiplicator := 1
       else if mmregs = [R_SUBMMY] then multiplicator := 2
       else if mmregs = [R_SUBMMZ] then multiplicator := 4
       else
        begin
          //TG TODO

        end;

      if MemRefInfo(opcode).BCSTTypes <> [] then
      begin
        str(MemRefInfo(opcode).BCSTXMMMultiplicator * multiplicator, bcst1);
        str(vbcst, bcst2);

        case vbcst of
           2: if not(bt1to2 in MemRefInfo(opcode).BCSTTypes) then
               Message2(asmr_e_mismatch_broadcasting_elements, '1to' + bcst1, '1to' + bcst2);
           4: if not(bt1to4 in MemRefInfo(opcode).BCSTTypes) then
               Message2(asmr_e_mismatch_broadcasting_elements, '1to' + bcst1, '1to' + bcst2);
           8: if not(bt1to8 in MemRefInfo(opcode).BCSTTypes) then
               Message2(asmr_e_mismatch_broadcasting_elements, '1to' + bcst1, '1to' + bcst2);
          16: if not(bt1to16 in MemRefInfo(opcode).BCSTTypes) then
               Message2(asmr_e_mismatch_broadcasting_elements, '1to' + bcst1, '1to' + bcst2);
        end;
      end
      else if MemRefInfo(opcode).BCSTXMMMultiplicator * multiplicator <> vbcst then
      begin
        str(MemRefInfo(opcode).BCSTXMMMultiplicator * multiplicator, bcst1);
        str(vbcst, bcst2);
        Message2(asmr_e_mismatch_broadcasting_elements, '1to' + bcst1, '1to' + bcst2);
      end;
    end;
  end;


end;


procedure Tx86Instruction.SetInstructionOpsize;
begin
  if opsize<>S_NO then
   exit;
  case ops of
    0 : ;
    1 :
      begin
        { "push es" must be stored as a long PM }
        if ((opcode=A_PUSH) or
            (opcode=A_POP)) and
           (operands[1].opr.typ=OPR_REGISTER) and
           is_segment_reg(operands[1].opr.reg) then
{$ifdef i8086}
          opsize:=S_W
{$else i8086}
          opsize:=S_L
{$endif i8086}
        else
          opsize:=tx86operand(operands[1]).opsize;
      end;
    2 :
      begin
        case opcode of
          A_MOVZX,A_MOVSX :
            begin
              if tx86operand(operands[1]).opsize=S_NO then
                begin
                  tx86operand(operands[1]).opsize:=S_B;
                  if (m_delphi in current_settings.modeswitches) then
                    Message(asmr_w_unable_to_determine_reference_size_using_byte)
                  else
                    Message(asmr_e_unable_to_determine_reference_size);
                end;
              case tx86operand(operands[1]).opsize of
                S_W :
                  case tx86operand(operands[2]).opsize of
                    S_L :
                      opsize:=S_WL;
{$ifdef x86_64}
                    S_Q :
                      opsize:=S_WQ;
{$endif}
                    else
                      ;
                  end;
                S_B :
                  begin
                    case tx86operand(operands[2]).opsize of
                      S_W :
                        opsize:=S_BW;
                      S_L :
                        opsize:=S_BL;
{$ifdef x86_64}
                      S_Q :
                        opsize:=S_BQ;
{$endif}
                      else
                        ;
                    end;
                  end;
                else
                  ;
              end;
            end;
          A_MOVSS,
          A_VMOVSS,
          A_MOVD : { movd is a move from a mmx register to a
                     32 bit register or memory, so no opsize is correct here PM }
            exit;
          A_MOVQ :
            opsize:=S_IQ;
          A_CVTSI2SS,
          A_CVTSI2SD,
          A_OUT :
            opsize:=tx86operand(operands[1]).opsize;
          else
            opsize:=tx86operand(operands[2]).opsize;
        end;
      end;
    3 :
      begin
        case opcode of
          A_VCVTSI2SS,
          A_VCVTSI2SD:
            opsize:=tx86operand(operands[1]).opsize;
        else
          opsize:=tx86operand(operands[ops]).opsize;
        end;
      end;
    4 :
        opsize:=tx86operand(operands[ops]).opsize;

  end;
end;


procedure Tx86Instruction.CheckOperandSizes;
var
  sizeerr : boolean;
  i : longint;
begin
  { Check only the most common opcodes here, the others are done in
    the assembler pass }
  case opcode of
    A_PUSH,A_POP,A_DEC,A_INC,A_NOT,A_NEG,
    A_CMP,A_MOV,
    A_ADD,A_SUB,A_ADC,A_SBB,
    A_AND,A_OR,A_TEST,A_XOR: ;
  else
    exit;
  end;
  { Handle the BW,BL,WL separatly }
  sizeerr:=false;
  { special push/pop selector case }
  if ((opcode=A_PUSH) or
      (opcode=A_POP)) and
     (operands[1].opr.typ=OPR_REGISTER) and
     is_segment_reg(operands[1].opr.reg) then
    exit;
  if opsize in [S_BW,S_BL,S_WL] then
   begin
     if ops<>2 then
      sizeerr:=true
     else
      begin
        case opsize of
          S_BW :
            sizeerr:=(tx86operand(operands[1]).opsize<>S_B) or (tx86operand(operands[2]).opsize<>S_W);
          S_BL :
            sizeerr:=(tx86operand(operands[1]).opsize<>S_B) or (tx86operand(operands[2]).opsize<>S_L);
          S_WL :
            sizeerr:=(tx86operand(operands[1]).opsize<>S_W) or (tx86operand(operands[2]).opsize<>S_L);
{$ifdef x86_64}
          S_BQ:
            sizeerr:=(tx86operand(operands[1]).opsize<>S_B) or (tx86operand(operands[2]).opsize<>S_Q);
          S_WQ:
            sizeerr:=(tx86operand(operands[1]).opsize<>S_W) or (tx86operand(operands[2]).opsize<>S_Q);
          S_LQ:
            sizeerr:=(tx86operand(operands[1]).opsize<>S_L) or (tx86operand(operands[2]).opsize<>S_Q);
{$endif}
          else
            ;
        end;
      end;
   end
  else
   begin
     for i:=1 to ops do
      begin
        if (operands[i].opr.typ<>OPR_CONSTANT) and
           (tx86operand(operands[i]).opsize in [S_B,S_W,S_L]) and
           (tx86operand(operands[i]).opsize<>opsize) then
         sizeerr:=true;
      end;
   end;
  if sizeerr then
   begin
     { if range checks are on then generate an error }
     if (cs_compilesystem in current_settings.moduleswitches) or
        not (cs_check_range in current_settings.localswitches) then
       Message(asmr_w_size_suffix_and_dest_dont_match)
     else
       Message(asmr_e_size_suffix_and_dest_dont_match);
   end;
end;


{ This check must be done with the operand in ATT order
  i.e.after swapping in the intel reader
  but before swapping in the NASM and TASM writers PM }
procedure Tx86Instruction.CheckNonCommutativeOpcodes;
begin
  if (
      (ops=2) and
      (operands[1].opr.typ=OPR_REGISTER) and
      (operands[2].opr.typ=OPR_REGISTER) and
      { if the first is ST and the second is also a register
        it is necessarily ST1 .. ST7 }
      ((operands[1].opr.reg=NR_ST) or
       (operands[1].opr.reg=NR_ST0))
     ) or
     (ops=0) then
      if opcode=A_FSUBR then
        opcode:=A_FSUB
      else if opcode=A_FSUB then
        opcode:=A_FSUBR
      else if opcode=A_FDIVR then
        opcode:=A_FDIV
      else if opcode=A_FDIV then
        opcode:=A_FDIVR
      else if opcode=A_FSUBRP then
        opcode:=A_FSUBP
      else if opcode=A_FSUBP then
        opcode:=A_FSUBRP
      else if opcode=A_FDIVRP then
        opcode:=A_FDIVP
      else if opcode=A_FDIVP then
        opcode:=A_FDIVRP;
  if  (
       (ops=1) and
       (operands[1].opr.typ=OPR_REGISTER) and
       (getregtype(operands[1].opr.reg)=R_FPUREGISTER) and
       (operands[1].opr.reg<>NR_ST) and
       (operands[1].opr.reg<>NR_ST0)
      ) then
      if opcode=A_FSUBRP then
        opcode:=A_FSUBP
      else if opcode=A_FSUBP then
        opcode:=A_FSUBRP
      else if opcode=A_FDIVRP then
        opcode:=A_FDIVP
      else if opcode=A_FDIVP then
        opcode:=A_FDIVRP;
end;

procedure Tx86Instruction.FixupOpcode;
begin
  { does nothing by default }
end;

{*****************************************************************************
                              opcode Adding
*****************************************************************************}

function Tx86Instruction.ConcatInstruction(p : TAsmList) : tai;
var
  siz   : topsize;
  i     : longint;
  asize : int64;
  ai   : taicpu;
begin
  ConcatInstruction:=nil;

  ai:=nil;
  for i:=1 to Ops do
    if not operands[i].CheckOperand then
      exit;

{ Get Opsize }
  if (opsize<>S_NO) or (Ops=0) then
   siz:=opsize
  else
   begin
     if (Ops=2) and (operands[1].opr.typ=OPR_REGISTER) then
      siz:=tx86operand(operands[1]).opsize
     else
      siz:=tx86operand(operands[Ops]).opsize;
     { MOVD should be of size S_LQ or S_QL, but these do not exist PM }
     if (ops=2) and
        (tx86operand(operands[1]).opsize<>S_NO) and
        (tx86operand(operands[2]).opsize<>S_NO) and
        (tx86operand(operands[1]).opsize<>tx86operand(operands[2]).opsize) then
       siz:=S_NO;
   end;

   if ((opcode=A_MOVD)or
       (opcode=A_CVTSI2SS)) and
      ((tx86operand(operands[1]).opsize=S_NO) or
       (tx86operand(operands[2]).opsize=S_NO)) then
     siz:=S_NO;
   { NASM does not support FADD without args
     as alias of FADDP
     and GNU AS interprets FADD without operand differently
     for version 2.9.1 and 2.9.5 !! }
   if (ops=0) and
      ((opcode=A_FADD) or
       (opcode=A_FMUL) or
       (opcode=A_FSUB) or
       (opcode=A_FSUBR) or
       (opcode=A_FDIV) or
       (opcode=A_FDIVR)) then
     begin
       if opcode=A_FADD then
         opcode:=A_FADDP
       else if opcode=A_FMUL then
         opcode:=A_FMULP
       else if opcode=A_FSUB then
         opcode:=A_FSUBP
       else if opcode=A_FSUBR then
         opcode:=A_FSUBRP
       else if opcode=A_FDIV then
         opcode:=A_FDIVP
       else if opcode=A_FDIVR then
         opcode:=A_FDIVRP;
       message1(asmr_w_fadd_to_faddp,std_op2str[opcode]);
     end;

  {It is valid to specify some instructions without operand size.}
  if siz=S_NO then
    begin
      if (ops=1) and (opcode=A_INT) then
        siz:=S_B;
      if (ops=1) and (opcode=A_XABORT) then
        siz:=S_B;
{$ifdef i8086}
      if (ops=1) and (opcode=A_BRKEM) then
        siz:=S_B;
{$endif i8086}
      if (ops=1) and (opcode=A_RET) or (opcode=A_RETN) or (opcode=A_RETF) or
                     (opcode=A_RETW) or (opcode=A_RETNW) or (opcode=A_RETFW) or
{$ifndef x86_64}
                     (opcode=A_RETD) or (opcode=A_RETND) or
{$endif x86_64}
                     (opcode=A_RETFD)
{$ifdef x86_64}
                  or (opcode=A_RETQ) or (opcode=A_RETNQ) or (opcode=A_RETFQ)
{$endif x86_64}
          then
        siz:=S_W;
      if (ops=1) and (opcode=A_PUSH) then
        begin
{$ifdef i8086}
          if (tx86operand(operands[1]).opr.val>=-128) and (tx86operand(operands[1]).opr.val<=127) then
            begin
              siz:=S_B;
              message(asmr_w_unable_to_determine_constant_size_using_byte);
            end
          else
            begin
              siz:=S_W;
              message(asmr_w_unable_to_determine_constant_size_using_word);
            end;
{$else i8086}
          { We are a 32 compiler, assume 32-bit by default. This is Delphi
            compatible but bad coding practise.}

          siz:=S_L;
          message(asmr_w_unable_to_determine_reference_size_using_dword);
{$endif i8086}
        end;
      if (opcode=A_JMP) or (opcode=A_JCC) or (opcode=A_CALL) then
        if ops=1 then
          siz:=S_NEAR
        else
          siz:=S_FAR;
    end;

   { GNU AS interprets FDIV without operand differently
     for version 2.9.1 and 2.10
     we add explicit args to it !! }
  if (ops=0) and
     ((opcode=A_FSUBP) or
      (opcode=A_FSUBRP) or
      (opcode=A_FDIVP) or
      (opcode=A_FDIVRP) or
      (opcode=A_FSUB) or
      (opcode=A_FSUBR) or
      (opcode=A_FADD) or
      (opcode=A_FADDP) or
      (opcode=A_FDIV) or
      (opcode=A_FDIVR)) then
     begin
       message1(asmr_w_adding_explicit_args_fXX,std_op2str[opcode]);
       ops:=2;
       operands[1].opr.typ:=OPR_REGISTER;
       operands[2].opr.typ:=OPR_REGISTER;
       operands[1].opr.reg:=NR_ST0;
       operands[2].opr.reg:=NR_ST1;
     end;
  if (ops=1) and
     (
      (operands[1].opr.typ=OPR_REGISTER) and
      (getregtype(operands[1].opr.reg)=R_FPUREGISTER) and
      (operands[1].opr.reg<>NR_ST) and
      (operands[1].opr.reg<>NR_ST0)
     ) and
     (
      (opcode=A_FSUBP) or
      (opcode=A_FSUBRP) or
      (opcode=A_FDIVP) or
      (opcode=A_FDIVRP) or
      (opcode=A_FADDP) or
      (opcode=A_FMULP)
     ) then
     begin
       message1(asmr_w_adding_explicit_first_arg_fXX,std_op2str[opcode]);
       ops:=2;
       operands[2].opr.typ:=OPR_REGISTER;
       operands[2].opr.reg:=operands[1].opr.reg;
       operands[1].opr.reg:=NR_ST0;
     end;

  if (ops=1) and
     (
      (operands[1].opr.typ=OPR_REGISTER) and
      (getregtype(operands[1].opr.reg)=R_FPUREGISTER) and
      (operands[1].opr.reg<>NR_ST) and
      (operands[1].opr.reg<>NR_ST0)
     ) and
     (
      (opcode=A_FSUB) or
      (opcode=A_FSUBR) or
      (opcode=A_FDIV) or
      (opcode=A_FDIVR) or
      (opcode=A_FADD) or
      (opcode=A_FMUL)
     ) then
     begin
       message1(asmr_w_adding_explicit_second_arg_fXX,std_op2str[opcode]);
       ops:=2;
       operands[2].opr.typ:=OPR_REGISTER;
       operands[2].opr.reg:=NR_ST0;
     end;

   { Check for 'POP CS' }
   if (opcode=A_POP) and (ops=1) and (operands[1].opr.typ=OPR_REGISTER) and
      (operands[1].opr.reg=NR_CS) then
{$ifdef i8086}
     { On i8086 we print only a warning, because 'POP CS' works on 8086 and 8088
       CPUs, but isn't supported on any later CPU }
     Message(asmr_w_pop_cs_not_portable);
{$else i8086}
     { On the i386 and x86_64 targets, we print out an error, because no CPU,
       supported by these targets support 'POP CS' }
     Message(asmr_e_pop_cs_not_valid);
{$endif i8086}

   { I tried to convince Linus Torvalds to add
     code to support ENTER instruction
     (when raising a stack page fault)
     but he replied that ENTER is a bad instruction and
     Linux does not need to support it
     So I think its at least a good idea to add a warning
     if someone uses this in assembler code
     FPC itself does not use it at all PM }
   if (opcode=A_ENTER) and
      (target_info.system in [system_i386_linux,system_i386_FreeBSD,system_i386_android]) then
     Message(asmr_w_enter_not_supported_by_linux);

  ai:=taicpu.op_none(opcode,siz);
  ai.fileinfo:=filepos;
  ai.SetOperandOrder(op_att);
  ai.Ops:=Ops;
  ai.Allocate_oper(Ops);
  for i:=1 to Ops do
  begin
    ai.oper[i-1]^.vopext := (operands[i] as tx86operand).vopext;

    case operands[i].opr.typ of
       OPR_CONSTANT :
         ai.loadconst(i-1,operands[i].opr.val);
       OPR_REGISTER:
         ai.loadreg(i-1,operands[i].opr.reg);
       OPR_SYMBOL:
{$ifdef i8086}
        if operands[i].opr.symseg then
          taicpu(ai).loadsegsymbol(i-1,operands[i].opr.symbol)
        else
{$endif i8086}
          ai.loadsymbol(i-1,operands[i].opr.symbol,operands[i].opr.symofs);
       OPR_LOCAL :
         with operands[i].opr do
           begin
             ai.loadlocal(i-1,localsym,localsymofs,localindexreg,
                          localscale,localgetoffset,localforceref);
             ai.oper[i-1]^.localoper^.localsegment:=localsegment;
           end;
       OPR_REFERENCE:
         begin

           if current_settings.optimizerswitches <> [] then
            if (not(MemRefInfo(opcode).MemRefSize in MemRefSizeInfoVMems)) and (opcode<>A_XLAT) and not is_x86_string_op(opcode) then
             optimize_ref(operands[i].opr.ref,true);

           ai.loadref(i-1,operands[i].opr.ref);
           if operands[i].size<>OS_NO then
             begin
               asize:=0;
               case operands[i].size of
                   OS_8,OS_S8 :
                     asize:=OT_BITS8;
                   OS_16,OS_S16, OS_M16:
                     asize:=OT_BITS16;
                   OS_32,OS_S32 :
{$ifdef i8086}
                     if siz=S_FAR then
                       asize:=OT_FAR
                     else
                       asize:=OT_BITS32;
{$else i8086}
                     asize:=OT_BITS32;
{$endif i8086}
                   OS_F32,OS_M32 :
                     asize:=OT_BITS32;
                   OS_64,OS_S64:
                     begin
                       { Only FPU and SSE/AVX operations know about 64bit
                         values, for all integer operations it is seen as 32bit

                          this applies only to i386, see tw16622}

                       if (gas_needsuffix[opcode] in [attsufFPU,attsufFPUint]) or (MemRefInfo(opcode).ExistsSSEAVX) then
                        asize:=OT_BITS64
{$ifdef i386}
                       else
                         asize:=OT_BITS32
{$endif i386}
                         ;
                     end;
                   OS_F64,OS_C64, OS_M64 :
                     asize:=OT_BITS64;
                   OS_F80 :
                     asize:=OT_BITS80;
                   OS_128,OS_M128,OS_MS128:
                     asize := OT_BITS128;
                   OS_M256,OS_MS256:
                     asize := OT_BITS256;
                   OS_M512,OS_MS512:
                     asize := OT_BITS512;
                   else
                     ;
                 end;
               if asize<>0 then
                 ai.oper[i-1]^.ot:=(ai.oper[i-1]^.ot and OT_NON_SIZE) or asize;
             end;
         end;
       else
         ;
    end;


  end;

 { Condition ? }
  if condition<>C_None then
   ai.SetCondition(condition);

  { Set is_jmp, it enables asmwriter to emit short jumps if appropriate }
  if (opcode=A_JMP) or (opcode=A_JCC) then
    ai.is_jmp := True;

 { Concat the opcode or give an error }
  if assigned(ai) then
    p.concat(ai)
  else
   Message(asmr_e_invalid_opcode_and_operand);
  result:=ai;
end;

function Tx86Instruction.getstring: string;
var
  i : longint;
  s, sval : string;
  regnr: string;
  addsize : boolean;
begin
  s:='['+std_op2str[opcode];
  for i:=1 to ops do
   begin
     with operands[i] as Tx86Operand do
       begin
         if i=0 then
          s:=s+' '
         else
          s:=s+',';
         { type }
         addsize:=false;

         case operands[i].opr.typ of
            OPR_CONSTANT : begin
                             str(operands[i].opr.val, sval);
                             s:=s+ sval;
                           end;
            OPR_REGISTER : begin
                             regnr := '';
                             str(getsupreg(opr.reg),regnr);

                             if getsubreg(opr.reg)= R_SUBMMX then
                              s:=s+'xmmreg' + regnr
                             else
                               if getsubreg(opr.reg)= R_SUBMMY then
                                s:=s+'ymmreg' + regnr
                             else
                               if getsubreg(opr.reg)= R_SUBMMZ then
                                s:=s+'zmmreg' + regnr
                             else
                               if getregtype(opr.reg)= R_MMXREGISTER then
                                s:=s+'mmxreg'
                             else
                               if getregtype(opr.reg)= R_FPUREGISTER then
                                s:=s+'fpureg'
                             else
                               if getregtype(opr.reg)=R_INTREGISTER then
                               begin
                                 s:=s+'reg';
                                 addsize:=true;
                               end;
                           end;
               OPR_LOCAL,
            OPR_REFERENCE: begin
                             s:=s + 'mem';
                             addsize:=true;
                           end;
                      else s:=s + '???';
         end;

         if addsize then
          begin
            sval := '';
            str(tcgsize2size[size], sval);

            s := s + sval;
          end;

         if vopext <> 0 then
          begin
            str(vopext and $07, regnr);
            if vopext and OTVE_VECTOR_WRITEMASK = OTVE_VECTOR_WRITEMASK then
              s := s + ' {k' + regnr + '}';

            if vopext and OTVE_VECTOR_ZERO = OTVE_VECTOR_ZERO then
              s := s + ' {z}';

            if vopext and OTVE_VECTOR_SAE = OTVE_VECTOR_SAE then
              s := s + ' {sae}';


            if vopext and OTVE_VECTOR_BCST = OTVE_VECTOR_BCST then
             case vopext and OTVE_VECTOR_BCST_MASK of
                OTVE_VECTOR_BCST2: s := s + ' {1to2}';
                OTVE_VECTOR_BCST4: s := s + ' {1to4}';
                OTVE_VECTOR_BCST8: s := s + ' {1to8}';
               OTVE_VECTOR_BCST16: s := s + ' {1to16}';
             end;

            if vopext and OTVE_VECTOR_ER = OTVE_VECTOR_ER then
             case vopext and OTVE_VECTOR_ER_MASK of
                OTVE_VECTOR_RNSAE: s := s + ' {rn-sae}';
                OTVE_VECTOR_RDSAE: s := s + ' {rd-sae}';
                OTVE_VECTOR_RUSAE: s := s + ' {ru-sae}';
                OTVE_VECTOR_RZSAE: s := s + ' {rz-sae}';
             end;

          end;
       end;
   end;
  GetString:=s+']';
end;

function Tx86Instruction.MightHaveExtension: boolean;

begin
  Result:=aasmcpu.MightHaveExtension(opcode);
end;

end.
