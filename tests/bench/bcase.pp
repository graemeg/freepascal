{$goto on}
program bcase;

{$mode objfpc}{$H+}

uses
  SysUtils;

{ Utility functions }
function GetRealTime(const st: TSystemTime): Real;
  begin
    Result := st.Hour*3600.0 + st.Minute*60.0 + st.Second + st.MilliSecond/1000.0;
  end;

{$push}
{$warn 5057 off}
function GetRealTime : Real;
  var
    st:TSystemTime;
  begin
    GetLocalTime(st);
    result:=GetRealTime(st);
  end;
{$pop}

function IIf(Condition: Boolean; TrueRes, FalseRes: Integer): Integer; inline;
  begin
    if Condition then
      Result := TrueRes
    else
      Result := FalseRes;
  end;

const
  ITERATIONS = 33554432;

  AES_S_Box: array[Byte] of Byte = (
    $63, $7c, $77, $7b, $f2, $6b, $6f, $c5, $30, $01, $67, $2b, $fe, $d7, $ab, $76,
    $ca, $82, $c9, $7d, $fa, $59, $47, $f0, $ad, $d4, $a2, $af, $9c, $a4, $72, $c0,
    $b7, $fd, $93, $26, $36, $3f, $f7, $cc, $34, $a5, $e5, $f1, $71, $d8, $31, $15,
    $04, $c7, $23, $c3, $18, $96, $05, $9a, $07, $12, $80, $e2, $eb, $27, $b2, $75,
    $09, $83, $2c, $1a, $1b, $6e, $5a, $a0, $52, $3b, $d6, $b3, $29, $e3, $2f, $84,
    $53, $d1, $00, $ed, $20, $fc, $b1, $5b, $6a, $cb, $be, $39, $4a, $4c, $58, $cf,
    $d0, $ef, $aa, $fb, $43, $4d, $33, $85, $45, $f9, $02, $7f, $50, $3c, $9f, $a8,
    $51, $a3, $40, $8f, $92, $9d, $38, $f5, $bc, $b6, $da, $21, $10, $ff, $f3, $d2,
    $cd, $0c, $13, $ec, $5f, $97, $44, $17, $c4, $a7, $7e, $3d, $64, $5d, $19, $73,
    $60, $81, $4f, $dc, $22, $2a, $90, $88, $46, $ee, $b8, $14, $de, $5e, $0b, $db,
    $e0, $32, $3a, $0a, $49, $06, $24, $5c, $c2, $d3, $ac, $62, $91, $95, $e4, $79,
    $e7, $c8, $37, $6d, $8d, $d5, $4e, $a9, $6c, $56, $f4, $ea, $65, $7a, $ae, $08,
    $ba, $78, $25, $2e, $1c, $a6, $b4, $c6, $e8, $dd, $74, $1f, $4b, $bd, $8b, $8a,
    $70, $3e, $b5, $66, $48, $03, $f6, $0e, $61, $35, $57, $b9, $86, $c1, $1d, $9e,
    $e1, $f8, $98, $11, $69, $d9, $8e, $94, $9b, $1e, $87, $e9, $ce, $55, $28, $df,
    $8c, $a1, $89, $0d, $bf, $e6, $42, $68, $41, $99, $2d, $0f, $b0, $54, $bb, $16
  );

  FirstWeighted: array[0..255] of Byte = (
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63, $63,
    $ba, $78, $25, $2e, $1c, $a6, $b4, $c6, $e8, $dd, $74, $1f, $4b, $bd, $8b, $8a,
    $70, $3e, $b5, $66, $48, $03, $f6, $0e, $61, $35, $57, $b9, $86, $c1, $1d, $9e,
    $e1, $f8, $98, $11, $69, $d9, $8e, $94, $9b, $1e, $87, $e9, $ce, $55, $28, $df,
    $8c, $a1, $89, $0d, $bf, $e6, $42, $68, $41, $99, $2d, $0f, $b0, $54, $bb, $16
  );

  LastWeighted: array[0..255] of Byte = (
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16, $16,
    $ba, $78, $25, $2e, $1c, $a6, $b4, $c6, $e8, $dd, $74, $1f, $4b, $bd, $8b, $8a,
    $70, $3e, $b5, $66, $48, $03, $f6, $0e, $61, $35, $57, $b9, $86, $c1, $1d, $9e,
    $e1, $f8, $98, $11, $69, $d9, $8e, $94, $9b, $1e, $87, $e9, $ce, $55, $28, $df,
    $8c, $a1, $89, $0d, $bf, $e6, $42, $68, $41, $99, $2d, $0f, $b0, $54, $bb, $16
  );

  AlmostFullExpected: array[0..255] of Byte = (
    $63, $7c, $77, $7b, $f2, $6b, $6f, $c5, $30, $01, $67, $2b, $fe, $d7, $ab, $76,
    $ca, $82, $c9, $7d, $fa, $59, $47, $f0, $ad, $d4, $a2, $af, $9c, $a4, $72, $c0,
    $b7, $fd, $93, $26, $36, $3f, $f7, $cc, $34, $a5, $e5, $f1, $71, $d8, $31, $15,
    $04, $c7, $23, $c3, $18, $96, $05, $9a, $07, $12, $80, $e2, $eb, $27, $b2, $75,
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00,
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00,
    $d0, $ef, $aa, $fb, $43, $4d, $33, $85, $45, $f9, $02, $7f, $50, $3c, $9f, $a8,
    $51, $a3, $40, $8f, $92, $9d, $38, $f5, $bc, $b6, $da, $21, $10, $ff, $f3, $d2,
    $cd, $0c, $13, $ec, $5f, $97, $44, $17, $c4, $a7, $7e, $3d, $64, $5d, $19, $73,
    $60, $81, $4f, $dc, $22, $2a, $90, $88, $46, $ee, $b8, $14, $de, $5e, $0b, $db,
    $e0, $32, $3a, $0a, $49, $06, $24, $5c, $c2, $d3, $ac, $62, $91, $95, $e4, $79,
    $e7, $c8, $37, $6d, $8d, $d5, $4e, $a9, $6c, $56, $f4, $ea, $65, $7a, $ae, $08,
    $ba, $78, $25, $2e, $1c, $a6, $b4, $c6, $e8, $dd, $74, $1f, $4b, $bd, $8b, $8a,
    $70, $3e, $b5, $66, $48, $03, $f6, $0e, $61, $35, $57, $b9, $86, $c1, $1d, $9e,
    $e1, $f8, $98, $11, $69, $d9, $8e, $94, $9b, $1e, $87, $e9, $ce, $55, $28, $df,
    $8c, $a1, $89, $0d, $bf, $e6, $42, $68, $41, $99, $2d, $0f, $b0, $54, $bb, $16
  );

type
  TInstructionSet = ( { Truncated to 1024 entries }
    A_NONE = -512, A_ADC, A_ADD, A_AND, A_BSF, A_BSR, A_BSWAP, A_BT, A_BTC, A_BTR, A_BTS,
    A_CALL, A_CBW, A_CDQ, A_CLC, A_CLD, A_CLI, A_CLTS, A_CMC, A_CMP, A_CMPSB,
    A_CMPSD, A_CMPSW, A_CMPXCHG, A_CMPXCHG486, A_CMPXCHG8B, A_CPUID, A_CWD, A_CWDE,
    A_DEC, A_DIV, A_EMMS, A_ENTER, A_F2XM1, A_FABS, A_FADD, A_FADDP, A_FBLD, A_FBSTP,
    A_FCHS, A_FCLEX,A_FCMOVB, A_FCMOVBE, A_FCMOVE, A_FCMOVNB, A_FCMOVNBE, A_FCMOVNE,
    A_FCMOVNU, A_FCMOVU, A_FCOM, A_FCOMI, A_FCOMIP, A_FCOMP, A_FCOMPP, A_FCOS,
    A_FDECSTP, A_FDISI, A_FDIV, A_FDIVP, A_FDIVR, A_FDIVRP, A_FEMMS, A_FENI, A_FFREE,
    A_FIADD, A_FICOM, A_FICOMP, A_FIDIV, A_FIDIVR, A_FILD, A_FIMUL, A_FINCSTP,
    A_FINIT, A_FIST, A_FISTP, A_FISTTP, A_FISUB, A_FISUBR, A_FLD, A_FLD1, A_FLDCW,
    A_FLDENV, A_FLDL2E, A_FLDL2T, A_FLDLG2, A_FLDLN2, A_FLDPI, A_FLDZ, A_FMUL,
    A_FMULP, A_FNCLEX, A_FNDISI, A_FNENI, A_FNINIT, A_FNOP, A_FNSAVE, A_FNSTCW,
    A_FNSTENV, A_FNSTSW, A_FPATAN, A_FPREM, A_FPREM1, A_FPTAN, A_FRNDINT, A_FRSTOR,
    A_FSAVE, A_FSCALE, A_FSETPM, A_FSIN, A_FSINCOS, A_FSQRT, A_FST, A_FSTCW,
    A_FSTENV, A_FSTP, A_FSTSW, A_FSUB, A_FSUBP, A_FSUBR, A_FSUBRP, A_FTST, A_FUCOM,
    A_FUCOMI, A_FUCOMIP, A_FUCOMP, A_FUCOMPP, A_FWAIT, A_FXAM, A_FXCH, A_FXTRACT,
    A_FYL2X, A_FYL2XP1, A_HLT, A_IBTS, A_ICEBP, A_IDIV, A_IMUL, A_IN, A_INC, A_INSB,
    A_INSD, A_INSW, A_INT, A_INT01, A_INT1, A_INT03, A_INT3, A_INVD, A_INVLPG,
    A_IRET, A_IRETD, A_IRETW, A_IRETQ, A_JECXZ, A_JRCXZ, A_JMP, A_LAHF, A_LAR,
    A_LCALL, A_LEA, A_LEAVE, A_LFS, A_LGDT, A_LGS, A_LIDT, A_LJMP, A_LLDT, A_LMSW,
    A_LOADALL, A_LOADALL286, A_LOCK, A_LODSB, A_LODSD, A_LODSW, A_LOOP, A_LOOPE,
    A_LOOPNE, A_LOOPNZ, A_LOOPZ, A_LSL, A_LSS, A_LTR, A_MONITOR, A_MOV, A_MOVD,
    A_MOVQ, A_MOVSB, A_MOVSD, A_MOVSQ, A_MOVSW, A_MOVSX, A_MOVZX, A_MUL, A_MWAIT,
    A_NEG, A_NOP, A_NOT, A_OR, A_OUT, A_OUTSB, A_OUTSD, A_OUTSW, A_PACKSSDW,
    A_PACKSSWB, A_PACKUSWB, A_PADDB, A_PADDD, A_PADDSB, A_PADDSIW, A_PADDSW,
    A_PADDUSB, A_PADDUSW, A_PADDW, A_PAND, A_PANDN, A_PAVEB, A_PAVGUSB, A_PCMPEQB,
    A_PCMPEQD, A_PCMPEQW, A_PCMPGTB, A_PCMPGTD, A_PCMPGTW, A_PDISTIB, A_PF2ID,
    A_PFACC, A_PFADD, A_PFCMPEQ, A_PFCMPGE, A_PFCMPGT, A_PFMAX, A_PFMIN, A_PFMUL,
    A_PFRCP, A_PFRCPIT1, A_PFRCPIT2, A_PFRSQIT1, A_PFRSQRT, A_PFSUB, A_PFSUBR,
    A_PI2FD, A_PMACHRIW, A_PMADDWD, A_PMAGW, A_PMULHRIW, A_PMULHRWA, A_PMULHRWC,
    A_PMULHW, A_PMULLW, A_PMVGEZB, A_PMVLZB, A_PMVNZB, A_PMVZB, A_POP, A_POPF,
    A_POPFW, A_POPFQ, A_POR, A_PREFETCH, A_PREFETCHW, A_PSLLD, A_PSLLDQ, A_PSLLQ,
    A_PSLLW, A_PSRAD, A_PSRAW, A_PSRLD, A_PSRLQ, A_PSRLW, A_PSUBB, A_PSUBD, A_PSUBSB,
    A_PSUBSIW, A_PSUBSW, A_PSUBUSB, A_PSUBUSW, A_PSUBW, A_PUNPCKHBW, A_PUNPCKHDQ,
    A_PUNPCKHWD, A_PUNPCKLBW, A_PUNPCKLDQ, A_PUNPCKLWD, A_PUSH, A_PUSHF, A_PUSHFW,
    A_PUSHFQ, A_PXOR, A_RCL, A_RCR, A_RDSHR, A_RDMSR, A_RDPMC, A_RDTSC, A_REP,
    A_REPE, A_REPNE, A_REPNZ, A_REPZ, A_RET, A_RETF, A_RETN, A_RETW, A_RETFW,
    A_RETNW, A_RETFD, A_RETQ, A_RETFQ, A_RETNQ, A_ROL, A_ROR, A_RSDC, A_RSLDT, A_RSM,
    A_SAHF, A_SAL, A_SAR, A_SBB, A_SCASB, A_SCASD, A_SCASQ, A_SCASW, A_SEGCS,
    A_SEGDS, A_SEGES, A_SEGFS, A_SEGGS, A_SEGSS, A_SGDT, A_SHL, A_SHLD, A_SHR,
    A_SHRD, A_SIDT, A_SLDT, A_SMI, A_SMINT, A_SMINTOLD, A_SMSW, A_STC, A_STD, A_STI,
    A_STOSB, A_STOSD, A_STOSW, A_STR, A_SUB, A_SVDC, A_SVLDT, A_SVTS, A_SYSCALL,
    A_SYSENTER, A_SYSEXIT, A_SYSRET, A_TEST, A_UD1, A_UD2, A_UMOV, A_VERR, A_VERW,
    A_WAIT, A_WBINVD, A_WRSHR, A_WRMSR, A_XADD, A_XBTS, A_XCHG, A_XLAT, A_XLATB,
    A_XOR, A_XSTORE, A_XCRYPTECB, A_XCRYPTCBC, A_XCRYPTCFB, A_XCRYPTOFB, A_CMOVcc,
    A_Jcc, A_SETcc, A_MOVS, A_CMPS, A_SCAS, A_LODS, A_STOS, A_INS, A_OUTS, A_ADDPS,
    A_ADDSS, A_ANDNPS, A_ANDPS, A_CMPEQPS, A_CMPEQSS, A_CMPLEPS, A_CMPLESS,
    A_CMPLTPS, A_CMPLTSS, A_CMPNEQPS, A_CMPNEQSS, A_CMPNLEPS, A_CMPNLESS,
    A_CMPNLTPS, A_CMPNLTSS, A_CMPORDPS, A_CMPORDSS, A_CMPUNORDPS, A_CMPUNORDSS,
    A_CMPPS, A_CMPSS, A_COMISS, A_CVTPI2PS, A_CVTPS2PI, A_CVTSI2SS, A_CVTSS2SI,
    A_CVTTPS2PI, A_CVTTSS2SI, A_DIVPS, A_DIVSS, A_LDMXCSR, A_MAXPS, A_MAXSS, A_MINPS,
    A_MINSS, A_MOVAPS, A_MOVHPS, A_MOVLHPS, A_MOVLPS, A_MOVHLPS, A_MOVMSKPS,
    A_MOVNTPS, A_MOVSS, A_MOVUPS, A_MULPS, A_MULSS, A_ORPS, A_RCPPS, A_RCPSS,
    A_RSQRTPS, A_RSQRTSS, A_SHUFPS, A_SQRTPS, A_SQRTSS, A_STMXCSR, A_SUBPS, A_SUBSS,
    A_UCOMISS, A_UNPCKHPS, A_UNPCKLPS, A_XORPS, A_FXRSTOR, A_FXSAVE, A_PREFETCHNTA,
    A_PREFETCHT0, A_PREFETCHT1, A_PREFETCHT2, A_SFENCE, A_MASKMOVQ, A_MOVNTQ,
    A_PAVGB, A_PAVGW, A_PEXTRW, A_PINSRW, A_PMAXSW, A_PMAXUB, A_PMINSW, A_PMINUB,
    A_PMOVMSKB, A_PMULHUW, A_PSADBW, A_PSHUFW, A_PFNACC, A_PFPNACC, A_PI2FW, A_PF2IW,
    A_PSWAPD, A_FFREEP, A_MASKMOVDQU, A_CLFLUSH, A_MOVNTDQ, A_MOVNTI, A_MOVNTPD,
    A_PAUSE, A_LFENCE, A_MFENCE, A_MOVDQA, A_MOVDQU, A_MOVDQ2Q, A_MOVQ2DQ, A_PADDQ,
    A_PMULUDQ, A_PSHUFD, A_PSHUFHW, A_PSHUFLW, A_PSRLDQ, A_PSUBQ, A_PUNPCKHQDQ,
    A_PUNPCKLQDQ, A_ADDPD, A_ADDSD, A_ANDNPD, A_ANDPD, A_CMPEQPD, A_CMPEQSD,
    A_CMPLEPD, A_CMPLESD, A_CMPLTPD, A_CMPLTSD, A_CMPNEQPD, A_CMPNEQSD, A_CMPNLEPD,
    A_CMPNLESD, A_CMPNLTPD, A_CMPNLTSD, A_CMPORDPD, A_CMPORDSD, A_CMPUNORDPD,
    A_CMPUNORDSD, A_CMPPD, A_COMISD, A_CVTDQ2PD, A_CVTDQ2PS, A_CVTPD2DQ, A_CVTPD2PI,
    A_CVTPD2PS, A_CVTPI2PD, A_CVTPS2DQ, A_CVTPS2PD, A_CVTSD2SI, A_CVTSD2SS,
    A_CVTSI2SD, A_CVTSS2SD, A_CVTTPD2PI, A_CVTTPD2DQ, A_CVTTPS2DQ, A_CVTTSD2SI,
    A_DIVPD, A_DIVSD, A_MAXPD, A_MAXSD, A_MINPD, A_MINSD, A_MOVAPD, A_MOVHPD,
    A_MOVLPD, A_MOVMSKPD, A_MOVUPD, A_MULPD, A_MULSD, A_ORPD, A_SHUFPD, A_SQRTPD,
    A_SQRTSD, A_SUBPD, A_SUBSD, A_UCOMISD, A_UNPCKHPD, A_UNPCKLPD, A_XORPD,
    A_ADDSUBPD, A_ADDSUBPS, A_HADDPD, A_HADDPS, A_HSUBPD, A_HSUBPS, A_LDDQU,
    A_MOVDDUP, A_MOVSHDUP, A_MOVSLDUP, A_VMREAD, A_VMWRITE, A_VMCALL, A_VMLAUNCH,
    A_VMRESUME, A_VMXOFF, A_VMXON, A_VMCLEAR, A_VMPTRLD, A_VMPTRST, A_VMRUN,
    A_VMMCALL, A_VMLOAD, A_VMSAVE, A_STGI, A_CLGI, A_SKINIT, A_INVLPGA, A_MONTMUL,
    A_XSHA1, A_XSHA256, A_DMINT, A_RDM, A_MOVABS, A_MOVSXD, A_CQO, A_CDQE,
    A_CMPXCHG16B, A_MOVNTSS, A_MOVNTSD, A_INSERTQ, A_EXTRQ, A_LZCNT, A_PABSB,
    A_PABSW, A_PABSD, A_PALIGNR, A_PHADDW, A_PHADDD, A_PHADDSW, A_PHSUBW, A_PHSUBD,
    A_PHSUBSW, A_PMADDUBSW, A_PMULHRSW, A_PSHUFB, A_PSIGNB, A_PSIGNW, A_PSIGND,
    A_BLENDPS, A_BLENDPD, A_BLENDVPS, A_BLENDVPD, A_DPPS, A_DPPD, A_EXTRACTPS,
    A_INSERTPS, A_MOVNTDQA, A_MPSADBW, A_PACKUSDW, A_PBLENDVB, A_PBLENDW, A_PCMPEQQ,
    A_PEXTRB, A_PEXTRD, A_PEXTRQ, A_PHMINPOSUW, A_PINSRB, A_PINSRD, A_PINSRQ, A_PMAXSB,
    A_PMAXSD, A_PMAXUD, A_PMAXUW, A_PMINSB, A_PMINSD, A_PMINUW, A_PMINUD, A_PMOVSXBW,
    A_PMOVSXBD, A_PMOVSXBQ, A_PMOVSXWD, A_PMOVSXWQ, A_PMOVSXDQ, A_PMOVZXBW, A_PMOVZXBD,
    A_PMOVZXBQ, A_PMOVZXWD, A_PMOVZXWQ, A_PMOVZXDQ, A_PMULDQ, A_PMULLD, A_PTEST,
    A_ROUNDPS, A_ROUNDPD, A_ROUNDSS, A_ROUNDSD, A_CRC32, A_PCMPESTRI, A_PCMPESTRM,
    A_PCMPISTRI, A_PCMPISTRM, A_PCMPGTQ, A_POPCNT, A_AESENC, A_AESENCLAST, A_AESDEC,
    A_AESDECLAST, A_AESIMC, A_AESKEYGENASSIST, A_RDTSCP, A_STOSQ, A_LODSQ, A_CMPSQ,
    A_VADDPD, A_VADDPS, A_VADDSD, A_VADDSS, A_VADDSUBPD, A_VADDSUBPS, A_VAESDEC,
    A_VAESDECLAST, A_VAESENC, A_VAESENCLAST, A_VAESIMC, A_VAESKEYGENASSIST, A_VANDNPD,
    A_VANDNPS, A_VANDPD, A_VANDPS, A_VBLENDPD, A_VBLENDPS, A_VBLENDVPD, A_VBLENDVPS,
    A_VBROADCASTF128, A_VBROADCASTSD, A_VBROADCASTSS, A_VCMPEQPS, A_VCMPLTPS,
    A_VCMPLEPS, A_VCMPUNORDPS, A_VCMPNEQPS, A_VCMPNLTPS, A_VCMPNLEPS, A_VCMPORDPS,
    A_VCMPEQ_UQPS, A_VCMPNGEPS, A_VCMPNGTPS, A_VCMPFALSEPS, A_VCMPNEQ_OQPS,
    A_VCMPGEPS, A_VCMPGTPS, A_VCMPTRUEPS, A_VCMPEQ_OSPS, A_VCMPLT_OQPS, A_VCMPLE_OQPS,
    A_VCMPUNORD_SPS, A_VCMPNEQ_USPS, A_VCMPNLT_UQPS, A_VCMPNLE_UQPS, A_VCMPORD_SPS,
    A_VCMPEQ_USPS, A_VCMPNGE_UQPS, A_VCMPNGT_UQPS, A_VCMPFALSE_OSPS, A_VCMPNEQ_OSPS,
    A_VCMPGE_OQPS, A_VCMPGT_OQPS, A_VCMPTRUE_USPS, A_VCMPEQPD, A_VCMPLTPD, A_VCMPLEPD,
    A_VCMPUNORDPD, A_VCMPNEQPD, A_VCMPNLTPD, A_VCMPNLEPD, A_VCMPORDPD, A_VCMPEQ_UQPD,
    A_VCMPNGEPD, A_VCMPNGTPD, A_VCMPFALSEPD, A_VCMPNEQ_OQPD, A_VCMPGEPD, A_VCMPGTPD,
    A_VCMPTRUEPD, A_VCMPEQ_OSPD, A_VCMPLT_OQPD, A_VCMPLE_OQPD, A_VCMPUNORD_SPD,
    A_VCMPNEQ_USPD, A_VCMPNLT_UQPD, A_VCMPNLE_UQPD, A_VCMPORD_SPD, A_VCMPEQ_USPD,
    A_VCMPNGE_UQPD, A_VCMPNGT_UQPD, A_VCMPFALSE_OSPD, A_VCMPNEQ_OSPD, A_VCMPGE_OQPD,
    A_VCMPGT_OQPD, A_VCMPTRUE_USPD, A_VCMPPD, A_VCMPPS, A_VCMPSD, A_VCMPSS, A_VCOMISD,
    A_VCOMISS, A_VCVTDQ2PD, A_VCVTDQ2PS, A_VCVTPD2DQ, A_VCVTPD2PS, A_VCVTPS2DQ,
    A_VCVTPS2PD, A_VCVTSD2SI, A_VCVTSD2SS, A_VCVTSI2SD, A_VCVTSI2SS, A_VCVTSS2SD,
    A_VCVTSS2SI, A_VCVTTPD2DQ, A_VCVTTPS2DQ, A_VCVTTSD2SI, A_VCVTTSS2SI, A_VDIVPD,
    A_VDIVPS, A_VDIVSD, A_VDIVSS, A_VDPPD, A_VDPPS, A_VEXTRACTF128, A_VEXTRACTPS,
    A_VHADDPD, A_VHADDPS, A_VHSUBPD, A_VHSUBPS, A_VINSERTF128, A_VINSERTPS, A_VLDDQU,
    A_VLDMXCSR, A_VMASKMOVDQU, A_VMASKMOVPD, A_VMASKMOVPS, A_VMAXPD, A_VMAXPS,
    A_VMAXSD, A_VMAXSS, A_VMINPD, A_VMINPS, A_VMINSD, A_VMINSS, A_VMOVAPD, A_VMOVAPS,
    A_VMOVD, A_VMOVDDUP, A_VMOVDQA, A_VMOVDQU, A_VMOVHLPS, A_VMOVHPD, A_VMOVHPS,
    A_VMOVLHPS, A_VMOVLPD, A_VMOVLPS, A_VMOVMSKPD, A_VMOVMSKPS, A_VMOVNTDQ,
    A_VMOVNTDQA, A_VMOVNTPD, A_VMOVNTPS, A_VMOVQ, A_VMOVSD, A_VMOVSHDUP, A_VMOVSLDUP,
    A_VMOVSS, A_VMOVUPD, A_VMOVUPS, A_VMPSADBW, A_VMULPD, A_VMULPS, A_VMULSD,
    A_VMULSS, A_VORPD, A_VORPS, A_VPABSB, A_VPABSD, A_VPABSW, A_VPACKSSDW,
    A_VPACKSSWB, A_VPACKUSDW, A_VPACKUSWB, A_VPADDB, A_VPADDD, A_VPADDQ, A_VPADDSB,
    A_VPADDSW, A_VPADDUSB, A_VPADDUSW, A_VPADDW, A_VPALIGNR, A_VPAND, A_VPANDN,
    A_VPAVGB, A_VPAVGW, A_VPBLENDVB, A_VPBLENDW, A_VPCLMULQDQ, A_VPCMPEQB, A_VPCMPEQD,
    A_VPCMPEQQ, A_VPCMPEQW, A_VPCMPESTRI, A_VPCMPESTRM, A_VPCMPGTB, A_VPCMPGTD,
    A_VPCMPGTQ, A_VPCMPGTW, A_VPCMPISTRI, A_VPCMPISTRM, A_VPERM2F128, A_VPERMILPD,
    A_VPERMILPS, A_VPEXTRB, A_VPEXTRD, A_VPEXTRQ, A_VPEXTRW, A_VPHADDD, A_VPHADDSW,
    A_VPHADDW, A_VPHMINPOSUW, A_VPHSUBD, A_VPHSUBSW, A_VPHSUBW, A_VPINSRB, A_VPINSRD,
    A_VPINSRQ, A_VPINSRW, A_VPMADDUBSW, A_VPMADDWD, A_VPMAXSB, A_VPMAXSD, A_VPMAXSW,
    A_VPMAXUB, A_VPMAXUD, A_VPMAXUW, A_VPMINSB, A_VPMINSD, A_VPMINSW, A_VPMINUB,
    A_VPMINUD, A_VPMINUW, A_VPMOVMSKB, A_VPMOVSXBD, A_VPMOVSXBQ, A_VPMOVSXBW,
    A_VPMOVSXDQ, A_VPMOVSXWD, A_VPMOVSXWQ, A_VPMOVZXBD, A_VPMOVZXBQ, A_VPMOVZXBW,
    A_VPMOVZXDQ, A_VPMOVZXWD, A_VPMOVZXWQ, A_VPMULDQ, A_VPMULHRSW, A_VPMULHUW,
    A_VPMULHW, A_VPMULLD, A_VPMULLW, A_VPMULUDQ, A_VPOR, A_VPSADBW, A_VPSHUFB,
    A_VPSHUFD, A_VPSHUFHW, A_VPSHUFLW, A_VPSIGNB, A_VPSIGND, A_VPSIGNW, A_VPSLLD,
    A_VPSLLDQ, A_VPSLLQ, A_VPSLLW, A_VPSRAD, A_VPSRAW, A_VPSRLD, A_VPSRLDQ, A_VPSRLQ,
    A_VPSRLW, A_VPSUBB, A_VPSUBD, A_VPSUBQ, A_VPSUBSB, A_VPSUBSW, A_VPSUBUSB,
    A_VPSUBUSW, A_VPSUBW, A_VPTEST, A_VPUNPCKHBW, A_VPUNPCKHDQ, A_VPUNPCKHQDQ,
    A_VPUNPCKHWD, A_VPUNPCKLBW, A_VPUNPCKLDQ, A_VPUNPCKLQDQ, A_VPUNPCKLWD, A_VPXOR,
    A_VRCPPS, A_VRCPSS, A_VROUNDPD, A_VROUNDPS, A_VROUNDSD, A_VROUNDSS, A_VRSQRTPS,
    A_VRSQRTSS, A_VSHUFPD, A_VSHUFPS, A_VSQRTPD, A_VSQRTPS, A_VSQRTSD, A_VSQRTSS,
    A_VSTMXCSR, A_VSUBPD, A_VSUBPS, A_VSUBSD, A_VSUBSS, A_VTESTPD, A_VTESTPS,
    A_VUCOMISD, A_VUCOMISS, A_VUNPCKHPD, A_VUNPCKHPS, A_VUNPCKLPD, A_VUNPCKLPS,
    A_VXORPD, A_VXORPS, A_VZEROALL, A_VZEROUPPER, A_ANDN, A_BEXTR, A_TZCNT, A_BZHI,
    A_MULX, A_PDEP, A_PEXT, A_RORX, A_SARX, A_SHLX, A_SHRX, A_VBROADCASTI128,
    A_VEXTRACTI128, A_VINSERTI128, A_VPBLENDD, A_VPBROADCASTB, A_VPBROADCASTD,
    A_VPBROADCASTQ, A_VPBROADCASTW, A_VPERM2I128, A_VPERMD);

const
  ExtremeRange1Expected: array[0..1023] of Byte = (
    $00, $00, $00, $01, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 0 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 16 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 32 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 48 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 64 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 80 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 96 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 112 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 128 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $09, { 144 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 160 }
    $00, $00, $00, $00, $00, $00, $00, $02, $00, $00, $00, $08, $00, $00, $03, $03, { 176 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 192 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 208 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 224 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 240 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 256 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 272 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 288 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $0B, $00, $00, $00, $00, { 304 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $0B, $00, $00, $00, $00, $00, $00, { 320 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $0A, $00, $00, $00, $00, $00, { 336 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 352 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $0C, $00, $00, $00, $00, $00, $00, { 368 }
    $00, $00, $07, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 384 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 400 }
    $00, $00, $00, $00, $00, $05, $00, $00, $00, $00, $00, $00, $08, $05, $00, $07, { 416 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 432 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 448 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 464 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 480 }
    $07, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 496 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 512 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $05, $00, $00, $00, $05, { 528 }
    $00, $07, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 544 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 560 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 576 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 592 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 608 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 624 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 640 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 656 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $06, $06, $00, $00, $00, $00, { 672 }
    $00, $00, $00, $00, $00, $00, $06, $06, $00, $00, $00, $00, $00, $00, $00, $00, { 688 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 704 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 720 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 736 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 752 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 768 }
    $00, $00, $00, $00, $00, $00, $00, $06, $06, $00, $00, $00, $00, $00, $00, $00, { 784 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 800 }
    $04, $04, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 816 }
    $00, $00, $00, $08, $00, $00, $08, $04, $04, $00, $00, $00, $06, $06, $06, $06, { 832 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 848 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 864 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 880 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 896 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 912 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 928 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 944 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 960 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $06, $06, $00, { 976 }
    $00, $00, $00, $00, $00, $00, $00, $06, $06, $00, $00, $00, $00, $00, $00, $00, { 992 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00  { 1008 }
  );

  ExtremeRange2Expected: array[0..1023] of Byte = (
    $00, $00, $00, $01, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 0 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 16 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 32 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 48 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 64 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 80 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 96 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 112 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 128 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $09, { 144 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 160 }
    $00, $00, $00, $00, $00, $00, $00, $02, $00, $00, $00, $26, $00, $00, $0D, $03, { 176 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 192 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 208 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 224 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 240 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 256 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 272 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 288 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $0B, $00, $00, $00, $00, { 304 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $27, $00, $00, $00, $00, $00, $00, { 320 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $0A, $00, $00, $00, $00, $00, { 336 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 352 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $0C, $00, $00, $00, $00, $00, $00, { 368 }
    $00, $00, $07, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 384 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 400 }
    $00, $00, $00, $00, $00, $12, $00, $00, $00, $00, $00, $00, $08, $05, $00, $22, { 416 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 432 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 448 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 464 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 480 }
    $23, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 496 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 512 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $11, $00, $00, $00, $13, { 528 }
    $00, $21, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 544 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 560 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 576 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 592 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 608 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 624 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 640 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 656 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $1A, $1B, $00, $00, $00, $00, { 672 }
    $00, $00, $00, $00, $00, $00, $1C, $1D, $00, $00, $00, $00, $00, $00, $00, $00, { 688 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 704 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 720 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 736 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 752 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 768 }
    $00, $00, $00, $00, $00, $00, $00, $14, $15, $00, $00, $00, $00, $00, $00, $00, { 784 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 800 }
    $0F, $0E, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 816 }
    $00, $00, $00, $24, $00, $00, $25, $04, $10, $00, $00, $00, $18, $19, $1E, $1F, { 832 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 848 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 864 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 880 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 896 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 912 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 928 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 944 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 960 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $16, $17, $00, { 976 }
    $00, $00, $00, $00, $00, $00, $00, $20, $06, $00, $00, $00, $00, $00, $00, $00, { 992 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00  { 1008 }
  );

  ExtremeRange3Expected: array[0..1023] of Byte = (
    $00, $00, $44, $01, $3F, $40, $00, $00, $00, $41, $42, $00, $00, $00, $00, $00, { 0 }
    $00, $00, $00, $45, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 16 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 32 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 48 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 64 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 80 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 96 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 112 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 128 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $09, { 144 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 160 }
    $00, $00, $00, $00, $00, $00, $00, $02, $00, $00, $00, $26, $00, $00, $0D, $03, { 176 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 192 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 208 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 224 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 240 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 256 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 272 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 288 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $0B, $00, $00, $00, $00, { 304 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $27, $00, $00, $00, $00, $00, $00, { 320 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $0A, $00, $00, $00, $00, $00, { 336 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 352 }
    $00, $43, $00, $00, $00, $00, $00, $00, $00, $0C, $00, $00, $00, $00, $00, $00, { 368 }
    $00, $00, $07, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 384 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 400 }
    $00, $00, $00, $00, $00, $12, $00, $00, $00, $00, $00, $00, $08, $05, $00, $22, { 416 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 432 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 448 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 464 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 480 }
    $23, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 496 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 512 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $11, $00, $00, $00, $13, { 528 }
    $00, $21, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 544 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 560 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 576 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 592 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $33, $34, $00, $00, $00, { 608 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 624 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 640 }
    $00, $00, $00, $2E, $2F, $30, $31, $32, $00, $00, $00, $00, $00, $00, $00, $00, { 656 }
    $00, $00, $00, $00, $00, $00, $00, $00, $2C, $2D, $1A, $1B, $00, $00, $35, $36, { 672 }
    $37, $38, $39, $3A, $00, $00, $1C, $1D, $00, $00, $00, $00, $29, $00, $00, $00, { 688 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 704 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 720 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 736 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 752 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 768 }
    $00, $00, $00, $00, $00, $00, $00, $14, $15, $00, $00, $00, $00, $00, $00, $00, { 784 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 800 }
    $0F, $0E, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 816 }
    $00, $00, $00, $24, $00, $00, $25, $04, $10, $00, $00, $00, $18, $19, $1E, $1F, { 832 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 848 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 864 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 880 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 896 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 912 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $3B, $3C, $3D, $3E, $00, $00, $00, { 928 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 944 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, { 960 }
    $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $00, $16, $17, $00, { 976 }
    $00, $00, $00, $00, $00, $00, $00, $20, $06, $00, $00, $00, $00, $00, $00, $28, { 992 }
    $00, $00, $00, $00, $46, $47, $2A, $00, $00, $00, $00, $00, $00, $00, $00, $2B  { 1008 }
  );

{ TTestAncestor }
type
  TTestAncestor = class
    private
      FStartTime: Real;
      FEndTime: Real;
      FAvgTime: Real;
      procedure SetStartTime;
      procedure SetEndTime;
    protected
      procedure DoTestIteration(Iteration: Integer); virtual; abstract;
    public
      constructor Create; virtual;
      destructor Destroy; override;
      procedure Run;
      function TestTitle: shortstring; virtual; abstract;
      function WriteResults: Boolean; virtual; abstract;
      property RunTime: Real read FAvgTime;
  end;

  TTestClass = class of TTestAncestor;

  TByteTest = class(TTestAncestor)
    protected
      FResultStorage: array[Byte] of Byte;
  end;

  TWordTest = class(TTestAncestor)
    protected
      FResultStorage: array[Word] of Byte;
  end;

  TMappedTest = class(TByteTest)
    protected
      procedure DoMapping(Index, Input: Integer); virtual; abstract;
  end;

  TCompleteByteRange = class(TMappedTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
      procedure DoMapping(Index, Input: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TCompleteByteRangeFirstWeighted = class(TCompleteByteRange)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TCompleteByteRangeLastWeighted = class(TCompleteByteRange)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TAlmostFullByteRange = class(TMappedTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
      procedure DoMapping(Index, Input: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TAlmostFullByteRangeFirstWeighted = class(TAlmostFullByteRange)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TAlmostFullByteRangeLastWeighted = class(TAlmostFullByteRange)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWithDefault = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWithDefaultUnlikely = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWithDefaultWeighted = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWithElse = class(TSingleEntryWithDefault)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  TSingleEntryWithElseUnlikely = class(TSingleEntryWithDefaultUnlikely)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  TSingleEntryWithElseWeighted = class(TSingleEntryWithDefaultWeighted)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  TSingleEntryAtZeroWithElse = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryAtMinus1WithDefault = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryAtMinus4WithElse = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWith0To5RangeWithElse = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWith0To50RangeWithElse = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWith1To5RangeWithElse = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWith1To50RangeWithElse = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWithMinus1To5RangeWithElse = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSingleEntryWithMinus1To50RangeWithElse = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TExtremeRange1 = class(TWordTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TExtremeRange2 = class(TWordTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TExtremeRange3 = class(TWordTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TExtremeRange4 = class(TWordTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataTest1 = class(TWordTest)
    protected
      procedure DoCaseBlock(Index: Integer; Input: TInstructionSet); inline;
  end;

  TSparseDataEqual1 = class(TSparseDataTest1)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataMOVWeighted1 = class(TSparseDataTest1)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataMidpointWeighted1 = class(TSparseDataTest1)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataTest2 = class(TWordTest)
    protected
      procedure DoCaseBlock(Index: Integer; Input: TInstructionSet); inline;
  end;

  TSparseDataEqual2 = class(TSparseDataTest2)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataMOVWeighted2 = class(TSparseDataTest2)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataMidpointWeighted2 = class(TSparseDataTest2)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataTest3 = class(TWordTest)
    protected
      procedure DoCaseBlock(Index: Integer; Input: TInstructionSet); inline;
  end;

  TSparseDataEqual3 = class(TSparseDataTest3)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataMOVWeighted3 = class(TSparseDataTest3)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TSparseDataMidpointWeighted3 = class(TSparseDataTest3)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TLinearListDependsOnInput = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

  TCStyleCascade = class(TByteTest)
    protected
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
      function WriteResults: Boolean; override;
  end;

{ TTestAncestor }
constructor TTestAncestor.Create;
  begin
    FStartTime := 0;
    FEndTime := 0;
    FAvgTime := 0;
  end;

destructor TTestAncestor.Destroy;
  begin
    inherited Destroy;
  end;

procedure TTestAncestor.SetStartTime;
  begin
    FStartTime := GetRealTime();
  end;

procedure TTestAncestor.SetEndTime;
  begin
    FEndTime := GetRealTime();
    if FEndTime < FStartTime then { Happens if the test runs past midnight }
      FEndTime := FEndTime + 86400.0;
  end;

procedure TTestAncestor.Run;
  var
    X: Integer;
  begin
    SetStartTime;
    for X := 0 to ITERATIONS - 1 do
      DoTestIteration(X);

    SetEndTime;

    FAvgTime := FEndTime - FStartTime;
  end;

{ TCompleteByteRange }
function TCompleteByteRange.TestTitle: shortstring;
  begin
    Result := 'Byte domain, entirely covered; equal polling';
  end;

function TCompleteByteRange.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> AES_S_Box[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(AES_S_Box[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TCompleteByteRange.DoMapping(Index, Input: Integer);
  begin
    case Input of
      { First row of S-Box (except zero) }
      $00: FResultStorage[Index] := $63;
      $01: FResultStorage[Index] := $7c;
      $02: FResultStorage[Index] := $77;
      $03: FResultStorage[Index] := $7b;
      $04: FResultStorage[Index] := $f2;
      $05: FResultStorage[Index] := $6b;
      $06: FResultStorage[Index] := $6f;
      $07: FResultStorage[Index] := $c5;
      $08: FResultStorage[Index] := $30;
      $09: FResultStorage[Index] := $01;
      $0A: FResultStorage[Index] := $67;
      $0B: FResultStorage[Index] := $2b;
      $0C: FResultStorage[Index] := $fe;
      $0D: FResultStorage[Index] := $d7;
      $0E: FResultStorage[Index] := $ab;
      $0F: FResultStorage[Index] := $76;
      {Last row of S-Box }
      $F0: FResultStorage[Index] := $8c;
      $F1: FResultStorage[Index] := $a1;
      $F2: FResultStorage[Index] := $89;
      $F3: FResultStorage[Index] := $0d;
      $F4: FResultStorage[Index] := $bf;
      $F5: FResultStorage[Index] := $e6;
      $F6: FResultStorage[Index] := $42;
      $F7: FResultStorage[Index] := $68;
      $F8: FResultStorage[Index] := $41;
      $F9: FResultStorage[Index] := $99;
      $FA: FResultStorage[Index] := $2d;
      $FB: FResultStorage[Index] := $0f;
      $FC: FResultStorage[Index] := $b0;
      $FD: FResultStorage[Index] := $54;
      $FE: FResultStorage[Index] := $bb;
      $FF: FResultStorage[Index] := $16;
      { Everything else }
      $10..$EF: FResultStorage[Index] := AES_S_Box[Input];
    end;
  end;

procedure TCompleteByteRange.DoTestIteration(Iteration: Integer);
  var
    Input: Byte;
  begin
    Input := Iteration and $FF;
    DoMapping(Input, Input);
  end;

{ TCompleteByteRangeFirstWeighted }

function TCompleteByteRangeFirstWeighted.TestTitle: shortstring;
  begin
    Result := 'Byte domain, entirely covered; first weighted';
  end;

function TCompleteByteRangeFirstWeighted.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> FirstWeighted[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(FirstWeighted[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TCompleteByteRangeFirstWeighted.DoTestIteration(Iteration: Integer);
  var
    Input: Byte;
  begin
    Input := Iteration and $FF;
    if Input < $C0 then
      DoMapping(Input, 0)
    else
      DoMapping(Input, Input);
  end;

{ TCompleteByteRangeLastWeighted }

function TCompleteByteRangeLastWeighted.TestTitle: shortstring;
  begin
    Result := 'Byte domain, entirely covered; last weighted';
  end;

function TCompleteByteRangeLastWeighted.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> LastWeighted[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(LastWeighted[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TCompleteByteRangeLastWeighted.DoTestIteration(Iteration: Integer);
  var
    Input: Byte;
  begin
    Input := Iteration and $FF;
    if Input < $C0 then
      DoMapping(Input, $FF)
    else
      DoMapping(Input, Input);
  end;

{ TAlmostFullByteRange }

function TAlmostFullByteRange.TestTitle: shortstring;
  begin
    Result := 'Byte domain, almost entirely covered; equal polling';
  end;

function TAlmostFullByteRange.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> AlmostFullExpected[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(AlmostFullExpected[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TAlmostFullByteRange.DoMapping(Index, Input: Integer);
  begin
    case Input of
      { First row of S-Box }
      $00: FResultStorage[Index] := $63;
      $01: FResultStorage[Index] := $7c;
      $02: FResultStorage[Index] := $77;
      $03: FResultStorage[Index] := $7b;
      $04: FResultStorage[Index] := $f2;
      $05: FResultStorage[Index] := $6b;
      $06: FResultStorage[Index] := $6f;
      $07: FResultStorage[Index] := $c5;
      $08: FResultStorage[Index] := $30;
      $09: FResultStorage[Index] := $01;
      $0A: FResultStorage[Index] := $67;
      $0B: FResultStorage[Index] := $2b;
      $0C: FResultStorage[Index] := $fe;
      $0D: FResultStorage[Index] := $d7;
      $0E: FResultStorage[Index] := $ab;
      $0F: FResultStorage[Index] := $76;
      { Other rows }
      $10..$3F: FResultStorage[Index] := AES_S_Box[Input];
      $60..$FF: FResultStorage[Index] := AES_S_Box[Input];
      { Zeroed rows }
      else FResultStorage[Index] := $00;
    end;
  end;

procedure TAlmostFullByteRange.DoTestIteration(Iteration: Integer);
  var
    Input: Byte;
  begin
    Input := Iteration and $FF;
    DoMapping(Input, Input);
  end;

{ TAlmostFullByteRangeFirstWeighted }

function TAlmostFullByteRangeFirstWeighted.TestTitle: shortstring;
  begin
    Result := 'Byte domain, almost entirely covered; first weighted';
  end;

function TAlmostFullByteRangeFirstWeighted.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> FirstWeighted[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(FirstWeighted[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TAlmostFullByteRangeFirstWeighted.DoTestIteration(Iteration: Integer);
  var
    Input: Byte;
  begin
    Input := Iteration and $FF;
    if Input < $C0 then
      DoMapping(Input, 0)
    else
      DoMapping(Input, Input);
  end;

{ TAlmostFullByteRangeLastWeighted }

function TAlmostFullByteRangeLastWeighted.TestTitle: shortstring;
  begin
    Result := 'Byte domain, almost entirely covered; last weighted';
  end;

function TAlmostFullByteRangeLastWeighted.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> LastWeighted[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(LastWeighted[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TAlmostFullByteRangeLastWeighted.DoTestIteration(Iteration: Integer);
  var
    Input: Byte;
  begin
    Input := Iteration and $FF;
    if Input < $C0 then
      DoMapping(Input, $FF)
    else
      DoMapping(Input, Input);
  end;

{ TSingleEntryWithDefault }
function TSingleEntryWithDefault.TestTitle: shortstring;
  begin
    Result := 'Single entry with default value; 1/256 match chance';
  end;

function TSingleEntryWithDefault.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> IIf(X = 71, 1, 0) then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(IIf(X = 71, 1, 0), 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryWithDefault.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    FResultStorage[Index] := 0;
    case Index of
      71: FResultStorage[Index] := 1;
    end;
  end;

{ TSingleEntryWithDefaultUnlikely }

function TSingleEntryWithDefaultUnlikely.TestTitle: shortstring;
  begin
    Result := 'Single entry with default value; 75% match chance';
  end;

function TSingleEntryWithDefaultUnlikely.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> IIf(((X and $2) shr 1) or (X and $1) = 1, 1, 0) then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(IIf(((X and $2) shr 1) or (X and $1) = 1, 1, 0), 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryWithDefaultUnlikely.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    FResultStorage[Index] := 0;
    case ((Index and $2) shr 1) or (Index and $1) of
      1: FResultStorage[Index] := 1;
    end;
  end;

{ TSingleEntryWithDefaultWeighted }

function TSingleEntryWithDefaultWeighted.TestTitle: shortstring;
  begin
    Result := 'Single entry with default value; 25% match chance';
  end;

function TSingleEntryWithDefaultWeighted.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;

    for X := 0 to 255 do
      if FResultStorage[X] <> IIf(((X and $2) shr 1) and (X and $1) = 1, 1, 0) then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(IIf(((X and $2) shr 1) and (X and $1) = 1, 1, 0), 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryWithDefaultWeighted.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    FResultStorage[Index] := 0;
    case ((Index and $2) shr 1) and (Index and $1) of
      1: FResultStorage[Index] := 1;
    end;
  end;

{ TSingleEntryWithElse }
function TSingleEntryWithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry with else block; 1/256 match chance';
  end;

procedure TSingleEntryWithElse.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case Index of
      71: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryWithElseUnlikely }
function TSingleEntryWithElseUnlikely.TestTitle: shortstring;
  begin
    Result := 'Single entry with else block; 75% match chance';
  end;

procedure TSingleEntryWithElseUnlikely.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case ((Index and $2) shr 1) or (Index and $1) of
      1: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryWithElseWeighted }

function TSingleEntryWithElseWeighted.TestTitle: shortstring;
  begin
    Result := 'Single entry with else block; 25% match chance';
  end;

procedure TSingleEntryWithElseWeighted.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case ((Index and $2) shr 1) and (Index and $1) of
      1: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryAtZeroWithElse }

function TSingleEntryAtZeroWithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry of "0:" and else block';
  end;

function TSingleEntryAtZeroWithElse.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    if FResultStorage[0] <> 1 then
      begin
        WriteLn('FAIL - Index 0; expected $01 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;

    for X := 1 to $FF do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryAtZeroWithElse.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case Index of
      0: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryAtMinus1WithDefault }

function TSingleEntryAtMinus1WithDefault.TestTitle: shortstring;
  begin
    Result := 'Single entry of "-1:" with default value';
  end;

function TSingleEntryAtMinus1WithDefault.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    for X := 0 to $FE do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    if FResultStorage[255] <> 1 then
      begin
        WriteLn('FAIL - Index 255; expected $01 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;
  end;

procedure TSingleEntryAtMinus1WithDefault.DoTestIteration(Iteration: Integer);
  var
    Index: ShortInt;
  begin
    Index := ShortInt(Iteration and $FF);
    FResultStorage[Byte(Index)] := 0;
    case Index of
      -1: FResultStorage[255] := 1;
    end;
  end;

{ TSingleEntryAtMinus4WithElse }

function TSingleEntryAtMinus4WithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry of "-4:" and else block';
  end;

function TSingleEntryAtMinus4WithElse.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;
    for X := 0 to 251 do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    if FResultStorage[252] <> 1 then
      begin
        WriteLn('FAIL - Index 0; expected $01 got $', hexstr(FResultStorage[252], 2));
        Result := False;
        Exit;
      end;

    for X := 253 to 255 do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryAtMinus4WithElse.DoTestIteration(Iteration: Integer);
  var
    Index: ShortInt;
  begin
    Index := ShortInt(Iteration and $FF);
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Byte(Index)] := $FF;
    case Index of
      -4: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryWith0To5RangeWithElse }

function TSingleEntryWith0To5RangeWithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry of "0..5" and else block';
  end;

function TSingleEntryWith0To5RangeWithElse.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    for X := 0 to 5 do
      if FResultStorage[X] <> 1 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $01 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    for X := 6 to $FF do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryWith0To5RangeWithElse.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case Index of
      0..5: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryWith0To50RangeWithElse }

function TSingleEntryWith0To50RangeWithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry of "0..50" and else block';
  end;

function TSingleEntryWith0To50RangeWithElse.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    for X := 0 to 50 do
      if FResultStorage[X] <> 1 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $01 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    for X := 51 to $FF do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryWith0To50RangeWithElse.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case Index of
      0..50: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryWith1To5RangeWithElse }

function TSingleEntryWith1To5RangeWithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry of "1..5" and else block';
  end;

function TSingleEntryWith1To5RangeWithElse.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;
    if FResultStorage[0] <> 0 then
      begin
        WriteLn('FAIL - Index 0; expected $00 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;

    for X := 1 to 5 do
      if FResultStorage[X] <> 1 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $01 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    for X := 6 to $FF do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryWith1To5RangeWithElse.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case Index of
      1..5: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryWith1To50RangeWithElse }

function TSingleEntryWith1To50RangeWithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry of "1..50" and else block';
  end;

function TSingleEntryWith1To50RangeWithElse.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;
    if FResultStorage[0] <> 0 then
      begin
        WriteLn('FAIL - Index 0; expected $00 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;

    for X := 1 to 50 do
      if FResultStorage[X] <> 1 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $01 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    for X := 51 to $FF do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSingleEntryWith1To50RangeWithElse.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case Index of
      1..50: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;


{ TSingleEntryWithMinus1To5RangeWithElse }

function TSingleEntryWithMinus1To5RangeWithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry of "-1..5" and else block';
  end;

function TSingleEntryWithMinus1To5RangeWithElse.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;
    for X := 0 to 5 do
      if FResultStorage[X] <> 1 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $01 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    for X := 6 to $FE do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    if FResultStorage[$FF] <> 1 then
      begin
        WriteLn('FAIL - Index 255; expected $00 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;
  end;

procedure TSingleEntryWithMinus1To5RangeWithElse.DoTestIteration(Iteration: Integer);
  var
    Index: ShortInt;
  begin
    Index := ShortInt(Iteration and $FF);
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Byte(Index)] := $FF;
    case Index of
      -1..5: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TSingleEntryWithMinus1To50RangeWithElse }

function TSingleEntryWithMinus1To50RangeWithElse.TestTitle: shortstring;
  begin
    Result := 'Single entry of "-1..50" and else block';
  end;

function TSingleEntryWithMinus1To50RangeWithElse.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;
    for X := 0 to 50 do
      if FResultStorage[X] <> 1 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $01 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    for X := 51 to $FE do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    if FResultStorage[$FF] <> 1 then
      begin
        WriteLn('FAIL - Index 255; expected $00 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;
  end;

procedure TSingleEntryWithMinus1To50RangeWithElse.DoTestIteration(Iteration: Integer);
  var
    Index: ShortInt;
  begin
    Index := ShortInt(Iteration and $FF);
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Byte(Index)] := $FF;
    case Index of
      -1..50: FResultStorage[Index] := 1;
      else FResultStorage[Index] := 0;
    end;
  end;

{ TExtremeRange1 }

function TExtremeRange1.TestTitle: shortstring;
  begin
    Result := 'Two labels, one with extreme spread, equal polling';
  end;

function TExtremeRange1.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    if FResultStorage[0] <> 1 then
      begin
        WriteLn('FAIL - Index 0; expected $01 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;

    for X := 1 to $FFFE do
      if FResultStorage[X] <> 2 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $02 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    if FResultStorage[65535] <> 0 then
      begin
        WriteLn('FAIL - Index 65535; expected $02 got $', hexstr(FResultStorage[65535], 2));
        Result := False;
        Exit;
      end;
  end;

procedure TExtremeRange1.DoTestIteration(Iteration: Integer);
  var
    Index: Word;
  begin
    Index := Iteration and $FFFF;
    FResultStorage[Index] := 0; { Covers $FFFF }
    case Index of
      0:
        FResultStorage[Index] := 1;
      1..$FFFE:
        FResultStorage[Index] := 2;
    end;
  end;

{ TExtremeRange2 }

function TExtremeRange2.TestTitle: shortstring;
  begin
    Result := 'Two labels, one with extreme spread, 50% else chance';
  end;

function TExtremeRange2.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    for X := 0 to $FFFF do
      if FResultStorage[X] <> (X and $1) then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(X and $1, 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TExtremeRange2.DoTestIteration(Iteration: Integer);
  var
    Index, Input: Word;
  begin
    Index := (Iteration and $FFFF);
    Input := (Iteration and $1) - 1;
    FResultStorage[Index] := 0; { Covers $FFFF }
    case Input of
      0..$FFFD:
        FResultStorage[Index] := 1;
      $FFFE:
        FResultStorage[Index] := 2;
    end;
  end;

{ TExtremeRange3 }

function TExtremeRange3.TestTitle: shortstring;
  begin
    Result := 'Two labels, sparse values, equal polling across range';
  end;

function TExtremeRange3.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    if FResultStorage[0] <> 1 then
      begin
        WriteLn('FAIL - Index 0; expected $01 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;

    for X := 1 to $FFFE do
      if FResultStorage[X] <> 2 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $02 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    if FResultStorage[65535] <> 0 then
      begin
        WriteLn('FAIL - Index 65535; expected $02 got $', hexstr(FResultStorage[65535], 2));
        Result := False;
        Exit;
      end;
  end;

procedure TExtremeRange3.DoTestIteration(Iteration: Integer);
  var
    Index: Word;
  begin
    Index := Iteration and $FFFF;
    FResultStorage[Index] := 2; { Covers 1..$FFFE }
    case Index of
      0:
        FResultStorage[Index] := 1;
      $FFFF:
        FResultStorage[Index] := 0;
    end;
  end;

{ TExtremeRange4 }

function TExtremeRange4.TestTitle: shortstring;
  begin
    Result := 'Two labels, sparse values, always triggered';
  end;

function TExtremeRange4.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    for X := 0 to $FFFF do
      if FResultStorage[X] <> (X and $1) then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(X and $1, 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TExtremeRange4.DoTestIteration(Iteration: Integer);
  var
    Index, Input: Word;
  begin
    Index := (Iteration and $FFFF);
    Input := (Iteration and $1) - 1;
    FResultStorage[Index] := 2; { Covers 1..$FFFE }
    case Input of
      0:
        FResultStorage[Index] := 1;
      $FFFF:
        FResultStorage[Index] := 0;
    end;
  end;

{ TSparseDataTest1 }

procedure TSparseDataTest1.DoCaseBlock(Index: Integer; Input: TInstructionSet);
  begin
    case Input of
      A_AND:
        FResultStorage[Index] := 1;
      A_MOV:
        FResultStorage[Index] := 2;
      A_MOVSX,
      A_MOVZX:
        FResultStorage[Index] := 3;
      A_VMOVAPS,
      A_VMOVAPD,
      A_VMOVUPS,
      A_VMOVUPD:
        FResultStorage[Index] := 4;
      A_MOVAPD,
      A_MOVAPS,
      A_MOVUPD,
      A_MOVUPS:
        FResultStorage[Index] := 5;
      A_VDIVSD,
      A_VDIVSS,
      A_VSUBSD,
      A_VSUBSS,
      A_VMULSD,
      A_VMULSS,
      A_VADDSD,
      A_VADDSS,
      A_VANDPD,
      A_VANDPS,
      A_VORPD,
      A_VORPS,
      A_VXORPD,
      A_VXORPS:
        FResultStorage[Index] := 6;
      A_MULSD,
      A_MULSS,
      A_ADDSD,
      A_ADDSS:
        FResultStorage[Index] := 7;
      A_VMOVSD,
      A_VMOVSS,
      A_MOVSD,
      A_MOVSS:
        FResultStorage[Index] := 8;
      A_LEA:
        FResultStorage[Index] := 9;
      A_SUB:
        FResultStorage[Index] := 10;
      A_SHL,A_SAL:
        FResultStorage[Index] := 11;
      A_SETcc:
        FResultStorage[Index] := 12;
      else
        FResultStorage[Index] := 0;
    end;
  end;

{ TSparseDataEqual1 }

function TSparseDataEqual1.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 12 sparse labels, equal polling';
  end;

function TSparseDataEqual1.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      if FResultStorage[X] <> ExtremeRange1Expected[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(ExtremeRange1Expected[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSparseDataEqual1.DoTestIteration(Iteration: Integer);
  var
    X: SmallInt;
  begin
    X := Iteration and 1023;
    DoCaseBlock(X, TInstructionSet(X - 512))
  end;

{ TSparseDataMOVWeightedl }

function TSparseDataMOVWeighted1.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 12 sparse labels, 75% particular match';
  end;

function TSparseDataMOVWeighted1.WriteResults: Boolean;
  var
    X, Expected: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      begin
        Expected := IIf((X and $3) = 0, ExtremeRange1Expected[X], 2);
        if FResultStorage[X] <> Expected then
          begin
            WriteLn('FAIL - Index ', X, '; expected $', hexstr(Expected, 2), ' got $', hexstr(FResultStorage[X], 2));
            Result := False;
            Exit;
          end;
      end;
  end;

procedure TSparseDataMOVWeighted1.DoTestIteration(Iteration: Integer);
  var
    X: SmallInt; P: TInstructionSet;
  begin
    X := Iteration and 1023;
    P := TInstructionSet(IIf((X and $3) = 0, X - 512, Ord(A_MOV)));
    DoCaseBlock(X, P);
  end;

{ TSparseDataMidpointWeighted1 }

function TSparseDataMidpointWeighted1.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 12 sparse labels, 75% midpoint match';
  end;

function TSparseDataMidpointWeighted1.WriteResults: Boolean;
  var
    X, Expected: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      begin
        Expected := IIf((X and $3) = 0, ExtremeRange1Expected[X], 6);
        if FResultStorage[X] <> Expected then
          begin
            WriteLn('FAIL - Index ', X, '; expected $', hexstr(Expected, 2), ' got $', hexstr(FResultStorage[X], 2));
            Result := False;
            Exit;
          end;
      end;
  end;

procedure TSparseDataMidpointWeighted1.DoTestIteration(Iteration: Integer);
  var
    X: Word; P: TInstructionSet;
  begin
    X := Iteration and 1023;
    P := TInstructionSet(IIf((X and $3) = 0, X - 512, Ord(A_VADDSD)));
    DoCaseBlock(X, P);
  end;

{ TSparseDataTest2 }

procedure TSparseDataTest2.DoCaseBlock(Index: Integer; Input: TInstructionSet);
  begin
    case Input of
      A_AND:
        FResultStorage[Index] := 1;
      A_MOV:
        FResultStorage[Index] := 2;
      A_MOVSX:
        FResultStorage[Index] := 13;
      A_MOVZX:
        FResultStorage[Index] := 3;
      A_VMOVAPS:
        FResultStorage[Index] := 14;
      A_VMOVAPD:
        FResultStorage[Index] := 15;
      A_VMOVUPS:
        FResultStorage[Index] := 16;
      A_VMOVUPD:
        FResultStorage[Index] := 4;
      A_MOVAPD:
        FResultStorage[Index] := 17;
      A_MOVAPS:
        FResultStorage[Index] := 18;
      A_MOVUPD:
        FResultStorage[Index] := 19;
      A_MOVUPS:
        FResultStorage[Index] := 5;
      A_VDIVSD:
        FResultStorage[Index] := 20;
      A_VDIVSS:
        FResultStorage[Index] := 21;
      A_VSUBSD:
        FResultStorage[Index] := 22;
      A_VSUBSS:
        FResultStorage[Index] := 23;
      A_VMULSD:
        FResultStorage[Index] := 24;
      A_VMULSS:
        FResultStorage[Index] := 25;
      A_VADDSD:
        FResultStorage[Index] := 26;
      A_VADDSS:
        FResultStorage[Index] := 27;
      A_VANDPD:
        FResultStorage[Index] := 28;
      A_VANDPS:
        FResultStorage[Index] := 29;
      A_VORPD:
        FResultStorage[Index] := 30;
      A_VORPS:
        FResultStorage[Index] := 31;
      A_VXORPD:
        FResultStorage[Index] := 32;
      A_VXORPS:
        FResultStorage[Index] := 6;
      A_MULSD:
        FResultStorage[Index] := 33;
      A_MULSS:
        FResultStorage[Index] := 34;
      A_ADDSD:
        FResultStorage[Index] := 35;
      A_ADDSS:
        FResultStorage[Index] := 7;
      A_VMOVSD:
        FResultStorage[Index] := 36;
      A_VMOVSS:
        FResultStorage[Index] := 37;
      A_MOVSD:
        FResultStorage[Index] := 38;
      A_MOVSS:
        FResultStorage[Index] := 8;
      A_LEA:
        FResultStorage[Index] := 9;
      A_SUB:
        FResultStorage[Index] := 10;
      A_SHL:
        FResultStorage[Index] := 39;
      A_SAL:
        FResultStorage[Index] := 11;
      A_SETcc:
        FResultStorage[Index] := 12;
      else
        FResultStorage[Index] := 0;
    end;
  end;

{ TSparseDataEqual2 }

function TSparseDataEqual2.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 39 sparse labels, equal polling';
  end;

function TSparseDataEqual2.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      if FResultStorage[X] <> ExtremeRange2Expected[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(ExtremeRange2Expected[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSparseDataEqual2.DoTestIteration(Iteration: Integer);
  var
    X: SmallInt;
  begin
    X := Iteration and 1023;
    DoCaseBlock(X, TInstructionSet(X - 512))
  end;

{ TSparseDataMOVWeighted2 }

function TSparseDataMOVWeighted2.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 39 sparse labels, 75% particular match';
  end;

function TSparseDataMOVWeighted2.WriteResults: Boolean;
  var
    X, Expected: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      begin
        Expected := IIf((X and $3) = 0, ExtremeRange2Expected[X], 2);
        if FResultStorage[X] <> Expected then
          begin
            WriteLn('FAIL - Index ', X, '; expected $', hexstr(Expected, 2), ' got $', hexstr(FResultStorage[X], 2));
            Result := False;
            Exit;
          end;
      end;
  end;

procedure TSparseDataMOVWeighted2.DoTestIteration(Iteration: Integer);
  var
    X: SmallInt; P: TInstructionSet;
  begin
    X := Iteration and 1023;
    P := TInstructionSet(IIf((X and $3) = 0, X - 512, Ord(A_MOV)));
    DoCaseBlock(X, P);
  end;

{ TSparseDataMidpointWeighted2 }

function TSparseDataMidpointWeighted2.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 39 sparse labels, 75% midpoint match';
  end;

function TSparseDataMidpointWeighted2.WriteResults: Boolean;
  var
    X, Expected: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      begin
        Expected := IIf((X and $3) = 0, ExtremeRange2Expected[X], 26);
        if FResultStorage[X] <> Expected then
          begin
            WriteLn('FAIL - Index ', X, '; expected $', hexstr(Expected, 2), ' got $', hexstr(FResultStorage[X], 2));
            Result := False;
            Exit;
          end;
      end;
  end;

procedure TSparseDataMidpointWeighted2.DoTestIteration(Iteration: Integer);
  var
    X: SmallInt; P: TInstructionSet;
  begin
    X := Iteration and 1023;
    P := TInstructionSet(IIf((X and $3) = 0, X - 512, Ord(A_VADDSD)));
    DoCaseBlock(X, P);
  end;

{ TSparseDataTest3 }

procedure TSparseDataTest3.DoCaseBlock(Index: Integer; Input: TInstructionSet);
  begin
    case Input of
      A_AND:
        FResultStorage[Index] := 1;
      A_MOV:
        FResultStorage[Index] := 2;
      A_MOVSX:
        FResultStorage[Index] := 13;
      A_MOVZX:
        FResultStorage[Index] := 3;
      A_VMOVAPS:
        FResultStorage[Index] := 14;
      A_VMOVAPD:
        FResultStorage[Index] := 15;
      A_VMOVUPS:
        FResultStorage[Index] := 16;
      A_VMOVUPD:
        FResultStorage[Index] := 4;
      A_MOVAPD:
        FResultStorage[Index] := 17;
      A_MOVAPS:
        FResultStorage[Index] := 18;
      A_MOVUPD:
        FResultStorage[Index] := 19;
      A_MOVUPS:
        FResultStorage[Index] := 5;
      A_VDIVSD:
        FResultStorage[Index] := 20;
      A_VDIVSS:
        FResultStorage[Index] := 21;
      A_VSUBSD:
        FResultStorage[Index] := 22;
      A_VSUBSS:
        FResultStorage[Index] := 23;
      A_VMULSD:
        FResultStorage[Index] := 24;
      A_VMULSS:
        FResultStorage[Index] := 25;
      A_VADDSD:
        FResultStorage[Index] := 26;
      A_VADDSS:
        FResultStorage[Index] := 27;
      A_VANDPD:
        FResultStorage[Index] := 28;
      A_VANDPS:
        FResultStorage[Index] := 29;
      A_VORPD:
        FResultStorage[Index] := 30;
      A_VORPS:
        FResultStorage[Index] := 31;
      A_VXORPD:
        FResultStorage[Index] := 32;
      A_VXORPS:
        FResultStorage[Index] := 6;
      A_MULSD:
        FResultStorage[Index] := 33;
      A_MULSS:
        FResultStorage[Index] := 34;
      A_ADDSD:
        FResultStorage[Index] := 35;
      A_ADDSS:
        FResultStorage[Index] := 7;
      A_VMOVSD:
        FResultStorage[Index] := 36;
      A_VMOVSS:
        FResultStorage[Index] := 37;
      A_MOVSD:
        FResultStorage[Index] := 38;
      A_MOVSS:
        FResultStorage[Index] := 8;
      A_LEA:
        FResultStorage[Index] := 9;
      A_SUB:
        FResultStorage[Index] := 10;
      A_SHL:
        FResultStorage[Index] := 39;
      A_SAL:
        FResultStorage[Index] := 11;
      A_SETcc:
        FResultStorage[Index] := 12;
      A_MULX:
        FResultStorage[Index] := 40;
      A_VBROADCASTF128:
        FResultStorage[Index] := 41;
      A_VBROADCASTI128:
        FResultStorage[Index] := 42;
      A_VPERMD:
        FResultStorage[Index] := 43;
      A_VADDPD:
        FResultStorage[Index] := 44;
      A_VADDPS:
        FResultStorage[Index] := 45;
      A_ROUNDPS:
        FResultStorage[Index] := 46;
      A_ROUNDPD:
        FResultStorage[Index] := 47;
      A_ROUNDSS:
        FResultStorage[Index] := 48;
      A_ROUNDSD:
        FResultStorage[Index] := 49;
      A_CRC32:
        FResultStorage[Index] := 50;
      A_DPPS:
        FResultStorage[Index] := 51;
      A_DPPD:
        FResultStorage[Index] := 52;
      A_VAESDEC:
        FResultStorage[Index] := 53;
      A_VAESDECLAST:
        FResultStorage[Index] := 54;
      A_VAESENC:
        FResultStorage[Index] := 55;
      A_VAESENCLAST:
        FResultStorage[Index] := 56;
      A_VAESIMC:
        FResultStorage[Index] := 57;
      A_VAESKEYGENASSIST:
        FResultStorage[Index] := 58;
      A_VPSHUFB:
        FResultStorage[Index] := 59;
      A_VPSHUFD:
        FResultStorage[Index] := 60;
      A_VPSHUFHW:
        FResultStorage[Index] := 61;
      A_VPSHUFLW:
        FResultStorage[Index] := 62;
      A_BSF:
        FResultStorage[Index] := 63;
      A_BSR:
        FResultStorage[Index] := 64;
      A_BTR:
        FResultStorage[Index] := 65;
      A_BTS:
        FResultStorage[Index] := 66;
      A_XOR:
        FResultStorage[Index] := 67;
      A_ADD:
        FResultStorage[Index] := 68;
      A_CMP:
        FResultStorage[Index] := 69;
      A_SHLX:
        FResultStorage[Index] := 70;
      A_SHRX:
        FResultStorage[Index] := 71;
      else
        FResultStorage[Index] := 0;
    end;
  end;

{ TSparseDataEqual3 }

function TSparseDataEqual3.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 71 sparse labels, equal polling';
  end;

function TSparseDataEqual3.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      if FResultStorage[X] <> ExtremeRange3Expected[X] then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(ExtremeRange3Expected[X], 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TSparseDataEqual3.DoTestIteration(Iteration: Integer);
  var
    X: SmallInt;
  begin
    X := Iteration and 1023;
    DoCaseBlock(X, TInstructionSet(X - 512))
  end;

{ TSparseDataMOVWeightedl }

function TSparseDataMOVWeighted3.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 71 sparse labels, 75% particular match';
  end;

function TSparseDataMOVWeighted3.WriteResults: Boolean;
  var
    X, Expected: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      begin
        Expected := IIf((X and $3) = 0, ExtremeRange3Expected[X], 2);
        if FResultStorage[X] <> Expected then
          begin
            WriteLn('FAIL - Index ', X, '; expected $', hexstr(Expected, 2), ' got $', hexstr(FResultStorage[X], 2));
            Result := False;
            Exit;
          end;
      end;
  end;

procedure TSparseDataMOVWeighted3.DoTestIteration(Iteration: Integer);
  var
    X: SmallInt; P: TInstructionSet;
  begin
    X := Iteration and 1023;
    P := TInstructionSet(IIf((X and $3) = 0, X - 512, Ord(A_MOV)));
    DoCaseBlock(X, P);
  end;

{ TSparseDataMidpointWeighted3 }

function TSparseDataMidpointWeighted3.TestTitle: shortstring;
  begin
    Result := 'Domain of 1024, 71 sparse labels, 75% midpoint match';
  end;

function TSparseDataMidpointWeighted3.WriteResults: Boolean;
  var
    X, Expected: Word;
  begin
    Result := True;

    for X := 0 to 1023 do
      begin
        Expected := IIf((X and $3) = 0, ExtremeRange3Expected[X], 26);
        if FResultStorage[X] <> Expected then
          begin
            WriteLn('FAIL - Index ', X, '; expected $', hexstr(Expected, 2), ' got $', hexstr(FResultStorage[X], 2));
            Result := False;
            Exit;
          end;
      end;
  end;

procedure TSparseDataMidpointWeighted3.DoTestIteration(Iteration: Integer);
  var
    X: SmallInt; P: TInstructionSet;
  begin
    X := Iteration and 1023;
    P := TInstructionSet(IIf((X and $3) = 0, X - 512, Ord(A_VADDSD)));
    DoCaseBlock(X, P);
  end;


{ TLinearListDependsOnInput }

function TLinearListDependsOnInput.TestTitle: shortstring;
  begin
    Result := 'Linear list depends on input';
  end;

function TLinearListDependsOnInput.WriteResults: Boolean;
  var
    X: Word;
  begin
    Result := True;
    if FResultStorage[0] <> 0 then
      begin
        WriteLn('FAIL - Index 0; expected $00 got $', hexstr(FResultStorage[0], 2));
        Result := False;
        Exit;
      end;

    for X := 1 to 7 do
      if FResultStorage[X] <> (X and $3) then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr(X and $3, 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    for X := 8 to 11 do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    if FResultStorage[12] <> $10 then
      begin
        WriteLn('FAIL - Index 12; expected $10 got $', hexstr(FResultStorage[12], 2));
        Result := False;
        Exit;
      end;

    for X := 13 to $FF do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TLinearListDependsOnInput.DoTestIteration(Iteration: Integer);
  var
    Index: Byte;
  begin
    Index := Iteration and $FF;
    { This helps catch errors where all branches, including else, are skipped }
    FResultStorage[Index] := $FF;
    case Index of
      1..3: FResultStorage[Index] := Index;
      4..7: FResultStorage[Index] := Index - 4;
      12:   FResultStorage[Index] := $10;
      else  FResultStorage[Index] := 0;
    end;
  end;


{ TCStyleCascade }

function TCStyleCascade.TestTitle: shortstring;
  begin
    Result := 'C-style cascade using ''goto''';
  end;

function TCStyleCascade.WriteResults: Boolean;
  var
    X: Byte;
  begin
    Result := True;
    for X := 0 to 5 do
      if FResultStorage[X] <> ((1 shl X) - 1) then
        begin
          WriteLn('FAIL - Index ', X, '; expected $', hexstr((1 shl X) - 1, 2), ' got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;

    for X := 6 to $FF do
      if FResultStorage[X] <> 0 then
        begin
          WriteLn('FAIL - Index ', X, '; expected $00 got $', hexstr(FResultStorage[X], 2));
          Result := False;
          Exit;
        end;
  end;

procedure TCStyleCascade.DoTestIteration(Iteration: Integer);
  var
    X, Tmp: Byte;
  label
    Set1, Set2, Set3, Set4, Default;
  begin
    X := Iteration and $FF;
    Tmp := 0;
    case X of
      $1: goto Set1;
      $2: goto Set2;
      $3: goto Set3;
      $4: goto Set4;
      $5: Tmp := 16;
    else
      goto Default;
    end;
  Set4:
    Tmp := Tmp or $8;
  Set3:
    Tmp := Tmp or $4;
  Set2:
    Tmp := Tmp or $2;
  Set1:
    Tmp := Tmp or $1;
  Default:
    FResultStorage[X] := Tmp;
  end;


{ Main function }
const
  { TCompleteByteRange and descendants
      - Entirely-covered jump tree
      - 33 labels, no else block; full coverage (all 256 byte values covered)
        - Root: values are polled with equal probability
        - FirstWeighted: first branch is polled 3 times as often
        - LastWeighted: last branch is polled 3 times as often
    TAlmostFullByteRange
      - Almost full jump tree - 18 labels, else block covers 32 values; 224 byte values covered
        - Root: values are polled with equal probability
        - FirstWeighted: first branch is polled 3 times as often
        - LastWeighted: last branch is polled 3 times as often
  }

  TestClasses: array[0..35] of TTestClass = (
    TCompleteByteRange,
    TCompleteByteRangeFirstWeighted,
    TCompleteByteRangeLastWeighted,
    TAlmostFullByteRange,
    TAlmostFullByteRangeFirstWeighted,
    TAlmostFullByteRangeLastWeighted,
    TSingleEntryWithDefault,
    TSingleEntryWithDefaultUnlikely,
    TSingleEntryWithDefaultWeighted,
    TSingleEntryWithElse,
    TSingleEntryWithElseUnlikely,
    TSingleEntryWithElseWeighted,
    TSingleEntryAtZeroWithElse,
    TSingleEntryAtMinus1WithDefault,
    TSingleEntryAtMinus4WithElse,
    TSingleEntryWith0To5RangeWithElse,
    TSingleEntryWith0To50RangeWithElse,
    TSingleEntryWith1To5RangeWithElse,
    TSingleEntryWith1To50RangeWithElse,
    TSingleEntryWithMinus1To5RangeWithElse,
    TSingleEntryWithMinus1To50RangeWithElse,
    TExtremeRange1,
    TExtremeRange2,
    TExtremeRange3,
    TExtremeRange4,
    TSparseDataEqual1,
    TSparseDataMOVWeighted1,
    TSparseDataMidpointWeighted1,
    TSparseDataEqual2,
    TSparseDataMOVWeighted2,
    TSparseDataMidpointWeighted2,
    TSparseDataEqual3,
    TSparseDataMOVWeighted3,
    TSparseDataMidpointWeighted3,
    TLinearListDependsOnInput,
    TCStyleCascade
  );

var
  CurrentObject: TTestAncestor;
  Failed: Boolean;
  X: Integer;
  SummedUpAverageDuration, AverageDuration : Double;
begin
  SummedUpAverageDuration := 0.0;
  Failed := False;
  WriteLn('Case node compilation and timing test');
  WriteLn('-------------------------------------');
  for X := low(TestClasses) to High(TestClasses) do
    begin
      try
        CurrentObject := TestClasses[X].Create;
        try
          Write(CurrentObject.TestTitle:56, ' - ');
          CurrentObject.Run;

          if CurrentObject.WriteResults then
            begin
              AverageDuration := ((CurrentObject.RunTime * 1000000000.0) / ITERATIONS);
              WriteLn('Pass - average iteration duration: ', AverageDuration:1:3, ' ns');
              SummedUpAverageDuration := SummedUpAverageDuration + AverageDuration;
            end
          else
            { Final average isn't processed if a test failed, so there's no need
              to calculate and add the average duration to it }
            Failed := True;

        finally
          CurrentObject.Free;
        end;
      except on E: Exception do
        begin
          WriteLn('Exception "', E.ClassName, '" raised while running test object of class "', TestClasses[X].ClassName, '"');
          Failed := True;
        end;
      end;
    end;

  if Failed then
    Halt(1);

  WriteLn(#10'ok');
  WriteLn('- Sum of average durations: ', SummedUpAverageDuration:1:3, ' ns');
  WriteLn('- Overall average duration: ', (SummedUpAverageDuration / Length(TestClasses)):1:3, ' ns');
end.
