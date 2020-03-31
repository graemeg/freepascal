in_x86_movss
,in_x86_movapd
,in_x86_movhpd
,in_x86_movlpd
,in_x86_movupd
,in_x86_movsd_from_mem
,in_x86_movd_from_mem
,in_x86_pmovsxbd_from_mem
,in_x86_pmovzxbd_from_mem
,in_x86_pmovsxwq_from_mem
,in_x86_pmovzxwq_from_mem
: //out r0:xmm;r1:ptr32;
  begin
    case inlinenumber of
      in_x86_pmovzxwq_from_mem: begin op:=A_pmovzxwq end;
      in_x86_pmovsxwq_from_mem: begin op:=A_pmovsxwq end;
      in_x86_pmovzxbd_from_mem: begin op:=A_pmovzxbd end;
      in_x86_pmovsxbd_from_mem: begin op:=A_pmovsxbd end;
      in_x86_movd_from_mem: begin op:=A_movd end;
      in_x86_movsd_from_mem: begin op:=A_movsd end;
      in_x86_movupd: begin op:=A_movupd end;
      in_x86_movlpd: begin op:=A_movlpd end;
      in_x86_movhpd: begin op:=A_movhpd end;
      in_x86_movapd: begin op:=A_movapd end;
      in_x86_movss: begin op:=A_movss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[1].location.reference,location.register));
  end;
in_x86_movaps
,in_x86_movups
,in_x86_sqrtpd_from_mem
,in_x86_movdqa_from_mem
,in_x86_movdqu_from_mem
,in_x86_movsldup_from_mem
,in_x86_movshdup_from_mem
,in_x86_lddqu
,in_x86_pabsb_from_mem
,in_x86_pabsw_from_mem
,in_x86_pabsd_from_mem
,in_x86_movntdqa
: //out r0:xmm;r1:ptr128;
  begin
    case inlinenumber of
      in_x86_movntdqa: begin op:=A_movntdqa end;
      in_x86_pabsd_from_mem: begin op:=A_pabsd end;
      in_x86_pabsw_from_mem: begin op:=A_pabsw end;
      in_x86_pabsb_from_mem: begin op:=A_pabsb end;
      in_x86_lddqu: begin op:=A_lddqu end;
      in_x86_movshdup_from_mem: begin op:=A_movshdup end;
      in_x86_movsldup_from_mem: begin op:=A_movsldup end;
      in_x86_movdqu_from_mem: begin op:=A_movdqu end;
      in_x86_movdqa_from_mem: begin op:=A_movdqa end;
      in_x86_sqrtpd_from_mem: begin op:=A_sqrtpd end;
      in_x86_movups: begin op:=A_movups end;
      in_x86_movaps: begin op:=A_movaps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[1].location.reference,location.register));
  end;
in_x86_movss_to_mem
,in_x86_movapd_to_mem
,in_x86_movntpd_to_mem
,in_x86_movhpd_to_mem
,in_x86_movlpd_to_mem
,in_x86_movupd_to_mem
,in_x86_movsd_to_mem
,in_x86_movd_to_mem
: //r0:ptr32;r1:xmm;
  begin
    case inlinenumber of
      in_x86_movd_to_mem: begin op:=A_movd end;
      in_x86_movsd_to_mem: begin op:=A_movsd end;
      in_x86_movupd_to_mem: begin op:=A_movupd end;
      in_x86_movlpd_to_mem: begin op:=A_movlpd end;
      in_x86_movhpd_to_mem: begin op:=A_movhpd end;
      in_x86_movntpd_to_mem: begin op:=A_movntpd end;
      in_x86_movapd_to_mem: begin op:=A_movapd end;
      in_x86_movss_to_mem: begin op:=A_movss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_ref(op,S_NO,paraarray[2].location.register,paraarray[1].location.reference));
  end;
in_x86_movaps_to_mem
,in_x86_movups_to_mem
,in_x86_movdqa
,in_x86_movdqu
,in_x86_movntdq
: //r0:ptr128;r1:xmm;
  begin
    case inlinenumber of
      in_x86_movntdq: begin op:=A_movntdq end;
      in_x86_movdqu: begin op:=A_movdqu end;
      in_x86_movdqa: begin op:=A_movdqa end;
      in_x86_movups_to_mem: begin op:=A_movups end;
      in_x86_movaps_to_mem: begin op:=A_movaps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_ref(op,S_NO,paraarray[2].location.register,paraarray[1].location.reference));
  end;
in_x86_movss_to_val
: //out r0:f32;r1:xmm;
  begin
    case inlinenumber of
      in_x86_movss_to_val: begin op:=A_movss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_movss_from_val
: //out r0:xmm;r1:f32;
  begin
    case inlinenumber of
      in_x86_movss_from_val: begin op:=A_movss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_movlps
,in_x86_movhps
,in_x86_cvtpi2ps_from_mem
,in_x86_addsd_from_mem
,in_x86_divsd_from_mem
,in_x86_maxsd_from_mem
,in_x86_minsd_from_mem
,in_x86_mulsd_from_mem
,in_x86_subsd_from_mem
,in_x86_comisd_from_mem
,in_x86_ucomisd_from_mem
,in_x86_cvtdq2pd_from_mem
,in_x86_cvtpi2pd_from_mem
,in_x86_cvtps2pd_from_mem
,in_x86_cvtsd2ss_from_mem
: //var r0:xmm;r1:ptr64;
  begin
    case inlinenumber of
      in_x86_cvtsd2ss_from_mem: begin op:=A_cvtsd2ss end;
      in_x86_cvtps2pd_from_mem: begin op:=A_cvtps2pd end;
      in_x86_cvtpi2pd_from_mem: begin op:=A_cvtpi2pd end;
      in_x86_cvtdq2pd_from_mem: begin op:=A_cvtdq2pd end;
      in_x86_ucomisd_from_mem: begin op:=A_ucomisd end;
      in_x86_comisd_from_mem: begin op:=A_comisd end;
      in_x86_subsd_from_mem: begin op:=A_subsd end;
      in_x86_mulsd_from_mem: begin op:=A_mulsd end;
      in_x86_minsd_from_mem: begin op:=A_minsd end;
      in_x86_maxsd_from_mem: begin op:=A_maxsd end;
      in_x86_divsd_from_mem: begin op:=A_divsd end;
      in_x86_addsd_from_mem: begin op:=A_addsd end;
      in_x86_cvtpi2ps_from_mem: begin op:=A_cvtpi2ps end;
      in_x86_movhps: begin op:=A_movhps end;
      in_x86_movlps: begin op:=A_movlps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_movlps_to_mem
,in_x86_movhps_to_mem
,in_x86_movq_to_mem
: //r0:ptr64;r1:xmm;
  begin
    case inlinenumber of
      in_x86_movq_to_mem: begin op:=A_movq end;
      in_x86_movhps_to_mem: begin op:=A_movhps end;
      in_x86_movlps_to_mem: begin op:=A_movlps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_ref(op,S_NO,paraarray[2].location.register,paraarray[1].location.reference));
  end;
in_x86_movlhps
,in_x86_movhlps
,in_x86_addss
,in_x86_subss
,in_x86_mulss
,in_x86_divss
,in_x86_rcpss
,in_x86_sqrtss
,in_x86_maxss
,in_x86_minss
,in_x86_rsqrtss
,in_x86_addps
,in_x86_subps
,in_x86_mulps
,in_x86_divps
,in_x86_rcpps
,in_x86_sqrtps
,in_x86_maxps
,in_x86_minps
,in_x86_rsqrtps
,in_x86_andps
,in_x86_orps
,in_x86_xorps
,in_x86_andnps
,in_x86_unpckhps
,in_x86_unpcklps
,in_x86_addpd
,in_x86_addsd
,in_x86_divpd
,in_x86_divsd
,in_x86_maxpd
,in_x86_maxsd
,in_x86_minpd
,in_x86_minsd
,in_x86_mulpd
,in_x86_mulsd
,in_x86_subpd
,in_x86_subsd
,in_x86_andpd
,in_x86_andnpd
,in_x86_orpd
,in_x86_xorpd
,in_x86_comisd
,in_x86_ucomisd
,in_x86_unpckhpd
,in_x86_unpcklpd
,in_x86_cvtdq2pd
,in_x86_cvtdq2ps
,in_x86_cvtpd2dq
,in_x86_cvtpd2ps
,in_x86_cvtps2dq
,in_x86_cvtps2pd
,in_x86_cvtsd2ss
,in_x86_cvtss2sd
,in_x86_cvttpd2dq
,in_x86_cvttps2dq
,in_x86_packssdw
,in_x86_packsswb
,in_x86_packuswb
,in_x86_paddb
,in_x86_paddw
,in_x86_paddd
,in_x86_paddq
,in_x86_paddsb
,in_x86_paddsw
,in_x86_paddusb
,in_x86_paddusw
,in_x86_pand
,in_x86_pandn
,in_x86_por
,in_x86_pxor
,in_x86_pcmpeqb
,in_x86_pcmpeqw
,in_x86_pcmpeqd
,in_x86_pcmpgtb
,in_x86_pcmpgtw
,in_x86_pcmpgtd
,in_x86_pmullw
,in_x86_pmulhw
,in_x86_pmulhuw_sse2
,in_x86_pmuludq
,in_x86_psllw_sse2
,in_x86_pslld_sse2
,in_x86_psllq_sse2
,in_x86_psrad_sse2
,in_x86_psraw_sse2
,in_x86_psrlw_sse2
,in_x86_psrld_sse2
,in_x86_psrlq_sse2
,in_x86_psubb
,in_x86_psubw
,in_x86_psubd
,in_x86_psubq
,in_x86_psubsb
,in_x86_psubsw
,in_x86_pmaddwd
,in_x86_psubusb
,in_x86_psubusw
,in_x86_punpckhbw
,in_x86_punpckhwd
,in_x86_punpckhdq
,in_x86_punpcklbw
,in_x86_punpcklwd
,in_x86_punpckldq
,in_x86_pavgb_sse2
,in_x86_pavgw_sse2
,in_x86_pminub_sse2
,in_x86_pminsw_sse2
,in_x86_pmaxsw_sse2
,in_x86_pmaxub_sse2
,in_x86_psadbw_sse2
,in_x86_punpckhqdq
,in_x86_punpcklqdq
,in_x86_addsubps
,in_x86_addsubpd
,in_x86_haddps
,in_x86_haddpd
,in_x86_hsubps
,in_x86_hsubpd
,in_x86_psignb
,in_x86_psignw
,in_x86_psignd
,in_x86_pshufb
,in_x86_pmulhrsw
,in_x86_pmaddubsw
,in_x86_phsubw
,in_x86_phsubsw
,in_x86_phsubd
,in_x86_phaddsw
,in_x86_phaddw
,in_x86_phaddd
,in_x86_phminposuw
,in_x86_pmulld
,in_x86_pmuldq
,in_x86_pminsb
,in_x86_pminuw
,in_x86_pminsd
,in_x86_pminud
,in_x86_pmaxsb
,in_x86_pmaxuw
,in_x86_pmaxsd
,in_x86_pmaxud
,in_x86_ptest
,in_x86_pcmpeqq
,in_x86_packusdw
,in_x86_pcmpgtq
: //var r0:xmm;r1:xmm;
  begin
    case inlinenumber of
      in_x86_pcmpgtq: begin op:=A_pcmpgtq end;
      in_x86_packusdw: begin op:=A_packusdw end;
      in_x86_pcmpeqq: begin op:=A_pcmpeqq end;
      in_x86_ptest: begin op:=A_ptest end;
      in_x86_pmaxud: begin op:=A_pmaxud end;
      in_x86_pmaxsd: begin op:=A_pmaxsd end;
      in_x86_pmaxuw: begin op:=A_pmaxuw end;
      in_x86_pmaxsb: begin op:=A_pmaxsb end;
      in_x86_pminud: begin op:=A_pminud end;
      in_x86_pminsd: begin op:=A_pminsd end;
      in_x86_pminuw: begin op:=A_pminuw end;
      in_x86_pminsb: begin op:=A_pminsb end;
      in_x86_pmuldq: begin op:=A_pmuldq end;
      in_x86_pmulld: begin op:=A_pmulld end;
      in_x86_phminposuw: begin op:=A_phminposuw end;
      in_x86_phaddd: begin op:=A_phaddd end;
      in_x86_phaddw: begin op:=A_phaddw end;
      in_x86_phaddsw: begin op:=A_phaddsw end;
      in_x86_phsubd: begin op:=A_phsubd end;
      in_x86_phsubsw: begin op:=A_phsubsw end;
      in_x86_phsubw: begin op:=A_phsubw end;
      in_x86_pmaddubsw: begin op:=A_pmaddubsw end;
      in_x86_pmulhrsw: begin op:=A_pmulhrsw end;
      in_x86_pshufb: begin op:=A_pshufb end;
      in_x86_psignd: begin op:=A_psignd end;
      in_x86_psignw: begin op:=A_psignw end;
      in_x86_psignb: begin op:=A_psignb end;
      in_x86_hsubpd: begin op:=A_hsubpd end;
      in_x86_hsubps: begin op:=A_hsubps end;
      in_x86_haddpd: begin op:=A_haddpd end;
      in_x86_haddps: begin op:=A_haddps end;
      in_x86_addsubpd: begin op:=A_addsubpd end;
      in_x86_addsubps: begin op:=A_addsubps end;
      in_x86_punpcklqdq: begin op:=A_punpcklqdq end;
      in_x86_punpckhqdq: begin op:=A_punpckhqdq end;
      in_x86_psadbw_sse2: begin op:=A_psadbw end;
      in_x86_pmaxub_sse2: begin op:=A_pmaxub end;
      in_x86_pmaxsw_sse2: begin op:=A_pmaxsw end;
      in_x86_pminsw_sse2: begin op:=A_pminsw end;
      in_x86_pminub_sse2: begin op:=A_pminub end;
      in_x86_pavgw_sse2: begin op:=A_pavgw end;
      in_x86_pavgb_sse2: begin op:=A_pavgb end;
      in_x86_punpckldq: begin op:=A_punpckldq end;
      in_x86_punpcklwd: begin op:=A_punpcklwd end;
      in_x86_punpcklbw: begin op:=A_punpcklbw end;
      in_x86_punpckhdq: begin op:=A_punpckhdq end;
      in_x86_punpckhwd: begin op:=A_punpckhwd end;
      in_x86_punpckhbw: begin op:=A_punpckhbw end;
      in_x86_psubusw: begin op:=A_psubusw end;
      in_x86_psubusb: begin op:=A_psubusb end;
      in_x86_pmaddwd: begin op:=A_pmaddwd end;
      in_x86_psubsw: begin op:=A_psubsw end;
      in_x86_psubsb: begin op:=A_psubsb end;
      in_x86_psubq: begin op:=A_psubq end;
      in_x86_psubd: begin op:=A_psubd end;
      in_x86_psubw: begin op:=A_psubw end;
      in_x86_psubb: begin op:=A_psubb end;
      in_x86_psrlq_sse2: begin op:=A_psrlq end;
      in_x86_psrld_sse2: begin op:=A_psrld end;
      in_x86_psrlw_sse2: begin op:=A_psrlw end;
      in_x86_psraw_sse2: begin op:=A_psraw end;
      in_x86_psrad_sse2: begin op:=A_psrad end;
      in_x86_psllq_sse2: begin op:=A_psllq end;
      in_x86_pslld_sse2: begin op:=A_pslld end;
      in_x86_psllw_sse2: begin op:=A_psllw end;
      in_x86_pmuludq: begin op:=A_pmuludq end;
      in_x86_pmulhuw_sse2: begin op:=A_pmulhuw end;
      in_x86_pmulhw: begin op:=A_pmulhw end;
      in_x86_pmullw: begin op:=A_pmullw end;
      in_x86_pcmpgtd: begin op:=A_pcmpgtd end;
      in_x86_pcmpgtw: begin op:=A_pcmpgtw end;
      in_x86_pcmpgtb: begin op:=A_pcmpgtb end;
      in_x86_pcmpeqd: begin op:=A_pcmpeqd end;
      in_x86_pcmpeqw: begin op:=A_pcmpeqw end;
      in_x86_pcmpeqb: begin op:=A_pcmpeqb end;
      in_x86_pxor: begin op:=A_pxor end;
      in_x86_por: begin op:=A_por end;
      in_x86_pandn: begin op:=A_pandn end;
      in_x86_pand: begin op:=A_pand end;
      in_x86_paddusw: begin op:=A_paddusw end;
      in_x86_paddusb: begin op:=A_paddusb end;
      in_x86_paddsw: begin op:=A_paddsw end;
      in_x86_paddsb: begin op:=A_paddsb end;
      in_x86_paddq: begin op:=A_paddq end;
      in_x86_paddd: begin op:=A_paddd end;
      in_x86_paddw: begin op:=A_paddw end;
      in_x86_paddb: begin op:=A_paddb end;
      in_x86_packuswb: begin op:=A_packuswb end;
      in_x86_packsswb: begin op:=A_packsswb end;
      in_x86_packssdw: begin op:=A_packssdw end;
      in_x86_cvttps2dq: begin op:=A_cvttps2dq end;
      in_x86_cvttpd2dq: begin op:=A_cvttpd2dq end;
      in_x86_cvtss2sd: begin op:=A_cvtss2sd end;
      in_x86_cvtsd2ss: begin op:=A_cvtsd2ss end;
      in_x86_cvtps2pd: begin op:=A_cvtps2pd end;
      in_x86_cvtps2dq: begin op:=A_cvtps2dq end;
      in_x86_cvtpd2ps: begin op:=A_cvtpd2ps end;
      in_x86_cvtpd2dq: begin op:=A_cvtpd2dq end;
      in_x86_cvtdq2ps: begin op:=A_cvtdq2ps end;
      in_x86_cvtdq2pd: begin op:=A_cvtdq2pd end;
      in_x86_unpcklpd: begin op:=A_unpcklpd end;
      in_x86_unpckhpd: begin op:=A_unpckhpd end;
      in_x86_ucomisd: begin op:=A_ucomisd end;
      in_x86_comisd: begin op:=A_comisd end;
      in_x86_xorpd: begin op:=A_xorpd end;
      in_x86_orpd: begin op:=A_orpd end;
      in_x86_andnpd: begin op:=A_andnpd end;
      in_x86_andpd: begin op:=A_andpd end;
      in_x86_subsd: begin op:=A_subsd end;
      in_x86_subpd: begin op:=A_subpd end;
      in_x86_mulsd: begin op:=A_mulsd end;
      in_x86_mulpd: begin op:=A_mulpd end;
      in_x86_minsd: begin op:=A_minsd end;
      in_x86_minpd: begin op:=A_minpd end;
      in_x86_maxsd: begin op:=A_maxsd end;
      in_x86_maxpd: begin op:=A_maxpd end;
      in_x86_divsd: begin op:=A_divsd end;
      in_x86_divpd: begin op:=A_divpd end;
      in_x86_addsd: begin op:=A_addsd end;
      in_x86_addpd: begin op:=A_addpd end;
      in_x86_unpcklps: begin op:=A_unpcklps end;
      in_x86_unpckhps: begin op:=A_unpckhps end;
      in_x86_andnps: begin op:=A_andnps end;
      in_x86_xorps: begin op:=A_xorps end;
      in_x86_orps: begin op:=A_orps end;
      in_x86_andps: begin op:=A_andps end;
      in_x86_rsqrtps: begin op:=A_rsqrtps end;
      in_x86_minps: begin op:=A_minps end;
      in_x86_maxps: begin op:=A_maxps end;
      in_x86_sqrtps: begin op:=A_sqrtps end;
      in_x86_rcpps: begin op:=A_rcpps end;
      in_x86_divps: begin op:=A_divps end;
      in_x86_mulps: begin op:=A_mulps end;
      in_x86_subps: begin op:=A_subps end;
      in_x86_addps: begin op:=A_addps end;
      in_x86_rsqrtss: begin op:=A_rsqrtss end;
      in_x86_minss: begin op:=A_minss end;
      in_x86_maxss: begin op:=A_maxss end;
      in_x86_sqrtss: begin op:=A_sqrtss end;
      in_x86_rcpss: begin op:=A_rcpss end;
      in_x86_divss: begin op:=A_divss end;
      in_x86_mulss: begin op:=A_mulss end;
      in_x86_subss: begin op:=A_subss end;
      in_x86_addss: begin op:=A_addss end;
      in_x86_movhlps: begin op:=A_movhlps end;
      in_x86_movlhps: begin op:=A_movlhps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_addss_from_mem
,in_x86_subss_from_mem
,in_x86_mulss_from_mem
,in_x86_divss_from_mem
,in_x86_rcpss_from_mem
,in_x86_sqrtss_from_mem
,in_x86_maxss_from_mem
,in_x86_minss_from_mem
,in_x86_rsqrtss_from_mem
,in_x86_cvtsi2ss_from_mem
,in_x86_cvtsi2sd_from_mem
,in_x86_cvtss2sd_from_mem
: //var r0:xmm;r1:ptr32;
  begin
    case inlinenumber of
      in_x86_cvtss2sd_from_mem: begin op:=A_cvtss2sd end;
      in_x86_cvtsi2sd_from_mem: begin op:=A_cvtsi2sd end;
      in_x86_cvtsi2ss_from_mem: begin op:=A_cvtsi2ss end;
      in_x86_rsqrtss_from_mem: begin op:=A_rsqrtss end;
      in_x86_minss_from_mem: begin op:=A_minss end;
      in_x86_maxss_from_mem: begin op:=A_maxss end;
      in_x86_sqrtss_from_mem: begin op:=A_sqrtss end;
      in_x86_rcpss_from_mem: begin op:=A_rcpss end;
      in_x86_divss_from_mem: begin op:=A_divss end;
      in_x86_mulss_from_mem: begin op:=A_mulss end;
      in_x86_subss_from_mem: begin op:=A_subss end;
      in_x86_addss_from_mem: begin op:=A_addss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_addps_from_mem
,in_x86_subps_from_mem
,in_x86_mulps_from_mem
,in_x86_divps_from_mem
,in_x86_rcpps_from_mem
,in_x86_sqrtps_from_mem
,in_x86_maxps_from_mem
,in_x86_minps_from_mem
,in_x86_rsqrtps_from_mem
,in_x86_andps_from_mem
,in_x86_orps_from_mem
,in_x86_xorps_from_mem
,in_x86_andnps_from_mem
,in_x86_unpckhps_from_mem
,in_x86_unpcklps_from_mem
,in_x86_addpd_from_mem
,in_x86_divpd_from_mem
,in_x86_maxpd_from_mem
,in_x86_minpd_from_mem
,in_x86_mulpd_from_mem
,in_x86_subpd_from_mem
,in_x86_andpd_from_mem
,in_x86_andnpd_from_mem
,in_x86_orpd_from_mem
,in_x86_xorpd_from_mem
,in_x86_unpckhpd_from_mem
,in_x86_unpcklpd_from_mem
,in_x86_cvtdq2ps_from_mem
,in_x86_cvtpd2dq_from_mem
,in_x86_cvtpd2ps_from_mem
,in_x86_cvtps2dq_from_mem
,in_x86_cvttpd2dq_from_mem
,in_x86_cvttps2dq_from_mem
,in_x86_packssdw_from_mem
,in_x86_packsswb_from_mem
,in_x86_packuswb_from_mem
,in_x86_paddb_from_mem
,in_x86_paddw_from_mem
,in_x86_paddd_from_mem
,in_x86_paddq_from_mem
,in_x86_paddsb_from_mem
,in_x86_paddsw_from_mem
,in_x86_paddusb_from_mem
,in_x86_paddusw_from_mem
,in_x86_pand_from_mem
,in_x86_pandn_from_mem
,in_x86_por_from_mem
,in_x86_pxor_from_mem
,in_x86_pcmpeqb_from_mem
,in_x86_pcmpeqw_from_mem
,in_x86_pcmpeqd_from_mem
,in_x86_pcmpgtb_from_mem
,in_x86_pcmpgtw_from_mem
,in_x86_pcmpgtd_from_mem
,in_x86_pmullw_from_mem
,in_x86_pmulhw_from_mem
,in_x86_pmulhuw_from_mem
,in_x86_pmuludq_from_mem
,in_x86_psllw_from_mem
,in_x86_pslld_from_mem
,in_x86_psllq_from_mem
,in_x86_psrad_from_mem
,in_x86_psraw_from_mem
,in_x86_psrlw_from_mem
,in_x86_psrld_from_mem
,in_x86_psrlq_from_mem
,in_x86_psubb_from_mem
,in_x86_psubw_from_mem
,in_x86_psubd_from_mem
,in_x86_psubq_from_mem
,in_x86_psubsb_from_mem
,in_x86_psubsw_from_mem
,in_x86_pmaddwd_from_mem
,in_x86_psubusb_from_mem
,in_x86_psubusw_from_mem
,in_x86_punpckhbw_from_mem
,in_x86_punpckhwd_from_mem
,in_x86_punpckhdq_from_mem
,in_x86_punpcklbw_from_mem
,in_x86_punpcklwd_from_mem
,in_x86_punpckldq_from_mem
,in_x86_pavgb_from_mem
,in_x86_pavgw_from_mem
,in_x86_pminub_from_mem
,in_x86_pminsw_from_mem
,in_x86_pmaxsw_from_mem
,in_x86_pmaxub_from_mem
,in_x86_psadbw_from_mem
,in_x86_punpckhqdq_from_mem
,in_x86_punpcklqdq_from_mem
,in_x86_addsubps_from_mem
,in_x86_addsubpd_from_mem
,in_x86_haddps_from_mem
,in_x86_haddpd_from_mem
,in_x86_hsubps_from_mem
,in_x86_hsubpd_from_mem
,in_x86_psignb_from_mem
,in_x86_psignw_from_mem
,in_x86_psignd_from_mem
,in_x86_pshufb_from_mem
,in_x86_pmulhrsw_from_mem
,in_x86_pmaddubsw_from_mem
,in_x86_phsubw_from_mem
,in_x86_phsubsw_from_mem
,in_x86_phsubd_from_mem
,in_x86_phaddsw_from_mem
,in_x86_phaddw_from_mem
,in_x86_phaddd_from_mem
,in_x86_phminposuw_from_mem
,in_x86_pmulld_from_mem
,in_x86_pmuldq_from_mem
,in_x86_pminsb_from_mem
,in_x86_pminuw_from_mem
,in_x86_pminsd_from_mem
,in_x86_pminud_from_mem
,in_x86_pmaxsb_from_mem
,in_x86_pmaxuw_from_mem
,in_x86_pmaxsd_from_mem
,in_x86_pmaxud_from_mem
,in_x86_ptest_from_mem
,in_x86_pcmpeqq_from_mem
,in_x86_packusdw_from_mem
,in_x86_pcmpgtq_from_mem
: //var r0:xmm;r1:ptr128;
  begin
    case inlinenumber of
      in_x86_pcmpgtq_from_mem: begin op:=A_pcmpgtq end;
      in_x86_packusdw_from_mem: begin op:=A_packusdw end;
      in_x86_pcmpeqq_from_mem: begin op:=A_pcmpeqq end;
      in_x86_ptest_from_mem: begin op:=A_ptest end;
      in_x86_pmaxud_from_mem: begin op:=A_pmaxud end;
      in_x86_pmaxsd_from_mem: begin op:=A_pmaxsd end;
      in_x86_pmaxuw_from_mem: begin op:=A_pmaxuw end;
      in_x86_pmaxsb_from_mem: begin op:=A_pmaxsb end;
      in_x86_pminud_from_mem: begin op:=A_pminud end;
      in_x86_pminsd_from_mem: begin op:=A_pminsd end;
      in_x86_pminuw_from_mem: begin op:=A_pminuw end;
      in_x86_pminsb_from_mem: begin op:=A_pminsb end;
      in_x86_pmuldq_from_mem: begin op:=A_pmuldq end;
      in_x86_pmulld_from_mem: begin op:=A_pmulld end;
      in_x86_phminposuw_from_mem: begin op:=A_phminposuw end;
      in_x86_phaddd_from_mem: begin op:=A_phaddd end;
      in_x86_phaddw_from_mem: begin op:=A_phaddw end;
      in_x86_phaddsw_from_mem: begin op:=A_phaddsw end;
      in_x86_phsubd_from_mem: begin op:=A_phsubd end;
      in_x86_phsubsw_from_mem: begin op:=A_phsubsw end;
      in_x86_phsubw_from_mem: begin op:=A_phsubw end;
      in_x86_pmaddubsw_from_mem: begin op:=A_pmaddubsw end;
      in_x86_pmulhrsw_from_mem: begin op:=A_pmulhrsw end;
      in_x86_pshufb_from_mem: begin op:=A_pshufb end;
      in_x86_psignd_from_mem: begin op:=A_psignd end;
      in_x86_psignw_from_mem: begin op:=A_psignw end;
      in_x86_psignb_from_mem: begin op:=A_psignb end;
      in_x86_hsubpd_from_mem: begin op:=A_hsubpd end;
      in_x86_hsubps_from_mem: begin op:=A_hsubps end;
      in_x86_haddpd_from_mem: begin op:=A_haddpd end;
      in_x86_haddps_from_mem: begin op:=A_haddps end;
      in_x86_addsubpd_from_mem: begin op:=A_addsubpd end;
      in_x86_addsubps_from_mem: begin op:=A_addsubps end;
      in_x86_punpcklqdq_from_mem: begin op:=A_punpcklqdq end;
      in_x86_punpckhqdq_from_mem: begin op:=A_punpckhqdq end;
      in_x86_psadbw_from_mem: begin op:=A_psadbw end;
      in_x86_pmaxub_from_mem: begin op:=A_pmaxub end;
      in_x86_pmaxsw_from_mem: begin op:=A_pmaxsw end;
      in_x86_pminsw_from_mem: begin op:=A_pminsw end;
      in_x86_pminub_from_mem: begin op:=A_pminub end;
      in_x86_pavgw_from_mem: begin op:=A_pavgw end;
      in_x86_pavgb_from_mem: begin op:=A_pavgb end;
      in_x86_punpckldq_from_mem: begin op:=A_punpckldq end;
      in_x86_punpcklwd_from_mem: begin op:=A_punpcklwd end;
      in_x86_punpcklbw_from_mem: begin op:=A_punpcklbw end;
      in_x86_punpckhdq_from_mem: begin op:=A_punpckhdq end;
      in_x86_punpckhwd_from_mem: begin op:=A_punpckhwd end;
      in_x86_punpckhbw_from_mem: begin op:=A_punpckhbw end;
      in_x86_psubusw_from_mem: begin op:=A_psubusw end;
      in_x86_psubusb_from_mem: begin op:=A_psubusb end;
      in_x86_pmaddwd_from_mem: begin op:=A_pmaddwd end;
      in_x86_psubsw_from_mem: begin op:=A_psubsw end;
      in_x86_psubsb_from_mem: begin op:=A_psubsb end;
      in_x86_psubq_from_mem: begin op:=A_psubq end;
      in_x86_psubd_from_mem: begin op:=A_psubd end;
      in_x86_psubw_from_mem: begin op:=A_psubw end;
      in_x86_psubb_from_mem: begin op:=A_psubb end;
      in_x86_psrlq_from_mem: begin op:=A_psrlq end;
      in_x86_psrld_from_mem: begin op:=A_psrld end;
      in_x86_psrlw_from_mem: begin op:=A_psrlw end;
      in_x86_psraw_from_mem: begin op:=A_psraw end;
      in_x86_psrad_from_mem: begin op:=A_psrad end;
      in_x86_psllq_from_mem: begin op:=A_psllq end;
      in_x86_pslld_from_mem: begin op:=A_pslld end;
      in_x86_psllw_from_mem: begin op:=A_psllw end;
      in_x86_pmuludq_from_mem: begin op:=A_pmuludq end;
      in_x86_pmulhuw_from_mem: begin op:=A_pmulhuw end;
      in_x86_pmulhw_from_mem: begin op:=A_pmulhw end;
      in_x86_pmullw_from_mem: begin op:=A_pmullw end;
      in_x86_pcmpgtd_from_mem: begin op:=A_pcmpgtd end;
      in_x86_pcmpgtw_from_mem: begin op:=A_pcmpgtw end;
      in_x86_pcmpgtb_from_mem: begin op:=A_pcmpgtb end;
      in_x86_pcmpeqd_from_mem: begin op:=A_pcmpeqd end;
      in_x86_pcmpeqw_from_mem: begin op:=A_pcmpeqw end;
      in_x86_pcmpeqb_from_mem: begin op:=A_pcmpeqb end;
      in_x86_pxor_from_mem: begin op:=A_pxor end;
      in_x86_por_from_mem: begin op:=A_por end;
      in_x86_pandn_from_mem: begin op:=A_pandn end;
      in_x86_pand_from_mem: begin op:=A_pand end;
      in_x86_paddusw_from_mem: begin op:=A_paddusw end;
      in_x86_paddusb_from_mem: begin op:=A_paddusb end;
      in_x86_paddsw_from_mem: begin op:=A_paddsw end;
      in_x86_paddsb_from_mem: begin op:=A_paddsb end;
      in_x86_paddq_from_mem: begin op:=A_paddq end;
      in_x86_paddd_from_mem: begin op:=A_paddd end;
      in_x86_paddw_from_mem: begin op:=A_paddw end;
      in_x86_paddb_from_mem: begin op:=A_paddb end;
      in_x86_packuswb_from_mem: begin op:=A_packuswb end;
      in_x86_packsswb_from_mem: begin op:=A_packsswb end;
      in_x86_packssdw_from_mem: begin op:=A_packssdw end;
      in_x86_cvttps2dq_from_mem: begin op:=A_cvttps2dq end;
      in_x86_cvttpd2dq_from_mem: begin op:=A_cvttpd2dq end;
      in_x86_cvtps2dq_from_mem: begin op:=A_cvtps2dq end;
      in_x86_cvtpd2ps_from_mem: begin op:=A_cvtpd2ps end;
      in_x86_cvtpd2dq_from_mem: begin op:=A_cvtpd2dq end;
      in_x86_cvtdq2ps_from_mem: begin op:=A_cvtdq2ps end;
      in_x86_unpcklpd_from_mem: begin op:=A_unpcklpd end;
      in_x86_unpckhpd_from_mem: begin op:=A_unpckhpd end;
      in_x86_xorpd_from_mem: begin op:=A_xorpd end;
      in_x86_orpd_from_mem: begin op:=A_orpd end;
      in_x86_andnpd_from_mem: begin op:=A_andnpd end;
      in_x86_andpd_from_mem: begin op:=A_andpd end;
      in_x86_subpd_from_mem: begin op:=A_subpd end;
      in_x86_mulpd_from_mem: begin op:=A_mulpd end;
      in_x86_minpd_from_mem: begin op:=A_minpd end;
      in_x86_maxpd_from_mem: begin op:=A_maxpd end;
      in_x86_divpd_from_mem: begin op:=A_divpd end;
      in_x86_addpd_from_mem: begin op:=A_addpd end;
      in_x86_unpcklps_from_mem: begin op:=A_unpcklps end;
      in_x86_unpckhps_from_mem: begin op:=A_unpckhps end;
      in_x86_andnps_from_mem: begin op:=A_andnps end;
      in_x86_xorps_from_mem: begin op:=A_xorps end;
      in_x86_orps_from_mem: begin op:=A_orps end;
      in_x86_andps_from_mem: begin op:=A_andps end;
      in_x86_rsqrtps_from_mem: begin op:=A_rsqrtps end;
      in_x86_minps_from_mem: begin op:=A_minps end;
      in_x86_maxps_from_mem: begin op:=A_maxps end;
      in_x86_sqrtps_from_mem: begin op:=A_sqrtps end;
      in_x86_rcpps_from_mem: begin op:=A_rcpps end;
      in_x86_divps_from_mem: begin op:=A_divps end;
      in_x86_mulps_from_mem: begin op:=A_mulps end;
      in_x86_subps_from_mem: begin op:=A_subps end;
      in_x86_addps_from_mem: begin op:=A_addps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_cmpss
,in_x86_cmpps
,in_x86_shufps
,in_x86_cmppd
,in_x86_cmpsd
,in_x86_shufpd
,in_x86_palignr
,in_x86_dpps
,in_x86_dppd
,in_x86_blendps
,in_x86_blendpd
,in_x86_insertps
,in_x86_mpsadbw
,in_x86_pblendw
,in_x86_pcmpestri
,in_x86_pcmpestrm
,in_x86_pcmpistri
,in_x86_pcmpistrm
: //var r0:xmm;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pcmpistrm: begin op:=A_pcmpistrm end;
      in_x86_pcmpistri: begin op:=A_pcmpistri end;
      in_x86_pcmpestrm: begin op:=A_pcmpestrm end;
      in_x86_pcmpestri: begin op:=A_pcmpestri end;
      in_x86_pblendw: begin op:=A_pblendw end;
      in_x86_mpsadbw: begin op:=A_mpsadbw end;
      in_x86_insertps: begin op:=A_insertps end;
      in_x86_blendpd: begin op:=A_blendpd end;
      in_x86_blendps: begin op:=A_blendps end;
      in_x86_dppd: begin op:=A_dppd end;
      in_x86_dpps: begin op:=A_dpps end;
      in_x86_palignr: begin op:=A_palignr end;
      in_x86_shufpd: begin op:=A_shufpd end;
      in_x86_cmpsd: begin op:=A_cmpsd end;
      in_x86_cmppd: begin op:=A_cmppd end;
      in_x86_shufps: begin op:=A_shufps end;
      in_x86_cmpps: begin op:=A_cmpps end;
      in_x86_cmpss: begin op:=A_cmpss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_cmpss_from_mem
,in_x86_insertps_from_mem
,in_x86_pinsrd_from_mem
: //var r0:xmm;r1:ptr32;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrd_from_mem: begin op:=A_pinsrd end;
      in_x86_insertps_from_mem: begin op:=A_insertps end;
      in_x86_cmpss_from_mem: begin op:=A_cmpss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_cmpps_from_mem
,in_x86_shufps_from_mem
,in_x86_cmppd_from_mem
,in_x86_shufpd_from_mem
,in_x86_palignr_from_mem
,in_x86_dpps_from_mem
,in_x86_dppd_from_mem
,in_x86_blendps_from_mem
,in_x86_blendpd_from_mem
,in_x86_mpsadbw_from_mem
,in_x86_pblendw_from_mem
,in_x86_pcmpestri_from_mem
,in_x86_pcmpestrm_from_mem
,in_x86_pcmpistri_from_mem
,in_x86_pcmpistrm_from_mem
: //var r0:xmm;r1:ptr128;imm:i32;
  begin
    case inlinenumber of
      in_x86_pcmpistrm_from_mem: begin op:=A_pcmpistrm end;
      in_x86_pcmpistri_from_mem: begin op:=A_pcmpistri end;
      in_x86_pcmpestrm_from_mem: begin op:=A_pcmpestrm end;
      in_x86_pcmpestri_from_mem: begin op:=A_pcmpestri end;
      in_x86_pblendw_from_mem: begin op:=A_pblendw end;
      in_x86_mpsadbw_from_mem: begin op:=A_mpsadbw end;
      in_x86_blendpd_from_mem: begin op:=A_blendpd end;
      in_x86_blendps_from_mem: begin op:=A_blendps end;
      in_x86_dppd_from_mem: begin op:=A_dppd end;
      in_x86_dpps_from_mem: begin op:=A_dpps end;
      in_x86_palignr_from_mem: begin op:=A_palignr end;
      in_x86_shufpd_from_mem: begin op:=A_shufpd end;
      in_x86_cmppd_from_mem: begin op:=A_cmppd end;
      in_x86_shufps_from_mem: begin op:=A_shufps end;
      in_x86_cmpps_from_mem: begin op:=A_cmpps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_cvtsi2ss
: //var r0:xmm;r1:reg;
  begin
    case inlinenumber of
      in_x86_cvtsi2ss: begin op:=A_cvtsi2ss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[2].location, paraarray[2].resultdef,uinttype,true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_cvtss2si
,in_x86_cvttss2si
: //out r0:reg;r1:xmm;
  begin
    case inlinenumber of
      in_x86_cvttss2si: begin op:=A_cvttss2si end;
      in_x86_cvtss2si: begin op:=A_cvtss2si; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_INT);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_cvtss2si_from_mem
,in_x86_cvttss2si_from_mem
: //out r0:reg;r1:ptr32;
  begin
    case inlinenumber of
      in_x86_cvttss2si_from_mem: begin op:=A_cvttss2si end;
      in_x86_cvtss2si_from_mem: begin op:=A_cvtss2si; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_REGISTER,OS_INT);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[1].location.reference,location.register));
  end;
in_x86_cvtpi2ps
,in_x86_cvtpi2pd
: //var r0:xmm;r1:mm;
  begin
    case inlinenumber of
      in_x86_cvtpi2pd: begin op:=A_cvtpi2pd end;
      in_x86_cvtpi2ps: begin op:=A_cvtpi2ps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_cvtps2pi
,in_x86_cvttps2pi
,in_x86_movdq2q
: //out r0:mm;r1:xmm;
  begin
    case inlinenumber of
      in_x86_movdq2q: begin op:=A_movdq2q end;
      in_x86_cvttps2pi: begin op:=A_cvttps2pi end;
      in_x86_cvtps2pi: begin op:=A_cvtps2pi; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMXREGISTER,OS_M64);
    location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_cvtps2pi_from_mem
,in_x86_cvttps2pi_from_mem
: //out r0:mm;r1:ptr64;
  begin
    case inlinenumber of
      in_x86_cvttps2pi_from_mem: begin op:=A_cvttps2pi end;
      in_x86_cvtps2pi_from_mem: begin op:=A_cvtps2pi; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMXREGISTER,OS_M64);
    location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[1].location.reference,location.register));
  end;
in_x86_pmulhuw_mmx
,in_x86_psadbw_mmx
,in_x86_pavgb_mmx
,in_x86_pavgw_mmx
,in_x86_pmaxub_mmx
,in_x86_pminub_mmx
,in_x86_pmaxsw_mmx
,in_x86_pminsw_mmx
: //var r0:mm;r1:mm;
  begin
    case inlinenumber of
      in_x86_pminsw_mmx: begin op:=A_pminsw end;
      in_x86_pmaxsw_mmx: begin op:=A_pmaxsw end;
      in_x86_pminub_mmx: begin op:=A_pminub end;
      in_x86_pmaxub_mmx: begin op:=A_pmaxub end;
      in_x86_pavgw_mmx: begin op:=A_pavgw end;
      in_x86_pavgb_mmx: begin op:=A_pavgb end;
      in_x86_psadbw_mmx: begin op:=A_psadbw end;
      in_x86_pmulhuw_mmx: begin op:=A_pmulhuw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_pmulhuw_mmx_from_mem
,in_x86_psadbw_mmx_from_mem
,in_x86_pavgb_mmx_from_mem
,in_x86_pavgw_mmx_from_mem
,in_x86_pmaxub_mmx_from_mem
,in_x86_pminub_mmx_from_mem
,in_x86_pmaxsw_mmx_from_mem
,in_x86_pminsw_mmx_from_mem
: //var r0:mm;r1:ptr64;
  begin
    case inlinenumber of
      in_x86_pminsw_mmx_from_mem: begin op:=A_pminsw end;
      in_x86_pmaxsw_mmx_from_mem: begin op:=A_pmaxsw end;
      in_x86_pminub_mmx_from_mem: begin op:=A_pminub end;
      in_x86_pmaxub_mmx_from_mem: begin op:=A_pmaxub end;
      in_x86_pavgw_mmx_from_mem: begin op:=A_pavgw end;
      in_x86_pavgb_mmx_from_mem: begin op:=A_pavgb end;
      in_x86_psadbw_mmx_from_mem: begin op:=A_psadbw end;
      in_x86_pmulhuw_mmx_from_mem: begin op:=A_pmulhuw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_pextrw_mmx
: //out r0:reg;r1:mm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrw_mmx: begin op:=A_pextrw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_INT);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register,location.register));
  end;
in_x86_pinsrw_mmx
: //var r0:mm;r1:reg;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrw_mmx: begin op:=A_pinsrw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[2].location, paraarray[2].resultdef,uinttype,true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_pmovmskb_mmx
: //out r0:reg;r1:mm;
  begin
    case inlinenumber of
      in_x86_pmovmskb_mmx: begin op:=A_pmovmskb; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_INT);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_INT);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_pshufw
: //out r0:mm;r1:mm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pshufw: begin op:=A_pshufw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMXREGISTER,OS_M64);
    location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register,location.register));
  end;
in_x86_pshufw_from_mem
: //out r0:mm;r1:ptr64;imm:i32;
  begin
    case inlinenumber of
      in_x86_pshufw_from_mem: begin op:=A_pshufw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMXREGISTER,OS_M64);
    location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.reference,location.register));
  end;
in_x86_movmskpd
,in_x86_movd_to_reg
: //out r0:r32;r1:xmm;
  begin
    case inlinenumber of
      in_x86_movd_to_reg: begin op:=A_movd end;
      in_x86_movmskpd: begin op:=A_movmskpd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_32);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_movsd_to_val
: //out r0:f64;r1:xmm;
  begin
    case inlinenumber of
      in_x86_movsd_to_val: begin op:=A_movsd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_movsd_from_val
: //out r0:xmm;r1:f64;
  begin
    case inlinenumber of
      in_x86_movsd_from_val: begin op:=A_movsd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_sqrtpd
,in_x86_sqrtsd
,in_x86_movddup
,in_x86_movsldup
,in_x86_movshdup
,in_x86_pabsb
,in_x86_pabsw
,in_x86_pabsd
,in_x86_pmovsxbw
,in_x86_pmovzxbw
,in_x86_pmovsxbd
,in_x86_pmovzxbd
,in_x86_pmovsxbq
,in_x86_pmovzxbq
,in_x86_pmovsxwd
,in_x86_pmovzxwd
,in_x86_pmovsxwq
,in_x86_pmovzxwq
,in_x86_pmovsxdq
,in_x86_pmovzxdq
: //out r0:xmm;r1:xmm;
  begin
    case inlinenumber of
      in_x86_pmovzxdq: begin op:=A_pmovzxdq end;
      in_x86_pmovsxdq: begin op:=A_pmovsxdq end;
      in_x86_pmovzxwq: begin op:=A_pmovzxwq end;
      in_x86_pmovsxwq: begin op:=A_pmovsxwq end;
      in_x86_pmovzxwd: begin op:=A_pmovzxwd end;
      in_x86_pmovsxwd: begin op:=A_pmovsxwd end;
      in_x86_pmovzxbq: begin op:=A_pmovzxbq end;
      in_x86_pmovsxbq: begin op:=A_pmovsxbq end;
      in_x86_pmovzxbd: begin op:=A_pmovzxbd end;
      in_x86_pmovsxbd: begin op:=A_pmovsxbd end;
      in_x86_pmovzxbw: begin op:=A_pmovzxbw end;
      in_x86_pmovsxbw: begin op:=A_pmovsxbw end;
      in_x86_pabsd: begin op:=A_pabsd end;
      in_x86_pabsw: begin op:=A_pabsw end;
      in_x86_pabsb: begin op:=A_pabsb end;
      in_x86_movshdup: begin op:=A_movshdup end;
      in_x86_movsldup: begin op:=A_movsldup end;
      in_x86_movddup: begin op:=A_movddup end;
      in_x86_sqrtsd: begin op:=A_sqrtsd end;
      in_x86_sqrtpd: begin op:=A_sqrtpd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_sqrtsd_from_mem
,in_x86_movq_from_mem
,in_x86_movddup_from_mem
,in_x86_pmovsxbw_from_mem
,in_x86_pmovzxbw_from_mem
,in_x86_pmovsxwd_from_mem
,in_x86_pmovzxwd_from_mem
,in_x86_pmovsxdq_from_mem
,in_x86_pmovzxdq_from_mem
: //out r0:xmm;r1:ptr64;
  begin
    case inlinenumber of
      in_x86_pmovzxdq_from_mem: begin op:=A_pmovzxdq end;
      in_x86_pmovsxdq_from_mem: begin op:=A_pmovsxdq end;
      in_x86_pmovzxwd_from_mem: begin op:=A_pmovzxwd end;
      in_x86_pmovsxwd_from_mem: begin op:=A_pmovsxwd end;
      in_x86_pmovzxbw_from_mem: begin op:=A_pmovzxbw end;
      in_x86_pmovsxbw_from_mem: begin op:=A_pmovsxbw end;
      in_x86_movddup_from_mem: begin op:=A_movddup end;
      in_x86_movq_from_mem: begin op:=A_movq end;
      in_x86_sqrtsd_from_mem: begin op:=A_sqrtsd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[1].location.reference,location.register));
  end;
in_x86_cmpsd_from_mem
: //var r0:xmm;r1:ptr64;imm:i32;
  begin
    case inlinenumber of
      in_x86_cmpsd_from_mem: begin op:=A_cmpsd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_cvtpd2pi
,in_x86_cvttpd2pi
: //var r0:mm;r1:xmm;
  begin
    case inlinenumber of
      in_x86_cvttpd2pi: begin op:=A_cvttpd2pi end;
      in_x86_cvtpd2pi: begin op:=A_cvtpd2pi; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_cvtpd2pi_from_mem
,in_x86_cvttpd2pi_from_mem
: //var r0:mm;r1:ptr128;
  begin
    case inlinenumber of
      in_x86_cvttpd2pi_from_mem: begin op:=A_cvttpd2pi end;
      in_x86_cvtpd2pi_from_mem: begin op:=A_cvtpd2pi; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_cvtsd2si
,in_x86_cvttsd2si
: //var r0:sreg;r1:xmm;
  begin
    case inlinenumber of
      in_x86_cvttsd2si: begin op:=A_cvttsd2si end;
      in_x86_cvtsd2si: begin op:=A_cvtsd2si; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[1].location, paraarray[1].resultdef,sinttype,false);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_cvtsd2si_from_mem
,in_x86_cvttsd2si_from_mem
: //var r0:sreg;r1:ptr64;
  begin
    case inlinenumber of
      in_x86_cvttsd2si_from_mem: begin op:=A_cvttsd2si end;
      in_x86_cvtsd2si_from_mem: begin op:=A_cvtsd2si; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[1].location, paraarray[1].resultdef,sinttype,false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_cvtsi2sd
: //var r0:xmm;r1:r32;
  begin
    case inlinenumber of
      in_x86_cvtsi2sd: begin op:=A_cvtsi2sd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[2].location, paraarray[2].resultdef,u32inttype,true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_movd_from_reg
: //out r0:xmm;r1:r32;
  begin
    case inlinenumber of
      in_x86_movd_from_reg: begin op:=A_movd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[1].location, paraarray[1].resultdef,u32inttype,true);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_pmovmskb
: //var r0:r32;r1:xmm;
  begin
    case inlinenumber of
      in_x86_pmovmskb: begin op:=A_pmovmskb; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[1].location, paraarray[1].resultdef,u32inttype,false);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_pextrw_sse2
: //out r0:r16;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrw_sse2: begin op:=A_pextrw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_16);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_16);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register,location.register));
  end;
in_x86_pinsrw_sse2
,in_x86_pinsrb
,in_x86_pinsrd
: //var r0:xmm;r1:r32;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrd: begin op:=A_pinsrd end;
      in_x86_pinsrb: begin op:=A_pinsrb end;
      in_x86_pinsrw_sse2: begin op:=A_pinsrw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[2].location, paraarray[2].resultdef,u32inttype,true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_pinsrw_from_mem
: //var r0:xmm;r1:ptr16;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrw_from_mem: begin op:=A_pinsrw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.reference,paraarray[1].location.register));
  end;
in_x86_psllw_sse2_imm
,in_x86_pslld_sse2_imm
,in_x86_psllq_sse2_imm
,in_x86_psrad_sse2_imm
,in_x86_psraw_sse2_imm
,in_x86_psrlw_sse2_imm
,in_x86_psrld_sse2_imm
,in_x86_psrlq_sse2_imm
,in_x86_pslldq
,in_x86_psrldq
: //var r0:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_psrldq: begin op:=A_psrldq end;
      in_x86_pslldq: begin op:=A_pslldq end;
      in_x86_psrlq_sse2_imm: begin op:=A_psrlq end;
      in_x86_psrld_sse2_imm: begin op:=A_psrld end;
      in_x86_psrlw_sse2_imm: begin op:=A_psrlw end;
      in_x86_psraw_sse2_imm: begin op:=A_psraw end;
      in_x86_psrad_sse2_imm: begin op:=A_psrad end;
      in_x86_psllq_sse2_imm: begin op:=A_psllq end;
      in_x86_pslld_sse2_imm: begin op:=A_pslld end;
      in_x86_psllw_sse2_imm: begin op:=A_psllw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register));
  end;
in_x86_maskmovdqu
: //addr:edi_ptr;r0:xmm;r1:xmm;
  begin
    case inlinenumber of
      in_x86_maskmovdqu: begin op:=A_maskmovdqu; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    hlcg.getcpuregister(current_asmdata.CurrAsmList,{$if defined(cpu64bitalu)}NR_RDI{$else}NR_EDI{$endif});
    hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,paraarray[1].resultdef,voidpointertype,paraarray[1].location,{$if defined(cpu64bitalu)}NR_RDI{$else}NR_EDI{$endif});
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[3].location, true);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[3].location.register,paraarray[2].location.register));
    hlcg.ungetcpuregister(current_asmdata.CurrAsmList,{$if defined(cpu64bitalu)}NR_RDI{$else}NR_EDI{$endif});
  end;
in_x86_movq2dq
: //out r0:xmm;r1:mm;
  begin
    case inlinenumber of
      in_x86_movq2dq: begin op:=A_movq2dq; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_pshufhw
,in_x86_pshuflw
,in_x86_pshufd
,in_x86_roundps
,in_x86_roundss
,in_x86_roundpd
,in_x86_roundsd
: //out r0:xmm;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_roundsd: begin op:=A_roundsd end;
      in_x86_roundpd: begin op:=A_roundpd end;
      in_x86_roundss: begin op:=A_roundss end;
      in_x86_roundps: begin op:=A_roundps end;
      in_x86_pshufd: begin op:=A_pshufd end;
      in_x86_pshuflw: begin op:=A_pshuflw end;
      in_x86_pshufhw: begin op:=A_pshufhw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register,location.register));
  end;
in_x86_pshufhw_from_mem
,in_x86_pshuflw_from_mem
,in_x86_pshufd_from_mem
,in_x86_roundps_from_mem
,in_x86_roundpd_from_mem
: //out r0:xmm;r1:ptr128;imm:i32;
  begin
    case inlinenumber of
      in_x86_roundpd_from_mem: begin op:=A_roundpd end;
      in_x86_roundps_from_mem: begin op:=A_roundps end;
      in_x86_pshufd_from_mem: begin op:=A_pshufd end;
      in_x86_pshuflw_from_mem: begin op:=A_pshuflw end;
      in_x86_pshufhw_from_mem: begin op:=A_pshufhw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.reference,location.register));
  end;
in_x86_blendvps
,in_x86_blendvpd
,in_x86_pblendvb
: //var r0:xmm;r1:xmm;mask:implicit_xmm0;
  begin
    case inlinenumber of
      in_x86_pblendvb: begin op:=A_pblendvb end;
      in_x86_blendvpd: begin op:=A_blendvpd end;
      in_x86_blendvps: begin op:=A_blendvps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[3].location, true);
    hlcg.getcpuregister(current_asmdata.CurrAsmList,NR_XMM0);
    hlcg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,paraarray[3].resultdef,x86_m128type,paraarray[3].location,NR_XMM0,nil);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[2].location.register,paraarray[1].location.register));
    hlcg.ungetcpuregister(current_asmdata.CurrAsmList,NR_XMM0);
  end;
in_x86_blendvps_from_mem
,in_x86_blendvpd_from_mem
,in_x86_pblendvb_from_mem
: //var r0:xmm;r1:ptr128;mask:implicit_xmm0;
  begin
    case inlinenumber of
      in_x86_pblendvb_from_mem: begin op:=A_pblendvb end;
      in_x86_blendvpd_from_mem: begin op:=A_blendvpd end;
      in_x86_blendvps_from_mem: begin op:=A_blendvps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[3].location, true);
    hlcg.getcpuregister(current_asmdata.CurrAsmList,NR_XMM0);
    hlcg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,paraarray[3].resultdef,x86_m128type,paraarray[3].location,NR_XMM0,nil);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[2].location.reference,paraarray[1].location.register));
    hlcg.ungetcpuregister(current_asmdata.CurrAsmList,NR_XMM0);
  end;
in_x86_roundss_from_mem
: //out r0:xmm;r1:ptr32;imm:i32;
  begin
    case inlinenumber of
      in_x86_roundss_from_mem: begin op:=A_roundss; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.reference,location.register));
  end;
in_x86_roundsd_from_mem
: //out r0:xmm;r1:ptr64;imm:i32;
  begin
    case inlinenumber of
      in_x86_roundsd_from_mem: begin op:=A_roundsd; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.reference,location.register));
  end;
in_x86_extractps
,in_x86_pextrd
: //out r0:r32;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrd: begin op:=A_pextrd end;
      in_x86_extractps: begin op:=A_extractps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_32);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register,location.register));
  end;
in_x86_extractps_from_mem
,in_x86_pextrd_to_mem
: //r0:ptr32;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrd_to_mem: begin op:=A_pextrd end;
      in_x86_extractps_from_mem: begin op:=A_extractps; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_ref(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.reference));
  end;
in_x86_pinsrb_from_mem
: //var r0:xmm;r1:ptr8;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrb_from_mem: begin op:=A_pinsrb; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.reference,paraarray[1].location.register));
  end;
{$ifdef X86_64}
in_x86_pinsrq
: //var r0:xmm;r1:reg;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrq: begin op:=A_pinsrq; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[2].location, paraarray[2].resultdef,uinttype,true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.register));
  end;
{$endif}
{$ifdef X86_64}
in_x86_pinsrq_from_mem
: //var r0:xmm;r1:ptr64;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrq_from_mem: begin op:=A_pinsrq; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    location_make_ref(paraarray[2].location);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_ref_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.reference,paraarray[1].location.register));
  end;
{$endif}
in_x86_pextrb
: //out r0:r8;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrb: begin op:=A_pextrb; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_8);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_8);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register,location.register));
  end;
in_x86_pextrb_to_mem
: //r0:ptr8;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrb_to_mem: begin op:=A_pextrb; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_ref(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.reference));
  end;
in_x86_pextrw_sse41_to_mem
: //r0:ptr16;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrw_sse41_to_mem: begin op:=A_pextrw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_ref(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.reference));
  end;
{$ifdef X86_64}
in_x86_pextrq
: //out r0:r32;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrq: begin op:=A_pextrq; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_32);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register,location.register));
  end;
{$endif}
{$ifdef X86_64}
in_x86_pextrq_to_mem
: //r0:ptr64;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrq_to_mem: begin op:=A_pextrq; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_force_mmreg(current_asmdata.CurrAsmList, paraarray[2].location, true);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_ref(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.reference));
  end;
{$endif}
in_x86_pmovsxbq_from_mem
,in_x86_pmovzxbq_from_mem
: //out r0:xmm;r1:ptr16;
  begin
    case inlinenumber of
      in_x86_pmovzxbq_from_mem: begin op:=A_pmovzxbq end;
      in_x86_pmovsxbq_from_mem: begin op:=A_pmovsxbq; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_make_ref(paraarray[1].location);
    location_reset(location,LOC_MMREGISTER,OS_M128);
    location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);
    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,S_NO,paraarray[1].location.reference,location.register));
  end;
