in_x86_movss
,in_x86_movaps
,in_x86_movups
: //out r0:xmm;r1:ptr32;
  begin
    case inlinenumber of
      in_x86_movups: begin op:=A_movups end;
      in_x86_movaps: begin op:=A_movaps end;
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
in_x86_movss_to_mem
,in_x86_movaps_to_mem
,in_x86_movups_to_mem
: //r0:ptr32;r1:xmm;
  begin
    case inlinenumber of
      in_x86_movups_to_mem: begin op:=A_movups end;
      in_x86_movaps_to_mem: begin op:=A_movaps end;
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
: //var r0:xmm;r1:ptr32;
  begin
    case inlinenumber of
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
,in_x86_pmulhuw
,in_x86_psadbw
,in_x86_pavgb
,in_x86_pavgw
,in_x86_pmaxub
,in_x86_pminub
,in_x86_pmaxsw
,in_x86_pminsw
: //var r0:xmm;r1:xmm;
  begin
    case inlinenumber of
      in_x86_pminsw: begin op:=A_pminsw end;
      in_x86_pmaxsw: begin op:=A_pmaxsw end;
      in_x86_pminub: begin op:=A_pminub end;
      in_x86_pmaxub: begin op:=A_pmaxub end;
      in_x86_pavgw: begin op:=A_pavgw end;
      in_x86_pavgb: begin op:=A_pavgb end;
      in_x86_psadbw: begin op:=A_psadbw end;
      in_x86_pmulhuw: begin op:=A_pmulhuw end;
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
in_x86_cmpss
,in_x86_cmpps
,in_x86_shufps
: //var r0:xmm;r1:xmm;imm:i32;
  begin
    case inlinenumber of
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
in_x86_cvtsi2ss
: //var r0:xmm;r1:r32;
  begin
    case inlinenumber of
      in_x86_cvtsi2ss: begin op:=A_cvtsi2ss; end;
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
in_x86_cvtss2si
,in_x86_cvttss2si
: //out r0:r32;r1:xmm;
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
    location_reset(location,LOC_REGISTER,OS_32);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);
    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,S_NO,paraarray[1].location.register,location.register));
  end;
in_x86_cvtpi2ps
: //var r0:xmm;r1:mm;
  begin
    case inlinenumber of
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
: //out r0:mm;r1:xmm;
  begin
    case inlinenumber of
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
in_x86_pextrw_mmx
: //out r0:r32;r1:mm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrw_mmx: begin op:=A_pextrw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(2);

    for i := 1 to 2 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_32);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[2]),paraarray[1].location.register,location.register));
  end;
in_x86_pinsrw_mmx
: //var r0:mm;r1:r32;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrw_mmx: begin op:=A_pinsrw; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(3);

    for i := 1 to 3 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, false);
    hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[2].location, paraarray[2].resultdef,u32inttype,true);
    location:=paraarray[1].location;
    current_asmdata.CurrAsmList.concat(taicpu.op_const_reg_reg(op,S_NO,GetConstInt(paraarray[3]),paraarray[2].location.register,paraarray[1].location.register));
  end;
in_x86_pmovmskb
: //out r0:r32;r1:mm;
  begin
    case inlinenumber of
      in_x86_pmovmskb: begin op:=A_pmovmskb; end;
      else
        Internalerror(2020010201);
    end;

    GetParameters(1);

    for i := 1 to 1 do secondpass(paraarray[i]);

    location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[1].location, true);
    location_reset(location,LOC_REGISTER,OS_32);
    location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);
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
in_x86_pextrw
: //out r0:r32;r1:xmm;imm:i32;
  begin
    case inlinenumber of
      in_x86_pextrw: begin op:=A_pextrw; end;
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
in_x86_pinsrw
: //var r0:xmm;r1:r32;imm:i32;
  begin
    case inlinenumber of
      in_x86_pinsrw: begin op:=A_pinsrw; end;
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
