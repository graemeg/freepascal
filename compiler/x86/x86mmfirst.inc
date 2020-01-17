in_x86_movss
,in_x86_movaps
,in_x86_movups
: //out r0:xmm;r1:ptr32;
  begin
    expectloc:=LOC_MMREGISTER;
    result:=nil;
  end;
in_x86_movss_to_mem
,in_x86_movaps_to_mem
,in_x86_movups_to_mem
: //r0:ptr32;r1:xmm;
  begin
    expectloc:=LOC_VOID;
    result:=nil;
  end;
in_x86_movss_to_val
: //out r0:f32;r1:xmm;
  begin
    expectloc:=LOC_MMREGISTER;
    result:=nil;
  end;
in_x86_movss_from_val
: //out r0:xmm;r1:f32;
  begin
    expectloc:=LOC_MMREGISTER;
    result:=nil;
  end;
in_x86_movlps
,in_x86_movhps
: //var r0:xmm;r1:ptr32;
  begin
    expectloc:=LOC_MMREGISTER;
    result:=nil;
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
    expectloc:=LOC_MMREGISTER;
    result:=nil;
  end;
in_x86_cmpss
,in_x86_cmpps
,in_x86_shufps
: //var r0:xmm;r1:xmm;imm:i32;
  begin
    expectloc:=LOC_MMREGISTER;
    result:=nil;
  end;
in_x86_cvtsi2ss
: //var r0:xmm;r1:r32;
  begin
    expectloc:=LOC_MMREGISTER;
    result:=nil;
  end;
in_x86_cvtss2si
,in_x86_cvttss2si
: //out r0:r32;r1:xmm;
  begin
    expectloc:=LOC_REGISTER;
    result:=nil;
  end;
in_x86_cvtpi2ps
: //var r0:xmm;r1:mm;
  begin
    expectloc:=LOC_MMREGISTER;
    result:=nil;
  end;
in_x86_cvtps2pi
,in_x86_cvttps2pi
: //out r0:mm;r1:xmm;
  begin
    expectloc:=LOC_MMXREGISTER;
    result:=nil;
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
    expectloc:=LOC_MMXREGISTER;
    result:=nil;
  end;
in_x86_pextrw_mmx
: //out r0:r32;r1:mm;imm:i32;
  begin
    expectloc:=LOC_REGISTER;
    result:=nil;
  end;
in_x86_pinsrw_mmx
: //var r0:mm;r1:r32;imm:i32;
  begin
    expectloc:=LOC_MMXREGISTER;
    result:=nil;
  end;
in_x86_pmovmskb
: //out r0:r32;r1:mm;
  begin
    expectloc:=LOC_REGISTER;
    result:=nil;
  end;
in_x86_pshufw
: //out r0:mm;r1:mm;imm:i32;
  begin
    expectloc:=LOC_MMXREGISTER;
    result:=nil;
  end;
in_x86_pextrw
: //out r0:r32;r1:xmm;imm:i32;
  begin
    expectloc:=LOC_REGISTER;
    result:=nil;
  end;
in_x86_pinsrw
: //var r0:xmm;r1:r32;imm:i32;
  begin
    expectloc:=LOC_MMREGISTER;
    result:=nil;
  end;
