{ don't edit, this file is generated from x86ins.dat }
(
  (
    opcode  : A_NONE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #0;
    flags   : if_none
  ),
  (
    opcode  : A_AAA;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#55;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_AAD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#213#10;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_AAD;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #1#213#20;
    flags   : if_8086 or if_sb or if_nox86_64
  ),
  (
    opcode  : A_AAM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#212#10;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_AAM;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #1#212#20;
    flags   : if_8086 or if_sb or if_nox86_64
  ),
  (
    opcode  : A_AAS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#63;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#17#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#19#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#16#65;
    flags   : if_8086
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#18#72;
    flags   : if_8086
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#131#130#13;
    flags   : if_8086
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #213#1#21#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #213#1#129#130#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#21#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#129#130#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#20#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#128#130#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#1#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#3#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#0#65;
    flags   : if_8086
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#2#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#131#128#13;
    flags   : if_8086
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #208#1#5#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #213#1#129#128#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#5#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#129#128#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#4#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#128#128#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#33#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#35#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#32#65;
    flags   : if_8086
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#34#72;
    flags   : if_8086
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#131#132#13;
    flags   : if_8086
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #213#1#37#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #213#1#129#132#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#37#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#129#132#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#36#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#128#132#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_ARPL;
    ops     : 2;
    optypes : (ot_reg16,ot_reg16,ot_none,ot_none);
    code    : #1#99#65;
    flags   : if_286 or if_prot or if_nox86_64
  ),
  (
    opcode  : A_ARPL;
    ops     : 2;
    optypes : (ot_memory,ot_reg16,ot_none,ot_none);
    code    : #1#99#65;
    flags   : if_286 or if_prot or if_sm or if_nox86_64
  ),
  (
    opcode  : A_BOUND;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32,ot_memory,ot_none,ot_none);
    code    : #208#1#98#72;
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_BSF;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#2#15#188#72;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_BSR;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#2#15#189#72;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_BSWAP;
    ops     : 1;
    optypes : (ot_reg32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#15#8#200;
    flags   : if_486
  ),
  (
    opcode  : A_BT;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#163#65;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_BT;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#2#15#186#132#21;
    flags   : if_386 or if_sb
  ),
  (
    opcode  : A_BTC;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#187#65;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_BTC;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#2#15#186#135#21;
    flags   : if_386 or if_sb
  ),
  (
    opcode  : A_BTR;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#179#65;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_BTR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#2#15#186#134#21;
    flags   : if_386 or if_sb
  ),
  (
    opcode  : A_BTS;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#171#65;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_BTS;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#2#15#186#133#21;
    flags   : if_386 or if_sb
  ),
  (
    opcode  : A_CALL;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits32,ot_none,ot_none,ot_none);
    code    : #213#1#255#130;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_CALL;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#255#130;
    flags   : if_8086
  ),
  (
    opcode  : A_CALL;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #208#1#232#52;
    flags   : if_8086
  ),
  (
    opcode  : A_CALL;
    ops     : 1;
    optypes : (ot_immediate or ot_near,ot_none,ot_none,ot_none);
    code    : #208#1#232#52;
    flags   : if_8086
  ),
  (
    opcode  : A_CALL;
    ops     : 1;
    optypes : (ot_immediate or ot_far,ot_none,ot_none,ot_none);
    code    : #208#1#154#28#31;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_CALL;
    ops     : 1;
    optypes : (ot_memory or ot_near,ot_none,ot_none,ot_none);
    code    : #208#1#255#130;
    flags   : if_8086
  ),
  (
    opcode  : A_CALL;
    ops     : 1;
    optypes : (ot_memory or ot_far,ot_none,ot_none,ot_none);
    code    : #208#1#255#131;
    flags   : if_8086
  ),
  (
    opcode  : A_CALL;
    ops     : 2;
    optypes : (ot_immediate,ot_immediate,ot_none,ot_none);
    code    : #215#1#154#29#24;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_CALL;
    ops     : 2;
    optypes : (ot_immediate or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#154#25#24;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_CALL;
    ops     : 2;
    optypes : (ot_immediate,ot_immediate or ot_bits16,ot_none,ot_none);
    code    : #212#1#154#25#24;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_CALL;
    ops     : 2;
    optypes : (ot_immediate or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #213#1#154#33#24;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_CALL;
    ops     : 2;
    optypes : (ot_immediate,ot_immediate or ot_bits32,ot_none,ot_none);
    code    : #213#1#154#33#24;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_CBW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#152;
    flags   : if_8086
  ),
  (
    opcode  : A_CDQ;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#153;
    flags   : if_386
  ),
  (
    opcode  : A_CLC;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#248;
    flags   : if_8086
  ),
  (
    opcode  : A_CLD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#252;
    flags   : if_8086
  ),
  (
    opcode  : A_CLI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#250;
    flags   : if_8086
  ),
  (
    opcode  : A_CLTS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#6;
    flags   : if_286 or if_priv
  ),
  (
    opcode  : A_CMC;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#245;
    flags   : if_8086
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#57#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#59#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#56#65;
    flags   : if_8086
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#58#72;
    flags   : if_8086
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#131#135#13;
    flags   : if_8086
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #213#1#61#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #213#1#129#135#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#61#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#129#135#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#60#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#128#135#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_memory,ot_immediate or ot_bits32,ot_none,ot_none);
    code    : #213#1#129#135#33;
    flags   : if_386 or if_sd
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_memory,ot_immediate or ot_bits16,ot_none,ot_none);
    code    : #212#1#129#135#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_memory,ot_immediate or ot_bits8,ot_none,ot_none);
    code    : #1#128#135#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_CMPSB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #218#1#166;
    flags   : if_8086
  ),
  (
    opcode  : A_CMPSD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #218#213#1#167;
    flags   : if_386
  ),
  (
    opcode  : A_CMPSD;
    ops     : 3;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #220#2#15#194#72#22;
    flags   : if_willamette or if_sse2 or if_sm2 or if_sb or if_ar2
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#242#217#2#15#194#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
  ),
  (
    opcode  : A_CMPSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#242#193#217#2#15#194#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_CMPSW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #218#212#1#167;
    flags   : if_8086
  ),
  (
    opcode  : A_CMPXCHG;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#177#65;
    flags   : if_pent or if_sm
  ),
  (
    opcode  : A_CMPXCHG;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #2#15#176#65;
    flags   : if_pent
  ),
  (
    opcode  : A_CMPXCHG486;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#167#65;
    flags   : if_486 or if_sm
  ),
  (
    opcode  : A_CMPXCHG486;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #2#15#166#65;
    flags   : if_486 or if_undoc
  ),
  (
    opcode  : A_CMPXCHG8B;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#199#129;
    flags   : if_pent
  ),
  (
    opcode  : A_CPUID;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#162;
    flags   : if_pent
  ),
  (
    opcode  : A_CWD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#153;
    flags   : if_8086
  ),
  (
    opcode  : A_CWDE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#152;
    flags   : if_386
  ),
  (
    opcode  : A_DAA;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#39;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_DAS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#47;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_DEC;
    ops     : 1;
    optypes : (ot_reg16 or ot_bits32,ot_none,ot_none,ot_none);
    code    : #208#8#72;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_DEC;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#255#129;
    flags   : if_8086
  ),
  (
    opcode  : A_DEC;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#254#129;
    flags   : if_8086
  ),
  (
    opcode  : A_DIV;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#247#134;
    flags   : if_8086
  ),
  (
    opcode  : A_DIV;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#246#134;
    flags   : if_8086
  ),
  (
    opcode  : A_EMMS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#119;
    flags   : if_pent or if_mmx
  ),
  (
    opcode  : A_ENTER;
    ops     : 2;
    optypes : (ot_immediate,ot_immediate,ot_none,ot_none);
    code    : #1#200#24#21;
    flags   : if_186
  ),
  (
    opcode  : A_F2XM1;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#240;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FABS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#225;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADD;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#216#128;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADD;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#220#128;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#222#193;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADD;
    ops     : 1;
    optypes : (ot_fpureg or ot_to,ot_none,ot_none,ot_none);
    code    : #1#220#8#192;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADD;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#220#8#192;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADD;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#216#8#192;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADD;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#216#9#192;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADDP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#222#193;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADDP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#222#8#192;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FADDP;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#222#8#192;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FBLD;
    ops     : 1;
    optypes : (ot_memory or ot_bits80,ot_none,ot_none,ot_none);
    code    : #1#223#132;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FBLD;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#223#132;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FBSTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits80,ot_none,ot_none,ot_none);
    code    : #1#223#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FBSTP;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#223#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCHS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCLEX;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#155#219#226;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCMOVB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#218#193;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVB;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#218#8#192;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVB;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#218#9#192;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVBE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#218#209;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVBE;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#218#8#208;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVBE;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#218#9#208;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#218#201;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVE;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#218#8#200;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVE;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#218#9#200;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#193;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNB;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#219#8#192;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNB;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#219#9#192;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNBE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#209;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNBE;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#219#8#208;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNBE;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#219#9#208;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#201;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNE;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#219#8#200;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNE;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#219#9#200;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNU;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#217;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNU;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#219#8#216;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVNU;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#219#9#216;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVU;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#218#217;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVU;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#218#8#216;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCMOVU;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#218#9#216;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCOM;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#216#130;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOM;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#220#130;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#216#209;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOM;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#216#8#208;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOM;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#216#9#208;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOMI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#241;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCOMI;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#219#8#240;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCOMI;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#219#9#240;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCOMIP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#223#241;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCOMIP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#223#8#240;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCOMIP;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#223#9#240;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FCOMP;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#216#131;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOMP;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#220#131;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOMP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#216#217;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOMP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#216#8#216;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOMP;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#216#9#216;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOMPP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#222#217;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FCOS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#255;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FDECSTP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#246;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDISI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#155#219#225;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIV;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#216#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIV;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#220#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIV;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#220#241;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIV;
    ops     : 1;
    optypes : (ot_fpureg or ot_to,ot_none,ot_none,ot_none);
    code    : #1#220#8#240;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIV;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#220#8#240;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIV;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#216#8#240;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIV;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#216#9#240;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#222#241;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVP;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#222#8#240;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#222#8#240;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVR;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#216#135;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVR;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#220#135;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVR;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#220#249;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVR;
    ops     : 1;
    optypes : (ot_fpureg or ot_to,ot_none,ot_none,ot_none);
    code    : #1#220#8#248;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVR;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#220#8#248;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVR;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#216#8#248;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVR;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#216#9#248;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVRP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#222#249;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVRP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#222#8#248;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FDIVRP;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#222#8#248;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FEMMS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#14;
    flags   : if_pent or if_3dnow
  ),
  (
    opcode  : A_FENI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#155#219#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FFREE;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#221#8#192;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIADD;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#222#128;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIADD;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#218#128;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FICOM;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#222#130;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FICOM;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#218#130;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FICOMP;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#222#131;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FICOMP;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#218#131;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIDIV;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#222#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIDIV;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#218#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIDIVR;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#222#135;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIDIVR;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#218#135;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FILD;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#219#128;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FILD;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#223#128;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FILD;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#223#133;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIMUL;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#222#129;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIMUL;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#218#129;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FINCSTP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#247;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FINIT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#155#219#227;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIST;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#219#130;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FIST;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#223#130;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FISTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#219#131;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FISTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#223#131;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FISTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#223#135;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FISTTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#219#129;
    flags   : if_prescott or if_fpu
  ),
  (
    opcode  : A_FISTTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#223#129;
    flags   : if_prescott or if_fpu
  ),
  (
    opcode  : A_FISTTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#221#129;
    flags   : if_prescott or if_fpu
  ),
  (
    opcode  : A_FISUB;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#222#132;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FISUB;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#218#132;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FISUBR;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #1#222#133;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FISUBR;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#218#133;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLD;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#217#128;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLD;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#221#128;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLD;
    ops     : 1;
    optypes : (ot_memory or ot_bits80,ot_none,ot_none,ot_none);
    code    : #1#219#133;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLD;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#217#8#192;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLD1;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#232;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLDCW;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#217#133;
    flags   : if_8086 or if_fpu or if_sw
  ),
  (
    opcode  : A_FLDENV;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#217#132;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLDL2E;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#234;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLDL2T;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#233;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLDLG2;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#236;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLDLN2;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#237;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLDPI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#235;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FLDZ;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#238;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMUL;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#216#129;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMUL;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#220#129;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMUL;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#220#201;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMUL;
    ops     : 1;
    optypes : (ot_fpureg or ot_to,ot_none,ot_none,ot_none);
    code    : #1#220#8#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMUL;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#220#8#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMUL;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#216#8#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMUL;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#216#9#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMULP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#222#201;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMULP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#222#8#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FMULP;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#222#8#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FNCLEX;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#226;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FNDISI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#225;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FNENI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FNINIT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#227;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FNOP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#208;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FNSAVE;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#221#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FNSTCW;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#217#135;
    flags   : if_8086 or if_fpu or if_sw
  ),
  (
    opcode  : A_FNSTENV;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#217#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FNSTSW;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#221#135;
    flags   : if_8086 or if_fpu or if_sw
  ),
  (
    opcode  : A_FNSTSW;
    ops     : 1;
    optypes : (ot_reg_ax,ot_none,ot_none,ot_none);
    code    : #2#223#224;
    flags   : if_286 or if_fpu
  ),
  (
    opcode  : A_FPATAN;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#243;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FPREM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#248;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FPREM1;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#245;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FPTAN;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#242;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FRNDINT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#252;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FRSTOR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #1#221#132;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSAVE;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#155#221#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSCALE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#253;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSETPM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#228;
    flags   : if_286 or if_fpu
  ),
  (
    opcode  : A_FSIN;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#254;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FSINCOS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#251;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FSQRT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#250;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FST;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#217#130;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FST;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#221#130;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FST;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#221#8#208;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSTCW;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#155#217#135;
    flags   : if_8086 or if_fpu or if_sw
  ),
  (
    opcode  : A_FSTENV;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#155#217#134;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#217#131;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#221#131;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSTP;
    ops     : 1;
    optypes : (ot_memory or ot_bits80,ot_none,ot_none,ot_none);
    code    : #1#219#135;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSTP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#221#8#216;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSTSW;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#155#221#135;
    flags   : if_8086 or if_fpu or if_sw
  ),
  (
    opcode  : A_FSTSW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#155#223#224;
    flags   : if_286 or if_fpu
  ),
  (
    opcode  : A_FSTSW;
    ops     : 1;
    optypes : (ot_reg_ax,ot_none,ot_none,ot_none);
    code    : #3#155#223#224;
    flags   : if_286 or if_fpu
  ),
  (
    opcode  : A_FSUB;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#216#132;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUB;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#220#132;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#220#225;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUB;
    ops     : 1;
    optypes : (ot_fpureg or ot_to,ot_none,ot_none,ot_none);
    code    : #1#220#8#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUB;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#220#8#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUB;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#216#8#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUB;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#216#9#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#222#225;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#222#8#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBP;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#222#8#224;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBR;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #1#216#133;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBR;
    ops     : 1;
    optypes : (ot_memory or ot_bits64,ot_none,ot_none,ot_none);
    code    : #1#220#133;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBR;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#220#233;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBR;
    ops     : 1;
    optypes : (ot_fpureg or ot_to,ot_none,ot_none,ot_none);
    code    : #1#220#8#232;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBR;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#220#8#232;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBR;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#216#8#232;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBR;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#216#9#232;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBRP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#222#233;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBRP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#222#8#232;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FSUBRP;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#222#8#232;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FTST;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#228;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FUCOM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#221#225;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FUCOM;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#221#8#224;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FUCOM;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#221#9#224;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FUCOMI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#219#233;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FUCOMI;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#219#8#232;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FUCOMI;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#219#9#232;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FUCOMIP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#223#233;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FUCOMIP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#223#8#232;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FUCOMIP;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#223#9#232;
    flags   : if_p6 or if_fpu
  ),
  (
    opcode  : A_FUCOMP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#221#233;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FUCOMP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#221#8#232;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FUCOMP;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#221#9#232;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FUCOMPP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#218#233;
    flags   : if_386 or if_fpu
  ),
  (
    opcode  : A_FWAIT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#155;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FXAM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#229;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FXCH;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#201;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FXCH;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#217#8#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FXCH;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpu0,ot_none,ot_none);
    code    : #1#217#8#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FXCH;
    ops     : 2;
    optypes : (ot_fpu0,ot_fpureg,ot_none,ot_none);
    code    : #1#217#9#200;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FXTRACT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#244;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FYL2X;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#241;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_FYL2XP1;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#217#249;
    flags   : if_8086 or if_fpu
  ),
  (
    opcode  : A_HLT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#244;
    flags   : if_8086 or if_priv
  ),
  (
    opcode  : A_IBTS;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#167#65;
    flags   : if_386 or if_sm or if_undoc
  ),
  (
    opcode  : A_ICEBP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#241;
    flags   : if_386
  ),
  (
    opcode  : A_IDIV;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#247#135;
    flags   : if_8086
  ),
  (
    opcode  : A_IDIV;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#246#135;
    flags   : if_8086
  ),
  (
    opcode  : A_IMUL;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#2#15#175#72;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_IMUL;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#247#133;
    flags   : if_8086
  ),
  (
    opcode  : A_IMUL;
    ops     : 3;
    optypes : (ot_reg32 or ot_bits64,ot_rm_gpr,ot_immediate or ot_bits8 or ot_signed,ot_none);
    code    : #208#1#107#72#14;
    flags   : if_286 or if_sm
  ),
  (
    opcode  : A_IMUL;
    ops     : 3;
    optypes : (ot_reg32 or ot_bits64,ot_rm_gpr,ot_immediate,ot_none);
    code    : #208#1#105#72#34;
    flags   : if_286 or if_sm or if_sd or if_ar2
  ),
  (
    opcode  : A_IMUL;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#107#64#13;
    flags   : if_286
  ),
  (
    opcode  : A_IMUL;
    ops     : 2;
    optypes : (ot_reg32,ot_immediate,ot_none,ot_none);
    code    : #213#1#105#64#33;
    flags   : if_286 or if_sd
  ),
  (
    opcode  : A_IMUL;
    ops     : 3;
    optypes : (ot_reg16,ot_rm_gpr,ot_immediate or ot_bits8 or ot_signed,ot_none);
    code    : #212#1#107#72#14;
    flags   : if_286 or if_sm
  ),
  (
    opcode  : A_IMUL;
    ops     : 3;
    optypes : (ot_reg16,ot_rm_gpr,ot_immediate,ot_none);
    code    : #212#1#105#72#26;
    flags   : if_286 or if_sm or if_sw or if_ar2
  ),
  (
    opcode  : A_IMUL;
    ops     : 2;
    optypes : (ot_reg16,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #212#1#107#64#13;
    flags   : if_286
  ),
  (
    opcode  : A_IMUL;
    ops     : 2;
    optypes : (ot_reg16,ot_immediate,ot_none,ot_none);
    code    : #212#1#105#64#25;
    flags   : if_286 or if_sw
  ),
  (
    opcode  : A_IMUL;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#246#133;
    flags   : if_8086
  ),
  (
    opcode  : A_IN;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#228#21;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_IN;
    ops     : 2;
    optypes : (ot_reg_ax or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #208#1#229#21;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_IN;
    ops     : 2;
    optypes : (ot_reg_al,ot_reg_dx,ot_none,ot_none);
    code    : #1#236;
    flags   : if_8086
  ),
  (
    opcode  : A_IN;
    ops     : 2;
    optypes : (ot_reg_ax or ot_bits32,ot_reg_dx,ot_none,ot_none);
    code    : #208#1#237;
    flags   : if_8086
  ),
  (
    opcode  : A_INC;
    ops     : 1;
    optypes : (ot_reg16 or ot_bits32,ot_none,ot_none,ot_none);
    code    : #208#8#64;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_INC;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#255#128;
    flags   : if_8086
  ),
  (
    opcode  : A_INC;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#254#128;
    flags   : if_8086
  ),
  (
    opcode  : A_INSB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#108;
    flags   : if_186
  ),
  (
    opcode  : A_INSD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#109;
    flags   : if_386
  ),
  (
    opcode  : A_INSW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#109;
    flags   : if_186
  ),
  (
    opcode  : A_INT;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #1#205#20;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_INT01;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#241;
    flags   : if_386
  ),
  (
    opcode  : A_INT1;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#241;
    flags   : if_386
  ),
  (
    opcode  : A_INT03;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#204;
    flags   : if_8086
  ),
  (
    opcode  : A_INT3;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#204;
    flags   : if_8086
  ),
  (
    opcode  : A_INTO;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#206;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_INVD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#8;
    flags   : if_486 or if_priv
  ),
  (
    opcode  : A_INVLPG;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#1#135;
    flags   : if_486 or if_priv
  ),
  (
    opcode  : A_IRET;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #215#1#207;
    flags   : if_8086
  ),
  (
    opcode  : A_IRETD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#207;
    flags   : if_386
  ),
  (
    opcode  : A_IRETW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#207;
    flags   : if_8086
  ),
  (
    opcode  : A_JCXZ;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #200#1#227#40;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_JECXZ;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #201#1#227#40;
    flags   : if_386
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_immediate or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#235#40;
    flags   : if_8086 or if_pass2
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_immediate or ot_bits16 or ot_bits32,ot_none,ot_none,ot_none);
    code    : #208#1#233#52;
    flags   : if_8086 or if_pass2
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits32,ot_none,ot_none,ot_none);
    code    : #213#1#255#132;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#255#132;
    flags   : if_8086
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_immediate or ot_short,ot_none,ot_none,ot_none);
    code    : #1#235#40;
    flags   : if_8086 or if_pass2
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_immediate or ot_near,ot_none,ot_none,ot_none);
    code    : #208#1#233#52;
    flags   : if_8086 or if_pass2
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_immediate or ot_far,ot_none,ot_none,ot_none);
    code    : #208#1#234#28#31;
    flags   : if_8086 or if_pass2 or if_nox86_64
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_memory or ot_near,ot_none,ot_none,ot_none);
    code    : #208#1#255#132;
    flags   : if_8086
  ),
  (
    opcode  : A_JMP;
    ops     : 1;
    optypes : (ot_memory or ot_far,ot_none,ot_none,ot_none);
    code    : #208#1#255#133;
    flags   : if_8086
  ),
  (
    opcode  : A_JMP;
    ops     : 2;
    optypes : (ot_immediate,ot_immediate,ot_none,ot_none);
    code    : #215#1#234#29#24;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_JMP;
    ops     : 2;
    optypes : (ot_immediate,ot_immediate or ot_bits16,ot_none,ot_none);
    code    : #212#1#234#25#24;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_JMP;
    ops     : 2;
    optypes : (ot_immediate,ot_immediate or ot_bits32,ot_none,ot_none);
    code    : #213#1#234#33#24;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_LAHF;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#159;
    flags   : if_8086
  ),
  (
    opcode  : A_LAR;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#2#15#2#72;
    flags   : if_286 or if_prot or if_sm
  ),
  (
    opcode  : A_LCALL;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #213#1#255#131;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_LCALL;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#255#131;
    flags   : if_8086
  ),
  (
    opcode  : A_LDS;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32,ot_memory,ot_none,ot_none);
    code    : #208#1#197#72;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_LEA;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_memory,ot_none,ot_none);
    code    : #193#208#1#141#72;
    flags   : if_8086
  ),
  (
    opcode  : A_LEAVE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#201;
    flags   : if_186
  ),
  (
    opcode  : A_LES;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32,ot_memory,ot_none,ot_none);
    code    : #208#1#196#72;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_LFS;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32,ot_memory,ot_none,ot_none);
    code    : #208#2#15#180#72;
    flags   : if_386
  ),
  (
    opcode  : A_LGDT;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#1#130;
    flags   : if_286 or if_priv
  ),
  (
    opcode  : A_LGS;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32,ot_memory,ot_none,ot_none);
    code    : #208#2#15#181#72;
    flags   : if_386
  ),
  (
    opcode  : A_LIDT;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#1#131;
    flags   : if_286 or if_priv
  ),
  (
    opcode  : A_LJMP;
    ops     : 1;
<<<<<<< HEAD
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #213#1#255#133;
    flags   : if_386 or if_nox86_64
=======
    optypes : (ot_regmem or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #192#208#1#255#133;
    flags   : if_8086
  ),
  (
    opcode  : A_LJMP;
    ops     : 1;
    optypes : (ot_memory or ot_far,ot_none,ot_none);
    code    : #192#208#1#255#133;
    flags   : if_8086
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_LJMP;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#255#133;
    flags   : if_8086
  ),
  (
    opcode  : A_LLDT;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16,ot_none,ot_none,ot_none);
    code    : #2#15#0#130;
    flags   : if_286 or if_prot or if_priv
  ),
  (
    opcode  : A_LMSW;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16,ot_none,ot_none,ot_none);
    code    : #2#15#1#134;
    flags   : if_286 or if_priv
  ),
  (
    opcode  : A_LOADALL;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#7;
    flags   : if_386 or if_undoc
  ),
  (
    opcode  : A_LOADALL286;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#5;
    flags   : if_286 or if_undoc
  ),
  (
    opcode  : A_LOCK;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#240;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_LODSB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#172;
    flags   : if_8086
  ),
  (
    opcode  : A_LODSD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#173;
    flags   : if_386
  ),
  (
    opcode  : A_LODSW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#173;
    flags   : if_8086
  ),
  (
    opcode  : A_LOOP;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #202#1#226#40;
    flags   : if_8086
  ),
  (
    opcode  : A_LOOP;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_cx,ot_none,ot_none);
    code    : #200#1#226#40;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_LOOP;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_ecx or ot_bits64,ot_none,ot_none);
    code    : #201#1#226#40;
    flags   : if_386
  ),
  (
    opcode  : A_LOOPE;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #202#1#225#40;
    flags   : if_8086
  ),
  (
    opcode  : A_LOOPE;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_cx,ot_none,ot_none);
    code    : #200#1#225#40;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_LOOPE;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_ecx or ot_bits64,ot_none,ot_none);
    code    : #201#1#225#40;
    flags   : if_386
  ),
  (
    opcode  : A_LOOPNE;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #202#1#224#40;
    flags   : if_8086
  ),
  (
    opcode  : A_LOOPNE;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_cx,ot_none,ot_none);
    code    : #200#1#224#40;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_LOOPNE;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_ecx or ot_bits64,ot_none,ot_none);
    code    : #201#1#224#40;
    flags   : if_386
  ),
  (
    opcode  : A_LOOPNZ;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #202#1#224#40;
    flags   : if_8086
  ),
  (
    opcode  : A_LOOPNZ;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_cx,ot_none,ot_none);
    code    : #200#1#224#40;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_LOOPNZ;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_ecx or ot_bits64,ot_none,ot_none);
    code    : #201#1#224#40;
    flags   : if_386
  ),
  (
    opcode  : A_LOOPZ;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #202#1#225#40;
    flags   : if_8086
  ),
  (
    opcode  : A_LOOPZ;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_cx,ot_none,ot_none);
    code    : #200#1#225#40;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_LOOPZ;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_ecx or ot_bits64,ot_none,ot_none);
    code    : #201#1#225#40;
    flags   : if_386
  ),
  (
    opcode  : A_LSL;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#2#15#3#72;
    flags   : if_286 or if_prot or if_sm
  ),
  (
    opcode  : A_LSS;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_memory,ot_none,ot_none);
    code    : #208#2#15#178#72;
    flags   : if_386
  ),
  (
    opcode  : A_LTR;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16,ot_none,ot_none,ot_none);
    code    : #2#15#0#131;
    flags   : if_286 or if_prot or if_priv
  ),
  (
    opcode  : A_MONITOR;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#200;
    flags   : if_prescott
  ),
  (
    opcode  : A_MONITOR;
    ops     : 3;
    optypes : (ot_reg_eax,ot_reg_ecx,ot_reg_edx,ot_none);
    code    : #3#15#1#200;
    flags   : if_prescott
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_mem_offs,ot_reg_ax,ot_none,ot_none);
    code    : #212#1#163#36;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_mem_offs,ot_reg_eax,ot_none,ot_none);
    code    : #213#1#163#36;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#137#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg_ax,ot_mem_offs,ot_none,ot_none);
    code    : #212#1#161#37;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg_eax,ot_mem_offs,ot_none,ot_none);
    code    : #213#1#161#37;
    flags   : if_386 or if_sm
=======
=======
>>>>>>> origin/fixes_2_2
    optypes : (ot_reg_ax,ot_mem_offs,ot_none);
    code    : #212#193#1#161#29;
    flags   : if_8086 or if_sm or if_nox86_64
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#139#72;
=======
    optypes : (ot_reg_eax,ot_mem_offs,ot_none);
    code    : #213#193#1#161#29;
    flags   : if_386 or if_sm or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg_eax,ot_mem_offs,ot_none);
    code    : #213#193#1#161#29;
    flags   : if_386 or if_sm or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_regmem,ot_none);
    code    : #193#208#1#139#72;
>>>>>>> graemeg/fixes_2_2
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_immediate,ot_none,ot_none);
    code    : #213#8#184#33;
    flags   : if_386 or if_sd
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #213#1#199#128#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg16,ot_immediate,ot_none,ot_none);
    code    : #212#8#184#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#199#128#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_mem_offs,ot_reg_al,ot_none,ot_none);
    code    : #1#162#36;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#136#65;
    flags   : if_8086
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg_al,ot_mem_offs,ot_none,ot_none);
    code    : #1#160#37;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#138#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg8,ot_immediate,ot_none,ot_none);
    code    : #8#176#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#198#128#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32,ot_reg_cs,ot_none,ot_none);
    code    : #208#1#140#129;
    flags   : if_8086
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32,ot_reg_dess,ot_none,ot_none);
    code    : #208#1#140#65;
    flags   : if_8086
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32,ot_reg_fsgs,ot_none,ot_none);
    code    : #208#1#140#65;
    flags   : if_386
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg_dess,ot_rm_gpr or ot_bits16 or ot_bits32,ot_none,ot_none);
    code    : #209#1#142#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg_fsgs,ot_rm_gpr or ot_bits16 or ot_bits32,ot_none,ot_none);
    code    : #209#1#142#72;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_reg_cr4,ot_none,ot_none);
    code    : #2#15#32#132;
    flags   : if_pent or if_priv or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_reg_creg,ot_none,ot_none);
    code    : #2#15#32#65;
    flags   : if_386 or if_priv or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_reg_dreg,ot_none,ot_none);
    code    : #2#15#33#65;
    flags   : if_386 or if_priv or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_reg_treg,ot_none,ot_none);
    code    : #2#15#36#65;
    flags   : if_386 or if_priv or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg_cr4,ot_reg32,ot_none,ot_none);
    code    : #2#15#34#140;
    flags   : if_pent or if_priv or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg_creg,ot_reg32,ot_none,ot_none);
    code    : #2#15#34#72;
    flags   : if_386 or if_priv or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg_dreg,ot_reg32,ot_none,ot_none);
    code    : #2#15#35#72;
    flags   : if_386 or if_priv or if_nox86_64
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg_treg,ot_reg32,ot_none,ot_none);
    code    : #2#15#38#72;
    flags   : if_386 or if_priv or if_nox86_64
  ),
  (
    opcode  : A_MOVD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_rm_gpr or ot_bits32,ot_none,ot_none);
    code    : #2#15#110#72;
    flags   : if_pent or if_mmx or if_sd
  ),
  (
    opcode  : A_MOVD;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_mmxreg,ot_none,ot_none);
    code    : #2#15#126#65;
<<<<<<< HEAD
    flags   : if_pent or if_mmx or if_sd
=======
    flags   : if_pent or if_mmx
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_MOVD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_rm_gpr or ot_bits32,ot_none,ot_none);
    code    : #241#2#15#110#72;
=======
    optypes : (ot_xmmreg,ot_reg32,ot_none);
    code    : #1#102#211#2#15#110#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVD;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#126#65;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #1#102#213#2#15#126#65;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    optypes : (ot_xmmreg,ot_reg32,ot_none);
    code    : #1#102#211#2#15#110#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVD;
    ops     : 2;
<<<<<<< HEAD
<<<<<<< HEAD
    optypes : (ot_rm_gpr or ot_bits32,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#126#65;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#213#2#15#110#72;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    optypes : (ot_reg32,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#126#65;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#111#72;
    flags   : if_pent or if_mmx or if_sm
=======
    flags   : if_willamette or if_sse2
>>>>>>> origin/cpstrnew
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxrm,ot_mmxreg,ot_none,ot_none);
    code    : #2#15#127#65;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#213#2#15#110#72;
    flags   : if_willamette or if_sse2
>>>>>>> origin/cpstrnew
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#126#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#214#65;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_reg64,ot_none);
    code    : #1#102#214#2#15#110#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none);
    code    : #1#102#214#2#15#126#65;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_reg64,ot_none);
    code    : #1#102#214#2#15#110#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none);
    code    : #1#102#214#2#15#126#65;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_reg64,ot_none);
    code    : #1#102#214#2#15#110#72;
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#211#2#15#126#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg64,ot_xmmreg,ot_none);
    code    : #1#102#214#2#15#126#65;
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#214#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_reg64,ot_none);
    code    : #1#102#214#2#15#110#72;
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#1#102#211#2#15#214#65;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg64,ot_xmmreg,ot_none);
    code    : #1#102#214#2#15#126#65;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #219#193#2#15#126#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVSB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#164;
    flags   : if_8086
  ),
  (
    opcode  : A_MOVSD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#165;
    flags   : if_386
  ),
  (
    opcode  : A_MOVSD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#16#72;
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#242#211#2#15#16#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#242#211#2#15#17#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVSD;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#1#242#211#2#15#17#65;
>>>>>>> graemeg/fixes_2_2
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVSD;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #220#2#15#17#65;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVSW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#165;
    flags   : if_8086
  ),
  (
    opcode  : A_MOVSX;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_rm_gpr or ot_bits16,ot_none,ot_none);
    code    : #208#2#15#191#72;
    flags   : if_386
  ),
  (
    opcode  : A_MOVSX;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #208#2#15#190#72;
    flags   : if_386
  ),
  (
    opcode  : A_MOVZX;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_rm_gpr or ot_bits16,ot_none,ot_none);
    code    : #208#2#15#183#72;
    flags   : if_386
  ),
  (
    opcode  : A_MOVZX;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #208#2#15#182#72;
    flags   : if_386
  ),
  (
    opcode  : A_MUL;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#247#132;
    flags   : if_8086
  ),
  (
    opcode  : A_MUL;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#246#132;
    flags   : if_8086
  ),
  (
    opcode  : A_MWAIT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#201;
    flags   : if_prescott
  ),
  (
    opcode  : A_MWAIT;
    ops     : 2;
    optypes : (ot_reg_eax,ot_reg_ecx,ot_none,ot_none);
    code    : #3#15#1#201;
    flags   : if_prescott
  ),
  (
    opcode  : A_NEG;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#247#131;
    flags   : if_8086
  ),
  (
    opcode  : A_NEG;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#246#131;
    flags   : if_8086
  ),
  (
    opcode  : A_NOP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#144;
    flags   : if_8086
  ),
  (
    opcode  : A_NOT;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#1#247#130;
    flags   : if_8086
  ),
  (
    opcode  : A_NOT;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#246#130;
    flags   : if_8086
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#9#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#11#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#8#65;
    flags   : if_8086
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#10#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#131#129#13;
    flags   : if_8086
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #213#1#13#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #213#1#129#129#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#13#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#129#129#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#12#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_OR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#128#129#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_OUT;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_al,ot_none,ot_none);
    code    : #1#230#20;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_OUT;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_ax,ot_none,ot_none);
    code    : #212#1#231#20;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_OUT;
    ops     : 2;
    optypes : (ot_immediate,ot_reg_eax,ot_none,ot_none);
    code    : #213#1#231#20;
    flags   : if_386 or if_sb
  ),
  (
    opcode  : A_OUT;
    ops     : 2;
    optypes : (ot_reg_dx,ot_reg_al,ot_none,ot_none);
    code    : #1#238;
    flags   : if_8086
  ),
  (
    opcode  : A_OUT;
    ops     : 2;
    optypes : (ot_reg_dx,ot_reg_ax,ot_none,ot_none);
    code    : #212#1#239;
    flags   : if_8086
  ),
  (
    opcode  : A_OUT;
    ops     : 2;
    optypes : (ot_reg_dx,ot_reg_eax,ot_none,ot_none);
    code    : #213#1#239;
    flags   : if_386
  ),
  (
    opcode  : A_OUTSB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#110;
    flags   : if_186
  ),
  (
    opcode  : A_OUTSD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#111;
    flags   : if_386
  ),
  (
    opcode  : A_OUTSW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#111;
    flags   : if_186
  ),
  (
    opcode  : A_PACKSSDW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#107#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PACKSSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#107#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PACKSSWB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#99#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#107#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PACKSSWB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#99#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#107#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PACKUSWB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#103#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PACKUSWB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#103#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PADDB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#252#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#99#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PADDB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#252#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#99#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PADDD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#254#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#254#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PADDSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#236#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#103#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PADDSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#236#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#103#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PADDSIW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#81#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
  ),
  (
    opcode  : A_PADDSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#237#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#237#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PADDUSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#220#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#252#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PADDUSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#220#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#252#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PADDUSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#221#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PADDUSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#221#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PADDW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#253#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#254#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PADDW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#253#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#254#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PAND;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#219#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PAND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#219#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PANDN;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#223#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#236#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PANDN;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#223#72;
    flags   : if_willamette or if_sse2 or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#236#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PAVEB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#80#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
  ),
  (
    opcode  : A_PAVGUSB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#191;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PCMPEQB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#116#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PCMPEQB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#116#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#237#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PCMPEQD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#118#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#237#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPEQD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#118#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PCMPEQW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#117#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PCMPEQW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#117#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#220#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PCMPGTB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#100#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#220#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPGTB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#100#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PCMPGTD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#102#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PCMPGTD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#102#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#221#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PCMPGTW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#101#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#221#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPGTW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#101#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PDISTIB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none,ot_none);
    code    : #2#15#84#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
  ),
  (
    opcode  : A_PF2ID;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#29;
    flags   : if_pent or if_3dnow or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#253#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PFACC;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#174;
    flags   : if_pent or if_3dnow or if_sm
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#253#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PFADD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#158;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFCMPEQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#176;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFCMPGE;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#144;
    flags   : if_pent or if_3dnow or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#219#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PFCMPGT;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#160;
    flags   : if_pent or if_3dnow or if_sm
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#219#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PFMAX;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#164;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFMIN;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#148;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFMUL;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#180;
    flags   : if_pent or if_3dnow or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#223#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PFRCP;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#150;
    flags   : if_pent or if_3dnow or if_sm
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#223#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PFRCPIT1;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#166;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFRCPIT2;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#182;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFRSQIT1;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#167;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFRSQRT;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#151;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFSUB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#154;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFSUBR;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#170;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PI2FD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#13;
    flags   : if_pent or if_3dnow or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#116#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMACHRIW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none,ot_none);
    code    : #2#15#94#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#116#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDWD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#245#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PMADDWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#245#72;
    flags   : if_willamette or if_sm or if_sse2
  ),
  (
    opcode  : A_PMAGW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#82#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#118#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHRIW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#93#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#118#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHRWA;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#183;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PMULHRWC;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#89#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
  ),
  (
    opcode  : A_PMULHW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#229#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#117#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#229#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#117#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PMULLW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#213#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PMULLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#213#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PMVGEZB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none,ot_none);
    code    : #2#15#92#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#100#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMVLZB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none,ot_none);
    code    : #2#15#91#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#100#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMVNZB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none,ot_none);
    code    : #2#15#90#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
  ),
  (
    opcode  : A_PMVZB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none,ot_none);
    code    : #2#15#88#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
  ),
  (
<<<<<<< HEAD
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_reg16,ot_none,ot_none,ot_none);
    code    : #212#8#88;
    flags   : if_8086
  ),
  (
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none);
    code    : #213#8#88;
    flags   : if_386 or if_nox86_64
=======
    opcode  : A_PCMPGTD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#102#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PCMPGTD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#102#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#143#128;
    flags   : if_8086
  ),
  (
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits32,ot_none,ot_none,ot_none);
    code    : #213#1#143#128;
    flags   : if_386 or if_nox86_64
  ),
  (
<<<<<<< HEAD
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_reg_cs,ot_none,ot_none,ot_none);
    code    : #1#15;
    flags   : if_8086 or if_undoc
  ),
  (
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_reg_dess,ot_none,ot_none,ot_none);
    code    : #4;
    flags   : if_8086 or if_nox86_64
=======
    opcode  : A_PCMPGTW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#101#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PCMPGTW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#101#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_reg_fsgs,ot_none,ot_none,ot_none);
    code    : #1#15#5#221;
    flags   : if_386
  ),
  (
    opcode  : A_POPA;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #215#1#97;
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_POPAD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#97;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_POPAW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#97;
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_POPF;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #215#1#157;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_POPFD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#157;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_POPFW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#157;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_POR;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#235#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_POR;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#235#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PREFETCH;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#13#128;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PREFETCHW;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#13#129;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PSLLD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#242#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSLLD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_immediate,ot_none,ot_none);
    code    : #2#15#114#134#21;
    flags   : if_pent or if_mmx or if_sb or if_ar1
  ),
  (
    opcode  : A_PSLLD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#242#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSLLD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#114#134#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSLLDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#115#135#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSLLQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#243#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSLLQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_immediate,ot_none,ot_none);
    code    : #2#15#115#134#21;
    flags   : if_pent or if_mmx or if_sb or if_ar1
  ),
  (
    opcode  : A_PSLLQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#243#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSLLQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#115#134#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSLLW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#241#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSLLW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_immediate,ot_none,ot_none);
    code    : #2#15#113#134#21;
    flags   : if_pent or if_mmx or if_sb or if_ar1
  ),
  (
    opcode  : A_PSLLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#241#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSLLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#113#134#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRAD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#226#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSRAD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_immediate,ot_none,ot_none);
    code    : #2#15#114#132#21;
    flags   : if_pent or if_mmx or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRAD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#226#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSRAD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#114#132#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRAW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#225#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSRAW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_immediate,ot_none,ot_none);
    code    : #2#15#113#132#21;
    flags   : if_pent or if_mmx or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRAW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#225#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSRAW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#113#132#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRLD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#210#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSRLD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_immediate,ot_none,ot_none);
    code    : #2#15#114#130#21;
    flags   : if_pent or if_mmx or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRLD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#210#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSRLD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#114#130#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRLQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#211#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSRLQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_immediate,ot_none,ot_none);
    code    : #2#15#115#130#21;
    flags   : if_pent or if_mmx or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRLQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#211#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#245#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSRLQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#115#130#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#245#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSRLW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#209#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSRLW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_immediate,ot_none,ot_none);
    code    : #2#15#113#130#21;
    flags   : if_pent or if_mmx or if_sb or if_ar1
  ),
  (
    opcode  : A_PSRLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#209#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSRLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#113#130#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSUBB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#248#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSUBB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#248#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSUBD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#250#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#250#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSUBSB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#232#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSUBSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#232#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSUBSIW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none,ot_none);
    code    : #2#15#85#72;
    flags   : if_pent or if_mmx or if_sm or if_cyrix
  ),
  (
    opcode  : A_PSUBSIW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none,ot_none);
    code    : #2#15#85#72;
    flags   : if_pent or if_mmx or if_cyrix
  ),
  (
    opcode  : A_PSUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#233#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#233#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSUBUSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#216#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSUBUSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#216#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#229#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSUBUSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#217#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#229#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSUBUSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#217#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSUBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#249#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#249#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKHBW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#104#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PUNPCKHBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#104#72;
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#213#72;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKHDQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#106#72;
    flags   : if_pent or if_mmx or if_sm
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#213#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PUNPCKHDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#106#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKHWD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#105#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PUNPCKHWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#105#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKLBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#96#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PUNPCKLBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#96#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKLDQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#98#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PUNPCKLDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#98#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKLWD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#97#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PUNPCKLWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#97#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_reg16,ot_none,ot_none,ot_none);
    code    : #212#8#80;
    flags   : if_8086
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none);
    code    : #213#8#80;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#255#134;
    flags   : if_8086
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits32,ot_none,ot_none,ot_none);
    code    : #213#1#255#134;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_immediate or ot_bits32,ot_none,ot_none,ot_none);
    code    : #213#1#104#32#221;
    flags   : if_386
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_immediate or ot_bits16,ot_none,ot_none,ot_none);
    code    : #212#1#104#24#221;
    flags   : if_286
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none,ot_none);
    code    : #1#106#12#221;
    flags   : if_286
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_reg_fsgs,ot_none,ot_none,ot_none);
    code    : #1#15#7#221;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_reg_sreg,ot_none,ot_none,ot_none);
    code    : #6;
    flags   : if_8086 or if_nox86_64
  ),
  (
    opcode  : A_PUSHA;
    ops     : 0;
<<<<<<< HEAD
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #215#1#96;
=======
    optypes : (ot_none,ot_none,ot_none);
    code    : #215#1#97;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_PUSHAD;
    ops     : 0;
<<<<<<< HEAD
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#96;
=======
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#97;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSHAW;
    ops     : 0;
<<<<<<< HEAD
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#96;
=======
    optypes : (ot_none,ot_none,ot_none);
    code    : #212#1#97;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_PUSHF;
    ops     : 0;
<<<<<<< HEAD
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #215#1#156;
    flags   : if_8086
=======
    optypes : (ot_none,ot_none,ot_none);
    code    : #215#1#157;
    flags   : if_186 or if_nox86_64
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ),
  (
    opcode  : A_PUSHFD;
    ops     : 0;
<<<<<<< HEAD
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#156;
=======
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#157;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSHFW;
    ops     : 0;
<<<<<<< HEAD
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#156;
    flags   : if_8086
=======
    optypes : (ot_none,ot_none,ot_none);
    code    : #212#1#157;
    flags   : if_186 or if_nox86_64
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ),
  (
    opcode  : A_PXOR;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#239#72;
    flags   : if_pent or if_mmx or if_sm
  ),
  (
    opcode  : A_PXOR;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#239#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_RCL;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_unity,ot_none,ot_none);
    code    : #208#1#209#130;
    flags   : if_8086
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#235#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_RCL;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none,ot_none);
    code    : #208#1#211#130;
    flags   : if_8086
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#235#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_RCL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#1#193#130#21;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_RCL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_unity,ot_none,ot_none);
    code    : #1#208#130;
    flags   : if_8086
  ),
  (
    opcode  : A_RCL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg_cl,ot_none,ot_none);
    code    : #1#210#130;
    flags   : if_8086
  ),
  (
    opcode  : A_RCL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#192#130#21;
    flags   : if_186 or if_sb
  ),
  (
    opcode  : A_RCR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_unity,ot_none,ot_none);
    code    : #208#1#209#131;
    flags   : if_8086
  ),
  (
    opcode  : A_RCR;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none,ot_none);
    code    : #208#1#211#131;
    flags   : if_8086
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#242#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> origin/fixes_2_2
  ),
  (
    opcode  : A_RCR;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#1#193#131#21;
    flags   : if_8086 or if_sb
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#242#72;
    flags   : if_willamette or if_sse2
>>>>>>> origin/fixes_2_2
  ),
  (
    opcode  : A_RCR;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_rm_gpr or ot_bits8,ot_unity,ot_none,ot_none);
    code    : #1#208#131;
    flags   : if_8086
=======
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#114#134#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> origin/fixes_2_2
  ),
  (
    opcode  : A_RCR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg_cl,ot_none,ot_none);
    code    : #1#210#131;
    flags   : if_8086
  ),
  (
    opcode  : A_RCR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#192#131#21;
    flags   : if_186 or if_sb
  ),
  (
    opcode  : A_RDSHR;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#54;
    flags   : if_p6 or if_cyrix or if_smm
  ),
  (
    opcode  : A_RDMSR;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#50;
    flags   : if_pent or if_priv
  ),
  (
    opcode  : A_RDPMC;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#51;
    flags   : if_p6
  ),
  (
    opcode  : A_RDTSC;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#49;
    flags   : if_pent
  ),
  (
    opcode  : A_REP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#243;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_REPE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#243;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_REPNE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#242;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_REPNZ;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#242;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_REPZ;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#243;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_RET;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#195;
    flags   : if_8086
  ),
  (
    opcode  : A_RET;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #1#194#24;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_RETF;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#203;
    flags   : if_8086
  ),
  (
    opcode  : A_RETF;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #1#202#24;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_RETN;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#195;
    flags   : if_8086
  ),
  (
    opcode  : A_RETN;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none);
    code    : #1#194#24;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_ROL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_unity,ot_none,ot_none);
    code    : #208#1#209#128;
    flags   : if_8086
  ),
  (
    opcode  : A_ROL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none,ot_none);
    code    : #208#1#211#128;
    flags   : if_8086
  ),
  (
    opcode  : A_ROL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#1#193#128#21;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_ROL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_unity,ot_none,ot_none);
    code    : #1#208#128;
    flags   : if_8086
  ),
  (
    opcode  : A_ROL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg_cl,ot_none,ot_none);
    code    : #1#210#128;
    flags   : if_8086
  ),
  (
    opcode  : A_ROL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#192#128#21;
    flags   : if_186 or if_sb
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_unity,ot_none,ot_none);
    code    : #208#1#209#129;
    flags   : if_8086
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none,ot_none);
    code    : #208#1#211#129;
    flags   : if_8086
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#1#193#129#21;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_unity,ot_none,ot_none);
    code    : #1#208#129;
    flags   : if_8086
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg_cl,ot_none,ot_none);
    code    : #1#210#129;
    flags   : if_8086
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#192#129#21;
    flags   : if_186 or if_sb
  ),
  (
    opcode  : A_RSDC;
    ops     : 2;
    optypes : (ot_reg_sreg,ot_memory or ot_bits80,ot_none,ot_none);
    code    : #2#15#121#65;
    flags   : if_486 or if_cyrix or if_smm
  ),
  (
    opcode  : A_RSLDT;
    ops     : 1;
    optypes : (ot_memory or ot_bits80,ot_none,ot_none,ot_none);
    code    : #2#15#123#128;
    flags   : if_486 or if_cyrix or if_smm
  ),
  (
    opcode  : A_RSM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#170;
    flags   : if_pent or if_smm
  ),
  (
    opcode  : A_SAHF;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#158;
    flags   : if_8086
  ),
  (
    opcode  : A_SAL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_unity,ot_none,ot_none);
    code    : #208#1#209#132;
    flags   : if_8086
  ),
  (
    opcode  : A_SAL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none,ot_none);
    code    : #208#1#211#132;
    flags   : if_8086
  ),
  (
    opcode  : A_SAL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#1#193#132#21;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_SAL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_unity,ot_none,ot_none);
    code    : #1#208#132;
    flags   : if_8086
  ),
  (
    opcode  : A_SAL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg_cl,ot_none,ot_none);
    code    : #1#210#132;
    flags   : if_8086
  ),
  (
    opcode  : A_SAL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#192#132#21;
    flags   : if_186 or if_sb
  ),
  (
    opcode  : A_SALC;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#214;
    flags   : if_8086 or if_undoc or if_nox86_64
  ),
  (
    opcode  : A_SAR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_unity,ot_none,ot_none);
    code    : #208#1#209#135;
    flags   : if_8086
  ),
  (
    opcode  : A_SAR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none,ot_none);
    code    : #208#1#211#135;
    flags   : if_8086
  ),
  (
    opcode  : A_SAR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#1#193#135#21;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_SAR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_unity,ot_none,ot_none);
    code    : #1#208#135;
    flags   : if_8086
  ),
  (
    opcode  : A_SAR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg_cl,ot_none,ot_none);
    code    : #1#210#135;
    flags   : if_8086
  ),
  (
    opcode  : A_SAR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#192#135#21;
    flags   : if_186 or if_sb
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#25#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#27#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#131#131#13;
    flags   : if_8086
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#24#65;
    flags   : if_8086
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#26#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #213#1#29#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #208#1#129#131#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#29#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #208#1#129#131#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#28#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_SBB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#128#131#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_SCASB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #218#1#174;
    flags   : if_8086
  ),
  (
    opcode  : A_SCASD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #218#213#1#175;
    flags   : if_386
  ),
  (
    opcode  : A_SCASW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #218#212#1#175;
    flags   : if_8086
  ),
  (
    opcode  : A_SEGCS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#46;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_SEGDS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#62;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_SEGES;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#38;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_SEGFS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#100;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_SEGGS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#101;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_SEGSS;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#54;
    flags   : if_8086 or if_pre
  ),
  (
    opcode  : A_SGDT;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#1#128;
    flags   : if_286
  ),
  (
    opcode  : A_SHL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_unity,ot_none,ot_none);
    code    : #208#1#209#132;
    flags   : if_8086
  ),
  (
    opcode  : A_SHL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none,ot_none);
    code    : #208#1#211#132;
    flags   : if_8086
  ),
  (
    opcode  : A_SHL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#1#193#132#21;
    flags   : if_186 or if_sw
  ),
  (
    opcode  : A_SHL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_unity,ot_none,ot_none);
    code    : #1#208#132;
    flags   : if_8086
  ),
  (
    opcode  : A_SHL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg_cl,ot_none,ot_none);
    code    : #1#210#132;
    flags   : if_8086
  ),
  (
    opcode  : A_SHL;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#192#132#21;
    flags   : if_186 or if_sb
  ),
  (
    opcode  : A_SHLD;
    ops     : 3;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg16 or ot_bits32 or ot_bits64,ot_immediate,ot_none);
    code    : #209#2#15#164#65#22;
    flags   : if_386 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_SHLD;
    ops     : 3;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none);
    code    : #209#2#15#165#65;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_SHR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_unity,ot_none,ot_none);
    code    : #208#1#209#133;
    flags   : if_8086
  ),
  (
    opcode  : A_SHR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none,ot_none);
    code    : #208#1#211#133;
    flags   : if_8086
  ),
  (
    opcode  : A_SHR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none,ot_none);
    code    : #208#1#193#133#21;
    flags   : if_186 or if_sw
  ),
  (
    opcode  : A_SHR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_unity,ot_none,ot_none);
    code    : #1#208#133;
    flags   : if_8086
  ),
  (
    opcode  : A_SHR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg_cl,ot_none,ot_none);
    code    : #1#210#133;
    flags   : if_8086
  ),
  (
    opcode  : A_SHR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#192#133#21;
    flags   : if_186 or if_sb
  ),
  (
    opcode  : A_SHRD;
    ops     : 3;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg16 or ot_bits32 or ot_bits64,ot_immediate,ot_none);
    code    : #209#2#15#172#65#22;
    flags   : if_386 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_SHRD;
    ops     : 3;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_reg16 or ot_bits32 or ot_bits64,ot_reg_cl,ot_none);
    code    : #209#2#15#173#65;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_SIDT;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#1#129;
    flags   : if_286
  ),
  (
    opcode  : A_SLDT;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#0#128;
    flags   : if_286
  ),
  (
    opcode  : A_SLDT;
    ops     : 1;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#2#15#0#128;
    flags   : if_286
  ),
  (
    opcode  : A_SMI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#241;
    flags   : if_386 or if_undoc
  ),
  (
    opcode  : A_SMINT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#56;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_SMINTOLD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#126;
    flags   : if_486 or if_cyrix
  ),
  (
    opcode  : A_SMSW;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#2#15#1#132;
    flags   : if_286
  ),
  (
    opcode  : A_STC;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#249;
    flags   : if_8086
  ),
  (
    opcode  : A_STD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#253;
    flags   : if_8086
  ),
  (
    opcode  : A_STI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#251;
    flags   : if_8086
  ),
  (
    opcode  : A_STOSB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#170;
    flags   : if_8086
  ),
  (
    opcode  : A_STOSD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #213#1#171;
    flags   : if_386
  ),
  (
    opcode  : A_STOSW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #212#1#171;
    flags   : if_8086
  ),
  (
    opcode  : A_STR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#0#129;
    flags   : if_286 or if_prot
  ),
  (
    opcode  : A_STR;
    ops     : 1;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none,ot_none);
    code    : #208#2#15#0#129;
    flags   : if_286 or if_prot
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#41#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#43#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#40#65;
    flags   : if_8086
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#42#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#131#133#13;
    flags   : if_8086
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #213#1#45#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #208#1#129#133#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#45#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#129#133#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#44#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#128#133#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_SVDC;
    ops     : 2;
    optypes : (ot_memory or ot_bits80,ot_reg_sreg,ot_none,ot_none);
    code    : #2#15#120#65;
    flags   : if_486 or if_cyrix or if_smm
  ),
  (
    opcode  : A_SVLDT;
    ops     : 1;
    optypes : (ot_memory or ot_bits80,ot_none,ot_none,ot_none);
    code    : #2#15#122#128;
    flags   : if_486 or if_cyrix or if_smm
  ),
  (
    opcode  : A_SVTS;
    ops     : 1;
    optypes : (ot_memory or ot_bits80,ot_none,ot_none,ot_none);
    code    : #2#15#124#128;
    flags   : if_486 or if_cyrix or if_smm
  ),
  (
    opcode  : A_SYSCALL;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#5;
    flags   : if_p6 or if_amd
  ),
  (
    opcode  : A_SYSENTER;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#52;
    flags   : if_p6
  ),
  (
    opcode  : A_SYSEXIT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#53;
    flags   : if_p6 or if_priv
  ),
  (
    opcode  : A_SYSRET;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#7;
    flags   : if_p6 or if_priv or if_amd
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#133#65;
    flags   : if_8086 or if_sm
  ),
  (
<<<<<<< HEAD
<<<<<<< HEAD
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_memory,ot_none,ot_none);
    code    : #208#1#133#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_reg8,ot_reg8,ot_none,ot_none);
    code    : #1#132#65;
    flags   : if_8086
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#132#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #213#1#169#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#169#25;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#168#17;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #213#1#247#128#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#247#128#25;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#246#128#17;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_memory,ot_immediate or ot_bits32,ot_none,ot_none);
    code    : #213#1#247#128#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_memory,ot_immediate or ot_bits16,ot_none,ot_none);
    code    : #212#1#247#128#25;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_TEST;
    ops     : 2;
    optypes : (ot_memory,ot_immediate or ot_bits8,ot_none,ot_none);
    code    : #1#246#128#17;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_UD1;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#185;
    flags   : if_286 or if_undoc
  ),
  (
    opcode  : A_UD2;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#11;
    flags   : if_286
  ),
  (
    opcode  : A_UMOV;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#17#65;
    flags   : if_386 or if_undoc or if_sm
  ),
  (
    opcode  : A_UMOV;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_memory,ot_none,ot_none);
    code    : #208#2#15#19#72;
    flags   : if_386 or if_undoc or if_sm
  ),
  (
    opcode  : A_UMOV;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #2#15#16#65;
    flags   : if_386 or if_undoc
  ),
  (
    opcode  : A_UMOV;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #2#15#18#72;
    flags   : if_386 or if_undoc
  ),
  (
    opcode  : A_VERR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#0#132;
    flags   : if_286 or if_prot
  ),
  (
    opcode  : A_VERR;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #2#15#0#132;
    flags   : if_286 or if_prot
  ),
  (
    opcode  : A_VERR;
    ops     : 1;
    optypes : (ot_reg16,ot_none,ot_none,ot_none);
    code    : #2#15#0#132;
    flags   : if_286 or if_prot
  ),
  (
    opcode  : A_VERW;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#0#133;
    flags   : if_286 or if_prot
  ),
  (
    opcode  : A_VERW;
    ops     : 1;
    optypes : (ot_memory or ot_bits16,ot_none,ot_none,ot_none);
    code    : #2#15#0#133;
    flags   : if_286 or if_prot
  ),
  (
    opcode  : A_VERW;
    ops     : 1;
    optypes : (ot_reg16,ot_none,ot_none,ot_none);
    code    : #2#15#0#133;
    flags   : if_286 or if_prot
  ),
  (
    opcode  : A_WAIT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#155;
    flags   : if_8086
  ),
  (
    opcode  : A_WBINVD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#9;
    flags   : if_486 or if_priv
  ),
  (
    opcode  : A_WRSHR;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#55;
    flags   : if_p6 or if_cyrix or if_smm
  ),
  (
    opcode  : A_WRMSR;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#48;
    flags   : if_pent or if_priv
  ),
  (
    opcode  : A_XADD;
    ops     : 2;
    optypes : (ot_memory,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#193#65;
    flags   : if_486 or if_sm
  ),
  (
    opcode  : A_XADD;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #2#15#192#65;
    flags   : if_486
  ),
  (
    opcode  : A_XADD;
    ops     : 2;
    optypes : (ot_reg8,ot_reg8,ot_none,ot_none);
    code    : #2#15#192#65;
    flags   : if_486
  ),
  (
    opcode  : A_XADD;
    ops     : 2;
    optypes : (ot_reg16,ot_reg16,ot_none,ot_none);
    code    : #208#2#15#193#65;
    flags   : if_486
  ),
  (
    opcode  : A_XADD;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #209#2#15#193#65;
    flags   : if_486
  ),
  (
    opcode  : A_XBTS;
    ops     : 2;
    optypes : (ot_reg16,ot_memory,ot_none,ot_none);
    code    : #212#2#15#166#72;
    flags   : if_386 or if_sw or if_undoc
  ),
  (
    opcode  : A_XBTS;
    ops     : 2;
    optypes : (ot_reg16,ot_reg16,ot_none,ot_none);
    code    : #212#2#15#166#72;
    flags   : if_386 or if_undoc
  ),
  (
    opcode  : A_XBTS;
    ops     : 2;
    optypes : (ot_reg32,ot_memory,ot_none,ot_none);
    code    : #213#2#15#166#72;
    flags   : if_386 or if_sd or if_undoc
  ),
  (
    opcode  : A_XBTS;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #213#2#15#166#72;
    flags   : if_386 or if_undoc
  ),
  (
    opcode  : A_XCHG;
    ops     : 2;
    optypes : (ot_reg_ax,ot_reg16,ot_none,ot_none);
    code    : #212#9#144;
    flags   : if_8086
  ),
  (
    opcode  : A_XCHG;
    ops     : 2;
    optypes : (ot_reg_eax,ot_reg32,ot_none,ot_none);
    code    : #213#9#144;
    flags   : if_386
  ),
  (
    opcode  : A_XCHG;
    ops     : 2;
    optypes : (ot_reg16,ot_reg_ax,ot_none,ot_none);
    code    : #212#8#144;
    flags   : if_8086
  ),
  (
    opcode  : A_XCHG;
    ops     : 2;
    optypes : (ot_reg32,ot_reg_eax,ot_none,ot_none);
    code    : #213#8#144;
    flags   : if_386
  ),
  (
    opcode  : A_XCHG;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#135#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_XCHG;
    ops     : 2;
    optypes : (ot_memory,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#135#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_XCHG;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#134#72;
    flags   : if_8086
  ),
  (
    opcode  : A_XCHG;
    ops     : 2;
    optypes : (ot_memory or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#134#65;
    flags   : if_8086
  ),
  (
    opcode  : A_XLAT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#215;
    flags   : if_8086
  ),
  (
    opcode  : A_XLATB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #1#215;
    flags   : if_8086
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_rm_gpr,ot_reg16 or ot_bits32 or ot_bits64,ot_none,ot_none);
    code    : #208#1#49#65;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#51#72;
    flags   : if_8086 or if_sm
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_reg8,ot_none,ot_none);
    code    : #1#48#65;
    flags   : if_8086
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_reg8,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #1#50#72;
    flags   : if_8086
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate or ot_bits8 or ot_signed,ot_none,ot_none);
    code    : #208#1#131#134#13;
    flags   : if_8086
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_reg_eax,ot_immediate,ot_none,ot_none);
    code    : #213#1#53#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_immediate,ot_none,ot_none);
    code    : #208#1#129#134#33;
    flags   : if_386 or if_sm
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_reg_ax,ot_immediate,ot_none,ot_none);
    code    : #212#1#53#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits16,ot_immediate,ot_none,ot_none);
    code    : #212#1#129#134#25;
    flags   : if_8086 or if_sw
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_reg_al,ot_immediate,ot_none,ot_none);
    code    : #1#52#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_XOR;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits8,ot_immediate,ot_none,ot_none);
    code    : #1#128#134#17;
    flags   : if_8086 or if_sb
  ),
  (
    opcode  : A_XSTORE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#167#192;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_XCRYPTECB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #219#3#15#167#200;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_XCRYPTCBC;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #219#3#15#167#208;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_XCRYPTCFB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #219#3#15#167#224;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_XCRYPTOFB;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #219#3#15#167#232;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_CMOVcc;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#1#15#11#64#72;
    flags   : if_p6 or if_sm
  ),
  (
    opcode  : A_Jcc;
    ops     : 1;
    optypes : (ot_immediate or ot_bits8,ot_none,ot_none,ot_none);
    code    : #11#112#40;
    flags   : if_8086
  ),
  (
    opcode  : A_Jcc;
    ops     : 1;
    optypes : (ot_immediate or ot_bits16 or ot_bits32,ot_none,ot_none,ot_none);
    code    : #208#1#15#11#128#52;
    flags   : if_386 or if_pass2
  ),
  (
    opcode  : A_Jcc;
    ops     : 1;
    optypes : (ot_immediate or ot_short,ot_none,ot_none,ot_none);
    code    : #11#112#40;
    flags   : if_8086
  ),
  (
    opcode  : A_Jcc;
    ops     : 1;
    optypes : (ot_immediate or ot_near,ot_none,ot_none,ot_none);
    code    : #208#1#15#11#128#52;
    flags   : if_386 or if_pass2
  ),
  (
    opcode  : A_SETcc;
    ops     : 1;
    optypes : (ot_rm_gpr or ot_bits8,ot_none,ot_none,ot_none);
    code    : #1#15#11#144#128;
    flags   : if_386
  ),
  (
    opcode  : A_ADDPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#88#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_ADDSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#88#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_ANDNPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#85#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_ANDPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#84#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPEQPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#194#72#1#0;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPEQSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#194#72#1#0;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPLEPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#194#72#1#2;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPLESS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#194#72#1#2;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPLTPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#194#72#1#1;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPLTSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#194#72#1#1;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPNEQPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#194#72#1#4;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPNEQSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#194#72#1#4;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPNLEPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#194#72#1#6;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPNLESS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#194#72#1#6;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPNLTPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#194#72#1#5;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPNLTSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#194#72#1#5;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPORDPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#194#72#1#7;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPORDSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#194#72#1#7;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPUNORDPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#194#72#1#3;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPUNORDSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#194#72#1#3;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CMPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #217#2#15#194#72#18;
    flags   : if_katmai or if_sse or if_sb or if_ar2
  ),
  (
    opcode  : A_CMPSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #219#2#15#194#72#18;
    flags   : if_katmai or if_sse or if_sb or if_ar2
  ),
  (
    opcode  : A_COMISS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#47#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CVTPI2PS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#2#15#42#72;
    flags   : if_katmai or if_sse or if_mmx
  ),
  (
    opcode  : A_CVTPS2PI;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #217#2#15#45#72;
    flags   : if_katmai or if_sse or if_mmx
  ),
  (
    opcode  : A_CVTPS2PI;
    ops     : 2;
    optypes : (ot_mmxreg,ot_xmmreg,ot_none,ot_none);
    code    : #217#2#15#45#72;
    flags   : if_katmai or if_sse or if_mmx
  ),
  (
    opcode  : A_CVTSI2SS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_rm_gpr or ot_bits32,ot_none,ot_none);
    code    : #219#209#2#15#42#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CVTSS2SI;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #219#208#2#15#45#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CVTSS2SI;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #219#208#2#15#45#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CVTTPS2PI;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #217#2#15#44#72;
    flags   : if_katmai or if_sse or if_mmx
  ),
  (
    opcode  : A_CVTTPS2PI;
    ops     : 2;
    optypes : (ot_mmxreg,ot_xmmreg,ot_none,ot_none);
    code    : #217#2#15#44#72;
    flags   : if_katmai or if_sse or if_mmx
  ),
  (
    opcode  : A_CVTTSS2SI;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #219#208#2#15#44#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CVTTSS2SI;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #219#208#2#15#44#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_DIVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#94#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_DIVSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#94#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_LDMXCSR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#174#130;
    flags   : if_katmai or if_sse or if_sd
  ),
  (
    opcode  : A_MAXPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#95#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MAXSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#95#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MINPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#93#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MINSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#93#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVAPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#40#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVAPS;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #2#15#41#65;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVHPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none,ot_none);
    code    : #2#15#22#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVHPS;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #2#15#23#65;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVLHPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #2#15#22#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVLPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none,ot_none);
    code    : #2#15#18#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVLPS;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #2#15#19#65;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVHLPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #2#15#18#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVMSKPS;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #2#15#80#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVNTPS;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #2#15#43#65;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#16#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVSS;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #219#2#15#17#65;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVUPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#16#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MOVUPS;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #217#2#15#17#65;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MULPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#89#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_MULSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#89#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_ORPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#86#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_RCPPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#83#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_RCPSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#83#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_RSQRTPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#82#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_RSQRTSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#82#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_SHUFPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #2#15#198#72#18;
    flags   : if_katmai or if_sse or if_sb or if_ar2
  ),
  (
    opcode  : A_SQRTPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#81#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_SQRTSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#81#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_STMXCSR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#174#131;
    flags   : if_katmai or if_sse or if_sd
  ),
  (
    opcode  : A_SUBPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #217#2#15#92#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_SUBSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#92#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_UCOMISS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#46#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_UNPCKHPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#21#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_UNPCKLPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#20#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_XORPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#87#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_FXRSTOR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#174#129;
    flags   : if_p6 or if_sse or if_fpu
  ),
  (
    opcode  : A_FXSAVE;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#174#128;
    flags   : if_p6 or if_sse or if_fpu
  ),
  (
    opcode  : A_PREFETCHNTA;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#24#128;
    flags   : if_katmai
  ),
  (
    opcode  : A_PREFETCHT0;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#24#129;
    flags   : if_katmai
  ),
  (
    opcode  : A_PREFETCHT1;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#24#130;
    flags   : if_katmai
  ),
  (
    opcode  : A_PREFETCHT2;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#24#131;
    flags   : if_katmai
  ),
  (
    opcode  : A_SFENCE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#174#248;
    flags   : if_katmai
  ),
  (
    opcode  : A_MASKMOVQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none,ot_none);
    code    : #2#15#247#72;
    flags   : if_katmai or if_mmx
  ),
  (
    opcode  : A_MOVNTQ;
    ops     : 2;
    optypes : (ot_memory,ot_mmxreg,ot_none,ot_none);
    code    : #2#15#231#65;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PAVGB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#224#72;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PAVGB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#224#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PAVGW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#227#72;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PAVGW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#227#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_mmxreg,ot_immediate,ot_none);
    code    : #2#15#197#72#18;
    flags   : if_katmai or if_mmx or if_sb or if_ar2
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate,ot_none);
    code    : #241#2#15#197#72#22;
    flags   : if_sse41
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate,ot_none);
    code    : #241#3#15#58#21#65#22;
    flags   : if_sse41
  ),
  (
    opcode  : A_PINSRW;
    ops     : 3;
    optypes : (ot_mmxreg,ot_reg16,ot_immediate,ot_none);
    code    : #2#15#196#72#18;
    flags   : if_katmai or if_mmx or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRW;
    ops     : 3;
    optypes : (ot_mmxreg,ot_reg32,ot_immediate,ot_none);
    code    : #2#15#196#72#18;
    flags   : if_katmai or if_mmx or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRW;
    ops     : 3;
    optypes : (ot_mmxreg,ot_memory,ot_immediate,ot_none);
    code    : #2#15#196#72#18;
    flags   : if_katmai or if_mmx or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRW;
    ops     : 3;
    optypes : (ot_mmxreg,ot_memory or ot_bits16,ot_immediate,ot_none);
    code    : #2#15#196#72#18;
    flags   : if_katmai or if_mmx or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg16,ot_immediate,ot_none);
    code    : #241#2#15#196#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate,ot_none);
    code    : #241#2#15#196#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate,ot_none);
    code    : #241#2#15#196#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits16,ot_immediate,ot_none);
    code    : #241#2#15#196#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PMAXSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#238#72;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PMAXSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#238#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PMAXUB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#222#72;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PMAXUB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#222#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PMINSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#234#72;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PMINSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#234#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PMINUB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#218#72;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PMINUB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#218#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PMOVMSKB;
    ops     : 2;
    optypes : (ot_reg32,ot_mmxreg,ot_none,ot_none);
    code    : #2#15#215#72;
    flags   : if_katmai or if_mmx
  ),
  (
    opcode  : A_PMOVMSKB;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#215#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PMULHUW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#228#72;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PMULHUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#228#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSADBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#246#72;
    flags   : if_katmai or if_mmx or if_sm
  ),
  (
    opcode  : A_PSADBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#246#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSHUFW;
    ops     : 3;
    optypes : (ot_mmxreg,ot_mmxrm,ot_immediate,ot_none);
    code    : #2#15#112#72#18;
    flags   : if_katmai or if_mmx or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PFNACC;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#138;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PFPNACC;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#142;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PI2FW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#12;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PF2IW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#28;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_PSWAPD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#15#72#1#187;
    flags   : if_pent or if_3dnow or if_sm
  ),
  (
    opcode  : A_FFREEP;
    ops     : 1;
    optypes : (ot_fpureg,ot_none,ot_none,ot_none);
    code    : #1#223#8#192;
    flags   : if_pent or if_3dnow or if_fpu
  ),
  (
    opcode  : A_MASKMOVDQU;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#247#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CLFLUSH;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#174#135;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVNTDQ;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#231#65;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVNTI;
    ops     : 2;
    optypes : (ot_memory,ot_reg32 or ot_bits64,ot_none,ot_none);
    code    : #208#2#15#195#65;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVNTPD;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#43#65;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PAUSE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #219#1#144;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_LFENCE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#174#232;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MFENCE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#174#240;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVDQA;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#127#65;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVDQA;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#111#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVDQU;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #219#2#15#127#65;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVDQU;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#111#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVDQ2Q;
    ops     : 2;
    optypes : (ot_mmxreg,ot_xmmreg,ot_none,ot_none);
    code    : #220#2#15#214#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVQ2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_mmxreg,ot_none,ot_none);
    code    : #219#2#15#214#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PADDQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#212#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PADDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#212#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PMULUDQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#244#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PMULUDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#244#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSHUFD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#2#15#112#72#18;
    flags   : if_willamette or if_sse2 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PSHUFHW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #219#2#15#112#72#18;
    flags   : if_willamette or if_sse2 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PSHUFLW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #220#2#15#112#72#18;
    flags   : if_willamette or if_sse2 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PSRLDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none,ot_none);
    code    : #241#2#15#115#131#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSUBQ;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #2#15#251#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSUBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#251#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKHQDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#109#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKLQDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#108#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_ADDPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#88#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_ADDSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#88#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_ANDNPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#85#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_ANDPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#84#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPEQPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#194#72#1#0;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPEQSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#194#72#1#0;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPLEPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#194#72#1#2;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPLESD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#194#72#1#2;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPLTPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#194#72#1#1;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPLTSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#194#72#1#1;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPNEQPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#194#72#1#4;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPNEQSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#194#72#1#4;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPNLEPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#194#72#1#6;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPNLESD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#194#72#1#6;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPNLTPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#194#72#1#5;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPNLTSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#194#72#1#5;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPORDPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#194#72#1#7;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPORDSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#194#72#1#7;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPUNORDPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#194#72#1#3;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CMPUNORDSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#194#72#1#3;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#2#15#194#72#22;
    flags   : if_willamette or if_sse2 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_COMISD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#47#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTDQ2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #219#2#15#230#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTDQ2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #219#2#15#230#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTDQ2PS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #2#15#91#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CVTPD2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#230#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CVTPD2PI;
    ops     : 2;
    optypes : (ot_mmxreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#45#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTPD2PS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#90#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CVTPI2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_mmxrm,ot_none,ot_none);
    code    : #241#2#15#42#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTPS2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#91#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CVTPS2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #2#15#90#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTPS2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #2#15#90#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTSD2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #220#2#15#45#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTSD2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#2#15#45#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTSD2SS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #220#2#15#90#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTSD2SS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#2#15#90#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTSI2SD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_rm_gpr or ot_bits32,ot_none,ot_none);
    code    : #220#2#15#42#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTSS2SD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #219#2#15#90#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTSS2SD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #219#2#15#90#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTTPD2PI;
    ops     : 2;
    optypes : (ot_mmxreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#44#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTTPD2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#230#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CVTTPS2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#91#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_CVTTSD2SI;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #220#208#2#15#44#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTTSD2SI;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#208#2#15#44#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_DIVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#94#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_DIVSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#94#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MAXPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#95#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#95#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MINPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#93#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#93#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVAPD;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#41#65;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVAPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#40#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVHPD;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#23#65;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVHPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none,ot_none);
    code    : #241#2#15#22#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVLPD;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#19#65;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVLPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none,ot_none);
    code    : #241#2#15#18#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVMSKPD;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#80#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVUPD;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#17#65;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MOVUPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#16#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MULPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#89#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_MULSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#89#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_ORPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#86#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_SHUFPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#2#15#198#72#22;
    flags   : if_willamette or if_sse2 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_SQRTPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#81#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_SQRTSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#81#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_SUBPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#92#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_SUBSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#92#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_UCOMISD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#46#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_UNPCKHPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#21#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_UNPCKLPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#20#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_XORPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#87#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_ADDSUBPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#208#72;
    flags   : if_prescott or if_sse3 or if_sm
  ),
  (
    opcode  : A_ADDSUBPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#208#72;
    flags   : if_prescott or if_sse3 or if_sm
  ),
  (
    opcode  : A_HADDPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#124#72;
    flags   : if_prescott or if_sse3 or if_sm
  ),
  (
    opcode  : A_HADDPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#124#72;
    flags   : if_prescott or if_sse3 or if_sm
  ),
  (
    opcode  : A_HSUBPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#2#15#125#72;
    flags   : if_prescott or if_sse3 or if_sm
  ),
  (
    opcode  : A_HSUBPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#125#72;
    flags   : if_prescott or if_sse3 or if_sm
  ),
  (
    opcode  : A_LDDQU;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none,ot_none);
    code    : #220#2#15#240#72;
    flags   : if_prescott or if_sse3
  ),
  (
    opcode  : A_MOVDDUP;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#2#15#18#72;
    flags   : if_prescott or if_sse3
  ),
  (
    opcode  : A_MOVSHDUP;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#22#72;
    flags   : if_prescott or if_sse3 or if_sm
  ),
  (
    opcode  : A_MOVSLDUP;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#2#15#18#72;
    flags   : if_prescott or if_sse3 or if_sm
  ),
  (
    opcode  : A_VMREAD;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #2#15#120#65;
    flags   : if_386 or if_priv or if_prot
  ),
  (
    opcode  : A_VMREAD;
    ops     : 2;
    optypes : (ot_memory,ot_reg32,ot_none,ot_none);
    code    : #2#15#120#65;
    flags   : if_386 or if_priv or if_prot or if_sm
  ),
  (
    opcode  : A_VMWRITE;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #2#15#121#72;
    flags   : if_386 or if_priv or if_prot
  ),
  (
    opcode  : A_VMWRITE;
    ops     : 2;
    optypes : (ot_reg32,ot_memory,ot_none,ot_none);
    code    : #2#15#121#72;
    flags   : if_386 or if_priv or if_prot or if_sm
  ),
  (
    opcode  : A_VMCALL;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#193;
    flags   : if_386 or if_priv or if_prot
  ),
  (
    opcode  : A_VMLAUNCH;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#194;
    flags   : if_386 or if_priv or if_prot
  ),
  (
    opcode  : A_VMRESUME;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#195;
    flags   : if_386 or if_priv or if_prot
  ),
  (
    opcode  : A_VMXOFF;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#196;
    flags   : if_386 or if_priv or if_prot
  ),
  (
    opcode  : A_VMXON;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #219#2#15#199#134;
    flags   : if_priv or if_prot
  ),
  (
    opcode  : A_VMCLEAR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #241#2#15#199#134;
    flags   : if_priv or if_prot
  ),
  (
    opcode  : A_VMPTRLD;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#199#134;
    flags   : if_priv or if_prot
  ),
  (
    opcode  : A_VMPTRST;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none,ot_none);
    code    : #2#15#199#135;
    flags   : if_priv or if_prot
  ),
  (
    opcode  : A_VMRUN;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#216;
    flags   : if_386 or if_svm or if_priv or if_prot
  ),
  (
    opcode  : A_VMMCALL;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#217;
    flags   : if_386 or if_svm
  ),
  (
    opcode  : A_VMLOAD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#218;
    flags   : if_386 or if_svm or if_priv or if_prot
  ),
  (
    opcode  : A_VMSAVE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#219;
    flags   : if_386 or if_svm or if_priv or if_prot
  ),
  (
    opcode  : A_STGI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#220;
    flags   : if_386 or if_svm or if_priv or if_prot
  ),
  (
    opcode  : A_CLGI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#221;
    flags   : if_386 or if_svm or if_priv or if_prot
  ),
  (
    opcode  : A_SKINIT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#222;
    flags   : if_386 or if_svm or if_priv or if_prot
  ),
  (
    opcode  : A_INVLPGA;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#223;
    flags   : if_386 or if_svm or if_priv or if_prot
  ),
  (
    opcode  : A_MONTMUL;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #219#3#15#166#192;
    flags   : if_centaur
  ),
  (
    opcode  : A_XSHA1;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #219#3#15#166#200;
    flags   : if_centaur
  ),
  (
    opcode  : A_XSHA256;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #219#3#15#166#208;
    flags   : if_centaur
  ),
  (
    opcode  : A_DMINT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#57;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_RDM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #2#15#58;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_MOVNTSS;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #219#2#15#43#65;
    flags   : if_sse4 or if_sd
  ),
  (
    opcode  : A_MOVNTSD;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none,ot_none);
    code    : #220#213#2#15#43#65;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #220#2#15#121#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTQ;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate,ot_immediate);
    code    : #220#2#15#120#72#22#23;
    flags   : if_sse4 or if_sb
  ),
  (
    opcode  : A_EXTRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_immediate,ot_immediate,ot_none);
    code    : #241#2#15#120#128#21#22;
    flags   : if_sse4 or if_sb
  ),
  (
    opcode  : A_EXTRQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#2#15#121#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_LZCNT;
    ops     : 2;
    optypes : (ot_reg16,ot_rm_gpr,ot_none,ot_none);
    code    : #208#219#2#15#189#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_LZCNT;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #209#219#2#15#189#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#28#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#28#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#29#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#29#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#30#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#30#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_mmxrm,ot_immediate,ot_none);
    code    : #217#3#15#58#15#72#22;
    flags   : if_ssse3 or if_mmx or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#15#72#22;
    flags   : if_ssse3 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#1#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#1#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#2#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#2#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#3#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#3#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#5#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#5#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#6#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#6#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#7#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#7#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#4#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#4#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#11#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#11#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#0#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#0#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#8#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#8#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#9#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#9#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxrm,ot_none,ot_none);
    code    : #217#3#15#56#10#72;
    flags   : if_ssse3 or if_mmx or if_sm
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#10#72;
    flags   : if_ssse3 or if_sm
  ),
  (
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#12#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#13#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#20#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#21#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#64#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#65#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_memory,ot_xmmreg,ot_immediate,ot_none);
    code    : #241#213#3#15#58#23#65#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_immediate,ot_none);
    code    : #241#3#15#58#23#65#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#33#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_MOVNTDQA;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none,ot_none);
    code    : #241#3#15#56#42#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#66#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#43#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#16#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#14#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#41#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_immediate,ot_none);
    code    : #241#3#15#58#20#65#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_memory or ot_bits8,ot_xmmreg,ot_immediate,ot_none);
    code    : #241#3#15#58#20#65#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate,ot_none);
    code    : #241#3#15#58#22#65#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate,ot_none);
    code    : #241#3#15#58#22#65#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#65#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32 or ot_bits64,ot_immediate,ot_none);
    code    : #241#3#15#58#32#72#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits8,ot_immediate,ot_none);
    code    : #241#3#15#58#32#72#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate,ot_none);
    code    : #241#3#15#58#34#72#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_immediate,ot_none);
    code    : #241#3#15#58#34#72#22;
    flags   : if_sse41 or if_sb or if_ar2
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#60#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#61#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#63#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#62#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#56#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#57#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#58#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#59#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#32#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#33#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#34#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#35#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#36#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#37#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#48#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#49#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#50#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#51#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#52#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#53#72;
    flags   : if_sse41 or if_sm
  ),
  (
<<<<<<< HEAD
    opcode  : A_PMULDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#40#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PMULLD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#64#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_PTEST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#23#72;
    flags   : if_sse41 or if_sm
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#8#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#9#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#10#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
=======
    opcode  : A_POPA;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #215#1#97;
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_POPAD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#97;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_POPAW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #212#1#97;
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_POPF;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #215#1#157;
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_POPFD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#157;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_POPFW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #212#1#157;
    flags   : if_186 or if_nox86_64
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#11#72#22;
    flags   : if_sse41 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_CRC32;
    ops     : 2;
    optypes : (ot_reg32,ot_rm_gpr or ot_bits8,ot_none,ot_none);
    code    : #220#3#15#56#240#72;
    flags   : if_sse42
  ),
  (
    opcode  : A_CRC32;
    ops     : 2;
    optypes : (ot_reg32,ot_rm_gpr or ot_bits16 or ot_bits32,ot_none,ot_none);
    code    : #209#220#3#15#56#241#72;
    flags   : if_sse42
  ),
  (
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#97#72#22;
    flags   : if_sse42 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#96#72#22;
    flags   : if_sse42 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#99#72#22;
    flags   : if_sse42 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#98#72#22;
    flags   : if_sse42 or if_sm2 or if_sb or if_ar2
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#55#72;
    flags   : if_sse42 or if_sm
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg16,ot_rm_gpr or ot_bits16,ot_none,ot_none);
    code    : #219#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg32,ot_rm_gpr or ot_bits32,ot_none,ot_none);
    code    : #219#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#220#72;
    flags   : if_sse4 or if_sm
  ),
  (
    opcode  : A_AESENCLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#221#72;
    flags   : if_sse4 or if_sm
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#222#72;
    flags   : if_sse4 or if_sm
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#223#72;
    flags   : if_sse4 or if_sm
  ),
  (
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#3#15#56#219#72;
    flags   : if_sse4 or if_sm
  ),
  (
    opcode  : A_AESKEYGENASSIST;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate,ot_none);
    code    : #241#3#15#58#223#72#22;
    flags   : if_sse4 or if_sb or if_ar2
  ),
  (
    opcode  : A_RDTSCP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #3#15#1#249;
    flags   : if_sse4 or if_sm
  ),
  (
    opcode  : A_VADDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#88#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#88#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#88#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#88#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #220#242#248#1#88#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#88#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#88#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#88#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDSUBPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#208#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDSUBPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#208#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDSUBPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #220#242#244#248#1#208#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VADDSUBPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #220#242#248#1#208#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VAESDEC;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#222#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VAESDECLAST;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#223#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VAESENC;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#220#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VAESENCLAST;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#221#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VAESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#249#1#219#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VAESKEYGENASSIST;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#223#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VANDNPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#85#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VANDNPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#85#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VANDNPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#85#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VANDNPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#85#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VANDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#84#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VANDPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#84#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VANDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#84#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VANDPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#84#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBLENDPD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#13#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBLENDPD;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#13#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBLENDPS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#12#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBLENDPS;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#12#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBLENDVPD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_xmmreg);
    code    : #241#242#250#1#75#61#80#247;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBLENDVPD;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_ymmreg);
    code    : #241#242#244#250#1#75#61#80#247;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBLENDVPS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_xmmreg);
    code    : #241#242#250#1#74#61#80#247;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBLENDVPS;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_ymmreg);
    code    : #241#242#244#250#1#74#61#80#247;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBROADCASTF128;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits128,ot_none,ot_none);
    code    : #241#242#244#249#1#26#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBROADCASTSD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#244#249#1#25#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBROADCASTSD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#25#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VBROADCASTSS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#244#249#1#24#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBROADCASTSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#249#1#24#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VBROADCASTSS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#24#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VBROADCASTSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#24#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VCMPPD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#248#1#194#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCMPPD;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#248#1#194#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCMPPS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #242#248#1#194#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCMPPS;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #242#244#248#1#194#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCMPSD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_immediate or ot_bits8);
    code    : #220#242#248#1#194#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCMPSD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8);
    code    : #220#242#248#1#194#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCMPSS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_immediate or ot_bits8);
    code    : #219#242#248#1#194#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCMPSS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8);
    code    : #219#242#248#1#194#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCOMISD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#248#1#47#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCOMISD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#47#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCOMISS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #242#248#1#47#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCOMISS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#47#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTDQ2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #219#242#248#1#230#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTDQ2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #219#242#248#1#230#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTDQ2PD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #219#242#244#248#1#230#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTDQ2PD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits128,ot_none,ot_none);
    code    : #219#242#244#248#1#230#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTDQ2PS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #242#248#1#91#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTDQ2PS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #242#244#248#1#91#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPD2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #220#242#248#1#230#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPD2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_ymmrm,ot_none,ot_none);
    code    : #220#242#244#248#1#230#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPD2PS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#248#1#90#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPD2PS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#248#1#90#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPS2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#248#1#91#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPS2DQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#248#1#91#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPS2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #242#248#1#90#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPS2PD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#90#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTPS2PD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmrm,ot_none,ot_none);
    code    : #242#244#248#1#90#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSD2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#242#248#1#45#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSD2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #220#242#248#1#45#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSD2SI;
    ops     : 2;
    optypes : (ot_reg64,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#242#243#248#1#45#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSD2SI;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none,ot_none);
    code    : #220#242#243#248#1#45#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSD2SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #220#242#248#1#90#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSD2SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#90#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSI2SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_rm_gpr or ot_bits32,ot_none);
    code    : #220#242#248#1#42#61#80;
    flags   : if_avx or if_sandybridge or if_sd
  ),
  (
    opcode  : A_VCVTSI2SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_rm_gpr or ot_bits64,ot_none);
    code    : #220#242#243#248#1#42#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSI2SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_rm_gpr or ot_bits32,ot_none);
    code    : #219#242#248#1#42#61#80;
    flags   : if_avx or if_sandybridge or if_sd
  ),
  (
    opcode  : A_VCVTSI2SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_rm_gpr or ot_bits64,ot_none);
    code    : #219#242#243#248#1#42#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSS2SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#90#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSS2SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#90#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSS2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #219#242#248#1#45#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSS2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #219#242#248#1#45#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSS2SI;
    ops     : 2;
    optypes : (ot_reg64,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #219#242#243#248#1#45#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTSS2SI;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none,ot_none);
    code    : #219#242#243#248#1#45#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTPD2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#248#1#230#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTPD2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#248#1#230#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTPS2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#242#248#1#91#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTPS2DQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #219#242#244#248#1#91#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTSD2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#242#248#1#44#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTSD2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #220#242#248#1#44#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTSD2SI;
    ops     : 2;
    optypes : (ot_reg64,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#242#243#248#1#44#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTSD2SI;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none,ot_none);
    code    : #220#242#243#248#1#44#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTSS2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #219#242#248#1#44#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTSS2SI;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #219#242#248#1#44#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTSS2SI;
    ops     : 2;
    optypes : (ot_reg64,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #219#242#243#248#1#44#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VCVTTSS2SI;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none,ot_none);
    code    : #219#242#243#248#1#44#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDIVPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#94#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDIVPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#94#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDIVPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#94#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDIVPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#94#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDIVSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #220#242#248#1#94#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDIVSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#94#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDIVSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#94#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDIVSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#94#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDPPD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#65#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDPPS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#64#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VDPPS;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#64#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VEXTRACTF128;
    ops     : 3;
    optypes : (ot_xmmrm,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#250#1#25#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VEXTRACTPS;
    ops     : 3;
    optypes : (ot_rm_gpr or ot_bits32,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#23#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VHADDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#124#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VHADDPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#124#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VHADDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #220#242#248#1#124#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VHADDPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #220#242#244#248#1#124#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VHSUBPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#125#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VHSUBPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#125#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VHSUBPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #220#242#248#1#125#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VHSUBPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #220#242#244#248#1#125#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VINSERTF128;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#24#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VINSERTPS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_immediate or ot_bits8);
    code    : #241#242#250#1#33#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VINSERTPS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8);
    code    : #241#242#250#1#33#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
<<<<<<< HEAD
    opcode  : A_VLDDQU;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits128,ot_none,ot_none);
    code    : #220#242#248#1#240#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VLDDQU;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits256,ot_none,ot_none);
    code    : #220#242#244#248#1#240#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VLDMXCSR;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #242#248#1#174#130;
    flags   : if_avx or if_sandybridge
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    opcode  : A_PUSHA;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #215#1#96;
    flags   : if_186 or if_nox86_64
<<<<<<< HEAD
<<<<<<< HEAD
  ),
  (
    opcode  : A_PUSHAD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#96;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSHAW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #212#1#96;
    flags   : if_186 or if_nox86_64
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_VMASKMOVDQU;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#247#72;
    flags   : if_avx or if_sandybridge
  ),
  (
<<<<<<< HEAD
    opcode  : A_VMASKMOVPD;
    ops     : 3;
    optypes : (ot_memory or ot_bits256,ot_ymmreg,ot_ymmreg,ot_none);
    code    : #241#242#244#249#1#47#61#66;
    flags   : if_avx or if_sandybridge
=======
    opcode  : A_PUSHFD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#156;
    flags   : if_386 or if_nox86_64
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_VMASKMOVPD;
    ops     : 3;
    optypes : (ot_memory or ot_bits128,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #241#242#249#1#47#61#66;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMASKMOVPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_memory or ot_bits256,ot_none);
    code    : #241#242#244#249#1#45#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMASKMOVPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits128,ot_none);
    code    : #241#242#249#1#45#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMASKMOVPS;
    ops     : 3;
    optypes : (ot_memory or ot_bits256,ot_ymmreg,ot_ymmreg,ot_none);
    code    : #241#242#244#249#1#46#61#66;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMASKMOVPS;
    ops     : 3;
    optypes : (ot_memory or ot_bits128,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #241#242#249#1#46#61#66;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMASKMOVPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_memory or ot_bits256,ot_none);
    code    : #241#242#244#249#1#44#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMASKMOVPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits128,ot_none);
    code    : #241#242#249#1#44#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMAXPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#95#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMAXPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#95#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMAXPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#95#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMAXPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#95#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMAXSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #220#242#248#1#95#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMAXSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#95#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMAXSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#95#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMAXSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#95#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMINPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#93#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMINPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#93#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMINPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#93#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMINPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#93#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMINSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #220#242#248#1#93#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMINSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#93#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMINSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#93#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMINSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#93#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVAPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#248#1#40#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVAPD;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#41#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVAPD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#248#1#40#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVAPD;
    ops     : 2;
    optypes : (ot_ymmrm,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#41#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVAPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #242#248#1#40#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVAPS;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#41#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVAPS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #242#244#248#1#40#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVAPS;
    ops     : 2;
    optypes : (ot_ymmrm,ot_ymmreg,ot_none,ot_none);
    code    : #242#244#248#1#41#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_rm_gpr or ot_bits32,ot_none,ot_none);
    code    : #241#242#248#1#110#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVD;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits32,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#126#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDDUP;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #220#242#244#248#1#18#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDDUP;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#242#248#1#18#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDDUP;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #220#242#248#1#18#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDQA;
    ops     : 2;
    optypes : (ot_ymmrm,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#127#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDQA;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#248#1#111#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDQA;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#127#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDQA;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#248#1#111#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDQU;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #219#242#244#248#1#111#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDQU;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#242#248#1#111#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDQU;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #219#242#248#1#127#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVDQU;
    ops     : 2;
    optypes : (ot_ymmrm,ot_ymmreg,ot_none,ot_none);
    code    : #219#242#244#248#1#127#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVHLPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #242#248#1#18#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVHPD;
    ops     : 2;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#23#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVHPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #241#242#248#1#22#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVHPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #242#248#1#22#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVHPS;
    ops     : 2;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#23#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVLHPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #242#248#1#22#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVLPD;
    ops     : 2;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#19#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVLPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #241#242#248#1#18#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVLPS;
    ops     : 2;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#19#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVLPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #242#248#1#18#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVMSKPD;
    ops     : 2;
    optypes : (ot_reg32,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#80#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVMSKPD;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#80#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVMSKPD;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#80#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVMSKPD;
    ops     : 2;
    optypes : (ot_reg64,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#80#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVMSKPS;
    ops     : 2;
    optypes : (ot_reg32,ot_ymmreg,ot_none,ot_none);
    code    : #242#244#248#1#80#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVMSKPS;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#80#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVMSKPS;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#80#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVMSKPS;
    ops     : 2;
    optypes : (ot_reg64,ot_ymmreg,ot_none,ot_none);
    code    : #242#244#248#1#80#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVNTDQ;
    ops     : 2;
    optypes : (ot_memory or ot_bits128,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#231#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVNTDQ;
    ops     : 2;
    optypes : (ot_memory or ot_bits256,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#231#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVNTDQA;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits128,ot_none,ot_none);
    code    : #241#242#249#1#42#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVNTDQA;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits256,ot_none,ot_none);
    code    : #241#242#244#249#1#42#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VMOVNTPD;
    ops     : 2;
    optypes : (ot_memory or ot_bits256,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#43#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVNTPD;
    ops     : 2;
    optypes : (ot_memory or ot_bits128,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#43#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVNTPS;
    ops     : 2;
    optypes : (ot_memory or ot_bits128,ot_ymmreg,ot_none,ot_none);
    code    : #242#244#248#1#43#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVNTPS;
    ops     : 2;
    optypes : (ot_memory or ot_bits128,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#43#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVQ;
    ops     : 2;
    optypes : (ot_rm_gpr or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#243#248#1#126#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_rm_gpr or ot_bits64,ot_none,ot_none);
    code    : #241#242#243#248#1#110#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#16#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #220#242#248#1#16#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#17#61#66;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSD;
    ops     : 2;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #220#242#248#1#17#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSHDUP;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#242#248#1#22#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSHDUP;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #219#242#244#248#1#22#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSLDUP;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #219#242#248#1#18#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSLDUP;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #219#242#244#248#1#18#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#16#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #219#242#248#1#16#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#17#61#66;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVSS;
    ops     : 2;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_none,ot_none);
    code    : #219#242#248#1#17#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVUPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#248#1#16#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVUPD;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#17#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVUPD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#248#1#16#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVUPD;
    ops     : 2;
    optypes : (ot_ymmrm,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#17#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVUPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #242#248#1#16#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVUPS;
    ops     : 2;
    optypes : (ot_xmmrm,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#17#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVUPS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #242#244#248#1#16#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMOVUPS;
    ops     : 2;
    optypes : (ot_ymmrm,ot_ymmreg,ot_none,ot_none);
    code    : #242#244#248#1#17#65;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMPSADBW;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#66#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMPSADBW;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#66#61#80#23;
    flags   : if_avx2
  ),
  (
    opcode  : A_VMULPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#89#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMULPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#89#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMULPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#89#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMULPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#89#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMULSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #220#242#248#1#89#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMULSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#89#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMULSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#89#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VMULSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#89#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VORPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#86#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VORPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#86#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VORPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#86#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#249#1#28#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPABSB;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#28#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#249#1#30#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPABSD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#30#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPABSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#249#1#29#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPABSW;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#29#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPACKSSDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#107#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPACKSSDW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#107#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPACKSSWB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#99#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPACKSSWB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#99#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPACKUSDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#43#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPACKUSDW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#43#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPACKUSWB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#103#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPACKUSWB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#103#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPADDB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#252#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPADDB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#252#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPADDD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#254#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPADDD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#254#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPADDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#212#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPADDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#212#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPADDSB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#236#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPADDSB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#236#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPADDSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#237#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPADDSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#237#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPADDUSB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#220#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPADDUSB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#220#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPADDUSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#221#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPADDUSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#221#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPADDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#253#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPADDW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#253#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPALIGNR;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#15#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPALIGNR;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#15#61#80#23;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPAND;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#219#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPAND;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#219#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPANDN;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#223#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPANDN;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#223#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPAVGB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#224#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPAVGB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#224#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPAVGW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#227#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPAVGW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#227#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBLENDVB;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_xmmreg);
    code    : #241#242#250#1#76#61#80#247;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPBLENDVB;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_ymmreg);
    code    : #241#242#244#250#1#76#61#80#247;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBLENDW;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#14#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPBLENDW;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#14#61#80#23;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCLMULQDQ;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#68#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPEQB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#116#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPEQB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#116#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCMPEQD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#118#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPEQD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#118#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCMPEQQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#41#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPEQQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#41#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCMPEQW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#117#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPEQW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#117#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#97#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#96#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPGTB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#100#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPGTB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#100#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCMPGTD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#102#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPGTD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#102#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCMPGTQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#55#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPGTQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#55#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCMPGTW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#101#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPGTW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#101#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#99#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#98#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPERM2F128;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#6#61#80#23;
    flags   : if_avx or if_sandybridge
=======
  ),
  (
    opcode  : A_PUSHAD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#96;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSHAW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #212#1#96;
    flags   : if_186 or if_nox86_64
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_VPERMILPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#13#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
<<<<<<< HEAD
    opcode  : A_VPERMILPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#5#72#22;
    flags   : if_avx or if_sandybridge
=======
    opcode  : A_PUSHFD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#156;
    flags   : if_386 or if_nox86_64
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_VPERMILPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#250#1#5#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPERMILPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#13#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPERMILPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#12#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPERMILPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#4#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPERMILPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#250#1#4#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPERMILPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#12#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRB;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#20#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRB;
    ops     : 3;
    optypes : (ot_reg64,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#20#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRB;
    ops     : 3;
    optypes : (ot_memory or ot_bits8,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#20#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRD;
    ops     : 3;
    optypes : (ot_rm_gpr or ot_bits32,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#22#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRQ;
    ops     : 3;
    optypes : (ot_rm_gpr or ot_bits64,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#243#250#1#22#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#197#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRW;
    ops     : 3;
    optypes : (ot_reg64,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#197#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#21#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRW;
    ops     : 3;
    optypes : (ot_reg64,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#21#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPEXTRW;
    ops     : 3;
    optypes : (ot_memory or ot_bits16,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#21#65#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPHADDD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#2#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPHADDD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#2#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPHADDSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#3#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPHADDSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#3#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPHADDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#1#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPHADDW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#1#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#249#1#65#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPHSUBD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#6#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPHSUBD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#6#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPHSUBSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#7#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPHSUBSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#7#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPHSUBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#5#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPHSUBW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#5#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPINSRB;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_reg32,ot_immediate or ot_bits8);
    code    : #241#242#250#1#32#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPINSRB;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits8,ot_immediate or ot_bits8);
    code    : #241#242#250#1#32#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPINSRD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_rm_gpr or ot_bits32,ot_immediate or ot_bits8);
    code    : #241#242#250#1#34#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPINSRQ;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_rm_gpr or ot_bits64,ot_immediate or ot_bits8);
    code    : #241#242#243#250#1#34#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPINSRW;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_reg32,ot_immediate or ot_bits8);
    code    : #241#242#248#1#196#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPINSRW;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits16,ot_immediate or ot_bits8);
    code    : #241#242#248#1#196#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMADDUBSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#4#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMADDUBSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#4#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMADDWD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#245#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMADDWD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#245#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMAXSB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#60#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMAXSB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#60#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMAXSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#61#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMAXSD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#61#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMAXSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#238#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMAXSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#238#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMAXUB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#222#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMAXUB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#222#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMAXUD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#63#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMAXUD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#63#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMAXUW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#62#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMAXUW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#62#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMINSB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#56#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMINSB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#56#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMINSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#57#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMINSD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#57#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMINSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#234#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMINSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#234#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMINUB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#218#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMINUB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#218#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMINUD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#59#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMINUD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#59#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMINUW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#58#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMINUW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#58#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVMSKB;
    ops     : 2;
    optypes : (ot_reg64,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#215#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVMSKB;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#215#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVMSKB;
    ops     : 2;
    optypes : (ot_reg32,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#215#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVMSKB;
    ops     : 2;
    optypes : (ot_reg64,ot_ymmreg,ot_none,ot_none);
    code    : #241#242#244#248#1#215#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#249#1#33#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#33#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXBD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#33#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXBD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#244#249#1#33#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits16,ot_none,ot_none);
    code    : #241#242#249#1#34#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#34#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXBQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#244#249#1#34#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXBQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#34#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#249#1#32#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#32#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXBW;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#32#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#249#1#37#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#37#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXDQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#37#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#249#1#35#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#35#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXWD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#35#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#249#1#36#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#36#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVSXWQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#244#249#1#36#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVSXWQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#36#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#249#1#49#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#49#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXBD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#244#249#1#49#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXBD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#49#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits16,ot_none,ot_none);
    code    : #241#242#249#1#50#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#50#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXBQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#244#249#1#50#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXBQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#50#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#249#1#48#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#48#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXBW;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#48#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#249#1#53#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#53#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXDQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#53#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#249#1#51#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#51#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXWD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#51#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#249#1#52#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#52#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMOVZXWQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#244#249#1#52#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMOVZXWQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#52#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMULDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#40#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMULDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#40#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMULHRSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#11#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMULHRSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#11#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMULHUW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#228#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMULHUW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#228#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMULHW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#229#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMULHW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#229#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMULLD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#64#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMULLD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#64#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMULLW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#213#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMULLW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#213#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMULUDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#244#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPMULUDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#244#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPOR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#235#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPOR;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#235#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#246#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSADBW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#246#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSHUFB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#0#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSHUFB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#0#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSHUFD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#112#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSHUFD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#112#72#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSHUFHW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #219#242#248#1#112#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSHUFHW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #219#242#244#248#1#112#72#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSHUFLW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #220#242#248#1#112#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSHUFLW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #220#242#244#248#1#112#72#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSIGNB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#8#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSIGNB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#8#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSIGND;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#10#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSIGND;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#10#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSIGNW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#9#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSIGNW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#9#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#114#60#142#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSLLD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#242#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSLLD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#114#60#142#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_none);
    code    : #241#242#244#248#1#242#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#115#60#143#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSLLDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#115#60#143#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#243#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSLLQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#115#60#142#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSLLQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_none);
    code    : #241#242#244#248#1#243#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#115#60#142#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#113#60#142#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSLLW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#241#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSLLW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#113#60#142#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_none);
    code    : #241#242#244#248#1#241#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRAD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#114#60#140#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRAD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#226#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRAD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#114#60#140#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRAD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_none);
    code    : #241#242#244#248#1#226#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRAW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#113#60#140#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRAW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#225#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRAW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#113#60#140#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRAW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_none);
    code    : #241#242#244#248#1#225#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#114#60#138#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRLD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#210#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRLD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#114#60#138#22;
    flags   : if_avx2
  ),
  (
<<<<<<< HEAD
    opcode  : A_VPSRLD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_none);
    code    : #241#242#244#248#1#210#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#115#60#139#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRLDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#115#60#139#22;
    flags   : if_avx2
=======
    opcode  : A_PUSHA;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #215#1#96;
    flags   : if_186 or if_nox86_64
  ),
  (
    opcode  : A_PUSHAD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#96;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSHAW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #212#1#96;
    flags   : if_186 or if_nox86_64
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_VPSRLQ;
    ops     : 3;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#115#60#138#22;
    flags   : if_avx or if_sandybridge
=======
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#2#15#197#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#21#65#22;
    flags   : if_sse4
>>>>>>> graemeg/cpstrnew
=======
    opcode  : A_PUSHFD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#156;
    flags   : if_386 or if_nox86_64
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_VPSRLQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#211#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRLQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#115#60#138#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_none);
    code    : #241#242#244#248#1#211#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#248#1#113#60#138#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRLW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#209#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSRLW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#248#1#113#60#138#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_none);
    code    : #241#242#244#248#1#209#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSUBB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#248#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSUBB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#248#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSUBD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#250#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSUBD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#250#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSUBQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#251#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSUBQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#251#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSUBSB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#232#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSUBSB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#232#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSUBSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#233#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSUBSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#233#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSUBUSB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#216#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSUBUSB;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#216#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSUBUSW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#217#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSUBUSW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#217#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSUBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#249#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPSUBW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#249#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPTEST;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#23#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPTEST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#249#1#23#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKHBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#104#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKHBW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#104#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPUNPCKHDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#106#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKHDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#106#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPUNPCKHQDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#109#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKHQDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#109#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPUNPCKHWD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#105#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKHWD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#105#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPUNPCKLBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#96#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKLBW;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#96#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPUNPCKLDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#98#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKLDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#98#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPUNPCKLQDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#108#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKLQDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#108#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPUNPCKLWD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#97#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPUNPCKLWD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#97#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPXOR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#239#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VPXOR;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#239#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VRCPPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #242#248#1#83#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VRCPPS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #242#244#248#1#83#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VRCPSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#83#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VRCPSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#83#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VROUNDPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#250#1#9#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#9#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VROUNDPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#250#1#8#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#250#1#8#72#22;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VROUNDSD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_immediate or ot_bits8);
    code    : #241#242#250#1#11#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VROUNDSD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8);
    code    : #241#242#250#1#11#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VROUNDSS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_immediate or ot_bits8);
    code    : #241#242#250#1#10#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VROUNDSS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_immediate or ot_bits8);
    code    : #241#242#250#1#10#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VRSQRTPS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #242#244#248#1#82#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VRSQRTPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #242#248#1#82#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VRSQRTSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#82#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VRSQRTSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#82#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSHUFPD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#248#1#198#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSHUFPD;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#248#1#198#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSHUFPS;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #242#248#1#198#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSHUFPS;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #242#244#248#1#198#61#80#23;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSQRTPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#248#1#81#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSQRTPD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#248#1#81#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSQRTPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #242#248#1#81#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSQRTPS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #242#244#248#1#81#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSQRTSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #220#242#248#1#81#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSQRTSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#81#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSQRTSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#81#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSQRTSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#81#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSTMXCSR;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none);
    code    : #242#248#1#174#131;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSUBPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#92#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSUBPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#92#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSUBPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#92#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSUBPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#92#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSUBSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits64,ot_none);
    code    : #220#242#248#1#92#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSUBSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #220#242#248#1#92#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSUBSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits32,ot_none);
    code    : #219#242#248#1#92#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VSUBSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#242#248#1#92#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VTESTPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#249#1#15#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VTESTPD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#15#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VTESTPS;
    ops     : 2;
    optypes : (ot_ymmreg,ot_ymmrm,ot_none,ot_none);
    code    : #241#242#244#249#1#14#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VTESTPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmrm,ot_none,ot_none);
    code    : #241#242#249#1#14#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUCOMISD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#248#1#46#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUCOMISD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#248#1#46#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUCOMISS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #242#248#1#46#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUCOMISS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #242#248#1#46#72;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUNPCKHPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#21#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUNPCKHPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#21#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUNPCKHPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#21#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUNPCKHPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#21#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUNPCKLPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#20#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUNPCKLPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#20#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUNPCKLPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#20#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VUNPCKLPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#20#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VXORPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#248#1#87#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VXORPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#248#1#87#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VXORPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #242#248#1#87#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VXORPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #242#244#248#1#87#61#80;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VZEROALL;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #242#244#248#1#119;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_VZEROUPPER;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none);
    code    : #242#248#1#119;
    flags   : if_avx or if_sandybridge
  ),
  (
    opcode  : A_ANDN;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_rm_gpr or ot_bits32,ot_none);
    code    : #242#249#1#242#61#80;
    flags   : if_bmi1
  ),
  (
    opcode  : A_BEXTR;
    ops     : 3;
    optypes : (ot_reg32,ot_rm_gpr or ot_bits32,ot_reg32,ot_none);
    code    : #242#249#1#247#62#72;
    flags   : if_bmi1
  ),
  (
    opcode  : A_TZCNT;
    ops     : 2;
    optypes : (ot_reg16 or ot_bits32 or ot_bits64,ot_rm_gpr,ot_none,ot_none);
    code    : #208#219#2#15#188#72;
    flags   : if_bmi1 or if_sm
  ),
  (
    opcode  : A_RORX;
    ops     : 3;
    optypes : (ot_reg32,ot_rm_gpr or ot_bits32,ot_immediate or ot_bits8,ot_none);
    code    : #220#242#250#1#240#72#22;
    flags   : if_bmi2
  ),
  (
    opcode  : A_SARX;
    ops     : 3;
    optypes : (ot_reg32,ot_rm_gpr or ot_bits32,ot_reg32,ot_none);
    code    : #219#242#249#1#247#62#72;
    flags   : if_bmi2
  ),
  (
    opcode  : A_SHLX;
    ops     : 3;
    optypes : (ot_reg32,ot_rm_gpr or ot_bits32,ot_reg32,ot_none);
    code    : #241#242#249#1#247#62#72;
    flags   : if_bmi2
  ),
  (
    opcode  : A_SHRX;
    ops     : 3;
    optypes : (ot_reg32,ot_rm_gpr or ot_bits32,ot_reg32,ot_none);
    code    : #220#242#249#1#247#62#72;
    flags   : if_bmi2
  ),
  (
    opcode  : A_VBROADCASTI128;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits128,ot_none,ot_none);
    code    : #241#242#244#249#1#90#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VEXTRACTI128;
    ops     : 3;
    optypes : (ot_xmmrm,ot_ymmreg,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#244#250#1#57#65#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VINSERTI128;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#56#61#80#23;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBLENDD;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#2#61#80#23;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBLENDD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_immediate or ot_bits8);
    code    : #241#242#250#1#2#61#80#23;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#120#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTB;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#120#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits8,ot_none,ot_none);
    code    : #241#242#249#1#120#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTB;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits8,ot_none,ot_none);
    code    : #241#242#244#249#1#120#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#88#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#249#1#88#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#88#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTD;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits32,ot_none,ot_none);
    code    : #241#242#244#249#1#88#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#244#249#1#89#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_none,ot_none);
    code    : #241#242#249#1#89#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#89#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTQ;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#89#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory or ot_bits16,ot_none,ot_none);
    code    : #241#242#249#1#121#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#249#1#121#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTW;
    ops     : 2;
    optypes : (ot_ymmreg,ot_memory or ot_bits16,ot_none,ot_none);
    code    : #241#242#244#249#1#121#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPBROADCASTW;
    ops     : 2;
    optypes : (ot_ymmreg,ot_xmmreg,ot_none,ot_none);
    code    : #241#242#244#249#1#121#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPERM2I128;
    ops     : 4;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8);
    code    : #241#242#244#250#1#70#61#80#23;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPERMD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#54#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPERMPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#243#244#250#1#1#72#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPERMPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#22#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPERMQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmrm,ot_immediate or ot_bits8,ot_none);
    code    : #241#242#243#244#250#1#0#72#22;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMASKMOVD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits128,ot_none);
    code    : #241#242#249#1#140#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMASKMOVD;
    ops     : 3;
    optypes : (ot_memory or ot_bits128,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #241#242#249#1#142#61#66;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMASKMOVD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_memory or ot_bits256,ot_none);
    code    : #241#242#244#249#1#140#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMASKMOVD;
    ops     : 3;
    optypes : (ot_memory or ot_bits256,ot_ymmreg,ot_ymmreg,ot_none);
    code    : #241#242#244#249#1#142#61#66;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMASKMOVQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_memory or ot_bits128,ot_none);
    code    : #241#242#243#249#1#140#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMASKMOVQ;
    ops     : 3;
    optypes : (ot_memory or ot_bits128,ot_xmmreg,ot_xmmreg,ot_none);
    code    : #241#242#243#249#1#142#61#66;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMASKMOVQ;
    ops     : 3;
    optypes : (ot_memory or ot_bits256,ot_ymmreg,ot_ymmreg,ot_none);
    code    : #241#242#243#244#249#1#142#61#66;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPMASKMOVQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_memory or ot_bits256,ot_none);
    code    : #241#242#243#244#249#1#140#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLVD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#71#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLVD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#71#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLVQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#243#249#1#71#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSLLVQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#243#244#249#1#71#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRAVD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#70#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRAVD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#70#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLVD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#69#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLVD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#69#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLVQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#243#244#249#1#69#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPSRLVQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#243#249#1#69#61#80;
    flags   : if_avx2
  ),
  (
    opcode  : A_VGATHERDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmem64,ot_xmmreg,ot_none);
    code    : #241#242#243#249#1#146#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VGATHERDPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_xmem64,ot_ymmreg,ot_none);
    code    : #241#242#243#244#249#1#146#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VGATHERDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmem32,ot_xmmreg,ot_none);
    code    : #241#242#249#1#146#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VGATHERDPS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymem32,ot_ymmreg,ot_none);
    code    : #241#242#244#249#1#146#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VGATHERQPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmem64,ot_xmmreg,ot_none);
    code    : #241#242#243#249#1#147#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VGATHERQPD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymem64,ot_ymmreg,ot_none);
    code    : #241#242#243#244#249#1#147#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VGATHERQPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmem32,ot_xmmreg,ot_none);
    code    : #241#242#249#1#147#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VGATHERQPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_ymem32,ot_xmmreg,ot_none);
    code    : #241#242#244#249#1#147#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPGATHERDD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmem32,ot_xmmreg,ot_none);
    code    : #241#242#249#1#144#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPGATHERDD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymem32,ot_ymmreg,ot_none);
    code    : #241#242#244#249#1#144#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPGATHERDQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmem64,ot_xmmreg,ot_none);
    code    : #241#242#243#249#1#144#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPGATHERDQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_xmem64,ot_ymmreg,ot_none);
    code    : #241#242#243#244#249#1#144#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPGATHERQD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmem32,ot_xmmreg,ot_none);
    code    : #241#242#249#1#145#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPGATHERQD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_ymem32,ot_xmmreg,ot_none);
    code    : #241#242#244#249#1#145#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPGATHERQQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmem64,ot_xmmreg,ot_none);
    code    : #241#242#243#249#1#145#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VPGATHERQQ;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymem64,ot_ymmreg,ot_none);
    code    : #241#242#243#244#249#1#145#62#72;
    flags   : if_avx2
  ),
  (
    opcode  : A_VFMADD132PD;
    ops     : 3;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#152#61#80;
    flags   : if_fma
=======
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#2#15#197#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#21#65#22;
    flags   : if_sse4
>>>>>>> graemeg/cpstrnew
  ),
  (
    opcode  : A_VFMADD132PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#152#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD213PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#168#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD213PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#168#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD231PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#184#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD231PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#184#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDPD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_xmmreg);
    code    : #241#242#250#1#105#61#80#247;
    flags   : if_fma4
  ),
  (
    opcode  : A_VFMADDPD;
    ops     : 4;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmreg,ot_xmmrm);
    code    : #241#242#250#243#1#105#61#88#246;
    flags   : if_fma4
  ),
  (
    opcode  : A_VFMADD132PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#152#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD132PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#152#61#80;
    flags   : if_fma
=======
  ),
  (
    opcode  : A_PUSHAD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#96;
    flags   : if_386 or if_nox86_64
  ),
  (
    opcode  : A_PUSHAW;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #212#1#96;
    flags   : if_186 or if_nox86_64
>>>>>>> origin/cpstrnew
  ),
  (
    opcode  : A_VFMADD213PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#168#61#80;
    flags   : if_fma
  ),
  (
<<<<<<< HEAD
    opcode  : A_VFMADD213PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#168#61#80;
    flags   : if_fma
=======
    opcode  : A_PUSHFD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #213#1#156;
    flags   : if_386 or if_nox86_64
>>>>>>> origin/cpstrnew
  ),
  (
    opcode  : A_VFMADD231PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#184#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD231PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#184#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD132SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#153#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD213SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#169#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD231SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#185#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD132SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#153#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD213SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#169#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADD231SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#185#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB132PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#150#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB132PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#150#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB213PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#166#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB213PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#166#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB231PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#182#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB231PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#182#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB132PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#150#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB132PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#150#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB213PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#166#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB213PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#166#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB231PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#182#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMADDSUB231PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#182#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD132PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#151#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD132PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#151#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD213PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#167#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD213PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#167#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD231PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#183#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD231PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#183#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD132PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#151#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD132PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#151#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD213PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#167#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD213PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#167#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD231PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#183#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUBADD231PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#183#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB132PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#154#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB132PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#154#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB213PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#170#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB213PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#170#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB231PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#186#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB231PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#186#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB132PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#154#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB132PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#154#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB213PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#170#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB213PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#170#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB231PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#186#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB231PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#186#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB132SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#155#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB213SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#171#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB231SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#187#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB132SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#155#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB213SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#171#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFMSUB231SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#187#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD132PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#156#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD132PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#156#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD213PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#172#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD213PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#172#61#80;
    flags   : if_fma
  ),
  (
<<<<<<< HEAD
    opcode  : A_VFNMADD231PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#188#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD231PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#188#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD132PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#156#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD132PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#156#61#80;
    flags   : if_fma
=======
    opcode  : A_PSLLD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#242#72;
    flags   : if_willamette or if_sse2 or if_sm
=======
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#115#135#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> origin/fixes_2_2
  ),
  (
    opcode  : A_PSLLD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#242#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PSLLD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#114#134#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
  ),
  (
    opcode  : A_PSLLDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#115#135#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_VFNMADD213PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#172#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD213PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#172#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD231PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#188#61#80;
    flags   : if_fma
  ),
  (
<<<<<<< HEAD
    opcode  : A_VFNMADD231PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#188#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD132SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#157#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD213SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#173#61#80;
    flags   : if_fma
=======
    opcode  : A_PSLLQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#243#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSLLQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#243#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PSLLQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#115#134#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_VFNMADD231SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#189#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD132SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#157#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMADD213SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#173#61#80;
    flags   : if_fma
  ),
  (
<<<<<<< HEAD
    opcode  : A_VFNMADD231SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#189#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB132PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#158#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB132PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#158#61#80;
    flags   : if_fma
=======
    opcode  : A_PSLLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#241#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSLLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#241#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PSLLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#113#134#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_VFNMSUB213PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#174#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB213PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#174#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB231PD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#190#61#80;
    flags   : if_fma
  ),
  (
<<<<<<< HEAD
    opcode  : A_VFNMSUB231PD;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#243#1#190#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB132PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#158#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB132PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#158#61#80;
    flags   : if_fma
=======
    opcode  : A_PSRAD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#226#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSRAD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#226#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PSRAD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#114#132#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_VFNMSUB213PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#174#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB213PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#174#61#80;
    flags   : if_fma
  ),
  (
<<<<<<< HEAD
    opcode  : A_VFNMSUB231PS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#190#61#80;
    flags   : if_fma
=======
    opcode  : A_MOVNTSD;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#220#213#211#2#15#43#65;
    flags   : if_sse4
>>>>>>> graemeg/cpstrnew
  ),
  (
<<<<<<< HEAD
    opcode  : A_VFNMSUB231PS;
    ops     : 3;
    optypes : (ot_ymmreg,ot_ymmreg,ot_ymmrm,ot_none);
    code    : #241#242#244#249#1#190#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB132SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#159#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB213SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#175#61#80;
    flags   : if_fma
=======
    opcode  : A_PSRAW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#225#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSRAW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#225#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PSRAW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#113#132#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_VFNMSUB231SD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#243#1#191#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB132SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#159#61#80;
    flags   : if_fma
  ),
  (
<<<<<<< HEAD
    opcode  : A_VFNMSUB213SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#175#61#80;
    flags   : if_fma
  ),
  (
    opcode  : A_VFNMSUB231SS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_xmmrm,ot_none);
    code    : #241#242#249#1#191#61#80;
    flags   : if_fma
=======
    opcode  : A_PABSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#28#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#210#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#28#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#210#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#28#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#114#130#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#28#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#29#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#29#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#29#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#211#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#29#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#211#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#30#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#115#130#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_mmxreg,ot_immediate);
    code    : #217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_memory,ot_immediate);
    code    : #193#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#15#72#22;
    flags   : if_sse4
=======
    opcode  : A_PSRLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#209#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSRLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#209#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PSRLW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#113#130#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#1#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#248#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#1#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#248#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#2#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#250#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#2#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#250#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#3#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#232#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#3#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#232#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#5#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#233#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#6#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#233#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#6#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#6#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#6#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#216#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#7#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#216#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#7#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#217#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#4#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#217#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#4#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#4#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#4#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#249#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#11#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#249#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#11#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#104#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#0#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#104#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#0#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#106#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#8#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#106#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#8#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#105#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#9#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#105#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#9#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#9#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#9#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#96#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#10#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#96#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#10#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#98#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg or ot_signed,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#12#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg or ot_signed,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#12#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#2#15#58#253#1#13#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#2#15#58#253#1#13#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#20#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#98#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#20#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#21#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#21#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#64#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#97#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PUNPCKLWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#97#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#64#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#65#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#65#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_memory,ot_xmmreg,ot_immediate);
    code    : #1#102#213#3#15#58#23#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_immediate);
    code    : #1#102#192#3#15#58#23#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#33#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#33#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_MOVNTDQA;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#42#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#66#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#66#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#43#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#43#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#16#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#16#72;
=======
    code    : #1#102#193#211#2#15#227#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_mmxreg,ot_immediate);
    code    : #2#15#197#72#18;
    flags   : if_katmai or if_mmx or if_sb or if_ar2
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#2#15#197#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#21#65#22;
>>>>>>> graemeg/cpstrnew
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_signed);
    code    : #1#102#217#3#15#58#14#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate or ot_signed);
    code    : #1#102#193#217#3#15#58#14#72#22;
    flags   : if_sse4
=======
    opcode  : A_PXOR;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#239#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#41#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#239#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#41#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#3#15#58#20#65#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_memory or ot_bits8,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#20#65#22;
    flags   : if_sse4
=======
    opcode  : A_RCL;
    ops     : 2;
    optypes : (ot_regmem or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none);
    code    : #192#208#1#193#130#21;
    flags   : if_8086 or if_sb
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRQ;
    ops     : 3;
    optypes : (ot_reg64,ot_xmmreg,ot_immediate);
    code    : #1#102#214#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRQ;
    ops     : 3;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_immediate);
    code    : #1#102#192#214#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#65#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#65#72;
    flags   : if_sse4
=======
    optypes : (ot_regmem or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none);
    code    : #192#208#1#193#131#21;
    flags   : if_8086 or if_sb
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#217#3#15#58#32#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits8,ot_immediate);
    code    : #1#102#193#217#3#15#58#32#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#217#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_immediate);
    code    : #1#102#193#217#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg64,ot_immediate);
    code    : #1#102#214#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_immediate);
    code    : #1#102#193#214#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#60#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#60#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#61#72;
    flags   : if_sse4
=======
    optypes : (ot_regmem or ot_bits16 or ot_bits32 or ot_bits64,ot_immediate,ot_none);
    code    : #192#208#1#193#131#21;
    flags   : if_8086 or if_sb
>>>>>>> origin/fixes_2_2
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#61#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#63#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#63#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#62#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#62#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#56#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#56#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#57#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#57#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#58#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#58#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#59#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#59#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#32#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#32#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#33#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#33#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#34#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#34#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#35#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#35#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#36#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#36#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#37#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#37#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#48#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#48#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#49#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#49#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#50#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#50#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#51#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#51#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#52#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#52#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#53#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#53#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#40#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#40#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PTEST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#23#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#8#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#8#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#9#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#9#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#10#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#10#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#11#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#11#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#97#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#97#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#96#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#96#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#99#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#99#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#98#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#98#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#55#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#55#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg16,ot_regmem or ot_bits16,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg32,ot_regmem or ot_bits32,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg64,ot_regmem or ot_bits64,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#220#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#220#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESENCLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#221#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESENCLAST;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#221#72;
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#220#213#211#2#15#43#65;
>>>>>>> graemeg/cpstrnew
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#222#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#222#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#223#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#223#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#219#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#219#72;
    flags   : if_sse4
  ),
  (
=======
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#28#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#28#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#28#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#28#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#29#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#29#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#29#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#29#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_mmxreg,ot_immediate);
    code    : #217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_memory,ot_immediate);
    code    : #193#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#6#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#6#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#6#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#6#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#4#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#4#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#4#72;
    flags   : if_sse4
=======
    optypes : (ot_regmem or ot_bits8,ot_reg8,ot_none);
    code    : #193#211#1#132#65;
    flags   : if_8086 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#4#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#0#72;
    flags   : if_sse4
=======
    optypes : (ot_regmem or ot_bits8,ot_reg8,ot_none);
    code    : #193#211#1#132#65;
    flags   : if_8086 or if_sm
>>>>>>> origin/fixes_2_2
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#9#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#9#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#9#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#9#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg or ot_signed,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#12#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg or ot_signed,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#12#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#2#15#58#253#1#13#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#2#15#58#253#1#13#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#20#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#20#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#21#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#21#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#64#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#64#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#65#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#65#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_memory,ot_xmmreg,ot_immediate);
    code    : #1#102#213#3#15#58#23#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_immediate);
    code    : #1#102#192#3#15#58#23#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#33#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#33#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_MOVNTDQA;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#42#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#66#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#66#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#43#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#43#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#16#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#16#72;
=======
    code    : #1#102#193#211#2#15#227#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_mmxreg,ot_immediate);
    code    : #2#15#197#72#18;
    flags   : if_katmai or if_mmx or if_sb or if_ar2
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#2#15#197#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#21#65#22;
>>>>>>> origin/cpstrnew
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_signed);
    code    : #1#102#217#3#15#58#14#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate or ot_signed);
    code    : #1#102#193#217#3#15#58#14#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#41#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#41#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#3#15#58#20#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_memory or ot_bits8,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#20#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRQ;
    ops     : 3;
    optypes : (ot_reg64,ot_xmmreg,ot_immediate);
    code    : #1#102#214#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRQ;
    ops     : 3;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_immediate);
    code    : #1#102#192#214#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#65#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#65#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#217#3#15#58#32#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits8,ot_immediate);
    code    : #1#102#193#217#3#15#58#32#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#217#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_immediate);
    code    : #1#102#193#217#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg64,ot_immediate);
    code    : #1#102#214#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_immediate);
    code    : #1#102#193#214#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#60#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#60#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#61#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#61#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#63#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#63#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#88#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#62#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#62#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#85#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#56#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#2#15#85#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#56#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#84#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#57#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#2#15#84#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#57#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#58#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#1#0;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#58#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#1#0;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#59#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#1#0;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#59#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#32#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#1#2;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#32#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#1#2;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#33#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#1#2;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#33#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#34#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#1#1;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#34#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#1#1;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#35#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#1#1;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#35#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#36#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#1#4;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#36#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#1#4;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#37#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#1#4;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#37#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#48#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#1#6;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#48#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#1#6;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#49#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#1#6;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#49#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#50#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#1#5;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#50#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#1#5;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#51#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#1#5;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#51#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#52#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#1#7;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#52#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#1#7;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#53#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#1#7;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#53#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#40#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#1#3;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#40#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#1#3;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PTEST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#23#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#1#3;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#8#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#8#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#58#9#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#194#72#18;
    flags   : if_katmai or if_sse or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#58#9#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#194#72#18;
    flags   : if_katmai or if_sse or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#58#10#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#10#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#11#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#194#72#18;
    flags   : if_katmai or if_sse or if_sb or if_ar2
  ),
  (
    opcode  : A_COMISS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #193#211#2#15#47#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_COMISS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#2#15#47#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#11#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#97#72#22;
    flags   : if_sse4
=======
    opcode  : A_CVTPI2PS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_mmxreg,ot_none);
    code    : #211#217#2#15#42#72;
    flags   : if_katmai or if_sse or if_mmx
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#97#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#96#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#96#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#99#72#22;
    flags   : if_sse4
=======
    opcode  : A_CVTPS2PI;
    ops     : 2;
    optypes : (ot_mmxreg,ot_xmmreg,ot_none);
    code    : #211#217#2#15#45#72;
    flags   : if_katmai or if_sse or if_mmx
  ),
  (
    opcode  : A_CVTSI2SS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #219#193#209#2#15#42#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_CVTSI2SS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_reg32 or ot_bits64,ot_none);
    code    : #219#211#209#2#15#42#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#99#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#98#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#98#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#55#72;
    flags   : if_sse4
=======
    optypes : (ot_reg32 or ot_bits64,ot_memory,ot_none);
    code    : #219#193#208#2#15#45#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#55#72;
    flags   : if_sse4
=======
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_none);
    code    : #219#211#208#2#15#45#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg16,ot_regmem or ot_bits16,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg32,ot_regmem or ot_bits32,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
=======
    optypes : (ot_mmxreg,ot_xmmreg,ot_none);
    code    : #211#217#2#15#44#72;
    flags   : if_katmai or if_sse or if_mmx
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg64,ot_regmem or ot_bits64,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
=======
    optypes : (ot_reg32 or ot_bits64,ot_memory,ot_none);
    code    : #219#193#208#2#15#44#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#220#72;
    flags   : if_sse4
=======
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_none);
    code    : #219#211#208#2#15#44#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#220#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESENCLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#221#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#94#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESENCLAST;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#221#72;
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#220#213#211#2#15#43#65;
>>>>>>> graemeg/cpstrnew
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#94#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#222#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#94#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_LDMXCSR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #192#211#2#15#174#130;
    flags   : if_katmai or if_sse or if_sd
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#222#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#223#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#95#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#223#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#2#15#95#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#219#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#95#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
<<<<<<< HEAD
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#219#72;
    flags   : if_sse4
  ),
  (
>>>>>>> graemeg/cpstrnew
=======
    opcode  : A_PABSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#28#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#217#2#15#93#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#28#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #219#193#2#15#93#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#28#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#93#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#28#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#40#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#29#72;
    flags   : if_sse4
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#211#2#15#41#65;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#29#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#2#15#40#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#29#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#2#15#41#65;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#29#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#22#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#30#72;
    flags   : if_sse4
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#211#2#15#23#65;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#30#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#2#15#22#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#30#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#18#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_mmxreg,ot_immediate);
    code    : #217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_memory,ot_immediate);
    code    : #193#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#1#72;
    flags   : if_sse4
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#211#2#15#19#65;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#1#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#2#15#18#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#1#72;
    flags   : if_sse4
=======
    optypes : (ot_reg32,ot_xmmreg,ot_none);
    code    : #211#2#15#80#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#2#72;
    flags   : if_sse4
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #211#2#15#43#65;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#2#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#211#2#15#16#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#3#72;
    flags   : if_sse4
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #219#192#211#2#15#17#65;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#3#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#211#2#15#16#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#3#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#17#65;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#5#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#217#2#15#16#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#5#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#17#65;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#5#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#89#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#6#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#2#15#89#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#6#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #219#193#211#2#15#89#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#6#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#89#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#6#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#86#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#7#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#2#15#86#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#7#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#83#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#7#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#211#2#15#83#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#4#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#211#2#15#83#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#4#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#4#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#217#2#15#82#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#4#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#211#2#15#82#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#11#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#82#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_SHUFPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #193#211#2#15#198#72#18;
    flags   : if_katmai or if_sse or if_sb or if_ar2
  ),
  (
    opcode  : A_SHUFPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #211#2#15#198#72#18;
    flags   : if_katmai or if_sse or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#0#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#217#2#15#81#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#0#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #219#193#2#15#81#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#0#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#81#72;
    flags   : if_katmai or if_sse
  ),
  (
    opcode  : A_STMXCSR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #192#211#2#15#174#131;
    flags   : if_katmai or if_sse or if_sd
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#8#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#217#2#15#92#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#8#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #219#193#211#2#15#92#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#8#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#92#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#8#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#46#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#9#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#2#15#46#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#9#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #193#211#2#15#21#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#9#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#2#15#21#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#9#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #193#211#2#15#20#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#10#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#2#15#20#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#10#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #193#211#2#15#87#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#10#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#2#15#87#72;
    flags   : if_katmai or if_sse
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg or ot_signed,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#12#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg or ot_signed,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#12#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#2#15#58#253#1#13#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#2#15#58#253#1#13#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#20#72;
    flags   : if_sse4
=======
    opcode  : A_PREFETCHNTA;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #192#211#2#15#24#128;
    flags   : if_katmai
  ),
  (
    opcode  : A_PREFETCHT0;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #192#211#2#15#24#129;
    flags   : if_katmai
  ),
  (
    opcode  : A_PREFETCHT1;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #192#211#2#15#24#130;
    flags   : if_katmai
  ),
  (
    opcode  : A_PREFETCHT2;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #192#211#2#15#24#131;
    flags   : if_katmai
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#20#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#21#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#21#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#64#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#64#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#65#72#22;
    flags   : if_sse4
=======
    opcode  : A_PAVGB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#224#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#65#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_memory,ot_xmmreg,ot_immediate);
    code    : #1#102#213#3#15#58#23#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_immediate);
    code    : #1#102#192#3#15#58#23#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#33#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#33#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_MOVNTDQA;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#42#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#211#2#15#224#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#66#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#66#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#43#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#227#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#43#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#211#2#15#227#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#16#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#16#72;
    flags   : if_sse4
=======
    opcode  : A_PEXTRW;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#2#15#197#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_signed);
    code    : #1#102#217#3#15#58#14#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate or ot_signed);
    code    : #1#102#193#217#3#15#58#14#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#41#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#41#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#3#15#58#20#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRB;
    ops     : 3;
<<<<<<< HEAD
    optypes : (ot_memory or ot_bits8,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#20#65#22;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#211#2#15#196#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
<<<<<<< HEAD
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#3#15#58#22#65#22;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#2#15#196#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
<<<<<<< HEAD
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#22#65#22;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory or ot_bits16,ot_immediate);
    code    : #1#102#193#2#15#196#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PEXTRQ;
    ops     : 3;
    optypes : (ot_reg64,ot_xmmreg,ot_immediate);
    code    : #1#102#214#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRQ;
    ops     : 3;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_immediate);
    code    : #1#102#192#214#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#65#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#238#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#65#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#238#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#217#3#15#58#32#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits8,ot_immediate);
    code    : #1#102#193#217#3#15#58#32#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#217#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_immediate);
    code    : #1#102#193#217#3#15#58#34#72#22;
    flags   : if_sse4
=======
    opcode  : A_PMAXUB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#222#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PMAXUB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#2#15#222#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PINSRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg64,ot_immediate);
    code    : #1#102#214#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_immediate);
    code    : #1#102#193#214#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#60#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#234#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#60#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#234#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#61#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#61#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#63#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#218#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#63#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#218#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#62#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#62#72;
    flags   : if_sse4
=======
    optypes : (ot_reg32,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#215#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#56#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#56#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#57#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#228#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#57#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#228#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#58#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#58#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#59#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#246#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#59#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#2#15#246#72;
    flags   : if_willamette or if_sse2 or if_sm
  ),
  (
    opcode  : A_PSHUFW;
    ops     : 3;
    optypes : (ot_mmxreg,ot_mmxreg,ot_immediate);
    code    : #2#15#112#72#18;
    flags   : if_katmai or if_mmx or if_sb or if_ar2
  ),
  (
    opcode  : A_PSHUFW;
    ops     : 3;
    optypes : (ot_mmxreg,ot_memory,ot_immediate);
    code    : #193#2#15#112#72#18;
    flags   : if_katmai or if_mmx or if_sm2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#32#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#32#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#33#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#33#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#34#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#34#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#35#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#35#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#36#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#36#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#37#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#37#72;
    flags   : if_sse4
=======
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#247#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CLFLUSH;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #192#211#2#15#174#135;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVNTDQ;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #1#102#192#211#2#15#231#65;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#48#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#48#72;
    flags   : if_sse4
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #1#102#192#211#2#15#43#65;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#49#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#49#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#50#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#50#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#111#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#51#72;
    flags   : if_sse4
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #1#102#192#211#2#15#127#65;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#51#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#211#2#15#111#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#52#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#127#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#52#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#211#2#15#111#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#53#72;
    flags   : if_sse4
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #219#192#211#2#15#127#65;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#53#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#211#2#15#111#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#40#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#127#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULDQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#40#72;
    flags   : if_sse4
=======
    optypes : (ot_mmxreg,ot_xmmreg,ot_none);
    code    : #1#242#211#2#15#214#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PTEST;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#23#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_mmxreg,ot_none);
    code    : #219#211#2#15#214#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#8#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#8#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#9#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#9#72#22;
    flags   : if_sse4
=======
    opcode  : A_PADDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#212#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PADDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#211#2#15#212#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#10#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#10#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#11#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#11#72#22;
    flags   : if_sse4
=======
    opcode  : A_PMULUDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#244#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_PMULUDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#211#2#15#244#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#58#97#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#112#72#18;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#58#97#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#211#2#15#112#72#18;
    flags   : if_willamette or if_sse2 or if_sm2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#58#96#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#112#72#18;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#58#96#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#211#2#15#112#72#18;
    flags   : if_willamette or if_sse2 or if_sm2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#58#99#72#22;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#112#72#18;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#99#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#98#72#22;
    flags   : if_sse4
=======
    opcode  : A_PSRLDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_immediate,ot_none);
    code    : #1#102#211#2#15#115#131#21;
    flags   : if_willamette or if_sse2 or if_sb or if_ar1
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#98#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#55#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#55#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#251#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg16,ot_regmem or ot_bits16,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#211#2#15#251#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg32,ot_regmem or ot_bits32,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#109#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_reg64,ot_regmem or ot_bits64,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#211#2#15#109#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#220#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#108#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#220#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#211#2#15#108#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESENCLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#221#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#217#2#15#88#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESENCLAST;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#221#72;
=======
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #192#220#213#211#2#15#43#65;
>>>>>>> origin/cpstrnew
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#217#2#15#88#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#222#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#217#2#15#88#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#222#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#193#217#2#15#88#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#223#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#217#2#15#85#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#223#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#217#2#15#85#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#219#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#217#2#15#84#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_ANDPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#2#15#84#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
<<<<<<< HEAD
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#219#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#217#2#15#194#72#1#0;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
>>>>>>> graemeg/cpstrnew
=======
    opcode  : A_PABSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#28#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#217#2#15#194#72#1#0;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#28#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#242#193#217#2#15#194#72#1#0;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#28#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#217#2#15#194#72#1#0;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#28#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#217#2#15#194#72#1#2;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#29#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#217#2#15#194#72#1#2;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#29#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#242#193#217#2#15#194#72#1#2;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#29#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#217#2#15#194#72#1#2;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#29#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#193#217#2#15#194#72#1#1;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#30#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#217#2#15#194#72#1#1;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#30#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#242#193#217#2#15#194#72#1#1;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#30#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#217#2#15#194#72#1#1;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PABSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#30#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_mmxreg,ot_immediate);
    code    : #217#3#15#58#15#72#22;
    flags   : if_sse4
=======
    opcode  : A_CMPNEQPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#217#2#15#194#72#1#4;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_mmxreg,ot_memory,ot_immediate);
    code    : #193#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#15#72#22;
    flags   : if_sse4
=======
    opcode  : A_CMPNEQPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#242#211#217#2#15#194#72#1#4;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PALIGNR;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#15#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#1#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#217#2#15#194#72#1#6;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#1#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#217#2#15#194#72#1#6;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#1#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#2#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#217#2#15#194#72#1#5;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#2#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#217#2#15#194#72#1#5;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#2#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#3#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#217#2#15#194#72#1#7;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#3#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#217#2#15#194#72#1#7;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHADDSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#3#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#5#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#217#2#15#194#72#1#3;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#5#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#217#2#15#194#72#1#3;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CMPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#2#15#194#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#5#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#6#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#47#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#6#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#211#2#15#47#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#6#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#6#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#211#2#15#230#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#7#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #211#2#15#91#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#7#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#230#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PHSUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#7#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#4#72;
    flags   : if_sse4
=======
    optypes : (ot_mmxreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#45#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#4#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#4#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#90#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMADDUBSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#4#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#11#72;
    flags   : if_sse4
=======
    optypes : (ot_xmmreg,ot_mmxreg,ot_none);
    code    : #1#102#211#2#15#42#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#11#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#91#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULHRSW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#11#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#0#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#2#15#90#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSHUFB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#0#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#8#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#90#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#8#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#9#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#9#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#9#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#211#2#15#90#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGNW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#9#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #219#193#211#2#15#90#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
<<<<<<< HEAD
    optypes : (ot_mmxreg,ot_mmxreg,ot_none);
    code    : #217#3#15#56#10#72;
    flags   : if_sse4
=======
    optypes : (ot_mmxreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#44#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_mmxreg,ot_memory,ot_none);
    code    : #193#217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#10#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#230#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PSIGND;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#10#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg or ot_signed,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#12#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPS;
    ops     : 3;
    optypes : (ot_xmmreg or ot_signed,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#12#72#22;
    flags   : if_sse4
=======
    opcode  : A_CVTTPS2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #219#211#2#15#91#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_CVTTPS2DQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #219#193#2#15#91#72;
    flags   : if_willamette or if_sse2 or if_sm
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#2#15#58#253#1#13#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#2#15#58#253#1#13#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#20#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#94#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_BLENDVPS;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#20#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#21#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#94#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_BLENDVPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#21#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#64#72#22;
    flags   : if_sse4
=======
    opcode  : A_MAXPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#95#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_DPPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#64#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#65#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_DPPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#65#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_memory,ot_xmmreg,ot_immediate);
    code    : #1#102#213#3#15#58#23#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRACTPS;
    ops     : 3;
    optypes : (ot_reg32 or ot_bits64,ot_xmmreg,ot_immediate);
    code    : #1#102#192#3#15#58#23#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#33#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#33#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_MOVNTDQA;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#42#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#66#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_MPSADBW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#66#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#43#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#95#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PACKUSDW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#43#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#16#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#93#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PBLENDVB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#16#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate or ot_signed);
    code    : #1#102#217#3#15#58#14#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PBLENDW;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate or ot_signed);
    code    : #1#102#193#217#3#15#58#14#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#41#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#93#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPEQQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#41#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#3#15#58#20#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRB;
    ops     : 3;
    optypes : (ot_memory or ot_bits8,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#20#65#22;
    flags   : if_sse4
=======
    opcode  : A_MOVAPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#40#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVAPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#41#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
    optypes : (ot_reg32,ot_xmmreg,ot_immediate);
    code    : #1#102#211#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRD;
    ops     : 3;
    optypes : (ot_memory or ot_bits32,ot_xmmreg,ot_immediate);
    code    : #1#102#192#211#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRQ;
    ops     : 3;
    optypes : (ot_reg64,ot_xmmreg,ot_immediate);
    code    : #1#102#214#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PEXTRQ;
    ops     : 3;
    optypes : (ot_memory or ot_bits64,ot_xmmreg,ot_immediate);
    code    : #1#102#192#214#3#15#58#22#65#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#65#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PHMINPOSUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#65#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#217#3#15#58#32#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRB;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits8,ot_immediate);
    code    : #1#102#193#217#3#15#58#32#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg32,ot_immediate);
    code    : #1#102#217#3#15#58#34#72#22;
    flags   : if_sse4
=======
    opcode  : A_MOVMSKPD;
    ops     : 2;
    optypes : (ot_reg32,ot_xmmreg,ot_none);
    code    : #211#1#102#211#2#15#80#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVUPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#16#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MOVUPD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#211#2#15#17#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PINSRD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits32,ot_immediate);
    code    : #1#102#193#217#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_reg64,ot_immediate);
    code    : #1#102#214#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PINSRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory or ot_bits64,ot_immediate);
    code    : #1#102#193#214#3#15#58#34#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#60#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#89#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#60#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#61#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#89#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_MULSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #193#1#242#211#2#15#89#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#61#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#63#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#86#72;
    flags   : if_willamette or if_sse2
  ),
  (
    opcode  : A_SHUFPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#211#2#15#198#72#22;
    flags   : if_willamette or if_sse2 or if_sb or if_ar2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#63#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#62#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#81#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMAXUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#62#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#56#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#81#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINSB;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#56#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#57#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#92#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINSD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#57#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#58#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#92#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINUW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#58#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#59#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#46#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMINUD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#59#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#32#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#21#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#32#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#33#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#20#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#33#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#34#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#87#72;
    flags   : if_willamette or if_sse2
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#34#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#35#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#35#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#36#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#208#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#36#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#37#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#208#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVSXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#37#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#48#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#124#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBW;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#48#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#49#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#124#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#49#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#50#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#102#211#2#15#125#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXBQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#50#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#51#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#125#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWD;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#193#217#3#15#56#51#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#1#242#211#2#15#240#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#52#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXWQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#52#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#53#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #1#242#211#2#15#18#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMOVZXDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#53#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PMULDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#40#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#3#243#15#22#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PMULDQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#40#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PTEST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
<<<<<<< HEAD
<<<<<<< HEAD
    code    : #1#102#217#3#15#56#23#72;
    flags   : if_sse4
=======
=======
>>>>>>> origin/fixes_2_2
    code    : #211#3#243#15#18#72;
    flags   : if_prescott or if_sse3
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#8#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#8#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#9#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDPD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#9#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#10#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSS;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#10#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#11#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_ROUNDSD;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#11#72#22;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#97#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPESTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#97#72#22;
    flags   : if_sse4
=======
    opcode  : A_VMXON;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #219#192#2#15#199#134;
    flags   : if_priv or if_prot
  ),
  (
    opcode  : A_VMCLEAR;
    ops     : 1;
    optypes : (ot_memory,ot_none,ot_none);
    code    : #1#102#192#2#15#199#134;
    flags   : if_priv or if_prot
>>>>>>> graemeg/fixes_2_2
  ),
  (
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#96#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPESTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#96#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#99#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRI;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#99#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#98#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPISTRM;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#98#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#55#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_PCMPGTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#55#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg16,ot_regmem or ot_bits16,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg32,ot_regmem or ot_bits32,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg64,ot_regmem or ot_bits64,ot_none);
    code    : #1#243#193#208#2#15#184#72;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#220#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESENC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#220#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESENCLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#221#72;
    flags   : if_sse4
  ),
  (
<<<<<<< HEAD
    opcode  : A_AESENCLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#221#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#222#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDEC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#222#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#223#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESDECLAST;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#223#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #1#102#217#3#15#56#219#72;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESIMC;
    ops     : 2;
    optypes : (ot_xmmreg,ot_memory,ot_none);
    code    : #1#102#193#217#3#15#56#219#72;
    flags   : if_sse4
  ),
  (
>>>>>>> origin/cpstrnew
    opcode  : A_AESKEYGEN;
    ops     : 3;
    optypes : (ot_xmmreg,ot_xmmreg,ot_immediate);
    code    : #1#102#217#3#15#58#223#72#22;
    flags   : if_sse4
  ),
  (
    opcode  : A_AESKEYGEN;
    ops     : 3;
    optypes : (ot_xmmreg,ot_memory,ot_immediate);
    code    : #1#102#193#217#3#15#58#223#72#22;
    flags   : if_sse4
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
=======
    opcode  : A_RDM;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none);
    code    : #2#15#58;
    flags   : if_p6 or if_cyrix
  ),
  (
    opcode  : A_MOVNTSS;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #65#219#192#62#2#15#43#63;
    flags   : if_sse4 or if_sd
  ),
  (
    opcode  : A_MOVNTSD;
    ops     : 2;
    optypes : (ot_memory,ot_xmmreg,ot_none);
    code    : #65#192#220#213#62#2#15#43#63;
    flags   : if_sse4
  ),
  (
    opcode  : A_INSERTQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #72#220#62#2#15#121#63;
    flags   : if_sse4
  ),
  (
    opcode  : A_EXTRQ;
    ops     : 3;
    optypes : (ot_xmmreg,ot_immediate,ot_immediate);
    code    : #128#222#62#2#15#120#63#253#17#253#18;
    flags   : if_sse4 or if_sb
  ),
  (
    opcode  : A_EXTRQ;
    ops     : 2;
    optypes : (ot_xmmreg,ot_xmmreg,ot_none);
    code    : #72#222#62#2#15#121#63;
    flags   : if_sse4
  ),
  (
    opcode  : A_LZCNT;
    ops     : 2;
    optypes : (ot_reg16,ot_regmem,ot_none);
    code    : #72#208#219#193#62#2#15#189#63;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_LZCNT;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_regmem,ot_none);
    code    : #72#209#219#193#62#2#15#189#63;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg16,ot_regmem,ot_none);
    code    : #72#208#219#193#62#2#15#184#63;
    flags   : if_386 or if_sm or if_sse4
  ),
  (
    opcode  : A_POPCNT;
    ops     : 2;
    optypes : (ot_reg32 or ot_bits64,ot_regmem,ot_none);
    code    : #72#209#219#193#62#2#15#184#63;
    flags   : if_386 or if_sm or if_sse4
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  )
);
