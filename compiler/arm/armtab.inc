{ don't edit, this file is generated from armins.dat }
(
  (
    opcode  : A_NONE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #0;
    flags   : if_none
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#65#64;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#241#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ADC;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#235#64#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#235#64#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#241#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ADC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#235#64#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#235#64#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #4#0#160;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ADC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#0#160;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ADC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#2#160;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #97#68#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_reglo,ot_none,ot_none,ot_none);
    code    : #96#24#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #96#28#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #96#28#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #107#48#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reglo,ot_regsp,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #100#168#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_regsp,ot_regsp,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #100#176#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reg32,ot_regsp,ot_reg32,ot_none,ot_none,ot_none);
    code    : #100#68#104;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_regsp,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #100#68#133;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#241#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADD;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#235#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#235#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#241#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#235#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#235#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #4#0#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ADD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#0#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#2#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ADDW;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #129#242#0#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ADF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ADF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ADR;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #103#160#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam6,ot_none,ot_none,ot_none,ot_none);
    code    : #103#160#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ADR;
    ops     : 2;
    optypes : (ot_reg32,ot_immediate or ot_bits32,ot_none,ot_none,ot_none,ot_none);
    code    : #129#242#175#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADR;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #129#242#175#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADR;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #129#242#175#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ADR;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #51#2#15;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#64#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#0#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_AND;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_AND;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#234#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_AND;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#240#0#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_AND;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#234#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_AND;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#234#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_AND;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #4#0#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_AND;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#0#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_AND;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#2#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_B;
    ops     : 1;
    optypes : (ot_immediate24,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #98#224#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_B;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #98#224#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_B;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #98#224#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_B;
    ops     : 1;
    optypes : (ot_immediate24,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #99#208#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_B;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #99#208#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_B;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #99#208#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_B;
    ops     : 1;
    optypes : (ot_immediate24,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #1#10;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_B;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #1#10;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BIC;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#67#128;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_BIC;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#32#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BIC;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#32#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_BIC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#234#32#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_BIC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#240#32#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BIC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#234#32#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_BIC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#234#32#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_BIC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #6#1#192;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BIC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#1#192;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BIC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#3#192;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BL;
    ops     : 1;
    optypes : (ot_immediate24,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #141#240#208;
    flags   : if_thumb or if_thumb32 or if_armv4t
  ),
  (
    opcode  : A_BL;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #141#240#208;
    flags   : if_thumb or if_thumb32 or if_armv4t
  ),
  (
    opcode  : A_BL;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #141#240#208;
    flags   : if_thumb or if_thumb32 or if_armv4t
  ),
  (
    opcode  : A_BL;
    ops     : 1;
    optypes : (ot_immediate24,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #1#11;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BL;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #1#11;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BLX;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #98#71#128;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_BLX;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #141#240#192;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BLX;
    ops     : 1;
    optypes : (ot_immediate24,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #141#240#192;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BLX;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #141#240#192;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BLX;
    ops     : 1;
    optypes : (ot_immediate24,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #40#250;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_BLX;
    ops     : 1;
    optypes : (ot_memory or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #40#250;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_BLX;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #3#1#47#255#48;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_BKPT;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #96#190#0;
    flags   : if_thumb or if_armv5t
  ),
  (
    opcode  : A_BKPT;
    ops     : 1;
    optypes : (ot_immediate,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #49#1#32#112;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_BKPT;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #49#1#32#112;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_BX;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #98#71#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_BX;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #3#1#47#255#16;
    flags   : if_arm32 or if_armv4t
  ),
  (
    opcode  : A_CDP;
    ops     : 2;
    optypes : (ot_reg8,ot_reg8,ot_none,ot_none,ot_none,ot_none);
    code    : #192#1#16#65;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_CMN;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #111#66#192;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_CMN;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#241#16#15#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_CMN;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#235#16#15#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CMN;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#235#16#15#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CMN;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #12#1#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_CMN;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #14#1#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_CMN;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #15#1#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #111#66#128;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #97#69#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #111#40#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#241#176#15#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#235#176#15#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CMP;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#235#176#15#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #12#1#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_CMP;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #14#1#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_CMP;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #15#3#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_CMF;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#144;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_CMF;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#144;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_CMFE;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#192;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_CMFE;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#192;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_STF;
    ops     : 2;
    optypes : (ot_fpureg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #160#12#0#1#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_LDF;
    ops     : 2;
    optypes : (ot_fpureg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #160#12#16#1#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_LFM;
    ops     : 3;
    optypes : (ot_fpureg,ot_immediate or ot_bits32,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #160#12#16#2#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_LFM;
    ops     : 3;
    optypes : (ot_fpureg,ot_immediateshifter,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #160#12#16#2#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_CLZ;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#176#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_CLZ;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #50#1#111#15#16;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_CPS;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #143#243#175#129#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_CPS;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #70#241#2#0#0;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_CPSID;
    ops     : 1;
    optypes : (ot_modeflags,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #108#182#112;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_CPSID;
    ops     : 1;
    optypes : (ot_modeflags,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #143#243#175#134#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CPSID;
    ops     : 2;
    optypes : (ot_modeflags,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #143#243#175#135#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CPSID;
    ops     : 1;
    optypes : (ot_modeflags,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #70#241#12#0#0;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_CPSID;
    ops     : 2;
    optypes : (ot_modeflags,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #70#241#14#0#0;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_CPSIE;
    ops     : 1;
    optypes : (ot_modeflags,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #108#182#96;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_CPSIE;
    ops     : 1;
    optypes : (ot_modeflags,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #143#243#175#132#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CPSIE;
    ops     : 2;
    optypes : (ot_modeflags,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #143#243#175#133#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_CPSIE;
    ops     : 1;
    optypes : (ot_modeflags,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #70#241#8#0#0;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_CPSIE;
    ops     : 2;
    optypes : (ot_modeflags,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #70#241#10#0#0;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_EOR;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#64#64;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_EOR;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#128#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_EOR;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#128#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_EOR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#234#128#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_EOR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#240#128#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_EOR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#234#128#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_EOR;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#234#128#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_EOR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #4#0#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_EOR;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#0#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_EOR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#2#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDC;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #209#192#1#17#65;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #105#200;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDM;
    ops     : 2;
    optypes : (ot_reglo,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #105#200;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #140#232#16#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LDM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #140#232#16#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LDM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #38#129;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #38#129;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRBT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#16#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDRBT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#112;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRBT;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#112;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRB;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam3,ot_none,ot_none,ot_none,ot_none);
    code    : #101#92#0#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDRB;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam4,ot_none,ot_none,ot_none,ot_none);
    code    : #102#120#0#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDRB;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#16#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LDRB;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#80;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam3,ot_none,ot_none,ot_none,ot_none);
    code    : #101#88#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam4,ot_none,ot_none,ot_none,ot_none);
    code    : #102#104#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam5,ot_none,ot_none,ot_none,ot_none);
    code    : #103#152#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #103#152#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam6,ot_none,ot_none,ot_none,ot_none);
    code    : #103#72#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDR;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#80#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LDR;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#16;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRH;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam3,ot_none,ot_none,ot_none,ot_none);
    code    : #101#90#0#1;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDRH;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam4,ot_none,ot_none,ot_none,ot_none);
    code    : #102#136#0#1;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDRH;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#48#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LDRH;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #34#16#176;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRSB;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam3,ot_none,ot_none,ot_none,ot_none);
    code    : #101#86#0#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDRSB;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#249#16#0#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDRSB;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #34#16#208;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRSB;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #35#80#208;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRSB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediate or ot_bits32,ot_none,ot_none,ot_none);
    code    : #36#80#208;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRSB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #37#16#208;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRSH;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam3,ot_none,ot_none,ot_none,ot_none);
    code    : #101#94#0#1;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LDRSH;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#249#48#0#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDRSH;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #34#16#240;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#80#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDRT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#48;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MCR;
    ops     : 5;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_regf,ot_regf,ot_none);
    code    : #28#14#0#1;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MCR;
    ops     : 6;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_regf,ot_regf,ot_immediateshifter);
    code    : #28#14#0#1;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MCR2;
    ops     : 5;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_regf,ot_regf,ot_none);
    code    : #28#254#0#1;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_MCR2;
    ops     : 6;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_regf,ot_regf,ot_immediateshifter);
    code    : #28#254#0#1;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_MRC;
    ops     : 5;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_regf,ot_regf,ot_none);
    code    : #28#14#16#1;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MRC;
    ops     : 6;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_regf,ot_regf,ot_immediateshifter);
    code    : #28#14#16#1;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MRC2;
    ops     : 5;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_regf,ot_regf,ot_none);
    code    : #28#254#16#1;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_MRC2;
    ops     : 6;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_regf,ot_regf,ot_immediateshifter);
    code    : #28#254#16#1;
    flags   : if_arm32 or if_armv5t
  ),
  (
    opcode  : A_MCRR;
    ops     : 5;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_reg32,ot_regf,ot_none);
    code    : #29#12#64#0;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_MCRR2;
    ops     : 5;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_reg32,ot_regf,ot_none);
    code    : #29#252#64#0;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_MRRC;
    ops     : 5;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_reg32,ot_regf,ot_none);
    code    : #29#12#80#0;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_MRRC2;
    ops     : 5;
    optypes : (ot_regf,ot_immediateshifter,ot_reg32,ot_reg32,ot_regf,ot_none);
    code    : #29#252#80#0;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_MLA;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#0#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MLA;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#0#32#9;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#0#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #97#70#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #107#32#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#79#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#79#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_shifterop,ot_none,ot_none,ot_none,ot_none);
    code    : #8#1#160;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MOV;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #10#1#160;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MOV;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #11#1#160;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MRS;
    ops     : 2;
    optypes : (ot_reg32,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #150#243#239#128#0;
    flags   : if_thumb32 or if_armv6
  ),
  (
    opcode  : A_MRS;
    ops     : 2;
    optypes : (ot_reg32,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #16#1#15;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MSR;
    ops     : 2;
    optypes : (ot_regf,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #150#243#128#128#0;
    flags   : if_thumb32 or if_armv6
  ),
  (
    opcode  : A_MSR;
    ops     : 2;
    optypes : (ot_regf,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #18#1#32#240;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MSR;
    ops     : 2;
    optypes : (ot_regs,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #18#1#32#240;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MSR;
    ops     : 2;
    optypes : (ot_regf,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #19#3#32#240;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MSR;
    ops     : 2;
    optypes : (ot_regs,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #19#3#32#240;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MUL;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #100#67#64;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_MUL;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_reglo,ot_none,ot_none,ot_none);
    code    : #100#67#64;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_MUL;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#251#0#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MUL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#251#0#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MUL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #20#0#0#144;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MVF;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#1;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_MVF;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#1;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_MVN;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#67#192;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_MVN;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#111#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MVN;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#111#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_MVN;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #8#1#224;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MVN;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #10#1#224;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MVN;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #11#1#224;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_VMOV;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #144#238#176#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #64#14#176#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 2;
    optypes : (ot_vreg,ot_immediatemm,ot_none,ot_none,ot_none,ot_none);
    code    : #144#238#176#10#0;
    flags   : if_thumb32 or if_vfpv3
  ),
  (
    opcode  : A_VMOV;
    ops     : 2;
    optypes : (ot_vreg,ot_immediatemm,ot_none,ot_none,ot_none,ot_none);
    code    : #64#14#176#10#0;
    flags   : if_arm32 or if_vfpv3
  ),
  (
    opcode  : A_VMOV;
    ops     : 2;
    optypes : (ot_reg32,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #144#238#16#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 2;
    optypes : (ot_vreg,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #144#238#0#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 2;
    optypes : (ot_reg32,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #64#14#16#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 2;
    optypes : (ot_vreg,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #64#14#0#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_vreg,ot_vreg,ot_none,ot_none);
    code    : #144#236#80#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 4;
    optypes : (ot_vreg,ot_vreg,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #144#236#64#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_vreg,ot_vreg,ot_none,ot_none);
    code    : #64#12#80#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 4;
    optypes : (ot_vreg,ot_vreg,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #64#12#64#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_vreg,ot_none,ot_none,ot_none);
    code    : #144#236#80#11#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 3;
    optypes : (ot_vreg,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #144#236#64#11#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_vreg,ot_none,ot_none,ot_none);
    code    : #64#12#80#11#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMOV;
    ops     : 3;
    optypes : (ot_vreg,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #64#12#64#11#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_NOP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #97#191#0;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_NOP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #47#3#32#240#0;
    flags   : if_arm32 or if_armv6k
  ),
  (
    opcode  : A_NOP;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #47#225#160#0#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ORN;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ORN;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ORN;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#234#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ORN;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#240#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ORN;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#234#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ORN;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#234#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ORR;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#67#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ORR;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ORR;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#64#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ORR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#234#64#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ORR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#240#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_ORR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#234#64#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ORR;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#234#64#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ORR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #4#1#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ORR;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #5#1#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ORR;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#1#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ORR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#3#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_RSB;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_immediatezero,ot_none,ot_none,ot_none);
    code    : #107#66#64;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_RSB;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#241#192#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_RSB;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#235#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_RSB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#235#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_RSB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#241#192#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_RSB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#235#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_RSB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#235#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_RSB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #6#0#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_RSB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#0#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_RSB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#0#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_RSC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #4#0#224;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_RSC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #5#0#224;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_RSC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#0#224;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_RSC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#2#224;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SBC;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#65#128;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SBC;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#241#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SBC;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#235#96#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SBC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#235#96#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SBC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#241#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SBC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#235#96#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SBC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#235#96#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SBC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #4#0#192;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SBC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #5#0#192;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SBC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_immediate,ot_none,ot_none);
    code    : #6#0#192;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SBC;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#0#192;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SBC;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #7#2#192;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SFM;
    ops     : 3;
    optypes : (ot_fpureg,ot_immediate or ot_bits32,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #160#12#0#2#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_SFM;
    ops     : 3;
    optypes : (ot_fpureg,ot_immediateshifter,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #160#12#0#2#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_SIN;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#17;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_SIN;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#17;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_SMLAL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #133#251#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMLAL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#0#224#9;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SMULL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #133#251#128#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMULL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#0#192#9;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #105#192;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STM;
    ops     : 2;
    optypes : (ot_reglo,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #105#192;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #140#232#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_STM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #140#232#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_STM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #38#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #38#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam3,ot_none,ot_none,ot_none,ot_none);
    code    : #101#80#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam4,ot_none,ot_none,ot_none,ot_none);
    code    : #102#96#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam5,ot_none,ot_none,ot_none,ot_none);
    code    : #103#144#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STR;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #103#144#0#2;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STR;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#64#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_STR;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STRB;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam3,ot_none,ot_none,ot_none,ot_none);
    code    : #101#84#0#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STRB;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam4,ot_none,ot_none,ot_none,ot_none);
    code    : #102#112#0#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STRB;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#0#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_STRB;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STRBT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#0#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_STRBT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STRBT;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STRH;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam3,ot_none,ot_none,ot_none,ot_none);
    code    : #101#82#0#1;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STRH;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam4,ot_none,ot_none,ot_none,ot_none);
    code    : #102#128#0#1;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_STRH;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#32#0#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_STRH;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #34#0#176;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STRT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#64#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_STRT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #23#4#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_regsp,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #100#176#128;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_regsp,ot_regsp,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #100#176#128;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #96#26#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_reglo,ot_none,ot_none,ot_none);
    code    : #96#26#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #96#30#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #96#30#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reglo,ot_immediate or ot_bits8,ot_none,ot_none,ot_none,ot_none);
    code    : #107#56#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #107#56#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#241#160#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SUB;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#235#160#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#235#160#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #128#241#160#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#235#160#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SUB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#235#160#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #4#0#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #4#0#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #4#0#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SUB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #6#0#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SWP;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #39#16#9;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SWPB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #39#20#9;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TEQ;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#144#15#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_TEQ;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#144#15#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_TEQ;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#234#144#15#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_TEQ;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #12#1#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TEQ;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #13#1#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TEQ;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #14#1#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TEQ;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #15#3#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TST;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #111#66#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_TST;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #128#240#16#15#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_TST;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#16#15#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_TST;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #128#234#16#15#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_TST;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #12#1#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TST;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #13#1#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TST;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #14#1#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TST;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #15#3#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_UMLAL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #133#251#224#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UMLAL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#0#160#9;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_UMULL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #133#251#160#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UMULL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#0#128#9;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_WFS;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#2;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_LDRD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #137#232#80#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDRD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #25#0#0#0#208;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_PLD;
    ops     : 1;
    optypes : (ot_memoryam2,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #135#248#16#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_PLD;
    ops     : 1;
    optypes : (ot_memoryam2,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #37#245#80#240#0;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_PLDW;
    ops     : 1;
    optypes : (ot_memoryam2,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #135#248#48#240#0;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_PLDW;
    ops     : 1;
    optypes : (ot_memoryam2,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #37#245#16#240#0;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_QADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #130#250#128#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #26#1#0#5;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_QDADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #130#250#128#240#144;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QDADD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #26#1#64#5;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_QDSUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #130#250#128#240#176;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QDSUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #26#1#96#5;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_QSUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #130#250#128#240#160;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QSUB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #26#1#32#5;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLABB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#1#0#8;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLABT;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#1#0#12;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLATB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#1#0#10;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLATT;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#1#0#14;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLALBB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#1#64#8;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLALBT;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#1#64#12;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLALTB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#1#64#10;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLALTT;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#1#64#14;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLAWB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#48#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMLAWB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#1#32#8;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMLAWT;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#48#0#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMLAWT;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#1#32#12;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_VLDM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#16#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VLDM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#16#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VLDM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#16#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VLDM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#16#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VSTM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#0#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VSTM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#0#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VSTM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#0#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VSTM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#0#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VPOP;
    ops     : 1;
    optypes : (ot_reglist,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#189#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VPOP;
    ops     : 1;
    optypes : (ot_reglist,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#189#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VPUSH;
    ops     : 1;
    optypes : (ot_reglist,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #148#237#45#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VPUSH;
    ops     : 1;
    optypes : (ot_reglist,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #68#13#45#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VLDR;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #149#237#16#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VLDR;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #69#13#16#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VSTR;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #149#237#0#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VSTR;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #69#13#0#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_SMULBB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#1#96#8#0;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMULBT;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#1#96#12#0;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMULTB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#1#96#10#0;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMULTT;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#1#96#14#0;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMULWB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #20#1#32#160;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_SMULWT;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #20#1#32#224;
    flags   : if_arm32 or if_armv5te
  ),
  (
    opcode  : A_STRD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #137#232#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_STRD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none);
    code    : #25#0#0#0#240;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRHT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#48#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDRHT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #25#0#48#0#176;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STRHT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#32#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_STRHT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#248#32#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_STRHT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #30#0#32#0#176;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRSBT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#249#16#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDRSBT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #30#0#48#0#208;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDRSHT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #136#249#48#14#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDRSHT;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #30#0#48#0#240;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_FSTD;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #149#237#0#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSTD;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #69#13#0#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSTM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#0#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSTM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#0#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSTM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#0#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSTM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#0#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSTS;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #149#237#0#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSTS;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #69#13#0#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_BFC;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #132#243#111#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BFC;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_immediate or ot_bits32,ot_none,ot_none,ot_none);
    code    : #132#243#111#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BFC;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #45#7#192#0#31;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BFC;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_immediate or ot_bits32,ot_none,ot_none,ot_none);
    code    : #45#7#192#0#31;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BFI;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_immediateshifter,ot_none,ot_none);
    code    : #132#243#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BFI;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_immediate or ot_bits32,ot_none,ot_none);
    code    : #132#243#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BFI;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_immediateshifter,ot_none,ot_none);
    code    : #45#7#192#0#16;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BFI;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_immediate or ot_bits32,ot_none,ot_none);
    code    : #45#7#192#0#16;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_CLREX;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #128#243#191#143#47;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_CLREX;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #47#245#127#240#31;
    flags   : if_arm32 or if_armv6k
  ),
  (
    opcode  : A_LDREX;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none,ot_none);
    code    : #138#232#80#15#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_LDREX;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none,ot_none);
    code    : #24#1#144#15#159;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDREXB;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none,ot_none);
    code    : #138#232#208#15#79;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_LDREXB;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none,ot_none);
    code    : #24#1#208#15#159;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDREXD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none);
    code    : #138#232#208#0#127;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_LDREXD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none);
    code    : #24#1#176#15#159;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LDREXH;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none,ot_none);
    code    : #138#232#208#15#95;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_LDREXH;
    ops     : 2;
    optypes : (ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none,ot_none);
    code    : #24#1#240#15#159;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STREX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none);
    code    : #139#232#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_STREX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none);
    code    : #24#1#128#15#144;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STREXB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none);
    code    : #139#232#192#15#64;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_STREXB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none);
    code    : #24#1#192#15#144;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STREXD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none);
    code    : #139#232#192#0#112;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_STREXD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none);
    code    : #24#1#160#15#144;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_STREXH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none);
    code    : #139#232#192#15#80;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_STREXH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_memoryam6,ot_none,ot_none,ot_none);
    code    : #24#1#224#15#144;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_MLS;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#0#0#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MLS;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#0#96#9;
    flags   : if_arm32 or if_armv6t2
  ),
  (
    opcode  : A_PKHBT;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#234#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_PKHBT;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#234#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_PKHBT;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#128#1;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_PKHBT;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #22#6#128#1;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_PKHTB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#234#192#0#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_PKHTB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #128#234#192#0#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_PKHTB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#128#1;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_PKHTB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #22#6#128#5;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_PLI;
    ops     : 1;
    optypes : (ot_memoryam2,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #135#249#16#240#0;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_PLI;
    ops     : 1;
    optypes : (ot_memoryam2,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #37#244#80#240#0;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_QADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#32#241;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_QADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#128#240#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#32#249;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_QASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#160#240#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#32#243;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_QSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#224#240#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#32#245;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_QSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#208#240#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#32#247;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_QSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#192#240#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_QSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#32#255;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_RBIT;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#160;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_RBIT;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #50#6#255#15#48;
    flags   : if_arm32 or if_armv6t2
  ),
  (
    opcode  : A_REV;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #97#186#0;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_REV;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_REV;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #50#6#191#15#48;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_REV16;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #97#186#64;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_REV16;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#144;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_REV16;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #50#6#191#15#176;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_REVSH;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #97#186#192;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_REVSH;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#176;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_REVSH;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #50#6#255#15#176;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#72#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#16#241;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#64#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#16#249;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#80#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#16#243;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SBFX;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_immediateshifter,ot_none,ot_none);
    code    : #132#243#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SBFX;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_immediateshifter,ot_none,ot_none);
    code    : #45#7#160#0#80;
    flags   : if_arm32 or if_armv6t2
  ),
  (
    opcode  : A_SEL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#160#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SEL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#128#251;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SETEND;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #43#241#1#0#0;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SEV;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #100#191#64;
    flags   : if_thumb or if_armv7
  ),
  (
    opcode  : A_SEV;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #47#3#32#240#4;
    flags   : if_arm32 or if_armv6k
  ),
  (
    opcode  : A_ASR;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #96#1#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ASR;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #96#1#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ASR;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#65#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ASR;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #130#234#79#0#32;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ASR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #130#234#79#0#32;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ASR;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#64#240#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ASR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#64#240#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ASR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#80;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ASR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#64;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LSR;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #96#8#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LSR;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #96#8#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LSR;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#64#192;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LSR;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #130#234#79#0#16;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LSR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #130#234#79#0#16;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LSR;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#32#240#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LSR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#32#240#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LSR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#48;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LSR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#32;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LSL;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #96#0#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LSL;
    ops     : 3;
    optypes : (ot_reglo,ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #96#0#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LSL;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#64#128;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_LSL;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #130#234#79#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LSL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #130#234#79#0#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LSL;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#96#240#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LSL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#96#240#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_LSL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#16;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_LSL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#0;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #107#65#192;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #130#234#79#0#48;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ROR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #130#234#79#0#48;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ROR;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#250#96#240#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ROR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#96#240#0;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_ROR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#112;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ROR;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_RRX;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #128#234#79#0#48;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_RRX;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #48#1#160#0#96;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_UMAAL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #133#251#224#0#96;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UMAAL;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#0#64#9;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SHADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#32;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SHADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#48#241;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SHADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#128#240#32;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SHADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#48#249;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SHASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#160#240#32;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SHASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#48#243;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SHSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#224#240#32;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SHSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#48#245;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SHSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#208#240#32;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SHSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#48#247;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SHSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#192#240#32;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SHSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#48#255;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SMLAD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#32#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMLAD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#7#0#1;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SMLALD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #133#251#192#0#192;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMLALD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#7#64#1;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SMLSD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMLSD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#7#0#5;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SMLSLD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #133#251#208#0#192;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMLSLD;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #22#7#64#5;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SMMLA;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#80#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMMLA;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#7#80#1;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SMMLS;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#96#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMMLS;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#7#80#13;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SMMUL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#251#80#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMMUL;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#7#80#1#15;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SMUAD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#251#32#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMUAD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#7#0#1#15;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SMUSD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#251#64#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SMUSD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#7#0#5#15;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SSAT;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_none,ot_none,ot_none);
    code    : #131#243#0#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SSAT;
    ops     : 4;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #131#243#0#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SSAT;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_none,ot_none,ot_none);
    code    : #42#6#160#0#16;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SSAT;
    ops     : 4;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #42#6#160#0#16;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SSAT16;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_none,ot_none,ot_none);
    code    : #131#243#32#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SSAT16;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_none,ot_none,ot_none);
    code    : #42#6#160#15#48;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#224#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#16#245;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#208#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#16#247;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#192#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#16#255;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTAB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #134#250#64#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SXTAB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #134#250#64#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SXTAB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#160#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTAB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #22#6#160#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTAB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #134#250#32#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SXTAB16;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #134#250#32#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SXTAB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#128#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTAB16;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #22#6#128#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTAH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #134#250#0#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SXTAH;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #134#250#0#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SXTAH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#176#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTAH;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #22#6#176#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UBFX;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_immediateshifter,ot_none,ot_none);
    code    : #132#243#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UBFX;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_immediateshifter,ot_immediateshifter,ot_none,ot_none);
    code    : #45#7#224#0#80;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_UXTAB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #134#250#80#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTAB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #134#250#80#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTAB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#224#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTAB;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #22#6#224#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTAB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #134#250#48#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTAB16;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #134#250#48#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTAB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #134#250#64#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTAB16;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #134#250#64#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTAB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#192#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTAB16;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #22#6#192#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTAH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #134#250#16#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTAH;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #134#250#16#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTAH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#240#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTAH;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #22#6#240#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTB;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #97#178#64;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_SXTB;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #134#250#79#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SXTB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #134#250#79#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SXTB;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #27#6#175#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #27#6#175#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTB16;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #134#250#47#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SXTB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #134#250#47#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_SXTB16;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #27#6#143#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #27#6#143#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTH;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #97#178#0;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_SXTH;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #134#250#15#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SXTH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #134#250#15#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_SXTH;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #27#6#191#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_SXTH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #27#6#191#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTB;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #97#178#192;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_UXTB;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #134#250#95#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_UXTB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #134#250#95#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_UXTB;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #27#6#239#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTB;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #27#6#239#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTB16;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #134#250#63#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #134#250#63#240#128;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UXTB16;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #27#6#207#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #27#6#207#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTH;
    ops     : 2;
    optypes : (ot_reglo,ot_reglo,ot_none,ot_none,ot_none,ot_none);
    code    : #97#178#128;
    flags   : if_thumb or if_armv6
  ),
  (
    opcode  : A_UXTH;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #134#250#31#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_UXTH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #134#250#31#240#128;
    flags   : if_thumb32 or if_wide or if_armv6t2
  ),
  (
    opcode  : A_UXTH;
    ops     : 2;
    optypes : (ot_reg32,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #27#6#255#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UXTH;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_shifterop,ot_none,ot_none,ot_none);
    code    : #27#6#255#7;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#64;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#80#241;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#128#240#64;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#80#249;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#160#240#64;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#80#243;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UHADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#96;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UHADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#112#241;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UHADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#128#240#96;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UHADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#112#249;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UHASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#160#240#96;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UHASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#112#243;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UHSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#224#240#96;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UHSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#112#245;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UHSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#208#240#96;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UHSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#112#247;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UHSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#192#240#96;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UHSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#112#255;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UQADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#144#240#80;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UQADD16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#96#241;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UQADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#128#240#80;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UQADD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#96#249;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UQASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#160#240#80;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UQASX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#96#243;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UQSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#224#240#80;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UQSAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#96#245;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UQSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#208#240#80;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UQSUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#96#247;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_UQSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#192#240#80;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_UQSUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#96#255;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_USAD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#251#112#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_USAD8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#7#128#1#15;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_USADA8;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #128#251#112#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_USADA8;
    ops     : 4;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none);
    code    : #21#7#128#1;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_USAT;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_none,ot_none,ot_none);
    code    : #131#243#128#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_USAT;
    ops     : 4;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #131#243#128#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_USAT;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_none,ot_none,ot_none);
    code    : #42#6#224#0#16;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_USAT;
    ops     : 4;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_shifterop,ot_none,ot_none);
    code    : #42#6#224#0#16;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_USAT16;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_none,ot_none,ot_none);
    code    : #131#243#160#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_USAT16;
    ops     : 3;
    optypes : (ot_reg32,ot_immediateshifter,ot_reg32,ot_none,ot_none,ot_none);
    code    : #42#6#224#15#48;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_USAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#224#240#64;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_USAX;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#80#245;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_USUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#208#240#64;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_USUB16;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#80#247;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_USUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#250#192#240#64;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_USUB8;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #22#6#80#255;
    flags   : if_arm32 or if_armv6
  ),
  (
    opcode  : A_WFE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #100#191#32;
    flags   : if_thumb or if_armv7
  ),
  (
    opcode  : A_WFE;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #47#3#32#240#2;
    flags   : if_arm32 or if_armv6k
  ),
  (
    opcode  : A_WFI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #100#191#48;
    flags   : if_thumb or if_armv7
  ),
  (
    opcode  : A_WFI;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #47#3#32#240#3;
    flags   : if_arm32 or if_armv6k
  ),
  (
    opcode  : A_YIELD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #100#191#16;
    flags   : if_thumb or if_armv7
  ),
  (
    opcode  : A_YIELD;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #47#3#32#240#1;
    flags   : if_arm32 or if_armv6k
  ),
  (
    opcode  : A_FABSD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#176#10#192#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FABSD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#176#10#192#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FABSS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#176#10#192#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FABSS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#176#10#192#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FADDD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#48#10#0#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FADDD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#48#10#0#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FADDS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#48#10#0#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FADDS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#48#10#0#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#180#10#64#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#180#10#64#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#180#10#64#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#180#10#64#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPED;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#180#10#192#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPED;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#180#10#192#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPES;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#180#10#192#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPES;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#180#10#192#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPZD;
    ops     : 1;
    optypes : (ot_vreg,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#181#10#64#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPZD;
    ops     : 1;
    optypes : (ot_vreg,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#181#10#64#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPZS;
    ops     : 1;
    optypes : (ot_vreg,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#181#10#64#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPZS;
    ops     : 1;
    optypes : (ot_vreg,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#181#10#64#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPEZD;
    ops     : 1;
    optypes : (ot_vreg,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#181#10#192#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPEZD;
    ops     : 1;
    optypes : (ot_vreg,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#181#10#192#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPEZS;
    ops     : 1;
    optypes : (ot_vreg,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#181#10#192#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCMPEZS;
    ops     : 1;
    optypes : (ot_vreg,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#181#10#192#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCPYD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#176#11#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCPYD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#176#11#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCPYS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#176#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCPYS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#176#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCVTDS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#183#10#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCVTDS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#183#10#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FCVTSD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#183#11#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FCVTSD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#183#11#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FDIVD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#128#10#0#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FDIVD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#128#10#0#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FDIVS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#128#10#0#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FDIVS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#128#10#0#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FLDD;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #149#237#16#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FLDD;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #69#13#16#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FLDM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#16#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FLDM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #148#236#16#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FLDM;
    ops     : 2;
    optypes : (ot_memoryam4,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#16#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FLDM;
    ops     : 2;
    optypes : (ot_reg32,ot_reglist,ot_none,ot_none,ot_none,ot_none);
    code    : #68#12#16#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FLDS;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #149#237#16#10;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FLDS;
    ops     : 2;
    optypes : (ot_vreg,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #69#13#16#10;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMACD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#0#10#0#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMACD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#0#10#0#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMACS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#0#10#0#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMACS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#0#10#0#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMRS;
    ops     : 2;
    optypes : (ot_reg32,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #144#238#16#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMRS;
    ops     : 2;
    optypes : (ot_reg32,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #64#14#16#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMRX;
    ops     : 2;
    optypes : (ot_reg32,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #145#238#240#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMRX;
    ops     : 2;
    optypes : (ot_regf,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #145#238#240#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMRX;
    ops     : 2;
    optypes : (ot_reg32,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #65#14#240#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMRX;
    ops     : 2;
    optypes : (ot_regf,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #65#14#240#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMSCD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#16#10#0#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMSCD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#16#10#0#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMSCS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#16#10#0#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMSCS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#16#10#0#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMSR;
    ops     : 2;
    optypes : (ot_vreg,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #144#238#0#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMSR;
    ops     : 2;
    optypes : (ot_vreg,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #64#14#0#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMSTAT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #128#238#241#250#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMSTAT;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #47#14#241#250#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMULD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#32#10#0#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMULD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#32#10#0#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMULS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#32#10#0#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMULS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#32#10#0#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMXR;
    ops     : 2;
    optypes : (ot_regf,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #145#238#224#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMXR;
    ops     : 2;
    optypes : (ot_regf,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #65#14#224#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FNEGD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#177#10#64#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FNEGD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#177#10#64#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FNEGS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#177#10#64#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FNEGS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#177#10#64#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FNMACD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#0#10#64#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FNMACD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#0#10#64#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FNMACS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#0#10#64#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FNMACS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#0#10#64#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FNMSCD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#16#10#64#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FNMSCD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#16#10#64#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FNMSCS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#16#10#64#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FNMSCS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#16#10#64#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FNMULD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#32#10#64#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FNMULD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#32#10#64#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FNMULS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#32#10#64#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FNMULS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#32#10#64#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSITOD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#184#11#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSITOD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#184#11#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSITOS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#184#10#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSITOS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#184#10#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSQRTD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#177#10#192#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSQRTD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#177#10#192#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSQRTS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#177#10#192#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSQRTS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#177#10#192#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSUBD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#48#10#64#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSUBD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#48#10#64#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FSUBS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#48#10#64#1;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FSUBS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#48#10#64#1;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FTOSID;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#189#11#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FTOSID;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#189#11#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FTOSIS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#189#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FTOSIS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#189#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FTOUID;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#188#11#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FTOUID;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#188#11#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FTOUIS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#188#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FTOUIS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#188#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FUITOD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#184#11#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FUITOD;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#184#11#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FUITOS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#238#184#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FUITOS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#184#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMDRR;
    ops     : 3;
    optypes : (ot_vreg,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #144#236#64#11#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMDRR;
    ops     : 3;
    optypes : (ot_vreg,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #64#12#64#11#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_FMRRD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_vreg,ot_none,ot_none,ot_none);
    code    : #144#236#80#11#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_FMRRD;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_vreg,ot_none,ot_none,ot_none);
    code    : #64#12#80#11#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_reglist,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #105#188;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_POP;
    ops     : 1;
    optypes : (ot_reglist,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #38#139;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_reglist,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #105#180;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_PUSH;
    ops     : 1;
    optypes : (ot_reglist,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #38#128;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SDIV;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#251#144#240#240;
    flags   : if_thumb32 or if_armv7r or if_armv7m
  ),
  (
    opcode  : A_SDIV;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#7#16#1#15;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_UDIV;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #128#251#176#240#240;
    flags   : if_thumb32 or if_armv7r or if_armv7m
  ),
  (
    opcode  : A_UDIV;
    ops     : 3;
    optypes : (ot_reg32,ot_reg32,ot_reg32,ot_none,ot_none,ot_none);
    code    : #21#7#48#1#15;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_MOVT;
    ops     : 2;
    optypes : (ot_reg32,ot_immediate,ot_none,ot_none,ot_none,ot_none);
    code    : #129#242#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MOVT;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #129#242#192#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MOVT;
    ops     : 2;
    optypes : (ot_reg32,ot_immediate,ot_none,ot_none,ot_none,ot_none);
    code    : #44#3#64;
    flags   : if_arm32 or if_armv6t2
  ),
  (
    opcode  : A_MOVT;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #44#3#64;
    flags   : if_arm32 or if_armv6t2
  ),
  (
    opcode  : A_IT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#8#0;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_IT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#4#136;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#4#8;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITEE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#2#204;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITEE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITTE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#2#76;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITTE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITET;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#2#140;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITET;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITTT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#2#12;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITTT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITEEE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#1#238;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITEEE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITTEE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#1#110;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITTEE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITETE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#1#174;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITETE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITTTE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#1#46;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITTTE;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITEET;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#1#206;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITEET;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITTET;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#1#78;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITTET;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITETT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#1#142;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITETT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_ITTTT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #106#191#1#14;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_ITTTT;
    ops     : 1;
    optypes : (ot_condition,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #254;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_TBB;
    ops     : 1;
    optypes : (ot_memoryam2,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #142#232#208#240#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_TBH;
    ops     : 1;
    optypes : (ot_memoryam2,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #142#232#208#240#16;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MOVW;
    ops     : 2;
    optypes : (ot_reg32,ot_immediate or ot_bits32,ot_none,ot_none,ot_none,ot_none);
    code    : #44#3#0;
    flags   : if_arm32 or if_armv6t2
  ),
  (
    opcode  : A_MOVW;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #44#3#0;
    flags   : if_arm32 or if_armv6t2
  ),
  (
    opcode  : A_MOVW;
    ops     : 2;
    optypes : (ot_reg32,ot_immediate or ot_bits32,ot_none,ot_none,ot_none,ot_none);
    code    : #129#242#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_MOVW;
    ops     : 2;
    optypes : (ot_reg32,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #129#242#64#0#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_CBZ;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #104#177;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_CBZ;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #104#177;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_CBNZ;
    ops     : 2;
    optypes : (ot_reglo,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #104#185;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_CBNZ;
    ops     : 2;
    optypes : (ot_reglo,ot_memoryam2,ot_none,ot_none,ot_none,ot_none);
    code    : #104#185;
    flags   : if_thumb or if_armv6t2
  ),
  (
    opcode  : A_VABS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#176#10#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VABS;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#176#10#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VADD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#48#10#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VADD;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#48#10#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VCMP;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#180#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VCMP;
    ops     : 2;
    optypes : (ot_vreg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#181#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VCMP;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#180#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VCMP;
    ops     : 2;
    optypes : (ot_vreg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#181#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VCMPE;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#180#10#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VCMPE;
    ops     : 2;
    optypes : (ot_vreg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#181#10#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VCMPE;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#180#10#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VCMPE;
    ops     : 2;
    optypes : (ot_vreg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#181#10#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VCVT;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #147#238#184#10#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VCVT;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #147#238#186#10#64;
    flags   : if_thumb32 or if_vfpv3
  ),
  (
    opcode  : A_VCVT;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#184#10#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VCVT;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #67#14#186#10#64;
    flags   : if_arm32 or if_vfpv3
  ),
  (
    opcode  : A_VCVTR;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #147#238#184#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VCVTR;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #67#14#184#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VDIV;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#128#10#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VDIV;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#128#10#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMRS;
    ops     : 2;
    optypes : (ot_reg32,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #145#238#240#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMRS;
    ops     : 2;
    optypes : (ot_regf,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #145#238#240#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMRS;
    ops     : 2;
    optypes : (ot_reg32,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #65#14#240#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMRS;
    ops     : 2;
    optypes : (ot_regf,ot_regf,ot_none,ot_none,ot_none,ot_none);
    code    : #65#14#240#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMSR;
    ops     : 2;
    optypes : (ot_regf,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #145#238#224#10#16;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMSR;
    ops     : 2;
    optypes : (ot_regf,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #65#14#224#10#16;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMLA;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#0#10#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMLA;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#0#10#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMLS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#0#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMLS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#0#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VMUL;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#32#10#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VMUL;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#32#10#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VNMLA;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#16#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VNMLA;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#16#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VNMLS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#16#10#0;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VNMLS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#16#10#0;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VNMUL;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#32#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VNMUL;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#32#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VFMA;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#160#10#0;
    flags   : if_thumb32 or if_vfpv4
  ),
  (
    opcode  : A_VFMA;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#160#10#0;
    flags   : if_arm32 or if_vfpv4
  ),
  (
    opcode  : A_VFMS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#160#10#64;
    flags   : if_thumb32 or if_vfpv4
  ),
  (
    opcode  : A_VFMS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#160#10#64;
    flags   : if_arm32 or if_vfpv4
  ),
  (
    opcode  : A_VFNMA;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#144#10#64;
    flags   : if_thumb32 or if_vfpv4
  ),
  (
    opcode  : A_VFNMA;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#144#10#64;
    flags   : if_arm32 or if_vfpv4
  ),
  (
    opcode  : A_VFNMS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#144#10#0;
    flags   : if_thumb32 or if_vfpv4
  ),
  (
    opcode  : A_VFNMS;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#144#10#0;
    flags   : if_arm32 or if_vfpv4
  ),
  (
    opcode  : A_VNEG;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#177#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VNEG;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#177#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VSQRT;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #146#238#177#10#192;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VSQRT;
    ops     : 2;
    optypes : (ot_vreg,ot_vreg,ot_none,ot_none,ot_none,ot_none);
    code    : #66#14#177#10#192;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_VSUB;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #146#238#48#10#64;
    flags   : if_thumb32 or if_vfpv2
  ),
  (
    opcode  : A_VSUB;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#14#48#10#64;
    flags   : if_arm32 or if_vfpv2
  ),
  (
    opcode  : A_DMB;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #128#243#191#143#80;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_DMB;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #46#245#127#240#80;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_ISB;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #128#243#191#143#96;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_ISB;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #46#245#127#240#96;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_DSB;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #128#243#191#143#64;
    flags   : if_thumb32 or if_armv7
  ),
  (
    opcode  : A_DSB;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #46#245#127#240#64;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_SMC;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #46#1#96#0#112;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_SMC;
    ops     : 1;
    optypes : (ot_immediate or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #46#1#96#0#112;
    flags   : if_arm32 or if_armv7
  ),
  (
    opcode  : A_SVC;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #97#223#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SVC;
    ops     : 1;
    optypes : (ot_immediate or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #97#223#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_SVC;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #2#15;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_SVC;
    ops     : 1;
    optypes : (ot_immediate or ot_bits32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #2#15;
    flags   : if_arm32 or if_armv4
  ),
  (
    opcode  : A_BXJ;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #128#243#192#143#0;
    flags   : if_thumb32 or if_armv6t2
  ),
  (
    opcode  : A_BXJ;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #3#1#47#255#32;
    flags   : if_arm32 or if_armv5tej
  ),
  (
    opcode  : A_UDF;
    ops     : 1;
    optypes : (ot_immediateshifter,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #97#222#0;
    flags   : if_thumb or if_armv4t
  ),
  (
    opcode  : A_UDF;
    ops     : 0;
    optypes : (ot_none,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #0;
    flags   : if_arm32 or if_armv4t
  ),
  (
    opcode  : A_VEOR;
    ops     : 3;
    optypes : (ot_vreg,ot_vreg,ot_vreg,ot_none,ot_none,ot_none);
    code    : #66#243#0#1#16;
    flags   : if_arm32 or if_neon
  ),
  (
    opcode  : A_TAN;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#21;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_TAN;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#21;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_SQT;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#9;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_SQT;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#9;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_SUF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#4;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_SUF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#4;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RSF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#6;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RSF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#6;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RND;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#7;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RND;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#7;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_POL;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#24;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_POL;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#24;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RDF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#10;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RDF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#10;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RFS;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#3;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RFC;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#5;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_WFC;
    ops     : 1;
    optypes : (ot_reg32,ot_none,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#4;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RMF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#16;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RMF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#16;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RPW;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#12;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_RPW;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#12;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_MNF;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#3;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_MNF;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#3;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_MUF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#2;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_MUF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#2;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ABS;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#5;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ABS;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#5;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ACS;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#25;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ACS;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#25;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ASN;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#23;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ASN;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#23;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ATN;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#27;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_ATN;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#27;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_CNF;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#176;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_CNF;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#176;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_CNFE;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#240;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_CNFE;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#240;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_COS;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#19;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_COS;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#19;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_DVF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#8;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_DVF;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#8;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_EXP;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#15;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_EXP;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#15;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_FDV;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#20;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_FDV;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#20;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_FLT;
    ops     : 2;
    optypes : (ot_fpureg,ot_reg32,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#0;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_FIX;
    ops     : 2;
    optypes : (ot_reg32,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #162#14#16;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_FML;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#18;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_FML;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#18;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_FRD;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none);
    code    : #161#0#22;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_FRD;
    ops     : 3;
    optypes : (ot_fpureg,ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none);
    code    : #161#0#22;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_LGN;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#13;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_LGN;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#13;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_LOG;
    ops     : 2;
    optypes : (ot_fpureg,ot_fpureg,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#11;
    flags   : if_arm32 or if_fpa
  ),
  (
    opcode  : A_LOG;
    ops     : 2;
    optypes : (ot_fpureg,ot_immediateshifter,ot_none,ot_none,ot_none,ot_none);
    code    : #161#1#11;
    flags   : if_arm32 or if_fpa
  )
);
