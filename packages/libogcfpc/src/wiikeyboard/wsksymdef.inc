
{$IFDEF OGC_INTERFACE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
VAR
  KS_BackSpace: byte = $08;
  KS_Tab: byte = $09;
  KS_Linefeed: byte = $0a;
  KS_Clear: byte = $0b;
  KS_Return: byte = $0d;
  KS_Escape: byte = $1b;
  KS_space: byte = $20;
  KS_exclam: byte = $21;
  KS_quotedbl: byte = $22;
  KS_numbersign: byte = $23;
  KS_dollar: byte = $24;
  KS_percent: byte = $25;
  KS_ampersand: byte = $26;
  KS_apostrophe: byte = $27;
  KS_parenleft: byte = $28;
  KS_parenright: byte = $29;
  KS_asterisk: byte = $2a;
  KS_plus: byte = $2b;
  KS_comma: byte = $2c;
  KS_minus: byte = $2d;
  KS_period: byte = $2e;
  KS_slash: byte = $2f;
  KS_0: byte = $30;
  KS_1: byte = $31;
  KS_2: byte = $32;
  KS_3: byte = $33;
  KS_4: byte = $34;
  KS_5: byte = $35;
  KS_6: byte = $36;
  KS_7: byte = $37;
  KS_8: byte = $38;
  KS_9: byte = $39;
  KS_colon: byte = $3a;
  KS_semicolon: byte = $3b;
  KS_less: byte = $3c;
  KS_equal: byte = $3d;
  KS_greater: byte = $3e;
  KS_question: byte = $3f;
  KS_at: byte = $40;

  KS_A: byte = $41; public name 'KS_A';
  KS_B: byte = $42; public name 'KS_B';
  KS_C: byte = $43; public name 'KS_C';
  KS_D: byte = $44; public name 'KS_D';
  KS_E: byte = $45; public name 'KS_E';
  KS_F: byte = $46; public name 'KS_F';
  KS_G: byte = $47; public name 'KS_G';
  KS_H: byte = $48; public name 'KS_H';
  KS_I: byte = $49; public name 'KS_I';
  KS_J: byte = $4a; public name 'KS_J';
  KS_K: byte = $4b; public name 'KS_K';
  KS_L: byte = $4c; public name 'KS_L';
  KS_M: byte = $4d; public name 'KS_M';                  
  KS_N: byte = $4e; public name 'KS_N';
  KS_O: byte = $4f; public name 'KS_O';
  KS_P: byte = $50; public name 'KS_P';
  KS_Q: byte = $51; public name 'KS_Q';
  KS_R: byte = $52; public name 'KS_R';
  KS_S: byte = $53; public name 'KS_S';
  KS_T: byte = $54; public name 'KS_T';
  KS_U: byte = $55; public name 'KS_U';
  KS_V: byte = $56; public name 'KS_V';
  KS_W: byte = $57; public name 'KS_W';
  KS_X: byte = $58; public name 'KS_X';
  KS_Y: byte = $59; public name 'KS_Y';
  KS_Z: byte = $5a; public name 'KS_Z';

  KS_bracketleft: byte = $5b;
  KS_backslash: byte = $5c;
  KS_bracketright: byte = $5d;
  KS_asciicircum: byte = $5e;
  KS_underscore: byte = $5f;
  KS_grave: byte = $60;

  KS_a_: byte = $61; public name 'KS_a';
  KS_b_: byte = $62; public name 'KS_b';
  KS_c_: byte = $63; public name 'KS_c';
  KS_d_: byte = $64; public name 'KS_d';
  KS_e_: byte = $65; public name 'KS_e';
  KS_f_: byte = $66; public name 'KS_f';
  KS_g_: byte = $67; public name 'KS_g';
  KS_h_: byte = $68; public name 'KS_h';
  KS_i_: byte = $69; public name 'KS_i';
  KS_j_: byte = $6a; public name 'KS_j';
  KS_k_: byte = $6b; public name 'KS_k';
  KS_l_: byte = $6c; public name 'KS_l';
  KS_m_: byte = $6d; public name 'KS_m';
  KS_n_: byte = $6e; public name 'KS_n';
  KS_o_: byte = $6f; public name 'KS_o';
  KS_p_: byte = $70; public name 'KS_p';
  KS_q_: byte = $71; public name 'KS_q';
  KS_r_: byte = $72; public name 'KS_r';                  
  KS_s_: byte = $73; public name 'KS_s';
  KS_t_: byte = $74; public name 'KS_t';
  KS_u_: byte = $75; public name 'KS_u';                   
  KS_v_: byte = $76; public name 'KS_v';
  KS_w_: byte = $77; public name 'KS_w';
  KS_x_: byte = $78; public name 'KS_x';
  KS_y_: byte = $79; public name 'KS_y';
  KS_z_: byte = $7a; public name 'KS_z';

  KS_braceleft: byte = $7b;
  KS_bar: byte = $7c;
  KS_braceright: byte = $7d;
  KS_asciitilde: byte = $7e;
  KS_Delete: byte = $7f;
  KS_nobreakspace: byte = $a0;
  KS_exclamdown: byte = $a1;
  KS_cent: byte = $a2;
  KS_sterling: byte = $a3;
  KS_currency: byte = $a4;
  KS_yen: byte = $a5;
  KS_brokenbar: byte = $a6;
  KS_section: byte = $a7;
  KS_diaeresis: byte = $a8;
  KS_copyright: byte = $a9;
  KS_ordfeminine: byte = $aa;
  KS_guillemotleft: byte = $ab;
  KS_notsign: byte = $ac;
  KS_hyphen: byte = $ad;
  KS_registered: byte = $ae;
  KS_macron: byte = $af;
  KS_degree: byte = $b0;
  KS_plusminus: byte = $b1;
  KS_twosuperior: byte = $b2;
  KS_threesuperior: byte = $b3;
  KS_acute: byte = $b4;
  KS_mu: byte = $b5;
  KS_paragraph: byte = $b6;
  KS_periodcentered: byte = $b7;
  KS_cedilla: byte = $b8;
  KS_onesuperior: byte = $b9;
  KS_masculine: byte = $ba;
  KS_guillemotright: byte = $bb;
  KS_onequarter: byte = $bc;
  KS_onehalf: byte = $bd;
  KS_threequarters: byte = $be;
  KS_questiondown: byte = $bf;
  KS_Agrave: byte = $c0; public name 'KS_Agrave';
  KS_Aacute: byte = $c1; public name 'KS_Aacute';
  KS_Acircumflex: byte = $c2; public name 'KS_Acircumflex';
  KS_Atilde: byte = $c3; public name 'KS_Atilde';
  KS_Adiaeresis: byte = $c4; public name 'KS_Adiaeresis';
  KS_Aring: byte = $c5; public name 'KS_Aring';
  KS_AE: byte = $c6; public name 'KS_AE';
  KS_Ccedilla: byte = $c7; public name 'KS_Ccedilla';
  KS_Egrave: byte = $c8; public name 'KS_Egrave'; 
  KS_Eacute: byte = $c9; public name 'KS_Eacute';
  KS_Ecircumflex: byte = $ca; public name 'KS_Ecircumflex';
  KS_Ediaeresis: byte = $cb; public name 'KS_Ediaeresis';
  KS_Igrave: byte = $cc; public name 'KS_Igrave';
  KS_Iacute: byte = $cd; public name 'KS_Iacute';
  KS_Icircumflex: byte = $ce; public name 'KS_Icircumflex';
  KS_Idiaeresis: byte = $cf; public name 'KS_Idiaeresis';
  KS_ETH: byte = $d0; public name 'KS_ETH';
  KS_Ntilde: byte = $d1; public name 'KS_Ntilde';
  KS_Ograve: byte = $d2; public name 'KS_Ograve';
  KS_Oacute: byte = $d3; public name 'KS_Oacute';
  KS_Ocircumflex: byte = $d4; public name 'KS_Ocircumflex';
  KS_Otilde: byte = $d5; public name 'KS_Otilde';
  KS_Odiaeresis: byte = $d6; public name 'KS_Odiaeresis';
  KS_multiply: byte = $d7;
  KS_Ooblique: byte = $d8; public name 'KS_Ooblique';
  KS_Ugrave: byte = $d9; public name 'KS_Ugrave';
  KS_Uacute: byte = $da; public name 'KS_Uacute';
  KS_Ucircumflex: byte = $db; public name 'KS_Ucircumflex';
  KS_Udiaeresis: byte = $dc; public name 'KS_Udiaeresis';
  KS_Yacute: byte = $dd; public name 'KS_Yacute';
  KS_THORN: byte = $de; public name 'KS_THORN';
  KS_ssharp: byte = $df;
  KS_agrave_: byte = $e0; public name 'KS_agrave';
  KS_aacute_: byte = $e1; public name 'KS_aacute';
  KS_acircumflex_: byte = $e2; public name 'KS_acircumflex';
  KS_atilde_: byte = $e3; public name 'KS_atilde';
  KS_adiaeresis_: byte = $e4; public name 'KS_adiaeresis';
  KS_aring_: byte = $e5; public name 'KS_aring';
  KS_ae_: byte = $e6; public name 'KS_ae';
  KS_ccedilla_: byte = $e7; public name 'KS_ccedilla';
  KS_egrave_: byte = $e8; public name 'KS_egrave';
  KS_eacute_: byte = $e9; public name 'KS_eacute';
  KS_ecircumflex_: byte = $ea; public name 'KS_ecircumflex';
  KS_ediaeresis_: byte = $eb; public name 'KS_ediaeresis';
  KS_igrave_: byte = $ec; public name 'KS_igrave';
  KS_iacute_: byte = $ed; public name 'KS_iacute';
  KS_icircumflex_: byte = $ee; public name 'KS_icircumflex';
  KS_idiaeresis_: byte = $ef; public name 'KS_idiaeresis';
  KS_eth_: byte = $f0; public name 'KS_eth';
  KS_ntilde_: byte = $f1; public name 'KS_ntilde';
  KS_ograve_: byte = $f2; public name 'KS_ograve';
  KS_oacute_: byte = $f3; public name 'KS_oacute';
  KS_ocircumflex_: byte = $f4; public name 'KS_ocircumflex';
  KS_otilde_: byte = $f5; public name 'KS_otilde';
  KS_odiaeresis_: byte = $f6; public name 'KS_odiaeresis';
  KS_division: byte = $f7;
  KS_oslash: byte = $f8;
  KS_ugrave_: byte = $f9; public name 'KS_ugrave';
  KS_uacute_: byte = $fa; public name 'KS_uacute';
  KS_ucircumflex_: byte = $fb; public name 'KS_ucircumflex';
  KS_udiaeresis_: byte = $fc; public name 'KS_udiaeresis';
  KS_yacute_: byte = $fd; public name 'KS_yacute';
  KS_thorn_: byte = $fe; public name 'KS_thorn';
  KS_ydiaeresis_: byte = $ff; public name 'KS_ydiaeresis';
  KS_Odoubleacute: word = $150; public name 'KS_Odoubleacute';
  KS_odoubleacute_: word = $151; public name 'KS_odoubleacute';
  KS_Udoubleacute: word = $170; public name 'KS_Udoubleacute';
  KS_udoubleacute_: word = $171; public name 'KS_udoubleacute';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
  KS_BackSpace = $08;
  KS_Tab = $09;
  KS_Linefeed = $0a;
  KS_Clear = $0b;
  KS_Return = $0d;
  KS_Escape = $1b;
  KS_space = $20;
  KS_exclam = $21;
  KS_quotedbl = $22;
  KS_numbersign = $23;
  KS_dollar = $24;
  KS_percent = $25;
  KS_ampersand = $26;
  KS_apostrophe = $27;
  KS_parenleft = $28;
  KS_parenright = $29;
  KS_asterisk = $2a;
  KS_plus = $2b;
  KS_comma = $2c;
  KS_minus = $2d;
  KS_period = $2e;
  KS_slash = $2f;
  KS_0 = $30;
  KS_1 = $31;
  KS_2 = $32;
  KS_3 = $33;
  KS_4 = $34;
  KS_5 = $35;
  KS_6 = $36;
  KS_7 = $37;
  KS_8 = $38;
  KS_9 = $39;
  KS_colon = $3a;
  KS_semicolon = $3b;
  KS_less = $3c;
  KS_equal = $3d;
  KS_greater = $3e;
  KS_question = $3f;
  KS_at = $40;
  KS_A = $41;
  KS_B = $42;
  KS_C = $43;
  KS_D = $44;
  KS_E = $45;
  KS_F = $46;
  KS_G = $47;
  KS_H = $48;
  KS_I = $49;
  KS_J = $4a;
  KS_K = $4b;
  KS_L = $4c;
  KS_M = $4d;
  KS_N = $4e;
  KS_O = $4f;
  KS_P = $50;
  KS_Q = $51;
  KS_R = $52;
  KS_S = $53;
  KS_T = $54;
  KS_U = $55;
  KS_V = $56;
  KS_W = $57;
  KS_X = $58;
  KS_Y = $59;
  KS_Z = $5a;
  KS_bracketleft = $5b;
  KS_backslash = $5c;
  KS_bracketright = $5d;
  KS_asciicircum = $5e;
  KS_underscore = $5f;
  KS_grave = $60;
  KS_a = $61;
  KS_b = $62;
  KS_c = $63;
  KS_d = $64;
  KS_e = $65;
  KS_f = $66;
  KS_g = $67;
  KS_h = $68;
  KS_i = $69;
  KS_j = $6a;
  KS_k = $6b;
  KS_l = $6c;
  KS_m = $6d;
  KS_n = $6e;
  KS_o = $6f;
  KS_p = $70;
  KS_q = $71;
  KS_r = $72;
  KS_s = $73;
  KS_t = $74;
  KS_u = $75;
  KS_v = $76;
  KS_w = $77;
  KS_x = $78;
  KS_y = $79;
  KS_z = $7a;
  KS_braceleft = $7b;
  KS_bar = $7c;
  KS_braceright = $7d;
  KS_asciitilde = $7e;
  KS_Delete = $7f;
  KS_nobreakspace = $a0;
  KS_exclamdown = $a1;
  KS_cent = $a2;
  KS_sterling = $a3;
  KS_currency = $a4;
  KS_yen = $a5;
  KS_brokenbar = $a6;
  KS_section = $a7;
  KS_diaeresis = $a8;
  KS_copyright = $a9;
  KS_ordfeminine = $aa;
  KS_guillemotleft = $ab;
  KS_notsign = $ac;
  KS_hyphen = $ad;
  KS_registered = $ae;
  KS_macron = $af;
  KS_degree = $b0;
  KS_plusminus = $b1;
  KS_twosuperior = $b2;
  KS_threesuperior = $b3;
  KS_acute = $b4;
  KS_mu = $b5;
  KS_paragraph = $b6;
  KS_periodcentered = $b7;
  KS_cedilla = $b8;
  KS_onesuperior = $b9;
  KS_masculine = $ba;
  KS_guillemotright = $bb;
  KS_onequarter = $bc;
  KS_onehalf = $bd;
  KS_threequarters = $be;
  KS_questiondown = $bf;
  KS_Agrave = $c0;
  KS_Aacute = $c1;
  KS_Acircumflex = $c2;
  KS_Atilde = $c3;
  KS_Adiaeresis = $c4;
  KS_Aring = $c5;
  KS_AE = $c6;
  KS_Ccedilla = $c7;
  KS_Egrave = $c8;
  KS_Eacute = $c9;
  KS_Ecircumflex = $ca;
  KS_Ediaeresis = $cb;
  KS_Igrave = $cc;
  KS_Iacute = $cd;
  KS_Icircumflex = $ce;
  KS_Idiaeresis = $cf;
  KS_ETH = $d0;
  KS_Ntilde = $d1;
  KS_Ograve = $d2;
  KS_Oacute = $d3;
  KS_Ocircumflex = $d4;
  KS_Otilde = $d5;
  KS_Odiaeresis = $d6;
  KS_multiply = $d7;
  KS_Ooblique = $d8;
  KS_Ugrave = $d9;
  KS_Uacute = $da;
  KS_Ucircumflex = $db;
  KS_Udiaeresis = $dc;
  KS_Yacute = $dd;
  KS_THORN = $de;
  KS_ssharp = $df;
  KS_agrave = $e0;
  KS_aacute = $e1;
  KS_acircumflex = $e2;
  KS_atilde = $e3;
  KS_adiaeresis = $e4;
  KS_aring = $e5;
  KS_ae = $e6;
  KS_ccedilla = $e7;
  KS_egrave = $e8;
  KS_eacute = $e9;
  KS_ecircumflex = $ea;
  KS_ediaeresis = $eb;
  KS_igrave = $ec;
  KS_iacute = $ed;
  KS_icircumflex = $ee;
  KS_idiaeresis = $ef;
  KS_eth = $f0;
  KS_ntilde = $f1;
  KS_ograve = $f2;
  KS_oacute = $f3;
  KS_ocircumflex = $f4;
  KS_otilde = $f5;
  KS_odiaeresis = $f6;
  KS_division = $f7;
  KS_oslash = $f8;
  KS_ugrave = $f9;
  KS_uacute = $fa;
  KS_ucircumflex = $fb;
  KS_udiaeresis = $fc;
  KS_yacute = $fd;
  KS_thorn = $fe;
  KS_ydiaeresis = $ff;
  KS_Odoubleacute = $150;
  KS_odoubleacute = $151;
  KS_Udoubleacute = $170;
  KS_udoubleacute = $171;
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
  (*
 * Group Dead (dead accents)
  *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  KS_dead_grave: word = $0300;
  KS_dead_acute: word = $0301;
  KS_dead_circumflex: word = $0302;
  KS_dead_tilde: word = $0303;
  KS_dead_diaeresis: word = $0308;
  KS_dead_abovering: word = $030a;
  KS_dead_cedilla: word = $0327;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  KS_dead_grave = $0300;
  KS_dead_acute = $0301;
  KS_dead_circumflex = $0302;
  KS_dead_tilde = $0303;
  KS_dead_diaeresis = $0308;
  KS_dead_abovering = $030a;
  KS_dead_cedilla = $0327;
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
  (*
 * Group Cyrillic (koi8-r)
  *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  KS_Cyrillic_A: byte = $e1; public name 'KS_Cyrillic_A';
  KS_Cyrillic_BE: byte = $e2; public name 'KS_Cyrillic_BE';
  KS_Cyrillic_VE: byte = $f7; public name 'KS_Cyrillic_VE';
  KS_Cyrillic_GE: byte = $e7; public name 'KS_Cyrillic_GE';
  KS_Cyrillic_DE: byte = $e4; public name 'KS_Cyrillic_DE';
  KS_Cyrillic_IE: byte = $e5; public name 'KS_Cyrillic_IE';
  KS_Cyrillic_YO: byte = $b3; public name 'KS_Cyrillic_YO';
  KS_Cyrillic_ZHE: byte = $f6; public name 'KS_Cyrillic_ZHE';
  KS_Cyrillic_ZE: byte = $fa; public name 'KS_Cyrillic_ZE';
  KS_Cyrillic_I: byte = $e9; public name 'KS_Cyrillic_I';                
  KS_Cyrillic_ISHORT: byte = $ea; public name 'KS_Cyrillic_ISHORT';
  KS_Cyrillic_IUKR: byte = $b6; public name 'KS_Cyrillic_IUKR';
  KS_Cyrillic_YI: byte = $b7; public name 'KS_Cyrillic_YI';
  KS_Cyrillic_KA: byte = $eb; public name 'KS_Cyrillic_KA';
  KS_Cyrillic_EL: byte = $ec; public name 'KS_Cyrillic_EL';
  KS_Cyrillic_EM: byte = $ed; public name 'KS_Cyrillic_EM';
  KS_Cyrillic_EN: byte = $ee; public name 'KS_Cyrillic_EN';
  KS_Cyrillic_O: byte = $ef; public name 'KS_Cyrillic_O';
  KS_Cyrillic_PE: byte = $f0; public name 'KS_Cyrillic_PE';
  KS_Cyrillic_ER: byte = $f2; public name 'KS_Cyrillic_ER';
  KS_Cyrillic_ES: byte = $f3; public name 'KS_Cyrillic_ES';
  KS_Cyrillic_TE: byte = $f4; public name 'KS_Cyrillic_TE';
  KS_Cyrillic_U: byte = $f5; public name 'KS_Cyrillic_U';
  KS_Cyrillic_EF: byte = $e6; public name 'KS_Cyrillic_EF';
  KS_Cyrillic_HA: byte = $e8; public name 'KS_Cyrillic_HA';               
  KS_Cyrillic_TSE: byte = $e3; public name 'KS_Cyrillic_TSE';
  KS_Cyrillic_CHE: byte = $fe; public name 'KS_Cyrillic_CHE';
  KS_Cyrillic_SHA: byte = $fb; public name 'KS_Cyrillic_SHA';
  KS_Cyrillic_SCHA: byte = $fd; public name 'KS_Cyrillic_SCHA';
  KS_Cyrillic_HSIGHN: byte = $ff; public name 'KS_Cyrillic_HSIGHN';
  KS_Cyrillic_YERU: byte = $f9; public name 'KS_Cyrillic_YERU';
  KS_Cyrillic_SSIGHN: byte = $f8; public name 'KS_Cyrillic_SSIGHN';
  KS_Cyrillic_E: byte = $fc; public name 'KS_Cyrillic_E';
  KS_Cyrillic_YU: byte = $e0; public name 'KS_Cyrillic_YU';
  KS_Cyrillic_YA: byte = $f1; public name 'KS_Cyrillic_YA';
  KS_Cyrillic_a_: byte = $c1; public name 'KS_Cyrillic_a';
  KS_Cyrillic_be_: byte = $c2; public name 'KS_Cyrillic_be';
  KS_Cyrillic_ve_: byte = $d7; public name 'KS_Cyrillic_ve';
  KS_Cyrillic_ge_: byte = $c7; public name 'KS_Cyrillic_ge';
  KS_Cyrillic_de_: byte = $c4; public name 'KS_Cyrillic_de';
  KS_Cyrillic_ie_: byte = $c5; public name 'KS_Cyrillic_ie';
  KS_Cyrillic_yo_: byte = $a3; public name 'KS_Cyrillic_yo';
  KS_Cyrillic_zhe_: byte = $d6; public name 'KS_Cyrillic_zhe';
  KS_Cyrillic_ze_: byte = $da; public name 'KS_Cyrillic_ze';
  KS_Cyrillic_i_: byte = $c9; public name 'KS_Cyrillic_i';                
  KS_Cyrillic_ishort_: byte = $ca; public name 'KS_Cyrillic_ishort';
  KS_Cyrillic_iukr_: byte = $a6; public name 'KS_Cyrillic_iukr';
  KS_Cyrillic_yi_: byte = $a7; public name 'KS_Cyrillic_yi';
  KS_Cyrillic_ka_: byte = $cb; public name 'KS_Cyrillic_ka';
  KS_Cyrillic_el_: byte = $cc; public name 'KS_Cyrillic_el';
  KS_Cyrillic_em_: byte = $cd; public name 'KS_Cyrillic_em';
  KS_Cyrillic_en_: byte = $ce; public name 'KS_Cyrillic_en';
  KS_Cyrillic_o_: byte = $cf; public name 'KS_Cyrillic_o';
  KS_Cyrillic_pe_: byte = $d0; public name 'KS_Cyrillic_pe';
  KS_Cyrillic_er_: byte = $d2; public name 'KS_Cyrillic_er';
  KS_Cyrillic_es_: byte = $d3; public name 'KS_Cyrillic_es';
  KS_Cyrillic_te_: byte = $d4; public name 'KS_Cyrillic_te';
  KS_Cyrillic_u_: byte = $d5; public name 'KS_Cyrillic_u';
  KS_Cyrillic_ef_: byte = $c6; public name 'KS_Cyrillic_ef';
  KS_Cyrillic_ha_: byte = $c8; public name 'KS_Cyrillic_ha';
  KS_Cyrillic_tse_: byte = $c3; public name 'KS_Cyrillic_tse';
  KS_Cyrillic_che_: byte = $de; public name 'KS_Cyrillic_che';
  KS_Cyrillic_sha_: byte = $db; public name 'KS_Cyrillic_sha';
  KS_Cyrillic_scha_: byte = $dd; public name 'KS_Cyrillic_scha';
  KS_Cyrillic_hsighn_: byte = $df; public name 'KS_Cyrillic_hsighn';
  KS_Cyrillic_yeru_: byte = $d9; public name 'KS_Cyrillic_yeru';
  KS_Cyrillic_ssighn_: byte = $d8; public name 'KS_Cyrillic_ssighn';
  KS_Cyrillic_e_: byte = $dc; public name 'KS_Cyrillic_e';
  KS_Cyrillic_yu_: byte = $c0; public name 'KS_Cyrillic_yu';
  KS_Cyrillic_ya_: byte = $d1; public name 'KS_Cyrillic_ya';
  KS_Cyrillic_gheukr_: byte = $ad; public name 'KS_Cyrillic_gheukr';
  KS_Cyrillic_GHEUKR: byte = $bd; public name 'KS_Cyrillic_GHEUKR';
  KS_Cyrillic_yeukr_: byte = $a4; public name 'KS_Cyrillic_yeukr';
  KS_Cyrillic_YEUKR: byte = $b4; public name 'KS_Cyrillic_YEUKR';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  KS_Cyrillic_A = $e1;
  KS_Cyrillic_BE = $e2;
  KS_Cyrillic_VE = $f7;
  KS_Cyrillic_GE = $e7;
  KS_Cyrillic_DE = $e4;
  KS_Cyrillic_IE = $e5;
  KS_Cyrillic_YO = $b3;
  KS_Cyrillic_ZHE = $f6;
  KS_Cyrillic_ZE = $fa;
  KS_Cyrillic_I = $e9;
  KS_Cyrillic_ISHORT = $ea;
  KS_Cyrillic_IUKR = $b6;
  KS_Cyrillic_YI = $b7;
  KS_Cyrillic_KA = $eb;
  KS_Cyrillic_EL = $ec;
  KS_Cyrillic_EM = $ed;
  KS_Cyrillic_EN = $ee;
  KS_Cyrillic_O = $ef;
  KS_Cyrillic_PE = $f0;
  KS_Cyrillic_ER = $f2;
  KS_Cyrillic_ES = $f3;
  KS_Cyrillic_TE = $f4;
  KS_Cyrillic_U = $f5;
  KS_Cyrillic_EF = $e6;
  KS_Cyrillic_HA = $e8;
  KS_Cyrillic_TSE = $e3;
  KS_Cyrillic_CHE = $fe;
  KS_Cyrillic_SHA = $fb;
  KS_Cyrillic_SCHA = $fd;
  KS_Cyrillic_HSIGHN = $ff;
  KS_Cyrillic_YERU = $f9;
  KS_Cyrillic_SSIGHN = $f8;
  KS_Cyrillic_E = $fc;
  KS_Cyrillic_YU = $e0;
  KS_Cyrillic_YA = $f1;
  KS_Cyrillic_a = $c1;
  KS_Cyrillic_be = $c2;
  KS_Cyrillic_ve = $d7;
  KS_Cyrillic_ge = $c7;
  KS_Cyrillic_de = $c4;
  KS_Cyrillic_ie = $c5;
  KS_Cyrillic_yo = $a3;
  KS_Cyrillic_zhe = $d6;
  KS_Cyrillic_ze = $da;
  KS_Cyrillic_i = $c9;
  KS_Cyrillic_ishort = $ca;
  KS_Cyrillic_iukr = $a6;
  KS_Cyrillic_yi = $a7;
  KS_Cyrillic_ka = $cb;
  KS_Cyrillic_el = $cc;
  KS_Cyrillic_em = $cd;
  KS_Cyrillic_en = $ce;
  KS_Cyrillic_o = $cf;
  KS_Cyrillic_pe = $d0;
  KS_Cyrillic_er = $d2;
  KS_Cyrillic_es = $d3;
  KS_Cyrillic_te = $d4;
  KS_Cyrillic_u = $d5;
  KS_Cyrillic_ef = $c6;
  KS_Cyrillic_ha = $c8;
  KS_Cyrillic_tse = $c3;
  KS_Cyrillic_che = $de;
  KS_Cyrillic_sha = $db;
  KS_Cyrillic_scha = $dd;
  KS_Cyrillic_hsighn = $df;
  KS_Cyrillic_yeru = $d9;
  KS_Cyrillic_ssighn = $d8;
  KS_Cyrillic_e = $dc;
  KS_Cyrillic_yu = $c0;
  KS_Cyrillic_ya = $d1;
  KS_Cyrillic_gheukr = $ad;
  KS_Cyrillic_GHEUKR = $bd;
  KS_Cyrillic_yeukr = $a4;
  KS_Cyrillic_YEUKR = $b4;
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
  (*
 * Group Latin-2 (iso8859-2)
  *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  KS_L2_Aogonek: byte = $a1; public name 'KS_L2_Aogonek';
  KS_L2_breve: byte = $a2;
  KS_L2_Lstroke: byte = $a3;
  KS_L2_Lcaron: byte = $a5;
  KS_L2_Sacute: byte = $a6;
  KS_L2_Scaron: byte = $a9;
  KS_L2_Scedilla: byte = $aa;
  KS_L2_Tcaron: byte = $ab;
  KS_L2_Zacute: byte = $ac;
  KS_L2_Zcaron: byte = $ae;
  KS_L2_Zdotabove: byte = $af;
  KS_L2_aogonek_: byte = $b1; public name 'KS_L2_aogonek';
  KS_L2_ogonek: byte = $b2;
  KS_L2_lstroke_: byte = $b3; public name 'KS_L2_lstroke';
  KS_L2_lcaron_: byte = $b5; public name 'KS_L2_lcaron';
  KS_L2_sacute_: byte = $b6; public name 'KS_L2_sacute';
  KS_L2_caron: byte = $b7;
  KS_L2_scaron_: byte = $b9; public name 'KS_L2_scaron';
  KS_L2_scedilla_: byte = $ba; public name 'KS_L2_scedilla';
  KS_L2_tcaron_: byte = $bb; public name 'KS_L2_tcaron';
  KS_L2_zacute_: byte = $bc; public name 'KS_L2_zacute';
  KS_L2_dblacute: byte = $bd;
  KS_L2_zcaron_: byte = $be; public name 'KS_L2_zcaron';
  KS_L2_zdotabove_: byte = $bf; public name 'KS_L2_zdotabove';
  KS_L2_Racute: byte = $c0;
  KS_L2_Abreve: byte = $c3;
  KS_L2_Lacute: byte = $c5;
  KS_L2_Cacute: byte = $c6;
  KS_L2_Ccaron: byte = $c8;
  KS_L2_Eogonek: byte = $ca;
  KS_L2_Ecaron: byte = $cc;
  KS_L2_Dcaron: byte = $cf;
  KS_L2_Dstroke: byte = $d0;
  KS_L2_Nacute: byte = $d1;
  KS_L2_Ncaron: byte = $d2;
  KS_L2_Odoubleacute: byte = $d5;
  KS_L2_Rcaron: byte = $d8;
  KS_L2_Uring: byte = $d9;
  KS_L2_Udoubleacute: byte = $db;
  KS_L2_Tcedilla: byte = $de;
  KS_L2_racute_: byte = $e0; public name 'KS_L2_racute';
  KS_L2_abreve_: byte = $e3; public name 'KS_L2_abreve';
  KS_L2_lacute_: byte = $e5; public name 'KS_L2_lacute';               
  KS_L2_cacute_: byte = $e6; public name 'KS_L2_cacute';
  KS_L2_ccaron_: byte = $e8; public name 'KS_L2_ccaron';
  KS_L2_eogonek_: byte = $ea; public name 'KS_L2_eogonek';
  KS_L2_ecaron_: byte = $ec; public name 'KS_L2_ecaron';
  KS_L2_dcaron_: byte = $ef; public name 'KS_L2_dcaron';                
  KS_L2_dstroke_: byte = $f0; public name 'KS_L2_dstroke';
  KS_L2_nacute_: byte = $f1; public name 'KS_L2_nacute';
  KS_L2_odoubleacute_: byte = $f5; public name 'KS_L2_odoubleacute';
  KS_L2_rcaron_: byte = $f8; public name 'KS_L2_rcaron';
  KS_L2_uring_: byte = $f9; public name 'KS_L2_uring';
  KS_L2_udoubleacute_: byte = $fb; public name 'KS_L2_udoubleacute';
  KS_L2_tcedilla_: byte = $fe; public name 'KS_L2_tcedilla';
  KS_L2_dotabove: byte = $ff;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  KS_L2_Aogonek = $a1;
  KS_L2_breve = $a2;
  KS_L2_Lstroke = $a3;
  KS_L2_Lcaron = $a5;
  KS_L2_Sacute = $a6;
  KS_L2_Scaron = $a9;
  KS_L2_Scedilla = $aa;
  KS_L2_Tcaron = $ab;
  KS_L2_Zacute = $ac;
  KS_L2_Zcaron = $ae;
  KS_L2_Zdotabove = $af;
  KS_L2_aogonek = $b1;
  KS_L2_ogonek = $b2;
  KS_L2_lstroke = $b3;
  KS_L2_lcaron = $b5;
  KS_L2_sacute = $b6;
  KS_L2_caron = $b7;
  KS_L2_scaron = $b9;
  KS_L2_scedilla = $ba;
  KS_L2_tcaron = $bb;
  KS_L2_zacute = $bc;
  KS_L2_dblacute = $bd;
  KS_L2_zcaron = $be;
  KS_L2_zdotabove = $bf;
  KS_L2_Racute = $c0;
  KS_L2_Abreve = $c3;
  KS_L2_Lacute = $c5;
  KS_L2_Cacute = $c6;
  KS_L2_Ccaron = $c8;
  KS_L2_Eogonek = $ca;
  KS_L2_Ecaron = $cc;
  KS_L2_Dcaron = $cf;
  KS_L2_Dstroke = $d0;
  KS_L2_Nacute = $d1;
  KS_L2_Ncaron = $d2;
  KS_L2_Odoubleacute = $d5;
  KS_L2_Rcaron = $d8;
  KS_L2_Uring = $d9;
  KS_L2_Udoubleacute = $db;
  KS_L2_Tcedilla = $de;
  KS_L2_racute = $e0;
  KS_L2_abreve = $e3;
  KS_L2_lacute = $e5;
  KS_L2_cacute = $e6;
  KS_L2_ccaron = $e8;
  KS_L2_eogonek = $ea;
  KS_L2_ecaron = $ec;
  KS_L2_dcaron = $ef;
  KS_L2_dstroke = $f0;
  KS_L2_nacute = $f1;
  KS_L2_odoubleacute = $f5;
  KS_L2_rcaron = $f8;
  KS_L2_uring = $f9;
  KS_L2_udoubleacute = $fb;
  KS_L2_tcedilla = $fe;
  KS_L2_dotabove = $ff;
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
  (*
 * Group Latin-5 (iso8859-9)
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  KS_L5_Gbreve: byte = $d0; public name 'KS_L5_Gbreve';
  KS_L5_Idotabove: byte = $dd;
  KS_L5_Scedilla: byte = $de; public name 'KS_L5_Scedilla';
  KS_L5_gbreve_: byte = $f0; public name 'KS_L5_gbreve';
  KS_L5_idotless: byte = $fd;
  KS_L5_scedilla_: byte = $fe; public name 'KS_L5_scedilla';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  KS_L5_Gbreve = $d0;
  KS_L5_Idotabove = $dd;
  KS_L5_Scedilla = $de;
  KS_L5_gbreve = $f0;
  KS_L5_idotless = $fd;
  KS_L5_scedilla = $fe;
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
  (*
 * Group Latin-7 (iso8859-13)
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  KS_L7_rightdblquot: byte = $a1;
  KS_L7_dbllow9quot: byte = $a5;
  KS_L7_Ostroke: byte = $a8;
  KS_L7_Rcedilla: byte = $aa;
  KS_L7_AE: byte = $af;
  KS_L7_leftdblquot: byte = $b4;
  KS_L7_ostroke_: byte = $b8; public name 'KS_L7_ostroke';
  KS_L7_rcedilla_: byte = $ba; public name 'KS_L7_rcedilla';
  KS_L7_ae_: byte = $bf; public name 'KS_L7_ae';
  KS_L7_Aogonek: byte = $c0;
  KS_L7_Iogonek: byte = $c1;
  KS_L7_Amacron: byte = $c2;
  KS_L7_Cacute: byte = $c3;
  KS_L7_Eogonek: byte = $c6;
  KS_L7_Emacron: byte = $c7;
  KS_L7_Ccaron: byte = $c8;
  KS_L7_Zacute: byte = $ca;
  KS_L7_Edot: byte = $cb;
  KS_L7_Gcedilla: byte = $cc;
  KS_L7_Kcedilla: byte = $cd;
  KS_L7_Imacron: byte = $ce;
  KS_L7_Lcedilla: byte = $cf;
  KS_L7_Scaron: byte = $d0;
  KS_L7_Nacute: byte = $d1;
  KS_L7_Ncedilla: byte = $d2;
  KS_L7_Omacron: byte = $d4;
  KS_L7_Uogonek: byte = $d8;
  KS_L7_Lstroke: byte = $d9;
  KS_L7_Sacute: byte = $da;
  KS_L7_Umacron: byte = $db;
  KS_L7_Zdot: byte = $dd;
  KS_L7_Zcaron: byte = $de;
  KS_L7_aogonek_: byte = $e0; public name 'KS_L7_aogonek';
  KS_L7_iogonek_: byte = $e1; public name 'KS_L7_iogonek';
  KS_L7_amacron_: byte = $e2; public name 'KS_L7_amacron';
  KS_L7_cacute_: byte = $e3; public name 'KS_L7_cacute';
  KS_L7_eogonek_: byte = $e6; public name 'KS_L7_eogonek';
  KS_L7_emacron_: byte = $e7; public name 'KS_L7_emacron';
  KS_L7_ccaron_: byte = $e8; public name 'KS_L7_ccaron';
  KS_L7_zacute_: byte = $ea; public name 'KS_L7_zacute';
  KS_L7_edot_: byte = $eb; public name 'KS_L7_edot';
  KS_L7_gcedilla_: byte = $ec; public name 'KS_L7_gcedilla';
  KS_L7_kcedilla_: byte = $ed; public name 'KS_L7_kcedilla';
  KS_L7_imacron_: byte = $ee; public name 'KS_L7_imacron';
  KS_L7_lcedilla_: byte = $ef; public name 'KS_L7_lcedilla';
  KS_L7_scaron_: byte = $f0; public name 'KS_L7_scaron';
  KS_L7_nacute_: byte = $f1; public name 'KS_L7_nacute';
  KS_L7_ncedilla_: byte = $f2; public name 'KS_L7_ncedilla';
  KS_L7_omacron_: byte = $f4; public name 'KS_L7_omacron';
  KS_L7_uogonek_: byte = $f8; public name 'KS_L7_uogonek';
  KS_L7_lstroke_: byte = $f9; public name 'KS_L7_lstroke';
  KS_L7_sacute_: byte = $fa; public name 'KS_L7_sacute';
  KS_L7_umacron_: byte = $fb; public name 'KS_L7_umacron';
  KS_L7_zdot_: byte = $fd; public name 'KS_L7_zdot';
  KS_L7_zcaron_: byte = $fe; public name 'KS_L7_zcaron';
  KS_L7_rightsnglquot: byte = $ff;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  KS_L7_rightdblquot = $a1;
  KS_L7_dbllow9quot = $a5;
  KS_L7_Ostroke = $a8;
  KS_L7_Rcedilla = $aa;
  KS_L7_AE = $af;
  KS_L7_leftdblquot = $b4;
  KS_L7_ostroke = $b8;
  KS_L7_rcedilla = $ba;
  KS_L7_ae = $bf;
  KS_L7_Aogonek = $c0;
  KS_L7_Iogonek = $c1;
  KS_L7_Amacron = $c2;
  KS_L7_Cacute = $c3;
  KS_L7_Eogonek = $c6;
  KS_L7_Emacron = $c7;
  KS_L7_Ccaron = $c8;
  KS_L7_Zacute = $ca;
  KS_L7_Edot = $cb;
  KS_L7_Gcedilla = $cc;
  KS_L7_Kcedilla = $cd;
  KS_L7_Imacron = $ce;
  KS_L7_Lcedilla = $cf;
  KS_L7_Scaron = $d0;
  KS_L7_Nacute = $d1;
  KS_L7_Ncedilla = $d2;
  KS_L7_Omacron = $d4;
  KS_L7_Uogonek = $d8;
  KS_L7_Lstroke = $d9;
  KS_L7_Sacute = $da;
  KS_L7_Umacron = $db;
  KS_L7_Zdot = $dd;
  KS_L7_Zcaron = $de;
  KS_L7_aogonek = $e0;
  KS_L7_iogonek = $e1;
  KS_L7_amacron = $e2;
  KS_L7_cacute = $e3;
  KS_L7_eogonek = $e6;
  KS_L7_emacron = $e7;
  KS_L7_ccaron = $e8;
  KS_L7_zacute = $ea;
  KS_L7_edot = $eb;
  KS_L7_gcedilla = $ec;
  KS_L7_kcedilla = $ed;
  KS_L7_imacron = $ee;
  KS_L7_lcedilla = $ef;
  KS_L7_scaron = $f0;
  KS_L7_nacute = $f1;
  KS_L7_ncedilla = $f2;
  KS_L7_omacron = $f4;
  KS_L7_uogonek = $f8;
  KS_L7_lstroke = $f9;
  KS_L7_sacute = $fa;
  KS_L7_umacron = $fb;
  KS_L7_zdot = $fd;
  KS_L7_zcaron = $fe;
  KS_L7_rightsnglquot = $ff;
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
  (*
 * Group 1 (modifiers)
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
  KS_Shift_L = $f101;
  KS_Shift_R = $f102;
  KS_Control_L = $f103;
  KS_Control_R = $f104;
  KS_Caps_Lock = $f105;
  KS_Shift_Lock = $f106;
  KS_Alt_L = $f107;
  KS_Alt_R = $f108;
  KS_Multi_key = $f109;
  KS_Mode_switch = $f10a;
  KS_Num_Lock = $f10b;
  KS_Hold_Screen = $f10c;
  KS_Cmd = $f10d;
  KS_Cmd1 = $f10e;
  KS_Cmd2 = $f10f;
  KS_Meta_L = $f110;
  KS_Meta_R = $f111;
  KS_Zenkaku_Hankaku = $f112;  (* Zenkaku/Hankaku toggle  *)
  KS_Hiragana_Katakana = $f113;  (* Hiragana/Katakana toggle  *)
  KS_Henkan_Mode = $f114;  (* Start/Stop Conversion  *)
  KS_Henkan = $f115;  (* Alias for Henkan_Mode  *)
  KS_Muhenkan = $f116;  (* Cancel Conversion  *)
  KS_Mode_Lock = $f117;
  (*
 * Group 2 (keypad) character in low byte
  *)


  KS_KP_F1 = $f291;
  KS_KP_F2 = $f292;
  KS_KP_F3 = $f293;
  KS_KP_F4 = $f294;
  KS_KP_Home = $f295;
  KS_KP_Left = $f296;
  KS_KP_Up = $f297;
  KS_KP_Right = $f298;
  KS_KP_Down = $f299;
  KS_KP_Prior = $f29a;
  KS_KP_Next = $f29b;
  KS_KP_End = $f29c;
  KS_KP_Begin = $f29d;
  KS_KP_Insert = $f29e;
  KS_KP_Delete = $f29f;
  KS_KP_Space = $f220;
  KS_KP_Tab = $f209;
  KS_KP_Enter = $f20d;
  KS_KP_Equal = $f23d;
  KS_KP_Numbersign = $f223;
  KS_KP_Multiply = $f22a;
  KS_KP_Add = $f22b;
  KS_KP_Separator = $f22c;
  KS_KP_Subtract = $f22d;
  KS_KP_Decimal = $f22e;
  KS_KP_Divide = $f22f;
  KS_KP_0 = $f230;
  KS_KP_1 = $f231;
  KS_KP_2 = $f232;
  KS_KP_3 = $f233;
  KS_KP_4 = $f234;
  KS_KP_5 = $f235;
  KS_KP_6 = $f236;
  KS_KP_7 = $f237;
  KS_KP_8 = $f238;
  KS_KP_9 = $f239;
  (*
 * Group 3 (function)
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  KS_f1_: word = $f300; public name 'KS_f1';
  KS_f2_: word = $f301; public name 'KS_f2';
  KS_f3_: word = $f302; public name 'KS_f3';
  KS_f4_: word = $f303; public name 'KS_f4';
  KS_f5_: word = $f304; public name 'KS_f5';
  KS_f6_: word = $f305; public name 'KS_f6';               
  KS_f7_: word = $f306; public name 'KS_f7';
  KS_f8_: word = $f307; public name 'KS_f8';
  KS_f9_: word = $f308; public name 'KS_f9';
  KS_f10_: word = $f309; public name 'KS_f10';
  KS_f11_: word = $f30a; public name 'KS_f11';
  KS_f12_: word = $f30b; public name 'KS_f12';
  KS_f13_: word = $f30c; public name 'KS_f13';
  KS_f14_: word = $f30d; public name 'KS_f14';
  KS_f15_: word = $f30e; public name 'KS_f15';
  KS_f16_: word = $f30f; public name 'KS_f16';               
  KS_f17_: word = $f310; public name 'KS_f17';
  KS_f18_: word = $f311; public name 'KS_f18';
  KS_f19_: word = $f312; public name 'KS_f19';
  KS_f20_: word = $f313; public name 'KS_f20';
  KS_F1: word = $f340; public name 'KS_F1';
  KS_F2: word = $f341; public name 'KS_F2';
  KS_F3: word = $f342; public name 'KS_F3';
  KS_F4: word = $f343; public name 'KS_F4';               
  KS_F5: word = $f344; public name 'KS_F5';
  KS_F6: word = $f345; public name 'KS_F6';
  KS_F7: word = $f346; public name 'KS_F7';
  KS_F8: word = $f347; public name 'KS_F8';
  KS_F9: word = $f348; public name 'KS_F9';
  KS_F10: word = $f349; public name 'KS_F10';
  KS_F11: word = $f34a; public name 'KS_F11';
  KS_F12: word = $f34b; public name 'KS_F12';
  KS_F13: word = $f34c; public name 'KS_F13';
  KS_F14: word = $f34d; public name 'KS_F14';
  KS_F15: word = $f34e; public name 'KS_F15';
  KS_F16: word = $f34f; public name 'KS_F16';
  KS_F17: word = $f350; public name 'KS_F17';
  KS_F18: word = $f351; public name 'KS_F18';
  KS_F19: word = $f352; public name 'KS_F19';
  KS_F20: word = $f353; public name 'KS_F20';
const
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  KS_f1 = $f300;
  KS_f2 = $f301;
  KS_f3 = $f302;
  KS_f4 = $f303;
  KS_f5 = $f304;
  KS_f6 = $f305;
  KS_f7 = $f306;
  KS_f8 = $f307;
  KS_f9 = $f308;
  KS_f10 = $f309;
  KS_f11 = $f30a;
  KS_f12 = $f30b;
  KS_f13 = $f30c;
  KS_f14 = $f30d;
  KS_f15 = $f30e;
  KS_f16 = $f30f;
  KS_f17 = $f310;
  KS_f18 = $f311;
  KS_f19 = $f312;
  KS_f20 = $f313;
  KS_F1 = $f340;
  KS_F2 = $f341;
  KS_F3 = $f342;
  KS_F4 = $f343;
  KS_F5 = $f344;
  KS_F6 = $f345;
  KS_F7 = $f346;
  KS_F8 = $f347;
  KS_F9 = $f348;
  KS_F10 = $f349;
  KS_F11 = $f34a;
  KS_F12 = $f34b;
  KS_F13 = $f34c;
  KS_F14 = $f34d;
  KS_F15 = $f34e;
  KS_F16 = $f34f;
  KS_F17 = $f350;
  KS_F18 = $f351;
  KS_F19 = $f352;
  KS_F20 = $f353;
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
  KS_Home = $f381;
  KS_Prior = $f382;
  KS_Next = $f383;
  KS_Up = $f384;
  KS_Down = $f385;
  KS_Left = $f386;
  KS_Right = $f387;
  KS_End = $f388;
  KS_Insert = $f389;
  KS_Help = $f38a;
  KS_Execute = $f38b;
  KS_Find = $f38c;
  KS_Select = $f38d;
  KS_Again = $f38e;
  KS_Props = $f38f;
  KS_Undo = $f390;
  KS_Front = $f391;
  KS_Copy = $f392;
  KS_Open = $f393;
  KS_Paste = $f394;
  KS_Cut = $f395;
  KS_Menu = $f3c0;
  KS_Pause = $f3c1;
  KS_Print_Screen = $f3c2;
  KS_AudioMute = $f3d1;
  KS_AudioLower = $f3d2;
  KS_AudioRaise = $f3d3;
  (*
 * Group 4 (command)
  *)


  KS_Cmd_Screen0 = $f400;
  KS_Cmd_Screen1 = $f401;
  KS_Cmd_Screen2 = $f402;
  KS_Cmd_Screen3 = $f403;
  KS_Cmd_Screen4 = $f404;
  KS_Cmd_Screen5 = $f405;
  KS_Cmd_Screen6 = $f406;
  KS_Cmd_Screen7 = $f407;
  KS_Cmd_Screen8 = $f408;
  KS_Cmd_Screen9 = $f409;
  KS_Cmd_Screen10 = $f40a;
  KS_Cmd_Screen11 = $f40b;
  KS_Cmd_Debugger = $f420;
  KS_Cmd_ResetEmul = $f421;
  KS_Cmd_ResetClose = $f422;
  KS_Cmd_BacklightOn = $f423;
  KS_Cmd_BacklightOff = $f424;
  KS_Cmd_BacklightToggle = $f425;
  KS_Cmd_BrightnessUp = $f426;
  KS_Cmd_BrightnessDown = $f427;
  KS_Cmd_BrightnessRotate = $f428;
  KS_Cmd_ContrastUp = $f429;
  KS_Cmd_ContrastDown = $f42a;
  KS_Cmd_ContrastRotate = $f42b;
  KS_Cmd_ScrollBack = $f42c;
  KS_Cmd_ScrollFwd = $f42d;
  KS_Cmd_KbdReset = $f42e;
  (*
 * Group 5 (internal)
  *)


  KS_voidSymbol = $f500;
  (*ENDKEYSYMDECL *)


  (*
 * keysym groups
  *)


  KS_GROUP_Mod = $f100;
  KS_GROUP_Keypad = $f200;
  KS_GROUP_Function = $f300;
  KS_GROUP_Command = $f400;
  KS_GROUP_Internal = $f500;
  KS_GROUP_Dead = $f801;  (* not encoded in keysym  *)
  KS_GROUP_Ascii = $f802;  (* not encoded in keysym  *)
  KS_GROUP_Keycode = $f803;  (* not encoded in keysym  *)
  KS_NUMKEYCODES = $1000;

function KS_KEYCODE(v: cint): cint; inline;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
function KS_KEYCODE(v: cint): cint; inline;
>>>>>>> graemeg/cpstrnew
=======
function KS_KEYCODE(v: cint): cint; inline;
>>>>>>> graemeg/cpstrnew
=======
function KS_KEYCODE(v: cint): cint; inline;
>>>>>>> graemeg/cpstrnew
=======
function KS_KEYCODE(v: cint): cint; inline;
>>>>>>> origin/cpstrnew
function KS_GROUP(k: cint): cint; inline;
function KS_VALUE(k: cint): cint; inline;
function KB_ENCODING(e: cint): cint; inline;
function KB_VARIANT(e: cint): cint; inline;

const
  KB_NONE = $0000;
  KB_USER = $0100;
  KB_US = $0200;
  KB_DE = $0300;
  KB_DK = $0400;
  KB_IT = $0500;
  KB_FR = $0600;
  KB_UK = $0700;
  KB_JP = $0800;
  KB_SV = $0900;
  KB_NO = $0a00;
  KB_ES = $0b00;
  KB_HU = $0c00;
  KB_BE = $0d00;
  KB_RU = $0e00;
  KB_SG = $0f00;
  KB_SF = $1000;
  KB_PT = $1100;
  KB_UA = $1200;
  KB_LT = $1300;
  KB_LA = $1400;
  KB_BR = $1500;
  KB_NL = $1600;
  KB_TR = $1700;
  KB_PL = $1800;
  KB_SI = $1900;
  KB_CF = $1a00;
  KB_LV = $1b00;
  KB_NODEAD = $000001;  (* disable dead accents  *)
  KB_DECLK = $000002;  (* DEC LKnnn layout  *)
  KB_LK401 = $000004;  (* DEC LK401 instead LK201  *)
  KB_SWAPCTRLCAPS = $000008;  (* swap Left-Control and Caps-Lock  *)
  KB_DVORAK = $000010;  (* Dvorak layout  *)
  KB_METAESC = $000020;  (* generate ESC prefix on ALT-key  *)
  KB_IOPENER = $000040;  (* f1-f12 -> ESC,f1-f11  *)
  KB_MACHDEP = $000080;  (* machine dependent  *)
  KB_APPLE = $010000;  (* Apple specific layout  *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(*
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
  KB_ENCTAB = (( KB_USER,	'user' ),
  	( KB_US,	'us' ),
  	( KB_DE,	'de' ),
  	( KB_DK,	'dk' ),
  	( KB_IT,	'it' ),
  	( KB_FR,	'fr' ),
  	( KB_UK,	'uk' ),
  	( KB_JP,	'jp' ),
  	( KB_SV,	'sv' ),
  	( KB_NO,	'no' ),
  	( KB_ES,	'es' ),
  	( KB_HU,	'hu' ),
  	( KB_BE,	'be' ),
  	( KB_RU,	'ru' ),
  	( KB_UA,	'ua' ),
  	( KB_SG,	'sg' ),
  	( KB_SF,	'sf' ),
  	( KB_PT,	'pt' ),
  	( KB_LT,	'lt' ),
  	( KB_LA,	'la' ),
  	( KB_BR,	'br' ),
  	( KB_NL,	'nl' ),
  	( KB_TR,	'tr' ),
  	( KB_PL,	'pl' ),
  	( KB_SI,	'si' ),
  	( KB_CF,	'cf' ),
  	( KB_LV,	'lv' ));

  KB_VARTAB = (( KB_NODEAD,	'nodead' ),
  	( KB_DECLK,	'declk' ),
  	( KB_LK401,	'lk401' ),
  	( KB_SWAPCTRLCAPS, 'swapctrlcaps' ),
  	( KB_DVORAK,	'dvorak' ),
  	( KB_METAESC,	'metaesc' ),
  	( KB_IOPENER,	'iopener' ),
  	( KB_MACHDEP,	'machdep' ),
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  	( KB_APPLE,	'apple' ));        *)
=======
  	( KB_APPLE,	'apple' ));
>>>>>>> graemeg/cpstrnew
=======
  	( KB_APPLE,	'apple' ));
>>>>>>> graemeg/cpstrnew
=======
  	( KB_APPLE,	'apple' ));
>>>>>>> graemeg/cpstrnew
=======
  	( KB_APPLE,	'apple' ));
>>>>>>> origin/cpstrnew
{$ENDIF}



{$IFDEF OGC_IMPLEMENTATION}
function KS_KEYCODE(v: cint): cint; inline;
begin
  KS_KEYCODE := v or $e000;
end;

function KS_GROUP(k: cint): cint; inline;
begin
  if (k >= $0300) and (k < $0370) then
    KS_GROUP := KS_GROUP_Dead
  else
	if (k and $f000) = $e000 then
    KS_GROUP := KS_GROUP_Keycode
  else
	if (k and $f800) = $f000 then
    KS_GROUP := (k and $ff00)
  else
		KS_GROUP := KS_GROUP_Ascii;
end;

function KS_VALUE(k: cint): cint; inline;
begin
  if (k and $f000) = $e000 then
    KS_VALUE := (k and $0fff)
  else
	if (k and $f800) = $f000 then
    KS_VALUE := (k and $00ff)
  else
    KS_VALUE := k;
end;

function KB_ENCODING(e: cint): cint; inline;
begin
  KB_ENCODING := (e and $0000ff00);
end;

function KB_VARIANT(e: cint): cint; inline;
begin
  KB_VARIANT := (e and $ffff00ff);
end;
{$ENDIF}

