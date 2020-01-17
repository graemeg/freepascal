{
    This file is part of the Free Pascal run time library and compiler.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Internal Function/Constant Evaluator numbers

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
{ Internal functions }
   fpc_in_lo_word           = 1;
   fpc_in_hi_word           = 2;
   fpc_in_lo_long           = 3;
   fpc_in_hi_long           = 4;
   fpc_in_ord_x             = 5;
   fpc_in_length_string     = 6;
   fpc_in_chr_byte          = 7;
   fpc_in_write_x           = 14;
   fpc_in_writeln_x         = 15;
   fpc_in_read_x            = 16;
   fpc_in_readln_x          = 17;
   fpc_in_concat_x          = 18;
   fpc_in_assigned_x        = 19;
   fpc_in_str_x_string      = 20;
   fpc_in_ofs_x             = 21;
   fpc_in_sizeof_x          = 22;
   fpc_in_typeof_x          = 23;
   fpc_in_val_x             = 24;
   fpc_in_reset_x           = 25;
   fpc_in_rewrite_x         = 26;
   fpc_in_low_x             = 27;
   fpc_in_high_x            = 28;
   fpc_in_seg_x             = 29;
   fpc_in_pred_x            = 30;
   fpc_in_succ_x            = 31;
   fpc_in_reset_typedfile   = 32;
   fpc_in_rewrite_typedfile = 33;
   fpc_in_settextbuf_file_x = 34;
   fpc_in_inc_x             = 35;
   fpc_in_dec_x             = 36;
   fpc_in_include_x_y       = 37;
   fpc_in_exclude_x_y       = 38;
   fpc_in_break             = 39;
   fpc_in_continue          = 40;
   fpc_in_assert_x_y        = 41;
   fpc_in_addr_x            = 42;
   fpc_in_typeinfo_x        = 43;
   fpc_in_setlength_x       = 44;
   fpc_in_finalize_x        = 45;
   fpc_in_new_x             = 46;
   fpc_in_dispose_x         = 47;
   fpc_in_exit              = 48;
   fpc_in_copy_x            = 49;
   fpc_in_initialize_x      = 50;
   fpc_in_leave             = 51; {macpas}
   fpc_in_cycle             = 52; {macpas}
   fpc_in_slice             = 53;
   fpc_in_unaligned_x       = 54;
   fpc_in_get_frame         = 56;
   fpc_in_get_caller_addr   = 57;
   fpc_in_get_caller_frame  = 58;
   fpc_in_pack_x_y_z        = 59;
   fpc_in_unpack_x_y_z      = 60;
   fpc_in_bitsizeof_x       = 61;
   fpc_in_writestr_x        = 62;
   fpc_in_readstr_x         = 63;
   fpc_in_abs_long          = 64;
   fpc_in_ror_x             = 65;
   fpc_in_ror_x_x           = 66;
   fpc_in_rol_x             = 67;
   fpc_in_rol_x_x           = 68;
   fpc_objc_selector_x      = 69;
   fpc_objc_protocol_x      = 70;
   fpc_objc_encode_x        = 71;
   fpc_in_sar_x_y           = 72;
   fpc_in_sar_x             = 73;
   fpc_in_bsf_x             = 74;
   fpc_in_bsr_x             = 75;
   fpc_in_default_x         = 76;
   fpc_in_box_x             = 77; { managed platforms: wrap in class instance }
   fpc_in_unbox_x_y         = 78; { manage platforms: extract from class instance }
   fpc_in_popcnt_x          = 79;
   fpc_in_aligned_x         = 80;
   fpc_in_setstring_x_y_z   = 81;
   fpc_in_insert_x_y_z      = 82;
   fpc_in_delete_x_y_z      = 83;
   fpc_in_reset_typedfile_name   = 84;
   fpc_in_rewrite_typedfile_name = 85;
   fpc_in_and_assign_x_y    = 86;
   fpc_in_or_assign_x_y     = 87;
   fpc_in_xor_assign_x_y    = 88;
   fpc_in_sar_assign_x_y    = 89;
   fpc_in_shl_assign_x_y    = 90;
   fpc_in_shr_assign_x_y    = 91;
   fpc_in_rol_assign_x_y    = 92;
   fpc_in_ror_assign_x_y    = 93;
   fpc_in_neg_assign_x      = 94;
   fpc_in_not_assign_x      = 95;
   fpc_in_faraddr_x         = 97;
   fpc_in_volatile_x        = 98;

{ Internal constant functions }
   fpc_in_const_sqr        = 100;
   fpc_in_const_abs        = 101;
   fpc_in_const_odd        = 102;
   fpc_in_const_ptr        = 103;
   fpc_in_const_swap_word  = 104;
   fpc_in_const_swap_long  = 105;
   fpc_in_lo_qword         = 106;
   fpc_in_hi_qword         = 107;
   fpc_in_const_swap_qword = 108;
   fpc_in_prefetch_var     = 109;
   fpc_in_const_eh_return_data_regno = 110;

{ FPU functions }
   fpc_in_trunc_real       = 120;
   fpc_in_round_real       = 121;
   fpc_in_frac_real        = 122;
   fpc_in_int_real         = 123;
   fpc_in_exp_real         = 124;
   fpc_in_cos_real         = 125;
   fpc_in_pi_real          = 126;
   fpc_in_abs_real         = 127;
   fpc_in_sqr_real         = 128;
   fpc_in_sqrt_real        = 129;
   fpc_in_arctan_real      = 130;
   fpc_in_ln_real          = 131;
   fpc_in_sin_real         = 132;
   fpc_in_fma_single       = 133;
   fpc_in_fma_double       = 134;
   fpc_in_fma_extended     = 135;
   fpc_in_fma_float128     = 136;

{ MMX functions }
{ these contants are used by the mmx unit }

   { MMX }
   fpc_in_mmx_pcmpeqb      = 200;
   fpc_in_mmx_pcmpeqw      = 201;
   fpc_in_mmx_pcmpeqd      = 202;
   fpc_in_mmx_pcmpgtb      = 203;
   fpc_in_mmx_pcmpgtw      = 204;
   fpc_in_mmx_pcmpgtd      = 205;

   { 3DNow }

   { SSE }

   fpc_in_cpu_first        = 10000;
   fpc_in_x86_mm_first     = 11000;
   
