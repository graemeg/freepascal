{
    This file is part of the Free Pascal run time library and compiler.
    Copyright (c) 1998-2002 by the Free Pascal development team

    Internal Function/Constant Evaluator numbers

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit compinnr;

{$i fpcdefs.inc}

{$packenum 4}

interface

const
  { this file needs to be kept in sync with rtl/inc/innr.in }
  in_cpu_first   = 10000;
  in_x86_mm_first    = 11000;

type
   tinlinenumber=(
     in_none              = -1,
{ Internal functions }
     in_lo_word           = 1,
     in_hi_word           = 2,
     in_lo_long           = 3,
     in_hi_long           = 4,
     in_ord_x             = 5,
     in_length_x          = 6,
     in_chr_byte          = 7,
     in_write_x           = 14,
     in_writeln_x         = 15,
     in_read_x            = 16,
     in_readln_x          = 17,
     in_concat_x          = 18,
     in_assigned_x        = 19,
     in_str_x_string      = 20,
     in_ofs_x             = 21,
     in_sizeof_x          = 22,
     in_typeof_x          = 23,
     in_val_x             = 24,
     in_reset_x           = 25,
     in_rewrite_x         = 26,
     in_low_x             = 27,
     in_high_x            = 28,
     in_seg_x             = 29,
     in_pred_x            = 30,
     in_succ_x            = 31,
     in_reset_typedfile   = 32,
     in_rewrite_typedfile = 33,
     in_settextbuf_file_x = 34,
     in_inc_x             = 35,
     in_dec_x             = 36,
     in_include_x_y       = 37,
     in_exclude_x_y       = 38,
     in_break             = 39,
     in_continue          = 40,
     in_assert_x_y        = 41,
     in_addr_x            = 42,
     in_typeinfo_x        = 43,
     in_setlength_x       = 44,
     in_finalize_x        = 45,
     in_new_x             = 46,
     in_dispose_x         = 47,
     in_exit              = 48,
     in_copy_x            = 49,
     in_initialize_x      = 50,
     in_leave             = 51, {macpas}
     in_cycle             = 52, {macpas}
     in_slice_x           = 53,
     in_unaligned_x       = 54,
     in_get_frame         = 56,
     in_get_caller_addr   = 57,
     in_get_caller_frame  = 58,
     in_pack_x_y_z        = 59,
     in_unpack_x_y_z      = 60,
     in_bitsizeof_x       = 61,
     in_writestr_x        = 62,
     in_readstr_x         = 63,
     in_abs_long          = 64,
     in_ror_x             = 65,
     in_ror_x_y           = 66,
     in_rol_x             = 67,
     in_rol_x_y           = 68,
     in_objc_selector_x   = 69,
     in_objc_protocol_x   = 70,
     in_objc_encode_x     = 71,
     in_sar_x_y           = 72,
     in_sar_x             = 73,
     in_bsf_x             = 74,
     in_bsr_x             = 75,
     in_default_x         = 76,
     in_box_x             = 77, { managed platforms: wrap in class instance }
     in_unbox_x_y         = 78, { manage platforms: extract from class instance }
     in_popcnt_x          = 79,
     in_aligned_x         = 80,
     in_setstring_x_y_z   = 81,
     in_insert_x_y_z      = 82,
     in_delete_x_y_z      = 83,
     in_reset_typedfile_name   = 84,
     in_rewrite_typedfile_name = 85,
     in_and_assign_x_y    = 86,
     in_or_assign_x_y     = 87,
     in_xor_assign_x_y    = 88,
     in_sar_assign_x_y    = 89,
     in_shl_assign_x_y    = 90,
     in_shr_assign_x_y    = 91,
     in_rol_assign_x_y    = 92,
     in_ror_assign_x_y    = 93,
     in_neg_assign_x      = 94,
     in_not_assign_x      = 95,
     in_gettypekind_x     = 96,
     in_faraddr_x         = 97,
     in_volatile_x        = 98,
     in_ismanagedtype_x   = 99,

{ Internal constant functions }
     in_const_sqr        = 100,
     in_const_abs        = 101,
     in_const_odd        = 102,
     in_const_ptr        = 103,
     in_const_swap_word  = 104,
     in_const_swap_long  = 105,
     in_lo_qword         = 106,
     in_hi_qword         = 107,
     in_const_swap_qword = 108,
     in_prefetch_var     = 109,
     in_const_eh_return_data_regno = 110,

{ FPU functions }
     in_trunc_real       = 120,
     in_round_real       = 121,
     in_frac_real        = 122,
     in_int_real         = 123,
     in_exp_real         = 124,
     in_cos_real         = 125,
     in_pi_real          = 126,
     in_abs_real         = 127,
     in_sqr_real         = 128,
     in_sqrt_real        = 129,
     in_arctan_real      = 130,
     in_ln_real          = 131,
     in_sin_real         = 132,
     in_fma_single       = 133,
     in_fma_double       = 134,
     in_fma_extended     = 135,
     in_fma_float128     = 136,

{ MMX functions }
{ these contants are used by the mmx unit }

     { MMX }
     in_mmx_pcmpeqb      = 200,
     in_mmx_pcmpeqw      = 201,
     in_mmx_pcmpeqd      = 202,
     in_mmx_pcmpgtb      = 203,
     in_mmx_pcmpgtw      = 204,
     in_mmx_pcmpgtd      = 205

     { 3DNow }

     { SSE }

{$if defined(X86)}
     ,
     {$i x86/cx86innr.inc}
{$endif }
{$if defined(AVR)}
     ,
     {$i ccpuinnr.inc}
{$endif }
   );

implementation
end.
