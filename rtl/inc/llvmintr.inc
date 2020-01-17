{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2015 Jonas Maebe

    This file contains declarations for LLVM intrinsics

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


procedure llvm_memcpy64(dest, source: pointer; len: qword; align: cardinal; isvolatile: LLVMBool1); compilerproc; external name 'llvm.memcpy.p0i8.p0i8.i64';
{ LLVM 7.0+, alignment specified as part of the dest and source parameters }
procedure llvm_memcpy64_indivalign(dest, source: pointer; len: qword; isvolatile: LLVMBool1); compilerproc; external name 'llvm.memcpy.p0i8.p0i8.i64';

function llvm_frameaddress(level: longint): pointer; compilerproc; external name 'llvm.frameaddress';

function llvm_eh_typeid_for(sym: pointer): longint; compilerproc; external name 'llvm.eh.typeid.for';

procedure llvm_lifetime_start(size: int64; ptr: pointer); compilerproc; external name 'llvm.lifetime.start';
procedure llvm_lifetime_end(size: int64; ptr: pointer); compilerproc; external name 'llvm.lifetime.end';

function llvm_ctlz(src: UInt8; is_zero_undef: LLVMBool1): UInt8; external name 'llvm.ctlz.i8';
function llvm_ctlz(src: UInt16; is_zero_undef: LLVMBool1): UInt16; external name 'llvm.ctlz.i16';
function llvm_ctlz(src: UInt32; is_zero_undef: LLVMBool1): UInt32; external name 'llvm.ctlz.i32';
function llvm_ctlz(src: UInt64; is_zero_undef: LLVMBool1): UInt64; external name 'llvm.ctlz.i64';

function llvm_cttz(src: UInt8; is_zero_undef: LLVMBool1): UInt8; external name 'llvm.cttz.i8';
function llvm_cttz(src: UInt16; is_zero_undef: LLVMBool1): UInt16; external name 'llvm.cttz.i16';
function llvm_cttz(src: UInt32; is_zero_undef: LLVMBool1): UInt32; external name 'llvm.cttz.i32';
function llvm_cttz(src: UInt64; is_zero_undef: LLVMBool1): UInt64; external name 'llvm.cttz.i64';

function llvm_ctpop(src: UInt8): UInt8; external name 'llvm.ctpop.i8';
function llvm_ctpop(src: UInt16): UInt16; external name 'llvm.ctpop.i16';
function llvm_ctpop(src: UInt32): UInt32; external name 'llvm.ctpop.i32';
function llvm_ctpop(src: UInt64): UInt64; external name 'llvm.ctpop.i64';

function llvm_sqrt_f32(val: single): single; compilerproc; external name 'llvm.sqrt.f32';
function llvm_experimental_constrained_sqrt(val: single; rounding, exceptions: LLVMMetadata): single; external name 'llvm.experimental.constrained.sqrt.f32';
function llvm_sqrt_f64(val: double): double; compilerproc; external name 'llvm.sqrt.f64';
function llvm_experimental_constrained_sqrt(val: double; rounding, exceptions: LLVMMetadata): double; external name 'llvm.experimental.constrained.sqrt.f64';
{$ifdef SUPPORT_EXTENDED}
function llvm_sqrt_f80(val: cextended): cextended; compilerproc; external name 'llvm.sqrt.x86_fp80';
function llvm_experimental_constrained_sqrt(val: cextended; rounding, exceptions: LLVMMetadata): cextended; external name 'llvm.experimental.constrained.sqrt.x86_fp80';
{$endif}
{$ifdef SUPPORT_FLOAT128}
function llvm_sqrt_f128(val: float128): float128; compilerproc; external name 'llvm.sqrt.f128';
function llvm_experimental_constrained_sqrt(val: float128; rounding, exceptions: LLVMMetadata): float128; external name 'llvm.experimental.constrained.sqrt.f128';
{$endif}

function llvm_fma_f32(a, b, c: single): single; compilerproc; external name 'llvm.fma.f32';
function llvm_experimental_constrained_fma(a, b, c: single; rounding, exceptions: LLVMMetadata): single; external name 'llvm.experimental.constrained.fma.f32';
function llvm_fma_f64(a, b, c: double): double; compilerproc; external name 'llvm.fma.f64';
function llvm_experimental_constrained_fma(a, b, c: double; rounding, exceptions: LLVMMetadata): double; external name 'llvm.experimental.constrained.fma.f64';
{$ifdef SUPPORT_EXTENDED}
function llvm_fma_f80(a, b, c: cextended): cextended; compilerproc; external name 'llvm.fma.x86_fp80';
function llvm_experimental_constrained_fma(a, b, c: cextended; rounding, exceptions: LLVMMetadata): cextended; external name 'llvm.experimental.constrained.fma.x86_fp80';
{$endif}
{$ifdef SUPPORT_FLOAT128}
function llvm_fma_f128(a, b, c: float128): float128; compilerproc; external name 'llvm.fma.f128';
function llvm_experimental_constrained_fma(a, b, c: float128; rounding, exceptions: LLVMMetadata): float128; external name 'llvm.experimental.constrained.fma.f128';
{$endif}

function llvm_experimental_constrained_fadd(a, b: single; rounding, exceptions: LLVMMetadata): single; external name 'llvm.experimental.constrained.fadd.f32';
function llvm_experimental_constrained_fsub(a, b: single; rounding, exceptions: LLVMMetadata): single; external name 'llvm.experimental.constrained.fsub.f32';
function llvm_experimental_constrained_fmul(a, b: single; rounding, exceptions: LLVMMetadata): single; external name 'llvm.experimental.constrained.fmul.f32';
function llvm_experimental_constrained_fdiv(a, b: single; rounding, exceptions: LLVMMetadata): single; external name 'llvm.experimental.constrained.fdiv.f32';
function llvm_experimental_constrained_fadd(a, b: double; rounding, exceptions: LLVMMetadata): double; external name 'llvm.experimental.constrained.fadd.f64';
function llvm_experimental_constrained_fsub(a, b: double; rounding, exceptions: LLVMMetadata): double; external name 'llvm.experimental.constrained.fsub.f64';
function llvm_experimental_constrained_fmul(a, b: double; rounding, exceptions: LLVMMetadata): double; external name 'llvm.experimental.constrained.fmul.f64';
function llvm_experimental_constrained_fdiv(a, b: double; rounding, exceptions: LLVMMetadata): double; external name 'llvm.experimental.constrained.fdiv.f64';
{$ifdef SUPPORT_EXTENDED}
function llvm_experimental_constrained_fadd(a, b: extended; rounding, exceptions: LLVMMetadata): extended; external name 'llvm.experimental.constrained.fadd.x86_fp80';
function llvm_experimental_constrained_fsub(a, b: extended; rounding, exceptions: LLVMMetadata): extended; external name 'llvm.experimental.constrained.fsub.x86_fp80';
function llvm_experimental_constrained_fmul(a, b: extended; rounding, exceptions: LLVMMetadata): extended; external name 'llvm.experimental.constrained.fmul.x86_fp80';
function llvm_experimental_constrained_fdiv(a, b: extended; rounding, exceptions: LLVMMetadata): extended; external name 'llvm.experimental.constrained.fdiv.x86_fp80';
{$endif}
{$ifdef SUPPORT_FLOAT128}
function llvm_experimental_constrained_fadd(a, b: float128; rounding, exceptions: LLVMMetadata): float128; external name 'llvm.experimental.constrained.fadd.f128';
function llvm_experimental_constrained_fsub(a, b: float128; rounding, exceptions: LLVMMetadata): float128; external name 'llvm.experimental.constrained.fsub.f128';
function llvm_experimental_constrained_fmul(a, b: float128; rounding, exceptions: LLVMMetadata): float128; external name 'llvm.experimental.constrained.fmul.f128';
function llvm_experimental_constrained_fdiv(a, b: float128; rounding, exceptions: LLVMMetadata): float128; external name 'llvm.experimental.constrained.fdiv.f128';
{$endif}

function llvm_experimental_constrained_fptrunc_f32_f64(a: double; rounding, exceptions: LLVMMetadata): single; compilerproc; external name 'llvm.experimental.constrained.fptrunc.f32.f64';
function llvm_experimental_constrained_fpext_f64_f32(a: single; exceptions: LLVMMetadata): double; compilerproc; external name 'llvm.experimental.constrained.fpext.f64.f32';
{$ifdef SUPPORT_EXTENDED}
function llvm_experimental_constrained_fptrunc_f32_f80(a: extended; rounding, exceptions: LLVMMetadata): single; compilerproc; external name 'llvm.experimental.constrained.fptrunc.f32.x86_fp80';
function llvm_experimental_constrained_fptrunc_f64_f80(a: extended; rounding, exceptions: LLVMMetadata): double; compilerproc; external name 'llvm.experimental.constrained.fptrunc.f64.x86_fp80';
function llvm_experimental_constrained_fpext_f80_f32(a: single; exceptions: LLVMMetadata): extended; compilerproc; external name 'llvm.experimental.constrained.fpext.x86_fp80.f32';
function llvm_experimental_constrained_fpext_f80_f64(a: double; exceptions: LLVMMetadata): extended; compilerproc; external name 'llvm.experimental.constrained.fpext.x86_fp80.f64';
{$ifdef SUPPORT_FLOAT128}
function llvm_experimental_constrained_fptrunc_f128_f80(a: extended; rounding, exceptions: LLVMMetadata): float128; compilerproc; external name 'llvm.experimental.constrained.fptrunc.f128.x86_fp80';
function llvm_experimental_constrained_fpext_f80_f32(a: float128; exceptions: LLVMMetadata): extended; compilerproc; external name 'llvm.experimental.constrained.fpext.x86_fp80.f128';
{$endif}
{$endif}
{$ifdef SUPPORT_FLOAT128}
function llvm_experimental_constrained_fptrunc_f32_f128(a: float128; rounding, exceptions: LLVMMetadata): single; compilerproc; external name 'llvm.experimental.constrained.fptrunc.f32.f128';
function llvm_experimental_constrained_fptrunc_f64_f128(a: float128; rounding, exceptions: LLVMMetadata): double; compilerproc; external name 'llvm.experimental.constrained.fptrunc.f64.f128';
function llvm_experimental_constrained_fpext_f128_f32(a: single; exceptions: LLVMMetadata): float128; compilerproc; external name 'llvm.experimental.constrained.fpext.f128.f32';
function llvm_experimental_constrained_fpext_f128_f64(a: double; exceptions: LLVMMetadata): float128; compilerproc; external name 'llvm.experimental.constrained.fpext.f128.f64';
{$endif}

{ only include the cases that may trigger rounding }
function llvm_experimental_constrained_sitofp_f32_i32(val: longint; rounding, exceptions: LLVMMetadata): single; compilerproc; external name 'llvm.experimental.constrained.sitofp.f32.i32';
function llvm_experimental_constrained_sitofp_f32_i64(val: int64; rounding, exceptions: LLVMMetadata): single; compilerproc; external name 'llvm.experimental.constrained.sitofp.f32.i64';
function llvm_experimental_constrained_sitofp_f64_i64(val: int64; rounding, exceptions: LLVMMetadata): double; compilerproc; external name 'llvm.experimental.constrained.sitofp.f64.i64';
function llvm_experimental_constrained_uitofp_f32_i32(val: cardinal; rounding, exceptions: LLVMMetadata): single; compilerproc; external name 'llvm.experimental.constrained.uitofp.f32.i32';
function llvm_experimental_constrained_uitofp_f32_i64(val: qword; rounding, exceptions: LLVMMetadata): single; compilerproc; external name 'llvm.experimental.constrained.uitofp.f32.i64';
function llvm_experimental_constrained_uitofp_f64_i64(val: qword; rounding, exceptions: LLVMMetadata): double; compilerproc; external name 'llvm.experimental.constrained.uitofp.f64.i64';
{$ifdef SUPPORT_EXTENDED}
function llvm_experimental_constrained_uitofp_f80_i64(val: qword; rounding, exceptions: LLVMMetadata): extended; compilerproc; external name 'llvm.experimental.constrained.uitofp.x86_fp80.i64';
{$endif}
