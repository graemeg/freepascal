{
    Copyright (C) 2013 by Max Nazhalov

    This file contains generalized floating point<->ASCII conversion routines.
    It is included by the FLT_CONV.INC after setting-up correct conditional
    definitions, therefore it sholud not be used directly.
    Refer to FLT_CONV.INC for further explanation.

    This library is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License as published by
    the Free Software Foundation; either version 2 of the License, or (at your
    option) any later version with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,
    and to copy and distribute the resulting executable under terms of your
    choice, provided that you also meet, for each linked independent module,
    the terms and conditions of the license of that module. An independent
    module is a module which is not derived from or based on this library.
    If you modify this library, you may extend this exception to your version
    of the library, but you are not obligated to do so. If you do not wish to
    do so, delete this exception statement from your version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU Library General Public License for more details.

    You should have received a copy of the GNU Library General Public License
    along with this library; if not, write to the Free Software Foundation,
    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

 ****************************************************************************
}

type
    // "Do-It-Yourself Floating Point" structure
    TDIY_FP = record
      {$ifdef VALREAL_128}
        fh : qword;
      {$endif}
      {$ifdef VALREAL_32}
        f : dword;
      {$else}
        f : qword;
      {$endif}
      {$ifdef VALREAL_80}
        fh : dword;
      {$endif}
        e : integer;
    end;

    TDIY_FP_Power_of_10 = record
        c : TDIY_FP;
        e10 : integer;
    end;

{$if defined(VALREAL_80) or defined(VALREAL_128)}

(****************************************************************************
 * diy_util_add
 *
 * Helper routine: summ of multiword unsigned integers
 *
 * Used by:
 *   [80,128] diy_fp_cached_power10
 *   [80,128] diy_fp_multiply
 *   [80,128] float<->ASCII
 *
 ****************************************************************************)
{$ifdef VALREAL_80}
procedure diy_util_add( var xh: dword; var xl: qword; const yh: dword; const yl: qword ); {$ifdef grisu1_inline}inline;{$endif}
{$else VALREAL_128}
procedure diy_util_add( var xh, xl: qword; const yh, yl: qword ); {$ifdef grisu1_inline}inline;{$endif}
{$endif VALREAL_*}
var
    temp: qword;
begin
    temp := xl + yl;
    xh := xh + yh + ord( temp < xl );
    xl := temp;
end;

(****************************************************************************
 * diy_util_shl
 *
 * Helper routine: left shift of multiword unsigned integer
 *
 * Used by:
 *   [80,128] float<->ASCII
 *
 ****************************************************************************)
{$ifdef VALREAL_80}
procedure diy_util_shl( var h: dword; var l: qword; const count: integer );
{$else VALREAL_128}
procedure diy_util_shl( var h, l: qword; const count: integer );
{$endif VALREAL_*}
begin
    if ( count = 0 ) then
        exit;
{$ifdef grisu1_debug}
    assert( count > 0 );
  {$ifdef VALREAL_80}
    assert( count < 96 );
  {$else VALREAL_128}
    assert( count < 128 );
  {$endif VALREAL_*}
{$endif grisu1_debug}
    if ( count = 1 ) then
    begin
        diy_util_add( h, l, h, l );
        exit;
    end;
    if ( count >= 64 ) then
    begin
        if ( count > 64 ) then
            h := ( l shl ( count - 64 ) )
        else
            h := l;
        l := 0;
        exit;
    end;
  {$ifdef VALREAL_80}
    if ( count = 32 ) then
        h := hi( l )
    else
  {$endif VALREAL_80}
    if ( count < 32 ) then
        h := ( h shl count ) + ( hi( l ) shr ( 32 - count ) )
    else
  {$ifdef VALREAL_80}
        h := ( l shr ( 64 - count ) );
  {$else VALREAL_128}
        h := ( h shl count ) + ( l shr ( 64 - count ) );
  {$endif VALREAL_*}
    l := ( l shl count );
end;

(****************************************************************************
 * diy_util_shr
 *
 * Helper routine: right shift of multiword unsigned integer
 *
 * Used by:
 *   [80,128] float<->ASCII
 *
 ****************************************************************************)
{$ifdef VALREAL_80}
procedure diy_util_shr( var h: dword; var l: qword; const count: integer );
{$else VALREAL_128}
procedure diy_util_shr( var h, l: qword; const count: integer );
{$endif VALREAL_*}
begin
    if ( count = 0 ) then
        exit;
  {$ifdef grisu1_debug}
    assert( count > 0 );
  {$endif grisu1_debug}
    if ( count = 1 ) then
    begin
        l := l shr 1;
        if ( lo(h) and 1 <> 0 ) then
            l := l or qword($8000000000000000);
        h := h shr 1;
        exit;
    end;
    if ( count < 64 ) then
    begin
        l := ( qword( h ) shl ( ( - count ) and 63 ) ) or ( l shr count );
      {$ifdef VALREAL_80}
        if ( count >= 32 ) then
            h := 0
        else
      {$endif VALREAL_80}
            h := h shr count;
        exit;
    end;
  {$ifdef VALREAL_80}
    if ( count < 96 ) then
  {$else VALREAL_128}
    if ( count < 128 ) then
  {$endif VALREAL_*}
        l := h shr ( count and 63 )
    else
        l := 0;
    h := 0;
end;

{$endif VALREAL_80 | VALREAL_128}

(****************************************************************************
 * diy_fp_multiply
 *
 * "Do-It-Yourself Floating Point" multiplication routine
 *
 * Simplified implementation:
 *  > restricted input:
 *     - both operands should be normalized
 *  > relaxed output:
 *     - rounding is simple [half is rounded-up]
 *     - normalization is optional and performed at the very end if requested
 *       [at most 1 shift required since both multipliers are normalized]
 *
 * Used by:
 *   [all] float<->ASCII
 *   [>32] diy_fp_cached_power10
 *
 ****************************************************************************)
function diy_fp_multiply( const x, y: TDIY_FP; normalize: boolean ): TDIY_FP;
const
    C_1_SHL_31 = dword($80000000);
{$ifdef VALREAL_32}
//***************** 32-bit *********************
var
    a, b, c, d, ac, bc, ad, bd, t1: dword;
begin
    a := ( x.f shr 16 );
    b := ( x.f and $0000FFFF );
    c := ( y.f shr 16 );
    d := ( y.f and $0000FFFF );
    ac := a * c;
    bc := b * c;
    ad := a * d;
    bd := b * d;
    t1 := ( bc and $0000FFFF )
        + ( bd shr 16 )
        + ( ad and $0000FFFF )
        + ( 1 shl 15 ); // round
    diy_fp_multiply.f := ac
        + ( ad shr 16 )
        + ( bc shr 16 )
        + ( t1 shr 16 );
    diy_fp_multiply.e := x.e + y.e + 32;
    if normalize then with diy_fp_multiply do
    begin
        if ( f and C_1_SHL_31 = 0 ) then
        begin
            inc( f, f );
            dec( e );
        end;
      {$ifdef grisu1_debug}
        assert( f and C_1_SHL_31 <> 0 );
      {$endif grisu1_debug}
    end;
end;
{$else not VALREAL_32}
(*-------------------------------------------------------
 | u32_mul_u32_to_u64 [local]
 |
 | Local routine of the "diy_fp_multiply"; common to float64..float128:
 |     uint32 * uint32 -> uint64
 |
 *-------------------------------------------------------*)
function u32_mul_u32_to_u64( const a, b: dword ): qword; {$ifdef grisu1_inline}inline;{$endif}
begin
    // it seems this pattern is very tightly optimized by FPC at least for i386
    u32_mul_u32_to_u64 := qword( a ) * b;
end;
{$endif VALREAL_*}
{$ifdef VALREAL_64}
//***************** 64-bit *********************
var
    a, b, c, d: dword;
    ac, bc, ad, bd, t1: qword;
begin
    a := hi( x.f ); b := x.f;
    c := hi( y.f ); d := y.f;
    ac := u32_mul_u32_to_u64( a, c );
    bc := u32_mul_u32_to_u64( b, c );
    ad := u32_mul_u32_to_u64( a, d );
    bd := u32_mul_u32_to_u64( b, d );
    t1 := qword( C_1_SHL_31 ) // round
        + hi( bd ) + lo( bc ) + lo( ad );
    diy_fp_multiply.f := ac + hi( ad ) + hi( bc ) + hi( t1 );
    diy_fp_multiply.e := x.e + y.e + 64;
    if normalize then with diy_fp_multiply do
    begin
        if ( hi( f ) and C_1_SHL_31 = 0 ) then
        begin
            inc( f, f );
            dec( e );
        end;
      {$ifdef grisu1_debug}
        assert( hi( f ) and C_1_SHL_31 <> 0 );
      {$endif grisu1_debug}
    end;
end;
{$endif VALREAL_64}
{$ifdef VALREAL_80}
//***************** 96-bit *********************
var
    a, b, c, u, v, w: dword;
    au, av, aw, bu, bv, bw, cu, cv, cw, t1, t2: qword;
begin
    a := x.fh; b := hi( x.f ); c := x.f;
    u := y.fh; v := hi( y.f ); w := y.f;
    au := u32_mul_u32_to_u64( a, u );
    bu := u32_mul_u32_to_u64( b, u );
    cu := u32_mul_u32_to_u64( c, u );
    av := u32_mul_u32_to_u64( a, v );
    bv := u32_mul_u32_to_u64( b, v );
    cv := u32_mul_u32_to_u64( c, v );
    aw := u32_mul_u32_to_u64( a, w );
    bw := u32_mul_u32_to_u64( b, w );
    cw := u32_mul_u32_to_u64( c, w );
    t1 := ( cw shr 32 ) + lo( bw ) + lo( cv );
    t1 := qword( C_1_SHL_31 ) // round
        + hi( t1 ) + hi( bw ) + hi( cv ) + lo( aw ) + lo( bv ) + lo( cu );
    t1 := ( t1 shr 32 ) + hi( aw ) + hi( bv ) + hi( cu ) + lo( av ) + lo( bu );
    t2 := au + hi( av ) + hi( bu ) + hi( t1 );
    diy_fp_multiply.f := ( t2 shl 32 ) + lo( t1 );
    diy_fp_multiply.fh := hi( t2 );
    diy_fp_multiply.e := x.e + y.e + 96;
    if normalize then with diy_fp_multiply do
    begin
        if ( fh and C_1_SHL_31 = 0 ) then
        begin
            diy_util_add( fh, f, fh, f );
            dec( e );
        end;
      {$ifdef grisu1_debug}
        assert( fh and C_1_SHL_31 <> 0 );
      {$endif grisu1_debug}
    end;
end;
{$endif VALREAL_80}
{$ifdef VALREAL_128}
//***************** 128-bit ********************
var
    a, b, c, d, u, v, w, z: dword;
    au, av, aw, az, bu, bv, bw, bz, cu, cv, cw, cz, du, dv, dw, dz, t1, t2: qword;
begin
    a := hi( x.fh ); b := x.fh; c := hi( x.f ); d := x.f;
    u := hi( y.fh ); v := y.fh; w := hi( y.f ); z := y.f;
    au := u32_mul_u32_to_u64( a, u );
    bu := u32_mul_u32_to_u64( b, u );
    cu := u32_mul_u32_to_u64( c, u );
    du := u32_mul_u32_to_u64( d, u );
    av := u32_mul_u32_to_u64( a, v );
    bv := u32_mul_u32_to_u64( b, v );
    cv := u32_mul_u32_to_u64( c, v );
    dv := u32_mul_u32_to_u64( d, v );
    aw := u32_mul_u32_to_u64( a, w );
    bw := u32_mul_u32_to_u64( b, w );
    cw := u32_mul_u32_to_u64( c, w );
    dw := u32_mul_u32_to_u64( d, w );
    az := u32_mul_u32_to_u64( a, z );
    bz := u32_mul_u32_to_u64( b, z );
    cz := u32_mul_u32_to_u64( c, z );
    dz := u32_mul_u32_to_u64( d, z );
    t1 := ( dz shr 32 ) + lo( cz ) + lo( dw );
    t1 := ( t1 shr 32 ) + hi( cz ) + hi( dw ) + lo( bz ) + lo( cw ) + lo( dv );
    t1 := qword( C_1_SHL_31 ) // round
        + hi( t1 ) + hi( bz ) + hi( cw ) + hi( dv ) + lo( az ) + lo( bw ) + lo( cv ) + lo( du );
    t2 := ( t1 shr 32 ) + hi( az ) + hi( bw ) + hi( cv ) + hi( du ) + lo( aw ) + lo( bv ) + lo( cu );
    t1 := ( t2 shr 32 ) + hi( aw ) + hi( bv ) + hi( cu ) + lo( av ) + lo( bu );
    diy_fp_multiply.f := ( t1 shl 32 ) + lo( t2 );
    diy_fp_multiply.fh := au + hi( av ) + hi( bu ) + hi( t1 );
    diy_fp_multiply.e := x.e + y.e + 128;
    if normalize then with diy_fp_multiply do
    begin
        if ( hi( fh ) and C_1_SHL_31 = 0 ) then
        begin
            diy_util_add( fh, f, fh, f );
            dec( e );
        end;
      {$ifdef grisu1_debug}
        assert( hi( fh ) and C_1_SHL_31 <> 0 );
      {$endif grisu1_debug}
    end;
end;
{$endif VALREAL_128}

(****************************************************************************
 * diy_fp_cached_power10
 *
 * The main purpose of this routine is to return normalized correctly rounded
 * DIY-floating-point approximation of the power of 10, which has to be used
 * by the Grisu1 as a scaling factor, intended to shift a binary exponent of
 * the original number into selected [ alpha .. gamma ] range.
 *
 * This routine is also usable as a helper during ASCII -> float conversion,
 * so the range of cached powers is slightly extended beyond the requirements
 * of the Grisu1.
 *
 * Used by:
 *   [all] float<->ASCII
 *
 ****************************************************************************)
procedure diy_fp_cached_power10( exp10: integer; out factor: TDIY_FP_Power_of_10 );
{$ifdef VALREAL_32}
const
    // alpha =-29; gamma = 0; step = 1E+8
    cache: array [ 0 .. 13 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: dword($FB158593); e: -218 ); e10: -56 ),
        ( c: ( f: dword($BB127C54); e: -191 ); e10: -48 ),
        ( c: ( f: dword($8B61313C); e: -164 ); e10: -40 ),
        ( c: ( f: dword($CFB11EAD); e: -138 ); e10: -32 ),
        ( c: ( f: dword($9ABE14CD); e: -111 ); e10: -24 ),
        ( c: ( f: dword($E69594BF); e:  -85 ); e10: -16 ),
        ( c: ( f: dword($ABCC7712); e:  -58 ); e10:  -8 ),
        ( c: ( f: dword($80000000); e:  -31 ); e10:   0 ),
        ( c: ( f: dword($BEBC2000); e:   -5 ); e10:   8 ),
        ( c: ( f: dword($8E1BC9BF); e:   22 ); e10:  16 ),
        ( c: ( f: dword($D3C21BCF); e:   48 ); e10:  24 ),
        ( c: ( f: dword($9DC5ADA8); e:   75 ); e10:  32 ),
        ( c: ( f: dword($EB194F8E); e:  101 ); e10:  40 ),
        ( c: ( f: dword($AF298D05); e:  128 ); e10:  48 )
    );
var
    i, min10: integer;
begin
    // find index
    min10 := cache[ low( cache ) ].e10;
    if ( exp10 <= min10 ) then
        i := 0
    else
    begin
        i := ( exp10 - min10 ) div 8;
        if ( i >= high(cache) ) then
            i := high(cache)
        else
            if ( cache[ i ].e10 <> exp10 ) then
                inc( i ); // round-up
    end;
    // generate result
    factor := cache[ i ];
end;
{$endif VALREAL_32}
//**************************************
{$ifdef VALREAL_64}
const
    // alpha =-61; gamma = 0
    // full cache: 1E-450 .. 1E+432, step = 1E+18
    // sparse = 1/10
    C_PWR10_DELTA = 18;
    C_PWR10_COUNT = 50;
    base: array [ 0 .. 9 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: qword($825ECC24C8737830); e: -362 ); e10:  -90 ),
        ( c: ( f: qword($E2280B6C20DD5232); e: -303 ); e10:  -72 ),
        ( c: ( f: qword($C428D05AA4751E4D); e: -243 ); e10:  -54 ),
        ( c: ( f: qword($AA242499697392D3); e: -183 ); e10:  -36 ),
        ( c: ( f: qword($9392EE8E921D5D07); e: -123 ); e10:  -18 ),
        ( c: ( f: qword($8000000000000000); e:  -63 ); e10:    0 ),
        ( c: ( f: qword($DE0B6B3A76400000); e:   -4 ); e10:   18 ),
        ( c: ( f: qword($C097CE7BC90715B3); e:   56 ); e10:   36 ),
        ( c: ( f: qword($A70C3C40A64E6C52); e:  116 ); e10:   54 ),
        ( c: ( f: qword($90E40FBEEA1D3A4B); e:  176 ); e10:   72 )
    );
    factor_plus: array [ 0 .. 1 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: qword($F6C69A72A3989F5C); e:   534 ); e10:  180 ),
        ( c: ( f: qword($EDE24AE798EC8284); e:  1132 ); e10:  360 ) 
    );
    factor_minus: array [ 0 .. 1 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: qword($84C8D4DFD2C63F3B); e:  -661 ); e10: -180 ),
        ( c: ( f: qword($89BF722840327F82); e: -1259 ); e10: -360 ) 
    );
    corrector: array [ 0 .. C_PWR10_COUNT - 1 ] of shortint = (
    // extra mantissa correction [ulp; signed]
        0,  0,  0,  0,  1,  0,  0,  0,  1, -1,
        0,  1,  1,  1, -1,  0,  0,  1,  0, -1,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
       -1,  0,  0, -1,  0,  0,  0,  0,  0, -1,
        0,  0,  0,  0,  1,  0,  0,  0, -1,  0
    );
{$endif VALREAL_64}
//**************************************
{$ifdef VALREAL_80}
const
    // alpha =-93; gamma =+30
    // full cache: 1E-5032 .. 1E+4995, step = 1E+37
    // sparse = 1/16
    C_PWR10_DELTA = 37;
    C_PWR10_COUNT = 272;
    base: array [ 0 .. 15 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: qword($07286FAA1AF5AF66); fh: dword($D1476E2C); e:  -1079 ); e10:  -296 ),
        ( c: ( f: qword($99107C22CB550FB4); fh: dword($C4CE17B3); e:   -956 ); e10:  -259 ),
        ( c: ( f: qword($99F6858428E2557B); fh: dword($B9131798); e:   -833 ); e10:  -222 ),
        ( c: ( f: qword($4738705E9624AB51); fh: dword($AE0B158B); e:   -710 ); e10:  -185 ),
        ( c: ( f: qword($0D5FDAF5C13E60D1); fh: dword($A3AB6658); e:   -587 ); e10:  -148 ),
        ( c: ( f: qword($163FA42E504BCED2); fh: dword($99EA0196); e:   -464 ); e10:  -111 ),
        ( c: ( f: qword($483BB9B9B1C6F22B); fh: dword($90BD77F3); e:   -341 ); e10:   -74 ),
        ( c: ( f: qword($545C75757E50D641); fh: dword($881CEA14); e:   -218 ); e10:   -37 ),
        ( c: ( f: qword($0000000000000000); fh: dword($80000000); e:    -95 ); e10:     0 ),
        ( c: ( f: qword($BB48DB201E86D400); fh: dword($F0BDC21A); e:     27 ); e10:    37 ),
        ( c: ( f: qword($4DCDAB14C696963C); fh: dword($E264589A); e:    150 ); e10:    74 ),
        ( c: ( f: qword($C1D1EA966C9E18AC); fh: dword($D4E5E2CD); e:    273 ); e10:   111 ),
        ( c: ( f: qword($C8965D3D6F928295); fh: dword($C83553C5); e:    396 ); e10:   148 ),
        ( c: ( f: qword($96706114873D5D9F); fh: dword($BC4665B5); e:    519 ); e10:   185 ),
        ( c: ( f: qword($56105DAD7425A83F); fh: dword($B10D8E14); e:    642 ); e10:   222 ),
        ( c: ( f: qword($B84603568A892ABB); fh: dword($A67FF273); e:    765 ); e10:   259 )
    );
    factor_plus: array [ 0 .. 7 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: qword($3576D3D149738BA0); fh: dword($BF87DECC); e:   1871 ); e10:   592 ),
        ( c: ( f: qword($750E83050A40DE03); fh: dword($8F4C0691); e:   3838 ); e10:  1184 ),
        ( c: ( f: qword($727E5D9756BC4BF8); fh: dword($D66B8D68); e:   5804 ); e10:  1776 ),
        ( c: ( f: qword($CE9DB63FD51AF6A3); fh: dword($A06C0BD4); e:   7771 ); e10:  2368 ),
        ( c: ( f: qword($5A7ADBC5B8787D89); fh: dword($F00B82D7); e:   9737 ); e10:  2960 ),
        ( c: ( f: qword($22D732D7AE7EDAA7); fh: dword($B397FD9A); e:  11704 ); e10:  3552 ),
        ( c: ( f: qword($CCD2839E0367500B); fh: dword($865DB7A9); e:  13671 ); e10:  4144 ),
        ( c: ( f: qword($FCBEE713F3BE171A); fh: dword($C90E78C7); e:  15637 ); e10:  4736 )
    );
    factor_minus: array [ 0 .. 7 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: qword($2F85DC7AE66FEACF); fh: dword($AB15B5D2); e:  -2062 ); e10:  -592 ),
        ( c: ( f: qword($4237088F4C7284FA); fh: dword($E4AC057C); e:  -4029 ); e10: -1184 ),
        ( c: ( f: qword($D2DCB34CEC42875C); fh: dword($98D24C2F); e:  -5995 ); e10: -1776 ),
        ( c: ( f: qword($B50918191D8106CD); fh: dword($CC42DD5C); e:  -7962 ); e10: -2368 ),
        ( c: ( f: qword($10CF24303CA163B8); fh: dword($8881FC6C); e:  -9928 ); e10: -2960 ),
        ( c: ( f: qword($BF10EA474FE1E9B1); fh: dword($B674CE73); e: -11895 ); e10: -3552 ),
        ( c: ( f: qword($478E074A0E85FC7F); fh: dword($F3DEFE25); e: -13862 ); e10: -4144 ),
        ( c: ( f: qword($A3BD093CC62364C2); fh: dword($A2FAA242); e: -15828 ); e10: -4736 )
    );
    corrector: array [ 0 .. C_PWR10_COUNT - 1 ] of shortint = (
    // extra mantissa correction [ulp; signed]
        0,  0,  0,  1,  0,  0,  1,  1,  0,  0,  1,  1, -1,  1,  0,  0,
        0,  1,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  1,  2,  2,  0,  1,  1,  0,  0, -2,  0,
        2,  0,  1,  1,  1,  1,  1,  2,  0,  0,  2,  1,  0,  1,  0,  0,
        0,  0,  1, -1,  0,  0,  1,  1,  0,  0,  1,  0, -1,  0, -1,  0,
        0,  0,  1,  0,  0,  0,  1,  0,  0,  0,  1,  1, -1,  0, -1,  1,
        0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0, -1,  0,
       -1,  0,  0, -1,  0, -1,  1,  1,  0, -1,  0,  0, -1, -1, -1,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
       -1,  0,  0,  0,  0, -1,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0,
        2,  2,  1,  1,  0,  0,  0,  2,  0,  0,  1,  1,  0,  0,  1,  1,
        0,  0,  1,  0,  0,  0,  1,  2,  0,  0,  1,  0,  0,  0, -1,  0,
        0,  0,  2,  0,  0,  0,  1,  1,  0,  0,  0,  1, -1,  1,  0,  1,
        0,  0,  0, -1,  0,  0,  0,  1,  0,  0,  1,  0,  0,  0,  0,  0,
        0,  0,  1,  1, -1,  0,  0,  2,  0,  0,  1,  1,  0,  1,  1,  1,
       -1, -1,  1, -2,  0,  0,  0, -1,  1, -1,  1, -1, -1, -1,  0,  0,
        1,  1,  0,  0,  0,  0,  1,  1,  0,  0,  1,  0,  0,  0,  0,  0
    );
{$endif VALREAL_80}
//**************************************
{$ifdef VALREAL_128}
const
    // alpha =-125; gamma = -2
    // full cache: 1E-5032 .. 1E+4995, step = 1E+37
    // sparse = 1/16
    C_PWR10_DELTA = 37;
    C_PWR10_COUNT = 272;
    base: array [ 0 .. 15 ] of TDIY_FP_Power_of_10 = (
        ( c: ( fh: qword($D1476E2C07286FAA); f: qword($1AF5AF660DB4AEE2); e:  -1111 ); e10:  -296 ),
        ( c: ( fh: qword($C4CE17B399107C22); f: qword($CB550FB4384D21D4); e:   -988 ); e10:  -259 ),
        ( c: ( fh: qword($B913179899F68584); f: qword($28E2557B59846E3F); e:   -865 ); e10:  -222 ),
        ( c: ( fh: qword($AE0B158B4738705E); f: qword($9624AB50B148D446); e:   -742 ); e10:  -185 ),
        ( c: ( fh: qword($A3AB66580D5FDAF5); f: qword($C13E60D0D2E0EBBA); e:   -619 ); e10:  -148 ),
        ( c: ( fh: qword($99EA0196163FA42E); f: qword($504BCED1BF8E4E46); e:   -496 ); e10:  -111 ),
        ( c: ( fh: qword($90BD77F3483BB9B9); f: qword($B1C6F22B5E6F48C3); e:   -373 ); e10:   -74 ),
        ( c: ( fh: qword($881CEA14545C7575); f: qword($7E50D64177DA2E55); e:   -250 ); e10:   -37 ),
        ( c: ( fh: qword($8000000000000000); f: qword($0000000000000000); e:   -127 ); e10:     0 ),
        ( c: ( fh: qword($F0BDC21ABB48DB20); f: qword($1E86D40000000000); e:     -5 ); e10:    37 ),
        ( c: ( fh: qword($E264589A4DCDAB14); f: qword($C696963C7EED2DD2); e:    118 ); e10:    74 ),
        ( c: ( fh: qword($D4E5E2CDC1D1EA96); f: qword($6C9E18AC7007C91A); e:    241 ); e10:   111 ),
        ( c: ( fh: qword($C83553C5C8965D3D); f: qword($6F92829494E5ACC7); e:    364 ); e10:   148 ),
        ( c: ( fh: qword($BC4665B596706114); f: qword($873D5D9F0DDE1FEF); e:    487 ); e10:   185 ),
        ( c: ( fh: qword($B10D8E1456105DAD); f: qword($7425A83E872C5F47); e:    610 ); e10:   222 ),
        ( c: ( fh: qword($A67FF273B8460356); f: qword($8A892ABAF368F137); e:    733 ); e10:   259 )
    );
    factor_plus: array [ 0 .. 7 ] of TDIY_FP_Power_of_10 = (
        ( c: ( fh: qword($BF87DECC3576D3D1); f: qword($49738B9F99B4642D); e:   1839 ); e10:   592 ),
        ( c: ( fh: qword($8F4C0691750E8305); f: qword($0A40DE037C9AD730); e:   3806 ); e10:  1184 ),
        ( c: ( fh: qword($D66B8D68727E5D97); f: qword($56BC4BF837B34968); e:   5772 ); e10:  1776 ),
        ( c: ( fh: qword($A06C0BD4CE9DB63F); f: qword($D51AF6A3244A6983); e:   7739 ); e10:  2368 ),
        ( c: ( fh: qword($F00B82D75A7ADBC5); f: qword($B8787D891AB45D5B); e:   9705 ); e10:  2960 ),
        ( c: ( fh: qword($B397FD9A22D732D7); f: qword($AE7EDAA76FBBD923); e:  11672 ); e10:  3552 ),
        ( c: ( fh: qword($865DB7A9CCD2839E); f: qword($0367500A8E9A1790); e:  13639 ); e10:  4144 ),
        ( c: ( fh: qword($C90E78C7FCBEE713); f: qword($F3BE171A27BF81DB); e:  15605 ); e10:  4736 )
    );
    factor_minus: array [ 0 .. 7 ] of TDIY_FP_Power_of_10 = (
        ( c: ( fh: qword($AB15B5D22F85DC7A); f: qword($E66FEACEB7836F69); e:  -2094 ); e10:  -592 ),
        ( c: ( fh: qword($E4AC057C4237088F); f: qword($4C7284F9EDDA793D); e:  -4061 ); e10: -1184 ),
        ( c: ( fh: qword($98D24C2FD2DCB34C); f: qword($EC42875C0B22B986); e:  -6027 ); e10: -1776 ),
        ( c: ( fh: qword($CC42DD5CB5091819); f: qword($1D8106CCF8EE85B4); e:  -7994 ); e10: -2368 ),
        ( c: ( fh: qword($8881FC6C10CF2430); f: qword($3CA163B873AA88A6); e:  -9960 ); e10: -2960 ),
        ( c: ( fh: qword($B674CE73BF10EA47); f: qword($4FE1E9B0FCDF7B3D); e: -11927 ); e10: -3552 ),
        ( c: ( fh: qword($F3DEFE25478E074A); f: qword($0E85FC7F4EDBD3CB); e: -13894 ); e10: -4144 ),
        ( c: ( fh: qword($A2FAA242A3BD093C); f: qword($C62364C260A887E2); e: -15860 ); e10: -4736 )
    );
    corrector: array [ 0 .. C_PWR10_COUNT - 1 ] of shortint = (
      // extra mantissa correction [ulp; signed]
        1,  0,  1,  0,  1,  1,  0,  0,  0,  0,  0,  1,  0,  0,  2,  0,
       -1, -1,  0, -1,  0, -1,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0,
        1,  0,  0,  0,  1, -1,  0, -1, -1,  1,  0,  1,  0,  0,  1,  1,
        0, -2,  0,  0,  0, -1,  0,  0,  0,  0, -2,  0,  0,  0,  0,  0,
        0, -1,  1,  0,  1,  0,  0, -1,  0,  1,  0,  0,  1,  0,  1,  1,
        1, -1,  0,  0,  1, -1,  0,  0,  0,  1,  0,  1,  1, -1,  1,  1,
        0,  0,  1,  0,  0,  0, -1,  0, -1,  0,  0,  0,  0,  0,  0,  1,
        0,  0,  2,  1,  0, -1, -1, -1, -1,  0, -1,  1,  0, -1,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0, -1,  1, -1, -1,  0, -1,  0, -1,  0,  0,  0,  0,  0,
        1, -1,  2,  1,  2,  0, -1,  1,  0,  0,  0,  1,  2,  0,  1,  1,
        0,  0,  1,  0,  1,  0,  0,  0,  0,  1,  0,  1,  1,  0,  1,  1,
        0,  0,  0,  1,  0, -1, -1,  0, -1,  0,  0,  0,  0,  0,  0,  1,
       -1, -1,  0,  0,  0,  0,  0, -1, -1,  0,  0,  0,  1,  0,  0,  0,
        0,  0,  1,  0, -1,  0,  0,  0, -1,  0, -1,  0,  1,  0,  0, -1,
        0, -1,  1, -1,  1, -1,  0, -1,  0,  1, -1,  0,  1,  1,  1,  1,
        0, -1,  1, -1,  0, -2,  0, -1, -1,  0, -1,  0,  0, -1,  0,  0
    );
{$endif VALREAL_128}
//**************************************
{$ifndef VALREAL_32} // common for float64..float128
var
    i, xmul, inod, min10: integer;
    A: TDIY_FP_Power_of_10;
  {$ifdef VALREAL_80}
    ch: dword;
  {$endif}
  {$ifdef VALREAL_128}
    ch: qword;
  {$endif}
    cx: shortint;
begin
    // find non-sparse index
    min10 := base [ low( base ) ].e10 + factor_minus[ high( factor_minus ) ].e10;
    if ( exp10 <= min10 ) then
        i := 0
    else
    begin
        i := ( exp10 - min10 ) div C_PWR10_DELTA;
        if ( i * C_PWR10_DELTA + min10 <> exp10 ) then
            inc( i ); // round-up
        if ( i > C_PWR10_COUNT - 1 ) then
            i := C_PWR10_COUNT - 1;
    end;
    // generate result
    inod := i mod length( base );
    xmul := ( i div length( base ) ) - length( factor_minus );
    if ( xmul = 0 ) then
    begin
        // base
        factor := base[ inod ];
        exit;
    end;
    // surrogate
    A := base[ inod ];
    if ( xmul > 0 ) then
    begin
        dec( xmul );
        factor.e10 := A.e10 + factor_plus[ xmul ].e10;
        if ( A.e10 <> 0 ) then
            factor.c := diy_fp_multiply( A.c, factor_plus[ xmul ].c, TRUE )
        else
        begin
            // exact
            factor.c := factor_plus[ xmul ].c;
            exit;
        end;
    end
    else
    begin
        xmul := - ( xmul + 1 );
        factor.e10 := A.e10 + factor_minus[ xmul ].e10;
        if ( A.e10 <> 0 ) then
            factor.c := diy_fp_multiply( A.c, factor_minus[ xmul ].c, TRUE )
        else
        begin
            // exact
            factor.c := factor_minus[ xmul ].c;
            exit;
        end;
    end;
    // adjust mantissa
    cx := corrector[ i ];
    if ( cx <> 0 ) then
  {$ifdef VALREAL_64}
        inc( factor.c.f, int64( cx ) );
  {$else VALREAL_80 | VALREAL_128}
    begin
        ch := 0;
        if ( cx < 0 ) then
            dec( ch );
        diy_util_add( factor.c.fh, factor.c.f, ch, int64( cx ) );
    end;
  {$endif VALREAL_*}
    //
end;
{$endif VALREAL_64..VALREAL_128}

(*==========================================================================*
 *                                                                          *
 *                              Float -> ASCII                              *
 *                                                                          *
 *==========================================================================*)

procedure str_real( min_width, frac_digits: integer; const v: ValReal; real_type: TReal_Type; out str: shortstring );

{$undef VALREAL_PACK}
{$i flt_pack.inc}

const
{$ifdef VALREAL_32}
    C_FRAC2_BITS  = 23;
    C_EXP2_BIAS   = 127;
    C_DIY_FP_Q    = 32;
    C_GRISU_ALPHA =-29;
    C_GRISU_GAMMA = 0;
    RT_NATIVE = RT_S32REAL;
{$endif VALREAL_32}
{$ifdef VALREAL_64}
    C_FRAC2_BITS  = 52;
    C_EXP2_BIAS   = 1023;
    C_DIY_FP_Q    = 64;
    C_GRISU_ALPHA =-61;
    C_GRISU_GAMMA = 0;
    RT_NATIVE = RT_S64REAL;
{$endif VALREAL_64}
{$ifdef VALREAL_80}
    C_FRAC2_BITS  = 63;
    C_EXP2_BIAS   = 16383;
    C_DIY_FP_Q    = 96;
    C_GRISU_ALPHA =-93;
    C_GRISU_GAMMA = 30;
    RT_NATIVE = RT_S80REAL;
{$endif VALREAL_80}
{$ifdef VALREAL_128}
    C_FRAC2_BITS  = 112;
    C_EXP2_BIAS   = 16383;
    C_DIY_FP_Q    = 128;
    C_GRISU_ALPHA =-125;
    C_GRISU_GAMMA =-2;
    RT_NATIVE = RT_S128REAL;
{$endif VALREAL_128}

(****************************************************************************)
    // handy const
    C_EXP2_SPECIAL = C_EXP2_BIAS * 2 + 1;
{$ifdef VALREAL_32}
    C_MANT2_INTEGER = dword(1) shl C_FRAC2_BITS;
{$endif VALREAL_32}
{$if defined(VALREAL_64) or defined(VALREAL_80)}
    C_MANT2_INTEGER = qword(1) shl C_FRAC2_BITS;
{$endif VALREAL_64 | VALREAL_80}
{$ifdef VALREAL_128}
    C_MANT2_INTEGER_H = qword(1) shl ( C_FRAC2_BITS - 64 );
{$endif VALREAL_128}

    C_MAX_WIDTH = 255; // shortstring

{$ifdef fpc_softfpu_implementation}
    C_NO_MIN_WIDTH = -1; // just for convenience
{$else}
    C_NO_MIN_WIDTH = -32767; // this value is the one used internally by FPC
{$endif}

type
    TAsciiDigits = array [ 0 .. 39 ] of byte;

(*-------------------------------------------------------
 | gen_digits_32 [local]
 | gen_digits_64 [local] (used only for float64..float128 -> ASCII)
 |
 | These routines perform conversion of 32-bit/64-bit unsigned integer
 | to the sequence of byte-sized decimal digits.
 |
 *-------------------------------------------------------*)
function gen_digits_32( var buf: TAsciiDigits; pos: integer; x: dword; pad_9zero: boolean = false ): integer;
const
    digits: array [ 0 .. 9 ] of dword = (
              0,
             10,
            100,
           1000,
          10000,
         100000,
        1000000,
       10000000,
      100000000,
     1000000000
    );
var
    n: integer;
    m, z: dword;
begin
    // Calculate amount of digits
    if ( x = 0 ) then
        // emit nothing if padding is not required
        n := 0
    else
    begin
        n :=( ( BSRdword( x ) + 1 ) * 1233 ) shr 12;
        if ( x >= digits[ n ] ) then
            inc( n );
    end;
    if pad_9zero and ( n < 9 ) then
        n := 9;
    gen_digits_32 := n;
    // Emit digits
    m := x;
    while ( n > 0 ) do
    begin
        dec( n );
        if ( m <> 0 ) then
        begin
            z := m div 10;
            buf[ pos + n ] := m - z * 10;
            m := z;
        end
        else
            buf[ pos + n ] := 0;
    end;
end;

{$ifndef VALREAL_32}
function gen_digits_64( var buf: TAsciiDigits; pos: integer; const x: qword; pad_19zero: boolean = false ): integer;
var
    n_digits: integer;
    temp: qword;
    splitl, splitm, splith: dword;
begin
    // Split X into 3 unsigned 32-bit integers; lower two should be less than 10 digits long
    if ( x < 1000000000 ) then
    begin
        splith := 0;
        splitm := 0;
        splitl := x;
    end
    else
    begin
        temp := x div 1000000000;
        splitl := x - temp * 1000000000;
        if ( temp < 1000000000 ) then
        begin
            splith := 0;
            splitm := temp;
        end
        else
        begin
            splith := temp div 1000000000;
            splitm := lo( temp ) - splith * 1000000000;
        end;
    end;
    // Generate digits
    n_digits := gen_digits_32( buf, pos, splith, false );
    if pad_19zero and ( n_digits = 0 ) then
    begin
        // at most 18 digits expected from splitm and splitl, so add one more
        buf[ pos ] := 0;
        n_digits := 1;
    end;
    inc( n_digits, gen_digits_32( buf, pos + n_digits, splitm, n_digits <> 0 ) );
    inc( n_digits, gen_digits_32( buf, pos + n_digits, splitl, n_digits <> 0 ) );
    gen_digits_64 := n_digits;
end;
{$endif VALREAL_64..VALREAL_128}

(*-------------------------------------------------------
 | round_digits [local]
 |
 | Performs digit sequence rounding, returns decimal point correction.
 |
 *-------------------------------------------------------*)
function round_digits( var buf: TAsciiDigits; var n_current: integer; n_max: integer; half_round_to_even: boolean = true ): integer;
var
    n: integer;
    dig_round, dig_sticky: byte;
  {$ifdef GRISU1_F2A_AGRESSIVE_ROUNDUP}
    i: integer;
  {$endif}
begin
    round_digits := 0;
    n := n_current;
{$ifdef grisu1_debug}
    assert( n_max >= 0 );
    assert( n_max < n );
{$endif grisu1_debug}
    n_current := n_max;
    // Get round digit
    dig_round := buf[n_max];

{$ifdef GRISU1_F2A_AGRESSIVE_ROUNDUP}
    // Detect if rounding-up the second last digit turns the "dig_round"
    // into "5"; also make sure we have at least 1 digit between "dig_round"
    // and the second last.
    if not half_round_to_even then
        if ( dig_round = 4 ) and ( n_max < n - 3 ) then
            if ( buf[ n - 2 ] >= 8 ) then // somewhat arbitrary..
            begin
                // check for only "9" are in between
                i := n - 2;
                repeat
                    dec( i );
                until ( i = n_max ) or ( buf[i] <> 9 );
                if ( i = n_max ) then
                    // force round-up
                    dig_round := 9; // any value ">=5"
            end;
{$endif}

    if ( dig_round < 5 ) then
        exit;

    // Handle "round half to even" case
    if ( dig_round = 5 ) and half_round_to_even and ( ( n_max = 0 ) or ( buf[ n_max - 1 ] and 1 = 0 ) ) then
    begin
        // even and a half: check if exactly the half
        dig_sticky := 0;
        while ( n > n_max + 1 ) and ( dig_sticky = 0 ) do
        begin
            dec( n );
            dig_sticky := buf[n];
        end;
        if ( dig_sticky = 0 ) then
            exit; // exactly a half -> no rounding is required
    end;

    // Round-up
    while ( n_max > 0 ) do
    begin
        dec( n_max );
        inc( buf[n_max] );
        if ( buf[n_max] < 10 ) then
        begin
            // no more overflow: stop now
            n_current := n_max + 1;
            exit;
        end;
        // continue rounding
    end;

    // Overflow out of the 1st digit, all n_max digits became 0
    buf[0] := 1;
    n_current := 1;
    round_digits := 1;
end;

(*-------------------------------------------------------
 | do_fillchar [local]
 |
 | Fills string region with certain character.
 |
 *-------------------------------------------------------*)
{$ifdef cpujvm}
procedure do_fillchar( var str: shortstring; pos, count: integer; c: char );
begin
  while count>0 do
    begin
      str[pos]:=c;
      inc(pos);
      dec(count);
    end;
end;
{$else not cpujvm}
procedure do_fillchar( var str: shortstring; pos, count: integer; c: char ); {$ifdef grisu1_inline}inline;{$endif}
begin
  fillchar( str[pos], count, c );
end;
{$endif cpujvm}

(*-------------------------------------------------------
 | try_return_fixed [local]
 |
 | This routine tries to format the number in the fixed-point representation.
 | If the resulting string is estimated to be too long to fit into shortstring,
 | routine returns FALSE giving the caller a chance to return the exponential
 | representation.
 | Otherwise, it returns TRUE.
 |
 | Not implemented [and why to do it at all?]:
 |     Here also a good place to limit the fixed point formatting by exponent
 |     range, falling back to exponential notation (just return FALSE).
 |
 *-------------------------------------------------------*)
function try_return_fixed( out str: shortstring; minus: boolean; const digits: TAsciiDigits; n_digits_have, fixed_dot_pos, min_width, frac_digits: integer ): boolean;
var
    rounded: boolean;
    temp_round: TAsciiDigits;
    i, j, len, cut_digits_at: integer;
    n_spaces, n_spaces_max, n_before_dot, n_before_dot_pad0, n_after_dot_pad0, n_after_dot, n_tail_pad0: integer;
begin
    try_return_fixed := false;
{$ifdef grisu1_debug}
    assert( n_digits_have >= 0 );
    assert( min_width <= C_MAX_WIDTH );
    assert( frac_digits >= 0 );
    assert( frac_digits <= C_MAX_WIDTH - 3 );
{$endif grisu1_debug}
    // Round digits if necessary
    rounded := false;
    cut_digits_at := fixed_dot_pos + frac_digits;
    if ( cut_digits_at < 0 ) then
        // zero
        n_digits_have := 0
    else
    if ( cut_digits_at < n_digits_have ) then
    begin
        // round digits
{$ifdef cpujvm}
        temp_round := digits;
{$else not cpujvm}
        if ( n_digits_have > 0 ) then
            move( digits, temp_round, n_digits_have * sizeof( digits[0] ) );
{$endif cpujvm}
        inc( fixed_dot_pos, round_digits( temp_round, n_digits_have, cut_digits_at {$ifdef GRISU1_F2A_HALF_ROUNDUP}, false {$endif} ) );
        rounded := true;
    end;
    // Before dot: digits, pad0
    if ( fixed_dot_pos <= 0 ) or ( n_digits_have = 0 ) then
    begin
        n_before_dot := 0;
        n_before_dot_pad0 := 1;
    end
    else
    if ( fixed_dot_pos > n_digits_have ) then
    begin
        n_before_dot := n_digits_have;
        n_before_dot_pad0 := fixed_dot_pos - n_digits_have;
    end
    else
    begin
        n_before_dot := fixed_dot_pos;
        n_before_dot_pad0 := 0;
    end;
    // After dot: pad0, digits, pad0
    if ( fixed_dot_pos < 0 ) then
        n_after_dot_pad0 := - fixed_dot_pos
    else
        n_after_dot_pad0 := 0;
    if ( n_after_dot_pad0 > frac_digits ) then
        n_after_dot_pad0 := frac_digits;
    n_after_dot := n_digits_have - n_before_dot;
    n_tail_pad0 := frac_digits - n_after_dot - n_after_dot_pad0;
{$ifdef grisu1_debug}
    assert( n_tail_pad0 >= 0 );
{$endif grisu1_debug}
    // Estimate string length
    len := ord( minus ) + n_before_dot + n_before_dot_pad0;
    if ( frac_digits > 0 ) then
        inc( len, n_after_dot_pad0 + n_after_dot + n_tail_pad0 + 1 );
    n_spaces_max := C_MAX_WIDTH - len;
    if ( n_spaces_max < 0 ) then
        exit;
    // Calculate space-padding length
    n_spaces := min_width - len;
    if ( n_spaces > n_spaces_max ) then
        n_spaces := n_spaces_max;
    if ( n_spaces > 0 ) then
        inc( len, n_spaces );
    // Allocate storage
    SetLength( str, len );
    i := 1;
    // Leading spaces
    if ( n_spaces > 0 ) then
    begin
        do_fillchar( str, i, n_spaces, ' ' );
        inc( i, n_spaces );
    end;
    // Sign
    if minus then
    begin
        str[i] := '-';
        inc( i );
    end;
    // Integer significant digits
    j := 0;
    if rounded then
        while ( n_before_dot > 0 ) do
        begin
            str[i] := char( temp_round[j] + ord('0') );
            inc( i );
            inc( j );
            dec( n_before_dot );
        end
    else
        while ( n_before_dot > 0 ) do
        begin
            str[i] := char( digits[j] + ord('0') );
            inc( i );
            inc( j );
            dec( n_before_dot );
        end;
    // Integer 0-padding
    if ( n_before_dot_pad0 > 0 ) then
    begin
        do_fillchar( str, i, n_before_dot_pad0, '0' );
        inc( i, n_before_dot_pad0 );
    end;
    //
    if ( frac_digits <> 0 ) then
    begin
        // Dot
        str[i] := '.';
        inc( i );
        // Pre-fraction 0-padding
        if ( n_after_dot_pad0 > 0 ) then
        begin
            do_fillchar( str, i, n_after_dot_pad0, '0' );
            inc( i, n_after_dot_pad0 );
        end;
        // Fraction significant digits
        if rounded then
            while ( n_after_dot > 0 ) do
            begin
                str[i] := char( temp_round[j] + ord('0') );
                inc( i );
                inc( j );
                dec( n_after_dot );
            end
        else
            while ( n_after_dot > 0 ) do
            begin
                str[i] := char( digits[j] + ord('0') );
                inc( i );
                inc( j );
                dec( n_after_dot );
            end;
        // Tail 0-padding
        if ( n_tail_pad0 > 0 ) then
        begin
            do_fillchar( str, i, n_tail_pad0, '0' );
{$ifdef grisu1_debug}
            inc( i, n_tail_pad0 );
{$endif grisu1_debug}
        end;
    end;
    //
{$ifdef grisu1_debug}
    assert( i = len + 1 );
{$endif grisu1_debug}
    try_return_fixed := true
end;

(*-------------------------------------------------------
 | return_exponential [local]
 |
 | Formats the number in the exponential representation.
 |
 *-------------------------------------------------------*)
procedure return_exponential( out str: shortstring; minus: boolean; const digits: TAsciiDigits; n_digits_have, n_digits_req, d_exp, n_digits_exp, min_width: integer );
var
    e_minus: boolean;
    i, j, len, n_exp, n_spaces, n_spaces_max: integer;
    buf_exp: TAsciiDigits;
begin
{$ifdef grisu1_debug}
    assert( n_digits_have >= 0 );
    assert( n_digits_have <= n_digits_req );
    assert( min_width <= C_MAX_WIDTH );
{$endif grisu1_debug}
    // Prepare exponent
    e_minus := ( d_exp < 0 );
    if e_minus then
        d_exp := - d_exp;
    n_exp := gen_digits_32( buf_exp, 0, d_exp, false );
    if ( n_exp <= n_digits_exp ) then
        len := n_digits_exp
    else
        len := n_exp;
    // Estimate string length
    inc( len, 1{sign} + n_digits_req + 1{E} + 1{E-sign} );
    if ( n_digits_req > 1 ) then
        inc( len ); // dot
    // Calculate space-padding length
    n_spaces_max := C_MAX_WIDTH - len;
    n_spaces := min_width - len;
    if ( n_spaces > n_spaces_max ) then
        n_spaces := n_spaces_max;
    if ( n_spaces > 0 ) then
        inc( len, n_spaces );
    // Allocate storage
    SetLength( str, len );
    i := 1;
    // Leading spaces
    if ( n_spaces > 0 ) then
    begin
        do_fillchar( str, i, n_spaces, ' ' );
        inc( i, n_spaces );
    end;
    // Sign
    if minus then
        str[i] := '-'
    else
        str[i] := ' ';
    inc( i );
    // Integer part
    if ( n_digits_have > 0 ) then
        str[i] := char( digits[0] + ord('0') )
    else
        str[i] := '0';
    inc( i );
    // Dot
    if ( n_digits_req > 1 ) then
    begin
        str[i] := '.';
        inc( i );
    end;
    // Fraction significant digits
    j := 1;
    while ( j < n_digits_have ) and ( j < n_digits_req ) do
    begin
        str[i] := char( digits[j] + ord('0') );
        inc( i );
        inc( j );
    end;
    // Fraction 0-padding
    j := n_digits_req - j;
    if ( j > 0 ) then
    begin
        do_fillchar( str, i, j, '0' );
        inc( i, j );
    end;
    // Exponent designator
    str[i] := 'E';
    inc( i );
    // Exponent sign
    if e_minus then
        str[i] := '-'
    else
        str[i] := '+';
    inc( i );
    // Exponent 0-padding
    j := n_digits_exp - n_exp;
    if ( j > 0 ) then
    begin
        do_fillchar( str, i, j, '0' );
        inc( i, j );
    end;
    // Exponent digits
    for j := 0 to n_exp - 1 do
    begin
        str[i] := char( buf_exp[j] + ord('0') );
        inc( i );
    end;
{$ifdef grisu1_debug}
    assert( i = len + 1 );
{$endif grisu1_debug}
end;

(*-------------------------------------------------------
 | return_special [local]
 |
 | This routine formats one of special results.
 |
 *-------------------------------------------------------*)
procedure return_special(  out str: shortstring; sign: integer; const spec: shortstring; min_width: integer );
var
    i, slen, len, n_spaces, n_spaces_max: integer;
begin
    slen := length(spec);
    if ( sign = 0 ) then
        len := slen
    else
        len := slen + 1;
    n_spaces_max := C_MAX_WIDTH - len;
    // Calculate space-padding length
    n_spaces := min_width - len;
    if ( n_spaces > n_spaces_max ) then
        n_spaces := n_spaces_max;
    if ( n_spaces > 0 ) then
        inc( len, n_spaces );
    // Allocate storage
    SetLength( str, len );
    i := 1;
    // Leading spaces
    if ( n_spaces > 0 ) then
    begin
        do_fillchar( str, i, n_spaces, ' ' );
        inc( i, n_spaces );
    end;
    // Sign
    if ( sign <> 0 ) then
    begin
        if ( sign > 0 ) then
            str[i] := '+'
        else
            str[i] := '-';
        inc( i );
    end;
    // Special
    while slen>0 do
      begin
        str[i+slen-1] := spec[slen];
        dec(slen);
      end;
end;

{$if defined(VALREAL_80) or defined(VALREAL_128)}
{$ifndef FPC_SYSTEM_HAS_U128_DIV_U64_TO_U64}
(*-------------------------------------------------------
 | u128_div_u64_to_u64 [local]
 |
 | Divides unsigned 128-bit integer by unsigned 64-bit integer.
 | Returns 64-bit quotient and reminder.
 |
 | This routine is used here only for splitting specially prepared unsigned
 | 128-bit integer into two 64-bit ones before converting it to ASCII.
 |
 *-------------------------------------------------------*)
function u128_div_u64_to_u64( const xh, xl: qword; const y: qword; out quotient, remainder: qword ): boolean;
var
    b,                // Number base
    v : qword;        // Norm. divisor
    un1, un0,         // Norm. dividend LSD's
    vn1, vn0 : dword; // Norm. divisor digits
    q1, q0,           // Quotient digits
    un64, un21, un10, // Dividend digit pairs
    rhat: qword;      // A remainder
    s: integer;       // Shift amount for norm
begin
    // Overflow check
    if ( xh >= y ) then
    begin
        u128_div_u64_to_u64 := false;
        exit;
    end;
    // Count leading zeros
    s := 63 - BSRqword( y ); // 0 <= s <= 63
    // Normalize divisor
    v := y shl s;
    // Break divisor up into two 32-bit digits
    vn1 := hi(v);
    vn0 := lo(v);
    // Shift dividend left
    un64 := xh shl s;
    if ( s > 0 ) then
        un64 := un64 or ( xl shr ( 64 - s ) );
    un10 := xl shl s;
    // Break right half of dividend into two digits
    un1 := hi(un10);
    un0 := lo(un10);
    // Compute the first quotient digit, q1
    q1 := un64 div vn1;
    rhat := un64 - q1 * vn1;
    b := qword(1) shl 32; // Number base
    while ( hi(q1) <> 0 ) or ( q1 * vn0 > b * rhat + un1 ) do
    begin
        dec( q1 );
        inc( rhat, vn1 );
        if rhat >= b then
            break;
    end;
    // Multiply and subtract
    un21 := un64 * b + un1 - q1 * v;
    // Compute the second quotient digit, q0
    q0 := un21 div vn1;
    rhat := un21 - q0 * vn1;
    while ( hi(q0) <> 0 ) or ( q0 * vn0 > b * rhat + un0 ) do
    begin
        dec( q0 );
        inc( rhat, vn1 );
        if ( hi(rhat) <> 0 ) then
            break;
    end;
    // Result
    remainder := ( un21 * b + un0 - q0 * v ) shr s;
    quotient := q1 * b + q0;
    u128_div_u64_to_u64 := true;
end;
{$endif FPC_SYSTEM_HAS_U128_DIV_U64_TO_U64}
{$endif VALREAL_80 | VALREAL_128}

(*-------------------------------------------------------
 | count_leading_zero [local]
 |
 | Counts number of 0-bits at most significant bit position.
 |
 *-------------------------------------------------------*)
{$ifdef VALREAL_32}
function count_leading_zero( const X: dword ): integer; {$ifdef grisu1_inline}inline;{$endif}
begin
    count_leading_zero := 31 - BSRdword( X );
end;
{$else not VALREAL_32}
function count_leading_zero( const X: qword ): integer; {$ifdef grisu1_inline}inline;{$endif}
begin
    count_leading_zero := 63 - BSRqword( X );
end;
{$endif VALREAL_*}

{$if defined(VALREAL_80) or defined(VALREAL_128)}
(*-------------------------------------------------------
 | make_frac_mask [local]
 |
 | Makes DIY_FP fractional part mask:
 |     result := ( 1 shl one.e ) - 1
 |
 *-------------------------------------------------------*)
{$ifdef VALREAL_80}
procedure make_frac_mask( out h: dword; out l: qword; one_e: integer ); {$ifdef grisu1_inline}inline;{$endif}
{$else VALREAL_128}
procedure make_frac_mask( out h, l: qword; one_e: integer ); {$ifdef grisu1_inline}inline;{$endif}
{$endif VALREAL_*}
begin
  {$ifdef grisu1_debug}
    assert( one_e <= 0 );
    assert( one_e > - ( sizeof( l ) + sizeof( h ) ) * 8 );
  {$endif grisu1_debug}
    if ( one_e <= - 64 ) then
    begin
        l := qword( -1 );
        h := ( {$ifdef VALREAL_128} qword {$else} dword {$endif} ( 1 ) shl ( - one_e - 64 ) ) - 1;
    end
    else
    begin
        l := ( qword( 1 ) shl ( - one_e ) ) - 1;
        h := 0;
    end;
end;
{$endif VALREAL_80 | VALREAL_128}

(*-------------------------------------------------------
 | k_comp [local]
 |
 | Calculates the exp10 of a factor required to bring the binary exponent
 | of the original number into selected [ alpha .. gamma ] range:
 |     result := ceiling[ ( alpha - e ) * log10(2) ]
 |
 *-------------------------------------------------------*)
function k_comp( e, alpha{, gamma}: integer ): integer;
{$ifdef fpc_softfpu_implementation}
///////////////
//
// Assuming no HardFloat available.
// Note: using softfpu here significantly slows down overall
// conversion performance, so we use integers.
//
const
    D_LOG10_2: TDIY_FP = // log10(2) = 0.301029995663981195213738894724493027
      {$ifdef VALREAL_32}
        ( f: dword($9A209A85); e: -33 );
      {$endif}
      {$ifdef VALREAL_64}
        ( f: qword($9A209A84FBCFF799); e: -65 );
      {$endif}
      {$ifdef VALREAL_80}
        ( f: qword($FBCFF7988F8959AC); fh: dword($9A209A84); e: -97 );
      {$endif}
      {$ifdef VALREAL_128}
        ( fh: qword($9A209A84FBCFF798); f: qword($8F8959AC0B7C9178); e: -129 );
      {$endif}
var
    x, n: integer;
    y, z: TDIY_FP;
  {$ifdef VALREAL_32}
    mask_one: dword;
  {$else not VALREAL_32}
    mask_one: qword;
  {$endif}
  {$ifdef VALREAL_80}
    mask_oneh: dword;
  {$endif}
  {$ifdef VALREAL_128}
    mask_oneh: qword;
  {$endif}
    plus, round_up: boolean;
begin
    x := alpha - e;
    if ( x = 0 ) then
    begin
        k_comp := 0;
        exit;
    end;
    plus := ( x > 0 );
    if plus then
        y.f := x
    else
        y.f := - x;
    round_up := plus;
    n := C_DIY_FP_Q - 1 - BSRdword( y.f );
  {$if defined(VALREAL_32) or defined(VALREAL_64)}
    y.f := y.f shl n;
  {$else VALREAL_80 | VALREAL_128}
    y.fh := 0;
    diy_util_shl( y.fh, y.f, n );
  {$endif VALREAL_*}
    y.e := - n;
    z := diy_fp_multiply( y, D_LOG10_2, false );
    if ( z.e <= - C_DIY_FP_Q ) then
    begin
        round_up := plus and ( 0 <>
  {$if defined(VALREAL_32) or defined(VALREAL_64)}
          z.f
  {$else VALREAL_80 | VALREAL_128}
          z.f or z.fh
  {$endif}
        );
        n := 0;
    end
    else
    begin
        if plus then
        begin
          {$if defined(VALREAL_32) or defined(VALREAL_64)}
            mask_one := ( {$ifdef VALREAL_64} qword {$else} dword {$endif} ( 1 ) shl ( - z.e ) ) - 1;
            round_up := ( z.f and mask_one <> 0 );
          {$else VALREAL_80 | VALREAL_128}
            make_frac_mask( mask_oneh, mask_one, z.e );
            round_up := ( z.f and mask_one <> 0 ) or ( z.fh and mask_oneh <> 0 );
          {$endif VALREAL_*}
        end;
      {$if defined(VALREAL_32) or defined(VALREAL_64)}
        n := z.f shr ( - z.e );
      {$else VALREAL_80 | VALREAL_128}
        diy_util_shr( z.fh, z.f, - z.e );
        n := z.f;
      {$endif VALREAL_*}
    end;
    if not plus then
        n := - n;
    if round_up then
        k_comp := n + 1
    else
        k_comp := n;
end;
{$else not fpc_softfpu_implementation}
///////////////
//
// HardFloat implementation
//
{$if defined(SUPPORT_SINGLE) and defined(VALREAL_32)}
// If available, use single math for VALREAL_32
var
    dexp: single;
const
    D_LOG10_2: single =
{$elseif defined(SUPPORT_DOUBLE) and not defined(VALREAL_32)}
// If available, use double math for all types >VALREAL_32
var
    dexp: double;
const
    D_LOG10_2: double =
{$else}
// Use native math
var
    dexp: ValReal;
const
    D_LOG10_2: ValReal =
{$endif}
    0.301029995663981195213738894724493027; // log10(2)
var
    x, n: integer;
begin
    x := alpha - e;
    dexp := x * D_LOG10_2;
    // ceil( dexp )
    n := trunc( dexp );
    if ( x > 0 ) then
        if ( dexp <> n ) then
            inc( n ); // round-up
    k_comp := n;
end;
{$endif fpc_softfpu_implementation}

(****************************************************************************)
var
    w, D: TDIY_FP;
    c_mk: TDIY_FP_Power_of_10;
    n, mk, dot_pos, n_digits_exp, n_digits_need, n_digits_have: integer;
    n_digits_req, n_digits_sci: integer;
    minus: boolean;
  {$ifndef VALREAL_32}
    fl, one_maskl: qword;
  {$endif not VALREAL_32}
  {$ifdef VALREAL_80}
    templ: qword;
    fh, one_maskh, temph: dword;
  {$endif VALREAL_80}
  {$ifdef VALREAL_128}
    templ: qword;
    fh, one_maskh, temph: qword;
  {$endif VALREAL_128}
    one_e: integer;
    one_mask, f: dword;
    buf: TAsciiDigits;

begin

    // Limit parameters
    if ( frac_digits > 216 ) then
        frac_digits := 216; // Delphi compatible
    if ( min_width <= C_NO_MIN_WIDTH ) then
        min_width := -1 // no minimal width
    else
        if ( min_width < 0 ) then
            min_width := 0 // minimal width is as short as possible
        else
            if ( min_width > C_MAX_WIDTH ) then
                min_width := C_MAX_WIDTH;

    // Format profile: select "n_digits_need" and "n_digits_exp"
    n_digits_req := float_format[real_type].nDig_mantissa;
    n_digits_exp := float_format[real_type].nDig_exp10;
    // number of digits to be calculated by Grisu
    n_digits_need := float_format[RT_NATIVE].nDig_mantissa;
    if ( n_digits_req < n_digits_need ) then
        n_digits_need := n_digits_req;
    // number of mantissa digits to be printed in exponential notation
    if ( min_width < 0 ) then
        n_digits_sci := n_digits_req
    else
    begin
        n_digits_sci := min_width - 1{sign} - 1{dot} - 1{E} - 1{E-sign} - n_digits_exp;
        if ( n_digits_sci < 2 ) then
            n_digits_sci := 2; // at least 2 digits
        if ( n_digits_sci > n_digits_req ) then
            n_digits_sci := n_digits_req; // at most requested by real_type
    end;

    // Float -> DIY_FP
    w := unpack_float( v, minus );

    // Handle Zero
    if ( w.e = 0 ) and ( w.f {$ifdef VALREAL_128} or w.fh {$endif} = 0 ) then
    begin
        buf[0] := 0; // to avoid "warning: uninitialized"
        if ( frac_digits >= 0 ) then
            if try_return_fixed( str, minus, buf, 0, 1, min_width, frac_digits ) then
                exit
          {$ifdef grisu1_debug}
            else
                assert( FALSE ) // should never fail with these arguments
          {$endif grisu1_debug};
        return_exponential( str, minus, buf, 0, n_digits_sci, 0, n_digits_exp, min_width );
        exit;
    end;

  {$ifdef VALREAL_80}
    // Handle non-normals
    if ( w.e <> 0 ) and ( w.e <> C_EXP2_SPECIAL ) then
        if ( w.f and C_MANT2_INTEGER = 0 ) then
        begin
            // -> QNaN
            w.f := qword(-1);
            w.e := C_EXP2_SPECIAL;
        end;
  {$endif VALREAL_80}

    // Handle specials
    if ( w.e = C_EXP2_SPECIAL ) then
    begin
        if ( min_width < 0 ) then
            // backward compat..
            min_width := float_format[real_type].nDig_mantissa + float_format[real_type].nDig_exp10 + 4;
        n := 1 - ord(minus) * 2; // default special sign [-1|+1]
      {$if defined(VALREAL_32) or defined(VALREAL_64)}
        if ( w.f = 0 ) then
      {$endif VALREAL_32 | VALREAL_64}
      {$ifdef VALREAL_80}
        if ( w.f = qword(C_MANT2_INTEGER) ) then
      {$endif VALREAL_80}
      {$ifdef VALREAL_128}
        if ( w.fh or w.f = 0 ) then
      {$endif VALREAL_128}
        begin
           // Inf
           return_special( str, n, C_STR_INF, min_width );
        end
        else
        begin
           // NaN [also pseudo-NaN, pseudo-Inf, non-normal for floatx80]
       {$ifdef GRISU1_F2A_NAN_SIGNLESS}
           n := 0;
       {$endif}
       {$ifndef GRISU1_F2A_NO_SNAN}
         {$ifdef VALREAL_128}
           if ( w.fh and ( C_MANT2_INTEGER_H shr 1 ) = 0 ) then
         {$else}
           if ( w.f and ( C_MANT2_INTEGER shr 1 ) = 0 ) then
         {$endif}
               return_special( str, n, C_STR_SNAN, min_width )
           else
       {$endif GRISU1_F2A_NO_SNAN}
               return_special( str, n, C_STR_QNAN, min_width );
        end;
        exit;
    end;

    // Handle denormals
    if ( w.e <> 0 ) then
    begin
        // normal
    {$ifdef VALREAL_128}
        w.fh := w.fh or C_MANT2_INTEGER_H;
    {$else not VALREAL_128}
      {$ifndef VALREAL_80}
        w.f := w.f or C_MANT2_INTEGER;
      {$endif not VALREAL_80}
    {$endif VALREAL_*}
        n := C_DIY_FP_Q - C_FRAC2_BITS - 1;
    end
    else
    begin
        // denormal
    {$ifdef VALREAL_128}
        if ( w.fh = 0 ) then
            n := count_leading_zero( w.f ) + 64
        else
            n := count_leading_zero( w.fh );
    {$else not VALREAL_128}
      {$ifdef VALREAL_80}
        // also handle pseudo-denormals
        n := count_leading_zero( w.f ) + 32;
      {$else VALREAL_32 | VALREAL_64}
        n := count_leading_zero( w.f );
      {$endif VALREAL_*}
    {$endif VALREAL_*}
        inc( w.e );
    end;

    // Final normalization
  {$if defined(VALREAL_32) or defined(VALREAL_64)}
    w.f := w.f shl n;
  {$else VALREAL_80 | VALREAL_128}
    diy_util_shl( w.fh, w.f, n );
  {$endif VALREAL_*}
    dec( w.e, C_EXP2_BIAS + n + C_FRAC2_BITS );

    //
    // 1. Find the normalized "c_mk = f_c * 2^e_c" such that "alpha <= e_c + e_w + q <= gamma"
    // 2. Define "V = D * 10^k": multiply the input number by "c_mk", do not normalize to land into [ alpha .. gamma ]
    // 3. Generate digits ( n_digits_need + "round" )
    //

    if ( C_GRISU_ALPHA <= w.e ) and ( w.e <= C_GRISU_GAMMA ) then
    begin
        // no scaling required
        D := w;
        c_mk.e10 := 0;
    end
    else
    begin
        mk := k_comp( w.e, C_GRISU_ALPHA{, C_GRISU_GAMMA} );
        diy_fp_cached_power10( mk, c_mk );
        // Let "D = f_D * 2^e_D := w (*) c_mk"
        if c_mk.e10 = 0 then
            D := w
        else
            D := diy_fp_multiply( w, c_mk.c, FALSE );
    end;

{$ifdef grisu1_debug}
    assert( ( C_GRISU_ALPHA <= D.e ) and ( D.e <= C_GRISU_GAMMA ) );
{$endif grisu1_debug}

    // Generate digits: integer part
{$ifdef grisu1_debug}
  {$ifdef VALREAL_80}
    assert( D.e <= 32 );
  {$else not VALREAL_80}
    assert( D.e <= 0 );
  {$endif VALREAL_*}
{$endif grisu1_debug}

  {$ifdef VALREAL_32}
    n_digits_have := gen_digits_32( buf, 0, D.f shr ( - D.e ) );
  {$endif VALREAL_32}
  
  {$ifdef VALREAL_64}
    n_digits_have := gen_digits_64( buf, 0, D.f shr ( - D.e ) );
  {$endif VALREAL_64}
  
  {$ifdef VALREAL_80}
    fl := D.f;
    fh := D.fh;
    if ( D.e > 0 ) then
    begin
        templ := ( qword(fh) shl D.e ) and qword($FFFFFFFF00000000);
        diy_util_shl( fh, fl, D.e );
        inc( templ, fh );
    end
    else
    begin
        diy_util_shr( fh, fl, - D.e );
        templ := fh;
    end;
  {$endif VALREAL_80}

  {$ifdef VALREAL_128}
    fl := D.f;
    templ := D.fh;
    diy_util_shr( templ, fl, - D.e );
  {$endif VALREAL_128}

  {$if defined(VALREAL_80) or defined(VALREAL_128)}
    if ( templ = 0 ) then
        n_digits_have := gen_digits_64( buf, 0, fl )
    else
    begin
        if not u128_div_u64_to_u64( templ, fl, qword(10000000000000000000), templ, fl ) then
{$ifdef grisu1_debug}
            assert( FALSE ) // never overflows by design
{$endif grisu1_debug};
        n_digits_have := gen_digits_64( buf, 0, templ );
        inc( n_digits_have, gen_digits_64( buf, n_digits_have, fl, n_digits_have > 0 ) );
    end;
  {$endif VALREAL_80 | VALREAL_128}

    dot_pos := n_digits_have;

    // Generate digits: fractional part
    f := 0; // "sticky" digit
    if ( D.e < 0 ) then
    repeat
        // MOD by ONE
        one_e := D.e;
      {$ifdef VALREAL_32}
        one_mask := dword( 1 ) shl ( - D.e ) - 1;
        f := D.f and one_mask;
      {$endif VALREAL_32}
      {$ifdef VALREAL_64}
        one_maskl := qword( 1 ) shl ( - D.e ) - 1;
        fl := D.f and one_maskl;
      {$endif VALREAL_64}
      {$if defined(VALREAL_80) or defined(VALREAL_128)}
        make_frac_mask( one_maskh, one_maskl, D.e );
        fl := D.f and one_maskl;
        fh := D.fh and one_maskh;

        // 128/96-bit loop
        while ( one_e < -61 ) and ( n_digits_have < n_digits_need + 1 ) and ( fl or fh <> 0 ) do
        begin
            // f := f * 5;
            templ := fl;
            temph := fh;
            diy_util_shl( fh, fl, 2 );
            diy_util_add( fh, fl, temph, templ );
            // one := one / 2
            diy_util_shr( one_maskh, one_maskl, 1 );
            inc( one_e );
            // DIV by one
            templ := fl;
            temph := fh;
            diy_util_shr( temph, templ, - one_e );
            buf[ n_digits_have ] := lo(templ);
            // MOD by one
            fl := fl and one_maskl;
            fh := fh and one_maskh;
            // next
            inc( n_digits_have );
        end;
        if ( n_digits_have >= n_digits_need + 1 ) then
        begin
            // only "sticky" digit remains
            f := ord( fl or fh <> 0 );
            break;
        end;
      {$endif VALREAL_80 | VALREAL_128}

      {$ifndef VALREAL_32}
        // 64-bit loop
        while ( one_e < -29 ) and ( n_digits_have < n_digits_need + 1 ) and ( fl <> 0 ) do
        begin
            // f := f * 5;
            inc( fl, fl shl 2 );
            // one := one / 2
            one_maskl := one_maskl shr 1;
            inc( one_e );
            // DIV by one
            buf[ n_digits_have ] := fl shr ( - one_e );
            // MOD by one
            fl := fl and one_maskl;
            // next
            inc( n_digits_have );
        end;
        if ( n_digits_have >= n_digits_need + 1 ) then
        begin
            // only "sticky" digit remains
            f := ord( fl <> 0 );
            break;
        end;
        one_mask := lo(one_maskl);
        f := lo(fl);
      {$endif not VALREAL_32}

        // 32-bit loop
        while ( n_digits_have < n_digits_need + 1 ) and ( f <> 0 ) do
        begin
            // f := f * 5;
            inc( f, f shl 2 );
            // one := one / 2
            one_mask := one_mask shr 1;
            inc( one_e );
            // DIV by one
            buf[ n_digits_have ] := f shr ( - one_e );
            // MOD by one
            f := f and one_mask;
            // next
            inc( n_digits_have );
        end;

    until true;

    // Append "sticky" digit if any
    if ( f <> 0 ) and ( n_digits_have >= n_digits_need + 1 ) then
    begin
        // single "<>0" digit is enough
        n_digits_have := n_digits_need + 2;
        buf[ n_digits_need + 1 ] := 1;
    end;

    // Round to n_digits_need using "roundTiesToEven"
    if ( n_digits_have > n_digits_need ) then
        inc( dot_pos, round_digits( buf, n_digits_have, n_digits_need ) );

    // Generate output
    if ( frac_digits >= 0 ) then
        if try_return_fixed( str, minus, buf, n_digits_have, dot_pos - c_mk.e10, min_width, frac_digits ) then
            exit;
    if ( n_digits_have > n_digits_sci ) then
        inc( dot_pos, round_digits( buf, n_digits_have, n_digits_sci {$ifdef GRISU1_F2A_HALF_ROUNDUP}, false {$endif} ) );
    return_exponential( str, minus, buf, n_digits_have, n_digits_sci, dot_pos - c_mk.e10 - 1, n_digits_exp, min_width );

end;

(****************************************************************************)

{$ifndef fpc_softfpu_implementation}
procedure str_real_iso( len, f: longint; d: ValReal; real_type: treal_type; out s: string );
var
    i: integer;
begin
    str_real( len, f, d, real_type, s );
    for i := length( s ) downto 1 do
        if ( s[i] ='E' ) then
        begin
            s[i] := 'e';
            break; // only one "E" expected
        end;
end;
{$endif not fpc_softfpu_implementation}

(*==========================================================================*
 *                                                                          *
 *                              ASCII -> Float                              *
 *                                                                          *
 *==========================================================================*)

function val_real( const src: shortstring; out err_pos: ValSInt ): ValReal;

{$define VALREAL_PACK}
{$i flt_pack.inc}

{ NOTE: C_MAX_DIGITS_ACCEPTED should fit into unsigned integer which forms DIY_FP }
const
{$ifdef VALREAL_32}
    C_MAX_DIGITS_ACCEPTED = 9;
    C_EXP10_OVER = 100;
    C_DIY_FP_Q   = 32;
    C_FRAC2_BITS = 23;
    C_EXP2_BIAS  = 127;
{$endif VALREAL_32}
{$ifdef VALREAL_64}
    C_MAX_DIGITS_ACCEPTED = 19;
    C_EXP10_OVER = 1000;
    C_DIY_FP_Q   = 64;
    C_FRAC2_BITS = 52;
    C_EXP2_BIAS  = 1023;
{$endif VALREAL_64}
{$ifdef VALREAL_80}
    C_MAX_DIGITS_ACCEPTED = 28;
    C_EXP10_OVER = 10000;
    C_DIY_FP_Q   = 96;
    C_FRAC2_BITS = 63;
    C_EXP2_BIAS  = 16383;
{$endif VALREAL_80}
{$ifdef VALREAL_128}
    C_MAX_DIGITS_ACCEPTED = 38;
    C_EXP10_OVER = 10000;
    C_DIY_FP_Q   = 128;
    C_FRAC2_BITS = 112;
    C_EXP2_BIAS  = 16383;
{$endif VALREAL_128}

(****************************************************************************)
    // handy const
    C_EXP2_SPECIAL   = C_EXP2_BIAS * 2 + 1;
    C_DIY_SHR_TO_FLT = C_DIY_FP_Q - 1 - C_FRAC2_BITS;
    C_DIY_EXP_TO_FLT = C_DIY_FP_Q - 1 + C_EXP2_BIAS;
    C_DIY_ROUND_BIT  = 1 shl ( C_DIY_SHR_TO_FLT - 1 );
    C_DIY_ROUND_MASK = ( C_DIY_ROUND_BIT shl 2 ) - 1;
{$ifdef VALREAL_128}
    C_FLT_INT_BITh   = qword(1) shl ( C_FRAC2_BITS - 64 );
    C_FLT_FRAC_MASKh = C_FLT_INT_BITh - 1;
{$else not VALREAL_128}
  {$ifdef VALREAL_32}
    C_FLT_INT_BIT    = dword(1) shl C_FRAC2_BITS;
    C_FLT_FRAC_MASK  = C_FLT_INT_BIT - 1;
  {$else VALREAL_64 | VALREAL_80}
    C_FLT_INT_BIT    = qword(1) shl C_FRAC2_BITS;
    C_FLT_FRAC_MASK  = qword(C_FLT_INT_BIT) - 1;
  {$endif VALREAL_*}
{$endif VALREAL_*}

    // specials
{$ifdef VALREAL_32}
    C_FLT_MANT_INF  = dword($00000000);
  {$ifndef GRISU1_A2F_NO_SNAN}
    C_FLT_MANT_SNAN = dword($00200000);
  {$endif GRISU1_A2F_NO_SNAN}
    C_FLT_MANT_QNAN = dword($00400000);
{$endif VALREAL_32}
{$ifdef VALREAL_64}
    C_FLT_MANT_INF  = qword($0000000000000000);
  {$ifndef GRISU1_A2F_NO_SNAN}
    C_FLT_MANT_SNAN = qword($0004000000000000);
  {$endif GRISU1_A2F_NO_SNAN}
    C_FLT_MANT_QNAN = qword($0008000000000000);
{$endif VALREAL_64}
{$ifdef VALREAL_80}
    C_FLT_MANT_INF  = qword($8000000000000000);
  {$ifndef GRISU1_A2F_NO_SNAN}
    C_FLT_MANT_SNAN = qword($A000000000000000);
  {$endif GRISU1_A2F_NO_SNAN}
    C_FLT_MANT_QNAN = qword($C000000000000000);
{$endif VALREAL_80}
{$ifdef VALREAL_128}
    C_FLT_MANT_INFh  = qword($0000000000000000);
    C_FLT_MANT_INF   = qword($0000000000000000);
  {$ifndef GRISU1_A2F_NO_SNAN}
    C_FLT_MANT_SNANh = qword($0000400000000000);
    C_FLT_MANT_SNAN  = qword($0000000000000000);
  {$endif GRISU1_A2F_NO_SNAN}
    C_FLT_MANT_QNANh = qword($0000800000000000);
    C_FLT_MANT_QNAN  = qword($0000000000000000);
{$endif VALREAL_128}

(*-------------------------------------------------------
 | factor_10_inexact [local]
 |
 | Calculates an arbitrary normalized power of 10 required for final scaling.
 | The result of this calculation may be off by several ulp's from exact.
 |
 | Returns an overflow/underflow status:
 |    "<0": underflow
 |    "=0": ok
 |    ">0": overflow
 |
 *-------------------------------------------------------*)
function factor_10_inexact( const exp10: integer; out C: TDIY_FP_Power_of_10 ): integer;
const
{$ifdef VALREAL_32}
    factor: array [ 0 .. 7 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: $80000000; e: -31); e10:   0 ),
        ( c: ( f: $CCCCCCCD; e: -35); e10:  -1 ),
        ( c: ( f: $A3D70A3D; e: -38); e10:  -2 ),
        ( c: ( f: $83126E98; e: -41); e10:  -3 ),
        ( c: ( f: $D1B71759; e: -45); e10:  -4 ),
        ( c: ( f: $A7C5AC47; e: -48); e10:  -5 ),
        ( c: ( f: $8637BD06; e: -51); e10:  -6 ),
        ( c: ( f: $D6BF94D6; e: -55); e10:  -7 )
    );
{$endif VALREAL_32}
{$ifdef VALREAL_64}
    factor: array [ 0 .. 17 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: qword($8000000000000000); e:  -63); e10:   0 ),
        ( c: ( f: qword($CCCCCCCCCCCCCCCD); e:  -67); e10:  -1 ),
        ( c: ( f: qword($A3D70A3D70A3D70A); e:  -70); e10:  -2 ),
        ( c: ( f: qword($83126E978D4FDF3B); e:  -73); e10:  -3 ),
        ( c: ( f: qword($D1B71758E219652C); e:  -77); e10:  -4 ),
        ( c: ( f: qword($A7C5AC471B478423); e:  -80); e10:  -5 ),
        ( c: ( f: qword($8637BD05AF6C69B6); e:  -83); e10:  -6 ),
        ( c: ( f: qword($D6BF94D5E57A42BC); e:  -87); e10:  -7 ),
        ( c: ( f: qword($ABCC77118461CEFD); e:  -90); e10:  -8 ),
        ( c: ( f: qword($89705F4136B4A597); e:  -93); e10:  -9 ),
        ( c: ( f: qword($DBE6FECEBDEDD5BF); e:  -97); e10: -10 ),
        ( c: ( f: qword($AFEBFF0BCB24AAFF); e: -100); e10: -11 ),
        ( c: ( f: qword($8CBCCC096F5088CC); e: -103); e10: -12 ),
        ( c: ( f: qword($E12E13424BB40E13); e: -107); e10: -13 ),
        ( c: ( f: qword($B424DC35095CD80F); e: -110); e10: -14 ),
        ( c: ( f: qword($901D7CF73AB0ACD9); e: -113); e10: -15 ),
        ( c: ( f: qword($E69594BEC44DE15B); e: -117); e10: -16 ),
        ( c: ( f: qword($B877AA3236A4B449); e: -120); e10: -17 )
    );
{$endif VALREAL_64}
{$ifdef VALREAL_80}
    factor: array [ 0 .. 36 ] of TDIY_FP_Power_of_10 = (
        ( c: ( f: qword($0000000000000000); fh: dword($80000000); e:  -95 ); e10:   0 ),
        ( c: ( f: qword($CCCCCCCCCCCCCCCD); fh: dword($CCCCCCCC); e:  -99 ); e10:  -1 ),
        ( c: ( f: qword($70A3D70A3D70A3D7); fh: dword($A3D70A3D); e: -102 ); e10:  -2 ),
        ( c: ( f: qword($8D4FDF3B645A1CAC); fh: dword($83126E97); e: -105 ); e10:  -3 ),
        ( c: ( f: qword($E219652BD3C36113); fh: dword($D1B71758); e: -109 ); e10:  -4 ),
        ( c: ( f: qword($1B4784230FCF80DC); fh: dword($A7C5AC47); e: -112 ); e10:  -5 ),
        ( c: ( f: qword($AF6C69B5A63F9A4A); fh: dword($8637BD05); e: -115 ); e10:  -6 ),
        ( c: ( f: qword($E57A42BC3D329076); fh: dword($D6BF94D5); e: -119 ); e10:  -7 ),
        ( c: ( f: qword($8461CEFCFDC20D2B); fh: dword($ABCC7711); e: -122 ); e10:  -8 ),
        ( c: ( f: qword($36B4A59731680A89); fh: dword($89705F41); e: -125 ); e10:  -9 ),
        ( c: ( f: qword($BDEDD5BEB573440E); fh: dword($DBE6FECE); e: -129 ); e10: -10 ),
        ( c: ( f: qword($CB24AAFEF78F69A5); fh: dword($AFEBFF0B); e: -132 ); e10: -11 ),
        ( c: ( f: qword($6F5088CBF93F87B7); fh: dword($8CBCCC09); e: -135 ); e10: -12 ),
        ( c: ( f: qword($4BB40E132865A5F2); fh: dword($E12E1342); e: -139 ); e10: -13 ),
        ( c: ( f: qword($095CD80F538484C2); fh: dword($B424DC35); e: -142 ); e10: -14 ),
        ( c: ( f: qword($3AB0ACD90F9D3701); fh: dword($901D7CF7); e: -145 ); e10: -15 ),
        ( c: ( f: qword($C44DE15B4C2EBE68); fh: dword($E69594BE); e: -149 ); e10: -16 ),
        ( c: ( f: qword($36A4B44909BEFEBA); fh: dword($B877AA32); e: -152 ); e10: -17 ),
        ( c: ( f: qword($921D5D073AFF322E); fh: dword($9392EE8E); e: -155 ); e10: -18 ),
        ( c: ( f: qword($B69561A52B31E9E4); fh: dword($EC1E4A7D); e: -159 ); e10: -19 ),
        ( c: ( f: qword($92111AEA88F4BB1D); fh: dword($BCE50864); e: -162 ); e10: -20 ),
        ( c: ( f: qword($74DA7BEED3F6FC17); fh: dword($971DA050); e: -165 ); e10: -21 ),
        ( c: ( f: qword($BAF72CB15324C68B); fh: dword($F1C90080); e: -169 ); e10: -22 ),
        ( c: ( f: qword($95928A2775B7053C); fh: dword($C16D9A00); e: -172 ); e10: -23 ),
        ( c: ( f: qword($44753B52C4926A96); fh: dword($9ABE14CD); e: -175 ); e10: -24 ),
        ( c: ( f: qword($D3EEC5513A83DDBE); fh: dword($F79687AE); e: -179 ); e10: -25 ),
        ( c: ( f: qword($76589DDA95364AFE); fh: dword($C6120625); e: -182 ); e10: -26 ),
        ( c: ( f: qword($91E07E48775EA265); fh: dword($9E74D1B7); e: -185 ); e10: -27 ),
        ( c: ( f: qword($8300CA0D8BCA9D6E); fh: dword($FD87B5F2); e: -189 ); e10: -28 ),
        ( c: ( f: qword($359A3B3E096EE458); fh: dword($CAD2F7F5); e: -192 ); e10: -29 ),
        ( c: ( f: qword($5E14FC31A125837A); fh: dword($A2425FF7); e: -195 ); e10: -30 ),
        ( c: ( f: qword($4B43FCF480EACF95); fh: dword($81CEB32C); e: -198 ); e10: -31 ),
        ( c: ( f: qword($453994BA67DE18EE); fh: dword($CFB11EAD); e: -202 ); e10: -32 ),
        ( c: ( f: qword($D0FADD61ECB1AD8B); fh: dword($A6274BBD); e: -205 ); e10: -33 ),
        ( c: ( f: qword($DA624AB4BD5AF13C); fh: dword($84EC3C97); e: -208 ); e10: -34 ),
        ( c: ( f: qword($C3D07787955E4EC6); fh: dword($D4AD2DBF); e: -212 ); e10: -35 ),
        ( c: ( f: qword($697392D2DDE50BD2); fh: dword($AA242499); e: -215 ); e10: -36 )
    );
{$endif VALREAL_80}
{$ifdef VALREAL_128}
    factor: array [ 0 .. 36 ] of TDIY_FP_Power_of_10 = (
        ( c: ( fh: qword($8000000000000000); f: qword($0000000000000000); e: -127 ); e10:   0 ),
        ( c: ( fh: qword($CCCCCCCCCCCCCCCC); f: qword($CCCCCCCCCCCCCCCD); e: -131 ); e10:  -1 ),
        ( c: ( fh: qword($A3D70A3D70A3D70A); f: qword($3D70A3D70A3D70A4); e: -134 ); e10:  -2 ),
        ( c: ( fh: qword($83126E978D4FDF3B); f: qword($645A1CAC083126E9); e: -137 ); e10:  -3 ),
        ( c: ( fh: qword($D1B71758E219652B); f: qword($D3C36113404EA4A9); e: -141 ); e10:  -4 ),
        ( c: ( fh: qword($A7C5AC471B478423); f: qword($0FCF80DC33721D54); e: -144 ); e10:  -5 ),
        ( c: ( fh: qword($8637BD05AF6C69B5); f: qword($A63F9A49C2C1B110); e: -147 ); e10:  -6 ),
        ( c: ( fh: qword($D6BF94D5E57A42BC); f: qword($3D32907604691B4D); e: -151 ); e10:  -7 ),
        ( c: ( fh: qword($ABCC77118461CEFC); f: qword($FDC20D2B36BA7C3D); e: -154 ); e10:  -8 ),
        ( c: ( fh: qword($89705F4136B4A597); f: qword($31680A88F8953031); e: -157 ); e10:  -9 ),
        ( c: ( fh: qword($DBE6FECEBDEDD5BE); f: qword($B573440E5A884D1B); e: -161 ); e10: -10 ),
        ( c: ( fh: qword($AFEBFF0BCB24AAFE); f: qword($F78F69A51539D749); e: -164 ); e10: -11 ),
        ( c: ( fh: qword($8CBCCC096F5088CB); f: qword($F93F87B7442E45D4); e: -167 ); e10: -12 ),
        ( c: ( fh: qword($E12E13424BB40E13); f: qword($2865A5F206B06FBA); e: -171 ); e10: -13 ),
        ( c: ( fh: qword($B424DC35095CD80F); f: qword($538484C19EF38C94); e: -174 ); e10: -14 ),
        ( c: ( fh: qword($901D7CF73AB0ACD9); f: qword($0F9D37014BF60A10); e: -177 ); e10: -15 ),
        ( c: ( fh: qword($E69594BEC44DE15B); f: qword($4C2EBE687989A9B4); e: -181 ); e10: -16 ),
        ( c: ( fh: qword($B877AA3236A4B449); f: qword($09BEFEB9FAD487C3); e: -184 ); e10: -17 ),
        ( c: ( fh: qword($9392EE8E921D5D07); f: qword($3AFF322E62439FCF); e: -187 ); e10: -18 ),
        ( c: ( fh: qword($EC1E4A7DB69561A5); f: qword($2B31E9E3D06C32E5); e: -191 ); e10: -19 ),
        ( c: ( fh: qword($BCE5086492111AEA); f: qword($88F4BB1CA6BCF584); e: -194 ); e10: -20 ),
        ( c: ( fh: qword($971DA05074DA7BEE); f: qword($D3F6FC16EBCA5E03); e: -197 ); e10: -21 ),
        ( c: ( fh: qword($F1C90080BAF72CB1); f: qword($5324C68B12DD6338); e: -201 ); e10: -22 ),
        ( c: ( fh: qword($C16D9A0095928A27); f: qword($75B7053C0F178294); e: -204 ); e10: -23 ),
        ( c: ( fh: qword($9ABE14CD44753B52); f: qword($C4926A9672793543); e: -207 ); e10: -24 ),
        ( c: ( fh: qword($F79687AED3EEC551); f: qword($3A83DDBD83F52205); e: -211 ); e10: -25 ),
        ( c: ( fh: qword($C612062576589DDA); f: qword($95364AFE032A819D); e: -214 ); e10: -26 ),
        ( c: ( fh: qword($9E74D1B791E07E48); f: qword($775EA264CF55347E); e: -217 ); e10: -27 ),
        ( c: ( fh: qword($FD87B5F28300CA0D); f: qword($8BCA9D6E188853FC); e: -221 ); e10: -28 ),
        ( c: ( fh: qword($CAD2F7F5359A3B3E); f: qword($096EE45813A04330); e: -224 ); e10: -29 ),
        ( c: ( fh: qword($A2425FF75E14FC31); f: qword($A1258379A94D028D); e: -227 ); e10: -30 ),
        ( c: ( fh: qword($81CEB32C4B43FCF4); f: qword($80EACF948770CED7); e: -230 ); e10: -31 ),
        ( c: ( fh: qword($CFB11EAD453994BA); f: qword($67DE18EDA5814AF2); e: -234 ); e10: -32 ),
        ( c: ( fh: qword($A6274BBDD0FADD61); f: qword($ECB1AD8AEACDD58E); e: -237 ); e10: -33 ),
        ( c: ( fh: qword($84EC3C97DA624AB4); f: qword($BD5AF13BEF0B113F); e: -240 ); e10: -34 ),
        ( c: ( fh: qword($D4AD2DBFC3D07787); f: qword($955E4EC64B44E864); e: -244 ); e10: -35 ),
        ( c: ( fh: qword($AA242499697392D2); f: qword($DDE50BD1D5D0B9EA); e: -247 ); e10: -36 )
    );
{$endif VALREAL_128}
var
    i: integer;
    a, b: TDIY_FP_Power_of_10;
begin
    diy_fp_cached_power10( exp10, a );
    i := a.e10 - exp10;
    if ( i < 0 ) then
    begin
        factor_10_inexact := 1; // overflow
        exit;
    end;
    if ( i > high( factor ) ) then
    begin
        factor_10_inexact := -1; // underflow
        exit;
    end;
    b := factor[i];
{$ifdef grisu1_debug}
    assert( exp10 =  a.e10 + b.e10 );
{$endif grisu1_debug}
    if ( b.e10 = 0 ) then
       C := a
    else
    if ( a.e10 = 0 ) then
       C := b
    else
    begin
        C.c := diy_fp_multiply( a.c, b.c, TRUE );
        c.e10 := exp10;
    end;
    factor_10_inexact := 0; // no error
end;

(*-------------------------------------------------------
 | add_digit [local]
 |
 | This helper routine performs next digit addition:
 |     X := X * 10 + digit
 |
 *-------------------------------------------------------*)
{$ifdef VALREAL_80}
procedure add_digit( var h: dword; var l: qword; digit: byte ); {$ifdef grisu1_inline}inline;{$endif}
var
    temp1, temp2: qword;
begin
    //
    temp1 := lo(l);
    inc( temp1, ( temp1 shl 3 ) + temp1 + digit );
    //
    temp2 := h;
    temp2 := ( temp2 shl 32 ) + hi(l);
    inc( temp2, ( temp2 shl 3 ) + temp2 + hi(temp1) );
    //
    h := hi(temp2);
    l := ( temp2 shl 32 ) + lo(temp1);
    //
end;
{$endif VALREAL_80}
{$ifdef VALREAL_128}
procedure add_digit( var h, l: qword; digit: byte ); {$ifdef grisu1_inline}inline;{$endif}
var
    templ, temph, temp1, temp2: qword;
begin
    templ := l;
    temph := h;
    diy_util_shl( temph, templ, 3 );
    //
    temp1 := lo(l);
    inc( temp1, lo(templ) + temp1 + digit );
    //
    temp2 := hi(l);
    inc( temp2, hi(templ) + temp2 + hi(temp1) );
    //
    inc( h, temph + h + hi(temp2) );
    l := ( temp2 shl 32 ) + lo(temp1);
    //
end;
{$endif VALREAL_128}

(*-------------------------------------------------------
 | count_leading_zero [local] <<<duplicate>>>
 |
 | Counts number of 0-bits at most significant bit position.
 |
 *-------------------------------------------------------*)
{$if defined(VALREAL_32) or defined(VALREAL_80)}
function count_leading_zero( const X: dword ): integer; {$ifdef grisu1_inline}inline;{$endif}
begin
    count_leading_zero := 31 - BSRdword( X );
end;
{$endif VALREAL_32 | VALREAL_80}
{$ifndef VALREAL_32}
function count_leading_zero( const X: qword ): integer; {$ifdef grisu1_inline}inline;{$endif}
begin
    count_leading_zero := 63 - BSRqword( X );
end;
{$endif not VALREAL_32}

(*-------------------------------------------------------
 | match_special [local]
 |
 | Routine compares source string tail with the string representing
 | one of special values: Inf | QNaN | SNaN
 |
 *-------------------------------------------------------*)
function match_special( src_pos: integer; const src, spec: shortstring ): boolean;
var
    sl, xl: integer;
begin
    match_special := false;
    xl := length( src );
    sl := length( spec );
    if ( sl <> xl - src_pos + 1 ) then
        exit;
{$ifdef grisu1_debug}
    assert( sl > 0 );
{$endif grisu1_debug}
    repeat
        if ( UpCase( src[xl] ) <> UpCase( spec[sl] ) ) then
            exit;
        dec( xl );
        dec( sl );
    until sl <= 0;
    match_special := true;
end;

(****************************************************************************)
var
    a: char;
    mantissa, bit_round, bit_round_mask: {$ifdef VALREAL_32} dword {$else} qword {$endif};
{$ifdef VALREAL_80}
    mantissa_h: dword;
{$endif}
{$ifdef VALREAL_128}
    mantissa_h, bit_round_h, bit_round_mask_h: qword;
{$endif}
    dig_num, exp10, overflow, n, src_pos, src_len: integer;
    exp_read, exp_temp: longint;
    b, dig_round, dig_sticky: byte;
    minus, exp_minus, special: boolean;
    C: TDIY_FP_Power_of_10;
    D: TDIY_FP;

begin

    err_pos := 1;
    src_pos := 1;
    src_len := length(src);

    // Pre-initialize result
{$ifdef GRISU1_A2F_ERROR_RET0}
    pack_float( val_real, false, 0, {$ifdef VALREAL_128} 0, {$endif} 0 );
{$else}
  {-ifdef GRISU1_A2F_NO_SNAN}
    // "real indefinite"
    pack_float( val_real, true, C_EXP2_SPECIAL, {$ifdef VALREAL_128} C_FLT_MANT_QNANh, {$endif} C_FLT_MANT_QNAN );
(*{-else}
    // SNaN is preferable for catching uninitialized variables access, but may cause troubles with implicit float type conversions
    pack_float( val_real, false, C_EXP2_SPECIAL, {$ifdef VALREAL_128} C_FLT_MANT_SNANh, {$endif} C_FLT_MANT_SNAN );
  {-endif}*)
{$endif}

    // search for a sign skipping leading spaces
    minus := false;
    while ( src_pos <= src_len ) do
    begin
        a := src[src_pos];
        case a of
        '+':
            begin
                inc( src_pos );
                break;
            end;
        '-':
            begin
                minus := true;
                inc( src_pos );
                break;
            end;
        else
            if a <> ' ' then
                break;
        end;
        inc( src_pos );
    end;
    if ( src_pos > src_len ) then
    begin
        // syntax: nothing to evaluate
        err_pos := src_pos;
        exit;
    end;

    // handle specials
    case src[src_pos] of
       '0' .. '9', '.', 'E', 'e': special := false;
    else
        special := true;
    end;
    if special then
    begin
        mantissa := C_FLT_MANT_INF;
{$ifdef VALREAL_128}
        mantissa_h := C_FLT_MANT_INFh;
{$endif}
        if not match_special( src_pos, src, C_STR_INF ) then
        begin
          {$ifndef GRISU1_A2F_NO_SNAN}
            if match_special( src_pos, src, C_STR_SNAN ) then
            begin
                mantissa := C_FLT_MANT_SNAN;
{$ifdef VALREAL_128}
                mantissa_h := C_FLT_MANT_SNANh;
{$endif}
            end
            else
          {$endif GRISU1_A2F_NO_SNAN}
            if match_special( src_pos, src, C_STR_QNAN ) then
            begin
              {$ifdef GRISU1_A2F_QNAN_REAL_INDEFINITE}
                minus := TRUE;
              {$endif}
                mantissa := C_FLT_MANT_QNAN;
{$ifdef VALREAL_128}
                mantissa_h := C_FLT_MANT_QNANh;
{$endif}
            end
            else
                special := false;
        end;
        if special then
        begin
            pack_float( val_real, minus, C_EXP2_SPECIAL, {$ifdef VALREAL_128} mantissa_h, {$endif} mantissa );
            src_pos := 0;
        end;
        err_pos := src_pos;
        exit;
    end;

    // consume mantissa digits skipping leading zeroes
    // empty mantissa ("0.E#", ".0E#", ".E#", "E#") is allowed at least in D5
    mantissa := 0;
{$if defined(VALREAL_80) or defined(VALREAL_128)}
    mantissa_h := 0;
{$endif VALREAL_80 | VALREAL_128}
    dig_num := 0;
    exp10 := 0;
    dig_round := 0;
    dig_sticky := 0;

    // leading zero loop
    while ( src_pos <= src_len ) and ( src[src_pos] = '0' ) do
        inc( src_pos );

    // integer part loop
    while ( src_pos <= src_len ) do
    begin
        a := src[src_pos];
        if ( a < '0' ) or ( a > '9' ) then
            break;
        b := ord(a) - ord('0');
        if ( dig_num < C_MAX_DIGITS_ACCEPTED ) then
            // normal digit
{$if defined(VALREAL_32) or defined(VALREAL_64)}
            inc( mantissa, ( mantissa shl 3 ) + mantissa + b )
{$else VALREAL_80 | VALREAL_128}
            add_digit( mantissa_h, mantissa, b )
{$endif VALREAL_*}
        else
        begin
            // over-required digits: use them for rounding later
            if ( dig_num = C_MAX_DIGITS_ACCEPTED ) then
                dig_round := b // main digit for rounding
            else
                dig_sticky := dig_sticky or b; // just "<>0" to judge "round to even" case later
            inc( exp10 ); // move [yet virtual] dot to the right
        end;
        inc( dig_num );
        inc( src_pos );
    end;

    // fraction part
    if ( src_pos <= src_len ) and ( src[src_pos] = '.' ) then
    begin
        // skip dot
        inc( src_pos );
        // leading zero loop, if integer part was 0
        if dig_num = 0 then
            while ( src_pos <= src_len ) and ( src[src_pos] = '0' ) do
            begin
                dec( exp10 ); // move the dot to the left
                inc( src_pos );
            end;
        // fraction part loop
        while ( src_pos <= src_len ) do
        begin
            a := src[src_pos];
            if ( a < '0' ) or ( a > '9' ) then
                break;
            b := ord(a) - ord('0');
            if ( dig_num < C_MAX_DIGITS_ACCEPTED ) then
            begin
                // normal digit
{$if defined(VALREAL_32) or defined(VALREAL_64)}
                inc( mantissa, ( mantissa shl 3 ) + mantissa + b );
{$else VALREAL_80 | VALREAL_128}
                add_digit( mantissa_h, mantissa, b );
{$endif VALREAL_*}
                dec( exp10 ); // move the dot to the left
            end
            else
            begin
                // over-required digits: use them for rounding later
                if ( dig_num = C_MAX_DIGITS_ACCEPTED ) then
                    dig_round := b // main digit for rounding
                else
                    dig_sticky := dig_sticky or b; // just "<>0" to judge "round to even" case later
            end;
            inc( dig_num );
            inc( src_pos );
        end;
    end;

    // round digits
{$ifndef GRISU1_A2F_HALF_ROUNDUP}
    if ( dig_round = 5 ) and ( dig_sticky = 0 ) and ( mantissa and 1 = 0 ) then
        // need to "round to even", and already even..
        dec( dig_round ); // ..so force no round-up
{$endif not GRISU1_A2F_HALF_ROUNDUP}
    if ( dig_round >= 5 ) then
    begin
        // round-up
        inc( mantissa );
{$if defined(VALREAL_80) or defined(VALREAL_128)}
        if ( mantissa = 0 ) then
            inc( mantissa_h );
{$endif VALREAL_*}
    end;

    // consume exponent digits
    exp_read := 0;
    if ( src_pos <= src_len ) then
    begin
        exp_minus := false;
        case src[src_pos] of
        'e', 'E':;
        else
            // syntax: "E" expected
            err_pos := src_pos;
            exit;
        end;
        inc( src_pos );
        if ( src_pos > src_len ) then
        begin
            // syntax: empty exponent
            err_pos := src_pos;
            exit;
        end;
        case src[src_pos] of
        '+':
            inc( src_pos );
        '-':
            begin
                exp_minus := true;
                inc( src_pos );
            end;
        end;
        while ( src_pos <= src_len ) do
        begin
            a := src[src_pos];
            if ( a < '0' ) or ( a > '9' ) then
            begin
                // syntax: bad digit
                err_pos := src_pos;
                exit;
            end;
            if ( exp_read < 100000 ) then
                inc( exp_read, ( exp_read shl 3 ) + exp_read + ord(a) - ord('0') );
         // else just syntax check
            inc( src_pos );
        end;
        if exp_minus then
            exp_read := - exp_read;
    end;
    exp_temp := exp_read + exp10;
    if ( exp_read >= 100000 ) or ( exp_temp >= C_EXP10_OVER ) then
        exp10 := C_EXP10_OVER
    else
    if ( exp_read <= - 100000 ) or ( exp_temp <= - C_EXP10_OVER ) then
        exp10 := - C_EXP10_OVER
    else
        exp10 := exp_temp;

    // nothing should remain in the "src" here
    if ( src_pos <= src_len ) then
    begin
        err_pos := src_pos;
        exit;
    end;

    // zero [or negative exponent overflow]
    if ( mantissa {$if defined(VALREAL_80) or defined(VALREAL_128)} or mantissa_h {$endif} = 0 )
    or ( exp10 <= - C_EXP10_OVER ) then
    begin
        pack_float( val_real, minus, 0, {$ifdef VALREAL_128} 0, {$endif} 0 ); // +0|-0
        err_pos := 0;
        exit;
    end;

    if ( exp10 >= C_EXP10_OVER ) then
        // exponent overflowed -> return Inf
        overflow := 1
    else
    begin
        // make DIY_FP
{$if defined(VALREAL_32) or defined(VALREAL_64)}
        n := count_leading_zero( mantissa );
        D.f := mantissa shl n;
{$else VALREAL_80 | VALREAL_128}
        if ( mantissa_h = 0 ) then
            n := count_leading_zero( mantissa ) + sizeof( mantissa_h ) * 8
        else
            n := count_leading_zero( mantissa_h );
        D.f := mantissa;
        D.fh := mantissa_h;
        diy_util_shl( D.fh, D.f, n );
{$endif VALREAL_*}
        D.e := - n;
        // get factor
        overflow := factor_10_inexact( exp10, C ); // <>0 -> over/underflow
    end;

    if ( overflow = 0 ) then
    begin
        // multiply
        if ( C.e10 <> 0 ) then
            // C <> 1
            D := diy_fp_multiply( D, C.c, TRUE );
        // calculate round increment
        if ( D.f and C_DIY_ROUND_MASK = C_DIY_ROUND_BIT ) then
            // round to even and already even
            b := 0
        else
            b := ord( D.f and C_DIY_ROUND_BIT <> 0 );
        // shift and round
{$if defined(VALREAL_32) or defined(VALREAL_64)}
        D.f := ( D.f shr C_DIY_SHR_TO_FLT ) + b;
        // handle round overflow
        if ( D.f and ( C_FLT_INT_BIT shl 1 ) <> 0 ) then
        begin
            D.f := D.f shr 1;
            inc( D.e );
        end;
{$else VALREAL_80 or VALREAL_128}
        diy_util_shr( D.fh, D.f, C_DIY_SHR_TO_FLT );
        if ( b <> 0 ) then
            diy_util_add( D.fh, D.f, 0, b );
        // handle round overflow
        if ( D.fh {$ifdef VALREAL_128} and ( C_FLT_INT_BITh shl 1 ) {$endif} <> 0 ) then
        begin
            diy_util_shr( D.fh, D.f, 1 );
            inc( D.e );
        end;
    {$if defined(grisu1_debug) and defined(VALREAL_80)}
        assert( D.fh = 0 );
    {$endif grisu1_debug}
{$endif VALREAL_*}
        // calculate exponent
        D.e := D.e + C_DIY_EXP_TO_FLT;
        if ( D.e >= C_EXP2_SPECIAL ) then
          ///////////////////
          //
          // overflow
          //
          ///////////////////
            overflow := 1
        else
        if ( D.e < - C_FRAC2_BITS ) then
          ///////////////////
          //
          // underflow
          //
          ///////////////////
            overflow := -1
        else
        if ( D.e <= 0 ) then
        begin
          ///////////////////
          //
          // denormal (and also an extreme case of "D.e=-C_FRAC2_BITS", where
          // rounding may produce either the lowest denormal or underflow)
          //
          ///////////////////
            n := 1 - D.e; // SHR amount
            // round bit
{$ifdef VALREAL_32}
            bit_round := dword(1) shl ( n - 1 );
{$endif VALREAL_32}
{$if defined(VALREAL_64) or defined(VALREAL_80)}
            bit_round := qword(1) shl ( n - 1 );
{$endif VALREAL_64 | VALREAL_80}
{$ifdef VALREAL_128}
            bit_round := 1;
            bit_round_h := 0;
            diy_util_shl( bit_round_h, bit_round, n - 1 );
            // mask for ulp and all least bits including the round one
            bit_round_mask := bit_round;
            bit_round_mask_h := bit_round_h;
            diy_util_shl( bit_round_mask_h, bit_round_mask, 2 );
            if ( bit_round_mask = 0 ) then
                dec( bit_round_mask_h );
            dec( bit_round_mask );
{$else not VALREAL_128}
            // mask for ulp and all least bits including the round one
            bit_round_mask := ( bit_round shl 2 ) - 1;
            // Note[floatx80]: works correctly despite the overflow is ignored in extreme case "D.e=-C_FRAC2_BITS"
{$endif VALREAL_*}
            // round increment
            if ( D.f and bit_round_mask = bit_round ) {$ifdef VALREAL_128} and ( D.fh and bit_round_mask_h = bit_round_h ) {$endif} then
                // round to even and already even -> no round-up
                b := 0
            else
                b := ord( ( D.f and bit_round ) {$ifdef VALREAL_128} or ( D.fh and bit_round_h ) {$endif} <> 0 );
            // shift and round
            if ( D.e = - C_FRAC2_BITS ) then
            begin
                // extreme case: rounding may result in either lowest denormal or zero
              {$ifdef VALREAL_128}
                D.fh := 0;
              {$endif VALREAL_128}
                D.f := b;
                if ( b = 0 ) then
                    overflow := -1; // underflow
            end
            else
          {$ifdef VALREAL_128}
            begin
                diy_util_shr( D.fh, D.f, n );
                if ( b <> 0 ) then
                    diy_util_add( D.fh, D.f, 0, b );
            end;
          {$else not VALREAL_128}
                D.f := ( D.f shr n ) + b;
          {$endif VALREAL_*}
            D.e := 0;
{$ifdef grisu1_debug}
          {$ifdef VALREAL_128}
            assert( ( D.f or D.fh <> 0 ) or ( overflow = -1 ) );
            assert( D.fh and not C_FLT_FRAC_MASKh = 0 );
          {$else not VALREAL_128}
            assert( ( D.f <> 0 ) or ( overflow = -1 ) );
            assert( D.f and not C_FLT_FRAC_MASK = 0 );
          {$endif VALREAL_*}
{$endif grisu1_debug}
        end
        else
        begin
          ///////////////////
          //
          // normal: 0 < D.e < C_EXP2_SPECIAL
          //
          ///////////////////
{$ifdef grisu1_debug}
          {$ifdef VALREAL_32}
            assert( D.f and not C_FLT_FRAC_MASK = C_FLT_INT_BIT );
          {$endif VALREAL_32}
          {$if defined(VALREAL_64) or defined(VALREAL_80)}
            assert( D.f and not qword(C_FLT_FRAC_MASK) = qword(C_FLT_INT_BIT) );
          {$endif VALREAL_64 | VALREAL_80}
          {$ifdef VALREAL_128}
            assert( D.fh and not C_FLT_FRAC_MASKh = C_FLT_INT_BITh );
          {$endif VALREAL_128}
{$endif grisu1_debug}
        {$ifndef VALREAL_80}
           // clear the implicit integer bit
          {$ifdef VALREAL_128}
            D.fh := D.fh and C_FLT_FRAC_MASKh;
          {$else not VALREAL_128}
            D.f := D.f and C_FLT_FRAC_MASK;
          {$endif VALREAL_*}
        {$endif not VALREAL_80}
        end;
    end;

    // final result
    if ( overflow < 0 ) then
    begin
        // underflow [+0|-0]
        pack_float( val_real, minus, 0, {$ifdef VALREAL_128} 0, {$endif} 0 );
    end
    else
    if ( overflow > 0 ) then
    begin
        // overflow [+Inf|-Inf]
        pack_float( val_real, minus, C_EXP2_SPECIAL, {$ifdef VALREAL_128} C_FLT_MANT_INFh, {$endif} C_FLT_MANT_INF );
    end
    else
    begin
        // no error
        pack_float( val_real, minus, D.e, {$ifdef VALREAL_128} D.fh, {$endif} D.f );
    end;
    err_pos := 0;

end;
