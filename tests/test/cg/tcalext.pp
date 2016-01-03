{****************************************************************}
{  CODE GENERATOR TEST PROGRAM                                   }
{****************************************************************}
{ NODE TESTED : secondcalln()                                    }
{****************************************************************}
{ PRE-REQUISITES: secondload()                                   }
{                 secondassign()                                 }
{                 secondcalln()                                  }
{                 secondadd()                                    }
{                 secondtypeconv()                               }
{****************************************************************}
{ DEFINES:                                                       }
{****************************************************************}
{ REMARKS: This tests a subset of the secondcalln() , it         }
{          verifies the usage of external cdecl                  }
{          modules compiled with C compilers.                    }
{****************************************************************}


{$ifndef USE_PASCAL_OBJECT}
{$MODE OBJFPC}
{$R+}
uses strings,ctypes;
{$L ctest.o}
{$endif USE_PASCAL_OBJECT}

{$ifdef FPC_HAS_TYPE_EXTENDED}
{$define test_longdouble}
{$endif}

{ Use C alignment of records }
{$PACKRECORDS C}
const
   RESULT_U8BIT = $55;
   RESULT_U16BIT = $500F;
   RESULT_U32BIT = $500F0000;
   RESULT_U64BIT = $1BCDABCD;
   RESULT_S16BIT = -12;
   RESULT_S32BIT = -120;
   RESULT_S64BIT = -12000;
   RESULT_FLOAT  = 14.54;
   RESULT_DOUBLE = 15.54;
   RESULT_LONGDOUBLE = 16.54;
   RESULT_PCHAR  = 'Hello world';

type
 _1byte_ = record
  u8 : byte;
 end;

 _3byte_ = record
  u8 : byte;
  u16 : word;
 end;

 _3byte_s = record
  u16 : word;
  w8 : byte;
 end;

 _5byte_ = record
  u8 : byte;
  u32 : cardinal;
 end;

_7byte_ = record
  u8: byte;
  s64: int64;
  u16: word;
end;
  byte_array = array [0..1] of byte;
  word_array = array [0..1] of word;
  cardinal_array = array [0..1] of cardinal;
  qword_array = array [0..1] of qword;
  smallint_array = array [0..1] of smallint;
  longint_array = array [0..1] of longint;
  int64_array = array [0..1] of int64;
  single_array = array [0..1] of single;
  double_array = array [0..1] of double;
  clongdouble_array = array [0..1] of clongdouble;


{ simple parameter passing }
procedure test_param_u8(x: byte); cdecl; external;
procedure test_param_u16(x : word); cdecl; external;
procedure test_param_u32(x: cardinal); cdecl; external;
procedure test_param_u64(x: qword); cdecl; external;
procedure test_param_s16(x : smallint); cdecl; external;
procedure test_param_s32(x: longint); cdecl; external;
procedure test_param_s64(x: int64); cdecl; external;
procedure test_param_float(x : single); cdecl; external;
procedure test_param_double(x: double); cdecl; external;
procedure test_param_longdouble(x: clongdouble); cdecl; external;
procedure test_param_var_u8(var x: byte); cdecl; external;

{ array parameter passing }
procedure test_array_param_u8(x: byte_array); cdecl; external;
procedure test_array_param_u16(x : word_array); cdecl; external;
procedure test_array_param_u32(x: cardinal_array); cdecl; external;
procedure test_array_param_u64(x: qword_array); cdecl; external;
procedure test_array_param_s16(x :smallint_array); cdecl; external;
procedure test_array_param_s32(x: longint_array); cdecl; external;
procedure test_array_param_s64(x: int64_array); cdecl; external;
procedure test_array_param_float(x : single_array); cdecl; external;
procedure test_array_param_double(x: double_array); cdecl; external;
procedure test_array_param_longdouble(x: clongdouble_array); cdecl; external;

{ mixed parameter passing }
procedure test_param_mixed_u16(z: byte; x : word; y :byte); cdecl; external;
procedure test_param_mixed_u32(z: byte; x: cardinal; y: byte); cdecl; external;
procedure test_param_mixed_s64(z: byte; x: int64; y: byte); cdecl; external;
procedure test_param_mixed_float(x: single; y: byte); cdecl; external;
procedure test_param_mixed_double(x: double; y: byte); cdecl; external;
procedure test_param_mixed_long_double(x: clongdouble; y: byte); cdecl; external;
procedure test_param_mixed_var_u8(var x: byte;y:byte); cdecl; external;
{ structure parameter testing }
procedure test_param_struct_tiny(buffer :   _1BYTE_); cdecl; external;
procedure test_param_struct_small(buffer :  _3BYTE_); cdecl; external;
procedure test_param_struct_small_s(buffer :  _3BYTE_S); cdecl; external;
procedure test_param_struct_medium(buffer : _5BYTE_); cdecl; external;
procedure test_param_struct_large(buffer :  _7BYTE_); cdecl; external;
{ mixed with structure parameter testing }
procedure test_param_mixed_struct_tiny(buffer :   _1BYTE_; y :byte); cdecl; external;
procedure test_param_mixed_struct_small(buffer :  _3BYTE_; y :byte); cdecl; external;
procedure test_param_mixed_struct_small_s(buffer :  _3BYTE_S; y :byte); cdecl; external;
procedure test_param_mixed_struct_medium(buffer : _5BYTE_; y :byte); cdecl; external;
procedure test_param_mixed_struct_large(buffer :  _7BYTE_; y :byte); cdecl; external;
{ function result value testing }
function test_function_u8: byte; cdecl; external;
function test_function_u16: word; cdecl; external;
function test_function_u32: cardinal; cdecl; external;
function test_function_u64: qword; cdecl; external;
function test_function_s16: smallint; cdecl; external;
function test_function_s32: longint; cdecl; external;
function test_function_s64: int64; cdecl; external;
function test_function_pchar: pchar; cdecl; external;
function test_function_float : single; cdecl; external;
function test_function_double : double; cdecl; external;
function test_function_longdouble: clongdouble; cdecl; external;
function test_function_tiny_struct : _1byte_; cdecl; external;
function test_function_small_struct : _3byte_; cdecl; external;
function test_function_small_struct_s : _3byte_s; cdecl; external;
function test_function_medium_struct : _5byte_; cdecl; external;
function test_function_struct : _7byte_; cdecl; external;






var
 global_u8bit : byte; cvar; external;
 global_u16bit : word; cvar; external;
 global_u32bit : cardinal; cvar;external;
 global_u64bit : qword; cvar; external;
 global_s16bit : smallint; cvar; external;
 global_s32bit : longint; cvar;external;
 global_s64bit : int64; cvar; external;
 global_float : single; cvar;external;
 global_double : double; cvar;external;
 global_long_double : clongdouble; cvar; external;
 value_u8bit : byte;
 value_s16bit : smallint;
 value_s32bit : longint;
 value_s64bit : int64;
 value_u16bit : word;
 value_u32bit : cardinal;
 value_u64bit : qword;
 value_float : single;
 value_double : double;
 value_long_double : extended;
 array_u8bit : array [0..1] of byte;
 array_s16bit : array [0..1] of smallint;
 array_s32bit : array [0..1] of longint;
 array_s64bit : array [0..1] of int64;
 array_u16bit : array [0..1] of word;
 array_u32bit : array [0..1] of cardinal;
 array_u64bit : array [0..1] of qword;
 array_float : array [0..1] of single;
 array_double : array [0..1] of double;
 array_long_double : array [0..1] of clongdouble;

 procedure clear_globals;
  begin
    global_u8bit := 0;
    global_u16bit := 0;
    global_u32bit := 0;
    global_u64bit := 0;
    global_s16bit := 0;
    global_s32bit := 0;
    global_s64bit := 0;
    global_float := 0.0;
    global_double := 0.0;
    global_long_double := 0.0;
   end;

 procedure clear_values;
  begin
    value_u8bit := 0;
    value_u16bit := 0;
    value_u32bit := 0;
    value_u64bit := 0;
    value_s16bit := 0;
    value_s32bit := 0;
    value_s64bit := 0;
    value_float := 0.0;
    value_double := 0.0;
    value_long_double := 0.0;
  end;

{ in sub procedure to detect stack corruption when exiting }
procedure dotest;
const
  has_errors : boolean = false;

  procedure fail;
   begin
     WriteLn('Failed!');
     has_errors:=true;
   end;


var failed : boolean;
    tinystruct : _1BYTE_;
    smallstruct : _3BYTE_;
    smallstruct_s : _3BYTE_S;
    mediumstruct : _5BYTE_;
    bigstruct : _7BYTE_;
    pc: pchar;
begin
  Write('External simple parameter testing...');
  failed := false;

  clear_values;
  clear_globals;

  value_u8bit := RESULT_U8BIT;
  test_param_u8(value_u8bit);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u16bit := RESULT_U16BIT;
  test_param_u16(value_u16bit);
  if global_u16bit <> RESULT_U16BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u32bit := RESULT_U32BIT;
  test_param_u32(value_u32bit);
  if global_u32bit <> RESULT_U32BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u64bit := RESULT_U64BIT;
  test_param_u64(value_u64bit);
  if global_u64bit <> RESULT_U64BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_s16bit := RESULT_S16BIT;
  test_param_s16(value_s16bit);
  if global_s16bit <> RESULT_S16BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_s32bit := RESULT_S32BIT;
  test_param_s32(value_s32bit);
  if global_s32bit <> RESULT_S32BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_s64bit := RESULT_S64BIT;
  test_param_s64(value_s64bit);
  if global_s64bit <> RESULT_S64BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_float := RESULT_FLOAT;
  test_param_float(value_float);
  if trunc(global_float) <> trunc(RESULT_FLOAT) then
    failed := true;

  clear_values;
  clear_globals;

  value_double := RESULT_DOUBLE;
  test_param_double(value_double);
  if trunc(global_double) <> trunc(RESULT_DOUBLE) then
    failed := true;

{$ifdef test_longdouble}
  clear_values;
  clear_globals;

  value_long_double := RESULT_LONGDOUBLE;
  test_param_longdouble(value_long_double);
  if Abs(RESULT_LONGDOUBLE - global_long_double) > 1E-15 then
    failed := true;
{$endif}

  { var parameter testing }
  clear_values;
  clear_globals;
  test_param_var_u8(value_u8bit);
  if value_u8bit <> RESULT_U8BIT then
     failed := true;

  If failed then
   fail
  else
    WriteLn('Passed!');

  Write('External array parameter testing...');
  failed := false;

  clear_values;
  clear_globals;

  array_u8bit[1] := RESULT_U8BIT;
  test_array_param_u8(array_u8bit);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  array_u16bit[1] := RESULT_U16BIT;
  test_array_param_u16(array_u16bit);
  if global_u16bit <> RESULT_U16BIT then
    failed := true;

  clear_values;
  clear_globals;

  array_u32bit[1] := RESULT_U32BIT;
  test_array_param_u32(array_u32bit);
  if global_u32bit <> RESULT_U32BIT then
    failed := true;

  clear_values;
  clear_globals;

  array_u64bit[1] := RESULT_U64BIT;
  test_array_param_u64(array_u64bit);
  if global_u64bit <> RESULT_U64BIT then
    failed := true;

  clear_values;
  clear_globals;

  array_s16bit[1] := RESULT_S16BIT;
  test_array_param_s16(array_s16bit);
  if global_s16bit <> RESULT_S16BIT then
    failed := true;

  clear_values;
  clear_globals;

  array_s32bit[1] := RESULT_S32BIT;
  test_array_param_s32(array_s32bit);
  if global_s32bit <> RESULT_S32BIT then
    failed := true;

  clear_values;
  clear_globals;

  array_s64bit[1] := RESULT_S64BIT;
  test_array_param_s64(array_s64bit);
  if global_s64bit <> RESULT_S64BIT then
    failed := true;

  clear_values;
  clear_globals;

  array_float[1] := RESULT_FLOAT;
  test_array_param_float(array_float);
  if trunc(global_float) <> trunc(RESULT_FLOAT) then
    failed := true;

  clear_values;
  clear_globals;

  array_double[1] := RESULT_DOUBLE;
  test_array_param_double(array_double);
  if trunc(global_double) <> trunc(RESULT_DOUBLE) then
    failed := true;

  clear_values;
  clear_globals;

{$ifdef test_longdouble}
  array_long_double[1] := RESULT_LONGDOUBLE;
  test_array_param_longdouble(array_long_double);
<<<<<<< HEAD
  if Abs(RESULT_LONGDOUBLE - global_long_double) > 1E-15 then
=======
  if trunc(global_long_double) <> trunc(RESULT_LONGDOUBLE) then
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
    failed := true;
{$endif test_longdouble}

  If failed then
   fail
  else
    WriteLn('Passed!');

  Write('External mixed parameter testing...');
  failed := false;

  clear_values;
  clear_globals;
  test_param_mixed_var_u8(value_u8bit,RESULT_U8BIT);
  if value_u8bit <> RESULT_U8BIT then
     failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u8bit := RESULT_U8BIT;
  value_u16bit := RESULT_U16BIT;
  test_param_mixed_u16(value_u8bit, value_u16bit, value_u8bit);
  if global_u16bit <> RESULT_U16BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u8bit := RESULT_U8BIT;
  value_u32bit := RESULT_U32BIT;
  test_param_mixed_u32(value_u8bit, value_u32bit, value_u8bit);
  if global_u32bit <> RESULT_U32BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u8bit := RESULT_U8BIT;
  value_s64bit := RESULT_S64BIT;
  test_param_mixed_s64(value_u8bit, value_s64bit, value_u8bit);
  if global_s64bit <> RESULT_S64BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u8bit := RESULT_U8BIT;
  value_float := RESULT_FLOAT;
  test_param_mixed_float(value_float, value_u8bit);
  if global_float <> value_float then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  If failed then
   fail
  else
    WriteLn('Passed!');

  Write('External mixed parameter testing with floating values...');
  failed := false;

  clear_values;
  clear_globals;

  value_u8bit := RESULT_U8BIT;
  value_double := RESULT_DOUBLE;
  test_param_mixed_double(value_double, value_u8bit);
  if global_double <> value_double then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

{$ifdef test_longdouble}
  value_u8bit := RESULT_U8BIT;
  value_long_double := RESULT_LONGDOUBLE;
  test_param_mixed_long_double(value_long_double, value_u8bit);
  if Abs(RESULT_LONGDOUBLE - global_long_double) > 1E-15 then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
{$endif FPC_HAS_TYPE_EXTENDED}
>>>>>>> graemeg/fixes_2_2
=======
{$endif FPC_HAS_TYPE_EXTENDED}
>>>>>>> origin/fixes_2_2

  If failed then
   fail
  else
    WriteLn('Passed!');

  Write('External struct parameter testing...');
  failed := false;

  clear_values;
  clear_globals;

  tinystruct.u8 := RESULT_U8BIT;
  test_param_struct_tiny(tinystruct);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  smallstruct.u8 := RESULT_U8BIT;
  smallstruct.u16 := RESULT_u16BIT;
  test_param_struct_small(smallstruct);
  if global_u16bit <> RESULT_U16BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  smallstruct_s.u16 := RESULT_U16BIT;
  smallstruct_s.w8 := RESULT_U8BIT;
  test_param_struct_small_s(smallstruct_s);
  if global_u16bit <> RESULT_U16BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  mediumstruct.u8 := RESULT_U8BIT;
  mediumstruct.u32 := RESULT_U32BIT;
  test_param_struct_medium(mediumstruct);
  if global_u32bit <> RESULT_U32BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;


  bigstruct.u8 := RESULT_U8BIT;
  bigstruct.u16 := RESULT_U16BIT;
  bigstruct.s64 := RESULT_S64BIT;
  test_param_struct_large(bigstruct);
  if global_s64bit <> RESULT_S64BIT then
    failed := true;
  if global_u16bit <> RESULT_U16BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  If failed then
   fail
  else
    WriteLn('Passed!');


  Write('External mixed struct/byte parameter testing...');
  failed := false;

  clear_values;
  clear_globals;

  test_param_mixed_struct_tiny(tinystruct,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  smallstruct.u16 := RESULT_u16BIT;
  test_param_mixed_struct_small(smallstruct,RESULT_U8BIT);
  if global_u16bit <> RESULT_U16BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  smallstruct_s.u16 := RESULT_U16BIT;
  test_param_mixed_struct_small_s(smallstruct_s,RESULT_U8BIT);
  if global_u16bit <> RESULT_U16BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  mediumstruct.u32 := RESULT_U32BIT;
  test_param_mixed_struct_medium(mediumstruct,RESULT_U8BIT);
  if global_u32bit <> RESULT_U32BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;


  bigstruct.u16 := RESULT_U16BIT;
  bigstruct.s64 := RESULT_S64BIT;
  test_param_mixed_struct_large(bigstruct,RESULT_U8BIT);
  if global_s64bit <> RESULT_S64BIT then
    failed := true;
  if global_u16bit <> RESULT_U16BIT then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  If failed then
   fail
  else
    WriteLn('Passed!');


  Write('Integer function result testing...');
  failed := false;

  clear_values;
  clear_globals;

  value_u8bit := test_function_u8;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u16bit := test_function_u16;
  if value_u16bit <> RESULT_U16BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u32bit := test_function_u32;
  if value_u32bit <> RESULT_U32BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_u64bit := test_function_u64;
  if value_u64bit <> RESULT_U64BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_s16bit := test_function_s16;
  if value_s16bit <> RESULT_S16BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_s32bit := test_function_s32;
  if value_s32bit <> RESULT_S32BIT then
    failed := true;

  clear_values;
  clear_globals;

  value_s64bit := test_function_s64;
  if value_s64bit <> RESULT_S64BIT then
    failed := true;

  clear_values;
  clear_globals;

  If failed then
   fail
  else
    WriteLn('Passed!');

  Write('pchar function result testing...');
  failed := false;
  
  { verify if the contents both strings are equal }
  pc := test_function_pchar;
  if strcomp(pc, RESULT_PCHAR) <> 0 then
    failed := true;

  clear_values;
  clear_globals;

  If failed then
   fail
  else
    WriteLn('Passed!');

  Write('Real function result testing...');
  failed := false;
  value_float := test_function_float;
  if trunc(value_float) <> trunc(RESULT_FLOAT) then
    failed := true;

  clear_values;
  clear_globals;

  value_double := test_function_double;
  if trunc(value_double) <> trunc(RESULT_DOUBLE) then
    failed := true;

{$ifdef test_longdouble}
  clear_values;
  clear_globals;

  value_long_double := test_function_longdouble;
  if Abs(RESULT_LONGDOUBLE - value_long_double) > 1E-15 then
    failed := true;
{$endif}

  clear_values;
  clear_globals;

  If failed then
   fail
  else
    WriteLn('Passed!');

  Write('Function result testing for struct...');
  failed := false;

  tinystruct := test_function_tiny_struct;
  if tinystruct.u8 <> RESULT_U8BIT then
    failed := true;

  smallstruct := test_function_small_struct;
  if smallstruct.u8 <> RESULT_U8BIT then
    failed := true;
  if smallstruct.u16 <> RESULT_U16BIT then
    failed := true;

  smallstruct_s := test_function_small_struct_s;
  if smallstruct_s.u16 <> RESULT_U16BIT then
    failed := true;
  if smallstruct_s.w8 <> RESULT_U8BIT then
    failed := true;

  mediumstruct := test_function_medium_struct;
  if mediumstruct.u8 <> RESULT_U8BIT then
    failed := true;
  if mediumstruct.u32 <> RESULT_U32BIT then
    failed := true;

  bigstruct := test_function_struct;
  if bigstruct.u8 <> RESULT_U8BIT then
    failed := true;
  if bigstruct.s64 <> RESULT_S64BIT then
    failed := true;
  if bigstruct.u16 <> RESULT_U16BIT then
    failed := true;

  If failed then
   fail
  else
    WriteLn('Passed!');

  if has_errors then
    Halt(1);
end;

begin
  dotest;
end.
