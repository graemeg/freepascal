<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Tests passing of different records by value to C methods.
 One type of these records has one field which is a simple array of bytes,
 the other consists of a few fields of atomic size.

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Tests passing of different records by value to C methods. 
 One type of these records has one field which is a simple array of bytes,
 the other consists of a few fields of atomic size.
 
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
 Note that it does not only test a single field of these records, but all
 by comparing the sum of the field values with the sum returned by the
 C function.
}
program calext6;
{$MODE DELPHI}

{ requires libgcc for the C functions }
{$ifdef FPUSOFT}
  {$define NO_FLOAT}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(CPUARMEL) and defined(FPUSOFT)}
=======
{$ifdef CPUARMEL}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef CPUARMEL}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef CPUARMEL}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef CPUARMEL}
>>>>>>> origin/cpstrnew
{ for softfloat calls in the C code }
{$linklib gcc}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef VER2_4}
uses
  ctypes;

type
  cextended = clongdouble;
{$endif VER2_4}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type
  int8_t = shortint;
  pint8_t = ^int8_t;
  int16_t = smallint;
  int32_t = longint;
  int64_t = int64;

var
  success : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef CPUx86_64}
    {$define HAS_GET_FRAME}
  {$endif CPUx86_64}
  {$ifdef CPUi386}
    {$define HAS_GET_FRAME}
    {$define TestFPUStack}
  {$endif CPUi386}
  {$ifdef HAS_GET_FRAME}
    {$define UseStackCheck}
  {$endif HAS_GET_FRAME}

{$ifdef UseStackCheck}
var
  stackval : pointer;

procedure SetStack;
var
  newval : pointer;
begin
{$ifdef HAS_GET_FRAME}
  newval:=Get_Frame;
{$endif HAS_GET_FRAME}
  stackval:=newval;
end;

procedure CheckStack;
var
  newval : pointer;
{$ifdef cpui386}
  { Test tag word of coprocessor }
  {$asmmode att}
  fpusave : record
    Ctrl, Status,Tag : dword;
    other : array [1..512] of byte;
  end;
{$endif}
begin
{$ifdef HAS_GET_FRAME}
  newval:=Get_Frame;
{$endif HAS_GET_FRAME}
  if newval<>stackval then
    begin
      Writeln('Stack value changed: 0x',
        hexstr(ptruint(stackval),2*sizeof(ptruint)),
        ' to 0x',hexstr(ptruint(newval),2*sizeof(ptruint)));
      Stackval:=newval;
    end;
 {$ifdef cpui386}
    asm
      fnsave fpusave
      frstor fpusave
    end;
    if ((fpusave.tag and $ffff)<>$ffff) then
      Writeln('Tag word not $ffff, ',hexstr((fpusave.tag and $ffff),4));
  {$endif}
end;
{$endif UseStackCheck}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$packrecords c}

type
  struct1 = record
    v : single;
  end;

  struct2 = record
    v : double;
  end;

  struct3 = record
    v1 : single;
    v2 : single;
  end;

  struct4 = record
    v1 : double;
    v2 : single;
  end;

  struct5 = record
    v1 : double;
    v2 : double;
  end;

  struct6 = record
    v1 : double;
    v2 : single;
    v3 : single;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> origin/cpstrnew
  struct7 = record
    v1 : single;
    v2 : int32_t;
    v3 : double;
  end;

  struct8 = record
    case byte of
      0: (v1: single);
      1: (d: double);
  end;

  struct9 = record
    v1 : int64_t;
    v2 : single;
  end;

  struct10 = record
    v1 : int64_t;
    v2 : int16_t;
    v3 : single;
  end;

  struct11 = record
    v1 : int64_t;
    v2 : double;
  end;

  struct12 = record
    v1 : int64_t;
    v2 : single;
    v3 : single;
  end;

  struct13 = record
    v1 : double;
    v2 : int64_t;
  end;

  struct14 = record
    v1 : double;
    v2 : int32_t;
    v3 : int16_t;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  struct15 = record
=======
  
  struct15 = record 
>>>>>>> graemeg/cpstrnew
=======
  
  struct15 = record 
>>>>>>> graemeg/cpstrnew
=======
  
  struct15 = record 
>>>>>>> graemeg/cpstrnew
=======
  
  struct15 = record 
>>>>>>> origin/cpstrnew
    v1 : double;
    v2 : int32_t;
    v3 : single;
  end;

  struct16 = record
    v1 : single;
    v2 : single;
    v3 : single;
    v4 : single;
  end;

  struct17 = record
    v1 : single;
    v2 : double;
  end;

  struct31 = record
    v1 : cextended;
    v2 : single;
  end;

procedure fill(var mem; size : integer);
var
  i : Integer;
  p : pint8_t;
begin
  p := @mem;
  for i := 0 to size-1 do begin
    p^ := random(255)+1;
    inc(p);
  end;
end;

procedure verify(val1, val2 : int64_t; nr : Integer); overload;
begin
  success := success and (val1 = val2);
  Write('Testing test ', nr , ', was ', val1, ', should be ', val2, '...');
  if (val1 = val2) then
    WriteLn('Success.')
  else
    WriteLn('Failed');
end;

procedure verify(val1, val2 : double; nr : Integer); overload;
begin
  success := success and (val1 = val2);
  Write('Testing test ', nr , ', was ', val1, ', should be ', val2, '...');
  if (val1 = val2) then
    WriteLn('Success.')
  else
    WriteLn('Failed');
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(FPC_HAS_TYPE_EXTENDED) and (sizeof(double)<>sizeof(cextended))}
=======
{$ifdef FPC_HAS_TYPE_EXTENDED}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef FPC_HAS_TYPE_EXTENDED}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef FPC_HAS_TYPE_EXTENDED}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef FPC_HAS_TYPE_EXTENDED}
>>>>>>> origin/cpstrnew
procedure verify(val1, val2 : cextended; nr : Integer); overload;
begin
  success := success and (val1 = val2);
  Write('Testing test ', nr , ', was ', val1, ', should be ', val2, '...');
  if (val1 = val2) then
    WriteLn('Success.')
  else
    WriteLn('Failed');
end;
{$endif FPC_HAS_TYPE_EXTENDED}

function check1(s : struct1) : single;
begin
  result := s.v;
end;

function check2(s : struct2) : double;
begin
  result := s.v;
end;

function check3(s : struct3) : single;
begin
  result := s.v1 + s.v2;
end;

function check4(s : struct4) : double;
begin
  result := s.v1 + s.v2;
end;

function check5(s : struct5) : double;
begin
  result := s.v1 + s.v2;
end;

function check6(s : struct6) : double;
begin
  result := s.v1 + s.v2;
end;

function check7(s : struct7) : double;
begin
  result := s.v1 + s.v2 + s.v3;
end;

function check8(s : struct8) : double;
begin
  result := s.d;
end;

function check9(s : struct9) : int64_t;
begin
  result := s.v1 + trunc(s.v2);
end;

function check10(s : struct10) : int64_t;
begin
  result := s.v1 + s.v2 + trunc(s.v3);
end;

function check11(s : struct11) : int64_t;
begin
  result := s.v1 + trunc(s.v2);
end;

function check12(s : struct12) : int64_t;
begin
  result := s.v1 + trunc(s.v2) + trunc(s.v3);
end;

function check13(s : struct13) : int64_t;
begin
  result := trunc(s.v1) + s.v2 ;
end;

function check14(s : struct14) : int64_t;
begin
  result := trunc(s.v1) + s.v2 + s.v3;
end;

function check15(s : struct15) : int64_t;
begin
  result := trunc(s.v1) + s.v2 + trunc(s.v3);
end;

function check16(s : struct16) : single;
begin
  result := s.v1 + s.v2 + s.v3 + s.v4;
end;

function check17(s : struct17) : double;
begin
  result := s.v1 + s.v2;
end;

function check31(s : struct31) : cextended;
begin
  { don't perform an addition, because that causes the C code to depend on
    libgcc }
  result := s.v1;
end;


{$L tcext6.o}
function pass1(s : struct1; b: byte) : single; cdecl; external;
function pass2(s : struct2; b: byte) : double; cdecl; external;
function pass3(s : struct3; b: byte) : single; cdecl; external;
function pass4(s : struct4; b: byte) : double; cdecl; external;
function pass5(s : struct5; b: byte) : double; cdecl; external;
function pass6(s : struct6; b: byte) : double; cdecl; external;
function pass61(d1,d2,d3,d4,d5: double; s : struct6; b: byte) : double; cdecl; external;
function pass7(s : struct7; b: byte) : double; cdecl; external;
function pass8(s : struct8; b: byte) : double; cdecl; external;
function pass9(s : struct9; b: byte) : int64_t; cdecl; external;
function pass10(s : struct10; b: byte) : int64_t; cdecl; external;
function pass11(s : struct11; b: byte) : int64_t; cdecl; external;
function pass12(s : struct12; b: byte) : int64_t; cdecl; external;
function pass13(s : struct13; b: byte) : int64_t; cdecl; external;
function pass14(s : struct14; b: byte) : int64_t; cdecl; external;
function pass15(s : struct15; b: byte) : int64_t; cdecl; external;
function pass16(s : struct16; b: byte) : single; cdecl; external;
function pass17(s : struct17; b: byte) : single; cdecl; external;
{$ifdef FPC_HAS_TYPE_EXTENDED}
function pass31(s : struct31; b: byte; var ss: single) : cextended; cdecl; external;
{$endif}

function pass1a(b: byte; s : struct1) : struct1; cdecl; external;
function pass2a(b: byte; s : struct2) : struct2; cdecl; external;
function pass3a(b: byte; s : struct3) : struct3; cdecl; external;
function pass4a(b: byte; s : struct4) : struct4; cdecl; external;
function pass5a(b: byte; s : struct5) : struct5; cdecl; external;
function pass6a(b: byte; s : struct6) : struct6; cdecl; external;
function pass7a(b: byte; s : struct7) : struct7; cdecl; external;
function pass8a(b: byte; s : struct8) : struct8; cdecl; external;
function pass9a(b: byte; s : struct9) : struct9; cdecl; external;
function pass10a(b: byte; s : struct10) : struct10; cdecl; external;
function pass11a(b: byte; s : struct11) : struct11; cdecl; external;
function pass12a(b: byte; s : struct12) : struct12; cdecl; external;
function pass13a(b: byte; s : struct13) : struct13; cdecl; external;
function pass14a(b: byte; s : struct14) : struct14; cdecl; external;
function pass15a(b: byte; s : struct15) : struct15; cdecl; external;
function pass16a(b: byte; s : struct16) : struct16; cdecl; external;
function pass17a(b: byte; s : struct17) : struct17; cdecl; external;
{$ifdef FPC_HAS_TYPE_EXTENDED}
function pass31a(b: byte; s : struct31) : struct31; cdecl; external;
{$endif}

procedure dotest;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef TestFPUStack }
  i : longint;
  {$endif TestFPUStack }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  s1, s1a: struct1;
  s2, s2a: struct2;
  s3, s3a: struct3;
  s4, s4a: struct4;
  s5, s5a: struct5;
  s6, s6a: struct6;
  s7, s7a: struct7;
  s8, s8a: struct8;
  s9, s9a: struct9;
  s10, s10a: struct10;
  s11, s11a: struct11;
  s12, s12a: struct12;
  s13, s13a: struct13;
  s14, s14a: struct14;
  s15, s15a: struct15;
  s16, s16a: struct16;
  s17, s17a: struct17;
  s31, s31a: struct31;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> origin/cpstrnew
  ss: single;

begin
  success := true;

{$ifndef NO_FLOAT}
  s1.v:=2.0;

  s2.v:=3.0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  s3.v1:=4.5;
  s3.v2:=5.125;

  s4.v1:=6.175;
  s4.v2:=7.5;

  s5.v1:=8.075;
  s5.v2:=9.000125;

  s6.v1:=10.25;
  s6.v2:=11.5;
  s6.v3:=12.125;

  s7.v1:=13.5;
  s7.v2:=14;
  s7.v3:=15.0625;

  s8.d:=16.000575;

  s9.v1:=$123456789012345;
  s9.v2:=17.0;

  s10.v1:=$234567890123456;
  s10.v2:=-12399;
  s10.v3:=18.0;

  s11.v1:=$345678901234567;
  s11.v2:=19.0;

  s12.v1:=$456789012345678;
  s12.v2:=20.0;
  s12.v3:=21.0;

  s13.v1:=22.0;
  s13.v2:=$567890123456789;

  s14.v1:=23.0;
  s14.v2:=$19283774;
  s14.v3:=12356;

  s15.v1:=24.0;
  s15.v2:=$28195647;
  s15.v3:=25.0;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  
  s3.v1:=4.5;
  s3.v2:=5.125;
  
  s4.v1:=6.175;
  s4.v2:=7.5;
  
  s5.v1:=8.075;
  s5.v2:=9.000125;
  
  s6.v1:=10.25;
  s6.v2:=11.5;
  s6.v3:=12.125;
  
  s7.v1:=13.5;
  s7.v2:=14;
  s7.v3:=15.0625;
  
  s8.d:=16.000575;
  
  s9.v1:=$123456789012345;
  s9.v2:=17.0;
  
  s10.v1:=$234567890123456;
  s10.v2:=-12399;
  s10.v3:=18.0;
  
  s11.v1:=$345678901234567;
  s11.v2:=19.0;
  
  s12.v1:=$456789012345678;
  s12.v2:=20.0;
  s12.v3:=21.0;
  
  s13.v1:=22.0;
  s13.v2:=$567890123456789;
  
  s14.v1:=23.0;
  s14.v2:=$19283774;
  s14.v3:=12356;
  
  s15.v1:=24.0;
  s15.v2:=$28195647;
  s15.v3:=25.0;
  
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
  s16.v1:=26.5;
  s16.v2:=27.75;
  s16.v3:=28.25;
  s16.v4:=29.125;

  s17.v1:=31.25;
  s17.v2:=32.125;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  s31.v1:=32.625;
  s31.v2:=33.5;

{$ifdef UseStackCheck}
  SetStack;
{$endif UseStackCheck}
  verify(pass1(s1,1), check1(s1), 1);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass2(s2,2), check2(s2), 2);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass3(s3,3), check3(s3), 3);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass4(s4,4), check4(s4), 4);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass5(s5,5), check5(s5), 5);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass6(s6,6), check6(s6), 6);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass7(s7,7), check7(s7), 7);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass8(s8,8), check8(s8), 8);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass9(s9,9), check9(s9), 9);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass10(s10,10), check10(s10), 10);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass11(s11,11), check11(s11), 11);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass12(s12,12), check12(s12), 12);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass13(s13,13), check13(s13), 13);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass14(s14,14), check14(s14), 14);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass15(s15,15), check15(s15), 15);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass16(s16,16), check16(s16), 16);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass17(s17,17), check17(s17), 17);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
{$ifdef FPC_HAS_TYPE_EXTENDED}
  verify(pass31(s31,31,ss), check31(s31), 31);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(ss,s31.v2,32);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
{$endif}

  verify(check1(pass1a(1,s1)), check1(s1), 41);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check2(pass2a(2,s2)), check2(s2), 42);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check3(pass3a(3,s3)), check3(s3), 43);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check4(pass4a(4,s4)), check4(s4), 44);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check5(pass5a(5,s5)), check5(s5), 45);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check6(pass6a(6,s6)), check6(s6), 46);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check7(pass7a(7,s7)), check7(s7), 47);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check8(pass8a(8,s8)), check8(s8), 48);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check9(pass9a(9,s9)), check9(s9), 49);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check10(pass10a(10,s10)), check10(s10), 50);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check11(pass11a(11,s11)), check11(s11), 51);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check12(pass12a(12,s12)), check12(s12), 52);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check13(pass13a(13,s13)), check13(s13), 53);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check14(pass14a(14,s14)), check14(s14), 54);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check15(pass15a(15,s15)), check15(s15), 55);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check16(pass16a(16,s16)), check16(s16), 56);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check17(pass17a(17,s17)), check17(s17), 57);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
{$ifdef FPC_HAS_TYPE_EXTENDED}
  verify(check31(pass31a(31,s31)), check31(s31), 71);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
{$endif}

  verify(pass1a(1,s1).v, s1.v, 81);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass2a(2,s2).v, s2.v, 82);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass3a(3,s3).v1, s3.v1, 83);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass3a(3,s3).v2, s3.v2, 103);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass4a(4,s4).v1, s4.v1, 84);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass5a(5,s5).v1, s5.v1, 85);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass6a(6,s6).v1, s6.v1, 86);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass7a(7,s7).v1, s7.v1, 87);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass7a(7,s7).v2, s7.v2, 107);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass8a(8,s8).d, s8.d, 88);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass9a(9,s9).v1, s9.v1, 89);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass10a(10,s10).v1, s10.v1, 90);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass10a(10,s10).v2, s10.v2, 90);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass11a(11,s11).v1, s11.v1, 91);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass12a(12,s12).v1, s12.v1, 92);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass13a(13,s13).v1, s13.v1, 93);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass14a(14,s14).v1, s14.v1, 94);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass15a(15,s15).v1, s15.v1, 95);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass16a(16,s16).v1, s16.v1, 96);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(pass17a(17,s17).v1, s17.v1, 97);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
{$ifdef FPC_HAS_TYPE_EXTENDED}
  verify(pass31a(31,s31).v1, s31.v1, 101);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
{$endif}

  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s1a:=pass1a(1,s1);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check1(s1a), check1(s1), 111);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s2a:=pass2a(2,s2);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check2(s2a), check2(s2), 112);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s3a:=pass3a(3,s3);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check3(s3a), check3(s3), 113);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s3a:=pass3a(3,s3);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check3(s3a), check3(s3), 114);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s4a:=pass4a(4,s4);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check4(s4a), check4(s4), 115);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s5a:=pass5a(5,s5);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check5(s5a), check5(s5), 116);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s6a:=pass6a(6,s6);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check6(s6a), check6(s6), 117);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s7a:=pass7a(7,s7);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check7(s7a), check7(s7), 118);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s7a:=pass7a(7,s7);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check7(s7a), check7(s7), 119);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s8a:=pass8a(8,s8);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check8(s8a), check8(s8), 120);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s9a:=pass9a(9,s9);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check9(s9a), check9(s9), 121);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s10a:=pass10a(10,s10);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check10(s10a), check10(s10), 122);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s10a:=pass10a(10,s10);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check10(s10a), check10(s10), 123);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s11a:=pass11a(11,s11);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check11(s11a), check11(s11), 124);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s12a:=pass12a(12,s12);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check12(s12a), check12(s12), 125);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s13a:=pass13a(13,s13);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check13(s13a), check13(s13), 126);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s14a:=pass14a(14,s14);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check14(s14a), check14(s14), 127);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s15a:=pass15a(15,s15);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check15(s15a), check15(s15), 128);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s16a:=pass16a(16,s16);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check16(s16a), check16(s16), 129);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s17a:=pass17a(17,s17);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check17(s17a), check17(s17), 130);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
{$ifdef FPC_HAS_TYPE_EXTENDED}
  {$ifdef TestFPUStack }
  for i:=1 to 12 do
  {$endif TestFPUStack }
  s31a:=pass31a(31,s31);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(check31(s31a), check31(s31), 131);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
  verify(s31.v2,s31a.v2,132);
{$ifdef UseStackCheck}
  CheckStack;
{$endif UseStackCheck}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  
  s31.v1:=32.625;
  s31.v2:=33.5;

  verify(pass1(s1,1), check1(s1), 1);
  verify(pass2(s2,2), check2(s2), 2);
  verify(pass3(s3,3), check3(s3), 3);
  verify(pass4(s4,4), check4(s4), 4);
  verify(pass5(s5,5), check5(s5), 5);
  verify(pass6(s6,6), check6(s6), 6);
  verify(pass7(s7,7), check7(s7), 7);
  verify(pass8(s8,8), check8(s8), 8);
  verify(pass9(s9,9), check9(s9), 9);
  verify(pass10(s10,10), check10(s10), 10);
  verify(pass11(s11,11), check11(s11), 11);
  verify(pass12(s12,12), check12(s12), 12);
  verify(pass13(s13,13), check13(s13), 13);
  verify(pass14(s14,14), check14(s14), 14);
  verify(pass15(s15,15), check15(s15), 15);
  verify(pass16(s16,16), check16(s16), 16);
  verify(pass17(s17,17), check17(s17), 17);
{$ifdef FPC_HAS_TYPE_EXTENDED}
  verify(pass31(s31,31,ss), check31(s31), 31);
  verify(ss,s31.v2,32);
{$endif}

  verify(check1(pass1a(1,s1)), check1(s1), 41);
  verify(check2(pass2a(2,s2)), check2(s2), 42);
  verify(check3(pass3a(3,s3)), check3(s3), 43);
  verify(check4(pass4a(4,s4)), check4(s4), 44);
  verify(check5(pass5a(5,s5)), check5(s5), 45);
  verify(check6(pass6a(6,s6)), check6(s6), 46);
  verify(check7(pass7a(7,s7)), check7(s7), 47);
  verify(check8(pass8a(8,s8)), check8(s8), 48);
  verify(check9(pass9a(9,s9)), check9(s9), 49);
  verify(check10(pass10a(10,s10)), check10(s10), 50);
  verify(check11(pass11a(11,s11)), check11(s11), 51);
  verify(check12(pass12a(12,s12)), check12(s12), 52);
  verify(check13(pass13a(13,s13)), check13(s13), 53);
  verify(check14(pass14a(14,s14)), check14(s14), 54);
  verify(check15(pass15a(15,s15)), check15(s15), 55);
  verify(check16(pass16a(16,s16)), check16(s16), 56);
  verify(check17(pass17a(17,s17)), check17(s17), 57);
{$ifdef FPC_HAS_TYPE_EXTENDED}
  verify(check31(pass31a(31,s31)), check31(s31), 71);
{$endif}

  verify(pass1a(1,s1).v, s1.v, 81);
  verify(pass2a(2,s2).v, s2.v, 82);
  verify(pass3a(3,s3).v1, s3.v1, 83);
  verify(pass3a(3,s3).v2, s3.v2, 103);
  verify(pass4a(4,s4).v1, s4.v1, 84);
  verify(pass5a(5,s5).v1, s5.v1, 85);
  verify(pass6a(6,s6).v1, s6.v1, 86);
  verify(pass7a(7,s7).v1, s7.v1, 87);
  verify(pass7a(7,s7).v2, s7.v2, 107);
  verify(pass8a(8,s8).d, s8.d, 88);
  verify(pass9a(9,s9).v1, s9.v1, 89);
  verify(pass10a(10,s10).v1, s10.v1, 90);
  verify(pass10a(10,s10).v2, s10.v2, 90);
  verify(pass11a(11,s11).v1, s11.v1, 91);
  verify(pass12a(12,s12).v1, s12.v1, 92);
  verify(pass13a(13,s13).v1, s13.v1, 93);
  verify(pass14a(14,s14).v1, s14.v1, 94);
  verify(pass15a(15,s15).v1, s15.v1, 95);
  verify(pass16a(16,s16).v1, s16.v1, 96);
  verify(pass17a(17,s17).v1, s17.v1, 97);
{$ifdef FPC_HAS_TYPE_EXTENDED}
  verify(pass31a(31,s31).v1, s31.v1, 101);
{$endif}

  s1a:=pass1a(1,s1);
  verify(check1(s1a), check1(s1), 111);
  s2a:=pass2a(2,s2);
  verify(check2(s2a), check2(s2), 112);
  s3a:=pass3a(3,s3);
  verify(check3(s3a), check3(s3), 113);
  s3a:=pass3a(3,s3);
  verify(check3(s3a), check3(s3), 114);
  s4a:=pass4a(4,s4);
  verify(check4(s4a), check4(s4), 115);
  s5a:=pass5a(5,s5);
  verify(check5(s5a), check5(s5), 116);
  s6a:=pass6a(6,s6);
  verify(check6(s6a), check6(s6), 117);
  s7a:=pass7a(7,s7);
  verify(check7(s7a), check7(s7), 118);
  s7a:=pass7a(7,s7);
  verify(check7(s7a), check7(s7), 119);
  s8a:=pass8a(8,s8);
  verify(check8(s8a), check8(s8), 120);
  s9a:=pass9a(9,s9);
  verify(check9(s9a), check9(s9), 121);
  s10a:=pass10a(10,s10);
  verify(check10(s10a), check10(s10), 122);
  s10a:=pass10a(10,s10);
  verify(check10(s10a), check10(s10), 123);
  s11a:=pass11a(11,s11);
  verify(check11(s11a), check11(s11), 124);
  s12a:=pass12a(12,s12);
  verify(check12(s12a), check12(s12), 125);
  s13a:=pass13a(13,s13);
  verify(check13(s13a), check13(s13), 126);
  s14a:=pass14a(14,s14);
  verify(check14(s14a), check14(s14), 127);
  s15a:=pass15a(15,s15);
  verify(check15(s15a), check15(s15), 128);
  s16a:=pass16a(16,s16);
  verify(check16(s16a), check16(s16), 129);
  s17a:=pass17a(17,s17);
  verify(check17(s17a), check17(s17), 130);
{$ifdef FPC_HAS_TYPE_EXTENDED}
  s31a:=pass31a(31,s31);
  verify(check31(s31a), check31(s31), 131);
  verify(s31.v2,s31a.v2,132);
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
{$endif}

{$endif ndef nofloat}

  if (not success) then
    halt(1);
end;

begin
  dotest;
end.
