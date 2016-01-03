{
    This file is part of the Free Pascal packages.
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 1999-2014 by the Free Pascal development team
=======
    Copyright (c) 1999-2006 by the Free Pascal development team
>>>>>>> graemeg/fixes_2_2
=======
    Copyright (c) 1999-2006 by the Free Pascal development team
>>>>>>> origin/fixes_2_2

    Implements a MD2 digest algorithm (RFC 1319)
    Implements a MD4 digest algorithm (RFC 1320)
    Implements a MD5 digest algorithm (RFC 1321)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
// Define to use original MD5 code on i386 processors.
// Undefine to use original implementation.
{ the assembler implementation does not work on Darwin }
{$ifdef darwin}
{$DEFINE MD5PASCAL}
{$endif darwin}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
unit md5;

{$mode objfpc}
{$inline on}
{$h+}

interface


(******************************************************************************
 * types and constants
 ******************************************************************************)

const
  MDDefBufSize = 1024;

type
  TMDVersion = (
    MD_VERSION_2,
    MD_VERSION_4,
    MD_VERSION_5
  );

  PMDDigest = ^TMDDigest;
  TMDDigest = array[0..15] of Byte;

  PMD2Digset = PMDDigest;
  TMD2Digest = TMDDigest;

  PMD4Digset = PMDDigest;
  TMD4Digest = TMDDigest;

  PMD5Digset = PMDDigest;
  TMD5Digest = TMDDigest;

  PMDContext = ^TMDContext;
<<<<<<< HEAD
<<<<<<< HEAD
  TMDHashFunc = procedure(Context: PMDContext; Buffer: Pointer);
  TMDContext = record
    Version : TMDVersion;
    Hash    : TMDHashFunc;
=======
  TMDContext = record
    Version : TMDVersion;
>>>>>>> graemeg/fixes_2_2
=======
  TMDContext = record
    Version : TMDVersion;
>>>>>>> origin/fixes_2_2
    Align   : PtrUInt;
    State   : array[0..3] of Cardinal;
    BufCnt  : QWord;
    Buffer  : array[0..63] of Byte;
    case Integer of
<<<<<<< HEAD
<<<<<<< HEAD
      0: (Length   : QWord);
=======
      0: (Length   : PtrUInt);
>>>>>>> graemeg/fixes_2_2
=======
      0: (Length   : PtrUInt);
>>>>>>> origin/fixes_2_2
      1: (Checksum : array[0..15] of Byte);
  end;

  PMD2Context = PMDContext;
  TMD2Context = TMDContext;

  PMD4Context = PMDContext;
  TMD4Context = TMDContext;

  PMD5Context = PMDContext;
  TMD5Context = TMDContext;



(******************************************************************************
 * Core raw functions
 ******************************************************************************)

<<<<<<< HEAD
<<<<<<< HEAD
procedure MDInit(out Context: TMDContext; const Version: TMDVersion);
procedure MDUpdate(var Context: TMDContext; var Buf; const BufLen: PtrUInt);
procedure MDFinal(var Context: TMDContext; out Digest: TMDDigest);
=======
procedure MDInit(var Context: TMDContext; const Version: TMDVersion);
procedure MDUpdate(var Context: TMDContext; var Buf; const BufLen: PtrUInt);
procedure MDFinal(var Context: TMDContext; var Digest: TMDDigest);
>>>>>>> graemeg/fixes_2_2
=======
procedure MDInit(var Context: TMDContext; const Version: TMDVersion);
procedure MDUpdate(var Context: TMDContext; var Buf; const BufLen: PtrUInt);
procedure MDFinal(var Context: TMDContext; var Digest: TMDDigest);
>>>>>>> origin/fixes_2_2


(******************************************************************************
 * Auxilary functions
 ******************************************************************************)

function MDString(const S: String; const Version: TMDVersion): TMDDigest;
function MDBuffer(var Buf; const BufLen: PtrUInt; const Version: TMDVersion): TMDDigest;
<<<<<<< HEAD
<<<<<<< HEAD
function MDFile(const Filename: RawByteString; const Version: TMDVersion; const Bufsize: PtrUInt = MDDefBufSize): TMDDigest;
function MDFile(const Filename: UnicodeString; const Version: TMDVersion; const Bufsize: PtrUInt = MDDefBufSize): TMDDigest;
=======
function MDFile(const Filename: String; const Version: TMDVersion; const Bufsize: PtrUInt = MDDefBufSize): TMDDigest;

>>>>>>> graemeg/fixes_2_2
=======
function MDFile(const Filename: String; const Version: TMDVersion; const Bufsize: PtrUInt = MDDefBufSize): TMDDigest;

>>>>>>> origin/fixes_2_2

(******************************************************************************
 * Helper functions
 ******************************************************************************)

function MDPrint(const Digest: TMDDigest): String;
function MDMatch(const Digest1, Digest2: TMDDigest): Boolean;


(******************************************************************************
 * Dedicated raw functions
 ******************************************************************************)

<<<<<<< HEAD
<<<<<<< HEAD
procedure MD2Init(out Context: TMD2Context); inline;
procedure MD2Update(var Context: TMD2Context; var Buf; const BufLen: PtrUInt); external name 'MD_UPDATE';
procedure MD2Final(var Context: TMD2Context; out Digest: TMD2Digest); external name 'MD_FINAL';

procedure MD4Init(out Context: TMD4Context); inline;
procedure MD4Update(var Context: TMD4Context; var Buf; const BufLen: PtrUInt); external name 'MD_UPDATE';
procedure MD4Final(var Context: TMD4Context; out Digest: TMD4Digest); external name 'MD_FINAL';

procedure MD5Init(out Context: TMD5Context); inline;
procedure MD5Update(var Context: TMD5Context; var Buf; const BufLen: PtrUInt); external name 'MD_UPDATE';
procedure MD5Final(var Context: TMD5Context; out Digest: TMD5Digest); external name 'MD_FINAL';
=======
=======
>>>>>>> origin/fixes_2_2
procedure MD2Init(var Context: TMD2Context); inline;
procedure MD2Update(var Context: TMD2Context; var Buf; const BufLen: PtrUInt); inline;
procedure MD2Final(var Context: TMD2Context; var Digest: TMD2Digest); inline;

procedure MD4Init(var Context: TMD4Context); inline;
procedure MD4Update(var Context: TMD4Context; var Buf; const BufLen: PtrUInt); inline;
procedure MD4Final(var Context: TMD4Context; var Digest: TMD4Digest); inline;

procedure MD5Init(var Context: TMD5Context); inline;
procedure MD5Update(var Context: TMD5Context; var Buf; const BufLen: PtrUInt); inline;
procedure MD5Final(var Context: TMD5Context; var Digest: TMD5Digest); inline;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


(******************************************************************************
 * Dedicated auxilary functions
 ******************************************************************************)

function MD2String(const S: String): TMD2Digest; inline;
<<<<<<< HEAD
<<<<<<< HEAD
function MD2Buffer(var Buf; const BufLen: PtrUInt): TMD2Digest;
function MD2File(const Filename: RawByteString; const Bufsize: PtrUInt = MDDefBufSize): TMD2Digest; overload; inline;
function MD2File(const Filename: UnicodeString; const Bufsize: PtrUInt = MDDefBufSize): TMD2Digest; overload; inline;

function MD4String(const S: String): TMD4Digest; inline;
function MD4Buffer(var Buf; const BufLen: PtrUInt): TMD4Digest;
function MD4File(const Filename: RawByteString; const Bufsize: PtrUInt = MDDefBufSize): TMD4Digest; inline;
function MD4File(const Filename: UnicodeString; const Bufsize: PtrUInt = MDDefBufSize): TMD4Digest; inline;

function MD5String(const S: String): TMD5Digest; inline;
function MD5Buffer(var Buf; const BufLen: PtrUInt): TMD5Digest;
function MD5File(const Filename: RawByteString; const Bufsize: PtrUInt = MDDefBufSize): TMD5Digest; inline;
function MD5File(const Filename: UnicodeString; const Bufsize: PtrUInt = MDDefBufSize): TMD5Digest; inline;
=======
=======
>>>>>>> origin/fixes_2_2
function MD2Buffer(var Buf; const BufLen: PtrUInt): TMD2Digest; inline;
function MD2File(const Filename: String; const Bufsize: PtrUInt = MDDefBufSize): TMD2Digest; inline;

function MD4String(const S: String): TMD4Digest; inline;
function MD4Buffer(var Buf; const BufLen: PtrUInt): TMD4Digest; inline;
function MD4File(const Filename: String; const Bufsize: PtrUInt = MDDefBufSize): TMD4Digest; inline;

function MD5String(const S: String): TMD5Digest; inline;
function MD5Buffer(var Buf; const BufLen: PtrUInt): TMD5Digest; inline;
function MD5File(const Filename: String; const Bufsize: PtrUInt = MDDefBufSize): TMD5Digest; inline;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2



(******************************************************************************
 * Dedicated helper functions
 ******************************************************************************)

function MD2Print(const Digest: TMD2Digest): String; inline;
function MD2Match(const Digest1, Digest2: TMD2Digest): Boolean; inline;

function MD4Print(const Digest: TMD4Digest): String; inline;
function MD4Match(const Digest1, Digest2: TMD4Digest): Boolean; inline;

function MD5Print(const Digest: TMD5Digest): String; inline;
<<<<<<< HEAD
<<<<<<< HEAD
function StrtoMD5(const MD5String:String):TMDDigest;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MD5Match(const Digest1, Digest2: TMD5Digest): Boolean; inline;

implementation

<<<<<<< HEAD
<<<<<<< HEAD
uses sysutils;
=======
=======
>>>>>>> origin/fixes_2_2

function rol(x: Cardinal; n: Byte): Cardinal;
begin
  Result := (x shl n) or (x shr (32 - n));
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

// inverts the bytes of (Count div 4) cardinals from source to target.
procedure Invert(Source, Dest: Pointer; Count: PtrUInt);
var
  S: PByte;
  T: PCardinal;
  I: PtrUInt;
begin
  S := Source;
  T := Dest;
  for I := 1 to (Count div 4) do
  begin
    T^ := S[0] or (S[1] shl 8) or (S[2] shl 16) or (S[3] shl 24);
    inc(S,4);
    inc(T);
  end;
end;


procedure MD2Transform(var Context: TMDContext; Buffer: Pointer);
const
  PI_SUBST: array[0..255] of Byte = (
  41, 46, 67, 201, 162, 216, 124, 1, 61, 54, 84, 161, 236, 240, 6,
  19, 98, 167, 5, 243, 192, 199, 115, 140, 152, 147, 43, 217, 188,
  76, 130, 202, 30, 155, 87, 60, 253, 212, 224, 22, 103, 66, 111, 24,
  138, 23, 229, 18, 190, 78, 196, 214, 218, 158, 222, 73, 160, 251,
  245, 142, 187, 47, 238, 122, 169, 104, 121, 145, 21, 178, 7, 63,
  148, 194, 16, 137, 11, 34, 95, 33, 128, 127, 93, 154, 90, 144, 50,
  39, 53, 62, 204, 231, 191, 247, 151, 3, 255, 25, 48, 179, 72, 165,
  181, 209, 215, 94, 146, 42, 172, 86, 170, 198, 79, 184, 56, 210,
  150, 164, 125, 182, 118, 252, 107, 226, 156, 116, 4, 241, 69, 157,
  112, 89, 100, 113, 135, 32, 134, 91, 207, 101, 230, 45, 168, 2, 27,
  96, 37, 173, 174, 176, 185, 246, 28, 70, 97, 105, 52, 64, 126, 15,
  85, 71, 163, 35, 221, 81, 175, 58, 195, 92, 249, 206, 186, 197,
  234, 38, 44, 83, 13, 110, 133, 40, 132, 9, 211, 223, 205, 244, 65,
  129, 77, 82, 106, 220, 55, 200, 108, 193, 171, 250, 36, 225, 123,
  8, 12, 189, 177, 74, 120, 136, 149, 139, 227, 99, 232, 109, 233,
  203, 213, 254, 59, 0, 29, 57, 242, 239, 183, 14, 102, 88, 208, 228,
  166, 119, 114, 248, 235, 117, 75, 10, 49, 68, 80, 180, 143, 237,
  31, 26, 219, 153, 141, 51, 159, 17, 131, 20
);
var
  i: Cardinal;
  j: Cardinal;
  t: Cardinal;
  x: array[0..47] of Byte;
begin
  { Form encryption block from state, block, state ^ block }
  Move(Context.State, x[0], 16);
  Move(Buffer^, x[16], 16);
  for i := 0 to 15 do
    x[i+32] := PByte(@Context.State)[i] xor PByte(Buffer)[i];

  { Encrypt block (18 rounds) }
  t := 0;
  for i := 0 to 17 do
  begin
    for j := 0 to 47 do
    begin
      x[j] := x[j] xor PI_SUBST[t];
      t := x[j];
    end;
    t := (t + i) and $FF;
  end;

  { Save new state }
  Move(x[0], Context.State, 16);

  { Update checksum }
  t := Context.Checksum[15];
  for i := 0 to 15 do
  begin
    Context.Checksum[i] := Context.Checksum[i] xor PI_SUBST[PByte(Buffer)[i] xor t];
    t := Context.Checksum[i];
  end;

  { Zeroize sensitive information. }
  FillChar(x, Sizeof(x), 0);
end;


procedure MD4Transform(var Context: TMDContext; Buffer: Pointer);

<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$r-,q-}

  procedure R1(var a: Cardinal; b,c,d,x: Cardinal; s: Byte);
  // F(x,y,z) = (x and y) or ((not x) and z)
  begin
    a := roldword(dword(a + {F(b,c,d)}((b and c) or ((not b) and d)) + x), s);
=======
=======
>>>>>>> origin/fixes_2_2
  procedure R1(var a: Cardinal; b,c,d,x: Cardinal; s: Byte);
  // F(x,y,z) = (x and y) or ((not x) and z)
  begin
    a := rol(a + {F(b,c,d)}((b and c) or ((not b) and d)) + x, s);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  procedure R2(var a: Cardinal; b,c,d,x: Cardinal; s: Byte);
  // G(x,y,z) = (x and y) or (x and z) or (y and z);
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    a := roldword(dword(a + {G(b,c,d)}((b and c) or (b and d) or (c and d)) + x + $5A827999), s);
=======
    a := rol(a + {G(b,c,d)}((b and c) or (b and d) or (c and d)) + x + $5A827999, s);
>>>>>>> graemeg/fixes_2_2
=======
    a := rol(a + {G(b,c,d)}((b and c) or (b and d) or (c and d)) + x + $5A827999, s);
>>>>>>> origin/fixes_2_2
  end;

  procedure R3(var a: Cardinal; b,c,d,x: Cardinal; s: Byte);
  // H(x,y,z) = x xor y xor z
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    a := roldword(dword(a + {H(b,c,d)}(b xor c xor d) + x + $6ED9EBA1), s);
  end;

{$pop}

=======
    a := rol(a + {H(b,c,d)}(b xor c xor d) + x + $6ED9EBA1, s);
  end;

>>>>>>> graemeg/fixes_2_2
=======
    a := rol(a + {H(b,c,d)}(b xor c xor d) + x + $6ED9EBA1, s);
  end;

>>>>>>> origin/fixes_2_2
var
  a, b, c, d: Cardinal;
  Block: array[0..15] of Cardinal;
begin
  Invert(Buffer, @Block, 64);
  a := Context.State[0];
  b := Context.State[1];
  c := Context.State[2];
  d := Context.State[3];

  // Round 1
  R1(a,b,c,d,Block[0],  3); R1(d,a,b,c,Block[1],  7); R1(c,d,a,b,Block[2], 11); R1(b,c,d,a,Block[3], 19);
  R1(a,b,c,d,Block[4],  3); R1(d,a,b,c,Block[5],  7); R1(c,d,a,b,Block[6], 11); R1(b,c,d,a,Block[7], 19);
  R1(a,b,c,d,Block[8],  3); R1(d,a,b,c,Block[9],  7); R1(c,d,a,b,Block[10],11); R1(b,c,d,a,Block[11],19);
  R1(a,b,c,d,Block[12], 3); R1(d,a,b,c,Block[13], 7); R1(c,d,a,b,Block[14],11); R1(b,c,d,a,Block[15],19);

  // Round 2
  R2(a,b,c,d,Block[0],  3); R2(d,a,b,c,Block[4],  5); R2(c,d,a,b,Block[8],  9); R2(b,c,d,a,Block[12],13);
  R2(a,b,c,d,Block[1],  3); R2(d,a,b,c,Block[5],  5); R2(c,d,a,b,Block[9],  9); R2(b,c,d,a,Block[13],13);
  R2(a,b,c,d,Block[2],  3); R2(d,a,b,c,Block[6],  5); R2(c,d,a,b,Block[10], 9); R2(b,c,d,a,Block[14],13);
  R2(a,b,c,d,Block[3],  3); R2(d,a,b,c,Block[7],  5); R2(c,d,a,b,Block[11], 9); R2(b,c,d,a,Block[15],13);

  // Round 3
  R3(a,b,c,d,Block[0],  3); R3(d,a,b,c,Block[8],  9); R3(c,d,a,b,Block[4], 11); R3(b,c,d,a,Block[12],15);
  R3(a,b,c,d,Block[2],  3); R3(d,a,b,c,Block[10], 9); R3(c,d,a,b,Block[6], 11); R3(b,c,d,a,Block[14],15);
  R3(a,b,c,d,Block[1],  3); R3(d,a,b,c,Block[9],  9); R3(c,d,a,b,Block[5], 11); R3(b,c,d,a,Block[13],15);
  R3(a,b,c,d,Block[3],  3); R3(d,a,b,c,Block[11], 9); R3(c,d,a,b,Block[7], 11); R3(b,c,d,a,Block[15],15);

<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$r-,q-}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  inc(Context.State[0], a);
  inc(Context.State[1], b);
  inc(Context.State[2], c);
  inc(Context.State[3], d);
<<<<<<< HEAD
<<<<<<< HEAD
{$pop}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  inc(Context.Length,64);
end;


<<<<<<< HEAD
<<<<<<< HEAD
{$IF (NOT(DEFINED(MD5PASCAL))) and (DEFINED(CPUI386)) }
{$i md5i386.inc}
{$ENDIF}
{$IF (NOT(DEFINED(MD5PASCAL))) and (DEFINED(CPUX86_64)) }
{$OPTIMIZATION USERBP} //PEEPHOLE
procedure MD5Transform(var Context: TMDContext; Buffer: Pointer);
type
  TBlock = array[0..15] of Cardinal;
  PBlock = ^TBlock;
var
  a, b, c, d: Cardinal;
  //Block: array[0..15] of Cardinal absolute Buffer;
  Block: PBlock absolute Buffer;
begin
  //Invert(Buffer, @Block, 64);
  a := Context.State[0];
  b := Context.State[1];
  c := Context.State[2];
  d := Context.State[3];

{$push}
{$r-,q-}

  // Round 1
  a := b + roldword(dword(a + ((b and c) or ((not b) and d)) + Block^[0]  + $d76aa478),  7);
  d := a + roldword(dword(d + ((a and b) or ((not a) and c)) + Block^[1]  + $e8c7b756), 12);
  c := d + roldword(dword(c + ((d and a) or ((not d) and b)) + Block^[2]  + $242070db), 17);
  b := c + roldword(dword(b + ((c and d) or ((not c) and a)) + Block^[3]  + $c1bdceee), 22);
  a := b + roldword(dword(a + ((b and c) or ((not b) and d)) + Block^[4]  + $f57c0faf),  7);
  d := a + roldword(dword(d + ((a and b) or ((not a) and c)) + Block^[5]  + $4787c62a), 12);
  c := d + roldword(dword(c + ((d and a) or ((not d) and b)) + Block^[6]  + $a8304613), 17);
  b := c + roldword(dword(b + ((c and d) or ((not c) and a)) + Block^[7]  + $fd469501), 22);
  a := b + roldword(dword(a + ((b and c) or ((not b) and d)) + Block^[8]  + $698098d8),  7);
  d := a + roldword(dword(d + ((a and b) or ((not a) and c)) + Block^[9]  + $8b44f7af), 12);
  c := d + roldword(dword(c + ((d and a) or ((not d) and b)) + Block^[10] + $ffff5bb1), 17);
  b := c + roldword(dword(b + ((c and d) or ((not c) and a)) + Block^[11] + $895cd7be), 22);
  a := b + roldword(dword(a + ((b and c) or ((not b) and d)) + Block^[12] + $6b901122),  7);
  d := a + roldword(dword(d + ((a and b) or ((not a) and c)) + Block^[13] + $fd987193), 12);
  c := d + roldword(dword(c + ((d and a) or ((not d) and b)) + Block^[14] + $a679438e), 17);
  b := c + roldword(dword(b + ((c and d) or ((not c) and a)) + Block^[15] + $49b40821), 22);
  // Round 2
  a := b + roldword(dword(a + ((b and d) or (c and (not d))) + Block^[1]  + $f61e2562),  5);
  d := a + roldword(dword(d + ((a and c) or (b and (not c))) + Block^[6]  + $c040b340),  9);
  c := d + roldword(dword(c + ((d and b) or (a and (not b))) + Block^[11] + $265e5a51), 14);
  b := c + roldword(dword(b + ((c and a) or (d and (not a))) + Block^[0]  + $e9b6c7aa), 20);
  a := b + roldword(dword(a + ((b and d) or (c and (not d))) + Block^[5]  + $d62f105d),  5);
  d := a + roldword(dword(d + ((a and c) or (b and (not c))) + Block^[10] + $02441453),  9);
  c := d + roldword(dword(c + ((d and b) or (a and (not b))) + Block^[15] + $d8a1e681), 14);
  b := c + roldword(dword(b + ((c and a) or (d and (not a))) + Block^[4]  + $e7d3fbc8), 20);
  a := b + roldword(dword(a + ((b and d) or (c and (not d))) + Block^[9]  + $21e1cde6),  5);
  d := a + roldword(dword(d + ((a and c) or (b and (not c))) + Block^[14] + $c33707d6),  9);
  c := d + roldword(dword(c + ((d and b) or (a and (not b))) + Block^[3]  + $f4d50d87), 14);
  b := c + roldword(dword(b + ((c and a) or (d and (not a))) + Block^[8]  + $455a14ed), 20);
  a := b + roldword(dword(a + ((b and d) or (c and (not d))) + Block^[13] + $a9e3e905),  5);
  d := a + roldword(dword(d + ((a and c) or (b and (not c))) + Block^[2]  + $fcefa3f8),  9);
  c := d + roldword(dword(c + ((d and b) or (a and (not b))) + Block^[7]  + $676f02d9), 14);
  b := c + roldword(dword(b + ((c and a) or (d and (not a))) + Block^[12] + $8d2a4c8a), 20);
  // Round 3
  a := b + roldword(dword(a + (b xor c xor d) + Block^[5]  + $fffa3942),  4);
  d := a + roldword(dword(d + (a xor b xor c) + Block^[8]  + $8771f681), 11);
  c := d + roldword(dword(c + (d xor a xor b) + Block^[11] + $6d9d6122), 16);
  b := c + roldword(dword(b + (c xor d xor a) + Block^[14] + $fde5380c), 23);
  a := b + roldword(dword(a + (b xor c xor d) + Block^[1]  + $a4beea44),  4);
  d := a + roldword(dword(d + (a xor b xor c) + Block^[4]  + $4bdecfa9), 11);
  c := d + roldword(dword(c + (d xor a xor b) + Block^[7]  + $f6bb4b60), 16);
  b := c + roldword(dword(b + (c xor d xor a) + Block^[10] + $bebfbc70), 23);
  a := b + roldword(dword(a + (b xor c xor d) + Block^[13] + $289b7ec6),  4);
  d := a + roldword(dword(d + (a xor b xor c) + Block^[0]  + $eaa127fa), 11);
  c := d + roldword(dword(c + (d xor a xor b) + Block^[3]  + $d4ef3085), 16);
  b := c + roldword(dword(b + (c xor d xor a) + Block^[6]  + $04881d05), 23);
  a := b + roldword(dword(a + (b xor c xor d) + Block^[9]  + $d9d4d039),  4);
  d := a + roldword(dword(d + (a xor b xor c) + Block^[12] + $e6db99e5), 11);
  c := d + roldword(dword(c + (d xor a xor b) + Block^[15] + $1fa27cf8), 16);
  b := c + roldword(dword(b + (c xor d xor a) + Block^[2]  + $c4ac5665), 23);
  // Round 4
  a := b + roldword(dword(a + (c xor (b or (not d))) + Block^[0]  + $f4292244),  6);
  d := a + roldword(dword(d + (b xor (a or (not c))) + Block^[7]  + $432aff97), 10);
  c := d + roldword(dword(c + (a xor (d or (not b))) + Block^[14] + $ab9423a7), 15);
  b := c + roldword(dword(b + (d xor (c or (not a))) + Block^[5]  + $fc93a039), 21);
  a := b + roldword(dword(a + (c xor (b or (not d))) + Block^[12] + $655b59c3),  6);
  d := a + roldword(dword(d + (b xor (a or (not c))) + Block^[3]  + $8f0ccc92), 10);
  c := d + roldword(dword(c + (a xor (d or (not b))) + Block^[10] + $ffeff47d), 15);
  b := c + roldword(dword(b + (d xor (c or (not a))) + Block^[1]  + $85845dd1), 21);
  a := b + roldword(dword(a + (c xor (b or (not d))) + Block^[8]  + $6fa87e4f),  6);
  d := a + roldword(dword(d + (b xor (a or (not c))) + Block^[15] + $fe2ce6e0), 10);
  c := d + roldword(dword(c + (a xor (d or (not b))) + Block^[6]  + $a3014314), 15);
  b := c + roldword(dword(b + (d xor (c or (not a))) + Block^[13] + $4e0811a1), 21);
  a := b + roldword(dword(a + (c xor (b or (not d))) + Block^[4]  + $f7537e82),  6);
  d := a + roldword(dword(d + (b xor (a or (not c))) + Block^[11] + $bd3af235), 10);
  c := d + roldword(dword(c + (a xor (d or (not b))) + Block^[2]  + $2ad7d2bb), 15);
  b := c + roldword(dword(b + (d xor (c or (not a))) + Block^[9]  + $eb86d391), 21);


  inc(Context.State[0],a);
  inc(Context.State[1],b);
  inc(Context.State[2],c);
  inc(Context.State[3],d);
{$pop}
  inc(Context.Length,64);
end;
{$OPTIMIZATION DEFAULT}
{$ENDIF}
{$IF DEFINED(MD5PASCAL) or (NOT ((DEFINED(CPUX86_64)) or (DEFINED(CPUI386))))}
// Original version
procedure MD5Transform(var Context: TMDContext; Buffer: Pointer);

{$push}
{$r-,q-}

  procedure R1(var a: Cardinal; b,c,d,x: Cardinal; s: Byte; ac: Cardinal);
  // F(x,y,z) = (x and y) or ((not x) and z)
  begin
    a := b + roldword(dword(a + {F(b,c,d)}((b and c) or ((not b) and d)) + x + ac), s);
=======
=======
>>>>>>> origin/fixes_2_2
procedure MD5Transform(var Context: TMDContext; Buffer: Pointer);

  procedure R1(var a: Cardinal; b,c,d,x: Cardinal; s: Byte; ac: Cardinal);
  // F(x,y,z) = (x and y) or ((not x) and z)
  begin
    a := b + rol(a + {F(b,c,d)}((b and c) or ((not b) and d)) + x + ac, s);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  procedure R2(var a: Cardinal; b,c,d,x: Cardinal; s: Byte; ac: Cardinal);
  // G(x,y,z) = (x and z) or (y and (not z))
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    a := b + roldword(dword(a + {G(b,c,d)}((b and d) or (c and (not d))) + x + ac), s);
=======
    a := b + rol(a + {G(b,c,d)}((b and d) or (c and (not d))) + x + ac, s);
>>>>>>> graemeg/fixes_2_2
=======
    a := b + rol(a + {G(b,c,d)}((b and d) or (c and (not d))) + x + ac, s);
>>>>>>> origin/fixes_2_2
  end;

  procedure R3(var a: Cardinal; b,c,d,x: Cardinal; s: Byte; ac: Cardinal);
  // H(x,y,z) = x xor y xor z;
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    a := b + roldword(dword(a + {H(b,c,d)}(b xor c xor d) + x + ac), s);
=======
    a := b + rol(a + {H(b,c,d)}(b xor c xor d) + x + ac, s);
>>>>>>> graemeg/fixes_2_2
=======
    a := b + rol(a + {H(b,c,d)}(b xor c xor d) + x + ac, s);
>>>>>>> origin/fixes_2_2
  end;

  procedure R4(var a: Cardinal; b,c,d,x: Cardinal; s: Byte; ac: Cardinal);
  // I(x,y,z) = y xor (x or (not z));
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    a := b + roldword(dword(a + {I(b,c,d)}(c xor (b or (not d))) + x + ac), s);
  end;

{$pop}

=======
    a := b + rol(a + {I(b,c,d)}(c xor (b or (not d))) + x + ac, s);
  end;

>>>>>>> graemeg/fixes_2_2
=======
    a := b + rol(a + {I(b,c,d)}(c xor (b or (not d))) + x + ac, s);
  end;

>>>>>>> origin/fixes_2_2
var
  a, b, c, d: Cardinal;
  Block: array[0..15] of Cardinal;
begin
  Invert(Buffer, @Block, 64);
  a := Context.State[0];
  b := Context.State[1];
  c := Context.State[2];
  d := Context.State[3];

  // Round 1
  R1(a,b,c,d,Block[0] , 7,$d76aa478); R1(d,a,b,c,Block[1] ,12,$e8c7b756); R1(c,d,a,b,Block[2] ,17,$242070db); R1(b,c,d,a,Block[3] ,22,$c1bdceee);
  R1(a,b,c,d,Block[4] , 7,$f57c0faf); R1(d,a,b,c,Block[5] ,12,$4787c62a); R1(c,d,a,b,Block[6] ,17,$a8304613); R1(b,c,d,a,Block[7] ,22,$fd469501);
  R1(a,b,c,d,Block[8] , 7,$698098d8); R1(d,a,b,c,Block[9] ,12,$8b44f7af); R1(c,d,a,b,Block[10],17,$ffff5bb1); R1(b,c,d,a,Block[11],22,$895cd7be);
  R1(a,b,c,d,Block[12], 7,$6b901122); R1(d,a,b,c,Block[13],12,$fd987193); R1(c,d,a,b,Block[14],17,$a679438e); R1(b,c,d,a,Block[15],22,$49b40821);

  // Round 2
  R2(a,b,c,d,Block[1] , 5,$f61e2562); R2(d,a,b,c,Block[6] , 9,$c040b340); R2(c,d,a,b,Block[11],14,$265e5a51); R2(b,c,d,a,Block[0] ,20,$e9b6c7aa);
  R2(a,b,c,d,Block[5] , 5,$d62f105d); R2(d,a,b,c,Block[10], 9,$02441453); R2(c,d,a,b,Block[15],14,$d8a1e681); R2(b,c,d,a,Block[4] ,20,$e7d3fbc8);
  R2(a,b,c,d,Block[9] , 5,$21e1cde6); R2(d,a,b,c,Block[14], 9,$c33707d6); R2(c,d,a,b,Block[3] ,14,$f4d50d87); R2(b,c,d,a,Block[8] ,20,$455a14ed);
  R2(a,b,c,d,Block[13], 5,$a9e3e905); R2(d,a,b,c,Block[2] , 9,$fcefa3f8); R2(c,d,a,b,Block[7] ,14,$676f02d9); R2(b,c,d,a,Block[12],20,$8d2a4c8a);

  // Round 3
  R3(a,b,c,d,Block[5] , 4,$fffa3942); R3(d,a,b,c,Block[8] ,11,$8771f681); R3(c,d,a,b,Block[11],16,$6d9d6122); R3(b,c,d,a,Block[14],23,$fde5380c);
  R3(a,b,c,d,Block[1] , 4,$a4beea44); R3(d,a,b,c,Block[4] ,11,$4bdecfa9); R3(c,d,a,b,Block[7] ,16,$f6bb4b60); R3(b,c,d,a,Block[10],23,$bebfbc70);
  R3(a,b,c,d,Block[13], 4,$289b7ec6); R3(d,a,b,c,Block[0] ,11,$eaa127fa); R3(c,d,a,b,Block[3] ,16,$d4ef3085); R3(b,c,d,a,Block[6] ,23,$04881d05);
  R3(a,b,c,d,Block[9] , 4,$d9d4d039); R3(d,a,b,c,Block[12],11,$e6db99e5); R3(c,d,a,b,Block[15],16,$1fa27cf8); R3(b,c,d,a,Block[2] ,23,$c4ac5665);

  // Round 4
  R4(a,b,c,d,Block[0] , 6,$f4292244); R4(d,a,b,c,Block[7] ,10,$432aff97); R4(c,d,a,b,Block[14],15,$ab9423a7); R4(b,c,d,a,Block[5] ,21,$fc93a039);
  R4(a,b,c,d,Block[12], 6,$655b59c3); R4(d,a,b,c,Block[3] ,10,$8f0ccc92); R4(c,d,a,b,Block[10],15,$ffeff47d); R4(b,c,d,a,Block[1] ,21,$85845dd1);
  R4(a,b,c,d,Block[8] , 6,$6fa87e4f); R4(d,a,b,c,Block[15],10,$fe2ce6e0); R4(c,d,a,b,Block[6] ,15,$a3014314); R4(b,c,d,a,Block[13],21,$4e0811a1);
  R4(a,b,c,d,Block[4] , 6,$f7537e82); R4(d,a,b,c,Block[11],10,$bd3af235); R4(c,d,a,b,Block[2] ,15,$2ad7d2bb); R4(b,c,d,a,Block[9] ,21,$eb86d391);

<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$r-,q-}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  inc(Context.State[0],a);
  inc(Context.State[1],b);
  inc(Context.State[2],c);
  inc(Context.State[3],d);
<<<<<<< HEAD
<<<<<<< HEAD
{$pop}
  inc(Context.Length,64);
end;
{$ENDIF}


procedure MDInit(out Context: TMDContext; const Version: TMDVersion);
=======
=======
>>>>>>> origin/fixes_2_2
  inc(Context.Length,64);
end;


procedure MDInit(var Context: TMDContext; const Version: TMDVersion);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  FillChar(Context, Sizeof(TMDContext), 0);
  Context.Version := Version;

  case Version of

    MD_VERSION_4, MD_VERSION_5:
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        if Version = MD_VERSION_4 then
          Context.Hash := TMDHashFunc(@MD4Transform)
        else
          Context.Hash := TMDHashFunc(@MD5Transform);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        Context.Align := 64;
        Context.State[0] := $67452301;
        Context.State[1] := $efcdab89;
        Context.State[2] := $98badcfe;
        Context.State[3] := $10325476;
        Context.Length := 0;
        Context.BufCnt := 0;
      end;

    MD_VERSION_2:
      begin
        Context.Align := 16;
<<<<<<< HEAD
<<<<<<< HEAD
        Context.Hash := TMDHashFunc(@MD2Transform)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;

  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure MDUpdate(var Context: TMDContext; var Buf; const BufLen: PtrUInt); [public,alias:'MD_UPDATE'];
=======
procedure MDUpdate(var Context: TMDContext; var Buf; const BufLen: PtrUInt);
>>>>>>> graemeg/fixes_2_2
=======
procedure MDUpdate(var Context: TMDContext; var Buf; const BufLen: PtrUInt);
>>>>>>> origin/fixes_2_2
var
  Align: PtrUInt;
  Src: Pointer;
  Num: PtrUInt;
begin
  if BufLen = 0 then
    Exit;

  Align := Context.Align;
  Src := @Buf;
  Num := 0;

  // 1. Transform existing data in buffer
  if Context.BufCnt > 0 then
  begin
    // 1.1 Try to fill buffer to "Align" bytes
    Num := Align - Context.BufCnt;
    if Num > BufLen then
      Num := BufLen;

    Move(Src^, Context.Buffer[Context.BufCnt], Num);
    Context.BufCnt := Context.BufCnt + Num;
    Src := Pointer(PtrUInt(Src) + Num);

    // 1.2 If buffer contains "Align" bytes, transform it
    if Context.BufCnt = Align then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
      Context.Hash(@Context, @Context.Buffer);
=======
=======
>>>>>>> origin/fixes_2_2
      case Context.Version of
        MD_VERSION_2: MD2Transform(Context, @Context.Buffer);
        MD_VERSION_4: MD4Transform(Context, @Context.Buffer);
        MD_VERSION_5: MD5Transform(Context, @Context.Buffer);
      end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      Context.BufCnt := 0;
    end;
  end;

  // 2. Transform "Align"-Byte blocks of Buf
  Num := BufLen - Num;
  while Num >= Align do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    Context.Hash(@Context, Src);
=======
=======
>>>>>>> origin/fixes_2_2
    case Context.Version of
      MD_VERSION_2: MD2Transform(Context, Src);
      MD_VERSION_4: MD4Transform(Context, Src);
      MD_VERSION_5: MD5Transform(Context, Src);
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Src := Pointer(PtrUInt(Src) + Align);
    Num := Num - Align;
  end;

  // 3. If there's a block smaller than "Align" Bytes left, add it to buffer
  if Num > 0 then
  begin
    Context.BufCnt := Num;
    Move(Src^, Context.Buffer, Num);
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure MDFinal(var Context: TMDContext; out Digest: TMDDigest); [public,alias:'MD_FINAL'];
=======
procedure MDFinal(var Context: TMDContext; var Digest: TMDDigest);
>>>>>>> graemeg/fixes_2_2
=======
procedure MDFinal(var Context: TMDContext; var Digest: TMDDigest);
>>>>>>> origin/fixes_2_2
const
{$ifdef FPC_BIG_ENDIAN}
  PADDING_MD45: array[0..15] of Cardinal = ($80000000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
{$else FPC_BIG_ENDIAN}
  PADDING_MD45: array[0..15] of Cardinal = ($80,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
{$endif FPC_BIG_ENDIAN}
var
  Length: QWord;
  Pads: Cardinal;
begin
  case Context.Version of

    MD_VERSION_4, MD_VERSION_5:
      begin
        // 1. Compute length of the whole stream in bits
        Length := 8 * (Context.Length + Context.BufCnt);

        // 2. Append padding bits
        if Context.BufCnt >= 56 then
          Pads := 120 - Context.BufCnt
        else
          Pads := 56 - Context.BufCnt;
        MDUpdate(Context, PADDING_MD45, Pads);

        // 3. Append length of the stream
        Length := NtoLE(Length);
        MDUpdate(Context, Length, 8);

        // 4. Invert state to digest
        Invert(@Context.State, @Digest, 16);
      end;

    MD_VERSION_2:
      begin
        Pads := 16 - Context.BufCnt;
        Length := NtoLE(QWord(Pads));
        while Pads > 0 do
        begin
          MDUpdate(Context, Length, 1);
          Dec(Pads);
        end;
        MDUpdate(Context, Context.Checksum, 16);
        Move(Context.State, Digest, 16);
      end;

  end;

  FillChar(Context, SizeOf(TMDContext), 0);
end;

function MDString(const S: String; const Version: TMDVersion): TMDDigest;
var
  Context: TMDContext;
begin
  MDInit(Context, Version);
  MDUpdate(Context, PChar(S)^, length(S));
  MDFinal(Context, Result);
end;

function MDBuffer(var Buf; const BufLen: PtrUInt; const Version: TMDVersion): TMDDigest;
var
  Context: TMDContext;
begin
  MDInit(Context, Version);
  MDUpdate(Context, buf, buflen);
  MDFinal(Context, Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function MDFile(const Filename: RawByteString; const Version: TMDVersion; const BufSize: PtrUInt): TMDDigest;
var
  F: File;
  Buf: Pchar;
  Context: TMDContext;
  Count: Cardinal;
  ofm: Longint;
begin
  MDInit(Context, Version);

  Assign(F, Filename);
  {$push}{$i-}
  ofm := FileMode;
  FileMode := 0;
  Reset(F, 1);
  {$pop}

  if IOResult = 0 then
  begin
    GetMem(Buf, BufSize);
    repeat
      BlockRead(F, Buf^, Bufsize, Count);
      if Count > 0 then
        MDUpdate(Context, Buf^, Count);
    until Count < BufSize;
    FreeMem(Buf, BufSize);
    Close(F);
  end;

  MDFinal(Context, Result);
  FileMode := ofm;
end;

function MDFile(const Filename: UnicodeString; const Version: TMDVersion; const BufSize: PtrUInt): TMDDigest;
=======
function MDFile(const Filename: String; const Version: TMDVersion; const BufSize: PtrUInt): TMDDigest;
>>>>>>> graemeg/fixes_2_2
=======
function MDFile(const Filename: String; const Version: TMDVersion; const BufSize: PtrUInt): TMDDigest;
>>>>>>> origin/fixes_2_2
var
  F: File;
  Buf: Pchar;
  Context: TMDContext;
  Count: Cardinal;
  ofm: Longint;
begin
  MDInit(Context, Version);

  Assign(F, Filename);
<<<<<<< HEAD
<<<<<<< HEAD
  {$push}{$i-}
  ofm := FileMode;
  FileMode := 0;
  Reset(F, 1);
  {$pop}
=======
=======
>>>>>>> origin/fixes_2_2
  {$i-}
  ofm := FileMode;
  FileMode := 0;
  Reset(F, 1);
  {$i+}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  if IOResult = 0 then
  begin
    GetMem(Buf, BufSize);
    repeat
      BlockRead(F, Buf^, Bufsize, Count);
      if Count > 0 then
        MDUpdate(Context, Buf^, Count);
    until Count < BufSize;
    FreeMem(Buf, BufSize);
    Close(F);
  end;

  MDFinal(Context, Result);
  FileMode := ofm;
end;

function MDPrint(const Digest: TMDDigest): String;
var
  I: Byte;
begin
  Result := '';
  for I := 0 to 15 do
    Result := Result + HexStr(Digest[i],2);
  Result := LowerCase(Result);
end;

function MDMatch(const Digest1, Digest2: TMDDigest): Boolean;
var
  A: array[0..3] of Cardinal absolute Digest1;
  B: array[0..3] of Cardinal absolute Digest2;
begin
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$B+}
  Result := (A[0] = B[0]) and (A[1] = B[1]) and (A[2] = B[2]) and (A[3] = B[3]);
{$pop}
end;

procedure MD2Init(out Context: TMD2Context);
=======
=======
>>>>>>> origin/fixes_2_2
  Result := (A[0] = B[0]) and (A[1] = B[1]) and (A[2] = B[2]) and (A[3] = B[3]);
end;

procedure MD2Init(var Context: TMD2Context);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  MDInit(Context, MD_VERSION_2);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure MD4Init(out Context: TMD4Context);
=======
=======
>>>>>>> origin/fixes_2_2
procedure MD2Update(var Context: TMD2Context; var Buf; const BufLen: PtrUInt);
begin
  MDUpdate(Context, Buf, BufLen);
end;

procedure MD2Final(var Context: TMD2Context; var Digest: TMD2Digest);
begin
  MDFinal(Context, Digest);
end;

procedure MD4Init(var Context: TMD4Context);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  MDInit(Context, MD_VERSION_4);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure MD5Init(out Context: TMD5Context);
=======
=======
>>>>>>> origin/fixes_2_2
procedure MD4Update(var Context: TMD4Context; var Buf; const BufLen: PtrUInt);
begin
  MDUpdate(Context, Buf, BufLen);
end;

procedure MD4Final(var Context: TMD4Context; var Digest: TMD4Digest);
begin
  MDFinal(Context, Digest);
end;

procedure MD5Init(var Context: TMD5Context);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  MDInit(Context, MD_VERSION_5);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
procedure MD5Update(var Context: TMD5Context; var Buf; const BufLen: PtrUInt);
begin
  MDUpdate(Context, Buf, BufLen);
end;

procedure MD5Final(var Context: TMD5Context; var Digest: TMD5Digest);
begin
  MDFinal(Context, Digest);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MD2String(const S: String): TMD2Digest;
begin
  Result := MDString(S, MD_VERSION_2);
end;

function MD2Buffer(var Buf; const BufLen: PtrUInt): TMD2Digest;
begin
  Result := MDBuffer(Buf, BufLen, MD_VERSION_2);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function MD2File(const Filename: RawByteString; const Bufsize: PtrUInt): TMD2Digest;
begin
  Result := MDFile(Filename, MD_VERSION_2, Bufsize);
end;

function MD2File(const Filename: UnicodeString; const Bufsize: PtrUInt): TMD2Digest;
=======
function MD2File(const Filename: String; const Bufsize: PtrUInt): TMD2Digest;
>>>>>>> graemeg/fixes_2_2
=======
function MD2File(const Filename: String; const Bufsize: PtrUInt): TMD2Digest;
>>>>>>> origin/fixes_2_2
begin
  Result := MDFile(Filename, MD_VERSION_2, Bufsize);
end;

function MD4String(const S: String): TMD4Digest;
begin
  Result := MDString(S, MD_VERSION_4);
end;

function MD4Buffer(var Buf; const BufLen: PtrUInt): TMD4Digest;
begin
  Result := MDBuffer(Buf, BufLen, MD_VERSION_4);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function MD4File(const Filename: RawByteString; const Bufsize: PtrUInt): TMD4Digest;
begin
  Result := MDFile(Filename, MD_VERSION_4, Bufsize);
end;

function MD4File(const Filename: UnicodeString; const Bufsize: PtrUInt): TMD4Digest;
=======
function MD4File(const Filename: String; const Bufsize: PtrUInt): TMD4Digest;
>>>>>>> graemeg/fixes_2_2
=======
function MD4File(const Filename: String; const Bufsize: PtrUInt): TMD4Digest;
>>>>>>> origin/fixes_2_2
begin
  Result := MDFile(Filename, MD_VERSION_4, Bufsize);
end;

function MD5String(const S: String): TMD5Digest;
begin
  Result := MDString(S, MD_VERSION_5);
end;

function MD5Buffer(var Buf; const BufLen: PtrUInt): TMD5Digest;
begin
  Result := MDBuffer(Buf, BufLen, MD_VERSION_5);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function MD5File(const Filename: RawByteString; const Bufsize: PtrUInt): TMD5Digest;
begin
  Result := MDFile(Filename, MD_VERSION_5, Bufsize);
end;

function MD5File(const Filename: UnicodeString; const Bufsize: PtrUInt): TMD5Digest;
=======
function MD5File(const Filename: String; const Bufsize: PtrUInt): TMD5Digest;
>>>>>>> graemeg/fixes_2_2
=======
function MD5File(const Filename: String; const Bufsize: PtrUInt): TMD5Digest;
>>>>>>> origin/fixes_2_2
begin
  Result := MDFile(Filename, MD_VERSION_5, Bufsize);
end;

function MD2Print(const Digest: TMD2Digest): String;
begin
  Result := MDPrint(Digest);
end;

function MD2Match(const Digest1, Digest2: TMD2Digest): Boolean;
begin
  Result := MDMatch(Digest1, Digest2);
end;

function MD4Print(const Digest: TMD4Digest): String;
begin
  Result := MDPrint(Digest);
end;

function MD4Match(const Digest1, Digest2: TMD4Digest): Boolean;
begin
  Result := MDMatch(Digest1, Digest2);
end;

function MD5Print(const Digest: TMD5Digest): String;
begin
  Result := MDPrint(Digest);
end;

function MD5Match(const Digest1, Digest2: TMD5Digest): Boolean;
begin
  Result := MDMatch(Digest1, Digest2);
end;

<<<<<<< HEAD
<<<<<<< HEAD
//convert the String representation of a digest to a TMDDigest
//on error all fields are set to $00
function StrtoMD5(const MD5String:String):TMDDigest;
   var I: Byte;
       t: longint;
       f: boolean;
   begin
     f:= Length(MD5String) = 32;
     if f then
      for I := 0 to 15 do
       begin
        f:= f and TryStrToInt('$'+copy(MD5String,i*2+1, 2), t);
        Result[I]:= t;
       end;
     if not f then
       FillChar(Result, Sizeof(Result), 0);
   end; 
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
