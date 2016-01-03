unit crc;

{
  crc32.c -- compute the CRC-32 of a data stream
  Copyright (C) 1995-1998 Mark Adler

  Pascal tranlastion
  Copyright (C) 1998 by Jacques Nomssi Nzali
  For conditions of distribution and use, see copyright notice in readme.txt
}

interface

function crc32(crc : cardinal; buf : Pbyte; len : cardinal) : cardinal;

{  Update a running crc with the bytes buf[0..len-1] and return the updated
   crc. If buf is NULL, this function returns the required initial value
   for the crc. Pre- and post-conditioning (one's complement) is performed
   within this function so it shouldn't be done by the application.
   Usage example:

    var
      crc : cardinal;
    begin
      crc := crc32(0, nil, 0);

      while (read_buffer(buffer, length) <> EOF) do
        crc := crc32(crc, buffer, length);

      if (crc <> original_crc) then error();
    end;

}

function get_crc_table : Pcardinal;  { can be used by asm versions of crc32() }


implementation

{$IFDEF DYNAMIC_CRC_TABLE}

{local}
const
  crc_table_empty : boolean = TRUE;
{local}
var
  crc_table : array[0..256-1] of uLongf;


{
  Generate a table for a byte-wise 32-bit CRC calculation on the polynomial:
  x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1.

  Polynomials over GF(2) are represented in binary, one bit per coefficient,
  with the lowest powers in the most significant bit.  Then adding polynomials
  is just exclusive-or, and multiplying a polynomial by x is a right shift by
  one.  If we call the above polynomial p, and represent a byte as the
  polynomial q, also with the lowest power in the most significant bit (so the
  byte 0xb1 is the polynomial x^7+x^3+x+1), then the CRC is (q*x^32) mod p,
  where a mod b means the remainder after dividing a by b.

  This calculation is done using the shift-register method of multiplying and
  taking the remainder.  The register is initialized to zero, and for each
  incoming bit, x^32 is added mod p to the register if the bit is a one (where
  x^32 mod p is p+x^32 = x^26+...+1), and the register is multiplied mod p by
  x (which is shifting right by one and adding x^32 mod p if the bit shifted
  out is a one).  We start with the highest power (least significant bit) of
  q and repeat for all eight bits of q.

  The table is simply the CRC of all possible eight bit values.  This is all
  the information needed to generate CRC's on data a byte at a time for all
  combinations of CRC register values and incoming bytes.
}
{local}
procedure make_crc_table;
var
 c    : cardinal;
 n,k  : integer;
 poly : cardinal; { polynomial exclusive-or pattern }

const
 { terms of polynomial defining this crc (except x^32): }
 p: array [0..13] of Byte = (0,1,2,4,5,7,8,10,11,12,16,22,23,26);

begin
  { make exclusive-or pattern from polynomial ($EDB88320) }
  poly := longint(0);
  for n := 0 to (sizeof(p) div sizeof(Byte))-1 do
    poly := poly or (longint(1) shl (31 - p[n]));

  for n := 0 to 255 do
  begin
    c := cardinal(n);
    for k := 0 to 7 do
    begin
      if (c and 1) <> 0 then
        c := poly xor (c shr 1)
      else
        c := (c shr 1);
    end;
    crc_table[n] := c;
  end;
  crc_table_empty := FALSE;
end;

{$ELSE}

{ ========================================================================
  Table of CRC-32's of all single-byte values (made by make_crc_table) }

{local}
const
  crc_table : array[0..256-1] of cardinal = (
  $00000000, $77073096, $ee0e612c, $990951ba, $076dc419,
  $706af48f, $e963a535, $9e6495a3, $0edb8832, $79dcb8a4,
  $e0d5e91e, $97d2d988, $09b64c2b, $7eb17cbd, $e7b82d07,
  $90bf1d91, $1db71064, $6ab020f2, $f3b97148, $84be41de,
  $1adad47d, $6ddde4eb, $f4d4b551, $83d385c7, $136c9856,
  $646ba8c0, $fd62f97a, $8a65c9ec, $14015c4f, $63066cd9,
  $fa0f3d63, $8d080df5, $3b6e20c8, $4c69105e, $d56041e4,
  $a2677172, $3c03e4d1, $4b04d447, $d20d85fd, $a50ab56b,
  $35b5a8fa, $42b2986c, $dbbbc9d6, $acbcf940, $32d86ce3,
  $45df5c75, $dcd60dcf, $abd13d59, $26d930ac, $51de003a,
  $c8d75180, $bfd06116, $21b4f4b5, $56b3c423, $cfba9599,
  $b8bda50f, $2802b89e, $5f058808, $c60cd9b2, $b10be924,
  $2f6f7c87, $58684c11, $c1611dab, $b6662d3d, $76dc4190,
  $01db7106, $98d220bc, $efd5102a, $71b18589, $06b6b51f,
  $9fbfe4a5, $e8b8d433, $7807c9a2, $0f00f934, $9609a88e,
  $e10e9818, $7f6a0dbb, $086d3d2d, $91646c97, $e6635c01,
  $6b6b51f4, $1c6c6162, $856530d8, $f262004e, $6c0695ed,
  $1b01a57b, $8208f4c1, $f50fc457, $65b0d9c6, $12b7e950,
  $8bbeb8ea, $fcb9887c, $62dd1ddf, $15da2d49, $8cd37cf3,
  $fbd44c65, $4db26158, $3ab551ce, $a3bc0074, $d4bb30e2,
  $4adfa541, $3dd895d7, $a4d1c46d, $d3d6f4fb, $4369e96a,
  $346ed9fc, $ad678846, $da60b8d0, $44042d73, $33031de5,
  $aa0a4c5f, $dd0d7cc9, $5005713c, $270241aa, $be0b1010,
  $c90c2086, $5768b525, $206f85b3, $b966d409, $ce61e49f,
  $5edef90e, $29d9c998, $b0d09822, $c7d7a8b4, $59b33d17,
  $2eb40d81, $b7bd5c3b, $c0ba6cad, $edb88320, $9abfb3b6,
  $03b6e20c, $74b1d29a, $ead54739, $9dd277af, $04db2615,
  $73dc1683, $e3630b12, $94643b84, $0d6d6a3e, $7a6a5aa8,
  $e40ecf0b, $9309ff9d, $0a00ae27, $7d079eb1, $f00f9344,
  $8708a3d2, $1e01f268, $6906c2fe, $f762575d, $806567cb,
  $196c3671, $6e6b06e7, $fed41b76, $89d32be0, $10da7a5a,
  $67dd4acc, $f9b9df6f, $8ebeeff9, $17b7be43, $60b08ed5,
  $d6d6a3e8, $a1d1937e, $38d8c2c4, $4fdff252, $d1bb67f1,
  $a6bc5767, $3fb506dd, $48b2364b, $d80d2bda, $af0a1b4c,
  $36034af6, $41047a60, $df60efc3, $a867df55, $316e8eef,
  $4669be79, $cb61b38c, $bc66831a, $256fd2a0, $5268e236,
  $cc0c7795, $bb0b4703, $220216b9, $5505262f, $c5ba3bbe,
  $b2bd0b28, $2bb45a92, $5cb36a04, $c2d7ffa7, $b5d0cf31,
  $2cd99e8b, $5bdeae1d, $9b64c2b0, $ec63f226, $756aa39c,
  $026d930a, $9c0906a9, $eb0e363f, $72076785, $05005713,
  $95bf4a82, $e2b87a14, $7bb12bae, $0cb61b38, $92d28e9b,
  $e5d5be0d, $7cdcefb7, $0bdbdf21, $86d3d2d4, $f1d4e242,
  $68ddb3f8, $1fda836e, $81be16cd, $f6b9265b, $6fb077e1,
  $18b74777, $88085ae6, $ff0f6a70, $66063bca, $11010b5c,
  $8f659eff, $f862ae69, $616bffd3, $166ccf45, $a00ae278,
  $d70dd2ee, $4e048354, $3903b3c2, $a7672661, $d06016f7,
  $4969474d, $3e6e77db, $aed16a4a, $d9d65adc, $40df0b66,
  $37d83bf0, $a9bcae53, $debb9ec5, $47b2cf7f, $30b5ffe9,
  $bdbdf21c, $cabac28a, $53b39330, $24b4a3a6, $bad03605,
  $cdd70693, $54de5729, $23d967bf, $b3667a2e, $c4614ab8,
  $5d681b02, $2a6f2b94, $b40bbe37, $c30c8ea1, $5a05df1b,
  $2d02ef8d);

{$ENDIF}

{ =========================================================================
  This function can be used by asm versions of crc32() }

function get_crc_table : {const} Pcardinal;
begin
{$ifdef DYNAMIC_CRC_TABLE}
  if (crc_table_empty) then
    make_crc_table;
{$endif}
  get_crc_table :=  {const} Pcardinal(@crc_table);
end;

{ ========================================================================= }

function crc32 (crc : cardinal; buf : Pbyte; len : cardinal): cardinal;
begin
  if (buf = nil) then
    crc32 := 0
  else
  begin

{$IFDEF DYNAMIC_CRC_TABLE}
    if crc_table_empty then
      make_crc_table;
{$ENDIF}

    crc := crc xor cardinal($ffffffff);
    while (len >= 8) do
    begin
      {DO8(buf)}
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);

      dec(len, 8);
    end;
    if (len <> 0) then
    repeat
      {DO1(buf)}
      crc := crc_table[(integer(crc) xor buf^) and $ff] xor (crc shr 8);
      inc(buf);

      dec(len);
    until (len = 0);
    crc32 := crc xor cardinal($ffffffff);
  end;
end;


end.