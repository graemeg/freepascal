{ %CPU=x86_64 }
Program AESTest;
{$INLINE ON}
{$ASMMODE INTEL}
{$MODE DELPHI}

Uses Cpu,SysUtils;

Type
  Word32 = LongWord;
  Word64 = QWord;
  WordPR = Word64;

  PAESKey256 = ^TAESKey256; TAESKey256 = Array [0..3] of Word64;

  PAESOpenedKey = ^TAESOpenedKey; TAESOpenedKey = Packed Record
    EnCryptRoundKeys    : Array [0..14, 0..3] Of Word32;
    Padding0            : Array [0..3] of Word32;
    DeCryptRoundKeys    : Array [0..14, 0..3] Of Word32;
    Padding1            : Array [0..3] of Word32;
  End;

Const
  Test_Key     : Array[0..3] of Word64 = ($0706050403020100, $0f0e0d0c0b0a0908, $1716151413121110, $1f1e1d1c1b1a1918);
  Test_Data    : Array[0..1] of Word64 = (Word64($7766554433221100), Word64($ffeeddccbbaa9988));
  Test_Crypt   : Array[0..1] of Word64 = (Word64($bf456751cab7a28e), Word64($8960494b9049fcea));

Var
  OpenedKey   : TAESOpenedKey;
  Data        : Array [0..1] of Word64;
  Passed      : Boolean;

Procedure OpenKey_AES(Key: PAESKey256; OpenedKey: PAESOpenedKey); Assembler; NoStackFrame;
  Procedure key_expansion; Assembler; NoStackFrame;
  Asm
   MOV RDX, RCX
   PSHUFD XMM2, XMM2, 011111111b; PXOR XMM2, XMM1; MOVD EAX, XMM2; MOV [RCX], EAX; ADD RCX, 4
   PSHUFD XMM1, XMM1, 011100101b; MOVD EBX, XMM1; XOR EAX, EBX; MOV [RCX], EAX; ADD RCX, 4
   PSHUFD XMM1, XMM1, 011100110b; MOVD EBX, XMM1; XOR EAX, EBX; MOV [RCX], EAX; ADD RCX, 4
   PSHUFD XMM1, XMM1, 011100111b; MOVD EBX, XMM1; XOR EAX, EBX; MOV [RCX], EAX; ADD RCX, 4
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   MOVDQU XMM4, [RDX]; AESKEYGENASSIST XMM4, XMM4, 0
=======
   MOVDQU XMM4, [RDX]; AESKEYGEN XMM4, XMM4, 0
>>>>>>> graemeg/cpstrnew
=======
   MOVDQU XMM4, [RDX]; AESKEYGEN XMM4, XMM4, 0
>>>>>>> graemeg/cpstrnew
=======
   MOVDQU XMM4, [RDX]; AESKEYGEN XMM4, XMM4, 0
>>>>>>> graemeg/cpstrnew
=======
   MOVDQU XMM4, [RDX]; AESKEYGEN XMM4, XMM4, 0
>>>>>>> origin/cpstrnew
   PSHUFD XMM4, XMM4, 011100110b; MOVD EAX, XMM4; MOVD EBX, XMM3; XOR EAX, EBX; MOV [RCX], EAX; ADD RCX, 4
   PSHUFD XMM3, XMM3, 011100101b; MOVD EBX, XMM3; XOR EAX, EBX; MOV [RCX], EAX; ADD RCX, 4
   PSHUFD XMM3, XMM3, 011100110b; MOVD EBX, XMM3; XOR EAX, EBX; MOV [RCX], EAX; ADD RCX, 4
   PSHUFD XMM3, XMM3, 011100111b; MOVD EBX, XMM3; XOR EAX, EBX; MOV [RCX], EAX; ADD RCX, 4
   MOVDQU XMM1, [RDX]; ADD RDX, $10; MOVDQU XMM3, [RDX]
  End;
Asm
 PUSH RBX
{$ifndef win64}
 // fix parameter locations
 MOV RDX,RSI
 MOV RCX,RDI 
{$endif win64}
 MOV R8, RDX
 MOVDQU XMM1, [RCX]; MOVDQU XMM3, [RCX+16]
 MOVDQU [RDX], XMM1; MOVDQU [RDX + $10], XMM3
 LEA RCX, [RDX+$20]
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 AESKEYGENASSIST XMM2, XMM3, $1;  CALL key_expansion
 AESKEYGENASSIST XMM2, XMM3, $2;  CALL key_expansion
 AESKEYGENASSIST XMM2, XMM3, $4;  CALL key_expansion
 AESKEYGENASSIST XMM2, XMM3, $8;  CALL key_expansion
 AESKEYGENASSIST XMM2, XMM3, $10; CALL key_expansion
 AESKEYGENASSIST XMM2, XMM3, $20; CALL key_expansion
 AESKEYGENASSIST XMM2, XMM3, $40; CALL key_expansion
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 AESKEYGEN XMM2, XMM3, $1;  CALL key_expansion
 AESKEYGEN XMM2, XMM3, $2;  CALL key_expansion
 AESKEYGEN XMM2, XMM3, $4;  CALL key_expansion
 AESKEYGEN XMM2, XMM3, $8;  CALL key_expansion
 AESKEYGEN XMM2, XMM3, $10; CALL key_expansion
 AESKEYGEN XMM2, XMM3, $20; CALL key_expansion
 AESKEYGEN XMM2, XMM3, $40; CALL key_expansion
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
 MOVDQU XMM0,  [R8+$00]; MOVDQU XMM1,  [R8+$10]; MOVDQU XMM2,  [R8+$20]; MOVDQU XMM3,  [R8+$30]
 MOVDQU XMM4,  [R8+$40]; MOVDQU XMM5,  [R8+$50]; MOVDQU XMM6,  [R8+$60]; MOVDQU XMM7,  [R8+$70]
 MOVDQU XMM8,  [R8+$80]; MOVDQU XMM9,  [R8+$90]; MOVDQU XMM10, [R8+$A0]; MOVDQU XMM11, [R8+$B0]
 MOVDQU XMM12, [R8+$C0]; MOVDQU XMM13, [R8+$D0]; MOVDQU XMM14, [R8+$E0]
 AESIMC XMM1,  XMM1; AESIMC XMM2,  XMM2;  AESIMC XMM3,  XMM3;  AESIMC XMM4,  XMM4
 AESIMC XMM5,  XMM5; AESIMC XMM6,  XMM6;  AESIMC XMM7,  XMM7;  AESIMC XMM8,  XMM8
 AESIMC XMM9,  XMM9; AESIMC XMM10, XMM10; AESIMC XMM11, XMM11; AESIMC XMM12, XMM12
 AESIMC XMM13, XMM13
 MOVDQU [R8+$100], XMM0;  MOVDQU [R8+$110], XMM1;  MOVDQU [R8+$120], XMM2;  MOVDQU [R8+$130], XMM3
 MOVDQU [R8+$140], XMM4;  MOVDQU [R8+$150], XMM5;  MOVDQU [R8+$160], XMM6;  MOVDQU [R8+$170], XMM7
 MOVDQU [R8+$180], XMM8;  MOVDQU [R8+$190], XMM9;  MOVDQU [R8+$1A0], XMM10; MOVDQU [R8+$1B0], XMM11
 MOVDQU [R8+$1C0], XMM12; MOVDQU [R8+$1D0], XMM13; MOVDQU [R8+$1E0], XMM14
 POP RBX
End;

Procedure EnCrypt_AES(InData, OutData: Pointer; DataSize: WordPR; EnCryptRoundKeys: Pointer); Assembler; NoStackFrame;
Asm
{$ifndef win64}
 // fix parameter locations
 MOV R9,RCX
 MOV R8,RDX
 MOV RDX,RSI
 MOV RCX,RDI
{$endif win64}
 // Loading encryption keys
 MOVDQU XMM0, [R9+16*0]
 MOVDQU XMM1, [R9+16*1]
 MOVDQU XMM2, [R9+16*2]
 MOVDQU XMM3, [R9+16*3]
 MOVDQU XMM4, [R9+16*4]
 MOVDQU XMM5, [R9+16*5]
 MOVDQU XMM6, [R9+16*6]
 MOVDQU XMM7, [R9+16*7]
 MOVDQU XMM8, [R9+16*8]
 MOVDQU XMM9, [R9+16*9]
 MOVDQU XMM10, [R9+16*10]
 MOVDQU XMM11, [R9+16*11]
 MOVDQU XMM12, [R9+16*12]
 MOVDQU XMM13, [R9+16*13]
 MOVDQU XMM14, [R9+16*14]
 // Setting the main loop
 XCHG RCX, R8
 SHR RCX, 4
@Loop: MOVDQU XMM15, [R8]; ADD R8, 16
       PXOR XMM15, XMM0
       AESENC XMM15, XMM1
       AESENC XMM15, XMM2
       AESENC XMM15, XMM3
       AESENC XMM15, XMM4
       AESENC XMM15, XMM5
       AESENC XMM15, XMM6
       AESENC XMM15, XMM7
       AESENC XMM15, XMM8
       AESENC XMM15, XMM9
       AESENC XMM15, XMM10
       AESENC XMM15, XMM11
       AESENC XMM15, XMM12
       AESENC XMM15, XMM13
       AESENCLAST XMM15, XMM14
       MOVDQU [RDX], XMM15; ADD RDX, 16
       LOOP @Loop
End;

Procedure DeCrypt_AES(InData, OutData: Pointer; DataSize: WordPR; DeCryptRoundKeys: Pointer); Assembler; NoStackFrame;
Asm
{$ifndef win64}
 // fix parameter locations
 MOV R9,RCX
 MOV R8,RDX
 MOV RDX,RSI
 MOV RCX,RDI  
{$endif win64}
 // Loading decryption keys
 MOVDQU XMM0, [R9+16*0]
 MOVDQU XMM1, [R9+16*1]
 MOVDQU XMM2, [R9+16*2]
 MOVDQU XMM3, [R9+16*3]
 MOVDQU XMM4, [R9+16*4]
 MOVDQU XMM5, [R9+16*5]
 MOVDQU XMM6, [R9+16*6]
 MOVDQU XMM7, [R9+16*7]
 MOVDQU XMM8, [R9+16*8]
 MOVDQU XMM9, [R9+16*9]
 MOVDQU XMM10, [R9+16*10]
 MOVDQU XMM11, [R9+16*11]
 MOVDQU XMM12, [R9+16*12]
 MOVDQU XMM13, [R9+16*13]
 MOVDQU XMM14, [R9+16*14]
 // Setting the main loop
 XCHG RCX, R8
 SHR RCX, 4
@Loop: MOVDQU XMM15, [R8]; ADD R8, 16
       PXOR XMM15, XMM14
       AESDEC XMM15, XMM13
       AESDEC XMM15, XMM12
       AESDEC XMM15, XMM11
       AESDEC XMM15, XMM10
       AESDEC XMM15, XMM9
       AESDEC XMM15, XMM8
       AESDEC XMM15, XMM7
       AESDEC XMM15, XMM6
       AESDEC XMM15, XMM5
       AESDEC XMM15, XMM4
       AESDEC XMM15, XMM3
       AESDEC XMM15, XMM2
       AESDEC XMM15, XMM1
       AESDECLAST XMM15, XMM0
       MOVDQU [RDX], XMM15; ADD RDX, 16
       LOOP @Loop
End;

BEGIN
  if AESSupport then
    begin
      OpenKey_AES(@Test_Key, @OpenedKey);
      EnCrypt_AES(@Test_Data, @Data, 16, @OpenedKey.EnCryptRoundKeys);
      Passed := SysUtils.CompareMem(@Data, @Test_Crypt, 16);
      DeCrypt_AES(@Data, @Data, 16, @OpenedKey.DeCryptRoundKeys);
      Passed := Passed and SysUtils.CompareMem(@Data, @Test_Data, 16);
      If Not Passed Then Halt(1);
      writeln('ok');
    end 
  else
    writeln('CPU has no AES instruction support');
END.
