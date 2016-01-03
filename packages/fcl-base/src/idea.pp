{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by Michael Van Canneyt and Florian Klaempfl

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ifdef fpc}
{$mode objfpc}
{$endif}

Unit idea;

{
 IDEA encryption routines for pascal
 ported from PGP 2.3

 IDEA encryption routines for pascal, ported from PGP 2.3
 Copyright (C) for this port 1998 Ingo Korb
 Copyright (C) for the stream support 1999 Michael Van Canneyt

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License as published by the Free Software Foundation; either
 version 2 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.

 You should have received a copy of the GNU Library General Public
 License along with this library; if not, write to the Free
 Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
}


{$R-,Q-}
{ Not nice but fast... }

INTERFACE

Uses Sysutils,Classes;

CONST 
  IDEAKEYSIZE   = 16;
  IDEABLOCKSIZE = 8;
  ROUNDS        = 8;
  KEYLEN        = (6*ROUNDS+4);

TYPE 
  TIDEAKey       = ARRAY[0..keylen-1] OF Word;
  TIdeaCryptKey  = ARRAY[0..7] OF Word;
  TIdeaCryptData = ARRAY[0..3] OF Word;
  
  { For backward compatibility }
  IDEAkey = TIDEAkey;
  IdeaCryptKey = TIdeaCryptKey;
  IdeaCryptData = TIdeaCryptData;
  
<<<<<<< HEAD
<<<<<<< HEAD
PROCEDURE EnKeyIdea(UserKey: TIdeacryptkey; OUT z: TIDEAKey);
PROCEDURE DeKeyIdea(z: TIDEAKey; OUT dk: TIDEAKey);
PROCEDURE CipherIdea(Input: TIDEACryptData; OUT outdata: TIDEACryptData; z: TIDEAKey);
=======
PROCEDURE EnKeyIdea(UserKey: TIdeacryptkey; VAR z: TIDEAKey);
PROCEDURE DeKeyIdea(z: TIDEAKey; VAR dk: TIDEAKey);
PROCEDURE CipherIdea(Input: TIDEACryptData; VAR outdata: TIDEACryptData; z: TIDEAKey);
>>>>>>> graemeg/fixes_2_2
=======
PROCEDURE EnKeyIdea(UserKey: TIdeacryptkey; VAR z: TIDEAKey);
PROCEDURE DeKeyIdea(z: TIDEAKey; VAR dk: TIDEAKey);
PROCEDURE CipherIdea(Input: TIDEACryptData; VAR outdata: TIDEACryptData; z: TIDEAKey);
>>>>>>> origin/fixes_2_2

Type
  EIDEAError = Class(EStreamError);

<<<<<<< HEAD
<<<<<<< HEAD
  { TIDEAStream }

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  TIDEAStream = Class(TOwnerStream)
  Private
    FKey    : TIDEAKey;
    FData   : TIDEACryptData;
    FBufpos : Byte;
<<<<<<< HEAD
<<<<<<< HEAD
    FPos    : Int64;
  Protected
    function GetPosition: Int64; override;
    procedure InvalidSeek; override;
    Procedure CreateCryptKey(Const S : String; Out Key : TIDEACryptKey);
  Public
    Constructor Create(AKey : TIDEAKey; Dest: TStream); overload;
    Property Key : TIDEAKey Read FKey;
  end;

  { TIDEAEncryptStream }

  TIDEAEncryptStream = Class(TIDEAStream)
  public
    Constructor Create(Const AKey : String; Dest: TStream); overload;
=======
=======
>>>>>>> origin/fixes_2_2
    FPos    : Longint;
  Public
    Constructor Create(AKey : TIDEAKey; Dest: TStream);
    Property Key : TIDEAKey Read FKey;
  end;

  TIDEAEncryptStream = Class(TIDEAStream)
  public
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Destructor Destroy; override;
    function Write(const Buffer; Count: Longint): Longint; override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
    procedure Flush;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  { TIDEADeCryptStream }

  TIDEADeCryptStream = Class(TIDEAStream)
  public
    Constructor Create(Const AKey : String; Dest: TStream); overload;
    function Read(var Buffer; Count: Longint): Longint; override;
    function Seek(const Offset: int64; Origin: TSeekOrigin): int64; override;
=======
=======
>>>>>>> origin/fixes_2_2
  TIDEADeCryptStream = Class(TIDEAStream)
  public
    function Read(var Buffer; Count: Longint): Longint; override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

Implementation

Const
  SNoSeekAllowed  = 'Seek not allowed on encryption streams';
<<<<<<< HEAD
<<<<<<< HEAD
  SErrEmptyKey    = 'String Key may not be empty';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

PROCEDURE mul(VAR a:Word; b: Word);
VAR p: LongInt;
BEGIN
  IF (a <> 0) THEN BEGIN
    IF (b <> 0) THEN BEGIN
      p := LongInt(a)*b;
      b := p;
      a := p SHR 16;
      IF (b < a) THEN a := b - a + 1
                 ELSE a := b - a;
    END ELSE a := 1 - a;
  END ELSE a := 1-b;
END;

FUNCTION inv(x: word): Word;
VAR t0,t1,q,y: Word;
BEGIN
  IF x <= 1 THEN BEGIN
    inv := x;
    exit;
  END;
  t1 := 65537 DIV x;
  y := 65537 MOD x;
  IF y = 1 THEN BEGIN
    inv := Word(1-t1);
    exit;
  END;
  t0 := 1;
  REPEAT
    q := x DIV y;
    x := x MOD y;
    t0 := t0 + q * t1;
    IF x = 1 THEN BEGIN
      inv := t0;
      exit;
    END;
    q := y DIV x;
    y := y MOD x;
    t1 := t1 + q*t0;
  UNTIL y = 1;
  inv := word(1-t1);
END;

<<<<<<< HEAD
<<<<<<< HEAD
PROCEDURE EnKeyIdea(userkey: ideacryptkey; OUT z: ideakey);
=======
PROCEDURE EnKeyIdea(userkey: ideacryptkey; VAR z: ideakey);
>>>>>>> graemeg/fixes_2_2
=======
PROCEDURE EnKeyIdea(userkey: ideacryptkey; VAR z: ideakey);
>>>>>>> origin/fixes_2_2
VAR zi,i,j: integer;
BEGIN
  FOR j := 0 TO 7 DO z[j] := userkey[j];
  zi := 0;
  i := 0;
  FOR j := 8 TO keylen-1 DO BEGIN
    Inc(i);
    z[zi+i+7] := (z[zi+(i AND 7)] SHL 9) OR (z[zi+((i+1) AND 7)] SHR 7);
    zi := zi + (i AND 8);
    i := i AND 7;
  END;
  FOR i := 0 TO 7 DO userkey[i] := 0;
END;

<<<<<<< HEAD
<<<<<<< HEAD
PROCEDURE DeKeyIdea(z: IDEAKey; OUT dk: ideakey);
=======
PROCEDURE DeKeyIdea(z: IDEAKey; VAR dk: ideakey);
>>>>>>> graemeg/fixes_2_2
=======
PROCEDURE DeKeyIdea(z: IDEAKey; VAR dk: ideakey);
>>>>>>> origin/fixes_2_2
VAR j: Integer;
    t1,t2,t3: Word;
    p: IDEAKey;
    ip: Integer;
    iz: Integer;
BEGIN
  iz := 0;
  ip := keylen;
  FOR j := 0 TO keylen - 1 DO p[j] := 0;
  t1 := inv(z[iz]);   Inc(iz);
  t2 := not(z[iz])+1; Inc(iz);
  t3 := not(z[iz])+1; Inc(iz);
  Dec(ip); p[ip] := inv(z[iz]); Inc(iz);
  Dec(ip); p[ip] := t3;
  Dec(ip); p[ip] := t2;
  Dec(ip); p[ip] := t1;
  FOR j := 1 TO rounds-1 DO BEGIN
    t1 := z[iz]; Inc(iz);
    Dec(ip); p[ip] := z[iz]; Inc(iz);
    Dec(ip); p[ip] := t1;
    t1 := inv(z[iz]);   Inc(iz);
    t2 := Not(z[iz])+1; Inc(iz);
    t3 := Not(z[iz])+1; Inc(iz);
    Dec(ip); p[ip] := inv(z[iz]); Inc(iz);
    Dec(ip); p[ip] := t2;
    Dec(ip); p[ip] := t3;
    Dec(ip); p[ip] := t1;
  END;
  t1 := z[iz]; Inc(iz);
  Dec(ip); p[ip] := z[iz]; Inc(iz);
  Dec(ip); p[ip] := t1;
  t1 := inv(z[iz]);   Inc(iz);
  t2 := Not(z[iz])+1; Inc(iz);
  t3 := Not(z[iz])+1; Inc(iz);
  Dec(ip); p[ip] := inv(z[iz]);
  Dec(ip); p[ip] := t3;
  Dec(ip); p[ip] := t2;
  Dec(ip); p[ip] := t1;
  FOR j := 0 TO KeyLen-1 DO BEGIN
    dk[j] := p[j];
    p[j] := 0;
  END;
  FOR j := 0 TO 51 DO z[j] := 0;
END;

<<<<<<< HEAD
<<<<<<< HEAD
PROCEDURE CipherIdea(input: ideacryptdata; OUT outdata: ideacryptdata; z:IDEAkey);
=======
PROCEDURE CipherIdea(input: ideacryptdata; VAR outdata: ideacryptdata; z:IDEAkey);
>>>>>>> graemeg/fixes_2_2
=======
PROCEDURE CipherIdea(input: ideacryptdata; VAR outdata: ideacryptdata; z:IDEAkey);
>>>>>>> origin/fixes_2_2
VAR x1, x2, x3, x4, t1, t2: Word;
    r: Integer;
    zi: Integer;
BEGIN
  zi := 0;
  x1 := input[0];
  x2 := input[1];
  x3 := input[2];
  x4 := input[3];
  FOR r := 1 TO ROUNDS DO BEGIN
    mul(x1,z[zi]);    Inc(zi);
    x2 := x2 + z[zi]; Inc(zi);
    x3 := x3 + z[zi]; Inc(zi);
    mul(x4, z[zi]);   Inc(zi);
    t2 := x1 XOR x3;
    mul(t2, z[zi]);   Inc(zi);
    t1 := t2 + (x2 XOR x4);
    mul(t1, z[zi]);   Inc(zi);
    t2 := t1+t2;
    x1 := x1 XOR t1;
    x4 := x4 XOR t2;
    t2 := t2 XOR x2;
    x2 := x3 XOR t1;
    x3 := t2;
  END;
  mul(x1, z[zi]);       Inc(zi);
  outdata[0] := x1;
  outdata[1] := x3 + z[zi]; Inc(zi);
  outdata[2] := x2 + z[zi]; Inc(zi);
  Mul(x4,z[zi]);
  outdata[3] := x4;
  FOR r := 0 TO 3 DO input[r] := 0;
  FOR r := 0 TO 51 DO z[r] := 0;
END;

{ ---------------------------------------------------------------------
    TIDEAStream
  ---------------------------------------------------------------------}
  

<<<<<<< HEAD
<<<<<<< HEAD
Constructor TIDEAStream.Create(AKey : TIDEAKey; Dest: TStream);
=======
Constructor TIDEAStream.Create(AKey : ideakey; Dest: TStream);
>>>>>>> graemeg/fixes_2_2
=======
Constructor TIDEAStream.Create(AKey : ideakey; Dest: TStream);
>>>>>>> origin/fixes_2_2

begin
  inherited Create(Dest);
  FKey:=AKey;
  FBufPos:=0;
  Fpos:=0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TIDEAStream.GetPosition: Int64;
begin
  Result:=FPos;
end;

procedure TIDEAStream.InvalidSeek;
begin
  Raise EIDEAError.Create(SNoSeekAllowed);
end;

procedure TIDEAStream.CreateCryptKey(const S: String; out Key: TIDEACryptKey);

Var
  KLen : Integer;

begin
  KLen:=Length(S);
  If (KLen=0) then
    Raise EIDEAError.Create(SErrEmptyKey);
  If (Length(S)>SizeOf(Key)) then
    KLen:=SizeOf(Key);
  FillChar(Key,SizeOf(Key),0);
  Move(S[1],Key,KLen);
end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ---------------------------------------------------------------------
    TIDEAEncryptStream
  ---------------------------------------------------------------------}

<<<<<<< HEAD
<<<<<<< HEAD
constructor TIDEAEncryptStream.Create(Const AKey: String; Dest: TStream);

Var
  K : TIdeaCryptKey;
  Z : TIDeaKey;

begin
  CreateCryptKey(AKey,K);
  EnKeyIDEA(K,Z);
  Inherited Create(Z,Dest);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Destructor TIDEAEncryptStream.Destroy;


begin
  Flush;
  Inherited Destroy;
end;

Procedure TIDEAEncryptStream.Flush;

Var
  OutData : IdeaCryptData;

begin
  If FBufPos>0 then
    begin
    // Fill with nulls
    FillChar(PChar(@FData)[FBufPos],SizeOf(FData)-FBufPos,#0);
    CipherIdea(Fdata,OutData,FKey);
    Source.Write(OutData,SizeOf(OutData));
    // fixed: Manual flush and then free will now work
    FBufPos := 0;
    end;
end;

function TIDEAEncryptStream.Write(const Buffer; Count: Longint): Longint;

Var
  mvsize : Longint;
  OutData : IDEAcryptdata;

begin
  Result:=0;
  While Count>0 do
    begin
    MVsize:=Count;
    If Mvsize>SizeOf(Fdata)-FBufPos then
      mvsize:=SizeOf(FData)-FBufPos;
    Move(PChar(@Buffer)[Result],PChar(@FData)[FBufPos],MVSize);
    If FBufPos+mvSize=Sizeof(FData) then
      begin
      // Empty buffer.
      CipherIdea(Fdata,OutData,FKey);
      // this will raise an exception if needed.
      Source.Writebuffer(OutData,SizeOf(OutData));
      FBufPos:=0;
      end
    else
      inc(FBufPos,mvsize);
    Dec(Count,MvSize);
    Inc(Result,mvSize);
    end;
  Inc(FPos,Result);
end;


function TIDEAEncryptStream.Seek(Offset: Longint; Origin: Word): Longint;

begin
  if (Offset = 0) and (Origin = soFromCurrent) then
    Result := FPos
  else
<<<<<<< HEAD
<<<<<<< HEAD
    InvalidSeek;
=======
    Raise EIDEAError.Create(SNoSeekAllowed);
>>>>>>> graemeg/fixes_2_2
=======
    Raise EIDEAError.Create(SNoSeekAllowed);
>>>>>>> origin/fixes_2_2
end;


{ ---------------------------------------------------------------------
    TIDEADecryptStream
  ---------------------------------------------------------------------}

<<<<<<< HEAD
<<<<<<< HEAD
constructor TIDEADeCryptStream.Create(const AKey: String; Dest: TStream);

Var
  K : TIdeaCryptKey;
  Z1,Z2 : TIDeaKey;

begin
  CreateCryptKey(AKey,K);
  EnKeyIDEA(K,Z1);
  DeKeyIDEA(Z1,Z2);
  Inherited Create(Z2,Dest);
end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function TIDEADeCryptStream.Read(var Buffer; Count: Longint): Longint;

Var
  mvsize : Longint;
  InData : IDEAcryptdata;

begin
  Result:=0;
  While Count>0 do
    begin
    // Empty existing buffer.
    If (FBufPos>0) then
      begin
      mvSize:=FBufPos;
      If MvSize>count then
        mvsize:=Count;
      Move(PChar(@FData)[0],PChar(@Buffer)[Result],MVSize);
      If ((Sizeof(FData)-MvSize)>0) then
        Move(PChar(@FData)[mvSize],PChar(@FData)[0],Sizeof(FData)-MvSize);
      Dec(Count,mvsize);
      Inc(Result,mvsize);
      FBufPos:=FBufPos-MvSize;
      end;
    // Fill buffer again if needed.
    If (Count>0) then
      begin
      mvsize:=Source.Read(InData,SizeOf(InData));
      If mvsize>0 then
        begin
        If MvSize<SizeOf(InData) Then
          // Fill with nulls
          FillChar(PChar(@InData)[mvsize],SizeOf(InData)-mvsize,#0);
        CipherIdea(InData,FData,FKey);
        FBufPos:=SizeOf(FData);
        end
      else
        Count:=0; // No more data available from stream; st
      end;
    end;
  Inc(FPos,Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TIDEADeCryptStream.Seek(const Offset: int64; Origin: TSeekOrigin): int64;

begin
  FakeSeekForward(Offset,Origin,fpos);
  Result:=FPos; // FPos updated by read
=======
=======
>>>>>>> origin/fixes_2_2
function TIDEADeCryptStream.Seek(Offset: Longint; Origin: Word): Longint;

Var Buffer : Array[0..1023] of byte;
    i : longint;

begin
  // Fake seek if possible by reading and discarding bytes.
  If ((Offset>=0) and (Origin = soFromCurrent)) or
    ((Offset>FPos) and (Origin = soFromBeginning)) then
      begin
      For I:=1 to (Offset div SizeOf(Buffer)) do
        ReadBuffer(Buffer,SizeOf(Buffer));
      ReadBuffer(Buffer,Offset mod SizeOf(Buffer));
      Result:=FPos;
      end
  else
    Raise EIDEAError.Create(SNoSeekAllowed);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

END.
