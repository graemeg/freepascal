{
    This file is part of the Free Component Library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Implement a buffered stream.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
{$H+}
unit bufstream;

interface

uses
  Classes, SysUtils;

Const
  DefaultBufferCapacity : Integer = 16; // Default buffer capacity in Kb.

Type


  { TBufStream }
  TBufStream = Class(TOwnerStream)
  Private
    FTotalPos : Int64;
    Fbuffer: Pointer;
    FBufPos: Integer;
    FBufSize: Integer;
    FCapacity: Integer;
    procedure SetCapacity(const AValue: Integer);
  Protected
<<<<<<< HEAD
<<<<<<< HEAD
    function GetPosition: Int64; override;
    function GetSize: Int64; override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure BufferError(const Msg : String);
    Procedure FillBuffer; Virtual;
    Procedure FlushBuffer; Virtual;
  Public
    Constructor Create(ASource : TStream; ACapacity: Integer);
    Constructor Create(ASource : TStream);
    Destructor Destroy; override;
    Property Buffer : Pointer Read Fbuffer;
    Property Capacity : Integer Read FCapacity Write SetCapacity;
    Property BufferPos : Integer Read FBufPos; // 0 based.
    Property BufferSize : Integer Read FBufSize; // Number of bytes in buffer.
  end;

  { TReadBufStream }

  TReadBufStream = Class(TBufStream)
  Public
<<<<<<< HEAD
<<<<<<< HEAD
    function Seek(const Offset: Int64; Origin: TSeekOrigin): Int64; override;
=======
    Function Seek(Offset: Longint; Origin: Word): Longint; override;
>>>>>>> graemeg/fixes_2_2
=======
    Function Seek(Offset: Longint; Origin: Word): Longint; override;
>>>>>>> origin/fixes_2_2
    Function Read(var ABuffer; ACount : LongInt) : Integer; override;
  end;

  { TWriteBufStream }

  TWriteBufStream = Class(TBufStream)
  Public
    Destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function Seek(const Offset: Int64; Origin: TSeekOrigin): Int64; override;
=======
    Function Seek(Offset: Longint; Origin: Word): Longint; override;
>>>>>>> graemeg/fixes_2_2
=======
    Function Seek(Offset: Longint; Origin: Word): Longint; override;
>>>>>>> origin/fixes_2_2
    Function Write(Const ABuffer; ACount : LongInt) : Integer; override;
  end;

implementation

Resourcestring
  SErrCapacityTooSmall = 'Capacity is less than actual buffer size.';
  SErrCouldNotFLushBuffer = 'Could not flush buffer';
  SErrInvalidSeek = 'Invalid buffer seek operation';

{ TBufStream }

procedure TBufStream.SetCapacity(const AValue: Integer);
begin
  if (FCapacity<>AValue) then
    begin
    If (AValue<FBufSize) then
      BufferError(SErrCapacityTooSmall);
    ReallocMem(FBuffer,AValue);
    FCapacity:=AValue;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TBufStream.GetPosition: Int64;
begin
  Result:=FTotalPos;
end;

function TBufStream.GetSize: Int64;
begin
  Result:=Source.Size;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TBufStream.BufferError(const Msg: String);
begin
  Raise EStreamError.Create(Msg);
end;

procedure TBufStream.FillBuffer;

Var
  RCount : Integer;
  P : PChar;

begin
  P:=Pchar(FBuffer);
  // Reset at beginning if empty.
  If (FBufSize-FBufPos)<=0 then
   begin
   FBufSize:=0;
   FBufPos:=0;
   end;
  Inc(P,FBufSize);
  RCount:=1;
  while (RCount<>0) and (FBufSize<FCapacity) do
    begin
    RCount:=FSource.Read(P^,FCapacity-FBufSize);
    Inc(P,RCount);
    Inc(FBufSize,RCount);
    end;
end;

procedure TBufStream.FlushBuffer;

Var
  WCount : Integer;
  P : PChar;

begin
  P:=Pchar(FBuffer);
  Inc(P,FBufPos);
  WCount:=1;
  While (WCount<>0) and ((FBufSize-FBufPos)>0) do
    begin
    WCount:=FSource.Write(P^,FBufSize-FBufPos);
    Inc(P,WCount);
    Inc(FBufPos,WCount);
    end;
  If ((FBufSize-FBufPos)<=0) then
    begin
    FBufPos:=0;
    FBufSize:=0;
    end
  else
    BufferError(SErrCouldNotFLushBuffer);
end;

constructor TBufStream.Create(ASource: TStream; ACapacity: Integer);
begin
  Inherited Create(ASource);
  SetCapacity(ACapacity);
end;

constructor TBufStream.Create(ASource: TStream);
begin
  Create(ASource,DefaultBufferCapacity*1024);
end;

destructor TBufStream.Destroy;
begin
  FBufSize:=0;
  SetCapacity(0);
  inherited Destroy;
end;

{ TReadBufStream }

<<<<<<< HEAD
<<<<<<< HEAD
function TReadBufStream.Seek(const Offset: Int64; Origin: TSeekOrigin): Int64;

begin
  FakeSeekForward(Offset,Origin,FTotalPos);
  Result:=FTotalPos; // Pos updated by fake read
=======
=======
>>>>>>> origin/fixes_2_2
function TReadBufStream.Seek(Offset: Longint; Origin: Word): Longint;

var
  I: Integer;
  Buf: array [0..4095] of Char;

begin
  // Emulate forward seek if possible.
  if ((Offset>=0) and (Origin = soFromCurrent)) or
     (((Offset-FTotalPos)>=0) and (Origin = soFromBeginning)) then
    begin
    if (Origin=soFromBeginning) then
      Dec(Offset,FTotalPos);
    if (Offset>0) then
      begin
      for I:=1 to (Offset div sizeof(Buf)) do
        ReadBuffer(Buf,sizeof(Buf));
      ReadBuffer(Buf, Offset mod sizeof(Buf));
      end;
    Result:=FTotalPos;
    end
  else
    BufferError(SErrInvalidSeek);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TReadBufStream.Read(var ABuffer; ACount: LongInt): Integer;

Var
  P,PB : PChar;
  Avail,MSize,RCount : Integer;

begin
  Result:=0;
  P:=PChar(@ABuffer);
  Avail:=1;
  While (Result<ACount) and (Avail>0) do
    begin
    If (FBufSize-FBufPos<=0) then
      FillBuffer;
    Avail:=FBufSize-FBufPos;
    If (Avail>0) then
      begin
      MSize:=ACount-Result;
      If (MSize>Avail) then
        MSize:=Avail;
      PB:=PChar(FBuffer);
      Inc(PB,FBufPos);
      Move(PB^,P^,MSIze);
      Inc(FBufPos,MSize);
      Inc(P,MSize);
      Inc(Result,MSize);
      end;
    end;
  Inc(FTotalPos,Result);
end;

{ TWriteBufStream }

destructor TWriteBufStream.Destroy;
begin
  FlushBuffer;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TWriteBufStream.Seek(const Offset: Int64; Origin: TSeekOrigin): Int64;

begin
  if (Offset=0) and (Origin=soCurrent) then
=======
function TWriteBufStream.Seek(Offset: Longint; Origin: Word): Longint;
begin
  if (Offset=0) and (Origin=soFromCurrent) then
>>>>>>> graemeg/fixes_2_2
=======
function TWriteBufStream.Seek(Offset: Longint; Origin: Word): Longint;
begin
  if (Offset=0) and (Origin=soFromCurrent) then
>>>>>>> origin/fixes_2_2
    Result := FTotalPos
  else
    BufferError(SErrInvalidSeek);
end;

function TWriteBufStream.Write(const ABuffer; ACount: LongInt): Integer;

Var
  P,PB : PChar;
  Avail,MSize,RCount : Integer;

begin
  Result:=0;
  P:=PChar(@ABuffer);
  While (Result<ACount) do
    begin
    If (FBufSize=FCapacity) then
      FlushBuffer;
    Avail:=FCapacity-FBufSize;
    MSize:=ACount-Result;
    If (MSize>Avail) then
      MSize:=Avail;
    PB:=PChar(FBuffer);
    Inc(PB,FBufSize);
    Move(P^,PB^,MSIze);
    Inc(FBufSize,MSize);
    Inc(P,MSize);
    Inc(Result,MSize);
    end;
  Inc(FTotalPos,Result);
end;


end.
