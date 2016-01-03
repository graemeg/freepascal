{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt

    Implementation of pipe stream.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}

Unit Pipes;

Interface

Uses sysutils,Classes;

Type
  EPipeError = Class(EStreamError);
  EPipeSeek = Class (EPipeError);
  EPipeCreation = Class (EPipeError);

  { TInputPipeStream }

  TInputPipeStream = Class(THandleStream)
    Private
      FPos : Int64;
      function GetNumBytesAvailable: DWord;
<<<<<<< HEAD
<<<<<<< HEAD
    protected
      function GetPosition: Int64; override;
      procedure InvalidSeek; override;
    public
      destructor Destroy; override;
      Function Write (Const Buffer; Count : Longint) :Longint; Override;
      function Seek(const Offset: int64; Origin: TSeekOrigin): int64; override;
=======
    public
      Function Write (Const Buffer; Count : Longint) :Longint; Override;
      Function Seek (Offset : Longint;Origin : Word) : longint;override;
>>>>>>> graemeg/fixes_2_2
=======
    public
      Function Write (Const Buffer; Count : Longint) :Longint; Override;
      Function Seek (Offset : Longint;Origin : Word) : longint;override;
>>>>>>> origin/fixes_2_2
      Function Read (Var Buffer; Count : Longint) : longint; Override;
      property NumBytesAvailable: DWord read GetNumBytesAvailable;
    end;

  TOutputPipeStream = Class(THandleStream)
    Public
<<<<<<< HEAD
<<<<<<< HEAD
      destructor Destroy; override;
      function Seek(const Offset: int64; Origin: TSeekOrigin): int64; override;
      Function Read (Var Buffer; Count : Longint) : longint; Override;
    end;

Function CreatePipeHandles (Var Inhandle,OutHandle : THandle; APipeBufferSize : Cardinal = 1024) : Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
      Function Seek (Offset : Longint;Origin : Word) : longint;override;
      Function Read (Var Buffer; Count : Longint) : longint; Override;
    end;

Function CreatePipeHandles (Var Inhandle,OutHandle : THandle) : Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Procedure CreatePipeStreams (Var InPipe : TInputPipeStream;
                             Var OutPipe : TOutputPipeStream);

Const EPipeMsg = 'Failed to create pipe.';
      ENoSeekMsg = 'Cannot seek on pipes';


Implementation

{$i pipes.inc}

Procedure CreatePipeStreams (Var InPipe : TInputPipeStream;
                             Var OutPipe : TOutputPipeStream);

Var InHandle,OutHandle : THandle;

begin
  if CreatePipeHandles (InHandle, OutHandle) then
    begin
    InPipe:=TInputPipeStream.Create (InHandle);
    OutPipe:=TOutputPipeStream.Create (OutHandle);
    end
  Else
    Raise EPipeCreation.Create (EPipeMsg)
end;

<<<<<<< HEAD
<<<<<<< HEAD
destructor TInputPipeStream.Destroy;
begin
  PipeClose (Handle);
  inherited;
end;

Function TInputPipeStream.Write (Const Buffer; Count : Longint) : longint;

begin
  WriteNotImplemented;
=======
=======
>>>>>>> origin/fixes_2_2
Function TInputPipeStream.Write (Const Buffer; Count : Longint) : longint;

begin
{$ifdef ver2_2_0}
  raise EStreamError.Create( 'Cannot write to InputPipeStream');
{$else}
  WriteNotImplemented;
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result := 0;
end;

Function TInputPipeStream.Read (Var Buffer; Count : Longint) : longint;

begin
  Result:=Inherited Read(Buffer,Count);
  Inc(FPos,Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TInputPipeStream.Seek(const Offset: int64; Origin: TSeekOrigin): int64;

begin
  FakeSeekForward(Offset,Origin,FPos);
  Result:=FPos;
end;

destructor TOutputPipeStream.Destroy;
begin
  PipeClose (Handle);
  inherited;
=======
=======
>>>>>>> origin/fixes_2_2
Function TInputPipeStream.Seek (Offset : Longint;Origin : Word) : longint;

Const BufSize = 100;

Var Buf : array[1..BufSize] of Byte;

begin
  If (Origin=soFromCurrent) and (Offset=0) then
     result:=FPos;
  { Try to fake seek by reading and discarding }
  if Not((Origin=soFromCurrent) and (Offset>=0) or
         ((Origin=soFrombeginning) and (OffSet>=FPos))) then
     Raise EPipeSeek.Create(ENoSeekMSg);
  if Origin=soFromBeginning then
    Dec(Offset,FPos);
  While ((Offset Div BufSize)>0)
        and (Read(Buf,SizeOf(Buf))=BufSize) do
     Dec(Offset,BufSize);
  If (Offset>0) then
    Read(Buf,BufSize);
  Result:=FPos;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Function TOutputPipeStream.Read(Var Buffer; Count : Longint) : longint;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  ReadNotImplemented;
  Result := 0;
end;

function TOutputPipeStream.Seek(const Offset: int64; Origin: TSeekOrigin): int64;

begin
  Result:=0; { to silence warning mostly }
  InvalidSeek;
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ver2_2_0}
  raise EStreamError.Create( 'Cannot read from OutputPipeStream');
{$else}
  ReadNotImplemented;
{$endif}
  Result := 0;
end;

Function TOutputPipeStream.Seek (Offset : Longint;Origin : Word) : longint;

begin
  Raise EPipeSeek.Create (ENoSeekMsg);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

end.
