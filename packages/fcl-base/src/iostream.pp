{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by Michael Van Canneyt and Florian Klaempfl

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}

unit iostream;

interface

uses Classes;

type
  TIOSType = (iosInput,iosOutPut,iosError);
  EIOStreamError = class(EStreamError);

  TIOStream = class(THandleStream)
  private
    FType : longint;
    FPos : Int64;
    zIOSType : TIOSType;
  public
    constructor Create(aIOSType : TiosType);
    function Read(var Buffer; Count : LongInt) : Longint; override;
    function Write(const Buffer; Count : LongInt) : LongInt; override;
    procedure SetSize(NewSize: Longint); override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
    end;

implementation

const
  SReadOnlyStream = 'Cannot write to an input stream.';
  SWriteOnlyStream = 'Cannot read from an output stream.';
  SInvalidOperation = 'Cannot perform this operation on a IOStream.';

constructor TIOStream.Create(aIOSType : TIOSType);
begin
{$ifdef win32}
  case aIOSType of
    iosInput : FType := StdInputHandle;
    iosOutput : FType := StdOutputHandle;
    iosError : FType := StdErrorHandle;
  end;
{$else}
  FType := Ord(aIOSType);
{$endif}
  inherited Create(FType);
  zIOSType := aIOSType;
end;

function TIOStream.Read(var Buffer; Count : LongInt) : Longint;
begin
  if (zIOSType <> iosInput) then
    raise EIOStreamError.Create(SWriteOnlyStream)
  else begin
    result := inherited Read(Buffer,Count);
    inc(FPos,result);
  end;
end;

function TIOStream.Write(const Buffer; Count : LongInt) : LongInt;
begin
  if (zIOSType = iosInput) then
    raise EIOStreamError.Create(SReadOnlyStream)
  else begin
    result := inherited Write(Buffer,Count);
    inc(FPos,result);
  end;
end;

procedure TIOStream.SetSize(NewSize: Longint);
begin
  raise EIOStreamError.Create(SInvalidOperation);
end;

function TIOStream.Seek(Offset: Longint; Origin: Word): Longint;
const
  BufSize = 1024;
var
  Buf : array[1..BufSize] of Byte;
begin
  If (Origin=soFromCurrent) and (Offset=0) then
     result:=FPos;
  { Try to fake seek by reading and discarding }
  if (zIOSType = iosOutput) or
     Not((Origin=soFromCurrent) and (Offset>=0) or
         ((Origin=soFrombeginning) and (OffSet>=FPos))) then
     Raise EIOStreamError.Create(SInvalidOperation);
  if Origin=soFromBeginning then
    Dec(Offset,FPos);
  While ((Offset Div BufSize)>0)
        and (Read(Buf,SizeOf(Buf))=BufSize) do
     Dec(Offset,BufSize);
  If (Offset>0) then
    Read(Buf,BufSize);
  Result:=FPos;
end;

end.
