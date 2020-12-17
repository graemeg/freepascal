{ Copyright (C) <2010> <Andrew Haines> itfstransform.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1301, USA.
}
{
  See the file COPYING.modifiedLGPL, included in this distribution,
  for details about the copyright.
}
unit ITSFTransform;

{ $DEFINE DEBUG_HELP2}

{$mode objfpc}{$H+}

interface

uses
  Classes, paslzx, ITOLITLSTypes;

type
  TGetObject = function(AName: String): TMemoryStream of object;

  TITSFTransformList = class;
  TITSFTransform = class;

  { TITSFTransform }
  TITSFTransform = class
  private
    FDataStream: TStream;
    FPrefix: String;
    FParentTransform: TITSFTransform;
  public
    GetObject: TGetObject; // GetObject(Name: String): TMemoryStream;
    OutStream: TMemoryStream;
    constructor Create(AGetObject: TGetObject; AParentTransform: TITSFTransform); virtual;

    function WantData(APrefix: String; ADataStream: TStream; const DataStart, DataLength: Integer; AOutStream: TMemoryStream): Boolean; virtual;

    property ParentTransform : TITSFTransform read FParentTransform;
    property Prefix: String read FPrefix write FPrefix;
    property DataStream: TStream read FDataStream write FDataStream;
    class function GUID: TGuid; virtual;
    class function GUIDString: String; virtual;
  end;

  TITSFTransformClass = class of TITSFTransform;

  { TPlainTransform }

  TPlainTransform = class(TITSFTransform)
    function WantData(APrefix: String; ADataStream: TStream; const DataStart, DataLength: Integer; AOutStream: TMemoryStream): Boolean; override;
  end;

  { TLZXv3Transform }

  TLZXv3Transform = class(TITSFTransform)
  private
    Entries: array of QWord;
    Data: TLZXv3ControlData;
    Table: TLZXv3ResetTable;
    function GetControlData: Boolean;
    function GetResetTable: Boolean;

    function FindChunkCompressedSize(AEntryIndex: Integer): DWord;
    function FindFirstChunkFromOffset(AOffset, ASize: QWord; out ChunkCount: DWord): Integer;

    function ExtractChunks(AFirstChunkIndex, AChunkCount: DWord; out ExtractedOffsetStart: QWord): TMemoryStream;
  public
    function WantData(APrefix: String; ADataStream: TStream; const DataStart, DataLength: Integer; AOutStream: TMemoryStream): Boolean; override;

    class function GUID: TGuid; override;
  end;

  { TITSFTransformList }

  TITSFTransformList = class(TFPList)
  private
    function GetTransform(AGuid: TGuid): TITSFTransformClass;
    function GetTransformIndex(AIndex: DWord): TITSFTransformClass;
    function GetTransformInstance(AIndex: DWord): TITSFTransform;
    procedure SetTransformInstance(AIndex: DWord; const AValue: TITSFTransform);
  public
    function AddTransform(ATransform: TITSFTransformClass): Integer;
    procedure Delete(AIndex: Integer);
    property Transform[AGuid: TGuid]: TITSFTransformClass read GetTransform;
    property TransformIndex[AIndex: DWord]: TITSFTransformClass read GetTransformIndex;
    property TransformInstance[AIndex: DWord]: TITSFTransform read GetTransformInstance write SetTransformInstance;
  end;

function RegisteredTransforms: TITSFTransformList;


implementation
uses
  SysUtils;

var
  LocTransforms: TITSFTransformList = nil;

type
  PITSFTranformItem = ^TITSFTranformItem;
  TITSFTranformItem = record
    //Guid: TGuid;
    Transform: TITSFTransformClass;
    Instance: TITSFTransform;
  end;

function RegisteredTransforms: TITSFTransformList;
begin
  if LocTransforms = nil then
    LocTransforms := TITSFTransformList.Create;
  Result := LocTransforms;
end;

{ TITSFTransform }

constructor TITSFTransform.Create(AGetObject: TGetObject; AParentTransform: TITSFTransform);
begin
  GetObject:=AGetObject;
  FParentTransform := AParentTransform;
end;

function TITSFTransform.WantData(APrefix: String; ADataStream: TStream;
  const DataStart, DataLength: Integer; AOutStream: TMemoryStream): Boolean;
begin
  Prefix := APrefix;
  DataStream := ADataStream;
  OutStream := AOutStream;
  {$IFDEF DEBUG_HELP2}
  WriteLn('WantData Class = ', ClassName);
  {$ENDIF}
end;

class function TITSFTransform.GUID: TGuid;
const
  AGuid: TGuid = '{00000000-0000-0000-0000-000000000000}';
begin
  Result := AGuid;
end;

class function TITSFTransform.GUIDString: String;
begin
  Result := GUIDToString(GUID);
end;

{ TITSFTransformList }

function TITSFTransformList.GetTransform(AGuid: TGuid): TITSFTransformClass;
var
  Item: PITSFTranformItem;
  i: Integer;
  GUID: TGuid;
begin
  Result := nil;
  for i := 0 to Count-1 do
  begin
    Item := PITSFTranformItem(Items[i]);
    GUID := Item^.Transform.GUID;
    if CompareByte(GUID,AGuid, 16) = 0 then
      Exit(Item^.Transform);
  end;
end;

function TITSFTransformList.GetTransformIndex(AIndex: DWord): TITSFTransformClass;
begin
  Result := PITSFTranformItem(Items[AIndex])^.Transform;
end;

function TITSFTransformList.GetTransformInstance(AIndex: DWord): TITSFTransform;
begin
  Result := PITSFTranformItem(Items[AIndex])^.Instance;
end;

procedure TITSFTransformList.SetTransformInstance(AIndex: DWord;
  const AValue: TITSFTransform);
begin
  PITSFTranformItem(Items[AIndex])^.Instance := AValue;
end;

function TITSFTransformList.AddTransform(ATransform: TITSFTransformClass): Integer;
var
  Item: PITSFTranformItem;
begin
  if not Assigned(ATransform) then
    Exit;
  New(Item);
  Item^.Transform:= ATransform;
  Item^.Instance := nil;
  Add(Item);
end;

procedure TITSFTransformList.Delete(AIndex: Integer);
var
  Item: PITSFTranformItem;
begin
  Item := PITSFTranformItem(Items[AIndex]);
  Dispose(Item);
  Inherited Delete(AIndex);
end;

{ TLZXv3Transform }

function TLZXv3Transform.FindFirstChunkFromOffset(AOffset, ASize: QWord; out ChunkCount: DWord): Integer;
var
  EndChunk: DWord;
begin
  Result := AOffset div Table.BlockSize;
  EndChunk := (AOffset + ASize) div Table.BlockSize;
  ChunkCount:=EndChunk-Result;
  //if ChunkCount = 0 then
    Inc(ChunkCount);
end;

function TLZXv3Transform.GetControlData: Boolean;
var
  ControlDataStream: TStream;
  ESize: LongWord;
begin
  Result := False;
  try
    ControlDataStream := GetObject(Prefix+'ControlData');

    if ControlDataStream = nil then
      Exit;


    ESize := NtoLE(ControlDataStream.ReadDWord);
    while ESize <> 7 do
    begin
      ControlDataStream.Seek(ESize*4, soFromCurrent);
      ESize := LEtoN(ControlDataStream.ReadDWord);
    end;
    if ESize = 7 then
      ControlDataStream.Read(Data, SizeOf(TLZXv3ControlData));

  finally
    if Assigned(ControlDataStream) then
      ControlDataStream.Free;
  end;
  Result := ESize = 7;
  //WriteLn('GetControlData = ', REsult);

end;

function TLZXv3Transform.GetResetTable: Boolean;
var
  WholePrefix: String;
  ResetStream: TStream;
  {$IFDEF ENDIAN_BIG}
  i: Integer;
  {$ENDIF}
begin
  Result := False;
  WholePrefix:=Prefix+'Transform/'+GUIDString+'/';
  ResetStream := GetObject(WholePrefix+'InstanceData/ResetTable');
  if ResetStream = nil then
    Exit;
  ResetStream.Read(Table, SizeOf(TLZXv3ResetTable));
  SetLength(Entries, Table.EntryCount);
  ResetStream.Read(Entries[0], Table.EntryCount*8);
  {$IFDEF ENDIAN_BIG}
  for i := Low(Entries) to High(Entries) do
    Entries[i] := LEtoN(Entries[i]);
  {$ENDIF}
  {$IFDEF DEBUG_HELP2}
  //for i := Low(Entries) to High(Entries) do
  //  WriteLn('Entry[',i,'] = ',Entries[i] ,' UnCompressStart = ', i*$8000);
  {$ENDIF}
  ResetStream.Free;

  Result := True;
end;

function TLZXv3Transform.FindChunkCompressedSize(AEntryIndex: Integer): DWord;
begin
  if AEntryIndex < High(Entries) then
    Result := Entries[AEntryIndex+1] - Entries[AEntryIndex]
  else
    Result := DataStream.Size-Entries[AEntryIndex];
end;

function TLZXv3Transform.ExtractChunks(AFirstChunkIndex, AChunkCount: DWord;
  out ExtractedOffsetStart: QWord): TMemoryStream;
var
  LZX: PLZXState;
  CStart,
  CSize: DWord;
  //CBuf: Pointer;
  Buf: TMemoryStream;
  CBuf: Pointer;
  UBuf: Pointer;
  USize: Dword;
  URes: DWord;
  WinCode: DWord;
  WinSize: QWord;
  BlockMask: Byte;
begin
  BlockMask := (Data.ResetInterval shl 1) - 1;

  // must start on a even numbered block
  while (AFirstChunkIndex mod Data.ResetInterval <> 0) and (AFirstChunkIndex > 0) do
  begin
    Dec(AFirstChunkIndex);
    Inc(AChunkCount);
  end;

  ExtractedOffsetStart := Table.BlockSize*AFirstChunkIndex;

  {$IFDEF DEBUG_HELP2}
  WriteLn('Getting Data, StartChunk=', AFirstChunkIndex,' Count = ', AChunkCount);
  WriteLn('Version = ', Data.Version);
  WriteLn('Window Size = ',Data.WindowSize);
  WriteLn('Block Size = ',Hexstr(Table.BlockSize,16));
  WriteLn('Block Size = ',Table.BlockSize);
  {$ENDIF}

  WinSize := (Data.WindowSize * Table.BlockSize);
  WinCode := 0;
  while WinSize > 1 do
  begin
    Inc(WinCode);
    //WriteLn(HexStr(WinSize, 16));
    WinSize := WinSize shr 1;
  end;

  LZX := LZXinit(WinCode);//ata.WindowSize);

  CBuf := GetMem(Table.BlockSize);
  UBuf := GetMem(Table.BlockSize);

  Result := TMemoryStream.Create;
  Buf := TMemoryStream.Create;

  CStart := Entries[AFirstChunkIndex];
  CSize  := Entries[AFirstChunkIndex+AChunkCount]+FindChunkCompressedSize(AFirstChunkIndex+AChunkCount);
  ParentTransform.WantData(Prefix, DataStream, CStart, CSize, Buf);
  Buf.Position:=0;

  while AChunkCount > 0 do
  begin
    Dec(AChunkCount);

    CSize  := FindChunkCompressedSize(AFirstChunkIndex);

    CSize := Buf.Read(CBuf^, CSize);
    if AFirstChunkIndex mod Data.ResetInterval = 0 then
    begin
      LZXreset(LZX);
      {$IFDEF DEBUG_HELP2}
      WriteLn('Reset LZX Window');
      {$ENDIF}
    end;
    URes := LZXdecompress(LZX, CBuf, UBuf, CSize, Table.BlockSize);
    //CBuf.Size := 0;
    {$IFDEF DEBUG_HELP2}
    WriteLn('Decompress = ', URes);
    {$ENDIF}

    Result.Write(UBuf^, Table.BlockSize);
    Inc(AFirstChunkIndex);
  end;
  Buf.Free;
  Freemem(UBuf);
  Freemem(CBuf);
  Result.Position:=0;
  LZXteardown(LZX);
end;


function TLZXv3Transform.WantData(APrefix: String; ADataStream: TStream; const DataStart,
  DataLength: Integer; AOutStream: TMemoryStream): Boolean;
var
  LZXData: TLZXv3ControlData;
  ResetTable: TLZXv3ResetTable;
  ChunkStart,
  ChunkCount: DWord;
  RawChunks: TStream;
  ChunkDataStart: QWord;
begin
  inherited WantData(APrefix, ADataStream, DataStart, DataLength, AOutStream);
  {$IFDEF DEBUG_HELP2}
  WriteLn('WantData Pre=',APrefix,' DS=', DataStart,' DL=',DataLength);
  {$ENDIF}

  Result := False;
  if not (GetControlData and GetResetTable) then
    Exit;
  {$IFDEF DEBUG_HELP2}
  WriteLn('Got Needed Info');
  {$ENDIF}
  ChunkStart := FindFirstChunkFromOffset(DataStart,DataLength, ChunkCount);

  RawChunks := ExtractChunks(ChunkStart, ChunkCount, ChunkDataStart);

  RawChunks.Position := DataStart-ChunkDataStart;
  AOutStream.CopyFrom(RawChunks, DataLength);
  RawChunks.Free;

  Result := True;
end;

class function TLZXv3Transform.GUID: TGuid;
const
  AGuid: TGuid = '{0A9007C6-4076-11D3-8789-0000F8105754}';
begin
  Result := AGuid;
end;

{ TPlainTransform }

function TPlainTransform.WantData(APrefix: String; ADataStream: TStream; const DataStart, DataLength: Integer;
  AOutStream: TMemoryStream): Boolean;
begin
  inherited WantData(APrefix, ADataStream, DataStart, DataLength, AOutStream);
  ADataStream.Position:=DataStart;
  AOutStream.CopyFrom(ADataStream, DataLength);
  Result := True;
end;

initialization
  RegisteredTransforms.AddTransform(TPlainTransform);
  RegisteredTransforms.AddTransform(TLZXv3Transform);

finalization
  if Assigned(LocTransforms) then
  begin
    while LocTransforms.Count > 0 do
    begin
      if Assigned(PITSFTranformItem(LocTransforms.Items[0])^.Instance) then
          (PITSFTranformItem(LocTransforms.Items[0])^.Instance).Free;
      LocTransforms.Delete(0);
    end;
    LocTransforms.Free;
  end

end.


