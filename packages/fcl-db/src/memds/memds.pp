{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2007 by the Free Pascal development team
    Some modifications (c) 2007 by Martin Schreiber

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$H+}
{$ENDIF}
{
  TMemDataset : In-memory dataset.
  - Has possibility to copy Structure/Data from other dataset.
  - Can load/save to/from stream.
  Ideas taken from THKMemTab Component by Harri Kasulke - Hamburg/Germany
  E-mail: harri.kasulke@okay.net
}

unit memds;

interface

uses
 sysutils, classes, db, types;

const
  // Stream Markers.
  MarkerSize  = SizeOf(Integer);

  smEOF       = 0;
  smFieldDefs = 1;
  smData      = 2;

type
  {$IFNDEF FPC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$i memdsdelphi.inc} // should set ptrint is longint|intptr
		       // & trecordbuffer ( if <2009)
  {$ENDIF}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ptrint = Integer;
  {$ENDIF}

  MDSError=class(Exception);
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
<<<<<<< HEAD
  MDSError=class(Exception);

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { TMemDataset }

  TMemDataset=class(TDataSet)
  private
<<<<<<< HEAD
    type
      TMDSBlobList = class(TFPList)
        public
          procedure Clear; reintroduce;
      end;
    var
      FOpenStream : TStream;
      FFileName : String;
      FFileModified : Boolean;
      FStream: TMemoryStream;
      FRecInfoOffset: integer;
      FRecCount: integer;
      FRecSize: integer;
      FCurrRecNo: integer;
      FIsOpen: boolean;
      FTableIsCreated: boolean;
      FFilterBuffer: TRecordBuffer;
      ffieldoffsets: PInteger;
      ffieldsizes: PInteger;
      FBlobs: TMDSBlobList;

    function GetRecordBufferPointer(p:TRecordBuffer; Pos:Integer):TRecordBuffer;
=======
    FOpenStream : TStream;
    FFileName : String;
    FFileModified : Boolean;
    FStream: TMemoryStream;
    FRecInfoOffset: integer;
    FRecCount: integer;
    FRecSize: integer;
    FRecBufferSize: integer;
    FCurrRecNo: integer;
    FIsOpen: boolean;
    FTableIsCreated: boolean;
    FFilterBuffer: PChar;
    ffieldoffsets: PInteger;
    ffieldsizes: PInteger;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetCharPointer(p:PChar; Pos:Integer):PChar;
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
    function GetIntegerPointer(p:PInteger; Pos:Integer):PInteger;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure calcrecordlayout;
    function  MDSGetRecordOffset(ARecNo: integer): longint;
    function  MDSGetFieldOffset(FieldNo: integer): integer;
    function  MDSGetBufferSize(FieldNo: integer): integer;
<<<<<<< HEAD
<<<<<<< HEAD
    function  MDSGetActiveBuffer(out Buffer: TRecordBuffer): Boolean;
    procedure MDSReadRecord(Buffer:TRecordBuffer;ARecNo:Integer);
    procedure MDSWriteRecord(Buffer:TRecordBuffer;ARecNo:Integer);
    procedure MDSAppendRecord(Buffer:TRecordBuffer);
    function  MDSFilterRecord(Buffer:TRecordBuffer): Boolean;
    function  MDSLocateRecord(const KeyFields: string; const KeyValues: Variant; Options: TLocateOptions; out ARecNo: integer): Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
    function  MDSGetActiveBuffer(var Buffer: PChar): Boolean;
    procedure MDSReadRecord(Buffer:PChar;ARecNo:Integer);
    procedure MDSWriteRecord(Buffer:PChar;ARecNo:Integer);
    procedure MDSAppendRecord(Buffer:PChar);
    function  MDSFilterRecord(Buffer: PChar): Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  protected
    // Mandatory
    function  AllocRecordBuffer: TRecordBuffer; override;
    procedure FreeRecordBuffer(var Buffer: TRecordBuffer); override;
    procedure GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    function  GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag; override;
    function  GetFieldData(Field: TField; Buffer: Pointer): Boolean; override;
    function  GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult; override;
    function  GetRecordSize: Word; override;
    procedure InternalAddRecord(Buffer: Pointer; DoAppend: Boolean); override;
    procedure InternalClose; override;
    procedure InternalDelete; override;
    procedure InternalFirst; override;
    procedure InternalGotoBookmark(ABookmark: Pointer); override;
    procedure InternalInitFieldDefs; override;
    procedure InternalInitRecord(Buffer: TRecordBuffer); override;
    procedure ClearCalcFields(Buffer: TRecordBuffer); override;
    procedure InternalLast; override;
    procedure InternalOpen; override;
    procedure InternalPost; override;
    procedure InternalSetToRecord(Buffer: TRecordBuffer); override;
    function  IsCursorOpen: Boolean; override;
    procedure SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag); override;
    procedure SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    procedure SetFieldData(Field: TField; Buffer: Pointer); override;

    // Optional.
    function GetRecordCount: Integer; override;
    procedure SetRecNo(Value: Integer); override;
    function GetRecNo: Integer; override;

    // Own.
    procedure SetFilterText(AValue: string); //silently drops filter
    Procedure RaiseError(Fmt : String; Args : Array of const);
    Procedure CheckMarker(F : TStream; Marker : Integer);
    Procedure WriteMarker(F : TStream; Marker : Integer);
    Procedure ReadFieldDefsFromStream(F : TStream);
    Procedure SaveFieldDefsToStream(F : TStream);
    // These should be overridden if you want to load more data.
    // E.g. index defs.
    Procedure LoadDataFromStream(F : TStream); virtual;
    // If SaveData=False, a size 0 block should be written.
    Procedure SaveDataToStream(F : TStream; SaveData : Boolean); virtual;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
  public
    constructor Create(AOwner:TComponent); override;
    destructor Destroy; override;
    function BookmarkValid(ABookmark: TBookmark): Boolean; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint; override;
    function CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream; override;
    function Locate(const KeyFields: string; const KeyValues: Variant; Options: TLocateOptions): boolean; override;
    function Lookup(const KeyFields: string; const KeyValues: Variant; const ResultFields: string): Variant; override;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure CreateTable;
>>>>>>> graemeg/fixes_2_2

    procedure CreateTable;
    Function  DataSize : Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure Clear(ClearDefs : Boolean);{$IFNDEF FPC} overload; {$ENDIF}
    Procedure Clear;{$IFNDEF FPC} overload; {$ENDIF}
=======

    procedure Clear(ClearDefs : Boolean);{$IFNDEF FPC} overload; {$ENDIF}
    procedure Clear;{$IFNDEF FPC} overload; {$ENDIF}
>>>>>>> graemeg/cpstrnew
=======

    procedure Clear(ClearDefs : Boolean);{$IFNDEF FPC} overload; {$ENDIF}
    procedure Clear;{$IFNDEF FPC} overload; {$ENDIF}
>>>>>>> graemeg/cpstrnew
=======

    procedure Clear(ClearDefs : Boolean);{$IFNDEF FPC} overload; {$ENDIF}
    procedure Clear;{$IFNDEF FPC} overload; {$ENDIF}
>>>>>>> graemeg/cpstrnew
=======

    procedure Clear(ClearDefs : Boolean);{$IFNDEF FPC} overload; {$ENDIF}
    procedure Clear;{$IFNDEF FPC} overload; {$ENDIF}
>>>>>>> origin/cpstrnew
    Procedure SaveToFile(AFileName : String);{$IFNDEF FPC} overload; {$ENDIF}
    Procedure SaveToFile(AFileName : String; SaveData : Boolean);{$IFNDEF FPC} overload; {$ENDIF}
    Procedure SaveToStream(F : TStream); {$IFNDEF FPC} overload; {$ENDIF}
    Procedure SaveToStream(F : TStream; SaveData : Boolean);{$IFNDEF FPC} overload; {$ENDIF}
    Procedure LoadFromStream(F : TStream);
    Procedure LoadFromFile(AFileName : String);
    Procedure CopyFromDataset(DataSet : TDataSet); {$IFNDEF FPC} overload; {$ENDIF}
    Procedure CopyFromDataset(DataSet : TDataSet; CopyData : Boolean); {$IFNDEF FPC} overload; {$ENDIF}

    Property FileModified : Boolean Read FFileModified;
<<<<<<< HEAD
<<<<<<< HEAD
    // TMemDataset does not implement Filter. Please use OnFilter instead.
    Property Filter; unimplemented;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  published
    Property FileName : String Read FFileName Write FFileName;
    property Filtered;
    Property Active;
    Property FieldDefs;
    property BeforeOpen;
    property AfterOpen;
    property BeforeClose;
    property AfterClose;
    property BeforeInsert;
    property AfterInsert;
    property BeforeEdit;
    property AfterEdit;
    property BeforePost;
    property AfterPost;
    property BeforeCancel;
    property AfterCancel;
    property BeforeDelete;
    property AfterDelete;
    property BeforeScroll;
    property AfterScroll;
    property OnDeleteError;
    property OnEditError;
    property OnNewRecord;
    property OnPostError;
    property OnFilterRecord;
  end;

implementation

uses
  DBConst, Variants, FmtBCD;

ResourceString
  SErrFieldTypeNotSupported = 'Fieldtype of Field "%s" not supported.';
  SErrBookMarkNotFound      = 'Bookmark %d not found.';
  SErrInvalidDataStream     = 'Error in data stream at position %d';
  SErrInvalidMarkerAtPos    = 'Wrong data stream marker at position %d. Got %d, expected %d';
  SErrNoFileName            = 'Filename must not be empty.';

<<<<<<< HEAD
<<<<<<< HEAD
type
  TMDSRecInfo=record
    Bookmark: Longint;
    BookmarkFlag: TBookmarkFlag;
  end;
  PRecInfo=^TMDSRecInfo;

  TMDSBlobField = record
    Buffer: Pointer;  // pointer to memory allocated for Blob data
    Size: PtrInt;     // size of Blob data
  end;

  { TMDSBlobStream }

  TMDSBlobStream = class(TStream)
    private
      FField      : TBlobField;
      FDataSet    : TMemDataset;
      FBlobField  : TMDSBlobField;
      FPosition   : PtrInt;
      FModified   : boolean;
      procedure AllocBlobField(NewSize: PtrInt);
      procedure FreeBlobField;
    public
      constructor Create(Field: TField; Mode: TBlobStreamMode);
      destructor Destroy; override;
      function Seek(const Offset: int64; Origin: TSeekOrigin): int64; override;
      function Read(var Buffer; Count: Longint): Longint; override;
      function Write(const Buffer; Count: Longint): Longint; override;
  end;

Const
  SizeRecInfo = SizeOf(TMDSRecInfo);

=======
Const
  SizeRecInfo = SizeOf(TMTRecInfo);
>>>>>>> graemeg/fixes_2_2
=======
Const
  SizeRecInfo = SizeOf(TMTRecInfo);
>>>>>>> origin/fixes_2_2

procedure unsetfieldisnull(nullmask: pbyte; const x: integer);

begin
 inc(nullmask,(x shr 3));
 nullmask^:= nullmask^ or (1 shl (x and 7));
end;


procedure setfieldisnull(nullmask: pbyte; const x: integer);

begin
 inc(nullmask,(x shr 3));
 nullmask^:= nullmask^ and Not (1 shl (x and 7));
end;


function getfieldisnull(nullmask: pbyte; const x: integer): boolean;

begin
 inc(nullmask,(x shr 3));
 result:= nullmask^ and (1 shl (x and 7)) = 0;
end;


{ ---------------------------------------------------------------------
    Stream functions
  ---------------------------------------------------------------------}

Function ReadInteger(S : TStream) : Integer;

begin
  S.ReadBuffer(Result,SizeOf(Result));
end;

Function ReadString(S : TStream) : String;

Var
  L : Integer;

begin
  L:=ReadInteger(S);
  Setlength(Result,L);
  If (L<>0) then
    S.ReadBuffer(Result[1],L);
end;

Procedure WriteInteger(S : TStream; Value : Integer);

begin
  S.WriteBuffer(Value,SizeOf(Value));
end;

Procedure WriteString(S : TStream; Value : String);

Var
  L : Integer;

begin
  L:=Length(Value);
  WriteInteger(S,Length(Value));
  If (L<>0) then
    S.WriteBuffer(Value[1],L);
end;


{ TMDSBlobStream }

constructor TMDSBlobStream.Create(Field: TField; Mode: TBlobStreamMode);
begin
  FField := Field as TBlobField;
  FDataSet := Field.DataSet as TMemDataset;
  if not Field.GetData(@FBlobField) then // IsNull
  begin
    FBlobField.Buffer := nil;
    FBlobField.Size := 0;
  end;

  if Mode = bmWrite then
    // release existing Blob
    FreeBlobField;
end;

destructor TMDSBlobStream.Destroy;
begin
  if FModified then
  begin
    if FBlobField.Size = 0 then // Empty blob = IsNull
      FField.SetData(nil)
    else
      FField.SetData(@FBlobField);
  end;
  inherited;
end;

procedure TMDSBlobStream.FreeBlobField;
begin
  FDataSet.FBlobs.Remove(FBlobField.Buffer);
  FreeMem(FBlobField.Buffer, FBlobField.Size);
  FBlobField.Buffer := nil;
  FBlobField.Size := 0;
  FModified := True;
end;

procedure TMDSBlobStream.AllocBlobField(NewSize: PtrInt);
begin
  FDataSet.FBlobs.Remove(FBlobField.Buffer);
  ReAllocMem(FBlobField.Buffer, NewSize);
  FDataSet.FBlobs.Add(FBlobField.Buffer);
  FModified := True;
end;

function TMDSBlobStream.Seek(const Offset: int64; Origin: TSeekOrigin): int64;
begin
  Case Origin of
    soBeginning : FPosition := Offset;
    soEnd       : FPosition := FBlobField.Size + Offset;
    soCurrent   : FPosition := FPosition + Offset;
  end;
  Result := FPosition;
end;

function TMDSBlobStream.Read(var Buffer; Count: Longint): Longint;
var p: Pointer;
begin
  if FPosition + Count > FBlobField.Size then
    Count := FBlobField.Size - FPosition;
  p := FBlobField.Buffer + FPosition;
  Move(p^, Buffer, Count);
  Inc(FPosition, Count);
  Result := Count;
end;

function TMDSBlobStream.Write(const Buffer; Count: Longint): Longint;
var p: Pointer;
begin
  AllocBlobField(FPosition+Count);
  p := FBlobField.Buffer + FPosition;
  Move(Buffer, p^, Count);
  Inc(FBlobField.Size, Count);
  Inc(FPosition, Count);
  Result := Count;
end;


{ TMemDataset.TMDSBlobList }

procedure TMemDataset.TMDSBlobList.Clear;
var i: integer;
begin
  for i:=0 to Count-1 do FreeMem(Items[i]);
  inherited Clear;
end;

{ ---------------------------------------------------------------------
    TMemDataset
  ---------------------------------------------------------------------}

constructor TMemDataset.Create(AOwner:TComponent);

begin
  inherited Create(AOwner);
  FStream:=TMemoryStream.Create;
  FRecCount:=0;
  FRecSize:=0;
  FRecInfoOffset:=0;
  FCurrRecNo:=-1;
<<<<<<< HEAD
<<<<<<< HEAD
  BookmarkSize := sizeof(Longint);
  FBlobs := TMDSBlobList.Create;
=======
=======
>>>>>>> origin/fixes_2_2
  BookmarkSize := sizeof(TMTRecInfo);
  FIsOpen:=False;
>>>>>>> graemeg/fixes_2_2
end;

destructor TMemDataset.Destroy;
begin
  FStream.Free;
  FreeMem(FFieldOffsets);
  FreeMem(FFieldSizes);
<<<<<<< HEAD
<<<<<<< HEAD
  FBlobs.Clear;
  FBlobs.Free;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  inherited Destroy;
end;

function TMemDataset.BookmarkValid(ABookmark: TBookmark): Boolean;
var
  ReqBookmark: integer;
begin
  Result := False;
  if ABookMark=nil then exit;
  ReqBookmark:=PInteger(ABookmark)^;
  Result := (ReqBookmark>=0) and (ReqBookmark<FRecCount);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TMemDataset.CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint;
const r: array[Boolean, Boolean] of ShortInt = ((2,-1),(1,0));
begin
  Result := r[Bookmark1=nil, Bookmark2=nil];
  if Result = 2 then
    Result := PInteger(Bookmark1)^ - PInteger(Bookmark2)^;
end;

function TMemDataset.CreateBlobStream(Field: TField; Mode: TBlobStreamMode
  ): TStream;
begin
  // Blobs are not saved to stream/file !
  if Mode = bmWrite then
    begin
    if not (State in [dsEdit, dsInsert, dsFilter, dsCalcFields]) then
      DatabaseErrorFmt(SNotEditing, [Name], Self);
    if Field.ReadOnly and not (State in [dsSetKey, dsFilter]) then
      DatabaseErrorFmt(SReadOnlyField, [Field.DisplayName]);
    end;
  Result := TMDSBlobStream.Create(Field, Mode);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TMemDataset.MDSGetRecordOffset(ARecNo: integer): longint;
begin
  Result:=FRecSize*ARecNo
end;

function TMemDataset.MDSGetFieldOffset(FieldNo: integer): integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result:= getIntegerPointer(ffieldoffsets, fieldno-1)^;
=======
 result:= getIntegerpointer(ffieldoffsets, fieldno-1)^;
>>>>>>> graemeg/cpstrnew
=======
 result:= getIntegerpointer(ffieldoffsets, fieldno-1)^;
>>>>>>> graemeg/cpstrnew
=======
 result:= getIntegerpointer(ffieldoffsets, fieldno-1)^;
>>>>>>> graemeg/cpstrnew
=======
 result:= getIntegerpointer(ffieldoffsets, fieldno-1)^;
>>>>>>> origin/cpstrnew
=======
 result:= ffieldoffsets[fieldno-1];
>>>>>>> graemeg/fixes_2_2
=======
 result:= ffieldoffsets[fieldno-1];
>>>>>>> origin/fixes_2_2
end;

procedure TMemDataset.RaiseError(Fmt: String; Args: array of const);

begin
  Raise MDSError.CreateFmt(Fmt,Args);
end;

function TMemDataset.MDSGetBufferSize(FieldNo: integer): integer;
var
<<<<<<< HEAD
<<<<<<< HEAD
 FD: TFieldDef;
begin
<<<<<<< HEAD
 FD := FieldDefs.Items[FieldNo-1];
 case FD.DataType of
  ftString,
    ftGuid:   result:=FD.Size+1;
  ftFixedChar:result:=FD.Size+1;
=======
 dt1:= FieldDefs.Items[FieldNo-1].Datatype;
 case dt1 of
  ftString:   result:=FieldDefs.Items[FieldNo-1].Size+1;
  ftFixedChar:result:=FieldDefs.Items[FieldNo-1].Size+1;
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
  ftBoolean:  result:=SizeOf(Wordbool);
  ftCurrency,
  ftFloat:    result:=SizeOf(Double);
  ftBCD:      result:=SizeOf(currency);
  ftLargeInt: result:=SizeOf(int64);
  ftSmallInt: result:=SizeOf(SmallInt);
  ftWord,
  ftAutoInc,
=======
=======
>>>>>>> origin/fixes_2_2
 dt1: tfieldtype;
begin
 dt1:= FieldDefs.Items[FieldNo-1].Datatype;
 case dt1 of
  ftString:   result:=FieldDefs.Items[FieldNo-1].Size+1;
  ftBoolean:  result:=SizeOf(Wordbool);
  ftFloat:    result:=SizeOf(Double);
  ftLargeInt: result:=SizeOf(int64);
  ftSmallInt: result:=SizeOf(SmallInt);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ftInteger:  result:=SizeOf(longint);
  ftDateTime,
    ftTime,
    ftDate:   result:=SizeOf(TDateTime);
<<<<<<< HEAD
<<<<<<< HEAD
  ftFmtBCD:   result:=SizeOf(TBCD);
  ftWideString, ftFixedWideChar:
              result:=(FD.Size+1)*SizeOf(WideChar);
  ftBytes:    result := FD.Size;
  ftVarBytes: result := FD.Size + SizeOf(Word);
  ftBlob, ftMemo, ftWideMemo:
              result := SizeOf(TMDSBlobField);
 else
  RaiseError(SErrFieldTypeNotSupported,[FD.Name]);
=======
 else
  RaiseError(SErrFieldTypeNotSupported,[FieldDefs.Items[FieldNo-1].Name]);
>>>>>>> graemeg/fixes_2_2
=======
 else
  RaiseError(SErrFieldTypeNotSupported,[FieldDefs.Items[FieldNo-1].Name]);
>>>>>>> origin/fixes_2_2
 end;
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
 Result:=Align(Result,4);
{$ENDIF}
end;

function TMemDataset.MDSGetActiveBuffer(out Buffer: TRecordBuffer): Boolean;
begin
<<<<<<< HEAD
  case State of
    dsEdit,
    dsInsert:
      Buffer:=ActiveBuffer;
    dsFilter:
      Buffer:=FFilterBuffer;
    dsCalcFields:
      Buffer:=CalcBuffer;
    else
      if IsEmpty then
        Buffer:=nil
      else
        Buffer:=ActiveBuffer;
  end;
  Result := Buffer<>nil;
=======
 case State of
   dsBrowse:
     if IsEmpty then
       Buffer:=nil
     else
       Buffer:=ActiveBuffer;
  dsEdit,
  dsInsert:
     Buffer:=ActiveBuffer;
  dsFilter:
     Buffer:=FFilterBuffer;
 else
   Buffer:=nil;
 end;
 Result:=(Buffer<>nil);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TMemDataset.MDSReadRecord(Buffer:TRecordBuffer;ARecNo:Integer);   //Reads a Rec from Stream in Buffer
begin
  FStream.Position:=MDSGetRecordOffset(ARecNo);
  FStream.ReadBuffer(Buffer^, FRecSize);
end;

procedure TMemDataset.MDSWriteRecord(Buffer:TRecordBuffer;ARecNo:Integer);  //Writes a Rec from Buffer to Stream
begin
  FStream.Position:=MDSGetRecordOffset(ARecNo);
  FStream.WriteBuffer(Buffer^, FRecSize);
  FFileModified:=True;
end;

procedure TMemDataset.MDSAppendRecord(Buffer:TRecordBuffer);   //Appends a Rec (from Buffer) to Stream
begin
  FStream.Position:=MDSGetRecordOffset(FRecCount);
  FStream.WriteBuffer(Buffer^, FRecSize);
  FFileModified:=True;
end;

//Abstract Overrides
function TMemDataset.AllocRecordBuffer: TRecordBuffer;
begin
  GetMem(Result, FRecSize+CalcFieldsSize);
end;

procedure TMemDataset.FreeRecordBuffer (var Buffer: TRecordBuffer);
begin
  FreeMem(Buffer);
end;

procedure TMemDataset.InternalInitRecord(Buffer: TRecordBuffer);
begin
  FillChar(Buffer^,FRecSize,0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TMemDataset.ClearCalcFields(Buffer: TRecordBuffer);
begin
  FillChar(Buffer[RecordSize], CalcFieldsSize, 0);
=======
var
  I : integer;

begin
 fillchar(buffer^,frecsize,0);
>>>>>>> graemeg/fixes_2_2
=======
var
  I : integer;

begin
 fillchar(buffer^,frecsize,0);
>>>>>>> origin/fixes_2_2
end;

procedure TMemDataset.InternalDelete;

Var
  TS : TMemoryStream;

begin
  if (FCurrRecNo<0) or (FCurrRecNo>=FRecCount) then
    Exit;
  // Very inefficient. We should simply move the last part closer to the beginning in
  // The FStream.
  TS:=TMemoryStream.Create;
  Try
    if FCurrRecNo>0 then
      begin
      FStream.Position:=MDSGetRecordOffset(0);      //Delete Rec
      if FCurrRecNo<FRecCount-1 then
        begin
        TS.CopyFrom(FStream, MDSGetRecordOffset(FCurrRecNo)-MDSGetRecordOffset(0));
        FStream.Position:=MDSGetRecordOffset(FCurrRecNo+1);
        TS.CopyFrom(FStream,(MDSGetRecordOffset(FRecCount))-MDSGetRecordOffset(FCurrRecNo+1));
        end
      else
        TS.CopyFrom(FStream,MDSGetRecordOffset(FRecCount-1));
      end
    else
      begin                                  //Delete first Rec
      FStream.Position:=MDSGetRecordOffset(FCurrRecNo+1);
      TS.CopyFrom(FStream,(MDSGetRecordOffset(FRecCount))-MDSGetRecordOffset(FCurrRecNo+1));
      end;
    FStream.LoadFromStream(TS);
    Dec(FRecCount);
    if FRecCount=0 then
      FCurrRecNo:=-1
    else
      if FCurrRecNo>=FRecCount then FCurrRecNo:=FRecCount-1;
  Finally
    TS.Free;
  end;
  FFileModified:=True;
end;

procedure TMemDataset.InternalInitFieldDefs;

begin
  If (FOpenStream<>Nil) then
    ReadFieldDefsFromStream(FOpenStream);
end;

procedure TMemDataset.CheckMarker(F: TStream; Marker: Integer);

Var
  I,P : Integer;

begin
  P:=F.Position;
  If F.Read(I,MarkerSize)<>MarkerSize then
    RaiseError(SErrInvalidDataStream,[P])
  else
    if (I<>Marker) then
      RaiseError(SErrInvalidMarkerAtPos,[P,I,Marker]);
end;

procedure TMemDataset.ReadFieldDefsFromStream(F : TStream);

Var
  I,ACount : Integer;
  FN : String;
  FS : Integer;
  B : Boolean;
  FT : TFieldType;

begin
  CheckMarker(F,smFieldDefs);
  FieldDefs.Clear;
  ACount:=ReadInteger(F);
  For I:=1 to ACount do
    begin
    FN:=ReadString(F);
    FS:=ReadInteger(F);
    FT:=TFieldType(ReadInteger(F));
    B:=ReadInteger(F)<>0;
    TFieldDef.Create(FieldDefs,FN,ft,FS,B,I);
    end;
  FTableIsCreated:=False;
end;

procedure TMemDataset.InternalFirst;
begin
  FCurrRecNo:=-1;
end;

procedure TMemDataset.InternalLast;
begin
  FCurrRecNo:=FRecCount;
end;

procedure TMemDataset.InternalOpen;

begin
  if not FTableIsCreated then CreateTable;
  If (FFileName<>'') then
    FOpenStream:=TFileStream.Create(FFileName,fmOpenRead);
  Try
    InternalInitFieldDefs;
    if DefaultFields then
      CreateFields;
    BindFields(True); // BindFields computes CalcFieldsSize
    if not FTableIsCreated then
      CreateTable;
    FCurrRecNo:=-1;
    If (FOpenStream<>Nil) then
      begin
      LoadDataFromStream(FOpenStream);
      CheckMarker(FOpenStream,smEOF);
      end;
  Finally
    FreeAndNil(FOpenStream);
  end;
  FIsOpen:=True;
end;

procedure TMemDataset.LoadDataFromStream(F: TStream);

Var
  Size : Integer;

begin
  CheckMarker(F,smData);
  Size:=ReadInteger(F);
  FBlobs.Clear;
  FStream.Clear;
  FStream.CopyFrom(F,Size);
  FRecCount:=Size div FRecSize;
  FCurrRecNo:=-1;
end;

procedure TMemDataset.LoadFromStream(F: TStream);

begin
  Close;
  ReadFieldDefsFromStream(F);
  CreateTable;
  LoadDataFromStream(F);
  CheckMarker(F,smEOF);
  FFileModified:=False;
end;

procedure TMemDataset.LoadFromFile(AFileName: String);

Var
  F : TFileStream;

begin
  F:=TFileStream.Create(AFileName,fmOpenRead);
  Try
    LoadFromStream(F);
  Finally
    F.Free;
  end;
end;


procedure TMemDataset.SaveToFile(AFileName: String);

begin
  SaveToFile(AFileName,True);
end;

procedure TMemDataset.SaveToFile(AFileName: String; SaveData: Boolean);

Var
  F : TFileStream;

begin
  If (AFileName='') then
    RaiseError(SErrNoFileName,[]);
  F:=TFileStream.Create(AFileName,fmCreate);
  try
    SaveToStream(F,SaveData);
  Finally
    F.Free;
  end;
end;

procedure TMemDataset.WriteMarker(F: TStream; Marker: Integer);

begin
  Writeinteger(F,Marker);
end;

procedure TMemDataset.SaveToStream(F: TStream);

begin
  SaveToStream(F,True);
end;

procedure TMemDataset.SaveToStream(F: TStream; SaveData: Boolean);

begin
  SaveFieldDefsToStream(F);
  If SaveData then
    SaveDataToStream(F,SaveData);
  WriteMarker(F,smEOF);
end;

procedure TMemDataset.SaveFieldDefsToStream(F: TStream);

Var
  I : Integer;
  FD : TFieldDef;

begin
  WriteMarker(F,smFieldDefs);
  WriteInteger(F,FieldDefs.Count);
  For I:=1 to FieldDefs.Count do
    begin
    FD:=FieldDefs[I-1];
    WriteString(F,FD.Name);
    WriteInteger(F,FD.Size);
    WriteInteger(F,Ord(FD.DataType));
    WriteInteger(F,Ord(FD.Required));
    end;
end;

procedure TMemDataset.SaveDataToStream(F: TStream; SaveData: Boolean);

begin
  if SaveData then
    begin
    WriteMarker(F,smData);
    WriteInteger(F,FStream.Size);
    FStream.Position:=0;
    F.CopyFrom(FStream,FStream.Size);
    FFileModified:=False;
    end
  else
    begin
    WriteMarker(F,smData);
    WriteInteger(F,0);
    end;
end;

procedure TMemDataset.InternalClose;

begin
 if (FFileModified) and (FFileName<>'') then begin
  SaveToFile(FFileName,True);
 end;
 FIsOpen:=False;
 FFileModified:=False;
 // BindFields(False);
<<<<<<< HEAD
<<<<<<< HEAD
 if DefaultFields then
  DestroyFields;
=======
 if DefaultFields then begin
  DestroyFields;
 end;
>>>>>>> graemeg/fixes_2_2
=======
 if DefaultFields then begin
  DestroyFields;
 end;
>>>>>>> origin/fixes_2_2
end;

procedure TMemDataset.InternalPost;
begin
  CheckActive;
  if not (State in [dsEdit, dsInsert]) then
    Exit;
  inherited InternalPost;
  if (State=dsEdit) then
    MDSWriteRecord(ActiveBuffer, FCurrRecNo)
  else
    InternalAddRecord(ActiveBuffer,True);
end;

function TMemDataset.IsCursorOpen: Boolean;

begin
  Result:=FIsOpen;
end;

<<<<<<< HEAD
function TMemDataset.GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
=======
function TMemDataset.GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

var
  Accepted: Boolean;

begin
  Result:=grOk;
  Accepted:=False;
  if (FRecCount<1) then
    begin
    Result:=grEOF;
    exit;
    end;
  repeat
    case GetMode of
      gmCurrent:
        if (FCurrRecNo>=FRecCount) or (FCurrRecNo<0) then
          Result:=grError;
      gmNext:
        if (FCurrRecNo<FRecCount-1) then
          Inc(FCurrRecNo)
        else
          Result:=grEOF;
      gmPrior:
        if (FCurrRecNo>0) then
          Dec(FCurrRecNo)
        else
          result:=grBOF;
    end;
    if result=grOK then
      begin
      MDSReadRecord(Buffer, FCurrRecNo);
      PRecInfo(Buffer+FRecInfoOffset)^.Bookmark:=FCurrRecNo;
      PRecInfo(Buffer+FRecInfoOffset)^.BookmarkFlag:=bfCurrent;
<<<<<<< HEAD
<<<<<<< HEAD
      GetCalcFields(Buffer);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      if (Filtered) then
        Accepted:=MDSFilterRecord(Buffer) //Filtering
      else
        Accepted:=True;
      if (GetMode=gmCurrent) and not Accepted then
        result:=grError;
      end;
  until (result<>grOK) or Accepted;
end;

function TMemDataset.GetFieldData(Field: TField; Buffer: Pointer): Boolean;
var
<<<<<<< HEAD
<<<<<<< HEAD
 SrcBuffer: TRecordBuffer;
 I: integer;
begin
 I:= Field.FieldNo - 1;
 result := MDSGetActiveBuffer(SrcBuffer);
 if not result then Exit;

 if I >= 0 then
   begin
<<<<<<< HEAD
   result := not getfieldisnull(pointer(srcbuffer),I);
   if result and assigned(Buffer) then
     Move(GetRecordBufferPointer(SrcBuffer, GetIntegerPointer(ffieldoffsets,I)^)^, Buffer^, GetIntegerPointer(FFieldSizes, I)^);
   end
 else // Calculated, Lookup
   begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Inc(SrcBuffer, RecordSize + Field.Offset);
   result := Boolean(SrcBuffer[0]);
   if result and assigned(Buffer) then
     Move(SrcBuffer[1], Buffer^, Field.DataSize);
=======
   Move(getcharpointer(SrcBuffer,getintegerpointer(ffieldoffsets,I)^)^, Buffer^,GetIntegerPointer(FFieldSizes, I)^);
>>>>>>> graemeg/cpstrnew
=======
   Move(getcharpointer(SrcBuffer,getintegerpointer(ffieldoffsets,I)^)^, Buffer^,GetIntegerPointer(FFieldSizes, I)^);
>>>>>>> graemeg/cpstrnew
=======
   Move(getcharpointer(SrcBuffer,getintegerpointer(ffieldoffsets,I)^)^, Buffer^,GetIntegerPointer(FFieldSizes, I)^);
>>>>>>> graemeg/cpstrnew
=======
   Move(getcharpointer(SrcBuffer,getintegerpointer(ffieldoffsets,I)^)^, Buffer^,GetIntegerPointer(FFieldSizes, I)^);
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 SrcBuffer: PChar;
 I: integer;
begin
 I:= Field.FieldNo - 1;
 result:= (I >= 0) and MDSGetActiveBuffer(SrcBuffer) and 
          not getfieldisnull(pointer(srcbuffer),I);
 if result and (buffer <> nil) then 
   begin
   Move((SrcBuffer+ffieldoffsets[I])^, Buffer^,FFieldSizes[I]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   end;
end;

procedure TMemDataset.SetFieldData(Field: TField; Buffer: Pointer);
var
<<<<<<< HEAD
<<<<<<< HEAD
 DestBuffer: TRecordBuffer;
=======
 DestBuffer: PChar;
>>>>>>> graemeg/fixes_2_2
=======
 DestBuffer: PChar;
>>>>>>> origin/fixes_2_2
 I,J: integer;

begin
 I:= Field.FieldNo - 1;
<<<<<<< HEAD
<<<<<<< HEAD
 if not MDSGetActiveBuffer(DestBuffer) then Exit;

 if I >= 0 then
   begin
<<<<<<< HEAD
   if State in [dsEdit, dsInsert, dsNewValue] then
     Field.Validate(Buffer);
   if Buffer = nil then
     setfieldisnull(pointer(DestBuffer),I)
   else
     begin
     unsetfieldisnull(pointer(DestBuffer),I);
     J:=GetIntegerPointer(FFieldSizes, I)^;
     if Field.DataType=ftString then
       Dec(J); // Do not move terminating 0, which is in the size.
     Move(Buffer^, GetRecordBufferPointer(DestBuffer, getIntegerPointer(FFieldOffsets, I)^)^, J);
=======
=======
 if (I >= 0) and  MDSGetActiveBuffer(DestBuffer) then 
   begin
>>>>>>> graemeg/fixes_2_2
=======
 if (I >= 0) and  MDSGetActiveBuffer(DestBuffer) then 
   begin
>>>>>>> origin/fixes_2_2
   if buffer = nil then 
     setfieldisnull(pointer(destbuffer),I)
   else 
     begin 
     unsetfieldisnull(pointer(destbuffer),I);
<<<<<<< HEAD
<<<<<<< HEAD
     J:=GetIntegerPointer(FFieldSizes, I)^;
     if Field.DataType=ftString then
       Dec(J); // Do not move terminating 0, which is in the size.
     Move(Buffer^,GetCharPointer(DestBuffer, getIntegerPointer(FFieldOffsets, I)^)^,J);
     dataevent(defieldchange,ptrint(field));
>>>>>>> graemeg/cpstrnew
     end;
   end
 else // Calculated, Lookup
   begin
   Inc(DestBuffer, RecordSize + Field.Offset);
   Boolean(DestBuffer[0]) := Buffer <> nil;
   if assigned(Buffer) then
     Move(Buffer^, DestBuffer[1], Field.DataSize);
   end;

 if not (State in [dsCalcFields, dsFilter, dsNewValue]) then
   DataEvent(deFieldChange, PtrInt(Field));
=======
=======
>>>>>>> origin/fixes_2_2
     J:=FFieldSizes[I];
     if Field.DataType=ftString then
       Dec(J); // Do not move terminating 0, which is in the size.
     Move(Buffer^,(DestBuffer+FFieldOffsets[I])^,J);
     dataevent(defieldchange,ptrint(field));
     end;
   end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TMemDataset.GetRecordSize: Word;

begin
 Result:= FRecSize;
end;

procedure TMemDataset.InternalGotoBookmark(ABookmark: Pointer);

var
  ReqBookmark: integer;

begin
  ReqBookmark:=PInteger(ABookmark)^;
  if (ReqBookmark>=0) and (ReqBookmark<FRecCount) then
    FCurrRecNo:=ReqBookmark
  else
    RaiseError(SErrBookMarkNotFound,[ReqBookmark]);
end;

procedure TMemDataset.InternalSetToRecord(Buffer: TRecordBuffer);

var
  ReqBookmark: integer;

begin
  ReqBookmark:=PRecInfo(Buffer+FRecInfoOffset)^.Bookmark;
  InternalGotoBookmark (@ReqBookmark);
end;

<<<<<<< HEAD
function TMemDataset.GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag;
=======
function TMemDataset.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Result:=PRecInfo(Buffer+FRecInfoOffset)^.BookmarkFlag;
end;

<<<<<<< HEAD
procedure TMemDataset.SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag);
=======
procedure TMemDataset.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  PRecInfo(Buffer+FRecInfoOffset)^.BookmarkFlag := Value;
end;

procedure TMemDataset.GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);

begin
  if Data<>nil then
    PInteger(Data)^:=PRecInfo(Buffer+FRecInfoOffset)^.Bookmark;
end;

procedure TMemDataset.SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);

begin
  if Data<>nil then
    PRecInfo(Buffer+FRecInfoOffset)^.Bookmark:=PInteger(Data)^
  else
    PRecInfo(Buffer+FRecInfoOffset)^.Bookmark:=0;
end;

function TMemDataset.MDSFilterRecord(Buffer: TRecordBuffer): Boolean;

var
  SaveState: TDatasetState;

begin
  Result:=True;
  if not Assigned(OnFilterRecord) then
    Exit;
  SaveState:=SetTempState(dsFilter);
  Try
    FFilterBuffer:=Buffer;
    OnFilterRecord(Self,Result);
  Finally  
    RestoreState(SaveState);
  end;  
end;

function TMemDataset.DataSize: Integer;

begin
  Result:=FStream.Size;
end;

procedure TMemDataset.Clear;

begin
  Clear(True);
end;

procedure TMemDataset.Clear(ClearDefs : Boolean);

begin
  FBlobs.Clear;
  FStream.Clear;
  FRecCount:=0;
  FCurrRecNo:=-1;
  if Active then
    Resync([]);
  If ClearDefs then
    begin
    Close;
    FieldDefs.Clear;
    FTableIsCreated:=False;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TMemDataset.calcrecordlayout;
var
  i,Count : integer;
begin
 Count := FieldDefs.Count;
 // Avoid mem-leak if CreateTable is called twice
<<<<<<< HEAD
 FreeMem(FFieldOffsets);
 Freemem(FFieldSizes);
=======
 FreeMem(ffieldoffsets);
 Freemem(ffieldsizes);
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
 {$IFDEF FPC}
 FFieldOffsets:=getmem(Count*sizeof(integer));
 FFieldSizes:=getmem(Count*sizeof(integer));
 {$ELSE}
 getmem(FFieldOffsets, Count*sizeof(integer));
 getmem(FFieldSizes, Count*sizeof(integer));
 {$ENDIF}
 FRecSize:= (Count+7) div 8; //null mask
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
 FRecSize:=Align(FRecSize,4);
{$ENDIF}
 for i:= 0 to Count-1 do
   begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   GetIntegerPointer(FFieldOffsets, i)^ := FRecSize;
   GetIntegerPointer(FFieldSizes,   i)^ := MDSGetBufferSize(i+1);
   FRecSize:= FRecSize+GetIntegerPointer(FFieldSizes, i)^;
=======
   GetIntegerPointer(ffieldoffsets, i)^ := frecsize;
   GetIntegerPointer(ffieldsizes,   i)^ := MDSGetbufferSize(i+1);
   FRecSize:= FRecSize+GetIntegerPointeR(FFieldSizes, i)^;
>>>>>>> graemeg/cpstrnew
=======
   GetIntegerPointer(ffieldoffsets, i)^ := frecsize;
   GetIntegerPointer(ffieldsizes,   i)^ := MDSGetbufferSize(i+1);
   FRecSize:= FRecSize+GetIntegerPointeR(FFieldSizes, i)^;
>>>>>>> graemeg/cpstrnew
=======
   GetIntegerPointer(ffieldoffsets, i)^ := frecsize;
   GetIntegerPointer(ffieldsizes,   i)^ := MDSGetbufferSize(i+1);
   FRecSize:= FRecSize+GetIntegerPointeR(FFieldSizes, i)^;
>>>>>>> graemeg/cpstrnew
=======
   GetIntegerPointer(ffieldoffsets, i)^ := frecsize;
   GetIntegerPointer(ffieldsizes,   i)^ := MDSGetbufferSize(i+1);
   FRecSize:= FRecSize+GetIntegerPointeR(FFieldSizes, i)^;
>>>>>>> origin/cpstrnew
   end;
 FRecInfoOffset:=FRecSize;
 FRecSize:=FRecSize+SizeRecInfo;
end;

procedure TMemDataset.CreateTable;

begin
  CheckInactive;
  Clear(False);
  calcrecordlayout;
=======
procedure tmemdataset.calcrecordlayout;
var
  i,count : integer;
begin
 Count := fielddefs.count;
 // Avoid mem-leak if CreateTable is called twice
 FreeMem(ffieldoffsets);
 Freemem(ffieldsizes);

 FFieldOffsets:=getmem(Count*sizeof(integer));
 FFieldSizes:=getmem(Count*sizeof(integer));
 FRecSize:= (Count+7) div 8; //null mask
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
 FRecSize:=Align(FRecSize,4);
{$ENDIF}
 for i:= 0 to Count-1 do
   begin
   ffieldoffsets[i] := frecsize;
   ffieldsizes[i] := MDSGetbufferSize(i+1);
   FRecSize:= FRecSize+FFieldSizes[i];
   end;
end;

procedure TMemDataset.CreateTable;

begin
=======
procedure tmemdataset.calcrecordlayout;
var
  i,count : integer;
begin
 Count := fielddefs.count;
 // Avoid mem-leak if CreateTable is called twice
 FreeMem(ffieldoffsets);
 Freemem(ffieldsizes);

 FFieldOffsets:=getmem(Count*sizeof(integer));
 FFieldSizes:=getmem(Count*sizeof(integer));
 FRecSize:= (Count+7) div 8; //null mask
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
 FRecSize:=Align(FRecSize,4);
{$ENDIF}
 for i:= 0 to Count-1 do
   begin
   ffieldoffsets[i] := frecsize;
   ffieldsizes[i] := MDSGetbufferSize(i+1);
   FRecSize:= FRecSize+FFieldSizes[i];
   end;
end;

procedure TMemDataset.CreateTable;

begin
>>>>>>> origin/fixes_2_2
  CheckInactive;
  FStream.Clear;
  FRecCount:=0;
  FCurrRecNo:=-1;
  FIsOpen:=False;
  calcrecordlayout;
  FRecInfoOffset:=FRecSize;
  FRecSize:=FRecSize+SizeRecInfo;
  FRecBufferSize:=FRecSize;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FTableIsCreated:=True;
end;

procedure TMemDataset.InternalAddRecord(Buffer: Pointer; DoAppend: Boolean);

begin
  MDSAppendRecord(ActiveBuffer);
  InternalLast;
  Inc(FRecCount);
end;

procedure TMemDataset.SetRecNo(Value: Integer);
begin
  CheckBrowseMode;
  if (Value>=1) and (Value<=FRecCount) then
    begin
    FCurrRecNo:=Value-1;
    Resync([]);
    end;
end;

function TMemDataset.GetRecNo: Integer;

begin
  UpdateCursorPos;
<<<<<<< HEAD
<<<<<<< HEAD
  if (FCurrRecNo<0) or (FRecCount=0) or (State=dsInsert) then
    Result:=0
=======
  if (FCurrRecNo<0) then
    Result:=1
>>>>>>> graemeg/fixes_2_2
=======
  if (FCurrRecNo<0) then
    Result:=1
>>>>>>> origin/fixes_2_2
  else
    Result:=FCurrRecNo+1;
end;

function TMemDataset.GetRecordCount: Integer;

begin
  CheckActive;
  Result:=FRecCount;
end;

procedure TMemDataset.CopyFromDataset(DataSet: TDataSet);

begin
  CopyFromDataset(Dataset,True);
end;

procedure TMemDataset.CopyFromDataset(DataSet: TDataSet; CopyData: Boolean);

Var
  I  : Integer;
  F,F1,F2 : TField;
  L1,L2  : TList;
  N : String;
  OriginalPosition: TBookMark;

begin
  Clear(True);
  // NOT from FieldDefs. The data may not be available in buffers !!
  For I:=0 to Dataset.FieldCount-1 do
    begin
    F:=Dataset.Fields[I];
    TFieldDef.Create(FieldDefs,F.FieldName,F.DataType,F.Size,F.Required,F.FieldNo);
    end;
  CreateTable;
  If CopyData then
  begin
    Open;
    L1:=TList.Create;
    Try
      L2:=TList.Create;
      Try
        For I:=0 to FieldDefs.Count-1 do
          begin
          N:=FieldDefs[I].Name;
          F1:=FieldByName(N);
          F2:=DataSet.FieldByName(N);
          L1.Add(F1);
          L2.Add(F2);
          end;
        DisableControls;
        Dataset.DisableControls;
        OriginalPosition:=Dataset.GetBookmark;
        Try
          Dataset.Open;
          Dataset.First; //make sure we copy from the beginning
          While not Dataset.EOF do
            begin
            Append;
            For I:=0 to L1.Count-1 do
              begin
              F1:=TField(L1[i]);
              F2:=TField(L2[I]);
              Case F1.DataType of
                ftFixedChar,
                ftString   : F1.AsString:=F2.AsString;
                ftBoolean  : F1.AsBoolean:=F2.AsBoolean;
                ftFloat    : F1.AsFloat:=F2.AsFloat;
                ftLargeInt : F1.AsInteger:=F2.AsInteger;
                ftSmallInt : F1.AsInteger:=F2.AsInteger;
                ftInteger  : F1.AsInteger:=F2.AsInteger;
                ftDate     : F1.AsDateTime:=F2.AsDateTime;
                ftTime     : F1.AsDateTime:=F2.AsDateTime;
                ftDateTime : F1.AsDateTime:=F2.AsDateTime;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                else         F1.AsString:=F2.AsString;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              end;
              end;
            Try
              Post;
            except
              Cancel;
              Raise;
            end;
            Dataset.Next;
            end;
        Finally
          DataSet.GotoBookmark(OriginalPosition); //Return to original record
          Dataset.EnableControls;
          EnableControls;
        end;
      finally
        L2.Free;
      end;
    finally
      l1.Free;
    end;
  end;
end;

function TMemDataset.GetRecordBufferPointer(p:TRecordBuffer; Pos:Integer):TRecordBuffer;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.GetIntegerPointer(p:PInteger; Pos:Integer):PInteger;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.MDSLocateRecord(const KeyFields: string; const KeyValues: Variant;
  Options: TLocateOptions; out ARecNo: integer): Boolean;
var
  SaveState: TDataSetState;
  lstKeyFields: TList;
  Matched: boolean;
  AKeyValues: variant;
  i: integer;
  AField: TField;
  s1,s2: string;
begin
  Result := false;
  SaveState := SetTempState(dsFilter);
  FFilterBuffer := TempBuffer;
  lstKeyFields := TList.Create;
  try
    GetFieldList(lstKeyFields, KeyFields);
    if VarArrayDimCount(KeyValues) = 0 then
      begin
      Matched := lstKeyFields.Count = 1;
      AKeyValues := VarArrayOf([KeyValues]);
      end
    else if VarArrayDimCount(KeyValues) = 1 then
      begin
      Matched := VarArrayHighBound(KeyValues,1) + 1 = lstKeyFields.Count;
      AKeyValues := KeyValues;
      end
    else
      Matched := false;

    if Matched then
    begin
      ARecNo:=0;
      while ARecNo<FRecCount do
      begin
        MDSReadRecord(FFilterBuffer, ARecNo);
        if Filtered then
          Result:=MDSFilterRecord(FFilterBuffer)
        else
          Result:=true;
        // compare field by field
        i:=0;
        while Result and (i<lstKeyFields.Count) do
        begin
          AField := TField(lstKeyFields[i]);
          // string fields
          if AField.DataType in [ftString, ftFixedChar] then
          begin
            s1 := AField.AsString;
            s2 := VarToStr(AKeyValues[i]);
            if loPartialKey in Options then
              s1 := copy(s1, 1, length(s2));
            if loCaseInsensitive in Options then
              Result := AnsiCompareText(s1, s2)=0
            else
              Result := s1=s2;
          end
          // all other fields
          else
            Result := AField.Value=AKeyValues[i];
          inc(i);
        end;
        if Result then
          break;
        inc(ARecNo);
      end;
    end;
  finally
    lstKeyFields.Free;
    RestoreState(SaveState);
  end;
end;

procedure TMemDataset.SetFilterText(AValue: string);
begin
  // Just do nothing; filter is not implemented
end;

function TMemDataset.Locate(const KeyFields: string; const KeyValues: Variant;
  Options: TLocateOptions): boolean;
var
  ARecNo: integer;
begin
  // Call inherited to make sure the dataset is bi-directional
  Result := inherited;
  CheckActive;

  Result:=MDSLocateRecord(KeyFields, KeyValues, Options, ARecNo);
  if Result then begin
    // TODO: generate scroll events if matched record is found
    FCurrRecNo:=ARecNo;
    Resync([]);
  end;
end;

function TMemDataset.Lookup(const KeyFields: string; const KeyValues: Variant;
  const ResultFields: string): Variant;
var
  ARecNo: integer;
  SaveState: TDataSetState;
begin
  if MDSLocateRecord(KeyFields, KeyValues, [], ARecNo) then
  begin
    SaveState := SetTempState(dsCalcFields);
    try
      // FFilterBuffer contains found record
      CalculateFields(FFilterBuffer); // CalcBuffer is set to FFilterBuffer
      Result:=FieldValues[ResultFields];
    finally
      RestoreState(SaveState);
    end;
  end
  else
    Result:=Null;
end;

function TMemDataset.GetCharPointer(p:PChar; Pos:Integer):PChar;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.GetIntegerPointer(p:PInteger; Pos:Integer):PInteger;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.GetCharPointer(p:PChar; Pos:Integer):PChar;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.GetIntegerPointer(p:PInteger; Pos:Integer):PInteger;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.GetCharPointer(p:PChar; Pos:Integer):PChar;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.GetIntegerPointer(p:PInteger; Pos:Integer):PInteger;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.GetCharPointer(p:PChar; Pos:Integer):PChar;
begin
  Result:=p;
  inc(Result, Pos);
end;

function TMemDataset.GetIntegerPointer(p:PInteger; Pos:Integer):PInteger;
begin
  Result:=p;
  inc(Result, Pos);
end;

end.
