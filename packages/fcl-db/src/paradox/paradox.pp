{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2007 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$H+}
{
  TParadox : Dataset wich can handle paradox files, based on PXLib.
<<<<<<< HEAD
<<<<<<< HEAD
  pxlib is an open source C library for handling paradox files. It
  is available from sourceforge:
  http://pxlib.sourceforge.net/
  it must be downloaded and installed separately. The header translations
  for version 0.6.2 of pxlib are available in the pxlib unit in the Free 
  Pascal Packages.
  
  The TParadox component was implemented by Michael Van Canneyt
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
}

unit paradox;

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
  sysutils, classes, db, pxlib, bufdataset_parser;
=======
  sysutils, classes, db, pxlib;
>>>>>>> graemeg/fixes_2_2
=======
  sysutils, classes, db, pxlib;
>>>>>>> origin/fixes_2_2

type
  EParadox=class(Exception);

  { TParadox }

  TParadox = Class(TDataSet)
  private
    FBlobFileName: String;
    FFileName  : String;
    FPXLibrary : String;
    FCurrRecNo : Integer;
    FDoc       : PPX_Doc;
<<<<<<< HEAD
<<<<<<< HEAD
    FFilterBuffer : TRecordBuffer;
=======
    FFilterBuffer : PChar;
>>>>>>> graemeg/fixes_2_2
=======
    FFilterBuffer : PChar;
>>>>>>> origin/fixes_2_2
    FOffsets   : PInteger;
    FTableName : String;
    FInputEncoding : String;
    FTargetEncoding : String;
<<<<<<< HEAD
<<<<<<< HEAD
    FParser         : TBufDatasetParser;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function GetInputEncoding: String;
    function GetTableName: String;
    function GetTargetEncoding: String;
    procedure OpenBlobFile;
    procedure PXAppendRecord(Buffer: Pointer);
<<<<<<< HEAD
<<<<<<< HEAD
    function PXFilterRecord(Buffer: TRecordBuffer): Boolean;
    function PXGetActiveBuffer(var Buffer: TRecordBuffer): Boolean;
=======
    function PXFilterRecord(Buffer: PChar): Boolean;
    function PXGetActiveBuffer(var Buffer: PChar): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
    function PXFilterRecord(Buffer: PChar): Boolean;
    function PXGetActiveBuffer(var Buffer: PChar): Boolean;
>>>>>>> origin/fixes_2_2
    procedure RaiseError(Fmt: String; Args: array of const);
    procedure SetBlobFileName(const AValue: String);
    procedure SetFileName(const AValue: String);
    procedure SetInputEncoding(const AValue: String);
    procedure SetOpenParams;
    procedure SetTableName(const AValue: String);
    procedure SetTargetEncoding(const AValue: String);
<<<<<<< HEAD
<<<<<<< HEAD
    function GetLibStored : Boolean;
  protected
    // Mandatory
    procedure SetFilterText(const Value: String); override; {virtual;}
    procedure SetFiltered(Value: Boolean); override; {virtual;}
    procedure ParseFilter(const AFilter: string);

    function  AllocRecordBuffer: TRecordBuffer; override;
    procedure FreeRecordBuffer(var Buffer: TRecordBuffer); override;
    procedure GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    function  GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag; override;
    function  GetFieldData(Field: TField; Buffer: Pointer): Boolean; override;
    function  GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult; override;
=======
=======
>>>>>>> origin/fixes_2_2
  protected
    // Mandatory
    
    function  AllocRecordBuffer: PChar; override;
    procedure FreeRecordBuffer(var Buffer: PChar); override;
    procedure GetBookmarkData(Buffer: PChar; Data: Pointer); override;
    function  GetBookmarkFlag(Buffer: PChar): TBookmarkFlag; override;
    function  GetFieldData(Field: TField; Buffer: Pointer): Boolean; override;
    function  GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult; override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function  GetRecordSize: Word; override;
    procedure InternalAddRecord(Buffer: Pointer; DoAppend: Boolean); override;
    procedure InternalClose; override;
    procedure InternalDelete; override;
    procedure InternalFirst; override;
    procedure InternalGotoBookmark(ABookmark: Pointer); override;
    procedure InternalInitFieldDefs; override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure InternalInitRecord(Buffer: TRecordBuffer); override;
    procedure InternalLast; override;
    procedure InternalOpen; override;
    procedure InternalPost; override;
    procedure InternalSetToRecord(Buffer: TRecordBuffer); override;
    function  IsCursorOpen: Boolean; override;
    procedure SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag); override;
    procedure SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure InternalInitRecord(Buffer: PChar); override;
    procedure InternalLast; override;
    procedure InternalOpen; override;
    procedure InternalPost; override;
    procedure InternalSetToRecord(Buffer: PChar); override;
    function  IsCursorOpen: Boolean; override;
    procedure SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag); override;
    procedure SetBookmarkData(Buffer: PChar; Data: Pointer); override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure SetFieldData(Field: TField; Buffer: Pointer); override;
    procedure DataConvert(aField: TField; aSource, aDest: Pointer; aToNative: Boolean); override;
    function  CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream; override;
    // Optional.
    function GetRecordCount: Integer; override;
    procedure SetRecNo(Value: Integer); override;
    function GetRecNo: Integer; override;
    // Exposed properties/procedures
    Function GetParam(Const ParamName : String) : String;
    Procedure SetParam(Const ParamName,ParamValue : String);
    property Doc : PPX_Doc Read FDoc;
    
  public
    constructor Create(AOwner:tComponent); override;
    destructor Destroy; override;
  published
<<<<<<< HEAD
<<<<<<< HEAD
    Property PXLibrary : String Read FPXLibrary Write FPXLibrary Stored GetLibStored;
=======
    Property PXLibrary : String Read FPXLibrary Write FPXLibrary;
>>>>>>> graemeg/fixes_2_2
=======
    Property PXLibrary : String Read FPXLibrary Write FPXLibrary;
>>>>>>> origin/fixes_2_2
    Property FileName : String Read FFileName Write SetFileName;
    Property BlobFileName : String Read FBlobFileName Write SetBlobFileName;
    Property TableName : String Read GetTableName Write SetTableName;
    Property TargetEncoding : String Read GetTargetEncoding Write SetTargetEncoding;
    Property InputEncoding : String Read GetInputEncoding Write SetInputEncoding;
<<<<<<< HEAD
<<<<<<< HEAD
    property filter;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
  
  // in front of graphic data
  TGraphicHeader = packed record
    Count: Word;                { Always 1 }
    HType: Word;                { Always $0100 }
    Size: Longint;              { Size of actual data }
  end;


Function PXFieldTypeToFieldType(PXFieldType : Integer) : TFieldType;

Const
  SParamInputencoding  = 'inputencoding';
  SParamTargetencoding = 'targetencoding';
  SParamTableName      = 'tablename';

implementation

uses ctypes;

ResourceString
  SErrFieldTypeNotSupported = 'Fieldtype of Field "%s" not supported: %d.';
  SErrBookMarkNotFound      = 'Bookmark %d not found.';
  SErrNoFileName            = 'Filename must not be empty.';
  SErrNoBlobFile            = 'Blob file "%s" does not exist';
  SErrInvalidBlobFile       = 'Blob file "%s" is invalid';
  SErrFailedToOpenFile      = 'Failed to open file "%s" as a paradox file.';
  SErrParadoxNotOpen        = 'Paradox file not opened';
  SErrGetParamFailed        = 'Get of parameter %s failed.';
  SErrSetParamFailed        = 'Set of parameter %s failed.';
  
Const
  PXFieldTypes : Array[1..pxfNumTypes] of TFieldType
             = (ftString, ftDate, ftSmallInt, ftInteger,
                ftCurrency, ftFloat,  ftUnknown { $07},ftunknown { $08},
                ftBoolean,ftUnknown { $0A},  ftunknown { $0B}, ftMemo,
                ftBlob, ftFmtMemo, ftParadoxOle, ftGraphic,
                ftUnknown { $11}, ftUnknown { $12}, ftUnknown { $13}, ftTime,
                ftDateTime, ftAutoinc, ftBCD, ftBytes);
  {
    Buffer layout :
    Bookmark      : Record number
    BookmarkFlag  : Flag
    Data          : Actual data
  }
Type
  PPXRecInfo = ^TPXRecInfo;
  TPXRecInfo = packed record
    Bookmark: Longint;
    BookmarkFlag: TBookmarkFlag;
  end;
  PDateTime = ^TDateTime;
  
Const
  DataOffSet = SizeOf(TPXRecInfo);

{ ---------------------------------------------------------------------
  Utility functions
  ---------------------------------------------------------------------}
             
Function PXFieldTypeToFieldType(PXFieldType : Integer) : TFieldType;

begin
  if (PXFieldType<1) or (PXFieldType>pxfNumTypes) then
    Result:=ftUnknown
  else
    Result:=PXFieldTypes[PXFieldType];
end;

Var
  PXLibRefcount : Integer = 0;

Procedure UninitPXLib;

begin
  If (PXLibRefCount>0) then
    begin
    Dec(PXLibRefCount);
    If (PXLibRefCount=0) then
      begin
      PX_ShutDown();
      FreePXLib;
      end;
    end;
end;

Procedure InitPXLib(LibName : String);

begin
  If (PXLibRefCount=0) then
    begin
    LoadPXLib(LibName);
    PX_Boot();
    end;
  Inc(PXLibRefCount);
end;

{ ---------------------------------------------------------------------
    TParadox
  ---------------------------------------------------------------------}


constructor TParadox.Create(AOwner:tComponent);

begin
  inherited create(aOwner);
  FPXLibrary:=pxlibraryname;
end;

Destructor TParadox.Destroy;
begin
  Close;
  UnInitPXLib;
  inherited Destroy;
end;


Procedure TParadox.RaiseError(Fmt : String; Args : Array of const);

begin
  Raise EParadox.CreateFmt(Fmt,Args);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TParadox.GetLibStored : boolean;

begin
  Result:=(FPXLibrary<>pxlibraryname);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TParadox.SetBlobFileName(const AValue: String);
begin
  if (FBlobFileName=AValue) then
    exit;
  CheckInactive;
  FBlobFileName:=AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TParadox.PXFilterRecord(Buffer: TRecordBuffer): Boolean;
=======
function TParadox.PXFilterRecord(Buffer: PChar): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TParadox.PXFilterRecord(Buffer: PChar): Boolean;
>>>>>>> origin/fixes_2_2

var
  SaveState: TDatasetState;

begin
  Result:=True;
<<<<<<< HEAD
<<<<<<< HEAD
  if not Assigned(OnFilterRecord) and Not Filtered then
=======
  if not Assigned(OnFilterRecord) then
>>>>>>> graemeg/fixes_2_2
=======
  if not Assigned(OnFilterRecord) then
>>>>>>> origin/fixes_2_2
    Exit;
  SaveState:=SetTempState(dsFilter);
  Try
    FFilterBuffer:=Buffer;
<<<<<<< HEAD
<<<<<<< HEAD
    If Assigned(OnFilterRecord) then
      OnFilterRecord(Self,Result);
    If Result and Filtered and (Filter<>'') then
      Result:=Boolean((FParser.ExtractFromBuffer(FFilterBuffer))^);
=======
    OnFilterRecord(Self,Result);
>>>>>>> graemeg/fixes_2_2
=======
    OnFilterRecord(Self,Result);
>>>>>>> origin/fixes_2_2
  Finally
    RestoreState(SaveState);
  end;
end;

{

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.MDSReadRecord(Buffer:TRecordBuffer;ARecNo:Integer);   //Reads a Rec from Stream in Buffer
=======
procedure TParadox.MDSReadRecord(Buffer:PChar;ARecNo:Integer);   //Reads a Rec from Stream in Buffer
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.MDSReadRecord(Buffer:PChar;ARecNo:Integer);   //Reads a Rec from Stream in Buffer
>>>>>>> origin/fixes_2_2
begin
  FStream.Position:=MDSGetRecordOffset(ARecNo);
  FStream.ReadBuffer(Buffer^, FRecSize);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.MDSWriteRecord(Buffer:TRecordBuffer;ARecNo:Integer);  //Writes a Rec from Buffer to Stream
=======
procedure TParadox.MDSWriteRecord(Buffer:PChar;ARecNo:Integer);  //Writes a Rec from Buffer to Stream
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.MDSWriteRecord(Buffer:PChar;ARecNo:Integer);  //Writes a Rec from Buffer to Stream
>>>>>>> origin/fixes_2_2
begin
  FStream.Position:=MDSGetRecordOffset(ARecNo);
  FStream.WriteBuffer(Buffer^, FRecSize);
  FFileModified:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.MDSAppendRecord(Buffer:TRecordBuffer);   //Appends a Rec (from Buffer) to Stream
=======
procedure TParadox.MDSAppendRecord(Buffer:PChar);   //Appends a Rec (from Buffer) to Stream
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.MDSAppendRecord(Buffer:PChar);   //Appends a Rec (from Buffer) to Stream
>>>>>>> origin/fixes_2_2
begin
  FStream.Position:=MDSGetRecordOffset(FRecCount);
  FStream.WriteBuffer(Buffer^, FRecSize);
  FFileModified:=True;
end;
}

<<<<<<< HEAD
<<<<<<< HEAD
function TParadox.PXGetActiveBuffer(var Buffer: TRecordBuffer): Boolean;
=======
function TParadox.PXGetActiveBuffer(var Buffer: PChar): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TParadox.PXGetActiveBuffer(var Buffer: PChar): Boolean;
>>>>>>> origin/fixes_2_2

begin
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
end;

procedure TParadox.SetFileName(const AValue: String);
begin
  CheckInactive;
  FFileName:=AValue;
end;

procedure TParadox.SetInputEncoding(const AValue: String);
begin
  If Assigned(FDoc) then
    SetParam(SParamInputencoding,AVAlue);
  FInputEncoding:=AValue;
end;

procedure TParadox.SetTableName(const AValue: String);
begin
  If Assigned(FDoc) then
    SetParam(SParamTableName,AVAlue);
  FTableName:=AValue;
end;

procedure TParadox.SetTargetEncoding(const AValue: String);
begin
  If Assigned(FDoc) then
    SetParam(SParamTargetEncoding,AVAlue);
  FTargetEncoding:=AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.SetFilterText(const Value: String);
begin
  if (Value<>Filter) then
    begin
    ParseFilter(Value);
    inherited;
    if IsCursorOpen and Filtered then
      Refresh;
    end;
end;

procedure TParadox.SetFiltered(Value: Boolean);
begin
  if (Value<>Filtered) then
    begin
    inherited;
    if IsCursorOpen then
      Refresh;
    end;
end;


//Abstract Overrides
function TParadox.AllocRecordBuffer: TRecordBuffer;
=======
//Abstract Overrides
function TParadox.AllocRecordBuffer: PChar;
>>>>>>> graemeg/fixes_2_2
=======
//Abstract Overrides
function TParadox.AllocRecordBuffer: PChar;
>>>>>>> origin/fixes_2_2
begin
  Result:=Nil;
  GetMem(Result,SizeOf(TPXRecInfo)+GetRecordSize);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.FreeRecordBuffer (var Buffer: TRecordBuffer);
=======
procedure TParadox.FreeRecordBuffer (var Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.FreeRecordBuffer (var Buffer: PChar);
>>>>>>> origin/fixes_2_2
begin
  FreeMem(Buffer);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.InternalInitRecord(Buffer: TRecordBuffer);
=======
procedure TParadox.InternalInitRecord(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.InternalInitRecord(Buffer: PChar);
>>>>>>> origin/fixes_2_2

begin
  fillchar((Buffer+DataOffSet)^,GetRecordSize,0);
end;

procedure TParadox.InternalDelete;

begin
  If (FCurrRecNo<>-1) then
    PX_delete_record(FDoc,FCurrRecNo);
end;

procedure TParadox.InternalInitFieldDefs;

Var
  I, CurrOffSet, ACount : Integer;
  FN : String;
  FS : Integer;
  B : Boolean;
  FT : TFieldType;
  pxf : Ppxfield_t;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  FieldDefs.Clear;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  pxf:=PX_get_fields(FDoc);
  ACount:= PX_get_num_fields(FDoc);
  ReallocMem(FOffsets,ACount*SizeOf(Integer));
  FillChar(FOffSets^,ACount*SizeOf(Integer),0);
  CurrOffSet:=DataOffset;
  For I:=0 to ACount-1 do
    begin
    FOffsets[I]:=CurrOffset;
    FN:=strpas(pxf^.px_fname);
    FT:=PXFieldTypeToFieldType(pxf^.px_ftype);
    If (FT=ftUnKnown) then
      RaiseError(SErrFieldTypeNotSupported,[FN,pxf^.px_ftype]);
    If (FT in [ftString,ftBlob,ftMemo,ftFmtMemo,ftGraphic,ftParadoxOle,ftBytes]) then
      FS:=pxf^.px_flen
    else if (Ft=ftBCD) then
      FS:=pxf^.px_fdc
    else
      FS:=0;
    B:=False; // No way to detect required paradox fields ?
    FieldDefs.Add(FN,ft,FS,B);
    Inc(CurrOffset,pxf^.px_flen);
    Inc(pxf);
    end;
end;

procedure TParadox.InternalFirst;
begin
  FCurrRecNo:=-1;
end;

procedure TParadox.InternalLast;
begin
  FCurrRecNo:=PX_Get_num_records(FDoc);
end;

procedure TParadox.SetOpenParams;

begin
  If (FTargetEncoding<>'') then
    SetParam(SParamTargetEncoding,FTargetEncoding);
  If (FInputEncoding<>'') then
    SetParam(SParamInputEncoding,FInputEncoding);
end;

procedure TParadox.OpenBlobFile;

Var
 BFN : string;
begin
  BFN:=FBlobFileName;
  If (BFN<>'') then
    if not FileExists(BFN) then
      RaiseError(SErrNoBlobFile,[BFN]);
  If (BFN='') then
    begin
    BFN:=ChangeFileExt(FFileName,'.mb');
    If Not FileExists(BFN) then
      begin
      BFN:=ChangeFileExt(FFileName,'.MB');
      If Not FileExists(BFN) then
        BFN:='';
      end;
    end;
  If (BFN<>'') then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    //Writeln('opening blib file',bfn);
=======
    Writeln('opening blib file',bfn);
>>>>>>> graemeg/fixes_2_2
=======
    Writeln('opening blib file',bfn);
>>>>>>> origin/fixes_2_2
    if PX_set_blob_file(FDoc,PChar(BFN))<>0 then
      RaiseError(SErrInvalidBlobFile,[BFN]);
    FBlobFileName:=BFN;
    end;
end;

procedure TParadox.InternalOpen;

Var
  FN : String;

begin
  InitPXLib(FPXLibrary);
  If (FFileName='') then
    RaiseError(SErrNoFileName,[]);
  FN:=FFileName;
  FDoc:=PX_New();
  try
    If (px_open_file(FDoc,PChar(FN))<>0) then
      RaiseError(SErrFailedToOpenFile,[FN]);
    SetOpenParams;
    OpenBlobFile;
    InternalInitFieldDefs;
    if DefaultFields then
      CreateFields;
    BindFields(True);
    FCurrRecNo:=-1;
  except
    If Assigned(FDoc) then
      begin
      PX_Delete(FDoc);
      FDoc:=Nil;
      end;
    Raise;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  try
    ParseFilter(Filter);
  except
    On E : Exception do
      Filter:='';
  end;
end;

procedure TParadox.ParseFilter(const AFilter: string);
begin
  // parser created?
  if Length(AFilter) > 0 then
  begin
    if (FParser = nil) and IsCursorOpen then
    begin
      FParser := TBufDatasetParser.Create(Self);
    end;
    // have a parser now?
    if FParser <> nil then
    begin
      // set options
      FParser.PartialMatch := not (foNoPartialCompare in FilterOptions);
      FParser.CaseInsensitive := foCaseInsensitive in FilterOptions;
      // parse expression
      FParser.ParseExpression(AFilter);
    end;
  end;
end;
procedure TParadox.InternalClose;

begin
  BindFields(False);
  if DefaultFields then
    DestroyFields;
  FreeAndNil(FParser);
=======
=======
>>>>>>> origin/fixes_2_2
end;

procedure TParadox.InternalClose;

begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FreeMem(FOffsets);
  FOffSets:=Nil;
  FCurrRecNo:=-1;
  If Assigned(FDoc) then
    begin
    PX_close(FDoc);
    PX_Delete(FDOc);
    end;
  FDoc:=Nil;
end;

procedure TParadox.InternalPost;
begin
  CheckActive;
  if ((State<>dsEdit) and (State<>dsInsert)) then
    Exit;
  if (State=dsEdit) then
<<<<<<< HEAD
<<<<<<< HEAD
    PX_put_recordn(FDoc,pansichar(ActiveBuffer), FCurrRecNo)
=======
    PX_put_recordn(FDoc,ActiveBuffer, FCurrRecNo)
>>>>>>> graemeg/fixes_2_2
=======
    PX_put_recordn(FDoc,ActiveBuffer, FCurrRecNo)
>>>>>>> origin/fixes_2_2
  else
    InternalAddRecord(ActiveBuffer,True);
end;

function TParadox.IsCursorOpen: Boolean;

begin
  Result:=(FDoc<>Nil);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TParadox.GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
=======
function TParadox.GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
>>>>>>> graemeg/fixes_2_2
=======
function TParadox.GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
>>>>>>> origin/fixes_2_2

var
  Accepted: Boolean;

begin
  Result:=grOk;
  Accepted:=False;
  if (GetRecordCount<1) then
    begin
    Result:=grEOF;
    exit;
    end;
  repeat
    case GetMode of
      gmCurrent:
        if (FCurrRecNo>=GetRecordCount) or (FCurrRecNo<0) then
          Result:=grError;
      gmNext:
        if (FCurrRecNo<GetRecordCount-1) then
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
<<<<<<< HEAD
<<<<<<< HEAD
      PX_get_record(Doc,FCurrRecNo,pansichar(Buffer+DataOffset));
=======
      PX_get_record(Doc,FCurrRecNo,Buffer+DataOffset);
>>>>>>> graemeg/fixes_2_2
=======
      PX_get_record(Doc,FCurrRecNo,Buffer+DataOffset);
>>>>>>> origin/fixes_2_2
      PPXRecInfo(Buffer)^.Bookmark:=FCurrRecNo;
      PPXRecInfo(Buffer)^.BookmarkFlag:=bfCurrent;
      if (Filtered) then
        Accepted:=PXFilterRecord(Buffer) //Filtering
      else
        Accepted:=True;
      if (GetMode=gmCurrent) and not Accepted then
        result:=grError;
      end;
  until (result<>grOK) or Accepted;
end;

function TParadox.GetFieldData(Field: TField; Buffer: Pointer): Boolean;

var
<<<<<<< HEAD
<<<<<<< HEAD
  Buf          : TRecordbuffer;
  No,pft,flen : integer;
  pxf          : PPx_field;
  Value        : Pchar;
  D            : clong;
=======
=======
>>>>>>> origin/fixes_2_2
  Buf          : PChar;
  No,pft,flen : integer;
  pxf          : PPx_field;
  Value        : Pchar;
  Y,M,D        : cint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  longv        : Clong;
  R            : Double;
  c            : Char;

begin
  No:=Field.FieldNo-1;
  Buf:=Nil;
  result:=(No>=0) and PXGetActiveBuffer(Buf);
  if result and (buffer <> nil) then
    begin
    pxf:=PX_get_field(FDoc,No);
    Flen:=pxf^.px_flen;       // Field length
    pft:=pxf^.px_ftype;    // Field type
    Assert(PXFieldTypes[pft]=Field.DataType,'Field types do not match');
    Inc(Buf,FOffsets[No]); // Move to actual field offset
    Case pft of
      pxfAlpha:
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        Result:=PX_get_data_alpha(FDoc,pansichar(Buf),flen,@value)>0;
        If result then
          begin
          Move(Value^,Buffer^,flen);
          If (Flen<=Field.DataSize) then
            Pchar(Buffer)[flen]:=#0;
          FDoc^.free(FDoc,value);
=======
=======
>>>>>>> origin/fixes_2_2
        Result:=PX_get_data_alpha(FDoc,Buf,flen,@value)>0;
        If result then
          begin
          Move(Value^,Buffer^,flen);
          doc^.free(doc,value);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;
        end;
      pxfDate:
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        Result:=PX_get_data_long(FDoc,pansichar(Buf),flen,@longv)>0;
        If Result then
          begin
          // 1721425 is the number of the days between the start of the
          // julian calendar (4714 BC) and jan-00-0000 (Paradox base date)
          // 2415019 is the number of the days between the start of the
          // julian calendar (4714 BC) and dec-30-1899 (TDateTime base date)
          PDateTime(Buffer)^:=Longv+1721425-2415019;
=======
=======
>>>>>>> origin/fixes_2_2
        Result:=PX_get_data_long(FDoc,Buf,flen,@longv)>0;
        If Result then
          begin
          PX_SdnToGregorian(longv+1721425,@Y,@M,@D);
          PDateTime(Buffer)^:=EncodeDate(Y,M,D);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;
        end;
      pxfShort:
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        Result:=PX_get_data_short(FDoc,pansichar(Buf), flen, @D)>0;
=======
        Result:=PX_get_data_short(FDoc,Buf, flen, @D)>0;
>>>>>>> graemeg/fixes_2_2
=======
        Result:=PX_get_data_short(FDoc,Buf, flen, @D)>0;
>>>>>>> origin/fixes_2_2
        If result then
          PSmallInt(Buffer)^:=D;
        end;
      pxfAutoInc,
      pxfLong:
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        Result:=(PX_get_data_long(FDoc,pansichar(buf),flen,@longv)>0);
=======
        Result:=(PX_get_data_long(FDoc,buf,flen,@longv)>0);
>>>>>>> graemeg/fixes_2_2
=======
        Result:=(PX_get_data_long(FDoc,buf,flen,@longv)>0);
>>>>>>> origin/fixes_2_2
        If Result then
          PInteger(Buffer)^:=Longv;
        end;
      pxfCurrency,
      pxfNumber:
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        Result:=(PX_get_data_double(FDoc,pansichar(Buf),Flen,@R)>0);
=======
        Result:=(PX_get_data_double(FDoc,Buf,Flen,@R)>0);
>>>>>>> graemeg/fixes_2_2
=======
        Result:=(PX_get_data_double(FDoc,Buf,Flen,@R)>0);
>>>>>>> origin/fixes_2_2
        If Result then
          PDouble(Buffer)^:=R;
        end;
      pxfLogical:
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        Result:=(PX_get_data_byte(FDoc,pansichar(Buf),flen,@C)>0);
        If result then
          PWordBool(Buffer)^:=(C<>#0);
        end;
      pxfBytes:
        begin
        Result:=PX_get_data_bytes(FDoc,pansichar(Buf),FLen,@Value)>0;
=======
=======
>>>>>>> origin/fixes_2_2
        Result:=(PX_get_data_byte(FDoc,Buf,flen,@C)>0);
        If result then
          PBoolean(Buffer)^:=(C<>#0);
        end;
      pxfBytes:
        begin
        Result:=PX_get_data_bytes(FDoc,Buf,FLen,@Value)>0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        If Result then
          begin
          Move(Value^,Buffer^,FLen);
          FDoc^.free(FDoc,value);
          end;
        end;
      pxfMemoBLOb,
      pxfBLOb,
      pxfFmtMemoBLOb,
      pxfOLE,
      pxfGraphic:
        begin
        Result:=True;
        Move(Buf^,Buffer^,FLen);
        end;
      pxfTime:
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        Result:=(PX_get_data_long(FDoc,pansichar(Buf),flen,@longv)>0);
=======
        Result:=(PX_get_data_long(FDoc,Buf,flen,@longv)>0);
>>>>>>> graemeg/fixes_2_2
=======
        Result:=(PX_get_data_long(FDoc,Buf,flen,@longv)>0);
>>>>>>> origin/fixes_2_2
        If result then
          PDateTime(Buffer)^:=longv/MSecsPerDay;
        end;
      pxfTimestamp:
        begin
<<<<<<< HEAD
<<<<<<< HEAD
        Result:=(PX_get_data_double(FDoc,pansichar(buf),flen,@R)>0);
        if Result then
          begin
<<<<<<< HEAD
          longv:=trunc(R /86400000);
          D:=Longv+1721425-2415019;
          longv:=(Trunc(r) mod 86400000);
=======
          R:=R/1000.0;
          longv:=trunc(R /86400);
          D:=Longv+1721425-2415019;
          longv:=(Trunc(r) mod 86400);
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
          PDateTime(Buffer)^:=D+(Longv/MSecsPerday);
=======
=======
>>>>>>> origin/fixes_2_2
        Result:=(PX_get_data_double(FDoc,buf,flen,@R)>0);
        if Result then
          begin
          R:=R/1000.0;
          longv:=trunc(R /86400);
          PX_SdnToGregorian(longv+1721425,@Y,@M,@D);
          longv:=(Trunc(r) mod 86400);
          PDateTime(Buffer)^:=EncodeDate(Y,M,d)+(Longv/MSecsPerday);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;
        end;
      pxfBCD:
        begin
        Result:=(PX_get_data_bcd(FDoc,pcuchar(Buf),pxf^.px_fdc,@Value)>0);
        if Result then
          begin
          PCurrency(Buffer)^:=StrToCurr(StrPas(value));
          FDoc^.free(FDoc,value);
          end;
        end;
    else
      RaiseError('Unknown type (%d) (%d)',[pxf^.px_ftype, pxf^.px_flen]);
    end;
    end;
end;

procedure TParadox.SetFieldData(Field: TField; Buffer: Pointer);

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
 I: integer;

begin
 DestBuffer:=Nil;
 I:=Field.FieldNo-1;
 if (I >= 0) and  PXGetActiveBuffer(DestBuffer) then
   begin
   dataevent(deFieldChange,ptrint(field));
   end;
end;

procedure TParadox.DataConvert(aField: TField; aSource, aDest: Pointer;
  aToNative: Boolean);
begin
  If AField.DataType in [ftDate,ftTime,ftDateTime] then
    PDateTime(aDest)^:=PDateTime(aSource)^
  else
    inherited DataConvert(aField, aSource, aDest, aToNative);
end;


function TParadox.CreateBlobStream(Field: TField; Mode: TBlobStreamMode
  ): TStream;

TYpe
  PGraphicHeader = ^TGraphicHeader;
Var
  FBuf,Value,V2 : Pchar;
  FLen,Res : Integer;
  M,D : Cint;
  H : PGraphicHeader;
  
begin
  Result:=Nil;
  FLen:=Field.Size;
  If Mode=bmRead then
    begin
    FBuf:=GetMem(FLen);
    Try
      If Not Field.GetData(FBuf,True) then
        exit;
      if (Field.DataType=ftGraphic) then
        Res:=PX_get_data_graphic(FDoc,FBuf,FLen,@M,@D,@Value)
      else
        Res:=PX_get_data_blob(FDoc,FBuf,FLen,@M,@D,@Value);
      If (Res>0) and (Value<>Nil) then
        begin
        Result:=TMemoryStream.Create;
        V2:=Value;
        if (Field.DataType=ftGraphic) then
<<<<<<< HEAD
<<<<<<< HEAD
          begin
          Result.WriteAnsiString('bmp');
          Result.WriteBuffer(V2^,D-SizeOf(TGraphicHeader));
          end
        else
          Result.WriteBuffer(V2^,D);
=======
          Result.WriteAnsiString('bmp');
        Result.WriteBuffer(V2^,D-SizeOf(TGraphicHeader));
>>>>>>> graemeg/fixes_2_2
=======
          Result.WriteAnsiString('bmp');
        Result.WriteBuffer(V2^,D-SizeOf(TGraphicHeader));
>>>>>>> origin/fixes_2_2
        Result.Position:=0;
        FDoc^.free(FDoc,Value);
        end;
    Finally
      FreeMem(FBuf);
    end;
    end
  else
    Result:=TMemoryStream.Create;
end;

function TParadox.GetRecordSize: Word;

begin
 Result:=PX_Get_RecordSize(FDoc);
end;

procedure TParadox.InternalGotoBookmark(ABookmark: Pointer);

var
  ReqBookmark: integer;

begin
  ReqBookmark:=PInteger(ABookmark)^;
  if (ReqBookmark>=0) and (ReqBookmark<GetRecordCount) then
    FCurrRecNo:=ReqBookmark
  else
    RaiseError(SErrBookMarkNotFound,[ReqBookmark]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.InternalSetToRecord(Buffer: TRecordBuffer);
=======
procedure TParadox.InternalSetToRecord(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.InternalSetToRecord(Buffer: PChar);
>>>>>>> origin/fixes_2_2

var
  ReqBookmark: integer;

begin
  ReqBookmark:=PPXRecInfo(Buffer)^.Bookmark;
  InternalGotoBookmark (@ReqBookmark);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TParadox.GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag;
=======
function TParadox.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
>>>>>>> graemeg/fixes_2_2
=======
function TParadox.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
>>>>>>> origin/fixes_2_2

begin
  Result:=PPXRecInfo(Buffer)^.BookmarkFlag;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag);
=======
procedure TParadox.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
>>>>>>> origin/fixes_2_2

begin
  PPXRecInfo(Buffer)^.BookmarkFlag := Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
=======
procedure TParadox.GetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.GetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> origin/fixes_2_2

begin
  if Data<>nil then
    PInteger(Data)^:=PPXRecInfo(Buffer)^.Bookmark;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParadox.SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
=======
procedure TParadox.SetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TParadox.SetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> origin/fixes_2_2

begin
  if Data<>nil then
    PPXRecInfo(Buffer)^.Bookmark:=PInteger(Data)^
  else
    PPXRecInfo(Buffer)^.Bookmark:=0;
end;

procedure TParadox.InternalAddRecord(Buffer: Pointer; DoAppend: Boolean);

begin
  PXAppendRecord(ActiveBuffer);
  InternalLast;
end;

procedure TParadox.PXAppendRecord(Buffer : Pointer);

begin
end;

function TParadox.GetInputEncoding: String;
begin
  If Assigned(FDoc) then
    Result:=GetParam('inputencoding')
  else
    Result:=FInputEncoding;
end;

function TParadox.GetTableName: String;
begin
  If Assigned(FDoc) then
    Result:=GetParam('tablename')
  else
    Result:=FInputEncoding;
end;

function TParadox.GetTargetEncoding: String;
begin
  If Assigned(FDoc) then
    Result:=GetParam('targetencoding')
  else
    Result:=FTargetEncoding;
end;

procedure TParadox.SetRecNo(Value: Integer);
begin
  CheckBrowseMode;
  if (Value>=1) and (Value<=GetRecordCount) then
    begin
    FCurrRecNo:=Value-1;
    Resync([]);
    end;
end;

Function TParadox.GetRecNo: Longint;

begin
  UpdateCursorPos;
  if (FCurrRecNo<0) then
    Result:=1
  else
    Result:=FCurrRecNo+1;
end;

function TParadox.GetParam(const ParamName: String): String;

Var
  V : Pchar;

begin
  If Not Assigned(FDoc) then
    RaiseError(SErrParadoxNotOpen,[]);
  if (PX_Get_parameter(FDoc,Pchar(ParamName),@V)<>0) then
    RaiseError(SErrGetParamFailed,[ParamName]);
  If (V<>Nil) then
    Result:=strpas(V);
end;

procedure TParadox.SetParam(const ParamName, ParamValue: String);
begin
  If Not Assigned(FDoc) then
    RaiseError(SErrParadoxNotOpen,[]);
  if (PX_Set_parameter(FDoc,Pchar(ParamName),PChar(ParamValue))<>0) then
    RaiseError(SErrSetParamFailed,[ParamName]);
end;

Function TParadox.GetRecordCount: Longint;

begin
  If Assigned(FDoc) then
    Result:=PX_Get_num_records(FDoc)
  else
    Result:=0;
end;


end.
