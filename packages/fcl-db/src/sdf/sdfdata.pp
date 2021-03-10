unit SdfData;

{$mode objfpc}
{$h+}

//-----------------------------------------------------------------------------
{ Unit Name  : SdfData  Application : TSdfDataSet TFixedFormatDataSet Components
  Version    : 2.05
  Author     : Orlando Arrocha           email: oarrocha@hotmail.com
  Purpose    : This components are designed to access directly text files as
               database tables. The files may be limited (SDF) or fixed size
               columns.
---------------
Modifications
---------------
30/Jul/15 LacaK:
      Added TSDFStringList to support reading of CSV files, which have embedded
      CRLF between double-quotes.
7/Jun/12 BigChimp:
      Quote fields with delimiters or quotes to match Delphi SDF definition
      (see e.g. help on TStrings.CommaText)
14/Jul/11 BigChimp:
      Added AllowMultiLine property so user can use fields that have line endings
      (Carriage Return and/or Line Feed) embedded in their fields (fields need to be
      quoted). For now: output only (reading these fields does not work yet)
12/Mar/04  Lazarus version (Sergey Smirnov AKA SSY)
      Locate and CheckString functions are removed because of Variant data type.
      Many things are changed for FPC/Lazarus compatibility.
02/Jun/02  Version 2.05 (Doriano Biondelli)
      TrimSpace property added for those cases where you need to retrieve the
      field with spaces.
01/Jan/02  Version 2.04 (Orlando Arrocha)
      FieldList is now populated.
      Locate was changed to improve speed and some bug fixing too. Thanks for
         asking and testing Marcelo Castro
16/Dec/01  Version 2.03 (Orlando Arrocha)
           Fixed some bugs and added some recomentdations. Here is a list:
      Quotations on the last field was not removed properly. Special thanks to
         Daniel Nakasone for helping with the solution.
      Appending first record to empty files was failing. Thanks again Daniel
         Nakasone for the report
      GetFieldData now trims the trailing spaces of the field, so users doesn't
         needs to do it by themselves anymore. Thanks for the recomendation
         Juergen Gehrke.
      FieldDefs is now available from the designer. Recomended by Leslie Drewery.
                ****** THANKS TO ALL & KEEP SENDING RECOMENDATIONS *****
05/Oct/01  Version 2.02 (Ben Hay)
      Locate function : implement the virtual tdataset method "Locate".
                ****** THANKS BEN *****
11/Sep/01  Version 2.01 (Leslie Drewery)
           Added additional logic to handle Corrupt Data by making sure the
           Quotes are closed and the delimiter/<CR>/<LF> are the next
           characters.
           Altered buffer method to create on constructor and cleared when opened.
      New Resource File. Nice Icons
      SaveToStream method included
      LoadFromStream method included
                ****** THANKS LESLIE *****
14/Ago/01  Version 2.00 (Orlando Arrocha)
           John Dung Nguyen showed me how to make this compatible with C-Builder
           and encouraged me to include a filter.
           Dimitry V. Borko says that russian CSV files used other delimiters,
           so now you can change it.
      OnFilter and other events included.
      Delimiter property added to TSdfDataSet. No more dependency on CommaText
         methodology -- choose your own delimiter.
      BufToStore/StoreToBuf methods lets you translate data records to and from
         your propietary storage format.
      TTextDataSet removed dependencies.
      TBaseTextDataSet class removed. // TBaseTextDataSet = TFixedFormatDataSet;
                ****** THANKS JOHN ******   ***** THANKS DIMMY *****
19/Jul/01  Version 1.03 (Orlando Arrocha)
      TBaseTextDataSet class introduced.
      FileName property changed datatype to TFileName and removed the property
         editor to segregate design-time code from runtime units.
      *** To add file browsing functionality please install
      *** TFileNamePropertyEditor -- also freeware.
                                     ********** THANKS WAYNE *********
18/Jun/01  Version 1.02 (Wayne Brantley)
      Schema replaces SchemaFileName property. Same as SchemaFileName, except
         you can define the schema inside the component. If you still need an
         external file, just use Schema.LoadFromFile()
      TFixedFormatDataSet class introduced. Use this class for a Fixed length
         format file (instead of delimited). The full schema definition
         (including lengths) is obviously required.
      Bug Fixed - When FirstLineSchema is true and there were no records, it
         would display garbage.

30/Mar/01  Version 1.01 (Orlando Arrocha)
           Ligia Maria Pimentel suggested to use the first line of the file to
           define the field names.  ****** THANKS LIGIA ******
      FileMustExist property. You must put this property to FALSE if you want to
         create a new file.
      FirstLineSchema property. You can define the field names on the first line
         of your file. Fields have to be defined with this format
            <field_name1> [= field_size1] , <field_name2> [= field_size2] ...
      SchemaFileName property.  (Changed to Schema by 1.02 Wayne)
         Lets you define the fields attributes (only supports field name and
         size). Have to be defined in this format (one field per line) :
            <field_name> [= field_size]
         NOTE: fields that doesn't define the length get the record size.
      RemoveBlankRecords procedure. Removes all the blank records from the file.
      RemoveExtraColumns procedure. If the file have more columns than the
         scheme or the field definition at design time, it remove the extra
         values from the file.
      SaveFileAs. Let you save the file to another filename.
         NOTE: This component save changes on closing the table, so you can use
               this to save data before that event.
Jan 2001 Version 1.0 TSdfDataSet introduced.
---------
TERMS
---------
 This component is provided AS-IS without any warranty of any kind, either
 express or implied. This component is freeware and can be used in any software
 product. Credits on applications will be welcomed.
 If you find it useful, improve it or have a wish list... please drop me a mail,
 I'll be glad to hear your comments.
----------------
How to Install
----------------
 1. Copy this SDFDATA.PAS and the associated SDFDATA.DCR to the folder from
    where you wish to install the component. This will probably be $(DELPHI)\lib
    or a sub-folder.
 2. Install the TSdfDataSet and TFixedFormatDataSet components by choosing the
    Component | Install Component menu option.
 3. Select the "Into exisiting package" page of the Install Components dialogue.
 4. Browse to the folder where you saved this file and select it.
 5. Ensure that the "Package file name" edit box contains $(DELPHI)\DCLUSR??.DPK
    or the one you prefer for DB related objects.
 6. Accept that the package will be rebuilt.
}
//-----------------------------------------------------------------------------
interface

uses
  DB, Classes, SysUtils, DBConst;

type
//-----------------------------------------------------------------------------
// TRecInfo
  PRecInfo = ^TRecInfo;
  TRecInfo = packed record
    Bookmark: PtrInt;
    BookmarkFlag: TBookmarkFlag;
  end;
//-----------------------------------------------------------------------------

  { TSDFStringList }

  TSDFStringList = class(TStringList)
    protected
      FMultiLine: boolean;
      procedure SetTextStr(const Value: string); override;
  end;

//-----------------------------------------------------------------------------
// TFixedFormatDataSet
//-----------------------------------------------------------------------------
  TFixedFormatDataSet = class(TDataSet)
  private
    FCodePage: String;
    FSchema             :TStringList;
    FFileName           :TFileName;
    FFilterBuffer       :TRecordBuffer;
    FFileMustExist      :Boolean;
    FReadOnly           :Boolean;
    FLoadFromStream     :Boolean;
    FTrimSpace          :Boolean;
    FEncoding : TEncoding;
    procedure SetCodePage(AValue: String);
    procedure SetSchema(const Value: TStringList);
    procedure SetFileName(Value : TFileName);
    procedure SetFileMustExist(Value : Boolean);
    procedure SetTrimSpace(Value : Boolean);
    procedure SetReadOnly(Value : Boolean);
    procedure RemoveWhiteLines(List : TStrings; IsFileRecord : Boolean);
    procedure LoadFieldScheme(List : TStrings; MaxSize : Integer);
    function GetActiveRecBuf(out RecBuf: TRecordBuffer): Boolean;
    procedure SetFieldOfs(var Buffer : TRecordBuffer; FieldNo : Integer);
  protected
    FData               :TSDFStringList;
    FDataOffset         :Integer;
    FCurRec             :Integer;
    FRecordSize         :Integer;
    FRecBufSize         :Integer;
    FRecInfoOfs         :Integer;
    FLastBookmark       :PtrInt;
    FSaveChanges        :Boolean;
    FDefaultRecordLength:Cardinal;
  protected
    function AllocRecordBuffer: TRecordBuffer; override;
    procedure FreeRecordBuffer(var Buffer: TRecordBuffer); override;
    procedure InternalAddRecord(Buffer: Pointer; DoAppend: Boolean); override;
    procedure InternalClose; override;
    procedure InternalDelete; override;
    procedure InternalFirst; override;
    procedure InternalGotoBookmark(ABookmark: Pointer); override;
    procedure InternalHandleException; override;
    procedure InternalInitFieldDefs; override;
    procedure InternalInitRecord(Buffer: TRecordBuffer); override;
    procedure InternalLast; override;
    procedure InternalOpen; override;
    procedure InternalPost; override;
    procedure InternalEdit; override;
    procedure InternalSetToRecord(Buffer: TRecordBuffer); override;
    function IsCursorOpen: Boolean; override;
    procedure GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    function GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag; override;
    function GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult; override;
    function GetRecordSize: Word; override;
    procedure SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag); override;
    procedure SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    procedure SetFieldData(Field: TField; Buffer: Pointer); override;
    procedure ClearCalcFields(Buffer: TRecordBuffer); override;
    function GetRecordCount: Longint; override;
    function GetRecNo: Longint; override;
    procedure SetRecNo(Value: Integer); override;
    function GetCanModify: boolean; override;
    function RecordFilter(RecBuf: TRecordBuffer): Boolean;
    function BufToStore(Buffer: TRecordBuffer): String; virtual;
    function StoreToBuf(Source: String): String; virtual;
  public
    property DefaultRecordLength: Cardinal read FDefaultRecordLength
      write FDefaultRecordLength default 250;
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    function  BookmarkValid(ABookmark: TBookmark): Boolean; override;
    function CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint; override;
    function  GetFieldData(Field: TField; Buffer: Pointer): Boolean; override;
    procedure RemoveBlankRecords; dynamic;
    procedure RemoveExtraColumns; dynamic;
    procedure SaveFileAs(strFileName : String); dynamic;
    property  CanModify;
    procedure LoadFromStream(Stream :TStream);
    procedure SaveToStream(Stream :TStream);
  published
    property FileMustExist: Boolean read FFileMustExist write SetFileMustExist;
    property ReadOnly: Boolean read FReadOnly write SetReadOnly;
    property FileName : TFileName read FFileName write SetFileName;
    property Schema: TStringList read FSchema write SetSchema;
    property TrimSpace: Boolean read FTrimSpace write SetTrimSpace default True;
    Property CodePage : String Read FCodePage Write SetCodePage;
    property FieldDefs;
    property Active;
    property AutoCalcFields;
    property Filtered;
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
//    property BeforeRefresh;
//    property AfterRefresh;
    property OnCalcFields;
    property OnDeleteError;
    property OnEditError;
    property OnFilterRecord;
    property OnNewRecord;
    property OnPostError;
  end;

//-----------------------------------------------------------------------------
// TSdfDataSet
//-----------------------------------------------------------------------------
  TSdfDataSet = class(TFixedFormatDataSet)
  private
    FDelimiter : Char;
    FFirstLineAsSchema : Boolean;
    FMultiLine         : Boolean;
    FStripTrailingDelimiters : Boolean;
    procedure DoStripTrailingDelimiters(var S: String);
    procedure SetMultiLine(const Value: Boolean);
    procedure SetFirstLineAsSchema(Value : Boolean);
    procedure SetDelimiter(Value : Char);
  protected
    procedure InternalInitFieldDefs; override;
    function BufToStore(Buffer: TRecordBuffer): String; override;
    function StoreToBuf(Source: String): String; override;
    function ExtractDelimited(const S: String; var Pos: integer): string; virtual;
  public
    constructor Create(AOwner: TComponent); override;
  published
    // Whether or not to allow fields containing CR and/or LF (on write only)
    property AllowMultiLine: Boolean read FMultiLine write SetMultiLine;
    property Delimiter: Char read FDelimiter write SetDelimiter;
    property FirstLineAsSchema: Boolean read FFirstLineAsSchema write SetFirstLineAsSchema;
    // Set this to True if you want to strip all last delimiters
    Property StripTrailingDelimiters : Boolean Read FStripTrailingDelimiters Write FStripTrailingDelimiters;
  end;

procedure Register;

implementation

//{$R *.Res}

Resourcestring
  SErrUnknownCodePage = 'Unknown code page: %s';


//-----------------------------------------------------------------------------
// TFixedFormatDataSet
//-----------------------------------------------------------------------------
constructor TFixedFormatDataSet.Create(AOwner : TComponent);
begin
  FDefaultRecordLength := 250;
  FFileMustExist  := TRUE;
  FLoadFromStream := False;
  FRecordSize   := 0;
  FTrimSpace    := TRUE;
  FSchema       := TStringList.Create;
  FData         := TSDFStringList.Create;  // Load the textfile into a StringList
  inherited Create(AOwner);
end;

destructor TFixedFormatDataSet.Destroy;
begin
  inherited Destroy;
  FreeAndNil(FEncoding);
  FreeAndNil(FData);
  FreeAndNil(FSchema);
end;

procedure TFixedFormatDataSet.SetSchema(const Value: TStringList);
begin
  CheckInactive;
  FSchema.Assign(Value);
end;

procedure TFixedFormatDataSet.SetCodePage(AValue: String);

Var
  F : TSystemCodePage;

begin
  if FCodePage=AValue then Exit;
  CheckInactive;
  F:=CodePageNameToCodePage(aValue);
  if (F=$FFFF) then
    DatabaseErrorFmt(SErrUnknownCodePage,[aValue]);
  FCodePage:=AValue;
  FreeAndNil(FEncoding);
  FEncoding:=TMBCSEncoding.Create(F);
end;

procedure TFixedFormatDataSet.SetFileMustExist(Value : Boolean);
begin
  CheckInactive;
  FFileMustExist := Value;
end;

procedure TFixedFormatDataSet.SetTrimSpace(Value : Boolean);
begin
  CheckInactive;
  FTrimSpace := Value;
end;

procedure TFixedFormatDataSet.SetReadOnly(Value : Boolean);
begin
  CheckInactive;
  FReadOnly := Value;
end;

procedure TFixedFormatDataSet.SetFileName(Value : TFileName);
begin
  CheckInactive;
  FFileName := Value;
end;

function FieldDefDataSize(fd:TFieldDef):integer;
begin
  result:=fd.Size*fd.CharSize+1;
end;

procedure TFixedFormatDataSet.InternalInitFieldDefs;
var
  i, Len, DataLen, MaxLen :Integer;
  LstFields      :TStrings;
  FEnc : TSystemCodePage;

begin
  if not Assigned(FData) then Exit;

  MaxLen := 0;
  FieldDefs.Clear;
  for i := FData.Count - 1 downto 0 do  // Find out the longest record
  begin
    Len := Length(FData[i]);
    if Len > MaxLen then
      MaxLen := Len;
    FData.Objects[i] := TObject(Pointer(i+1));   // Fabricate Bookmarks
  end;
  if (MaxLen = 0) then
    MaxLen := FDefaultRecordLength;

  FRecordSize := 0;
  LstFields := TStringList.Create;
  try
    LoadFieldScheme(LstFields, MaxLen);
    for i := 0 to LstFields.Count -1 do  // Add fields
    begin
      Len := StrToIntDef(LstFields.Values[LstFields.Names[i]], MaxLen);
      if Assigned(FEncoding) then
        Fenc:=FEncoding.CodePage
      else
        FEnc:=DefaultSystemCodePage;
      DataLen:=FieldDefDataSize(FieldDefs.Add(Trim(LstFields.Names[i]), ftString, Len, 0, False,False,FieldDefs.Count+1,FEnc));
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
      DataLen := Align(DataLen, SizeOf(PtrInt));
{$ENDIF}
      Inc(FRecordSize, DataLen);
    end;
  finally
    LstFields.Free;
  end;
end;

procedure TFixedFormatDataSet.InternalOpen;
var
  Stream : TStream;
begin
  if not Assigned(FData) then Exit;

  FSaveChanges := FALSE;
  if (not FileMustExist) and (not FileExists(FileName)) then
  begin
    Stream := TFileStream.Create(FileName, fmCreate);
    Stream.Free;
  end;
  if not FLoadFromStream then
    FData.LoadFromFile(FileName);
  FRecordSize := FDefaultRecordLength;
  InternalInitFieldDefs;
  if FRecordSize = 0 then
    FRecordSize := FDefaultRecordLength;
  if DefaultFields then
    CreateFields;
  BindFields(TRUE);
  BookmarkSize := SizeOf(PtrInt);
  FRecInfoOfs := FRecordSize + CalcFieldsSize; // Initialize the offset for TRecInfo in the buffer
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
  FRecInfoOfs := Align(FRecInfoOfs, SizeOf(PtrInt));
{$ENDIF}
  FRecBufSize := FRecInfoOfs + SizeOf(TRecInfo);
  FLastBookmark := FData.Count;
  FCurRec := FDataOffset - 1;
end;

procedure TFixedFormatDataSet.InternalClose;
begin
  if (not FReadOnly) and (FSaveChanges) then  // Write any edits to disk
    FData.SaveToFile(FileName);
  FLoadFromStream := False;
  FData.Clear;          // Clear data
  BindFields(FALSE);
  if DefaultFields then // Destroy the TField
    DestroyFields;
  FCurRec := -1;        // Reset these internal flags
  FLastBookmark := 0;
  FRecordSize := 0;
end;

function TFixedFormatDataSet.IsCursorOpen: Boolean;
begin
  Result := Assigned(FData) and (FRecordSize > 0);
end;

procedure TFixedFormatDataSet.InternalHandleException;
begin
{$ifndef fpc}
   Application.HandleException(Self);
{$else}
  inherited;
{$endif}
end;

// Loads Data from a stream.
procedure TFixedFormatDataSet.LoadFromStream(Stream: TStream);
begin
  if assigned(stream) then
  begin
    Active          := False; //Make sure the Dataset is Closed.
    Stream.Position := 0;     //Make sure you are at the top of the Stream.
    FLoadFromStream := True;
    if not Assigned(FData) then
      raise Exception.Create('Data buffer unassigned');
    if Assigned(FEncoding) then
      FData.LoadFromStream(Stream,FEncoding)
    else
      FData.LoadFromStream(Stream);
    Active := True;
  end
  else
    raise exception.Create('Invalid Stream Assigned (Load From Stream');
end;

// Saves Data as text to a stream.
procedure TFixedFormatDataSet.SaveToStream(Stream: TStream);
begin
  if assigned(stream) then
    if assigned(Fencoding) then
      FData.SaveToStream(Stream,FEncoding)
    else
      FData.SaveToStream(Stream)
  else
    raise exception.Create('Invalid Stream Assigned (Save To Stream');
end;

// Record Functions
function TFixedFormatDataSet.AllocRecordBuffer: TRecordBuffer;
begin
  if FRecBufSize > 0 then
    Result := AllocMem(FRecBufSize)
  else
    Result := nil;
end;

procedure TFixedFormatDataSet.FreeRecordBuffer(var Buffer: TRecordBuffer);
begin
  if Buffer <> nil then
    FreeMem(Buffer);
end;

procedure TFixedFormatDataSet.InternalInitRecord(Buffer: TRecordBuffer);
begin
  FillChar(Buffer[0], FRecordSize, 0);
end;

procedure TFixedFormatDataSet.ClearCalcFields(Buffer: TRecordBuffer);
begin
  FillChar(Buffer[RecordSize], CalcFieldsSize, 0);
end;

function TFixedFormatDataSet.GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode;
  DoCheck: Boolean): TGetResult;
var
  Accepted : Boolean;
begin
  if (FData.Count <= FDataOffset) then
    Result := grEOF
  else
  begin
    Result := grOK;
    repeat
      Accepted := TRUE;
      case GetMode of
        gmNext:
          if FCurRec >= FData.Count - 1  then
            Result := grEOF
          else
            Inc(FCurRec);
        gmPrior:
          if FCurRec <= FDataOffset then
            Result := grBOF
          else
            Dec(FCurRec);
        gmCurrent:
          if (FCurRec < FDataOffset) or (FCurRec >= FData.Count) then
            Result := grError;
      end;

      if Result = grOk then
      begin
        Move(StoreToBuf(FData[FCurRec])[1], Buffer[0], FRecordSize);
        with PRecInfo(Buffer + FRecInfoOfs)^ do
        begin
          Bookmark := PtrInt(FData.Objects[FCurRec]);
          BookmarkFlag := bfCurrent;
        end;
        if CalcFieldsSize > 0 then GetCalcFields(Buffer);

        if Filtered then
        begin
          Accepted := RecordFilter(Buffer);
          if not Accepted and (GetMode = gmCurrent) then
            Inc(FCurRec);
        end;
      end
      else if (Result = grError) and DoCheck then
        DatabaseError('No Records');
    until (Result <> grOK) or Accepted;
  end;
end;

function TFixedFormatDataSet.GetRecordCount: Longint;
begin
  Result := FData.Count - FDataOffset;
  if Result < 0 then Result := 0; // closed dataset
end;

function TFixedFormatDataSet.GetRecNo: Longint;
var
  RecBuf: TRecordBuffer;
begin
  if GetActiveRecBuf(RecBuf) and (State <> dsInsert) then
  begin
    UpdateCursorPos;
    Result := FCurRec + 1 - FDataOffset;
  end
  else
    Result := 0;
end;

procedure TFixedFormatDataSet.SetRecNo(Value: Integer);
begin
  CheckBrowseMode;
  if (Value >= 0) and (Value <= RecordCount) and (Value <> RecNo) then
  begin
    DoBeforeScroll;
    FCurRec := Value - 1 + FDataOffset;
    Resync([]);
    DoAfterScroll;
  end;
end;

function TFixedFormatDataSet.GetRecordSize: Word;
begin
  Result := FRecordSize;
end;

function TFixedFormatDataSet.GetActiveRecBuf(out RecBuf: TRecordBuffer): Boolean;
begin
  case State of
    dsCalcFields: RecBuf := CalcBuffer;
    dsFilter: RecBuf := FFilterBuffer;
    else if IsEmpty then RecBuf := nil else RecBuf := ActiveBuffer;
  end;
  Result := RecBuf <> nil;
end;

function TFixedFormatDataSet.RecordFilter(RecBuf: TRecordBuffer): Boolean;
var
  SaveState: TDataSetState;
begin                          // Returns true if accepted in the filter
  SaveState := SetTempState(dsFilter);
  FFilterBuffer := RecBuf;
  Result := TRUE;
  if Result and Assigned(OnFilterRecord) then
    OnFilterRecord(Self, Result);
  RestoreState(SaveState);
end;

function TFixedFormatDataSet.GetCanModify: boolean;
begin
  Result := not FReadOnly;
end;

// Field Related
procedure TFixedFormatDataSet.LoadFieldScheme(List : TStrings; MaxSize : Integer);
var
  tmpFieldName : string;
  tmpSchema : TStrings;
  i : Integer;
begin
  tmpSchema := TStringList.Create;
  try       // Load Schema Structure
    if (Schema.Count > 0) then
    begin
      tmpSchema.Assign(Schema);
      RemoveWhiteLines(tmpSchema, FALSE);
    end
    else
      tmpSchema.Add('Line');
    for i := 0 to tmpSchema.Count -1 do // Interpret Schema
    begin
      tmpFieldName := tmpSchema.Names[i];
      if (tmpFieldName = '') then
        tmpFieldName := Format('%s=%d', [tmpSchema[i], MaxSize])
      else
        tmpFieldName := tmpSchema[i];
      List.Add(tmpFieldName);
    end;
  finally
    tmpSchema.Free;
  end;
end;

function TFixedFormatDataSet.GetFieldData(Field: TField; Buffer: Pointer): Boolean;
var
  RecBuf,
  BufEnd: PChar;
begin
  Result := GetActiveRecBuf(TRecordBuffer(RecBuf));
  if Result then
  begin
    if Field.FieldNo > 0 then
    begin
      SetFieldOfs(TRecordBuffer(RecBuf), Field.FieldNo);
      Result := RecBuf < StrEnd(RecBuf); // just ''=Null
      if Result and Assigned(Buffer) then
      begin
        StrLCopy(Buffer, RecBuf, Field.DataSize);
        if FTrimSpace then // trim trailing spaces
        begin
          BufEnd := StrEnd(Buffer);
          repeat
            Dec(BufEnd);
            if (BufEnd^ = ' ') then
              BufEnd^ := #0
            else
              break;
          until (BufEnd = Buffer);
        end;
      end;
    end
    else // fkCalculated, fkLookup
    begin
      Inc(RecBuf, FRecordSize + Field.Offset); // Offset is calculated using DataSize not Size
      Result := Boolean(RecBuf[0]);
      if Result and Assigned(Buffer) then
        Move(RecBuf[1], Buffer^, Field.DataSize);
    end;
  end;
end;

procedure TFixedFormatDataSet.SetFieldData(Field: TField; Buffer: Pointer);
var
  RecBuf: PChar;
begin
  if not (State in dsWriteModes) then
    DatabaseErrorFmt(SNotEditing, [Name], Self);
  GetActiveRecBuf(TRecordBuffer(RecBuf));
  if Field.FieldNo > 0 then
  begin
    if State = dsCalcFields then
      DatabaseError('Dataset not in edit or insert mode', Self);
    if Field.ReadOnly and not (State in [dsSetKey, dsFilter]) then
      DatabaseErrorFmt(SReadOnlyField, [Field.DisplayName]);
    if State in [dsEdit, dsInsert, dsNewValue] then
      Field.Validate(Buffer);
    if Assigned(Buffer) and (Field.FieldKind <> fkInternalCalc) then
    begin
      SetFieldOfs(TRecordBuffer(RecBuf), Field.FieldNo);
      Move(Buffer^, RecBuf[0], Field.DataSize);
    end;
  end
  else // fkCalculated, fkLookup
  begin
    Inc(RecBuf, FRecordSize + Field.Offset);
    Boolean(RecBuf[0]) := Assigned(Buffer);
    if Assigned(Buffer) then
      Move(Buffer^, RecBuf[1], Field.DataSize);
  end;
  if not (State in [dsCalcFields, dsFilter, dsNewValue]) then
    DataEvent(deFieldChange, PtrInt(Field));
end;

procedure TFixedFormatDataSet.SetFieldOfs(var Buffer : TRecordBuffer; FieldNo : Integer);
var
  i, Len : Integer;
begin
  i := 1;
  while (i < FieldNo) and (i < FieldDefs.Count) do
  begin
    Len := FieldDefDataSize(FieldDefs[i-1]);
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
    Len := Align(Len, SizeOf(PtrInt));
{$ENDIF}
    Inc(Buffer, Len);
    Inc(i);
  end;
end;

// Navigation / Editing
procedure TFixedFormatDataSet.InternalFirst;
begin
  FCurRec := FDataOffset - 1;
end;

procedure TFixedFormatDataSet.InternalLast;
begin
  FCurRec := FData.Count;
end;

procedure TFixedFormatDataSet.InternalPost;
begin
  inherited InternalPost;
  FSaveChanges := TRUE;
  if (State = dsEdit) then // just update the data in the string list
    FData[FCurRec] := BufToStore(ActiveBuffer)
  else // append or insert
    InternalAddRecord(ActiveBuffer, GetBookmarkFlag(ActiveBuffer)=bfEOF);
end;

procedure TFixedFormatDataSet.InternalEdit;
begin

end;

procedure TFixedFormatDataSet.InternalDelete;
begin
  FSaveChanges := TRUE;
  FData.Delete(FCurRec);
  if FCurRec >= FData.Count then
    Dec(FCurRec);
end;

procedure TFixedFormatDataSet.InternalAddRecord(Buffer: Pointer; DoAppend: Boolean);
begin
  FSaveChanges := TRUE;
  Inc(FLastBookmark);
  if DoAppend then
    InternalLast;
  if (FCurRec >= FDataOffset) then
    FData.InsertObject(FCurRec, BufToStore(Buffer), TObject(Pointer(FLastBookmark)))
  else
    FData.AddObject(BufToStore(Buffer), TObject(Pointer(FLastBookmark)));
end;

function TFixedFormatDataSet.BookmarkValid(ABookmark: TBookmark): Boolean;
begin
  Result := Assigned(ABookmark) and (FData.IndexOfObject(TObject(PPtrInt(ABookmark)^)) <> -1);
end;

function TFixedFormatDataSet.CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint;
const r: array[Boolean, Boolean] of ShortInt = ((2,-1),(1,0));
begin
  Result := r[Bookmark1=nil, Bookmark2=nil];
  if Result = 2 then
    Result := PPtrInt(Bookmark1)^ - PPtrInt(Bookmark2)^;
end;

procedure TFixedFormatDataSet.InternalGotoBookmark(ABookmark: Pointer);
var
  Index: Integer;
begin
  Index := FData.IndexOfObject(TObject(PPtrInt(ABookmark)^));
  if Index <> -1 then
    FCurRec := Index
  else
    DatabaseError('Bookmark not found');
end;

procedure TFixedFormatDataSet.InternalSetToRecord(Buffer: TRecordBuffer);
begin
  if (State <> dsInsert) then
    InternalGotoBookmark(@PRecInfo(Buffer + FRecInfoOfs)^.Bookmark);
end;

function TFixedFormatDataSet.GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag;
begin
  Result := PRecInfo(Buffer + FRecInfoOfs)^.BookmarkFlag;
end;

procedure TFixedFormatDataSet.SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag);
begin
  PRecInfo(Buffer + FRecInfoOfs)^.BookmarkFlag := Value;
end;

procedure TFixedFormatDataSet.GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
begin
  Move(Buffer[FRecInfoOfs], Data^, BookmarkSize);
end;

procedure TFixedFormatDataSet.SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
begin
  Move(Data^, Buffer[FRecInfoOfs], BookmarkSize);
end;

procedure TFixedFormatDataSet.RemoveWhiteLines(List : TStrings; IsFileRecord : Boolean);
var
  i : integer;
begin
  for i := List.Count -1 downto 0 do
  begin
    if (Trim(List[i]) = '' ) then
      if IsFileRecord then
      begin
        FCurRec := i;
        InternalDelete;
      end
      else
        List.Delete(i);
  end;
end;

procedure TFixedFormatDataSet.RemoveBlankRecords;
begin
  RemoveWhiteLines(FData, TRUE);
end;

procedure TFixedFormatDataSet.RemoveExtraColumns;
var
  i : Integer;
begin
  for i := FData.Count -1 downto 0 do
    FData[i] := BufToStore(TRecordBuffer(StoreToBuf(FData[i])));
  FData.SaveToFile(FileName);
end;

procedure TFixedFormatDataSet.SaveFileAs(strFileName : String);
begin
  FData.SaveToFile(strFileName);
  FFileName := strFileName;
  FSaveChanges := FALSE;
end;

function TFixedFormatDataSet.StoreToBuf(Source: String): String;
var i, Len: integer;
    Src, Dest: PChar;
begin
  // moves fixed length fields from Source to record buffer and null-terminates each field
  SetLength(Result, FRecordSize);
  Src  := PChar(Source);
  Dest := PChar(Result);
  for i := 0 to FieldDefs.Count - 1 do
  begin
    Len := FieldDefDataSize(FieldDefs[i])-1;
    Move(Src^, Dest^, Len);
    Inc(Src, Len);
    Inc(Dest, Len);
    Dest^ := #0;
    Inc(Dest);
  end;
end;

function TFixedFormatDataSet.BufToStore(Buffer: TRecordBuffer): String;
var i, Len, SrcLen: integer;
    Src, Dest: PChar;
begin
  // calculate fixed length record size
  Len := 0;
  for i := 0 to FieldDefs.Count - 1 do
    Inc(Len, FieldDefDataSize(FieldDefs[i])-1);
  SetLength(Result, Len);

  Src  := PChar(Buffer);
  Dest := PChar(Result);
  for i := 0 to FieldDefs.Count - 1 do
  begin
    Len := FieldDefDataSize(FieldDefs[i])-1;
    Move(Src^, Dest^, Len);
    // fields in record buffer are null-terminated, but pad them with spaces to fixed length
    SrcLen := StrLen(Src);
    FillChar(Dest[SrcLen], Len-SrcLen, ' ');
    Inc(Src, Len+1);
    Inc(Dest, Len);
  end;
end;


//-----------------------------------------------------------------------------
// TSDFStringList
//-----------------------------------------------------------------------------

procedure TSDFStringList.SetTextStr(const Value: string);
var
  S: string;
  P: integer;

  function GetNextLine(const Value: string; out S: string; var P: Integer): Boolean;
  const
    CR: char = #13;
    LF: char = #10;
    DQ: char = '"';
  var
    L, P1: integer;
    InDQ: boolean;
  begin
    // RFC 4180:
    //  Each record is located on a separate line, delimited by a line break (CRLF)
    //  Fields containing line breaks (CRLF), double quotes, and commas should be enclosed in double-quotes.
    Result := False;
    L := Length(Value);
    if P > L then Exit;
    P1 := P;
    InDQ := False;
    while (P <= L) and (not(Value[P] in [CR,LF]) or InDQ) do
    begin
      if Value[P] = DQ then InDQ := not InDQ;
      inc(P);
    end;
    S := Copy(Value, P1, P-P1);
    if (P <= L) and (Value[P] = CR) then
      inc(P);
    if (P <= L) and (Value[P] = LF) then
      inc(P);
    Result := True;
  end;

begin
  if FMultiLine then // CRLF can be enclosed between double-quotes
    try
      BeginUpdate;
      Clear;
      P:=1;
      while GetNextLine(Value,S,P) do
        Add(S);
    finally
      EndUpdate;
    end
  else
    inherited;
end;


//-----------------------------------------------------------------------------
// TSdfDataSet
//-----------------------------------------------------------------------------
constructor TSdfDataSet.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FDelimiter := ',';
  FFirstLineAsSchema := FALSE;
  FMultiLine := False;
end;

function TSdfDataSet.ExtractDelimited(const S: String; var Pos: integer): string;
const
  CR: char = #13;
  LF: char = #10;
  DQ: char = '"';
var
  Len, P1: integer;
  pSrc, pDest: PChar;
begin
  Len := Length(S);
  P1 := Pos;

  // RFC 4180:
  //   Spaces are considered part of a field and should not be ignored
  //
  //   If double-quotes are used to enclose fields, then a double-quote
  //   appearing inside a field must be escaped by preceding it with
  //   another double quote

  if (S[Pos] = DQ) then
    // quoted field
    begin
    // skip leading double-quote
    Inc(Pos);
    // allocate output buffer
    SetLength(Result, Len-P1+1);
    pSrc := @S[Pos];
    pDest := @Result[1];
    while (Pos <= Len) do
      begin
      if (pSrc[0] = DQ) then
        begin
        if (pSrc[1] = DQ) then // doubled DQ
          begin
          Inc(pSrc);           // dequote double-quote
          Inc(Pos);
          end
        else if (pSrc[1] in [Delimiter,' ',CR,LF,#0]) then // DQ followed by delimiter or end of record
          break;
        end
      else if not FMultiLine and (pSrc[0] in [CR,LF,#0]) then // end of record while multiline disabled
        break;
      pDest^ := pSrc^;
      Inc(pSrc);
      Inc(pDest);
      Inc(Pos);
      end;
    SetLength(Result, pDest-@Result[1]);
    // skip trailing DQ and white spaces after DQ
    while (Pos <= Len) and not(S[Pos] in [Delimiter,CR,LF,#0]) do
      Inc(Pos);
    end
  else
    // unquoted field
    begin
    while (Pos <= Len) and not(S[Pos] in [Delimiter,CR,LF,#0]) do
      Inc(Pos);
    Result := Copy(S, P1, Pos-P1);
    end;

  // skip final field delimiter
  if (Pos <= Len) and (S[Pos] = Delimiter) then
    Inc(Pos);
  // skip end of record, line break CRLF
  while (Pos <= Len) and (S[Pos] in [CR,LF]) do
    Inc(Pos);
end;

procedure TSdfDataSet.InternalInitFieldDefs;
var
  Len, Pos : Integer;
  SchemaLine, S, FN : String;

begin
  if not IsCursorOpen then
    exit;
  if (FData.Count = 0) and (Schema.Count > 0) and FirstLineAsSchema then
  begin
    Schema.Delimiter := Delimiter;
    FData.Append(Schema.DelimitedText);
  end
  else if (FData.Count = 0) or (Trim(FData[0]) = '') then
  begin
    FirstLineAsSchema := FALSE;
  end
  else if (Schema.Count = 0) or FirstLineAsSchema then
  begin
    Schema.Clear;
    SchemaLine:=FData[0];

    if StripTrailingDelimiters then
      DoStripTrailingDelimiters(SchemaLine);

    Len := Length(SchemaLine);
    Pos := 1;
    while Pos <= Len do
    begin
      S := ExtractDelimited(SchemaLine, Pos);
      if FirstLineAsSchema then
        FN := S
      else
        FN := '';
      if FN = '' then // Special case: "a,b,,c"
        FN := Format('Field%d', [Schema.Count + 1]);
      Schema.Add(FN);
    end;
    // Special case: "f1,f2," are 3 fields, last unnamed.
    if (Len>0) and (SchemaLine[Len]=Delimiter) then
      Schema.Add(Format('Field%d', [Schema.Count + 1]));
  end;
  inherited;
end;

function TSdfDataSet.StoreToBuf(Source: String): String;
var
  MaxLen, // Maximum field length as defined in FieldDefs + null terminator
  i,
  Pos,
  Len     : Integer; // Actual length of field
  S       : String;
  Dest    : PChar;
begin
  SetLength(Result, FRecordSize);
  FillChar(Result[1], FRecordSize, Ord(' '));

  Pos := 1;
  Dest := PChar(Result);

  for i := 0 to FieldDefs.Count - 1 do
  begin
    MaxLen := FieldDefDataSize(FieldDefs[i])-1;
    S := ExtractDelimited(Source, Pos);
    Len := Length(S);

    if Len > MaxLen then
      Len := MaxLen;

    if Len = 0 then // bug in StrPLCopy
      Dest^ := #0
    else
      StrPLCopy(Dest, S, Len+1); // null-terminate
    Inc(Dest, MaxLen+1);
   end;
end;

function TSdfDataSet.BufToStore(Buffer: TRecordBuffer): String;
const
  CR: char = #13;
  LF: char = #10;
  DQ: char = '"';
var
  Src: PChar;
  S : String;
  i, MaxLen, Len : Integer;
  QuoteMe: boolean;
begin
  Result := '';
  Src := PChar(Buffer);
  for i := 0 to FieldDefs.Count - 1 do
  begin
    MaxLen := FieldDefDataSize(FieldDefs[i])-1;
    Len := StrLen(Src); // field values are null-terminated in record buffer
    if Len > MaxLen then
      Len := MaxLen;
    SetString(S, Src, Len);
    Inc(Src, MaxLen+1);

    QuoteMe:=false;
    if FMultiLine then
      begin
      // If multiline enabled, quote whenever we find carriage return or linefeed
      if (not QuoteMe) and ((Pos(CR, S) > 0) or (Pos(LF, S) > 0)) then QuoteMe:=true;
      end
    else
      begin
      // If we don't allow multiline, remove all CR and LF because they mess with the record ends:
      S := StringReplace(S, CR, '', [rfReplaceAll]);
      S := StringReplace(S, LF, '', [rfReplaceAll]);
      end;

    // Check for any delimiters or quotes occurring in field text
    if not QuoteMe then
      QuoteMe := (Pos(FDelimiter, S) > 0) or (Pos(DQ, S) > 0);

    if QuoteMe then
      S := AnsiQuotedStr(S, DQ);

    Result := Result + S + FDelimiter;
  end;
  DoStripTrailingDelimiters(Result)
end;

procedure TSdfDataSet.DoStripTrailingDelimiters(var S: String);
var
  L,P : integer;
begin
  L:=Length(S);
  P:=L;
  while (P>0) and (S[P]=FDelimiter) and ((P=L) or StripTrailingDelimiters) do
    Dec(P);
  if P<L then
    S:=Copy(S,1,P);
end;

procedure TSdfDataSet.SetDelimiter(Value : Char);
begin
  CheckInactive;
  FDelimiter := Value;
end;

procedure TSdfDataSet.SetFirstLineAsSchema(Value : Boolean);
begin
  CheckInactive;
  FFirstLineAsSchema := Value;
  FDataOffset:=Ord(FFirstLineAsSchema);
end;

procedure TSdfDataSet.SetMultiLine(const Value: Boolean);
begin
  FMultiLine:=Value;
  FData.FMultiLine:=Value;
end;


//-----------------------------------------------------------------------------
// This procedure is used to register this component on the component palette
//-----------------------------------------------------------------------------
procedure Register;
begin
  RegisterComponents('Data Access', [TFixedFormatDataSet]);
  RegisterComponents('Data Access', [TSdfDataSet]);
end;

end.
