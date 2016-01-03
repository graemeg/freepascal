unit dbf_parser;

interface

{$I dbf_common.inc}

uses
  SysUtils,
  Classes,
{$ifdef KYLIX}
  Libc,
{$endif}
{$ifndef WINDOWS}
  dbf_wtil,
{$endif}
  db,
  dbf_prscore,
  dbf_common,
  dbf_fields,
  dbf_prsdef,
  dbf_prssupp;

type

  TStringFieldMode = (smRaw, smAnsi, smAnsiTrim);

  TDbfParser = class(TCustomExpressionParser)
  private
    FDbfFile: Pointer;
    FFieldVarList: TStringList;
    FIsExpression: Boolean;       // expression or simple field?
    FFieldType: TExpressionType;
    FCaseInsensitive: Boolean;
    FStringFieldMode: TStringFieldMode;
    FPartialMatch: boolean;

  protected
    FCurrentExpression: string;

    procedure FillExpressList; override;
    procedure HandleUnknownVariable(VarName: string); override;
    function  GetVariableInfo(VarName: string): TDbfFieldDef;
    function  CurrentExpression: string; override;
    procedure ValidateExpression(AExpression: string); virtual;
    function  GetResultType: TExpressionType; override;
    function  GetResultLen: Integer;

    procedure SetCaseInsensitive(NewInsensitive: Boolean);
    procedure SetStringFieldMode(NewMode: TStringFieldMode);
    procedure SetPartialMatch(NewPartialMatch: boolean);
  public
    constructor Create(ADbfFile: Pointer);
    destructor Destroy; override;

    procedure ClearExpressions; override;

    procedure ParseExpression(AExpression: string); virtual;
    function ExtractFromBuffer(Buffer: TRecordBuffer): PChar; virtual;

    property DbfFile: Pointer read FDbfFile write FDbfFile;
    property Expression: string read FCurrentExpression;
    property ResultLen: Integer read GetResultLen;

    property CaseInsensitive: Boolean read FCaseInsensitive write SetCaseInsensitive;
    property StringFieldMode: TStringFieldMode read FStringFieldMode write SetStringFieldMode;
    property PartialMatch: boolean read FPartialMatch write SetPartialMatch;
  end;

implementation

uses
  dbf,
  dbf_dbffile,
  dbf_str
{$ifdef WINDOWS}
  ,Windows
{$endif}
  ;

type
// TFieldVar aids in retrieving field values from records
// in their proper type

  TFieldVar = class(TObject)
  private
    FFieldDef: TDbfFieldDef;
    FDbfFile: TDbfFile;
    FFieldName: string;
    FExprWord: TExprWord;
  protected
    function GetFieldVal: Pointer; virtual; abstract;
    function GetFieldType: TExpressionType; virtual; abstract;
    procedure SetExprWord(NewExprWord: TExprWord); virtual;

    property ExprWord: TExprWord read FExprWord write SetExprWord;
  public
    constructor Create(UseFieldDef: TDbfFieldDef; ADbfFile: TDbfFile);

    procedure Refresh(Buffer: TRecordBuffer); virtual; abstract;

    property FieldVal: Pointer read GetFieldVal;
    property FieldDef: TDbfFieldDef read FFieldDef;
    property FieldType: TExpressionType read GetFieldType;
    property DbfFile: TDbfFile read FDbfFile;
    property FieldName: string read FFieldName;
  end;

  TStringFieldVar = class(TFieldVar)
  protected
    FFieldVal: PChar;
    FMode: TStringFieldMode;

    function GetFieldVal: Pointer; override;
    function GetFieldType: TExpressionType; override;
    procedure SetExprWord(NewExprWord: TExprWord); override;
    procedure SetMode(NewMode: TStringFieldMode);
    procedure UpdateExprWord;
  public
    destructor Destroy; override;

    procedure Refresh(Buffer: TRecordBuffer); override;

    property Mode: TStringFieldMode read FMode write SetMode;
  end;

  TFloatFieldVar = class(TFieldVar)
  private
    FFieldVal: Double;
  protected
    function GetFieldVal: Pointer; override;
    function GetFieldType: TExpressionType; override;
  public
    procedure Refresh(Buffer: TRecordBuffer); override;
  end;

  TIntegerFieldVar = class(TFieldVar)
  private
    FFieldVal: Integer;
  protected
    function GetFieldVal: Pointer; override;
    function GetFieldType: TExpressionType; override;
  public
    procedure Refresh(Buffer: TRecordBuffer); override;
  end;

{$ifdef SUPPORT_INT64}
  TLargeIntFieldVar = class(TFieldVar)
  private
    FFieldVal: Int64;
  protected
    function GetFieldVal: Pointer; override;
    function GetFieldType: TExpressionType; override;
  public
    procedure Refresh(Buffer: TRecordBuffer); override;
  end;
{$endif}

  TDateTimeFieldVar = class(TFieldVar)
  private
    FFieldVal: TDateTimeRec;
    function GetFieldType: TExpressionType; override;
  protected
    function GetFieldVal: Pointer; override;
  public
    procedure Refresh(Buffer: TRecordBuffer); override;
  end;

  TBooleanFieldVar = class(TFieldVar)
  private
    FFieldVal: boolean;
    function GetFieldType: TExpressionType; override;
  protected
    function GetFieldVal: Pointer; override;
  public
    procedure Refresh(Buffer: TRecordBuffer); override;
  end;

{ TFieldVar }

constructor TFieldVar.Create(UseFieldDef: TDbfFieldDef; ADbfFile: TDbfFile);
begin
  inherited Create;

  // store field
  FFieldDef := UseFieldDef;
  FDbfFile := ADbfFile;
  FFieldName := UseFieldDef.FieldName;
end;

procedure TFieldVar.SetExprWord(NewExprWord: TExprWord);
begin
  FExprWord := NewExprWord;
end;

{ TStringFieldVar }

destructor TStringFieldVar.Destroy;
begin
  if FMode <> smRaw then
    FreeMem(FFieldVal);

  inherited;
end;

function TStringFieldVar.GetFieldVal: Pointer;
begin
  Result := @FFieldVal;
end;

function TStringFieldVar.GetFieldType: TExpressionType;
begin
  Result := etString;
end;

procedure TStringFieldVar.Refresh(Buffer: TRecordBuffer);
var
  Len: Integer;
  Src: TRecordBuffer;
begin
  Src := Buffer+FieldDef.Offset;
  if FMode <> smRaw then
  begin
    // copy field data
    Len := FieldDef.Size;
    if FMode = smAnsiTrim then
<<<<<<< HEAD
<<<<<<< HEAD
      while (Len >= 1) and (Src[Len-1] = TRecordbufferbasetype(' ')) do Dec(Len);
=======
      while (Len >= 1) and (Src[Len-1] = ' ') do Dec(Len);
>>>>>>> graemeg/fixes_2_2
=======
      while (Len >= 1) and (Src[Len-1] = ' ') do Dec(Len);
>>>>>>> origin/fixes_2_2
    // translate to ANSI
    Len := TranslateString(DbfFile.UseCodePage, GetACP, pansichar(Src), FFieldVal, Len);
    FFieldVal[Len] := #0;
  end else
    FFieldVal := pansichar(Src);
end;

procedure TStringFieldVar.SetExprWord(NewExprWord: TExprWord);
begin
  inherited;
  UpdateExprWord;
end;

procedure TStringFieldVar.UpdateExprWord;
begin
  if FMode <> smAnsiTrim then
    FExprWord.FixedLen := FieldDef.Size
  else
    FExprWord.FixedLen := -1;
end;

procedure TStringFieldVar.SetMode(NewMode: TStringFieldMode);
begin
  if NewMode = FMode then exit;
  FMode := NewMode;
  if NewMode = smRaw then
  begin
    FreeMem(FFieldVal);
    FFieldVal := nil;
  end else
    GetMem(FFieldVal, FieldDef.Size*3+1);
  UpdateExprWord;
end;

//--TFloatFieldVar-----------------------------------------------------------
function TFloatFieldVar.GetFieldVal: Pointer;
begin
  Result := @FFieldVal;
end;

function TFloatFieldVar.GetFieldType: TExpressionType;
begin
  Result := etFloat;
end;

procedure TFloatFieldVar.Refresh(Buffer: TRecordBuffer);
begin
  // database width is default 64-bit double
  if not FDbfFile.GetFieldDataFromDef(FieldDef, FieldDef.FieldType, Buffer, @FFieldVal, false) then
    FFieldVal := 0.0;
end;

//--TIntegerFieldVar----------------------------------------------------------
function TIntegerFieldVar.GetFieldVal: Pointer;
begin
  Result := @FFieldVal;
end;

function TIntegerFieldVar.GetFieldType: TExpressionType;
begin
  Result := etInteger;
end;

procedure TIntegerFieldVar.Refresh(Buffer: TRecordBuffer);
begin
  FFieldVal := 0;
  FDbfFile.GetFieldDataFromDef(FieldDef, FieldDef.FieldType, Buffer, @FFieldVal, false);
end;

{$ifdef SUPPORT_INT64}

//--TLargeIntFieldVar----------------------------------------------------------
function TLargeIntFieldVar.GetFieldVal: Pointer;
begin
  Result := @FFieldVal;
end;

function TLargeIntFieldVar.GetFieldType: TExpressionType;
begin
  Result := etLargeInt;
end;

procedure TLargeIntFieldVar.Refresh(Buffer: TRecordBuffer);
begin
  if not FDbfFile.GetFieldDataFromDef(FieldDef, FieldDef.FieldType, Buffer, @FFieldVal, false) then
    FFieldVal := 0;
end;

{$endif}

//--TDateTimeFieldVar---------------------------------------------------------
function TDateTimeFieldVar.GetFieldVal: Pointer;
begin
  Result := @FFieldVal;
end;

function TDateTimeFieldVar.GetFieldType: TExpressionType;
begin
  Result := etDateTime;
end;

procedure TDateTimeFieldVar.Refresh(Buffer: TRecordBuffer);
begin
  if not FDbfFile.GetFieldDataFromDef(FieldDef, ftDateTime, Buffer, @FFieldVal, false) then
    FFieldVal.DateTime := 0.0;
end;

//--TBooleanFieldVar---------------------------------------------------------
function TBooleanFieldVar.GetFieldVal: Pointer;
begin
  Result := @FFieldVal;
end;

function TBooleanFieldVar.GetFieldType: TExpressionType;
begin
  Result := etBoolean;
end;

procedure TBooleanFieldVar.Refresh(Buffer: TRecordBuffer);
var
  lFieldVal: word;
begin
  if FDbfFile.GetFieldDataFromDef(FieldDef, ftBoolean, Buffer, @lFieldVal, false) then
    FFieldVal := lFieldVal <> 0
  else
    FFieldVal := false;
end;

//--TDbfParser---------------------------------------------------------------

constructor TDbfParser.Create(ADbfFile: Pointer);
begin
  FDbfFile := ADbfFile;
  FFieldVarList := TStringList.Create;
  FCaseInsensitive := true;
  inherited Create;
end;

destructor TDbfParser.Destroy;
begin
  ClearExpressions;
  inherited;
  FreeAndNil(FFieldVarList);
end;

function TDbfParser.GetResultType: TExpressionType;
begin
  // if not a real expression, return type ourself
  if FIsExpression then
    Result := inherited GetResultType
  else
    Result := FFieldType;
end;

function TDbfParser.GetResultLen: Integer;
begin
  // set result len for fixed length expressions / fields
  case ResultType of
    etBoolean:  Result := 1;
    etInteger:  Result := 4;
    etFloat:    Result := 8;
    etDateTime: Result := 8;
    etString:
    begin
      if not FIsExpression and (TStringFieldVar(FFieldVarList.Objects[0]).Mode <> smAnsiTrim) then
        Result := TStringFieldVar(FFieldVarList.Objects[0]).FieldDef.Size
      else
        Result := -1;
    end;
  else
    Result := -1;
  end;
end;

procedure TDbfParser.SetCaseInsensitive(NewInsensitive: Boolean);
begin
  if FCaseInsensitive <> NewInsensitive then
  begin
    // clear and regenerate functions
    FCaseInsensitive := NewInsensitive;
    FillExpressList;
  end;
end;

procedure TDbfParser.SetPartialMatch(NewPartialMatch: boolean);
begin
  if FPartialMatch <> NewPartialMatch then
  begin
    // refill function list
    FPartialMatch := NewPartialMatch;
    FillExpressList;
  end;
end;

procedure TDbfParser.SetStringFieldMode(NewMode: TStringFieldMode);
var
  I: integer;
begin
  if FStringFieldMode <> NewMode then
  begin
    // clear and regenerate functions, custom fields will be deleted too
    FStringFieldMode := NewMode;
    for I := 0 to FFieldVarList.Count - 1 do
      if FFieldVarList.Objects[I] is TStringFieldVar then
        TStringFieldVar(FFieldVarList.Objects[I]).Mode := NewMode;
  end;
end;

procedure TDbfParser.FillExpressList;
var
  lExpression: string;
begin
  lExpression := FCurrentExpression;
  ClearExpressions;
  FWordsList.FreeAll;
  FWordsList.AddList(DbfWordsGeneralList, 0, DbfWordsGeneralList.Count - 1);
  if FCaseInsensitive then
  begin
    FWordsList.AddList(DbfWordsInsensGeneralList, 0, DbfWordsInsensGeneralList.Count - 1);
    if FPartialMatch then
    begin
      FWordsList.AddList(DbfWordsInsensPartialList, 0, DbfWordsInsensPartialList.Count - 1);
    end else begin
      FWordsList.AddList(DbfWordsInsensNoPartialList, 0, DbfWordsInsensNoPartialList.Count - 1);
    end;
  end else begin
    FWordsList.AddList(DbfWordsSensGeneralList, 0, DbfWordsSensGeneralList.Count - 1);
    if FPartialMatch then
    begin
      FWordsList.AddList(DbfWordsSensPartialList, 0, DbfWordsSensPartialList.Count - 1);
    end else begin
      FWordsList.AddList(DbfWordsSensNoPartialList, 0, DbfWordsSensNoPartialList.Count - 1);
    end;
  end;
  if Length(lExpression) > 0 then
    ParseExpression(lExpression);
end;

function TDbfParser.GetVariableInfo(VarName: string): TDbfFieldDef;
begin
  Result := TDbfFile(FDbfFile).GetFieldInfo(VarName);
end;

procedure TDbfParser.HandleUnknownVariable(VarName: string);
var
  FieldInfo: TDbfFieldDef;
  TempFieldVar: TFieldVar;
begin
  // is this variable a fieldname?
  FieldInfo := GetVariableInfo(VarName);
  if FieldInfo = nil then
    raise EDbfError.CreateFmt(STRING_INDEX_BASED_ON_UNKNOWN_FIELD, [VarName]);

  // define field in parser
  case FieldInfo.FieldType of
    ftString:
      begin
        TempFieldVar := TStringFieldVar.Create(FieldInfo, TDbfFile(FDbfFile));
        TempFieldVar.ExprWord := DefineStringVariable(VarName, TempFieldVar.FieldVal);
        TStringFieldVar(TempFieldVar).Mode := FStringFieldMode;
      end;
    ftBoolean:
      begin
        TempFieldVar := TBooleanFieldVar.Create(FieldInfo, TDbfFile(FDbfFile));
        TempFieldVar.ExprWord := DefineBooleanVariable(VarName, TempFieldVar.FieldVal);
      end;
    ftFloat:
      begin
        TempFieldVar := TFloatFieldVar.Create(FieldInfo, TDbfFile(FDbfFile));
        TempFieldVar.ExprWord := DefineFloatVariable(VarName, TempFieldVar.FieldVal);
      end;
    ftAutoInc, ftInteger, ftSmallInt:
      begin
        TempFieldVar := TIntegerFieldVar.Create(FieldInfo, TDbfFile(FDbfFile));
        TempFieldVar.ExprWord := DefineIntegerVariable(VarName, TempFieldVar.FieldVal);
      end;
{$ifdef SUPPORT_INT64}
    ftLargeInt:
      begin
        TempFieldVar := TLargeIntFieldVar.Create(FieldInfo, TDbfFile(FDbfFile));
        TempFieldVar.ExprWord := DefineLargeIntVariable(VarName, TempFieldVar.FieldVal);
      end;
{$endif}
    ftDate, ftDateTime:
      begin
        TempFieldVar := TDateTimeFieldVar.Create(FieldInfo, TDbfFile(FDbfFile));
        TempFieldVar.ExprWord := DefineDateTimeVariable(VarName, TempFieldVar.FieldVal);
      end;
  else
    raise EDbfError.CreateFmt(STRING_INDEX_BASED_ON_INVALID_FIELD, [VarName]);
  end;

  // add to our own list
  FFieldVarList.AddObject(VarName, TempFieldVar);
end;

function TDbfParser.CurrentExpression: string;
begin
  Result := FCurrentExpression;
end;

procedure TDbfParser.ClearExpressions;
var
  I: Integer;
begin
  inherited;

  // test if already freed
  if FFieldVarList <> nil then
  begin
    // free field list
    for I := 0 to FFieldVarList.Count - 1 do
    begin
      // replacing with nil = undefining variable
      FWordsList.DoFree(TFieldVar(FFieldVarList.Objects[I]).FExprWord);
      TFieldVar(FFieldVarList.Objects[I]).Free;
    end;
    FFieldVarList.Clear;
  end;

  // clear expression
  FCurrentExpression := EmptyStr;
end;

procedure TDbfParser.ValidateExpression(AExpression: string);
begin
end;

procedure TDbfParser.ParseExpression(AExpression: string);
begin
  // clear any current expression
  ClearExpressions;

  // is this a simple field or complex expression?
  FIsExpression := GetVariableInfo(AExpression) = nil;
  if FIsExpression then
  begin
    // parse requested
    CompileExpression(AExpression);
  end else begin
    // simple field, create field variable for it
    HandleUnknownVariable(AExpression);
    FFieldType := TFieldVar(FFieldVarList.Objects[0]).FieldType;
  end;

  ValidateExpression(AExpression);

  // if no errors, assign current expression
  FCurrentExpression := AExpression;
end;

function TDbfParser.ExtractFromBuffer(Buffer: TRecordBuffer): PChar;
var
  I: Integer;
begin
  // prepare all field variables
  for I := 0 to FFieldVarList.Count - 1 do
    TFieldVar(FFieldVarList.Objects[I]).Refresh(Buffer);

  // complex expression?
  if FIsExpression then
  begin
    // execute expression
    EvaluateCurrent;
    Result := ExpResult;
  end else begin
    // simple field, get field result
    Result := TFieldVar(FFieldVarList.Objects[0]).FieldVal;
    // if string then dereference
    if FFieldType = etString then
      Result := PPChar(Result)^;
  end;
end;

end.

