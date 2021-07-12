
procedure SkipQuotesString(var p : pchar; QuoteChar : char; EscapeSlash, EscapeRepeat : Boolean);
var notRepeatEscaped : boolean;
begin
  Inc(p);
  repeat
    notRepeatEscaped := True;
    while not (p^ in [#0, QuoteChar]) do
    begin
      if EscapeSlash and (p^='\') and (p[1] <> #0) then Inc(p,2) // make sure we handle \' and \\ correct
      else Inc(p);
    end;
    if p^=QuoteChar then
    begin
      Inc(p); // skip final '
      if (p^=QuoteChar) and EscapeRepeat then // Handle escaping by ''
      begin
      notRepeatEscaped := False;
      inc(p);
      end
    end;
  until notRepeatEscaped;
end;

{ TParamsEnumerator }

function TParamsEnumerator.GetCurrent: TParam;
begin
  Result := FParams[FPosition];
end;

constructor TParamsEnumerator.Create(AParams: TParams);
begin
  inherited Create;
  FParams := AParams;
  FPosition := -1;
end;

function TParamsEnumerator.MoveNext: Boolean;
begin
  inc(FPosition);
  Result := FPosition < FParams.Count;
end;

{ TParams }

function TParams.GetItem(Index: Integer): TParam;
begin
  Result:=(Inherited GetItem(Index)) as TParam;
end;

function TParams.GetParamValue(const ParamName: string): Variant;
begin
  Result:=ParamByName(ParamName).Value;
end;

procedure TParams.SetItem(Index: Integer; Value: TParam);
begin
  Inherited SetItem(Index,Value);
end;

procedure TParams.SetParamValue(const ParamName: string; const Value: Variant);
begin
  ParamByName(ParamName).Value:=Value;
end;

function TParams.CreateParseOpts(DoCreate, EscapeSlash, EscapeRepeat: Boolean): TSQLParseOptions;

  Procedure SetO(B : Boolean; O : TSQLParseOption);

  begin
    if B then Include(Result,O);
  end;

begin
  Result:=[];
  SetO(DoCreate,spoCreate);
  SetO(EscapeSlash,spoEscapeSlash);
  SetO(EscapeRepeat,spoEscapeRepeat);
end;

procedure TParams.AssignTo(Dest: TPersistent);
begin
 if (Dest is TParams) then
   TParams(Dest).Assign(Self)
 else
   inherited AssignTo(Dest);
end;

function TParams.GetDataSet: TDataSet;
begin
  If (FOwner is TDataset) Then
    Result:=TDataset(FOwner)
  else
    Result:=Nil;
end;

function TParams.GetOwner: TPersistent;
begin
  Result:=FOwner;
end;

class function TParams.ParamClass: TParamClass;
begin
  Result:=TParam;
end;

constructor TParams.Create(AOwner: TPersistent; AItemClass: TCollectionItemClass
  );
begin
  Inherited Create(AItemClass);
  FOwner:=AOwner;
end;


constructor TParams.Create(AOwner: TPersistent);
begin
  Create(AOwner,ParamClass);
end;

constructor TParams.Create;
begin
  Create(TPersistent(Nil));
end;

procedure TParams.AddParam(Value: TParam);
begin
  Value.Collection:=Self;
end;

procedure TParams.AssignValues(Value: TParams);

Var
  I : Integer;
  P,PS : TParam;

begin
  For I:=0 to Value.Count-1 do
    begin
    PS:=Value[i];
    P:=FindParam(PS.Name);
    If Assigned(P) then
      P.Assign(PS);
    end;
end;

function TParams.CreateParam(FldType: TFieldType; const ParamName: string;
  ParamType: TParamType): TParam;

begin
  Result:=Add as TParam;
  Result.Name:=ParamName;
  Result.DataType:=FldType;
  Result.ParamType:=ParamType;
end;

function TParams.FindParam(const Value: string): TParam;

Var
  I : Integer;

begin
  Result:=Nil;
  I:=Count-1;
  While (Result=Nil) and (I>=0) do
    If (CompareText(Value,Items[i].Name)=0) then
      Result:=Items[i]
    else
      Dec(i);
end;

procedure TParams.GetParamList(List: TList; const ParamNames: string);

Var
  P: TParam;
  N: String;
  StrPos: Integer;

begin
  if (ParamNames = '') or (List = nil) then
    Exit;
  StrPos := 1;
  repeat
    N := ExtractFieldName(ParamNames, StrPos);
    P := ParamByName(N);
    List.Add(P);
  until StrPos > Length(ParamNames);
end;

function TParams.IsEqual(Value: TParams): Boolean;

Var
  I : Integer;

begin
  Result:=(Value.Count=Count);
  I:=Count-1;
  While Result and (I>=0) do
    begin
    Result:=Items[I].IsEqual(Value[i]);
    Dec(I);
    end;
end;

function TParams.GetEnumerator: TParamsEnumerator;
begin
  Result:=TParamsEnumerator.Create(Self);
end;

function TParams.ParamByName(const Value: string): TParam;
begin
  Result:=FindParam(Value);
  If (Result=Nil) then
    DatabaseErrorFmt(SParameterNotFound,[Value],Dataset);
end;

function TParams.ParseSQL(SQL: String; DoCreate: Boolean): String;

var
  pb : TParamBinding;
  rs : string;
  PO : TSQLParseOptions;

begin
  PO:=CreateParseOpts(DoCreate,True,True);
  Result := DoParseSQL(SQL,PO,psInterbase, pb, ' ',rs);
end;

function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash,
  EscapeRepeat: Boolean; ParameterStyle: TParamStyle): String;

var pb : TParamBinding;
    rs : string;
    PO : TSQLParseOptions;

begin
  PO:=CreateParseOpts(DoCreate,EscapeSlash,EscapeRepeat);
  Result := DoParseSQL(SQL,PO,ParameterStyle,pb,' ',rs);
end;

function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash,
  EscapeRepeat: Boolean; ParameterStyle: TParamStyle; out
  ParamBinding: TParambinding): String;

var
  rs : string;
  PO : TSQLParseOptions;

begin
  PO:=CreateParseOpts(DoCreate,EscapeSlash, EscapeRepeat);
  Result := DoParseSQL(SQL,PO, ParameterStyle,ParamBinding,' ',rs);
end;

function SkipComments(var p: PChar; EscapeSlash, EscapeRepeat : Boolean) : Boolean;

begin
  Result := False;
  case p^ of
    '''', '"', '`':
      begin
        Result := True;
        // single quote, double quote or backtick delimited string
        SkipQuotesString(p, p^, EscapeSlash, EscapeRepeat);
      end;
    '-': // possible start of -- comment
      begin
        Inc(p);
        if p^='-' then // -- comment
        begin
          Result := True;
          repeat // skip until at end of line
            Inc(p);
          until p^ in [#10, #13, #0];
          while p^ in [#10, #13] do Inc(p); // newline is part of comment
        end;
      end;
    '/': // possible start of /* */ comment
      begin
        Inc(p);
        if p^='*' then // /* */ comment
        begin
          Result := True;
          Inc(p);
          while p^ <> #0 do
          begin
            if p^='*' then // possible end of comment
            begin
              Inc(p);
              if p^='/' then Break; // end of comment
            end
            else
              Inc(p);
          end;
          if p^='/' then Inc(p); // skip final /
        end;
      end;
  end; {case}
end;


function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash,
  EscapeRepeat: Boolean; ParameterStyle: TParamStyle; out
  ParamBinding: TParambinding; out ReplaceString: string): String;

var
  PO : TSQLParseOptions;

begin
  PO:=CreateParseOpts(DoCreate,EscapeSlash, EscapeRepeat);
  Result:=DoParseSQL(SQL,PO,ParameterStyle,ParamBinding,' ',ReplaceString);
end;

function TParams.ParseSQL(SQL: String; Options: TSQLParseOptions; ParameterStyle: TParamStyle; out ParamBinding: TParambinding;
  MacroChar: Char; out ReplaceString: string): String;
begin
  Result:=DoParseSQL(SQL,Options,ParameterStyle,ParamBinding,MacroChar,ReplaceString);
end;

function TParams.DoParseSQL(SQL: String; Options : TSQLParseOptions; ParameterStyle: TParamStyle; out
  ParamBinding: TParambinding; MacroChar : Char; out ReplaceString: string): String;

type
  // used for ParamPart
  TStringPart = record
    Start,Stop:integer;
  end;

const
  ParamAllocStepSize = 8;

var
  IgnorePart:boolean;
  p,ParamNameStart,BufStart:PChar;
  ParamName:string;
  QuestionMarkParamCount,ParameterIndex,NewLength:integer;
  ParamCount:integer; // actual number of parameters encountered so far;
                      // always <= Length(ParamPart) = Length(Parambinding)
                      // Parambinding will have length ParamCount in the end
  ParamPart:array of TStringPart; // describe which parts of buf are parameters
  NewQueryLength:integer;
  NewQuery:string;
  NewQueryIndex,BufIndex,CopyLen,i:integer;    // Parambinding will have length ParamCount in the end
  ParamDelim : Set of char;
  b:integer;
  tmpParam:TParam;

begin
  if spoCreate in Options then Clear;
  // Parse the SQL and build ParamBinding
  ParamCount:=0;
  NewQueryLength:=Length(SQL);
  SetLength(ParamPart,ParamAllocStepSize);
  SetLength(ParamBinding,ParamAllocStepSize);
  QuestionMarkParamCount:=0; // number of ? params found in query so far

  ReplaceString := '$';
  if ParameterStyle = psSimulated then
    while pos(ReplaceString,SQL) > 0 do ReplaceString := ReplaceString+'$';

  p:=PChar(SQL);
  BufStart:=p; // used to calculate ParamPart.Start values
  if spoUseMacro in options then
    ParamDelim:=[MacroChar]
  else
    ParamDelim:=[':','?'];
  repeat
    while SkipComments(p,spoEscapeSlash in Options ,spoEscapeRepeat in options) do ;
    if p^ in ParamDelim then // parameter
        begin
          IgnorePart := False;
          if (P^<>'?') then
          begin // find parameter name
            Inc(p);
            if p^ in [':','=',' '] then  // ignore ::, since some databases (postgres) uses this as a cast (wb 4813)
            begin
              IgnorePart := True;
              Inc(p);
            end
            else
            begin
              if p^='"' then // Check if the parameter-name is between quotes
                begin
                ParamNameStart:=p;
                SkipQuotesString(p,'"',spoEscapeSlash in Options,spoEscapeRepeat in Options);
                // Do not include the quotes in ParamName, but they must be included
                // when the parameter is replaced by some place-holder.
                ParamName:=Copy(ParamNameStart+1,1,p-ParamNameStart-2);
                end
              else
                begin
                ParamNameStart:=p;
                while not (p^ in (SQLDelimiterCharacters+[#0,'=','+','-','*','\','/','[',']','|','<','>'])) do
                  Inc(p);
                ParamName:=Copy(ParamNameStart,1,p-ParamNameStart);
                end;
            end;
          end
          else
          begin
            Inc(p);
            ParamNameStart:=p;
            ParamName:='';
          end;

          if not IgnorePart then
          begin
            Inc(ParamCount);
            if ParamCount>Length(ParamPart) then
            begin
              NewLength:=Length(ParamPart)+ParamAllocStepSize;
              SetLength(ParamPart,NewLength);
              SetLength(ParamBinding,NewLength);
            end;

            if spoCreate in Options then
              begin
              // Check if this is the first occurance of the parameter
              tmpParam := FindParam(ParamName);
              // If so, create the parameter and assign the Parameterindex
              if not assigned(tmpParam) then
                ParameterIndex := CreateParam(ftUnknown, ParamName, ptInput).Index
              else  // else only assign the ParameterIndex
                ParameterIndex := tmpParam.Index;
              end
            // else find ParameterIndex
            else
              begin
                if ParamName<>'' then
                  ParameterIndex:=ParamByName(ParamName).Index
                else
                begin
                  ParameterIndex:=QuestionMarkParamCount;
                  Inc(QuestionMarkParamCount);
                end;
              end;
            if ParameterStyle in [psPostgreSQL,psSimulated] then
              begin
              i:=ParameterIndex+1;
              repeat
                inc(NewQueryLength);
                i:=i div 10;
              until i=0;
              end;

            // store ParameterIndex in FParamIndex, ParamPart data
            ParamBinding[ParamCount-1]:=ParameterIndex;
            ParamPart[ParamCount-1].Start:=ParamNameStart-BufStart;
            ParamPart[ParamCount-1].Stop:=p-BufStart+1;

            // update NewQueryLength
            Dec(NewQueryLength,p-ParamNameStart);
          end;
        end
      else
        if P^ = #0 then
          Break// end of SQL
        else
          Inc(p);
  until false;

  SetLength(ParamPart,ParamCount);
  SetLength(ParamBinding,ParamCount);

  if ParamCount>0 then
  begin
    // replace :ParamName by ? for interbase and by $x for postgresql/psSimulated
    // (using ParamPart array and NewQueryLength)
    if (ParameterStyle = psSimulated) and (length(ReplaceString) > 1) then
      inc(NewQueryLength,(paramcount)*(length(ReplaceString)-1));

    SetLength(NewQuery,NewQueryLength);
    NewQueryIndex:=1;
    BufIndex:=1;
    for i:=0 to High(ParamPart) do
    begin
      CopyLen:=ParamPart[i].Start-BufIndex;
      System.Move(SQL[BufIndex],NewQuery[NewQueryIndex],CopyLen);
      Inc(NewQueryIndex,CopyLen);
      case ParameterStyle of
        psInterbase : begin
                        NewQuery[NewQueryIndex]:='?';
                        Inc(NewQueryIndex);
                      end;
        psPostgreSQL,
        psSimulated : begin
                        ParamName := IntToStr(ParamBinding[i]+1);
                        for b := 1 to length(ReplaceString) do
                          begin
                          NewQuery[NewQueryIndex]:='$';
                          Inc(NewQueryIndex);
                          end;
                        for b := 1 to length(ParamName) do
                          begin
                          NewQuery[NewQueryIndex]:=ParamName[b];
                          Inc(NewQueryIndex);
                          end;
                      end;
      end;
      BufIndex:=ParamPart[i].Stop;
    end;
    CopyLen:=Length(SQL)+1-BufIndex;
    if CopyLen > 0 then
      System.Move(SQL[BufIndex],NewQuery[NewQueryIndex],CopyLen);
  end
  else
    NewQuery:=SQL;
  Result := NewQuery;
end;


procedure TParams.RemoveParam(Value: TParam);
begin
   Value.Collection:=Nil;
end;

{ TParam }

Function TParam.GetDataSet: TDataSet;
begin
  If Assigned(Collection) and (Collection is TParams) then
    Result:=TParams(Collection).GetDataset
  else
    Result:=Nil;
end;

Function TParam.IsParamStored: Boolean;
begin
  Result:=Bound;
end;

Procedure TParam.AssignParam(Param: TParam);
begin
  if Not Assigned(Param) then
    begin
    Clear;
    FDataType:=ftunknown;
    FParamType:=ptUnknown;
    Name:='';
    Size:=0;
    Precision:=0;
    NumericScale:=0;
    end
  else
    begin
    FDataType:=Param.DataType;
    if Param.IsNull then
      Clear
    else
      FValue:=Param.FValue;
    FBound:=Param.Bound;
    Name:=Param.Name;
    if (ParamType=ptUnknown) then
      ParamType:=Param.ParamType;
    Size:=Param.Size;
    Precision:=Param.Precision;
    NumericScale:=Param.NumericScale;
    end;
end;

Procedure TParam.AssignTo(Dest: TPersistent);
begin
  if (Dest is TField) then
    AssignToField(TField(Dest))
  else
    inherited AssignTo(Dest);
end;

Function TParam.GetAsBoolean: Boolean;
begin
  If IsNull then
    Result:=False
  else
    Result:=FValue;
end;

Function TParam.GetAsBytes: TBytes;
begin
  if IsNull then
    Result:=nil
  else if VarIsArray(FValue) then
    Result:=FValue
  else if VarIsStr(FValue) then
    Result:=BytesOf(VarToStr(FValue))
  else
    // todo: conversion from other variant types to TBytes
    Result:=FValue;
end;

Function TParam.GetAsCurrency: Currency;
begin
  If IsNull then
    Result:=0.0
  else
    Result:=FValue;
end;

Function TParam.GetAsDateTime: TDateTime;
begin
  If IsNull then
    Result:=0.0
  else
    Result:=FValue;
end;

Function TParam.GetAsFloat: Double;
begin
  If IsNull then
    Result:=0.0
  else
    Result:=FValue;
end;

Function TParam.GetAsInteger: Longint;
begin
  If IsNull then
    Result:=0
  else
    Result:=FValue;
end;

Function TParam.GetAsLargeInt: LargeInt;
begin
  If IsNull then
    Result:=0
  else
    Result:=FValue;
end;

function TParam.GetAsLongWord: LongWord;
begin
  If IsNull then
    Result:=0
  else
    Result:=FValue;
end;

Function TParam.GetAsMemo: string;
begin
  If IsNull then
    Result:=''
  else
    Result:=FValue;
end;

Function TParam.GetAsString: string;
var P: Pointer;
begin
  If IsNull then
    Result:=''
  else if (FDataType in [ftBytes, ftVarBytes, ftBlob]) and VarIsArray(FValue) then
  begin
    SetLength(Result, (VarArrayHighBound(FValue, 1) + 1) div SizeOf(Char));
    P := VarArrayLock(FValue);
    try
      Move(P^, Result[1], Length(Result) * SizeOf(Char));
    finally
      VarArrayUnlock(FValue);
    end;
  end
  else
    Result:=FValue;
end;

Function TParam.GetAsAnsiString: AnsiString;
begin
  Result := GetAsString;
end;

Function TParam.GetAsUnicodeString: UnicodeString;
begin
  if IsNull then
    Result := ''
  else
    Result := FValue;
end;

Function TParam.GetAsUTF8String: UTF8String;
begin
  if IsNull then
    Result := ''
  else
    Result := FValue;
end;

Function TParam.GetAsWideString: WideString;
begin
  if IsNull then
    Result := ''
  else
    Result := FValue;
end;

Function TParam.GetAsVariant: Variant;
begin
  if IsNull then
    Result:=Null
  else
    Result:=FValue;
end;

Function TParam.GetAsFMTBCD: TBCD;
begin
  If IsNull then
    Result:=0
  else
    Result:=VarToBCD(FValue);
end;

Function TParam.GetDisplayName: string;
begin
  if (FName<>'') then
    Result:=FName
  else
    Result:=inherited GetDisplayName
end;

Function TParam.GetIsNull: Boolean;
begin
  Result:= VarIsNull(FValue) or VarIsClear(FValue);
end;

Function TParam.IsEqual(AValue: TParam): Boolean;
begin
  Result:=(Name=AValue.Name)
          and (IsNull=AValue.IsNull)
          and (Bound=AValue.Bound)
          and (DataType=AValue.DataType)
          and (ParamType=AValue.ParamType)
          and (VarType(FValue)=VarType(AValue.FValue))
          and (FValue=AValue.FValue);
end;

Procedure TParam.SetAsBCD(const AValue: Currency);
begin
  FDataType:=ftBCD;
  Value:=AValue;
end;

Procedure TParam.SetAsBlob(const AValue: TBlobData);
begin
  FDataType:=ftBlob;
  Value:=AValue;
end;

Procedure TParam.SetAsBoolean(AValue: Boolean);
begin
  FDataType:=ftBoolean;
  Value:=AValue;
end;

procedure TParam.SetAsByte(const AValue: LongInt);
begin
  FDataType:=ftByte;
  Value:=AValue;
end;

Procedure TParam.SetAsBytes(const AValue: TBytes);
begin
  FDataType:=ftVarBytes;
  Value:=AValue;
end;

Procedure TParam.SetAsCurrency(const AValue: Currency);
begin
  FDataType:=ftCurrency;
  Value:=AValue;
end;

Procedure TParam.SetAsDate(const AValue: TDateTime);
begin
  FDataType:=ftDate;
  Value:=AValue;
end;

Procedure TParam.SetAsDateTime(const AValue: TDateTime);
begin
  FDataType:=ftDateTime;
  Value:=AValue;
end;

Procedure TParam.SetAsFloat(const AValue: Double);
begin
  FDataType:=ftFloat;
  Value:=AValue;
end;

Procedure TParam.SetAsInteger(AValue: Longint);
begin
  FDataType:=ftInteger;
  Value:=AValue;
end;

Procedure TParam.SetAsLargeInt(AValue: LargeInt);
begin
  FDataType:=ftLargeint;
  Value:=AValue;
end;

Procedure TParam.SetAsLongWord(AValue: LongWord);
begin
  FDataType:=ftLongWord;
  Value:=AValue;
end;

Procedure TParam.SetAsMemo(const AValue: string);
begin
  FDataType:=ftMemo;
  Value:=AValue;
end;


Procedure TParam.SetAsSmallInt(AValue: LongInt);
begin
  FDataType:=ftSmallInt;
  Value:=AValue;
end;

Procedure TParam.SetAsString(const AValue: string);
begin
  if FDataType <> ftFixedChar then
    FDataType := ftString;
  Value:=AValue;
end;

Procedure TParam.SetAsAnsiString(const AValue: AnsiString);
begin
  if FDataType <> ftFixedChar then
    FDataType := ftString;
  Value:=AValue;
end;

Procedure TParam.SetAsUTF8String(const AValue: UTF8String);
begin
  if FDataType <> ftFixedChar then
    FDataType := ftString;
  Value:=AValue;
end;

Procedure TParam.SetAsUnicodeString(const AValue: UnicodeString);
begin
  if FDataType <> ftFixedWideChar then
    FDataType := ftWideString;
  Value := AValue;
end;

Procedure TParam.SetAsWideString(const AValue: WideString);
begin
  if FDataType <> ftFixedWideChar then
    FDataType := ftWideString;
  Value := AValue;
end;


Procedure TParam.SetAsTime(const AValue: TDateTime);
begin
  FDataType:=ftTime;
  Value:=AValue;
end;

Procedure TParam.SetAsVariant(const AValue: Variant);
begin
  FValue:=AValue;
  FBound:=not VarIsClear(AValue);
  if FDataType = ftUnknown then
    case VarType(Value) of
      varBoolean  : FDataType:=ftBoolean;
      varSmallint,
      varShortInt,
      varByte     : FDataType:=ftSmallInt;
      varWord,
      varInteger  : FDataType:=ftInteger;
      varCurrency : FDataType:=ftCurrency;
      varLongWord : FDataType:=ftLongWord;
      varSingle,
      varDouble   : FDataType:=ftFloat;
      varDate     : FDataType:=ftDateTime;
      varString,
      varOleStr   : if (FDataType<>ftFixedChar) then
                      FDataType:=ftString;
      varInt64    : FDataType:=ftLargeInt;
    else
      if VarIsFmtBCD(Value) then
        FDataType:=ftFmtBCD
      else if VarIsArray(AValue) and (VarType(AValue) and varTypeMask = varByte) then
        FDataType:=ftVarBytes
      else
        FDataType:=ftUnknown;
    end;
end;

Procedure TParam.SetAsWord(AValue: LongInt);
begin
  FDataType:=ftWord;
  Value:=AValue;
end;

Procedure TParam.SetAsFMTBCD(const AValue: TBCD);
begin
  FDataType:=ftFMTBcd;
  FValue:=VarFmtBCDCreate(AValue);
end;

Procedure TParam.SetDataType(AValue: TFieldType);

Var
  VT : Integer;

begin
  if FDataType=AValue then Exit;
  FDataType:=AValue;
  VT:=FieldTypetoVariantMap[AValue];
  If (VT=varError) then
    clear
  else
    if not VarIsEmpty(FValue) then
      begin
      Try
        FValue:=VarAsType(FValue,VT)
      except
        Clear;
      end { try }
      end;
end;

Procedure TParam.SetText(const AValue: string);
begin
  Value:=AValue;
end;

constructor TParam.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  ParamType:=ptUnknown;
  DataType:=ftUnknown;
  FValue:=Unassigned;
end;

constructor TParam.Create(AParams: TParams; AParamType: TParamType);
begin
  Create(AParams);
  ParamType:=AParamType;
end;

Procedure TParam.Assign(Source: TPersistent);
begin
  if (Source is TParam) then
    AssignParam(TParam(Source))
  else if (Source is TField) then
    AssignField(TField(Source))
  else if (Source is TStrings) then
    AsMemo:=TStrings(Source).Text
  else
    inherited Assign(Source);
end;

Procedure TParam.AssignField(Field: TField);
begin
  if Assigned(Field) then
    begin
    // Need TField.Value
    AssignFieldValue(Field,Field.Value);
    Name:=Field.FieldName;
    end
  else
    begin
    Clear;
    Name:='';
    end
end;

Procedure TParam.AssignToField(Field : TField);

begin
  if Assigned(Field) then
    case FDataType of
      ftUnknown  : DatabaseErrorFmt(SUnknownParamFieldType,[Name],DataSet);
      ftByte     : Field.AsInteger:=AsByte;
      // Need TField.AsSmallInt
      ftSmallint : Field.AsInteger:=AsSmallInt;
      // Need TField.AsWord
      ftWord     : Field.AsInteger:=AsWord;
      ftInteger,
      ftAutoInc  : Field.AsInteger:=AsInteger;
      ftLongWord : Field.AsLongWord:=AsLongword;
      ftLargeInt : Field.AsLargeInt:=AsLargeInt;
      ftCurrency : Field.AsCurrency:=AsCurrency;
      ftFloat    : Field.AsFloat:=AsFloat;
      ftBoolean  : Field.AsBoolean:=AsBoolean;
      ftBlob,
      ftGraphic..ftTypedBinary,
      ftOraBlob,
      ftOraClob,
      ftString,
      ftMemo,
      ftAdt,
      ftFixedChar: Field.AsString:=AsString;
      ftTime,
      ftDate,
      ftDateTime : Field.AsDateTime:=AsDateTime;
      ftBytes,
      ftVarBytes : Field.AsVariant:=Value;
      ftFmtBCD   : Field.AsBCD:=AsFMTBCD;
      ftFixedWideChar,
      ftWideString: Field.AsWideString:=AsWideString;
    else
      If not (DataType in [ftCursor, ftArray, ftDataset,ftReference]) then
        DatabaseErrorFmt(SBadParamFieldType, [Name], DataSet);
    end;
end;

Procedure TParam.AssignFromField(Field : TField);

begin
  if Assigned(Field) then
    begin
    FDataType:=Field.DataType;
    case Field.DataType of
      ftUnknown  : DatabaseErrorFmt(SUnknownParamFieldType,[Name],DataSet);
      ftByte     : AsByte:=Field.AsInteger;
      // Need TField.AsSmallInt
      ftSmallint : AsSmallint:=Field.AsInteger;
      // Need TField.AsWord
      ftWord     : AsWord:=Field.AsInteger;
      ftInteger,
      ftAutoInc  : AsInteger:=Field.AsInteger;
      ftLargeInt : AsLargeInt:=Field.AsLargeInt;
      ftLongWord : AsLongWord:=Field.AsLongWord;
      ftBCD,
      ftCurrency : AsCurrency:=Field.AsCurrency;
      ftFloat    : AsFloat:=Field.AsFloat;
      ftBoolean  : AsBoolean:=Field.AsBoolean;
      ftBlob,
      ftGraphic..ftTypedBinary,
      ftOraBlob,
      ftOraClob,
      ftString,
      ftMemo,
      ftAdt,
      ftFixedChar: AsString:=Field.AsString;
      ftTime,
      ftDate,
      ftDateTime : AsDateTime:=Field.AsDateTime;
      ftBytes,
      ftVarBytes : Value:=Field.AsVariant;
      ftFmtBCD   : AsFMTBCD:=Field.AsBCD;
      ftFixedWideChar,
      ftWideString: AsWideString:=Field.AsWideString;
    else
      If not (DataType in [ftCursor, ftArray, ftDataset,ftReference]) then
        DatabaseErrorFmt(SBadParamFieldType, [Name], DataSet);
    end;
    end;
end;

Procedure TParam.AssignFieldValue(Field: TField; const AValue: Variant);

begin
  If Assigned(Field) then
    begin

    if (Field.DataType = ftString) and TStringField(Field).FixedChar then
      FDataType := ftFixedChar
    else if (Field.DataType = ftMemo) and (Field.Size > 255) then
      FDataType := ftString
    else if (Field.DataType = ftWideString) and TWideStringField(Field).FixedChar then
      FDataType := ftFixedWideChar
    else if (Field.DataType = ftWideMemo) and (Field.Size > 255) then
      FDataType := ftWideString
    else
      FDataType := Field.DataType;

    if VarIsNull(AValue) then
      Clear
    else
      Value:=AValue;

    Size:=Field.DataSize;
    FBound:=True;

    end;
end;

Procedure TParam.Clear;
begin
  FValue:=UnAssigned;
end;

Procedure TParam.GetData(Buffer: Pointer);

Var
  P  : Pointer;
  S  : String;
  ws : WideString;
  l  : Integer;
begin
  case FDataType of
    ftUnknown  : DatabaseErrorFmt(SUnknownParamFieldType,[Name],DataSet);
    ftSmallint : PSmallint(Buffer)^:=AsSmallInt;
    ftWord     : PWord(Buffer)^:=AsWord;
    ftInteger,
    ftAutoInc  : PInteger(Buffer)^:=AsInteger;
    ftLargeInt : PInt64(Buffer)^:=AsLargeInt;
    ftLongWord : PLongWord(Buffer)^:=AsLongWord;
    ftCurrency : PDouble(Buffer)^:=AsCurrency;
    ftFloat    : PDouble(Buffer)^:=AsFloat;
    ftBoolean  : PWordBool(Buffer)^:=AsBoolean;
    ftString,
    ftMemo,
    ftAdt,
    ftFixedChar:
      begin
      S:=AsString;
      StrMove(PChar(Buffer),PChar(S),Length(S)+1);
      end;
    ftWideString,
    ftWideMemo: begin
      ws := GetAsWideString;
      l := Length(ws);
      if l > 0 then
        Move(ws[1], Buffer, Succ(l)*2)
      else
        PWideChar(Buffer)^ := #0
    end;
    ftTime     : PInteger(Buffer)^:=DateTimeToTimeStamp(AsTime).Time;
    ftDate     : PInteger(Buffer)^:=DateTimeToTimeStamp(AsTime).Date;
    ftDateTime : PDouble(Buffer)^:=TimeStampToMSecs(DateTimeToTimeStamp(AsDateTime));
    ftBlob,
    ftGraphic..ftTypedBinary,
    ftOraBlob,
    ftOraClob  :
      begin
      S:=GetAsString;
      Move(PChar(S)^, Buffer^, Length(S));
      end;
    ftBytes, ftVarBytes:
      begin
      if VarIsArray(FValue) then
        begin
        P:=VarArrayLock(FValue);
        try
          Move(P^, Buffer^, VarArrayHighBound(FValue, 1) + 1);
        finally
          VarArrayUnlock(FValue);
        end;
        end;
      end;
    ftFmtBCD   : PBCD(Buffer)^:=AsFMTBCD;
  else
    If not (DataType in [ftCursor, ftArray, ftDataset,ftReference]) then
      DatabaseErrorFmt(SBadParamFieldType, [Name], DataSet);
  end;
end;

Function TParam.GetDataSize: Integer;
begin
  Result:=0;
  case DataType of
    ftUnknown  : DatabaseErrorFmt(SUnknownParamFieldType,[Name],DataSet);
    ftBoolean  : Result:=SizeOf(WordBool);
    ftInteger,
    ftAutoInc  : Result:=SizeOf(Integer);
    ftSmallint : Result:=SizeOf(SmallInt);
    ftWord     : Result:=SizeOf(Word);
    ftLongWord : Result:=SizeOf(LongWord);
    ftLargeInt : Result:=SizeOf(Int64);
    ftTime,
    ftDate     : Result:=SizeOf(Integer);
    ftDateTime,
    ftCurrency,
    ftFloat    : Result:=SizeOf(Double);
    ftString,
    ftFixedChar,
    ftMemo,
    ftADT      : Result:=Length(AsString)+1;
    ftBytes,
    ftVarBytes : if VarIsArray(FValue) then
                   Result:=VarArrayHighBound(FValue,1)+1
                 else
                   Result:=0;
    ftBlob,
    ftGraphic..ftTypedBinary,
    ftOraClob,
    ftOraBlob  : Result:=Length(AsString);
    ftArray,
    ftDataSet,
    ftReference,
    ftCursor   : Result:=0;
    ftFmtBCD   : Result:=SizeOf(TBCD);
  else
    DatabaseErrorFmt(SBadParamFieldType,[Name],DataSet);
  end;

end;

Procedure TParam.LoadFromFile(const FileName: string; BlobType: TBlobType);

Var
  S : TFileStream;

begin
  S:=TFileStream.Create(FileName, fmOpenRead or fmShareDenyWrite);
  Try
    LoadFromStream(S,BlobType);
  Finally
    FreeAndNil(S);
  end;
end;

Procedure TParam.LoadFromStream(Stream: TStream; BlobType: TBlobType);

Var
  Temp : String;

begin
  FDataType:=BlobType;
  With Stream do
    begin
    Position:=0;
    SetLength(Temp,Size);
    ReadBuffer(Pointer(Temp)^,Size);
    end;
  Value:=Temp;
end;

Procedure TParam.SetBlobData(Buffer: Pointer; ASize: Integer);

Var
  Temp : TBlobData;

begin
  SetLength(Temp,ASize);
  Move(Buffer^,Temp[0],ASize);
  AsBlob:=Temp;
end;

Procedure TParam.SetData(Buffer: Pointer);

  Function FromTimeStamp(T,D : Integer) : TDateTime;

  Var TS : TTimeStamp;

  begin
    TS.Time:=T;
    TS.Date:=D;
    Result:=TimeStampToDateTime(TS);
  end;

begin
  case FDataType of
    ftUnknown  : DatabaseErrorFmt(SUnknownParamFieldType,[Name],DataSet);
    ftSmallint : AsSmallInt:=PSmallint(Buffer)^;
    ftWord     : AsWord:=PWord(Buffer)^;
    ftInteger,
    ftAutoInc  : AsInteger:=PInteger(Buffer)^;
    ftLargeInt : AsLargeInt:=PInt64(Buffer)^;
    ftLongWord : AsLongWord:=PLongWord(Buffer)^;
    ftCurrency : AsCurrency:= PDouble(Buffer)^;
    ftFloat    : AsFloat:=PDouble(Buffer)^;
    ftBoolean  : AsBoolean:=PWordBool(Buffer)^;
    ftString,
    ftFixedChar: AsString:=StrPas(Buffer);
    ftMemo     : AsMemo:=StrPas(Buffer);
    ftTime     : AsTime:=FromTimeStamp(PInteger(Buffer)^,DateDelta);
    ftDate     : Asdate:=FromTimeStamp(0,PInteger(Buffer)^);
    ftDateTime : AsDateTime:=TimeStampToDateTime(MSecsToTimeStamp(trunc(PDouble(Buffer)^)));
    ftCursor   : FValue:=0;
    ftBlob,
    ftGraphic..ftTypedBinary,
    ftOraBlob,
    ftOraClob  : SetBlobData(Buffer, StrLen(PChar(Buffer)));
    ftFmtBCD   : AsFMTBCD:=PBCD(Buffer)^;
  else
    DatabaseErrorFmt(SBadParamFieldType,[Name],DataSet);
  end;
end;


procedure TParams.CopyParamValuesFromDataset(ADataset: TDataset;
  CopyBound: Boolean);

Var
  I : Integer;
  P : TParam;
  F : TField;

begin
  If assigned(ADataSet) then
    For I:=0 to Count-1 do
     begin
     P:=Items[i];
     if CopyBound or (not P.Bound) then
       begin
       // Master dataset must be active and unbound parameters must have fields
       // with same names in master dataset (Delphi compatible behavior)
       F:=ADataSet.FieldByName(P.Name);
       P.AssignField(F);
       If Not CopyBound then
         P.Bound:=False;
       end;
    end;
end;
