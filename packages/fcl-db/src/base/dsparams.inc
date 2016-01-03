
<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ TParams }

Function TParams.GetItem(Index: Integer): TParam;
begin
  Result:=(Inherited GetItem(Index)) as TParam;
end;

Function TParams.GetParamValue(const ParamName: string): Variant;
begin
  Result:=ParamByName(ParamName).Value;
end;

Procedure TParams.SetItem(Index: Integer; Value: TParam);
begin
  Inherited SetItem(Index,Value);
end;

Procedure TParams.SetParamValue(const ParamName: string; const Value: Variant);
begin
  ParamByName(ParamName).Value:=Value;
end;

Procedure TParams.AssignTo(Dest: TPersistent);
begin
 if (Dest is TParams) then
   TParams(Dest).Assign(Self)
 else
   inherited AssignTo(Dest);
end;

Function TParams.GetDataSet: TDataSet;
begin
  If (FOwner is TDataset) Then
    Result:=TDataset(FOwner)
  else
    Result:=Nil;
end;

Function TParams.GetOwner: TPersistent;
begin
  Result:=FOwner;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Class Function TParams.ParamClass: TParamClass;
begin
  Result:=TParam;
end;

Constructor TParams.Create(AOwner: TPersistent; AItemClass: TCollectionItemClass
  );
begin
  Inherited Create(AItemClass);
  FOwner:=AOwner;
end;


Constructor TParams.Create(AOwner: TPersistent);
begin
  Create(AOwner,ParamClass);
end;

Constructor TParams.Create;
=======
=======
>>>>>>> origin/fixes_2_2

constructor TParams.Create(AOwner: TPersistent);
begin
  Inherited Create(TParam);
  Fowner:=AOwner;
end;

constructor TParams.Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Create(TPersistent(Nil));
end;

Procedure TParams.AddParam(Value: TParam);
begin
  Value.Collection:=Self;
end;

Procedure TParams.AssignValues(Value: TParams);

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

Function TParams.CreateParam(FldType: TFieldType; const ParamName: string;
  ParamType: TParamType): TParam;

begin
  Result:=Add as TParam;
<<<<<<< HEAD
<<<<<<< HEAD
  Result.Name:=ParamName;
  Result.DataType:=FldType;
  Result.ParamType:=ParamType;
=======
=======
>>>>>>> origin/fixes_2_2
  With Result do
    begin
    Name:=ParamName;
    DataType:=FldType;
    ParamType:=ParamType;
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Function TParams.FindParam(const Value: string): TParam;

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

Procedure TParams.GetParamList(List: TList; const ParamNames: string);

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
  Function NextName(Var S : String) : String;
  Var
    P : Integer;
  begin
    P:=Pos(';',S);
    If (P=0) then
      P:=Length(S)+1;
    Result:=Copy(S,1,P-1);
    system.Delete(S,1,P);
  end;

Var
  L,N : String;

begin
  L:=ParamNames;
  While (Length(L)>0) do
    begin
    N:=NextName(L);
    List.Add(ParamByName(N));
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Function TParams.IsEqual(Value: TParams): Boolean;

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

<<<<<<< HEAD
<<<<<<< HEAD
Function TParams.GetEnumerator: TParamsEnumerator;
begin
  Result:=TParamsEnumerator.Create(Self);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function TParams.ParamByName(const Value: string): TParam;
begin
  Result:=FindParam(Value);
  If (Result=Nil) then
    DatabaseErrorFmt(SParameterNotFound,[Value],Dataset);
end;

Function TParams.ParseSQL(SQL: String; DoCreate: Boolean): String;

var pb : TParamBinding;
    rs : string;

begin
  Result := ParseSQL(SQL,DoCreate,True,True,psInterbase, pb, rs);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash,
  EscapeRepeat: Boolean; ParameterStyle: TParamStyle): String;
=======
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat : Boolean; ParameterStyle : TParamStyle): String;
>>>>>>> graemeg/fixes_2_2
=======
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat : Boolean; ParameterStyle : TParamStyle): String;
>>>>>>> origin/fixes_2_2

var pb : TParamBinding;
    rs : string;

begin
  Result := ParseSQL(SQL,DoCreate,EscapeSlash,EscapeRepeat,ParameterStyle,pb, rs);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash,
  EscapeRepeat: Boolean; ParameterStyle: TParamStyle; out
  ParamBinding: TParambinding): String;
=======
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat : Boolean; ParameterStyle : TParamStyle; var ParamBinding: TParambinding): String;
>>>>>>> graemeg/fixes_2_2
=======
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat : Boolean; ParameterStyle : TParamStyle; var ParamBinding: TParambinding): String;
>>>>>>> origin/fixes_2_2

var rs : string;

begin
  Result := ParseSQL(SQL,DoCreate,EscapeSlash, EscapeRepeat, ParameterStyle,ParamBinding, rs);
end;

function SkipComments(var p: PChar; EscapeSlash, EscapeRepeat : Boolean) : Boolean;
<<<<<<< HEAD
<<<<<<< HEAD

begin
  Result := False;
  case p^ of
    '''', '"', '`':
      begin
        Result := True;
        // single quote, double quote or backtick delimited string
        SkipQuotesString(p, p^, EscapeSlash, EscapeRepeat);
      end;
=======
=======
>>>>>>> origin/fixes_2_2
var notRepeatEscaped : boolean;

  procedure SkipQuotesString(QuoteChar : char);
  begin
    Inc(p);
    Result := True;
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

begin
  result := false;
  case p^ of
    '''': SkipQuotesString('''');       // single quote delimited string
    '"':  SkipQuotesString('"');        // double quote delimited string
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    '-': // possible start of -- comment
      begin
        Inc(p);
        if p^='-' then // -- comment
        begin
          Result := True;
          repeat // skip until at end of line
            Inc(p);
<<<<<<< HEAD
<<<<<<< HEAD
          until p^ in [#10, #13, #0];
          while p^ in [#10, #13] do Inc(p); // newline is part of comment
        end;
=======
          until p^ in [#10, #0];
        end
>>>>>>> graemeg/fixes_2_2
=======
          until p^ in [#10, #0];
        end
>>>>>>> origin/fixes_2_2
      end;
    '/': // possible start of /* */ comment
      begin
        Inc(p);
        if p^='*' then // /* */ comment
        begin
          Result := True;
          repeat
            Inc(p);
            if p^='*' then // possible end of comment
            begin
              Inc(p);
              if p^='/' then Break; // end of comment
            end;
          until p^=#0;
          if p^='/' then Inc(p); // skip final /
        end;
      end;
  end; {case}
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash,
  EscapeRepeat: Boolean; ParameterStyle: TParamStyle; out
  ParamBinding: TParambinding; out ReplaceString: string): String;
=======
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat: Boolean; ParameterStyle : TParamStyle; var ParamBinding: TParambinding; var ReplaceString : string): String;
>>>>>>> graemeg/fixes_2_2
=======
Function TParams.ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat: Boolean; ParameterStyle : TParamStyle; var ParamBinding: TParambinding; var ReplaceString : string): String;
>>>>>>> origin/fixes_2_2

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
  b:integer;
  tmpParam:TParam;

begin
  if DoCreate then Clear;
  // Parse the SQL and build ParamBinding
  ParamCount:=0;
  NewQueryLength:=Length(SQL);
  SetLength(ParamPart,ParamAllocStepSize);
<<<<<<< HEAD
<<<<<<< HEAD
  SetLength(ParamBinding,ParamAllocStepSize);
=======
  SetLength(Parambinding,ParamAllocStepSize);
>>>>>>> graemeg/fixes_2_2
=======
  SetLength(Parambinding,ParamAllocStepSize);
>>>>>>> origin/fixes_2_2
  QuestionMarkParamCount:=0; // number of ? params found in query so far

  ReplaceString := '$';
  if ParameterStyle = psSimulated then
    while pos(ReplaceString,SQL) > 0 do ReplaceString := ReplaceString+'$';

  p:=PChar(SQL);
  BufStart:=p; // used to calculate ParamPart.Start values
  repeat
<<<<<<< HEAD
<<<<<<< HEAD
    while SkipComments(p,EscapeSlash,EscapeRepeat) do ;
=======
    SkipComments(p,EscapeSlash,EscapeRepeat);
>>>>>>> graemeg/fixes_2_2
=======
    SkipComments(p,EscapeSlash,EscapeRepeat);
>>>>>>> origin/fixes_2_2
    case p^ of
      ':','?': // parameter
        begin
          IgnorePart := False;
          if p^=':' then
          begin // find parameter name
            Inc(p);
<<<<<<< HEAD
<<<<<<< HEAD
            if p^ in [':','=',' '] then  // ignore ::, since some databases uses this as a cast (wb 4813)
=======
            if p^ in [':','='] then  // ignore ::, since some databases uses this as a cast (wb 4813)
>>>>>>> graemeg/fixes_2_2
=======
            if p^ in [':','='] then  // ignore ::, since some databases uses this as a cast (wb 4813)
>>>>>>> origin/fixes_2_2
            begin
              IgnorePart := True;
              Inc(p);
            end
            else
            begin
<<<<<<< HEAD
<<<<<<< HEAD
              if p^='"' then // Check if the parameter-name is between quotes
                begin
                ParamNameStart:=p;
                SkipQuotesString(p,'"',EscapeSlash,EscapeRepeat);
                // Do not include the quotes in ParamName, but they must be included
                // when the parameter is replaced by some place-holder.
                ParamName:=Copy(ParamNameStart+1,1,p-ParamNameStart-2);
                end
              else
                begin
                ParamNameStart:=p;
                while not (p^ in (SQLDelimiterCharacters+[#0,'=','+','-','*','\','/','[',']','|'])) do
                  Inc(p);
                ParamName:=Copy(ParamNameStart,1,p-ParamNameStart);
                end;
=======
=======
>>>>>>> origin/fixes_2_2
              ParamNameStart:=p;
              while not (p^ in (SQLDelimiterCharacters+[#0,'=','+','-','*','\','/','[',']','|'])) do
                Inc(p);
              ParamName:=Copy(ParamNameStart,1,p-ParamNameStart);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

            if DoCreate then
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
<<<<<<< HEAD
<<<<<<< HEAD
              i:=ParameterIndex+1;
              repeat
                inc(NewQueryLength);
                i:=i div 10;
              until i=0;
=======
=======
>>>>>>> origin/fixes_2_2
              if ParameterIndex > 8 then
                inc(NewQueryLength,2)
              else
                inc(NewQueryLength,1)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
              end;

            // store ParameterIndex in FParamIndex, ParamPart data
            ParamBinding[ParamCount-1]:=ParameterIndex;
            ParamPart[ParamCount-1].Start:=ParamNameStart-BufStart;
            ParamPart[ParamCount-1].Stop:=p-BufStart+1;

            // update NewQueryLength
            Dec(NewQueryLength,p-ParamNameStart);
          end;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
      #0:Break; // end of SQL
=======
      #0:Break;
>>>>>>> graemeg/fixes_2_2
=======
      #0:Break;
>>>>>>> origin/fixes_2_2
    else
      Inc(p);
    end;
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
      Move(SQL[BufIndex],NewQuery[NewQueryIndex],CopyLen);
      Inc(NewQueryIndex,CopyLen);
      case ParameterStyle of
<<<<<<< HEAD
<<<<<<< HEAD
        psInterbase : begin
                        NewQuery[NewQueryIndex]:='?';
                        Inc(NewQueryIndex);
                      end;
=======
        psInterbase : NewQuery[NewQueryIndex]:='?';
>>>>>>> graemeg/fixes_2_2
=======
        psInterbase : NewQuery[NewQueryIndex]:='?';
>>>>>>> origin/fixes_2_2
        psPostgreSQL,
        psSimulated : begin
                        ParamName := IntToStr(ParamBinding[i]+1);
                        for b := 1 to length(ReplaceString) do
                          begin
                          NewQuery[NewQueryIndex]:='$';
                          Inc(NewQueryIndex);
                          end;
<<<<<<< HEAD
<<<<<<< HEAD
                        for b := 1 to length(ParamName) do
                          begin
                          NewQuery[NewQueryIndex]:=ParamName[b];
                          Inc(NewQueryIndex);
                          end;
                      end;
      end;
=======
=======
>>>>>>> origin/fixes_2_2
                        NewQuery[NewQueryIndex]:= paramname[1];
                        if length(paramname)>1 then
                          begin
                          Inc(NewQueryIndex);
                          NewQuery[NewQueryIndex]:= paramname[2]
                          end;
                      end;
      end;
      Inc(NewQueryIndex);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      BufIndex:=ParamPart[i].Stop;
    end;
    CopyLen:=Length(SQL)+1-BufIndex;
    if CopyLen > 0 then
      Move(SQL[BufIndex],NewQuery[NewQueryIndex],CopyLen);
  end
  else
    NewQuery:=SQL;
<<<<<<< HEAD
<<<<<<< HEAD

=======
    
>>>>>>> graemeg/fixes_2_2
=======
    
>>>>>>> origin/fixes_2_2
  Result := NewQuery;
end;


Procedure TParams.RemoveParam(Value: TParam);
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

<<<<<<< HEAD
<<<<<<< HEAD
function TParam.GetAsBytes: TBytes;
begin
  if IsNull then
    Result:=nil
  else if VarIsArray(FValue) then
    Result:=FValue
  else
    // todo: conversion from other variant types to TBytes
    Result:=FValue;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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


Function TParam.GetAsMemo: string;
begin
  If IsNull then
    Result:=''
  else
    Result:=FValue;
end;

Function TParam.GetAsString: string;
<<<<<<< HEAD
<<<<<<< HEAD
var P: Pointer;
begin
  If IsNull then
    Result:=''
  else if (FDataType in [ftBytes, ftVarBytes]) and VarIsArray(FValue) then
  begin
    SetLength(Result, (VarArrayHighBound(FValue, 1) + 1) div SizeOf(Char));
    P := VarArrayLock(FValue);
    try
      Move(P^, Result[1], Length(Result) * SizeOf(Char));
    finally
      VarArrayUnlock(FValue);
    end;
  end
=======
begin
  If IsNull then
    Result:=''
>>>>>>> graemeg/fixes_2_2
=======
begin
  If IsNull then
    Result:=''
>>>>>>> origin/fixes_2_2
  else
    Result:=FValue;
end;

function TParam.GetAsWideString: WideString;
begin
  if IsNull then
    Result := ''
  else
    Result := FValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
Function TParam.GetAsVariant: Variant;
begin
  if IsNull then
    Result:=Null
  else
    Result:=FValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TParam.GetAsFMTBCD: TBCD;
begin
  If IsNull then
    Result:=0
  else
    Result:=VarToBCD(FValue);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
procedure TParam.SetAsBCD(const AValue: Currency);
begin
  FDataType:=ftBCD;
  Value:=AValue;
end;

Procedure TParam.SetAsBlob(const AValue: TBlobData);
begin
  FDataType:=ftBlob;
  Value:=AValue;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure TParam.SetAsBlob(const AValue: TBlobData);
begin
  FValue:=AValue;
  FDataType:=ftBlob;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsBoolean(AValue: Boolean);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftBoolean;
  Value:=AValue;
end;

procedure TParam.SetAsBytes(const AValue: TBytes);
begin
  FDataType:=ftVarBytes;
  Value:=AValue;
=======
  FValue:=AValue;
  FDataType:=ftBoolean;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  FDataType:=ftBoolean;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsCurrency(const AValue: Currency);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftCurrency;
  Value:=AValue;
=======
  FValue:=Avalue;
  FDataType:=ftCurrency;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=Avalue;
  FDataType:=ftCurrency;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsDate(const AValue: TDateTime);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftDate;
  Value:=AValue;
=======
  FValue:=Avalue;
  FDataType:=ftDate;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=Avalue;
  FDataType:=ftDate;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsDateTime(const AValue: TDateTime);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftDateTime;
  Value:=AValue;
=======
  FValue:=AValue;
  FDataType:=ftDateTime;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  FDataType:=ftDateTime;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsFloat(const AValue: Double);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftFloat;
  Value:=AValue;
=======
  FValue:=AValue;
  FDataType:=ftFloat;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  FDataType:=ftFloat;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsInteger(AValue: Longint);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftInteger;
  Value:=AValue;
=======
  FValue:=AValue;
  FDataType:=ftInteger;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  FDataType:=ftInteger;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsLargeInt(AValue: LargeInt);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftLargeint;
  Value:=AValue;
=======
  FValue:=AValue;
  FDataType:=ftLargeint;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  FDataType:=ftLargeint;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsMemo(const AValue: string);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftMemo;
  Value:=AValue;
=======
  FValue:=AValue;
  FDataType:=ftMemo;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  FDataType:=ftMemo;
>>>>>>> origin/fixes_2_2
end;


Procedure TParam.SetAsSmallInt(AValue: LongInt);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftSmallInt;
  Value:=AValue;
=======
  FValue:=AValue;
  FDataType:=ftSmallInt;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  FDataType:=ftSmallInt;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsString(const AValue: string);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FDataType <> ftFixedChar then
    FDataType := ftString;
  Value:=AValue;
=======
  FValue:=AValue;
  if FDataType <> ftFixedChar then
    FDataType := ftString;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  if FDataType <> ftFixedChar then
    FDataType := ftString;
>>>>>>> origin/fixes_2_2
end;

procedure TParam.SetAsWideString(const aValue: WideString);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FDataType <> ftFixedWideChar then
    FDataType := ftWideString;
  Value := aValue;
=======
  FValue := aValue;
  if FDataType <> ftFixedWideChar then
    FDataType := ftWideString;
>>>>>>> graemeg/fixes_2_2
=======
  FValue := aValue;
  if FDataType <> ftFixedWideChar then
    FDataType := ftWideString;
>>>>>>> origin/fixes_2_2
end;


Procedure TParam.SetAsTime(const AValue: TDateTime);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftTime;
  Value:=AValue;
=======
  FValue:=AValue;
  FDataType:=ftTime;
>>>>>>> graemeg/fixes_2_2
=======
  FValue:=AValue;
  FDataType:=ftTime;
>>>>>>> origin/fixes_2_2
end;

Procedure TParam.SetAsVariant(const AValue: Variant);
begin
  FValue:=AValue;
<<<<<<< HEAD
<<<<<<< HEAD
  FBound:=not VarIsClear(AValue);
=======
  FBound:=not VarIsClear(Value);
>>>>>>> graemeg/fixes_2_2
=======
  FBound:=not VarIsClear(Value);
>>>>>>> origin/fixes_2_2
  if FDataType = ftUnknown then
    case VarType(Value) of
      varBoolean  : FDataType:=ftBoolean;
      varSmallint,
      varShortInt,
      varByte     : FDataType:=ftSmallInt;
      varWord,
      varInteger  : FDataType:=ftInteger;
      varCurrency : FDataType:=ftCurrency;
      varLongWord,
      varSingle,
      varDouble   : FDataType:=ftFloat;
      varDate     : FDataType:=ftDateTime;
      varString,
      varOleStr   : if (FDataType<>ftFixedChar) then
                      FDataType:=ftString;
      varInt64    : FDataType:=ftLargeInt;
    else
<<<<<<< HEAD
<<<<<<< HEAD
      if VarIsFmtBCD(Value) then
        FDataType:=ftFmtBCD
      else if VarIsArray(AValue) and (VarType(AValue) and varTypeMask = varByte) then
        FDataType:=ftVarBytes
      else
        FDataType:=ftUnknown;
=======
      FDataType:=ftUnknown;
>>>>>>> graemeg/fixes_2_2
=======
      FDataType:=ftUnknown;
>>>>>>> origin/fixes_2_2
    end;
end;

Procedure TParam.SetAsWord(AValue: LongInt);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FDataType:=ftWord;
  Value:=AValue;
end;

procedure TParam.SetAsFMTBCD(const AValue: TBCD);
begin
  FDataType:=ftFMTBcd;
  FValue:=VarFmtBCDCreate(AValue);
end;
=======
=======
>>>>>>> origin/fixes_2_2
  FValue:=AValue;
  FDataType:=ftWord;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Procedure TParam.SetDataType(AValue: TFieldType);

Var
  VT : Integer;

begin
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
  else if (source is TStrings) then
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

procedure TParam.AssignToField(Field : TField);

begin
  if Assigned(Field) then
    case FDataType of
      ftUnknown  : DatabaseErrorFmt(SUnknownParamFieldType,[Name],DataSet);
      // Need TField.AsSmallInt
      ftSmallint : Field.AsInteger:=AsSmallInt;
      // Need TField.AsWord
      ftWord     : Field.AsInteger:=AsWord;
      ftInteger,
      ftAutoInc  : Field.AsInteger:=AsInteger;
<<<<<<< HEAD
<<<<<<< HEAD
      ftCurrency : Field.AsCurrency:=AsCurrency;
      ftFloat    : Field.AsFloat:=AsFloat;
=======
      // Need TField.AsCurrency
      ftCurrency : Field.asFloat:=AsCurrency;
      ftFloat    : Field.asFloat:=AsFloat;
>>>>>>> graemeg/fixes_2_2
=======
      // Need TField.AsCurrency
      ftCurrency : Field.asFloat:=AsCurrency;
      ftFloat    : Field.asFloat:=AsFloat;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
      ftVarBytes : Field.AsVariant:=Value;
      ftFmtBCD   : Field.AsBCD:=AsFMTBCD;
=======
      ftVarBytes : ; // Todo.
>>>>>>> graemeg/fixes_2_2
=======
      ftVarBytes : ; // Todo.
>>>>>>> origin/fixes_2_2
    else
      If not (DataType in [ftCursor, ftArray, ftDataset,ftReference]) then
        DatabaseErrorFmt(SBadParamFieldType, [Name], DataSet);
    end;
end;

procedure TParam.AssignFromField(Field : TField);

begin
  if Assigned(Field) then
    begin
    FDataType:=Field.DataType;
    case Field.DataType of
      ftUnknown  : DatabaseErrorFmt(SUnknownParamFieldType,[Name],DataSet);
      // Need TField.AsSmallInt
      ftSmallint : AsSmallint:=Field.AsInteger;
      // Need TField.AsWord
      ftWord     : AsWord:=Field.AsInteger;
      ftInteger,
      ftAutoInc  : AsInteger:=Field.AsInteger;
<<<<<<< HEAD
<<<<<<< HEAD
      ftBCD,
      ftCurrency : AsCurrency:=Field.AsCurrency;
      ftFloat    : AsFloat:=Field.AsFloat;
=======
      // Need TField.AsCurrency
      ftCurrency : AsCurrency:=Field.asCurrency;
      ftFloat    : AsFloat:=Field.asFloat;
>>>>>>> graemeg/fixes_2_2
=======
      // Need TField.AsCurrency
      ftCurrency : AsCurrency:=Field.asCurrency;
      ftFloat    : AsFloat:=Field.asFloat;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
      ftVarBytes : Value:=Field.AsVariant;
      ftFmtBCD   : AsFMTBCD:=Field.AsBCD;
=======
      ftVarBytes : ; // Todo.
>>>>>>> graemeg/fixes_2_2
=======
      ftVarBytes : ; // Todo.
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
      FDataType := ftFixedChar
    else if (Field.DataType = ftMemo) and (Field.Size > 255) then
      FDataType := ftString
    else if (Field.DataType = ftWideString) and TWideStringField(Field).FixedChar then
      FDataType := ftFixedWideChar
    else if (Field.DataType = ftWideMemo) and (Field.Size > 255) then
      FDataType := ftWideString
    else
      FDataType := Field.DataType;
=======
=======
>>>>>>> origin/fixes_2_2
      DataType := ftFixedChar
    else if (Field.DataType = ftMemo) and (Field.Size > 255) then
      DataType := ftString
    else if (Field.DataType = ftWideString) and TWideStringField(Field).FixedChar then
      DataType := ftFixedWideChar
    else if (Field.DataType = ftWideMemo) and (Field.Size > 255) then
      DataType := ftWideString
    else
      DataType := Field.DataType;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
    ftCurrency : PDouble(Buffer)^:=AsCurrency;
    ftFloat    : PDouble(Buffer)^:=AsFloat;
    ftBoolean  : PWordBool(Buffer)^:=AsBoolean;
    ftString,
    ftMemo,
    ftAdt,
    ftFixedChar:
      begin
      S:=AsString;
      StrMove(PChar(Buffer),Pchar(S),Length(S)+1);
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
<<<<<<< HEAD
<<<<<<< HEAD
    ftFmtBCD   : PBCD(Buffer)^:=AsFMTBCD;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    ftFmtBCD   : Result:=SizeOf(TBCD);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  else
    DatabaseErrorFmt(SBadParamFieldType,[Name],DataSet);
  end;


end;

Procedure TParam.LoadFromFile(const FileName: string; BlobType: TBlobType);

Var
  S : TFileStream;

begin
  S:=TFileStream.Create(FileName,fmOpenRead);
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
    FValue:=Temp;
    end;
end;

Procedure TParam.SetBlobData(Buffer: Pointer; ASize: Integer);

Var
  Temp : String;

begin
  SetLength(Temp,ASize);
  Move(Buffer^,Temp,ASize);
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
<<<<<<< HEAD
<<<<<<< HEAD
    ftFmtBCD   : AsFMTBCD:=PBCD(Buffer)^;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  else
    DatabaseErrorFmt(SBadParamFieldType,[Name],DataSet);
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TParams.CopyParamValuesFromDataset(ADataSet: TDataSet;
  CopyBound: Boolean);
=======
Procedure TParams.CopyParamValuesFromDataset(ADataset : TDataset; CopyBound : Boolean);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TParams.CopyParamValuesFromDataset(ADataset : TDataset; CopyBound : Boolean);
>>>>>>> origin/fixes_2_2

Var
  I : Integer;
  P : TParam;
  F : TField;
<<<<<<< HEAD
<<<<<<< HEAD

begin
  If assigned(ADataSet) then
=======
  
begin
  If (ADataSet<>Nil) then
>>>>>>> graemeg/fixes_2_2
=======
  
begin
  If (ADataSet<>Nil) then
>>>>>>> origin/fixes_2_2
    For I:=0 to Count-1 do
     begin
     P:=Items[i];
     if CopyBound or (not P.Bound) then
       begin
<<<<<<< HEAD
<<<<<<< HEAD
       // Master dataset must be active and unbound parameters must have fields
       // with same names in master dataset (Delphi compatible behavior)
       F:=ADataSet.FieldByName(P.Name);
=======
       F:=ADataset.FieldByName(P.Name);
>>>>>>> graemeg/fixes_2_2
=======
       F:=ADataset.FieldByName(P.Name);
>>>>>>> origin/fixes_2_2
       P.AssignField(F);
       If Not CopyBound then
         P.Bound:=False;
       end;
    end;
end;
