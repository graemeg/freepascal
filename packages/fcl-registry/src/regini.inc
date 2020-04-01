
{******************************************************************************
                                TRegIniFile
 ******************************************************************************}

constructor TRegIniFile.Create(const FN: String);
begin
  Create(FN, KEY_ALL_ACCESS);
end;

constructor TRegIniFile.Create(const FN: String;aaccess:longword);
begin
  inherited Create(aaccess);
  fFileName := FN;
  if fFileName<>'' then begin
    fPath := fFileName + '\';
    if fPath[1]='\' then
      System.Delete(fPath,1,1);
    OpenKey(fFileName, aaccess <> KEY_READ);
  end
  else
    fPath := '';
  fPreferStringValues:=True; // Delphi compatibility
end;

procedure TRegIniFile.DeleteKey(const Section, Ident: String);
begin
  if OpenSection(Section) then
  try
    DeleteValue(Ident);
  finally
    CloseSection;
  end;
end;

procedure TRegIniFile.EraseSection(const Section: string);
begin
  inherited DeleteKey(Section);
end;

procedure TRegIniFile.ReadSection(const Section: string; Strings: TStrings);
begin
  if OpenSection(Section) then
  try
    GetValueNames(Strings);
  finally
    CloseSection;
  end;
end;

procedure TRegIniFile.ReadSections(Strings: TStrings);
begin
  GetKeyNames(Strings);
end;

procedure TRegIniFile.ReadSectionValues(const Section: string; Strings: TStrings);
var
 ValList : TStringList;
 V : String;
 i : Integer;
begin
  if OpenSection(Section) then
  try
 	  ValList := TStringList.Create;
 	  try
      GetValueNames(ValList);
      for i:=0 to ValList.Count-1 do
      begin
        V := inherited ReadString(ValList.Strings[i]);
        Strings.Add(ValList.Strings[i] + '=' + V);
      end;
 	  finally
      ValList.Free;
 	  end;
  finally
    CloseSection;
  end;
end;

procedure TRegIniFile.WriteBool(const Section, Ident: string; Value: Boolean);
begin
  if OpenSection(Section,True) then
	try
    if not fPreferStringValues then
  	  inherited WriteBool(Ident,Value)
    else begin
      if ValueExists(Ident) and (GetDataType(Ident)=rdInteger) then
    	  inherited WriteBool(Ident,Value)
      else
        inherited WriteString(Ident,BoolToStr(Value));
    end;
  finally
    CloseSection;
	end;
end;

procedure TRegIniFile.WriteInteger(const Section, Ident: string; Value: LongInt);
begin
  if OpenSection(Section,True) then
  try
    if not fPreferStringValues then
      inherited WriteInteger(Ident,Value)
    else begin
      if ValueExists(Ident) and (GetDataType(Ident)=rdInteger) then
    	  inherited WriteInteger(Ident,Value)
      else
        inherited WriteString(Ident,IntToStr(Value));
    end;
  finally
    CloseSection;
  end;
end;

procedure TRegIniFile.WriteString(const Section, Ident, Value: String);
begin
  if OpenSection(Section,True) then
  try
    inherited WriteString(Ident,Value);
  finally
    CloseSection;
  end;
end;

procedure TRegIniFile.WriteDate(const Section, Ident: string; Value: TDateTime);

begin
  if OpenSection(Section,true) then 
    try
      if not fPreferStringValues then
        inherited WriteDate(Ident,Value)
      else if ValueExists(Ident) and (GetDataType(Ident)<>rdString) then
	    inherited WriteDate(Ident,Value)
      else
        inherited WriteString(Ident,DateToStr(Value));
  finally
    CloseKey;
  end;
end;

procedure TRegIniFile.WriteDateTime(const Section, Ident: string; Value: TDateTime);

begin
  if OpenSection(Section,true) then 
    try
      if not fPreferStringValues then
        inherited WriteDateTime(Ident,Value)
      else if ValueExists(Ident) and (GetDataType(Ident)<>rdString) then
	    inherited WriteDateTime(Ident,Value)
      else
        inherited WriteString(Ident,DateTimeToStr(Value));
  finally
    CloseKey;
  end;
end;

procedure TRegIniFile.WriteTime(const Section, Ident: string; Value: TDateTime);

begin
  if OpenSection(Section,true) then 
    try
      if not fPreferStringValues then
        inherited WriteTime(Ident,Value)
      else if ValueExists(Ident) and (GetDataType(Ident)<>rdString) then
	    inherited WriteTime(Ident,Value)
      else
        inherited WriteString(Ident,TimeToStr(Value));
  finally
    CloseKey;
  end;
end;

procedure TRegIniFile.WriteFloat(const Section, Ident: string; Value: Double);

begin
  if OpenSection(Section,true) then 
    try
      if not fPreferStringValues then
        inherited WriteFloat(Ident,Value)
      else if ValueExists(Ident) and (GetDataType(Ident)<>rdString) then
	    inherited WriteFloat(Ident,Value)
      else
        inherited WriteString(Ident,FloatToStr(Value));
  finally
    CloseKey;
  end;
end;

    

function TRegIniFile.ReadBool(const Section, Ident: string; Default: Boolean): Boolean;
begin
	Result := Default;
  if OpenSection(Section) then
	try
    if ValueExists(Ident) then
      if (not fPreferStringValues) or (GetDataType(Ident)=rdInteger) then
  	    Result := inherited ReadBool(Ident)
      else
        Result := StrToBool(inherited ReadString(Ident));
	finally
    CloseSection;
	end;
end;

function TRegIniFile.ReadInteger(const Section, Ident: string; Default: LongInt): LongInt;
begin
  Result := Default;
  if OpenSection(Section) then
  try
    if ValueExists(Ident) then
      if (not fPreferStringValues) or (GetDataType(Ident)=rdInteger) then
        Result := inherited ReadInteger(Ident)
      else
        Result := StrToInt(inherited ReadString(Ident));
  finally
    CloseSection;
  end;
end;

function TRegIniFile.ReadString(const Section, Ident, Default: String): String;
begin
  Result := Default;
  if OpenSection(Section) then
  try
    if ValueExists(Ident) then
      Result := inherited ReadString(Ident);
  finally
    CloseSection;
  end;
end;

function TRegIniFile.ReadDate(const Section, Ident: string; Default: TDateTime):TDateTime;
begin
  Result := Default;
  if OpenSection(Section) then
  try
    if ValueExists(Ident) then
      if (not fPreferStringValues) or (GetDataType(Ident)<>rdString) then
        Result := inherited ReadDate(Ident)
      else
        Result := StrToDateDef(inherited ReadString(Ident),Result);
  finally
    CloseSection;
  end;
end;

function TRegIniFile.ReadDateTime(const Section, Ident: string; Default: TDateTime):TDateTime;
begin
  Result := Default;
  if OpenSection(Section) then
  try
    if ValueExists(Ident) then
      if (not fPreferStringValues) or (GetDataType(Ident)<>rdString) then
        Result := inherited ReadDateTime(Ident)
      else
        Result := StrToDateTimeDef(inherited ReadString(Ident),Result);
  finally
    CloseSection;
  end;
end;
 
function TRegIniFile.ReadTime(const Section, Ident: string; Default: TDateTime):TDateTime;

begin
  Result := Default;
  if OpenSection(Section) then
  try
    if ValueExists(Ident) then
      if (not fPreferStringValues) or (GetDataType(Ident)<>rdString) then
        Result := inherited ReadTime(Ident)
      else
        Result := StrToTimeDef(inherited ReadString(Ident),Result);
  finally
    CloseSection;
  end;
end;

function TRegIniFile.ReadFloat(const Section, Ident: string; Default: Double): Double;

begin
  Result := Default;
  if OpenSection(Section) then
  try
    if ValueExists(Ident) then
      if (not fPreferStringValues) or (GetDataType(Ident)<>rdString) then
        Result := inherited ReadFloat(Ident)
      else
        Result := StrToFloatDef(inherited ReadString(Ident),Result);
  finally
    CloseSection;
  end;
end;

function TRegIniFile.OpenSection(const Section: string; CreateSection : Boolean = false): boolean;
var
  k: HKEY;
  S : String;

begin
  S:=Section;
  If (S<>'') and (S[1] = '\') then
    Delete(S,1,1);
  if CreateSection and (S<>'') then
    CreateKey('\'+CurrentPath+'\'+S);
  if S <> '' then
    k:=GetKey('\'+CurrentPath+'\'+S)
  else
    k:=GetKey('\'+CurrentPath);
  if k = 0 then
    begin
    Result:=False;
    exit;
    end;
  SetCurrentKey(k);
  Result:=True;
end;

procedure TRegIniFile.CloseSection;
begin
  CloseKey(CurrentKey);
end;

