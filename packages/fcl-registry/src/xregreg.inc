
{ ---------------------------------------------------------------------
    System dependent Registry implementation - using XML file.
  ---------------------------------------------------------------------}

uses xmlreg;


Const
  XFileName = 'reg.xml';

Procedure TRegistry.SysRegCreate;
var s : string;
begin
  s:=includetrailingpathdelimiter(GetAppConfigDir(GlobalXMLFile));
  ForceDirectories(s);
  FSysData:=TXMLRegistry.Create(s+XFileName);
  TXmlRegistry(FSysData).AutoFlush:=False;
end;

Procedure TRegistry.SysRegFree;

begin
  if Assigned(FSysData) then
    TXMLRegistry(FSysData).Flush;
  TXMLRegistry(FSysData).Free;
end;

function TRegistry.SysCreateKey(const Key: String): Boolean;

begin
  Result:=TXmlRegistry(FSysData).CreateKey(Key);
end;

function TRegistry.DeleteKey(const Key: String): Boolean;

begin
  Result:=TXMLRegistry(FSysData).DeleteKey(Key);
end;

function TRegistry.DeleteValue(const Name: String): Boolean;
begin
  Result:=TXmlRegistry(FSysData).DeleteValue(Name);
end;

function TRegistry.SysGetData(const Name: String; Buffer: Pointer;
          BufSize: Integer; var RegData: TRegDataType): Integer;

Var
  DataType : TDataType;
begin
  Result:=BufSize;
  If TXmlregistry(FSysData).GetValueData(Name,DataType,Buffer^,Result) then
    begin
    Case DataType of
      dtUnknown : RegData:=rdUnknown;
      dtString  : RegData:=rdString;
      dtDWord   : RegData:=rdInteger;
      dtBinary  : RegData:=rdBinary;
    end;
    end
  else
    Result:=-1;
end;


function TRegistry.GetDataInfo(const ValueName: String; var Value: TRegDataInfo): Boolean;

Var
  Info : TDataInfo;

begin
  Result := TXmlRegistry(FSysData).GetValueInfo(ValueName,Info);
  If Not Result then
    With Value do
      begin
      RegData:=rdunknown;
      DataSize:=0;
      end
  else
    With Value do
      begin
      Case Info.DataType of
        dtUnknown: RegData:=rdUnknown;
        dtDword  : Regdata:=rdInteger;
        dtString : RegData:=rdString;
        dtBinary : RegData:=rdBinary;
      end;
      DataSize:=Info.DataSize;
      end;
end;

function TRegistry.GetKey(const Key: String): HKEY;
begin
  Result := 0;
end;

function TRegistry.GetKeyInfo(var Value: TRegKeyInfo): Boolean;

Var
  Info : TKeyInfo;

begin
  Result:=TXmlRegistry(FSysData).GetKeyInfo(info);
  If Result then
    With Value,Info do
      begin
      NumSubKeys:=SubKeys;
      MaxSubKeyLen:=SubKeyLen;
      NumValues:= Values;
      MaxValueLen:=ValueLen;
      MaxDataLen:=DataLen;
      FileTime:=FTime;
      end;
end;

function TRegistry.KeyExists(const Key: string): Boolean;
begin
  Result:=TXmlRegistry(FSysData).KeyExists(Key);
end;

function TRegistry.LoadKey(const Key, FileName: string): Boolean;
begin
  Result := False;
end;

function TRegistry.OpenKey(const Key: string; CanCreate: Boolean): Boolean;

begin
  Result:=TXmlRegistry(FSysData).SetKey(Key,CanCreate);
  FCurrentKey:=1;
end;

function TRegistry.OpenKeyReadOnly(const Key: string): Boolean;

begin
  Result:=TXmlRegistry(FSysData).SetKey(Key,False);
end;

function TRegistry.RegistryConnect(const UNCName: string): Boolean;
begin
  Result := True;
end;

function TRegistry.ReplaceKey(const Key, FileName, BackUpFileName: string): Boolean;
begin
  Result := False;
end;

function TRegistry.RestoreKey(const Key, FileName: string): Boolean;
begin
  Result := False;
end;

function TRegistry.SaveKey(const Key, FileName: string): Boolean;
begin
  Result := False;
end;

function TRegistry.UnLoadKey(const Key: string): Boolean;
begin
  Result := False;
end;

function TRegistry.ValueExists(const Name: string): Boolean;
begin
  Result := TXmlRegistry(FSysData).ValueExists(Name);
end;

procedure TRegistry.ChangeKey(Value: HKey; const Path: String);
begin

end;

procedure TRegistry.GetKeyNames(Strings: TStrings);
begin
  TXmlRegistry(FSysData).EnumSubKeys(Strings);
end;

procedure TRegistry.GetValueNames(Strings: TStrings);
begin
  TXmlRegistry(FSysData).EnumValues(Strings);
end;


Function TRegistry.SysPutData(const Name: string; Buffer: Pointer;
  BufSize: Integer; RegData: TRegDataType) : Boolean;

Var
  DataType : TDataType;

begin
  Case RegData of
    rdUnknown               : DataType := dtUnknown;
    rdString,rdExpandString : DataType := dtString;
    rdInteger               : DataType := dtDword;
    rdBinary                : DataType := dtBinary;
  end;
  Result:=TXMLRegistry(FSysData).SetValueData(Name,DataType,Buffer^,BufSize);
end;

procedure TRegistry.RenameValue(const OldName, NewName: string);
begin
  TXMLRegistry(FSysData).RenameValue(OldName,NewName);
end;

procedure TRegistry.SetCurrentKey(Value: HKEY);
begin
  fCurrentKey := Value;
end;

procedure TRegistry.SetRootKey(Value: HKEY);

Var
  S: String;

begin
  If (Value=HKEY_CLASSES_ROOT) then
    S:='HKEY_CLASSES_ROOT'
  else if (Value=HKEY_CURRENT_USER) then
    S:='HKEY_CURRENT_USER'
  else if (Value=HKEY_LOCAL_MACHINE) then
    S:='HKEY_LOCAL_MACHINE'
  else if (Value=HKEY_USERS) then
    S:='HKEY_USERS'
  else if Value=HKEY_PERFORMANCE_DATA then
    S:='HKEY_PERFORMANCE_DATA'
  else if (Value=HKEY_CURRENT_CONFIG) then
    S:='HKEY_CURRENT_CONFIG'
  else if (Value=HKEY_DYN_DATA) then
    S:='HKEY_DYN_DATA'
  else
    S:=Format('Key%d',[Value]);
  TXmlRegistry(FSysData).SetRootKey(S);
  fRootKey := Value;
end;

procedure TRegistry.CloseKey;

begin
  // CloseKey is called from destructor, which includes cases of failed construction.
  // FSysData may be unassigned at this point.
  if Assigned(FSysData) then
  begin
    TXMLRegistry(FSysData).Flush;
    TXMLRegistry(FSysData).SetRootKey(TXMLRegistry(FSysData).RootKey);
  end;
end;

procedure TRegistry.CloseKey(key:HKEY);

begin
  if Assigned(FSysData) then
  begin
    TXMLRegistry(FSysData).Flush;
    TXMLRegistry(FSysData).SetRootKey(TXMLRegistry(FSysData).RootKey);
  end;
end;

procedure TRegistry.CloseKey(key:HKEY);

begin
  TXMLRegistry(FSysData).Flush;
  TXMLRegistry(FSysData).SetRootKey(TXMLRegistry(FSysData).RootKey);
end;

procedure TRegistry.CloseKey(key:HKEY);

begin
  TXMLRegistry(FSysData).Flush;
  TXMLRegistry(FSysData).SetRootKey(TXMLRegistry(FSysData).RootKey);
end;
