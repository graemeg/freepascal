type
  TWinRegData = record
    RootKeyOwned: Boolean;
  end;
  PWinRegData = ^TWinRegData;

{******************************************************************************
                                  TRegistry
 ******************************************************************************}

Procedure TRegistry.SysRegCreate;
begin
  FStringSizeIncludesNull:=True;
  New(PWinRegData(FSysData));
  PWinRegData(FSysData)^.RootKeyOwned := False;
end;

Procedure TRegistry.SysRegfree;

begin
  if PWinRegData(FSysData)^.RootKeyOwned and (RootKey <> 0) then
    RegCloseKey(RootKey);
  Dispose(PWinRegData(FSysData));
end;

Function PrepKey(Const S : String) : pChar;

begin
  Result:=PChar(S);
  If Result^='\' then
    Inc(Result);
end;

Function RelativeKey(Const S : String) : Boolean;

begin
  Result:=(S='') or (S[1]<>'\')
end;


function TRegistry.sysCreateKey(const Key: String): Boolean;
Var
  P: PChar;
  Disposition: Dword;
  Handle: HKEY;
  SecurityAttributes: Pointer; //LPSECURITY_ATTRIBUTES;

begin
  SecurityAttributes := Nil;
  P:=PrepKey(Key);
  Result:=RegCreateKeyExA(GetBaseKey(RelativeKey(Key)),
                         P,
                         0,
                         '',
                         REG_OPTION_NON_VOLATILE,
                         KEY_ALL_ACCESS,
                         SecurityAttributes,
                         Handle,
                         @Disposition) = ERROR_SUCCESS;
  RegCloseKey(Handle);
end;

function TRegistry.DeleteKey(const Key: String): Boolean;

Var
  P: PChar;
begin
  P:=PRepKey(Key);
  Result:=RegDeleteKeyA(GetBaseKey(RelativeKey(Key)),P)=ERROR_SUCCESS;
end;

function TRegistry.DeleteValue(const Name: String): Boolean;
begin
  Result := RegDeleteValueA(fCurrentKey, @Name[1]) = ERROR_SUCCESS;
end;

function TRegistry.SysGetData(const Name: String; Buffer: Pointer;
          BufSize: Integer; var RegData: TRegDataType): Integer;
Var
  P: PChar;
  RD : DWord;

begin
  P := PChar(Name);
  If RegQueryValueExA(fCurrentKey,P,Nil,
                      @RD,Buffer,lpdword(@BufSize))<>ERROR_SUCCESS Then
    Result:=-1
  else
    begin
    If (RD=REG_SZ) then
      RegData:=rdString
    else if (RD=REG_EXPAND_SZ) then
      Regdata:=rdExpandString
    else if (RD=REG_DWORD) then
      RegData:=rdInteger
    else if (RD=REG_BINARY) then
      RegData:=rdBinary
    else
      RegData:=rdUnknown;
    Result:=BufSize;
    end;
end;

function TRegistry.GetDataInfo(const ValueName: String; var Value: TRegDataInfo): Boolean;

Var
  P: PChar;

begin
  P:=PChar(ValueName);
  With Value do
    Result:=RegQueryValueExA(fCurrentKey,P,Nil,lpdword(@RegData),Nil,lpdword(@DataSize))=ERROR_SUCCESS;
  If Not Result Then
    begin
    Value.RegData := rdUnknown;
    Value.DataSize := 0
    end
end;


function TRegistry.GetKey(const Key: String): HKEY;
var
  S : string;
  Rel : Boolean;
begin
  Result:=0;
  S:=Key;
  Rel:=RelativeKey(S);
  if not(Rel) then
    Delete(S,1,1);
{$ifdef WinCE}
  RegOpenKeyEx(GetBaseKey(Rel),PWideChar(WideString(S)),0,FAccess,Result);
{$else WinCE}
  RegOpenKeyEx(GetBaseKey(Rel),PChar(S),0,FAccess,Result);
{$endif WinCE}
end;


function TRegistry.GetKeyInfo(var Value: TRegKeyInfo): Boolean;
var
  winFileTime: Windows.FILETIME;
  sysTime: TSystemTime;
begin
  FillChar(Value, SizeOf(Value), 0);
  With Value do
    Result:=RegQueryInfoKeyA(CurrentKey,nil,nil,nil,lpdword(@NumSubKeys),
              lpdword(@MaxSubKeyLen),nil,lpdword(@NumValues),lpdword(@MaxValueLen),
              lpdword(@MaxDataLen),nil,@winFileTime)=ERROR_SUCCESS;
  if Result then
  begin
    FileTimeToSystemTime(@winFileTime, @sysTime);
    Value.FileTime := SystemTimeToDateTime(sysTime);
  end;
end;


function TRegistry.KeyExists(const Key: string): Boolean;
var
  KeyHandle : HKEY;
  OldAccess : LONG;
begin
  Result:=false;
  OldAccess:=FAccess;
  try
<<<<<<< HEAD
<<<<<<< HEAD
    FAccess:=KEY_QUERY_VALUE or KEY_ENUMERATE_SUB_KEYS or STANDARD_RIGHTS_READ 
    {$ifndef WinCE} or (OldAccess and (KEY_WOW64_64KEY or KEY_WOW64_32KEY)) {$endif};
=======
    FAccess:=KEY_QUERY_VALUE or KEY_ENUMERATE_SUB_KEYS or STANDARD_RIGHTS_READ;
>>>>>>> graemeg/fixes_2_2
=======
    FAccess:=KEY_QUERY_VALUE or KEY_ENUMERATE_SUB_KEYS or STANDARD_RIGHTS_READ;
>>>>>>> origin/fixes_2_2
    KeyHandle:=GetKey(Key);
    if KeyHandle<>0 then
      begin
        RegCloseKey(KeyHandle);
        Result:=true;
      end;
  finally
    FAccess:=OldAccess;
  end;
end;


function TRegistry.LoadKey(const Key, FileName: string): Boolean;
begin
  Result := False;
end;


function TRegistry.OpenKey(const Key: string; CanCreate: Boolean): Boolean;

Var
  P: PChar;
  Handle: HKEY;
  Disposition: Integer;
  SecurityAttributes: Pointer; //LPSECURITY_ATTRIBUTES;
  S: string;
begin
  SecurityAttributes := Nil;
  P:=PrepKey(Key);
  If CanCreate then
    begin
    Handle:=0;
    Result:=RegCreateKeyExA(GetBaseKey(RelativeKey(Key)),P,0,'',

                           REG_OPTION_NON_VOLATILE,
                           fAccess,SecurityAttributes,Handle,
                           pdword(@Disposition))=ERROR_SUCCESS

    end
  else
    Result:=RegOpenKeyExA(GetBaseKey(RelativeKey(Key)),
                         P,0,fAccess,Handle)=ERROR_SUCCESS;
  If Result then begin
    if RelativeKey(Key) then
      S:=CurrentPath + Key
    else
      S:=P;
    ChangeKey(Handle, S);
  end;
end;

function TRegistry.OpenKeyReadOnly(const Key: string): Boolean;

Var
  OldAccess: LongWord;
begin
  OldAccess:=fAccess;
  fAccess:=KEY_READ {$ifndef WinCE} or (OldAccess and (KEY_WOW64_64KEY or KEY_WOW64_32KEY)) {$endif};
  try
    Result:=OpenKey(Key, False);
  finally
    fAccess:=OldAccess;
  end;
end;

function TRegistry.RegistryConnect(const UNCName: string): Boolean;
{$ifndef WinCE}
var
  newroot: HKEY;
{$endif}
begin
{$ifdef WinCE}
  Result:=False;
{$else}
  Result:=RegConnectRegistryA(PChar(UNCName),RootKey,newroot)=ERROR_SUCCESS;
  if Result then begin
    RootKey:=newroot;
    PWinRegData(FSysData)^.RootKeyOwned:=True;
  end;
{$endif}
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
  Result := false;
end;

function TRegistry.ValueExists(const Name: string): Boolean;

var
  Info : TRegDataInfo;

begin
  Result:=GetDataInfo(Name,Info);
end;

procedure TRegistry.CloseKey;
begin
  If (CurrentKey<>0) then
    begin
    if LazyWrite then
      RegCloseKey(CurrentKey)
    else
      RegFlushKey(CurrentKey);
    fCurrentKey:=0;
    end;
  fCurrentPath:='';
end;

procedure TRegistry.CloseKey(key:HKEY);
begin
  RegCloseKey(key);
end;

procedure TRegistry.CloseKey(key:HKEY);
begin
  RegCloseKey(CurrentKey)
end;

procedure TRegistry.CloseKey(key:HKEY);
begin
  RegCloseKey(CurrentKey)
end;

procedure TRegistry.ChangeKey(Value: HKey; const Path: String);
begin
  CloseKey;
  FCurrentKey:=Value;
  FCurrentPath:=Path;
end;

procedure TRegistry.GetKeyNames(Strings: TStrings);

Var
  L : Cardinal;
  I: Integer;
  Info: TRegKeyInfo;
  P : PChar;

begin
   Strings.Clear;
   if GetKeyInfo(Info) then
     begin
     L:=Info.MaxSubKeyLen+1;
     GetMem(P,L);
     Try
       for I:=0 to Info.NumSubKeys-1 do
         begin
         L:=Info.MaxSubKeyLen+1;
         RegEnumKeyExA(CurrentKey,I,P,L,Nil,Nil,Nil,Nil);
         Strings.Add(StrPas(P));
         end;
     Finally
       FreeMem(P);
     end;
     end;
end;

procedure TRegistry.GetValueNames(Strings: TStrings);

Var
  L : Cardinal;
  I: Integer;
  Info: TRegKeyInfo;
  P : PChar;

begin
   Strings.Clear;
   if GetKeyInfo(Info) then
     begin
     L:=Info.MaxValueLen+1;
     GetMem(P,L);
     Try
       for I:=0 to Info.NumValues-1 do
         begin
         L:=Info.MaxValueLen+1;
         RegEnumValueA(CurrentKey,I,P,L,Nil,Nil,Nil,Nil);
         Strings.Add(StrPas(P));
         end;
     Finally
       FreeMem(P);
     end;
     end;

end;

Function TRegistry.SysPutData(const Name: string; Buffer: Pointer;
  BufSize: Integer; RegData: TRegDataType) : Boolean;

Var
  P: PChar;
  RegDataType: DWORD;

begin
  Case RegData of
    rdUnknown      : RegDataType:=REG_NONE;
    rdString       : RegDataType:=REG_SZ;
    rdExpandString : RegDataType:=REG_EXPAND_SZ;
    rdInteger      : RegDataType:=REG_DWORD;
    rdBinary       : RegDataType:=REG_BINARY;
  end;
  P:=PChar(Name);
  Result:=RegSetValueExA(fCurrentKey,P,0,RegDataType,Buffer,BufSize)=ERROR_SUCCESS;
end;

procedure TRegistry.RenameValue(const OldName, NewName: string);

var
  L: Integer;
  InfoO,InfoN : TRegDataInfo;
  D : TRegDataType;
  P: PChar;

begin
  If GetDataInfo(OldName,InfoO) and Not GetDataInfo(NewName,InfoN) then
    begin
    L:=InfoO.DataSize;
    if L>0 then
      begin
      GetMem(P,L);
      try
        L:=GetData(OldName,P,L,D);
        If SysPutData(NewName,P,L,D) then
          DeleteValue(OldName);
      finally
        FreeMem(P);
      end;
      end;
    end;
end;

procedure TRegistry.SetCurrentKey(Value: HKEY);
begin
  fCurrentKey := Value;
end;

procedure TRegistry.SetRootKey(Value: HKEY);
begin
  if fRootKey = Value then
    Exit;
  { close a root key that was opened using RegistryConnect }
  if PWinRegData(FSysData)^.RootKeyOwned and (fRootKey <> 0) then begin
    RegCloseKey(fRootKey);
    PWinRegData(FSysData)^.RootKeyOwned := False;
  end;
  fRootKey := Value;
end;

