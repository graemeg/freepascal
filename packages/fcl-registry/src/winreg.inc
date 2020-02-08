Const
  RegDataWords : Array [TRegDataType] of DWORD
               = (REG_NONE,REG_SZ,REG_EXPAND_SZ,REG_BINARY,REG_DWORD,REG_DWORD_BIG_ENDIAN,
                  REG_LINK,REG_MULTI_SZ,REG_RESOURCE_LIST,REG_FULL_RESOURCE_DESCRIPTOR,REG_RESOURCE_REQUIREMENTS_LIST,REG_QWORD);

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

Function PrepKey(Const S : UnicodeString) : UnicodeString;

begin
  Result := S;
  if (Result <> '') and (Result[1] = '\') then
    System.Delete(Result, 1, 1);
end;

Function RelativeKey(Const S : UnicodeString) : Boolean;

begin
  Result:=(S='') or (S[1]<>'\')
end;

function RegDataWordToRegDataType(RD: DWORD): TRegDataType;
begin
  // Test in ascending because rdString is most commonly used
  Result := Succ(Low(RegDataWords));
  repeat
    if RegDataWords[Result] = RD then
      Exit;
    Inc(Result);
  until Result > High(Result);
  Result := Low(RegDataWords);
end;

function TRegistry.sysCreateKey(Key: UnicodeString): Boolean;
Var
  Disposition: Dword;
  Handle: HKEY;
  SecurityAttributes: Pointer; //LPSECURITY_ATTRIBUTES;
  U: UnicodeString;

begin
  SecurityAttributes := Nil;
  U:=PrepKey(Key);
  FLastError:=RegCreateKeyExW(GetBaseKey(RelativeKey(Key)),
                              PWideChar(U),
                              0,
                              '',
                              REG_OPTION_NON_VOLATILE,
                              FACCESS,
                              SecurityAttributes,
                              Handle,
                              @Disposition);
  Result:=FLastError=ERROR_SUCCESS;
  RegCloseKey(Handle);
end;

function TRegistry.DeleteKey(const Key: UnicodeString): Boolean;

Var
  u: UnicodeString;
begin
  u:=PRepKey(Key);
  FLastError:=RegDeleteKeyW(GetBaseKey(RelativeKey(Key)),PWideChar(u));
  Result:=FLastError=ERROR_SUCCESS;
end;


function TRegistry.DeleteValue(const Name: UnicodeString): Boolean;
begin
  FLastError:= RegDeleteValueW(fCurrentKey, PWideChar(Name));
  Result:=FLastError=ERROR_SUCCESS;
end;


function TRegistry.SysGetData(const Name: UnicodeString; Buffer: Pointer;
          BufSize: Integer; Out RegData: TRegDataType): Integer;
Var
  RD : DWord;

begin
  FLastError:=RegQueryValueExW(fCurrentKey,PWideChar(Name),Nil,
                      @RD,Buffer,lpdword(@BufSize));
  if (FLastError<>ERROR_SUCCESS) Then
    Result:=-1
  else
    begin
    RegData:=RegDataWordToRegDataType(RD);
    Result:=BufSize;
    end;
end;

function TRegistry.GetDataInfo(const ValueName: UnicodeString; out Value: TRegDataInfo): Boolean;

Var
  RD : DWord;

begin
  With Value do
    begin
    FLastError:=RegQueryValueExW(fCurrentKey,PWideChar(ValueName),Nil,@RD,Nil,lpdword(@DataSize));
    Result:=FLastError=ERROR_SUCCESS;
    if Result then
      RegData:=RegDataWordToRegDataType(RD);
    end;
  If Not Result Then
    begin
    Value.RegData := rdUnknown;
    Value.DataSize := 0
    end
end;


function TRegistry.GetKey(Key: UnicodeString): HKEY;
var
  Rel : Boolean;
begin
  Result:=0;
  Rel:=RelativeKey(Key);
  if not(Rel) then
    Delete(Key,1,1);
{$ifdef WinCE}
  FLastError:=RegOpenKeyEx(GetBaseKey(Rel),PWideChar(Key),0,FAccess,Result);
{$else WinCE}
  FLastError:=RegOpenKeyExW(GetBaseKey(Rel),PWideChar(Key),0,FAccess,Result);
{$endif WinCE}
end;


function TRegistry.GetKeyInfo(out Value: TRegKeyInfo): Boolean;
var
  winFileTime: Windows.FILETIME;
  sysTime: TSystemTime;
  LocalFileTime: Windows.FILETIME;
begin
  FillChar(Value, SizeOf(Value), 0);
  With Value do
    begin
    FLastError:=RegQueryInfoKeyA(CurrentKey,nil,nil,nil,lpdword(@NumSubKeys),
              lpdword(@MaxSubKeyLen),nil,lpdword(@NumValues),lpdword(@MaxValueLen),
              lpdword(@MaxDataLen),nil,@winFileTime);
    Result:=FLastError=ERROR_SUCCESS;          
    end;          
  if Result and FileTimeToLocalFileTime(@winFileTime, @LocalFileTime) and
    FileTimeToSystemTime(@LocalFileTime, @sysTime) then
  begin
    Value.FileTime := SystemTimeToDateTime(sysTime);
  end;
end;


function TRegistry.KeyExists(const Key: UnicodeString): Boolean;
var
  KeyHandle : HKEY;
  OldAccess : LONG;
begin
  Result:=false;
  OldAccess:=FAccess;
  try
    FAccess:=KEY_QUERY_VALUE or KEY_ENUMERATE_SUB_KEYS or STANDARD_RIGHTS_READ 
    {$ifndef WinCE} or (OldAccess and (KEY_WOW64_64KEY or KEY_WOW64_32KEY)) {$endif};
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


function TRegistry.LoadKey(const Key, FileName: UnicodeString): Boolean;
begin
  Result := False;
end;



function TRegistry.OpenKey(const Key: UnicodeString; CanCreate: Boolean): Boolean;

Var
  u, S: UnicodeString;
  Handle: HKEY;
  Disposition: Integer;
  SecurityAttributes: Pointer; //LPSECURITY_ATTRIBUTES;
begin
  SecurityAttributes := Nil;
  u:=PrepKey(Key);
  If CanCreate then
    begin
    Handle:=0;
    FLastError:=RegCreateKeyExW(GetBaseKey(RelativeKey(Key)),PWideChar(u),0,'',
                           REG_OPTION_NON_VOLATILE,
                           fAccess,SecurityAttributes,Handle,
                           pdword(@Disposition));
    Result:=FLastError=ERROR_SUCCESS;
    end
  else
    begin
    FLastError:=RegOpenKeyExW(GetBaseKey(RelativeKey(Key)),
                              PWideChar(u),0,fAccess,Handle);
    Result:=FLastError=ERROR_SUCCESS;
    end;                     
  If Result then begin
    if RelativeKey(Key) then
      S:=CurrentPath + Key
    else
      S:=u;
    ChangeKey(Handle, S);
  end;
end;


function TRegistry.OpenKeyReadOnly(const Key: UnicodeString): Boolean;

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


function TRegistry.RegistryConnect(const UNCName: UnicodeString): Boolean;
{$ifndef WinCE}
var
  newroot: HKEY;
{$endif}
begin
{$ifdef WinCE}
  Result:=False;
{$else}
  FLastError:=RegConnectRegistryW(PWideChar(UNCName),RootKey,newroot);
  Result:=FLastError=ERROR_SUCCESS;
  if Result then begin
    RootKey:=newroot;
    PWinRegData(FSysData)^.RootKeyOwned:=True;
  end;
{$endif}
end;


function TRegistry.ReplaceKey(const Key, FileName, BackUpFileName: UnicodeString): Boolean;
begin
  Result := False;
end;


function TRegistry.RestoreKey(const Key, FileName: UnicodeString): Boolean;
begin
  Result := False;
end;


function TRegistry.SaveKey(const Key, FileName: UnicodeString): Boolean;
begin
  Result := False;
end;


function TRegistry.UnLoadKey(const Key: UnicodeString): Boolean;
begin
  Result := false;
end;


function TRegistry.ValueExists(const Name: UnicodeString): Boolean;

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

procedure TRegistry.ChangeKey(Value: HKey; const Path: UnicodeString);
begin
  CloseKey;
  FCurrentKey:=Value;
  FCurrentPath:=Path;
end;


function TRegistry.GetKeyNames: TUnicodeStringArray;

var
  Info:    TRegKeyInfo;
  dwLen:   DWORD;
  lpName:  LPWSTR;
  dwIndex: DWORD;
  lResult: LONGINT;
  u:       UnicodeString;

begin
  Result:=nil;
  if GetKeyInfo(Info) then
  begin
    dwLen:=Info.MaxSubKeyLen+1;
    GetMem(lpName,dwLen*SizeOf(WideChar));
    try
      //writeln('TRegistry.GetKeyNames: Info.NumSubKeys=',Info.NumSubKeys);
      SetLength(Result, Info.NumSubKeys);
      for dwIndex:=0 to Info.NumSubKeys-1 do
      begin
        dwLen:=Info.MaxSubKeyLen+1;
        lResult:=RegEnumKeyExW(CurrentKey,dwIndex,lpName,dwLen,Nil,Nil,Nil,Nil);
        if lResult<>ERROR_SUCCESS then
          raise ERegistryException.Create(SysErrorMessage(lResult));
        if dwLen=0 then
          u:=''
        else
        begin           // dwLen>0
          u:=lpName;
        end;            // if dwLen=0
        Result[dwIndex]:=u;
      end;              // for dwIndex:=0 ...
    finally
      FreeMem(lpName);
    end;
  end;
end;


Function TRegistry.GetValueNames: TUnicodeStringArray;

var
  Info:    TRegKeyInfo;
  dwLen:   DWORD;
  lpName:  LPWSTR;
  dwIndex: DWORD;
  lResult: LONGINT;
  u:       UnicodeString;

begin
  Result:=nil;
  if GetKeyInfo(Info) then
  begin
    dwLen:=Info.MaxValueLen+1;
    GetMem(lpName,dwLen*SizeOf(WideChar));
    try
      SetLength(Result, Info.NumValues);
      for dwIndex:=0 to Info.NumValues-1 do
      begin
        dwLen:=Info.MaxValueLen+1;
        lResult:=RegEnumValueW(CurrentKey,dwIndex,lpName,dwLen,Nil,Nil,Nil,Nil);
        if lResult<>ERROR_SUCCESS then
          raise ERegistryException.Create(SysErrorMessage(lResult));
        if dwLen=0 then
          u:=''
        else
        begin           // dwLen>0
          u:=lpName;
        end;            // if dwLen=0
        Result[dwIndex]:=u;
      end;              // for dwIndex:=0 ...

    finally
      FreeMem(lpName);
    end;
  end;
end;


Function TRegistry.SysPutData(const Name: UnicodeString; Buffer: Pointer;
  BufSize: Integer; RegData: TRegDataType) : Boolean;


Var
  RegDataType: DWORD;
  B : Pchar;
  S : String;

begin
  RegDataType:=RegDataWords[RegData];
  FLastError:=RegSetValueExW(fCurrentKey,PWideChar(Name),0,RegDataType,Buffer,BufSize);
  Result:=FLastError=ERROR_SUCCESS;
end;

procedure TRegistry.RenameValue(const OldName, NewName: UnicodeString);

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

function TRegistry.GetLastErrorMsg: string;
begin
  if FLastError <> ERROR_SUCCESS then
    Result:=SysErrorMessage(FLastError)
  else
    Result:='';
end;

