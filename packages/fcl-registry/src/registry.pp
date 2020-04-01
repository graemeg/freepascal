Unit Registry;

{$mode objfpc}
{$H+}

interface

{$ifndef windows}
{$define XMLREG}
{$endif}

Uses
  {$ifndef XMLREG}
    Windows,
  {$endif XMLREG}
    Classes,
    SysUtils,
    inifiles;

{$I regdef.inc}

type
  ERegistryException = class(Exception);

  TRegKeyInfo = record
    NumSubKeys: Integer;
    MaxSubKeyLen: Integer;
    NumValues: Integer;
    MaxValueLen: Integer;
    MaxDataLen: Integer;
    FileTime: TDateTime;
  end;

  TRegDataType = (rdUnknown, rdString, rdExpandString, rdBinary, rdInteger, rdIntegerBigEndian,
                  rdLink, rdMultiString, rdResourceList, rdFullResourceDescriptor,  rdResourceRequirementList, rdInt64);

  TRegDataInfo = record
    RegData: TRegDataType;
    DataSize: Integer;
  end;

  TUnicodeStringArray = Array of UnicodeString;

{ ---------------------------------------------------------------------
    TRegistry
  ---------------------------------------------------------------------}

  { TRegistry }

  TRegistry = class(TObject)
  private
    FLastError: Longint;
    FStringSizeIncludesNull : Boolean;
    FSysData : Pointer;
    fAccess: LongWord;
    fCurrentKey: HKEY;
    fRootKey: HKEY;
    fLazyWrite: Boolean;
    fCurrentPath: UnicodeString;
    function FixPath(APath: UnicodeString): UnicodeString;
    function GetLastErrorMsg: string;
    function RegMultiSzDataToUnicodeStringArray(U: UnicodeString): TUnicodeStringArray;
    function ListToArray(List: TStrings; IsUtf8: Boolean): TUnicodeStringArray;
    procedure ArrayToList(const Arr: TUnicodeStringArray; List: TStrings; ForceUtf8: Boolean);
    procedure SetRootKey(Value: HKEY);
    Procedure SysRegCreate;
    Procedure SysRegFree;
    Function  SysGetData(const Name: UnicodeString; Buffer: Pointer; BufSize: Integer; Out RegData: TRegDataType): Integer;
    Function  SysPutData(const Name: UnicodeString; Buffer: Pointer; BufSize: Integer; RegData: TRegDataType) : Boolean;
    Function  SysCreateKey(Key: UnicodeString): Boolean;
  protected
    function GetBaseKey(Relative: Boolean): HKey;
    function GetData(const Name: UnicodeString; Buffer: Pointer;
                  BufSize: Integer; Out RegData: TRegDataType): Integer;
    function GetData(const Name: String; Buffer: Pointer;
                  BufSize: Integer; Out RegData: TRegDataType): Integer;
    function GetKey(Key: UnicodeString): HKEY;
    function GetKey(Key: String): HKEY;
    procedure ChangeKey(Value: HKey; const Path: UnicodeString);
    procedure ChangeKey(Value: HKey; const Path: String);
    procedure PutData(const Name: UnicodeString; Buffer: Pointer;
                  BufSize: Integer; RegData: TRegDataType);
    procedure PutData(const Name: String; Buffer: Pointer;
                  BufSize: Integer; RegData: TRegDataType);
    procedure SetCurrentKey(Value: HKEY);
  public
    constructor Create; overload;
    constructor Create(aaccess:longword); overload;
    destructor Destroy; override;

    function CreateKey(const Key: UnicodeString): Boolean;
    function CreateKey(const Key: String): Boolean;
    function DeleteKey(const Key: UnicodeString): Boolean;
    function DeleteKey(const Key: String): Boolean;
    function DeleteValue(const Name: UnicodeString): Boolean;
    function DeleteValue(const Name: String): Boolean;
    function GetDataInfo(const ValueName: UnicodeString; Out Value: TRegDataInfo): Boolean;
    function GetDataInfo(const ValueName: String; Out Value: TRegDataInfo): Boolean;
    function GetDataSize(const ValueName: UnicodeString): Integer;
    function GetDataSize(const ValueName: String): Integer;
    function GetDataType(const ValueName: UnicodeString): TRegDataType;
    function GetDataType(const ValueName: String): TRegDataType;
    function GetKeyInfo(Out Value: TRegKeyInfo): Boolean;
    function HasSubKeys: Boolean;
    function KeyExists(const Key: UnicodeString): Boolean;
    function KeyExists(const Key: String): Boolean;
    function LoadKey(const Key, FileName: UnicodeString): Boolean;  unimplemented;
    function LoadKey(const Key, FileName: String): Boolean;  unimplemented;
    function OpenKey(const Key: UnicodeString; CanCreate: Boolean): Boolean;
    function OpenKey(const Key: String; CanCreate: Boolean): Boolean;
    function OpenKeyReadOnly(const Key: UnicodeString): Boolean;
    function OpenKeyReadOnly(const Key: String): Boolean;
    function ReadCurrency(const Name: UnicodeString): Currency;
    function ReadCurrency(const Name: String): Currency;
    function ReadBinaryData(const Name: UnicodeString; var Buffer; BufSize: Integer): Integer;
    function ReadBinaryData(const Name: String; var Buffer; BufSize: Integer): Integer;
    function ReadBool(const Name: UnicodeString): Boolean;
    function ReadBool(const Name: String): Boolean;
    function ReadDate(const Name: UnicodeString): TDateTime;
    function ReadDate(const Name: String): TDateTime;
    function ReadDateTime(const Name: UnicodeString): TDateTime;
    function ReadDateTime(const Name: String): TDateTime;
    function ReadFloat(const Name: UnicodeString): Double;
    function ReadFloat(const Name: String): Double;
    function ReadInteger(const Name: UnicodeString): Integer;
    function ReadInteger(const Name: String): Integer;
    function ReadInt64(const Name: UnicodeString): Int64;
    function ReadInt64(const Name: String): Int64;
    function ReadString(const Name: UnicodeString): UnicodeString;
    function ReadString(const Name: String): string;
    procedure ReadStringList(const Name: UnicodeString; AList: TStrings; ForceUtf8: Boolean=False);
    procedure ReadStringList(const Name: String; AList: TStrings);
    function ReadStringArray(const Name: UnicodeString): TUnicodeStringArray;
    function ReadStringArray(const Name: String): TStringArray;
    function ReadTime(const Name: UnicodeString): TDateTime;
    function ReadTime(const Name: String): TDateTime;
    function RegistryConnect(const UNCName: UnicodeString): Boolean;
    function RegistryConnect(const UNCName: String): Boolean;
    function ReplaceKey(const Key, FileName, BackUpFileName: UnicodeString): Boolean; unimplemented;
    function ReplaceKey(const Key, FileName, BackUpFileName: String): Boolean;  unimplemented;
    function RestoreKey(const Key, FileName: UnicodeString): Boolean;  unimplemented;
    function RestoreKey(const Key, FileName: String): Boolean;  unimplemented;
    function SaveKey(const Key, FileName: UnicodeString): Boolean;
    function SaveKey(const Key, FileName: String): Boolean;
    function UnLoadKey(const Key: UnicodeString): Boolean;
    function UnLoadKey(const Key: String): Boolean;
    function ValueExists(const Name: UnicodeString): Boolean;
    function ValueExists(const Name: String): Boolean;

    procedure CloseKey;
    procedure CloseKey(key:HKEY);
    procedure GetKeyNames(Strings: TStrings);
    function GetKeyNames: TUnicodeStringArray;
    procedure GetValueNames(Strings: TStrings);
    //ToDo
    function GetValueNames: TUnicodeStringArray;
    procedure MoveKey(const OldName, NewName: UnicodeString; Delete: Boolean);  unimplemented;
    procedure MoveKey(const OldName, NewName: String; Delete: Boolean);  unimplemented;
    procedure RenameValue(const OldName, NewName: UnicodeString);
    procedure RenameValue(const OldName, NewName: String);
    procedure WriteCurrency(const Name: UnicodeString; Value: Currency);
    procedure WriteCurrency(const Name: String; Value: Currency);
    procedure WriteBinaryData(const Name: UnicodeString; const Buffer; BufSize: Integer);
    procedure WriteBinaryData(const Name: String; const Buffer; BufSize: Integer);
    procedure WriteBool(const Name: UnicodeString; Value: Boolean);
    procedure WriteBool(const Name: String; Value: Boolean);
    procedure WriteDate(const Name: UnicodeString; Value: TDateTime);
    procedure WriteDate(const Name: String; Value: TDateTime);
    procedure WriteDateTime(const Name: UnicodeString; Value: TDateTime);
    procedure WriteDateTime(const Name: String; Value: TDateTime);
    procedure WriteFloat(const Name: UnicodeString; Value: Double);
    procedure WriteFloat(const Name: String; Value: Double);
    procedure WriteInteger(const Name: UnicodeString; Value: Integer);
    procedure WriteInteger(const Name: String; Value: Integer);
    procedure WriteInt64(const Name: UnicodeString; Value: Int64);
    procedure WriteInt64(const Name: String; Value: Int64);
    procedure WriteString(const Name, Value: UnicodeString);
    procedure WriteString(const Name, Value: String);
    procedure WriteExpandString(const Name, Value: UnicodeString);
    procedure WriteExpandString(const Name, Value: String);
    procedure WriteStringList(const Name: UnicodeString; List: TStrings; IsUtf8: Boolean=False);
    procedure WriteStringArray(const Name: UnicodeString; const Arr: TUnicodeStringArray);
    procedure WriteStringArray(const Name: String; const Arr: TStringArray);
    procedure WriteTime(const Name: UnicodeString; Value: TDateTime);
    procedure WriteTime(const Name: String; Value: TDateTime);

    property Access: LongWord read fAccess write fAccess;
    property CurrentKey: HKEY read fCurrentKey;
    property CurrentPath: UnicodeString read fCurrentPath;
    property LazyWrite: Boolean read fLazyWrite write fLazyWrite;
    property RootKey: HKEY read fRootKey write SetRootKey;
    Property StringSizeIncludesNull : Boolean read FStringSizeIncludesNull;
    property LastError: Longint read FLastError; platform;
    property LastErrorMsg: string read GetLastErrorMsg; platform;
  end;

{ ---------------------------------------------------------------------
    TRegIniFile
  ---------------------------------------------------------------------}
  TRegIniFile = class(TRegistry)
  private
    fFileName          : String;
    fPath              : String;
    fPreferStringValues: Boolean;
    function OpenSection(const Section: string; CreateSection : Boolean = false): boolean;
    procedure CloseSection;
  public
    constructor Create(const FN: string); overload;
    constructor Create(const FN: string;aaccess:longword); overload;
    function ReadString(const Section, Ident, Default: string): string;
    function ReadInteger(const Section, Ident: string; Default: Longint): Longint;
    function ReadBool(const Section, Ident: string; Default: Boolean): Boolean;
    function ReadDate(const Section, Ident: string; Default: TDateTime):TDateTime;
    function ReadDateTime(const Section, Ident: string; Default: TDateTime):TDateTime;
    function ReadTime(const Section, Ident: string; Default: TDateTime):TDateTime;
    function ReadFloat(const Section, Ident: string; Default: Double): Double;

    procedure WriteString(const Section, Ident, Value: String);
    procedure WriteInteger(const Section, Ident: string; Value: Longint);
    procedure WriteBool(const Section, Ident: string; Value: Boolean);
    procedure WriteDate(const Section, Ident: string; Value: TDateTime);
    procedure WriteDateTime(const Section, Ident: string; Value: TDateTime);
    procedure WriteTime(const Section, Ident: string; Value: TDateTime);
    procedure WriteFloat(const Section, Ident: string; Value: Double);
    procedure ReadSection(const Section: string; Strings: TStrings);
    procedure ReadSections(Strings: TStrings);
    procedure ReadSectionValues(const Section: string; Strings: TStrings);
    procedure EraseSection(const Section: string);
    procedure DeleteKey(const Section, Ident: String);

    property FileName: String read fFileName;
    property PreferStringValues: Boolean read fPreferStringValues
                write fPreferStringValues;
  end{$ifdef XMLREG}deprecated 'Use TRegistry instead. Will be removed in 4.0'{$endif} platform; 

{ ---------------------------------------------------------------------
    TRegIniFile
  ---------------------------------------------------------------------}


  TRegistryIniFile = class(TCustomIniFile)
  private
    FRegIniFile: TRegIniFile;
  public
    constructor Create(const AFileName: string); overload;
    constructor Create(const AFileName: string; AAccess: LongWord); overload;
    destructor destroy; override;
    function ReadDate(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadDateTime(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadInteger(const Section, Name: string; Default: Longint): Longint; override;
    function ReadFloat(const Section, Name: string; Default: Double): Double; override;
    function ReadString(const Section, Name, Default: string): string; override;
    function ReadTime(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadBinaryStream(const Section, Name: string; Value: TStream): Integer; override; unimplemented;
    procedure WriteDate(const Section, Name: string; Value: TDateTime); override;
    procedure WriteDateTime(const Section, Name: string; Value: TDateTime); override;
    procedure WriteFloat(const Section, Name: string; Value: Double); override;
    procedure WriteInteger(const Section, Name: string; Value: Longint); override;
    procedure WriteString(const Section, Name, Value: String); override;
    procedure WriteTime(const Section, Name: string; Value: TDateTime); override;
    procedure WriteBinaryStream(const Section, Name: string; Value: TStream); override;
    procedure ReadSection(const Section: string; Strings: TStrings); override;
    procedure ReadSections(Strings: TStrings); override;
    procedure ReadSectionValues(const Section: string; Strings: TStrings); override;
    procedure EraseSection(const Section: string); override;
    procedure DeleteKey(const Section, Name: String); override;
    procedure UpdateFile; override;
    function ValueExists(const Section, Ident: string): Boolean; override;
    property RegIniFile: TRegIniFile read FRegIniFile;
  end{$ifdef XMLREG}deprecated 'Use TRegistry instead. Will be removed in 4.0'{$endif} platform; 

ResourceString
  SInvalidRegType   = 'Invalid registry data type: "%s"';
  SRegCreateFailed  = 'Failed to create key: "%s"';
  SRegSetDataFailed = 'Failed to set data for value "%s"';
  SRegGetDataFailed = 'Failed to get data for value "%s"';

var
  GlobalXMLFile : Boolean = False;

implementation

{ ---------------------------------------------------------------------
    Include implementation-dependent code
  ---------------------------------------------------------------------}


{$ifdef XMLREG}
{$i xregreg.inc}
{$else}
{$i winreg.inc}
{$endif}

{ ---------------------------------------------------------------------
    Generic, implementation-independent code.
  ---------------------------------------------------------------------}

{$ifdef DebugRegistry}
function DbgS(const S: UnicodeString): String;
var
  C: WideChar;
begin
  Result := '';
  for C in S do Result := Result + IntToHex(Word(C),4) + #32;
  Result := TrimRight(Result);
end;
{$endif}

constructor TRegistry.Create;

begin
  inherited Create;
  FAccess     := KEY_ALL_ACCESS;
  FRootKey    := HKEY_CURRENT_USER;
  FLazyWrite  := True;
  FCurrentKey := 0;
  SysRegCreate;
end;

constructor TRegistry.Create(aaccess: longword);

begin
  Create;
  FAccess     := aaccess;
end;

destructor TRegistry.Destroy;
begin
  CloseKey;
  SysRegFree;
  inherited Destroy;
end;

function TRegistry.CreateKey(const Key: UnicodeString): Boolean;

begin
  Result:=SysCreateKey(Key);
  If Not Result Then
    Raise ERegistryException.CreateFmt(SRegCreateFailed, [Key]);
end;

function TRegistry.CreateKey(const Key: String): Boolean;
begin
  Result:=CreateKey(UnicodeString(Key));
end;

function TRegistry.DeleteKey(const Key: String): Boolean;
begin
  Result:=DeleteKey(UnicodeString(Key));
end;

function TRegistry.DeleteValue(const Name: String): Boolean;
begin
  Result:=DeleteValue(UnicodeString(Name));
end;

function TRegistry.GetDataInfo(const ValueName: String; out Value: TRegDataInfo
  ): Boolean;
begin
  Result:=GetDataInfo(UnicodeString(ValueName), Value);
end;

function TRegistry.GetBaseKey(Relative: Boolean): HKey;
begin
  If Relative and (CurrentKey<>0) Then
    Result := CurrentKey
  else
    Result := RootKey;
end;

function TRegistry.GetData(const Name: UnicodeString; Buffer: Pointer; BufSize: Integer; out RegData: TRegDataType): Integer;
begin
  Result:=SysGetData(Name,Buffer,BufSize,RegData);
  If (Result=-1) then
    Raise ERegistryException.CreateFmt(SRegGetDataFailed, [Name]);
end;

function TRegistry.GetData(const Name: String; Buffer: Pointer;
  BufSize: Integer; out RegData: TRegDataType): Integer;
begin
  Result:=GetData(UnicodeString(Name), Buffer, BufSize, RegData);
end;

function TRegistry.GetKey(Key: String): HKEY;
begin
  Result:=GetKey(UnicodeString(Key));
end;

procedure TRegistry.ChangeKey(Value: HKey; const Path: String);
begin
  ChangeKey(Value, UnicodeString(Path));
end;


procedure TRegistry.PutData(const Name: UnicodeString; Buffer: Pointer;
  BufSize: Integer; RegData: TRegDataType);

begin
  If Not SysPutData(Name,Buffer,BufSize,RegData) then
    Raise ERegistryException.CreateFmt(SRegSetDataFailed, [Name]);
end;

procedure TRegistry.PutData(const Name: String; Buffer: Pointer;
  BufSize: Integer; RegData: TRegDataType);
begin
  PutData(UnicodeString(Name), Buffer, BufSize, RegData);
end;


function TRegistry.GetDataSize(const ValueName: UnicodeString): Integer;

Var
  Info: TRegDataInfo;

begin
  If GetDataInfo(ValueName,Info) Then
    Result := Info.DataSize
  else
    Result := -1;
end;

function TRegistry.GetDataSize(const ValueName: String): Integer;
begin
  Result:=GetDataSize(UnicodeString(ValueName));
end;

function TRegistry.GetDataType(const ValueName: UnicodeString): TRegDataType;

Var
  Info: TRegDataInfo;

begin
  GetDataInfo(ValueName, Info);
  Result:=Info.RegData;
end;

function TRegistry.GetDataType(const ValueName: String): TRegDataType;
begin
  Result:=GetDataType(UnicodeString(ValueName));
end;


function TRegistry.KeyExists(const Key: String): Boolean;
begin
  Result:=KeyExists(UnicodeString(Key));
end;

function TRegistry.LoadKey(const Key, FileName: String): Boolean;
begin
  Result:=LoadKey(UnicodeString(Key), UnicodeString(FileName));
end;

function TRegistry.OpenKey(const Key: String; CanCreate: Boolean): Boolean;
begin
  Result:=OpenKey(UnicodeString(Key), CanCreate);
end;

function TRegistry.OpenKeyReadOnly(const Key: String): Boolean;
begin
  Result:=OpenKeyReadOnly(UnicodeString(Key));
end;

function TRegistry.HasSubKeys: Boolean;

Var
  Info : TRegKeyInfo;

begin
  Result:=GetKeyInfo(Info);
  If Result then
    Result:=(Info.NumSubKeys>0);
end;

function TRegistry.ReadBinaryData(const Name: UnicodeString; var Buffer; BufSize: Integer): Integer;

Var
  RegDataType: TRegDataType;

begin
  Result := GetData(Name, @Buffer, BufSize, RegDataType);
  If not (RegDataType in [rdBinary, rdUnknown]) Then
    Raise ERegistryException.CreateFmt(SInvalidRegType, [Name]);
end;

function TRegistry.ReadBinaryData(const Name: String; var Buffer;
  BufSize: Integer): Integer;
begin
  Result:=ReadBinaryData(UnicodeString(Name), Buffer, BufSize);
end;

function TRegistry.ReadInteger(const Name: UnicodeString): Integer;

Var
  RegDataType: TRegDataType;

begin
  GetData(Name, @Result, SizeOf(Integer), RegDataType);
  If RegDataType<>rdInteger Then
    Raise ERegistryException.CreateFmt(SInvalidRegType, [Name]);
end;

function TRegistry.ReadInteger(const Name: String): Integer;
begin
  Result:=ReadInteger(UnicodeString(Name));
end;

function TRegistry.ReadInt64(const Name: UnicodeString): Int64;

Var
  RegDataType: TRegDataType;

begin
  GetData(Name, @Result, SizeOf(Int64), RegDataType);
  If RegDataType<>rdInt64 Then
    Raise ERegistryException.CreateFmt(SInvalidRegType, [Name]);
end;

function TRegistry.ReadInt64(const Name: String): Int64;
begin
  Result:=ReadInt64(UnicodeString(Name));
end;

function TRegistry.ReadBool(const Name: UnicodeString): Boolean;

begin
  Result:=ReadInteger(Name)<>0;
end;

function TRegistry.ReadBool(const Name: String): Boolean;
begin
  Result:=ReadBool(UnicodeString(Name));
end;

function TRegistry.ReadCurrency(const Name: UnicodeString): Currency;

begin
  Result:=Default(Currency);
  ReadBinaryData(Name, Result, SizeOf(Currency));
end;

function TRegistry.ReadCurrency(const Name: String): Currency;
begin
  Result:=ReadCurrency(UnicodeString(Name));
end;

function TRegistry.ReadDate(const Name: UnicodeString): TDateTime;

begin
  Result:=Trunc(ReadDateTime(Name));
end;

function TRegistry.ReadDate(const Name: String): TDateTime;
begin
  Result:=ReadDate(UnicodeString(Name));
end;

function TRegistry.ReadDateTime(const Name: UnicodeString): TDateTime;

begin
  Result:=Default(TDateTime);
  ReadBinaryData(Name, Result, SizeOf(TDateTime));
end;

function TRegistry.ReadDateTime(const Name: String): TDateTime;
begin
  Result:=ReadDateTime(UnicodeString(Name));
end;

function TRegistry.ReadFloat(const Name: UnicodeString): Double;

begin
  Result:=Default(Double);
  ReadBinaryData(Name,Result,SizeOf(Double));
end;

function TRegistry.ReadFloat(const Name: String): Double;
begin
  Result:=ReadFloat(UnicodeString(Name));
end;

function TRegistry.ReadString(const Name: UnicodeString): UnicodeString;

Var
  Info : TRegDataInfo;
  ReadDataSize: Integer;
  u: UnicodeString;

begin
  Result:='';
  GetDataInfo(Name,Info);
  if info.datasize>0 then
  begin
    if Not (Info.RegData in [rdString,rdExpandString]) then
      Raise ERegistryException.CreateFmt(SInvalidRegType, [Name]);
    if Odd(Info.DataSize) then
      SetLength(u,round((Info.DataSize+1)/SizeOf(UnicodeChar)))
    else
      SetLength(u,round(Info.DataSize/SizeOf(UnicodeChar)));
    ReadDataSize := GetData(Name,@u[1],Info.DataSize,Info.RegData);
    if ReadDataSize > 0 then
    begin
      // If the data has the REG_SZ, REG_MULTI_SZ or REG_EXPAND_SZ type,
      // the size includes any terminating null character or characters
      // unless the data was stored without them! (RegQueryValueEx @ MSDN)
      if StringSizeIncludesNull and
         (u[Length(u)] = WideChar(0)) then
        SetLength(u,Length(u)-1);
      Result:=u;
    end;
  end;
end;

function TRegistry.ReadString(const Name: String): string;
begin
  Result:=ReadString(UnicodeString(Name));
end;


procedure TRegistry.ReadStringList(const Name: UnicodeString; AList: TStrings; ForceUtf8: Boolean=False);

Var
  UArr: TUnicodeStringArray;

begin
  UArr := ReadStringArray(Name);
  ArrayToList(UArr, AList, ForceUtf8);
end;

procedure TRegistry.ReadStringList(const Name: String; AList: TStrings);
begin
  ReadStringList(UnicodeString(Name), AList);
end;

function TRegistry.FixPath(APath: UnicodeString): UnicodeString;
const
  Delim={$ifdef XMLREG}'/'{$else}'\'{$endif};
begin
  //At this point we know the path is valid, since this is only called after OpenKey succeeded
  //Just sanitize it
  while (Pos(Delim+Delim,APath) > 0) do
    APath := UnicodeStringReplace(APath, Delim+Delim,Delim,[rfReplaceAll]);
  if (Length(APath) > 1) and (APath[Length(APath)] = Delim) then
    System.Delete(APath, Length(APath), 1);
  Result := APath;
end;

function TRegistry.RegMultiSzDataToUnicodeStringArray(U: UnicodeString): TUnicodeStringArray;
var
  Len, i, p: Integer;
  Sub: UnicodeString;
begin
  Result := nil;
  if (U = '') then Exit;
  Len := 1;
  for i := 1 to Length(U) do if (U[i] = #0) then Inc(Len);
  SetLength(Result, Len);
  i := 0;

  while (U <> '') and (i < Length(Result)) do
  begin
    p := Pos(#0, U);
    if (p = 0) then p := Length(U) + 1;
    Sub := Copy(U, 1, p - 1);
    Result[i] := Sub;
    System.Delete(U, 1, p);
    Inc(i);
  end;
end;

function TRegistry.ListToArray(List: TStrings; IsUtf8: Boolean): TUnicodeStringArray;
var
  i, curr, Len: Integer;
  u: UnicodeString;
begin
  Result := nil;
  Len := List.Count;
  SetLength(Result, Len);
  //REG_MULTI_SZ data cannot contain empty strings
  curr := 0;
  for i := 0 to List.Count - 1 do
  begin
    if IsUtf8 then
      u := Utf8Decode(List[i])
    else
      u := List[i];
    if (u>'') then
    begin
      Result[curr] := u;
      inc(curr);
    end
    else
      Dec(Len);
  end;
  if (Len <> List.Count) then SetLength(Result, Len);
end;

procedure TRegistry.ArrayToList(const Arr: TUnicodeStringArray; List: TStrings; ForceUtf8: Boolean);
var
  i: Integer;
begin
  List.Clear;
  for i := Low(Arr) to High(Arr) do
  begin
    if ForceUtf8 then
      List.Add(Utf8Encode(Arr[i]))
    else
      List.Add(String(Arr[i]));
  end;
end;

function TRegistry.ReadStringArray(const Name: UnicodeString): TUnicodeStringArray;
Var
  Info : TRegDataInfo;
  ReadDataSize: Integer;
  Data: UnicodeString;

begin
  Result := nil;
  GetDataInfo(Name,Info);
  //writeln('TRegistry.ReadStringArray: datasize=',info.datasize);
  if info.datasize>0 then
    begin
     If Not (Info.RegData in [rdMultiString]) then
       Raise ERegistryException.CreateFmt(SInvalidRegType, [Name]);
     SetLength(Data,Info.DataSize);
     ReadDataSize := GetData(Name,PWideChar(Data),Info.DataSize,Info.RegData) div SizeOf(WideChar);
     //writeln('TRegistry.ReadStringArray: ReadDataSize=',ReadDataSize);
     if ReadDataSize > 0 then
     begin
       // Windows returns the data with or without trailing zero's, so just strip all trailing null characters
        while (Data[ReadDataSize] = #0) do Dec(ReadDataSize);
       SetLength(Data, ReadDataSize);
       //writeln('Data=',dbgs(data));
       //Data := UnicodeStringReplace(Data, #0, AList.LineBreak, [rfReplaceAll]);
       //AList.Text := Data;
       Result := RegMultiSzDataToUnicodeStringArray(Data);
     end
   end
end;

function TRegistry.ReadStringArray(const Name: String): TStringArray;
var
  UArr: TUnicodeStringArray;
  i: Integer;
begin
  Result := nil;
  UArr := ReadStringArray(UnicodeString(Name));
  SetLength(Result, Length(UArr));
  for i := Low(UArr) to High(UArr) do Result[i] := UArr[i];
end;

function TRegistry.ReadTime(const Name: UnicodeString): TDateTime;

begin
  Result:=Frac(ReadDateTime(Name));
end;

function TRegistry.ReadTime(const Name: String): TDateTime;
begin
  Result:=ReadTime(UnicodeString(Name));
end;

function TRegistry.RegistryConnect(const UNCName: String): Boolean;
begin
  Result:=RegistryConnect(UnicodeString(UNCName));
end;

function TRegistry.ReplaceKey(const Key, FileName, BackUpFileName: String): Boolean;
begin
  Result:=ReplaceKey(UnicodeString(Key), UnicodeString(FileName), UnicodeString(BackUpFileName))
end;

function TRegistry.RestoreKey(const Key, FileName: String): Boolean;
begin
  Result:=RestoreKey(UnicodeString(Key), UnicodeString(FileName));
end;

function TRegistry.SaveKey(const Key, FileName: String): Boolean;
begin
  Result:=SaveKey(UnicodeString(Key), UnicodeString(FileName));
end;

function TRegistry.UnLoadKey(const Key: String): Boolean;
begin
  Result:=UnloadKey(UnicodeString(Key));
end;

function TRegistry.ValueExists(const Name: String): Boolean;
begin
  Result:=ValueExists(UnicodeString(Name));
end;

procedure TRegistry.WriteBinaryData(const Name: UnicodeString; const Buffer; BufSize: Integer);
begin
  PutData(Name, @Buffer, BufSize, rdBinary);
end;

procedure TRegistry.WriteBinaryData(const Name: String; const Buffer;
  BufSize: Integer);
begin
  WriteBinaryData(UnicodeString(Name), Buffer, BufSize);
end;

procedure TRegistry.WriteBool(const Name: UnicodeString; Value: Boolean);

begin
  WriteInteger(Name,Ord(Value));
end;

procedure TRegistry.WriteBool(const Name: String; Value: Boolean);
begin
  WriteBool(UnicodeString(Name), Value);
end;

procedure TRegistry.WriteCurrency(const Name: UnicodeString; Value: Currency);
begin
  WriteBinaryData(Name, Value, SizeOf(Currency));
end;

procedure TRegistry.WriteCurrency(const Name: String; Value: Currency);
begin
  WriteCurrency(UnicodeString(Name), Value);
end;

procedure TRegistry.WriteDate(const Name: UnicodeString; Value: TDateTime);
begin
  WriteBinarydata(Name, Value, SizeOf(TDateTime));
end;

procedure TRegistry.WriteDate(const Name: String; Value: TDateTime);
begin
  WriteDate(UnicodeString(Name), Value);
end;

procedure TRegistry.WriteTime(const Name: UnicodeString; Value: TDateTime);
begin
  WriteBinaryData(Name, Value, SizeOf(TDateTime));
end;

procedure TRegistry.WriteTime(const Name: String; Value: TDateTime);
begin
  WriteTime(UnicodeString(Name), Value);
end;

procedure TRegistry.WriteDateTime(const Name: UnicodeString; Value: TDateTime);
begin
  WriteBinaryData(Name, Value, SizeOf(TDateTime));
end;

procedure TRegistry.WriteDateTime(const Name: String; Value: TDateTime);
begin
  WriteDateTime(UnicodeString(Name), Value);
end;

procedure TRegistry.WriteExpandString(const Name, Value: UnicodeString);
begin
  PutData(Name, PWideChar(Value), ByteLength(Value), rdExpandString);
end;

procedure TRegistry.WriteExpandString(const Name, Value: String);
begin
  WriteExpandString(UnicodeString(Name), UnicodeString(Value));
end;


procedure TRegistry.WriteStringList(const Name: UnicodeString; List: TStrings; IsUtf8: Boolean=False);

Var
  UArr: TUnicodeStringArray;
begin
  UArr := ListToArray(List, IsUtf8);
  WriteStringArray(Name, UArr);
end;

procedure TRegistry.WriteStringArray(const Name: UnicodeString; const Arr: TUnicodeStringArray);
Var
  Data: UnicodeString;
  u: UnicodeString;
  i: Integer;
begin
  Data := '';
  //REG_MULTI_SZ data cannot contain empty strings
  for i := Low(Arr) to High(Arr) do
  begin
    u := Arr[i];
    if (u>'') then
    begin
      if (Data>'') then
        Data := Data + #0 + u
      else
        Data := Data + u;
    end;
  end;
  if StringSizeIncludesNull then
    Data := Data + #0#0;
  //writeln('Data=',Dbgs(Data));
  PutData(Name, PWideChar(Data), ByteLength(Data), rdMultiString);
end;

procedure TRegistry.WriteStringArray(const Name: String; const Arr: TStringArray);
var
  UArr: TUnicodeStringArray;
  i: Integer;
begin
  UArr := nil;
  SetLength(UArr, Length(Arr));
  for i := Low(Arr) to High(Arr) do UArr[i] := Arr[i];
  WriteStringArray(UnicodeString(Name), UArr);
end;

procedure TRegistry.WriteFloat(const Name: UnicodeString; Value: Double);
begin
  WriteBinaryData(Name, Value, SizeOf(Double));
end;

procedure TRegistry.WriteFloat(const Name: String; Value: Double);
begin
  WriteFloat(UnicodeString(Name), Value);
end;

procedure TRegistry.WriteInteger(const Name: UnicodeString; Value: Integer);
begin
  PutData(Name, @Value, SizeOf(Integer), rdInteger);
end;

procedure TRegistry.WriteInteger(const Name: String; Value: Integer);
begin
  WriteInteger(UnicodeString(Name), Value);
end;

procedure TRegistry.WriteInt64(const Name: UnicodeString; Value: Int64);
begin
  PutData(Name, @Value, SizeOf(Int64), rdInt64);
end;

procedure TRegistry.WriteInt64(const Name: String; Value: Int64);
begin
  WriteInt64(UnicodeString(Name), Value);
end;

procedure TRegistry.WriteString(const Name, Value: UnicodeString);
begin
  PutData(Name, PWideChar(Value), ByteLength(Value), rdString);
end;

procedure TRegistry.WriteString(const Name, Value: String);
begin
  WriteString(UnicodeString(Name), UnicodeString(Value));
end;

procedure TRegistry.GetKeyNames(Strings: TStrings);
var
  UArr: TUnicodeStringArray;
begin
  UArr := GetKeyNames;
  ArrayToList(UArr, Strings, True);
end;

procedure TRegistry.GetValueNames(Strings: TStrings);
var
  UArr: TUnicodeStringArray;
begin
  UArr := GetValueNames;
  ArrayToList(UArr, Strings, True);
end;

procedure TRegistry.MoveKey(const OldName, NewName: UnicodeString; Delete: Boolean);
begin

end;

procedure TRegistry.MoveKey(const OldName, NewName: String; Delete: Boolean);
begin
  MoveKey(UnicodeString(OldName), UnicodeString(NewName), Delete);
end;

procedure TRegistry.RenameValue(const OldName, NewName: String);
begin
  RenameValue(UnicodeString(OldName), UnicodeString(NewName));
end;

{ ---------------------------------------------------------------------
    Include TRegIniFile implementation
  ---------------------------------------------------------------------}

{$i regini.inc}

{ TRegistryIniFile }

// interface from
// http://www.koders.com/delphi/fid65C1FFAEF89B0CDC4B93FF94C1819686CA6141FC.aspx
constructor TRegistryIniFile.Create(const AFileName: string;
  AAccess: LongWord);
begin
  inherited create(AFilename);
  FRegInifile:=TreginiFile.Create(AFileName,AAccess);
end;

constructor TRegistryIniFile.Create(const AFileName: string);
begin
  Create(AFileName,KEY_ALL_ACCESS);
end;

destructor TRegistryIniFile.destroy;

begin
  FreeAndNil(FRegInifile);
  Inherited;
end;

procedure TRegistryIniFile.DeleteKey(const Section, Name: String);
begin
  FRegIniFile.Deletekey(section,name);
end;

procedure TRegistryIniFile.EraseSection(const Section: string);
begin
  FRegIniFile.EraseSection(section);
end;

function TRegistryIniFile.ReadBinaryStream(const Section, Name: string;
  Value: TStream): Integer;
begin
  result:=-1; // unimplemented
 //
end;

function TRegistryIniFile.ReadDate(const Section, Name: string;
  Default: TDateTime): TDateTime;
begin
  Result:=FRegInifile.ReadDate(Section,Name,Default);
end;

function TRegistryIniFile.ReadDateTime(const Section, Name: string;
  Default: TDateTime): TDateTime;
begin
  Result:=FRegInifile.ReadDateTime(Section,Name,Default);
end;

function TRegistryIniFile.ReadFloat(const Section, Name: string;
  Default: Double): Double;
begin
  Result:=FRegInifile.ReadFloat(Section,Name,Default);
end;

function TRegistryIniFile.ReadInteger(const Section, Name: string;
  Default: Integer): Longint;
begin
  Result:=FRegInifile.ReadInteger(Section, Name, Default);
end;

procedure TRegistryIniFile.ReadSection(const Section: string; Strings: TStrings);
begin
  FRegIniFile.ReadSection(Section,strings);
end;

procedure TRegistryIniFile.ReadSections(Strings: TStrings);
begin
  FRegIniFile.ReadSections(strings);
end;

procedure TRegistryIniFile.ReadSectionValues(const Section: string;
  Strings: TStrings);
begin
  FRegIniFile.ReadSectionValues(Section,strings);
end;

function TRegistryIniFile.ReadString(const Section, Name,
  Default: string): string;
begin
  Result:=FRegInifile.ReadString(Section, Name, Default);
end;

function TRegistryIniFile.ReadTime(const Section, Name: string;
  Default: TDateTime): TDateTime;
begin
  Result:=FRegInifile.ReadTime(Section,Name,Default);
end;

procedure TRegistryIniFile.UpdateFile;
begin
//  FRegIniFile.UpdateFile; ??
end;

procedure TRegistryIniFile.WriteBinaryStream(const Section, Name: string;
  Value: TStream);
begin
 // ??
end;

procedure TRegistryIniFile.WriteDate(const Section, Name: string;
  Value: TDateTime);
begin
  FRegInifile.WriteDate(Section,Name, Value);
end;

procedure TRegistryIniFile.WriteDateTime(const Section, Name: string;
  Value: TDateTime);
begin
  FRegInifile.WriteDateTime(Section,Name, Value);
end;

procedure TRegistryIniFile.WriteFloat(const Section, Name: string;
  Value: Double);
begin
  FRegInifile.WriteFloat(Section,Name, Value);
end;

procedure TRegistryIniFile.WriteInteger(const Section, Name: string;
  Value: Integer);
begin
  FRegInifile.WriteInteger(Section, Name, Value);
end;

procedure TRegistryIniFile.WriteString(const Section, Name, Value: String);
begin
  FRegInifile.WriteString(Section, Name, Value);
end;

procedure TRegistryIniFile.WriteTime(const Section, Name: string;
  Value: TDateTime);
begin
  FRegInifile.WriteTime(Section,Name, Value);
end;

function TRegistryIniFile.ValueExists(const Section, Ident: string): Boolean;
begin
  with FRegInifile do
    if OpenSection(Section) then
      try
        Result:=FRegInifile.ValueExists(Ident);
      finally
        CloseSection;
      end;
end;

{$ifdef XMLREG}
finalization
  TXMLRegistryInstance.FreeXMLRegistryCache;
{$endif}

end.
