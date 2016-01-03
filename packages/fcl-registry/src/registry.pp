Unit registry;

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
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

  {$I regdef.inc}
>>>>>>> graemeg/fixes_2_2

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

  TRegDataType = (rdUnknown, rdString, rdExpandString, rdBinary, rdInteger);

  TRegDataInfo = record
    RegData: TRegDataType;
    DataSize: Integer;
  end;

{ ---------------------------------------------------------------------
    TRegistry
  ---------------------------------------------------------------------}

  TRegistry = class(TObject)
  private
    FStringSizeIncludesNull : Boolean;
    FSysData : Pointer;
    fAccess: LongWord;
    fCurrentKey: HKEY;
    fRootKey: HKEY;
    fLazyWrite: Boolean;
    fCurrentPath: string;
    procedure SetRootKey(Value: HKEY);
    Procedure SysRegCreate;
    Procedure SysRegFree;
    Function  SysGetData(const Name: String; Buffer: Pointer; BufSize: Integer; var RegData: TRegDataType): Integer;
    Function  SysPutData(const Name: string; Buffer: Pointer; BufSize: Integer; RegData: TRegDataType) : Boolean;
    Function  SysCreateKey(const Key: String): Boolean;
  protected
    function GetBaseKey(Relative: Boolean): HKey;
    function GetData(const Name: string; Buffer: Pointer;
                  BufSize: Integer; var RegData: TRegDataType): Integer;
    function GetKey(const Key: string): HKEY;
    procedure ChangeKey(Value: HKey; const Path: string);
    procedure PutData(const Name: string; Buffer: Pointer;
                  BufSize: Integer; RegData: TRegDataType);
    procedure SetCurrentKey(Value: HKEY);
  public
    constructor Create; overload;
    constructor Create(aaccess:longword); overload;
    destructor Destroy; override;

    function CreateKey(const Key: string): Boolean;
    function DeleteKey(const Key: string): Boolean;
    function DeleteValue(const Name: string): Boolean;
    function GetDataInfo(const ValueName: string; var Value: TRegDataInfo): Boolean;
    function GetDataSize(const ValueName: string): Integer;
    function GetDataType(const ValueName: string): TRegDataType;
    function GetKeyInfo(var Value: TRegKeyInfo): Boolean;
    function HasSubKeys: Boolean;
    function KeyExists(const Key: string): Boolean;
    function LoadKey(const Key, FileName: string): Boolean;
    function OpenKey(const Key: string; CanCreate: Boolean): Boolean;
    function OpenKeyReadOnly(const Key: string): Boolean;
    function ReadCurrency(const Name: string): Currency;
    function ReadBinaryData(const Name: string; var Buffer; BufSize: Integer): Integer;
    function ReadBool(const Name: string): Boolean;
    function ReadDate(const Name: string): TDateTime;
    function ReadDateTime(const Name: string): TDateTime;
    function ReadFloat(const Name: string): Double;
    function ReadInteger(const Name: string): Integer;
    function ReadString(const Name: string): string;
    function ReadTime(const Name: string): TDateTime;
    function RegistryConnect(const UNCName: string): Boolean;
    function ReplaceKey(const Key, FileName, BackUpFileName: string): Boolean;
    function RestoreKey(const Key, FileName: string): Boolean;
    function SaveKey(const Key, FileName: string): Boolean;
    function UnLoadKey(const Key: string): Boolean;
    function ValueExists(const Name: string): Boolean;

    procedure CloseKey;
    procedure CloseKey(key:HKEY);
    procedure GetKeyNames(Strings: TStrings);
    procedure GetValueNames(Strings: TStrings);
    procedure MoveKey(const OldName, NewName: string; Delete: Boolean);
    procedure RenameValue(const OldName, NewName: string);
    procedure WriteCurrency(const Name: string; Value: Currency);
    procedure WriteBinaryData(const Name: string; var Buffer; BufSize: Integer);
    procedure WriteBool(const Name: string; Value: Boolean);
    procedure WriteDate(const Name: string; Value: TDateTime);
    procedure WriteDateTime(const Name: string; Value: TDateTime);
    procedure WriteFloat(const Name: string; Value: Double);
    procedure WriteInteger(const Name: string; Value: Integer);
    procedure WriteString(const Name, Value: string);
    procedure WriteExpandString(const Name, Value: string);
    procedure WriteTime(const Name: string; Value: TDateTime);

    property Access: LongWord read fAccess write fAccess;
    property CurrentKey: HKEY read fCurrentKey;
    property CurrentPath: string read fCurrentPath;
    property LazyWrite: Boolean read fLazyWrite write fLazyWrite;
    property RootKey: HKEY read fRootKey write SetRootKey;
    Property StringSizeIncludesNull : Boolean read FStringSizeIncludesNull;
  end;

{ ---------------------------------------------------------------------
    TRegIniFile
  ---------------------------------------------------------------------}
  TRegIniFile = class(TRegistry)
  private
    fFileName          : String;
    fPath              : String;
    fPreferStringValues: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function OpenSection(const Section: string; CreateSection : Boolean = false): boolean;
    procedure CloseSection;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
  end;

{ ---------------------------------------------------------------------
    TRegIniFile
  ---------------------------------------------------------------------}


  TRegistryIniFile = class(TCustomIniFile)
  private
    FRegIniFile: TRegIniFile;
  public
    constructor Create(const AFileName: string); overload;
    constructor Create(const AFileName: string; AAccess: LongWord); overload;
    function ReadDate(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadDateTime(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadInteger(const Section, Name: string; Default: Longint): Longint; override;
    function ReadFloat(const Section, Name: string; Default: Double): Double; override;
    function ReadString(const Section, Name, Default: string): string; override;
    function ReadTime(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadBinaryStream(const Section, Name: string; Value: TStream): Integer; override;
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
  end;

{ ---------------------------------------------------------------------
    TRegIniFile
  ---------------------------------------------------------------------}


  TRegistryIniFile = class(TCustomIniFile)
  private
    FRegIniFile: TRegIniFile;
  public
    constructor Create(const AFileName: string); overload;
    constructor Create(const AFileName: string; AAccess: LongWord); overload;
    function ReadDate(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadDateTime(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadInteger(const Section, Name: string; Default: Longint): Longint; override;
    function ReadFloat(const Section, Name: string; Default: Double): Double; override;
    function ReadString(const Section, Name, Default: string): string; override;
    function ReadTime(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadBinaryStream(const Section, Name: string; Value: TStream): Integer; override;
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
    property RegIniFile: TRegIniFile read FRegIniFile;
  end;

{ ---------------------------------------------------------------------
    TRegIniFile
  ---------------------------------------------------------------------}


  TRegistryIniFile = class(TCustomIniFile)
  private
    FRegIniFile: TRegIniFile;
  public
    constructor Create(const AFileName: string); overload;
    constructor Create(const AFileName: string; AAccess: LongWord); overload;
    function ReadDate(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadDateTime(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadInteger(const Section, Name: string; Default: Longint): Longint; override;
    function ReadFloat(const Section, Name: string; Default: Double): Double; override;
    function ReadString(const Section, Name, Default: string): string; override;
    function ReadTime(const Section, Name: string; Default: TDateTime): TDateTime; override;
    function ReadBinaryStream(const Section, Name: string; Value: TStream): Integer; override;
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
    property RegIniFile: TRegIniFile read FRegIniFile;
  end;

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


Constructor TRegistry.Create;

begin
  inherited Create;
  FAccess     := KEY_ALL_ACCESS;
  FRootKey    := HKEY_CURRENT_USER;
  FLazyWrite  := True;
  FCurrentKey := 0;
  SysRegCreate;
end;

Constructor TRegistry.Create(aaccess:longword);

begin
  Create;
  FAccess     := aaccess;
end;

Destructor TRegistry.Destroy;
begin
  CloseKey;
  SysRegFree;
  inherited Destroy;
end;

function TRegistry.CreateKey(const Key: String): Boolean;

begin
  Result:=SysCreateKey(Key);
  If Not Result Then
    Raise ERegistryException.CreateFmt(SRegCreateFailed, [Key]);
end;

function TRegistry.GetBaseKey(Relative: Boolean): HKey;
begin
  If Relative and (CurrentKey<>0) Then
    Result := CurrentKey
  else
    Result := RootKey;
end;

function TRegistry.GetData(const Name: String; Buffer: Pointer;
          BufSize: Integer; var RegData: TRegDataType): Integer;
begin
  Result:=SysGetData(Name,Buffer,BufSize,RegData);
  If (Result=-1) then
    Raise ERegistryException.CreateFmt(SRegGetDataFailed, [Name]);
end;

procedure TRegistry.PutData(const Name: string; Buffer: Pointer;
  BufSize: Integer; RegData: TRegDataType);

begin
  If Not SysPutData(Name,Buffer,BufSize,RegData) then
    Raise ERegistryException.CreateFmt(SRegSetDataFailed, [Name]);
end;


function TRegistry.GetDataSize(const ValueName: String): Integer;

Var
  Info: TRegDataInfo;

begin
  If GetDataInfo(ValueName,Info) Then
    Result := Info.DataSize
  else
    Result := -1;
end;

function TRegistry.GetDataType(const ValueName: string): TRegDataType;

Var
  Info: TRegDataInfo;

begin
  GetDataInfo(ValueName, Info);
  Result:=Info.RegData;
end;

Function TRegistry.HasSubKeys: Boolean;

Var
  Info : TRegKeyInfo;

begin
  Result:=GetKeyInfo(Info);
  If Result then
    Result:=(Info.NumSubKeys>0);
end;

function TRegistry.ReadBinaryData(const Name: string; var Buffer; BufSize: Integer): Integer;

Var
  RegDataType: TRegDataType;

begin
  Result := GetData(Name, @Buffer, BufSize, RegDataType);
  If (RegDataType<>rdBinary) Then
    Raise ERegistryException.CreateFmt(SInvalidRegType, [Name]);
end;

function TRegistry.ReadInteger(const Name: string): Integer;

Var
  RegDataType: TRegDataType;

begin
  GetData(Name, @Result, SizeOf(Integer), RegDataType);
  If RegDataType<>rdInteger Then
    Raise ERegistryException.CreateFmt(SInvalidRegType, [Name]);
end;

function TRegistry.ReadBool(const Name: string): Boolean;

begin
  Result:=ReadInteger(Name)<>0;
end;

function TRegistry.ReadCurrency(const Name: string): Currency;

begin
  ReadBinaryData(Name, Result, SizeOf(Currency));
end;

function TRegistry.ReadDate(const Name: string): TDateTime;

begin
  ReadBinaryData(Name, Result, SizeOf(TDateTime));
  Result:=Trunc(Result);
end;

function TRegistry.ReadDateTime(const Name: string): TDateTime;

begin
  ReadBinaryData(Name, Result, SizeOf(TDateTime));
end;

function TRegistry.ReadFloat(const Name: string): Double;

begin
  ReadBinaryData(Name,Result,SizeOf(Double));
end;

function TRegistry.ReadString(const Name: string): string;

Var
  Info : TRegDataInfo;
  ReadDataSize: Integer;

begin
  GetDataInfo(Name,Info);
  if info.datasize>0 then
    begin
     If Not (Info.RegData in [rdString,rdExpandString]) then
       Raise ERegistryException.CreateFmt(SInvalidRegType, [Name]);
     SetLength(Result,Info.DataSize);
     ReadDataSize := GetData(Name,PChar(Result),Info.DataSize,Info.RegData);
     if ReadDataSize > 0 then
     begin
       // If the data has the REG_SZ, REG_MULTI_SZ or REG_EXPAND_SZ type,
       // the size includes any terminating null character or characters
       // unless the data was stored without them! (RegQueryValueEx @ MSDN)
       if StringSizeIncludesNull then
         if Result[ReadDataSize] = #0 then
           Dec(ReadDataSize);
       SetLength(Result, ReadDataSize);
     end
     else
<<<<<<< HEAD
       Result := '';
=======
       SetLength(Result, Info.DataSize);
     GetData(Name,PChar(Result),Info.DataSize,Info.RegData);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   end
  else
    result:='';
end;

function TRegistry.ReadTime(const Name: string): TDateTime;

begin
  ReadBinaryData(Name, Result, SizeOf(TDateTime));
  Result:=Frac(Result);
end;

procedure TRegistry.WriteBinaryData(const Name: string; var Buffer; BufSize: Integer);
begin
  PutData(Name, @Buffer, BufSize, rdBinary);
end;

procedure TRegistry.WriteBool(const Name: string; Value: Boolean);

begin
  WriteInteger(Name,Ord(Value));
end;

procedure TRegistry.WriteCurrency(const Name: string; Value: Currency);
begin
  WriteBinaryData(Name, Value, SizeOf(Currency));
end;

procedure TRegistry.WriteDate(const Name: string; Value: TDateTime);
begin
  WriteBinarydata(Name, Value, SizeOf(TDateTime));
end;

procedure TRegistry.WriteTime(const Name: string; Value: TDateTime);
begin
  WriteBinaryData(Name, Value, SizeOf(TDateTime));
end;

procedure TRegistry.WriteDateTime(const Name: string; Value: TDateTime);
begin
  WriteBinaryData(Name, Value, SizeOf(TDateTime));
end;

procedure TRegistry.WriteExpandString(const Name, Value: string);

begin
  PutData(Name, PChar(Value), Length(Value),rdExpandString);
end;

procedure TRegistry.WriteFloat(const Name: string; Value: Double);
begin
  WriteBinaryData(Name, Value, SizeOf(Double));
end;

procedure TRegistry.WriteInteger(const Name: string; Value: Integer);
begin
  PutData(Name, @Value, SizeOf(Integer), rdInteger);
end;

procedure TRegistry.WriteString(const Name, Value: string);

begin
  PutData(Name, PChar(Value), Length(Value), rdString);
end;

procedure TRegistry.MoveKey(const OldName, NewName: string; Delete: Boolean);
begin

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
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=FRegInifile.ReadDate(Section,Name,Default);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;
begin 
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              if ValueExists(Name) THen 
                result:=FRegIniFile.ReadDate(Name)
              else
                result:=default;
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
       else
         result:=default; 
    end;          
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TRegistryIniFile.ReadDateTime(const Section, Name: string;
  Default: TDateTime): TDateTime;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=FRegInifile.ReadDateTime(Section,Name,Default);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              if ValueExists(Name) THen 
                result:=FRegIniFile.ReadDateTime(Name)
              else
                result:=default;
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
       else
         result:=default; 
    end;          
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TRegistryIniFile.ReadFloat(const Section, Name: string;
  Default: Double): Double;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=FRegInifile.ReadFloat(Section,Name,Default);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              if ValueExists(Name) THen 
                result:=FRegIniFile.ReadFloat(Name)
              else
                result:=default;
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
       else
         result:=default; 
    end;          
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TRegistryIniFile.ReadInteger(const Section, Name: string;
  Default: Integer): Longint;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=FRegInifile.ReadInteger(Section, Name, Default);
end;

procedure TRegistryIniFile.ReadSection(const Section: string; Strings: TStrings);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              if ValueExists(Name) THen 
                result:=FRegIniFile.ReadInteger(section,Name,default)
              else
                result:=default;
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
       else
         result:=default; 
    end;          
end;

procedure TRegistryIniFile.ReadSection(const Section: string;
  Strings: TStrings);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=FRegInifile.ReadString(Section, Name, Default);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              if ValueExists(Name) THen 
                result:=FRegIniFile.ReadString(section,Name,default)
              else
                result:=default;
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
       else
         result:=default; 
    end;          
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TRegistryIniFile.ReadTime(const Section, Name: string;
  Default: TDateTime): TDateTime;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=FRegInifile.ReadTime(Section,Name,Default);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              if ValueExists(Name) THen 
                result:=FRegIniFile.ReadTime(Name)
              else
                result:=default;
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
       else
         result:=default; 
    end;          
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
begin
  FRegInifile.WriteDate(Section,Name, Value);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              FRegIniFile.WriteDate(name,value)
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
    end;          
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TRegistryIniFile.WriteDateTime(const Section, Name: string;
  Value: TDateTime);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  FRegInifile.WriteDateTime(Section,Name, Value);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              FRegIniFile.WriteDateTime(Name,value)
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TRegistryIniFile.WriteFloat(const Section, Name: string;
  Value: Double);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  FRegInifile.WriteFloat(Section,Name, Value);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              FRegIniFile.WriteFloat(Name,value)
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
    end;          
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TRegistryIniFile.WriteInteger(const Section, Name: string;
  Value: Integer);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  FRegInifile.WriteInteger(Section, Name, Value);
end;

procedure TRegistryIniFile.WriteString(const Section, Name, Value: String);
begin
  FRegInifile.WriteString(Section, Name, Value);
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              FRegIniFile.WriteInteger(section,Name,value)
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
    end;          

end;

procedure TRegistryIniFile.WriteString(const Section, Name, Value: String);
var sectkey,curkey : HKey;  
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin           
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;               
            SetCurrentKey(sectKey);
            try             // save current key
              FRegIniFile.WriteString(section,Name,value)
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TRegistryIniFile.WriteTime(const Section, Name: string;
  Value: TDateTime);
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
var sectkey,curkey : HKey;
begin
  with FRegInifile do
    begin
      sectkey:=getkey(Section);
      if sectkey<>0 then
        begin
          try // allocation ok
            curkey:=FRegIniFile.CurrentKey;
            SetCurrentKey(sectKey);
            try             // save current key
              FRegIniFile.WriteTime(Name,value)
            finally
              SetCurrentKey(CurKey);
              end;
          finally
            closekey(sectkey);
            end;
        end
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

end.
