{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 Erik WachtMeester.

    File which provides TIniFile and friends.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{* Original disclaimer:
 * FCL inifiles.pp rewrite by Erik Wachtmeester (erikw@hotelconcepts.com)
 *
 * Proposed replacement for inifiles.pp v 1.8
 *
 * This version is Borland Delphi 5 compatible, implementing the classes
 * TCustomIniFile, TIniFile and TMemIniFile, with all the public
 * properties and methods that Delphi 5 implements.
 *
 * (inifiles.pp v 1.8 only implements TIniFile with some properties and
 *  methods missing, and some functionality added)
 *
 * In order to stay compatible with v 1.8, I added:
 * - TIniFile can be created and loaded from, and saved to a stream.
 * - ReadSectionRaw method (although it doesn't add empty lines to the
 *   TStrings recipient like v 1.8, since empty lines aren't stored in
 *   the SectionList object structure)
 * - ReadInteger supports '0x' type hex formats
 * - Comment support (this isn't standard in ini files)
 * - EscapeLineFeeds creation parameter
 *
 * Since the SectionList object structure is very different from the
 * way Delphi 5 accesses ini files (Delphi mostly uses Windows calls
 * like GetPrivateProfileString, etc.) it's completely platform
 * independant, and probably faster.
 * The only drawback is memory consumption: all sections, keys and
 * values are kept in memory. But same goes for inifiles.pp v 1.8
 * (the FFileBuffer member) and for Delphi's TMemIniFile.
 * Anyway, Windows restricts ini files to 64K max, so this shouldn't be
 * too much of a problem.
 *
 *}

unit IniFiles;

{$mode objfpc}
{$H+}

interface

uses classes, sysutils, contnrs;

type

  { TStringHash }

  TStringHash  = class
  private
    FAddReplacesExisting: Boolean;
    FHashList : TFPDataHashTable;
  public
    constructor Create(ACapacity : Cardinal = 256);
    destructor Destroy;override;
    procedure Add(const Key: string; Value: Integer);
    procedure Clear;
    function Modify(const Key: string; Value: Integer): Boolean;
    procedure Remove(const Key: string);
    function ValueOf(const Key: string): Integer;
    Property AddReplacesExisting : Boolean Read FAddReplacesExisting Write FAddReplacesExisting;
  end;

  { THashedStringList }

  THashedStringList = class(TStringList)
  private
    FValueHash: TFPHashList;
    FNameHash: TFPHashList;
    FValueHashValid: Boolean;
    FNameHashValid: Boolean;
    procedure UpdateValueHash;
    procedure UpdateNameHash;
  protected
    procedure Changed; override;
  public
    destructor Destroy; override;
    function IndexOf(const S: String): Integer; override;
    function IndexOfName(const Name: String): Integer; override;
  end;

  TIniFileKey = class
  Private
    FIdent: string;
    FValue: string;
  public
    constructor Create(const AIdent, AValue: string);
    property Ident: string read FIdent write FIdent;
    property Value: string read FValue write FValue;
  end;

  TIniFileKeyList = class(TList)
  private
    function GetItem(Index: integer): TIniFileKey;
    function KeyByName(const AName: string; CaseSensitive : Boolean): TIniFileKey;
  public
    destructor Destroy; override;
    procedure Clear; override;
    property Items[Index: integer]: TIniFileKey read GetItem; default;
  end;

  TIniFileSection = class
  private
    FName: string;
    FKeyList: TIniFileKeyList;
  public
    Function Empty : Boolean;
    constructor Create(const AName: string);
    destructor Destroy; override;
    property Name: string read FName;
    property KeyList: TIniFileKeyList read FKeyList;
  end;

  TIniFileSectionList = class(TList)
  private
    function GetItem(Index: integer): TIniFileSection;
    function SectionByName(const AName: string; CaseSensitive : Boolean): TIniFileSection;
  public
    destructor Destroy; override;
    procedure Clear;override;
    property Items[Index: integer]: TIniFileSection read GetItem; default;
  end;

  TIniFileOption = (ifoStripComments,    // Strip comments when reading file
                    ifoStripInvalid,     // Strip invalid lines when reading file.
                    ifoEscapeLineFeeds, // Escape linefeeds when reading file.
                    ifoCaseSensitive,   // Use Case sensitive section/key names
                    ifoStripQuotes,     // Strip quotes when reading string values.
                    ifoFormatSettingsActive, // Use format settings when writing date/float etc.
                    ifoWriteStringBoolean // Write booleans as string
                    );
  TIniFileOptions = Set of TIniFileOption;

  TSectionValuesOption = (svoIncludeComments,svoIncludeInvalid, svoIncludeQuotes);
  TSectionValuesOptions = set of TSectionValuesOption;

  { TCustomIniFile }

  TCustomIniFile = class
  Private
    FBoolFalseStrings: TStringArray;
    FBoolTrueStrings: TStringArray;
    FEncoding: TEncoding;
    FFileName: string;
    FOptions: TIniFileOptions;
    FOwnsEncoding: Boolean;
    FSectionList: TIniFileSectionList;
    function GetOption(AIndex: TIniFileOption): Boolean;
    procedure SetOption(AIndex: TIniFileOption; AValue: Boolean);
    procedure SetOptions(AValue: TIniFileOptions);
  protected
    procedure SetEncoding(const aEncoding: TEncoding); virtual;
  public
    FormatSettings: TFormatSettings;
    constructor Create(const AFileName: string; ADefaultEncoding: TEncoding; AOptions : TIniFileOptions = []);
    constructor Create(const AFileName: string; ADefaultEncoding: TEncoding; AOwnsEncoding: Boolean; AOptions : TIniFileOptions = []);
    constructor Create(const AFileName: string; AOptions : TIniFileOptions = []); virtual;
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean); virtual;
    destructor Destroy; override;
    Procedure SetBoolStringValues(ABoolValue : Boolean; Values : Array of string);
    function SectionExists(const Section: string): Boolean; virtual;
    function ReadString(const Section, Ident, Default: string): string; virtual; abstract;
    procedure WriteString(const Section, Ident, Value: String); virtual; abstract;
    function ReadInteger(const Section, Ident: string; Default: Longint): Longint; virtual;
    procedure WriteInteger(const Section, Ident: string; Value: Longint); virtual;
    function ReadInt64(const Section, Ident: string; Default: Int64): Int64; virtual;
    procedure WriteInt64(const Section, Ident: string; Value: Int64); virtual;
    function ReadBool(const Section, Ident: string; Default: Boolean): Boolean; virtual;
    procedure WriteBool(const Section, Ident: string; Value: Boolean); virtual;
    function ReadDate(const Section, Ident: string; Default: TDateTime): TDateTime; virtual;
    function ReadDateTime(const Section, Ident: string; Default: TDateTime): TDateTime; virtual;
    function ReadFloat(const Section, Ident: string; Default: Double): Double; virtual;
    function ReadTime(const Section, Ident: string; Default: TDateTime): TDateTime; virtual;
    function ReadBinaryStream(const Section, Name: string; Value: TStream): Integer; virtual;
    procedure WriteDate(const Section, Ident: string; Value: TDateTime); virtual;
    procedure WriteDateTime(const Section, Ident: string; Value: TDateTime); virtual;
    procedure WriteFloat(const Section, Ident: string; Value: Double); virtual;
    procedure WriteTime(const Section, Ident: string; Value: TDateTime); virtual;
    procedure WriteBinaryStream(const Section, Name: string; Value: TStream); virtual;
    procedure ReadSection(const Section: string; Strings: TStrings); virtual; abstract;
    procedure ReadSections(Strings: TStrings); virtual; abstract;
    procedure ReadSectionValues(const Section: string; Strings: TStrings; Options : TSectionValuesOptions); virtual;    overload;
    procedure ReadSectionValues(const Section: string; Strings: TStrings); virtual;overload;
    procedure EraseSection(const Section: string); virtual; abstract;
    procedure DeleteKey(const Section, Ident: String); virtual; abstract;
    procedure UpdateFile; virtual; abstract;
    function ValueExists(const Section, Ident: string): Boolean; virtual;
    property Encoding: TEncoding read FEncoding write SetEncoding;
    property FileName: string read FFileName;
    Property Options : TIniFileOptions Read FOptions Write SetOptions;
    property EscapeLineFeeds: boolean index ifoEscapeLineFeeds Read GetOption ;deprecated 'Use options instead';
    Property CaseSensitive : Boolean index ifoCaseSensitive Read GetOption Write SetOption; deprecated  'Use options instead';
    Property StripQuotes : Boolean index ifoStripQuotes Read GetOption Write SetOption; deprecated 'Use options instead';
    Property FormatSettingsActive : Boolean index ifoFormatSettingsActive Read GetOption Write SetOption;deprecated  'Use options instead';
    Property BoolTrueStrings : TStringArray Read FBoolTrueStrings Write FBoolTrueStrings;
    Property BoolFalseStrings : TStringArray Read FBoolFalseStrings Write FBoolFalseStrings;
    Property OwnsEncoding: Boolean Read FOwnsEncoding;
  end;

  { TIniFile }

  TIniFile = class(TCustomIniFile)
  Private
    FStream: TStream;
    FCacheUpdates: Boolean;
    FDirty : Boolean;
    FWriteBOM: Boolean;
    procedure FillSectionList(AStrings: TStrings);
    Procedure DeleteSection(ASection : TIniFileSection);
    Procedure MaybeDeleteSection(ASection : TIniFileSection);
    procedure SetCacheUpdates(const AValue: Boolean);
    procedure SetWriteBOM(const aWriteBOM: Boolean);
  protected
    procedure ReadIniValues;
    procedure MaybeUpdateFile;
    property Dirty : Boolean Read FDirty;
    procedure SetEncoding(const aEncoding: TEncoding); override;
  public
    constructor Create(const AFileName: string; AOptions : TIniFileoptions = []); overload; override;
    constructor Create(AStream: TStream; AOptions : TIniFileoptions = []); overload;
    constructor Create(AStream: TStream; AEscapeLineFeeds : Boolean); overload; deprecated 'Use Options argument instead';
    constructor Create(AStream: TStream; ADefaultEncoding: TEncoding; AOptions : TIniFileOptions = []);
    constructor Create(AStream: TStream; ADefaultEncoding: TEncoding; AOwnsEncoding: Boolean; AOptions : TIniFileOptions = []);
    destructor Destroy; override;
    function ReadString(const Section, Ident, Default: string): string; override;
    procedure WriteString(const Section, Ident, Value: String); override;
    procedure ReadSection(const Section: string; Strings: TStrings); override;
    procedure ReadSectionRaw(const Section: string; Strings: TStrings);
    procedure ReadSections(Strings: TStrings); override;
    procedure ReadSectionValues(const Section: string; Strings: TStrings; AOptions : TSectionValuesOptions = [svoIncludeInvalid]); overload; override;
    procedure EraseSection(const Section: string); override;
    procedure DeleteKey(const Section, Ident: String); override;
    procedure UpdateFile; override;
    property Stream: TStream read FStream;
    property CacheUpdates : Boolean read FCacheUpdates write SetCacheUpdates;
    property WriteBOM: Boolean Read FWriteBOM Write SetWriteBOM;
  end;

  { TMemIniFile }

  TMemIniFile = class(TIniFile)
  public
    constructor Create(const AFileName: string; AOptions : TIniFileoptions = []); overload; override;
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean); overload; override;
    procedure Clear;
    procedure GetStrings(List: TStrings);
    procedure Rename(const AFileName: string; Reload: Boolean);
    procedure SetStrings(List: TStrings);
  end;

implementation

Resourcestring
  SErrCouldNotCreatePath = 'Could not create directory "%s"';

const
   Brackets  : array[0..1] of Char = ('[', ']');
   Separator : Char = '=';
   Comment   : Char = ';';
   LF_Escape : Char = '\';

function CharToBool(AChar: char): boolean;
begin
  Result := (Achar = '1');
end;

function BoolToChar(ABool: boolean): char;
begin
  if ABool then
    Result := '1'
  else
    Result := '0';
end;

function IsComment(const AString: string): boolean;
begin
  Result:=(Length(aString)>0) and (Copy(AString, 1, 1) = Comment);
end;

{ TStringHash }

constructor TStringHash.Create(ACapacity : Cardinal = 256);
begin
  FHashList := TFPDataHashTable.Create;
end;

destructor TStringHash.Destroy;
begin
  FreeAndNil(FHashList);
  inherited;
end;

procedure TStringHash.Add(const Key: string; Value: Integer);
begin
  if Not (AddReplacesExisting and Modify(Key,Value)) then
    FHashList.Add(Key, Pointer(Value));
end;

procedure TStringHash.Clear;
begin
  FHashList.Clear;
end;

function TStringHash.Modify(const Key: string; Value: Integer): Boolean;
Var
  AIndex : Integer;
  Node : THTDataNode;

begin
  Node := THTDataNode(FHashList.Find(Key));
  Result:=Assigned(Node);
  if Result Then
    Node.Data:=Pointer(Value);
end;

procedure TStringHash.Remove(const Key: string);

begin
  FHashList.Delete(Key);
end;

function TStringHash.ValueOf(const Key: string): Integer;
Var
  N : THTDataNode;
begin
  N:=THTDataNode(FHashList.Find(Key));
  If Assigned(N) then
    Result:=PTrInt(N.Data)
  else
    Result:=-1;
end;

{ THashedStringList }

destructor THashedStringList.Destroy;
begin
  FreeAndNil(FValueHash);
  FreeAndNil(FNameHash);
  inherited Destroy;
end;

function THashedStringList.IndexOf(const S: String): Integer;
var
  I: Integer;
begin
  if not FValueHashValid then
    UpdateValueHash;

  if CaseSensitive then
    I := FValueHash.FindIndexOf(S)
  else
    I := FValueHash.FindIndexOf(AnsiUpperCase(S));
  if I >= 0 then
    Result := Integer(FValueHash[I])-1
  else
    Result := -1;
end;

function THashedStringList.IndexOfName(const Name: String): Integer;
var
  I: Integer;
begin
  if not FNameHashValid then
    UpdateNameHash;

  if CaseSensitive then
    I := FNameHash.FindIndexOf(Name)
  else
    I := FNameHash.FindIndexOf(AnsiUpperCase(Name));
  if I >= 0 then
    Result := Integer(FNameHash[I])-1
  else
    Result := -1;
end;

procedure THashedStringList.Changed;
begin
  FValueHashValid := False;
  FNameHashValid := False;
  inherited Changed;
end;

procedure THashedStringList.UpdateValueHash;
var
  I: Integer;
begin
  if not Assigned(FValueHash) then
    FValueHash := TFPHashList.Create
  else
    FValueHash.Clear;
  for I := 0 to Count - 1 do
    if CaseSensitive then
      FValueHash.Add(Strings[I], Pointer(I+1))
    else
      FValueHash.Add(AnsiUpperCase(Strings[I]), Pointer(I+1));
  FValueHashValid := True;
end;

procedure THashedStringList.UpdateNameHash;
var
  I: Integer;
begin
  if not Assigned(FNameHash) then
    FNameHash := TFPHashList.Create
  else
    FNameHash.Clear;
  for I := 0 to Count - 1 do
    if CaseSensitive then
      FNameHash.Add(Names[I], Pointer(I+1))
    else
      FNameHash.Add(AnsiUpperCase(Names[I]), Pointer(I+1));
  FNameHashValid := True;
end;

{ TIniFileKey }

constructor TIniFileKey.Create(const AIdent, AValue: string);
begin
  FIdent := AIdent;
  FValue := AValue;
end;

{ TIniFileKeyList }

function TIniFileKeyList.GetItem(Index: integer): TIniFileKey;
begin
  Result := nil;
  if (Index >= 0) and (Index < Count) then
    Result := TIniFileKey(inherited Items[Index]);
end;

function TIniFileKeyList.KeyByName(const AName: string; CaseSensitive : Boolean): TIniFileKey;
var
  i: integer;
begin
  Result := nil;
  if (AName > '') and not IsComment(AName) then
    If CaseSensitive then
      begin
      for i := 0 to Count-1 do
        if Items[i].Ident=AName then
          begin
          Result := Items[i];
          Break;
          end;
      end
    else
      for i := 0 to Count-1 do
        if CompareText(Items[i].Ident, AName) = 0 then begin
          Result := Items[i];
          Break;
        end;
end;

destructor TIniFileKeyList.Destroy;
begin
  Clear;
  inherited Destroy;
end;

procedure TIniFileKeyList.Clear;
var
  i: integer;
begin
  for i := Count-1 downto 0 do
    Items[i].Free;
  inherited Clear;
end;

Function TIniFileSection.Empty : Boolean;

Var
  I : Integer;

begin
  Result:=True;
  I:=0;
  While Result and (I<KeyList.Count)  do
    begin
    result:=IsComment(KeyList[i].Ident);
    Inc(i);
    end;
end;


{ TIniFileSection }

constructor TIniFileSection.Create(const AName: string);
begin
  FName := AName;
  FKeyList := TIniFileKeyList.Create;
end;

destructor TIniFileSection.Destroy;
begin
  FKeyList.Free;
end;

{ TIniFileSectionList }

function TIniFileSectionList.GetItem(Index: integer): TIniFileSection;
begin
  Result := nil;
  if (Index >= 0) and (Index < Count) then
    Result := TIniFileSection(inherited Items[Index]);
end;

function TIniFileSectionList.SectionByName(const AName: string; CaseSensitive : Boolean): TIniFileSection;
var
  i: integer;
begin
  Result := nil;
  if (AName > '') and not IsComment(AName) then
    If CaseSensitive then
      begin
      for i:=0 to Count-1 do
        if (Items[i].Name=AName) then
          begin
          Result := Items[i];
          Break;
          end;
      end
    else
      for i := 0 to Count-1 do
        if CompareText(Items[i].Name, AName) = 0 then
          begin
          Result := Items[i];
          Break;
          end;
end;

destructor TIniFileSectionList.Destroy;
begin
  Clear;
  inherited Destroy;
end;

procedure TIniFileSectionList.Clear;
var
  i: integer;
begin
  for i := Count-1 downto 0 do
    Items[i].Free;
  inherited Clear;
end;

{ TCustomIniFile }


function TCustomIniFile.GetOption(AIndex: TIniFileOption): Boolean;
begin
  Result:=AIndex in FOptions;
end;


procedure TCustomIniFile.SetOption(AIndex: TIniFileOption; AValue: Boolean);
begin
  if AIndex in [ifoStripComments,ifoStripInvalid] then
    Raise Exception.Create('Flags ifoStripComments or ifoStripInvalid must be set/unset in the constructor');
  if AValue then
    Include(FOptions,AIndex)
  else
    Exclude(FOptions,AIndex)
end;

procedure TCustomIniFile.SetOptions(AValue: TIniFileOptions);

Const
  CreateOnlyOptions = [ifoStripComments,ifoStripInvalid];
begin
  if FOptions=AValue then Exit;
  if (Foptions*CreateOnlyOptions)<>(AValue*CreateOnlyOptions) then
    Raise Exception.Create('Can only change StripComments or StripInvalid in constructor');
  FOptions:=AValue;
end;

constructor TCustomIniFile.Create(const AFileName: string; AOptions : TIniFileOptions = []);
begin
  FFileName := AFileName;
  FSectionList := TIniFileSectionList.Create;
  FOptions:=AOptions;
  FormatSettings := DefaultFormatSettings;
  with FormatSettings do begin
    DecimalSeparator := '.';
    ThousandSeparator := ',';
    ListSeparator := ';';
    DateSeparator := '/';
    TimeSeparator := ':';
    ShortDateFormat := 'yyyy/mm/dd';
    ShortTimeFormat := 'hh:nn';
    LongTimeFormat := 'hh:nn:ss';
  end;
end;

constructor TCustomIniFile.Create(const AFileName: string;
  AEscapeLineFeeds: Boolean);
begin
  if AEscapeLineFeeds then
    Create(AFileName,[ifoEscapeLineFeeds])
  else
    Create(AFileName,[])
end;

constructor TCustomIniFile.Create(const AFileName: string;
  ADefaultEncoding: TEncoding; AOwnsEncoding: Boolean;
  AOptions: TIniFileOptions);
begin
  FEncoding := ADefaultEncoding;
  if FEncoding <> nil then
    FOwnsEncoding := AOwnsEncoding and not TEncoding.IsStandardEncoding(FEncoding);
  Create(AFileName, AOptions);
end;

constructor TCustomIniFile.Create(const AFileName: string;
  ADefaultEncoding: TEncoding; AOptions: TIniFileOptions);
begin
  FEncoding := ADefaultEncoding;
  if FEncoding <> nil then
    FOwnsEncoding := not TEncoding.IsStandardEncoding(FEncoding);
  Create(AFileName, AOptions);
end;

destructor TCustomIniFile.Destroy;
begin
  FSectionList.Free;
  if FOwnsEncoding then
    FEncoding.Free;
  inherited Destroy;
end;

procedure TCustomIniFile.SetBoolStringValues(ABoolValue: Boolean;
  Values: array of string);

Var
  A : TstringArray;
  I : Integer;

begin
  SetLength(A,Length(Values));
  For I:=0 to length(Values)-1 do
    A[i]:=Values[i];
  If AboolValue then
    FBoolTrueStrings:=A
  else
    FBoolFalseStrings:=A;
end;

procedure TCustomIniFile.SetEncoding(const aEncoding: TEncoding);
begin
  if FEncoding = aEncoding then Exit;
  if FOwnsEncoding then
    FEncoding.Free;
  FEncoding := aEncoding;
  FOwnsEncoding := Assigned(FEncoding) and not TEncoding.IsStandardEncoding(FEncoding);
end;

function TCustomIniFile.SectionExists(const Section: string): Boolean;

Var
  S : TIniFileSection;

begin
  S:=FSectionList.SectionByName(Section,CaseSensitive);
  Result:=Assigned(S) and Not S.Empty;
end;

function TCustomIniFile.ReadInteger(const Section, Ident: string; Default: Longint): Longint;
begin
  // StrToInfDef() supports hex numbers prefixed with '0x' via val()
  Result := StrToIntDef(ReadString(Section, Ident, ''), Default);
end;

procedure TCustomIniFile.WriteInteger(const Section, Ident: string; Value: Longint);
begin
  WriteString(Section, Ident, IntToStr(Value));
end;

function TCustomIniFile.ReadInt64(const Section, Ident: string; Default: Int64
  ): Int64;
begin
  Result := StrToInt64Def(ReadString(Section, Ident, ''), Default);
end;

procedure TCustomIniFile.WriteInt64(const Section, Ident: string; Value: Int64);
begin
  WriteString(Section, Ident, IntToStr(Value));
end;

function IndexOfString(A : TStringArray; S : String) : integer;

begin
  Result:=Length(A)-1;
  While (Result>=0) and (CompareText(A[Result],S)<>0) do
    Dec(Result);
end;

function TCustomIniFile.ReadBool(const Section, Ident: string; Default: Boolean): Boolean;

var
  s: string;
begin
  Result := Default;
  s:=ReadString(Section, Ident, '');
  if s > '' then
    if (Length(FBoolTrueStrings)>0) or (Length(FBoolFalseStrings)>0) then
      begin
      if IndexOfString(FBoolTrueStrings,S)>=0 then
        Result:=True
      else if IndexOfString(FBoolFalseStrings,S)>=0 then
        Result:=False
      end
    else
      Result := CharToBool(s[1]);
end;

procedure TCustomIniFile.WriteBool(const Section, Ident: string; Value: Boolean);

Var
  S : String;

begin
  if (ifoWriteStringBoolean in options) then
    begin
    if Value then
      begin
      if Length(BoolTrueStrings)>0 then
        S:=BoolTrueStrings[0]
      else
        S:='true';
      end
    else
      begin
      if Length(BoolFalseStrings)>0 then
        S:=BoolFalseStrings[0]
      else
        S:='false';
      end;
    end
  else
    S:=BoolToChar(Value);
  WriteString(Section, Ident, S);
end;

function TCustomIniFile.ReadDate(const Section, Ident: string; Default: TDateTime): TDateTime;

begin
  if FormatSettingsActive then begin
    if not TryStrToDate(ReadString(Section, Ident, ''), Result, FormatSettings) then
      Result := Default;
  end else
    Result := StrToDateDef(ReadString(Section, Ident, ''),Default);
end;

function TCustomIniFile.ReadDateTime(const Section, Ident: string; Default: TDateTime): TDateTime;

begin
  if FormatSettingsActive then begin
    if not TryStrToDateTime(ReadString(Section, Ident, ''), Result, FormatSettings) then
      Result := Default;
  end else
    Result := StrToDateTimeDef(ReadString(Section, Ident, ''),Default);
end;

function TCustomIniFile.ReadFloat(const Section, Ident: string; Default: Double): Double;

begin
  if FormatSettingsActive then
    Result:=StrToFloatDef(ReadString(Section, Ident, ''),Default, FormatSettings)
  else
    Result:=StrToFloatDef(ReadString(Section, Ident, ''),Default);
end;

function TCustomIniFile.ReadTime(const Section, Ident: string; Default: TDateTime): TDateTime;

begin
  if FormatSettingsActive then
    Result := StrToTimeDef(ReadString(Section, Ident, ''),Default, FormatSettings.TimeSeparator)
  else
    Result := StrToTimeDef(ReadString(Section, Ident, ''),Default);
end;

procedure TCustomIniFile.WriteDate(const Section, Ident: string; Value: TDateTime);
begin
  if FormatSettingsActive then
    WriteString(Section, Ident, DateToStr(Value, FormatSettings))
  else
    WriteString(Section, Ident, DateToStr(Value));
end;

procedure TCustomIniFile.WriteDateTime(const Section, Ident: string; Value: TDateTime);
begin
  if FormatSettingsActive then
    WriteString(Section, Ident, DateTimeToStr(Value, FormatSettings))
  else
    WriteString(Section, Ident, DateTimeToStr(Value));
end;

procedure TCustomIniFile.WriteFloat(const Section, Ident: string; Value: Double);
begin
  if FormatSettingsActive then
    WriteString(Section, Ident, FloatToStr(Value, FormatSettings))
  else
    WriteString(Section, Ident, FloatToStr(Value));
end;

procedure TCustomIniFile.WriteTime(const Section, Ident: string; Value: TDateTime);
begin
  if FormatSettingsActive then
    WriteString(Section, Ident, TimeToStr(Value, FormatSettings))
  else
    WriteString(Section, Ident, TimeToStr(Value));
end;

function TCustomIniFile.ValueExists(const Section, Ident: string): Boolean;
var
  oSection: TIniFileSection;
begin
  Result := False;
  oSection := FSectionList.SectionByName(Section,CaseSensitive);
  if oSection <> nil then
    Result := (oSection.KeyList.KeyByName(Ident,CaseSensitive) <> nil);
end;

function TCustomIniFile.ReadBinaryStream(const Section, Name: string; Value: TStream): Integer;

Var
  M : TMemoryStream;
  S : String;
  PB,PR : PByte;
  PC : PChar;
  H : String[3];
  i,l2,code : Integer;


begin
  S:=ReadString(Section,Name,'');
  Setlength(H,3);
  H[1]:='$';
  Result:=Length(S) div 2;
  If Result>0 then
    begin
    GetMem(PR,Result);
    Try
      PC:=PChar(S);
      PB:=PR;
      For I:=1 to Result do
        begin
        H[2]:=PC[0];
        H[3]:=PC[1];
        Val(H,PB^,code);
        Inc(PC,2);
        Inc(PB);
        end;
      Value.WriteBuffer(PR^,Result);
    finally
      FreeMem(PR);
    end;
    end;
end;

procedure TCustomIniFile.WriteBinaryStream(const Section, Name: string;
  Value: TStream);


Var
  M : TMemoryStream;
  S : String;
  PB : PByte;
  PC : PChar;
  H : String[2];
  i : Integer;

begin
  M:=TMemoryStream.Create;
  Try
    M.CopyFrom(Value,0);
    SetLength(S,M.Size*2);
    If (length(S)>0) then
      begin
      PB:=M.Memory;
      PC:=PChar(S);
      For I:=1 to Length(S) div 2 do
        begin
        H:=HexStr(PB^,2);
        PC[0]:=H[1];
        PC[1]:=H[2];
        Inc(PC,2);
        Inc(PB);
        end;
      end;
    WriteString(Section,Name,S);
  Finally
    M.Free;
  end;
end;

procedure TCustomIniFile.ReadSectionValues(const Section: string; Strings: TStrings; Options: TSectionValuesOptions);

type
  TOldSectionValues = Procedure (const Section: string; Strings: TStrings) of object;

var
  CurrSV,
  TCustomSV: TOldSectionValues;
  CurrClass   : TClass;

begin
  if (Options<>[]) then
    Raise Exception.Create('Options not supported, options must be empty');
  // Redirect calls to old implementation, if it is overridden.
  CurrSV:=nil;
  CurrClass:=Classtype;
  while (CurrClass<>nil) and (CurrClass<>TCustomIniFile) do
   CurrClass:=CurrClass.Classparent;
  if CurrClass<>nil then
    begin
    CurrSV:=@Self.ReadSectionValues;
    TCustomSV:=@TCustomIniFile(@CurrClass).ReadSectionValues;
    if TMethod(TCustomSV).Code=TMethod(CurrSV).Code then
      CurrSV:=nil;
   end;
  if Assigned(CurrSV) then
    ReadSectionValues(Section,Strings)
  else
    Raise Exception.Create('ReadSectionValues not overridden');
end;

procedure TCustomIniFile.ReadSectionValues(const Section: string;
  Strings: TStrings);
begin
  ReadSectionValues(Section,Strings,[svoIncludeInvalid]);
end;

{ TIniFile }


constructor TIniFile.Create(const AFileName: string; AOptions: TIniFileoptions);
begin
  If Not (self is TMemIniFile) then
    Include(AOptions,ifoStripQuotes);
  inherited Create(AFileName,AOptions);
  FStream := nil;
  ReadIniValues;
end;

constructor TIniFile.Create(AStream: TStream; AEscapeLineFeeds : Boolean);

begin
  if AEscapeLineFeeds then
    Create(AStream,[ifoEscapeLineFeeds])
  else
    Create(AStream,[]);
end;

constructor TIniFile.Create(AStream: TStream; ADefaultEncoding: TEncoding;
  AOwnsEncoding: Boolean; AOptions: TIniFileOptions);
begin
  FEncoding := ADefaultEncoding;
  if FEncoding <> nil then
    FOwnsEncoding := AOwnsEncoding and not TEncoding.IsStandardEncoding(FEncoding);
  Create(AStream, AOptions);
end;

constructor TIniFile.Create(AStream: TStream; ADefaultEncoding: TEncoding;
  AOptions: TIniFileOptions);
begin
  FEncoding := ADefaultEncoding;
  if FEncoding <> nil then
    FOwnsEncoding := not TEncoding.IsStandardEncoding(FEncoding);
  Create(AStream, AOptions);
end;

constructor TIniFile.Create(AStream: TStream; AOptions: TIniFileoptions);

var
  slLines: TStringList;

begin
  inherited Create('',AOptions);
  FStream := AStream;
  slLines := TStringList.Create;
  try
    // read the ini file values
    slLines.LoadFromStream(FStream, FEncoding);
    FillSectionList(slLines);
    FWriteBOM := Assigned(FEncoding) and ((FEncoding.CodePage=CP_UTF16) or (FEncoding.CodePage=CP_UTF16BE)); // write BOM for UTF16 by default
  finally
    slLines.Free;
  end;
end;

destructor TIniFile.Destroy;
begin
  If FDirty and FCacheUpdates then
    try
      UpdateFile;
    except
      // Eat exception. Compatible to D7 behaviour, see comments to bug 19046
    end;  
  inherited destroy;
end;

procedure TIniFile.FillSectionList(AStrings: TStrings);
var
  i,j,sLen: integer;
  sLine, sIdent, sValue: string;
  oSection: TIniFileSection;

  procedure RemoveBackslashes;
  var
    i,l: integer;
    s: string;
    bAppendNextLine, bAppended: boolean;
  begin
    AStrings.BeginUpdate;
    try
      For I:=AStrings.Count-2 downto 0 do
        begin
        S:=AStrings[i];
        L:=Length(S);
        If (I<AStrings.Count-1) and (L>0) and (S[L]=LF_Escape) then
          begin
          S:=Copy(S,1,L-1)+AStrings[I+1];
          AStrings.Delete(I+1);
          AStrings[i]:=S;
          end;
        end;
    finally
      AStrings.EndUpdate;
    end;
  end;

Var
  addKey : Boolean;

begin
  oSection := nil;
  FSectionList.Clear;
  if EscapeLineFeeds then
    RemoveBackslashes;
  for i := 0 to AStrings.Count-1 do begin
    sLine := Trim(AStrings[i]);
    sLen:=Length(sLine);
    if (sLen>0)  then
      begin
      if IsComment(sLine) and (oSection = nil) then
        begin
        // comment at the beginning of the ini file
        if Not (ifoStripComments in Options) then
          begin
          oSection := TIniFileSection.Create(sLine);
          FSectionList.Add(oSection);
          end;
        continue;
        end;
      if (sLine[1]=Brackets[0]) and (sLine[sLen]= Brackets[1]) then
        begin
        // regular section
        oSection := TIniFileSection.Create(Copy(sLine, 2, sLen - 2));
        FSectionList.Add(oSection);
        end
      else if oSection <> nil then
        begin
        if IsComment(sLine) then
          begin
          AddKey:=Not (ifoStripComments in Options);
          // comment within a section
          sIdent := sLine;
          sValue := '';
          end
        else
          begin
          // regular key
          j:=Pos(Separator, sLine);
          if j=0 then
           begin
           AddKey:=Not (ifoStripInvalid in Options);
           sIdent:='';
           sValue:=sLine
           end
          else
           begin
           AddKey:=True;
           sIdent:=Trim(Copy(sLine, 1,  j - 1));
           sValue:=Trim(Copy(sLine, j + 1, sLen - j));
           end;
        end;
        if AddKey then
          oSection.KeyList.Add(TIniFileKey.Create(sIdent, sValue));
        end;
      end;
  end;
end;

function TIniFile.ReadString(const Section, Ident, Default: string): string;
var
  oSection: TIniFileSection;
  oKey: TIniFileKey;
  J: integer;
begin
  Result := Default;
  oSection := FSectionList.SectionByName(Section,CaseSensitive);
  if oSection <> nil then begin
    oKey := oSection.KeyList.KeyByName(Ident,CaseSensitive);
    if oKey <> nil then
      If StripQuotes then
      begin
        J:=Length(oKey.Value);
        // Joost, 2-jan-2007: The check (J>1) is there for the case that
        // the value consist of a single double-quote character. (see
        // mantis bug 6555)
        If (J>1) and ((oKey.Value[1] in ['"','''']) and (oKey.Value[J]=oKey.Value[1])) then
           Result:=Copy(oKey.Value,2,J-2)
        else
           Result:=oKey.Value;
      end
      else Result:=oKey.Value;
    end;
  end;

procedure TIniFile.SetCacheUpdates(const AValue: Boolean);
begin
  if FCacheUpdates and not AValue and FDirty then
    UpdateFile;
  FCacheUpdates := AValue;
end;

procedure TIniFile.SetEncoding(const aEncoding: TEncoding);
begin
  if FEncoding = aEncoding then Exit;
  inherited SetEncoding(aEncoding);
  if Assigned(FEncoding) and ((FEncoding.CodePage=CP_UTF16) or (FEncoding.CodePage=CP_UTF16BE)) then
    FWriteBOM := True;
  MaybeUpdateFile;
end;

procedure TIniFile.SetWriteBOM(const aWriteBOM: Boolean);
begin
  if FWriteBOM = aWriteBOM then Exit;
  FWriteBOM := aWriteBOM;
  MaybeUpdateFile;
end;

procedure TIniFile.WriteString(const Section, Ident, Value: String);
var
  oSection: TIniFileSection;
  oKey: TIniFileKey;
begin
  if (Section > '') and (Ident > '') then 
    begin
    // update or add key
    oSection := FSectionList.SectionByName(Section,CaseSensitive);
    if (oSection = nil) then 
      begin
      oSection := TIniFileSection.Create(Section);
      FSectionList.Add(oSection);
      end;
    with oSection.KeyList do 
      begin
      oKey := KeyByName(Ident,CaseSensitive);
      if oKey <> nil then
        oKey.Value := Value
      else
        oSection.KeyList.Add(TIniFileKey.Create(Ident, Value));
      end;
    end;
  MaybeUpdateFile;
end;

procedure TIniFile.ReadSection(const Section: string; Strings: TStrings);
var
  oSection: TIniFileSection;
  i: integer;
begin
  Strings.BeginUpdate;
  try
    Strings.Clear;
    oSection := FSectionList.SectionByName(Section,CaseSensitive);
    if oSection <> nil then with oSection.KeyList do
      for i := 0 to Count-1 do
        if not IsComment(Items[i].Ident) then
          Strings.Add(Items[i].Ident);
  finally
    Strings.EndUpdate;
  end;
end;

procedure TIniFile.ReadSectionRaw(const Section: string; Strings: TStrings);
var
  oSection: TIniFileSection;
  i: integer;
begin
  Strings.BeginUpdate;
  try
    Strings.Clear;
    oSection := FSectionList.SectionByName(Section,CaseSensitive);
    if oSection <> nil then with oSection.KeyList do
      for i := 0 to Count-1 do
         begin
           if Items[i].Ident<>'' then
            Strings.Add(Items[i].Ident + Separator +Items[i].Value)
           else
            Strings.Add(Items[i].Value);
         end;
  finally
    Strings.EndUpdate;
  end;
end;

procedure TIniFile.ReadSections(Strings: TStrings);
var
  i: integer;
begin
  Strings.BeginUpdate;
  try
    Strings.Clear;
    for i := 0 to FSectionList.Count-1 do
      if not IsComment(FSectionList[i].Name) then
        Strings.Add(FSectionList[i].Name);
  finally
    Strings.EndUpdate;
  end;
end;

procedure TIniFile.ReadSectionValues(const Section: string; Strings: TStrings; AOptions : TSectionValuesOptions = [svoIncludeInvalid]);
var
  oSection: TIniFileSection;
  s: string;
  i,J: integer;
  KeyIsComment,IncludeComments,IncludeInvalid,DoStripQuotes : boolean;
  K : TIniFileKey;

begin
  IncludeComments:=(svoIncludeComments in AOptions) Or (ifoStripComments in Options);
  IncludeInvalid:=(svoIncludeInvalid in AOptions) Or (ifoStripInvalid in Options);
  DoStripQuotes:=StripQuotes and Not (svoIncludeQuotes in AOptions);
  Strings.BeginUpdate;
  try
    Strings.Clear;
    oSection := FSectionList.SectionByName(Section,CaseSensitive);
    if oSection = nil then
      Exit;
    for i := 0 to oSection.KeyList.Count-1 do
      begin
      K:=oSection.KeyList.Items[i];
      if IncludeInvalid or (K.Ident<>'') then
        begin
        s := K.Value;
        KeyIsComment:=IsComment(K.Ident);
        if IncludeComments Or Not KeyIsComment then
          begin
          If DoStripQuotes then
            begin
            J:=Length(s);
            // Joost, 2-jan-2007: The check (J>1) is there for the case that
            // the value consist of a single double-quote character. (see
            // mantis bug 6555)
            If (J>1) and ((s[1] in ['"','''']) and (s[J]=s[1])) then
               s:=Copy(s,2,J-2);
            end;
          if KeyIsComment then
            S:=K.Ident
          else if k.ident<>'' then
            s:=K.Ident+Separator+s;
          Strings.Add(s);
          end;
        end;
      end;
  finally
    Strings.EndUpdate;
  end;
end;

procedure TIniFile.DeleteSection(ASection : TIniFileSection);

begin
  FSectionList.Delete(FSectionList.IndexOf(ASection));
  ASection.Free;
end;

procedure TIniFile.MaybeDeleteSection(ASection: TIniFileSection);

begin
  If Asection.Empty then
    DeleteSection(ASection);
end;

procedure TIniFile.EraseSection(const Section: string);
var
  oSection: TIniFileSection;
begin
  oSection := FSectionList.SectionByName(Section,CaseSensitive);
  if oSection <> nil then begin
    { It is needed so UpdateFile doesn't find a defunct section }
    { and cause the program to crash }
    DeleteSection(OSection);
    MaybeUpdateFile;
  end;
end;

procedure TIniFile.DeleteKey(const Section, Ident: String);
var
 oSection: TIniFileSection;
 oKey: TIniFileKey;
begin
  oSection := FSectionList.SectionByName(Section,CaseSensitive);
  if oSection <> nil then
    begin
    oKey := oSection.KeyList.KeyByName(Ident,CaseSensitive);
    if oKey <> nil then
      begin
      oSection.KeyList.Delete(oSection.KeyList.IndexOf(oKey));
      oKey.Free;
      MaybeUpdateFile;
      end;
    end;
end;

procedure TIniFile.UpdateFile;
var
  slLines: TStringList;
  i, j: integer;
  D : String;
  
begin
  slLines := TStringList.Create;
  try
    for i := 0 to FSectionList.Count-1 do
      with FSectionList[i] do begin
        if IsComment(Name) then
          // comment
          slLines.Add(Name)
        else
          // regular section
          slLines.Add(Brackets[0] + Name + Brackets[1]);
        for j := 0 to KeyList.Count-1 do
          if IsComment(KeyList[j].Ident) then
            // comment
            slLines.Add(KeyList[j].Ident)
          else
            // regular key
            slLines.Add(KeyList[j].Ident + Separator + KeyList[j].Value);
        if (i < FSectionList.Count-1) and not IsComment(Name) then
          slLines.Add('');
      end;
    slLines.WriteBOM := FWriteBOM;
    if FFileName > '' then
      begin
      D:=ExtractFilePath(FFileName);
      If D <> '' Then
        if not ForceDirectories(D) then
          Raise EInoutError.CreateFmt(SErrCouldNotCreatePath,[D]);
      slLines.SaveToFile(FFileName, FEncoding);
      end
    else if FStream <> nil then
      begin
      Fstream.Size:=0;
      slLines.SaveToStream(FStream, FEncoding);
      end;
    FillSectionList(slLines);
    FDirty := false;
  finally
    slLines.Free;
  end;
end;

procedure TIniFile.MaybeUpdateFile;
begin
  If FCacheUpdates then
    FDirty:=True
  else
    UpdateFile;
end;

procedure TIniFile.ReadIniValues;
var
  slLines: TStringList;
begin
  FSectionList.Clear;

  if FileExists(FFileName) then
  begin
    slLines := TStringList.Create;
    try
      // read the ini file values
      if FEncoding<>nil then
        slLines.DefaultEncoding := FEncoding; // TStrings clones the encoding.
      slLines.Options := slLines.Options + [soPreserveBOM];
      slLines.LoadFromFile(FFileName, nil);
      if (FEncoding=nil) or (FEncoding.CodePage<>slLines.Encoding.CodePage) then
      begin
        if FOwnsEncoding then
          FEncoding.Free;
        if TEncoding.IsStandardEncoding(slLines.Encoding) then
        begin
          FEncoding := slLines.Encoding;
          FOwnsEncoding := False;
        end else
        begin
          FEncoding := slLines.Encoding.Clone;
          FOwnsEncoding := True;
        end;
      end;
      FWriteBOM := slLines.WriteBOM;
      FillSectionList(slLines);
    finally
      slLines.Free;
    end;
  end else
  begin
    FWriteBOM := Assigned(FEncoding) and ((FEncoding.CodePage=CP_UTF16) or (FEncoding.CodePage=CP_UTF16BE)); // write BOM for UTF16 by default
  end;
end;

{ TMemIniFile }

constructor TMemIniFile.Create(const AFileName: string; AOptions: TIniFileoptions);
begin
  inherited;
  FCacheUpdates:=True;
end;

constructor TMemIniFile.Create(const AFileName: string; AEscapeLineFeeds : Boolean);

begin
  Inherited;
  FCacheUpdates:=True;
end;

procedure TMemIniFile.Clear;
begin
  FSectionList.Clear;
end;

procedure TMemIniFile.GetStrings(List: TStrings);
var
  i, j: integer;
  oSection: TIniFileSection;
begin
  List.BeginUpdate;
  try
    for i := 0 to FSectionList.Count-1 do begin
      oSection := FSectionList[i];
      with oSection do begin
        if IsComment(Name) then
          List.Add(Name)
        else
          List.Add(Brackets[0] + Name + Brackets[1]);
        for j := 0 to KeyList.Count-1 do begin
          if IsComment(KeyList[j].Ident) then
            List.Add(KeyList[j].Ident)
          else
            List.Add(KeyList[j].Ident + Separator + KeyList[j].Value);
        end;
      end;
      if i < FSectionList.Count-1 then
        List.Add('');
    end;
  finally
    List.EndUpdate;
  end;
end;

procedure TMemIniFile.Rename(const AFileName: string; Reload: Boolean);
begin
  FFileName := AFileName;
  FStream := nil;
  if Reload then
    ReadIniValues;
end;

procedure TMemIniFile.SetStrings(List: TStrings);
begin
  FillSectionList(List);
end;

end.
