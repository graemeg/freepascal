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
<<<<<<< HEAD
<<<<<<< HEAD

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

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
    constructor Create;
>>>>>>> graemeg/fixes_2_2
=======
    constructor Create;
>>>>>>> origin/fixes_2_2
    destructor Destroy; override;
    function IndexOf(const S: String): Integer; override;
    function IndexOfName(const Name: String): Integer; override;
  end;

  TIniFileKey = class
  Private
    FIdent: string;
    FValue: string;
  public
<<<<<<< HEAD
<<<<<<< HEAD
    constructor Create(const AIdent, AValue: string);
=======
    constructor Create(AIdent, AValue: string);
>>>>>>> graemeg/fixes_2_2
=======
    constructor Create(AIdent, AValue: string);
>>>>>>> origin/fixes_2_2
    property Ident: string read FIdent write FIdent;
    property Value: string read FValue write FValue;
  end;

  TIniFileKeyList = class(TList)
  private
    function GetItem(Index: integer): TIniFileKey;
<<<<<<< HEAD
<<<<<<< HEAD
    function KeyByName(const AName: string; CaseSensitive : Boolean): TIniFileKey;
=======
    function KeyByName(AName: string; CaseSensitive : Boolean): TIniFileKey;
>>>>>>> graemeg/fixes_2_2
=======
    function KeyByName(AName: string; CaseSensitive : Boolean): TIniFileKey;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    constructor Create(const AName: string);
=======
    constructor Create(AName: string);
>>>>>>> graemeg/fixes_2_2
=======
    constructor Create(AName: string);
>>>>>>> origin/fixes_2_2
    destructor Destroy; override;
    property Name: string read FName;
    property KeyList: TIniFileKeyList read FKeyList;
  end;

  TIniFileSectionList = class(TList)
  private
    function GetItem(Index: integer): TIniFileSection;
<<<<<<< HEAD
<<<<<<< HEAD
    function SectionByName(const AName: string; CaseSensitive : Boolean): TIniFileSection;
=======
    function SectionByName(AName: string; CaseSensitive : Boolean): TIniFileSection;
>>>>>>> graemeg/fixes_2_2
=======
    function SectionByName(AName: string; CaseSensitive : Boolean): TIniFileSection;
>>>>>>> origin/fixes_2_2
  public
    destructor Destroy; override;
    procedure Clear;override;
    property Items[Index: integer]: TIniFileSection read GetItem; default;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  { TCustomIniFile }

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  TCustomIniFile = class
  Private
    FFileName: string;
    FSectionList: TIniFileSectionList;
    FEscapeLineFeeds: boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    FCaseSensitive : Boolean;
    FStripQuotes : Boolean;
    FFormatSettingsActive: Boolean;
  public
    FormatSettings: TFormatSettings;
=======
    FCaseSensitive : Boolean; 
    FStripQuotes : Boolean;
  public
>>>>>>> graemeg/fixes_2_2
=======
    FCaseSensitive : Boolean; 
    FStripQuotes : Boolean;
  public
>>>>>>> origin/fixes_2_2
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean = False); virtual;
    destructor Destroy; override;
    function SectionExists(const Section: string): Boolean; virtual;
    function ReadString(const Section, Ident, Default: string): string; virtual; abstract;
    procedure WriteString(const Section, Ident, Value: String); virtual; abstract;
    function ReadInteger(const Section, Ident: string; Default: Longint): Longint; virtual;
    procedure WriteInteger(const Section, Ident: string; Value: Longint); virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    function ReadInt64(const Section, Ident: string; Default: Int64): Longint; virtual;
    procedure WriteInt64(const Section, Ident: string; Value: Int64); virtual;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
    procedure ReadSectionValues(const Section: string; Strings: TStrings); virtual; abstract;
    procedure EraseSection(const Section: string); virtual; abstract;
    procedure DeleteKey(const Section, Ident: String); virtual; abstract;
    procedure UpdateFile; virtual; abstract;
    function ValueExists(const Section, Ident: string): Boolean; virtual;
    property FileName: string read FFileName;
    property EscapeLineFeeds: boolean read FEscapeLineFeeds;
    Property CaseSensitive : Boolean Read FCaseSensitive Write FCaseSensitive;
    Property StripQuotes : Boolean Read FStripQuotes Write FStripQuotes;
<<<<<<< HEAD
<<<<<<< HEAD
    Property FormatSettingsActive: Boolean Read FFormatSettingsActive write FFormatSettingsActive;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  { TIniFile }

  TIniFile = class(TCustomIniFile)
  Private
    FStream: TStream;
    FCacheUpdates: Boolean;
    FDirty : Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    FBOM : String;
    procedure FillSectionList(AStrings: TStrings);
    Procedure DeleteSection(ASection : TIniFileSection);
    Procedure MaybeDeleteSection(ASection : TIniFileSection);
    procedure SetCacheUpdates(const AValue: Boolean);
=======
    procedure FillSectionList(AStrings: TStrings);
    Procedure DeleteSection(ASection : TIniFileSection);
    Procedure MaybeDeleteSection(ASection : TIniFileSection);
>>>>>>> graemeg/fixes_2_2
=======
    procedure FillSectionList(AStrings: TStrings);
    Procedure DeleteSection(ASection : TIniFileSection);
    Procedure MaybeDeleteSection(ASection : TIniFileSection);
>>>>>>> origin/fixes_2_2
  protected
    procedure MaybeUpdateFile;
    property Dirty : Boolean Read FDirty;
  public
<<<<<<< HEAD
<<<<<<< HEAD
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean = False); overload; override;
    constructor Create(AStream: TStream; AEscapeLineFeeds : Boolean = False); overload;
=======
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean = False); override;
    constructor Create(AStream: TStream; AEscapeLineFeeds : Boolean = False);
>>>>>>> graemeg/fixes_2_2
=======
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean = False); override;
    constructor Create(AStream: TStream; AEscapeLineFeeds : Boolean = False);
>>>>>>> origin/fixes_2_2
    destructor Destroy; override;
    function ReadString(const Section, Ident, Default: string): string; override;
    procedure WriteString(const Section, Ident, Value: String); override;
    procedure ReadSection(const Section: string; Strings: TStrings); override;
    procedure ReadSectionRaw(const Section: string; Strings: TStrings);
    procedure ReadSections(Strings: TStrings); override;
    procedure ReadSectionValues(const Section: string; Strings: TStrings); override;
    procedure EraseSection(const Section: string); override;
    procedure DeleteKey(const Section, Ident: String); override;
    procedure UpdateFile; override;
    property Stream: TStream read FStream;
<<<<<<< HEAD
<<<<<<< HEAD
    property CacheUpdates : Boolean read FCacheUpdates write SetCacheUpdates;
=======
    property CacheUpdates : Boolean Read FCacheUpdates Write FCacheUpdates;
>>>>>>> graemeg/fixes_2_2
=======
    property CacheUpdates : Boolean Read FCacheUpdates Write FCacheUpdates;
>>>>>>> origin/fixes_2_2
  end;

  TMemIniFile = class(TIniFile)
  public
<<<<<<< HEAD
<<<<<<< HEAD
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean = False); overload; override;
=======
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean = False); override;
>>>>>>> graemeg/fixes_2_2
=======
    constructor Create(const AFileName: string; AEscapeLineFeeds : Boolean = False); override;
>>>>>>> origin/fixes_2_2
    procedure Clear;
    procedure GetStrings(List: TStrings);
    procedure Rename(const AFileName: string; Reload: Boolean);
    procedure SetStrings(List: TStrings);
  end;

implementation

<<<<<<< HEAD
<<<<<<< HEAD
Resourcestring
  SErrCouldNotCreatePath = 'Could not create directory "%s"';

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
function IsComment(const AString: string): boolean;
=======
function IsComment(AString: string): boolean;
>>>>>>> graemeg/fixes_2_2
=======
function IsComment(AString: string): boolean;
>>>>>>> origin/fixes_2_2
begin
  Result := False;
  if AString > '' then
    Result := (Copy(AString, 1, 1) = Comment);
end;

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
{ THashedStringList }

constructor THashedStringList.Create;
begin
  inherited;
  FValueHash := nil;
  FNameHash := nil;
  FValueHashValid := False;
  FNameHashValid := False;
end;

destructor THashedStringList.Destroy;
begin
  if Assigned(FValueHash) then
    FValueHash.Free;
  if Assigned(FNameHash) then
    FNameHash.Free;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  inherited Destroy;
end;

function THashedStringList.IndexOf(const S: String): Integer;
var
  I: Integer;
begin
  if not FValueHashValid then
    UpdateValueHash;

  I := FValueHash.FindIndexOf(S);
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

  I := FNameHash.FindIndexOf(Name);
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
    FValueHash.Add(Strings[I], Pointer(I+1));
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
    FNameHash.Add(Names[I], Pointer(I+1));
  FNameHashValid := True;
end;

{ TIniFileKey }

<<<<<<< HEAD
<<<<<<< HEAD
constructor TIniFileKey.Create(const AIdent, AValue: string);
=======
constructor TIniFileKey.Create(AIdent, AValue: string);
>>>>>>> graemeg/fixes_2_2
=======
constructor TIniFileKey.Create(AIdent, AValue: string);
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
function TIniFileKeyList.KeyByName(const AName: string; CaseSensitive : Boolean): TIniFileKey;
=======
function TIniFileKeyList.KeyByName(AName: string; CaseSensitive : Boolean): TIniFileKey;
>>>>>>> graemeg/fixes_2_2
=======
function TIniFileKeyList.KeyByName(AName: string; CaseSensitive : Boolean): TIniFileKey;
>>>>>>> origin/fixes_2_2
var
  i: integer;
begin
  Result := nil;
  if (AName > '') and not IsComment(AName) then
    If CaseSensitive then
      begin
      for i := 0 to Count-1 do
<<<<<<< HEAD
<<<<<<< HEAD
        if Items[i].Ident=AName then
=======
        if Items[i].Ident=AName then 
>>>>>>> graemeg/fixes_2_2
=======
        if Items[i].Ident=AName then 
>>>>>>> origin/fixes_2_2
          begin
          Result := Items[i];
          Break;
          end;
      end
<<<<<<< HEAD
<<<<<<< HEAD
    else
=======
    else  
>>>>>>> graemeg/fixes_2_2
=======
    else  
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
constructor TIniFileSection.Create(const AName: string);
=======
constructor TIniFileSection.Create(AName: string);
>>>>>>> graemeg/fixes_2_2
=======
constructor TIniFileSection.Create(AName: string);
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
function TIniFileSectionList.SectionByName(const AName: string; CaseSensitive : Boolean): TIniFileSection;
=======
function TIniFileSectionList.SectionByName(AName: string; CaseSensitive : Boolean): TIniFileSection;
>>>>>>> graemeg/fixes_2_2
=======
function TIniFileSectionList.SectionByName(AName: string; CaseSensitive : Boolean): TIniFileSection;
>>>>>>> origin/fixes_2_2
var
  i: integer;
begin
  Result := nil;
  if (AName > '') and not IsComment(AName) then
    If CaseSensitive then
      begin
      for i:=0 to Count-1 do
<<<<<<< HEAD
<<<<<<< HEAD
        if (Items[i].Name=AName) then
=======
        if (Items[i].Name=AName) then 
>>>>>>> graemeg/fixes_2_2
=======
        if (Items[i].Name=AName) then 
>>>>>>> origin/fixes_2_2
          begin
          Result := Items[i];
          Break;
          end;
      end
    else
      for i := 0 to Count-1 do
<<<<<<< HEAD
<<<<<<< HEAD
        if CompareText(Items[i].Name, AName) = 0 then
=======
        if CompareText(Items[i].Name, AName) = 0 then 
>>>>>>> graemeg/fixes_2_2
=======
        if CompareText(Items[i].Name, AName) = 0 then 
>>>>>>> origin/fixes_2_2
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

constructor TCustomIniFile.Create(const AFileName: string; AEscapeLineFeeds : Boolean = False);
begin
  FFileName := AFileName;
  FSectionList := TIniFileSectionList.Create;
  FEscapeLineFeeds := AEscapeLineFeeds;
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

destructor TCustomIniFile.Destroy;
begin
  FSectionList.Free;
  inherited Destroy;
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

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomIniFile.ReadInt64(const Section, Ident: string; Default: Int64
  ): Longint;
begin
  Result := StrToInt64Def(ReadString(Section, Ident, ''), Default);
end;

procedure TCustomIniFile.WriteInt64(const Section, Ident: string; Value: Int64);
begin
  WriteString(Section, Ident, IntToStr(Value));
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TCustomIniFile.ReadBool(const Section, Ident: string; Default: Boolean): Boolean;
var
  s: string;
begin
  Result := Default;
  s := ReadString(Section, Ident, '');
  if s > '' then
    Result := CharToBool(s[1]);
end;

procedure TCustomIniFile.WriteBool(const Section, Ident: string; Value: Boolean);
begin
  WriteString(Section, Ident, BoolToChar(Value));
end;

function TCustomIniFile.ReadDate(const Section, Ident: string; Default: TDateTime): TDateTime;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FFormatSettingsActive then begin
    if not TryStrToDate(ReadString(Section, Ident, ''), Result, FormatSettings) then
      Result := Default;
  end else
    Result := StrToDateDef(ReadString(Section, Ident, ''),Default);
=======
  Result := StrToDateDef(ReadString(Section, Ident, ''),Default);
>>>>>>> graemeg/fixes_2_2
=======
  Result := StrToDateDef(ReadString(Section, Ident, ''),Default);
>>>>>>> origin/fixes_2_2
end;

function TCustomIniFile.ReadDateTime(const Section, Ident: string; Default: TDateTime): TDateTime;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FFormatSettingsActive then begin
    if not TryStrToDateTime(ReadString(Section, Ident, ''), Result, FormatSettings) then
      Result := Default;
  end else
    Result := StrToDateTimeDef(ReadString(Section, Ident, ''),Default);
=======
  Result := StrToDateTimeDef(ReadString(Section, Ident, ''),Default);
>>>>>>> graemeg/fixes_2_2
=======
  Result := StrToDateTimeDef(ReadString(Section, Ident, ''),Default);
>>>>>>> origin/fixes_2_2
end;

function TCustomIniFile.ReadFloat(const Section, Ident: string; Default: Double): Double;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FFormatSettingsActive then
    Result:=StrToFloatDef(ReadString(Section, Ident, ''),Default, FormatSettings)
  else
    Result:=StrToFloatDef(ReadString(Section, Ident, ''),Default);
=======
  Result:=StrToFloatDef(ReadString(Section, Ident, ''),Default);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=StrToFloatDef(ReadString(Section, Ident, ''),Default);
>>>>>>> origin/fixes_2_2
end;

function TCustomIniFile.ReadTime(const Section, Ident: string; Default: TDateTime): TDateTime;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FFormatSettingsActive then
    Result := StrToTimeDef(ReadString(Section, Ident, ''),Default, FormatSettings.TimeSeparator)
  else
    Result := StrToTimeDef(ReadString(Section, Ident, ''),Default);
=======
  Result := StrToTimeDef(ReadString(Section, Ident, ''),Default);
>>>>>>> graemeg/fixes_2_2
=======
  Result := StrToTimeDef(ReadString(Section, Ident, ''),Default);
>>>>>>> origin/fixes_2_2
end;

procedure TCustomIniFile.WriteDate(const Section, Ident: string; Value: TDateTime);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FFormatSettingsActive then
    WriteString(Section, Ident, DateToStr(Value, FormatSettings))
  else
    WriteString(Section, Ident, DateToStr(Value));
=======
  WriteString(Section, Ident, DateToStr(Value));
>>>>>>> graemeg/fixes_2_2
=======
  WriteString(Section, Ident, DateToStr(Value));
>>>>>>> origin/fixes_2_2
end;

procedure TCustomIniFile.WriteDateTime(const Section, Ident: string; Value: TDateTime);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FFormatSettingsActive then
    WriteString(Section, Ident, DateTimeToStr(Value, FormatSettings))
  else
    WriteString(Section, Ident, DateTimeToStr(Value));
=======
  WriteString(Section, Ident, DateTimeToStr(Value));
>>>>>>> graemeg/fixes_2_2
=======
  WriteString(Section, Ident, DateTimeToStr(Value));
>>>>>>> origin/fixes_2_2
end;

procedure TCustomIniFile.WriteFloat(const Section, Ident: string; Value: Double);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FFormatSettingsActive then
    WriteString(Section, Ident, FloatToStr(Value, FormatSettings))
  else
    WriteString(Section, Ident, FloatToStr(Value));
=======
  WriteString(Section, Ident, FloatToStr(Value));
>>>>>>> graemeg/fixes_2_2
=======
  WriteString(Section, Ident, FloatToStr(Value));
>>>>>>> origin/fixes_2_2
end;

procedure TCustomIniFile.WriteTime(const Section, Ident: string; Value: TDateTime);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FFormatSettingsActive then
    WriteString(Section, Ident, TimeToStr(Value, FormatSettings))
  else
    WriteString(Section, Ident, TimeToStr(Value));
=======
  WriteString(Section, Ident, TimeToStr(Value));
>>>>>>> graemeg/fixes_2_2
=======
  WriteString(Section, Ident, TimeToStr(Value));
>>>>>>> origin/fixes_2_2
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

procedure TCustomInifile.WriteBinaryStream(const Section, Name: string; Value: TStream);


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

{ TIniFile }

constructor TIniFile.Create(const AFileName: string; AEscapeLineFeeds : Boolean = False);
var
  slLines: TStringList;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FBOM := '';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  If Not (self is TMemIniFile) then
    StripQuotes:=True;
  inherited Create(AFileName,AEscapeLineFeeds);
  FStream := nil;
  slLines := TStringList.Create;
  try
<<<<<<< HEAD
<<<<<<< HEAD
    if FileExists(FFileName) then
=======
    if FileExists(FFileName) then 
>>>>>>> graemeg/fixes_2_2
=======
    if FileExists(FFileName) then 
>>>>>>> origin/fixes_2_2
      begin
      // read the ini file values
      slLines.LoadFromFile(FFileName);
      FillSectionList(slLines);
<<<<<<< HEAD
<<<<<<< HEAD
      end
=======
      end 
>>>>>>> graemeg/fixes_2_2
=======
      end 
>>>>>>> origin/fixes_2_2
  finally
    slLines.Free;
  end;
end;

constructor TIniFile.Create(AStream: TStream; AEscapeLineFeeds : Boolean = False);
var
  slLines: TStringList;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FBOM := '';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  inherited Create('',AEscapeLineFeeds);
  FStream := AStream;
  slLines := TStringList.Create;
  try
    // read the ini file values
    slLines.LoadFromStream(FStream);
    FillSectionList(slLines);
  finally
    slLines.Free;
  end;
end;

destructor TIniFile.destroy;
begin
  If FDirty and FCacheUpdates then
<<<<<<< HEAD
<<<<<<< HEAD
    try
      UpdateFile;
    except
      // Eat exception. Compatible to D7 behaviour, see comments to bug 19046
    end;  
=======
    UpdateFile;
>>>>>>> graemeg/fixes_2_2
=======
    UpdateFile;
>>>>>>> origin/fixes_2_2
  inherited destroy;
end;

procedure TIniFile.FillSectionList(AStrings: TStrings);
<<<<<<< HEAD
<<<<<<< HEAD
const
  Utf8Bom    = #$EF#$BB#$BF;        { Die einzelnen BOM Typen }

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  i,j: integer;
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

begin
  oSection := nil;
  FSectionList.Clear;
  if FEscapeLineFeeds then
    RemoveBackslashes;
<<<<<<< HEAD
<<<<<<< HEAD
  if (AStrings.Count > 0) and (copy(AStrings.Strings[0],1,Length(Utf8Bom)) = Utf8Bom) then
  begin
    FBOM := Utf8Bom;
    AStrings.Strings[0] := copy(AStrings.Strings[0],Length(Utf8Bom)+1,Length(AStrings.Strings[0]));
  end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  for i := 0 to AStrings.Count-1 do begin
    sLine := Trim(AStrings[i]);
    if sLine > '' then
      begin
      if IsComment(sLine) and (oSection = nil) then begin
        // comment at the beginning of the ini file
        oSection := TIniFileSection.Create(sLine);
        FSectionList.Add(oSection);
        continue;
      end;
      if (Copy(sLine, 1, 1) = Brackets[0]) and (Copy(sLine, length(sLine), 1) = Brackets[1]) then begin
        // regular section
        oSection := TIniFileSection.Create(Copy(sLine, 2, Length(sLine) - 2));
        FSectionList.Add(oSection);
      end else if oSection <> nil then begin
        if IsComment(sLine) then begin
          // comment within a section
          sIdent := sLine;
          sValue := '';
        end else begin
          // regular key
          j:=Pos(Separator, sLine);
          if j=0 then
           begin
             sIdent:='';
             sValue:=sLine
           end
          else
           begin
             sIdent:=Trim(Copy(sLine, 1,  j - 1));
             sValue:=Trim(Copy(sLine, j + 1, Length(sLine) - j));
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
             If StripQuotes then
               begin
               J:=Length(sValue);
               // Joost, 2-jan-2007: The check (J>1) is there for the case that
               // the value consist of a single double-quote character. (see
               // mantis bug 6555)
               If (J>1) and ((sValue[1] in ['"','''']) and (sValue[J]=sValue[1])) then
                 sValue:=Copy(sValue,2,J-2);
               end;  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           end;
        end;
        oSection.KeyList.Add(TIniFileKey.Create(sIdent, sValue));
      end;
      end;
  end;
end;

function TIniFile.ReadString(const Section, Ident, Default: string): string;
var
  oSection: TIniFileSection;
  oKey: TIniFileKey;
<<<<<<< HEAD
<<<<<<< HEAD
  J: integer;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Result := Default;
  oSection := FSectionList.SectionByName(Section,CaseSensitive);
  if oSection <> nil then begin
    oKey := oSection.KeyList.KeyByName(Ident,CaseSensitive);
    if oKey <> nil then
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
      Result := oKey.Value;
  end;
>>>>>>> graemeg/fixes_2_2
=======
      Result := oKey.Value;
  end;
>>>>>>> origin/fixes_2_2
end;

procedure TIniFile.WriteString(const Section, Ident, Value: String);
var
  oSection: TIniFileSection;
  oKey: TIniFileKey;
begin
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
  if (Section > '') and (Ident > '') then begin
    // update or add key
    oSection := FSectionList.SectionByName(Section,CaseSensitive);
    if (Value > '') then begin
      if oSection = nil then begin
        oSection := TIniFileSection.Create(Section);
        FSectionList.Add(oSection);
      end;
      with oSection.KeyList do begin
        oKey := KeyByName(Ident,CaseSensitive);
        if oKey <> nil then
          oKey.Value := Value
        else
          oSection.KeyList.Add(TIniFileKey.Create(Ident, Value));
      end;
    end else if oSection <> nil then begin
      // remove key
      oKey := oSection.KeyList.KeyByName(Ident,CaseSensitive);
      if oKey <> nil then begin
        oSection.KeyList.Remove(oKey);
      end;
    end;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
        if not IsComment(Items[i].Ident) then
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

procedure TIniFile.ReadSectionValues(const Section: string; Strings: TStrings);
var
  oSection: TIniFileSection;
  s: string;
<<<<<<< HEAD
<<<<<<< HEAD
  i,J: integer;
=======
  i: integer;
>>>>>>> graemeg/fixes_2_2
=======
  i: integer;
>>>>>>> origin/fixes_2_2
begin
  Strings.BeginUpdate;
  try
    Strings.Clear;
    oSection := FSectionList.SectionByName(Section,CaseSensitive);
    if oSection <> nil then with oSection.KeyList do
      for i := 0 to Count-1 do begin
<<<<<<< HEAD
<<<<<<< HEAD
        s := Items[i].Value;
      If StripQuotes then
        begin
          J:=Length(s);
          // Joost, 2-jan-2007: The check (J>1) is there for the case that
          // the value consist of a single double-quote character. (see
          // mantis bug 6555)
          If (J>1) and ((s[1] in ['"','''']) and (s[J]=s[1])) then
             s:=Copy(s,2,J-2);
        end;
        if Items[i].Ident<>'' then
          s:=Items[i].Ident+Separator+s;
=======
        s := Items[i].Ident+Separator+Items[i].Value;
>>>>>>> graemeg/fixes_2_2
=======
        s := Items[i].Ident+Separator+Items[i].Value;
>>>>>>> origin/fixes_2_2
        Strings.Add(s);
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

Procedure TIniFile.MaybeDeleteSection(ASection : TIniFileSection);

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
<<<<<<< HEAD
<<<<<<< HEAD
  if oSection <> nil then
    begin
    oKey := oSection.KeyList.KeyByName(Ident,CaseSensitive);
    if oKey <> nil then
      begin
      oSection.KeyList.Delete(oSection.KeyList.IndexOf(oKey));
      oKey.Free;
=======
=======
>>>>>>> origin/fixes_2_2
  if oSection <> nil then 
    begin
    oKey := oSection.KeyList.KeyByName(Ident,CaseSensitive);
    if oKey <> nil then 
      begin
      oSection.KeyList.Delete(oSection.KeyList.IndexOf(oKey));
      oKey.Free;
      MaybeDeleteSection(oSection);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      MaybeUpdateFile;
      end;
    end;
end;

procedure TIniFile.UpdateFile;
var
  slLines: TStringList;
  i, j: integer;
<<<<<<< HEAD
<<<<<<< HEAD
  D : String;
  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
    if slLines.Count > 0 then
      slLines.Strings[0] := FBOM + slLines.Strings[0];
    if FFileName > '' then
      begin
      D:=ExtractFilePath(FFileName);
      If D <> '' Then
        if not ForceDirectories(D) then
          Raise EInoutError.CreateFmt(SErrCouldNotCreatePath,[D]);
      slLines.SaveToFile(FFileName);
      end
    else if FStream <> nil then
      begin
      Fstream.Size:=0;
      slLines.SaveToStream(FStream);
      end;
    FillSectionList(slLines);
    FDirty := false;
=======
=======
>>>>>>> origin/fixes_2_2
    if FFileName > '' then
      slLines.SaveToFile(FFileName)
    else if FStream <> nil then
      slLines.SaveToStream(FStream);
    FillSectionList(slLines);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

{ TMemIniFile }

constructor TMemIniFile.Create(const AFileName: string; AEscapeLineFeeds : Boolean = False);

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
var
  slLines: TStringList;
begin
  FFileName := AFileName;
  FStream := nil;
  if Reload then begin
    slLines := TStringList.Create;
    try
      slLines.LoadFromFile(FFileName);
      FillSectionList(slLines);
    finally
      slLines.Free;
    end;
  end;
end;

procedure TMemIniFile.SetStrings(List: TStrings);
begin
  FillSectionList(List);
end;

end.
