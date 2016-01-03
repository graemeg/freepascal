unit dbf_dbffile;

interface

{$I dbf_common.inc}

uses
  Classes, SysUtils,
{$ifdef WINDOWS}
  Windows,
{$else}
{$ifdef KYLIX}
  Libc, 
{$endif}  
  Types, dbf_wtil,
{$endif}
  Db,
  dbf_common,
  dbf_cursor,
  dbf_pgfile,
  dbf_fields,
  dbf_memo,
  dbf_idxfile;

//====================================================================
//=== Dbf support (first part)
//====================================================================
//  TxBaseVersion = (xUnknown,xClipper,xBaseIII,xBaseIV,xBaseV,xFoxPro,xVisualFoxPro);
//  TPagedFileMode = (pfOpen,pfCreate);
//  TDbfGetMode = (xFirst,xPrev,xCurrent, xNext, xLast);
//  TDbfGetResult = (xOK, xBOF, xEOF, xError);

type

//====================================================================
  TDbfIndexMissingEvent = procedure(var DeleteLink: Boolean) of object;
  TUpdateNullField = (unfClear, unfSet);
  TNullFieldFlag = (nfNullFlag, nfVarlengthFlag); //the field that the nullflags bit applies to

//====================================================================
  TDbfGlobals = class;
//====================================================================

  { TDbfFile }

  TDbfFile = class(TPagedFile)
  protected
    FBackLink: string;
    FBackLinkOffset: integer; //position of VFP backlink within header
    FMdxFile: TIndexFile;
    FMemoFile: TMemoFile;
    FMemoStream: TStream;
    FFieldDefs: TDbfFieldDefs;
    FIndexNames: TStringList;
    FIndexFiles: TList;
    FIndexStream: TStream;
    FDbfVersion: TXBaseVersion;
    FPrevBuffer: TRecordBuffer;
    FDefaultBuffer: TRecordBuffer;
    FRecordBufferSize: Integer;
    FLockUserLen: DWORD;
    FFileCodePage: Cardinal;
    FUseCodePage: Cardinal;
    FFileLangId: Byte;
    FCountUse: Integer;
    FCurIndex: Integer;
    FForceClose: Boolean;
    FLockField: TDbfFieldDef;
    FNullField: TDbfFieldDef;
    FAutoIncPresent: Boolean;
    FCopyDateTimeAsString: Boolean;
    FDateTimeHandling: TDateTimeHandling;
    FOnLocaleError: TDbfLocaleErrorEvent;
    FOnIndexMissing: TDbfIndexMissingEvent;
    // Yes if table has blob/memo type field(s) (storage in external file)
    function  HasBlob: Boolean;
    // File extension for memo field; uppercase if FFileName is uppercase
    // (useful for *nix case-sensitive filesystems)
    function  GetMemoExt: string;

    function GetLanguageId: Integer;
    function GetLanguageStr: string;

  protected
    // Reads the field's properties from the field header(s)
    procedure ConstructFieldDefs;
    procedure InitDefaultBuffer;
    // Shows if the (null or varlength) flag for AFieldDef is set.
    function IsNullFlagSet(const Src: Pointer; var AFieldDef: TDbfFieldDef; WhichField: TNullFieldFlag): boolean;
    // Updates _NULLFLAGS field with null or varlength flag for field
    procedure UpdateNullField(Buffer: Pointer; AFieldDef: TDbfFieldDef; Action: TUpdateNullField; WhichField: TNullFieldFlag);
    procedure WriteLockInfo(Buffer: TRecordBuffer);

  public
    constructor Create;
    destructor Destroy; override;

    procedure Open;
    procedure Close;
    procedure Zap;

    // Write out field definitions to header etc.
    procedure FinishCreate(AFieldDefs: TDbfFieldDefs; MemoSize: Integer);
    function GetIndexByName(AIndexName: string): TIndexFile;
    procedure SetRecordSize(NewSize: Integer); override;

    procedure TryExclusive; override;
    procedure EndExclusive; override;
    procedure OpenIndex(IndexName, IndexField: string; CreateIndex: Boolean; Options: TIndexOptions);
    function  DeleteIndex(const AIndexName: string): Boolean;
    procedure CloseIndex(AIndexName: string);
    procedure RepageIndex(AIndexFile: string);
    procedure CompactIndex(AIndexFile: string);

    // Inserts new record
    function  Insert(Buffer: TRecordBuffer): integer;
    // Write dbf header as well as EOF marker at end of file if necessary
    procedure WriteHeader; override;
    // Writes autoinc value to record buffer and updates autoinc value in field header
    procedure ApplyAutoIncToBuffer(DestBuf: TRecordBuffer);
    procedure FastPackTable;
    procedure RestructureTable(DbfFieldDefs: TDbfFieldDefs; Pack: Boolean);
    procedure Rename(DestFileName: string; NewIndexFileNames: TStrings; DeleteFiles: boolean);
    function  GetFieldInfo(FieldName: string): TDbfFieldDef;
    // Copies record buffer to field buffer
    // Returns true if not null & data succesfully copied; false if field is null
    function  GetFieldData(Column: Integer; DataType: TFieldType; Src,Dst: Pointer; 
      NativeFormat: boolean): Boolean;
    // Copies record buffer to field buffer
    // Returns true if not null & data succesfully copied; false if field is null
    function  GetFieldDataFromDef(AFieldDef: TDbfFieldDef; DataType: TFieldType; 
      Src, Dst: Pointer; NativeFormat: boolean): Boolean;
    // Copies field buffer to record buffer for this field
    procedure SetFieldData(Column: Integer; DataType: TFieldType; Src,Dst: Pointer; NativeFormat: boolean);
    // Fill DestBuf with default field data
    procedure InitRecord(DestBuf: TRecordBuffer);
    procedure PackIndex(lIndexFile: TIndexFile; AIndexName: string);
    procedure RegenerateIndexes;
    procedure LockRecord(RecNo: Integer; Buffer: TRecordBuffer);
    procedure UnlockRecord(RecNo: Integer; Buffer: TRecordBuffer);
    procedure RecordDeleted(RecNo: Integer; Buffer: TRecordBuffer);
    procedure RecordRecalled(RecNo: Integer; Buffer: TRecordBuffer);

    property MemoFile: TMemoFile read FMemoFile;
    // Backing stream for stream/memory-based memo "files"
    property MemoStream: TStream read FMemoStream write FMemoStream;
    property FieldDefs: TDbfFieldDefs read FFieldDefs;
    property IndexNames: TStringList read FIndexNames;
    property IndexFiles: TList read FIndexFiles;
    // Backing stream for stream/memory-based index "files"
    property IndexStream: TStream read FIndexStream write FIndexStream;
    property MdxFile: TIndexFile read FMdxFile;
    property LanguageId: Integer read GetLanguageId;
    property LanguageStr: string read GetLanguageStr;
    property FileCodePage: Cardinal read FFileCodePage;
    property UseCodePage: Cardinal read FUseCodePage write FUseCodePage;
    property FileLangId: Byte read FFileLangId write FFileLangId;
    // Visual Foxpro: relative path to .dbc database file containing
    // long field names and other metadata
    // Empty if this is a "free table", not linked to a .dbc file
    property BackLink: string read FBackLink write FBackLink;
    // Dbase (clone) version that this format emulates. Related to tablelevel.
    property DbfVersion: TXBaseVersion read FDbfVersion write FDbfVersion;
    property PrevBuffer: TRecordBuffer read FPrevBuffer;
    property ForceClose: Boolean read FForceClose;
    property CopyDateTimeAsString: Boolean read FCopyDateTimeAsString write FCopyDateTimeAsString;
    property DateTimeHandling: TDateTimeHandling read FDateTimeHandling write FDateTimeHandling;

    property OnIndexMissing: TDbfIndexMissingEvent read FOnIndexMissing write FOnIndexMissing;
    property OnLocaleError: TDbfLocaleErrorEvent read FOnLocaleError write FOnLocaleError;
  end;

//====================================================================
  TDbfCursor = class(TVirtualCursor)
  protected
    FPhysicalRecNo: Integer;
  public
    constructor Create(DbfFile: TDbfFile);
    function Next: Boolean; override;
    function Prev: Boolean; override;
    procedure First; override;
    procedure Last; override;

    function GetPhysicalRecNo: Integer; override;
    procedure SetPhysicalRecNo(RecNo: Integer); override;

    function GetSequentialRecordCount: Integer; override;
    function GetSequentialRecNo: Integer; override;
    procedure SetSequentialRecNo(RecNo: Integer); override;
  end;

//====================================================================

  { TDbfGlobals }

  TDbfGlobals = class
  protected
    FCodePages: TList;
    FCurrencyAsBCD: Boolean;
    FDefaultOpenCodePage: Integer;
    FDefaultCreateLangId: Byte;
    FUserName: string;
    FUserNameLen: DWORD;

    // Translates FDefaultCreateLangId back to codepage
    function  GetDefaultCreateCodePage: Integer;
    // Takes codepage and sets FDefaultCreateLangId
    procedure SetDefaultCreateCodePage(NewCodePage: Integer);
    procedure InitUserName;
  public
    constructor Create;
    destructor Destroy; override;

    function CodePageInstalled(ACodePage: Integer): Boolean;

    property CurrencyAsBCD: Boolean read FCurrencyAsBCD write FCurrencyAsBCD;
    property DefaultOpenCodePage: Integer read FDefaultOpenCodePage write FDefaultOpenCodePage;
    property DefaultCreateCodePage: Integer read GetDefaultCreateCodePage write SetDefaultCreateCodePage;
    property DefaultCreateLangId: Byte read FDefaultCreateLangId write FDefaultCreateLangId;
    property UserName: string read FUserName;
    property UserNameLen: DWORD read FUserNameLen;
  end;

var
  DbfGlobals: TDbfGlobals;

implementation

uses
{$ifndef WINDOWS}
 {$IFNDEF OS2}
  {$ifndef FPC}
  RTLConsts,
  {$else FPC}
  BaseUnix,
  {$endif FPC}
 {$ENDIF OS2}
{$endif WINDOWS}
{$ifdef SUPPORT_MATH_UNIT}
  Math,
{$endif}
  dbf_str, dbf_lang, dbf_prssupp, dbf_prsdef;

const
  sDBF_DEC_SEP = '.';
  FIELD_DESCRIPTOR_ARRAY_TERMINATOR = $0D; // Marker at end of list of fields within header
  NULLFLAGSFIELD = '_NULLFLAGS'; //Visual Foxpro system field with flags for field=null and field has varlength byte

{$I dbf_struct.inc}

//====================================================================
// International separator
// thanks to Bruno Depero from Italy
// and Andreas Wöllenstein from Denmark
//====================================================================
function DbfStrToFloat(const Src: PChar; const Size: Integer): Extended;
var
  iPos: PChar;
  eValue: extended;
  endChar: Char;
begin
  // temp null-term string
  endChar := (Src + Size)^;
  (Src + Size)^ := #0;
  // we only have to convert if decimal separator different
  if DecimalSeparator <> sDBF_DEC_SEP then
  begin
    // search dec sep
    iPos := StrScan(Src, sDBF_DEC_SEP);
    // replace
    if iPos <> nil then
      iPos^ := DecimalSeparator;
  end else
    iPos := nil;
  // convert to double
  if TextToFloat(Src, eValue {$ifndef VER1_0}, fvExtended{$endif}) then
    Result := eValue
  else
    Result := 0;
  // restore dec sep
  if iPos <> nil then
    iPos^ := sDBF_DEC_SEP;
  // restore Char of null-term
  (Src + Size)^ := endChar;
end;

procedure FloatToDbfStr(const Val: Extended; const Size, Precision: Integer; const Dest: PChar);
var
  Buffer: array [0..24] of Char;
  resLen: Integer;
  iPos: PChar;
begin
  // convert to temporary buffer
  resLen := FloatToText(@Buffer[0], Val, {$ifndef FPC_VERSION}fvExtended,{$endif} ffFixed, Size, Precision);
  // prevent overflow in destination buffer
  if resLen > Size then
    resLen := Size;
  // null-terminate buffer
  Buffer[resLen] := #0;
  // we only have to convert if decimal separator different
  if DecimalSeparator <> sDBF_DEC_SEP then
  begin
    iPos := StrScan(@Buffer[0], DecimalSeparator);
    if iPos <> nil then
      iPos^ := sDBF_DEC_SEP;
  end;
  // fill destination with spaces
  FillChar(Dest^, Size, ' ');
  // now copy right-aligned to destination
  Move(Buffer[0], Dest[Size-resLen], resLen);
end;

function GetIntFromStrLength(Src: Pointer; Size: Integer; Default: Integer): Integer;
var
  endChar: Char;
  Code: Integer;
begin
  // save Char at pos term. null
  endChar := (PChar(Src) + Size)^;
  (PChar(Src) + Size)^ := #0;
  // convert
  Val(PChar(Src), Result, Code);
  // check success
  if Code <> 0 then
    Result := Default;
  // restore prev. ending Char
  (PChar(Src) + Size)^ := endChar;
end;

//====================================================================
// TDbfFile
//====================================================================
constructor TDbfFile.Create;
begin
  // init variables first
  FBackLink := '';
  FBackLinkOffset := 0;
  FFieldDefs := TDbfFieldDefs.Create(nil);
  FIndexNames := TStringList.Create;
  FIndexFiles := TList.Create;

  // now initialize inherited
  inherited;
end;

destructor TDbfFile.Destroy;
var
  I: Integer;
begin
  // close file
  Close;

  // free files
  for I := 0 to Pred(FIndexFiles.Count) do
    TPagedFile(FIndexFiles.Items[I]).Free;

  // free lists
  FreeAndNil(FIndexFiles);
  FreeAndNil(FIndexNames);
  FreeAndNil(FFieldDefs);

  // call ancestor
  inherited;
end;

procedure TDbfFile.Open;
var
  lMemoFileName: string;
  lMdxFileName: string;
  MemoFileClass: TMemoFileClass;
  I: Integer;
  deleteLink: Boolean;
  lModified: boolean;

  procedure GetVersion;
  var
    version: byte;
  begin
    // OH 2000-11-15 dBase7 support. I built dBase Tables with different
    // BDE dBase Level (1. without Memo, 2. with Memo)
    //                          Header Byte ($1d hex) (29 dec) -> Language driver ID.
    //  $03,$83 xBaseIII        Header Byte $1d=$00, Float -> N($13.$04) DateTime C($1E)
    //  $03,$8B xBaseIV/V       Header Byte $1d=$58, Float -> N($14.$04)
    //  $04,$8C xBaseVII        Header Byte $1d=$00  Float -> O($08)     DateTime @($08)
    //  $03,$F5 FoxPro Level 25 Header Byte $1d=$03, Float -> N($14.$04)
    // Access 97
    //  $03,$83 dBaseIII        Header Byte $1d=$00, Float -> N($13.$05) DateTime D($08)
    //  $03,$8B dBaseIV/V       Header Byte $1d=$00, Float -> N($14.$05) DateTime D($08)
    //  $03,$F5 FoxPro Level 25 Header Byte $1d=$09, Float -> N($14.$05) DateTime D($08)

    version := PDbfHdr(Header)^.VerDBF;
    FDbfVersion := xUnknown;
    // Some hardcoded versions for Visual FoxPro; see MS documentation
    // (including the correction at the bottom):
    // http://msdn.microsoft.com/en-US/library/st4a0s68%28v=vs.80%29.aspx
    case version of
      $30, $31, $32 {VFP9 with new data types}: FDbfVersion:=xVisualFoxPro;
      $F5, $FB: FDbfVersion:=xFoxPro;
    end;
    if FDbfVersion = xUnknown then
      case (version and $07) of
        $03: //dbf with/without memo. Could be Foxpro, too
          if not(version in [$03,$8B]) {e.g. dbase IV < v2.0 with 0 language ID} and
            (LanguageID = 0) then
            FDbfVersion := xBaseIII
          else
            FDbfVersion := xBaseIV;
        $04:
          FDbfVersion := xBaseVII;
        $02 {FoxBase, not readable by current MS Visual FoxPro driver}, $05:
          FDbfVersion := xFoxPro;
      else
        begin
          // not a valid DBF file
          raise EDbfError.Create(STRING_INVALID_DBF_FILE);
        end;
      end;
    FFieldDefs.DbfVersion := FDbfVersion;
  end;

  procedure GetCodePage;
  var
    LangStr: PChar;
  begin
    // determine codepage
    case FDbfVersion of
      xBaseVII:
      begin
        // cache language str
        LangStr := @PEndFixedHdrVII(PChar(Header) + SizeOf(rDbfHdr))^.LanguageDriverName;
        // VdBase 7 Language strings
        //  'DBWIN...' -> Charset 1252 (ansi)
        //  'DB999...' -> Code page 999, 9 any digit
        //  'DBHEBREW' -> Code page 1255 ??
        //  'FOX..999' -> Code page 999, 9 any digit
        //  'FOX..WIN' -> Charset 1252 (ansi)
        if (LangStr[0] = 'D') and (LangStr[1] = 'B') then
        begin
          if StrLComp(LangStr+2, 'WIN', 3) = 0 then
            FFileCodePage := 1252
          else
          if StrLComp(LangStr+2, 'HEBREW', 6) = 0 then
          begin
            FFileCodePage := 1255;
          end else begin
            FFileCodePage := GetIntFromStrLength(LangStr+2, 3, 0);
            if (Ord(LangStr[5]) >= Ord('0')) and (Ord(LangStr[5]) <= Ord('9')) then
              FFileCodePage := FFileCodePage * 10 + Ord(LangStr[5]) - Ord('0');
          end;
        end else
        if StrLComp(LangStr, 'FOX', 3) = 0 then
        begin
          if StrLComp(LangStr+5, 'WIN', 3) = 0 then
            FFileCodePage := 1252
          else
            FFileCodePage := GetIntFromStrLength(LangStr+5, 3, 0)
        end else begin
          FFileCodePage := 0;
        end;
        FFileLangId := GetLangId_From_LangName(LanguageStr);
      end;
    else
      begin
        // DBase II..V, FoxPro, Visual FoxPro
        FFileLangId := PDbfHdr(Header)^.Language;
        FFileCodePage := LangId_To_CodePage[FFileLangId];
      end;
    end;
    // determine used codepage, if no codepage, then use default codepage
    FUseCodePage := FFileCodePage;
    if FUseCodePage = 0 then
      FUseCodePage := DbfGlobals.DefaultOpenCodePage;
  end;

  procedure GetBackLink;
  // Gets backlink info - only supported in Visual Foxpro
  begin
    FBackLink:='';
    if FDBFVersion=xVisualFoxPro then //only format that supports it
    begin
      FBackLink:= StrPas(@PEndHdrVFP(PChar(Header) + FBackLinkOffset)^.Backlink);
    end;
  end;

begin
  // check if not already opened
  if not Active then
  begin
    // open requested file
    OpenFile;

    // check if we opened an already existing file
    lModified := false;
    if not FileCreated then
    begin
      HeaderSize := sizeof(rDbfHdr); // temporary, required for getting version
      GetVersion;

      RecordSize := PDbfHdr(Header)^.RecordSize;
      HeaderSize := PDbfHdr(Header)^.FullHdrSize;
      if (HeaderSize = 0) or (RecordSize = 0) then
      begin
        HeaderSize := 0;
        RecordSize := 0;
        RecordCount := 0;
        FForceClose := true;
        exit;
      end;

      // check if specified recordcount is right; correct if not
      if PDbfHdr(Header)^.RecordCount <> RecordCount then
      begin
        PDbfHdr(Header)^.RecordCount := RecordCount;
        lModified := true;
      end;

      GetCodePage;
      // get list of fields
      ConstructFieldDefs;
      GetBackLink;

      // open blob file if present
      lMemoFileName := ChangeFileExt(FileName, GetMemoExt);
      if HasBlob then
      begin
        // open blob file; if it doesn't exist yet create it
        // using AutoCreate as long as we're not running read-only
        // If needed, fake a memo file:
        if (Mode=pfReadOnly) and (not FileExists(lMemoFileName)) then
          MemoFileClass := TNullMemoFile
        else if (FDbfVersion in [xFoxPro,xVisualFoxPro]) then
          MemoFileClass := TFoxProMemoFile
        else
          MemoFileClass := TDbaseMemoFile; //fallback/default
        FMemoFile := MemoFileClass.Create(Self);
        FMemoFile.FileName := lMemoFileName;
        if (Mode in [pfMemoryOpen,pfMemoryCreate]) then
          FMemoFile.Stream:=FMemoStream;
        FMemoFile.Mode := Mode;
        FMemoFile.AutoCreate := true;
        FMemoFile.MemoRecordSize := 0;
        FMemoFile.DbfVersion := FDbfVersion;
        FMemoFile.Open;
        // set header blob flag corresponding to field list
        if not(FDbfVersion in [xFoxPro,xVisualFoxPro]) then
        begin
          PDbfHdr(Header)^.VerDBF := PDbfHdr(Header)^.VerDBF or $80;
          lModified := true;
        end;
      end else // no HasBlob
        if not(FDbfVersion in [xFoxPro,xVisualFoxPro]) then
        begin
          PDbfHdr(Header)^.VerDBF := PDbfHdr(Header)^.VerDBF and $7F;
          lModified := true;
        end;

      // check if mdx flagged
      if not(FDbfVersion in [xFoxPro,xVisualFoxPro]) and (PDbfHdr(Header)^.MDXFlag <> 0) then
      begin
        // open mdx file if present
        lMdxFileName := ChangeFileExt(FileName, '.mdx');
        // Deal with case-sensitive filesystems:
        if (FileName<>'') and (UpperCase(FileName)=FileName) then
          lMdxFileName := UpperCase(lMdxFileName);
        if FileExists(lMdxFileName) or ((Mode in [pfMemoryOpen,pfMemoryCreate])) then
        begin
          // open file
          FMdxFile := TIndexFile.Create(Self);
          FMdxFile.FileName := lMdxFileName;
          FMdxFile.Mode := Mode;
          if (Mode in [pfMemoryOpen,pfMemoryCreate]) then
          begin
            FMdxFile.Stream := FIndexStream;
            FMdxFile.AutoCreate := true;
          end
          else
          begin
            FMdxFile.AutoCreate := false;
          end;
          FMdxFile.OnLocaleError := FOnLocaleError;
          FMdxFile.CodePage := UseCodePage;
          FMdxFile.Open;
          // is index ready for use?
          if not FMdxFile.ForceClose then
          begin
            FIndexFiles.Add(FMdxFile);
            // get index tag names known
            FMdxFile.GetIndexNames(FIndexNames);
          end else begin
            // asked to close! close file
            FreeAndNil(FMdxFile);
          end;
        end else begin
          // ask user
          deleteLink := true;
          if Assigned(FOnIndexMissing) then
            FOnIndexMissing(deleteLink);
          // correct flag
          if deleteLink then
          begin
            PDbfHdr(Header)^.MDXFlag := 0;
            lModified := true;
          end else
            FForceClose := true;
        end;
      end;
    end;

    // record changes
    if lModified then
      WriteHeader;
    
    // open indexes
    for I := 0 to FIndexFiles.Count - 1 do
      TIndexFile(FIndexFiles.Items[I]).Open;
  end;
end;

procedure TDbfFile.Close;
var
  MdxIndex, I: Integer;
begin
  if Active then
  begin
    // close index files first
    MdxIndex := -1;
    for I := 0 to FIndexFiles.Count - 1 do
    begin
      TIndexFile(FIndexFiles.Items[I]).Close;
      if TIndexFile(FIndexFiles.Items[I]) = FMdxFile then
        MdxIndex := I;
    end;
    // free memo file if any
    FreeAndNil(FMemoFile);

    // now we can close physical dbf file
    CloseFile;

    // free FMdxFile, remove it from the FIndexFiles and Names lists
    if MdxIndex >= 0 then
      FIndexFiles.Delete(MdxIndex);
    I := 0;
    while I < FIndexNames.Count do
    begin
      if FIndexNames.Objects[I] = FMdxFile then
      begin
        FIndexNames.Delete(I);
      end else begin
        Inc(I);
      end;
    end;
    FreeAndNil(FMdxFile);
    FreeMemAndNil(Pointer(FPrevBuffer));
    FreeMemAndNil(Pointer(FDefaultBuffer));

    // reset variables
    FFileLangId := 0;
  end;
end;

procedure TDbfFile.FinishCreate(AFieldDefs: TDbfFieldDefs; MemoSize: Integer);
var
  lEndHdrVFP:  rEndHdrVFP; //Contains Visual FoxPro backlink
  lFieldDescIII: rFieldDescIII;
  lFieldDescVII: rFieldDescVII;
  lFieldDescPtr: Pointer;
  lFieldDef: TDbfFieldDef;
  lMemoFileName: string;
  I, lFieldOffset, lSize, lPrec: Integer;
  lHasBlob: Boolean;
  lLocaleID: LCID;
  lNullVarFlagCount: integer; //(VFP only) Keeps track of number null/varlength flags needed for _NULLFLAGS size calculation
begin
  try
    // first reset file
    RecordCount := 0;
    lHasBlob := false;
    lNullVarFlagCount := 0;
    // determine codepage & locale
    if FDbfVersion in [xFoxPro, xVisualFoxPro] then
    begin
      // Don't use DbfGlobals default language ID as it is dbase-based
      if FFileLangId = 0 then
        FFileLangId := ConstructLangId(LangId_To_CodePage[FFileLangId],GetUserDefaultLCID, true);
    end
    else
    begin
      // DBase
      if FFileLangId = 0 then
        FFileLangId := DbfGlobals.DefaultCreateLangId;
    end;
    FFileCodePage := LangId_To_CodePage[FFileLangId];
    lLocaleID := LangId_To_Locale[FFileLangId];
    FUseCodePage := FFileCodePage;

    // Prepare header size. This size may be changed later depending on number
    // of fields etc - we start out with the first, fixed part of the header,
    // write out the variable parts (field descriptor arrays etc) and then
    // correct the header length in the header.
    if FDbfVersion = xBaseVII then
    begin
      // version xBaseVII without memo
      HeaderSize := SizeOf(rDbfHdr) + SizeOf(rEndFixedHdrVII);
      RecordSize := SizeOf(rFieldDescVII);
      FillChar(Header^, HeaderSize, #0);
      PDbfHdr(Header)^.VerDBF := $04;
      // write language string. FPC needs an explicit cast to pchar to avoid calling widestring version of StrPLCopy
      StrPLCopy(
        PChar(@PEndFixedHdrVII(PChar(Header)+SizeOf(rDbfHdr))^.LanguageDriverName[32]),
        PChar(ConstructLangName(FFileCodePage, lLocaleID, false)),
        63-32);
      lFieldDescPtr := @lFieldDescVII;
    end else begin
      // DBase III..V, (Visual) FoxPro without memo
      // rEndHdrVFP is covered at the end as it is placed after the variable
      // length part of the header.
      HeaderSize := SizeOf(rDbfHdr);
      RecordSize := SizeOf(rFieldDescIII);
      FillChar(Header^, HeaderSize, #0);
      // Note: VerDBF may be changed later on depending on what features/fields are used
      // (autoincrement etc)
      case FDbfVersion of
        xFoxPro: PDbfHdr(Header)^.VerDBF := $03; {FoxBASE+/FoxPro/dBASE III PLUS/dBASE IV, no memo
        alternative $02 FoxBASE is not readable by current MS Visual FoxPro drivers.
        }
        xVisualFoxPro: PDbfHdr(Header)^.VerDBF := $30; {Visual FoxPro no autoincrement,no varchar}
        else PDbfHdr(Header)^.VerDBF := $03; {FoxBASE+/FoxPro/dBASE III PLUS/dBASE IV, no memo}
      end;

      // standard language WE/Western Europe
      if FDbfVersion=xBaseIII then
        PDbfHdr(Header)^.Language := 0 //no language support
      else
        PDbfHdr(Header)^.Language := FFileLangId;

      // init field ptr
      lFieldDescPtr := @lFieldDescIII;
    end;

    // Begin variable part of the header
    // Writing field definitions
    FFieldDefs.Clear;
    // deleted mark takes 1 byte, so skip over that
    lFieldOffset := 1;
    for I := 1 to AFieldDefs.Count do
    begin
      lFieldDef := AFieldDefs.Items[I-1];

      // check if datetime conversion
      if FCopyDateTimeAsString then
        if lFieldDef.FieldType = ftDateTime then
        begin
          // convert to string
          lFieldDef.FieldType := ftString;
          lFieldDef.Size := 22;
        end;

      // update source
      lFieldDef.FieldName := AnsiUpperCase(lFieldDef.FieldName);
      lFieldDef.Offset := lFieldOffset;
      lHasBlob := lHasBlob or lFieldDef.IsBlob;
      // Check for Foxpro, too, as it can get auto-upgraded to vfp:
      if (FDbfVersion in [xFoxPro,xVisualFoxPro]) then
      begin
        if (lFieldDef.NativeFieldType='Q') or (lFieldDef.NativeFieldType='V') then
        begin
          lNullVarFlagCount:=lNullVarFlagCount+1;
        end;
        if (lFieldDef.NullPosition>=0) then
          lNullVarFlagCount:=lNullVarFlagCount+1;
      end;

      // apply field transformation tricks
      lSize := lFieldDef.Size;
      lPrec := lFieldDef.Precision;
      if (lFieldDef.NativeFieldType = 'C')
{$ifndef USE_LONG_CHAR_FIELDS}
        and (FDbfVersion in [xFoxPro,xVisualFoxPro])
{$endif}
        then
      begin
        // Up to 32kb strings
        // Stores high byte of size in precision, low in size
        lPrec := lSize shr 8;
        lSize := lSize and $FF;
      end;

      // update temp field properties
      if FDbfVersion = xBaseVII then
      begin
        FillChar(lFieldDescVII, SizeOf(lFieldDescVII), #0);
        StrPLCopy(lFieldDescVII.FieldName, lFieldDef.FieldName, SizeOf(lFieldDescVII.FieldName)-1);
        lFieldDescVII.FieldType := lFieldDef.NativeFieldType;
        lFieldDescVII.FieldSize := lSize;
        lFieldDescVII.FieldPrecision := lPrec;
        lFieldDescVII.NextAutoInc := SwapIntLE(lFieldDef.AutoInc);
        //lFieldDescVII.MDXFlag := ???
      end else begin
        FillChar(lFieldDescIII, SizeOf(lFieldDescIII), #0);
        StrPLCopy(lFieldDescIII.FieldName, lFieldDef.FieldName, SizeOf(lFieldDescIII.FieldName)-1);
        lFieldDescIII.FieldType := lFieldDef.NativeFieldType;
        lFieldDescIII.FieldSize := lSize;
        lFieldDescIII.FieldPrecision := lPrec;
        if (FDbfVersion in [xFoxPro,xVisualFoxPro]) then
          lFieldDescIII.FieldOffset := SwapIntLE(lFieldOffset);

        // Upgrade the version info if needed for supporting field types used.
        // This is also what Visual FoxPro does with FoxPro tables to which you
        // add new VFP features.
        if (FDBFVersion in [xUnknown,xFoxPro,xVisualFoxPro]) then
        begin
          // VerDBF=$03 also includes dbase formats, so we perform an extra check
          if (PDbfHdr(Header)^.VerDBF in [$02,$03]) and
            ((lFieldDef.NativeFieldType in ['0', 'Y', 'T', 'O', '+', 'Q', 'V']) or
            (lNullVarFlagCount>0)) then
          begin
            PDbfHdr(Header)^.VerDBF := $30; {Visual FoxPro}
            FDBFVersion:=xVisualFoxPro;
          end;

          // Upgrade if a non-empty backlink is specified - for FoxPro only
          if (FBackLink<>'') and
            ((FDBFVersion=xFoxPro) or (PDbfHdr(Header)^.VerDBF=$02)) then
          begin
            PDbfHdr(Header)^.VerDBF := $30; {Visual FoxPro}
            FDBFVersion:=xVisualFoxPro; //needed to write the backlink info
          end;

          //AutoInc only support in Visual Foxpro; another upgrade
          //Note: .AutoIncrementNext is really a cardinal (see the definition)
          lFieldDescIII.AutoIncrementNext:=SwapIntLE(lFieldDef.AutoInc);
          lFieldDescIII.AutoIncrementStep:=lFieldDef.AutoIncStep;
          // Set autoincrement flag using AutoIncStep as a marker
          if (lFieldDef.AutoIncStep<>0) then
            lFieldDescIII.VisualFoxProFlags:=(lFieldDescIII.VisualFoxProFlags or $0C);
          if (PDbfHdr(Header)^.VerDBF = $30) and (lFieldDef.AutoIncStep<>0) then
          begin
            PDbfHdr(Header)^.VerDBF := $31; {Visual FoxPro, autoincrement enabled}
            FDBFVersion:=xVisualFoxPro;
          end;

          // Only supported in Visual FoxPro but let's not upgrade format as
          // IsSystemField is a minor property
          if (lFieldDef.IsSystemField) then
            lFieldDescIII.VisualFoxProFlags:=(lFieldDescIII.VisualFoxProFlags or $01);
        end;
      end;

      // Update our field list
      with FFieldDefs.AddFieldDef do
      begin
        Assign(lFieldDef);
        Offset := lFieldOffset;
        AutoInc := 0;
      end;

      // save field properties
      WriteRecord(I, lFieldDescPtr);
      Inc(lFieldOffset, lFieldDef.Size);
    end;

    // Visual Foxpro: write _NULLFLAGS field if required
    if (FDBFVersion=xVisualFoxPro) and (lNullVarFlagCount>0) then
    begin
      FillChar(lFieldDescIII, SizeOf(lFieldDescIII), #0);
      StrPLCopy(lFieldDescIII.FieldName, NULLFLAGSFIELD, 10);
      lFieldDescIII.FieldType := '0'; //bytes
      lFieldDescIII.FieldSize := 1+(lNullVarFlagCount-1) div 8; //Number of bytes needed for all bit flags
      lFieldDescIII.FieldPrecision := 0;
      lFieldDescIII.FieldOffset := SwapIntLE(lFieldOffset);
      lFieldDescIII.VisualFoxProFlags:=$01+$04 ; //System column (hidden)+Column can store null values (which is a bit of a paradox)
      // Save field properties
      WriteRecord(AFieldDefs.Count+1, @lFieldDescIII);
      Inc(lFieldOffset, lFieldDescIII.FieldSize);
    end;

    // End of field descriptor; usually end of header as well.
    // Visual Foxpro backlink info is part of the header but comes after the
    // terminator
    WriteChar(FIELD_DESCRIPTOR_ARRAY_TERMINATOR);

    { For Visual FoxPro, add back-link info }
    if (FDbfVersion = xVisualFoxPro) then
    begin
      FBackLinkOffset := Stream.Position;
      // Backlink is defined as all $0 bytes if empty
      lEndHdrVFP.Backlink:=FBackLink+StringOfChar(#0, SizeOf(lEndHdrVFP.BackLink));
      WriteBlock(@lEndHdrVFP,SizeOf(lEndHdrVFP),Stream.Position);
    end;

    // Write memo bit to begin of header
    if lHasBlob then
    begin
      case FDbfVersion of
        xBaseIII: PDbfHdr(Header)^.VerDBF := PDbfHdr(Header)^.VerDBF or $80;
        xFoxPro: if (PDbfHdr(Header)^.VerDBF in [$02,$03]) then {change from FoxBASE to...}
          PDbfHdr(Header)^.VerDBF := $F5; {...FoxPro 2.x (or earlier) with memo}
        xVisualFoxPro: //MSDN says field 28 or $02 to set memo flag
          PDbfHdr(Header)^.MDXFlag := PDbfHdr(Header)^.MDXFlag or $02;
        else PDbfHdr(Header)^.VerDBF := PDbfHdr(Header)^.VerDBF or $88;
      end;
    end;

    // Update header to correct sizes
    PDbfHdr(Header)^.RecordSize := lFieldOffset;
    if lNullVarFlagCount>0 then
      PDbfHdr(Header)^.FullHdrSize := HeaderSize + RecordSize * (AFieldDefs.Count + 1) + 1
    else
      PDbfHdr(Header)^.FullHdrSize := HeaderSize + RecordSize * AFieldDefs.Count + 1;
    if DbfVersion=xVisualFoxPro then
      PDbfHdr(Header)^.FullHdrSize := PDbfHdr(Header)^.FullHdrSize + SizeOf(rEndHdrVFP);

    // write dbf header to disk
    inherited WriteHeader;
  finally
    RecordSize := PDbfHdr(Header)^.RecordSize;
    HeaderSize := PDbfHdr(Header)^.FullHdrSize;

    // write full header to disk (dbf+fields)
    WriteHeader;
  end;

  if HasBlob and (FMemoFile=nil) then
  begin
    lMemoFileName := ChangeFileExt(FileName, GetMemoExt);
    if (FDbfVersion in [xFoxPro,xVisualFoxPro]) then
      FMemoFile := TFoxProMemoFile.Create(Self)
    else
      FMemoFile := TDbaseMemoFile.Create(Self);
    FMemoFile.FileName := lMemoFileName;
    if (Mode in [pfMemoryOpen,pfMemoryCreate]) then
      FMemoFile.Stream:=FMemoStream;
    FMemoFile.Mode := Mode;
    FMemoFile.AutoCreate := AutoCreate;
    FMemoFile.MemoRecordSize := MemoSize;
    FMemoFile.DbfVersion := FDbfVersion;
    FMemoFile.Open;
  end;
end;

function TDbfFile.HasBlob: Boolean;
var
  I: Integer;
begin
  Result := false;
  for I := 0 to FFieldDefs.Count-1 do
    if FFieldDefs.Items[I].IsBlob then
    begin
      Result := true;
      break;
    end;
end;

function TDbfFile.GetMemoExt: string;
begin
  case FDbfVersion of
    xFoxPro, xVisualFoxPro: Result := '.fpt'
    else Result := '.dbt';
  end;
  if (FFileName<>'') and (FFileName=UpperCase(FFileName)) then
    Result := UpperCase(Result);
end;

procedure TDbfFile.Zap;
begin
  // make recordcount zero
  RecordCount := 0;
  // update recordcount
  PDbfHdr(Header)^.RecordCount := RecordCount;
  // update disk header
  WriteHeader;
  // update indexes
  RegenerateIndexes;
end;

procedure TDbfFile.WriteHeader;
var
  SystemTime: TSystemTime;
  lDataHdr: PDbfHdr;
  EofTerminator: Byte;
begin
  if (HeaderSize=0) then
    exit;

  //FillHeader(0);
  lDataHdr := PDbfHdr(Header);
  GetLocalTime(SystemTime);
  lDataHdr^.Year := SystemTime.wYear - 1900;
  lDataHdr^.Month := SystemTime.wMonth;
  lDataHdr^.Day := SystemTime.wDay;
//  lDataHdr.RecordCount := RecordCount;
  inherited WriteHeader;

  // Write terminator at the end of the file, after the records:
  EofTerminator := $1A;
  // We're using lDataHdr to make sure we have the latest/correct version
  WriteBlock(@EofTerminator, 1, CalcPageOffset(lDataHdr.RecordCount+1));
end;

procedure TDbfFile.ConstructFieldDefs;
var
  // The size of the fixed part of the header
  // excluding the field descriptor array
  // also excluding everything that comes after the field descriptor array
  // like VFP backlink records
  lFakeHeaderSize: Integer;
  lFieldSize: Integer;
  lPropHdrOffset, lFieldOffset: Integer;
  lFieldDescIII: rFieldDescIII;
  lFieldDescVII: rFieldDescVII;
  lFieldPropsHdr: rFieldPropsHdr;
  lStdProp: rStdPropEntry;
  TempFieldDef: TDbfFieldDef;
  lSize,lPrec,I, lColumnCount: Integer;
  lAutoInc: Cardinal;
  dataPtr: PChar;
  lNativeFieldType: Char;
  lFieldName: string;
  lCanHoldNull: boolean; //Can the field store nulls, i.e. is it nullable?
  lIsVFPSystemField: boolean; //Is this a Visual FoxPro system/hidden field?
  lIsVFPVarLength: boolean; //Is this a Visual FoxPro varbinary/varchar field,
  // where varlength bit is maintained in _NULLFLAGS
  lCurrentNullPosition: integer;
begin
  FFieldDefs.Clear;
  case DbfVersion of
    xBaseVII:
      begin
        lFakeHeaderSize := SizeOf(rDbfHdr) + SizeOf(rEndFixedHdrVII);
        lFieldSize := SizeOf(rFieldDescVII);
      end;
    else
    begin
      // DBase III..V, (Visual) FoxPro
      if DBfVersion = xVisualFoxPro then
        lFakeHeaderSize := SizeOf(rDbfHdr)
      else
        lFakeHeaderSize := SizeOf(rDbfHdr);
      lFieldSize := SizeOf(rFieldDescIII);
    end;
  end;

  // This is of course not true but it shrinks the perceived header to just
  // before the records with field info:
  HeaderSize := lFakeHeaderSize;
  RecordSize := lFieldSize;
  if FDbfVersion=xVisualFoxPro then
    lColumnCount := (PDbfHdr(Header)^.FullHdrSize - lFakeHeaderSize - SizeOf(rEndHdrVFP)) div lFieldSize
  else
    lColumnCount := (PDbfHdr(Header)^.FullHdrSize - lFakeHeaderSize) div lFieldSize;

  FBackLinkOffset := 0;
  FLockField := nil;
  FNullField := nil;
  FAutoIncPresent := false;
  lFieldOffset := 1;
  lAutoInc := 0;
  I := 1;
  lCurrentNullPosition := 0; // Contains the next value for the _NULLFLAGS bit position
  lCanHoldNull := false;
  lIsVFPSystemField := false;
  lIsVFPVarLength := false;
  try
    // Specs say there has to be at least one field, so use repeat:
    repeat
      // version field info?
      if FDbfVersion = xBaseVII then
      begin
        ReadRecord(I, @lFieldDescVII);
        lFieldName := AnsiUpperCase(PChar(@lFieldDescVII.FieldName[0]));
        lSize := lFieldDescVII.FieldSize;
        lPrec := lFieldDescVII.FieldPrecision;
        lNativeFieldType := lFieldDescVII.FieldType;
        lAutoInc := SwapIntLE(lFieldDescVII.NextAutoInc);
        if lNativeFieldType = '+' then
          FAutoIncPresent := true;
      end else begin
        // DBase III..V, FoxPro, Visual FoxPro
        ReadRecord(I, @lFieldDescIII);
        lFieldName := AnsiUpperCase(PChar(@lFieldDescIII.FieldName[0]));
        lSize := lFieldDescIII.FieldSize;
        lPrec := lFieldDescIII.FieldPrecision;
        lNativeFieldType := lFieldDescIII.FieldType;
        if (FDBFVersion=xVisualFoxPro) and ((lFieldDescIII.VisualFoxProFlags and $0C)<>0) then
        begin
          // We do not test for an I field - we could implement our own N autoincrement this way...
          lAutoInc:=lFieldDescIII.AutoIncrementNext;
          FAutoIncPresent:=true;
        end;

        // Only Visual FoxPro supports null fields, if the nullable field flag is on
        lCanHoldNull := (FDbfVersion in [xVisualFoxPro]) and
          ((lFieldDescIII.VisualFoxProFlags and $2) <> 0) and
          (lFieldName <> NULLFLAGSFIELD {the field where null status is stored can never be null itself});
        // System/hidden flag (VFP only):
        lIsVFPSystemField := (FDbfVersion in [xVisualFoxPro]) and
          ((lFieldDescIII.VisualFoxProFlags and $01)=$01);
        // Only Visual Foxpro supports varbinary/varchar fields where a flag indicates
        // if the actual size is stored in the last data byte.
        lIsVFPVarLength := (FDbfVersion in [xVisualFoxPro]) and
          (lNativeFieldType in ['Q','V']) and
          (lFieldName <> NULLFLAGSFIELD);
      end;

      // apply field transformation tricks
      if (lNativeFieldType = 'C') 
{$ifndef USE_LONG_CHAR_FIELDS}
        and (FDbfVersion in [xFoxPro,xVisualFoxPro])
{$endif}
        then
      begin
        // (V)FP uses the byte where precision is normally stored
        // for the high byte of the field size
        lSize := lSize + lPrec shl 8;
        lPrec := 0;
      end;

      // add field
      TempFieldDef := FFieldDefs.AddFieldDef;
      with TempFieldDef do
      begin
        FieldName := lFieldName;
        Offset := lFieldOffset;
        Size := lSize;
        Precision := lPrec;
        AutoInc := lAutoInc;
        NativeFieldType := lNativeFieldType;
        IsSystemField := lIsVFPSystemField;
        if lIsVFPVarLength then
        begin
          // The varlength flag uses the same _NULLFLAGS field as the null flags.
          // It comes before the null bit for that field, if any.
          VarLengthPosition := lCurrentNullPosition;
          inc(lCurrentNullPosition);
        end else
          VarLengthPosition := -1;
        if lCanHoldNull then
        begin
          NullPosition := lCurrentNullPosition;
          inc(lCurrentNullPosition);
        end else
          NullPosition := -1;
      end;

      // check valid field:
      //  1) non-empty field name
      //  2) known field type
      //  {3) no changes have to be made to precision or size}
      if (Length(lFieldName) = 0) or (TempFieldDef.FieldType = ftUnknown) then
        raise EDbfError.Create(STRING_INVALID_DBF_FILE_FIELDERROR);

      // determine if lock field present, if present, then store additional info
      if lFieldName = '_DBASELOCK' then
      begin
        FLockField := TempFieldDef;
        FLockUserLen := lSize - 8;
        if FLockUserLen > DbfGlobals.UserNameLen then
          FLockUserLen := DbfGlobals.UserNameLen;
      end else
      if (FDbfVersion=xVisualFoxPro) and (uppercase(lFieldName) = NULLFLAGSFIELD) then
        FNullField := TempFieldDef;

      // goto next field
      Inc(lFieldOffset, lSize);
      Inc(I);

      // continue until header termination character found
      // or end of header reached
    until (I > lColumnCount) or (ReadChar = FIELD_DESCRIPTOR_ARRAY_TERMINATOR);

    if FDbfVersion=xVisualFoxPro then
      FBackLinkOffset:=Stream.Position+SizeOf(FIELD_DESCRIPTOR_ARRAY_TERMINATOR); //after FIELD_DESCRIPTION_ARRAY_TERMINATOR

    // test if not too many fields
    if FFieldDefs.Count >= 4096 then
      raise EDbfError.CreateFmt(STRING_INVALID_FIELD_COUNT, [FFieldDefs.Count]);

    // do not check FieldOffset = PDbfHdr(Header).RecordSize because additional 
    // data could be present in record

    // get current position
    lPropHdrOffset := Stream.Position;

    // dBase 7 -> read field properties, test if enough space, maybe no header
    if (FDbfVersion = xBaseVII) and (lPropHdrOffset + Sizeof(lFieldPropsHdr) <
      PDbfHdr(Header)^.FullHdrSize) then
    begin
      // read in field properties header
      ReadBlock(@lFieldPropsHdr, SizeOf(lFieldPropsHdr), lPropHdrOffset);
      // read in standard properties
      lFieldOffset := lPropHdrOffset + lFieldPropsHdr.StartStdProps;
      for I := 0 to lFieldPropsHdr.NumStdProps - 1 do
      begin
        // read property data
        ReadBlock(@lStdProp, SizeOf(lStdProp), lFieldOffset+I*SizeOf(lStdProp));
        // is this a constraint?
        if lStdProp.FieldOffset = 0 then
        begin
          // this is a constraint...not implemented
        end else if lStdProp.FieldOffset <= FFieldDefs.Count then begin
          // get fielddef for this property
          TempFieldDef := FFieldDefs.Items[lStdProp.FieldOffset-1];
          // allocate space to store data
          TempFieldDef.AllocBuffers;
          // dataPtr = nil -> no data to retrieve
          dataPtr := nil;
          // store data
          case lStdProp.PropType of
            FieldPropType_Required: TempFieldDef.Required := true;
            FieldPropType_Default:
              begin
                dataPtr := TempFieldDef.DefaultBuf;
                TempFieldDef.HasDefault := true;
              end;
            FieldPropType_Min:
              begin
                dataPtr := TempFieldDef.MinBuf;
                TempFieldDef.HasMin := true;
              end;
            FieldPropType_Max:
              begin
                dataPtr := TempFieldDef.MaxBuf;
                TempFieldDef.HasMax := true;
              end;
          end;
          // get data for this property
          if dataPtr <> nil then
            ReadBlock(dataPtr, lStdProp.DataSize, lPropHdrOffset + lStdProp.DataOffset);
        end;
      end;
      // todo: read dbase7 custom properties...not implemented
      // todo: read dbase7 RI/referential integrity properties...not implemented
    end;
  finally
    // Restore proper sizes so normal records after the header can be read
    HeaderSize := PDbfHdr(Header)^.FullHdrSize;
    RecordSize := PDbfHdr(Header)^.RecordSize;
  end;
end;

function TDbfFile.GetLanguageId: Integer;
begin
  Result := PDbfHdr(Header)^.Language;
end;

function TDbfFile.GetLanguageStr: string;
begin
  if FDbfVersion = xBaseVII then
    Result := PEndFixedHdrVII(PChar(Header) + SizeOf(rDbfHdr))^.LanguageDriverName
  else
    Result := '';  // Only supported in DbaseVII
end;

function TDbfFile.IsNullFlagSet(const Src: Pointer; var AFieldDef: TDbfFieldDef; WhichField: TNullFieldFlag): boolean;
var
  NullFlagByte: Pointer;
begin
  case WhichField of
  nfNullFlag:
    begin
      if (AFieldDef.NullPosition<0) or (FNullField=nil) then
        result:=false //field is not even nullable
      else
      begin
        // go to _NULLFLAGS byte that has this field's null flag
        // Find out the byte where the null bit for the field is stored by doing
        // NullPosition shr3 (= NullPosition div 8)...
        NullFlagByte := PChar(Src) + FNullField.Offset + (AFieldDef.NullPosition shr 3);
        // ... get the correct bit in the byte by the equivalent of getting the bit number in that byte:
        // NullPosition and $7 (=mod 8)... and going to the bit value in the byte (by shl)
        // The result is true if the field is null.
        Result := (PByte(NullFlagByte)^ and (1 shl (AFieldDef.NullPosition and $7))) <> 0;
      end;
    end;
  nfVarlengthFlag:
    begin
      if (AFieldDef.VarLengthPosition<0) or (FNullField=nil) then
        result:=false //field *never* has a varlength byte
      else
      begin
        NullFlagByte := PChar(Src) + FNullField.Offset + (AFieldDef.VarLengthPosition shr 3);
        Result := (PByte(NullFlagByte)^ and (1 shl (AFieldDef.VarLengthPosition and $7))) <> 0
      end;
    end;
  end;
end;

{
  I fill the holes with the last records.
  now we can do an 'in-place' pack
}
procedure TDbfFile.FastPackTable;
var
  iDel,iNormal: Integer;
  pDel,pNormal: PChar;

  function FindFirstDel: Boolean;
  begin
    while iDel<=iNormal do
    begin
      ReadRecord(iDel, pDel);
      if (PChar(pDel)^ <> ' ') then
      begin
        Result := true;
        exit;
      end;
      Inc(iDel);
    end;
    Result := false;
  end;

  function FindLastNormal: Boolean;
  begin
    while iNormal>=iDel do
    begin
      ReadRecord(iNormal, pNormal);
      if (PChar(pNormal)^= ' ') then
      begin
        Result := true;
        exit;
      end;
      dec(iNormal);
    end;
    Result := false;
  end;

begin
  if RecordSize < 1 then Exit;

  GetMem(pNormal, RecordSize);
  GetMem(pDel, RecordSize);
  try
    iDel := 1;
    iNormal := RecordCount;

    while FindFirstDel do
    begin
      // iDel is definitely deleted
      if FindLastNormal then
      begin
        // but is not anymore
        WriteRecord(iDel, pNormal);
        PChar(pNormal)^ := '*';
        WriteRecord(iNormal, pNormal);
      end else begin
        // Cannot find a record after iDel so iDel must be deleted
        dec(iDel);
        break;
      end;
    end;
    // FindFirstDel failed means than iDel is full
    RecordCount := iDel;
    RegenerateIndexes;
    // Pack Memofields
  finally
    FreeMem(pNormal);
    FreeMem(pDel);
  end;
end;

procedure TDbfFile.Rename(DestFileName: string; NewIndexFileNames: TStrings; DeleteFiles: boolean);
var
  lIndexFileNames: TStrings;
  lIndexFile: TIndexFile;
  NewBaseName: string;
  I: integer;
begin
  // todo: verify if this works with memo files
  // get memory for index file list
  lIndexFileNames := TStringList.Create;
  try 
    // save index filenames
    for I := 0 to FIndexFiles.Count - 1 do
    begin
      lIndexFile := TIndexFile(IndexFiles[I]);
      lIndexFileNames.Add(lIndexFile.FileName);
      // prepare changing the dbf file name, needs changes in index files
      lIndexFile.PrepareRename(NewIndexFileNames[I]);
    end;

    // close file
    Close;

    if DeleteFiles then
    begin
      SysUtils.DeleteFile(DestFileName);
      SysUtils.DeleteFile(ChangeFileExt(DestFileName, GetMemoExt));
    end else begin
      I := 0;
      FindNextName(DestFileName, NewBaseName, I);
      SysUtils.RenameFile(DestFileName, NewBaseName);
      SysUtils.RenameFile(ChangeFileExt(DestFileName, GetMemoExt), 
        ChangeFileExt(NewBaseName, GetMemoExt));
    end;
    // delete old index files
    for I := 0 to NewIndexFileNames.Count - 1 do
      SysUtils.DeleteFile(NewIndexFileNames.Strings[I]);
    // rename the new dbf files
    SysUtils.RenameFile(FileName, DestFileName);
    SysUtils.RenameFile(ChangeFileExt(FileName, GetMemoExt), 
      ChangeFileExt(DestFileName, GetMemoExt));
    // rename new index files
    for I := 0 to NewIndexFileNames.Count - 1 do
      SysUtils.RenameFile(lIndexFileNames.Strings[I], NewIndexFileNames.Strings[I]);
  finally
    lIndexFileNames.Free;
  end;  
end;

type
  TRestructFieldInfo = record
    SourceOffset: Integer;
    DestOffset: Integer;
    Size: Integer;
  end;

  { assume nobody has more than 8192 fields, otherwise possibly range check error }
  PRestructFieldInfo = ^TRestructFieldInfoArray;
  TRestructFieldInfoArray = array[0..8191] of TRestructFieldInfo;

procedure TDbfFile.RestructureTable(DbfFieldDefs: TDbfFieldDefs; Pack: Boolean);
var
  DestDbfFile: TDbfFile;
  TempIndexDef: TDbfIndexDef;
  TempIndexFile: TIndexFile;
  DestFieldDefs: TDbfFieldDefs;
  TempDstDef, TempSrcDef: TDbfFieldDef;
  OldIndexFiles: TStrings;
  IndexName, NewBaseName: string;
  I, lRecNo, lFieldNo, lFieldSize, lBlobPageNo, lWRecNo, srcOffset, dstOffset: Integer;
  pBuff, pDestBuff: TRecordBuffer;
  RestructFieldInfo: PRestructFieldInfo;
  BlobStream: TMemoryStream;
begin
  // nothing to do?
  if (RecordSize < 1) or ((DbfFieldDefs = nil) and not Pack) then
    exit;

  // if no exclusive access, terrible things can happen!
  CheckExclusiveAccess;

  // make up some temporary filenames
  lRecNo := 0;
  FindNextName(FileName, NewBaseName, lRecNo);

  // select final field definition list
  if DbfFieldDefs = nil then
  begin
    DestFieldDefs := FFieldDefs;
  end else begin
    DestFieldDefs := DbfFieldDefs;
    // copy autoinc values
    for I := 0 to DbfFieldDefs.Count - 1 do
    begin
      lFieldNo := DbfFieldDefs.Items[I].CopyFrom;
      if (lFieldNo >= 0) and (lFieldNo < FFieldDefs.Count) then
        DbfFieldDefs.Items[I].AutoInc := FFieldDefs.Items[lFieldNo].AutoInc;
    end;
  end;

  // create temporary dbf
  DestDbfFile := TDbfFile.Create;
  DestDbfFile.FileName := NewBaseName;
  DestDbfFile.AutoCreate := true;
  DestDbfFile.Mode := pfExclusiveCreate;
  DestDbfFile.OnIndexMissing := FOnIndexMissing;
  DestDbfFile.OnLocaleError := FOnLocaleError;
  DestDbfFile.DbfVersion := FDbfVersion;
  DestDbfFile.FileLangId := FileLangId;
  DestDbfFile.Open;
  // create dbf header
  if FMemoFile <> nil then
    DestDbfFile.FinishCreate(DestFieldDefs, FMemoFile.RecordSize)
  else
    if (DestDbfFile.DbfVersion in [xFoxPro,xVisualFoxPro]) then
      DestDbfFile.FinishCreate(DestFieldDefs, 64) {VFP default}
    else
      DestDbfFile.FinishCreate(DestFieldDefs, 512);

  // adjust size and offsets of fields
  GetMem(RestructFieldInfo, sizeof(TRestructFieldInfo)*DestFieldDefs.Count);
  for lFieldNo := 0 to DestFieldDefs.Count - 1 do
  begin
    TempDstDef := DestFieldDefs.Items[lFieldNo];
    if TempDstDef.CopyFrom >= 0 then
    begin
      TempSrcDef := FFieldDefs.Items[TempDstDef.CopyFrom];
      if TempDstDef.NativeFieldType in ['F', 'N'] then
      begin
        // get minimum field length
        lFieldSize := Min(TempSrcDef.Precision, TempDstDef.Precision) +
          Min(TempSrcDef.Size - TempSrcDef.Precision,
            TempDstDef.Size - TempDstDef.Precision);
        // if one has dec separator, but other not, we lose one digit
        if (TempDstDef.Precision > 0) xor 
          ((TempSrcDef.NativeFieldType in ['F', 'N']) and (TempSrcDef.Precision > 0)) then
          Dec(lFieldSize);
        // should not happen, but check nevertheless (maybe corrupt data)
        if lFieldSize < 0 then
          lFieldSize := 0;
        srcOffset := TempSrcDef.Size - TempSrcDef.Precision -
          (TempDstDef.Size - TempDstDef.Precision);
        if srcOffset < 0 then
        begin
          dstOffset := -srcOffset;
          srcOffset := 0;
        end else begin
          dstOffset := 0;
        end;
      end else begin
        lFieldSize := Min(TempSrcDef.Size, TempDstDef.Size);
        srcOffset := 0;
        dstOffset := 0;
      end;
      with RestructFieldInfo[lFieldNo] do
      begin
        Size := lFieldSize;
        SourceOffset := TempSrcDef.Offset + srcOffset;
        DestOffset := TempDstDef.Offset + dstOffset;
      end;
    end;
  end;

  // add indexes
  TempIndexDef := TDbfIndexDef.Create(nil);
  for I := 0 to FIndexNames.Count - 1 do
  begin
    // get length of extension -> determines MDX or NDX
    IndexName := FIndexNames.Strings[I];
    TempIndexFile := TIndexFile(FIndexNames.Objects[I]);
    TempIndexFile.GetIndexInfo(IndexName, TempIndexDef);
    if Length(ExtractFileExt(IndexName)) > 0 then
    begin
      // NDX index, get unique file name
      lRecNo := 0;
      FindNextName(IndexName, IndexName, lRecNo);
    end;
    // add this index
    DestDbfFile.OpenIndex(IndexName, TempIndexDef.SortField, true, TempIndexDef.Options);
  end;
  TempIndexDef.Free;

  // get memory for record buffers
  GetMem(pBuff, RecordSize);
  BlobStream := TMemoryStream.Create;
  OldIndexFiles := TStringList.Create;
  // if restructure, we need memory for dest buffer, otherwise use source
  if DbfFieldDefs = nil then
    pDestBuff := pBuff
  else
    GetMem(pDestBuff, DestDbfFile.RecordSize);

  // Go through record data:
  try
{$ifdef USE_CACHE}
    BufferAhead := true;
    DestDbfFile.BufferAhead := true;
{$endif}
    lWRecNo := 1;
    for lRecNo := 1 to RecordCount do
    begin
      // read record from original dbf
      ReadRecord(lRecNo, pBuff);
      // copy record unless (deleted or user wants packing)
      if (ansichar(pBuff^) <> '*') or not Pack then
      begin
        // if restructure, initialize dest
        if DbfFieldDefs <> nil then
        begin
          DestDbfFile.InitRecord(pDestBuff);
          // copy deleted mark (the first byte)
          pDestBuff^ := pBuff^;
        end;

        if (DbfFieldDefs <> nil) or (FMemoFile <> nil) then
        begin
          // copy fields
          for lFieldNo := 0 to DestFieldDefs.Count-1 do
          begin
            TempDstDef := DestFieldDefs.Items[lFieldNo];
            // handle blob fields differently
            // don't try to copy new blob fields!
            // DbfFieldDefs = nil -> pack only
            // TempDstDef.CopyFrom >= 0 -> copy existing (blob) field
            if TempDstDef.IsBlob and ((DbfFieldDefs = nil) or (TempDstDef.CopyFrom >= 0)) then
            begin
              // get current blob blockno
              GetFieldData(lFieldNo, ftInteger, pBuff, @lBlobPageNo, false);
              // valid blockno read?
              if lBlobPageNo > 0 then
              begin
                BlobStream.Clear;
                FMemoFile.ReadMemo(lBlobPageNo, BlobStream);
                BlobStream.Position := 0;
                // always append
                DestDbfFile.FMemoFile.WriteMemo(lBlobPageNo, 0, BlobStream);
              end;
              // write new blockno
              DestDbfFile.SetFieldData(lFieldNo, ftInteger, @lBlobPageNo, pDestBuff, false);
            end else if (DbfFieldDefs <> nil) and (TempDstDef.CopyFrom >= 0) then
            begin
              // copy content of field
              with RestructFieldInfo[lFieldNo] do
                Move(pBuff[SourceOffset], pDestBuff[DestOffset], Size);
            end;
          end;
        end;

        // write record
        DestDbfFile.WriteRecord(lWRecNo, pDestBuff);
        // update indexes
        for I := 0 to DestDbfFile.IndexFiles.Count - 1 do
          TIndexFile(DestDbfFile.IndexFiles.Items[I]).Insert(lWRecNo, pDestBuff);

        // go to next record
        Inc(lWRecNo);
      end;
    end;

{$ifdef USE_CACHE}
    BufferAhead := false;
    DestDbfFile.BufferAhead := false;
{$endif}

    // save index filenames
    for I := 0 to FIndexFiles.Count - 1 do
      OldIndexFiles.Add(TIndexFile(IndexFiles[I]).FileName);

    // close dbf
    Close;

    // if restructure -> rename the old dbf files
    // if pack only -> delete the old dbf files
    DestDbfFile.Rename(FileName, OldIndexFiles, DbfFieldDefs = nil);
    
    // we have to reinit fielddefs if restructured
    Open;

    // crop deleted records
    RecordCount := lWRecNo - 1;
    // update date/time stamp, recordcount
    PDbfHdr(Header)^.RecordCount := RecordCount;
    WriteHeader;
  finally
    // close temporary file
    FreeAndNil(DestDbfFile);
    // free mem
    FreeAndNil(OldIndexFiles);
    FreeMem(pBuff);
    FreeAndNil(BlobStream);
    FreeMem(RestructFieldInfo);
    if DbfFieldDefs <> nil then
      FreeMem(pDestBuff);
  end;
end;

procedure TDbfFile.RegenerateIndexes;
var
  lIndexNo: Integer;
begin
  // recreate every index in every file
  for lIndexNo := 0 to FIndexFiles.Count-1 do
  begin
    PackIndex(TIndexFile(FIndexFiles.Items[lIndexNo]), EmptyStr);
  end;
end;

function TDbfFile.GetFieldInfo(FieldName: string): TDbfFieldDef;
var
  I: Integer;
  lfi: TDbfFieldDef;
begin
  FieldName := AnsiUpperCase(FieldName);
  for I := 0 to FFieldDefs.Count-1 do
  begin
    lfi := TDbfFieldDef(FFieldDefs.Items[I]);
    if lfi.fieldName = FieldName then
    begin
      Result := lfi;
      exit;
    end;
  end;
  Result := nil;
end;

// NOTE: Dst may be nil!
function TDbfFile.GetFieldData(Column: Integer; DataType: TFieldType; 
  Src, Dst: Pointer; NativeFormat: boolean): Boolean;
var
  TempFieldDef: TDbfFieldDef;
begin
  TempFieldDef := TDbfFieldDef(FFieldDefs.Items[Column]);
  Result := GetFieldDataFromDef(TempFieldDef, DataType, Src, Dst, NativeFormat);
end;

// NOTE: Dst may be nil!
function TDbfFile.GetFieldDataFromDef(AFieldDef: TDbfFieldDef; DataType: TFieldType; 
  Src, Dst: Pointer; NativeFormat: boolean): Boolean;
var
  FieldOffset, FieldSize: Integer;
//  s: string;
  ldd, ldm, ldy, lth, ltm, lts: Integer;
  date: TDateTime;
  timeStamp: TTimeStamp;
  asciiContents: boolean;
  SrcRecord: Pointer;

{$ifdef SUPPORT_INT64}
  function GetInt64FromStrLength(Src: Pointer; Size: Integer; Default: Int64): Int64;
  var
    endChar: Char;
    Code: Integer;
  begin
    // save Char at pos term. null
    endChar := (PChar(Src) + Size)^;
    (PChar(Src) + Size)^ := #0;
    // convert
    Val(PChar(Src), Result, Code);
    // check success
    if Code <> 0 then Result := Default;
    // restore prev. ending Char
    (PChar(Src) + Size)^ := endChar;
  end;
{$endif}

  procedure CorrectYear(var wYear: Integer);
  var wD, wM, wY, CenturyBase: Word;

{$ifndef DELPHI_5}
  // Delphi 3 standard behavior, no change possible
  const TwoDigitYearCenturyWindow= 0;
{$endif}

  begin
     if wYear >= 100 then
       Exit;
     DecodeDate(Date, wY, wm, wD);
     // use Delphi-Date-Window
     CenturyBase := wY{must be CurrentYear} - TwoDigitYearCenturyWindow;
     Inc(wYear, CenturyBase div 100 * 100);
     if (TwoDigitYearCenturyWindow > 0) and (wYear < CenturyBase) then
        Inc(wYear, 100);
  end;

  procedure SaveDateToDst;
  begin
    if not NativeFormat then
    begin
      // Delphi 5 requests a TDateTime
      PDateTime(Dst)^ := date;
    end else begin
      // Delphi 3 and 4 request a TDateTimeRec
      //  date is TTimeStamp.date
      //  datetime = msecs == BDE timestamp as we implemented it
      if DataType = ftDateTime then
      begin
        PDateTimeRec(Dst)^.DateTime := date;
      end else begin
        PLongInt(Dst)^ := DateTimeToTimeStamp(date).Date;
      end;
    end;
  end;

begin
  // test if non-nil source (record buffer)
  if Src = nil then
  begin
    Result := false;
    exit;
  end;

  // check Dst = nil, called with dst = nil to check empty field
  if (FNullField <> nil) and (Dst = nil) and (AFieldDef.NullPosition >= 0) then
  begin
    result:= not(IsNullFlagSet(Src, AFieldDef, nfNullFlag));
    exit;
  end;

  FieldOffset := AFieldDef.Offset;
  FieldSize := AFieldDef.Size;
  SrcRecord := Src;
  Src := PChar(Src) + FieldOffset;
  asciiContents := false;
  Result := true;
  // field types that are binary and of which the fieldsize should not be truncated
  case AFieldDef.NativeFieldType of
    '+', 'I': //Autoincrement, integer
      begin
        if not(FDbfVersion in [xFoxPro,xVisualFoxPro]) then
        begin
          Result := Unaligned(PDWord(Src)^) <> 0;
          if Result and (Dst <> nil) then
          begin
            PDWord(Dst)^ := SwapIntBE(Unaligned(PDWord(Src)^));
            if Result then
              PInteger(Dst)^ := Integer(PDWord(Dst)^ xor $80000000);
          end;
        end else begin
          Result := true;
          if Dst <> nil then
            PInteger(Dst)^ := SwapIntLE(Unaligned(PInteger(Src)^));
        end;
      end;
    'O':
      begin
{$ifdef SUPPORT_INT64}
        Result := Unaligned(PInt64(Src)^) <> 0;
        if Result and (Dst <> nil) then
        begin
          SwapInt64BE(Src, Dst);
          if PInt64(Dst)^ > 0 then
            PInt64(Dst)^ := not PInt64(Dst)^
          else
            PDouble(Dst)^ := PDouble(Dst)^ * -1;
        end;
{$endif}
      end;
    '@':
      begin
        Result := (Unaligned(PInteger(Src)^) <> 0) and (Unaligned(PInteger(PChar(Src)+4)^) <> 0);
        if Result and (Dst <> nil) then
        begin
          SwapInt64BE(Src, Dst);
          if FDateTimeHandling = dtBDETimeStamp then
            date := BDETimeStampToDateTime(PDouble(Dst)^)
          else
            date := PDateTime(Dst)^;
          SaveDateToDst;
        end;
      end;
    'T':
      begin
        // all binary zeroes -> empty datetime
{$ifdef SUPPORT_INT64}        
        Result := Unaligned(PInt64(Src)^) <> 0;
{$else}        
        Result := (Unaligned(PInteger(Src)^) <> 0) or (Unaligned(PInteger(PChar(Src)+4)^) <> 0);
{$endif}        
        if Result and (Dst <> nil) then
        begin
          timeStamp.Date := SwapIntLE(Unaligned(PInteger(Src)^)) - JulianDateDelta;
          timeStamp.Time := SwapIntLE(Unaligned(PInteger(PChar(Src)+4)^));
          date := TimeStampToDateTime(timeStamp);
          SaveDateToDst;
        end;
      end;
    'Y': // currency
      begin
{$ifdef SUPPORT_INT64}
        Result := true;
        if Dst <> nil then
        begin
          PInt64(Dst)^ := SwapIntLE(Unaligned(PInt64(Src)^));
          if DataType = ftCurrency then
            PDouble(Dst)^ := PInt64(Dst)^ / 10000.0;
        end;
{$endif}
      end;
    'B':  // Foxpro double
      begin
        if (FDbfVersion in [xFoxPro,xVisualFoxPro]) then
        begin
        {$ifdef SUPPORT_INT64}
          Result := Unaligned(PInt64(Src)^) <> 0;
          if Result and (Dst <> nil) then
          begin
            SwapInt64LE(Src, Dst);
            PDouble(Dst)^ := PDouble(Dst)^;
          end;
        {$endif} end else
          asciiContents := true;
      end;
    'M':
      begin
        if FieldSize = 4 then
        begin
          Result := Unaligned(PInteger(Src)^) <> 0;
          if Dst <> nil then
            PInteger(Dst)^ := SwapIntLE(Unaligned(PInteger(Src)^));
        end else
          asciiContents := true;
      end;
    'Q', 'V':  // Visual Foxpro varbinary, varchar
      //todo: check if codepage conversion/translation for varchar is needed
      begin
        if (FDbfVersion in [xVisualFoxPro]) then
        begin
          Result := true;
          // The length byte is only stored if the field is not full
          if (Dst <> nil) then
          begin
            //clear the destination, just in case
            Fillchar(pbyte(Dst)^,Fieldsize,0);
            if IsNullFlagSet(SrcRecord, AFieldDef, nfVarlengthFlag) then
            // so we decrease the fieldsize and let the rest of the code handle it
              FieldSize:=(PByte(Src)+FieldSize-1)^;
            // If field is not null:
            if not(IsNullFlagSet(SrcRecord, AFieldDef, nfNullFlag)) then
              if Afielddef.FieldType=ftVarBytes then
              begin
                PWord(Dst)^:=Fieldsize; //Store size in destination
                move(Src^, pbyte(Dst+sizeof(Word))^, FieldSize)
              end
              else
                move(Src^, pbyte(Dst)^, FieldSize)
            else
              result:=false;
          end;
        end;
      end;
    '0':  // Zero not letter 0: bytes
      begin
        if (Dst <> nil) then
        begin
          //clear the destination, just in case
          Fillchar(pbyte(Dst)^,Fieldsize,0);
          move(Src^, pbyte(Dst)^, FieldSize);
          Result := true;
        end;
      end;
  else
    asciiContents := true;
  end;
  if asciiContents then
  begin
    //    SetString(s, PChar(Src) + FieldOffset, FieldSize );
    //    s := {TrimStr(s)} TrimRight(s);
    // truncate spaces at end by shortening fieldsize
    while (FieldSize > 0) and ((PChar(Src) + FieldSize - 1)^ = ' ') do
      dec(FieldSize);
    // if not string field, truncate spaces at beginning too
    if DataType <> ftString then
      while (FieldSize > 0) and (PChar(Src)^ = ' ') do
      begin
        inc(PChar(Src));
        dec(FieldSize);
      end;
    // return if field is empty
    Result := FieldSize > 0;
    if Result and (Dst <> nil) then     // data not needed if Result= false or Dst=nil
      case DataType of
      ftBoolean:
        begin
          // in DBase- FileDescription lowercase t is allowed too
          // with asking for Result= true s must be longer then 0
          // else an AV occurs, maybe field is NULL
          if (PChar(Src)^ = 'T') or (PChar(Src)^ = 't') then
            PWord(Dst)^ := 1
          else
            PWord(Dst)^ := 0;
        end;
      ftSmallInt:
        PSmallInt(Dst)^ := GetIntFromStrLength(Src, FieldSize, 0);
{$ifdef SUPPORT_INT64}
      ftLargeInt:
        PLargeInt(Dst)^ := GetInt64FromStrLength(Src, FieldSize, 0);
{$endif}
      ftInteger:
        PInteger(Dst)^ := GetIntFromStrLength(Src, FieldSize, 0);
      ftFloat, ftCurrency:
        PDouble(Dst)^ := DbfStrToFloat(Src, FieldSize);
      ftDate, ftDateTime:
        begin
          // get year, month, day
          ldy := GetIntFromStrLength(PChar(Src) + 0, 4, 1);
          ldm := GetIntFromStrLength(PChar(Src) + 4, 2, 1);
          ldd := GetIntFromStrLength(PChar(Src) + 6, 2, 1);
          //if (ly<1900) or (ly>2100) then ly := 1900;
          //Year from 0001 to 9999 is possible
          //everyting else is an error, an empty string too
          //Do DateCorrection with Delphis possibillities for one or two digits
          if (ldy < 100) and (PChar(Src)[0] = #32) and (PChar(Src)[1] = #32) then
            CorrectYear(ldy);
          try
            date := EncodeDate(ldy, ldm, ldd);
          except
            date := 0;
          end;

          // time stored too?
          if (AFieldDef.FieldType = ftDateTime) and (DataType = ftDateTime) then
          begin
            // get hour, minute, second
            lth := GetIntFromStrLength(PChar(Src) + 8,  2, 1);
            ltm := GetIntFromStrLength(PChar(Src) + 10, 2, 1);
            lts := GetIntFromStrLength(PChar(Src) + 12, 2, 1);
            // encode
            try
              date := date + EncodeTime(lth, ltm, lts, 0);
            except
              date := 0;
            end;
          end;

          SaveDateToDst;
        end;
      ftString:
        StrLCopy(Dst, Src, FieldSize);
    end else begin
      case DataType of
      ftString:
        if Dst <> nil then
          PChar(Dst)[0] := #0;
      end;
    end;
  end;
end;

procedure TDbfFile.UpdateNullField(Buffer: Pointer; AFieldDef: TDbfFieldDef; 
  Action: TUpdateNullField; WhichField: TNullFieldFlag);
var
  NullDst: pbyte;
  Mask: byte;
begin
  // this field has null setting capability...
  // ... but no Super Cow Powers.
  case WhichField of
  nfNullFlag:
    begin
      // Find out the byte where the length bit for the field is stored by doing
      // NullPosition shr3 (= NullPosition div 8)...
      NullDst := PByte(PChar(Buffer) + FNullField.Offset + (AFieldDef.NullPosition shr 3));
      // ... get the correct bit in the byte by the equivalent of
      // getting the bit number in that byte:
      // NullPosition and $7 (=mod 8)...
      // and going to the bit value in the byte (shl)
      Mask := 1 shl (AFieldDef.NullPosition and $7);
    end;
  nfVarlengthFlag:
    begin
      NullDst := PByte(PChar(Buffer) + FNullField.Offset + (AFieldDef.VarLengthPosition shr 3));
      Mask := 1 shl (AFieldDef.VarLengthPosition and $7);
    end;
  end;

  if Action = unfSet then
  begin
    // set flag
    NullDst^ := NullDst^ or Mask;
  end else begin //unfClear
    // clear flag
    NullDst^ := NullDst^ and not Mask;
  end;
end;

procedure TDbfFile.SetFieldData(Column: Integer; DataType: TFieldType; 
  Src, Dst: Pointer; NativeFormat: boolean);
const
  IsBlobFieldToPadChar: array[Boolean] of Char = (#32, '0');
  SrcNilToUpdateNullField: array[boolean] of TUpdateNullField = (unfClear, unfSet);
var
  DstRecord: Pointer;
  FieldSize,FieldPrec: Integer;
  TempFieldDef: TDbfFieldDef;
  Len: Integer;
  IntValue: dword;
  year, month, day: Word;
  hour, minute, sec, msec: Word;
  date: TDateTime;
  timeStamp: TTimeStamp;
  asciiContents: boolean;

  procedure LoadDateFromSrc;
  begin
    if not NativeFormat then
    begin
      // Delphi 5, new format, passes a TDateTime
      date := PDateTime(Src)^;
    end else begin
      // Delphi 3 and 4, old "native" format, pass a TDateTimeRec with a time stamp
      //  date = integer
      //  datetime = msecs == BDETimeStampToDateTime as we implemented it
      if DataType = ftDateTime then
      begin
        date := PDouble(Src)^;
      end else begin
        timeStamp.Time := 0;
        timeStamp.Date := PLongInt(Src)^;
        date := TimeStampToDateTime(timeStamp);
      end;
    end;
  end;

begin
  TempFieldDef := TDbfFieldDef(FFieldDefs.Items[Column]);
  FieldSize := TempFieldDef.Size;
  FieldPrec := TempFieldDef.Precision;

  DstRecord:=Dst; //beginning of record
  Dst := PChar(Dst) + TempFieldDef.Offset; //beginning of field

  // if src = nil then write empty field
  // symmetry with above loading code

  // Visual Foxpro has special _nullfield for flagging fields as `null'
  if (FNullField <> nil) and (TempFieldDef.NullPosition >= 0) then
    UpdateNullField(DstRecord, TempFieldDef, SrcNilToUpdateNullField[Src = nil],nfNullFlag);

  // copy field data to record buffer
  asciiContents := false;
  case TempFieldDef.NativeFieldType of
    '+', 'I' {autoincrement, integer}:
      begin
        if not(FDbfVersion in [xFoxPro,xVisualFoxPro]) then
        begin
          if Src = nil then
            IntValue := 0
          else
            IntValue := PDWord(Src)^ xor $80000000;
          Unaligned(PDWord(Dst)^) := SwapIntBE(IntValue);
        end else begin
          if Src = nil then
            Unaligned(PDWord(Dst)^) := 0
          else
            Unaligned(PDWord(Dst)^) := SwapIntLE(PDWord(Src)^);
        end;
      end;
    'O':
      begin
{$ifdef SUPPORT_INT64}
        if Src = nil then
        begin
          Unaligned(PInt64(Dst)^) := 0;
        end else begin
          if PDouble(Src)^ < 0 then
            Unaligned(PInt64(Dst)^) := not PInt64(Src)^
          else
            Unaligned(PDouble(Dst)^) := (PDouble(Src)^) * -1;
          SwapInt64BE(Dst, Dst);
        end;
{$endif}
      end;
    '@':
      begin
        if Src = nil then
        begin
{$ifdef SUPPORT_INT64}
          Unaligned(PInt64(Dst)^) := 0;
{$else}          
          Unaligned(PInteger(Dst)^) := 0;
          Unaligned(PInteger(PChar(Dst)+4)^) := 0;
{$endif}
        end else begin
          LoadDateFromSrc;
          if FDateTimeHandling = dtBDETimeStamp then
            date := DateTimeToBDETimeStamp(date);
          SwapInt64BE(@date, Dst);
        end;
      end;
    'T':
      begin
        // all binary zeroes -> empty datetime
        if Src = nil then
        begin
{$ifdef SUPPORT_INT64}
          Unaligned(PInt64(Dst)^) := 0;
{$else}          
          Unaligned(PInteger(Dst)^) := 0;
          Unaligned(PInteger(PChar(Dst)+4)^) := 0;
{$endif}          
        end else begin
          LoadDateFromSrc;
          timeStamp := DateTimeToTimeStamp(date);
          Unaligned(PInteger(Dst)^) := SwapIntLE(timeStamp.Date + JulianDateDelta);
          Unaligned(PInteger(PChar(Dst)+4)^) := SwapIntLE(timeStamp.Time);
        end;
      end;
    'Y':
      begin
{$ifdef SUPPORT_INT64}
        if Src = nil then
        begin
          Unaligned(PInt64(Dst)^) := 0;
        end else begin
          case DataType of
            ftCurrency:
              Unaligned(PInt64(Dst)^) := Trunc(PDouble(Src)^ * 10000);
            ftBCD:
              Unaligned(PCurrency(Dst)^) := PCurrency(Src)^;
          end;
          SwapInt64LE(Dst, Dst);
        end;
{$endif}
      end;
    'B' {(Visual) FoxPro Double}:
      begin
        if DbfVersion in [xFoxPro,xVisualFoxPro] then
        begin
          if Src = nil then
            Unaligned(PDouble(Dst)^) := 0
          else
            SwapInt64LE(Src, Dst);
        end else
          asciiContents := true;
      end;
    'M':
      begin
        if FieldSize = 4 then
        begin
          if Src = nil then
            Unaligned(PInteger(Dst)^) := 0
          else
            Unaligned(PInteger(Dst)^) := SwapIntLE(PInteger(Src)^);
        end else
          asciiContents := true;
      end;
    'Q': //Visual FoxPro varbinary
      begin
        // copy data, and update varlength flag/varlength byte in field data
        if Src = nil then
          Len := 0
        else
        begin
          Len := PWord(Src)^;
          if Len > FieldSize then
            Len := FieldSize;
        end;
        if Len < FieldSize then
        begin
          // Clear flag and store actual size byte in last data byte
          PByte(PChar(Dst)+TempFieldDef.Size-1)^:=Len;
          UpdateNullField(DstRecord, TempFieldDef, unfSet, nfVarlengthFlag);
        end
        else
        begin
          UpdateNullField(DstRecord, TempFieldDef, unfClear, nfVarlengthFlag);
        end;

        Move((Src+sizeof(word))^, Dst^, Len);
        // fill remaining data area with spaces, keeping room for size indicator if needed
        if Len=FieldSize then
          FillChar((PChar(Dst)+Len)^, FieldSize - Len, ' ')
        else
          FillChar((PChar(Dst)+Len)^, FieldSize - Len - 1, ' ');
      end;
    'V': //Visual FoxPro varchar
      begin
        // copy data, and update varlength flag/varlength byte in field data
        Len := StrLen(Src);
        if Len > FieldSize then
          Len := FieldSize;
        if Len < FieldSize then
        begin
          // Clear flag and store actual size byte in last data byte
          PByte(PChar(Dst)+TempFieldDef.Size-1)^:=Len;
          UpdateNullField(DstRecord, TempFieldDef, unfSet, nfVarlengthFlag);
        end
        else
        begin
          UpdateNullField(DstRecord, TempFieldDef, unfClear, nfVarlengthFlag);
        end;

        Move(Src^, Dst^, Len);
        // fill remaining data area with spaces, keeping room for size indicator if needed
        if Len=FieldSize then
          FillChar((PChar(Dst)+Len)^, FieldSize - Len, ' ')
        else
          FillChar((PChar(Dst)+Len)^, FieldSize - Len - 1, ' ');
      end
  else
    asciiContents := true;
  end;
  if asciiContents then
  begin
    if Src = nil then
    begin
      FillChar(Dst^, FieldSize, ' ');
    end else begin
      case DataType of
        ftBoolean:
          begin
            if PWord(Src)^ <> 0 then
              PChar(Dst)^ := 'T'
            else
              PChar(Dst)^ := 'F';
          end;
        ftSmallInt:
          GetStrFromInt_Width(PSmallInt(Src)^, FieldSize, PChar(Dst), #32);
{$ifdef SUPPORT_INT64}
        ftLargeInt:
          GetStrFromInt64_Width(PLargeInt(Src)^, FieldSize, PChar(Dst), #32);
{$endif}
        ftFloat, ftCurrency:
          FloatToDbfStr(PDouble(Src)^, FieldSize, FieldPrec, PChar(Dst));
        ftInteger:
          GetStrFromInt_Width(PInteger(Src)^, FieldSize, PChar(Dst),
            IsBlobFieldToPadChar[TempFieldDef.IsBlob]);
        ftDate, ftDateTime:
          begin
            LoadDateFromSrc;
            // decode
            DecodeDate(date, year, month, day);
            // format is yyyymmdd
            GetStrFromInt_Width(year,  4, PChar(Dst),   '0');
            GetStrFromInt_Width(month, 2, PChar(Dst)+4, '0');
            GetStrFromInt_Width(day,   2, PChar(Dst)+6, '0');
            // do time too if datetime
            if DataType = ftDateTime then
            begin
              DecodeTime(date, hour, minute, sec, msec);
              // format is hhmmss
              GetStrFromInt_Width(hour,   2, PChar(Dst)+8,  '0');
              GetStrFromInt_Width(minute, 2, PChar(Dst)+10, '0');
              GetStrFromInt_Width(sec,    2, PChar(Dst)+12, '0');
            end;
          end;
        ftString:
          begin
            // copy data
            Len := StrLen(Src);
            if Len > FieldSize then
              Len := FieldSize;
            Move(Src^, Dst^, Len);
            // fill remaining space with spaces
            FillChar((PChar(Dst)+Len)^, FieldSize - Len, ' ');
          end;
      end;  // case datatype
    end;
  end;
end;

procedure TDbfFile.InitDefaultBuffer;
var
  lRecordSize: integer;
  TempFieldDef: TDbfFieldDef;
  I: Integer;
begin
  lRecordSize := PDbfHdr(Header)^.RecordSize;
  // clear buffer (assume all string, fix specific fields later)
  //   note: Self.RecordSize is used for reading fielddefs too
  GetMem(FDefaultBuffer, lRecordSize+1);
  FillChar(FDefaultBuffer^, lRecordSize, ' ');
  
  // set nullflags field so that all fields are null (and var* fields marked as full)
  if FNullField <> nil then
    FillChar(PChar(FDefaultBuffer+FNullField.Offset)^, FNullField.Size, $FF);

  // check binary and default fields
  for I := 0 to FFieldDefs.Count-1 do
  begin
    TempFieldDef := FFieldDefs.Items[I];
    // binary (non-text) field? (foxpro memo fields are binary, but dbase not)
    if (TempFieldDef.NativeFieldType in ['I', 'O', '@', '+', '0', 'W', 'Y'])
        or ((TempFieldDef.NativeFieldType = 'M') and (TempFieldDef.Size = 4) {Visual FoxPro?}) then
      FillChar(PChar(FDefaultBuffer+TempFieldDef.Offset)^, TempFieldDef.Size, 0);
    // copy default value?
    if TempFieldDef.HasDefault then
    begin
      Move(TempFieldDef.DefaultBuf[0], FDefaultBuffer[TempFieldDef.Offset], TempFieldDef.Size);
      // clear the null flag, this field has a value
      if FNullField <> nil then
        UpdateNullField(FDefaultBuffer, TempFieldDef, unfClear, nfNullFlag);
      // Check for varbinary/varchar and if default matches it, then mark field as full
      if (TempFieldDef.VarLengthPosition>=0) then
        if (strlen(FDefaultBuffer)>=TempFieldDef.Size) then
          UpdateNullField(FDefaultBuffer, TempFieldDef, unfClear, nfVarlengthFlag)
        else
          begin
            // Set flag and store actual size byte in last data byte
            UpdateNullField(FDefaultBuffer, TempFieldDef, unfSet, nfVarlengthFlag);
            //todo: verify pointer use
            PByte(PChar(FDefaultBuffer)+TempFieldDef.Size)^:=strlen(FDefaultBuffer);
          end;
    end;
  end;
end;

procedure TDbfFile.InitRecord(DestBuf: TRecordBuffer);
begin
  if FDefaultBuffer = nil then
    InitDefaultBuffer;
  Move(FDefaultBuffer^, DestBuf^, RecordSize);
end;

procedure TDbfFile.ApplyAutoIncToBuffer(DestBuf: TRecordBuffer);
var
  TempFieldDef: TDbfFieldDef;
  I, NextVal, lAutoIncOffset: {LongWord} Cardinal;    {Delphi 3 does not know LongWord?}
begin
  if FAutoIncPresent then
  begin
    // if shared, reread header to find new autoinc values
    if NeedLocks then
    begin
      // lock header so nobody else can use this value
      LockPage(0, true);
    end;

    // find autoinc fields
    for I := 0 to FFieldDefs.Count-1 do
    begin
      TempFieldDef := FFieldDefs.Items[I];
      if (DbfVersion=xBaseVII) and
        (TempFieldDef.NativeFieldType = '+') then
      begin
        // read current auto inc, from header or field, depending on sharing
        lAutoIncOffset := sizeof(rDbfHdr) + sizeof(rEndFixedHdrVII) +
          FieldDescVII_AutoIncOffset + I * sizeof(rFieldDescVII);
        if NeedLocks then
        begin
          ReadBlock(@NextVal, 4, lAutoIncOffset);
          NextVal := SwapIntLE(NextVal);
        end else
          NextVal := TempFieldDef.AutoInc;
        // store to buffer, positive = high bit on, so flip it
        PCardinal(DestBuf+TempFieldDef.Offset)^ := SwapIntBE(NextVal or $80000000);
        // increase
        Inc(NextVal);
        TempFieldDef.AutoInc := NextVal;
        // write new value to header buffer
        PCardinal(FHeader+lAutoIncOffset)^ := SwapIntLE(NextVal);
      end
      else //No DBaseVII
      if (DbfVersion=xVisualFoxPro) and
        (TempFieldDef.AutoIncStep<>0) then
      begin
        // read current auto inc from field header
        NextVal:=TempFieldDef.AutoInc; //todo: is this correct
        PCardinal(DestBuf+TempFieldDef.Offset)^ := SwapIntBE(NextVal); //todo: is swapintbe correct?
        // Increase with step size
        NextVal:=NextVal+TempFieldDef.AutoIncStep;
        // write new value back
        TempFieldDef.AutoInc:=NextVal;
      end;
    end;

    // write modified header (new autoinc values) to file
    WriteHeader;
    
    // release lock if locked
    if NeedLocks then
      UnlockPage(0);
  end;
end;

procedure TDbfFile.TryExclusive;
var
  I: Integer;
begin
  inherited;

  // exclusive succeeded? open index & memo exclusive too
  if Mode in [pfMemoryCreate..pfExclusiveOpen] then
  begin
    // indexes
    for I := 0 to FIndexFiles.Count - 1 do
      TPagedFile(FIndexFiles[I]).TryExclusive;
    // memo
    if FMemoFile <> nil then
      FMemoFile.TryExclusive;
  end;
end;

procedure TDbfFile.EndExclusive;
var
  I: Integer;
begin
  // end exclusive on index & memo too
  for I := 0 to FIndexFiles.Count - 1 do
    TPagedFile(FIndexFiles[I]).EndExclusive;
  // memo
  if FMemoFile <> nil then
    FMemoFile.EndExclusive;
  // dbf file
  inherited;
end;

procedure TDbfFile.OpenIndex(IndexName, IndexField: string; CreateIndex: Boolean; Options: TIndexOptions);
  //
  // assumes IndexName is not empty
  //
const
  // memcr, memop, excr, exopen, rwcr, rwopen, rdonly
  IndexOpenMode: array[boolean, pfMemoryCreate..pfReadOnly] of TPagedFileMode =
   ((pfMemoryCreate, pfMemoryCreate, pfExclusiveOpen, pfExclusiveOpen, pfReadWriteOpen, pfReadWriteOpen,
     pfReadOnly),
    (pfMemoryCreate, pfMemoryCreate, pfExclusiveCreate, pfExclusiveCreate, pfReadWriteCreate, pfReadWriteCreate,
     pfReadOnly));
var
  lIndexFile: TIndexFile;
  lIndexFileName: string;
  createMdxFile: Boolean;
  tempExclusive: boolean;
  addedIndexFile: Integer;
  addedIndexName: Integer;
begin
  // init
  addedIndexFile := -1;
  addedIndexName := -1;
  createMdxFile := false;
  // index already opened?
  lIndexFile := GetIndexByName(IndexName);
  if (lIndexFile <> nil) and (lIndexFile = FMdxFile) and CreateIndex then
  begin
    // index already exists in MDX file
    // delete it to save space, this causes a repage
    FMdxFile.DeleteIndex(IndexName);
    // index no longer exists
    lIndexFile := nil;
  end;
  if (lIndexFile = nil) and (IndexName <> EmptyStr) then
  begin
    // check if no extension, then create MDX index
    if Length(ExtractFileExt(IndexName)) = 0 then
    begin
      // check if mdx index already opened
      if FMdxFile <> nil then
      begin
        lIndexFileName := EmptyStr;
        lIndexFile := FMdxFile;
      end else begin
        lIndexFileName := ChangeFileExt(FileName, '.mdx');
        createMdxFile := true;
      end;
    end else begin
      lIndexFileName := IndexName;
    end;
    // do we need to open / create file?
    if lIndexFileName <> EmptyStr then
    begin
      // try to open / create the file
      lIndexFile := TIndexFile.Create(Self);
      lIndexFile.FileName := lIndexFileName;
      lIndexFile.Mode := IndexOpenMode[CreateIndex, Mode];
      lIndexFile.AutoCreate := CreateIndex or (Length(IndexField) > 0);
      if (Mode in [pfMemoryOpen,pfMemoryCreate]) then
      begin
        if FIndexStream = nil then
          FIndexStream := TMemoryStream.Create;
        lIndexFile.Stream := FIndexStream;
      end;
      lIndexFile.CodePage := UseCodePage;
      lIndexFile.OnLocaleError := FOnLocaleError;
      lIndexFile.Open;
      // index file ready for use?
      if not lIndexFile.ForceClose then
      begin
        // if we had to create the index, store that info
        CreateIndex := lIndexFile.FileCreated;
        // check if trying to create empty index
        if CreateIndex and (IndexField = EmptyStr) then
        begin
          FreeAndNil(lIndexFile);
          CreateIndex := false;
          createMdxFile := false;
        end else begin
          // add new index file to list
          addedIndexFile := FIndexFiles.Add(lIndexFile);
        end;
        // created accompanying mdx file?
        if createMdxFile then
          FMdxFile := lIndexFile;
      end else begin
        // asked to close! close file
        FreeAndNil(lIndexFile);
      end;
    end;

    // check if file succesfully opened
    if lIndexFile <> nil then
    begin
      // add index to list
      addedIndexName := FIndexNames.AddObject(IndexName, lIndexFile);
    end;
  end;
  // create it or open it?
  if lIndexFile <> nil then
  begin
    if not CreateIndex then
      if lIndexFile = FMdxFile then
        CreateIndex := lIndexFile.IndexOf(IndexName) < 0;
    if CreateIndex then
    begin
      // try get exclusive mode
      tempExclusive := IsSharedAccess;
      if tempExclusive then TryExclusive;
      // always uppercase index expression
      IndexField := AnsiUpperCase(IndexField);
      try
        try
          // create index if asked
          lIndexFile.CreateIndex(IndexField, IndexName, Options);
          // add all records
          PackIndex(lIndexFile, IndexName);
          // if we wanted to open index readonly, but we created it, then reopen
          if Mode = pfReadOnly then
          begin
            lIndexFile.CloseFile;
            lIndexFile.Mode := pfReadOnly;
            lIndexFile.OpenFile;
          end;
          // if mdx file just created, write changes to dbf header
          // set MDX flag to true
          PDbfHdr(Header)^.MDXFlag := 1;
          WriteHeader;
        except
          on EDbfError do
          begin
            // :-( need to undo 'damage'....
            // remove index from list(s) if just added
            if addedIndexFile >= 0 then
              FIndexFiles.Delete(addedIndexFile);
            if addedIndexName >= 0 then
              FIndexNames.Delete(addedIndexName);
            // if no file created, do not destroy!
            if addedIndexFile >= 0 then
            begin
              lIndexFile.Close;
              Sysutils.DeleteFile(lIndexFileName);
              if FMdxFile = lIndexFile then
                FMdxFile := nil;
              lIndexFile.Free;
            end;
            raise;
          end;
        end;
      finally
        // return to previous mode
        if tempExclusive then EndExclusive;
      end;
    end;
  end;
end;

procedure TDbfFile.PackIndex(lIndexFile: TIndexFile; AIndexName: string);
var
  prevMode: TIndexUpdateMode;
  prevIndex: string;
  cur, last: Integer;
{$ifdef USE_CACHE}
  prevCache: Integer;
{$endif}
begin
  // save current mode in case we change it
  prevMode := lIndexFile.UpdateMode;
  prevIndex := lIndexFile.IndexName;
  // check if index specified
  if Length(AIndexName) > 0 then
  begin
    // only pack specified index, not all
    lIndexFile.IndexName := AIndexName;
    lIndexFile.ClearIndex;
    lIndexFile.UpdateMode := umCurrent;
  end else begin
    lIndexFile.IndexName := EmptyStr;
    lIndexFile.Clear;
    lIndexFile.UpdateMode := umAll;
  end;
  // prepare update
  cur := 1;
  last := RecordCount;
{$ifdef USE_CACHE}
  BufferAhead := true;
  prevCache := lIndexFile.CacheSize;
  lIndexFile.CacheSize := GetFreeMemory;
  if lIndexFile.CacheSize < 16384 * 1024 then
    lIndexFile.CacheSize := 16384 * 1024;
{$endif}
  try
    try
      while cur <= last do
      begin
        ReadRecord(cur, FPrevBuffer);
        lIndexFile.Insert(cur, FPrevBuffer);
        inc(cur);
      end;
    except
      on E: EDbfError do
      begin
        lIndexFile.DeleteIndex(lIndexFile.IndexName);
        raise;
      end;
    end;
  finally
    // restore previous mode
{$ifdef USE_CACHE}
    BufferAhead := false;
    lIndexFile.BufferAhead := true;
{$endif}
    lIndexFile.Flush;
{$ifdef USE_CACHE}
    lIndexFile.BufferAhead := false;
    lIndexFile.CacheSize := prevCache;
{$endif}
    lIndexFile.UpdateMode := prevMode;
    lIndexFile.IndexName := prevIndex;
  end;
end;

procedure TDbfFile.RepageIndex(AIndexFile: string);
var
  lIndexNo: Integer;
begin
  // DBF MDX index?
  if Length(AIndexFile) = 0 then
  begin
    if FMdxFile <> nil then
    begin
      // repage attached mdx
      FMdxFile.RepageFile;
    end;
  end else begin
    // search index file
    lIndexNo := FIndexNames.IndexOf(AIndexFile);
    // index found?
    if lIndexNo >= 0 then
      TIndexFile(FIndexNames.Objects[lIndexNo]).RepageFile;
  end;
end;

procedure TDbfFile.CompactIndex(AIndexFile: string);
var
  lIndexNo: Integer;
begin
  // DBF MDX index?
  if Length(AIndexFile) = 0 then
  begin
    if FMdxFile <> nil then
    begin
      // repage attached mdx
      FMdxFile.CompactFile;
    end;
  end else begin
    // search index file
    lIndexNo := FIndexNames.IndexOf(AIndexFile);
    // index found?
    if lIndexNo >= 0 then
      TIndexFile(FIndexNames.Objects[lIndexNo]).CompactFile;
  end;
end;

procedure TDbfFile.CloseIndex(AIndexName: string);
var
  lIndexNo: Integer;
  lIndex: TIndexFile;
begin
  // search index file
  lIndexNo := FIndexNames.IndexOf(AIndexName);
  // don't close mdx file
  if (lIndexNo >= 0) then
  begin
    // get index pointer
    lIndex := TIndexFile(FIndexNames.Objects[lIndexNo]);
    if (lIndex <> FMdxFile) then
    begin
      // close file
      lIndex.Free;
      // remove from lists
      FIndexFiles.Remove(lIndex);
      FIndexNames.Delete(lIndexNo);
      // was this the current index?
      if (FCurIndex = lIndexNo) then
      begin
        FCurIndex := -1;
        //FCursor := FDbfCursor;
      end;
    end;
  end;
end;

function TDbfFile.DeleteIndex(const AIndexName: string): Boolean;
var
  lIndexNo: Integer;
  lIndex: TIndexFile;
  lFileName: string;
begin
  // search index file
  lIndexNo := FIndexNames.IndexOf(AIndexName);
  Result := lIndexNo >= 0;
  // found index?
  if Result then
  begin
    // can only delete indexes from MDX files
    lIndex := TIndexFile(FIndexNames.Objects[lIndexNo]);
    if lIndex = FMdxFile then
    begin
      lIndex.DeleteIndex(AIndexName);
      // remove it from the list
      FIndexNames.Delete(lIndexNo);
      // no more MDX indexes?
      lIndexNo := FIndexNames.IndexOfObject(FMdxFile);
      if lIndexNo = -1 then
      begin
        // no MDX indexes left
        lIndexNo := FIndexFiles.IndexOf(FMdxFile);
        if lIndexNo >= 0 then
          FIndexFiles.Delete(lIndexNo);
        lFileName := FMdxFile.FileName;
        FreeAndNil(FMdxFile);
        // erase file
        Sysutils.DeleteFile(lFileName);
        // clear mdx flag
        PDbfHdr(Header)^.MDXFlag := 0;
        WriteHeader;
      end;
    end else begin
      // close index first
      CloseIndex(AIndexName);
      // delete file from disk
      SysUtils.DeleteFile(AIndexName);
    end;
  end;
end;

<<<<<<< HEAD
function TDbfFile.Insert(Buffer: TRecordBuffer): integer;
=======
function TDbfFile.Insert(Buffer: PChar): integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type
  TErrorContext = (ecNone, ecInsert, ecWriteIndex, ecWriteDbf);
var
  newRecord: Integer;
  lIndex: TIndexFile;

  procedure RollBackIndexesAndRaise(Count: Integer; ErrorContext: TErrorContext);
  var
    errorMsg: string;
    I: Integer;
  begin
    // rollback committed indexes
    for I := 0 to Count-1 do
    begin
      lIndex := TIndexFile(FIndexFiles.Items[I]);
      lIndex.Delete(newRecord, Buffer);
    end;

    // reset any dbf file error
    ResetError;

    // if part of indexes committed -> always index error msg
    // if error while rolling back index -> index error msg
    case ErrorContext of
      ecInsert: begin TIndexFile(FIndexFiles.Items[Count]).InsertError; exit; end;
      ecWriteIndex: errorMsg := STRING_WRITE_INDEX_ERROR;
      ecWriteDbf: errorMsg := STRING_WRITE_ERROR;
    end;
    raise EDbfWriteError.Create(errorMsg);
  end;

var
  I: Integer;
  error: TErrorContext;
begin
  // get new record index
  Result := 0;
  newRecord := RecordCount+1;
  // lock record so we can write data
  while not LockPage(newRecord, false) do
    Inc(newRecord);
  // write autoinc value
  ApplyAutoIncToBuffer(Buffer);
  error := ecNone;
  I := 0;
  while I < FIndexFiles.Count do
  begin
    lIndex := TIndexFile(FIndexFiles.Items[I]);
    if not lIndex.Insert(newRecord, Buffer) then
      error := ecInsert;
    if lIndex.WriteError then
      error := ecWriteIndex;
    if error <> ecNone then
    begin
      // if there's an index write error, I shouldn't
      // try to write the dbf header and the new record,
      // but raise an exception right away
      UnlockPage(newRecord);
      RollBackIndexesAndRaise(I, ecWriteIndex);
    end;
    Inc(I);
  end;

  // indexes ok -> continue inserting
  // update header record count
  LockPage(0, true);
  // read current header
  ReadHeader;
  // increase current record count
  Inc(PDbfHdr(Header)^.RecordCount);
  // write header to disk
  WriteHeader;
  // done with header
  UnlockPage(0);

  if WriteError then
  begin
    // couldn't write header, so I shouldn't
    // even try to write the record.
    //
    // At this point I should "roll back"
    // the already written index records.
    // if this fails, I'm in deep trouble!
    UnlockPage(newRecord);
    RollbackIndexesAndRaise(FIndexFiles.Count, ecWriteDbf);
  end;

  // write locking info
  if FLockField <> nil then
    WriteLockInfo(Buffer);
  // write buffer to disk
  WriteRecord(newRecord, Buffer);
  // done updating, unlock
  UnlockPage(newRecord);
  // error occurred while writing?
  if WriteError then
  begin
    // The record couldn't be written, so
    // the written index records and the
    // change to the header have to be
    // rolled back
    LockPage(0, true);
    ReadHeader;
    Dec(PDbfHdr(Header)^.RecordCount);
    WriteHeader;
    UnlockPage(0);
<<<<<<< HEAD
    // roll back indexes, too
=======
    // roll back indexes too
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    RollbackIndexesAndRaise(FIndexFiles.Count, ecWriteDbf);
  end else
    Result := newRecord;
end;

procedure TDbfFile.WriteLockInfo(Buffer: TRecordBuffer);
//
// *) assumes FHasLockField = true
//
var
  year, month, day, hour, minute, sec, msec: Word;
  lockoffset: integer;
begin
  // increase change count
  lockoffset := FLockField.Offset;
  Inc(PWord(Buffer+lockoffset)^);
  // set time
  DecodeDate(Now(), year, month, day);
  DecodeTime(Now(), hour, minute, sec, msec);
  Buffer[lockoffset+2] := TRecordBufferBaseType(hour);
  Buffer[lockoffset+3] := TRecordBufferBaseType(minute);
  Buffer[lockoffset+4] := TRecordBufferBaseType(sec);
  // set date
  Buffer[lockoffset+5] := TRecordBufferBaseType(year - 1900);
  Buffer[lockoffset+6] := TRecordBufferBaseType(month);
  Buffer[lockoffset+7] := TRecordBufferBaseType(day);
  // set name
  FillChar(Buffer[lockoffset+8], FLockField.Size-8, ' ');
  Move(DbfGlobals.UserName[1], Buffer[lockoffset+8], FLockUserLen);
end;

procedure TDbfFile.LockRecord(RecNo: Integer; Buffer: TRecordBuffer);
begin
  if LockPage(RecNo, false) then
  begin
    // reread data
    ReadRecord(RecNo, Buffer);
    // store previous data for updating indexes
    Move(Buffer^, FPrevBuffer^, RecordSize);
    // lock succeeded, update lock info, if field present
    if FLockField <> nil then
    begin
      // update buffer
      WriteLockInfo(Buffer);
      // write to disk
      WriteRecord(RecNo, Buffer);
    end;
  end else
    raise EDbfError.Create(STRING_RECORD_LOCKED);
end;

procedure TDbfFile.UnlockRecord(RecNo: Integer; Buffer: TRecordBuffer);
var
  I: Integer;
  lIndex, lErrorIndex: TIndexFile;
begin
  // update indexes, possible key violation
  I := 0;
  while I < FIndexFiles.Count do
  begin
    lIndex := TIndexFile(FIndexFiles.Items[I]);
    if not lIndex.Update(RecNo, FPrevBuffer, Buffer) then
    begin
      // error -> rollback
      lErrorIndex := lIndex;
      while I > 0 do
      begin
        Dec(I);
        lIndex := TIndexFile(FIndexFiles.Items[I]);
        lIndex.Update(RecNo, Buffer, FPrevBuffer);
      end;
      lErrorIndex.InsertError;
    end;
    Inc(I);
  end;
  // write new record buffer, all keys ok
  WriteRecord(RecNo, Buffer);
  // done updating, unlock
  UnlockPage(RecNo);
end;

procedure TDbfFile.RecordDeleted(RecNo: Integer; Buffer: TRecordBuffer);
var
  I: Integer;
  lIndex: TIndexFile;
begin
  // notify indexes: record deleted
  for I := 0 to FIndexFiles.Count - 1 do
  begin
    lIndex := TIndexFile(FIndexFiles.Items[I]);
    lIndex.RecordDeleted(RecNo, Buffer);
  end;
end;

procedure TDbfFile.RecordRecalled(RecNo: Integer; Buffer: TRecordBuffer);
var
  I: Integer;
  lIndex, lErrorIndex: TIndexFile;
begin
  // notify indexes: record recalled
  I := 0;
  while I < FIndexFiles.Count do
  begin
    lIndex := TIndexFile(FIndexFiles.Items[I]);
    if not lIndex.RecordRecalled(RecNo, Buffer) then
    begin
      lErrorIndex := lIndex;
      while I > 0 do
      begin
        Dec(I);
        lIndex.RecordDeleted(RecNo, Buffer);
      end;
      lErrorIndex.InsertError;
    end;
    Inc(I);
  end;
end;

procedure TDbfFile.SetRecordSize(NewSize: Integer);
begin
  if NewSize <> RecordSize then
  begin
    if FPrevBuffer <> nil then
      FreeMemAndNil(Pointer(FPrevBuffer));

    if NewSize > 0 then
      GetMem(FPrevBuffer, NewSize);
  end;
  inherited;
end;

function TDbfFile.GetIndexByName(AIndexName: string): TIndexFile;
var
  I: Integer;
begin
  I := FIndexNames.IndexOf(AIndexName);
  if I >= 0 then
    Result := TIndexFile(FIndexNames.Objects[I])
  else
    Result := nil;
end;

//====================================================================
// TDbfCursor
//====================================================================
constructor TDbfCursor.Create(DbfFile: TDbfFile);
begin
  inherited Create(DbfFile);
end;

function TDbfCursor.Next: Boolean;
begin
  if TDbfFile(PagedFile).IsRecordPresent(FPhysicalRecNo) then
  begin
    inc(FPhysicalRecNo);
    Result := TDbfFile(PagedFile).IsRecordPresent(FPhysicalRecNo);
  end else begin
    FPhysicalRecNo := TDbfFile(PagedFile).CachedRecordCount + 1;
    Result := false;
  end;
end;

function TDbfCursor.Prev: Boolean;
begin
  if FPhysicalRecNo > 0 then
    dec(FPhysicalRecNo)
  else
    FPhysicalRecNo := 0;
  Result := FPhysicalRecNo > 0;
end;

procedure TDbfCursor.First;
begin
  FPhysicalRecNo := 0;
end;

procedure TDbfCursor.Last;
var
  max: Integer;
begin
  max := TDbfFile(PagedFile).RecordCount;
  if max = 0 then
    FPhysicalRecNo := 0
  else
    FPhysicalRecNo := max + 1;
end;

function TDbfCursor.GetPhysicalRecNo: Integer;
begin
  Result := FPhysicalRecNo;
end;

procedure TDbfCursor.SetPhysicalRecNo(RecNo: Integer);
begin
  FPhysicalRecNo := RecNo;
end;

function TDbfCursor.GetSequentialRecordCount: Integer;
begin
  Result := TDbfFile(PagedFile).RecordCount;
end;

function TDbfCursor.GetSequentialRecNo: Integer;
begin
  Result := FPhysicalRecNo;
end;

procedure TDbfCursor.SetSequentialRecNo(RecNo: Integer);
begin
  FPhysicalRecNo := RecNo;
end;

// codepage enumeration procedure
var
  TempCodePageList: TList;

// LPTSTR = PChar ok?

function CodePagesProc(CodePageString: PChar): Cardinal; stdcall;
begin
  // add codepage to list
  TempCodePageList.Add(Pointer(GetIntFromStrLength(CodePageString, StrLen(CodePageString), -1)));

  // continue enumeration
  Result := 1;
end;

//====================================================================
// TDbfGlobals
//====================================================================
constructor TDbfGlobals.Create;
begin
  FCodePages := TList.Create;
  FDefaultOpenCodePage := GetACP;
  // the following sets FDefaultCreateLangId
  DefaultCreateCodePage := GetACP;
  FCurrencyAsBCD := true;
  // determine which code pages are installed
  TempCodePageList := FCodePages;
  EnumSystemCodePages(@CodePagesProc, {CP_SUPPORTED} CP_INSTALLED);
  TempCodePageList := nil;
  InitUserName;
end;

procedure TDbfGlobals.InitUserName;
{$ifdef FPC}
{$ifndef WINDOWS}
 {$IFNDEF OS2}
var
  TempName: UTSName;
 {$ENDIF OS2}
{$endif}
{$endif}
begin
{$ifdef WINDOWS}
{$ifdef wince}
  FUserName:='cedevice';
  FUserNameLen:=Length(FUserName);
{$else}
  FUserNameLen := MAX_COMPUTERNAME_LENGTH+1;
  SetLength(FUserName, FUserNameLen);
  Windows.GetComputerName(PChar(FUserName), 
    {$ifdef DELPHI_3}Windows.DWORD({$endif}
      FUserNameLen
    {$ifdef DELPHI_3}){$endif}
    );
  SetLength(FUserName, FUserNameLen);
{$endif wince}
{$else}
{$ifdef FPC}
 {$IFDEF OS2}
  FUserName := GetEnvironmentVariable ('HOSTNAME');
 {$ELSE OS2}
  FpUname(TempName);
  FUserName := TempName.machine;
 {$ENDIF OS2}
  FUserNameLen := Length(FUserName);
{$endif}  
{$endif}
end;

destructor TDbfGlobals.Destroy; {override;}
begin
  FCodePages.Free;
end;

function TDbfGlobals.GetDefaultCreateCodePage: Integer;
begin
  Result := LangId_To_CodePage[FDefaultCreateLangId];
end;

procedure TDbfGlobals.SetDefaultCreateCodePage(NewCodePage: Integer);
begin
  FDefaultCreateLangId := ConstructLangId(NewCodePage, GetUserDefaultLCID, false);
end;

function TDbfGlobals.CodePageInstalled(ACodePage: Integer): Boolean;
begin
  Result := FCodePages.IndexOf(Pointer(ACodePage)) >= 0;
end;

initialization
finalization
  FreeAndNil(DbfGlobals);


(*
  Not implemented yet (encrypted cdx is undocumented;
  unencrypted cdx could be implemented)
  TFoxCDXHeader         = Record
    PointerRootNode     : Integer;
    PointerFreeList     : Integer;
    Reserved_8_11       : Cardinal;
    KeyLength           : Word;
    IndexOption         : Byte;
    IndexSignature      : Byte;
    Reserved_Null       : TFoxReservedNull;
    SortOrder           : Word;
    TotalExpressionLen  : Word;
    ForExpressionLen    : Word;
    Reserved_506_507    : Word;
    KeyExpressionLen    : Word;
    KeyForExpression    : TKeyForExpression;
  End;
  PFoxCDXHeader         = ^TFoxCDXHeader;

  TFoxCDXNodeCommon     = Record
    NodeAttributes      : Word;
    NumberOfKeys        : Word;
    PointerLeftNode     : Integer;
    PointerRightNode    : Integer;
  End;

  TFoxCDXNodeNonLeaf    = Record
    NodeCommon          : TFoxCDXNodeCommon;
    TempBlock           : Array [12..511] of Byte;
  End;
  PFoxCDXNodeNonLeaf    = ^TFoxCDXNodeNonLeaf;

  TFoxCDXNodeLeaf       = Packed Record
    NodeCommon          : TFoxCDXNodeCommon;
    BlockFreeSpace      : Word;
    RecordNumberMask    : Integer;
    DuplicateCountMask  : Byte;
    TrailByteCountMask  : Byte;
    RecNoBytes          : Byte;
    DuplicateCountBytes : Byte;
    TrailByteCountBytes : Byte;
    HoldingByteCount    : Byte;
    DataBlock           : TDataBlock;
  End;
  PFoxCDXNodeLeaf       = ^TFoxCDXNodeLeaf;

*)

end.

