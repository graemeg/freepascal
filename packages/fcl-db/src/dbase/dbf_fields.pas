unit dbf_fields;

interface

{$I dbf_common.inc}

uses
  Classes,
  SysUtils,
  db,
  dbf_common,
  dbf_str;

type
  PDbfFieldDef = ^TDbfFieldDef;

  { TDbfFieldDef }

  TDbfFieldDef = class(TCollectionItem)
  private
    FAutoIncStep: Integer;
    FFieldName: string;
    FFieldType: TFieldType;
    FIsSystemField: Boolean;
    FVarLengthPosition: integer;
    FNativeFieldType: TDbfFieldType;
    FDefaultBuf: PChar;
    FMinBuf: PChar;
    FMaxBuf: PChar;
    FSize: Integer;
    FPrecision: Integer;
    FHasDefault: Boolean;
    FHasMin: Boolean;
    FHasMax: Boolean;
    FAllocSize: Integer;
    FCopyFrom: Integer;
    FOffset: Integer;
    FAutoInc: Cardinal;
    FRequired: Boolean;
    FIsLockField: Boolean;
    FNullPosition: integer;

    function  GetDbfVersion: TXBaseVersion;
    procedure SetNativeFieldType(lFieldType: TDbfFieldType);
    procedure SetFieldType(lFieldType: TFieldType);
    procedure SetSize(lSize: Integer);
    procedure SetPrecision(lPrecision: Integer);
    // Converts VCL/LCL field types to dbf native field type markers ('C' etc)
    procedure VCLToNative;
    // Converts dbf native field type markers ('C' etc) to VCL/LCL field types
    procedure NativeToVCL;
    procedure FreeBuffers;
  protected
    function  GetDisplayName: string; override;
    procedure AssignTo(Dest: TPersistent); override;
    // File is compatible with this database product
    property DbfVersion: TXBaseVersion read GetDbfVersion;
  public
    constructor Create(ACollection: TCollection); override;
    destructor Destroy; override;

    procedure Assign(Source: TPersistent); override;
    procedure AssignDb(DbSource: TFieldDef);

    // Checks and adjusts field size & precision
    procedure CheckSizePrecision;
    procedure SetDefaultSize;
    procedure AllocBuffers;
    // Yes if field is a blob/memo type field (storage in external file)
    function  IsBlob: Boolean;

    property DefaultBuf: PChar read FDefaultBuf;
    property MinBuf: PChar read FMinBuf;
    property MaxBuf: PChar read FMaxBuf;
    property HasDefault: Boolean read FHasDefault write FHasDefault;
    property HasMin: Boolean read FHasMin write FHasMin;
    property HasMax: Boolean read FHasMax write FHasMax;
    // Distance of field from beginning of record
    property Offset: Integer read FOffset write FOffset;
    // Value for autoinc
    property AutoInc: Cardinal read FAutoInc write FAutoInc;
    // Step size for autoinc (Visual FoxPro only)
    property AutoIncStep: Integer read FAutoIncStep write FAutoIncStep;
    // Field contains lock data (not a normal field)
    property IsLockField: Boolean read FIsLockField write FIsLockField;
    // Field is a system, hidden field (Visual FoxPro supported only)
    property IsSystemField: Boolean read FIsSystemField write FIsSystemField;
    property CopyFrom: Integer read FCopyFrom write FCopyFrom;
  published
    property FieldName: string     read FFieldName write FFieldName;
    // VCL/LCL field type mapped to this field
    property FieldType: TFieldType read FFieldType write SetFieldType;
    // If using varchar/varbinary/var...:
    // VFP uses a varlength bit in _NullFields in physical order (bit number corresponds to physical order)
    // If flag=1, the actually used length/size is stored in the last data byte of the field
    // If the var* field is nullable, 2 bits are used:
    // lower bit number is varlength, next is null flag.
    // Note: VarLengthPosition property is 0 based
    // http://msdn.microsoft.com/en-us/library/st4a0s68%28v=VS.80%29.aspx
    property VarLengthPosition: integer read FVarLengthPosition write FVarLengthPosition;
    // Native dbf field type (C character etc)
    property NativeFieldType: TDbfFieldType read FNativeFieldType write SetNativeFieldType;
    // Size in physical dbase file.
    // Note: this often differs from the VCL field sizes
    property Size: Integer         read FSize write SetSize;
    // Visual FoxPro: position of field null flag in _NullFields field
    // Reflects the physical field order, except if varchar/varbinary/var* fields
    // are used (see VarLengthPosition property for details)
    // Note: NullPosition property is 0 based
    // http://msdn.microsoft.com/en-us/library/st4a0s68%28v=VS.80%29.aspx
    property NullPosition: integer read FNullPosition write FNullPosition;
    property Precision: Integer    read FPrecision write SetPrecision;
    property Required: Boolean     read FRequired  write FRequired;
  end;

  TDbfFieldDefs = class(TCollection)
  private
    FOwner: TPersistent;
    FDbfVersion: TXBaseVersion;
    function GetItem(Idx: Integer): TDbfFieldDef;
  protected
    function GetOwner: TPersistent; override;
  public
    constructor Create(Owner: TPersistent);

{$ifdef SUPPORT_DEFAULT_PARAMS}
    procedure Add(const Name: string; DataType: TFieldType; Size: Integer = 0; Required: Boolean = False);
{$else}
    procedure Add(const Name: string; DataType: TFieldType; Size: Integer; Required: Boolean);
{$endif}
    function AddFieldDef: TDbfFieldDef;

    property Items[Idx: Integer]: TDbfFieldDef read GetItem;
    property DbfVersion: TXBaseVersion read FDbfVersion write FDbfVersion;
  end;

implementation

uses
  dbf_dbffile;      // for dbf header structures

{$I dbf_struct.inc}

const
(*
The theory for Delphi/FPC is:
    ftSmallint  16 bits = -32768 to 32767
                          123456 = 6 digit max theorically
                          DIGITS_SMALLINT = 6;
    ftInteger  32 bits = -2147483648 to 2147483647
                         12345678901 = 11 digits max
                         DIGITS_INTEGER = 11;
    ftLargeInt 64 bits = -9223372036854775808 to 9223372036854775807
                         12345678901234567890 = 20 digits max
                         DIGITS_LARGEINT = 20;

But in fact if I accept 6 digits into a ftSmallInt then tDbf will not
be able to handles fields with 999999 (6 digits).

So I oversize the field type in order to accept anything coming from the
database.
    ftSmallint  16 bits = -32768 to 32767
    ... dbf supports:       -999 to  9999
                           4 digits max in practice
        therefore         DIGITS_SMALLINT = 4;
    ftWord 16 bits sign = 0 to     65535
    ... dbf supports:     0 to 999999999 (in an N field)
        therefore         DIGITS_WORD = 5;
    ftInteger  32 bits  = -2147483648 to 2147483647
    ... dbf supports:       -99999999 to  999999999                                        12345678901 = 11 digits max
        therefore         DIGITS_INTEGER = 9;
    ftLargeInt 64 bits  = -9223372036854775808 to 9223372036854775807
    ... dbf supports:       -99999999999999999 to  999999999999999999
        therefore         DIGITS_LARGEINT = 18;
 *)
  DIGITS_SMALLINT = 4;
  DIGITS_WORD = 5;
  DIGITS_INTEGER = 9;
  DIGITS_LARGEINT = 18;
  DIGITS_LONGWORD = 9;

//====================================================================
// DbfFieldDefs
//====================================================================
function TDbfFieldDefs.GetItem(Idx: Integer): TDbfFieldDef;
begin
  Result := TDbfFieldDef(inherited GetItem(Idx));
end;

constructor TDbfFieldDefs.Create(Owner: TPersistent);
begin
  inherited Create(TDbfFieldDef);
  FOwner := Owner;
end;

function TDbfFieldDefs.AddFieldDef: TDbfFieldDef;
begin
  Result := TDbfFieldDef(inherited Add);
end;

function TDbfFieldDefs.GetOwner: TPersistent; {override;}
begin
  Result := FOwner;
end;

procedure TDbfFieldDefs.Add(const Name: string; DataType: TFieldType; Size: Integer; Required: Boolean);
var
  FieldDef: TDbfFieldDef;
begin
  FieldDef := AddFieldDef;
  FieldDef.FieldName := Name;
  FieldDef.FieldType := DataType;
  if Size <> 0 then
    FieldDef.Size := Size;
  FieldDef.Required := Required;
end;

//====================================================================
// DbfFieldDef
//====================================================================
constructor TDbfFieldDef.Create(ACollection: TCollection); {virtual}
begin
  inherited;

  FDefaultBuf := nil;
  FMinBuf := nil;
  FMaxBuf := nil;
  FAllocSize := 0;
  FCopyFrom := -1;
  FPrecision := 0;
  FHasDefault := false;
  FHasMin := false;
  FHasMax := false;
  FNullPosition := -1;
  FVarLengthPosition := -1;
end;

destructor TDbfFieldDef.Destroy; {override}
begin
  FreeBuffers;
  inherited;
end;

procedure TDbfFieldDef.Assign(Source: TPersistent);
var
  DbfSource: TDbfFieldDef;
begin
  if Source is TDbfFieldDef then
  begin
    // copy from another TDbfFieldDef
    DbfSource := TDbfFieldDef(Source);
    FFieldName := DbfSource.FieldName;
    FFieldType := DbfSource.FieldType;
    FNativeFieldType := DbfSource.NativeFieldType;
    FSize := DbfSource.Size;
    FPrecision := DbfSource.Precision;
    FRequired := DbfSource.Required;
    FCopyFrom := DbfSource.Index;
    FIsLockField := DbfSource.IsLockField;
    FIsSystemField := DbfSource.IsSystemField;
    FNullPosition := DbfSource.NullPosition;
    FVarLengthPosition:=DbfSource.VarLengthPosition;
    // copy default,min,max
    AllocBuffers;
    if DbfSource.DefaultBuf <> nil then
      Move(DbfSource.DefaultBuf^, FDefaultBuf^, FAllocSize*3);
    FHasDefault := DbfSource.HasDefault;
    FHasMin := DbfSource.HasMin;
    FHasMax := DbfSource.HasMax;
    // do we need offsets?
    FOffset := DbfSource.Offset;
    FAutoInc := DbfSource.AutoInc;
    FAutoIncStep := DbfSource.AutoIncStep;
{$ifdef SUPPORT_FIELDDEF_TPERSISTENT}
  end else if Source is TFieldDef then begin
    AssignDb(TFieldDef(Source));
{$endif}
  end else
    inherited Assign(Source);
end;

procedure TDbfFieldDef.AssignDb(DbSource: TFieldDef);
begin
  // copy from Db.TFieldDef
  FFieldName := DbSource.Name;
  FFieldType := DbSource.DataType;
  // We do NOT copy over size if TFieldDef size is different from our native size
  if not(DBSource.DataType in [ftBCD,ftCurrency]) then
    FSize := DbSource.Size;
  FPrecision := DbSource.Precision;
  FRequired := DbSource.Required;
{$ifdef SUPPORT_FIELDDEF_INDEX}
  FCopyFrom := DbSource.Index;
{$endif}
  FIsLockField := false;
  FIsSystemField := false;
  // convert VCL fieldtypes to native DBF fieldtypes
  VCLToNative;
  // for integer / float fields try to fill in Size/precision
  if FSize = 0 then
    SetDefaultSize
  else
    CheckSizePrecision;
  // VCL does not have default value support
  AllocBuffers;
  FHasDefault := false;
  FHasMin := false;
  FHasMax := false;
  FOffset := 0;
  FAutoInc := 0;
  FAutoIncStep := 0;
end;

procedure TDbfFieldDef.AssignTo(Dest: TPersistent);
var
  DbDest: TFieldDef;
begin
{$ifdef SUPPORT_FIELDDEF_TPERSISTENT}
  // copy to VCL fielddef?
  if Dest is TFieldDef then
  begin
    DbDest := TFieldDef(Dest);
    // VCL TFieldDef does not know how to handle TDbfFieldDef!
    // what a shame :-)
{$ifdef SUPPORT_FIELDDEF_ATTRIBUTES}
    DbDest.Attributes := [];
    DbDest.ChildDefs.Clear;
    DbDest.DataType := FFieldType;
    DbDest.Required := FRequired;
    DbDest.Size := FSize;
    DbDest.Name := FFieldName;
{$endif}
  end else
{$endif}
    inherited AssignTo(Dest);
end;

function TDbfFieldDef.GetDbfVersion: TXBaseVersion;
begin
  Result := TDbfFieldDefs(Collection).DbfVersion;
end;

procedure TDbfFieldDef.SetFieldType(lFieldType: TFieldType);
begin
  FFieldType := lFieldType;
  VCLToNative;
  SetDefaultSize;
end;

procedure TDbfFieldDef.SetNativeFieldType(lFieldType: TDbfFieldType);
begin
  // convert lowercase to uppercase
  if (lFieldType >= 'a') and (lFieldType <= 'z') then
    lFieldType := Chr(Ord(lFieldType)-32);
  FNativeFieldType := lFieldType;
  NativeToVCL;
  CheckSizePrecision;
end;

procedure TDbfFieldDef.SetSize(lSize: Integer);
begin
  FSize := lSize;
  CheckSizePrecision;
end;

procedure TDbfFieldDef.SetPrecision(lPrecision: Integer);
begin
  FPrecision := lPrecision;
  CheckSizePrecision;
end;

procedure TDbfFieldDef.NativeToVCL;
begin
  case FNativeFieldType of
    '+' : //dbase7+ autoinc
      if DbfVersion = xBaseVII then
        FFieldType := ftAutoInc;
    'I' : //visual foxpro integer
      // todo: is this the right property to check for? Can't we check flags directly
      if FAutoIncStep=0 then
        FFieldType := ftInteger
      else
        FFieldType := ftAutoInc;
    'O' : //double, 8 bytes?
      FFieldType := ftFloat;
    '@', 'T' {Foxpro? datetime}:
      FFieldType := ftDateTime;
    'C', //character
    #$91  {Russian 'C'}:
      FFieldType := ftString;
    'L' : //logical
      FFieldType := ftBoolean;
    'F', 'N': //float/numeric
      begin
        if (FPrecision = 0) then
        begin
          if FSize <= DIGITS_SMALLINT then
            FFieldType := ftSmallInt
          else
          if FSize <= DIGITS_INTEGER then
            FFieldType := ftInteger
          else
{$ifdef SUPPORT_INT64}
            FFieldType := ftLargeInt;
{$else}
            FFieldType := ftFloat;
{$endif}
        end else begin
          FFieldType := ftFloat;
        end;
      end;
    'D' : //date
      FFieldType := ftDate;
    'M' : //memo
      FFieldType := ftMemo;
    'B' : //binary or float
      if (DbfVersion = xFoxPro) or (DbfVersion=xVisualFoxPro) then
        FFieldType := ftFloat
      else
        FFieldType := ftBlob;
    'G' : //general
      FFieldType := ftDBaseOle;
    'Y' : //currency
      if DbfGlobals.CurrencyAsBCD then
        FFieldType := ftBCD
      else
        FFieldType := ftCurrency;
    '0' : //zero, not the letter O
      FFieldType := ftBytes;
    'P' : //picture
      if (DBFversion in [xFoxPro,xVisualFoxPro]) then
        FFieldType := ftBlob; {Picture, but probably not compatible with ftGraphic storage}
    'V' : //VFP 9 Varchar; character with length indication
      if (DbfVersion = xVisualFoxPro) then
        FFieldType := ftString;
      //todo: verify if 'V' for other systems exists. DBF "Varifields"?
    'W' : //BLOB
      if (DBFVersion = xVisualFoxPro) then
        FFieldType := ftBlob;
    'Q' : //varbinary
      if (DBFVersion = xVisualFoxPro) then
        FFieldType := ftVarBytes;
  else
    FNativeFieldType := #0;
    FFieldType := ftUnknown;
  end; //case
end;

procedure TDbfFieldDef.VCLToNative;
begin
  FNativeFieldType := #0;
  // to do: look into ftBytes support; e.g. Visual FoxPro varbytes?
  case FFieldType of
    ftAutoInc  :
      if DbfVersion=xVisualFoxPro then
        FNativeFieldType  := 'I'
      else
        FNativeFieldType  := '+'; //Apparently xbaseV/7+ only; not (Visual) Foxpro
    ftDateTime :
      if DbfVersion = xBaseVII then
        FNativeFieldType := '@'
      else
      if (DbfVersion = xFoxPro) or (DbfVersion = xVisualFoxPro) then
        FNativeFieldType := 'T'
      else
        FNativeFieldType := 'D';
{$ifdef SUPPORT_FIELDTYPES_V4}
    ftFixedChar,
    ftWideString,
{$endif}
    ftString   :
      FNativeFieldType := 'C'; // VFP9: could have used V but this works, too.
    ftBoolean  :
      FNativeFieldType := 'L'; //logical
    ftFloat, ftSmallInt, ftWord
{$ifdef SUPPORT_INT64}
      , ftLargeInt
{$endif}
{$ifdef SUPPORT_LONGWORD}
      , ftLongWord, ftShortInt, ftByte
{$endif}
               :
      FNativeFieldType := 'N'; //numerical
    ftDate     :
      FNativeFieldType := 'D'; //date
    ftMemo     :
      FNativeFieldType := 'M'; //memo
    ftBlob     :
      case DBFVersion of
        xFoxPro:
          FNativeFieldType := 'P'; //picture; best we can do
        xVisualFoxPro:
          FNativeFieldType := 'W'; //blob
        xBaseIII,xBaseIV:
          FNativeFieldType := 'M'; //memo; best we can do
        xBaseV,xBaseVII:
          FNativeFieldType := 'B'; //binary
      else
        FNativeFieldType := 'M'; //fallback
      end;
    ftVarBytes :
      //todo: figure out if we can use the same fallbacks as ftBlob
      case DBFVersion of
        xVisualFoxPro:
          FNativeFieldType := 'Q'; //variant bytes
      end;
    ftDBaseOle :
      FNativeFieldType := 'G'; //general
      //todo: verify if this is dbaseV/7 specific
    ftGraphic  :
      // Let's store this as a BLOB even though FoxPro has P(icture).
      // P is apparently not recommended
      FNativeFieldType := 'B'; //BLOB
    ftInteger  :
      if (DbfVersion in [xBaseVII,xVisualFoxPro]) then
        FNativeFieldType := 'I' //integer
      else
        FNativeFieldType := 'N'; //numeric
    ftBCD, ftCurrency:
      if (DbfVersion = xFoxPro) or (DBFVersion = xVisualFoxPro) then
        FNativeFieldType := 'Y';
  end;
  if FNativeFieldType = #0 then
    raise EDbfError.CreateFmt(STRING_INVALID_VCL_FIELD_TYPE, [GetDisplayName, Ord(FFieldType)]);
end;

procedure TDbfFieldDef.SetDefaultSize;
begin
  // choose default values for variable size fields
  case FFieldType of
    ftFloat:
      begin
        FSize := 18;
        FPrecision := 8;
      end;
    ftCurrency, ftBCD:
      begin
        FSize := 8; // Stored in dbase as 8 bytes; up to 18 (or 20) characters including .-
        // FPC ftBCD/ftCurrency TFieldDef.Size has max 4 which is 4 bytes after decimal
        FPrecision := 4; //Total number of digits
      end;
    ftSmallInt:
      begin
        FSize := DIGITS_SMALLINT;
        FPrecision := 0;
      end;
    ftWord:
      begin
        FSize := DIGITS_WORD;
        FPrecision := 0;
      end;
    ftInteger, ftAutoInc:
      begin
        if DbfVersion in [xBaseVII,xVisualFoxPro] then
          FSize := 4 //I, @ field
        else
          FSize := DIGITS_INTEGER;
        FPrecision := 0;
      end;
{$ifdef SUPPORT_INT64}
    ftLargeInt:
      begin
        FSize := DIGITS_LARGEINT;
        FPrecision := 0;
      end;
{$endif}
{$ifdef SUPPORT_LONGWORD}
    ftLongWord:
      begin
        FSize := DIGITS_LONGWORD;
        FPrecision := 0;
      end;
    ftShortInt,
    ftByte:
      begin
        FSize := 3;
        FPrecision := 0;
      end;
{$endif}
    ftString {$ifdef SUPPORT_FIELDTYPES_V4}, ftFixedChar, ftWideString{$endif}:
      begin
        FSize := 30;
        FPrecision := 0;
      end;
  end; // case fieldtype

  // set sizes for fields that are restricted to single Size/precision
  CheckSizePrecision;
end;

procedure TDbfFieldDef.CheckSizePrecision;
// FSize means size in the database, not any VCL field size
begin
  case FNativeFieldType of
    'C','V','Q': // Character, Visual FoxPro varchar,Visual FoxPro varbinary
      begin
        if FSize < 0 then
          FSize := 0;
        if (DbfVersion = xFoxPro) or (DbfVersion = xVisualFoxPro) then
        begin
          if FSize >= $FFFF then
            FSize := $FFFF;
        end else begin
          if FSize >= $FF then
            FSize := $FF;
        end;
        FPrecision := 0;
      end;
    'L': // Logical/boolean
      begin
        FSize := 1;
        FPrecision := 0;
      end;
    'N','F': // Binary code decimal numeric, floating point binary numeric
      begin
        // ftBCD: precision=total number of digits; Delphi supports max 32
        // Note: this field can be stored as BCD or integer, depending on FPrecision;
        // that's why we allow 0 precision
        if FSize < 1   then FSize := 1;
        // Removed, bug report 39009
        // if FSize >= 20 then FSize := 20;
        if FPrecision > FSize-2 then FPrecision := FSize-2; //Leave space for . and -
        if FPrecision < 0       then FPrecision := 0;
      end;
    'D': // Date
      begin
        FSize := 8;
        FPrecision := 0;
      end;
    'B': // (Visual)Foxpro double, DBase binary
      begin
        if not(DbfVersion in [xFoxPro,xVisualFoxPro]) then
        begin
          FSize := 10;
          FPrecision := 0;
        end
        else
        begin
          FSize := 8; //Foxpro double
          FPrecision := 0;
        end;
      end;
    'M','G','P','W': // Memo, general, FoxPro picture, Visual FoxPro blob
      begin
        if (DbfVersion = xVisualFoxPro) then
        begin
          if (FSize <> 4) and (FSize <> 10) then
            FSize := 4;
        end else
          FSize := 10; //Dbase, includes FoxPro
        FPrecision := 0;
      end;
    '+','I': // Autoincrement, integer
      begin
        FSize := 4;
        FPrecision := 0;
      end;
    '@', 'O': //Timestamp, double (both DBase 7)
      begin
        FSize := 8;
        FPrecision := 0;
      end;
    'T': // DateTime
      begin
        if (DbfVersion = xFoxPro) or (DbfVersion = xVisualFoxPro) then
          FSize := 8
        else
          FSize := 14;
        FPrecision := 0;
      end;
    'Y': // Currency
      begin
        FSize := 8;
        FPrecision := 4;
      end;
  else
    // no idea/unimportant, let other code sort it out
  end;
end;

function TDbfFieldDef.GetDisplayName: string; {override;}
begin
  Result := FieldName;
end;

function TDbfFieldDef.IsBlob: Boolean; {override;}
begin
  // 'B' is float in (V)FP; W is Blob (VFP9)
  if (DbfVersion in [xFoxPro,xVisualFoxPro]) then
    Result := FNativeFieldType in ['M','G','W']
  else
    Result := FNativeFieldType in ['M','G','B'];
end;

procedure TDbfFieldDef.FreeBuffers;
begin
  if FDefaultBuf <> nil then
  begin
    // one buffer for all
    FreeMemAndNil(Pointer(FDefaultBuf));
    FMinBuf := nil;
    FMaxBuf := nil;
  end;
  FAllocSize := 0;
end;

procedure TDbfFieldDef.AllocBuffers;
begin
  // Size changed?
  if FAllocSize <> FSize then
  begin
    // free old buffers
    FreeBuffers;
    // alloc new
    GetMem(FDefaultBuf, FSize*3);
    FMinBuf := FDefaultBuf + FSize;
    FMaxBuf := FMinBuf + FSize;
    // store allocated Size
    FAllocSize := FSize;
  end;
end;

end.

