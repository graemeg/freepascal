unit fpdbfexport;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, db, dbf, fpdbexport;
  
Type

  { TDBFExportFieldItem }

  TDBFExportFieldItem = Class(TExportFieldItem)
  private
    FDestField: TField;
  Protected
    Property DestField : TField Read FDestField;
  end;

  { TDBFExportFormatSettings }

<<<<<<< HEAD
<<<<<<< HEAD
  TTableFormat = (tfDBaseIII,tfDBaseIV,tfDBaseVII,tfFoxPro,tfVisualFoxPro);
=======
  TTableFormat = (tfDBaseIII,tfDBaseIV,tfDBaseVII,tfFoxPro);
>>>>>>> graemeg/fixes_2_2
=======
  TTableFormat = (tfDBaseIII,tfDBaseIV,tfDBaseVII,tfFoxPro);
>>>>>>> origin/fixes_2_2
  
  TDBFExportFormatSettings = class(TExportFormatSettings)
  private
    FAutoRename: Boolean;
    FTableFormat: TTableFormat;
  public
    Procedure Assign(Source : TPersistent); override;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure InitSettings; override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Published
    Property TableFormat : TTableFormat Read FTableFormat Write FTableFormat;
    Property AutoRenameFields : Boolean Read FAutoRename Write FAutoRename;
  end;
  { TFPCustomDBFExport }

  TFPCustomDBFExport = Class(TCustomDatasetExporter)
  Private
    FDBF : TDBF;
    FFileName: String;
    FAppendData: Boolean;
    function GetSettings: TDBFExportFormatSettings;
    procedure SetSettings(const AValue: TDBFExportFormatSettings);
  Protected
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure CheckExportFieldName(ThisExportField: TExportFieldItem; const MaxFieldNameLength: integer);
=======
    Procedure CheckExportFieldNames; virtual;
>>>>>>> graemeg/fixes_2_2
=======
    Procedure CheckExportFieldNames; virtual;
>>>>>>> origin/fixes_2_2
    Function BindFields : Boolean; override;
    Function CreateFormatSettings : TCustomExportFormatSettings; override;

    Function CreateExportFields : TExportFields; override;
    Procedure DoBeforeExecute; override;
    Procedure DoAfterExecute; override;
    Procedure DoDataRowStart; override;
    Procedure DoDataRowEnd; override;
    Procedure ExportField(EF : TExportFieldItem); override;
    Property FileName : String Read FFileName Write FFileName;
    Property AppendData : Boolean Read FAppendData Write FAppendData;
    Property DBF : TDBF Read FDBF;
  public
    Property FormatSettings : TDBFExportFormatSettings Read GetSettings Write SetSettings;
  end;

  TFPDBFExport = Class(TFPCustomDBFExport)
  Published
    Property FileName;
    Property Dataset;
    Property ExportFields;
    Property FromCurrent;
    Property RestorePosition;
    Property FormatSettings;
    Property OnExportRow;
  end;
  
Procedure RegisterDBFExportFormat;
Procedure UnRegisterDBFExportFormat;

Const
  SDBFExport = 'DBF';
  SDBFFilter = '*.dbf';
  
ResourceString
  SErrFailedToDeleteFile = 'Failed to delete existing DBF file: %s';
  SDBFDescription = 'DBF files';

implementation


{ TFPCustomDBFExport }

function TFPCustomDBFExport.GetSettings: TDBFExportFormatSettings;
begin
  Result:=TDBFExportFormatSettings(Inherited FormatSettings);
end;

procedure TFPCustomDBFExport.SetSettings(const AValue: TDBFExportFormatSettings
  );
begin
  Inherited FormatSettings.Assign(AValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFPCustomDBFExport.CheckExportFieldName(ThisExportField: TExportFieldItem; const MaxFieldNameLength: integer);
// Cut off field name at max length, and rename if it already exists
Const
  CounterInvalid=100;

Var
  NameCounter : Integer;
  NewFieldName : String;
  
begin
  If (Length(ThisExportField.ExportedName)>MaxFieldNameLength) then
    begin
    NewFieldName:=Copy(ThisExportField.ExportedName,1,MaxFieldNameLength);
    If ExportFields.IndexOfExportedName(NewFieldName)<>-1 then
      begin
      // Try using 2-character number sequence to generate unique name
      NameCounter:=1;
      Repeat
        NewFieldName:=Copy(ThisExportField.ExportedName,1,MaxFieldNameLength-2)+Format('%.2d',[NameCounter]);
      Until (ExportFields.IndexOfExportedName(NewFieldName)=-1) or (NameCounter=CounterInvalid);
      if NameCounter=CounterInvalid then
        ExportError('Could not create a unique export field name for field %s',[ThisExportField.FieldName]);
      end;
    ThisExportField.ExportedName:=NewFieldName;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TFPCustomDBFExport.CheckExportFieldNames;

Var
  I,J : Integer;
  EF : TExportFieldItem;
  FN : String;
  
begin
  For I:=0 to ExportFields.Count-1 do
    begin
    EF:=ExportFields[i];
    If (Length(EF.ExportedName)>10) then
      begin
      FN:=Copy(EF.ExportedName,1,10);
      If ExportFIelds.IndexOfExportedName(FN)<>-1 then
        begin
        J:=1;
        Repeat
          FN:=Copy(EF.ExportedName,1,8)+Format('%.2d',[J]);
        Until (ExportFIelds.IndexOfExportedName(FN)=-1);
        end;
      EF.ExportedName:=FN;
      end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;

function TFPCustomDBFExport.BindFields: Boolean;

Const
<<<<<<< HEAD
<<<<<<< HEAD
  // Translate tableformat to tablelevel
  Levels : Array[TTableFormat] of integer = (3,4,7,25,30);
  
Var
  EF : TDBFExportFieldItem;
  i : Integer;
  MaxFieldName: integer;

begin
  Result:=Inherited;
  // DBase III,IV, and FoxPro have a 10 character field length limit.
  // Visual Foxpro free tables (without .dbc file) also
  // DBase VII has a 32 character field length limit.
  if (FormatSettings.TableFormat=tfDbaseVII) then
    MaxFieldName:=32
  else
    MaxFieldName:=10;
=======
=======
>>>>>>> origin/fixes_2_2
  Levels : Array[TTableFormat] of integer = (3,4,7,25);
  
Var
  EF : TDBFExportFieldItem;
  I : Integer;
  
begin
  If FormatSettings.AutoRenameFields and (FormatSettings.TableFormat=tfDbaseIII) then
    CheckExportFieldNames;
  Result:=Inherited;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  try
    with FDBF.FieldDefs do
      begin
      Clear;
<<<<<<< HEAD
<<<<<<< HEAD
      For i:=0 to ExportFields.Count-1 do
        begin
        EF:=ExportFields[i] as TDBFExportFieldItem;
        If FormatSettings.AutoRenameFields then
          CheckExportFieldName(EF,MaxFieldName);
        If EF.Enabled and Assigned(EF.Field) then
          Add(EF.ExportedName,EF.Field.DataType,EF.Field.Size);
=======
=======
>>>>>>> origin/fixes_2_2
      For I:=0 to ExportFields.Count-1 do
        begin
        EF:=ExportFIelds[i] as TDBFExportFieldItem;
        If EF.ENabled and Assigned(EF.Field) then
          Add(EF.ExportedName,EF.FIeld.DataType,EF.Field.Size);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        end;
      FDBF.TableLevel:=Levels[FormatSettings.TableFormat];
      FDBF.CreateTable;
      FDBF.Exclusive := true;
      FDBF.Open;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
    For i:=0 to ExportFields.Count-1 do
=======
    For I:=0 to ExportFields.Count-1 do
>>>>>>> graemeg/fixes_2_2
=======
    For I:=0 to ExportFields.Count-1 do
>>>>>>> origin/fixes_2_2
      begin
      EF:=ExportFIelds[i] as TDBFExportFieldItem;
      If EF.Enabled then
        EF.FDestField:=FDBF.FieldByName(EF.ExportedName);
      end;
  except
    UnBindFields;
    Raise;
  end;
end;

function TFPCustomDBFExport.CreateFormatSettings: TCustomExportFormatSettings;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TDBFExportFormatSettings.Create(True);
=======
  Result:=TDBFExportFormatSettings.Create(False);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TDBFExportFormatSettings.Create(False);
>>>>>>> origin/fixes_2_2
end;

function TFPCustomDBFExport.CreateExportFields: TExportFields;
begin
  Result:=TExportFields.Create(TDBFExportFieldItem);
end;

procedure TFPCustomDBFExport.DoBeforeExecute;

Var
  FE : Boolean;
  
begin
  Inherited;
  FDBF:=TDBF.Create(Self);
  FDBF.TableName:=FFileName;
  FE:=FileExists(FFileName);
  If FAppendData and FE then
    FDBF.Open
  else
    begin
    If FE and Not AppendData then
      begin
      If not DeleteFile(FFileName) then
        Raise EDataExporter.CreateFmt(SErrFailedToDeleteFile,[FFileName]);
      end;
    end;
end;

procedure TFPCustomDBFExport.DoAfterExecute;
begin
  try
    FreeAndNil(FDBF);
  finally
    Inherited;
  end;
end;

procedure TFPCustomDBFExport.DoDataRowStart;
begin
  FDBF.Append;
end;

procedure TFPCustomDBFExport.DoDataRowEnd;
begin
  FDBF.Post;
end;

procedure TFPCustomDBFExport.ExportField(EF: TExportFieldItem);

Var
  F : TDBFExportFieldItem;
  
begin
  F:=EF as TDBFExportFieldItem;
  With F do
<<<<<<< HEAD
<<<<<<< HEAD
    // Export depending on field datatype;
    // convert to dbf data types where necessary.
    // Fall back to string if unknown datatype
    If FIeld.IsNull then
      DestField.Clear
    else if Field.Datatype in (IntFieldTypes+[ftAutoInc,ftLargeInt]) then
      DestField.AsInteger:=Field.AsInteger
    else if Field.Datatype in [ftBCD,ftCurrency,ftFloat,ftFMTBcd] then
      DestField.AsFloat:=Field.AsFloat
    else if Field.DataType in [ftString,ftFixedChar] then
      DestField.AsString:=Field.AsString
    else if (Field.DataType in ([ftWideMemo,ftWideString,ftFixedWideChar]+BlobFieldTypes)) then
      DestField.AsWideString:=Field.AsWideString
      { Note: we test for the wide text fields before the MemoFieldTypes, in order to
      let ftWideMemo end up at the right place }
    else if Field.DataType in MemoFieldTypes then
      DestField.AsString:=Field.AsString
    else if Field.DataType=ftBoolean then
      DestField.AsBoolean:=Field.AsBoolean
    else if field.DataType in DateFieldTypes then
      DestField.AsDatetime:=Field.AsDateTime
    else
      DestField.AsString:=Field.AsString
=======
=======
>>>>>>> origin/fixes_2_2
    If FIeld.IsNull then
      DestField.Clear
    else If Field.Datatype in IntFieldTypes then
      DestField.AsInteger:=Field.AsInteger
    else if Field.dataType in [ftString,ftFixedChar] then
      DestField.AsString:=Field.AsString
    else if Field.DataType=ftBoolean then
      DestField.AsBoolean:=Field.AsBoolean
    else if (Field.DataType in ([ftWidestring,ftFixedWideChar]+BlobFieldTypes)) then
      DestField.AsWideString:=Field.AsWideString
    else if field.DataType in DateFieldTypes then
      DestField.AsDatetime:=Field.AsDateTime
    else
      DestField.AsDatetime:=Field.AsDateTime
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Procedure RegisterDBFExportFormat;
begin
  RegisterExportFormat(SDBFExport,SDBFDescription,SDBFFilter,TFPDBFExport);
end;

Procedure UnRegisterDBFExportFormat;
begin
  UnregisterExportFormat(SDBFExport);
end;

{ TDBFExportFormatSettings }

procedure TDBFExportFormatSettings.Assign(Source: TPersistent);

Var
  FS : TDBFExportFormatSettings;
  
begin
  If Source is TDBFExportFormatSettings then
    begin
    FS:=Source as TDBFExportFormatSettings;
    AutoRenameFields:=FS.AutoRenameFields;
    TableFormat:=FS.TableFormat;
    end;
  inherited Assign(Source);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TDBFExportFormatSettings.InitSettings;
begin
  inherited InitSettings;
  FAutoRename:=true; // sensible to avoid duplicate table names
  FTableFormat:=tfDBaseIV; //often used
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.

