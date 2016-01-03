{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by Michael Van Canneyt, member of the
    Free Pascal development team

    Firebird/Interbase Data Dictionary Engine Implementation.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpddfb;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, sqldb, fpdatadict, fpddsqldb, db;
  
Type

  { TSQLDBFBDDEngine }

  { TFPDDFBSQLEngine }

  TFPDDFBSQLEngine = Class(TFPDDSQLEngine)
  Public
    Function  CreateSequenceSQL(Sequence : TDDSequenceDef) : String; override;
  end;

  TSQLDBFBDDEngine = Class(TSQLDBDDEngine)
  private
    function ConvertFBFieldType(FDfieldtype, FBsubtype: integer): TFieldType;
  Protected
    Function CreateConnection(AConnectString  : String) : TSQLConnection; override;
  Public
    function ImportFields(Table: TDDTableDef): Integer; override;
    Function ImportIndexes(Table : TDDTableDef) : Integer; override;
    Function ImportSequences(Sequences : TDDSequenceDefs; List : TStrings; UpdateExisting : boolean) : Integer; override;
    Function ImportDomains(Domains : TDDDomainDefs; List : TStrings; UpdateExisting : boolean) : Integer; override;
    Function CreateSQLEngine : TFPDDSQLEngine; override;
    Class function EngineCapabilities : TFPDDEngineCapabilities; override;
    Class function Description : string; override;
    Class function DBType : String; override;
  end;

  // Backwards compatibility
  TSQLDBIBDDEngine = TSQLDBFBDDEngine;
  

Procedure RegisterFBDDEngine;
Procedure UnRegisterFBDDEngine;

implementation

uses ibconnection;

Procedure RegisterFBDDEngine;

begin
  RegisterDictionaryEngine(TSQLDBFBDDEngine);
end;

Procedure UnRegisterFBDDEngine;

begin
  UnRegisterDictionaryEngine(TSQLDBFBDDEngine);
end;

{ TSQLDBFBDDEngine }

function TSQLDBFBDDEngine.CreateConnection(AConnectString: String
  ): TSQLConnection;
begin
  Result:=TIBConnection.Create(Self);
end;

class function TSQLDBFBDDEngine.EngineCapabilities: TFPDDEngineCapabilities;
begin
  Result:=[ecImport,ecCreateTable,ecViewTable, ecTableIndexes,
           ecRunQuery, ecRowsAffected, ecSequences, ecDomains];
end;

class function TSQLDBFBDDEngine.Description: string;
begin
  Result:='Firebird/Interbase connection using SQLDB';
end;

class function TSQLDBFBDDEngine.DBType: String;
begin
  Result:='Firebird/Interbase';
end;

function TSQLDBFBDDEngine.ConvertFBFieldType (FDfieldtype, FBsubtype : integer) : TFieldType;
var b : byte;
begin
  if FDFieldType > 255 then
    begin
    if FDFieldType = 261 then
      result := ftBlob       {BLOB}
    else
      result := ftUnknown;
    end
  else
    begin
    b := byte(FDFieldType and $FF);
    if (b in [7,8,16]) and (FBsubtype <> 0) then
      // BCD types: 1= Numeric, 2 := Decimal
      result := ftBCD
    else
      case b of
        14 : result := ftFixedChar; {CHAR}
        37 : result := ftString;    {VARCHAR}
        40 : result := ftString;    {CSTRING ?}
        11 : result := ftFloat;     {D-FLOAT ?}
        27 : result := ftFloat;     {DOUBLE}
        10 : result := ftFloat;     {FLOAT}
        16 : result := ftLargeint;  {INT64}
        8  : result := ftInteger;   {INTEGER}
        9  : result := ftlargeint;  {QUAD ?}
        7  : result := ftSmallint;  {SMALLINT}
        12 : result := ftDate;      {DATE dialect 3}
        13 : result := ftTime;      {TIME}
        35 : result := ftDateTime;  {TIMESTAMP dialect 3, DATE in dialect 1,2}
        else result := ftUnknown;
      end;
    end;
end;

function TSQLDBFBDDEngine.ImportIndexes(Table: TDDTableDef): Integer;
const
  SQLindexes = 'SELECT '+
        'I.RDB$INDEX_NAME as IndexName, '+
        'I.RDB$INDEX_TYPE as IndexType, '+
        'I.RDB$UNIQUE_FLAG as IsUnique, '+
        'R.RDB$CONSTRAINT_TYPE as ConstraintType, '+
        'R.RDB$CONSTRAINT_NAME as ConstraintName '+
        'FROM '+
        'RDB$INDICES I '+
        'LEFT JOIN RDB$RELATION_CONSTRAINTS R ON I.RDB$INDEX_NAME = R.RDB$INDEX_NAME '+
        'WHERE '+
        'I.RDB$RELATION_NAME=''%s'' '+
        'AND I.RDB$FOREIGN_KEY is null '+
        'ORDER BY I.RDB$INDEX_NAME';

  {
  SQLchecks = 'SELECT '+
        'R.RDB$CONSTRAINT_NAME as ConstraintName, '+
        'R.RDB$CONSTRAINT_TYPE as ConstraintType, '+
        'T.RDB$TRIGGER_SOURCE as CheckSource, '+
        'FROM '+
        'RDB$RELATION_CONSTRAINTS R '+
        'LEFT JOIN RDB$CHECK_CONSTRAINTS C ON R.RDB$CONSTRAINT_NAME = C.RDB$CONSTRAINT_NAME '+
        'LEFT JOIN RDB$TRIGGERS T ON T.RDB$TRIGGER_NAME = C.RDB$TRIGGER_NAME '+
        'WHERE '+
        'R.RDB$RELATION_NAME=''%s'' '+
        'ORDER BY R.RDB$CONSTRAINT_NAME';

  SQLforeign = 'SELECT '+
        'R.RDB$CONSTRAINT_NAME as ConstraintName, '+
        'R.RDB$INDEX_NAME as IndexName, '+
        'E.RDB$CONST_NAME_UQ as RefUnique, '+
        'E.RDB$UPDATE_RULE as OnUpdate, '+
        'E.RDB$DELETE_RULE as OnDelete, '+
        'I.RDB$INDEX_TYPE as IndexType '+
        'FROM '+
        'RDB$RELATION_CONSTRAINTS R '+
        'LEFT JOIN RDB$REF_CONSTRAINTS E ON E.RDB$CONSTRAINT_NAME = R.RDB$CONSTRAINT_NAME '+
        'LEFT JOIN RDB$INDICES I ON I.RDB$INDEX_NAME = R.RDB$INDEX_NAME '+
        'WHERE '+
        'R.RDB$RELATION_NAME=''%s'' '+
        'ORDER BY R.RDB$CONSTRAINT_NAME';
  }
  SQLFields = 'SELECT RDB$FIELD_NAME as IndexField '+
              'FROM RDB$INDEX_SEGMENTS '+
              'WHERE RDB$INDEX_NAME = :IndexName '+
              'ORDER BY RDB$FIELD_POSITION';
        
Var
  Q, QF : TSQLQuery;
  PIndexName : TParam;
  FConstraintName, FConstraintType,
  FIndexType, FIndexName, FUnique : TField;
  //FCheckSource, FRefUnique,
  //FOnUpdate, FOnDelete : TField;
  Index : TDDIndexDef;

  procedure BindIndexFields;
  begin
    PIndexName := QF.params.parambyname ('IndexName');
    FConstraintName := Q.Fieldbyname('ConstraintName');
    FConstraintType := Q.Fieldbyname('ConstraintType');
    FIndexType := Q.Fieldbyname('IndexType');
    FIndexName := Q.Fieldbyname('IndexName');
    FUnique := Q.Fieldbyname('IsUnique');
  end;
  {
  procedure BindCheckFields;
  begin
    FCheckSource := Q.Fieldbyname('CheckSource');
  end;
  
  procedure BindForeignFields;
  begin
    FRefUnique := Q.Fieldbyname('RefUnique');
    FOnUpdate := Q.Fieldbyname('OnUpdate');
    FOnDelete := Q.Fieldbyname('OnDelete');
  end;
  }
  function CreateIndex (AName, indexname: string) : TDDIndexDef;
  var n, s : string;
  begin
    n := trim(AName);
    if n = '' then
      n := trim(indexname);
    if trim (indexName) = '' then
      indexname := AName;
    result := Table.Indexes.AddIndex(n);
    PIndexName.asstring := indexname;
    QF.Open;
    try
      s := trim(QF.Fields[0].asstring);
      QF.Next;
      while not QF.eof do
        begin
        s := s + ';' + trim(QF.Fields[0].asstring);
        QF.Next;
        end;
    finally
      QF.Close;
    end;
    result.Fields := s;
  end;
  
  function ImportIndices : integer;
  begin
    result := 0;
    Q.SQL.text := format (SQLindexes, [Table.TableName]);
    Q.Open;
    try
      result := 0;
      Q.First;
      BindIndexFields;
      while not Q.eof do
        begin
        with CreateIndex (FConstraintName.asstring, FIndexName.asstring) do
          begin
          inc (result);
          if trim(FConstraintType.asstring) = 'PRIMARY KEY' then
            options := options + [ixPrimary]
          else if FUnique.asinteger = 1 then
            options := options + [ixUnique];
          if FIndextype.asinteger = 1 then
            options := options + [ixDescending];
          end;
        Q.Next;
        end;
    finally
      Q.Close;
    end;
  end;

begin
  Q:=CreateSQLQuery(Nil);
  try
    QF:=CreateSQLQuery(Nil);
    try
      QF.SQl.Text := SQLFields;
      QF.Prepare;
      try
        ImportIndices;
        //ImportChecks;
        //ImportForeignKeys;
      finally
        QF.Unprepare;
      end;
    finally
      QF.Free;
    end;
  finally
    Q.Free;
  end;
end;

function TSQLDBFBDDEngine.ImportSequences(Sequences: TDDSequenceDefs;
  List: TStrings; UpdateExisting: boolean): Integer;

const
  SQL = 'SELECT RDB$GENERATOR_Name FROM RDB$Generators WHERE RDB$System_Flag = 0';
  
Var
  Q : TSQLQuery;
  Seq : TDDSequenceDef;
  n : string;

begin
  result := 0;
  Q:=CreateSQLQuery(Nil);
  try
    Q.Sql.Text := SQL;
    Q.Open;
    try
      while not Q.eof do
        begin
        n := trim(Q.Fields[0].asstring);
        seq := Sequences.FindSequence(n);
        if not assigned (Seq) then
          Seq := Sequences.AddSequence(n)
        else if not UpdateExisting then
          Seq := nil;
        if assigned (Seq) then
          begin
          Seq.Increment := 0;
          Seq.StartValue := 0;
          inc (result);
          end;
        Q.Next;
        end;
    finally
      Q.CLose;
    end;
  finally
    Q.Free;
  end;
end;

function TSQLDBFBDDEngine.ImportDomains(Domains: TDDDomainDefs; List: TStrings;
  UpdateExisting: boolean): Integer;

const
  SQL = 'SELECT ' +
        ' RDB$FIELD_NAME as Name,' +
        ' RDB$DEFAULT_SOURCE as DomainDefault,' +
        ' RDB$FIELD_LENGTH as CharLength,' +
        ' RDB$FIELD_PRECISION as FieldPrecision,' +
        ' RDB$FIELD_SCALE as Scale,' +
        ' RDB$FIELD_TYPE as FieldType,' +
        ' RDB$FIELD_SUB_TYPE as Subtype,' +
        ' RDB$NULL_FLAG as DomainNull' +
        ' FROM '+
        ' RDB$FIELDS'+
        ' WHERE RDB$System_Flag = 0 and not (RDB$Field_Name like ''RDB$%'')';

Var
  Q : TSQLQuery;
  FName, FDomainName, FDomainDefault,
  FCharLength, FPrecision, FScale, FFieldType, FSubType, FDomainnull : TField;

  procedure BindFields;
  begin
    FName := q.fieldbyname('Name');
    FDomainDefault := q.fieldbyname('DomainDefault');
    FCharLength := q.fieldbyname('CharLength');
    FPrecision := q.fieldbyname('FieldPrecision');
    FScale := q.fieldbyname('Scale');
    FFieldType := q.fieldbyname('FieldType');
    FSubType := q.fieldbyname('SubType');
    FDomainnull := q.fieldbyname('Domainnull');
  end;

  function ImportDomain : boolean;
  var Dom : TDDDomainDef;
      n : string;
  begin
    n := trim(FName.asstring);
    Dom := Domains.FindDomain(n);
    if not assigned (Dom) then
      Dom := Domains.AddDomain(n)
    else if not UpdateExisting then
      Dom := nil;
    if assigned (Dom) then
      begin
      result := true;
      Dom.FieldType := ConvertFBFieldType (FFieldType.asinteger, FSubType.asinteger);
      Dom.Precision := FPrecision.asinteger;
      if FScale.asinteger < 0 then
        Dom.Size := -FScale.asinteger
      else if Dom.Fieldtype in [ftString, ftFixedChar] then
        Dom.Size := FCharLength.asinteger
      else
        Dom.Size := 0;
      //Dom.DefaultExpression := copy(trim(FDomainDefault.asstring), 9, maxint);
      Dom.Required := FDomainnull.asinteger = 1;
      end
    else
      result := false;
  end;
  
begin
  result := 0;
  Q:=CreateSQLQuery(Nil);
  try
    Q.Sql.Text := SQL;
    Q.Open;
    BindFields;
    try
      while not Q.eof do
        begin
        if ImportDomain then
          inc (result);
        Q.Next;
        end;
    finally
      Q.CLose;
    end;
  finally
    Q.Free;
  end;
end;

function TSQLDBFBDDEngine.ImportFields(Table: TDDTableDef): Integer;
Const
  SQL = 'SELECT ' +
        ' F.RDB$FIELD_POSITION as FieldPosition,' +
        ' F.RDB$FIELD_NAME as Name,' +
        ' F.RDB$NULL_FLAG as FieldNull,' +
        ' F.RDB$Description as Description,' +
        ' F.RDB$DEFAULT_SOURCE as FieldDefault,' +
        ' D.RDB$DEFAULT_SOURCE as DomainDefault,' +
        ' D.RDB$FIELD_LENGTH as CharLength,' +
        ' D.RDB$FIELD_PRECISION as FieldPrecision,' +
        ' D.RDB$FIELD_SCALE as Scale,' +
        ' D.RDB$FIELD_TYPE as FieldType,' +
        ' D.RDB$FIELD_SUB_TYPE as Subtype,' +
        ' D.RDB$NULL_FLAG as DomainNull,' +
        ' D.RDB$FIELD_NAME as DName ' +
        ' FROM '+
        ' RDB$RELATION_FIELDS F left join RDB$FIELDS D on F.RDB$FIELD_Source = D.RDB$FIELD_NAME'+
        ' WHERE (RDB$RELATION_NAME = ''%s'')' +
        ' ORDER BY RDB$FIELD_POSITION';

Var
  Q : TSQLQuery;
  FName, FPosition, FFieldnull, FDescription, FFieldDefault,
  FDomainDefault, FDomainnull, FDomainName,
  FCharLength, FPrecision, FScale, FFieldType, FSubType : TField;

  procedure BindFields;
  begin
    FName := q.fieldbyname('Name');
    FPosition := q.fieldbyname('FieldPosition');
    FFieldnull := q.fieldbyname('FieldNull');
    FDescription := q.fieldbyname('Description');
    FFieldDefault := q.fieldbyname('FieldDefault');
    FDomainDefault := q.fieldbyname('DomainDefault');
    FCharLength := q.fieldbyname('CharLength');
    FPrecision := q.fieldbyname('FieldPrecision');
    FScale := q.fieldbyname('Scale');
    FFieldType := q.fieldbyname('FieldType');
    FSubType := q.fieldbyname('SubType');
    FDomainnull := q.fieldbyname('Domainnull');
    FDomainName := q.fieldbyname('DName');
  end;

  {Opmerking: bestaande fielddefs die niet meer in de tabel zitten worden niet verwijderd !? }

<<<<<<< HEAD
<<<<<<< HEAD
  function ImportFieldDef(APosition : Integer) : boolean;
=======
  function ImportFieldDef : boolean;
>>>>>>> graemeg/fixes_2_2
=======
  function ImportFieldDef : boolean;
>>>>>>> origin/fixes_2_2
  var FD : TDDFieldDef;
      n, s : string;
  begin
    n := trim(FName.asstring);
    FD := Table.Fields.FindField(n);
    if not assigned (FD) then
      FD := Table.AddField(n);
    FD.FieldName := n;
    FD.FieldType := ConvertFBFieldType (FFieldType.asinteger, FSubType.asinteger);
    FD.Precision := FPrecision.asinteger;
    if FScale.asinteger < 0 then
      FD.Size := -FScale.asinteger
    else if FD.Fieldtype in [ftString, ftFixedChar] then
      FD.Size := FCharLength.asinteger
    else
      FD.Size := 0;
      { // Fixed length types don't have a size in the dictionary
      case byte(FFieldType.asinteger and $FF) of
        7 : FD.Size := 2;
        10,8 : FD.Size := 4;
        35,11,27,9,16,12 : FD.Size := 8;
      end; }
    if not fDescription.IsNull then
      FD.Hint := FDescription.asstring;
    s := trim(FFieldDefault.asstring);
    n := trim(FDomainDefault.asstring);
    if s <> '' then
      FD.DefaultExpression := copy(s, 9, maxint)
    else if n <> '' then
      FD.DefaultExpression := copy(n, 9, maxint);
    if FDomainnull.asinteger = 0 then
      if FFieldnull.asinteger = 1 then
        FD.Required:=true
      else
        FD.Required:=false
    else
      FD.Required:=false;
<<<<<<< HEAD
<<<<<<< HEAD
    FD.index := APosition;
=======
    FD.index := FPosition.AsInteger;
>>>>>>> graemeg/fixes_2_2
=======
    FD.index := FPosition.AsInteger;
>>>>>>> origin/fixes_2_2
    s := trim(FDomainName.asstring);
    if copy(s, 1, 4) <> 'RDB$' then
      FD.DomainName := s
    else
      FD.DomainName := '';
    result := true;
  end;

  function ImportFromSQLDef : integer;
  begin
    result := 0;
    Q.First;
    BindFields;
    while not Q.eof do
      begin
<<<<<<< HEAD
<<<<<<< HEAD
      if ImportFieldDef(Result) then
=======
      if ImportFieldDef then
>>>>>>> graemeg/fixes_2_2
=======
      if ImportFieldDef then
>>>>>>> origin/fixes_2_2
        inc (result);
      Q.Next;
      end;
  end;

begin
  Q:=CreateSQLQuery(Nil);
  try
    Q.Sql.Text:=Format(SQL,[Table.TableName]);
    Q.Open;
    try
      result := ImportFromSQLDef;
    finally
      Q.CLose;
    end;
  finally
    Q.Free;
  end;
end;

function TSQLDBFBDDEngine.CreateSQLEngine: TFPDDSQLEngine;
begin
  Result:=TFPDDFBSQLEngine.Create;
end;

{ TFPDDFBSQLEngine }

function TFPDDFBSQLEngine.CreateSequenceSQL(Sequence: TDDSequenceDef): String;
begin
  Result:='CREATE GENERATOR '+Sequence.SequenceName;
end;

end.

