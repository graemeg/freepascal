unit TestBasics;

{$IFDEF FPC}
  {$mode objfpc}{$H+}
{$ENDIF}

interface

uses
  fpcunit, testutils, testregistry, testdecorator,
  Classes, SysUtils, db;

type

  { TTestBasics }

  TTestBasics = class(TTestCase)
  private
    function CreateDatasetWith3Fields: TDataset;
  protected
  published
    procedure TestParseSQL;
    procedure TestInitFielddefsFromFields;
    procedure TestDoubleFieldDef;
    procedure TestFieldDefWithoutDS;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure TestGetParamList;
    procedure TestGetFieldList;
    procedure TestExtractFieldName; //move record then copy. Is copy identical? Has record position changed?
    procedure TestCheckFieldNames;
    procedure TestFindField;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

implementation

uses toolsunit;

Type HackedDataset = class(TDataset);

{ TTestBasics }

function TTestBasics.CreateDatasetWith3Fields: TDataset;
var
  F: TField;
begin
  Result := TDataSet.Create(nil);
  F := TIntegerField.Create(Result);
  F.FieldName := 'Field1';
  F.DataSet := Result;

  F := TIntegerField.Create(Result);
  F.FieldName := 'Field2';
  F.DataSet := Result;

  F := TIntegerField.Create(Result);
  F.FieldName := 'Field3';
  F.DataSet := Result;
end;

procedure TTestBasics.TestParseSQL;
var Params  : TParams;
    ReplStr : string;
    pb      : TParamBinding;
    i       : integer;
    SQLStr  : string;
begin
  Params := TParams.Create;

  AssertEquals(     'select * from table where id = $1',
    params.ParseSQL('select * from table where id = :id',true,True,True,psPostgreSQL));

  AssertEquals(     'select * from table where id = $1',
    params.ParseSQL('select * from table where id = :id',false,True,True,psPostgreSQL));

  AssertEquals(     'update test set 1=$1 2=$2 3=$3 4=$4 5=$5 6=$6 7=$7 8=$8 9=$9 where (id = $2)',
    params.ParseSQL('update test set 1=:1 2=:2 3=:par3 4=:par4 5=:par5 6=:par6 7=:par7 8=:par8 9=:par9 where (id = :2)',true,True,True,psPostgreSQL));

  AssertEquals(     'update test set 1=$1 2=$2 3=$3 4=$4 5=$5 6=$6 7=$7 8=$8 9=$9 where (id = $3) and (test=''$test'')',
    params.ParseSQL('update test set 1=:1 2=:2 3=:par3 4=:par4 5=:par5 6=:par6 7=:par7 8=:par8 9=:par9 where (id = :par3) and (test=''$test'')',true,true,true,psPostgreSQL));

  AssertEquals(     'update test set 1=$1 2=$2 3=$3 4=$4 5=$5 6=$6 7=$7 8=$8 9=$9 10=$10 11=$11 12=$5 where (id = $3) and (test=''$test'')',
    params.ParseSQL('update test set 1=:1 2=:2 3=:par3 4=:par4 5=:par5 6=:par6 7=:par7 8=:par8 9=:par9 10=:par10 11=:11 12=:par5 where (id = :par3) and (test=''$test'')',true,true,true,psPostgreSQL));

  AssertEquals(     'select * from table where id = $1',
    params.ParseSQL('select * from table where id = :id',true,true,false,psSimulated,pb,ReplStr));
  AssertEquals('$',ReplStr);

  AssertEquals(     'update test set 1=$1 2=$2 3=$3 4=$4 5=$5 6=$6 7=$7 8=$8 9=$9 where (id = $2)',
    params.ParseSQL('update test set 1=:1 2=:2 3=:par3 4=:par4 5=:par5 6=:par6 7=:par7 8=:par8 9=:par9 where (id = :2)',true,true,false,psSimulated,pb,ReplStr));
  AssertEquals('$',ReplStr);

  AssertEquals(     'update test set 1=$$1 2=$$2 3=$$3 4=$$4 5=$$5 6=$$6 7=$$7 8=$$8 9=$$9 where (id = $$3) and (test=''$test'')',
    params.ParseSQL('update test set 1=:1 2=:2 3=:par3 4=:par4 5=:par5 6=:par6 7=:par7 8=:par8 9=:par9 where (id = :par3) and (test=''$test'')',true,true,false,psSimulated,pb,ReplStr));
  AssertEquals('$$',ReplStr);

  AssertEquals(     'update test set 1=$$1 2=$$2 3=$$3 4=$$4 5=$$5 6=$$6 7=$$7 8=$$8 9=$$9 10=$$10 11=$$11 12=$$5 where (id = $$3) and (test=''$test'')',
    params.ParseSQL('update test set 1=:1 2=:2 3=:par3 4=:par4 5=:par5 6=:par6 7=:par7 8=:par8 9=:par9 10=:par10 11=:11 12=:par5 where (id = :par3) and (test=''$test'')',true,True,True,psSimulated));
  AssertEquals('$$',ReplStr);

  AssertEquals(     'update test set 1=$$$1 2=$$$2 3=$$$3 4=$$$4 5=$$$5 6=$$$6 7=$$$7 8=$$$8 9=$$$9 10=$$$10 11=$$$11 12=$$$5 where (id$$ = $$$3) and (test$=''$test'')',
    params.ParseSQL('update test set 1=:1 2=:2 3=:par3 4=:par4 5=:par5 6=:par6 7=:par7 8=:par8 9=:par9 10=:par10 11=:11 12=:par5 where (id$$ = :par3) and (test$=''$test'')',true,true,False,psSimulated,pb,ReplStr));
  AssertEquals('$$$',ReplStr);

  AssertEquals(     'select * from table where id = ?',
    params.ParseSQL('select * from table where id = :id',true,true,true,psInterbase));

// Test bug 10345
  AssertEquals(     'select email from table where upper(email) like ''%''||?||''%''',
    params.ParseSQL('select email from table where upper(email) like ''%''||:email||''%''',true,true,true,psInterbase));

// Test escape-sequences:
  AssertEquals(     'select * from table where ''id '''' = :id''',
    params.ParseSQL('select * from table where ''id '''' = :id''',true,False,True,psPostgreSQL));

  AssertEquals(     'select * from table where "id "" = :id"',
    params.ParseSQL('select * from table where "id "" = :id"',true,False,True,psPostgreSQL));

  AssertEquals(     'select * from table where "id \" = :id"',
    params.ParseSQL('select * from table where "id \" = :id"',true,True,False,psPostgreSQL));

  AssertEquals(     'select * from table where "id \" = $1',
    params.ParseSQL('select * from table where "id \" = :id',true,False,False,psPostgreSQL));

  AssertEquals(     'select * from table where "id  = :id\',
    params.ParseSQL('select * from table where "id  = :id\',true,True,True,psInterbase));

// Test strange-field names
  AssertEquals(     'select * from table where "field-name" = ?',
    params.ParseSQL('select * from table where "field-name" = :"field-name"',true,True,True,psInterbase));
  AssertEquals('field-name',Params.Items[0].Name);

  AssertEquals(     'select * from table where "field-name" = ?',
    params.ParseSQL('select * from table where "field-name" = :"field-name',true,True,True,psInterbase));

// Test more than 99 params - bug 19645
  SQLStr := 'update test set';
  for i := 1 to 101 do
    SQLStr := format('%s field%d=:par%d', [SQLStr,i,i]);
  AssertEquals( StringReplace(SQLStr, ':par', '$', [rfReplaceAll]),
    Params.ParseSQL(SQLStr, True, True, True, psPostgreSQL) );

// Test comments:
  // Simple comment
  AssertEquals(     'select * from table where id= --comment :c'#10'$1-$2 or id= --:c'#13'-$3',
    Params.ParseSQL('select * from table where id= --comment :c'#10':a-:b or id= --:c'#13'-:d', True, True, True, psPostgreSQL));
  // Bracketed comment
  AssertEquals(     'select * from table where id=/*comment :c*/$1-$2',
    Params.ParseSQL('select * from table where id=/*comment :c*/:a-:b', True, True, True, psPostgreSQL));
  // Consecutive comments, with quote in second comment
  AssertEquals(     '--c1'#10'--c'''#10'select '':a'' from table where id=$1',
    Params.ParseSQL('--c1'#10'--c'''#10'select '':a'' from table where id=:id', True, True, True, psPostgreSQL));

  Params.Free;
end;

procedure TTestBasics.TestInitFielddefsFromFields;

var ds       : TDataset;
    F1,F2,F3 : Tfield;
    
  Procedure CompareFieldAndFieldDef(Fld: TField; FldDef : TFieldDef);
  
  begin
    AssertEquals(Fld.FieldName,FldDef.Name);
    AssertEquals(Fld.Size,FldDef.Size);
    AssertEquals(Fld.Required,FldDef.Required);
    AssertTrue(Fld.DataType=FldDef.DataType);
  end;
    
begin
  ds := TDataset.Create(nil);
  try

  F1:=TStringField.Create(ds);
  F1.Size := 10;
  F1.Name := 'StringFld';
  F1.FieldName := 'FStringFld';
  F1.Required := false;
  F1.Dataset:=ds;

  F2:=TIntegerField.Create(ds);
  F2.Name := 'IntegerFld';
  F2.FieldName := 'FIntegerFld';
  F2.Required := True;
  F2.Dataset:=ds;

  F3:=TBCDField.Create(ds);
  F3.Name := 'BCDFld';
  F3.FieldName := 'FBCDFld';
  F3.Required := false;
  F3.Dataset:=ds;
  (f3 as TBCDField).Precision := 2;
  
  HackedDataset(ds).InitFieldDefsFromfields;
  
  AssertEquals(3,ds.FieldDefs.Count);
  
  CompareFieldAndFieldDef(F1,ds.FieldDefs[0]);
  CompareFieldAndFieldDef(F2,ds.FieldDefs[1]);
  CompareFieldAndFieldDef(F3,ds.FieldDefs[2]);
  finally
    ds.Free;
  end;

end;

procedure TTestBasics.TestDoubleFieldDef;
var ds : TDataset;
    PassException : boolean;
begin
  // If a second field with the same name is added to a TFieldDefs, an exception
  // should occur
  ds := TDataset.create(nil);
  try
    ds.FieldDefs.Add('Field1',ftInteger);
    PassException:=False;
    try
      ds.FieldDefs.Add('Field1',ftString,10,false)
    except
      on E: EDatabaseError do PassException := True;
    end;
    AssertTrue(PassException);
  finally
    ds.Free;
  end;
end;

procedure TTestBasics.TestFieldDefWithoutDS;
var FieldDefs : TFieldDefs;
begin
  FieldDefs := TFieldDefs.Create(nil);
  try
    FieldDefs.Add('test',ftString);
  finally
    FieldDefs.Free;
  end;
end;

procedure TTestBasics.TestGetFieldList;
var
  ds: TDataSet;
  List: TList;
  ExceptionRaised: Boolean;
begin
  ds := CreateDatasetWith3Fields;
  try
    List := TList.Create;
    try
      //should not
      List.Clear;
      ds.GetFieldList(List, '');
      AssertEquals(0, List.Count);

      List.Clear;
      ExceptionRaised := False;
      try
        ds.GetFieldList(List, ' ');
      except
        on E: EDatabaseError do ExceptionRaised := True;
      end;
      AssertTrue(ExceptionRaised);

      List.Clear;
      ds.GetFieldList(List, 'Field1');
      AssertEquals(1, List.Count);

      List.Clear;
      ds.GetFieldList(List, ' Field1 ');
      AssertEquals(1, List.Count);

      List.Clear;
      ds.GetFieldList(List, 'Field1;Field2');
      AssertEquals(2, List.Count);

      List.Clear;
      ds.GetFieldList(List, 'Field1;Field2;');
      AssertEquals(2, List.Count);

      List.Clear;
      ds.GetFieldList(List, 'Field1;Field2;Field3');
      AssertEquals(3, List.Count);
    finally
      List.Destroy;
    end;
  finally
    ds.Destroy;
  end;
end;

procedure TTestBasics.TestGetParamList;
var
  Params: TParams;
  P: TParam;
  List: TList;
  ExceptionRaised: Boolean;
begin
  Params := TParams.Create(nil);
  try
    P := TParam.Create(Params, ptInput);
    P.Name := 'Param1';

    P := TParam.Create(Params, ptInput);
    P.Name := 'Param2';

    P := TParam.Create(Params, ptInput);
    P.Name := 'Param3';

    List := TList.Create;
    try
      List.Clear;
      Params.GetParamList(List, '');
      AssertEquals(0, List.Count);

      List.Clear;
      ExceptionRaised := False;
      try
        Params.GetParamList(List, ' ');
      except
        on E: EDatabaseError do ExceptionRaised := True;
      end;
      AssertTrue(ExceptionRaised);

      List.Clear;
      Params.GetParamList(List, 'Param1');
      AssertEquals(1, List.Count);

      List.Clear;
      Params.GetParamList(List, ' Param1 ');
      AssertEquals(1, List.Count);

      List.Clear;
      Params.GetParamList(List, 'Param1;');
      AssertEquals(1, List.Count);

      List.Clear;
      Params.GetParamList(List, 'Param1;Param2');
      AssertEquals(2, List.Count);

      List.Clear;
      Params.GetParamList(List, 'Param1;Param2;Param3');
      AssertEquals(3, List.Count);
    finally
      List.Destroy;
    end;
  finally
    Params.Destroy;
  end;
end;


procedure TTestBasics.TestExtractFieldName;
var
  i: Integer;
  Fields: String;
  FieldName: String;
begin
  Fields := '';
  i := 1;
  FieldName := ExtractFieldName(Fields, i);
  AssertEquals(1, i);
  AssertEquals('', FieldName);

  Fields := 'test';
  i := 1;
  FieldName := ExtractFieldName(Fields, i);
  AssertEquals(5, i);
  AssertEquals('test', FieldName);

  Fields := 'test;';
  i := 1;
  FieldName := ExtractFieldName(Fields, i);
  AssertEquals(6, i);
  AssertEquals('test', FieldName);

  Fields := ' test ';
  i := 1;
  FieldName := ExtractFieldName(Fields, i);
  AssertEquals(7, i);
  AssertEquals('test', FieldName);

  Fields := 'test;xxx';
  i := 1;
  FieldName := ExtractFieldName(Fields, i);
  AssertEquals(6, i);
  AssertEquals('test', FieldName);
  FieldName := ExtractFieldName(Fields, i);
  AssertEquals(9, i);
  AssertEquals('xxx', FieldName);
end;

procedure TTestBasics.TestCheckFieldNames;
var
  ds: TDataSet;
  ExceptionRaised: Boolean;
begin
  ds := CreateDatasetWith3Fields;
  try
    ExceptionRaised := False;
    try
      ds.Fields.CheckFieldNames('');
    except
      ExceptionRaised := True;
    end;
    AssertFalse(ExceptionRaised);

    ExceptionRaised := False;
    try
      ds.Fields.CheckFieldNames('Field1;Field2');
    except
      ExceptionRaised := True;
    end;
    AssertFalse(ExceptionRaised);

    ExceptionRaised := False;
    try
      ds.Fields.CheckFieldNames('Field1;NonExistentField');
    except
      ExceptionRaised := True;
    end;
    AssertTrue(ExceptionRaised);
  finally
    ds.Destroy;
  end;
end;

procedure TTestBasics.TestFindField;
var
  ds: TDataSet;
  F: TField;
begin
  ds := CreateDatasetWith3Fields;
  try
    F := ds.FindField('');
    AssertTrue(F = nil);

    F := ds.FindField('field3');
    AssertTrue(F <> nil);

    F := ds.FindField('NonExistentField');
    AssertTrue(F = nil);
  finally
    ds.Destroy;
  end;
end;

procedure TTestBasics.TestDoubleFieldDef;
var ds : TDataset;
    PassException : boolean;
begin
  // If a second field with the same name is added to a TFieldDefs, an exception
  // should occur
  ds := TDataset.create(nil);
  ds.FieldDefs.Add('Field1',ftInteger);
  PassException:=False;
  try
    ds.FieldDefs.Add('Field1',ftString,10,false)
  except
    on E: EDatabaseError do PassException := True;
  end;
  AssertTrue(PassException);
end;

procedure TTestBasics.TestFieldDefWithoutDS;
var FieldDefs : TFieldDefs;
begin
  FieldDefs := TFieldDefs.Create(nil);
  FieldDefs.Add('test',ftString);
  FieldDefs.Free;
end;

procedure TTestBasics.TestDoubleFieldDef;
var ds : TDataset;
    PassException : boolean;
begin
  // If a second field with the same name is added to a TFieldDefs, an exception
  // should occur
  ds := TDataset.create(nil);
  ds.FieldDefs.Add('Field1',ftInteger);
  PassException:=False;
  try
    ds.FieldDefs.Add('Field1',ftString,10,false)
  except
    on E: EDatabaseError do PassException := True;
  end;
  AssertTrue(PassException);
end;

procedure TTestBasics.TestFieldDefWithoutDS;
var FieldDefs : TFieldDefs;
begin
  FieldDefs := TFieldDefs.Create(nil);
  FieldDefs.Add('test',ftString);
  FieldDefs.Free;
end;

initialization
  RegisterTest(TTestBasics);
end.
