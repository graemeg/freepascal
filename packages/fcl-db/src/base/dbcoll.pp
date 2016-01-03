unit dbcoll;

interface

uses db, classes, sysutils;

{ ---------------------------------------------------------------------
  TFieldMap
  ---------------------------------------------------------------------}

type

  { TFieldMap }

  TFieldMap = Class(TObject)
  private
    FDataset: TDataset;
  Protected
    Function FindField(FN : String) : TField;
    Function FieldByName(FN : String) : TField;
  Public
    Constructor Create(ADataset : TDataset); virtual;
    Procedure InitFields; virtual; abstract;
    Procedure LoadObject(AObject: TObject); virtual; abstract;
    Function GetFromField(F : TField; ADefault : Integer) : Integer; overload;
    Function GetFromField(F : TField; ADefault : String) : String; overload;
    Function GetFromField(F : TField; ADefault : Boolean) : Boolean; overload;
    Function GetFromField(F : TField; ADefault : TDateTime) : TDateTime; overload;
    Function GetFromField(F : TField; ADefault : Double) : Double; overload;
    Function GetFromField(F : TField; ADefault : Currency) : Currency; overload;
    Property Dataset : TDataset Read FDataset;
  end;
  TFieldMapClass = Class of TFieldMap;
  
  EDBCollection = Class(Exception);

  { TDBCollectionItem }

  TDBCollectionItem = Class(TCollectionItem)
  Protected
    Procedure LoadFromMap(F : TFieldMap); virtual;
    Class Function FieldMapClass: TFieldMapClass; virtual; abstract;
  Public
    Procedure LoadFromDataset(ADataset : TDataset);
  end;
  
  { TDBCollection }

  TDBCollection = Class(TCollection)
  Protected
    Function AddDBItem : TDBCollectionItem;
    Procedure DoLoadFromFieldMap(Map : TFieldMap); virtual;
  Public
    Procedure LoadFromDataset(Dataset : TDataset);
  end;

implementation

resourcestring
<<<<<<< HEAD
<<<<<<< HEAD
  SErrNoDatasetForField = '%s: no dataset to search field %s in.';
=======
  SErrNoDatasetForField = '%s: Geen dataset om veld %s in te zoeken.';
>>>>>>> graemeg/fixes_2_2
=======
  SErrNoDatasetForField = '%s: Geen dataset om veld %s in te zoeken.';
>>>>>>> origin/fixes_2_2

{ TFieldMap }

constructor TFieldMap.Create(ADataset: TDataset);
begin
  FDataset:=ADataset;
  InitFields;
end;

function TFieldMap.FieldByName(FN: String): TField;
begin
  If (FDataset=Nil) then
    begin
    Raise EDBCollection.CreateFmt(SErrNoDatasetForField,[ClassName,FN]);
    result := nil;
    end
  else
    Result:=FDataset.FieldByName(FN);
end;

function TFieldMap.FindField(FN: String): TField;
begin
  If (FDataset=Nil) then
    Result:=Nil
  else
    Result:=FDataset.FindField(FN);
end;

function TFieldMap.GetFromField(F: TField; ADefault: Integer): Integer;
begin
  If Assigned(F) then
    Result:=F.AsInteger
  else
    Result:=ADefault;
end;

function TFieldMap.GetFromField(F: TField; ADefault: String): String;
begin
  If Assigned(F) then
    Result:=F.AsString
  else
    Result:=ADefault;
end;

function TFieldMap.GetFromField(F: TField; ADefault: Boolean): Boolean;
begin
  If Assigned(F) then
    begin
    if (F is TStringField) then
      Result:=(F.AsString='+')
    else
      Result:=F.AsBoolean
    end
  else
    Result:=ADefault;
end;

function TFieldMap.GetFromField(F: TField; ADefault: TDateTime): TDateTime;
begin
  If Assigned(F) then
    Result:=F.AsDateTime
  else
    Result:=ADefault;
end;

function TFieldMap.GetFromField(F: TField; ADefault: Double): Double;
begin
  If Assigned(F) then
    Result:=F.AsFloat
  else
    Result:=ADefault;
end;

function TFieldMap.GetFromField(F: TField; ADefault: Currency): Currency;
begin
  If Assigned(F) then
    Result:=F.AsCurrency
  else
    Result:=ADefault;
end;

{ TDBCollection }

function TDBCollection.AddDBItem: TDBCollectionItem;
begin
  Result:=Add as TDBCollectionItem;
end;

procedure TDBCollection.DoLoadFromFieldMap(Map: TFieldMap);

Var
  I : TDBCollectionItem;

begin
  While Not Map.Dataset.EOF do
    begin
    I:=AddDBItem;
    try
      I.LoadFromMap(Map);
    Except
      I.Free;
      Raise;
    end;
    Map.Dataset.Next;
    end;
end;

procedure TDBCollection.LoadFromDataset(Dataset: TDataset);

Var
  M : TFieldMap;

begin
  M:=TDBCollectionItem(ItemClass).FieldMapClass.Create(Dataset);
  Try
    DoLoadFromFieldMap(M);
  finally
    M.Free;
  end;
end;

{ TDBCollectionItem }

procedure TDBCollectionItem.LoadFromMap(F: TFieldMap);
begin
  F.LoadObject(Self);
end;

procedure TDBCollectionItem.LoadFromDataset(ADataset: TDataset);

Var
  M : TFieldMap;

begin
  M:=FieldMapClass.Create(ADataset);
  Try
    LoadFromMap(M);
  Finally
    M.Free;
  end;
end;

end.
 
