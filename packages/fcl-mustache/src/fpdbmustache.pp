{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2021 by Michael Van Canneyt (michael@freepascal.org)

    This file contains a Mustache DB context, getting data from a dataset

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit fpdbmustache;

{$mode ObjFPC}{$H+}

interface

uses
  Classes, SysUtils, db, fpMustache;

Type

  { TDatasetCollectionItem }

  TDatasetCollectionItem = Class(TCollectionItem)
  private
    FDataset: TDataSet;
    FSection: String;
  Public
    Property Dataset : TDataSet Read FDataset Write FDataset;
    Property SectionName : String Read FSection Write FSection;
  end;

  TDatasetCollection = Class(TCollection)
  private
    function GetDS(aIndex : Integer): TDatasetCollectionItem;
  Public
    Function IndexOfDataset(aDataset : TDataset) : Integer;
    Function IndexOfSection(aSection : String) : Integer;
    Property Datasets[aIndex : Integer] : TDatasetCollectionItem Read GetDS; default;
  end;

  { TMustacheDBContext }

  TMustacheDBContext = Class(TMustacheContext)
  Private
    Type
      TPair = Record
        atStart : Boolean;
        Value : TDataset;
      end;
  Private
    FStack : Array of TPair;
    FCount : Integer;
    FStaticValues: TStrings;
    FDatasets : TDatasetCollection;
    Function FindField(Const aName : TMustacheString) : TField;
    function GetDataset(aIndex : Integer): TDatasetCollectionItem;
    function GetDatasetCount: INteger;
    procedure SetStaticValues(AValue: TStrings);
  Public
    Constructor Create(aCallback : TGetTextValueEvent); override;
    Destructor destroy; override;
    Procedure Clear;
    Function MoveNextSectionItem(Const aName : TMustacheString) : Boolean; override;
    Function PushSection(Const aName : TMustacheString) : TMustacheSectionType; override;
    Procedure PopSection(Const aName : TMustacheString); override;
    Function GetTextValue(Const aName : TMustacheString) : TMustacheString; override;
    Procedure AddDataset(aDataset : TDataset; aSectionName : String = '');
    Procedure RemoveDataset(aDataset : TDataset);
    Property StaticValues : TStrings Read FStaticValues Write SetStaticValues;
    Property Datasets[aIndex : Integer] : TDatasetCollectionItem Read GetDataset;
    Property DatasetCount : INteger Read GetDatasetCount;
  end;

implementation

uses StrUtils;

Resourcestring
  SErrPopSectionNoPush = 'PopSection %s without push';
  SErrDatasetNameEmpty = 'Dataset name and section cannot both be empty';
  SErrDatasetEmpty = 'Dataset is Nil';
  SErrDuplicateDataSetName = 'Duplicate dataset name: %s';

{ TMustacheDBContext }

function TMustacheDBContext.FindField(const aName: TMustacheString): TField;

Var
  aCount : Integer;

begin
  Result:=Nil;
  aCount:=FCount-1;
  While (Result=Nil) and (aCount>=0) do
    begin
    Result:=FStack[aCount].Value.FieldByName(aName);
    Dec(aCount);
    end;
end;

function TMustacheDBContext.GetDataset(aIndex : Integer
  ): TDatasetCollectionItem;
begin
  Result:=FDatasets[aIndex];
end;

function TMustacheDBContext.GetDatasetCount: INteger;
begin
  Result:=FDatasets.Count;
end;

procedure TMustacheDBContext.SetStaticValues(AValue: TStrings);
begin
  if FStaticValues=AValue then Exit;
  FStaticValues.Assign(AValue);
end;

constructor TMustacheDBContext.Create(aCallback: TGetTextValueEvent);
begin
  inherited Create(aCallback);
  FDatasets:=TDatasetCollection.Create(TDatasetCollectionItem);
  FStaticValues:=TStringList.Create;
  SetLength(FStack,JSONListGrowCount);
  FCount:=0;
end;

destructor TMustacheDBContext.destroy;
begin
  FreeAndNil(FStaticValues);
  FreeAndNil(FDatasets);
  inherited destroy;
end;

procedure TMustacheDBContext.Clear;
begin
  FStaticValues.Clear;
  FDatasets.Clear;
end;

function TMustacheDBContext.MoveNextSectionItem(const aName: TMustacheString
  ): Boolean;
begin
  if FStack[FCount-1].atStart then
    FStack[FCount-1].atStart:=False
  else
    FStack[FCount-1].Value.Next;
  Result:=Not FStack[FCount-1].Value.EOF;
end;

function TMustacheDBContext.PushSection(const aName: TMustacheString
  ): TMustacheSectionType;

Var
  aDS : TDataset;
  Idx : Integer;
begin
  Result:=mstNone;
  Idx:=FDatasets.IndexOfSection(aName);
  if Idx=-1 then
    Exit;
  aDS:=FDatasets[Idx].Dataset;
  if aDS.IsEmpty then
    exit;
  if FCount=Length(FStack) then
    SetLength(FStack,FCount+JSONListGrowCount);
  FStack[FCount].Value:=aDS;
  FStack[FCount].atStart:=True;
  Inc(FCount,1);
  Result:=mstList;
end;

procedure TMustacheDBContext.PopSection(const aName: TMustacheString);
begin
  if FCount<1 then
    Raise EMustache.CreateFmt(SErrPopSectionNoPush,[aName]);
  Dec(FCount,1);
end;

function TMustacheDBContext.GetTextValue(const aName: TMustacheString
  ): TMustacheString;

Var
  F : TField;
  idx : Integer;

begin
  F:=Nil;
  if Pos('.',aName)=0 then
    F:=FindField(aName)
  else if WordCount(aName,['.'])=2 then
    begin
    Idx:=FDatasets.IndexOfSection(ExtractWord(1,aName,['.']));
    if (Idx<>-1) then
      F:=FDatasets[Idx].Dataset.FindField(ExtractWord(2,aName,['.']));
    end;
  If Assigned(F) then
    Result:=F.AsString
  else
    begin
    Idx:=FStaticValues.IndexOfName(aName);
    if Idx<>-1 then
      Result:=FStaticValues.ValueFromIndex[Idx]
    else
      Result:=Inherited GetTextValue(aName);
    end;
end;

procedure TMustacheDBContext.AddDataset(aDataset: TDataset; aSectionName: String);

Var
  DCI : TDatasetCollectionItem;
  aName : String;

begin
  aName:=aSectionName;
  if aName='' then
    aName:=aDataset.Name;
  if aName='' then
    raise EMustache.Create(SErrDatasetNameEmpty);
  if aDataset=Nil then
    raise EMustache.Create(SErrDatasetEmpty);
  if FDatasets.IndexOfSection(aName)<>-1 then
    raise EMustache.CreateFmt(SErrDuplicateDataSetName, [aName]);
  DCI:=FDatasets.Add as TDatasetCollectionItem;
  DCI.Dataset:=aDataset;
  DCI.SectionName:=aName;
end;

procedure TMustacheDBContext.RemoveDataset(aDataset: TDataset);

Var
  Idx : Integer;

begin
  Idx:=FDatasets.IndexOfDataset(aDataset);
  if Idx<>-1 then
    FDatasets.Delete(Idx);
end;

{ TDatasetCollection }

function TDatasetCollection.GetDS(aIndex : Integer): TDatasetCollectionItem;
begin
  Result:=Items[aIndex] as TDatasetCollectionItem;
end;

function TDatasetCollection.IndexOfDataset(aDataset: TDataset): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (GetDS(Result).Dataset<>ADataset) do
    Dec(Result);
end;

function TDatasetCollection.IndexOfSection(aSection: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and not SameText(GetDS(Result).SectionName,ASection) do
    Dec(Result);
end;

end.

