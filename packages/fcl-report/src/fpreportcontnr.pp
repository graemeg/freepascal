{
    This file is part of the Free Component Library.
    Copyright (c) 2017 Michael Van Canneyt, member of the Free Pascal development team

    Report Data loop classes based on object lists in contnrs unit.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpreportcontnr;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpreport, contnrs;

Type
  { TFPReportObjectData }

  TFPReportObjectData = class(TFPReportData)
  private
    FIndex : Integer;
  protected
    Function GetObjectCount : Integer; virtual; Abstract;
    Function GetObject(Aindex :Integer) : TObject; virtual; Abstract;
    Function GetObjectClass : TClass; virtual;
    procedure DoGetValue(const AFieldName: string; var AValue: variant); override;
    procedure DoInitDataFields; override;
    procedure DoOpen; override;
    procedure DoFirst; override;
    procedure DoNext; override;
    procedure DoClose; override;
    function  DoEOF: boolean; override;
  Public
    property  DataFields;
  end;

  { TFPReportCollectionData }

  TFPReportCollectionData = class(TFPReportObjectData)
  private
    FCollection: TCollection;
    FOwnsCollection: Boolean;
  Protected
    Function GetObjectCount : Integer; override;
    Function GetObject(Aindex :Integer) : TObject; override;
    Function GetObjectClass : TClass; override;
  Public
    Destructor Destroy; override;
    Property Collection : TCollection Read FCollection Write FCollection;
  Published
    Property OwnsCollection : Boolean Read FOwnsCollection Write FOwnsCollection;
  end;


  { TFPReportObjectListData }

  TFPReportObjectListData = class(TFPReportObjectData)
  private
    FList : TFPObjectList;
    FOwnsList: Boolean;
  Protected
    Function GetObjectCount : Integer; override;
    Function GetObject(Aindex :Integer) : TObject; override;
    Function GetObjectClass : TClass; override;
  Public
    Destructor Destroy; override;
    Property List : TFPObjectList Read FList Write FList;
  Published
    Property OwnsList : Boolean Read FOwnsList Write FOwnsList;
  end;

implementation

uses typinfo, variants;

{ TFPReportObjectListData }

function TFPReportObjectListData.GetObjectCount: Integer;
begin
  if Assigned(FList) then
    Result:=FList.Count
  else
    Result:=0;
end;

function TFPReportObjectListData.GetObject(Aindex: Integer): TObject;
begin
  if Assigned(FList) then
    Result:=FList[AIndex]
  else
    Result:=Nil;
end;

function TFPReportObjectListData.GetObjectClass: TClass;
begin
  if Assigned(FList) and (FList.Count>0) then
    Result:=FList[0].ClassType
  else
    Result:=Nil;
end;

destructor TFPReportObjectListData.Destroy;
begin
  if FOwnsList then
    FreeAndNil(Flist);
  inherited Destroy;
end;

{ TFPReportCollectionData }

function TFPReportCollectionData.GetObjectCount: Integer;
begin
  if Assigned(FCollection) then
    Result:=FCollection.Count
  else
    Result:=0;
end;

function TFPReportCollectionData.GetObject(Aindex: Integer): TObject;
begin
  if Assigned(FCollection) then
    Result:=FCollection.Items[AIndex]
  else
    Result:=Nil;
end;

function TFPReportCollectionData.GetObjectClass: TClass;

begin
  if Assigned(FCollection) then
    if (FCollection.Count>0) then
      Result:=FCollection.Items[0].ClassType
    else
      Result:=FCollection.ItemClass
  else
    Result:=Nil;
end;

destructor TFPReportCollectionData.Destroy;
begin
  if FOwnsCollection then
    FreeAndNil(FCollection);
  inherited Destroy;
end;

{ TFPReportObjectData }

function TFPReportObjectData.GetObjectClass: TClass;

Var
  O : TObject;

begin
  O:=GetObject(0);
  if Assigned(O) then
    Result:=O.ClassType
  else
    Result:=nil;
end;

procedure TFPReportObjectData.DoGetValue(const AFieldName: string;
  var AValue: variant);

Var
  O : TObject;
  PI : PPropInfo;

begin
  inherited DoGetValue(AFieldName, AValue);
  O:=GetObject(FIndex);
  if Assigned(O) then
    begin
    PI:=GetPropInfo(O,AFieldName);
    if Assigned(PI) then
      {$if FPC_FULLVERSION<=30000}
      aValue:=GetPropValue(O,PI^.Name,True);
      {$else}
      aValue:=GetPropValue(O,PI,True);
      {$endif}
    end;
end;

procedure TFPReportObjectData.DoInitDataFields;


Const
  tkAllowed = tkProperties -
              [tkArray,tkRecord,tkInterface,tkClass,
               tkObject,tkDynArray,tkInterfaceRaw,tkProcVar,
               tkHelper{$if FPC_FULLVERSION>30000},tkFile,tkClassRef,tkPointer{$ENDIF}];

Var
  C : TClass;
  PL : PPropList;
  I,Count : Integer;
  K : TFPReportFieldKind;
  Tk : TTypeKind;

begin
  inherited DoInitDataFields;
  C:=GetObjectClass;
  if C=Nil then exit;
  Count:=GetPropList(C,PL);
  try
    For I:=0 to Count-1 do
      begin
      TK:=PL^[i]^.PropType^.Kind;
      if (Tk in tkAllowed) then
        begin
        Case TK of
        tkInteger,tkInt64,tkQWord :
           K:=rfkInteger;
        tkSet,tkSString,tkLString,tkAString,

        tkChar,tkEnumeration,tkWChar,tkUString,tkUChar,tkWString,tkVariant :
           k:=rfkString;
        tkFloat :
           if PL^[i]^.PropType=TypeInfo(TDateTime) then
             K:=rfkDateTime
           else
             K:=rfkFloat;
       tkBool:
           K:=rfkBoolean;
       end;
       Datafields.AddField(PL^[i]^.Name,K);
       end;
     end;
  finally
    FreeMem(PL);
  end;
end;

procedure TFPReportObjectData.DoOpen;
begin
  inherited DoOpen;
  FIndex:=0;
end;

procedure TFPReportObjectData.DoFirst;
begin
  inherited DoFirst;
  FIndex:=0;
end;

procedure TFPReportObjectData.DoNext;
begin
  inherited DoNext;
  Inc(FIndex);
end;

procedure TFPReportObjectData.DoClose;
begin
  FIndex:=-1;
  inherited DoClose;
  DataFields.Clear;
end;

function TFPReportObjectData.DoEOF: boolean;
begin
  Result:=inherited DoEOF;
  Result:=Result or (FIndex<0) or (FIndex>=GetObjectCount);
end;



end.

