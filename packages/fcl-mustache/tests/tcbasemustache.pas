{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2021 by Michael Van Canneyt (michael@freepascal.org)

    Helper classes for Mustache test cases

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit tcbasemustache;

{$mode ObjFPC}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, fpmustache;

type

  { TTestContext }

  (* StringList with following encoding
    // Null value
    aName=<null>
    // false value
    aName=<null>
    // plain value
    aName=AValue
    // Object value & member. Object value must be present
    SubObj={}
    SubObj.aName=aValue
    // Array and members. Array value must be present
    SubObj.SubArr=[]
    SubObj.SubArr[0]={}
    SubObj.SubArr[0].aName=aValue
    SubObj.SubArr[1]={}
    Subobj.SubArr[1].aName=aValue
  *)

  TTestContext = class (TMustacheContext)
  Private
    FValues : TStringList;
    FPath : String;
  public
    Constructor Create(aCallback: TGetTextValueEvent); override;
    Destructor destroy; override;
    Function GetTextValue(Const aName : TMustacheString) : TMustacheString; override;
    Function MoveNextSectionItem(Const aName : TMustacheString) : Boolean; override;
    Function PushSection(Const aName : TMustacheString) : TMustacheSectionType; override;
    Procedure PopSection(Const aName : TMustacheString); override;
    Procedure SetValue(const aPath,aValue : string);
    Property Values : TStringList read FValues;
  end;

  TBaseMustacheTest = class(TTestCase)
  Private
    FPartials: TStrings;
    FTemplate: String;
    FResult: TMustacheElement;
    FParser: TMustacheParser;
  Protected
    Function CreateParser : TMustacheParser; virtual; abstract;
    Procedure DoGetPartial(const aName: TMustacheString; var aHandled: Boolean;  var aValue: TMustacheString);
  Public
    Class Procedure AssertEquals(Msg : String; aExpected,aActual : TMustacheElementType); overload;
    Class Function AssertElement(aParent : TMustacheElement; aIndex: Integer; aType: TMustacheElementType; aData: String; aClass : TMustacheElementClass = Nil) : TMustacheElement; overload;
    Function AssertElement(aIndex: Integer; aType: TMustacheElementType; aData: String; aClass : TMustacheElementClass = Nil) : TMustacheElement; overload;
    Procedure AssertResultCount(aCount : Integer);
    procedure SetUp; override;
    procedure TearDown; override;
    Procedure CallParser;
    Procedure AddPartial(Const aName,aText: TMustacheString);
    Property Partials : TStrings Read FPartials;
    Property Template : String Read FTemplate Write FTemplate;
    property ParseResult : TMustacheElement Read FResult;
    property Parser : TMustacheParser Read FParser;
  end;


implementation

uses strutils, typinfo;

{ TTestContext }

constructor TTestContext.Create(aCallback: TGetTextValueEvent);
begin
  inherited Create(aCallback);
  FValues:=TStringList.Create;
  FValues.OwnsObjects:=True;
end;

destructor TTestContext.destroy;
begin
  FreeAndNil(FValues);
  inherited destroy;
end;

function TTestContext.GetTextValue(const aName: TMustacheString
  ): TMustacheString;

Var
  aPath,N : String;
  Done : Boolean;
begin
  Result:='';
  aPath:=FPath;
  Done:=False;
  Repeat
    if aPath<>'' then
      N:=aPath+'.'+aName
    else
      begin
      N:=aName;
      Done:=True;
      end;
    Result:=FValues.Values[N];
    if not Done then
      aPath:=Copy(aPath,1,RPos('.',aPath)-1);
  until (Result<>'') or Done;
end;

function TTestContext.MoveNextSectionItem(const aName: TMustacheString
  ): Boolean;

Var
  L,P,Idx : Integer;
  N : String;

begin
  L:=Length(FPath);
  if (L>0) and (FPath[L]=']') then
    begin
    P:=RPos('[',FPath)+1;
    Idx:=StrToIntDef(Copy(FPath,P,L-P),-1);
    N:=Copy(FPath,1,P-1)+IntToStr(Idx+1)+']';
    Result:=FValues.Values[N]<>''; // We could check for {}
    if Result then
      FPath:=N;
    end;

end;

function TTestContext.PushSection(const aName: TMustacheString): TMustacheSectionType;

Var
  aPath,S : String;

begin
  if FPath<>'' then
    FPath:=FPath+'.';
  aPath:=FPath+aName;
  S:=Values.Values[aPath];
  if S='{}' then
    begin
    FPath:=aPath;
    result:=mstSingle;
    end;
  if S='[]' then
    begin
    if Values.Values[aPath+'[0]']='' then
      Result:=mstNone
    else
      begin
      FPath:=aPath+'[-1]';
      result:=mstList;
      end;
    end
  else if (s='<null>') or (s='<false>') or (s='') then
    begin
    Result:=mstNone;
    end
  else
    begin
    FPath:=aPath;
    result:=mstSingle;
    end;

end;

procedure TTestContext.PopSection(const aName: TMustacheString);
begin
  FPath:=Copy(FPath,1,RPos('.',FPath)-1);
end;

procedure TTestContext.SetValue(const aPath, aValue: string);
begin
  Values.Values[aPath]:=aValue;
end;


{ TBaseMustacheTest }

procedure TBaseMustacheTest.SetUp;

begin
  Inherited;
  FParser:=CreateParser;
  FParser.Partials:=TMustachePartialList.Create(metRoot,Nil,0);
  FParser.OnGetPartial:=@DoGetPartial;
  FPartials:=TStringList.Create;
  TStringList(FPartials).OwnsObjects:=True;
end;

procedure TBaseMustacheTest.TearDown;

begin
  FreeAndNil(FPartials);
  FreeAndNil(FResult);
  FParser.Partials.Free;
  FreeAndNil(FParser);
  Inherited;
end;

procedure TBaseMustacheTest.DoGetPartial(const aName: TMustacheString;
  var aHandled: Boolean; var aValue: TMustacheString);
begin
  aValue:=FPartials.Values[aName];
  aHandled:=FPartials.IndexOfName(aName)<>-1;
end;

class function TBaseMustacheTest.AssertElement(aParent: TMustacheElement;
  aIndex: Integer; aType: TMustacheElementType; aData: String;
  aClass: TMustacheElementClass): TMustacheElement;
Var
  El : TMustacheElement;
  aChild : String;
begin
  AssertNotNull('Have parent',aParent);
  AssertTrue(Format('Index %d in range 0..%d',[aIndex,aParent.ChildCount-1]),(aIndex>=0) and (aIndex<aParent.ChildCount));
  EL:=aParent.Children[aIndex];
  aChild:=Format('Child %d',[aIndex]);
  AssertNotNull('Have result '+aChild,El);
  AssertEquals(aChild+' has correct type',aType,El.ElementType);
  AssertEquals(aChild+' has correct data',aData,El.Data);
  if (aClass<>Nil) then
    AssertEquals(aChild+' has correct class',aClass,el.Classtype);
  Result:=El;
end;

function TBaseMustacheTest.AssertElement(aIndex: Integer;
  aType: TMustacheElementType; aData: String; aClass : TMustacheElementClass = Nil): TMustacheElement;

begin
  AssertNotNull('Have result',FResult);
  Result:=AssertElement(FResult,aIndex,aType,aData,aClass);
end;

procedure TBaseMustacheTest.AssertResultCount(aCount: Integer);
begin
  AssertNotNull('Have result',FResult);
  AssertEquals('Result count',aCount,FResult.ChildCount);
end;


procedure TBaseMustacheTest.CallParser;

begin
  Parser.Template:=Template;
  FResult:=Parser.Parse;
end;

procedure TBaseMustacheTest.AddPartial(const aName, aText: TMustacheString);

//Var
//  T : TMustacheTextElement;

begin
//  T:=TMustacheTextElement.Create(metText,Nil,0);
//  T.Data:=aText;
  FPartials.Add(aName+'='+atext);
end;

class procedure TBaseMustacheTest.AssertEquals(Msg: String; aExpected,
  aActual: TMustacheElementType);

begin
  AssertEquals(Msg,GetEnumName(typeInfo(TMustacheElementType),Ord(aExpected)),
                       GetEnumName(typeInfo(TMustacheElementType),Ord(aActual)));
end;


end.

