{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2021 by Michael Van Canneyt (michael@freepascal.org)

    This file contains a Mustache parser and renderer.

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit fpmustache;

{$mode objfpc}{$H+}
{$WARN 5024 off : Parameter "$1" not used}
interface

uses
  Classes, SysUtils, fpjson;

Type
  EMustache = Class(Exception);

  TMustacheString = UTF8String;
  TMustacheChar = AnsiChar;
  TMustacheContext = class;

  TMustacheOutput = Class(TObject)
  Public
    // add atext to output
    Procedure Output(Const aText : TMustacheString); virtual; abstract;
    Procedure Reset; virtual; abstract;
  end;

  { TMustacheStringOutput }

  TMustacheStringOutput = Class(TMustacheOutput)
  private
    FData: TMustacheString;
  Public
    // Override
    Procedure Output(Const aText : TMustacheString); override;
    Procedure Reset; override;
    // The rendered TMustacheString
    Property Data : TMustacheString Read FData;
  end;

  { TMustacheElement }

  TMustacheElementType = (metRoot,metComment,metText,metVariable,metSection,metInvertedSection,metPartial);

  TMustacheElement = Class(TObject)
  private
    FPosition: Integer;
    FType : TMustacheElementType;
    FParent : TMustacheElement;
  Protected
    function GetCount: Integer; virtual;
    function GetElement(aIndex : Integer): TMustacheElement; virtual;
    Function GetData : TMustacheString ; virtual; abstract;
    Procedure SetData(Const aData : TMustacheString) ; virtual; abstract;
    Function GetPrefix : TMustacheString; virtual;
    Procedure SetPrefix (aValue : TMustacheString); virtual;
    Procedure Dump(aList : Tstrings; aIndent : TMustacheString; aDumpChildren : Boolean = true); virtual;
  Public
    Constructor Create(aType : TMustacheElementType; aParent : TMustacheElement;aPosition : Integer); virtual;
    // Add a child. Parent always owns child
    Procedure AddChild(aChild : TMustacheElement); virtual;
    // Render the text for this element
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput; const aPrefix : String = ''; aLast : Boolean = False); virtual; abstract;
    // Position in template
    Property Position : Integer Read FPosition;
    // Parent element
    Property Parent : TMustacheElement Read FParent;
    // Access to children
    Property Children[aIndex : Integer] : TMustacheElement Read GetElement;
    // Child count
    Property ChildCount : Integer Read GetCount;
    // Element type
    Property ElementType : TMustacheElementType Read FType;
    // The data for this element. What this is, depends on the kind.
    // etText : the text;
    // etValue : the variable name
    // etSection : the section name.
    // etInvertedSection : the section name.
    Property Data : TMustacheString Read GetData Write SetData;
    // Whitespace prefix. Normally only used for partials
    Property Prefix : TMustacheString Read GetPrefix Write SetPrefix;
  end;
  TMustacheElementClass = Class of TMustacheElement;
  TMustacheElementArray = Array of TMustacheElement;

  { TMustacheNamedElement }

  TMustacheNamedElement = Class(TMustacheElement)
  private
    FName: TMustacheString;
  Protected
    Procedure SetData(Const aData : TMustacheString); override;
    Function GetData : TMustacheString; override;
  Public
    Property Name : TMustacheString Read FName;
  end;

  { TMustacheParentElement }

  TMustacheParentElement = Class(TMustacheNamedElement)
  Private
    FChildren : TMustacheElementArray;
    FCount : Integer;
  Protected
    function GetElement(aIndex : Integer): TMustacheElement; override;
    function GetCount : Integer; override;
  Public
    Destructor Destroy; override;
    Procedure AddChild(aChild : TMustacheElement); override;
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput; const aPrefix : String = ''; aLast : Boolean = False); override;
  end;


  { TMustacheTextElement }

  TMustacheTextElement = Class(TMustacheElement)
  Private
    FData : TMustacheString;
  Protected
    Procedure SetData(Const aData : TMustacheString) ; override;
    Function GetData : TMustacheString; override;
  Public
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput; const aPrefix : String = ''; aLast : Boolean = False); override;
  end;

  { TMustacheVariableElement }

  TMustacheVariableElement = Class(TMustacheNamedElement)
  private
    FNoUnescape: Boolean;
  Protected
    Procedure SetData(Const aData : TMustacheString); override;
  Public
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput; const aPrefix : String = ''; aLast : Boolean = False); override;
    Property NoUnescape : Boolean Read FNoUnescape;
  end;

  { TMustacheSectionElement }

  TMustacheSectionElement = Class(TMustacheParentElement)
  Public
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput; const aPrefix : String = ''; aLast : Boolean = False); override;
  end;

  { TMustachePartialElement }

  TMustachePartialElement = Class(TMustacheElement)
  Private
    FPrefix : TMustacheString;
    FPartialName : TMustacheString;
    FPartial : TMustacheElement;
  Protected
    Function GetData : TMustacheString ; override;
    Procedure SetData(Const aData : TMustacheString) ; override;
    Procedure Dump(aList : Tstrings; aIndent : TMustacheString; aDumpChildren : Boolean = true); override;
    Function GetPrefix : TMustacheString; override;
    Procedure SetPrefix (aValue : TMustacheString); override;
  Public
    Destructor Destroy; override;
    Procedure AddChild(aChild: TMustacheElement); override;
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput; const aPrefix : String = ''; aLast : Boolean = False); override;
    Property Partial : TMustacheElement Read FPartial;
  end;

  { TMustachePartialList }

  TMustachePartialList = Class(TMustacheParentElement)
  Public
    Function FindPartial(aName : TMustacheString) : TMustacheElement;
  end;

  { TMustacheParser }
  TGetTextValueEvent = Procedure (Const aName : TMustacheString; var aHandled : Boolean; var aValue : TMustacheString) of Object;

  TMustacheParser = Class(TObject)
  private
    FStopTag: TMustacheString;
    FStartTag: TMustacheString;
    FTemplate: TMustacheString;
    FOnGetPartial: TGetTextValueEvent;
    FPartials: TMustachePartialList;
    Class var DefaultTypes : Array[TMustacheElementType] of TMustacheElementClass;
  Protected
    // Called to create a default element for a {{ }} tag. By default creates a variable.
    // Override this if you want to create additional elements.
    function CreateDefault(aParent: TMustacheElement; aPosition: Integer; const aName: String): TMustacheElement; virtual;
    // Create element for indicated type, must add it to parent.
    // You can override this to provide customized behaviour.
    function CreateElement(aType: TMustacheElementType; aParent: TMustacheElement; aPosition: Integer): TMustacheElement; virtual;
    // Parse
    Procedure DoParse(aParent : TMustacheElement; Const aTemplate, aStart, aStop : TMustacheString); virtual;
    // Called to get the template of a partial. The template is parsed, and the result added to the partials list.
    Function GetPartial(const aName : TMustacheString) : TMustacheString; virtual;
    // Auxuliary functions for the peculiar whitespace handling of Mustache specs...
    function EndsOnWhiteSpace(aElement: TMustacheElement): Boolean; virtual;
    function GetEndingWhiteSpace(aElement: TMustacheElement): TMustacheString; virtual;
    procedure ExtractStartStop(const aName: TMustacheString; out aStart, aStop: TMustacheString); virtual;
    procedure TrimEndingWhiteSpace(aElement: TMustacheElement); virtual;
  Public
    // Create a new parser.
    Constructor Create(aTemplate : TMustacheString = '';aStart: TMustacheString='';aStop: TMustacheString = ''); virtual;
    // Set the default TMustacheElements for the descendents
    Class procedure SetDefaultTypeClass(aType : TMustacheElementType; aClass: TMustacheElementClass);
    // Parse the template and
    Procedure Parse(aParent : TMustacheElement);
    Function Parse : TMustacheElement;
    // Will be used to hold partials. You must set this before calling Parse.
    Property Partials : TMustachePartialList Read FPartials Write FPartials;
    // The template created on startup
    Property Template : TMustacheString Read FTemplate write FTemplate;
    // The initial start tag marker, by default {{
    Property StartTag : TMustacheString Read FStartTag Write FStartTag;
    // The initial end tag marker, by default }}
    Property StopTag : TMustacheString Read FStopTag Write FSTopTag;
    // Event called to get the source of a partial.
    Property OnGetPartial : TGetTextValueEvent Read FOnGetPartial Write FOnGetPartial;
  end;

  { TMustacheContext }
  TMustacheSectionType = (mstNone,mstSingle,mstList);

  TMustacheContext = Class(TObject)
  Private
    FCallback : TGetTextValueEvent;
  Public
    Constructor Create(aCallback : TGetTextValueEvent); virtual;
    // Helper function to quote HTML
    Class Function QuoteHTML(aString : TMustacheString) :TMustacheString; virtual;
    // Move to next section item. aName is section name. Returns True if move successful
    Function MoveNextSectionItem(Const aName : TMustacheString) : Boolean; virtual;
    // Push a new section context with name aName.
    Function PushSection(Const aName : TMustacheString) : TMustacheSectionType; virtual;
    // Pop current section. aName is for verification.
    Procedure PopSection(Const aName : TMustacheString); virtual;
    // Return the value of a variable with name aName.
    Function GetTextValue(Const aName : TMustacheString) : TMustacheString; virtual;
  end;

  { TMustacheJSONContext }

  TMustacheJSONContext = Class(TMustacheContext)
  Private
    Type
      TPair = Record
        Index : Integer; // if array, index of current element.
        Value : TJSONData;
      end;
  Private
    FCurrentData: TJSONData;
    FStack : Array of TPair;
    FCount : Integer;
    Function FindValue(Const aName : TMustacheString) : TJSONData;
    function GetRootData: TJSONData;
  Public
    Constructor Create(aJSON : TJSONData; aCallback : TGetTextValueEvent); reintroduce;
    Function MoveNextSectionItem(Const aName : TMustacheString) : Boolean; override;
    Function PushSection(Const aName : TMustacheString) : TMustacheSectionType; override;
    Procedure PopSection(Const aName : TMustacheString); override;
    Function GetTextValue(Const aName : TMustacheString) : TMustacheString; override;
    Property RootData : TJSONData read GetRootData;
  end;

  TMustache = Class(TComponent)
  private
    FCompiled: TMustacheElement;
    FCompiledPartials: TMustachePartialList;
    FOnGetValue: TGetTextValueEvent;
    FPartials: TStrings;
    FStartTag: TMustacheString;
    FStopTag: TMustacheString;
    FTemplate: TMustacheString;
    procedure SetPartials(AValue: TStrings);
    procedure SetStartTag(AValue: TMustacheString);
    procedure SetStopTag(AValue: TMustacheString);
    procedure SetTemplate(AValue: TMustacheString);
  Protected
    Procedure DoGetPartial(Const aName : TMustacheString; var aHandled : Boolean; var aValue : TMustacheString); virtual;
    Procedure Reset; virtual;
    Function CreatePartials : TMustachePartialList;
    function CreateParser(aTemplate: TMustacheString): TMustacheParser; virtual;
    Property Compiled : TMustacheElement Read FCompiled;
    Property CompiledPartials : TMustachePartialList Read FCompiledPartials;
  Public
    Constructor Create(aOwner : TComponent); override;
    Destructor Destroy; override;
    Procedure Compile;
    Procedure Dump(aList : Tstrings; aindent : TMustacheString); overload; virtual;
    Function Dump: TMustacheString;overload;
    Procedure Render(aContext : TMustacheContext; aOutput : TMustacheOutput); virtual; overload;
    Function Render(aContext : TMustacheContext) : TMustacheString; overload;
    Function Render(const aJSON : TJSONData) : TMustacheString; overload;
    Function Render(const aJSON : TJSONStringType) : TMustacheString; overload;
    Class function CreateMustache(aOwner: TComponent; aTemplate: TMustacheString): TMustache; virtual;
    Class Function Render(aTemplate : TMustacheString; const aJSON : TJSONStringType) : TMustacheString;
  Published
    Property Template : TMustacheString Read FTemplate Write SetTemplate;
    Property OnGetValue : TGetTextValueEvent Read FOnGetValue Write FOnGetValue;
    Property StartTag : TMustacheString Read FStartTag Write SetStartTag;
    Property StopTag : TMustacheString Read FStopTag Write SetStopTag;
    Property Partials : TStrings Read FPartials Write SetPartials;
  end;


Const
  ListGrowCount = 10;
  JSONListGrowCount = 10;

implementation

uses TypInfo;



Resourcestring
  SErrNoChildForElement = 'Class %s does not support child elements.';
  SErrInvalidIndex= '%s: Index %d is not in valid range [0..%d].';
  SErrUnterminatedTag = 'Tag %s opened on position %d but not closed.';
  SErrEmptyTag = 'Tag %s on position %d is empty.';
  SErrSectionClose = 'Structural error: Section "%s" on position %d is closed by tag "%s" on position %d.';
  SErrNotClosedSection = 'Structural error: Section "%s" on position %d is not closed.';
  SErrNoSectionToClose = 'Structural error: Section "%s" on position %d was never opened.';
  SErrInvalidDelimiter = 'Invalid set delimiter: %s';
  SErrInvalidDelimiterValue = 'Invalid set delimiter %s value: %s in "%s"';
  SErrNoPartials = 'No partials list';

  // SErrPartialNotFound = 'Partial "%s" not found.';
  SStartTag = 'Start';
  SStopTag = 'Stop';

{ TMustachePartialList }

function TMustachePartialList.FindPartial(aName: TMustacheString ): TMustacheElement;

Var
  I : Integer;

begin
  Result:=Nil;
  I:=ChildCount-1;
  While (Result=Nil) and (I>=0) do
    begin
    Result:=Children[I];
    If (Result.Data<>aName) then
      Result:=Nil;
    Dec(I);
    end;
end;

{ TMustachePartialElement }

function TMustachePartialElement.GetData: TMustacheString;
begin
  Result:=FPartialName;
end;

procedure TMustachePartialElement.SetData(const aData: TMustacheString);
begin
  FPartialName:=aData;
end;

procedure TMustachePartialElement.AddChild(aChild: TMustacheElement);
begin
  If (FPartial<>Nil) and (aChild<>Nil) then
    Raise EMustache.Create('Cannot set partial twice');
  FPartial:=aChild;
end;

procedure TMustachePartialElement.Dump(aList: Tstrings;
  aIndent: TMustacheString; aDumpChildren: Boolean);
begin
  inherited Dump(aList, aIndent, aDumpChildren);
  if Prefix<>'' then
    aList[aList.Count-1]:=aList[aList.Count-1]+' Prefix: "'+Prefix+'"';
end;

function TMustachePartialElement.GetPrefix: TMustacheString;
begin
  Result:=FPrefix;
end;

procedure TMustachePartialElement.SetPrefix(aValue: TMustacheString);
begin
  FPrefix:=aValue;
end;

procedure TMustachePartialElement.Render(aContext: TMustacheContext;
  aOutput: TMustacheOutput; const aPrefix : String = ''; aLast : Boolean = False);

begin
  FPartial.Render(aContext,aOutput,Prefix);
end;

destructor TMustachePartialElement.Destroy;
begin
  inherited Destroy;
end;


{ TMustache }

function TMustache.CreateParser(aTemplate : TMustacheString): TMustacheParser;
begin
  Result:=TMustacheParser.Create(aTemplate);
end;

constructor TMustache.Create(aOwner: TComponent);
begin
  Inherited;
  FPartials:=TStringList.Create;
  FCompiledPartials:=CreatePartials;
end;

destructor TMustache.Destroy;
begin
  Reset;
  FreeAndNil(FPartials);
  FreeAndNil(FCompiledPartials);
  inherited Destroy;
end;

procedure TMustache.Compile;

Var
  Parser : TMustacheParser;

begin
  Parser:=CreateParser(Self.Template);
  try
    Parser.OnGetPartial:=@DoGetPartial;
    //Parser.Template:=Self.Template;
    Parser.Partials:=Self.FCompiledPartials;
    if Self.StartTag<>'' then
      Parser.StartTag:=Self.StartTag;
    if Self.StopTag<>'' then
      Parser.StopTag:=Self.StopTag;
    FCompiled:=Parser.Parse;
  finally
    Parser.Free;
  end;
end;

procedure TMustache.Dump(aList: Tstrings; aindent: TMustacheString);
begin
  if Assigned(Compiled) then
    Compiled.Dump(aList,aIndent);
end;

function TMustache.Dump: TMustacheString;

Var
  I : integer;
  L : TStrings;

begin
  L:=TStringList.Create;
  try
    Dump(L,'');
    if Partials.Count>0 then
      begin
      L.Add('Partials:');
      for I:=0 to Partials.Count-1 do
        L.Add('Partial '+IntToStr(I)+': '+Partials[i]);
      L.Add('End of partials');
      end;
    Result:=L.Text;
  finally
    L.Free;
  end;
end;

procedure TMustache.Render(aContext: TMustacheContext; aOutput: TMustacheOutput);

begin
  if not Assigned(Compiled) then
    Compile;
  Compiled.Render(aContext,aOutput);
end;

function TMustache.Render(aContext: TMustacheContext): TMustacheString;

Var
  S : TMustacheStringOutput;

begin
  S:=TMustacheStringOutput.Create;
  try
    Render(aContext,S);
    Result:=S.Data;
  finally
    S.Free;
  end;
end;

function TMustache.Render(const aJSON: TJSONData): TMustacheString;

Var
  C : TMustacheJSONContext;

begin
  C:=TMustacheJSONContext.Create(aJSON,FOnGetValue);
  try
    Result:=Render(C);
  finally
    C.Free;
  end;
end;

function TMustache.Render(const aJSON: TJSONStringType): TMustacheString;

Var
  JSONData : TJSONData;

begin
  JSONData:=GetJSON(aJSON);
  try
    Result:=Render(JSONData);
  finally
    JSONData.Free;
  end;
end;

class function TMustache.CreateMustache(aOwner : TComponent; aTemplate : TMustacheString) : TMustache;

begin
  Result:=TMustache.Create(aOwner);
  Result.Template:=aTemplate;
end;

procedure TMustache.SetStartTag(AValue: TMustacheString);
begin
  if FStartTag=AValue then Exit;
  FStartTag:=AValue;
  Reset;
end;

procedure TMustache.SetPartials(AValue: TStrings);
begin
  if FPartials=AValue then Exit;
  FPartials.Assign(AValue);
end;

procedure TMustache.SetStopTag(AValue: TMustacheString);
begin
  if FStopTag=AValue then Exit;
  FStopTag:=AValue;
  Reset;
end;

procedure TMustache.SetTemplate(AValue: TMustacheString);
begin
  if FTemplate=AValue then Exit;
  FTemplate:=AValue;
  Reset;
end;

procedure TMustache.DoGetPartial(const aName: TMustacheString;
  var aHandled: Boolean; var aValue: TMustacheString);
begin
  aValue:=FPartials.Values[aName];
  aHandled:=aValue<>'';
  if Not aHandled then
    aHandled:=FPartials.IndexOfName(aName)<>-1;
end;

procedure TMustache.Reset;
begin
  FreeAndNil(FCompiled);
  FreeAndNil(FCompiledPartials);
  FCompiledPartials:=CreatePartials;
end;

function TMustache.CreatePartials: TMustachePartialList;
begin
  Result:=TMustachePartialList.Create(metRoot,Nil,0);
end;

class function TMustache.Render(aTemplate: TMustacheString;
  const aJSON: TJSONStringType): TMustacheString;

begin
  With CreateMustache(Nil,aTemplate) do
    try
      Result:=Render(aJSON);
    finally
      Free;
    end;
end;

{ TMustacheJSONContext }

function TMustacheJSONContext.FindValue(const aName: TMustacheString
  ): TJSONData;
Var
  aCount : Integer;
  N : TMustacheString;

begin
  Result:=Nil;
  aCount:=FCount-1;
  While (Result=Nil) and (aCount>=0) do
    begin
    N:=aName;
    if N='.' then
      N:='';
    With FStack[aCount] do
      if (Index>=0) and (Index<Value.Count) then
        Result:=Value.Items[Index].FindPath(N)
      else
        Result:=Value.FindPath(N);
    Dec(aCount);
    end;
end;

function TMustacheJSONContext.GetRootData: TJSONData;
begin
  Result:=FStack[0].Value;
end;


constructor TMustacheJSONContext.Create(aJSON: TJSONData;
  aCallback: TGetTextValueEvent);
begin
  Inherited Create(aCallBack);
  SetLength(FStack,JSONListGrowCount);
  FStack[0].Value:=aJSON;
  FStack[0].Index:=-1;
  FCount:=1;
end;

function TMustacheJSONContext.MoveNextSectionItem(const aName: TMustacheString
  ): Boolean;

begin
  With FStack[FCount-1] do
    begin
    Inc(Index);
    Result:=Index<Value.Count;
    end;
end;

function TMustacheJSONContext.PushSection(const aName: TMustacheString
  ): TMustacheSectionType;

Var
  S : TJSONData;

begin
  Result:=mstNone;
  S:=FindValue(aName);
  if S=Nil then
    Exit;
  if (S.JSONType=jtArray) then
    begin
    if (S.Count>0) then
      Result:=mstList
    end
  else if Not ((S.JSONType=jtNull) or ((S.JSONType=jtBoolean) and Not S.AsBoolean)) then
     Result:=mstSingle;
  if Result<>mstNone then
    begin
    if FCount=Length(FStack) then
      SetLength(FStack,FCount+JSONListGrowCount);
    FStack[FCount].Value:=S;
    FStack[FCount].Index:=-1;
    Inc(FCount,1);
    end;
end;

procedure TMustacheJSONContext.PopSection(const aName: TMustacheString);
begin
  if FCount<1 then
    Raise EMustache.CreateFmt('PopSection %s without push',[aName]);
  Dec(FCount,1);
end;

function TMustacheJSONContext.GetTextValue(const aName: TMustacheString): TMustacheString;

Var
  aJSON : TJSONData;

begin
  Result:='';
  aJSON:=FindValue(aName);
  if not Assigned(aJSON) then
    Result:=Inherited GetTextValue(aName)
  else
    if (AJSON.JSONType=jtNumber) and (TJSONNumber(aJSON).NumberType=ntFloat) then
      Result:=FormatFloat('0.0###########',aJSON.AsFloat)
    else
      Result:=aJSON.AsString;
end;

{ TMustacheSectionElement }

procedure TMustacheSectionElement.Render(aContext: TMustacheContext;
  aOutput: TMustacheOutput; const aPrefix: String; aLast : Boolean = False);

Var
  L : TMustacheSectionType;

begin
   L:=aContext.PushSection(Name);
   if ElementType=metInvertedSection then
     begin
     if L=mstNone then
       inherited Render(aContext, aOutput,aPrefix);
     end
   else
     Case L of
     mstSingle :
        inherited Render(aContext, aOutput);
     mstList :
        while aContext.MoveNextSectionItem(Name) do
          inherited Render(aContext, aOutput,aPrefix);
     end;
  if L<>mstNone then
    aContext.PopSection(Name);
end;

{ TMustacheContext }

constructor TMustacheContext.Create(aCallback: TGetTextValueEvent);
begin
  FCallback:=aCallback;
end;

class function TMustacheContext.QuoteHTML(aString: TMustacheString
  ): TMustacheString;

Const
  QuoteChars = ['<','>','&','"'];

Var
  I,Last,Len : Integer;
  Res : TMustacheString;

  Procedure AddToResult; overload;

  begin
    Res:=Res+Copy(aString,Last,I-Last);
    Last:=I;
  end;

  Procedure AddToResult(aTerm : TMustacheString); overload;

  begin
    Res:=Res+aTerm;
    Last:=Last+1;
  end;

begin
  Res:='';
  Last:=1;
  Len:=Length(Astring);
  I:=1;
  While (I<=Len) do
    begin
    While (I<=Len) and not (aString[i] in QuoteChars) do
      Inc(I);
    AddToResult;
    if I<=Len then
      Case aString[i] of
        '<' : AddToResult('&lt;');
        '>' : AddToResult('&gt;');
        '&' : AddToResult('&amp;');
        '"' : AddToResult('&quot;');
      end;
    Inc(i);
    end;
  AddToResult;
  Result:=Res;
end;

function TMustacheContext.MoveNextSectionItem(const aName: TMustacheString): Boolean;
begin
  Result:=False
end;

function TMustacheContext.PushSection(const aName: TMustacheString): TMustacheSectionType;
begin
  Result:=mstNone;
end;

procedure TMustacheContext.PopSection(const aName: TMustacheString);
begin
  //
end;

function TMustacheContext.GetTextValue(const aName: TMustacheString): TMustacheString;

var
  aHandled : Boolean;

begin
  aHandled:=False;
  Result:='';
  if Assigned(FCallBack) then
    FCallBack(aName,aHandled,Result);
end;

{ TMustacheTextElement }

procedure TMustacheTextElement.SetData(const aData: TMustacheString);
begin
  FData:=aData;
end;

function TMustacheTextElement.GetData: TMustacheString;
begin
  Result:=FData;
end;

procedure TMustacheTextElement.Render(aContext: TMustacheContext;
  aOutput: TMustacheOutput; const aPrefix: String; aLast : Boolean = False);

Var
  S : String;
  L : Integer;

begin
  if (ElementType=metText) then
    begin
    S:=FData;
    L:=Length(S);
    if (aPrefix<>'')  then
      begin
      if (S[L]=#10) and aLast then
        S:=StringReplace(Copy(S,1,L-1),#10,#10+aPrefix,[rfReplaceAll])+#10
      else
        S:=StringReplace(S,#10,#10+aPrefix,[rfReplaceAll]);
{$IFDEF DEBUGMUSTACHE}
      Writeln('Adding prefix =]',aPrefix,'[= to =]',FData, '[=  --->  =]',S,'["');
{$ENDIF}
      end;
    aOutput.Output(S);
    end;
end;

{ TMustacheVariableElement }

procedure TMustacheVariableElement.SetData(const aData: TMustacheString);

Var
  L : Integer;
  N : TMustacheString;
begin
  N:=aData;
  L:=Length(N);
  FNoUnescape:=(L>1) and (N[1]='{') and (N[L]='}');
  if NoUnescape then
    N:=Copy(N,2,L-2)
  else
    begin
    FNoUnescape:=(L>0) and (N[1]='&');
    if NoUnescape then
      N:=Copy(N,2,L-1);
    end;
  inherited SetData(N);
end;

procedure TMustacheVariableElement.Render(aContext: TMustacheContext;
  aOutput: TMustacheOutput; const aPrefix: String; aLast : Boolean = False);

Var
  aValue : TMustacheString;

begin
  aValue:='';
  if Assigned(aContext) then
    begin
    aValue:=aContext.GetTextValue(Name);
    if Not NoUnescape then
      aValue:=aContext.QuoteHTML(aValue);
    end;
  aOutput.Output(aValue);
end;

{ TMustacheParser }

function TMustacheParser.CreateElement(aType: TMustacheElementType; aParent : TMustacheElement; aPosition : Integer): TMustacheElement;

begin
  Result:=DefaultTypes[aType].Create(aType,aParent,aPosition);
  if Assigned(aParent) then
    aParent.AddChild(Result);
end;

constructor TMustacheParser.Create(aTemplate: TMustacheString; aStart: TMustacheString;
  aStop: TMustacheString);
begin
  FStartTag:=aStart;
  FStopTag:=aStop;
  FTemplate:=aTemplate;
  if FStartTag='' then
    FStartTag:='{{';
  if FStopTag='' then
    FStopTag:='}}';
end;

class procedure TMustacheParser.SetDefaultTypeClass(aType: TMustacheElementType;
  aClass: TMustacheElementClass);

begin
  DefaultTypes[aType]:=aClass;
end;

function TMustacheParser.GetPartial(const aName: TMustacheString): TMustacheString;

Var
  Handled : Boolean;
begin
  Result:='';
  Handled:=False;
  if Assigned(FOnGetPartial) then
    FOnGetPartial(aName,Handled,Result);
//  If not Handled then
//    Raise EMustache.CreateFmt(SErrPartialNotFound,[aName]);
end;

procedure TMustacheParser.ExtractStartStop(const aName: TMustacheString; out aStart,
  aStop: TMustacheString);

  Function Invalid(S : TMustacheString) : Boolean;
  begin
    Invalid:=(Length(S)=0) or (Pos('=',S)<>0);
  end;

Var
  DLen,NLen : Integer;
  N : TMustacheString;

begin
  NLen:=Length(aName);
  if aName[NLen]<>'=' then
    Raise EMustache.CreateFmt(SErrInvalidDelimiter,[aName]);
  N:=Copy(aName,1,NLen-1);
  DLen:=(NLen-1) div 2;
  aStart:=Trim(Copy(N,1,DLen));
  aStop:=Trim(Copy(N,NLen-DLen,DLen));
  // Writeln('New: "',aStart,'" - "',aStop,'" - ',DLEn);
  if Invalid(aStop) then
    Raise EMustache.CreateFmt(SErrInvalidDelimiterValue,[SStopTag,aStop,N]);
  if Invalid(aStart) then
    Raise EMustache.CreateFmt(SErrInvalidDelimiterValue,[SStartTag,aStart,N]);
end;

procedure TMustacheParser.Parse(aParent: TMustacheElement);

begin
  DoParse(aParent,FTemplate,StartTag, StopTag);
end;

function TMustacheParser.EndsOnWhiteSpace(aElement: TMustacheElement): Boolean;

Var
  I : Integer;
  S : TMustacheString;

begin
  // if on standalone line, the entire line must be removed, see specs comments.standalone
  Result:=(aElement.ElementType=metText);
  s:=aElement.Data;
  I:=Length(S);
  While Result and (I>0) do
     begin
     if S[i] in [#13,#10] then
       Break;
     Result:=(S[I]=' ');
     Dec(i);
     end;
  Result:=Result and ((I>0) or (aElement.Position=1));
end;

function TMustacheParser.GetEndingWhiteSpace(aElement: TMustacheElement): TMustacheString;

Var
  S : TMustacheString;
  I : Integer;

begin
  s:=aElement.Data;
  I:=Length(S);
  While (I>0) and (S[I]=' ') do
     Dec(i);
  Result:=Copy(S,I+1);
end;

procedure TMustacheParser.TrimEndingWhiteSpace(aElement: TMustacheElement);

Var
  I : Integer;
  S : TMustacheString;

begin
  s:=aElement.Data;
  I:=Length(S);
  While (I>0) and (S[I]=' ') do
     Dec(i);
  aElement.Data:=Copy(S,1,I);
end;

Function TMustacheParser.CreateDefault(aParent : TMustacheElement; aPosition : Integer;Const aName : String) : TMustacheElement;

begin
  Result:=CreateElement(metVariable,aParent,aPosition);
  Result.SetData(aName);
end;

procedure TMustacheParser.DoParse(aParent: TMustacheElement; const aTemplate,
  aStart, aStop: TMustacheString);

Var
  currParent : TMustacheElement;
  aLen,clStop, lStart,lStop, NewPos, Current, Total : Integer;
  aName,cStart,cStop,R : TMustacheString;
  C: TMustacheChar;
  IsWhiteSpace : Boolean;
  Partial,WhiteSpaceEl : TMustacheELement;

  Function CheckWhiteSpace : Boolean;

  begin
    WhiteSpaceEl:=Nil;
    With CurrParent do
      begin
      Result:=(ChildCount=0) or EndsOnWhiteSpace(Children[ChildCount-1]);
      if Result and (ChildCount>0) then
         WhiteSpaceEl:=Children[ChildCount-1];
      end;
  end;

  Procedure FinishWhiteSpace(Full : Boolean = true);
  Var
    I : Integer;
  begin
    I:=NewPos;
    While IsWhiteSpace and (I+clStop<=Total) do
      begin
      C:=aTemplate[I+clStop];
      if (C in [#13,#10]) then
        Break;
      isWhiteSpace:=aTemplate[I+clStop]=' ';
      I:=I+1;
      end;
    if isWhiteSpace then
      begin
      While (I<=Total) and (aTemplate[I+clStop] in [#13,#10]) do
        Inc(I);
      NewPos:=I;
      if Assigned(WhiteSpaceEl) and full then
        TrimEndingWhiteSpace(WhiteSpaceEl);
      end;
  end;

begin
  currParent:=aParent;
  cStart:=aStart;
  cStop:=aStop;
  lStart:=Length(cStart);
  lStop:=Length(cStop);
  Current:=1;
  Total:=Length(aTemplate);
  While (Current<=Total) do
    begin
    C:=Template[Current];
    NewPos:=Pos(cStart,aTemplate,Current);
    if NewPos=0 then
      NewPos:=Total+1;
    // Stash what we have till now.
    if NewPos>Current then
      begin
      R:=Copy(aTemplate,Current,NewPos-Current);
      CreateElement(metText,currParent,Current).SetData(R);
      Current:=NewPos;
      end;
    if Current<Total then
      begin
      NewPos:=Pos(cStop,aTemplate,Current+lStart);
      if (NewPos=0) then
        Raise EMustache.CreateFmt(SErrUnterminatedTag,[cStart,Current]);
      aLen:=NewPos-Current-LStart;
      aName:=Copy(aTemplate,Current+LStart,ALen);
      if (aName='') then
        Raise EMustache.CreateFmt(SErrEmptyTag,[cStart,Current]);
      C:=aName[1];
      if C in ['=','#','^','/','!','>'] then
        aName:=Copy(aName,2,Length(aName)-1);
      clStop:=Lstop; // Can change.
      case C of
        '=' :
          begin
          IsWhiteSpace:=CheckWhiteSpace;
          if IsWhiteSpace then
            FinishWhiteSpace;
          ExtractStartStop(aName,cStart,cStop);
          lStart:=Length(cStart);
          lStop:=Length(cStop);
          //R:=Copy(aTemplate,newPos+clStop);
          //Writeln(R);
          end;
        '{' :
          begin
          if (cStop='}}') then
            begin
            if (FTemplate[NewPos+lStop]<>'}') then
              Raise EMustache.CreateFmt(SErrUnterminatedTag,[cStart,Current]);
            inc(NewPos);
            aName:=aName+'}';
            end;
          CreateElement(metVariable,currParent,Current).SetData(aName);
          end;
        '#' :
          begin
          IsWhiteSpace:=CheckWhiteSpace;
          CurrParent:=CreateElement(metSection,currParent,Current);
          CurrParent.SetData(aName);
          if IsWhiteSpace then
            FinishWhiteSpace;
          end;
        '!' :
          begin
          IsWhiteSpace:=CheckWhiteSpace;
          CreateElement(metComment,currParent,Current).SetData(aName);
          if IsWhiteSpace then
            FinishWhiteSpace;
          end;
        '^' :
          begin
          IsWhiteSpace:=CheckWhiteSpace;
          CurrParent:=CreateElement(metInvertedSection,currParent,Current);
          CurrParent.SetData(aName);
          if IsWhiteSpace then
            FinishWhiteSpace;
          end;
        '>' :
          begin
          // Find or create compiled partial;
          IsWhiteSpace:=CheckWhiteSpace;
          aName:=Trim(aName);
          if not Assigned(Partials) then
            Raise EMustache.Create(SErrNoPartials);
          Partial:=Partials.FindPartial(aName);
          if Partial=Nil then
            begin
            Partial:=CreateElement(metRoot,Partials,Current);
            Partial.Data:=aName;
            DoParse(Partial,GetPartial(aName),FStartTag,FStopTag);
            end;
          // Create reference and insert into current tree
          With CreateElement(metPartial,currParent,Current) do
            begin
            AddChild(Partial);
            Data:=aName;
            if isWhitespace and assigned(WhiteSpaceEl) then
              Prefix:=GetEndingWhiteSpace(WhiteSpaceEl);
            end;
          if IsWhiteSpace then
            FinishWhiteSpace(False);
          end;
        '/' :
          begin
          IsWhiteSpace:=CheckWhiteSpace;
          if Not (CurrParent.ElementType in [metSection,metInvertedSection]) then
            Raise EMustache.CreateFmt(SErrNoSectionToClose,[aName,Current])
          else if (CurrParent.Data<>Trim(aName)) then
            Raise EMustache.CreateFmt(SErrSectionClose,[currParent.Data,CurrParent.Position,aName,Current])
          else
            currParent:=currParent.Parent;
          if IsWhiteSpace then
            FinishWhiteSpace;
          end
      else
        CreateDefault(CurrParent,Current,aName);
      end;
      Current:=NewPos+clStop;
      end;
    end;
  if CurrParent<>aParent then
    Raise EMustache.CreateFmt(SErrNotClosedSection,[currParent.Data,CurrParent.Position])

end;

function TMustacheParser.Parse: TMustacheElement;

begin
  Result:=TMustacheParentElement.Create(metRoot,Nil,1);
  try
    Parse(Result);
  except
    Result.Free;
    Raise;
  end;
end;

{ TMustacheNamedElement }

procedure TMustacheNamedElement.SetData(Const aData: TMustacheString);

begin
  FName:=Trim(aData);
end;

function TMustacheNamedElement.GetData: TMustacheString;
begin
  Result:=FName;
end;

{ TMustacheParentElement }

function TMustacheParentElement.GetElement(aIndex : Integer): TMustacheElement;
begin
  If (aIndex<0) or (aIndex>=FCount) then
    Raise EMustache.CreateFmt(SErrInvalidIndex,[ClassName,aIndex,FCount-1]);
  Result:=FChildren[aIndex];
end;

function TMustacheParentElement.GetCount: Integer;
begin
  Result:=FCount;
end;

destructor TMustacheParentElement.Destroy;
begin
  While FCount>0 do
    begin
    Dec(FCount);
    FreeAndNil(FChildren[FCount]);
    end;
  inherited Destroy;
end;


procedure TMustacheParentElement.AddChild(aChild: TMustacheElement);

Var
  Len : Integer;

begin
  Len:=Length(FChildren);
  if (FCount>=Len) then
    SetLength(FChildren,Len+ListGrowCount);
  FChildren[FCount]:=aChild;
  Inc(FCount);
end;

procedure TMustacheParentElement.Render(aContext: TMustacheContext;
  aOutput: TMustacheOutput; const aPrefix: String; aLast : Boolean = False);

Var
  I : integer;

begin
  For I:=0 to ChildCount-1 do
    Children[I].Render(aContext,aOutPut,aPrefix,I=ChildCount-1);
end;

{ TMustacheElement }

function TMustacheElement.GetCount: Integer;
begin
  Result:=0;
end;

function TMustacheElement.GetElement(aIndex : Integer): TMustacheElement;
begin
  Result:=Nil;
end;

function TMustacheElement.GetPrefix: TMustacheString;
begin
  Result:='';
end;

procedure TMustacheElement.SetPrefix(aValue: TMustacheString);
begin
  //
end;

procedure TMustacheElement.Dump(aList: Tstrings; aIndent: TMustacheString; aDumpChildren : Boolean = true);

Var
  I : Integer;

begin
  aList.Add(aIndent+Format('%s (%s, %d) : "%s"',[ClassName,GetEnumName(TypeInfo(TMustacheElementType),Ord(ElementType)),Position,Data]));
  if aDumpChildren then
    For I:=0 to ChildCount-1 do
      Children[I].Dump(aList,'  '+aIndent);
end;

constructor TMustacheElement.Create(aType : TMustacheElementType; aParent : TMustacheElement;aPosition: Integer);
begin
  FType:=aType;
  FParent:=aParent;
  FPosition:=aPosition;
end;

procedure TMustacheElement.AddChild(aChild: TMustacheElement);
begin
  Raise EMustache.CreateFmt(SErrNoChildForElement,[ClassName])
end;

{ TMustacheStringOutput }

procedure TMustacheStringOutput.Output(const aText: TMustacheString);
begin
  FData:=FData+aText;
{$IFDEF DEBUGMUSTACHE}
  Writeln('--');
  Writeln('Output -]',aText,'[-');
  Writeln('--');
{$ENDIF}
end;

procedure TMustacheStringOutput.Reset;
begin
  FData:='';
end;

begin
  TMustacheParser.SetDefaultTypeClass(metRoot,TMustacheParentElement);
  TMustacheParser.SetDefaultTypeClass(metComment,TMustacheTextElement);
  TMustacheParser.SetDefaultTypeClass(metText,TMustacheTextElement);
  TMustacheParser.SetDefaultTypeClass(metVariable,TMustacheVariableElement);
  TMustacheParser.SetDefaultTypeClass(metSection,TMustacheSectionElement);
  TMustacheParser.SetDefaultTypeClass(metInvertedSection,TMustacheSectionElement);
  TMustacheParser.SetDefaultTypeClass(metPartial,TMustachePartialElement);
end.

