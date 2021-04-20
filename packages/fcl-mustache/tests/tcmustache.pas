{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2021 by Michael Van Canneyt (michael@freepascal.org)

    Test cases for basic mustache parser support

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit tcmustache;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, fpmustache, tcbasemustache;

type

  { TTestMustacheParser }


  TTestMustacheParser= class(TBaseMustacheTest)
  private
  protected
    Function CreateParser : TMustacheParser; override;
  Public
    procedure SetUp; override;
    procedure TearDown; override;
  published
    procedure TestEmpty;
    procedure TestText;
    procedure TestVariable;
    procedure TestVariableErrNonClosed;
    procedure TestVariableAlternateStartStop;
    procedure TestDottedVariable;
    procedure TestVariableNoUnescape;
    procedure TestVariableNoUnescapeErrNonClosed;
    procedure TestVariableNoUnescapeAlternateStartStop;
    procedure TestComment;
    procedure TestCommentSurround;
    procedure TestCommentStandalone;
    procedure TestCommentStandaloneSpaced;
    procedure TestSetDelimiter;
    procedure TestSetDelimiterErrInvalid;
    procedure TestSection;
    procedure TestSectionNested;
    procedure TestSectionErrNotClosed;
    procedure TestSectionErrWrongClosed;
    procedure TestSectionErrNotStarted;
    procedure TestTextSection;
    procedure TestPartial;
  end;

  { TTestMustacheOutput }

  TTestMustacheOutput = class(TTestCase)
  Published
    Procedure TestStringOutput;
  end;

  { TTestMustacheElement }

  TTestMustacheElement = class(TTestCase)
  private
    FContext: TTestContext;
    FEl: TMustacheElement;
    Foutput: TMustacheStringOutput;
    procedure DoCallBack(const aName: TMustacheString; var aHandled: Boolean;
      var aValue: TMustacheString);
  Public
    Procedure SetUp; override;
    Procedure TearDown; override;
    Property Context : TTestContext Read FContext;
    Property Output : TMustacheStringOutput Read Foutput;
    Property El : TMustacheElement Read FEl;
  Published
    Procedure TestEmpty;
    Procedure TestTextElement;
    Procedure TestTextElementNoEscape;
    Procedure TestTextElementComment;
    Procedure TestTextElementPrefix;
    procedure TestTextElementPrefixNotLast;
    procedure TestTextElementPrefixLast;
    Procedure TestVariableElement;
    Procedure TestVariableElementNoEscape;
    Procedure TestVariableElementEscape;
    Procedure TestSectionEmpty;
    Procedure TestSectionValue;
    Procedure TestSectionValueFalse;
    Procedure TestSectionValueNull;
    Procedure TestSectionValueEmptyArray;
    Procedure TestSectionValueArray1El;
    Procedure TestSectionValueArray2El;
    Procedure TestSectionValueArray2ElValue;
    Procedure TestSectionValueArray1ElValueSuper;
    Procedure TestSectionValueArray2ElValueSuper;
    Procedure TestParentElement;
    Procedure TestParentElementRender;
    Procedure TestParentElementRenderPrefix;
  end;

implementation

uses Typinfo;

Const
  SNeedsQuoting = '< > & "';
  SQuotedResult = '&lt; &gt; &amp; &quot;';


{ TTestMustacheElement }

procedure TTestMustacheElement.DoCallBack(const aName: TMustacheString;
  var aHandled: Boolean; var aValue: TMustacheString);
begin
  aValue:='';
end;

procedure TTestMustacheElement.SetUp;
begin
  inherited SetUp;
  FOutput:=TMustacheStringOutput.Create;
  FContext:=TTestContext.Create(@DoCallBack);
end;

procedure TTestMustacheElement.TearDown;
begin
  FreeAndNil(FContext);
  FreeAndNil(FOutput);
  FreeAndNil(FEl);
  inherited TearDown;
end;

procedure TTestMustacheElement.TestEmpty;
begin
  AssertNotNull('Have output',Output);
end;

procedure TTestMustacheElement.TestTextElement;

begin
  Fel:=TMustacheTextElement.Create(metText,Nil,0);
  El.Render(Nil,Output);
  AssertEquals('No output','',Output.Data);
  El.Data:='me';
  El.Render(Nil,Output);
  AssertEquals('Correct output','me',Output.Data);
end;

procedure TTestMustacheElement.TestTextElementNoEscape;
begin
  Fel:=TMustacheTextElement.Create(metText,Nil,0);
  El.Data:=SNeedsQuoting;
  El.Render(Nil,Output);
  AssertEquals('Correct output',SNeedsQuoting,Output.Data);
end;

procedure TTestMustacheElement.TestTextElementComment;
begin
  Fel:=TMustacheTextElement.Create(metComment,Nil,0);
  El.Data:='Something';
  El.Render(Nil,Output);
  AssertEquals('Correct output','',Output.Data);
end;

procedure TTestMustacheElement.TestTextElementPrefix;
begin
  Fel:=TMustacheTextElement.Create(metText,Nil,0);
  El.Data:='me'#10'you';
  El.Render(Nil,Output,'  ');
  AssertEquals('Correct output 1','me'#10'  you',Output.Data);
end;

procedure TTestMustacheElement.TestTextElementPrefixNotLast;
begin
  Fel:=TMustacheTextElement.Create(metText,Nil,0);
  El.Data:='me'#10'you'#10;
  El.Render(Nil,Output,'  ');
  AssertEquals('Correct output 2','me'#10'  you'#10'  ',Output.Data);
end;

procedure TTestMustacheElement.TestTextElementPrefixLast;

begin
  Fel:=TMustacheTextElement.Create(metText,Nil,0);
  El.Data:='me'#10'you'#10;
  El.Render(Nil,Output,'  ',True);
  AssertEquals('Correct output 2','me'#10'  you'#10,Output.Data);
end;


procedure TTestMustacheElement.TestVariableElement;
begin
  Fel:=TMustacheVariableElement.Create(metText,Nil,0);
  Context.Values.Values['name']:='abc';
  El.Data:='name';
  El.Render(Context,Output);
  AssertEquals('Correct output','abc',Output.Data);
end;


procedure TTestMustacheElement.TestVariableElementNoEscape;
begin
  Fel:=TMustacheVariableElement.Create(metText,Nil,0);
  Context.Values.Values['name']:=SNeedsQuoting;
  El.Data:='{name}';
  El.Render(Context,Output);
  AssertEquals('Correct output',SNeedsQuoting,Output.Data);
end;

procedure TTestMustacheElement.TestVariableElementEscape;
begin
  Fel:=TMustacheVariableElement.Create(metText,Nil,0);
  Context.Values.Values['name']:=SNeedsQuoting;
  El.Data:='name';
  El.Render(Context,Output);
  AssertEquals('Correct output',SQuotedResult,Output.Data);
end;

procedure TTestMustacheElement.TestSectionEmpty;

Var
  T : TMustacheTextElement;

begin
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheTextElement.Create(metText,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('No output','',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValue;
Var
  T : TMustacheTextElement;

begin
  Context.SetValue('s','b');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheTextElement.Create(metText,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('Single pass','a',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValueFalse;
Var
  T : TMustacheTextElement;

begin
  Context.SetValue('s','<false>');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheTextElement.Create(metText,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('no pass','',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValueNull;

Var
  T : TMustacheTextElement;

begin
  Context.SetValue('s','<null>');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheTextElement.Create(metText,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('no pass','',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValueEmptyArray;
Var
  T : TMustacheTextElement;

begin
  Context.SetValue('s','[]');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheTextElement.Create(metText,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('no pass','',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValueArray1El;
Var
  T : TMustacheTextElement;

begin
  Context.SetValue('s','[]');
  Context.SetValue('s[0]','toto');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheTextElement.Create(metText,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('Single pass','a',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValueArray2El;

Var
  T : TMustacheTextElement;

begin
  Context.SetValue('s','[]');
  Context.SetValue('s[0]','toto');
  Context.SetValue('s[1]','tata');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheTextElement.Create(metText,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('Double pass','aa',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValueArray2ElValue;

Var
  T : TMustacheElement;

begin
  Context.SetValue('s','[]');
  Context.SetValue('s[0]','{}');
  Context.SetValue('s[0].a','1');
  Context.SetValue('s[1]','{}');
  Context.SetValue('s[1].a','2');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheVariableElement.Create(metVariable,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('Double pass','12',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValueArray1ElValueSuper;

Var
  T : TMustacheElement;

begin
  Context.SetValue('s','[]');
  Context.SetValue('s[0]','{}');
  Context.SetValue('s[0].b','1');
  Context.SetValue('a','2');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheVariableElement.Create(metVariable,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  Fel.Render(Context,Output);
  AssertEquals('Single pass','2',Output.Data);
end;

procedure TTestMustacheElement.TestSectionValueArray2ElValueSuper;
Var
  T : TMustacheElement;

begin
  Context.SetValue('s','[]');
  Context.SetValue('s[0]','{}');
  Context.SetValue('s[0].b','1');
  Context.SetValue('s[1]','{}');
  Context.SetValue('s[1].b','2');
  Context.SetValue('a','.a.');
  Fel:=TMustacheSectionElement.Create(metSection,Nil,0);
  Fel.Data:='s';
  T:=TMustacheVariableElement.Create(metVariable,Nil,0);
  Fel.AddChild(T);
  T.Data:='a';
  T:=TMustacheVariableElement.Create(metVariable,Nil,0);
  Fel.AddChild(T);
  T.Data:='b';
  Fel.Render(Context,Output);
  AssertEquals('Single pass','.a.1.a.2',Output.Data);
end;

procedure TTestMustacheElement.TestParentElement;

Var
  SEl : TMustacheElement;

begin
  Fel:=TMustacheParentElement.Create(metSection,Nil,0);
  Sel:=TMustacheTextElement.Create(metText,Fel,0);
  AssertSame('Parent stored',Fel,Sel.Parent);
  AssertEquals('Not added to parent',0,FEl.ChildCount);
  Fel.AddChild(Sel);
  AssertEquals('added to parent - count',1,FEl.ChildCount);
  AssertSame('added to parent - stored',Sel,FEl.Children[0]);
end;

procedure TTestMustacheElement.TestParentElementRender;

Var
  SEl : TMustacheElement;

begin
  Fel:=TMustacheParentElement.Create(metSection,Nil,0);
  Sel:=TMustacheTextElement.Create(metText,Fel,0);
  Sel.Data:='a';
  Fel.AddChild(Sel);
  Sel:=TMustacheTextElement.Create(metText,Fel,0);
  Sel.Data:='b';
  Fel.AddChild(Sel);
  Sel:=TMustacheTextElement.Create(metText,Fel,0);
  Sel.Data:='c';
  Fel.AddChild(Sel);
  Fel.Render(Context,Output);
  AssertEquals('Correct output','abc',Output.Data);
end;

procedure TTestMustacheElement.TestParentElementRenderPrefix;
Var
  SEl : TMustacheElement;

begin
  Fel:=TMustacheParentElement.Create(metSection,Nil,0);
  Sel:=TMustacheTextElement.Create(metText,Fel,0);
  Sel.Data:='a'#10'b';
  Fel.AddChild(Sel);
  Sel:=TMustacheTextElement.Create(metText,Fel,0);
  Sel.Data:='d'#10'e';
  Fel.AddChild(Sel);
  Sel:=TMustacheTextElement.Create(metText,Fel,0);
  Sel.Data:='f'#10;
  Fel.AddChild(Sel);
  Fel.Render(Context,Output,'  ');
  AssertEquals('Correct output','a'#10'  bd'#10'  ef'#10,Output.Data);
end;

{ TTestMustacheOutput }

procedure TTestMustacheOutput.TestStringOutput;

Var
  SO : TMustacheStringOutput;

begin
  SO:=TMustacheStringOutput.Create;
  try
     AssertEquals('Empty start','',SO.Data);
     SO.Output('abc');
     AssertEquals('Output 1','abc',SO.Data);
     SO.Output('def');
     AssertEquals('Output 2','abcdef',SO.Data);
  finally
    SO.Free;
  end;

end;

function TTestMustacheParser.CreateParser: TMustacheParser;
begin
  Result:=TMustacheParser.Create;
end;

procedure TTestMustacheParser.SetUp;
begin
  inherited SetUp;
end;

procedure TTestMustacheParser.TearDown;
begin
  inherited TearDown;
end;

procedure TTestMustacheParser.TestEmpty;

begin
  AssertNotNull('Have parser',Parser);
  AssertNull('Have no result',ParseResult);
  AssertEquals('Have no template','',Template);
end;


procedure TTestMustacheParser.TestText;

begin
  Template:='a simple text';
  CallParser;
  AssertResultCount(1);
  AssertElement(0,metText,'a simple text');
end;

procedure TTestMustacheParser.TestVariable;

Var
  el : TMustacheVariableElement;

begin
  Template:='{{a}}';
  CallParser;
  AssertResultCount(1);
  el:=AssertElement(0,metVariable,'a',TMustacheVariableElement) as TMustacheVariableElement;
  AssertFalse('unescape',El.NoUnescape);
end;

procedure TTestMustacheParser.TestVariableErrNonClosed;

begin
  Template:='{{a';
  AssertException('Have error',EMustache,@CallParser,'Tag {{ opened on position 1 but not closed.');
  Template:='{{a}';
  AssertException('Have error',EMustache,@CallParser,'Tag {{ opened on position 1 but not closed.');
end;

procedure TTestMustacheParser.TestVariableAlternateStartStop;
Var
  el : TMustacheVariableElement;

begin
  Parser.StartTag:='<<';
  Parser.StopTag:='>>';
  Template:='<<a>>';
  CallParser;
  AssertResultCount(1);
  el:=AssertElement(0,metVariable,'a',TMustacheVariableElement) as TMustacheVariableElement;
  AssertFalse('unescape',El.NoUnescape);
end;

procedure TTestMustacheParser.TestDottedVariable;
begin
  Template:='{{a.b}}';
  CallParser;
  AssertResultCount(1);
  AssertElement(0,metVariable,'a.b');
end;

procedure TTestMustacheParser.TestVariableNoUnescape;
Var
  el : TMustacheVariableElement;

begin
  Template:='{{{a}}}';
  CallParser;
  AssertResultCount(1);
  el:=AssertElement(0,metVariable,'a',TMustacheVariableElement) as TMustacheVariableElement;
  AssertTrue('unescape',El.NoUnescape);
end;

procedure TTestMustacheParser.TestVariableNoUnescapeErrNonClosed;
begin
  Template:='{{{a';
  AssertException('Have error',EMustache,@CallParser,'Tag {{ opened on position 1 but not closed.');
  Template:='{{{a}';
  AssertException('Have error',EMustache,@CallParser,'Tag {{ opened on position 1 but not closed.');
  Template:='{{{a}}';
  AssertException('Have error',EMustache,@CallParser,'Tag {{ opened on position 1 but not closed.');
end;

procedure TTestMustacheParser.TestVariableNoUnescapeAlternateStartStop;

Var
  el : TMustacheVariableElement;

begin
  Parser.StartTag:='<<';
  Parser.StopTag:='>>';
  Template:='<<{a}>>';
  CallParser;
  AssertResultCount(1);
  el:=AssertElement(0,metVariable,'a',TMustacheVariableElement) as TMustacheVariableElement;
  AssertTrue('unescape',El.NoUnescape);
end;

procedure TTestMustacheParser.TestComment;

begin
  Parser.StartTag:='<<';
  Parser.StopTag:='>>';
  Template:='<<! a comment>>';
  CallParser;
  AssertResultCount(1);
  AssertElement(0,metComment,' a comment',TMustacheTextElement);
end;

procedure TTestMustacheParser.TestCommentSurround;
begin
  Template:='ab{{! a comment}}cd';
  CallParser;
  AssertResultCount(3);
  AssertElement(0,metText,'ab',TMustacheTextElement);
  AssertElement(1,metComment,' a comment',TMustacheTextElement);
  AssertElement(2,metText,'cd',TMustacheTextElement);
end;

procedure TTestMustacheParser.TestCommentStandalone;
begin
  Template:='a'+sLineBreak+'{{! a comment}}'+sLineBreak+'b';
  CallParser;
  AssertResultCount(3);
  AssertElement(0,metText,'a'+sLineBreak,TMustacheTextElement);
  AssertElement(1,metComment,' a comment',TMustacheTextElement);
  AssertElement(2,metText,'b',TMustacheTextElement);
end;

procedure TTestMustacheParser.TestCommentStandaloneSpaced;
begin
  Template:='a'+sLineBreak+'  {{! a comment}}  '+sLineBreak+'b';
  CallParser;
  AssertResultCount(3);
  AssertElement(0,metText,'a'+sLineBreak,TMustacheTextElement);
  AssertElement(1,metComment,' a comment',TMustacheTextElement);
  AssertElement(2,metText,'b',TMustacheTextElement);
end;

procedure TTestMustacheParser.TestSetDelimiter;

begin
  Template:='{{=<< >>=}}<<! a comment>>';
  CallParser;
  AssertResultCount(1);
  AssertElement(0,metComment,' a comment',TMustacheTextElement);
end;

procedure TTestMustacheParser.TestSetDelimiterErrInvalid;
begin
  Template:='{{=== ===}}';
  AssertException('Have error',EMustache,@CallParser,'Invalid set delimiter Stop value: == in "== =="');
end;

procedure TTestMustacheParser.TestSection;

Var
  el : TMustacheSectionElement;

begin
  Template:='{{#a}}{{/a}}';
  CallParser;
  AssertResultCount(1);
  el:=AssertElement(0,metSection,'a',TMustacheSectionElement) as TMustacheSectionElement;
  AssertEquals('No elements in section',0,el.ChildCount);
end;

procedure TTestMustacheParser.TestSectionNested;

Var
  el : TMustacheSectionElement;

begin
  Template:='{{#a}}{{#b}}{{/b}}{{/a}}';
  CallParser;
  AssertResultCount(1);
  el:=AssertElement(0,metSection,'a',TMustacheSectionElement) as TMustacheSectionElement;
  AssertEquals('elements in section',1,el.ChildCount);
  el:=AssertElement(el,0,metSection,'b',TMustacheSectionElement) as TMustacheSectionElement;
  AssertEquals('elements in section sub',0,el.ChildCount);
end;

procedure TTestMustacheParser.TestSectionErrNotClosed;

begin
  Template:='{{#a}}';
  AssertException('Have error',EMustache,@CallParser,'Structural error: Section "a" on position 1 is not closed.');
end;

procedure TTestMustacheParser.TestSectionErrWrongClosed;
begin
  Template:='{{#a}}{{#b}}{{/a}}{{/b}}';
  AssertException('Have error',EMustache,@CallParser,'Structural error: Section "b" on position 7 is closed by tag "a" on position 13.');
end;

procedure TTestMustacheParser.TestSectionErrNotStarted;
begin
  Template:='{{/a}}';
  AssertException('Have error',EMustache,@CallParser,'Structural error: Section "a" on position 1 was never opened.');
end;

procedure TTestMustacheParser.TestTextSection;

Var
  el : TMustacheSectionElement;

begin
  Template:='{{#a}}bbb{{/a}}';
  CallParser;
  AssertResultCount(1);
  el:=AssertElement(0,metSection,'a',TMustacheSectionElement) as TMustacheSectionElement;
  AssertEquals('No elements in section',1,el.ChildCount);
  AssertElement(el,0,metText,'bbb');
end;

procedure TTestMustacheParser.TestPartial;

Var
  el : TMustachePartialElement;

begin
  AddPartial('part','bcd');
  Template:='a{{>part}}e';
  CallParser;
  AssertResultCount(3);
  AssertElement(0,metText,'a',TMustacheTextElement);
  el:=AssertElement(1,metPartial,'part',TMustachePartialElement) as TMustachePartialElement;
  AssertElement(2,metText,'e',TMustacheTextElement);
  AssertEquals('Correct partial','part',El.Partial.Data);
  AssertEquals('Correct partial',1,El.Partial.ChildCount);
  AssertElement(el.Partial,0,metText,'bcd',TMustacheTextElement);
end;


initialization
  RegisterTests([TTestMustacheParser,TTestMustacheOutput,TTestMustacheElement]);
end.

