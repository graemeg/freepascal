{
    This file is part of the Free Component Library

    JSON FPCUNit test for parser
    Copyright (c) 2007 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit testjsonparser;

interface

uses
  Classes, SysUtils, fpcunit, testutils, testregistry,fpjson,
<<<<<<< HEAD
<<<<<<< HEAD
  jsonscanner,jsonParser,testjsondata;
=======
  jsonParser,testjsondata;
>>>>>>> graemeg/fixes_2_2
=======
  jsonParser,testjsondata;
>>>>>>> origin/fixes_2_2

type

  { TTestParser }

<<<<<<< HEAD
<<<<<<< HEAD
  TTestParser = class(TTestJSON)
  private
    procedure CallNoHandlerStream;
=======
  TTestParser= class(TTestJSON)
  private
>>>>>>> graemeg/fixes_2_2
=======
  TTestParser= class(TTestJSON)
  private
>>>>>>> origin/fixes_2_2
    procedure DoTestError(S: String);
    procedure DoTestFloat(F: TJSONFloat); overload;
    procedure DoTestFloat(F: TJSONFloat; S: String); overload;
    procedure DoTestObject(S: String; const ElNames: array of String; DoJSONTest : Boolean = True);
    procedure DoTestString(S : String);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure DoTestArray(S: String; ACount: Integer; HaveComments : Boolean=False);
    Procedure DoTestClass(S : String; AClass : TJSONDataClass);
    procedure CallNoHandler;
=======
    procedure DoTestArray(S: String; ACount: Integer);
>>>>>>> graemeg/fixes_2_2
=======
    procedure DoTestArray(S: String; ACount: Integer);
>>>>>>> origin/fixes_2_2
  published
    procedure TestEmpty;
    procedure TestNull;
    procedure TestTrue;
    procedure TestFalse;
    procedure TestFloat;
    procedure TestInteger;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure TestInt64;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure TestString;
    procedure TestArray;
    procedure TestObject;
    procedure TestMixed;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure TestComment;
    procedure TestErrors;
    Procedure TestClasses;
    Procedure TestHandler;
    Procedure TestNoHandlerError;
    Procedure TestHandlerResult;
    Procedure TestHandlerResultStream;
=======
    procedure TestErrors;
>>>>>>> graemeg/fixes_2_2
=======
    procedure TestErrors;
>>>>>>> origin/fixes_2_2
  end;

implementation

procedure TTestParser.TestEmpty;

Var
  P : TJSONParser;
  J : TJSONData;
  
begin
  P:=TJSONParser.Create('');
  Try
    J:=P.Parse;
    If (J<>Nil) then
      Fail('Empty returns Nil');
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;

procedure TTestParser.TestInteger;

Var
  P : TJSONParser;
  J : TJSONData;

begin
  P:=TJSONParser.Create('1');
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of 1 fails');
    TestJSONType(J,jtNumber);
    TestAsInteger(J,1);
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TTestParser.TestInt64;

Var
  P : TJSONParser;
  J : TJSONData;

begin
  P:=TJSONParser.Create('123456789012345');
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of 123456789012345 fails');
    TestJSONType(J,jtNumber);
    TestAsInt64(J,123456789012345);
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TTestParser.TestNull;

Var
  P : TJSONParser;
  J : TJSONData;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  P:=TJSONParser.Create('null');
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of null fails');
=======
=======
>>>>>>> origin/fixes_2_2
  P:=TJSONParser.Create('Null');
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of Null fails');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    TestJSONType(J,jtNull);
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;

procedure TTestParser.TestTrue;

Var
  P : TJSONParser;
  J : TJSONData;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  P:=TJSONParser.Create('true');
=======
  P:=TJSONParser.Create('True');
>>>>>>> graemeg/fixes_2_2
=======
  P:=TJSONParser.Create('True');
>>>>>>> origin/fixes_2_2
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of True fails');
    TestJSONType(J,jtBoolean);
    TestAsBoolean(J,True);
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;

procedure TTestParser.TestFalse;

Var
  P : TJSONParser;
  J : TJSONData;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  P:=TJSONParser.Create('false');
=======
  P:=TJSONParser.Create('False');
>>>>>>> graemeg/fixes_2_2
=======
  P:=TJSONParser.Create('False');
>>>>>>> origin/fixes_2_2
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of False fails');
    TestJSONType(J,jtBoolean);
    TestAsBoolean(J,False);
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;

procedure TTestParser.TestFloat;


begin
  DoTestFloat(1.2);
  DoTestFloat(-1.2);
  DoTestFloat(0);
  DoTestFloat(1.2e1);
  DoTestFloat(-1.2e1);
  DoTestFloat(0);
  DoTestFloat(1.2,'1.2');
  DoTestFloat(-1.2,'-1.2');
  DoTestFloat(0,'0.0');
end;

procedure TTestParser.TestString;

begin
  DoTestString('A string');
  DoTestString('');
  DoTestString('\"');
end;


procedure TTestParser.TestArray;

Var
  S1,S2,S3 : String;


begin
  DoTestArray('[]',0);
<<<<<<< HEAD
<<<<<<< HEAD
  DoTestArray('[null]',1);
  DoTestArray('[true]',1);
  DoTestArray('[false]',1);
  DoTestArray('[1]',1);
  DoTestArray('[1, 2]',2);
  DoTestArray('[1, 2, 3]',3);
  DoTestArray('[1234567890123456]',1);
  DoTestArray('[1234567890123456, 2234567890123456]',2);
  DoTestArray('[1234567890123456, 2234567890123456, 3234567890123456]',3);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Str(12/10,S1);
  Delete(S1,1,1);
  Str(34/10,S2);
  Delete(S2,1,1);
  Str(34/10,S3);
  Delete(S3,1,1);
=======
  Str(Double(1.2),S1);
  Str(Double(2.3),S2);
  Str(Double(3.4),S3);
>>>>>>> graemeg/cpstrnew
=======
  Str(Double(1.2),S1);
  Str(Double(2.3),S2);
  Str(Double(3.4),S3);
>>>>>>> graemeg/cpstrnew
=======
  Str(Double(1.2),S1);
  Str(Double(2.3),S2);
  Str(Double(3.4),S3);
>>>>>>> graemeg/cpstrnew
=======
  Str(Double(1.2),S1);
  Str(Double(2.3),S2);
  Str(Double(3.4),S3);
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  DoTestArray('[Null]',1);
  DoTestArray('[True]',1);
  DoTestArray('[False]',1);
  DoTestArray('[1]',1);
  DoTestArray('[1, 2]',2);
  DoTestArray('[1, 2, 3]',3);
  Str(1.2,S1);
  Str(2.3,S2);
  Str(3.4,S3);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  DoTestArray('['+S1+']',1);
  DoTestArray('['+S1+', '+S2+']',2);
  DoTestArray('['+S1+', '+S2+', '+S3+']',3);
  DoTestArray('["A string"]',1);
  DoTestArray('["A string", "Another string"]',2);
  DoTestArray('["A string", "Another string", "Yet another string"]',3);
<<<<<<< HEAD
<<<<<<< HEAD
  DoTestArray('[null, false]',2);
  DoTestArray('[true, false]',2);
  DoTestArray('[null, 1]',2);
=======
  DoTestArray('[Null, False]',2);
  DoTestArray('[True, False]',2);
  DoTestArray('[Null, 1]',2);
>>>>>>> graemeg/fixes_2_2
=======
  DoTestArray('[Null, False]',2);
  DoTestArray('[True, False]',2);
  DoTestArray('[Null, 1]',2);
>>>>>>> origin/fixes_2_2
  DoTestArray('[1, "A string"]',2);
  DoTestArray('[1, []]',2);
  DoTestArray('[1, [1, 2]]',2);
end;

procedure TTestParser.TestMixed;

Const

  SAddr ='{ "addressbook": { "name": "Mary Lebow", '+
         '  "address": {'+
         '      "street": "5 Main Street",'+LineEnding+
         '        "city": "San Diego, CA",'+LineEnding+
         '        "zip": 91912,'+LineEnding+
         '    },'+LineEnding+
         '    "phoneNumbers": [  '+LineEnding+
         '        "619 332-3452",'+LineEnding+
         '        "664 223-4667"'+LineEnding+
         '    ]'+LineEnding+
         ' }'+LineEnding+
         '}';

begin
  DoTestArray('[1, {}]',2);
  DoTestArray('[1, { "a" : 1 }]',2);
  DoTestArray('[1, { "a" : 1 }, 1]',3);
  DoTestObject('{ "a" : [1, 2] }',['a']);
  DoTestObject('{ "a" : [1, 2], "B" : { "c" : "d" } }',['a','B']);
  DoTestObject(SAddr,['addressbook'],False);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TTestParser.TestComment;
begin
  DoTestArray('/* */ [1, {}]',2,True);
  DoTestArray('//'+sLineBreak+'[1, { "a" : 1 }]',2,True);
  DoTestArray('/* '+sLineBreak+' */ [1, {}]',2,True);
  DoTestArray('/*'+sLineBreak+'*/ [1, {}]',2,True);
  DoTestArray('/*'+sLineBreak+'*/ [1, {}]',2,True);
  DoTestArray('/*'+sLineBreak+'*'+sLineBreak+'*/ [1, {}]',2,True);
  DoTestArray('/**'+sLineBreak+'**'+sLineBreak+'**/ [1, {}]',2,True);
  DoTestArray('/* */ [1, {}]',2,True);
  DoTestArray('[1, { "a" : 1 }]//'+sLineBreak,2,True);
  DoTestArray('[1, {}]/* '+sLineBreak+' */ ',2,True);
  DoTestArray('[1, {}]/*'+sLineBreak+'*/ ',2,True);
  DoTestArray('[1, {}]/*'+sLineBreak+'*/ ',2,True);
  DoTestArray('[1, {}]/*'+sLineBreak+'*'+sLineBreak+'*/ ',2,True);
  DoTestArray(' [1, {}]/**'+sLineBreak+'**'+sLineBreak+'**/',2,True);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TTestParser.TestObject;
begin
  DoTestObject('{}',[]);
  DoTestObject('{ "a" : 1 }',['a']);
  DoTestObject('{ "a" : 1, "B" : "String" }',['a','B']);
  DoTestObject('{ "a" : 1, "B" : {} }',['a','B']);
  DoTestObject('{ "a" : 1, "B" : { "c" : "d" } }',['a','B']);
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TTestParser.DoTestObject(S: String; const ElNames: array of String;
  DoJSONTest: Boolean);
=======
procedure TTestParser.DoTestObject(S : String; Const ElNames : Array of String; DoJSONTest : Boolean = True);
>>>>>>> graemeg/fixes_2_2
=======
procedure TTestParser.DoTestObject(S : String; Const ElNames : Array of String; DoJSONTest : Boolean = True);
>>>>>>> origin/fixes_2_2

Var
  P : TJSONParser;
  J : TJSONData;
  O : TJSONObject;
  I : Integer;

begin
  P:=TJSONParser.Create(S);
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of object "'+S+'" fails');
    TestJSONType(J,jtObject);
    TestItemCount(J,High(ElNames)-Low(ElNames)+1);
    O:=TJSONObject(J);
    For I:=Low(ElNames) to High(ElNames) do
      AssertEquals(Format('Element %d name',[I-Low(Elnames)])
                   ,ElNames[i], O.Names[I-Low(ElNames)]);
    If DoJSONTest then
      self.TestJSON(J,S);
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TTestParser.DoTestArray(S : String; ACount : Integer; HaveComments : Boolean = False);
=======
procedure TTestParser.DoTestArray(S : String; ACount : Integer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TTestParser.DoTestArray(S : String; ACount : Integer);
>>>>>>> origin/fixes_2_2

Var
  P : TJSONParser;
  J : TJSONData;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  P:=TJSONParser.Create(S,[joComments]);
=======
  P:=TJSONParser.Create(S);
>>>>>>> graemeg/fixes_2_2
=======
  P:=TJSONParser.Create(S);
>>>>>>> origin/fixes_2_2
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of array "'+S+'" fails');
    TestJSONType(J,jtArray);
    TestItemCount(J,ACount);
<<<<<<< HEAD
<<<<<<< HEAD
    if not HaveComments then
      TestJSON(J,S);
=======
    TestJSON(J,S);
>>>>>>> graemeg/fixes_2_2
=======
    TestJSON(J,S);
>>>>>>> origin/fixes_2_2
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TTestParser.DoTestClass(S: String; AClass: TJSONDataClass);

Var
  P : TJSONParser;
  D : TJSONData;

begin
  P:=TJSONParser.Create(S);
  try
    D:=P.Parse;
    try
      AssertEquals('Correct class for '+S+' : ',AClass,D.ClassType);
    finally
      D.Free
    end;
  finally
    P.Free;
  end;
end;

procedure TTestParser.TestErrors;

begin
{
  DoTestError('a');
  DoTestError('"b');
  DoTestError('1Tru');
}
=======
procedure TTestParser.TestErrors;

begin
  DoTestError('a');
  DoTestError('"b');
  DoTestError('1Tru');
>>>>>>> graemeg/fixes_2_2
=======
procedure TTestParser.TestErrors;

begin
  DoTestError('a');
  DoTestError('"b');
  DoTestError('1Tru');
>>>>>>> origin/fixes_2_2
  DoTestError('b"');
  DoTestError('{"a" : }');
  DoTestError('{"a" : ""');
  DoTestError('{"a : ""');
<<<<<<< HEAD
<<<<<<< HEAD
{
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  DoTestError('[1,]');
  DoTestError('[,]');
  DoTestError('[,,]');
  DoTestError('[1,,]');
<<<<<<< HEAD
<<<<<<< HEAD
}
end;

procedure TTestParser.TestClasses;
begin
  SetMyInstanceTypes;
  DoTestClass('null',TMyNull);
  DoTestClass('true',TMyBoolean);
  DoTestClass('1',TMyInteger);
  DoTestClass('1.2',TMyFloat);
  DoTestClass('123456789012345',TMyInt64);
  DoTestClass('"tata"',TMyString);
  DoTestClass('{}',TMyObject);
  DoTestClass('[]',TMyArray);
end;

procedure TTestParser.CallNoHandler;

begin
  GetJSON('1',True).Free;
end;

procedure TTestParser.CallNoHandlerStream;

Var
  S : TStringStream;

begin
  S:=TstringStream.Create('1');
  try
    GetJSON(S,True).Free;
  finally
    S.Free;
  end;
end;

procedure TTestParser.TestHandler;
begin
  AssertNotNull('Handler installed',GetJSONParserHandler);
end;

procedure TTestParser.TestNoHandlerError;

Var
  H : TJSONParserHandler;

begin
  H:=GetJSONParserHandler;
  try
    AssertSame('SetJSONParserHandler returns previous handler',H,SetJSONParserHandler(Nil));
    AssertException('No handler raises exception',EJSON,@CallNoHandler);
    AssertException('No handler raises exception',EJSON,@CallNoHandlerStream);
  finally
    SetJSONParserHandler(H);
  end;
end;

procedure TTestParser.TestHandlerResult;

Var
  D : TJSONData;

begin
  D:=GetJSON('"123"');
  try
    AssertEquals('Have correct string','123',D.AsString);
  finally
    D.Free;
  end;
end;

procedure TTestParser.TestHandlerResultStream;
Var
  D : TJSONData;
  S : TStream;

begin
  S:=TStringStream.Create('"123"');
  try
    D:=GetJSON(S);
    try
      AssertEquals('Have correct string','123',D.AsString);
    finally
      D.Free;
    end;
  finally
    S.Free;
  end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TTestParser.DoTestError(S : String);

Var
  P : TJSONParser;
  J : TJSONData;
  ParseOK : Boolean;
  N : String;

begin
  ParseOK:=False;
  P:=TJSONParser.Create(S);
<<<<<<< HEAD
<<<<<<< HEAD
  P.Strict:=True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  J:=Nil;
  Try
    Try
      Repeat
        FreeAndNil(J);
        J:=P.Parse;
        ParseOK:=True;
        If (J<>Nil) then
          N:=J.ClassName;
      Until (J=Nil)
    Finally
      FreeAndNil(J);
      FreeAndNil(P);
    end;
  except
    ParseOk:=False;
  end;
  If ParseOK then
    Fail('Parse of JSON string "'+S+'" should fail, but returned '+N);
end;

procedure TTestParser.DoTestString(S: String);

Var
  P : TJSONParser;
  J : TJSONData;

begin
  P:=TJSONParser.Create('"'+S+'"');
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of string "'+S+'" fails');
    TestJSONType(J,jtString);
    TestAsString(J,JSONStringToString(S));
    TestJSON(J,'"'+S+'"');
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;

end;

procedure TTestParser.DoTestFloat(F : TJSONFloat);

Var
  S : String;

begin
  Str(F,S);
  DoTestFloat(F,S);
end;

procedure TTestParser.DoTestFloat(F : TJSONFloat; S : String);

Var
  P : TJSONParser;
  J : TJSONData;

begin
  P:=TJSONParser.Create(S);
  Try
    J:=P.Parse;
    If (J=Nil) then
      Fail('Parse of float '+S+' fails');
    TestJSONType(J,jtNumber);
    TestAsFloat(J,F);
  Finally
    FreeAndNil(J);
    FreeAndNil(P);
  end;
end;


initialization
  RegisterTest(TTestParser);
end.

