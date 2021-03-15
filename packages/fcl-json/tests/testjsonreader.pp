unit testjsonreader;

{$mode objfpc}

interface

uses
  Classes, SysUtils, fpcunit, testregistry,fpjson,jsonscanner,jsonreader, testjsondata;

Const
  DefaultOpts = [joUTF8,joStrict];

type
  
  { TMyJSONReader }

  TMyJSONReader = Class(TBaseJSONReader)
  Private
    FList : TStrings;
    function GetList: TStrings;
    procedure Push(const aType : String; const AValue: String='');
  protected
    procedure BooleanValue(const AValue: Boolean); override;
    procedure EndArray; override;
    procedure EndObject; override;
    procedure FloatValue(const AValue: Double); override;
    procedure Int64Value(const AValue: int64); override;
    procedure IntegerValue(const AValue: integer); override;
    procedure KeyValue(const AKey: TJSONStringType); override;
    procedure NullValue; override;
    procedure NumberValue(const AValue: TJSONStringType); override;
    procedure QWordValue(const AValue: QWord); override;
    procedure StartArray; override;
    procedure StartObject; override;
    procedure StringValue(const AValue: TJSONStringType); override;
  Public
    destructor Destroy; override;
    Property List : TStrings Read GetList;
  end;

  { TTestParser }

  { TTestReader }

  { TBaseTestReader }

  TBaseTestReader = class(TTestJSON)
  private
    FOptions : TJSONOptions;
    procedure CallNoHandlerStream;
    procedure DoTestFloat(F: TJSONFloat); overload;
    procedure DoTestFloat(F: TJSONFloat; S: String); overload;
    procedure DoTestString(S: String; AValue: String='');
    procedure DoTrailingCommaErrorArray;
    procedure DoTrailingCommaErrorObject;
  Protected
    procedure DoTestError(S: String; Options : TJSONOptions = DefaultOpts); virtual; abstract;
    Procedure TestRead(aJSON : String; AResult : Array of string); virtual; abstract;
  published
    procedure TestEmpty;
    procedure TestNull;
    procedure TestTrue;
    procedure TestFalse;
    procedure TestFloat;
    procedure TestFloatError;
    procedure TestInteger;
    procedure TestInt64;
    procedure TestString;
    procedure TestArray;
    procedure TestObject;
    procedure TestObjectError;
    procedure TestTrailingComma;
    procedure TestTrailingCommaErrorArray;
    procedure TestTrailingCommaErrorObject;
    procedure TestMixed;
    Procedure TestComment;
    procedure TestErrors;
    procedure TestGarbageOK;
    procedure TestGarbageFail;
  end;

  TTestReader = Class(TBaseTestReader)
  Private
    FReader: TMyJSONReader;
  Protected
    Procedure Teardown; override;
  Public
    procedure DoTestError(S: String; Options : TJSONOptions = DefaultOpts); override;
    Procedure TestRead(aJSON : String; AResult : Array of string); override;
    Property Reader : TMyJSONReader Read FReader;
  end;

  { TJSONConsumer }

  TJSONConsumer = Class(TInterfacedObject,IJSONConsumer)
  Private
    FList : TStrings;
    procedure Push(const aType : String; const AValue: String='');
  protected
    procedure BooleanValue(const AValue: Boolean);
    procedure EndArray;
    procedure EndObject;
    procedure FloatValue(const AValue: Double);
    procedure Int64Value(const AValue: int64);
    procedure IntegerValue(const AValue: integer);
    procedure KeyName(const AKey: TJSONStringType);
    procedure NullValue;
    procedure NumberValue(const AValue: TJSONStringType);
    procedure QWordValue(const AValue: QWord);
    procedure StartArray;
    procedure StartObject;
    procedure StringValue(const AValue: TJSONStringType);
  Public
    Constructor Create(AList : TStrings);
    Property List : TStrings Read FList;
  end;

  { TTestJSONConsumerReader }

  TTestJSONConsumerReader = Class(TBaseTestReader)
  Private
    FList : TStrings;
    FReader: TJSONConsumerReader;
  Protected
    Procedure Teardown; override;
  Public
    procedure DoTestError(S: String; Options : TJSONOptions = DefaultOpts); override;
    Procedure TestRead(aJSON : String; AResult : Array of string); override;
    Property Reader : TJSONConsumerReader Read FReader;
  end;

  { TTestJSONEventReader }

  TTestJSONEventReader = Class(TBaseTestReader)
  Private
    FList : TStrings;
    FReader: TJSONEventReader;
  Protected
    procedure Push(const aType : String; const AValue: String='');
    procedure BooleanValue(Sender: TObject; const AValue: Boolean);
    procedure EndArray(Sender: TObject);
    procedure EndObject(Sender: TObject);
    procedure FloatValue(Sender: TObject; const AValue: Double);
    procedure Int64Value(Sender: TObject; const AValue: int64);
    procedure IntegerValue(Sender: TObject; const AValue: integer);
    procedure KeyValue(Sender: TObject; const AKey: TJSONStringType);
    procedure NullValue(Sender: TObject);
    procedure NumberValue(Sender: TObject; const AValue: TJSONStringType);
    procedure QWordValue(Sender: TObject; const AValue: QWord);
    procedure StartArray(Sender: TObject);
    procedure StartObject(Sender: TObject);
    procedure StringValue(Sender: TObject; const AValue: TJSONStringType);
    Procedure HookupEvents(AReader: TJSONEventReader);
    Procedure Teardown; override;
  Public
    procedure DoTestError(S: String; Options : TJSONOptions = DefaultOpts); override;
    Procedure TestRead(aJSON : String; AResult : Array of string); override;
    Property Reader : TJSONEventReader Read FReader;
  end;


implementation


{ TMyJSONReader }

function TMyJSONReader.GetList: TStrings;
begin
  If FList=Nil then
    FList:=TStringList.Create;
  Result:=Flist;
end;

procedure TMyJSONReader.Push(const aType : String; const AValue : String = '');

begin
  if AValue<>'' then
    List.Add(aType+':'+AValue)
  else
    List.Add(aType);
end;

procedure TMyJSONReader.BooleanValue(const AValue: Boolean);
begin
  Push('boolean',BoolToStr(AValue));
end;

procedure TMyJSONReader.EndArray;
begin
  Push('ea');
end;

procedure TMyJSONReader.EndObject;
begin
  Push('eo');
end;

procedure TMyJSONReader.FloatValue(const AValue: Double);
begin
  List.Add('float:'+formatFloat('##.##',AVAlue));
end;

procedure TMyJSONReader.Int64Value(const AValue: int64);
begin
  Push('int64',IntToStr(aValue));
end;

procedure TMyJSONReader.IntegerValue(const AValue: integer);
begin
  Push('integer',IntToStr(aValue));
end;

procedure TMyJSONReader.KeyValue(const AKey: TJSONStringType);
begin
  Push('key',akey);
end;

procedure TMyJSONReader.NullValue;
begin
  Push('null');
end;

procedure TMyJSONReader.NumberValue(const AValue: TJSONStringType);
begin
  Push('number',aValue);
end;

procedure TMyJSONReader.QWordValue(const AValue: QWord);
begin
  Push('qword',IntToStr(AValue));
end;

procedure TMyJSONReader.StartArray;
begin
  Push('sa');
end;

procedure TMyJSONReader.StartObject;
begin
  Push('so');
end;

procedure TMyJSONReader.StringValue(const AValue: TJSONStringType);
begin
  List.Add('string:'+AValue)
end;

destructor TMyJSONReader.Destroy;
begin
  FreeAndNil(Flist);
  inherited Destroy;
end;

procedure TBaseTestReader.TestEmpty;

begin
  TestRead('',[]);
end;

procedure TBaseTestReader.TestInteger;


begin
  TestRead('1',['number:1','integer:1']);
end;

procedure TBaseTestReader.TestInt64;

begin
  TestRead('123456789012345',['number:123456789012345','int64:123456789012345']);
end;

procedure TBaseTestReader.TestNull;


begin
  TestRead('null',['null']);
end;

procedure TBaseTestReader.TestTrue;

begin
  TestRead('true',['boolean:'+BoolToStr(true)]);
end;

procedure TBaseTestReader.TestFalse;

begin
  TestRead('false',['boolean:'+BoolToStr(false)]);
end;

procedure TBaseTestReader.TestFloat;


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

procedure TBaseTestReader.TestFloatError;
begin
  DoTestError('.12',[joStrict]);
  DoTestError('.12E',[]);
  DoTestError('0.12E+',[]);
  DoTestError('.12E+-1',[]);
end;

procedure TBaseTestReader.TestString;

const
  GlowingStar = #$F0#$9F#$8C#$9F;
  Chinese = #$95e8#$88ab#$8111#$5b50#$6324#$574f#$4e86;
  Chinese4b = #$95e8#$d867#$de3d#$88ab#$8111#$5b50#$6324#$574f#$4e86;

begin
  DoTestString('A string');
  DoTestString('');
  DoTestString('\"','"');
  DoTestString('\u00f8','ø'); // this is ø
  DoTestString('\u00f8\"','ø"'); // this is ø"
  DoTestString('\ud83c\udf1f',GlowingStar);
  DoTestString('\u0041\u0042','AB');   //issue #0038622
  DoTestString('\u0041\u0042\u0043','ABC');
  DoTestString('\u0041\u0042\u0043\u0044','ABCD');
  DoTestString('\u95e8\u88ab\u8111\u5b50\u6324\u574f\u4e86',Utf8Encode(Chinese));
  DoTestString('\u95e8\ud867\ude3d\u88ab\u8111\u5b50\u6324\u574f\u4e86',Utf8Encode(Chinese4b));   
end;


procedure TBaseTestReader.TestArray;

Var
  S1,S2,S3 : String;

begin
  TestRead('[]',['sa','ea']);
  TestRead('[null]',['sa','null','ea']);
  TestRead('[true]',['sa','boolean:'+BoolToStr(true),'ea']);
  TestRead('[false]',['sa','boolean:'+BoolToStr(false),'ea']);
  TestRead('[1]',['sa','number:1','integer:1','ea']);
  TestRead('[1, 2]',['sa','number:1','integer:1','number:2','integer:2','ea']);
  TestRead('[1, 2, 3]',['sa','number:1','integer:1','number:2','integer:2','number:3','integer:3','ea']);
  TestRead('[1234567890123456]',['sa','number:1234567890123456','int64:1234567890123456','ea']);
  TestRead('[1234567890123456, 2234567890123456]',
    ['sa','number:1234567890123456','int64:1234567890123456','number:2234567890123456','int64:2234567890123456','ea']);
  TestRead('[1234567890123456, 2234567890123456, 3234567890123456]',
    ['sa','number:1234567890123456','int64:1234567890123456','number:2234567890123456','int64:2234567890123456',
     'number:3234567890123456','int64:3234567890123456','ea']);
  Str(12/10,S1);
  Delete(S1,1,1);
  Str(34/10,S2);
  Delete(S2,1,1);
  Str(34/10,S3);
  Delete(S3,1,1);
  TestRead('['+S1+']',['sa','number:'+s1,'float:'+formatfloat('##.##',12/10),'ea']);
  {
  TestRead('['+S1+', '+S2+']',2,true);
  TestRead('['+S1+', '+S2+', '+S3+']',3,true);
  TestRead('["A string"]',1);
  TestRead('["A string", "Another string"]',2);
  TestRead('["A string", "Another string", "Yet another string"]',3);
  TestRead('[null, false]',2);
  TestRead('[true, false]',2);
  TestRead('[null, 1]',2);
  TestRead('[1, "A string"]',2);
  TestRead('[1, []]',2);
  TestRead('[1, [1, 2]]',2);}
end;

procedure TBaseTestReader.TestTrailingComma;
begin
  FOptions:=[joIgnoreTrailingComma];
  TestRead('[1, 2, ]',['sa','number:1','integer:1','number:2','integer:2','ea']);
  TestRead('{ "a" : 1, }',['so','key:a', 'number:1','integer:1','eo']);
end;

procedure TBaseTestReader.TestTrailingCommaErrorArray;
begin
  AssertException('Need joIgnoreTrailingComma in options to allow trailing comma',EJSONParser,@DoTrailingCommaErrorArray) ;
end;

procedure TBaseTestReader.TestTrailingCommaErrorObject;
begin
  AssertException('Need joIgnoreTrailingComma in options to allow trailing comma',EJSONParser,@DoTrailingCommaErrorObject);
end;

procedure TBaseTestReader.DoTrailingCommaErrorArray;
begin
  TestRead('[1, 2, ]',['sa','number:1','integer:1','number:2','integer:2','ea']);
end;

procedure TBaseTestReader.DoTrailingCommaErrorObject;
begin
  TestRead('{ "a" : 1, }',['so','key:a', 'number:1','integer:1','eo']);
end;


procedure TBaseTestReader.TestMixed;

begin
  TestRead('[1, {}]',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('[1, { "a" : 1 }]',['sa','number:1','integer:1','so','key:a','number:1','integer:1','eo','ea']);
  TestRead('[1, { "a" : 1 }, 1]',['sa','number:1','integer:1','so','key:a','number:1','integer:1','eo','number:1','integer:1','ea']);
  TestRead('{ "a" : [1, 2] }',['so','key:a','sa','number:1','integer:1','number:2','integer:2','ea','eo']);
  TestRead('{ "a" : [1, 2], "B" : { "c" : "d" } }',
    ['so','key:a','sa','number:1','integer:1','number:2','integer:2','ea','key:B','so','key:c','string:d','eo','eo']);
end;

procedure TBaseTestReader.TestComment;
begin
  FOptions:=[joComments];
  TestRead('/* */ [1, {}]',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('//'+sLineBreak+' [1, {}]',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('/* '+sLineBreak+' */ [1, {}]',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('/*'+sLineBreak+'*/ [1, {}]',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('/*'+sLineBreak+'*'+sLineBreak+'*/ [1, {}]',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('/**'+sLineBreak+'**'+sLineBreak+'**/ [1, {}]',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('/* */ [1, {}]',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('[1, {}]//',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('[1, {}]/* '+sLineBreak+' */',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('[1, {}]/* '+sLineBreak+' */ ',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('[1, {}]/* '+sLineBreak+'*'+sLineBreak+'*/ ',['sa','number:1','integer:1','so','eo','ea']);
  TestRead('[1, {}]/**'+sLineBreak+'**'+sLineBreak+'**/ ',['sa','number:1','integer:1','so','eo','ea']);
end;

procedure TBaseTestReader.TestObject;
begin
  TestRead('{}',['so','eo']);
  TestRead('{ "a" : 1 }',['so','key:a','number:1','integer:1','eo']);
  TestRead('{ "a" : 1, "B" : "String" }',['so','key:a','number:1','integer:1','key:B','string:String','eo']);
  TestRead('{ "a" : 1, "B" : {} }',['so','key:a','number:1','integer:1','key:B','so','eo','eo']);
  TestRead('{ "a" : 1, "B" : { "c" : "d" } }',['so','key:a','number:1','integer:1','key:B','so','key:c','string:d','eo','eo']);
end;

procedure TBaseTestReader.TestObjectError;
begin
  DoTestError('{ "name" : value }',[joUTF8]);
end;


procedure TBaseTestReader.TestErrors;

begin

  DoTestError('a');
  DoTestError('"b');
  DoTestError('1Tru');

  DoTestError('b"');
  DoTestError('{"a" : }');
  DoTestError('{"a" : ""');
  DoTestError('{"a : ""');

  DoTestError('[1,]');
  DoTestError('[,]');
  DoTestError('[,,]');
  DoTestError('[1,,]');

end;

procedure TBaseTestReader.TestGarbageOK;
begin
  TestRead('"a"sss',['string:a']);
  TestRead('[null]xxx',['sa','null','ea']);
end;

procedure TBaseTestReader.TestGarbageFail;
begin
  DoTestError('"a"sss',[joStrict]);
  DoTestError('[null]aaa',[joStrict]);
end;


procedure TBaseTestReader.CallNoHandlerStream;

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

procedure TBaseTestReader.DoTestString(S: String; AValue : String = '');

begin
  if AValue='' then
    AValue:=S;
  TestRead('"'+S+'"',['string:'+AValue]);
end;

procedure TBaseTestReader.DoTestFloat(F : TJSONFloat);

Var
  S : String;

begin
  Str(F,S);
  DoTestFloat(F,S);
end;

procedure TBaseTestReader.DoTestFloat(F : TJSONFloat; S : String);

begin
  TestRead(S,['number:'+trim(S),'float:'+formatfloat('##.##',F)]);
end;

procedure TTestReader.Teardown;
begin
  FreeAndNil(FReader);
  inherited Teardown;
end;

procedure TTestReader.TestRead(aJSON: String; AResult: array of string);

Var
  I : Integer;

begin
  FreeAndNil(FReader);
  FReader:=TMyJSONReader.Create(aJSON,Foptions);
  TMyJSONReader(FReader).DoExecute;
  AssertEquals(aJSON+': Number of events',Length(AResult),FReader.List.Count);
  For I:=0 to Length(AResult)-1 do
    AssertEquals(aJSON+': Event number '+IntToStr(I),AResult[i],FReader.List[I]);
end;

procedure TTestReader.DoTestError(S : String; Options : TJSONOptions = DefaultOpts);

Var
  P:TMyJSONReader;
  ParseOK : Boolean;

begin
  ParseOK:=False;
  P:=TMyJSONReader.Create(S,FOptions);
  P.OPtions:=Options;
  Try
    Try
      P.DoExecute;
      ParseOk:=True;
    Finally
      FreeAndNil(P);
    end;
  except
    ParseOk:=False;
  end;
  If ParseOK then
    Fail('Parse of JSON string "'+S+'" should fail, but succeeded');
end;


{ TJSONConsumer }

procedure TJSONConsumer.Push(const aType : String; const AValue : String = '');

begin
  if AValue<>'' then
    List.Add(aType+':'+AValue)
  else
    List.Add(aType);
end;

procedure TJSONConsumer.BooleanValue(const AValue: Boolean);
begin
  Push('boolean',BoolToStr(AValue));
end;

procedure TJSONConsumer.EndArray;
begin
  Push('ea');
end;

procedure TJSONConsumer.EndObject;
begin
  Push('eo');
end;

procedure TJSONConsumer.FloatValue(const AValue: Double);
begin
  List.Add('float:'+formatFloat('##.##',AVAlue));
end;

procedure TJSONConsumer.Int64Value(const AValue: int64);
begin
  Push('int64',IntToStr(aValue));
end;

procedure TJSONConsumer.IntegerValue(const AValue: integer);
begin
  Push('integer',IntToStr(aValue));
end;

procedure TJSONConsumer.KeyName(const AKey: TJSONStringType);
begin
  Push('key',akey);
end;

procedure TJSONConsumer.NullValue;
begin
  Push('null');
end;

procedure TJSONConsumer.NumberValue(const AValue: TJSONStringType);
begin
  Push('number',aValue);
end;

procedure TJSONConsumer.QWordValue(const AValue: QWord);
begin
  Push('qword',IntToStr(AValue));
end;

procedure TJSONConsumer.StartArray;
begin
  Push('sa');
end;

procedure TJSONConsumer.StartObject;
begin
  Push('so');
end;

procedure TJSONConsumer.StringValue(const AValue: TJSONStringType);
begin
  List.Add('string:'+AValue)
end;

constructor TJSONConsumer.Create(AList: TStrings);
begin
  FList:=AList;
end;

procedure TTestJSONConsumerReader.TestRead(aJSON: String; AResult: array of string);

Var
  I : Integer;

begin
  FreeAndNil(FReader);
  FreeAndNil(Flist);
  FList:=TStringList.Create;
  FReader:=TJSONConsumerReader.Create(aJSON,Foptions);
  FReader.Consumer:=TJSONConsumer.Create(FList);
  TJSONConsumerReader(FReader).Execute;
  AssertEquals(aJSON+': Number of events',Length(AResult),FList.Count);
  For I:=0 to Length(AResult)-1 do
    AssertEquals(aJSON+': Event number '+IntToStr(I),AResult[i],FList[I]);
end;

procedure TTestJSONConsumerReader.Teardown;
begin
  FreeAndNil(FReader);
  FreeAndNil(FList);
  inherited Teardown;
end;

procedure TTestJSONConsumerReader.DoTestError(S : String; Options : TJSONOptions = DefaultOpts);

Var
  P:TJSONConsumerReader;
  ParseOK : Boolean;

begin
  ParseOK:=False;
  FreeAndNil(FReader);
  FreeAndNil(Flist);
  FList:=TStringList.Create;
  P:=TJSONConsumerReader.Create(S,Options);
  P.Consumer:=TJSONConsumer.Create(FList);
  P.OPtions:=Options;
  Try
    Try
      P.Execute;
      ParseOk:=True;
    Finally
      FreeAndNil(P);
    end;
  except
    ParseOk:=False;
  end;
  If ParseOK then
    Fail('Parse of JSON string "'+S+'" should fail, but succeeded');
end;

{ TTestJSONEventReader }

procedure TTestJSONEventReader.Teardown;
begin
  FreeAndNil(Freader);
  FreeAndNil(Flist);
  inherited Teardown;
end;

procedure TTestJSONEventReader.DoTestError(S: String; Options: TJSONOptions);

Var
  P:TJSONEventReader;
  ParseOK : Boolean;

begin
  ParseOK:=False;
  FreeAndNil(FReader);
  FreeAndNil(Flist);
  FList:=TStringList.Create;
  P:=TJSONEventReader.Create(S,Options);
  HookupEvents(P);
  P.OPtions:=Options;
  Try
    Try
      P.Execute;
      ParseOk:=True;
    Finally
      FreeAndNil(P);
    end;
  except
    ParseOk:=False;
  end;
  If ParseOK then
    Fail('Parse of JSON string "'+S+'" should fail, but succeeded');
end;

procedure TTestJSONEventReader.TestRead(aJSON: String; AResult: array of string);

Var
  I : Integer;

begin
  FreeAndNil(FReader);
  FreeAndNil(Flist);
  FList:=TStringList.Create;
  FReader:=TJSONEventReader.Create(aJSON,Foptions);
  HookupEvents(FReader);
  FReader.Execute;
  AssertEquals(aJSON+': Number of events',Length(AResult),FList.Count);
  For I:=0 to Length(AResult)-1 do
    AssertEquals(aJSON+': Event number '+IntToStr(I),AResult[i],FList[I]);
end;

procedure TTestJSONEventReader.Push(const aType: String; const AValue: String);
begin
  if AValue<>'' then
    FList.Add(aType+':'+AValue)
  else
    FList.Add(aType);
end;

procedure TTestJSONEventReader.BooleanValue(Sender: TObject; const AValue: Boolean);
begin
  Push('boolean',BoolToStr(AValue));
end;

procedure TTestJSONEventReader.EndArray(Sender: TObject);
begin
  Push('ea');
end;

procedure TTestJSONEventReader.EndObject(Sender: TObject);
begin
  Push('eo');
end;

procedure TTestJSONEventReader.FloatValue(Sender: TObject; const AValue: Double);
begin
  FList.Add('float:'+formatFloat('##.##',AVAlue));
end;

procedure TTestJSONEventReader.Int64Value(Sender: TObject; const AValue: int64);
begin
  Push('int64',IntToStr(aValue));
end;

procedure TTestJSONEventReader.IntegerValue(Sender: TObject; const AValue: integer);
begin
  Push('integer',IntToStr(aValue));
end;

procedure TTestJSONEventReader.KeyValue(Sender: TObject; const AKey: TJSONStringType);
begin
  Push('key',akey);
end;

procedure TTestJSONEventReader.NullValue(Sender: TObject);
begin
  Push('null');
end;

procedure TTestJSONEventReader.NumberValue(Sender: TObject; const AValue: TJSONStringType);
begin
  Push('number',aValue);
end;

procedure TTestJSONEventReader.QWordValue(Sender: TObject; const AValue: QWord);
begin
  Push('qword',IntToStr(AValue));
end;

procedure TTestJSONEventReader.StartArray(Sender: TObject);
begin
  Push('sa');
end;

procedure TTestJSONEventReader.StartObject(Sender: TObject);
begin
  Push('so');
end;

procedure TTestJSONEventReader.StringValue(Sender: TObject; const AValue: TJSONStringType);
begin
  FList.Add('string:'+AValue)
end;

procedure TTestJSONEventReader.HookupEvents(AReader: TJSONEventReader);
begin
  With Areader do
    begin
    OnNullValue:=@NullValue;
    OnBooleanValue:=@BooleanValue;
    OnNumberValue:=@NumberValue;
    OnFloatValue:=@FloatValue;
    OnIntegerValue:=@IntegerValue;
    OnInt64Value:=@Int64Value;
    OnQWordValue:=@QWordValue;
    OnStringValue:=@StringValue;
    OnKeyName:=@KeyValue;
    OnStartObject:=@StartObject;
    OnEndObject:=@EndObject;
    OnStartArray:=@StartArray;
    OnEndArray:=@EndArray;
    end;
end;

initialization
  RegisterTests([TTestReader,TTestJSONConsumerReader,TTestJSONEventReader]);

end.

