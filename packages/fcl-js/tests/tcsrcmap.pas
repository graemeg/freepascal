unit TCSrcMap;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, fpjson, JSSrcMap;

type

  { TCustomTestSrcMap }

  TCustomTestSrcMap = class(TTestCase)
  protected
    procedure CheckEl(aName: String; El: TJSONData; aClass: TClass);
    function GetEl(Obj: TJSONObject; aName: String; aClass: TClass): TJSONData;
  end;

  { TTestSrcMap }

  TTestSrcMap = class(TCustomTestSrcMap)
  published
    procedure Test_Base64VLQ;
    procedure TestSrcMapIgnoreDuplicate;
    procedure TestSrcMapNames;
    procedure TestSrcMapLoad;
  end;

implementation

{ TCustomTestSrcMap }

procedure TCustomTestSrcMap.CheckEl(aName: String; El: TJSONData; aClass: TClass);
begin
  AssertNotNull('json "'+aName+'" exists',El);
  AssertEquals('json "'+aName+'" class',El.ClassType,aClass);
end;

function TCustomTestSrcMap.GetEl(Obj: TJSONObject; aName: String; aClass: TClass): TJSONData;
begin
  Result:=Obj.Elements[aName];
  CheckEl(aName,Result,aClass);
end;

{ TTestSrcMap }

procedure TTestSrcMap.Test_Base64VLQ;
var
  i: Integer;
  s: String;
  p: PChar;
  j: NativeInt;
begin
  for i:=-511 to 511 do
  begin
    s:=EncodeBase64VLQ(i);
    p:=PChar(s);
    j:=DecodeBase64VLQ(p);
    if i<>j then
      Fail('Encode/DecodeBase64VLQ OrigIndex='+IntToStr(i)+' Code="'+s+'" NewIndex='+IntToStr(j));
  end;
end;

procedure TTestSrcMap.TestSrcMapIgnoreDuplicate;
var
  sm: TSourceMap;
  Obj: TJSONObject;
  El: TJSONData;
  Arr: TJSONArray;
begin
  Obj:=nil;
  sm:=TSourceMap.Create('generated.js');
  try
    sm.AddMapping(1,0,'a.js',1,0);
    sm.AddMapping(2,0);
    sm.AddMapping(2,0);
    sm.AddMapping(3,0,'a.js',2,0);

    //writeln(sm.ToString);
    {
      version: 3,
      file: 'generated.js',
      sources: ['a.js'],
      names: [],
      mappings: 'AAAA;A;AACA'
    }
    Obj:=sm.ToJSON;

    // version
    El:=GetEl(Obj,'version',TJSONIntegerNumber);
    AssertEquals('json "version" value',El.AsInt64,3);

    // file
    El:=GetEl(Obj,'file',TJSONString);
    AssertEquals('json "file" value',El.AsString,'generated.js');

    // sources
    Arr:=TJSONArray(GetEl(Obj,'sources',TJSONArray));
    AssertEquals('json "sources".count',Arr.Count,1);
    El:=Arr[0];
    CheckEl('sources[0]',El,TJSONString);
    AssertEquals('json "sources[0]" value',El.AsString,'a.js');

    // names
    Arr:=TJSONArray(GetEl(Obj,'names',TJSONArray));
    AssertEquals('json "names".count',Arr.Count,0);

    // mappings
    El:=GetEl(Obj,'mappings',TJSONString);
    AssertEquals('json "mappings" value',El.AsString,'AAAA;A;AACA');

  finally
    Obj.Free;
    sm.Free;
  end;
end;

procedure TTestSrcMap.TestSrcMapNames;
var
  sm: TSourceMap;
  Obj: TJSONObject;
  El: TJSONData;
  Arr: TJSONArray;
begin
  Obj:=nil;
  sm:=TSourceMap.Create('generated.js');
  try
    sm.AddMapping(1,1,'a.js',2,2,'foo');
    sm.AddMapping(3,3,'a.js',4,4,'foo');
    {$IFDEF VerboseSrcMap}
    writeln(sm.ToString);
    {$ENDIF}
    {
      version: 3,
      file: 'generated.js',
      sources: ['a.js'],
      names: ['foo'],
      mappings: 'CACEA;;GAEEA'
    }
    Obj:=sm.ToJSON;

    // version
    El:=GetEl(Obj,'version',TJSONIntegerNumber);
    AssertEquals('json "version" value',3,El.AsInt64);

    // file
    El:=GetEl(Obj,'file',TJSONString);
    AssertEquals('json "file" value','generated.js',El.AsString);

    // sources
    Arr:=TJSONArray(GetEl(Obj,'sources',TJSONArray));
    AssertEquals('json "sources".count',Arr.Count,1);
    El:=Arr[0];
    CheckEl('sources[0]',El,TJSONString);
    AssertEquals('json "sources[0]" value','a.js',El.AsString);

    // names
    Arr:=TJSONArray(GetEl(Obj,'names',TJSONArray));
    AssertEquals('json "names".count',Arr.Count,1);
    El:=Arr[0];
    CheckEl('names[0]',El,TJSONString);
    AssertEquals('json "names[0]" value','foo',El.AsString);

    // mappings
    El:=GetEl(Obj,'mappings',TJSONString);
    AssertEquals('json "mappings" value','CACEA;;GAEEA',El.AsString);

  finally
    Obj.Free;
    sm.Free;
  end;
end;

procedure TTestSrcMap.TestSrcMapLoad;
var
  sm, sm2: TSourceMap;
  Obj: TJSONObject;
  i: Integer;
begin
  Obj:=nil;
  sm2:=nil;
  sm:=TSourceMap.Create('generated.js');
  try
    sm.AddMapping(1,1,'a.js',2,2,'foo');
    sm.AddMapping(3,3,'a.js',4,4,'foo');
    {$IFDEF VerboseSrcMap}
    writeln(sm.ToString);
    {$ENDIF}
    {
      version: 3,
      file: 'generated.js',
      sources: ['a.js'],
      names: ['foo'],
      mappings: 'CACEA;;GAEEA'
    }
    Obj:=sm.ToJSON;

    sm2:=TSourceMap.Create('(not set)');
    sm2.LoadFromJSON(Obj);

    AssertEquals('same GeneratedFilename',sm.GeneratedFilename,sm2.GeneratedFilename);
    AssertEquals('same SourceCount',sm.SourceCount,sm2.SourceCount);
    for i:=0 to sm.SourceCount-1 do
      AssertEquals('same SourceFiles['+IntToStr(i)+']',sm.SourceFiles[i],sm2.SourceFiles[i]);
    AssertEquals('same NameCount',sm.NameCount,sm2.NameCount);
    for i:=0 to sm.NameCount-1 do
      AssertEquals('same Names['+IntToStr(i)+']',sm.Names[i],sm2.Names[i]);
    AssertEquals('same Count',sm.Count,sm2.Count);
    for i:=0 to sm.Count-1 do
      begin
      AssertEquals('same Items['+IntToStr(i)+'].Index',sm[i].Index,sm2[i].Index);
      AssertEquals('same Items['+IntToStr(i)+'].GeneratedLine',sm[i].GeneratedLine,sm2[i].GeneratedLine);
      AssertEquals('same Items['+IntToStr(i)+'].GeneratedColumn',sm[i].GeneratedColumn,sm2[i].GeneratedColumn);
      AssertEquals('same Items['+IntToStr(i)+'].SrcFileIndex',sm[i].SrcFileIndex,sm2[i].SrcFileIndex);
      AssertEquals('same Items['+IntToStr(i)+'].SrcLine',sm[i].SrcLine,sm2[i].SrcLine);
      AssertEquals('same Items['+IntToStr(i)+'].SrcColumn',sm[i].SrcColumn,sm2[i].SrcColumn);
      AssertEquals('same Items['+IntToStr(i)+'].NameIndex',sm[i].NameIndex,sm2[i].NameIndex);
      end;

  finally
    Obj.Free;
    sm.Free;
    sm2.Free;
  end;
end;

initialization
  RegisterTests([TTestSrcMap]);
end.

