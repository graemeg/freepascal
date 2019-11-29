{ *********************************************************************
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018 Mattias Gaertner.

    Javascript Source Map

    See Source Maps Revision 3:
    https://docs.google.com/document/d/1U1RGAehQwRypUTovF1KRlpiOFze0b-_2gc6fAH0KY0k/edit?hl=en_US&pli=1&pli=1#

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  **********************************************************************}
unit JSSrcMap;

{$mode objfpc}{$H+}

{$ifdef fpc}
  {$define UsePChar}
  {$define HasJsonParser}
  {$define HasStreams}
  {$define HasFS}
{$endif}

{$ifdef pas2js}
  {$ifdef nodejs}
    {$define HasFS}
  {$endif}
{$endif}

interface

uses
  {$ifdef pas2js}
  JS,
    {$ifdef nodejs}
    NodeJSFS,
    {$endif}
  {$else}
  contnrs,
  {$endif}
  Classes, SysUtils, fpjson
  {$ifdef HasJsonParser}
  , jsonparser, jsonscanner
  {$endif}
  ;

const
  Base64Chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';

type
  EJSSourceMap = class(Exception);

  { TSourceMapSegment }

  TSourceMapSegment = class
  public
    Index: integer; // index in Items
    GeneratedLine: integer; // 1-based
    GeneratedColumn: integer; // 0-based
    SrcFileIndex: integer; // index in FSources
    SrcLine: integer;
    SrcColumn: integer;
    NameIndex: integer; // index in FNames
  end;

  TSourceMapSrc = class
  public
    Filename: string; // as added by AddMapping
    TranslatedFilename: string; // same as Filename, can be altered, written to JSON
    Source: String;
  end;

  TSourceMapOption = (
    smoAddMonotonous, // true = AddMapping GeneratedLine/Col must be behind last add, false = check all adds for duplicate
    smoAutoLineStart, // automatically add a first column mapping, repeating last mapping
    smoSafetyHeader, // insert ')]}' at start
    smoAllowSrcLine0 // don't bark on SrcLine=0
    );
  TSourceMapOptions = set of TSourceMapOption;
const
  DefaultSourceMapOptions = [smoAddMonotonous,smoSafetyHeader];
type

  { TSourceMap }

  TSourceMap = class
  private
    type

      { TStringToIndex }

      TStringToIndex = class
      private
        FItems: {$ifdef pas2js}TJSObject{$else}TFPHashList{$endif};
      public
        constructor Create;
        destructor Destroy; override;
        procedure Clear;
        procedure Add(const Value: String; Index: integer);
        function FindValue(const Value: String): integer;
      end;
  private
    FGeneratedFilename: string;
    FNames: TStrings; // in adding order
    FNameToIndex: TStringToIndex; // name to index in FNames
    FItems: TFPList; // TSourceMapSegment, in adding order
    FOptions: TSourceMapOptions;
    FSorted: boolean;
    FSourceRoot: string;
    FSources: TFPList; // list of TSourceMapSrc, in adding order
    FSourceToIndex: TStringToIndex; // srcfile to index in FSources
    FVersion: integer;
    function GetNames(Index: integer): string;
    function GetItems(Index: integer): TSourceMapSegment;
    function GetSourceContents(Index: integer): String;
    function GetSourceFiles(Index: integer): String;
    function GetSourceTranslatedFiles(Index: integer): String;
    procedure SetGeneratedFilename(const AValue: string);
    procedure SetSorted(const AValue: boolean);
    procedure SetSourceContents(Index: integer; const AValue: String);
    procedure SetSourceTranslatedFiles(Index: integer; const AValue: String);
    procedure Sort;
  public
    constructor Create(const aGeneratedFilename: string);
    destructor Destroy; override;
    procedure Clear; virtual;
    function AddMapping(
      GeneratedLine: integer; // 1-based
      GeneratedCol: integer = 0; // 0-based
      const SourceFile: string = ''; // can be empty ''
      SrcLine: integer = 1; // 1-based
      SrcCol: integer = 0; // 0-based
      const Name: String = ''): TSourceMapSegment; virtual;
    function CreateMappings: String; virtual;
    procedure ParseMappings(const Mapping: String); virtual;
    function ToJSON: TJSONObject; virtual;
    function ToString: string; override;
    procedure LoadFromJSON(Obj: TJSONObject); virtual;
    procedure SaveToStream(aStream: TFPJSStream); virtual;
    {$ifdef HasStreams}
    procedure LoadFromStream(aStream: TStream); virtual;
    procedure SaveToFile(Filename: string); virtual;
    procedure LoadFromFile(Filename: string); virtual;
    {$endif}
    property GeneratedFilename: string read FGeneratedFilename write SetGeneratedFilename;
    function IndexOfName(const Name: string; AddIfNotExists: boolean = false): integer;
    function IndexOfSourceFile(const SrcFile: string; AddIfNotExists: boolean = false): integer;
    function IndexOfSegmentAt(GeneratedLine, GeneratedCol: integer): integer;
    function Count: integer; // segments
    property Items[Index: integer]: TSourceMapSegment read GetItems; default; // segments
    function SourceCount: integer;
    property SourceRoot: string read FSourceRoot write FSourceRoot;
    property SourceFiles[Index: integer]: String read GetSourceFiles;
    property SourceTranslatedFiles[Index: integer]: String read GetSourceTranslatedFiles
      write SetSourceTranslatedFiles;
    property SourceContents[Index: integer]: String read GetSourceContents write SetSourceContents;
    function NameCount: integer;
    property Names[Index: integer]: string read GetNames;
    property Version: integer read FVersion; // 3
    property Options: TSourceMapOptions read FOptions write FOptions;
    property Sorted: boolean read FSorted write SetSorted; // Segments are sorted for GeneratedLine/Col
  end;

function DefaultSrcMapHeader: string;

function EncodeBase64VLQ(i: NativeInt): String; // base64 Variable Length Quantity
function DecodeBase64VLQ(const s: string): NativeInt; // base64 Variable Length Quantity
function DecodeBase64VLQ(
  {$ifdef UsePChar}var p: PChar{$else}const s: string; var p: integer{$endif}): NativeInt; // base64 Variable Length Quantity

function CompareSegmentWithGeneratedLineCol(
  Item1, Item2: {$ifdef pas2js}jsvalue{$else}Pointer{$endif}): Integer;

procedure DebugSrcMapLine(GeneratedLine: integer; var GeneratedLineSrc: String;
  SrcMap: TSourceMap; out InfoLine: String);

implementation

function DefaultSrcMapHeader: string;
begin
  Result:=')]}'''+LineEnding;
end;

function EncodeBase64VLQ(i: NativeInt): String;
{ Convert signed number to base64-VLQ:
  Each base64 has 6bit, where the most significant bit is the continuation bit
  (1=there is a next base64 character).
  The first character contains the sign bit in the last bit (1=negative)
  and the 5 least significant bits of the number.
  For example:
  A = 0 = %000000 => 0
  B = 1 = %000001 => -0
  C = 2 = %000010 => 1
  iF = 34 5 = %100010 %000101 = + 0001 00101 = 100101 = 37
}

  procedure RaiseRange;
  begin
    raise ERangeError.Create('EncodeBase64VLQ');
  end;

var
  digits: NativeInt;
begin
  Result:='';
  if i<0 then
    begin
    i:=-i;
    if i>(High(NativeInt)-1) shr 1 then
      RaiseRange;
    i:=(i shl 1)+1;
    end
  else
    begin
    if i>High(NativeInt) shr 1 then
      RaiseRange;
    i:=i shl 1;
    end;
  repeat
    digits:=i and %11111;
    i:=i shr 5;
    if i>0 then
      inc(digits,%100000); // need another char -> set continuation bit
    Result:=Result+Base64Chars[digits+1];
  until i=0;
end;

function DecodeBase64VLQ(const s: string): NativeInt;
var
  {$ifdef UsePChar}
  p: PChar;
  {$else}
  p: integer;
  {$endif}
begin
  if s='' then
    raise EConvertError.Create('DecodeBase64VLQ empty');
  {$ifdef UsePChar}
  p:=PChar(s);
  Result:=DecodeBase64VLQ(p);
  if p-PChar(s)<>length(s) then
    raise EConvertError.Create('DecodeBase64VLQ waste');
  {$else}
  p:=1;
  Result:=DecodeBase64VLQ(s,p);
  {$endif}
end;

function DecodeBase64VLQ(
  {$ifdef UsePChar}var p: PChar{$else}const s: string; var p: integer{$endif}): NativeInt;
{ Convert base64-VLQ to signed number,
  For the fomat see EncodeBase64VLQ
}
var
  {$ifdef UsePChar}
  run: PChar;
  {$else}
  run, l: integer;
  {$endif}

  procedure RaiseInvalid;
  begin
    p:=run;
    raise ERangeError.Create('DecodeBase64VLQ');
  end;

const
  MaxShift = {$ifdef pas2js}32{$else}63{$endif}-5; // actually log2(High(NativeInt))-5
var
  c: Char;
  digit, Shift: Integer;
begin
  Result:=0;
  Shift:=0;
  run:=p;
  {$ifdef UsePChar}
  {$else}
  l:=length(s);
  {$endif}
  repeat
    {$ifdef UsePChar}
    c:=run^;
    {$else}
    if run>l then
      RaiseInvalid;
    c:=s[run];
    {$endif}
    case c of
    'A'..'Z': digit:=ord(c)-ord('A');
    'a'..'z': digit:=ord(c)-ord('a')+26;
    '0'..'9': digit:=ord(c)-ord('0')+52;
    '+': digit:=62;
    '/': digit:=63;
    else RaiseInvalid;
    end;
    inc(run);
    if Shift>MaxShift then
      RaiseInvalid;
    inc(Result,(digit and %11111) shl Shift);
    inc(Shift,5);
  until digit<%100000;
  if (Result and 1)>0 then
    Result:=-(Result shr 1)
  else
    Result:=Result shr 1;
  p:=run;
end;

function CompareSegmentWithGeneratedLineCol(
    Item1, Item2: {$ifdef pas2js}jsvalue{$else}Pointer{$endif}): Integer;
var
  Seg1: TSourceMapSegment absolute Item1;
  Seg2: TSourceMapSegment absolute Item2;
begin
  if Seg1.GeneratedLine<Seg2.GeneratedLine then
    Result:=-1
  else if Seg1.GeneratedLine>Seg2.GeneratedLine then
    Result:=1
  else if Seg1.GeneratedColumn<Seg2.GeneratedColumn then
    Result:=-1
  else if Seg1.GeneratedColumn>Seg2.GeneratedColumn then
    Result:=1
  // compare Index to keep adding order
  else if Seg1.Index<Seg2.Index then
    Result:=-1
  else if Seg1.Index>Seg2.Index then
    Result:=1
  else
    Result:=0;
end;

procedure DebugSrcMapLine(GeneratedLine: integer; var GeneratedLineSrc: String;
  SrcMap: TSourceMap; out InfoLine: String);
var
  JS, Origins, Addition: String;
  GeneratedCol: integer; // 0-based
  i, diff, GenColStep, LastSrcFile, LastSrcLine: Integer;
  aSeg: TSourceMapSegment;
begin
  InfoLine:='';
  JS:=GeneratedLineSrc;
  Origins:='';
  GeneratedCol:=0;// 0-based
  LastSrcFile:=0;
  LastSrcLine:=-1;
  i:=SrcMap.IndexOfSegmentAt(GeneratedLine,GeneratedCol);
  aSeg:=nil;
  if i<0 then
    begin
    // no segment at line start
    i:=0;
    if (i=SrcMap.Count) then
      aSeg:=nil
    else
      aSeg:=SrcMap[i];
    if (aSeg=nil) or (aSeg.GeneratedLine>GeneratedLine) then
      begin
      // no segment in line
      for i:=1 to length(JS) do Origins:=Origins+'?';
      GeneratedLineSrc:=JS;
      InfoLine:=Origins;
      exit;
      end
    else
      begin
      // show "?" til start of first segment
      for i:=1 to aSeg.GeneratedColumn do Origins:=Origins+'?';
      end;
    end
  else
    begin
    aSeg:=SrcMap[i];
    if i>0 then
      LastSrcFile:=SrcMap[i-1].SrcFileIndex;
    end;

  repeat
    Addition:='';
    if (aSeg.GeneratedLine=GeneratedLine) and (aSeg.GeneratedColumn=GeneratedCol) then
      begin
      // segment starts here  -> write "|line,col"
      Addition:='|';
      if LastSrcFile<>aSeg.SrcFileIndex then
        begin
        Addition:=Addition+{$ifdef HasFS}ExtractFileName{$endif}(SrcMap.SourceFiles[aSeg.SrcFileIndex])+',';
        LastSrcFile:=aSeg.SrcFileIndex;
        end;
      if LastSrcLine<>aSeg.SrcLine then
        begin
        Addition:=Addition+IntToStr(aSeg.SrcLine)+',';
        LastSrcLine:=aSeg.SrcLine;
        end;
      Addition:=Addition+IntToStr(aSeg.SrcColumn);
      Origins:=Origins+Addition;
      end;
    inc(i);
    // skip segments at same GeneratedLine/Col
    while (i<SrcMap.Count) do
      begin
      aSeg:=SrcMap[i];
      if (aSeg.GeneratedLine=GeneratedLine) and (aSeg.GeneratedColumn=GeneratedCol) then
        inc(i)
      else
        break;
      end;
    if (i=SrcMap.Count) then
      aSeg:=nil
    else
      aSeg:=SrcMap[i];
    if (aSeg=nil) or (aSeg.GeneratedLine>GeneratedLine) then
      begin
      // in the last segment
      while length(Origins)<length(JS) do
        Origins:=Origins+'.';
      GeneratedLineSrc:=JS;
      InfoLine:=Origins;
      exit;
      end;
    // there is another segment in this line
    // -> align JS and Origins
    GenColStep:=aSeg.GeneratedColumn-GeneratedCol;
    diff:=GenColStep-length(Addition);
    if diff<0 then
      // for example:
      //  JS:       if(~~e)~~~{
      //  Origins:  |12,3|12,5|12,7
      Insert(StringOfChar('~',-diff),JS,length(Origins)-length(Addition)+1+GenColStep)
    else
      while diff>0 do
        begin
        Origins:=Origins+'.';
        dec(diff);
        end;
    GeneratedCol:=aSeg.GeneratedColumn;
  until false;
end;

{ TSourceMap.TStringToIndex }

constructor TSourceMap.TStringToIndex.Create;
begin
  {$ifdef pas2js}
  FItems:=TJSObject.new;
  {$else}
  FItems:=TFPHashList.Create;
  {$endif}
end;

destructor TSourceMap.TStringToIndex.Destroy;
begin
  {$ifdef pas2js}
  FItems:=nil;
  {$else}
  FItems.Clear;
  FreeAndNil(FItems);
  {$endif}
  inherited Destroy;
end;

procedure TSourceMap.TStringToIndex.Clear;
begin
  {$ifdef pas2js}
  FItems:=TJSObject.new;
  {$else}
  FItems.Clear;
  {$endif}
end;

procedure TSourceMap.TStringToIndex.Add(const Value: String; Index: integer);
begin
  {$ifdef pas2js}
  FItems['%'+Value]:=Index;
  {$else}
  // Note: nil=0 means not found in TFPHashList
  FItems.Add(Value,{%H-}Pointer(PtrInt(Index+1)));
  {$endif}
end;

function TSourceMap.TStringToIndex.FindValue(const Value: String
  ): integer;
begin
  {$ifdef pas2js}
  if FItems.hasOwnProperty('%'+Value) then
    Result:=integer(FItems['%'+Value])
  else
    Result:=-1;
  {$else}
  // Note: nil=0 means not found in TFPHashList
  Result:=integer({%H-}PtrInt(FItems.Find(Value))){%H-}-1;
  {$endif}
end;

{ TSourceMap }

procedure TSourceMap.SetGeneratedFilename(const AValue: string);
begin
  if FGeneratedFilename=AValue then Exit;
  FGeneratedFilename:=AValue;
end;

procedure TSourceMap.SetSorted(const AValue: boolean);
begin
  if FSorted=AValue then Exit;
  if AValue then
    Sort
  else
    FSorted:=false;
end;

procedure TSourceMap.SetSourceContents(Index: integer; const AValue: String);
begin
  TSourceMapSrc(FSources[Index]).Source:=AValue;
end;

procedure TSourceMap.SetSourceTranslatedFiles(Index: integer;
  const AValue: String);
begin
  TSourceMapSrc(FSources[Index]).TranslatedFilename:=AValue;
end;

procedure TSourceMap.Sort;
var
  i: Integer;
begin
  if FSorted then exit;
  FItems.Sort(@CompareSegmentWithGeneratedLineCol);
  for i:=0 to Count-1 do
    Items[i].Index:=i;
  FSorted:=true;
end;

function TSourceMap.GetItems(Index: integer): TSourceMapSegment;
begin
  Result:=TSourceMapSegment(FItems[Index]);
end;

function TSourceMap.GetSourceContents(Index: integer): String;
begin
  Result:=TSourceMapSrc(FSources[Index]).Source;
end;

function TSourceMap.GetNames(Index: integer): string;
begin
  Result:=FNames[Index];
end;

function TSourceMap.GetSourceFiles(Index: integer): String;
begin
  Result:=TSourceMapSrc(FSources[Index]).Filename;
end;

function TSourceMap.GetSourceTranslatedFiles(Index: integer): String;
begin
  Result:=TSourceMapSrc(FSources[Index]).TranslatedFilename;
end;

constructor TSourceMap.Create(const aGeneratedFilename: string);
begin
  FOptions:=DefaultSourceMapOptions;
  FVersion:=3;
  FNames:=TStringList.Create;
  FNameToIndex:=TStringToIndex.Create;
  FItems:=TFPList.Create;
  FSources:=TFPList.Create;
  FSourceToIndex:=TStringToIndex.Create;
  GeneratedFilename:=aGeneratedFilename;
  FSorted:=true;
end;

destructor TSourceMap.Destroy;
begin
  Clear;
  FreeAndNil(FSourceToIndex);
  FreeAndNil(FSources);
  FreeAndNil(FItems);
  FreeAndNil(FNameToIndex);
  FreeAndNil(FNames);
  inherited Destroy;
end;

procedure TSourceMap.Clear;
var
  i: Integer;
begin
  FGeneratedFilename:='';
  FSourceToIndex.Clear;
  for i:=0 to FSources.Count-1 do
    TObject(FSources[i]).{$ifdef pas2js}Destroy{$else}Free{$endif};
  FSources.Clear;
  for i:=0 to FItems.Count-1 do
    TObject(FItems[i]).{$ifdef pas2js}Destroy{$else}Free{$endif};
  FItems.Clear;
  FNameToIndex.Clear;
  FNames.Clear;
  FSourceRoot:='';
  FSorted:=true;
end;

function TSourceMap.AddMapping(GeneratedLine: integer; GeneratedCol: integer;
  const SourceFile: string; SrcLine: integer; SrcCol: integer;
  const Name: String): TSourceMapSegment;

  procedure RaiseInvalid(Msg: string);
  begin
    raise EJSSourceMap.CreateFmt('%s (GeneratedLine=%d GeneratedCol=%d SrcFile="%s" SrcLine=%d SrcCol=%d Name="%s")',
      [Msg,GeneratedLine,GeneratedCol,SourceFile,SrcLine,SrcCol,Name]);
  end;

var
  NodeCnt: Integer;
  OtherNode: TSourceMapSegment;
begin
  {$IFDEF VerboseSrcMap}
  writeln('TSourceMap.AddMapping Gen:Line=',GeneratedLine,',Col=',GeneratedCol,
    ' Src:File=',ExtractFileName(SourceFile),',Line=',SrcLine,',Col=',SrcCol,' Name=',Name);
  {$ENDIF}
  if GeneratedLine<1 then
    RaiseInvalid('invalid GeneratedLine');
  if GeneratedCol<0 then
    RaiseInvalid('invalid GeneratedCol');
  if SourceFile='' then
    begin
    if Count=0 then
      RaiseInvalid('missing source file');
    if SrcLine<>1 then
      RaiseInvalid('invalid SrcLine');
    if SrcCol<>0 then
      RaiseInvalid('invalid SrcCol');
    if Name<>'' then
      RaiseInvalid('invalid Name');
    end
  else
    begin
    if SrcLine<1 then
    begin
      if (SrcLine<0) or not (smoAllowSrcLine0 in Options) then
        RaiseInvalid('invalid SrcLine');
    end;
    if SrcCol<0 then
      RaiseInvalid('invalid SrcCol');
    end;

  // Note: same line/col is allowed
  NodeCnt:=Count;
  if (NodeCnt>0) then
    begin
    OtherNode:=Items[NodeCnt-1];
    if (OtherNode.GeneratedLine>GeneratedLine)
        or ((OtherNode.GeneratedLine=GeneratedLine)
          and (OtherNode.GeneratedColumn>GeneratedCol)) then
      begin
      if smoAddMonotonous in FOptions then
        RaiseInvalid('GeneratedLine/Col not monotonous')
      else
        FSorted:=false;
      end;
    end;

  // add
  Result:=TSourceMapSegment.Create;
  Result.Index:=FItems.Count;
  Result.GeneratedLine:=GeneratedLine;
  Result.GeneratedColumn:=GeneratedCol;
  if SourceFile='' then
    Result.SrcFileIndex:=-1
  else
    Result.SrcFileIndex:=IndexOfSourceFile(SourceFile,true);
  Result.SrcLine:=SrcLine;
  Result.SrcColumn:=SrcCol;
  if Name<>'' then
    Result.NameIndex:=IndexOfName(Name,true)
  else
    Result.NameIndex:=-1;
  FItems.Add(Result);
end;

function TSourceMap.CreateMappings: String;

{$ifdef pas2js}
var
  buf: TJSArray;

  procedure AddStr(const s: string); inline;
  begin
    buf.push(s);
  end;

  procedure AddChar(c: char); inline;
  begin
    buf.push(c);
  end;
{$else}
var
  buf: TMemoryStream;

  procedure AddStr(const s: string);
  begin
    if s<>'' then
      buf.Write(s[1],length(s)*sizeof(char));
  end;

  procedure AddChar(c: char);
  begin
    buf.Write(c,sizeof(char));
  end;
{$endif}

var
  i, LastGeneratedLine, LastGeneratedColumn, j, LastSrcFileIndex, LastSrcLine,
    LastSrcColumn, SrcLine, LastNameIndex: Integer;
  Item: TSourceMapSegment;
begin
  Result:='';
  LastGeneratedLine:=1;
  LastGeneratedColumn:=0;
  LastSrcFileIndex:=0;
  LastSrcLine:=0;
  LastSrcColumn:=0;
  LastNameIndex:=0;
  {$ifdef pas2js}
  buf:=TJSArray.new;
  {$else}
  buf:=TMemoryStream.Create;
  {$endif}
  try
    for i:=0 to Count-1 do
      begin
      Item:=Items[i];
      if LastGeneratedLine<Item.GeneratedLine then
        begin
        // new line
        LastGeneratedColumn:=0; // column is reset every generated line
        for j:=LastGeneratedLine+1 to Item.GeneratedLine do
          begin
          AddChar(';');
          if (smoAutoLineStart in FOptions)
              and ((j<Item.GeneratedLine) or (Item.GeneratedColumn>0)) then
            begin
            // repeat mapping at start of line
            // column 0
            AddStr(EncodeBase64VLQ(0-LastGeneratedColumn));
            LastGeneratedColumn:=0;
            // same src file index
            AddStr(EncodeBase64VLQ(0));
            // same src line
            AddStr(EncodeBase64VLQ(0));
            // same src column
            AddStr(EncodeBase64VLQ(0));
            if j=Item.GeneratedLine then
              AddChar(',');
            end;
          end;
        LastGeneratedLine:=Item.GeneratedLine;
        end
      else if i>0 then
        begin
        // not the first segment
        if (LastGeneratedLine=Item.GeneratedLine)
            and (LastGeneratedColumn=Item.GeneratedColumn) then
          continue;
        AddChar(',');
        end;
      // column diff
      //writeln('TSourceMap.CreateMappings Seg=',i,' Gen:Line=',LastGeneratedLine,',Col=',Item.GeneratedColumn,' Src:File=',Item.SrcFileIndex,',Line=',Item.SrcLine,',Col=',Item.SrcColumn,' Name=',Item.NameIndex);
      AddStr(EncodeBase64VLQ(Item.GeneratedColumn-LastGeneratedColumn));
      LastGeneratedColumn:=Item.GeneratedColumn;

      if Item.SrcFileIndex<0 then
        continue; // no source -> segment length 1
      // src file index diff
      AddStr(EncodeBase64VLQ(Item.SrcFileIndex-LastSrcFileIndex));
      LastSrcFileIndex:=Item.SrcFileIndex;
      // src line diff
      SrcLine:=Item.SrcLine-1; // 0 based in version 3
      AddStr(EncodeBase64VLQ(SrcLine-LastSrcLine));
      LastSrcLine:=SrcLine;
      // src column diff
      AddStr(EncodeBase64VLQ(Item.SrcColumn-LastSrcColumn));
      LastSrcColumn:=Item.SrcColumn;
      // name index
      if Item.NameIndex<0 then
        continue; // no name -> segment length 4
      AddStr(EncodeBase64VLQ(Item.NameIndex-LastNameIndex));
      LastNameIndex:=Item.NameIndex;
      end;
    {$ifdef pas2js}
    Result:=buf.join('');
    {$else}
    SetLength(Result,buf.Size);
    if Result<>'' then
      Move(buf.Memory^,Result[1],buf.Size);
    {$endif}
  finally
    {$ifdef pas2js}
    {$else}
    buf.Free;
    {$endif}
  end;
end;

procedure TSourceMap.ParseMappings(const Mapping: String);
const
  MaxInt = High(integer) div 2;
{$ifdef UsePChar}
var
  p: PChar;

  function Decode: NativeInt; inline;
  begin
    Result:=DecodeBase64VLQ(p);
  end;

  procedure E(const Msg: string);
  begin
    raise EJSSourceMap.CreateFmt(Msg,[PtrUInt(p-PChar(Mapping))+1]);
  end;
{$else}
var
  p: integer;

  function Decode: NativeInt; inline;
  begin
    Result:=DecodeBase64VLQ(Mapping,p);
  end;

  procedure E(const Msg: string);
  begin
    raise EJSSourceMap.CreateFmt(Msg,[p]);
  end;
{$endif}
var
  GeneratedLine, LastColumn, Column, LastSrcFileIndex, LastSrcLine,
    LastSrcColumn, LastNameIndex, SrcFileIndex, SrcLine, SrcColumn,
    NameIndex, l: Integer;
  ColDiff, SrcFileIndexDiff, SrcLineDiff, SrcColumnDiff,
    NameIndexDiff: NativeInt;
  Segment: TSourceMapSegment;
begin
  l:=length(Mapping);
  if l=0 then exit;
  p:={$ifdef UsePChar}PChar(Mapping){$else}1{$endif};
  GeneratedLine:=1;
  LastColumn:=0;
  LastSrcFileIndex:=0;
  LastSrcLine:=0;
  LastSrcColumn:=0;
  LastNameIndex:=0;
  while {$ifdef UsePChar}true{$else}p<=l{$endif} do
    begin
    case {$ifdef UsePChar}p^{$else}Mapping[p]{$endif} of
    {$ifdef UsePChar}
    #0:
      if p-PChar(Mapping)=length(Mapping) then
        exit
      else
        E('unexpected #0 at %d');
    {$endif}
    ',':
      begin
      // next segment
      inc(p);
      end;
    ';':
      begin
      // next line
      inc(GeneratedLine);
      LastColumn:=0;
      inc(p);
      end;
    else
      begin
      ColDiff:=Decode;
      if (ColDiff>MaxInt) or (ColDiff<-MaxInt) then
        E('column out of range at %d');
      Column:=LastColumn+integer(ColDiff);
      if (Column>MaxInt) or (Column<-MaxInt) then
        E('column out of range at %d');
      LastColumn:=Column;

      Segment:=TSourceMapSegment.Create;
      Segment.Index:=FItems.Count;
      FItems.Add(Segment);
      Segment.GeneratedLine:=GeneratedLine;
      Segment.GeneratedColumn:=Column;
      Segment.SrcFileIndex:=-1;
      Segment.NameIndex:=-1;
      if {$ifdef UsePChar}not (p^ in [',',';',#0]){$else}(p<=l) and not (Mapping[p] in [',',';']){$endif} then
        begin
        // src file index
        SrcFileIndexDiff:=Decode;
        if (SrcFileIndexDiff>MaxInt) or (SrcFileIndexDiff<-MaxInt) then
          E('src file index out of range at %d');
        SrcFileIndex:=LastSrcFileIndex+integer(SrcFileIndexDiff);
        if (SrcFileIndex<0) or (SrcFileIndex>=SourceCount) then
          E('src file index out of range at %d');
        LastSrcFileIndex:=SrcFileIndex;
        Segment.SrcFileIndex:=SrcFileIndex;
        // src line
        SrcLineDiff:=Decode;
        if (SrcLineDiff>MaxInt) or (SrcLineDiff<-MaxInt) then
          E('src line out of range at %d');
        SrcLine:=LastSrcLine+integer(SrcLineDiff);
        if (SrcLine>MaxInt) or (SrcLine<-MaxInt) then
          E('src line out of range at %d');
        LastSrcLine:=SrcLine;
        Segment.SrcLine:=SrcLine+1; // lines are stored 0-based
        // src column
        SrcColumnDiff:=Decode;
        if (SrcColumnDiff>MaxInt) or (SrcColumnDiff<-MaxInt) then
          E('src column out of range at %d');
        SrcColumn:=LastSrcColumn+integer(SrcColumnDiff);
        if (SrcColumn>MaxInt) or (SrcColumn<-MaxInt) then
          E('src column out of range at %d');
        LastSrcColumn:=SrcColumn;
        Segment.SrcColumn:=SrcColumn;
        if {$ifdef UsePChar}not (p^ in [',',';',#0]){$else}(p<=l) and not (Mapping[p] in [',',';']){$endif} then
          begin
          // name index
          NameIndexDiff:=Decode;
          if (NameIndexDiff>MaxInt) or (NameIndexDiff<-MaxInt) then
            E('name index out of range at %d');
          NameIndex:=LastNameIndex+integer(NameIndexDiff);
          if (NameIndex<0) or (NameIndex>=NameCount) then
            E('name index out of range at %d');
          LastNameIndex:=NameIndex;
          Segment.NameIndex:=NameIndex;
          end;
        end;
      end;
    end;
    end;
end;

function TSourceMap.ToJSON: TJSONObject;
var
  Obj: TJSONObject;
  i: Integer;
  Arr: TJSONArray;
  Mappings: String;
begin
  Result:=nil;
  Mappings:=CreateMappings;

  Obj:=TJSONObject.Create;
  try
    // "version" - integer
    Obj.Add('version',Version);

    // "file" - GeneratedFilename
    if GeneratedFilename<>'' then
      Obj.Add('file',GeneratedFilename);

    // "sourceRoot" - SourceRoot
    if SourceRoot<>'' then
      Obj.Add('sourceRoot',SourceRoot);

    // "sources" - array of filenames
    Arr:=TJSONArray.Create;
    Obj.Add('sources',Arr);
    for i:=0 to SourceCount-1 do
      Arr.Add(SourceTranslatedFiles[i]);

    // "sourcesContent" - array of source content: null or source as string
    // only needed if there is a source
    i:=SourceCount-1;
    while i>=0 do
      if SourceContents[i]='' then
        dec(i)
      else
        begin
        // there is a source -> add array
        Arr:=TJSONArray.Create;
        Obj.Add('sourcesContent',Arr);
        for i:=0 to SourceCount-1 do
          if SourceContents[i]='' then
            Arr.Add(TJSONNull.Create)
          else
            Arr.Add(SourceContents[i]);
        break;
        end;

    // "names" - array of names
    Arr:=TJSONArray.Create;
    Obj.Add('names',Arr);
    for i:=0 to NameCount-1 do
      Arr.Add(Names[i]);

    // "mappings" - string
    Obj.Add('mappings',Mappings);

    Result:=Obj;
  finally
    if Result=nil then
      Obj.Free;
  end;
end;

function TSourceMap.ToString: string;
var
  Obj: TJSONObject;
begin
  Obj:=ToJSON;
  try
    if smoSafetyHeader in Options then
      Result:=DefaultSrcMapHeader+Obj.AsJSON
    else
      Result:=Obj.AsJSON;
  finally
    Obj.Free;
  end;
end;

procedure TSourceMap.LoadFromJSON(Obj: TJSONObject);
var
  aVersion, i, j: integer;
  Arr: TJSONArray;
  Data: TJSONData;
  aFilename, aName: String;
  aMappings: String;
begin
  // Note: does not support sections yet
  Clear;

  // "version" - integer
  aVersion:=Obj.Get('version',0);
  if aVersion<>Version then
    raise EJSSourceMap.CreateFmt('unsupported version %d',[aVersion]);

  // "file" - GeneratedFilename
  GeneratedFilename:=String(Obj.Get('file',''));

  // "sourceRoot" - SourceRoot
  SourceRoot:=Obj.Get('sourceRoot','');

  // "sources" - array of filenames
  Arr:=nil;
  if not Obj.Find('sources',Arr) then
    raise EJSSourceMap.Create('missing sources array');
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if not (Data is TJSONString) then
      raise EJSSourceMap.CreateFmt('sources must string, but found %s',[Data.ClassName]);
    aFilename:=String(TJSONString(Data).AsString);
    j:=IndexOfSourceFile(aFilename,true);
    if j<>i then
      raise EJSSourceMap.CreateFmt('duplicate source file "%s" at %d',[aFilename,i]);
    end;

  // optional: "sourcesContent" - array of sources
  Arr:=nil;
  if Obj.Find('sourcesContent',Arr) then
    begin
    if Arr.Count<>SourceCount then
      raise EJSSourceMap.CreateFmt('number of elements in sources %d mismatch sourcesContent %d',[SourceCount,Arr.Count]);
    for i:=0 to Arr.Count-1 do
      begin
      Data:=Arr[i];
      if (Data is TJSONString) then
        SourceContents[i]:=String(TJSONString(Data).AsString)
      else if Data is TJSONNull then
      else
        raise EJSSourceMap.CreateFmt('sourcesContent[%d] must be string',[i]);
      end;
    end;

  // optional: "names" - array of strings
  Arr:=nil;
  if Obj.Find('names',Arr) then
    for i:=0 to Arr.Count-1 do
      begin
      Data:=Arr[i];
      if not (Data is TJSONString) then
        raise EJSSourceMap.CreateFmt('names must string, but found %s',[Data.ClassName]);
      aName:=String(TJSONString(Data).AsString);
      j:=IndexOfName(aName,true);
      if j<>i then
        raise EJSSourceMap.CreateFmt('duplicate name "%s" at %d',[aName,i]);
      end;

  // "mappings" - string
  aMappings:=Obj.Get('mappings','');
  ParseMappings(aMappings);
end;

procedure TSourceMap.SaveToStream(aStream: TFPJSStream);
var
  Obj: TJSONObject;
begin
  Obj:=ToJSON;
  try
    if smoSafetyHeader in Options then
      begin
      {$ifdef pas2js}
      aStream.push(DefaultSrcMapHeader);
      {$else}
      aStream.Write(DefaultSrcMapHeader[1],length(DefaultSrcMapHeader));
      {$endif}
      end;
    Obj.DumpJSON(aStream);
  finally
    Obj.Free;
  end;
end;

{$ifdef HasStreams}
procedure TSourceMap.LoadFromStream(aStream: TStream);
var
  s: string;
  P: TJSONParser;
  Data: TJSONData;
begin
  s:='';
  SetLength(s,aStream.Size-aStream.Position);
  if s<>'' then
    aStream.Read(s[1],length(s));
  if LeftStr(s,4)=')]}''' then
    Delete(s,1,4)
  else if LeftStr(s,3)=')]}' then
    Delete(s,1,3);
  P:=TJSONParser.Create(s,[joUTF8]);
  try
    Data:=P.Parse;
    if not (Data is TJSONObject) then
      raise EJSSourceMap.Create('source map must be a JSON object');
    LoadFromJSON(TJSONObject(Data));
  finally
    P.Free;
  end;
end;

procedure TSourceMap.SaveToFile(Filename: string);
var
  TheStream: TMemoryStream;
begin
  TheStream:=TMemoryStream.Create;
  try
    SaveToStream(TheStream);
    TheStream.Position:=0;
    TheStream.SaveToFile(Filename);
  finally
    TheStream.Free;
  end;
end;

procedure TSourceMap.LoadFromFile(Filename: string);
var
  TheStream: TMemoryStream;
begin
  TheStream:=TMemoryStream.Create;
  try
    TheStream.LoadFromFile(Filename);
    TheStream.Position:=0;
    LoadFromStream(TheStream);
  finally
    TheStream.Free;
  end;
end;
{$endif}

function TSourceMap.IndexOfName(const Name: string; AddIfNotExists: boolean
  ): integer;
begin
  Result:=FNameToIndex.FindValue(Name);
  if (Result>=0) or not AddIfNotExists then exit;
  Result:=FNames.Count;
  FNames.Add(Name);
  FNameToIndex.Add(Name,Result);
end;

function TSourceMap.IndexOfSourceFile(const SrcFile: string;
  AddIfNotExists: boolean): integer;
var
  Src: TSourceMapSrc;
begin
  Result:=FSourceToIndex.FindValue(SrcFile);
  if (Result>=0) or not AddIfNotExists then exit;
  Src:=TSourceMapSrc.Create;
  Src.Filename:=SrcFile;
  Src.TranslatedFilename:=SrcFile;
  Result:=FSources.Count;
  FSources.Add(Src);
  FSourceToIndex.Add(SrcFile,Result);
end;

function TSourceMap.IndexOfSegmentAt(GeneratedLine, GeneratedCol: integer
  ): integer;
var
  l, r, m: Integer;
  aSeg: TSourceMapSegment;
begin
  Sort;
  l:=0;
  r:=Count-1;
  aSeg:=nil;
  while l<=r do
    begin
    m:=(l+r) div 2;
    aSeg:=Items[m];
    if aSeg.GeneratedLine<GeneratedLine then
      l:=m+1
    else if aSeg.GeneratedLine>GeneratedLine then
      r:=m-1
    else if aSeg.GeneratedColumn<GeneratedCol then
      l:=m+1
    else if aSeg.GeneratedColumn>GeneratedCol then
      r:=m-1
    else
      begin
      // exact match found
      Result:=m;
      // -> return the leftmost exact match
      while Result>0 do
        begin
        aSeg:=Items[Result-1];
        if (aSeg.GeneratedLine<>GeneratedLine)
            or (aSeg.GeneratedColumn<>GeneratedCol) then
          exit;
        dec(Result);
        end;
      exit;
      end;
    end;
  // no exact match found
  if aSeg=nil then
    exit(-1);
  // return the next lower. Note: there may be no such segment
  if (aSeg.GeneratedLine>GeneratedLine)
      or ((aSeg.GeneratedLine=GeneratedLine) and (aSeg.GeneratedColumn>GeneratedCol)) then
    dec(m);
  Result:=m;
end;

function TSourceMap.Count: integer;
begin
  Result:=FItems.Count;
end;

function TSourceMap.SourceCount: integer;
begin
  Result:=FSources.Count;
end;

function TSourceMap.NameCount: integer;
begin
  Result:=FNames.Count;
end;

end.

