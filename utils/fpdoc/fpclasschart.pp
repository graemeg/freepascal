{

    FPClass chart -  Free Pascal class chart generation tool
    Copyright (c) 2008 - Michael Van Canneyt, michael@freepascal.org

    * Free Pascal class chart generation tool

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}
{$h+}

program fpclasschart;

uses
  SysUtils, Classes, Typinfo, Gettext, dom, xmlread,
  dGlobals, PasTree, PParser,PScanner, xmlwrite, fpdocclasstree;

resourcestring
  STitle = 'fpClassTree - Create class tree from pascal sources';
  SVersion = 'Version %s [%s]';
  SCopyright = '(c) 2008 - Michael Van Canneyt, michael@freepascal.org';
  SCmdLineInvalidOption = 'Ignoring unknown option "%s"';
  SDone = 'Done.';
  SSkipMerge = 'Cannot merge %s into %s tree.';
  SErrNoSuchMergeFile = 'Merge file %s does not exist.';
  SMergedFile = 'Merged %d classes from file %s.';
  SClassesAdded = 'Added %d classes from %d files.';

type

  { TClassTreeEngine }


  TClassTreeEngine = class(TFPDocEngine)
  Private
    FTree : TClassTreeBuilder;
    FObjects : TStringList;
  public
    Constructor Create(AClassTree : TXMLDocument; AObjectKindSet : TPasObjKindSet);
    Destructor Destroy; override;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility :TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement; override;
  end;

  { TClassChartFormatter }
  TClassMode = (cmNormal,cmSubClass,cmheadClass,cmFirstClass);
  TClassChartFormatter = Class (TObject)
  private
    FClassMode: TClassMode;
    FClassTree: TXMLDocument;
    FCurrentColCount: Integer;
    FCurrentRowCount: Integer;
    FFileName: String;
    FLargeHeadClassObjects: TStrings;
    FLevel: Integer;
    FMaxObjectsPerColumn: Integer;
    FStartColumnObjects: TStrings;
  Protected
    procedure FirstClass(E : TDomElement); virtual;
    procedure DoEmitClass(E : TDomElement); virtual;
    procedure DoHeadClass(E: TDomElement); virtual;
    procedure DoNextColumn(E: TDomElement); virtual;
    procedure EndSubClass(E: TDomElement; HasSiblings : Boolean); virtual;
    procedure StartSubClass(E: TDomElement); virtual;
    Procedure StartChart; virtual;
    Procedure EndChart; virtual;
    procedure EmitClass(E : TDomElement; HasSiblings : Boolean);
  Public
    Constructor Create (AXML : TXMLDocument); virtual;
    Destructor Destroy; override;
    Procedure CreateChart;
    Property CurrentColCount : Integer Read FCurrentColCount;
    Property CurrentRowCount : Integer Read FCurrentRowCount;
    Property ClassTree : TXMLDocument Read FClassTree;
    Property Level : Integer Read FLevel Write FLevel;
    Property ClassMode : TClassMode Read FClassMode;
  Published
    Property FileName : String Read FFileName Write FFilename;
    Property StartColumnObjects : TStrings Read FStartColumnObjects;
    Property LargeHeadClassObjects : TStrings Read FLargeHeadClassObjects;
    Property MaxObjectsPerColumn : Integer Read FMaxObjectsPerColumn Write FMaxObjectsPerColumn;
  end;

{ TClassTreeBuilder }

{ TChartFormatter }

constructor TClassChartFormatter.Create(AXML: TXMLDocument);
begin
  FClassTree:=AXML;
  MaxObjectsPerColumn:=60;
  FStartColumnObjects:=TStringList.Create;
  FLargeHeadClassObjects:=TStringList.Create;
  FLargeHeadClassObjects.Add('TPersistent');
  FLargeHeadClassObjects.Add('TComponent');
end;

destructor TClassChartFormatter.Destroy;
begin
  FreeAndNil(FStartColumnObjects);
  FreeAndNil(FLargeHeadClassObjects);
  Inherited;
end;

procedure TClassChartFormatter.CreateChart;

Var
  N : TDomNode;
  E : TDomElement;
  I : Integer;
  L : TFPList;

begin
  (FStartColumnObjects as TStringList).Sorted:=False;
  (FLargeHeadClassObjects as TStringList).Sorted:=False;
  StartChart;
  try
    N:=FClassTree.DocumentElement.FirstChild;
    FCurrentColCount:=0;
    FCurrentRowCount:=0;
    FLevel:=0;
    L:=TFPList.Create;
    try
      While (N<>nil) do
        begin
        If (N.NodeType=ELEMENT_NODE) then
          L.Add(N);
        N:=N.NextSibling;
        end;
      If (L.Count>0) then
        begin
        FirstClass(TDomElement(L[0]));
        For I:=0 to L.Count-1 do
          EmitClass(TDomElement(L[i]),I<L.Count-1);
        end;
    finally
      L.Free;
    end;
    L:=TFPList.Create;
    try
      For I:=0 to FLargeHeadClassObjects.Count-1 do
        If Assigned(FLargeHeadClassObjects.Objects[i]) then
          L.Add(FLargeHeadClassObjects.Objects[i]);
      FLargeHeadClassObjects.Clear;
      For I:=0 to L.Count-1 do
        begin
        E:= TDomElement(L[i]);
        DoHeadClass(E);
        EmitClass(E,I<L.Count-1);
        end;
    finally
      L.Free;
    end;
  finally
    EndChart;
  end;
end;

procedure TClassChartFormatter.FirstClass(E : TDomElement);

begin
  FClassMode:=cmFirstClass;
end;

procedure TClassChartFormatter.DoEmitClass(E : TDomElement);
begin
  //Reset
  FClassMode:=cmNormal;
end;

procedure TClassChartFormatter.DoHeadClass(E : TDomElement);
begin
  DoNextColumn(E);
  FClassMode:=cmHeadClass;
  // Do nothing
end;

procedure TClassChartFormatter.StartSubClass(E : TDomElement);
begin
  FClassMode:=cmSubClass;
end;

procedure TClassChartFormatter.EndSubClass(E : TDomElement; HasSiblings : Boolean);
begin
  FClassMode:=cmNormal;
end;

procedure TClassChartFormatter.DoNextColumn(E : TDomElement);

begin
  Inc(FCurrentColCount);
  FCurrentRowCount:=0;
end;

procedure TClassChartFormatter.StartChart;
begin
  // Do nothing
end;

procedure TClassChartFormatter.EndChart;
begin
  // Do nothing
end;

procedure TClassChartFormatter.EmitClass(E : TDomElement; HasSiblings: Boolean);

Var
  N : TDomNode;
  I : Integer;
  L : TFPList;

begin
  Inc(Flevel);
  try
    I:=FStartColumnObjects.IndexOf(E.NodeName);
    if (-1<>I) or ((FCurrentRowCount>MaxObjectsPerColumn) and (FLevel=2)) then
      DoNextColumn(E)
    else
      begin
      I:=FLargeHeadClassObjects.IndexOf(E.NodeName);
      if (-1<>I) then
        begin
        FLargeHeadClassObjects.Objects[i]:=E;
        Exit; // Must be picked up later.
        end;
      end;
    DoEmitClass(E);
    N:=E.FirstChild;
    L:=TFPList.Create;
    try
      While (N<>Nil) do
        begin
        if (N.NodeType=ELEMENT_NODE) then
           L.Add(N);
        N:=N.NextSibling;
        end;
      If L.Count>0 then
        begin
        StartSubClass(TDomElement(L[0]));
        For I:=0 to L.Count-1 do
          begin
          EmitClass(TDomElement(L[i]),I<L.Count-1);
          FClassMode:=cmNormal;
          end;
        EndSubClass(E,HasSiblings);
        end;
    Finally
      L.Free;
    end;
    Inc(FCurrentRowCount);
  finally
    Dec(Flevel);
  end;
end;

Type

  { TPostScriptClassChartFormatter }

  TPostScriptClassChartFormatter = Class(TClassChartFormatter)
    FFile : Text;
    FMode : TClassMode;
    FIndent : Integer;
    Procedure EmitLine(S : String);
  Protected
    procedure DoEmitClass(E : TDomElement); override;
    procedure DoNextColumn(E: TDomElement); override;
    procedure DoHeadClass(E: TDomElement); override;
    procedure StartSubClass(E: TDomElement); override;
    procedure EndSubClass(E: TDomElement; HasSiblings : Boolean); override;
    Procedure StartChart; override;
    Procedure EndChart; override;
  end;

{ TPostScriptClassChartFormatter }

procedure TPostScriptClassChartFormatter.EmitLine(S: String);
begin
  Writeln(FFile,StringofChar(' ',Findent*2),S);
end;

procedure TPostScriptClassChartFormatter.DoEmitClass(E: TDomElement);
begin
  Case ClassMode of
    cmFirstClass : EmitLine(Format('(%s) Ready drawlargebox',[E.NodeName]));
    cmNormal     : EmitLine(Format('(%s) Ready newclass',[E.NodeName]));
    cmSubClass   : EmitLine(Format('(%s) Ready newchildclass',[E.NodeName]));
    cmHeadClass  : EmitLine(Format('(%s) Ready newlargeheadclass',[E.NodeName]));
  end;
end;

procedure TPostScriptClassChartFormatter.DoNextColumn(E: TDomElement);
begin
  Inherited;
  FIndent:=0;
  EmitLine('newcolumn');

end;

procedure TPostScriptClassChartFormatter.DoHeadClass(E: TDomElement);
begin
//  DoNextColumn(E);
  inherited DoHeadClass(E);
end;


procedure TPostScriptClassChartFormatter.EndSubClass(E: TDomElement; HasSiblings : Boolean);
begin
  if HasSiblings then
    EmitLine('onelevelback')
  else
    EmitLine('onelevelbackempty');
  If FIndent>0 then
    Dec(Findent);
end;

procedure TPostScriptClassChartFormatter.StartSubClass(E: TDomElement);
begin
  inherited StartSubClass(E);
  Inc(Findent);
end;

procedure TPostScriptClassChartFormatter.StartChart;
begin
  Assign(FFile,FileName);
  Rewrite(FFile);
end;

procedure TPostScriptClassChartFormatter.EndChart;
begin
  Close(FFile);
end;

type
  { TGraphVizClassChartFormatter }

  TGraphVizClassChartFormatter = class(TClassChartFormatter)
    FFile : Text;
    FMode : TClassMode;
    FIndent : integer;
    Procedure EmitLine(S : string);
  Protected
    procedure DoEmitClass(E : TDomElement); override;
    procedure DoNextColumn(E: TDomElement); override;
    procedure DoHeadClass(E: TDomElement); override;
    procedure StartSubClass(E: TDomElement); override;
    procedure EndSubClass(E: TDomElement; HasSiblings : Boolean); override;
    Procedure StartChart; override;
    Procedure EndChart; override;
  end;

  { TGraphVizClassChartFormatter }

  procedure TGraphVizClassChartFormatter.EmitLine(S: String);
  begin
    Writeln(FFile,StringofChar(' ',Findent*2),S);
  end;

  procedure TGraphVizClassChartFormatter.DoEmitClass(E: TDomElement);
  begin
    Case ClassMode of
      cmFirstClass : EmitLine(Format('%s -> %s', [E.ParentNode.NodeName, E.NodeName]));
      cmNormal     : EmitLine(Format('%s -> %s', [E.ParentNode.NodeName, E.NodeName]));
      cmSubClass   : EmitLine(Format('%s -> %s', [E.ParentNode.NodeName, E.NodeName]));
      cmHeadClass  : EmitLine(Format('%s -> %s', [E.ParentNode.NodeName, E.NodeName]));
    end;
  end;

  procedure TGraphVizClassChartFormatter.DoNextColumn(E: TDomElement);
  begin
    Inherited;
    FIndent:=0;
  end;

  procedure TGraphVizClassChartFormatter.DoHeadClass(E: TDomElement);
  begin
  //  DoNextColumn(E);
    inherited DoHeadClass(E);
  end;


  procedure TGraphVizClassChartFormatter.EndSubClass(E: TDomElement; HasSiblings : Boolean);
  begin
    If FIndent>0 then
      Dec(Findent);
  end;

  procedure TGraphVizClassChartFormatter.StartSubClass(E: TDomElement);
  begin
    inherited StartSubClass(E);
    Inc(Findent);
  end;

  procedure TGraphVizClassChartFormatter.StartChart;
  begin
    Assign(FFile,FileName);
    Rewrite(FFile);
    EmitLine('digraph G {');
  end;

  procedure TGraphVizClassChartFormatter.EndChart;
  begin
    EmitLine('}');
    Close(FFile);
  end;


Type
  TOutputFormat = (ofXML,ofPostscript, ofGraphViz);

Var
  OutputFormat : TOutputFormat = ofXML;

const
  OSTarget: String = {$I %FPCTARGETOS%};
  CPUTarget: String = {$I %FPCTARGETCPU%};
  FPCVersion: String = {$I %FPCVERSION%};
  FPCDate: String = {$I %FPCDATE%};


function TClassTreeEngine.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility : TPasMemberVisibility;
  const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;


begin
  Result := AClass.Create(AName, AParent);
  Result.Visibility:=AVisibility;
  if AClass.InheritsFrom(TPasModule) then
    CurModule := TPasModule(Result);
  If AClass.InheritsFrom(TPasClassType) then
    begin
    FObjects.AddObject(AName,Result);
   // Writeln('Added : ',AName);
    end;
end;

Constructor TClassTreeEngine.Create(AClassTree : TXMLDocument; AObjectKindSet : TPasObjKindSet);
begin
  Inherited Create;
  FPackage:=TPasPackage.Create('dummy',Nil);
  FTree:=TClassTreeBuilder.Create(Self,FPackage,AObjectKindSet);
  FObjects:=TStringList.Create;
end;

destructor TClassTreeEngine.Destroy;
begin
  FreeAndNil(FTree);
  FreeAndNil(FPackage);
  FreeAndNil(FObjects);
  inherited Destroy;
end;


{ ---------------------------------------------------------------------
  Main program. Document all units.    
  ---------------------------------------------------------------------}

Function MergeNodes(Doc : TXMLDocument;Dest,Source : TDomElement) : Integer;

Var
  N : TDomNode;
  S,E : TDomElement;


begin
  Result:=0;
  N:=Source.FirstChild;
  While (N<>Nil) do
    begin
    if (N.NodeType=ELEMENT_NODE) then
      begin
      S:=N as TDomElement;
      E:=Dest.FindNode(N.NodeName) as TDomElement;
      If (E=Nil) then
        begin
        E:=Doc.CreateElement(N.NodeName);
        If S['unit']<>'' then
          E['Unit']:=S['unit'];
        Dest.AppendChild(E);
        Inc(Result);
        end;
      Result:=Result+MergeNodes(Doc,E,S);
      end;
    N:=N.NextSibling;
    end;
end;

Function MergeTrees (Dest,Source : TXMLDocument) : Integer;

Var
  S,D : TDomElement;

begin
  Result:=0;
  D:=Dest.DocumentElement;
  S:=Source.DocumentElement;
  If (S.NodeName=D.NodeName) then
    Result:=MergeNodes(Dest,D,S)
  else
    Writeln(StdErr,Format(SSkipMerge,[S.NodeName,D.NodeName]));
end;

Function MergeTrees (Dest : TXMLDocument; aRootNode : TPasElementNode) : Integer;

Var
  aSrc : TXMLDocument;

  Procedure AppendChildClasses(aParent : TDomElement; aNode : TPasElementNode);

  Var
    El : TDomElement;
    aChild : TPasElementNode;
    I : Integer;
    M : TPasModule;

  begin
    If (ANode=Nil) or (aNode.ChildCount=0)  then exit;
    for I:=0 to aNode.ChildCount-1 do
      begin
      aChild:=aNode.Children[I];
      El:=aSrc.CreateElement(UTF8Decode(aChild.Element.Name));
      M:=aChild.Element.GetModule;
      If M<>Nil then
        EL['unit']:=UTF8Decode(M.Name);
      aParent.AppendChild(El);
      AppendChildClasses(El,aChild);
      end;
  end;

begin
  Result:= 0;
  aSrc:=TXMLDocument.Create();
  try
    aSrc.AppendChild(aSrc.CreateElement('TObject'));
    AppendChildClasses(aSrc.DocumentElement,aRootNode);
    MergeTrees(Dest,aSrc);
    Inc(Result);
  finally
    aSrc.Free;
  end;
end;



Function AnalyseFiles(Const AOutputName : String; InputFiles,MergeFiles : TStrings; AObjectKind : TPasObjKind) : String;


Var
  XML,XML2 : TXMLDocument;
  I,ACount : Integer;
  Engine: TClassTreeEngine;

begin
  Result:='';
  ACount:=0;
  XML:=TXMLDocument.Create;
  Try
    //XML.
    XML.AppendChild(XML.CreateElement('TObject'));
    For I:=0 to MergeFiles.Count-1 do
      begin
      XMl2:=TXMLDocument.Create;
      ReadXMLFile(XML2,MergeFiles[i]);
      try
        ACount:=ACount+MergeTrees(XML,XML2);
        WriteLn(StdErr,Format(SMergedFile,[ACount,MergeFiles[i]]));
      Finally
        FreeAndNil(XML2);
      end;
      end;
    For I:=0 to InputFiles.Count-1 do
      begin
      Engine := TClassTreeEngine.Create(XML,[AObjectKind]);
      Try
        ParseSource(Engine,InputFiles[I],OSTarget,CPUTarget);
        Engine.Ftree.BuildTree(Engine.FObjects);
        ACount:=ACount+MergeTrees(XML,Engine.FTree.RootNode);
      Finally
        FreeAndNil(Engine);
      end;
      end;
    Case OutputFormat of
      ofXML :
        WriteXMlFile(XML,AOutputName);
      ofPostScript :
        With TPostScriptClassChartFormatter.Create(XML) do
          try
            FileName:=AOutputName;
            CreateChart;
          finally
            Free;
          end;
      ofGraphViz :
        With TGraphVizClassChartFormatter.Create(XML) do
          try
            FileName:=AOutputName;
            CreateChart;
          finally
            Free;
          end;
    end;
    Writeln(StdErr,Format(SClassesAdded,[ACount,InputFiles.Count]));
  Finally
    XML.Free;
  end;
end;

{ ---------------------------------------------------------------------
    Option management
  ---------------------------------------------------------------------}


var  
  cmdObjectKind : TPasObjKind;
  InputFiles, 
  MergeFiles : TStringList;
  DocLang : String;
  OutputName: String;

procedure InitOptions;
begin
  InputFiles := TStringList.Create;
  MergeFiles := TStringList.Create;
end;

procedure FreeOptions;

begin
  MergeFiles.Free;
  InputFiles.Free;
end;

{ ---------------------------------------------------------------------
  Usage  
  ---------------------------------------------------------------------}
  
Procedure Usage;

begin
  Writeln('Usage : ',ExtractFileName(Paramstr(0)),' [options]');
  Writeln('Where [options] is one or more of :');
  Writeln(' --merge=filename    Filename with object tree to merge.');
  Writeln(' --help              Emit help.');
  Writeln(' --input=cmdline     Input file to create skeleton for.');
  Writeln('                     Use options are as for compiler.');
  Writeln(' --kind=objectkind   Specify object kind. One of object, class, interface.');
  Writeln(' --lang=language     Use selected language.');
  Writeln(' --output=filename   Send output to file.');
  Writeln(' --format=name       Kind of output to create: XML, PostScript, GraphViz.');
end;

procedure ParseOption(const s: String);

  procedure AddToFileList(List: TStringList; const FileName: String);
  var
    f: Text;
    s: String;
  begin
    if Copy(FileName, 1, 1) = '@' then
    begin
      Assign(f, Copy(FileName, 2, Length(FileName)));
      Reset(f);
      while not EOF(f) do
      begin
        ReadLn(f, s);
        List.Add(s);
      end;
      Close(f);
    end else
      List.Add(FileName);
  end;

var
  i: Integer;
  Cmd, Arg: String;
begin
  cmdObjectKind:=okClass;
  if (s = '-h') or (s = '--help') then
    begin
    Usage;
    Halt(0);
    end;
  i := Pos('=', s);
  if i > 0 then
  begin
    Cmd := Copy(s, 1, i - 1);
    Arg := Copy(s, i + 1, Length(s));
  end else
  begin
    Cmd := s;
    SetLength(Arg, 0);
  end;
  if (Cmd = '-i') or (Cmd = '--input') then
    AddToFileList(InputFiles, Arg)
  else if (Cmd = '-l') or (Cmd = '--lang') then
    DocLang := Arg
  else if (Cmd = '-o') or (Cmd = '--output') then
    OutputName := Arg
  else if (Cmd = '-k') or (Cmd = '--kind') then
    cmdObjectKind:=TPasObjKind(GetEnumValue(TypeInfo(TPasObjKind),'ok'+Arg))
  else if (Cmd = '-f') or (Cmd = '--format') then
    OutputFormat:=TOutputFormat(GetEnumValue(TypeInfo(TOutputFormat),'of'+Arg))
  else if Cmd = '--merge' then
    begin
    if FileExists(Arg) then
      MergeFiles.Add(Arg)
    else
      Writeln(StdErr,Format(SErrNoSuchMergeFile,[arg]));
    end
  else
    begin
    WriteLn(StdErr, Format(SCmdLineInvalidOption, [s]));
    Usage;
    Halt(1);
    end;
end;

Function ParseCommandLine : Integer;

Const
{$IFDEF Unix}
  MoFileTemplate = '/usr/local/share/locale/%s/LC_MESSAGES/makeskel.mo';
{$ELSE}
  MoFileTemplate ='intl/makeskel.%s.mo';
{$ENDIF}

var
  MOFilename: string;
  i: Integer;

begin
  Result:=0;
  if ParamCount=0 then
    begin
      Usage;
      Halt(0);
    end;
  DocLang:='';
  for i := 1 to ParamCount do
    ParseOption(ParamStr(i));
  If (DocLang<>'') then
    begin
    MOFilename:=Format(MOFileTemplate,[DocLang]);
    if FileExists(MOFilename) then
      gettext.TranslateResourceStrings(MoFileName)
    else
      writeln('NOTE: unable to find translation file ',MOFilename);
    // Translate internal documentation strings
    TranslateDocStrings(DocLang);
    end;
end;


{ ---------------------------------------------------------------------
  Main Program  
  ---------------------------------------------------------------------}
  
Procedure Run;
  
var
  E: Integer;

begin
  WriteLn(STitle);
  WriteLn(Format(SVersion, [FPCVersion, FPCDate]));
  WriteLn(SCopyright);
 
  InitOptions;
  Try
    E:=ParseCommandLine;
    If E<>0 then
      Halt(E);
    WriteLn;
    AnalyseFiles(OutputName,InputFiles,MergeFiles,cmdObjectKind);
    WriteLn(StdErr,SDone);
  Finally  
    FreeOptions;
  end;  
end;

Begin
  Run;  
end.

