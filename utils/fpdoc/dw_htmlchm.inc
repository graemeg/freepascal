{%mainunit dw_html}
{$IFDEF chmInterface}
type

  { TCHMHTMLWriter }

  TCHMHTMLWriter = class(THTMLWriter)
  private
    FOutChm: TStream;
    FChm: TChmWriter;
    FTempUncompressed: TStream;
    FTempUncompressedName: String;
<<<<<<< HEAD
<<<<<<< HEAD
    FChmTitle: String;
    FTOCName,
    FIndexName,
    FDefaultPage: String;
    FMakeSearchable,
    FNoBinToc,
    FNoBinIndex,
=======
=======
>>>>>>> origin/fixes_2_2
    FTOCName,
    FIndexName,
    FDefaultPage: String;
    FCSSFile: String;
    FMakeSearchable,
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    FNoBinToc,
    FNoBinIndex,
>>>>>>> origin/fixes_2.4
    FAutoTOC,
    FAutoIndex: Boolean;
    FOtherFiles: String;
    procedure ProcessOptions;
    function RetrieveOtherFiles(const DataName: String; out PathInChm: String; out FileName: String; var Stream: TStream): Boolean;
    procedure LastFileAdded(Sender: TObject);
    procedure GenerateTOC;
    procedure GenerateIndex;
  public
    procedure WriteHTMLPages; override;
    
    function  InterPretOption(const Cmd,Arg : String): boolean; override;

    class procedure Usage(List: TStrings); override;
<<<<<<< HEAD
<<<<<<< HEAD
    Class Function FileNameExtension : String; override;
    Class procedure SplitImport(var AFilename, ALinkPrefix: String); override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
{$ELSE} // implementation

{ TCHMHTMLWriter }

procedure TCHMHTMLWriter.ProcessOptions;
var
  TempStream: TMemoryStream;
begin
  if FDefaultPage = '' then
    FDefaultPage := 'index.html'
  else
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    DoLog('Note: --index-page not assigned. Using default "index.html"');
=======
    WriteLn('Note: --index-page not assigned. Using default "index.html"');
>>>>>>> graemeg/fixes_2_2
=======
    WriteLn('Note: --index-page not assigned. Using default "index.html"');
>>>>>>> origin/fixes_2_2
  end;
  
  if FCSSFile <> '' then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if not FileExists(FCSSFile) Then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Raise Exception.CreateFmt('Can''t find CSS file "%S"',[FCSSFILE]);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
    if not FileExists(FCSSFile) Then
>>>>>>> origin/fixes_2.4
      begin
        Writeln(stderr,'Can''t find CSS file "',FCSSFILE,'"');
        halt(1);
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
    TempStream := TMemoryStream.Create;
    TempStream.LoadFromFile(FCSSFile);
    TempStream.Position := 0;
    FChm.AddStreamToArchive('fpdoc.css', '/', TempStream, True);
    TempStream.Free;
  end;

  FChm.DefaultPage := FDefaultPage;
  
  if FOtherFiles <> '' then
  begin
    FChm.FilesToCompress.LoadFromFile(FOtherFiles);
  end;

  FChm.FullTextSearch := FMakeSearchable;

end;

function TCHMHTMLWriter.RetrieveOtherFiles(const DataName: String; out
  PathInChm: String; out FileName: String; var Stream: TStream): Boolean;
var
  Dir: String;
begin
  if Stream <> nil then
    Stream.Free;
  Stream := TMemoryStream.Create;
  TMemoryStream(Stream).LoadFromFile(DataName);
  FileName := ExtractFileName(DataName);
  
  if ExtractFileDir(DataName) <> '' then
    PathInChm := ExtractRelativepath(GetCurrentDir, ExtractFileDir(DataName))
  else
    PathInChm := '/';
  FixHTMLpath(PathInChm);
  Stream.Position := 0;
end;

procedure TCHMHTMLWriter.LastFileAdded(Sender: TObject);
var
  TmpStream: TMemoryStream;
begin
  TmpStream := TMemoryStream.Create;
  if FAutoTOC then
    GenerateTOC
  else
    if FTOCName <> '' then
    begin
      TmpStream.LoadFromFile(FTOCName);
      TmpStream.Position := 0;
      FChm.AppendTOC(TmpStream);
      TmpStream.Size := 0;
    end;
    
  if FAutoIndex then
    GenerateIndex
  else
    if FIndexName <> '' then
    begin
      TmpStream.LoadFromFile(FIndexName);
      TmpStream.Position := 0;
      FChm.AppendIndex(TmpStream);
    end;
  TmpStream.Free;
<<<<<<< HEAD
<<<<<<< HEAD
  DoLog('Finishing compressing...');
=======
  WriteLn('Finishing compressing...');
>>>>>>> graemeg/fixes_2_2
=======
  WriteLn('Finishing compressing...');
>>>>>>> origin/fixes_2_2
end;

function TOCSort(Item1, Item2: TChmSiteMapItem): Integer;
begin
  Result := CompareText(LowerCase(Item1.Text), LowerCase(Item2.Text));
end;

function GetAlphaItem(AItems: TChmSiteMapItems; AName: String): TChmSiteMapItem;
var
  x: Integer;
begin
  Result := nil;
  for x := 0 to AItems.Count-1 do
  begin
    if AItems.Item[x].Text = AName then
      Exit(AItems.Item[x]);
  end;
  Result := AItems.NewItem;
  Result.Text := AName;
end;
     
procedure TCHMHTMLWriter.GenerateTOC;
var
  TOC: TChmSiteMap;
  Element: TPasElement;
  k: Integer;
  j: Integer;
  i: Integer;
  AModule: TPasModule;
  Member: TPasElement;
  Stream: TMemoryStream;
  TmpItem:  TChmSiteMapItem;
  ObjByUnitItem,
  AlphaObjItem,
   ObjUnitItem,
  RoutinesByUnitItem,
    RoutinesUnitItem,
  AlphaRoutinesItem: TChmSiteMapItem;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  DoLog('Generating Table of contents...');
=======
  WriteLn('Generating Table of contents...');
>>>>>>> graemeg/fixes_2_2
=======
  WriteLn('Generating Table of contents...');
>>>>>>> origin/fixes_2_2
  if Assigned(Package) then
  begin
    Toc := TChmSiteMap.Create(stTOC);
    Stream := TMemoryStream.Create;
    ObjByUnitItem := TOC.Items.NewItem;
    ObjByUnitItem.Text      := 'Classes and Objects, by Unit';
    AlphaObjItem := TOC.Items.NewItem;
    AlphaObjItem.Text       := 'Alphabetical Classes and Objects List';
    RoutinesByUnitItem := TOC.Items.NewItem;
    RoutinesByUnitItem.Text := 'Routines, by Unit';
    AlphaRoutinesItem  := TOC.Items.NewItem;
    AlphaRoutinesItem.Text  := 'Alphabetical Routines List';

    // objects and classes
    for i := 0 to Package.Modules.Count - 1 do
    begin
      AModule := TPasModule(Package.Modules[i]);
<<<<<<< HEAD
<<<<<<< HEAD
      If not assigned(AModule.InterfaceSection) Then
         Continue;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      ObjUnitItem := ObjByUnitItem.Children.NewItem;
      ObjUnitItem.Text := AModule.Name;
      RoutinesUnitItem := RoutinesByUnitItem.Children.NewItem;
      RoutinesUnitItem.Text := AModule.Name;
      for j := 0 to AModule.InterfaceSection.Classes.Count-1 do
      begin
        Element := TPasClassType(AModule.InterfaceSection.Classes[j]);
        // by unit
        TmpItem := ObjUnitItem.Children.NewItem;
        TmpItem.Text := Element.Name;
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(Element, 0));
        
        //alpha
        TmpItem := GetAlphaItem(AlphaObjItem.Children, UpperCase(Copy(Element.Name, 1, 2))).Children.NewItem;
        TmpItem.Text := Element.Name;
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(Element, 0));
        
      end;
      
      // non object procedures and functions
      for j := 0 to AModule.InterfaceSection.Functions.Count-1 do
      begin
        Element := TPasFunctionType(AModule.InterfaceSection.Functions[j]);
        // by unit
        TmpItem := RoutinesUnitItem.Children.NewItem;
        TmpItem.Text := Element.Name;
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(Element, 0));
        
        // alpha
        TmpItem := GetAlphaItem(AlphaRoutinesItem.Children, UpperCase(Element.Name[1])).Children.NewItem;
        TmpItem.Text := Element.Name;
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(Element, 0));
      end;
    end;
  end;
  // cleanup
  for i := ObjByUnitItem.Children.Count-1 downto 0 do
  begin
    if ObjByUnitItem.Children.Item[i].Children.Count = 0 then
      ObjByUnitItem.Children.Delete(i);
  end;

  for i := RoutinesByUnitItem.Children.Count-1 downto 0 do
  begin
    if RoutinesByUnitItem.Children.Item[i].Children.Count = 0 then
      RoutinesByUnitItem.Children.Delete(i);
  end;
  
  for i := TOC.Items.Count-1 downto 0 do
  begin
    if TOC.Items.Item[i].Children.Count = 0 then
      TOC.Items.Delete(i);
  end;
  
  // Sort
  for i := 0 to TOC.Items.Count-1 do
  begin
    TOC.Items.Item[i].Children.Sort(TListSortCompare(@TOCSort));
    for j := 0 to TOC.Items.Item[i].Children.Count-1 do
    begin
      TOC.Items.Item[i].Children.Item[j].Children.Sort(TListSortCompare(@TOCSort));
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  if not fnobintoc then
    fchm.AppendBinaryTOCFromSiteMap(Toc);  
=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
=======

  if not fnobintoc then
    fchm.AppendBinaryTOCFromSiteMap(Toc);  
>>>>>>> origin/fixes_2.4
  TOC.SaveToStream(Stream);
  TOC.Free;

  fchm.AppendTOC(Stream);
  Stream.Free;
end;

type
  TClassMemberType = (cmtProcedure, cmtFunction, cmtConstructor, cmtDestructor,
      cmtInterface, cmtProperty, cmtVariable, cmtUnknown);
  
function ElementType(Element: TPasElement): TClassMemberType;
var
  ETypeName: String;
begin
  Result := cmtUnknown;
  ETypeName := Element.ElementTypeName;
  //overloaded we don't care
  if ETypeName[1] = 'o' then ETypeName := Copy(ETypeName, 11, Length(ETypeName));
  
  if ETypeName[1] = 'f' then Exit(cmtFunction);
  if ETypeName[1] = 'c' then Exit(cmtConstructor);
  if ETypeName[1] = 'v' then Exit(cmtVariable);
  if ETypeName[1] = 'i' then Exit(cmtInterface);
  // the p's
  if ETypeName[4] = 'c' then Exit(cmtProcedure);
  if ETypeName[4] = 'p' then Exit(cmtProperty);
  
end;

procedure TCHMHTMLWriter.GenerateIndex;
var
  Index: TChmSiteMap;
  i, j, k: Integer;
  TmpItem: TChmSiteMapItem;
  ParentItem: TChmSiteMapItem;
  AModule: TPasModule;
  TmpElement: TPasElement;
  ParentElement: TPasElement;
  MemberItem: TChmSiteMapItem;
  Stream: TMemoryStream;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  DoLog('Generating Index...');

  if Assigned(Package) then
  begin
=======
=======
>>>>>>> origin/fixes_2_2
  WriteLn('Generating Index...');

  if Assigned(Package) then
  begin
  try
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Index := TChmSiteMap.Create(stIndex);
    Stream := TMemoryStream.Create;
    for i := 0 to Package.Modules.Count - 1 do
    begin
      AModule := TPasModule(Package.Modules[i]);
<<<<<<< HEAD
<<<<<<< HEAD
      if not assigned(AModule.InterfaceSection) then
        continue;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      ParentItem := Index.Items.NewItem;
      ParentItem.Text := AModule.Name;
      ParentItem.Local := FixHTMLpath(Allocator.GetFilename(AModule, 0));

      //  classes
      for j := 0 to AModule.InterfaceSection.Classes.Count-1 do
      begin
        ParentElement := TPasClassType(AModule.InterfaceSection.Classes[j]);
        ParentItem := Index.Items.NewItem;
        ParentItem.Text := ParentELement.Name;
        ParentItem.Local := FixHTMLpath(Allocator.GetFilename(ParentElement, 0));
        for k := 0 to TPasClassType(ParentElement).Members.Count-1 do
        begin
          TmpElement := TPasElement(TPasClassType(ParentElement).Members.Items[k]);
          if Engine.HidePrivate and(TmpElement.Visibility = visPrivate) then
            continue;
          if Engine.HideProtected and(TmpElement.Visibility = visProtected) then
            continue;
          TmpItem := ParentItem.Children.NewItem;
          case ElementType(TmpElement) of
            cmtProcedure   : TmpItem.Text := TmpElement.Name + ' procedure';
            cmtFunction    : TmpItem.Text := TmpElement.Name + ' function';
            cmtConstructor : TmpItem.Text := TmpElement.Name + ' constructor';
            cmtDestructor  : TmpItem.Text := TmpElement.Name + ' destructor';
            cmtProperty    : TmpItem.Text := TmpElement.Name + ' property';
            cmtVariable    : TmpItem.Text := TmpElement.Name + ' variable';
            cmtInterface   : TmpItem.Text := TmpElement.Name + ' interface';
            cmtUnknown     : TmpItem.Text := TmpElement.Name;
          end;
          TmpItem.Local := FixHTMLpath(Allocator.GetFilename(TmpElement, 0));
          {
          ParentElement = Class
             TmpElement = Member
          }
          MemberItem := nil;
          MemberItem := GetAlphaItem(Index.Items, TmpElement.Name);
          // ahh! if MemberItem.Local is empty MemberType is not shown!
          MemberItem.Local := FixHTMLpath(Allocator.GetFilename(TmpElement, 0));

          TmpItem := MemberItem.Children.NewItem;
          TmpItem.Text := ParentElement.Name;
          TmpITem.Local := FixHTMLpath(Allocator.GetFilename(TmpElement, 0));
        end;
      end;
      // routines
      for j := 0 to AModule.InterfaceSection.Functions.Count-1 do
      begin
        ParentElement := TPasProcedureType(AModule.InterfaceSection.Functions[j]);
        TmpItem := Index.Items.NewItem;
<<<<<<< HEAD
<<<<<<< HEAD
        TmpItem.Text := ParentElement.Name + ' ' + ParentElement.ElementTypeName;
=======
        TmpItem.Text := ParentElement.Name + ' ' + TPasFunction(ParentElement).ElementTypeName;
>>>>>>> graemeg/fixes_2_2
=======
        TmpItem.Text := ParentElement.Name + ' ' + TPasFunction(ParentElement).ElementTypeName;
>>>>>>> origin/fixes_2_2
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(ParentElement, 0));
      end;
      // consts
      for j := 0 to AModule.InterfaceSection.Consts.Count-1 do
      begin
        ParentElement := TPasElement(AModule.InterfaceSection.Consts[j]);
        TmpItem := Index.Items.NewItem;
        TmpItem.Text := ParentElement.Name;
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(ParentElement, 0));
      end;
      // types
      for j := 0 to AModule.InterfaceSection.Types.Count-1 do
      begin
        ParentElement := TPasType(AModule.InterfaceSection.Types[j]);
        TmpItem := Index.Items.NewItem;
        TmpItem.Text := ParentElement.Name;
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(ParentElement, 0));
        // enums
        if ParentELement is TPasEnumType then
        begin
          ParentItem := TmpItem;
          for k := 0 to TPasEnumType(ParentElement).Values.Count-1 do
          begin
            TmpElement := TPasType(TPasEnumType(ParentElement).Values.Items[k]);
            // subitem
            TmpItem := ParentItem.Children.NewItem;
            TmpItem.Text := TmpElement.Name;
            TmpItem.Local := ParentItem.Local;
            // root level
            TmpItem := Index.Items.NewItem;
            TmpItem.Text := TmpElement.Name;
            TmpItem.Local := ParentItem.Local;
          end;
        end;
      end;
      // variables
      for j := 0 to AModule.InterfaceSection.Variables.Count-1 do
      begin
        ParentElement := TPasElement(AModule.InterfaceSection.Variables[j]);
        TmpItem := Index.Items.NewItem;
        TmpItem.Text := ParentElement.Name + ' var';
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(ParentElement, 0));
      end;
      // declarations
      {
      for j := 0 to AModule.InterfaceSection.Declarations.Count-1 do
      begin
        ParentElement := TPasElement(AModule.InterfaceSection.Declarations[j]);
        TmpItem := Index.Items.NewItem;
        TmpItem.Text := ParentElement.Name;
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(ParentElement, 0));
      end;
      // resource strings
      for j := 0 to AModule.InterfaceSection.ResStrings.Count-1 do
      begin
        ParentElement := TPasElement(AModule.InterfaceSection.ResStrings[j]);
        TmpItem := Index.Items.NewItem;
        TmpItem.Text := ParentElement.Name;
        TmpItem.Local := FixHTMLpath(Allocator.GetFilename(ParentElement, 0));
      end;
      }
    end;

    // Sort
    Index.Items.Sort(TListSortCompare(@TOCSort));
    for i := 0 to Index.Items.Count-1 do
    begin
      Index.Items.Item[i].Children.Sort(TListSortCompare(@TOCSort));
    end;

    // save
    Index.SaveToStream(Stream);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if not fnobinindex then
      fchm.AppendBinaryindexFromSitemap(index,false);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    if not fnobinindex then
      fchm.AppendBinaryindexFromSitemap(index,false);
>>>>>>> origin/fixes_2.4
    Index.Free;
    Stream.Position :=0 ;
    FChm.AppendIndex(Stream);
    Stream.Free;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  except
    Dump_Stack(StdOut, get_frame);
    Halt(1);
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;

procedure TCHMHTMLWriter.WriteHTMLPages;
var
  i: Integer;
  PageDoc: TXMLDocument;
  FileStream: TMemoryStream;
  FileName: String;
  FilePath: String;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FileName := Engine.Output;
  if FileName = '' then
    Raise Exception.Create('Error: no --output option used.'); 
  
  if ExtractFileExt(FileName) <> FileNameExtension then
    FileName := ChangeFileExt(FileName, FileNameExtension);

  FOutChm := TFileStream.Create(FileName, fmOpenReadWrite or fmCreate);
=======
=======
>>>>>>> origin/fixes_2_2
  if Engine.Output = '' then
  begin
    WriteLn('Error: no --output option used.');
    Exit;
  end;
  
  if ExtractFileExt(Engine.Output) <> '.chm' then
    ChangeFileExt(Engine.OutPut, '.chm');

  FOutChm := TFileStream.Create(Engine.Output, fmOpenReadWrite or fmCreate);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FTempUncompressedName := GetTempFileName+IntToStr(GetProcessID) +'.raw';
  FTempUncompressed := TFileStream.Create(FTempUncompressedName, fmOpenReadWrite  or fmCreate);
  FChm := TChmWriter.Create(FOutChm, False);
<<<<<<< HEAD
<<<<<<< HEAD
  FChm.Title := FChmTitle;
  FChm.TempRawStream := FTempUncompressed;
  FChm.OnGetFileData := @RetrieveOtherFiles;
  FChm.OnLastFile := @LastFileAdded;
  fchm.hasbinarytoc:=not fnobintoc;;
  fchm.hasbinaryindex:=not fnobinindex;
=======
=======
>>>>>>> origin/fixes_2_2
  FChm.Title := Copy(Package.Name, 2, Length(Package.Name));
  FChm.TempRawStream := FTempUncompressed;
  FChm.OnGetFileData := @RetrieveOtherFiles;
  FChm.OnLastFile := @LastFileAdded;
<<<<<<< HEAD
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  fchm.hasbinarytoc:=not fnobintoc;;
  fchm.hasbinaryindex:=not fnobinindex;
>>>>>>> origin/fixes_2.4
  ProcessOptions;

  FileStream := TMemoryStream.Create;
  for i := 0 to PageInfos.Count - 1 do
    with TPageInfo(PageInfos[i]) do
    begin
      PageDoc := CreateHTMLPage(Element, SubpageIndex);
      try
        FileName := ExtractFileName(Allocator.GetFilename(Element, SubpageIndex));
        FilePath := '/'+FixHTMLpath(ExtractFilePath(Allocator.GetFilename(Element, SubpageIndex)));

        try
          WriteHTMLFile(PageDoc, FileStream);
          FChm.AddStreamToArchive(FileName, FilePath, FileStream, True);
        except
	  on E: Exception do
<<<<<<< HEAD
<<<<<<< HEAD
            DoLog(Format(SErrCouldNotCreateFile, [FileName, e.Message]));
=======
            WriteLn(Format(SErrCouldNotCreateFile, [FileName, e.Message]));
>>>>>>> graemeg/fixes_2_2
=======
            WriteLn(Format(SErrCouldNotCreateFile, [FileName, e.Message]));
>>>>>>> origin/fixes_2_2
        end;
      finally
        PageDoc.Free;
        FileStream.Size := 0;
      end;
    end;
  FileStream.Free;
<<<<<<< HEAD
<<<<<<< HEAD

  DoLog('HTML Files written. Collecting other files and compressing...this could take some time');

  //write any found images to CHM stream
  FileStream := TMemoryStream.Create;
  for i := 0 to FImageFileList.Count - 1 do
  begin
{$ifdef imagetest}    DoLog('  adding image: '+FImageFileList[i]); {$endif}
    if FileExists(FImageFileList[i]) then
    begin
{$ifdef imagetest}    DoLog(' - found'); {$endif}
      FileName := ExtractFileName(FImageFileList[i]);
      FilePath := '/'+FixHTMLpath(ExtractFilePath(FImageFileList[i]));

      FileStream.LoadFromFile(FImageFileList[i]);
      FChm.AddStreamToArchive(FileName, FilePath, FileStream, True);
      FileStream.Size := 0;
    end
    else
      {$ifdef imagetest}  DoLog(' - not found'){$endif};
  end;
  FileStream.Free;

=======
  WriteLn('HTML Files written. Collecting other files and compressing...this could take some time');
>>>>>>> graemeg/fixes_2_2
=======
  WriteLn('HTML Files written. Collecting other files and compressing...this could take some time');
>>>>>>> origin/fixes_2_2
  FChm.Execute;
  FChm.Free;
  // we don't need to free FTempUncompressed
  // FTempUncompressed.Free;
  FOutChm.Free;
  DeleteFile(FTempUncompressedName);
end;

function TCHMHTMLWriter.InterPretOption(const Cmd, Arg: String): boolean;
begin
  Result:=True;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FNoBinToc:=False;
  FnoBinIndex:=False;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  FNoBinToc:=False;
  FnoBinIndex:=False;
>>>>>>> origin/fixes_2.4
  if Cmd = '--toc-file' then
    FTOCName := arg
  else if Cmd = '--index-file' then
    FIndexName := arg
  else if Cmd = '--default-page' then
    FDefaultPage := arg
  else if Cmd = '--other-files' then
    FOtherFiles := arg
<<<<<<< HEAD
<<<<<<< HEAD
=======
  else if Cmd = '--css-file' then
    FCSSFile := arg
>>>>>>> graemeg/fixes_2_2
=======
  else if Cmd = '--css-file' then
    FCSSFile := arg
>>>>>>> origin/fixes_2_2
  else if Cmd = '--auto-index' then
    FAutoIndex := True
  else if Cmd = '--auto-toc' then
    FAutoTOC := True
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  else if Cmd = '--no-bintoc' then
    FNoBinToc := True
  else if Cmd = '--no-binindex' then
    FNoBinIndex := True
  else if Cmd = '--make-searchable' then
    FMakeSearchable := True
  else if Cmd = '--chm-title' then
    FChmTitle := arg
  else
    Result:=inherited InterPretOption(Cmd, Arg);

  if Length(FChmTitle) = 0 then
    FChmTitle := Copy(Package.Name, 2, Length(Package.Name));
=======
=======
>>>>>>> origin/fixes_2_2
  else if Cmd = '--make-searchable' then
    FMakeSearchable := True
  else
    Result:=inherited InterPretOption(Cmd, Arg);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

class procedure TCHMHTMLWriter.Usage(List: TStrings);
begin
  THTMLWriter.Usage(List);
  List.add('--default-page');
  List.Add(SCHMUsageDefPage);
  List.add('--toc-file');
  List.Add(SCHMUsageTOC);
  List.add('--index-file');
  List.Add(SCHMUsageIndex);
  List.add('--other-files');
  List.Add(SCHMUsageOtrFiles);
  List.add('--css-file');
  List.Add(SCHMUsageCSSFile);
  List.add('--auto-index');
  List.Add(SCHMUsageAutoIDX);
  List.add('--auto-toc');
  List.Add(SCHMUsageAutoTOC);
  List.add('--make-searchable');
  List.Add(SCHMUsageMakeSearch);
<<<<<<< HEAD
<<<<<<< HEAD
  List.Add('--chm-title');
  List.Add(SCHMUsageChmTitle);
end;

Class Function TCHMHTMLWriter.FileNameExtension : String; 

begin
  result:='.chm';
end;

class procedure TCHMHTMLWriter.SplitImport(var AFilename, ALinkPrefix: String);
var
  i: integer;
begin
  i := Pos(',', AFilename);
  if i > 0 then
    begin  //split into filename and prefix
    ALinkPrefix := Copy(AFilename,i+1,Length(AFilename));
    SetLength(AFilename, i-1);
    end
  else if ALinkPrefix = '' then
    begin  //synthesize outdir\pgk.xct, ms-its:pkg.chm::/
    ALinkPrefix := 'ms-its:' + ChangeFileExt(ExtractFileName(AFilename), '.chm') + '::/';
    AFilename := ChangeFileExt(AFilename, '.xct');
    end;
end;
=======
end;


>>>>>>> graemeg/fixes_2_2
=======
end;


>>>>>>> origin/fixes_2_2

{$ENDIF}
