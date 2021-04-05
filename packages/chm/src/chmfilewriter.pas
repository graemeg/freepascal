{ Copyright (C) <2005> <Andrew Haines> chmfilewriter.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
}
{
  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.
}
unit chmfilewriter;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, chmwriter, inifiles, contnrs, chmsitemap, avl_tree,
  {for html scanning } dom,SAX_HTML,dom_html;

type
  TChmProject = class;
  TChmProjectErrorKind = (chmerror,chmwarning,chmhint,chmnote,chmnone);

  TChmProgressCB = procedure (Project: TChmProject; CurrentFile: String) of object;
  TChmErrorCB    = procedure (Project: TChmProject;errorkind:TChmProjectErrorKind;msg:String;detaillevel:integer=0);

  { TChmProject }

  TChmProject = class
  private
    FAutoFollowLinks: Boolean;
    FDefaultFont: String;
    FDefaultPage: String;
    FFiles: TStrings;
    FIndexFileName: String;
    FMakeBinaryTOC: Boolean;
    FMakeBinaryIndex: Boolean;
    FMakeSearchable: Boolean;
    FFileName: String;
    FOnProgress: TChmProgressCB;
    FOnError   : TChmErrorCB;
    FOutputFileName: String;
    FTableOfContentsFileName: String;
    FTitle: String;
    FWindows : TObjectList;
    FMergeFiles : TStringlist;
    fDefaultWindow : string;
    fScanHtmlContents  : Boolean;
    fOtherFiles : TStrings; // Files found in a scan.
    fAllowedExtensions: TStringList;
    fTotalFileList : TAvlTree;
    fAnchorList    : TStringList;
    FSpareString   : TStringIndex;
    FBasePath      : String;     // location of the .hhp file. Needed to resolve relative paths
    FReadmeMessage : String;     // readme message
    FToc,
    FIndex         : TCHMSiteMap;
    FTocStream,
    FIndexStream   : TMemoryStream;
    FCores         : Integer;
    FLocaleID      : Word;
  protected
    function GetData(const DataName: String; out PathInChm: String; out FileName: String; var Stream: TStream): Boolean;
    procedure LastFileAdded(Sender: TObject);
    procedure readIniOptions(keyvaluepairs:tstringlist);
    procedure ScanHtml;
    procedure ScanList(toscan,newfiles:TStrings;recursion:boolean);
    procedure ScanSitemap(sitemap:TChmSiteMap;newfiles:TStrings;recursion:boolean);
    function  FileInTotalList(const s:String):boolean;
    function  SanitizeURL(const basepath, instring, localpath, localname:string; var outstring:String):Boolean;
  public
    constructor Create; virtual;
    destructor Destroy; override;
    procedure LoadFromFile(AFileName: String); virtual;
    procedure LoadFromhhp (AFileName:String;LeaveInclude:Boolean); virtual;
    procedure SaveToFile(AFileName: String); virtual;
    procedure SaveToHHP(AFileName: String);
    procedure WriteChm(AOutStream: TStream); virtual;
    procedure ShowUndefinedAnchors;
    function ProjectDir: String;
    procedure LoadSitemaps;
    procedure AddFileWithContext(contextid:integer;filename:ansistring;contextname:ansistring='');
    procedure Error(errorkind:TChmProjectErrorKind;msg:String;detaillevel:integer=0);
    // though stored in the project file, it is only there for the program that uses the unit
    // since we actually write to a stream
    property OutputFileName: String read FOutputFileName write FOutputFileName;
    property FileName: String read FFileName write FFileName;
    property Files: TStrings read FFiles write FFiles;  // html files
    property OtherFiles: TStrings read FOtherFiles write FOtherFiles;  // other files (.css, img etc)
    property AutoFollowLinks: Boolean read FAutoFollowLinks write FAutoFollowLinks;
    property TableOfContentsFileName: String read FTableOfContentsFileName write FTableOfContentsFileName;
    property MakeBinaryTOC: Boolean read FMakeBinaryTOC write FMakeBinaryTOC;
    property MakeBinaryIndex: Boolean read FMakeBinaryIndex write FMakeBinaryIndex;
    property Title: String read FTitle write FTitle;
    property IndexFileName: String read FIndexFileName write FIndexFileName;
    property MakeSearchable: Boolean read FMakeSearchable write FMakeSearchable;
    property DefaultPage: String read FDefaultPage write FDefaultPage;
    property DefaultFont: String read FDefaultFont write FDefaultFont;
    property Windows :TObjectList read FWindows write FWindows;
    property MergeFiles :TStringlist read FMergeFiles write FMergefiles;
    property OnProgress: TChmProgressCB read FOnProgress write FOnProgress;
    property OnError   : TChmErrorCB read FOnError write FOnError;
    property DefaultWindow : String read FDefaultWindow write FDefaultWindow;
    property ScanHtmlContents  : Boolean read fScanHtmlContents write fScanHtmlContents;
    property ReadmeMessage : String read FReadmeMessage write FReadmeMessage;
    property AllowedExtensions : TStringList read FAllowedExtensions;
    property Cores : integer read fcores write fcores;
    property LocaleID: word read FLocaleID write FLocaleID;
  end;

  TChmContextNode = Class
                     URLName       : AnsiString;
                     ContextNumber : THelpContext;
                     ContextName   : AnsiString;
                    End;

Const
  ChmErrorKindText : array[TCHMProjectErrorKind] of string = ('Error','Warning','Hint','Note','');

implementation

uses XmlCfg, CHMTypes;

type

  { TFirstReference }

  TFirstReference = class
  public
   Location: string;
   constructor Create(Const ALocation: string);
  end;

{ TFirstReference }

constructor TFirstReference.Create(const ALocation: string);
begin
  Location := ALocation;
end;

{ TChmProject }

function TChmProject.GetData(const DataName: String; out PathInChm: String; out
  FileName: String; var Stream: TStream): Boolean;
begin
  Result := False; // Return true to abort compressing files
  TMemoryStream(Stream).LoadFromFile(ProjectDir+DataName);
  // clean up the filename
  FileName := StringReplace(ExtractFileName(DataName), '\', '/', [rfReplaceAll]);
  FileName := StringReplace(FileName, '//', '/', [rfReplaceAll]);

  PathInChm := '/'+ExtractFilePath(DataName);
  if Assigned(FOnProgress) then FOnProgress(Self, DataName);
end;

procedure TChmProject.LastFileAdded(Sender: TObject);
var
  Writer: TChmWriter;
begin
  // Assign the TOC and index files
  Writer := TChmWriter(Sender);
  writer.cores:=fcores;
  {$ifdef chmindex}
    Writeln('binindex filename ',IndexFileName);
  {$endif}
  if assigned(FIndexStream) then
    begin
    FIndexStream.position:=0;
    Writer.AppendIndex(FIndexStream);
    if MakeBinaryIndex then
    begin
      Error(chmnote,'CHM has binary index');      
      Writer.AppendBinaryIndexFromSiteMap(FIndex,False);
    end;
  end;
  if assigned(FTocStream) then
    begin
    Writer.AppendTOC(FTOCStream);
    if MakeBinaryTOC then
    begin
      Error(chmnote,'CHM has binary toc');
      Writer.AppendBinaryTOCFromSiteMap(FToc);
    end;
  end;
  if not assigned(sender) then
    Writer.Free;
end;

constructor TChmProject.Create;
begin
  FFiles := TStringList.Create;
  FOtherFiles := TStringList.Create;
  FAllowedExtensions:=TStringList.Create;
  FAllowedExtensions.add('.HTM');
  FAllowedExtensions.add('.HTML');
  FWindows:=TObjectList.Create(True);
  FMergeFiles:=TStringlist.Create;
  ScanHtmlContents:=False;
  FTotalFileList:=TAvlTree.Create(@CompareStrings);
  FSparestring  :=TStringIndex.Create;
  fAnchorList := TStringList.Create;
  fAnchorList.Sorted := True;
  fAnchorList.OwnsObjects := True;
end;

destructor TChmProject.Destroy;
var i : integer;
begin
  for i:=0 to ffiles.count -1 do
    ffiles.objects[i].free;
  FMergeFiles.Free;
  FFiles.Free;
  FOtherFiles.Free;
  FWindows.Free;
  FSpareString.Free;
  FTotalFileList.FreeAndClear;
  FTotalFileList.Free;
  fAllowedExtensions.Free;
  FToc.free;
  FIndex.free;
  FTocStream.Free;
  FIndexStream.Free;
  fAnchorList.Free;
  inherited Destroy;
end;

Type
   TSectionEnum = (secOptions,secWindows,secFiles,secMergeFiles,secAlias,secMap,secInfoTypes,secTextPopups,secUnknown);
   TOptionEnum = (OPTAUTO_INDEX,OPTAUTO_TOC,OPTBINARY_INDEX,OPTBINARY_TOC,OPTCITATION,
       OPTCOMPRESS,OPTCOPYRIGHT,OPTCOMPATIBILITY,OPTCOMPILED_FILE,OPTCONTENTS_FILE,
       OPTCREATE_CHI_FILE,OPTDBCS,OPTDEFAULT_FONT,OPTDEFAULT_WINDOW,OPTDEFAULT_TOPIC,
       OPTDISPLAY_COMPILE_NOTES,OPTDISPLAY_COMPILE_PROGRESS,OPTENHANCED_DECOMPILATION,OPTERROR_LOG_FILE,OPTFLAT,
       OPTFULL_TEXT_SEARCH_STOP_LIST,OPTFULL_TEXT_SEARCH,OPTIGNORE,OPTINDEX_FILE,OPTLANGUAGE,OPTPREFIX,
       OPTSAMPLE_STAGING_PATH,OPTSAMPLE_LIST_FILE,OPTTMPDIR,OPTTITLE,OPTCUSTOM_TAB,OPTUNKNOWN);

Const
  SectionNames : Array[TSectionEnum] of String =
      ('OPTIONS','WINDOWS','FILES','MERGE FILES','ALIAS','MAP','INFOTYPES','TEXT POPUPS','UNKNOWN');

  OptionKeys : array [TOptionEnum] of String =
      ('AUTO INDEX','AUTO TOC','BINARY INDEX','BINARY TOC','CITATION',
       'COMPRESS','COPYRIGHT','COMPATIBILITY','COMPILED FILE','CONTENTS FILE',
       'CREATE CHI FILE','DBCS','DEFAULT FONT','DEFAULT WINDOW','DEFAULT TOPIC',
       'DISPLAY COMPILE NOTES','DISPLAY COMPILE PROGRESS','ENHANCED DECOMPILATION','ERROR LOG FILE','FLAT',
       'FULL-TEXT SEARCH STOP LIST','FULL-TEXT SEARCH','IGNORE','INDEX FILE','LANGUAGE','PREFIX',
       'SAMPLE STAGING PATH','SAMPLE LIST FILE','TMPDIR','TITLE','CUSTOM TAB','UNKNOWN');

function FindSectionName (const name:string):TSectionEnum;

begin
  result:=low(TSectionEnum);
  while (result<secUnknown) and (name<>SectionNames[Result]) do
    inc(result);
end;

function FindOptionName(Const name:string):TOptionEnum;

begin
  result:=low(TOptionEnum);
  while (result<optUnknown) and (name<>OptionKeys[Result]) do
    inc(result);
end;

// hex codes of LCID (Locale IDs) see at http://msdn.microsoft.com/en-us/goglobal/bb964664.aspx
function GetLanguageID(const sValue: String): word;
const
  DefaultLCID = $0409; // default "English - United States", 0x0409
var
  ACode: word;
begin
  Result := DefaultLCID;
  if Length(sValue) >= 5 then
  begin
    Val(Trim(Copy(sValue, 1, 6)), Result, ACode);
    //if Code <> 0 then
    //Result := DefaultLCID;
  end
end;


procedure TChmProject.readIniOptions(keyvaluepairs:tstringlist);
var i : integer;
    Opt : TOptionEnum;
    OptVal,
    OptValUpper : string;
begin
  for i:=0 to keyvaluepairs.count-1 do
    begin
      Opt:=findoptionname(uppercase(keyvaluepairs.names[i]));
      optval :=keyvaluepairs.valuefromindex[i];
      optvalupper:=uppercase(OptVal);
      case Opt Of
      OPTAUTO_INDEX                : ;
      OPTAUTO_TOC                  : ;
      OPTBINARY_INDEX              : MakeBinaryIndex:=optvalupper='YES';
      OPTBINARY_TOC                : MakeBinaryToc  :=optvalupper='YES';
      OPTCITATION                  : ;
      OPTCOMPRESS                  : ; // Doesn't seem to have effect in workshop
      OPTCOPYRIGHT                 : ;
      OPTCOMPATIBILITY             : ;
      OPTCOMPILED_FILE             : OutputFilename:=optval;
      OPTCONTENTS_FILE             : TableOfContentsFileName:=optval;
      OPTCREATE_CHI_FILE           : ;
      OPTDBCS                      : ; // What this field makes unicode is not known?
      OPTDEFAULT_FONT              : defaultfont:=optval;
      OPTDEFAULT_WINDOW            : defaultwindow:=optval;
      OPTDEFAULT_TOPIC             : defaultpage:=optval;
      OPTDISPLAY_COMPILE_NOTES     : ;
      OPTDISPLAY_COMPILE_PROGRESS  : ;
      OPTENHANCED_DECOMPILATION    : ;
      OPTERROR_LOG_FILE            : ;
      OPTFLAT                      : ;
      OPTFULL_TEXT_SEARCH_STOP_LIST: ;
      OPTFULL_TEXT_SEARCH          : MakeSearchable:=optvalupper='YES';
      OPTIGNORE                    : ;
      OPTINDEX_FILE                : Indexfilename:=optval;
      OPTLANGUAGE                  : LocaleID := GetLanguageID(optval);
      OPTPREFIX                    : ;  // doesn't seem to have effect
      OPTSAMPLE_STAGING_PATH       : ;
      OPTSAMPLE_LIST_FILE          : ;
      OPTTMPDIR                    : ;
      OPTTITLE                     : Title:=optval;
      OPTCUSTOM_TAB                : ;
      OPTUNKNOWN                   : ;  // can be used for errors on unknown keys
      end;
    end;
end;


procedure TChmProject.LoadFromFile(AFileName: String);
var
  Cfg: TXMLConfig;
  MergeFileCount,
  WinCount,
  FileCount: Integer;
  I  : Integer;
  nd : TChmContextNode;
  win: TCHMWindow;
  s  : String;

begin
  Cfg := TXMLConfig.Create(nil);
  Cfg.Filename := AFileName;
  FileName := expandfilename(AFileName);
  FBasePath:=extractfilepath(FileName);

  Files.Clear;
  FileCount := Cfg.GetValue('Files/Count/Value', 0);
  for I := 0 to FileCount-1 do
    begin
      nd:=TChmContextNode.Create;
      nd.urlname:=Cfg.GetValue('Files/FileName'+IntToStr(I)+'/Value','');
      nd.contextnumber:=Cfg.GetValue('Files/FileName'+IntToStr(I)+'/ContextNumber',0);
      nd.contextname:=Cfg.GetValue('Files/FileName'+IntToStr(I)+'/ContextName','');
      Files.AddObject(nd.URLNAME,nd);
    end;

  FileCount := Cfg.GetValue('OtherFiles/Count/Value', 0);
  for I := 0 to FileCount-1 do
    begin
      s:=Cfg.GetValue('OtherFiles/FileName'+IntToStr(I)+'/Value','');
      OtherFiles.Add(s);
    end;

  WinCount:= Cfg.GetValue('Windows/Count/Value', 0);
  for i:=0 To WinCount-1 do
    begin
      win:=TCHMWindow.Create;
      win.loadfromxml(cfg,'Windows/item'+inttostr(i)+'/');
      fwindows.add(win);
    end;

  Mergefilecount:=Cfg.getValue('MergeFiles/Count/Value', 0);
  for i:=0 To MergeFileCount-1 do
    Mergefiles.add(Cfg.getValue('MergeFiles/FileName'+IntToStr(I)+'/value',''));

  // load some values that changed key backwards compatible.

  IndexFileName := Cfg.GetValue('Files/IndexFile/Value','');
  if IndexFileName='' Then
    IndexFileName := Cfg.GetValue('Settings/IndexFile/Value','');

  TableOfContentsFileName := Cfg.GetValue('Files/TOCFile/Value','');
  If TableOfContentsFileName='' then
    TableOfContentsFileName := Cfg.GetValue('Settings/TOCFile/Value','');

  // For chm file merging, bintoc must be false and binindex true. Change defaults in time?
  // OTOH, merging will be mostly done for fpdoc files, and that doesn't care about defaults.

  S:=Cfg.GetValue('Files/MakeBinaryTOC/Value', '');
  if s='' Then
    MakeBinaryTOC := Cfg.GetValue('Settings/MakeBinaryTOC/Value', True)
  else
    MakeBinaryTOC := Cfg.GetValue('Files/MakeBinaryTOC/Value', True);

  S:=Cfg.GetValue('Files/MakeBinaryIndex/Value', '');
  if s='' Then
    MakeBinaryIndex := Cfg.GetValue('Settings/MakeBinaryIndex/Value', False)
  else
    MakeBinaryIndex := Cfg.GetValue('Files/MakeBinaryIndex/Value', False);

  AutoFollowLinks := Cfg.GetValue('Settings/AutoFollowLinks/Value', False);
  MakeSearchable := Cfg.GetValue('Settings/MakeSearchable/Value', False);
  DefaultPage := Cfg.GetValue('Settings/DefaultPage/Value', '');
  Title := Cfg.GetValue('Settings/Title/Value', '');
  OutputFileName := Cfg.GetValue('Settings/OutputFileName/Value', '');
  DefaultFont  := Cfg.GetValue('Settings/DefaultFont/Value', '');
  DefaultWindow:= Cfg.GetValue('Settings/DefaultWindow/Value', '');
  ScanHtmlContents:=  Cfg.GetValue('Settings/ScanHtmlContents/Value', False);
  LocaleID := Cfg.GetValue('Settings/LocaleID/Value', $0409);

  Cfg.Free;
end;

function cleanupstring(const s:string):string;
var
  i:integer;
begin
  i:=pos(';',s);
  if i>0 then
    result:=trim(copy(s,1,i-1))
  else
    result:=trim(s);
end;

procedure TChmProject.LoadFromhhp (AFileName:String;LeaveInclude:Boolean);
// leaveinclude=true leaves includefiles includefiles.

procedure addalias(const key,value :string);

var i,j : integer;
    node: TCHMContextNode;
    keyupper,valueupper : string;
begin
 { Defaults other than global }
   MakeBinaryIndex:=True;

 {$ifdef hhp_debug}
   writeln('alias entry:',key,'=',value);
 {$endif}
 keyupper:=uppercase(value);
 i:=0; j:=files.count;
 while (i<j) and (uppercase(TCHMContextnode(files.objects[i]).UrlName)<>keyupper) do
  inc(i);
 if i=j then
  begin
   {$ifdef hhp_debug}
    writeln('alias new node:',key);
   {$endif}
    node:=TCHMContextNode.create;
    valueupper:=stringReplace(value, '\', '/', [rfReplaceAll]);
    valueupper:= StringReplace(valueupper, '//', '/', [rfReplaceAll]);
    node.URLName:=valueupper;
    node.contextname:=key;
  end
 else
  begin
    node:=TCHMContextNode(Files.objects[i]);
    node.ContextName:=key;
  end;
end;

procedure processalias(strs:TStringlist);
var i,j : integer;
    s : string;
    strls2:tstringlist;

begin
 for i:=0 to strs.count-1 do
  begin
    s:=cleanupstring(strs[i]);
    if uppercase(copy(s,1,8))='#INCLUDE' then
      begin
        delete(s,1,8);
        s:=trim(s);
        if fileexists(s) then
          begin
            strls2:=TstringList.create;
            strls2.loadfromfile(s);
            processalias(strls2);
            strls2.free;
          end;

      end
    else
     begin
       s:=cleanupstring(s);
       j:=pos('=',s);
       if j>0 then
         addalias(trim(copy(s,1,j-1)),copy(s,j+1,length(s)-j));
     end;
  end;
end;

procedure addmap(const key,value :string);

var i,j : integer;
    node: TCHMContextNode;
    keyupper : string;
begin
 {$ifdef hhp_debug}
 writeln('map entry:',key,'=',value);
 {$endif}
 keyupper:=uppercase(key);
 i:=0; j:=files.count;
 while (i<j) and (uppercase(TCHMContextnode(files.objects[i]).contextname)<>keyupper) do
  inc(i);
 if i=j then
    raise Exception.create('context "'+key+'" not found!')
 else
  begin
    node:=TCHMContextNode(Files.objects[i]);
    node.Contextnumber:=strtointdef(value,0);
  end;
end;

procedure processmap(strs:TStringlist);
var i,j : integer;
    s : string;
    strls2:tstringlist;

begin
 for i:=0 to strs.count-1 do
  begin
    s:=cleanupstring(strs[i]);
    {$ifdef hhp_debug}
      writeln('map item:',s);
    {$endif}
    if uppercase(copy(s,1,8))='#INCLUDE' then
      begin
        delete(s,1,8);
        s:=trim(s);
        if fileexists(s) then
          begin
            strls2:=TstringList.create;
            strls2.loadfromfile(s);
            processmap(strls2);
            strls2.free;
          end;
      end
    else
     begin
       s:=cleanupstring(s);
       if uppercase(copy(s,1,7))='#DEFINE' Then
         begin
           delete(s,1,7);
           s:=trim(s);
           j:=pos(' ',s);
           if j>0 then
             addmap(trim(copy(s,1,j-1)),copy(s,j+1,length(s)-j));
         end
       else
         begin
            {$ifdef hhp_debug}
              writeln('map leftover:',s);
            {$endif}
         end;
     end;
  end;
end;

var
  Fini      : TMemIniFile;  // TMemInifile is more compatible with Delphi. Delphi's API based TIniFile fails on .hhp files.
  secs,strs : TStringList;
  i,j       : Integer;
  section   : TSectionEnum;
  nd        : TChmContextNode;

begin
 { Defaults other than global }
  MakeBinaryIndex:=True;
  filename:=expandfilename(afilename);
  FBasePath:=extractfilepath(filename);
  Fini:=TMeminiFile.Create(AFileName);
  secs := TStringList.create;
  strs := TStringList.create;
  fini.readsections(secs);

  // Do the files section first so that we can emit errors if
  // other sections reference unknown files.

  fini.readsectionvalues(SectionNames[secFiles] ,strs);
  if strs.count>0 then
    for j:=0 to strs.count-1 do
      begin
          nd:=TChmContextNode.Create;
          nd.urlname:=StringReplace(strs[j],'\', '/', [rfReplaceAll]);
          nd.contextnumber:=0;
          nd.contextname:='';
          Files.AddObject(nd.urlname,nd);
        end;

  // aliases also add file nodes.

  fini.readsectionvalues(SectionNames[secAlias] ,strs); // resolve all aliases.
  if strs.count>0 then
    processalias(strs);

  // map files only add to existing file nodes.
  fini.readsectionvalues(SectionNames[secmap] ,strs);
  if strs.count>0 then
    processmap(strs);


  for i:=0 to secs.count-1 do
    begin
      section:=FindSectionName(Uppercase(Secs[i]));
      if section<>secunknown then
        fini.readsectionvalues(secs[i] ,strs);
      case section of
      secOptions   : readinioptions(strs);
      secWindows   : for j:=0 to strs.count-1 do
                       FWindows.add(TCHMWindow.Create(strs[j]));
      secFiles     : ; // already done
      secMergeFiles: FMergeFiles.Assign(Strs); // just a filelist
      secAlias     : ; // already done
      secMap       : ; // already done
      secInfoTypes : ; // unused for now.
      secTextPopups: ; // rarely used.
      end;
    end;
  secs.free;
  strs.free;
  fini.free;
  ScanHtmlContents:=true;
end;

procedure TChmProject.AddFileWithContext(contextid:integer;filename:ansistring;contextname:ansistring='');
var x : integer;
    nd : TChmContextNode;
begin
  x:=files.indexof(filename);
  if x=-1 then
    begin
      nd:=TChmContextNode.Create;
      nd.urlname:=filename;
      nd.contextnumber:=contextid;
      nd.contextname:=contextname;
      Files.AddObject(nd.urlname,nd);
    end
  else
   begin
     nd:=TChmContextNode(files.objects[x]);
     if not assigned(nd) then
       begin
         nd:=TChmContextNode.Create;
         nd.urlname:=filename;
         files.objects[x]:=nd;
       end;
      nd.contextnumber:=contextid;
      nd.contextname:=contextname;
   end;
end;

procedure TChmProject.SaveToFile(AFileName: String);
var
  Cfg: TXMLConfig;
  I  : Integer;
  nd : TChmContextNode;
begin
  Cfg := TXMLConfig.Create(nil);
  Cfg.StartEmpty := True;
  Cfg.Filename := AFileName;
  Cfg.Clear;
  Cfg.SetValue('Files/Count/Value', Files.Count);
  for I := 0 to Files.Count-1 do
  begin
    nd:=TChmContextNode(files.objects[i]);
    Cfg.SetValue('Files/FileName'+IntToStr(I)+'/Value', Files.Strings[I]);
    if assigned(nd) then
      begin
        Cfg.SetValue('Files/FileName'+IntToStr(I)+'/ContextNumber', nd.contextnumber);
        Cfg.SetValue('Files/FileName'+IntToStr(I)+'/ContextName', nd.contextname);
      end;
  end;

  Cfg.SetValue('OtherFiles/Count/Value', OtherFiles.Count);
  for I := 0 to OtherFiles.Count-1 do
    Cfg.SetValue('OtherFiles/FileName'+IntToStr(I)+'/Value', OtherFiles.Strings[I]);


  Cfg.SetValue('Windows/Count/Value', FWindows.count);
  for i:=0 To FWindows.Count-1 do
    TCHMWindow(FWindows[i]).savetoxml(cfg,'Windows/item'+inttostr(i)+'/');

  Cfg.SetValue('MergeFiles/Count/Value', FMergeFiles.count);
  for i:=0 To FMergeFiles.Count-1 do
    Cfg.SetValue('MergeFiles/FileName'+IntToStr(I)+'/value',FMergeFiles[i]);

  // delete legacy keys.
  Cfg.DeleteValue('Files/IndexFile/Value');
  Cfg.DeleteValue('Files/TOCFile/Value');
  Cfg.DeleteValue('Files/MakeBinaryTOC/Value');
  Cfg.DeleteValue('Files/MakeBinaryIndex/Value');
  Cfg.SetValue('Settings/IndexFile/Value', IndexFileName);
  Cfg.SetValue('Settings/TOCFile/Value', TableOfContentsFileName);
  Cfg.SetValue('Settings/MakeBinaryTOC/Value',MakeBinaryTOC);
  Cfg.SetValue('Settings/MakeBinaryIndex/Value',MakeBinaryIndex);

  Cfg.SetValue('Settings/AutoFollowLinks/Value', AutoFollowLinks);
  Cfg.SetValue('Settings/MakeSearchable/Value', MakeSearchable);
  Cfg.SetValue('Settings/DefaultPage/Value', DefaultPage);
  Cfg.SetValue('Settings/Title/Value', Title);
  Cfg.SetValue('Settings/OutputFileName/Value', OutputFileName);
  Cfg.SetValue('Settings/DefaultFont/Value', DefaultFont);

  Cfg.SetValue('Settings/DefaultWindow/Value', DefaultWindow);
  Cfg.SetValue('Settings/ScanHtmlContents/Value', ScanHtmlContents);
  Cfg.SetValue('Settings/LocaleID/Value', LocaleID);

  Cfg.Flush;
  Cfg.Free;
end;

function TChmProject.ProjectDir: String;
begin
  Result := ExtractFilePath(FileName);
end;

procedure TChmProject.Error(errorkind:TChmProjectErrorKind;msg:String;detaillevel:integer=0);
begin
  if assigned(OnError) then
    OnError(self,errorkind,msg,detaillevel);
end;

const
   protocols   : array[0..4] of string = ('HTTP:','HTTPS:','FTP:','MS-ITS:', 'MAILTO:');
   protocollen : array[0..4] of integer= ( 5 ,6, 4 ,7, 7);

function TChmProject.SanitizeURL(const basepath,instring,localpath,localname:string;var outstring:String):Boolean;
var i,j,len : integer;
  Anchor: String;
begin
  result:=true; outstring:='';
  if instring='' then
    exit(false);

  len:=length(instring);
  if len=0 then
    exit(false);
  { Check for protocols before adding local path }
  i:=0;
  while (i<=high(protocols)) do
    begin
      if strlicomp(@protocols[i][1],@instring[1],protocollen[i])=0 then
        exit(false);
      inc(i);
    end;
   outstring:=localpath+instring;
   i:=pos('#',outstring);
   if i<>0 then begin
     if i<>length(outstring) then // trims lone '#' at end of url.
       begin
         if i > 1 then
           Anchor := outstring
         else
           Anchor := localname+outstring;
         j := fAnchorList.IndexOf(Anchor);
         if j < 0 then begin
           fAnchorList.AddObject(Anchor,TFirstReference.Create(localname));
           Anchor := '(new) '+Anchor;
         end;
         Error(CHMNote, 'Anchor found '+Anchor+' while scanning '+localname,1);
       end;
     delete(outstring,i,length(outstring)-i+1);
   end;

  outstring:=expandfilename(includetrailingpathdelimiter(fbasepath)+StringReplace(outstring,'%20',' ',[rfReplaceAll]));// expandfilename(instring));

  outstring:=extractrelativepath(basepath,outstring);
  outstring:=StringReplace(outstring,'\','/',[rfReplaceAll]);
  if outstring='' then
    result:=false;
end;

function  TChmProject.FileInTotalList(const s:String):boolean;

begin
  FSpareString.TheString:=S;
  result:=assigned(fTotalFileList.FindKey(FSpareString,@CompareStrings));
end;

procedure TChmProject.ScanList(toscan,newfiles:TStrings;recursion:boolean);
 // toscan, list to search for htmlfiles to scan.
 // newfiles, the resulting list of files.
 // totalfilelist, the list that contains all found and specified files to check against.
 // localfilelist (local var), files found in this file.
var
  localpath : string;

function findattribute(node:TDomNode;attributename:string):String;
var
    Attributes: TDOMNamedNodeMap;
    atnode    : TDomNode;
    n         : integer;
begin
  Result := '';
  if assigned(node) then
    begin
      Attributes:=node.Attributes;
      if assigned(attributes) then
        for n:=0 to attributes.length-1 do
          begin
            atnode :=attributes[n];
            if assigned(atnode) and (uppercase(atnode.nodename)=attributename) then
              exit(atnode.nodevalue);
          end;
    end;
end;

procedure checkattributes(node:TDomNode;attributename:string; const localname: string; filelist :TStringList);
var
    fn  : String;
    val : String;
begin
  val := findattribute(node,attributename);
  if sanitizeurl(fbasepath,val,localpath,localname,fn) then
    if (Length(fn) > 0) { Skip links to self using named anchors }
        and not FileInTotalList(uppercase(fn)) then
      filelist.add(fn);
end;

procedure checkattributesA(node:TDomNode;const localname: string; filelist :TStringList);
// workaround for "a" tag that has href and src. If src exists, don't check href, this
// avoids spurious warnings.
var
    fn  : String;
    val : String;
    found : boolean;
begin
  found:=false;
  val := findattribute(node,'SRC');
  if sanitizeurl(fbasepath,val,localpath,localname,fn) then
      found:=true;
  if not found then
    begin
      val := findattribute(node,'HREF');
      if sanitizeurl(fbasepath,val,localpath,localname,fn) then
        found:=true;
    end;
 if found and not FileInTotalList(uppercase(fn)) then
      filelist.add(fn);
end;

function scantags(prnt:TDomNode; const localname: string; filelist:TStringlist):TDomNode;

var
  att : ansistring;

procedure AddAnchor(const s:string);
var
   i   : Integer;
begin
  i := fAnchorList.IndexOf(localname+'#'+s);
  if i < 0 then begin
    fAnchorList.Add(localname+'#'+s);
    Error(ChmNote,'New Anchor with '+att+' '+s+' found while scanning '+localname,1);
  end else if fAnchorList.Objects[i] = nil then
    Error(chmwarning,'Duplicate anchor definitions with '+att+' '+s+' found while scanning '+localname,1)
  else begin
    fAnchorList.Objects[i].Free;
    fAnchorList.Objects[i] := nil;
    Error(ChmNote,'Anchor with '+att+' '+s+' defined while scanning '+localname,1);
  end;
end;

var chld: TDomNode;
    s,attrval  : ansistring;
    idfound : boolean;


begin
  result:=nil;
  if assigned(prnt )  then
    begin
      chld:=prnt.firstchild;
      while assigned(chld) do
        begin
          scantags(chld, localname, filelist);  // depth first.
          if (chld is TDomElement) then
            begin
              s:=uppercase(tdomelement(chld).tagname);
              att := 'ID';
              attrval := findattribute(chld, att);
              idfound:=attrval  <> '' ;
              if idfound then
                addanchor(attrval);
              if s='LINK' then
                begin
                  //printattributes(chld,'');
                  checkattributes(chld,'HREF',localname,filelist);
                end;
              if s='SCRIPT' then
                begin
                  //printattributes(chld,'');
                  checkattributes(chld,'SRC',localname,filelist);
                end;
             if s='IMG' then
               begin
                  //printattributes(chld,'');
                  checkattributes(chld,'SRC',localname,filelist);
               end;
             if s='A' then
               begin
                  //printattributes(chld,'');
                  checkattributesA(chld,localname,filelist);
                  if not idfound then
                    begin
                      att := 'NAME';
                      attrval := findattribute(chld, att);
                      if attrval  <> '' then
                       addanchor(attrval);
                    end;
                end;
            end;
          chld:=chld.nextsibling;
        end;
    end;
end;

var
  localfilelist: TStringList;
  domdoc : THTMLDocument;
  i,j    : Integer;
  fn,reffn   : string;
  tmplst : Tstringlist;

function trypath(const vn:string;const vl:string):boolean;
var vn2: String;
  strrec : TStringIndex;
begin
  vn2:=uppercase(vn);
  if FileInTotalList(vn2) then
   begin
     Error(ChmNote,'Found duplicate file '+vl+' while scanning '+fn,1);
     exit(true);
   end;

  result:=false;
  if fileexists(vn) then  // correct for relative path .html file?
    begin
      result:=true;
      StrRec:=TStringIndex.Create;
      StrRec.TheString:=vn2;
      StrRec.Strid    :=0;
      fTotalFileList.Add(StrRec);
      newfiles.add(vl);
      Error(ChmNote,'Found file '+vl+' while scanning '+fn,1);
    end;
end;

var efn : string;

begin
 localfilelist:=TStringList.Create;
 for j:=0 to toscan.count-1 do
   begin
     fn:=toscan[j];
     localfilelist.clear;
     if (FAllowedExtensions.Indexof(uppercase(extractfileext(fn)))<>-1) then
       begin
         if fbasepath<>'' then
           efn:=IncludeTrailingPathDelimiter(FBasePath)+fn
         else
           efn:=fn;
         if fileexists(efn) then
           begin
             domdoc:=THtmlDocument.Create;
             try
               Error(chmnote,'Scanning file '+fn+'.',5);
               ReadHtmlFile(domdoc,efn);
               localpath:=extractfilepath(fn);
               if (length(localpath)>0) and not (localpath[length(localpath)] in ['/','\']) then
                 localpath:=localpath+pathsep;
               scantags(domdoc,extractfilename(fn),localfilelist);
               for i:=0 to localFilelist.count-1 do
                 begin
                   reffn:=localfilelist[i];
                   if not trypath(IncludeTrailingPathDelimiter(fbasepath)+reffn,reffn) then  //  if not trypath(localpath+s) then
                       Error(ChmWarning,'Found file '+reffn+' while scanning '+fn+', but couldn''t find it on disk',2);
                 end;
             except
               on e:EDomError do
                  Error(ChmError,'Html parsing '+fn+', failed with a DOM error: '+e.Message);
               on e:exception do
                  Error(ChmError,'Html parsing '+fn+', failed. Please submit a bug.');
               end;
             domdoc.free;
           end
         else
           begin
             Error(chmnote,'Can''t find file '+fn+' to scan it.',5);
           end;
        end
     else if FileExists(fn) and (uppercase(ExtractFileExt(fn))='.CSS') then
       begin
         tmplst:=TStringList.Create;
         try
           tmplst.LoadFromFile(fn);

           for i:=0 to tmplst.Count-1 do
             begin
               reffn:=tmplst[i];
               if pos('url(''', reffn)>0 then
                 begin
                   delete(reffn,1,pos('url(''', reffn)+4);
                   reffn:=trim(copy(reffn,1,pos('''',reffn)-1));
                   if not trypath(IncludeTrailingPathDelimiter(fbasepath)+reffn,reffn) then
//                     if not trypath(localpath+s) then
                       Error(ChmWarning,'Found file '+reffn+' while scanning '+fn+', but couldn''t find it on disk',2);
                 end;
             end;
         finally
           tmplst.Free;
         end;
       end
     else
       Error(chmnote,'Not scanning file because of unknown extension '+fn,5);
   end;
  localfilelist.free;
  if (newfiles.count>0) and recursion then
    begin
      tmplst:=TStringList.Create;
      scanlist(newfiles,tmplst,true);
      newfiles.addstrings(tmplst);
      tmplst.free;
    end;
end;

procedure TChmProject.ScanSitemap(sitemap:TChmSiteMap;newfiles:TStrings;recursion:boolean);

procedure scanitems(it:TChmSiteMapItems);

var i,j : integer;
    x : TChmSiteMapItem;
    si  : TChmSiteMapSubItem;
    s : string;
    strrec : TStringIndex;

begin
  for i:=0 to it.count -1 do
    begin
      x:=it.item[i];
      for j:=0 to x.SubItemcount-1 do
         begin
           si:=x.SubItem[j];
           if sanitizeurl(fbasepath,si.local,'','Site Map for '+x.Text,S) then   // sanitize, remove stuff etc.
             begin
               if not FileInTotalList(uppercase(s)) then
                 begin
                   if fileexists(s) then
                     begin
                       Error(chmnote,'Good url: '+s+'.',5);
                       StrRec:=TStringIndex.Create;
                       StrRec.TheString:=uppercase(s);
                       StrRec.Strid    :=0;
                       fTotalFileList.Add(StrRec);
                       newfiles.add(s);
                     end
                   else
                     Error(chmnote,'duplicate url: '+s+'.',5);
                 end
               else
                 Error(chmnote,'duplicate url: '+s+'.',5);
             end
           else
            Error(chmnote,'Bad url: '+s+'.',5);
         end;
      if assigned(x.children) and (x.children.count>0) then
        scanitems(x.children);
    end;
end;

var
    localfilelist: TStringList;

begin
  localfilelist:=TStringList.Create;
  scanitems(sitemap.items);
  scanlist(newfiles,localfilelist,true);
  newfiles.addstrings(localfilelist);
  localfilelist.free;
end;

procedure TChmProject.ScanHtml;
var
  helplist,
  localfilelist: TStringList;
  i      : integer;
  strrec : TStringIndex;
begin

 for i:=0 to otherfiles.count-1 do
   begin
     StrRec:=TStringIndex.Create;
     StrRec.TheString:=uppercase(otherfiles[i]);
     StrRec.Strid    :=0;
     fTotalFileList.Add(StrRec);
   end;

 for i:=0 to files.count-1 do
   begin
     StrRec:=TStringIndex.Create;
     StrRec.TheString:=uppercase(files[i]);
     StrRec.Strid    :=0;
     fTotalFileList.Add(StrRec);
   end;

 localfilelist:= TStringList.create;
 scanlist(ffiles,localfilelist,true);
 otherfiles.addstrings(localfilelist);
 localfilelist.clear;
 if (FDefaultpage<>'') and (not FileInTotalList(uppercase(fdefaultpage))) then
   begin
     Error(chmnote,'Scanning default file : '+fdefaultpage+'.',3);
     helplist:=TStringlist.Create;
     helplist.add(fdefaultpage);
     scanlist(helplist,localfilelist,true);
     otherfiles.addstrings(localfilelist);
     localfilelist.clear;
   end;
 if assigned(FToc) then
   begin
       Error(chmnote,'Scanning TOC file : '+FTableOfContentsFileName+'.',3);
        try
          scansitemap(ftoc,localfilelist,true);
          otherfiles.addstrings(localfilelist);
        except
          on e: Exception do
            error(chmerror,'Error scanning TOC file ('+FTableOfContentsFileName+')');
          end;
   end;
  LocalFileList.clear;
  if assigned(FIndex) then
    begin
      Error(chmnote,'Scanning Index file : '+FIndexFileName+'.',3);
      try
        scansitemap(FIndex,localfilelist,true);
        otherfiles.addstrings(localfilelist);
      except
        on e: Exception do
          error(chmerror,'Error scanning index file ('+FIndexFileName+')');
        end;
    end;
 localfilelist.free;
end;


procedure TChmProject.WriteChm(AOutStream: TStream);
var
  Writer     : TChmWriter;
  TOCStream,
  IndexStream: TFileStream;
  nd         : TChmContextNode;
  I          : Integer;
begin

  LoadSiteMaps;

  // Scan html for "rest" files.
  If ScanHtmlContents Then
    ScanHtml;                 // Since this is slowing we opt to skip this step, and only do this on html load.

  IndexStream := nil;
  TOCStream := nil;

  Writer := TChmWriter.Create(AOutStream, False);

  // our callback to get data
  Writer.OnGetFileData := @GetData;
  Writer.OnLastFile    := @LastFileAdded;

  // give it the list of html files
  Writer.FilesToCompress.AddStrings(Files);

  // give it the list of other files

  Writer.FilesToCompress.AddStrings(OtherFiles);

  // now some settings in the chm
  Writer.DefaultPage := DefaultPage;
  Writer.Title := Title;
  Writer.DefaultFont := DefaultFont;
  Writer.FullTextSearch := MakeSearchable;
  Writer.HasBinaryTOC := MakeBinaryTOC;
  Writer.HasBinaryIndex := MakeBinaryIndex;
  Writer.IndexName := IndexFileName;
  Writer.TocName   := TableOfContentsFileName;
  Writer.ReadmeMessage := ReadmeMessage;
  Writer.DefaultWindow := FDefaultWindow;
  Writer.LocaleID := FLocaleID;
  for i:=0 to files.count-1 do
    begin
      nd:=TChmContextNode(files.objects[i]);
      if not fileexists(IncludeTrailingPathDelimiter(FBasePath)+files[i]) then
         Error(chmWarning,'File '+Files[i]+' does not exist');
      if assigned(nd) and (nd.contextnumber<>0) then
        Writer.AddContext(nd.ContextNumber,IncludeTrailingPathDelimiter(FBasePath)+files[i]);
    end;
  if FWIndows.Count>0 then
    Writer.Windows:=FWIndows;
  if FMergeFiles.Count>0 then
    Writer.Mergefiles:=FMergeFiles;
  if assigned(ftoc) then
    Writer.TocSitemap:=ftoc;

  // and write!

  Error(chmnone,'Writing CHM '+OutputFileName,0);

  Writer.Execute;

  if Assigned(TOCStream) then TOCStream.Free;
  if Assigned(IndexStream) then IndexStream.Free;
  Writer.Free;
end;

procedure TChmProject.ShowUndefinedAnchors;
var
  i:Integer;
begin
  for i := 0 to fAnchorList.Count-1 do
    if fAnchorList.Objects[i] <> nil then
       Error(chmerror,'Anchor '+fAnchorList[i]+' undefined; first use '+TFirstReference(fAnchorList.Objects[i]).Location);
end;

procedure TChmProject.LoadSitemaps;
var
  FullFileName: string;
// #IDXHDR (merged files) goes into the system file, and need to keep  TOC sitemap around
begin
   if FTableOfContentsFileName<>'' then
   begin
     FullFileName := IncludeTrailingPathDelimiter(ProjectDir()) + ExtractFileName(FTableOfContentsFileName);
     if FileExists(FullFileName) then
       begin
         FreeAndNil(FTocStream);
         FTocStream:=TMemoryStream.Create;
         try
           FTocStream.loadfromfile(FullFileName);
           //writeln(ftableofcontentsfilename, ' ' ,ftocstream.size);
           FTocStream.Position:=0;
           FreeAndNil(FToc);
           FToc:=TChmSiteMap.Create(sttoc);
           FToc.loadfromstream(FTocStream);
         except
          on e:exception do
            begin
               error(chmerror,'Error loading TOC file '+FullFileName);
               freeandnil(ftoc); freeandnil(FTocStream);
             end;
           end;
       end
     else
       error(chmerror,'Can''t find TOC file '+FullFileName);
   end;
   if FIndexFileName<>'' then
     begin
       FullFileName := IncludeTrailingPathDelimiter(ProjectDir()) + ExtractFileName(FIndexFileName);
       if FileExists(FullFileName) then
         begin
            FreeAndNil(FIndexStream);
            FIndexStream:=TMemoryStream.Create;
            try
              FIndexStream.LoadFromFile(FullFileName);
              FIndexStream.Position:=0;
              FreeAndNil(FIndex);
              FIndex:=TChmSiteMap.Create(stindex);
              FIndex.loadfromfile(FullFileName);
              Error(chmnote,'Index items:'+inttostr(findex.Items.count));
            except
              on e: Exception do
                begin
                  error(chmerror,'Error loading index file '+FullFileName);
                  freeandnil(findex); freeandnil(findexstream);
                end;
              end;
           end
         else
           error(chmerror,'Can''t find index file '+FullFileName);
     end;
end;


function BoolAsStr(b: Boolean): string;
begin
  if b then
    Result := 'Yes'
  else
    Result := 'No';
end;

procedure TChmProject.SaveToHHP(AFileName: String);
var
  sl: TStringList;
  s : string;
  i: Integer;
  ContextItem: TChmContextNode;

  procedure SetOption(const AKey, AValue: string);
  begin
    if AValue <> '' then
      sl.Add(AKey + '=' + AValue);
  end;

begin
  sl := TStringList.Create();
  try
    sl.Add('[OPTIONS]');
    SetOption('Title', Title);
    SetOption('Compatibility', '1.1 or later');
    SetOption('Compiled file', OutputFileName);
    SetOption('Default Topic', DefaultPage);
    SetOption('Default Font', DefaultFont);
    SetOption('Default Window', DefaultWindow);
    SetOption('Display compile progress', 'Yes');
    //SetOption('Error log file', 'errors.log');
    SetOption('Contents file', TableOfContentsFileName);
    //SetOption('Auto Index', BoolAsStr(MakeBinaryIndex));
    SetOption('Index file', IndexFileName);
    SetOption('Binary Index', BoolAsStr(MakeBinaryIndex));
    SetOption('Binary TOC', BoolAsStr(MakeBinaryTOC));
    SetOption('Full-text search', BoolAsStr(MakeSearchable));
    SetOption('Language', '0x' + IntToHex(LocaleID, 4));

    sl.Add('');
    sl.Add('[FILES]');
    for i := 0 to Files.Count - 1 do
    begin
      s := StringReplace(Files.Strings[i], '/', '\', [rfReplaceAll]);
      sl.Add(s);
    end;

    if MergeFiles.Count > 0 then
    begin
      sl.Add('');
      sl.Add('[MERGE FILES]');
      for i := 0 to MergeFiles.Count - 1 do
      begin
        sl.Add(MergeFiles.Strings[i]);
      end;
    end;

    if Windows.Count > 0 then
    begin
      sl.Add('');
      sl.Add('[WINDOWS]');
      for i := 0 to Windows.Count-1 do
      begin
        TCHMWindow(Windows[i]).SaveToIni(s);
        sl.Add(s);
      end;
    end;

    if Files.Count > 0 then
    begin
      sl.Add('');
      sl.Add('[ALIAS]');
      for i := 0 to Files.Count - 1 do
      begin
        contextitem:=TChmContextNode(files.objects[i]);
        if assigned(contextitem) then
          sl.Add(ContextItem.ContextName + '=' + ContextItem.UrlName);
      end;

      sl.Add('');
      sl.Add('[MAP]');
      for I := 0 to Files.Count-1 do
      begin
        contextitem:=TChmContextNode(files.objects[i]);
        if assigned(contextitem) then
          sl.Add('#define ' + ContextItem.ContextName + ' ' + IntToStr(ContextItem.ContextNumber));
      end;
    end;

    sl.SaveToFile(AFileName);
  finally
    sl.Free();
  end;
end;
end.

