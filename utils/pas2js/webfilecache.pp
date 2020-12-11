unit webfilecache;

{$mode objfpc}

// Enable this to write lots of debugging info to the browser console.
{$DEFINE VERBOSEWEBCACHE}

interface

uses
  Classes, SysUtils, JS, Web, fpjson, pas2jsfs, pscanner, contnrs;

type
  TPas2jsWebFS = Class;

  { TWebFileContent }

  TWebFileContent = Class(TObject)
  private
    FContents: string;
    FFileName: String;
    FModified: Boolean;
    procedure SetContents(AValue: string);
  Public
    Constructor Create(const aFileName,aContents : String);
    Property FileName : String Read FFileName Write FFileName;
    Property Contents : string Read FContents Write SetContents;
    Property Modified : Boolean Read FModified;
  end;
  { TWebFilesCache }

  TWebFilesCache = Class(TObject)
  Private
    FFiles : TFPObjectHashTable;
    Function FindFile(aFileName : String) : TWebFileContent;
  Public
    Constructor Create;
    Destructor Destroy; override;
    Function HasFile(aFileName : String) : Boolean;
    Function GetFileContent(Const aFileName : String) : String;
    function SetFileContent(const aFileName, aContent: String): Boolean;
  end;

  { TPas2jsWebFile }

  TPas2jsWebFile = Class(TPas2jsFile)
  public
    function CreateLineReader(RaiseOnError: boolean): TSourceLineReader; override;
    function Load(RaiseOnError: boolean; Binary: boolean): boolean; override;
  end;

  { TWebSourceLineReader }

  TWebSourceLineReader = Class(TSourceLineReader)
  private
    FFS: TPas2jsFS;
  Protected
    Property FS : TPas2jsFS Read FFS;
    Procedure IncLineNumber; override;
  end;

  // aFileName is the original filename, not normalized one
  TLoadFileEvent = Reference to Procedure(Sender : TObject; aFileName : String; aError : string);

  { TLoadFileRequest }

  TLoadFileRequest = Class(TObject)
    FFS : TPas2jsWebFS;
    FFileName : string;
    FXML : TJSXMLHttpRequest;
    FOnLoaded : TLoadFileEvent;
  private
    procedure DoChange;
  Public
    constructor Create(aFS: TPas2jsWebFS; const aFileName : string; aOnLoaded: TLoadFileEvent);
    Procedure DoLoad(const aURL : String);
  end;


  { TPas2jsWebFS }

  TPas2jsWebFS = Class(TPas2jsFS)
  Private
    FCache : TWebFilesCache;
    FLoadBaseURL: String;
    FOnLoadedFile: TLoadFileEvent;
  protected
    // Only for names, no paths
    Class Function NormalizeFileName(Const aFileName : String) : String;
    function FindSourceFileName(const aFilename: string): String; override;
  public
    Constructor Create; override;
    // Overrides
    function CreateResolver: TPas2jsFSResolver; override;
    function FileExists(const aFileName: String): Boolean; override;
    function FindCustomJSFileName(const aFilename: string): String; override;
    function FindIncludeFileName(const aFilename, SrcDir, ModuleDir: string; Mode: TModeSwitch): String; override;
    function FindUnitFileName(const aUnitname, InFilename, ModuleDir: string; out IsForeign: boolean): String; override;
    function FindUnitJSFileName(const aUnitFilename: string): String; override;
    function LoadFile(Filename: string; Binary: boolean=false): TPas2jsFile; override;
    procedure SaveToFile(ms: TFPJSStream; Filename: string); override;
    Function SetFileContent(Const aFileName,aContents : String) : Boolean;
    Function GetFileContent(Const aFileName : String) : String;
    // Returns false if the file was already loaded. OnLoaded is called in either case.
    Function LoadFile(aFileName : String; OnLoaded : TLoadFileEvent = Nil) : Boolean;
    // Returns number of load requests. OnLoaded is called for each file in the list
    Function LoadFiles(aList : TStrings;OnLoaded : TLoadFileEvent = Nil) : Integer;
    Function LoadFiles(aList : array of String;OnLoaded : TLoadFileEvent = Nil) : integer;
    Property OnLoadedFile : TLoadFileEvent Read FOnLoadedFile Write FOnLoadedFile;
    Property LoadBaseURL : String Read FLoadBaseURL Write FLoadBaseURL;
  end;

  { TPas2jsFileResolver }

  { TPas2jsWebResolver }

  TPas2jsWebResolver = class(TPas2jsFSResolver)
  private
    function GetWebFS: TPas2jsWebFS;
  public
    Property WebFS : TPas2jsWebFS Read GetWebFS;
  end;

implementation

{ TWebSourceLineReader }

procedure TWebSourceLineReader.IncLineNumber;
begin
  if (FFS<>nil) then
    FFS.IncReadLineCounter;
  inherited IncLineNumber;
end;

{ TLoadFileRequest }

procedure TLoadFileRequest.DoChange;

Var
  Err : String;
begin
  Case FXML.readyState of
    TJSXMLHttpRequest.UNSENT : ;
    TJSXMLHttpRequest.OPENED : ;
    TJSXMLHttpRequest.HEADERS_RECEIVED : ;
    TJSXMLHttpRequest.LOADING : ;
    TJSXMLHttpRequest.DONE :
      begin
      if (FXML.Status div 100)=2 then
        begin
        Err:='';
        // FS will normalize filename
        FFS.SetFileContent(FFileName,FXML.responsetext)
        end
      else
        Err:='Error loading file: '+FXML.StatusText;
      If Assigned(FOnLoaded) then
        FOnLoaded(FFS,FFileName,Err);
      if Assigned(FFS.OnLoadedFile) then
        FFS.OnLoadedFile(FFS,FFileName,Err);
      Free;
      end;
  end
end;

constructor TLoadFileRequest.Create(aFS: TPas2jsWebFS; const aFileName : string; aOnLoaded: TLoadFileEvent);
begin
  FFS:=aFS;
  FOnLoaded:=aOnLoaded;
  FFileName:=aFileName;
end;

Procedure TLoadFileRequest.DoLoad(const aURL: String);
begin
  FXML:=TJSXMLHttpRequest.new;
  FXML.onreadystatechange:=@DoChange;
  // Maybe one day allow do this sync, so the compiler can load files on demand.
  FXML.Open('GET',aURL);
  FXML.Send;
end;

{ TPas2jsWebFile }

function TPas2jsWebFile.CreateLineReader(RaiseOnError: boolean): TSourceLineReader;
begin
  {$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': Creating line reader for ',FileName);
  {$ENDIF}
  if Load(RaiseOnError,False) then
    begin
    Result:=TWebSourceLineReader.Create(FileName,Source);
    TWebSourceLineReader(Result).FFS:=Self.FS;
    end
  else
    Result:=Nil;
end;

function TPas2jsWebFile.Load(RaiseOnError: boolean; Binary: boolean): boolean;
begin
  Result:=False;
  {$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': Loading for ',FileName);
  {$ENDIF}
  With (FS as TPas2jsWebFS).FCache do
    if HasFile(FileName) then
      begin
      SetSource(GetFileContent(FileName));
      Result:=True;
      end;
  if Not Result then
    if RaiseOnError then
      Raise EFileNotFoundError.Create('File not loaded '+FileName)
{$IFDEF VERBOSEWEBCACHE}
    else Writeln('File not loaded '+FileName);
{$ENDIF}
end;

{ TWebFilesCache }

function TWebFilesCache.FindFile(aFileName: String): TWebFileContent;

Var
  N : THTCustomNode;

begin
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': Looking for file : ',aFileName);
{$ENDIF}
  N:=FFiles.Find(aFileName);
  if N=Nil then
    result:=Nil
  else
    Result:=TWebFileContent(THTObjectNode(N).Data);
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': Looking for file : ',aFileName, ': ',Assigned(Result));
{$ENDIF}
end;

constructor TWebFilesCache.Create;
begin
  FFiles:=TFPObjectHashTable.Create(True);
end;

destructor TWebFilesCache.Destroy;
begin
  FreeAndNil(FFiles);
  inherited Destroy;
end;

function TWebFilesCache.HasFile(aFileName: String): Boolean;
begin
  Result:=FindFile(aFileName)<>Nil;
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': HasFile(',aFileName,') : ',Result);
{$ENDIF}
end;

function TWebFilesCache.GetFileContent(const aFileName: String): String;

Var
  W : TWebFileContent;

begin
  {$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': GetFileContent(',aFileName,')');
  {$ENDIF}
  W:=FindFile(aFileName);
  if Assigned(W) then
    Result:=W.Contents
  else
    Raise EFileNotFoundError.Create('No such file '+AFileName);
end;

function TWebFilesCache.SetFileContent(const aFileName, aContent: String) : Boolean;

Var
  W : TWebFileContent;

begin
  {$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': SetFileContent(',aFileName,')');
  {$ENDIF}
  W:=FindFile(aFileName);
  Result:=Assigned(W);
  if Result then
    W.Contents:=aContent
  else
    FFiles.Add(aFileName,TWebFileContent.Create(aFileName,aContent));
end;

{ TWebFileContent }

procedure TWebFileContent.SetContents(AValue: string);
begin
  if FContents=AValue then Exit;
  FContents:=AValue;
  FModified:=True;
end;

constructor TWebFileContent.Create(const aFileName, aContents: String);
begin
  FContents:=aContents;
  FFileName:=aFileName;
end;


{ TPas2jsWebFS }

function TPas2jsWebFS.FileExists(const aFileName: String): Boolean;
begin
  {$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FileExists(',aFileName,')');
  {$ENDIF}
  Result:=FCache.HasFile(NormalizeFileName(aFileName));
  {$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FileExists(',aFileName,') : ',Result);
  {$ENDIF}
end;

function TPas2jsWebFS.FindCustomJSFileName(const aFilename: string): String;
begin
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindCustomJSFileName(',aFileName,')');
{$ENDIF}
  Result:=NormalizeFileName(aFileName);
  If not FCache.HasFile(Result) then
    Result:='';
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindCustomJSFileName(',aFileName,'): ',Result);
{$ENDIF}
end;

function TPas2jsWebFS.FindIncludeFileName(const aFilename, SrcDir, ModuleDir: string; Mode: TModeSwitch
  ): String;
begin
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindIncludeFileName(',aFileName,',',ModuleDir,')');
{$ENDIF}
  Result:=NormalizeFileName(aFileName);
  If not FCache.HasFile(Result) then
    Result:='';
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindIncludeFileName(',aFileName,') : ',Result);
{$ENDIF}
end;

class function TPas2jsWebFS.NormalizeFileName(const aFileName: String): String;
begin
  Result:=LowerCase(ExtractFileName(aFileName));
end;

function TPas2jsWebFS.FindSourceFileName(const aFilename: string): String;
begin
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindSourceFileName(',aFileName,')');
{$ENDIF}
  Result:=NormalizeFileName(aFileName);
  If not FCache.HasFile(Result) then
    Result:='';
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindSourceFileName(',aFileName,') : ',Result);
{$ENDIF}
end;

constructor TPas2jsWebFS.Create;
begin
  inherited Create;
  FCache:=TWebFilesCache.Create;
end;

function TPas2jsWebFS.CreateResolver: TPas2jsFSResolver;
begin
  Result:=TPas2jsWebResolver.Create(Self);
end;

function TPas2jsWebFS.FindUnitFileName(const aUnitname, InFilename, ModuleDir: string; out IsForeign: boolean): String;
begin
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindUnitFileName(',aUnitName,',',InFilename,',',ModuleDir,')');
{$ENDIF}
  Result:=NormalizeFileName(aUnitName+'.pas');
  isForeign:=False;
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindUnitFileName(',aUnitName,') : ',Result);
{$ENDIF}
end;

function TPas2jsWebFS.FindUnitJSFileName(const aUnitFilename: string): String;
begin
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindUnitJSFileName(',aUnitFileName,')');
{$ENDIF}
  Result:=NormalizeFileName(aUnitFileName);
{$IFDEF VERBOSEWEBCACHE}
  Writeln(ClassName,': FindUnitJSFileName(',aUnitFileName,') : ',Result);
{$ENDIF}
end;

function TPas2jsWebFS.LoadFile(Filename: string; Binary: boolean): TPas2jsFile;

begin
  Result:=TPas2jsWebFile.Create(Self,FileName);
  Result.Load(True,False);
end;

(*
  // Check if we should not be using this instead, as the compiler outputs UTF8 ?
  // Found on
  // https://weblog.rogueamoeba.com/2017/02/27/javascript-correctly-converting-a-byte-array-to-a-utf-8-string/
function stringFromUTF8Array(data)
  {
    const extraByteMap = [ 1, 1, 1, 1, 2, 2, 3, 0 ];
    var count = data.length;
    var str = "";

    for (var index = 0;index < count;)
    {
      var ch = data[index++];
      if (ch & 0x80)
      {
        var extra = extraByteMap[(ch >> 3) & 0x07];
        if (!(ch & 0x40) || !extra || ((index + extra) > count))
          return null;

        ch = ch & (0x3F >> extra);
        for (;extra > 0;extra -= 1)
        {
          var chx = data[index++];
          if ((chx & 0xC0) != 0x80)
            return null;

          ch = (ch << 6) | (chx & 0x3F);
        }
      }

      str += String.fromCharCode(ch);
    }

    return str;
  }
*)
procedure TPas2jsWebFS.SaveToFile(ms: TFPJSStream; Filename: string);

Var
  aContent : String;
  i : Integer;
  v : JSValue;

begin
  aContent:='';
  for I:=0 to MS.Length-1 do
    begin
    v:=MS[i];
    {AllowWriteln}
    Writeln('Char ',i,'(',v,') : ',TJSString.fromCharCode(v));
    {AllowWriteln-}
    aContent:=aContent+TJSString.fromCharCode(MS[i]);
    end;
  SetFileContent(FileName,aContent);
end;

function TPas2jsWebFS.SetFileContent(const aFileName, aContents: String): Boolean;
begin
  Result:=FCache.SetFileContent(NormalizeFileName(aFileName),aContents);
end;

function TPas2jsWebFS.GetFileContent(const aFileName: String): String;
begin
  Result:=FCache.GetFileContent(NormalizeFileName(aFileName));
end;

function TPas2jsWebFS.LoadFile(aFileName: String; OnLoaded: TLoadFileEvent): Boolean;

Var
  FN : String;
  aURL : String;
  LF : TLoadFileRequest;

begin
  FN:=NormalizeFileName(aFileName);
  Result:=Not FCache.HasFile(FN);
  if Not result then
    begin
    // It is already loaded
    if Assigned(OnLoaded) then
      OnLoaded(Self,aFileName,'')
    end
  else
    begin
    // Not yet already loaded
    aURL:=IncludeTrailingPathDelimiter(LoadBaseURL)+FN;
    LF:=TLoadFileRequest.Create(Self,aFileName,OnLoaded);
    LF.DoLoad(aURL);
    end;
end;

function TPas2jsWebFS.LoadFiles(aList: TStrings; OnLoaded: TLoadFileEvent
  ): Integer;

Var
  i: Integer;

begin
  Result:=0;
  For I:=0 to aList.Count-1 do
    if LoadFile(aList[i],OnLoaded) then
      Inc(Result);
end;

function TPas2jsWebFS.LoadFiles(aList: array of String; OnLoaded: TLoadFileEvent
  ): integer;

Var
  i: Integer;

begin
  Result:=0;
  For I:=0 to Length(aList)-1 do
    if LoadFile(aList[i],OnLoaded) then
      Inc(Result);
end;

{ TPas2jsWebResolver }

function TPas2jsWebResolver.GetWebFS: TPas2jsWebFS;
begin
  Result:=TPas2jsWebFS(FS)
end;



end.

