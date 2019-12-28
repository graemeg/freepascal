unit pas2jsresources;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,
  {$IFDEF pas2js}
  web,
  {$ELSE}
  base64,
  {$ENDIF}
  pas2jsfs, jsTree;

Type
  TResourceScopeMode = (rmProgram,rmUnit);

  { TPas2jsResourceHandler }
  TResourceOutputMode = (romNone,romJS,romFile,romExtraJS);
  TPas2jsResourceHandler = class(TObject)
  private
    FCurrentUnitName: String;
    FFS: TPas2JSFS;
  Protected
    // Must be overridden
    function GetResourceCount: Integer; virtual; abstract;
    // Content of file, if file based.
    function GetAsString: String; virtual; abstract;
    // Detect some common formats
    Function GetFormat(const aFileName : string; aOptions : TStrings) : string; virtual;
  Public
    Constructor Create(aFS : TPas2JSFS); virtual;
    // Called for every found resource
    Procedure HandleResource (aFileName : string; Options : TStrings); virtual; abstract;
    // Extension of output file, if file based
    Class Function OutputFileExtension : String; virtual;
    // True if output is file based (i.e. written to separate file)
    Class Function OutputMode : TResourceOutputMode; virtual; abstract;
    // Load resource file. Can be used in descendents
    Function LoadFile(aFileName : string) : TPas2jsFile;
    // Load resource file and encode as base64 string. Can be used in descendents
    Function GetFileAsBase64(aFileName : string)  : string;
    // This is called for every unit.
    Procedure StartUnit(Const aUnitName : String); virtual;
    // This is called at the start of every unit if OutputIsUnitBased is true.
    Procedure ClearUnit; virtual;
    // This is called at the end of every unit if OutputIsUnitBased is true. Only once if not.
    Procedure DoneUnit(isMainFile : Boolean); virtual;
    // This is called when Javascript is written for a unit
    Function WriteJS(const aUnitName : String; aModule : TJSElement) : TJSElement; virtual;
    // Current unit.
    Property CurrentUnitName : String Read FCurrentUnitName;
    // Passed at create
    property FS : TPas2JSFS Read FFS;
    // Return file content for writing to file if IsFileBased
    Property AsString : String Read GetAsString;
    // Number of resources
    Property ResourceCount : Integer Read GetResourceCount;
  end;

  { TNoResources }

  TNoResources = Class(TPas2jsResourceHandler)
  Public
    Procedure HandleResource (aFileName : string; Options : TStrings); override;
    Class Function OutputMode : TResourceOutputMode; override;
    function GetResourceCount: Integer; override;
    function GetAsString: String; override;
  end;

implementation

{ TNoResources }

procedure TNoResources.HandleResource(aFileName: string; Options: TStrings);
begin
  // Do nothing
  if aFileName='' then ;
  if Options=nil then ;
end;


class function TNoResources.OutputMode: TResourceOutputMode;
begin
  result:=romNone;
end;

function TNoResources.GetResourceCount: Integer;
begin
  Result:=0;
end;

function TNoResources.GetAsString: String;
begin
  Result:='';
end;

{ TPas2jsResourceHandler }


function TPas2jsResourceHandler.GetFormat(const aFileName: string; aOptions: TStrings): string;

Var
  E : String;

begin
  Result:=aOptions.Values['format'];
  if Result='' then
    begin
    E:=ExtractFileExt(aFileName);
    if (E<>'') and (E[1]='.') then
      E:=Copy(E,2,Length(E)-1);
    if Pos(LowerCase(E),';png;jpg;jpeg;bmp;ico;')>0 then
      Result:='image/'+E
    else if Pos(LowerCase(E),';htm;html;')>0 then
      Result:='text/html'
    else if Pos(LowerCase(E),';txt;lpr;pas;pp;')>0 then
      Result:='text/text'
    else if Pos(LowerCase(E),';js;')>0 then
      Result:='application/javascript'
    else if Pos(LowerCase(E),';json;')>0 then
      Result:='application/javascript'
    else
      Result:='application/octet-stream';
    end;
end;

constructor TPas2jsResourceHandler.Create(aFS: TPas2JSFS);
begin
  FFS:=aFS;
end;


class function TPas2jsResourceHandler.OutputFileExtension: String;
begin
  Result:='';
end;


function TPas2jsResourceHandler.LoadFile(aFileName: string): TPas2jsFile;
begin
  Result:=FS.LoadFile(aFileName,True);
end;

function TPas2jsResourceHandler.GetFileAsBase64(aFileName: string): string;

Var
  F : TPas2JSFile;

begin
  F:=LoadFile(aFileName);
  {$IFDEF pas2js}
  Result:=window.atob(F.Source);
  {$ELSE}
  Result:=EncodeStringBase64(F.Source);
  {$ENDIF}
  // Do not release, FS will release all files
end;

procedure TPas2jsResourceHandler.ClearUnit;
begin
  FCurrentUnitName:='';
end;

procedure TPas2jsResourceHandler.StartUnit(const aUnitName: String);
begin
  FCurrentUnitName:=aUnitName;
end;

procedure TPas2jsResourceHandler.DoneUnit(isMainFile: Boolean);
begin
  if not isMainFile then
    ClearUnit;
end;

function TPas2jsResourceHandler.WriteJS(const aUnitName: String; aModule: TJSElement): TJSElement;
begin
  Result:=aModule;
  if aUnitName='' then ;
end;


end.

