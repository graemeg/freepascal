unit pas2jshtmlresources;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, pas2jsResources, pas2jsFS;

Type

  { THTMLResourceLinkHandler }

  THTMLResourceLinkHandler = Class(TPas2jsResourceHandler)
  Private
    FLinkType: string;
    FPrefix: String;
    FResources : tstrings;
  Protected
    function GetResourceCount: Integer; override;
    function GetAsString: String; override;
    Function CreateDataLink(Const aResourceName,aFormat,aData : String) : String;
  Public
    Constructor Create(aFS : TPas2JSFS); override;
    Class Function OutputMode : TResourceOUtputMode; override;
    Class Function OutputFileExtension : String; override;
    Procedure HandleResource (aFileName : string; Options : TStrings); override;
    Procedure ClearUnit; override;
    Procedure DoneUnit(IsMainFile : Boolean); override;
    destructor Destroy; override;
    // ID is IDPrefix-resourcename. The default Prefix is 'resource'
    Property IDPrefix : String Read FPrefix Write FPrefix;
    Property LinkType : string Read FLinkType Write FLinkType;
    Property Resources : TStrings Read FResources;
  end;

implementation

{ THTMLResourceLinkHandler }

function THTMLResourceLinkHandler.GetResourceCount: Integer;
begin
  Result:=FResources.Count;
end;

function THTMLResourceLinkHandler.GetAsString: String;
begin
  Result:=FResources.Text;
end;

function THTMLResourceLinkHandler.CreateDataLink(const aResourceName, aFormat, aData: String): String;
begin
  Result:=Format('<link rel="%s" as="script" id="%s-%s" data-unit="%s" href="data:%s;base64,%s" />',[linkType,IDPrefix,aResourceName,CurrentUnitName,aFormat,aData]);
end;

procedure THTMLResourceLinkHandler.HandleResource(aFileName: string; Options: TStrings);

Var
  S : String;
  aFormat,ResourceName : String;


begin
  S:=GetFileAsBase64(aFileName);
  aFormat:=GetFormat(aFileName,Options);
  ResourceName:=Options.Values['name'];
  if ResourceName='' then
    ResourceName:=ChangeFileExt(ExtractFileName(aFileName),'');
  Resources.Add(CreateDataLink(ResourceName,aFormat,S))
end;

constructor THTMLResourceLinkHandler.Create(aFS: TPas2JSFS);
begin
  inherited Create(aFS);
  FResources:=TStringList.Create;
  IDPrefix:='resource';
  LinkType:='preload';
end;

class function THTMLResourceLinkHandler.OutputMode: TResourceOutputMode;
begin
  Result:=romFile;
end;


class function THTMLResourceLinkHandler.OutputFileExtension: String;
begin
  Result:='.html';
end;

procedure THTMLResourceLinkHandler.ClearUnit;
begin
  inherited ClearUnit;
  FResources.Clear;
end;

procedure THTMLResourceLinkHandler.DoneUnit(IsMainFile : Boolean);
begin
  // Do no call inherited, it will clear the list
  if IsMainFile then ;
end;


destructor THTMLResourceLinkHandler.Destroy;
begin
  FreeAndNil(FResources);
  inherited Destroy;
end;

end.

