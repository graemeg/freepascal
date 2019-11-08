unit pas2jsjsresources;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, pas2jsResources, pas2jsFS;

Type

  { TJSResourceLinkHandler }

  { TJSResourceHandler }

  TJSResourceHandler = Class(TPas2jsResourceHandler)
  Private
    FResources : TStrings;
    function GetResourceJSStatement(aFileName: String; Options: TStrings): String;
  Protected
    function GetResourceCount: Integer; override;
    function GetAsString: String; override;
  Public
    Constructor Create(aFS : TPas2JSFS); override;
    Class Function OutputMode : TResourceOutputMode; override;
    Class Function OutputFileExtension : String; override;
    Procedure HandleResource (aFileName : string; Options : TStrings); override;
    destructor Destroy; override;
    Property Resources : TStrings Read FResources;
  end;

implementation

{ TJSResourceHandler }

function TJSResourceHandler.GetResourceCount: Integer;
begin
  Result:=FResources.Count;
end;

function TJSResourceHandler.GetAsString: String;

Var
  I : Integer;
  N,V : String;

begin
  Result:='';
  For I:=0 to FResources.Count-1 do
    begin
    FResources.GetNameValue(I,N,V);
    Result:=Result+V+#10;
    end;
end;

constructor TJSResourceHandler.Create(aFS: TPas2JSFS);
begin
  inherited Create(aFS);
  FResources:=TStringList.Create;
end;

class function TJSResourceHandler.OutputMode: TResourceOutputMode;
begin
  Result:=romExtraJS;
end;

class function TJSResourceHandler.OutputFileExtension: String;
begin
  Result:='.js';
end;

Function TJSResourceHandler.GetResourceJSStatement(aFileName : String; Options: TStrings) : String;

Const
  SAddResource = 'rtl.addResource({name: "%s", unit: "%s", format: "%s", encoding: "base64", data: "%s"});';

Var
  aFormat,aName,aData : String;

begin
  aData:=GetFileAsBase64(aFileName);
  aFormat:=GetFormat(aFileName,Options);
  aName:=Options.Values['name'];
  if aName='' then
    aName:=ChangeFileExt(ExtractFileName(aFileName),'');
  Result:=Format(SAddResource,[aName,CurrentUnitName,aFormat,aData]);
end;

procedure TJSResourceHandler.HandleResource(aFileName: string; Options: TStrings);
begin
  // PRepending unit name allows to extract easier all resources for a single unit
  FResources.Add(CurrentUnitName+'='+GetResourceJSStatement(aFileName,Options));
end;


destructor TJSResourceHandler.Destroy;
begin
  FreeAndNil(FResources);
  inherited Destroy;
end;

end.

