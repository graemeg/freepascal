{
    This file is part of the Free Component Library

    Merge 2 JSON files.
    Copyright (c) 2021 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program jsonmerge;

{$mode objfpc}{$H+}

uses
  Classes, SysUtils, CustApp, fpJSON, jsonparser, fpjsonapply;

type

  { TJSONMergeApplication }

  TJSONMergeApplication = class(TCustomApplication)
  private
    function ParseOptions: string;
  protected
    FApplier : TJSONApplier;
    procedure DoRun; override;
  public
    constructor Create(TheOwner: TComponent); override;
    destructor Destroy; override;
    procedure Usage(const aErrorMsg: String); virtual;
  end;

{ TJSONMergeApplication }

Function TJSONMergeApplication.ParseOptions : string;

begin
  Result:='';
  FApplier.SourceFileName:=GetOptionValue('s','source');
  FApplier.ApplyFileName:=GetOptionValue('a','apply');
  FApplier.DestFileName:=GetOptionValue('d','destination');
  FApplier.CaseInsensitive:=HasOption('i','ignorecase');
  FApplier.RemoveNonExisting:=HasOption('r','remove');
  FApplier.Formatted:=HasOption('f','format');
  FApplier.SourcePath:=GetOptionValue('p','path');
  FApplier.ApplyPath:=GetOptionValue('y','apply-path');
  if (FApplier.SourceFileName='') then
    Result:='Missing source filename'
  else if (FApplier.ApplyFileName='') then
    Result:='Missing apply filename';
  if (Result='') and (FApplier.DestFileName='') then
    FApplier.DestFileName:=FApplier.SourceFileName;
end;

procedure TJSONMergeApplication.DoRun;
var
  ErrorMsg: String;
begin
  Terminate;
  ErrorMsg:=CheckOptions('hs:a:d:irfp:y:', ['help','source:','apply:','destination:','ignorecase','remove','format','path:','apply-path:']);
  if (ErrorMsg='') and not HasOption('h','help') then
    ErrorMsg:=ParseOptions;
  if (ErrorMsg<>'') or HasOption('h','help') then
    begin
    Usage(ErrorMsg);
    Exit;
    end;
  FApplier.Execute;
end;

constructor TJSONMergeApplication.Create(TheOwner: TComponent);
begin
  inherited Create(TheOwner);
  StopOnException:=True;
  FApplier:=TJSONApplier.Create(Self);
end;

destructor TJSONMergeApplication.Destroy;
begin
  FreeAndNil(FApplier);
  inherited Destroy;
end;

procedure TJSONMergeApplication.Usage(const aErrorMsg: String);
begin
  if (aErrorMsg<>'') then
    Writeln(aErrorMsg);
  writeln('Usage: ', ExeName, ' -h');
  writeln('where');
  writeln('-a --apply=FILE        File with JSON to apply to input.');
  writeln('-d --destination=FILE  File to write resulting JSON to (defaults to input)');
  writeln('-f --format            Format destination JSON.');
  writeln('-h --help              This help message.');
  writeln('-i --ignorecase        Ignore case when looking for element names.');
  writeln('-p --path=PATH         Start applying at element at PATH in source.');
  writeln('-r --remove            Remove elements in source not existing in apply file.');
  writeln('-s --source=FILE       File with JSON input.');
  writeln('-y --apply-path=PATH   Start applying at element at PATH in apply.');
  ExitCode:=Ord(aErrorMsg<>'');
end;

var
  Application: TJSONMergeApplication;

begin
  Application:=TJSONMergeApplication.Create(nil);
  Application.Title:='JSON merge tool';
  Application.Run;
  Application.Free;
end.

