{
    This file is part of the Free Component Library

    Apply elements from one JSON object to another.
    Copyright (c) 2021 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpjsonapply;

{$mode ObjFPC}{$H+}

interface

uses
  Classes, SysUtils, fpJSON;

Type
  TOwnsJSON = (ojSource,ojApply);
  TOwnsJSONs = set of TOwnsJSON;

  { TJSONApplier }

  TJSONApplier = class(TComponent)
  private
    FApplyFileName: String;
    FApplyJSON: TJSONObject;
    FApplyPath: String;
    FCaseInsensitive: Boolean;
    FCloneSource: boolean;
    FDestFileName: String;
    FDestJSON: TJSONObject;
    FForceCorrectType: Boolean;
    FFormatted: Boolean;
    FOwnsJSON: TOwnsJSONs;
    FRemoveNonExisting: Boolean;
    FSourceFileName: String;
    FSourceJSON: TJSONObject;
    FSourcePath: String;
    procedure MaybeLoadApply;
    procedure MaybeLoadSource;
    procedure SetApplyJSON(AValue: TJSONObject);
    procedure SetSourceJSON(AValue: TJSONObject);
  Protected
    procedure Apply(aSrc, aApply: TJSONObject); virtual;
    procedure SaveDestJSON(aFileName : string);
    procedure SaveDestJSON(aStream : TStream);
  Public
    destructor destroy; override;
    // apply ApplyJSON to SourceJSON, set result in DestJSON
    Procedure Execute; virtual;
    // Source JSON. If not set, load from SourceFileName
    Property SourceJSON : TJSONObject Read FSourceJSON Write SetSourceJSON;
    // JSON to apply. If not set, load from ApplyFileName
    Property ApplyJSON : TJSONObject Read FApplyJSON Write SetApplyJSON;
    // Destination JSON. Can be equal to SourceJSON if CloneSource is not True.
    Property DestJSON : TJSONObject  Read FDestJSON;
    // Are SourceJSON, ApplyJSON owned by the component ? Are set when loading from file.
    Property OwnsJSON : TOwnsJSONs Read FOwnsJSON Write FOwnsJSON;
  Published
    // File to load SourceJSON from if it is not set.
    Property SourceFileName : String Read FSourceFileName Write FSourceFileName;
    // JSON path in source JSON where to start merging. Must exist and be an object!
    Property SourcePath : String Read FSourcePath Write FSourcePath;
    // File to load ApplyJSON from if it is not set.
    Property ApplyFileName : String Read FApplyFileName Write FApplyFileName;
    // JSON path in apply JSON where to start merging. Must exist and be an object!
    Property ApplyPath : String Read FApplyPath Write FApplyPath;
    // file to write DestJSON to after merging. Can be empty
    Property DestFileName : String Read FDestFileName Write FDestFileName;
    // Make a clone copy of SourceJSON before applying ApplyJSON ?
    Property CloneSource : boolean Read FCloneSource Write FCloneSource;
    // Search names case insensitively ?
    Property CaseInsensitive : Boolean Read FCaseInsensitive Write FCaseInsensitive;
    // If the type of an entry is different in Source and Apply, overwrite the entry with the value in Apply
    Property ForceCorrectType : Boolean Read FForceCorrectType Write FForceCorrectType;
    // After adding new entries from Apply, remove entries in Source that are not in apply.
    property RemoveNonExisting : Boolean Read FRemoveNonExisting Write FRemoveNonExisting;
    // Write formatted output in Destfilename (or not)
    Property Formatted : Boolean Read FFormatted Write FFormatted;
  end;



implementation

Resourcestring
  SErrSourceEmpty = 'Cannot apply to empty source object';
  SErrApplyEmpty = 'Cannot apply empty object';
  SErrSourceIsNotObject = 'JSON source file does not contain a JSON object';
  SErrApplyIsNotObject = 'JSON apply file does not contain a JSON object';
  SErrPathNotFound = 'Path "%s" in %s JSON not found';

{ TJSONApplier }

procedure TJSONApplier.SetApplyJSON(AValue: TJSONObject);
begin
  if FApplyJSON=AValue then Exit;
  if ojApply in FOwnsJSON then
    FreeAndNil(FApplyJSON);
  FApplyJSON:=AValue;
end;

procedure TJSONApplier.SetSourceJSON(AValue: TJSONObject);
begin
  if FSourceJSON=AValue then Exit;
  if ojSource in FOwnsJSON then
    FreeAndNil(FSourceJSON);
  FSourceJSON:=AValue;
end;

procedure TJSONApplier.MaybeLoadSource;

Var
  D : TJSONData;
  F : TFileStream;

begin
  If (FSourceJSON=Nil) and (SourceFileName<>'') then
    begin
    F:=TFileStream.Create(SourceFileName, fmOpenRead or fmShareDenyWrite);
    try
      D:=GetJSON(F);
      if D.JSONType<>jtObject then
        begin
        D.Free;
        Raise EJSON.Create(SErrSourceIsNotObject)
        end;
    finally
      F.Free;
    end;
    SourceJSON:=D as TJSONObject;
    Include(FOwnsJSON,ojSource);
    end;
end;

procedure TJSONApplier.MaybeLoadApply;

Var
  D : TJSONData;
  F : TFileStream;

begin
  If (ApplyFileName<>'') then
    begin
    F:=TFileStream.Create(ApplyFileName, fmOpenRead or fmShareDenyWrite);
    try
      D:=GetJSON(F);
      if D.JSONType<>jtObject then
        begin
        D.Free;
        Raise EJSON.Create(SErrApplyIsNotObject)
        end;
    finally
      F.Free;
    end;
    ApplyJSON:=D as TJSONObject;
    Include(FOwnsJSON,ojApply);
    end;
end;

procedure TJSONApplier.Apply(aSrc, aApply : TJSONObject);

Var
  aEnum : TJSONEnum;
  aIdx : Integer;

begin
  for aEnum in aApply do
    begin
    aIdx:=aSrc.IndexOfName(aEnum.Key,CaseInsensitive);
    if (aIdx<>-1) and FForceCorrectType and (aSrc.Items[aIdx].JSONType<>aEnum.Value.JSONType) then
      begin
      aSrc.Delete(aIdx);
      aIdx:=-1;
      end;
    if aIdx=-1 then
       aSrc.Add(aEnum.Key,aEnum.Value.Clone)
    else
       if (aSrc.Items[aIdx].JSONType=jtObject) and (aEnum.Value.JSONType=jtObject) then
         Apply(aSrc.Items[aIdx] as TJSONObject,aEnum.Value as TJSONObject);
    end;
  if RemoveNonExisting then
    begin
    for aIdx:=aSrc.Count-1 downto 0 do
      if aApply.IndexOfName(aSrc.Names[aIdx],CaseInsensitive)=-1 then
        aSrc.Delete(aIdx);
    end;
end;

procedure TJSONApplier.SaveDestJSON(aFileName: string);

Var
  F : TFileStream;

begin
  F:=TFileStream.Create(aFileName,fmCreate);
  try
    SaveDestJSON(F);
  finally
    F.Free;
  end;
end;

procedure TJSONApplier.SaveDestJSON(aStream: TStream);

Var
  S : TJSONStringType;

begin
  if Formatted then
    S:=DestJSON.FormatJSON()
  else
    S:=DestJSON.AsJSON;
  aStream.WriteBuffer(S[1],Length(S)*SizeOf(TJSONCharType));
end;

destructor TJSONApplier.destroy;
begin
  if FDestJSON<>FSourceJSON then
    FreeAndNil(FDestJSON);
  // Will free if needed
  SourceJSON:=Nil;
  ApplyJSON:=Nil;
  Inherited;
end;


procedure TJSONApplier.Execute;

  Function FindStart(aJSON : TJSONObject; aPath,aDesc : String) : TJSONObject;

  Var
    D : TJSONData;

  begin
    Result:=aJSON;
    if aPath='' then
      exit;
    D:=Result.FindPath(aPath);
    if (D=Nil) or Not (D is TJSONObject) then
      Raise EJSON.CreateFmt(SErrPathNotFound,[aPath,aDesc]);
    Result:=D as TJSONObject;
  end;

begin
  MaybeLoadSource;
  MaybeLoadApply;
  if (SourceJSON=Nil) then
    Raise EJSON.Create(SErrSourceEmpty);
  if (ApplyJSON=Nil) then
    Raise EJSON.Create(SErrApplyEmpty);
  if CloneSource then
    FDestJSON:=SourceJSON.Clone as TJSONObject
  else
    FDestJSON:=SourceJSON;
  Apply(FindStart(FDestJSON,SourcePath,'Source'),FindStart(ApplyJSON,ApplyPath,'Apply'));
  if (DestFileName<>'') then
    SaveDestJSON(DestFileName);
end;

end.

