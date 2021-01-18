unit fpdocproj;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;

Type

  { TFPDocPackage }

  TFPDocPackage = Class(TCollectionItem)
  private
    FContent: String;
    FDescriptions: TStrings;
    FImports: TStrings;
    FInputs: TStrings;
    FName: String;
    FOutput: String;
  Public
    constructor Create(ACollection: TCollection); override;
    destructor destroy; override;
    procedure Assign(Source : TPersistent); override;
    Property Name : String Read FName Write FName;
    Property Inputs : TStrings Read FinPuts;
    Property Descriptions : TStrings Read FDescriptions;
    Property Imports : TStrings read FIMports;
    Property ContentFile : String Read FContent Write FContent;
    Property Output : String Read FOutput Write FOutput;
  end;

  { TFPDocPackages }

  TFPDocPackages = Class(TCollection)
  private
    function GetP(AIndex : Integer): TFPDocPackage;
    procedure SetP(AIndex : Integer; const AValue: TFPDocPackage);
  Public
    Function IndexOfPackage(Const AName : String) : Integer;
    Function FindPackage(Const AName : String) : TFPDOcPackage;
    Property Packages[AIndex : Integer] : TFPDocPackage Read GetP Write SetP; Default;
  end;

  { TEngineOptions }

  TEngineOptions = Class(TPersistent)
  private
    FBackEndoptions: TStrings;
    FCPUTarget: String;
    FDefaultPackageName: String;
    FEmitNotes: Boolean;
    FEndianNess: String;
    FFormat: String;
    FHidePrivate: Boolean;
    FHideProtected: Boolean;
    FFallBackSeeAlsoLinks: Boolean;
    FIO: Boolean;
    FLanguage: String;
    FMoDir: String;
    FOSTarget: String;
    FSOPE: Boolean;
    FWarnDocumentationEmpty: Boolean;
    FWarnNoNode: Boolean;
    FDontTrim : Boolean;
    FInfoUsedFile: Boolean;
    FWarnXCT: Boolean;
    procedure SetBackendOptions(const AValue: TStrings);
  Public
    Constructor Create;
    Destructor Destroy; override;
    procedure Assign(Source : TPersistent); override;
  Published
    Property OSTarget : String Read FOSTarget Write FOStarget;
    Property CPUTarget : String Read FCPUTarget Write FCPUTarget;
    Property EndianNess : String Read FEndianNess Write FEndianNess;
    Property Language : String Read FLanguage Write fLanguage;
    Property Backend : String Read FFormat Write FFormat;
    Property BackendOptions : TStrings Read FBackEndoptions Write SetBackendOptions;
    Property StopOnParseError : Boolean Read FSOPE Write FSOPE;
    Property HideProtected : Boolean Read FHideProtected Write FHideProtected;
    Property FallBackSeeAlsoLinks :Boolean Read  FFallBackSeeAlsoLinks Write FFallBackSeeAlsoLinks;
    Property WarnNoNode : Boolean Read FWarnNoNode Write FWarnNoNode;
    Property InfoUsedFile : Boolean Read FInfoUsedFile Write FInfoUsedFile;
    Property WarnDocumentationEmpty : Boolean Read FWarnDocumentationEmpty Write FWarnDocumentationEmpty;
    Property WarnXCT : Boolean Read FWarnXCT Write FWarnXCT;
    Property ShowPrivate : Boolean Read FHidePrivate Write FHidePrivate;
    Property InterfaceOnly : Boolean Read FIO Write FIO;
    Property MoDir : String Read FMoDir Write FMODir;
    Property DefaultPackageName : String Read FDefaultPackageName Write FDefaultPackageName;
    Property DontTrim : Boolean Read FDontTrim Write FDontTrim;
    Property EmitNotes : Boolean Read FEmitNotes Write FEmitNotes;

  end;

  { TFPDocProject }

  TFPDocProject = Class(TComponent)
  private
    FOptions: TEngineOptions;
    FPackages: TFPDocPackages;
    procedure setOptions(const AValue: TEngineOptions);
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
  Published
    Property Packages : TFPDocPackages Read FPackages Write FPackages;
    Property Options : TEngineOptions Read FOptions Write setOptions;
  end;

Procedure SplitInputFileOption(Const AInputFile : String; Out AFile,AOption : String);

implementation

Procedure SplitInputFileOption(Const AInputFile : String; Out AFile,AOption : String);

  Function GetNextWord(Var s : string) : String;

  Const
    WhiteSpace = [' ',#9,#10,#13];

  var
    i,j: integer;

  begin
    I:=1;
    While (I<=Length(S)) and (S[i] in WhiteSpace) do
      Inc(I);
    J:=I;
    While (J<=Length(S)) and (not (S[J] in WhiteSpace)) do
      Inc(J);
    if (I<=Length(S)) then
      Result:=Copy(S,I,J-I);
    Delete(S,1,J);
  end;

Var
  S,W,F,O : String;

begin
  S:=AInputFile;
  O:='';
  F:='';
  While (S<>'') do
    begin
    W:=GetNextWord(S);
    If (W<>'') then
      begin
      if W[1]='-' then
        begin
        if (O<>'') then
          O:=O+' ';
        o:=O+W;
        end
      else
        F:=W;
      end;
    end;
  aFile:=F;
  AOption:=O;
end;

{ TEngineOptions }

procedure TEngineOptions.SetBackendOptions(const AValue: TStrings);
begin
  if FBackEndoptions=AValue then exit;
  FBackEndoptions.Assign(AValue);
end;

constructor TEngineOptions.Create;
begin
  FBackendOptions:=TStringList.Create;
end;

destructor TEngineOptions.Destroy;
begin
  FreeAndNil(FBackendOptions);
  inherited Destroy;
end;

procedure TEngineOptions.Assign(Source: TPersistent);

var
  O : TEngineOptions;

begin
  if (Source is TEngineOptions) then
    begin
    O:=Source as TEngineOptions;
    FBackEndoptions.Assign(O.BackendOptions);
    FCPUTarget:=O.CPUTarget;
    FFormat:=O.Backend;
    FLanguage:=O.Language;
    FOSTarget:=O.OSTarget;
    FSOPE:=O.StopOnParseError;
    HideProtected:=O.HideProtected;
    WarnNoNode:=O.WarnNoNode;
    InfoUsedFile:=O.InfoUsedFile;
    WarnDocumentationEmpty:=O.WarnDocumentationEmpty;
    WarnXCT:=O.WarnXCT;
    ShowPrivate:=O.ShowPrivate;
    InterfaceOnly:=O.InterfaceOnly;
    MoDir:=O.MoDir;
    end
  else
    inherited Assign(Source);
end;

{ TFPDocProject }

procedure TFPDocProject.setOptions(const AValue: TEngineOptions);
begin
  if FOptions=AValue then exit;
  FOptions.Assign(AValue);
end;

constructor TFPDocProject.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FPackages:=TFPDocPackages.Create(TFPDocPackage);
  FOptions:=TEngineOptions.Create;
end;

destructor TFPDocProject.Destroy;
begin
  FreeAndNil(Foptions);
  FreeAndNil(FPackages);
  inherited Destroy;
end;

{ TFPDocPackages }

function TFPDocPackages.GetP(AIndex : Integer): TFPDocPackage;
begin
  Result:=TFPDocPackage(Items[AIndex]);
end;

procedure TFPDocPackages.SetP(AIndex : Integer; const AValue: TFPDocPackage);
begin
  Items[AIndex]:=AValue;
end;

function TFPDocPackages.IndexOfPackage(const AName: String): Integer;

begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetP(Result).Name,AName)<>0) do
    Dec(Result)
end;

function TFPDocPackages.FindPackage(const AName: String): TFPDOcPackage;

Var
  I : Integer;

begin
  I:=IndexOfPackage(AName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetP(I);
end;

{ TFPDocPackage }

constructor TFPDocPackage.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FImports:=TStringList.Create;
  FDescriptions:=TStringList.Create;
  FInputs:=TStringList.Create;
end;

destructor TFPDocPackage.destroy;
begin
  FreeAndNil(FDescriptions);
  FreeAndNil(FIMports);
  FreeAndNil(FinPuts);
  inherited destroy;
end;

procedure TFPDocPackage.Assign(Source: TPersistent);

Var
  P : TFPDocPackage;

begin
  If Source is TFPDocPackage then
    begin
    P:=Source as TFPDocPackage;
    Fname:=P.Name;
    FContent:=P.ContentFile;
    FImports.Assign(P.Imports);
    FInputs.Assign(P.Inputs);
    FDescriptions.Assign(P.Descriptions);
    end
  else
    inherited Assign(Source);
end;

end.

