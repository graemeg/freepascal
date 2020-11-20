unit passrcutil;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, pscanner, pparser, pastree;

Type

  { TPasSrcAnalysis }

  TPasSrcAnalysis = class(TComponent)
  private
    FFilename : string;
    FResolver : TBaseFileResolver;
    FScanner  : TPascalScanner;
    FParser   : TPasParser;
    FModule   : TPasModule;
    FContainer : TPasTreeContainer;
    FStream: TStream;
    procedure SetFileName(AValue: string);
    Function ResourceStringCount(Section : TPasSection) : Integer;
  Protected
    Procedure FreeParser;
    Procedure CheckParser;
    Procedure Parse;
    procedure GetRecordFields(Rec: TPasrecordType; List: TStrings; const APrefix: String = ''); virtual;
    procedure GetClassMembers(AClass: TPasClassType; List: TStrings; AVisibilities : TPasMemberVisibilities; const APrefix: String = ''); virtual;
    procedure GetEnumValues(Enum: TPasEnumType; List: TStrings; const APrefix: String = ''); virtual;
    procedure GetIdentifiers(Section: TPasSection; List: TStrings; Recurse: Boolean);virtual;
    procedure GetUses(ASection: TPasSection; List: TStrings);virtual;
  Public
    Destructor Destroy; override;
    Procedure GetInterfaceUnits(List : TStrings);
    Procedure GetImplementationUnits(List : TStrings);
    Procedure GetUsedUnits(List : TStrings);
    Procedure GetInterfaceIdentifiers(List : TStrings; Recurse : Boolean = False);
    Procedure GetImplementationIdentifiers(List : TStrings; Recurse : Boolean = False);
    Procedure GetAllIdentifiers(List : TStrings; Recurse : Boolean = False);
    Function InterfaceHasResourcestrings : Boolean;
    Function ImplementationHasResourcestrings : Boolean;
    Function HasResourcestrings : Boolean;
    Property Stream : TStream Read FStream Write FStream;
  Published
    Property FileName : string Read FFilename Write SetFileName;
  end;



implementation

Type
  { TSrcContainer }
  TSrcContainer = Class(TPasTreeContainer)
  Public
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;overload; override;
    function FindElement(const AName: String): TPasElement; override;
  end;
  { TSrcContainer }

function TSrcContainer.CreateElement(AClass: TPTreeElement;
  const AName: String; AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
begin
  Result:=AClass.Create(AName,AParent);
  Result.Visibility:=AVisibility;
  Result.SourceFilename:=ASourceFileName;
  Result.SourceLinenumber:=ASourceLineNumber;
end;

function TSrcContainer.FindElement(const AName: String): TPasElement;
begin
  if AName='' then ;
  Result:=Nil;
end;

{ TPasSrcAnalysis }

procedure TPasSrcAnalysis.SetFileName(AValue: string);
begin
  if FFilename=AValue then Exit;
  FFilename:=AValue;
  FreeParser;
end;

function TPasSrcAnalysis.ResourceStringCount(Section: TPasSection): Integer;
begin
  Result:=0;
  If Assigned(Section) and Assigned(Section.ResStrings) then
   Result:=Section.ResStrings.Count;;
end;

procedure TPasSrcAnalysis.FreeParser;

begin
  FreeAndNil(FParser);
  FreeAndNil(FScanner);
  FreeAndNil(FContainer);
  FreeAndNil(FResolver);
  FreeAndNil(FModule);
end;

procedure TPasSrcAnalysis.CheckParser;

Var
  D : String;

begin
  If (FParser<>Nil) then
    exit;
  Try
    If Assigned(Stream) then
      begin
      FResolver:=TStreamResolver.Create;
      TStreamResolver(Fresolver).AddStream(FileName,Stream);
      end
    else
      FResolver:=TFileResolver.Create;
    D:=ExtractFilePath(FileName);
    If (D='') then
      D:='.';
    FResolver.ModuleDirectory:=D;
    FResolver.BaseDirectory:=D;

    FResolver.AddIncludePath(D); // still needed?

    FScanner:=TPascalScanner.Create(FResolver);
    FScanner.OpenFile(FileName);
    FContainer:=TSrcContainer.Create;
    FParser:=TPasParser.Create(FScanner,FResolver,FContainer);
    FScanner.AddDefine('FPC');
  except
    FreeParser;
    Raise;
  end;
end;

procedure TPasSrcAnalysis.Parse;
begin
  If FModule<>Nil then exit;
  CheckParser;
  FParser.ParseMain(FModule);
end;

procedure TPasSrcAnalysis.GetRecordFields(Rec: TPasrecordType; List: TStrings;
  const APrefix: String = '');

Var
  I : Integer;
  E : TPasElement;
  V : TPasVariant;

begin
  For I:=0 to Rec.Members.Count-1 do
    begin
    E:=TPasElement(Rec.Members[I]);
    if E<>Nil then
      List.Add(APrefix+E.Name);
    end;
  If Assigned(Rec.Variants) then
    For I:=0 to Rec.Variants.Count-1 do
      begin
      V:=TPasVariant(Rec.Variants[I]);
      if (v<>Nil) and (V.members<>Nil) then
        GetRecordFields(V.Members,List,APrefix);
      end;
end;

procedure TPasSrcAnalysis.GetClassMembers(AClass: TPasClassType; List: TStrings;
  AVisibilities: TPasMemberVisibilities; const APrefix: String);
Var
  I : Integer;
  E : TPasElement;
begin
  For I:=0 to AClass.Members.Count-1 do
    begin
    E:=TPasElement(AClass.Members[I]);
    if (E<>Nil) and ((AVisibilities=[]) or (E.Visibility in AVisibilities)) then
      List.Add(APrefix+E.Name);
    end;
end;

destructor TPasSrcAnalysis.Destroy;
begin
  FreeParser;
  inherited Destroy;
end;

procedure TPasSrcAnalysis.GetUses(ASection : TPasSection; List: TStrings);

Var
  I : Integer;
begin
  If not Assigned(ASection) then exit;
  if ASection.UsesList.Count=length(ASection.UsesClause) then
    For I:=0 to length(ASection.UsesClause)-1 do
      List.Add(ASection.UsesClause[i].Name)
  else
    For I:=0 to ASection.UsesList.Count-1 do
      List.Add(TPasElement(ASection.UsesList[i]).Name);
end;

procedure TPasSrcAnalysis.GetInterfaceUnits(List: TStrings);
begin
  Parse;
  GetUses(Fmodule.InterfaceSection,List);
end;

procedure TPasSrcAnalysis.GetImplementationUnits(List: TStrings);
begin
  Parse;
  GetUses(Fmodule.ImplementationSection,List);
end;

procedure TPasSrcAnalysis.GetUsedUnits(List: TStrings);
begin
  Parse;
  GetUses(Fmodule.InterfaceSection,List);
  GetUses(Fmodule.ImplementationSection,List);
end;

procedure TPasSrcAnalysis.GetEnumValues(Enum : TPasEnumType;List : TStrings; Const APrefix : String = '');

Var
  I : Integer;
  E : TPasElement;

begin
  For I:=0 to Enum.Values.Count-1 do
    begin
    E:=TPasElement(Enum.Values[I]);
    If (E<>Nil) then
      List.Add(APrefix+E.Name);
    end;
end;

procedure TPasSrcAnalysis.GetIdentifiers(Section : TPasSection; List: TStrings; Recurse : Boolean);

Var
  I : Integer;
  E : TPasElement;

begin
  if not (Assigned(Section) and Assigned(Section.Declarations)) then
    Exit;
  For I:=0 to Section.Declarations.Count-1 do
    begin
    E:=TPasElement(Section.Declarations[I]);
    If (E.Name<>'') then
      List.Add(E.Name);
    if Recurse then
      begin
      If E is TPasEnumType then
        GetEnumValues(TPasEnumType(E),List,E.Name+'.')
      else if E is TPasRecordType then
        GetRecordFields(TPasRecordType(E),List,E.Name+'.')
      else if E is TPasClassType then
        GetClassMembers(TPasClassType(E),List,[],E.Name+'.')
      end;
    end;
end;

procedure TPasSrcAnalysis.GetInterfaceIdentifiers(List: TStrings; Recurse : Boolean = False);
begin
  Parse;
  GetIdentifiers(Fmodule.InterfaceSection,List,Recurse);
end;

procedure TPasSrcAnalysis.GetImplementationIdentifiers(List: TStrings;
  Recurse: Boolean);
begin
  Parse;
  GetIdentifiers(Fmodule.ImplementationSection,List,Recurse);
end;

procedure TPasSrcAnalysis.GetAllIdentifiers(List: TStrings; Recurse: Boolean);
begin
  Parse;
  GetIdentifiers(Fmodule.InterfaceSection,List,Recurse);
  GetIdentifiers(Fmodule.ImplementationSection,List,Recurse);
end;

function TPasSrcAnalysis.InterfaceHasResourcestrings: Boolean;
begin
  Parse;
  Result:=ResourceStringCount(Fmodule.InterfaceSection)>0;
end;

function TPasSrcAnalysis.ImplementationHasResourcestrings: Boolean;
begin
  Parse;
  Result:=ResourceStringCount(Fmodule.ImplementationSection)>0;
end;

function TPasSrcAnalysis.HasResourcestrings: Boolean;
begin
  Parse;
  Result:=(ResourceStringCount(Fmodule.InterfaceSection)>0)
           or (ResourceStringCount(Fmodule.ImplementationSection)>0);
end;

end.

