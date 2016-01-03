unit fpdocxmlopts;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, fpdocproj, dom, fptemplate;

Type
  { TXMLFPDocOptions }

  TXMLFPDocOptions = Class(TComponent)
  private
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Classes, SysUtils, fpdocproj, dom;

Type

  { TXMLFPocOptions }

  { TXMLFPDocOptions }

  TXMLFPDocOptions = Class(TComponent)
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
  Protected
    Procedure Error(Const Msg : String);
    Procedure Error(Const Fmt : String; Args : Array of Const);
    Procedure LoadPackage(APackage : TFPDocPackage; E : TDOMElement); virtual;
    Procedure LoadPackages(Packages : TFPDocPackages; E : TDOMElement);
    Procedure LoadEngineOptions(Options : TEngineOptions; E : TDOMElement); virtual;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure SaveEngineOptions(Options : TEngineOptions; XML : TXMLDocument; AParent : TDOMElement); virtual;
    procedure SaveDescription(const ADescription: String; XML: TXMLDocument;  AParent: TDOMElement); virtual;
    procedure SaveImportFile(const AImportFile: String; XML: TXMLDocument; AParent: TDOMElement);virtual;
    procedure SaveInputFile(const AInputFile: String; XML: TXMLDocument; AParent: TDOMElement);virtual;
    Procedure SavePackage(APackage : TFPDocPackage; XML : TXMLDocument; AParent : TDOMElement); virtual;
  Public
    Procedure LoadOptionsFromFile(AProject : TFPDocProject; Const AFileName : String);
    Procedure LoadFromXML(AProject : TFPDocProject; XML : TXMLDocument); virtual;
    Procedure SaveOptionsToFile(AProject : TFPDocProject; Const AFileName : String);
    procedure SaveToXML(AProject : TFPDocProject; ADoc: TXMLDocument); virtual;
  end;
  EXMLFPdoc = Class(Exception);

Function IndexOfString(S : String; List : Array of string) : Integer;

Const
  OptionCount = 12;
  OptionNames : Array[0..OptionCount] of string
         = ('hide-protected','warn-no-node','show-private',
            'stop-on-parser-error', 'ostarget','cputarget',
            'mo-dir','parse-impl','format', 'language',
            'package','dont-trim','emit-notes');

implementation

Uses XMLRead, XMLWrite;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Public
    Procedure LoadOptionsFromFile(AProject : TFPDocProject; Const AFileName : String);
    Procedure LoadFromXML(AProject : TFPDocProject; XML : TXMLDocument); virtual;
  end;
  EXMLFPdoc = Class(Exception);

implementation

Uses XMLRead;
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

Resourcestring
  SErrInvalidRootNode = 'Invalid options root node: Got "%s", expected "docproject"';
  SErrNoPackagesNode = 'No "packages" node found in docproject';
  SErrNoInputFile = 'unit tag without file attribute found';
  SErrNoDescrFile = 'description tag without file attribute';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrNoImportFile = 'Import tag without file attribute';
  SErrNoImportPrefix = 'Import tag without prefix attribute';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ TXMLFPDocOptions }

Function IndexOfString(S : String; List : Array of string) : Integer;

begin
  S:=UpperCase(S);
  Result:=High(List);
  While (Result>=0) and (S<>UpperCase(List[Result])) do
    Dec(Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TXMLFPDocOptions.Error(Const Msg: String);
begin
  Raise EXMLFPDoc.Create(Msg);
end;

procedure TXMLFPDocOptions.Error(const Fmt: String; Args: array of const);
begin
  Raise EXMLFPDoc.CreateFmt(Fmt,Args);
end;



procedure TXMLFPDocOptions.LoadPackage(APackage: TFPDocPackage; E: TDOMElement);

  Function LoadInput(I : TDOMElement) : String;

  Var
    S : String;

  begin
    Result:=I['file'];
    If (Result='') then
      Error(SErrNoInputFile);
    S:=I['options'];
    if (S<>'') then
      Result:=S+' '+Result;
  end;

  Function LoadDescription(I : TDOMElement) : String;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  begin
    Result:=I['file'];
    If (Result='') then
      Error(SErrNoDescrFile);
  end;

  Function LoadImport(I : TDOMElement) : String;

  Var
    S : String;
  begin
    Result:=I['file'];
    If (Result='') then
      Error(SErrNoImportFile);
    S:=I['prefix'];
    If (S='') then
      Error(SErrNoImportPrefix);
    Result:=Result+','+S;
  end;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Var
    S : String;

  begin
    Result:=I['file'];
    If (Result='') then
      Error(SErrNoDescrFile);
  end;

Const
  OpCount = 0;
  OpNames : Array[0..OpCount] of string
          = ('');
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
Var
  N,S : TDOMNode;
  O : TDomElement;

begin
  APackage.Name:=E['name'];
  APackage.output:=E['output'];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  APackage.ContentFile:=E['content'];
=======
  APackage.ContentFile:=E['contentfile'];
>>>>>>> graemeg/cpstrnew
=======
  APackage.ContentFile:=E['contentfile'];
>>>>>>> graemeg/cpstrnew
=======
  APackage.ContentFile:=E['contentfile'];
>>>>>>> graemeg/cpstrnew
=======
  APackage.ContentFile:=E['contentfile'];
>>>>>>> origin/cpstrnew
  N:=E.FirstChild;
  While (N<>Nil) do
    begin
    If (N.NodeType=ELEMENT_NODE) then
      begin
      O:=N as TDOMElement;
      If (O.NodeName='units') then
        begin
        S:=O.FirstChild;
        While (S<>Nil) do
          begin
          If (S.NodeType=Element_Node) and (S.NodeName='unit') then
            APackage.Inputs.add(LoadInput(S as TDomElement));
          S:=S.NextSibling;
          end;
        end
      else If (O.NodeName='descriptions') then
        begin
        S:=O.FirstChild;
        While (S<>Nil) do
          begin
          If (S.NodeType=Element_Node) and (S.NodeName='description') then
            APackage.Descriptions.add(LoadDescription(S as TDomElement));
          S:=S.NextSibling;
          end;
        end
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      else If (O.NodeName='imports') then
        begin
        S:=O.FirstChild;
        While (S<>Nil) do
          begin
          If (S.NodeType=Element_Node) and (S.NodeName='import') then
            APackage.Imports.add(LoadImport(S as TDomElement));
          S:=S.NextSibling;
          end;
        end
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;
    N:=N.NextSibling;
    end;
end;

procedure TXMLFPDocOptions.LoadPackages(Packages: TFPDocPackages; E: TDOMElement
  );

Var
  N : TDOMNode;

begin
  N:=E.FirstChild;
  While (N<>Nil) do
    begin
    If (N.NodeName='package') and (N.NodeType=ELEMENT_NODE) then
      LoadPackage(Packages.Add as TFPDocPackage, N as TDOMElement);
    N:=N.NextSibling;
    end;
end;

procedure TXMLFPDocOptions.LoadEngineOptions(Options: TEngineOptions;
  E: TDOMElement);

  Function TrueValue(V : String) : Boolean;

  begin
    V:=LowerCase(V);
    Result:=(v='true') or (v='1') or (v='yes');
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Const
  NCount = 10;
  ONames : Array[0..NCount] of string
         = ('hide-protected','warn-no-node','show-private',
            'stop-on-parser-error', 'ostarget','cputarget',
            'mo-dir','parse-impl','format', 'language',
            'package');
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

Var
  O : TDOMnode;
  N,V : String;

begin
  O:=E.FirstChild;
  While (O<>Nil) do
    begin
    If (O.NodeType=Element_NODE) and (O.NodeName='option') then
      begin
      N:=LowerCase(TDOMElement(o)['name']);
      V:=TDOMElement(o)['value'];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Case IndexOfString(N,OptionNames) of
=======
      Case IndexOfString(N,ONames) of
>>>>>>> graemeg/cpstrnew
=======
      Case IndexOfString(N,ONames) of
>>>>>>> graemeg/cpstrnew
=======
      Case IndexOfString(N,ONames) of
>>>>>>> graemeg/cpstrnew
=======
      Case IndexOfString(N,ONames) of
>>>>>>> origin/cpstrnew
        0 : Options.HideProtected:=TrueValue(v);
        1 : Options.WarnNoNode:=TrueValue(v);
        2 : Options.ShowPrivate:=TrueValue(v);
        3 : Options.StopOnParseError:=TrueValue(v);
        4 : Options.ostarget:=v;
        5 : Options.cputarget:=v;
        6 : Options.MoDir:=V;
        7 : Options.InterfaceOnly:=Not TrueValue(V);
        8 : Options.Backend:=V;
        9 : Options.Language:=v;
        10 : Options.DefaultPackageName:=V;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        11 : Options.DontTrim:=TrueValue(V);
        12 : Options.EmitNotes:=TrueValue(V);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      else
        Options.BackendOptions.add('--'+n);
        Options.BackendOptions.add(v);
      end;
      end;
    O:=O.NextSibling
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLFPDocOptions.SaveToXML(AProject: TFPDocProject; ADoc: TXMLDocument);

var
  i: integer;
  E,PE: TDOMElement;

begin
  E:=ADoc.CreateElement('docproject');
  ADoc.AppendChild(E);
  E:=ADoc.CreateElement('options');
  ADoc.DocumentElement.AppendChild(E);
  SaveEngineOptions(AProject.Options,ADoc,E);
  E:=ADoc.CreateElement('packages');
  ADoc.DocumentElement.AppendChild(E);
  for i := 0 to AProject.Packages.Count - 1 do
    begin
    PE:=ADoc.CreateElement('package');
    E.AppendChild(PE);
    SavePackage(AProject.Packages[i],ADoc,PE);
    end;
end;

Procedure TXMLFPDocOptions.SaveEngineOptions(Options : TEngineOptions; XML : TXMLDocument; AParent : TDOMElement);

  procedure AddStr(const n, v: string);
  var
    E : TDOMElement;
  begin
    if (v='') then
      Exit;
    E:=XML.CreateElement('option');
    AParent.AppendChild(E);
    E['name'] := n;
    E['value'] := v;
  end;

  procedure AddBool(const AName: string; B: Boolean);

  begin
    if B then
      AddStr(Aname,'true')
    else
      AddStr(Aname,'false');
  end;

begin
  AddStr('ostarget', Options.OSTarget);
  AddStr('cputarget', Options.CPUTarget);
  AddStr('mo-dir', Options.MoDir);
  AddStr('format', Options.Backend);
  AddStr('language', Options.Language);
  AddStr('package', Options.DefaultPackageName);
  AddBool('hide-protected', Options.HideProtected);
  AddBool('warn-no-node', Options.WarnNoNode);
  AddBool('show-private', Options.ShowPrivate);
  AddBool('stop-on-parser-error', Options.StopOnParseError);
  AddBool('parse-impl', Options.InterfaceOnly);
  AddBool('dont-trim', Options.DontTrim);
  AddBool('emit-notes', Options.EmitNotes);
end;


Procedure TXMLFPDocOptions.SaveInputFile(Const AInputFile : String; XML : TXMLDocument; AParent: TDOMElement);

Var
  F,O : String;

begin
  SplitInputFileOption(AInputFile,F,O);
  AParent['file']:=F;
  AParent['options']:=O;
end;

Procedure TXMLFPDocOptions.SaveDescription(Const ADescription : String; XML : TXMLDocument; AParent: TDOMElement);

begin
  AParent['file']:=ADescription;
end;

procedure TXMLFPDocOptions.SaveImportFile(const AImportFile: String;
  XML: TXMLDocument; AParent: TDOMElement);

Var
  I : integer;

begin
  I:=Pos(',',AImportFile);
  AParent['file']:=Copy(AImportFile,1,I-1);
  AParent['prefix']:=Copy(AImportFile,i+1,Length(AImportFile));
end;

Procedure TXMLFPDocOptions.SavePackage(APackage: TFPDocPackage; XML : TXMLDocument; AParent: TDOMElement);


var
  i: integer;
  E,PE : TDomElement;

begin
  AParent['name']:=APackage.Name;
  AParent['output']:=APackage.Output;
  AParent['content']:=APackage.ContentFile;
  // Units
  PE:=XML.CreateElement('units');
  AParent.AppendChild(PE);
  for i:=0 to APackage.Inputs.Count-1 do
    begin
    E:=XML.CreateElement('unit');
    PE.AppendChild(E);
    SaveInputFile(APackage.Inputs[i],XML,E);
    end;
  // Descriptions
  PE:=XML.CreateElement('descriptions');
  AParent.AppendChild(PE);
  for i:=0 to APackage.Descriptions.Count-1 do
    begin
    E:=XML.CreateElement('description');
    PE.AppendChild(E);
    SaveDescription(APackage.Descriptions[i],XML,E);
    end;
  // Imports
  PE:=XML.CreateElement('imports');
  AParent.AppendChild(PE);
  for i:=0 to APackage.Imports.Count-1 do
    begin
    E:=XML.CreateElement('import');
    PE.AppendChild(E);
    SaveImportFile(APackage.Imports[i],XML,E);
    end;
end;



procedure TXMLFPDocOptions.LoadOptionsFromFile(AProject: TFPDocProject; const AFileName: String);
=======
procedure TXMLFPDocOptions.LoadOptionsFromFile(AProject: TFPDocProject;
  const AFileName: String);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLFPDocOptions.LoadOptionsFromFile(AProject: TFPDocProject;
  const AFileName: String);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLFPDocOptions.LoadOptionsFromFile(AProject: TFPDocProject;
  const AFileName: String);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLFPDocOptions.LoadOptionsFromFile(AProject: TFPDocProject;
  const AFileName: String);
>>>>>>> origin/cpstrnew

Var
  XML : TXMLDocument;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ReadXMLFile(XML,AFileName);
  try
    LoadFromXML(AProject,XML);
  finally
    FreeAndNil(XML);
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   XMLRead.ReadXMLFile(XML,AFileName);
   try
     LoadFromXML(AProject,XML);
   finally
     FreeAndNil(XML);
   end;
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
end;

procedure TXMLFPDocOptions.LoadFromXML(AProject: TFPDocProject;
  XML: TXMLDocument);

Var
  E : TDOMElement;
  N : TDomNode;

begin
  E:=XML.DocumentElement;
  if (E.NodeName<>'docproject') then
    Error(SErrInvalidRootNode,[E.NodeName]);
  N:=E.FindNode('packages');
  If (N=Nil) or (N.NodeType<>ELEMENT_NODE) then
    Error(SErrNoPackagesNode);
  LoadPackages(AProject.Packages,N as TDomElement);
  N:=E.FindNode('options');
  If (N<>Nil) and (N.NodeType=ELEMENT_NODE) then
    LoadEngineOptions(AProject.Options,N as TDOMElement);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TXMLFPDocOptions.SaveOptionsToFile(AProject: TFPDocProject; const AFileName: String);

Var
  XML : TXMLDocument;

begin
  XML:=TXMLDocument.Create;
  try
    SaveToXML(AProject,XML);
    WriteXMLFile(XML, AFileName);
  finally
    XML.Free;
  end;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end.

