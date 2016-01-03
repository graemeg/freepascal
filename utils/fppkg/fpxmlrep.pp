{
    This file is part of the Free Pascal Utilities
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpxmlrep;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, dom, fprepos;

Type

  { TFPXMLHandler }

  TFPXMLHandler = Class(TObject)
  private
    FIgnoreUnknownNodes: Boolean;
  Protected
    Function AddTextNode(Const NodeName,NodeContent : String; XML : TXMLDocument; Parent : TDomNode_WithChildren) : TDomElement;
    Function GetNextElement(Start : TDomNode) : TDomElement;
    Function FindNextElement(Start : TDomNode; NodeName : String) : TDomElement;
    Function NodeText(E : TDomElement) : String;
    procedure CheckNodeType(E : TDomElement; NodeName : String);
  public
    property IgnoreUnknownNodes : Boolean Read FIgnoreUnknownNodes Write FIgnoreUnknownNodes;
  end;

  { TFPXMLRepositoryHandler }

  TFPXMLRepositoryHandler = Class(TFPXMLHandler)
  private
    // The DO versions do not check the node type. They do the actual work
    // for the public XMLtoXYZ versions..
    Function  DoXMLToCPUs(N : TDomElement) : TCPUs; virtual;
    Function  DoXMLToOSes(N : TDomElement) : TOSes; virtual;
    Procedure DoXMLToVersion(E : TDomElement; V : TFPVersion); virtual;
    Procedure DoXMLToDependency(E : TDomElement; D : TFPDependency); virtual;
    Procedure DoXMLToDependencies(E : TDomElement; DS : TFPDependencies); virtual;
    Procedure DoXMLToPackage(E : TDomElement; P : TFPPackage); virtual;
    Procedure DoXMLToPackages(E : TDomElement; PS: TFPPackages); virtual;
    Procedure DoXMLToRepository(E : TDomElement; R: TFPRepository);  virtual;
  Public
    // Saving
    Function VersionToXML(V : TFPVersion; XML : TXMLDocument; Parent : TDomNode_WithChildren) : TDomElement; virtual;
    function OSesToXML(OSes:TOSes; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement; virtual;
    function CPUsToXML(CPUs:TCPUs; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement; virtual;
    Function DependencyToXML(D : TFPDependency; XML : TXMLDocument; Parent : TDomNode_WithChildren) : TDomElement; virtual;
    Function DependenciesToXML(DS : TFPDependencies; XML : TXMLDocument; Parent : TDomNode_WithChildren) : TDomElement; virtual;
    Function PackageToXML(P : TFPPackage; XML : TXMLDocument; Parent : TDomNode_WithChildren) : TDomElement; virtual;
    Function PackagesToXML(PS: TFPPackages; XML : TXMLDocument; Parent : TDomNode_WithChildren) : TDomElement; virtual;
    Function RepositoryToXML(R : TFPRepository; XML : TXMLDocument; Parent : TDomNode_WithChildren) : TDomElement;  virtual;
    Procedure SaveToXml(D : TFPDependency; Stream : TStream);
    Procedure SaveToXml(DS : TFPDependencies; Stream : TStream);
    Procedure SaveToXml(P : TFPPackage; Stream : TStream);
    Procedure SaveToXml(PS : TFPPackages; Stream : TStream);
    Procedure SaveToXml(R : TFPRepository; Stream : TStream);
    Procedure SaveToXml(D : TFPDependency; const FileName: String);
    Procedure SaveToXml(DS : TFPDependencies; const FileName: String);
    Procedure SaveToXml(P : TFPPackage; const FileName: String);
    Procedure SaveToXml(PS : TFPPackages; const FileName: String);
    Procedure SaveToXml(R : TFPRepository; const FileName: String);
    // Loading
    Procedure XMLToVersion(E : TDomElement; V : TFPVersion);
    Procedure XMLToDependency(E : TDomElement; D : TFPDependency);
    Procedure XMLToDependencies(E : TDomElement; DS : TFPDependencies);
    Procedure XMLToPackage(E : TDomElement; P : TFPPackage);
    Procedure XMLToPackages(E : TDomElement; PS: TFPPackages);
    Procedure XMLToRepository(E : TDomElement; R: TFPRepository);
    Procedure LoadFromXml(D : TFPDependency; Stream : TStream);
    Procedure LoadFromXml(DS : TFPDependencies; Stream : TStream);
    Procedure LoadFromXml(P : TFPPackage; Stream : TStream);
    Procedure LoadFromXml(PS : TFPPackages; Stream : TStream);
    Procedure LoadFromXml(R : TFPRepository; Stream : TStream);
    Procedure LoadFromXml(D : TFPDependency; const FileName: String);
    Procedure LoadFromXml(DS : TFPDependencies; const FileName: String);
    Procedure LoadFromXml(P : TFPPackage; const FileName: String);
    Procedure LoadFromXml(PS : TFPPackages; const FileName: String);
    Procedure LoadFromXml(R : TFPRepository; const FileName: String);
  end;

  { TFPXMLMirrorHandler }

  TFPXMLMirrorHandler = Class(TFPXMLHandler)
  private
    procedure DoXMLToMirror(E: TDomElement; P: TFPMirror);
    procedure DoXMLToMirrors(E: TDomElement; PS: TFPMirrors);
  Public
    // Loading
    Procedure XMLToMirrors(E : TDomElement; PS: TFPMirrors);
    Procedure LoadFromXml(PS : TFPMirrors; Stream : TStream);
    Procedure LoadFromXml(PS : TFPMirrors; const FileName : String);
  end;

  EXMLPackage = Class(EPackage);

implementation

uses xmlwrite,xmlread,typinfo;

Const
  SNodeRepository   = 'repository';
  SNodePackages     = 'packages';
  SNodePackage      = 'package';
  SNodeAuthor       = 'author';
  SNodeLicense      = 'license';
  SNodeHomepageURL  = 'homepageurl';
  SNodeDownloadURL  = 'downloadurl';
  SNodeFileName     = 'filename';
  SNodeEmail        = 'email';
  SNodeVersion      = 'version';
  SNodeDescription  = 'description';
  SNodeDependencies = 'dependencies';
  SNodeDependency   = 'dependency';
  SNodeOSes         = 'oses';
  SNodeCPUS         = 'cpus';
  SNodeOS           = 'os';
  SNodeCPU          = 'cpu';

  // Mirrors
  SNodeURL          = 'url';
  SNodeContact      = 'contact';
  SNodeWeight       = 'weight';
  SNodeMirror       = 'mirror';
  SNodeMirrors      = 'mirrors';

  SAttrName         = 'name';
  SAttrPackageName  = 'packagename';
  SAttrMinVersion   = 'minversion';
  SAttrMajor        = 'major';
  SAttrMinor        = 'minor';
  SAttrMicro        = 'micro';
  SAttrBuild        = 'build';

ResourceString
  SErrInvalidXMLDocument = 'Wrong root tag in XML document. Expected "%s", got "%s".';
  SErrUnknownNode = 'Unknown XML tag ("%s") encountered while reading %s "%s".';


{****************************************************************************
                             TFPXMLHandler
****************************************************************************}

function TFPXMLHandler.AddTextNode(Const NodeName,NodeContent : String; XML : TXMLDocument; Parent : TDomNode_WithChildren) : TDomElement;
begin
  Result:=XML.CreateElement(NodeName);
  Try
    Result.AppendChild(XML.CreateTextNode(NodeContent));
    If Assigned(Parent) then
      Parent.AppendChild(Result);
  Except
    Parent.RemoveChild(Result);
    Result.Free;
    Raise;
  end;
end;


function TFPXMLHandler.GetNextElement(Start: TDomNode): TDomElement;
begin
  Result:=Nil;
  While (Start<>Nil) and (Start.NodeType<>ELEMENT_NODE) do
    Start:=Start.NextSibling;
  If (Start<>Nil) then
    Result:=Start as TDomElement;
end;


function TFPXMLHandler.FindNextElement(Start: TDomNode; NodeName: String): TDomElement;
begin
  Result:=GetNextElement(Start);
  While (Result<>Nil) and (Result.NodeName<>NodeName) do
    Result:=GetNextElement(Result.NextSibling);
end;


procedure TFPXMLHandler.CheckNodeType(E : TDomElement; NodeName : String);
begin
  If (E.NodeName<>NodeName) then
    Raise EXMLPackage.CreateFmt(SErrInvalidXMLDocument,[NodeName,E.NodeName]);
end;


Function TFPXMLHandler.NodeText(E : TDomElement) : String;
Var
  N : TDomNode;
begin
  N:=E.FirstChild;
  While (N<>Nil) and (N.NodeType<>TEXT_NODE) do
    N:=N.NextSibling;
  If (N<>Nil) then
    Result:=N.NodeValue;
end;


{****************************************************************************
                        TFPXMLRepositoryHandler
****************************************************************************}

function TFPXMLRepositoryHandler.VersionToXML(V : TFPVersion; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement;
begin
  Result:=XML.CreateElement(SNodeVersion);
  try
    If Not Assigned(Parent) then
      Parent:=XML;
    Parent.AppendChild(Result);
    Result[SAttrMajor]:=IntToStr(V.Major);
    Result[SAttrMinor]:=IntToStr(V.Minor);
    Result[SAttrMicro]:=IntToStr(V.Micro);
    Result[SAttrBuild]:=IntToStr(V.Build);
  except
    Parent.RemoveChild(Result);
    Result.Free;
    Raise;
  end;
end;


function TFPXMLRepositoryHandler.DependencyToXML(D: TFPDependency; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement;
Var
  E : TDOMElement;
begin
  Result:=XML.CreateElement(SNodeDependency);
  Try
    If Not Assigned(Parent) then
      Parent:=XML;
    Parent.AppendChild(Result);
    E:=XML.CreateElement(SNodePackage);
    E[SAttrPackageName]:=D.PackageName;
    Result.AppendChild(E);
    if not D.MinVersion.Empty then
      VersionToXML(D.MinVersion,XML,Result);
    if D.OSes<>AllOSes then
      OSesToXML(D.OSes,XML,Result);
    if D.CPUs<>AllCPUs then
      CPUsToXML(D.CPUs,XML,Result);
  Except
    Parent.RemoveChild(Result);
    Result.Free;
    Raise;
  end;
end;


function TFPXMLRepositoryHandler.DependenciesToXML(DS: TFPDependencies; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement;
Var
  I : Integer;
begin
  If (DS.Count>0) then
    begin
    Result:=XML.CreateElement(SNodeDependencies);
    Try
      If Not Assigned(Parent) then
        Parent:=XML;
      Parent.AppendChild(Result);
      For I:=0 to DS.Count-1 do
        DependencyToXML(DS[i],XML,Result);
    except
      Parent.RemoveChild(Result);
      Result.Free;
      Raise;
    end;
    end;
end;


function TFPXMLRepositoryHandler.OSesToXML(OSes:TOSes; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement;
Var
  ES : TDomElement;
  O : TOS;
begin
  Result:=XML.CreateElement(SNodeOSes);
  Parent.AppendChild(Result);
  For O:=Low(TOS) to High(TOS) do
    If (O in OSes) then
      begin
        ES:=XML.CreateElement(SNodeOS);
        ES[SAttrName]:=GetEnumName(TypeInfo(TOS),Ord(O));
        Result.AppendChild(ES);
      end;
end;


function TFPXMLRepositoryHandler.CPUsToXML(CPUs:TCPUs; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement;
Var
  ES : TDomElement;
  C : TCPU;
begin
  Result:=XML.CreateElement(SNodeCPUs);
  Parent.AppendChild(Result);
  For C:=Low(TCPU) to High(TCPU) do
    If (C in CPUs) then
      begin
        ES:=XML.CreateElement(SNodeCPU);
        ES[SAttrName]:=GetEnumName(TypeInfo(TCPU),Ord(C));
        Result.AppendChild(ES);
      end;
end;


function TFPXMLRepositoryHandler.PackageToXML(P: TFPPackage; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement;
begin
  Result:=XML.CreateElement(SNodePackage);
  Try
    If Not Assigned(Parent) then
      Parent:=XMl;
    Parent.AppendChild(Result);
    Result[SAttrName]:=P.Name;
    // Version
    VersionToXML(P.Version,XML,Result);
    AddTextNode(SNodeAuthor,P.Author,XML,Result);
    AddTextNode(SNodeHomepageURL,P.HomepageURL,XML,Result);
    AddTextNode(SNodeDownloadURL,P.DownloadURL,XML,Result);
    AddTextNode(SNodeFileName,P.FileName,XML,Result);
    AddTextNode(SNodeEmail,P.Email,XML,Result);
    AddTextNode(SNodeDescription,P.Description,XML,Result);
    AddTextNode(SNodeLicense,P.License,XML,Result);
    if P.OSes<>AllOSes then
      OSesToXML(P.OSes,XML,Result);
    if P.CPUs<>AllCPUs then
      CPUsToXML(P.CPUs,XML,Result);
    If P.Dependencies.Count>0 then
      DependenciesToXML(P.Dependencies,XML,Result);
  Except
    Parent.RemoveChild(Result);
    Result.Free;
    Raise;
  end;
end;


function TFPXMLRepositoryHandler.PackagesToXML(PS: TFPPackages; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement;
Var
  I : Integer;
begin
  Result:=XML.CreateElement(SNodePackages);
  Try
    If Not Assigned(Parent) then
      Parent:=XML;
    Parent.AppendChild(Result);
    For I:=0 to PS.Count-1 do
      PackageToXML(PS[i],XML,Result);
  except
    Parent.RemoveChild(Result);
    Result.Free;
    Raise;
  end;
end;


function TFPXMLRepositoryHandler.RepositoryToXML(R: TFPRepository; XML: TXMLDocument; Parent : TDomNode_WithChildren): TDomElement;
Var
  I : Integer;
  P : TDomElement;
begin
  Result:=XML.CreateElement(sNodeRepository);
  Try
    If Not Assigned(Parent) then
      Parent:=XML;
    Parent.AppendChild(Result);
    If (R.PackageCount>0) then
      begin
      P:=XML.CreateElement(SNodePackages);
      Try
        Result.AppendChild(P);
        For I:=0 to R.PackageCount-1 do
          PackageToXML(R.Packages[i],XML,P);
      except
        Result.RemoveChild(P);
        P.Free;
        Raise;
      end;
      end;
  except
    Parent.RemoveChild(Result);
    Result.Free;
    Raise;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(D: TFPDependency; Stream: TStream);
Var
  X : TXMLDocument;
begin
  X:=TXMLDocument.Create;
  Try
    DependencyToXML(D,X,Nil);
    WriteXMLFile(X,Stream);
  Finally
    D.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(DS: TFPDependencies; Stream: TStream);
Var
  D : TXMLDocument;
begin
  D:=TXMLDocument.Create;
  Try
    DependenciesToXML(DS,D,Nil);
    WriteXMLFile(D,Stream);
  Finally
    D.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(P: TFPPackage; Stream: TStream);
Var
  D : TXMLDocument;
begin
  D:=TXMLDocument.Create;
  Try
    PackageToXML(P,D,Nil);
    WriteXMLFile(D,Stream);
  Finally
    D.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(PS: TFPPackages; Stream: TStream);
Var
  D : TXMLDocument;
begin
  D:=TXMLDocument.Create;
  Try
    PackagesToXML(PS,D,Nil);
    WriteXMLFile(D,Stream);
  Finally
    D.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(R: TFPRepository; Stream: TStream);
Var
  D : TXMLDocument;
begin
  D:=TXMLDocument.Create;
  Try
    RepositoryToXML(R,D,Nil);
    WriteXMLFile(D,Stream);
  Finally
    D.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(D: TFPDependency; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToXML(D,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(DS: TFPDependencies; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToXML(DS,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(P: TFPPackage; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToXML(P,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(PS: TFPPackages; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToXML(PS,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.SaveToXml(R: TFPRepository; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToXML(R,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.XMLToVersion(E: TDomElement; V: TFPVersion);
begin
  CheckNodeType(E,SNodeVersion);
  DoXMLToVersion(E,V);
end;


procedure TFPXMLRepositoryHandler.DoXMLToVersion(E: TDomElement; V: TFPVersion);
begin
  V.Major:=Abs(StrToIntDef(E[SAttrMajor],0));
  V.Minor:=Abs(StrToIntDef(E[SAttrMinor],0));
  V.Micro:=Abs(StrToIntDef(E[SAttrMicro],0));
  V.Build:=Abs(StrToIntDef(E[SAttrBuild],0));
end;


procedure TFPXMLRepositoryHandler.XMLToDependency(E: TDomElement; D: TFPDependency);
begin
  CheckNodeType(E,SNodeDependency);
  DoXMLToDependency(E,D);
end;


procedure TFPXMLRepositoryHandler.DoXMLToDependency(E: TDomElement; D: TFPDependency);
var
  N : TDomElement;
begin
  N:=GetNextElement(E.FirstChild);
  While (N<>Nil) do
    begin
      if (N.NodeName=sNodePackage) then
        D.PackageName:=N[SAttrPackageName]
      else if (N.NodeName=sNodeVersion) then
        DoXMlToVersion(N,D.MinVersion)
      else if (N.NodeName=sNodeOSes) then
        D.OSes:=DoXMLToOSes(N)
      else if (N.NodeName=sNodeCPUS) then
        D.CPUs:=DoXMLToCPUs(N)
      else if Not IgnoreUnknownNodes then
        Raise EXMLPackage.CreateFmt(SErrUnknownNode,[N.NodeName,sNodeDependency,'']);
      N:=GetNextElement(N.NextSibling);
    end;
end;


procedure TFPXMLRepositoryHandler.XMLToDependencies(E: TDomElement; DS: TFPDependencies);
begin
  CheckNodeType(E,SNodeDependencies);
  DoXMLToDependencies(E,DS)
end;


procedure TFPXMLRepositoryHandler.DoXMLToDependencies(E: TDomElement; DS: TFPDependencies);
Var
  DN : TDomElement;
  D : TFPDependency;
begin
  DN:=FindNextElement(E.FirstChild,SNodeDependency);
  While (DN<>Nil) do
    begin
      D:=DS.AddDependency('','');
      Try
        DoXMLToDependency(DN,D);
      except
        D.Free;
        Raise;
      end;
      DN:=FindNextElement(DN.NextSibling,SNodeDependency);
    end;
end;


procedure TFPXMLRepositoryHandler.XMLToPackage(E: TDomElement; P: TFPPackage);
begin
  CheckNodeType(E,sNodePackage);
  DoXMLToPackage(E,P);
end;


Function TFPXMLRepositoryHandler.DoXMLToOSes(N : TDomElement) : TOSes;
Var
  E : TDomElement;
  J : Integer;
begin
  Result:=[];
  E:=FindNextElement(N.FirstChild,SNodeOS);
  While (E<>Nil) do
    begin
    J:=GetEnumValue(TypeInfo(TOS),E[SAttrName]);
    If (J<>-1) then
      Include(Result,TOS(J));
    E:=FindNextElement(E.NextSibling,SNodeOS);
    end;
end;


Function TFPXMLRepositoryHandler.DoXMLToCPUs(N : TDomElement) : TCPUS;
Var
  E : TDomElement;
  J : Integer;
begin
  Result:=[];
  E:=FindNextElement(N.FirstChild,SNodeCPU);
  While (E<>Nil) do
    begin
      J:=GetEnumValue(TypeInfo(TCPU),E[SAttrName]);
      If (J<>-1) then
        Include(Result,TCPU(J));
      E:=FindNextElement(E.NextSibling,SNodeCPU);
    end;
end;


procedure TFPXMLRepositoryHandler.DoXMLToPackage(E: TDomElement; P: TFPPackage);
Var
  N : TDomElement;
begin
  P.Name:=E[sAttrName];
  N:=GetNextElement(E.FirstChild);
  While (N<>Nil) do
    begin
      if (N.NodeName=sNodeAuthor) then
        P.Author:=NodeText(N)
      else if (N.NodeName=sNodeHomepageURL) then
        P.HomepageURL:=NodeText(N)
      else if (N.NodeName=sNodeDownloadURL) then
        P.DownloadURL:=NodeText(N)
      else if (N.NodeName=sNodeFileName) then
        P.FileName:=NodeText(N)
      else if (N.NodeName=sNodeEmail) then
        P.Email:=NodeText(N)
      else if (N.NodeName=sNodeDescription) then
        P.Description:=NodeText(N)
      else if (N.NodeName=sNodeLicense) then
        P.License:=NodeText(N)
      else if (N.NodeName=sNodeVersion) then
        DoXMlToVersion(N,P.Version)
      else if (N.NodeName=sNodeOSes) then
        P.OSes:=DoXMLToOSes(N)
      else if (N.NodeName=sNodeCPUS) then
        P.CPUs:=DoXMLToCPUs(N)
      else if (N.NodeName=sNodeDependencies) then
        DoXMlToDependencies(N,P.Dependencies)
      else if Not IgnoreUnknownNodes then
        Raise EXMLPackage.CreateFmt(SErrUnknownNode,[N.NodeName,sNodePackage,P.Name]);
      N:=GetNextElement(N.NextSibling);
    end;
end;


procedure TFPXMLRepositoryHandler.XMLToPackages(E: TDomElement; PS: TFPPackages);
begin
  CheckNodeType(E,SNodePackages);
  DoXMLToPackages(E,PS);
end;


procedure TFPXMLRepositoryHandler.DoXMLToPackages(E: TDomElement; PS: TFPPackages);
Var
  PN : TDomElement;
  P : TFPPackage;
begin
  PN:=FindNextElement(E.FirstChild,SNodePackage);
  While (PN<>Nil) do
    begin
      P:=PS.AddPackage('');
      try
        DoXMLToPackage(PN,P);
      except
        P.Free;
        Raise;
      end;
      PN:=FindNextElement(PN.NextSibling,SNodePackage);
    end;
end;


procedure TFPXMLRepositoryHandler.XMLToRepository(E: TDomElement; R: TFPRepository);
begin
  CheckNodeType(E,SNodeRepository);
  DoXMLToRepository(E,R);
end;


procedure TFPXMLRepositoryHandler.DoXMLToRepository(E: TDomElement; R: TFPRepository);
Var
  PSN,PN : TDomElement;
  P : TFPPackage;
begin
  PSN:=FindNextElement(E.FirstChild,SNodePackages);
  If (PSN<>Nil) then
    begin
      PN:=FindNextElement(PSN.FirstChild,SNodePackage);
      While (PN<>Nil) do
        begin
          P:=R.AddPackage('');
          try
            DoXMLToPackage(PN,P);
          except
            P.Free;
            Raise;
          end;
          PN:=FindNextElement(PN.NextSibling,SNodePackage);
        end;
    end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(D: TFPDependency; Stream: TStream);
Var
  XML : TXMLDocument;
begin
  XML:=TXMLDocument.Create;
  try
    Xmlread.ReadXMLFile(XML,Stream);
    XmlToDependency(XML.DocumentElement,D);
  finally
    XML.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(DS: TFPDependencies; Stream: TStream);
Var
  XML : TXMLDocument;
begin
  XML:=TXMLDocument.Create;
  try
    xmlread.ReadXMLFile(XML,Stream);
    XmlToDependencies(XML.DocumentElement,DS);
  finally
    XML.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(P: TFPPackage; Stream: TStream);
Var
  XML : TXMLDocument;
begin
  XML:=TXMLDocument.Create;
  try
    xmlread.ReadXMLFile(XML,Stream);
    XmlToPackage(XML.DocumentElement,P);
  finally
    XML.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(PS: TFPPackages; Stream: TStream);
Var
  XML : TXMLDocument;
begin
  XML:=TXMLDocument.Create;
  try
    xmlread.ReadXMLFile(XML,Stream);
    XmlToPackages(XML.DocumentElement,PS);
  finally
    XML.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(R: TFPRepository; Stream: TStream);
Var
  XML : TXMLDocument;
begin
  XML:=TXMLDocument.Create;
  try
    xmlread.ReadXMLFile(XML,Stream);
    XmlToRepository(XML.DocumentElement,R);
  finally
    XML.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(D: TFPDependency; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmOpenRead);
  try
    LoadFromXMl(D,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(DS: TFPDependencies; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmOpenRead);
  try
    LoadFromXMl(DS,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(P: TFPPackage; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmOpenRead);
  try
    LoadFromXMl(P,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(PS: TFPPackages; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmOpenRead);
  try
    LoadFromXMl(PS,F);
  finally
    F.Free;
  end;
end;


procedure TFPXMLRepositoryHandler.LoadFromXml(R: TFPRepository; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmOpenRead);
  try
    LoadFromXMl(R,F);
  finally
    F.Free;
  end;
end;


{****************************************************************************
                           TFPXMLMirrorHandler
****************************************************************************}

procedure TFPXMLMirrorHandler.DoXMLToMirror(E: TDomElement; P: TFPMirror);
Var
  N : TDomElement;
begin
  P.Name:=E[sAttrName];
  N:=GetNextElement(E.FirstChild);
  While (N<>Nil) do
    begin
      if (N.NodeName=sNodeURL) then
        P.URL:=NodeText(N)
      else if (N.NodeName=sNodeContact) then
        P.Contact:=NodeText(N)
      else if (N.NodeName=sNodeWeight) then
        P.Weight:=StrToInt(NodeText(N))
      else if Not IgnoreUnknownNodes then
        Raise EXMLPackage.CreateFmt(SErrUnknownNode,[N.NodeName,sNodeMirror,P.Name]);
      N:=GetNextElement(N.NextSibling);
    end;
end;


procedure TFPXMLMirrorHandler.DoXMLToMirrors(E: TDomElement; PS: TFPMirrors);
Var
  PN : TDomElement;
  P : TFPMirror;
begin
  PN:=FindNextElement(E.FirstChild,SNodeMirror);
  While (PN<>Nil) do
    begin
    P:=PS.AddMirror('');
    try
      DoXMLToMirror(PN,P);
    except
      P.Free;
      Raise;
    end;
    PN:=FindNextElement(PN.NextSibling,SNodeMirror);
    end;
end;


procedure TFPXMLMirrorHandler.XMLToMirrors(E: TDomElement; PS: TFPMirrors);
begin
  CheckNodeType(E,SNodeMirrors);
  DoXMLToMirrors(E,PS);
end;


procedure TFPXMLMirrorHandler.LoadFromXml(PS: TFPMirrors; Stream: TStream);
Var
  XML : TXMLDocument;
begin
  XML:=TXMLDocument.Create;
  try
    xmlread.ReadXMLFile(XML,Stream);
    XmlToMirrors(XML.DocumentElement,PS);
  finally
    XML.Free;
  end;
end;


procedure TFPXMLMirrorHandler.LoadFromXml(PS: TFPMirrors; const FileName: String);
Var
  F : TFileStream;
begin
  F:=TFileStream.Create(FileName,fmOpenRead);
  try
    LoadFromXMl(PS,F);
  finally
    F.Free;
  end;
end;


end.

