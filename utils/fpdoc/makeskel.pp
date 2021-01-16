{
    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2000 - 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org
    2005-2012 by
      various FPC contributors

    * Skeleton XML description file generator.
    This generator scans Pascal source code for identifiers and emits XML files
    suitable for further processing with the fpdoc documentation system:
    users can edit the XML file and add (help) description.

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
program MakeSkel;

{$mode objfpc}
{$h+}

uses
  fpdocstrs, SysUtils, Classes, Gettext, dGlobals, PasTree, PParser,PScanner;

resourcestring
  STitle = 'MakeSkel - FPDoc skeleton XML description file generator';
  SVersion = 'Version %s [%s]';
  SCmdLineHelp = 'See documentation for usage.';
  SCmdLineInvalidOption = 'Ignoring unknown option "%s"';
  SNoPackageNameProvided = 'Please specify a package name with --package=<name>';
  SOutputMustNotBeDescr = 'Output file must be different from description filenames.';
  SCreatingNewNode = 'Creating documentation for new node : %s';
  SNodeNotReferenced = 'Documentation node "%s" no longer used';
  SDone = 'Done.';

type
  TCmdLineAction = (actionHelp, actionConvert);

  TNodePair = Class(TObject)
  Private
    FEl : TPasElement;
    FNode : TDocNode;
  Public  
    Constructor Create(AnElement : TPasElement; ADocNode : TDocNode);
    Property Element : TPasElement Read FEl;
    Property DocNode : TDocNode Read FNode;
  end;

  { TSkelEngine }

  TSkelEngine = class(TFPDocEngine)
  Private
    FEmittedList, 
    FNodeList,
    FModules : TStringList;
    Procedure  DoWriteUnReferencedNodes(N : TDocNode; NodePath : String);
    function EffectiveVisibility(El: TPasElement): TPasMemberVisibility;
  public
    Destructor Destroy; override;
    Function MustWriteElement(El : TPasElement; Full : Boolean) : Boolean;
    Function WriteElement(Var F : Text; El : TPasElement; ADocNode : TDocNode) : Boolean;
    function FindModule(const AName: String): TPasModule; override;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility :TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement; override;
    procedure WriteUnReferencedNodes;
    Procedure WriteNodes(Var F : Text; AModule : TPasModule; List : TStrings);
    Procedure DocumentFile(Var F : Text; Const AFileName,ATarget,ACPU : String);
    Property NodeList : TStringList Read FNodeList;
    Property EmittedList : TStringList Read FEmittedList;
  end;

const
  CmdLineAction: TCmdLineAction = actionConvert;
  OSTarget: String = {$I %FPCTARGETOS%};
  CPUTarget: String = {$I %FPCTARGETCPU%};
  FPCVersion: String = {$I %FPCVERSION%};
  FPCDate: String = {$I %FPCDATE%};

var
  WriteDeclaration,
  UpdateMode,
  SortNodes,
  DisableOverride,
  DisableErrors,
  DisableSeealso,
  DisableArguments,
  DisableProtected,
  DisablePrivate,
  DisableFunctionResults: Boolean;
  EmitClassSeparator: Boolean;
  
  
Constructor TNodePair.Create(AnElement : TPasElement; ADocNode : TDocNode);

begin
  Fel:=Anelement;
  FNode:=ADocNode;
end;

function TSkelEngine.FindModule(const AName: String): TPasModule; 

Var
  I : Integer;

begin
  Result:=Inherited FindModule(AName);
  If (Result=Nil) then
    begin // Create dummy list and search in that.
    If (FModules=Nil) then
      begin
      FModules:=TStringList.Create;
      FModules.Sorted:=True;
      end;
    I:=FModules.IndexOf(AName);
    IF (I=-1) then
      begin
      Result:=TPasModule.Create(AName,Nil);
      FModules.AddObject(AName,Result);
      end
    else
      Result:=FModules.Objects[i] as TPasModule;  
    end;  
end;

Destructor TSkelEngine.Destroy; 

Var
  I : Integer;

begin
  If Assigned(FModules) then 
    begin
   { For I:=0 to FModules.Count-1 do
      FModules.Objects[i].Release;}
    FreeAndNil(FModules);    
    end;
end;

Function TSkelEngine.EffectiveVisibility (El : TPasElement) :  TPasMemberVisibility;

Var
  V : TPasMemberVisibility;

begin
  Result:=EL.Visibility;
  El:=el.Parent;
  While Assigned(El) do
    begin
    V:=EL.Visibility;
    if V=visStrictPrivate then
      V:=visPrivate
    else if V=visStrictProtected then
      V:=visProtected;
    if (V<>visDefault) and ((V<Result) or (Result=visDefault)) then
      Result:=V;
    EL:=el.Parent;
    end;
end;

Function TSkelEngine.MustWriteElement(El : TPasElement; Full : Boolean) : Boolean;

Var
  VisibilityOK : Boolean;
  V : TPasMemberVisibility;


begin
  V:=EffectiveVisibility(El);
  Case V of
    visPrivate,visStrictPrivate:
      VisibilityOK:= not DisablePrivate;
    visProtected,visStrictProtected:
      VisibilityOK:= not DisableProtected;
  else
    VisibilityOK:=True;
  end;
  Result:= Assigned(el.Parent)
           and (Length(El.Name) > 0)
           and VisibilityOK
           and (Not (El is TPasExpr))
           and (not DisableArguments or (El.ClassType <> TPasArgument))
           and (not DisableFunctionResults or (El.ClassType <> TPasResultElement));
  If Result and Full then
    begin
    Result:=(Not Assigned(FEmittedList) or (FEmittedList.IndexOf(El.FullName)=-1));
    If DisableOverride and (El is TPasProcedure) then
      Result:=Not TPasProcedure(El).IsOverride;
    end;  
end;


function TSkelEngine.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility : TPasMemberVisibility;
  const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;

Var
  DN : TDocNode;

begin
  Result := AClass.Create(AName, AParent);
  Result.Visibility:=AVisibility;
  // Let function/procedure arguments and function results
  // inherit visibility from their parents if visDefault visibility is
  // specified.
  // This allows easier text searches on visibility in the resulting XML
  if (AVisibility=visDefault) and
    ((Result is TPasArgument) or (Result is TPasResultElement)) then
    Result.Visibility:=AParent.Visibility;

  if AClass.InheritsFrom(TPasModule) then
    CurModule := TPasModule(Result);
  // Track this element
  If UpdateMode then
    begin
    DN:=FindDocNode(Result);    
    If Assigned(DN) then
      DN.IncRefCount;
    end
  else
    DN:=Nil;  
  // See if we need to write documentation for it
  If MustWriteElement(Result,False) then
    FNodeList.AddObject(Result.PathName,TNodePair.Create(Result,DN));
end;

Function TSkelEngine.WriteElement(Var F : Text;El : TPasElement; ADocNode : TDocNode) : Boolean;

  Function WriteOnlyShort(APasElement : TPasElement) : Boolean;

  begin
    Result:=(APasElement.ClassType=TPasArgument) or
            (APasElement.ClassType=TPasResultElement) or
            (APasElement.ClassType=TPasEnumValue) or
            (aPaselement.ClassType=TPasUsesUnit) or
            ((APasElement.CLassType=TPasVariable) and (APasElement.Parent is TPasRecordType));
  end;

  Function IsTypeVarConst(APasElement : TPasElement) : Boolean;

  begin
    With APasElement do
      Result:=(InheritsFrom(TPasType) and not InheritsFrom(TPasClassType)) or
              (InheritsFrom(TPasResString)) or
              (InheritsFrom(TPasVariable));
  end;
  
  Function NeedDeclaration(El : TPasElement) : boolean;
  
  begin
    Result:=IsTypeVarConst(El) 
            or WriteOnlyShort(El) 
            or EL.InheritsFrom(TPasProcedure) 
  end;
    
begin
  // Check again, this time with full declaration.
  Result:=MustWriteElement(El,True);
  If Result and UpdateMode then
     Result:=(ADocNode=Nil);
  If Not Result Then
    Exit;
  If UpdateMode then
    Writeln(stderr,Format(ScreatingNewNode,[el.PathName]));
  FEmittedList.Add(El.FullName); // So we don't emit again.
  WriteLn(f);
  if EmitClassSeparator and (El.ClassType = TPasClassType) then
    begin
    WriteLn(f, '<!--');
    WriteLn(f, '  ********************************************************************');
    WriteLn(f, '    ', El.PathName);
    WriteLn(f, '  ********************************************************************');
    WriteLn(f, '-->');
    WriteLn(f);
    end;
  If Not (WriteDeclaration and NeedDeclaration(El)) then  
    Writeln(F,'<!-- ', El.ElementTypeName,' Visibility: ',VisibilityNames[El.Visibility], ' -->')
  else  
    begin
    Writeln(F,'<!-- ',El.ElementTypeName,' Visibility: ',VisibilityNames[El.Visibility]);
    Writeln(F,'     Declaration: ',El.GetDeclaration(True),' -->');
    end;
  WriteLn(f,'<element name="', El.FullName, '">');
  WriteLn(f, '<short></short>');
  if Not WriteOnlyShort(El) then
    begin
    WriteLn(f, '<descr>');
    WriteLn(f, '</descr>');
    if not (DisableErrors or IsTypeVarConst(El)) then
      begin
      WriteLn(f, '<errors>');
      WriteLn(f, '</errors>');
      end;
    if not DisableSeealso then
      begin
      WriteLn(f, '<seealso>');
      WriteLn(f, '</seealso>');
      end;
    end;
  WriteLn(f, '</element>');
end;

Procedure  TSkelEngine.DoWriteUnReferencedNodes(N : TDocNode; NodePath : String);

begin
  If (N<>Nil) then
    begin
    If (NodePath<>'') then
      NodePath:=NodePath+'.';
    DoWriteUnReferencedNodes(N.FirstChild,NodePath+N.Name);
    While (N<>Nil) do
      begin
      if (N.RefCount=0) and (N.Node<>Nil) and (Not N.TopicNode) then
        Writeln(stderr,Format(SNodeNotReferenced,[NodePath+N.Name]));
      N:=N.NextSibling;
      end;
    end;
end;

procedure TSkelEngine.WriteUnReferencedNodes;

begin
  DoWriteUnReferencedNodes(RootDocNode,'');
end;

Procedure TSkelEngine.WriteNodes(Var F : Text; AModule : TPasModule; List : TStrings);

Var
  P : TNodePair;
  I : integer;
  
begin
  WriteLn(f);
  WriteLn(f, '<!--');
  WriteLn(f, '  ====================================================================');
  WriteLn(f, '    ', Amodule.Name);
  WriteLn(f, '  ====================================================================');
  WriteLn(f, '-->');
  WriteLn(f);
  WriteLn(f, '<module name="', AModule.Name, '">');
  if not UpdateMode then
    begin
    WriteLn(f, '<short></short>');
    WriteLn(f, '<descr>');
    WriteLn(f, '</descr>');
    end;
  Try 
    For I:=0 to List.Count-1 do
      begin
      P:=List.Objects[i] as TNodePair;
      If (P.Element<>AModule) then
        WriteElement(F,P.Element,P.DocNode);
      end;  
  Finally
    WriteLn(f, '');
    WriteLn(f, '</module> <!-- ', AModule.Name, ' -->');
    WriteLn(f, '');
  end;   
end;

Procedure TSkelEngine.DocumentFile(Var F : Text; Const AFileName,ATarget,ACPU : String);

  Procedure ResolveOperators;

  Var
    E : TPasElement;
    P : TNodePair;
    N : TDocNode;
    I : integer;

  begin
    For I:=0 to FNodeList.Count-1 do
      begin
      P:=TNodePair(FNodeList.Objects[i]);
      if P.Element.InheritsFrom(TPasOperator) then
        begin
        N:=FindDocNode(P.Element);
        If Assigned(N) then
          N.IncRefCount;
        P.FNode:=N;
        end;
      end;
  end;

Var
  Module : TPasModule;
  I : Integer;
  N : TDocNode;

begin
// wrong because afilename is a cmdline with other options. Straight testing filename is therefore wrong.
//  if not(FileExists(AFileName)) then
//    raise Exception.CreateFmt('Cannot find source file %s to document.',[AFileName]);
  FNodeList:=TStringList.Create;
  Try
    FEmittedList:=TStringList.Create;
    FEmittedList.Sorted:=True;
    try
      Module:=ParseSource (Self,AFileName+' -dFPC',ATarget,ACPU,[poUseStreams,poSkipDefaultDefs]);
      If UpdateMode then
        begin
        N:=FindDocNode(Module);
        If Assigned(N) then
           N.IncRefCount;
        ResolveOperators;
        end;
      If SortNodes then  
        FNodelist.Sorted:=True;   
      WriteNodes(F,Module,FNodeList);  
      If UpdateMode then
        WriteUnReferencedNodes;
    Finally
      FEmittedList.Free;
    end;  
  Finally  
    For I:=0 to FNodeList.Count-1 do
      FNodeList.Objects[i].Free;
    FNodeList.Free;  
  end;  
end;

{ ---------------------------------------------------------------------
  Main program. Document all units.    
  ---------------------------------------------------------------------}
  
Function DocumentPackage(Const APackageName,AOutputName : String; InputFiles,DescrFiles : TStrings) : String;

Var
  F : Text;
  I,J : Integer;
  Engine: TSkelEngine;

begin
  Result:='';
  Assign(f, AOutputName);
  Rewrite(f);
  Try
    WriteLn(f, '<?xml version="1.0" encoding="ISO-8859-1"?>');
    WriteLn(f, '<fpdoc-descriptions>');
    WriteLn(f, '<package name="', APackageName, '">');
    Try
      I:=0;
      While (Result='') And (I<InputFiles.Count) do
        begin
        Engine := TSkelEngine.Create;
        Try
          Engine.SetPackageName(APackageName);
          if UpdateMode then
            For J:=0 to DescrFiles.Count-1 do
              Engine.AddDocFile(DescrFiles[J]);
          Try    
            Engine.DocumentFile(F,InputFiles[I],OSTarget,CPUTarget);
          except
            on E:Exception do
            begin
              WriteLn('Error while documenting: '+E.message);
              Result:='Error while documenting: '+E.message;
            end;
          end;
        Finally
          Engine.Free;
        end;
        Inc(I);
        end;
    Finally
      WriteLn(f, '</package>');
      WriteLn(f, '</fpdoc-descriptions>');
    end;
  finally
    Close(f);
  end;
end;

{ ---------------------------------------------------------------------
    Option management
  ---------------------------------------------------------------------}
  

var  
  InputFiles, 
  DescrFiles : TStringList;
  DocLang : String;
  PackageName, 
  OutputName: String;

procedure InitOptions;
begin
  InputFiles := TStringList.Create;
  DescrFiles := TStringList.Create;
end;

procedure FreeOptions;

begin
  DescrFiles.Free;
  InputFiles.Free;
end;

procedure ParseOption(const s: String);

  procedure AddToFileList(List: TStringList; const FileName: String);
  var
    f: Text;
    s: String;
  begin
    if Copy(FileName, 1, 1) = '@' then
    begin
      Assign(f, Copy(FileName, 2, Length(FileName)));
      Reset(f);
      while not EOF(f) do
      begin
        ReadLn(f, s);
        List.Add(s);
      end;
      Close(f);
    end else
      List.Add(FileName);
  end;

var
  i: Integer;
  Cmd, Arg: String;
begin
  if (s = '-h') or (s = '--help') then
    CmdLineAction := actionHelp
  else if s = '--update' then
    UpdateMode := True
  else if s = '--disable-arguments' then
    DisableArguments := True
  else if s = '--disable-errors' then
    DisableErrors := True
  else if s = '--disable-function-results' then
    DisableFunctionResults := True
  else if s = '--disable-seealso' then
    DisableSeealso := True
  else if s = '--disable-private' then
    DisablePrivate := True
  else if s = '--disable-override' then
    DisableOverride := True
  else if s = '--disable-protected' then
    begin
    DisableProtected := True;
    DisablePrivate :=True;
    end
  else if (s = '--emitclassseparator') or (s='--emit-class-separator') then
    EmitClassSeparator := True
  else if (s = '--emit-declaration') then
    WriteDeclaration := True
  else if (s = '--sort-nodes') then
    SortNodes := True
  else
  begin
    i := Pos('=', s);
    if i > 0 then
    begin
      Cmd := Copy(s, 1, i - 1);
      Arg := Copy(s, i + 1, Length(s));
    end else
    begin
      Cmd := s;
      SetLength(Arg, 0);
    end;
    if (Cmd = '-i') or (Cmd = '--input') then
      AddToFileList(InputFiles, Arg)
    else if (Cmd = '-l') or (Cmd = '--lang') then
      DocLang := Arg
    else if (Cmd = '-o') or (Cmd = '--output') then
      OutputName := Arg
    else if Cmd = '--package' then
      PackageName := Arg
    else if Cmd = '--descr' then
      begin
      if FileExists(Arg) then
        DescrFiles.Add(Arg);
      end
    else
      WriteLn(StdErr, Format(SCmdLineInvalidOption, [s]));
  end;
end;

Function ParseCommandLine : Integer;

Const
{$IFDEF Unix}
  MoFileTemplate = '/usr/local/share/locale/%s/LC_MESSAGES/makeskel.mo';
{$ELSE}
  MoFileTemplate ='intl/makeskel.%s.mo';
{$ENDIF}

var
  MOFilename: string;
  i: Integer;
  
begin
  Result:=0;
  DocLang:='';
  for i := 1 to ParamCount do
    ParseOption(ParamStr(i));
  If (DocLang<>'') then
    begin
    MOFilename:=Format(MOFileTemplate,[DocLang]);
    if FileExists(MOFilename) then
      gettext.TranslateResourceStrings(MoFileName)
    else
      writeln('NOTE: unable to find translation file ',MOFilename);
    // Translate internal documentation strings
    TranslateDocStrings(DocLang);
    end;
  // Action is to create the XML skeleton
  if (Length(PackageName) = 0) and (CmdLineAction<>ActionHelp) then
    begin
    WriteLn(SNoPackageNameProvided);
    Result:=2;
    end;
  if DescrFiles.IndexOf(OutputName)<>-1 then
    begin
    Writeln(SOutputMustNotBeDescr);
    Result:=3;
    end;
end;

{ ---------------------------------------------------------------------
  Usage  
  ---------------------------------------------------------------------}
  
Procedure Usage;

begin
  Writeln('Usage : ',ExtractFileName(Paramstr(0)),' [options]');
  Writeln('Where [options] is one or more of :');
  Writeln(' --descr=filename    Filename for update.');
  Writeln(' --disable-arguments Do not create nodes for function arguments.');
  Writeln(' --disable-errors    Do not create errors node.');
  Writeln(' --disable-function-results');
  Writeln('                     Do not create nodes for function arguments.');
  Writeln(' --disable-override  Do not create nodes for override methods.');
  Writeln(' --disable-private   Do not create nodes for class private fields.');
  Writeln(' --disable-protected Do not create nodes for class protected fields.');
  Writeln(' --disable-seealso   Do not create seealso node.');
  Writeln(' --emit-class-separator');
  Writeln('                     Emit descriptive comment between classes.');
  Writeln(' --emit-declaration  Emit declaration for elements.');
  Writeln(' --help              Emit help.');
  Writeln(' --input=cmdline     Input file to create skeleton for.');
  Writeln('                     Use options are as for compiler.');
  Writeln(' --lang=language     Use selected language.');
  Writeln(' --output=filename   Send output to file.');
  Writeln(' --package=name      Specify package name (mandatory).');
  Writeln(' --sort-nodes        Sort element nodes (not modules)');
  Writeln(' --update            Update mode. Output only missing nodes.');
end;

{ ---------------------------------------------------------------------
  Main Program  
  ---------------------------------------------------------------------}
  
Procedure Run;
  
var
  E: Integer;

begin
  WriteLn(STitle);
  WriteLn(Format(SVersion, [FPCVersion, FPCDate]));
  WriteLn(SCopyright1);
  WriteLn(SCopyright2);
  InitOptions;
  Try
    E:=ParseCommandLine;
    If E<>0 then
      Halt(E);
    WriteLn;
    if CmdLineAction = actionHelp then
      Usage
    else
      begin
      DocumentPackage(PackageName,OutputName,InputFiles,DescrFiles);
      WriteLn(SDone);
      end;
  Finally  
    FreeOptions;
  end;  
end;

Begin
  Run;  
end.

