{

    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2000 - 2002 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    * Global declarations
    * Link list management
    * Document node tree
    * Main engine

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$MODE objfpc}
{$H+}
unit dGlobals;


interface

uses Classes, DOM, PasTree, PParser, uriparser, SysUtils;

Const
  CacheSize = 20;
  ContentBufSize = 4096 * 8;

Var
  LEOL : Integer;
  modir : string;


Const
  SVisibility: array[TPasMemberVisibility] of string =
       ('Default', 'Private', 'Protected', 'Public',
       'Published', 'Automated','Strict Private','Strict Protected',
       'Required', 'Optional' // ObjCClass
       );

type
  TBufType = Array[1..ContentBufSize-1] of byte;

  // Assumes a list of TObject instances and frees them on destruction

  TObjectList = class(TFPList)
  public
    destructor Destroy; override;
  end;

  TPasExternalClassType = Class(TPasClassType);
  TPasExternalModule = Class(TPasModule);

  { Link entry tree
    TFPDocEngine stores the root of the entry tree in its property
    "RootLinkNode". The root has one child node for each package, for which
    documentation links are available. The children of a package node
    are module nodes; and the children of a module node are the top-level
    declarations of this module; the next level in the tree stores e.g. record
    members, and so on...
  }

  TLinkNode = class
  private
    FFirstChild, FNextSibling: TLinkNode;
    FName: String;
    FLink: String;
  public
    constructor Create(const AName, ALink: String);
    destructor Destroy; override;
    function FindChild(const APathName: String): TLinkNode;
    function CreateChildren(const APathName, ALinkTo: String): TLinkNode;
    // Properties for tree structure
    property FirstChild: TLinkNode read FFirstChild;
    property NextSibling: TLinkNode read FNextSibling;
    // Link properties
    property Name: String read FName;
    property Link: String read FLink;
  end;


  { Documentation entry tree
    TFPDocEngine stores the root of the entry tree in its property
    "RootDocNode". The root has one child node for each package, for which
    documentation is being provided by the user. The children of a package node
    are module nodes; and the children of a module node are the top-level
    declarations of this module; the next level in the tree stores e.g. record
    members, and so on...
  }

  { TDocNode }

  TDocNode = class
  private
    FFirstChild, FNextSibling: TDocNode;
    FName: String;
    FNode: TDOMElement;
    FIsSkipped: Boolean;
    FShortDescr: TDOMElement;
    FDescr: TDOMElement;
    FErrorsDoc: TDOMElement;
    FSeeAlso: TDOMElement;
    FFirstExample: TDOMElement;
    FNotes : TDomElement;
    FLink: String;
    FTopicNode : Boolean;
    FRefCount : Integer;
    FVersion: TDomElement;
  public
    constructor Create(const AName: String; ANode: TDOMElement);
    destructor Destroy; override;
    Function IncRefcount : Integer;
    function FindChild(const APathName: String): TDocNode;
    function CreateChildren(const APathName: String): TDocNode;
    // Properties for tree structure
    property FirstChild: TDocNode read FFirstChild;
    property NextSibling: TDocNode read FNextSibling;
    // Basic properties
    property Name: String read FName;
    property Node: TDOMElement read FNode;
    // Data fetched from the XML document
    property IsSkipped: Boolean read FIsSkipped;
    property ShortDescr: TDOMElement read FShortDescr;
    property Descr: TDOMElement read FDescr;
    property ErrorsDoc: TDOMElement read FErrorsDoc;
    Property Version : TDomElement Read FVersion;
    property SeeAlso: TDOMElement read FSeeAlso;
    property FirstExample: TDOMElement read FFirstExample;
    property Notes : TDOMElement read FNotes;
    property Link: String read FLink;
    Property TopicNode : Boolean Read FTopicNode;
    Property RefCount : Integer Read FRefCount;
  end;
  


  // The main FPDoc engine
  TFPDocLogLevel = (dleWarnNoNode, dleWarnUsedFile, dleDocumentationEmpty, dleXCT);
  TFPDocLogLevels = set of TFPDocLogLevel;
  TOnParseUnitEvent = Procedure (Sender : TObject; Const AUnitName : String; Out AInputFile,OSTarget,CPUTarget : String) of  Object;

  { TFPDocEngine }
  TFPDocEngine = class(TPasTreeContainer)
  private
    FDocLogLevels: TFPDocLogLevels;
    FOnParseUnit: TOnParseUnitEvent;
    function ResolveLinkInPackages(AModule: TPasModule; const ALinkDest: String; Strict: Boolean=False): String;
    function ResolveLinkInUsedUnits(AModule: TPasModule; const ALinkDest: String; Strict: Boolean=False): String;
  protected
    FAlwaysVisible : TStringList;
    DescrDocs: TObjectList;             // List of XML documents
    DescrDocNames: TStringList;         // Names of the XML documents
    FRootLinkNode: TLinkNode;           // Global tree of TlinkNode from the imported .xct files
    FRootDocNode: TDocNode;             // Global tree of TDocNode from the .xml documentation files
    FPackages: TFPList;                 // Global list of TPasPackage objects and full tree of sources
    CurModule: TPasModule;
    CurPackageDocNode: TDocNode;
    function ParseUsedUnit(AName, AInputLine,AOSTarget,ACPUTarget: String): TPasModule; virtual;
    Function LogEvent(E : TFPDocLogLevel) : Boolean;
    Procedure DoLog(Const Msg : String);overload;
    Procedure DoLog(Const Fmt : String; Args : Array of const);overload;
  public
    Output: String;
    HasContentFile: Boolean;
    HidePrivate: Boolean;       // Hide private class members in output?
    HideProtected: Boolean;     // Hide protected class members in output?
    FalbackSeeAlsoLinks: Boolean; // Simplify SeeAlso Links
    constructor Create;
    destructor Destroy; override;
    procedure SetPackageName(const APackageName: String);
    // The process importing of objects from external .xct file
    procedure ReadContentFile(const AFilename, ALinkPrefix: String);
    // Creation of an own .xct output file
    procedure WriteContentFile(const AFilename: String);

    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
      override;
    function FindElement(const AName: String ; AModule: TPasModule): TPasElement; overload;
    function FindElement(const AName: String): TPasElement; override;
    function FindModule(const AName: String): TPasModule; override;
    Function HintsToStr(Hints : TPasMemberHints) : String;

    // Link tree support
    procedure AddLink(const APathName, ALinkTo: String);
    function FindAbsoluteLink(const AName: String): String;
    // resolve link inside actual AModule and AModule.Parent = APackage
    function ResolveLink(AModule: TPasModule; const ALinkDest: String; Strict : Boolean = False): String;
    function FindLinkedNode(ANode: TDocNode): TDocNode;
    Function ShowElement(El : TPasElement) : Boolean; inline;

    // Call this before documenting.
    Procedure StartDocumenting; virtual;

    // Documentation file support
    procedure AddDocFile(const AFilename: String;DontTrim:boolean=false);

    // Documentation retrieval
    function FindDocNode(AElement: TPasElement): TDocNode;
    function FindDocNode(ARefModule: TPasModule; const AName: String): TDocNode;
    function FindShortDescr(AElement: TPasElement): TDOMElement;
    function FindShortDescr(ARefModule: TPasModule; const AName: String): TDOMElement;
    function GetExampleFilename(const ExElement: TDOMElement): String;

    property RootLinkNode: TLinkNode read FRootLinkNode;
    property RootDocNode: TDocNode read FRootDocNode;
    Property DocLogLevels : TFPDocLogLevels Read FDocLogLevels Write FDocLogLevels;
    Property OnParseUnit : TOnParseUnitEvent Read FOnParseUnit Write FOnParseUnit;
  end;


procedure TranslateDocStrings(const Lang: String);
{$IFDEF EXCEPTION_STACK}
function DumpExceptionCallStack(E: Exception):String;
{$ENDIF}

Function IsLinkNode(Node : TDomNode) : Boolean;
Function IsExampleNode(Example : TDomNode) : Boolean;

// returns true is link is an absolute URI
Function IsLinkAbsolute(ALink: String): boolean;


implementation

uses Gettext, XMLRead, fpdocstrs;

const
  AbsoluteLinkPrefixes : array[0..2] of string = ('/', 'http://', 'ms-its:');


{ TObjectList }

destructor TObjectList.Destroy;
var
  i: Integer;
begin
  for i := 0 to Count - 1 do
    TObject(Items[i]).Free;
  inherited Destroy;
end;


{ TLinkNode }

constructor TLinkNode.Create(const AName, ALink: String);
begin
  inherited Create;
  FName := AName;
  FLink := ALink;
end;

destructor TLinkNode.Destroy;
begin
  if Assigned(FirstChild) then
    FirstChild.Free;
  if Assigned(NextSibling) then
    NextSibling.Free;
  inherited Destroy;
end;

function TLinkNode.FindChild(const APathName: String): TLinkNode;
var
  DotPos: Integer;
  ChildName: String;
  Child: TLinkNode;
begin
  if Length(APathName) = 0 then
    Result := Self
  else
  begin
    DotPos := Pos('.', APathName);
    if DotPos = 0 then
      ChildName := APathName
    else
      ChildName := Copy(APathName, 1, DotPos - 1);
    Child := FirstChild;
    while Assigned(Child) do
    begin
      if CompareText(Child.Name, ChildName) = 0 then
      begin
        if DotPos = 0 then
          Result := Child
        else
          Result := Child.FindChild(
            Copy(APathName, DotPos + 1, Length(APathName)));
        exit;
      end;
      Child := Child.NextSibling;
    end;
    Result := nil;
  end;
end;

function TLinkNode.CreateChildren(const APathName, ALinkTo: String): TLinkNode;
var
  DotPos: Integer;
  ChildName: String;
  Child, LastChild: TLinkNode;
begin
  if Length(APathName) = 0 then
    Result := Self
  else
  begin
    DotPos := Pos('.', APathName);
    if DotPos = 0 then
      ChildName := APathName
    else
      ChildName := Copy(APathName, 1, DotPos - 1);
    Child := FirstChild;
    LastChild := nil;
    while Assigned(Child) do
    begin
      if CompareText(Child.Name, ChildName) = 0 then
      begin
        if DotPos = 0 then
          Result := Child
        else
          Result := Child.CreateChildren(
            Copy(APathName, DotPos + 1, Length(APathName)), ALinkTo);
        exit;
      end;
      LastChild := Child;
      Child := Child.NextSibling;
    end;
    Result := TLinkNode.Create(ChildName, ALinkTo);
    if Assigned(LastChild) then
      LastChild.FNextSibling := Result
    else
      FFirstChild := Result;
  end;
end;


{ TDocNode }

constructor TDocNode.Create(const AName: String; ANode: TDOMElement);
begin
  inherited Create;
  FName := AName;
  FNode := ANode;
end;

destructor TDocNode.Destroy;
begin
  if Assigned(FirstChild) then
    FirstChild.Free;
  if Assigned(NextSibling) then
    NextSibling.Free;
  inherited Destroy;
end;

Function TDocNode.IncRefcount : Integer;

begin
  Inc(FRefCount);
  Result:=FRefCount;
end;

function TDocNode.FindChild(const APathName: String): TDocNode;
var
  DotPos: Integer;
  ChildName: String;
  Child: TDocNode;
begin
  if Length(APathName) = 0 then
    Result := Self
  else
  begin
    DotPos := Pos('.', APathName);
    if DotPos = 0 then
      ChildName := APathName
    else
      ChildName := Copy(APathName, 1, DotPos - 1);
    Child := FirstChild;
    while Assigned(Child) do
    begin
      if CompareText(Child.Name, ChildName) = 0 then
      begin
        if DotPos = 0 then
          Result := Child
        else
          Result := Child.FindChild(
            Copy(APathName, DotPos + 1, Length(APathName)));
        exit;
      end;
      Child := Child.NextSibling;
    end;
    Result := nil;
  end;
end;

function TDocNode.CreateChildren(const APathName: String): TDocNode;
var
  DotPos: Integer;
  ChildName: String;
  Child: TDocNode;
begin
  if Length(APathName) = 0 then
    Result := Self
  else
  begin
    DotPos := Pos('.', APathName);
    if DotPos = 0 then
      ChildName := APathName
    else
      ChildName := Copy(APathName, 1, DotPos - 1);
    Child := FirstChild;
    while Assigned(Child) do
    begin
      if CompareText(Child.Name, ChildName) = 0 then
      begin
        if DotPos = 0 then
          Result := Child
        else
          Result := Child.CreateChildren(
            Copy(APathName, DotPos + 1, Length(APathName)));
        exit;
      end;
      Child := Child.NextSibling;
    end;
    // No child found, let's create one
    Result := TDocNode.Create(ChildName, nil);
    if Assigned(FirstChild) then
    begin
      Result.FNextSibling := FirstChild;
      FFirstChild := Result;
    end else
      FFirstChild := Result;

    if DotPos > 0 then
      Result := Result.CreateChildren(
        Copy(APathName, DotPos + 1, Length(APathName)));
  end;
end;


{ TFPDocEngine }

function TFPDocEngine.LogEvent(E: TFPDocLogLevel): Boolean;
begin
  Result:=E in FDocLogLevels;
end;

procedure TFPDocEngine.DoLog(const Msg: String);
begin
  If Assigned(OnLog) then
    OnLog(Self,Msg);
end;

procedure TFPDocEngine.DoLog(const Fmt: String; Args: array of const);
begin
  DoLog(Format(Fmt,Args));
end;

constructor TFPDocEngine.Create;
begin
  inherited Create;
  DescrDocs := TObjectList.Create;
  FAlwaysVisible := TStringList.Create;
  FAlwaysVisible.CaseSensitive:=True;
  DescrDocNames := TStringList.Create;
  FRootLinkNode := TLinkNode.Create('', '');
  FRootDocNode := TDocNode.Create('', nil);
  HidePrivate := True;
  InterfaceOnly:=True;
  FPackages := TFPList.Create;
end;

destructor TFPDocEngine.Destroy;
var
  i: Integer;
begin
  if FPackages.Count > 0 then
  for i := 0 to FPackages.Count - 1 do
    TPasPackage(FPackages[i]).Release{$IFDEF CheckPasTreeRefCount}('TFPDocEngine.Destroy'){$ENDIF}
  else
    FreeAndNil(FPackages);
  FreeAndNil(FRootDocNode);
  FreeAndNil(FRootLinkNode);
  FreeAndNil(DescrDocNames);
  FreeAndNil(DescrDocs);
  FreeAndNil(FAlwaysVisible);
  FreeAndNil(FPackages);
  inherited Destroy;
end;

procedure TFPDocEngine.SetPackageName(const APackageName: String);
begin
  ASSERT(not Assigned(Package));
  FPackage := TPasPackage(inherited CreateElement(TPasPackage,
    '#' + APackageName, nil, '', 0));
  FPackages.Add(FPackage);
  CurPackageDocNode := RootDocNode.FindChild('#' + APackageName);
  If Assigned(CurPackageDocNode) then
    CurPackageDocNode.IncRefCount;
end;

procedure TFPDocEngine.ReadContentFile(const AFilename, ALinkPrefix: String);
var
  f: Text;
  inheritanceinfo : TStringlist; // contents list of TPasClass with inheritance info
                                 // like this #PackageName.ModuleName.ClassName
  tmpLinkPrefix : string;

  procedure ReadLinkTree;
  var
    s: String;
    PrevSpaces, ThisSpaces, i, StackIndex: Integer;
    CurParent, PrevSibling, NewNode: TLinkNode;
    ParentStack, SiblingStack: array[0..7] of TLinkNode;
  begin
    PrevSpaces := 0;
    CurParent := RootLinkNode;
    PrevSibling := CurParent.FirstChild;
    if assigned(PrevSibling) then
      while assigned(PrevSibling.NextSibling) do
        PrevSibling := PrevSibling.NextSibling;
    StackIndex := 0;
    while True do
    begin
      ReadLn(f, s);
      if Length(s) = 0 then
        break;
      ThisSpaces := 0;
      while s[ThisSpaces + 1] = ' ' do
        Inc(ThisSpaces);
      if ThisSpaces <> PrevSpaces then
      begin
        if ThisSpaces > PrevSpaces then
        begin
          { Dive down one level }
          ParentStack[StackIndex] := CurParent;
          SiblingStack[StackIndex] := PrevSibling;
          Inc(StackIndex);
          CurParent := PrevSibling;
          PrevSibling := nil;
        end else
          while PrevSpaces > ThisSpaces do
          begin
            Dec(StackIndex);
            CurParent := ParentStack[StackIndex];
            PrevSibling := SiblingStack[StackIndex];
            Dec(PrevSpaces);
          end;
        PrevSpaces := ThisSpaces;
      end;

      i := ThisSpaces + 1;
      while s[i] <> ' ' do
        Inc(i);
      if ALinkPrefix <> '' then
        tmpLinkPrefix := ExcludeTrailingPathDelimiter(ALinkPrefix)+'/';
      NewNode := TLinkNode.Create(Copy(s, ThisSpaces + 1, i - ThisSpaces - 1),
        tmpLinkPrefix + Copy(s, i + 1, Length(s)));
      if pos(' ',newnode.link)>0 then
        writeln(stderr,'Bad format imported node: name="',newnode.name,'" link="',newnode.link,'"');
      if Assigned(PrevSibling) then
        PrevSibling.FNextSibling := NewNode
      else
        CurParent.FFirstChild := NewNode;
      PrevSibling := NewNode;
    end;
  end;

  function ResolvePackageModule(AName:String;out pkg:TPasPackage;out module:TPasModule;createnew:boolean):String;
  var
    DotPos, DotPos2, i: Integer;
    s: String;
    HPackage: TPasPackage;

  begin
    pkg:=nil; module:=nil; result:='';

    // Find or create package
    DotPos := Pos('.', AName);
    s := Copy(AName, 1, DotPos - 1);
    HPackage := nil;
    for i := 0 to FPackages.Count - 1 do
      if CompareText(TPasPackage(FPackages[i]).Name, s) = 0 then
      begin
        HPackage := TPasPackage(FPackages[i]);
        break;
      end;
    if not Assigned(HPackage) then
    begin
      if not CreateNew then
        exit;
      HPackage := TPasPackage(inherited CreateElement(TPasPackage, s, nil,
        '', 0));
      FPackages.Add(HPackage);
    end;

    // Find or create module
    DotPos2 := DotPos;
    repeat
      Inc(DotPos2);
    until AName[DotPos2] = '.';
    s := Copy(AName, DotPos + 1, DotPos2 - DotPos - 1);
    Module := nil;
    for i := 0 to HPackage.Modules.Count - 1 do
      if CompareText(TPasModule(HPackage.Modules[i]).Name, s) = 0 then
      begin
        Module := TPasModule(HPackage.Modules[i]);
        break;
      end;
    if not Assigned(Module) then
    begin
      if not CreateNew then
        exit;
      Module := TPasExternalModule.Create(s, HPackage);
      Module.InterfaceSection := TInterfaceSection.Create('', Module);
      Module.PackageName:= HPackage.Name;
      // Module.AddRef{$IFDEF CheckPasTreeRefCount}('ReadContentFile.ResolvePackageModule'){$ENDIF};
      HPackage.Modules.Add(Module);
    end;
    pkg:=hpackage;
    result:=Copy(AName, DotPos2 + 1, length(AName)-dotpos2);
  end;

  function SearchInList(clslist:TFPList;s:string):TPasElement;
  var i : integer;
      ClassEl: TPasElement;
  begin
    result:=nil;
    for i:=0 to clslist.count-1 do
      begin
        ClassEl := TPasElement(clslist[i]);
        if CompareText(ClassEl.Name,s) =0 then
          exit(Classel); 
      end;
  end;

  function ResolveClassType(AName:String):TPasClassType;
  var 
     pkg     : TPasPackage;
     module  : TPasModule;
     s       : string; 
  begin
    Result:=nil;
    s:=ResolvePackageModule(AName,pkg,module,False);
    if not assigned(module) then
      exit;
    result:=TPasClassType(SearchInList(Module.InterfaceSection.Classes,s));
  end;

  function ResolveAliasType(AName:String):TPasAliasType;
  var 
     pkg     : TPasPackage;
     module  : TPasModule;
     s       : string; 
  begin
    Result:=nil;
    s:=ResolvePackageModule(AName,pkg,module,False);
    if not assigned(module) then
      exit;
    result:=TPasAliasType(SearchInList(Module.InterfaceSection.Types,s));
    if not (result is TPasAliasType) then
      result:=nil;
  end;

  procedure ReadClasses;

    function CreateClass(const AName: String;InheritanceStr:String): TPasClassType;
    var
      s: String;
      HPackage: TPasPackage;
      Module: TPasModule;

    begin
      s:= ResolvePackageModule(AName,HPackage,Module,True);
      // Create node for class
      Result := TPasExternalClassType.Create(s, Module.InterfaceSection);
      Result.ObjKind := okClass;
      // Result.AddRef{$IFDEF CheckPasTreeRefCount}('ReadContentFile.ResolveAndLinkClass'){$ENDIF};
      Module.InterfaceSection.Declarations.Add(Result);
      Module.InterfaceSection.Classes.Add(Result);
      // defer processing inheritancestr till all classes are loaded.
      if inheritancestr<>'' then
        InheritanceInfo.AddObject(Inheritancestr,result);
    end;

    procedure splitalias(var instr:string;out outstr:string);
    var i,j:integer;
    begin
     if length(instr)=0 then exit;
     instr:=trim(instr);
     i:=pos('(',instr);
     if i>0 then
      begin 
        j:=length(instr)-i;
        if instr[length(instr)]=')' then
          dec(j);
        outstr:=copy(instr,i+1,j);
        delete(instr,i,j+2);
      end
    end;

    Function ResolveAndLinkClass(clname:String;IsClass:boolean;cls:TPasClassType):TPasClassType;
    begin
     result:=TPasClassType(ResolveClassType(clname)); 
     if assigned(result) and not (cls=result) then  // save from tobject=implicit tobject
       begin
         result.addref{$IFDEF CheckPasTreeRefCount}('ReadContentFile.ResolveAndLinkClass'){$ENDIF};
         if IsClass then
           begin
             cls.ancestortype:=result;
//             writeln(cls.name, ' has as ancestor ',result.pathname);
           end
         else
           begin    
             cls.interfaces.add(result);
//             writeln(cls.name, ' implements ',result.pathname);
           end;
       end
     else
       if (dleXCT in FDocLogLevels) and (cls<>result) then
         DoLog('Warning : ancestor class %s of class %s could not be resolved',[clname,cls.name]);
    end;

    function CreateAliasType (alname,clname : string;parentclass:TPasClassType; out cl2 :TPasClassType):TPasAliasType;
    // create alias clname =  alname
    var
      pkg     : TPasPackage;
      module  : TPasModule;
      s       : string;
    begin
        Result:=nil;
        s:=ResolvePackageModule(Alname,pkg,module,True);
        if not assigned(module) then
          exit;
        cl2:=TPasClassType(ResolveClassType(alname));
        if assigned( cl2) and not (parentclass=cl2) then  
          begin
            result:=ResolveAliasType(clname);
            if assigned(result) then
              begin
    //            writeln('found alias ',clname,' (',s,') ',result.classname);
              end
            else
              begin
    //            writeln('new alias ',clname,' (',s,') ');
                cl2.addref{$IFDEF CheckPasTreeRefCount}('ReadContentFile.CreateAliasType'){$ENDIF};
                Result := TPasAliasType(CreateElement(TPasAliasType,s,module.interfacesection,vispublic,'',0));
                module.interfacesection.Declarations.Add(Result);
                TPasAliasType(Result).DestType := cl2;
              end
          end
    end;

    procedure ProcessInheritanceStrings(inhInfo:TStringList);

    var i,j : integer;
       cls : TPasClassType;
       cls2: TPasClassType;
       clname,
       alname : string;
       inhclass   : TStringList;
    begin
     inhclass:=TStringList.Create;
     inhclass.delimiter:=',';
     if InhInfo.Count>0 then
       for i:=0 to InhInfo.Count-1 do
         begin
           cls:=TPasClassType(InhInfo.Objects[i]);
           inhclass.clear; 
           inhclass.delimitedtext:=InhInfo[i];

           for j:= 0 to inhclass.count-1 do
             begin
               // writeln('processing',inhclass[j]);
               clname:=inhclass[j];
               splitalias(clname,alname);
               if alname<>'' then // the class//interface we refered to is an alias
                 begin
                   // writeln('Found alias pair ',clname,' = ',alname);
                   if (dleXCT in FDocLogLevels) and not assigned(CreateAliasType(alname,clname,cls,cls2)) then
                      DoLog('Warning: creating alias %s for %s failed!',[alname,clname]);
                 end 
               else
                 cls2:=ResolveAndLinkClass(clname,j=0,cls);
             end;
         end;
    inhclass.free;
    end;

  var
    s, Name: String;
    CurClass: TPasClassType;
    i: Integer;
    Member: TPasElement;
  begin
    inheritanceinfo :=TStringlist.Create;
    Try
      CurClass := nil;
      while True do
      begin
        ReadLn(f, s);
        if Length(s) = 0 then
          break;
        if s[1] = '#' then
        begin
          // New class
          i := Pos(' ', s);
          CurClass := CreateClass(Copy(s, 1, i - 1), copy(s,i+1,length(s)));
        end else
        begin
          i := Pos(' ', s);
          if i = 0 then
            Name := Copy(s, 3, Length(s))
          else
            Name := Copy(s, 3, i - 3);

          case s[2] of
            'M':
              Member := TPasProcedure.Create(Name, CurClass);
            'P':
              begin
                Member := TPasProperty.Create(Name, CurClass);
                if i > 0 then
                  while i <= Length(s) do
                  begin
                    case s[i] of
                      'r':
                        TPasProperty(Member).ReadAccessorName := '<dummy>';
                      'w':
                        TPasProperty(Member).WriteAccessorName := '<dummy>';
                      's':
                        TPasProperty(Member).StoredAccessorName := '<dummy>';
                    end;
                    Inc(i);
                  end;
              end;
            'V':
              Member := TPasVariable.Create(Name, CurClass);
            else
              raise Exception.Create('Invalid member type: ' + s[2]);
          end;
          CurClass.Members.Add(Member);
        end;
      end;
      ProcessInheritanceStrings(Inheritanceinfo);
    finally
      inheritanceinfo.Free;
    end;
  end;

var
  s: String;
  buf : TBufType;

begin
  buf:=Default(TBufType);
  if not FileExists(AFileName) then
    raise EInOutError.Create('File not found: ' + AFileName);
  Assign(f, AFilename);
  Reset(f);
  SetTextBuf(F,Buf,SizeOf(Buf));
  while not EOF(f) do
  begin
    ReadLn(f, s);
    if (Length(s) = 0) or (s[1] = '#') then
      continue;
    if s = ':link tree' then
      ReadLinkTree
    else if s = ':classes' then
      ReadClasses
    else
      repeat
        ReadLn(f, s);
      until EOF(f) or (Length(s) = 0);
  end;
  Close(f);
end;

procedure TFPDocEngine.WriteContentFile(const AFilename: String);
var
  ContentFile: Text;

  procedure ProcessLinkNode(ALinkNode: TLinkNode; const AIdent: String);
  var
    ChildNode: TLinkNode;
  begin
    WriteLn(ContentFile, AIdent, ALinkNode.Name, ' ', ALinkNode.Link);
    ChildNode := ALinkNode.FirstChild;
    while Assigned(ChildNode) do
    begin
      ProcessLinkNode(ChildNode, AIdent + ' ');
      ChildNode := ChildNode.NextSibling;
    end;
  end;

  function CheckImplicitLink(const s : String):String;
  begin
    if uppercase(s)='IUNKNOWN' then
     Result:='#rtl.System.IUnknown'
    else if uppercase(s)='TOBJECT' then
     Result:='#rtl.System.TObject'
   else
     Result:=s;
  end;
var
  LinkNode: TLinkNode;
  i, j, k: Integer;
  Module: TPasModule;
  Alias : TPasAliasType;
  MemberDecl: TPasMembersType;
  ClassLikeDecl : TPasClassType;
  Member: TPasElement;
  s: String;
  Buf : TBufType;

begin
  Buf:=Default(TBufType);
  Assign(ContentFile, AFilename);
  Rewrite(ContentFile);
  SetTextBuf(ContentFile,Buf,SizeOf(Buf));
  try
    WriteLn(ContentFile, '# FPDoc Content File');
    WriteLn(ContentFile, ':link tree');
    LinkNode := RootLinkNode.FirstChild;
    while Assigned(LinkNode) do
    begin
      if LinkNode.Name = Package.Name then
      begin
        ProcessLinkNode(LinkNode, '');
      end;
      LinkNode := LinkNode.NextSibling;
    end;

  if Assigned(Package) then
  begin
    WriteLn(ContentFile);
    WriteLn(ContentFile, ':classes');
    for i := 0 to Package.Modules.Count - 1 do
    begin
      Module := TPasModule(Package.Modules[i]);
      if not assigned(Module.InterfaceSection) then
        continue;
      for j := 0 to Module.InterfaceSection.Classes.Count - 1 do
        begin
        MemberDecl := TPasClassType(Module.InterfaceSection.Classes[j]);
        if MemberDecl is TPasClassType then
          ClassLikeDecl:=MemberDecl as TPasClassType
        else
          ClassLikeDecl:=nil;
        Write(ContentFile, CheckImplicitLink(MemberDecl.PathName), ' ');
        if Assigned(ClassLikeDecl) then
          begin
          if Assigned(ClassLikeDecl.AncestorType) then
            begin
            // simple aliases to class types are coded as "alias(classtype)"
            Write(ContentFile, CheckImplicitLink(ClassLikeDecl.AncestorType.PathName));
            if ClassLikeDecl.AncestorType is TPasAliasType then
               begin
               alias:= TPasAliasType(ClassLikeDecl.AncestorType);
               if assigned(alias.desttype) and (alias.desttype is TPasClassType) then
                  write(ContentFile,'(',alias.desttype.PathName,')');
               end;
            end
          else if ClassLikeDecl.ObjKind = okClass then
            Write(ContentFile, '#rtl.System.TObject')
          else if ClassLikeDecl.ObjKind = okInterface then
           Write(ContentFile, '#rtl.System.IUnknown');
          if ClassLikeDecl.Interfaces.Count>0 then
            begin
            for k:=0 to ClassLikeDecl.Interfaces.count-1 do
              begin
                write(contentfile,',',CheckImplicitLink(TPasType(ClassLikeDecl.Interfaces[k]).PathName));
                if TPasElement(ClassLikeDecl.Interfaces[k]) is TPasAliasType then
                  begin
                    alias:= TPasAliasType(ClassLikeDecl.Interfaces[k]);
                    if assigned(alias.desttype) and (alias.desttype is TPasClassType) then
                      write(ContentFile,'(',CheckImplicitLink(alias.desttype.PathName),')');
                  end;
              end;
            end;
          end;
        writeln(contentfile);
        for k := 0 to MemberDecl.Members.Count - 1 do
        begin
          Member := TPasElement(MemberDecl.Members[k]);
          Write(ContentFile, Chr(Ord(Member.Visibility) + Ord('0')));
          S:='';
          if Member.ClassType = TPasVariable then
            Write(ContentFile, 'V')
          else if Member.ClassType = TPasProperty then
          begin
            Write(ContentFile, 'P');
            if Length(TPasProperty(Member).ReadAccessorName) > 0 then
              s := s + 'r';
            if Length(TPasProperty(Member).WriteAccessorName) > 0 then
              s := s + 'w';
            if Length(TPasProperty(Member).StoredAccessorName) > 0 then
              s := s + 's';
          end else
            Write(ContentFile, 'M');    // Member must be a method
          Write(ContentFile, Member.Name);
          if Length(s) > 0 then
            WriteLn(ContentFile, ' ', s)
          else
            WriteLn(ContentFile);
        end;
      end;
    end;
  end;
  finally
    Close(ContentFile);
  end;
end;

function TFPDocEngine.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
begin
  Result := AClass.Create(AName, AParent);
  Result.Visibility := AVisibility;
  if AClass.InheritsFrom(TPasModule) then
    CurModule := TPasModule(Result);
  Result.SourceFilename := ASourceFilename;
  Result.SourceLinenumber := ASourceLinenumber;
end;

function TFPDocEngine.FindElement ( const AName: String; AModule: TPasModule
  ) : TPasElement;
var
  l: TFPList;
  i: Integer;

begin
  If Assigned(AModule) and Assigned(AModule.InterfaceSection) and
     Assigned(AModule.InterfaceSection.Declarations) then
    begin
    l:=AModule.InterfaceSection.Declarations;
    for i := 0 to l.Count - 1 do
      begin
      Result := TPasElement(l[i]);
      if CompareText(Result.Name, AName) = 0 then
        exit;
      end;
    end;
  Result := nil;
end;

function TFPDocEngine.FindElement(const AName: String): TPasElement;

var
  i: Integer;
  Module: TPasElement;
begin
  Result := FindElement( AName, CurModule );
  if not Assigned(Result) and assigned (CurModule.InterfaceSection) then
    for i := CurModule.InterfaceSection.UsesList.Count - 1 downto 0 do
    begin
      Module := TPasElement(CurModule.InterfaceSection.UsesList[i]);
      if Module.ClassType.InheritsFrom(TPasModule) then
      begin
        Result := FindElement(AName, TPasModule(Module));
        if Assigned(Result) then
          exit;
      end;
    end;
end;

function TFPDocEngine.FindModule(const AName: String): TPasModule;

  function FindInPackage(APackage: TPasPackage): TPasModule;
  var
    i: Integer;
  begin
    if not Assigned(APackage) then Exit;
    for i := 0 to APackage.Modules.Count - 1 do
    begin
      Result := TPasModule(APackage.Modules[i]);
      if CompareText(Result.Name, AName) = 0 then
        exit;
    end;
    Result := nil;
  end;

var
  i: Integer;
  AInPutLine,OSTarget,CPUTarget : String;

begin
  Result := FindInPackage(Package);
  if not Assigned(Result) and (FPackages.Count > 0) then
    for i := FPackages.Count - 1 downto 0 do
    begin
      if TPasPackage(FPackages[i]) = Package then
        continue;
      Result := FindInPackage(TPasPackage(FPackages[i]));
      if Assigned(Result) then
        exit;
    end;
  if Not Assigned(Result) and Assigned(FOnParseUnit) then
    begin
    FOnParseUnit(Self,AName,AInputLine,OSTarget,CPUTarget);
    If (AInPutLine<>'') then
      Result:=ParseUsedUnit(AName,AInputLine,OSTarget,CPUTarget);
    end;
end;

function TFPDocEngine.HintsToStr(Hints: TPasMemberHints): String;

Var
  H : TPasMemberHint;

begin
  Result:='';
  For h:=Low(TPasMemberHint) to High(TPasMemberHint) do
    if h in Hints then
      begin
      if (Result<>'') then
        Result:=Result+', ';
      Result:=Result+cPasMemberHint[h]
      end;
end;

function TFPDocEngine.ParseUsedUnit(AName, AInputLine, AOSTarget,
  ACPUTarget: String): TPasModule;

Var
  M : TPasModule;

begin
  if dleWarnUsedFile in FDocLogLevels then
    DoLog(SParsingUsedUnit,[AName,AInputLine]);
  M:=CurModule;
  CurModule:=Nil;
  try
    ParseSource(Self,AInputLine,AOSTarget,ACPUTarget,[poUseStreams]); //[poSkipDefaultDefs];
    Result:=CurModule;
  finally
    CurModule:=M;
  end;
end;

procedure TFPDocEngine.AddLink(const APathName, ALinkTo: String);
begin
  RootLinkNode.CreateChildren(APathName, ALinkTo);
end;

function TFPDocEngine.FindAbsoluteLink(const AName: String): String;
var
  LinkNode: TLinkNode;
begin
  LinkNode := RootLinkNode.FindChild(AName);
  if Assigned(LinkNode) then
    Result := LinkNode.Link
  else
    SetLength(Result, 0);
end;

function TFPDocEngine.ResolveLinkInPackages(AModule: TPasModule; const ALinkDest: String; Strict : Boolean = False): String;

Var
  ThisPackage: TLinkNode;

begin
  { Try all packages }
  Result:='';
  ThisPackage:=RootLinkNode.FirstChild;
  while Assigned(ThisPackage) and (Result='') do
    begin
    Result:=ResolveLink(AModule, ThisPackage.Name + '.' + ALinkDest, Strict);
    ThisPackage := ThisPackage.NextSibling;
    end;
end;

function TFPDocEngine.ResolveLinkInUsedUnits(AModule: TPasModule; const ALinkDest: String; Strict : Boolean = False): String;

var
  i: Integer;
  UL: TFPList;

begin
  Result:='';
  UL:=AModule.InterfaceSection.UsesList;
  I:=UL.Count-1;
  While (Result='') and (I>=0) do
    begin
    Result:=ResolveLinkInPackages(AModule,TPasType(UL[i]).Name+'.'+ALinkDest, strict);
    Dec(I);
    end;
end;

function TFPDocEngine.ResolveLink(AModule: TPasModule; const ALinkDest: String; Strict : Boolean = False): String;
var
  i: Integer;
begin
{
  if Assigned(AModule) then
      system.WriteLn('ResolveLink(', AModule.Name, ' - ', ALinkDest, ')... ')
    else
      system.WriteLn('ResolveLink(Nil - ', ALinkDest, ')... ');
}
  if (ALinkDest='') then
    Exit('');
  if (ALinkDest[1] = '#') then
    // Link has full path
    Result := FindAbsoluteLink(ALinkDest)
  else if (AModule=Nil) then
    // Trying to add package name only
    Result:= FindAbsoluteLink(RootLinkNode.FirstChild.Name+'.'+ALinkDest)
  else
    begin
    if Pos(LowerCase(AModule.Name)+'.',LowerCase(ALinkDest)) = 1 then
      // fix ERROR - Link starts from name of module
      Result := ResolveLink(AModule, AModule.packagename + '.' + ALinkDest, Strict)
    else
      // Link should be a first level inside of module
      Result := ResolveLink(AModule, AModule.PathName + '.' + ALinkDest, Strict);
    if (Result='') then
      begin
      Result:=ResolveLinkInPackages(AModule,ALinkDest,Strict);
      if (Result='') then
        Result:=ResolveLinkInUsedUnits(Amodule,AlinkDest,Strict);
      end;
    end;
  // Match on parent : class/enumerated/record/module
  if (Result='') and not strict then
    begin
    // TODO: I didn't see a calling this code at entire lcl package
    // Writeln('INFO UnStrinct(): ' + ALinkDest);
    for i := Length(ALinkDest) downto 1 do
      if ALinkDest[i] = '.' then
        begin
        Result := ResolveLink(AModule, Copy(ALinkDest, 1, i - 1), Strict);
        //if Result <> '' then Writeln('INFO LinkResolved UnStrinct(): '+Result);
        exit;
        end;
    end;
end;

procedure ReadXMLFileALT(OUT ADoc:TXMLDocument;const AFileName:ansistring);
var
  Parser: TDOMParser;
  Src: TXMLInputSource;
  FileStream: TStream;
begin
  ADoc := nil;
  FileStream := TFileStream.Create(AFilename, fmOpenRead+fmShareDenyWrite);
  try
    Parser := TDOMParser.Create; // create a parser object
    try
      Src := TXMLInputSource.Create(FileStream); // and the input source
      src.SystemId:=UTF8Decode(FileNameToUri(AFileName));
      try
        Parser.Options.PreserveWhitespace := True;
        Parser.Parse(Src, ADoc);
      finally
        Src.Free; // cleanup
      end;
    finally 
     Parser.Free;
     end;
  finally
    FileStream.Free;
  end;
end;

procedure TFPDocEngine.AddDocFile(const AFilename: String;DontTrim:boolean=false);

Var
  PN : String;

  function ReadNode(OwnerDocNode: TDocNode; Element: TDOMElement): TDocNode;
  var
    Subnode: TDOMNode;
  begin
    if OwnerDocNode = RootDocNode then
      Result := OwnerDocNode.CreateChildren('#' + UTF8Encode(Element['name']))
    else
      Result := OwnerDocNode.CreateChildren(UTF8Encode(Element['name']));
    Result.FNode := Element;
    Result.FLink := UTF8Encode(Element['link']);
    if (Element['alwaysvisible'] = '1') and (Element.NodeName='element') then
      FAlwaysVisible.Add(LowerCase(PN+'.'+TDocNode(OwnerDocNode).Name+'.'+UTF8Encode(Element['name'])));
    Result.FIsSkipped := Element['skip'] = '1';
    Subnode := Element.FirstChild;
    while Assigned(Subnode) do
    begin
      if Subnode.NodeType = ELEMENT_NODE then
      begin
        if Subnode.NodeName = 'short' then
          Result.FShortDescr := TDOMElement(Subnode)
        else if Subnode.NodeName = 'descr' then
          Result.FDescr := TDOMElement(Subnode)
        else if Subnode.NodeName = 'version' then
          begin
          Result.FVersion := TDOMElement(Subnode)
          end
        else if Subnode.NodeName = 'errors' then
          Result.FErrorsDoc := TDOMElement(Subnode)
        else if Subnode.NodeName = 'seealso' then
          Result.FSeeAlso := TDOMElement(Subnode)
        else if (Subnode.NodeName = 'example') and
          not Assigned(Result.FirstExample) then
          Result.FFirstExample := TDOMElement(Subnode)
        else if (Subnode.NodeName = 'notes') then
          Result.FNotes := TDOMElement(Subnode);
      end;
      Subnode := Subnode.NextSibling;
    end;
  end;

  Procedure ReadTopics(TopicNode : TDocNode);

  Var
    SubNode : TDOMNode;

  begin
    SubNode:=TopicNode.FNode.FirstChilD;
    While Assigned(SubNode) do
      begin
      If (SubNode.NodeType=ELEMENT_NODE) and (SubNode.NodeName='topic') then
        With ReadNode(TopicNode,TDomElement(SubNode)) do
          // We could allow recursion here, but we won't, because it doesn't work on paper.
          FTopicNode:=True;
      SubNode:=Subnode.NextSibling;
      end;
  end;

var
  Node, Subnode, Subsubnode: TDOMNode;
  Doc: TXMLDocument;
  PackageDocNode, TopicNode,ModuleDocNode: TDocNode;

begin
  if DontTrim then
    ReadXMLFileALT(Doc, AFilename)
  else
    ReadXMLFile(Doc, AFilename);
  DescrDocs.Add(Doc);
  DescrDocNames.Add(AFilename);

  Node := Doc.DocumentElement.FirstChild;
  while Assigned(Node) do
    begin
    if (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'package') then
      begin
      PackageDocNode := ReadNode(RootDocNode, TDOMElement(Node));
      PackageDocNode.IncRefCount;
      PN:=PackageDocNode.Name;
      // Scan all 'module' elements within this package element
      Subnode := Node.FirstChild;
      while Assigned(Subnode) do
        begin
        if (Subnode.NodeType = ELEMENT_NODE) then
          begin
          If (Subnode.NodeName = 'module') then
            begin
            ModuleDocNode := ReadNode(PackageDocNode, TDOMElement(Subnode));
            // Scan all 'element' elements within this module element
            Subsubnode := Subnode.FirstChild;
            while Assigned(Subsubnode) do
              begin
              if (Subsubnode.NodeType = ELEMENT_NODE) then
                begin
                if (Subsubnode.NodeName = 'element') then
                  ReadNode(ModuleDocNode, TDOMElement(Subsubnode))
                else if (SubSubNode.NodeName='topic') then
                  begin
                  TopicNode:=ReadNode(ModuleDocNode,TDomElement(SubSubNode));
                  TopicNode.FTopicNode:=True;
                  ReadTopics(TopicNode);
                  end;
                end;
              Subsubnode := Subsubnode.NextSibling;
              end;
            end
          else if (SubNode.NodeName='topic') then
            begin
            TopicNode:=ReadNode(PackageDocNode,TDomElement(SubNode));
            TopicNode.FTopicNode:=True;
            ReadTopics(TopicNode);
            end;
          end;
        Subnode := Subnode.NextSibling;
      end;
    end;
    Node := Node.NextSibling;
  end;
end;

function TFPDocEngine.FindDocNode(AElement: TPasElement): TDocNode;
begin
  Result:=Nil;
  If not Assigned(AElement) then
    exit;
  if aElement.CustomData is TDocNode then
    Exit(TDocNode(aElement.CustomData));
  if AElement.InheritsFrom(TPasUnresolvedTypeRef) then
    Result := FindDocNode(AElement.GetModule, AElement.Name)
  else
    begin
    Result := RootDocNode.FindChild(AElement.PathName);
    if (Result=Nil) and (AElement is TPasoperator) then
      Result:=RootDocNode.FindChild(TPasOperator(AElement).OldName(True));
    end;
  if (Result<>Nil) then
    begin
    if aElement.CustomData=Nil then
      aElement.CustomData:=Result;
    end
  else if (dleWarnNoNode in FDocLogLevels) and
          (Length(AElement.PathName)>0) and
          (AElement.PathName[1]='#') then
    DoLog(Format('No documentation node found for identifier : %s',[AElement.PathName]));
end;

function TFPDocEngine.FindDocNode(ARefModule: TPasModule;
  const AName: String): TDocNode;
var
  CurPackage: TDocNode;
  UnitList: TFPList;
  i: Integer;
begin
  if Length(AName) = 0 then
    Result := nil
  else
  begin
    if AName[1] = '#' then
      Result := RootDocNode.FindChild(AName)
    else
      Result := RootDocNode.FindChild(Package.Name + '.' + AName);
    if (not Assigned(Result)) and Assigned(ARefModule) then
      Result := RootDocNode.FindChild(ARefModule.PathName + '.' + AName);

    if (not Assigned(Result)) and (AName[1] <> '#') then
    begin
      CurPackage := RootDocNode.FirstChild;
      while Assigned(CurPackage) do
      begin
        Result := RootDocNode.FindChild(CurPackage.Name + '.' + AName);
        if Assigned(Result) then
          break;
        CurPackage := CurPackage.NextSibling;
      end;
      if not Assigned(Result) and assigned(CurModule.InterfaceSection) then
      begin
        { Okay, then we have to try all imported units of the current module }
        UnitList := CurModule.InterfaceSection.UsesList;
        for i := UnitList.Count - 1 downto 0 do
        begin
          { Try all packages }
          CurPackage := RootDocNode.FirstChild;
          while Assigned(CurPackage) do
          begin
            Result := RootDocNode.FindChild(CurPackage.Name + '.' +
              TPasType(UnitList[i]).Name + '.' + AName);
            if Assigned(Result) then
              break;
            CurPackage := CurPackage.NextSibling;
          end;
        end;
      end;
    end;
  end;
end;

function TFPDocEngine.FindShortDescr(AElement: TPasElement): TDOMElement;

var
  DocNode,N: TDocNode;

begin
  DocNode := FindDocNode(AElement);
  if Assigned(DocNode) then
    begin
    N:=FindLinkedNode(DocNode);
    If (N<>Nil) then
      DocNode:=N;
    Result := DocNode.ShortDescr;
    end
  else
    Result := nil;
end;


function TFPDocEngine.FindLinkedNode(ANode : TDocNode) : TDocNode;

begin
  If (ANode.Link='') then
    Result:=Nil
  else
    Result:=FindDocNode(CurModule,ANode.Link);
end;

function TFPDocEngine.ShowElement(El: TPasElement): Boolean;
begin
  Case El.Visibility of
    visStrictPrivate,
    visPrivate :
      Result:=Not HidePrivate;
    visStrictProtected,
    visProtected :
      begin
      Result:=Not HideProtected;
      if not Result then
        Result:=FAlwaysVisible.IndexOf(LowerCase(El.PathName))<>-1;
      end
  Else
    Result:=True
  end;
end;

procedure TFPDocEngine.StartDocumenting;
begin
  FAlwaysVisible.Sorted:=True;
end;

function TFPDocEngine.FindShortDescr(ARefModule: TPasModule;
  const AName: String): TDOMElement;

var
  N,DocNode: TDocNode;

begin
  DocNode := FindDocNode(ARefModule, AName);
  if Assigned(DocNode) then
    begin
    N:=FindLinkedNode(DocNode);
    If (N<>Nil) then
      DocNode:=N;
    Result := DocNode.ShortDescr;
    end
  else
    Result := nil;
end;

function TFPDocEngine.GetExampleFilename(const ExElement: TDOMElement): String;
var
  i: Integer;
  fn : String;
  
begin
  Result:='';
  for i := 0 to DescrDocs.Count - 1 do
    begin
    Fn:=UTF8Encode(ExElement['file']);
    if (FN<>'') and (TDOMDocument(DescrDocs[i]) = ExElement.OwnerDocument) then
      begin
      Result := ExtractFilePath(DescrDocNames[i]) + FN;
      if (ExtractFileExt(Result)='') then
        Result:=Result+'.pp';
      end;
    end;  
end;


{ Global helpers }

procedure TranslateDocStrings(const Lang: String);

Const
{$ifdef unix}
  DefDir = '/usr/local/share/locale';
{$else}  
  DefDir = 'intl';
{$endif}

var
  mo: TMOFile;
  dir : string;
begin
  dir:=modir;
  If Dir='' then
    Dir:=DefDir;
  Dir:=IncludeTrailingPathDelimiter(Dir);
{$IFDEF Unix}
  mo := TMOFile.Create(Format(Dir+'%s/LC_MESSAGES/dglobals.mo', [Lang]));
{$ELSE}
  mo := TMOFile.Create(Format(Dir+'dglobals.%s.mo', [Lang]));
{$ENDIF}
  try
    TranslateResourceStrings(mo);
  finally
    mo.Free;
  end;
end;

Function IsLinkNode(Node : TDomNode) : Boolean;

begin
  Result:=Assigned(Node) and (Node.NodeType = ELEMENT_NODE) and (Node.NodeName = 'link');
end;

Function IsExampleNode(Example : TDomNode) : Boolean;

begin
  Result:=Assigned(Example) and (Example.NodeType = ELEMENT_NODE) and (Example.NodeName = 'example')
end;

function IsLinkAbsolute(ALink: String): boolean;
var
  i: integer;
begin
  Result := false;
  for i := low(AbsoluteLinkPrefixes) to high(AbsoluteLinkPrefixes) do
    if CompareText(AbsoluteLinkPrefixes[i], copy(ALink,1,length(AbsoluteLinkPrefixes[i])))=0 then begin
      Result := true;
      break;
    end;
end;

{$IFDEF EXCEPTION_STACK}
function DumpExceptionCallStack(E: Exception):String;
var
  I: Integer;
  Frames: PPointer;
begin
  Result := 'Program exception! ' + LineEnding +
    'Stacktrace:' + LineEnding + LineEnding;
  if E <> nil then begin
    Result := Result + 'Exception class: ' + E.ClassName + LineEnding +
    'Message: ' + E.Message + LineEnding;
  end;
  Result := Result + BackTraceStrFunc(ExceptAddr);
  Frames := ExceptFrames;
  for I := 0 to ExceptFrameCount - 1 do
    Result := Result + LineEnding + BackTraceStrFunc(Frames[I]);
end;
{$ENDIF}

initialization
  LEOL:=Length(LineEnding);
end.
