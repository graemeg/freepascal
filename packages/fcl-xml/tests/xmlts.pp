{
    This file is part of the Free Component Library (FCL)

    FCL test runner for OASIS/NIST XML test suite
    It is somewhat based on 'harness.js' script
    (see http://xmlconf.sourceforge.net)
    Copyright (c) 2006 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program xmlts;

{$IFDEF FPC}
{$MODE OBJFPC}{$H+}
{$ENDIF}
{$APPTYPE CONSOLE}

uses
  SysUtils,
  Classes,
  DOM,
  XMLRead,
  XMLWrite,
  UriParser;

const
  harness = 'Pascal version';
  version = '0.0.1 alpha :)';
  parser = 'FCL XML parser';
  parserName = parser;
  os = 'Unknown OS';
  runtime = 'FPC RTL';
<<<<<<< HEAD
<<<<<<< HEAD
  { Defines which tests to skip (sets for editions 1-4 and edition 5 are mutually exclusive) }
  FifthEditionCompliant = True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


type
  TDiagCategory = (dcInfo, dcNegfail, dcFail, dcPass);

  TTestSuite = class
  private
    FTemplate: TXMLDocument;
    FParser: TDOMParser;
    FPassed, FFailCount: Integer;
    FFalsePasses: Integer;
    FRootUri: string;
    FSuiteName: string;
    FDoc: TXMLDocument;
    FValidating: Boolean;
    FSuiteTitle: DOMString;
    FState: DOMString;
    FSkipped: Integer;
    FTotal: Integer;
    table_valid: TDOMNode;
    table_output: TDOMNode;
    table_invalid: TDOMNode;
    table_not_wf: TDOMNode;
    table_informative: TDOMNode;
    FValError: string;
    FTestID: DOMString;
    FErrLine, FErrCol: Integer;
    procedure LoadTemplate(const Name: string);
    procedure HandleTemplatePIs(Element: TDOMNode);
    procedure Diagnose(Element, Table: TDOMNode; Category: TDiagCategory; const Error: DOMString);
    procedure DiagnoseOut(const ErrorMsg: DOMString);
    function CompareNodes(actual, correct: TDOMNode; out Msg: string): Boolean;
    procedure ErrorHandler(Error: EXMLReadError);
  public
    constructor Create;
    procedure Run(const Tests: string);
    procedure RunTest(Element: TDOMElement);
    destructor Destroy; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
{ obsolete, now TDOMNode.BaseURI does the job }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function GetBaseURI(Element: TDOMNode; const DocumentURI: string): string;
var
  Ent: TDOMNode;
  Uri1, Uri2, s: WideString;
begin
  case Element.NodeType of
  ELEMENT_NODE, TEXT_NODE, CDATA_SECTION_NODE,
  PROCESSING_INSTRUCTION_NODE, COMMENT_NODE, DOCUMENT_TYPE_NODE:
    if Assigned(Element.ParentNode)
      then Result := GetBaseURI(Element.ParentNode, DocumentURI)
      else Result := '';

  ATTRIBUTE_NODE: begin
    Result := '';
    if Assigned(TDomAttr(Element).OwnerElement) then
    begin
      Result := GetBaseURI(TDomAttr(Element).OwnerElement, DocumentURI);
    end;
  end;

  ENTITY_REFERENCE_NODE: begin
    Ent := Element.OwnerDocument.DocType.Entities.GetNamedItem(Element.NodeName);
    if Assigned(Ent) and (TDOMEntity(Ent).SystemID <> '') then
    begin
      Uri1 := TDOMEntity(Ent).SystemID;
      if IsAbsoluteURI(Uri1) then
      begin
        Result := Uri1;
      end else begin
        Uri2 := GetBaseURI(Element.ParentNode, DocumentUri);
        ResolveRelativeUri(Uri2, Uri1, s);
        Result := s;
      end;
    end
    else
    begin
      if Assigned(Element.ParentNode)
        then Result := GetBaseURI(Element.ParentNode, DocumentURI)
        else Result := '';
    end;
  end;

  DOCUMENT_NODE: Result := DocumentURI;
  else
    Result := '';
  end;
end;

{ TTestSuite }

constructor TTestSuite.Create;
begin
  inherited Create;
  FParser := TDOMParser.Create;
  FParser.Options.PreserveWhitespace := True;
  FParser.Options.ExpandEntities := True;
  FParser.Options.IgnoreComments := True;
  FParser.Options.CDSectionsAsText := True;
end;

procedure TTestSuite.ErrorHandler(Error: EXMLReadError);
begin
  // allow fatal error position to override that of validation error
  if (FErrLine < 0) or (Error.Severity = esFatal) then
  begin
    FErrLine := Error.Line;
    FErrCol := Error.LinePos;
  end;  

  if Error.Severity = esError then
  begin
    if FValError = '' then // fetch the _first_ message
      FValError := Error.Message;
{ uncomment the line below to verify that the suite correctly handles
  exception raised from the handler }    
//  Abort;  
  end;
end;

procedure TTestSuite.LoadTemplate(const Name: string);
var
  tables: TDOMNodeList;
  I: Integer;
  id: DOMString;
  el: TDOMElement;
begin
  ReadXMLFile(FTemplate, Name);
  tables := FTemplate.DocumentElement.GetElementsByTagName('table');
  try
    for I := 0 to tables.Count-1 do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
      el := TDOMElement(tables[I]);
=======
      el := TDOMElement(tables.Item[I]);
>>>>>>> graemeg/fixes_2_2
=======
      el := TDOMElement(tables.Item[I]);
>>>>>>> origin/fixes_2_2
      id := el['id'];
      if id = 'valid' then
        table_valid := el
      else if ((id = 'invalid-negative') and FValidating) or ((id = 'invalid-positive') and not FValidating) then
        table_invalid := el
      else if id = 'valid-output' then
        table_output := el
      else if id = 'not-wf' then
        table_not_wf := el
      else if id = 'error' then
        table_informative := el;
    end;
  finally
    tables.Free;
  end;
end;

destructor TTestSuite.Destroy;
begin
  FDoc.Free;
  FTemplate.Free;
  FParser.Free;
  inherited;
end;

procedure TTestSuite.HandleTemplatePIs(Element: TDOMNode);
var
  Children: TDOMNodeList;
  Child: TDOMNode;
  NewChild: TDOMNode;
  Remove: Boolean;
  Index: Integer;
  Data: DOMString;
begin
  Children := element.childNodes;
  Remove := False;
  Index := 0;

  repeat
<<<<<<< HEAD
<<<<<<< HEAD
    Child := Children[Index];
=======
    Child := Children.Item[Index];
>>>>>>> graemeg/fixes_2_2
=======
    Child := Children.Item[Index];
>>>>>>> origin/fixes_2_2
    if Child = nil then Break;
    Inc(index);

    // inside a rejected <?if ...?>...<?endif?>
    if Remove and (child.nodeType <> PROCESSING_INSTRUCTION_NODE) then
    begin
      Element.removeChild(child);
      Dec(Index);
      Continue;
    end;
    if Child.hasChildNodes then
    begin
      HandleTemplatePIs(Child);
      Continue;
    end;

    if Child.nodeType <> PROCESSING_INSTRUCTION_NODE then
      Continue;

    Data := Child.NodeValue;

    if Child.NodeName = 'run-id' then
    begin
      newChild := nil;
      if Data = 'name' then
        newChild := FTemplate.createTextNode(parser)
      else if Data = 'description' then
        newChild := FTemplate.createTextNode (parserName)
      else if Data = 'general-entities' then
        newChild := FTemplate.createTextNode('included')
      else if Data = 'parameter-entities' then
        newChild := FTemplate.createTextNode ('included')
      else if Data = 'type' then
      begin
        if FValidating then
           Data := 'Validating'
        else
           Data := 'Non-Validating';
        newChild := FTemplate.createTextNode(Data);
      end
      // ... test run description
      else if Data = 'date' then
        newChild := FTemplate.createTextNode(DateTimeToStr(Now))
      else if Data = 'harness' then
        newChild := FTemplate.createTextNode(harness)
      else if Data = 'java' then
        newChild := FTemplate.createTextNode(runtime)
      else if Data = 'os' then
        newChild := FTemplate.createTextNode(os)
      else if Data = 'testsuite' then
        newChild := FTemplate.createTextNode(FSuiteTitle)
      else if Data = 'version' then
        newChild := FTemplate.createTextNode(version)
      // ... test result info
      else if Data = 'failed' then
        newChild := FTemplate.createTextNode(IntToStr(FFailCount))
      else if Data = 'passed' then
        newChild := FTemplate.createTextNode(IntToStr(FPassed))
      else if Data = 'passed-negative' then
        newChild := FTemplate.createTextNode(IntToStr(FFalsePasses))
      else if Data = 'skipped' then
        newChild := FTemplate.createTextNode(IntToStr(FSkipped))
      else if Data = 'status' then
        newChild := FTemplate.createTextNode (FState);

      Element.replaceChild (newChild, child);
      Continue;
    end

    // if/endif don't nest, and always have the same parent
    // we rely on those facts here!
    else if Child.NodeName = 'if' then
    begin
      Remove := not (((Data = 'validating') and FValidating) or
                   ((Data = 'nonvalidating') and not FValidating));
      element.removeChild(child);
      Dec(Index);
      Continue;
    end
    else if Child.NodeName = 'endif' then
    begin
      Remove := False;
      element.removeChild(child);
      Dec(Index);
      Continue;
    end;
  until False;
  Children.Free;
end;


procedure TTestSuite.Run(const Tests: string);
var
  Cases: TDOMNodeList;
  I: Integer;
begin
  FRootURI := FilenameToURI(Tests);
<<<<<<< HEAD
<<<<<<< HEAD
  writeln('Loading test suite from ', Tests);
  ReadXMLFile(FDoc, Tests);
  FSuiteTitle := FDoc.DocumentElement['PROFILE'];
  Cases := FDoc.DocumentElement.GetElementsByTagName('TEST');
=======
=======
>>>>>>> origin/fixes_2_2
  ReadXMLFile(FDoc, Tests);
  FSuiteTitle := FDoc.DocumentElement['PROFILE'];
  Cases := FDoc.DocumentElement.GetElementsByTagName('TEST');
  writeln('Using test suite: ', Tests);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  writeln;
  writeln('Testing, validation = ', FValidating);
  try
    for I := 0 to Cases.Count-1 do
<<<<<<< HEAD
<<<<<<< HEAD
      RunTest(Cases[I] as TDOMElement);
=======
      RunTest(Cases.Item[I] as TDOMElement);
>>>>>>> graemeg/fixes_2_2
=======
      RunTest(Cases.Item[I] as TDOMElement);
>>>>>>> origin/fixes_2_2
    I := Cases.Count;
  finally
    Cases.Free;
  end;

  FPassed := FTotal-FFailCount;
  Dec(FPassed, FSkipped);

  writeln('Found ', I, ' basic test cases.');
  writeln('Found ', FTotal, ' overall test cases.');
  writeln('Skipped: ', FSkipped);
  writeln('Passed: ', FPassed);
  writeln('Failed: ', FFailCount);
  writeln('Negative passes: ', FFalsePasses, ' (need examination).');
  writeln;

  if FPassed = 0 then
    FState := 'N/A'
  else if FPassed = FTotal - FSkipped then
    FState := 'CONFORMS (provisionally)'
  else
    FState := 'DOES NOT CONFORM';

end;

procedure TTestSuite.RunTest(Element: TDOMElement);
var
<<<<<<< HEAD
<<<<<<< HEAD
  s: string;
=======
  s: UTF8string;
>>>>>>> graemeg/fixes_2_2
=======
  s: UTF8string;
>>>>>>> origin/fixes_2_2
  TestType: DOMString;
  TempDoc, RefDoc: TXMLDocument;
  table: TDOMNode;
  Positive: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
  outURI: string;
  FailMsg: string;
  ExceptionClass: TClass;
=======
  outURI: UTF8string;
  FailMsg: string;
>>>>>>> graemeg/fixes_2_2
=======
  outURI: UTF8string;
  FailMsg: string;
>>>>>>> origin/fixes_2_2
  docNode, refNode: TDOMNode;
  docMap, refMap: TDOMNamedNodeMap;
  docN, refN: TDOMNotation;
  I: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
  root: string;
  xmlEdition: DOMString;
=======
  root: UTF8String;
>>>>>>> graemeg/fixes_2_2
=======
  root: UTF8String;
>>>>>>> origin/fixes_2_2
begin
  FErrLine := -1;
  FErrCol := -1;
  FTestID := Element['ID'];
  TestType := Element['TYPE'];
<<<<<<< HEAD
<<<<<<< HEAD
  xmlEdition := Element['EDITION'];
  if (xmlEdition <> '') and ((Pos(WideChar('5'), Element['EDITION']) = 0) = FifthEditionCompliant) then
=======
  if Pos(WideChar('5'), Element['EDITION']) > 0 then
>>>>>>> graemeg/fixes_2_2
=======
  if Pos(WideChar('5'), Element['EDITION']) > 0 then
>>>>>>> origin/fixes_2_2
  begin
    Inc(FSkipped);
    Exit;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  root := Element.BaseURI;
=======
  root := GetBaseURI(Element, FRootUri);
>>>>>>> graemeg/fixes_2_2
=======
  root := GetBaseURI(Element, FRootUri);
>>>>>>> origin/fixes_2_2
  ResolveRelativeURI(root, UTF8Encode(Element['URI']), s);

  table := nil;
  outURI := '';
  Positive := False;
  if TestType = 'not-wf' then
    table := table_not_wf
  else if TestType = 'error' then
    table := table_informative
  else if TestType = 'valid' then
  begin
    if Element.hasAttribute('OUTPUT') then
      ResolveRelativeURI(root, UTF8Encode(Element['OUTPUT']), outURI);
    table := table_valid;
    Positive := True;
  end
  else if TestType = 'invalid' then
  begin
    table := table_invalid;
    Positive := not FValidating;
  end;

  if TestType <> 'error' then
  begin
    Inc(FTotal);
    if outURI <> '' then Inc(FTotal);
  end;

  FailMsg := '';
  FValError := '';
  TempDoc := nil;
  try
    try
      FParser.Options.Validate := FValidating;
      FParser.Options.Namespaces := (Element['NAMESPACE'] <> 'no');
      FParser.OnError := {$IFDEF FPC}@{$ENDIF}ErrorHandler;
      FParser.ParseUri(s, TempDoc);
    except
      on E: Exception do
        if E.ClassType <> EAbort then
        begin
<<<<<<< HEAD
<<<<<<< HEAD
          ExceptionClass := E.ClassType;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          FailMsg := E.Message;
          FValError := '';
        end;
    end;

    if table = table_informative then
    begin
      if FailMsg <> '' then
        Diagnose(element, table, dcInfo, '(fatal) ' + FailMsg)
      else if FValError <> '' then
        Diagnose(element, table, dcInfo, '(error) ' + FValError)
      else
        Diagnose(Element, table, dcInfo, '');
      Exit;
    end;

    if not Positive then  // must have been failed
    begin
      if (FailMsg = '') and (FValError = '') then
      begin
        Inc(FFailCount);
        Diagnose(element, table, dcNegfail, '');
      end
      else // FailMsg <> '' or FValError <> '' -> actually failed
      begin
        if FailMsg <> '' then  // Fatal error
        begin
          { outside not-wf category it is a test failure }
<<<<<<< HEAD
<<<<<<< HEAD
          if (table <> table_not_wf) or (ExceptionClass <> EXMLReadError) then
=======
          if table <> table_not_wf then
>>>>>>> graemeg/fixes_2_2
=======
          if table <> table_not_wf then
>>>>>>> origin/fixes_2_2
          begin
            Inc(FFailCount);
            Diagnose(Element, table, dcFail, FailMsg);
          end
          else
          begin
            Inc(FFalsePasses);
            Diagnose(Element, table, dcPass, FailMsg);
          end;
        end
        else
        begin
          { outside invalid category it is a test failure }
          if table = table_not_wf then
          begin
            Inc(FFailCount);
            Diagnose(Element, table, dcFail, FValError);
          end
          else
          begin
            Inc(FFalsePasses);
            Diagnose(Element, table, dcPass, FValError);
          end;
        end;
      end;
      Exit;
    end
    else   // must have been succeeded
      if (FailMsg <> '') or (FValError <> '') then
      begin
        Inc(FFailCount);
        if FailMsg <> '' then
          Diagnose(Element, table, dcFail, FailMsg)
        else
          Diagnose(Element, table, dcFail, FValError);
        if (outURI <> '') and (FailMsg <> '') then
        begin
          Inc(FFailCount);
          DiagnoseOut('[ input failed, no output to test ]');
        end;
        Exit;
      end;

    if outURI = '' then Exit;
<<<<<<< HEAD
<<<<<<< HEAD
=======
    TempDoc.DocumentElement.Normalize;
>>>>>>> graemeg/fixes_2_2
=======
    TempDoc.DocumentElement.Normalize;
>>>>>>> origin/fixes_2_2
    try
      // reference data must be parsed in non-validating mode because it contains DTDs
      // only when Notations need to be reported
      FParser.Options.Validate := False;
      FParser.ParseUri(outURI, RefDoc);
      try
        docNode := TempDoc.FirstChild;
        refNode := RefDoc.FirstChild;
        repeat
          if refNode = nil then
          begin
            if docNode <> nil then
            begin
              Inc(FFailCount);
              DiagnoseOut('Extra data: ' + docNode.NodeName + ' / ' + docNode.NodeValue);
            end;
            Exit;
          end;
          if docNode = nil then
          begin
            Inc(FFailCount);
            DiagnoseOut('Missing data: ' + refNode.NodeName + ' / ' + refNode.NodeValue);
            Exit;
          end;

          if refNode.NodeType = DOCUMENT_TYPE_NODE then
          begin
            if docNode.NodeType <> DOCUMENT_TYPE_NODE then
            begin
              Inc(FFailCount);
              DiagnoseOut('[ no doctype from parsing testcase ]');
              Exit;
            end;

            refMap := TDOMDocumentType(refNode).Notations;
            docMap := TDOMDocumentType(docNode).Notations;

            for I := 0 to refMap.Length-1 do
            begin
              refN := TDOMNotation(refMap[I]);
              docN := TDOMNotation(docMap.GetNamedItem(refMap[I].NodeName));
              if not Assigned(docN) then
              begin
                Inc(FFailCount);
                DiagnoseOut('missing notation declaration: ' + refN.NodeName);
                Exit;
              end;
              if (refN.PublicID <> docN.PublicID) or (refN.SystemID <> docN.SystemID) then
              begin
                Inc(FFailCount);
                DiagnoseOut('incorrect notation declaration: ' + refN.NodeName);
                Exit;
              end;
            end;

            refNode := refNode.NextSibling;
            docNode := docNode.NextSibling;
            Continue;
          end;

          if docNode.NodeType = DOCUMENT_TYPE_NODE then  // skip DocType
            docNode := docNode.NextSibling;

          if not CompareNodes(docNode, refNode, FailMsg) then
          begin
            Inc(FFailCount);
            DiagnoseOut(FailMsg);
            Exit;
          end;

          docNode := docNode.NextSibling;
          refNode := refNode.NextSibling;
        until False;
      finally
        RefDoc.Free;
      end;
    except
      on E: Exception do
      begin
        Inc(FFailCount);
        DiagnoseOut('[ can''t read reference data: '+E.Message+' ]');
      end;
    end;
  finally
    TempDoc.Free;
  end;
end;


procedure TTestSuite.Diagnose(Element, Table: TDOMNode; Category: TDiagCategory;
  const Error: DOMString);
var
  tr, td, txt, tmp: TDOMNode;
  s: DOMString;
begin
  tr := FTemplate.CreateElement('tr');
  if Assigned(Element) then              // column 1: section/chapter, if known
  begin
    s := TDOMElement(Element)['SECTIONS'];
    td := FTemplate.CreateElement('td');
    td.AppendChild(FTemplate.CreateTextNode(s));
    tr.AppendChild(td);
  end;

  td := FTemplate.CreateElement('td');   // column 2: test ID
  td.AppendChild(FTemplate.CreateTextNode(FTestID));
  tr.AppendChild(td);
  // third column is description
  if Assigned(Element) then
  begin
    td := FTemplate.CreateElement('td');
    txt := Element.FirstChild;
    while Assigned(txt) do
    begin
      td.AppendChild(txt.CloneNode(true, FTemplate));
      txt := txt.NextSibling;
    end;
    tr.AppendChild(td);
  end;
  // fourth column is reason
  td := FTemplate.CreateElement('td');
  if Element = nil then
    s := Error
  else if Category <> dcInfo then
  begin
    if Error <> '' then
    begin
      if FValError <> '' then
        s := '(error) ' + Error
      else
        s := '(fatal) ' + Error;
    end
    else
      s := '[wrongly accepted]';
  end
  else // informative
  begin
    if Error <> '' then
      s := Error
    else
      s := '[accepted]';
  end;
  // TODO: use &nbsp if text is empty
  txt := FTemplate.CreateTextNode(s);

  if (Category <> dcPass) and (Category <> dcInfo) then
  begin
    tmp := FTemplate.CreateElement('em');
    tmp.AppendChild(txt);
    txt := tmp;
    TDOMElement(td)['bgcolor'] := '#ffaacc';
  end;
  td.AppendChild(txt);
  tr.AppendChild(td);

  table.AppendChild(tr);
end;

procedure TTestSuite.DiagnoseOut(const ErrorMsg: DOMString);
var
  tr, td, txt: TDOMNode;
begin
  tr := FTemplate.CreateElement('tr');

  td := FTemplate.CreateElement('td');
  td.AppendChild(FTemplate.CreateTextNode(FTestID));
  tr.AppendChild(td);

  td := FTemplate.CreateElement('td');
  txt := FTemplate.CreateElement('em');
  txt.AppendChild(FTemplate.CreateTextNode(ErrorMsg));
  td.AppendChild(txt);
  TDOMElement(td)['bgcolor'] := '#ffaacc';
  tr.AppendChild(td);
  table_output.AppendChild(tr);
end;

procedure Canonicalize(node: TDOMNode);
var
  child, work: TDOMNode;
  Frag: TDOMDocumentFragment;
begin
  child := node.FirstChild;
  while Assigned(child) do
  begin
    if child.NodeType = CDATA_SECTION_NODE then
    begin
      work := node.OwnerDocument.CreateTextNode(child.NodeValue);
      node.ReplaceChild(work, child);
      child := work;
    end
    else if child.NodeType = COMMENT_NODE then
    begin
      work := child.NextSibling;
      node.RemoveChild(child);
      child := work;
      Continue;
    end
    else if child.NodeType = ENTITY_REFERENCE_NODE then
    begin
      Frag := node.OwnerDocument.CreateDocumentFragment;
      try
        work := child.FirstChild;
        while Assigned(work) do
        begin
          Frag.AppendChild(work.CloneNode(true));
          work := work.NextSibling;
        end;
        work := Frag.FirstChild;     // references may be nested
        if work = nil then
          work := Child.PreviousSibling;

        node.ReplaceChild(Frag, child);
        child := work;
      finally
        Frag.Free;
      end;
      Continue;
    end;
    if child.HasChildNodes then
      Canonicalize(child);
    child := child.NextSibling;
  end;
end;

function TTestSuite.CompareNodes(actual, correct: TDOMNode;
  out Msg: string): Boolean;
var
  actAtts, refAtts: TDOMNamedNodeMap;
  actList, refList: TDOMNodeList;
  I: Integer;
  s1, s2: DOMString;
begin
  Msg := '';
  Result := False;
  if actual.NodeType <> correct.NodeType then
    FmtStr(Msg, 'actual.NodeType (%d) != correct.NodeType (%d)', [actual.NodeType, correct.NodeType])
  else if actual.NodeName <> correct.NodeName then
    FmtStr(Msg, 'actual.NodeName (%s) != correct.NodeName (%s)', [actual.NodeName, correct.NodeName])
  else if actual.NodeValue <> correct.NodeValue then
    FmtStr(Msg, 'actual.NodeValue (%s) != correct.NodeValue (%s)', [actual.NodeValue, correct.NodeValue]);
  if Msg <> '' then
    Exit;

  if actual.NodeType = ELEMENT_NODE then
  begin
    // first, compare attributes
    actAtts := actual.Attributes;
    refAtts := correct.Attributes;
    if actAtts.Length <> refAtts.Length then
    begin
      FmtStr(Msg, 'Element ''%s'': attributes.length (%d) != %d', [actual.NodeName, actAtts.Length, refAtts.Length]);
      Exit;
    end;
    for I := 0 to actAtts.Length -1 do
    begin
      s1 := refAtts.GetNamedItem(actAtts[I].NodeName).NodeValue;
      s2 := actAtts[I].NodeValue;
      if s1 <> s2 then
      begin
        FmtStr(Msg, 'Element ''%s'', attribute ''%s'': actual.AttValue (%s) != correct.AttValue (%s)', [actual.NodeName, actAtts[I].NodeName, s2, s1]);
        Exit;
      end;
    end;
    // next, compare children
    actList := actual.ChildNodes;
    refList := correct.ChildNodes;
    try
      if actList.Count <> refList.Count then
      begin
        FmtStr(Msg, 'Element ''%s'': actual.ChildNodeCount (%d) != correct.ChildNodeCount (%d)', [actual.NodeName, actList.Count, refList.Count]);
        Exit;
      end;
      for I := 0 to actList.Count -1 do
        if not CompareNodes(actList[I], refList[I], Msg) then
          Exit;
    finally
      actList.Free;
      refList.Free;
    end;
  end;
  Result := True;
end;



var
  i: Integer;
  s: string;
  SuiteName, ReportName, TemplateName: string;
  Validation: Boolean;
begin
  writeln('FCL driver for OASIS/NIST XML Test Suite');
  writeln('Copyright (c) 2006 by Sergei Gorelkin');
  TemplateName := ExtractFilePath(ParamStr(0)) + 'template.xml';
  if ParamCount < 2 then
  begin
    writeln;
    writeln('Usage: ', ParamStr(0), ' <suite> <report> [-t template][-v]');
    writeln('  -t: specify report template');
    writeln('  -v: validating mode');
    Exit;
  end;

  SuiteName := ExpandFilename(ParamStr(1));
  ReportName := ExpandFilename(ParamStr(2));
  i := 3;
  Validation := False;
  while i <= ParamCount do
  begin
    s := Lowercase(ParamStr(i));
    if s = '-v' then
      Validation := True
    else if s = '-t' then
      TemplateName := ExpandFileName(ParamStr(i+1));
    Inc(i);
  end;

  with TTestSuite.Create do
  try
    FSuiteName := SuiteName;
    FValidating := Validation;
    LoadTemplate(TemplateName);
    if Assigned(FTemplate) then
    begin
      Run(FSuiteName);
      HandleTemplatePIs(FTemplate.DocumentElement);
      writeln('Writing report to: ', ReportName);
      WriteXMLFile(FTemplate, ReportName);
    end;
  finally
    Free;
  end;

end.
