{**********************************************************************

    This file is part of the Free Component Library (FCL)

    Generates fpcunit code from w3.org XML test descriptions
    Copyright (c) 2008 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program testgen;
{$mode objfpc}{$H+}

uses
  Classes, SysUtils, DOM, XMLRead, XMLWrite, URIParser;

var
  cntr: Integer = 0;
  api: TXMLDocument;
  forced: Boolean = False;
  TestCount: Integer = 0;
  FailCount: Integer = 0;
<<<<<<< HEAD
<<<<<<< HEAD
  IgnoreCount: Integer = 0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function PascalType(const s: WideString): string;
begin
  if (s = 'DOMString') or (s = 'boolean') or (s = 'DOMError') or (s = 'double') then
    result := s
  else if s = 'int' then
    result := 'Integer'
  else if s = 'short' then
    result := 'SmallInt'
  else if s = 'Collection' then
    result := '_collection'
  else if s = 'List' then
    result := '_list'
<<<<<<< HEAD
<<<<<<< HEAD
  else if (Pos(WideString('DOM'), s) = 1) or (Pos(WideString('XPath'), s) = 1) or
          (Pos(WideString('HTML'), s) = 1) then
=======
  else if (Pos(WideString('DOM'), s) = 1) or (Pos(WideString('XPath'), s) = 1) then
>>>>>>> graemeg/fixes_2_2
=======
  else if (Pos(WideString('DOM'), s) = 1) or (Pos(WideString('XPath'), s) = 1) then
>>>>>>> origin/fixes_2_2
    result := 'T' + s
  else
    result := 'TDOM'+s;
end;

function ReplaceQuotes(const s: WideString): string;
var
  quoted: Boolean;
begin
  quoted := (s[1] = '"') and (s[Length(s)] = '"');
  if quoted then
    result := UTF8Encode(Copy(s, 2, Length(s)-2))
  else
    result := UTF8Encode(s);
    
  result := StringReplace(result, '\"', '"', [rfReplaceAll]);
  result := StringReplace(result, '''', '''''', [rfreplaceAll]);
  result := StringReplace(result, '\n', '''#10''', [rfReplaceAll]);
  result := StringReplace(result, '\\', '\', [rfreplaceAll]);

  if quoted then
    result := '''' + result + '''';
end;

procedure AppendParam(var s: string; n: TDOMElement; const attName: DOMString);
begin
  if n.HasAttribute(attName) then
    s := s + ReplaceQuotes(n[attName])
  else
<<<<<<< HEAD
<<<<<<< HEAD
    s := s + 'nil';
=======
    s := s + '''''';
>>>>>>> graemeg/fixes_2_2
=======
    s := s + '''''';
>>>>>>> origin/fixes_2_2
  s := s + ', ';
end;

function FirstElement(n: TDOMNode): TDOMElement;
var
  child: TDOMNode;
begin
  child := n.FirstChild;
  while Assigned(child) and (child.nodeType <> ELEMENT_NODE) do
    child := child.NextSibling;
  result := TDOMElement(child);
end;

procedure GetChildElements(el: TDOMNode; List: TList);
var
  child: TDOMNode;
begin
  List.Clear;
  child := el.FirstChild;
  while Assigned(child) do
  begin
    if child.NodeType = ELEMENT_NODE then
      List.Add(child);
    child := child.NextSibling;
  end;
end;

procedure DumpUnprocessed(e: TDOMElement; dest: TStrings);
var
  s: TStringStream;
begin
  s := TStringStream.Create('');
  try
    writeXML(e, s);
    dest.Text := dest.Text + '(*****' + s.DataString + sLineBreak + '*)' + sLineBreak;
  finally
    s.Free;
  end;
end;

function CondToStr(e: TDOMElement; out r: string): Boolean;
var
  tmp: string;
  child: TDOMNode;
begin
  Result := True;
  if e.TagName = 'equals' then
    r := e['actual'] + ' = ' + ReplaceQuotes(e['expected'])
  else if e.TagName = 'notEquals' then
    r := e['actual'] + ' <> ' + ReplaceQuotes(e['expected'])
  else if e.TagName = 'less' then
    r := e['actual'] + ' < ' + ReplaceQuotes(e['expected'])
  else if e.TagName = 'greater' then
    r := e['actual'] + ' > ' + ReplaceQuotes(e['expected'])
    
  // casting to Pointer works for both objects and strings
  else if e.TagName = 'isNull' then
    r := 'Pointer(' + e['obj'] + ') = nil'
  else if e.TagName = 'notNull' then
    r := 'Assigned(Pointer('+e['obj']+'))'
  else if e.TagName = 'isTrue' then
    r := e['value']
  else if (e.TagName = 'notTrue') or (e.TagName = 'isFalse') then
    r := 'not ' + e['value']
  else if e.TagName = 'contentType' then
    r := 'ContentTypeIs('''+e['type']+''')'
  else if e.TagName = 'implementationAttribute' then
  begin
    r := 'implementationAttribute[''' + e['name'] + '''] = ' + e['value'];
  end
  else if e.TagName = 'contains' then
  begin
    if e['interface'] = 'DOMString' then
      r := 'Pos(WideString(' +  replaceQuotes(e['str']) + '), ' + e['obj'] + ') > 0'
    else
      r := 'bad_condition(''contains intf=' + e['interface'] + ''')';
  end
  else if e.TagName = 'same' then
  begin
  // maybe it would be sufficient to just compare pointers, but let's emit a helper for now
    r := 'IsSame('+ e['expected'] + ', ' + e['actual'] + ')';
  end
  else if e.TagName = 'not' then
  begin
    child := e.FirstChild;
    while Assigned(child) do
    begin
      if child.nodeType = ELEMENT_NODE then
      begin
        if CondToStr(TDOMElement(child), tmp) then
          r := 'not ('+tmp+')';
        Break;
      end;
      child := child.NextSibling;
    end;
  end
  else if (e.TagName = 'and') or (e.TagName = 'or') then
  begin
    r := '';
    child := e.FirstChild;
    while Assigned(child) do
    begin
      if child.nodeType = ELEMENT_NODE then
      begin
        if CondToStr(TDOMElement(child), tmp) then
        begin
          if r <> '' then r := r + ' ' + e.TagName + ' ';
          r := r + '('+tmp+')';
        end;
      end;
      child := child.NextSibling;
    end;
  end
  else
  begin
    r := 'bad_condition(''' + e.TagName + ''')';
    Result := False;
  end;
end;

procedure ConvertTest(rootNode: TDOMElement; rslt: TStrings);
var
  child, subchild: TDOMNode;
  n: DOMString;
  SuccessVarFlag: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
  FailFlag, IgnoreFlag: Boolean;
=======
  FailFlag: Boolean;
>>>>>>> graemeg/fixes_2_2
=======
  FailFlag: Boolean;
>>>>>>> origin/fixes_2_2
  Inits, VarTypes: TStringList;

function TypeOfVar(const varname: string): string;
begin
  result := VarTypes.Values[varname];
end;

function IsCollection(node: TDOMElement): Boolean;
var
  s: string;
begin
  s := TypeOfVar(node['collection']);
  Result := (s = '_collection') or (s = '_list');
end;

procedure CastTo(node: TDOMElement; const typename: string);
begin
  if (not node.HasAttribute('interface')) and
    node.HasAttribute('obj') and
    (TypeOfVar(node['obj']) <> PascalType(typename)) then
  node['interface'] := typename;
end;

function getobj(e: TDOMElement): string;
var
  s: string;
begin
  result := e['obj'];
  if e.HasAttribute('interface') then
  begin
    s := PascalType(e['interface']);
    if TypeOfVar(e['obj']) <> s then
      result := s+'('+result+')';
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function fixname(e: TDOMElement): string;
begin
  if e.HasAttribute('_fixup_') then
    result := e['_fixup_']
  else  
    result := e.TagName;
end;

function argstring(e: TDOMElement; args: TDOMNodeList): string;
var
  I: Integer;
  argnode: TDOMElement;
begin
  Result := '';
  for I := 0 to args.Length-1 do
  begin
    argnode := args[I] as TDOMElement;
    Result := Result + ReplaceQuotes(e[argnode.TextContent]);
    if argnode.HasAttribute('type') then
      Result := Result + ' as ' + PascalType(argnode['type']);
    if I <> args.Length-1 then
      Result := Result + ', ';
  end;
end;

function prop_call(e: TDOMElement): string;
begin
  if e.HasAttribute('var') then
    Result := e['var'] + ' := ' + getobj(e) + '.' + fixname(e) + ';'
  else
    Result := getobj(e) + '.' + fixname(e) + ' := ' + ReplaceQuotes(e['value']) + ';';
end;

function func_call(e: TDOMElement; args: TDOMNodeList; const rsltType: string=''; IsDefProp: Boolean=False): string;
=======
=======
>>>>>>> origin/fixes_2_2
function prop_call(e: TDOMElement): string;
begin
  if e.HasAttribute('var') then
    Result := e['var'] + ' := ' + getobj(e) + '.' + e.TagName + ';'
  else
    Result := getobj(e) + '.' + e.TagName + ' := ' + ReplaceQuotes(e['value']) + ';';
end;

function func_call(e: TDOMElement; const args: array of DOMString; const rsltType: string=''): string;
var
  I: Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  if (rsltType <> '') and (TypeOfVar(e['var']) <> rsltType) then
    Result := rsltType + '(' + e['var'] + ')'
  else
    Result := e['var'];
<<<<<<< HEAD
<<<<<<< HEAD
  if IsDefProp then
    Result := Result + ' := ' + getobj(e) + '[' + argstring(e, args) + ']'
  else
  begin
    Result := Result + ' := ' + getobj(e) + '.' + fixname(e);
    if args.Length > 0 then
      Result := Result + '(' + argstring(e, args) + ')';
  end;
  Result := Result + ';';
end;

function func_call(e: TDOMElement; const args: array of DOMString): string;
var
  I: Integer;
begin
  Result := e['var'] + ' := ' + getobj(e) + '.' + e.TagName;
=======
  Result := Result + ' := ' + getobj(e) + '.' + e.TagName;
>>>>>>> graemeg/fixes_2_2
=======
  Result := Result + ' := ' + getobj(e) + '.' + e.TagName;
>>>>>>> origin/fixes_2_2
  if Length(args) > 0 then
  begin
    Result := Result + '(';
    for I := 0 to High(args) do
    begin
      Result := Result + ReplaceQuotes(e[args[I]]);
      if I <> High(args) then
        Result := Result + ', ';
    end;
    Result := Result + ')';
  end;
  Result := Result + ';';
end;

function method_call(e: TDOMElement; args: TDOMNodeList): string;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result := getobj(e) + '.' + fixname(e);
  if args.Length > 0 then
    Result := Result + '(' + argstring(e, args) + ')';
=======
=======
>>>>>>> origin/fixes_2_2
var
  I: Integer;
begin
  Result := getobj(e) + '.' + e.TagName;
  if args.Length > 0 then
  begin
    Result := Result + '(';
    for I := 0 to args.Length-1 do
    begin
      Result := Result + ReplaceQuotes(e[args[I].TextContent]);
      if I <> args.Length-1 then
        Result := Result + ', ';
    end;
    Result := Result + ')';
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result := Result + ';';
end;

procedure FixKeywords(node: TDOMElement; const AttrName: DOMString);
var
  v: DOMString;
begin
  v := node[AttrName];
  if v = 'testName' then              // clash with TTest.TestName property
    node[AttrName] := 'test_Name'
  else if v = 'implementation' then
    node[AttrName] := 'DOMImpl'
  else if v = 'type' then
    node[AttrName] := 'type_';
end;

procedure ConvertStatement(node: TDOMElement; const indent: string);
var
  s: DOMString;
  cond: string;
  apinode: TDOMElement;
  arglist: TDOMNodeList;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  args: array of DOMString;
  I: Integer;
>>>>>>> graemeg/fixes_2_2
=======
  args: array of DOMString;
  I: Integer;
>>>>>>> origin/fixes_2_2
begin
  FixKeywords(node, 'var');
  FixKeywords(node, 'obj');

  s := node.TagName;
  apinode := api.GetElementById(s);
  // If not found by name only, try prepending the interface name.
  // This enables support of same-named methods with different param lists on different objects
  if (apinode = nil) and node.HasAttribute('interface') then
    apinode := api.GetElementById(node['interface'] + '.' + s);
  if assigned(apinode) then
  begin
    // handle most of DOM API in consistent way
<<<<<<< HEAD
<<<<<<< HEAD
    
    if apinode.HasAttribute('rename') then   // handles reserved words, e.g 'type' -> 'htmlType'
      node['_fixup_'] := apinode['rename'];  // use this trick because DOM node cannot be renamed (yet)
    
    arglist := apinode.GetElementsByTagName('arg');

    if apinode.HasAttribute('objtype') then
      CastTo(node, apinode['objtype']);
      
=======
=======
>>>>>>> origin/fixes_2_2
    arglist := apinode.GetElementsByTagName('arg');
    SetLength(args, arglist.Length);
    for I := 0 to arglist.Length-1 do
      args[I] := arglist[I].TextContent;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    if apinode['type'] = 'prop' then
      rslt.Add(indent + prop_call(node))
    else if apinode['type'] = 'method' then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
      if apinode.HasAttribute('objtype') then
        CastTo(node, apinode['objtype']);
>>>>>>> graemeg/fixes_2_2
=======
      if apinode.HasAttribute('objtype') then
        CastTo(node, apinode['objtype']);
>>>>>>> origin/fixes_2_2
      rslt.Add(indent + method_call(node, arglist));
    end
    else
    begin
      if apinode.HasAttribute('result') then
        cond := PascalType(apinode['result'])
      else
        cond := '';
<<<<<<< HEAD
<<<<<<< HEAD
      rslt.Add(indent + func_call(node, arglist, cond, apinode['type']='defprop'));
=======
      if apinode.HasAttribute('objtype') then
        CastTo(node, apinode['objtype']);
      rslt.Add(indent + func_call(node, args, cond));
>>>>>>> graemeg/fixes_2_2
=======
      if apinode.HasAttribute('objtype') then
        CastTo(node, apinode['objtype']);
      rslt.Add(indent + func_call(node, args, cond));
>>>>>>> origin/fixes_2_2
      if apinode['gc'] = 'yes' then
        rslt.Add(indent + 'GC(' + node['var'] + ');');
    end;
    Exit;
  end;

  // now, various hacks and workarounds
<<<<<<< HEAD
<<<<<<< HEAD
  if s = 'length' then
=======
=======
>>>>>>> origin/fixes_2_2

  // TODO: modify DOM to expose item() as function
  if s = 'item' then
    rslt.Add(indent + 'TDOMNode('+node['var'] + ') := ' + node['obj'] + '['+node['index']+'];')
  else if s = 'length' then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  begin
    if node['interface'] = 'DOMString' then
      rslt.Add(indent + node['var'] + ' := system.length(' + node['obj'] + ');')
    else
      rslt.Add(indent + func_call(node, []));
  end
  else if s = 'implementation' then
  begin
    if node.HasAttribute('obj') then
      rslt.Add(indent + node['var'] + ' := ' + node['obj'] + '.impl;')
    else
      rslt.Add(indent + node['var'] + ' := GetImplementation;');
  end
  else if s = 'hasFeature' then
  begin
    if node.hasAttribute('var') then
    begin
      // we don't have null strings, replace with an empty one
      if not node.hasAttribute('version') then
        node['version'] := '""';
      rslt.Add(indent + func_call(node, ['feature', 'version']))
    end
    else
      rslt.Add(indent + 'CheckFeature(' + ReplaceQuotes(node['feature']) + ');')
  end
  
  // service (non-DOM) statements follow
  
  else if s = 'append' then
    rslt.Add(indent + '_append(' + node['collection'] + ', ' + ReplaceQuotes(node['item']) + ');')
  else if s = 'assign' then
  begin
    cond := TypeOfVar(node['var']);
    if (cond = '_collection') or (cond = '_list') then
      rslt.Add(indent + '_assign(' + node['var'] + ', ' + node['value'] + ');')
    else // emit an assignment operator. Force type for the case where they assign Document to Element.
      rslt.Add(indent + node['var'] + ' := ' + TypeOfVar(node['var']) + '(' + ReplaceQuotes(node['value']) + ');');
  end  
  else if s = 'increment' then
    rslt.Add(indent + 'Inc(' + node['var'] + ', ' + node['value'] + ');')
  else if s = 'decrement' then
    rslt.Add(indent + 'Dec(' + node['var'] + ', ' + node['value'] + ');')
  else if s = 'plus' then
    rslt.Add(indent + node['var'] + ' := ' + ReplaceQuotes(node['op1']) + ' + ' + ReplaceQuotes(node['op2']) + ';')

  else if s = 'fail' then
    rslt.Add(indent + s + '(''' + node['id'] + ''');')
  else if s = 'assertEquals' then
  begin
    cond := TypeOfVar(node['actual']);
    if cond = '_collection' then
      rslt.Add(indent + 'AssertEqualsCollection(''' + node['id'] + ''', ' + ReplaceQuotes(node['expected']) + ', ' + node['actual'] + ');')
    else if cond = '_list' then
      rslt.Add(indent + 'AssertEqualsList(''' + node['id'] + ''', ' + ReplaceQuotes(node['expected']) + ', ' + node['actual'] + ');')
<<<<<<< HEAD
<<<<<<< HEAD
    else if cond = 'DOMString' then
      rslt.Add(indent + 'AssertEqualsW(''' + node['id'] + ''', ' + ReplaceQuotes(node['expected']) + ', ' + node['actual'] + ');')
    else if node['ignoreCase'] = 'true' then
      rslt.Add(indent + 'AssertEqualsNoCase(''' + node['id'] + ''', ' + ReplaceQuotes(node['expected']) + ', ' + node['actual'] + ');')
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    else
      rslt.Add(indent + s + '(''' + node['id'] + ''', ' + ReplaceQuotes(node['expected']) + ', ' + node['actual'] + ');');
  end
  else if s = 'assertSame' then
    rslt.Add(indent + s + '(''' + node['id'] + ''', ' + ReplaceQuotes(node['expected']) + ', ' + node['actual'] + ');')
  else if (s = 'assertNull') or (s = 'assertNotNull') {or (s='assertFalse')} then
    rslt.Add(indent + s + '(''' + node['id'] + ''', ' + node['actual'] + ');')
  else if s = 'assertSize' then
    rslt.Add(indent + s + '(''' + node['id'] + ''', ' + node['size'] + ', ' + node['collection']+');')
  else if s = 'assertInstanceOf' then
    rslt.Add(indent + s + '(''' + node['id'] + ''', ' + node['obj'] + ', ''' + PascalType(node['type'])+''');')
  else if (s = 'assertTrue') or (s='assertFalse') then
    if node.HasChildNodes then
    begin
      child := FirstElement(node);
      CondToStr(TDOMElement(child), cond);
      rslt.Add(indent + s + '(''' + node['id'] + ''', ' + cond + ');');
    end
    else
      rslt.Add(indent + s + '(''' + node['id'] + ''', ' + node['actual'] + ');')
  else if s = 'assertURIEquals' then
  begin
    // TODO: maybe add 'flags' argument to specify which strings are non-NULL
    cond := '''' + node['id'] + ''', ';
    AppendParam(cond, node, 'scheme');
    AppendParam(cond, node, 'path');
    AppendParam(cond, node, 'host');
    AppendParam(cond, node, 'file');
    AppendParam(cond, node, 'name');
    AppendParam(cond, node, 'query');
    AppendParam(cond, node, 'fragment');

    if node.HasAttribute('isAbsolute') then
      cond := cond + node['isAbsolute']
    else
      cond := cond + 'False';
    cond := cond + ', ';

    cond := cond + node['actual'];
    rslt.Add(indent + s + '(' + cond + ');');
  end
  else if n = 'load' then
    rslt.Add(indent + 'Load('+node['var']+', '''+ node['href']+''');')
  else if s = 'implementationAttribute' then
<<<<<<< HEAD
<<<<<<< HEAD
  begin
    if (node['name']='signed') and (node['value']='true') then
      IgnoreFlag := True;
    rslt.Add(indent + s + '[''' + node['name'] + '''] := ' + node['value'] + ';')
  end
=======
    rslt.Add(indent + s + '[''' + node['name'] + '''] := ' + node['value'] + ';')
>>>>>>> graemeg/fixes_2_2
=======
    rslt.Add(indent + s + '[''' + node['name'] + '''] := ' + node['value'] + ';')
>>>>>>> origin/fixes_2_2
  else if s = 'createXPathEvaluator' then
    rslt.Add(indent + node['var'] + ' := CreateXPathEvaluator(' + node['document'] + ');')
  else if s = 'comment' then
    rslt.Add(indent + '{ Source comment: ' + node.TextContent + ' }')
  else
  begin
    if not FailFlag then
      rslt.Add(indent + 'Fail(''This test is not completely converted'');');
    FailFlag := True;
    DumpUnprocessed(node, rslt);
  end;
end;

procedure ConvertException(el: TDOMElement; const ExceptClass: string; indent: string);
var
  excode: string;
begin
  if not SuccessVarFlag then
    rslt.Insert(2, '  success: Boolean;');
  SuccessVarFlag := True;
  rslt.Add(indent+'success := False;');
  rslt.Add(indent+'try');
  child := el.FirstChild;
  while assigned(child) do
  begin
    if child.nodeType = ELEMENT_NODE then
    begin
      excode := child.nodeName;
      subchild := child.FirstChild;
      while Assigned(subchild) do
      begin
        if subchild.nodeType = ELEMENT_NODE then
          ConvertStatement(TDOMElement(subchild), indent + '  ');
        subchild := subchild.NextSibling;
      end;
    end;
    child := child.NextSibling;
  end;
  rslt.Add(indent+'except');
  rslt.Add(indent+'  on E: Exception do');
  rslt.Add(indent+'    success := (E is ' + ExceptClass +') and (' + ExceptClass + '(E).Code = ' + excode + ');');
  rslt.Add(indent+'end;');
  rslt.Add(indent+'AssertTrue('''+el['id']+''', success);');
end;

procedure ConvertBlock(el: TDOMNode; indent: string);
var
  curr: TDOMNode;
  element: TDOMElement;
  List: TList;
  cond: string;
  Frag: TDOMDocumentFragment;
  I: Integer;
  ElseNode: TDOMNode;
  IsColl: Boolean;
begin
  List := TList.Create;
  curr := el.FirstChild;
  indent := indent + '  ';
  while Assigned(curr) do
  begin
    if (curr.NodeType <> ELEMENT_NODE) or
      (curr.NodeName = 'var') or (curr.NodeName = 'metadata') then
    begin
      curr := curr.NextSibling;
      Continue;
    end;
    element := TDOMElement(curr);
    n := element.TagName;
    if n = 'assertDOMException' then
      ConvertException(element, 'EDOMError', indent)
    else if n = 'assertXPathException' then
      ConvertException(element, 'EXPathException', indent)
    else if n = 'try' then
    begin
      GetChildElements(curr, List);
      rslt.Add(indent+'try');
      I := 0;
      while I < List.Count do
      begin
        Child := TDOMNode(List[I]);
        if Child.NodeName = 'catch' then
          break;
        ConvertStatement(TDOMElement(child), indent + '  ');
        Inc(I);
      end;
      if (child.NodeName <> 'catch') or (Pointer(Child) <> List.Last) then
        rslt.Add('{ ERROR: misplaced "catch" tag }');
      GetChildElements(child, List);
      cond := '';
      for I := 0 to List.Count-1 do
      begin
        if TDOMElement(List[I]).TagName <> 'DOMException' then
        begin
          rslt.Add('{ ERROR: unhandled: ' + TDOMElement(List[I]).TagName +' }');
          Break;
        end;
        if cond <> '' then cond := cond + ', ';
        cond := cond + TDOMElement(List[I])['code'];
      end;
      
      rslt.Add(indent+'except');
      rslt.Add(indent+'  on E: EDOMError do');
      rslt.Add(indent+'    if not (E.code in ['+cond+']) then raise;');
      rslt.Add(indent+'end;');
    end
    else if n = 'if' then
    begin
      ElseNode := nil;
      GetChildElements(curr, List);
      if (List.Count > 1) and CondToStr(TDOMElement(List[0]), cond) then
      begin
        rslt.Add(indent+ 'if '+cond+' then');
        frag := curr.OwnerDocument.CreateDocumentFragment;
        try
          // first node is the condition; skip it
          for I := 1 to List.Count-1 do
          begin
            child := TDOMNode(List[I]);
            if child.NodeName = 'else' then
            begin
              ElseNode := child;
              Break;
            end;
            frag.AppendChild(child.CloneNode(True));
          end;
          rslt.add(indent+'begin');
          ConvertBlock(frag, indent);
          if Assigned(ElseNode) then
          begin
            rslt.add(indent+'end');
            rslt.Add(indent+'else');
            rslt.Add(indent+'begin');
            ConvertBlock(ElseNode, indent);
          end;
          rslt.add(indent+'end;');
        finally
          frag.Free;
        end;
      end
      else
      begin
        rslt.Add('{ ERROR: malformed "if" tag }');
        dumpunprocessed(element, rslt);
      end;
    end
    else if n = 'for-each' then
    begin
      // having loop var name globally unique isn't a must.
      cond := 'loop'+IntToStr(cntr);
      Inc(cntr);
<<<<<<< HEAD
<<<<<<< HEAD
      rslt.Insert(rslt.IndexOf('var')+1, '  ' + cond + ': Integer;');
=======
      rslt.Insert(2, '  ' + cond + ': Integer;');
>>>>>>> graemeg/fixes_2_2
=======
      rslt.Insert(2, '  ' + cond + ': Integer;');
>>>>>>> origin/fixes_2_2
      IsColl := IsCollection(element);
      if IsColl then
        rslt.Add(indent+'for '+cond+' := 0 to ' + 'High(' + element['collection'] + ') do')
      else
        rslt.Add(indent+'for '+cond+' := 0 to ' + element['collection'] + '.Length-1 do');
      rslt.Add(indent+'begin');
      if IsColl then
        rslt.Add(indent+'  ' + element['member'] + ' := '+element['collection']+'['+cond+'];')
      else
        rslt.Add(indent+'  ' + 'TDOMNode('+element['member'] + ') := '+element['collection']+'['+cond+'];');
      ConvertBlock(element, indent);
      rslt.Add(indent+'end;');
    end
    else if n = 'while' then
    begin
      GetChildElements(curr, List);
      if (List.Count > 1) and CondToStr(TDOMElement(List[0]), cond) then
      begin
        rslt.Add(indent+ 'while '+cond+' do');
        frag := curr.OwnerDocument.CreateDocumentFragment;
        try
          for I := 1 to List.Count-1 do  // skip first node which is the condition
          begin
            child := TDOMNode(List[I]);
            frag.AppendChild(child.CloneNode(True));
          end;
          rslt.add(indent+'begin');
          ConvertBlock(frag, indent);
          rslt.add(indent+'end;');
        finally
          frag.Free;
        end;
      end
      else
      begin
        rslt.Add('{ ERROR: malformed "while" tag }');
        DumpUnprocessed(element, rslt);
      end;
    end
    else
      ConvertStatement(element, indent);
    curr := curr.NextSibling;
  end;
  List.Free;
end;

procedure ConvertVars;
var
  TypedConsts: TStrings;
  I, J: Integer;
  vars, subvars: TDOMNodeList;
  node: TDOMElement;
  hs: string;
begin
  TypedConsts := TStringList.Create;
  vars := rootNode.GetElementsByTagName('var');
  if vars.Count > 0 then
  begin
    rslt.Add('var');
    for I := 0 to vars.Count-1 do
    begin
      node := TDOMElement(vars[I]);
      FixKeywords(node, 'name');
      if node.hasAttribute('isNull') or node.hasAttribute('value') then
      begin
        // TODO: isNull is identified by 'yes' value, not by mere attr presence?
        // TODO: consider putting isNull things to constants
        if node.hasAttribute('value') then
          hs := ReplaceQuotes(Node['value'])
        else
        begin
          if node['type'] = 'DOMString' then
            hs := ''''''
          else
            hs := 'nil';
        end;
        Inits.Add('  ' + node['name'] + ' := ' + hs + ';');
      end;
      if Node.HasChildNodes then
      begin
        subvars := Node.GetElementsByTagName('member');
        try
          if subvars.Count > 0 then
          begin
            if TDOMElement(subvars[0]).HasAttribute('type') then
              hs := PascalType(TDOMElement(subvars[0]).GetAttribute('type'))
            else
              hs := 'DOMString';
            TypedConsts.Add('  ' + Node['name'] + ': array[0..' + IntToStr(subvars.Count-1) + '] of ' + hs + ' = (');
            for J := 0 to subvars.Count-1 do
            begin
              hs := '    ' + ReplaceQuotes(subvars[J].TextContent);
              if J = subvars.Count-1 then
                TypedConsts.Add(hs + ');')
              else
                TypedConsts.Add(hs + ',');
            end;
          end
          else
            DumpUnprocessed(Node, rslt);
        finally
          subvars.Free;
        end;
      end
      else
        rslt.Add('  ' + Node['name'] +': '+ PascalType(Node['type'])+';');
      VarTypes.Add(Node['name'] + '=' + PascalType(Node['type']));
    end;
    if TypedConsts.Count > 0 then
    begin
      rslt.add('const');
      rslt.AddStrings(TypedConsts);
    end;
  end;
  vars.Free;
  TypedConsts.Free;
end;

// ConvertTest() itself
begin
  SuccessVarFlag := False;
  FailFlag := False;
<<<<<<< HEAD
<<<<<<< HEAD
  IgnoreFlag := False;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  VarTypes := TStringList.Create;
  Inits := TStringList.Create;
  ConvertVars;
  rslt.add('begin');
  rslt.AddStrings(Inits);
  Inits.Free;
  ConvertBlock(rootNode, '');
  VarTypes.Free;
  rslt.add('end;');
  rslt.Add('');
  
  if FailFlag then
  begin
    if not forced then
      rslt.Clear;
    Inc(FailCount);
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  if IgnoreFlag then
  begin
    rslt.Clear;
    Inc(IgnoreCount);
  end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

// Intercepting validation errors while loading API
type
  TErrHandler = class(TObject)
  public
    procedure HandleError(E: EXMLReadError);
  end;

procedure TErrHandler.HandleError(E: EXMLReadError);
begin
  raise E;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function IsBlacklisted(const s: string; const list: array of string): Boolean;
var
  I: Integer;
begin
  Result := True;
  for I := Low(list) to High(list) do
  begin
    if s = list[I] then
      Exit;
  end;
  Result := False;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
  UnitHeader =

'{ AUTOGENERATED FILE - DO NOT EDIT'#10+
'  This Pascal source file was generated by testgen program'#10 +
'  and is a derived work from the source document.'#10 +
'  The source document contained the following notice:'#10+
<<<<<<< HEAD
<<<<<<< HEAD
'%0:s}'#10+
'unit %1:s;'#10 +
=======
'%s}'#10+
'unit %s;'#10 +
>>>>>>> graemeg/fixes_2_2
=======
'%s}'#10+
'unit %s;'#10 +
>>>>>>> origin/fixes_2_2
'{$mode objfpc}{$h+}'#10 +
'{$notes off}'#10 +
'{$codepage utf8}'#10 +
'interface'#10 +
#10 +
'uses'#10 +
<<<<<<< HEAD
<<<<<<< HEAD
'  SysUtils, Classes, DOM, xmlread, fpcunit, contnrs, domunit, testregistry%3:s;'#10 +
#10 +
'type'#10 +
'  %2:s = class(TDOMTestBase)'#10 +
=======
=======
>>>>>>> origin/fixes_2_2
'  SysUtils, Classes, DOM, xmlread, fpcunit, contnrs, domunit, testregistry;'#10 +
#10 +
'type'#10 +
'  %s = class(TDOMTestBase)'#10 +
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
'  protected'#10 +
'    function GetTestFilesURI: string; override;'#10 +
'  published'#10;

procedure ConvertSuite(const BaseURI: DOMString; const UnitFileName: string);
var
  suite, testdoc: TXMLDocument;
  testlist: TDOMNodeList;
  root: TDOMElement;
  href, testuri: DOMString;
  I: Integer;
  sl, all, impl: TStringList;
  Pars: TDOMParser;
  eh: TErrHandler;
<<<<<<< HEAD
<<<<<<< HEAD
  class_name, unit_name, notice, casename, add_units: string;
  comment: TDOMNode;
  blacklist: array of string;
=======
  class_name, unit_name, notice: string;
  comment: TDOMNode;
>>>>>>> graemeg/fixes_2_2
=======
  class_name, unit_name, notice: string;
  comment: TDOMNode;
>>>>>>> origin/fixes_2_2
begin
  Pars := TDOMParser.Create;
  eh := TErrHandler.Create;
  Pars.Options.Validate := True;
  Pars.OnError := @eh.HandleError;
  // API database must be loaded in validating mode
  Pars.ParseURI('file:api.xml', api);

<<<<<<< HEAD
<<<<<<< HEAD
  // Prepare the array of blacklisted test names
  testlist := api.GetElementsByTagName('blacklist');
  try
    SetLength(blacklist, testlist.length);
    for I := 0 to testlist.length-1 do
      blacklist[I] := testlist[I].TextContent;
  finally
    testlist.Free;
  end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  sl := TStringList.Create;
  all := TStringList.Create;
  impl := TStringList.Create;

  Pars.OnError := nil;
  Pars.Options.ExpandEntities := True;
  Pars.ParseURI(BaseURI + 'alltests.xml', suite);
  // extract the copyright notice
  notice := '';
  comment := suite.FirstChild;
  while Assigned(comment) do
  begin
    if (comment.nodeType = COMMENT_NODE) and
      (Pos(DOMString('Copyright'), comment.nodeValue) > 0) then
    begin
      notice := comment.nodeValue;
      Break;
    end;
    comment := comment.nextSibling;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  // Check if we need the additional units to use
  add_units := '';
  testlist := api.GetElementsByTagName('uses');
  try
    for I := 0 to testlist.Length-1 do
    begin
      root := TDOMElement(testlist[I]);
      if Pos(root['pattern'], BaseURI) <> 0 then
        add_units := add_units + ', ' + root['unit'];
    end;
  finally
    testlist.Free;
  end;

  unit_name := ChangeFileExt(ExtractFileName(UnitFileName), '');
  class_name := 'TTest' + UpperCase(unit_name[1]) + copy(unit_name, 2, MaxInt);
  // provide unit header
  all.Text := Format(UnitHeader, [notice, unit_name, class_name, add_units]);
=======
=======
>>>>>>> origin/fixes_2_2
  unit_name := ChangeFileExt(ExtractFileName(UnitFileName), '');
  class_name := 'TTest' + UpperCase(unit_name[1]) + copy(unit_name, 2, MaxInt);
  // provide unit header
  all.Text := Format(UnitHeader, [notice, unit_name, class_name]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // emit the 'GetPathToModuleFiles' function body
  impl.Add('implementation');
  impl.Add('');
  impl.Add('function '+class_name+'.GetTestFilesURI: string;');
  impl.Add('begin');
  impl.Add('  result := ''' + BaseURI + ''';');
  impl.Add('end;');
  impl.Add('');
  
  testlist := suite.GetElementsByTagName('suite.member');
  testcount := testlist.Count;
  writeln;
  writeln(testcount, ' test cases found');
  for I := 0 to testcount-1 do
  begin
    href := TDOMElement(testlist[I])['href'];
<<<<<<< HEAD
<<<<<<< HEAD
    ResolveRelativeURI(BaseURI, href, testuri);

=======
    // simple concatenation should suffice, but be paranoid
    ResolveRelativeURI(BaseURI, href, testuri);
>>>>>>> graemeg/fixes_2_2
=======
    // simple concatenation should suffice, but be paranoid
    ResolveRelativeURI(BaseURI, href, testuri);
>>>>>>> origin/fixes_2_2
    Pars.ParseURI(testuri, testdoc);
    try
      sl.Clear;
      root := testdoc.DocumentElement;
      // fix clash with local vars having the same name
<<<<<<< HEAD
<<<<<<< HEAD
      casename := root['name'];
      if casename = 'attrname' then
        casename := 'attr_name';
      if IsBlacklisted(casename, blacklist) then
      begin
        writeln('Test case "', casename, '" is blacklisted, skipping');
        Continue;
      end;
      sl.Add('procedure ' + class_name + '.' + casename + ';');
      try
        ConvertTest(root, sl);
      except
        Writeln('An exception occured while converting ', casename);
=======
=======
>>>>>>> origin/fixes_2_2
      if root['name'] = 'attrname' then
        root['name'] := 'attr_name';
      sl.Add('procedure ' + class_name + '.' + root['name'] + ';');
      try
      ConvertTest(root, sl);
      except
        Writeln('An exception occured while converting '+root['name']);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        raise;
      end;
      if sl.Count > 0 then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        all.add('    procedure '+casename+';');
=======
        all.add('    procedure '+root['name']+';');
>>>>>>> graemeg/fixes_2_2
=======
        all.add('    procedure '+root['name']+';');
>>>>>>> origin/fixes_2_2
        impl.AddStrings(sl)
      end;
    finally
      testdoc.Free;
    end;
  end;
  testlist.Free;
  suite.Free;

  // terminate class declaration
  all.Add('  end;');
  all.Add('');
  // append all procedure bodies
  all.AddStrings(impl);

  all.Add('initialization');
  all.Add('  RegisterTest('+class_name+');');
  all.Add('end.');
  all.SaveToFile(UnitFileName);
  impl.Free;
  all.Free;
  sl.Free;
  eh.Free;
  Pars.Free;
end;

var
  SuiteName: string;
  OutputUnit: string;
  s: string;
  I: Integer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  writeln('testgen - w3.org DOM test suite to Object Pascal converter');
=======
  writeln('testgen - w3.org DOM test suite to Pascal converter');
>>>>>>> graemeg/fixes_2_2
=======
  writeln('testgen - w3.org DOM test suite to Pascal converter');
>>>>>>> origin/fixes_2_2
  writeln('Copyright (c) 2008 by Sergei Gorelkin');
  
  if ParamCount < 2 then
  begin
    writeln;
<<<<<<< HEAD
<<<<<<< HEAD
    writeln('Usage: ', ExtractFileName(ParamStr(0)), ' <suite dir> <outputunit.pp> [-f]');
=======
    writeln('Usage: ', ParamStr(0), ' <suite dir> <outputunit.pp> [-f]');
>>>>>>> graemeg/fixes_2_2
=======
    writeln('Usage: ', ParamStr(0), ' <suite dir> <outputunit.pp> [-f]');
>>>>>>> origin/fixes_2_2
    writeln('  -f: force conversion of tests which contain unknown tags');
    Exit;
  end;

  SuiteName := ExpandFilename(ParamStr(1));
  OutputUnit := ExpandFilename(ParamStr(2));
  i := 3;
  while i <= ParamCount do
  begin
    s := Lowercase(ParamStr(i));
    if s = '-f' then
      forced := True;
    Inc(i);
  end;
  // strip filename if present, we're going to read all dir
  if not DirectoryExists(SuiteName) then
    SuiteName := ExtractFilePath(SuiteName)
  else
    SuiteName := IncludeTrailingPathDelimiter(SuiteName);

  ConvertSuite(FilenameToURI(SuiteName), OutputUnit);

<<<<<<< HEAD
<<<<<<< HEAD
  writeln(testcount - FailCount - IgnoreCount, ' tests converted successfully');
=======
  writeln(testcount - FailCount, ' tests converted successfully');
>>>>>>> graemeg/fixes_2_2
=======
  writeln(testcount - FailCount, ' tests converted successfully');
>>>>>>> origin/fixes_2_2
  if FailCount > 0 then
  begin
    writeln(FailCount, ' tests contain tags that are not supported yet');
    if forced then
    begin
      writeln('Conversion of these tests was forced,');
      writeln('the resulting file may not compile!');
    end
    else
      writeln('These tests were skipped');
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  if IgnoreCount > 0 then
  begin
    writeln(IgnoreCount, ' tests were skipped because they are not');
    writeln('   applicable to our DOM implementation.');
  end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.

