{**********************************************************************

    This file is part of the Free Component Library (FCL)

    Some DOM test cases adapted by hand (because automatic conversion
    is not yet possible for them).
    Copyright (c) 2001-2004 World Wide Web Consortium,
    (Massachusetts Institute of Technology, Institut National de
    Recherche en Informatique et en Automatique, Keio University). All
    Rights Reserved.
    Copyright (c) 2009 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit extras2;
{$mode objfpc}{$H+}
interface

uses
  SysUtils, Classes, DOM, xmlread, xmlwrite, domunit, testregistry;

implementation

type
  TDOMTestExtra2 = class(TDOMTestBase)
  published
    procedure ls3_canonicform08;
    procedure ls3_canonicform09;
    procedure ls3_canonicform10;
    procedure ls3_canonicform11;
    procedure ls3_DomWriterTest5;
    procedure ls3_DomWriterTest6;
  end;

const
// This is example #1 from c14n specs, but modified to comply with HTML grammar
  canonicform01 =
'<?xml version="1.0"?>'^M^J+
^M^J+
'<?xml-stylesheet   href="doc.xsl"'^M^J+
'   type="text/xsl"   ?>'^M^J+
^M^J+
'<!DOCTYPE html SYSTEM "xhtml1-strict.dtd">'^M^J+
'<html xmlns="http://www.w3.org/1999/xhtml"><head><title>canonicalform01</title></head><body onload="parent.loadComplete()">'^M^J+
'<p>Hello, world!<!-- Comment 1 --></p></body></html>'^M^J+
^M^J+
'<?pi-without-data     ?>'^M^J+
^M^J+
'<!-- Comment 2 -->'^M^J+
^M^J+
'<!-- Comment 3 -->'^M^J;

  canonicform03 =
'<!DOCTYPE html [<!ATTLIST acronym title CDATA "default">]>'^M^J+
'<html xmlns="http://www.w3.org/1999/xhtml"><head><title>canonicalform03</title></head><body onload="parent.loadComplete()">'^M^J+
'   <br   />'^M^J+
'   <br   ></br>'^M^J+
'   <div   name = "elem3"   id="elem3"   />'^M^J+
'   <div   name="elem4"   id="elem4"   ></div>'^M^J+
'   <div a:attr="out" b:attr="sorted" name="all" class="I''m"'^M^J+
'      xmlns:b="http://www.ietf.org"'^M^J+
'      xmlns:a="http://www.w3.org"'^M^J+
'      xmlns="http://example.org"/>'^M^J+
'   <div xmlns="" xmlns:a="http://www.w3.org">'^M^J+
'      <div xmlns="http://www.ietf.org">'^M^J+
'         <div xmlns="" xmlns:a="http://www.w3.org">'^M^J+
'            <acronym xmlns="" xmlns:a="http://www.ietf.org"/>'^M^J+
'         </div>'^M^J+
'      </div>'^M^J+
'   </div>'^M^J+
'</body></html>'^M^J;

{ TDOMTestExtra }

{ test canonical form with comments }
procedure TDOMTestExtra2.ls3_canonicform08;
var
  doc: TDOMDocument;
  node: TDOMNode;
  nodeType: Integer;
  nodeValue: DOMString;
  length: Integer;
begin
<<<<<<< HEAD
  FParser.Options.CanonicalForm := True;
=======
// canonical form: PreserveWhitespace, Namespaces, NamespaceDeclarations = True;
//                 Entities, CDSections = False;
  FParser.Options.PreserveWhitespace := True;
  FParser.Options.Namespaces := True;
>>>>>>> origin/fixes_2.4
  LoadStringData(doc, canonicform01);
  begin
    node := TDOMNode(doc).firstChild;
    nodeType := node.nodeType;
    assertEquals('PIisFirstChild', 7, nodeType);
    nodeValue := TDOMProcessingInstruction(node).data;
    length := system.length(nodeValue);
    assertEquals('piDataLength', 36, length);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('TextisSecondChild', 3, nodeType);
    nodeValue := node.nodeValue;
    length := system.length(nodeValue);
    assertEquals('secondChildLength', 1, length);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('ElementisThirdChild', 1, nodeType);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('TextisFourthChild', 3, nodeType);
    nodeValue := node.nodeValue;
    length := system.length(nodeValue);
    assertEquals('fourthChildLength', 1, length);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('PIisFifthChild', 7, nodeType);
    nodeValue := TDOMProcessingInstruction(node).data;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    assertEqualsW('trailingPIData', '', nodeValue);
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> graemeg/cpstrnew
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> graemeg/cpstrnew
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> graemeg/cpstrnew
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> origin/cpstrnew
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> origin/fixes_2.4
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('TextisSixthChild', 3, nodeType);
    nodeValue := node.nodeValue;
    length := system.length(nodeValue);
    assertEquals('sixthChildLength', 1, length);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('CommentisSeventhChild', 8, nodeType);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('TextisEighthChild', 3, nodeType);
    nodeValue := node.nodeValue;
    length := system.length(nodeValue);
    assertEquals('eighthChildLength', 1, length);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('CommentisNinthChild', 8, nodeType);
    node := node.nextSibling;
    assertNull('TenthIsNull', node);
  end;
end;

{ test canonical form without comments }
procedure TDOMTestExtra2.ls3_canonicform09;
var
  doc: TDOMDocument;
  node: TDOMNode;
  nodeType: Integer;
  nodeValue: DOMString;
  length: Integer;
begin
<<<<<<< HEAD
  FParser.Options.CanonicalForm := True;
=======
// canonical form: PreserveWhitespace, Namespaces, NamespaceDeclarations = True;
//                 Entities, CDSections = False;
  FParser.Options.PreserveWhitespace := True;
  FParser.Options.Namespaces := True;
>>>>>>> origin/fixes_2.4
  FParser.Options.IgnoreComments := True;
  LoadStringData(doc, canonicform01);
  begin
    node := TDOMNode(doc).firstChild;
    nodeType := node.nodeType;
    assertEquals('PIisFirstChild', 7, nodeType);
    nodeValue := TDOMProcessingInstruction(node).data;
    length := system.length(nodeValue);
    assertEquals('piDataLength', 36, length);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('TextisSecondChild', 3, nodeType);
    nodeValue := node.nodeValue;
    length := system.length(nodeValue);
    assertEquals('secondChildLength', 1, length);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('ElementisThirdChild', 1, nodeType);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('TextisFourthChild', 3, nodeType);
    nodeValue := node.nodeValue;
    length := system.length(nodeValue);
    assertEquals('fourthChildLength', 1, length);
    node := node.nextSibling;
    nodeType := node.nodeType;
    assertEquals('PIisFifthChild', 7, nodeType);
    nodeValue := TDOMProcessingInstruction(node).data;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    assertEqualsW('trailingPIData', '', nodeValue);
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> graemeg/cpstrnew
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> graemeg/cpstrnew
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> graemeg/cpstrnew
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> origin/cpstrnew
=======
    assertEquals('trailingPIData', '', nodeValue);
>>>>>>> origin/fixes_2.4
    node := node.nextSibling;
    assertNull('SixthIsNull', node);
  end;
end;

{ test removal of superfluous namespace declarations }
procedure TDOMTestExtra2.ls3_canonicform10;
var
  doc: TDOMDocument;
  divList: TDOMNodeList;
  divEl: TDOMElement;
  node: TDOMNode;
begin
<<<<<<< HEAD
  FParser.Options.CanonicalForm := True;
=======
  FParser.Options.PreserveWhitespace := True;
  FParser.Options.Namespaces := True;
>>>>>>> origin/fixes_2.4
  LoadStringData(doc, canonicform03);

  divList := doc.getElementsByTagName('div');
  TDOMNode(divEl) := divList[5];
  node := divEl.getAttributeNode('xmlns');
  assertNotNull('xmlnsPresent', node);
  node := divEl.getAttributeNode('xmlns:a');
  assertNull('xmlnsANotPresent', node);
end;

{ test that defaulted attributes are being replaced by 'normal' ones }
procedure TDOMTestExtra2.ls3_canonicform11;
var
  doc: TDOMDocument;
  elemList: TDOMNodeList;
  elem: TDOMElement;
  attr: TDOMAttr;
  attrSpecified: Boolean;
  attrValue: DOMString;
begin
<<<<<<< HEAD
  FParser.Options.CanonicalForm := True;
=======
  FParser.Options.PreserveWhitespace := True;
  FParser.Options.Namespaces := True;
>>>>>>> origin/fixes_2.4
  LoadStringData(doc, canonicform03);

  elemList := doc.getElementsByTagName('acronym');
  TDOMNode(elem) := elemList[0];
  attr := elem.getAttributeNode('title');
  assertNotNull('titlePresent', attr);
  attrSpecified := attr.specified;
  assertTrue('titleSpecified', attrSpecified);
  attrValue := attr.nodeValue;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('titleValue', 'default', attrValue);
=======
  assertEquals('titleValue', 'default', attrValue);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('titleValue', 'default', attrValue);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('titleValue', 'default', attrValue);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('titleValue', 'default', attrValue);
>>>>>>> origin/cpstrnew
=======
  assertEquals('titleValue', 'default', attrValue);
>>>>>>> origin/fixes_2.4
end;

{ tests that namespace fixup is done while serializing }
{ attribute has no prefix }
procedure TDOMTestExtra2.ls3_DomWriterTest5;
var
  domImpl: TDOMImplementation;
  origDoc: TDOMDocument;
  parsedDoc: TDOMDocument;
  docElem: TDOMElement;
  stream: TStringStream;
  docElemLocalName: DOMString;
  docElemNS: DOMString;
  attrValue: DOMString;
const
  namespaceURI = 'http://www.example.com/DOMWriterTest5';
begin
  FParser.Options.Namespaces := True;
  domImpl := GetImplementation;
  origDoc := domImpl.createDocument(namespaceURI, 'test', nil);
<<<<<<< HEAD
  GC(origDoc);
=======
>>>>>>> origin/fixes_2.4
  docElem := origDoc.documentElement;
  docElem.setAttributeNS(namespaceURI, 'attr', 'test value');

  stream := TStringStream.Create('');
  GC(stream);
  writeXML(origDoc, stream);

  LoadStringData(parsedDoc, stream.DataString);

  docElem := parsedDoc.documentElement;
  docElemLocalName := docElem.localName;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('docElemLocalName', 'test', docElemLocalName);
  docElemNS := TDOMNode(docElem).namespaceURI;
  assertEqualsW('docElemNS', namespaceURI, docElemNS);
  attrValue := docElem.getAttributeNS(namespaceURI, 'attr');
  assertEqualsW('properNSAttrValue', 'test value', attrValue);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  assertEquals('docElemLocalName', 'test', docElemLocalName);
  docElemNS := TDOMNode(docElem).namespaceURI;
  assertEquals('docElemNS', namespaceURI, docElemNS);
  attrValue := docElem.getAttributeNS(namespaceURI, 'attr');
  assertEquals('properNSAttrValue', 'test value', attrValue);
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
end;

{ tests that namespace fixup is done while serializing }
{ same as above, but using an attribute that has a prefix }
procedure TDOMTestExtra2.ls3_DomWriterTest6;
var
  domImpl: TDOMImplementation;
  origDoc: TDOMDocument;
  parsedDoc: TDOMDocument;
  docElem: TDOMElement;
  stream: TStringStream;
  docElemLocalName: DOMString;
  docElemNS: DOMString;
  attrValue: DOMString;
const
  namespaceURI = 'http://www.example.com/DOMWriterTest5';
begin
  FParser.Options.Namespaces := True;
  domImpl := GetImplementation;
  origDoc := domImpl.createDocument(namespaceURI, 'test', nil);
<<<<<<< HEAD
  GC(origDoc);
=======
>>>>>>> origin/fixes_2.4
  docElem := origDoc.documentElement;
  docElem.setAttributeNS(namespaceURI, 'test:attr', 'test value');

  stream := TStringStream.Create('');
  GC(stream);
  writeXML(origDoc, stream);

  LoadStringData(parsedDoc, stream.DataString);

  docElem := parsedDoc.documentElement;
  docElemLocalName := docElem.localName;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('docElemLocalName', 'test', docElemLocalName);
  docElemNS := TDOMNode(docElem).namespaceURI;
  assertEqualsW('docElemNS', namespaceURI, docElemNS);
  attrValue := docElem.getAttributeNS(namespaceURI, 'attr');
  assertEqualsW('properNSAttrValue', 'test value', attrValue);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  assertEquals('docElemLocalName', 'test', docElemLocalName);
  docElemNS := TDOMNode(docElem).namespaceURI;
  assertEquals('docElemNS', namespaceURI, docElemNS);
  attrValue := docElem.getAttributeNS(namespaceURI, 'attr');
  assertEquals('properNSAttrValue', 'test value', attrValue);
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
end;

initialization
  RegisterTest(TDOMTestExtra2);

end.

