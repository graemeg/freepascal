{**********************************************************************

    This file is part of the Free Component Library (FCL)

    DOM Test cases which are missing from w3.org test suite
    Copyright (c) 2008 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit extras;
{$mode objfpc}{$H+}
interface

uses
<<<<<<< HEAD
  SysUtils, Classes, DOM, xmlread, xmlwrite, domunit, testregistry;
=======
  SysUtils, Classes, DOM, xmlread, domunit, testregistry;
>>>>>>> origin/fixes_2.4

implementation

type
  TDOMTestExtra = class(TDOMTestBase)
  published
    procedure attr_ownership01;
    procedure attr_ownership02;
    procedure attr_ownership03;
    procedure attr_ownership04;
<<<<<<< HEAD
    procedure attr_ownership05;
    procedure replacesamechild;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure insertbeforefirst;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure nsFixup1;
    procedure nsFixup2;
    procedure nsFixup3;
=======
>>>>>>> origin/fixes_2.4
  end;

{ TDOMTestExtra }

// verify that an attribute created by Element.SetAttribute()
// has its OwnerElement assigned properly
procedure TDOMTestExtra.attr_ownership01;
var
  doc: TDOMDocument;
  el: TDOMElement;
  attr: TDOMAttr;
  attrOwner: TDOMElement;
begin
  LoadStringData(doc, '<doc/>');
  el := doc.CreateElement('element1');
  el.SetAttribute('newAttr', 'newValue');
  attr := el.GetAttributeNode('newAttr');
  AssertNotNull('attribute', attr);
  attrOwner := attr.OwnerElement;
  AssertEquals('ownerElement', el, attrOwner);
  AssertTrue('specified', attr.Specified);
end;

// verify that an attribute created by Element.SetAttributeNS()
// has its OwnerElement assigned properly
procedure TDOMTestExtra.attr_ownership02;
var
  doc: TDOMDocument;
  el: TDOMElement;
  attr: TDOMAttr;
  attrOwner: TDOMElement;
begin
  LoadStringData(doc, '<doc/>');
  el := doc.CreateElement('element1');
  el.SetAttributeNS('http://www.freepascal.org', 'fpc:newAttr', 'newValue');
  attr := el.GetAttributeNodeNS('http://www.freepascal.org', 'newAttr');
  AssertNotNull('attribute', attr);
  attrOwner := attr.OwnerElement;
  AssertEquals('ownerElement', el, attrOwner);
  AssertTrue('specified', attr.Specified);
end;

// verify that NamedNodeMap.SetNamedItem() resets OwnerElement
// of the attribute being replaced
procedure TDOMTestExtra.attr_ownership03;
var
  doc: TDOMDocument;
  el: TDOMElement;
  attr, attr2: TDOMAttr;
  retNode: TDOMNode;
begin
  LoadStringData(doc, '<doc/>');
  el := doc.CreateElement('element1');
  attr := doc.CreateAttribute('newAttr');
  el.SetAttributeNode(attr);
  AssertEquals('ownerElement_before', el, attr.OwnerElement);
  attr2 := doc.CreateAttribute('newAttr');
  retNode := el.Attributes.SetNamedItem(attr2);
  AssertSame('retNode', attr, retNode);
  AssertNull('ownerElement_after', attr.OwnerElement);
  AssertEquals('ownerElement2', el, attr2.OwnerElement);
end;

// verify that NamedNodeMap.SetNamedItemNS() resets OwnerElement
// of the attribute being replaced
procedure TDOMTestExtra.attr_ownership04;
var
  doc: TDOMDocument;
  el: TDOMElement;
  attr, attr2: TDOMAttr;
  retNode: TDOMNode;
begin
  LoadStringData(doc, '<doc/>');
  el := doc.CreateElement('element1');
  attr := doc.CreateAttributeNS('http://www.freepascal.org', 'fpc:newAttr');
  el.SetAttributeNodeNS(attr);
  AssertEquals('ownerElement_before', el, attr.OwnerElement);
  attr2 := doc.CreateAttributeNS('http://www.freepascal.org', 'fpc:newAttr');
  retNode := el.Attributes.SetNamedItemNS(attr2);
  AssertSame('retNode', attr, retNode);
  AssertNull('ownerElement_after', attr.OwnerElement);
  AssertEquals('ownerElement2', el, attr2.OwnerElement);
end;


<<<<<<< HEAD
// verify that Element.removeAttributeNode() resets ownerElement
// of the attribute being removed
procedure TDOMTestExtra.attr_ownership05;
var
  doc: TDOMDocument;
  el: TDOMElement;
  attr: TDOMAttr;
begin
  LoadStringData(doc, '<doc/>');
  el := doc.CreateElement('element1');
  attr := doc.CreateAttributeNS('http://www.freepascal.org', 'fpc:newAttr');
  el.SetAttributeNodeNS(attr);
  AssertEquals('ownerElement_before', el, attr.OwnerElement);
  el.RemoveAttributeNode(attr);
  AssertNull('ownerElement_after', attr.ownerElement);
end;

// verify that replacing a node by itself does not remove it from the tree
// (specs say this is implementation-dependent, but guess that means either
//  no-op or raising an exception, not removal).
procedure TDOMTestExtra.replacesamechild;
var
  doc: TDOMDocument;
  root, el, prev, next: TDOMNode;
begin
  LoadStringData(doc, '<root><child1/><child2/><child3/></root>');
  root := doc.DocumentElement;
  el := root.ChildNodes[1];
  prev := el.PreviousSibling;
  next := el.NextSibling;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEqualsW('prev_name_before', 'child1', prev.NodeName);
  AssertEqualsW('next_name_before', 'child3', next.NodeName);
=======
  AssertEquals('prev_name_before', 'child1', prev.NodeName);
  AssertEquals('next_name_before', 'child3', next.NodeName);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('prev_name_before', 'child1', prev.NodeName);
  AssertEquals('next_name_before', 'child3', next.NodeName);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('prev_name_before', 'child1', prev.NodeName);
  AssertEquals('next_name_before', 'child3', next.NodeName);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('prev_name_before', 'child1', prev.NodeName);
  AssertEquals('next_name_before', 'child3', next.NodeName);
>>>>>>> origin/cpstrnew
  root.replaceChild(el, el);
  prev := el.PreviousSibling;
  next := el.NextSibling;
  AssertNotNull('prev_after', prev);
  AssertNotNull('prev_after', next);  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEqualsW('prev_name_after', 'child1', prev.NodeName);
  AssertEqualsW('next_name_after', 'child3', next.NodeName);
end;

// verify that inserting a node before the first child sets
// both refnode.previoussibling and newnode.nextsibling properties
procedure TDOMTestExtra.insertbeforefirst;
var
  doc: TDOMDocument;
  root, refchild, newchild: TDOMNode;
begin
  LoadStringData(doc, '<root><child1/><child2/><child3/></root>');
  root := doc.DocumentElement;
  refchild := root.FirstChild;
  newchild := doc.CreateElement('new');
  root.insertbefore(newchild, refchild);
  AssertEquals('prev', refchild.previoussibling, newchild);
  AssertEquals('next', newchild.nextsibling, refchild);
  AssertEquals('child', root.firstchild, newchild);
=======
  AssertEquals('prev_name_after', 'child1', prev.NodeName);
  AssertEquals('next_name_after', 'child3', next.NodeName);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('prev_name_after', 'child1', prev.NodeName);
  AssertEquals('next_name_after', 'child3', next.NodeName);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('prev_name_after', 'child1', prev.NodeName);
  AssertEquals('next_name_after', 'child3', next.NodeName);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('prev_name_after', 'child1', prev.NodeName);
  AssertEquals('next_name_after', 'child3', next.NodeName);
>>>>>>> origin/cpstrnew
end;

const
  nsURI1 = 'http://www.example.com/ns1';
  nsURI2 = 'http://www.example.com/ns2';

// verify the namespace fixup with two nested elements
// (same localName, different nsURI, and no prefixes)
procedure TDOMTestExtra.nsFixup1;
var
  domImpl: TDOMImplementation;
  origDoc: TDOMDocument;
  parsedDoc: TDOMDocument;
  docElem: TDOMElement;
  el: TDOMElement;
  stream: TStringStream;
  list: TDOMNodeList;
begin
  FParser.Options.Namespaces := True;
  domImpl := GetImplementation;
  origDoc := domImpl.createDocument(nsURI1, 'test', nil);
  GC(origDoc);
  docElem := origDoc.documentElement;
  el := origDoc.CreateElementNS(nsURI2, 'test');
  docElem.AppendChild(el);

  stream := TStringStream.Create('');
  GC(stream);
  writeXML(origDoc, stream);
  LoadStringData(parsedDoc, stream.DataString);

  docElem := parsedDoc.documentElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('docElemLocalName', 'test', docElem.localName);
  assertEqualsW('docElemNS', nsURI1, docElem.namespaceURI);
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> origin/cpstrnew

  list := docElem.GetElementsByTagNameNS(nsURI2, '*');
  assertEquals('ns2_elementCount', 1, list.Length);
  el := TDOMElement(list[0]);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('ns2_nodeName', 'test', el.nodeName);
=======
  assertEquals('ns2_nodeName', 'test', el.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('ns2_nodeName', 'test', el.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('ns2_nodeName', 'test', el.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('ns2_nodeName', 'test', el.nodeName);
>>>>>>> origin/cpstrnew
end;

// verify the namespace fixup with two nested elements
// (same localName, different nsURI, different prefixes)
procedure TDOMTestExtra.nsFixup2;
var
  domImpl: TDOMImplementation;
  origDoc: TDOMDocument;
  parsedDoc: TDOMDocument;
  docElem: TDOMElement;
  el: TDOMElement;
  stream: TStringStream;
  list: TDOMNodeList;
begin
  FParser.Options.Namespaces := True;
  domImpl := GetImplementation;
  origDoc := domImpl.createDocument(nsURI1, 'a:test', nil);
  GC(origDoc);
  docElem := origDoc.documentElement;
  el := origDoc.CreateElementNS(nsURI2, 'b:test');
  docElem.AppendChild(el);

  stream := TStringStream.Create('');
  GC(stream);
  writeXML(origDoc, stream);
  LoadStringData(parsedDoc, stream.DataString);

  docElem := parsedDoc.documentElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('docElemLocalName', 'test', docElem.localName);
  assertEqualsW('docElemNS', nsURI1, docElem.namespaceURI);
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> origin/cpstrnew

  list := docElem.GetElementsByTagNameNS(nsURI2, '*');
  assertEquals('ns2_elementCount', 1, list.Length);
  el := TDOMElement(list[0]);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('ns2_nodeName', 'b:test', el.nodeName);
=======
  assertEquals('ns2_nodeName', 'b:test', el.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('ns2_nodeName', 'b:test', el.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('ns2_nodeName', 'b:test', el.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('ns2_nodeName', 'b:test', el.nodeName);
>>>>>>> origin/cpstrnew
end;

// verify the namespace fixup with two nested elements and an attribute
// attribute's prefix must change to that of document element
procedure TDOMTestExtra.nsFixup3;
var
  domImpl: TDOMImplementation;
  origDoc: TDOMDocument;
  parsedDoc: TDOMDocument;
  docElem: TDOMElement;
  el: TDOMElement;
  stream: TStringStream;
  list: TDOMNodeList;
  attr: TDOMAttr;
begin
  FParser.Options.Namespaces := True;
  domImpl := GetImplementation;
  origDoc := domImpl.createDocument(nsURI1, 'a:test', nil);
  GC(origDoc);
  docElem := origDoc.documentElement;
  el := origDoc.CreateElementNS(nsURI2, 'b:test');
  docElem.AppendChild(el);
  el.SetAttributeNS(nsURI1, 'test:attr', 'test value');

  stream := TStringStream.Create('');
  GC(stream);
  writeXML(origDoc, stream);
  LoadStringData(parsedDoc, stream.DataString);

  docElem := parsedDoc.documentElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('docElemLocalName', 'test', docElem.localName);
  assertEqualsW('docElemNS', nsURI1, docElem.namespaceURI);
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('docElemLocalName', 'test', docElem.localName);
  assertEquals('docElemNS', nsURI1, docElem.namespaceURI);
>>>>>>> origin/cpstrnew

  list := docElem.GetElementsByTagNameNS(nsURI2, '*');
  assertEquals('ns2_elementCount', 1, list.Length);
  el := TDOMElement(list[0]);
  attr := el.GetAttributeNodeNS(nsURI1, 'attr');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  assertEqualsW('attr_nodeName', 'a:attr', attr.nodeName);
=======
  assertEquals('attr_nodeName', 'a:attr', attr.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('attr_nodeName', 'a:attr', attr.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('attr_nodeName', 'a:attr', attr.nodeName);
>>>>>>> graemeg/cpstrnew
=======
  assertEquals('attr_nodeName', 'a:attr', attr.nodeName);
>>>>>>> origin/cpstrnew
end;
=======
>>>>>>> origin/fixes_2.4


initialization
  RegisterTest(TDOMTestExtra);

end.

