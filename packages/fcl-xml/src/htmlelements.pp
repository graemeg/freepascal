{
    $Id: header,v 1.1 2000/07/13 06:33:45 michael Exp $
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit htmlelements;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, DOM, HtmlDefs, strutils;
=======
  Classes, SysUtils, DOM, HtmlDefs;
>>>>>>> graemeg/fixes_2_2
=======
  Classes, SysUtils, DOM, HtmlDefs;
>>>>>>> origin/fixes_2_2

type

  TURI = string;

  { THtmlCustomElement }

  THtmlCustomElement = class (TDOMElement)
  private
    FElementTag: THTMLElementTag;
    function GetAttributeName(index:integer): DOMString;
    function GetAttributeValue(index:integer): DOMString;
    function GetTagName: DOMString;
    procedure WriteAttributes (const aStream : TStream);
    procedure WriteSubNodes (const aStream : TStream);
  protected
<<<<<<< HEAD
<<<<<<< HEAD
    function  GetNodeName: DOMString; override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure StringToStream (const aStream : TStream; s : string);
    procedure StringToStream (const aStream : TStream; Fmt : string; Args : array of const);
    function EscapeString (s : string) : string;
  public
    constructor create (AOwner: TDOMDocument); virtual;

    function AsString : string;
    procedure WriteToStream (const aStream : TStream);  virtual;

    function  GetAttribute(const name: THTMLAttributeTag): DOMString;
    procedure SetAttribute(const name:THTMLAttributeTag; const value: DOMString);
    procedure RemoveAttribute(const name: THTMLAttributeTag);

    property ElementTag : THTMLElementTag read FElementTag write FElementTag;
    property TagName : DOMString read GetTagName;
<<<<<<< HEAD
<<<<<<< HEAD
=======
    property NodeName : DOMstring read GetTagName;
>>>>>>> graemeg/fixes_2_2
=======
    property NodeName : DOMstring read GetTagName;
>>>>>>> origin/fixes_2_2
    property AttributeNames [index:integer] : DOMString read GetAttributeName;
    property AttributeValues [index:integer] : DOMString read GetAttributeValue;
  end;
  THTMLElementClass = class of THTMLCustomELement;


  { THTMLDocument }

  THTMLDocument = class (TDOMDocument)
  public
    procedure SaveToStream (const aStream : TStream);
    procedure SaveToFile (const afilename : string);
    function Asstring : string;
  end;

  { THTMLIDElement }
  
  THTMLIDElement = class (THTMLCustomElement)
  public
    property ID : DOMString index atID read GetAttribute write SetAttribute;
  end;
  
  { THTMLs18nElement }

  THTMLs18nElement = class (THTMLCustomElement)
  private
    function GetDir: THTMLDir;
    procedure SetDir(const AValue: THTMLDir);
  public
    property Dir : THTMLDir read GetDir write SetDir;
    property Lang : DOMString index atLang read GetAttribute write SetAttribute;
  end;
  
  THTMLCoreAttrsElement = class (THTMLIDElement)
  public
    property elementclass : DOMString index atclass read GetAttribute write SetAttribute;
    property style : DOMString index atstyle read GetAttribute write SetAttribute;
    property title : DOMString index attitle read GetAttribute write SetAttribute;
  end;
  
  THTMLCores18nElement = class (THTMLCoreAttrsElement)
  private
    function GetDir: THTMLDir;
    procedure SetDir(const AValue: THTMLDir);
  public
    property Dir : THTMLDir read GetDir write SetDir;
    property Lang : DOMString index atLang read GetAttribute write SetAttribute;
  end;
  
  THTMLAttrsElement = class (THTMLCores18nElement)
    property onclick : DOMString index atonclick read GetAttribute write SetAttribute;
    property ondblclick : DOMString index atondblclick read GetAttribute write SetAttribute;
    property onmousedown : DOMString index atonmousedown read GetAttribute write SetAttribute;
    property onmouseup : DOMString index atonmouseup read GetAttribute write SetAttribute;
    property onmouseover : DOMString index atonmouseover read GetAttribute write SetAttribute;
    property onmousemove : DOMString index atonmousemove read GetAttribute write SetAttribute;
    property onmouseout : DOMString index atonmouseout read GetAttribute write SetAttribute;
    property onkeypress : DOMString index atonkeypress read GetAttribute write SetAttribute;
    property onkeydown : DOMString index atonkeydown read GetAttribute write SetAttribute;
    property onkeyup : DOMString index atonkeyup read GetAttribute write SetAttribute;
  end;
  
// Descendants for all the elements, generated
{$i tagsintf.inc}

  { THTML_text }

  THTML_text = class (THTMLCustomElement)
    FNodeValue : DOMString;
  protected
    function  GetNodeValue: DOMString; override;
    procedure SetNodeValue(const AValue: DOMString); override;
  public
    constructor create (AOwner: TDOMDocument); override;
    procedure WriteToStream (const aStream : TStream);  override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
function EscapeHTML(const s : String) : String;
function UnescapeHTML(const s : String) : String;

implementation

function EscapeHTML ( const S : String ) : String;
begin
  // &apos; does not work on all versions of ie, so do not use it.
  Result := StringsReplace(s,['&','<','>','"',#39],['&amp;','&lt;','&gt;','&quot;','&#39;'],[rfReplaceAll]);
end;

function UnescapeHTML ( const S : String ) : String;
begin
  Result := StringsReplace(s,['&amp;','&lt;','&gt;','&quot;','&apos;','&#39;'],['&','<','>','"',#39,#39],[rfReplaceAll]);
end;

=======
implementation

>>>>>>> graemeg/fixes_2_2
=======
implementation

>>>>>>> origin/fixes_2_2

{ THtmlCustomElement }

function THtmlCustomElement.GetAttributeName(index:integer): DOMString;
var d : TDOMNode;
begin
  d := TDOMNode(Attributes[index]);
  result := d.NodeName;
end;

function THtmlCustomElement.GetAttributeValue(index:integer): DOMString;
var d : TDOMNode;
begin
  d := TDOMNode(Attributes[index]);
  result := d.NodeValue;
end;

function THtmlCustomElement.GetTagName: DOMString;
begin
  result := HTMLElementProps[FElementTag].Name
end;

procedure THtmlCustomElement.WriteAttributes(const aStream: TStream);
var a : THTMLAttributeTag;
    attrs : THTMLAttributeSet;
    s : DOMstring;
begin
  attrs := HTMLElementProps[ElementTag].Attributes;
  for a := low(THTMLAttributeTag) to high(THTMLAttributeTag) do
    if a in attrs then
      begin
      s := GetAttribute (a);
      if s <> '' then
        if a in booleanAttributes then
          StringToStream (aStream, ' %s', [HTMLAttributeTag[a]])
        else
          StringToStream (aStream, ' %s="%s"', [HTMLAttributeTag[a], s]);
      end;
end;

procedure THtmlCustomElement.WriteSubNodes(const aStream: TStream);
var d : TDomNode;
begin
  d := GetFirstChild;
  while assigned (d) do
    begin
    if d is THtmlCustomElement then
      THtmlCustomElement(d).writetostream (aStream);
    d := d.NextSibling;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function THtmlCustomElement.GetNodeName: DOMString;
begin
  Result:=GetTagName;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure THtmlCustomElement.StringToStream(const aStream: TStream; s: string);
begin
  if s <> '' then
    astream.WriteBuffer (s[1], length(s));
end;

procedure THtmlCustomElement.StringToStream(const aStream: TStream; Fmt: string;
  Args: array of const);
begin
  StringToStream (aStream, format (Fmt, args));
end;

function THtmlCustomElement.EscapeString(s: string): string;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  result := EscapeHTML(s);
=======
  result := s;
  //TODO: Needs to convert all the special signs to their html names ("<" has to be "&lt;" etc.)
>>>>>>> graemeg/fixes_2_2
=======
  result := s;
  //TODO: Needs to convert all the special signs to their html names ("<" has to be "&lt;" etc.)
>>>>>>> origin/fixes_2_2
end;

constructor THtmlCustomElement.create(AOwner: TDOMDocument);
begin
  inherited create (AOwner);
  FElementTag := etUnknown;
end;

function THtmlCustomElement.AsString: string;
var s : TStringStream;
begin
  s := TStringStream.Create ('');
  try
    WriteToStream (s);
    result := s.datastring;
  finally
    s.free;
  end;
end;

procedure THtmlCustomElement.WriteToStream(const aStream: TStream);
var f : THTMLElementFlags;
begin
  StringToStream (aStream, '<%s', [TagName]);
  WriteAttributes (aStream);
  StringToStream (aStream, '>'#13#10);
  f := HTMLElementProps[FELementTag].flags;
  if (efSubelementContent in f) or (efPCDATAContent in f) then
    begin
    WriteSubNodes (aStream);
    StringToStream (aStream, '</%s>'#13#10, [TagName]);
    end;
end;

function THtmlCustomElement.GetAttribute(const name: THTMLAttributeTag): DOMString;
begin
  result := inherited GetAttribute (HTMLAttributeTag[name]);
end;

procedure THtmlCustomElement.SetAttribute(const name: THTMLAttributeTag;
  const value: DOMString);
begin
  inherited SetAttribute (HTMLAttributeTag[name], value);
end;

procedure THtmlCustomElement.RemoveAttribute(const name: THTMLAttributeTag);
begin
  inherited RemoveAttribute (HTMLAttributeTag[name]);
end;

{ THTMLs18nElement }

function THTMLs18nElement.GetDir: THTMLDir;
var r : THTMLDir;
    s : DOMString;
begin
  s := GetAttribute (atDir);
  r := high(THTMLdir);
  while (r > low(THTMLDir)) and (comparetext(s,HTMLDir[r]) <> 0) do
    begin
    dec (r);
    end;
  result := r;
end;

procedure THTMLs18nElement.SetDir(const AValue: THTMLDir);
begin
  if AValue = dirEmpty then
    RemoveAttribute(atDir)
  else
    SetAttribute (atDir, HTMLDir[AValue]);
end;

{ THTMLCores18nElement }

function THTMLCores18nElement.GetDir: THTMLDir;
var r : THTMLDir;
    s : DOMString;
begin
  s := GetAttribute (atDir);
  r := high(THTMLdir);
  while (r > low(THTMLDir)) and (comparetext(s,HTMLDir[r]) <> 0) do
    begin
    dec (r);
    end;
  result := r;
end;

procedure THTMLCores18nElement.SetDir(const AValue: THTMLDir);
begin
  if AValue = dirEmpty then
    RemoveAttribute(atDir)
  else
    SetAttribute (atDir, HTMLDir[AValue]);
end;

// generated implementations
{$i tagsimpl.inc}

{ THTML_text }

function THTML_text.GetNodeValue: DOMString;
begin
  Result := FNodeValue;
end;

procedure THTML_text.SetNodeValue(const AValue: DOMString);
begin
  FNodeValue := AValue;
end;

constructor THTML_text.create (AOwner: TDOMDocument);
begin
  inherited create (AOwner);
  ElementTag := ettext;
end;

procedure THTML_text.WriteToStream(const aStream: TStream);
begin
  StringToStream (aStream, NodeValue+#13#10);
end;

{ THTMLDocument }

procedure THTMLDocument.SaveToStream(const aStream: TStream);
var d : TDOMNode;
begin
  d := FirstChild;
  while assigned(d) do
    begin
    if d is THTMLCustomElement then
      THTMLCustomELement(d).WriteToStream(aStream);
    d := d.NextSibling;
    end;
end;

procedure THTMLDocument.SaveToFile(const afilename: string);
var f : TFileStream;
begin
  f := TFileStream.Create (afilename, fmCreate);
  try
    SaveToStream (f);
  finally
    f.Free;
  end;
end;

function THTMLDocument.Asstring: string;
var s : TStringStream;
begin
  s := TStringStream.Create ('');
  try
    SaveToStream (s);
    result := s.DataString;
  finally
    s.Free;
  end;
end;

end.

