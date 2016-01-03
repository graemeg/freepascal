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
unit htmlwriter; 

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DOM, htmlelements;

type

  HTMLWriterException = class (exception);

  { THTMLwriter }

  THTMLwriter = class
  private
    FCurrentElement : THTMLCustomElement;
    FDocument: THTMLDocument;
    procedure SetDocument(const AValue: THTMLDocument);
    procedure SetCurrentElement (AValue : THTMLCustomElement);
  protected
    function CreateElement (tag : THTMLElementClass; s : string) : THTMLCustomElement;
    function CreateElement (tag : THTMLElementClass; sub : THTMLCustomElement) : THTMLCustomElement;
    function CreateElement (tag : THTMLElementClass; subs : Array of THTMLCustomElement) : THTMLCustomElement;
    function CreateElement (tag : THTMLElementClass; subs : TDOMNodelist) : THTMLCustomElement;
    function AddElement (tag : THTMLElementClass) : THTMLCustomElement;
  public
    function StartElement (tag : THTMLElementClass) : THTMLCustomElement;
    function EndElement (tag : THTMLElementClass) : THTMLCustomElement;
    constructor create (aDocument : THTMLDocument);
    procedure AddElement (el : THTMLCustomElement);
    procedure AddElements (subs : TDOMNodelist);
    procedure AddElements (subs : array of THTMLCustomElement);
    function Text (s : string) : THTML_Text;
    function Text (Fmt : string; args : array of const) : THTML_Text;
    { Form input elements }
    function FormText (aname, avalue: DOMstring) : THTML_Input;
    function FormText (aname, avalue: DOMstring; alength : integer) : THTML_Input;
    function FormMemo (aname, avalue: DOMstring; arows,acols: integer) : THTML_Textarea;
    function FormSelect (aname: DOMstring; preselect, size: integer; Options: TStrings; UseValues:boolean) : THTML_Select;
    function FormSelect (aname, preselect: DOMstring; size: integer; Options: TStrings; UseValues:boolean) : THTML_Select;
    function FormPasswd (aname: DOMstring) : THTML_Input;
    function FormCheckbox (aname, avalue: DOMstring; achecked: boolean) : THTML_Input;
    function FormRadio (aname, avalue: DOMstring; achecked: boolean) : THTML_Input;
    function FormSubmit (aname, avalue: DOMstring) : THTML_Input;
    function FormImage (aname, imagesrc, ausemap: DOMstring) : THTML_Input;
    function FormReset : THTML_Input;
    function FormButton (aname, caption, aOnClick: DOMstring) : THTML_Input;
    function FormHidden (aname, aValue: DOMstring) : THTML_Input;
    function FormFile (aname, aValue:DOMstring) : THTML_Input;
<<<<<<< HEAD
<<<<<<< HEAD
    { Other useful links to elements }
=======
    { Other usefull links to elements }
>>>>>>> graemeg/fixes_2_2
=======
    { Other usefull links to elements }
>>>>>>> origin/fixes_2_2
    function Meta (aname, ahtpequiv,acontent: DOMString) : THTML_meta;
    function Link (arel, ahref, athetype, amedia: DOMString) : THTML_link;
    function Script (s, athetype, asrc: DOMString) : THTML_script;
    {$i wtagsintf.inc}
    property Document : THTMLDocument read FDocument write SetDocument;
    property CurrentElement : THTMLCustomElement read FCurrentElement write SetCurrentElement;
  end;

implementation

uses HTMLDefs;

resourcestring
  sErrNoCorespondingParent = 'No open element found with tag "%s"';

{ THTMLwriter }

procedure THTMLwriter.SetDocument(const AValue: THTMLDocument);
begin
  if FDocument <> AValue then
    begin
    FDocument := AValue;
    FCurrentElement := nil;
    end;
end;

function THTMLwriter.CreateElement(tag: THTMLElementClass; s: string): THTMLCustomElement;
begin
  result := StartElement (tag);
  Text (s);
  EndElement (tag);
end;

function THTMLwriter.CreateElement(tag: THTMLElementClass; sub: THTMLCustomElement): THTMLCustomElement;
begin
  result := StartElement (tag);
  AddElement (sub);
  EndElement (tag);
end;

function THTMLwriter.CreateElement(tag: THTMLElementClass; subs: array of THTMLCustomElement): THTMLCustomElement;
begin
  result := StartElement (tag);
  AddElements (subs);
  EndElement (tag);
end;

function THTMLwriter.CreateElement(tag: THTMLElementClass; subs: TDOMNodelist): THTMLCustomElement;
begin
  result := StartElement (tag);
  AddElements (subs);
  EndElement (tag);
end;

function THTMLwriter.StartElement(tag: THTMLElementClass): THTMLCustomElement;
begin
  result := AddElement (tag);
  FCurrentElement := result;
end;

function THTMLwriter.EndElement(tag: THTMLElementClass): THTMLCustomElement;
var d : TDOMNode;
begin
  d := FCurrentElement;
  while assigned(d) and not (d is tag) do
    d := d.ParentNode;
  if assigned (d) then
    begin
    result := THTMLCustomElement(d);
<<<<<<< HEAD
<<<<<<< HEAD
    if result.ParentNode = FDocument then
      FCurrentElement := nil
    else
      FCurrentElement := THTMLCustomElement(result.ParentNode);
=======
    FCurrentElement := THTMLCustomElement(result.ParentNode);
>>>>>>> graemeg/fixes_2_2
=======
    FCurrentElement := THTMLCustomElement(result.ParentNode);
>>>>>>> origin/fixes_2_2
    end
  else
    raise HTMLWriterException.CreateFmt (sErrNoCorespondingParent, [tag.ClassName]);
end;

constructor THTMLwriter.create(aDocument: THTMLDocument);
begin
  inherited create;
  FDocument := aDocument;
end;

procedure THTMLwriter.SetCurrentElement(AValue: THTMLCustomElement);
begin
  if not assigned (AValue) then
    FCurrentElement := nil
  else
    if AValue.OwnerDocument = FDocument then
      FCurrentElement := AValue;
end;

function THTMLwriter.AddElement(tag: THTMLElementClass): THTMLCustomElement;
begin
  result := tag.Create (Document);
  AddElement (result);
end;

procedure THTMLwriter.AddElement(el: THTMLCustomElement);
begin
  if assigned (FCurrentElement) then
    FCurrentElement.AppendChild (el)
  else
    FDocument.AppendChild (el);
end;

procedure THTMLwriter.AddElements(subs: TDOMNodelist);
var r : integer;
    d : TDOMNode;
begin
  for r := 0 to subs.count-1 do
    begin
    d := subs.item[r];
    if d is THTMLCustomElement then
      AddElement (THTMLCustomElement(d));
    end;
end;

procedure THTMLwriter.AddElements(subs: array of THTMLCustomElement);
var r : integer;
begin
  for r := 0 to high(subs) do
    AddElement (subs[r]);
end;

function THTMLwriter.Text (s : string): THTML_Text;
begin
  result := THTML_text(AddElement(THTML_Text));
  result.NodeValue := s;
end;

function THTMLwriter.Text(Fmt: string; args: array of const): THTML_Text;
begin
  result := text(format(fmt, args));
end;

{ Form input elements }

function THTMLwriter.FormText(aname, avalue: DOMstring): THTML_Input;
begin
  result := input;
  with result do
    begin
    thetype := itText;
    name := aname;
    value := avalue;
    end;
end;

function THTMLwriter.FormText(aname, avalue: DOMstring; alength: integer): THTML_Input;
begin
  result := FormText (aname, avalue);
  result.size := inttostr(alength);
end;

function THTMLwriter.FormMemo(aname, avalue: DOMstring; arows, acols: integer): THTML_Textarea;
begin
  result := textarea(avalue);
  with result do
    begin
    name := aname;
    rows := inttostr(arows);
    cols := inttostr(acols);
    end;
end;

function THTMLwriter.FormSelect(aname: DOMstring; preselect, size: integer;
  Options: TStrings; UseValues:boolean): THTML_Select;
var r : integer;
    n,v : string;
begin
  result := StartSelect;
  result.size := inttostr(size);
  result.name := aname;
  if UseValues then
    for r := 0 to options.count-1 do
      begin
      Options.GetNameValue (r, v, n);
      with Option (n) do
        begin
        selected := (preselect = r);
        Value := v;
        end;
      end
  else
    for r := 0 to options.count-1 do
      Option (Options[r]).selected := (preselect = r);
  EndSelect;
end;

function THTMLwriter.FormSelect(aname, preselect: DOMstring; size: integer;
  Options: TStrings; UseValues:boolean): THTML_Select;
begin
  if UseValues then
    result := FormSelect (aname, Options.IndexOfName(preselect), size, Options, UseValues)
  else
    result := FormSelect (aname, Options.IndexOf(preselect), size, Options, UseValues);
end;

function THTMLwriter.FormPasswd(aname: DOMstring): THTML_Input;
begin
  result := input;
  with result do
    begin
    thetype := itPassword;
    name := aname;
    end;
end;

function THTMLwriter.FormCheckbox(aname, avalue: DOMstring; achecked: boolean): THTML_Input;
begin
  result := input;
  with result do
    begin
    thetype := itCheckbox;
    name := aname;
    value := avalue;
    checked := achecked;
    end;
end;

function THTMLwriter.FormRadio(aname, avalue: DOMstring; achecked: boolean): THTML_Input;
begin
  result := input;
  with result do
    begin
    thetype := itCheckbox;
    name := aname;
    value := avalue;
    checked := achecked;
    end;
end;

function THTMLwriter.FormSubmit(aname, avalue: DOMstring): THTML_Input;
begin
  result := input;
  with result do
    begin
    thetype := itSubmit;
    name := aname;
    value := avalue;
    end;
end;

function THTMLwriter.FormImage(aname, imagesrc, ausemap: DOMstring): THTML_Input;
begin
  result := input;
  with result do
    begin
    thetype := itimage;
    name := aname;
    src := imagesrc;
    usemap := ausemap;
    end;
end;

function THTMLwriter.FormReset: THTML_Input;
begin
  result := input;
  result.thetype := itReset;
end;

function THTMLwriter.FormButton(aname, caption, aOnClick: DOMstring): THTML_Input;
begin
  result := input;
  with result do
    begin
    thetype := itButton;
    name := aname;
    value := caption;
    onclick := aonclick;
    end;
end;

function THTMLwriter.FormHidden(aname, aValue: DOMstring): THTML_Input;
begin
  result := Input;
  with result do
    begin
    thetype := itHidden;
    name := aname;
    value := avalue;
    end;
end;

function THTMLwriter.FormFile(aname, aValue: DOMstring): THTML_Input;
begin
  result := Input;
  with result do
    begin
    thetype := itFile;
    name := aname;
    value := aValue;
    end;
end;

function THTMLwriter.Meta(aname, ahtpequiv, acontent: DOMString): THTML_meta;
begin
  result := tagmeta;
  with result do
    begin
    name := aname;
    httpequiv := ahtpequiv;
    content := acontent;
    end;
end;

function THTMLwriter.Link(arel, ahref, athetype, amedia: DOMString): THTML_link;
begin
  result := taglink;
  with result do
    begin
    rel := arel;
    href := ahref;
    thetype := athetype;
    media := amedia;
    end;
end;

function THTMLwriter.Script(s, athetype, asrc: DOMString): THTML_script;
begin
  result := tagscript(s);
  with result do
    begin
    thetype := athetype;
    src := asrc;
    end;
end;

{$i wtagsimpl.inc}

end.

