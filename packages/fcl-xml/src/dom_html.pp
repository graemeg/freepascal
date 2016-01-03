{
    This file is part of the Free Component Library

    Implementation of DOM HTML interfaces
    Copyright (c) 2002 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
=======
{ Please note that this is a very early version, most properties and methods
  are not implemented yet. }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$mode objfpc}
{$H+}

unit DOM_HTML;

interface

uses DOM, htmldefs, xmlutils, SysUtils;

type
  THTMLDocument = class;
  THTMLFormElement = class;
  THTMLTableCaptionElement = class;
  THTMLTableSectionElement = class;

  TFilterProc = function(aNode: TDOMNode): TFilterResult;

  // Tests cast Collection <-> OptionsCollection in arbitrary way...
  THTMLCollection = class(TDOMNodeList)
  protected
    FFilterProc: TFilterProc;
    function NodeFilter(aNode: TDOMNode): TFilterResult; override;
  public
    constructor Create(aNode: TDOMNode; Proc: TFilterProc);
    function NamedItem(const Index: DOMString): TDOMNode;
  end;

  // differs from HTMLCollection in that Length *may* be writable.
  // for now, let's pretend it's not...
  THTMLOptionsCollection = class(THTMLCollection)
  end;

  THTMLElement = class(TDOMElement)
  private
    function GetStrAttr(idx: THTMLAttributeTag): DOMString;
    procedure SetStrAttr(idx: THTMLAttributeTag; const value: DOMString);
    function GetIntAttr(idx: THTMLAttributeTag): Integer;
    procedure SetIntAttr(idx: THTMLAttributeTag; value: Integer);
    function GetUIntAttr(idx: THTMLAttributeTag): Cardinal;
    procedure SetUIntAttr(idx: THTMLAttributeTag; value: Cardinal);
    function GetBoolAttr(idx: THTMLAttributeTag): Boolean;
    procedure SetBoolAttr(idx: THTMLAttributeTag; value: Boolean);
  protected
    function GetForm: THTMLFormElement;
  public
    property ID: DOMString index atid read GetStrAttr write SetStrAttr;
    property Title: DOMString index attitle read GetStrAttr write SetStrAttr;
    property Lang: DOMString index atlang read GetStrAttr write SetStrAttr;
    property Dir: DOMString index atdir read GetStrAttr write SetStrAttr;
    property ClassName: DOMString index atclass read GetStrAttr write SetStrAttr;
  end;

  THTMLHtmlElement = class(THTMLElement)
  public
    property Version: DOMString index atversion read GetStrAttr write SetStrAttr;
  end;

  THTMLHeadElement = class(THTMLElement)
  public
    property Profile: DOMString index atprofile read GetStrAttr write SetStrAttr;
  end;

  THTMLLinkElement = class(THTMLElement)
  public
    property Disabled: Boolean index atdisabled read GetBoolAttr write SetBoolAttr;
    property Charset: DOMString index atcharset read GetStrAttr write SetStrAttr;
    property HRef: DOMString index athref read GetStrAttr write SetStrAttr;
    property HRefLang: DOMString index athreflang read GetStrAttr write SetStrAttr;
    property Media: DOMString index atmedia read GetStrAttr write SetStrAttr;
    property Rel: DOMString index atrel read GetStrAttr write SetStrAttr;
    property Rev: DOMString index atrev read GetStrAttr write SetStrAttr;
    property Target: DOMString index attarget read GetStrAttr write SetStrAttr;
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
  end;

  THTMLTitleElement = class(THTMLElement)
  public
    property Text: DOMString read GetTextContent write SetTextContent;
  end;

  THTMLMetaElement = class(THTMLElement)
  public
    property Content: DOMString index atcontent read GetStrAttr write SetStrAttr;
    property HTTPEquiv: DOMString index athttpequiv read GetStrAttr write SetStrAttr;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property Scheme: DOMString index atscheme read GetStrAttr write SetStrAttr;
  end;

  THTMLBaseElement = class(THTMLElement)
  public
    property HRef: DOMString index athref read GetStrAttr write SetStrAttr;
    property Target: DOMString index attarget read GetStrAttr write SetStrAttr;
  end;

  THTMLIsIndexElement = class(THTMLElement)
  public
    property Form: THTMLFormElement read GetForm;
    property Prompt: DOMString index atprompt read GetStrAttr write SetStrAttr; // 4.01 deprecated
  end;

  THTMLStyleElement = class(THTMLElement)
  public
    property Disabled: Boolean index atdisabled read GetBoolAttr write SetBoolAttr;
    property Media: DOMString index atmedia read GetStrAttr write SetStrAttr;
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
  end;

  THTMLBodyElement = class(THTMLElement)
  public
    property ALink: DOMString index atalink read GetStrAttr write SetStrAttr;
    property Background: DOMString index atbackground read GetStrAttr write SetStrAttr;
    property BgColor: DOMString index atbgcolor read GetStrAttr write SetStrAttr;
    property Link: DOMString index atlink read GetStrAttr write SetStrAttr;
    property Text: DOMString index attext read GetStrAttr write SetStrAttr;
    property VLink: DOMString index atvlink read GetStrAttr write SetStrAttr;
  end;

  THTMLFormElement = class(THTMLElement)
  private
    function GetElements: THTMLCollection;
    function GetLength: Integer;
  public
    property Elements: THTMLCollection read GetElements;
    property Length: Integer read GetLength;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property AcceptCharset: DOMString index atacceptcharset read GetStrAttr write SetStrAttr;
    property Action: DOMString index ataction read GetStrAttr write SetStrAttr;
    property EncType: DOMString index atenctype read GetStrAttr write SetStrAttr;
    property Method: DOMString index atmethod read GetStrAttr write SetStrAttr;
    property Target: DOMString index attarget read GetStrAttr write SetStrAttr;
    procedure Submit;
    procedure Reset;
  end;

  THTMLSelectElement = class(THTMLElement)
  private
    FSelectedIndex: Integer;
    function GetType: DOMString;
    function GetValue: DOMString;
    procedure SetValue(const value: DOMString);
    function GetOptions: THTMLOptionsCollection;
    function GetLength: Cardinal;
    procedure SetLength(aValue: Cardinal);
  public
    property HTMLType: DOMString read GetType;
    property SelectedIndex: Integer read FSelectedIndex write FSelectedIndex;
    property Value: DOMString read GetValue write SetValue;
    // maps to Options.Length
    property Length: Cardinal read GetLength write SetLength;
    property Form: THTMLFormElement read GetForm;
    property Options: THTMLOptionsCollection read GetOptions;
    property Disabled: Boolean index atdisabled read GetBoolAttr write SetBoolAttr;
    property Multiple: Boolean index atmultiple read GetBoolAttr write SetBoolAttr;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property Size: Integer index atsize read GetIntAttr write SetIntAttr;
    property TabIndex: Integer index attabindex read GetIntAttr write SetIntAttr;
    procedure Add(Element, Before: THTMLElement);
    procedure Remove(Index: Integer);
    procedure Blur;
    procedure Focus;
  end;

  THTMLOptGroupElement = class(THTMLElement)
  public
    property Disabled: Boolean index atdisabled read GetBoolAttr write SetBoolAttr;
    property GroupLabel: DOMString index atlabel read GetStrAttr write SetStrAttr;
  end;

  THTMLOptionElement = class(THTMLElement)
  private
    function GetIndex: Integer;
  public
    property Form: THTMLFormElement read GetForm;
    property DefaultSelected: Boolean index atselected read GetBoolAttr write SetBoolAttr;
    property Text: DOMString read GetTextContent;
    property Index: Integer read GetIndex;
    property Disabled: Boolean index atdisabled read GetBoolAttr write SetBoolAttr;
    // TODO: name? was 'label'
    property OptionLabel: DOMString index atlabel read GetStrAttr write SetStrAttr;
    // TODO: GUI state, must not be mapped to attribute
    property Selected: Boolean index atselected read GetBoolAttr write SetBoolAttr;
    property Value: DOMString index atvalue read GetStrAttr write SetStrAttr;
  end;

  THTMLInputElement = class(THTMLElement)
  private
    FChecked: Boolean; // !!! TEMP
  public
    property DefaultValue: DOMString index atvalue read GetStrAttr write SetStrAttr;
    property DefaultChecked: Boolean index atchecked read GetBoolAttr write SetBoolAttr;
    property Form: THTMLFormElement read GetForm;
    property Accept: DOMString index ataccept read GetStrAttr write SetStrAttr;
    property AccessKey: DOMString index ataccesskey read GetStrAttr write SetStrAttr;
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property Alt: DOMString index atalt read GetStrAttr write SetStrAttr;
    // TODO: GUI state, not mapped to attribute
    property Checked: Boolean read FChecked write FChecked;
    property Disabled: Boolean index atdisabled read GetBoolAttr write SetBoolAttr;
    property MaxLength: Integer index atmaxlength read GetIntAttr write SetIntAttr;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property ReadOnly: Boolean index atreadonly read GetBoolAttr write SetBoolAttr;
    property Size: Cardinal index atsize read GetUIntAttr write SetUIntAttr;
    property Src: DOMString index atsrc read GetStrAttr write SetStrAttr;
    property TabIndex: Integer index attabindex read GetIntAttr write SetIntAttr;
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
    property UseMap: DOMString index atusemap read GetStrAttr write SetStrAttr;
    // TODO: GUI state, not mapped to attribute
    property Value: DOMString index atvalue read GetStrAttr write SetStrAttr;
    procedure Blur;
    procedure Focus;
    procedure Select;
    procedure Click;
  end;

  THTMLTextAreaElement = class(THTMLElement)
  private
    function GetType: DOMString;
  public
    property DefaultValue: DOMString read GetTextContent write SetTextContent;
    property Form: THTMLFormElement read GetForm;
    property AccessKey: DOMString index ataccesskey read GetStrAttr write SetStrAttr;
    property Cols: Integer index atcols read GetIntAttr write SetIntAttr;
    property Disabled: Boolean index atdisabled read GetBoolAttr write SetBoolAttr;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property ReadOnly: Boolean index atreadonly read GetBoolAttr write SetBoolAttr;
    property Rows: Integer index atrows read GetIntAttr write SetIntAttr;
    property TabIndex: Integer index attabindex read GetIntAttr write SetIntAttr;
    property HTMLType: DOMString read GetType;
    // TODO: GUI state, not mapped to attribute
    property Value: DOMString read GetTextContent write SetTextContent;
    procedure Blur;
    procedure Focus;
    procedure Select;
  end;

  THTMLButtonElement = class(THTMLElement)
  public
    property Form: THTMLFormElement read GetForm;
    property AccessKey: DOMString index ataccesskey read GetStrAttr write SetStrAttr;
    property Disabled: Boolean index atdisabled read GetBoolAttr write SetBoolAttr;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property TabIndex: Integer index attabindex read GetIntAttr write SetIntAttr;
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
    property Value: DOMString index atvalue read GetStrAttr write SetStrAttr;
  end;

  THTMLLabelElement = class(THTMLElement)
  public
    property Form: THTMLFormElement read GetForm;
    property AccessKey: DOMString index ataccesskey read GetStrAttr write SetStrAttr;
    property HtmlFor: DOMString index atfor read GetStrAttr write SetStrAttr;
  end;

  THTMLFieldSetElement = class(THTMLElement)
  public
    property Form: THTMLFormElement read GetForm;
  end;

  THTMLLegendElement = class(THTMLElement)
  public
    property Form: THTMLFormElement read GetForm;
    property AccessKey: DOMString index ataccesskey read GetStrAttr write SetStrAttr;
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
  end;

  THTMLUListElement = class(THTMLElement)
  public
    property Compact: Boolean index atcompact read GetBoolAttr write SetBoolAttr; // 4.01 deprecated
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
  end;

  THTMLOListElement = class(THTMLElement)
  public
    property Compact: Boolean index atcompact read GetBoolAttr write SetBoolAttr; // 4.01 deprecated
    property Start: Integer index atstart read GetIntAttr write SetIntAttr;       // 4.01 deprecated
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr; // 4.01 deprecated
  end;

  THTMLDListElement = class(THTMLElement)
  public
    property Compact: Boolean index atcompact read GetBoolAttr write SetBoolAttr; // 4.01 deprecated
  end;

  THTMLDirectoryElement = class(THTMLElement) // 4.01 deprecated
  public
    property Compact: Boolean index atcompact read GetBoolAttr write SetBoolAttr;
  end;

  THTMLMenuElement = class(THTMLElement)  // 4.01 deprecated
  public
    property Compact: Boolean index atcompact read GetBoolAttr write SetBoolAttr;
  end;

  THTMLLIElement = class(THTMLElement)
  public
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;  // 4.01 deprecated
    property Value: Integer index atvalue read GetIntAttr write SetIntAttr;  // 4.01 deprecated
  end;

  THTMLDivElement = class(THTMLElement)
  public
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;  // 4.01 deprecated
  end;

  THTMLParagraphElement = class(THTMLElement)
  public
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;  // 4.01 deprecated
  end;

  THTMLHeadingElement = class(THTMLElement)
  public
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;  // 4.01 deprecated
  end;

  THTMLQuoteElement = class(THTMLElement)
  public
    property Cite: DOMString index atcite read GetStrAttr write SetStrAttr;
  end;

  THTMLPreElement = class(THTMLElement)
  public
    property Width: Integer index atwidth read GetIntAttr write SetIntAttr;  // 4.01 deprecated
  end;

  THTMLBRElement = class(THTMLElement)
  public
    property Clear: DOMString index atclear read GetStrAttr write SetStrAttr; // 4.01 deprecated
  end;

  // yep, BaseFont.size is integer, but Font.size is a String
  THTMLBaseFontElement = class(THTMLElement) // 4.01 deprecated
  public
    property Color: DOMString index atcolor read GetStrAttr write SetStrAttr; // 4.01 deprecated
    property Face: DOMString index atface read GetStrAttr write SetStrAttr;   // 4.01 deprecated
    property Size: Integer index atsize read GetIntAttr write SetIntAttr; // 4.01 deprecated
  end;

  THTMLFontElement = class(THTMLElement)  // 4.01 deprecated
  public
    property Color: DOMString index atcolor read GetStrAttr write SetStrAttr;
    property Face: DOMString index atface read GetStrAttr write SetStrAttr;
    property Size: DOMString index atsize read GetStrAttr write SetStrAttr;
  end;

  THTMLHRElement = class(THTMLElement)
  public
    property Align: DOMString index atalign  read GetStrAttr write SetStrAttr;
    property NoShade: Boolean index atnoshade read GetBoolAttr write SetBoolAttr; // 4.01 deprecated
    property Size: DOMString index atsize read GetStrAttr write SetStrAttr;
    property Width: DOMString index atwidth read GetStrAttr write SetStrAttr;
  end;

  THTMLModElement = class(THTMLElement)
  public
    property Cite: DOMString index atcite read GetStrAttr write SetStrAttr;
    property DateTime: DOMString index atdatetime read GetStrAttr write SetStrAttr;
  end;

  THTMLAnchorElement = class(THTMLElement)
  public
    property AccessKey: DOMString index ataccesskey read GetStrAttr write SetStrAttr;
    property Charset: DOMString index atcharset read GetStrAttr write SetStrAttr;
    property Coords: DOMString index atcoords read GetStrAttr write SetStrAttr;
    property HRef: DOMString index athref read GetStrAttr write SetStrAttr;
    property HRefLang: DOMString index athreflang read GetStrAttr write SetStrAttr;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property Rel: DOMString index atrel read GetStrAttr write SetStrAttr;
    property Rev: DOMString index atrev read GetStrAttr write SetStrAttr;
    property Shape: DOMString index atshape read GetStrAttr write SetStrAttr;
    property TabIndex: Integer index attabindex read GetIntAttr write SetIntAttr;
    property Target: DOMString index attarget read GetStrAttr write SetStrAttr;
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
    procedure Blur;
    procedure Focus;
  end;

  THTMLImageElement = class(THTMLElement)
  public
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property Alt: DOMString index atalt read GetStrAttr write SetStrAttr;
    property Border: DOMString index atborder read GetStrAttr write SetStrAttr;
    property Height: Integer index atheight read GetIntAttr write SetIntAttr;
    property HSpace: Integer index athspace read GetIntAttr write SetIntAttr; // 4.01 deprecated
    property IsMap: Boolean index atismap read GetBoolAttr write SetBoolAttr;
    property LongDesc: DOMString index atlongdesc read GetStrAttr write SetStrAttr;
    property Src: DOMString index atsrc read GetStrAttr write SetStrAttr;
    property UseMap: DOMString index atusemap read GetStrAttr write SetStrAttr;
    property VSpace: Integer index atvspace read GetIntAttr write SetIntAttr; // 4.01 deprecated
    property Width: Integer index atwidth read GetIntAttr write SetIntAttr;
  end;

  THTMLObjectElement = class(THTMLElement)
  private
    function GetContentDocument: TDOMDocument;
  public
    property Form: THTMLFormElement read GetForm;
    property Code: DOMString index atcode read GetStrAttr write SetStrAttr; // 4.01 deprecated
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property Archive: DOMString index atarchive read GetStrAttr write SetStrAttr;
    property Border: DOMString index atborder read GetStrAttr write SetStrAttr;
    property CodeBase: DOMString index atcodebase read GetStrAttr write SetStrAttr;
    property CodeType: DOMString index atcodetype read GetStrAttr write SetStrAttr;
    property Data: DOMString index atdata read GetStrAttr write SetStrAttr;
    property Declare: Boolean index atdeclare read GetBoolAttr write SetBoolAttr;
    property Height: DOMString index atheight read GetStrAttr write SetStrAttr;
    property HSpace: Integer index athspace read GetIntAttr write SetIntAttr; // 4.01 deprecated
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property StandBy: DOMString index atstandby read GetStrAttr write SetStrAttr;
    property TabIndex: Integer index attabindex read GetIntAttr write SetIntAttr;
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
    property UseMap: DOMString index atusemap read GetStrAttr write SetStrAttr;
    property VSpace: Integer index atvspace read GetIntAttr write SetIntAttr; // 4.01 deprecated
    property Width: DOMString index atwidth read GetStrAttr write SetStrAttr;
    property ContentDocument: TDOMDocument read GetContentDocument;
  end;

  THTMLParamElement = class(THTMLElement)
  public
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
    property Value: DOMString index atvalue read GetStrAttr write SetStrAttr;
    property ValueType: DOMString index atvaluetype read GetStrAttr write SetStrAttr;
  end;

  THTMLAppletElement = class(THTMLElement)
  public
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property Alt: DOMString index atalt read GetStrAttr write SetStrAttr;
    property Archive: DOMString index atarchive read GetStrAttr write SetStrAttr;
    property Code: DOMString index atcode read GetStrAttr write SetStrAttr;  // 4.01 deprecated
    property CodeBase: DOMString index atcodebase  read GetStrAttr write SetStrAttr;
    property Height: DOMString index atheight read GetStrAttr write SetStrAttr;
    property HSpace: Integer index athspace read GetIntAttr write SetIntAttr;  // 4.01 deprecated
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property AppletObject: DOMString index atobject read GetStrAttr write SetStrAttr;
    property VSpace: Integer index atvspace read GetIntAttr write SetIntAttr;  // 4.01 deprecated
    property Width: DOMString index atwidth read GetStrAttr write SetStrAttr;
  end;

  THTMLMapElement = class(THTMLElement)
  private
    function GetAreas: THTMLCollection;
  public
    property Areas: THTMLCollection read GetAreas;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
  end;

  THTMLAreaElement = class(THTMLElement)
  public
    property AccessKey: DOMString index ataccesskey  read GetStrAttr write SetStrAttr;
    property Alt: DOMString index atalt read GetStrAttr write SetStrAttr;
    property Coords: DOMString index atcoords read GetStrAttr write SetStrAttr;
    property HRef: DOMString index athref read GetStrAttr write SetStrAttr;
    property NoHRef: Boolean index atnohref read GetBoolAttr write SetBoolAttr;
    property Shape: DOMString index atshape read GetStrAttr write SetStrAttr;
    property TabIndex: Integer index attabindex read GetIntAttr write SetIntAttr;
    property Target: DOMString index attarget read GetStrAttr write SetStrAttr;
  end;

  THTMLScriptElement = class(THTMLElement)
  private
    function GetEvent: DOMString;
    procedure SetEvent(const Value: DOMString);
  public
    property Text: DOMString read GetTextContent write SetTextContent;
    property HtmlFor: DOMString index atfor read GetStrAttr write SetStrAttr;
    { reserved for future use }
    property Event: DOMString read GetEvent write SetEvent;
    property Charset: DOMString index atcharset read GetStrAttr write SetStrAttr;
    property Defer: Boolean index atdefer read GetBoolAttr write SetBoolAttr;
    property Src: DOMString index atsrc read GetStrAttr write SetStrAttr;
    property HTMLType: DOMString index attype read GetStrAttr write SetStrAttr;
  end;

  THTMLTableElement = class(THTMLElement)
  private
    function GetRows: THTMLCollection;
    function GetBodies: THTMLCollection;
    function GetCaption: THTMLTableCaptionElement;
    procedure SetCaption(value: THTMLTableCaptionElement);
    function GetHead: THTMLTableSectionElement;
    procedure SetHead(value: THTMLTableSectionElement);
    function GetFoot: THTMLTableSectionElement;
    procedure SetFoot(value: THTMLTableSectionElement);
  public
    property Caption: THTMLTableCaptionElement read GetCaption write SetCaption;
    property THead: THTMLTableSectionElement read GetHead write SetHead;
    property TFoot: THTMLTableSectionElement read GetFoot write SetFoot;
    property Rows: THTMLCollection read GetRows;
    property TBodies: THTMLCollection read GetBodies;
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property BgColor: DOMString index atbgcolor read GetStrAttr write SetStrAttr;  // 4.01 deprecated
    property Border: DOMString index atborder read GetStrAttr write SetStrAttr;
    property CellPadding: DOMString index atcellpadding read GetStrAttr write SetStrAttr;
    property CellSpacing: DOMString index atcellspacing read GetStrAttr write SetStrAttr;
    property Frame: DOMString index atframe read GetStrAttr write SetStrAttr;
    property Rules: DOMString index atrules read GetStrAttr write SetStrAttr;
    property Summary: DOMString index atsummary read GetStrAttr write SetStrAttr;
    property Width: DOMString index atwidth read GetStrAttr write SetStrAttr;
    function CreateTHead: THTMLElement;
    procedure DeleteTHead;
    function CreateTFoot: THTMLElement;
    procedure DeleteTFoot;
    function CreateCaption: THTMLElement;
    procedure DeleteCaption;
    function InsertRow(Index: Integer): THTMLElement;
    procedure DeleteRow(Index: Integer);
  end;

  THTMLTableCaptionElement = class(THTMLElement)
  public
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
  end;

  THTMLTableColElement = class(THTMLElement)
  public
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property Ch: DOMString index atchar read GetStrAttr write SetStrAttr;
    property ChOff: DOMString index atcharoff read GetStrAttr write SetStrAttr;
    property Span: Integer index atspan read GetIntAttr write SetIntAttr;
    property VAlign: DOMString index atvalign read GetStrAttr write SetStrAttr;
    property Width: DOMString index atwidth read GetStrAttr write SetStrAttr;
  end;

  THTMLTableSectionElement = class(THTMLElement)
  private
    function GetRows: THTMLCollection;
  public
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property Ch: DOMString index atchar read GetStrAttr write SetStrAttr;
    property ChOff: DOMString index atcharoff read GetStrAttr write SetStrAttr;
    property VAlign: DOMString index atvalign read GetStrAttr write SetStrAttr;
    property Rows: THTMLCollection read GetRows;
    function InsertRow(Index: Integer): THTMLElement;
    procedure DeleteRow(Index: Integer);
  end;

  THTMLTableRowElement = class(THTMLElement)
  private
    function GetRowIndex: Integer;
    function GetSectionRowIndex: Integer;
    function GetCells: THTMLCollection;
  public
    property RowIndex: Integer read GetRowIndex;
    property SectionRowIndex: Integer read GetSectionRowIndex;
    property Cells: THTMLCollection read GetCells;
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property BgColor: DOMString index atbgcolor read GetStrAttr write SetStrAttr;  // 4.01 deprecated
    property Ch: DOMString index atchar read GetStrAttr write SetStrAttr;
    property ChOff: DOMString index atcharoff read GetStrAttr write SetStrAttr;
    property VAlign: DOMString index atvalign read GetStrAttr write SetStrAttr;
    function InsertCell(Index: Integer): THTMLElement;
    procedure DeleteCell(Index: Integer);
  end;

  THTMLTableCellElement = class(THTMLElement)
  private
    function GetCellIndex: Integer;
  public
    property CellIndex: Integer read GetCellIndex;
    property Abbr: DOMString index atabbr read GetStrAttr write SetStrAttr;
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property Axis: DOMString index ataxis read GetStrAttr write SetStrAttr;
    property BgColor: DOMString index atbgcolor read GetStrAttr write SetStrAttr;  // 4.01 deprecated
    property Ch: DOMString index atchar read GetStrAttr write SetStrAttr;
    property ChOff: DOMString index atcharoff read GetStrAttr write SetStrAttr;
    property ColSpan: Integer index atcolspan read GetIntAttr write SetIntAttr;
    property Headers: DOMString index atheaders read GetStrAttr write SetStrAttr;
    property Height: DOMString index atheight read GetStrAttr write SetStrAttr;
    property NoWrap: Boolean index atnowrap read GetBoolAttr write SetBoolAttr;  // 4.01 deprecated
    property RowSpan: Integer index atrowspan read GetIntAttr write SetIntAttr;
    property Scope: DOMString index atscope read GetStrAttr write SetStrAttr;
    property VAlign: DOMString index atvalign read GetStrAttr write SetStrAttr;
    property Width: DOMString index atwidth read GetStrAttr write SetStrAttr;
  end;

  THTMLFrameSetElement = class(THTMLElement)
  public
    property Cols: DOMString index atcols read GetStrAttr write SetStrAttr;
    property Rows: DOMString index atrows read GetStrAttr write SetStrAttr;
  end;

  THTMLFrameElement = class(THTMLElement)
  private
    FContentDocument: TDOMDocument;  // !!! TEMP
  public
    property FrameBorder: DOMString index atframeborder  read GetStrAttr write SetStrAttr;
    property LongDesc: DOMString index atlongdesc read GetStrAttr write SetStrAttr;
    property MarginHeight: DOMString index atmarginheight read GetStrAttr write SetStrAttr;
    property MarginWidth: DOMString index atmarginwidth read GetStrAttr write SetStrAttr;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property NoResize: Boolean index atnoresize read GetBoolAttr write SetBoolAttr;
    property Scrolling: DOMString index atscrolling  read GetStrAttr write SetStrAttr;
    property Src: DOMString index atsrc read GetStrAttr write SetStrAttr;
    property ContentDocument: TDOMDocument read FContentDocument;
  end;

  THTMLIFrameElement = class(THTMLElement)
  private
    FContentDocument: TDOMDocument;  // !!! TEMP
  public
    property Align: DOMString index atalign read GetStrAttr write SetStrAttr;
    property FrameBorder: DOMString index atframeborder read GetStrAttr write SetStrAttr;
    property Height: DOMString index atheight read GetStrAttr write SetStrAttr;
    property LongDesc: DOMString index atlongdesc read GetStrAttr write SetStrAttr;
    property MarginHeight: DOMString index atmarginheight  read GetStrAttr write SetStrAttr;
    property MarginWidth: DOMString index atmarginwidth read GetStrAttr write SetStrAttr;
    property Name: DOMString index atname read GetStrAttr write SetStrAttr;
    property Scrolling: DOMString index atscrolling read GetStrAttr write SetStrAttr;
    property Src: DOMString index atsrc read GetStrAttr write SetStrAttr;
    property Width: DOMString index atwidth read GetStrAttr write SetStrAttr;
    property ContentDocument: TDOMDocument read FContentDocument;
  end;

  THTMLDocument = class(TXMLDocument)
  private
    function GetTitle: DOMString;
    procedure SetTitle(const Value: DOMString);
    function GetBody: THTMLElement;
    procedure SetBody(value: THTMLElement);
    function GetAnchors: THTMLCollection;
    function GetApplets: THTMLCollection;
    function GetCookie: DOMString;
    procedure SetCookie(const Value: DOMString);
    function GetDomain: DOMString;
    function GetForms: THTMLCollection;
    function GetImages: THTMLCollection;
    function GetLinks: THTMLCollection;
    function GetReferrer: DOMString;
  public
    property Title: DOMString read GetTitle write SetTitle;
    property Referrer: DOMString read GetReferrer;
    property Domain: DOMString read GetDomain;
    property URL: DOMString read FURI;
    property Body: THTMLElement read GetBody write SetBody;
    property Images: THTMLCollection read GetImages;
    property Applets: THTMLCollection read GetApplets;
    property Links: THTMLCollection read GetLinks;
    property Forms: THTMLCollection read GetForms;
    property Anchors: THTMLCollection read GetAnchors;
    property Cookie: DOMString read GetCookie write SetCookie;

    procedure Open;
    procedure Close;
    procedure Write(const AText: DOMString);
    procedure WriteLn(const AText: DOMString);
    function GetElementsByName(const ElementName: DOMString): TDOMNodeList;
    function HashForName(const aName: DOMString): PHashItem;

    // Helper functions (not in DOM standard):
    function CreateElement(const tagName: DOMString): THTMLElement;
    function CreateSubElement: THTMLElement;
    function CreateSupElement: THTMLElement;
    function CreateSpanElement: THTMLElement;
    function CreateBDOElement: THTMLElement;
    function CreateTTElement: THTMLElement;
    function CreateIElement: THTMLElement;
    function CreateBElement: THTMLElement;
    function CreateUElement: THTMLElement;
    function CreateSElement: THTMLElement;
    function CreateStrikeElement: THTMLElement;
    function CreateBigElement: THTMLElement;
    function CreateSmallElement: THTMLElement;
    function CreateEmElement: THTMLElement;
    function CreateStrongElement: THTMLElement;
    function CreateDfnElement: THTMLElement;
    function CreateCodeElement: THTMLElement;
    function CreateSampElement: THTMLElement;
    function CreateKbdElement: THTMLElement;
    function CreateVarElement: THTMLElement;
    function CreateCiteElement: THTMLElement;
    function CreateAcronymElement: THTMLElement;
    function CreateAbbrElement: THTMLElement;
    function CreateDDElement: THTMLElement;
    function CreateDTElement: THTMLElement;
    function CreateNoFramesElement: THTMLElement;
    function CreateNoScriptElement: THTMLElement;
    function CreateAddressElement: THTMLElement;
    function CreateCenterElement: THTMLElement;
    function CreateHtmlElement: THTMLHtmlElement;
    function CreateHeadElement: THTMLHeadElement;
    function CreateLinkElement: THTMLLinkElement;
{    function CreateTitleElement: THTMLTitleElement;
    function CreateMetaElement: THTMLMetaElement;
    function CreateBaseElement: THTMLBaseElement;
    function CreateIsIndexElement: THTMLIsIndexElement;
    function CreateStyleElement: THTMLStyleElement;}
    function CreateBodyElement: THTMLBodyElement;
{    function CreateFormElement: THTMLFormElement;
    function CreateSelectElement: THTMLSelectElement;
    function CreateOptGroupElement: THTMLOptGroupElement;
    function CreateOptionElement: THTMLOptionElement;
    function CreateInputElement: THTMLInputElement;
    function CreateTextAreaElement: THTMLTextAreaElement;
    function CreateButtonElement: THTMLButtonElement;
    function CreateLabelElement: THTMLLabelElement;
    function CreateFieldSetElement: THTMLFieldSetElement;
    function CreateLegendElement: THTMLLegendElement;}
    function CreateUListElement: THTMLUListElement;
    function CreateOListElement: THTMLOListElement;
    function CreateDListElement: THTMLDListElement;
{    function CreateDirectoryElement: THTMLDirectoryElement;
    function CreateMenuElement: THTMLMenuElement;}
    function CreateLIElement: THTMLLIElement;
{    function CreateDivElement: THTMLDivElement;}
    function CreateParagraphElement: THTMLParagraphElement;
{    function CreateHeadingElement: THTMLHeadingElement;
    function CreateQuoteElement: THTMLQuoteElement;
    function CreatePreElement: THTMLPreElement;
    function CreateBRElement: THTMLBreElement;
    function CreateBaseFontElement: THTMLBaseFontElement;
    function CreateFontElement: THTMFontLElement;
    function CreateHRElement: THTMLHREElement;
    function CreateModElement: THTMLModElement;
    function CreateAnchorElement: THTMLAnchorElement;
    function CreateImageElement: THTMLImageElement;
    function CreateObjectElement: THTMLObjectElement;
    function CreateParamElement: THTMLParamElement;
    function CreateAppletElement: THTMLAppletElement;
    function CreateMapElement: THTMLMapElement;
    function CreateAreaElement: THTMLAreaElement;
    function CreateScriptElement: THTMLScriptElement;
    function CreateTableElement: THTMLTableElement;
    function CreateTableCaptionElement: THTMLTableCaptionElement;
    function CreateTableColElement: THTMLTableColElement;
    function CreateTableSectionElement: THTMLTableSectionElement;
    function CreateTableRowElement: THTMLTableRowElement;
    function CreateTableCellElement: THTMLTableCellElement;
    function CreateFrameSetElement: THTMLFrameSetElement;
    function CreateFrameElement: THTMLFrameElement;
    function CreateIFrameElement: THTMLIFrameElement;}
  end;


implementation

{ THTMLCollection }

constructor THTMLCollection.Create(aNode: TDOMNode; Proc: TFilterProc);
begin
  inherited Create(aNode);
  FFilterProc := Proc;
end;

function THTMLCollection.NodeFilter(aNode: TDOMNode): TFilterResult;
begin
  Result := FFilterProc(aNode);
end;

function THTMLCollection.NamedItem(const Index: DOMString): TDOMNode;
var
  I: Cardinal;
begin
// Finds node with matching 'id' attribute
  for I := 0 to Length-1 do
  begin
    // TODO: could be improved when we're able to build ID table for HTML docs.
    Result := GetItem(I);
    if (Result.NodeType = ELEMENT_NODE) and (TDOMElement(Result)['id'] = Index) then
      Exit;
  end;
// HTML4.01: additionally search for a node with matching 'name' attr
  Result := nil;
end;


{ THTMLElement }

function THTMLElement.GetForm: THTMLFormElement;
var
  r: TDOMNode;
begin
// TODO: rewrite properly
  r := ParentNode;
  while Assigned(r) and (r.NodeName <> 'form') do
    r := r.ParentNode;
  result := THTMLFormElement(r);
end;

function THTMLElement.GetStrAttr(idx: THTMLAttributeTag): DOMString;
begin
// TODO: values of attributes that are value lists must be returned in lowercase
  result := GetAttribute(HTMLAttributeTag[idx]);
end;

procedure THTMLElement.SetStrAttr(idx: THTMLAttributeTag; const Value: DOMString);
begin
  SetAttribute(HTMLAttributeTag[idx], Value);
end;

function THTMLElement.GetIntAttr(idx: THTMLAttributeTag): Integer;
begin
  if not TryStrToInt(GetAttribute(HTMLAttributeTag[idx]), result) then
    result := 0;
end;

procedure THTMLElement.SetIntAttr(idx: THTMLAttributeTag; value: Integer);
begin
  SetAttribute(HTMLAttributeTag[idx], IntToStr(value));
end;

function THTMLElement.GetUIntAttr(idx: THTMLAttributeTag): Cardinal;
var
  tmp: DOMString;
  code: word;
begin
  tmp := GetAttribute(HTMLAttributeTag[idx]);
  val(tmp, result, code);
  if code <> 0 then
    result := 0;
end;

procedure THTMLElement.SetUIntAttr(idx: THTMLAttributeTag; value: Cardinal);
begin
  SetAttribute(HTMLAttributeTag[idx], IntToStr(value));
end;

function THTMLElement.GetBoolAttr(idx: THTMLAttributeTag): Boolean;
begin
// attribute value is irrelevant?
  result := HasAttribute(HTMLAttributeTag[idx]);
end;

procedure THTMLElement.SetBoolAttr(idx: THTMLAttributeTag; value: Boolean);
begin
  if value then
    SetAttribute(HTMLAttributeTag[idx], HTMLAttributeTag[idx])
  else
    RemoveAttribute(HTMLAttributeTag[idx]);
end;

{ THTMLTableElement }

function TableRowFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and (aNode.NodeName = 'tr') then
    Result := frNorecurseTrue
  else
    Result := frFalse;
end;

function TableBodyFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and (aNode.NodeName = 'tbody') then
    Result := frNorecurseTrue
  else
    Result := frFalse;
end;

function THTMLTableElement.GetRows: THTMLCollection;
begin
  result := THTMLCollection.Create(Self, @TableRowFilter);
end;

function THTMLTableElement.GetBodies: THTMLCollection;
begin
  result := THTMLCollection.Create(Self, @TableBodyFilter);
end;

function THTMLTableElement.GetCaption: THTMLTableCaptionElement;
var
  child: TDOMNode;
begin
  child := FirstChild;
  while Assigned(child) do
  begin
    if child.NodeName = 'caption' then
      Break;
    child := child.NextSibling;
  end;
  result := THTMLTableCaptionElement(child);
end;

procedure THTMLTableElement.SetCaption(value: THTMLTableCaptionElement);
begin
  // TODO: implement
end;

function THTMLTableElement.GetHead: THTMLTableSectionElement;
var
  child: TDOMNode;
begin
  child := FirstChild;
  while Assigned(child) do
  begin
    if child.NodeName = 'thead' then
      Break;
    child := child.NextSibling;
  end;
  result := THTMLTableSectionElement(child);
end;

procedure THTMLTableElement.SetHead(value: THTMLTableSectionElement);
begin
  // TODO: implement
end;

function THTMLTableElement.GetFoot: THTMLTableSectionElement;
var
  child: TDOMNode;
begin
  child := FirstChild;
  while Assigned(child) do
  begin
    if child.NodeName = 'tfoot' then
      Break;
    child := child.NextSibling;
  end;
  result := THTMLTableSectionElement(child);
end;

procedure THTMLTableElement.SetFoot(value: THTMLTableSectionElement);
begin
  // TODO: implement
end;

function THTMLTableElement.CreateTHead: THTMLElement;
begin
  Result := GetHead;
  if Assigned(Result) then
    Exit;
  Result := THTMLTableSectionElement.Create(OwnerDocument);
  Result.FNSI.QName := THTMLDocument(OwnerDocument).HashForName('thead');
  AppendChild(Result);
end;

procedure THTMLTableElement.DeleteTHead;
var
  node: TDOMNode;
begin
  node := GetHead;
  if Assigned(node) then
    RemoveChild(node);
end;

function THTMLTableElement.CreateTFoot: THTMLElement;
begin
  Result := GetFoot;
  if Assigned(Result) then
    Exit;
  Result := THTMLTableSectionElement.Create(OwnerDocument);
  Result.FNSI.QName := THTMLDocument(OwnerDocument).HashForName('tfoot');
  AppendChild(Result);
end;

procedure THTMLTableElement.DeleteTFoot;
var
  node: TDOMNode;
begin
  node := GetFoot;
  if Assigned(node) then
    RemoveChild(node);
end;

function THTMLTableElement.CreateCaption: THTMLElement;
begin
  Result := GetCaption;
  if Assigned(Result) then
    Exit;
  Result := THTMLTableCaptionElement.Create(OwnerDocument);
  Result.FNSI.QName := THTMLDocument(OwnerDocument).HashForName('caption');
  AppendChild(Result);
end;

procedure THTMLTableElement.DeleteCaption;
var
  node: TDOMNode;
begin
  node := GetCaption;
  if Assigned(node) then
    RemoveChild(node);
end;

function THTMLTableElement.InsertRow(Index: Integer): THTMLElement;
begin
{ Insert a new empty row in the table. The new row is inserted immediately
 before and in the same section as the current indexth row in the table.
  If index is -1 or equal to the number of rows, the new row is appended.
  In addition, when the table is empty the row is inserted into a TBODY which
  is created and inserted into the table. }
  Result := nil;
end;

procedure THTMLTableElement.DeleteRow(Index: Integer);
begin
end;

{ THTMLTableSectionElement }

function THTMLTableSectionElement.GetRows: THTMLCollection;
begin
  result := THTMLCollection.Create(Self, @TableRowFilter);
end;

function THTMLTableSectionElement.InsertRow(Index: Integer): THTMLElement;
var
  row: TDOMNode;
begin
  with GetRows do
  try
    if (Index < -1) or (Index > Integer(Length)) then
      raise EDOMIndexSize.Create('HTMLTableSectionElement.InsertRow');
    row := GetItem(LongWord(Index));  // may be nil
    Result := THTMLTableRowElement.Create(OwnerDocument);
    Result.FNSI.QName := THTMLDocument(OwnerDocument).HashForName('tr');
    InsertBefore(Result, row);
  finally
    Free;
  end;
end;

procedure THTMLTableSectionElement.DeleteRow(Index: Integer);
var
  row: TDOMNode;
begin
  with GetRows do
  try
    if Index = -1 then
      Index := Length-1;
    row := GetItem(LongWord(Index));
    if row = nil then
      raise EDOMIndexSize.Create('HTMLTableSectionElement.DeleteRow');
    RemoveChild(row);
  finally
    Free;
  end;
end;

{ THTMLDocument }

function THTMLDocument.GetTitle: DOMString;
var
  Node: TDOMNode;
begin
  Result := '';
  if not Assigned(DocumentElement) then
    exit;
  Node := DocumentElement.FirstChild;
  while Assigned(Node) and (Node.NodeName <> 'head') do
    Node := Node.NextSibling;
  if not Assigned(Node) then
    exit;
  Node := Node.FirstChild;
  while Assigned(Node) and (Node.NodeName <> 'title') do
    Node := Node.NextSibling;
  if not Assigned(Node) then
    exit;
  Node := Node.FirstChild;
  if Assigned(Node) and (Node.NodeType = TEXT_NODE) then
    Result := Node.NodeValue;
end;

procedure THTMLDocument.SetTitle(const Value: DOMString);
var
  Node: TDOMNode;
  TitleEl: TDOMElement;
begin
  if not Assigned(DocumentElement) then
    AppendChild(CreateHtmlElement);
  Node := DocumentElement.FirstChild;
  while Assigned(Node) and (Node.NodeName <> 'head') do
    Node := Node.NextSibling;
  if not Assigned(Node) then
  begin
    Node := CreateHeadElement;
    DocumentElement.InsertBefore(Node, DocumentElement.FirstChild);
  end;
  TitleEl := TDOMElement(Node.FirstChild);
  while Assigned(TitleEl) and (TitleEl.NodeName <> 'title') do
    TitleEl := TDOMElement(TitleEl.NextSibling);
  if not Assigned(TitleEl) then
  begin
    TitleEl := CreateElement('title');
    Node.AppendChild(TitleEl);
  end;
  while Assigned(TitleEl.FirstChild) do
    TitleEl.RemoveChild(TitleEl.FirstChild);
  TitleEl.AppendChild(CreateTextNode(Value));
end;

function THTMLDocument.GetBody: THTMLElement;
var
  Node: TDOMNode;
begin
  if not Assigned(DocumentElement) then
    AppendChild(CreateHtmlElement);
  Node := DocumentElement.FirstChild;
  while Assigned(Node) and (Node.NodeName <> 'body') do
    Node := Node.NextSibling;
  if not Assigned(Node) then
  begin
    Node := CreateBodyElement;
    DocumentElement.AppendChild(Node);
  end;
  result := THTMLElement(Node);
end;

procedure THTMLDocument.SetBody(value: THTMLElement);
begin

end;

function DocImageFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and (aNode.NodeName = 'img') then
    Result := frNorecurseTrue
  else
    Result := frFalse;
end;

function DocFormFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and (aNode.NodeName = 'form') then
    Result := frTrue
  else
    Result := frFalse;
end;

function DocAnchorFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and (aNode.NodeName = 'a') and
    TDOMElement(aNode).HasAttribute('name') then
    Result := frTrue
  else
    Result := frFalse;
end;

function DocAppletFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and ((aNode.NodeName = 'applet') or
    (aNode.NodeName = 'object')) then
    Result := frTrue
  else
    Result := frFalse;
end;

function DocLinksFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and ((aNode.NodeName = 'area') or
  ((aNode.NodeName = 'a') and TDOMElement(aNode).HasAttribute('href'))) then
    Result := frTrue
  else
    Result := frFalse;
end;

type
  TByNameNodeList = class(TDOMNodeList)
  protected
    FFilter: DOMString;
    function NodeFilter(aNode: TDOMNode): TFilterResult; override;
  public
    constructor Create(aNode: TDOMNode; const aFilter: DOMString);
  end;

constructor TByNameNodeList.Create(aNode: TDOMNode; const aFilter: DOMString);
begin
  inherited Create(aNode);
  FFilter := aFilter;
end;

function TByNameNodeList.NodeFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and (TDOMElement(aNode)['name'] = FFilter) then
    Result := frTrue
  else
    Result := frFalse;
end;

function THTMLDocument.GetAnchors: THTMLCollection;
begin
  Result := THTMLCollection.Create(Self, @DocAnchorFilter);
end;

function THTMLDocument.GetApplets: THTMLCollection;
begin
  Result := THTMLCollection.Create(Self, @DocAppletFilter);
end;

function THTMLDocument.GetForms: THTMLCollection;
begin
  Result := THTMLCollection.Create(Self, @DocFormFilter);
end;

function THTMLDocument.GetImages: THTMLCollection;
begin
  Result := THTMLCollection.Create(Self, @DocImageFilter);
end;

function THTMLDocument.GetLinks: THTMLCollection;
begin
  Result := THTMLCollection.Create(Self, @DocLinksFilter);
end;

function THTMLDocument.GetDomain: DOMString;
begin
  Result := '';
end;

function THTMLDocument.GetReferrer: DOMString;
begin
  Result := '';
end;

function THTMLDocument.GetCookie: DOMString;
begin
  Result := '';
end;

procedure THTMLDocument.SetCookie(const Value: DOMString);
begin
end;

procedure THTMLDocument.Open;
begin
end;

procedure THTMLDocument.Close;
begin
end;

procedure THTMLDocument.Write(const AText: DOMString);
begin
end;

procedure THTMLDocument.WriteLn(const AText: DOMString);
begin
end;

function THTMLDocument.GetElementsByName(const ElementName: DOMString): TDOMNodeList;
begin
  Result := TByNameNodeList.Create(Self, ElementName);
end;

function THTMLDocument.CreateElement(const tagName: DOMString): THTMLElement;
begin
  Result := THTMLElement.Create(Self);
  Result.FNSI.QName := FNames.FindOrAdd(DOMPChar(tagName), Length(tagName));
end;

function THTMLDocument.HashForName(const aName: DOMString): PHashItem;
begin
  Result := FNames.FindOrAdd(DOMPChar(aName), Length(aName));
end;

function THTMLDocument.CreateSubElement: THTMLElement; begin Result := CreateElement('sub') end;
function THTMLDocument.CreateSupElement: THTMLElement; begin Result := CreateElement('sup') end;
function THTMLDocument.CreateSpanElement: THTMLElement; begin Result := CreateElement('span') end;
function THTMLDocument.CreateBDOElement: THTMLElement; begin Result := CreateElement('bdo') end;
function THTMLDocument.CreateTTElement: THTMLElement; begin Result := CreateElement('tt') end;
function THTMLDocument.CreateIElement: THTMLElement; begin Result := CreateElement('i') end;
function THTMLDocument.CreateBElement: THTMLElement; begin Result := CreateElement('b') end;
function THTMLDocument.CreateUElement: THTMLElement; begin Result := CreateElement('u') end;
function THTMLDocument.CreateSElement: THTMLElement; begin Result := CreateElement('s') end;
function THTMLDocument.CreateStrikeElement: THTMLElement; begin Result := CreateElement('strike') end;
function THTMLDocument.CreateBigElement: THTMLElement; begin Result := CreateElement('big') end;
function THTMLDocument.CreateSmallElement: THTMLElement; begin Result := CreateElement('small') end;
function THTMLDocument.CreateEmElement: THTMLElement; begin Result := CreateElement('em') end;
function THTMLDocument.CreateStrongElement: THTMLElement; begin Result := CreateElement('strong') end;
function THTMLDocument.CreateDfnElement: THTMLElement; begin Result := CreateElement('dfn') end;
function THTMLDocument.CreateCodeElement: THTMLElement; begin Result := CreateElement('code') end;
function THTMLDocument.CreateSampElement: THTMLElement; begin Result := CreateElement('samp') end;
function THTMLDocument.CreateKbdElement: THTMLElement; begin Result := CreateElement('kbd') end;
function THTMLDocument.CreateVarElement: THTMLElement; begin Result := CreateElement('var') end;
function THTMLDocument.CreateCiteElement: THTMLElement; begin Result := CreateElement('cite') end;
function THTMLDocument.CreateAcronymElement: THTMLElement; begin Result := CreateElement('acronym') end;
function THTMLDocument.CreateAbbrElement: THTMLElement; begin Result := CreateElement('abbr') end;
function THTMLDocument.CreateDDElement: THTMLElement; begin Result := CreateElement('dd') end;
function THTMLDocument.CreateDTElement: THTMLElement; begin Result := CreateElement('dt') end;
function THTMLDocument.CreateNoFramesElement: THTMLElement; begin Result := CreateElement('noframes') end;
function THTMLDocument.CreateNoScriptElement: THTMLElement; begin Result := CreateElement('noscript') end;
function THTMLDocument.CreateAddressElement: THTMLElement; begin Result := CreateElement('address') end;
function THTMLDocument.CreateCenterElement: THTMLElement; begin Result := CreateElement('center') end;

function THTMLDocument.CreateHtmlElement: THTMLHtmlElement;
begin
  Result := THTMLHtmlElement.Create(Self);
  Result.FNSI.QName := HashForName('html');
end;

function THTMLDocument.CreateHeadElement: THTMLHeadElement;
begin
  Result := THTMLHeadElement.Create(Self);
  Result.FNSI.QName := HashForName('head');
end;

function THTMLDocument.CreateLinkElement: THTMLLinkElement;
begin
  Result := THTMLLinkElement.Create(Self);
  Result.FNSI.QName := HashForName('a');
end;

function THTMLDocument.CreateBodyElement: THTMLBodyElement;
begin
  Result := THTMLBodyElement.Create(Self);
  Result.FNSI.QName := HashForName('body');
end;

function THTMLDocument.CreateUListElement: THTMLUListElement;
begin
  Result := THTMLUListElement.Create(Self);
  Result.FNSI.QName := HashForName('ul');
end;

function THTMLDocument.CreateOListElement: THTMLOListElement;
begin
  Result := THTMLOListElement.Create(Self);
  Result.FNSI.QName := HashForName('ol');
end;

function THTMLDocument.CreateDListElement: THTMLDListElement;
begin
  Result := THTMLDListElement.Create(Self);
  Result.FNSI.QName := HashForName('dl');
end;

function THTMLDocument.CreateLIElement: THTMLLIElement;
begin
  Result := THTMLLIElement.Create(Self);
  Result.FNSI.QName := HashForName('li');
end;
//...
function THTMLDocument.CreateParagraphElement: THTMLParagraphElement;
begin
  Result := THTMLParagraphElement.Create(Self);
  Result.FNSI.QName := HashForName('p');
end;

{ THTMLFormElement }

function FormElementsFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and ((aNode.NodeName = 'input') or
    (aNode.NodeName = 'select') or (aNode.NodeName = 'textarea') or
    (aNode.NodeName = 'label') or (aNode.NodeName = 'button')) then
    Result := frTrue
  else
    Result := frFalse;
end;

function THTMLFormElement.GetElements: THTMLCollection;
begin
  result := THTMLCollection.Create(Self, @FormElementsFilter);
end;

function THTMLFormElement.GetLength: Integer;
begin
  with GetElements do
  try
    Result := Length;
  finally
    Free;
  end;
end;

procedure THTMLFormElement.Submit;
begin
end;

procedure THTMLFormElement.Reset;
begin
end;

{ THTMLMapElement }

function MapAreasFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and (aNode.NodeName = 'area') then
    Result := frTrue
  else
    Result := frFalse;
end;

function THTMLMapElement.GetAreas: THTMLCollection;
begin
  result := THTMLCollection.Create(Self, @MapAreasFilter);
end;

{ THTMLSelectElement }

function SelectOptionFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and (aNode.NodeName = 'option') then
    Result := frTrue
  else
    Result := frFalse;
end;

function THTMLSelectElement.GetType: DOMString;
begin
  if HasAttribute(HTMLAttributeTag[atmultiple]) then
    result := 'select-multiple'
  else
    result := 'select-one';
end;

function THTMLSelectElement.GetValue: DOMString;
begin
  result := '';
end;

procedure THTMLSelectElement.SetValue(const value: DOMString);
begin

end;

function THTMLSelectElement.GetOptions: THTMLOptionsCollection;
begin
  result := THTMLOptionsCollection.Create(Self, @SelectOptionFilter);
end;

procedure THTMLSelectElement.Add(Element, Before: THTMLElement);
begin
end;

procedure THTMLSelectElement.Remove(Index: Integer);
begin
end;

procedure THTMLSelectElement.Blur;
begin
end;

procedure THTMLSelectElement.Focus;
begin
end;

function THTMLSelectElement.GetLength: Cardinal;
begin
  with GetOptions do
  try
    Result := Length;
  finally
    Free;
  end;
end;

procedure THTMLSelectElement.SetLength(aValue: Cardinal);
begin
  raise EDOMNotSupported.Create('HTMLSelectElement.SetLength');
end;

{ THTMLTableRowElement }

function RowCellsFilter(aNode: TDOMNode): TFilterResult;
begin
  if (aNode.NodeType = ELEMENT_NODE) and
   ((aNode.NodeName = 'td') or (aNode.NodeName = 'th')) then
    Result := frNorecurseTrue
  else
    Result := frFalse;
end;

function THTMLTableRowElement.GetCells: THTMLCollection;
begin
  result := THTMLCollection.Create(Self, @RowCellsFilter);
end;

function THTMLTableRowElement.GetRowIndex: Integer;
begin
{ result := SectionRowIndex;
  if parent is 'tbody' then inc(result, table.head.rowcount);
  if parent is 'tfoot' then inc(result, table.bodies.rowcount);
  // what about multiple bodies?
}
  result := -1;
end;

function THTMLTableRowElement.GetSectionRowIndex: Integer;
var
  node: TDOMNode;
begin
  result := 0;
  node := PreviousSibling;
  while Assigned(node) do
  begin
  // TODO: should we also check whether the name is same?
    if node.NodeType = ELEMENT_NODE then
      Inc(result);
    node := node.PreviousSibling;
  end;
end;

function THTMLTableRowElement.InsertCell(Index: Integer): THTMLElement;
var
  cell: TDOMNode;
begin
  with GetCells do
  try
    if (Index < -1) or (Index > Integer(Length)) then
      raise EDOMIndexSize.Create('HTMLTableRowElement.InsertCell');
    cell := GetItem(LongWord(Index));  // may be nil
    Result := THTMLTableCellElement.Create(OwnerDocument);
    Result.FNSI.QName := THTMLDocument(OwnerDocument).HashForName('td');
    InsertBefore(Result, cell);
  finally
    Free;
  end;
end;

procedure THTMLTableRowElement.DeleteCell(Index: Integer);
var
  cell: TDOMNode;
begin
  with GetCells do
  try
    if Index = -1 then
      Index := Length-1;
    cell := GetItem(LongWord(Index));
    if cell = nil then
      raise EDOMIndexSize.Create('HTMLTableRowElement.DeleteCell');
    RemoveChild(cell);
  finally
    Free;
  end;
end;

{ THTMLTableCellElement }

function THTMLTableCellElement.GetCellIndex: Integer;
var
  node: TDOMNode;
begin
  result := 0;
  node := PreviousSibling;
  while Assigned(node) do
  begin
  // TODO: should we also check whether the name is same?
    if node.NodeType = ELEMENT_NODE then
      Inc(result);
    node := node.PreviousSibling;
  end;

end;

{ THTMLOptionElement }

// TODO: probably should account for possible OPTGROUP elements (no tests)
function THTMLOptionElement.GetIndex: Integer;
var
  node: TDOMNode;
begin
  result := 0;
  node := PreviousSibling;
  while Assigned(node) do
  begin
  // TODO: should we also check whether the name is same?
    if node.NodeType = ELEMENT_NODE then
      Inc(result);
    node := node.PreviousSibling;
  end;
end;

{ THTMLTextAreaElement }

function THTMLTextAreaElement.GetType: DOMString;
begin
  result := 'textarea';
end;

procedure THTMLTextAreaElement.Blur;
begin
end;

procedure THTMLTextAreaElement.Focus;
begin
end;

procedure THTMLTextAreaElement.Select;
begin
end;

{ THTMLInputElement }

procedure THTMLInputElement.Blur;
begin
end;

procedure THTMLInputElement.Focus;
begin
end;

procedure THTMLInputElement.Select;
begin
end;

procedure THTMLInputElement.Click;
begin
end;

{ THTMLAnchorElement }

procedure THTMLAnchorElement.Blur;
begin
end;

procedure THTMLAnchorElement.Focus;
begin
end;

{ THTMLObjectElement }

function THTMLObjectElement.GetContentDocument: TDOMDocument;
begin
  result := nil;
end;

{ THTMLScriptElement }

function THTMLScriptElement.GetEvent: DOMString;
begin
  Result := '';
end;

procedure THTMLScriptElement.SetEvent(const Value: DOMString);
begin
end;

end.
