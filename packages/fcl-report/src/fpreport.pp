{
    This file is part of the Free Component Library.
    Copyright (c) 2008 Michael Van Canneyt, member of the Free Pascal development team
    Portions (c) 2016 WISA b.v.b.a.

    GUI independent reporting engine core

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit fpreport;

{$mode objfpc}{$H+}
{$inline on}

// Global debugging
{ $define gdebug}
// Separate for aggregate variables
{ $define gdebuga}

interface

uses
  Classes,
  SysUtils,
  Variants,
  contnrs,
  fpCanvas,
  fpImage,
  fpTTF,
  fpreportstreamer,
{$IF FPC_FULLVERSION>=30101}
  fpexprpars,
{$ELSE}
  fprepexprpars,
{$ENDIF}
  fpReportHTMLParser;

type

  // Do not use other types than the ones below in fpreport.
  TFPReportString = string;
  TFPReportUnits  = single; // Units are defined as Millimetres
  TFPReportScale  = single;
  TFPReportColor  = type UInt32;
  TFPReportCullThreshold = 1..100;

  // A position in report units
  TFPReportPoint = record
    Top: TFPReportUnits;
    Left: TFPReportUnits;
  end;

  // A rectangle in report units (measures)

  { TFPReportRect }

  TFPReportRect = object  // not a class for static allocations. Not a record because we want methods
    Top: TFPReportUnits;
    Left: TFPReportUnits;
    Width: TFPReportUnits;
    Height: TFPReportUnits;
    procedure SetRect(aleft, atop, awidth, aheight: TFPReportUnits);
    Procedure OffsetRect(aLeft,ATop : TFPReportUnits);
    Function IsEmpty : Boolean;
    function Bottom: TFPReportUnits;
    function Right: TFPReportUnits;
    function AsString : String;
  end;


  // Scaling factors (mostly for zoom/resize)
  TFPReportScales = record
    Vertical: TFPreportScale;
    Horizontal: TFPreportScale;
  end;


  // Forward declarations
  TFPReportElement        = class;
  TFPCustomReport         = class;
  TFPReportCustomBand     = class;
  TFPReportCustomPage     = class;
  TFPReportCustomGroupFooterBand = class;
  TFPReportData           = class;
  TFPReportFrame          = class;
  TFPReportCustomMemo     = class;
  TFPReportChildBand      = class;
  TFPReportCustomDataBand = class;
  TFPReportCustomChildBand = Class;
  TFPReportCustomDataHeaderBand = class;
  TFPReportCustomDataFooterBand = class;
  TFPReportCustomGroupHeaderBand = class;
  TFPReportExporter       = class;
  TFPReportTextAlignment  = class;
  TFPReportLayouter       = Class;
  TFPReportClassMapping   = Class;

  TBandList = class;

  TFPReportElementClass   = class of TFPReportElement;
  TFPReportBandClass      = class of TFPReportCustomBand;

  TFPReportState          = (rsDesign, rsLayout, rsRender);
  TFPReportPaperOrientation = (poPortrait, poLandscape);
  TFPReportVertTextAlignment = (tlTop, tlCenter, tlBottom);
  TFPReportHorzTextAlignment = (taLeftJustified, taRightJustified, taCentered, taWidth);
  TFPReportShapeType      = (stEllipse, stCircle, stLine, stSquare, stTriangle, stRoundedRect{, stArrow});  // rectangle can be handled by Frame
  TFPReportOrientation    = (orNorth, orNorthEast, orEast, orSouthEast, orSouth, orSouthWest, orWest, orNorthWest);
  TFPReportFrameLine      = (flTop, flBottom, flLeft, flRight);
  TFPReportFrameLines     = set of TFPReportFrameLine;
  TFPReportFrameShape     = (fsNone, fsRectangle, fsRoundedRect, fsDoubleRect, fsShadow);
  TFPReportFieldKind      = (rfkString, rfkBoolean, rfkInteger, rfkFloat, rfkDateTime, rfkStream, rfkCurrency);
  TFPReportStretchMode    = (smDontStretch, smActualHeight, smActualHeightStretchOnly, smActualHeightShrinkOnly, smMaxHeight);
  TFPReportHTMLTag        = (htRegular, htBold, htItalic);
  TFPReportHTMLTagSet     = set of TFPReportHTMLTag;
  TFPReportColumnLayout   = (clVertical, clHorizontal);
  TFPReportBandPosition   = (bpNormal, bpStackAtBottom);
  TFPReportSection        = (rsNone, rsPage, rsColumn);
  TFPReportVisibleOnPage  = (vpAll, vpFirstOnly, vpLastOnly, vpFirstAndLastOnly, vpNotOnFirst, vpNotOnLast, vpNotOnFirstAndLast);
  TFPReportBandType       = (btUnknown,btPageHeader,btReportTitle,btColumnHeader,btDataHeader,btGroupHeader,btDataband,btGroupFooter,
                             btDataFooter,btColumnFooter,btReportSummary,btPageFooter,btChild);
  TFPReportBandTypes = Set of TFPReportBandType;

  TFPReportBandMultiplicity = (bmUnrestricted,bmOncePerPage,bmOncePerDataloop);
  TFPReportBandMultiplicities = Set of TFPReportBandMultiplicity;

  TFPReportMemoOption     = (
            moSuppressRepeated,
            moHideZeros,
            moDisableExpressions,
            moAllowHTML,
            moDisableWordWrap,
            moNoResetAggregateOnPrint,
            moResetAggregateOnGroup,
            moResetAggregateOnPage,
            moResetAggregateOnColumn
            );
  TFPReportMemoOptions    = set of TFPReportMemoOption;
  TFPReportWordOverflow = (woTruncate, // truncate the word
                           woOverflow, // Allow to overflow
                           woSplit,    // Split word at max char count that fits length.
                           woAsterisk,  // Replace word with * chars.
                           woEllipsis  // Truncate word, add ... ellipsis.
                           );

  TFPReportSections    = set of rsPage..rsColumn;

  TFPReportResetType      = (
            rtNone,
            rtGroup,
            rtPage,
            rtColumn
            );

const
  { The format is always RRGGBB (Red, Green, Blue) - no alpha channel }
  clNone          = TFPReportColor($80000000);  // a special condition: $80 00 00 00
  { commonly known colors }
  clAqua          = TFPReportColor($00FFFF);
  clBlack         = TFPReportColor($000000);
  clBlue          = TFPReportColor($0000FF);
  clCream         = TFPReportColor($FFFBF0);
  clDkGray        = TFPReportColor($A9A9A9);
  clFuchsia       = TFPReportColor($FF00FF);
  clGray          = TFPReportColor($808080);
  clGreen         = TFPReportColor($008000);
  clLime          = TFPReportColor($00FF00);
  clLtGray        = TFPReportColor($C0C0C0);
  clMaroon        = TFPReportColor($800000);
  clNavy          = TFPReportColor($000080);
  clOlive         = TFPReportColor($808000);
  clPurple        = TFPReportColor($800080);
  clRed           = TFPReportColor($FF0000);
  clDkRed         = TFPReportColor($C00000);
  clSilver        = TFPReportColor($C0C0C0);
  clTeal          = TFPReportColor($008080);
  clWhite         = TFPReportColor($FFFFFF);
  clYellow        = TFPReportColor($FFFF00);
  { some common alias colors }
  clCyan          = clAqua;
  clMagenta       = clFuchsia;

const
  { Some color constants used throughout the demos, designer and documentation. }
  clPageHeaderFooter    = TFPReportColor($E4E4E4);
  clReportTitleSummary  = TFPReportColor($63CF80);
  clGroupHeaderFooter   = TFPReportColor($FFF1D7);
  clColumnHeaderFooter  = TFPReportColor($FF8E62);
  clDataHeaderFooter    = TFPReportColor($CBD5EC);
  clDataBand            = TFPReportColor($89B7EA);
  clChildBand           = TFPReportColor($B4DFFF);


  DefaultBandColors : Array[TFPReportBandType] of TFPReportColor = (
    clNone,                 // Unknown
    clPageHeaderFooter,     // Page header
    clReportTitleSummary,   // Report Title
    clColumnHeaderFooter,   // Column header
    clDataHeaderFooter,     // Data header
    clGroupHeaderFooter,    // Group header
    clDataBand,             // Databand
    clGroupHeaderFooter,    // Group footer
    clDataHeaderFooter,     // Data footer
    clColumnHeaderFooter,   // Column footer
    clReportTitleSummary,   // Report summary
    clPageHeaderFooter,     // Page footer
    clChildBand             // Child
  );

  clDarkMoneyGreen = TFPReportColor($A0BCA0);

  { These are default values, but replaced with darker version of DefaultBandColors[] }
  DefaultBandRectangleColors : Array[TFPReportBandType] of TFPReportColor = (
    clNone,              // Unknown
    clDarkMoneyGreen,    // Page header
    cldkGray,            // Report Title
    clDarkMoneyGreen,    // Column header
    clDarkMoneyGreen,    // Data header
    clDarkMoneyGreen,    // Group header
    clBlue,              // Databand
    clDarkMoneyGreen,    // Group footer
    clDarkMoneyGreen,    // Data footer
    clDarkMoneyGreen,    // Column footer
    clDarkMoneyGreen,    // Report summary
    clDarkMoneyGreen,    // Page footer
    clDkGray             // Child
  );

   ReportFieldKindNames : Array[TFPReportFieldKind] of string
      = ('String', 'Boolean', 'Integer', 'Float', 'DateTime', 'Stream', 'Currency');

     {btUnknown,btPageHeader,btReportTitle,btColumnHeader,
      btDataHeader,btGroupHeader,btDataband,btGroupFooter,
      btDataFooter,btColumnFooter,btReportSummary,btPageFooter,
      btChild}

  FPReportBandMultiplicity : Array[TFPReportBandType] of TFPReportBandMultiplicity
    = (bmUnrestricted,bmOncePerPage,bmOncePerPage,bmOncePerPage,
       bmOncePerDataloop,bmUnrestricted,bmOncePerDataloop,bmUnrestricted,
       bmOncePerDataloop,bmOncePerPage,bmOncePerPage,bmOncePerPage,
       bmUnrestricted);

const
  cMMperInch = 25.4;
  cCMperInch = 2.54;
  cMMperCM = 10;
  cEllipsis : UnicodeChar = #$2026; // '…';
  DefaultBandNames : Array[TFPReportBandType] of string
    = ('Unknown','Page Header','Report Title','Column Header', 'Data Header','Group Header','Data','Group Footer',
       'Data Footer','Column Footer','Report Summary','PageFooter','Child');

type
  // Event handlers
  TFPReportGetEOFEvent      = procedure(Sender: TObject; var IsEOF: boolean) of object;
  TFPReportGetValueEvent    = procedure(Sender: TObject; const AValueName: string; var AValue: variant) of object;
  TFPReportBeginReportEvent = procedure of object;
  TFPReportEndReportEvent   = procedure of object;
  TFPReportGetValueNamesEvent = procedure(Sender: TObject; List: TStrings) of object;
  TFPReportGetFieldKindEvent = Procedure(Sender: TObject; aName : String; var AKind : TFPReportFieldKind) of object;
  TFPReportBeforePrintEvent = procedure(Sender: TFPReportElement) of object;
  TFPReportQueryUsePrevValue = function: Boolean of object;


  TFPReportExporterConfigHandler = Procedure (Sender : TObject; AExporter : TFPReportExporter; var Cancelled : Boolean) of object;

  { TFPReportExporter }

  TFPReportExporter = class(TComponent)
  private
    FAutoRun: Boolean;
    FBaseFileName: string;
    FPReport: TFPCustomReport;
    procedure SetFPReport(AValue: TFPCustomReport);
  protected
    procedure SetBaseFileName(AValue: string); virtual;
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    procedure DoExecute(const ARTObjects: TFPList); virtual; abstract;
    // Override this to render an image on the indicated position. If AImage is non-nil on return, it will be freed by caller.
    Procedure RenderImage(aPos : TFPReportRect; var AImage: TFPCustomImage) ; virtual;
    Procedure RenderUnknownElement(aBasePos : TFPReportPoint; AElement : TFPReportElement; ADPI : Integer);
    Class function DefaultConfig : TFPReportExporterConfigHandler; virtual;
  public
    procedure Execute;
    // Descendents can treat this as a hint to set the filename.
    Procedure SetFileName(Const aFileName : String);virtual;
    Class Procedure RegisterExporter;
    Class Procedure UnRegisterExporter;
    Class Function Description : String; virtual;
    Class Function Name : String; virtual;
    // DefaultExtension should return non-empty if output is file based.
    // Must contain .
    Class Function DefaultExtension : String; virtual;
    Class Function MultiFile : Boolean; virtual;
    Function ShowConfig : Boolean;
  Published
    Property AutoRun : Boolean Read FAutoRun Write FAutoRun;
    property Report: TFPCustomReport read FPReport write SetFPReport;
  end;
  TFPReportExporterClass = Class of TFPReportExporter;


  // Width & Height are in portrait position, units: millimetres.
  TFPReportPaperSize = class(TObject)
  private
    FWidth: TFPReportUnits;
    FHeight: TFPReportUnits;
  public
    constructor Create(const AWidth, AHeight: TFPReportUnits);
    property Width: TFPReportUnits read FWidth;
    property Height: TFPReportUnits read FHeight;
  end;


  { TFPReportFont }

  TFPReportFont = class(TPersistent)
  private
    FOnChanged: TNotifyEvent;
  private
    FFontName: string;
    FFontSize: integer;
    FFontColor: TFPReportColor;
    procedure   SetFontName(const avalue: string);
    procedure   SetFontSize(const avalue: integer);
    procedure   SetFontColor(const avalue: TFPReportColor);
    Procedure Changed;
    Property OnChanged : TNotifyEvent Read FOnChanged Write FOnChanged;
  public
    constructor Create; virtual;
    procedure   Assign(Source: TPersistent); override;
  Published
    property    Name: string read FFontName write SetFontName;
    { value is in font Point units }
    property    Size: integer read FFontSize write SetFontSize default 10;
    property    Color: TFPReportColor read FFontColor write SetFontColor default clBlack;
  end;


  { TFPReportComponent }

  TFPReportComponent = class(TComponent)
  private
    FReportState: TFPReportState;
  protected
    // called when the layouter starts its job on the report.
    procedure StartLayout; virtual;
    // called when the layouter ends its job on the report.
    procedure EndLayout; virtual;
    // called when the renderer starts its job on the report.
    procedure StartRender; virtual;
    // called when the renderer ends its job on the report.
    procedure EndRender; virtual;
  Protected
    Procedure FixupReference(Const PN,PV : String; C : TFPReportElement); virtual;
  public
    Function AllocateName : String;
    // Called when done reading
    procedure WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); virtual;
    procedure ReadElement(AReader: TFPReportStreamer); virtual;
    // called when the designer starts editing this component .
    Procedure StartDesigning; virtual;
    // called when the designer ends editing this component .
    Procedure EndDesigning; virtual;
    property ReportState: TFPReportState read FReportState;
  end;




  // The Memo text is prepared as one or more TFPTextBlock objects for the report renderers
  TFPTextBlock = class(TObject)
  public
    Pos: TFPReportPoint;
    Width: TFPReportUnits;
    Height: TFPReportUnits;
    Descender: TFPReportUnits;
    Text: TFPReportString;
    FontName: string;
    FGColor: TFPReportColor;
    BGColor: TFPReportColor;
  end;

  // Extension of TFPTextBlock with support to hold URL information
  TFPHTTPTextBlock = class(TFPTextBlock)
  private
    FURL: String;
  public
    property URL: string read FURL write FURL;
  end;


  TFPTextBlockList = class(TFPObjectList)
  protected
    function GetItem(AIndex: Integer): TFPTextBlock; reintroduce;
    procedure SetItem(AIndex: Integer; AObject: TFPTextBlock); reintroduce;
  public
    property Items[AIndex: Integer]: TFPTextBlock read GetItem write SetItem; default;
  end;


  { TFPReportDataField }

  TFPReportDataField = class(TCollectionItem)
  private
    FDisplayWidth: integer;
    FFieldKind: TFPReportFieldKind;
    FFieldName: string;
    FValue: variant;
    FPrevValue: variant;
    FOnGetUsePrevValue: TFPReportQueryUsePrevValue;
    FExprIdentierDef: TFPExprIdentifierDef;
  Protected
    Procedure InitValue(SavePrevious : Boolean); virtual;
  public
    property OnGetUsePrevValue: TFPReportQueryUsePrevValue read FOnGetUsePrevValue write FOnGetUsePrevValue;
    property ExprIdentierDef: TFPExprIdentifierDef read FExprIdentierDef write FExprIdentierDef;
    procedure GetRTValue(Var Result : TFPExpressionResult; ConstRef AName : ShortString);
    function GetValue: variant; virtual;
    procedure Assign(Source: TPersistent); override;
  published
    property FieldName: string read FFieldName write FFieldName;
    property FieldKind: TFPReportFieldKind read FFieldKind write FFieldKind;
    property DisplayWidth: integer read FDisplayWidth write FDisplayWidth;
  end;


  TFPReportDataFields = class(TCollection)
  private
    FReportData: TFPReportData;
    function GetF(AIndex: integer): TFPReportDataField;
    procedure SetF(AIndex: integer; const AValue: TFPReportDataField);
  public
    function AddField(AFieldName: string; AFieldKind: TFPReportFieldKind): TFPReportDataField;
    function IndexOfField(const AFieldName: string): integer;
    function FindField(const AFieldName: string): TFPReportDataField; overload;
    function FindField(const AFieldName: string; const AFieldKind: TFPReportFieldKind): TFPReportDataField; overload;
    function FieldByName(const AFieldName: string): TFPReportDataField;
    property ReportData: TFPReportData read FReportData;
    property Fields[AIndex: integer]: TFPReportDataField read GetF write SetF; default;
  end;


  { TFPReportData }

  TFPReportData = class(TFPReportComponent)
  private
    FDataFields: TFPReportDataFields;
    FOnClose: TNotifyEvent;
    FOnFirst: TNotifyEvent;
    FOnGetEOF: TFPReportGetEOFEvent;
    FOnNext: TNotifyEvent;
    FOnOpen: TNotifyEvent;
    FRecNo: integer;
    FIsOpened: boolean; // tracking the state
    function GetFieldCount: integer;
    function GetFieldName(Index: integer): string;
    function GetFieldType(AFieldName: string): TFPReportFieldKind;
    function GetFieldValue(AFieldName: string): variant;
    function GetFieldWidth(AFieldName: string): integer;
    function GetLastFieldValue(AFieldName: string): variant;
    procedure InitFieldValues(SavePrevious: Boolean);
    procedure SetDataFields(const AValue: TFPReportDataFields);
  protected
    function CreateDataFields: TFPReportDataFields; virtual;
    procedure DoGetValue(const AFieldName: string; var AValue: variant); virtual;
    // Fill Datafields Collection. Should not change after Open.
    function GetIsOpened: boolean; virtual;
    procedure DoInitDataFields; virtual;
    procedure DoOpen; virtual;
    procedure DoFirst; virtual;
    procedure DoNext; virtual;
    procedure DoClose; virtual;
    function  DoEOF: boolean; virtual;
    property DataFields: TFPReportDataFields read FDataFields write SetDataFields;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    // Navigation
    procedure InitFieldDefs;
    procedure Open;
    procedure First;
    procedure Next;
    procedure Close;
    function EOF: boolean;
    //  Public access methods
    procedure GetFieldList(List: TStrings);
    Function IndexOfField (const AFieldName: string): Integer;
    function HasField(const AFieldName: string): boolean;
    property FieldNames[Index: integer]: string read GetFieldName;
    property FieldValues[AFieldName: string]: variant read GetFieldValue; default;
    property LastFieldValues[AFieldName: string]: variant read GetLastFieldValue;
    property FieldWidths[AFieldName: string]: integer read GetFieldWidth;
    property FieldTypes[AFieldName: string]: TFPReportFieldKind read GetFieldType;
    property FieldCount: integer read GetFieldCount;
    property RecNo: integer read FRecNo;
    property IsOpened: boolean read GetIsOpened;
  published
    property OnOpen: TNotifyEvent read FOnOpen write FOnOpen;
    property OnClose: TNotifyEvent read FOnClose write FOnClose;
    property OnFirst: TNotifyEvent read FOnFirst write FOnFirst;
    property OnNext: TNotifyEvent read FOnNext write FOnNext;
    property OnGetEOF: TFPReportGetEOFEvent read FOnGetEOF write FOnGetEOF;
  end;


  { TFPReportUserData }

  TFPReportUserData = class(TFPReportData)
  private
    FOnGetFieldKind: TFPReportGetFieldKindEvent;
    FOnGetValue: TFPReportGetValueEvent;
    FOnGetNames: TFPReportGetValueNamesEvent;
  protected
    function GetFieldDataType(const AFieldName: String): TFPReportFieldKind; virtual;
    procedure DoGetValue(const AFieldName: string; var AValue: variant); override;
    procedure DoInitDataFields; override;
  published
    property DataFields;
    property OnGetValue: TFPReportGetValueEvent read FOnGetValue write FOnGetValue;
    property OnGetNames: TFPReportGetValueNamesEvent read FOnGetNames write FOnGetNames;
    property OnGetFieldKind: TFPReportGetFieldKindEvent read FOnGetFieldKind write FOnGetFieldKind;
  end;

  { TFPReportDataItem }

  TFPReportDataItem = Class(TCollectionItem)
  private
    FData: TFPReportData;
    procedure SetData(AValue: TFPReportData);
  Protected
    Function GetDisplayName: string; override;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    property Data : TFPReportData Read FData Write SetData;
  end;


  { TFPReportDataCollection }

  TFPReportDataCollection = Class(TCollection)
  private
    function GetData(AIndex : Integer): TFPReportDataItem;
    procedure SetData(AIndex : Integer; AValue: TFPReportDataItem);
  Public
    Function IndexOfReportData(AData : TFPReportData) : Integer;
    Function IndexOfReportData(Const ADataName : String) : Integer;
    Function FindReportDataItem(AData : TFPReportData) : TFPReportDataItem;
    Function FindReportDataItem(Const ADataName : String) : TFPReportDataItem;
    Function FindReportData(Const ADataName : String) : TFPReportData;
    function AddReportData(AData: TFPReportData): TFPReportDataItem;
    Property Data[AIndex : Integer] : TFPReportDataItem Read GetData Write SetData; default;
  end;

  // The frame around each printable element.
  TFPReportFrame = class(TPersistent)
  private
    FColor: TFPReportColor;
    FFrameLines: TFPReportFrameLines;
    FFrameShape: TFPReportFrameShape;
    FPenStyle: TFPPenStyle;
    FReportElement: TFPReportElement;
    FWidth: integer;
    FBackgroundColor: TFPReportColor;
    procedure SetColor(const AValue: TFPReportColor);
    procedure SetFrameLines(const AValue: TFPReportFrameLines);
    procedure SetFrameShape(const AValue: TFPReportFrameShape);
    procedure SetPenStyle(const AValue: TFPPenStyle);
    procedure SetWidth(const AValue: integer);
    procedure SetBackgrounColor(AValue: TFPReportColor);
  protected
    procedure Changed; // Called whenever the visual properties change.
  public
    constructor Create(AElement: TFPReportElement);
    procedure Assign(ASource: TPersistent); override;
    procedure WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportFrame = nil); virtual;
    procedure ReadElement(AReader: TFPReportStreamer); virtual;
    function Equals(AFrame: TFPReportFrame): boolean; reintroduce;
    property ReportElement: TFPReportElement read FReportElement;
  published
    { Lines are only drawn if Shape = fsNone }
    property Lines: TFPReportFrameLines read FFrameLines write SetFrameLines;
    property Shape: TFPReportFrameShape read FFrameShape write SetFrameShape default fsNone;
    { The pen color used for stroking of shapes or for lines. }
    property Color: TFPReportColor read FColor write SetColor default clNone;
    { The fill color for shapes - where applicable. }
    property BackgroundColor: TFPReportColor read FBackgroundColor write SetBackgrounColor default clNone;
    property Pen: TFPPenStyle read FPenStyle write SetPenStyle default psSolid;
    { The width of the pen. }
    property Width: integer read FWidth write SetWidth;
  end;


  TFPReportTextAlignment = class(TPersistent)
  private
    FReportElement: TFPReportElement;
    FHorizontal: TFPReportHorzTextAlignment;
    FVertical: TFPReportVertTextAlignment;
    FTopMargin: TFPReportUnits;
    FBottomMargin: TFPReportUnits;
    FLeftMargin: TFPReportUnits;
    FRightMargin: TFPReportUnits;
    procedure   SetHorizontal(AValue: TFPReportHorzTextAlignment);
    procedure   SetVertical(AValue: TFPReportVertTextAlignment);
    procedure   SetTopMargin(AValue: TFPReportUnits);
    procedure   SetBottomMargin(AValue: TFPReportUnits);
    procedure   SetLeftMargin(AValue: TFPReportUnits);
    procedure   SetRightMargin(AValue: TFPReportUnits);
  protected
    procedure   Changed; // Called whenever the visual properties change.
  public
    constructor Create(AElement: TFPReportElement);
    procedure   Assign(ASource: TPersistent); override;
    procedure   WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportTextAlignment = nil); virtual;
    procedure   ReadElement(AReader: TFPReportStreamer); virtual;
  published
    property    Horizontal: TFPReportHorzTextAlignment read FHorizontal write SetHorizontal;
    property    Vertical: TFPReportVertTextAlignment read FVertical write SetVertical;
    property    TopMargin: TFPReportUnits read FTopMargin write SetTopMargin default 0;
    property    BottomMargin: TFPReportUnits read FBottomMargin write SetBottomMargin default 0;
    property    LeftMargin: TFPReportUnits read FLeftMargin write SetLeftMargin default 1.0;
    property    RightMargin: TFPReportUnits read FRightMargin write SetRightMargin default 1.0;
  end;


  // Position/Size related properties - this class doesn't notify FReportElement about property changes

  { TFPReportCustomLayout }

  TFPReportCustomLayout = class(TPersistent)
  private
    FPos: TFPReportRect;
    function    GetHeight: TFPreportUnits;
    function    GetWidth: TFPreportUnits;
    procedure   SetLeft(const AValue: TFPreportUnits);
    procedure   SetTop(const AValue: TFPreportUnits);
    procedure   SetWidth(const AValue: TFPreportUnits);
    procedure   SetHeight(const AValue: TFPreportUnits);
    function    GetLeft: TFPreportUnits;
    function    GetTop: TFPreportUnits;
  protected
    FReportElement: TFPReportElement;
    procedure   Changed; virtual; abstract;// Called whenever the visual properties change.
    property    Height: TFPreportUnits read GetHeight write SetHeight;
    property    Left: TFPreportUnits read GetLeft write SetLeft;
    property    Top: TFPreportUnits read GetTop write SetTop;
    property    Width: TFPreportUnits read GetWidth write SetWidth;
  public
    constructor Create(AElement: TFPReportElement);
    procedure   Assign(Source: TPersistent); override;
    procedure   WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportCustomLayout = nil); virtual;
    procedure   ReadElement(AReader: TFPReportStreamer); virtual;
    function    Equals(ALayout: TFPReportCustomLayout): boolean; reintroduce;
    procedure   GetBoundsRect(Out ARect: TFPReportRect);
    { a convenience function to set all four values in one go }
    procedure   SetPosition(aleft, atop, awidth, aheight: TFPReportUnits);
    procedure   SetPosition(Const ARect: TFPReportRect);
    property    ReportElement: TFPReportElement read FReportElement;
  end;


  // Position/Size related properties. Also notifies FReportElement about property changes
  TFPReportLayout = class(TFPReportCustomLayout)
  protected
    procedure Changed; override;
  published
    property  Height;
    property  Left;
    property  Top;
    property  Width;
  end;


  { Anything that must be drawn as part of the report descends from this. }

  { TFPReportElement }

  TFPReportElement = class(TFPReportComponent)
  private
    FFrame: TFPReportFrame;
    FLayout: TFPReportLayout;
    FParent: TFPReportElement;
    FUpdateCount: integer;
    FVisible: boolean;
    FRTLayout: TFPReportLayout;
    FOnBeforePrint: TFPReportBeforePrintEvent;
    FStretchMode: TFPReportStretchMode;
    FVisibleExpr: String;
    function GetReport: TFPCustomReport;
    procedure SetFrame(const AValue: TFPReportFrame);
    procedure SetLayout(const AValue: TFPReportLayout);
    procedure SetVisible(const AValue: boolean);
    procedure SetVisibleExpr(AValue: String);
  protected
    Procedure ParentFontChanged; virtual;
    procedure ApplyStretchMode(const ADesiredHeight: TFPReportUnits); virtual;
    function GetDateTimeFormat: String; virtual;
    function ExpandMacro(const s: String; const AIsExpr: boolean): TFPReportString; virtual;
    function GetReportBand: TFPReportCustomBand; virtual;
    function GetReportPage: TFPReportCustomPage; virtual;
    Procedure SaveDataToNames; virtual;
    Procedure RestoreDataFromNames; virtual;
    function CreateFrame: TFPReportFrame; virtual;
    function CreateLayout: TFPReportLayout; virtual;
    procedure CreateRTLayout; virtual;
    procedure SetParent(const AValue: TFPReportElement); virtual;
    procedure Changed; // Called whenever the visual properties change.
    procedure DoChanged; virtual; // Called when changed and changecount reaches zero.
    function PrepareObject(aRTParent: TFPReportElement): TFPReportElement; virtual;
    { descendants can add any extra properties to output here }
    procedure DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); virtual;
    procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    { triggers OnBeforePrint event }
    procedure BeforePrint; virtual;
    function EvaluateExpression(const AExpr: String; out Res: TFPExpressionResult): Boolean;
    function EvaluateExpressionAsText(const AExpr: String): String;
    { this is run against the runtime (RT) version of this element, and before BeforePrint is called. }
    procedure RecalcLayout; virtual; abstract;
    property  StretchMode: TFPReportStretchMode read FStretchMode write FStretchMode default smDontStretch;
    property  OnBeforePrint: TFPReportBeforePrintEvent read FOnBeforePrint write FOnBeforePrint;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    Class Function ElementType : String; virtual;
    Class Function RegisterElement : TFPReportClassMapping; virtual;
    Class Procedure UnRegisterElement; virtual;
    Function CreatePropertyHash : String; virtual;
    function ExpressionResultToString(const Res: TFPExpressionResult): String; virtual;
    function Equals(AElement: TFPReportElement): boolean; virtual; reintroduce;
    procedure WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure ReadElement(AReader: TFPReportStreamer); override;
    procedure Assign(Source: TPersistent); override;
    procedure BeginUpdate;
    procedure EndUpdate;
    Procedure Validate(aErrors : TStrings); virtual;
    function EvaluateVisibility : boolean; virtual;
    property Parent: TFPReportElement read FParent write SetParent;
    Property Report : TFPCustomReport read GetReport;
    property Page: TFPReportCustomPage read GetReportPage;
    property Band: TFPReportCustomBand read GetReportBand;
    { Runtime Layout - populated when layouting of report is calculated. }
    property  RTLayout: TFPReportLayout read FRTLayout write FRTLayout; // TOOD: Maybe we should rename this to PrintLayout?
  published
    property Layout: TFPReportLayout read FLayout write SetLayout;
    property Frame: TFPReportFrame read FFrame write SetFrame;
    property Visible: boolean read FVisible write SetVisible;
    property VisibleExpr: String read FVisibleExpr write SetVisibleExpr;
  end;


  { TFPReportElementWithChildren }

  TFPReportElementWithChildren = class(TFPReportElement)
  private
    FChildren: TFPList;
    function GetChild(AIndex: integer): TFPReportElement;
    function GetChildCount: integer;
  protected
    Procedure NotifyFontChange;
    Procedure SaveDataToNames; override;
    Procedure RestoreDataFromNames; override;
    procedure RemoveChild(const AChild: TFPReportElement); virtual;
    procedure AddChild(const AChild: TFPReportElement); virtual;
    procedure PrepareObjects(aRTParent: TFPReportElement); virtual;
    { This should run against the runtime version of the children }
    procedure RecalcLayout; override;
    procedure ApplyStretchMode(const ADesiredHeight: TFPReportUnits);override;
  public
    destructor  Destroy; override;
    Procedure   Validate(aErrors : TStrings); override;
    // called when the designer starts editing this component .
    Procedure StartDesigning; override;
    // called when the designer ends editing this component .
    Procedure EndDesigning; override;
    procedure   WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure   ReadElement(AReader: TFPReportStreamer); override;
    function    Equals(AElement: TFPReportElement): boolean; override;
    property    Child[AIndex: integer]: TFPReportElement read GetChild;
    property    ChildCount: integer read GetChildCount;
  end;


  TFPReportMargins = class(TPersistent)
  private
    FTop: TFPReportUnits;
    FBottom: TFPReportUnits;
    FLeft: TFPReportUnits;
    FRight: TFPReportUnits;
    FPage: TFPReportCustomPage;
    procedure   SetBottom(const AValue: TFPReportUnits);
    procedure   SetLeft(const AValue: TFPReportUnits);
    procedure   SetRight(const AValue: TFPReportUnits);
    procedure   SetTop(const AValue: TFPReportUnits);
  protected
    procedure   Changed; virtual;
  public
    constructor Create(APage: TFPReportCustomPage);
    procedure   Assign(Source: TPersistent); override;
    function    Equals(AMargins: TFPReportMargins): boolean; reintroduce;
    procedure   WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportMargins = nil); virtual;
    procedure   ReadElement(AReader: TFPReportStreamer); virtual;
    property    Page: TFPReportCustomPage read FPage;
  published
    property    Top: TFPReportUnits read FTop write SetTop;
    property    Bottom: TFPReportUnits read FBottom write SetBottom;
    property    Left: TFPReportUnits read FLeft write SetLeft;
    property    Right: TFPReportUnits read FRight write SetRight;
  end;


  TFPReportPageSize = class(TPersistent)
  private
    FHeight: TFPReportUnits;
    FPage: TFPReportCustomPage;
    FPaperName: string;
    FWidth: TFPReportUnits;
    procedure SetHeight(const AValue: TFPReportUnits);
    procedure SetPaperName(const AValue: string);
    procedure SetWidth(const AValue: TFPReportUnits);
  protected
    procedure CheckPaperSize;
    procedure Changed; virtual;
  public
    constructor Create(APage: TFPReportCustomPage);
    procedure Assign(Source: TPersistent); override;
    property Page: TFPReportCustomPage read FPage;
  published
    property PaperName: string read FPaperName write SetPaperName;
    property Width: TFPReportUnits read FWidth write SetWidth;
    property Height: TFPReportUnits read FHeight write SetHeight;
  end;


  { Layout is relative to the page.
    That means that top/left equals top/left margin
     Width/Height is equals to page height/width minus margins
     Page orientation is taken into consideration. }

  { TFPReportCustomPage }

  TFPReportCustomPage = class(TFPReportElementWithChildren)
  private
    FData: TFPReportData;
    FDataName : String;
    FFont: TFPReportFont;
    FMargins: TFPReportMargins;
    FOnPageSizeChange: TNotifyEvent;
    FOrientation: TFPReportPaperOrientation;
    FPageSize: TFPReportPageSize;
    FReport: TFPCustomReport;
    FBands: TBandList;
    FColumnLayout: TFPReportColumnLayout;
    FColumnCount: Byte;
    FColumnGap: TFPReportUnits;
    function GetBandCount: integer;
    function BandWidthFromColumnCount: TFPReportUnits;
    procedure ApplyBandWidth(ABand: TFPReportCustomBand);
    function  GetIsMultiColumn: Boolean; inline;
    procedure HandleFontChange(Sender: TObject);
    procedure SetFont(AValue: TFPReportFont);
    procedure SetMargins(const AValue: TFPReportMargins);
    procedure SetOrientation(const AValue: TFPReportPaperOrientation);
    procedure SetPageSize(const AValue: TFPReportPageSize);
    procedure SetReport(const AValue: TFPCustomReport);
    procedure SetReportData(const AValue: TFPReportData);
    procedure SetColumnLayout(AValue: TFPReportColumnLayout);
    procedure SetColumnCount(AValue: Byte);
    procedure SetColumnGap(AValue: TFPReportUnits);
  protected
    function GetReportPage: TFPReportCustomPage; override;
    function GetReportBand: TFPReportCustomBand; override;
    Procedure SaveDataToNames; override;
    Procedure RestoreDataFromNames; override;
    procedure RemoveChild(const AChild: TFPReportElement); override;
    procedure AddChild(const AChild: TFPReportElement); override;
    procedure MarginsChanged; virtual;
    procedure PageSizeChanged; virtual;
    procedure RecalcLayout; override;
    procedure CalcPrintPosition; virtual;
    function  PrepareObject(aRTParent: TFPReportElement): TFPReportElement; override;
    procedure PrepareObjects(aRTParent: TFPReportElement); override;
    procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    procedure DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    Function GetPageIndex : Integer; Virtual;
    Procedure SetPageIndex(aIndex : Integer);
  public
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    Class Function ElementType: String; override;
    procedure   Assign(Source: TPersistent); override;
    procedure   ReadElement(AReader: TFPReportStreamer); override;
    function    FindBandWithType(ABandType: TFPReportBandType): TFPReportCustomBand;
    function    FindBandWithTypeAndData(ABandType: TFPReportBandType; aData: TFPReportData): TFPReportCustomBand;
    Function    CheckBandMultiplicity(aBand : TFPReportCustomBand) : Boolean;
    function    CheckBandMultiplicity(aBandType: TFPReportBandType; aData: TFPReportData): Boolean;
    function    FindBand(ABand: TFPReportBandClass): TFPReportCustomBand;
    Function    FindMainDataloop : TFPReportData;
    property    PageSize: TFPReportPageSize read FPageSize write SetPageSize;
    property    Margins: TFPReportMargins read FMargins write SetMargins;
    property    Report: TFPCustomReport read FReport write SetReport;
    property    Bands : TBandList read FBands;
    property    BandCount: integer read GetBandCount;
    property    Orientation: TFPReportPaperOrientation read FOrientation write SetOrientation;
    property    Data: TFPReportData read FData write SetReportData;
    property    IsMultiColumn: Boolean read GetIsMultiColumn;
    property    ColumnCount: Byte read FColumnCount write SetColumnCount default 1;
    property    ColumnGap: TFPReportUnits read FColumnGap write SetColumnGap default 0;
    property    ColumnLayout: TFPReportColumnLayout read FColumnLayout write SetColumnLayout default clVertical;
    property    Font: TFPReportFont read FFont write SetFont;
    Property    OnPageSizeChange : TNotifyEvent Read FOnPageSizeChange Write FOnPageSizeChange;
    property    PageIndex : Integer Read GetPageIndex Write SetPageIndex;
  end;
  TFPReportCustomPageClass = Class of TFPReportCustomPage;


  TFPReportPage = class(TFPReportCustomPage)
  published
    property ColumnCount;
    property ColumnGap;
    property ColumnLayout;
    property Data;
    property Font;
    property Margins;
    property PageSize;
    property Orientation;
    Property PageIndex;
  end;


  { TFPReportCustomBand }

  TFPReportCustomBand = class(TFPReportElementWithChildren)
  private
    FChildBand: TFPReportCustomChildBand;
    FParentBand,
    FMainBand: TFPReportCustomBand;
    FKeepTogetherWithChildren: Boolean;
    FUseParentFont: boolean;
    FVisibleOnPage: TFPReportVisibleOnPage;
    FFont: TFPReportFont;
    FIsOverflowed: Boolean;
    FIsColumnType: Boolean;
    FBandPosition: TFPReportBandPosition;
    function    GetParentFont: TFPReportFont;
    procedure HandleFontChange(Sender: TObject);
    procedure ReassignParentFont;
    procedure   SetBandPosition(pBandPosition: TFPReportBandPosition); virtual;
    procedure   SetChildBand(AValue: TFPReportCustomChildBand);
    procedure   SetFont(AValue: TFPReportFont);
    procedure   SetKeepTogetherWithChildren(pKeepTogetherWithChildren: Boolean); virtual;
    procedure SetMainBand(AValue: TFPReportCustomBand);
    procedure   SetUseParentFont(AValue: boolean);
    procedure   SetVisibleOnPage(AValue: TFPReportVisibleOnPage);
  protected
    procedure FixupReference(Const PN, PV: String; C: TFPReportElement); override;
    procedure ParentFontChanged; override;
    function CalcDesiredHeight: TFPReportUnits; virtual;
    function    GetReportPage: TFPReportCustomPage; override;
    function    GetData: TFPReportData; virtual;
    procedure   SetDataFromName(AName : String); virtual;
    procedure   SetParent(const AValue: TFPReportElement); override;
    procedure   CreateRTLayout; override;
    Procedure   Notification(AComponent: TComponent; Operation: TOperation); override;
    function    PrepareObject(aRTParent: TFPReportElement): TFPReportElement; override;
    { this is normally run against the runtime version of the Band instance. }
    procedure   RecalcLayout; override;
    procedure   BeforePrint; override;
    procedure   DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure   BeginRuntimeProcessing; virtual;
    procedure   EndRuntimeProcessing; virtual;
    function    NeedsUpdateYPos: Boolean; virtual;
    procedure   AfterPrintBand(pBand: TFPReportCustomBand); virtual;
    procedure   BeforePrintWithChilds; virtual;
    procedure   MovedToNextPageWithChilds; virtual;
    procedure   AfterPrintWithChilds; virtual;
    property    Font: TFPReportFont read FFont write SetFont;
    property    UseParentFont: boolean read FUseParentFont write SetUseParentFont;
    { when set to True then band and child bands are keept on the same page (no page break between them) }
    property    KeepTogetherWithChildren: Boolean read FKeepTogetherWithChildren write SetKeepTogetherWithChildren default True;
    { band position:
      fpNormal:        after detail or inner group footer
      fpStackAtBottom: stacked at bottom before page footer }
    property    BandPosition: TFPReportBandPosition read FBandPosition write SetBandPosition default bpStackAtBottom;
  public
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    Procedure   Validate(aErrors : TStrings); override;
    procedure   Assign(Source: TPersistent); override;
    Procedure SendToBack(El : TFPReportElement);
    Procedure BringToFront(El : TFPReportElement);
    Class Function ReportBandType : TFPReportBandType; virtual;
    Class Function ElementType : String; override;
    procedure   WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure   ReadElement(AReader: TFPReportStreamer); override;
    property    VisibleOnPage: TFPReportVisibleOnPage read FVisibleOnPage write SetVisibleOnPage;
    function    EvaluateVisibility: boolean; override;
    property    ChildBand: TFPReportCustomChildBand read FChildBand write SetChildBand;
    property    ParentBand: TFPReportCustomBand read FParentBand;
    property    MainBand: TFPReportCustomBand read FMainBand Write SetMainBand;
    property    Page : TFPReportCustomPage read GetReportPage;
  end;
  TFPReportCustomBandClass = Class of TFPReportCustomBand;


  { TFPReportCustomBandWithData }

  TFPReportCustomBandWithData = class(TFPReportCustomBand)
  private
    FData: TFPReportData;
    FDataName : String;
    procedure ResolveDataName;
    procedure   SetData(const AValue: TFPReportData);
  protected
    procedure ProcessAggregates(const AData: TFPReportData); virtual;
    Procedure SaveDataToNames; override;
    Procedure RestoreDataFromNames; override;
    function  GetData: TFPReportData; override;
    Procedure SetDataFromName(AName: String); override;
    procedure  Notification(AComponent: TComponent; Operation: TOperation); override;
  public
    constructor Create(AOwner: TComponent); override;
    Procedure Validate(aErrors : TStrings); override;
  published
    property    Data: TFPReportData read GetData write SetData;
  end;
  TFPReportCustomBandWithDataClass = Class of TFPReportCustomBandWithData;


  { TFPReportCustomDataBand }

  TFPReportCustomDataBand = class(TFPReportCustomBandWithData)
  private
    FFooterBand: TFPReportCustomDataFooterBand;
    FHeaderBand: TFPReportCustomDataHeaderBand;
    FMasterBand: TFPReportCustomDataBand;
    FDisplayPosition: Integer;
    procedure SetFooterBand(AValue: TFPReportCustomDataFooterBand);
    procedure SetHeaderBand(AValue: TFPReportCustomDataHeaderBand);
    procedure SetMasterBand(AValue: TFPReportCustomDataBand);
  protected
    Procedure FixupReference(Const PN,PV : String; C : TFPReportElement); override;
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    property  DisplayPosition: Integer read FDisplayPosition write FDisplayPosition default 0;
    // No longer used, set the FooterBand.Data or HeaderBand.Data properties instead.
    // When setting HeaderBand/Footer properties, the FooterBand.Data/HeaderBand.Data are set to the data band data property.
    property  FooterBand: TFPReportCustomDataFooterBand read FFooterBand write SetFooterBand; deprecated;
    property  HeaderBand: TFPReportCustomDataHeaderBand read FHeaderBand write SetHeaderBand; deprecated;
    property  MasterBand: TFPReportCustomDataBand read FMasterBand write SetMasterBand;
  public
    procedure DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure ReadElement(AReader: TFPReportStreamer); override;
    constructor Create(AOwner: TComponent); override;
  end;
  TFPReportCustomDataBandClass = Class of TFPReportCustomDataBand;


  { Master data band. The report loop happens on this band. }

  { TFPReportDataBand }

  TFPReportDataBand = class(TFPReportCustomDataBand)
  protected
  Public
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  published
    property    ChildBand;
    property    DisplayPosition;
    property    Font;
    property    MasterBand;
    property    StretchMode;
    property    UseParentFont;
    property    VisibleOnPage;
    property    KeepTogetherWithChildren;
    property    OnBeforePrint;
  end;


  { TFPReportCustomChildBand }

  TFPReportCustomChildBand = class(TFPReportCustomBand)
  protected
  Public
    Procedure   Validate(aErrors : TStrings); override;
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  end;


  TFPReportChildBand = class(TFPReportCustomChildBand)
  published
    property    ChildBand;
    property    Font;
    property    StretchMode;
    property    UseParentFont;
    property    VisibleOnPage;
    property    OnBeforePrint;
  end;


  { TFPReportCustomPageFooterBand }

  TFPReportCustomPageFooterBand = class(TFPReportCustomBand)
  protected
  Public
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  end;


  TFPReportPageFooterBand = class(TFPReportCustomPageFooterBand)
  published
    property    ChildBand;
    property    Font;
    property    UseParentFont;
    property    VisibleOnPage;
    property    KeepTogetherWithChildren;
    property    OnBeforePrint;
  end;


  { TFPReportCustomPageHeaderBand }

  TFPReportCustomPageHeaderBand = class(TFPReportCustomBand)
  protected
  public
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  end;


  TFPReportPageHeaderBand = class(TFPReportCustomPageHeaderBand)
  published
    property    ChildBand;
    property    Font;
    property    StretchMode;
    property    UseParentFont;
    property    VisibleOnPage;
    property    KeepTogetherWithChildren;
    property    OnBeforePrint;
  end;


  { TFPReportCustomColumnHeaderBand }

  TFPReportCustomColumnHeaderBand = class(TFPReportCustomBandWithData)
  protected
  Public
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  end;


  TFPReportColumnHeaderBand = class(TFPReportCustomColumnHeaderBand)
  published
    property    ChildBand;
    property    StretchMode;
    property    Data;
    property    Font;
    property    UseParentFont;
    property    OnBeforePrint;
  end;


  { TFPReportCustomColumnFooterBand }

  TFPReportCustomColumnFooterBand = class(TFPReportCustomBandWithData)
  public
    constructor Create(AOwner: TComponent); override;
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
    property    BandPosition;
  end;


  TFPReportColumnFooterBand = class(TFPReportCustomColumnFooterBand)
  published
  property    ChildBand;
    property    Font;
    property    BandPosition;
    property    UseParentFont;
    property    OnBeforePrint;
    property    StretchMode;
  end;


  { TFPReportCustomGroupHeaderBand }

  TFPReportCustomGroupHeaderBand = class(TFPReportCustomBandWithData)
  private
    FOverflowedFooterNeedsReprintedHeader: TFPReportSections;
    FIntermediateFooter: TFPReportSections;
    FLastGroupConditionValue,
    FGroupConditionValue: string;
    FParentGroupHeader: TFPReportCustomGroupHeaderBand;
    FChildGroupHeader: TFPReportCustomGroupHeaderBand;
    FGroupFooter: TFPReportCustomGroupFooterBand;
    FGroupCondition: string;
    FOverflowWithFirstDataBand: TFPReportSections;
    FRTBands: TBandList;
    FReprintedHeader: TFPReportSections;
    FStartOnNewSection: TFPReportSection;
    FDetailsPrinted: Boolean;
    { runtime properties }
    FNeedsReprintedHeader: Boolean;
    FNeedsIntermediateFooter: Boolean;
    FNeedsPrevVariables: Boolean;
    procedure   SetGroupHeader(AValue: TFPReportCustomGroupHeaderBand);
    procedure   InternalEvaluateGroupCondition;
    procedure   SetKeepTogetherWithChildren(pKeepTogetherWithChildren: Boolean); override;
    procedure   SetOverflowedFooterNeedsReprintedHeader(pOverflowedFooterNeedsReprintedHeader: TFPReportSections);
    procedure   SetOverflowWithFirstDataBand(pOverflowWithFirstDataBand: TFPReportSections);
    procedure   SetReprintedHeader(pReprintedHeader: TFPReportSections);
    procedure   SetStartOnNewSection(pStartOnNewSection: TFPReportSection);
  protected
    procedure   DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure   Notification(AComponent: TComponent; Operation: TOperation); override;
    Procedure   FixupReference(Const PN, PV: String; C: TFPReportElement); override;
    procedure   BeforePrintWithChilds; override;
    procedure   MovedToNextPageWithChilds; override;
    procedure   AfterPrintWithChilds; override;
    procedure   StoreRTBands(pBands: TBandList);
    { This property defines the hierarchy of nested groups. For the top most group, this property will be nil. }
    property    ParentGroupHeader: TFPReportCustomGroupHeaderBand read FParentGroupHeader write SetGroupHeader;
    { Indicates related GroupFooter band. This will automatically be set by the GroupFooter. }
    property    GroupFooter: TFPReportCustomGroupFooterBand read FGroupFooter;
    { can be a field name or an expression }
    property    GroupCondition: string read FGroupCondition write FGroupCondition;
    { Run-time calculated value }
    property    GroupConditionValue : string read FGroupConditionValue write FGroupConditionValue;
    { start group on new page or column }
    property    StartOnNewSection: TFPReportSection read FStartOnNewSection write SetStartOnNewSection;
    { reprint header on new page/column }
    property    ReprintedHeader: TFPReportSections read FReprintedHeader write SetReprintedHeader;
    { reprints header if group footer is overflowed and ReprintHeader is true }
    property    OverflowedFooterNeedsReprintedHeader: TFPReportSections read FOverflowedFooterNeedsReprintedHeader write SetOverflowedFooterNeedsReprintedHeader;
    { print footer before every page/column break }
    property    IntermediateFooter: TFPReportSections read FIntermediateFooter write FIntermediateFooter;
    property    BandPosition;
    { if first data band flows over to next page/column, also move this header }
    property    OverflowWithFirstDataBand: TFPReportSections read FOverflowWithFirstDataBand write SetOverflowWithFirstDataBand default [rsPage, rsColumn];
    function    NeedsOverflowWithFirstDataBand(pIsLastColumn: Boolean): Boolean;
    function    NeedsIntermediateFooter(pIsLastColumn: Boolean): Boolean;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    Class Function ElementType : String; override;
    Procedure  Validate(aErrors : TStrings); override;
    procedure   Assign(Source: TPersistent); override;
    procedure   ReadElement(AReader: TFPReportStreamer); override;
    procedure   EvaluateGroupCondition;
    function    GroupChanged: Boolean;
    function    IsInitialGroupChange: Boolean;
    procedure   ResetGroupConditionValues;
    Class Function ReportBandType : TFPReportBandType; override;
    { This property defines the hierarchy of nested groups. For the bottom most group, this property will be nil. }
    property    ChildGroupHeader: TFPReportCustomGroupHeaderBand read FChildGroupHeader;
  end;


  TFPReportGroupHeaderBand = class(TFPReportCustomGroupHeaderBand)
  public
    property    GroupFooter;
  published
    property    ChildBand;
    property    Font;
    property    GroupCondition;
    property    ParentGroupHeader;
    property    UseParentFont;
    property    KeepTogetherWithChildren;
    property    OnBeforePrint;
    property    StretchMode;
    property    StartOnNewSection;
    property    ReprintedHeader;
    property    OverflowedFooterNeedsReprintedHeader;
    property    IntermediateFooter;
    property    OverflowWithFirstDataBand;
    property    BandPosition;
  end;


  { Report title band - prints once at the beginning of the report }

  { TFPReportCustomTitleBand }

  TFPReportCustomTitleBand = class(TFPReportCustomBand)
  protected
  Public
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  end;


  TFPReportTitleBand = class(TFPReportCustomTitleBand)
  published
    property    StretchMode;
    property    ChildBand;
    property    Font;
    property    UseParentFont;
    property    KeepTogetherWithChildren;
    property    OnBeforePrint;
  end;


  { Report summary band - prints once at the end of the report }

  { TFPReportCustomSummaryBand }

  TFPReportCustomSummaryBand = class(TFPReportCustomBand)
  private
    FStartNewPage: boolean;
  protected
    procedure   DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    property    StartNewPage: boolean read FStartNewPage write FStartNewPage default False;
  public
    constructor Create(AOwner: TComponent); override;
    procedure   Assign(Source: TPersistent); override;
    procedure   ReadElement(AReader: TFPReportStreamer); override;
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  end;


  TFPReportSummaryBand = class(TFPReportCustomSummaryBand)
  published
    property    StretchMode;
    property    ChildBand;
    property    Font;
    property    StartNewPage;
    property    UseParentFont;
    property    KeepTogetherWithChildren;
    property    OnBeforePrint;
  end;


  { TFPReportCustomGroupFooterBand }

  TFPReportCustomGroupFooterBand = class(TFPReportCustomBandWithData)
  private
    FGroupHeader: TFPReportCustomGroupHeaderBand;
    FDoNotConsiderInFooterSpaceNeeded: Boolean;
    procedure SetGroupHeader(const AValue: TFPReportCustomGroupHeaderBand);
  protected
    procedure FixupReference(Const PN, PV: String; C: TFPReportElement); override;
    procedure SetBandPosition(pBandPosition: TFPReportBandPosition); override;
    procedure DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    procedure BeginRuntimeProcessing; override;
    procedure EndRuntimeProcessing; override;
    function  NeedsUpdateYPos: Boolean; override;
    procedure BeforePrintWithChilds; override;
    procedure AfterPrintWithChilds; override;
    property  GroupHeader: TFPReportCustomGroupHeaderBand read FGroupHeader write SetGroupHeader;
  public
    Destructor Destroy; override;
    procedure ReadElement(AReader: TFPReportStreamer); override;
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
    property  BandPosition;
  end;


  TFPReportGroupFooterBand = class(TFPReportCustomGroupFooterBand)
  published
    property    ChildBand;
    property    Font;
    property    GroupHeader;
    property    BandPosition;
    property    UseParentFont;
    property    KeepTogetherWithChildren;
    property    OnBeforePrint;
    property    StretchMode;
  end;


  { TFPReportCustomDataHeaderBand }

  TFPReportCustomDataHeaderBand = class(TFPReportCustomBandWithData)
  Public
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  end;


  TFPReportDataHeaderBand = class(TFPReportCustomDataHeaderBand)
  Published
    property    ChildBand;
    property    StretchMode;
    property    Font;
    property    UseParentFont;
    property    OnBeforePrint;
  end;


  { TFPReportCustomDataFooterBand }

  TFPReportCustomDataFooterBand = class(TFPReportCustomBandWithData)
  protected
  Public
    Class Function ElementType : String; override;
    Class Function ReportBandType : TFPReportBandType; override;
  end;


  TFPReportDataFooterBand = class(TFPReportCustomDataFooterBand)
  published
    property    ChildBand;
    property    StretchMode;
    property    Font;
    property    UseParentFont;
    property    OnBeforePrint;
  end;


  TFPReportImageItem = class(TCollectionItem)
  private
    FImage: TFPCustomImage;
    FOwnsImage: Boolean;
    FStreamed: TBytes;
    FWidth: Integer;
    FHeight: Integer;
    function    GetHeight: Integer;
    function    GetStreamed: TBytes;
    function    GetWidth: Integer;
    procedure   SetImage(AValue: TFPCustomImage);
    procedure   SetStreamed(AValue: TBytes);
    procedure   LoadPNGFromStream(AStream: TStream);
  public
    constructor Create(ACollection: TCollection); override;
    destructor  Destroy; override;
    procedure   CreateStreamedData;
    function    WriteImageStream(AStream: TStream): UInt64; virtual;
    function    Equals(AImage: TFPCustomImage): boolean; reintroduce;
    procedure   WriteElement(AWriter: TFPReportStreamer);
    procedure   ReadElement(AReader: TFPReportStreamer);
    property    Image: TFPCustomImage read FImage write SetImage;
    property    StreamedData: TBytes read GetStreamed write SetStreamed;
    property    OwnsImage: Boolean read FOwnsImage write FOwnsImage default True;
    property    Width: Integer read GetWidth;
    property    Height: Integer read GetHeight;
  end;


  { TFPReportImages }

  TFPReportImages = class(TOwnedCollection)
  private
    function    GetImg(AIndex: Integer): TFPReportImageItem;
    function GetReportOwner: TFPCustomReport;
  protected
  public
    constructor Create(AOwner: TFPCustomReport; AItemClass: TCollectionItemClass);
    function    AddImageItem: TFPReportImageItem;
    function    AddFromStream(const AStream: TStream; Handler: TFPCustomImageReaderClass; KeepImage: Boolean = False): Integer;
    function    AddFromFile(const AFileName: string; KeepImage: Boolean = False): Integer;
    function    AddFromData(const AImageData: Pointer; const AImageDataSize: LongWord): integer;
    function    GetIndexFromID(const AID: integer): integer;
    Function    GetImageFromID(const AID: integer): TFPCustomImage;
    Function    GetImageItemFromID(const AID: integer): TFPReportImageItem;
    property    Images[AIndex: Integer]: TFPReportImageItem read GetImg; default;
    property    Owner: TFPCustomReport read GetReportOwner;
  end;

  { TFPReportVariable }

  TFPReportVariable = Class(TCollectionItem)
  private
    FName: String;
    FValue: TFPExpressionResult;
    FSavedValue: TFPExpressionResult;
    FLastValue: TFPExpressionResult;
    FExpression: String;
    FExpressionNode: TFPExprNode;
    FIsAggregate: Boolean;
    FAggregateValue: TFPExpressionResult;
    FAggregateValues: TList;
    FAggregateValuesIndex: Integer;
    FLastRecordNo: Integer;
    FResetType: TFPReportResetType;
    FResetValue: String;
    FResetValueExpression: String;
    FResetValueExpressionNode: TFPExprNode;
    FDataName : String;
    procedure CheckType(aType: TResultType);
    procedure ClearAggregateValues;
    function GetAsBoolean: Boolean;
    function GetAsCurrency: Currency;
    function GetAsDateTime: TDateTime;
    function GetAsFloat: TexprFloat;
    function GetAsInteger: Int64;
    function GetAsString: String;
    function GetDataType: TResultType;
    function GetER: TFPExpressionResult;
    function GetValue: String;
    procedure SetAsBoolean(AValue: Boolean);
    procedure SetAsCurrency(AValue: Currency);
    procedure SetAsDateTime(AValue: TDateTime);
    procedure SetAsFloat(AValue: TExprFloat);
    procedure SetAsInteger(AValue: Int64);
    procedure SetAsString(AValue: String);
    procedure SetDataType(AValue: TResultType);
    procedure SetER(AValue: TFPExpressionResult);
    procedure SetName(AValue: String);
    procedure SetResetType(AValue: TFPReportResetType);
    procedure SetValue(AValue: String);
    Procedure SaveValue; virtual;
    Procedure RestoreValue; virtual;
  Protected
    Procedure ReleaseExpressionNodes;
    procedure InitializeExpression(Expr: TFPExpressionParser; AData: TFPReportDataCollection; IsFirstpass: Boolean);
    procedure ExtractDataName(aData: TFPReportDataCollection; ANode: TFPExprNode);
    procedure ExtractDataName(aData: TFPReportDataCollection);
    Procedure GetRTValue(Var Result : TFPExpressionResult; ConstRef AName : ShortString); virtual;
    procedure GetRTExpressionValue(Var Result : TFPExpressionResult; ConstRef AName : ShortString); virtual;
  Public
    constructor Create(ACollection: TCollection); override;
    destructor Destroy; override;
    Procedure Assign(Source : TPersistent); override;
    // Init, update and finish aggregates. Called at pass start, open dataset, new record, EOF respectively
    procedure InitExpressionValue(aData : TFPReportData; IsFirstpass : Boolean);
    procedure UpdateExpressionValue(aData : TFPReportData; IsFirstpass : Boolean);
    procedure DoneExpressionValue(aData : TFPReportData; IsFirstpass : Boolean);
    Procedure WriteElement(aWriter : TFPReportStreamer); virtual;
    Procedure ReadElement(aWriter : TFPReportStreamer); virtual;
    Property AsExpressionResult : TFPExpressionResult Read GetER Write SetER;
    Property AsString : String Read GetAsString Write SetAsString;
    Property AsInteger : Int64 Read GetAsInteger Write SetAsInteger;
    Property AsBoolean : Boolean Read GetAsBoolean Write SetAsBoolean;
    Property AsFloat : TExprFloat Read GetAsFloat Write SetAsFloat;
    Property AsCurrency : Currency Read GetAsCurrency Write SetAsCurrency;
    Property AsDateTime : TDateTime Read GetAsDateTime Write SetAsDateTime;
  Published
    Property Name : String Read FName Write SetName;
    Property DataType : TResultType Read GetDataType Write SetDataType;
    property Value : String Read GetValue Write SetValue;
    property Expression: String Read FExpression Write FExpression;
    property ResetValueExpression: String Read FResetValueExpression Write FResetValueExpression;
    property ResetType : TFPReportResetType Read FResetType Write SetResetType;
  end;

  { TFPReportVariables }

  TFPReportVariables = Class(TOwnedCollection)
  private
    function GetV(aIndex : Integer): TFPReportVariable;
    procedure SetV(aIndex : Integer; AValue: TFPReportVariable);
  Protected
  public
    Procedure ReleaseExpressionNodes;
    // Init, update and finish aggregates. Called at start, new record, EOF respectively
    procedure InitExpressionValues(aData: TFPReportData; isFirstPass : Boolean);
    procedure DoneExpressionValues(aData: TFPReportData; isFirstPass : Boolean);
    procedure UpdateExpressionValues(aData: TFPReportData; isFirstPass: Boolean);
    Function IndexOfVariable(aName : String)  : Integer;
    Function FindVariable(aName : String)  : TFPReportVariable;
    Function AddVariable(aName : String)  : TFPReportVariable;
    Function AddDataVariable(aName : String)  : TFPReportVariable;
    Function AddExprVariable(aName : String; aExpr: String; aType: TResultType; aResetGroup: TFPReportCustomGroupHeaderBand) : TFPReportVariable;
    Function AddExprVariable(aName : String; aExpr: String; aType: TResultType; aResetType: TFPReportResetType = rtnone; aResetValueExpression: String = '') : TFPReportVariable;
    Property Variable[aIndex : Integer] : TFPReportVariable Read GetV Write SetV; default;
  end;

  { TFPCustomReport }

  TFPCustomReport = class(TFPReportComponent)
  private
    FPages: TFPList;
    FRTCurDsgnPageIdx: integer;
    FOnBeginReport: TFPReportBeginReportEvent;
    FOnEndReport: TFPReportEndReportEvent;
    FReportData: TFPReportDataCollection;
    FRTObjects: TFPList;  // see property
    FRTCurPageIdx: integer; // RTObjects index reference to current page being layout
    FRTUsePrevVariableValuesCount: Integer;
    FRTUsePrevVariableValues: Boolean;
    FRTInRepeatedGroupHeader: Boolean; // true while repeated group header is printed
    FRTInIntermediateGroupFooter: Boolean; // true while non final group footer is printed
    FRTIsOverflowed: Boolean; // true when overflowed band is reprinted
    FRTGroupDetailsPrinted: Boolean; // true in group footer if at least one detail has been printed
    FExpr: TFPexpressionParser;
    FTitle: string;
    FAuthor: string;
    FPageNumber: integer;
    FColumnNumber: integer;
    FPageCount: integer; // requires two-pass reporting
    FPageNumberPerDesignerPage: integer; // page number per report designer pages
    FDateCreated: TDateTime;
    FReferenceList: TStringList;
    FImages: TFPReportImages;
    FOnBeforeRenderReport: TNotifyEvent;
    FOnAfterRenderReport: TNotifyEvent;
    FTwoPass: boolean;
    FIsFirstPass: boolean;
    FLoopData: TFPReportData;
    FPerDesignerPageCount: array of UInt32;
    FUsePageCountMarker: Boolean;
    FVariables : TFPReportVariables;
    FDataAdded : TFPList;
    procedure BuiltinGetFieldIsNull(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    function GetPage(AIndex: integer): TFPReportCustomPage;
    function GetPageCount: integer; { this is designer page count }
    function GetRenderedPageCount: integer;
    procedure BuiltinExprRecNo(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure BuiltinGetPageNumber(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure BuiltinGetColumnNumber(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure BuiltinGetPageNoPerDesignerPage(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure BuiltinGetPageCount(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure BuiltinGetInRepeatedGroupHeader(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure BuiltInGetInIntermediateGroupFooter(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure BuiltinGetIsOverflowed(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure BuiltinGetIsGroupDetailsPrinted(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    { checks if children are visble, removes children if needed, and recalc Band.Layout bounds }
    procedure EmptyRTObjects;
    procedure ClearDataBandLastTextValues(ABand: TFPReportCustomBandWithData);
    // Init, update and finish aggregates. Called at start, new record, EOF respectively
    procedure InitAggregates(APage : TFPReportCustomPage; const AData: TFPReportData);
    procedure UpdateAggregates(APage : TFPReportCustomPage; const AData: TFPReportData);
    procedure DoneAggregates(APage : TFPReportCustomPage; const AData: TFPReportData);

    { these three methods are used to resolve references while reading a report from file. }
    procedure ClearReferenceList;
    procedure AddReference(AComponent : TFPReportComponent; const APropertyName,AValueName: string);
    procedure FixupReferences;

    procedure DoBeforeRenderReport;
    procedure DoAfterRenderReport;
    procedure DoProcessTwoPass;
    function  DoGetUsePrevValue: Boolean;
    procedure SetReportData(AValue: TFPReportDataCollection);
    procedure SetUsePageCountMarker(AValue: Boolean);
    procedure SetVariables(AValue: TFPReportVariables);
    procedure RTBeginUsePrevVariableValues;
    procedure RTEndUsePrevVariableValues;
  protected
    class function IsStringValueZero(const AValue: string): boolean; virtual;
    function CreateVariables: TFPReportVariables; virtual;
    function CreateImages: TFPReportImages; virtual;
    function CreateReportData: TFPReportDataCollection; virtual;
    function CreateLayouter : TFPReportLayouter; virtual;

    procedure CollectReportData; virtual;
    procedure RestoreDefaultVariables; virtual;
    procedure DoPrepareReport; virtual;
    procedure DoBeginReport; virtual;
    procedure DoEndReport; virtual;
    procedure InitializeDefaultExpressions; virtual;
    procedure InitializeExpressionVariables; virtual;
    procedure InitializeAggregates(IsFirstPass: Boolean); virtual;
    procedure CacheMemoExpressions(const APage: TFPReportCustomPage); virtual;
    procedure StartRender; override;
    procedure EndRender; override;
    // stores object instances for and during layouting
    property  RTObjects: TFPList read FRTObjects;
    property Images: TFPReportImages read FImages;
    property Pages[AIndex: integer]: TFPReportCustomPage read GetPage;
    property PageCount: integer read GetPageCount;
    property IsFirstPass: boolean read FIsFirstPass write FIsFirstPass default False;
    property OnBeginReport: TFPReportBeginReportEvent read FOnBeginReport write FOnBeginReport;
    property OnEndReport: TFPReportEndReportEvent read FOnEndReport write FOnEndReport;
    property OnBeforeRenderReport: TNotifyEvent read FOnBeforeRenderReport write FOnBeforeRenderReport;
    property OnAfterRenderReport: TNotifyEvent read FOnAfterRenderReport write FOnAfterRenderReport;

  public
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    class function ReportKindToResultType(const AType: TFPReportFieldKind): TResultType;
    Function StreamToReportElements(aStream : TStream) : TFPObjectList;
    Procedure Clear(ClearData : Boolean = True);
    Procedure SaveDataToNames;
    Procedure RestoreDataFromNames;
    procedure WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure WriteRTElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil);
    procedure ReadElement(AReader: TFPReportStreamer); override;
    procedure AddPage(APage: TFPReportCustomPage);
    procedure RemovePage(APage: TFPReportCustomPage);
    function  IndexOfPage(aPage: TFPReportCustomPage): Integer;
    function  FindRecursive(const AName: string): TFPReportElement;
    procedure Validate;
    procedure Validate(aErrors: TStrings);
    procedure RunReport;
    Procedure ClearPreparedReport; virtual;
    Function  Prepared : Boolean;
    Procedure StartDesigning; override;
    Procedure EndDesigning; override;
    procedure RenderReport(const AExporter: TFPReportExporter);
    procedure AddBuiltinsToExpressionIdentifiers(Idents: TFPExprIdentifierDefs); virtual;
    Property Variables : TFPReportVariables Read FVariables Write SetVariables;
    Property ExpressionParser : TFPExpressionParser Read FExpr;
    {$IFDEF gdebug}
    function DebugPreparedPageAsJSON(const APageNo: Byte): string;
    {$ENDIF}
    property Author: string read FAuthor write FAuthor;
    property DateCreated: TDateTime read FDateCreated write FDateCreated;
    property Title: string read FTitle write FTitle;
    property TwoPass: boolean read FTwoPass write FTwoPass default False;
    Property ReportData : TFPReportDataCollection Read FReportData Write SetReportData;
    Property UsePageCountMarker : Boolean Read FUsePageCountMarker Write SetUsePageCountMarker;
  end;


  TFPReport = class(TFPCustomReport)
  public
    property Pages;
    property PageCount;
    property Images;
    Property IsFirstPass;
  published
    property Author;
    property Title;
    property TwoPass;
    property ReportData;
    property OnAfterRenderReport;
    property OnBeforeRenderReport;
    property OnBeginReport;
    property OnEndReport;
  end;

  TFPReportCustomDataManager = Class(TComponent)
  Public
    procedure WriteElement(AWriter: TFPReportStreamer); virtual; abstract;
    procedure ReadElement(AReader: TFPReportStreamer); virtual; abstract;
    Procedure ApplyToReport(aReport : TFPCustomReport; AErrors: TStrings); virtual; abstract;
    Procedure RemoveFromReport(aReport : TFPCustomReport); virtual; abstract;
  end;


  { TFPReportLayouter }
  TOverFlowAction = (oaNone,oaBandWithChilds,oaSingleBand);
  TOverFlowActions = Set of TOverFlowAction;

  { TLoopData }

  TLoopData = Class
  private
    FData: TFPReportData;
    FParentLoop: TLoopData;
    FDataHeaderPrinted: boolean;
    FDataHeader : TFPReportCustomDataHeaderBand;
    FDataFooter : TFPReportCustomDataFooterBand;
    FDataBand : TFPReportCustomDataBand;
    FGroupFooterList: TBandList;
    FGroupHeaderList: TBandList;
    FSubLoops : TFPObjectList;
    function GetSubLoop(AIndex : Integer): TLoopData;
    function GetSubLoopCount: Integer;
  Protected
    procedure CollectDataBands(APage: TFPReportCustomPage);
    procedure PrepareGroups(APage: TFPReportCustomPage);
    procedure SetParentLoop(AValue: TLoopData);
    Procedure PrepareBandSubLoops(APage: TFPReportCustomPage);
    Procedure SetDetailsPrinted;
    Procedure ResetGroups;
  Public
    Constructor Create(aData : TFPReportData);
    Destructor Destroy; override;
    Property Data : TFPReportData Read FData;
    Property ParentLoop : TLoopData Read FParentLoop Write SetParentLoop;
    Property SubLoop[AIndex : Integer] : TLoopData Read GetSubLoop;
    Property SubLoopCount : Integer Read GetSubLoopCount;
  end;

  TFPReportLayouter = Class(TComponent)
  Private
    FMyReport: TFPCustomReport;
    FCurrentLoop : TLoopData;
    FNewPage: boolean;  // indicates if a new ReportPage needs to be created - used if DataBand spans multiple pages for example
    FNewColumn: boolean;
    FPageFooter: TFPReportCustomBand;
    FSpaceLeft: TFPReportUnits;
    FColumnYStartPos: TFPReportUnits;
    FLastYPos: TFPReportUnits;
    FLastXPos: TFPReportUnits;
    FPageFooterYPos: TFPReportUnits;
    FOverflowed: boolean;
    FColumnDetailsPrinted: Boolean;
    FRTCurrentColumn: UInt8;
    FRTIsMultiColumn: boolean;
    FPageHeader: TFPReportCustomPageHeaderBand;
    FTitle: TFPReportCustomTitleBand;
    FSummary : TFPReportCustomSummaryBand;
    FColumnHeader: TFPReportCustomColumnHeaderBand;
    FColumnFooter: TFPReportCustomColumnFooterBand;
    FRTBottomStackedFooterList: TBandList;
    FRTPage: TFPReportCustomPage;
    FCurrentRTColumnFooterBand: TFPReportCustomColumnFooterBand;
    function FindFooter(aPage: TFPReportCustomPage; aData: TFPReportData): TFPReportCustomDataFooterBand;
    function FindHeader(aPage: TFPReportCustomPage; aData: TFPReportData): TFPReportCustomDataHeaderBand;
    function GetCurrentLoop: TLoopData;
    function GetPage(AIndex: integer): TFPReportCustomPage;
    function GetRTCurDsgnPageIdx: Integer;
    function GetPerDesignerPageCount(Index : Cardinal): Cardinal;
    function GetRTCurPageIdx: Integer;
    function GetRTIsLastColumn: Boolean;
    function GetRTObjects: TFPList;
    procedure InitReportData(aMainData: TFPReportData);
    procedure SetGetPerDesignerPageCount(Index : Cardinal; AValue: Cardinal);
    Function GetPageNumberPerDesignerPage : Integer;
    procedure SetRTCurDsgnPageIdx(pPageIdx: Integer);
    Procedure SetPageNumberPerDesignerPage(aValue : Integer);
    function FooterSpaceNeeded: TFPReportUnits;
  protected
    procedure PushLoop(aLoop : TLoopData); virtual;
    Function PopLoop : TLoopData; virtual;
    procedure RemoveBandsFromPage(aList: TBandList); virtual;
    function HandleOverflowedBands(aHandledBands: TBandList; aBand: TFPReportCustomBand;
      var aRTBand: TFPReportCustomBand): TOverFlowAction; virtual;
    procedure CheckNewOrOverFlow(CheckMulticolumn: Boolean = True); virtual;
    procedure SetPageCount(aCount : Integer);
    procedure IncPageNumberPerDesignerPage;
    procedure InitRTCurPageIdx;
    procedure IncPageNumber;
    Procedure InitPageNumber;
    Function IsFirstPass : Boolean;
    Function TwoPass : Boolean;
    procedure InitPass(aPassIdx: Integer); virtual;
    procedure InitBandList(aPage: TFPReportCustomPage); virtual;
    procedure InitDesignPage(aPageIdx: integer; APage : TFPReportCustomPage); virtual;
    procedure RunDataLoop(aPage: TFPReportCustomPage; aData: TFPReportData); virtual;
    procedure PrepareRecord(aData: TFPReportData);
    procedure PrepareHeaderFooter(APage: TFPReportCustomPage);virtual;
    procedure PrepareBottomStackedFooters; virtual;
    procedure UpdateSpaceRemaining(const ABand: TFPReportCustomBand; const AUpdateYPos: boolean = True);virtual;
    function CommonRuntimeBandProcessing(const aBand: TFPReportCustomBand): TFPReportCustomBand; virtual;
    procedure ShowDataBand; virtual;
    procedure ShowDataHeaderBand; virtual;
    procedure ShowDetailBands; virtual;
    procedure ShowColumnFooterBand(aBand: TFPReportCustomColumnFooterBand); virtual;
    function HandleHeaderBands: Boolean; virtual;
    Procedure HandleFooterBands; virtual;
    procedure HandleBottomStackedFooters; virtual;
    procedure HandleRepeatedGroupHeaderBands(pNewPage: Boolean); virtual;
    procedure HandleGroupBands; virtual;
    procedure HandleLastGroupFooters; virtual;
    procedure HandleReportSummaryBands; virtual;
    procedure ShowGroupHeaderBand(aBand: TFPReportCustomGroupHeaderBand; aCheckStartOnNewSection: Boolean); virtual;
    procedure ShowGroupFooterBand(aBand: TFPReportCustomGroupFooterBand); virtual;
    function ShowBandWithChilds(aBand: TFPReportCustomBand): Boolean; virtual;
    function NoSpaceRemaining: boolean; virtual;
    procedure StartNewPage; virtual;
    procedure StartNewColumn; virtual;
    procedure EndColumn; virtual;
    procedure EndPage; virtual;
    procedure HandleOverflowed; virtual;
    Procedure DoExecute; virtual;
    // In case descendents need these, make them available
    Property CurrentLoop : TLoopData Read GetCurrentLoop;
    Property PerDesignerPageCount [Index : Cardinal] : Cardinal Read GetPerDesignerPageCount Write SetGetPerDesignerPageCount;
    property Pages[AIndex: integer]: TFPReportCustomPage read GetPage;
    property RTObjects: TFPList read GetRTObjects;
    Property RTCurPageIdx : Integer Read GetRTCurPageIdx;
    Property RTCurColumn : UInt8 Read FRTCurrentColumn;
    Property RTCurPage : TFPReportCustomPage Read FRTPage;
    Property RTCurColumnFooterBand : TFPReportCustomColumnFooterBand Read FCurrentRTColumnFooterBand;
    Property RTCurDsgnPageIdx : Integer Read GetRTCurDsgnPageIdx write SetRTCurDsgnPageIdx;
    property RTIsLastColumn: Boolean read GetRTIsLastColumn;
    Property PageNumberPerDesignerPage : Integer Read GetPageNumberPerDesignerPage Write SetPageNumberPerDesignerPage;
  Public
    Constructor Create(aOwner : TComponent); override;
    Destructor Destroy; override;

    Procedure Execute(aReport : TFPCustomReport);
    Property Report : TFPCustomReport Read FMyReport;
  end;

  EReportError = class(Exception);
  EInvalidReportError = Class(EReportError);
  EReportExportError = class(EReportError);
  EReportFontNotFound = class(EReportError);


  TFPReportPaperManager = class(TComponent)
  private
    FPaperSizes: TStringList;
    function GetPaperCount: integer;
    function GetPaperHeight(AIndex: integer): TFPReportUnits;
    function GetPaperHeightByName(AName: string): TFPReportUnits;
    function GetPaperName(AIndex: integer): string;
    function GetPaperWidth(AIndex: integer): TFPReportUnits;
    function GetPaperWidthByName(AName: string): TFPReportUnits;
  protected
    function FindPaper(const AName: string): TFPReportPaperSize;
    function GetPaperByname(const AName: string): TFPReportPaperSize;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    procedure Clear;
    procedure RegisterPaper(const AName: string; const AWidth, AHeight: TFPReportUnits);
    procedure RegisterStandardSizes;
    { assign registered names to AList - useful to populate ComboBoxes etc }
    procedure GetRegisteredSizes(var AList: TStringList);
    function IndexOfPaper(const AName: string): integer;
    property PaperNames[AIndex: integer]: string read GetPaperName;
    property PaperHeight[AIndex: integer]: TFPReportUnits read GetPaperHeight;
    property PaperWidth[AIndex: integer]: TFPReportUnits read GetPaperWidth;
    property HeightByName[AName: string]: TFPReportUnits read GetPaperHeightByName;
    property WidthByName[AName: string]: TFPReportUnits read GetPaperWidthByName;
    property PaperCount: integer read GetPaperCount;
  end;


  TExprNodeInfoRec = record
    Position: UInt32;
    ExprNode: TFPExprNode;
  end;


  { TFPReportCustomMemo }

  TFPReportCustomMemo = class(TFPReportElement)
  private
    FCullThreshold: TFPReportCullThreshold;
    FText: TFPReportString;
    FIsExpr: boolean;
    FTextAlignment: TFPReportTextAlignment;
    FTextLines: TStrings;
    FLineSpacing: TFPReportUnits;
    FCurTextBlock: TFPTextBlock;
    FTextBlockList: TFPTextBlockList;
    FParser: THTMLParser;
    { These six fields are used by PrepareTextBlocks() }
    FTextBlockState: TFPReportHTMLTagSet;
    FTextBlockXOffset: TFPReportUnits;
    FTextBlockYOffset: TFPReportUnits;
    FLastURL: string;
    FLastBGColor: TFPReportColor;
    FLastFGColor: TFPReportColor;
    FLinkColor: TFPReportColor;
    FOptions: TFPReportMemoOptions;
    FLastText: string; // used by moSuppressRepeated
    FOriginal: TFPReportCustomMemo;
    ExpressionNodes: array of TExprNodeInfoRec;
    FFont: TFPReportFont;
    FUseParentFont: Boolean;
    FWordOverflow: TFPReportWordOverflow;
    function    GetParentFont: TFPReportFont;
    procedure   HandleFontChange(Sender: TObject);
    procedure   SetCullThreshold(AValue: TFPReportCullThreshold);
    procedure   SetText(AValue: TFPReportString);
    procedure   SetUseParentFont(AValue: Boolean);
    procedure   SetWordOverflow(AValue: TFPReportWordOverflow);
    procedure   ApplyHorzTextAlignment;
    procedure   ApplyVertTextAlignment;
    function    GetTextLines: TStrings;
    procedure   SetLineSpacing(AValue: TFPReportUnits);
    procedure   HTMLOnFoundTag(NoCaseTag, ActualTag: string);
    procedure   HTMLOnFoundText(Text: string);
    function    PixelsToMM(APixels: single): single; inline;
    function    mmToPixels(mm: single): integer; inline;
    { Result is in millimeters. }
    function TextHeight(const AText, FontName: string; FontSize: Integer; out
      ADescender: TFPReportUnits): TFPReportUnits;
    { Result is in millimeters. }
    function TextWidth(const AText, FontName: string; FontSize: Integer
      ): TFPReportUnits;
    procedure   SetLinkColor(AValue: TFPReportColor);
    procedure   SetTextAlignment(AValue: TFPReportTextAlignment);
    procedure   SetOptions(const AValue: TFPReportMemoOptions);
    procedure   ParseText;
    procedure   ClearExpressionNodes;
    procedure   AddSingleTextBlock(const AText: string);
    procedure   AddMultipleTextBlocks(const AText: string);
    function    IsExprAtArrayPos(const APos: integer): Boolean;
    procedure   SetFont(const AValue: TFPReportFont);
    procedure   CullTextOutOfBounds;
  protected
    procedure   AddTextLine(lFC: TFPFontCacheItem; var S: UTF8String; MaxW: TFPReportUnits);
    procedure   WrapText(const AText: UTF8String; lFC: TFPFontCacheItem; const ALineWidth: TFPReportUnits; out AHeight: TFPReportUnits);  virtual;
    procedure   ReassignParentFont;
    procedure   ParentFontChanged; override;
    function    CreateTextAlignment: TFPReportTextAlignment; virtual;
    function    GetExpr: TFPExpressionParser; virtual;
    procedure   RecalcLayout; override;
    procedure   DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    procedure   ExpandExpressions;
    procedure   UpdateAggregates;
    function    PrepareObject(aRTParent: TFPReportElement): TFPReportElement; override;
    procedure   SetParent(const AValue: TFPReportElement); override;
    property    Text: TFPReportString read FText write SetText;
    property    Font: TFPReportFont read FFont write SetFont;
    property    TextAlignment: TFPReportTextAlignment read FTextAlignment write SetTextAlignment;
    property    LineSpacing: TFPReportUnits read FLineSpacing write SetLineSpacing default 1;
    property    LinkColor: TFPReportColor read FLinkColor write SetLinkColor default clBlue;
    { The moUsesHTML enables supports for <b>, <i>, <font color=yxz bgcolor=xyz> and <a href="..."> tags.
      NOTE: The FONT tag's color attribute will override the FontColor property. }
    property    Options: TFPReportMemoOptions read FOptions write SetOptions default [];
    { Used by Runtime Memos - this is a reference back to the original design memo. }
    property    Original: TFPReportCustomMemo read FOriginal write FOriginal;
    property    UseParentFont: Boolean read FUseParentFont write SetUseParentFont default True;
    { % of line height that should be visible, otherwise it's culled if StretchMode = smDontStretch. Valid range is 1-100% and default is 75%}
    property    CullThreshold: TFPReportCullThreshold read FCullThreshold write SetCullThreshold default 75;
    Property    WordOverflow : TFPReportWordOverflow read FWordOverflow write SetWordOverflow;
  protected
    // *****************************
    //   This block is made Protected simply for Unit Testing purposes.
    //   Interfaces would have worked nicely for this.
    // *****************************

    // --------------->  Start <-----------------
    function    CreateTextBlock(const IsURL: boolean): TFPTextBlock;
    { HtmlColorToFPReportColor() supports RRGGBB, #RRGGBB and #RGB color formats. }
    function    HtmlColorToFPReportColor(AColorStr: string; ADefault: TFPReportColor = clBlack): TFPReportColor;
    procedure   PrepareTextBlocks;
    { Extract a sub-string within defined delimiters. If AIndex is > 0 then extract the
      AIndex'th sub-string. AIndex uses 1-based numbering. The AStartPos returns the position
      of the returned sub-string in ASource. }
    function    SubStr(const ASource, AStartDelim, AEndDelim: string; AIndex: integer; out AStartPos: integer): string;
    { Count the number of blocks of text in AValue separated by AToken }
    function    TokenCount(const AValue: string; const AToken: string = '['): integer;
    { Return the n-th token defined by APos. APos is 1-based. }
    function    Token(const AValue, AToken: string; const APos: integer): string;
    // --------------->  End  <-----------------

  public
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    Class Function ElementType : String; override;
    procedure   Assign(Source: TPersistent); override;
    procedure   ReadElement(AReader: TFPReportStreamer); override;
    { Only returns the internal FTextLines if StretchMode <> smDontStretch, otherwise it returns nil. Don't free the TStrings result. }
    property    TextLines: TStrings read GetTextLines;
    { after layouting, this contains all the memo text and positions they should be displayed at. }
    property    TextBlockList: TFPTextBlockList read FTextBlockList;
  end;


  TFPReportMemo = class(TFPReportCustomMemo)
  published
    property  CullThreshold;
    property  Font;
    property  LineSpacing;
    property  LinkColor;
    property  Options;
    Property  WordOverflow;
    property  StretchMode;
    property  Text;
    property  TextAlignment;
    property  UseParentFont;
    property  OnBeforePrint;
  end;


  { TFPReportCustomShape }

  TFPReportCustomShape = class(TFPReportElement)
  private
    FColor: TFPReportColor;
    FShapeType: TFPReportShapeType;
    FOrientation: TFPReportOrientation;
    FCornerRadius: TFPReportUnits;
    procedure   SetShapeType(AValue: TFPReportShapeType);
    procedure   SetOrientation(AValue: TFPReportOrientation);
    procedure   SetCornerRadius(AValue: TFPReportUnits);
  protected
    Procedure   RecalcLayout; override;
    procedure   DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    property    ShapeType: TFPReportShapeType read FShapeType write SetShapeType default stEllipse;
    property    Orientation: TFPReportOrientation read FOrientation write SetOrientation default orNorth;
    property    CornerRadius: TFPReportUnits read FCornerRadius write SetCornerRadius;
    Property    Color : TFPReportColor Read FColor Write FColor default clBlack;
  public
    constructor Create(AOwner: TComponent); override;
    Class Function ElementType : String; override;
    procedure Assign(Source: TPersistent); override;
    Procedure ReadElement(AReader: TFPReportStreamer); override;
    Function  CreatePropertyHash: String; override;
  end;


  TFPReportShape = class(TFPReportCustomShape)
  published
    property    ShapeType;
    property    Orientation;
    property    CornerRadius;
    property    Color; 
    property    StretchMode;
  end;


  { TFPReportCustomImage }

  TFPReportCustomImage = class(TFPReportElement)
  private
    FImage: TFPCustomImage;
    FStretched: boolean;
    FFieldName: TFPReportString;
    FImageID: integer;
    procedure   SetImage(AValue: TFPCustomImage);
    procedure   SetStretched(AValue: boolean);
    procedure   SetFieldName(AValue: TFPReportString);
    procedure   LoadDBData(AData: TFPReportData);
    procedure   SetImageID(AValue: integer);
    function    GetImage: TFPCustomImage;
  protected
    procedure   DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil); override;
    Procedure   RecalcLayout; override;
    function    PrepareObject(aRTParent: TFPReportElement): TFPReportElement; override;
    property    Image: TFPCustomImage read GetImage write SetImage;
    property    ImageID: integer read FImageID write SetImageID;
    property    Stretched: boolean read FStretched write SetStretched;
    property    FieldName: TFPReportString read FFieldName write SetFieldName;
  public
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    Class Function ElementType : String; override;
    Function    GetRTImageID : Integer;
    Function    GetRTImage : TFPCustomImage;
    procedure   Assign(Source: TPersistent); override;
    procedure   ReadElement(AReader: TFPReportStreamer); override;
    procedure   LoadFromFile(const AFileName: string);
    Procedure   LoadFromStream(const AStream: TStream; aHandler: TFPCustomImageReaderClass);
    procedure   LoadPNGFromStream(AStream: TStream);
    procedure   LoadImage(const AImageData: Pointer; const AImageDataSize: LongWord);
  end;


  TFPReportImage = class(TFPReportCustomImage)
  published
    property    Image;
    property    ImageID;
    property    Stretched;
    property    FieldName;
    property    OnBeforePrint;
  end;


  { TFPReportCustomCheckbox }

  TFPReportCustomCheckbox = class(TFPReportElement)
  private
    FExpression: TFPReportString;
    FFalseImageID: Integer;
    FTrueImageID: Integer;
    procedure   SetExpression(AValue: TFPReportString);
    function    LoadImage(const AImageData: Pointer; const AImageDataSize: LongWord): TFPCustomImage; overload;
    function    LoadImage(AStream: TStream): TFPCustomImage; overload;
  protected
    Class Var
      ImgTrue: TFPCustomImage;
      ImgFalse: TFPCustomImage;
  Protected
    FTestResult: Boolean;
    Procedure   RecalcLayout; override;
    Procedure   DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement=nil); override;
    Function    PrepareObject(aRTParent: TFPReportElement): TFPReportElement; override;
    property    Expression: TFPReportString read FExpression write SetExpression;
  public
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    Class function ElementType: String; override;
    function    GetDefaultImage(Checked: Boolean): TFPCustomImage;
    Function    GetImage(Checked: Boolean) : TFPCustomImage;
    Function    GetRTResult : Boolean;
    Function    GetRTImage : TFPCustomImage;
    Function    CreatePropertyHash: String; override;
    procedure   Assign(Source: TPersistent); override;
    Procedure   ReadElement(AReader: TFPReportStreamer); override;
    Property    TrueImageID : Integer Read FTrueImageID Write FTrueImageID;
    Property    FalseImageID : Integer Read FFalseImageID Write FFalseImageID;
  end;


  TFPReportCheckbox = class(TFPReportCustomCheckbox)
  published
    property    Expression;
    Property    TrueImageID ;
    Property    FalseImageID ;
  end;

  { TFPReportElementEditor }

  TFPReportElementEditor = Class(TComponent)
  private
    FElement: TFPReportElement;
  Protected
    procedure SetElement(AValue: TFPReportElement); virtual;
  Public
    Class function DefaultClass : TFPReportElementClass; virtual;
    Class Procedure RegisterEditor;
    Class Procedure UnRegisterEditor;
    Function Execute: Boolean; virtual; abstract;
    Property Element : TFPReportElement Read FElement Write SetElement;
  end;
  TFPReportElementEditorClass = Class of TFPReportElementEditor;

  { A class to hold the TFPReportElement class mappings. The factory maintains
    a list of these and uses the ReportElementClass property to create the objects. }

  { TFPReportClassMapping }

  TFPReportImageRenderCallBack = Procedure(aElement : TFPReportElement; aImage: TFPCustomImage);
  TFPReportElementExporterCallBack = Procedure(aPos : TFPReportPoint; aElement : TFPReportElement; AExporter : TFPReportExporter; ADPI: Integer);

  TFPReportElementRenderer = Record
    aClass : TFPReportExporterClass;
    aCallback : TFPReportElementExporterCallBack;
  end;
  TFPReportElementRendererArray = Array of TFPReportElementRenderer;

  TFPReportClassMapping = class(TObject)
  private
    FEditorClass: TFPReportElementEditorClass;
    FIConData: TBytes;
    FImageRenderCallBack: TFPReportImageRenderCallBack;
    FMappingName: string;
    FReportElementClass: TFPReportElementClass;
    FRenderers : TFPReportElementRendererArray;
    FStandard: Boolean;
  public
    constructor Create(const AMappingName: string; AElementClass: TFPReportElementClass);
    Procedure SetIconFromBytes(B : Array of Byte);
    Function IndexOfExportRenderer(AClass : TFPReportExporterClass) : Integer;
    Function AddRenderer(aExporterClass : TFPReportExporterClass; aCallback : TFPReportElementExporterCallBack) : TFPReportElementExporterCallBack;
    Function FindRenderer(aClass : TFPReportExporterClass) : TFPReportElementExporterCallBack;
    property MappingName: string read FMappingName;
    Property ImageRenderCallback : TFPReportImageRenderCallBack Read FImageRenderCallBack Write FImageRenderCallBack;
    property ReportElementClass: TFPReportElementClass read FReportElementClass;
    // Class to edit the element visually
    property EditorClass : TFPReportElementEditorClass Read FEditorClass Write FEditorClass;
    // element Icon data in PNG format, will be shown in menu editor.
    property IconData : TBytes Read FIConData Write FIconData;
    Property Standard : Boolean Read FStandard;
  end;


  { Factory pattern - Create a descendant of the TFPReportElement at runtime. }

  { TFPReportElementFactory }

  TFPReportElementFactory = class(TObject)
  private
    FList: TFPObjectList;
    function GetBC(aType : TFPReportBandType): TFPReportCustomBandClass;
    function GetM(Aindex : integer): TFPReportClassMapping;
    function GetMappingCount: Integer;
  Protected
    function IndexOfElementClass(const AElementClass: TFPReportElementClass): Integer;
    Function IndexOfElementName(const AElementName: string) : Integer;
  public
    constructor Create;
    destructor  Destroy; override;
    function GetDefaultBandType(AType: TFPReportBandType): TFPReportCustomBandClass; virtual;
    Function    FindRenderer(aClass : TFPReportExporterClass; AElement : TFPReportElementClass) : TFPReportElementExporterCallBack;
    Function    FindImageRenderer(AElement : TFPReportElementClass) : TFPReportImageRenderCallBack;
    Function    RegisterImageRenderer(AElement : TFPReportElementClass; ARenderer : TFPReportImageRenderCallBack) : TFPReportImageRenderCallBack;
    Function    RegisterElementRenderer(AElement : TFPReportElementClass; ARenderClass: TFPReportExporterClass; ARenderer : TFPReportElementExporterCallBack) : TFPReportElementExporterCallBack;
    procedure   RegisterEditorClass(const AElementName: string; AEditorClass: TFPReportElementEditorClass);
    procedure   RegisterEditorClass(AReportElementClass: TFPReportElementClass; AEditorClass: TFPReportElementEditorClass);
    procedure   UnRegisterEditorClass(const AElementName: string; AEditorClass: TFPReportElementEditorClass);
    procedure   UnRegisterEditorClass(AReportElementClass: TFPReportElementClass; AEditorClass: TFPReportElementEditorClass);
    Function    RegisterClass(const AElementName: string; AReportElementClass: TFPReportElementClass) : TFPReportClassMapping;
    procedure   RemoveClass(const AElementName: string);
    function    CreateInstance(const AElementName: string; AOwner: TComponent): TFPReportElement; overload;
    Function    FindEditorClassForInstance(AInstance : TFPReportElement) : TFPReportElementEditorClass;
    Function    FindEditorClassForInstance(AClass : TFPReportElementClass) : TFPReportElementEditorClass ;
    procedure   AssignReportElementTypes(AStrings: TStrings);
    Function    PageClass : TFPReportCustomPageClass;
    Property    BandClasses[aType : TFPReportBandType] : TFPReportCustomBandClass Read GetBC;
    Property  Mappings[Aindex : integer] : TFPReportClassMapping read GetM;
    Property  MappingCount : Integer Read GetMappingCount;
  end;

  { keeps track of interested bands. eg: a list of page header like bands etc. }

  { TBandList }

  TBandList = class(TObject)
  private
    FList: TFPList;
    function    GetCount: Integer;
    function    GetItems(AIndex: Integer): TFPReportCustomBand;
    procedure   SetItems(AIndex: Integer; AValue: TFPReportCustomBand);
  public
    constructor Create;
    destructor  Destroy; override;
    procedure   Remove(AItem: TFPReportCustomBand);
    function    Add(AItem: TFPReportCustomBand): Integer;
    procedure   Clear;
    procedure   Delete(AIndex: Integer);
    function    Find(ABand: TFPReportBandClass): TFPReportCustomBand; overload;
    function    Find(ABand: TFPReportBandClass; out AResult: TFPReportCustomBand): Integer; overload;
    procedure   Sort(Compare: TListSortCompare);
    property    Count: Integer read GetCount;
    property    Items[AIndex: Integer]: TFPReportCustomBand read GetItems write SetItems; default;
  end;

  { TFPReportExportManager }

  TFPReportExportManager = Class(TComponent)
  Private
    Flist : TFPObjectList;
    FOnConfigCallBack: TFPReportExporterConfigHandler;
    function GetExporter(AIndex : Integer): TFPReportExporterClass;
    function GetExporterCount: Integer;
  Protected
    Procedure RegisterExport(AClass : TFPReportExporterClass); virtual;
    Procedure UnRegisterExport(AClass : TFPReportExporterClass); virtual;
    Function ConfigExporter(AExporter : TFPReportExporter) : Boolean; virtual;
  Public
    Constructor Create(AOwner : TComponent);override;
    Destructor Destroy; override;
    Procedure Clear;
    Function IndexOfExporter(Const AName : String) : Integer;
    Function IndexOfExporter(Const AClass : TFPReportExporterClass) : Integer;
    Function FindExporter(Const AName : String) : TFPReportExporterClass;
    Function ExporterConfigHandler(Const AClass : TFPReportExporterClass) : TFPReportExporterConfigHandler;
    Procedure RegisterConfigHandler(Const AName : String; ACallBack : TFPReportExporterConfigHandler);
    Property Exporter[AIndex : Integer] : TFPReportExporterClass Read GetExporter;
    Property ExporterCount : Integer Read GetExporterCount;
    // GLobal one, called when no specific callback is configured
    Property OnConfigCallBack : TFPReportExporterConfigHandler Read FOnConfigCallBack Write FOnConfigCallBack;
  end;

procedure ReportError(Msg: string); inline;
procedure ReportError(Fmt: string; Args: array of const);
function  HorzTextAlignmentToString(AEnum: TFPReportHorzTextAlignment): string; inline;
function  StringToHorzTextAlignment(AName: string): TFPReportHorzTextAlignment; inline;
function  VertTextAlignmentToString(AEnum: TFPReportVertTextAlignment): string; inline;
function  StringToVertTextAlignment(AName: string): TFPReportVertTextAlignment; inline;
function  ReportSectionsToString(AValue: TFPReportSections): string; inline;
function  StringToReportSections(AValue: string): TFPReportSections; inline;
function  ReportSectionToString(AEnum: TFPReportSection): string; inline;
function  StringToReportSection(AName: string): TFPReportSection; inline;
{ Converts R, G, B color channel values into the TFPReportColor (RRGGBB format) type. }
function RGBToReportColor(R, G, B: Byte): TFPReportColor;
{ Base64 encode stream data }
function FPReportStreamToMIMEEncodeString(const AStream: TStream): string;
{ Base64 decode string to a stream }
procedure FPReportMIMEEncodeStringToStream(const AString: string; const AStream: TStream);

function PaperManager: TFPReportPaperManager;

// The ElementFactory is a singleton
function gElementFactory: TFPReportElementFactory;

Function ReportExportManager : TFPReportExportManager;

{ this should probably be more configurable or flexible per platform }

Const

  { Note, these are the postscript names, not the human-readable ones. }

{$IFDEF UNIX}
  cDefaultFont = 'LiberationSans';
{$ELSE}
{$IFDEF WINDOWS}
  cDefaultFont = 'ArialMT';
{$ELSE}
  cDefaultFont = 'Helvetica';
{$ENDIF}
{$ENDIF}

Var
  ReportDefaultFont : string = cDefaultFont;

implementation

uses
  strutils,
  typinfo,
  FPReadPNG,
  FPWritePNG,
  base64;

resourcestring
  cPageCountMarker = '~PC~';

  SErrInvalidLineWidth   = 'Invalid line width: %d';
  SErrInvalidParent      = '%s cannot be used as a parent for %s';
  SErrInvalidChildIndex  = 'Invalid child index : %d';
  SErrInvalidPageIndex   = 'Invalid page index : %d';
  SErrNotAReportPage     = '%s (%s) is not a TFPReportCustomPage.';
  SErrDuplicatePaperName = 'Paper name %s already exists';
  SErrUnknownPaper       = 'Unknown paper name : "%s"';
  SErrUnknownField       = '%s: No such field : "%s"';
  SErrInitFieldsNotAllowedAfterOpen =
    'Calling InitDataFields to change the Datafields collection after Open() is not allowed.';
  SErrUnknownMacro       = '**unknown**';
  //SErrNoFileFound        = 'No file found: "%s"';
  SErrChildBandCircularReference = 'ChildBand circular reference detected and not allowed.';
  SErrFontNotFound       = 'Font not found: "%s"';
  SErrNeedPages = 'Need at least 1 report page.';
  SErrInvalidReport = 'Invalid report, detected %d errors:'+sLineBreak+'%s';
  SErrEmptyGroupExpression = 'Group header "%s" needs a group expression';
  SErrNoPageForBand = 'No page for band "%s".';
  SErrDanglingChild = 'Child band "%s" is not used by other bands.';

  SErrRegisterEmptyExporter     = 'Attempt to register empty exporter';
  SErrRegisterDuplicateExporter = 'Attempt to register duplicate exporter: "%s"';
  SErrRegisterUnknownElement = 'Unable to find registered report element <%s>.';
  SErrUnknownExporter = 'Unknown exporter: "%s"';
  SErrMultipleDataBands = 'A report page may not have more than one master databand.';
  // SErrCantAssignReportFont = 'Can''t Assign() report font - Source is not TFPReportFont.';
  SErrNoStreamInstanceWasSupplied = 'No valid TStream instance was supplied.';
  SErrIncorrectDescendant = 'AElement is not a TFPReportElementWithChildren descendant.';

  SErrUnknownResultType = 'Unknown result type: "%s"';
  SErrInvalidFloatingPointValue = '%s is not a valid floating point value';
  SErrResultTypeMisMatch = 'Result type is %s, expected %s';
  SErrDuplicateVariable = 'Duplicate variable name : %s';
  SErrInvalidVariableName = 'Invalid variable name: "%s"';
  SErrUnknownBandType = 'Unknown band type : %d';
  SErrInInvalidISO8601DateTime = '%s is an invalid ISO datetime value';
  SErrCouldNotGetDefaultBandType = 'Could not get default band class for type '
    +'%s';
  SErrBandClassMustDescendFrom = 'Band class for band type %s must descend '
    +'from %s';
  SErrPageClassMustDescendFrom = 'Page class for must descend from %s';
  SErrCannotRegisterWithoutDefaultClass = 'Cannot register/unregister editor without default element class.';
  SErrUnknownElementName = 'Unknown element name : %s';
  SErrUnknownElementClass = 'Unknown element class : %s';
  SErrResetGroupMissing = 'ResetType is rtGroup but no ResetGroup specified';
  SErrEmptyResetValue = 'ResetType is specified, but no ResetExpression is provided';
  SErrExprVariableAggregateOnWrongLevel= 'ExprVariable has Aggregate but not on highest level: %s';
  SErrAggregateWithoutDataName = 'ExprVariable has Aggregate but cannot determine data source: %s';

{ includes Report Checkbox element images }
{$I fpreportcheckbox.inc}

var
  uPaperManager: TFPReportPaperManager;
  uElementFactory: TFPReportElementFactory;
  EM : TFPReportExportManager;

{ Auxiliary routines }

Function SafeVariant(V : Variant) : String;

begin
  if VarIsNull(V) then
    Result:='Null'
  else
    Result:=V;
end;

function DefExpressionResultToString(const Res : TFPExpressionResult): String;

begin
  case Res.ResultType of
    rtString  : Result := Res.ResString;
    rtInteger : Result := IntToStr(Res.ResInteger);
    rtFloat   : Result := FloatToStr(Res.ResFloat);
    rtCurrency  : Result := CurrToStr(Res.ResCurrency);
    rtBoolean : Result := BoolToStr(Res.resBoolean, True);
    rtDateTime : Result := FormatDateTime('yyyy-mm-dd"T"hh:nn:ss', Res.resDateTime);
  end;
end;

procedure ReportError(Msg: string); inline;
begin
  raise EReportError.Create(Msg);
end;

procedure ReportError(Fmt: string; Args: array of const);
begin
  raise EReportError.CreateFmt(Fmt, Args);
end;

function PaperManager: TFPReportPaperManager;
begin
  if uPaperManager = nil then
    uPaperManager := TFPReportPaperManager.Create(nil);
  Result := uPaperManager;
end;

function gElementFactory: TFPReportElementFactory;
begin
  if uElementFactory = nil then
    uElementFactory := TFPReportElementFactory.Create;
  Result := uElementFactory;
end;


function ReportExportManager: TFPReportExportManager;
begin
  If EM=Nil then
    EM:=TFPReportExportManager.Create(Nil);
  Result:=EM;
end;

// TODO: See if the following generic function can replace the multiple enum-to-string functions
//generic function EnumValueAsName<T>(v: T): String;
//begin
//  Result := GetEnumName(TypeInfo(T), LongInt(v));
//end;

function HorzTextAlignmentToString(AEnum: TFPReportHorzTextAlignment): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportHorzTextAlignment), Ord(AEnum));
end;

function StringToHorzTextAlignment(AName: string): TFPReportHorzTextAlignment; inline;
begin
  Result := TFPReportHorzTextAlignment(GetEnumValue(TypeInfo(TFPReportHorzTextAlignment), AName));
end;

function VertTextAlignmentToString(AEnum: TFPReportVertTextAlignment): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportVertTextAlignment), Ord(AEnum));
end;

function StringToVertTextAlignment(AName: string): TFPReportVertTextAlignment; inline;
begin
  Result := TFPReportVertTextAlignment(GetEnumValue(TypeInfo(TFPReportVertTextAlignment), AName));
end;

function ReportSectionsToString(AValue: TFPReportSections): string;
  var
    lIndex: integer;
  begin
    Result := '';
    for lIndex := Ord(Low(TFPReportSections)) to Ord(High(TFPReportSections)) do
    begin
      if TFPReportSection(lIndex) in AValue then
      begin
        if Result = '' then
          Result := GetEnumName(TypeInfo(TFPReportSection), lIndex)
        else
          Result := Result + ',' + GetEnumName(TypeInfo(TFPReportSection), lIndex);
      end;
    end;
end;

function StringToReportSections(AValue: string): TFPReportSections;
  var
    lList: TStrings;
    lIndex: integer;
  begin
    Result := [];
    lList := nil;
    lList := TStringList.Create;
    try
      lList.Delimiter := ',';
      lList.DelimitedText := AValue;
      for lIndex := 0 to lList.Count - 1 do
        Include(Result, TFPReportSection(GetEnumValue(TypeInfo(TFPReportSection), lList[lIndex])));
    finally
      lList.Free;
    end;
end;

function ReportSectionToString(AEnum: TFPReportSection): string;
begin
  result := GetEnumName(TypeInfo(TFPReportSection), Ord(AEnum));
end;

function StringToReportSection(AName: string): TFPReportSection;
begin
  Result := TFPReportSection(GetEnumValue(TypeInfo(TFPReportSection), AName));
end;

function RGBToReportColor(R, G, B: Byte): TFPReportColor;
begin
  Result := (R shl 16) or (G shl 8) or B;
end;

function QWordToReportColor(AQWord: QWord):TFPReportColor;
begin
  Result := TFPReportColor(AQWord and $FFFFFFFF);
end;

function StretchModeToString(AEnum: TFPReportStretchMode): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportStretchMode), Ord(AEnum));
end;

function StringToStretchMode(AName: string): TFPReportStretchMode; inline;
begin
  Result := TFPReportStretchMode(GetEnumValue(TypeInfo(TFPReportStretchMode), AName));
end;

function ShapeTypeToString(AEnum: TFPReportShapeType): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportShapeType), Ord(AEnum));
end;

function StringToShapeType(AName: string): TFPReportShapeType; inline;
begin
  Result := TFPReportShapeType(GetEnumValue(TypeInfo(TFPReportShapeType), AName));
end;

function FrameShapeToString(AEnum: TFPReportFrameShape): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportFrameShape), Ord(AEnum));
end;

function StringToFrameShape(AName: string): TFPReportFrameShape; inline;
begin
  Result := TFPReportFrameShape(GetEnumValue(TypeInfo(TFPReportFrameShape), AName));
end;

function FramePenToString(AEnum: TFPPenStyle): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPPenStyle), Ord(AEnum));
end;

function StringToFramePen(AName: string): TFPPenStyle; inline;
begin
  Result := TFPPenStyle(GetEnumValue(TypeInfo(TFPPenStyle), AName));
end;

function ColumnLayoutToString(AEnum: TFPReportColumnLayout): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportColumnLayout), Ord(AEnum));
end;

function StringToColumnLayout(AName: string): TFPReportColumnLayout; inline;
begin
  Result := TFPReportColumnLayout(GetEnumValue(TypeInfo(TFPReportColumnLayout), AName));
end;

function OrientationToString(AEnum: TFPReportOrientation): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportOrientation), Ord(AEnum));
end;

function StringToOrientation(AName: string): TFPReportOrientation; inline;
begin
  Result := TFPReportOrientation(GetEnumValue(TypeInfo(TFPReportOrientation), AName));
end;

function PaperOrientationToString(AEnum: TFPReportPaperOrientation): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportPaperOrientation), Ord(AEnum));
end;

function StringToPaperOrientation(AName: string): TFPReportPaperOrientation; inline;
begin
  Result := TFPReportPaperOrientation(GetEnumValue(TypeInfo(TFPReportPaperOrientation), AName));
end;

function BandPositionToString(AEnum: TFPReportBandPosition): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportBandPosition), Ord(AEnum));
end;

function StringToBandPosition(AName: string): TFPReportBandPosition; inline;
begin
  Result := TFPReportBandPosition(GetEnumValue(TypeInfo(TFPReportBandPosition), AName));
end;

function VisibleOnPageToString(AEnum: TFPReportVisibleOnPage): string; inline;
begin
  result := GetEnumName(TypeInfo(TFPReportVisibleOnPage), Ord(AEnum));
end;

function StringToVisibleOnPage(AName: string): TFPReportVisibleOnPage; inline;
begin
  Result := TFPReportVisibleOnPage(GetEnumValue(TypeInfo(TFPReportVisibleOnPage), AName));
end;

function StringToMemoOptions(const AValue: string): TFPReportMemoOptions;
var
  lList: TStrings;
  lIndex: integer;
begin
  Result := [];
  lList := nil;
  lList := TStringList.Create;
  try
    lList.Delimiter := ',';
    lList.DelimitedText := AValue;
    for lIndex := 0 to lList.Count - 1 do
      Include(Result, TFPReportMemoOption(GetEnumValue(TypeInfo(TFPReportMemoOption), lList[lIndex])));
  finally
    lList.Free;
  end;
end;

function MemoOptionsToString(const AValue: TFPReportMemoOptions): String;
var
  lIndex: integer;
begin
  Result := '';
  for lIndex := Ord(Low(TFPReportMemoOption)) to Ord(High(TFPReportMemoOption)) do
  begin
    if TFPReportMemoOption(lIndex) in AValue then
    begin
      if Result = '' then
        Result := GetEnumName(TypeInfo(TFPReportMemoOption), lIndex)
      else
        Result := Result + ',' + GetEnumName(TypeInfo(TFPReportMemoOption), lIndex);
    end;
  end;
end;

function FPReportStreamToMIMEEncodeString(const AStream: TStream): string;
var
  OutStream: TStringStream;
  b64encoder: TBase64EncodingStream;
  LPos: integer;
begin
  if not Assigned(AStream) then
    ReportError(SErrNoStreamInstanceWasSupplied);
  LPos:= AStream.Position;
  try
    OutStream := TStringStream.Create('');
    try
      AStream.Position := 0;

      b64encoder := TBase64EncodingStream.Create(OutStream);
      b64encoder.CopyFrom(AStream, AStream.Size);
    finally
      b64encoder.Free;
      result := OutStream.DataString;
      OutStream.Free;
    end;
  finally
    AStream.Position:= LPos;
  end;
end;

procedure FPReportMIMEEncodeStringToStream(const AString: string; const AStream: TStream);
var
  InputStream: TStringStream;
  b64decoder: TBase64DecodingStream;
begin
  if not Assigned(AStream) then
    ReportError(SErrNoStreamInstanceWasSupplied);
  InputStream:= TStringStream.Create(AString);
  try
    AStream.Size := 0;
    b64decoder := TBase64DecodingStream.Create(InputStream, bdmMIME);
    try
      AStream.CopyFrom(b64decoder, b64decoder.Size);
      AStream.Position:=0;
    finally
      b64decoder.Free;
    end;
  finally
    InputStream.Free;
  end;
end;

procedure FillMem(Dest: pointer; Size: longint; Data: Byte );
begin
  FillChar(Dest^, Size, Data);
end;

function SortDataBands(Item1, Item2: Pointer): Integer;
begin
  if TLoopData(Item1).FDataBand.DisplayPosition < TLoopData(Item2).FDataBand.DisplayPosition then
    Result := -1
  else if TLoopData(Item1).FDataBand.DisplayPosition > TLoopData(Item2).FDataBand.DisplayPosition then
    Result := 1
  else
    Result := 0;
end;

Type
  { TFPReportReg }
  TFPReportExportReg = Class(TObject)
  private
    FClass: TFPReportExporterClass;
    FonConfig: TFPReportExporterConfigHandler;
  Public
    Constructor Create(AClass : TFPReportExporterClass);
    Property TheClass : TFPReportExporterClass Read FClass;
    Property OnConfig : TFPReportExporterConfigHandler Read FonConfig Write FOnConfig;
  end;

const
  { Summary of ISO 8601  http://www.cl.cam.ac.uk/~mgk25/iso-time.html }
  ISO8601DateFormat = 'yyyymmdd"T"hhnnss';    // for storage

function DateTimeToISO8601(const ADateTime: TDateTime): string;
begin
  Result := FormatDateTime(ISO8601DateFormat,ADateTime);
  if Pos('18991230', Result) = 1 then
    begin
    Delete(Result,1,8);
    Result:='00000000'+Result;
    end;
end;

function ISO8601ToDateTime(const AValue: string): TDateTime;
var
  lY, lM, lD, lH, lMi, lS: Integer;
begin
  if Trim(AValue) = '' then
  begin
    Result := 0;
    Exit; //==>
  end;

    //          1         2
    // 12345678901234567890123
    // yyyymmddThhnnss
  if not (TryStrToInt(Copy(AValue, 1, 4),lY)
      and TryStrToInt(Copy(AValue, 5, 2),lM)
      and TryStrToInt(Copy(AValue, 7, 2),lD)
      and TryStrToInt(Copy(AValue, 10, 2),lH)
      and TryStrToInt(Copy(AValue, 12, 2),lMi)
      and TryStrToInt(Copy(AValue, 14, 2),lS)) then
      raise EConvertError.CreateFmt(SErrInInvalidISO8601DateTime, [AValue]);

  { Cannot EncodeDate if any part equals 0. EncodeTime is okay. }
  if (lY = 0) or (lM = 0) or (lD = 0) then
    Result := EncodeTime(lH, lMi, lS, 0)
  else
    Result := EncodeDate(lY, lM, lD) + EncodeTime(lH, lMi, lS, 0);
end;

{ TLoopData }


procedure TLoopData.SetParentLoop(AValue: TLoopData);
begin
  if FParentLoop=AValue then Exit;
  FParentLoop:=AValue;
end;

procedure TLoopData.PrepareBandSubLoops(APage: TFPReportCustomPage);

Var
  lDetailBand : TFPReportCustomBand;
  DBand : TFPReportCustomDataBand;
  I : integer;
  LD : TLoopData;

begin
  { collect bands of interest }
  for i := 0 to aPage.BandCount-1 do
    begin
    lDetailBand := aPage.Bands[i];
    if (lDetailBand is TFPReportCustomDataBand) then
      begin
      DBand:=lDetailBand as TFPReportCustomDataBand;
      if (DBand.MasterBand=FDataBand) and (DBand.Data <> nil) then
        begin
        LD:=TLoopData.Create(DBand.Data);
        FSubLoops.Add(LD);
        LD.PrepareGroups(aPage);
        LD.CollectDataBands(aPage);
        // Recurse
        LD.PrepareBandSubLoops(aPage);
        end;
      end;
    end;
  if FSubLoops.Count<>0 then
    FSubLoops.Sort(@SortDataBands);
end;

procedure TLoopData.SetDetailsPrinted;

Var
  I : Integer;

begin
  for i := 0 to FGroupHeaderList.Count-1 do
    TFPReportCustomGroupHeaderBand(FGroupHeaderList[i]).FDetailsPrinted := True;
end;

procedure TLoopData.ResetGroups;

Var
  I : Integer;

begin
  for i := 0 to FGroupHeaderList.Count-1 do
    TFPReportCustomGroupHeaderBand(FGroupHeaderList[i]).ResetGroupConditionValues;
end;

constructor TLoopData.Create(aData: TFPReportData);
begin
  FData:=aData;
  FGroupFooterList := TBandList.Create;
  FGroupHeaderList := TBandList.Create;
  FSubLoops:=TFPObjectList.Create;
end;

destructor TLoopData.Destroy;
begin
  FData:=Nil;
  FreeAndNil(FSubLoops);
  FreeAndNil(FGroupFooterList);
  FreeAndNil(FGroupHeaderList);
  inherited Destroy;
end;


{ TFPReportElementEditor }

procedure TFPReportElementEditor.SetElement(AValue: TFPReportElement);
begin
  if FElement=AValue then Exit;
  FElement:=AValue;
end;

class function TFPReportElementEditor.DefaultClass: TFPReportElementClass;
begin
  Result:=Nil;
end;

class procedure TFPReportElementEditor.RegisterEditor;

Var
  C : TFPReportElementClass;

begin
  C:=DefaultClass;
  If C=Nil then
    Raise EReportError.Create(SErrCannotRegisterWithoutDefaultClass);
  gElementFactory.RegisterEditorClass(C,Self);
end;

class procedure TFPReportElementEditor.UnRegisterEditor;
Var
  C : TFPReportElementClass;

begin
  C:=DefaultClass;
  If C=Nil then
    Raise EReportError.Create(SErrCannotRegisterWithoutDefaultClass);
  gElementFactory.UnRegisterEditorClass(C,Self);
end;

{ TFPReportDataCollection }

function TFPReportDataCollection.GetData(AIndex : Integer): TFPReportDataItem;
begin
  Result:=Items[Aindex] as TFPReportDataItem;
end;

procedure TFPReportDataCollection.SetData(AIndex : Integer;
  AValue: TFPReportDataItem);
begin
  Items[Aindex]:=AValue;
end;

function TFPReportDataCollection.IndexOfReportData(AData: TFPReportData
  ): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (GetData(Result).Data<>AData) do
    Dec(Result);
end;

function TFPReportDataCollection.IndexOfReportData(const ADataName: String
  ): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and ((GetData(Result).Data=Nil) or (CompareText(GetData(Result).Data.Name,ADataName)<>0)) do
    Dec(Result);
end;

function TFPReportDataCollection.FindReportDataItem(const ADataName: String): TFPReportDataItem;

Var
  I : Integer;

begin
  I:=IndexOfReportData(ADataName);
  if I=-1 then
    Result:=Nil
  else
    Result:=GetData(I);
end;

function TFPReportDataCollection.FindReportDataItem(AData: TFPReportData): TFPReportDataItem;
Var
  I : Integer;
begin
  I:=IndexOfReportData(AData);
  if I=-1 then
    Result:=Nil
  else
    Result:=GetData(I);
end;

function TFPReportDataCollection.FindReportData(const ADataName: String): TFPReportData;
Var
  I : TFPReportDataItem;
begin
  I:=FindReportDataItem(aDataName);
  If Assigned(I) then
    Result:=I.Data
  else
    Result:=Nil;
end;

function TFPReportDataCollection.AddReportData(AData: TFPReportData ): TFPReportDataItem;
begin
  Result:=Add as TFPReportDataItem;
  Result.Data:=AData;
end;

{ TFPReportDataItem }

procedure TFPReportDataItem.SetData(AValue: TFPReportData);
begin
  if FData=AValue then Exit;
  FData:=AValue;
end;

function TFPReportDataItem.GetDisplayName: string;
begin
  if Assigned(Data) then
    Result:=Data.Name
  else
    Result:=inherited GetDisplayName;
end;

procedure TFPReportDataItem.Assign(Source: TPersistent);
begin
  if Source is TFPReportDataItem then
    FData:=TFPReportDataItem(Source).Data
  else
    inherited Assign(Source);
end;

{ TFPReportVariables }

function TFPReportVariables.GetV(aIndex : Integer): TFPReportVariable;
begin
  Result:=Items[aIndex] as TFPReportVariable;
end;

procedure TFPReportVariables.SetV(aIndex : Integer; AValue: TFPReportVariable);
begin
  Items[aIndex]:=AValue;
end;

procedure TFPReportVariables.ReleaseExpressionNodes;
var
  i: Integer;
begin
  for i:=0 to Count-1 do
    GetV(i).ReleaseExpressionNodes;
end;

procedure TFPReportVariables.InitExpressionValues(aData: TFPReportData; isFirstPass: Boolean);

var
  i: Integer;

begin
  for i:=0 to Count-1 do
    GetV(i).InitExpressionValue(aData,isFirstPass);
end;

procedure TFPReportVariables.DoneExpressionValues(aData: TFPReportData; isFirstPass: Boolean);
var
  i: Integer;
begin
  for i:=0 to Count-1 do
    GetV(i).DoneExpressionValue(aData,isFirstPass);
end;

procedure TFPReportVariables.UpdateExpressionValues(aData : TFPReportData; isFirstPass: Boolean);
var
  i: Integer;
begin
  for i:=0 to Count-1 do
    GetV(i).UpdateExpressionValue(aData,isFirstPass);
end;

function TFPReportVariables.IndexOfVariable(aName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(getV(Result).Name,aName)<>0) do
    Dec(Result);
end;

function TFPReportVariables.FindVariable(aName: String): TFPReportVariable;

Var
  I : Integer;

begin
  I:=IndexOfVariable(aName);
  if I=-1 then
    Result:=nil
  else
    Result:=getV(I);
end;

function TFPReportVariables.AddVariable(aName: String): TFPReportVariable;
begin
  if (IndexOfVariable(aName)<>-1) then
    raise EReportError.CreateFmt(SErrDuplicateVariable, [aName]);
  Result:=add as TFPReportVariable;
  with Result do
  begin
    Name:=aName;
    DataType:=rtString;
    AsString:='';
  end;
end;

function TFPReportVariables.AddDataVariable(aName: String): TFPReportVariable;
begin
  if (IndexOfVariable(aName)<>-1) then
    raise EReportError.CreateFmt(SErrDuplicateVariable, [aName]);
  Result:=add as TFPReportVariable;
  with Result do
  begin
    Name:=aName;
  end;
end;

function TFPReportVariables.AddExprVariable(aName: String; aExpr: String; aType: TResultType; aResetGroup: TFPReportCustomGroupHeaderBand): TFPReportVariable;
var
  lGrp: TFPReportCustomGroupHeaderBand;
  lResetValueExpression: String;

  procedure ExtendResetValueExpression(aExtention: String; aAtEnd: Boolean);
  begin
    if lResetValueExpression='' then
      lResetValueExpression:=aExtention
    else
      if aAtEnd then
        lResetValueExpression:=lResetValueExpression+'+'+aExtention
      else
        lResetValueExpression:=aExtention+'+'+lResetValueExpression;
  end;

begin
  if not Assigned(aResetGroup) then
    raise EReportError.Create(SErrResetGroupMissing);
  lResetValueExpression:='';
  lGrp:=aResetGroup;
  while Assigned(lGrp) do
    begin
    ExtendResetValueExpression(lGrp.GroupCondition, false);
    lGrp:=lGrp.ParentGroupHeader;
    end;
  Result := AddExprVariable(aName, aExpr, aType, rtGroup, lResetValueExpression);
end;

function TFPReportVariables.AddExprVariable(aName: String; aExpr: String;
  aType: TResultType; aResetType: TFPReportResetType;
  aResetValueExpression: String): TFPReportVariable;
begin
  if (IndexOfVariable(aName)<>-1) then
    raise EReportError.CreateFmt(SErrDuplicateVariable, [aName]);
  Result:=add as TFPReportVariable;
  if (aResetValueExpression='') then
    case aResetType of
      rtPage:   aResetValueExpression:='PageNo';
      rtColumn: aResetValueExpression:='ColNo';
    end;
  if (aResetType<>rtNone) and (aResetValueExpression='') then
    raise EReportError.CreateFmt(SErrEmptyResetValue, [aName]);
  with Result do
    begin
    Name:=aName;
    FExpression:=aExpr;
    DataType:=aType;
    ResetType:=aResetType;
    ResetValueExpression:=aResetValueExpression;
    end;
end;

{ TFPReportVariable }

procedure TFPReportVariable.SetValue(AValue: String);

Var
  C : Integer;
  f : TExprFloat;
  CC : Currency;

begin
  if GetValue=AValue then
    Exit;
  if (AValue<>'') then
    Case DataType of
      rtBoolean  : AsBoolean:=StrToBool(AValue);
      rtInteger  : AsInteger:=StrToInt(AValue);
      rtFloat    : begin
                   Val(AValue,F,C);
                   if C<>0 then
                     raise EConvertError.CreateFmt(
                       SErrInvalidFloatingPointValue, [AValue]);
                   ASFloat:=F;
                   end;
      rtCurrency : begin
                   Val(AValue,CC,C);
                   if C<>0 then
                     raise EConvertError.CreateFmt(
                       SErrInvalidFloatingPointValue, [AValue]);
                   AsCurrency:=CC;
                   end;
      rtDateTime : asDateTime:=ISO8601ToDateTime(AValue);
      rtString   : AsString:=AValue;
    else
      raise EConvertError.CreateFmt(SErrUnknownResultType, [GetEnumName(TypeInfo
        (TResultType), Ord(DataType))])
    end;
end;

procedure TFPReportVariable.GetRTValue(Var Result: TFPExpressionResult; ConstRef AName: ShortString);
begin
  if (Result.ResultType=Self.DataType) then
    Result:=FValue;
end;

procedure TFPReportVariable.GetRTExpressionValue(
  Var Result: TFPExpressionResult; ConstRef AName: ShortString);
var
  lRpt: TFPCustomReport;
begin
  lRpt := Collection.Owner as TFPCustomReport;
  if lRpt.FRTUsePrevVariableValues {or lRpt.FLoopData.EOF} then
    Result:=FLastValue
  else
    Result:=FAggregateValue;
end;

constructor TFPReportVariable.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FExpressionNode:=nil;
  FAggregateValues:=TList.Create;
  FLastRecordNo:=-1;
end;

destructor TFPReportVariable.Destroy;
begin
  ClearAggregateValues;
  FAggregateValues.Free;
  ReleaseExpressionNodes;
  inherited Destroy;
end;

procedure TFPReportVariable.ClearAggregateValues;

var
  lResult: PFPExpressionResult;

begin
  While FAggregateValues.Count >0 do
  begin
    lResult:=PFPExpressionResult(FAggregateValues.Last);
    Dispose(lResult);
    FAggregateValues.Delete(FAggregateValues.Count-1);
  end;
end;

procedure TFPReportVariable.SaveValue;
begin
  FSavedValue:=FValue;
end;

procedure TFPReportVariable.RestoreValue;
begin
  FValue:=FSavedValue;
end;

procedure TFPReportVariable.ReleaseExpressionNodes;
begin
  FreeAndNil(FExpressionNode);
  FreeAndNil(FResetValueExpressionNode);
  FResetValue:='';
end;

procedure TFPReportVariable.InitializeExpression(Expr : TFPExpressionParser; AData : TFPReportDataCollection; IsFirstpass : Boolean);

begin
  FResetValue:=#0;
  fAggregateValuesIndex:=0;
  if Not IsFirstPass then
    exit;
  ClearAggregateValues;
  Expr.Expression:=Expression;
  Expr.ExtractNode(FExpressionNode);
  FIsAggregate:=FExpressionNode.IsAggregate;
  if FExpressionNode.HasAggregate and
     not FExpressionNode.IsAggregate then
    raise EReportError.CreateFmt(SErrExprVariableAggregateOnWrongLevel, [FExpressionNode.AsString]);
  if FIsAggregate then
    begin
    if FDataName='' then
      ExtractDataName(AData);
    if (FDataName='') then
      raise EReportError.CreateFmt(SErrAggregateWithoutDataName, [FExpressionNode.AsString]);
    FExpressionNode.InitAggregate;
    end
  else
    begin
    FResetType:=rtNone;
    FResetValueExpression:='';
    end;
  if ResetValueExpression<>'' then
    begin
    Expr.Expression := ResetValueExpression;
    Expr.ExtractNode(FResetValueExpressionNode);
    end;
end;

procedure TFPReportVariable.ExtractDataName(aData : TFPReportDataCollection; ANode : TFPExprNode);

Var
  L,I : Integer;
  DS : String;

begin
  if (aNode is TFPExprVariable) then
    begin
    DS:=ExtractWord(1,TFPExprVariable(ANode).Identifier.Name,['.']);
      If AData.FindReportData(DS)<>Nil then
      FDataName:=DS;
      end
  else if (ANode is TFPExprFunction) then
    begin
    I:=0;
    L:=Length(TFPExprFunction(ANode).ArgumentNodes);
    While (I<L) and (FDataName='') do
      begin
      ExtractDataName(aData,TFPExprFunction(ANode).ArgumentNodes[i]);
      Inc(I);
      end;
    end
  else if (ANode is TFPBinaryOperation) then
    begin
    ExtractDataName(aData,TFPBinaryOperation(ANode).left);
    if FDataName='' then
      ExtractDataName(aData,TFPBinaryOperation(ANode).Right);
    end
  else if (ANode is TFPUnaryOperator) then
    ExtractDataName(aData,TFPUnaryOperator(ANode).Operand);
end;

procedure TFPReportVariable.ExtractDataName(aData : TFPReportDataCollection);
begin
  ExtractDataName(aData,FExpressionNode);
  {$ifdef gdebug}
  Writeln('Expr ',Expression,'-> Data name ',FDataName);
  {$endif}
end;

function TFPReportVariable.GetValue: String;
begin
  Case DataType of
    rtBoolean  : Result:=BoolToStr(AsBoolean,True);
    rtInteger  : Result:=IntToStr(AsInteger);
    rtFloat    : Str(AsFloat,Result);
    rtCurrency : Str(AsCurrency,Result);
    rtDateTime : Result:=DateTimeToISO8601(AsDateTime);
    rtString   : Result:=AsString
  else
    Raise EConvertError.CreateFmt(SErrUnknownResultType,[GetEnumName(TypeInfo(TResultType),Ord(DataType))])
  end;
end;

function TFPReportVariable.GetER: TFPExpressionResult;

begin
  Result:=FValue;
end;

procedure TFPReportVariable.CheckType(aType: TResultType);
begin
  if DataType<>aType then
    raise EConvertError.CreateFmt(SErrResultTypeMisMatch, [
       GetEnumName(TypeInfo(TResultType),Ord(DataType)),
       GetEnumName(TypeInfo(TResultType),Ord(aType))
    ]);
end;

function TFPReportVariable.GetAsInteger: Int64;
begin
  CheckType(rtInteger);
  Result:=FValue.ResInteger;
end;

function TFPReportVariable.GetAsBoolean: Boolean;
begin
  CheckType(rtBoolean);
  Result:=FValue.Resboolean;
end;

function TFPReportVariable.GetAsCurrency: Currency;
begin
  CheckType(rtCurrency);
  Result:=FValue.ResCurrency;
end;

function TFPReportVariable.GetAsDateTime: TDateTime;
begin
  CheckType(rtDateTime);
  Result:=FValue.ResDateTime;
end;

function TFPReportVariable.GetAsFloat: TexprFloat;
begin
  CheckType(rtFloat);
  Result:=FValue.ResFloat;
end;

function TFPReportVariable.GetAsString: String;
begin
  CheckType(rtString);
  Result:=FValue.ResString;
end;

function TFPReportVariable.GetDataType: TResultType;
begin
  Result:=FValue.ResultType;
end;

procedure TFPReportVariable.SetAsInteger(AValue: Int64);
begin
  DataType:=rtinteger;
  FValue.ResInteger:=AValue;
end;

procedure TFPReportVariable.SetAsString(AValue: String);
begin
  DataType:=rtString;
  FValue.resString:=AValue;
end;

procedure TFPReportVariable.SetAsBoolean(AValue: Boolean);
begin
  FValue.ResultType:=rtBoolean;
  FValue.resBoolean:=AValue;
end;

procedure TFPReportVariable.SetAsCurrency(AValue: Currency);
begin
  FValue.ResultType:=rtCurrency;
  FValue.ResCurrency:=AValue;
end;

procedure TFPReportVariable.SetAsDateTime(AValue: TDateTime);
begin
  FValue.ResultType:=rtDateTime;
  FValue.ResDateTime:=AValue;
end;

procedure TFPReportVariable.SetAsFloat(AValue: TExprFloat);
begin
  FValue.ResultType:=rtFloat;
  FValue.ResFloat:=AValue;
end;

procedure TFPReportVariable.SetDataType(AValue: TResultType);
begin
  if FValue.ResultType=AValue then
    exit;
  if FValue.ResultType=rtString then
    FValue.resString:='';
  FValue.ResultType:=AValue;
end;

procedure TFPReportVariable.SetER(AValue: TFPExpressionResult);

begin
  FValue:=AValue;
end;

procedure TFPReportVariable.SetName(AValue: String);
begin
  if FName=AValue then Exit;
  {$IF FPC_FULLVERSION < 30002}
  if Not IsValidIdent(aValue) then
  {$ELSE}
  if Not IsValidIdent(aValue,True,true) then
  {$ENDIF}
    raise EReportError.CreateFmt(SErrInvalidVariableName, [aValue]);
  if (Collection is TFPReportVariables) then
    If ((Collection as TFPReportVariables).FindVariable(AValue)<>Nil) then
      raise EReportError.CreateFmt(SErrDuplicateVariable, [aValue]);

  FName:=AValue;
end;

procedure TFPReportVariable.SetResetType(AValue: TFPReportResetType);
begin
  if FResetType=AValue then Exit;
  FResetType:=AValue;
  Case FResetType of
    rtPage:   FResetValueExpression:='PageNo';
    rtColumn: FResetValueExpression:='ColNo';
  end;
end;

procedure TFPReportVariable.Assign(Source: TPersistent);

Var
  V : TFPReportVariable;

begin
  if Source is TFPReportVariable then
    begin
    V:=Source as TFPReportVariable;
    FName:=V.Name;
    FValue:=V.FValue;
    FExpression:=V.Expression;
    FResetType:=V.FResetType;
    FResetValueExpression:=V.FResetValueExpression;
    FResetValue:=V.FResetValue;
    end
  else
    inherited Assign(Source);
end;


procedure TFPReportVariable.InitExpressionValue(aData: TFPReportData; IsFirstpass: Boolean);

begin
  if not FIsAggregate then
    exit;
  if Not SameText(aData.Name,FDataName) then
    exit;
  If not IsFirstPass then
    exit;
  if (FResetValue=#0) then
    begin
    FResetValue:=#255;
    FLastValue.ResultType:=rtFloat;
    FLastValue.ResFloat:=0;
    FAggregateValue.ResultType:=rtFloat;
    FAggregateValue.ResFloat:=0;
    FExpressionNode.InitAggregate;
    end
end;

procedure TFPReportVariable.DoneExpressionValue(aData: TFPReportData; IsFirstpass: Boolean);

Var
  lResult: PFPExpressionResult;

begin
  if not FIsAggregate then
    exit;
  if (FResetType=rtNone) then
    exit;
  if not IsFirstPass then
    exit;
  if Not SameText(aData.Name,FDataName) then
    exit;
  If FResetValue=#255 then
    Exit;
  lResult:= new(PFPExpressionResult);
  lResult^:=FAggregateValue;
  {$ifdef gdebuga}
  Writeln('Variable : ',FName, ', Pushing value on stack ',DefExpressionResultToString(FAggregateValue),'aData: ',aData.Name);
  {$endif}
  FAggregateValues.Add(lResult);
  FResetValue:=#255;
end;

procedure TFPReportVariable.UpdateExpressionValue(aData: TFPReportData; IsFirstpass: Boolean);

var
  lResetValue: String;
  IsReset : Boolean;

  Function NeedReset : Boolean;

  begin
    Result:= (FResetType<>rtNone) and (lResetValue<>FResetValue);
  end;


begin
  if FExpression='' then
    exit;
  if not FIsAggregate then
    begin
    FLastValue:=FAggregateValue;
    if not aData.EOF then
      FAggregateValue:=FExpressionNode.NodeValue;
    exit;
    end;
  if Not SameText(aData.Name,FDataName) then
    exit;
  if (FResetType<>rtNone) then
    lResetValue:=DefExpressionResultToString(FResetValueExpressionNode.NodeValue);
  {$ifdef gdebuga}
  Write('Aggregate ',Name,' (',IsFirstPass,', ',FResetType,'): xp: ',Expression,' reset: "',FResetValueExpression,'"');
  if FResetValueExpression<>'' then
    Write(' Current reset:',lResetValue,', saved reset: ',FResetValue,') ');
  Writeln;
  {$endif}
  if IsFirstPass then
    begin
    if NeedReset then
      begin
      {$ifdef gdebuga}
      Writeln('Aggregate', Name,'Reset changed');
      {$endif}
      if (FResetValue<>#255) and (FResetValue<>#0) then
        begin
        {$ifdef gdebuga}
        Writeln('Aggregate ',Name,'pushing to stack.');
        {$endif}
        DoneExpressionValue(aData,isFirstpass); // Push
        end;
      FExpressionNode.InitAggregate;
      FResetValue:=lResetValue;
      end;
    FExpressionNode.UpdateAggregate;
    FLastValue:=FAggregateValue;
    FAggregateValue:=FExpressionNode.NodeValue;
    end
  else if (FResetType<>rtNone) then
    begin
    IsReset:=NeedReset;
    if IsReset then
      begin
      {$ifdef gdebuga}
      Writeln('Aggregate ',Name,'Reset changed');
      {$endif}
      if (FResetValue<>#255) and (FResetValue<>#0)  then
        begin
        {$ifdef gdebuga}
        Writeln('Aggregate ',Name,'Retrieving next value ',FAggregateValuesIndex);
        {$endif}
        inc(FAggregateValuesIndex);
        end;
      FResetValue:=lResetValue;
      FLastValue:=FAggregateValue;
      FAggregateValue:=PFPExpressionResult(FAggregateValues[FAggregateValuesIndex])^;
      end
    else
      begin
      FLastValue:=FAggregateValue;
      FAggregateValue:=PFPExpressionResult(FAggregateValues[FAggregateValuesIndex])^;
      end;
    end;
  {$ifdef gdebuga}
  Writeln('Aggregate ',Name,'---> current value: ',DefExpressionResultToString(FAggregateValue));
  {$endif}
end;

procedure TFPReportVariable.WriteElement(aWriter: TFPReportStreamer);
begin
  With AWriter do
    begin
    WriteString('Name',Self.Name);
    WriteString('DataType',ResultTypeName(DataType));
//    WriteString('Value',Value);
    WriteString('Expression',Expression);
    WriteString('ResetValueExpression',ResetValueExpression);
    WriteString('ResetType',GetEnumName(TypeInfo(TFPReportResetType),Ord(ResetType)));
    end;
end;

procedure TFPReportVariable.ReadElement(aWriter: TFPReportStreamer);

Var
  S : String;
  I : integer;
begin
  With AWriter do
    begin
    Self.Name:=ReadString('Name','');
    S:=ReadString('DataType','string');
    if S<>'' then
      I:=GetEnumValue(TypeInfo(TResultType),S)
    else
      I:=-1;
    if I=-1 then
      DataType:=rtString
    else
      DataType:=TResultType(I);
//    Value:=ReadString('Value','');
    Expression:=ReadString('Expression','');
    ResetValueExpression:=ReadString('ResetValueExpression','');
    S:=ReadString('ResetType','');
    if S<>'' then
      I:=GetEnumValue(TypeInfo(TFPReportResetType),S)
    else
      I:=-1;
    if I=-1 then
      ResetType:=rtNone
    else
      ResetType:=TFPReportResetType(I);
    end;
end;

{ TFPReportExportReg }

constructor TFPReportExportReg.Create(AClass: TFPReportExporterClass);
begin
  FCLass:=AClass;
end;


{ TFPReportExportManager }


function TFPReportExportManager.GetExporter(AIndex : Integer
  ): TFPReportExporterClass;
begin
  Result:=TFPReportExportReg(FList.Items[AIndex]).TheClass;
end;

function TFPReportExportManager.GetExporterCount: Integer;

begin
  Result:=FList.Count;
end;

procedure TFPReportExportManager.RegisterExport(AClass: TFPReportExporterClass);
begin
  if AClass=Nil then
    Raise EReportError.Create(SErrRegisterEmptyExporter);
  If IndexOfExporter(AClass.Name)<>-1 then
    Raise EReportError.CreateFmt(SErrRegisterDuplicateExporter,[AClass.Name]);
  FList.Add(TFPReportExportReg.Create(AClass));
end;

procedure TFPReportExportManager.UnRegisterExport(AClass: TFPReportExporterClass);

Var
  I : Integer;

begin
  I:=IndexOfExporter(AClass);
  if I<>-1 then
    FList.Delete(i);
end;

function TFPReportExportManager.ConfigExporter(AExporter: TFPReportExporter): Boolean;
Var
  H : TFPReportExporterConfigHandler;
begin
  H:=ExporterConfigHandler(TFPReportExporterClass(AExporter.ClassType));
  if (H=Nil) then
    H:=AExporter.DefaultConfig;
  if H=Nil then
    H:=OnConfigCallBack;
  Result:=False;
  If Assigned(H) then
    H(Self,AExporter,Result);
  Result:=Not Result;
end;

constructor TFPReportExportManager.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FList:=TFPObjectList.Create(True);
end;

destructor TFPReportExportManager.Destroy;
begin
  Clear;
  FList.Free;
  inherited Destroy;
end;

procedure TFPReportExportManager.Clear;
begin
  FList.Clear;
end;

function TFPReportExportManager.IndexOfExporter(const AName: String): Integer;
begin
  Result:=ExporterCount-1;
  While (Result>=0) and (CompareText(AName,Exporter[Result].Name)<>0) do
    Dec(Result);
end;

function TFPReportExportManager.IndexOfExporter(
  const AClass: TFPReportExporterClass): Integer;
begin
  Result:=ExporterCount-1;
  While (Result>=0) and (AClass<>Exporter[Result]) do
    Dec(Result);
end;

function TFPReportExportManager.FindExporter(const AName: String): TFPReportExporterClass;

Var
  I : Integer;

begin
  I:=IndexOfExporter(AName);
  If I<>-1 then
    Result:=Exporter[i]
  else
    Result:=Nil;
end;

function TFPReportExportManager.ExporterConfigHandler(
  const AClass: TFPReportExporterClass): TFPReportExporterConfigHandler;

Var
  I : Integer;

begin
  I:=IndexOfExporter(AClass);
  if I<>-1 then
    Result:=TFPReportExportReg(FList[i]).OnConfig
  else
    Result:=nil;
end;

procedure TFPReportExportManager.RegisterConfigHandler(const AName: String;
  ACallBack: TFPReportExporterConfigHandler);

Var
  I : integer;

begin
  I:=IndexOfExporter(AName);
  If (I=-1) Then
    Raise EReportError.CreateFmt(SErrUnknownExporter,[AName]);
  TFPReportExportReg(FList[i]).OnConfig:=ACallBack;
end;

{ TBandList }

function TBandList.GetCount: Integer;
begin
  Result := FList.Count;
end;

function TBandList.GetItems(AIndex: Integer): TFPReportCustomBand;
begin
  Result := TFPReportCustomBand(FList.Items[AIndex]);
end;

procedure TBandList.SetItems(AIndex: Integer; AValue: TFPReportCustomBand);
begin
  FList.Items[AIndex] := AValue;
end;

constructor TBandList.Create;
begin
  FList := TFPList.Create;
end;

destructor TBandList.Destroy;
begin
  FList.Clear;
  FreeAndNil(FList);
  inherited Destroy;
end;

procedure TBandList.Remove(AItem: TFPReportCustomBand);
begin
  if Assigned(AItem) then
    FList.Remove(aItem);
end;

function TBandList.Add(AItem: TFPReportCustomBand): Integer;
begin
  Result := -1;
  if Assigned(AItem) then
    if FList.IndexOf(AItem) = -1 then { we don't add duplications }
      Result := FList.Add(AItem);
end;

procedure TBandList.Clear;
begin
  FList.Clear;
end;

procedure TBandList.Delete(AIndex: Integer);
begin
  FList.Delete(AIndex);
end;

function TBandList.Find(ABand: TFPReportBandClass): TFPReportCustomBand;
begin
  Find(ABand, Result);
end;

function TBandList.Find(ABand: TFPReportBandClass; out AResult: TFPReportCustomBand): Integer;
var
  i: integer;
begin
  AResult := nil;
  Result := -1;
  for i := 0 to Count-1 do
  begin
    if Items[i] is ABand then
    begin
      Result := i;
      AResult := Items[i];
      Break;
    end;
  end;
end;

procedure TBandList.Sort(Compare: TListSortCompare);
begin
  FList.Sort(Compare);
end;

{ TFPReportCustomMemo }

procedure TFPReportCustomMemo.SetText(AValue: TFPReportString);
begin
  if FText = AValue then
    Exit;
  FText := AValue;
  Changed;
end;

procedure TFPReportCustomMemo.HandleFontChange(Sender: TObject);
begin
  FUseParentFont:=False;
  Changed;
end;

procedure TFPReportCustomMemo.SetCullThreshold(AValue: TFPReportCullThreshold);
begin
  if FCullThreshold = AValue then
    Exit;
  FCullThreshold := AValue;
  Changed;
end;

function TFPReportCustomMemo.GetParentFont: TFPReportFont;

begin
  if Assigned(Band) then
    Result := Band.Font
  else
    Result:=Nil;
end;

procedure TFPReportCustomMemo.SetUseParentFont(AValue: Boolean);

begin
  if FUseParentFont = AValue then
    Exit;
  FUseParentFont := AValue;
  if aValue then
    ReassignParentFont;
  Changed;
end;

procedure TFPReportCustomMemo.SetWordOverflow(AValue: TFPReportWordOverflow);
begin
  if FWordOverflow=AValue then Exit;
  FWordOverflow:=AValue;
  Changed;
end;

{ All = True) indicates that if the text is split over multiple lines the last
  line must also be processed before continuing. If All = False, then double
  CR can be ignored. }

procedure TFPReportCustomMemo.AddTextLine(lFC: TFPFontCacheItem; Var S : UTF8String; MaxW : TFPReportUnits);

var
  w: single;
  m: integer;
  s2, s3: UTF8String;

begin
  s2  := s;
  w   := lFC.TextWidth(s2, Font.Size);
  if (Length(s2) > 1) and (w > maxw) then
  begin
    while w > maxw do
    begin
      m := Length(s);
      repeat
        Dec(m);
        s2  := Copy(s,1,m);
        w   := lFC.TextWidth(s2, Font.Size);
      until w <= maxw;

      s3 := s2; // we might need the value of s2 later again

      // are we in the middle of a word. If so find the beginning of word.
      while (m > 0) and (s2[m] <> ' ') do
        Dec(m);
      s2  := Copy(s,1,m);

      if s2 = '' then
        begin
        // Single word does not fit. S3 is max word that fits.
        s2 := s3;
        Case WordOverflow of
          woOverflow:
            begin
              { We reached the beginning of the line without finding a word that fits the maxw.
                So we are forced to use a longer than maxw word. We were in the middle of
                a word, so now find the end of the current word. }
            m := Length(s2);
            while (m < Length(s)) and (s[m]<> ' ') do
              Inc(m);
            s2:=Copy(s,1,m);
            end;
          woTruncate:
            m:=Length(S); // Discard the remainder of the word.
          woSplit:
            m:=Length(S3); // S3 was the longest possible part of the word. Split after
          woEllipsis:
            begin
            repeat
              S2:=Copy(S2,1,Length(S2)-1);
            Until (Length(S2)<1) or (lFC.TextWidth(s2+UTF8Encode(cEllipsis), Font.Size)<MaxW);
            S2:=S2+UTF8Encode(cEllipsis);
            m:=Length(S); // Discard the remainder of the word.
            end;
          woAsterisk:
            begin
            w:= lFC.TextWidth('*', Font.Size);
            S2:=StringOfChar('*',round(MaxW / w));
            m:=Length(S); // Discard the remainder of the word.
            end;
       end;
       end;
      FTextLines.Add(s2);
      s   := Copy(s, m+1, Length(s));
      s2  := s;
      w   := lFC.TextWidth(s2, Font.Size);
    end; { while }
    if s2 <> '' then
      FTextLines.Add(s2);
    s := '';
  end
  else
  begin
    if s2 <> '' then
      FTextLines.Add(s2);
    s := '';
  end; { if/else }
end;

procedure TFPReportCustomMemo.WrapText(const AText: UTF8String; lFC: TFPFontCacheItem; const ALineWidth: TFPReportUnits; out AHeight: TFPReportUnits);

var
  maxw: single; // value in pixels
  n: integer;
  s: UTF8String;
  c: char;
  lWidth: single;

  lDescenderHeight: single;
  lHeight: single;


begin
  if AText = '' then
    Exit;

  if ALineWidth = 0 then
    Exit;
  { result is in pixels }
  lWidth := lFC.TextWidth(Text, Font.Size);
  lHeight := lFC.TextHeight(Text, Font.Size, lDescenderHeight);
  { convert pixels to mm as our Reporting Units are defined as mm. }
  AHeight := PixelsToMM(lHeight+lDescenderHeight);

  s := '';
  n := 1;
  maxw := mmToPixels(ALineWidth - TextAlignment.LeftMargin - TextAlignment.RightMargin);
  { Do we really need to do text wrapping? There must be no linefeed characters and lWidth must be less than maxw. }
  if ((Pos(#13, AText) = 0) and (Pos(#10, AText) = 0)) and (lWidth <= maxw) then
  begin
    FTextLines.Add(AText);
    Exit;
  end;

  { We got here, so wrapping is needed. First process line wrapping as indicated
    by LineEnding characters in the text. }
  while n <= Length(AText) do
    begin
    c := AText[n];
    if (c = #13) or (c = #10) then
    begin
      { See code comment of AddLine() for the meaning of the True argument. }
      AddTextLine(lfc,S,maxw);
      if (c = #13) and (n < Length(AText)) and (AText[n+1] = #10) then
        Inc(n);
    end
    else
      s := s + c;
    Inc(n);
    end; { while }

  { Now wrap lines that are longer than ALineWidth }
  AddTextLine(lfc,S,maxW);
end;

procedure TFPReportElement.ApplyStretchMode(const ADesiredHeight: TFPReportUnits);

begin
  if Not Assigned(RTLayout) then
    Exit;
  Case StretchMode of
    smMaxHeight:
      begin
      RTLayout.Height := aDesiredHeight;
      if Assigned(Parent) and Assigned(RTLayout) then
        RTLayout.Height:=Parent.RTLayout.Height-RTLayout.Top;
      end;
    smActualHeight:
      begin
      RTLayout.Height := aDesiredHeight;
      end;
    smActualHeightStretchOnly:
      begin
      if aDesiredHeight>RTLayout.Height then { only grow height if needed. We don't shrink. }
        RTLayout.Height := aDesiredHeight;
      end;
    smActualHeightShrinkOnly:
      begin
      if aDesiredHeight<RTLayout.Height then { only shrink height if needed. We don't grow. }
        RTLayout.Height := ADesiredHeight;
      end;
  end;
end;

{ this affects only X coordinate of text blocks }
procedure TFPReportCustomMemo.ApplyHorzTextAlignment;
var
  i: integer;
  tb: TFPTextBlock;
  lList: TFPList;
  lLastYPos: TFPReportUnits;

  procedure ProcessLeftJustified;
  var
    idx: integer;
    b: TFPTextBlock;
    lXOffset: TFPReportUnits;
  begin
    if TextAlignment.LeftMargin = 0 then
      exit;
    { All the text blocks must move by LeftMargin to the right. }
    lXOffset := TextAlignment.LeftMargin;
    for idx := 0 to lList.Count-1 do
    begin
      b := TFPTextBlock(lList[idx]);
      b.Pos.Left := lXOffset + b.Pos.Left
    end;
  end;

  procedure ProcessRightJustified;
  var
    idx: integer;
    b: TFPTextBlock;
    lXOffset: TFPReportUnits;
  begin
    lXOffset := Layout.Width - TextAlignment.RightMargin;
    for idx := lList.Count-1 downto 0 do
    begin
      b := TFPTextBlock(lList[idx]);
      b.Pos.Left := lXOffset - b.Width;
      lXOffset := b.Pos.Left;
    end;
  end;

  procedure ProcessCentered;
  var
    idx: integer;
    b: TFPTextBlock;
    lXOffset: TFPReportUnits;
    lTotalWidth: TFPReportUnits;
  begin
    lTotalWidth := 0;
    for idx := 0 to lList.Count-1 do
    begin
      b := TFPTextBlock(lList[idx]);
      lTotalWidth := lTotalWidth + b.Width;
    end;
    lXOffset := (Layout.Width - lTotalWidth) / 2;
    if lXOffset < 0.0 then { it should never be, but lets play it safe }
      lXOffset := 0.0;
    for idx := 0 to lList.Count-1 do
    begin
      b := TFPTextBlock(lList[idx]);
      b.Pos.Left := lXOffset;
      lXOffset := lXOffset + b.Width;
    end;
  end;

  procedure ProcessWidth;
  var
    idx: integer;
    b: TFPTextBlock;
    lXOffset: TFPReportUnits;
    lSpace: TFPReportUnits;
    lTotalWidth: TFPReportUnits;
  begin
    lTotalWidth := 0;
    for idx := 0 to lList.Count-1 do
    begin
      b := TFPTextBlock(lList[idx]);
      lTotalWidth := lTotalWidth + b.Width;
    end;
    lSpace := (Layout.Width - TextAlignment.LeftMargin - TextAlignment.RightMargin - lTotalWidth) / (lList.Count-1);
    { All the text blocks must move by LeftMargin to the right. }
    lXOffset := TextAlignment.LeftMargin;
    for idx := 0 to lList.Count-1 do
    begin
      b := TFPTextBlock(lList[idx]);
      b.Pos.Left := lXOffset;
      lXOffset := lXOffset + b.Width + lSpace;
    end;
  end;

begin
  lList := TFPList.Create;
  lLastYPos := 0;
  for i := 0 to FTextBlockList.Count-1 do
  begin
    tb := FTextBlockList[i];
    if tb.Pos.Top = lLastYPos then // still on the same text line
      lList.Add(tb)
    else
    begin
      { a new line has started - process what we have collected in lList }
      case TextAlignment.Horizontal of
        taLeftJustified:   ProcessLeftJustified;
        taRightJustified:  ProcessRightJustified;
        taCentered:        ProcessCentered;
        taWidth:           ProcessWidth;
      end;
      lList.Clear;
      lLastYPos := tb.Pos.Top;
      lList.Add(tb)
    end; { if..else }
  end; { for i }

  { process the last text line's items }
  if lList.Count > 0 then
  begin
    case TextAlignment.Horizontal of
      taLeftJustified:   ProcessLeftJustified;
      taRightJustified:  ProcessRightJustified;
      taCentered:        ProcessCentered;
      taWidth:           ProcessWidth;
    end;
  end;
  lList.Free;
end;

{ this affects only Y coordinate of text blocks }
procedure TFPReportCustomMemo.ApplyVertTextAlignment;
var
  i: integer;
  tb: TFPTextBlock;
  lList: TFPList;
  lLastYPos: TFPReportUnits;
  lTotalHeight: TFPReportUnits;
  lYOffset: TFPReportUnits;

  procedure ProcessTop;
  var
    idx: integer;
    b: TFPTextBlock;
  begin
    if lList.Count = 0 then
      Exit;
    for idx := 0 to lList.Count-1 do
    begin
      b := TFPTextBlock(lList[idx]);
      b.Pos.Top := lYOffset;
    end;
    lYOffset := lYOffset + LineSpacing + b.Height + b.Descender;
  end;

  procedure ProcessCenter;
  var
    idx: integer;
    b: TFPTextBlock;
  begin
    for idx := 0 to lList.Count-1 do
    begin
      b := TFPTextBlock(lList[idx]);
      b.Pos.Top := lYOffset;
    end;
    lYOffset := lYOffset + LineSpacing + b.Height + b.Descender;
  end;

  procedure ProcessBottom;
  var
    idx: integer;
    b: TFPTextBlock;
  begin
    for idx := 0 to lList.Count-1 do
    begin
      b := TFPTextBlock(lList[idx]);
      b.Pos.Top := lYOffset;
    end;
    lYOffset := lYOffset - LineSpacing - b.Height - b.Descender;
  end;

begin
  if FTextBlockList.Count = 0 then
    Exit;
  lList := TFPList.Create;
  try
  lLastYPos := FTextBlockList[FTextBlockList.Count-1].Pos.Top;  // last textblock's Y coordinate
  lTotalHeight := 0;

  if TextAlignment.Vertical = tlTop then
  begin
    if TextAlignment.TopMargin = 0 then
      Exit; // nothing to do
    lYOffset := TextAlignment.TopMargin;
    for i := 0 to FTextBlockList.Count-1 do
    begin
      tb := FTextBlockList[i];
      if tb.Pos.Top = lLastYPos then // still on the same text line
        lList.Add(tb)
      else
      begin
        { a new line has started - process what we have collected in lList }
        ProcessTop;

        lList.Clear;
        lLastYPos := tb.Pos.Top;
        lList.Add(tb)
      end; { if..else }
    end; { for i }
  end

  else if TextAlignment.Vertical = tlBottom then
  begin
    lYOffset := Layout.Height;
    for i := FTextBlockList.Count-1 downto 0 do
    begin
      tb := FTextBlockList[i];
      if i = FTextBlockList.Count-1 then
        lYOffset := lYOffset - tb.Height - tb.Descender - TextAlignment.BottomMargin;  // only need to do this for one line
      if tb.Pos.Top = lLastYPos then // still on the same text line
        lList.Add(tb)
      else
      begin
        { a new line has started - process what we have collected in lList }
        ProcessBottom;

        lList.Clear;
        lLastYPos := tb.Pos.Top;
        lList.Add(tb)
      end; { if..else }
    end; { for i }
  end

  else if TextAlignment.Vertical = tlCenter then
  begin
    { First, collect the total height of all the text lines }
    lTotalHeight := 0;
    lLastYPos := 0;
    for i := 0 to FTextBlockList.Count-1 do
    begin
      tb := FTextBlockList[i];
      if i = 0 then  // do this only for the first block
        lTotalHeight := tb.Height + tb.Descender;
      if tb.Pos.Top = lLastYPos then // still on the same text line
        Continue
      else
      begin
        { a new line has started - process what we have collected in lList }
        lTotalHeight := lTotalHeight + LineSpacing + tb.Height + tb.Descender;
      end; { if..else }
      lLastYPos := tb.Pos.Top;
    end; { for i }

    { Now process them line-by-line }
    lList.Clear;
    lYOffset := (Layout.Height - lTotalHeight) / 2;
    lLastYPos := 0;
    for i := 0 to FTextBlockList.Count-1 do
    begin
      tb := FTextBlockList[i];
      if tb.Pos.Top = lLastYPos then // still on the same text line
        lList.Add(tb)
      else
      begin
        { a new line has started - process what we have collected in lList }
        ProcessCenter;

        lList.Clear;
        lLastYPos := tb.Pos.Top;
        lList.Add(tb)
      end; { if..else }
    end; { for i }
  end;

  { process the last text line's items }
  if lList.Count > 0 then
  begin
    case TextAlignment.Vertical of
      tlTop:     ProcessTop;
      tlCenter:  ProcessCenter;
      tlBottom:  ProcessBottom;
    end;
  end;

  finally
    lList.Free;
  end;
end;

{ package the text into TextBlock objects. We don't apply Memo Margins here - that
  gets done in the Apply*TextAlignment() methods. }
procedure TFPReportCustomMemo.PrepareTextBlocks;
var
  i: integer;
begin
  { blockstate is cleared outside the FOR loop because the font state could
    roll over to multiple lines. }
  FTextBlockState := [];
  FTextBlockYOffset := 0;
  FLastURL := '';
  FLastFGColor := clNone;
  FLastBGColor := clNone;
  for i := 0 to FTextLines.Count-1 do
  begin
    FTextBlockXOffset := 0;
    if Assigned(FCurTextBlock) then
      FTextBlockYOffset := FTextBlockYOffset + FCurTextBlock.Height + FCurTextBlock.Descender + LineSpacing;
    if moAllowHTML in Options then
    begin
      FParser := THTMLParser.Create(FTextLines[i]);
      try
        FParser.OnFoundTag := @HTMLOnFoundTag;
        FParser.OnFoundText := @HTMLOnFoundText;
        FParser.Exec;
      finally
        FParser.Free;
      end;
    end
    else
    begin
      if TextAlignment.Horizontal <> taWidth then
        AddSingleTextBlock(FTextLines[i])
      else
        AddMultipleTextBlocks(FTextLines[i]);
    end;
  end; { for i }
end;

function TFPReportCustomMemo.GetTextLines: TStrings;
begin
  if StretchMode <> smDontStretch then
    Result := FTextLines
  else
    Result := nil;
end;

procedure TFPReportCustomMemo.SetLineSpacing(AValue: TFPReportUnits);
begin
  if FLineSpacing = AValue then
    Exit;
  FLineSpacing := AValue;
  Changed;
end;

procedure TFPReportCustomMemo.HTMLOnFoundTag(NoCaseTag, ActualTag: string);
var
  v: string;
begin
  if NoCaseTag = '<B>' then
    Include(FTextBlockState, htBold)
  else if NoCaseTag = '</B>' then
    Exclude(FTextBlockState, htBold)
  else if NoCaseTag = '<I>' then
    Include(FTextBlockState, htItalic)
  else if NoCaseTag = '</I>' then
    Exclude(FTextBlockState, htItalic)
  else if (FParser.GetTagName(NoCaseTag) = 'A') then
    FLastURL := FParser.GetVal(ActualTag, 'href')
  else if (FParser.GetTagName(NoCaseTag) = '/A') then
    FLastURL := ''
  else if FParser.GetTagName(NoCaseTag) = 'FONT' then
  begin
    { process the opening tag }
    v := FParser.GetVal(NoCaseTag, 'color');
    if v <> '' then
      FLastFGColor := HtmlColorToFPReportColor(v);
    v := FParser.GetVal(NoCaseTag, 'bgcolor');
    if v <> '' then
      FLastBGColor := HtmlColorToFPReportColor(v);
  end
  else if FParser.GetTagName(NoCaseTag) = '/FONT' then
  begin
    { process the closing tag }
    FLastFGColor := clNone;
    FLastBGColor := clNone;
  end;
end;

procedure TFPReportCustomMemo.HTMLOnFoundText(Text: string);
var
  lNewFontName: string;
  lDescender: TFPReportUnits;
  lHasURL: boolean;
begin
  lHasURL := FLastURL <> '';

  FCurTextBlock := CreateTextBlock(lHasURL);
  if lHasURL then
  begin
    TFPHTTPTextBlock(FCurTextBlock).URL := FLastURL;
    FCurTextBlock.FGColor := LinkColor;
  end;

  try
    FCurTextBlock.Text := Text;

    if FLastFGColor <> clNone then
      FCurTextBlock.FGColor := FLastFGColor;
    if FLastBGColor <> clNone then
      FCurTextBlock.BGColor := FLastBGColor;
    if (([htBold,htItalic] * FTextBlockState)=[]) then
      lNewFontName:=Font.Name
    else
      lNewFontName:=gTTFontCache.FindPostScriptFontname(Font.Name, htBold in FTextBlockState, htItalic in FTextBlockState);

    FCurTextBlock.FontName := lNewFontName;

    FCurTextBlock.Width := TextWidth(FCurTextBlock.Text, FCurTextBlock.FontName, Font.Size);
    FCurTextBlock.Height := TextHeight(FCurTextBlock.Text,FCurTextBlock.FontName, Font.Size, lDescender);
    FCurTextBlock.Descender := lDescender;

    // get X offset from previous textblocks
    FCurTextBlock.Pos.Left := FTextBlockXOffset;
    FCurTextBlock.Pos.Top := FTextBlockYOffset;
    FTextBlockXOffset := FTextBlockXOffset + FCurTextBlock.Width;
  except
    on E: EReportFontNotFound do
    begin
      FCurTextBlock.Free;
      raise;
    end;
  end;
  FTextBlockList.Add(FCurTextBlock);
end;

function TFPReportCustomMemo.PixelsToMM(APixels: single): single;
begin
  Result := (APixels * cMMperInch) / gTTFontCache.DPI;
end;

function TFPReportCustomMemo.mmToPixels(mm: single): integer;
begin
  Result := Round(mm * (gTTFontCache.DPI / cMMperInch));
end;

function TFPReportCustomMemo.TextHeight(const AText, FontName: string; FontSize: Integer; out ADescender: TFPReportUnits): TFPReportUnits;
var
  lHeight: single;
  lDescenderHeight: single;
  lFC: TFPFontCacheItem;

begin
  lFC := gTTFontCache.FindFont(FontName); // we are doing a PostScript Name lookup (it contains Bold, Italic info)
  if not Assigned(lFC) then
    raise EReportFontNotFound.CreateFmt(SErrFontNotFound, [FontName]);
  { Both lHeight and lDescenderHeight are in pixels }
  lHeight := lFC.TextHeight(AText, FontSize, lDescenderHeight);

  { convert pixels to mm. }
  ADescender := PixelsToMM(lDescenderHeight);
  Result := PixelsToMM(lHeight);
end;

function TFPReportCustomMemo.TextWidth(const AText, FontName: string; FontSize: Integer): TFPReportUnits;
var
  lWidth: single;
  lFC: TFPFontCacheItem;
begin
  // TODO: FontName might need to change to TextBlock.FontName.
  lFC := gTTFontCache.FindFont(FontName); // we are doing a PostScript Name lookup (it contains Bold, Italic info)
  if not Assigned(lFC) then
    raise EReportFontNotFound.CreateFmt(SErrFontNotFound, [FontName]);
  { result is in pixels }
  lWidth := lFC.TextWidth(AText, FontSize);

  { convert pixels to mm. }
  Result := PixelsToMM(lWidth);
end;

procedure TFPReportCustomMemo.SetLinkColor(AValue: TFPReportColor);
begin
  if FLinkColor = AValue then
    Exit;
  FLinkColor := AValue;
  Changed;
end;

procedure TFPReportCustomMemo.SetTextAlignment(AValue: TFPReportTextAlignment);
begin
  if FTextAlignment = AValue then
    Exit;
  BeginUpdate;
  try
    FTextAlignment.Assign(AValue);
  finally
    EndUpdate;
  end;
end;

procedure TFPReportCustomMemo.SetOptions(const AValue: TFPReportMemoOptions);
begin
  if FOptions = AValue then
    Exit;
  FOptions := AValue;
  { If Options conflicts with StretchMode, then remove moDisableWordWrap from Options. }
  if StretchMode <> smDontStretch then
    Exclude(FOptions, moDisableWordWrap);
  Changed;
end;

function TFPReportCustomMemo.SubStr(const ASource, AStartDelim, AEndDelim: string; AIndex: integer; out
  AStartPos: integer): string;
var
  liStart : integer;
  liEnd  : integer;
  i: integer;
begin
  liStart := 0;
  if AIndex < 1 then
    AIndex := 1;
  for i := 1 to AIndex do
    liStart := PosEx(AStartDelim, ASource, liStart+1);

  result := '';
  AStartPos := -1;

  if liStart <> 0 then
    liStart := liStart + Length(AStartDelim);

  liEnd := PosEx(AEndDelim, ASource, liStart);
  if liEnd <> 0 then
    liEnd := liEnd - 1;

  if (liStart = 0) or (liEnd = 0) then
    Exit; //==>

  result := Copy(ASource, liStart, liEnd - liStart + 1);
  AStartPos := liStart;
end;

procedure TFPReportCustomMemo.ParseText;

var
  lCount: integer;
  n: TFPExprNode;
  i: integer;
  str: string;
  lStartPos: integer;

begin
  { clear array and then set the correct array size }
  ClearExpressionNodes;
  if Pos('[', Text) > 0 then
    lCount := TokenCount(Text)-1
  else
    exit;
  SetLength(ExpressionNodes, lCount);
  str := '';
  n := nil;
  for i := 1 to lCount do
  begin
    str := SubStr(Text, '[', ']', i, lStartPos);
    if str <> '' then
    begin
      GetExpr.Expression := str;
      GetExpr.ExtractNode(n);
      if n.HasAggregate then
        n.InitAggregate;
      ExpressionNodes[i-1].Position := lStartPos;
      ExpressionNodes[i-1].ExprNode := n;
    end;
  end;
end;

procedure TFPReportCustomMemo.ClearExpressionNodes;
var
  i: integer;
begin
  for i := 0 to Length(ExpressionNodes)-1 do
    FreeAndNil(ExpressionNodes[i].ExprNode);
  SetLength(ExpressionNodes, 0);
end;

procedure TFPReportCustomMemo.AddSingleTextBlock(const AText: string);
var
  lDescender: TFPReportUnits;
begin
  if AText = '' then
    Exit;  //==>
  FCurTextBlock := CreateTextBlock(false);
  try
    FCurTextBlock.Text := AText;
    FCurTextBlock.FontName := Font.Name;
    FCurTextBlock.Width := TextWidth(FCurTextBlock.Text, FCurTextBlock.FontName, Font.Size);
    FCurTextBlock.Height := TextHeight(FCurTextBlock.Text, FCurTextBlock.FontName, Font.Size, lDescender);
    FCurTextBlock.Descender := lDescender;

    // get X offset from previous textblocks
    FCurTextBlock.Pos.Left := FTextBlockXOffset;
    FCurTextBlock.Pos.Top := FTextBlockYOffset;
    FTextBlockXOffset := FTextBlockXOffset + FCurTextBlock.Width;
  except
    on E: EReportFontNotFound do
    begin
      FCurTextBlock.Free;
      raise;
    end;
  end;
  FTextBlockList.Add(FCurTextBlock);
end;

procedure TFPReportCustomMemo.AddMultipleTextBlocks(const AText: string);
var
  lCount: integer;
  i: integer;
begin
  lCount := TokenCount(AText, ' ');
  for i := 1 to lCount do
    AddSingleTextBlock(Token(AText, ' ', i));
end;

function TFPReportCustomMemo.TokenCount(const AValue: string; const AToken: string): integer;
var
  i, iCount : integer;
  lsValue : string;
begin
  Result := 0;
  if AValue = '' then
    Exit; //==>

  iCount := 0;
  lsValue := AValue;
  i := pos(AToken, lsValue);
  while i <> 0 do
  begin
    delete(lsValue, i, length(AToken));
    inc(iCount);
    i := pos(AToken, lsValue);
  end;
  Result := iCount+1;
end;

function TFPReportCustomMemo.Token(const AValue, AToken: string; const APos: integer): string;
var
  i, iCount, iNumToken: integer;
  lsValue: string;
begin
  result := '';

  iNumToken := TokenCount(AValue, AToken);
  if APos = 1 then
  begin
    if pos(AToken, AValue) = 0 then
      result := AValue
    else
      result := copy(AValue, 1, pos(AToken, AValue)-1);
  end
  else if (iNumToken < APos-1) or (APos<1) then
  begin
    result := '';
  end
  else
  begin
    { Remove leading blocks }
    iCount := 1;
    lsValue := AValue;
    i := pos(AToken, lsValue);
    while (i<>0) and (iCount<APos) do
    begin
      delete(lsValue, 1, i + length(AToken) - 1);
      inc(iCount);
      i := pos(AToken, lsValue);
    end;

    if (i=0) and (iCount=APos) then
      result := lsValue
    else if (i=0) and (iCount<>APos) then
      result := ''
    else
      result := copy(lsValue, 1, i-1);
  end;
end;

function TFPReportCustomMemo.IsExprAtArrayPos(const APos: integer): Boolean;
var
  i: integer;
begin
  Result := False;
  for i := 0 to Length(Original.ExpressionNodes)-1 do
  begin
    if Original.ExpressionNodes[i].Position = APos then
    begin
      if Original.ExpressionNodes[i].ExprNode <> nil then
      begin
        Result := True;
        Break;
      end;
    end;
  end;
end;

procedure TFPReportCustomMemo.SetFont(const AValue: TFPReportFont);
begin
  UseParentFont := False;
  FFont.Assign(AValue);
  Changed;
end;

{ Only called if StretchMode = smDontStretch. This methods removes text that
  will not fit in the space allocated for the Memo. }
procedure TFPReportCustomMemo.CullTextOutOfBounds;
var
  i: integer;
  lBlock: TFPTextBlock;
  lRemainingHeight: single;
  d: single;
begin
  for i := FTextBlockList.Count-1 downto 0 do
  begin
    lBlock := FTextBlockList[i];

    if lBlock.Pos.Top >= Layout.Height then // completely out of bounds
    begin
      FTextBlockList.Delete(i);
    end
    else if (lBlock.Pos.Top + lBlock.Height + lBlock.Descender) > Layout.Height then // partially out of bounds
    begin
      lRemainingHeight :=  Layout.Height - lBlock.Pos.Top;
      { calculate % of text [height] that falls inside the bounderies of the Memo. }
      d := (lRemainingHeight / (lBlock.Height + lBlock.Descender)) * 100;

      {$IFDEF gDEBUG}
      writeln(Format('Memo Culling: %2.2f%% of line height is visible', [d]));
      {$ENDIF}

      if CullThreshold > d then
      begin
        FTextBlockList.Delete(i);
      end;
    end;
  end;
end;

function TFPReportCustomMemo.CreateTextAlignment: TFPReportTextAlignment;
begin
  Result := TFPReportTextAlignment.Create(self);
end;

function TFPReportCustomMemo.GetExpr: TFPExpressionParser;
begin
  Result:=Report.FExpr;
end;

function TFPReportCustomMemo.CreateTextBlock(const IsURL: boolean): TFPTextBlock;
begin
  if IsURL then
    result := TFPHTTPTextBlock.Create
  else
    result := TFPTextBlock.Create;
  result.FontName := Font.Name;
  result.FGColor := Font.Color;
  result.BGColor := clNone;
end;

function TFPReportCustomMemo.HtmlColorToFPReportColor(AColorStr: string; ADefault: TFPReportColor): TFPReportColor;
var
  N1, N2, N3: integer;
  i: integer;
  Len: integer;

  function IsCharWord(ch: char): boolean;
  begin
    Result := ch in ['a'..'z', 'A'..'Z', '_', '0'..'9'];
  end;

  function IsCharHex(ch: char): boolean;
  begin
    Result := ch in ['0'..'9', 'a'..'f', 'A'..'F'];
  end;

begin
  Result := ADefault;
  Len := 0;
  if (AColorStr <> '') and (AColorStr[1] = '#') then
    Delete(AColorStr, 1, 1);
  if (AColorStr = '') then
    exit;

  //delete after first nonword char
  i := 1;
  while (i <= Length(AColorStr)) and IsCharWord(AColorStr[i]) do
    Inc(i);
  Delete(AColorStr, i, Maxint);

  //allow only #rgb, #rrggbb
  Len := Length(AColorStr);
  if (Len <> 3) and (Len <> 6) then
    Exit;

  for i := 1 to Len do
    if not IsCharHex(AColorStr[i]) then
      Exit;

  if Len = 6 then
  begin
    N1 := StrToInt('$'+Copy(AColorStr, 1, 2));
    N2 := StrToInt('$'+Copy(AColorStr, 3, 2));
    N3 := StrToInt('$'+Copy(AColorStr, 5, 2));
  end
  else
  begin
    N1 := StrToInt('$'+AColorStr[1]+AColorStr[1]);
    N2 := StrToInt('$'+AColorStr[2]+AColorStr[2]);
    N3 := StrToInt('$'+AColorStr[3]+AColorStr[3]);
  end;

  Result := RGBToReportColor(N1, N2, N3);
end;

procedure TFPReportCustomMemo.RecalcLayout;

  Function CalcNeededHeight(aHeight : TFPReportUnits) : TFPReportUnits;
  begin
    Result :=((AHeight + LineSpacing) * TextLines.Count) + TextAlignment.TopMargin + TextAlignment.BottomMargin;
  end;

var
  h, maxW: TFPReportUnits;
  lFC : TFPFontCacheItem;
  S : UTF8String;

begin
  FTextBlockList.Clear;
  FCurTextBlock := nil;
  if not Assigned(FTextLines) then
    FTextLines := TStringList.Create
  else
    FTextLines.Clear;
  { We are doing a PostScript Name lookup (it contains Bold, Italic info) }
  lFC := gTTFontCache.FindFont(Font.Name);
  if not Assigned(lFC) then
    raise EReportFontNotFound.CreateFmt(SErrFontNotFound, [Font.Name]);
  if not (moDisableWordWrap in Options) then
    WrapText(Text, lfc, Layout.Width, h)
  else
    begin
    maxw := mmToPixels(Layout.Width - TextAlignment.LeftMargin - TextAlignment.RightMargin);
    S:=Text;
    AddTextLine(lfc,S,maxw);
    end;

  if StretchMode <> smDontStretch then
    ApplyStretchMode(CalcNeededHeight(h));

  PrepareTextBlocks;
  if StretchMode = smDontStretch then
    CullTextOutOfBounds;
  ApplyVertTextAlignment;
  ApplyHorzTextAlignment;
end;

procedure TFPReportCustomMemo.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);

var
  T: TFPReportTextAlignment;

begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  AWriter.WriteString('Text', Text);
  AWriter.WriteInteger('CullThreshold', CullThreshold);

  AWriter.WriteBoolean('UseParentFont', UseParentFont);
  if not UseParentFont then
  begin
    AWriter.WriteString('FontName', Font.Name);
    AWriter.WriteInteger('FontSize', Font.Size);
    AWriter.WriteQWord('FontColor', Font.Color);
  end;

  AWriter.WriteFloat('LineSpacing', LineSpacing);
  AWriter.WriteQWord('LinkColor', LinkColor);
  AWriter.WriteString('Options', MemoOptionsToString(Options));
  if (AOriginal <> nil) then
      T := TFPReportCustomMemo(AOriginal).TextAlignment
    else
      T := nil;
    AWriter.PushElement('TextAlignment');
    try
      FTextAlignment.WriteElement(AWriter, T);
    finally
      AWriter.PopElement;
    end;
end;

procedure TFPReportCustomMemo.ExpandExpressions;
var
  lCount: integer;
  str: string;
  n: TFPExprNode;
  i: integer;
  lStartPos: integer;
  lResult: string;
  s: string;
begin
  lCount := TokenCount(Text);
  if lCount = 0 then
    Exit;
  lResult := Text;
  str := '';
  n := nil;
  for i := 0 to lCount-1 do
  begin
    str := SubStr(Text, '[', ']', i+1, lStartPos);
    if str <> '' then
    begin
      if IsExprAtArrayPos(lStartPos) then
      begin
        n := Original.ExpressionNodes[i].ExprNode;
        S:=ExpressionResultToString(n.NodeValue);
        lResult := StringReplace(lResult, '[' + str + ']', s, [rfReplaceAll]);
      end;
    end;
  end;
  Text := lResult;
end;



procedure TFPReportCustomMemo.UpdateAggregates;

var
  i : Integer;

begin
  for i := 0 to Length(ExpressionNodes)-1 do
    begin
    if Assigned(ExpressionNodes[i].ExprNode) then
      if ExpressionNodes[i].ExprNode.HasAggregate then
        ExpressionNodes[i].ExprNode.UpdateAggregate;
    end;  { for ... }
end;

function TFPReportCustomMemo.PrepareObject(aRTParent: TFPReportElement): TFPReportElement;

Var
  m : TFPReportCustomMemo;
  I : integer;
  N : TFPExprNode;

  Procedure CheckVisibility;

  begin
    // visibility handling
    if (moHideZeros in m.Options) then
      begin
      m.Visible:=Not TFPCustomReport.IsStringValueZero(m.Text);
      if not M.Visible then
        exit;
      end;
    if (moSuppressRepeated in m.Options) then
      begin
      m.Visible:=m.Original.FLastText <> m.Text;
      if not M.Visible then
        exit;
      m.Original.FLastText := m.Text;
      end;
  end;

begin
  Result:=Inherited PrepareObject(aRTParent);
  if Result=Nil then
    exit;
  m:=TFPReportCustomMemo(Result);
  if moDisableExpressions in m.Options then
    begin
    CheckVisibility;
    Exit; // nothing further to do
    end;
  m.ExpandExpressions;
  CheckVisibility;
  // aggregate handling
  for I := 0 to Length(m.Original.ExpressionNodes)-1 do
    begin
    n := m.Original.ExpressionNodes[I].ExprNode;
    if Assigned(n)
       and n.HasAggregate
       and Not (moNoResetAggregateOnPrint in m.Options) then
        n.InitAggregate;
    end;
end;

procedure TFPReportCustomMemo.SetParent(const AValue: TFPReportElement);
begin
  inherited SetParent(AValue);
  if UseParentFont then
    ReassignParentFont;
end;

procedure TFPReportCustomMemo.ReassignParentFont;

Var
  F : TFPReportFont;
  B : Boolean;

begin
  B:=UseParentFont;
  Try
    F:=GetParentFont;
    If Assigned(F) then
      Font.Assign(F);
  Finally
    FUseParentFont:=B;
  end;
end;

procedure TFPReportCustomMemo.ParentFontChanged;
begin
  inherited ParentFontChanged;
  if UseParentFont then
    begin
    ReassignParentFont;
    end;
end;

constructor TFPReportCustomMemo.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FIsExpr := False;
  FLinkColor := clBlue;
  FTextAlignment := CreateTextAlignment;
  FTextLines := TStringList.Create;
  FLineSpacing := 1; // millimeters
  FTextBlockList := TFPTextBlockList.Create;
  FOptions := [];
  FOriginal := nil;
  FUseParentFont := True;
  FFont := TFPReportFont.Create;
  FFont.OnChanged:=@HandleFontChange;
  ReassignParentFont;
  FCullThreshold := 75;
end;

destructor TFPReportCustomMemo.Destroy;
begin
  FreeAndNil(FTextLines);
  FreeAndNil(FTextBlockList);
  FreeAndNil(FTextAlignment);
  ClearExpressionNodes;
  FreeAndNil(FFont);
  inherited Destroy;
end;

class function TFPReportCustomMemo.ElementType: String;
begin
  Result:='Memo';
end;

procedure TFPReportCustomMemo.Assign(Source: TPersistent);
var
  E: TFPReportCustomMemo;
begin
  inherited Assign(Source);
  if (Source is TFPReportCustomMemo) then
  begin
    E := Source as TFPReportCustomMemo;
    Text := E.Text;
    CullThreshold := E.CullThreshold;
    Font.Assign(E.Font);
    UseParentFont := E.UseParentFont;
    LineSpacing := E.LineSpacing;
    LinkColor := E.LinkColor;
    TextAlignment.Assign(E.TextAlignment);
    Options := E.Options;
    Original := E;
    WordOverflow:= E.WordOverflow;
  end;
end;

procedure TFPReportCustomMemo.ReadElement(AReader: TFPReportStreamer);

var
  E: TObject;
begin
  inherited ReadElement(AReader);
  E := AReader.FindChild('TextAlignment');
  if Assigned(E) then
  begin
    AReader.PushElement(E);
    try
      FTextAlignment.ReadElement(AReader);
    finally
      AReader.PopElement;
    end;
  end;
  FText := AReader.ReadString('Text', '');
  FCullThreshold := AReader.ReadInteger('CullThreshold', CullThreshold);
  UseParentFont := AReader.ReadBoolean('UseParentFont', UseParentFont);
  if not UseParentFont then
    begin
    Font.Name := AReader.ReadString('FontName', Font.Name);
    Font.Size := AReader.ReadInteger('FontSize', Font.Size);
    Font.Color := QWordToReportColor(AReader.ReadQWord('FontColor', Font.Color));
    end
  else
    ReAssignParentFont;
  FLineSpacing := AReader.ReadFloat('LineSpacing', LineSpacing);
  FLinkColor := QWordToReportColor(AReader.ReadQWord('LinkColor', LinkColor));
  Options := StringToMemoOptions(AReader.ReadString('Options', ''));
  Changed;
end;

{ TFPReportCustomShape }

procedure TFPReportCustomShape.SetShapeType(AValue: TFPReportShapeType);
begin
  if FShapeType = AValue then
    Exit;
  FShapeType := AValue;
  Changed;
end;

procedure TFPReportCustomShape.SetOrientation(AValue: TFPReportOrientation);
begin
  if FOrientation = AValue then
    Exit;
  FOrientation := AValue;
  Changed;
end;

procedure TFPReportCustomShape.SetCornerRadius(AValue: TFPReportUnits);
begin
  if FCornerRadius = AValue then
    Exit;
  FCornerRadius := AValue;
  Changed;
end;

procedure TFPReportCustomShape.RecalcLayout;
var
  h: TFPReportUnits;
begin
  if StretchMode = smDontStretch then
    exit;
  h := Layout.Height;
  ApplyStretchMode(h);
end;

procedure TFPReportCustomShape.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  AWriter.WriteString('ShapeType', ShapeTypeToString(ShapeType));
  AWriter.WriteString('Orientation', OrientationToString(Orientation));
  AWriter.WriteFloat('CornerRadius', CornerRadius);
  AWriter.WriteQWord('Color', Color);
end;

constructor TFPReportCustomShape.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FOrientation := orNorth;
  FCornerRadius := 5.0;
  FShapeType := stEllipse;
  FColor:=clBlack;
end;

class function TFPReportCustomShape.ElementType: String;
begin
  Result:='Shape';
end;

procedure TFPReportCustomShape.Assign(Source: TPersistent);

Var
  S :  TFPReportCustomShape;

begin
  inherited Assign(Source);
  if Source is TFPReportCustomShape then
    begin
    S:=Source as TFPReportCustomShape;
    FOrientation:=S.Orientation;
    FCornerRadius:=S.CornerRadius;
    FShapeType:=S.ShapeType;
    FColor:=S.Color;
    end;
end;

procedure TFPReportCustomShape.ReadElement(AReader: TFPReportStreamer);
begin
  inherited ReadElement(AReader);
  ShapeType:=StringToShapeType(AReader.ReadString('ShapeType', ShapeTypeToString(ShapeType)));
  Orientation:=StringToOrientation(AReader.ReadString('Orientation', OrientationToString(Orientation)));
  CornerRadius:=AReader.ReadFloat('CornerRadius', CornerRadius);
  Color:=QWordToReportColor(AReader.ReadQWord('Color', Color));
end;

function TFPReportCustomShape.CreatePropertyHash: String;
begin
  Result:=inherited CreatePropertyHash;
  Result:=Result+'-'+IntToStr(Ord(ShapeType))
                +'-'+IntToStr(Ord(Orientation))
                +'-'+IntToStr(Color)
                +'-'+FormatFloat('000.###',CornerRadius);
end;

{ TFPReportCustomImage }

procedure TFPReportCustomImage.SetImage(AValue: TFPCustomImage);
begin
  if FImage = AValue then
    Exit;
  if Assigned(FImage) then
    FImage.Free;
  FImage := AValue;
  if Assigned(FImage) then
    FImageID := -1; { we are not using the global Report.Images here }
  Changed;
end;

procedure TFPReportCustomImage.SetStretched(AValue: boolean);
begin
  if FStretched = AValue then
    Exit;
  FStretched := AValue;
  Changed;
end;

procedure TFPReportCustomImage.SetFieldName(AValue: TFPReportString);
begin
  if FFieldName = AValue then
    Exit;
  FFieldName := AValue;
  Changed;
end;

procedure TFPReportCustomImage.LoadDBData(AData: TFPReportData);
var
  s: string;
  lStream: TMemoryStream;
begin
  s := AData.FieldValues[FFieldName];
  lStream := TMemoryStream.Create;
  try
    FPReportMIMEEncodeStringToStream(s, lStream);
    LoadPNGFromStream(lStream)
  finally
    lStream.Free;
  end;
end;

procedure TFPReportCustomImage.SetImageID(AValue: integer);
begin
  if FImageID = AValue then
    Exit;
  FImageID := AValue;
  Changed;
end;

function TFPReportCustomImage.GetImage: TFPCustomImage;
var
  c: integer;
  i: integer;
  img: TFPReportImageItem;
begin
  Result := nil;
  if ImageID = -1 then { images comes from report data }
    result := FImage
  else
  begin { image comes from global report.images list }
    c := Report.Images.Count-1;
    for i := 0 to c do
    begin
      img := Report.Images[i];
      if ImageID = img.ID then
      begin
         Result := TFPCustomImage(img.FImage);
         Exit;
      end;
    end; { for i }
  end;
end;

procedure TFPReportCustomImage.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
var
  idx: integer;
begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  { Even though we work with CollectionItem ID values, write the CollectionItem
    Index value instead. Why? Because when we read the report back, the Index
    and ID values will match. }
  idx := TFPReportCustomBand(Parent).Page.Report.Images.GetIndexFromID(ImageID);
  AWriter.WriteInteger('ImageIndex', idx);
  AWriter.WriteBoolean('Stretched', Stretched);
  AWriter.WriteString('FieldName', FieldName);
end;

procedure TFPReportCustomImage.RecalcLayout;
begin
  // Do nothing
end;

function TFPReportCustomImage.PrepareObject(aRTParent: TFPReportElement): TFPReportElement;

Var
  Img : TFPReportCustomImage;
  B : TFPReportCustomBand;

begin
  Result:=inherited PrepareObject(aRTParent);
  if Result=Nil then
    exit;
  img := TFPReportCustomImage(Result);
  B:=artParent as TFPReportCustomBand;
  if (img.FieldName <> '') and Assigned(B.GetData) then
    img.LoadDBData(B.GetData);
end;

constructor TFPReportCustomImage.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FImage := nil;
  FStretched := False;
  FImageID := -1;
end;

destructor TFPReportCustomImage.Destroy;
begin
  FImage.Free;
  inherited Destroy;
end;

class function TFPReportCustomImage.ElementType: String;
begin
  Result:='Image';
end;

function TFPReportCustomImage.GetRTImageID: Integer;
begin
  Result:=ImageID;
end;

function TFPReportCustomImage.GetRTImage: TFPCustomImage;
begin
  Result:=Image;
end;

procedure TFPReportCustomImage.Assign(Source: TPersistent);
var
  i: TFPReportCustomImage;
begin
  inherited Assign(Source);
  if Source is TFPReportCustomImage then
  begin
    i := (Source as TFPReportCustomImage);
    if Assigned(i.Image) then
    begin
      if not Assigned(FImage) then
        FImage := TFPCompactImgRGBA8Bit.Create(0, 0);
      FImage.Assign(i.Image);
    end;
    FStretched := i.Stretched;
    FFieldName := i.FieldName;
    FImageID := i.ImageID;
  end;
end;

procedure TFPReportCustomImage.ReadElement(AReader: TFPReportStreamer);
begin
  inherited ReadElement(AReader);
  { See code comments in DoWriteLocalProperties() }
  ImageID := AReader.ReadInteger('ImageIndex', -1);
  Stretched := AReader.ReadBoolean('Stretched', Stretched);
  FieldName := AReader.ReadString('FieldName', FieldName);
end;

procedure TFPReportCustomImage.LoadFromFile(const AFileName: string);

var
  R : TFPCustomReport;
  i : integer;
begin
  R:=Report;
  I:=R.Images.AddFromFile(AFileName,True);
  ImageID:=R.Images[I].ID;
end;

procedure TFPReportCustomImage.LoadFromStream(const AStream: TStream; aHandler: TFPCustomImageReaderClass);

var
  R : TFPCustomReport;
  i : integer;
begin
  R:=Report;
  I:=R.Images.AddFromStream(aStream,aHandler,True);
  ImageID:=R.Images[I].ID;
end;

procedure TFPReportCustomImage.LoadPNGFromStream(AStream: TStream);

var
  R : TFPCustomReport;
  i : integer;

begin
  R:=Report;
  I:=R.Images.AddFromStream(AStream,TFPReaderPNG,true);
  ImageID:=R.Images[I].ID;
end;

procedure TFPReportCustomImage.LoadImage(const AImageData: Pointer; const AImageDataSize: LongWord);

var
  s: TMemoryStream;

begin
  s := TMemoryStream.Create;
  try
    s.Write(AImageData^, AImageDataSize);
    s.Position := 0;
    LoadPNGFromStream(s);
  finally
    s.Free;
  end;
end;

{ TFPReportCustomCheckbox }

procedure TFPReportCustomCheckbox.SetExpression(AValue: TFPReportString);
begin
  if FExpression = AValue then
    Exit;
  FExpression := AValue;
  Changed;
end;

function TFPReportCustomCheckbox.LoadImage(const AImageData: Pointer; const AImageDataSize: LongWord): TFPCustomImage;
var
  s: TMemoryStream;
begin
  s := TMemoryStream.Create;
  try
    s.Write(AImageData^, AImageDataSize);
    Result := LoadImage(s);
  finally
    s.Free;
  end;
end;

function TFPReportCustomCheckbox.LoadImage(AStream: TStream): TFPCustomImage;
var
  img: TFPCompactImgRGBA8Bit;
  reader: TFPReaderPNG;
begin
  Result := nil;
  if AStream = nil then
    Exit;

  img := TFPCompactImgRGBA8Bit.Create(0, 0);
  try
    try
      AStream.Position := 0;
      reader := TFPReaderPNG.Create;
      img.LoadFromStream(AStream, reader); // auto sizes image
      Result := img;
    except
      on e: Exception do
      begin
        Result := nil;
      end;
    end;
  finally
    reader.Free;
  end;
end;

procedure TFPReportCustomCheckbox.RecalcLayout;
begin
  // Do nothing
end;

procedure TFPReportCustomCheckbox.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  AWriter.WriteString('Expression',Expression);
end;

function TFPReportCustomCheckbox.PrepareObject(aRTParent: TFPReportElement): TFPReportElement;

Var
  C : TFPReportCustomCheckbox;
  S : String;

begin
  Result:=inherited PrepareObject(aRTParent);
  if Result=Nil then
    exit;
  C:=TFPReportCustomCheckbox(Result);
  s:=ExpandMacro(C.Expression, True);
  C.FTestResult := StrToBoolDef(s, False);
end;

constructor TFPReportCustomCheckbox.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  { 3x3 millimeters }
  Layout.Width := 3;
  Layout.Height := 3;
  FTrueImageID:=-1;
  FFalseImageID:=-1;
end;

destructor TFPReportCustomCheckbox.Destroy;
begin
  inherited Destroy;
end;

class function TFPReportCustomCheckbox.ElementType: String;
begin
  Result:='Checkbox';
end;

function TFPReportCustomCheckbox.GetImage(Checked: Boolean): TFPCustomImage;

begin
  Result:=nil;
  if Checked then
    Result:=Report.Images.GetImageFromID(TrueImageID)
  else
    Result:=Report.Images.GetImageFromID(FalseImageID);
  if (Result=Nil) then
    Result:=GetDefaultImage(Checked);
end;

function TFPReportCustomCheckbox.GetRTResult: Boolean;
begin
  Result:=FTestResult;
end;

function TFPReportCustomCheckbox.GetRTImage: TFPCustomImage;
begin
  Result:=GetImage(GetRTResult);
end;

function TFPReportCustomCheckbox.GetDefaultImage(Checked: Boolean): TFPCustomImage;

begin
  if Checked then
    begin
    if (ImgTrue=Nil) then
      ImgTrue:=LoadImage(@fpreport_checkbox_true, SizeOf(fpreport_checkbox_true));
    Result:=ImgTrue;
    end
  else
    begin
    if (ImgFalse=Nil) then
      ImgFalse:=LoadImage(@fpreport_checkbox_false, SizeOf(fpreport_checkbox_false));
    Result:=ImgFalse;
    end;
end;

function TFPReportCustomCheckbox.CreatePropertyHash: String;
begin
  Result:=inherited CreatePropertyHash;
  Result:=Result+IntToStr(TrueImageID)+IntToStr(FalseImageID);
  Result:=Result+IntToStr(Ord(GetRTResult));
end;

procedure TFPReportCustomCheckbox.Assign(Source: TPersistent);
var
  cb: TFPReportCustomCheckbox;
begin
  inherited Assign(Source);
  if Source is TFPReportCustomCheckbox then
  begin
    cb := (Source as TFPReportCustomCheckbox);
    FTrueImageID:=cb.FTrueImageID;
    FFalseImageID:=cb.FFalseImageID;
    FExpression := cb.Expression;
  end;
end;

procedure TFPReportCustomCheckbox.ReadElement(AReader: TFPReportStreamer);
begin
  inherited ReadElement(AReader);
  Expression:=AReader.ReadString('Expression','');
end;


{ TFPReportUserData }

procedure TFPReportUserData.DoGetValue(const AFieldName: string; var AValue: variant);
begin
  inherited DoGetValue(AFieldName, AValue);
  if Assigned(FOnGetValue) then
    FOnGetValue(Self, AFieldName, AValue);
end;

Function TFPReportUserData.GetFieldDataType(Const AFieldName : String) : TFPReportFieldKind;

begin
  Result:=rfkString;
  If Assigned(OnGetFieldKind) then
    OnGetFieldKind(Self,AFieldName,Result);
end;

procedure TFPReportUserData.DoInitDataFields;

var
  sl: TStringList;
  i: integer;

begin
  inherited DoInitDataFields;
  if Assigned(FOnGetNames) then
    begin
    sl:= TStringList.Create;
    try
      FOnGetNames(self, sl);
      for i := 0 to sl.Count-1 do
        if (Datafields.IndexOfField(sl[i])=-1) then
          DataFields.AddField(sl[i],GetFieldDataType(sl[i]));
    finally
      SL.Free;
    end;
    end;
end;


{ TFPReportCustomDataBand }

procedure TFPReportCustomDataBand.SetFooterBand(AValue: TFPReportCustomDataFooterBand);
begin
  if FFooterBand=AValue then Exit;
  if Assigned(FFooterBand) then
    FFooterBand.RemoveFreeNotification(Self);
  FFooterBand:=AValue;
  if Assigned(FFooterBand) and Assigned(Self.Data) then
    FFooterBand.Data:=Self.Data;
end;

procedure TFPReportCustomDataBand.SetHeaderBand(AValue: TFPReportCustomDataHeaderBand);
begin
  if FHeaderBand=AValue then Exit;
  if Assigned(FHeaderBand) then
    FHeaderBand.RemoveFreeNotification(Self);
  FHeaderBand:=AValue;
  if Assigned(FHeaderBand) and Assigned(Self.Data) then
    FHeaderBand.Data:=Self.Data;
end;

procedure TFPReportCustomDataBand.SetMasterBand(AValue: TFPReportCustomDataBand);
begin
  if FMasterBand=AValue then Exit;
  if Assigned(FMasterBand) then
    FMasterBand.RemoveFreeNotification(Self);
  FMasterBand:=AValue;
  if Assigned(FMasterBand) then
    FMasterBand.FreeNotification(Self);
end;

procedure TFPReportCustomDataBand.FixupReference(const PN, PV: String; C: TFPReportElement);
begin
  If SameText('FooterBand',PN) and (C is TFPReportCustomDataFooterBand) then
    FooterBand:=TFPReportCustomDataFooterBand(C)
  else If SameText('HeaderBand',PN) and (C is TFPReportCustomDataHeaderBand) then
    HeaderBand:=TFPReportCustomDataHeaderBand(C)
  else If SameText('MasterBand',PN) and (C is TFPReportCustomDataBand) then
    MasterBand:=TFPReportCustomDataBand(C)
  else
    Inherited;
end;

procedure TFPReportCustomDataBand.Notification(AComponent: TComponent; Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if Operation=opRemove then
    begin
    if AComponent = FMasterBand then
      FMasterBand:=Nil;
    end;
end;

procedure TFPReportCustomDataBand.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement = nil);

  Procedure WBand(B : TFPReportCustomBand; Prefix : String);

  begin
    If Assigned(B) then
      AWriter.WriteString(Prefix+'Band',B.Name);
  end;

begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  WBand(MasterBand,'Master');
end;

procedure TFPReportCustomDataBand.ReadElement(AReader: TFPReportStreamer);

  Function RBand(Prefix : String) : TFPReportElement;

  Var
    S : String;

  begin
    Result:=Nil;
    S:=AReader.ReadString(Prefix+'Band','');
    if (S<>'') then
      begin
      Result:=Report.FindRecursive(S);
      if Result=Nil then
        Report.AddReference(Self,Prefix+'Band',S);
      end;
  end;

begin
  inherited ReadElement(AReader);
  FooterBand:=TFPReportCustomDataFooterBand(RBand('Footer'));
  HeaderBand:=TFPReportCustomDataHeaderBand(RBand('Header'));
  MasterBand:=TFPReportCustomDataBand(RBand('Master'));
end;

constructor TFPReportCustomDataBand.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FDisplayPosition := 0;
end;

{ TFPReportDataBand }

class function TFPReportDataBand.ElementType: String;
begin
  Result := 'DataBand';
end;

class function TFPReportDataBand.ReportBandType: TFPReportBandType;
begin
  Result:=btDataband;
end;

{ TFPReportCustomChildBand }

Class function TFPReportCustomChildBand.ElementType: string;
begin
  Result := 'ChildBand';
end;

procedure TFPReportCustomChildBand.Validate(aErrors: TStrings);

Var
  I : integer;
  B : TFPReportCustomBand;

begin
  inherited Validate(aErrors);
  if Not Assigned(Page) then
    exit;
  I:=0;
  B:=Nil;
  While (B=Nil) and (I<Page.BandCount) do
    begin
    B:=Page.Bands[i];
    if B.ChildBand<>Self then
      B:=Nil;
    Inc(I);
    end;
  if (B=Nil) then
    aErrors.Add(Format(SErrDanglingChild,[Name]));
end;

class function TFPReportCustomChildBand.ReportBandType: TFPReportBandType;
begin
  Result:=btChild;
end;

{ TFPReportCustomPageFooterBand }

class function TFPReportCustomPageFooterBand.ElementType: String;
begin
  Result := 'PageFooterBand';
end;

class function TFPReportCustomPageFooterBand.ReportBandType: TFPReportBandType;
begin
  Result:=btPageFooter;
end;

{ TFPReportCustomPageHeaderBand }

class function TFPReportCustomPageHeaderBand.ElementType: String;
begin
  Result := 'PageHeaderBand';
end;

class function TFPReportCustomPageHeaderBand.ReportBandType: TFPReportBandType;
begin
  Result:=btPageHeader;
end;

{ TFPReportCustomColumnHeaderBand }

class function TFPReportCustomColumnHeaderBand.ElementType: String;
begin
  Result := 'ColumnHeaderBand';
end;

class function TFPReportCustomColumnHeaderBand.ReportBandType: TFPReportBandType;
begin
  Result:=btColumnHeader;
end;

{ TFPReportCustomColumnFooterBand }

class function TFPReportCustomColumnFooterBand.ElementType: String;
begin
  Result := 'ColumnFooterBand';
end;

constructor TFPReportCustomColumnFooterBand.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FBandPosition := bpStackAtBottom;
end;

class function TFPReportCustomColumnFooterBand.ReportBandType: TFPReportBandType;
begin
  Result:=btColumnFooter;
end;

{ TFPReportCustomGroupHeaderBand }

procedure TFPReportCustomGroupHeaderBand.SetGroupHeader(AValue: TFPReportCustomGroupHeaderBand);
begin
  if FParentGroupHeader = AValue then
    Exit;
  if Assigned(FParentGroupHeader) then
  begin
    FParentGroupHeader.FChildGroupHeader := nil;
    FParentGroupHeader.RemoveFreeNotification(Self);
  end;
  FParentGroupHeader := AValue;
  if Assigned(FParentGroupHeader) then
  begin
    FParentGroupHeader.FChildGroupHeader := Self;
    FParentGroupHeader.FreeNotification(Self);
  end;
end;

class function TFPReportCustomGroupHeaderBand.ElementType: String;
begin
  Result := 'GroupHeaderBand';
end;

procedure TFPReportCustomGroupHeaderBand.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  AWriter.WriteString('GroupCondition', FGroupCondition);
  AWriter.WriteString('StartOnNewSection', ReportSectionToString(FStartOnNewSection));
  AWriter.WriteString('ReprintedHeader', ReportSectionsToString(FReprintedHeader));
  AWriter.WriteString('IntermediateFooter', ReportSectionsToString(FIntermediateFooter));
  AWriter.WriteString('OverflowedFooterNeedsReprintedHeader', ReportSectionsToString(FOverflowedFooterNeedsReprintedHeader));
  AWriter.WriteString('OverflowWithFirstDataBand', ReportSectionsToString(FOverflowWithFirstDataBand));
  If Assigned(ParentGroupHeader) then
    AWriter.WriteString('ParentGroupHeader', ParentGroupHeader.Name);
end;

procedure TFPReportCustomGroupHeaderBand.Notification(AComponent: TComponent; Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if (Operation = opRemove) then
    begin
    if (AComponent = FChildGroupHeader) then
      FChildGroupHeader := nil
    else if (AComponent = FParentGroupHeader) then
      FParentGroupHeader := nil;
    end;

end;

procedure TFPReportCustomGroupHeaderBand.FixupReference(const PN, PV: String; C: TFPReportElement);
begin
  if SameText(PN,'ParentGroupHeader') then
    ParentGroupHeader:=TFPReportCustomGroupHeaderBand(C)
  else
    inherited FixupReference(PN, PV, C);
end;

procedure TFPReportCustomGroupHeaderBand.BeforePrintWithChilds;
begin
  inherited BeforePrintWithChilds;
  { the new group begins: if IntermediateFooter is true,
    intermediate group footer is needed                 }
  FNeedsIntermediateFooter := True;
  { the new group begins:
    no details of this group are printed                }
  if not Report.FRTInRepeatedGroupHeader then
    FDetailsPrinted := False;
  Report.FRTGroupDetailsPrinted := FDetailsPrinted;
end;

procedure TFPReportCustomGroupHeaderBand.MovedToNextPageWithChilds;
begin
  inherited MovedToNextPageWithChilds;
  { the new group begins on next page: if IntermediateFooter it true,
    intermediate group footer is *not* needed on old page             }
  FNeedsIntermediateFooter := False;
end;

procedure TFPReportCustomGroupHeaderBand.AfterPrintWithChilds;
begin
  inherited AfterPrintWithChilds;
  { if header is shown then repeated header is needed on next page break }
  FNeedsReprintedHeader := True;
end;

procedure TFPReportCustomGroupHeaderBand.StoreRTBands(pBands: TBandList);

var
  i: Integer;

begin
  FRTBands.Clear;
  if OverflowWithFirstDataBand <> [] then
    for i := 0 to pBands.Count-1 do
      FRTBands.Add(pBands[i]);
end;

function TFPReportCustomGroupHeaderBand.NeedsOverflowWithFirstDataBand(
  pIsLastColumn: Boolean): Boolean;
begin
  Result := (
    pIsLastColumn and
    (rsPage in FOverflowWithFirstDataBand)
  ) or
  (rsColumn in FOverflowWithFirstDataBand);
end;

function TFPReportCustomGroupHeaderBand.NeedsIntermediateFooter(
  pIsLastColumn: Boolean): Boolean;
begin
  Result :=
  FNeedsIntermediateFooter and
  (
    (
      pIsLastColumn and
      (rsPage in IntermediateFooter)
    ) or
    (rsColumn in IntermediateFooter)
  );
end;

constructor TFPReportCustomGroupHeaderBand.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FParentGroupHeader := nil;
  FChildGroupHeader := nil;
  FGroupFooter := nil;
  FOverflowWithFirstDataBand := [rsPage, rsColumn];
  FStartOnNewSection := rsNone;
  FRTBands := TBandList.Create;
end;

destructor TFPReportCustomGroupHeaderBand.Destroy;
begin
  FRTBands.Free;
  inherited Destroy;
end;

procedure TFPReportCustomGroupHeaderBand.Validate(aErrors: TStrings);
begin
  inherited Validate(aErrors);
  If (GroupCondition='') then
    aErrors.Add(Format(SErrEmptyGroupExpression,[Name]));
end;

procedure TFPReportCustomGroupHeaderBand.Assign(Source: TPersistent);

var
  E: TFPReportCustomGroupHeaderBand;

begin
  inherited Assign(Source);
  if Source is TFPReportCustomGroupHeaderBand then
  begin
    E := TFPReportCustomGroupHeaderBand(Source);
    FGroupCondition := E.GroupCondition;
    FStartOnNewSection := E.StartOnNewSection;
    FReprintedHeader := E.ReprintedHeader;
    FIntermediateFooter := E.IntermediateFooter;
    FOverflowedFooterNeedsReprintedHeader := E.OverflowedFooterNeedsReprintedHeader;
    FOverflowWithFirstDataBand := E.OverflowWithFirstDataBand;
  end;
end;

procedure TFPReportCustomGroupHeaderBand.ReadElement(AReader: TFPReportStreamer);

Var
  S : String;
  C : TFPReportElement;

begin
  inherited ReadElement(AReader);
  FGroupCondition := AReader.ReadString('GroupCondition', '');
  FStartOnNewSection := StringToReportSection(AReader.ReadString('StartOnNewSection', 'rsNone'));
  FReprintedHeader := StringToReportSections(AReader.ReadString('ReprintedHeader', ''));
  FIntermediateFooter := StringToReportSections(AReader.ReadString('IntermediateFooter', ''));
  FOverflowedFooterNeedsReprintedHeader := StringToReportSections(AReader.ReadString('OverflowedFooterNeedsReprintedHeader', ''));
  FOverflowWithFirstDataBand := StringToReportSections(AReader.ReadString('OverflowWithFirstDataBand', 'rsPage,rsColumn'));
  S:=AReader.ReadString('ParentGroupHeader','');
  if (S<>'') then
    begin
    C:=Report.FindRecursive(S);
    if C is TFPReportCustomGroupHeaderBand then
      ParentGroupHeader:=TFPReportCustomGroupHeaderBand(C)
    else
      Report.AddReference(Self,'ParentGroupHeader',S);
    end;
end;

procedure TFPReportCustomGroupHeaderBand.EvaluateGroupCondition;
begin
  if Assigned(FChildGroupHeader) then
    FChildGroupHeader.EvaluateGroupCondition
  else
    InternalEvaluateGroupCondition;
end;

function TFPReportCustomGroupHeaderBand.GroupChanged: Boolean;
begin
  if Assigned(FParentGroupHeader) then
    Result := FParentGroupHeader.GroupChanged or
      (FLastGroupConditionValue <> FGroupConditionValue)
  else
    Result := FLastGroupConditionValue <> FGroupConditionValue;
end;

function TFPReportCustomGroupHeaderBand.IsInitialGroupChange: Boolean;
begin
  Result := GroupChanged and (FLastGroupConditionValue = '');
end;

procedure TFPReportCustomGroupHeaderBand.ResetGroupConditionValues;
begin
  FLastGroupConditionValue := '';
  FGroupConditionValue := '';
end;

procedure TFPReportCustomGroupHeaderBand.InternalEvaluateGroupCondition;
begin
  FLastGroupConditionValue := FGroupConditionValue;
  if Data.EOF then
    FGroupConditionValue := #255
  else
    FGroupConditionValue := EvaluateExpressionAsText(GroupCondition);
  if FLastGroupConditionValue <> FGroupConditionValue then
    { repated group header needs previous variables }
    FNeedsPrevVariables := True;
  if Assigned(FParentGroupHeader) then
    FParentGroupHeader.InternalEvaluateGroupCondition;
end;

procedure TFPReportCustomGroupHeaderBand.SetKeepTogetherWithChildren(
  pKeepTogetherWithChildren: Boolean);
begin
  inherited SetKeepTogetherWithChildren(pKeepTogetherWithChildren);
  if not FKeepTogetherWithChildren then
    FOverflowWithFirstDataBand := [];
end;

procedure TFPReportCustomGroupHeaderBand.SetOverflowedFooterNeedsReprintedHeader
  (pOverflowedFooterNeedsReprintedHeader: TFPReportSections);
begin
  if FOverflowedFooterNeedsReprintedHeader =
    pOverflowedFooterNeedsReprintedHeader then Exit;
  FOverflowedFooterNeedsReprintedHeader :=
    pOverflowedFooterNeedsReprintedHeader;
  if FOverflowedFooterNeedsReprintedHeader <> [] then
    { only works if ReprintedHeader is set }
    FReprintedHeader := FOverflowedFooterNeedsReprintedHeader;
end;

procedure TFPReportCustomGroupHeaderBand.SetOverflowWithFirstDataBand(
  pOverflowWithFirstDataBand: TFPReportSections);
begin
  if FOverflowWithFirstDataBand = pOverflowWithFirstDataBand then Exit;
  FOverflowWithFirstDataBand := pOverflowWithFirstDataBand;
  if FOverflowWithFirstDataBand <> [] then
    { only works if KeepTogetherWithChildren }
    FKeepTogetherWithChildren := True;
end;

procedure TFPReportCustomGroupHeaderBand.SetReprintedHeader(
  pReprintedHeader: TFPReportSections);
begin
  if FReprintedHeader = pReprintedHeader then Exit;
  FReprintedHeader := pReprintedHeader;
  if FReprintedHeader <> [rsPage, rsColumn] then
    { also unset OverflowedFooterNeedsReprintedHeader }
    if FReprintedHeader = [] then
      FOverflowedFooterNeedsReprintedHeader := []
    else if FReprintedHeader = [rsPage] then
      { remove rsColumn }
      FOverflowedFooterNeedsReprintedHeader := FOverflowedFooterNeedsReprintedHeader - [rsColumn]
    else if FReprintedHeader = [rsColumn] then
      { remove rsPage }
      FOverflowedFooterNeedsReprintedHeader := FOverflowedFooterNeedsReprintedHeader - [rsPage];
end;

procedure TFPReportCustomGroupHeaderBand.SetStartOnNewSection(
  pStartOnNewSection: TFPReportSection);
begin
  if FStartOnNewSection = pStartOnNewSection then
    Exit;
  FStartOnNewSection := pStartOnNewSection;
end;

class function TFPReportCustomGroupHeaderBand.ReportBandType: TFPReportBandType;
begin
  Result:=btGroupHeader;
end;

{ TFPReportCustomTitleBand }

class function TFPReportCustomTitleBand.ElementType: String;
begin
  Result := 'ReportTitleBand';
end;

class function TFPReportCustomTitleBand.ReportBandType: TFPReportBandType;
begin
  Result:=btReportTitle;
end;

{ TFPReportCustomSummaryBand }

class function TFPReportCustomSummaryBand.ElementType: String;
begin
  Result := 'ReportSummaryBand';
end;

procedure TFPReportCustomSummaryBand.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  AWriter.WriteBoolean('StartNewPage', StartNewPage);
end;

constructor TFPReportCustomSummaryBand.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FStartNewPage := False;
end;

procedure TFPReportCustomSummaryBand.Assign(Source: TPersistent);
begin
  inherited Assign(Source);
  if Source is TFPReportCustomSummaryBand then
    FStartNewPage := TFPReportCustomSummaryBand(Source).StartNewPage;
end;

procedure TFPReportCustomSummaryBand.ReadElement(AReader: TFPReportStreamer);
begin
  inherited ReadElement(AReader);
  FStartNewPage := AReader.ReadBoolean('StartNewPage', False);
end;

class function TFPReportCustomSummaryBand.ReportBandType: TFPReportBandType;
begin
  Result:=btReportSummary;
end;

{ TFPReportComponent }

procedure TFPReportComponent.StartLayout;
begin
  FReportState := rsLayout;
end;

procedure TFPReportComponent.EndLayout;
begin
  FReportState := rsDesign;
end;

procedure TFPReportComponent.StartRender;
begin
  FReportState := rsRender;
end;

procedure TFPReportComponent.EndRender;
begin
  FReportState := rsDesign;
end;

function TFPReportComponent.AllocateName: String;

Var
  BaseName : String;
  I : Integer;

begin
  BaseName:=ClassName;
  If SameText(Copy(BaseName,1,9),'TFPReport') then
    Delete(BaseName,1,9)
  else If SameText(Copy(BaseName,1,1),'T') then
    Delete(BaseName,1,1);
  I:=1;
  Repeat
    Result:=BaseName+IntToStr(I);
    Inc(I);
  Until (Owner=Nil) or (Owner.FindComponent(Result)=Nil);
end;

procedure TFPReportComponent.FixupReference(const PN, PV: String; C: TFPReportElement);
begin
  // Do nothing
end;

procedure TFPReportComponent.StartDesigning;
begin
  SetDesigning(True,True);
  FReportState:=rsDesign;
end;

procedure TFPReportComponent.EndDesigning;
begin
  SetDesigning(False,True);
  FReportState:=rsDesign;
end;

procedure TFPReportComponent.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  AWriter.WriteString('Name', Name);
end;

procedure TFPReportComponent.ReadElement(AReader: TFPReportStreamer);

Var
  N : String;
  C : TComponent;
begin
  try
    N := AReader.ReadString('Name', 'UnknownName');
    if Assigned(Owner) and (N<>'') then
      begin
      C:=Owner.FindComponent(N);
      if (C<>Self) and (C<>Nil) then
        begin
        N:=AllocateName;
        AReader.Modified;
        end;
      end;
    Name:=N;
  except
    On E : EComponentError do
      begin
      // This should never happen, but we leave it in place just in case.
      Name:=AllocateName;
      AReader.Modified;
      end;
  end;
end;

{ TFPReportRect }

procedure TFPReportRect.SetRect(aleft, atop, awidth, aheight: TFPReportUnits);
begin
  Left   := aleft;
  Top    := atop;
  Width  := awidth;
  Height := aheight;
end;

procedure TFPReportRect.OffsetRect(aLeft, ATop: TFPReportUnits);
begin
  Left:=Left+ALeft;
  Top:=Top+ATop;
end;

function TFPReportRect.IsEmpty: Boolean;
begin
  Result:=(Width=0) and (Height=0);
end;

function TFPReportRect.Bottom: TFPReportUnits;
begin
  Result := Top + Height;
end;

function TFPReportRect.Right: TFPReportUnits;
begin
  Result := Left + Width;
end;

function TFPReportRect.AsString: String;
begin
  Result:=Format('(x: %5.2f, y: %5.2f, w: %5.2f, h: %5.2f)',[Left,Top,Width,Height]);
end;

{ TFPReportFrame }

procedure TFPReportFrame.SetColor(const AValue: TFPReportColor);
begin
  if FColor = AValue then
    Exit;
  FColor := AValue;
  Changed;
end;

procedure TFPReportFrame.SetFrameLines(const AValue: TFPReportFrameLines);
begin
  if FFrameLines = AValue then
    Exit;
  FFrameLines := AValue;
  Changed;
end;

procedure TFPReportFrame.SetFrameShape(const AValue: TFPReportFrameShape);
begin
  if FFrameShape = AValue then
    Exit;
  FFrameShape := AValue;
  Changed;
end;

procedure TFPReportFrame.SetPenStyle(const AValue: TFPPenStyle);
begin
  if FPenStyle = AValue then
    Exit;
  FPenStyle := AValue;
  Changed;
end;

procedure TFPReportFrame.SetWidth(const AValue: integer);
begin
  if FWidth = AValue then
    Exit;
  if (AValue < 0) then
    ReportError(SErrInvalidLineWidth, [AValue]);
  FWidth := AValue;
  Changed;
end;

procedure TFPReportFrame.SetBackgrounColor(AValue: TFPReportColor);
begin
  if FBackgroundColor = AValue then
    Exit;
  FBackgroundColor := AValue;
  Changed;
end;

procedure TFPReportFrame.Changed;
begin
  if Assigned(FReportElement) then
    FReportElement.Changed;
end;

constructor TFPReportFrame.Create(AElement: TFPReportElement);
begin
  inherited Create;
  FReportElement := AElement;
  FPenStyle := psSolid;
  FWidth := 1;
  FColor := clNone;
  FBackgroundColor := clNone;
  FFrameShape := fsNone;
end;

procedure TFPReportFrame.Assign(ASource: TPersistent);
var
  F: TFPReportFrame;
begin
  if (ASource is TFPReportFrame) then
  begin
    F := (ASource as TFPReportFrame);
    FFrameLines := F.FFrameLines;
    FFrameShape := F.FFrameShape;
    FColor := F.FColor;
    FBackgroundColor := F.BackgroundColor;
    FPenStyle := F.FPenStyle;
    FWidth := F.FWidth;
    Changed;
  end
  else
    inherited Assign(ASource);
end;

function TFPReportFrame.Equals(AFrame: TFPReportFrame): boolean;
begin
  Result := (AFrame = Self) or ((Color = AFrame.Color) and (Pen = AFrame.Pen) and
    (Width = AFrame.Width) and (Shape = AFrame.Shape) and (Lines = AFrame.Lines) and
    (BackgroundColor = AFrame.BackgroundColor));
end;


procedure TFPReportFrame.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportFrame);
var
  I, J: integer;
begin
  if (AOriginal = nil) then
  begin
    AWriter.WriteQWord('Color', Color);
    AWriter.WriteString('Pen', FramePenToString(Pen));
    AWriter.WriteInteger('Width', Ord(Width));
    AWriter.WriteString('Shape', FrameShapeToString(Shape));
    //TODO Write out the enum values instead of the Integer value.
    I := integer(Lines);
    AWriter.WriteInteger('Lines', I);
    AWriter.WriteQWord('BackgroundColor', BackgroundColor);
  end
  else
  begin
    AWriter.WriteQWordDiff('Color', Color, AOriginal.Color);
    AWriter.WriteStringDiff('Pen', FramePenToString(Pen), FramePenToString(AOriginal.Pen));
    AWriter.WriteIntegerDiff('Width', Ord(Width), AOriginal.Width);
    AWriter.WriteStringDiff('Shape', FrameShapeToString(Shape), FrameShapeToString(AOriginal.Shape));
    I := integer(Lines);
    J := integer(Aoriginal.Lines);
    AWriter.WriteIntegerDiff('Lines', I, J);
    AWriter.WriteQWordDiff('BackgroundColor', BackgroundColor, AOriginal.BackgroundColor);
  end;
end;

procedure TFPReportFrame.ReadElement(AReader: TFPReportStreamer);
var
  I: integer;
begin
  Color := QWordToReportColor(AReader.ReadQWord('Color', Color ));
  Pen := StringToFramePen(AReader.ReadString('Pen', 'psSolid'));
  Width := AReader.ReadInteger('Width', Ord(Width));
  Shape := StringToFrameShape(AReader.ReadString('Shape', 'fsNone'));
  I := integer(Lines);
  Lines := TFPReportFrameLines(AReader.ReadInteger('Lines', I));
  BackgroundColor := QWordToReportColor(AReader.ReadQWord('BackgroundColor', BackgroundColor));
end;

{ TFPReportTextAlignment }

procedure TFPReportTextAlignment.SetHorizontal(AValue: TFPReportHorzTextAlignment);
begin
  if FHorizontal = AValue then
    Exit;
  FHorizontal := AValue;
  Changed;
end;

procedure TFPReportTextAlignment.SetVertical(AValue: TFPReportVertTextAlignment);
begin
  if FVertical = AValue then
    Exit;
  FVertical := AValue;
  Changed;
end;

procedure TFPReportTextAlignment.SetTopMargin(AValue: TFPReportUnits);
begin
  if FTopMargin = AValue then
    Exit;
  FTopMargin := AValue;
  Changed;
end;

procedure TFPReportTextAlignment.SetBottomMargin(AValue: TFPReportUnits);
begin
  if FBottomMargin = AValue then
    Exit;
  FBottomMargin := AValue;
  Changed;
end;

procedure TFPReportTextAlignment.SetLeftMargin(AValue: TFPReportUnits);
begin
  if FLeftMargin = AValue then
    Exit;
  FLeftMargin := AValue;
  Changed;
end;

procedure TFPReportTextAlignment.SetRightMargin(AValue: TFPReportUnits);
begin
  if FRightMargin = AValue then
    Exit;
  FRightMargin := AValue;
  Changed;
end;

procedure TFPReportTextAlignment.Changed;
begin
  if Assigned(FReportElement) then
    FReportElement.Changed;
end;

constructor TFPReportTextAlignment.Create(AElement: TFPReportElement);
begin
  inherited Create;
  FReportElement := AElement;
  FHorizontal := taLeftJustified;
  FVertical := tlTop;
  FLeftMargin := 1.0;
  FRightMargin := 1.0;
  FTopMargin := 0;
  FBottomMargin := 0;
end;

procedure TFPReportTextAlignment.Assign(ASource: TPersistent);
var
  F: TFPReportTextAlignment;
begin
  if (ASource is TFPReportTextAlignment) then
  begin
    F := (ASource as TFPReportTextAlignment);
    FHorizontal   := F.Horizontal;
    FVertical     := F.Vertical;
    FLeftMargin   := F.LeftMargin;
    FRightMargin  := F.RightMargin;
    FTopMargin    := F.TopMargin;
    FBottomMargin := F.BottomMargin;
    Changed;
  end
  else
    inherited Assign(ASource);
end;

procedure TFPReportTextAlignment.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportTextAlignment);
begin
  if (AOriginal = nil) then
  begin
    AWriter.WriteString('Horizontal', HorzTextAlignmentToString(Horizontal));
    AWriter.WriteString('Vertical', VertTextAlignmentToString(Vertical));
    AWriter.WriteFloat('LeftMargin', LeftMargin);
    AWriter.WriteFloat('RightMargin', RightMargin);
    AWriter.WriteFloat('TopMargin', TopMargin);
    AWriter.WriteFloat('BottomMargin', BottomMargin);
  end
  else
  begin
    AWriter.WriteStringDiff('Horizontal', HorzTextAlignmentToString(Horizontal), HorzTextAlignmentToString(AOriginal.Horizontal));
    AWriter.WriteStringDiff('Vertical', VertTextAlignmentToString(Vertical), VertTextAlignmentToString(AOriginal.Vertical));
    AWriter.WriteFloatDiff('LeftMargin', LeftMargin, AOriginal.LeftMargin);
    AWriter.WriteFloatDiff('RightMargin', RightMargin, AOriginal.RightMargin);
    AWriter.WriteFloatDiff('TopMargin', TopMargin, AOriginal.TopMargin);
    AWriter.WriteFloatDiff('BottomMargin', BottomMargin, AOriginal.BottomMargin);
  end;
end;

procedure TFPReportTextAlignment.ReadElement(AReader: TFPReportStreamer);
begin
  Horizontal := StringToHorzTextAlignment(AReader.ReadString('Horizontal', 'taLeftJustified'));
  Vertical := StringToVertTextAlignment(AReader.ReadString('Vertical', 'tlTop'));
  LeftMargin := AReader.ReadFloat('LeftMargin', LeftMargin);
  RightMargin := AReader.ReadFloat('RightMargin', RightMargin);
  TopMargin := AReader.ReadFloat('TopMargin', TopMargin);
  BottomMargin := AReader.ReadFloat('BottomMargin', BottomMargin);
end;

{ TFPReportCustomLayout }

function TFPReportCustomLayout.GetWidth: TFPreportUnits;
begin
  Result := FPos.Width;
end;

function TFPReportCustomLayout.GetHeight: TFPreportUnits;
begin
  Result := FPos.Height;
end;

procedure TFPReportCustomLayout.SetLeft(const AValue: TFPreportUnits);
begin
  if (AValue = FPos.Left) then
    Exit;
  FPos.Left := AValue;
  Changed;
end;

procedure TFPReportCustomLayout.SetTop(const AValue: TFPreportUnits);
begin
  if (AValue = FPos.Top) then
    Exit;
  FPos.Top := AValue;
  Changed;
end;

procedure TFPReportCustomLayout.SetWidth(const AValue: TFPreportUnits);
begin
  if (AValue = FPos.Width) then
    Exit;
  FPos.Width := AValue;
  Changed;
end;

procedure TFPReportCustomLayout.SetHeight(const AValue: TFPreportUnits);
begin
  if (AValue = FPos.Height) then
    Exit;
  FPos.Height := AValue;
  Changed;
end;

function TFPReportCustomLayout.GetLeft: TFPreportUnits;
begin
  Result := FPos.Left;
end;

function TFPReportCustomLayout.GetTop: TFPreportUnits;
begin
  Result := FPos.Top;
end;

constructor TFPReportCustomLayout.Create(AElement: TFPReportElement);
begin
  FReportElement := AElement;
end;

procedure TFPReportCustomLayout.Assign(Source: TPersistent);
var
  l: TFPReportCustomLayout;
begin
  if Source is TFPReportCustomLayout then
  begin
    l := (Source as TFPReportCustomLayout);
    FPos.Height := l.Height;
    FPos.Left := l.Left;
    FPos.Top := l.Top;
    FPos.Width := l.Width;
  end
  else
    inherited Assign(Source);
end;

procedure TFPReportCustomLayout.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportCustomLayout);
begin
  if (AOriginal = nil) then
  begin
    AWriter.WriteFloat('Top', Top);
    AWriter.WriteFloat('Left', Left);
    AWriter.WriteFloat('Width', Width);
    AWriter.WriteFloat('Height', Height);
  end
  else
  begin
    AWriter.WriteFloatDiff('Top', Top, AOriginal.Top);
    AWriter.WriteFloatDiff('Left', Left, AOriginal.Left);
    AWriter.WriteFloatDiff('Width', Width, AOriginal.Width);
    AWriter.WriteFloatDiff('Height', Height, AOriginal.Height);
  end;
end;

procedure TFPReportCustomLayout.ReadElement(AReader: TFPReportStreamer);
begin
  FPos.Top := AReader.ReadFloat('Top', Top);
  FPos.Left := AReader.ReadFloat('Left', Left);
  FPos.Width := AReader.ReadFloat('Width', Width);
  FPos.Height := AReader.ReadFloat('Height', Height);
  Changed;
end;

function TFPReportCustomLayout.Equals(ALayout: TFPReportCustomLayout): boolean;
begin
  Result := (ALayout = Self) or ((ALayout.FPos.Top = FPos.Top) and (ALayout.FPos.Left = FPos.Left) and
    (ALayout.FPos.Right = FPos.Right) and (ALayout.FPos.Bottom = FPos.Bottom));
end;

procedure TFPReportCustomLayout.GetBoundsRect(out ARect: TFPReportRect);
begin
  ARect := FPos;
end;

procedure TFPReportCustomLayout.SetPosition(aleft, atop, awidth, aheight: TFPReportUnits);
begin
  FPos.SetRect(aleft,aTop,aWidth,aHeight);
  Changed;
end;

procedure TFPReportCustomLayout.SetPosition(const ARect: TFPReportRect);
begin
  FPos.SetRect(ARect.Left,ARect.Top,ARect.Width,ARect.Height);
  Changed;
end;

{ TFPReportLayout }

procedure TFPReportLayout.Changed;
begin
  if Assigned(FReportElement) then
    FReportElement.Changed;
end;

{ TFPReportElement }

procedure TFPReportElement.SetFrame(const AValue: TFPReportFrame);
begin
  if FFrame = AValue then
    Exit;
  BeginUpdate;
  try
    FFrame.Assign(AValue);
  finally
    EndUpdate;
  end;
end;

function TFPReportElement.GetReport: TFPCustomReport;

Var
  El : TFpReportElement;

begin
  Result:=Nil;
  El:=Self;
  While Assigned(El) and not El.InheritsFrom(TFPReportCustomPage) do
    El:=El.Parent;
  if Assigned(El) then
    Result:=TFPReportCustomPage(el).Report;
end;

function TFPReportElement.GetReportBand: TFPReportCustomBand;
begin
  if (Self is TFPReportCustomBand) then
    Result := Self as TFPReportCustomBand
  else if (Parent is TFPReportCustomBand) then
    Result := Parent as TFPReportCustomBand
  else
    Result := nil;
end;

function TFPReportElement.EvaluateVisibility: boolean;

var
  Res : TFPExpressionResult;

begin
  Result := Visible;
  if Result and (FVisibleExpr <> '') then 
  begin
    if EvaluateExpression(FVisibleExpr,res) then
      if (res.ResultType=rtBoolean) then // We may need to change this.
        Result:= Res.ResBoolean;
    {$ifdef gdebug}
    writeln('TFPReportElement.EvaluateVisibility: VisibleExpr=' , FVisibleExpr , '=' , Res.ResultType,' : ',Res.ResBoolean);
    {$endif}
  end;
end;

procedure TFPReportElement.SetLayout(const AValue: TFPReportLayout);
begin
  if FLayout = AValue then
    Exit;
  BeginUpdate;
  try
    FLayout.Assign(AValue);
  finally
    EndUpdate;
  end;
end;

procedure TFPReportElement.SetParent(const AValue: TFPReportElement);

begin
  if FParent = AValue then
    Exit;
  if (AValue <> nil) and not (AValue is TFPReportElementWithChildren) then
    ReportError(SErrInvalidParent, [AValue.Name, Self.Name]);
  if Assigned(FParent) then
    TFPReportElementWithChildren(FParent).RemoveChild(Self);
  FParent := AValue;
  if Assigned(FParent) then
  begin
    TFPReportElementWithChildren(FParent).AddChild(Self);
    FParent.FreeNotification(self);
  end;
  Changed;
end;

procedure TFPReportElement.SetVisible(const AValue: boolean);
begin
  if FVisible = AValue then
    Exit;
  FVisible := AValue;
  Changed;
end;

procedure TFPReportElement.SetVisibleExpr(AValue: String);
begin
  if FVisibleExpr=AValue then Exit;
  FVisibleExpr:=AValue;
  Changed;
end;

procedure TFPReportElement.ParentFontChanged;
begin
  // Do nothing;
end;

function TFPReportElement.ExpandMacro(const s: String; const AIsExpr: boolean): TFPReportString;

var
  pstart: integer;
  pend: integer;
  len: integer;
  m: string;  // macro
  mv: string; // macro value
  r: string;
  lFoundMacroInMacro: boolean;

begin
  r := s;
  lFoundMacroInMacro := False;
  pstart := Pos('[', r);
  while (pstart > 0) or lFoundMacroInMacro do
  begin
    if lFoundMacroInMacro then
    begin
      pstart := Pos('[', r);
      lFoundMacroInMacro := False;
    end;
    len := Length(r);
    pend := pstart + 2;
    while pend < len do
    begin
      if r[pend] = '[' then  // a macro inside a macro
      begin
        lFoundMacroInMacro := True;
        pstart := pend;
      end;
      if r[pend] = ']' then
        break
      else
        inc(pend);
    end;

    m := Copy(r, pstart, (pend-pstart)+1);
    len := Length(m);
    try
      if Assigned(Band.GetData) then
      begin
        try
          mv := Band.GetData.FieldValues[Copy(m, 2, len-2)];
        except
          on e: EVariantTypeCastError do  // maybe we have an expression not data field
          begin
            mv := EvaluateExpressionAsText(Copy(m, 2, len-2));
          end;
        end;
      end
      else
      begin // No Data assigned, but maybe we have an expression
        mv := EvaluateExpressionAsText(Copy(m, 2, len-2));
      end;
    except
      on e: EVariantTypeCastError do    // ReportData.OnGetValue did not handle all macros, so handle this gracefully
        mv := SErrUnknownMacro+': '+copy(m,2,len-2);
      on e: EExprParser do
        mv := SErrUnknownMacro+': '+copy(m,2,len-2);
    end;
    r := StringReplace(r, m, mv, [rfReplaceAll, rfIgnoreCase]);
    // look for more macros
    pstart := PosEx('[', r,PStart+Length(mv));
  end;
  { This extra check is mostly for ReportGroupHeader expression processing }
  if (pstart = 0) and Assigned(Band.GetData) and AIsExpr then
  begin
    try
      r := EvaluateExpressionAsText(r);
    except
      on E: Exception do
      begin
        { $ifdef gdebug}
        writeln('ERROR in expression: ', E.Message);
        { $endif}
        // do nothing - move on as we probably handled the expression a bit earlier in this code
      end;
    end;
  end;
  Result := r;
end;

function TFPReportElement.GetReportPage: TFPReportCustomPage;
begin
  if Assigned(Band) then
    Result := Band.Page
  else
    Result:=Nil;
end;

procedure TFPReportElement.SaveDataToNames;
begin
  // Do nothing
end;

procedure TFPReportElement.RestoreDataFromNames;
begin
  // Do nothing
end;

function TFPReportElement.CreateFrame: TFPReportFrame;
begin
  Result := TFPReportFrame.Create(Self);
end;

function TFPReportElement.CreateLayout: TFPReportLayout;
begin
  Result := TFPReportLayout.Create(Self);
end;

procedure TFPReportElement.CreateRTLayout;
begin
  FRTLayout := TFPReportLayout.Create(self);
  FRTLayout.Assign(FLayout);
end;

procedure TFPReportElement.Changed;
begin
  if (FUpdateCount = 0) then
    DoChanged;
end;

procedure TFPReportElement.DoChanged;
begin
  // Do nothing
end;

function TFPReportElement.PrepareObject(aRTParent: TFPReportElement): TFPReportElement;
begin
  Result := nil;
  if not self.EvaluateVisibility then
    Exit;
  if Parent is TFPReportCustomBand then
    begin
    Result:=TFPReportElementClass(Self.ClassType).Create(aRTParent);
    Result.Assign(self);
    Result.CreateRTLayout;
    end;
end;

procedure TFPReportElement.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  // will be implemented by descendant classes
end;

constructor TFPReportElement.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FLayout := CreateLayout;
  FFrame := CreateFrame;
  FVisible := True;
  FStretchMode := smDontStretch;
  if AOwner is TFPReportElement then
    Parent := TFPReportElement(AOwner);
end;

destructor TFPReportElement.Destroy;
begin
  FreeAndNil(FLayout);
  FreeAndNil(FFrame);
  if Assigned(FParent) then
    (FParent as TFPReportElementWithChildren).RemoveChild(Self);
  FreeAndNil(FRTLayout);
  inherited Destroy;
end;

class function TFPReportElement.ElementType: String;
begin
  Result:=ClassName;
end;

class function TFPReportElement.RegisterElement: TFPReportClassMapping;
begin
  Result:=gElementFactory.RegisterClass(ElementType,Self);
end;

class procedure TFPReportElement.UnRegisterElement;
begin
  gElementFactory.RemoveClass(ElementType);
end;

function TFPReportElement.CreatePropertyHash: String;

Var
  L : TFPReportCustomLayout;

begin
  if Assigned(RTLayout) then
    L:=RTLayout
  else
    L:=Layout;
  if Assigned(L) then
    Result:=Format('%6.3f%6.3f',[L.Width,L.Height]);
end;

procedure TFPReportElement.Notification(AComponent: TComponent; Operation: TOperation);
begin
  if Operation = opRemove then
  begin
    if AComponent = FParent then
      FParent := nil;
  end;
  inherited Notification(AComponent, Operation);
end;

procedure TFPReportElement.BeforePrint;
begin
  if Assigned(FOnBeforePrint) then
    FOnBeforePrint(self);
end;

function TFPReportElement.EvaluateExpression(const AExpr: String; out
  Res: TFPExpressionResult): Boolean;

var
  lExpr: TFPExpressionParser;

begin
  Result:=Assigned(Report);
  if not Result then
    exit;
  lExpr := Report.FExpr;
  lExpr.Expression := AExpr;
  Res:=lExpr.Evaluate;
end;

function TFPReportElement.GetDateTimeFormat: String;

begin
  Result:='yyyy-mm-dd';
end;


function TFPReportElement.ExpressionResultToString(const Res : TFPExpressionResult): String;

begin
  case Res.ResultType of
    rtString  : Result := Res.ResString;
    rtInteger : Result := IntToStr(Res.ResInteger);
    rtFloat   : Result := FloatToStr(Res.ResFloat);
    rtCurrency  : Result := CurrToStr(Res.ResCurrency);
    rtBoolean : Result := BoolToStr(Res.resBoolean, True);
    rtDateTime : Result := FormatDateTime(GetDateTimeFormat, Res.resDateTime);
  end;
end;

function TFPReportElement.EvaluateExpressionAsText(const AExpr: String): String;

Var
  Res : TFPExpressionResult;

begin
  Result:='';
  if EvaluateExpression(AExpr,Res) then
    Result:=ExpressionResultToString(Res);
end;


function TFPReportElement.Equals(AElement: TFPReportElement): boolean;
begin
  Result := (AElement = Self) or ((AElement.ClassType = AElement.ClassType) and
    (AElement.Frame.Equals(Self.Frame)) and (AElement.Layout.Equals(Self.Layout))
    and (AElement.Visible = Self.Visible) and (AElement.VisibleExpr = Self.VisibleExpr));
end;

procedure TFPReportElement.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
var
  L: TFPReportCustomLayout;
  F: TFPReportFrame;
begin
  inherited WriteElement(AWriter, AOriginal);
  if (AOriginal <> nil) then
  begin
    L := AOriginal.Layout;
    F := AOriginal.Frame;
  end
  else
  begin
    F := nil;
    L := nil;
  end;
  // Always write Layout.
  AWriter.PushElement('Layout');
  try
    FLayout.WriteElement(AWriter, L);
  finally
    AWriter.PopElement;
  end;
  // now for the Frame
  if (not Assigned(F)) or (not F.Equals(FFrame)) then
  begin
    AWriter.PushElement('Frame');
    try
      FFrame.WriteElement(AWriter, F);
    finally
      AWriter.PopElement;
    end;
  end;
  AWriter.WriteBoolean('Visible', FVisible);
  AWriter.WriteString('VisibleExpr', FVisibleExpr);
  AWriter.WriteString('StretchMode', StretchModeToString(StretchMode));
  DoWriteLocalProperties(AWriter, AOriginal);
end;

procedure TFPReportElement.ReadElement(AReader: TFPReportStreamer);
var
  E: TObject;
begin
  inherited ReadElement(AReader);
  E := AReader.FindChild('Layout');
  if Assigned(E) then
  begin
    AReader.PushElement(E);
    try
      FLayout.ReadElement(AReader);
    finally
      AReader.PopElement;
    end;
  end;
  E := AReader.FindChild('Frame');
  if Assigned(E) then
  begin
    AReader.PushElement(E);
    try
      FFrame.ReadElement(AReader);
    finally
      AReader.PopElement;
    end;
  end;
  FVisible := AReader.ReadBoolean('Visible', Visible);
  FVisibleExpr := AReader.ReadString('VisibleExpr', FVisibleExpr);
  FStretchMode := StringToStretchMode(AReader.ReadString('StretchMode', 'smDontStretch'));
  // TODO: implement reading OnBeforePrint information
end;

procedure TFPReportElement.Assign(Source: TPersistent);
var
  E: TFPReportElement;
begin
  { Don't call inherited here. }
//  inherited Assign(Source);
  if (Source is TFPReportElement) then
  begin
    E := Source as TFPReportElement;
    Frame.Assign(E.Frame);
    Layout.Assign(E.Layout);
    //FParent := E.Parent; // this is nonsense (e.g. in Destroy we need then parent used in Create)
    Visible := E.Visible;
    VisibleExpr := E.VisibleExpr;
    StretchMode := E.StretchMode;
    OnBeforePrint := E.OnBeforePrint;
  end;
end;

procedure TFPReportElement.BeginUpdate;
begin
  Inc(FUpdateCount);
end;

procedure TFPReportElement.EndUpdate;
begin
  if (FUpdateCount > 0) then
    Dec(FUpdateCount);
  if (FUpdateCount = 0) then
    DoChanged;
end;

procedure TFPReportElement.Validate(aErrors: TStrings);
begin
  // Do nothing
end;

{ TFPReportElementWithChildren }

function TFPReportElementWithChildren.GetChild(AIndex: integer): TFPReportElement;
begin
  if Assigned(FChildren) then
    Result := TFPReportElement(FChildren[AIndex])
  else
    ReportError(SErrInvalidChildIndex, [0]);
end;

function TFPReportElementWithChildren.GetChildCount: integer;
begin
  if Assigned(FChildren) then
    Result := FChildren.Count
  else
    Result := 0;
end;

procedure TFPReportElementWithChildren.NotifyFontChange;

Var
  I : Integer;

begin
  For I:=0 to ChildCount-1 do
    Child[i].ParentFontChanged;
end;

procedure TFPReportElementWithChildren.SaveDataToNames;

Var
  I :Integer;

begin
  inherited SaveDataToNames;
  For I:=0 to ChildCount-1 do
    Child[i].SaveDataToNames;
end;

procedure TFPReportElementWithChildren.RestoreDataFromNames;

Var
  I :Integer;
begin
  inherited RestoreDataFromNames;
  For I:=0 to ChildCount-1 do
    Child[i].RestoreDataFromNames;
end;

procedure TFPReportElementWithChildren.RemoveChild(const AChild: TFPReportElement);
begin
  if Assigned(FChildren) then
  begin
    FChildren.Remove(AChild);
    if FChildren.Count = 0 then
      FreeAndNil(FChildren);
  end;
  if not (csDestroying in ComponentState) then
    Changed;
end;

procedure TFPReportElementWithChildren.AddChild(const AChild: TFPReportElement);
begin
  if not Assigned(FChildren) then
    FChildren := TFPList.Create;
  FChildren.Add(AChild);
  if not (csDestroying in ComponentState) then
    Changed;
end;

procedure TFPReportElementWithChildren.PrepareObjects(aRTParent: TFPReportElement);
var
  i: integer;
begin
  for i := 0 to ChildCount - 1 do
    Child[i].PrepareObject(aRTParent);
end;

procedure TFPReportElementWithChildren.RecalcLayout;

var
  i: integer;
begin
  if Assigned(FChildren) then
    for i := 0 to FChildren.Count -1 do
      Child[i].RecalcLayout;
end;

procedure TFPReportElementWithChildren.ApplyStretchMode(const ADesiredHeight: TFPReportUnits);

var
   OldH,H: TFPReportUnits;
   i: Integer;
   c: TFPReportElement;

begin
  OldH:=RTLayout.Height;
  inherited ApplyStretchMode(ADesiredHeight);
  H:=RTLayout.Height;
  // If the height changed, recalc for smMaxheight.
  if (H<>OldH) then
    for i := 0 to ChildCount-1 do
      begin
      c := Child[i];
      if (c.StretchMode = smMaxHeight) then
        C.RecalcLayout;
      end;
end;

destructor TFPReportElementWithChildren.Destroy;
var
  i: integer;
begin
  if Assigned(FChildren) then
    begin
    for i := 0 to FChildren.Count - 1 do
      begin
      Child[i].FParent := nil;
      Child[i].Free;
      end;
    FreeAndNil(FChildren);
    end;
  inherited Destroy;
end;

procedure TFPReportElementWithChildren.Validate(aErrors: TStrings);

Var
  I : Integer;

begin
  inherited Validate(aErrors);
  For I:=0 to ChildCount-1 do
    Child[i].Validate(aErrors);
end;

procedure TFPReportElementWithChildren.StartDesigning;

Var
  I : Integer;

begin
  inherited StartDesigning;
  For I:=0 to ChildCount-1 do
    Child[I].StartDesigning;
end;

procedure TFPReportElementWithChildren.EndDesigning;

Var
  I : Integer;

begin
  inherited EndDesigning;
  For I:=0 to ChildCount-1 do
    Child[I].EndDesigning;
end;

procedure TFPReportElementWithChildren.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
var
  i: integer;
begin
  inherited WriteElement(AWriter, AOriginal);
  if Not Assigned(FChildren) then
     exit;
  AWriter.PushElement('Children');
  try
    for i := 0 to FChildren.Count-1 do
    begin
      AWriter.PushElement(IntToStr(i)); // use child index as identifier
      try
        AWriter.PushElement(Child[I].ElementType);
        try
          Child[i].WriteElement(AWriter, AOriginal);
        finally
          AWriter.PopElement;
        end;
      finally
        AWriter.PopElement;
      end;
    end;
  finally
    AWriter.PopElement;
  end;
end;

procedure TFPReportElementWithChildren.ReadElement(AReader: TFPReportStreamer);
var
  E: TObject;
  i: integer;
  c: TFPReportElement;
  lName: string;
  o : TComponent;

begin
  inherited ReadElement(AReader);
  E := AReader.FindChild('Children');
  O:=Report;
  if (O=Nil) then
    O:=Self.Owner;
  if Not Assigned(E) then
    exit;
  AReader.PushElement(E);
  for i := 0 to AReader.ChildCount-1 do
  begin
    E := AReader.GetChild(i);
    AReader.PushElement(E); // child index is the identifier
    try
      lName := AReader.CurrentElementName;
      c := gElementFactory.CreateInstance(lName,O);
      c.Parent:=Self;
      AReader.PushElement(Areader.GetChild(0));
      try
        c.ReadElement(AReader);
      Finally
        AReader.PopElement;
      end;
    finally
      AReader.PopElement;
    end;
  end;  { for i }
  AReader.PopElement;
end;

function TFPReportElementWithChildren.Equals(AElement: TFPReportElement): boolean;
var
  lRE: TFPReportElementWithChildren;
  i: integer;
begin
  if not (AElement is TFPReportElementWithChildren) then
    ReportError(SErrIncorrectDescendant);
  lRE := TFPReportElementWithChildren(AElement);
  Result := inherited Equals(lRE);
  if Result then
  begin
    Result := ChildCount = lRe.ChildCount;
    if Result then
    begin
      for i := 0 to ChildCount-1 do
      begin
        Result := self.Child[i].Equals(lRE.Child[i]);
        if not Result then
          Break;
      end;
    end;
  end;
end;

{ TFPReportCustomPage }

procedure TFPReportCustomPage.SetReport(const AValue: TFPCustomReport);
begin
  if FReport = AValue then
    Exit;
  if Assigned(FReport) then
  begin
    FReport.RemoveFreeNotification(self);
    FReport.RemovePage(Self);
  end;
  FReport := AValue;
  if Assigned(FReport) then
  begin
    FReport.AddPage(Self);
    FReport.FreeNotification(Self);
  end;
end;

procedure TFPReportCustomPage.SetReportData(const AValue: TFPReportData);
begin
  if FData = AValue then
    Exit;
  if Assigned(FData) then
    FData.RemoveFreeNotification(Self);
  FData := AValue;
  if Assigned(FData) then
    FData.FreeNotification(Self);
end;

procedure TFPReportCustomPage.SetColumnLayout(AValue: TFPReportColumnLayout);
begin
  if FColumnLayout = AValue then
    Exit;
  FColumnLayout := AValue;
end;

procedure TFPReportCustomPage.SetColumnCount(AValue: Byte);
begin
  if FColumnCount = AValue then
    Exit;
  if AValue < 1 then
    FColumnCount := 1
  else
    FColumnCount := AValue;
  RecalcLayout;
end;

procedure TFPReportCustomPage.SetColumnGap(AValue: TFPReportUnits);
begin
  if FColumnGap = AValue then
    Exit;
  if AValue < 0 then
    FColumnGap := 0
  else
    FColumnGap := AValue;
  RecalcLayout;
end;

function TFPReportCustomPage.GetReportPage: TFPReportCustomPage;
begin
  Result := nil;
end;

function TFPReportCustomPage.GetReportBand: TFPReportCustomBand;
begin
  Result := nil;
end;

procedure TFPReportCustomPage.SaveDataToNames;
begin
  inherited ;
  If Assigned(Data) then
    FDataName:=Data.Name;
end;

procedure TFPReportCustomPage.RestoreDataFromNames;
begin
  Inherited;
  if FDataName<>'' then
    Data:=Report.ReportData.FindReportData(FDataName)
  else
    Data:=Nil;
end;

procedure TFPReportCustomPage.RemoveChild(const AChild: TFPReportElement);
begin
  inherited RemoveChild(AChild);
  if (AChild is TFPReportCustomBand) and Assigned(FBands) then
    FBands.Remove(TFPReportCustomBand(AChild));
end;

procedure TFPReportCustomPage.AddChild(const AChild: TFPReportElement);
var
  lBand: TFPReportCustomBand;
begin
  inherited AddChild(AChild);
  if (AChild is TFPReportCustomBand) then
    begin
    lBand := TFPReportCustomBand(AChild);
    FBands.Add(lBand);
    ApplyBandWidth(lBand);
    if (AChild is TFPReportCustomBandWithData) then
      TFPReportCustomBandWithData(AChild).Data := self.Data;
    end;
end;

procedure TFPReportCustomPage.RecalcLayout;
var
  W, H: TFPReportunits;
  b: integer;
begin
  if (Pagesize.Width = 0) and (Pagesize.Height = 0) then
    Exit;
  case Orientation of
    poPortrait:
    begin
      W := PageSize.Width;
      H := PageSize.Height;
    end;
    poLandscape:
    begin
      W := PageSize.Height;
      H := PageSize.Width;
    end;
  end;
  BeginUpdate;
  try
    Layout.Left := Margins.Left;
    Layout.Width := W - Margins.Left - Margins.Right;
    Layout.Top := Margins.Top;
    Layout.Height := H - Margins.Top - Margins.Bottom;

    for b := 0 to BandCount-1 do
      ApplyBandWidth(Bands[b]);
  finally
    EndUpdate;
  end;
end;

procedure TFPReportCustomPage.CalcPrintPosition;
var
  i: integer;
  b: TFPReportCustomBand;
  ly: TFPReportUnits;
begin
  if not Assigned(RTLayout) then
    exit;
  ly := Layout.Top;
  for i := 0 to BandCount - 1 do
  begin
    b := Bands[i];
    b.RTLayout.Top := ly;
    ly := ly + b.Layout.Height;
  end;
end;

function TFPReportCustomPage.PrepareObject(aRTParent: TFPReportElement): TFPReportElement;
begin
  Result := TFPReportCustomPage.Create(nil);
  Result.Assign(self);
  Result.CreateRTLayout;
  PrepareObjects(aRTParent);
end;

procedure TFPReportCustomPage.PrepareObjects(aRTParent: TFPReportElement);
begin
  // inherited PrepareObjects(aRTParent);
end;

procedure TFPReportCustomPage.MarginsChanged;
begin
  RecalcLayout;
end;

procedure TFPReportCustomPage.PageSizeChanged;
begin
  RecalcLayout;
  If Assigned(FOnPageSizeChange) then
    FOnPageSizeChange(Self);
end;

constructor TFPReportCustomPage.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FMargins := TFPReportMargins.Create(Self);
  FPageSize := TFPReportPageSize.Create(Self);
  if AOwner is TFPCustomReport then
    Report := AOwner as TFPCustomReport;
  FColumnCount := 1;
  FColumnLayout := clVertical;
  FFont := TFPReportFont.Create;
  FFont.OnChanged:=@HandleFontChange;
  FBands:=TBandList.Create;
end;

destructor TFPReportCustomPage.Destroy;
begin
  Report := nil;
  FreeAndNil(FMargins);
  FreeAndNil(FPageSize);
  FreeAndNil(FBands);
  FreeAndNil(FFont);
  inherited Destroy;
end;

class function TFPReportCustomPage.ElementType: String;
begin
  Result:='Page';
end;

procedure TFPReportCustomPage.Assign(Source: TPersistent);
var
  E: TFPReportCustomPage;
begin
  if (Source is TFPReportCustomPage) then
  begin
    E := Source as TFPReportCustomPage;
    PageSize.Assign(E.PageSize);
    Orientation := E.Orientation;
    Report := E.Report;
    Font.Assign(E.Font);
    ColumnCount := E.ColumnCount;
    ColumnGap := E.ColumnGap;
  end;
  inherited Assign(Source);
end;

procedure TFPReportCustomPage.ReadElement(AReader: TFPReportStreamer);
var
  E: TObject;
begin
  ColumnCount := AReader.ReadInteger('ColumnCount', 1);
  ColumnGap := AReader.ReadFloat('ColumnGap', 0);
  ColumnLayout := StringToColumnLayout(AReader.ReadString('ColumnLayout', 'clVertical'));
  Orientation := StringToPaperOrientation(AReader.ReadString('Orientation', 'poPortrait'));
  Pagesize.PaperName := AReader.ReadString('PageSize.PaperName', 'A4');
  Pagesize.Width := AReader.ReadFloat('PageSize.Width', 210);
  Pagesize.Height := AReader.ReadFloat('PageSize.Height', 297);
  Font.Name := AReader.ReadString('FontName', Font.Name);
  Font.Size := AReader.ReadInteger('FontSize', Font.Size);
  Font.Color := QWordToReportColor(AReader.ReadQWord('FontColor', Font.Color));
  FDataName:=AReader.ReadString('Data','');
  if FDataName<>'' then
    RestoreDataFromNames;
  E := AReader.FindChild('Margins');
  if Assigned(E) then
  begin
    AReader.PushElement(E);
    try
      FMargins.ReadElement(AReader);
    finally
      AReader.PopElement;
    end;
  end;
  inherited ReadElement(AReader);
end;

function TFPReportCustomPage.CheckBandMultiplicity(aBand: TFPReportCustomBand): Boolean;

Var
  D : TFPReportData;

begin
  if aBand is TFPReportCustomBandWithData then
    D:=TFPReportCustomBandWithData(aBand).GetData
  else
    D:=Nil;
  Result:=CheckBandMultiplicity(aBand.ReportBandType,D);
end;

function TFPReportCustomPage.CheckBandMultiplicity(aBandType: TFPReportBandType; aData: TFPReportData): Boolean;

Var
  M: TFPReportBandMultiplicity;

begin
  M:=FPReportBandMultiplicity[aBandType];
  Case M of
    bmUnrestricted : Result:=True;
    bmOncePerPage : Result:=FindBandWithType(aBandType)=Nil;
    bmOncePerDataLoop :
      begin
      Result:=aData=Nil;
      if not Result then
        Result:=FindBandWithTypeAndData(aBandType,aData)=Nil;
      end;
  end;
end;

function TFPReportCustomPage.FindBandWithType(ABandType: TFPReportBandType): TFPReportCustomBand;

var
  i: integer;

begin
  Result := nil;
  I:=0;
  While (Result=Nil) and (I<BandCount) do
    begin
    if Bands[i].ReportBandType=ABandType then
      Result := Bands[i];
    Inc(I);
    end;
end;

function TFPReportCustomPage.FindBandWithTypeAndData(ABandType: TFPReportBandType; aData: TFPReportData): TFPReportCustomBand;
var
  i: integer;

begin
  Result := nil;
  I:=0;
  While (Result=Nil) and (I<BandCount) do
    begin
    if (Bands[i].ReportBandType=ABandType) then
      if Bands[i] is TFPReportCustomBandWithData then
        if TFPReportCustomBandWithData(Bands[i]).GetData=aData then
          Result := Bands[i];
    Inc(I);
    end;
end;

function TFPReportCustomPage.FindBand(ABand: TFPReportBandClass): TFPReportCustomBand;

var
  i: integer;

begin
  Result := nil;
  I:=0;
  While (Result=Nil) and (I<BandCount) do
    begin
    if Bands[i] is ABand then
      Result := Bands[i];
    Inc(I);
    end;
end;

function TFPReportCustomPage.FindMainDataloop: TFPReportData;

Var
  I : Integer;
  B : TFPReportCustomDataBand;

begin
  Result:=Nil;
  I:=0;
  While (Result=Nil) and (I<BandCount) do
    begin
    if Bands[i] is TFPReportCustomDataBand then
      begin
      B:=TFPReportCustomDataBand(Bands[i]);
      if (B.MasterBand=Nil) then
        Result:=B.Data;
      end;
    Inc(I);
    end;
end;

procedure TFPReportCustomPage.Notification(AComponent: TComponent; Operation: TOperation);
begin
  if (Operation = opRemove) then
    if (AComponent = FReport) then
      FReport := nil
    else if (AComponent = FData) then
      FData := nil;
  inherited Notification(AComponent, Operation);
end;

procedure TFPReportCustomPage.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  AWriter.WriteFloat('ColumnCount', ColumnCount);
  AWriter.WriteFloat('ColumnGap', ColumnGap);
  AWriter.WriteString('ColumnLayout', ColumnLayoutToString(ColumnLayout));
  AWriter.WriteString('Orientation', PaperOrientationToString(Orientation));
  AWriter.WriteString('PageSize.PaperName', PageSize.PaperName);
  AWriter.WriteFloat('PageSize.Width', PageSize.Width);
  AWriter.WriteFloat('PageSize.Height', PageSize.Height);
  AWriter.WriteString('FontName', Font.Name);
  AWriter.WriteInteger('FontSize', Font.Size);
  AWriter.WriteQWord('FontColor', Font.Color);
  if Assigned(FData) then
    AWriter.WriteString('Data',FData.Name);
  AWriter.PushElement('Margins');
  try
    FMargins.WriteElement(AWriter, nil);
  finally
    AWriter.PopElement;
  end;
end;

function TFPReportCustomPage.GetPageIndex: Integer;
begin
  Result:=-1;
  If (Report<>Nil) then
    Result:=Report.IndexOfPage(Self);
end;

procedure TFPReportCustomPage.SetPageIndex(aIndex: Integer);

Var
  I : Integer;

begin
  I:=PageIndex;
  if Assigned(Report) then
    Report.FPages.Move(I,aIndex);
end;

function TFPReportCustomPage.GetBandCount: integer;
begin
  if Assigned(FBands) then
    Result := FBands.Count
  else
    Result := 0;
end;

function TFPReportCustomPage.BandWidthFromColumnCount: TFPReportUnits;
var
  lTotalColumnGap: TFPReportUnits;
begin
  if ColumnCount = 1 then
    Result := Layout.Width
  else
  begin
    if ColumnGap > 0.0 then
      lTotalColumnGap := ColumnGap * (ColumnCount-1)
    else
      lTotalColumnGap := 0.0;
    Result := (1 / ColumnCount) * (Layout.Width - lTotalColumnGap);
  end;
end;

procedure TFPReportCustomPage.ApplyBandWidth(ABand: TFPReportCustomBand);

var
  lBand: TFPReportCustomBand;

begin
  { handle child bands like main parent band }
  lBand := ABand.FMainBand;
  { set Band Width appropriately - certain bands are not affected by ColumnCount }
  if (lBand is TFPReportCustomTitleBand) or
  (lBand is TFPReportCustomSummaryBand) or
  (lBand is TFPReportCustomPageHeaderBand) or
  (lBand is TFPReportCustomPageFooterBand) then
  begin
    ABand.Layout.Width := Layout.Width;
    ABand.FIsColumnType := False;
  end
  else
  begin
    ABand.Layout.Width := BandWidthFromColumnCount;
    ABand.FIsColumnType := True;
  end;
end;

function TFPReportCustomPage.GetIsMultiColumn: Boolean;
begin
  Result := FColumnCount > 1;
end;

procedure TFPReportCustomPage.HandleFontChange(Sender: TObject);
begin
  NotifyFontChange;
  Changed;
end;

procedure TFPReportCustomPage.SetFont(AValue: TFPReportFont);
begin
  FFont.Assign(AValue);
end;

procedure TFPReportCustomPage.SetMargins(const AValue: TFPReportMargins);
begin
  if FMargins = AValue then
    Exit;
  FMargins.Assign(AValue);
end;

procedure TFPReportCustomPage.SetOrientation(const AValue: TFPReportPaperOrientation);
begin
  if FOrientation = AValue then
    Exit;
  FOrientation := AValue;
  RecalcLayout;
end;

procedure TFPReportCustomPage.SetPageSize(const AValue: TFPReportPageSize);

begin
  if FPageSize = AValue then
    Exit;
  FPageSize.Assign(AValue);
end;

{ TFPCustomReport }

function TFPCustomReport.IndexOfPage(aPage : TFPReportCustomPage) : Integer;

begin
  if Assigned(FPages) then
    Result:=FPages.IndexOf(aPage)
  else
    Result:=-1;
end;

function TFPCustomReport.GetPage(AIndex: integer): TFPReportCustomPage;
begin
  if Assigned(FPages) then
    Result := TFPReportCustomPage(FPages[AIndex])
  else
    ReportError(SErrInValidPageIndex, [AIndex]);
end;

function TFPCustomReport.GetPageCount: integer;
begin
  if Assigned(FPages) then
    Result := FPages.Count
  else
    Result := 0;
end;

function TFPCustomReport.GetRenderedPageCount: integer;
var
  i: integer;
begin
  Result := 0;
  for i := Low(FPerDesignerPageCount) to High(FPerDesignerPageCount) do
    inc(Result, FPerDesignerPageCount[i]);
end;

procedure TFPCustomReport.BuiltinExprRecNo(var Result: TFPExpressionResult; const Args: TExprParameterArray);

Var
  S : String;
  D : TFPReportData;

begin
  S:='';
  if Length(Args)=1 then
    S:=Args[0].ResString;
  if (S<>'') then
    D:=ReportData.FindReportData(S)
  else
    D:=FLoopData;
  if Assigned(D) then
    Result.ResInteger:=D.RecNo
  else
    Result.ResInteger:=-1;
end;

procedure TFPCustomReport.BuiltinGetPageNumber(var Result: TFPExpressionResult; const Args: TExprParameterArray);
begin
  Result.ResInteger := FPageNumber;
end;

procedure TFPCustomReport.BuiltinGetColumnNumber(
  var Result: TFPExpressionResult; const Args: TExprParameterArray);
begin
  Result.ResInteger := FColumnNumber;
end;

procedure TFPCustomReport.BuiltinGetPageNoPerDesignerPage(var Result: TFPExpressionResult;
  const Args: TExprParameterArray);
begin
  Result.ResInteger := FPageNumberPerDesignerPage;
end;

procedure TFPCustomReport.BuiltinGetPageCount(var Result: TFPExpressionResult; const Args: TExprParameterArray);

begin
  if UsePageCountMarker then
    Result.ResString := cPageCountMarker
  else
    Result.ResInteger := FPageCount;
end;

procedure TFPCustomReport.BuiltinGetInRepeatedGroupHeader(
  var Result: TFPExpressionResult; const Args: TExprParameterArray);
begin
  Result.ResultType:=rtBoolean;
  Result.ResBoolean := FRTInRepeatedGroupHeader;
end;

procedure TFPCustomReport.BuiltInGetInIntermediateGroupFooter(
  var Result: TFPExpressionResult; const Args: TExprParameterArray);
begin
  Result.ResultType:=rtBoolean;
  Result.ResBoolean := FRTInIntermediateGroupFooter;
end;

procedure TFPCustomReport.BuiltinGetIsOverflowed(
  var Result: TFPExpressionResult; const Args: TExprParameterArray);
begin
  Result.ResultType:=rtBoolean;
  Result.ResBoolean := FRTIsOverflowed;
end;

procedure TFPCustomReport.BuiltinGetIsGroupDetailsPrinted(
  var Result: TFPExpressionResult; const Args: TExprParameterArray);
begin
  Result.ResultType:=rtBoolean;
  Result.ResBoolean := FRTGroupDetailsPrinted;
end;


procedure TFPCustomReport.BuiltinGetFieldIsNull(var Result: TFPExpressionResult; const Args: TExprParameterArray);

Var
  DN,FN : String;
  I : Integer;
  RD : TFPReportData;

begin
  Result.ResultType:=rtBoolean;
  Result.ResBoolean := True;
  FN:=Args[0].ResString;
  DN:='';
  I:=Pos('.',FN);
  if I>0 then
    begin
    DN:=Copy(FN,1,I-1);
    Delete(FN,1,I);
    end;
  RD:=ReportData.FindReportData(DN);
  if (DN='') and (ReportData.Count=1) then
    RD:=ReportData[0].Data;
  if Assigned(RD) and (RD.IndexOfField(FN)<>-1) then
    Result.ResBoolean:=VarIsNull(RD.FieldValues[FN]);
end;


procedure TFPCustomReport.EmptyRTObjects;
begin
  while RTObjects.Count > 0 do
  begin

    TFPReportElement(RTObjects[0]).Free;
    RTObjects.Delete(0);
  end;
end;

procedure TFPCustomReport.ClearDataBandLastTextValues(ABand: TFPReportCustomBandWithData);
var
  i: integer;
  m: TFPReportCustomMemo;
begin
  for i := 0 to ABand.ChildCount-1 do
  begin
    if ABand.Child[i] is TFPReportCustomMemo then
    begin
      m := TFPReportCustomMemo(ABand.Child[i]);
      m.FLastText := '';
    end;
  end;
end;

procedure TFPCustomReport.InitAggregates(APage: TFPReportCustomPage; const AData: TFPReportData);
begin
  Variables.InitExpressionValues(aData,IsFirstPass);
end;

procedure TFPCustomReport.DoneAggregates(APage: TFPReportCustomPage; const AData: TFPReportData);
begin
  Variables.DoneExpressionValues(aData,IsFirstPass);
end;


procedure TFPCustomReport.UpdateAggregates(APage: TFPReportCustomPage; const AData: TFPReportData);

var
  i: integer;

begin
  Variables.UpdateExpressionValues(aData,IsFirstPass);
  for I := 0 to aPage.BandCount-1 do
    if (aPage.Bands[I] is TFPReportCustomBandWithData) then
      TFPReportCustomBandWithData(aPage.Bands[I]).ProcessAggregates(AData);
end;

procedure TFPCustomReport.ClearReferenceList;
begin
  if not Assigned(FReferenceList) then
    FReferenceList := TStringList.Create
  else
    FReferenceList.Clear;
end;

procedure TFPCustomReport.AddReference(AComponent: TFPReportComponent; const APropertyName, AValueName: string);
begin
  FReferenceList.AddObject(AComponent.Name+'.'+APropertyName+'='+AValueName,AComponent);
end;

procedure TFPCustomReport.FixupReferences;
var
  i,j: integer;
  PN,CN,VN : String;
  p: TFPReportComponent;
  c: TFPReportElement;
begin
  for i := 0 to FReferenceList.Count-1 do
    begin
    FReferenceList.GetNameValue(I,PN,VN);
    J:=Pos('.',PN);
    CN:=Copy(PN,1,J-1);
    Delete(PN,1,J);
    p:=TFPReportComponent(FReferenceList.Objects[i]);
    if P=Nil then
      p:=FindRecursive(CN);
    if not Assigned(p) then
      Continue; // failded to find the component
    c := FindRecursive(FReferenceList.ValueFromIndex[i]);
    P.FixupReference(PN,VN,C);
    end;
end;

procedure TFPCustomReport.DoBeforeRenderReport;
begin
  if Assigned(FOnBeforeRenderReport) then
    FOnBeforeRenderReport(self);
end;

procedure TFPCustomReport.DoAfterRenderReport;
begin
  if Assigned(FOnAfterRenderReport) then
    FOnAfterRenderReport(self);
end;

procedure TFPCustomReport.DoProcessTwoPass;
var
  p, b, m: integer; // page, band, memo
  i: integer;
  rpage: TFPReportCustomPage;
  rband: TFPReportCustomBand;
  rmemo: TFPReportCustomMemo;
  txtblk: TFPTextBlock;
begin
  for p := 0 to RTObjects.Count-1 do  // page
  begin
    rpage := TFPReportPage(RTObjects[p]);
    for b := 0 to rpage.BandCount-1 do
    begin
      rband := rpage.Bands[b];
      for m := 0 to rband.ChildCount-1 do // band
      begin
        if rband.Child[m] is TFPReportCustomMemo then // memo
        begin
          rmemo := TFPReportCustomMemo(rband.Child[m]);
          for i := 0 to rmemo.TextBlockList.Count-1 do
          begin
            txtblk := rmemo.TextBlockList[i];
            txtblk.Text := StringReplace(txtblk.Text, cPageCountMarker, IntToStr(FPageCount), [rfReplaceAll, rfIgnoreCase]);
          end;
        end;
      end; { m }
    end; { b }
  end;
end;

function TFPCustomReport.DoGetUsePrevValue: Boolean;
begin
  Result := FRTUsePrevVariableValues;
end;

procedure TFPCustomReport.SetReportData(AValue: TFPReportDataCollection);
begin
  if FReportData=AValue then Exit;
  FReportData.Assign(AValue);
end;

procedure TFPCustomReport.SetUsePageCountMarker(AValue: Boolean);
begin
  if FUsePageCountMarker=AValue then Exit;
  FUsePageCountMarker:=AValue;
end;

procedure TFPCustomReport.SetVariables(AValue: TFPReportVariables);
begin
  if FVariables=AValue then Exit;
  FVariables.Assign(AValue);
end;

procedure TFPCustomReport.RTBeginUsePrevVariableValues;
begin
  if FRTUsePrevVariableValuesCount = 0 then
    FRTUsePrevVariableValues := True;
  inc(FRTUsePrevVariableValuesCount);
end;

procedure TFPCustomReport.RTEndUsePrevVariableValues;
begin
  dec(FRTUsePrevVariableValuesCount);
  if FRTUsePrevVariableValuesCount = 0 then
    FRTUsePrevVariableValues := False;
end;

procedure TFPCustomReport.DoPrepareReport;

Var
  L : TFPReportLayouter;

begin
  FPageCount:=0;
  L:=CreateLayouter;
  try
    FDataAdded:=TFPList.Create;
    SetLength(FPerDesignerPageCount, PageCount);
    L.Execute(Self);
  finally
    SetLength(FPerDesignerPageCount, 0);
    FreeAndNil(FDataAdded);
    L.Free;
  end;
end;

procedure TFPCustomReport.DoBeginReport;
begin
  if Assigned(FOnBeginReport) then
    FOnBeginReport;
end;

procedure TFPCustomReport.DoEndReport;
begin
  if Assigned(FOnEndReport) then
    FOnEndReport;
end;

procedure TFPCustomReport.RestoreDefaultVariables;

Var
  I : Integer;

begin
  For I:=0 to FVariables.Count-1 do
    FVariables[i].RestoreValue;
end;

procedure TFPCustomReport.AddBuiltinsToExpressionIdentifiers(Idents : TFPExprIdentifierDefs);

begin
  Idents.AddDateTimeVariable('TODAY', Date);
  Idents.AddStringVariable('AUTHOR', Author);
  Idents.AddStringVariable('TITLE', Title);
  Idents.AddFunction('RecNo', 'I', 'S', @BuiltinExprRecNo);
  Idents.AddFunction('PageNo', 'I', '', @BuiltinGetPageNumber);
  Idents.AddFunction('ColNo', 'I', '', @BuiltinGetColumnNumber);
  Idents.AddFunction('PageNoPerDesignerPage', 'I', '', @BuiltInGetPageNoPerDesignerPage);
  Idents.AddFunction('InRepeatedGroupHeader', 'B', '', @BuiltInGetInRepeatedGroupHeader);
  Idents.AddFunction('InIntermediateGroupFooter', 'B', '', @BuiltInGetInIntermediateGroupFooter);
  Idents.AddFunction('IsOverflowed', 'B', '', @BuiltInGetIsOverflowed);
  Idents.AddFunction('IsGroupDetailPrinted', 'B', '', @BuiltinGetIsGroupDetailsPrinted);
  Idents.AddFunction('FieldIsNull', 'B', 'S', @BuiltinGetFieldIsNull);
end;

procedure TFPCustomReport.InitializeDefaultExpressions;

Var
  I : Integer;
  V : TFPReportVariable;
  lHasAggregates: Boolean;

begin
  FExpr.Clear;
  FExpr.Identifiers.Clear;
  FExpr.BuiltIns := [bcStrings,bcDateTime,bcMath,bcBoolean,bcConversion,bcData,bcVaria,bcUser, bcAggregate];
  AddBuiltInsToExpressionIdentifiers(FExpr.Identifiers);
  lHasAggregates:=false;
  For I:=0 to FVariables.Count-1 do
  begin
    V:=FVariables[i];
    V.SaveValue;
    if V.Expression = '' then
      FExpr.Identifiers.AddVariable(V.Name,V.DataType,@V.GetRTValue)
    else
      lHasAggregates:=true;
  end;
  if lHasAggregates then
    TwoPass:=true;
  if UsePageCountMarker then
    FExpr.Identifiers.AddFunction('PageCount', 'S', '', @BuiltinGetPageCount)
  else
    FExpr.Identifiers.AddFunction('PageCount', 'I', '', @BuiltinGetPageCount);
end;

procedure TFPCustomReport.InitializeAggregates(IsFirstPass : Boolean);

var
  i: Integer;
  v: TFPReportVariable;

begin
  For I:=0 to FVariables.Count-1 do
    begin
    v:=FVariables[I];
    if (v.Expression<>'') then
      v.InitializeExpression(FExpr,ReportData,IsFirstPass);
    end;
  if IsFirstPass then
    For I:=0 to FVariables.Count-1 do
      begin
      v:=FVariables[I];
      if v.Expression<>'' then
        FExpr.Identifiers.AddVariable(v.Name, v.DataType, @v.GetRTExpressionValue);
      end;
end;


Class function TFPCustomReport.ReportKindToResultType(const AType: TFPReportFieldKind): TResultType;
begin
  case AType of
    rfkString:      Result := rtString;
    rfkBoolean:     Result := rtBoolean;
    rfkInteger:     Result := rtInteger;
    rfkFloat:       Result := rtFloat;
    rfkCurrency:     Result := rtCurrency;
    rfkDateTime:    Result := rtDateTime;
    rfkStream:      Result := rtString; //  TODO:  What do we do here?????
  else
    Result := rtString;
  end;
end;

Function TFPCustomReport.StreamToReportElements(aStream: TStream): TFPObjectList;

Var
  I : Integer;
  S : TFPReportJSONStreamer;
  aName : String;
  E : TObject;
  C : TFPReportElement;

begin
  Result:=TFPObjectList.Create(True);
  try
    S:=TFPReportJSONStreamer.Create(Nil);
    try
      S.InitFromStream(aStream);
      for i := 0 to S.ChildCount-1 do
        begin
        E:=S.GetChild(i);
        S.PushElement(E); // child index is the identifier
        try
          aName := S.CurrentElementName;
          if aName='Page' then
            C:=TFPReportCustomPage.Create(Self)
          else
            c:=gElementFactory.CreateInstance(aName, Self);
          c.Parent:=Nil;
          c.ReadElement(S);
          Result.Add(C);
        finally
          S.PopElement;
        end;
        end;
    finally
      S.Free;
    end;
  except
     FreeAndNil(Result);
     Raise;
  end;
end;

procedure TFPCustomReport.InitializeExpressionVariables;

var
  i,j: Integer;
  f: string;
  r: TResultType;
  d: string;
  df: TFPReportDataField;
  aData : TFPReportData;

begin
  For J:=0 to ReportData.Count-1 do
    begin
    aData:=ReportData[J].Data;
    {$ifdef gdebug}
    writeln('********** TFPCustomReport.InitializeExpressionVariables');
    {$endif}
    F:='';
    For i:=0 to FExpr.Identifiers.Count-1 do
      f:=f+FExpr.Identifiers[i].Name+'; ';
    if FDataAdded.IndexOf(AData)=-1 then
      begin
      for i := 0 to AData.DataFields.Count-1 do
        begin
        d := AData.Name;
        df := AData.DataFields[i];
        f := df.FieldName;
        df.OnGetUsePrevValue := @DoGetUsePrevValue;
        r := ReportKindToResultType(df.FieldKind);
        if d <> '' then
          begin
          {$ifdef gdebug}
          writeln('registering (dotted name)... '+ d+'.'+f);
          {$endif}
          df.ExprIdentierDef := FExpr.Identifiers.AddVariable(d+'.'+f, r, @df.GetRTValue);
          end
        else
          begin
          {$ifdef gdebug}
          writeln('registering... '+ f);
          {$endif}
          df.ExprIdentierDef := FExpr.Identifiers.AddVariable(f, r, @df.GetRTValue);
          end;
        end;
      FDataAdded.Add(AData);
      end;
   end;
end;

procedure TFPCustomReport.CacheMemoExpressions(const APage: TFPReportCustomPage);

var
  b: integer;
  c: integer;
  m: TFPReportCustomMemo;

begin
  for b := 0 to aPage.BandCount-1 do
    for c := 0 to aPage.Bands[b].ChildCount-1 do
      if aPage.Bands[b].Child[c] is TFPReportCustomMemo then
        begin
        m := TFPReportCustomMemo(aPage.Bands[b].Child[c]);
        m.ParseText;
        end;
end;

constructor TFPCustomReport.Create(AOwner: TComponent);

begin
  inherited Create(AOwner);
  FReportData:=CreateReportData;
  FRTObjects := TFPList.Create;
  FImages := CreateImages;
  FVariables:=CreateVariables;
  FRTCurPageIdx := -1;
  FDateCreated := Now;
  FTwoPass := False;
  FIsFirstPass := False;
end;

function TFPCustomReport.CreateImages: TFPReportImages;

begin
  Result:=TFPReportImages.Create(self, TFPReportImageItem);
end;

function TFPCustomReport.CreateVariables: TFPReportVariables;

begin
  Result:=TFPReportVariables.Create(Self,TFPReportVariable);
end;

function TFPCustomReport.CreateReportData : TFPReportDataCollection;

begin
  Result:=TFPReportDataCollection.Create(TFPReportDataItem);
end;

function TFPCustomReport.CreateLayouter: TFPReportLayouter;
begin
  Result:=TFPReportLayouter.Create(Self);
end;

destructor TFPCustomReport.Destroy;
begin
  EmptyRTObjects;
  FreeAndNil(FReportData);
  FreeAndNil(FRTObjects);
  FreeAndNil(FPages);
  FreeAndNil(FExpr);
  FreeAndNil(FReferenceList);
  FreeAndNil(FImages);
  FreeAndNil(FVariables);
  inherited Destroy;
end;

procedure TFPCustomReport.Clear(ClearData : Boolean = True);

Var
  P : TFPReportCustomPage;

begin
  // Previous run objects
  EmptyRTObjects;
  // Variables
  FRTCurPageIdx := -1;
  FDateCreated := Now;
  FTwoPass := False;
  FIsFirstPass := False;
  // Collections
  FreeAndNil(FExpr); // Special case, recreated on run
  if ClearData then
    FReportData.Clear;
  if Assigned(FPages) then
    begin
    While PageCount>0 do
      begin
      P:=Pages[PageCount-1];
      RemovePage(P);
      FreeAndNil(P);
      end;
    FPages.Clear;
    end;
  ClearReferenceList;
  If Assigned(Fimages) then
    FImages.Clear;
  If Assigned(FVariables) then
    FVariables.Clear;
end;

procedure TFPCustomReport.SaveDataToNames;

Var
  I : Integer;

begin
  For I:=0 to PageCount-1 do
    Pages[i].SaveDataToNames;
end;

procedure TFPCustomReport.RestoreDataFromNames;
Var
  I : Integer;

begin
  For I:=0 to PageCount-1 do
    Pages[i].RestoreDataFromNames;
end;

procedure TFPCustomReport.AddPage(APage: TFPReportCustomPage);
begin
  if not Assigned(FPages) then
  begin
    FPages := TFPList.Create;
    FPages.Add(APage);
  end
  else if FPages.IndexOf(APage) = -1 then
    FPages.Add(APage);
end;

procedure TFPCustomReport.RemovePage(APage: TFPReportCustomPage);
begin
  if Assigned(FPages) then
    FPages.Remove(APage);
end;

procedure TFPCustomReport.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
var
  i: integer;
begin
  // ignore AOriginal here as we don't support whole report diffs, only element diffs
  AWriter.PushElement('Report');
  try
    inherited WriteElement(AWriter, AOriginal);
    // local properties
    AWriter.WriteString('Title', Title);
    AWriter.WriteString('Author', Author);
    AWriter.WriteBoolean('TwoPass',TwoPass);
    AWriter.WriteDateTime('DateCreated', DateCreated);
    // now the design-time images
    AWriter.PushElement('Images');
    try
      for i := 0 to Images.Count-1 do
      begin
        AWriter.PushElement(IntToStr(i)); // use image index as identifier
        try
          Images[i].WriteElement(AWriter);
        finally
          AWriter.PopElement;
        end;
      end;
    finally
      AWriter.PopElement;
    end;
    // now the pages
    AWriter.PushElement('Pages');
    try
      for i := 0 to PageCount - 1 do
      begin
        AWriter.PushElement(IntToStr(i)); // use page index as identifier
        try
          Pages[i].WriteElement(AWriter);
        finally
          AWriter.PopElement;
        end;
      end;
    finally
      AWriter.PopElement;
    end;
    AWriter.PushElement('Variables');
    try
      for i := 0 to Variables.Count - 1 do
        begin
          AWriter.PushElement(IntToStr(i)); // use variable index as identifier
          try
            Variables[i].WriteElement(AWriter);
          finally
            AWriter.PopElement;
          end;
        end;
    finally
      AWriter.PopElement;
    end;
  finally
    AWriter.PopElement;
  end;
  // TODO: Implement writing OnRenderReport, OnBeginReport, OnEndReport
end;

procedure TFPCustomReport.WriteRTElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
var
  i: integer;
begin
  // ignore AOriginal here as we don't support whole report diffs, only element diffs
  AWriter.PushElement('Report');
  try
    inherited WriteElement(AWriter, AOriginal);
    // local properties
    AWriter.WriteString('Title', Title);
    AWriter.WriteString('Author', Author);
    AWriter.WriteBoolean('TwoPass',TwoPass);
    AWriter.WriteDateTime('DateCreated', DateCreated);
    // now the pages
    AWriter.PushElement('Pages');
    try
      for i := 0 to RTObjects.Count - 1 do
      begin
        AWriter.PushElement(IntToStr(i)); // use page index as identifier
        try
          TFPReportComponent(RTObjects[i]).WriteElement(AWriter);
        finally
          AWriter.PopElement;
        end;
      end;
    finally
      AWriter.PopElement;
    end;
  finally
    AWriter.PopElement;
  end;
end;

procedure TFPCustomReport.ReadElement(AReader: TFPReportStreamer);
var
  E: TObject;
  i: integer;
  p: TFPReportPage;
  v : TFPReportVariable;
  lImgItem: TFPReportImageItem;
begin
  ClearReferenceList;
  E := AReader.FindChild('Report');
  if Assigned(E) then
  begin
    AReader.PushElement(E);
    try
      inherited ReadElement(AReader);
      FTitle := AReader.ReadString('Title', Title);
      FAuthor := AReader.ReadString('Author', Author);
      FTwoPass := AReader.ReadBoolean('TwoPass',TwoPass);
      FDateCreated := AReader.ReadDateTime('DateCreated', Now);

      E := AReader.FindChild('Images');
      if Assigned(E) then
      begin
        AReader.PushElement(E);
        for i := 0 to AReader.ChildCount-1 do
        begin
          E := AReader.GetChild(i);
          AReader.PushElement(E); // child index is the identifier
          try
            lImgItem := Images.AddImageItem;
            lImgItem.ReadElement(AReader);
          finally
            AReader.PopElement;
          end;
        end; { for i }
        AReader.PopElement;
      end;  { images }

      E := AReader.FindChild('Pages');
      if Assigned(E) then
      begin
        AReader.PushElement(E);
        for i := 0 to AReader.ChildCount-1 do
        begin
          E := AReader.GetChild(i);
          AReader.PushElement(E); // child index is the identifier
          try
            p := TFPReportPage.Create(self);
            p.ReadElement(AReader);
            AddPage(p);
          finally
            AReader.PopElement;
          end;
        end;  { for i }
        AReader.PopElement;
      end; { pages }
      E := AReader.FindChild('Variables');
      if Assigned(E) then
      begin
        Variables.Clear;
        AReader.PushElement(E);
        for i := 0 to AReader.ChildCount-1 do
        begin
          E := AReader.GetChild(i);
          AReader.PushElement(E); // child index is the identifier
          try
            v := Variables.Add as TFPReportVariable;
            v.ReadElement(AReader);
          finally
            AReader.PopElement;
          end;
        end;  { for I }
        AReader.PopElement;
      end; { Variables }

      // TODO: Implement reading OnRenderReport, OnBeginReport, OnEndReport
    finally
      AReader.PopElement;
    end;
  end;
  FixupReferences;
end;

procedure TFPCustomReport.StartRender;
begin
  inherited StartRender;
  DoBeforeRenderReport;
end;

procedure TFPCustomReport.EndRender;
begin
  inherited EndRender;
  DoAfterRenderReport;
end;

procedure TFPCustomReport.StartDesigning;

Var
  I : Integer;

begin
  SetDesigning(True,True);
  For I:=0 to PageCount-1 do
    Pages[i].StartDesigning;
end;

procedure TFPCustomReport.EndDesigning;

Var
  I : Integer;

begin
  SetDesigning(False,True);
  For I:=0 to PageCount-1 do
    Pages[i].EndDesigning;
end;

function TFPCustomReport.FindRecursive(const AName: string): TFPReportElement;
var
  p, b, c: integer;
begin
  Result := nil;
  if AName = '' then
    Exit;
  for p := 0 to PageCount-1 do
  begin
    for b := 0 to Pages[p].BandCount-1 do
    begin
      if SameText(Pages[p].Bands[b].Name, AName) then
        Result := Pages[p].Bands[b];
      if Assigned(Result) then
        Exit;

      for c := 0 to Pages[p].Bands[b].ChildCount-1 do
      begin
        if SameText(Pages[p].Bands[b].Child[c].Name, AName) then
          Result := Pages[p].Bands[b].Child[c];
        if Assigned(Result) then
          Exit;
      end;
    end;
  end;
end;

procedure TFPCustomReport.Validate(aErrors : TStrings);

Var
  I : Integer;

begin
  if PageCount=0 then
    aErrors.Add(SErrNeedPages);
  For I:=0 to PageCount-1 do
    Pages[i].Validate(aErrors);
end;

procedure TFPCustomReport.Validate;

Var
  Errs : TStrings;

begin
  Errs:=TStringList.Create;
  try
    Validate(Errs);
    if Errs.Count>0 then
      Raise EInvalidReportError.CreateFmt(SErrInvalidReport,[Errs.Count,Errs.Text]);
  Finally
    Errs.Free;
  end;
end;

procedure TFPCustomReport.CollectReportData;

  Procedure CheckData(D : TFPReportData);

  begin
    if (D<>Nil) and (ReportData.FindReportDataItem(D)=Nil) then
      ReportData.AddReportData(D);
  end;

Var
  I,J : integer;
  P : TFPReportCustomPage;
  B : TFPReportCustomBandWithData;

begin
  For i:=0 to PageCount-1 do
    begin
    P:=Pages[i];
    CheckData(P.Data);
    For J:=0 to P.BandCount-1 do
      if (P.Bands[J] is TFPReportCustomBandWithData) then
        begin
        B:=TFPReportCustomBandWithData(P.Bands[J]);
        CheckData(B.Data);
        end;
    end;
end;

procedure TFPCustomReport.RunReport;
begin
  DoBeginReport;
  ClearPreparedReport;
  StartLayout;
  CollectReportData;
  Validate;
  FExpr := TFPexpressionParser.Create(nil);
  try
    InitializeDefaultExpressions;
    DoPrepareReport;
  finally
    RestoreDefaultVariables;
    FreeAndNil(FExpr);
  end;
  EndLayout;
  DoEndReport;
end;

procedure TFPCustomReport.ClearPreparedReport;
begin
  EmptyRTObjects;
  FVariables.ReleaseExpressionNodes;
end;

function TFPCustomReport.Prepared: Boolean;
begin
  Result:=RTObjects.Count>0;
end;

procedure TFPCustomReport.RenderReport(const AExporter: TFPReportExporter);
begin
  if not Assigned(AExporter) then
    Exit;
  StartRender;
  try
    AExporter.Report := self;
    AExporter.Execute;
  finally
    EndRender;
  end;
end;

{$IFDEF gdebug}
function TFPCustomReport.DebugPreparedPageAsJSON(const APageNo: Byte): string;
var
  rs: TFPReportStreamer;
begin
  if APageNo > RTObjects.Count-1 then
    Exit;
  rs := TFPReportJSONStreamer.Create(nil);
  try
    TFPReportCustomPage(RTObjects[APageNo]).WriteElement(rs);
    Result := TFPReportJSONStreamer(rs).JSON.FormatJSON;
  finally
    rs.Free;
  end;
end;
{$ENDIF}

{ TFPReportMargins }

procedure TFPReportMargins.SetBottom(const AValue: TFPReportUnits);
begin
  if FBottom = AValue then
    Exit;
  FBottom := AValue;
  Changed;
end;

procedure TFPReportMargins.SetLeft(const AValue: TFPReportUnits);
begin
  if FLeft = AValue then
    Exit;
  FLeft := AValue;
  Changed;
end;

procedure TFPReportMargins.SetRight(const AValue: TFPReportUnits);
begin
  if FRight = AValue then
    Exit;
  FRight := AValue;
  Changed;
end;

procedure TFPReportMargins.SetTop(const AValue: TFPReportUnits);
begin
  if FTop = AValue then
    Exit;
  FTop := AValue;
  Changed;
end;

procedure TFPReportMargins.Changed;
begin
  if Assigned(FPage) then
    FPage.MarginsChanged;
end;

constructor TFPReportMargins.Create(APage: TFPReportCustomPage);
begin
  inherited Create;
  FPage := APage;
end;

procedure TFPReportMargins.Assign(Source: TPersistent);
var
  S: TFPReportMargins;
begin
  if Source is TFPReportMargins then
  begin
    S := Source as TFPReportMargins;
    FTop := S.Top;
    FBottom := S.Bottom;
    FLeft := S.Left;
    FRight := S.Right;
    Changed;
  end
  else
    inherited Assign(Source);
end;

function TFPReportMargins.Equals(AMargins: TFPReportMargins): boolean;
begin
  Result := (AMargins = Self)
    or ((Top = AMargins.Top) and (Left = AMargins.Left) and
        (Right = AMargins.Right) and (Bottom = AMargins.Bottom));
end;

procedure TFPReportMargins.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportMargins);
begin
  if (AOriginal = nil) then
  begin
    AWriter.WriteFloat('Top', Top);
    AWriter.WriteFloat('Left', Left);
    AWriter.WriteFloat('Bottom', Bottom);
    AWriter.WriteFloat('Right', Right);
  end
  else
  begin
    AWriter.WriteFloatDiff('Top', Top, AOriginal.Top);
    AWriter.WriteFloatDiff('Left', Left, AOriginal.Left);
    AWriter.WriteFloatDiff('Bottom', Bottom, AOriginal.Bottom);
    AWriter.WriteFloatDiff('Right', Right, AOriginal.Right);
  end;
end;

procedure TFPReportMargins.ReadElement(AReader: TFPReportStreamer);
begin
  Top := AReader.ReadFloat('Top', Top);
  Left := AReader.ReadFloat('Left', Left);
  Bottom := AReader.ReadFloat('Bottom', Bottom);
  Right := AReader.ReadFloat('Right', Right);
end;

{ TFPReportCustomBand }

function TFPReportCustomBand.GetReportPage: TFPReportCustomPage;
begin
  Result := Parent as TFPReportCustomPage;
end;

function TFPReportCustomBand.GetParentFont: TFPReportFont;
begin
  If Assigned(Page) then
    Result:=Page.Font
  else
    Result:=Nil;
end;

procedure TFPReportCustomBand.HandleFontChange(Sender: TObject);
begin
  FUseParentFont:=False;
end;

Class function TFPCustomReport.IsStringValueZero(const AValue: string): boolean;
var
  lIntVal: integer;
  lFloatVal: double;
begin
  Result := False;
  if TryStrToInt(AValue, lIntVal) then
  begin
    if lIntVal = 0 then
      Result := True;
  end
  else if TryStrToFloat(AValue, lFloatVal) then
  begin
    if lFloatVal = 0 then
      Result := True;
  end;
end;

procedure TFPReportCustomBand.SetBandPosition(
  pBandPosition: TFPReportBandPosition);
begin
  if FBandPosition = pBandPosition then Exit;
  FBandPosition := pBandPosition;
end;

procedure TFPReportCustomBand.SetChildBand(AValue: TFPReportCustomChildBand);
var
  b: TFPReportCustomBand;
begin
  if FChildBand = AValue then
    Exit;
  if Assigned(FChildBand) then
    FChildBand.RemoveFreeNotification(Self);
  b := aValue;
  while b <> nil do
    begin
    b:=b.ChildBand;
    if b=self then
      raise EReportError.Create(SErrChildBandCircularReference);
    end;
  FChildBand := AValue;
  if Assigned(FChildBand) then
    begin
    FChildBand.RemoveFreeNotification(Self);
    FChildBand.FParentBand := Self;
    FChildBand.FMainBand := FMainBand;
    Page.ApplyBandWidth(FChildBand);
    end;
end;

Function TFPReportCustomBand.CalcDesiredHeight : TFPReportUnits;

var
  R,H: TFPReportUnits;
  c: TFPReportElement;
  i: integer;

begin
  R := 0;
  for i := 0 to ChildCount-1 do
    begin
    c:=Child[i];
    h:=c.RTLayout.Top + c.RTLayout.Height;
    if H>R then
      R:=H;
    end;
  Result:=R;
end;

procedure TFPReportCustomBand.SetFont(AValue: TFPReportFont);
begin
  UseParentFont:=False;
  FFont.Assign(AValue);
  Changed;
end;

procedure TFPReportCustomBand.SetKeepTogetherWithChildren(
  pKeepTogetherWithChildren: Boolean);
begin
  if FKeepTogetherWithChildren = pKeepTogetherWithChildren then Exit;
  FKeepTogetherWithChildren := pKeepTogetherWithChildren;
end;

procedure TFPReportCustomBand.SetMainBand(AValue: TFPReportCustomBand);
begin
  if FMainBand=AValue then Exit;
  if Assigned(FMainBand) then
    FMainBand.RemoveFreeNotification(Self);
  FMainBand:=AValue;
  if Assigned(FMainBand) then
    FMainBand.FreeNotification(Self);
end;

procedure TFPReportCustomBand.SetUseParentFont(AValue: boolean);

begin
  if FUseParentFont = AValue then
    Exit;
  FUseParentFont := AValue;
  if AValue then
    ReassignParentFont;
  Changed;
end;

procedure TFPReportCustomBand.SetVisibleOnPage(AValue: TFPReportVisibleOnPage);
begin
  if FVisibleOnPage = AValue then
    Exit;
  FVisibleOnPage := AValue;
  Changed;
end;

procedure TFPReportCustomBand.ParentFontChanged;
begin
  inherited ParentFontChanged;
  if UseParentFont then
    begin
    ReassignParentFont;
    NotifyFontChange;
    end;
end;

class function TFPReportCustomBand.ElementType: String;
begin
  Result := 'FPCustomReportBand';
end;

function TFPReportCustomBand.GetData: TFPReportData;
begin
  result := nil;
end;

procedure TFPReportCustomBand.SetDataFromName(AName: String);
begin
  // Do nothing
end;

procedure TFPReportCustomBand.SetParent(const AValue: TFPReportElement);
begin
  if not ((AValue = nil) or (AValue is TFPReportCustomPage)) then
    ReportError(SErrNotAReportPage, [AValue.ClassName, AValue.Name]);
  inherited SetParent(AValue);
  if UseParentFont then
    ReassignParentFont;
end;

procedure TFPReportCustomBand.ReassignParentFont;

Var
  F : TFPReportFont;
  B : Boolean;

begin
  B:=UseParentFont;
  try
  F:=GetParentFont;
  if Assigned(F) then
    FFont.Assign(F);
  Finally
    FUseParentFont:=B;
  end;
end;

procedure TFPReportCustomBand.CreateRTLayout;
begin
  inherited CreateRTLayout;
  FRTLayout.Left := Page.Layout.Left;
end;

procedure TFPReportCustomBand.Notification(AComponent: TComponent; Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if Operation=opRemove then
    begin
    if AComponent=FChildBand then
      FChildBand:=Nil
    else if AComponent=FMainband then
      FMainBand:=nil;
    end;
end;

function TFPReportCustomBand.PrepareObject(aRTParent: TFPReportElement): TFPReportElement;

var
  lBand: TFPReportCustomBand;

begin
  lBand := TFPReportBandClass(ClassType).Create(aRTParent);
  lBand.Assign(self);
  lBand.CreateRTLayout;
  Result := lBand;
  PrepareObjects(lBand);
end;

procedure TFPReportCustomBand.RecalcLayout;

Var
  I : Integer;

begin
  For I:=ChildCount-1 downto 0 do
    if Not Child[i].EvaluateVisibility then
      RemoveChild(Child[i]);
  inherited RecalcLayout;
  if StretchMode <> smDontStretch then
    ApplyStretchMode(CalcDesiredHeight);
end;

procedure TFPReportCustomBand.Assign(Source: TPersistent);
var
  E: TFPReportCustomBand;
begin
  inherited Assign(Source);
  if Source is TFPReportCustomBand then
  begin
    E := TFPReportCustomBand(Source);
    MainBand := E.MainBand;
    FIsColumnType := E.FIsColumnType;
    ChildBand := E.ChildBand;
    FStretchMode := E.StretchMode;
    FVisibleOnPage := E.VisibleOnPage;
    FBandPosition := E.BandPosition;
    Font.Assign(E.Font);
    UseParentFont := E.UseParentFont;
  end;
end;

procedure TFPReportCustomBand.FixupReference(const PN, PV: String; C: TFPReportElement);
begin
  if SameText(PN,'ChildBand') then
    begin
    if C is TFPReportCustomChildBand then
      ChildBand:=C as TFPReportCustomChildBand
    end
  else
    inherited FixupReference(PN, PV, C);
end;

procedure TFPReportCustomBand.SendToBack(El: TFPReportElement);

Var
  I : integer;

begin
  I:=FChildren.IndexOf(El);
  If I>0 then
    FChildren.Move(I,0);
end;

procedure TFPReportCustomBand.BringToFront(El: TFPReportElement);
Var
  I : integer;

begin
  I:=FChildren.IndexOf(El);
  If (I>=0) and (I<FChildren.Count-1) then
    FChildren.Move(I,FChildren.Count-1);
end;

class function TFPReportCustomBand.ReportBandType: TFPReportBandType;
begin
  Result:=btUnknown;
end;

procedure TFPReportCustomBand.BeforePrint;
var
  i: integer;
  c: TFPReportElement;
begin
  inherited BeforePrint;
  if Visible = false then
    exit;
  if Assigned(FChildren) then
  begin
    for i := 0 to FChildren.Count-1 do
    begin
      c := Child[i];
      c.BeforePrint;
    end;
  end;
end;

procedure TFPReportCustomBand.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  AWriter.WriteBoolean('UseParentFont', UseParentFont);
  AWriter.WriteString('BandPosition', BandPositionToString(FBandPosition));
  if not UseParentFont then
  begin
    AWriter.WriteString('FontName', Font.Name);
    AWriter.WriteInteger('FontSize', Font.Size);
    AWriter.WriteQWord('FontColor', Font.Color);
  end;
end;

procedure TFPReportCustomBand.BeginRuntimeProcessing;
begin
  // Do nothing
end;

procedure TFPReportCustomBand.EndRuntimeProcessing;
begin
  // Do nothing
end;

function TFPReportCustomBand.NeedsUpdateYPos: Boolean;
begin
  Result := True;
end;

procedure TFPReportCustomBand.AfterPrintBand(pBand: TFPReportCustomBand);
begin
  // Do nothing
end;

procedure TFPReportCustomBand.BeforePrintWithChilds;
begin
  // Do nothing
end;

procedure TFPReportCustomBand.MovedToNextPageWithChilds;
begin
  // Do nothing
end;

procedure TFPReportCustomBand.AfterPrintWithChilds;
begin
  // Do nothing
end;

constructor TFPReportCustomBand.Create(AOwner: TComponent);
begin
  FMainBand := Self;
  inherited Create(AOwner);
  FVisibleOnPage := vpAll;
  FUseParentFont := True;
  FKeepTogetherWithChildren := True;
  FBandPosition := bpNormal;
  FFont:=TFPReportFont.Create;
  FFont.OnChanged:=@HandleFontChange;
  ReassignParentFont;
end;

destructor TFPReportCustomBand.Destroy;
begin
  FreeAndNil(FFont);
  inherited Destroy;
end;

procedure TFPReportCustomBand.Validate(AErrors : TStrings);
begin
  if (Page=Nil) then
    aErrors.Add(Format(SErrNoPageForBand,[Name]));
end;

procedure TFPReportCustomBand.WriteElement(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  inherited WriteElement(AWriter, AOriginal);
  if Assigned(ChildBand) then
    AWriter.WriteString('ChildBand', ChildBand.Name);
  if Assigned(GetData) then
    AWriter.WriteString('Data', GetData.Name);
  AWriter.WriteString('VisibleOnPage', VisibleOnPageToString(FVisibleOnPage));
  AWriter.WriteBoolean('KeepTogetherWithChildren', FKeepTogetherWithChildren);
end;

procedure TFPReportCustomBand.ReadElement(AReader: TFPReportStreamer);

var
  E: TObject;
  s: string;

begin
  s := AReader.ReadString('ChildBand', '');
  if (s<>'') then
    Page.Report.AddReference(Self, 'ChildBand', s);
  FVisibleOnPage := StringToVisibleOnPage(AReader.ReadString('VisibleOnPage', 'vpAll'));
  FKeepTogetherWithChildren := AReader.ReadBoolean('KeepTogetherWithChildren', FKeepTogetherWithChildren);
  FBandPosition := StringToBandPosition(AReader.ReadString('BandPosition', 'bpNormal'));
  UseParentFont := AReader.ReadBoolean('UseParentFont', UseParentFont);
  if not UseParentFont then
    begin
    Font.Name := AReader.ReadString('FontName', Font.Name);
    Font.Size := AReader.ReadInteger('FontSize', Font.Size);
    Font.Color := QWordToReportColor(AReader.ReadQWord('FontColor', Font.Color));
    end
  else
    ReAssignParentFont;
  S:=AReader.ReadString('Data','');
  if (S<>'') then
    SetDataFromName(S);
  // This must come last: e.g. the UseParentFont assumes the font is properly set up
  inherited ReadElement(AReader);
end;

function TFPReportCustomBand.EvaluateVisibility: boolean;

begin
  Result := inherited EvaluateVisibility;
  if not Result then
    exit;
  Result := False;
  if FVisibleOnPage = vpAll then
  begin
    // do nothing special
  end
  else if (Report.FPageNumberPerDesignerPage = 1) then
  begin // first page rules
    if (FVisibleOnPage in [vpFirstOnly, vpFirstAndLastOnly]) then
    begin
      // do nothing special
    end
    else if (FVisibleOnPage in [vpNotOnFirst, vpLastOnly, vpNotOnFirstAndLast]) then
      Exit; // user asked to skip this band
  end
  else if (Report.FPageNumberPerDesignerPage > 1) then
  begin  // multi-page rules
    if FVisibleOnPage in [vpFirstOnly] then
      Exit  // user asked to skip this band
    else if FVisibleOnPage in [vpNotOnFirst] then
    begin
      // do nothing special
    end
    else if (not Report.IsFirstPass) then
    begin // last page rules
      if (FVisibleOnPage in [vpLastOnly, vpFirstAndLastOnly]) and (Report.FPageNumberPerDesignerPage < Report.FPerDesignerPageCount[Report.FRTCurDsgnPageIdx]) then
        Exit
      else if (FVisibleOnPage in [vpNotOnLast, vpFirstOnly, vpNotOnFirstAndLast]) and (Report.FPageNumberPerDesignerPage = Report.FPerDesignerPageCount[Report.FRTCurDsgnPageIdx]) then
        Exit; // user asked to skip this band
    end;
  end;
  Result := True;
end;

{ TFPReportCustomBandWithData }

procedure TFPReportCustomBandWithData.SetData(const AValue: TFPReportData);
begin
  if FData = AValue then
    Exit;
  if Assigned(FData) then
    FData.RemoveFreeNotification(Self);
  FData := AValue;
  if Assigned(FData) then
    FData.FreeNotification(Self);
end;

procedure TFPReportCustomBandWithData.ProcessAggregates(const AData: TFPReportData);

Var
  I : Integer;

begin
  if AData<>Data then
    exit;
  for I := 0 to ChildCount-1 do
    if Child[I] is TFPReportCustomMemo then
      TFPReportCustomMemo(Child[I]).UpdateAggregates;
end;


procedure TFPReportCustomBandWithData.SaveDataToNames;
begin
  inherited SaveDataToNames;
  if Assigned(FData) then
    FDataName:=FData.Name
  else
    FDataName:='';
end;

procedure TFPReportCustomBandWithData.ResolveDataName;

begin
  if (FDataName<>'') then
    Data:=Report.ReportData.FindReportData(FDataName)
  else
    Data:=Nil;
end;
procedure TFPReportCustomBandWithData.RestoreDataFromNames;

begin
  inherited RestoreDataFromNames;
  ResolveDataName;
end;

function TFPReportCustomBandWithData.GetData: TFPReportData;
begin
  Result := FData;
end;

procedure TFPReportCustomBandWithData.SetDataFromName(AName: String);
begin
  FDataName:=AName;
  ResolveDataName;
end;

procedure TFPReportCustomBandWithData.Notification(AComponent: TComponent; Operation: TOperation);
begin
  if Operation = opRemove then
  begin
    if AComponent = FData then
      FData := nil;
  end;
  inherited Notification(AComponent, Operation);
end;

constructor TFPReportCustomBandWithData.Create(AOwner: TComponent);
begin
  FData := nil;
  inherited Create(AOwner);
end;

procedure TFPReportCustomBandWithData.Validate(aErrors: TStrings);
begin
  inherited Validate(aErrors);
  if (Data=Nil) then
    aErrors.Add(Format('Band "%s" has no data assigned.',[Name]));
end;

{ TFPReportCustomGroupFooterBand }

procedure TFPReportCustomGroupFooterBand.SetGroupHeader(const AValue: TFPReportCustomGroupHeaderBand);
begin
  if FGroupHeader = AValue then
    Exit;
  if Assigned(FGroupHeader) then
    begin
    FGroupHeader.FGroupFooter := nil;
    FGroupHeader.RemoveFreeNotification(Self);
    end;
  FGroupHeader := AValue;
  if Assigned(FGroupHeader) then
    begin
    FGroupHeader.FGroupFooter := Self;
    FGroupHeader.FreeNotification(Self);
    Data:=FGroupHeader.Data;
    end;
end;

procedure TFPReportCustomGroupFooterBand.FixupReference(const PN, PV: String; C: TFPReportElement);
begin
  if SameText(PN,'Groupheader') then
    GroupHeader:=TFPReportCustomGroupHeaderBand(C)
  else
    inherited FixupReference(PN, PV, C);
end;

procedure TFPReportCustomGroupFooterBand.SetBandPosition(
  pBandPosition: TFPReportBandPosition);
begin
  inherited SetBandPosition(pBandPosition);
  if (FBandPosition = bpStackAtBottom) and
  (FGroupHeader.FStartOnNewSection = rsNone) then
    if Page.IsMultiColumn then
      FGroupHeader.FStartOnNewSection := rsColumn
    else
      FGroupHeader.FStartOnNewSection := rsPage;
end;

class function TFPReportCustomGroupFooterBand.ElementType: String;
begin
  Result := 'GroupFooterBand';
end;

procedure TFPReportCustomGroupFooterBand.DoWriteLocalProperties(AWriter: TFPReportStreamer; AOriginal: TFPReportElement);
begin
  inherited DoWriteLocalProperties(AWriter, AOriginal);
  if Assigned(GroupHeader) then
    AWriter.WriteString('GroupHeader', GroupHeader.Name);
end;

procedure TFPReportCustomGroupFooterBand.Notification(AComponent: TComponent; Operation: TOperation);
begin
  if (Operation = opRemove) and (AComponent = FGroupHeader) then
    FGroupHeader := nil;
  inherited Notification(AComponent, Operation);
end;

procedure TFPReportCustomGroupFooterBand.BeginRuntimeProcessing;
begin
  inherited BeginRuntimeProcessing;
  Report.RTBeginUsePrevVariableValues;
end;

procedure TFPReportCustomGroupFooterBand.EndRuntimeProcessing;
begin
  Report.RTEndUsePrevVariableValues;
  inherited EndRuntimeProcessing;
end;

function TFPReportCustomGroupFooterBand.NeedsUpdateYPos: Boolean;
begin
  Result := FBandPosition <> bpStackAtBottom;
end;

procedure TFPReportCustomGroupFooterBand.BeforePrintWithChilds;
begin
  inherited BeforePrintWithChilds;
  Report.FRTGroupDetailsPrinted := GroupHeader.FDetailsPrinted;
end;

procedure TFPReportCustomGroupFooterBand.AfterPrintWithChilds;
begin
  inherited AfterPrintWithChilds;
  if not Report.FRTInIntermediateGroupFooter then begin
    { if the footer is printed then it doesn't need a repeated
      group header if page break occurs due to no space        }
    GroupHeader.FNeedsReprintedHeader := False;
    { the old group is finished so next repeated group header
      does not need to use previous varaible values            }
    GroupHeader.FNeedsPrevVariables := False;
    { the old group is finished
      so an intermediate group footer is needed  }
    GroupHeader.FNeedsIntermediateFooter := False;

    Report.FRTGroupDetailsPrinted := False;
  end;
end;

destructor TFPReportCustomGroupFooterBand.Destroy;
begin
  GroupHeader:=Nil;
  inherited Destroy;
end;

procedure TFPReportCustomGroupFooterBand.ReadElement(AReader: TFPReportStreamer);
var
  s: string;
  c: TFPReportElement;
begin
  inherited ReadElement(AReader);
  s := AReader.ReadString('GroupHeader', '');
  if s<>'' then
    begin
    c:=Report.FindRecursive(S);
    if Not (C is TFPReportCustomGroupHeaderBand) then
      Report.AddReference(Self,'GroupHeader',S)
    else
      GroupHeader := TFPReportCustomGroupHeaderBand(c);
    end;
end;

class function TFPReportCustomGroupFooterBand.ReportBandType: TFPReportBandType;
begin
  Result:=btGroupFooter;
end;


{ TFPReportImageItem }

function TFPReportImageItem.GetHeight: Integer;
begin
  If Assigned(FImage) then
    Result:=FImage.Height
  else
    Result:=FHeight;
end;

function TFPReportImageItem.GetStreamed: TBytes;
begin
  if Length(FStreamed)=0 then
    CreateStreamedData;
  Result:=FStreamed;
end;

function TFPReportImageItem.GetWidth: Integer;
begin
  If Assigned(FImage) then
    Result:=FImage.Width
  else
    Result:=FWidth;
end;

procedure TFPReportImageItem.SetImage(AValue: TFPCustomImage);
begin
  if FImage=AValue then Exit;
  FImage:=AValue;
  SetLength(FStreamed,0);
end;

procedure TFPReportImageItem.SetStreamed(AValue: TBytes);
begin
  If AValue=FStreamed then exit;
  SetLength(FStreamed,0);
  FStreamed:=AValue;
end;

procedure TFPReportImageItem.LoadPNGFromStream(AStream: TStream);
var
  PNGReader: TFPReaderPNG;
begin
  if not Assigned(AStream) then
    Exit;

  { we use Image property here so it frees any previous image }
  if Assigned(FImage) then
    FreeAndNil(FImage);
  FImage := TFPCompactImgRGBA8Bit.Create(0, 0);
  try
    PNGReader := TFPReaderPNG.Create;
    try
      FImage.LoadFromStream(AStream, PNGReader); // auto size image
    finally
      PNGReader.Free;
    end;
  except
    FreeAndNil(FImage);
  end;
end;

constructor TFPReportImageItem.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FOwnsImage := True;
end;

destructor TFPReportImageItem.Destroy;
begin
  if FOwnsImage then
    FreeAndNil(FImage);
  inherited Destroy;
end;

procedure TFPReportImageItem.CreateStreamedData;
Var
  X, Y: Integer;
  C: TFPColor;
  MS: TMemoryStream;
  Str: TStream;
  CWhite: TFPColor; // white color
begin
  FillMem(@CWhite, SizeOf(CWhite), $FF);
  FWidth:=Image.Width;
  FHeight:=Image.Height;
  Str := nil;
  MS := TMemoryStream.Create;
  try
    Str := MS;
    for Y:=0 to FHeight-1 do
      for X:=0 to FWidth-1 do
        begin
        C:=Image.Colors[x,y];
        if C.alpha < $FFFF then // remove alpha channel - assume white background
          C := AlphaBlend(CWhite, C);

        Str.WriteByte(C.Red shr 8);
        Str.WriteByte(C.Green shr 8);
        Str.WriteByte(C.blue shr 8);
        end;
    if Str<>MS then
      Str.Free;
    Str := nil;
    SetLength(FStreamed, MS.Size);
    MS.Position := 0;
    if MS.Size>0 then
      MS.ReadBuffer(FStreamed[0], MS.Size);
  finally
    Str.Free;
    MS.Free;
  end;
end;

function TFPReportImageItem.WriteImageStream(AStream: TStream): UInt64;
var
  Img: TBytes;
begin
  Img := StreamedData;
  Result := Length(Img);
  AStream.WriteBuffer(Img[0],Result);
end;

function TFPReportImageItem.Equals(AImage: TFPCustomImage): boolean;
var
  x, y: Integer;
begin
  Result := True;
  for x := 0 to Image.Width-1 do
    for y := 0 to Image.Height-1 do
      if Image.Pixels[x, y] <> AImage.Pixels[x, y] then
      begin
        Result := False;
        Exit;
      end;
end;

procedure TFPReportImageItem.WriteElement(AWriter: TFPReportStreamer);
var
  ms: TMemoryStream;
  png: TFPWriterPNG;
begin
  if Assigned(Image) then
  begin
    ms := TMemoryStream.Create;
    try
      png := TFPWriterPNG.create;
      png.Indexed := False;
      Image.SaveToStream(ms, png);
      ms.Position := 0;
      AWriter.WriteStream('ImageData', ms);
    finally
      png.Free;
      ms.Free;
    end;
  end;
end;

procedure TFPReportImageItem.ReadElement(AReader: TFPReportStreamer);
var
  ms: TMemoryStream;
begin
  ms := TMemoryStream.Create;
  try
    if AReader.ReadStream('ImageData', ms) then
    begin
      ms.Position := 0;
      LoadPNGFromStream(ms);
    end;
  finally
    ms.Free;
  end;
end;

{ TFPReportImages }

function TFPReportImages.GetImg(AIndex: Integer): TFPReportImageItem;
begin
  Result := Items[AIndex] as TFPReportImageItem;
end;

function TFPReportImages.GetReportOwner: TFPCustomReport;
begin
  Result:=Owner as TFPCustomReport;
end;


constructor TFPReportImages.Create(AOwner: TFPCustomReport; AItemClass: TCollectionItemClass);
begin
  inherited Create(aOwner,AItemClass);
end;

function TFPReportImages.AddImageItem: TFPReportImageItem;
begin
  Result := Add as TFPReportImageItem;
end;

function TFPReportImages.AddFromStream(const AStream: TStream;
    Handler: TFPCustomImageReaderClass; KeepImage: Boolean): Integer;
var
  I: TFPCustomImage;
  IP: TFPReportImageItem;
  Reader: TFPCustomImageReader;
begin
  IP := AddImageItem;
  I := TFPCompactImgRGBA8Bit.Create(0,0);
  Reader := Handler.Create;
  try
    I.LoadFromStream(AStream, Reader);
  finally
    Reader.Free;
  end;
  IP.Image := I;
  if Not KeepImage then
  begin
    IP.CreateStreamedData;
    IP.FImage := Nil; // not through property, that would clear the image
    I.Free;
  end;
  Result := Count-1;
end;

function TFPReportImages.AddFromFile(const AFileName: string; KeepImage: Boolean): Integer;

  {$IF NOT (FPC_FULLVERSION >= 30101)}
  function FindReaderFromExtension(extension: String): TFPCustomImageReaderClass;
  var
    s: string;
    r: integer;
  begin
    extension := lowercase (extension);
    if (extension <> '') and (extension[1] = '.') then
      system.delete (extension,1,1);
    with ImageHandlers do
    begin
      r := count-1;
      s := extension + ';';
      while (r >= 0) do
      begin
        Result := ImageReader[TypeNames[r]];
        if (pos(s,{$if (FPC_FULLVERSION = 20604)}Extentions{$else}Extensions{$endif}[TypeNames[r]]+';') <> 0) then
          Exit;
        dec (r);
      end;
    end;
    Result := nil;
  end;

  function FindReaderFromFileName(const filename: String): TFPCustomImageReaderClass;
  begin
    Result := FindReaderFromExtension(ExtractFileExt(filename));
  end;
  {$ENDIF}

var
  FS: TFileStream;
begin
  FS := TFileStream.Create(AFileName, fmOpenRead or fmShareDenyNone);
  try
    Result := AddFromStream(FS,
      {$IF (FPC_FULLVERSION >= 30101)}TFPCustomImage.{$ENDIF}FindReaderFromFileName(AFileName), KeepImage);
  finally
    FS.Free;
  end;
end;

function TFPReportImages.AddFromData(const AImageData: Pointer; const AImageDataSize: LongWord): integer;
var
  s: TMemoryStream;
begin
  s := TMemoryStream.Create;
  try
    s.Write(AImageData^, AImageDataSize);
    s.Position := 0;
    Result := AddFromStream(s, TFPReaderPNG, True);
  finally
    s.Free;
  end;
end;

function TFPReportImages.GetIndexFromID(const AID: integer): integer;
var
  i: integer;
begin
  result := -1;
  if AID<0 then
    exit;
  for i := 0 to Count-1 do
  begin
    if Images[i].ID = AID then
    begin
      Result := i;
      Exit;
    end;
  end;
end;

function TFPReportImages.GetImageFromID(const AID: integer): TFPCustomImage;

Var
  II : TFPReportImageItem;

begin
  II:=GetImageItemFromID(AID);
  if II<>Nil then
    Result:=II.Image
  else
    Result:=Nil;
end;

function TFPReportImages.GetImageItemFromID(const AID: integer): TFPReportImageItem;

Var
  I : Integer;
begin
  I:=GetIndexFromID(AID);
  if I<>-1 then
    Result:=Images[I]
  else
    Result:=Nil;
end;

{ TFPReportPageSize }

procedure TFPReportPageSize.SetHeight(const AValue: TFPReportUnits);
begin
  if FHeight = AValue then
    Exit;
  FHeight := AValue;
  Changed;
end;

procedure TFPReportPageSize.CheckPaperSize;
var
  i: integer;
begin
  I := PaperManager.IndexOfPaper(FPaperName);
  if (I <> -1) then
  begin
    FWidth := PaperManager.PaperWidth[I];
    FHeight := PaperManager.PaperHeight[I];
    Changed;
  end;
end;

procedure TFPReportPageSize.SetPaperName(const AValue: string);
begin
  if FPaperName = AValue then
    Exit;
  FPaperName := AValue;
  if (FPaperName <> '') then
    CheckPaperSize;
end;

procedure TFPReportPageSize.SetWidth(const AValue: TFPReportUnits);
begin
  if FWidth = AValue then
    Exit;
  FWidth := AValue;
  Changed;
end;

procedure TFPReportPageSize.Changed;
begin
  if Assigned(FPage) then
    FPage.PageSizeChanged;
end;

constructor TFPReportPageSize.Create(APage: TFPReportCustomPage);
begin
  FPage := APage;
end;

procedure TFPReportPageSize.Assign(Source: TPersistent);
var
  S: TFPReportPageSize;
begin
  if Source is TFPReportPageSize then
  begin
    S := Source as TFPReportPageSize;
    FPaperName := S.FPaperName;
    FWidth := S.FWidth;
    FHeight := S.FHeight;
    Changed;
  end
  else
    inherited Assign(Source);
end;

{ TFPReportExporter }

procedure TFPReportExporter.SetFPReport(AValue: TFPCustomReport);
begin
  if FPReport = AValue then
    Exit;
  if Assigned(FPReport) then
    FPReport.RemoveFreeNotification(Self);
  FPReport := AValue;
  if Assigned(FPReport) then
    FPReport.FreeNotification(Self);
end;

procedure TFPReportExporter.SetBaseFileName(AValue: string);
begin
  if FBaseFileName=AValue then Exit;
  FBaseFileName:=AValue;
end;

procedure TFPReportExporter.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if (Operation=opRemove) and (AComponent=FPReport) then
    FPReport:=Nil;
end;

procedure TFPReportExporter.RenderImage(aPos: TFPReportRect; var AImage: TFPCustomImage);
begin
  // Do nothing
end;

TYpe

  { TMyFPCompactImgRGBA8Bit }

  TMyFPCompactImgRGBA8Bit = Class(TFPCompactImgRGBA8Bit)
    procedure SetInternalColor (x, y: integer; const Value: TFPColor); override;
  end;

{ TMyFPCompactImgRGBA8Bit }

procedure TMyFPCompactImgRGBA8Bit.SetInternalColor(x, y: integer; const Value: TFPColor);
begin
  if (X<0) or (Y<0) or (X>=Width) or (Y>=Height) then
    Writeln('(',X,',',Y,') not in (0,0)x(',Width-1,',',Height-1,')')
  else
    inherited SetInternalColor(x, y, Value);
end;

procedure TFPReportExporter.RenderUnknownElement(aBasePos: TFPReportPoint;
  AElement: TFPReportElement; ADPI: Integer);

Var
  C : TFPReportElementExporterCallBack;
  IC : TFPReportImageRenderCallBack;
  Img : TFPCustomImage;
  H,W : Integer;
  R : TFPReportRect;
  L : TFPReportLayout;

begin
  L:=aElement.RTLayout;
  if (L=Nil) then
    L:=aElement.Layout;
  // Actually, this could be cached using propertyhash...
  C:=gElementFactory.FindRenderer(TFPReportExporterClass(self.ClassType),TFPReportElementClass(aElement.ClassType));
  if (C<>Nil) then
    // There is a direct renderer
    C(aBasePos, aElement,Self,aDPI)
  else
    begin
    // There is no direct renderer, try rendering to image
    IC:=gElementFactory.FindImageRenderer(TFPReportElementClass(aElement.ClassType));
    if Assigned(IC) then
      begin
      H := Round(L.Height * (aDPI / cMMperInch));
      W := Round(L.Width * (aDPI / cMMperInch));
      Img:=TFPCompactImgRGBA8Bit.Create(W,H);
      try
        IC(aElement,Img);
        R.Left:=aBasePos.Left+L.Left;
        R.Top:=aBasePos.Top+L.Top;
        R.Width:=L.Width;
        R.Height:=L.Height;
        RenderImage(R,Img);
      finally
        Img.Free;
      end;
      end;
    end;
end;


class function TFPReportExporter.DefaultConfig: TFPReportExporterConfigHandler;
begin
  Result:=Nil;
end;

procedure TFPReportExporter.Execute;
begin
  if (FPReport.RTObjects.Count=0) and AutoRun then
    FPreport.RunReport;
  if FPReport.RTObjects.Count > 0 then
    DoExecute(FPReport.RTObjects);
end;

procedure TFPReportExporter.SetFileName(const aFileName: String);
begin
  // Do nothing
end;

class procedure TFPReportExporter.RegisterExporter;
begin
  ReportExportManager.RegisterExport(Self);
end;

class procedure TFPReportExporter.UnRegisterExporter;
begin
  ReportExportManager.UnRegisterExport(Self);
end;

class function TFPReportExporter.Description: String;
begin
  Result:='';
end;

class function TFPReportExporter.Name: String;
begin
  Result:=ClassName;
end;

class function TFPReportExporter.DefaultExtension: String;
begin
  Result:='';
end;

class function TFPReportExporter.MultiFile: Boolean;
begin
  Result:=False;
end;

function TFPReportExporter.ShowConfig: Boolean;
begin
  Result:=ReportExportManager.ConfigExporter(Self);
end;

{ TFPReportPaperSize }

constructor TFPReportPaperSize.Create(const AWidth, AHeight: TFPReportUnits);
begin
  FWidth := AWidth;
  FHeight := AHeight;
end;

{ TFPReportFont }

procedure TFPReportFont.SetFontName(const avalue: string);
begin
  FFontName := AValue;
  Changed;
end;

procedure TFPReportFont.SetFontSize(const avalue: integer);
begin
  FFontSize := AValue;
  Changed;
end;

procedure TFPReportFont.SetFontColor(const avalue: TFPReportColor);
begin
  FFontColor := AValue;
  Changed;
end;

procedure TFPReportFont.Changed;
begin
  If Assigned(FOnChanged) then
    FOnChanged(Self);
end;

constructor TFPReportFont.Create;
begin
  inherited Create;
  FFontName := ReportDefaultFont;
  FFontColor := clBlack;
  FFontSize := 10;
end;

procedure TFPReportFont.Assign(Source: TPersistent);
var
  o: TFPReportFont;
begin
  if (Source is TFPReportFont) then
    begin
    o := TFPReportFont(Source);
    FFontName := o.Name;
    FFontSize := o.Size;
    FFontColor := o.Color;
    Changed;
    end
  else
    Inherited Assign(Source);
end;

{ TFPReportPaperManager }

function TFPReportPaperManager.GetPaperHeight(AIndex: integer): TFPReportUnits;
begin
  Result := TFPReportPaperSize(FPaperSizes.Objects[AIndex]).Height;
end;

function TFPReportPaperManager.GetPaperHeightByName(AName: string): TFPReportUnits;
begin
  Result := GetPaperByName(AName).Height;
end;

function TFPReportPaperManager.GetPaperCount: integer;
begin
  Result := FPaperSizes.Count;
end;

function TFPReportPaperManager.GetPaperName(AIndex: integer): string;
begin
  Result := FPaperSizes[AIndex];
end;

function TFPReportPaperManager.GetPaperWidth(AIndex: integer): TFPReportUnits;
begin
  Result := TFPReportPaperSize(FPaperSizes.Objects[AIndex]).Width;
end;

function TFPReportPaperManager.GetPaperWidthByName(AName: string): TFPReportUnits;
begin
  Result := GetPaperByName(AName).Width;
end;

function TFPReportPaperManager.FindPaper(const AName: string): TFPReportPaperSize;
var
  I: integer;
begin
  I := IndexOfPaper(AName);
  if (I = -1) then
    Result := nil
  else
    Result := TFPReportPaperSize(FPaperSizes.Objects[i]);
end;

function TFPReportPaperManager.GetPaperByname(const AName: string): TFPReportPaperSize;
begin
  Result := FindPaper(AName);
  if Result = nil then
    ReportError(SErrUnknownPaper, [AName]);
end;

constructor TFPReportPaperManager.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FPaperSizes := TStringList.Create;
  FPaperSizes.Sorted := True;
end;

destructor TFPReportPaperManager.Destroy;
var
  I: integer;
begin
  if Assigned(FPaperSizes) then
  begin
    for I := 0 to FPaperSizes.Count - 1 do
      FPaperSizes.Objects[i].Free;
    FreeAndNil(FPaperSizes);
  end;
  inherited Destroy;
end;

procedure TFPReportPaperManager.Clear;
var
  i: integer;
begin
  for i := 0 to FPaperSizes.Count-1 do
    if Assigned(FPaperSizes.Objects[i]) then
      FPaperSizes.Objects[i].Free;
  FPaperSizes.Clear;
end;

function TFPReportPaperManager.IndexOfPaper(const AName: string): integer;
begin
  if not Assigned(FPaperSizes) then
    Result := -1
  else
    Result := FPaperSizes.IndexOf(AName);
end;

procedure TFPReportPaperManager.RegisterPaper(const AName: string; const AWidth, AHeight: TFPReportUnits);
var
  I: integer;
  S: TFPReportPaperSize;
begin
  I := FPaperSizes.IndexOf(AName);
  if (I = -1) then
  begin
    S := TFPReportPaperSize.Create(AWidth, AHeight);
    FPaperSizes.AddObject(AName, S);
  end
  else
    ReportError(SErrDuplicatePaperName, [AName]);
end;

{ Got details from Wikipedia [https://simple.wikipedia.org/wiki/Paper_size] }
procedure TFPReportPaperManager.RegisterStandardSizes;
begin
  // As per TFPReportUnits, size is specified in millimetres.
  RegisterPaper('A3', 297, 420);
  RegisterPaper('A4', 210, 297);
  RegisterPaper('A5', 148, 210);
  RegisterPaper('Letter', 216, 279);
  RegisterPaper('Legal', 216, 356);
  RegisterPaper('Ledger', 279, 432);
  RegisterPaper('DL',	220, 110);
  RegisterPaper('B5',	176, 250);
  RegisterPaper('C5',	162, 229);
end;

procedure TFPReportPaperManager.GetRegisteredSizes(var AList: TStringList);
var
  i: integer;
begin
  if not Assigned(AList) then
    Exit;
  AList.Clear;
  for i := 0 to FPaperSizes.Count - 1 do
    AList.Add(PaperNames[i]);
end;

procedure DoneReporting;
begin
  if Assigned(uPaperManager) then
    FreeAndNil(uPaperManager);
  TFPReportCustomCheckbox.ImgFalse.Free;
  TFPReportCustomCheckbox.ImgTrue.Free;
end;

{ TFPTextBlockList }

function TFPTextBlockList.GetItem(AIndex: Integer): TFPTextBlock;
begin
  Result := TFPTextBlock(inherited GetItem(AIndex));
end;

procedure TFPTextBlockList.SetItem(AIndex: Integer; AObject: TFPTextBlock);
begin
  inherited SetItem(AIndex, AObject);
end;

{ TFPReportDataField }

function TFPReportDataField.GetValue: variant;
begin
  Result := Null;
  if Assigned(Collection) then
    TFPReportDatafields(Collection).ReportData.DoGetValue(FieldName, Result);
end;



procedure TFPReportDataField.InitValue(SavePrevious: Boolean);
begin
  if Not SavePrevious then
    FPrevValue := nil
  else
    FPrevValue := FValue;
  FValue:=GetValue;
//  Writeln('Init ',Self.FieldName,' : ',safeVariant(FValue),' Previous : ',SafeVariant(FPrevValue));
end;

procedure TFPReportDataField.GetRTValue(Var Result: TFPExpressionResult;
  ConstRef AName: ShortString);

  procedure SetResult(const pValue: Variant);
  begin
    if Assigned(FExprIdentierDef) then
      if varIsNull(pValue) then
        case FExprIdentierDef.ResultType of
          rtBoolean:    Result.ResBoolean   := False;
          rtInteger:    Result.ResInteger   := 0;
          rtFloat:      Result.ResFloat     := 0.0;
          rtCurrency:   Result.ResCurrency  := 0.0;
          rtDateTime:   Result.ResDateTime  := 0.0;
          rtString:     Result.ResString    := '';
        end
      else
        case FExprIdentierDef.ResultType of
          rtBoolean:    Result.ResBoolean   := pValue;
          rtInteger:    Result.ResInteger   := pValue;
          rtFloat:      Result.ResFloat     := pValue;
          rtCurrency:   Result.ResCurrency  := pValue;
          rtDateTime:   Result.ResDateTime  := pValue;
          rtString:     Result.ResString    := pValue;
        end;
  end;

begin
  if Assigned(FOnGetUsePrevValue) and FOnGetUsePrevValue() then
    begin
//    Writeln(FieldName,' Getting previous value : ',SafeVariant(FPrevValue));
    SetResult(FPrevValue)
    end
  else
    begin
//    Writeln(FieldName,' Getting current value : ',SafeVariant(FValue));
    SetResult(FValue);
    end;
end;

procedure TFPReportDataField.Assign(Source: TPersistent);
var
  F: TFPReportDataField;
begin
  if Source is TFPReportDataField then
  begin
    F := Source as TFPReportDataField;
    FDisplayWidth := F.FDisplayWidth;
    FFieldKind := F.FFieldKind;
    FFieldName := F.FFieldName;
    FOnGetUsePrevValue := F.FOnGetUsePrevValue;
    FExprIdentierDef := F.FExprIdentierDef;
  end
  else
    inherited Assign(Source);
end;

{ TFPReportDataFields }

function TFPReportDataFields.GetF(AIndex: integer): TFPReportDataField;
begin
  Result := TFPReportDataField(Items[AIndex]);
end;

procedure TFPReportDataFields.SetF(AIndex: integer; const AValue: TFPReportDataField);
begin
  Items[AIndex] := AValue;
end;

function TFPReportDataFields.AddField(AFieldName: string; AFieldKind: TFPReportFieldKind): TFPReportDataField;
begin
  Result := Add as TFPReportDataField;
  try
    Result.FieldName := AFieldName;
    Result.FieldKind := AFieldKind;
  except
    Result.Free;
    raise;
  end;
end;

function TFPReportDataFields.IndexOfField(const AFieldName: string): integer;
begin
  Result := Count - 1;
  while (Result >= 0) and (CompareText(AFieldName, GetF(Result).FieldName) <> 0) do
    Dec(Result);
end;

function TFPReportDataFields.FindField(const AFieldName: string): TFPReportDataField;
var
  I: integer;
begin
  I := IndexOfField(AFieldName);
  if (I = -1) then
    Result := nil
  else
    Result := GetF(I);
end;

function TFPReportDataFields.FindField(const AFieldName: string; const AFieldKind: TFPReportFieldKind): TFPReportDataField;
var
  lIndex: integer;
begin
  lIndex := Count - 1;
  while (lIndex >= 0) and (not SameText(AFieldName, GetF(lIndex).FieldName)) and (GetF(lIndex).FieldKind <> AFieldKind) do
      Dec(lIndex);

  if (lIndex = -1) then
    Result := nil
  else
    Result := GetF(lIndex);
end;

function TFPReportDataFields.FieldByName(const AFieldName: string): TFPReportDataField;
begin
  Result := FindField(AFieldName);
  if (Result = nil) then
  begin
    if Assigned(ReportData) then
      ReportError(SErrUnknownField, [ReportData.Name, AFieldName])
    else
      ReportError(SErrUnknownField, ['', AFieldName]);
  end;
end;

{ TFPReportData }

procedure TFPReportData.SetDataFields(const AValue: TFPReportDataFields);
begin
  if (FDataFields = AValue) then
    Exit;
  FDataFields.Assign(AValue);
end;

function TFPReportData.GetFieldCount: integer;
begin
  Result := FDatafields.Count;
end;

function TFPReportData.GetFieldName(Index: integer): string;
begin
  Result := FDatafields[Index].FieldName;
end;

function TFPReportData.GetFieldType(AFieldName: string): TFPReportFieldKind;
begin
  Result := FDatafields.FieldByName(AFieldName).FieldKind;
end;

function TFPReportData.GetFieldValue(AFieldName: string): variant;
begin
  Result := varNull;
  DoGetValue(AFieldName, Result);
end;

function TFPReportData.GetFieldWidth(AFieldName: string): integer;
begin
  Result := FDataFields.FieldByName(AFieldName).DisplayWidth;
end;

function TFPReportData.GetIsOpened: boolean;
begin
  Result:=FIsOpened;
end;

function TFPReportData.GetLastFieldValue(AFieldName: string): variant;
begin
  Result := FDataFields.FieldByName(AFieldName).FPrevValue;
end;

function TFPReportData.CreateDataFields: TFPReportDataFields;
begin
  Result := TFPReportDataFields.Create(TFPReportDataField);
end;

procedure TFPReportData.DoGetValue(const AFieldName: string; var AValue: variant);
begin
  AValue := Null;
end;

procedure TFPReportData.DoInitDataFields;
begin
  // Do nothing.
end;

procedure TFPReportData.DoOpen;
begin
  // Do nothing
end;

procedure TFPReportData.DoFirst;
begin
  // Do nothing
end;

procedure TFPReportData.DoNext;
begin
  // Do nothing
end;

procedure TFPReportData.DoClose;
begin
  // Do nothing
end;

function TFPReportData.DoEOF: boolean;
begin
  Result := False;
end;

constructor TFPReportData.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FDatafields := CreateDataFields;
  FDatafields.FReportData := Self;
end;

destructor TFPReportData.Destroy;
begin
  FreeAndNil(FDatafields);
  inherited Destroy;
end;

procedure TFPReportData.InitFieldDefs;
begin
  if IsOpened then
    ReportError(SErrInitFieldsNotAllowedAfterOpen);
  DoInitDataFields;
end;

procedure TFPReportData.Open;

begin
  if IsOpened then
    exit;
  if Assigned(FOnOpen) then
    FOnOpen(Self);
  DoOpen;
  InitFieldDefs;
  FIsOpened := True;
  FRecNo := 1;
  If not EOF then
    InitFieldValues(false);
end;

procedure TFPReportData.InitFieldValues(SavePrevious : Boolean);

var
  I: Integer;

begin
  for I := 0 to FDataFields.Count - 1 do
    FDataFields[i].InitValue(SavePrevious);
end;

procedure TFPReportData.First;
begin
  FRecNo := 1;
  if Assigned(FOnFirst) then
    FOnFirst(Self);
  DoFirst;
  InitFieldValues(False);
end;

procedure TFPReportData.Next;
begin
  Inc(FRecNo);
  if Assigned(FOnNext) then
    FOnNext(Self);
  DoNext;
  if not EOF then
    InitFieldValues(True);
end;

procedure TFPReportData.Close;
begin
  if Assigned(FOnClose) then
    FOnClose(Self);
  FIsOpened := False;
  DoClose;
  FRecNo := -1;
end;

function TFPReportData.EOF: boolean;
begin
  Result := False;
  if Assigned(FOnGetEOF) then
    FOnGetEOF(Self, Result);
  if not Result then
    Result := DoEOF;
end;

procedure TFPReportData.GetFieldList(List: TStrings);
var
  I: integer;
begin
  List.BeginUpdate;
  try
    List.Clear;
    for I := 0 to FDataFields.Count - 1 do
      List.add(FDataFields[I].FieldName);
  finally
    List.EndUpdate;
  end;
end;

function TFPReportData.IndexOfField(const AFieldName: string): Integer;
begin
  Result:=  FDataFields.IndexOfField(AFieldName);
end;

function TFPReportData.HasField(const AFieldName: string): boolean;
begin
  Result := FDataFields.IndexOfField(AFieldName) <> -1;
end;


{ TFPReportClassMapping }

function TFPReportClassMapping.IndexOfExportRenderer(
  AClass: TFPReportExporterClass): Integer;
begin
  Result:=Length(FRenderers)-1;
  While (Result>=0) and (FRenderers[Result].aClass<>AClass) do
    Dec(Result);
end;

constructor TFPReportClassMapping.Create(const AMappingName: string; AElementClass: TFPReportElementClass);
begin
  FMappingName :=  AMappingName;
  FReportElementClass := AElementClass;
end;

procedure TFPReportClassMapping.SetIconFromBytes(B: array of Byte);
begin
  SetLength(FIConData,Length(B));
  Move(B[0],FIconData[0],Length(B));
end;

function TFPReportClassMapping.AddRenderer(aExporterClass: TFPReportExporterClass; aCallback: TFPReportElementExporterCallBack ): TFPReportElementExporterCallBack;

Var
  I : Integer;

begin
  Result:=nil;
  I:=IndexOfExportRenderer(aExporterClass);
  if (I=-1) then
    begin
    I:=Length(FRenderers);
    SetLength(FRenderers,I+1);
    FRenderers[i].aClass:=aExporterClass;
    FRenderers[i].aCallback:=Nil;
    end;
  Result:=FRenderers[i].aCallback;
  FRenderers[i].aCallback:=aCallback;
end;

function TFPReportClassMapping.FindRenderer(aClass: TFPReportExporterClass): TFPReportElementExporterCallBack;

Var
  I : Integer;

begin
  I:=IndexOfExportRenderer(aClass);
  if I<>-1 then
    Result:=FRenderers[I].aCallback
  else
    Result:=Nil;
end;

{ TFPReportElementFactory }

function TFPReportElementFactory.GetM(Aindex : integer): TFPReportClassMapping;
begin
  Result:=TFPReportClassMapping(FList[AIndex]);
end;

Function TFPReportElementFactory.GetDefaultBandType(AType : TFPReportBandType) : TFPReportCustomBandClass;

begin
  Case AType of
    btUnknown       : Result:=Nil;
    btPageHeader    : Result:=TFPReportPageHeaderBand;
    btReportTitle   : Result:=TFPReportTitleBand;
    btColumnHeader  : Result:=TFPReportColumnHeaderBand;
    btDataHeader    : Result:=TFPReportDataHeaderBand;
    btGroupHeader   : Result:=TFPReportGroupHeaderBand;
    btDataband      : Result:=TFPReportDataBand;
    btGroupFooter   : Result:=TFPReportGroupFooterBand;
    btDataFooter    : Result:=TFPReportDataFooterBand;
    btColumnFooter  : Result:=TFPReportColumnFooterBand;
    btReportSummary : Result:=TFPReportSummaryBand;
    btPageFooter    : Result:=TFPReportPageFooterBand;
    btChild         : Result:=TFPReportChildBand;
  else
    raise EReportError.CreateFmt(SErrUnknownBandType, [Ord(AType)]);
  end;
end;


function TFPReportElementFactory.GetBC(aType : TFPReportBandType): TFPReportCustomBandClass;

Var
  C : TFPReportCustomBandClass;
  N : String;
  I : Integer;

begin
  C:=GetDefaultBandType(aType);
  if C=Nil then
    begin
    N:=GetEnumName(TypeInfo(TFPReportBandType),Ord(AType));
    Raise EReportError.CreateFmt(SErrCouldNotGetDefaultBandType, [N]);
    end;
  I:=IndexOfElementName(C.ElementType);
  If (I=-1) then
    Result:=C
  else
    begin
    Result:=TFPReportCustomBandClass(Mappings[I].ReportElementClass);
    If Not Result.InheritsFrom(C) then
      raise EReportError.CreateFmt(SErrBandClassMustDescendFrom, [N, C.ClassName]);
    end;
end;

function TFPReportElementFactory.GetMappingCount: Integer;
begin
  Result:=FList.Count;
end;

function TFPReportElementFactory.IndexOfElementName(const AElementName: string): Integer;

begin
  Result:=Flist.Count-1;
  While (Result>=0) and not SameText(Mappings[Result].MappingName, AElementName) do
    Dec(Result);
end;

function TFPReportElementFactory.IndexOfElementClass(const AElementClass: TFPReportElementClass): Integer;

begin
  Result:=Flist.Count-1;
  While (Result>=0) and (Mappings[Result].ReportElementClass<>AElementClass) do
    Dec(Result);
end;

constructor TFPReportElementFactory.Create;
begin
  FList := TFPObjectList.Create;
end;

destructor TFPReportElementFactory.Destroy;
begin
  FList.Free;
  inherited Destroy;
end;

function TFPReportElementFactory.FindRenderer(aClass: TFPReportExporterClass;
  AElement: TFPReportElementClass): TFPReportElementExporterCallBack;

Var
  I : Integer;

begin
  Result:=nil;
  I:=IndexOfElementClass(aElement);
  if I<>-1 then
    Result:=Mappings[i].FindRenderer(aClass);
end;

function TFPReportElementFactory.FindImageRenderer(
  AElement: TFPReportElementClass): TFPReportImageRenderCallBack;
Var
  I : Integer;

begin
  Result:=nil;
  I:=IndexOfElementClass(aElement);
  if I<>-1 then
    Result:=Mappings[i].ImageRenderCallback;
end;

function TFPReportElementFactory.RegisterImageRenderer(AElement: TFPReportElementClass; ARenderer: TFPReportImageRenderCallBack
  ): TFPReportImageRenderCallBack;
Var
  I : Integer;
begin
  Result:=nil;
  I:=IndexOfElementClass(aElement);
  if I<>-1 then
    begin
    Result:=Mappings[i].ImageRenderCallback;
    Mappings[i].ImageRenderCallback:=ARenderer;
    end;
end;

function TFPReportElementFactory.RegisterElementRenderer(AElement: TFPReportElementClass; ARenderClass: TFPReportExporterClass;
  ARenderer: TFPReportElementExporterCallBack): TFPReportElementExporterCallBack;
Var
  I : Integer;
begin
  Result:=nil;
  I:=IndexOfElementClass(aElement);
  if (I<>-1) then
    Result:=Mappings[i].AddRenderer(aRenderClass,ARenderer);
end;

procedure TFPReportElementFactory.RegisterEditorClass(const AElementName: string; AEditorClass: TFPReportElementEditorClass);

Var
  I : integer;

begin
  I:=IndexOfElementName(aElementName);
  if I<>-1 then
    Mappings[i].EditorClass:=AEditorClass
  else
    Raise EReportError.CreateFmt(SErrUnknownElementName,[AElementName]);
end;

procedure TFPReportElementFactory.RegisterEditorClass(AReportElementClass: TFPReportElementClass;
  AEditorClass: TFPReportElementEditorClass);

Var
  I : integer;

begin
  I:=IndexOfElementClass(aReportElementClass);
  if I<>-1 then
    Mappings[i].EditorClass:=AEditorClass
  else
    if AReportElementClass<>Nil then
      Raise EReportError.CreateFmt(SErrUnknownElementClass,[AReportElementClass.ClassName])
    else
      Raise EReportError.CreateFmt(SErrUnknownElementClass,['Nil']);
end;

procedure TFPReportElementFactory.UnRegisterEditorClass(const AElementName: string; AEditorClass: TFPReportElementEditorClass);

Var
  I : integer;

begin
  I:=IndexOfElementName(aElementName);
  if I<>-1 then
    if Mappings[i].EditorClass=AEditorClass then
      Mappings[i].EditorClass:=nil;
end;

procedure TFPReportElementFactory.UnRegisterEditorClass(AReportElementClass: TFPReportElementClass;
  AEditorClass: TFPReportElementEditorClass);
Var
  I : integer;

begin
  I:=IndexOfElementClass(aReportElementClass);
  if I<>-1 then
    if Mappings[i].EditorClass=AEditorClass then
      Mappings[i].EditorClass:=nil;
end;

function TFPReportElementFactory.RegisterClass(const AElementName: string; AReportElementClass: TFPReportElementClass
  ): TFPReportClassMapping;
var
  i: integer;
begin
  I:=IndexOfElementName(AElementName);
  if I<>-1 then
    exit;
  Result:=TFPReportClassMapping.Create(AElementName, AReportElementClass);
  FList.Add(Result);
end;

procedure TFPReportElementFactory.RemoveClass(const AElementName: string);

var
  i: integer;
begin
  I:=IndexOfElementName(AElementName);
  if I<>-1 then
    FList.Delete(I);
end;

function TFPReportElementFactory.CreateInstance(const AElementName: string; AOwner: TComponent): TFPReportElement;
var
  i: integer;
begin
  Result := nil;
  for i := 0 to FList.Count - 1 do
  begin
    if SameText(Mappings[I].MappingName, AElementName) then
    begin
      Result := Mappings[I].ReportElementClass.Create(AOwner);
      Break; //==>
    end;
  end;
  if Result = nil then
    ReportError(SErrRegisterUnknownElement, [AElementName]);
end;

function TFPReportElementFactory.FindEditorClassForInstance(AInstance: TFPReportElement): TFPReportElementEditorClass;
begin
  if AInstance<>Nil then
    Result:=FindEditorClassForInstance(TFPReportElementClass(Ainstance.ClassType))
  else
    Result:=Nil;
end;

function TFPReportElementFactory.FindEditorClassForInstance(AClass: TFPReportElementClass): TFPReportElementEditorClass;

Var
  I : Integer;

begin
  I:=IndexOfElementClass(AClass);
  if I<>-1 then
    Result:=Mappings[I].EditorClass
  else
    Result:=nil;
end;

procedure TFPReportElementFactory.AssignReportElementTypes(AStrings: TStrings);
var
  i: integer;
begin
  AStrings.Clear;
  for i := 0 to FList.Count - 1 do
    AStrings.Add(Mappings[I].MappingName);
end;

function TFPReportElementFactory.PageClass: TFPReportCustomPageClass;

Var
  I : Integer;

begin
  I:=IndexOfElementName(TFPReportCustomPage.ElementType);
  if I=-1 then
    Result:=nil
  else
    Result:=TFPReportCustomPageClass(Mappings[i].ReportElementClass);
end;

{ TFPReportCustomDataHeaderBand }

class function TFPReportCustomDataHeaderBand.ElementType: String;
begin
  Result := 'DataHeaderBand';
end;

class function TFPReportCustomDataHeaderBand.ReportBandType: TFPReportBandType;
begin
  Result:=btDataHeader;
end;

{ TFPReportCustomDataFooterBand }

class function TFPReportCustomDataFooterBand.ElementType: String;
begin
  Result := 'DataFooterBand';
end;

class function TFPReportCustomDataFooterBand.ReportBandType: TFPReportBandType;
begin
  Result:=btDataFooter;
end;

{ ---------------------------------------------------------------------
  TFPReportLayouter
  ---------------------------------------------------------------------}

procedure TFPReportLayouter.UpdateSpaceRemaining(const ABand: TFPReportCustomBand; const AUpdateYPos: boolean = True);
begin
  FSpaceLeft := FSpaceLeft - ABand.RTLayout.Height;
  if AUpdateYPos then
    FLastYPos := FLastYPos + ABand.RTLayout.Height;
end;

function TFPReportLayouter.CommonRuntimeBandProcessing(const aBand: TFPReportCustomBand): TFPReportCustomBand;

// We evaluate visibility, and if it's not visible, we free and nil the result.
// This frees the caller from the task of doing so.

begin
  aBand.MainBand.BeginRuntimeProcessing;
  Result:=TFPReportCustomBand(aBand.PrepareObject(FRTPage));
  Result.RecalcLayout;
  Result.BeforePrint;
  if Result.EvaluateVisibility then
  begin
    if aBand.MainBand is TFPReportCustomPageFooterBand then
    begin
      FPageFooterYPos := FPageFooterYPos - Result.RTLayout.Height;
      Result.RTLayout.Top := FPageFooterYPos;
    end
    else
      Result.RTLayout.Top := FLastYPos;
    if aBand.FIsColumnType then
      Result.RTLayout.Left := FLastXPos;
  end
  else
  begin
     { remove band from current page }
     Result.Page.RemoveChild(Result);
     { free mem }
     FreeAndNil(Result);
  end;
  aBand.MainBand.EndRuntimeProcessing;
end;

function TFPReportLayouter.HandleHeaderBands: Boolean;
begin
  Result := False;
  { Show all header bands }
  if Assigned(FPageHeader) then
    //ShowPageHeaderBand(FPageHeader);
    ShowBandWithChilds(FPageHeader);
  if Assigned(FTitle) then
    ShowBandWithChilds(FTitle);
end;

procedure TFPReportLayouter.HandleFooterBands;

Var
  lFooter, lRTBand, lBand: TFPReportCustomBand;
  i: Integer;
  lBandPosition: TFPReportBandPosition;
  lList: TBandList;
  lGrp: TFPReportCustomGroupHeaderBand;

begin
  //WriteLn('HandleFooterBands:');
  { 1. from bottom to top for page footer }
  if Assigned(FPageFooter) and RTIsLastColumn then
  begin
    lList:=TBandList.Create;
    try
      lBand := FPageFooter;
      while Assigned(lBand) do
      begin
        lList.Add(lBand);
        lBand := lBand.ChildBand;
      end;
      FPageFooter.BeforePrintWithChilds;
      for i := lList.Count-1 downto 0 do
      begin
        lRTBand := CommonRuntimeBandProcessing(lList[i]);
        if Assigned(lRTBand) then
          FPageFooter.AfterPrintBand(lRTBand);
      end;
      FPageFooter.AfterPrintWithChilds;
    finally
      lList.Free;
    end;
  end;
  { 2. from top to bottom for group footer }
  for i:=CurrentLoop.FGroupFooterList.Count-1 downto 0 do
  begin
    lFooter:=CurrentLoop.FGroupFooterList[i];
    lGrp := TFPReportCustomGroupFooterBand(lFooter).GroupHeader;
    if not lGrp.NeedsIntermediateFooter(RTIsLastColumn) then
      Continue;
    lBandPosition := lFooter.BandPosition;
    Report.FRTInIntermediateGroupFooter := True;
    lFooter.BeforePrintWithChilds;
    lBand := lFooter;
    while Assigned(lBand) do
    begin
      lRTBand := CommonRuntimeBandProcessing(lBand);
      if Assigned(lRTBand) then begin
        if lBandPosition = bpNormal then
          UpdateSpaceRemaining(lRTBand);
        lFooter.AfterPrintBand(lRTBand);
        if lFooter.BandPosition = bpStackAtBottom then
        begin
          { mark for HandleBottomStackedFooters }
          lRTBand.RTLayout.Top := -1;
          FRTBottomStackedFooterList.Add(lRTBand);
        end;
      end;
      lBand := lBand.ChildBand;
    end;
    lFooter.AfterPrintWithChilds;
    Report.FRTInIntermediateGroupFooter := False;
  end;
  { 3. Handle column footer }
  if Assigned(FColumnFooter) then
    ShowColumnFooterBand(FColumnFooter);
end;

procedure TFPReportLayouter.HandleBottomStackedFooters;
var
  lOffset: TFPReportUnits;
  i: Integer;
  lFooter: TFPReportCustomBand;
begin
  { move all allready layouted group footers (only bpStackAtBottom)
    up by offset of page footer                                      }
  lOffset := FRTPage.RTLayout.Top + FRTPage.RTLayout.Height - FPageFooterYPos;
  for i:=0 to FRTBottomStackedFooterList.Count-1 do
  begin
    lFooter := FRTBottomStackedFooterList[i];
    lFooter.RTLayout.Top := lFooter.RTLayout.Top - lOffset;
  end;
  FRTBottomStackedFooterList.Clear;
end;

procedure TFPReportLayouter.HandleRepeatedGroupHeaderBands(pNewPage: Boolean);

var
  I: Integer;
  lGrp: TFPReportCustomGroupHeaderBand;

  function ReprintedHeader: Boolean;
  begin
    Result := (
      pNewPage and
      (rsPage in lGrp.ReprintedHeader)
    ) or
    (rsColumn in lGrp.ReprintedHeader);
  end;

  function OverflowedFooterNeedsReprintedHeader: Boolean;
  begin
    Result := (
      pNewPage and
      (rsPage in lGrp.OverflowedFooterNeedsReprintedHeader)
    ) or
    (rsColumn in lGrp.OverflowedFooterNeedsReprintedHeader);
  end;

begin
  for I:=CurrentLoop.FGroupHeaderList.Count-1 downto 0 do
  begin
    lGrp := TFPReportCustomGroupHeaderBand(CurrentLoop.FGroupHeaderList[I]);
    if ReprintedHeader and
    ( not lGrp.GroupChanged or
    lGrp.FNeedsReprintedHeader) and
    ( not (Assigned(lGrp.GroupFooter) and lGrp.GroupFooter.FIsOverflowed) or
      OverflowedFooterNeedsReprintedHeader ) then
    begin
      Report.FRTInRepeatedGroupHeader := True;
      if lGrp.FNeedsPrevVariables then
        Report.RTBeginUsePrevVariableValues;
      ShowGroupHeaderBand(lGrp, False);
      Report.FRTInRepeatedGroupHeader := False;
      if lGrp.FNeedsPrevVariables then
         Report.RTEndUsePrevVariableValues;
    end;
  end;
end;

procedure TFPReportLayouter.ShowColumnFooterBand(aBand: TFPReportCustomColumnFooterBand);

var
  lBand, lRTBand : TFPReportCustomBand;

begin
  aBand.BeforePrintWithChilds;
  lBand := aBand;
  while Assigned(lBand) do
  begin
    lRTBand := CommonRuntimeBandProcessing(lBand);
    if Assigned(lRTBand) then
    begin
      UpdateSpaceRemaining(lRTBand);
      aBand.AfterPrintBand(lRTBand);
      if aBand.BandPosition = bpStackAtBottom then
      begin
        { mark for HandleBottomStackedFooters }
        lRTBand.RTLayout.Top := -1;
        FRTBottomStackedFooterList.Add(lRTBand);
      end;
    end;
    lBand := lBand.ChildBand;
  end;
  aBand.AfterPrintWithChilds;
end;

function TFPReportLayouter.NoSpaceRemaining: boolean;

var
  lSpaceNeeded: TFPReportUnits;

begin
  lSpaceNeeded := FooterSpaceNeeded;
  Result:=(FSpaceLeft-lSpaceNeeded)< 0;
  //writeln(' -> ',FormatFloat('#,##0.0', FSpaceLeft-lSpaceNeeded));
  if Result then
  begin
    if FRTIsMultiColumn and not RTIsLastColumn then
    begin
      FNewColumn := True;
    end
    else
    begin
      FOverflowed := True;
      FNewPage := True;
    end;
  end
end;

procedure TFPReportLayouter.EndColumn;
begin
  { handle footers }
  FPageFooterYPos := FRTPage.RTLayout.Top + FRTPage.RTLayout.Height;
  { page, column and group footers}
  HandleFooterBands;
  { bottom stacked group and column footers }
  PrepareBottomStackedFooters;
end;

procedure TFPReportLayouter.EndPage;
begin
  if Assigned(FRTPage) then
    begin
    EndColumn;
    { bottom stacked group and column footers }
    HandleBottomStackedFooters;
    end;
end;

procedure TFPReportLayouter.StartNewColumn;
begin
  { prepare next column/page }
  if Assigned(CurrentLoop.FDataBand) then
    report.ClearDataBandLastTextValues(CurrentLoop.FDataBand);
  FLastYPos := FColumnYStartPos;
  FSpaceLeft := Pages[RTCurDsgnPageIdx].Layout.Height - (FColumnYStartPos - Pages[RTCurDsgnPageIdx].Layout.Top);
  inc(FRTCurrentColumn);
  if FRTCurrentColumn > 1 then
    FLastXPos := FLastXPos + FRTPage.ColumnGap + FRTPage.BandWidthFromColumnCount;
  Report.FColumnNumber := FRTCurrentColumn;
  { print column header }
  if Assigned(FColumnHeader) then
    ShowBandWithChilds(FColumnHeader);
  FNewColumn := False;
  HandleRepeatedGroupHeaderBands(FRTCurrentColumn = 1);
  FColumnDetailsPrinted := False;
end;

procedure TFPReportLayouter.HandleOverflowed;

var
  lPrevRTPage: TFPReportCustomPage;
  lOverflowBand: TFPReportCustomBand;
  lBandCount: integer;

begin
  FOverflowed := False;
  lPrevRTPage := TFPReportCustomPage(RTObjects[RTCurPageIdx-1]);
  if (CurrentLoop.FGroupFooterList.Find(TFPReportCustomColumnFooterBand) <> nil) then
    lBandCount := lPrevRTPage.BandCount - 2  // skip over the ColumnFooter band
  else
    lBandCount := lPrevRTPage.BandCount - 1;
  lOverflowBand := lPrevRTPage.Bands[lBandCount]; // get the last band - the one that didn't fit
  lPrevRTPage.RemoveChild(lOverflowBand);
  FRTPage.AddChild(lOverflowBand);

  { Fix position of last band that caused the overflow }
  lOverflowBand.RTLayout.Top := FLastYPos;
  lOverflowBand.RTLayout.Left := FLastXPos;
  UpdateSpaceRemaining(lOverflowBand);
end;

procedure TFPReportLayouter.PrepareHeaderFooter(APage: TFPReportCustomPage);
begin
  FTitle:=TFPReportCustomTitleBand(APage.FindBand(TFPReportCustomTitleBand));
  FSummary:=TFPReportCustomSummaryBand(APage.FindBand(TFPReportCustomSummaryBand));
  FPageHeader := TFPReportCustomPageHeaderBand(APage.FindBand(TFPReportCustomPageHeaderBand));
  FPageFooter := APage.FindBand(TFPReportCustomPageFooterBand);
  FColumnFooter := TFPReportColumnFooterBand(aPage.FindBand(TFPReportColumnFooterBand));
  FColumnHeader := TFPReportColumnHeaderBand(aPage.FindBand(TFPReportColumnHeaderBand));
end;

function TLoopData.GetSubLoop(AIndex : Integer): TLoopData;
begin
  Result:=TLoopData(FSubloops[AIndex]);
end;

function TLoopData.GetSubLoopCount: Integer;
begin
  Result:=FSubloops.Count
end;

procedure TLoopData.CollectDataBands(APage: TFPReportCustomPage);

Var
  I : Integer;
  lBand : TFPReportCustomBandWithData;

begin
  // find Bands of interest for this data loop
  for I := 0 to aPage.BandCount-1 do
    if APage.Bands[i] is TFPReportCustomBandWithData then
      begin
      lBand := aPage.Bands[I] as TFPReportCustomBandWithData;
      // Connected to our data loop
      if (lBand.Data=FData) then
        begin
        if (lBand is TFPReportCustomDataBand) then
          begin
          { Do a quick sanity check - we may not have more than one master data band }
          if (FDataBand<>Nil) then
            ReportError(SErrMultipleDataBands);
          FDataBand:=TFPReportCustomDataBand(lBand);
          end
        else if (lBand is TFPReportCustomDataHeaderBand) then
          FDataHeader:=lBand as TFPReportCustomDataHeaderBand
        else if (lBand is TFPReportCustomDataFooterBand) then
          FDataFooter:=lBand as TFPReportCustomDataFooterBand
        end
      end;
end;


procedure TLoopData.PrepareGroups(APage: TFPReportCustomPage);

var
  I: Integer;
  lGroup: TFPReportCustomGroupHeaderBand;
  lGrp: TFPReportCustomGroupHeaderBand;

begin
  FGroupHeaderList.Clear;
  FGroupFooterList.Clear;
  lGroup := nil;
  // search for lowest group (without child group)
  I:=0;
  While (lGroup=Nil) and (I<APage.BandCount) do
    begin
    if APage.Bands[I] is TFPReportCustomGroupHeaderBand then
      begin
      lGroup:=APage.Bands[I] as TFPReportCustomGroupHeaderBand;
      if (LGroup.Data<>FData) or Assigned(lGroup.ChildGroupHeader) then
        lGroup := Nil;
      end;
    Inc(i);
    end;
  if not Assigned(lGroup) then
    exit;
  // populate list from lowest to highest group level
  while Assigned(lGroup) do
    begin
    lGroup.ResetGroupConditionValues;
    FGroupHeaderList.Add(lGroup);
    lGroup := lGroup.ParentGroupHeader;
    end;
  // Now collect group footers
  for i:=FGroupHeaderList.Count-1 downto 0 do
    begin
    lGrp:=TFPReportCustomGroupHeaderBand(FGroupHeaderList[i]);
    if Assigned(lGrp.GroupFooter) and (lGrp.FIntermediateFooter <> []) then
      FGroupFooterList.Add(lGrp.GroupFooter);
    end;
end;

procedure TFPReportLayouter.PrepareBottomStackedFooters;
var
  lPageFooterYPos: TFPReportUnits;
  i: Integer;
  lRTBand: TFPReportCustomBand;
begin
  // handle bpStackAtBottom from bottom to top
  lPageFooterYPos := (FRTPage.RTLayout.Top + FRTPage.RTLayout.Height);
  for i := FRTBottomStackedFooterList.Count-1 downto 0 do
  begin
    lRTBand := FRTBottomStackedFooterList[i];
    if lRTBand.RTLayout.Top = -1 then
    begin
      lPageFooterYPos := lPageFooterYPos - lRTBand.RTLayout.Height;
      lRTBand.RTLayout.Top := lPageFooterYPos;
    end;
  end;
end;

function TFPReportLayouter.GetPerDesignerPageCount(Index : Cardinal): Cardinal;
begin
  Result:=Report.FPerDesignerPageCount[Index];
end;

procedure TFPReportLayouter.InitRTCurPageIdx;

begin
  Report.FRTCurPageIdx:=-1;
end;

function TFPReportLayouter.GetRTCurPageIdx: Integer;
begin
  Result:=Report.FRTCurPageIdx;
end;

function TFPReportLayouter.GetRTIsLastColumn: Boolean;
begin
  Result := FRTCurrentColumn = FRTPage.ColumnCount;
end;

function TFPReportLayouter.GetRTObjects: TFPList;
begin
  Result:=Report.RTObjects;
end;

procedure TFPReportLayouter.SetGetPerDesignerPageCount(Index : Cardinal; AValue: Cardinal);
begin
  Report.FPerDesignerPageCount[Index]:=AValue;
end;


procedure TFPReportLayouter.SetPageNumberPerDesignerPage(aValue: Integer);
begin
  Report.FPageNumberPerDesignerPage:=aValue;
end;

function TFPReportLayouter.FooterSpaceNeeded: TFPReportUnits;

var
  i: Integer;
  lFooter, lBand: TFPReportCustomBand;
  lValue: TFPReportUnits;
  lGrpFooter: TFPReportCustomGroupFooterBand;

begin
  //Write('FooterSpaceNeeded: ');
  Result := 0;
  for i:=0 to CurrentLoop.FGroupFooterList.Count-1 do
  begin
    lValue := 0;
    lFooter:=CurrentLoop.FGroupFooterList[i];
    lGrpFooter := TFPReportCustomGroupFooterBand(lFooter);
    if not lGrpFooter.GroupHeader.NeedsIntermediateFooter(RTIsLastColumn) or
    lGrpFooter.FDoNotConsiderInFooterSpaceNeeded then
      Continue;
    Report.FRTInIntermediateGroupFooter := True;
    while Assigned(lFooter) do
    begin
      lBand:=TFPReportCustomBand(lFooter.PrepareObject(FRTPage));
      try
        lBand.BeforePrint;
        if lBand.EvaluateVisibility then begin
          lValue := lValue + lBand.RTLayout.Height;
          Result := Result + lBand.RTLayout.Height;
        end;
        lFooter := lFooter.ChildBand;
      finally
        lBand.Page.RemoveChild(lBand);
        lBand.Free;
      end;
    end;
    Report.FRTInIntermediateGroupFooter := False;
  end;
  if Assigned(FPageFooter) then
  begin
    lValue := 0;
    lFooter := FPageFooter;
    while Assigned(lFooter) do
    begin
      lBand:=TFPReportCustomBand(lFooter.PrepareObject(FRTPage));
      try
        lBand.BeforePrint;
        if lBand.EvaluateVisibility then begin
          lValue := lValue + lBand.RTLayout.Height;
          Result := Result + lBand.RTLayout.Height;
        end;
        lFooter := lFooter.ChildBand;
      finally
        lBand.Page.RemoveChild(lBand);
        lBand.Free;
      end;
    end;
    //write('PF:',FormatFloat('#,##0.0', lValue),' ');
  end;
  if Assigned(FColumnFooter) then
  begin
    lValue := 0;
    lFooter := FColumnFooter;
    while Assigned(lFooter) do
    begin
      lBand:=TFPReportCustomBand(lFooter.PrepareObject(FRTPage));
      try
        lBand.BeforePrint;
        if lBand.EvaluateVisibility then begin
          lValue := lValue + lBand.RTLayout.Height;
          Result := Result + lBand.RTLayout.Height;
        end;
        lFooter := lFooter.ChildBand;
      finally
        lBand.Page.RemoveChild(lBand);
        lBand.Free;
      end;
    end;
    //write('CF:',FormatFloat('#,##0.0', lValue),' ');
  end;

  //Write(' = ',FormatFloat('#,##0.0', Result));
end;

procedure TFPReportLayouter.SetPageCount(aCount: Integer);
begin
  Report.FPageCount:=aCount;
end;

function TFPReportLayouter.GetPage(AIndex: integer): TFPReportCustomPage;
begin
  Result:=Report.Pages[AIndex];
end;

function TFPReportLayouter.GetRTCurDsgnPageIdx: Integer;
begin
  Result := Report.FRTCurDsgnPageIdx;
end;

function TFPReportLayouter.GetPageNumberPerDesignerPage: Integer;
begin
  Result:=Report.FPageNumberPerDesignerPage;
end;

procedure TFPReportLayouter.SetRTCurDsgnPageIdx(pPageIdx: Integer);
begin
  Report.FRTCurDsgnPageIdx := pPageIdx;
end;

function TFPReportLayouter.IsFirstPass: Boolean;
begin
  Result:=Report.IsFirstPass;
end;

function TFPReportLayouter.TwoPass: Boolean;
begin
  Result:=Report.TwoPass;
end;

procedure TFPReportLayouter.IncPageNumber;
begin
  Inc(Report.FPageNumber);
end;

procedure TFPReportLayouter.InitPageNumber;
begin
  Report.FPageNumber:=0;
end;

procedure TFPReportLayouter.IncPageNumberPerDesignerPage;

begin
  inc(Report.FPageNumberPerDesignerPage);
end;

procedure TFPReportLayouter.StartNewPage;
begin
  if Assigned(FRTPage) then
    EndPage;
  { new page }
  if Assigned(CurrentLoop.FDataBand) then
    Report.ClearDataBandLastTextValues(CurrentLoop.FDataBand);
  FSpaceLeft := Pages[RTCurDsgnPageIdx].Layout.Height; // original designer page

  FRTPage := TFPReportCustomPage(Pages[RTCurDsgnPageIdx].PrepareObject(nil));
  Report.FRTCurPageIdx := Report.RTObjects.Add(FRTPage);

  FLastYPos := FRTPage.RTLayout.Top;
  FLastXPos := FRTPage.RTLayout.Left;
  FRTCurrentColumn := 0;
  IncPageNumber;
  if IsFirstPass then
    PerDesignerPageCount[RTCurDsgnPageIdx] := PerDesignerPageCount[RTCurDsgnPageIdx] + 1;
  if (PageNumberPerDesignerPage = 1) then
    { remove title band }
    FTitle := nil;
  IncPageNumberPerDesignerPage;
  if HandleHeaderBands then
    exit;
  FColumnYStartPos := FLastYPos;
  StartNewColumn;
  FNewPage := False;
end;

procedure TFPReportLayouter.ShowDataBand;

Var
  lBand : TFPReportCustomDataBand;

begin
  lBand:=CurrentLoop.FDataBand;
  if ShowBandWithChilds(lBand) then
    FColumnDetailsPrinted := True;
  CurrentLoop.SetDetailsPrinted;
  ShowDetailBands;
end;

procedure TFPReportLayouter.ShowDataHeaderBand;
begin
  if Not assigned(CurrentLoop.FDataHeader) then
    exit;
  if CurrentLoop.FDataHeaderPrinted then
    Exit; // nothing further to do
  if ShowBandWithChilds(CurrentLoop.FDataHeader) then
    CurrentLoop.FDataHeaderPrinted := True;
end;

Function TFPReportLayouter.FindHeader (aPage : TFPReportCustomPage; aData : TFPReportData) : TFPReportCustomDataHeaderBand;

Var
  i : integer;

begin
  I:=0;
  Result:=Nil;
  While (Result=Nil) and (I<aPage.BandCount) do
    begin
    if (aPage.Bands[i] is TFPReportCustomDataHeaderBand) and
       (TFPReportCustomDataHeaderBand(aPage.Bands[i]).Data=aData) then
       Result:=TFPReportCustomDataHeaderBand(aPage.Bands[i]);
    Inc(i);
    end;
end;

function TFPReportLayouter.GetCurrentLoop: TLoopData;
begin
  if (FCurrentLoop=Nil) then
    Raise EReportError.Create('Internal error: Loop requested and none available!');
  Result:=FCurrentLoop;
end;

Function TFPReportLayouter.FindFooter (aPage : TFPReportCustomPage; aData : TFPReportData) : TFPReportCustomDataFooterBand;

Var
  i : integer;

begin
  I:=0;
  Result:=Nil;
  While (Result=Nil) and (I<aPage.BandCount) do
    begin
    if (aPage.Bands[i] is TFPReportCustomDataFooterBand) and
       (TFPReportCustomDataFooterBand(aPage.Bands[i]).Data=aData) then
       Result:=TFPReportCustomDataFooterBand(aPage.Bands[i]);
    Inc(i);
    end;
end;

Procedure DumpData(lData : TFPReportData);

Var
  I : Integer;
begin
  Write('Fields [',Ldata.Name,']');
  For i:=0 to lData.FieldCount-1 do
    Write(', ',lData.FieldNames[i],': ',SafeVariant(LData.GetFieldValue(lData.FieldNames[i])));
  Writeln();
end;

procedure TFPReportLayouter.ShowDetailBands;

var
  LD : TLoopData;
  lPage : TFPReportCustomPage;
  lData: TFPReportData;
  i : integer;

begin
  lPage:=Pages[RTCurDsgnPageIdx];
  { process Detail bands }
  for i := 0 to CurrentLoop.SubLoopCount-1 do
    begin
    LD:=CurrentLoop.SubLoop[I];
    PushLoop(LD);
    try
      LD.FDataHeaderPrinted:=False;
      LD.ResetGroups;
      lData := LD.Data;
      {$ifdef gdebug}
      Writeln('Detail loop ',ldata.Name);
      Writeln('-----------');
      {$endif}
      if not lData.IsOpened then
        lData.Open;
      lData.First;
      Report.InitAggregates(lPage,lData);
      while not lData.EOF do
        begin
        {$ifdef gdebug}
        Writeln('detail Record');
        {$endif}
        // DumpData(lData);
        PrepareRecord(lData);
        Report.UpdateAggregates(lPage,lData);
        if FNewPage then
          StartNewPage;
        ShowDataHeaderBand;
        HandleGroupBands;
        ShowDataBand;
        lData.Next;
        end;  { while not lData.EOF }
      {$ifdef gdebug}
      Writeln('detail Records done');
      {$endif}
      Report.DoneAggregates(lPage,lData);
      PrepareRecord(lData);
      CheckNewOrOverFlow;
      HandleLastGroupFooters;
      // only print if we actually had data
      if (lData.RecNo > 1) then
        begin
        if Assigned(CurrentLoop.FDataFooter) then
          ShowBandWithChilds(CurrentLoop.FDataFooter);
        end;
    Finally
      lData.Close;
      PopLoop;
    end;
    end;
end;

procedure TFPReportLayouter.HandleGroupBands;

Var
  I, lHighestGroupWithChange: Integer;
  lGroup: TFPReportCustomGroupHeaderBand;
  lGroupChanged: Boolean;

begin
  if CurrentLoop.FGroupHeaderList.Count=0 then
    exit;
  lGroupChanged := false;
  lHighestGroupWithChange := 0;
  // process footers
  For I := 0 to CurrentLoop.FGroupHeaderList.Count - 1 do
  begin
    lGroup := TFPReportCustomGroupHeaderBand(CurrentLoop.FGroupHeaderList[I]);
    if lGroup.GroupChanged then
    begin
      lGroupChanged := true;
      lHighestGroupWithChange := I;
      if Assigned(lGroup.GroupFooter) and
      not lGroup.IsInitialGroupChange then
      begin
        lGroup.GroupFooter.FDoNotConsiderInFooterSpaceNeeded := True;
        ShowGroupFooterBand(lGroup.GroupFooter);
      end;
    end
    else
      break;
  end;

  if not lGroupChanged then
    exit;

  For I := lHighestGroupWithChange downto 0 do
  begin
    lGroup := TFPReportCustomGroupHeaderBand(CurrentLoop.FGroupHeaderList[I]);
    if Assigned(lGroup.GroupFooter) then
      lGroup.GroupFooter.FDoNotConsiderInFooterSpaceNeeded := False;
  end;

  if Assigned(CurrentLoop.FDataBand) then
    Report.ClearDataBandLastTextValues(CurrentLoop.FDataBand);

  // process headers
  For I := lHighestGroupWithChange downto 0 do
  begin
    lGroup := TFPReportCustomGroupHeaderBand(CurrentLoop.FGroupHeaderList[I]);
    ShowGroupHeaderBand(lGroup, True);
  end;
end;

procedure TFPReportLayouter.HandleLastGroupFooters;

Var
  I: Integer;
  lBand: TFPReportCustomGroupFooterBand;

begin
  for I := 0 to CurrentLoop.FGroupHeaderList.Count-1 do
    begin
    lBand := TFPReportCustomGroupHeaderBand(CurrentLoop.FGroupHeaderList[I]).GroupFooter;
    if Assigned(lBand) then
      begin
      lBand.FDoNotConsiderInFooterSpaceNeeded := True;
      ShowGroupFooterBand(lBand);
      end;
    end;
end;


procedure TFPReportLayouter.PushLoop(aLoop: TLoopData);
begin
  if Assigned(aLoop) then
    ALoop.ParentLoop:=FCurrentLoop;
  FCurrentLoop:=aLoop;
end;

function TFPReportLayouter.PopLoop: TLoopData;
begin
  Result:=FCurrentLoop;
  if Assigned(Result) then
    FCurrentLoop:=Result.ParentLoop
  else
    FCurrentLoop:=Nil;
end;

procedure TFPReportLayouter.InitBandList(aPage: TFPReportCustomPage);

begin
  // Create a list of bands that need to be printed as page headers
  PrepareHeaderFooter(aPage);
  // Don't do this if there is no data...
  if Assigned(CurrentLoop.Data) then
    begin
    // Create a list of group headers/footers
    CurrentLoop.PrepareGroups(aPage);
    // Create a list of group headers
    CurrentLoop.CollectDataBands(aPage);
    // Recurse
    CurrentLoop.PrepareBandSubLoops(aPage);
    end;
end;

procedure TFPReportLayouter.CheckNewOrOverFlow(CheckMulticolumn: Boolean = True);
begin
  if CheckMulticolumn and FNewColumn then
    StartNewColumn;
  if FNewPage then
    StartNewPage;
  { handle overflowed bands. Remove from old page, add to new page }
  if FOverflowed then
    HandleOverflowed;
end;

procedure TFPReportLayouter.InitReportData(aMainData : TFPReportData);

Var
  I : Integer;
  oData : TFPReportData;

begin
  if Assigned(aMainData) and not aMainData.IsOpened then
    aMainData.Open;
  For I:=0 to Report.ReportData.Count-1 do
    begin
    oData:=Report.ReportData[i].Data;
    if Assigned(oData) and (oData<>aMainData) and (not odata.IsOpened) then
      oData.Open;
    end;
  if Assigned(aMainData) then
    aMainData.First;
  if IsFirstPass then
    begin
    Report.InitializeExpressionVariables;
    Report.InitializeAggregates(True);
    end
  else
    Report.InitializeAggregates(False);
end;

procedure TFPReportLayouter.RunDataLoop(aPage: TFPReportCustomPage; aData: TFPReportData);

Var
  aLoop: TLoopData;

begin
  {$ifdef gdebug}
  Writeln('------------------');
  Writeln('Run loop ',IsFirstPass);
  Writeln('------------------');
  {$endif}
  Report.FLoopData:=aData;
  aLoop:=TLoopData.Create(aData);
  try
    PushLoop(aLoop);
    InitBandList(aPage);
    Report.InitAggregates(aPage,aData);
    if Not Assigned(aData) then
      StartNewPage
    else
      begin
      If not aData.IsOpened then
        aData.Open;
      aData.First;
      while not aData.EOF do
        begin
        {$ifdef gdebug}
        Writeln('*** Page Record');
        {$endif}
        // DumpData(aPageData);
        PrepareRecord(aData);
        Report.UpdateAggregates(aPage,aData);
        if FNewPage then
          StartNewPage;
        ShowDataHeaderBand;
        HandleGroupBands;
        ShowDataBand;
        aData.Next;
        end;
      {$ifdef gdebug}
      Writeln('*** Page Record done');
      {$endif}
      Report.DoneAggregates(aPage,aData);
      PrepareRecord(aData);
      end;
    CheckNewOrOverFlow(True);
    HandleLastGroupFooters;
    // only print if we actually had data
    if assigned(aData) and (aData.RecNo > 1) then
      begin
      if Assigned(CurrentLoop.FDataFooter) then
        ShowBandWithChilds(CurrentLoop.FDataFooter);
      end;
    if Assigned(aData) and not (TwoPass and IsFirstPass) then
      aData.Close;
    HandleReportSummaryBands;
    EndPage;
    if (PopLoop<>aLoop) then
      Raise EReportError.Create('Internal consistency error: Popped loop is not current loop!');
  finally
    FreeAndNil(aLoop)
  end;
end;

procedure TFPReportLayouter.PrepareRecord(aData : TFPReportData);

begin
//  Report.Variables.PrepareExpressionValues(aData);
  if CurrentLoop.FGroupHeaderList.Count > 0 then
    TFPReportCustomGroupHeaderBand(CurrentLoop.FGroupHeaderList[0]).EvaluateGroupCondition;
end;


procedure TFPReportLayouter.InitPass(aPassIdx: Integer);

begin
  Report.FIsFirstPass := (aPassIdx = 1);
  Report.EmptyRTObjects;
  InitRTCurPageIdx;
  FOverflowed := False;
  InitPageNumber;
  FRTPage := nil;
end;

procedure TFPReportLayouter.InitDesignPage(aPageIdx: integer; APage : TFPReportCustomPage);

begin
  RTCurDsgnPageIdx:=aPageIdx;
  FRTIsMultiColumn := aPage.IsMultiColumn;
  PageNumberPerDesignerPage := 0;
  FRTCurrentColumn := 1;
  FNewColumn := False;
  FNewPage:=True;
  FRTPage := nil;
  FColumnHeader := nil;
  FColumnFooter := nil;
  FSummary:=Nil;
  FTitle:=Nil;
  FPageHeader:=Nil;
  FPageFooter:=Nil;
end;

procedure TFPReportLayouter.HandleReportSummaryBands;

begin
  if not Assigned(FSummary) then
    exit;
  if (FSummary.StartNewPage) or (FSummary.Layout.Height > (FSpaceLeft - FooterSpaceNeeded)) then
    StartNewPage;
  ShowBandWithChilds(FSummary);
end;

procedure TFPReportLayouter.ShowGroupHeaderBand(aBand: TFPReportCustomGroupHeaderBand; aCheckStartOnNewSection: Boolean);
begin
  if aCheckStartOnNewSection and
  aBand.GroupChanged and
  FColumnDetailsPrinted then
    if aBand.StartOnNewSection = rsPage then
      StartNewPage
    else if aBand.StartOnNewSection = rsColumn then
    begin
      if RTIsLastColumn then
        StartNewPage
      else begin
        EndColumn;
        StartNewColumn;
      end;
    end;
  ShowBandWithChilds(aBand);
end;

procedure TFPReportLayouter.ShowGroupFooterBand(aBand: TFPReportCustomGroupFooterBand);
begin
  ShowBandWithChilds(aBand);
  FColumnDetailsPrinted := True;
end;

procedure TFPReportLayouter.RemoveBandsFromPage(aList: TBandList);

Var
  i : Integer;
  B : TFPReportCustomBand;

begin
  for i := 0 to aList.Count-1 do
  begin
    B := aList[i];
    { remove band from current page }
    B.Page.RemoveChild(B);
    { correct LastYPos }
    FLastYPos := FLastYPos - B.RTLayout.Height;
    { free mem }
    B.Free;
  end;
  aList.Clear;
end;

function TFPReportLayouter.HandleOverflowedBands(aHandledBands: TBandList; aBand: TFPReportCustomBand;
  var aRTBand: TFPReportCustomBand): TOverFlowAction;

var
  lGrp, lToMoveGrp: TFPReportCustomGroupHeaderBand;
  i: Integer;

begin
  Result := oaNone;
  lToMoveGrp := nil;
  if FNewColumn or FOverflowed then
  begin
    if aBand is TFPReportCustomGroupFooterBand then
      TFPReportCustomGroupFooterBand(aBand).FDoNotConsiderInFooterSpaceNeeded := False;
    if aBand.KeepTogetherWithChildren then
    begin
      { complete band with child bands move to next column/page }
      Result:=oaBandWithChilds;
      { remove all overflowed bands and start again on new column/page }
      RemoveBandsFromPage(aHandledBands);
      //writeln('   complete move to next column/page');
      { notify band }
      aBand.MovedToNextPageWithChilds;
      { if OverflowWithFirstDataBand is set,
        also move header to next column/page }
      if CurrentLoop.FGroupHeaderList.Count > 0 then
      begin
        { when data band overflows use start with lowest gropup header }
        if aBand is TFPReportCustomDataBand and
        not Assigned(TFPReportCustomDataBand(aband).MasterBand) then
          lToMoveGrp := TFPReportCustomGroupHeaderBand(CurrentLoop.FGroupHeaderList[0])
        { when group header overflows use start with parent group header }
        else if aBand is TFPReportCustomGroupHeaderBand then
          lToMoveGrp := TFPReportCustomGroupHeaderBand(aBand).ParentGroupHeader;
        { remove group headers if OverflowWithFirstDataBand and no details printed }
        lGrp := lToMoveGrp;
        while Assigned(lGrp) do
        begin
          if lGrp.NeedsOverflowWithFirstDataBand(RTIsLastColumn) and not lGrp.FDetailsPrinted then
          begin
            //writeln('      also group ', lGrp.GroupCondition);
            { remove RT bands of group from current page }
            RemoveBandsFromPage(lGrp.FRTBands);
            { mark group as completed -> no intermediate footer needed on
              current column/page and no reprinted header needed
              as following group header will start on new column/page     }
            lGrp.FNeedsIntermediateFooter := False;
            lGrp.FNeedsReprintedHeader := False;
          end;
          { next is parent group header }
          lGrp := lGrp.ParentGroupHeader;
        end;
      end;
    end
    else
    begin
      { only current band moves to next column/page }
      Result:=oaSingleBand;
      { remove band from current page }
      aRTBand.Page.RemoveChild(aRTBand);
      { correct LastYPos }
      FLastYPos := FLastYPos - aRTBand.RTLayout.Height;
      { free mem }
      aRTBand.Free;
      //writeln('   part move to next page');
    end;

    { set state variable }
    aBand.FIsOverflowed := True;
    { handle new column }
    if FNewColumn and not RTIsLastColumn then
    begin
      EndColumn;
      StartNewColumn;
    end;
    { do not handle overflow in CheckNewOrOverFlow }
    FOverflowed := False;
  end;

  if FNewPage then
    StartNewPage;

  { we are on a new page/column now }
  { reprint moved headers only (not overflowed ones) }
  if Assigned(lToMoveGrp) then
  begin
    for i := CurrentLoop.FGroupHeaderList.Count-1 downto 0 do
    begin
      lGrp := TFPReportCustomGroupHeaderBand(CurrentLoop.FGroupHeaderList[i]);
      if lGrp.NeedsOverflowWithFirstDataBand(RTIsLastColumn) and not lGrp.FDetailsPrinted then
      begin
        //writeln('      reprint group header', lGrp.GroupCondition);
        ShowBandWithChilds(lGrp);
      end;
      { lToMovedGRoup is last group to reprint }
      if lGrp = lToMoveGrp then
        break;
    end;
  end;
end;


function TFPReportLayouter.ShowBandWithChilds(aBand: TFPReportCustomBand): Boolean;

Var
  lHandledBands: TBandList;
  overFlowAction : TOverFlowAction;
  lBand,lRTBand: TFPReportCustomBand;

begin
  //write('ShowBandWithChilds: ', aBand.ClassName);
  //if aBand is TFPReportCustomGroupHeaderBand then
  //  write(': ',TFPReportCustomGroupHeaderBand(aBand).GroupCondition)
  //else if aBand is TFPReportCustomGroupFooterBand then
  //  write(': ',TFPReportCustomGroupFooterBand(aBand).GroupHeader.GroupCondition);
  //writeln(': Space = ', FormatFloat('#,##0.0', FSpaceLeft));
  Result := False;
  if not Assigned(aBand) then
    Exit;

  lHandledBands := TBandList.Create;
  try
    lBand := aBand;
    while Assigned(lBand) do
    begin
      overflowAction:=oaNone;
      if (lBand=aBand) then
        aBand.BeforePrintWithChilds;
      lRTBand := CommonRuntimeBandProcessing(lBand);
      if lRTBand<>Nil then
        begin
        Result := True;
        lHandledBands.Add(lRTBand);
        UpdateSpaceRemaining(lRTBand, aBand.NeedsUpdateYPos);
        if NoSpaceRemaining then
          overFlowAction := HandleOverflowedBands(lHandledBands, aBand, lRTBand);
        if (overFlowAction=oaNone) then
        begin
          aBand.AfterPrintBand(lRTBand);
          if aBand.BandPosition = bpStackAtBottom then
          begin
            { mark for HandleBottomStackedFooters }
            lRTBand.RTLayout.Top := -1;
            FRTBottomStackedFooterList.Add(lRTBand);
          end
        end
        else
          Report.FRTIsOverflowed := True;
        end;
      // Decide what band to process next.
      Case overFlowAction of
      oaBandWithChilds:
        lBand:=aBand; // Restart from the main band.
      oaSingleBand:
        ; // do nothing, same band again
      oaNone:
        // Next band
        lBand := lBand.ChildBand;
      end;
    end; { while Assigned(lBand) }
    if (aBand is TFPReportCustomGroupHeaderBand) and
    not Report.FRTInRepeatedGroupHeader and
    (lHandledBands.Count > 0) then
      TFPReportCustomGroupHeaderBand(aBand).StoreRTBands(lHandledBands);

    aBand.FIsOverflowed := False;
    Report.FRTIsOverflowed := False;
    aBand.AfterPrintWithChilds;
  finally
    lHandledBands.Free;
  end;
end;

procedure TFPReportLayouter.DoExecute;

Var
  lPageIdx : Integer;
  lPassIdx : Integer;
  lPage : TFPReportCustomPage;
  lPageData,iData, lMainData: TFPReportData;
  aPassCount : Integer;

begin
  if Report.TwoPass then
    aPassCount := 2
  else
    aPassCount := 1;
  // Pass loop
  for lPassIdx := 1 to aPassCount do
    begin
    InitPass(lPassIdx);
    // Design page loop
    for lPageIdx := 0 to Report.PageCount-1 do
      begin
      lPage:=Pages[lPageIdx];
      lPageData:=Pages[lPageIdx].Data;
      lMainData:=lPage.FindMainDataloop;
      if (lPageData<>Nil) and (lPageData=lMainData) then
        lPageData:=Nil;
      iData:=lPageData;
      If iData=Nil then
        iData:=lMainData;
      InitDesignPage(lPageIdx,lPage);
      InitReportData(IData);
      if lPassIdx=1 then
        Report.CacheMemoExpressions(lPage);
      if (lPageData=Nil) then
        RunDataLoop(lPage,lMainData)
      else
        begin
        lPageData.First;
        While not lPageData.EOF do
          begin
          RunDataLoop(lPage,lMainData);
          lPageData.Next;
          FNewPage:=True;
          end;
        end;
      end;
    SetPageCount(RTObjects.Count);
    end;
  // DoProcessPass only substitutes cPageCountMarker by FPageCount.
  // It is pointless to do so if we're doing 2 passes anyway
  if Report.UsePageCountMarker then
    Report.DoProcessTwoPass;
end;

constructor TFPReportLayouter.Create(AOwner : TComponent);
begin
  Inherited Create(AOwner);
end;

destructor TFPReportLayouter.Destroy;
begin
  inherited Destroy;
end;

procedure TFPReportLayouter.Execute(aReport: TFPCustomReport);

begin
  FRTBottomStackedFooterList := nil;
  FmyReport:=AReport;
  try
    FRTBottomStackedFooterList := TBandList.Create;
    DoExecute;
  finally
    FreeAndNil(FRTBottomStackedFooterList);
    FMyReport:=Nil; // Don't free :)
  end;
end;


{ A function borrowed from fpGUI Toolkit. }
function fpgDarker(const AColor: TFPReportColor; APercent: Byte): TFPReportColor;

  function GetRed(const c: TFPReportColor): Byte; inline;
  begin
    Result := (c shr 16) and $FF;
  end;

  function GetGreen(const c: TFPReportColor): Byte; inline;
  begin
    Result := (c shr 8) and $FF;
  end;

  function GetBlue(const c: TFPReportColor): Byte; inline;
  begin
    Result := c and $FF;
  end;

var
  r, g, b: Byte;
begin
  r := GetRed(AColor);
  g := GetGreen(AColor);
  b := GetBlue(AColor);

  r := Round(r*APercent/100);
  g := Round(g*APercent/100);
  b := Round(b*APercent/100);

  Result := b or (g shl 8) or (r shl 16);
end;


{ Defines colors that can be used by a report designer or demos. }
procedure SetupBandRectColors;
var
  i: TFPReportBandType;
begin
  for i := Low(TFPReportBandType) to High(TFPReportBandType) do
    DefaultBandRectangleColors[i] := fpgDarker(DefaultBandColors[i], 70);
end;

Procedure RegisterStandardReportClasses;

begin
  TFPReportTitleBand.RegisterElement;
  TFPReportSummaryBand.RegisterElement;
  TFPReportGroupHeaderBand.RegisterElement;
  TFPReportGroupFooterBand.RegisterElement;
  TFPReportDataBand.RegisterElement;
  TFPReportChildBand.RegisterElement;
  TFPReportPageHeaderBand.RegisterElement;
  TFPReportPageFooterBand.RegisterElement;
  TFPReportDataHeaderBand.RegisterElement;
  TFPReportDataFooterBand.RegisterElement;
  TFPReportColumnHeaderBand.RegisterElement;
  TFPReportColumnFooterBand.RegisterElement;
  TFPReportMemo.RegisterElement.FStandard:=True;
  TFPReportImage.RegisterElement.FStandard:=True;
  TFPReportCheckbox.RegisterElement.FStandard:=True;
  TFPReportShape.RegisterElement.FStandard:=True;
  TFPReportPage.RegisterElement.FStandard:=True;
end;

initialization
  uElementFactory := nil;
  RegisterStandardReportClasses;
  SetupBandRectColors;

finalization
  DoneReporting;
  uElementFactory.Free;
  EM.Free;
end.
