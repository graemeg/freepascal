{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998 by Berczi Gabor

    Views and view-related functions for the IDE

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit FPViews;

{$i globdir.inc}

interface

uses
  Dos,Objects,Drivers,
  FVConsts,
  Views,Menus,Dialogs,App,Gadgets,Tabs,
  ASCIITAB,
  WEditor,WCEdit,
  WUtils,WHelp,WHlpView,WViews,WANSI,
  Comphook,
{$ifndef NODEBUG}
  { Needed here for CORE_ADDR definition }
  {$ifdef GDBMI}
    gdbmiint,
  {$else GDBMI}
    gdbint,
  {$endif GDBMI}
{$endif NODEBUG}
  FPConst,FPUsrScr;

type
    TEditor = TCodeEditor;
    PEditor = PCodeEditor;

    PStoreCollection = ^TStoreCollection;
    TStoreCollection = object(TStringCollection)
      function Add(const S: string): PString;
    end;

    PIntegerLine = ^TIntegerLine;
    TIntegerLine = object(TInputLine)
      constructor Init(var Bounds: TRect; AMin, AMax: longint);
    end;

    PFPHeapView = ^TFPHeapView;
    TFPHeapView = object(THeapView)
      constructor Init(var Bounds: TRect);
      constructor InitKb(var Bounds: TRect);
      procedure   HandleEvent(var Event: TEvent); virtual;
    end;

    PFPClockView = ^TFPClockView;
    TFPClockView = object(TClockView)
      constructor Init(var Bounds: TRect);
      procedure   HandleEvent(var Event: TEvent); virtual;
      function    GetPalette: PPalette; virtual;
    end;

    PFPWindow = ^TFPWindow;
    TFPWindow = object(TWindow)
      AutoNumber: boolean;
      procedure   HandleEvent(var Event: TEvent); virtual;
      procedure   SetState(AState: Word; Enable: Boolean); virtual;
      procedure   UpdateCommands; virtual;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
      procedure   Update; virtual;
      procedure   SelectInDebugSession;
    end;

    PFPHelpViewer = ^TFPHelpViewer;
    TFPHelpViewer = object(THelpViewer)
      function    GetLocalMenu: PMenu; virtual;
      function    GetCommandTarget: PView; virtual;
    end;

    PFPHelpWindow = ^TFPHelpWindow;
    TFPHelpWindow = object(THelpWindow)
      constructor Init(var Bounds: TRect; ATitle: TTitleStr; ASourceFileID: word; AContext: THelpCtx; ANumber: Integer);
      destructor  Done;virtual;
      procedure   InitHelpView; virtual;
      procedure   Show; {virtual;}
      procedure   Hide; {virtual;}
      procedure   HandleEvent(var Event: TEvent); virtual;
      function    GetPalette: PPalette; virtual;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
    end;

    PTextScroller = ^TTextScroller;
    TTextScroller = object(TStaticText)
      TopLine: integer;
      Speed  : integer;
      Lines  : PUnsortedStringCollection;
      constructor Init(var Bounds: TRect; ASpeed: integer; AText: PUnsortedStringCollection);
      function    GetLineCount: integer; virtual;
      function    GetLine(I: integer): string; virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
      procedure   Update; virtual;
      procedure   Reset; virtual;
      procedure   Scroll; virtual;
      procedure   Draw; virtual;
      destructor  Done; virtual;
    private
      LastTT: longint;
    end;

    TAlign = (alLeft,alCenter,alRight);

    PFPToolTip = ^TFPToolTip;
    TFPToolTip = object(TView)
      constructor Init(var Bounds: TRect; const AText: string; AAlign: TAlign);
      procedure   Draw; virtual;
      function    GetText: string;
      procedure   SetText(const AText: string);
      function    GetAlign: TAlign;
      procedure   SetAlign(AAlign: TAlign);
      function    GetPalette: PPalette; virtual;
      destructor  Done; virtual;
    private
      Text: PString;
      Align: TAlign;
    end;

    PSourceEditor = ^TSourceEditor;
    TSourceEditor = object(TFileEditor)
      CompileStamp : longint;
      CodeCompleteTip: PFPToolTip;
      constructor Init(var Bounds: TRect; AHScrollBar, AVScrollBar:
          PScrollBar; AIndicator: PIndicator;const AFileName: string);
{$ifndef NODEBUG}
    private
      ShouldHandleBreakpoints : boolean;
{$endif NODEBUG}
    public
      { Syntax highlight }
      function  IsReservedWord(const S: string): boolean; virtual;
      function  IsAsmReservedWord(const S: string): boolean; virtual;
      function  GetSpecSymbolCount(SpecClass: TSpecSymbolClass): integer; virtual;
      function  GetSpecSymbol(SpecClass: TSpecSymbolClass; Index: integer): pstring; virtual;
      { CodeTemplates }
      function    TranslateCodeTemplate(var Shortcut: string; ALines: PUnsortedStringCollection): boolean; virtual;
      function    SelectCodeTemplate(var ShortCut: string): boolean; virtual;
      { CodeComplete }
      function    CompleteCodeWord(const WordS: string; var Text: string): boolean; virtual;
      procedure   FindMatchingDelimiter(ScanForward: boolean); virtual;
      procedure   SetCodeCompleteWord(const S: string); virtual;
      procedure   AlignCodeCompleteTip;
      procedure   HandleEvent(var Event: TEvent); virtual;
{$ifdef DebugUndo}
      procedure   DumpUndo;
      procedure   UndoAll;
      procedure   RedoAll;
{$endif DebugUndo}
      function    Valid(Command: Word): Boolean;virtual;
      function    GetLocalMenu: PMenu; virtual;
      function    GetCommandTarget: PView; virtual;
      function    CreateLocalMenuView(var Bounds: TRect; M: PMenu): PMenuPopup; virtual;
      procedure   ModifiedChanged; virtual;
      procedure   InsertOptions; virtual;
      procedure   PushInfo(Const st : string);virtual;
      procedure   PopInfo;virtual;
      procedure   DeleteLine(I: sw_integer); virtual;
      procedure   BackSpace; virtual;
      procedure   DelChar; virtual;
      procedure   DelSelect; virtual;
      function    InsertNewLine : Sw_integer;virtual;
      function    InsertLine(LineNo: sw_integer; const S: string): PCustomLine; virtual;
      procedure   AddLine(const S: string); virtual;
    end;

    PSourceWindow = ^TSourceWindow;
    TSourceWindow = object(TFPWindow)
      Editor    : PSourceEditor;
      Indicator : PIndicator;
      NoNameCount : longint;
      constructor Init(var Bounds: TRect; AFileName: string);
      function    GetTitle(MaxSize: sw_Integer): TTitleStr; virtual;
      procedure   SetTitle(ATitle: string); virtual;
      procedure   UpdateTitle; virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
      procedure   Update; virtual;
      procedure   UpdateCommands; virtual;
      function    GetPalette: PPalette; virtual;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
      procedure   Close; virtual;
      destructor  Done; virtual;
    end;

{$ifndef NODEBUG}
    PGDBSourceEditor = ^TGDBSourceEditor;
    TGDBSourceEditor = object(TSourceEditor)
      function   InsertNewLine : Sw_integer;virtual;
      function   Valid(Command: Word): Boolean; virtual;
      procedure  AddLine(const S: string); virtual;
      procedure  AddErrorLine(const S: string); virtual;
      { Syntax highlight }
      function  IsReservedWord(const S: string): boolean; virtual;
    private
      Silent,
      AutoRepeat,
      IgnoreStringAtEnd : boolean;
      LastCommand : String;
      end;

    PGDBWindow = ^TGDBWindow;
    TGDBWindow = object(TFPWindow)
      Editor    : PGDBSourceEditor;
      Indicator : PIndicator;
      constructor Init(var Bounds: TRect);
      procedure   HandleEvent(var Event: TEvent); virtual;
      procedure   WriteText(Buf : pchar;IsError : boolean);
      procedure   WriteString(Const S : string);
      procedure   WriteErrorString(Const S : string);
      procedure   WriteOutputText(Buf : pchar);
      procedure   WriteErrorText(Buf : pchar);
      function    GetPalette: PPalette;virtual;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
      procedure   UpdateCommands; virtual;
      destructor  Done; virtual;
    end;

    PDisasLine = ^TDisasLine;
    TDisasLine = object(TLine)
      address : CORE_ADDR;{ should be target size of address for cross debuggers }
    end;

    PDisasLineCollection = ^TDisasLineCollection;
    TDisasLineCollection = object(TLineCollection)
      function  At(Index: sw_Integer): PDisasLine;
    end;

    PDisassemblyEditor = ^TDisassemblyEditor;
    TDisassemblyEditor = object(TSourceEditor)
      CurrentSource : String;
      CurrentLine : longint;
      constructor Init(var Bounds: TRect; AHScrollBar, AVScrollBar:
          PScrollBar; AIndicator: PIndicator;const AFileName: string);
      procedure  ReleaseSource;
      destructor Done;virtual;
      procedure  AddSourceLine(const AFileName: string;line : longint); virtual;
      procedure  AddAssemblyLine(const S: string;AAddress : CORE_ADDR); virtual;
      function   GetCurrentLine(address : CORE_ADDR) : PDisasLine;
      private
        Source : PSourceWindow;
        OwnsSource : Boolean;
        DisasLines : PDisasLineCollection;
        MinAddress,MaxAddress : CORE_ADDR;
        CurL : PDisasLine;
      end;

    PDisassemblyWindow = ^TDisassemblyWindow;
    TDisassemblyWindow = object(TFPWindow)
      Editor    : PDisassemblyEditor;
      Indicator : PIndicator;
      constructor Init(var Bounds: TRect);
      procedure   LoadFunction(Const FuncName : string);
      procedure   LoadAddress(Addr : CORE_ADDR);
      function    ProcessPChar(p : pchar) : boolean;
      procedure   HandleEvent(var Event: TEvent); virtual;
      procedure   WriteSourceString(Const S : string;line : longint);
      procedure   WriteDisassemblyString(Const S : string;address : CORE_ADDR);
      procedure   SetCurAddress(address : CORE_ADDR);
      procedure   UpdateCommands; virtual;
      function    GetPalette: PPalette;virtual;
      destructor  Done; virtual;
    end;
{$endif NODEBUG}

    PClipboardWindow = ^TClipboardWindow;
    TClipboardWindow = object(TSourceWindow)
      constructor Init;
      procedure   Close; virtual;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
      destructor  Done; virtual;
    end;

    PMessageItem = ^TMessageItem;
    TMessageItem = object(TObject)
      TClass    : longint;
      Text      : PString;
      Module    : PString;
      Row,Col   : sw_integer;
      constructor Init(AClass: longint; const AText: string; AModule: PString; ARow, ACol: sw_integer);
      function    GetText(MaxLen: Sw_integer): string; virtual;
      procedure   Selected; virtual;
      function    GetModuleName: string; virtual;
      destructor  Done; virtual;
    end;

    PMessageListBox = ^TMessageListBox;
    TMessageListBox = object(THSListBox)
      Transparent : boolean;
      NoSelection : boolean;
      MaxWidth    : Sw_integer;
      ModuleNames : PStoreCollection;
      constructor Init(var Bounds: TRect; AHScrollBar, AVScrollBar: PScrollBar);
      procedure   SetState(AState: Word; Enable: Boolean); virtual;
      procedure   AddItem(P: PMessageItem); virtual;
      function    AddModuleName(const Name: string): PString; virtual;
      function    GetText(Item,MaxLen: Sw_Integer): String; virtual;
      procedure   Clear; virtual;
      procedure   TrackSource; virtual;
      procedure   GotoSource; virtual;
      procedure   Draw; virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
      function    GetLocalMenu: PMenu; virtual;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
      destructor  Done; virtual;
    end;


    PFPDlgWindow = ^TFPDlgWindow;
    TFPDlgWindow = object(TDlgWindow)
      procedure   HandleEvent(var Event: TEvent); virtual;
    end;

(*
    PTabItem = ^TTabItem;
    TTabItem = record
      Next : PTabItem;
      View : PView;
      Dis  : boolean;
    end;

    PTabDef = ^TTabDef;
    TTabDef = record
      Next     : PTabDef;
      Name     : PString;
      Items    : PTabItem;
      DefItem  : PView;
      ShortCut : char;
    end;

    PTab = ^TTab;
    TTab = object(TGroup)
      TabDefs   : PTabDef;
      ActiveDef : integer;
      DefCount  : word;
      constructor Init(var Bounds: TRect; ATabDef: PTabDef);
      function    AtTab(Index: integer): PTabDef; virtual;
      procedure   SelectTab(Index: integer); virtual;
      function    TabCount: integer;
      procedure   SelectNextTab(Forwards: boolean);
      function    Valid(Command: Word): Boolean; virtual;
      procedure   ChangeBounds(var Bounds: TRect); virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
      function    GetPalette: PPalette; virtual;
      procedure   Draw; virtual;
      procedure   SetState(AState: Word; Enable: Boolean); virtual;
      destructor  Done; virtual;
    private
      InDraw: boolean;
    end;
*)

    PScreenView = ^TScreenView;
    TScreenView = object(TScroller)
      Screen: PScreen;
      constructor Init(var Bounds: TRect; AHScrollBar, AVScrollBar: PScrollBar;
                    AScreen: PScreen);
      procedure   Draw; virtual;
      procedure   Update; virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
    end;

    PScreenWindow = ^TScreenWindow;
    TScreenWindow = object(TFPWindow)
      ScreenView : PScreenView;
      constructor Init(AScreen: PScreen; ANumber: integer);
      destructor  Done; virtual;
    end;

    PFPAboutDialog = ^TFPAboutDialog;
    TFPAboutDialog = object(TCenterDialog)
      constructor Init;
      procedure   ToggleInfo;
      procedure   HandleEvent(var Event: TEvent); virtual;
    private
      Scroller: PTextScroller;
      TitleST : PStaticText;
    end;

    PFPASCIIChart = ^TFPASCIIChart;
    TFPASCIIChart = object(TASCIIChart)
      constructor Init;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
      procedure   HandleEvent(var Event: TEvent); virtual;
      destructor  Done; virtual;
    end;

    PVideoModeListBox = ^TVideoModeListBox;
    TVideoModeListBox = object(TDropDownListBox)
      function    GetText(Item: pointer; MaxLen: sw_integer): string; virtual;
    end;

    PFPDesktop = ^TFPDesktop;
    TFPDesktop = object(TDesktop)
      constructor Init(var Bounds: TRect);
      procedure   InitBackground; virtual;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
    end;

    PFPMemo = ^TFPMemo;
    TFPMemo = object(TCodeEditor)
      constructor Init(var Bounds: TRect; AHScrollBar, AVScrollBar:
                    PScrollBar; AIndicator: PIndicator);
      function    IsReservedWord(const S: string): boolean; virtual;
      function    GetSpecSymbolCount(SpecClass: TSpecSymbolClass): integer; virtual;
      function    GetSpecSymbol(SpecClass: TSpecSymbolClass; Index: integer): pstring; virtual;
      function    GetPalette: PPalette; virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
    end;

    PFPCodeMemo = ^TFPCodeMemo;
    TFPCodeMemo = object(TFPMemo)
      constructor Init(var Bounds: TRect; AHScrollBar, AVScrollBar:
                    PScrollBar; AIndicator: PIndicator);
      function    IsReservedWord(const S: string): boolean; virtual;
      function    GetSpecSymbolCount(SpecClass: TSpecSymbolClass): integer; virtual;
      function    GetSpecSymbol(SpecClass: TSpecSymbolClass; Index: integer): pstring; virtual;
    end;

function  SearchFreeWindowNo: integer;

function IsWindow(P: PView): boolean;
function IsThereAnyEditor: boolean;
function IsThereAnyWindow: boolean;
function IsThereAnyVisibleWindow: boolean;
function IsThereAnyNumberedWindow: boolean;
function FirstEditorWindow: PSourceWindow;
function EditorWindowFile(const Name : String): PSourceWindow;
procedure AskToReloadAllModifiedFiles;

{$ifndef NODEBUG}
function InDisassemblyWindow :boolean;
{$endif NODEBUG}

function  NewTabItem(AView: PView; ANext: PTabItem): PTabItem;
procedure DisposeTabItem(P: PTabItem);
function  NewTabDef(AName: string; ADefItem: PView; AItems: PTabItem; ANext: PTabDef): PTabDef;
procedure DisposeTabDef(P: PTabDef);

function GetEditorCurWord(Editor: PEditor; ValidSpecChars: TCharSet): string;
procedure InitReservedWords;
procedure DoneReservedWords;
function GetReservedWordCount: integer;
function GetReservedWord(Index: integer): string;
function GetAsmReservedWordCount: integer;
function GetAsmReservedWord(Index: integer): string;

procedure TranslateMouseClick(View: PView; var Event: TEvent);

function GetNextEditorBounds(var Bounds: TRect): boolean;
function OpenEditorWindow(Bounds: PRect; FileName: string; CurX,CurY: sw_integer): PSourceWindow;
function IOpenEditorWindow(Bounds: PRect; FileName: string; CurX,CurY: sw_integer; ShowIt: boolean): PSourceWindow;
function LastSourceEditor : PSourceWindow;
function SearchOnDesktop(FileName : string;tryexts:boolean) : PSourceWindow;
function TryToOpenFile(Bounds: PRect; FileName: string; CurX,CurY: sw_integer;tryexts: boolean): PSourceWindow;
function TryToOpenFileMulti(Bounds: PRect; FileName: string; CurX,CurY: sw_integer;tryexts: boolean): PSourceWindow;
function ITryToOpenFile(Bounds: PRect; FileName: string; CurX,CurY: sw_integer;tryexts, ShowIt,
         ForceNewWindow:boolean): PSourceWindow;
function LocateSourceFile(const FileName: string; tryexts: boolean): string;

function SearchWindow(const Title: string): PWindow;

function StartEditor(Editor: PCodeEditor; FileName: string): boolean;

{$ifdef VESA}
procedure InitVESAScreenModes;
procedure DoneVESAScreenModes;
{$endif}

procedure NoDebugger;

const
      SourceCmds  : TCommandSet =
        ([cmSave,cmSaveAs,cmCompile,cmHide,cmDoReload]);
      EditorCmds  : TCommandSet =
        ([cmPrint,cmFind,cmReplace,cmSearchAgain,cmJumpLine,cmHelpTopicSearch,cmSelectAll,cmUnselect]);
      CompileCmds : TCommandSet =
        ([cmMake,cmBuild,cmRun]);

      CalcClipboard   : extended = 0;

      OpenFileName    : string = '';
      OpenFileLastExt : string[12] = '*.pas';
      NewEditorOpened : boolean = false;

var  MsgParms : array[1..10] of
         record
           case byte of
             0 : (Ptr : pointer);
             1 : (Long: longint);
         end;

const menu_key_common_copy_borland   = 'Ctrl+Ins';
      menu_key_common_copy_microsoft = 'Ctrl+C';

      menu_key_edit_undo             = 'Alt+BkSp';
      menu_key_edit_cut_borland      = 'Shift+Del';
      menu_key_edit_copy_borland     = menu_key_common_copy_borland;
      menu_key_edit_paste_borland    = 'Shift+Ins';
      menu_key_edit_cut_microsoft    = 'Ctrl+X';
      menu_key_edit_copy_microsoft   = menu_key_common_copy_microsoft;
      menu_key_edit_paste_microsoft  = 'Ctrl+V';
      menu_key_edit_all_borland      = '';
      menu_key_edit_clear            = 'Ctrl+Del';

      menu_key_common_helpindex      = 'Shift+F1';
      menu_key_common_topicsearch    = 'Ctrl+F1';
      menu_key_common_prevtopic      = 'Alt+F1';

      menu_key_help_helpindex= menu_key_common_helpindex;
      menu_key_help_topicsearch = menu_key_common_topicsearch;
      menu_key_help_prevtopic= menu_key_common_prevtopic;

      menu_key_hlplocal_index = menu_key_common_helpindex;
      menu_key_hlplocal_topicsearch = menu_key_common_topicsearch;
      menu_key_hlplocal_prevtopic = menu_key_common_prevtopic;
      menu_key_hlplocal_copy_borland = menu_key_common_copy_borland;
      menu_key_hlplocal_copy_microsoft = menu_key_common_copy_microsoft;

{Configurable keys.}
const menu_key_edit_cut:string[63]=menu_key_edit_cut_borland;
      menu_key_edit_copy:string[63]=menu_key_edit_copy_borland;
      menu_key_edit_paste:string[63]=menu_key_edit_paste_borland;
      menu_key_edit_all:string[63]=menu_key_edit_all_borland;
      menu_key_hlplocal_copy:string[63]=menu_key_hlplocal_copy_borland;
      cut_key:word=kbShiftDel;
      copy_key:word=kbCtrlIns;
      paste_key:word=kbShiftIns;
      all_key:word=kbNoKey;

procedure RegisterFPViews;

implementation

uses
  Video,Strings,Keyboard,Validate,
  globtype,Tokens,Version,
  systems,cpubase,
  itcpugas,
  {$if defined(I386) or defined(x64_86)}
     rax86,
  {$endif}
  {$ifdef m68k}
     ag68kgas,
  {$endif}
{$ifdef USE_EXTERNAL_COMPILER}
   fpintf, { superseeds version_string of version unit }
{$endif USE_EXTERNAL_COMPILER}
  {$ifdef VESA}Vesa,{$endif}
  FPSwitch,FPSymbol,FPDebug,FPVars,FPUtils,FPCompil,FPHelp,
  FPTools,FPIDE,FPCodTmp,FPCodCmp;

const
  RSourceEditor: TStreamRec = (
     ObjType: 1500;
     VmtLink: Ofs(TypeOf(TSourceEditor)^);
     Load:    @TSourceEditor.Load;
     Store:   @TSourceEditor.Store
  );
  RSourceWindow: TStreamRec = (
     ObjType: 1501;
     VmtLink: Ofs(TypeOf(TSourceWindow)^);
     Load:    @TSourceWindow.Load;
     Store:   @TSourceWindow.Store
  );
  RFPHelpViewer: TStreamRec = (
     ObjType: 1502;
     VmtLink: Ofs(TypeOf(TFPHelpViewer)^);
     Load:    @TFPHelpViewer.Load;
     Store:   @TFPHelpViewer.Store
  );
  RFPHelpWindow: TStreamRec = (
     ObjType: 1503;
     VmtLink: Ofs(TypeOf(TFPHelpWindow)^);
     Load:    @TFPHelpWindow.Load;
     Store:   @TFPHelpWindow.Store
  );
  RClipboardWindow: TStreamRec = (
     ObjType: 1504;
     VmtLink: Ofs(TypeOf(TClipboardWindow)^);
     Load:    @TClipboardWindow.Load;
     Store:   @TClipboardWindow.Store
  );
  RMessageListBox: TStreamRec = (
     ObjType: 1505;
     VmtLink: Ofs(TypeOf(TMessageListBox)^);
     Load:    @TMessageListBox.Load;
     Store:   @TMessageListBox.Store
  );
  RFPDesktop: TStreamRec = (
     ObjType: 1506;
     VmtLink: Ofs(TypeOf(TFPDesktop)^);
     Load:    @TFPDesktop.Load;
     Store:   @TFPDesktop.Store
  );

  RFPASCIIChart: TStreamRec = (
     ObjType: 1509;
     VmtLink: Ofs(TypeOf(TFPASCIIChart)^);
     Load:    @TFPASCIIChart.Load;
     Store:   @TFPASCIIChart.Store
  );
  RFPDlgWindow: TStreamRec = (
     ObjType: 1511;
     VmtLink: Ofs(TypeOf(TFPDlgWindow)^);
     Load:    @TFPDlgWindow.Load;
     Store:   @TFPDlgWindow.Store
  );
{$ifndef NODEBUG}
  RGDBWindow: TStreamRec = (
     ObjType: 1508;
     VmtLink: Ofs(TypeOf(TGDBWindow)^);
     Load:    @TGDBWindow.Load;
     Store:   @TGDBWindow.Store
  );
  RGDBSourceEditor: TStreamRec = (
     ObjType: 1507;
     VmtLink: Ofs(TypeOf(TGDBSourceEditor)^);
     Load:    @TGDBSourceEditor.Load;
     Store:   @TGDBSourceEditor.Store
  );
  RDisassemblyEditor: TStreamRec = (
     ObjType: 1512;
     VmtLink: Ofs(TypeOf(TDisassemblyEditor)^);
     Load:    @TDisassemblyEditor.Load;
     Store:   @TDisassemblyEditor.Store
  );
  RDisassemblyWindow: TStreamRec = (
     ObjType: 1513;
     VmtLink: Ofs(TypeOf(TDisassemblyWindow)^);
     Load:    @TDisassemblyWindow.Load;
     Store:   @TDisassemblyWindow.Store
  );
{$endif NODEBUG}
const
  GlobalNoNameCount : integer = 0;
var
  ReservedWords  : array[1..ReservedWordMaxLen] of PStringCollection;
  AsmReservedWords  : array[1..ReservedWordMaxLen] of PStringCollection;

{$ifdef useresstrings}
resourcestring
{$else}
const
{$endif}
      { Source editor local menu items }
      menu_srclocal_openfileatcursor = 'Open ~f~ile at cursor';
      menu_srclocal_browseatcursor = '~B~rowse symbol at cursor';
      menu_srclocal_topicsearch = 'Topic ~s~earch';
      menu_srclocal_options = '~O~ptions...';
      menu_srclocal_reload = '~R~eload modified file';

      { Help viewer local menu items }
      menu_hlplocal_debug = 'Debug infos';
      menu_hlplocal_contents = '~C~ontents';
      menu_hlplocal_index = '~I~ndex';
      menu_hlplocal_topicsearch = '~T~opic search';
      menu_hlplocal_prevtopic = '~P~revious topic';
      menu_hlplocal_copy = '~C~opy';

      { Messages local menu items }
      menu_msglocal_clear = '~C~lear';
      menu_msglocal_gotosource = '~G~oto source';
      menu_msglocal_tracksource = '~T~rack source';

      menu_edit_cut          = 'Cu~t~';
      menu_edit_copy         = '~C~opy';
      menu_edit_paste        = '~P~aste';
      menu_edit_clear        = 'C~l~ear';

      msg_errorreadingfile = 'Error reading file %s';
      msg_loadingfile = 'Loading %s';
      msg_storingfile = 'Storing %s';
      msg_closingfile = 'Closing %s';

      dialog_gdbwindow = 'GDB window';
      dialog_disaswindow = 'Disassembly window';
      dialog_clipboard = 'Clipboard';
      dialog_userscreen = 'User screen';
      dialog_about = 'About';
      label_about_compilerversion = 'Compiler Version';
      label_about_debugger = 'Debugger';

      menu_msglocal_saveas = 'Save ~a~s';
      msg_openingsourcefile = 'Opening source file... (%s)';
      msg_readingfileineditor = 'Reading %s into editor...';
      msg_nodebuggersupportavailable = 'No debugger support available.';

{****************************************************************************
                                TStoreCollection
****************************************************************************}

function TStoreCollection.Add(const S: string): PString;
var P: PString;
    Index: Sw_integer;
begin
  if S='' then P:=nil else
  if Search(@S,Index) then P:=At(Index) else
    begin
      P:=NewStr(S);
      Insert(P);
    end;
  Add:=P;
end;


function IsThereAnyEditor: boolean;
function EditorWindow(P: PView): boolean;
begin
  EditorWindow:=(P^.HelpCtx=hcSourceWindow);
end;
begin
  IsThereAnyEditor:=Desktop^.FirstThat(@EditorWindow)<>nil;
end;

procedure AskToReloadAllModifiedFiles;
  procedure EditorWindowModifiedOnDisk(P: PView);
begin
  if (P^.HelpCtx=hcSourceWindow) then
    PSourceWindow(P)^.Editor^.ReloadFile;
end;
begin
  Desktop^.ForEach(@EditorWindowModifiedOnDisk);
end;

function IsThereAnyHelpWindow: boolean;
begin
  IsThereAnyHelpWindow:=(HelpWindow<>nil) and (HelpWindow^.GetState(sfVisible));
end;

function IsThereAnyNumberedWindow: boolean;
var _Is: boolean;
begin
  _Is:=Message(Desktop,evBroadcast,cmSearchWindow,nil)<>nil;
  _Is:=_Is or ( (ClipboardWindow<>nil) and ClipboardWindow^.GetState(sfVisible));
  IsThereAnyNumberedWindow:=_Is;
end;

function IsWindow(P: PView): boolean;
var OK: boolean;
begin
  OK:=false;
  if (P^.HelpCtx=hcSourceWindow) or
     (P^.HelpCtx=hcHelpWindow) or
     (P^.HelpCtx=hcClipboardWindow) or
     (P^.HelpCtx=hcCalcWindow) or
     (P^.HelpCtx=hcInfoWindow) or
     (P^.HelpCtx=hcBrowserWindow) or
     (P^.HelpCtx=hcMessagesWindow) or
     (P^.HelpCtx=hcCompilerMessagesWindow) or
     (P^.HelpCtx=hcGDBWindow) or
     (P^.HelpCtx=hcdisassemblyWindow) or
     (P^.HelpCtx=hcWatchesWindow) or
     (P^.HelpCtx=hcRegistersWindow) or
     (P^.HelpCtx=hcFPURegisters) or
     (P^.HelpCtx=hcVectorRegisters) or
     (P^.HelpCtx=hcStackWindow) or
     (P^.HelpCtx=hcBreakpointListWindow) or
     (P^.HelpCtx=hcASCIITableWindow)
   then
     OK:=true;
   IsWindow:=OK;
end;

function IsThereAnyWindow: boolean;
function CheckIt(P: PView): boolean;
begin
  CheckIt:=IsWindow(P);
end;
begin
  IsThereAnyWindow:=Desktop^.FirstThat(@CheckIt)<>nil;
end;

function IsThereAnyVisibleWindow: boolean;
function CheckIt(P: PView): boolean;
begin
  CheckIt:=IsWindow(P) and P^.GetState(sfVisible);
end;
begin
  IsThereAnyVisibleWindow:=Desktop^.FirstThat(@CheckIt)<>nil;
end;

function FirstEditorWindow: PSourceWindow;
function EditorWindow(P: PView): boolean;
begin
  EditorWindow:=(P^.HelpCtx=hcSourceWindow);
end;
begin
  FirstEditorWindow:=pointer(Desktop^.FirstThat(@EditorWindow));
end;

function EditorWindowFile(const Name : String): PSourceWindow;
var
  SName : string;

  function EditorWindow(P: PView): boolean;
  begin
    EditorWindow:=(TypeOf(P^)=TypeOf(TSourceWindow)) and
                  (FixFileName(PSourceWindow(P)^.Editor^.FileName)=SName);
  end;

begin
  SName:=FixFileName(FExpand(Name));
  EditorWindowFile:=pointer(Desktop^.FirstThat(@EditorWindow));
end;


{$ifndef NODEBUG}
function InDisassemblyWindow :boolean;
var
  PW : PWindow;

function CheckIt(P: PView): boolean;
begin
  CheckIt:=IsWindow(P) and P^.GetState(sfVisible) and
     (P^.HelpCtx <> hcWatchesWindow) and
     (P^.HelpCtx <> hcStackWindow) and
     (P^.HelpCtx <> hcRegistersWindow) and
     (P^.HelpCtx <> hcVectorRegisters) and
     (P^.HelpCtx <> hcFPURegisters);
end;
begin
  PW:=PWindow(Desktop^.FirstThat(@CheckIt));
  InDisassemblyWindow:=Assigned(PW) and
    (TypeOf(PW^)=TypeOf(TDisassemblyWindow));
end;
{$endif NODEBUG}


function GetEditorCurWord(Editor: PEditor; ValidSpecChars: TCharSet): string;
var S: string;
    PS,PE: byte;
function Trim(S: string): string;
const TrimChars : set of char = [#0,#9,' ',#255];
begin
  while (length(S)>0) and (S[1] in TrimChars) do Delete(S,1,1);
  while (length(S)>0) and (S[length(S)] in TrimChars) do Delete(S,length(S),1);
  Trim:=S;
end;
const AlphaNum : set of char = ['A'..'Z','0'..'9','_'];
begin
  with Editor^ do
  begin
    S:=GetDisplayText(CurPos.Y);
    PS:=CurPos.X; while (PS>0) and (Upcase(S[PS]) in AlphaNum) do Dec(PS);
    PE:=CurPos.X; while (PE<length(S)) and (Upcase(S[PE+1]) in (AlphaNum+ValidSpecChars)) do Inc(PE);
    S:=Trim(copy(S,PS+1,PE-PS));
  end;
  GetEditorCurWord:=S;
end;


{*****************************************************************************
                                   Tab
*****************************************************************************}

function NewTabItem(AView: PView; ANext: PTabItem): PTabItem;
var P: PTabItem;
begin
  New(P); FillChar(P^,SizeOf(P^),0);
  P^.Next:=ANext; P^.View:=AView;
  NewTabItem:=P;
end;

procedure DisposeTabItem(P: PTabItem);
begin
  if P<>nil then
  begin
    if P^.View<>nil then Dispose(P^.View, Done);
    Dispose(P);
  end;
end;

function NewTabDef(AName: string; ADefItem: PView; AItems: PTabItem; ANext: PTabDef): PTabDef;
var P: PTabDef;
    x: byte;
begin
  New(P);
  P^.Next:=ANext; P^.Name:=NewStr(AName); P^.Items:=AItems;
  x:=pos('~',AName);
  if (x<>0) and (x<length(AName)) then P^.ShortCut:=Upcase(AName[x+1])
                                  else P^.ShortCut:=#0;
  P^.DefItem:=ADefItem;
  NewTabDef:=P;
end;

procedure DisposeTabDef(P: PTabDef);
var PI,X: PTabItem;
begin
  DisposeStr(P^.Name);
  PI:=P^.Items;
  while PI<>nil do
    begin
      X:=PI^.Next;
      DisposeTabItem(PI);
      PI:=X;
    end;
  Dispose(P);
end;


{*****************************************************************************
                               Reserved Words
*****************************************************************************}

function GetReservedWordCount: integer;
var
  Count,I: integer;
begin
  Count:=0;
  for I:=ord(Low(tToken)) to ord(High(tToken)) do
  with TokenInfo^[TToken(I)] do
     if (str<>'') and (str[1] in['A'..'Z']) and (length(str)>1) then
       Inc(Count);
  GetReservedWordCount:=Count;
end;

function GetReservedWord(Index: integer): string;
var
  Count,Idx,I: integer;
  S: string;
begin
  Idx:=-1;
  Count:=-1;
  I:=ord(Low(tToken));
  while (I<=ord(High(tToken))) and (Idx=-1) do
   with TokenInfo^[TToken(I)] do
    begin
      if (str<>'') and (str[1] in['A'..'Z']) and (length(str)>1) then
        begin
          Inc(Count);
          if Count=Index then
           Idx:=I;
        end;
      Inc(I);
    end;
  if Idx=-1 then
    S:=''
  else
    S:=TokenInfo^[TToken(Idx)].str;
  GetReservedWord:=S;
end;


{$ifdef powerpc}
  {$define USE_TasmCondFlag}
<<<<<<< HEAD
<<<<<<< HEAD
  { powerpc only has A_B prefix }
  const
    CondAsmOps = 1;
    CondAsmOpStr : array [0..CondAsmOps-1] of string[2] = ('b');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$define Use_gas_op2str}
{$endif}
{$ifdef powerpc64}
  {$define USE_TasmCondFlag}
<<<<<<< HEAD
<<<<<<< HEAD
  { powerpc64 only has A_B prefix }
  const
    CondAsmOps = 1;
    CondAsmOpStr : array [0..CondAsmOps-1] of string[2] = ('b');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$define Use_gas_op2str}
{$endif}
{$ifdef i386}
  {$define USE_TasmCond}
  {$define Use_std_op2str}
{$endif}
{$ifdef m68k}
  {$define USE_None}
  {$define Use_gas_op2str}
{$endif}

function GetAsmReservedWordCount: integer;
begin
  GetAsmReservedWordCount:=ord(lastop) - ord(firstop)
{$ifdef Use_TasmCond}
    + CondAsmOps*(ord(high(TasmCond))-ord(low(TasmCond)));
{$endif Use_TasmCond}
{$ifdef Use_TasmCondFlag}
    + CondAsmOps*(ord(high(TasmCondFlag))-ord(low(TasmCondFlag)));
{$endif Use_TasmCondFlag}
{$ifdef Use_None}
    ;
{$endif Use_None}
end;


{$define NOASM}
function GetAsmReservedWord(Index: integer): string;
var
  CondNum,CondOpNum : integer;
begin
{$ifdef m68k}
{$undef NOASM}
  if index <= ord(lastop) - ord(firstop) then
    GetAsmReservedWord:=gas_op2str[tasmop(Index+ord(firstop))]
  else
    GetAsmReservedWord:='';
  (*
    begin
      index:=index - (ord(lastop) - ord(firstop) );
      CondOpNum:= index div (ord(high(TasmCond))-ord(low(TasmCond)));
      CondNum:=index - (CondOpNum * (ord(high(TasmCond))-ord(low(TasmCond))));
      GetAsmReservedWord:=CondAsmOpStr[CondOpNum]+cond2str[TasmCond(CondNum+ord(low(TAsmCond))+1)];
    end;
    *)
{$else not m68k}
  if index <= ord(lastop) - ord(firstop) then
{$ifdef Use_gas_op2str}
    GetAsmReservedWord:=gas_op2str[tasmop(Index+ord(firstop))]
{$endif Use_gas_op2str}
{$ifdef Use_std_op2str}
    GetAsmReservedWord:=std_op2str[tasmop(Index+ord(firstop))]
{$endif Use_std_op2str}
{$ifdef Use_TASMCond}
{$undef NOASM}
  else
    begin
      index:=index - (ord(lastop) - ord(firstop) );
      CondOpNum:= index div (ord(high(TasmCond))-ord(low(TasmCond)));
      CondNum:=index - (CondOpNum * (ord(high(TasmCond))-ord(low(TasmCond))));
      GetAsmReservedWord:=CondAsmOpStr[CondOpNum]+cond2str[TasmCond(CondNum+ord(low(TAsmCond))+1)];
    end;
{$endif Use_TASMCond}
{$ifdef Use_TASMCondFlag}
{$undef NOASM}
  else
    begin
      index:=index - (ord(lastop) - ord(firstop) );
      CondOpNum:= index div (ord(high(TasmCondFlag))-ord(low(TasmCondFlag)));
      CondNum:=index - (CondOpNum * (ord(high(TasmCondFlag))-ord(low(TasmCondFlag))));
      GetAsmReservedWord:=CondAsmOpStr[CondOpNum]+AsmCondFlag2Str[TasmCondFlag(CondNum+ord(low(TAsmCondFlag))+1)];
    end;
{$endif Use_TASMCond}
{$endif not m68k}
{$ifdef NOASM}
  GetAsmReservedWord:='';
{$endif NOASM}
end;

procedure InitReservedWords;
var WordS: string;
    Idx,I,J : sw_integer;
begin
  InitTokens;
  for I:=Low(ReservedWords) to High(ReservedWords) do
    New(ReservedWords[I], Init(50,10));
  for I:=1 to GetReservedWordCount do
    begin
      WordS:=GetReservedWord(I-1); Idx:=length(WordS);
      if (Idx>=Low(ReservedWords)) and (Idx<=High(ReservedWords)) then
        ReservedWords[Idx]^.Insert(NewStr(WordS));
    end;
  for I:=Low(AsmReservedWords) to High(AsmReservedWords) do
    New(AsmReservedWords[I], Init(50,10));
  for I:=1 to GetAsmReservedWordCount do
    begin
      WordS:=UpcaseStr(GetAsmReservedWord(I-1)); Idx:=length(WordS);
      if (Idx>=Low(AsmReservedWords)) and (Idx<=High(AsmReservedWords)) then
        begin
          if not AsmReservedWords[Idx]^.Search(@WordS, J) then
            AsmReservedWords[Idx]^.Insert(NewStr(WordS));
        end;
    end;
end;

procedure DoneReservedWords;
var I: integer;
begin
  for I:=Low(ReservedWords) to High(ReservedWords) do
    if assigned(ReservedWords[I]) then
      begin
        dispose(ReservedWords[I],done);
        ReservedWords[I]:=nil;
      end;
  for I:=Low(AsmReservedWords) to High(AsmReservedWords) do
    if assigned(AsmReservedWords[I]) then
      begin
        dispose(AsmReservedWords[I],done);
        ReservedWords[I]:=nil;
      end;
  DoneTokens;
end;

function IsFPReservedWord(const S: string): boolean;
var _Is: boolean;
    Idx,Item: sw_integer;
    UpS: string;
begin
  Idx:=length(S); _Is:=false;
  if (Low(ReservedWords)<=Idx) and (Idx<=High(ReservedWords)) and
     (ReservedWords[Idx]<>nil) and (ReservedWords[Idx]^.Count<>0) then
    begin
      UpS:=UpcaseStr(S);
      _Is:=ReservedWords[Idx]^.Search(@UpS,Item);
    end;
  IsFPReservedWord:=_Is;
end;

function IsFPAsmReservedWord(S: string): boolean;
var _Is: boolean;
    Idx,Item,Len: sw_integer;
    LastC : Char;
    LastTwo : String[2];
begin
  Idx:=length(S); _Is:=false;
  if (Low(AsmReservedWords)<=Idx) and (Idx<=High(AsmReservedWords)) and
     (AsmReservedWords[Idx]<>nil) and (AsmReservedWords[Idx]^.Count<>0) then
    begin
      S:=UpcaseStr(S);
      _Is:=AsmReservedWords[Idx]^.Search(@S,Item);
{$ifdef i386}
      if not _Is and (Length(S)>1) then
        begin
          LastC:=S[Length(S)];
          if LastC in ['B','D','L','Q','S','T','V','W'] then
            begin
              Delete(S,Length(S),1);
              Dec(Idx);
              if (AsmReservedWords[Idx]<>nil) and (AsmReservedWords[Idx]^.Count<>0) then
                _Is:=AsmReservedWords[Idx]^.Search(@S,Item);
              if not _Is and (Length(S)>1) then
                begin
                  LastTwo:=S[Length(S)]+LastC;
                  if (LastTwo='BL') or
                     (LastTwo='WL') or
                     (LastTwo='BW') then
                    begin
                      Delete(S,Length(S),1);
                      Dec(Idx);
                      if (AsmReservedWords[Idx]<>nil) and (AsmReservedWords[Idx]^.Count<>0) then
                        _Is:=AsmReservedWords[Idx]^.Search(@S,Item);
                    end;
                end;
            end;
        end;
{$endif i386}
    end;
  IsFPAsmReservedWord:=_Is;
end;


{*****************************************************************************
                               SearchWindow
*****************************************************************************}

function SearchWindowWithNo(No: integer): PWindow;
var P: PWindow;
begin
  P:=Message(Desktop,evBroadcast,cmSearchWindow+No,nil);
  if pointer(P)=pointer(Desktop) then P:=nil;
  SearchWindowWithNo:=P;
end;

function SearchWindow(const Title: string): PWindow;
function Match(P: PView): boolean;
var W: PWindow;
    OK: boolean;
begin
  W:=nil;
  { we have a crash here because of the TStatusLine
    that can also have one of these values
    but is not a Window object PM }
  if P<>pointer(StatusLine) then
  if IsWindow(P) then
    W:=PWindow(P);
  OK:=(W<>nil);
  if OK then
  begin
    OK:=CompareText(W^.GetTitle(255),Title)=0;
  end;
  Match:=OK;
end;
var W: PView;
begin
  W:=Application^.FirstThat(@Match);
{    This is wrong because TStatusLine is also considered PM }
  if not Assigned(W) then W:=Desktop^.FirstThat(@Match);
  { But why do we need to check all ??
    Probably because of the ones which were not inserted into
    Desktop as the Messages view

    Exactly. Some windows are inserted directly in the Application and not
    in the Desktop. btw. Does TStatusLine.HelpCtx really change? Why?
    Only GetHelpCtx should return different values depending on the
    focused view (and it's helpctx), but TStatusLine's HelpCtx field
    shouldn't change...  Gabor

  if Assigned(W)=false then W:=Desktop^.FirstThat(@Match);}
  SearchWindow:=PWindow(W);
end;

function SearchFreeWindowNo: integer;
var No: integer;
begin
  No:=1;
  while (No<100) and (SearchWindowWithNo(No)<>nil) do
    Inc(No);
  if No=100 then No:=0;
  SearchFreeWindowNo:=No;
end;


{*****************************************************************************
                              TIntegerLine
 *****************************************************************************}

constructor TIntegerLine.Init(var Bounds: TRect; AMin, AMax: longint);
begin
  if inherited Init(Bounds, Bounds.B.X-Bounds.A.X-1)=false then
    Fail;
  Validator:=New(PRangeValidator, Init(AMin, AMax));
end;


{*****************************************************************************
                               SourceEditor
*****************************************************************************}

function SearchCoreForFileName(AFileName: string): PCodeEditorCore;
var EC: PCodeEditorCore;
function Check(P: PView): boolean;
var OK: boolean;
begin
  OK:=P^.HelpCtx=hcSourceWindow;
  if OK then
    with PSourceWindow(P)^ do
     if FixFileName(Editor^.FileName)=AFileName then
       begin
         EC:=Editor^.Core;
         OK:=true;
       end
     else
       OK:=false;
  Check:=OK;
end;
begin
  EC:=nil;
  AFileName:=FixFileName(AFileName);
  { do not use the same core for all new files }
  if AFileName<>'' then
    Desktop^.FirstThat(@Check);
  SearchCoreForFileName:=EC;
end;

constructor TSourceEditor.Init(var Bounds: TRect; AHScrollBar, AVScrollBar:
          PScrollBar; AIndicator: PIndicator;const AFileName: string);
var EC: PCodeEditorCore;
begin
  EC:=SearchCoreForFileName(AFileName);
  inherited Init(Bounds,AHScrollBar,AVScrollBar,AIndicator,EC,AFileName);
  SetStoreUndo(true);
  CompileStamp:=0;
end;

Const
  FreePascalSpecSymbolCount : array [TSpecSymbolClass] of integer =
  (
    3,{ssCommentPrefix}
    1,{ssCommentSingleLinePrefix}
    2,{ssCommentSuffix}
    1,{ssStringPrefix}
    1,{ssStringSuffix}
    1,{ssDirectivePrefix}
    1,{ssDirectiveSuffix}
    1,{ssAsmPrefix}
    1 {ssAsmSuffix}
  );

  FreePascalEmptyString : string[1] = '';
  FreePascalCommentPrefix1 : string[1] = '{';
  FreePascalCommentPrefix2 : string[2] = '(*';
  FreePascalCommentPrefix3 : string[2] = '//';
  FreePascalCommentSingleLinePrefix : string[2] = '//';
  FreePascalCommentSuffix1 : string[1] = '}';
  FreePascalCommentSuffix2 : string[2] = '*)';
  FreePascalStringPrefix : string[1] = '''';
  FreePascalStringSuffix : string[1] = '''';
  FreePascalDirectivePrefix : string[2] = '{$';
  FreePascalDirectiveSuffix : string[1] = '}';
  FreePascalAsmPrefix : string[3] = 'ASM';
  FreePascalAsmSuffix : string[3] = 'END';

function TSourceEditor.GetSpecSymbolCount(SpecClass: TSpecSymbolClass): integer;
begin
  GetSpecSymbolCount:=FreePascalSpecSymbolCount[SpecClass];
end;

function TSourceEditor.GetSpecSymbol(SpecClass: TSpecSymbolClass; Index: integer): pstring;
begin
  GetSpecSymbol:=@FreePascalEmptyString;
  case SpecClass of
    ssCommentPrefix :
      case Index of
        0 : GetSpecSymbol:=@FreePascalCommentPrefix1;
        1 : GetSpecSymbol:=@FreePascalCommentPrefix2;
        2 : GetSpecSymbol:=@FreePascalCommentPrefix3;
      end;
    ssCommentSingleLinePrefix :
      case Index of
        0 : GetSpecSymbol:=@FreePascalCommentSingleLinePrefix;
      end;
    ssCommentSuffix :
      case Index of
        0 : GetSpecSymbol:=@FreePascalCommentSuffix1;
        1 : GetSpecSymbol:=@FreePascalCommentSuffix2;
      end;
    ssStringPrefix :
      GetSpecSymbol:=@FreePascalStringPrefix;
    ssStringSuffix :
      GetSpecSymbol:=@FreePascalStringSuffix;
    { must be uppercased to avoid calling UpCaseStr in MatchesAnyAsmSymbol PM }
    ssAsmPrefix :
      GetSpecSymbol:=@FreePascalAsmPrefix;
    ssAsmSuffix :
      GetSpecSymbol:=@FreePascalAsmSuffix;
    ssDirectivePrefix :
      GetSpecSymbol:=@FreePascalDirectivePrefix;
    ssDirectiveSuffix :
      GetSpecSymbol:=@FreePascalDirectiveSuffix;
  end;
end;

function TSourceEditor.IsReservedWord(const S: string): boolean;
begin
  IsReservedWord:=IsFPReservedWord(S);
end;

function TSourceEditor.IsAsmReservedWord(const S: string): boolean;
begin
  IsAsmReservedWord:=IsFPAsmReservedWord(S);
end;

function TSourceEditor.TranslateCodeTemplate(var Shortcut: string; ALines: PUnsortedStringCollection): boolean;
begin
  TranslateCodeTemplate:=FPTranslateCodeTemplate(ShortCut,ALines);
end;

function TSourceEditor.SelectCodeTemplate(var ShortCut: string): boolean;
var D: PCodeTemplatesDialog;
    OK: boolean;
begin
  New(D, Init(true,ShortCut));
  OK:=Desktop^.ExecView(D)=cmOK;
  if OK then ShortCut:=D^.GetSelectedShortCut;
  Dispose(D, Done);
  SelectCodeTemplate:=OK;
end;

function TSourceEditor.CompleteCodeWord(const WordS: string; var Text: string): boolean;
begin
  CompleteCodeWord:=FPCompleteCodeWord(WordS,Text);
end;

procedure TSourceEditor.FindMatchingDelimiter(ScanForward: boolean);
var
  St,nextResWord : String;
  LineText,LineAttr: string;
  Res,found,addit : boolean;
  JumpPos: TPoint;
  X,Y,lexchange,curlevel,linecount : sw_integer;

   function GetLexChange(const S : string) : sw_integer;
   begin
     if (S='END') or (S='THEN') or (S='UNTIL') then
       GetLexChange:=-1
     else if (S='ASM') or (S='BEGIN') or (S='CASE') or (S='CLASS') or
        (S='IF') or (S='OBJECT') or (S='RECORD') or (S='REPEAT') then
       GetLexChange:=+1
     else
       GetLexChange:=0;
   end;

begin
  st:=UpcaseStr(GetCurrentWord);
  if st<>'' then
    Res:=IsReservedWord(St)
  else
    Res:=false;
  LexChange:=GetLexChange(St);
  if not res or (LexChange=0) or not
     IsFlagSet(efSyntaxHighlight) then
    Inherited FindMatchingDelimiter(ScanForward)
  else
    begin
      JumpPos.X:=-1; JumpPos.Y:=-1;
      Y:=CurPos.Y; X:=CurPos.X;
      found:=false;
      LineCount:=0;
      curlevel:=lexchange;
      if LexChange>0 then
        begin
          repeat
            Inc(LineCount);
            NextResWord:='';
            GetDisplayTextFormat(Y,LineText,LineAttr);
            if LineCount<>1 then X:=-1
            else if ord(LineAttr[X+1])<>coReservedWordColor then
              exit;
            repeat
              Inc(X);
              if X<length(LineText) then
               begin
                 AddIt:=ord(LineAttr[X+1])=coReservedWordColor;
                 if AddIt then
                   NextResWord:=NextResWord+UpCase(LineText[X+1]);
               end;
              if ((X=length(LineText)) or (Not AddIt)) and
                 (NextResWord<>'') and
                 IsReservedWord(NextResWord) then
                begin
                  LexChange:=GetLexChange(NextResWord);
                  CurLevel:=CurLevel+LexChange;
                  if CurLevel=0 then
                    begin
                      JumpPos.X:=X-Length(NextResWord);
                      JumpPos.Y:=Y;
                    end;
                  NextResWord:='';
                end;
            until (X>=length(LineText)) or (JumpPos.X<>-1);
            Inc(Y);
          until (Y>=GetLineCount) or (JumpPos.X<>-1);
          if (Y=GetLineCount) and (JumpPos.X=-1) then
            begin
              ErrorBox('No match',nil);
              exit;
            end;
        end
      else if (LexChange<0) then
        begin
          repeat
            Inc(LineCount);
            NextResWord:='';
            GetDisplayTextFormat(Y,LineText,LineAttr);
            if LineCount<>1 then
              X:=Length(LineText)
            else if ord(LineAttr[X+1])<>coReservedWordColor then
              exit;
            repeat
              Dec(X);
              if X>=0 then
               begin
                 AddIt:=ord(LineAttr[X+1])=coReservedWordColor;
                 if AddIt then
                   NextResWord:=UpCase(LineText[X+1])+NextResWord;
               end;
              if ((X=0) or (Not AddIt)) and
                 (NextResWord<>'') and
                 IsReservedWord(NextResWord) then
                begin
                  LexChange:=GetLexChange(NextResWord);
                  CurLevel:=CurLevel+LexChange;
                  if CurLevel=0 then
                    begin
                      if AddIt then
                        JumpPos.X:=X
                      else
                        JumpPos.X:=X+1;
                      JumpPos.Y:=Y;
                    end;
                  NextResWord:='';
                end;
            until (X<=0) or (JumpPos.X<>-1);
            Dec(Y);
          until (Y<0) or (JumpPos.X<>-1);
          if (Y<0) and (JumpPos.X=-1) then
            begin
              ErrorBox('No match',nil);
              exit;
            end;
        end;
      if JumpPos.X<>-1 then
      begin
        SetCurPtr(JumpPos.X,JumpPos.Y);
        TrackCursor(do_centre);
      end;
    end;
end;

procedure TSourceEditor.SetCodeCompleteWord(const S: string);
var R: TRect;
begin
  inherited SetCodeCompleteWord(S);
  if S='' then
    begin
      if Assigned(CodeCompleteTip) then Dispose(CodeCompleteTip, Done);
      CodeCompleteTip:=nil;
    end
  else
    begin
      R.Assign(0,0,20,1);
      if Assigned(CodeCompleteTip)=false then
        begin
          New(CodeCompleteTip, Init(R, S, alCenter));
          CodeCompleteTip^.Hide;
          Application^.Insert(CodeCompleteTip);
        end
      else
        CodeCompleteTip^.SetText(S);
      AlignCodeCompleteTip;
    end;
end;

procedure TSourceEditor.AlignCodeCompleteTip;
var P: TPoint;
    S: string;
    R: TRect;
begin
  if Assigned(CodeCompleteTip)=false then Exit;
  S:=CodeCompleteTip^.GetText;
  P.Y:=CurPos.Y;
  { determine the center of current word fragment }
  P.X:=CurPos.X-(length(GetCodeCompleteFrag) div 2);
  { calculate position for centering the complete word over/below the current }
  P.X:=P.X-(length(S) div 2);

  P.X:=P.X-Delta.X;
  P.Y:=P.Y-Delta.Y;
  MakeGlobal(P,P);
  if Assigned(CodeCompleteTip^.Owner) then
    CodeCompleteTip^.Owner^.MakeLocal(P,P);

  { ensure that the tooltip stays in screen }
  P.X:=Min(Max(0,P.X),ScreenWidth-length(S)-2-1);
  { align it vertically }
  if P.Y>round(ScreenHeight*3/4) then
    Dec(P.Y)
  else
    Inc(P.Y);
  R.Assign(P.X,P.Y,P.X+1+length(S)+1,P.Y+1);
  CodeCompleteTip^.Locate(R);
  if CodeCompleteTip^.GetState(sfVisible)=false then
    CodeCompleteTip^.Show;
end;

procedure TSourceEditor.ModifiedChanged;
begin
  inherited ModifiedChanged;
  if (@Self<>Clipboard) and GetModified then
    begin
      { global flags }
      EditorModified:=true;
      { reset compile flags as the file is
      not the same as at the compilation anymore }
      CompileStamp:=-1;
    end;
end;

procedure TSourceEditor.InsertOptions;
var C: PUnsortedStringCollection;
    Y: sw_integer;
    S: string;
begin
  Lock;
  New(C, Init(10,10));
  GetCompilerOptionLines(C);
  if C^.Count>0 then
  begin
    for Y:=0 to C^.Count-1 do
    begin
      S:=C^.At(Y)^;
      InsertLine(Y,S);
    end;
    AdjustSelectionPos(0,0,0,C^.Count);
    UpdateAttrs(0,attrAll);
    DrawLines(0);
    SetModified(true);
  end;
  Dispose(C, Done);
  UnLock;
end;

procedure  TSourceEditor.PushInfo(Const st : string);
begin
  PushStatus(st);
end;

procedure  TSourceEditor.PopInfo;
begin
  PopStatus;
end;

procedure TSourceEditor.DeleteLine(I: sw_integer);
begin
  inherited DeleteLine(I);
{$ifndef NODEBUG}
  If ShouldHandleBreakpoints then
    BreakpointsCollection^.AdaptBreakpoints(@Self,I,-1);
{$endif NODEBUG}
end;

procedure TSourceEditor.BackSpace;
{$ifndef NODEBUG}
var
  MoveBreakpointToPreviousLine,WasEnabled : boolean;
  PBStart,PBEnd : PBreakpoint;
  I : longint;
{$endif NODEBUG}
begin
{$ifdef NODEBUG}
  inherited Backspace;
{$else}
  MoveBreakpointToPreviousLine:=(CurPos.X=0) and (CurPos.Y>0);
  If MoveBreakpointToPreviousLine then
    begin
      ShouldHandleBreakpoints:=false;
      I:=CurPos.Y+1;
      PBEnd:=BreakpointsCollection^.FindBreakpointAt(@Self,I);
      PBStart:=BreakpointsCollection^.FindBreakpointAt(@Self,I-1);
    end;
  inherited Backspace;
  if MoveBreakpointToPreviousLine then
    begin
      ShouldHandleBreakpoints:=true;
      if assigned(PBEnd) then
        begin
          if assigned(PBStart) then
            begin
              if PBEnd^.state=bs_enabled then
                PBStart^.state:=bs_enabled;
              BreakpointsCollection^.Free(PBEnd);
            end
          else
            begin
              WasEnabled:=PBEnd^.state=bs_enabled;
              if WasEnabled then
                begin
                  PBEnd^.state:=bs_disabled;
                  PBEnd^.UpdateSource;
                end;
              PBEnd^.line:=I-1;
              if WasEnabled then
                begin
                  PBEnd^.state:=bs_enabled;
                  PBEnd^.UpdateSource;
                end;
            end;
        end;
      BreakpointsCollection^.AdaptBreakpoints(@Self,I,-1);
    end;
{$endif NODEBUG}
end;

function TSourceEditor.InsertNewLine : Sw_integer;
{$ifndef NODEBUG}
var
  MoveBreakpointToNextLine : boolean;
  I : longint;
{$endif NODEBUG}
begin
{$ifdef NODEBUG}
  InsertNewLine:=inherited InsertNewLine;
{$else}
  ShouldHandleBreakpoints:=false;
  MoveBreakpointToNextLine:=Cursor.x<Length(RTrim(GetDisplayText(CurPos.Y)));
  I:=CurPos.Y+1;
  InsertNewLine:=inherited InsertNewLine;
  if MoveBreakpointToNextLine then
    BreakpointsCollection^.AdaptBreakpoints(@Self,I-1,1)
  else
    BreakpointsCollection^.AdaptBreakpoints(@Self,I,1);
  ShouldHandleBreakpoints:=true;
{$endif NODEBUG}
end;

procedure TSourceEditor.DelChar;
var
  S: string;
  I,CI : sw_integer;
{$ifndef NODEBUG}
  PBStart,PBEnd : PBreakpoint;
  MoveBreakpointOneLineUp,WasEnabled : boolean;
{$endif NODEBUG}
begin
  if IsReadOnly then Exit;
  S:=GetLineText(CurPos.Y);
  I:=CurPos.Y+1;
  CI:=LinePosToCharIdx(CurPos.Y,CurPos.X);
{$ifndef NODEBUG}
  if ((CI>length(S)) or (S='')) and (CurPos.Y<GetLineCount-1) then
    begin
      MoveBreakpointOneLineUp:=true;
      ShouldHandleBreakpoints:=false;
      PBEnd:=BreakpointsCollection^.FindBreakpointAt(@Self,I+1);
      PBStart:=BreakpointsCollection^.FindBreakpointAt(@Self,I);
    end
  else
    MoveBreakpointOneLineUp:=false;
{$endif NODEBUG}
  Inherited DelChar;
{$ifndef NODEBUG}
  if MoveBreakpointOneLineUp then
    begin
      ShouldHandleBreakpoints:=true;
      if assigned(PBEnd) then
        begin
          if assigned(PBStart) then
            begin
              if PBEnd^.state=bs_enabled then
                PBStart^.state:=bs_enabled;
              BreakpointsCollection^.Free(PBEnd);
            end
          else
            begin
              WasEnabled:=PBEnd^.state=bs_enabled;
              if WasEnabled then
                begin
                  PBEnd^.state:=bs_disabled;
                  PBEnd^.UpdateSource;
                end;
              PBEnd^.line:=I;
              if WasEnabled then
                begin
                  PBEnd^.state:=bs_enabled;
                  PBEnd^.UpdateSource;
                end;
            end;
        end;
      BreakpointsCollection^.AdaptBreakpoints(@Self,I,-1);
    end;
{$endif NODEBUG}
end;

procedure TSourceEditor.DelSelect;
{$ifndef NODEBUG}
var
  MoveBreakpointToFirstLine,WasEnabled : boolean;
  PBStart,PBEnd : PBreakpoint;
  I,J : longint;
{$endif NODEBUG}
begin
{$ifdef NODEBUG}
  inherited DelSelect;
{$else}
  ShouldHandleBreakpoints:=false;
  J:=SelEnd.Y-SelStart.Y;
  MoveBreakpointToFirstLine:=J>0;
  PBEnd:=BreakpointsCollection^.FindBreakpointAt(@Self,SelEnd.Y);
  PBStart:=BreakpointsCollection^.FindBreakpointAt(@Self,SelEnd.Y);
  I:=SelStart.Y;
  inherited DelSelect;
  if MoveBreakpointToFirstLine and assigned(PBEnd) then
    begin
      If assigned(PBStart) then
        begin
          if PBEnd^.state=bs_enabled then
            PBStart^.state:=bs_enabled;
          BreakpointsCollection^.Free(PBEnd);
        end
      else
        begin
          WasEnabled:=PBEnd^.state=bs_enabled;
          if WasEnabled then
            begin
              PBEnd^.state:=bs_disabled;
              PBEnd^.UpdateSource;
            end;
          PBEnd^.line:=I;
          if WasEnabled then
            begin
              PBEnd^.state:=bs_enabled;
              PBEnd^.UpdateSource;
            end;
        end;
    end;
  BreakpointsCollection^.AdaptBreakpoints(@Self,I,-J);
  ShouldHandleBreakpoints:=true;
{$endif NODEBUG}
end;


function TSourceEditor.InsertLine(LineNo: sw_integer; const S: string): PCustomLine;
begin
  InsertLine := inherited InsertLine(LineNo,S);
{$ifndef NODEBUG}
  If ShouldHandleBreakpoints then
    BreakpointsCollection^.AdaptBreakpoints(@Self,LineNo,1);
{$endif NODEBUG}
end;

procedure TSourceEditor.AddLine(const S: string);
begin
  inherited AddLine(S);
{$ifndef NODEBUG}
  BreakpointsCollection^.AdaptBreakpoints(@Self,GetLineCount,1);
{$endif NODEBUG}
end;



function TSourceEditor.GetLocalMenu: PMenu;
var M: PMenu;
    MI: PMenuItem;
begin
  MI:=
    NewItem(menu_edit_cut,menu_key_edit_cut,cut_key,cmCut,hcCut,
    NewItem(menu_edit_copy,menu_key_edit_copy,copy_key,cmCopy,hcCopy,
    NewItem(menu_edit_paste,menu_key_edit_paste,paste_key,cmPaste,hcPaste,
    NewItem(menu_edit_clear,menu_key_edit_clear,kbCtrlDel,cmClear,hcClear,
    NewLine(
    NewItem(menu_srclocal_openfileatcursor,'',kbNoKey,cmOpenAtCursor,hcOpenAtCursor,
    NewItem(menu_srclocal_browseatcursor,'',kbNoKey,cmBrowseAtCursor,hcBrowseAtCursor,
    NewItem(menu_srclocal_topicsearch,menu_key_help_topicsearch,kbCtrlF1,cmHelpTopicSearch,hcHelpTopicSearch,
    NewLine(
    NewItem(menu_srclocal_options,'',kbNoKey,cmEditorOptions,hcEditorOptions,
    nil))))))))));
  if IsChangedOnDisk then
    MI:=NewItem(menu_srclocal_reload,'',kbNoKey,cmDoReload,hcDoReload,
      MI);
  M:=NewMenu(MI);
  GetLocalMenu:=M;
end;

function TSourceEditor.GetCommandTarget: PView;
begin
  GetCommandTarget:=@Self;
end;

function TSourceEditor.CreateLocalMenuView(var Bounds: TRect; M: PMenu): PMenuPopup;
var MV: PAdvancedMenuPopup;
begin
  New(MV, Init(Bounds,M));
  CreateLocalMenuView:=MV;
end;

{$ifdef DebugUndo}
procedure TSourceEditor.DumpUndo;
var
  i : sw_integer;
begin
  ClearToolMessages;
  AddToolCommand('UndoList Dump');
  for i:=0 to Core^.UndoList^.count-1 do
    with Core^.UndoList^.At(i)^ do
      begin
       if is_grouped_action then
         AddToolMessage('','Group '+ActionString[action]+' '+IntToStr(ActionCount)+' elementary actions',0,0)
       else
         AddToolMessage('',ActionString[action]+' '+IntToStr(StartPos.Y+1)+':'+IntToStr(StartPos.X+1)+
           ' '+IntToStr(EndPos.Y+1)+':'+IntToStr(EndPos.X+1)+' "'+GetStr(Text)+'"',0,0);
      end;
  if Core^.RedoList^.count>0 then
    AddToolCommand('RedoList Dump');
  for i:=0 to Core^.RedoList^.count-1 do
    with Core^.RedoList^.At(i)^ do
      begin
       if is_grouped_action then
         AddToolMessage('','Group '+ActionString[action]+' '+IntToStr(ActionCount)+' elementary actions',0,0)
       else
         AddToolMessage('',ActionString[action]+' '+IntToStr(StartPos.Y+1)+':'+IntToStr(StartPos.X+1)+
         ' '+IntToStr(EndPos.Y+1)+':'+IntToStr(EndPos.X+1)+' "'+GetStr(Text)+'"',0,0);
      end;
  UpdateToolMessages;
  if Assigned(MessagesWindow) then
    MessagesWindow^.Focus;
end;

procedure TSourceEditor.UndoAll;
begin
  While Core^.UndoList^.count>0 do
    Undo;
end;

procedure TSourceEditor.RedoAll;
begin
  While Core^.RedoList^.count>0 do
    Redo;
end;

{$endif DebugUndo}

function TSourceEditor.Valid(Command: Word): Boolean;
var OK: boolean;
begin
  OK:=inherited Valid(Command);
  if OK and ({(Command=cmClose) or already handled in TFileEditor.Valid PM }
     (Command=cmAskSaveAll)) then
    if IsClipboard=false then
      OK:=SaveAsk(false);
  Valid:=OK;
end;


procedure TSourceEditor.HandleEvent(var Event: TEvent);
var DontClear: boolean;
    S: string;
begin
  TranslateMouseClick(@Self,Event);
  case Event.What of
    evKeyDown :
      begin
        DontClear:=false;
        case Event.KeyCode of
          kbCtrlEnter :
            Message(@Self,evCommand,cmOpenAtCursor,nil);
        else DontClear:=true;
        end;
        if not DontClear then ClearEvent(Event);
      end;
  end;
  inherited HandleEvent(Event);
  case Event.What of
    evBroadcast :
      case Event.Command of
          cmCalculatorPaste :
            begin
              InsertText(FloatToStr(CalcClipboard,0));
              ClearEvent(Event);
            end;
      end;
    evCommand :
      begin
        DontClear:=false;
        case Event.Command of
{$ifdef DebugUndo}
          cmDumpUndo    : DumpUndo;
          cmUndoAll     : UndoAll;
          cmRedoAll     : RedoAll;
{$endif DebugUndo}
          cmDoReload : ReloadFile;
          cmBrowseAtCursor:
            begin
              S:=LowerCaseStr(GetEditorCurWord(@Self,[]));
              OpenOneSymbolBrowser(S);
            end;
          cmOpenAtCursor :
            begin
              S:=LowerCaseStr(GetEditorCurWord(@Self,['.']));
              if Pos('.',S)<>0 then
                OpenFileName:=S else
              OpenFileName:=S+'.pp'+ListSeparator+
                            S+'.pas'+ListSeparator+
                            S+'.inc';
              Message(Application,evCommand,cmOpen,nil);
            end;
          cmEditorOptions :
            Message(Application,evCommand,cmEditorOptions,@Self);
          cmHelp :
            Message(@Self,evCommand,cmHelpTopicSearch,@Self);
          cmHelpTopicSearch :
            HelpTopicSearch(@Self);
        else DontClear:=true;
        end;
        if not DontClear then ClearEvent(Event);
      end;
  end;
end;

constructor TFPHeapView.Init(var Bounds: TRect);
begin
  if inherited Init(Bounds)=false then Fail;
  Options:=Options or gfGrowHiX or gfGrowHiY;
  EventMask:=EventMask or evIdle;
  GrowMode:=gfGrowAll;
end;

constructor TFPHeapView.InitKb(var Bounds: TRect);
begin
  if inherited InitKb(Bounds)=false then Fail;
  Options:=Options or gfGrowHiX or gfGrowHiY;
  EventMask:=EventMask or evIdle;
  GrowMode:=gfGrowAll;
end;

procedure TFPHeapView.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evIdle :
      Update;
  end;
  inherited HandleEvent(Event);
end;

constructor TFPClockView.Init(var Bounds: TRect);
begin
  inherited Init(Bounds);
  EventMask:=EventMask or evIdle;
end;

procedure TFPClockView.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evIdle :
      Update;
  end;
  inherited HandleEvent(Event);
end;

function TFPClockView.GetPalette: PPalette;
const P: string[length(CFPClockView)] = CFPClockView;
begin
  GetPalette:=@P;
end;

procedure TFPWindow.SetState(AState: Word; Enable: Boolean);
var OldState: word;
begin
  OldState:=State;
  inherited SetState(AState,Enable);
  if AutoNumber then
    if (AState and (sfVisible+sfExposed))<>0 then
      if GetState(sfVisible+sfExposed) then
        begin
          if Number=0 then
            Number:=SearchFreeWindowNo;
          ReDraw;
        end
      else
        Number:=0;
  if ((AState and sfActive)<>0) and (((OldState xor State) and sfActive)<>0) then
    UpdateCommands;
end;

procedure TFPWindow.UpdateCommands;
begin
end;

procedure TFPWindow.Update;
begin
  ReDraw;
end;

procedure   TFPWindow.SelectInDebugSession;
var
  F,PrevCurrent : PView;
begin
  DeskTop^.Lock;
  PrevCurrent:=Desktop^.Current;
  F:=PrevCurrent;
  While assigned(F) and
    ((F^.HelpCtx = hcGDBWindow) or
     (F^.HelpCtx = hcdisassemblyWindow) or
     (F^.HelpCtx = hcWatchesWindow) or
     (F^.HelpCtx = hcStackWindow) or
     (F^.HelpCtx = hcRegistersWindow) or
     (F^.HelpCtx = hcVectorRegisters) or
     (F^.HelpCtx = hcFPURegisters)) do
    F:=F^.NextView;
  if F<>@Self then
    Select;
  if PrevCurrent<>F then
    Begin
      Desktop^.InsertBefore(@self,F);
      PrevCurrent^.Select;
    End;
  DeskTop^.Unlock;
end;

procedure TFPWindow.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evBroadcast :
      case Event.Command of
        cmUpdate :
          Update;
        cmSearchWindow+1..cmSearchWindow+99 :
          if (Event.Command-cmSearchWindow=Number) then
              ClearEvent(Event);
      end;
  end;
  inherited HandleEvent(Event);
end;


constructor TFPWindow.Load(var S: TStream);
begin
  inherited Load(S);
  S.Read(AutoNumber,SizeOf(AutoNumber));
end;

procedure TFPWindow.Store(var S: TStream);
begin
  inherited Store(S);
  S.Write(AutoNumber,SizeOf(AutoNumber));
end;

function TFPHelpViewer.GetLocalMenu: PMenu;
var M: PMenu;
begin
  M:=NewMenu(
{$ifdef DEBUG}
    NewItem(menu_hlplocal_debug,'',kbNoKey,cmHelpDebug,hcHelpDebug,
{$endif DEBUG}
    NewItem(menu_hlplocal_contents,'',kbNoKey,cmHelpContents,hcHelpContents,
    NewItem(menu_hlplocal_index,menu_key_hlplocal_index,kbShiftF1,cmHelpIndex,hcHelpIndex,
    NewItem(menu_hlplocal_topicsearch,menu_key_hlplocal_topicsearch,kbCtrlF1,cmHelpTopicSearch,hcHelpTopicSearch,
    NewItem(menu_hlplocal_prevtopic,menu_key_hlplocal_prevtopic,kbAltF1,cmHelpPrevTopic,hcHelpPrevTopic,
    NewLine(
    NewItem(menu_hlplocal_copy,menu_key_hlplocal_copy,copy_key,cmCopy,hcCopy,
    nil)))))))
{$ifdef DEBUG}
      )
{$endif DEBUG}
    ;
  GetLocalMenu:=M;
end;

function TFPHelpViewer.GetCommandTarget: PView;
begin
  GetCommandTarget:=Application;
end;

constructor TFPHelpWindow.Init(var Bounds: TRect; ATitle: TTitleStr; ASourceFileID: word;
  AContext: THelpCtx; ANumber: Integer);
begin
  inherited Init(Bounds,ATitle,ASourceFileID,AContext,ANumber);
  HelpCtx:=hcHelpWindow;
  HideOnClose:=true;
end;

destructor TFPHelpWindow.Done;
begin
  if HelpWindow=@Self then
    HelpWindow:=nil;
  Inherited Done;
end;

procedure TFPHelpWindow.InitHelpView;
var R: TRect;
begin
  GetExtent(R); R.Grow(-1,-1);
  HelpView:=New(PFPHelpViewer, Init(R, HSB, VSB));
  HelpView^.GrowMode:=gfGrowHiX+gfGrowHiY;
end;

procedure TFPHelpWindow.Show;
begin
  inherited Show;
  if GetState(sfVisible) and (Number=0) then
    begin
      Number:=SearchFreeWindowNo;
      ReDraw;
    end;
end;

procedure TFPHelpWindow.Hide;
begin
  inherited Hide;
  if GetState(sfVisible)=false then
    Number:=0;
end;

procedure TFPHelpWindow.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evBroadcast :
      case Event.Command of
        cmUpdate :
          ReDraw;
        cmSearchWindow+1..cmSearchWindow+99 :
          if (Event.Command-cmSearchWindow=Number) then
              ClearEvent(Event);
      end;
  end;
  inherited HandleEvent(Event);
end;

function TFPHelpWindow.GetPalette: PPalette;
const P: string[length(CIDEHelpDialog)] = CIDEHelpDialog;
begin
  GetPalette:=@P;
end;

constructor TFPHelpWindow.Load(var S: TStream);
begin
  Abstract;
end;

procedure TFPHelpWindow.Store(var S: TStream);
begin
  Abstract;
end;

constructor TSourceWindow.Init(var Bounds: TRect; AFileName: string);
var HSB,VSB: PScrollBar;
    R: TRect;
    PA : Array[1..2] of pointer;
    LoadFile: boolean;
begin
  inherited Init(Bounds,AFileName,{SearchFreeWindowNo}0);
  AutoNumber:=true;
  Options:=Options or ofTileAble;
  GetExtent(R); R.A.Y:=R.B.Y-1; R.Grow(-1,0); R.A.X:=14;
  New(HSB, Init(R)); HSB^.GrowMode:=gfGrowLoY+gfGrowHiX+gfGrowHiY; Insert(HSB);
  GetExtent(R); R.A.X:=R.B.X-1; R.Grow(0,-1);
  New(VSB, Init(R)); VSB^.GrowMode:=gfGrowLoX+gfGrowHiX+gfGrowHiY; Insert(VSB);
  GetExtent(R); R.A.X:=3; R.B.X:=14; R.A.Y:=R.B.Y-1;
  New(Indicator, Init(R));
  Indicator^.GrowMode:=gfGrowLoY+gfGrowHiY;
  Insert(Indicator);
  GetExtent(R); R.Grow(-1,-1);
  LoadFile:=(AFileName<>'') and (AFileName<>'*');
  if (AFileName='') then
    begin
      Inc(GlobalNoNameCount);
      NoNameCount:=GlobalNoNameCount;
    end
  else
    NoNameCount:=-1;
  if AFileName='*' then
    AFileName:='';
  New(Editor, Init(R, HSB, VSB, Indicator,AFileName));
  Editor^.GrowMode:=gfGrowHiX+gfGrowHiY;
  if LoadFile then
    begin
      if Editor^.LoadFile=false then
        ErrorBox(FormatStrStr(msg_errorreadingfile,AFileName),nil)
      { warn if modified, but not if modified in another
        already open window PM }
      else if Editor^.GetModified and (Editor^.Core^.GetBindingCount=1) then
        begin
          PA[1]:=@AFileName;
          Ptrint(PA[2]):={Editor^.ChangedLine}-1;
          EditorDialog(edChangedOnloading,@PA);
        end;
   end;
  Insert(Editor);
{$ifndef NODEBUG}
  If assigned(BreakpointsCollection) then
    BreakpointsCollection^.ShowBreakpoints(@Self);
{$endif NODEBUG}
  UpdateTitle;
end;

procedure TSourceWindow.UpdateTitle;
var Name: string;
    Count: sw_integer;
begin
  if Editor^.FileName<>'' then
    begin
      Name:=SmartPath(Editor^.FileName);
      Count:=Editor^.Core^.GetBindingCount;
      if Count>1 then
      begin
        Name:=Name+':'+IntToStr(Editor^.Core^.GetBindingIndex(Editor)+1);
      end;
      SetTitle(Name);
    end
  else if NoNameCount>=0 then
    begin
      SetTitle('noname'+IntToStrZ(NonameCount,2)+'.pas');
    end;
end;

function TSourceWindow.GetTitle(MaxSize: sw_Integer): TTitleStr;
begin
  GetTitle:=OptimizePath(inherited GetTitle(255),MaxSize);
end;

procedure TSourceWindow.SetTitle(ATitle: string);
begin
  if Title<>nil then DisposeStr(Title);
  Title:=NewStr(ATitle);
  Frame^.DrawView;
end;

procedure TSourceWindow.HandleEvent(var Event: TEvent);
var DontClear: boolean;
begin
  case Event.What of
    evBroadcast :
      case Event.Command of
        cmUpdate :
          Update;
        cmUpdateTitle :
          UpdateTitle;
        cmSearchWindow :
          if @Self<>ClipboardWindow then
            ClearEvent(Event);
      end;
    evCommand :
      begin
        DontClear:=false;
        case Event.Command of
          cmHide :
            Hide;
          cmSave :
            if Editor^.IsClipboard=false then
             if (Editor^.FileName='') then
              Editor^.SaveAs
             else
              Editor^.Save;
          cmSaveAs :
            if Editor^.IsClipboard=false then
              Editor^.SaveAs;
        else DontClear:=true;
        end;
        if DontClear=false then ClearEvent(Event);
      end;
  end;
  inherited HandleEvent(Event);
end;

procedure TSourceWindow.UpdateCommands;
var Active: boolean;
begin
  Active:=GetState(sfActive);
  if Editor^.IsClipboard=false then
  begin
    SetCmdState(SourceCmds+CompileCmds,Active);
    SetCmdState(EditorCmds,Active);
  end;
  SetCmdState(ToClipCmds+FromClipCmds+NulClipCmds+UndoCmd+RedoCmd,Active);
  Message(Application,evBroadcast,cmCommandSetChanged,nil);
end;

procedure TSourceWindow.Update;
begin
  ReDraw;
end;


function TSourceWindow.GetPalette: PPalette;
const P: string[length(CSourceWindow)] = CSourceWindow;
begin
  GetPalette:=@P;
end;

constructor TSourceWindow.Load(var S: TStream);
begin
  Title:=S.ReadStr;
  PushStatus(FormatStrStr(msg_loadingfile,GetStr(Title)));
  inherited Load(S);
  GetSubViewPtr(S,Indicator);
  GetSubViewPtr(S,Editor);
{$ifndef NODEBUG}
  If assigned(BreakpointsCollection) then
    BreakpointsCollection^.ShowBreakpoints(@Self);
{$endif NODEBUG}
  PopStatus;
end;

procedure TSourceWindow.Store(var S: TStream);
begin
  S.WriteStr(Title);
  PushStatus(FormatStrStr(msg_storingfile,GetStr(Title)));
  inherited Store(S);

  PutSubViewPtr(S,Indicator);
  PutSubViewPtr(S,Editor);
  PopStatus;
end;


procedure TSourceWindow.Close;
begin
  inherited Close;
end;

destructor TSourceWindow.Done;
begin
  PushStatus(FormatStrStr(msg_closingfile,GetStr(Title)));
  if not IDEApp.IsClosing then
    Message(Application,evBroadcast,cmSourceWndClosing,@Self);
  inherited Done;
  IDEApp.SourceWindowClosed;
{  if not IDEApp.IsClosing then
    Message(Application,evBroadcast,cmUpdate,@Self);}
  PopStatus;
end;


{$ifndef NODEBUG}

function TGDBSourceEditor.Valid(Command: Word): Boolean;
var OK: boolean;
begin
  OK:=TCodeEditor.Valid(Command);
  { do NOT ask for save !!
  if OK and ((Command=cmClose) or (Command=cmQuit)) then
     if IsClipboard=false then
    OK:=SaveAsk;  }
  Valid:=OK;
end;

procedure  TGDBSourceEditor.AddLine(const S: string);
begin
   if Silent or (IgnoreStringAtEnd and (S=LastCommand)) then exit;
   inherited AddLine(S);
   LimitsChanged;
end;

procedure  TGDBSourceEditor.AddErrorLine(const S: string);
begin
   if Silent then exit;
   inherited AddLine(S);
   { display like breakpoints in red }
   SetLineFlagState(GetLineCount-1,lfBreakpoint,true);
   LimitsChanged;
end;

const
  GDBReservedCount = 6;
  GDBReservedLongest = 3;
  GDBReserved : array[1..GDBReservedCount] of String[GDBReservedLongest] =
  ('gdb','b','n','s','f','bt');

function IsGDBReservedWord(const S : string) : boolean;
var
  i : longint;
begin
  for i:=1 to GDBReservedCount do
    if (S=GDBReserved[i]) then
      begin
        IsGDBReservedWord:=true;
        exit;
      end;
  IsGDBReservedWord:=false;
end;

function TGDBSourceEditor.IsReservedWord(const S: string): boolean;
begin
  IsReservedWord:=IsGDBReservedWord(S);
end;

function TGDBSourceEditor.InsertNewLine: Sw_integer;
Var
  S : string;
  CommandCalled : boolean;

begin
  if IsReadOnly then begin InsertNewLine:=-1; Exit; end;
  if CurPos.Y<GetLineCount then S:=GetDisplayText(CurPos.Y) else S:='';
  s:=Copy(S,1,CurPos.X);
  CommandCalled:=false;
  if Pos(GDBPrompt,S)=1 then
    Delete(S,1,length(GDBPrompt));
{$ifndef NODEBUG}
  if assigned(Debugger) then
    if S<>'' then
      begin
        LastCommand:=S;
        { should be true only if we are at the end ! }
        IgnoreStringAtEnd:=(CurPos.Y=GetLineCount-1) and
          (CurPos.X>=length(RTrim(GetDisplayText(GetLineCount-1))));
        Debugger^.Command(S);
        CommandCalled:=true;
        IgnoreStringAtEnd:=false;
      end
    else if AutoRepeat and (CurPos.Y=GetLineCount-1) then
      begin
        Debugger^.Command(LastCommand);
        CommandCalled:=true;
      end;
{$endif NODEBUG}
  InsertNewLine:=inherited InsertNewLine;
  If CommandCalled then
    InsertText(GDBPrompt);
end;


constructor TGDBWindow.Init(var Bounds: TRect);
var HSB,VSB: PScrollBar;
    R: TRect;
begin
  inherited Init(Bounds,dialog_gdbwindow,0);
  Options:=Options or ofTileAble;
  AutoNumber:=true;
  HelpCtx:=hcGDBWindow;
  GetExtent(R); R.A.Y:=R.B.Y-1; R.Grow(-1,0); R.A.X:=14;
  New(HSB, Init(R)); HSB^.GrowMode:=gfGrowLoY+gfGrowHiX+gfGrowHiY; Insert(HSB);
  GetExtent(R); R.A.X:=R.B.X-1; R.Grow(0,-1);
  New(VSB, Init(R)); VSB^.GrowMode:=gfGrowLoX+gfGrowHiX+gfGrowHiY; Insert(VSB);
  GetExtent(R); R.A.X:=3; R.B.X:=14; R.A.Y:=R.B.Y-1;
  New(Indicator, Init(R));
  Indicator^.GrowMode:=gfGrowLoY+gfGrowHiY;
  Insert(Indicator);
  GetExtent(R); R.Grow(-1,-1);
  New(Editor, Init(R, HSB, VSB, Indicator, GDBOutputFile));
  Editor^.GrowMode:=gfGrowHiX+gfGrowHiY;
  Editor^.SetFlags(efInsertMode+efSyntaxHighlight+efNoIndent+efExpandAllTabs);
  if ExistsFile(GDBOutputFile) then
    begin
      if Editor^.LoadFile=false then
        ErrorBox(FormatStrStr(msg_errorreadingfile,GDBOutputFile),nil);
    end
  else
  { Empty files are buggy !! }
    Editor^.AddLine('');
  Insert(Editor);
{$ifndef NODEBUG}
 {$ifndef GDBMI}
  if assigned(Debugger) then
    Debugger^.SetCommand('width ' + IntToStr(Size.X-1));
 {$endif GDBMI}
{$endif NODEBUG}
  Editor^.silent:=false;
  Editor^.AutoRepeat:=true;
  Editor^.InsertText(GDBPrompt);
end;

procedure TGDBWindow.HandleEvent(var Event: TEvent);
var DontClear: boolean;
begin
  case Event.What of
    evCommand :
      begin
        DontClear:=false;
        case Event.Command of
          cmSaveAs :
              Editor^.SaveAs;
        else DontClear:=true;
        end;
        if DontClear=false then ClearEvent(Event);
      end;
  end;
  inherited HandleEvent(Event);
end;

destructor TGDBWindow.Done;
begin
  if @Self=GDBWindow then
    GDBWindow:=nil;
  inherited Done;
end;

constructor TGDBWindow.Load(var S: TStream);
begin
  inherited Load(S);
  GetSubViewPtr(S,Indicator);
  GetSubViewPtr(S,Editor);
  GDBWindow:=@self;
end;

procedure TGDBWindow.Store(var S: TStream);
begin
  inherited Store(S);
  PutSubViewPtr(S,Indicator);
  PutSubViewPtr(S,Editor);
end;

function TGDBWindow.GetPalette: PPalette;
const P: string[length(CSourceWindow)] = CSourceWindow;
begin
  GetPalette:=@P;
end;

procedure TGDBWindow.WriteOutputText(Buf : pchar);
begin
  {selected normal color ?}
  WriteText(Buf,false);
end;

procedure TGDBWindow.WriteErrorText(Buf : pchar);
begin
  {selected normal color ?}
  WriteText(Buf,true);
end;

procedure TGDBWindow.WriteString(Const S : string);
begin
  Editor^.AddLine(S);
end;

procedure TGDBWindow.WriteErrorString(Const S : string);
begin
  Editor^.AddErrorLine(S);
end;

procedure TGDBWindow.WriteText(Buf : pchar;IsError : boolean);
  var p,pe : pchar;
      s : string;
begin
  p:=buf;
  DeskTop^.Lock;
  While assigned(p) and (p^<>#0) do
    begin
       pe:=strscan(p,#10);
       { if pe-p is more than High(s), discard for this round }
       if (pe<>nil) and (pe-p > high(s)) then
         pe:=nil;
       if (pe<>nil)  then
         pe^:=#0;
       s:=strpas(p);
       If IsError then
         Editor^.AddErrorLine(S)
       else
         Editor^.AddLine(S);
       { restore for dispose }
       if pe<>nil then
         pe^:=#10;
       if pe=nil then
         begin
           if strlen(p)<High(s) then
             p:=nil
           else
             p:=p+High(s);
         end
       else
         begin
           p:=pe;
           inc(p);
         end;
    end;
  DeskTop^.Unlock;
  Editor^.Draw;
end;

procedure TGDBWindow.UpdateCommands;
var Active: boolean;
begin
  Active:=GetState(sfActive);
  SetCmdState([cmSaveAs,cmHide,cmRun],Active);
  SetCmdState(EditorCmds,Active);
  SetCmdState(ToClipCmds+FromClipCmds+NulClipCmds+UndoCmd+RedoCmd,Active);
  Message(Application,evBroadcast,cmCommandSetChanged,nil);
end;


function  TDisasLineCollection.At(Index: sw_Integer): PDisasLine;
begin
  At := PDisasLine(Inherited At(Index));
end;

constructor TDisassemblyEditor.Init(var Bounds: TRect; AHScrollBar, AVScrollBar:
          PScrollBar; AIndicator: PIndicator;const AFileName: string);
begin
  Inherited Init(Bounds,AHScrollBar,AVScrollBar,AIndicator,AFileName);
  GrowMode:=gfGrowHiX+gfGrowHiY;
  SetFlags(efInsertMode+efSyntaxHighlight+efNoIndent+efExpandAllTabs{+efHighlightRow});
  New(DisasLines,Init(500,1000));
  Core^.ChangeLinesTo(DisasLines);
  { do not allow to write into that window }
  ReadOnly:=true;
  AddLine('');
  MinAddress:=0;
  MaxAddress:=0;
  CurL:=nil;
  OwnsSource:=false;
  Source:=nil;
end;

destructor TDisassemblyEditor.Done;
begin
  ReleaseSource;
  Inherited Done;
end;

procedure TDisassemblyEditor.ReleaseSource;
begin
  if OwnsSource and assigned(source) then
    begin
      Desktop^.Delete(Source);
      Dispose(Source,Done);
    end;
  OwnsSource:=false;
  Source:=nil;
  CurrentSource:='';
end;

procedure  TDisassemblyEditor.AddSourceLine(const AFileName: string;line : longint);
var
  S : String;
begin
   if AFileName<>CurrentSource then
     begin
       ReleaseSource;
       Source:=SearchOnDesktop(FileName,false);
       if not assigned(Source) then
         begin
           Source:=ITryToOpenFile(nil,AFileName,0,line,false,false,true);
           OwnsSource:=true;
         end
       else
         OwnsSource:=false;
       CurrentSource:=AFileName;
     end;
   if Assigned(Source) and (line>0) then
     S:=Trim(Source^.Editor^.GetLineText(line-1))
   else
     S:='<source not found>';
   CurrentLine:=Line;
   inherited AddLine(AFileName+':'+IntToStr(line)+' '+S);
   { display differently }
   SetLineFlagState(GetLineCount-1,lfSpecialRow,true);
   LimitsChanged;
end;

procedure  TDisassemblyEditor.AddAssemblyLine(const S: string;AAddress : CORE_ADDR);
var
  PL : PDisasLine;
  LI : PEditorLineInfo;
begin
   if AAddress<>0 then
<<<<<<< HEAD
<<<<<<< HEAD
     inherited AddLine('$'+hexstr(AAddress,sizeof(CORE_ADDR)*2)+S)
=======
     inherited AddLine('$'+hexstr(AAddress,sizeof(PtrUInt)*2)+S)
>>>>>>> graemeg/fixes_2_2
=======
     inherited AddLine('$'+hexstr(AAddress,sizeof(PtrUInt)*2)+S)
>>>>>>> origin/fixes_2_2
   else
     inherited AddLine(S);
   PL:=DisasLines^.At(DisasLines^.count-1);
   PL^.Address:=AAddress;
   LI:=PL^.GetEditorInfo(@Self);
   if AAddress<>0 then
     LI^.BeginsWithAsm:=true;
   LimitsChanged;
   if ((AAddress<minaddress) or (minaddress=0)) and (AAddress<>0) then
     MinAddress:=AAddress;
   if (AAddress>maxaddress) or (maxaddress=0) then
     MaxAddress:=AAddress;
end;

function   TDisassemblyEditor.GetCurrentLine(address : CORE_ADDR) : PDisasLine;

  function IsCorrectLine(PL : PDisasLine) : boolean;
    begin
      IsCorrectLine:=PL^.Address=Address;
    end;
  Var
    PL : PDisasLine;
begin
  PL:=DisasLines^.FirstThat(@IsCorrectLine);
  if Assigned(PL) then
    begin
      if assigned(CurL) then
        CurL^.SetFlagState(lfDebuggerRow,false);
      SetCurPtr(0,DisasLines^.IndexOf(PL));
      PL^.SetFlags(lfDebuggerRow);
      CurL:=PL;
      TrackCursor(do_not_centre);
    end;
  GetCurrentLine:=PL;
end;

    { PDisassemblyWindow = ^TDisassemblyWindow;
    TDisassemblyWindow = object(TFPWindow)
      Editor    : PDisassemblyEditor;
      Indicator : PIndicator; }
constructor TDisassemblyWindow.Init(var Bounds: TRect);
var HSB,VSB: PScrollBar;
    R: TRect;
begin
  inherited Init(Bounds,dialog_disaswindow,0);
  Options:=Options or ofTileAble;
  AutoNumber:=true;
  HelpCtx:=hcDisassemblyWindow;
  GetExtent(R); R.A.Y:=R.B.Y-1; R.Grow(-1,0); R.A.X:=14;
  New(HSB, Init(R)); HSB^.GrowMode:=gfGrowLoY+gfGrowHiX+gfGrowHiY; Insert(HSB);
  GetExtent(R); R.A.X:=R.B.X-1; R.Grow(0,-1);
  New(VSB, Init(R)); VSB^.GrowMode:=gfGrowLoX+gfGrowHiX+gfGrowHiY; Insert(VSB);
  GetExtent(R); R.A.X:=3; R.B.X:=14; R.A.Y:=R.B.Y-1;
  New(Indicator, Init(R));
  Indicator^.GrowMode:=gfGrowLoY+gfGrowHiY;
  Insert(Indicator);
  GetExtent(R); R.Grow(-1,-1);
  New(Editor, Init(R, HSB, VSB, nil, GDBOutputFile));
  Insert(Editor);
  DisassemblyWindow:=@Self;
end;

procedure   TDisassemblyWindow.LoadFunction(Const FuncName : string);
var
   p : pchar;
begin
{$ifndef NODEBUG}
  If not assigned(Debugger) then Exit;
  Debugger^.SetCommand('print symbol on');
  Debugger^.SetCommand('width 0xffffffff');
  Debugger^.Command('disas /m '+FuncName);
  p:=StrNew(Debugger^.GetOutput);
  ProcessPChar(p);
  if (Debugger^.IsRunning) and (FuncName='') then
    Editor^.GetCurrentLine(Debugger^.current_pc);
{$endif NODEBUG}
end;

procedure   TDisassemblyWindow.LoadAddress(Addr : CORE_ADDR);
var
   p : pchar;
begin
{$ifndef NODEBUG}
  If not assigned(Debugger) then Exit;
  Debugger^.SetCommand('print symbol on');
  Debugger^.SetCommand('width 0xffffffff');
  Debugger^.Command('disas /m 0x'+HexStr(Addr,sizeof(Addr)*2));
  p:=StrNew(Debugger^.GetOutput);
  ProcessPChar(p);
  if Debugger^.IsRunning and
     (Debugger^.current_pc>=Editor^.MinAddress) and
     (Debugger^.current_pc<=Editor^.MaxAddress) then
    Editor^.GetCurrentLine(Debugger^.current_pc);
{$endif NODEBUG}
end;


function TDisassemblyWindow.ProcessPChar(p : pchar) : boolean;
var
  p1: pchar;
  pline : pchar;
  pos1, pos2, CurLine, PrevLine : longint;
  CurAddr : CORE_ADDR;
  err : word;
  curaddress, cursymofs, CurFile,
  PrevFile, line : string;
begin
  ProcessPChar:=true;
  Lock;
  Editor^.DisasLines^.FreeAll;
  Editor^.SetFlags(Editor^.GetFlags or efSyntaxHighlight or efKeepLineAttr);

  Editor^.MinAddress:=0;
  Editor^.MaxAddress:=0;
  Editor^.CurL:=nil;
  p1:=p;
  PrevFile:='';
  PrevLine:=0;
  while assigned(p) do
    begin
      pline:=strscan(p,#10);
      if assigned(pline) then
        pline^:=#0;
      line:=trim(strpas(p));
      CurAddr:=0;
      if assigned(pline) then
        begin
          pline^:=#10;
          p:=pline+1;
        end
      else
        p:=nil;
      { now process the line }
      { Remove current position marker }
      if copy(line,1,3)='=> ' then
        begin
          system.delete(line,1,3);
        end;

      { line is hexaddr <symbol+sym_offset at filename:line> assembly }
      pos1:=pos('<',line);
      if pos1>0 then
        begin
          curaddress:=trim(copy(line,1,pos1-1));
          if copy(curaddress,1,2)='0x' then
            curaddress:='$'+copy(curaddress,3,length(curaddress)-2);
          val(curaddress,CurAddr,err);
          if err>0 then
            val(copy(curaddress,1,err-1),CurAddr,err);
          system.delete(line,1,pos1);
        end;
      pos1:=pos(' at ',line);
      pos2:=pos('>',line);
      if (pos1>0) and (pos1 < pos2) then
        begin
          cursymofs:=copy(line,1,pos1-1);
          CurFile:=copy(line,pos1+4,pos2-pos1-4);
          pos1:=pos(':',CurFile);
          if pos1>0 then
            begin
              val(copy(CurFile,pos1+1,high(CurFile)),CurLine,err);
              system.delete(CurFile,pos1,high(CurFile));
            end
          else
            CurLine:=0;
          system.delete(line,1,pos2);
        end
      else    { no ' at ' found before '>' }
        begin
          cursymofs:=copy(line,1,pos2-1);
          CurFile:='';
          system.delete(line,1,pos2);
        end;
      if (CurFile<>'') and ((CurFile<>PrevFile) or (CurLine<>PrevLine)) then
        begin
          WriteSourceString(CurFile,CurLine);
          PrevLine:=CurLine;
          PrevFile:=CurFile;
        end;
      WriteDisassemblyString(line,curaddr);
    end;
  StrDispose(p1);
  Editor^.ReleaseSource;
  Editor^.UpdateAttrs(0,attrForceFull);
  If assigned(BreakpointsCollection) then
    BreakpointsCollection^.ShowBreakpoints(@Self);
  Unlock;
  ReDraw;
end;

procedure   TDisassemblyWindow.HandleEvent(var Event: TEvent);
begin
  inherited HandleEvent(Event);
end;

procedure   TDisassemblyWindow.WriteSourceString(Const S : string;line : longint);
begin
  Editor^.AddSourceLine(S,line);
end;

procedure   TDisassemblyWindow.WriteDisassemblyString(Const S : string;address : CORE_ADDR);
begin
  Editor^.AddAssemblyLine(S,address);
end;

procedure   TDisassemblyWindow.SetCurAddress(address : CORE_ADDR);
begin
  if (address<Editor^.MinAddress) or (address>Editor^.MaxAddress) then
    LoadAddress(address);
  Editor^.GetCurrentLine(address);
end;

procedure TDisassemblyWindow.UpdateCommands;
var Active: boolean;
begin
  Active:=GetState(sfActive);
  SetCmdState(SourceCmds+CompileCmds,Active);
  SetCmdState(EditorCmds,Active);
  SetCmdState(ToClipCmds+FromClipCmds+NulClipCmds+UndoCmd+RedoCmd,false);
  Message(Application,evBroadcast,cmCommandSetChanged,nil);
end;


function    TDisassemblyWindow.GetPalette: PPalette;
const P: string[length(CSourceWindow)] = CSourceWindow;
begin
  GetPalette:=@P;
end;

destructor  TDisassemblyWindow.Done;
begin
  if @Self=DisassemblyWindow then
    DisassemblyWindow:=nil;
  inherited Done;
end;
{$endif NODEBUG}



constructor TClipboardWindow.Init;
var R: TRect;
    HSB,VSB: PScrollBar;
begin
  Desktop^.GetExtent(R);
  inherited Init(R, '*');
  SetTitle(dialog_clipboard);
  HelpCtx:=hcClipboardWindow;
  Number:=wnNoNumber;
  AutoNumber:=true;

  GetExtent(R); R.A.Y:=R.B.Y-1; R.Grow(-1,0); R.A.X:=14;
  New(HSB, Init(R)); HSB^.GrowMode:=gfGrowLoY+gfGrowHiX+gfGrowHiY; Insert(HSB);
  GetExtent(R); R.A.X:=R.B.X-1; R.Grow(0,-1);
  New(VSB, Init(R)); VSB^.GrowMode:=gfGrowLoX+gfGrowHiX+gfGrowHiY; Insert(VSB);
  GetExtent(R); R.A.X:=3; R.B.X:=14; R.A.Y:=R.B.Y-1;
  New(Indicator, Init(R));
  Indicator^.GrowMode:=gfGrowLoY+gfGrowHiY;
  Insert(Indicator);
  GetExtent(R); R.Grow(-1,-1);
  New(Editor, Init(R, HSB, VSB, Indicator, ''));
  Editor^.GrowMode:=gfGrowHiX+gfGrowHiY;
  Insert(Editor);

  Editor^.SetFlags(Editor^.GetFlags or efUseTabCharacters);
  Hide;

  Clipboard:=Editor;
end;

procedure TClipboardWindow.Close;
begin
  Hide;
end;

constructor TClipboardWindow.Load(var S: TStream);
begin
  inherited Load(S);

  Clipboard:=Editor;
end;

procedure TClipboardWindow.Store(var S: TStream);
begin
  inherited Store(S);
end;

destructor TClipboardWindow.Done;
begin
  inherited Done;
  Clipboard:=nil;
  ClipboardWindow:=nil;
end;


constructor TMessageListBox.Init(var Bounds: TRect; AHScrollBar, AVScrollBar: PScrollBar);
begin
  inherited Init(Bounds,1,AHScrollBar, AVScrollBar);
  GrowMode:=gfGrowHiX+gfGrowHiY;
  New(ModuleNames, Init(50,100));
  NoSelection:=true;
end;


function TMessageListBox.GetLocalMenu: PMenu;
var M: PMenu;
begin
  if (Owner<>nil) and (Owner^.GetState(sfModal)) then M:=nil else
  M:=NewMenu(
    NewItem(menu_msglocal_clear,'',kbNoKey,cmMsgClear,hcMsgClear,
    NewLine(
    NewItem(menu_msglocal_gotosource,'',kbNoKey,cmMsgGotoSource,hcMsgGotoSource,
    NewItem(menu_msglocal_tracksource,'',kbNoKey,cmMsgTrackSource,hcMsgTrackSource,
    NewLine(
    NewItem(menu_msglocal_saveas,'',kbNoKey,cmSaveAs,hcSaveAs,
    nil)))))));
  GetLocalMenu:=M;
end;

procedure TMessageListBox.SetState(AState: Word; Enable: Boolean);
var OldState: word;
begin
  OldState:=State;
  inherited SetState(AState,Enable);
  if ((AState and sfActive)<>0) and (((OldState xor State) and sfActive)<>0) then
    SetCmdState([cmSaveAs],Enable);
end;


procedure TMessageListBox.HandleEvent(var Event: TEvent);
var DontClear: boolean;
begin
  case Event.What of
    evKeyDown :
      begin
        DontClear:=false;
        case Event.KeyCode of
          kbEnter :
            begin
              Message(@Self,evCommand,cmMsgGotoSource,nil);
              ClearEvent(Event);
              exit;
            end;
        else
          DontClear:=true;
        end;
        if not DontClear then
          ClearEvent(Event);
      end;
    evBroadcast :
      case Event.Command of
        cmListItemSelected :
          if Event.InfoPtr=@Self then
            Message(@Self,evCommand,cmMsgTrackSource,nil);
      end;
    evCommand :
      begin
        DontClear:=false;
        case Event.Command of
          cmMsgGotoSource :
            if Range>0 then
              begin
                GotoSource;
                ClearEvent(Event);
                exit;
              end;
          cmMsgTrackSource :
            if Range>0 then
              TrackSource;
          cmMsgClear :
            Clear;
          cmSaveAs :
            SaveAs;
          else
            DontClear:=true;
        end;
        if not DontClear then
          ClearEvent(Event);
      end;
  end;
  inherited HandleEvent(Event);
end;

procedure TMessageListBox.AddItem(P: PMessageItem);
var W : integer;
begin
  if List=nil then New(List, Init(500,500));
  W:=length(P^.GetText(255));
  if W>MaxWidth then
  begin
    MaxWidth:=W;
    if HScrollBar<>nil then
       HScrollBar^.SetRange(0,MaxWidth);
  end;
  List^.Insert(P);
  SetRange(List^.Count);
  if Focused=List^.Count-1-1 then
     FocusItem(List^.Count-1);
  DrawView;
end;

function TMessageListBox.AddModuleName(const Name: string): PString;
var P: PString;
begin
  if ModuleNames<>nil then
    P:=ModuleNames^.Add(Name)
  else
    P:=nil;
  AddModuleName:=P;
end;

function TMessageListBox.GetText(Item,MaxLen: Sw_Integer): String;
var P: PMessageItem;
    S: string;
begin
  P:=List^.At(Item);
  S:=P^.GetText(MaxLen);
  GetText:=copy(S,1,MaxLen);
end;

procedure TMessageListBox.Clear;
begin
  if assigned(List) then
    Dispose(List, Done);
  List:=nil;
  MaxWidth:=0;
  if assigned(ModuleNames) then
    ModuleNames^.FreeAll;
  SetRange(0); DrawView;
  Message(Application,evBroadcast,cmClearLineHighlights,@Self);
end;

procedure TMessageListBox.TrackSource;
var W: PSourceWindow;
    P: PMessageItem;
    R: TRect;
    Row,Col: sw_integer;
    Found : boolean;
begin
  Message(Application,evBroadcast,cmClearLineHighlights,@Self);
  if Range=0 then Exit;
  P:=List^.At(Focused);
  if P^.Row=0 then Exit;
  Desktop^.Lock;
  GetNextEditorBounds(R);
  R.B.Y:=Owner^.Origin.Y;
  if P^.Row>0 then Row:=P^.Row-1 else Row:=0;
  if P^.Col>0 then Col:=P^.Col-1 else Col:=0;
  W:=EditorWindowFile(P^.GetModuleName);
  if assigned(W) then
    begin
      W^.GetExtent(R);
      R.B.Y:=Owner^.Origin.Y;
      W^.ChangeBounds(R);
      W^.Editor^.SetCurPtr(Col,Row);
    end
  else
    W:=TryToOpenFile(@R,P^.GetModuleName,Col,Row,true);
  { Try to find it by browsing }
  if W=nil then
    begin
      Desktop^.UnLock;
      Found:=IDEApp.OpenSearch(P^.GetModuleName+'*');
      if found then
        W:=TryToOpenFile(nil,P^.GetModuleName,Col,Row,true);
      Desktop^.Lock;
    end;
  if W<>nil then
    begin
      W^.Select;
      W^.Editor^.TrackCursor(do_centre);
      W^.Editor^.SetLineFlagExclusive(lfHighlightRow,Row);
    end;
  if Assigned(Owner) then
    Owner^.Select;
  Desktop^.UnLock;
end;

procedure TMessageListBox.GotoSource;
var W: PSourceWindow;
    P: PMessageItem;
    R:TRect;
    Row,Col: sw_integer;
    Found : boolean;
    Event : TEvent;
begin
  Message(Application,evBroadcast,cmClearLineHighlights,@Self);
  if Range=0 then Exit;
  P:=List^.At(Focused);
  if P^.Row=0 then Exit;
  Desktop^.Lock;
  if P^.Row>0 then Row:=P^.Row-1 else Row:=0;
  if P^.Col>0 then Col:=P^.Col-1 else Col:=0;
  W:=EditorWindowFile(P^.GetModuleName);
  if assigned(W) then
    begin
      W^.GetExtent(R);
      if Owner^.Origin.Y>R.A.Y+4 then
        R.B.Y:=Owner^.Origin.Y;
      W^.ChangeBounds(R);
      W^.Editor^.SetCurPtr(Col,Row);
    end
  else
   W:=TryToOpenFile(nil,P^.GetModuleName,Col,Row,true);
  { Try to find it by browsing }
  if W=nil then
    begin
      Desktop^.UnLock;
      Found:=IDEApp.OpenSearch(P^.GetModuleName+'*');
      if found then
        W:=TryToOpenFile(nil,P^.GetModuleName,Col,Row,true);
      Desktop^.Lock;
    end;
  if assigned(W) then
    begin
      { Message(Owner,evCommand,cmClose,nil);
        This calls close on StackWindow
        rendering P invalid
        so postpone it PM }
      W^.GetExtent(R);
      if (P^.TClass<>0) then
        W^.Editor^.SetErrorMessage(P^.GetText(R.B.X-R.A.X));
      W^.Select;
      Owner^.Hide;
    end;
  Desktop^.UnLock;
  if assigned(W) then
    begin
      Event.What:=evCommand;
      Event.command:=cmClose;
      Event.InfoPtr:=nil;
      fpide.PutEvent(Owner,Event);
    end;
end;

procedure TMessageListBox.Draw;
var
  I, J, Item: Sw_Integer;
  NormalColor, SelectedColor, FocusedColor, Color: Word;
  ColWidth, CurCol, Indent: Integer;
  B: TDrawBuffer;
  Text: String;
  SCOff: Byte;
  TC: byte;
procedure MT(var C: word); begin if TC<>0 then C:=(C and $ff0f) or (TC and $f0); end;
begin
  if (Owner<>nil) then TC:=ord(Owner^.GetColor(6)) else TC:=0;
  if State and (sfSelected + sfActive) = (sfSelected + sfActive) then
  begin
    NormalColor := GetColor(1);
    FocusedColor := GetColor(3);
    SelectedColor := GetColor(4);
  end else
  begin
    NormalColor := GetColor(2);
    SelectedColor := GetColor(4);
  end;
  if Transparent then
    begin MT(NormalColor); MT(SelectedColor); end;
  if NoSelection then
     SelectedColor:=NormalColor;
  if HScrollBar <> nil then Indent := HScrollBar^.Value
  else Indent := 0;
  ColWidth := Size.X div NumCols + 1;
  for I := 0 to Size.Y - 1 do
  begin
    for J := 0 to NumCols-1 do
    begin
      Item := J*Size.Y + I + TopItem;
      CurCol := J*ColWidth;
      if (State and (sfSelected + sfActive) = (sfSelected + sfActive)) and
        (Focused = Item) and (Range > 0) then
      begin
        Color := FocusedColor;
        SetCursor(CurCol+1,I);
        SCOff := 0;
      end
      else if (Item < Range) and IsSelected(Item) then
      begin
        Color := SelectedColor;
        SCOff := 2;
      end
      else
      begin
        Color := NormalColor;
        SCOff := 4;
      end;
      MoveChar(B[CurCol], ' ', Color, ColWidth);
      if Item < Range then
      begin
        Text := GetText(Item, ColWidth + Indent);
        Text := Copy(Text,Indent,ColWidth);
        MoveStr(B[CurCol+1], Text, Color);
        if ShowMarkers then
        begin
          WordRec(B[CurCol]).Lo := Byte(SpecialChars[SCOff]);
          WordRec(B[CurCol+ColWidth-2]).Lo := Byte(SpecialChars[SCOff+1]);
        end;
      end;
      MoveChar(B[CurCol+ColWidth-1], #179, GetColor(5), 1);
    end;
    WriteLine(0, I, Size.X, 1, B);
  end;
end;

constructor TMessageListBox.Load(var S: TStream);
begin
  inherited Load(S);
  New(ModuleNames, Init(50,100));
  NoSelection:=true;
end;

procedure TMessageListBox.Store(var S: TStream);
var OL: PCollection;
    ORV: sw_integer;
begin
  OL:=List; ORV:=Range;

  New(List, Init(1,1)); Range:=0;

  inherited Store(S);

  Dispose(List, Done);
  List:=OL; Range:=ORV;
  { ^^^ nasty trick - has anyone a better idea how to avoid storing the
    collection? Pasting here a modified version of TListBox.Store+
    TAdvancedListBox.Store isn't a better solution, since by eventually
    changing the obj-hierarchy you'll always have to modify this, too - BG }
end;

destructor TMessageListBox.Done;
begin
  inherited Done;
  if List<>nil then Dispose(List, Done);
  if ModuleNames<>nil then Dispose(ModuleNames, Done);
end;

constructor TMessageItem.Init(AClass: longint; const AText: string; AModule: PString; ARow, ACol: sw_integer);
begin
  inherited Init;
  TClass:=AClass;
  Text:=NewStr(AText);
  Module:=AModule;
  Row:=ARow; Col:=ACol;
end;

function TMessageItem.GetText(MaxLen: Sw_integer): string;
var S: string;
begin
  if Text=nil then S:='' else S:=Text^;
  if (Module<>nil) then
     S:=NameAndExtOf(Module^)+'('+IntToStr(Row)+') '+S;
  if length(S)>MaxLen then S:=copy(S,1,MaxLen-2)+'..';
  GetText:=S;
end;

procedure TMessageItem.Selected;
begin
end;

function TMessageItem.GetModuleName: string;
begin
  GetModuleName:=GetStr(Module);
end;

destructor TMessageItem.Done;
begin
  inherited Done;
  if Text<>nil then DisposeStr(Text);
{  if Module<>nil then DisposeStr(Module);}
end;


procedure  TFPDlgWindow.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evBroadcast :
      case Event.Command of
        cmSearchWindow+1..cmSearchWindow+99 :
          if (Event.Command-cmSearchWindow=Number) then
              ClearEvent(Event);
      end;
  end;
  inherited HandleEvent(Event);
end;


(*
constructor TTab.Init(var Bounds: TRect; ATabDef: PTabDef);
begin
  inherited Init(Bounds);
  Options:=Options or ofSelectable or ofFirstClick or ofPreProcess or ofPostProcess;
  GrowMode:=gfGrowHiX+gfGrowHiY+gfGrowRel;
  TabDefs:=ATabDef;
  ActiveDef:=-1;
  SelectTab(0);
  ReDraw;
end;

function TTab.TabCount: integer;
var i: integer;
    P: PTabDef;
begin
  I:=0; P:=TabDefs;
  while (P<>nil) do
    begin
      Inc(I);
      P:=P^.Next;
    end;
  TabCount:=I;
end;

function TTab.AtTab(Index: integer): PTabDef;
var i: integer;
    P: PTabDef;
begin
  i:=0; P:=TabDefs;
  while (I<Index) do
    begin
      if P=nil then RunError($AA);
      P:=P^.Next;
      Inc(i);
    end;
  AtTab:=P;
end;

procedure TTab.SelectTab(Index: integer);
var P: PTabItem;
    V: PView;
begin
  if ActiveDef<>Index then
  begin
    if Owner<>nil then Owner^.Lock;
    Lock;
    { --- Update --- }
    if TabDefs<>nil then
       begin
         DefCount:=1;
         while AtTab(DefCount-1)^.Next<>nil do Inc(DefCount);
       end
       else DefCount:=0;
    if ActiveDef<>-1 then
    begin
      P:=AtTab(ActiveDef)^.Items;
      while P<>nil do
        begin
          if P^.View<>nil then Delete(P^.View);
          P:=P^.Next;
        end;
    end;
    ActiveDef:=Index;
    P:=AtTab(ActiveDef)^.Items;
    while P<>nil do
      begin
        if P^.View<>nil then Insert(P^.View);
        P:=P^.Next;
      end;
    V:=AtTab(ActiveDef)^.DefItem;
    if V<>nil then V^.Select;
    ReDraw;
    { --- Update --- }
    UnLock;
    if Owner<>nil then Owner^.UnLock;
    DrawView;
  end;
end;

procedure TTab.ChangeBounds(var Bounds: TRect);
var D: TPoint;
procedure DoCalcChange(P: PView);
var
  R: TRect;
begin
  if P^.Owner=nil then Exit; { it think this is a bug in TV }
  P^.CalcBounds(R, D);
  P^.ChangeBounds(R);
end;
var
    P: PTabItem;
    I: integer;
begin
  D.X := Bounds.B.X - Bounds.A.X - Size.X;
  D.Y := Bounds.B.Y - Bounds.A.Y - Size.Y;
  inherited ChangeBounds(Bounds);
  for I:=0 to TabCount-1 do
  if I<>ActiveDef then
    begin
      P:=AtTab(I)^.Items;
      while P<>nil do
        begin
          if P^.View<>nil then DoCalcChange(P^.View);
          P:=P^.Next;
        end;
    end;
end;

procedure TTab.SelectNextTab(Forwards: boolean);
var Index: integer;
begin
  Index:=ActiveDef;
  if Index=-1 then Exit;
  if Forwards then Inc(Index) else Dec(Index);
  if Index<0 then Index:=DefCount-1 else
  if Index>DefCount-1 then Index:=0;
  SelectTab(Index);
end;

procedure TTab.HandleEvent(var Event: TEvent);
var Index : integer;
    I     : integer;
    X     : integer;
    Len   : byte;
    P     : TPoint;
    V     : PView;
    CallOrig: boolean;
    LastV : PView;
    FirstV: PView;
function FirstSelectable: PView;
var
    FV : PView;
begin
  FV := First;
  while (FV<>nil) and ((FV^.Options and ofSelectable)=0) and (FV<>Last) do
        FV:=FV^.Next;
  if FV<>nil then
    if (FV^.Options and ofSelectable)=0 then FV:=nil;
  FirstSelectable:=FV;
end;
function LastSelectable: PView;
var
    LV : PView;
begin
  LV := Last;
  while (LV<>nil) and ((LV^.Options and ofSelectable)=0) and (LV<>First) do
        LV:=LV^.Prev;
  if LV<>nil then
    if (LV^.Options and ofSelectable)=0 then LV:=nil;
  LastSelectable:=LV;
end;
begin
  if (Event.What and evMouseDown)<>0 then
     begin
       MakeLocal(Event.Where,P);
       if P.Y<3 then
          begin
            Index:=-1; X:=1;
            for i:=0 to DefCount-1 do
                begin
                  Len:=CStrLen(AtTab(i)^.Name^);
                  if (P.X>=X) and (P.X<=X+Len+1) then Index:=i;
                  X:=X+Len+3;
                end;
            if Index<>-1 then
               SelectTab(Index);
          end;
     end;
  if Event.What=evKeyDown then
     begin
       Index:=-1;
       case Event.KeyCode of
            kbCtrlTab :
              begin
                SelectNextTab((Event.KeyShift and kbShift)=0);
                ClearEvent(Event);
              end;
            kbTab,kbShiftTab  :
              if GetState(sfSelected) then
                 begin
                   if Current<>nil then
                   begin
                   LastV:=LastSelectable; FirstV:=FirstSelectable;
                   if ((Current=LastV) or (Current=PLabel(LastV)^.Link)) and (Event.KeyCode=kbShiftTab) then
                      begin
                        if Owner<>nil then Owner^.SelectNext(true);
                      end else
                   if ((Current=FirstV) or (Current=PLabel(FirstV)^.Link)) and (Event.KeyCode=kbTab) then
                      begin
                        Lock;
                        if Owner<>nil then Owner^.SelectNext(false);
                        UnLock;
                      end else
                   SelectNext(Event.KeyCode=kbShiftTab);
                   ClearEvent(Event);
                   end;
                 end;
       else
       for I:=0 to DefCount-1 do
           begin
             if Upcase(GetAltChar(Event.KeyCode))=AtTab(I)^.ShortCut
                then begin
                       Index:=I;
                       ClearEvent(Event);
                       Break;
                     end;
           end;
       end;
       if Index<>-1 then
          begin
            Select;
            SelectTab(Index);
            V:=AtTab(ActiveDef)^.DefItem;
            if V<>nil then V^.Focus;
          end;
     end;
  CallOrig:=true;
  if Event.What=evKeyDown then
     begin
     if ((Owner<>nil) and (Owner^.Phase=phPostProcess) and (GetAltChar(Event.KeyCode)<>#0)) or GetState(sfFocused)
        then
        else CallOrig:=false;
     end;
  if CallOrig then inherited HandleEvent(Event);
end;

function TTab.GetPalette: PPalette;
begin
  GetPalette:=nil;
end;

procedure TTab.Draw;
var B     : TDrawBuffer;
    i     : integer;
    C1,C2,C3,C : word;
    HeaderLen  : integer;
    X,X2       : integer;
    Name       : PString;
    ActiveKPos : integer;
    ActiveVPos : integer;
    FC   : char;
    ClipR      : TRect;
procedure SWriteBuf(X,Y,W,H: integer; var Buf);
var i: integer;
begin
  if Y+H>Size.Y then H:=Size.Y-Y;
  if X+W>Size.X then W:=Size.X-X;
  if Buffer=nil then WriteBuf(X,Y,W,H,Buf)
                else for i:=1 to H do
                         Move(Buf,Buffer^[X+(Y+i-1)*Size.X],W*2);
end;
procedure ClearBuf;
begin
  MoveChar(B,' ',C1,Size.X);
end;
begin
  if InDraw then Exit;
  InDraw:=true;
  { - Start of TGroup.Draw - }
{  if Buffer = nil then
  begin
    GetBuffer;
  end; }
  { - Start of TGroup.Draw - }

  C1:=GetColor(1); C2:=(GetColor(7) and $f0 or $08)+GetColor(9)*256; C3:=GetColor(8)+GetColor({9}8)*256;
  HeaderLen:=0; for i:=0 to DefCount-1 do HeaderLen:=HeaderLen+CStrLen(AtTab(i)^.Name^)+3; Dec(HeaderLen);
  if HeaderLen>Size.X-2 then HeaderLen:=Size.X-2;

  { --- 1. sor --- }
  ClearBuf; MoveChar(B[0],'³',C1,1); MoveChar(B[HeaderLen+1],'³',C1,1);
  X:=1;
  for i:=0 to DefCount-1 do
      begin
        Name:=AtTab(i)^.Name; X2:=CStrLen(Name^);
        if i=ActiveDef
           then begin
                  ActiveKPos:=X-1;
                  ActiveVPos:=X+X2+2;
                  if GetState(sfFocused) then C:=C3 else C:=C2;
                end
           else C:=C2;
        MoveCStr(B[X],' '+Name^+' ',C); X:=X+X2+3;
        MoveChar(B[X-1],'³',C1,1);
      end;
  SWriteBuf(0,1,Size.X,1,B);

  { --- 0. sor --- }
  ClearBuf; MoveChar(B[0],'Ú',C1,1);
  X:=1;
  for i:=0 to DefCount-1 do
      begin
        if I<ActiveDef then FC:='Ú'
                       else FC:='¿';
        X2:=CStrLen(AtTab(i)^.Name^)+2;
        MoveChar(B[X+X2],{'Â'}FC,C1,1);
        if i=DefCount-1 then X2:=X2+1;
        if X2>0 then
        MoveChar(B[X],'Ä',C1,X2);
        X:=X+X2+1;
      end;
  MoveChar(B[HeaderLen+1],'¿',C1,1);
  MoveChar(B[ActiveKPos],'Ú',C1,1); MoveChar(B[ActiveVPos],'¿',C1,1);
  SWriteBuf(0,0,Size.X,1,B);

  { --- 2. sor --- }
  MoveChar(B[1],'Ä',C1,Max(HeaderLen,0)); MoveChar(B[HeaderLen+2],'Ä',C1,Max(Size.X-HeaderLen-3,0));
  MoveChar(B[Size.X-1],'¿',C1,1);
  MoveChar(B[ActiveKPos],'Ù',C1,1);
  if ActiveDef=0 then MoveChar(B[0],'³',C1,1)
                 else MoveChar(B[0],{'Ã'}'Ú',C1,1);
  MoveChar(B[HeaderLen+1],'Ä'{'Á'},C1,1); MoveChar(B[ActiveVPos],'À',C1,1);
  MoveChar(B[ActiveKPos+1],' ',C1,Max(ActiveVPos-ActiveKPos-1,0));
  SWriteBuf(0,2,Size.X,1,B);

  { --- marad‚k sor --- }
  ClearBuf; MoveChar(B[0],'³',C1,1); MoveChar(B[Size.X-1],'³',C1,1);
  for i:=3 to Size.Y-1 do
    SWriteBuf(0,i,Size.X,1,B);
  { SWriteBuf(0,3,Size.X,Size.Y-4,B); this was wrong
    because WriteBuf then expect a buffer of size size.x*(size.y-4)*2 PM }

  { --- Size.X . sor --- }
  MoveChar(B[0],'À',C1,1); MoveChar(B[1],'Ä',C1,Max(Size.X-2,0)); MoveChar(B[Size.X-1],'Ù',C1,1);
  SWriteBuf(0,Size.Y-1,Size.X,1,B);

  { - End of TGroup.Draw - }
  if Buffer <> nil then
  begin
    Lock;
    Redraw;
    UnLock;
  end;
  if Buffer <> nil then WriteBuf(0, 0, Size.X, Size.Y, Buffer^) else
  begin
    GetClipRect(ClipR);
    Redraw;
    GetExtent(ClipR);
  end;
  { - End of TGroup.Draw - }
  InDraw:=false;
end;

function TTab.Valid(Command: Word): Boolean;
var PT : PTabDef;
    PI : PTabItem;
    OK : boolean;
begin
  OK:=true;
  PT:=TabDefs;
  while (PT<>nil) and (OK=true) do
        begin
          PI:=PT^.Items;
          while (PI<>nil) and (OK=true) do
                begin
                  if PI^.View<>nil then OK:=OK and PI^.View^.Valid(Command);
                  PI:=PI^.Next;
                end;
          PT:=PT^.Next;
        end;
  Valid:=OK;
end;

procedure TTab.SetState(AState: Word; Enable: Boolean);
begin
  inherited SetState(AState,Enable);
  if (AState and sfFocused)<>0 then DrawView;
end;

destructor TTab.Done;
var P,X: PTabDef;
procedure DeleteViews(P: PView);
begin
  if P<>nil then Delete(P);
end;
begin
  ForEach(@DeleteViews);
  inherited Done;
  P:=TabDefs;
  while P<>nil do
        begin
          X:=P^.Next;
          DisposeTabDef(P);
          P:=X;
        end;
end;
*)


constructor TScreenView.Init(var Bounds: TRect; AHScrollBar, AVScrollBar: PScrollBar;
              AScreen: PScreen);
begin
  inherited Init(Bounds,AHScrollBar,AVScrollBar);
  Screen:=AScreen;
  if Screen=nil then
   Fail;
  SetState(sfCursorVis,true);
  Update;
end;

procedure TScreenView.Update;
begin
  SetLimit(UserScreen^.GetWidth,UserScreen^.GetHeight);
  DrawView;
end;

procedure TScreenView.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evBroadcast :
      case Event.Command of
        cmUpdate  : Update;
      end;
  end;
  inherited HandleEvent(Event);
end;

procedure TScreenView.Draw;
var B: TDrawBuffer;
    X,Y: integer;
    Text,Attr: string;
    P: TPoint;
begin
  Screen^.GetCursorPos(P);
  for Y:=Delta.Y to Delta.Y+Size.Y-1 do
  begin
    if Y<Screen^.GetHeight then
      Screen^.GetLine(Y,Text,Attr)
    else
       begin Text:=''; Attr:=''; end;
    Text:=copy(Text,Delta.X+1,255); Attr:=copy(Attr,Delta.X+1,255);
    MoveChar(B,' ',GetColor(1),Size.X);
    for X:=1 to length(Text) do
      MoveChar(B[X-1],Text[X],ord(Attr[X]),1);
    WriteLine(0,Y-Delta.Y,Size.X,1,B);
  end;
  SetCursor(P.X-Delta.X,P.Y-Delta.Y);
end;

constructor TScreenWindow.Init(AScreen: PScreen; ANumber: integer);
var R: TRect;
    VSB,HSB: PScrollBar;
begin
  Desktop^.GetExtent(R);
  inherited Init(R, dialog_userscreen, ANumber);
  Options:=Options or ofTileAble;
  GetExtent(R); R.Grow(-1,-1); R.Move(1,0); R.A.X:=R.B.X-1;
  New(VSB, Init(R)); VSB^.Options:=VSB^.Options or ofPostProcess;
  VSB^.GrowMode:=gfGrowLoX+gfGrowHiX+gfGrowHiY; Insert(VSB);
  GetExtent(R); R.Grow(-1,-1); R.Move(0,1); R.A.Y:=R.B.Y-1;
  New(HSB, Init(R)); HSB^.Options:=HSB^.Options or ofPostProcess;
  HSB^.GrowMode:=gfGrowLoY+gfGrowHiX+gfGrowHiY; Insert(HSB);
  GetExtent(R); R.Grow(-1,-1);
  New(ScreenView, Init(R, HSB, VSB, AScreen));
  ScreenView^.GrowMode:=gfGrowHiX+gfGrowHiY;
  Insert(ScreenView);
  UserScreenWindow:=@Self;
end;

destructor TScreenWindow.Done;
begin
  inherited Done;
  UserScreenWindow:=nil;
end;

const InTranslate : boolean = false;

procedure TranslateMouseClick(View: PView; var Event: TEvent);
procedure TranslateAction(Action: integer);
var E: TEvent;
begin
  if Action<>acNone then
  begin
    E:=Event;
    E.What:=evMouseDown; E.Buttons:=mbLeftButton;
    View^.HandleEvent(E);
    Event.What:=evCommand;
    Event.Command:=ActionCommands[Action];
  end;
end;
begin
  if InTranslate then Exit;
  InTranslate:=true;
  case Event.What of
    evMouseDown :
      if (GetShiftState and kbAlt)<>0 then
        TranslateAction(AltMouseAction) else
      if (GetShiftState and kbCtrl)<>0 then
        TranslateAction(CtrlMouseAction);
  end;
  InTranslate:=false;
end;

function GetNextEditorBounds(var Bounds: TRect): boolean;
var P: PView;
begin
  P:=Desktop^.Current;
  while P<>nil do
  begin
    if P^.HelpCtx=hcSourceWindow then Break;
    P:=P^.NextView;
    if P=Desktop^.Current then
      begin
        P:=nil;
        break;
      end;
  end;
  if P=nil then Desktop^.GetExtent(Bounds) else
     begin
       P^.GetBounds(Bounds);
       Inc(Bounds.A.X); Inc(Bounds.A.Y);
     end;
  GetNextEditorBounds:=P<>nil;
end;

function IOpenEditorWindow(Bounds: PRect; FileName: string; CurX,CurY: sw_integer; ShowIt: boolean): PSourceWindow;
var R: TRect;
    W: PSourceWindow;
begin
  if Assigned(Bounds) then R.Copy(Bounds^) else
    GetNextEditorBounds(R);
  PushStatus(FormatStrStr(msg_openingsourcefile,SmartPath(FileName)));
  New(W, Init(R, FileName));
  if ShowIt=false then
    W^.Hide;
  if W<>nil then
  begin
    if (CurX<>0) or (CurY<>0) then
       with W^.Editor^ do
       begin
         SetCurPtr(CurX,CurY);
         TrackCursor(do_centre);
       end;
    W^.HelpCtx:=hcSourceWindow;
    Desktop^.Insert(W);
    { this makes loading a lot slower and is not needed as far as I can see (FK)
    Message(Application,evBroadcast,cmUpdate,nil);
    }
  end;
  PopStatus;
  IOpenEditorWindow:=W;
end;

function OpenEditorWindow(Bounds: PRect; FileName: string; CurX,CurY: sw_integer): PSourceWindow;
begin
  OpenEditorWindow:=IOpenEditorWindow(Bounds,FileName,CurX,CurY,true);
end;


function LastSourceEditor : PSourceWindow;

  function IsSearchedSource(P: PView) : boolean;
  begin
    if assigned(P) and
       (TypeOf(P^)=TypeOf(TSourceWindow)) then
         IsSearchedSource:=true
       else
         IsSearchedSource:=false;
  end;

begin
  LastSourceEditor:=PSourceWindow(Desktop^.FirstThat(@IsSearchedSource));
end;


function SearchOnDesktop(FileName : string;tryexts:boolean) : PSourceWindow;
var
    D,DS : DirStr;
    N,NS : NameStr;
    E,ES : ExtStr;
    SName : string;

function IsSearchedFile(W : PSourceWindow) : boolean;
  var Found: boolean;
  begin
    Found:=false;
    if (W<>nil) and (W^.HelpCtx=hcSourceWindow) then
      begin
        if (D='') then
          SName:=NameAndExtOf(PSourceWindow(W)^.Editor^.FileName)
        else
          SName:=PSourceWindow(W)^.Editor^.FileName;
        FSplit(SName,DS,NS,ES);
        SName:=UpcaseStr(NS+ES);

        if (E<>'') or (not tryexts) then
          begin
            if D<>'' then
              Found:=UpCaseStr(DS)+SName=UpcaseStr(D+N+E)
            else
              Found:=SName=UpcaseStr(N+E);
          end
        else
          begin
            Found:=SName=UpcaseStr(N+'.pp');
            if Found=false then
              Found:=SName=UpcaseStr(N+'.pas');
          end;
      end;
    IsSearchedFile:=found;
  end;
function IsSearchedSource(P: PView) : boolean;
begin
  if assigned(P) and
     (TypeOf(P^)=TypeOf(TSourceWindow)) then
       IsSearchedSource:=IsSearchedFile(PSourceWindow(P))
     else
       IsSearchedSource:=false;
end;

begin
  FSplit(FileName,D,N,E);
  SearchOnDesktop:=PSourceWindow(Desktop^.FirstThat(@IsSearchedSource));
end;

function TryToOpenFile(Bounds: PRect; FileName: string; CurX,CurY: sw_integer;tryexts:boolean): PSourceWindow;
begin
  TryToOpenFile:=ITryToOpenFile(Bounds,FileName,CurX,CurY,tryexts,true,false);
end;

function TryToOpenFileMulti(Bounds: PRect; FileName: string; CurX,CurY: sw_integer;tryexts:boolean): PSourceWindow;
var srec:SearchRec;
    dir,name,ext : string;
begin
 fsplit(filename,dir,name,ext);
 dir:=completedir(dir);
 FindFirst(filename,anyfile,Srec);
 while (DosError=0) do
   begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     ITryToOpenFile(Bounds,dir+srec.name,CurX,CurY,tryexts,true,false);
=======
     ITryToOpenFile(Bounds,dir+srec.name,CurX,CurY,tryexts,true,false);    
>>>>>>> graemeg/fixes_2_2
=======
     ITryToOpenFile(Bounds,dir+srec.name,CurX,CurY,tryexts,true,false);    
>>>>>>> origin/fixes_2_2
=======
     ITryToOpenFile(Bounds,dir+srec.name,CurX,CurY,tryexts,true,false);
>>>>>>> origin/fixes_2.4
     FindNext(srec);
   end;
  FindClose(srec);
end;


function LocateSingleSourceFile(const FileName: string; tryexts: boolean): string;
var D : DirStr;
    N : NameStr;
    E : ExtStr;

  function CheckDir(NewDir: DirStr; NewName: NameStr; NewExt: ExtStr): boolean;
  var OK: boolean;
  begin
    NewDir:=CompleteDir(NewDir);
    OK:=ExistsFile(NewDir+NewName+NewExt);
    if OK then begin D:=NewDir; N:=NewName; E:=NewExt; end;
    CheckDir:=OK;
  end;

  function CheckExt(NewExt: ExtStr): boolean;
  var OK: boolean;
  begin
    OK:=false;
    if D<>'' then OK:=CheckDir(D,N,NewExt) else
      if CheckDir('.'+DirSep,N,NewExt) then OK:=true;
    CheckExt:=OK;
  end;

  function TryToLocateIn(const DD : dirstr): boolean;
  var Found: boolean;
  begin
    D:=CompleteDir(DD);
    Found:=true;
    if (E<>'') or (not tryexts) then
     Found:=CheckExt(E)
    else
     if CheckExt('.pp') then
      Found:=true
    else
     if CheckExt('.pas') then
      Found:=true
    else
     if CheckExt('.inc') then
      Found:=true
    { try also without extension if no other exist }
    else
     if CheckExt('') then
      Found:=true
    else
      Found:=false;
    TryToLocateIn:=Found;
  end;
var Path,DrStr: string;
    Found: boolean;
begin
  FSplit(FileName,D,N,E);
  Found:=CheckDir(D,N,E);
  if not found then
    Found:=TryToLocateIn('.');
  DrStr:=GetSourceDirectories;
  if not Found then
   While pos(ListSeparator,DrStr)>0 do
    Begin
      Found:=TryToLocateIn(Copy(DrStr,1,pos(ListSeparator,DrStr)-1));
      if Found then
        break;
      DrStr:=Copy(DrStr,pos(ListSeparator,DrStr)+1,High(DrStr));
    End;
  if Found then Path:=FExpand(D+N+E) else Path:='';
  LocateSingleSourceFile:=Path;
end;

function LocateSourceFile(const FileName: string; tryexts: boolean): string;
var P: integer;
    FN,S: string;
    FFN: string;
begin
  FN:=FileName;
  repeat
    P:=Pos(ListSeparator,FN); if P=0 then P:=length(FN)+1;
    S:=copy(FN,1,P-1); Delete(FN,1,P);
    FFN:=LocateSingleSourceFile(S,tryexts);
  until (FFN<>'') or (FN='');
  LocateSourceFile:=FFN;
end;

function ITryToOpenFile(Bounds: PRect; FileName: string; CurX,CurY: sw_integer;tryexts:boolean;
         ShowIt,ForceNewWindow: boolean): PSourceWindow;
var
  W : PSourceWindow;
  DrStr: string;
begin
  W:=nil;
  if ForceNewWindow then
    W:=nil
  else
    W:=SearchOnDesktop(FileName,tryexts);
  if W<>nil then
    begin
      NewEditorOpened:=false;
{      if assigned(Bounds) then
        W^.ChangeBounds(Bounds^);}
      W^.Editor^.SetCurPtr(CurX,CurY);
    end
  else
    begin
      DrStr:=LocateSourceFile(FileName,tryexts);
      if DrStr<>'' then
        W:=IOpenEditorWindow(Bounds,DrStr,CurX,CurY,ShowIt);
      NewEditorOpened:=W<>nil;
      if assigned(W) then
        W^.Editor^.SetCurPtr(CurX,CurY);
    end;
  ITryToOpenFile:=W;
end;

function StartEditor(Editor: PCodeEditor; FileName: string): boolean;
var OK: boolean;
    E: PFileEditor;
    R: TRect;
begin
  R.Assign(0,0,0,0);
  New(E, Init(R,nil,nil,nil,nil,FileName));
  OK:=E<>nil;
  if OK then
  begin
    PushStatus(FormatStrStr(msg_readingfileineditor,FileName));
    OK:=E^.LoadFile;
    PopStatus;
   end;
  if OK then
    begin
      Editor^.Lock;
      E^.SelectAll(true);
      Editor^.InsertFrom(E);
      Editor^.SetCurPtr(0,0);
      Editor^.SelectAll(false);
      Editor^.UnLock;
      Dispose(E, Done);
    end;
  StartEditor:=OK;
end;

constructor TTextScroller.Init(var Bounds: TRect; ASpeed: integer; AText: PUnsortedStringCollection);
begin
  inherited Init(Bounds,'');
  EventMask:=EventMask or evIdle;
  Speed:=ASpeed; Lines:=AText;
end;

function TTextScroller.GetLineCount: integer;
var Count: integer;
begin
  if Lines=nil then Count:=0 else
    Count:=Lines^.Count;
  GetLineCount:=Count;
end;

function TTextScroller.GetLine(I: integer): string;
var S: string;
begin
  if I<Lines^.Count then
    S:=GetStr(Lines^.At(I))
  else
    S:='';
  GetLine:=S;
end;

procedure TTextScroller.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evIdle :
      Update;
  end;
  inherited HandleEvent(Event);
end;

procedure TTextScroller.Update;
begin
  if abs(GetDosTicks-LastTT)<Speed then Exit;
  Scroll;
  LastTT:=GetDosTicks;
end;

procedure TTextScroller.Reset;
begin
  TopLine:=0;
  LastTT:=GetDosTicks;
  DrawView;
end;

procedure TTextScroller.Scroll;
begin
  Inc(TopLine);
  if TopLine>=GetLineCount then
    Reset;
  DrawView;
end;

procedure TTextScroller.Draw;
var B: TDrawBuffer;
    C: word;
    Count,Y: integer;
    S: string;
begin
  C:=GetColor(1);
  Count:=GetLineCount;
  for Y:=0 to Size.Y-1 do
    begin
      if Count=0 then S:='' else
        S:=GetLine((TopLine+Y) mod Count);
      if copy(S,1,1)=^C then
        S:=CharStr(' ',Max(0,(Size.X-(length(S)-1)) div 2))+copy(S,2,255);
      MoveChar(B,' ',C,Size.X);
      MoveStr(B,S,C);
      WriteLine(0,Y,Size.X,1,B);
    end;
end;

destructor TTextScroller.Done;
begin
  inherited Done;
  if Lines<>nil then Dispose(Lines, Done);
end;

constructor TFPAboutDialog.Init;
var R,R2: TRect;
    C: PUnsortedStringCollection;
    I: integer;
    OSStr: string;
procedure AddLine(S: string);
begin
  C^.Insert(NewStr(S));
end;
begin
  R.Assign(0,0,58,14{$ifdef NODEBUG}-1{$endif});
  inherited Init(R, dialog_about);
  HelpCtx:=hcAbout;
  GetExtent(R); R.Grow(-3,-2);
  R2.Copy(R); R2.B.Y:=R2.A.Y+1;
  Insert(New(PStaticText, Init(R2, ^C'Free Pascal IDE for '+source_info.name)));
  R2.Move(0,1);
  Insert(New(PStaticText, Init(R2, ^C'Target CPU: '+target_cpu_string)));
  R2.Move(0,1);
  Insert(New(PStaticText, Init(R2, ^C'Version '+VersionStr+' '+{$i %date%})));
  R2.Move(0,1);
{$ifdef USE_GRAPH_SWITCH}
  Insert(New(PStaticText, Init(R2, ^C'With Graphic Support')));
  R2.Move(0,1);
{$endif USE_GRAPH_SWITCH}
  Insert(New(PStaticText, Init(R2, FormatStrStr2(^C'(%s %s)',label_about_compilerversion,Full_Version_String))));
{$ifndef NODEBUG}
  if pos('Fake',GDBVersion)=0 then
    begin
      R2.Move(0,1);
      Insert(New(PStaticText, Init(R2, FormatStrStr2(^C'(%s %s)',label_about_debugger,GDBVersion))));
      R2.Move(0,1);
    end
  else
{$endif NODEBUG}
    R2.Move(0,2);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Insert(New(PStaticText, Init(R2, ^C'Copyright (C) 1998-2014 by')));
=======
  Insert(New(PStaticText, Init(R2, ^C'Copyright (C) 1998-2009 by')));
>>>>>>> graemeg/cpstrnew
=======
  Insert(New(PStaticText, Init(R2, ^C'Copyright (C) 1998-2009 by')));
>>>>>>> graemeg/cpstrnew
=======
  Insert(New(PStaticText, Init(R2, ^C'Copyright (C) 1998-2009 by')));
>>>>>>> graemeg/cpstrnew
=======
  Insert(New(PStaticText, Init(R2, ^C'Copyright (C) 1998-2009 by')));
>>>>>>> origin/cpstrnew
=======
  Insert(New(PStaticText, Init(R2, ^C'Copyright (C) 1998-2008 by')));
>>>>>>> graemeg/fixes_2_2
=======
  Insert(New(PStaticText, Init(R2, ^C'Copyright (C) 1998-2008 by')));
>>>>>>> origin/fixes_2_2
=======
  Insert(New(PStaticText, Init(R2, ^C'Copyright (C) 1998-2009 by')));
>>>>>>> origin/fixes_2.4
  R2.Move(0,2);
  Insert(New(PStaticText, Init(R2, ^C'B‚rczi G bor')));
  R2.Move(0,1);
  Insert(New(PStaticText, Init(R2, ^C'Pierre Muller')));
  R2.Move(0,1);
  Insert(New(PStaticText, Init(R2, ^C'and')));
  R2.Move(0,1);
  Insert(New(PStaticText, Init(R2, ^C'Peter Vreman')));
  New(C, Init(50,10));
  for I:=1 to 7 do
  AddLine('');
  AddLine(^C'< Original concept >');
  AddLine(^C'Borland International, Inc.');
  AddLine('');
  AddLine(^C'< Compiler development >');
  AddLine(^C'Carl-Eric Codere');
  AddLine(^C'Daniel Mantione');
  AddLine(^C'Florian Kl„mpfl');
  AddLine(^C'Jonas Maebe');
  AddLine(^C'Mich„el Van Canneyt');
  AddLine(^C'Peter Vreman');
  AddLine(^C'Pierre Muller');
  AddLine('');
  AddLine(^C'< IDE development >');
  AddLine(^C'B‚rczi G bor');
  AddLine(^C'Peter Vreman');
  AddLine(^C'Pierre Muller');
  AddLine('');

  GetExtent(R);
  R.Grow(-1,-1); Inc(R.A.Y,3);
  New(Scroller, Init(R, 10, C));
  Scroller^.Hide;
  Insert(Scroller);
  R.Move(0,-1); R.B.Y:=R.A.Y+1;
  New(TitleST, Init(R, ^C'Team'));
  TitleST^.Hide;
  Insert(TitleST);

  InsertOK(@Self);
end;

procedure TFPAboutDialog.ToggleInfo;
begin
  if Scroller=nil then Exit;
  if Scroller^.GetState(sfVisible) then
    begin
      Scroller^.Hide;
      TitleST^.Hide;
    end
  else
    begin
      Scroller^.Reset;
      Scroller^.Show;
      TitleST^.Show;
    end;
end;

procedure TFPAboutDialog.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evKeyDown :
      case Event.KeyCode of
        kbAltI : { just like in BP }
          begin
            ToggleInfo;
            ClearEvent(Event);
          end;
      end;
  end;
  inherited HandleEvent(Event);
end;

constructor TFPASCIIChart.Init;
begin
  inherited Init;
  HelpCtx:=hcASCIITableWindow;
  Number:=SearchFreeWindowNo;
  ASCIIChart:=@Self;
end;

procedure TFPASCIIChart.Store(var S: TStream);
begin
  inherited Store(S);
end;

constructor TFPASCIIChart.Load(var S: TStream);
begin
  inherited Load(S);
end;

procedure TFPASCIIChart.HandleEvent(var Event: TEvent);
var W: PSourceWindow;
begin
  {writeln(stderr,'all what=',event.what,' cmd=', event.command);}
  case Event.What of
    evKeyDown :
      case Event.KeyCode of
        kbEsc :
          begin
            Close;
            ClearEvent(Event);
          end;
      end;
    evCommand :
      begin
      {writeln(stderr,'fpascii what=',event.what, ' cmd=', event.command, ' ',cmtransfer,' ',cmsearchwindow);}
      if Event.Command=(AsciiTableCommandBase+1) then // variable
          begin
            W:=FirstEditorWindow;
            if Assigned(W) and Assigned(Report) then
              Message(W,evCommand,cmAddChar,Event.InfoPtr);
            ClearEvent(Event);
          end
      else
      case Event.Command of
        cmTransfer :
          begin
            W:=FirstEditorWindow;
            if Assigned(W) and Assigned(Report) then
              Message(W,evCommand,cmAddChar,pointer(ptrint(ord(Report^.AsciiChar))));
            ClearEvent(Event);
          end;

        cmSearchWindow+1..cmSearchWindow+99 :
          if (Event.Command-cmSearchWindow=Number) then
              ClearEvent(Event);
      end;
      end;
  end;
  inherited HandleEvent(Event);
end;

destructor TFPASCIIChart.Done;
begin
  ASCIIChart:=nil;
  inherited Done;
end;

function TVideoModeListBox.GetText(Item: pointer; MaxLen: sw_integer): string;
var P: PVideoMode;
    S: string;
begin
  P:=Item;
  S:=IntToStr(P^.Col)+'x'+IntToStr(P^.Row)+' ';
  if P^.Color then
    S:=S+'color'
  else
    S:=S+'mono';
  GetText:=copy(S,1,MaxLen);
end;

constructor TFPDesktop.Init(var Bounds: TRect);
begin
  inherited Init(Bounds);
end;

procedure TFPDesktop.InitBackground;
var AV: PANSIBackground;
    FileName: string;
    R: TRect;
begin
  AV:=nil;
  FileName:=LocateFile(BackgroundPath);
  if FileName<>'' then
  begin
    GetExtent(R);
    New(AV, Init(R));
    AV^.GrowMode:=gfGrowHiX+gfGrowHiY;
    if AV^.LoadFile(FileName)=false then
    begin
      Dispose(AV, Done); AV:=nil;
    end;
    if Assigned(AV) then
      Insert(AV);
  end;
  Background:=AV;
  if Assigned(Background)=false then
    inherited InitBackground;
end;

constructor TFPDesktop.Load(var S: TStream);
begin
  inherited Load(S);
end;

procedure TFPDesktop.Store(var S: TStream);
begin
  inherited Store(S);
end;

constructor TFPToolTip.Init(var Bounds: TRect; const AText: string; AAlign: TAlign);
begin
  inherited Init(Bounds);
  SetAlign(AAlign);
  SetText(AText);
end;

procedure TFPToolTip.Draw;
var C: word;
procedure DrawLine(Y: integer; S: string);
var B: TDrawBuffer;
begin
  S:=copy(S,1,Size.X-2);
  case Align of
    alLeft   : S:=' '+S;
    alRight  : S:=LExpand(' '+S,Size.X);
    alCenter : S:=Center(S,Size.X);
  end;
  MoveChar(B,' ',C,Size.X);
  MoveStr(B,S,C);
  WriteLine(0,Y,Size.X,1,B);
end;
var S: string;
    Y: integer;
begin
  C:=GetColor(1);
  S:=GetText;
  for Y:=0 to Size.Y-1 do
    DrawLine(Y,S);
end;

function TFPToolTip.GetText: string;
begin
  GetText:=GetStr(Text);
end;

procedure TFPToolTip.SetText(const AText: string);
begin
  if AText<>GetText then
  begin
    if Assigned(Text) then DisposeStr(Text);
    Text:=NewStr(AText);
    DrawView;
  end;
end;

function TFPToolTip.GetAlign: TAlign;
begin
  GetAlign:=Align;
end;

procedure TFPToolTip.SetAlign(AAlign: TAlign);
begin
  if AAlign<>Align then
  begin
    Align:=AAlign;
    DrawView;
  end;
end;

destructor TFPToolTip.Done;
begin
  if Assigned(Text) then DisposeStr(Text); Text:=nil;
  inherited Done;
end;

function TFPToolTip.GetPalette: PPalette;
const S: string[length(CFPToolTip)] = CFPToolTip;
begin
  GetPalette:=@S;
end;

constructor TFPMemo.Init(var Bounds: TRect; AHScrollBar, AVScrollBar:
          PScrollBar; AIndicator: PIndicator);
begin
  inherited Init(Bounds,AHScrollBar,AVScrollBar,AIndicator,nil);
  SetFlags(Flags and not (efPersistentBlocks) or efSyntaxHighlight);
end;

procedure TFPMemo.HandleEvent(var Event: TEvent);
var DontClear: boolean;
    S: string;
begin
  case Event.What of
    evKeyDown :
      begin
        DontClear:=false;
        case Event.KeyCode of
          kbEsc:
            begin
              Event.What:=evCommand;
              Event.Command:=cmCancel;
              PutEvent(Event);
            end;
        else DontClear:=true;
        end;
        if not DontClear then ClearEvent(Event);
      end;
  end;
  inherited HandleEvent(Event);
end;

function TFPMemo.GetPalette: PPalette;
const P: string[length(CFPMemo)] = CFPMemo;
begin
  GetPalette:=@P;
end;

function TFPMemo.GetSpecSymbolCount(SpecClass: TSpecSymbolClass): integer;
begin
  GetSpecSymbolCount:=0;
end;

function TFPMemo.GetSpecSymbol(SpecClass: TSpecSymbolClass; Index: integer): pstring;
begin
  Abstract;
  GetSpecSymbol:=nil;
end;

function TFPMemo.IsReservedWord(const S: string): boolean;
begin
  IsReservedWord:=false;
end;

constructor TFPCodeMemo.Init(var Bounds: TRect; AHScrollBar, AVScrollBar:
          PScrollBar; AIndicator: PIndicator);
begin
  inherited Init(Bounds,AHScrollBar,AVScrollBar,AIndicator);
end;

function TFPCodeMemo.GetSpecSymbolCount(SpecClass: TSpecSymbolClass): integer;
begin
  GetSpecSymbolCount:=FreePascalSpecSymbolCount[SpecClass];
end;

function TFPCodeMemo.GetSpecSymbol(SpecClass: TSpecSymbolClass; Index: integer): pstring;
begin
  GetSpecSymbol:=@FreePascalEmptyString;
  case SpecClass of
    ssCommentPrefix :
      case Index of
        0 : GetSpecSymbol:=@FreePascalCommentPrefix1;
        1 : GetSpecSymbol:=@FreePascalCommentPrefix2;
        2 : GetSpecSymbol:=@FreePascalCommentPrefix3;
      end;
    ssCommentSingleLinePrefix :
      case Index of
        0 : GetSpecSymbol:=@FreePascalCommentSingleLinePrefix;
      end;
    ssCommentSuffix :
      case Index of
        0 : GetSpecSymbol:=@FreePascalCommentSuffix1;
        1 : GetSpecSymbol:=@FreePascalCommentSuffix2;
      end;
    ssStringPrefix :
      GetSpecSymbol:=@FreePascalStringPrefix;
    ssStringSuffix :
      GetSpecSymbol:=@FreePascalStringSuffix;
    { must be uppercased to avoid calling UpCaseStr in MatchesAnyAsmSymbol PM }
    ssAsmPrefix :
      GetSpecSymbol:=@FreePascalAsmPrefix;
    ssAsmSuffix :
      GetSpecSymbol:=@FreePascalAsmSuffix;
    ssDirectivePrefix :
      GetSpecSymbol:=@FreePascalDirectivePrefix;
    ssDirectiveSuffix :
      GetSpecSymbol:=@FreePascalDirectiveSuffix;
  end;
end;

function TFPCodeMemo.IsReservedWord(const S: string): boolean;
begin
  IsReservedWord:=IsFPReservedWord(S);
end;


{$ifdef VESA}
function VESASetVideoModeProc(const VideoMode: TVideoMode; Params: Longint): Boolean;
begin
  VESASetVideoModeProc:=VESASetMode(Params);
end;

procedure InitVESAScreenModes;
var ML: TVESAModeList;
    MI: TVESAModeInfoBlock;
    I: integer;
begin
  if VESAInit=false then Exit;
  if VESAGetModeList(ML)=false then Exit;
  for I:=1 to ML.Count do
    begin
      if VESAGetModeInfo(ML.Modes[I],MI) then
      with MI do
{$ifndef DEBUG}
        if (Attributes and vesa_vma_GraphicsMode)=0 then
{$else DEBUG}
        if ((Attributes and vesa_vma_GraphicsMode)=0) or
        { only allow 4 bit i.e. 16 color modes }
          (((Attributes and vesa_vma_CanBeSetInCurrentConfig)<>0) and
           (BitsPerPixel=8)) then
{$endif DEBUG}
          RegisterVesaVideoMode(ML.Modes[I]);
    end;
end;

procedure DoneVESAScreenModes;
begin
  FreeVesaModes;
end;
{$endif}

procedure NoDebugger;
begin
  InformationBox(msg_nodebuggersupportavailable,nil);
end;

procedure RegisterFPViews;
begin
  RegisterType(RSourceEditor);
  RegisterType(RSourceWindow);
  RegisterType(RFPHelpViewer);
  RegisterType(RFPHelpWindow);
  RegisterType(RClipboardWindow);
  RegisterType(RMessageListBox);
  RegisterType(RFPDesktop);
  RegisterType(RFPASCIIChart);
  RegisterType(RFPDlgWindow);
{$ifndef NODEBUG}
  RegisterType(RGDBWindow);
  RegisterType(RGDBSourceEditor);
{$endif NODEBUG}
end;


END.
