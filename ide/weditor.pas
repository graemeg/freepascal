{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998 by Berczi Gabor

    Code editor template objects

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$I globdir.inc}
unit WEditor;

interface
{tes}
uses
  Dos,Objects,Drivers,Views,Dialogs,Menus,
  FVConsts,
  WUtils,WViews;

const
      cmFileNameChanged      = 51234;
      cmASCIIChar            = 51235;
      cmClearLineHighlights  = 51236;
      cmSaveCancelled        = 51237;
      cmBreakLine            = 51238;
      cmSelStart             = 51239;
      cmSelEnd               = 51240;
      cmLastCursorPos        = 51241;
      cmIndentBlock          = 51242;
      cmUnIndentBlock        = 51243;
      cmSelectLine           = 51244;
      cmWriteBlock           = 51245;
      cmReadBlock            = 51246;
      cmPrintBlock           = 51247;
      cmResetDebuggerRow     = 51248;
      cmAddChar              = 51249;
      cmExpandCodeTemplate   = 51250;
      cmUpperCase            = 51251;
      cmLowerCase            = 51252;
      cmWindowStart          = 51253;
      cmWindowEnd            = 51254;
      cmFindMatchingDelimiter= 51255;
      cmFindMatchingDelimiterBack=51256;
      cmActivateMenu         = 51257;
      cmWordLowerCase        = 51258;
      cmWordUpperCase        = 51259;
      cmOpenAtCursor         = 51260;
      cmBrowseAtCursor       = 51261;
      cmInsertOptions        = 51262;
      cmToggleCase           = 51263;
      cmCreateFold           = 51264;
      cmToggleFold           = 51265;
      cmCollapseFold         = 51266;
      cmExpandFold           = 51267;
      cmDelToEndOfWord       = 51268;
<<<<<<< HEAD
      cmInputLineLen         = 51269;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
      
>>>>>>> graemeg/cpstrnew
=======
      
>>>>>>> graemeg/cpstrnew
=======
      
>>>>>>> graemeg/cpstrnew
=======
      
>>>>>>> origin/cpstrnew
      EditorTextBufSize = 32768;
      MaxLineLength     = 255;
      MaxLineCount      = 2000000;
=======

      EditorTextBufSize = 32768;
      MaxLineLength     = 255;
      MaxLineCount      = 2000000;

>>>>>>> graemeg/fixes_2_2

      CodeTemplateCursorChar = '|'; { char to signal cursor pos in templates }

      efBackupFiles         = $00000001;
      efInsertMode          = $00000002;
      efAutoIndent          = $00000004;
      efUseTabCharacters    = $00000008;
      efBackSpaceUnindents  = $00000010;
      efPersistentBlocks    = $00000020;
      efSyntaxHighlight     = $00000040;
      efBlockInsCursor      = $00000080;
      efVerticalBlocks      = $00000100;
      efHighlightColumn     = $00000200;
      efHighlightRow        = $00000400;
      efAutoBrackets        = $00000800;
      efExpandAllTabs       = $00001000;
      efKeepTrailingSpaces  = $00002000;
      efCodeComplete        = $00004000;
      efFolds               = $00008000;
      efNoIndent            = $00010000;
      efKeepLineAttr        = $00020000;
      efStoreContent        = $80000000;

      attrAsm       = 1;
      attrComment   = 2;
      attrForceFull = 128;
      attrAll       = attrAsm+attrComment;

      edOutOfMemory   = 0;
      edReadError     = 1;
      edWriteError    = 2;
      edCreateError   = 3;
      edSaveModify    = 4;
      edSaveUntitled  = 5;
      edSaveAs        = 6;
      edFind          = 7;
      edSearchFailed  = 8;
      edReplace       = 9;
      edReplacePrompt = 10;
      edTooManyLines  = 11;
      edGotoLine      = 12;
      edReplaceFile   = 13;
      edWriteBlock    = 14;
      edReadBlock     = 15;
      edFileOnDiskChanged = 16;
      edChangedOnloading = 17;
      edSaveError     = 18;
      edReloadDiskmodifiedFile = 19;
      edReloadDiskAndIDEModifiedFile = 20;

      ffmOptions      = $0007; ffsOptions     = 0;
      ffmDirection    = $0008; ffsDirection   = 3;
      ffmScope        = $0010; ffsScope       = 4;
      ffmOrigin       = $0020; ffsOrigin      = 5;
      ffDoReplace     = $0040;
      ffReplaceAll    = $0080;


      ffCaseSensitive    = $0001;
      ffWholeWordsOnly   = $0002;
      ffPromptOnReplace  = $0004;

      ffForward          = $0000;
      ffBackward         = $0008;

      ffGlobal           = $0000;
      ffSelectedText     = $0010;

      ffFromCursor       = $0000;
      ffEntireScope      = $0020;

{$ifdef TEST_REGEXP}
      ffUseRegExp        = $0100;
      ffmUseRegExpFind   = $0004;
      ffmOptionsFind     = $0003;
      ffsUseRegExpFind   = 8 - 2;
      ffmUseRegExpReplace = $0008;
      ffsUseRegExpReplace = 8 - 3;
{$endif TEST_REGEXP}

      coTextColor         = 0;
      coWhiteSpaceColor   = 1;
      coCommentColor      = 2;
      coReservedWordColor = 3;
      coIdentifierColor   = 4;
      coStringColor       = 5;
      coNumberColor       = 6;
      coAssemblerColor    = 7;
      coSymbolColor       = 8;
      coDirectiveColor    = 9;
      coHexNumberColor    = 10;
      coTabColor          = 11;
      coAsmReservedColor  = 12;
      coBreakColor        = 13;
      coFirstColor        = 0;
      coLastColor         = coBreakColor;

      lfBreakpoint        = $0001;
      lfHighlightRow      = $0002;
      lfDebuggerRow       = $0004;
      lfSpecialRow        = $0008;

      eaMoveCursor        = 1;
      eaInsertLine        = 2;
      eaInsertText        = 3;
      eaDeleteLine        = 4;
      eaDeleteText        = 5;
      eaSelectionChanged  = 6;
      eaCut               = 7;
      eaPaste             = 8;
      eaPasteWin          = 9;
      eaDelChar           = 10;
      eaClear             = 11;
      eaCopyBlock         = 12;
      eaMoveBlock         = 13;
      eaDelBlock          = 14;
      eaReadBlock         = 15;
      eaIndentBlock       = 16;
      eaUnindentBlock     = 17;
      eaOverwriteText     = 18;
      eaUpperCase         = 19;
      eaLowerCase         = 20;
      eaToggleCase        = 21;
      eaDummy             = 22;
      LastAction          = eaDummy;

      ActionString : array [0..LastAction-1] of string[13] =
        ('','Move','InsLine','InsText','DelLine','DelText',
         'SelChange','Cut','Paste','PasteWin','DelChar','Clear',
         'CopyBlock','MoveBlock','DelBlock',
         'ReadBlock','IndentBlock','UnindentBlock','Overwrite',
         'UpperCase','LowerCase','ToggleCase');

      CIndicator    = #2#3#1;
      CEditor       = #33#34#35#36#37#38#39#40#41#42#43#44#45#46#47#48#49#50;

      TAB      = #9;
      FindStrSize = 79;


type
    Tcentre = (do_not_centre,do_centre);

    PCustomCodeEditor = ^TCustomCodeEditor;
    PEditorLineInfo = ^TEditorLineInfo;
    PFoldCollection = ^TFoldCollection;

    PFold = ^TFold;
    TFold = object(TObject)
      constructor Init(AEditor: PCustomCodeEditor; AParentFold: PFold; ACollapsed: boolean);
      procedure   AddReference(P: PObject);
      procedure   RemoveReference(P: PObject);
      procedure   AddLineReference(Line: PEditorLineInfo);
      procedure   RemoveLineReference(Line: PEditorLineInfo);
      procedure   AddChildReference(Fold: PFold);
      procedure   RemoveChildReference(Fold: PFold);
      function    CanDispose: boolean;
      function    IsCollapsed: boolean;
      function    IsParent(AFold: PFold): boolean;
      function    GetLineCount: sw_integer;
      procedure   Collapse(ACollapse: boolean);
      procedure   Changed;
      function    GetLevel: sw_integer;
      destructor  Done; virtual;
    public
      ParentFold: PFold;
      Collapsed_: boolean;
      ReferenceCount: sw_integer;
      Editor: PCustomCodeEditor;
      LineCount_: sw_integer;
      Childs: PFoldCollection;
    end;

    TFoldCollection = object(TCollection)
      function At(Index: sw_Integer): PFold;
    end;

    TEditorLineInfo = object(TObject)
      Editor: PCustomCodeEditor;
      Format : PString;
      BeginsWithAsm,
      EndsWithAsm   : boolean;
      BeginsWithComment,
      EndsInSingleLineComment,
      EndsWithComment : boolean;
      BeginsWithDirective,
      EndsWithDirective : boolean;
      BeginCommentType,EndCommentType : byte;
      Fold: PFold;
      constructor Init(AEditor: PCustomCodeEditor);
      destructor  Done; virtual;
      function    GetFormat: string;
      procedure   SetFormat(const AFormat: string);
      procedure   SetFold(AFold: PFold);
      { Syntax information is now generated separately for each editor instance.
        This is not neccessary for a one-language IDE, but this unit contains
        a _generic_ editor object, which should be (and is) as flexible as
        possible.
        The overhead caused by generating the same syntax info for ex.
        twice isn't so much...   - Gabor }
    end;

    PEditorLineInfoCollection = ^TEditorLineInfoCollection;
    TEditorLineInfoCollection = object(TCollection)
      function At(Index: sw_Integer): PEditorLineInfo;
    end;

    PCustomLine = ^TCustomLine;
    TCustomLine = object(TObject)
      constructor Init(const AText: string; AFlags: longint);
   {a}function    GetText: string; virtual;
   {a}procedure   SetText(const AText: string); virtual;
   {a}function    GetEditorInfo(Editor: PCustomCodeEditor): PEditorLineInfo; virtual;
   {a}function    GetFlags: longint; virtual;
   {a}procedure   SetFlags(AFlags: longint); virtual;
      function    IsFlagSet(AFlag: longint): boolean; {$ifdef USEINLINE}inline;{$endif}
      procedure   SetFlagState(AFlag: longint; ASet: boolean);
      destructor  Done; virtual;
    public { internal use only! }
   {a}procedure AddEditorInfo(Index: sw_integer; AEditor: PCustomCodeEditor); virtual;
   {a}procedure RemoveEditorInfo(AEditor: PCustomCodeEditor); virtual;
    end;

    PLineCollection = ^TLineCollection;
    TLineCollection = object(TCollection)
      function  At(Index: sw_Integer): PCustomLine;
    end;

    PEditorAction = ^TEditorAction;
    TEditorAction = object(TObject)
      StartPos  : TPoint;
      EndPos    : TPoint;
      Text      : PString;
      ActionCount : longint;
      Flags : longint;
      Action    : byte;
      IsGrouped : boolean;
      TimeStamp : longint; { this is needed to keep track of line number &
                             position changes (for ex. for symbol browser)
                             the line&pos references (eg. symbol info) should
                             also contain such a timestamp. this will enable
                             to determine which changes have been made since
                             storage of the information and thus calculate
                             the (probably) changed line & position information,
                             so, we can still jump to the right position in the
                             editor even when it is heavily modified - Gabor }
      constructor init(act:byte; StartP,EndP:TPoint;Txt:String;AFlags : longint);
      constructor init_group(act:byte);
      function is_grouped_action : boolean;
      destructor done; virtual;
    end;

    PEditorActionCollection = ^TEditorActionCollection;
    TEditorActionCollection = object(TCollection)
      CurrentGroupedAction : PEditorAction;
      GroupLevel           : longint;
      function At(Idx : sw_integer) : PEditorAction;
    end;

    TSpecSymbolClass =
      (ssCommentPrefix,ssCommentSingleLinePrefix,ssCommentSuffix,ssStringPrefix,ssStringSuffix,
       ssDirectivePrefix,ssDirectiveSuffix,ssAsmPrefix,ssAsmSuffix);

    TEditorBookMark = record
      Valid  : boolean;
      Pos    : TPoint;
    end;

    TCompleteState = (csInactive,csOffering,csDenied);

    PEditorBinding = ^TEditorBinding;

    PEditorBindingCollection = ^TEditorBindingCollection;
    TEditorBindingCollection = object(TCollection)
      function At(Index: sw_Integer): PEditorBinding;
    end;

    TEditorBinding = object(TObject)
      Editor : PCustomCodeEditor;
      constructor Init(AEditor: PCustomCodeEditor);
      destructor  Done; virtual;
    end;

    PCustomCodeEditorCore = ^TCustomCodeEditorCore;
    TCustomCodeEditorCore = object(TObject)
    protected
      Bindings    : PEditorBindingCollection;
      LockFlag    : sw_integer;
      ChangedLine : sw_integer;
      ContentsChangedCalled : boolean;
      LimitsChangedCalled : boolean;
      ModifiedChangedCalled : boolean;
      TabSizeChangedCalled : boolean;
      StoreUndoChangedCalled : boolean;
{$ifdef TEST_PARTIAL_SYNTAX}
      LastSyntaxedLine : sw_integer;
      SyntaxComplete   : boolean;
{$endif TEST_PARTIAL_SYNTAX}
    public
      constructor Init;
      procedure   BindEditor(AEditor: PCustomCodeEditor);
      procedure   UnBindEditor(AEditor: PCustomCodeEditor);
      function    IsEditorBound(AEditor: PCustomCodeEditor): boolean;
      function    GetBindingCount: sw_integer;
      function    GetBindingIndex(AEditor: PCustomCodeEditor): sw_integer;
      function    SearchBinding(AEditor: PCustomCodeEditor): PEditorBinding;
      function    CanDispose: boolean;
      destructor  Done; virtual;
    public
   {a}function    GetModified: boolean; virtual;
      function    GetChangedLine: sw_integer;
   {a}procedure   SetModified(AModified: boolean); virtual;
   {a}function    GetStoreUndo: boolean; virtual;
   {a}procedure   SetStoreUndo(AStore: boolean); virtual;
   {a}function    GetSyntaxCompleted: boolean; virtual;
   {a}procedure   SetSyntaxCompleted(SC: boolean); virtual;
   {a}function    GetTabSize: integer; virtual;
   {a}procedure   SetTabSize(ATabSize: integer); virtual;
   {a}function    GetIndentSize: integer; virtual;
   {a}procedure   SetIndentSize(AIndentSize: integer); virtual;
      function    IsClipboard: Boolean;
    public
      { Notifications }
      procedure   BindingsChanged;
      procedure   ContentsChanged;
      procedure   LimitsChanged;
      procedure   ModifiedChanged;
      procedure   TabSizeChanged;
      procedure   StoreUndoChanged;
   {a}procedure   DoContentsChanged; virtual;
   {a}procedure   DoLimitsChanged; virtual;
   {a}procedure   DoModifiedChanged; virtual;
   {a}procedure   DoTabSizeChanged; virtual;
   {a}procedure   DoStoreUndoChanged; virtual;
   {a}procedure   DoSyntaxStateChanged; virtual;
      function    GetLastVisibleLine : sw_integer;
    public
      { Storage }
      function    LoadFromStream(Editor: PCustomCodeEditor; Stream: PFastBufStream): boolean; virtual;
      function    SaveToStream(Editor: PCustomCodeEditor; Stream: PStream): boolean; virtual;
      function    SaveAreaToStream(Editor: PCustomCodeEditor; Stream: PStream; StartP,EndP: TPoint): boolean; virtual;
    protected
      { Text & info storage abstraction }
   {a}procedure   ISetLineFlagState(Binding: PEditorBinding; LineNo: sw_integer; Flag: longint; ASet: boolean); virtual;
   {a}procedure   IGetDisplayTextFormat(Binding: PEditorBinding; LineNo: sw_integer;var DT,DF:string); virtual;
   {a}function    IGetLineFormat(Binding: PEditorBinding; LineNo: sw_integer): string; virtual;
   {a}procedure   ISetLineFormat(Binding: PEditorBinding; LineNo: sw_integer;const S: string); virtual;
    public
      { Text & info storage abstraction }
      function    CharIdxToLinePos(Line,CharIdx: sw_integer): sw_integer;
      function    LinePosToCharIdx(Line,X: sw_integer): sw_integer;
   {a}function    GetLineCount: sw_integer; virtual;
   {a}function    GetLine(LineNo: sw_integer): PCustomLine; virtual;
   {a}function    GetLineText(LineNo: sw_integer): string; virtual;
   {a}procedure   SetDisplayText(I: sw_integer;const S: string); virtual;
   {a}function    GetDisplayText(I: sw_integer): string; virtual;
   {a}procedure   SetLineText(I: sw_integer;const S: string); virtual;
      procedure   GetDisplayTextFormat(Editor: PCustomCodeEditor; I: sw_integer;var DT,DF:string); virtual;
      function    GetLineFormat(Editor: PCustomCodeEditor; I: sw_integer): string; virtual;
      procedure   SetLineFormat(Editor: PCustomCodeEditor; I: sw_integer;const S: string); virtual;
   {a}procedure   DeleteAllLines; virtual;
   {a}procedure   DeleteLine(I: sw_integer); virtual;
   {a}function    InsertLine(LineNo: sw_integer; const S: string): PCustomLine; virtual;
   {a}procedure   AddLine(const S: string); virtual;
   {a}procedure   GetContent(ALines: PUnsortedStringCollection); virtual;
   {a}procedure   SetContent(ALines: PUnsortedStringCollection); virtual;
   public
      procedure   Lock(AEditor: PCustomCodeEditor);
      procedure   UnLock(AEditor: PCustomCodeEditor);
      function    Locked: boolean;
   public
      { Syntax highlight }
      function    UpdateAttrs(FromLine: sw_integer; Attrs: byte): sw_integer; virtual;
      function    UpdateAttrsRange(FromLine, ToLine: sw_integer; Attrs: byte): sw_integer; virtual;
      function    DoUpdateAttrs(Editor: PCustomCodeEditor; FromLine: sw_integer; Attrs: byte): sw_integer; virtual;
      function    DoUpdateAttrsRange(Editor: PCustomCodeEditor; FromLine, ToLine: sw_integer;
                  Attrs: byte): sw_integer; virtual;
   public
     { Undo info storage }
   {a}procedure   AddAction(AAction: byte; AStartPos, AEndPos: TPoint; AText: string;AFlags : longint); virtual;
   {a}procedure   AddGroupedAction(AAction : byte); virtual;
   {a}procedure   CloseGroupedAction(AAction : byte); virtual;
   {a}function    GetUndoActionCount: sw_integer; virtual;
   {a}function    GetRedoActionCount: sw_integer; virtual;
      procedure   UpdateUndoRedo(cm : word; action : byte);virtual;
    end;

    TCaseAction = (caToLowerCase,caToUpperCase,caToggleCase);

    TCustomCodeEditor = object(TScroller)
      SelStart   : TPoint;
      SelEnd     : TPoint;
      Highlight  : TRect;
      CurPos     : TPoint;
      ELockFlag   : integer;
      NoSelect   : Boolean;
      AlwaysShowScrollBars: boolean;
   public
{      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);}
      procedure   ConvertEvent(var Event: TEvent); virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
      procedure   SetState(AState: Word; Enable: Boolean); virtual;
      procedure   LocalMenu(P: TPoint); virtual;
      function    GetLocalMenu: PMenu; virtual;
      function    GetCommandTarget: PView; virtual;
      function    CreateLocalMenuView(var Bounds: TRect; M: PMenu): PMenuPopup; virtual;
      function    GetPalette: PPalette; virtual;
    public
      procedure   Draw; virtual;
      procedure   DrawCursor; virtual;
      { this is the only way I found to avoid
        having the cursor being updated if lock is on PM }
      procedure   ResetCursor; virtual;
      procedure   DrawIndicator; virtual;
    public
   {a}function    GetFlags: longint; virtual;
   {a}procedure   SetFlags(AFlags: longint); virtual;
   {a}function    GetModified: boolean; virtual;
   {a}procedure   SetModified(AModified: boolean); virtual;
   {a}function    GetStoreUndo: boolean; virtual;
   {a}procedure   SetStoreUndo(AStore: boolean); virtual;
   {a}function    GetSyntaxCompleted: boolean; virtual;
   {a}procedure   SetSyntaxCompleted(SC: boolean); virtual;
   {a}function    GetLastSyntaxedLine: sw_integer; virtual;
   {a}procedure   SetLastSyntaxedLine(ALine: sw_integer); virtual;
      function    IsFlagSet(AFlag: longint): boolean;{$ifdef USEINLINE}inline;{$endif}
      function    GetReservedColCount: sw_integer; virtual;
   {a}function    GetTabSize: integer; virtual;
   {a}procedure   SetTabSize(ATabSize: integer); virtual;
   {a}function    GetIndentSize: integer; virtual;
   {a}procedure   SetIndentSize(AIndentSize: integer); virtual;
   {a}function    IsReadOnly: boolean; virtual;
   {a}function    IsClipboard: Boolean; virtual;
   {a}function    GetInsertMode: boolean; virtual;
   {a}procedure   SetInsertMode(InsertMode: boolean); virtual;
      procedure   SetCurPtr(X,Y: sw_integer); virtual;
      procedure   GetSelectionArea(var StartP,EndP: TPoint); virtual;
      procedure   SetSelection(A, B: TPoint); virtual;
      procedure   SetHighlight(A, B: TPoint); virtual;
      procedure   ChangeCaseArea(StartP,EndP: TPoint; CaseAction: TCaseAction); virtual;
      procedure   SetLineFlagState(LineNo: sw_integer; Flags: longint; ASet: boolean);
      procedure   SetLineFlagExclusive(Flags: longint; LineNo: sw_integer);
      procedure   Update; virtual;
      procedure   ScrollTo(X, Y: sw_Integer);
      procedure   TrackCursor(centre:Tcentre); virtual;
      procedure   Lock; virtual;
      procedure   UnLock; virtual;
    public
      { Text & info storage abstraction }
   {a}function    GetLineCount: sw_integer; virtual;
   {a}function    GetLine(LineNo: sw_integer): PCustomLine; virtual;
   {a}function    CharIdxToLinePos(Line,CharIdx: sw_integer): sw_integer; virtual;
   {a}function    LinePosToCharIdx(Line,X: sw_integer): sw_integer; virtual;
   {a}function    GetLineText(I: sw_integer): string; virtual;
   {a}procedure   SetDisplayText(I: sw_integer;const S: string); virtual;
   {a}function    GetDisplayText(I: sw_integer): string; virtual;
   {a}procedure   SetLineText(I: sw_integer;const S: string); virtual;
   {a}procedure   GetDisplayTextFormat(I: sw_integer;var DT,DF:string); virtual;
   {a}function    GetLineFormat(I: sw_integer): string; virtual;
   {a}procedure   SetLineFormat(I: sw_integer;const S: string); virtual;
   {a}procedure   DeleteAllLines; virtual;
   {a}procedure   DeleteLine(I: sw_integer); virtual;
   {a}function    InsertLine(LineNo: sw_integer; const S: string): PCustomLine; virtual;
   {a}procedure   AddLine(const S: string); virtual;
   {a}function    GetErrorMessage: string; virtual;
   {a}procedure   SetErrorMessage(const S: string); virtual;
   {a}procedure   AdjustSelection(DeltaX, DeltaY: sw_integer);
   {a}procedure   AdjustSelectionBefore(DeltaX, DeltaY: sw_integer);
   {a}procedure   AdjustSelectionPos(OldCurPosX, OldCurPosY: sw_integer; DeltaX, DeltaY: sw_integer);
   {a}procedure   GetContent(ALines: PUnsortedStringCollection); virtual;
   {a}procedure   SetContent(ALines: PUnsortedStringCollection); virtual;
   {a}function    LoadFromStream(Stream: PFastBufStream): boolean; virtual;
   {a}function    SaveToStream(Stream: PStream): boolean; virtual;
   {a}function    SaveAreaToStream(Stream: PStream; StartP,EndP: TPoint): boolean;virtual;
      function    LoadFromFile(const AFileName: string): boolean; virtual;
      function    SaveToFile(const AFileName: string): boolean; virtual;
    public
   {a}function    InsertFrom(Editor: PCustomCodeEditor): Boolean; virtual;
   {a}function    InsertText(const S: string): Boolean; virtual;
    public
      procedure   FlagsChanged(OldFlags: longint); virtual;
   {a}procedure   BindingsChanged; virtual;
      procedure   ContentsChanged; virtual;
      procedure   LimitsChanged; virtual;
      procedure   ModifiedChanged; virtual;
      procedure   PositionChanged; virtual;
      procedure   TabSizeChanged; virtual;
      procedure   SyntaxStateChanged; virtual;
      procedure   StoreUndoChanged; virtual;
      procedure   SelectionChanged; virtual;
      procedure   HighlightChanged; virtual;
   {a}procedure   DoLimitsChanged; virtual;
    public
     { Syntax highlight support }
   {a}function    GetSpecSymbolCount(SpecClass: TSpecSymbolClass): integer; virtual;
   {a}function    GetSpecSymbol(SpecClass: TSpecSymbolClass; Index: integer): pstring; virtual;
   {a}function    IsReservedWord(const S: string): boolean; virtual;
   {a}function    IsAsmReservedWord(const S: string): boolean; virtual;
    public
     { CodeTemplate support }
   {a}function    TranslateCodeTemplate(var Shortcut: string; ALines: PUnsortedStringCollection): boolean; virtual;
      function    SelectCodeTemplate(var ShortCut: string): boolean; virtual;
     { CodeComplete support }
   {a}function    CompleteCodeWord(const WordS: string; var Text: string): boolean; virtual;
   {a}function    GetCodeCompleteWord: string; virtual;
   {a}procedure   SetCodeCompleteWord(const S: string); virtual;
   {a}function    GetCodeCompleteFrag: string; virtual;
   {a}procedure   SetCodeCompleteFrag(const S: string); virtual;
      function    GetCompleteState: TCompleteState; virtual;
      procedure   SetCompleteState(AState: TCompleteState); virtual;
      procedure   ClearCodeCompleteWord; virtual;
     { Fold support }
      function    GetMaxFoldLevel: sw_integer; virtual;
      function    GetFoldStringWidth: sw_integer; virtual;
      procedure   GetFoldStrings(EditorLine: sw_integer; var Prefix, Suffix: openstring); virtual;
   {a}function    GetFoldCount: sw_integer; virtual;
   {a}function    GetFold(Index: sw_integer): PFold; virtual;
   {a}procedure   RegisterFold(AFold: PFold); virtual;
   {a}procedure   UnRegisterFold(AFold: PFold); virtual;
      function    ViewToEditorLine(ViewLine: sw_integer): sw_integer;
      function    EditorToViewLine(EditorLine: sw_integer): sw_integer;
      procedure   ViewToEditorPoint(P: TPoint; var NP: TPoint);
      procedure   EditorToViewPoint(P: TPoint; var NP: TPoint);
     { Fold support }
      function    CreateFold(StartY,EndY: sw_integer; Collapsed: boolean): boolean; virtual;
      procedure   FoldChanged(Fold: PFold); virtual;
      procedure   RemoveAllFolds; virtual;
   public
      { Syntax highlight }
   {a}function    UpdateAttrs(FromLine: sw_integer; Attrs: byte): sw_integer; virtual;
   {a}function    UpdateAttrsRange(FromLine, ToLine: sw_integer; Attrs: byte): sw_integer; virtual;
    public
     { Undo info storage }
   {a}procedure   AddAction(AAction: byte; AStartPos, AEndPos: TPoint; AText: string;AFlags : longint); virtual;
   {a}procedure   AddGroupedAction(AAction : byte); virtual;
   {a}procedure   CloseGroupedAction(AAction : byte); virtual;
   {a}function    GetUndoActionCount: sw_integer; virtual;
   {a}function    GetRedoActionCount: sw_integer; virtual;
    protected
      LastLocalCmd: word;
      KeyState    : Integer;
      Bookmarks   : array[0..9] of TEditorBookmark;
      DrawCalled,
      DrawCursorCalled: boolean;
      CurEvent    : PEvent;
      procedure   DrawLines(FirstLine: sw_integer);
      function    Overwrite: boolean;
      function    IsModal: boolean;
      procedure   CheckSels;
      procedure   CodeCompleteCheck;
      procedure   CodeCompleteApply;
      procedure   CodeCompleteCancel;
      procedure   UpdateUndoRedo(cm : word; action : byte);
      procedure   HideHighlight;
      function    ShouldExtend: boolean;
      function    ValidBlock: boolean;
      function    GetLineFold(EditorLine: sw_integer): PFold;
      function    IsLineVisible(EditorLine: sw_integer): boolean; virtual;
      function    NextVisibleLine(StartLine: sw_integer; Down: boolean): sw_integer;
      procedure   PushInfo(Const st : string);virtual;
      procedure   PopInfo;virtual;
    public
      { Editor primitives }
      procedure   SelectAll(Enable: boolean); virtual;
    public
      { Editor commands }
      SearchRunCount: integer;
      InASCIIMode: boolean;
      procedure Indent; virtual;
      procedure CharLeft; virtual;
      procedure CharRight; virtual;
      procedure WordLeft; virtual;
      procedure WordRight; virtual;
      procedure LineStart; virtual;
      procedure LineEnd; virtual;
      procedure LineUp; virtual;
      procedure LineDown; virtual;
      procedure PageUp; virtual;
      procedure PageDown; virtual;
      procedure TextStart; virtual;
      procedure TextEnd; virtual;
      procedure WindowStart; virtual;
      procedure WindowEnd; virtual;
      procedure JumpSelStart; virtual;
      procedure JumpSelEnd; virtual;
      procedure JumpMark(MarkIdx: integer); virtual;
      procedure DefineMark(MarkIdx: integer); virtual;
      procedure JumpToLastCursorPos; virtual;
      procedure FindMatchingDelimiter(ScanForward: boolean); virtual;
      procedure CreateFoldFromBlock; virtual;
      procedure ToggleFold; virtual;
      procedure CollapseFold; virtual;
      procedure ExpandFold; virtual;
      procedure UpperCase; virtual;
      procedure LowerCase; virtual;
      procedure WordLowerCase; virtual;
      procedure WordUpperCase; virtual;
      procedure InsertOptions; virtual;
      procedure ToggleCase; virtual;
      function  InsertNewLine: Sw_integer; virtual;
      procedure BreakLine; virtual;
      procedure BackSpace; virtual;
      procedure DelChar; virtual;
      procedure DelWord; virtual;
      procedure DelToEndOfWord; virtual;
      procedure DelStart; virtual;
      procedure DelEnd; virtual;
      procedure DelLine; virtual;
      procedure InsMode; virtual;
      procedure StartSelect; virtual;
      procedure EndSelect; virtual;
      procedure DelSelect; virtual;
      procedure HideSelect; virtual;
      procedure CopyBlock; virtual;
      procedure MoveBlock; virtual;
      procedure IndentBlock; virtual;
      procedure UnindentBlock; virtual;
      procedure SelectWord; virtual;
      procedure SelectLine; virtual;
      procedure WriteBlock; virtual;
      procedure ReadBlock; virtual;
      procedure PrintBlock; virtual;
      procedure ExpandCodeTemplate; virtual;
      procedure AddChar(C: char); virtual;
{$ifdef WinClipSupported}
      function  ClipCopyWin: Boolean; virtual;
      function  ClipPasteWin: Boolean; virtual;
{$endif WinClipSupported}
      function  ClipCopy: Boolean; virtual;
      procedure ClipCut; virtual;
      procedure ClipPaste; virtual;
      function  GetCurrentWord : string;
      function  GetCurrentWordArea(var StartP,EndP: TPoint): boolean;
      procedure Undo; virtual;
      procedure Redo; virtual;
      procedure Find; virtual;
      procedure Replace; virtual;
      procedure DoSearchReplace; virtual;
      procedure GotoLine; virtual;
    end;

    TCodeEditorDialog = function(Dialog: Integer; Info: Pointer): Word;

    TEditorInputLine = object(TInputLine)
         Procedure   HandleEvent(var Event : TEvent);virtual;
    end;
    PEditorInputLine = ^TEditorInputLine;
 
    TSearchHelperDialog = object(TDialog)
             OkButton: PButton;
             Procedure   HandleEvent(var Event : TEvent);virtual;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
    TSearchHelperDialog = object(TDialog)
             OkButton: PButton;
             Procedure   HandleEvent(var Event : TEvent);virtual;
    end;

=======
>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
    PSearchHelperDialog = ^TSearchHelperDialog;

const
     { used for ShiftDel and ShiftIns to avoid
       GetShiftState to be considered for extending
       selection (PM) }
     DontConsiderShiftState: boolean  = false;

     CodeCompleteMinLen : byte = 4; { minimum length of text to try to complete }

     ToClipCmds         : TCommandSet = ([cmCut,cmCopy,cmCopyWin,
       { cmUnselect should because like cut, copy, copywin:
         if there is a selection, it is active, else it isn't }
       cmUnselect]);
     FromClipCmds       : TCommandSet = ([cmPaste]);
     NulClipCmds        : TCommandSet = ([cmClear]);
     UndoCmd            : TCommandSet = ([cmUndo]);
     RedoCmd            : TCommandSet = ([cmRedo]);

function ExtractTabs(S: string; TabSize: Sw_integer): string;

function StdEditorDialog(Dialog: Integer; Info: Pointer): word;

const
     DefaultSaveExt     : string[12] = '.pas';
     FileDir            : DirStr = '';

     EditorDialog       : TCodeEditorDialog = {$ifdef fpc}@{$endif}StdEditorDialog;
     Clipboard          : PCustomCodeEditor = nil;
     FindStr            : String[FindStrSize] = '';
     ReplaceStr         : String[FindStrSize] = '';
     FindReplaceEditor  : PCustomCodeEditor = nil;
     FindFlags          : word = ffPromptOnReplace;
{$ifndef NO_UNTYPEDSET}
  {$define USE_UNTYPEDSET}
{$endif ndef NO_UNTYPEDSET}
     WhiteSpaceChars    {$ifdef USE_UNTYPEDSET}: set of char {$endif} = [#0,#32,#255];
     TabChars           {$ifdef USE_UNTYPEDSET}: set of char {$endif} = [#9];
     HashChars          {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['#'];
     AlphaChars         {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['A'..'Z','a'..'z','_'];
     NumberChars        {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['0'..'9'];
     HexNumberChars     {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['0'..'9','A'..'F','a'..'f'];
     RealNumberChars    {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['E','e','.'{,'+','-'}];

procedure RegisterWEditor;

implementation

uses
  Strings,Video,MsgBox,App,StdDlg,Validate,
{$ifdef WinClipSupported}
  WinClip,
{$endif WinClipSupported}
{$ifdef TEST_REGEXP}
  {$ifdef USE_OLD_REGEXP}
    oldregexpr,
  {$else not USE_OLD_REGEXP}
    regexpr,
  {$endif not USE_OLD_REGEXP}
{$endif TEST_REGEXP}
  WConsts,WCEdit;

type
    RecordWord = sw_word;

     TFindDialogRec = packed record
       Find     : String[FindStrSize];
       Options  : RecordWord{longint};
       { checkboxes need 32  bits PM  }
       { reverted to word in dialogs.TCluster for TP compatibility (PM) }
       { anyhow its complete nonsense : you can only have 16 fields
         but use a longint to store it !! }
       Direction: RecordWord;{ and tcluster has word size }
       Scope    : RecordWord;
       Origin   : RecordWord;
     end;

     TReplaceDialogRec = packed record
       Find     : String[FindStrSize];
       Replace  : String[FindStrSize];
       Options  : RecordWord{longint};
       Direction: RecordWord;
       Scope    : RecordWord;
       Origin   : RecordWord;
     end;

     TGotoLineDialogRec = packed record
       LineNo  : string[5];
       Lines   : sw_integer;
     end;

const
     kbShift = kbLeftShift+kbRightShift;

const
  FirstKeyCount = 46;
  FirstKeys: array[0..FirstKeyCount * 2] of Word = (FirstKeyCount,
    Ord(^A), cmWordLeft, Ord(^B), cmJumpLine, Ord(^C), cmPageDown,
    Ord(^D), cmCharRight, Ord(^E), cmLineUp,
    Ord(^F), cmWordRight, Ord(^G), cmDelChar,
    Ord(^H), cmBackSpace, Ord(^J), cmExpandCodeTemplate,
    Ord(^K), $FF02, Ord(^L), cmSearchAgain,
    Ord(^M), cmNewLine, Ord(^N), cmBreakLine,
    Ord(^O), $FF03,
    Ord(^P), cmASCIIChar, Ord(^Q), $FF01,
    Ord(^R), cmPageUp, Ord(^S), cmCharLeft,
    Ord(^T), cmDelToEndOfWord, Ord(^U), cmUndo,
    Ord(^V), cmInsMode, Ord(^X), cmLineDown,
    Ord(^Y), cmDelLine, kbLeft, cmCharLeft,
    kbRight, cmCharRight, kbCtrlLeft, cmWordLeft,
    kbCtrlRight, cmWordRight, kbHome, cmLineStart,
    kbCtrlHome, cmWindowStart, kbCtrlEnd, cmWindowEnd,
    kbEnd, cmLineEnd, kbUp, cmLineUp,
    kbDown, cmLineDown, kbPgUp, cmPageUp,
    kbPgDn, cmPageDown, kbCtrlPgUp, cmTextStart,
    kbCtrlPgDn, cmTextEnd, kbIns, cmInsMode,
    kbDel, cmDelChar, kbShiftIns, cmPaste,
    kbShiftDel, cmCut, kbCtrlIns, cmCopy,
    kbCtrlDel, cmClear,
    kbCtrlGrayMul, cmToggleFold, kbCtrlGrayMinus, cmCollapseFold, kbCtrlGrayPlus, cmExpandFold);
  QuickKeyCount = 29;
  QuickKeys: array[0..QuickKeyCount * 2] of Word = (QuickKeyCount,
    Ord('A'), cmReplace, Ord('C'), cmTextEnd,
    Ord('D'), cmLineEnd, Ord('F'), cmFind,
    Ord('H'), cmDelStart, Ord('R'), cmTextStart,
    Ord('S'), cmLineStart, Ord('Y'), cmDelEnd,
    Ord('G'), cmJumpLine, Ord('A'), cmReplace,
    Ord('B'), cmSelStart, Ord('K'), cmSelEnd,
    Ord('P'), cmLastCursorPos,
    Ord('E'), cmWindowStart, Ord('T'), cmWindowStart,
    Ord('U'), cmWindowEnd, Ord('X'), cmWindowEnd,
    Ord('['), cmFindMatchingDelimiter, Ord(']'), cmFindMatchingDelimiterBack,
    Ord('0'), cmJumpMark0, Ord('1'), cmJumpMark1, Ord('2'), cmJumpMark2,
    Ord('3'), cmJumpMark3, Ord('4'), cmJumpMark4, Ord('5'), cmJumpMark5,
    Ord('6'), cmJumpMark6, Ord('7'), cmJumpMark7, Ord('8'), cmJumpMark8,
    Ord('9'), cmJumpMark9);
  BlockKeyCount = 30;
  BlockKeys: array[0..BlockKeyCount * 2] of Word = (BlockKeyCount,
    Ord('B'), cmStartSelect, Ord('C'), cmCopyBlock,
    Ord('H'), cmHideSelect, Ord('K'), cmEndSelect,
    Ord('Y'), cmDelSelect, Ord('V'), cmMoveBlock,
    Ord('I'), cmIndentBlock, Ord('U'), cmUnindentBlock,
    Ord('T'), cmSelectWord, Ord('L'), cmSelectLine,
    Ord('W'), cmWriteBlock, Ord('R'), cmReadBlock,
    Ord('P'), cmPrintBlock,
    Ord('N'), cmUpperCase, Ord('O'), cmLowerCase,
    Ord('D'), cmActivateMenu,
    Ord('E'), cmWordLowerCase, Ord('F'), cmWordUpperCase,
    Ord('S'), cmSave, Ord('A'), cmCreateFold,
    Ord('0'), cmSetMark0, Ord('1'), cmSetMark1, Ord('2'), cmSetMark2,
    Ord('3'), cmSetMark3, Ord('4'), cmSetMark4, Ord('5'), cmSetMark5,
    Ord('6'), cmSetMark6, Ord('7'), cmSetMark7, Ord('8'), cmSetMark8,
    Ord('9'), cmSetMark9);
  MiscKeyCount = 6;
  MiscKeys: array[0..MiscKeyCount * 2] of Word = (MiscKeyCount,
    Ord('A'), cmOpenAtCursor, Ord('B'), cmBrowseAtCursor,
    Ord('G'), cmJumpLine, Ord('O'), cmInsertOptions,
    Ord('U'), cmToggleCase, Ord('L'), cmSelectLine);
  KeyMap: array[0..3] of Pointer = (@FirstKeys, @QuickKeys, @BlockKeys, @MiscKeys);

function ScanKeyMap(KeyMap: Pointer; KeyCode: Word): Word;
type
  pword = ^word;
var
  p : pword;
  count : sw_word;
begin
  p:=keymap;
  count:=p^;
  inc(p);
  while (count>0) do
   begin
     if (lo(p^)=lo(keycode)) and
        ((hi(p^)=0) or (hi(p^)=hi(keycode))) then
      begin
        inc(p);
        scankeymap:=p^;
        Exit;
      end;
     inc(p,2);
     dec(count);
   end;
  scankeymap:=0;
end;

function IsWordSeparator(C: char): boolean;
begin
  IsWordSeparator:=C in
      [' ',#0,#255,':','=','''','"',
      '.',',','/',';','$','#',
      '(',')','<','>','^','*',
      '+','-','?','&','[',']',
      '{','}','@','~','%','\',
      '!'];
end;

{function IsSpace(C: char): boolean;
begin
  IsSpace:=C in[' ',#0,#255];
end;}

function LTrim(S: string): string;
begin
  while (length(S)>0) and (S[1] in [#0,TAB,#32]) do
    Delete(S,1,1);
  LTrim:=S;
end;

{ TAB are not same as spaces if UseTabs is set PM }
function RTrim(S: string;cut_tabs : boolean): string;
begin
  while (length(S)>0) and
    ((S[length(S)] in [#0,#32]) or
    ((S[Length(S)]=TAB) and cut_tabs)) do
    Delete(S,length(S),1);
  RTrim:=S;
end;

function Trim(S: string): string;
begin
  Trim:=RTrim(LTrim(S),true);
end;

function EatIO: integer;
begin
  EatIO:=IOResult;
end;

function ExistsFile(const FileName: string): boolean;
var f: file;
    Exists: boolean;
begin
  if FileName='' then Exists:=false else
 begin
  {$I-}
  Assign(f,FileName);
  Reset(f,1);
  Exists:=EatIO=0;
  Close(f);
  EatIO;
  {$I+}
 end;
  ExistsFile:=Exists;
end;

function StrToInt(const S: string): longint;
var L: longint;
    C: integer;
begin
  Val(S,L,C); if C<>0 then L:=-1;
  StrToInt:=L;
end;

function RExpand(const S: string; MinLen: byte): string;
begin
  if length(S)<MinLen then
   RExpand:=S+CharStr(' ',MinLen-length(S))
  else
   RExpand:=S;
end;

{
function upper(const s : string) : string;
var
  i  : Sw_word;
begin
  for i:=1 to length(s) do
   if s[i] in ['a'..'z'] then
    upper[i]:=char(byte(s[i])-32)
   else
    upper[i]:=s[i];
  upper[0]:=s[0];
end;
}
type TPosOfs = int64;

function PosToOfs(const X,Y: sw_integer): TPosOfs;
begin
  PosToOfs:=TPosOfs(y) shl (sizeof(sw_integer)*8) or x;
end;

function PosToOfsP(const P: TPoint): TPosOfs;
begin
  PosToOfsP:=PosToOfs(P.X,P.Y);
end;

function PointOfs(P: TPoint): TPosOfs;
begin
  PointOfs:={longint(P.Y)*MaxLineLength+P.X}PosToOfsP(P);
end;


function ExtractTabs(S: string; TabSize: Sw_integer): string;
var
  P,PAdd: Sw_integer;
begin
  p:=0;
  while p<length(s) do
   begin
     inc(p);
     if s[p]=TAB then
      begin
        PAdd:=TabSize-((p-1) mod TabSize);
        s:=copy(S,1,P-1)+CharStr(' ',PAdd)+copy(S,P+1,High(s));
        inc(P,PAdd-1);
      end;
   end;
  ExtractTabs:=S;
end;

{function CompressUsingTabs(S: string; TabSize: byte): string;
var TabS: string;
    P: byte;
begin
  TabS:=CharStr(' ',TabSize);
  repeat
    P:=Pos(TabS,S);
    if P>0 then
      S:=copy(S,1,P-1)+TAB+copy(S,P+TabSize,High(S));
  until P=0;
  CompressUsingTabs:=S;
end;}


{*****************************************************************************
                           Forward/Backward Scanning
*****************************************************************************}

Const
  MaxBufLength   = $7fffff00;
  NotFoundValue  = -1;

Type
  Btable = Array[0..255] of Byte;
Procedure BMFMakeTable(const s:string; Var t : Btable);
Var
  x : sw_integer;
begin
  FillChar(t,sizeof(t),length(s));
  For x := length(s) downto 1 do
   if (t[ord(s[x])] = length(s)) then
    t[ord(s[x])] := length(s) - x;
end;


function BMFScan(var Block; Size: Sw_Word;const Str: String;const bt:BTable): Sw_Integer;
Var
  buffer : Array[0..MaxBufLength-1] of Byte Absolute block;
  s2     : String;
  len,
  numb   : Sw_Word;
  found  : Boolean;
begin
  len:=length(str);
  if len>size then
   begin
     BMFScan := NotFoundValue;
     exit;
   end;
  s2[0]:=chr(len);       { sets the length to that of the search String }
  found:=False;
  numb:=pred(len);
  While (not found) and (numb<size) do
   begin
     { partial match }
     if buffer[numb] = ord(str[len]) then
      begin
        { less partial! }
        if buffer[numb-pred(len)] = ord(str[1]) then
         begin
           move(buffer[numb-pred(len)],s2[1],len);
           if (str=s2) then
            begin
              found:=true;
              break;
            end;
         end;
        inc(numb);
     end
    else
     inc(numb,Bt[buffer[numb]]);
  end;
  if not found then
    BMFScan := NotFoundValue
  else
    BMFScan := numb - pred(len);
end;


function BMFIScan(var Block; Size: Sw_Word;const Str: String;const bt:BTable): Sw_Integer;
Var
  buffer : Array[0..MaxBufLength-1] of Char Absolute block;
  len,
  numb,
  x      : Sw_Word;
  found  : Boolean;
  p      : pchar;
  c      : char;
begin
  len:=length(str);
  if (len=0) or (len>size) then
   begin
     BMFIScan := NotFoundValue;
     exit;
   end;
  found:=False;
  numb:=pred(len);
  While (not found) and (numb<size) do
   begin
     { partial match }
     c:=buffer[numb];
     if c in ['a'..'z'] then
      c:=chr(ord(c)-32);
     if (c=str[len]) then
      begin
        { less partial! }
        p:=@buffer[numb-pred(len)];
        x:=1;
        while (x<=len) do
         begin
           if not(((p^ in ['a'..'z']) and (chr(ord(p^)-32)=str[x])) or
             (p^=str[x])) then
            break;
           inc(p);
           inc(x);
         end;
        if (x>len) then
         begin
           found:=true;
           break;
         end;
        inc(numb);
      end
     else
      inc(numb,Bt[ord(c)]);
   end;
  if not found then
    BMFIScan := NotFoundValue
  else
    BMFIScan := numb - pred(len);
end;


Procedure BMBMakeTable(const s:string; Var t : Btable);
Var
  x : sw_integer;
begin
  FillChar(t,sizeof(t),length(s));
  For x := 1 to length(s)do
   if (t[ord(s[x])] = length(s)) then
    t[ord(s[x])] := x-1;
end;


function BMBScan(var Block; Size: Sw_Word;const Str: String;const bt:BTable): Sw_Integer;
Var
  buffer : Array[0..MaxBufLength-1] of Byte Absolute block;
  s2     : String;
  len    : Sw_Word;
  numb   : Sw_Integer;
  found  : Boolean;
begin
  len:=length(str);
  if len>size then
   begin
     BMBScan := NotFoundValue;
     exit;
   end;
  s2[0]:=chr(len);       { sets the length to that of the search String }
  found:=False;
  numb:=size-len;
  While (not found) and (numb>=0) do
   begin
     { partial match }
     if buffer[numb] = ord(str[1]) then
      begin
        { less partial! }
        if buffer[numb+pred(len)] = ord(str[len]) then
         begin
           move(buffer[numb],s2[1],len);
           if (str=s2) then
            begin
              found:=true;
              break;
            end;
         end;
        dec(numb);
     end
    else
     dec(numb,Bt[buffer[numb]]);
  end;
  if not found then
    BMBScan := NotFoundValue
  else
    BMBScan := numb;
end;


function BMBIScan(var Block; Size: Sw_Word;const Str: String;const bt:BTable): Sw_Integer;
Var
  buffer : Array[0..MaxBufLength-1] of Char Absolute block;
  len,
  x      : Sw_Word;
  numb   : Sw_Integer;
  found  : Boolean;
  p      : pchar;
  c      : char;
begin
  len:=length(str);
  if (len=0) or (len>size) then
   begin
     BMBIScan := NotFoundValue;
     exit;
   end;
  found:=False;
  numb:=size-len;
  While (not found) and (numb>=0) do
   begin
     { partial match }
     c:=buffer[numb];
     if c in ['a'..'z'] then
      c:=chr(ord(c)-32);
     if (c=str[1]) then
      begin
        { less partial! }
        p:=@buffer[numb];
        x:=1;
        while (x<=len) do
         begin
           if not(((p^ in ['a'..'z']) and (chr(ord(p^)-32)=str[x])) or
             (p^=str[x])) then
            break;
           inc(p);
           inc(x);
         end;
        if (x>len) then
         begin
           found:=true;
           break;
         end;
        dec(numb);
      end
     else
      dec(numb,Bt[ord(c)]);
   end;
  if not found then
    BMBIScan := NotFoundValue
  else
    BMBIScan := numb;
end;


{*****************************************************************************
                            PLine,TLineCollection
*****************************************************************************}

constructor TCustomLine.Init(const AText: string; AFlags: longint);
begin
  inherited Init;
  SetText(AText);
end;

function TCustomLine.GetText: string;
begin
  Abstract;GetText:='';
end;

procedure TCustomLine.SetText(const AText: string);
begin
  Abstract;
end;

function TCustomLine.GetEditorInfo(Editor: PCustomCodeEditor): PEditorLineInfo;
begin
  Abstract;
  GetEditorInfo:=nil;
end;

function TCustomLine.GetFlags: longint;
begin
  Abstract;
  GetFlags:=0;
end;

procedure TCustomLine.SetFlags(AFlags: longint);
begin
  Abstract;
end;

function TCustomLine.IsFlagSet(AFlag: longint): boolean;{$ifdef USEINLINE}inline;{$endif}
begin
  IsFlagSet:=(GetFlags and AFlag)=AFlag;
end;

procedure TCustomLine.SetFlagState(AFlag: longint; ASet: boolean);
var N,O: longint;
begin
  O:=GetFlags; N:=O;
  if ASet then
    N:=N or AFlag
  else
    N:=N and (not AFlag);
  if N<>O then
    SetFlags(N);
end;

procedure TCustomLine.AddEditorInfo(Index: sw_integer; AEditor: PCustomCodeEditor);
begin
  { Abstract }
end;

procedure TCustomLine.RemoveEditorInfo(AEditor: PCustomCodeEditor);
begin
  { Abstract }
end;

destructor TCustomLine.Done;
begin
  inherited Done;
end;

function TLineCollection.At(Index: sw_Integer): PCustomLine;
begin
  At:=inherited At(Index);
end;

constructor TFold.Init(AEditor: PCustomCodeEditor; AParentFold: PFold; ACollapsed: boolean);
begin
  inherited Init;
  New(Childs, Init(10,10));
  Editor:=AEditor;
  ParentFold:=AParentFold;
  if Assigned(ParentFold) then
    ParentFold^.AddChildReference(@Self);
  Collapsed_:=ACollapsed;
  if Assigned(AEditor) then
    Editor^.RegisterFold(@Self);
end;

procedure TFold.AddReference(P: PObject);
begin
  Inc(ReferenceCount);
end;

procedure TFold.RemoveReference(P: PObject);
begin
  Dec(ReferenceCount);
  if CanDispose then
    Free;
end;

procedure TFold.AddLineReference(Line: PEditorLineInfo);
begin
  Inc(LineCount_);
  AddReference(Line);
end;

procedure TFold.RemoveLineReference(Line: PEditorLineInfo);
begin
  Dec(LineCount_);
  RemoveReference(Line);
end;

procedure TFold.AddChildReference(Fold: PFold);
begin
  Childs^.Insert(Fold);
  AddReference(Fold);
end;

procedure TFold.RemoveChildReference(Fold: PFold);
begin
  Childs^.Delete(Fold);
  RemoveReference(Fold);
end;

function TFold.CanDispose: boolean;
begin
  CanDispose:=ReferenceCount<=0;
end;

function TFold.IsCollapsed: boolean;
var C: boolean;
begin
  C:=Collapsed_;
  if Assigned(ParentFold) then C:=C or ParentFold^.IsCollapsed;
  IsCollapsed:=C;
end;

function TFold.IsParent(AFold: PFold): boolean;
var P: boolean;
begin
  P:=(ParentFold=AFold);
  if Assigned(ParentFold) then P:=P or ParentFold^.IsParent(AFold);
  IsParent:=P;
end;

function TFold.GetLineCount: sw_integer;
var Count: sw_integer;
procedure AddIt(P: PFold);
begin
  Inc(Count,P^.GetLineCount);
end;
begin
  Count:=LineCount_;
  if assigned(Childs) then Childs^.ForEach(@AddIt);
  GetLineCount:=Count;
end;

procedure TFold.Collapse(ACollapse: boolean);
begin
  if ACollapse<>Collapsed_ then
  begin
    Collapsed_:=ACollapse;
    if (not Collapsed_) and Assigned(ParentFold) then
      ParentFold^.Collapse(false);
    Changed;
  end;
end;

procedure TFold.Changed;
begin
  if Assigned(Editor) then
    Editor^.FoldChanged(@Self);
end;

function TFold.GetLevel: sw_integer;
var Level: sw_integer;
begin
  Level:=0;
  if Assigned(ParentFold) then
    Inc(Level,1+ParentFold^.GetLevel);
  GetLevel:=Level;
end;

destructor TFold.Done;
begin
  if Assigned(ParentFold) then
    ParentFold^.RemoveChildReference(@Self);
  if Assigned(Editor) then
    Editor^.UnRegisterFold(@Self);
  Childs^.DeleteAll; Dispose(Childs, Done);
  inherited Done;
end;

function TFoldCollection.At(Index: sw_Integer): PFold;
begin
  At:=inherited At(Index);
end;

constructor TEditorLineInfo.Init(AEditor: PCustomCodeEditor);
begin
  inherited Init;
  Editor:=AEditor;
end;

function TEditorLineInfo.GetFormat: string;
begin
  GetFormat:=GetStr(Format);
end;

procedure TEditorLineInfo.SetFormat(const AFormat: string);
begin
  SetStr(Format,AFormat);
end;

procedure TEditorLineInfo.SetFold(AFold: PFold);
begin
  if Assigned(Fold) then
    Fold^.RemoveLineReference(@Self);
  Fold:=AFold;
  if Assigned(Fold) then
    Fold^.AddLineReference(@Self);
end;

destructor TEditorLineInfo.Done;
begin
  if Format<>nil then
    DisposeStr(Format);
  Format:=nil;
  SetFold(nil);
  inherited Done;
end;

function TEditorLineInfoCollection.At(Index: sw_Integer): PEditorLineInfo;
begin
  At:=inherited At(Index);
end;

function TEditorBindingCollection.At(Index: sw_Integer): PEditorBinding;
begin
  At:=inherited At(Index);
end;

constructor TEditorBinding.Init(AEditor: PCustomCodeEditor);
begin
  inherited Init;
  Editor:=AEditor;
end;

destructor TEditorBinding.Done;
begin
  inherited Done;
end;

constructor TCustomCodeEditorCore.Init;
begin
  inherited Init;
  New(Bindings, Init(10,10));
end;

procedure TCustomCodeEditorCore.BindEditor(AEditor: PCustomCodeEditor);
var B: PEditorBinding;
    Count,I,Idx: sw_integer;
    L: PCustomLine;
begin
  assert(Aeditor<>nil);
  New(B, Init(AEditor));
  Bindings^.Insert(B);
  Idx:=Bindings^.IndexOf(B);
  Count:=GetLineCount;
  for I:=0 to Count-1 do
  begin
    L:=GetLine(I);
    if Assigned(L) then
      L^.AddEditorInfo(Idx,AEditor);
  end;

  BindingsChanged;
end;

procedure TCustomCodeEditorCore.UnBindEditor(AEditor: PCustomCodeEditor);
var B: PEditorBinding;
    Count,I: sw_integer;
    L: PCustomLine;
begin
  assert(Aeditor<>nil);
  B:=SearchBinding(AEditor);
  if Assigned(B) then
  begin
    Count:=GetLineCount;
    for I:=0 to Count-1 do
    begin
      L:=GetLine(I);
      if Assigned(L) then
        L^.RemoveEditorInfo(AEditor);
    end;
    Bindings^.Free(B);

    BindingsChanged;
  end;
end;

function TCustomCodeEditorCore.IsEditorBound(AEditor: PCustomCodeEditor): boolean;
begin
  IsEditorBound:=SearchBinding(AEditor)<>nil;
end;

function TCustomCodeEditorCore.GetBindingCount: sw_integer;
begin
  GetBindingCount:=Bindings^.Count;
end;

function TCustomCodeEditorCore.GetBindingIndex(AEditor: PCustomCodeEditor): sw_integer;
var B: PEditorBinding;
begin
  B:=SearchBinding(AEditor);
  GetBindingIndex:=Bindings^.IndexOf(B);
end;

function TCustomCodeEditorCore.SearchBinding(AEditor: PCustomCodeEditor): PEditorBinding;
function SearchEditor(P: PEditorBinding): boolean;
begin
  SearchEditor:=P^.Editor=AEditor;
end;
begin
  SearchBinding:=Bindings^.FirstThat(@SearchEditor);
end;

function TCustomCodeEditorCore.CanDispose: boolean;
begin
  CanDispose:=Assigned(Bindings) and (Bindings^.Count=0);
end;

function TCustomCodeEditorCore.GetModified: boolean;
begin
  Abstract;
  GetModified:=true;
end;
function TCustomCodeEditorCore.GetChangedLine: sw_integer;
begin
  GetChangedLine:=ChangedLine;
end;

procedure TCustomCodeEditorCore.SetModified(AModified: boolean);
begin
  Abstract;
end;

function TCustomCodeEditorCore.GetStoreUndo: boolean;
begin
  Abstract;
  GetStoreUndo:=false;
end;

procedure TCustomCodeEditorCore.SetStoreUndo(AStore: boolean);
begin
  Abstract;
end;

function TCustomCodeEditorCore.GetSyntaxCompleted: boolean;
begin
  Abstract;
  GetSyntaxCompleted:=true;
end;

procedure TCustomCodeEditorCore.SetSyntaxCompleted(SC : boolean);
begin
  Abstract;
end;


function TCustomCodeEditorCore.IsClipboard: Boolean;
function IsClip(P: PEditorBinding): boolean;
begin
  IsClip:=(P^.Editor=Clipboard);
end;
begin
  IsClipBoard:=Bindings^.FirstThat(@IsClip)<>nil;
end;

function TCustomCodeEditorCore.GetTabSize: integer;
begin
  Abstract;
  GetTabSize:=0;
end;

procedure TCustomCodeEditorCore.SetTabSize(ATabSize: integer);
begin
  Abstract;
end;

function TCustomCodeEditorCore.GetIndentSize: integer;
begin
  Abstract;
  GetIndentSize:=0;
end;

procedure TCustomCodeEditorCore.SetIndentSize(AIndentSize: integer);
begin
  Abstract;
end;

procedure TCustomCodeEditorCore.LimitsChanged;
begin
  if Locked then
    LimitsChangedCalled:=true
  else
    DoLimitsChanged;
end;

procedure TCustomCodeEditorCore.ContentsChanged;
begin
  if Locked then
    ContentsChangedCalled:=true
  else
    DoContentsChanged;
end;

procedure TCustomCodeEditorCore.ModifiedChanged;
begin
  if Locked then
    ModifiedChangedCalled:=true
  else
    DoModifiedChanged;
end;

procedure TCustomCodeEditorCore.TabSizeChanged;
begin
  if Locked then
    TabSizeChangedCalled:=true
  else
    DoTabSizeChanged;
end;

procedure TCustomCodeEditorCore.StoreUndoChanged;
begin
  if Locked then
    StoreUndoChangedCalled:=true
  else
    DoStoreUndoChanged;
end;


procedure TCustomCodeEditorCore.BindingsChanged;
procedure CallIt(P: PEditorBinding);
begin
  P^.Editor^.BindingsChanged;
end;
begin
  Bindings^.ForEach(@CallIt);
end;

procedure TCustomCodeEditorCore.DoLimitsChanged;
procedure CallIt(P: PEditorBinding);
begin
  P^.Editor^.DoLimitsChanged;
end;
begin
  Bindings^.ForEach(@CallIt);
end;

procedure TCustomCodeEditorCore.DoContentsChanged;
procedure CallIt(P: PEditorBinding);
begin
  P^.Editor^.ContentsChanged;
end;
begin
  Bindings^.ForEach(@CallIt);
end;

procedure TCustomCodeEditorCore.DoModifiedChanged;
procedure CallIt(P: PEditorBinding);
begin
  P^.Editor^.ModifiedChanged;
end;
begin
  Bindings^.ForEach(@CallIt);
end;

procedure TCustomCodeEditorCore.DoTabSizeChanged;
procedure CallIt(P: PEditorBinding);
begin
  P^.Editor^.TabSizeChanged;
end;
begin
  Bindings^.ForEach(@CallIt);
end;

procedure TCustomCodeEditorCore.UpdateUndoRedo(cm : word; action : byte);
procedure CallIt(P: PEditorBinding);
begin
  if (P^.Editor^.State and sfActive)<>0 then
    begin
      P^.Editor^.UpdateUndoRedo(cm,action);
    if cm=cmUndo then
      begin
        P^.Editor^.SetCmdState(UndoCmd,true);
        P^.Editor^.SetCmdState(RedoCmd,false);
        Message(Application,evBroadcast,cmCommandSetChanged,nil);
      end;
    end;
end;
begin
  Bindings^.ForEach(@CallIt);
end;


procedure TCustomCodeEditorCore.DoStoreUndoChanged;
procedure CallIt(P: PEditorBinding);
begin
  P^.Editor^.StoreUndoChanged;
end;
begin
  Bindings^.ForEach(@CallIt);
end;
procedure   TCustomCodeEditorCore.DoSyntaxStateChanged;
procedure CallIt(P: PEditorBinding);
begin
  P^.Editor^.SyntaxStateChanged;
end;
begin
  Bindings^.ForEach(@CallIt);
end;

function TCustomCodeEditorCore.GetLastVisibleLine : sw_integer;
var
  y : sw_integer;
procedure CallIt(P: PEditorBinding);
begin
  if y < P^.Editor^.Delta.Y+P^.Editor^.Size.Y then
    y:=P^.Editor^.Delta.Y+P^.Editor^.Size.Y;
end;
begin
  y:=0;
  Bindings^.ForEach(@CallIt);
  GetLastVisibleLine:=y;
end;

function TCustomCodeEditorCore.SaveToStream(Editor: PCustomCodeEditor; Stream: PStream): boolean;
var A,B: TPoint;
begin
  A.Y:=0; A.X:=0;
  B.Y:=GetLineCount-1;
  if GetLineCount>0 then
    B.X:=length(GetDisplayText(B.Y))
  else
    B.X:=0;
  SaveToStream:=SaveAreaToStream(Editor,Stream,A,B);
end;

procedure TCustomCodeEditorCore.ISetLineFlagState(Binding: PEditorBinding; LineNo: sw_integer; Flag: longint; ASet: boolean);
begin
  Abstract;
end;

procedure TCustomCodeEditorCore.IGetDisplayTextFormat(Binding: PEditorBinding; LineNo: sw_integer;var DT,DF:string);
begin
  Abstract;
end;

function TCustomCodeEditorCore.IGetLineFormat(Binding: PEditorBinding; LineNo: sw_integer): string;
begin
  Abstract;
  IGetLineFormat:='';
end;

procedure TCustomCodeEditorCore.ISetLineFormat(Binding: PEditorBinding; LineNo: sw_integer;const S: string);
begin
  Abstract;
end;

function TCustomCodeEditorCore.CharIdxToLinePos(Line,CharIdx: sw_integer): sw_integer;
var S: string;
    TabSize,CP,RX,NextInc: sw_integer;
begin
  S:=GetLineText(Line);
  (* this would fasten the code
    but UseTabCharacters is set for Editor not for EditorCore
    objects,which is dangerous anyway and should be changed ... PM
  if not IsFlagSet(efUseTabCharacters) then
    begin
     if CharIdx<=Length(S) then
       CharIdxToLinePos:=CharIdx-1
     else
       CharIdxToLinePos:=Length(S)-1;
     exit;
    end; *)

  TabSize:=GetTabSize;
  CP:=1; RX:=0;
  NextInc:=0;
  while {(CP<=length(S)) and }(CP<=CharIdx) do
   begin
     if NextInc>0 then
       Inc(RX,NextInc);
     if (CP<=length(S)) and (S[CP]=TAB) then
       NextInc:=TabSize-(RX mod TabSize) -1
     else
       NextInc:=0;
     Inc(RX);
     Inc(CP);
   end;
  CharIdxToLinePos:=RX-1;
end;

function TCustomCodeEditorCore.LinePosToCharIdx(Line,X: sw_integer): sw_integer;
var S: string;
    TabSize,CP,RX: sw_integer;
begin
  TabSize:=GetTabSize;
  S:=GetLineText(Line);
  (*
  if not IsFlagSet(efUseTabCharacters) then
    begin
      if S='' then
        CP:=0
      else if (Line<Length(S)) then
        LinePosToCharIdx:=Line+1
      else
        LinePosToCharIdx:=Length(S);
      exit;
    end; *)
  if S='' then
    CP:=0
  else
    begin
     CP:=0; RX:=0;
     while (RX<=X) and (CP<=length(S)) do
      begin
        Inc(CP);
        if (CP<=length(S)) and
           (S[CP]=TAB) then
          Inc(RX,TabSize-(RX mod TabSize))
        else
          Inc(RX);
      end;
    end;
  LinePosToCharIdx:=CP;
end;

function TCustomCodeEditorCore.GetLineCount: sw_integer;
begin
  Abstract;
  GetLineCount:=0;
end;

function TCustomCodeEditorCore.GetLine(LineNo: sw_integer): PCustomLine;
begin
  Abstract;
  GetLine:=nil;
end;

function TCustomCodeEditorCore.GetLineText(LineNo: sw_integer): string;
begin
  Abstract;
  GetLineText:='';
end;

procedure TCustomCodeEditorCore.SetDisplayText(I: sw_integer;const S: string);
begin
  Abstract;
end;

function TCustomCodeEditorCore.GetDisplayText(I: sw_integer): string;
begin
  Abstract;
  GetDisplayText:='';
end;

procedure TCustomCodeEditorCore.SetLineText(I: sw_integer;const S: string);
begin
  Abstract;
end;

procedure TCustomCodeEditorCore.GetDisplayTextFormat(Editor: PCustomCodeEditor; I: sw_integer;var DT,DF:string);
begin
  IGetDisplayTextFormat(SearchBinding(Editor),I,DT,DF);
end;

function TCustomCodeEditorCore.GetLineFormat(Editor: PCustomCodeEditor; I: sw_integer): string;
begin
  GetLineFormat:=IGetLineFormat(SearchBinding(Editor),I);
end;

procedure TCustomCodeEditorCore.SetLineFormat(Editor: PCustomCodeEditor; I: sw_integer; const S: string);
begin
  ISetLineFormat(SearchBinding(Editor),I,S);
end;

procedure TCustomCodeEditorCore.DeleteAllLines;
begin
  Abstract;
end;

procedure TCustomCodeEditorCore.DeleteLine(I: sw_integer);
begin
  Abstract;
end;

function TCustomCodeEditorCore.InsertLine(LineNo: sw_integer; const S: string): PCustomLine;
begin
  Abstract;
  InsertLine:=nil; { eliminate compiler warning }
end;

procedure TCustomCodeEditorCore.AddLine(const S: string);
begin
  Abstract;
end;

procedure TCustomCodeEditorCore.GetContent(ALines: PUnsortedStringCollection);
begin
  Abstract;
end;

procedure TCustomCodeEditorCore.SetContent(ALines: PUnsortedStringCollection);
begin
  Abstract;
end;

function TCustomCodeEditorCore.Locked: boolean;
begin
  Locked:=LockFlag>0;
end;

procedure TCustomCodeEditorCore.Lock(AEditor: PCustomCodeEditor);
begin
  Inc(LockFlag);
end;

procedure TCustomCodeEditorCore.UnLock(AEditor: PCustomCodeEditor);
begin
{$ifdef DEBUG}
  if LockFlag=0 then
    Bug('negative lockflag',nil)
  else
{$endif DEBUG}
    Dec(LockFlag);
  if (LockFlag>0) then
    Exit;

  if LimitsChangedCalled then
    begin
      DoLimitsChanged;
      LimitsChangedCalled:=false;
    end;

  if ModifiedChangedCalled then
    begin
      DoModifiedChanged;
      ModifiedChangedCalled:=false;
    end;

  if TabSizeChangedCalled then
    begin
      DoTabSizeChanged;
      TabSizeChangedCalled:=false;
    end;

  if StoreUndoChangedCalled then
    begin
      DoStoreUndoChanged;
      StoreUndoChangedCalled:=false;
    end;

  if ContentsChangedCalled then
    begin
      DoContentsChanged;
      ContentsChangedCalled:=false;
    end;

end;

function TCustomCodeEditorCore.UpdateAttrs(FromLine: sw_integer; Attrs: byte): sw_integer;
var MinLine: sw_integer;
procedure CallIt(P: PEditorBinding);
var I: sw_integer;
begin
  I:=DoUpdateAttrs(P^.Editor,FromLine,Attrs);
  if (I<MinLine) or (MinLine=-1) then MinLine:=I;
end;
begin
  MinLine:=-1;
  Bindings^.ForEach(@CallIt);
  UpdateAttrs:=MinLine;
end;

function TCustomCodeEditorCore.UpdateAttrsRange(FromLine, ToLine: sw_integer; Attrs: byte): sw_integer;
var MinLine: sw_integer;
procedure CallIt(P: PEditorBinding);
var I: sw_integer;
begin
  I:=DoUpdateAttrsRange(P^.Editor,FromLine,ToLine,Attrs);
  if (I<MinLine) or (MinLine=-1) then MinLine:=I;
end;
begin
  MinLine:=-1;
  Bindings^.ForEach(@CallIt);
  UpdateAttrsRange:=MinLine;
end;

function TCustomCodeEditorCore.DoUpdateAttrs(Editor: PCustomCodeEditor; FromLine: sw_integer; Attrs: byte): sw_integer;
type
    TCharClass = (ccWhiteSpace,ccTab,ccAlpha,
      ccNumber,ccHexNumber,ccRealNumber,
      ccHash,ccSymbol);
var
  SymbolIndex: Sw_integer;
  CurrentCommentType : Byte;
  FirstCC,LastCC: TCharClass;
  InAsm,InComment,InSingleLineComment,InDirective,InString: boolean;
  X,ClassStart: Sw_integer;
  SymbolConcat: string;
  LineText,Format: string;

  function MatchSymbol(const What, S: string): boolean;
  var Match: boolean;
  begin
    Match:=false;
    if length(What)>=length(S) then
      if copy(What,1+length(What)-length(S),length(S))=S then
    Match:=true;
    MatchSymbol:=Match;
  end;

  var MatchedSymbol: boolean;
      MatchingSymbol: string;
  type TPartialType = (pmNone,pmLeft,pmRight,pmAny);

  function MatchesAnySpecSymbol(SClass: TSpecSymbolClass; PartialMatch: TPartialType): boolean;
  var S: pstring;
      I: Sw_integer;
      Match,Found: boolean;
  begin
    Found:=false;
    if SymbolConcat<>'' then
    for I:=1 to Editor^.GetSpecSymbolCount(SClass) do
    begin
      SymbolIndex:=I;
      S:=Editor^.GetSpecSymbol(SClass,I-1);
      if (length(SymbolConcat)<length(S^)) or
         ((PartialMatch=pmNone) and (length(S^)<>length(SymbolConcat)))
          then
        Match:=false
      else
        begin
          case PartialMatch of
            pmNone : Match:=SymbolConcat=S^;
            pmRight:
              Match:=copy(SymbolConcat,length(SymbolConcat)-length(S^)+1,length(S^))=S^;
          else Match:=MatchSymbol(SymbolConcat,S^);
          end;
        end;
      if Match then
      begin
        MatchingSymbol:=S^; Found:=true; Break;
      end;
    end;
    MatchedSymbol:=MatchedSymbol or Found;
    MatchesAnySpecSymbol:=Found;
  end;

  function MatchesAsmSpecSymbol(Const OrigWhat: string; SClass: TSpecSymbolClass): boolean;
  var What : String;
      S: pstring;
      I: Sw_integer;
      Match,Found: boolean;
  begin
    Found:=false;
    What:=UpcaseStr(OrigWhat);
    if What<>'' then
    for I:=1 to Editor^.GetSpecSymbolCount(SClass) do
    begin
      SymbolIndex:=I;
      S:=Editor^.GetSpecSymbol(SClass,I-1);
      if (length(S^)<>length(What)) then
        Match:=false
      else
        begin
          {if CaseInsensitive then
            S:=UpcaseStr(S); asm symbols need to be uppercased PM }
          {case PartialMatch of
            pmNone : }
          Match:=What=S^;
          {  pmRight:
              Match:=copy(What,length(What)-length(S)+1,length(S))=S;
          else Match:=MatchSymbol(What,S);
          end;  }
        end;
      if Match then
      begin
        MatchingSymbol:=S^;
        Found:=true;
        Break;
      end;
    end;
    // MatchedSymbol:=MatchedSymbol or Found;
    MatchesAsmSpecSymbol:=Found;
  end;

  function IsCommentPrefix: boolean;
  begin
    IsCommentPrefix:=MatchesAnySpecSymbol(ssCommentPrefix,pmLeft);
  end;
                              {** **}
  function IsSingleLineCommentPrefix: boolean;
  begin
    IsSingleLineCommentPrefix:=MatchesAnySpecSymbol(ssCommentSingleLinePrefix,pmLeft);
  end;

  function IsCommentSuffix: boolean;
  begin
    IsCommentSuffix:=(MatchesAnySpecSymbol(ssCommentSuffix,pmRight))
      and (CurrentCommentType=SymbolIndex);
  end;

  function IsStringPrefix: boolean;
  begin
    IsStringPrefix:=MatchesAnySpecSymbol(ssStringPrefix,pmLeft);
  end;

  function IsStringSuffix: boolean;
  begin
    IsStringSuffix:=MatchesAnySpecSymbol(ssStringSuffix,pmRight);
  end;

  function IsDirectivePrefix: boolean;
  begin
    IsDirectivePrefix:=MatchesAnySpecSymbol(ssDirectivePrefix,pmLeft);
  end;

  function IsDirectiveSuffix: boolean;
  begin
    IsDirectiveSuffix:=MatchesAnySpecSymbol(ssDirectiveSuffix,pmRight);
  end;

  function IsAsmPrefix(const WordS: string): boolean;
  { var
     StoredMatchedSymbol : boolean;}
  begin
    {StoredMatchedSymbol:=MatchedSymbol;}
    IsAsmPrefix:=MatchesAsmSpecSymbol(WordS,ssAsmPrefix);
    {MatchedSymbol:=StoredMatchedSymbol;}
  end;

  function IsAsmSuffix(const WordS: string): boolean;
  {var
    StoredMatchedSymbol : boolean;}
  begin
    {StoredMatchedSymbol:=MatchedSymbol;}
    IsAsmSuffix:=MatchesAsmSpecSymbol(WordS,ssAsmSuffix);
    {MatchedSymbol:=StoredMatchedSymbol;}
  end;

  function GetCharClass(C: char): TCharClass;
  var CC: TCharClass;
  begin
  (*
     WhiteSpaceChars    {$ifdef USE_UNTYPEDSET}: set of char {$endif} = [#0,#32,#255];
     TabChars           {$ifdef USE_UNTYPEDSET}: set of char {$endif} = [#9];
     HashChars          {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['#'];
     AlphaChars         {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['A'..'Z','a'..'z','_'];
     NumberChars        {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['0'..'9'];
     HexNumberChars     {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['0'..'9','A'..'F','a'..'f'];
     RealNumberChars    {$ifdef USE_UNTYPEDSET}: set of char {$endif} = ['E','e','.'{,'+','-'}];
  *)
    if C in {$ifdef USE_UNTYPEDSET}[#0,#32,#255]{$else}WhiteSpaceChars{$endif} then
      CC:=ccWhiteSpace
    else if C in {$ifdef USE_UNTYPEDSET}[#9]{$else}TabChars{$endif} then
      CC:=ccTab
    else if C in {$ifdef USE_UNTYPEDSET}['#']{$else}HashChars{$endif} then
      CC:=ccHash
    else if (LastCC=ccHexNumber) and (C in {$ifdef USE_UNTYPEDSET}['0'..'9','A'..'F','a'..'f']{$else}HexNumberChars{$endif}) then
      CC:=ccHexNumber
    else if C in {$ifdef USE_UNTYPEDSET}['0'..'9']{$else}NumberChars{$endif} then
      CC:=ccNumber
    else if (LastCC=ccNumber) and (C in {$ifdef USE_UNTYPEDSET}['E','e','.']{$else}RealNumberChars{$endif}) then
      begin
        if (C='.') then
          begin
            if (X>=length(LineText)) or
               (LineText[X+1]='.') then
              cc:=ccSymbol
            else
              cc:=ccRealNumber;
          end
        else {'E','e'}
          begin
            if (X>=length(LineText)) or
               (LineText[X+1]in ['+','-','0'..'9']) then
              cc:=ccRealNumber
            else
              cc:=ccAlpha
          end;
      end
    else if C in {$ifdef USE_UNTYPEDSET}['A'..'Z','a'..'z','_']{$else}AlphaChars{$endif} then CC:=ccAlpha else
      CC:=ccSymbol;
    GetCharClass:=CC;
  end;

  procedure FormatWord(SClass: TCharClass; StartX:Sw_integer;EndX: Sw_integer);
  var
      C: byte;
      WordS: string;
  begin
    C:=0;
    WordS:=copy(LineText,StartX,EndX-StartX+1);
    if (InAsm=true) and (InComment=false) and (InString=false) and
        (InDirective=false) and (SClass=ccAlpha) and IsAsmSuffix(WordS) then InAsm:=false;
    if InDirective then C:=coDirectiveColor else
    if InComment then C:=coCommentColor else
    if InString then C:=coStringColor else
    if InAsm then
      begin
          if (SClass=ccAlpha) and Editor^.IsAsmReservedWord(WordS) then
            C:=coReservedWordColor
          else
            C:=coAssemblerColor;
      end
    else
    case SClass of
      ccWhiteSpace :
        C:=coWhiteSpaceColor;
      ccTab :
        C:=coTabColor;
      ccHexNumber:
        C:=coHexNumberColor;
      ccNumber,
      ccRealNumber :
        C:=coNumberColor;
      ccHash :
        C:=coStringColor;
      ccSymbol :
        C:=coSymbolColor;
      ccAlpha :
        begin
          if Editor^.IsReservedWord(WordS) then
            C:=coReservedWordColor
          else
            C:=coIdentifierColor;
        end;
    end;
    if EndX+1>=StartX then
      FillChar(Format[StartX],EndX+1-StartX,C);
    if (InString=false) and (InAsm=false) and (InComment=false) and
       (InDirective=false) and (SClass=ccAlpha) and IsAsmPrefix(WordS) then
      InAsm:=true;
  end;

  procedure ProcessChar(C: char);
  var CC: TCharClass;
      EX: Sw_integer;
      EndComment: pstring;
  begin
    CC:=GetCharClass(C);
    if ClassStart=X then
      FirstCC:=CC;
    if ( (CC<>LastCC) and
        (
         ((FirstCC=ccNumber) and (CC<>ccRealNumber) {and (CC<>ccNumber)}) or
        (((CC<>ccAlpha) or (LastCC<>ccNumber) ) and
          ( (CC<>ccNumber) or (LastCC<>ccAlpha) ) and
          ( (CC<>ccNumber) or (LastCC<>ccHash) ) and
          ( (CC<>ccRealNumber) or (LastCC<>ccNumber))
         ))) or

       (X>length(LineText)) or (CC=ccSymbol) then
      begin
        MatchedSymbol:=false;
        EX:=X-1;
        if (CC=ccSymbol) then
         begin
           if length(SymbolConcat)>=High(SymbolConcat) then
             Delete(SymbolConcat,1,1);
           SymbolConcat:=SymbolConcat+C;
           if  InComment and IsCommentSuffix then
              Inc(EX) else
           if InString and IsStringSuffix  then
              Inc(EX) else
           if InDirective and IsDirectiveSuffix then
              Inc(EX);
         end;
        if CC=ccRealNumber then
          Inc(EX);
        if (C='$') and (MatchedSymbol=false) and (IsDirectivePrefix=false) then
          CC:=ccHexNumber;
        if CC<>ccSymbol then SymbolConcat:='';
        FormatWord(LastCC,ClassStart,EX);
        ClassStart:=EX+1;
        if ClassStart=X then
          FirstCC:=CC;
        case CC of
          ccAlpha  : ;
          ccNumber :
            if (LastCC<>ccAlpha) then;
          ccSymbol :
              if (InComment=true) and (CurrentCommentType=1) and
                 (InDirective=false)  and IsDirectivePrefix then
                begin
                  InDirective:=true;
                  InComment:=false;
                  Dec(ClassStart,length(MatchingSymbol)-1);
                end
              else if (InComment=false) and
                 (InDirective=true) and IsDirectiveSuffix then
                 InDirective:=false
              else if (InComment=false) and
                 (InString=false) and (InDirective=false) and IsCommentPrefix then
                begin
                  InComment:=true;
                  CurrentCommentType:=SymbolIndex;
                  InSingleLineComment:=IsSingleLineCommentPrefix;
                  {InString:=false; }
                  Dec(ClassStart,length(MatchingSymbol)-1);
                  { Remove (* from SymbolConcat to avoid problem with (*) PM }
                  { fixes part of bug 1617 }
                  { but removed proper directive prefix detection ... }
                  EndComment:=Editor^.GetSpecSymbol(ssCommentSuffix,SymbolIndex);
                  if MatchingSymbol[length(MatchingSymbol)]=EndComment^[1] then
                    Delete(SymbolConcat,1,length(MatchingSymbol));
                end
              else if InComment and IsCommentSuffix then
                begin
                  InComment:=false;
                  InString:=false;
                end
              else if (InComment=false) and (InString=false) and IsStringPrefix then
                begin
                  InString:=true;
                  Dec(ClassStart,length(MatchingSymbol)-1);
                end
              else if (InComment=false) and (InString=true) and IsStringSuffix then
               InString:=false;
        end;
        if MatchedSymbol and (InComment=false) then
          SymbolConcat:='';
        LastCC:=CC;
      end;
  end;

var CurLineNr: Sw_integer;
    Line,NextLine,PrevLine{,OldLine}: PCustomLine;
    PrevLI,LI,nextLI: PEditorLineInfo;
begin
  if (not Editor^.IsFlagSet(efSyntaxHighlight)) or (FromLine>=GetLineCount) then
  begin
    SetLineFormat(Editor,FromLine,'');
    DoUpdateAttrs:=GetLineCount;
{$ifdef TEST_PARTIAL_SYNTAX}
    LastSyntaxedLine:=GetLineCount;
    if not SyntaxComplete then
      begin
        SyntaxComplete:=true;
        DoSyntaxStateChanged;
      end;
(*    { no Idle necessary }
    EventMask:=EventMask and not evIdle;*)
{$endif TEST_PARTIAL_SYNTAX}
    Editor^.SyntaxStateChanged;
    Exit;
  end;
{$ifdef TEST_PARTIAL_SYNTAX}
  If Editor^.IsFlagSet(efSyntaxHighlight) and (LastSyntaxedLine<FromLine)
     and (FromLine<GetLineCount) then
    CurLineNr:=LastSyntaxedLine
  else
{$endif TEST_PARTIAL_SYNTAX}
    CurLineNr:=FromLine;
  if CurLineNr>0 then
    PrevLine:=GetLine(CurLineNr-1)
  else
    PrevLine:=nil;
  repeat
    Line:=GetLine(CurLineNr);
    if Assigned(PrevLine) then PrevLI:=PrevLine^.GetEditorInfo(Editor) else PrevLI:=nil;
    if Assigned(Line) then LI:=Line^.GetEditorInfo(Editor) else LI:=nil;
    InSingleLineComment:=false;
    if PrevLI<>nil then
     begin
       InAsm:=PrevLI^.EndsWithAsm;
       InComment:=PrevLI^.EndsWithComment and not PrevLI^.EndsInSingleLineComment;
       CurrentCommentType:=PrevLI^.EndCommentType;
       InDirective:=PrevLI^.EndsWithDirective;
     end
    else
     begin
       InAsm:=false;
       InComment:=false;
       CurrentCommentType:=0;
       InDirective:=false;
     end;
{    OldLine:=Line;}
    if (not Editor^.IsFlagSet(efKeepLineAttr)) then
      begin
        LI^.BeginsWithAsm:=InAsm;
        LI^.BeginsWithComment:=InComment;
        LI^.BeginsWithDirective:=InDirective;
        LI^.BeginCommentType:=CurrentCommentType;
      end
    else
      begin
        InAsm:=LI^.BeginsWithAsm;
        InComment:=LI^.BeginsWithComment;
        InDirective:=LI^.BeginsWithDirective;
        CurrentCommentType:=LI^.BeginCommentType;
      end;
    LineText:=GetLineText(CurLineNr);
    Format:=CharStr(chr(coTextColor),length(LineText));
    LastCC:=ccWhiteSpace;
    ClassStart:=1;
    SymbolConcat:='';
    InString:=false;
    if LineText<>'' then
     begin
       for X:=1 to length(LineText) do
         ProcessChar(LineText[X]);
       Inc(X);
       ProcessChar(' ');
     end;
    SetLineFormat(Editor,CurLineNr,Format);
    LI^.EndsWithAsm:=InAsm;
    LI^.EndsWithComment:=InComment;
    LI^.EndsInSingleLineComment:=InSingleLineComment;
    LI^.EndCommentType:=CurrentCommentType;
    LI^.EndsWithDirective:=InDirective;
    Inc(CurLineNr);
    if CurLineNr>=GetLineCount then
     Break;
    NextLine:=GetLine(CurLineNr);
    if Assigned(NextLine) then NextLI:=NextLine^.GetEditorInfo(Editor) else NextLI:=nil;
    if ((Attrs and attrForceFull)=0) then
      if (*  Why should we go
         (InAsm=false) and (NextLI^.BeginsWithAsm=false) and
         (InComment=false) and (NextLI^.BeginsWithComment=false) and
         (InDirective=false) and (NextLI^.BeginsWithDirective=false) and
{          OldLine = Line so this is nonsense}
         (PrevLI^.EndsWithComment=LI^.EndsWithComment) and
         (PrevLI^.EndsWithAsm=LI^.EndsWithAsm) and
         (PrevLI^.EndsWithDirective=LI^.EndsWithDirective) and *)
{$ifdef TEST_PARTIAL_SYNTAX}
         (CurLineNr>FromLine) and
{$endif TEST_PARTIAL_SYNTAX}
         (NextLI^.BeginsWithAsm=LI^.EndsWithAsm) and
         (NextLI^.BeginsWithComment=LI^.EndsWithComment) and
         (NextLI^.BeginsWithDirective=LI^.EndsWithDirective) and
         (NextLI^.BeginCommentType=LI^.EndCommentType) and
         (NextLI^.Format<>nil) then
       Break;
{$ifdef TEST_PARTIAL_SYNTAX}
    if (CurLineNr<GetLineCount) and
       (CurLineNr>FromLine) and
       ((Attrs and attrForceFull)=0) and
       (CurLineNr>GetLastVisibleLine) then
      begin
        If SyntaxComplete then
          begin
            SyntaxComplete:=false;
            DoSyntaxStateChanged;
          end;
        LastSyntaxedLine:=CurLineNr-1;
        break;
      end;
{$endif TEST_PARTIAL_SYNTAX}
    PrevLine:=Line;
  until false;
  DoUpdateAttrs:=CurLineNr;
{$ifdef TEST_PARTIAL_SYNTAX}
  If LastSyntaxedLine<CurLineNr-1 then
    LastSyntaxedLine:=CurLineNr-1;
  if CurLineNr=GetLineCount then
    begin
      SyntaxComplete:=true;
      DoSyntaxStateChanged;
    end;
{$endif TEST_PARTIAL_SYNTAX}
end;

function TCustomCodeEditorCore.DoUpdateAttrsRange(Editor: PCustomCodeEditor; FromLine, ToLine: sw_integer;
         Attrs: byte): sw_integer;
var Line: Sw_integer;
begin
  Lock(Editor);
  Line:=FromLine;
  repeat
    Line:=DoUpdateAttrs(Editor,Line,Attrs);
  until (Line>=GetLineCount) or (Line>ToLine);
  DoUpdateAttrsRange:=Line;
  Unlock(Editor);
end;

procedure TCustomCodeEditorCore.AddAction(AAction: byte; AStartPos, AEndPos: TPoint; AText: string;AFlags : longint);
begin
  Abstract;
end;

procedure TCustomCodeEditorCore.AddGroupedAction(AAction : byte);
begin
  Abstract;
end;

procedure TCustomCodeEditorCore.CloseGroupedAction(AAction : byte);
begin
  Abstract;
end;

function TCustomCodeEditorCore.GetUndoActionCount: sw_integer;
begin
  Abstract;
  GetUndoActionCount:=0;
end;

function TCustomCodeEditorCore.GetRedoActionCount: sw_integer;
begin
  Abstract;
  GetRedoActionCount:=0;
end;

destructor TCustomCodeEditorCore.Done;
begin
{$ifdef DEBUG}
  if Bindings^.Count>0 then
    ErrorBox('Internal error: there are still '+IntToStr(Bindings^.Count)+' editors '+
      'registered at TCodeEditorCode.Done!!!',nil);
{$endif}
  if Assigned(Bindings) then Dispose(Bindings, Done); Bindings:=nil;
  inherited Done;
end;

procedure TCustomCodeEditor.Lock;
begin
  Inc(ELockFlag);
  LockScreenUpdate;
end;

procedure TCustomCodeEditor.UnLock;
begin
{$ifdef DEBUG}
  if Elockflag=0 then
    Bug('negative lockflag',nil)
  else
{$endif DEBUG}
  UnlockScreenUpdate;
  Dec(ELockFlag);
  if (ELockFlag>0) then
    Exit;

  if DrawCalled then
    DrawView;

  If DrawCursorCalled then
    Begin
      DrawCursor;
      DrawCursorCalled:=false;
    End;
end;

procedure TCustomCodeEditor.DrawIndicator;
begin
  { Abstract }
end;

procedure TCustomCodeEditor.AdjustSelectionPos(OldCurPosX, OldCurPosY: sw_integer; DeltaX, DeltaY: sw_integer);
var CP: TPoint;
begin
  if ValidBlock=false then Exit;

  CP.X:=OldCurPosX; CP.Y:=OldCurPosY;
  if (PosToOfsP(SelStart)<=PosToOfsP(CP)) and (PosToOfsP(CP)<PosToOfsP(SelEnd)) then
    begin
      { OldCurPos is IN selection }
      if (CP.Y=SelEnd.Y) then
        begin
          if ((SelStart.Y<>SelEnd.Y) or (SelStart.X<=CP.X)) and
             (CP.X<=SelEnd.X) then
            Inc(SelEnd.X,DeltaX);
        end
      else if (CP.Y=SelEnd.Y+DeltaY) then
        Inc(SelEnd.X,DeltaX);
      Inc(SelEnd.Y,DeltaY);
      SelectionChanged;
    end
  else
  if (PosToOfsP(CP)<=PosToOfsP(SelStart)) then
    begin
      { OldCurPos is BEFORE selection }
      if (CP.Y=SelStart.Y) and (CP.Y=SelEnd.Y) and (DeltaY<0) then
        begin
          SelStart:=CurPos; SelEnd:=CurPos;
        end
      else
      if (CP.Y=SelStart.Y) then
        begin
          if CP.X<SelStart.X then
            Inc(SelStart.X,DeltaX);
        end;
{      else}
        begin
          Inc(SelStart.Y,DeltaY);
          Inc(SelEnd.Y,DeltaY);
        end;
      if SelEnd.Y=CurPos.Y then Inc(SelEnd.X,DeltaX);
      SelectionChanged;
    end
  else
    begin
      { OldCurPos is AFTER selection }
      { actually we don't have to do anything here }
    end;
end;

function TCustomCodeEditor.GetFlags: longint;
begin
  { Abstract }
  GetFlags:=0;
end;

procedure TCustomCodeEditor.SetFlags(AFlags: longint);
begin
  { Abstract }
end;

function TCustomCodeEditor.GetModified: boolean;
begin
  { Abstract }
  GetModified:=true;
end;

procedure TCustomCodeEditor.SetModified(AModified: boolean);
begin
  { Abstract }
end;

function TCustomCodeEditor.GetStoreUndo: boolean;
begin
  { Abstract }
  GetStoreUndo:=false;
end;

procedure TCustomCodeEditor.SetStoreUndo(AStore: boolean);
begin
  { Abstract }
end;

function TCustomCodeEditor.GetSyntaxCompleted: boolean;
begin
  { Abstract }
  GetSyntaxCompleted:=true;
end;

procedure TCustomCodeEditor.SetSyntaxCompleted(SC : boolean);
begin
  { Abstract }
end;

function  TCustomCodeEditor.GetLastSyntaxedLine: sw_integer;
begin
  Abstract;
  GetLastSyntaxedLine:=0;
end;

procedure   TCustomCodeEditor.SetLastSyntaxedLine(ALine: sw_integer);
begin
  Abstract;
end;

function TCustomCodeEditor.IsFlagSet(AFlag: longint): boolean;{$ifdef USEINLINE}inline;{$endif}
begin
  IsFlagSet:=(GetFlags and AFlag)=AFlag;
end;

function TCustomCodeEditor.GetTabSize: integer;
begin
  { Abstract }
  GetTabSize:=5;
end;

procedure TCustomCodeEditor.SetTabSize(ATabSize: integer);
begin
  { Abstract }
end;

function TCustomCodeEditor.GetIndentSize: integer;
begin
  { Abstract }
  GetIndentSize:=1;
end;

procedure TCustomCodeEditor.SetIndentSize(AIndentSize: integer);
begin
  { Abstract }
end;

function TCustomCodeEditor.IsReadOnly: boolean;
begin
  { Abstract }
  IsReadOnly:=false;
end;

function TCustomCodeEditor.IsClipboard: Boolean;
begin
  { Abstract }
  IsClipboard:=false;
end;

function TCustomCodeEditor.GetLineCount: sw_integer;
begin
  Abstract;
  GetLineCount:=0;
end;

function TCustomCodeEditor.GetLine(LineNo: sw_integer): PCustomLine;
begin
  Abstract;
  GetLine:=nil;
end;

function TCustomCodeEditor.CharIdxToLinePos(Line,CharIdx: sw_integer): sw_integer;
begin
  Abstract;
  CharIdxToLinePos:=0;
end;

function TCustomCodeEditor.LinePosToCharIdx(Line,X: sw_integer): sw_integer;
begin
  Abstract;
  LinePosToCharIdx:=0;
end;

function TCustomCodeEditor.GetLineText(I: sw_integer): string;
begin
  Abstract;
  GetLineText:='';
end;

procedure TCustomCodeEditor.SetDisplayText(I: sw_integer;const S: string);
begin
  Abstract;
end;

function TCustomCodeEditor.GetDisplayText(I: sw_integer): string;
begin
  Abstract;
  GetDisplayText:='';
end;

procedure TCustomCodeEditor.SetLineText(I: sw_integer;const S: string);
begin
  Abstract;
end;

procedure TCustomCodeEditor.GetDisplayTextFormat(I: sw_integer;var DT,DF:string);
begin
  Abstract;
end;

function TCustomCodeEditor.GetLineFormat(I: sw_integer): string;
begin
  { Abstract }
  GetLineFormat:='';
end;

procedure TCustomCodeEditor.SetLineFormat(I: sw_integer;const S: string);
begin
  { Abstract }
end;

procedure TCustomCodeEditor.DeleteAllLines;
begin
  Abstract;
end;

procedure TCustomCodeEditor.DeleteLine(I: sw_integer);
begin
  Abstract;
end;

function TCustomCodeEditor.InsertLine(LineNo: sw_integer; const S: string): PCustomLine;
begin
  Abstract;
  InsertLine:=nil; { eliminate compiler warning }
end;

procedure TCustomCodeEditor.AddLine(const S: string);
begin
  Abstract;
end;

function TCustomCodeEditor.GetErrorMessage: string;
begin
  Abstract;
  GetErrorMessage:='';
end;

procedure TCustomCodeEditor.SetErrorMessage(const S: string);
begin
  Abstract;
end;

procedure TCustomCodeEditor.GetContent(ALines: PUnsortedStringCollection);
begin
  Abstract;
end;

procedure TCustomCodeEditor.SetContent(ALines: PUnsortedStringCollection);
begin
  Abstract;
end;

function TCustomCodeEditor.LoadFromStream(Stream: PFastBufStream): boolean;
begin
  Abstract;
  LoadFromStream:=false;
end;

function TCustomCodeEditor.SaveToStream(Stream: PStream): boolean;
var A,B: TPoint;
begin
  A.Y:=0; A.X:=0;
  B.Y:=GetLineCount-1;
  if GetLineCount>0 then
    B.X:=length(GetDisplayText(B.Y))
  else
    B.X:=0;
  SaveToStream:=SaveAreaToStream(Stream,A,B);
end;

function TCustomCodeEditor.SaveAreaToStream(Stream: PStream; StartP,EndP: TPoint): boolean;
begin
  Abstract;
  SaveAreaToStream:=false;
end;

function TCustomCodeEditor.LoadFromFile(const AFileName: string): boolean;
var S: PFastBufStream;
    OK: boolean;
begin
  New(S, Init(AFileName,stOpenRead,EditorTextBufSize));
  OK:=Assigned(S);
{$ifdef TEST_PARTIAL_SYNTAX}
  SetSyntaxCompleted(false);
  { Idle necessary }
  EventMask:=EventMask or evIdle;
{$endif TEST_PARTIAL_SYNTAX}
  if OK then OK:=LoadFromStream(S);
  if Assigned(S) then Dispose(S, Done);
  LoadFromFile:=OK;
end;

function TCustomCodeEditor.SaveToFile(const AFileName: string): boolean;
var OK: boolean;
    S: PBufStream;
begin
  New(S, Init(AFileName,stCreate,EditorTextBufSize));
  OK:=Assigned(S) and (S^.Status=stOK);
  if OK then OK:=SaveToStream(S);
  if Assigned(S) then Dispose(S, Done);
  SaveToFile:=OK;
end;


function TCustomCodeEditor.InsertFrom(Editor: PCustomCodeEditor): Boolean;
var OK: boolean;
    CP,RX,RSX,LineDelta,LineCount: Sw_integer;
    StartPos,DestPos,BPos,EPos: TPoint;
    LineStartX,LineEndX: Sw_integer;
    TabSize,CharIdxStart,CharIdxEnd: Sw_integer;
    S,DS,BeforeS,OrigS,AfterS: string;
    VerticalBlock: boolean;
    SEnd: TPoint;
begin
  if Editor^.IsFlagSet(efVerticalBlocks) then
    begin
      NotImplemented;
      Exit;
    end;
  Lock;

  { every data in the clipboard gets a new line }
  if (Clipboard=@Self) and (CurPos.X>0) then
    InsertNewLine;

  OK:=(Editor^.SelStart.X<>Editor^.SelEnd.X) or (Editor^.SelStart.Y<>Editor^.SelEnd.Y);
  if OK then
  begin
    StartPos:=CurPos; DestPos:=CurPos;
    EPos:=CurPos;
    VerticalBlock:=Editor^.IsFlagSet(efVerticalBlocks);
    LineDelta:=0; LineCount:=(Editor^.SelEnd.Y-Editor^.SelStart.Y)+1;
    OK:=GetLineCount<MaxLineCount;
    OrigS:=GetLineText(DestPos.Y);
    BeforeS:=Copy(OrigS,1,LinePosToCharIdx(DestPos.Y,DestPos.X-1));
    { we might need to add some spaces here,
      but how many ? }
    TabSize:=GetTabSize;
    CP:=1; RX:=0;
    while (CP<=length(BeforeS)) do
      begin
        if (BeforeS[CP]=TAB) then
          Inc(RX,TabSize-(RX mod TabSize))
        else
          Inc(RX);
        Inc(CP);
      end;
    BeforeS:=BeforeS+CharStr(' ',DestPos.X-RX);
    AfterS:=Copy(OrigS,LinePosToCharIdx(DestPos.Y,DestPos.X),High(OrigS));
    BPos:=CurPos;
    while OK and (LineDelta<LineCount) do
    begin
      if (LineDelta>0) and (VerticalBlock=false) then
        begin
          InsertLine(DestPos.Y,'');
          EPOS.X:=0;EPos.Y:=DestPos.Y;
          AddAction(eaInsertLine,BPos,EPos,'',GetFlags);
          LimitsChanged;
        end;

      If LineDelta>0 then
        BeforeS:='';
      if (LineDelta=0) or VerticalBlock then
        LineStartX:=Editor^.SelStart.X
      else
        LineStartX:=0;

      if (LineDelta=LineCount-1) or VerticalBlock then
        LineEndX:=Editor^.SelEnd.X-1
      else
        LineEndX:=High(S);

      CharIdxStart:=Editor^.LinePosToCharIdx(Editor^.SelStart.Y+LineDelta,LineStartX);
      CharIdxEnd:=Editor^.LinePosToCharIdx(Editor^.SelStart.Y+LineDelta,LineEndX);
      if LineEndX<LineStartX then
        S:=''
      else if VerticalBlock then
        S:=RExpand(copy(Editor^.GetLineText(Editor^.SelStart.Y+LineDelta),CharIdxStart,CharIdxEnd-CharIdxStart+1),
                   Min(CharIdxEnd-CharIdxStart+1,High(S)))
      else
        S:=copy(Editor^.GetLineText(Editor^.SelStart.Y+LineDelta),CharIdxStart,CharIdxEnd-CharIdxStart+1);
      if VerticalBlock=false then
        begin
          DS:=BeforeS+S;
          CP:=1; RX:=0;
          RSX :=0;
          while (CP<=length(DS)) do
            begin
              if (DS[CP]=TAB) then
                Inc(RX,TabSize-(RX mod TabSize))
              else
                Inc(RX);
              if CP=length(BeforeS) then
                RSX:=RX;
              Inc(CP);
            end;

          if LineDelta=LineCount-1 then
            begin
              SetLineText(DestPos.Y,DS+AfterS);
              BPos.X:=DestPos.X;BPos.Y:=DestPos.Y;
              EPOS.X:=DestPos.X+RX-RSX;EPos.Y:=DestPos.Y;
              AddAction(eaInsertText,BPos,EPos,S,GetFlags);
            end
          else
            begin
              SetLineText(DestPos.Y,DS);
              BPos.X:=DestPos.X;BPos.Y:=DestPos.Y;
              EPOS.X:=DestPos.X+RX-RSX;EPos.Y:=DestPos.Y;
              AddAction(eaInsertText,BPos,EPos,S,GetFlags);
            end;
          BPos.X:=EPos.X;
          if LineDelta=LineCount-1 then
            begin
              SEnd.Y:=DestPos.Y;
              SEnd.X:=DestPos.X+RX-RSX;
            end
          else
           begin
             Inc(DestPos.Y);
             DestPos.X:=0;
           end;
        end
      else { if VerticalBlock=false then .. else }
        begin
          { this is not yet implemented !! PM }
          S:=RExpand(S,LineEndX-LineStartX+1);
        end;
      Inc(LineDelta);
      OK:=GetLineCount<MaxLineCount;
    end;
    if not OK then EditorDialog(edTooManyLines,nil);
    { mainly to force eaMove insertion }
    if not IsClipboard then
      SetCurPtr(EPos.X,EPos.Y);
    SetCurPtr(StartPos.X,StartPos.Y);
    UpdateAttrs(StartPos.Y,attrAll);
    SetModified(true);
    LimitsChanged;
    SetSelection(CurPos,SEnd);
    if IsClipboard then
     begin
       Inc(DestPos.X,length(S));
       SetCurPtr(DestPos.X,DestPos.Y);
     end;
    DrawView;
  end;
  UnLock;
  InsertFrom:=OK;
end;

function TCustomCodeEditor.InsertText(const S: string): Boolean;
var I: sw_integer;
    OldPos: TPoint;
    HoldUndo : boolean;
begin
  Lock;
  OldPos:=CurPos;
  HoldUndo:=GetStoreUndo;
  SetStoreUndo(false);
  for I:=1 to length(S) do
    AddChar(S[I]);
  InsertText:=true;
  SetStoreUndo(HoldUndo);
  AddAction(eaInsertText,OldPos,CurPos,S,GetFlags);
  UnLock;
end;

procedure TCustomCodeEditor.ModifiedChanged;
begin
  { Abstract }
end;

procedure TCustomCodeEditor.PositionChanged;
begin
  { Abstract }
end;

procedure TCustomCodeEditor.TabSizeChanged;
begin
  { Abstract }
end;

procedure TCustomCodeEditor.SyntaxStateChanged;
begin
  { Abstract }
end;

procedure TCustomCodeEditor.StoreUndoChanged;
begin
  { Abstract }
end;

function TCustomCodeEditor.GetSpecSymbolCount(SpecClass: TSpecSymbolClass): integer;
begin
  { Abstract }
  GetSpecSymbolCount:=0;
end;

function TCustomCodeEditor.GetSpecSymbol(SpecClass: TSpecSymbolClass; Index: integer): pstring;
begin
  Abstract;
  GetSpecSymbol:=nil;
end;

function TCustomCodeEditor.IsReservedWord(const S: string): boolean;
begin
  { Abstract }
  IsReservedWord:=false;
end;

function TCustomCodeEditor.IsAsmReservedWord(const S: string): boolean;
begin
  { Abstract }
  IsAsmReservedWord:=false;
end;

function TCustomCodeEditor.TranslateCodeTemplate(var Shortcut: string; ALines: PUnsortedStringCollection): boolean;
begin
  { Abstract }
  TranslateCodeTemplate:=false;
end;

function TCustomCodeEditor.CompleteCodeWord(const WordS: string; var Text: string): boolean;
begin
  { Abstract }
  Text:='';
  CompleteCodeWord:=false;
end;

function TCustomCodeEditor.GetCodeCompleteWord: string;
begin
  { Abstract }
  GetCodeCompleteWord:='';
end;

function TCustomCodeEditor.CreateFold(StartY,EndY: sw_integer; Collapsed: boolean): boolean;
var F,ParentF: PFold;
    L: PCustomLine;
    EI: PEditorLineInfo;
    Y: sw_integer;
    OK: boolean;
begin
  OK:=true;
  Lock;
  for Y:=StartY to EndY do
  begin
    L:=GetLine(Y);
    if assigned(L) then
      EI:=L^.GetEditorInfo(@Self)
    else
      begin
        CreateFold:=False;
        exit;
      end;
    if Y=StartY then
      ParentF:=EI^.Fold
    else
      OK:=OK and (EI^.Fold=ParentF);
    if not OK then
      Break;
  end;
  if OK then
  begin
    New(F, Init(@Self,ParentF,Collapsed));
    for Y:=StartY to EndY do
      GetLine(Y)^.GetEditorInfo(@Self)^.SetFold(F);
    DrawView;
  end;
  UnLock;
  CreateFold:=OK;
end;

procedure TCustomCodeEditor.FoldChanged(Fold: PFold);
var F: PFold;
    I: sw_integer;
begin
  for I:=0 to GetFoldCount-1 do
  begin
    F:=GetFold(I);
    if F^.ParentFold=Fold then
      FoldChanged(F);
  end;
  if Fold^.IsCollapsed then
  begin
    F:=GetLineFold(CurPos.Y); I:=CurPos.Y;
    if F=Fold then
    begin
     while GetLineFold(I-1)=Fold do
       Dec(I);
     if I<>CurPos.Y then
       SetCurPtr(CurPos.X,I);
    end;
  end;
  DrawView;
end;

procedure TCustomCodeEditor.RemoveAllFolds;
var I: sw_integer;
    L: PCustomLine;
begin

  for I:=0 to GetLineCount-1 do
    begin
      L:=GetLine(I);
      if not assigned(L) then exit;
      with L^ do
        with GetEditorInfo(@Self)^ do
          SetFold(nil);
    end;
  DrawView;
end;

{ to be called if CurPos has already been changed }

procedure TCustomCodeEditor.AdjustSelection(DeltaX, DeltaY: sw_integer);
begin
  AdjustSelectionPos(CurPos.X-DeltaX,CurPos.Y-DeltaY,DeltaX,DeltaY);
end;

{ to be called if CurPos has not yet been changed }

procedure TCustomCodeEditor.AdjustSelectionBefore(DeltaX, DeltaY: sw_integer);
begin
  AdjustSelectionPos(CurPos.X,CurPos.Y,DeltaX,DeltaY);
end;

procedure TCustomCodeEditor.TrackCursor(centre:Tcentre);
var D,CP: TPoint;
begin
  D:=Delta;
  EditorToViewPoint(D,D); EditorToViewPoint(CurPos,CP);
  if CP.Y<Delta.Y then D.Y:=CP.Y else
   if CP.Y>Delta.Y+Size.Y-1 then D.Y:=CP.Y-Size.Y+1;
  if CP.X<Delta.X then D.X:=CP.X else
   if CP.X>Delta.X+Size.X-1 then D.X:=CP.X-Size.X+1;
  if {((Delta.X<>D.X) or (Delta.Y<>D.Y)) and }centre=do_centre then
  begin
     { loose centering for debugger PM }
     while (CP.Y-D.Y)<(Size.Y div 3) do Dec(D.Y);
     while (CP.Y-D.Y)>2*(Size.Y div 3) do Inc(D.Y);
  end;
  ViewToEditorPoint(D,D);
  if (Delta.X<>D.X) or (Delta.Y<>D.Y) then
    ScrollTo(D.X,D.Y);
  DrawCursor;
end;

procedure TCustomCodeEditor.ScrollTo(X, Y: sw_Integer);
begin
  inherited ScrollTo(X,Y);
  if (HScrollBar=nil) or (VScrollBar=nil) then
     begin Delta.X:=X; Delta.Y:=Y; end;
  DrawView;
end;

function TCustomCodeEditor.IsModal: boolean;
var IsM: boolean;
begin
  IsM:=GetState(sfModal);
  if Assigned(Owner) then
    IsM:=IsM or Owner^.GetState(sfModal);
  IsModal:=IsM;
end;

procedure TCustomCodeEditor.FlagsChanged(OldFlags: longint);
var I: sw_integer;
begin
  Lock;
  if ((OldFlags xor GetFlags) and efCodeComplete)<>0 then
    ClearCodeCompleteWord;
  SetInsertMode(IsFlagSet(efInsertMode));
  if ((OldFlags xor GetFlags) and efFolds)<>0 then
    if not IsFlagSet(efFolds) then
      RemoveAllFolds;
  if IsFlagSet(efSyntaxHighlight) then
    UpdateAttrs(0,attrAll) else
  for I:=0 to GetLineCount-1 do
    SetLineFormat(I,'');
  DrawView;
  UnLock;
end;

procedure TCustomCodeEditor.LimitsChanged;
begin
  Abstract;
end;

procedure TCustomCodeEditor.DoLimitsChanged;
begin
  SetLimit(MaxLineLength+1,EditorToViewLine(GetLineCount));
end;

procedure TCustomCodeEditor.BindingsChanged;
begin
  { Abstract }
end;

procedure TCustomCodeEditor.ContentsChanged;
begin
  DrawView;
end;

procedure TCustomCodeEditor.ConvertEvent(var Event: TEvent);
var
  Key: Word;
begin
  if Event.What = evKeyDown then
  begin
    if (Event.KeyShift and kbShift <> 0) and
      (Event.ScanCode >= $47) and (Event.ScanCode <= $51) then
      Event.CharCode := #0;
    Key := Event.KeyCode;
    if KeyState <> 0 then
    begin
      if (Lo(Key) >= $01) and (Lo(Key) <= $1A) then Inc(Key, $40);
      if (Lo(Key) >= $61) and (Lo(Key) <= $7A) then Dec(Key, $20);
    end;
    Key := ScanKeyMap(KeyMap[KeyState], Key);
    if (KeyState<>0) and (Key=0) then
      ClearEvent(Event); { eat second key if unrecognized after ^Q or ^K }
    KeyState := 0;
    if Key <> 0 then
      if Hi(Key) = $FF then
        begin
          KeyState := Lo(Key);
          ClearEvent(Event);
        end
      else
        begin
          Event.What := evCommand;
          Event.Command := Key;
        end;
  end;
end;

procedure TCustomCodeEditor.SetLineFlagState(LineNo: sw_integer; Flags: longint; ASet: boolean);
var L: PCustomLine;
begin
  { Avoid crashes if file was shorten for instance }
  if LineNo>=GetLineCount then
    exit;
  L:=GetLine(LineNo);
  if Assigned(L) then
    with L^ do
      if ASet then
        SetFlags(GetFlags or Flags)
      else
        SetFlags(GetFlags and not Flags);
end;

procedure TCustomCodeEditor.SetLineFlagExclusive(Flags: longint; LineNo: sw_integer);
var I,Count: sw_integer;
    L: PCustomLine;
begin
  Lock;
  Count:=GetLineCount;
  for I:=0 to Count-1 do
  begin
    L:=GetLine(I);
    if not assigned(L) then break;
    if I=LineNo then
      L^.SetFlags(L^.GetFlags or Flags)
    else
      L^.SetFlags(L^.GetFlags and (not Flags));
  end;
  UnLock;
end;

procedure TCustomCodeEditor.HandleEvent(var Event: TEvent);
var DontClear : boolean;

  procedure CheckScrollBar(P: PScrollBar; var D: Sw_Integer);
  begin
    if (Event.InfoPtr = P) and (P^.Value <> D) then
    begin
      D := P^.Value;
      DrawView;
    end;
  end;

  procedure GetMousePos(var P: TPoint);
  begin
    MakeLocal(Event.Where,P);
    Inc(P.X,Delta.X); Inc(P.Y,Delta.Y);
    Dec(P.X,GetReservedColCount);
    if P.X<0 then P.X:=0;
    if P.Y<0 then P.Y:=0;
  end;
type TCCAction = (ccCheck,ccClear,ccDontCare);
var
  StartP,P: TPoint;
  E: TEvent;
  OldEvent : PEvent;
  CCAction: TCCAction;
begin
  CCAction:=ccClear;
  E:=Event;
  OldEvent:=CurEvent;
  if (E.What and (evMouse or evKeyboard))<>0 then
    CurEvent:=@E;
  if (InASCIIMode=false) or (Event.What<>evKeyDown) then
   if (Event.What<>evKeyDown) or (Event.KeyCode<>kbEnter) or (IsReadOnly=false) then
   if (Event.What<>evKeyDown) or
      ((Event.KeyCode<>kbEnter) and (Event.KeyCode<>kbEsc)) or
      (GetCompleteState<>csOffering) then
    ConvertEvent(Event);
  case Event.What of
    evMouseDown :
      if MouseInView(Event.Where) then
       if (Event.Buttons=mbRightButton) then
         begin
           MakeLocal(Event.Where,P); Inc(P.X); Inc(P.Y);
           LocalMenu(P);
           ClearEvent(Event);
         end else
       if Event.Buttons=mbLeftButton then
        begin
          GetMousePos(P);
          StartP:=P;
          SetCurPtr(P.X,P.Y);
          repeat
            GetMousePos(P);
            if PointOfs(P)<PointOfs(StartP)
               then SetSelection(P,StartP)
               else SetSelection(StartP,P);
            SetCurPtr(P.X,P.Y);
            DrawView;
          until not MouseEvent(Event, evMouseMove+evMouseAuto);
          DrawView;
        end;
    evKeyDown :
      begin
        { Scancode is almost never zero PM }
        { this is supposed to enable entering of ASCII chars below 32,
          which are normally interpreted as control chars. So, when you enter
          Alt+24 (on the numeric pad) then this will normally move the cursor
          one line down, but if you do it in ASCII mode (also after Ctrl+B)
          then this will insert the ASCII #24 char (upper arrow) in the
          source code. - Gabor }
        if InASCIIMode {and (Event.CharCode<>0)} then
          begin
            AddChar(Event.CharCode);
            if (GetCompleteState<>csDenied) or (Event.CharCode=#32) then
              CCAction:=ccCheck
            else
              CCAction:=ccClear;
          end
        else
          begin
           DontClear:=false;
           case Event.KeyCode of
             kbAltF10 :
               Message(@Self,evCommand,cmLocalMenu,@Self);
             kbEnter  :
               if IsReadOnly then
                 DontClear:=true else
               if GetCompleteState=csOffering then
                 CodeCompleteApply
               else
                 Message(@Self,evCommand,cmNewLine,nil);
             kbEsc :
               if GetCompleteState=csOffering then
                 CodeCompleteCancel else
                if IsModal then
                  DontClear:=true;
           else
            case Event.CharCode of
             #9,#32..#255 :
               if (Event.CharCode=#9) and IsModal then
                 DontClear:=true
               else
                 begin
                   NoSelect:=true;
                   AddChar(Event.CharCode);
                   NoSelect:=false;
                   if (GetCompleteState<>csDenied) or (Event.CharCode=#32) then
                     CCAction:=ccCheck
                   else
                     CCAction:=ccClear;
                 end;
            else
              DontClear:=true;
            end; { case Event.CharCode .. }
           end; { case Event.KeyCode .. }
            if not DontClear then
             ClearEvent(Event);
          end;
        InASCIIMode:=false;
      end;
    evCommand :
      begin
        DontClear:=false;
        case Event.Command of
          cmASCIIChar   : InASCIIMode:=not InASCIIMode;
          cmAddChar     : AddChar(chr(longint(Event.InfoPtr)));
          cmCharLeft    : CharLeft;
          cmCharRight   : CharRight;
          cmWordLeft    : WordLeft;
          cmWordRight   : WordRight;
          cmLineStart   : LineStart;
          cmLineEnd     : LineEnd;
          cmLineUp      : LineUp;
          cmLineDown    : LineDown;
          cmPageUp      : PageUp;
          cmPageDown    : PageDown;
          cmTextStart   : TextStart;
          cmTextEnd     : TextEnd;
          cmWindowStart : WindowStart;
          cmWindowEnd   : WindowEnd;
          cmNewLine     : begin
                            InsertNewLine;
                            TrackCursor(do_not_centre);
                          end;
          cmBreakLine   : BreakLine;
          cmBackSpace   : BackSpace;
          cmDelChar     : DelChar;
          cmDelWord     : DelWord;
          cmDelToEndOfWord : DelToEndOfWord;
          cmDelStart    : DelStart;
          cmDelEnd      : DelEnd;
          cmDelLine     : DelLine;
          cmInsMode     : InsMode;
          cmStartSelect : StartSelect;
          cmHideSelect  : HideSelect;
          cmUpdateTitle : ;
          cmEndSelect   : EndSelect;
          cmDelSelect   : DelSelect;
          cmCopyBlock   : CopyBlock;
          cmMoveBlock   : MoveBlock;
          cmIndentBlock   : IndentBlock;
          cmUnindentBlock : UnindentBlock;
          cmSelStart    : JumpSelStart;
          cmSelEnd      : JumpSelEnd;
          cmLastCursorPos : JumpToLastCursorPos;
          cmFindMatchingDelimiter : FindMatchingDelimiter(true);
          cmFindMatchingDelimiterBack : FindMatchingDelimiter(false);
          cmUpperCase     : UpperCase;
          cmLowerCase     : LowerCase;
          cmWordLowerCase : WordLowerCase;
          cmWordUpperCase : WordUpperCase;
          cmInsertOptions : InsertOptions;
          cmToggleCase    : ToggleCase;
          cmCreateFold    : CreateFoldFromBlock;
          cmToggleFold    : ToggleFold;
          cmExpandFold    : ExpandFold;
          cmCollapseFold  : CollapseFold;
          cmJumpMark0..cmJumpMark9 : JumpMark(Event.Command-cmJumpMark0);
          cmSetMark0..cmSetMark9 : DefineMark(Event.Command-cmSetMark0);
          cmSelectWord  : SelectWord;
          cmSelectLine  : SelectLine;
          cmWriteBlock  : WriteBlock;
          cmReadBlock   : ReadBlock;
          cmPrintBlock  : PrintBlock;
        { ------ }
          cmFind        : Find;
          cmReplace     : Replace;
          cmSearchAgain : DoSearchReplace;
          cmJumpLine    : GotoLine;
        { ------ }
          cmCut         : ClipCut;
          cmCopy        : ClipCopy;
          cmPaste       : ClipPaste;

          cmSelectAll   : SelectAll(true);
          cmUnselect    : SelectAll(false);
{$ifdef WinClipSupported}
          cmCopyWin     : ClipCopyWin;
          cmPasteWin    : ClipPasteWin;
{$endif WinClipSupported}
          cmUndo        : Undo;
          cmRedo        : Redo;
          cmClear       : DelSelect;
          cmExpandCodeTemplate: ExpandCodeTemplate;
          cmLocalMenu :
            begin
              P:=CurPos; Inc(P.X); Inc(P.Y);
              LocalMenu(P);
            end;
          cmActivateMenu :
            Message(Application,evCommand,cmMenu,nil);
        else
          begin
            DontClear:=true;
            CCAction:=ccDontCare;
          end;
        end;
        if DontClear=false then
          ClearEvent(Event);
      end;
{$ifdef TEST_PARTIAL_SYNTAX}
    evIdle :
      begin
        CCAction:=ccDontCare;
        { Complete syntax by 20 lines increment }
        { could already be quite lengthy on slow systems }
        if not GetSyntaxCompleted then
          UpdateAttrsRange(GetLastSyntaxedLine,GetLastSyntaxedLine+20,AttrAll);
      end;
{$endif TEST_PARTIAL_SYNTAX}
    evBroadcast :
      begin
        CCAction:=ccDontCare;
        case Event.Command of
          cmUpdate :
            Update;
          cmClearLineHighlights :
            SetLineFlagExclusive(lfHighlightRow,-1);
          cmResetDebuggerRow :
            SetLineFlagExclusive(lfDebuggerRow,-1);
          cmScrollBarChanged:
            if (Event.InfoPtr = HScrollBar) or
               (Event.InfoPtr = VScrollBar) then
              begin
                CheckScrollBar(HScrollBar, Delta.X);
                CheckScrollBar(VScrollBar, Delta.Y);
              end;
        end;
      end;
  else CCAction:=ccDontCare;
  end;
  inherited HandleEvent(Event);
  CurEvent:=OldEvent;
  case CCAction of
    ccCheck : CodeCompleteCheck;
    ccClear : ClearCodeCompleteWord;
  end;
end;

procedure TCustomCodeEditor.UpdateUndoRedo(cm : word; action : byte);
var UndoMenu : PMenuItem;
begin
  UndoMenu:=PAdvancedMenuBar(MenuBar)^.GetMenuItem(cm);
  if assigned(UndoMenu) then
    begin
      If assigned(UndoMenu^.Param) then
        DisposeStr(UndoMenu^.Param);
      if action<lastaction then
        UndoMenu^.Param:=NewStr(ActionString[action]);
    end;
end;


procedure TCustomCodeEditor.Update;
begin
  Lock;
  LimitsChanged;
  SelectionChanged;
  HighlightChanged;
  UnLock;
end;

function TCustomCodeEditor.GetLocalMenu: PMenu;
begin
  GetLocalMenu:=nil;
end;

function TCustomCodeEditor.GetCommandTarget: PView;
begin
  GetCommandTarget:=@Self;
end;

function TCustomCodeEditor.CreateLocalMenuView(var Bounds: TRect; M: PMenu): PMenuPopup;
var MV: PMenuPopup;
begin
  New(MV, Init(Bounds, M));
  CreateLocalMenuView:=MV;
end;

procedure TCustomCodeEditor.LocalMenu(P: TPoint);
var M: PMenu;
    MV: PMenuPopUp;
    R: TRect;
    Re: word;
begin
  M:=GetLocalMenu;
  if M=nil then Exit;
  if LastLocalCmd<>0 then
     M^.Default:=SearchMenuItem(M,LastLocalCmd);
  Desktop^.GetExtent(R);
  MakeGlobal(P,R.A); {Desktop^.MakeLocal(R.A,R.A);}
  MV:=CreateLocalMenuView(R,M);
  Re:=Application^.ExecView(MV);
  if M^.Default=nil then LastLocalCmd:=0
     else LastLocalCmd:=M^.Default^.Command;
  Dispose(MV, Done);
  if Re<>0 then
    Message(GetCommandTarget,evCommand,Re,@Self);
end;

function TCustomCodeEditor.GetReservedColCount: sw_integer;
var LSX: sw_integer;
begin
  if IsFlagSet(efFolds) then LSX:=GetFoldStringWidth else LSX:=0;
  GetReservedColCount:=LSX;
end;

procedure TCustomCodeEditor.Draw;
function GetEIFold(EI: PEditorLineInfo): PFold;
begin
  if Assigned(EI) then GetEIFold:=EI^.Fold else GetEIFold:=nil;
end;
var SelectColor,
    HighlightColColor,
    HighlightRowColor,
    ErrorMessageColor  : word;
    B: TDrawBuffer;
    X,Y,AX,AY,MaxX,LSX: sw_integer;
    PX: TPoint;
    LineCount: sw_integer;
    Line: PCustomLine;
    LineText,Format: string;
    isBreak : boolean;
    C: char;
    FreeFormat: array[0..MaxLineLength] of boolean;
    Color: word;
    ColorTab: array[coFirstColor..coLastColor] of word;
    ErrorLine: integer;
    ErrorMsg: string[MaxViewWidth];
function CombineColors(Orig,Modifier: byte): byte;
var Color: byte;
begin
  if (Modifier and $0f)=0 then
    Color:=(Orig and $0f) or (Modifier and $f0)
  else
    Color:=(Orig and $f0) or (Modifier and $0f);
  { do not allow invisible }
  { use white as foreground in this case }
  if (Color and $f) = ((Color div $10) and $7) then
    Color:=(Color and $F0) or $F;
  CombineColors:=Color;
end;
var
    FoldPrefix,FoldSuffix: string;
{    SkipLine: boolean;}
{    FoldStartLine: sw_integer;}
begin
  if ELockFlag>0 then
    begin
      DrawCalled:=true;
      Exit;
    end;
  DrawCalled:=false;

  ErrorMsg:=copy(GetErrorMessage,1,MaxViewWidth);
  if ErrorMsg='' then ErrorLine:=-1 else
  if (CurPos.Y-Delta.Y)<(Size.Y div 2) then ErrorLine:=Size.Y-1
     else ErrorLine:=0;
  LineCount:=GetLineCount;
  ColorTab[coTextColor]:=GetColor(1);
  ColorTab[coWhiteSpaceColor]:=GetColor(2);
  ColorTab[coCommentColor]:=GetColor(3);
  ColorTab[coReservedWordColor]:=GetColor(4);
  ColorTab[coIdentifierColor]:=GetColor(5);
  ColorTab[coStringColor]:=GetColor(6);
  ColorTab[coNumberColor]:=GetColor(7);
  ColorTab[coAssemblerColor]:=GetColor(8);
  ColorTab[coSymbolColor]:=GetColor(9);
  ColorTab[coDirectiveColor]:=GetColor(13);
  ColorTab[coHexNumberColor]:=GetColor(14);
  ColorTab[coTabColor]:=GetColor(15);
  { break same as error }
  ColorTab[coBreakColor]:=GetColor(16);
  ColorTab[coAsmReservedColor]:=GetColor(17);
  SelectColor:=GetColor(10);
  HighlightColColor:=GetColor(11);
  HighlightRowColor:=GetColor(12);
  ErrorMessageColor:=GetColor(16);
{$ifdef TEST_PARTIAL_SYNTAX}
  If (not GetSyntaxCompleted) and (GetLastSyntaxedLine<Delta.Y+Size.Y) then
    UpdateAttrsRange(GetLastSyntaxedLine,Delta.Y+Size.Y,AttrAll);
{$endif TEST_PARTIAL_SYNTAX}
  LSX:=GetReservedColCount;
  Y:=0; AY:=Delta.Y;
  for Y:=0 to Size.Y-1 do
  begin
    if Y=ErrorLine then
      begin
        MoveChar(B,' ',ErrorMessageColor,Size.X);
        MoveStr(B,ErrorMsg,ErrorMessageColor);
        WriteLine(0,Y,Size.X,1,B);
      end
    else
      begin
        AY:=ViewToEditorLine(Delta.Y+Y);
        if (0<=AY) and (AY<LineCount) then
          begin
            Line:=GetLine(AY);
            if assigned(Line) then
              begin
                IsBreak:=Line^.IsFlagSet(lfBreakpoint);
              end
            else
              begin
                IsBreak:=false;
              end;
          end
        else
          begin
            Line:=nil;
            IsBreak:=false;
          end;

        begin
          Color:=ColorTab[coTextColor];
          FillChar(FreeFormat,SizeOf(FreeFormat),1);
          MoveChar(B,' ',Color,Size.X);
          GetDisplayTextFormat(AY,LineText,Format);

      {    if FlagSet(efSyntaxHighlight) then MaxX:=length(LineText)+1
             else }MaxX:=Size.X+Delta.X;
          for X:=1 to Min(MaxX,High(LineText)) do
          begin
            AX:=Delta.X+X-1;
            if X<=length(LineText) then C:=LineText[X] else C:=' ';

            PX.X:=AX-Delta.X; PX.Y:=AY;
            if (Highlight.A.X<>Highlight.B.X) or (Highlight.A.Y<>Highlight.B.Y) then
             { there's a highlight }
              begin
                if (PointOfs(Highlight.A)<=PointOfs(PX)) and (PointOfs(PX)<PointOfs(Highlight.B)) then
                  begin
                    Color:=SelectColor;
                    FreeFormat[X]:=false;
                  end;
              end
            else
             { no highlight }
              begin
                if IsFlagSet(efVerticalBlocks) then
                  begin
                    if (SelStart.X<=AX) and (AX<=SelEnd.X) and
                       (SelStart.Y<=AY) and (AY<=SelEnd.Y) then
                      begin
                        Color:=SelectColor; FreeFormat[X]:=false;
                      end;
                  end
                else
                  if PointOfs(SelStart)<>PointOfs(SelEnd) then
                   if (PointOfs(SelStart)<=PointOfs(PX)) and (PointOfs(PX)<PointOfs(SelEnd)) then
                    begin
                      Color:=SelectColor; FreeFormat[X]:=false;
                    end;
              end; { no highlight }
            if FreeFormat[X] then
             if X<=length(Format) then
               {Color:=ColorTab[ord(Format[X])] else Color:=ColorTab[coTextColor];
                 this give BoundsCheckError with -Cr quite often PM }
               Color:=ColorTab[ord(Format[X]) mod (coLastColor + 1)] else Color:=ColorTab[coTextColor];

            if IsFlagSet(efHighlightRow) and
               (PX.Y=CurPos.Y) then
              begin
                Color:=CombineColors(Color,HighlightRowColor);
                FreeFormat[X]:=false;
              end;
            if IsFlagSet(efHighlightColumn) and (PX.X=CurPos.X) then
              begin
                Color:=CombineColors(Color,HighlightColColor);
                FreeFormat[X]:=false;
              end;

            if Assigned(Line) and Line^.IsFlagSet(lfHighlightRow) then
              begin
                Color:=CombineColors(Color,HighlightRowColor);
                FreeFormat[X]:=false;
              end;
            if isbreak then
              begin
                Color:=ColorTab[coBreakColor];
                FreeFormat[X]:=false;
              end;
            if Assigned(Line) and Line^.isFlagSet(lfDebuggerRow) then
              begin
                Color:=CombineColors(Color,HighlightRowColor);
                FreeFormat[X]:=false;
              end;

            if (0<=LSX+X-1-Delta.X) and (LSX+X-1-Delta.X<MaxViewWidth) then
              MoveChar(B[LSX+X-1-Delta.X],C,Color,1);
          end; { for X:=1 to ... }
          if IsFlagSet(efFolds) then
          begin
            GetFoldStrings(AY,FoldPrefix,FoldSuffix);
            MoveStr(B[0],FoldPrefix,ColorTab[coTextColor]);
            if FoldSuffix<>'' then
              MoveStr(B[Size.X-1-length(FoldSuffix)],FoldSuffix,ColorTab[coTextColor]);
          end;
          WriteLine(0,Y,Size.X,1,B);
        end; { if not SkipLine ... }
      end; { not errorline }
  end; { while (Y<Size.Y) ... }
  DrawCursor;
end;

procedure TCustomCodeEditor.DrawCursor;
begin
  if Elockflag>0 then
    DrawCursorCalled:=true
  else
    begin
      SetCursor(GetReservedColCount+CurPos.X-Delta.X,EditorToViewLine(CurPos.Y)-Delta.Y);
      SetState(sfCursorIns,Overwrite);
    end;
end;

procedure TCustomCodeEditor.ResetCursor;
begin
  if Elockflag>0 then
    begin
      DrawCursorCalled:=true;
      exit;
    end
  else
    inherited ResetCursor;
end;

function TCustomCodeEditor.Overwrite: boolean;
begin
  Overwrite:=not IsFlagSet(efInsertMode);
end;

procedure TCustomCodeEditor.SetCodeCompleteWord(const S: string);
begin
  if S<>'' then
    SetCompleteState(csOffering)
  else
    SetCompleteState(csInactive);
end;

procedure TCustomCodeEditor.ClearCodeCompleteWord;
begin
  SetCodeCompleteWord('');
  SetCompleteState(csInactive);
end;

function TCustomCodeEditor.GetCompleteState: TCompleteState;
begin
  { Abstract }
  GetCompleteState:=csInactive;
end;

procedure TCustomCodeEditor.SetCompleteState(AState: TCompleteState);
begin
  { Abstract }
end;

function TCustomCodeEditor.UpdateAttrs(FromLine: sw_integer; Attrs: byte): sw_integer;
begin
  Abstract;
  UpdateAttrs:=-1;
end;

function TCustomCodeEditor.UpdateAttrsRange(FromLine, ToLine: sw_integer; Attrs: byte): sw_integer;
begin
  Abstract;
  UpdateAttrsRange:=-1;
end;

procedure TCustomCodeEditor.AddAction(AAction: byte; AStartPos, AEndPos: TPoint; AText: string;AFlags : longint);
begin
  { Abstract }
end;

procedure TCustomCodeEditor.AddGroupedAction(AAction : byte);
begin
  { Abstract }
end;

procedure TCustomCodeEditor.CloseGroupedAction(AAction : byte);
begin
  { Abstract }
end;

function TCustomCodeEditor.GetUndoActionCount: sw_integer;
begin
  { Abstract }
  GetUndoActionCount:=0;
end;

function TCustomCodeEditor.GetRedoActionCount: sw_integer;
begin
  { Abstract }
  GetRedoActionCount:=0;
end;

function TCustomCodeEditor.GetMaxFoldLevel: sw_integer;
var Max,L,I: sw_integer;
begin
  Max:=0;
  for I:=0 to GetFoldCount-1 do
  begin
    L:=GetFold(I)^.GetLevel;
    if L>Max then Max:=L;
  end;
  GetMaxFoldLevel:=Max;
end;

function TCustomCodeEditor.GetFoldStringWidth: sw_integer;
begin
  GetFoldStringWidth:=GetMaxFoldLevel;
end;

procedure TCustomCodeEditor.GetFoldStrings(EditorLine: sw_integer; var Prefix, Suffix: openstring);
var F: PFold;
    C: char;
begin
  Prefix:=CharStr(' ',GetFoldStringWidth); Suffix:='';
  F:=GetLineFold(EditorLine);
  if Assigned(F) then
  begin
    if F^.Collapsed_ then C:=#27 else C:=#26;
    Prefix[1+F^.GetLevel]:=C;
    if F^.Collapsed_ then
      Suffix:='('+IntToStr(F^.GetLineCount)+')';
  end;
end;

function TCustomCodeEditor.GetFoldCount: sw_integer;
begin
  GetFoldCount:=0;
end;

function TCustomCodeEditor.GetFold(Index: sw_integer): PFold;
begin
  GetFold:=nil;
end;

procedure TCustomCodeEditor.RegisterFold(AFold: PFold);
begin
  Abstract;
end;

procedure TCustomCodeEditor.UnRegisterFold(AFold: PFold);
begin
  Abstract;
end;

procedure TCustomCodeEditor.Indent;
var S, PreS: string;
    Shift: integer;
begin
  S:=GetLineText(CurPos.Y);
  if CurPos.Y>0 then
    PreS:=RTrim(GetLineText(CurPos.Y-1),not IsFlagSet(efUseTabCharacters))
  else
    PreS:='';
  if CurPos.X>=length(PreS) then
    Shift:=GetTabSize
  else
    begin
      Shift:=1;
      while (CurPos.X+Shift<length(PreS)) and (PreS[CurPos.X+Shift]<>' ') do
       Inc(Shift);
    end;
  SetLineText(CurPos.Y,RExpand(copy(S,1,CurPos.X+1),CurPos.X+1)+CharStr(' ',Shift)+copy(S,CurPos.X+2,High(S)));
  SetCurPtr(CurPos.X+Shift,CurPos.Y);
  UpdateAttrs(CurPos.Y,attrAll);
  DrawLines(CurPos.Y);
  SetModified(true);
end;

procedure TCustomCodeEditor.CharLeft;
begin
  if CurPos.X=0 then Exit;

  SetCurPtr(CurPos.X-1,CurPos.Y);
end;

procedure TCustomCodeEditor.CharRight;
begin
  if CurPos.X>=MaxLineLength then
    Exit;
  SetCurPtr(CurPos.X+1,CurPos.Y);
end;

procedure TCustomCodeEditor.WordLeft;
var X, Y: sw_integer;
    Line: string;
    GotIt,FoundNonSeparator: boolean;
begin
  X:=CurPos.X;
  Y:=CurPos.Y;
  GotIt:=false;
  FoundNonSeparator:=false;
  while (Y>=0) do
   begin
     if Y=CurPos.Y then
      begin
   X:=length(GetDisplayText(Y));
   if CurPos.X<X then
     X:=CurPos.X; Dec(X);
   if (X=-1) then
     begin
       Dec(Y);
       if Y>=0 then
        X:=length(GetDisplayText(Y));
       Break;
     end;
      end
     else
      X:=length(GetDisplayText(Y))-1;
     Line:=GetDisplayText(Y);
     while (X>=0) and (GotIt=false) do
      begin
   if FoundNonSeparator then
    begin
      if IsWordSeparator(Line[X+1]) then
       begin
         Inc(X);
         GotIt:=true;
         Break;
       end;
    end
   else
    if not IsWordSeparator(Line[X+1]) then
     FoundNonSeparator:=true;
   Dec(X);
   if (X=0) and (IsWordSeparator(Line[1])=false) then
    begin
      GotIt:=true;
      Break;
    end;
      end;
     if GotIt then
      Break;
     X:=0;
     Dec(Y);
     if Y>=0 then
      begin
   X:=length(GetDisplayText(Y));
   Break;
      end;
   end;
  if Y<0 then Y:=0; if X<0 then X:=0;
  SetCurPtr(X,Y);
end;

procedure TCustomCodeEditor.WordRight;
var X, Y: sw_integer;
    Line: string;
    GotIt: boolean;
begin
  X:=CurPos.X; Y:=CurPos.Y; GotIt:=false;
  while (Y<GetLineCount) do
  begin
    if Y=CurPos.Y then
       begin
    X:=CurPos.X; Inc(X);
    if (X>length(GetDisplayText(Y))-1) then
       begin Inc(Y); X:=0; end;
       end else X:=0;
    Line:=GetDisplayText(Y);
    while (X<=length(Line)+1) and (GotIt=false) and (Line<>'') do
    begin
      if X=length(Line)+1 then begin GotIt:=true; Dec(X); Break end;
      if IsWordSeparator(Line[X]) then
    begin
      while (Y<GetLineCount) and
       (X<=length(Line)) and (IsWordSeparator(Line[X])) do
       begin
         Inc(X);
         if X>=length(Line) then
            begin GotIt:=true; Dec(X); Break; end;
       end;
      if (GotIt=false) and (X<length(Line)) then
      begin
        Dec(X);
        GotIt:=true;
        Break;
      end;
    end;
      Inc(X);
    end;
    if GotIt then Break;
    X:=0;
    Inc(Y);
    if (Y<GetLineCount) then
    begin
      Line:=GetDisplayText(Y);
      if (Line<>'') and (IsWordSeparator(Line[1])=false) then Break;
    end;
  end;
  if Y=GetLineCount then Y:=GetLineCount-1;
  SetCurPtr(X,Y);
end;

procedure TCustomCodeEditor.LineStart;
begin
  SetCurPtr(0,CurPos.Y);
end;

procedure TCustomCodeEditor.LineEnd;
var
  s : string;
  i : longint;
begin
  if CurPos.Y<GetLineCount then
    begin
      s:=GetDisplayText(CurPos.Y);
      i:=length(s);
      while (i>0) and (s[i]=' ') do
        dec(i);
      SetCurPtr(i,CurPos.Y);
    end
  else
    SetCurPtr(0,CurPos.Y);
end;

function TCustomCodeEditor.NextVisibleLine(StartLine: sw_integer; Down: boolean): sw_integer;
var Count,NL: sw_integer;
begin
  if Down then
    begin
      Count:=GetLineCount;
      NL:=StartLine;
      while (NL<Count-1) and not IsLineVisible(NL) do
        Inc(NL);
      if NL>=Count then
        NL:=-1;
    end
  else
    begin
      NL:=StartLine;
      while (NL>0) and not IsLineVisible(NL) do
        Dec(NL);
    end;
  if not IsLineVisible(NL) then
    NL:=-1;
  NextVisibleLine:=NL;
end;

procedure TCustomCodeEditor.LineUp;
var NL: sw_integer;
begin
  NL:=NextVisibleLine(CurPos.Y-1,false);
  if NL<>-1 then
    SetCurPtr(CurPos.X,NL);
end;

procedure TCustomCodeEditor.LineDown;
var NL: sw_integer;
begin
  NL:=NextVisibleLine(CurPos.Y+1,true);
  if NL<>-1 then
    SetCurPtr(CurPos.X,NL);
end;

procedure TCustomCodeEditor.PageUp;
var NL: sw_integer;
begin
  ScrollTo(Delta.X,Max(Delta.Y-Size.Y,0));
  NL:=Max(CurPos.Y-(Size.Y),0);
  if not IsLineVisible(NL) then
    NL:=NextVisibleLine(NL,false);
  if NL>=0 then
    SetCurPtr(CurPos.X,Max(0,NL));
end;

procedure TCustomCodeEditor.PageDown;
var NL: sw_integer;
begin
  ScrollTo(Delta.X,Min(Delta.Y+Size.Y,GetLineCount-1));
  NL:=Min(CurPos.Y+(Size.Y{-1}),GetLineCount-1);
  if not IsLineVisible(NL) then
    NL:=NextVisibleLine(NL,true);
  if NL>=0 then
    SetCurPtr(CurPos.X,Min(GetLineCount-1,NL));
end;

procedure TCustomCodeEditor.TextStart;
begin
  SetCurPtr(0,0);
end;

procedure TCustomCodeEditor.TextEnd;
var s : string;
    i : longint;
begin
  s:=GetDisplayText(GetLineCount-1);
  i:=length(s);
  while (i>0) and (s[i]=' ') do
    dec(i);
  SetCurPtr(i,GetLineCount-1);
end;

procedure TCustomCodeEditor.WindowStart;
begin
  SetCurPtr(CurPos.X,Delta.Y);
end;

procedure TCustomCodeEditor.WindowEnd;
begin
  SetCurPtr(CurPos.X,Delta.Y+Size.Y-1);
end;

procedure TCustomCodeEditor.JumpSelStart;
begin
  if ValidBlock then
    SetCurPtr(SelStart.X,SelStart.Y);
end;

procedure TCustomCodeEditor.JumpSelEnd;
begin
  if ValidBlock then
  SetCurPtr(SelEnd.X,SelEnd.Y);
end;

procedure TCustomCodeEditor.JumpMark(MarkIdx: integer);
begin
  DontConsiderShiftState:=true;
  if (MarkIdx<Low(Bookmarks)) or (MarkIdx>High(Bookmarks)) then
    begin ErrorBox(FormatStrInt(msg_invalidmarkindex,MarkIdx),nil); Exit; end;

  with Bookmarks[MarkIdx] do
  if Valid=false then
    InformationBox(FormatStrInt(msg_marknotset,MarkIdx),nil)
  else
    SetCurPtr(Pos.X,Pos.Y);
  DontConsiderShiftState:=false;
end;

procedure TCustomCodeEditor.DefineMark(MarkIdx: integer);
begin
  if (MarkIdx<Low(Bookmarks)) or (MarkIdx>High(Bookmarks)) then
    begin
      ErrorBox(FormatStrInt(msg_invalidmarkindex,MarkIdx),nil);
      Exit;
    end;
  with Bookmarks[MarkIdx] do
   begin
     Pos:=CurPos;
     Valid:=true;
   end;
end;

procedure TCustomCodeEditor.JumpToLastCursorPos;
begin
  NotImplemented;
end;

procedure TCustomCodeEditor.UpperCase;
var StartP,EndP: TPoint;
begin
  if ValidBlock=false then Exit;
  GetSelectionArea(StartP,EndP);
  AddGroupedAction(eaUpperCase);
  ChangeCaseArea(StartP,EndP,caToUpperCase);
  CloseGroupedAction(eaUpperCase);
end;

procedure TCustomCodeEditor.LowerCase;
var StartP,EndP: TPoint;
begin
  if ValidBlock=false then Exit;
  GetSelectionArea(StartP,EndP);
  AddGroupedAction(eaLowerCase);
  ChangeCaseArea(StartP,EndP,caToLowerCase);
  CloseGroupedAction(eaLowerCase);
end;

procedure TCustomCodeEditor.ToggleCase;
var StartP,EndP: TPoint;
begin
  if ValidBlock=false then Exit;
  GetSelectionArea(StartP,EndP);
  AddGroupedAction(eaToggleCase);
  ChangeCaseArea(StartP,EndP,caToggleCase);
  CloseGroupedAction(eaToggleCase);
end;

procedure TCustomCodeEditor.WordLowerCase;
var StartP,EndP: TPoint;
begin
  if GetCurrentWordArea(StartP,EndP)=false then Exit;
  AddGroupedAction(eaLowerCase);
  ChangeCaseArea(StartP,EndP,caToLowerCase);
  CloseGroupedAction(eaLowerCase);
end;

procedure TCustomCodeEditor.WordUpperCase;
var StartP,EndP: TPoint;
begin
  if GetCurrentWordArea(StartP,EndP)=false then Exit;
  AddGroupedAction(eaUpperCase);
  ChangeCaseArea(StartP,EndP,caToUpperCase);
  CloseGroupedAction(eaUpperCase);
end;

procedure TCustomCodeEditor.CreateFoldFromBlock;
var StartY,EndY: sw_integer;
begin
  if not IsFlagSet(efFolds) then Exit;
  if not ValidBlock then Exit;
  StartY:=SelStart.Y; EndY:=SelEnd.Y;
  if SelEnd.X=0 then Dec(EndY);
  if CreateFold(StartY,EndY,false)=false then
    ErrorBox(msg_foldboundsarenotvalid,nil);
end;

procedure TCustomCodeEditor.ToggleFold;
var F: PFold;
begin
  if not IsFlagSet(efFolds) then Exit;
  F:=GetLineFold(CurPos.Y);
  if Assigned(F) then
    F^.Collapse(not F^.Collapsed_);
end;

procedure TCustomCodeEditor.ExpandFold;
var F: PFold;
begin
  if not IsFlagSet(efFolds) then Exit;
  F:=GetLineFold(CurPos.Y);
  if Assigned(F) then
    F^.Collapse(false);
end;

procedure TCustomCodeEditor.CollapseFold;
var F: PFold;
begin
  if not IsFlagSet(efFolds) then Exit;
  F:=GetLineFold(CurPos.Y);
  if Assigned(F) then
    F^.Collapse(true);
end;

procedure TCustomCodeEditor.ChangeCaseArea(StartP,EndP: TPoint; CaseAction: TCaseAction);
var Y,X: sw_integer;
    X1,X2: sw_integer;
    S: string;
    C: char;
    StartPos : TPoint;
    HoldUndo : boolean;
begin
  Lock;
  HoldUndo:=GetStoreUndo;
  SetStoreUndo(false);
  for Y:=StartP.Y to EndP.Y do
  begin
    S:=GetDisplayText(Y);
    { Pierre, please implement undo here! Gabor }
    X1:=0; X2:=length(S)-1;
    if Y=StartP.Y then X1:=StartP.X;
    if Y=EndP.Y then X2:=EndP.X;
    SetStoreUndo(HoldUndo);
    StartPos.X:=X1;
    StartPos.Y:=Y;
    { the only drawback is that we keep
      the original text even if Toggle where
      it is not really necessary PM }
    Addaction(eaOverwriteText,StartPos,StartPos,Copy(S,X1+1,X2-X1+1),GetFlags);
    SetStoreUndo(false);
    for X:=X1 to X2 do
    begin
      C:=S[X+1];
      case CaseAction of
        caToLowerCase : C:=LowCase(C);
        caToUpperCase : C:=UpCase(C);
        caToggleCase  : if C in['a'..'z'] then
                          C:=Upcase(C)
                        else
                          C:=LowCase(C);
       end;
      S[X+1]:=C;
    end;
    SetDisplayText(Y,S);
  end;
  UpdateAttrsRange(StartP.Y,EndP.Y,attrAll);
  DrawLines(CurPos.Y);
  SetModified(true);
  Addaction(eaMoveCursor,StartPos,CurPos,'',GetFlags);
  SetStoreUndo(HoldUndo);
  UnLock;
end;

procedure  TCustomCodeEditor.PushInfo(Const st : string);
begin
  { Dummies }
end;

procedure  TCustomCodeEditor.PopInfo;
begin
  { Dummies }
end;


procedure TCustomCodeEditor.InsertOptions;
begin
  { Abstract }
  NotImplemented;
end;

function TCustomCodeEditor.GetLineFold(EditorLine: sw_integer): PFold;
var L: PCustomLine;
    LI: PEditorLineInfo;
    F: PFold;
begin
  F:=nil;
  if IsFlagSet(efFolds) then
  if (0<=EditorLine) and (EditorLine<GetLineCount) then
  begin
    L:=GetLine(EditorLine);
    if Assigned(L) then
      LI:=L^.GetEditorInfo(@Self)
    else
      LI:=nil;
    if Assigned(LI) then
      F:=LI^.Fold;
  end;
  GetLineFold:=F;
end;

function TCustomCodeEditor.IsLineVisible(EditorLine: sw_integer): boolean;
var V: boolean;
    F,PrevF: PFold;
    FoldHeadline: boolean;
begin
  V:=true;
  if IsFlagSet(efFolds) then
    begin
      F:=GetLineFold(EditorLine);
      if Assigned(F) then
      begin
        PrevF:=GetLineFold(EditorLine-1);
        FoldHeadline:=false;
        if (PrevF<>F) and ((PrevF=nil) or (not PrevF^.IsParent(F))) then
          FoldHeadline:=true;
        if FoldHeadline then
          begin
            if Assigned(F^.ParentFold) and (F^.ParentFold^.IsCollapsed) then
              V:=false;
          end
        else
          if F^.IsCollapsed then
            V:=false;
      end;
    end;
  IsLineVisible:=V;
end;

function TCustomCodeEditor.ViewToEditorLine(ViewLine: sw_integer): sw_integer;
var I,Line,Count: sw_integer;
begin
  if not IsFlagSet(efFolds) then
    Line:=ViewLine
  else
    begin
      Count:=GetLineCount;
      I:=0; Line:=-1;
      while (Line<ViewLine) and (I<Count) do
      begin
        if IsLineVisible(I) then
          Inc(Line);
        Inc(I);
      end;
      if Line<>ViewLine then
        Line:=-1
      else
        Line:=I-1;
    end;
  ViewToEditorLine:=Line;
end;

function TCustomCodeEditor.EditorToViewLine(EditorLine: sw_integer): sw_integer;
var I,Line: sw_integer;
begin
  if not IsFlagSet(efFolds) then
    Line:=EditorLine
  else
    begin
      Line:=-1;
      for I:=0 to EditorLine do
        if IsLineVisible(I) then
          Inc(Line);
    end;
  EditorToViewLine:=Line;
end;

procedure TCustomCodeEditor.ViewToEditorPoint(P: TPoint; var NP: TPoint);
begin
  NP.X:=P.X-GetReservedColCount;
  NP.Y:=ViewToEditorLine(P.Y);
end;

procedure TCustomCodeEditor.EditorToViewPoint(P: TPoint; var NP: TPoint);
begin
  NP.X:=P.X+GetReservedColCount;
  NP.Y:=EditorToViewLine(P.Y);
end;

procedure TCustomCodeEditor.FindMatchingDelimiter(ScanForward: boolean);
const OpenSymbols  : string[6] = '[{(<''"';
      CloseSymbols : string[6] = ']})>''"';
var SymIdx: integer;
    LineText,LineAttr: string;
    CurChar: char;
    X,Y: sw_integer;
    LineCount: sw_integer;
    JumpPos: TPoint;
    BracketLevel: integer;
begin
  JumpPos.X:=-1; JumpPos.Y:=-1;
  LineText:=GetDisplayText(CurPos.Y);
  LineText:=copy(LineText,CurPos.X+1,1);
  if LineText='' then Exit;
  CurChar:=LineText[1];
  Y:=CurPos.Y; X:=CurPos.X; LineCount:=0;
  BracketLevel:=1;
  if ScanForward then
    begin
      SymIdx:=Pos(CurChar,OpenSymbols);
      if SymIdx=0 then Exit;
      repeat
        Inc(LineCount);
        GetDisplayTextFormat(Y,LineText,LineAttr);
        if LineCount<>1 then X:=-1;
        repeat
          Inc(X);
          if X<length(LineText) then
           if copy(LineAttr,X+1,1)<>chr(attrComment) then
             if (LineText[X+1]=CloseSymbols[SymIdx]) and (BracketLevel=1) then
               begin
                 JumpPos.X:=X; JumpPos.Y:=Y;
               end
             else
               if LineText[X+1]=OpenSymbols[SymIdx] then
                 Inc(BracketLevel)
               else
               if LineText[X+1]=CloseSymbols[SymIdx] then
                 if BracketLevel>1 then
                   Dec(BracketLevel);
        until (X>=length(LineText)) or (JumpPos.X<>-1);
        Inc(Y);
      until (Y>=GetLineCount) or (JumpPos.X<>-1);
    end
  else
    begin
      SymIdx:=Pos(CurChar,CloseSymbols);
      if SymIdx=0 then Exit;
      repeat
        Inc(LineCount);
        GetDisplayTextFormat(Y,LineText,LineAttr);
        if LineCount<>1 then X:=length(LineText);
        repeat
          Dec(X);
          if X>0 then
           if copy(LineAttr,X+1,1)<>chr(attrComment) then
             if (LineText[X+1]=OpenSymbols[SymIdx]) and (BracketLevel=1) then
               begin
                 JumpPos.X:=X; JumpPos.Y:=Y;
               end
             else
               if LineText[X+1]=CloseSymbols[SymIdx] then
                 Inc(BracketLevel)
               else
               if LineText[X+1]=OpenSymbols[SymIdx] then
                 if BracketLevel>1 then
                   Dec(BracketLevel);
        until (X<0) or (JumpPos.X<>-1);
        Dec(Y);
      until (Y<0) or (JumpPos.X<>-1);
    end;
  if JumpPos.X<>-1 then
  begin
    SetCurPtr(JumpPos.X,JumpPos.Y);
    TrackCursor(do_centre);
  end;
end;

function TCustomCodeEditor.InsertNewLine: Sw_integer;
var i,Ind: Sw_integer;
    S,IndentStr: string;
procedure CalcIndent(LineOver: Sw_integer);
begin
  if (LineOver<0) or (LineOver>GetLineCount) or ((GetFlags and efNoIndent)<>0) then
    Ind:=0 else
  begin
    repeat
      IndentStr:=GetDisplayText(LineOver);
      Dec(LineOver);
    until (LineOver<0) or (IndentStr<>'');
    Ind:=0;
    while (Ind<length(IndentStr)) and (IndentStr[Ind+1]=' ') do
     Inc(Ind);
  end;
  IndentStr:=CharStr(' ',Ind);
end;
var {SelBack: sw_integer;}
    SCP: TPoint;
    CI : sw_integer;
    HoldUndo : Boolean;
    L,NewL: PCustomLine;
    EI,NewEI: PEditorLineInfo;
begin
  if IsReadOnly then begin InsertNewLine:=-1; Exit; end;
  Lock;
  SCP:=CurPos;
  HoldUndo:=GetStoreUndo;
  SetStoreUndo(false);
  if CurPos.Y<GetLineCount then S:=GetLineText(CurPos.Y) else S:='';
  if Overwrite=false then
  begin
    if CurPos.Y<GetLineCount then
      begin
        L:=GetLine(CurPos.Y);
        if not assigned(L) then
          EI:=nil
        else
          EI:=L^.GetEditorInfo(@Self);
      end
    else
      EI:=nil;
{    SelBack:=0;}
    CI:=LinePosToCharIdx(CurPos.Y,CurPos.X);
    if GetLineCount>0 then
    begin
      S:=GetLineText(CurPos.Y);
{      SelBack:=length(S)-SelEnd.X;}
      SetLineText(CurPos.Y,RTrim(S,not IsFlagSet(efUseTabCharacters)));
    end;
    SetLineText(CurPos.Y,copy(S,1,CI-1));
    CalcIndent(CurPos.Y);
    S:=copy(S,CI,High(S));
    i:=1;
    while (i<=length(s)) and (i<=length(IndentStr)) and (s[i]=' ') do
      inc(i);
    if i>1 then
      Delete(IndentStr,1,i-1);
    NewL:=InsertLine(CurPos.Y+1,IndentStr+S);
    LimitsChanged;
(*    if PointOfs(SelStart)<>PointOfs(SelEnd) then { !!! check it - it's buggy !!! }
      begin SelEnd.Y:=CurPos.Y+1; SelEnd.X:=length(GetLineText(CurPos.Y+1))-SelBack; end;*)
    UpdateAttrs(CurPos.Y,attrAll);
    SetCurPtr(Ind,CurPos.Y+1);
    NewEI:=NewL^.GetEditorInfo(@Self);
    if Assigned(EI) and Assigned(NewEI) then
    begin
      NewEI^.SetFold(EI^.Fold);
      if Assigned(EI^.Fold) then
        if EI^.Fold^.IsCollapsed then
          EI^.Fold^.Collapse(false);
    end;
     SetStoreUndo(HoldUndo);
     { obsolete IndentStr is taken care of by the Flags PM }
     Addaction(eaInsertLine,SCP,CurPos,CharStr(' ',i-1){IndentStr},GetFlags);
     SetStoreUndo(false);
     AdjustSelectionPos(SCP.X,SCP.Y,CurPos.X-SCP.X,CurPos.Y-SCP.Y);
  end else
  begin
    CalcIndent(CurPos.Y);
    if CurPos.Y=GetLineCount-1 then
    begin
      AddLine(IndentStr);
      AdjustSelectionBefore(0,1);
      LimitsChanged;
      SetStoreUndo(HoldUndo);
      UpdateAttrs(CurPos.Y,attrAll);
      SetCurPtr(Ind,CurPos.Y+1);
      { obsolete IndentStr is taken care of by the Flags PM }
      Addaction(eaInsertLine,SCP,CurPos,''{IndentStr},GetFlags);
      SetStoreUndo(false);
    end
    else
    begin
      UpdateAttrs(CurPos.Y,attrAll);
      SetStoreUndo(HoldUndo);
      SetCurPtr(Ind,CurPos.Y+1);
      AddAction(eaMoveCursor,SCP,CurPos,'',GetFlags);
      SetStoreUndo(false);
    end;
  end;
  DrawLines(CurPos.Y);
  SetStoreUndo(HoldUndo);
  SetModified(true);
  Unlock;
end;

procedure TCustomCodeEditor.BreakLine;
begin
  NotImplemented; Exit;
end;

procedure TCustomCodeEditor.BackSpace;
var S,PreS: string;
    OI,CI,CP,Y,TX: Sw_integer;
    SCP,SC1 : TPoint;
    HoldUndo : Boolean;
begin
  if IsReadOnly then Exit;
  Lock;
  SCP:=CurPos;
  HoldUndo:=GetStoreUndo;
  SetStoreUndo(false);
  if CurPos.X=0 then
   begin
     if CurPos.Y>0 then
      begin
        CI:=Length(GetDisplayText(CurPos.Y-1));
        S:=GetLineText(CurPos.Y-1);
        SetLineText(CurPos.Y-1,S+GetLineText(CurPos.Y));
        SC1.X:=Length(S);SC1.Y:=CurPOS.Y-1;
        SetStoreUndo(HoldUndo);
        AddAction(eaDeleteLine,SCP,SC1,GetLineText(CurPos.Y),GetFlags);
        SetStoreUndo(false);
        DeleteLine(CurPos.Y);
        LimitsChanged;
        SetCurPtr(CI,CurPos.Y-1);
        AdjustSelectionPos(Ci,CurPos.Y,CurPos.X-SCP.X,CurPos.Y-SCP.Y);
      end;
   end
  else
   begin
     CP:=CurPos.X-1;
     S:=GetLineText(CurPos.Y);
     CI:=LinePosToCharIdx(CurPos.Y,CP);
     if (s[ci]=TAB) and (CharIdxToLinePos(Curpos.y,ci)=cp) then
      CP:=CharIdxToLinePos(CurPos.Y,CI-1)+1;
     if IsFlagSet(efBackspaceUnindents) then
      begin
        S:=GetDisplayText(CurPos.Y);
        if Trim(copy(S,1,CP+1))='' then
         begin
           Y:=CurPos.Y;
           while (Y>0) do
            begin
              Dec(Y);
              PreS:=GetDisplayText(Y);
              if Trim(copy(PreS,1,CP+1))<>'' then Break;
            end;
           if Y<0 then PreS:='';
           TX:=0;
           while (TX<length(PreS)) and (PreS[TX+1]=' ') do
            Inc(TX);
           if TX<CP then CP:=TX;
         end;
      end;
     S:=GetLineText(CurPos.Y);
     OI:=LinePosToCharIdx(CurPos.Y,CurPos.X);
     CI:=LinePosToCharIdx(CurPos.Y,CP);
     SetLineText(CurPos.Y,copy(S,1,CI-1)+copy(S,OI,High(S)));
     SetCurPtr(CP,CurPos.Y);
     SetStoreUndo(HoldUndo);
     Addaction(eaDeleteText,SCP,CurPos,Copy(S,CI,OI-CI),GetFlags);
     SetStoreUndo(false);
     AdjustSelectionPos(SCP.X-1,SCP.Y,CurPos.X-SCP.X,CurPos.Y-SCP.Y);
   end;
  UpdateAttrs(CurPos.Y,attrAll);
  DrawLines(CurPos.Y);
  SetStoreUndo(HoldUndo);
  SetModified(true);
  Unlock;
end;

procedure TCustomCodeEditor.DelChar;
var S: string;
    SDX,SDY,CI : sw_integer;
    HoldUndo : boolean;
    SCP : TPoint;
begin
  if IsReadOnly then Exit;
  Lock;
  HoldUndo:=GetStoreUndo;
  SetStoreUndo(false);
  S:=GetLineText(CurPos.Y);
  CI:=LinePosToCharIdx(CurPos.Y,CurPos.X);
  if (CI>length(S)) or (S='') then
   begin
     if CurPos.Y<GetLineCount-1 then
      begin
        SetLineText(CurPos.Y,S+CharStr(' ',CurPOS.X-Length(S))+GetLineText(CurPos.Y+1));
        SDX:=CurPos.X;
        SetStoreUndo(HoldUndo);
        SCP.X:=0;SCP.Y:=CurPos.Y+1;
        AddGroupedAction(eaDelChar);
        AddAction(eaMoveCursor,CurPos,SCP,'',GetFlags);
        S:=GetLineText(CurPos.Y+1);
        AddAction(eaDeleteLine,SCP,CurPos,S,GetFlags);
        CloseGroupedAction(eaDelChar);
        SetStoreUndo(false);
        DeleteLine(CurPos.Y+1);
        LimitsChanged;
        SDY:=-1;
        SetCurPtr(CurPos.X,CurPos.Y);
        UpdateAttrs(CurPos.Y,attrAll);
        AdjustSelectionPos(CurPos.X,CurPos.Y,SDX,SDY);
      end;
   end
  else
   begin
     SCP:=CurPos;
     { Problem if S[CurPos.X+1]=TAB !! PM }
     if S[CI]=TAB then
       begin
         { we want to remove the tab if we are at the first place
           of the tab, but the following test was true for the last position
           in tab
         if CharIdxToLinePos(Curpos.y,ci)=Curpos.x then }
         if CharIdxToLinePos(Curpos.y,ci-1)=Curpos.x-1 then
            Delete(S,Ci,1)
         else
          S:=Copy(S,1,CI-1)+CharStr(' ',GetTabSize-1)+Copy(S,CI+1,High(S));
         SetStoreUndo(HoldUndo);
         Addaction(eaDeleteText,CurPos,CurPos,#9,GetFlags);
         SDX:=-1;
         SetStoreUndo(false);
       end
     else
       begin
         SetStoreUndo(HoldUndo);
         Addaction(eaDeleteText,CurPos,CurPos,S[CI],GetFlags);
         SetStoreUndo(false);
         SDX:=-1;
         Delete(S,CI,1);
       end;
     SetLineText(CurPos.Y,S);
     SDY:=0;
     SetCurPtr(CurPos.X,CurPos.Y);
     UpdateAttrs(CurPos.Y,attrAll);
     AdjustSelectionPos(SCP.X,SCP.Y,SDX,SDY);
   end;
  DrawLines(CurPos.Y);
  SetStoreUndo(HoldUndo);
  SetModified(true);
  Unlock;
end;

procedure TCustomCodeEditor.DelWord;
var
  SP,EP : TPoint;
  SelSize : sw_integer;
begin
  if IsReadOnly then Exit;
  Lock;
  SP:=SelStart;
  EP:=SelEnd;
  SetSelection(SelStart,SelStart);
  SelectWord;
  SelSize:=SelEnd.X-SelStart.X;
  DelSelect;
  SetSelection(SP,EP);
  AdjustSelectionPos(CurPos.X,CurPos.Y,-SelSize,0);
  if SelSize>0 then
    SetModified(true);
  Unlock;
end;

procedure TCustomCodeEditor.DelToEndOfWord;
var
  SP,EP : TPoint;
  S : String;
  SelSize : sw_integer;
begin
  if IsReadOnly then Exit;
  Lock;
  SP:=SelStart;
  EP:=SelEnd;
  SetSelection(SelStart,SelStart);
  SelectWord;
  S:=GetDisplayText(CurPos.Y);
  if ((SelStart.X=SelEnd.X) and (SelStart.Y=SelEnd.Y)) then
    begin
      if (Length(S) <= CurPos.X) then
        begin
          SetSelection(SP,EP);
          DelChar;
          Unlock;
          exit;
        end
      else
        begin
          SelEnd.X:=CurPos.X+1;
          SelEnd.Y:=CurPos.Y;
        end;
    end;
  while (length(S)>= SelEnd.X+1) and
        ((S[SelEnd.X+1]=' ') or (S[SelEnd.X+1]=TAB))  do
    inc(SelEnd.X);
  SetSelection(CurPos,SelEnd);
  SelSize:=SelEnd.X-SelStart.X;
  DelSelect;
  SetSelection(SP,EP);
  AdjustSelectionPos(CurPos.X,CurPos.Y,-SelSize,0);
  if SelSize>0 then
    SetModified(true);
  Unlock;
end;

procedure TCustomCodeEditor.DelStart;
var S: string;
    OI: Sw_integer;
    HoldUndo : Boolean;
    SCP : TPoint;
begin
  if IsReadOnly then Exit;
  Lock;
  HoldUndo:=GetStoreUndo;
  SetStoreUndo(false);
  SCP:=CurPos;
  S:=GetLineText(CurPos.Y);
  if (S<>'') and (CurPos.X<>0) then
  begin
    OI:=LinePosToCharIdx(CurPos.Y,CurPos.X);
    SetLineText(CurPos.Y,copy(S,OI,High(S)));
    SetCurPtr(0,CurPos.Y);
    SetStoreUndo(HoldUndo);
    Addaction(eaDeleteText,SCP,CurPos,copy(S,1,OI-1),GetFlags);
    SetStoreUndo(false);
    AdjustSelectionPos(CurPos.X,CurPos.Y,-length(copy(S,1,OI-1)),0);
    UpdateAttrs(CurPos.Y,attrAll);
    DrawLines(CurPos.Y);
    SetModified(true);
  end;
  SetStoreUndo(HoldUndo);
  Unlock;
end;

procedure TCustomCodeEditor.DelEnd;
var S: string;
    OI: Sw_integer;
    HoldUndo : Boolean;
    SCP : TPoint;
begin
  if IsReadOnly then Exit;
  Lock;
  HoldUndo:=GetStoreUndo;
  SetStoreUndo(false);
  SCP:=CurPos;
  S:=GetLineText(CurPos.Y);
  if (S<>'') and (CurPos.X<>length(S)) then
  begin
    OI:=LinePosToCharIdx(CurPos.Y,CurPos.X);
    SetLineText(CurPos.Y,copy(S,1,OI-1));
    SetCurPtr(CurPos.X,CurPos.Y);
    SetStoreUndo(HoldUndo);
    Addaction(eaDeleteText,SCP,CurPos,copy(S,OI,High(S)),GetFlags);
    SetStoreUndo(false);
    AdjustSelectionPos(CurPos.X+1,CurPos.Y,-length(copy(S,OI,High(S)))+1,0);
    UpdateAttrs(CurPos.Y,attrAll);
    DrawLines(CurPos.Y);
    SetModified(true);
  end;
  SetStoreUndo(HoldUndo);
  Unlock;
end;

procedure TCustomCodeEditor.DelLine;
var
  HoldUndo : boolean;
  SP : TPoint;
  S : String;
begin
  if IsReadOnly then Exit;
  Lock;
  if GetLineCount>0 then
  begin
    SP:=CurPos;
    S:=GetLineText(CurPos.Y);
    HoldUndo:=GetStoreUndo;
    SetStoreUndo(false);
    DeleteLine(CurPos.Y);
    LimitsChanged;
    AdjustSelectionBefore(0,-1);
    SetCurPtr(0,CurPos.Y);
    UpdateAttrs(Max(0,CurPos.Y-1),attrAll);
    DrawLines(CurPos.Y);
    SetStoreUndo(HoldUndo);
    AddAction(eaDeleteLine,SP,CurPos,S,GetFlags);
    SetModified(true);
  end;
  Unlock;
end;

procedure TCustomCodeEditor.InsMode;
begin
  SetInsertMode(Overwrite);
end;

function TCustomCodeEditor.GetCurrentWordArea(var StartP,EndP: TPoint): boolean;
const WordChars = ['A'..'Z','a'..'z','0'..'9','_'];
var P : TPoint;
    S : String;
    StartPos,EndPos : byte;
    OK: boolean;
begin
  P:=CurPos;
  S:=GetLineText(P.Y);
  StartPos:=P.X+1;
  EndPos:=StartPos;
  OK:=(S[StartPos] in WordChars);
  if OK then
    begin
       While (StartPos>0) and (S[StartPos-1] in WordChars) do
         Dec(StartPos);
       While (EndPos<Length(S)) and (S[EndPos+1] in WordChars) do
         Inc(EndPos);
       StartP.X:=StartPos-1; StartP.Y:=CurPos.Y;
       EndP.X:=EndPos-1; EndP.Y:=CurPos.Y;
    end;
  GetCurrentWordArea:=OK;
end;

function  TCustomCodeEditor.GetCurrentWord : string;
var S: string;
    StartP,EndP: TPoint;
begin
  if GetCurrentWordArea(StartP,EndP)=false then
    S:=''
  else
    begin
      S:=GetLineText(StartP.Y);
      S:=copy(S,StartP.X+1,EndP.X-StartP.X+1);
    end;
  GetCurrentWord:=S;
end;

procedure TCustomCodeEditor.StartSelect;
var P1,P2: TPoint;
begin
  if ValidBlock=false then
    begin
{      SetSelection(SelStart,Limit);}
      P1:=CurPos; P1.X:=0; P2:=CurPos; {P2.X:=length(GetLineText(P2.Y))+1;}
      SetSelection(P1,P2);
    end
  else
    SetSelection(CurPos,SelEnd);
  if PointOfs(SelEnd)<PointOfs(SelStart) then
     SetSelection(SelStart,SelStart);
  CheckSels;
  DrawView;
end;

procedure TCustomCodeEditor.EndSelect;
var P: TPoint;
    LS: sw_integer;
begin
  P:=CurPos;
{  P.X:=Min(SelEnd.X,length(GetLineText(SelEnd.Y)));}
  LS:=length(GetLineText(SelEnd.Y));
  if LS<P.X then P.X:=LS;
  CheckSels;
  SetSelection(SelStart,P);
  DrawView;
end;

procedure TCustomCodeEditor.DelSelect;
var LineDelta, LineCount, CurLine: Sw_integer;
    StartX,EndX,LastX: Sw_integer;
    S: string;
    SPos : TPoint;
begin
  if IsReadOnly or (ValidBlock=false) then Exit;

  Lock;
  AddGroupedAction(eaDelBlock);
  LineCount:=(SelEnd.Y-SelStart.Y)+1;
  LineDelta:=0; LastX:=CurPos.X;
  CurLine:=SelStart.Y;
  { single line : easy }
  if LineCount=1 then
    begin
      S:=GetDisplayText(CurLine);
      StartX:=SelStart.X;
      EndX:=SelEnd.X;
      SetDisplayText(CurLine,RExpand(copy(S,1,StartX),StartX)
        +copy(S,EndX+1,High(S)));
      if GetStoreUndo then
        begin
          SPos.X:=StartX;
          SPos.Y:=CurLine;
          AddAction(eaDeleteText,SPos,SPos,Copy(S,StartX+1,EndX-StartX),GetFlags);
        end;
      Inc(CurLine);
      LastX:=SelStart.X;
    end
  { several lines : a bit less easy }
  else
    begin
      S:=GetDisplayText(CurLine);
      StartX:=SelStart.X;
      EndX:=SelEnd.X;
      SetDisplayText(CurLine,RExpand(copy(S,1,StartX),StartX)
        +copy(GetDisplayText(CurLine+LineCount-1),EndX+1,High(S)));
      if GetStoreUndo then
        begin
          SPos.X:=StartX;
          SPos.Y:=CurLine;
          AddAction(eaDeleteText,SPos,SPos,Copy(S,StartX+1,High(S)),GetFlags);
          S:=GetDisplayText(CurLine+LineCount-1);
        end;
      Inc(CurLine);
      Inc(LineDelta);
      LastX:=SelStart.X;
      while (LineDelta<LineCount) do
        begin
        { delete the complete line }
          DeleteLine(CurLine);
          Inc(LineDelta);
        end;
      if GetStoreUndo then
        begin
          AddAction(eaInsertText,SPos,SPos,Copy(S,EndX+1,High(S)),GetFlags);
        end;
    end;
  HideSelect;
  SetCurPtr(LastX,CurLine-1);
  UpdateAttrs(CurPos.Y,attrAll);
  DrawLines(CurPos.Y);
  SetModified(true);
  CloseGroupedAction(eaDelBlock);
  UnLock;
end;

procedure TCustomCodeEditor.HideSelect;
begin
  SetSelection(CurPos,CurPos);
  DrawLines(Delta.Y);
end;

procedure TCustomCodeEditor.CopyBlock;
var Temp: PCodeEditor;
    R: TRect;
begin
  if IsReadOnly or (ValidBlock=false) then Exit;

  Lock;
  GetExtent(R);
  AddGroupedAction(eaCopyBlock);
  New(Temp, Init(R, nil, nil, nil,nil));
  Temp^.InsertFrom(@Self);
(*  Temp^.SelectAll(true);
  { this selects one line too much because
    we have a empty line at creation to avoid
    negative line problems so we need to decrease SelEnd.Y }
  Dec(Temp^.SelEnd.Y);*)


  InsertFrom(Temp);
  Dispose(Temp, Done);
  CloseGroupedAction(eaCopyBlock);
  UnLock;
end;

procedure TCustomCodeEditor.MoveBlock;
var Temp: PCodeEditor;
    R: TRect;
    OldPos: TPoint;
begin
  if IsReadOnly then Exit;
  if (SelStart.X=SelEnd.X) and (SelStart.Y=SelEnd.Y) then Exit;
  Lock;
  AddGroupedAction(eaMoveBlock);
  GetExtent(R);
  New(Temp, Init(R, nil, nil, nil,nil));
  Temp^.InsertFrom(@Self);
  OldPos:=CurPos;
  if CurPos.Y>SelStart.Y then
    Dec(OldPos.Y,Temp^.GetLineCount-1);
  DelSelect;
  SetCurPtr(OldPos.X,OldPos.Y);
  InsertFrom(Temp);
  Dispose(Temp, Done);
  CloseGroupedAction(eaMoveBlock);
  UnLock;
end;

procedure TCustomCodeEditor.IndentBlock;
var
  ey,i{,indlen} : Sw_integer;
  S,Ind : String;
  Pos : Tpoint;
begin
  if IsReadOnly then Exit;
  if (SelStart.X=SelEnd.X) and (SelStart.Y=SelEnd.Y) then Exit;
  Lock;
  AddGroupedAction(eaIndentBlock);
  ey:=selend.y;
  if selend.x=0 then
   dec(ey);
  S:='';
  { If AutoIndent try to align first line to
    last line before selection }
  { DISABLED created problems PM
  if IsFlagSet(efAutoIndent) and (SelStart.Y>0) then
    begin
      i:=SelStart.Y-1;
      while (S='') and (i>=0) do
        begin
          S:=GetDisplayText(i);
          dec(i);
        end;
      if (S='') or (S[1]<>' ') then
        Ind:=' '
      else
        begin
          i:=1;
          while (i<=Length(S)) and (S[i]=' ') do
           inc(i);
          indlen:=i;
          S:=GetDisplayText(SelStart.Y);
          i:=1;
          while (i<=Length(S)) and (S[i]=' ') do
            inc(i);
          indlen:=indlen-i;
          if indlen<=0 then
            indlen:=1;
          Ind:=CharStr(' ',indlen);
        end;
    end
  else
   Ind:=' ';}
  Ind:=CharStr(' ',GetIndentSize);
  for i:=selstart.y to ey do
   begin
     S:=GetLineText(i);
     SetLineText(i,Ind+S);
     Pos.X:=0;Pos.Y:=i;
     AddAction(eaInsertText,Pos,Pos,Ind,GetFlags);
   end;
  SetCurPtr(CurPos.X,CurPos.Y);
  { must be added manually here PM }
  AddAction(eaMoveCursor,Pos,CurPos,'',GetFlags);
  UpdateAttrsRange(SelStart.Y,SelEnd.Y,attrAll);
  DrawLines(CurPos.Y);
  SetModified(true);
  CloseGroupedAction(eaIndentBlock);
  UnLock;
end;

procedure TCustomCodeEditor.UnindentBlock;
var
  ey,i,j,k,indlen : Sw_integer;
  S : String;
  Pos : TPoint;
begin
  if IsReadOnly then Exit;
  if (SelStart.X=SelEnd.X) and (SelStart.Y=SelEnd.Y) then Exit;
  Lock;
  AddGroupedAction(eaUnindentBlock);
  ey:=selend.y;
  if selend.x=0 then
   dec(ey);
  { If AutoIndent try to align first line to
    last line before selection }
  { Disabled created problems
  if IsFlagSet(efAutoIndent) and (SelStart.Y>0) then
    begin
      S:=GetDisplayText(SelStart.Y);
      i:=1;
      while (i<=Length(S)) and (S[i]=' ') do
        inc(i);
      indlen:=i-1;
      i:=SelStart.Y-1;
      S:='';
      while (S='') and (i>=0) do
        begin
          if Trim(Copy(GetDisplayText(i),1,indlen))='' then
            S:=''
          else
            S:=GetDisplayText(i);
          dec(i);
        end;
      if (S='') then
        Indlen:=1
      else
        begin
          i:=1;
          while (i<=Length(S)) and (S[i]=' ') do
           inc(i);
          indlen:=indlen-i+1;
          if indlen<=0 then
            indlen:=1;
        end;
    end
  else
   Indlen:=1;}
  Indlen:=GetIndentSize;
  for i:=selstart.y to ey do
   begin
     S:=GetLineText(i);
     k:=0;
     for j:=1 to indlen do
       if (length(s)>1) and (S[1]=' ') then
         begin
           Delete(s,1,1);
           inc(k);
         end;
     SetLineText(i,S);
     if k>0 then
       begin
         Pos.Y:=i;
         Pos.X:=0;
         AddAction(eaDeleteText,Pos,Pos,CharStr(' ',k),GetFlags);
       end;
   end;
  SetCurPtr(CurPos.X,CurPos.Y);
  UpdateAttrsRange(SelStart.Y,SelEnd.Y,attrAll);
  DrawLines(CurPos.Y);
  SetModified(true);
  CloseGroupedAction(eaUnindentBlock);
  UnLock;
end;

procedure TCustomCodeEditor.SelectWord;
const WordChars = ['A'..'Z','a'..'z','0'..'9','_'];
var S : String;
    StartPos,EndPos : byte;
    A,B: TPoint;
begin
  A:=CurPos;
  B:=CurPos;
  S:=GetDisplayText(A.Y);
  StartPos:=A.X+1;
  EndPos:=StartPos;
  if not (S[StartPos] in WordChars) then
    exit
  else
    begin
       While (StartPos>0) and (S[StartPos-1] in WordChars) do
         Dec(StartPos);
       While (EndPos<Length(S)) and (S[EndPos+1] in WordChars) do
         Inc(EndPos);
       A.X:=StartPos-1;
       B.X:=EndPos;
       SetSelection(A,B);
    end;
end;

procedure TCustomCodeEditor.SelectLine;
var A,B: TPoint;
begin
  if CurPos.Y<GetLineCount then
    begin
      A.Y:=CurPos.Y; A.X:=0;
      B.Y:=CurPos.Y+1; B.X:=0;
      SetSelection(A,B);
    end;
end;

procedure TCustomCodeEditor.WriteBlock;
var FileName: string;
    S: PBufStream;
begin
  if ValidBlock=false then Exit;

  FileName:='';
  if EditorDialog(edWriteBlock, @FileName) <> cmCancel then
  begin
    FileName := FExpand(FileName);

    New(S, Init(FileName, stCreate, 4096));
    if (S=nil) or (S^.Status<>stOK) then
      EditorDialog(edCreateError,@FileName)
    else
      if SaveAreaToStream(S,SelStart,SelEnd)=false then
        EditorDialog(edWriteError,@FileName);
    if Assigned(S) then Dispose(S, Done);
  end;
end;

procedure TCustomCodeEditor.ReadBlock;
var FileName: string;
    S: PFastBufStream;
    E: PCodeEditor;
    R: TRect;
begin
  if IsReadOnly then Exit;
  FileName:='';
  if EditorDialog(edReadBlock, @FileName) <> cmCancel then
  begin
    FileName := FExpand(FileName);

    New(S, Init(FileName, stOpenRead, 4096));
    if (S=nil) or (S^.Status<>stOK) then
      EditorDialog(edReadError,@FileName)
    else
      begin
        R.Assign(0,0,0,0);
        New(E, Init(R,nil,nil,nil,nil));
        AddGroupedAction(eaReadBlock);
        if E^.LoadFromStream(S)=false then
          EditorDialog(edReadError,@FileName)
        else
          begin
            E^.SelectAll(true);
            Self.InsertFrom(E);
          end;
        CloseGroupedAction(eaReadBlock);
        Dispose(E, Done);
      end;
    if Assigned(S) then Dispose(S, Done);
  end;
end;

procedure TCustomCodeEditor.PrintBlock;
begin
  NotImplemented; Exit;
end;

function TCustomCodeEditor.SelectCodeTemplate(var ShortCut: string): boolean;
begin
  { Abstract }
  SelectCodeTemplate:=false;
end;

procedure TCustomCodeEditor.ExpandCodeTemplate;
var Line,ShortCutInEditor,ShortCut: string;
    X,Y,I,LineIndent: sw_integer;
    CodeLines: PUnsortedStringCollection;
    CanJump: boolean;
    CP: TPoint;
begin
  {
    The usage of editing primitives in this routine make it pretty slow, but
    its speed is still acceptable and they make the implementation of Undo
    much easier... - Gabor
  }
  if IsReadOnly then Exit;

  Lock;

  CP.X:=-1; CP.Y:=-1;
  Line:=GetDisplayText(CurPos.Y);
  X:=CurPos.X; ShortCut:='';
  if X<=length(Line) then
  while (X>0) and (Line[X] in (NumberChars+AlphaChars)) do
  begin
    ShortCut:=Line[X]+ShortCut;
    Dec(X);
  end;
  ShortCutInEditor:=ShortCut;

  New(CodeLines, Init(10,10));
  if (ShortCut='') or (not TranslateCodeTemplate(ShortCut,CodeLines)) then
   if SelectCodeTemplate(ShortCut) then
     TranslateCodeTemplate(ShortCut,CodeLines);

  if CodeLines^.Count>0 then
  begin
    LineIndent:=X;
    SetCurPtr(X,CurPos.Y);
    if Copy(ShortCut,1,length(ShortCutInEditor))=ShortCutInEditor then
      begin
        for I:=1 to length(ShortCutInEditor) do
          DelChar;
      end
    else
      { restore correct position }
      SetCurPtr(X+Length(ShortCutInEditor),CurPos.Y);
    for Y:=0 to CodeLines^.Count-1 do
    begin
      Line:=GetStr(CodeLines^.At(Y));
      CanJump:=false;
      if (Y>0) then
        begin
           CanJump:=Trim(GetLineText(CurPos.Y))='';
           if CanJump=false then
             begin
(*                 for X:=1 to LineIndent do  { indent template lines to align }
                 AddChar(' ');            { them to the first line         }*)
               InsertText(CharStr(' ',LineIndent));
             end
           else
            SetCurPtr(CurPos.X+LineIndent,CurPos.Y);
        end;
      I:=Pos(CodeTemplateCursorChar,Line);
      if I>0 then
        begin
          Delete(Line,I,1);
          CP.X:=CurPos.X+I-1;
          CP.Y:=CurPos.Y;
        end;
      InsertText(Line);
      if Y<CodeLines^.Count-1 then
        begin
          InsertNewLine;               { line break }
          if CanJump=false then
            begin
              while CurPos.X>0 do       { unindent }
              begin
                SetCurPtr(CurPos.X-1,CurPos.Y);
                DelChar;
              end;
            end
          else
            SetCurPtr(0,CurPos.Y);
        end;
    end;
  end;
  Dispose(CodeLines, Done);

  if (CP.X<>-1) and (CP.Y<>-1) then
    SetCurPtr(CP.X,CP.Y);

  UnLock;
end;

procedure TCustomCodeEditor.AddChar(C: char);
const OpenBrackets  : string[10] = '[({';
      CloseBrackets : string[10] = '])}';
var S,SC,TabS: string;
    BI: byte;
    CI,TabStart,LocTabSize : Sw_integer;
    SP: TPoint;
    HoldUndo : boolean;
begin
  if IsReadOnly then Exit;

  Lock;
  SP:=CurPos;
  HoldUndo:=GetStoreUndo;
  SetStoreUndo(false);
  if (C<>TAB) or IsFlagSet(efUseTabCharacters) then
    SC:=C
  else
    begin
      LocTabSize:=GetTabSize - (CurPos.X mod GetTabSize);
      if (CurPos.Y<=1) or not IsFlagSet(efAutoIndent) then
        SC:=CharStr(' ',LocTabSize)
      else
        begin
          S:=GetLineText(CurPos.Y-1);
          BI:=CurPos.X+1;
          while (BI<=Length(S)) and (S[BI]=' ') do
            inc(BI);
          if (BI=CurPos.X+1) or (BI>Length(S)) then
            SC:=CharStr(' ',LocTabSize)
          else
            SC:=CharStr(' ',BI-CurPos.X-1);
        end;
    end;
  S:=GetLineText(CurPos.Y);
  if CharIdxToLinePos(CurPos.Y,length(S))<CurPos.X then
    begin
      S:=S+CharStr(' ',CurPos.X-CharIdxToLinePos(CurPos.Y,length(S)){-1});
      SetLineText(CurPos.Y,S);
    end;
  CI:=LinePosToCharIdx(CurPos.Y,CurPos.X);
  if CI>High(S) then
    begin
      Unlock;
      exit;
    end;
  if (CI>0) and (S[CI]=TAB) and not IsFlagSet(efUseTabCharacters) then
    begin
      if CI=1 then
        TabStart:=0
      else
        TabStart:=CharIdxToLinePos(CurPos.Y,CI-1)+1;
      if SC=Tab then TabS:=Tab else
        TabS:=CharStr(' ',CurPos.X-TabStart);
      SetLineText(CurPos.Y,copy(S,1,CI-1)+TabS+SC+copy(S,CI+1,High(S)));
      SetCurPtr(CharIdxToLinePos(CurPos.Y,CI+length(TabS)+length(SC)),CurPos.Y);
    end
  else
    begin
      if Overwrite and (CI<=length(S)) then
        begin
          SetLineText(CurPos.Y,copy(S,1,CI-1)+SC+copy(S,CI+length(SC),High(S)));
        end
      else
        SetLineText(CurPos.Y,copy(S,1,CI-1)+SC+copy(S,CI,High(S)));
      SetCurPtr(CharIdxToLinePos(CurPos.Y,CI+length(SC)),CurPos.Y);
    end;
 { must be before CloseBrackets !! }
  SetStoreUndo(HoldUndo);
  if Overwrite then
    Addaction(eaOverwriteText,SP,CurPos,Copy(S,CI,length(SC)),GetFlags)
  else
    Addaction(eaInsertText,SP,CurPos,SC,GetFlags);
  SetStoreUndo(false);
  if IsFlagSet(efAutoBrackets) then
    begin
      BI:=Pos(C,OpenBrackets);
      if (BI>0) then
        begin
          SetStoreUndo(HoldUndo);
          AddChar(CloseBrackets[BI]);
          SetStoreUndo(false);
          SetCurPtr(CurPos.X-1,CurPos.Y);
        end;
    end;
  UpdateAttrs(CurPos.Y,attrAll);
  if GetInsertMode then
    AdjustSelection(CurPos.X-SP.X,CurPos.Y-SP.Y);
  DrawLines(CurPos.Y);
  SetStoreUndo(HoldUndo);
  SetModified(true);
  UnLock;
end;

{$ifdef WinClipSupported}

const
   linelimit = 200;

function TCustomCodeEditor.ClipPasteWin: Boolean;
var
    StorePos : TPoint;
    first : boolean;

procedure InsertStringWrap(const s: string; var i : Longint);
var
    BPos,EPos: TPoint;
begin
  if first then
    begin
      { we need to cut the line in two
      if not at end of line PM }
      InsertNewLine;
      SetCurPtr(StorePos.X,StorePos.Y);
      InsertText(s);
      first:=false;
    end
  else
    begin
      Inc(i);
      InsertLine(i,s);
      BPos.X:=0;BPos.Y:=i;
      EPOS.X:=Length(s);EPos.Y:=i;
      AddAction(eaInsertLine,BPos,EPos,GetDisplayText(i),GetFlags);
    end;
end;

var
    OK: boolean;
    l,i,len,len10 : longint;
    p,p10,p2,p13 : pchar;
    s : string;
begin
  Lock;
  OK:=WinClipboardSupported;
  if OK then
    begin

      first:=true;
      StorePos:=CurPos;
      i:=CurPos.Y;
      l:=GetTextWinClipboardSize;
      if l=0 then
        OK:=false
      else
        OK:=GetTextWinClipBoardData(p,l);
      if OK then
        begin
          if l>500 then
            PushInfo(msg_readingwinclipboard);
          AddGroupedAction(eaPasteWin);
          p2:=p;
          len:=strlen(p2);
          // issue lines ((#13)#10 terminated) of maximally "linelimit" chars.
          // does not take initial X position into account
          repeat
            p13:=strpos(p2,#13);
            p10:=strpos(p2,#10);
            if len> linelimit then
              len:=linelimit;
            if assigned(p10) then
              begin
               len10:=p10-p2;
               if len10<len then
                 begin
                   if p13+1=p10 then
                     dec(len10);
                   len:=len10;
                 end
               else
                 p10:=nil;  // signal no cleanup
              end;
            setlength(s,len);
            if len>0 then
              move(p2^,s[1],len);
            // cleanup
            if assigned(p10) then
              p2:=p10+1
            else
              inc(p2,len);
            insertstringwrap(s,i);
            len:=strlen(p2);
          until len=0;

          SetCurPtr(StorePos.X,StorePos.Y);  // y+i to get after paste?
          SetModified(true);
          UpdateAttrs(StorePos.Y,attrAll);
          CloseGroupedAction(eaPasteWin);
          Update;
          if l>500 then
            PopInfo;
          { we must free the allocated memory }
          freemem(p,l);
          DrawView;
        end;
    end;
  ClipPasteWin:=OK;
  UnLock;
end;

function TCustomCodeEditor.ClipCopyWin: Boolean;
var OK,ShowInfo: boolean;
    p,p2 : pchar;
    s : string;
    i,str_begin,str_end,NumLines,PcLength : longint;
begin
  NumLines:=SelEnd.Y-SelStart.Y;
  if (NumLines>0) or (SelEnd.X>SelStart.X) then
    Inc(NumLines);
  if NumLines=0 then
    exit;
  Lock;
  ShowInfo:=SelEnd.Y-SelStart.Y>50;
  if ShowInfo then
    PushInfo(msg_copyingwinclipboard);
  { First calculate needed size }
  { for newlines first + 1 for terminal #0 }
  PcLength:=Length(EOL)*(NumLines-1)+1;

  { overestimated but can not be that big PM }
  for i:=SelStart.Y to SelEnd.Y do
    PCLength:=PCLength+Length(GetLineText(i));
  getmem(p,PCLength);
  i:=SelStart.Y;
  s:=GetLineText(i);
  str_begin:=LinePosToCharIdx(i,SelStart.X);
  if SelEnd.Y>SelStart.Y then
    str_end:=High(S)
  else
    str_end:=LinePosToCharIdx(i,SelEnd.X)-1;
  s:=copy(s,str_begin,str_end-str_begin+1);
  strpcopy(p,s);
  p2:=strend(p);
  inc(i);
  while i<SelEnd.Y do
    begin
      strpcopy(p2,EOL+GetLineText(i));
      p2:=strend(p2);
      Inc(i);
    end;
  if SelEnd.Y>SelStart.Y then
    begin
      s:=copy(GetLineText(i),1,LinePosToCharIdx(i,SelEnd.X)-1);
      strpcopy(p2,EOL+s);
    end;
  OK:=WinClipboardSupported;
  if OK then
    begin
      OK:=SetTextWinClipBoardData(p,strlen(p));
    end;
  ClipCopyWin:=OK;
  if ShowInfo then
    PopInfo;
  Freemem(p,PCLength);
  UnLock;
end;
{$endif WinClipSupported}

function TCustomCodeEditor.ClipCopy: Boolean;

var ShowInfo,CanPaste: boolean;

begin
  Lock;
  {AddGroupedAction(eaCopy);
   can we undo a copy ??
   maybe as an Undo Paste in Clipboard !! }
  clipcopy:=false;
  showinfo:=false;
  if (clipboard<>nil) and (clipboard<>@self) then
    begin
      ShowInfo:=SelEnd.Y-SelStart.Y>50;
      if ShowInfo then
        PushInfo(msg_copyingclipboard);
      clipcopy:=Clipboard^.InsertFrom(@Self);
      if ShowInfo then
        PopInfo;
      {Enable paste command.}
      CanPaste:=((Clipboard^.SelStart.X<>Clipboard^.SelEnd.X) or
                (Clipboard^.SelStart.Y<>Clipboard^.SelEnd.Y));
      SetCmdState(FromClipCmds,CanPaste);
    end;
  UnLock;
end;

procedure TCustomCodeEditor.ClipCut;
var
  ShowInfo,CanPaste : boolean;
begin
  if IsReadOnly then Exit;
  Lock;
  AddGroupedAction(eaCut);
  DontConsiderShiftState:=true;
  if (clipboard<>nil) and (clipboard<>@self) then
   begin
     ShowInfo:=SelEnd.Y-SelStart.Y>50;
     if ShowInfo then
       PushInfo(msg_cutting);
     if Clipboard^.InsertFrom(@Self) then
      begin
        if not IsClipBoard then
         DelSelect;
        SetModified(true);
      end;
     if ShowInfo then
       PopInfo;
     CanPaste:=((Clipboard^.SelStart.X<>Clipboard^.SelEnd.X) or
               (Clipboard^.SelStart.Y<>Clipboard^.SelEnd.Y));
     SetCmdState(FromClipCmds,CanPaste);
   end;
  CloseGroupedAction(eaCut);
  UnLock;
  DontConsiderShiftState:=false;
end;

procedure TCustomCodeEditor.ClipPaste;
var
  ShowInfo : boolean;
begin
  if IsReadOnly then Exit;
  DontConsiderShiftState:=true;
  Lock;
  AddGroupedAction(eaPaste);
  if Clipboard<>nil then
   begin
     ShowInfo:=Clipboard^.SelEnd.Y-Clipboard^.SelStart.Y>50;
     if ShowInfo then
       PushInfo(msg_pastingclipboard);
     InsertFrom(Clipboard);
     if ShowInfo then
       PopInfo;
   end;
  CloseGroupedAction(eaPaste);
  UnLock;
  DontConsiderShiftState:=false;
end;

procedure TCustomCodeEditor.Undo;
begin
  NotImplemented; Exit;
end;

procedure TCustomCodeEditor.Redo;
begin
  NotImplemented; Exit;
end;

procedure TCustomCodeEditor.GotoLine;
var
  GotoRec: TGotoLineDialogRec;
begin
  with GotoRec do
  begin
    LineNo:='1';
    Lines:=GetLineCount;
    {Linecount can be 0, but in that case there still is a cursor blinking in top
     of the window, which will become line 1 as soon as sometype hits a key.}
    if lines=0 then
      lines:=1;
    if EditorDialog(edGotoLine, @GotoRec) <> cmCancel then
    begin
      Lock;
      SetCurPtr(0,StrToInt(LineNo)-1);
      TrackCursor(do_centre);
      UnLock;
    end;
  end;
end;

procedure TCustomCodeEditor.Find;
var
  FindRec: TFindDialogRec;
  DoConf: boolean;
begin
  with FindRec do
  begin
    Find := FindStr;
    if GetCurrentWord<>'' then
      Find:=GetCurrentWord;
{$ifdef TEST_REGEXP}
    Options := ((FindFlags and ffmOptionsFind) shr ffsOptions) or
               ((FindFlags and ffUseRegExp) shr ffsUseRegExpFind);
{$else not TEST_REGEXP}
    Options := (FindFlags and ffmOptions) shr ffsOptions;
{$endif TEST_REGEXP}
    Direction := (FindFlags and ffmDirection) shr ffsDirection;
    Scope := (FindFlags and ffmScope) shr ffsScope;
    Origin := (FindFlags and ffmOrigin) shr ffsOrigin;
    DoConf:= (FindFlags and ffPromptOnReplace)<>0;
    FindReplaceEditor:=@self;
    if EditorDialog(edFind, @FindRec) <> cmCancel then
    begin
      FindStr := Find;
{$ifdef TEST_REGEXP}
      FindFlags := ((Options and ffmOptionsFind) shl ffsOptions) or (Direction shl ffsDirection) or
         ((Options and ffmUseRegExpFind) shl ffsUseRegExpFind) or
         (Scope shl ffsScope) or (Origin shl ffsOrigin);
{$else : not TEST_REGEXP}
      FindFlags := ((Options and ffmOptions) shl ffsOptions) or (Direction shl ffsDirection) or
         (Scope shl ffsScope) or (Origin shl ffsOrigin);
{$endif TEST_REGEXP}
      FindFlags := FindFlags and not ffDoReplace;
      if DoConf then
        FindFlags := (FindFlags or ffPromptOnReplace);
      SearchRunCount:=0;
      if FindStr<>'' then
        DoSearchReplace
      else
        EditorDialog(edSearchFailed,nil);
    end;
    FindReplaceEditor:=nil;
  end;
end;

procedure TCustomCodeEditor.Replace;
var
  ReplaceRec: TReplaceDialogRec;
  Re: word;
begin
  if IsReadOnly then Exit;
  with ReplaceRec do
  begin
    Find := FindStr;
    if GetCurrentWord<>'' then
      Find:=GetCurrentWord;
    Replace := ReplaceStr;
{$ifdef TEST_REGEXP}
    Options := (FindFlags and ffmOptions) shr ffsOptions or
               (FindFlags and ffUseRegExp) shr ffsUseRegExpReplace;
{$else not TEST_REGEXP}
    Options := (FindFlags and ffmOptions) shr ffsOptions;
{$endif TEST_REGEXP}
    Direction := (FindFlags and ffmDirection) shr ffsDirection;
    Scope := (FindFlags and ffmScope) shr ffsScope;
    Origin := (FindFlags and ffmOrigin) shr ffsOrigin;
    FindReplaceEditor:=@self;
    Re:=EditorDialog(edReplace, @ReplaceRec);
    FindReplaceEditor:=nil;
    if Re <> cmCancel then
    begin
      FindStr := Find;
      ReplaceStr := Replace;
      FindFlags := (Options shl ffsOptions) or (Direction shl ffsDirection) or
{$ifdef TEST_REGEXP}
         ((Options and ffmUseRegExpReplace) shl ffsUseRegExpReplace) or
{$endif TEST_REGEXP}
         (Scope shl ffsScope) or (Origin shl ffsOrigin);
      FindFlags := FindFlags or ffDoReplace;
      if Re = cmYes then
        FindFlags := FindFlags or ffReplaceAll;
      SearchRunCount:=0;
      if FindStr<>'' then
        DoSearchReplace
      else
        EditorDialog(edSearchFailed,nil);
    end;
  end;
end;

procedure TCustomCodeEditor.DoSearchReplace;
var S: string;
    DX,DY,P,Y,X: sw_integer;
    Count: sw_integer;
    Found,CanExit: boolean;
    SForward,DoReplace,DoReplaceAll: boolean;
{$ifdef TEST_REGEXP}
    UseRegExp : boolean;
    RegExpEngine : TRegExprEngine;
    RegExpFlags : tregexprflags;
    regexpindex,regexplen : longint;
    findstrpchar : pchar;
{$endif TEST_REGEXP}
    LeftOK,RightOK: boolean;
    FoundCount: sw_integer;
    A,B: TPoint;
    AreaStart,AreaEnd: TPoint;
    CanReplace,Confirm: boolean;
    Re: word;
    IFindStr : string;
    BT : BTable;
    Overwriting : boolean;

  function ContainsText(const SubS:string;var S: string; Start: Sw_integer): Sw_integer;
  var
    P: Sw_Integer;
  begin
    if Start<=0 then
     P:=0
    else
     begin
       if SForward then
        begin
          if Start>length(s) then
           P:=0
          else if FindFlags and ffCaseSensitive<>0 then
           P:=BMFScan(S[Start],length(s)+1-Start,FindStr,Bt)+1
          else
           P:=BMFIScan(S[Start],length(s)+1-Start,IFindStr,Bt)+1;
          if P>0 then
           Inc(P,Start-1);
        end
       else
        begin
          if start>length(s) then
           start:=length(s);
          if FindFlags and ffCaseSensitive<>0 then
           P:=BMBScan(S[1],Start,FindStr,Bt)+1
          else
           P:=BMBIScan(S[1],Start,IFindStr,Bt)+1;
        end;
     end;
    ContainsText:=P;
  end;

  function InArea(X,Y: sw_integer): boolean;
  begin
    InArea:=((AreaStart.Y=Y) and (AreaStart.X<=X)) or
       ((AreaStart.Y<Y) and (Y<AreaEnd.Y)) or
       ((AreaEnd.Y=Y) and (X<=AreaEnd.X));
  end;
var CurDY: sw_integer;
begin
  if FindStr='' then
    begin
      Find;
      { Find will call DoFindReplace at end again
        so we need to exit directly now PM }
      exit;
    end;
  Inc(SearchRunCount);

  SForward:=(FindFlags and ffmDirection)=ffForward;
  DoReplace:=(FindFlags and ffDoReplace)<>0;
  Confirm:=(FindFlags and ffPromptOnReplace)<>0;
  DoReplaceAll:=(FindFlags and ffReplaceAll)<>0;
{$ifdef TEST_REGEXP}
  UseRegExp:=(FindFlags and ffUseRegExp)<>0;
  if UseRegExp then
    begin
      if FindFlags and ffCaseSensitive<>0 then
        RegExpFlags:=[ref_caseinsensitive]
      else
        RegExpFlags:=[];
      getmem(findstrpchar,length(findstr)+1);
      strpcopy(findstrpchar,findstr);
      RegExpEngine:=GenerateRegExprEngine(findstrpchar,RegExpFlags);
      strdispose(findstrpchar);
    end;
{$endif TEST_REGEXP}
  Count:=GetLineCount;
  FoundCount:=0;
  { Empty file ? }
  if Count=0 then
   begin
     EditorDialog(edSearchFailed,nil);
     exit;
   end;

  if SForward then
    DY:=1
  else
    DY:=-1;
  DX:=DY;

  if FindStr<>'' then
    PushInfo('Looking for "'+FindStr+'"');
  if (FindFlags and ffmScope)=ffGlobal then
   begin
     AreaStart.X:=0;
     AreaStart.Y:=0;
     AreaEnd.X:=length(GetDisplayText(Count-1));
     AreaEnd.Y:=Count-1;
   end
  else
   begin
     AreaStart:=SelStart;
     AreaEnd:=SelEnd;
   end;

  { set a y value being inside the areal }
  Y:=Min(CurPos.Y,Count-1);

  if sForward then
    X:=CurPos.X-1
  else
    { if you change this, pleas check that repeated backward searching for single chars still works
      and that data is still found if searching starts outside the current line }
    X:=Min(CurPos.X,length(GetDisplayText(Y)));

  if SearchRunCount=1 then
    if (FindFlags and ffmOrigin)=ffEntireScope then
      if SForward then
        begin
          X:=AreaStart.X-1;
          Y:=AreaStart.Y;
        end
       else
        begin
          X:=AreaEnd.X+1;
          Y:=AreaEnd.Y;
        end;

  if FindFlags and ffCaseSensitive<>0 then
   begin
     if SForward then
      BMFMakeTable(FindStr,bt)
     else
      BMBMakeTable(FindStr,bt);
   end
  else
   begin
     IFindStr:=upcase(FindStr);
     if SForward then
      BMFMakeTable(IFindStr,bt)
     else
      BMBMakeTable(IFindStr,bt);
   end;

  inc(X,DX);
  CanExit:=false;
  if not DoReplace or (not Confirm and (Owner<>nil)) then
    Owner^.Lock;
  if InArea(X,Y) then
    repeat
      CurDY:=DY;
      S:=GetDisplayText(Y);
      if X>length(S)-1 then
        X:=length(S)-1;
{$ifdef TEST_REGEXP}
      if UseRegExp then
         begin
           getmem(findstrpchar,length(Copy(S,X+1,high(S)))+1);
           strpcopy(findstrpchar,Copy(S,X+1,high(S)));
           { If start of line is required do check other positions PM }
           if (FindStr[1]='^') and (X<>0) then
             Found:=false
           else
             Found:=RegExprPos(RegExpEngine,findstrpchar,regexpindex,regexplen);
           strdispose(findstrpchar);
           P:=regexpindex+X+1;
         end
      else
{$endif TEST_REGEXP}
        begin
          P:=ContainsText(FindStr,S,X+1);
          Found:=P<>0;
        end;
      if Found then
        begin
          A.X:=P-1;
          A.Y:=Y;
          B.Y:=Y;
{$ifdef TEST_REGEXP}
          if UseRegExp then
            B.X:=A.X+regexplen
          else
{$endif TEST_REGEXP}
            B.X:=A.X+length(FindStr);
        end;
      Found:=Found and InArea(A.X,A.Y);

      if Found and ((FindFlags and ffWholeWordsOnly)<>0) then
       begin
         LeftOK:=(A.X<=0) or (not( (S[A.X] in AlphaChars+NumberChars) ));
         RightOK:=(B.X>=length(S)) or (not( (S[B.X+1] in AlphaChars+NumberChars) ));
         Found:=LeftOK and RightOK;
         if not Found then
           begin
             CurDY:=0;
             If SForward then
               begin
                 X:=B.X+1;
                 if X>length(S) then
                   CurDY:=DY;
               end
             else
               begin
                 X:=A.X-1;
                 if X<0 then
                   CurDY:=DY;
               end;
           end;
       end;

      if Found then
        begin
          Inc(FoundCount);
          Lock;
          if SForward then
           SetCurPtr(B.X,B.Y)
          else
           SetCurPtr(A.X,A.Y);
          TrackCursor(do_centre);
          SetHighlight(A,B);
          UnLock;
          CurDY:=0;
          if not DoReplace then
            begin
              CanExit:=true;
              If SForward then
                begin
                  X:=B.X;
                  Y:=B.Y;
                end
              else
                begin
                  X:=A.X;
                  Y:=A.Y;
                end;
            end
          else
            begin
              if not confirm then
                CanReplace:=true
              else
                begin
                  Re:=EditorDialog(edReplacePrompt,@CurPos);
                  case Re of
                    cmYes :
                      CanReplace:=true;
                    cmNo :
                      CanReplace:=false;
                    else {cmCancel}
                      begin
                        CanReplace:=false;
                        CanExit:=true;
                      end;
                  end;
                end;
              if CanReplace then
                begin
                  Lock;
                  { don't use SetInsertMode here because it changes the cursor shape }
                  overwriting:=(GetFlags and efInsertMode)=0;
                  SetFlags(GetFlags or efInsertMode);
                  SetSelection(A,B);
                  DelSelect;
                  InsertText(ReplaceStr);
                  if SForward then
                    begin
                      X:=CurPos.X;
                      Y:=CurPos.Y;
                    end
                  else
                    begin
                      X:=A.X;
                      Y:=A.Y;
                    end;
                  if overwriting then
                    SetFlags(GetFlags and (not efInsertMode));
                  UnLock;
                end
              else
                begin
                  If SForward then
                    begin
                      X:=B.X;
                      Y:=B.Y;
                    end
                  else
                    begin
                      X:=A.X;
                      Y:=A.Y;
                    end;
                end;
              if (DoReplaceAll=false) then
                CanExit:=true;
            end;
        end;

      if (CanExit=false) and (CurDY<>0) then
        begin
          inc(Y,CurDY);
          if SForward then
            X:=0
          else
            X:=254;
          CanExit:=((Y>=Count) and sForward) or (Y<0);
        end;
      if not CanExit then
        CanExit:=(not InArea(X,Y)) and sForward;
    until CanExit;
  if (FoundCount=0) or (DoReplace) then
    SetHighlight(CurPos,CurPos);
  if (DoReplace=false) or ((Confirm=false) and (Owner<>nil)) then
    Owner^.UnLock;
  {if (DoReplace=false) or (Confirm=false) then
    UnLock;}
  if (FoundCount=0) then
    EditorDialog(edSearchFailed,nil);
  if FindStr<>'' then
    PopInfo;
{$ifdef TEST_REGEXP}
  if UseRegExp then
    DestroyRegExprEngine(RegExpEngine);
{$endif TEST_REGEXP}
  if (FindFlags and ffmScope)=ffSelectedText then
    { restore selection PM }
    begin
      SetSelection(AreaStart,AreaEnd);
    end;
end;

function TCustomCodeEditor.GetInsertMode: boolean;
begin
  GetInsertMode:=(GetFlags and efInsertMode)<>0;
end;

procedure TCustomCodeEditor.SetInsertMode(InsertMode: boolean);
begin
  if InsertMode then
    SetFlags(GetFlags or efInsertMode)
  else
    SetFlags(GetFlags and (not efInsertMode));
  DrawCursor;
end;

{ there is a problem with ShiftDel here
  because GetShitState tells to extend the
  selection which gives wrong results (PM) }

function TCustomCodeEditor.ShouldExtend: boolean;
var ShiftInEvent: boolean;
begin
  ShiftInEvent:=false;
  if Assigned(CurEvent) then
    if CurEvent^.What=evKeyDown then
      ShiftInEvent:=((CurEvent^.KeyShift and kbShift)<>0);
  ShouldExtend:=ShiftInEvent and
    not DontConsiderShiftState;
end;

procedure TCustomCodeEditor.SetCurPtr(X,Y: sw_integer);
var OldPos{,OldSEnd,OldSStart}: TPoint;
    Extended: boolean;
    F: PFold;
begin
  Lock;
  X:=Max(0,Min(MaxLineLength+1,X));
  Y:=Max(0,Min(GetLineCount-1,Y));
  OldPos:=CurPos;
{  OldSEnd:=SelEnd;
  OldSStart:=SelStart;}
  CurPos.X:=X;
  CurPos.Y:=Y;
  TrackCursor(do_not_centre);
  if not IsLineVisible(CurPos.Y) then
  begin
    F:=GetLineFold(CurPos.Y);
    if Assigned(F) then
      F^.Collapse(false);
  end;
  if not NoSelect and ShouldExtend then
    begin
      CheckSels;
      Extended:=false;
      if PointOfs(OldPos)=PointOfs(SelStart) then
        begin
          SetSelection(CurPos,SelEnd);
          Extended:=true;
        end;
      CheckSels;
      if Extended=false then
       if PointOfs(OldPos)=PointOfs(SelEnd) then
         begin
           if not ValidBlock then
             SetSelection(CurPos,CurPos);
           SetSelection(SelStart,CurPos); Extended:=true;
         end;
      CheckSels;
      if not Extended then
         if PointOfs(OldPos)<=PointOfs(CurPos) then
           begin
             SetSelection(OldPos,CurPos);
             Extended:=true;
           end
         else
           begin
             SetSelection(CurPos,OldPos);
             Extended:=true;
           end;
      DrawView;
    end
  else if not IsFlagSet(efPersistentBlocks) then
      begin
        HideSelect;
        DrawView;
      end;
{  if PointOfs(SelStart)=PointOfs(SelEnd) then
     SetSelection(CurPos,CurPos);}
  if (GetFlags and (efHighlightColumn+efHighlightRow))<>0 then
     DrawView;
  if ((CurPos.X<>OldPos.X) or (CurPos.Y<>OldPos.Y)) and
     ((Highlight.A.X<>HighLight.B.X) or (Highlight.A.Y<>HighLight.B.Y)) then
     HideHighlight;
  if (OldPos.Y<>CurPos.Y) and (0<=OldPos.Y) and (OldPos.Y<GetLineCount) then
     SetLineText(OldPos.Y,RTrim(GetLineText(OldPos.Y),not IsFlagSet(efUseTabCharacters)));
  if ((CurPos.X<>OldPos.X) or (CurPos.Y<>OldPos.Y)) and (GetErrorMessage<>'') then
    SetErrorMessage('');
{  if ((CurPos.X<>OldPos.X) or (CurPos.Y<>OldPos.Y)) and (HighlightRow<>-1) then
    SetHighlightRow(-1);}
  if ((CurPos.X<>OldPos.X) or (CurPos.Y<>OldPos.Y)) then
    AddAction(eaMoveCursor,OldPos,CurPos,'',GetFlags);
  if ((CurPos.X<>OldPos.X) or (CurPos.Y<>OldPos.Y)) then
    PositionChanged;{UpdateIndicator;}
  UnLock;
end;

procedure TCustomCodeEditor.CheckSels;
begin
  if (SelStart.Y>SelEnd.Y) or
     ( (SelStart.Y=SelEnd.Y) and (SelStart.X>SelEnd.X) ) then
       SetSelection(SelEnd,SelStart);
end;

procedure TCustomCodeEditor.CodeCompleteApply;
var S: string;
    FragLen,
    I: integer;
begin
  Lock;

  { here should be some kind or "mark" or "break" inserted in the Undo
    information, so activating it "undoes" only the completition first and
    doesn't delete the complete word at once... - Gabor }

  FragLen:=Length(GetCodeCompleteFrag);
  S:=GetCodeCompleteWord;
  for I:=FragLen+1 to length(S) do
    AddChar(S[I]);

  UnLock;
  SetCompleteState(csInactive);
end;

procedure TCustomCodeEditor.CodeCompleteCancel;
begin
  SetCompleteState(csDenied);
end;

procedure TCustomCodeEditor.CodeCompleteCheck;
var Line: string;
    X: sw_integer;
    CurWord,NewWord: string;
begin
  SetCodeCompleteFrag('');
  if (not IsFlagSet(efCodeComplete)) or (IsReadOnly=true) then Exit;

  Lock;

  Line:=GetDisplayText(CurPos.Y);
  X:=CurPos.X; CurWord:='';
  if X<=length(Line) then
  while (X>0) and (Line[X] in (NumberChars+AlphaChars)) do
  begin
    CurWord:=Line[X]+CurWord;
    Dec(X);
  end;

  if (length(CurWord)>=CodeCompleteMinLen) and CompleteCodeWord(CurWord,NewWord) then
    begin
      SetCodeCompleteFrag(CurWord);
      SetCodeCompleteWord(NewWord);
    end
  else
    ClearCodeCompleteWord;

  UnLock;
end;

function TCustomCodeEditor.GetCodeCompleteFrag: string;
begin
  { Abstract }
  GetCodeCompleteFrag:='';
end;

procedure TCustomCodeEditor.SetCodeCompleteFrag(const S: string);
begin
  { Abstract }
end;

procedure TCustomCodeEditor.DrawLines(FirstLine: sw_integer);
begin
  if FirstLine>=(Delta.Y+Size.Y) then Exit; { falls outside of the screen }
  DrawView;
end;

procedure TCustomCodeEditor.HideHighlight;
begin
  SetHighlight(CurPos,CurPos);
end;

procedure TCustomCodeEditor.GetSelectionArea(var StartP,EndP: TPoint);
begin
  StartP:=SelStart; EndP:=SelEnd;
  if EndP.X=0 then
    begin
      Dec(EndP.Y);
      EndP.X:=length(GetDisplayText(EndP.Y))-1;
    end
  else
   Dec(EndP.X);
end;

function TCustomCodeEditor.ValidBlock: boolean;
begin
  ValidBlock:=(SelStart.X<>SelEnd.X) or (SelStart.Y<>SelEnd.Y);
end;

procedure TCustomCodeEditor.SetSelection(A, B: TPoint);
var WV: boolean;
    OS,OE: TPoint;
begin
  WV:=ValidBlock;
  OS:=SelStart; OE:=SelEnd;
  SelStart:=A; SelEnd:=B;
  if (WV=false) and (ValidBlock=false) then { do nothing } else
    if (OS.X<>SelStart.X) or (OS.Y<>SelStart.Y) or
       (OE.X<>SelEnd.X) or (OE.Y<>SelEnd.Y) then
     SelectionChanged;
end;

procedure TCustomCodeEditor.SetHighlight(A, B: TPoint);
begin
  Highlight.A:=A; Highlight.B:=B;
  HighlightChanged;
end;

{procedure TCustomCodeEditor.SetHighlightRow(Row: sw_integer);
begin
  HighlightRow:=Row;
  DrawView;
end;}

{procedure TCodeEditor.SetDebuggerRow(Row: sw_integer);
begin
  DebuggerRow:=Row;
  DrawView;
end;}

procedure TCustomCodeEditor.SelectAll(Enable: boolean);
var A,B: TPoint;
begin
  if (Enable=false) or (GetLineCount=0) then
     begin A:=CurPos; B:=CurPos end
  else
     begin
       A.X:=0; A.Y:=0;
{       B.Y:=GetLineCount-1;
       B.X:=length(GetLineText(B.Y));}
       B.Y:=GetLineCount; B.X:=0;
     end;
  SetSelection(A,B);
  DrawView;
end;

procedure TCustomCodeEditor.SelectionChanged;
var Enable,CanPaste: boolean;
begin
  if GetLineCount=0 then
    begin
      SelStart.X:=0; SelStart.Y:=0; SelEnd:=SelStart;
    end
  else
    if SelEnd.Y>GetLineCount-1 then
     if (SelEnd.Y<>GetLineCount) or (SelEnd.X<>0) then
      begin
        SelEnd.Y:=GetLineCount-1;
        SelEnd.X:=length(GetDisplayText(SelEnd.Y));
      end;

  { we change the CurCommandSet, but only if we are top view }
  if ((State and sfFocused)<>0) then
    begin
      Enable:=((SelStart.X<>SelEnd.X) or (SelStart.Y<>SelEnd.Y)) and (Clipboard<>nil);
      SetCmdState(ToClipCmds,Enable and (Clipboard<>@Self));
      SetCmdState(NulClipCmds,Enable);
      CanPaste:=(Clipboard<>nil) and ((Clipboard^.SelStart.X<>Clipboard^.SelEnd.X) or
           (Clipboard^.SelStart.Y<>Clipboard^.SelEnd.Y));
      SetCmdState(FromClipCmds,CanPaste  and (Clipboard<>@Self));
      SetCmdState(UndoCmd,(GetUndoActionCount>0));
      SetCmdState(RedoCmd,(GetRedoActionCount>0));
      Message(Application,evBroadcast,cmCommandSetChanged,nil);
    end;
  DrawView;
end;

procedure TCustomCodeEditor.HighlightChanged;
begin
  DrawView;
end;

procedure TCustomCodeEditor.SetState(AState: Word; Enable: Boolean);
  procedure ShowSBar(SBar: PScrollBar);
  begin
    if Assigned(SBar) and (SBar^.GetState(sfVisible)=false) then
        SBar^.Show;
  end;
begin
  inherited SetState(AState,Enable);

  if AlwaysShowScrollBars then
   begin
     ShowSBar(HScrollBar);
     ShowSBar(VScrollBar);
   end;

  if (AState and (sfActive+sfSelected+sfFocused))<>0 then
    begin
      SelectionChanged;
      if ((State and sfFocused)=0) and (GetCompleteState=csOffering) then
        ClearCodeCompleteWord;
    end;
end;

function TCustomCodeEditor.GetPalette: PPalette;
const P: string[length(CEditor)] = CEditor;
begin
  GetPalette:=@P;
end;

function TCustomCodeEditorCore.LoadFromStream(Editor: PCustomCodeEditor; Stream: PFastBufStream): boolean;
var S: string;
    AllLinesComplete,LineComplete,hasCR,OK: boolean;
begin
  DeleteAllLines;
  ChangedLine:=-1;
  AllLinesComplete:=true;
  OK:=(Stream^.Status=stOK);
  if eofstream(Stream) then
   AddLine('')
  else
   begin
     while OK and (eofstream(Stream)=false) and (GetLineCount<MaxLineCount) do
       begin
         if not UseOldBufStreamMethod then
           Stream^.Readline(S,LineComplete,hasCR)
         else
           ReadlnFromStream(Stream,S,LineComplete,hasCR);
         AllLinesComplete:=AllLinesComplete and LineComplete;
         OK:=OK and (Stream^.Status=stOK);
         if OK then AddLine(S);
         if not LineComplete and (ChangedLine=-1) then
           ChangedLine:=GetLineCount;
       end;
     { Do not remove the final newline if it exists PM }
     if hasCR then
       AddLine('');
    end;
  LimitsChanged;
  if not AllLinesComplete then
    SetModified(true);
  if (GetLineCount=MaxLineCount) and not eofstream(stream) then
    EditorDialog(edTooManyLines,nil);
  LoadFromStream:=OK;
end;

function TCustomCodeEditorCore.SaveAreaToStream(Editor: PCustomCodeEditor; Stream: PStream; StartP,EndP: TPoint): boolean;
var S: string;
    OK: boolean;
    Line: Sw_integer;
begin
  if EndP.X=0 then
    begin
      if EndP.Y>0 then
        begin
          EndP.X:=length(GetDisplayText(EndP.Y));
        end
      else
        EndP.X:=0;
    end
  else
    Dec(EndP.X);
  OK:=(Stream^.Status=stOK); Line:=StartP.Y;
  while OK and (Line<=EndP.Y) and (Line<GetLineCount) do
  begin
    S:=GetLineText(Line);
    { Remove all traling spaces PM }
    if not Editor^.IsFlagSet(efKeepTrailingSpaces) then
      While (Length(S)>0) and (S[Length(S)]=' ') do
       Dec(S[0]);
    { if FlagSet(efUseTabCharacters) then
      S:=CompressUsingTabs(S,TabSize);
      }
    if Line=EndP.Y then S:=copy(S,1,LinePosToCharIdx(Line,EndP.X));
    if Line=StartP.Y then S:=copy(S,LinePosToCharIdx(Line,StartP.X),High(S));
    Stream^.Write(S[1],length(S));
    if Line<EndP.Y then
      Stream^.Write(EOL[1],length(EOL));
    Inc(Line);
    OK:=OK and (Stream^.Status=stOK);
  end;
  SaveAreaToStream:=OK;
end;


constructor TEditorAction.init(act:byte; StartP,EndP:TPoint;Txt:String;AFlags : longint);
begin
  Action:=act;
  StartPos:=StartP;
  EndPos:=EndP;
  Text:=NewStr(txt);
  ActionCount:=0;
  Flags:=AFlags;
  TimeStamp:=Now;
  IsGrouped:=false;
end;

constructor TEditorAction.init_group(act:byte);
begin
  Action:=act;
  ActionCount:=0;
  Flags:=0;
  IsGrouped:=true;
end;

function TEditorAction.Is_grouped_action : boolean;
begin
  Is_grouped_action:=IsGrouped;
end;

destructor TEditorAction.done;
begin
  DisposeStr(Text);
  inherited done;
end;


function TEditorActionCollection.At(Idx : sw_integer) : PEditorAction;
begin
  At:=PEditorAction(Inherited At(Idx));
end;

procedure TEditorInputLine.HandleEvent(var Event : TEvent);
var
  s,s2 : string;
  i : longint;
begin
     If (Event.What=evKeyDown) then
       begin
         if (Event.KeyCode=kbRight) and
            (CurPos = Length(Data^)) and
            Assigned(FindReplaceEditor) then
           Begin
             s:=FindReplaceEditor^.GetDisplayText(FindReplaceEditor^.CurPos.Y);
             s:=Copy(s,FindReplaceEditor^.CurPos.X + 1 -length(Data^),high(s));
             i:=pos(Data^,s);
             if i>0 then
               begin
                 s:=Data^+s[i+length(Data^)];
                 If not assigned(validator) or
                    Validator^.IsValidInput(s,False)  then
                   Begin
                     Event.CharCode:=s[length(s)];
                     Event.Scancode:=0;
                     Inherited HandleEvent(Event);
                   End;
               end;
             ClearEvent(Event);
           End
         else if (Event.KeyCode=kbShiftIns)  and
                 Assigned(Clipboard) and (Clipboard^.ValidBlock) then
           { paste from clipboard }
           begin
             i:=Clipboard^.SelStart.Y;
             s:=Clipboard^.GetDisplayText(i);
             i:=Clipboard^.SelStart.X;
             if i>0 then
              s:=copy(s,i+1,high(s));
             if (Clipboard^.SelStart.Y=Clipboard^.SelEnd.Y) then
               begin
                 i:=Clipboard^.SelEnd.X-i;
                 s:=copy(s,1,i);
               end;
             for i:=1 to length(s) do
               begin
                 s2:=Data^+s[i];
                 If not assigned(validator) or
                    Validator^.IsValidInput(s2,False)  then
                   Begin
                     Event.What:=evKeyDown;
                     Event.CharCode:=s[i];
                     Event.Scancode:=0;
                     Inherited HandleEvent(Event);
                   End;
               end;
             ClearEvent(Event);
           end
         else if (Event.KeyCode=kbCtrlIns)  and
                 Assigned(Clipboard) then
           { Copy to clipboard }
           begin
             s:=GetStr(Data);
             s:=copy(s,selstart+1,selend-selstart);
             Clipboard^.SelStart:=Clipboard^.CurPos;
             Clipboard^.InsertText(s);
             Clipboard^.SelEnd:=Clipboard^.CurPos;
             ClearEvent(Event);
           end
         else if (Event.KeyCode=kbShiftDel)  and
                 Assigned(Clipboard) then
           { Cut to clipboard }
           begin
             s:=GetStr(Data);
             s:=copy(s,selstart+1,selend-selstart);
             Clipboard^.SelStart:=Clipboard^.CurPos;
             Clipboard^.InsertText(s);
             Clipboard^.SelEnd:=Clipboard^.CurPos;
             s2:=GetStr(Data);
             { now remove the selected part }
             Event.keyCode:=kbDel;
             inherited HandleEvent(Event);
             ClearEvent(Event);
           end
         else
           Inherited HandleEvent(Event);
       End
     else
       Inherited HandleEvent(Event);
  s:=getstr(data);
  Message(Owner,evBroadCast,cminputlinelen,pointer(length(s)));
end;

procedure TSearchHelperDialog.HandleEvent(var Event : TEvent);
begin
 case Event.What of
     evBroadcast :
           case Event.Command of
                   cminputlinelen : begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                      if PtrInt(Event.InfoPtr)=0 then
                                        okbutton^.DisableCommands([cmok])
=======
                                      if Event.InfoLong=0 then
                                        okbutton^.DisableCommands([cmok]) 
>>>>>>> graemeg/cpstrnew
=======
                                      if Event.InfoLong=0 then
                                        okbutton^.DisableCommands([cmok]) 
>>>>>>> graemeg/cpstrnew
=======
                                      if Event.InfoLong=0 then
                                        okbutton^.DisableCommands([cmok]) 
>>>>>>> graemeg/cpstrnew
=======
                                      if Event.InfoLong=0 then
                                        okbutton^.DisableCommands([cmok]) 
>>>>>>> origin/cpstrnew
                                      else
                                        okbutton^.EnableCommands([cmok]);
                                      clearevent(event);
                                    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
             end;
=======
             end;      
>>>>>>> graemeg/cpstrnew
=======
             end;      
>>>>>>> graemeg/cpstrnew
=======
             end;      
>>>>>>> graemeg/cpstrnew
=======
             end;      
>>>>>>> origin/cpstrnew
       end;
  inherited HandleEvent(Event);
end;


function CreateFindDialog: PDialog;
var R,R1,R2: TRect;
    D: PSearchHelperDialog;
    IL1: PEditorInputLine;
    Control : PView;
    CB1: PCheckBoxes;
    RB1,RB2,RB3: PRadioButtons;
    but : PButton;
begin
  R.Assign(0,0,56,15);
  New(D, Init(R, dialog_find));
  with D^ do
  begin
    Options:=Options or ofCentered;
    GetExtent(R); R.Grow(-3,-2);
    R1.Copy(R); R1.B.X:=17; R1.B.Y:=R1.A.Y+1;
    R2.Copy(R); R2.B.X:=R2.B.X-3;R2.A.X:=17; R2.B.Y:=R2.A.Y+1;
    New(IL1, Init(R2, FindStrSize));
    IL1^.Data^:=FindStr;
    Insert(IL1);
    Insert(New(PLabel, Init(R1, label_find_texttofind, IL1)));
    R1.Assign(R2.B.X, R2.A.Y, R2.B.X+3, R2.B.Y);
    Control := New(PHistory, Init(R1, IL1, TextFindId));
    Insert(Control);

    R1.Copy(R); Inc(R1.A.Y,2); R1.B.Y:=R1.A.Y+1; R1.B.X:=R1.A.X+(R1.B.X-R1.A.X) div 2-1;
    R2.Copy(R1); R2.Move(0,1);
    R2.B.Y:=R2.A.Y+{$ifdef TEST_REGEXP}3{$else}2{$endif};
    New(CB1, Init(R2,
      NewSItem(label_find_casesensitive,
      NewSItem(label_find_wholewordsonly,
{$ifdef TEST_REGEXP}
      NewSItem(label_find_useregexp,
{$endif TEST_REGEXP}
      nil)))){$ifdef TEST_REGEXP}){$endif TEST_REGEXP};
    Insert(CB1);
    Insert(New(PLabel, Init(R1, label_find_options, CB1)));

    R1.Copy(R); Inc(R1.A.Y,2); R1.B.Y:=R1.A.Y+1; R1.A.X:=R1.B.X-(R1.B.X-R1.A.X) div 2+1;
    R2.Copy(R1); R2.Move(0,1); R2.B.Y:=R2.A.Y+2;
    New(RB1, Init(R2,
      NewSItem(label_find_forward,
      NewSItem(label_find_backward,
      nil))));
    Insert(RB1);
    Insert(New(PLabel, Init(R1, label_find_direction, RB1)));

    R1.Copy(R); Inc(R1.A.Y,6); R1.B.Y:=R1.A.Y+1; R1.B.X:=R1.A.X+(R1.B.X-R1.A.X) div 2-1;
    R2.Copy(R1); R2.Move(0,1); R2.B.Y:=R2.A.Y+2;
    New(RB2, Init(R2,
      NewSItem(label_find_global,
      NewSItem(label_find_selectedtext,
      nil))));
    Insert(RB2);
    Insert(New(PLabel, Init(R1, label_find_scope, RB2)));

    R1.Copy(R); Inc(R1.A.Y,6); R1.B.Y:=R1.A.Y+1; R1.A.X:=R1.B.X-(R1.B.X-R1.A.X) div 2+1;
    R2.Copy(R1); R2.Move(0,1); R2.B.Y:=R2.A.Y+2;
    New(RB3, Init(R2,
      NewSItem(label_find_fromcursor,
      NewSItem(label_find_entirescope,
      nil))));
    Insert(RB3);
    Insert(New(PLabel, Init(R1, label_find_origin, RB3)));

    GetExtent(R); R.Grow(-13,-1); R.A.Y:=R.B.Y-2; R.B.X:=R.A.X+10;
    Okbutton:=New(PButton, Init(R, btn_OK, cmOK, bfDefault));
    Insert(OkButton);
    R.Move(19,0);
    Insert(New(PButton, Init(R, btn_Cancel, cmCancel, bfNormal)));
  end;
  IL1^.Select;
  CreateFindDialog := D;
end;

function CreateReplaceDialog: PDialog;
var R,R1,R2: TRect;
    D: PDialog;
    Control : PView;
    IL1: PEditorInputLine;
    IL2: PEditorInputLine;
    CB1: PCheckBoxes;
    RB1,RB2,RB3: PRadioButtons;
begin
  R.Assign(0,0,56,18);
  New(D, Init(R, dialog_replace));
  with D^ do
  begin
    Options:=Options or ofCentered;
    GetExtent(R); R.Grow(-3,-2);
    R1.Copy(R); R1.B.X:=17; R1.B.Y:=R1.A.Y+1;
    R2.Copy(R); R2.B.X:=R2.B.X-3;R2.A.X:=17; R2.B.Y:=R2.A.Y+1;
    New(IL1, Init(R2, FindStrSize));
    IL1^.Data^:=FindStr;
    Insert(IL1);
    Insert(New(PLabel, Init(R1, label_replace_texttofind, IL1)));
    R1.Assign(R2.B.X, R2.A.Y, R2.B.X+3, R2.B.Y);
    Control := New(PHistory, Init(R1, IL1, TextFindId));
    Insert(Control);

    R1.Copy(R); R1.Move(0,2); R1.B.X:=17; R1.B.Y:=R1.A.Y+1;
    R2.Copy(R); R2.Move(0,2);R2.B.X:=R2.B.X-3;
    R2.A.X:=17; R2.B.Y:=R2.A.Y+1;
    New(IL2, Init(R2, FindStrSize));
    IL2^.Data^:=ReplaceStr;
    Insert(IL2);
    Insert(New(PLabel, Init(R1, label_replace_newtext, IL2)));
    R1.Assign(R2.B.X, R2.A.Y, R2.B.X+3, R2.B.Y);
    Control := New(PHistory, Init(R1, IL2, TextReplaceId));
    Insert(Control);

    R1.Copy(R); Inc(R1.A.Y,4); R1.B.Y:=R1.A.Y+1; R1.B.X:=R1.A.X+(R1.B.X-R1.A.X) div 2-1;
    R2.Copy(R1); R2.Move(0,1);
    R2.B.Y:=R2.A.Y+{$ifdef TEST_REGEXP}4{$else}3{$endif};
    New(CB1, Init(R2,
      NewSItem(label_replace_casesensitive,
      NewSItem(label_replace_wholewordsonly,
      NewSItem(label_replace_promptonreplace,
{$ifdef TEST_REGEXP}
      NewSItem(label_find_useregexp,
{$endif TEST_REGEXP}
      nil))))){$ifdef TEST_REGEXP}){$endif TEST_REGEXP};
    Insert(CB1);
    Insert(New(PLabel, Init(R1, label_replace_options, CB1)));

    R1.Copy(R); Inc(R1.A.Y,4); R1.B.Y:=R1.A.Y+1; R1.A.X:=R1.B.X-(R1.B.X-R1.A.X) div 2+1;
    R2.Copy(R1); R2.Move(0,1); R2.B.Y:=R2.A.Y+2;
    New(RB1, Init(R2,
      NewSItem(label_replace_forward,
      NewSItem(label_replace_backward,
      nil))));
    Insert(RB1);
    Insert(New(PLabel, Init(R1, label_replace_direction, RB1)));

    R1.Copy(R); Inc(R1.A.Y,9); R1.B.Y:=R1.A.Y+1; R1.B.X:=R1.A.X+(R1.B.X-R1.A.X) div 2-1;
    R2.Copy(R1); R2.Move(0,1); R2.B.Y:=R2.A.Y+2;
    New(RB2, Init(R2,
      NewSItem(label_replace_global,
      NewSItem(label_replace_selectedtext,
      nil))));
    Insert(RB2);
    Insert(New(PLabel, Init(R1, label_replace_scope, RB2)));

    R1.Copy(R); Inc(R1.A.Y,9); R1.B.Y:=R1.A.Y+1; R1.A.X:=R1.B.X-(R1.B.X-R1.A.X) div 2+1;
    R2.Copy(R1); R2.Move(0,1); R2.B.Y:=R2.A.Y+2;
    New(RB3, Init(R2,
      NewSItem(label_replace_fromcursor,
      NewSItem(label_replace_entirescope,
      nil))));
    Insert(RB3);
    Insert(New(PLabel, Init(R1, label_replace_origin, RB3)));

    GetExtent(R); R.Grow(-13,-1); R.A.Y:=R.B.Y-2; R.B.X:=R.A.X+10; R.Move(-10,0);
    Insert(New(PButton, Init(R, btn_OK, cmOK, bfDefault)));
    R.Move(11,0); R.B.X:=R.A.X+14;
    Insert(New(PButton, Init(R, btn_replace_changeall, cmYes, bfNormal)));
    R.Move(15,0); R.B.X:=R.A.X+10;
    Insert(New(PButton, Init(R, btn_Cancel, cmCancel, bfNormal)));
  end;
  IL1^.Select;
  CreateReplaceDialog := D;
end;

function CreateGotoLineDialog(Info: pointer): PDialog;
var D: PDialog;
    R,R1,R2: TRect;
    Control : PView;
    IL: PEditorInputLine;
begin
  R.Assign(0,0,40,7);
  New(D, Init(R, dialog_gotoline));
  with D^ do
  begin
    Options:=Options or ofCentered;
    GetExtent(R); R.Grow(-3,-2); R.B.Y:=R.A.Y+1;
    R1.Copy(R); R1.B.X:=27; R2.Copy(R);
    R2.B.X:=R2.B.X-3;R2.A.X:=27;
    New(IL, Init(R2,5));
    with TGotoLineDialogRec(Info^) do
    IL^.SetValidator(New(PRangeValidator, Init(1, Lines)));
    Insert(IL);
    Insert(New(PLabel, Init(R1, label_gotoline_linenumber, IL)));
    R1.Assign(R2.B.X, R2.A.Y, R2.B.X+3, R2.B.Y);
    Control := New(PHistory, Init(R1, IL, GotoId));
    Insert(Control);

    GetExtent(R); R.Grow(-8,-1); R.A.Y:=R.B.Y-2; R.B.X:=R.A.X+10;
    Insert(New(PButton, Init(R, btn_OK, cmOK, bfDefault)));
    R.Move(15,0);
    Insert(New(PButton, Init(R, btn_Cancel, cmCancel, bfNormal)));
  end;
  IL^.Select;
  CreateGotoLineDialog:=D;
end;

function StdEditorDialog(Dialog: Integer; Info: Pointer): Word;
var
  R: TRect;
  T: TPoint;
  Re: word;
  Name: string;
  DriveNumber : byte;
  StoreDir,StoreDir2 : DirStr;
  Title,DefExt: string;
  AskOW: boolean;
begin
  case Dialog of
    edOutOfMemory:
      StdEditorDialog := AdvMessageBox(msg_notenoughmemoryforthisoperation,
   nil, mfInsertInApp+ mfError + mfOkButton);
    edReadError:
      StdEditorDialog := AdvMessageBox(msg_errorreadingfile,
   @Info, mfInsertInApp+ mfError + mfOkButton);
    edWriteError:
      StdEditorDialog := AdvMessageBox(msg_errorwritingfile,
   @Info, mfInsertInApp+ mfError + mfOkButton);
    edSaveError:
      StdEditorDialog := AdvMessageBox(msg_errorsavingfile,
   @Info, mfInsertInApp+ mfError + mfOkButton);
    edCreateError:
      StdEditorDialog := AdvMessageBox(msg_errorcreatingfile,
   @Info, mfInsertInApp+ mfError + mfOkButton);
    edSaveModify:
      StdEditorDialog := AdvMessageBox(msg_filehasbeenmodifiedsave,
   @Info, mfInsertInApp+ mfInformation + mfYesNoCancel);
    edSaveUntitled:
      StdEditorDialog := AdvMessageBox(msg_saveuntitledfile,
   nil, mfInsertInApp+ mfInformation + mfYesNoCancel);
    edChangedOnloading:
      StdEditorDialog := AdvMessageBox(msg_filehadtoolonglines,
   Info, mfInsertInApp+ mfOKButton + mfInformation);
    edFileOnDiskChanged:
      StdEditorDialog := AdvMessageBox(msg_filewasmodified,
   @info, mfInsertInApp+ mfInformation + mfYesNoCancel);
    edReloadDiskmodifiedFile:
      StdEditorDialog := AdvMessageBox(msg_reloaddiskmodifiedfile,
   @info, mfInsertInApp+ mfInformation + mfYesNoCancel);
    edReloadDiskAndIDEModifiedFile:
      StdEditorDialog := AdvMessageBox(msg_reloaddiskandidemodifiedfile,
   @info, mfInsertInApp+ mfInformation + mfYesNoCancel);
    edSaveAs,edWriteBlock,edReadBlock:
      begin
        Name:=PString(Info)^;
        GetDir(0,StoreDir);
        DriveNumber:=0;
        if (Length(FileDir)>1) and (FileDir[2]=':') then
          begin
            { does not assume that lowercase are greater then uppercase ! }
            if (FileDir[1]>='a') and (FileDir[1]<='z') then
              DriveNumber:=Ord(FileDir[1])-ord('a')+1
            else
              DriveNumber:=Ord(FileDir[1])-ord('A')+1;
            GetDir(DriveNumber,StoreDir2);
            {$I-}
            ChDir(Copy(FileDir,1,2));
            EatIO;
            {$I+}
          end;
        if FileDir<>'' then
          begin
            {$I-}
            ChDir(TrimEndSlash(FileDir));
            EatIO;
            {$I+}
          end;
        case Dialog of
          edSaveAs     :
            begin
              Title:=dialog_savefileas;
              DefExt:='*'+DefaultSaveExt;
            end;
          edWriteBlock :
            begin
              Title:=dialog_writeblocktofile;
              DefExt:='*.*';
            end;
          edReadBlock  :
            begin
              Title:=dialog_readblockfromfile;
              DefExt:='*.*';
            end;
        else begin Title:='???'; DefExt:=''; end;
        end;
        Re:=Application^.ExecuteDialog(New(PFileDialog, Init(DefExt,
          Title, label_name, fdOkButton, FileId)), @Name);
        case Dialog of
          edSaveAs     :
            begin
              if ExtOf(Name)='' then
                Name:=Name+DefaultSaveExt;
              AskOW:=(Name<>PString(Info)^);
            end;
          edWriteBlock :
            begin
              if ExtOf(Name)='' then
                Name:=Name+DefaultSaveExt;
              AskOW:=true;
            end;
          edReadBlock  : AskOW:=false;
        else AskOW:=true;
        end;
        if (Re<>cmCancel) and AskOW then
          begin
            FileDir:=DirOf(FExpand(Name));
            if ExistsFile(Name) then
              if EditorDialog(edReplaceFile,@Name)<>cmYes then
                Re:=cmCancel;
          end;
        if DriveNumber<>0 then
          ChDir(StoreDir2);
        if StoreDir<>'' then
          ChDir(TrimEndSlash(StoreDir));

        if Re<>cmCancel then
          PString(Info)^:=Name;
        StdEditorDialog := Re;
      end;
    edGotoLine:
      StdEditorDialog :=
   Application^.ExecuteDialog(CreateGotoLineDialog(Info), Info);
    edFind:
      StdEditorDialog :=
   Application^.ExecuteDialog(CreateFindDialog, Info);
    edSearchFailed:
      StdEditorDialog := AdvMessageBox(msg_searchstringnotfound,
   nil, mfInsertInApp+ mfError + mfOkButton);
    edReplace:
      StdEditorDialog :=
   Application^.ExecuteDialog(CreateReplaceDialog, Info);
    edReplacePrompt:
      begin
   { Avoid placing the dialog on the same line as the cursor }
   R.Assign(0, 1, 40, 8);
   R.Move((Desktop^.Size.X - R.B.X) div 2, 0);
   Desktop^.MakeGlobal(R.B, T);
   Inc(T.Y);
   if PPoint(Info)^.Y <= T.Y then
     R.Move(0, Desktop^.Size.Y - R.B.Y - 2);
   StdEditorDialog := AdvMessageBoxRect(R, msg_replacethisoccourence,
     nil, mfInsertInApp+ mfYesNoCancel + mfInformation);
      end;
    edReplaceFile :
      StdEditorDialog :=
   AdvMessageBox(msg_fileexistsoverwrite,@Info,mfInsertInApp+mfConfirmation+
     mfYesButton+mfNoButton);
  end;
end;

procedure RegisterWEditor;
begin
{$ifndef NOOBJREG}
{$endif}
end;

END.
