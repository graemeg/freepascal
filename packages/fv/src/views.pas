{********[ SOURCE FILE OF GRAPHICAL FREE VISION ]**********}
{                                                          }
{   System independent GRAPHICAL clone of VIEWS.PAS        }
{                                                          }
{   Interface Copyright (c) 1992 Borland International     }
{                                                          }
{   Copyright (c) 1996, 1997, 1998, 1999 by Leon de Boer   }
{   ldeboer@attglobal.net  - primary e-mail address        }
{   ldeboer@starwon.com.au - backup e-mail address         }
{                                                          }
{****************[ THIS CODE IS FREEWARE ]*****************}
{                                                          }
{     This sourcecode is released for the purpose to       }
{   promote the pascal language on all platforms. You may  }
{   redistribute it and/or modify with the following       }
{   DISCLAIMER.                                            }
{                                                          }
{     This SOURCE CODE is distributed "AS IS" WITHOUT      }
{   WARRANTIES AS TO PERFORMANCE OF MERCHANTABILITY OR     }
{   ANY OTHER WARRANTIES WHETHER EXPRESSED OR IMPLIED.     }
{                                                          }
{*****************[ SUPPORTED PLATFORMS ]******************}
{                                                          }
{ Only Free Pascal Compiler supported                      }
{                                                          }
{**********************************************************}

UNIT Views;

{$CODEPAGE cp437}

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                  INTERFACE
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

{====Include file to sort compiler platform out =====================}
{$I platform.inc}
{====================================================================}

{==== Compiler directives ===========================================}

{$X+} { Extended syntax is ok }
{$R-} { Disable range checking }
{$S-} { Disable Stack Checking }
{$I-} { Disable IO Checking }
{$Q-} { Disable Overflow Checking }
{$V-} { Turn off strict VAR strings }
{====================================================================}

USES
   {$IFDEF OS_WINDOWS}                                { WIN/NT CODE }
         Windows,                                     { Standard unit }
   {$ENDIF}

   {$IFDEF OS_OS2}                                    { OS2 CODE }
     Os2Def, DosCalls, PmWin,
   {$ENDIF}

   Objects, FVCommon, Drivers, fvconsts;              { GFV standard units }

{***************************************************************************}
{                              PUBLIC CONSTANTS                             }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                              TView STATE MASKS                            }
{---------------------------------------------------------------------------}
CONST
   sfVisible   = $0001;                               { View visible mask }
   sfCursorVis = $0002;                               { Cursor visible }
   sfCursorIns = $0004;                               { Cursor insert mode }
   sfShadow    = $0008;                               { View has shadow }
   sfActive    = $0010;                               { View is active }
   sfSelected  = $0020;                               { View is selected }
   sfFocused   = $0040;                               { View is focused }
   sfDragging  = $0080;                               { View is dragging }
   sfDisabled  = $0100;                               { View is disabled }
   sfModal     = $0200;                               { View is modal }
   sfDefault   = $0400;                               { View is default }
   sfExposed   = $0800;                               { View is exposed }
   sfIconised  = $1000;                               { View is iconised }

{---------------------------------------------------------------------------}
{                             TView OPTION MASKS                            }
{---------------------------------------------------------------------------}
CONST
   ofSelectable  = $0001;                             { View selectable }
   ofTopSelect   = $0002;                             { Top selectable }
   ofFirstClick  = $0004;                             { First click react }
   ofFramed      = $0008;                             { View is framed }
   ofPreProcess  = $0010;                             { Pre processes }
   ofPostProcess = $0020;                             { Post processes }
   ofBuffered    = $0040;                             { View is buffered }
   ofTileable    = $0080;                             { View is tileable }
   ofCenterX     = $0100;                             { View centred on x }
   ofCenterY     = $0200;                             { View centred on y }
   ofCentered    = $0300;                             { View x,y centred }
   ofValidate    = $0400;                             { View validates }
   ofVersion     = $3000;                             { View TV version }
   ofVersion10   = $0000;                             { TV version 1 view }
   ofVersion20   = $1000;                             { TV version 2 view }

{---------------------------------------------------------------------------}
{                            TView GROW MODE MASKS                          }
{---------------------------------------------------------------------------}
CONST
   gfGrowLoX = $01;                                   { Left side grow }
   gfGrowLoY = $02;                                   { Top side grow  }
   gfGrowHiX = $04;                                   { Right side grow }
   gfGrowHiY = $08;                                   { Bottom side grow }
   gfGrowAll = $0F;                                   { Grow on all sides }
   gfGrowRel = $10;                                   { Grow relative }

{---------------------------------------------------------------------------}
{                           TView DRAG MODE MASKS                           }
{---------------------------------------------------------------------------}
CONST
   dmDragMove = $01;                                  { Move view }
   dmDragGrow = $02;                                  { Grow view }
   dmLimitLoX = $10;                                  { Limit left side }
   dmLimitLoY = $20;                                  { Limit top side }
   dmLimitHiX = $40;                                  { Limit right side }
   dmLimitHiY = $80;                                  { Limit bottom side }
   dmLimitAll = $F0;                                  { Limit all sides }

{---------------------------------------------------------------------------}
{                       >> NEW << TAB OPTION MASKS                          }
{---------------------------------------------------------------------------}
CONST
   tmTab      = $01;                                  { Tab move mask }
   tmShiftTab = $02;                                  { Shift+tab move mask }
   tmEnter    = $04;                                  { Enter move mask }
   tmLeft     = $08;                                  { Left arrow move mask }
   tmRight    = $10;                                  { Right arrow move mask }
   tmUp       = $20;                                  { Up arrow move mask }
   tmDown     = $40;                                  { Down arrow move mask }

{---------------------------------------------------------------------------}
{                        >> NEW << VIEW DRAW MASKS                          }
{---------------------------------------------------------------------------}
CONST
   vdBackGnd = $01;                                   { Draw backgound }
   vdInner   = $02;                                   { Draw inner detail }
   vdCursor  = $04;                                   { Draw cursor }
   vdBorder  = $08;                                   { Draw view border }
   vdFocus   = $10;                                   { Draw focus state }
   vdNoChild = $20;                                   { Draw no children }
   vdShadow  = $40;
   vdAll     = vdBackGnd + vdInner + vdCursor + vdBorder + vdFocus + vdShadow;

{---------------------------------------------------------------------------}
{                            TView HELP CONTEXTS                            }
{---------------------------------------------------------------------------}
CONST
   hcNoContext = 0;                                   { No view context }
   hcDragging  = 1;                                   { No drag context }

{---------------------------------------------------------------------------}
{                             TWindow FLAG MASKS                            }
{---------------------------------------------------------------------------}
CONST
   wfMove  = $01;                                     { Window can move }
   wfGrow  = $02;                                     { Window can grow }
   wfClose = $04;                                     { Window can close }
   wfZoom  = $08;                                     { Window can zoom }

{---------------------------------------------------------------------------}
{                              TWindow PALETTES                             }
{---------------------------------------------------------------------------}
CONST
   wpBlueWindow = 0;                                  { Blue palette }
   wpCyanWindow = 1;                                  { Cyan palette }
   wpGrayWindow = 2;                                  { Gray palette }

{---------------------------------------------------------------------------}
{                              COLOUR PALETTES                              }
{---------------------------------------------------------------------------}
CONST
   CFrame      = #1#1#2#2#3;                          { Frame palette }
   CScrollBar  = #4#5#5;                              { Scrollbar palette }
   CScroller   = #6#7;                                { Scroller palette }
   CListViewer = #26#26#27#28#29;                     { Listviewer palette }

   CBlueWindow = #8#9#10#11#12#13#14#15;              { Blue window palette }
   CCyanWindow = #16#17#18#19#20#21#22#23;            { Cyan window palette }
   CGrayWindow = #24#25#26#27#28#29#30#31;            { Grey window palette }

{---------------------------------------------------------------------------}
{                           TScrollBar PART CODES                           }
{---------------------------------------------------------------------------}
CONST
   sbLeftArrow  = 0;                                  { Left arrow part }
   sbRightArrow = 1;                                  { Right arrow part }
   sbPageLeft   = 2;                                  { Page left part }
   sbPageRight  = 3;                                  { Page right part }
   sbUpArrow    = 4;                                  { Up arrow part }
   sbDownArrow  = 5;                                  { Down arrow part }
   sbPageUp     = 6;                                  { Page up part }
   sbPageDown   = 7;                                  { Page down part }
   sbIndicator  = 8;                                  { Indicator part }

{---------------------------------------------------------------------------}
{              TScrollBar OPTIONS FOR TWindow.StandardScrollBar             }
{---------------------------------------------------------------------------}
CONST
   sbHorizontal     = $0000;                          { Horz scrollbar }
   sbVertical       = $0001;                          { Vert scrollbar }
   sbHandleKeyboard = $0002;                          { Handle keyboard }

{---------------------------------------------------------------------------}
{                            STANDARD COMMAND CODES                         }
{---------------------------------------------------------------------------}
CONST
   cmValid   = 0;                                     { Valid command }
   cmQuit    = 1;                                     { Quit command }
   cmError   = 2;                                     { Error command }
   cmMenu    = 3;                                     { Menu command }
   cmClose   = 4;                                     { Close command }
   cmZoom    = 5;                                     { Zoom command }
   cmResize  = 6;                                     { Resize command }
   cmNext    = 7;                                     { Next view command }
   cmPrev    = 8;                                     { Prev view command }
   cmHelp    = 9;                                     { Help command }
   cmOK      = 10;                                    { Okay command }
   cmCancel  = 11;                                    { Cancel command }
   cmYes     = 12;                                    { Yes command }
   cmNo      = 13;                                    { No command }
   cmDefault = 14;                                    { Default command }
   cmCut     = 20;                                    { Clipboard cut cmd }
   cmCopy    = 21;                                    { Clipboard copy cmd }
   cmPaste   = 22;                                    { Clipboard paste cmd }
   cmUndo    = 23;                                    { Clipboard undo cmd }
   cmClear   = 24;                                    { Clipboard clear cmd }
   cmTile    = 25;                                    { Tile subviews cmd }
   cmCascade = 26;                                    { Cascade subviews cmd }
   cmReceivedFocus     = 50;                          { Received focus }
   cmReleasedFocus     = 51;                          { Released focus }
   cmCommandSetChanged = 52;                          { Commands changed }
   cmScrollBarChanged  = 53;                          { Scrollbar changed }
   cmScrollBarClicked  = 54;                          { Scrollbar clicked on }
   cmSelectWindowNum   = 55;                          { Select window }
   cmListItemSelected  = 56;                          { Listview item select }

   cmNotify = 27;
   cmIdCommunicate     = 28;                          { Communicate via id }
   cmIdSelect          = 29;                          { Select via id }

{---------------------------------------------------------------------------}
{                          TWindow NUMBER CONSTANTS                         }
{---------------------------------------------------------------------------}
CONST
   wnNoNumber = 0;                                    { Window has no num }
   MaxViewWidth = 255;                                { Max view width }


{***************************************************************************}
{                          PUBLIC TYPE DEFINITIONS                          }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                            TWindow Title string                           }
{---------------------------------------------------------------------------}
TYPE
   TTitleStr = String[80];                            { Window title string }

{---------------------------------------------------------------------------}
{                            COMMAND SET RECORD                             }
{---------------------------------------------------------------------------}
TYPE
   TCommandSet = SET OF Byte;                         { Command set record }
   PCommandSet = ^TCommandSet;                        { Ptr to command set }

{---------------------------------------------------------------------------}
{                              PALETTE RECORD                               }
{---------------------------------------------------------------------------}
TYPE
   TPalette = String;                                 { Palette record }
   PPalette = ^TPalette;                              { Pointer to palette }

{---------------------------------------------------------------------------}
{                            TDrawBuffer RECORD                             }
{---------------------------------------------------------------------------}
TYPE
   TDrawBuffer = Array [0..MaxViewWidth - 1] Of Word; { Draw buffer record }
   PDrawBuffer = ^TDrawBuffer;                        { Ptr to draw buffer }

{---------------------------------------------------------------------------}
{                           TVideoBuffer RECORD                             }
{---------------------------------------------------------------------------}
TYPE
   TVideoBuf = ARRAY [0..3999] of Word;               { Video buffer }
   PVideoBuf = ^TVideoBuf;                            { Pointer to buffer }

{---------------------------------------------------------------------------}
{                            TComplexArea RECORD                            }
{---------------------------------------------------------------------------}
TYPE
   PComplexArea = ^TComplexArea;                      { Complex area }
   TComplexArea =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
   PACKED
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
   RECORD
      X1, Y1  : Sw_Integer;                              { Top left corner }
      X2, Y2  : Sw_Integer;                              { Lower right corner }
      NextArea: PComplexArea;                         { Next area pointer }
   END;

{***************************************************************************}
{                        PUBLIC OBJECT DEFINITIONS                          }
{***************************************************************************}

TYPE
   PGroup = ^TGroup;                                  { Pointer to group }

{---------------------------------------------------------------------------}
{                    TView OBJECT - ANCESTOR VIEW OBJECT                    }
{---------------------------------------------------------------------------}
   PView = ^TView;
   TView = OBJECT (TObject)
         GrowMode : Byte;                             { View grow mode }
         DragMode : Byte;                             { View drag mode }
         TabMask  : Byte;                             { Tab move masks }
         ColourOfs: Sw_Integer;                          { View palette offset }
         HelpCtx  : Word;                             { View help context }
         State    : Word;                             { View state masks }
         Options  : Word;                             { View options masks }
         EventMask: Word;                             { View event masks }
         Origin   : TPoint;                           { View origin }
         Size     : TPoint;                           { View size }
         Cursor   : TPoint;                           { Cursor position }
         Next     : PView;                            { Next peerview }
         Owner    : PGroup;                           { Owner group }
         HoldLimit: PComplexArea;                     { Hold limit values }

         RevCol    : Boolean;
         BackgroundChar : Char;

      CONSTRUCTOR Init (Var Bounds: TRect);
      CONSTRUCTOR Load (Var S: TStream);
      DESTRUCTOR Done; Virtual;
      FUNCTION Prev: PView;
      FUNCTION Execute: Word; Virtual;
      FUNCTION Focus: Boolean;
      FUNCTION DataSize: Sw_Word; Virtual;
      FUNCTION TopView: PView;
      FUNCTION PrevView: PView;
      FUNCTION NextView: PView;
      FUNCTION GetHelpCtx: Word; Virtual;
      FUNCTION EventAvail: Boolean;
      FUNCTION GetPalette: PPalette; Virtual;
      function MapColor (color:byte):byte;
      FUNCTION GetColor (Color: Word): Word;
      FUNCTION Valid (Command: Word): Boolean; Virtual;
      FUNCTION GetState (AState: Word): Boolean;
      FUNCTION TextWidth (const Txt: String): Sw_Integer;
      FUNCTION CTextWidth (const Txt: String): Sw_Integer;
      FUNCTION MouseInView (Point: TPoint): Boolean;
      FUNCTION CommandEnabled (Command: Word): Boolean;
      FUNCTION OverLapsArea (X1, Y1, X2, Y2: Sw_Integer): Boolean;
      FUNCTION MouseEvent (Var Event: TEvent; Mask: Word): Boolean;
      PROCEDURE Hide;
      PROCEDURE Show;
      PROCEDURE Draw; Virtual;
      PROCEDURE ResetCursor; Virtual;
      PROCEDURE Select;
      PROCEDURE Awaken; Virtual;
      PROCEDURE DrawView;
      PROCEDURE MakeFirst;
      PROCEDURE DrawCursor; Virtual;
      PROCEDURE HideCursor;
      PROCEDURE ShowCursor;
      PROCEDURE BlockCursor;
      PROCEDURE NormalCursor;
      PROCEDURE FocusFromTop; Virtual;
      PROCEDURE MoveTo (X, Y: Sw_Integer);
      PROCEDURE GrowTo (X, Y: Sw_Integer);
      PROCEDURE EndModal (Command: Word); Virtual;
      PROCEDURE SetCursor (X, Y: Sw_Integer);
      PROCEDURE PutInFrontOf (Target: PView);
      PROCEDURE SetCommands (Commands: TCommandSet);
      PROCEDURE EnableCommands (Commands: TCommandSet);
      PROCEDURE DisableCommands (Commands: TCommandSet);
      PROCEDURE SetState (AState: Word; Enable: Boolean); Virtual;
      PROCEDURE SetCmdState (Commands: TCommandSet; Enable: Boolean);
      PROCEDURE GetData (Var Rec); Virtual;
      PROCEDURE SetData (Var Rec); Virtual;
      PROCEDURE Store (Var S: TStream);
      PROCEDURE Locate (Var Bounds: TRect);
      PROCEDURE KeyEvent (Var Event: TEvent);
      PROCEDURE GetEvent (Var Event: TEvent); Virtual;
      PROCEDURE PutEvent (Var Event: TEvent); Virtual;
      PROCEDURE GetExtent (Var Extent: TRect);
      PROCEDURE GetBounds (Var Bounds: TRect);
      PROCEDURE SetBounds (Var Bounds: TRect);
      PROCEDURE GetClipRect (Var Clip: TRect);
      PROCEDURE ClearEvent (Var Event: TEvent);
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
      PROCEDURE ChangeBounds (Var Bounds: TRect); Virtual;
      PROCEDURE SizeLimits (Var Min, Max: TPoint); Virtual;
      PROCEDURE GetCommands (Var Commands: TCommandSet);
      PROCEDURE GetPeerViewPtr (Var S: TStream; Var P);
      PROCEDURE PutPeerViewPtr (Var S: TStream; P: PView);
      PROCEDURE CalcBounds (Var Bounds: TRect; Delta: TPoint); Virtual;

      FUNCTION Exposed: Boolean;   { This needs help!!!!! }
      PROCEDURE WriteBuf (X, Y, W, H: Sw_Integer; Var Buf);
      PROCEDURE WriteLine (X, Y, W, H: Sw_Integer; Var Buf);
      PROCEDURE MakeLocal (Source: TPoint; Var Dest: TPoint);
      PROCEDURE MakeGlobal (Source: TPoint; Var Dest: TPoint);
      PROCEDURE WriteStr (X, Y: Sw_Integer; Str: String; Color: Byte);
      PROCEDURE WriteChar (X, Y: Sw_Integer; C: Char; Color: Byte;
        Count: Sw_Integer);
      PROCEDURE DragView (Event: TEvent; Mode: Byte; Var Limits: TRect;
        MinSize, MaxSize: TPoint);
   private
      procedure CursorChanged;
      procedure DrawHide(LastView: PView);
      procedure DrawShow(LastView: PView);
      procedure DrawUnderRect(var R: TRect; LastView: PView);
      procedure DrawUnderView(DoShadow: Boolean; LastView: PView);
      procedure do_WriteView(x1,x2,y:Sw_Integer; var Buf);
      procedure do_WriteViewRec1(x1,x2:Sw_integer; p:PView; shadowCounter:Sw_integer);
      procedure do_WriteViewRec2(x1,x2:Sw_integer; p:PView; shadowCounter:Sw_integer);
      function  do_ExposedRec1(x1,x2:Sw_integer; p:PView):boolean;
      function  do_ExposedRec2(x1,x2:Sw_integer; p:PView):boolean;
   END;

   SelectMode = (NormalSelect, EnterSelect, LeaveSelect);

{---------------------------------------------------------------------------}
{                  TGroup OBJECT - GROUP OBJECT ANCESTOR                    }
{---------------------------------------------------------------------------}
{$ifndef TYPED_LOCAL_CALLBACKS}
   TGroupFirstThatCallback = CodePointer;
{$else}
   TGroupFirstThatCallback = Function(View: PView): Boolean is nested;
{$endif}

   TGroup = OBJECT (TView)
         Phase   : (phFocused, phPreProcess, phPostProcess);
         EndState: Word;                              { Modal result }
         Current : PView;                             { Selected subview }
         Last    : PView;                             { 1st view inserted }
         Buffer  : PVideoBuf;                         { Speed up buffer }
      CONSTRUCTOR Init (Var Bounds: TRect);
      CONSTRUCTOR Load (Var S: TStream);
      DESTRUCTOR Done; Virtual;
      FUNCTION First: PView;
      FUNCTION Execute: Word; Virtual;
      FUNCTION GetHelpCtx: Word; Virtual;
      FUNCTION DataSize: Sw_Word; Virtual;
      FUNCTION ExecView (P: PView): Word; Virtual;
      FUNCTION FirstThat (P:  TGroupFirstThatCallback): PView;
      FUNCTION Valid (Command: Word): Boolean; Virtual;
      FUNCTION FocusNext (Forwards: Boolean): Boolean;
      PROCEDURE Draw; Virtual;
      PROCEDURE Lock;
      PROCEDURE UnLock;
      PROCEDURE ResetCursor; Virtual;
      PROCEDURE Awaken; Virtual;
      PROCEDURE ReDraw;
      PROCEDURE SelectDefaultView;
      PROCEDURE Insert (P: PView);
      PROCEDURE Delete (P: PView);
      PROCEDURE ForEach (P: TCallbackProcParam);
      { ForEach can't be virtual because it generates SIGSEGV }
      PROCEDURE EndModal (Command: Word); Virtual;
      PROCEDURE SelectNext (Forwards: Boolean);
      PROCEDURE InsertBefore (P, Target: PView);
      PROCEDURE SetState (AState: Word; Enable: Boolean); Virtual;
      PROCEDURE GetData (Var Rec); Virtual;
      PROCEDURE SetData (Var Rec); Virtual;
      PROCEDURE Store (Var S: TStream);
      PROCEDURE EventError (Var Event: TEvent); Virtual;
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
      PROCEDURE ChangeBounds (Var Bounds: TRect); Virtual;
      PROCEDURE GetSubViewPtr (Var S: TStream; Var P);
      PROCEDURE PutSubViewPtr (Var S: TStream; P: PView);
      function ClipChilds: boolean; virtual;
      procedure BeforeInsert(P: PView); virtual;
      procedure AfterInsert(P: PView); virtual;
      procedure BeforeDelete(P: PView); virtual;
      procedure AfterDelete(P: PView); virtual;

      PRIVATE
         LockFlag: Byte;
         Clip    : TRect;
      FUNCTION IndexOf (P: PView): Sw_Integer;
      FUNCTION FindNext (Forwards: Boolean): PView;
      FUNCTION FirstMatch (AState: Word; AOptions: Word): PView;
      PROCEDURE ResetCurrent;
      PROCEDURE RemoveView (P: PView);
      PROCEDURE InsertView (P, Target: PView);
      PROCEDURE SetCurrent (P: PView; Mode: SelectMode);
      procedure DrawSubViews(P, Bottom: PView);
   END;

{---------------------------------------------------------------------------}
{                    TFrame OBJECT - FRAME VIEW OBJECT                      }
{---------------------------------------------------------------------------}
TYPE
   TFrame = OBJECT (TView)
      CONSTRUCTOR Init (Var Bounds: TRect);
      FUNCTION GetPalette: PPalette; Virtual;
      procedure Draw; virtual;
      procedure HandleEvent(var Event: TEvent); virtual;
      procedure SetState(AState: Word; Enable: Boolean); virtual;
   private
      FrameMode: Word;
      procedure FrameLine(var FrameBuf; Y, N: Sw_Integer; Color: Byte);
   END;
   PFrame = ^TFrame;

{---------------------------------------------------------------------------}
{                   TScrollBar OBJECT - SCROLL BAR OBJECT                   }
{---------------------------------------------------------------------------}
TYPE
   TScrollChars = Array [0..4] of Char;

   TScrollBar = OBJECT (TView)
         Value : Sw_Integer;                             { Scrollbar value }
         Min   : Sw_Integer;                             { Scrollbar minimum }
         Max   : Sw_Integer;                             { Scrollbar maximum }
         PgStep: Sw_Integer;                             { One page step }
         ArStep: Sw_Integer;                             { One range step }
         Id    : Sw_Integer;                             { Scrollbar ID }
      CONSTRUCTOR Init (Var Bounds: TRect);
      CONSTRUCTOR Load (Var S: TStream);
      FUNCTION GetPalette: PPalette; Virtual;
      FUNCTION ScrollStep (Part: Sw_Integer): Sw_Integer; Virtual;
      PROCEDURE Draw; Virtual;
      PROCEDURE ScrollDraw;                                          Virtual;
      PROCEDURE SetValue (AValue: Sw_Integer);
      PROCEDURE SetRange (AMin, AMax: Sw_Integer);
      PROCEDURE SetStep (APgStep, AArStep: Sw_Integer);
      PROCEDURE SetParams (AValue, AMin, AMax, APgStep, AArStep: Sw_Integer);
      PROCEDURE Store (Var S: TStream);
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
      PRIVATE
         Chars: TScrollChars;                         { Scrollbar chars }
      FUNCTION GetPos: Sw_Integer;
      FUNCTION GetSize: Sw_Integer;
      PROCEDURE DrawPos (Pos: Sw_Integer);
   END;
   PScrollBar = ^TScrollBar;

{---------------------------------------------------------------------------}
{                 TScroller OBJECT - SCROLLING VIEW ANCESTOR                }
{---------------------------------------------------------------------------}
TYPE
   TScroller = OBJECT (TView)
         Delta     : TPoint;
         Limit     : TPoint;
         HScrollBar: PScrollBar;                      { Horz scroll bar }
         VScrollBar: PScrollBar;                      { Vert scroll bar }
      CONSTRUCTOR Init (Var Bounds: TRect; AHScrollBar, AVScrollBar: PScrollBar);
      CONSTRUCTOR Load (Var S: TStream);
      FUNCTION GetPalette: PPalette; Virtual;
      PROCEDURE ScrollDraw;                                          Virtual;
      PROCEDURE SetLimit (X, Y: Sw_Integer);
      PROCEDURE ScrollTo (X, Y: Sw_Integer);
      PROCEDURE SetState (AState: Word; Enable: Boolean); Virtual;
      PROCEDURE Store (Var S: TStream);
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
      PROCEDURE ChangeBounds (Var Bounds: TRect); Virtual;
        PRIVATE
         DrawFlag: Boolean;
         DrawLock: Byte;
      PROCEDURE CheckDraw;
   END;
   PScroller = ^TScroller;

{---------------------------------------------------------------------------}
{                  TListViewer OBJECT - LIST VIEWER OBJECT                  }
{---------------------------------------------------------------------------}
TYPE
   TListViewer = OBJECT (TView)
         NumCols   : Sw_Integer;                         { Number of columns }
         TopItem   : Sw_Integer;                         { Top most item }
         Focused   : Sw_Integer;                         { Focused item }
         Range     : Sw_Integer;                         { Range of listview }
         HScrollBar: PScrollBar;                      { Horz scrollbar }
         VScrollBar: PScrollBar;                      { Vert scrollbar }
      CONSTRUCTOR Init (Var Bounds: TRect; ANumCols: Sw_Word; AHScrollBar,
        AVScrollBar: PScrollBar);
      CONSTRUCTOR Load (Var S: TStream);
      FUNCTION GetPalette: PPalette; Virtual;
      FUNCTION IsSelected (Item: Sw_Integer): Boolean; Virtual;
      FUNCTION GetText (Item: Sw_Integer; MaxLen: Sw_Integer): String; Virtual;
      PROCEDURE Draw; Virtual;
      PROCEDURE FocusItem (Item: Sw_Integer); Virtual;
      PROCEDURE SetTopItem (Item: Sw_Integer);
      PROCEDURE SetRange (ARange: Sw_Integer);
      PROCEDURE SelectItem (Item: Sw_Integer); Virtual;
      PROCEDURE SetState (AState: Word; Enable: Boolean); Virtual;
      PROCEDURE Store (Var S: TStream);
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
      PROCEDURE ChangeBounds (Var Bounds: TRect); Virtual;
      PROCEDURE FocusItemNum (Item: Sw_Integer); Virtual;
   END;
   PListViewer = ^TListViewer;

{---------------------------------------------------------------------------}
{                  TWindow OBJECT - WINDOW OBJECT ANCESTOR                  }
{---------------------------------------------------------------------------}
TYPE
   TWindow = OBJECT (TGroup)
         Flags   : Byte;                              { Window flags }
         Number  : Sw_Integer;                           { Window number }
         Palette : Sw_Integer;                           { Window palette }
         ZoomRect: TRect;                             { Zoom rectangle }
         Frame   : PFrame;                            { Frame view object }
         Title   : PString;                           { Title string }
      CONSTRUCTOR Init (Var Bounds: TRect; ATitle: TTitleStr; ANumber: Sw_Integer);
      CONSTRUCTOR Load (Var S: TStream);
      DESTRUCTOR Done; Virtual;
      FUNCTION GetPalette: PPalette; Virtual;
      FUNCTION GetTitle (MaxSize: Sw_Integer): TTitleStr; Virtual;
      FUNCTION StandardScrollBar (AOptions: Word): PScrollBar;
      PROCEDURE Zoom; Virtual;
      PROCEDURE Close; Virtual;
      PROCEDURE InitFrame; Virtual;
      PROCEDURE SetState (AState: Word; Enable: Boolean); Virtual;
      PROCEDURE Store (Var S: TStream);
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
      PROCEDURE SizeLimits (Var Min, Max: TPoint); Virtual;
   END;
   PWindow = ^TWindow;

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                         WINDOW MESSAGE ROUTINES                           }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-Message------------------------------------------------------------
Message sets up an event record and calls Receiver^.HandleEvent to
handle the event. Message returns nil if Receiver is nil, or if
the event is not handled successfully.
12Sep97 LdB
---------------------------------------------------------------------}
FUNCTION Message (Receiver: PView; What, Command: Word;
  InfoPtr: Pointer): Pointer;

{-NewMessage---------------------------------------------------------
NewMessage sets up an event record including the new fields and calls
Receiver^.HandleEvent to handle the event. Message returns nil if
Receiver is nil, or if the event is not handled successfully.
19Sep97 LdB
---------------------------------------------------------------------}
FUNCTION NewMessage (P: PView; What, Command: Word; Id: Sw_Integer; Data: Real;
  InfoPtr: Pointer): Pointer;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                     VIEW OBJECT REGISTRATION ROUTINES                     }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{-RegisterViews------------------------------------------------------
This registers all the view type objects used in this unit.
11Aug99 LdB
---------------------------------------------------------------------}
PROCEDURE RegisterViews;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                            NEW VIEW ROUTINES                              }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-CreateIdScrollBar--------------------------------------------------
Creates and scrollbar object of the given size and direction and sets
the scrollbar id number.
22Sep97 LdB
---------------------------------------------------------------------}
FUNCTION CreateIdScrollBar (X, Y, Size, Id: Sw_Integer; Horz: Boolean): PScrollBar;

{***************************************************************************}
{                        INITIALIZED PUBLIC VARIABLES                       }
{***************************************************************************}


{---------------------------------------------------------------------------}
{                 INITIALIZED DOS/DPMI/WIN/NT/OS2 VARIABLES                 }
{---------------------------------------------------------------------------}
CONST
   UseNativeClasses: Boolean = True;                  { Native class modes }
   CommandSetChanged: Boolean = False;                { Command change flag }
   ShowMarkers: Boolean = False;                      { Show marker state }
   ErrorAttr: Byte = $CF;                             { Error colours }
   PositionalEvents: Word = evMouse;                  { Positional defined }
   FocusedEvents: Word = evKeyboard + evCommand;      { Focus defined }
   MinWinSize: TPoint = (X: 16; Y: 6);                { Minimum window size }
   ShadowSize: TPoint = (X: 2; Y: 1);                 { Shadow sizes }
   ShadowAttr: Byte = $08;                            { Shadow attribute }

{ Characters used for drawing selected and default items in  }
{ monochrome color sets                                      }
   SpecialChars: Array [0..5] Of Char = (#175, #174, #26, #27, ' ', ' ');

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        STREAM REGISTRATION RECORDS                        }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{                         TView STREAM REGISTRATION                         }
{---------------------------------------------------------------------------}
CONST
   RView: TStreamRec = (
     ObjType: idView;                                 { Register id = 1 }
     VmtLink: TypeOf(TView);                          { Alt style VMT link }
     Load:    @TView.Load;                            { Object load method }
     Store:   @TView.Store                            { Object store method }
   );

{---------------------------------------------------------------------------}
{                        TFrame STREAM REGISTRATION                         }
{---------------------------------------------------------------------------}
CONST
   RFrame: TStreamRec = (
     ObjType: idFrame;                                { Register id = 2 }
     VmtLink: TypeOf(TFrame);                         { Alt style VMT link }
     Load:    @TFrame.Load;                           { Frame load method }
     Store:   @TFrame.Store                           { Frame store method }
   );

{---------------------------------------------------------------------------}
{                      TScrollBar STREAM REGISTRATION                       }
{---------------------------------------------------------------------------}
CONST
   RScrollBar: TStreamRec = (
     ObjType: idScrollBar;                            { Register id = 3 }
     VmtLink: TypeOf(TScrollBar);                     { Alt style VMT link }
     Load:    @TScrollBar.Load;                       { Object load method }
     Store:   @TScrollBar.Store                       { Object store method }
   );

{---------------------------------------------------------------------------}
{                       TScroller STREAM REGISTRATION                       }
{---------------------------------------------------------------------------}
CONST
   RScroller: TStreamRec = (
     ObjType: idScroller;                             { Register id = 4 }
     VmtLink: TypeOf(TScroller);                      { Alt style VMT link }
     Load:    @TScroller.Load;                        { Object load method }
     Store:   @TScroller.Store                        { Object store method }
   );

{---------------------------------------------------------------------------}
{                      TListViewer STREAM REGISTRATION                      }
{---------------------------------------------------------------------------}
CONST
   RListViewer: TStreamRec = (
     ObjType: idListViewer;                           { Register id = 5 }
     VmtLink: TypeOf(TListViewer);                    { Alt style VMT link }
     Load:    @TListViewer.Load;                      { Object load method }
     Store:   @TLIstViewer.Store                      { Object store method }
   );

{---------------------------------------------------------------------------}
{                        TGroup STREAM REGISTRATION                         }
{---------------------------------------------------------------------------}
CONST
   RGroup: TStreamRec = (
     ObjType: idGroup;                                { Register id = 6 }
     VmtLink: TypeOf(TGroup);                         { Alt style VMT link }
     Load:    @TGroup.Load;                           { Object load method }
     Store:   @TGroup.Store                           { Object store method }
   );

{---------------------------------------------------------------------------}
{                        TWindow STREAM REGISTRATION                        }
{---------------------------------------------------------------------------}
CONST
   RWindow: TStreamRec = (
     ObjType: idWindow;                               { Register id = 7 }
     VmtLink: TypeOf(TWindow);                        { Alt style VMT link }
     Load:    @TWindow.Load;                          { Object load method }
     Store:   @TWindow.Store                          { Object store method }
   );


{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                             IMPLEMENTATION
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

USES
  Video;

{***************************************************************************}
{                       PRIVATE TYPE DEFINITIONS                            }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                         TFixupList DEFINITION                             }
{---------------------------------------------------------------------------}
TYPE
   TFixupList = ARRAY [1..4096] Of Pointer;           { Fix up ptr array }
   PFixupList = ^TFixupList;                          { Ptr to fix up list }

{***************************************************************************}
{                      PRIVATE INITIALIZED VARIABLES                        }
{***************************************************************************}

{---------------------------------------------------------------------------}
{            INITIALIZED DOS/DPMI/WIN/NT/OS2 PRIVATE VARIABLES              }
{---------------------------------------------------------------------------}
CONST
   TheTopView  : PView = Nil;                         { Top focused view }
   LimitsLocked: PView = Nil;                         { View locking limits }
   OwnerGroup  : PGroup = Nil;                        { Used for loading }
   FixupList   : PFixupList = Nil;                    { Used for loading }
   CurCommandSet: TCommandSet = ([0..255] -
     [cmZoom, cmClose, cmResize, cmNext, cmPrev]);    { All active but these }

  vdInSetCursor  = $80;                               { AVOID RECURSION IN SetCursor }

  { Flags for TFrame }
  fmCloseClicked = $01;
  fmZoomClicked  = $02;


type
  TstatVar2 = record
    target : PView;
    offset,y : integer;
  end;

var
  staticVar1 : PDrawBuffer;
  staticVar2 : TstatVar2;


{***************************************************************************}
{                          PRIVATE INTERNAL ROUTINES                        }
{***************************************************************************}

    function posidx(const substr,s : string;idx:sw_integer):sw_integer;
      var
        i,j : sw_integer;
        e   : boolean;
      begin
        i:=idx;
        j:=0;
        e:=(length(SubStr)>0);
        while e and (i<=Length(s)-Length(SubStr)) do
         begin
           if (SubStr[1]=s[i]) and (Substr=Copy(s,i,Length(SubStr))) then
            begin
              j:=i;
              e:=false;
            end;
           inc(i);
         end;
        PosIdx:=j;
      end;


{$ifdef UNIX}
const
  MouseUsesVideoBuf = true;
{$else not UNIX}
const
  MouseUsesVideoBuf = false;
{$endif not UNIX}

procedure DrawScreenBuf(force:boolean);
begin
  if (GetLockScreenCount=0) then
   begin
{     If MouseUsesVideoBuf then
       begin
         LockScreenUpdate;
         HideMouse;
         ShowMouse;
         UnlockScreenUpdate;
       end
     else
       HideMouse;}
     UpdateScreen(force);
{     If not MouseUsesVideoBuf then
       ShowMouse;}
   end;
end;


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                         VIEW PORT CONTROL ROUTINES                        }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

TYPE
   ViewPortType = RECORD
     X1, Y1, X2, Y2: Integer;                         { Corners of viewport }
     Clip          : Boolean;                         { Clip status }
   END;

var
  ViewPort : ViewPortType;

{---------------------------------------------------------------------------}
{  GetViewSettings -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 05Dec2000 LdB }
{---------------------------------------------------------------------------}
PROCEDURE GetViewSettings (Var CurrentViewPort: ViewPortType);
BEGIN
   CurrentViewPort := ViewPort;                       { Textmode viewport }
END;

{---------------------------------------------------------------------------}
{  SetViewPort -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 05Dec2000 LdB     }
{---------------------------------------------------------------------------}
PROCEDURE SetViewPort (X1, Y1, X2, Y2: Integer; Clip: Boolean);
BEGIN
     If (X1 < 0) Then X1 := 0;                        { X1 negative fix }
     If (X1 >ScreenWidth) Then
       X1 := ScreenWidth;                             { X1 off screen fix }
     If (Y1 < 0) Then Y1 := 0;                        { Y1 negative fix }
     If (Y1 > ScreenHeight) Then
       Y1 := ScreenHeight;                            { Y1 off screen fix }
     If (X2 < 0) Then X2 := 0;                        { X2 negative fix }
     If (X2 > ScreenWidth) Then
       X2 := ScreenWidth;                             { X2 off screen fix }
     If (Y2 < 0) Then Y2 := 0;                        { Y2 negative fix }
     If (Y2 > ScreenHeight) Then
       Y2 := ScreenHeight;                            { Y2 off screen fix }
     ViewPort.X1 := X1;                               { Set X1 port value }
     ViewPort.Y1 := Y1;                               { Set Y1 port value }
     ViewPort.X2 := X2;                               { Set X2 port value }
     ViewPort.Y2 := Y2;                               { Set Y2 port value }
     ViewPort.Clip := Clip;                           { Set port clip value }
{ $ifdef DEBUG
     If WriteDebugInfo then
       Writeln(stderr,'New ViewPort(',X1,',',Y1,',',X2,',',Y2,')');
 $endif DEBUG}
END;

{***************************************************************************}
{                              OBJECT METHODS                               }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                            TView OBJECT METHODS                           }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TView--------------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 20Jun96 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TView.Init (Var Bounds: TRect);
BEGIN
   Inherited Init;                                    { Call ancestor }
   DragMode := dmLimitLoY;                            { Default drag mode }
   HelpCtx := hcNoContext;                            { Clear help context }
   State := sfVisible;                                { Default state }
   EventMask := evMouseDown + evKeyDown + evCommand;  { Default event masks }
   BackgroundChar := ' ';
   SetBounds(Bounds);                                 { Set view bounds }
END;

{--TView--------------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 06May98 LdB              }
{---------------------------------------------------------------------------}
{   This load method will read old original TV data from a stream but the   }
{  new options and tabmasks are not set so some NEW functionality is not    }
{  supported but it should work as per original TV code.                    }
{---------------------------------------------------------------------------}
CONSTRUCTOR TView.Load (Var S: TStream);
VAR i: Integer;
BEGIN
   Inherited Init;                                    { Call ancestor }
   S.Read(i, SizeOf(i)); Origin.X:=i;                 { Read origin x value }
   S.Read(i, SizeOf(i)); Origin.Y:=i;                 { Read origin y value }
   S.Read(i, SizeOf(i)); Size.X:=i;                   { Read view x size }
   S.Read(i, SizeOf(i)); Size.Y:=i;                   { Read view y size }
   S.Read(i, SizeOf(i)); Cursor.X:=i;                 { Read cursor x size }
   S.Read(i, SizeOf(i)); Cursor.Y:=i;                 { Read cursor y size }
   S.Read(GrowMode, SizeOf(GrowMode));                { Read growmode flags }
   S.Read(DragMode, SizeOf(DragMode));                { Read dragmode flags }
   S.Read(HelpCtx, SizeOf(HelpCtx));                  { Read help context }
   S.Read(State, SizeOf(State));                      { Read state masks }
   S.Read(Options, SizeOf(Options));                  { Read options masks }
   S.Read(Eventmask, SizeOf(Eventmask));              { Read event masks }
END;

{--TView--------------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Nov99 LdB              }
{---------------------------------------------------------------------------}
DESTRUCTOR TView.Done;
VAR P: PComplexArea;
BEGIN
   Hide;                                              { Hide the view }
   If (Owner <> Nil) Then Owner^.Delete(@Self);       { Delete from owner }
   While (HoldLimit <> Nil) Do Begin                  { Free limit memory }
     P := HoldLimit^.NextArea;                        { Hold next pointer }
     FreeMem(HoldLimit, SizeOf(TComplexArea));        { Release memory }
     HoldLimit := P;                                  { Shuffle to next }
   End;
END;

{--TView--------------------------------------------------------------------}
{  Prev -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TView.Prev: PView;
VAR NP : PView;
BEGIN
   Prev := @Self;
   NP := Next;
   While (NP <> Nil) AND (NP <> @Self) Do
     Begin
       Prev := NP;                                       { Locate next view }
       NP := NP^.Next;
     End;
END;

{--TView--------------------------------------------------------------------}
{  Execute -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB           }
{---------------------------------------------------------------------------}
FUNCTION TView.Execute: Word;
BEGIN
   Execute := cmCancel;                               { Return cancel }
END;

{--TView--------------------------------------------------------------------}
{  Focus -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 05May98 LdB             }
{---------------------------------------------------------------------------}
FUNCTION TView.Focus: Boolean;
VAR Res: Boolean;
BEGIN
   Res := True;                                       { Preset result }
   If (State AND (sfSelected + sfModal)=0) Then Begin { Not modal/selected }
     If (Owner <> Nil) Then Begin                     { View has an owner }
       Res := Owner^.Focus;                           { Return focus state }
       If Res Then                                    { Owner has focus }
         If ((Owner^.Current = Nil) OR                { No current view }
         (Owner^.Current^.Options AND ofValidate = 0) { Non validating view }
         OR (Owner^.Current^.Valid(cmReleasedFocus))) { Okay to drop focus }
           Then Select Else Res := False;             { Then select us }
     End;
   End;
   Focus := Res;                                      { Return focus result }
END;

{--TView--------------------------------------------------------------------}
{  DataSize -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
FUNCTION TView.DataSize: Sw_Word;
BEGIN
   DataSize := 0;                                     { Transfer size }
END;

{--TView--------------------------------------------------------------------}
{  TopView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB           }
{---------------------------------------------------------------------------}
FUNCTION TView.TopView: PView;
VAR P: PView;
BEGIN
   If (TheTopView = Nil) Then Begin                   { Check topmost view }
     P := @Self;                                      { Start with us }
     While (P <> Nil) AND (P^.State AND sfModal = 0)  { Check if modal }
       Do P := P^.Owner;                              { Search each owner }
     TopView := P;                                    { Return result }
   End Else TopView := TheTopView;                    { Return topview }
END;

{--TView--------------------------------------------------------------------}
{  PrevView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
FUNCTION TView.PrevView: PView;
BEGIN
   If (@Self = Owner^.First) Then PrevView := Nil     { We are first view }
     Else PrevView := Prev;                           { Return our prior }
END;

{--TView--------------------------------------------------------------------}
{  NextView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
FUNCTION TView.NextView: PView;
BEGIN
   If (@Self = Owner^.Last) Then NextView := Nil      { This is last view }
     Else NextView := Next;                           { Return our next }
END;

{--TView--------------------------------------------------------------------}
{  GetHelpCtx -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TView.GetHelpCtx: Word;
BEGIN
   If (State AND sfDragging <> 0) Then                { Dragging state check }
     GetHelpCtx := hcDragging Else                    { Return dragging }
     GetHelpCtx := HelpCtx;                           { Return help context }
END;

{--TView--------------------------------------------------------------------}
{  EventAvail -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TView.EventAvail: Boolean;
VAR Event: TEvent;
BEGIN
   GetEvent(Event);                                   { Get next event }
   If (Event.What <> evNothing) Then PutEvent(Event); { Put it back }
   EventAvail := (Event.What <> evNothing);           { Return result }
END;

{--TView--------------------------------------------------------------------}
{  GetPalette -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TView.GetPalette: PPalette;
BEGIN
   GetPalette := Nil;                                 { Return nil ptr }
END;

{--TView--------------------------------------------------------------------}
{  MapColor -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Jul99 LdB          }
{---------------------------------------------------------------------------}
function TView.MapColor(color:byte):byte;
var
  cur : PView;
  p   : PPalette;
begin
  if color=0 then
   MapColor:=errorAttr
  else
   begin
     cur:=@Self;
     repeat
       p:=cur^.GetPalette;
       if (p<>Nil) then
        if ord(p^[0])<>0 then
         begin
           if color>ord(p^[0]) then
            begin
              MapColor:=errorAttr;
              Exit;
            end;
           color:=ord(p^[color]);
           if color=0 then
            begin
              MapColor:=errorAttr;
              Exit;
            end;
         end;
       cur:=cur^.Owner;
     until (cur=Nil);
     MapColor:=color;
   end;
end;


{--TView--------------------------------------------------------------------}
{  GetColor -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Jul99 LdB          }
{---------------------------------------------------------------------------}
FUNCTION TView.GetColor (Color: Word): Word;
VAR Col: Byte; W: Word; P: PPalette; Q: PView;
BEGIN
   W := 0;                                            { Clear colour Sw_Word }
   If (Hi(Color) > 0) Then Begin                      { High colour req }
     Col := Hi(Color) + ColourOfs;                    { Initial offset }
     Q := @Self;                                      { Pointer to self }
     Repeat
       P := Q^.GetPalette;                            { Get our palette }
       If (P <> Nil) Then Begin                       { Palette is valid }
         If (Col <= Length(P^)) Then
           Col := Ord(P^[Col]) Else                   { Return colour }
           Col := ErrorAttr;                          { Error attribute }
       End;
       Q := Q^.Owner;                                 { Move up to owner }
     Until (Q = Nil);                                 { Until no owner }
     W := Col SHL 8;                                  { Translate colour }
   End;
   If (Lo(Color) > 0) Then Begin
     Col := Lo(Color) + ColourOfs;                    { Initial offset }
     Q := @Self;                                      { Pointer to self }
     Repeat
       P := Q^.GetPalette;                            { Get our palette }
       If (P <> Nil) Then Begin                       { Palette is valid }
         If (Col <= Length(P^)) Then
           Col := Ord(P^[Col]) Else                   { Return colour }
           Col := ErrorAttr;                          { Error attribute }
       End;
       Q := Q^.Owner;                                 { Move up to owner }
     Until (Q = Nil);                                 { Until no owner }
   End Else Col := ErrorAttr;                         { No colour found }
   GetColor := W OR Col;                              { Return color }
END;

{--TView--------------------------------------------------------------------}
{  Valid -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB             }
{---------------------------------------------------------------------------}
FUNCTION TView.Valid (Command: Word): Boolean;
BEGIN
   Valid := True;                                     { Simply return true }
END;

{--TView--------------------------------------------------------------------}
{  GetState -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
FUNCTION TView.GetState (AState: Word): Boolean;
BEGIN
   GetState := State AND AState = AState;             { Check states equal }
END;

{--TView--------------------------------------------------------------------}
{  TextWidth -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25Nov99 LdB         }
{---------------------------------------------------------------------------}
FUNCTION TView.TextWidth (const Txt: String): Sw_Integer;
BEGIN
   TextWidth := Length(Txt);             { Calc text length }
END;

FUNCTION TView.CTextWidth (const Txt: String): Sw_Integer;
VAR I: Sw_Integer; S: String;
BEGIN
   S := Txt;                                          { Transfer text }
   Repeat
     I := Pos('~', S);                                { Check for tilde }
      If (I <> 0) Then System.Delete(S, I, 1);        { Remove the tilde }
   Until (I = 0);                                     { Remove all tildes }
   CTextWidth := Length(S);             { Calc text length }
END;

{--TView--------------------------------------------------------------------}
{  MouseInView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
FUNCTION TView.MouseInView (Point: TPoint): Boolean;
BEGIN
  MakeLocal(Point,Point);
  MouseInView := (Point.X >= 0) and
                 (Point.Y >= 0) and
                 (Point.X < Size.X) and
                 (Point.Y < Size.Y);
END;

{--TView--------------------------------------------------------------------}
{  CommandEnabled -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB    }
{---------------------------------------------------------------------------}
FUNCTION TView.CommandEnabled(Command: Word): Boolean;
BEGIN
   CommandEnabled := (Command > 255) OR
     (Command IN CurCommandSet);                      { Check command }
END;

{--TView--------------------------------------------------------------------}
{  OverLapsArea -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Sep97 LdB      }
{---------------------------------------------------------------------------}
FUNCTION TView.OverlapsArea (X1, Y1, X2, Y2: Sw_Integer): Boolean;
BEGIN
   OverLapsArea := False;                             { Preset false }
   If (Origin.X > X2) Then Exit;                   { Area to the left }
   If ((Origin.X + Size.X) < X1) Then Exit;     { Area to the right }
   If (Origin.Y > Y2) Then Exit;                   { Area is above }
   If ((Origin.Y + Size.Y) < Y1) Then Exit;     { Area is below }
   OverLapsArea := True;                              { Return true }
END;

{--TView--------------------------------------------------------------------}
{  MouseEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TView.MouseEvent (Var Event: TEvent; Mask: Word): Boolean;
BEGIN
   Repeat
     GetEvent(Event);                                 { Get next event }
   Until (Event.What AND (Mask OR evMouseUp) <> 0);   { Wait till valid }
   MouseEvent := Event.What <> evMouseUp;             { Return result }
END;

{--TView--------------------------------------------------------------------}
{  Hide -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TView.Hide;
BEGIN
   If (State AND sfVisible <> 0) Then                 { View is visible }
     SetState(sfVisible, False);                      { Hide the view }
END;

{--TView--------------------------------------------------------------------}
{  Show -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TView.Show;
BEGIN
   If (State AND sfVisible = 0) Then                  { View not visible }
     SetState(sfVisible, True);                       { Show the view }
END;

{--TView--------------------------------------------------------------------}
{  Draw -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Sep97 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TView.Draw;
VAR B : TDrawBuffer;
BEGIN
  MoveChar(B, ' ', GetColor(1), Size.X);
  WriteLine(0, 0, Size.X, Size.Y, B);
END;


procedure TView.ResetCursor;
const
  sfV_CV_F:word = sfVisible + sfCursorVis + sfFocused;
var
  p,p2 : PView;
  G : PGroup;
  cur : TPoint;

  function Check0:boolean;
  var
    res : byte;
  begin
    res:=0;
    while res=0 do
     begin
       p:=p^.next;
       if p=p2 then
        begin
          p:=P^.owner;
          res:=1
        end
       else
        if ((p^.state and sfVisible)<>0) and
           (cur.x>=p^.origin.x) and
           (cur.x<p^.size.x+p^.origin.x) and
           (cur.y>=p^.origin.y) and
           (cur.y<p^.size.y+p^.origin.y) then
          res:=2;
     end;
    Check0:=res=2;
  end;

begin
  if ((state and sfV_CV_F) = sfV_CV_F) then
   begin
     p:=@Self;
     cur:=cursor;
     while true do
      begin
        if (cur.x<0) or (cur.x>=p^.size.x) or
           (cur.y<0) or (cur.y>=p^.size.y) then
          break;
        inc(cur.X,p^.origin.X);
        inc(cur.Y,p^.origin.Y);
        p2:=p;
        G:=p^.owner;
        if G=Nil then { top view }
         begin
           Video.SetCursorPos(cur.x,cur.y);
           if (state and sfCursorIns)<>0 then
             Video.SetCursorType(crBlock)
           else
             Video.SetCursorType(crUnderline);
           exit;
         end;
        if (G^.state and sfVisible)=0 then
         break;
        p:=G^.Last;
        if Check0 then
         break;
      end; { while }
   end; { if }
  Video.SetCursorType(crHidden);
end;


{--TView--------------------------------------------------------------------}
{  Select -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 05May98 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TView.Select;
BEGIN
   If (Options AND ofSelectable <> 0) Then            { View is selectable }
     If (Options AND ofTopSelect <> 0) Then MakeFirst { Top selectable }
     Else If (Owner <> Nil) Then                      { Valid owner }
       Owner^.SetCurrent(@Self, NormalSelect);        { Make owners current }
END;

{--TView--------------------------------------------------------------------}
{  Awaken -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TView.Awaken;
BEGIN                                                 { Abstract method }
END;


{--TView--------------------------------------------------------------------}
{  MakeFirst -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 29Sep99 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TView.MakeFirst;
BEGIN
   If (Owner <> Nil) Then Begin                       { Must have owner }
     PutInFrontOf(Owner^.First);                      { Float to the top }
   End;
END;

{--TView--------------------------------------------------------------------}
{  DrawCursor -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TView.DrawCursor;
BEGIN                                                 { Abstract method }
  if State and sfFocused <> 0 then
    ResetCursor;
END;


procedure TView.DrawHide(LastView: PView);
begin
  TView.DrawCursor;
  DrawUnderView(State and sfShadow <> 0, LastView);
end;


procedure TView.DrawShow(LastView: PView);
begin
  DrawView;
  if State and sfShadow <> 0 then
   DrawUnderView(True, LastView);
end;


procedure TView.DrawUnderRect(var R: TRect; LastView: PView);
begin
  Owner^.Clip.Intersect(R);
  Owner^.DrawSubViews(NextView, LastView);
  Owner^.GetExtent(Owner^.Clip);
end;


procedure TView.DrawUnderView(DoShadow: Boolean; LastView: PView);
var
  R: TRect;
begin
  GetBounds(R);
  if DoShadow then
   begin
     inc(R.B.X,ShadowSize.X);
     inc(R.B.Y,ShadowSize.Y);
   end;
  DrawUnderRect(R, LastView);
end;


procedure TView.DrawView;
begin
  if Exposed then
   begin
     LockScreenUpdate; { don't update the screen yet }
     Draw;
     UnLockScreenUpdate;
     DrawScreenBuf(false);
     TView.DrawCursor;
   end;
end;


{--TView--------------------------------------------------------------------}
{  HideCursor -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TView.HideCursor;
BEGIN
   SetState(sfCursorVis , False);                     { Hide the cursor }
END;

{--TView--------------------------------------------------------------------}
{  ShowCursor -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TView.ShowCursor;
BEGIN
   SetState(sfCursorVis , True);                      { Show the cursor }
END;

{--TView--------------------------------------------------------------------}
{  BlockCursor -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TView.BlockCursor;
BEGIN
   SetState(sfCursorIns, True);                       { Set insert mode }
END;

{--TView--------------------------------------------------------------------}
{  NormalCursor -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TView.NormalCursor;
BEGIN
   SetState(sfCursorIns, False);                      { Clear insert mode }
END;

{--TView--------------------------------------------------------------------}
{  FocusFromTop -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 11Aug99 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TView.FocusFromTop;
BEGIN
   If (Owner <> Nil) AND
     (Owner^.State AND sfSelected = 0)
       Then Owner^.Select;
   If (State AND sfFocused = 0) Then Focus;
   If (State AND sfSelected = 0) Then Select;
END;

{--TView--------------------------------------------------------------------}
{  MoveTo -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TView.MoveTo (X, Y: Sw_Integer);
VAR R: TRect;
BEGIN
   R.Assign(X, Y, X + Size.X, Y + Size.Y);            { Assign area }
   Locate(R);                                         { Locate the view }
END;

{--TView--------------------------------------------------------------------}
{  GrowTo -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TView.GrowTo (X, Y: Sw_Integer);
VAR R: TRect;
BEGIN
   R.Assign(Origin.X, Origin.Y, Origin.X + X,
     Origin.Y + Y);                                   { Assign area }
   Locate(R);                                         { Locate the view }
END;

{--TView--------------------------------------------------------------------}
{  EndModal -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TView.EndModal (Command: Word);
VAR P: PView;
BEGIN
   P := TopView;                                      { Get top view }
   If (P <> Nil) Then P^.EndModal(Command);           { End modal operation }
END;

{--TView--------------------------------------------------------------------}
{  SetCursor -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TView.SetCursor (X, Y: Sw_Integer);
BEGIN
  if (Cursor.X<>X) or (Cursor.Y<>Y) then
  begin
    Cursor.X := X;
    Cursor.Y := Y;
    CursorChanged;
  end;
  TView.DrawCursor;
END;


procedure TView.CursorChanged;
begin
  Message(Owner,evBroadcast,cmCursorChanged,@Self);
end;


{--TView--------------------------------------------------------------------}
{  PutInFrontOf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 29Sep99 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TView.PutInFrontOf (Target: PView);
VAR P, LastView: PView;
BEGIN
   If (Owner <> Nil) AND (Target <> @Self) AND
   (Target <> NextView) AND ((Target = Nil) OR
   (Target^.Owner = Owner)) Then                      { Check validity }
     If (State AND sfVisible = 0) Then Begin          { View not visible }
       Owner^.RemoveView(@Self);                      { Remove from list }
       Owner^.InsertView(@Self, Target);              { Insert into list }
     End Else Begin
       LastView := NextView;                          { Hold next view }
       If (LastView <> Nil) Then Begin                { Lastview is valid }
         P := Target;                                 { P is target }
         While (P <> Nil) AND (P <> LastView)
           Do P := P^.NextView;                       { Find our next view }
         If (P = Nil) Then LastView := Target;        { Lastview is target }
       End;
       State := State AND NOT sfVisible;              { Temp stop drawing }
       If (LastView = Target) Then
         DrawHide(LastView);
       Owner^.Lock;
       Owner^.RemoveView(@Self);                      { Remove from list }
       Owner^.InsertView(@Self, Target);              { Insert into list }
       State := State OR sfVisible;                   { Allow drawing again }
       If (LastView <> Target) Then
         DrawShow(LastView);
       If (Options AND ofSelectable <> 0) Then        { View is selectable }
         begin
           Owner^.ResetCurrent;  { Reset current }
           Owner^.ResetCursor;
         end;
       Owner^.Unlock;
     End;
END;

{--TView--------------------------------------------------------------------}
{  SetCommands -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TView.SetCommands (Commands: TCommandSet);
BEGIN
   CommandSetChanged := CommandSetChanged OR
     (CurCommandSet <> Commands);                     { Set change flag }
   CurCommandSet := Commands;                         { Set command set }
END;

{--TView--------------------------------------------------------------------}
{  EnableCommands -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB    }
{---------------------------------------------------------------------------}
PROCEDURE TView.EnableCommands (Commands: TCommandSet);
BEGIN
   CommandSetChanged := CommandSetChanged OR
     (CurCommandSet * Commands <> Commands);          { Set changed flag }
   CurCommandSet := CurCommandSet + Commands;         { Update command set }
END;

{--TView--------------------------------------------------------------------}
{  DisableCommands -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB   }
{---------------------------------------------------------------------------}
PROCEDURE TView.DisableCommands (Commands: TCommandSet);
BEGIN
   CommandSetChanged := CommandSetChanged OR
     (CurCommandSet * Commands <> []);                { Set changed flag }
   CurCommandSet := CurCommandSet - Commands;         { Update command set }
END;

{--TView--------------------------------------------------------------------}
{  SetState -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 23Sep99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TView.SetState (AState: Word; Enable: Boolean);
var
  Command: Word;
  OState : Word;
begin
  OState:=State;
  if Enable then
    State := State or AState
  else
    State := State and not AState;
  if Owner <> nil then
    case AState of
      sfVisible:
        begin
          if Owner^.State and sfExposed <> 0 then
            SetState(sfExposed, Enable);
          if Enable then
            DrawShow(nil)
          else
            DrawHide(nil);
          if Options and ofSelectable <> 0 then
            Owner^.ResetCurrent;
        end;
      sfCursorVis,
      sfCursorIns:
        TView.DrawCursor;
      sfShadow:
        DrawUnderView(True, nil);
      sfFocused:
        begin
          ResetCursor;
          if Enable then
            Command := cmReceivedFocus
          else
            Command := cmReleasedFocus;
          Message(Owner, evBroadcast, Command, @Self);
        end;
    end;
  if ((OState xor State) and (sfCursorVis+sfCursorIns+sfFocused))<>0 then
    CursorChanged;
end;


{--TView--------------------------------------------------------------------}
{  SetCmdState -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TView.SetCmdState (Commands: TCommandSet; Enable: Boolean);
BEGIN
   If Enable Then EnableCommands(Commands)            { Enable commands }
     Else DisableCommands(Commands);                  { Disable commands }
END;

{--TView--------------------------------------------------------------------}
{  GetData -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TView.GetData (Var Rec);
BEGIN                                                 { Abstract method }
END;

{--TView--------------------------------------------------------------------}
{  SetData -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TView.SetData (Var Rec);
BEGIN                                                 { Abstract method }
END;

{--TView--------------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 06May98 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TView.Store (Var S: TStream);
VAR SaveState: Word;
    i: integer;
BEGIN
   SaveState := State;                                { Hold current state }
   State := State AND NOT (sfActive OR sfSelected OR
     sfFocused OR sfExposed);                         { Clear flags }
   i:=Origin.X;S.Write(i, SizeOf(i));                 { Write view x origin }
   i:=Origin.Y;S.Write(i, SizeOf(i));                 { Write view y origin }
   i:=Size.X;S.Write(i, SizeOf(i));                   { Write view x size }
   i:=Size.Y;S.Write(i, SizeOf(i));                   { Write view y size }
   i:=Cursor.X;S.Write(i, SizeOf(i));                 { Write cursor x size }
   i:=Cursor.Y;S.Write(i, SizeOf(i));                 { Write cursor y size }
   S.Write(GrowMode, SizeOf(GrowMode));               { Write growmode flags }
   S.Write(DragMode, SizeOf(DragMode));               { Write dragmode flags }
   S.Write(HelpCtx, SizeOf(HelpCtx));                 { Write help context }
   S.Write(State, SizeOf(State));                     { Write state masks }
   S.Write(Options, SizeOf(Options));                 { Write options masks }
   S.Write(Eventmask, SizeOf(Eventmask));             { Write event masks }
   State := SaveState;                                { Reset state masks }
END;

{--TView--------------------------------------------------------------------}
{  Locate -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 24Sep99 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TView.Locate (Var Bounds: TRect);
VAR
    Min, Max: TPoint; R: TRect;

   FUNCTION Range(Val, Min, Max: Sw_Integer): Sw_Integer;
   BEGIN
     If (Val < Min) Then Range := Min Else            { Value to small }
       If (Val > Max) Then Range := Max Else          { Value to large }
         Range := Val;                                { Value is okay }
   END;

BEGIN
   SizeLimits(Min, Max);                              { Get size limits }
   Bounds.B.X := Bounds.A.X + Range(Bounds.B.X -
     Bounds.A.X, Min.X, Max.X);                       { X bound limit }
   Bounds.B.Y := Bounds.A.Y + Range(Bounds.B.Y
     - Bounds.A.Y, Min.Y, Max.Y);                     { Y bound limit }
   GetBounds(R);                                      { Current bounds }
   If NOT Bounds.Equals(R) Then Begin                 { Size has changed }
     ChangeBounds(Bounds);                            { Change bounds }
     If (State AND sfVisible <> 0) AND                { View is visible }
     (State AND sfExposed <> 0) AND (Owner <> Nil)    { Check view exposed }
       Then
      begin
        if State and sfShadow <> 0 then
          begin
            R.Union(Bounds);
            Inc(R.B.X, ShadowSize.X);
            Inc(R.B.Y, ShadowSize.Y);
          end;
        DrawUnderRect(R, nil);
      end;
   End;
END;

{--TView--------------------------------------------------------------------}
{  KeyEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TView.KeyEvent (Var Event: TEvent);
BEGIN
   Repeat
     GetEvent(Event);                                 { Get next event }
   Until (Event.What = evKeyDown);                    { Wait till keydown }
END;

{--TView--------------------------------------------------------------------}
{  GetEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TView.GetEvent (Var Event: TEvent);
BEGIN
  If (Owner <> Nil) Then Owner^.GetEvent(Event);      { Event from owner }
END;

{--TView--------------------------------------------------------------------}
{  PutEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TView.PutEvent (Var Event: TEvent);
BEGIN
   If (Owner <> Nil) Then Owner^.PutEvent(Event);     { Put in owner }
END;

{--TView--------------------------------------------------------------------}
{  GetExtent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TView.GetExtent (Var Extent: TRect);
BEGIN
   Extent.A.X := 0;                                   { Zero x field }
   Extent.A.Y := 0;                                   { Zero y field }
   Extent.B.X := Size.X;                              { Return x size }
   Extent.B.Y := Size.Y;                              { Return y size }
END;

{--TView--------------------------------------------------------------------}
{  GetBounds -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TView.GetBounds (Var Bounds: TRect);
BEGIN
   Bounds.A := Origin;                                { Get first corner }
   Bounds.B.X := Origin.X + Size.X;                   { Calc corner x value }
   Bounds.B.Y := Origin.Y + Size.Y;                   { Calc corner y value }
END;

{--TView--------------------------------------------------------------------}
{  SetBounds -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 24Sep99 LdB         }
{---------------------------------------------------------------------------}
procedure TView.SetBounds(var Bounds: TRect);
begin
  Origin := Bounds.A;                                { Get first corner }
  Size := Bounds.B;                                 { Get second corner }
  Dec(Size.X,Origin.X);
  Dec(Size.Y,Origin.Y);
end;

{--TView--------------------------------------------------------------------}
{  GetClipRect -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TView.GetClipRect (Var Clip: TRect);
BEGIN
   GetBounds(Clip);                                   { Get current bounds }
   If (Owner <> Nil) Then Clip.Intersect(Owner^.Clip);{ Intersect with owner }
   Clip.Move(-Origin.X, -Origin.Y);                   { Sub owner origin }
END;

{--TView--------------------------------------------------------------------}
{  ClearEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TView.ClearEvent (Var Event: TEvent);
BEGIN
   Event.What := evNothing;                           { Clear the event }
   Event.InfoPtr := @Self;                            { Set us as handler }
END;

{--TView--------------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TView.HandleEvent (Var Event: TEvent);
BEGIN
   If (Event.What = evMouseDown) Then                 { Mouse down event }
     If (State AND (sfSelected OR sfDisabled) = 0)    { Not selected/disabled }
       AND (Options AND ofSelectable <> 0) Then       { View is selectable }
       If (Focus = False) OR                          { Not view with focus }
         (Options AND ofFirstClick = 0)               { Not 1st click select }
           Then ClearEvent(Event);                    { Handle the event }
END;

{--TView--------------------------------------------------------------------}
{  ChangeBounds -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TView.ChangeBounds (Var Bounds: TRect);
BEGIN
   SetBounds(Bounds);                                 { Set new bounds }
   DrawView;                                          { Draw the view }
END;

{--TView--------------------------------------------------------------------}
{  SizeLimits -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TView.SizeLimits (Var Min, Max: TPoint);
BEGIN
   Min.X := 0;                                        { Zero x minimum }
   Min.Y := 0;                                        { Zero y minimum }
   If (Owner <> Nil) and(Owner^.ClipChilds) Then
     Max := Owner^.Size
   else                         { Max owner size }
    Begin
     Max.X := high(sw_integer);                      { Max possible x size }
     Max.Y := high(sw_integer);                        { Max possible y size }
   End;
END;

{--TView--------------------------------------------------------------------}
{  GetCommands -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TView.GetCommands (Var Commands: TCommandSet);
BEGIN
   Commands := CurCommandSet;                         { Return command set }
END;

{--TView--------------------------------------------------------------------}
{  GetPeerViewPtr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB    }
{---------------------------------------------------------------------------}
PROCEDURE TView.GetPeerViewPtr (Var S: TStream; Var P);
VAR Index: Integer;
BEGIN
   Index := 0;                                        { Zero index value }
   S.Read(Index, SizeOf(Index));                      { Read view index }
   If (Index = 0) OR (OwnerGroup = Nil) Then          { Check for peer views }
     Pointer(P) := Nil Else Begin                     { Return nil }
       Pointer(P) := FixupList^[Index];               { New view ptr }
       FixupList^[Index] := @P;                       { Patch this pointer }
     End;
END;

{--TView--------------------------------------------------------------------}
{  PutPeerViewPtr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB    }
{---------------------------------------------------------------------------}
PROCEDURE TView.PutPeerViewPtr (Var S: TStream; P: PView);
VAR Index: Integer;
BEGIN
   If (P = Nil) OR (OwnerGroup = Nil) Then Index := 0 { Return zero index }
     Else Index := OwnerGroup^.IndexOf(P);            { Return view index }
   S.Write(Index, SizeOf(Index));                     { Write the index }
END;

{--TView--------------------------------------------------------------------}
{  CalcBounds -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TView.CalcBounds (Var Bounds: Objects.TRect; Delta: TPoint);
VAR S, D: Sw_Integer; Min, Max: TPoint;

   FUNCTION Range (Val, Min, Max: Sw_Integer): Sw_Integer;
   BEGIN
     If (Val < Min) Then Range := Min Else            { Value below min }
     If (Val > Max) Then Range := Max Else            { Value above max }
       Range := Val;                                  { Accept value }
   END;

   PROCEDURE GrowI (Var I: Sw_Integer);
   BEGIN
     If (GrowMode AND gfGrowRel = 0) Then Inc(I, D)
       Else If  S = D then I := 1
         Else I := (I * S + (S - D) SHR 1) DIV (S - D); { Calc grow value }
   END;

BEGIN
   GetBounds(Bounds);                                 { Get bounds }
   If (GrowMode = 0) Then Exit;                       { No grow flags exits }
   S := Owner^.Size.X;                                { Set initial size }
   D := Delta.X;                                      { Set initial delta }
   If (GrowMode AND gfGrowLoX <> 0) Then
     GrowI(Bounds.A.X);                                { Grow left side }
   If (GrowMode AND gfGrowHiX <> 0) Then
     GrowI(Bounds.B.X);                                { Grow right side }
   If (Bounds.B.X - Bounds.A.X > MaxViewWidth) Then
     Bounds.B.X := Bounds.A.X + MaxViewWidth;         { Check values }
   S := Owner^.Size.Y; D := Delta.Y;                  { set initial values }
   If (GrowMode AND gfGrowLoY <> 0) Then
     GrowI(Bounds.A.Y);                                { Grow top side }
   If (GrowMode AND gfGrowHiY <> 0) Then
     GrowI(Bounds.B.Y);                                { grow lower side }
   SizeLimits(Min, Max);                              { Check sizes }
   Bounds.B.X := Bounds.A.X + Range(Bounds.B.X -
     Bounds.A.X, Min.X, Max.X);                       { Set right side }
   Bounds.B.Y := Bounds.A.Y + Range(Bounds.B.Y -
     Bounds.A.Y, Min.Y, Max.Y);                       { Set lower side }
END;

{***************************************************************************}
{                       TView OBJECT PRIVATE METHODS                        }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          TGroup OBJECT METHODS                            }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TGroup-------------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Jul99 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TGroup.Init (Var Bounds: TRect);
BEGIN
   Inherited Init(Bounds);                            { Call ancestor }
   Options := Options OR (ofSelectable + ofBuffered); { Set options }
   GetExtent(Clip);                                   { Get clip extents }
   EventMask := $FFFF;                                { See all events }
END;

{--TGroup-------------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TGroup.Load (Var S: TStream);
VAR I: Sw_Word;
    Count: Word;
    P, Q: ^Pointer; V: PView; OwnerSave: PGroup;
    FixupSave: PFixupList;
BEGIN
   Inherited Load(S);                                 { Call ancestor }
   GetExtent(Clip);                                   { Get view extents }
   OwnerSave := OwnerGroup;                           { Save current group }
   OwnerGroup := @Self;                               { We are current group }
   FixupSave := FixupList;                            { Save current list }
   Count := 0;                                        { Zero count value }
   S.Read(Count, SizeOf(Count));                      { Read entry count }
   If (MaxAvail >= Count*SizeOf(Pointer)) Then Begin  { Memory available }
     GetMem(FixupList, Count*SizeOf(Pointer));        { List size needed }
     FillChar(FixUpList^, Count*SizeOf(Pointer), #0); { Zero all entries }
     For I := 1 To Count Do Begin
       V := PView(S.Get);                             { Get view off stream }
       If (V <> Nil) Then InsertView(V, Nil);         { Insert valid views }
     End;
     V := Last;                                       { Start on last view }
     For I := 1 To Count Do Begin
       V := V^.Next;                                  { Fetch next view }
       P := FixupList^[I];                            { Transfer pointer }
       While (P <> Nil) Do Begin                      { If valid view }
         Q := P;                                      { Copy pointer }
         P := P^;                                     { Fetch pointer }
         Q^ := V;                                     { Transfer view ptr }
       End;
     End;
     FreeMem(FixupList, Count*SizeOf(Pointer));       { Release fixup list }
   End;
   OwnerGroup := OwnerSave;                           { Reload current group }
   FixupList := FixupSave;                            { Reload current list }
   GetSubViewPtr(S, V);                               { Load any subviews }
   SetCurrent(V, NormalSelect);                       { Select current view }
   If (OwnerGroup = Nil) Then Awaken;                 { If topview activate }
END;

{--TGroup-------------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
DESTRUCTOR TGroup.Done;
VAR P, T: PView;
BEGIN
   Hide;                                              { Hide the view }
   P := Last;                                         { Start on last }
   If (P <> Nil) Then Begin                           { Subviews exist }
     Repeat
       P^.Hide;                                       { Hide each view }
       P := P^.Prev;                                  { Prior view }
     Until (P = Last);                                { Loop complete }
     Repeat
       T := P^.Prev;                                  { Hold prior pointer }
       Dispose(P, Done);                              { Dispose subview }
       P := T;                                        { Transfer pointer }
     Until (Last = Nil);                              { Loop complete }
   End;
   Inherited Done;                                    { Call ancestor }
END;

{--TGroup-------------------------------------------------------------------}
{  First -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB             }
{---------------------------------------------------------------------------}
FUNCTION TGroup.First: PView;
BEGIN
   If (Last = Nil) Then First := Nil                  { No first view }
     Else First := Last^.Next;                        { Return first view }
END;

{--TGroup-------------------------------------------------------------------}
{  Execute -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB           }
{---------------------------------------------------------------------------}
FUNCTION TGroup.Execute: Word;
VAR Event: TEvent;
BEGIN
   Repeat
     EndState := 0;                                   { Clear end state }
     Repeat
       GetEvent(Event);                               { Get next event }
       HandleEvent(Event);                            { Handle the event }
       If (Event.What <> evNothing) Then
         EventError(Event);                           { Event not handled }
     Until (EndState <> 0);                           { Until command set }
   Until Valid(EndState);                             { Repeat until valid }
   Execute := EndState;                               { Return result }
   EndState := 0;                                     { Clear end state }
END;

{--TGroup-------------------------------------------------------------------}
{  GetHelpCtx -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TGroup.GetHelpCtx: Word;
VAR H: Word;
BEGIN
   H := hcNoContext;                                  { Preset no context }
   If (Current <> Nil) Then H := Current^.GetHelpCtx; { Current context }
   If (H=hcNoContext) Then H := Inherited GetHelpCtx; { Call ancestor }
   GetHelpCtx := H;                                   { Return result }
END;

{--TGroup-------------------------------------------------------------------}
{  DataSize -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Jul98 LdB          }
{---------------------------------------------------------------------------}
FUNCTION TGroup.DataSize: Sw_Word;
VAR Total: Word; P: PView;
BEGIN
   Total := 0;                                        { Zero totals count }
   P := Last;                                         { Start on last view }
   If (P <> Nil) Then Begin                           { Subviews exist }
     Repeat
       P := P^.Next;                                  { Move to next view }
       Total := Total + P^.DataSize;                  { Add view size }
     Until (P = Last);                                { Until last view }
   End;
   DataSize := Total;                                 { Return data size }
END;

{--TGroup-------------------------------------------------------------------}
{  ExecView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Jul99 LdB          }
{---------------------------------------------------------------------------}
FUNCTION TGroup.ExecView (P: PView): Word;
VAR SaveOptions: Word; SaveTopView, SaveCurrent: PView; SaveOwner: PGroup;
    SaveCommands: TCommandSet;
BEGIN
   If (P<>Nil) Then Begin
     SaveOptions := P^.Options;                       { Hold options }
     SaveOwner := P^.Owner;                           { Hold owner }
     SaveTopView := TheTopView;                       { Save topmost view }
     SaveCurrent := Current;                          { Save current view }
     GetCommands(SaveCommands);                       { Save commands }
     TheTopView := P;                                 { Set top view }
     P^.Options := P^.Options AND NOT ofSelectable;   { Not selectable }
     P^.SetState(sfModal, True);                      { Make modal }
     SetCurrent(P, EnterSelect);                      { Select next }
     If (SaveOwner = Nil) Then Insert(P);             { Insert view }
     ExecView := P^.Execute;                          { Execute view }
     If (SaveOwner = Nil) Then Delete(P);             { Remove view }
     SetCurrent(SaveCurrent, LeaveSelect);            { Unselect current }
     P^.SetState(sfModal, False);                     { Clear modal state }
     P^.Options := SaveOptions;                       { Restore options }
     TheTopView := SaveTopView;                       { Restore topview }
     SetCommands(SaveCommands);                       { Restore commands }
   End Else ExecView := cmCancel;                     { Return cancel }
END;

{ ********************************* REMARK ******************************** }
{    This call really is very COMPILER SPECIFIC and really can't be done    }
{    effectively any other way but assembler code as SELF & FRAMES need     }
{    to be put down in exact order and OPTIMIZERS make a mess of it.        }
{ ******************************** END REMARK *** Leon de Boer, 17Jul99 *** }

{--TGroup-------------------------------------------------------------------}
{  FirstThat -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Jul99 LdB         }
{---------------------------------------------------------------------------}
FUNCTION TGroup.FirstThat (P: TGroupFirstThatCallback): PView;
VAR
  Tp : PView;
BEGIN
  If (Last<>Nil) Then
   Begin
     Tp := Last;                                      { Set temporary ptr }
     Repeat
       Tp := Tp^.Next;                                { Get next view }
         IF Byte(PtrUInt(CallPointerMethodLocal(TCallbackFunBoolParam(P),
         { On most systems, locals are accessed relative to base pointer,
           but for MIPS cpu, they are accessed relative to stack pointer.
           This needs adaptation for so low level routines,
           like MethodPointerLocal and related objects unit functions. }
{$ifndef FPC_LOCALS_ARE_STACK_REG_RELATIVE}
         get_caller_frame(get_frame,get_pc_addr)
{$else}
         get_frame
{$endif}
         ,@self,Tp)))<>0 THEN
        Begin       { Test each view }
          FirstThat := Tp;                             { View returned true }
          Exit;                                        { Now exit }
        End;
     Until (Tp=Last);                                 { Until last }
     FirstThat := Nil;                                { None passed test }
   End
  Else
   FirstThat := Nil;                         { Return nil }
END;

{--TGroup-------------------------------------------------------------------}
{  Valid -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB             }
{---------------------------------------------------------------------------}
FUNCTION TGroup.Valid (Command: Word): Boolean;

   FUNCTION IsInvalid (P: PView): Boolean;
   BEGIN
     IsInvalid := NOT P^.Valid(Command);              { Check if valid }
   END;

BEGIN
   Valid := True;                                     { Preset valid }
   If (Command = cmReleasedFocus) Then Begin          { Release focus cmd }
     If (Current <> Nil) AND                          { Current view exists }
       (Current^.Options AND ofValidate <> 0) Then    { Validating view }
         Valid := Current^.Valid(Command);            { Validate command }
   End Else Valid := FirstThat(@IsInvalid) = Nil;     { Check first valid }
END;

{--TGroup-------------------------------------------------------------------}
{  FocusNext -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB         }
{---------------------------------------------------------------------------}
FUNCTION TGroup.FocusNext (Forwards: Boolean): Boolean;
VAR P: PView;
BEGIN
   P := FindNext(Forwards);                           { Find next view }
   FocusNext := True;                                 { Preset true }
   If (P <> Nil) Then FocusNext := P^.Focus;          { Check next focus }
END;


procedure TGroup.DrawSubViews(P, Bottom: PView);
begin
  if P <> nil then
   while P <> Bottom do
    begin
      P^.DrawView;
      P := P^.NextView;
    end;
end;


{--TGroup-------------------------------------------------------------------}
{  ReDraw -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 2Jun06 DM              }
{---------------------------------------------------------------------------}
procedure TGroup.Redraw;
begin
  {Lock to prevent screen update.}
  lockscreenupdate;
  DrawSubViews(First, nil);
  unlockscreenupdate;
  {Draw all views at once, forced update.}
  drawscreenbuf(true);
end;


PROCEDURE TGroup.ResetCursor;
BEGIN
  if (Current<>nil) then
    Current^.ResetCursor;
END;


{--TGroup-------------------------------------------------------------------}
{  Awaken -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.Awaken;

   PROCEDURE DoAwaken (P: PView);
   BEGIN
     If (P <> Nil) Then P^.Awaken;                    { Awaken view }
   END;

BEGIN
   ForEach(TCallbackProcParam(@DoAwaken));            { Awaken each view }
END;

{--TGroup-------------------------------------------------------------------}
{  Draw -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Sep97 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.Draw;
BEGIN
   If Buffer=Nil then
     DrawSubViews(First, nil)
   else
     WriteBuf(0,0,Size.X,Size.Y,Buffer);
END;


{--TGroup-------------------------------------------------------------------}
{  SelectDefaultView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.SelectDefaultView;
VAR P: PView;
BEGIN
   P := Last;                                         { Start at last }
   While (P <> Nil) Do Begin
     If P^.GetState(sfDefault) Then Begin             { Search 1st default }
       P^.Select;                                     { Select default view }
       P := Nil;                                      { Force kick out }
     End Else P := P^.PrevView;                       { Prior subview }
   End;
END;


function TGroup.ClipChilds: boolean;
begin
  ClipChilds:=true;
end;


procedure TGroup.BeforeInsert(P: PView);
begin
  { abstract }
end;

procedure TGroup.AfterInsert(P: PView);
begin
  { abstract }
end;

procedure TGroup.BeforeDelete(P: PView);
begin
  { abstract }
end;

procedure TGroup.AfterDelete(P: PView);
begin
  { abstract }
end;

{--TGroup-------------------------------------------------------------------}
{  Insert -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 29Sep99 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.Insert (P: PView);
BEGIN
  BeforeInsert(P);
  InsertBefore(P, First);
  AfterInsert(P);
END;

{--TGroup-------------------------------------------------------------------}
{  Delete -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.Delete (P: PView);
VAR SaveState: Word;
BEGIN
   BeforeDelete(P);
   SaveState := P^.State;                             { Save state }
   P^.Hide;                                           { Hide the view }
   RemoveView(P);                                     { Remove the view }
   P^.Owner := Nil;                                   { Clear owner ptr }
   P^.Next := Nil;                                    { Clear next ptr }
   if SaveState and sfVisible <> 0 then
     P^.Show;
   AfterDelete(P);
END;

{ ********************************* REMARK ******************************** }
{    This call really is very COMPILER SPECIFIC and really can't be done    }
{    effectively any other way but assembler code as SELF & FRAMES need     }
{    to be put down in exact order and OPTIMIZERS make a mess of it.        }
{ ******************************** END REMARK *** Leon de Boer, 17Jul99 *** }

{--TGroup-------------------------------------------------------------------}
{  ForEach -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Jul99 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.ForEach (P: TCallbackProcParam);
VAR
  Tp,Hp,L0 : PView;
{ Vars Hp and L0 are necessary to hold original pointers in case   }
{ when some view closes himself as a result of broadcast message ! }
BEGIN
  If (Last<>Nil) Then
   Begin
     Tp:=Last;
     Hp:=Tp^.Next;
     L0:=Last;              { Set temporary ptr }
     Repeat
       Tp:=Hp;
       if tp=nil then
        exit;
       Hp:=Tp^.Next;                        { Get next view }
       CallPointerMethodLocal(P,
         { On most systems, locals are accessed relative to base pointer,
           but for MIPS cpu, they are accessed relative to stack pointer.
           This needs adaptation for so low level routines,
           like MethodPointerLocal and related objects unit functions. }
{$ifndef FPC_LOCALS_ARE_STACK_REG_RELATIVE}
         get_caller_frame(get_frame,get_pc_addr)
{$else}
         get_frame
{$endif}
         ,@self,Tp);
     Until (Tp=L0);                                   { Until last }
   End;
END;



{--TGroup-------------------------------------------------------------------}
{  EndModal -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.EndModal (Command: Word);
BEGIN
   If (State AND sfModal <> 0) Then                   { This view is modal }
     EndState := Command Else                         { Set endstate }
     Inherited EndModal(Command);                     { Call ancestor }
END;

{--TGroup-------------------------------------------------------------------}
{  SelectNext -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.SelectNext (Forwards: Boolean);
VAR P: PView;
BEGIN
   P := FindNext(Forwards);                           { Find next view }
   If (P <> Nil) Then P^.Select;                      { Select view }
END;

{--TGroup-------------------------------------------------------------------}
{  InsertBefore -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 29Sep99 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.InsertBefore (P, Target: PView);
VAR SaveState : Word;
BEGIN
   If (P <> Nil) AND (P^.Owner = Nil) AND             { View valid }
   ((Target = Nil) OR (Target^.Owner = @Self))        { Target valid }
   Then Begin
     If (P^.Options AND ofCenterX <> 0) Then     { Centre on x axis }
       P^.Origin.X := (Size.X - P^.Size.X) div 2;
     If (P^.Options AND ofCenterY <> 0) Then     { Centre on y axis }
       P^.Origin.Y := (Size.Y - P^.Size.Y) div 2;
     SaveState := P^.State;                           { Save view state }
     P^.Hide;                                         { Make sure hidden }
     InsertView(P, Target);                           { Insert into list }
     If (SaveState AND sfVisible <> 0) Then P^.Show;  { Show the view }
     If (State AND sfActive <> 0) Then                { Was active before }
       P^.SetState(sfActive , True);                  { Make active again }
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  SetState -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.SetState (AState: Word; Enable: Boolean);

    PROCEDURE DoSetState (P: PView);
    BEGIN
      If (P <> Nil) Then P^.SetState(AState, Enable); { Set subview state }
    END;

    PROCEDURE DoExpose (P: PView);
    BEGIN
      If (P <> Nil) Then Begin
        If (P^.State AND sfVisible <> 0) Then         { Check view visible }
          P^.SetState(sfExposed, Enable);             { Set exposed flag }
      End;
    END;

BEGIN
   Inherited SetState(AState, Enable);                { Call ancestor }
   Case AState Of
     sfActive, sfDragging: Begin
         Lock;                                        { Lock the view }
         ForEach(TCallbackProcParam(@DoSetState));    { Set each subview }
         UnLock;                                      { Unlock the view }
       End;
     sfFocused: Begin
         If (Current <> Nil) Then
           Current^.SetState(sfFocused, Enable);          { Focus current view }
       End;
     sfExposed: Begin
         ForEach(TCallbackProcParam(@DoExpose));      { Expose each subview }
       End;
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  GetData -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 29Mar98 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.GetData (Var Rec);
VAR Total: Sw_Word; P: PView;
BEGIN
   Total := 0;                                        { Clear total }
   P := Last;                                         { Start at last }
   While (P <> Nil) Do Begin                          { Subviews exist }
     P^.GetData(TByteArray(Rec)[Total]);              { Get data }
     Inc(Total, P^.DataSize);                         { Increase total }
     P := P^.PrevView;                                { Previous view }
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  SetData -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 29Mar98 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.SetData (Var Rec);
VAR Total: Sw_Word; P: PView;
BEGIN
   Total := 0;                                        { Clear total }
   P := Last;                                         { Start at last }
   While (P <> Nil) Do Begin                          { Subviews exist }
     P^.SetData(TByteArray(Rec)[Total]);              { Get data }
     Inc(Total, P^.DataSize);                         { Increase total }
     P := P^.PrevView;                                { Previous view }
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Mar98 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.Store (Var S: TStream);
VAR Count: Word; OwnerSave: PGroup;

   PROCEDURE DoPut (P: PView);
   BEGIN
     S.Put(P);                                        { Put view on stream }
   END;

BEGIN
   TView.Store(S);                                    { Call view store }
   OwnerSave := OwnerGroup;                           { Save ownergroup }
   OwnerGroup := @Self;                               { Set as owner group }
   Count := IndexOf(Last);                            { Subview count }
   S.Write(Count, SizeOf(Count));                     { Write the count }
   ForEach(TCallbackProcParam(@DoPut));               { Put each in stream }
   PutSubViewPtr(S, Current);                         { Current on stream }
   OwnerGroup := OwnerSave;                           { Restore ownergroup }
END;

{--TGroup-------------------------------------------------------------------}
{  EventError -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.EventError (Var Event: TEvent);
BEGIN
   If (Owner <> Nil) Then Owner^.EventError(Event);   { Event error }
END;

{--TGroup-------------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.HandleEvent (Var Event: TEvent);

   FUNCTION ContainsMouse (P: PView): Boolean;
   BEGIN
     ContainsMouse := (P^.State AND sfVisible <> 0)   { Is view visible }
       AND P^.MouseInView(Event.Where);               { Is point in view }
   END;

   PROCEDURE DoHandleEvent (P: PView);
   BEGIN
     If (P = Nil) OR ((P^.State AND sfDisabled <> 0) AND
       (Event.What AND(PositionalEvents OR FocusedEvents) <>0 ))
        Then Exit;                                     { Invalid/disabled }
     Case Phase Of
       phPreProcess: If (P^.Options AND ofPreProcess = 0)
         Then Exit;                                   { Not pre processing }
       phPostProcess: If (P^.Options AND ofPostProcess = 0)
         Then Exit;                                   { Not post processing }
     End;
     If (Event.What AND P^.EventMask <> 0) Then       { View handles event }
       P^.HandleEvent(Event);                         { Pass to view }
   END;

BEGIN
   Inherited HandleEvent(Event);                      { Call ancestor }
   If (Event.What = evNothing) Then Exit;             { No valid event exit }
   If (Event.What AND FocusedEvents <> 0) Then Begin  { Focused event }
     Phase := phPreProcess;                           { Set pre process }
     ForEach(TCallbackProcParam(@DoHandleEvent));     { Pass to each view }
     Phase := phFocused;                              { Set focused }
     DoHandleEvent(Current);                          { Pass to current }
     Phase := phPostProcess;                          { Set post process }
     ForEach(TCallbackProcParam(@DoHandleEvent));     { Pass to each }
   End Else Begin
     Phase := phFocused;                              { Set focused }
     If (Event.What AND PositionalEvents <> 0) Then   { Positional event }
       DoHandleEvent(FirstThat(@ContainsMouse))       { Pass to first }
       Else ForEach(TCallbackProcParam(@DoHandleEvent)); { Pass to all }
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  ChangeBounds -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.ChangeBounds (Var Bounds: TRect);
VAR D: TPoint;

   PROCEDURE DoCalcChange (P: PView);
   VAR R: TRect;
   BEGIN
     P^.CalcBounds(R, D);                             { Calc view bounds }
     P^.ChangeBounds(R);                              { Change view bounds }
   END;

BEGIN
   D.X := Bounds.B.X - Bounds.A.X - Size.X;           { Delta x value }
   D.Y := Bounds.B.Y - Bounds.A.Y - Size.Y;           { Delta y value }
   If ((D.X=0) AND (D.Y=0)) Then Begin
     SetBounds(Bounds);                               { Set new bounds }
     { Force redraw }
     ReDraw;                                        { Draw the view }
   End Else Begin
     SetBounds(Bounds);                               { Set new bounds }
     GetExtent(Clip);                                 { Get new clip extents }
     Lock;                                            { Lock drawing }
     ForEach(TCallbackProcParam(@DoCalcChange));      { Change each view }
     UnLock;                                          { Unlock drawing }
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  GetSubViewPtr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 20May98 LdB     }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.GetSubViewPtr (Var S: TStream; Var P);
VAR Index, I: Sw_Word; Q: PView;
BEGIN
   Index := 0;                                        { Zero index value }
   S.Read(Index, SizeOf(Index));                      { Read view index }
   If (Index > 0) Then Begin                          { Valid index }
     Q := Last;                                       { Start on last }
     For I := 1 To Index Do Q := Q^.Next;             { Loop for count }
     Pointer(P) := Q;                                 { Return the view }
   End Else Pointer(P) := Nil;                        { Return nil }
END;

{--TGroup-------------------------------------------------------------------}
{  PutSubViewPtr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 20May98 LdB     }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.PutSubViewPtr (Var S: TStream; P: PView);
VAR Index: Sw_Word;
BEGIN
   If (P = Nil) Then Index := 0 Else                  { Nil view, Index = 0 }
     Index := IndexOf(P);                             { Calc view index }
   S.Write(Index, SizeOf(Index));                     { Write the index }
END;


{***************************************************************************}
{                       TGroup OBJECT PRIVATE METHODS                       }
{***************************************************************************}

{--TGroup-------------------------------------------------------------------}
{  IndexOf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB           }
{---------------------------------------------------------------------------}
FUNCTION TGroup.IndexOf (P: PView): Sw_Integer;
VAR I: Sw_Integer; Q: PView;
BEGIN
   Q := Last;                                         { Start on last view }
   If (Q <> Nil) Then Begin                           { Subviews exist }
     I := 1;                                          { Preset value }
     While (Q <> P) AND (Q^.Next <> Last) Do Begin
       Q := Q^.Next;                                  { Load next view }
       Inc(I);                                        { Increment count }
     End;
     If (Q <> P) Then IndexOf := 0 Else IndexOf := I; { Return index }
   End Else IndexOf := 0;                             { Return zero }
END;

{--TGroup-------------------------------------------------------------------}
{  FindNext -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 23Sep99 LdB          }
{---------------------------------------------------------------------------}
FUNCTION TGroup.FindNext (Forwards: Boolean): PView;
VAR P: PView;
BEGIN
   FindNext := Nil;                                   { Preset nil return }
   If (Current <> Nil) Then Begin                     { Has current view }
     P := Current;                                    { Start on current }
     Repeat
       If Forwards Then P := P^.Next                  { Get next view }
         Else P := P^.Prev;                           { Get prev view }
     Until ((P^.State AND (sfVisible+sfDisabled) = sfVisible) AND
            (P^.Options AND ofSelectable <> 0)) OR          { Tab selectable }
     (P = Current);                                   { Not singular select }
     If (P <> Current) Then FindNext := P;            { Return result }
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  FirstMatch -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TGroup.FirstMatch (AState: Word; AOptions: Word): PView;

   FUNCTION Matches (P: PView): Boolean;
   BEGIN
     Matches := (P^.State AND AState = AState) AND
       (P^.Options AND AOptions = AOptions);          { Return match state }
   END;

BEGIN
   FirstMatch := FirstThat(@Matches);                 { Return first match }
END;

{--TGroup-------------------------------------------------------------------}
{  ResetCurrent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.ResetCurrent;
BEGIN
   SetCurrent(FirstMatch(sfVisible, ofSelectable),
     NormalSelect);                                   { Reset current view }
END;

{--TGroup-------------------------------------------------------------------}
{  RemoveView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.RemoveView (P: PView);
VAR Q: PView;
BEGIN
   If (P <> Nil) AND (Last <> Nil) Then Begin         { Check view is valid }
     Q := Last;                                       { Start on last view }
     While (Q^.Next <> P) AND (Q^.Next <> Last) Do
       Q := Q^.Next;                                  { Find prior view }
     If (Q^.Next = P) Then Begin                      { View found }
       If (Q^.Next <> Q) Then Begin                   { Not only view }
         Q^.Next := P^.Next;                          { Rechain views }
         If (P = Last) Then Last := P^.Next;          { Fix if last removed }
       End Else Last := Nil;                          { Only view }
     End;
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  InsertView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.InsertView (P, Target: PView);
BEGIN
   If (P <> Nil) Then Begin                           { Check view is valid }
     P^.Owner := @Self;                               { Views owner is us }
     If (Target <> Nil) Then Begin                    { Valid target }
       Target := Target^.Prev;                        { 1st part of chain }
       P^.Next := Target^.Next;                       { 2nd part of chain }
       Target^.Next := P;                             { Chain completed }
     End Else Begin
       If (Last <> Nil) Then Begin                    { Not first view }
         P^.Next := Last^.Next;                       { 1st part of chain }
         Last^.Next := P;                             { Completed chain }
       End Else P^.Next := P;                         { 1st chain to self }
       Last := P;                                     { P is now last }
     End;
   End;
END;

{--TGroup-------------------------------------------------------------------}
{  SetCurrent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 23Sep99 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.SetCurrent (P: PView; Mode: SelectMode);

   PROCEDURE SelectView (P: PView; Enable: Boolean);
   BEGIN
     If (P <> Nil) Then                               { View is valid }
       P^.SetState(sfSelected, Enable);               { Select the view }
   END;

   PROCEDURE FocusView (P: PView; Enable: Boolean);
   BEGIN
     If (State AND sfFocused <> 0) AND (P <> Nil)     { Check not focused }
       Then P^.SetState(sfFocused, Enable);           { Focus the view }
   END;

BEGIN
   If (Current<>P) Then Begin                         { Not already current }
     Lock;                                            { Stop drawing }
     FocusView(Current, False);                       { Defocus current }
     If (Mode <> EnterSelect) Then
       SelectView(Current, False);                    { Deselect current }
     If (Mode<>LeaveSelect) Then SelectView(P, True); { Select view P }
     FocusView(P, True);                              { Focus view P }
     Current := P;                                    { Set as current view }
     UnLock;                                          { Redraw now }
   End;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           TFrame OBJECT METHODS                           }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TFrame-------------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TFrame.Init (Var Bounds: TRect);
BEGIN
   Inherited Init(Bounds);                            { Call ancestor }
   GrowMode := gfGrowHiX + gfGrowHiY;                 { Set grow modes }
   EventMask := EventMask OR evBroadcast;             { See broadcasts }
END;

procedure TFrame.FrameLine(var FrameBuf; Y, N: Sw_Integer; Color: Byte);
const
  InitFrame: array[0..17] of Byte =
    ($06, $0A, $0C, $05, $00, $05, $03, $0A, $09,
     $16, $1A, $1C, $15, $00, $15, $13, $1A, $19);
  FrameChars_437: array[0..31] of Char =
    '   À ³ÚÃ ÙÄÁ¿´ÂÅ   È ºÉÇ ¼ÍÏ»¶ÑÎ';
  FrameChars_850: array[0..31] of Char =
    '   À ³ÚÃ ÙÄÁ¿´ÂÅ   È ºÉº ¼ÍÍ»ºÍÎ';
var
  FrameMask : array[0..MaxViewWidth-1] of Byte;
  ColorMask : word;
  i,j,k     : {Sw_  lo and hi are used !! }integer;
  CurrView  : PView;
  p         : Pchar;
begin
  FrameMask[0]:=InitFrame[n];
  FillChar(FrameMask[1],Size.X-2,InitFrame[n+1]);
  FrameMask[Size.X-1]:=InitFrame[n+2];
  CurrView:=Owner^.Last^.Next;
  while (CurrView<>PView(@Self)) do
   begin
     if ((CurrView^.Options and ofFramed)<>0) and
        ((CurrView^.State and sfVisible)<>0) then
      begin
        i:=Y-CurrView^.Origin.Y;
        if (i<0) then
         begin
           inc(i);
           if i=0 then
            i:=$0a06
           else
            i:=0;
         end
        else
         begin
           if i<CurrView^.Size.Y then
            i:=$0005
           else
            if i=CurrView^.Size.Y then
             i:=$0a03
           else
            i:=0;
         end;
        if (i<>0) then
         begin
           j:=CurrView^.Origin.X;
           k:=CurrView^.Size.X+j;
           if j<1 then
            j:=1;
           if k>Size.X then
            k:=Size.X;
           if (k>j) then
            begin
              FrameMask[j-1]:=FrameMask[j-1] or lo(i);
              i:=(lo(i) xor hi(i)) or (i and $ff00);
              FrameMask[k]:=FrameMask[k] or lo(i);
              if hi(i)<>0 then
               begin
                 dec(k,j);
                 repeat
                   FrameMask[j]:=FrameMask[j] or hi(i);
                   inc(j);
                   dec(k);
                 until k=0;
               end;
            end;
         end;
      end;
     CurrView:=CurrView^.Next;
   end;
  ColorMask:=Color shl 8;
  p:=framechars_437;
  {$ifdef unix}
  {Codepage variables are currently Unix only.}
  if internal_codepage<>cp437 then
    p:=framechars_850;
  {$endif}
  for i:=0 to Size.X-1 do
    TVideoBuf(FrameBuf)[i]:=ord(p[FrameMask[i]]) or ColorMask;
end;


procedure TFrame.Draw;
const
  LargeC:array[boolean] of char=('^',#24);
  RestoreC:array[boolean] of char=('|',#18);
  ClickC:array[boolean] of char=('*',#15);
var
  CFrame, CTitle: Word;
  F, I, L, Width: Sw_Integer;
  B: TDrawBuffer;
  Title: TTitleStr;
  Min, Max: TPoint;
begin
  if State and sfDragging <> 0 then
   begin
     CFrame := $0505;
     CTitle := $0005;
     F := 0;
   end
  else if State and sfActive = 0 then
   begin
     CFrame := $0101;
     CTitle := $0002;
     F := 0;
   end
  else
   begin
     CFrame := $0503;
     CTitle := $0004;
     F := 9;
   end;
  CFrame := GetColor(CFrame);
  CTitle := GetColor(CTitle);
  Width := Size.X;
  L := Width - 10;
  if PWindow(Owner)^.Flags and (wfClose+wfZoom) <> 0 then
   Dec(L,6);
  FrameLine(B, 0, F, Byte(CFrame));
  if (PWindow(Owner)^.Number <> wnNoNumber) and
     (PWindow(Owner)^.Number < 10) then
   begin
     Dec(L,4);
     if PWindow(Owner)^.Flags and wfZoom <> 0 then
      I := 7
     else
      I := 3;
     WordRec(B[Width - I]).Lo := PWindow(Owner)^.Number + $30;
   end;
  if Owner <> nil then
   Title := PWindow(Owner)^.GetTitle(L)
  else
   Title := '';
  if Title <> '' then
   begin
     L := Length(Title);
     if L > Width - 10 then
      L := Width - 10;
     if L < 0 then
      L := 0;
     I := (Width - L) shr 1;
     MoveChar(B[I - 1], ' ', CTitle, 1);
     MoveBuf(B[I], Title[1], CTitle, L);
     MoveChar(B[I + L], ' ', CTitle, 1);
   end;
  if State and sfActive <> 0 then
  begin
    if PWindow(Owner)^.Flags and wfClose <> 0 then
      if FrameMode and fmCloseClicked = 0 then
        MoveCStr(B[2], '[~þ~]', CFrame)
      else
        MoveCStr(B[2], '[~'+ClickC[LowAscii]+'~]', CFrame);
    if PWindow(Owner)^.Flags and wfZoom <> 0 then
    begin
      MoveCStr(B[Width - 5], '[~'+LargeC[LowAscii]+'~]', CFrame);
      Owner^.SizeLimits(Min, Max);
      if FrameMode and fmZoomClicked <> 0 then
        WordRec(B[Width - 4]).Lo := ord(ClickC[LowAscii])
      else
        if (Owner^.Size.X=Max.X) and (Owner^.Size.Y=Max.Y) then
          WordRec(B[Width - 4]).Lo := ord(RestoreC[LowAscii]);
    end;
  end;
  WriteLine(0, 0, Size.X, 1, B);
  for I := 1 to Size.Y - 2 do
  begin
    FrameLine(B, I, F + 3, Byte(CFrame));
    WriteLine(0, I, Size.X, 1, B);
  end;
  FrameLine(B, Size.Y - 1, F + 6, Byte(CFrame));
  if State and sfActive <> 0 then
    if PWindow(Owner)^.Flags and wfGrow <> 0 then
      MoveCStr(B[Width - 2], '~ÄÙ~', CFrame);
  WriteLine(0, Size.Y - 1, Size.X, 1, B);
end;

{--TFrame-------------------------------------------------------------------}
{  GetPalette -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TFrame.GetPalette: PPalette;
CONST P: String[Length(CFrame)] = CFrame;             { Always normal string }
BEGIN
   GetPalette := PPalette(@P);                        { Return palette }
END;

procedure TFrame.HandleEvent(var Event: TEvent);
var
  Mouse: TPoint;

  procedure DragWindow(Mode: Byte);
  var
    Limits: TRect;
    Min, Max: TPoint;
  begin
    Owner^.Owner^.GetExtent(Limits);
    Owner^.SizeLimits(Min, Max);
    Owner^.DragView(Event, Owner^.DragMode or Mode, Limits, Min, Max);
    ClearEvent(Event);
  end;

begin
  TView.HandleEvent(Event);
  if Event.What = evMouseDown then
  begin
    MakeLocal(Event.Where, Mouse);
    if Mouse.Y = 0 then
    begin
      if (PWindow(Owner)^.Flags and wfClose <> 0) and
        (State and sfActive <> 0) and (Mouse.X >= 2) and (Mouse.X <= 4) then
      begin
        {Close button clicked.}
        repeat
          MakeLocal(Event.Where, Mouse);
          if (Mouse.X >= 2) and (Mouse.X <= 4) and (Mouse.Y = 0) then
            FrameMode := fmCloseClicked
          else FrameMode := 0;
          DrawView;
        until not MouseEvent(Event, evMouseMove + evMouseAuto);
        FrameMode := 0;
        if (Mouse.X >= 2) and (Mouse.X <= 4) and (Mouse.Y = 0) then
        begin
          Event.What := evCommand;
          Event.Command := cmClose;
          Event.InfoPtr := Owner;
          PutEvent(Event);
        end;
        ClearEvent(Event);
        DrawView;
      end else
        if (PWindow(Owner)^.Flags and wfZoom <> 0) and
          (State and sfActive <> 0) and (Event.Double or
          (Mouse.X >= Size.X - 5) and
          (Mouse.X <= Size.X - 3)) then
        begin
          {Zoom button clicked.}
          if not Event.Double then
            repeat
              MakeLocal(Event.Where, Mouse);
              if (Mouse.X >= Size.X - 5) and (Mouse.X <= Size.X - 3) and
                (Mouse.Y = 0) then
                FrameMode := fmZoomClicked
              else FrameMode := 0;
              DrawView;
            until not MouseEvent(Event, evMouseMove + evMouseAuto);
          FrameMode := 0;
          if ((Mouse.X >= Size.X - 5) and (Mouse.X <= Size.X - 3) and
              (Mouse.Y = 0)) or Event.Double then
          begin
            Event.What := evCommand;
            Event.Command := cmZoom;
            Event.InfoPtr := Owner;
            PutEvent(Event);
          end;
          ClearEvent(Event);
          DrawView;
        end else
          if PWindow(Owner)^.Flags and wfMove <> 0 then
            DragWindow(dmDragMove);
    end else
      if (State and sfActive <> 0) and (Mouse.X >= Size.X - 2) and
          (Mouse.Y >= Size.Y - 1) then
        if PWindow(Owner)^.Flags and wfGrow <> 0 then
          DragWindow(dmDragGrow);
  end;
end;


procedure TFrame.SetState(AState: Word; Enable: Boolean);
begin
  TView.SetState(AState, Enable);
  if AState and (sfActive + sfDragging) <> 0 then
   DrawView;
end;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                         TScrollBar OBJECT METHODS                         }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}


{--TScrollBar---------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22May98 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TScrollBar.Init (Var Bounds: TRect);
const
  VChars: array[boolean] of TScrollChars =
     (('^','V', #177, #254, #178),(#30, #31, #177, #254, #178));
  HChars: array[boolean] of TScrollChars =
     (('<','>', #177, #254, #178),(#17, #16, #177, #254, #178));
BEGIN
   Inherited Init(Bounds);                            { Call ancestor }
   PgStep := 1;                                       { Page step size = 1 }
   ArStep := 1;                                       { Arrow step sizes = 1 }
   If (Size.X = 1) Then Begin                         { Vertical scrollbar }
     GrowMode := gfGrowLoX + gfGrowHiX + gfGrowHiY;   { Grow vertically }
     Chars := VChars[LowAscii];                       { Vertical chars }
   End Else Begin                                     { Horizontal scrollbar }
     GrowMode := gfGrowLoY + gfGrowHiX + gfGrowHiY;   { Grow horizontal }
     Chars := HChars[LowAscii];                       { Horizontal chars }
   End;
END;

{--TScrollBar---------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22May98 LdB              }
{---------------------------------------------------------------------------}
{   This load method will read old original TV data from a stream with the  }
{   scrollbar id set to zero.                                               }
{---------------------------------------------------------------------------}
CONSTRUCTOR TScrollBar.Load (Var S: TStream);
VAR i: Integer;
BEGIN
   Inherited Load(S);                                 { Call ancestor }
   S.Read(i, SizeOf(i)); Value:=i;                    { Read current value }
   S.Read(i, SizeOf(i)); Min:=i;                      { Read min value }
   S.Read(i, SizeOf(i));  Max:=i;                     { Read max value }
   S.Read(i, SizeOf(i)); PgStep:=i;                   { Read page step size }
   S.Read(i, SizeOf(i)); ArStep:=i;                   { Read arrow step size }
   S.Read(Chars, SizeOf(Chars));                      { Read scroll chars }
END;

{--TScrollBar---------------------------------------------------------------}
{  GetPalette -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22May98 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TScrollBar.GetPalette: PPalette;
CONST P: String[Length(CScrollBar)] = CScrollBar;     { Always normal string }
BEGIN
   GetPalette := PPalette(@P);                        { Return palette }
END;

{--TScrollBar---------------------------------------------------------------}
{  ScrollStep -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22May98 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TScrollBar.ScrollStep (Part: Sw_Integer): Sw_Integer;
VAR Step: Sw_Integer;
BEGIN
   If (Part AND $0002 = 0) Then Step := ArStep        { Range step size }
     Else Step := PgStep;                             { Page step size }
   If (Part AND $0001 = 0) Then ScrollStep := -Step   { Upwards move }
     Else ScrollStep := Step;                         { Downwards move }
END;

{--TScrollBar---------------------------------------------------------------}
{  ScrollDraw -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22May98 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TScrollBar.ScrollDraw;
VAR P: PView;
BEGIN
   If (Id <> 0) Then Begin
     P := TopView;                                    { Get topmost view }
     NewMessage(P, evCommand, cmIdCommunicate, Id,
       Value, @Self);                                 { New Id style message }
   End;
   NewMessage(Owner, evBroadcast, cmScrollBarChanged,
     Id, Value, @Self);                               { Old TV style message }
END;


{--TScrollBar---------------------------------------------------------------}
{  SetValue -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19May98 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TScrollBar.SetValue (AValue: Sw_Integer);
BEGIN
   SetParams(AValue, Min, Max, PgStep, ArStep);       { Set value }
END;

{--TScrollBar---------------------------------------------------------------}
{  SetRange -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19May98 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TScrollBar.SetRange (AMin, AMax: Sw_Integer);
BEGIN
   SetParams(Value, AMin, AMax, PgStep, ArStep);      { Set range }
END;

{--TScrollBar---------------------------------------------------------------}
{  SetStep -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19May98 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TScrollBar.SetStep (APgStep, AArStep: Sw_Integer);
BEGIN
   SetParams(Value, Min, Max, APgStep, AArStep);      { Set step sizes }
END;

{--TScrollBar---------------------------------------------------------------}
{  SetParams -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 21Jul99 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TScrollBar.SetParams (AValue, AMin, AMax, APgStep, AArStep: Sw_Integer);
var
  OldValue : Sw_Integer;
BEGIN
   If (AMax < AMin) Then AMax := AMin;                { Max below min fix up }
   If (AValue < AMin) Then AValue := AMin;            { Value below min fix }
   If (AValue > AMax) Then AValue := AMax;            { Value above max fix }
   OldValue:=Value;
   If (Value <> AValue) OR (Min <> AMin) OR
   (Max <> AMax) Then Begin                           { Something changed }
     Min := AMin;                                   { Set new minimum }
     Max := AMax;                                   { Set new maximum }
     Value := AValue;                               { Set new value }
     DrawView;
     if OldValue <> AValue then
       ScrollDraw;
   End;
   PgStep := APgStep;                                 { Hold page step }
   ArStep := AArStep;                                 { Hold arrow step }
END;

{--TScrollBar---------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22May98 LdB             }
{---------------------------------------------------------------------------}
{  You can save data to the stream compatable with the old original TV by   }
{  temporarily turning off the ofGrafVersion making the call to this store  }
{  routine and resetting the ofGrafVersion flag after the call.             }
{---------------------------------------------------------------------------}
PROCEDURE TScrollBar.Store (Var S: TStream);
VAR i: Integer;
BEGIN
   TView.Store(S);                                    { TView.Store called }
   i:=Value;S.Write(i, SizeOf(i));                    { Write current value }
   i:=Min;S.Write(i, SizeOf(i));                      { Write min value }
   i:=Max;S.Write(i, SizeOf(i));                      { Write max value }
   i:=PgStep;S.Write(i, SizeOf(i));                   { Write page step size }
   i:=ArStep;S.Write(i, SizeOf(i));                   { Write arrow step size }
   S.Write(Chars, SizeOf(Chars));                     { Write scroll chars }
END;

{--TScrollBar---------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22May98 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TScrollBar.HandleEvent (Var Event: TEvent);
VAR Tracking: Boolean; I, P, S, ClickPart, Iv: Sw_Integer;
    Mouse: TPoint; Extent: TRect;

   FUNCTION GetPartCode: Sw_Integer;
   VAR Mark, Part : Sw_Integer;
   BEGIN
     Part := -1;                                      { Preset failure }
     If Extent.Contains(Mouse) Then Begin             { Contains mouse }
       If (Size.X = 1) Then Begin                     { Vertical scrollbar }
         Mark := Mouse.Y;                             { Calc position }
       End Else Begin                                 { Horizontal bar }
         Mark := Mouse.X;                             { Calc position }
       End;
       If (Mark >= P) AND (Mark < P+1) Then           { Within thumbnail }
         Part := sbIndicator;                         { Indicator part }
       If (Part <> sbIndicator) Then Begin            { Not indicator part }
         If (Mark < 1) Then Part := sbLeftArrow Else  { Left arrow part }
         If (Mark < P) Then Part := sbPageLeft Else   { Page left part }
         If (Mark < S-1) Then Part := sbPageRight Else  { Page right part }
           Part := sbRightArrow;                      { Right arrow part }
         If (Size.X = 1) Then Inc(Part, 4);           { Correct for vertical }
       End;
     End;
     GetPartCode := Part;                             { Return part code }
   END;

   PROCEDURE Clicked;
   BEGIN
     NewMessage(Owner, evBroadcast, cmScrollBarClicked,
       Id, Value, @Self);                             { Old TV style message }
   END;

BEGIN
   Inherited HandleEvent(Event);                      { Call ancestor }
   Case Event.What Of
     evNothing: Exit;                                 { Speed up exit }
     evCommand: Begin                                 { Command event }
       If (Event.Command = cmIdCommunicate) AND       { Id communication }
       (Event.Id = Id) AND (Event.InfoPtr <> @Self)   { Targeted to us }
       Then Begin
         SetValue(Round(Event.Data));                 { Set scrollbar value }
         ClearEvent(Event);                           { Event was handled }
       End;
     End;
     evKeyDown:
       If (State AND sfVisible <> 0) Then Begin       { Scrollbar visible }
         ClickPart := sbIndicator;                    { Preset result }
         If (Size.Y = 1) Then                         { Horizontal bar }
           Case CtrlToArrow(Event.KeyCode) Of
             kbLeft: ClickPart := sbLeftArrow;        { Left one item }
             kbRight: ClickPart := sbRightArrow;      { Right one item }
             kbCtrlLeft: ClickPart := sbPageLeft;     { One page left }
             kbCtrlRight: ClickPart := sbPageRight;   { One page right }
             kbHome: I := Min;                        { Move to start }
             kbEnd: I := Max;                         { Move to end }
             Else Exit;                               { Not a valid key }
           End
         Else                                         { Vertical bar }
           Case CtrlToArrow(Event.KeyCode) Of
             kbUp: ClickPart := sbUpArrow;            { One item up }
             kbDown: ClickPart := sbDownArrow;        { On item down }
             kbPgUp: ClickPart := sbPageUp;           { One page up }
             kbPgDn: ClickPart := sbPageDown;         { One page down }
             kbCtrlPgUp: I := Min;                    { Move to top }
             kbCtrlPgDn: I := Max;                    { Move to bottom }
             Else Exit;                               { Not a valid key }
           End;
         Clicked;                                     { Send out message }
         If (ClickPart <> sbIndicator) Then
           I := Value + ScrollStep(ClickPart);        { Calculate position }
         SetValue(I);                                 { Set new item }
         ClearEvent(Event);                           { Event now handled }
     End;
     evMouseDown: Begin                               { Mouse press event }
         Clicked;                                     { Scrollbar clicked }
         MakeLocal(Event.Where, Mouse);                 { Localize mouse }
         Extent.A.X := 0;                             { Zero x extent value }
         Extent.A.Y := 0;                             { Zero y extent value }
         Extent.B.X := Size.X;                        { Set extent x value }
         Extent.B.Y := Size.Y;                        { set extent y value }
         P := GetPos;                                 { Current position }
         S := GetSize;                                { Initial size }
         ClickPart := GetPartCode;                    { Get part code }
         If (ClickPart <> sbIndicator) Then Begin     { Not thumb nail }
           Repeat
             MakeLocal(Event.Where, Mouse);           { Localize mouse }
             If GetPartCode = ClickPart Then
               SetValue(Value+ScrollStep(ClickPart)); { Same part repeat }
           Until NOT MouseEvent(Event, evMouseAuto);  { Until auto done }
           Clicked;                                   { Scrollbar clicked }
         End Else Begin                               { Thumb nail move }
           Iv := Value;                               { Initial value }
           Repeat
             MakeLocal(Event.Where, Mouse);           { Localize mouse }
             Tracking := Extent.Contains(Mouse);      { Check contains }
             If Tracking Then Begin                   { Tracking mouse }
               If (Size.X=1) Then
                 I := Mouse.Y Else                    { Calc vert position }
                 I := Mouse.X;                        { Calc horz position }
               If (I < 0) Then I := 0;                { Check underflow }
               If (I > S) Then I := S;                { Check overflow }
             End Else I := GetPos;                    { Get position }
             If (I <> P) Then Begin
               SetValue(LongInt((LongInt(I)*(Max-Min))
                 +(S SHR 1)) DIV S + Min);            { Set new value }
               P := I;                                { Hold new position }
             End;
           Until NOT MouseEvent(Event, evMouseMove);  { Until not moving }
           If Tracking AND (S > 0) Then               { Tracking mouse }
             SetValue(LongInt((LongInt(P)*(Max-Min))+
               (S SHR 1)) DIV S + Min);               { Set new value }
           If (Iv <> Value) Then Clicked;             { Scroll has moved }
         End;
         ClearEvent(Event);                           { Clear the event }
     End;
   End;
END;

{***************************************************************************}
{                 TScrollBar OBJECT PRIVATE METHODS                         }
{***************************************************************************}

{--TScrollBar---------------------------------------------------------------}
{  GetPos -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 23May98 LdB            }
{---------------------------------------------------------------------------}
FUNCTION TScrollBar.GetPos: Sw_Integer;
VAR R: Sw_Integer;
BEGIN
   R := Max - Min;                                    { Get full range }
   If (R = 0) Then GetPos := 1 Else                   { Return zero }
     GetPos := LongInt((LongInt(Value-Min) * (GetSize -3))
       + (R SHR 1)) DIV R + 1;                        { Calc position }
END;

{--TScrollBar---------------------------------------------------------------}
{  GetSize -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 23May98 LdB           }
{---------------------------------------------------------------------------}
FUNCTION TScrollBar.GetSize: Sw_Integer;
VAR S: Sw_Integer;
BEGIN
   If Size.X = 1 Then
     S:= Size.Y
   else
     S:= Size.X;
   If (S < 3) Then S := 3;                            { Fix minimum size }
   GetSize := S;                                      { Return size }
END;


{--TScrollBar---------------------------------------------------------------}
{  Draw -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 27Oct99 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TScrollBar.Draw;
BEGIN
  DrawPos(GetPos);                                 { Draw position }
END;


procedure TScrollBar.DrawPos(Pos: Sw_Integer);
var
  S: Sw_Integer;
  B: TDrawBuffer;
begin
  S := GetSize - 1;
  MoveChar(B[0], Chars[0], GetColor(2), 1);
  if Max = Min then
    MoveChar(B[1], Chars[4], GetColor(1), S - 1)
  else
   begin
     MoveChar(B[1], Chars[2], GetColor(1), S - 1);
     MoveChar(B[Pos], Chars[3], GetColor(3), 1);
   end;
  MoveChar(B[S], Chars[1], GetColor(2), 1);
  WriteBuf(0, 0, Size.X, Size.Y, B);
end;


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                         TScroller OBJECT METHODS                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TScroller----------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TScroller.Init (Var Bounds: TRect; AHScrollBar, AVScrollBar: PScrollBar);
BEGIN
   Inherited Init(Bounds);                            { Call ancestor }
   Options := Options OR ofSelectable;                { View is selectable }
   EventMask := EventMask OR evBroadcast;             { See broadcasts }
   HScrollBar := AHScrollBar;                         { Hold horz scrollbar }
   VScrollBar := AVScrollBar;                         { Hold vert scrollbar }
END;

{--TScroller----------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB              }
{---------------------------------------------------------------------------}
{   This load method will read old original TV data from a stream as well   }
{   as the new graphical scroller views.                                    }
{---------------------------------------------------------------------------}
CONSTRUCTOR TScroller.Load (Var S: TStream);
VAR i: Integer;
BEGIN
   Inherited Load(S);                                 { Call ancestor }
   GetPeerViewPtr(S, HScrollBar);                     { Load horz scrollbar }
   GetPeerViewPtr(S, VScrollBar);                     { Load vert scrollbar }
   S.Read(i, SizeOf(i)); Delta.X:=i;                  { Read delta x value }
   S.Read(i, SizeOf(i)); Delta.Y:=i;                  { Read delta y value }
   S.Read(i, SizeOf(i)); Limit.X:=i;                  { Read limit x value }
   S.Read(i, SizeOf(i)); Limit.Y:=i;                  { Read limit y value }
END;

{--TScroller----------------------------------------------------------------}
{  GetPalette -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TScroller.GetPalette: PPalette;
CONST P: String[Length(CScroller)] = CScroller;       { Always normal string }
BEGIN
   GetPalette := PPalette(@P);                        { Scroller palette }
END;

{--TScroller----------------------------------------------------------------}
{  ScrollTo -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TScroller.ScrollTo (X, Y: Sw_Integer);
BEGIN
   Inc(DrawLock);                                     { Set draw lock }
   If (HScrollBar<>Nil) Then HScrollBar^.SetValue(X); { Set horz scrollbar }
   If (VScrollBar<>Nil) Then VScrollBar^.SetValue(Y); { Set vert scrollbar }
   Dec(DrawLock);                                     { Release draw lock }
   CheckDraw;                                         { Check need to draw }
END;

{--TScroller----------------------------------------------------------------}
{  SetState -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TScroller.SetState (AState: Word; Enable: Boolean);

   PROCEDURE ShowSBar (SBar: PScrollBar);
   BEGIN
     If (SBar <> Nil) Then                            { Scroll bar valid }
       If GetState(sfActive + sfSelected) Then        { Check state masks }
         SBar^.Show Else SBar^.Hide;                  { Draw appropriately }
   END;

BEGIN
   Inherited SetState(AState, Enable);                { Call ancestor }
   If (AState AND (sfActive + sfSelected) <> 0)       { Active/select change }
   Then Begin
     ShowSBar(HScrollBar);                            { Redraw horz scrollbar }
     ShowSBar(VScrollBar);                            { Redraw vert scrollbar }
   End;
END;

{--TScroller----------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB             }
{---------------------------------------------------------------------------}
{  The scroller is saved to the stream compatable with the old TV object.   }
{---------------------------------------------------------------------------}
PROCEDURE TScroller.Store (Var S: TStream);
VAR i: Integer;
BEGIN
   TView.Store(S);                                    { Call TView explicitly }
   PutPeerViewPtr(S, HScrollBar);                     { Store horz bar }
   PutPeerViewPtr(S, VScrollBar);                     { Store vert bar }
   i:=Delta.X;S.Write(i, SizeOf(i));                  { Write delta x value }
   i:=Delta.Y;S.Write(i, SizeOf(i));                  { Write delta y value }
   i:=Limit.X;S.Write(i, SizeOf(i));                  { Write limit x value }
   i:=Limit.Y;S.Write(i, SizeOf(i));                  { Write limit y value }
END;

{--TScroller----------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TScroller.HandleEvent (Var Event: TEvent);
BEGIN
   Inherited HandleEvent(Event);                      { Call ancestor }
   If (Event.What = evBroadcast) AND
     (Event.Command = cmScrollBarChanged) AND         { Scroll bar change }
     ((Event.InfoPtr = HScrollBar) OR                 { Our scrollbar? }
      (Event.InfoPtr = VScrollBar)) Then ScrollDraw;  { Redraw scroller }
END;

{--TScroller----------------------------------------------------------------}
{  ChangeBounds -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TScroller.ChangeBounds (Var Bounds: TRect);
BEGIN
   SetBounds(Bounds);                                 { Set new bounds }
   Inc(DrawLock);                                     { Set draw lock }
   SetLimit(Limit.X, Limit.Y);                        { Adjust limits }
   Dec(DrawLock);                                     { Release draw lock }
   DrawFlag := False;                                 { Clear draw flag }
   DrawView;                                          { Redraw now }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        TListViewer OBJECT METHODS                         }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

CONST TvListViewerName = 'LISTBOX';                   { Native name }

{--TListViewer--------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 28May98 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TListViewer.Init (Var Bounds: TRect; ANumCols: Sw_Word; AHScrollBar,
  AVScrollBar: PScrollBar);
VAR ArStep, PgStep: Sw_Integer;
BEGIN
   Inherited Init(Bounds);                            { Call ancestor }
   Options := Options OR (ofFirstClick+ofSelectable); { Set options }
   EventMask := EventMask OR evBroadcast;             { Set event mask }
   NumCols := ANumCols;                               { Hold column number }
   If (AVScrollBar <> Nil) Then Begin                 { Chk vert scrollbar }
     If (NumCols = 1) Then Begin                      { Only one column }
       PgStep := Size.Y -1;                           { Set page size }
       ArStep := 1;                                   { Set step size }
     End Else Begin                                   { Multiple columns }
       PgStep := Size.Y * NumCols;                    { Set page size }
       ArStep := Size.Y;                              { Set step size }
     End;
     AVScrollBar^.SetStep(PgStep, ArStep);            { Set scroll values }
   End;
   If (AHScrollBar <> Nil) Then
     AHScrollBar^.SetStep(Size.X DIV NumCols, 1);     { Set step size }
   HScrollBar := AHScrollBar;                         { Horz scrollbar held }
   VScrollBar := AVScrollBar;                         { Vert scrollbar held }
END;

{--TListViewer--------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 28May98 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TListViewer.Load (Var S: TStream);
VAR w: Word;
BEGIN
   Inherited Load(S);                                 { Call ancestor }
   GetPeerViewPtr(S, HScrollBar);                     { Get horz scrollbar }
   GetPeerViewPtr(S, VScrollBar);                     { Get vert scrollbar }
   S.Read(w, SizeOf(w)); NumCols:=w;                  { Read column number }
   S.Read(w, SizeOf(w)); TopItem:=w;                  { Read top most item }
   S.Read(w, SizeOf(w)); Focused:=w;                  { Read focused item }
   S.Read(w, SizeOf(w)); Range:=w;                    { Read listview range }
END;

{--TListViewer--------------------------------------------------------------}
{  GetPalette -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 28May98 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TListViewer.GetPalette: PPalette;
CONST P: String[Length(CListViewer)] = CListViewer;   { Always normal string }
BEGIN
   GetPalette := PPalette(@P);                        { Return palette }
END;

{--TListViewer--------------------------------------------------------------}
{  IsSelected -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 28May98 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TListViewer.IsSelected (Item: Sw_Integer): Boolean;
BEGIN
   If (Item = Focused) Then IsSelected := True Else
     IsSelected := False;                             { Selected item }
END;

{--TListViewer--------------------------------------------------------------}
{  GetText -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 28May98 LdB           }
{---------------------------------------------------------------------------}
FUNCTION TListViewer.GetText (Item: Sw_Integer; MaxLen: Sw_Integer): String;
BEGIN                                                 { Abstract method }
   GetText := '';                                     { Return empty }
END;

{--TListViewer--------------------------------------------------------------}
{  DrawBackGround -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 27Oct99 LdB    }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.Draw;
VAR  I, J, ColWidth, Item, Indent, CurCol: Sw_Integer;
     Color: Word; SCOff: Byte;
     Text: String; B: TDrawBuffer;
BEGIN
   ColWidth := Size.X DIV NumCols + 1;                { Calc column width }
   If (HScrollBar = Nil) Then Indent := 0 Else        { Set indent to zero }
     Indent := HScrollBar^.Value;                     { Fetch any indent }
   For I := 0 To Size.Y - 1 Do Begin                  { For each line }
     For J := 0 To NumCols-1 Do Begin                 { For each column }
       Item := J*Size.Y + I + TopItem;                { Process this item }
       CurCol := J*ColWidth;                          { Current column }
       If (State AND (sfSelected + sfActive) =
       (sfSelected + sfActive)) AND (Focused = Item)  { Focused item }
       AND (Range > 0) Then Begin
         Color := GetColor(3);                        { Focused colour }
         SetCursor(CurCol+1,I);                       { Set the cursor }
         SCOff := 0;                                  { Zero colour offset }
       End Else If (Item < Range) AND IsSelected(Item){ Selected item }
       Then Begin
         Color := GetColor(4);                        { Selected color }
         SCOff := 2;                                  { Colour offset=2 }
       End Else Begin
         Color := GetColor(2);                        { Normal Color }
         SCOff := 4;                                  { Colour offset=4 }
       End;
      MoveChar(B[CurCol], ' ', Color, ColWidth);     { Clear buffer }
       If (Item < Range) Then Begin                   { Within text range }
         Text := GetText(Item, ColWidth + Indent);    { Fetch text }
         Text := Copy(Text, Indent, ColWidth);        { Select right bit }
         MoveStr(B[CurCol+1], Text, Color);           { Transfer to buffer }
         If ShowMarkers Then Begin
           WordRec(B[CurCol]).Lo := Byte(
             SpecialChars[SCOff]);                        { Set marker character }
           WordRec(B[CurCol+ColWidth-2]).Lo := Byte(
             SpecialChars[SCOff+1]);                        { Set marker character }
         End;
       End;
       MoveChar(B[CurCol+ColWidth-1], #179,
         GetColor(5), 1);                             { Put centre line marker }
     End;
     WriteLine(0, I, Size.X, 1, B);                 { Write line to screen }
   End;
END;


{--TListViewer--------------------------------------------------------------}
{  FocusItem -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.FocusItem (Item: Sw_Integer);
BEGIN
   Focused := Item;                                   { Set focus to item }
   If (VScrollBar <> Nil) Then
     VScrollBar^.SetValue(Item);                      { Scrollbar to value }
   If (Item < TopItem) Then                           { Item above top item }
     If (NumCols = 1) Then TopItem := Item            { Set top item }
       Else TopItem := Item - Item MOD Size.Y         { Set top item }
   Else If (Item >= TopItem + (Size.Y*NumCols)) Then  { Item below bottom }
     If (NumCols = 1) Then TopItem := Item-Size.Y+1   { Set new top item }
     Else TopItem := Item - Item MOD Size.Y -
       (Size.Y*(NumCols-1));                          { Set new top item }
END;

{--TListViewer--------------------------------------------------------------}
{  SetTopItem -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 06Aug99 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.SetTopItem (Item: Sw_Integer);
BEGIN
   TopItem := Item;                                   { Set the top item }
END;

{--TListViewer--------------------------------------------------------------}
{  SetRange -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.SetRange (ARange: Sw_Integer);
BEGIN
   Range := ARange;                                   { Set new range }
   If (VScrollBar <> Nil) Then Begin                  { Vertical scrollbar }
     If (Focused > ARange) Then Focused := 0;         { Clear focused }
     VScrollBar^.SetParams(Focused, 0, ARange - 1,
       VScrollBar^.PgStep, VScrollBar^.ArStep);       { Set parameters }
   End;
END;

{--TListViewer--------------------------------------------------------------}
{  SelectItem -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.SelectItem (Item: Sw_Integer);
BEGIN
   Message(Owner, evBroadcast, cmListItemSelected,
     @Self);                                          { Send message }
END;

{--TListViewer--------------------------------------------------------------}
{  SetState -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 27Oct99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.SetState (AState: Word; Enable: Boolean);

   PROCEDURE ShowSBar(SBar: PScrollBar);
   BEGIN
     If (SBar <> Nil) Then                            { Valid scrollbar }
       If GetState(sfActive) AND GetState(sfVisible)  { Check states }
         Then SBar^.Show Else SBar^.Hide;             { Show or hide }
   END;

BEGIN
   Inherited SetState(AState, Enable);                { Call ancestor }
   If (AState AND (sfSelected + sfActive + sfVisible) <> 0)
   Then Begin                                         { Check states }
     DrawView;                                        { Draw the view }
     ShowSBar(HScrollBar);                            { Show horz scrollbar }
     ShowSBar(VScrollBar);                            { Show vert scrollbar }
   End;
END;

{--TListViewer--------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.Store (Var S: TStream);
VAR w: Word;
BEGIN
   TView.Store(S);                                    { Call TView explicitly }
   PutPeerViewPtr(S, HScrollBar);                     { Put horz scrollbar }
   PutPeerViewPtr(S, VScrollBar);                     { Put vert scrollbar }
   w:=NumCols;S.Write(w, SizeOf(w));                  { Write column number }
   w:=TopItem;S.Write(w, SizeOf(w));                  { Write top most item }
   w:=Focused;S.Write(w, SizeOf(w));                  { Write focused item }
   w:=Range;S.Write(w, SizeOf(w));                    { Write listview range }
END;

{--TListViewer--------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 27Oct99 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.HandleEvent (Var Event: TEvent);
CONST MouseAutosToSkip = 4;
VAR Oi, Ni: Sw_Integer; Ct, Cw: Word; Mouse: TPoint;

   PROCEDURE MoveFocus (Req: Sw_Integer);
   BEGIN
     FocusItemNum(Req);                               { Focus req item }
     DrawView;                                      { Redraw focus box }
   END;

BEGIN
   Inherited HandleEvent(Event);                      { Call ancestor }
   Case Event.What Of
     evNothing: Exit;                                 { Speed up exit }
     evKeyDown: Begin                                 { Key down event }
       If (Event.CharCode = ' ') AND (Focused < Range){ Spacebar select }
       Then Begin
         SelectItem(Focused);                         { Select focused item }
         Ni := Focused;                               { Hold new item }
       End Else Case CtrlToArrow(Event.KeyCode) Of
         kbUp: Ni := Focused - 1;                     { One item up }
         kbDown: Ni := Focused + 1;                   { One item down }
         kbRight: If (NumCols > 1) Then
           Ni := Focused + Size.Y Else Exit;          { One column right }
         kbLeft: If (NumCols > 1) Then
           Ni := Focused - Size.Y Else Exit;          { One column left }
         kbPgDn: Ni := Focused + Size.Y * NumCols;    { One page down }
         kbPgUp: Ni := Focused - Size.Y * NumCols;    { One page up }
         kbHome: Ni := TopItem;                       { Move to top }
         kbEnd: Ni := TopItem + (Size.Y*NumCols)-1;   { Move to bottom }
         kbCtrlPgDn: Ni := Range - 1;                 { Move to last item }
         kbCtrlPgUp: Ni := 0;                         { Move to first item }
         Else Exit;
       End;
       MoveFocus(Ni);                                 { Move the focus }
       ClearEvent(Event);                             { Event was handled }
     End;
     evBroadcast: Begin                               { Broadcast event }
       If (Options AND ofSelectable <> 0) Then        { View is selectable }
         If (Event.Command = cmScrollBarClicked) AND  { Scrollbar click }
         ((Event.InfoPtr = HScrollBar) OR
         (Event.InfoPtr = VScrollBar)) Then Select    { Scrollbar selects us }
         Else If (Event.Command = cmScrollBarChanged) { Scrollbar changed }
         Then Begin
           If (VScrollBar = Event.InfoPtr) Then Begin
             MoveFocus(VScrollBar^.Value);            { Focus us to item }
           End Else If (HScrollBar = Event.InfoPtr)
             Then DrawView;                           { Redraw the view }
         End;
     End;
     evMouseDown: Begin                               { Mouse down event }
       Cw := Size.X DIV NumCols + 1;                  { Column width }
       Oi := Focused;                                 { Hold focused item }
       MakeLocal(Event.Where, Mouse);                 { Localize mouse }
       If MouseInView(Event.Where) Then Ni := Mouse.Y
         + (Size.Y*(Mouse.X DIV Cw))+TopItem          { Calc item to focus }
         Else Ni := Oi;                               { Focus old item }
       Ct := 0;                                       { Clear count value }
       Repeat
         If (Ni <> Oi) Then Begin                     { Item is different }
           MoveFocus(Ni);                             { Move the focus }
           Oi := Focused;                             { Hold as focused item }
         End;
         MakeLocal(Event.Where, Mouse);               { Localize mouse }
         If NOT MouseInView(Event.Where) Then Begin
           If (Event.What = evMouseAuto) Then Inc(Ct);{ Inc auto count }
           If (Ct = MouseAutosToSkip) Then Begin
             Ct := 0;                                 { Reset count }
             If (NumCols = 1) Then Begin              { Only one column }
               If (Mouse.Y < 0) Then Ni := Focused-1; { Move up one item  }
               If (Mouse.Y >= Size.Y) Then
                 Ni := Focused+1;                     { Move down one item }
             End Else Begin                           { Multiple columns }
               If (Mouse.X < 0) Then                  { Mouse x below zero }
                 Ni := Focused-Size.Y;                { Move down 1 column }
               If (Mouse.X >= Size.X) Then            { Mouse x above width }
                 Ni := Focused+Size.Y;                { Move up 1 column }
               If (Mouse.Y < 0) Then                  { Mouse y below zero }
                 Ni := Focused-Focused MOD Size.Y;    { Move up one item }
               If (Mouse.Y > Size.Y) Then             { Mouse y above height }
                 Ni := Focused-Focused MOD
                   Size.Y+Size.Y-1;                   { Move down one item }
             End;
           End;
         End Else Ni := Mouse.Y + (Size.Y*(Mouse.X
           DIV Cw))+TopItem;                          { New item to focus }
       Until NOT MouseEvent(Event, evMouseMove +
         evMouseAuto);                                { Mouse stopped }
       If (Oi <> Ni) Then MoveFocus(Ni);              { Focus moved again }
       If (Event.Double AND (Range > Focused)) Then
         SelectItem(Focused);                         { Select the item }
       ClearEvent(Event);                             { Event was handled }
     End;
   End;
END;

{--TListViewer--------------------------------------------------------------}
{  ChangeBounds -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.ChangeBounds (Var Bounds: TRect);
BEGIN
   Inherited ChangeBounds(Bounds);                    { Call ancestor }
   If (HScrollBar <> Nil) Then                        { Valid horz scrollbar }
     HScrollBar^.SetStep(Size.X DIV NumCols,
       HScrollBar^.ArStep);                           { Update horz bar }
   If (VScrollBar <> Nil) Then                        { Valid vert scrollbar }
     VScrollBar^.SetStep(Size.Y * NumCols,
       VScrollBar^.ArStep);                           { Update vert bar }
END;

{***************************************************************************}
{                     TListViewer OBJECT PRIVATE METHODS                    }
{***************************************************************************}

{--TListViewer--------------------------------------------------------------}
{  FocusItemNum -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Jul99 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE TListViewer.FocusItemNum (Item: Sw_Integer);
BEGIN
   If (Item < 0) Then Item := 0 Else                  { Restrain underflow }
     If (Item >= Range) AND (Range > 0) Then
       Item := Range-1;                               { Restrain overflow }
   If (Range <> 0) Then FocusItem(Item);              { Set focus value }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          TWindow OBJECT METHODS                           }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TWindow------------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TWindow.Init (Var Bounds: TRect; ATitle: TTitleStr; ANumber: Sw_Integer);
BEGIN
   Inherited Init(Bounds);                            { Call ancestor }
   State := State OR sfShadow;                        { View is shadowed }
   Options := Options OR (ofSelectable+ofTopSelect);  { Select options set }
   GrowMode := gfGrowAll + gfGrowRel;                 { Set growmodes }
   Flags := wfMove + wfGrow + wfClose + wfZoom;       { Set flags }
   Title := NewStr(ATitle);                           { Hold title }
   Number := ANumber;                                 { Hold number }
   Palette := wpBlueWindow;                           { Default palette }
   InitFrame;                                         { Initialize frame }
   If (Frame <> Nil) Then Insert(Frame);              { Insert any frame }
   GetBounds(ZoomRect);                               { Default zoom rect }
END;

{--TWindow------------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB              }
{---------------------------------------------------------------------------}
{   This load method will read old original TV data from a stream however   }
{   although a frame view is read for compatability it is disposed of.      }
{---------------------------------------------------------------------------}
CONSTRUCTOR TWindow.Load (Var S: TStream);
VAR I: Integer;
BEGIN
   Inherited Load(S);                                 { Call ancestor }
   S.Read(Flags, SizeOf(Flags));                      { Read window flags }
   S.Read(i, SizeOf(i)); Number:=i;                                { Read window number }
   S.Read(i, SizeOf(i)); Palette:=i;                               { Read window palette }
   S.Read(i, SizeOf(i)); ZoomRect.A.X:=i;                          { Read zoom area x1 }
   S.Read(i, SizeOf(i)); ZoomRect.A.Y:=i;                          { Read zoom area y1 }
   S.Read(i, SizeOf(i)); ZoomRect.B.X:=i;                          { Read zoom area x2 }
   S.Read(i, SizeOf(i)); ZoomRect.B.Y:=i;                          { Read zoom area y2 }
   GetSubViewPtr(S, Frame);                           { Now read frame object }
   Title := S.ReadStr;                                { Read title }
END;

{--TWindow------------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
DESTRUCTOR TWindow.Done;
BEGIN
   Inherited Done;                                    { Call ancestor }
   If (Title <> Nil) Then DisposeStr(Title);          { Dispose title }
END;

{--TWindow------------------------------------------------------------------}
{  GetPalette -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TWindow.GetPalette: PPalette;
CONST  P: ARRAY [wpBlueWindow..wpGrayWindow] Of String[Length(CBlueWindow)] =
  (CBlueWindow, CCyanWindow, CGrayWindow);            { Always normal string }
BEGIN
   GetPalette := PPalette(@P[Palette]);               { Return palette }
END;

{--TWindow------------------------------------------------------------------}
{  GetTitle -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{  Modified 31may2002 PM  (No number included anymore)                      }
{---------------------------------------------------------------------------}
FUNCTION TWindow.GetTitle (MaxSize: Sw_Integer): TTitleStr;
VAR S: String;
BEGIN
   If (Title <> Nil) Then S:=Title^
   Else S := '';
   if Length(S)>MaxSize then
     GetTitle:=Copy(S,1,MaxSize)
   else
     GetTitle:=S;
END;

{--TWindow------------------------------------------------------------------}
{  StandardScrollBar -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB }
{---------------------------------------------------------------------------}
FUNCTION TWindow.StandardScrollBar (AOptions: Word): PScrollBar;
VAR R: TRect; S: PScrollBar;
BEGIN
   GetExtent(R);                                      { View extents }
   If (AOptions AND sbVertical = 0) Then
     R.Assign(R.A.X+2, R.B.Y-1, R.B.X-2, R.B.Y)       { Horizontal scrollbar }
     Else R.Assign(R.B.X-1, R.A.Y+1, R.B.X, R.B.Y-1); { Vertical scrollbar }
   S := New(PScrollBar, Init(R));                     { Create scrollbar }
   Insert(S);                                         { Insert scrollbar }
   If (AOptions AND sbHandleKeyboard <> 0) Then
     S^.Options := S^.Options or ofPostProcess;       { Post process }
   StandardScrollBar := S;                            { Return scrollbar }
END;

{--TWindow------------------------------------------------------------------}
{  Zoom -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 23Sep97 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TWindow.Zoom;
VAR R: TRect; Max, Min: TPoint;
BEGIN
   SizeLimits(Min, Max);                              { Return size limits }
   If ((Size.X <> Max.X) OR (Size.Y <> Max.Y))        { Larger size possible }
   Then Begin
     GetBounds(ZoomRect);                             { Get zoom bounds }
     R.A.X := 0;                                      { Zero x origin }
     R.A.Y := 0;                                      { Zero y origin }
     R.B := Max;                                      { Bounds to max size }
     Locate(R);                                       { Locate the view }
   End Else Locate(ZoomRect);                         { Move to zoom rect }
END;

{--TWindow------------------------------------------------------------------}
{  Close -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 23Sep97 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TWindow.Close;
BEGIN
   If Valid(cmClose) Then Free;                       { Dispose of self }
END;

{--TWindow------------------------------------------------------------------}
{  InitFrame -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TWindow.InitFrame;
VAR
  R: TRect;
BEGIN
  GetExtent(R);
  Frame := New(PFrame, Init(R));
END;

{--TWindow------------------------------------------------------------------}
{  SetState -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Mar98 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TWindow.SetState (AState: Word; Enable: Boolean);
VAR WindowCommands: TCommandSet;
BEGIN
   Inherited SetState(AState, Enable);                { Call ancestor }
   If (AState = sfSelected) Then
     SetState(sfActive, Enable);                      { Set active state }
   If (AState = sfSelected) OR ((AState = sfExposed)
   AND (State AND sfSelected <> 0)) Then Begin        { View is selected }
     WindowCommands := [cmNext, cmPrev];              { Set window commands }
     If (Flags AND (wfGrow + wfMove) <> 0) Then
       WindowCommands := WindowCommands + [cmResize]; { Add resize command }
     If (Flags AND wfClose <> 0) Then
       WindowCommands := WindowCommands + [cmClose];  { Add close command }
     If (Flags AND wfZoom <> 0) Then
       WindowCommands := WindowCommands + [cmZoom];   { Add zoom command }
     If Enable Then EnableCommands(WindowCommands)    { Enable commands }
       Else DisableCommands(WindowCommands);          { Disable commands }
   End;
END;

{--TWindow------------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Mar98 LdB             }
{---------------------------------------------------------------------------}
{  You can save data to the stream compatable with the old original TV by   }
{  temporarily turning off the ofGrafVersion making the call to this store  }
{  routine and resetting the ofGrafVersion flag after the call.             }
{---------------------------------------------------------------------------}
PROCEDURE TWindow.Store (Var S: TStream);
VAR i: Integer;
BEGIN
   TGroup.Store(S);                                   { Call group store }
   S.Write(Flags, SizeOf(Flags));                     { Write window flags }
   i:=Number;S.Write(i, SizeOf(i));                   { Write window number }
   i:=Palette;S.Write(i, SizeOf(i));                  { Write window palette }
   i:=ZoomRect.A.X;S.Write(i, SizeOf(i));             { Write zoom area x1 }
   i:=ZoomRect.A.Y;S.Write(i, SizeOf(i));             { Write zoom area y1 }
   i:=ZoomRect.B.X;S.Write(i, SizeOf(i));             { Write zoom area x2 }
   i:=ZoomRect.B.Y;S.Write(i, SizeOf(i));             { Write zoom area y2 }
   PutSubViewPtr(S, Frame);                           { Write any frame }
   S.WriteStr(Title);                                 { Write title string }
END;

{--TWindow------------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 11Aug99 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TWindow.HandleEvent (Var Event: TEvent);
VAR
  Min, Max: TPoint; Limits: TRect;

   PROCEDURE DragWindow (Mode: Byte);
   VAR Limits: TRect; Min, Max: TPoint;
   BEGIN
     Owner^.GetExtent(Limits);                        { Get owner extents }
     SizeLimits(Min, Max);                            { Restrict size }
     DragView(Event, DragMode OR Mode, Limits, Min,
       Max);                                          { Drag the view }
     ClearEvent(Event);                               { Clear the event }
   END;

BEGIN
   Inherited HandleEvent(Event);                      { Call ancestor }
   Case Event.What Of
     evNothing: Exit;                                 { Speeds up exit }
     evCommand:                                       { COMMAND EVENT }
       Case Event.Command Of                          { Command type case }
         cmResize:                                    { RESIZE COMMAND }
           If (Flags AND (wfMove + wfGrow) <> 0)      { Window can resize }
           AND (Owner <> Nil) Then Begin              { Valid owner }
             Owner^.GetExtent(Limits);                { Owners extents }
             SizeLimits(Min, Max);                    { Check size limits }
             DragView(Event, DragMode OR (Flags AND
               (wfMove + wfGrow)), Limits, Min, Max); { Drag the view }
             ClearEvent(Event);                       { Clear the event }
           End;
         cmClose:                                     { CLOSE COMMAND }
           If (Flags AND wfClose <> 0) AND            { Close flag set }
           ((Event.InfoPtr = Nil) OR                  { None specific close }
           (Event.InfoPtr = @Self)) Then Begin        { Close to us }
             ClearEvent(Event);                       { Clear the event }
             If (State AND sfModal = 0) Then Close    { Non modal so close }
             Else Begin                               { Modal window }
               Event.What := evCommand;               { Command event }
               Event.Command := cmCancel;             { Cancel command }
               PutEvent(Event);                       { Place on queue }
               ClearEvent(Event);                     { Clear the event }
             End;
           End;
         cmZoom:                                      { ZOOM COMMAND }
           If (Flags AND wfZoom <> 0) AND             { Zoom flag set }
           ((Event.InfoPtr = Nil) OR                  { No specific zoom }
           (Event.InfoPtr = @Self)) Then Begin
             Zoom;                                    { Zoom our window }
             ClearEvent(Event);                       { Clear the event }
           End;
       End;
     evBroadcast:                                     { BROADCAST EVENT }
       If (Event.Command = cmSelectWindowNum) AND
       (Event.InfoInt = Number) AND                   { Select our number }
       (Options AND ofSelectable <> 0) Then Begin     { Is view selectable }
         Select;                                      { Select our view }
         ClearEvent(Event);                           { Clear the event }
       End;
     evKeyDown: Begin                                 { KEYDOWN EVENT }
       Case Event.KeyCode Of
         kbTab: Begin                                 { TAB KEY }
           FocusNext(False);                          { Select next view }
           ClearEvent(Event);                         { Clear the event }
         End;
         kbShiftTab: Begin                            { SHIFT TAB KEY }
           FocusNext(True);                           { Select prior view }
           ClearEvent(Event);                         { Clear the event }
         End;
       End;
     End;
   End;                                               { Event.What case end }
END;

{--TWindow------------------------------------------------------------------}
{  SizeLimits -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 15Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TWindow.SizeLimits (Var Min, Max: TPoint);
BEGIN
   Inherited SizeLimits(Min, Max);                    { View size limits }
   Min.X := MinWinSize.X;                             { Set min x size }
   Min.Y := MinWinSize.Y;                             { Set min y size }
END;



{--TView--------------------------------------------------------------------}
{  Exposed -> Platforms DOS/DPMI/WIN/OS2 - Checked 17Sep97 LdB              }
{---------------------------------------------------------------------------}
function TView.do_ExposedRec1(x1,x2:sw_integer; p:PView):boolean;
var
  G : PGroup;
  dy,dx : sw_integer;
begin
  while true do
   begin
     p:=p^.Next;
     G:=p^.Owner;
     if p=staticVar2.target then
      begin
        do_exposedRec1:=do_exposedRec2(x1,x2,G);
        Exit;
      end;
     dy:=p^.origin.y;
     dx:=p^.origin.x;
     if ((p^.state and sfVisible)<>0) and (staticVar2.y>=dy) then
      begin
        if staticVar2.y<dy+p^.size.y then
         begin
           if x1<dx then
            begin
              if x2<=dx then
               continue;
              if x2>dx+p^.size.x then
               begin
                 if do_exposedRec1(x1,dx,p) then
                  begin
                    do_exposedRec1:=True;
                    Exit;
                  end;
                 x1:=dx+p^.size.x;
               end
              else
               x2:=dx;
           end
          else
           begin
             if x1<dx+p^.size.x then
              x1:=dx+p^.size.x;
             if x1>=x2 then
              begin
                do_exposedRec1:=False;
                Exit;
              end;
           end;
        end;
      end;
   end;
end;


function TView.do_ExposedRec2(x1,x2:Sw_integer; p:PView):boolean;
var
  G : PGroup;
  savedStat : TStatVar2;
begin
  if (p^.state and sfVisible)=0 then
   do_ExposedRec2:=false
  else
   begin
     G:=p^.Owner;
     if (G=Nil) or (G^.Buffer<>Nil) then
      do_ExposedRec2:=true
     else
      begin
        savedStat:=staticVar2;
        inc(staticVar2.y,p^.origin.y);
        inc(x1,p^.origin.x);
        inc(x2,p^.origin.x);
        staticVar2.target:=p;
        if (staticVar2.y<G^.clip.a.y) or (staticVar2.y>=G^.clip.b.y) then
         do_ExposedRec2:=false
        else
         begin
           if (x1<G^.clip.a.x) then
            x1:=G^.clip.a.x;
           if (x2>G^.clip.b.x) then
            x2:=G^.clip.b.x;
           if (x1>=x2) then
            do_ExposedRec2:=false
           else
            do_ExposedRec2:=do_exposedRec1(x1,x2,G^.Last);
         end;
        staticVar2 := savedStat;
      end;
   end;
end;


function TView.Exposed: Boolean;
var
  OK : boolean;
  y  : sw_integer;
begin
  if ((State and sfExposed)<>0) and (Size.X>0) and (Size.Y>0) then
   begin
     OK:=false;
     y:=0;
     while (y<Size.Y) and (not OK) do
      begin
        staticVar2.y:=y;
        OK:=do_ExposedRec2(0,Size.X,@Self);
        inc(y);
      end;
     Exposed:=OK;
   end
  else
   Exposed:=False
end;


{--TView--------------------------------------------------------------------}
{  MakeLocal -> Platforms DOS/DPMI/WIN/OS2 - Checked 12Sep97 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TView.MakeLocal (Source: TPoint; Var Dest: TPoint);
var
  cur : PView;
begin
  cur:=@Self;
  Dest:=Source;
  repeat
    dec(Dest.X,cur^.Origin.X);
    if dest.x<0 then
     break;
    dec(Dest.Y,cur^.Origin.Y);
    if dest.y<0 then
     break;
    cur:=cur^.Owner;
  until cur=nil;
end;


{--TView--------------------------------------------------------------------}
{  MakeGlobal -> Platforms DOS/DPMI/WIN/OS2 - Checked 12Sep97 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TView.MakeGlobal (Source: TPoint; Var Dest: TPoint);
var
  cur : PView;
begin
  cur:=@Self;
  Dest:=Source;
  repeat
    inc(Dest.X,cur^.Origin.X);
    inc(Dest.Y,cur^.Origin.Y);
    cur:=cur^.Owner;
  until cur=nil;
end;


procedure TView.do_writeViewRec1(x1,x2:Sw_integer; p:PView; shadowCounter:Sw_integer);
var
  G      : PGroup;
  c      : Word;
  BufPos,
  SrcPos,
  l,dx : Sw_integer;
begin
  repeat
    p:=p^.Next;
    if (p=staticVar2.target) then
     begin
       G:=p^.Owner;
       if (G^.buffer<>Nil) then
        begin
          BufPos:=G^.size.x * staticVar2.y + x1;
          SrcPos:=x1 - staticVar2.offset;
          l:=x2-x1;
          if (shadowCounter=0) then
           move(staticVar1^[SrcPos],PVideoBuf(G^.buffer)^[BufPos],l shl 1)
          else
           begin { paint with shadowAttr }
             while (l>0) do
              begin
                c:=staticVar1^[SrcPos];
                WordRec(c).hi:=shadowAttr;
                PVideoBuf(G^.buffer)^[BufPos]:=c;
                inc(BufPos);
                inc(SrcPos);
                dec(l);
              end;
           end;
        end;
       if G^.lockFlag=0 then
         do_writeViewRec2(x1,x2,G,shadowCounter);
       exit;
     end; { p=staticVar2.target }

    if ((p^.state and sfVisible)<>0) and (staticVar2.y>=p^.Origin.Y) then
     begin
       if staticVar2.y<p^.Origin.Y+p^.size.Y then
        begin
          if x1<p^.origin.x then
           begin
             if x2<=p^.origin.x then
              continue;
             do_writeViewRec1(x1,p^.origin.x,p,shadowCounter);
             x1:=p^.origin.x;
           end;
          dx:=p^.origin.x+p^.size.x;
          if (x2<=dx) then
           exit;
          if (x1<dx) then
           x1:=dx;
          inc(dx,shadowSize.x);
          if ((p^.state and sfShadow)<>0) and (staticVar2.y>=p^.origin.y+shadowSize.y) then
           if (x1>dx) then
            continue
           else
            begin
              inc(shadowCounter);
              if (x2<=dx) then
               continue
              else
               begin
                 do_writeViewRec1(x1,dx,p,shadowCounter);
                 x1:=dx;
                 dec(shadowCounter);
                 continue;
               end;
            end
           else
            continue;
        end;

       if ((p^.state and sfShadow)<>0) and (staticVar2.y<p^.origin.y+p^.size.y+shadowSize.y) then
        begin
          dx:=p^.origin.x+shadowSize.x;
          if x1<dx then
           begin
             if x2<=dx then
              continue;
             do_writeViewRec1(x1,dx,p,shadowCounter);
             x1:=dx;
           end;
          inc(dx,p^.size.x);
          if x1>=dx then
           continue;
          inc(shadowCounter);
          if x2<=dx then
           continue
          else
           begin
             do_writeViewRec1(x1,dx,p,shadowCounter);
             x1:=dx;
             dec(shadowCounter);
           end;
        end;
     end;
  until false;
end;


procedure TView.do_writeViewRec2(x1,x2:Sw_integer; p:PView; shadowCounter:Sw_integer);
var
  savedStatics : TstatVar2;
  dx : Sw_integer;
  G  : PGroup;
begin
  G:=P^.Owner;
  if ((p^.State and sfVisible) <> 0) and (G<>Nil) then
   begin
     savedStatics:=staticVar2;
     inc(staticVar2.y,p^.Origin.Y);
     dx:=p^.Origin.X;
     inc(x1,dx);
     inc(x2,dx);
     inc(staticVar2.offset,dx);
     staticVar2.target:=p;
     if (staticVar2.y >= G^.clip.a.y) and (staticVar2.y < G^.clip.b.y) then
      begin
        if (x1<g^.clip.a.x) then
         x1 := g^.clip.a.x;
        if (x2>g^.clip.b.x) then
         x2 := g^.clip.b.x;
        if x1<x2 then
         do_writeViewRec1(x1,x2,G^.Last,shadowCounter);
      end;
     staticVar2 := savedStatics;
   end;
end;


procedure TView.do_WriteView(x1,x2,y:Sw_integer; var Buf);
begin
  if (y>=0) and (y<Size.Y) then
   begin
     if x1<0 then
      x1:=0;
     if x2>Size.X then
      x2:=Size.X;
     if x1<x2 then
      begin
        staticVar2.offset:=x1;
        staticVar2.y:=y;
        staticVar1:=@Buf;
        do_writeViewRec2( x1, x2, @Self, 0 );
      end;
   end;
end;


procedure TView.WriteBuf(X, Y, W, H: Sw_Integer; var Buf);
var
  i : Sw_integer;
begin
  if h>0 then
   for i:= 0 to h-1 do
    do_writeView(X,X+W,Y+i,TVideoBuf(Buf)[W*i]);
end;


procedure TView.WriteChar(X,Y:Sw_Integer; C:Char; Color:Byte; Count:Sw_Integer);
var
  B : TDrawBuffer;
  myChar : word;
  i : Sw_integer;
begin
  myChar:=MapColor(Color);
  myChar:=(myChar shl 8) + ord(C);
  if Count>0 then
   begin
     if Count>maxViewWidth then
      Count:=maxViewWidth;
     for i:=0 to Count-1 do
      B[i]:=myChar;
     do_writeView(X,X+Count,Y,B);
   end;
  DrawScreenBuf(false);
end;


procedure TView.WriteLine(X, Y, W, H: Sw_Integer; var Buf);
var
  i:Sw_integer;
begin
  if h>0 then
   for i:=0 to h-1 do
    do_writeView(x,x+w,y+i,buf);
  DrawScreenBuf(false);
end;


procedure TView.WriteStr(X, Y: Sw_Integer; Str: String; Color: Byte);
var
  l,i : Sw_word;
  B : TDrawBuffer;
  myColor : word;
begin
  l:=length(Str);
  if l>0 then
   begin
     if l>maxViewWidth then
      l:=maxViewWidth;
     MyColor:=MapColor(Color);
     MyColor:=MyColor shl 8;
     for i:=0 to l-1 do
       B[i]:=MyColor+ord(Str[i+1]);
     do_writeView(x,x+l,y,b);
   end;
  DrawScreenBuf(false);
end;


procedure TView.DragView(Event: TEvent; Mode: Byte;
  var Limits: TRect; MinSize, MaxSize: TPoint);
var
  P, S: TPoint;
  SaveBounds: TRect;

  procedure MoveGrow(P, S: TPoint);
  var
    R: TRect;
  begin
    S.X := Min(Max(S.X, MinSize.X), MaxSize.X);
    S.Y := Min(Max(S.Y, MinSize.Y), MaxSize.Y);
    P.X := Min(Max(P.X, Limits.A.X - S.X + 1), Limits.B.X - 1);
    P.Y := Min(Max(P.Y, Limits.A.Y - S.Y + 1), Limits.B.Y - 1);
    if Mode and dmLimitLoX <> 0 then P.X := Max(P.X, Limits.A.X);
    if Mode and dmLimitLoY <> 0 then P.Y := Max(P.Y, Limits.A.Y);
    if Mode and dmLimitHiX <> 0 then P.X := Min(P.X, Limits.B.X - S.X);
    if Mode and dmLimitHiY <> 0 then P.Y := Min(P.Y, Limits.B.Y - S.Y);
    R.Assign(P.X, P.Y, P.X + S.X, P.Y + S.Y);
    Locate(R);
  end;

  procedure Change(DX, DY: Sw_Integer);
  begin
    if (Mode and dmDragMove <> 0) and (Event.KeyShift{GetShiftState} and $03 = 0) then
    begin
      Inc(P.X, DX);
      Inc(P.Y, DY);
    end else
    if (Mode and dmDragGrow <> 0) and (Event.KeyShift{GetShiftState} and $03 <> 0) then
    begin
      Inc(S.X, DX);
      Inc(S.Y, DY);
    end;
  end;

  procedure Update(X, Y: Sw_Integer);
  begin
    if Mode and dmDragMove <> 0 then
    begin
      P.X := X;
      P.Y := Y;
    end;
  end;

begin
  SetState(sfDragging, True);
  if Event.What = evMouseDown then
  begin
    if Mode and dmDragMove <> 0 then
    begin
      P.X := Origin.X - Event.Where.X;
      P.Y := Origin.Y - Event.Where.Y;
      repeat
        Inc(Event.Where.X, P.X);
        Inc(Event.Where.Y, P.Y);
        MoveGrow(Event.Where, Size);
      until not MouseEvent(Event, evMouseMove);
      {We need to process the mouse-up event, since not all terminals
       send drag events.}
      Inc(Event.Where.X, P.X);
      Inc(Event.Where.Y, P.Y);
      MoveGrow(Event.Where, Size);
    end else
    begin
      P.X := Size.X - Event.Where.X;
      P.Y := Size.Y - Event.Where.Y;
      repeat
        Inc(Event.Where.X, P.X);
        Inc(Event.Where.Y, P.Y);
        MoveGrow(Origin, Event.Where);
      until not MouseEvent(Event, evMouseMove);
      {We need to process the mouse-up event, since not all terminals
       send drag events.}
      Inc(Event.Where.X, P.X);
      Inc(Event.Where.Y, P.Y);
      MoveGrow(Origin, Event.Where);
    end;
  end else
  begin
    GetBounds(SaveBounds);
    repeat
      P := Origin;
      S := Size;
      KeyEvent(Event);
      case Event.KeyCode and $FF00 of
        kbLeft: Change(-1, 0);
        kbRight: Change(1, 0);
        kbUp: Change(0, -1);
        kbDown: Change(0, 1);
        kbCtrlLeft: Change(-8, 0);
        kbCtrlRight: Change(8, 0);
        kbHome: Update(Limits.A.X, P.Y);
        kbEnd: Update(Limits.B.X - S.X, P.Y);
        kbPgUp: Update(P.X, Limits.A.Y);
        kbPgDn: Update(P.X, Limits.B.Y - S.Y);
      end;
      MoveGrow(P, S);
    until (Event.KeyCode = kbEnter) or (Event.KeyCode = kbEsc);
    if Event.KeyCode = kbEsc then
      Locate(SaveBounds);
  end;
  SetState(sfDragging, False);
end;


{***************************************************************************}
{                         TScroller OBJECT METHODS                          }
{***************************************************************************}

PROCEDURE TScroller.ScrollDraw;
VAR D: TPoint;
BEGIN
   If (HScrollBar<>Nil) Then D.X := HScrollBar^.Value
     Else D.X := 0;                                   { Horz scroll value }
   If (VScrollBar<>Nil) Then D.Y := VScrollBar^.Value
     Else D.Y := 0;                                   { Vert scroll value }
   If (D.X<>Delta.X) OR (D.Y<>Delta.Y) Then Begin     { View has moved }
     SetCursor(Cursor.X+Delta.X-D.X,
       Cursor.Y+Delta.Y-D.Y);                         { Move the cursor }
     Delta := D;                                      { Set new delta }
     If (DrawLock<>0) Then DrawFlag := True           { Draw will need draw }
       Else DrawView;                                 { Redraw the view }
   End;
END;

PROCEDURE TScroller.SetLimit (X, Y: Sw_Integer);
VAR PState: Word;
BEGIN
   Limit.X := X;                                      { Hold x limit }
   Limit.Y := Y;                                      { Hold y limit }
   Inc(DrawLock);                                     { Set draw lock }
   If (HScrollBar<>Nil) Then Begin
     PState := HScrollBar^.State;                     { Hold bar state }
     HScrollBar^.State := PState AND NOT sfVisible;   { Temp not visible }
     HScrollBar^.SetParams(HScrollBar^.Value, 0,
       X-Size.X, Size.X-1, HScrollBar^.ArStep);       { Set horz scrollbar }
     HScrollBar^.State := PState;                     { Restore bar state }
   End;
   If (VScrollBar<>Nil) Then Begin
     PState := VScrollBar^.State;                     { Hold bar state }
     VScrollBar^.State := PState AND NOT sfVisible;   { Temp not visible }
     VScrollBar^.SetParams(VScrollBar^.Value, 0,
       Y-Size.Y, Size.Y-1, VScrollBar^.ArStep);       { Set vert scrollbar }
     VScrollBar^.State := PState;                     { Restore bar state }
   End;
   Dec(DrawLock);                                     { Release draw lock }
   CheckDraw;                                         { Check need to draw }
END;

{***************************************************************************}
{                      TScroller OBJECT PRIVATE METHODS                     }
{***************************************************************************}
PROCEDURE TScroller.CheckDraw;
BEGIN
   If (DrawLock = 0) AND DrawFlag Then Begin          { Clear & draw needed }
     DrawFlag := False;                               { Clear draw flag }
     DrawView;                                        { Draw now }
   End;
END;



{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          TGroup OBJECT METHODS                            }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}




{--TGroup-------------------------------------------------------------------}
{  Lock -> Platforms DOS/DPMI/WIN/OS2 - Checked 23Sep97 LdB                 }
{---------------------------------------------------------------------------}
{$ifndef  NoLock}
{$define UseLock}
{$endif ndef  NoLock}
PROCEDURE TGroup.Lock;
BEGIN
{$ifdef UseLock}
   {If (Buffer <> Nil) OR (LockFlag <> 0)
     Then} Inc(LockFlag);                              { Increment count }
{$endif UseLock}
END;

{--TGroup-------------------------------------------------------------------}
{  UnLock -> Platforms DOS/DPMI/WIN/OS2 - Checked 23Sep97 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TGroup.Unlock;
BEGIN
{$ifdef UseLock}
   If (LockFlag <> 0) Then Begin
     Dec(LockFlag);                                   { Decrement count }
     If (LockFlag = 0) Then DrawView;                 { Lock release draw }
   End;
{$endif UseLock}
END;


{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                         WINDOW MESSAGE ROUTINES                           }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  Message -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB           }
{---------------------------------------------------------------------------}
FUNCTION Message (Receiver: PView; What, Command: Word;
  InfoPtr: Pointer): Pointer;
VAR Event: TEvent;
BEGIN
   Message := Nil;                                    { Preset nil }
   If (Receiver <> Nil) Then Begin                    { Valid receiver }
     Event.What := What;                              { Set what }
     Event.Command := Command;                        { Set command }
     Event.Id := 0;                                   { Zero id field }
     Event.Data := 0;                                 { Zero data field }
     Event.InfoPtr := InfoPtr;                        { Set info ptr }
     Receiver^.HandleEvent(Event);                    { Pass to handler }
     If (Event.What = evNothing) Then
       Message := Event.InfoPtr;                      { Return handler }
   End;
END;

{---------------------------------------------------------------------------}
{  NewMessage -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION NewMessage (P: PView; What, Command: Word; Id: Sw_Integer;
  Data: Real; InfoPtr: Pointer): Pointer;
VAR Event: TEvent;
BEGIN
   NewMessage := Nil;                                 { Preset failure }
   If (P <> Nil) Then Begin
     Event.What := What;                              { Set what }
     Event.Command := Command;                        { Set event command }
     Event.Id := Id;                                  { Set up Id }
     Event.Data := Data;                              { Set up data }
     Event.InfoPtr := InfoPtr;                        { Set up event ptr }
     P^.HandleEvent(Event);                           { Send to view }
     If (Event.What = evNothing) Then
       NewMessage := Event.InfoPtr;                   { Return handler }
   End;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                            NEW VIEW ROUTINES                              }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  CreateIdScrollBar -> Platforms DOS/DPMI/WIN/NT/OS2 - Checked 22May97 LdB }
{---------------------------------------------------------------------------}
FUNCTION CreateIdScrollBar (X, Y, Size, Id: Sw_Integer; Horz: Boolean): PScrollBar;
VAR R: TRect; P: PScrollBar;
BEGIN
   If Horz Then R.Assign(X, Y, X+Size, Y+1) Else      { Horizontal bar }
     R.Assign(X, Y, X+1, Y+Size);                     { Vertical bar }
   P := New(PScrollBar, Init(R));                     { Create scrollbar }
   If (P <> Nil) Then Begin
     P^.Id := Id;                                     { Set scrollbar id }
     P^.Options := P^.Options OR ofPostProcess;       { Set post processing }
   End;
   CreateIdScrollBar := P;                            { Return scrollbar }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                      OBJECT REGISTRATION PROCEDURES                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  RegisterViews -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 28May97 LdB     }
{---------------------------------------------------------------------------}
PROCEDURE RegisterViews;
BEGIN
   RegisterType(RView);                               { Register views }
   RegisterType(RFrame);                              { Register frame }
   RegisterType(RScrollBar);                          { Register scrollbar }
   RegisterType(RScroller);                           { Register scroller }
   RegisterType(RListViewer);                         { Register listview }
   RegisterType(RGroup);                              { Register group }
   RegisterType(RWindow);                             { Register window }
END;

END.
