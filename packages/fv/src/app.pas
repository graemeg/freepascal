{********[ SOURCE FILE OF GRAPHICAL FREE VISION ]**********}
{                                                          }
{   System independent GRAPHICAL clone of APP.PAS          }
{                                                          }
{   Interface Copyright (c) 1992 Borland International     }
{                                                          }
{   Copyright (c) 1996, 1997, 1998, 1999 by Leon de Boer   }
{   ldeboer@attglobal.net  - primary e-mail addr           }
{   ldeboer@starwon.com.au - backup e-mail addr            }
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

UNIT App;

{2.0 compatibility}
{$ifdef VER2_0}
  {$macro on}
  {$define resourcestring := const}
{$endif}

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
       Windows,                                       { Standard units }
   {$ENDIF}

   {$IFDEF OS_OS2}                                    { OS2 CODE }
     {$IFDEF PPC_FPC}
     Os2Def, DosCalls, PmWin,                       { Standard units }
     {$ELSE}
      Os2Def, Os2Base, OS2PmApi,                       { Standard units }
     {$ENDIF}
   {$ENDIF}
   Dos,
   Video,
   FVCommon, {Memory,}                                { GFV standard units }
   Objects, Drivers, Views, Menus, HistList, Dialogs,
   msgbox, fvconsts;

{***************************************************************************}
{                              PUBLIC CONSTANTS                             }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                  STANDARD APPLICATION COMMAND CONSTANTS                   }
{---------------------------------------------------------------------------}
CONST
   cmNew       = 30;                                  { Open new file }
   cmOpen      = 31;                                  { Open a file }
   cmSave      = 32;                                  { Save current }
   cmSaveAs    = 33;                                  { Save current as }
   cmSaveAll   = 34;                                  { Save all files }
   cmChangeDir = 35;                                  { Change directories }
   cmDosShell  = 36;                                  { Dos shell }
   cmCloseAll  = 37;                                  { Close all windows }

{---------------------------------------------------------------------------}
{                       TApplication PALETTE ENTRIES                        }
{---------------------------------------------------------------------------}
CONST
   apColor      = 0;                                  { Coloured app }
   apBlackWhite = 1;                                  { B&W application }
   apMonochrome = 2;                                  { Monochrome app }

{---------------------------------------------------------------------------}
{                           TBackGround PALETTES                            }
{---------------------------------------------------------------------------}
CONST
   CBackground = #1;                                  { Background colour }

{---------------------------------------------------------------------------}
{                           TApplication PALETTES                           }
{---------------------------------------------------------------------------}
CONST
  { Turbo Vision 1.0 Color Palettes }

   CColor =
         #$81#$70#$78#$74#$20#$28#$24#$17#$1F#$1A#$31#$31#$1E#$71#$1F +
     #$37#$3F#$3A#$13#$13#$3E#$21#$3F#$70#$7F#$7A#$13#$13#$70#$7F#$7E +
     #$70#$7F#$7A#$13#$13#$70#$70#$7F#$7E#$20#$2B#$2F#$78#$2E#$70#$30 +
     #$3F#$3E#$1F#$2F#$1A#$20#$72#$31#$31#$30#$2F#$3E#$31#$13#$38#$00;

   CBlackWhite =
         #$70#$70#$78#$7F#$07#$07#$0F#$07#$0F#$07#$70#$70#$07#$70#$0F +
     #$07#$0F#$07#$70#$70#$07#$70#$0F#$70#$7F#$7F#$70#$07#$70#$07#$0F +
     #$70#$7F#$7F#$70#$07#$70#$70#$7F#$7F#$07#$0F#$0F#$78#$0F#$78#$07 +
     #$0F#$0F#$0F#$70#$0F#$07#$70#$70#$70#$07#$70#$0F#$07#$07#$78#$00;

   CMonochrome =
         #$70#$07#$07#$0F#$70#$70#$70#$07#$0F#$07#$70#$70#$07#$70#$00 +
     #$07#$0F#$07#$70#$70#$07#$70#$00#$70#$70#$70#$07#$07#$70#$07#$00 +
     #$70#$70#$70#$07#$07#$70#$70#$70#$0F#$07#$07#$0F#$70#$0F#$70#$07 +
     #$0F#$0F#$07#$70#$07#$07#$70#$07#$07#$07#$70#$0F#$07#$07#$70#$00;

   { Turbo Vision 2.0 Color Palettes }

   CAppColor =
         #$71#$70#$78#$74#$20#$28#$24#$17#$1F#$1A#$31#$31#$1E#$71#$1F +
     #$37#$3F#$3A#$13#$13#$3E#$21#$3F#$70#$7F#$7A#$13#$13#$70#$7F#$7E +
     #$70#$7F#$7A#$13#$13#$70#$70#$7F#$7E#$20#$2B#$2F#$78#$2E#$70#$30 +
     #$3F#$3E#$1F#$2F#$1A#$20#$72#$31#$31#$30#$2F#$3E#$31#$13#$38#$00 +
     #$17#$1F#$1A#$71#$71#$1E#$17#$1F#$1E#$20#$2B#$2F#$78#$2E#$10#$30 +
     #$3F#$3E#$70#$2F#$7A#$20#$12#$31#$31#$30#$2F#$3E#$31#$13#$38#$00 +
     #$37#$3F#$3A#$13#$13#$3E#$30#$3F#$3E#$20#$2B#$2F#$78#$2E#$30#$70 +
     #$7F#$7E#$1F#$2F#$1A#$20#$32#$31#$71#$70#$2F#$7E#$71#$13#$38#$00;

   CAppBlackWhite =
         #$70#$70#$78#$7F#$07#$07#$0F#$07#$0F#$07#$70#$70#$07#$70#$0F +
     #$07#$0F#$07#$70#$70#$07#$70#$0F#$70#$7F#$7F#$70#$07#$70#$07#$0F +
     #$70#$7F#$7F#$70#$07#$70#$70#$7F#$7F#$07#$0F#$0F#$78#$0F#$78#$07 +
     #$0F#$0F#$0F#$70#$0F#$07#$70#$70#$70#$07#$70#$0F#$07#$07#$78#$00 +
     #$07#$0F#$0F#$07#$70#$07#$07#$0F#$0F#$70#$78#$7F#$08#$7F#$08#$70 +
     #$7F#$7F#$7F#$0F#$70#$70#$07#$70#$70#$70#$07#$7F#$70#$07#$78#$00 +
     #$70#$7F#$7F#$70#$07#$70#$70#$7F#$7F#$07#$0F#$0F#$78#$0F#$78#$07 +
     #$0F#$0F#$0F#$70#$0F#$07#$70#$70#$70#$07#$70#$0F#$07#$07#$78#$00;

   CAppMonochrome =
         #$70#$07#$07#$0F#$70#$70#$70#$07#$0F#$07#$70#$70#$07#$70#$00 +
     #$07#$0F#$07#$70#$70#$07#$70#$00#$70#$70#$70#$07#$07#$70#$07#$00 +
     #$70#$70#$70#$07#$07#$70#$70#$70#$0F#$07#$07#$0F#$70#$0F#$70#$07 +
     #$0F#$0F#$07#$70#$07#$07#$70#$07#$07#$07#$70#$0F#$07#$07#$70#$00 +
     #$70#$70#$70#$07#$07#$70#$70#$70#$0F#$07#$07#$0F#$70#$0F#$70#$07 +
     #$0F#$0F#$07#$70#$07#$07#$70#$07#$07#$07#$70#$0F#$07#$07#$70#$00 +
     #$70#$70#$70#$07#$07#$70#$70#$70#$0F#$07#$07#$0F#$70#$0F#$70#$07 +
     #$0F#$0F#$07#$70#$07#$07#$70#$07#$07#$07#$70#$0F#$07#$07#$70#$00;

{---------------------------------------------------------------------------}
{                     STANDRARD HELP CONTEXT CONSTANTS                      }
{---------------------------------------------------------------------------}
CONST
{ Note: range $FF00 - $FFFF of help contexts are reserved by Borland }
   hcNew       = $FF01;                               { New file help }
   hcOpen      = $FF02;                               { Open file help }
   hcSave      = $FF03;                               { Save file help }
   hcSaveAs    = $FF04;                               { Save file as help }
   hcSaveAll   = $FF05;                               { Save all files help }
   hcChangeDir = $FF06;                               { Change dir help }
   hcDosShell  = $FF07;                               { Dos shell help }
   hcExit      = $FF08;                               { Exit program help }

   hcUndo      = $FF10;                               { Clipboard undo help }
   hcCut       = $FF11;                               { Clipboard cut help }
   hcCopy      = $FF12;                               { Clipboard copy help }
   hcPaste     = $FF13;                               { Clipboard paste help }
   hcClear     = $FF14;                               { Clipboard clear help }

   hcTile      = $FF20;                               { Desktop tile help }
   hcCascade   = $FF21;                               { Desktop cascade help }
   hcCloseAll  = $FF22;                               { Desktop close all }
   hcResize    = $FF23;                               { Window resize help }
   hcZoom      = $FF24;                               { Window zoom help }
   hcNext      = $FF25;                               { Window next help }
   hcPrev      = $FF26;                               { Window previous help }
   hcClose     = $FF27;                               { Window close help }

{***************************************************************************}
{                        PUBLIC OBJECT DEFINITIONS                          }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                  TBackGround OBJECT - BACKGROUND OBJECT                   }
{---------------------------------------------------------------------------}
TYPE
   TBackGround = OBJECT (TView)
         Pattern: Char;                               { Background pattern }
      CONSTRUCTOR Init (Var Bounds: TRect; APattern: Char);
      CONSTRUCTOR Load (Var S: TStream);
      FUNCTION GetPalette: PPalette; Virtual;
      PROCEDURE Draw; Virtual;
      PROCEDURE Store (Var S: TStream);
   END;
   PBackGround = ^TBackGround;

{---------------------------------------------------------------------------}
{                     TDeskTop OBJECT - DESKTOP OBJECT                      }
{---------------------------------------------------------------------------}
TYPE
   TDeskTop = OBJECT (TGroup)
         Background      : PBackground;               { Background view }
         TileColumnsFirst: Boolean;                   { Tile direction }
      CONSTRUCTOR Init (Var Bounds: TRect);
      CONSTRUCTOR Load (Var S: TStream);
      PROCEDURE TileError; Virtual;
      PROCEDURE InitBackGround; Virtual;
      PROCEDURE Tile (Var R: TRect);
      PROCEDURE Store (Var S: TStream);
      PROCEDURE Cascade (Var R: TRect);
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
   END;
   PDeskTop = ^TDeskTop;

{---------------------------------------------------------------------------}
{                  TProgram OBJECT - PROGRAM ANCESTOR OBJECT                }
{---------------------------------------------------------------------------}
TYPE
   TProgram = OBJECT (TGroup)
      CONSTRUCTOR Init;
      DESTRUCTOR Done; Virtual;
      FUNCTION GetPalette: PPalette; Virtual;
      FUNCTION CanMoveFocus: Boolean;
      FUNCTION ValidView (P: PView): PView;
      FUNCTION InsertWindow (P: PWindow): PWindow;
      FUNCTION ExecuteDialog (P: PDialog; Data: Pointer): Word;
      PROCEDURE Run; Virtual;
      PROCEDURE Idle; Virtual;
      PROCEDURE InitScreen; Virtual;
{      procedure DoneScreen; virtual;}
      PROCEDURE InitDeskTop; Virtual;
      PROCEDURE OutOfMemory; Virtual;
      PROCEDURE InitMenuBar; Virtual;
      PROCEDURE InitStatusLine; Virtual;
      PROCEDURE SetScreenMode (Mode: Word);
      PROCEDURE SetScreenVideoMode(const Mode: TVideoMode);
      PROCEDURE PutEvent (Var Event: TEvent); Virtual;
      PROCEDURE GetEvent (Var Event: TEvent); Virtual;
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
   END;
   PProgram = ^TProgram;

{---------------------------------------------------------------------------}
{                  TApplication OBJECT - APPLICATION OBJECT                 }
{---------------------------------------------------------------------------}
TYPE
   TApplication = OBJECT (TProgram)
      CONSTRUCTOR Init;
      DESTRUCTOR Done; Virtual;
      PROCEDURE Tile;
      PROCEDURE Cascade;
      PROCEDURE DosShell;
      PROCEDURE GetTileRect (Var R: TRect); Virtual;
      PROCEDURE HandleEvent (Var Event: TEvent); Virtual;
      procedure WriteShellMsg; virtual;
   END;
   PApplication = ^TApplication;                      { Application ptr }

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                STANDARD MENU AND STATUS LINES ROUTINES                    }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-StdStatusKeys------------------------------------------------------
Returns a pointer to a linked list of commonly used status line keys.
The default status line for TApplication uses StdStatusKeys as its
complete list of status keys.
22Oct99 LdB
---------------------------------------------------------------------}
FUNCTION StdStatusKeys (Next: PStatusItem): PStatusItem;

{-StdFileMenuItems---------------------------------------------------
Returns a pointer to a list of menu items for a standard File menu.
The standard File menu items are New, Open, Save, Save As, Save All,
Change Dir, OS Shell, and Exit.
22Oct99 LdB
---------------------------------------------------------------------}
FUNCTION StdFileMenuItems (Next: PMenuItem): PMenuItem;

{-StdEditMenuItems---------------------------------------------------
Returns a pointer to a list of menu items for a standard Edit menu.
The standard Edit menu items are Undo, Cut, Copy, Paste, and Clear.
22Oct99 LdB
---------------------------------------------------------------------}
FUNCTION StdEditMenuItems (Next: PMenuItem): PMenuItem;

{-StdWindowMenuItems-------------------------------------------------
Returns a pointer to a list of menu items for a standard Window menu.
The standard Window menu items are Tile, Cascade, Close All,
Size/Move, Zoom, Next, Previous, and Close.
22Oct99 LdB
---------------------------------------------------------------------}
FUNCTION StdWindowMenuItems (Next: PMenuItem): PMenuItem;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           OBJECT REGISTER ROUTINES                        }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{-RegisterApp--------------------------------------------------------
Calls RegisterType for each of the object types defined in this unit.
22oct99 LdB
---------------------------------------------------------------------}
PROCEDURE RegisterApp;

{***************************************************************************}
{                           OBJECT REGISTRATION                             }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                      TBackGround STREAM REGISTRATION                      }
{---------------------------------------------------------------------------}
CONST
  RBackGround: TStreamRec = (
     ObjType: idBackground;                           { Register id = 30 }
     VmtLink: TypeOf(TBackGround);
     Load:    @TBackGround.Load;                      { Object load method }
     Store:   @TBackGround.Store                      { Object store method }
  );

{---------------------------------------------------------------------------}
{                       TDeskTop STREAM REGISTRATION                        }
{---------------------------------------------------------------------------}
CONST
  RDeskTop: TStreamRec = (
     ObjType: idDesktop;                              { Register id = 31 }
     VmtLink: TypeOf(TDeskTop);
     Load:    @TDeskTop.Load;                         { Object load method }
     Store:   @TDeskTop.Store                         { Object store method }
  );

{***************************************************************************}
{                        INITIALIZED PUBLIC VARIABLES                       }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                       INITIALIZED PUBLIC VARIABLES                        }
{---------------------------------------------------------------------------}
CONST
   AppPalette: Integer = apColor;                     { Application colour }
   Desktop: PDeskTop = Nil;                           { Desktop object }
   MenuBar: PMenuView = Nil;                          { Application menu }
   StatusLine: PStatusLine = Nil;                     { App status line }
   Application : PApplication = Nil;                  { Application object }

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                IMPLEMENTATION
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

uses    Mouse{,Resource};

resourcestring  sVideoFailed='Video initialization failed.';
                sTypeExitOnReturn='Type EXIT to return...';


{***************************************************************************}
{                        PRIVATE DEFINED CONSTANTS                          }
{***************************************************************************}

{***************************************************************************}
{                      PRIVATE INITIALIZED VARIABLES                        }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                      INITIALIZED PRIVATE VARIABLES                        }
{---------------------------------------------------------------------------}
CONST Pending: TEvent = (What: evNothing);            { Pending event }

{---------------------------------------------------------------------------}
{  Tileable -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB          }
{---------------------------------------------------------------------------}
FUNCTION Tileable (P: PView): Boolean;
BEGIN
   Tileable := (P^.Options AND ofTileable <> 0) AND   { View is tileable }
     (P^.State AND sfVisible <> 0);                   { View is visible }
END;

{---------------------------------------------------------------------------}
{  ISqr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB              }
{---------------------------------------------------------------------------}
FUNCTION ISqr (X: Sw_Integer): Sw_Integer;
VAR I: Sw_Integer;
BEGIN
   I := 0;                                            { Set value to zero }
   Repeat
     Inc(I);                                          { Inc value }
   Until (I * I > X);                                 { Repeat till Sqr > X }
   ISqr := I - 1;                                     { Return result }
END;

{---------------------------------------------------------------------------}
{  MostEqualDivisors -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB }
{---------------------------------------------------------------------------}
PROCEDURE MostEqualDivisors (N: Integer; Var X, Y: Integer; FavorY: Boolean);
VAR I: Integer;
BEGIN
   I := ISqr(N);                                      { Int square of N }
   If ((N MOD I) <> 0) Then                           { Initial guess }
     If ((N MOD (I+1)) = 0) Then Inc(I);              { Add one row/column }
   If (I < (N DIV I)) Then I := N DIV I;              { In first page }
   If FavorY Then Begin                               { Horz preferred }
     X := N DIV I;                                    { Calc x position }
     Y := I;                                          { Set y position  }
   End Else Begin                                     { Vert preferred }
     Y := N DIV I;                                    { Calc y position }
     X := I;                                          { Set x position }
   End;
END;

{***************************************************************************}
{                               OBJECT METHODS                              }
{***************************************************************************}

{--TBackGround--------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TBackGround.Init (Var Bounds: TRect; APattern: Char);
BEGIN
   Inherited Init(Bounds);                            { Call ancestor }
   GrowMode := gfGrowHiX + gfGrowHiY;                 { Set grow modes }
   Pattern := APattern;                               { Hold pattern }
END;

{--TBackGround--------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TBackGround.Load (Var S: TStream);
BEGIN
   Inherited Load(S);                                 { Call ancestor }
   S.Read(Pattern, SizeOf(Pattern));                  { Read pattern data }
END;

{--TBackGround--------------------------------------------------------------}
{  GetPalette -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TBackGround.GetPalette: PPalette;
CONST P: String[Length(CBackGround)] = CbackGround;   { Always normal string }
BEGIN
   GetPalette := PPalette(@P);                        { Return palette }
END;

{--TBackGround--------------------------------------------------------------}
{  DrawBackground -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB    }
{---------------------------------------------------------------------------}
PROCEDURE TBackground.Draw;
VAR B: TDrawBuffer;
BEGIN
   MoveChar(B, Pattern, GetColor($01), Size.X);       { Fill draw buffer }
   WriteLine(0, 0, Size.X, Size.Y, B);                { Draw to area }
END;

{--TBackGround--------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TBackGround.Store (Var S: TStream);
BEGIN
   TView.Store(S);                                    { TView store called }
   S.Write(Pattern, SizeOf(Pattern));                 { Write pattern data }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          TDesktop OBJECT METHODS                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TDesktop-----------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TDesktop.Init (Var Bounds: Objects.TRect);
BEGIN
   Inherited Init(Bounds);                            { Call ancestor }
   GrowMode := gfGrowHiX + gfGrowHiY;                 { Set growmode }
   InitBackground;                                    { Create background }
   If (Background <> Nil) Then Insert(Background);    { Insert background }
END;

{--TDesktop-----------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TDesktop.Load (Var S: TStream);
BEGIN
   Inherited Load(S);                                 { Call ancestor }
   GetSubViewPtr(S, Background);                      { Load background }
   S.Read(TileColumnsFirst, SizeOf(TileColumnsFirst));{ Read data }
END;

{--TDesktop-----------------------------------------------------------------}
{  TileError -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TDeskTop.TileError;
BEGIN                                                 { Abstract method }
END;

{--TDesktop-----------------------------------------------------------------}
{  InitBackGround -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB    }
{---------------------------------------------------------------------------}
PROCEDURE TDesktop.InitBackground;
CONST Ch: Char = #176;
VAR R: TRect;
BEGIN
   GetExtent(R);                                      { Get desktop extents }
   BackGround := New(PBackground, Init(R, Ch));       { Insert a background }
END;

{--TDesktop-----------------------------------------------------------------}
{  Tile -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TDeskTop.Tile (Var R: TRect);
VAR NumCols, NumRows, NumTileable, LeftOver, TileNum: Integer;

   FUNCTION DividerLoc (Lo, Hi, Num, Pos: Integer): Integer;
   BEGIN
     DividerLoc := LongInt( LongInt(Hi - Lo) * Pos)
       DIV Num + Lo;                                  { Calc position }
   END;

   PROCEDURE DoCountTileable (P: PView); {$IFNDEF PPC_FPC}FAR;{$ENDIF}
   BEGIN
     If Tileable(P) Then Inc(NumTileable);            { Count tileable views }
   END;

   PROCEDURE CalcTileRect (Pos: Integer; Var NR: TRect);
   VAR X, Y, D: Integer;
   BEGIN
     D := (NumCols - LeftOver) * NumRows;             { Calc d value }
     If (Pos<D) Then Begin
       X := Pos DIV NumRows; Y := Pos MOD NumRows;    { Calc positions }
     End Else Begin
       X := (Pos - D) div (NumRows + 1) +
         (NumCols - LeftOver);                        { Calc x position }
       Y := (Pos - D) mod (NumRows + 1);              { Calc y position }
     End;
     NR.A.X := DividerLoc(R.A.X, R.B.X, NumCols, X);  { Top left x position }
     NR.B.X := DividerLoc(R.A.X, R.B.X, NumCols, X+1);{ Right x position }
     If (Pos >= D) Then Begin
       NR.A.Y := DividerLoc(R.A.Y, R.B.Y,NumRows+1,Y);{ Top y position }
       NR.B.Y := DividerLoc(R.A.Y, R.B.Y, NumRows+1,
        Y+1);                                         { Bottom y position }
     End Else Begin
       NR.A.Y := DividerLoc(R.A.Y, R.B.Y,NumRows,Y);  { Top y position }
       NR.B.Y := DividerLoc(R.A.Y, R.B.Y, NumRows,
        Y+1);                                         { Bottom y position }
     End;
   END;

   PROCEDURE DoTile(P: PView); {$IFNDEF PPC_FPC}FAR;{$ENDIF}
   VAR PState: Word; R: TRect;
   BEGIN
     If Tileable(P) Then Begin
       CalcTileRect(TileNum, R);                      { Calc tileable area }
       PState := P^.State;                            { Hold view state }
       P^.State := P^.State AND NOT sfVisible;        { Temp not visible }
       P^.Locate(R);                                  { Locate view }
       P^.State := PState;                            { Restore view state }
       Dec(TileNum);                                  { One less to tile }
     End;
   END;

BEGIN
   NumTileable := 0;                                  { Zero tileable count }
   ForEach(@DoCountTileable);                         { Count tileable views }
   If (NumTileable>0) Then Begin
     MostEqualDivisors(NumTileable, NumCols, NumRows,
     NOT TileColumnsFirst);                           { Do pre calcs }
     If ((R.B.X - R.A.X) DIV NumCols = 0) OR
     ((R.B.Y - R.A.Y) DIV NumRows = 0) Then TileError { Can't tile }
     Else Begin
       LeftOver := NumTileable MOD NumCols;           { Left over count }
       TileNum := NumTileable-1;                      { Tileable views }
       ForEach(@DoTile);                              { Tile each view }
       DrawView;                                      { Now redraw }
     End;
   End;
END;

{--TDesktop-----------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TDesktop.Store (Var S: TStream);
BEGIN
   TGroup.Store(S);                                   { Call group store }
   PutSubViewPtr(S, Background);                      { Store background }
   S.Write(TileColumnsFirst,SizeOf(TileColumnsFirst));{ Write data }
END;

{--TDesktop-----------------------------------------------------------------}
{  Cascade -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TDeskTop.Cascade (Var R: TRect);
VAR CascadeNum: Integer; LastView: PView; Min, Max: TPoint;

   PROCEDURE DoCount (P: PView); {$IFNDEF PPC_FPC}FAR;{$ENDIF}
   BEGIN
     If Tileable(P) Then Begin
       Inc(CascadeNum); LastView := P;                { Count cascadable }
     End;
   END;

   PROCEDURE DoCascade (P: PView); {$IFNDEF PPC_FPC}FAR;{$ENDIF}
   VAR PState: Word; NR: TRect;
   BEGIN
     If Tileable(P) AND (CascadeNum >= 0) Then Begin  { View cascadable }
       NR.Copy(R);                                    { Copy rect area }
       Inc(NR.A.X, CascadeNum);                       { Inc x position }
       Inc(NR.A.Y, CascadeNum);                       { Inc y position }
       PState := P^.State;                            { Hold view state }
       P^.State := P^.State AND NOT sfVisible;        { Temp stop draw }
       P^.Locate(NR);                                 { Locate the view }
       P^.State := PState;                            { Now allow draws }
       Dec(CascadeNum);                               { Dec count }
     End;
   END;

BEGIN
   CascadeNum := 0;                                   { Zero cascade count }
   ForEach(@DoCount);                                 { Count cascadable }
   If (CascadeNum>0) Then Begin
     LastView^.SizeLimits(Min, Max);                  { Check size limits }
     If (Min.X > R.B.X - R.A.X - CascadeNum) OR
     (Min.Y > R.B.Y - R.A.Y - CascadeNum) Then
     TileError Else Begin                             { Check for error }
       Dec(CascadeNum);                               { One less view }
       ForEach(@DoCascade);                           { Cascade view }
       DrawView;                                      { Redraw now }
     End;
   End;
END;

{--TDesktop-----------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 11May98 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TDesktop.HandleEvent (Var Event: TEvent);
BEGIN
   Inherited HandleEvent(Event);                      { Call ancestor }
   If (Event.What = evCommand) Then Begin
     Case Event.Command of                            { Command event }
       cmNext: FocusNext(False);                      { Focus next view }
       cmPrev: If (BackGround <> Nil) Then Begin
         If Valid(cmReleasedFocus) Then
          Current^.PutInFrontOf(Background);          { Focus last view }
       End Else FocusNext(True);                      { Focus prior view }
       Else Exit;
     End;
     ClearEvent(Event);                               { Clear the event }
   End;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          TProgram OBJECT METHODS                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}


{--TProgram-----------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TProgram.Init;
VAR R: TRect;
BEGIN
   R.Assign(0, 0, ScreenWidth, ScreenHeight);         { Full screen area }
   Inherited Init(R);                                 { Call ancestor }
   Application := PApplication(@Self);                { Set application ptr }
   InitScreen;                                        { Initialize screen }
   State := sfVisible + sfSelected + sfFocused +
      sfModal + sfExposed;                            { Deafult states }
   Options := 0;                                      { No options set }
   Size.X := ScreenWidth;                             { Set x size value }
   Size.Y := ScreenHeight;                            { Set y size value }
   InitStatusLine;                                    { Create status line }
   InitMenuBar;                                       { Create a bar menu }
   InitDesktop;                                       { Create desktop }
   If (Desktop <> Nil) Then Insert(Desktop);          { Insert desktop }
   If (StatusLine <> Nil) Then Insert(StatusLine);    { Insert status line }
   If (MenuBar <> Nil) Then Insert(MenuBar);          { Insert menu bar }
END;

{--TProgram-----------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB              }
{---------------------------------------------------------------------------}
DESTRUCTOR TProgram.Done;
BEGIN
   { Do not free the Buffer of Video Unit }
   If Buffer = Views.PVideoBuf(VideoBuf) then
     Buffer:=nil;
   If (Desktop <> Nil) Then Dispose(Desktop, Done);   { Destroy desktop }
   If (MenuBar <> Nil) Then Dispose(MenuBar, Done);   { Destroy menu bar }
   If (StatusLine <> Nil) Then
     Dispose(StatusLine, Done);                       { Destroy status line }
   Application := Nil;                                { Clear application }
   Inherited Done;                                    { Call ancestor }
END;

{--TProgram-----------------------------------------------------------------}
{  GetPalette -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
FUNCTION TProgram.GetPalette: PPalette;
CONST P: Array[apColor..apMonochrome] Of String = (CAppColor, CAppBlackWhite,
  CAppMonochrome);
BEGIN
   GetPalette := @P[AppPalette];                      { Return palette }
END;

{--TProgram-----------------------------------------------------------------}
{  CanMoveFocus -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 23Sep97 LdB      }
{---------------------------------------------------------------------------}
FUNCTION TProgram.CanMoveFocus: Boolean;
BEGIN
   If (Desktop <> Nil) Then                           { Valid desktop view }
     CanMovefocus := DeskTop^.Valid(cmReleasedFocus)  { Check focus move }
     Else CanMoveFocus := True;                       { No desktop who cares! }
END;

{--TProgram-----------------------------------------------------------------}
{  ValidView -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB         }
{---------------------------------------------------------------------------}
FUNCTION TProgram.ValidView (P: PView): PView;
BEGIN
   ValidView := Nil;                                  { Preset failure }
   If (P <> Nil) Then Begin
(*
     If LowMemory Then Begin                          { Check memroy }
       Dispose(P, Done);                              { Dispose view }
       OutOfMemory;                                   { Call out of memory }
       Exit;                                          { Now exit }
     End;
*)
     If NOT P^.Valid(cmValid) Then Begin              { Check view valid }
       Dispose(P, Done);                              { Dipose view }
       Exit;                                          { Now exit }
     End;
     ValidView := P;                                  { Return view }
   End;
END;

{--TProgram-----------------------------------------------------------------}
{  InsertWindow -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB      }
{---------------------------------------------------------------------------}
FUNCTION TProgram.InsertWindow (P: PWindow): PWindow;
BEGIN
   InsertWindow := Nil;                               { Preset failure }
   If (ValidView(P) <> Nil) Then                      { Check view valid }
     If (CanMoveFocus) AND (Desktop <> Nil)           { Can we move focus }
     Then Begin
       Desktop^.Insert(P);                            { Insert window }
       InsertWindow := P;                             { Return view ptr }
     End Else Dispose(P, Done);                       { Dispose view }
END;

{--TProgram-----------------------------------------------------------------}
{  ExecuteDialog -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB     }
{---------------------------------------------------------------------------}
FUNCTION TProgram.ExecuteDialog (P: PDialog; Data: Pointer): Word;
VAR ExecResult: Word;
BEGIN
   ExecuteDialog := cmCancel;                         { Preset cancel }
   If (ValidView(P) <> Nil) Then Begin                { Check view valid }
     If (Data <> Nil) Then P^.SetData(Data^);         { Set data }
     If (P <> Nil) Then P^.SelectDefaultView;         { Select default }
     ExecResult := Desktop^.ExecView(P);              { Execute view }
     If (ExecResult <> cmCancel) AND (Data <> Nil)
       Then P^.GetData(Data^);                        { Get data back }
     Dispose(P, Done);                                { Dispose of dialog }
     ExecuteDialog := ExecResult;                     { Return result }
   End;
END;

{--TProgram-----------------------------------------------------------------}
{  Run -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.Run;
BEGIN
   Execute;                                           { Call execute }
END;

{--TProgram-----------------------------------------------------------------}
{  Idle -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25Oct99 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.Idle;
BEGIN
   If (StatusLine <> Nil) Then StatusLine^.Update;    { Update statusline }
   If CommandSetChanged Then Begin                    { Check command change }
     Message(@Self, evBroadcast, cmCommandSetChanged,
       Nil);                                          { Send message }
     CommandSetChanged := False;                      { Clear flag }
   End;
  GiveUpTimeSlice;
END;

{--TProgram-----------------------------------------------------------------}
{  InitScreen -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.InitScreen;

{Initscreen is passive only, i.e. it detects the video size and capabilities
 after initalization. Active video initalization is the task of Tapplication.}

BEGIN
  { the orginal code can't be used here because of the limited
    video unit capabilities, the mono modus can't be handled
  }
<<<<<<< HEAD
<<<<<<< HEAD
  Drivers.DetectVideo;
=======
{  Drivers.InitVideo;}
>>>>>>> graemeg/fixes_2_2
=======
{  Drivers.InitVideo;}
>>>>>>> origin/fixes_2_2
  if (ScreenMode.Col div ScreenMode.Row<2) then
    ShadowSize.X := 1
  else
    ShadowSize.X := 2;

  ShadowSize.Y := 1;
  ShowMarkers := False;
  if ScreenMode.color then
    AppPalette := apColor
  else
    AppPalette := apBlackWhite;
  Buffer := Views.PVideoBuf(VideoBuf);
END;


{procedure TProgram.DoneScreen;
begin
  Drivers.DoneVideo;
  Buffer:=nil;
end;}


{--TProgram-----------------------------------------------------------------}
{  InitDeskTop -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.InitDesktop;
VAR R: TRect;
BEGIN
   GetExtent(R);                                      { Get view extent }
   If (MenuBar <> Nil) Then Inc(R.A.Y);               { Adjust top down }
   If (StatusLine <> Nil) Then Dec(R.B.Y);            { Adjust bottom up }
   DeskTop := New(PDesktop, Init(R));                 { Create desktop }
END;

{--TProgram-----------------------------------------------------------------}
{  OutOfMemory -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 08May98 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.OutOfMemory;
BEGIN                                                 { Abstract method }
END;

{--TProgram-----------------------------------------------------------------}
{  InitMenuBar -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.InitMenuBar;
VAR R: TRect;
BEGIN
   GetExtent(R);                                      { Get view extents }
   R.B.Y := R.A.Y + 1;                                { One line high  }
   MenuBar := New(PMenuBar, Init(R, Nil));            { Create menu bar }
END;

{--TProgram-----------------------------------------------------------------}
{  InitStatusLine -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB    }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.InitStatusLine;
VAR R: TRect;
BEGIN
   GetExtent(R);                                      { Get view extents }
   R.A.Y := R.B.Y - 1;                                { One line high }
   New(StatusLine, Init(R,
     NewStatusDef(0, $FFFF,
       NewStatusKey('~Alt-X~ Exit', kbAltX, cmQuit,
       StdStatusKeys(Nil)), Nil)));                   { Default status line }
END;

{--TProgram-----------------------------------------------------------------}
{  SetScreenMode -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25Oct99 LdB     }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.SetScreenMode (Mode: Word);
var
  R: TRect;
begin
  HideMouse;
{  DoneMemory;}
{  InitMemory;}
  InitScreen;
  Buffer := Views.PVideoBuf(VideoBuf);
  R.Assign(0, 0, ScreenWidth, ScreenHeight);
  ChangeBounds(R);
  ShowMouse;
end;

procedure TProgram.SetScreenVideoMode(const Mode: TVideoMode);
var
  R: TRect;
begin
  hidemouse;
{  DoneMouse;
  DoneMemory;}
  ScreenMode:=Mode;
{  InitMouse;
  InitMemory;}
<<<<<<< HEAD
<<<<<<< HEAD
{  InitScreen;
   Warning: InitScreen calls DetectVideo which
    resets ScreenMode to old value, call it after
    video mode was changed instead of before }
  Video.SetVideoMode(Mode);

  { Update ScreenMode to new value }
  InitScreen;
=======
  InitScreen;
  Video.SetVideoMode(Mode);
>>>>>>> graemeg/fixes_2_2
=======
  InitScreen;
  Video.SetVideoMode(Mode);
>>>>>>> origin/fixes_2_2
  ScreenWidth:=Video.ScreenWidth;
  ScreenHeight:=Video.ScreenHeight;
  Buffer := Views.PVideoBuf(VideoBuf);
  R.Assign(0, 0, ScreenWidth, ScreenHeight);
  ChangeBounds(R);
  ShowMouse;
end;

{--TProgram-----------------------------------------------------------------}
{  PutEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.PutEvent (Var Event: TEvent);
BEGIN
   Pending := Event;                                  { Set pending event }
END;

{--TProgram-----------------------------------------------------------------}
{  GetEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 10May98 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.GetEvent (Var Event: TEvent);
BEGIN
   Event.What := evNothing;
   If (Event.What = evNothing) Then Begin
     If (Pending.What <> evNothing) Then Begin        { Pending event }
       Event := Pending;                              { Load pending event }
       Pending.What := evNothing;                     { Clear pending event }
     End Else Begin
       NextQueuedEvent(Event);                        { Next queued event }
       If (Event.What = evNothing) Then Begin
         GetKeyEvent(Event);                          { Fetch key event }
         If (Event.What = evKeyDown) then
           Begin
             if Event.keyCode = kbAltF12 then
               ReDraw;
           End;
         If (Event.What = evNothing) Then Begin       { No mouse event }
           Drivers.GetMouseEvent(Event);              { Load mouse event }
           If (Event.What = evNothing) Then
             begin
<<<<<<< HEAD
<<<<<<< HEAD
{$IFNDEF HASAMIGA}
               { due to isses with the event handling in FV itself,
                 we skip this here, and let the IDE to handle it
                 directly on Amiga-like systems. The FV itself cannot
                 handle the System Events anyway. (KB) }
               Drivers.GetSystemEvent(Event);         { Load system event }
               If (Event.What = evNothing) Then
{$ENDIF}
=======
               Drivers.GetSystemEvent(Event);         { Load system event }
               If (Event.What = evNothing) Then
>>>>>>> graemeg/fixes_2_2
=======
               Drivers.GetSystemEvent(Event);         { Load system event }
               If (Event.What = evNothing) Then
>>>>>>> origin/fixes_2_2
                 Idle;     { Idle if no event }
             end;
         End;
       End;
     End;
   End;
END;

{--TProgram-----------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 12Sep97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TProgram.HandleEvent (Var Event: TEvent);
VAR C: Char;
BEGIN
   If (Event.What = evKeyDown) Then Begin             { Key press event }
     C := GetAltChar(Event.KeyCode);                  { Get alt char code }
     If (C >= '1') AND (C <= '9') Then
       If (Message(Desktop, evBroadCast, cmSelectWindowNum,
         Pointer(Byte(C) - $30)) <> Nil)              { Select window }
         Then ClearEvent(Event);                      { Clear event }
   End;
   Inherited HandleEvent(Event);                      { Call ancestor }
   If (Event.What = evCommand) AND                    { Command event }
   (Event.Command = cmQuit) Then Begin                { Quit command }
      EndModal(cmQuit);                               { Endmodal operation }
      ClearEvent(Event);                              { Clear event }
   End;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        TApplication OBJECT METHODS                        }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TApplication-------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB              }
{---------------------------------------------------------------------------}
CONSTRUCTOR TApplication.Init;

BEGIN
{   InitMemory;}                                              { Start memory up }
{   if not(InitResource) then
     begin
       writeln('Fatal: Can''t init resources');
       halt(1);
     end;}
   initkeyboard;
   if not Drivers.InitVideo then                              { Start video up }
     begin
       donekeyboard;
       writeln(sVideoFailed);
       halt(1);
     end;
   Drivers.InitEvents;                                        { Start event drive }
   Drivers.InitSysError;                                      { Start system error }
   InitHistory;                                               { Start history up }
   Inherited Init;                                            { Call ancestor }
   InitMsgBox;
   { init mouse and cursor }
   Video.SetCursorType(crHidden);
   Mouse.SetMouseXY(1,1);
END;

{--TApplication-------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB              }
{---------------------------------------------------------------------------}
DESTRUCTOR TApplication.Done;
BEGIN
   Inherited Done;                                    { Call ancestor }
   DoneHistory;                                       { Close history }
   Drivers.DoneSysError;                                      { Close system error }
   Drivers.DoneEvents;                                        { Close event drive }
   drivers.donevideo;
{   DoneMemory;}                                       { Close memory }
   donekeyboard;
{   DoneResource;}
END;

{--TApplication-------------------------------------------------------------}
{  Tile -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TApplication.Tile;
VAR R: TRect;
BEGIN
   GetTileRect(R);                                    { Tileable area }
   If (Desktop <> Nil) Then Desktop^.Tile(R);         { Tile desktop }
END;

{--TApplication-------------------------------------------------------------}
{  Cascade -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE TApplication.Cascade;
VAR R: TRect;
BEGIN
   GetTileRect(R);                                    { Cascade area }
   If (Desktop <> Nil) Then Desktop^.Cascade(R);      { Cascade desktop }
END;

{--TApplication-------------------------------------------------------------}
{  DosShell -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25Oct99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE TApplication.DosShell;

{$ifdef unix}
var s:string;
{$endif}

BEGIN                                                 { Compatability only }
  DoneSysError;
  DoneEvents;
  drivers.donevideo;
  drivers.donekeyboard;
{  DoneDosMem;}
  WriteShellMsg;
{$ifdef Unix}
  s:=getenv('SHELL');
  if s='' then
    s:='/bin/sh';
  exec(s,'');
{$else}
  SwapVectors;
  Exec(GetEnv('COMSPEC'), '');
  SwapVectors;
{$endif}
{  InitDosMem;}
  drivers.initkeyboard;
  drivers.initvideo;
  Video.SetCursorType(crHidden);
  InitScreen;
  InitEvents;
  InitSysError;
  Redraw;
END;

{--TApplication-------------------------------------------------------------}
{  GetTileRect -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TApplication.GetTileRect (Var R: TRect);
BEGIN
   If (DeskTop <> Nil) Then Desktop^.GetExtent(R)     { Desktop extents }
     Else GetExtent(R);                               { Our extents }
END;

{--TApplication-------------------------------------------------------------}
{  HandleEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE TApplication.HandleEvent (Var Event: TEvent);
BEGIN
   Inherited HandleEvent(Event);                      { Call ancestor }
   If (Event.What = evCommand) Then Begin
     Case Event.Command Of
       cmTile: Tile;                                  { Tile request }
       cmCascade: Cascade;                            { Cascade request }
       cmDosShell: DosShell;                          { DOS shell request }
       Else Exit;                                     { Unhandled exit }
     End;
     ClearEvent(Event);                               { Clear the event }
   End;
END;

procedure TApplication.WriteShellMsg;

begin
  writeln(sTypeExitOnReturn);
end;


{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                STANDARD MENU AND STATUS LINES ROUTINES                    }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  StdStatusKeys -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB     }
{---------------------------------------------------------------------------}
FUNCTION StdStatusKeys (Next: PStatusItem): PStatusItem;
BEGIN
   StdStatusKeys :=
     NewStatusKey('', kbAltX, cmQuit,
     NewStatusKey('', kbF10, cmMenu,
     NewStatusKey('', kbAltF3, cmClose,
     NewStatusKey('', kbF5, cmZoom,
     NewStatusKey('', kbCtrlF5, cmResize,
     NewStatusKey('', kbF6, cmNext,
     NewStatusKey('', kbShiftF6, cmPrev,
     Next)))))));
END;

{---------------------------------------------------------------------------}
{  StdFileMenuItems -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB  }
{---------------------------------------------------------------------------}
FUNCTION StdFileMenuItems (Next: PMenuItem): PMenuItem;
BEGIN
   StdFileMenuItems :=
     NewItem('~N~ew', '', kbNoKey, cmNew, hcNew,
     NewItem('~O~pen...', 'F3', kbF3, cmOpen, hcOpen,
     NewItem('~S~ave', 'F2', kbF2, cmSave, hcSave,
     NewItem('S~a~ve as...', '', kbNoKey, cmSaveAs, hcSaveAs,
     NewItem('Save a~l~l', '', kbNoKey, cmSaveAll, hcSaveAll,
     NewLine(
     NewItem('~C~hange dir...', '', kbNoKey, cmChangeDir, hcChangeDir,
     NewItem('OS shell', '', kbNoKey, cmDosShell, hcDosShell,
     NewItem('E~x~it', 'Alt+X', kbAltX, cmQuit, hcExit,
     Next)))))))));
END;

{---------------------------------------------------------------------------}
{  StdEditMenuItems -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB  }
{---------------------------------------------------------------------------}
FUNCTION StdEditMenuItems (Next: PMenuItem): PMenuItem;
BEGIN
   StdEditMenuItems :=
     NewItem('~U~ndo', '', kbAltBack, cmUndo, hcUndo,
     NewLine(
     NewItem('Cu~t~', 'Shift+Del', kbShiftDel, cmCut, hcCut,
     NewItem('~C~opy', 'Ctrl+Ins', kbCtrlIns, cmCopy, hcCopy,
     NewItem('~P~aste', 'Shift+Ins', kbShiftIns, cmPaste, hcPaste,
     NewItem('C~l~ear', 'Ctrl+Del', kbCtrlDel, cmClear, hcClear,
     Next))))));
END;

{---------------------------------------------------------------------------}
{ StdWindowMenuItems -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB }
{---------------------------------------------------------------------------}
FUNCTION StdWindowMenuItems (Next: PMenuItem): PMenuItem;
BEGIN
   StdWindowMenuItems :=
     NewItem('~T~ile', '', kbNoKey, cmTile, hcTile,
     NewItem('C~a~scade', '', kbNoKey, cmCascade, hcCascade,
     NewItem('Cl~o~se all', '', kbNoKey, cmCloseAll, hcCloseAll,
     NewLine(
     NewItem('~S~ize/Move','Ctrl+F5', kbCtrlF5, cmResize, hcResize,
     NewItem('~Z~oom', 'F5', kbF5, cmZoom, hcZoom,
     NewItem('~N~ext', 'F6', kbF6, cmNext, hcNext,
     NewItem('~P~revious', 'Shift+F6', kbShiftF6, cmPrev, hcPrev,
     NewItem('~C~lose', 'Alt+F3', kbAltF3, cmClose, hcClose,
     Next)))))))));
END;


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           OBJECT REGISTER ROUTINES                        }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  RegisterApp -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 22Oct99 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE RegisterApp;
BEGIN
   RegisterType(RBackground);                         { Register background }
   RegisterType(RDesktop);                            { Register desktop }
END;

END.
