{********[ SOURCE FILE OF GRAPHICAL FREE VISION ]**********}
{                                                          }
{    System independent clone of DRIVERS.PAS               }
{                                                          }
{    Interface Copyright (c) 1992 Borland International    }
{                                                          }
{    Copyright (c) 1996, 1997, 1998, 1999, 2000            }
{    by Leon de Boer                                       }
{    ldeboer@attglobal.net  - primary e-mail addr          }
{    ldeboer@projectent.com.au - backup e-mail addr        }
{                                                          }
{    Original FormatStr kindly donated by Marco Schmidt    }
{                                                          }
{    Mouse callback hook under FPC with kind assistance of }
{    Pierre Muller, Gertjan Schouten & Florian Klaempfl.   }
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

UNIT Drivers;

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                  INTERFACE
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

{====Include file to sort compiler platform out =====================}
{$I platform.inc}
{====================================================================}

{==== Compiler directives ===========================================}

{$X+} { Extended syntax is ok }
{$R-} { Disable range checking }
{$IFNDEF OS_UNIX}
{$S-} { Disable Stack Checking }
{$ENDIF}
{$I-} { Disable IO Checking }
{$Q-} { Disable Overflow Checking }
{$V-} { Turn off strict VAR strings }
{====================================================================}

{$ifdef CPU68K}
  {$DEFINE ENDIAN_BIG}
{$endif CPU68K}

{$ifdef FPC}
  {$INLINE ON}
{$endif}

USES
   {$IFDEF OS_WINDOWS}                                { WIN/NT CODE }
         Windows,                                     { Standard unit }
   {$ENDIF}

   {$ifdef OS_DOS}
     Dos,
   {$endif OS_DOS}

   {$IFDEF OS_OS2}                                    { OS2 CODE }
     {$IFDEF PPC_Virtual}                             { VIRTUAL PASCAL UNITS }
       OS2Def, OS2Base, OS2PMAPI,                     { Standard units }
     {$ENDIF}
     {$IFDEF PPC_Speed}                               { SPEED PASCAL UNITS }
       BseDos, Os2Def,                                { Standard units }
     {$ENDIF}
     {$IFDEF PPC_FPC}                                 { FPC UNITS }
       DosCalls, Os2Def,                              { Standard units }
     {$ENDIF}
   {$ENDIF}

   {$IFDEF OS_UNIX}
<<<<<<< HEAD
<<<<<<< HEAD
       unixtype,baseunix,unix,
=======
       baseunix,unix,
>>>>>>> graemeg/fixes_2_2
=======
       baseunix,unix,
>>>>>>> origin/fixes_2_2
   {$ENDIF}

   {$IFDEF OS_NETWARE_LIBC}
      libc,
   {$ENDIF}
   {$IFDEF OS_NETWARE_CLIB}
      nwserv,
   {$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
   {$IFDEF OS_AMIGA}
      dos, amigados,
   {$ENDIF}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   video,
   SysMsg,
   FVCommon, Objects;                                 { GFV standard units }

{***************************************************************************}
{                              PUBLIC CONSTANTS                             }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                              EVENT TYPE MASKS                             }
{---------------------------------------------------------------------------}
CONST
   evMouseDown = $0001;                               { Mouse down event }
   evMouseUp   = $0002;                               { Mouse up event }
   evMouseMove = $0004;                               { Mouse move event }
   evMouseAuto = $0008;                               { Mouse auto event }
   evKeyDown   = $0010;                               { Key down event }
   evCommand   = $0100;                               { Command event }
   evBroadcast = $0200;                               { Broadcast event }

{---------------------------------------------------------------------------}
{                             EVENT CODE MASKS                              }
{---------------------------------------------------------------------------}
CONST
   evNothing   = $0000;                               { Empty event }
   evMouse     = $000F;                               { Mouse event }
   evKeyboard  = $0010;                               { Keyboard event }
   evMessage   = $FF00;                               { Message event }

{---------------------------------------------------------------------------}
{                             EXTENDED KEY CODES                            }
{---------------------------------------------------------------------------}
CONST
   kbNoKey       = $0000;  kbAltEsc      = $0100;  kbEsc         = $011B;
   kbAltSpace    = $0200;  kbCtrlIns     = $0400;  kbShiftIns    = $0500;
   kbCtrlDel     = $0600;  kbShiftDel    = $0700;  kbAltBack     = $0800;
   kbAltShiftBack= $0900;  kbBack        = $0E08;  kbCtrlBack    = $0E7F;
   kbShiftTab    = $0F00;  kbTab         = $0F09;  kbAltQ        = $1000;
   kbCtrlQ       = $1011;  kbAltW        = $1100;  kbCtrlW       = $1117;
   kbAltE        = $1200;  kbCtrlE       = $1205;  kbAltR        = $1300;
   kbCtrlR       = $1312;  kbAltT        = $1400;  kbCtrlT       = $1414;
   kbAltY        = $1500;  kbCtrlY       = $1519;  kbAltU        = $1600;
   kbCtrlU       = $1615;  kbAltI        = $1700;  kbCtrlI       = $1709;
   kbAltO        = $1800;  kbCtrlO       = $180F;  kbAltP        = $1900;
   kbCtrlP       = $1910;  kbAltLftBrack = $1A00;  kbAltRgtBrack = $1B00;
   kbCtrlEnter   = $1C0A;  kbEnter       = $1C0D;  kbAltA        = $1E00;
   kbCtrlA       = $1E01;  kbAltS        = $1F00;  kbCtrlS       = $1F13;
   kbAltD        = $2000;  kbCtrlD       = $2004;  kbAltF        = $2100;
   kbCtrlF       = $2106;  kbAltG        = $2200;  kbCtrlG       = $2207;
   kbAltH        = $2300;  kbCtrlH       = $2308;  kbAltJ        = $2400;
   kbCtrlJ       = $240A;  kbAltK        = $2500;  kbCtrlK       = $250B;
   kbAltL        = $2600;  kbCtrlL       = $260C;  kbAltSemiCol  = $2700;
   kbAltQuote    = $2800;  kbAltOpQuote  = $2900;  kbAltBkSlash  = $2B00;
   kbAltZ        = $2C00;  kbCtrlZ       = $2C1A;  kbAltX        = $2D00;
   kbCtrlX       = $2D18;  kbAltC        = $2E00;  kbCtrlC       = $2E03;
   kbAltV        = $2F00;  kbCtrlV       = $2F16;  kbAltB        = $3000;
   kbCtrlB       = $3002;  kbAltN        = $3100;  kbCtrlN       = $310E;
   kbAltM        = $3200;  kbCtrlM       = $320D;  kbAltComma    = $3300;
   kbAltPeriod   = $3400;  kbAltSlash    = $3500;  kbAltGreyAst  = $3700;
   kbSpaceBar    = $3920;  kbF1          = $3B00;  kbF2          = $3C00;
   kbF3          = $3D00;  kbF4          = $3E00;  kbF5          = $3F00;
   kbF6          = $4000;  kbF7          = $4100;  kbF8          = $4200;
   kbF9          = $4300;  kbF10         = $4400;  kbHome        = $4700;
   kbUp          = $4800;  kbPgUp        = $4900;  kbGrayMinus   = $4A2D;
   kbLeft        = $4B00;  kbCenter      = $4C00;  kbRight       = $4D00;
   kbAltGrayPlus = $4E00;  kbGrayPlus    = $4E2B;  kbEnd         = $4F00;
   kbDown        = $5000;  kbPgDn        = $5100;  kbIns         = $5200;
   kbDel         = $5300;  kbShiftF1     = $5400;  kbShiftF2     = $5500;
   kbShiftF3     = $5600;  kbShiftF4     = $5700;  kbShiftF5     = $5800;
   kbShiftF6     = $5900;  kbShiftF7     = $5A00;  kbShiftF8     = $5B00;
   kbShiftF9     = $5C00;  kbShiftF10    = $5D00;  kbCtrlF1      = $5E00;
   kbCtrlF2      = $5F00;  kbCtrlF3      = $6000;  kbCtrlF4      = $6100;
   kbCtrlF5      = $6200;  kbCtrlF6      = $6300;  kbCtrlF7      = $6400;
   kbCtrlF8      = $6500;  kbCtrlF9      = $6600;  kbCtrlF10     = $6700;
   kbAltF1       = $6800;  kbAltF2       = $6900;  kbAltF3       = $6A00;
   kbAltF4       = $6B00;  kbAltF5       = $6C00;  kbAltF6       = $6D00;
   kbAltF7       = $6E00;  kbAltF8       = $6F00;  kbAltF9       = $7000;
   kbAltF10      = $7100;  kbCtrlPrtSc   = $7200;  kbCtrlLeft    = $7300;
   kbCtrlRight   = $7400;  kbCtrlEnd     = $7500;  kbCtrlPgDn    = $7600;
   kbCtrlHome    = $7700;  kbAlt1        = $7800;  kbAlt2        = $7900;
   kbAlt3        = $7A00;  kbAlt4        = $7B00;  kbAlt5        = $7C00;
   kbAlt6        = $7D00;  kbAlt7        = $7E00;  kbAlt8        = $7F00;
   kbAlt9        = $8000;  kbAlt0        = $8100;  kbAltMinus    = $8200;
   kbAltEqual    = $8300;  kbCtrlPgUp    = $8400;  kbF11         = $8500;
   kbF12         = $8600;  kbShiftF11    = $8700;  kbShiftF12    = $8800;
   kbCtrlF11     = $8900;  kbCtrlF12     = $8A00;  kbAltF11      = $8B00;
   kbAltF12      = $8C00;  kbCtrlUp      = $8D00;  kbCtrlMinus   = $8E00;
   kbCtrlCenter  = $8F00;  kbCtrlGreyPlus= $9000;  kbCtrlDown    = $9100;
   kbCtrlTab     = $9400;  kbAltHome     = $9700;  kbAltUp       = $9800;
   kbAltPgUp     = $9900;  kbAltLeft     = $9B00;  kbAltRight    = $9D00;
   kbAltEnd      = $9F00;  kbAltDown     = $A000;  kbAltPgDn     = $A100;
   kbAltIns      = $A200;  kbAltDel      = $A300;  kbAltTab      = $A500;

{ ------------------------------- REMARK ------------------------------ }
{ New keys not initially defined by Borland in their unit interface.    }
{ ------------------------------ END REMARK --- Leon de Boer, 15May96 - }
   kbFullStop    = $342E;  kbComma       = $332C;  kbBackSlash   = $352F;
   kbApostrophe  = $2827;  kbSemiColon   = $273B;  kbEqual       = $0D3D;
   kbGreaterThan = $343E;  kbLessThan    = $333C;  kbQuestion    = $353F;
   kbQuote       = $2822;  kbColon       = $273A;  kbPlus        = $0D2B;
   kbPipe        = $2B7C;  kbSlash       = $2B5C;  kbExclaim     = $0221;
   kbAt          = $0340;  kbNumber      = $0423;  kbPercent     = $0625;
   kbCaret       = $075E;  kbAmpersand   = $0826;  kbAsterix     = $092A;
   kbLeftBracket = $0A28;  kbRightBracket= $0B29;  kbApprox      = $2960;
   kbTilde       = $297E;  kbDollar      = $0524;  kbMinus       = $0C2D;
   kbUnderline   = $0C5F;  kbLeftSqBr    = $1A5B;  kbRightSqBr   = $1B5D;
   kbLeftCurlyBr = $1A7B;  kbRightCurlyBr= $1B7D;

{---------------------------------------------------------------------------}
{                      KEYBOARD STATE AND SHIFT MASKS                       }
{---------------------------------------------------------------------------}
CONST
   kbRightShift  = $0001;                             { Right shift key }
   kbLeftShift   = $0002;                             { Left shift key }
   kbCtrlShift   = $0004;                             { Control key down }
   kbAltShift    = $0008;                             { Alt key down }
   kbScrollState = $0010;                             { Scroll lock on }
   kbNumState    = $0020;                             { Number lock on }
   kbCapsState   = $0040;                             { Caps lock on }
   kbInsState    = $0080;                             { Insert mode on }

   kbBothShifts  = kbRightShift + kbLeftShift;        { Right & Left shifts }

{---------------------------------------------------------------------------}
{                         MOUSE BUTTON STATE MASKS                          }
{---------------------------------------------------------------------------}
CONST
   mbLeftButton   = $01;                              { Left mouse button }
   mbRightButton  = $02;                              { Right mouse button }
   mbMiddleButton = $04;                              { Middle mouse button }

{---------------------------------------------------------------------------}
{                         SCREEN CRT MODE CONSTANTS                         }
{---------------------------------------------------------------------------}
CONST
   smBW80    = $0002;                                 { Black and white }
   smCO80    = $0003;                                 { Colour mode }
   smMono    = $0007;                                 { Monochrome mode }
   smFont8x8 = $0100;                                 { 8x8 font mode }

{***************************************************************************}
{                          PUBLIC TYPE DEFINITIONS                          }
{***************************************************************************}

{ ******************************* REMARK ****************************** }
{    The TEvent definition is completely compatable with all existing   }
{  code but adds two new fields ID and Data into the message record     }
{  which helps with WIN/NT and OS2 message processing.                  }
{ ****************************** END REMARK *** Leon de Boer, 11Sep97 * }

{---------------------------------------------------------------------------}
{                          EVENT RECORD DEFINITION                          }
{---------------------------------------------------------------------------}
TYPE
   TEvent =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
   PACKED
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
   RECORD
      What: Sw_Word;                                     { Event type }
      Case Sw_Word Of
        evNothing: ();                                { ** NO EVENT ** }
        evMouse: (
          Buttons: Byte;                              { Mouse buttons }
          Double: Boolean;                            { Double click state }
          Where: TPoint);                             { Mouse position }
        evKeyDown: (
        { ** KEY EVENT ** }
          Case Sw_Integer Of
            0: (KeyCode:  Word);                       { Full key code }
            1: (
{$ifdef ENDIAN_BIG}
                ScanCode: Byte;
                CharCode: Char;
{$else not ENDIAN_BIG}
                CharCode: Char;                       { Char code }
                ScanCode: Byte;                       { Scan code }
{$endif not ENDIAN_BIG}
                KeyShift: byte));                     { Shift states }
        evMessage: (                                  { ** MESSAGE EVENT ** }
          Command: Sw_Word;                              { Message command }
          Id     : Sw_Word;                              { Message id }
          Data   : Real;                              { Message data }
          Case Sw_Word Of
            0: (InfoPtr: Pointer);                    { Message pointer }
            1: (InfoLong: Longint);                   { Message longint }
            2: (InfoWord: Word);                      { Message Sw_Word }
            3: (InfoInt: Integer);                    { Message Sw_Integer }
            4: (InfoByte: Byte);                      { Message byte }
            5: (InfoChar: Char));                     { Message character }
   END;
   PEvent = ^TEvent;

   TVideoMode = Video.TVideoMode;                     { Screen mode }

{---------------------------------------------------------------------------}
{                    ERROR HANDLER FUNCTION DEFINITION                      }
{---------------------------------------------------------------------------}
TYPE
   TSysErrorFunc = FUNCTION (ErrorCode: Sw_Integer; Drive: Byte): Sw_Integer;

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{ Get Dos counter ticks }
Function GetDosTicks:longint; { returns ticks at 18.2 Hz, just like DOS }


procedure GiveUpTimeSlice;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          BUFFER MOVE ROUTINES                             }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-CStrLen------------------------------------------------------------
Returns the length of string S, where S is a control string using tilde
characters ('~') to designate shortcut characters. The tildes are
excluded from the length of the string, as they will not appear on
the screen. For example, given the string '~B~roccoli' as its
parameter, CStrLen returns 8.
25May96 LdB
---------------------------------------------------------------------}
FUNCTION CStrLen (Const S: String): Sw_Integer;

{-MoveStr------------------------------------------------------------
Moves a string into a buffer for use with a view's WriteBuf or WriteLine.
Dest must be a TDrawBuffer (or an equivalent array of Sw_Words). The
characters in Str are moved into the low bytes of corresponding Sw_Words
in Dest. The high bytes of the Sw_Words are set to Attr, or remain
unchanged if Attr is zero.
25May96 LdB
---------------------------------------------------------------------}
PROCEDURE MoveStr (Var Dest; Const Str: String; Attr: Byte);

{-MoveCStr-----------------------------------------------------------
The characters in Str are moved into the low bytes of corresponding
Sw_Words in Dest. The high bytes of the Sw_Words are set to Lo(Attr) or
Hi(Attr). Tilde characters (~) in the string toggle between the two
attribute bytes passed in the Attr Sw_Word.
25May96 LdB
---------------------------------------------------------------------}
PROCEDURE MoveCStr (Var Dest; Const Str: String; Attrs: Word);

{-MoveBuf------------------------------------------------------------
Count bytes are moved from Source into the low bytes of corresponding
Sw_Words in Dest. The high bytes of the Sw_Words in Dest are set to Attr,
or remain unchanged if Attr is zero.
25May96 LdB
---------------------------------------------------------------------}
PROCEDURE MoveBuf (Var Dest, Source; Attr: Byte; Count: Sw_Word);

{-MoveChar------------------------------------------------------------
Moves characters into a buffer for use with a view's WriteBuf or
WriteLine. Dest must be a TDrawBuffer (or an equivalent array of Sw_Words).
The low bytes of the first Count Sw_Words of Dest are set to C, or
remain unchanged if Ord(C) is zero. The high bytes of the Sw_Words are
set to Attr, or remain unchanged if Attr is zero.
25May96 LdB
---------------------------------------------------------------------}
PROCEDURE MoveChar (Var Dest; C: Char; Attr: Byte; Count: Sw_Word);

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        KEYBOARD SUPPORT ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-GetAltCode---------------------------------------------------------
Returns the scancode corresponding to Alt+Ch key that is given.
25May96 LdB
---------------------------------------------------------------------}
FUNCTION GetAltCode (Ch: Char): Word;

{-GetCtrlCode--------------------------------------------------------
Returns the scancode corresponding to Alt+Ch key that is given.
25May96 LdB
---------------------------------------------------------------------}
FUNCTION GetCtrlCode (Ch: Char): Word;

{-GetAltChar---------------------------------------------------------
Returns the ascii character for the Alt+Key scancode that was given.
25May96 LdB
---------------------------------------------------------------------}
FUNCTION GetAltChar (KeyCode: Word): Char;

{-GetCtrlChar--------------------------------------------------------
Returns the ascii character for the Ctrl+Key scancode that was given.
25May96 LdB
---------------------------------------------------------------------}
FUNCTION GetCtrlChar (KeyCode: Word): Char;

{-CtrlToArrow--------------------------------------------------------
Converts a WordStar-compatible control key code to the corresponding
cursor key code.
25May96 LdB
---------------------------------------------------------------------}
FUNCTION CtrlToArrow (KeyCode: Word): Word;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        KEYBOARD CONTROL ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-GetShiftState------------------------------------------------------
Returns a byte containing the current Shift key state. The return
value contains a combination of the kbXXXX constants for shift states.
08Jul96 LdB
---------------------------------------------------------------------}
FUNCTION GetShiftState: Byte;

{-GetKeyEvent--------------------------------------------------------
Checks whether a keyboard event is available. If a key has been pressed,
Event.What is set to evKeyDown and Event.KeyCode is set to the scan
code of the key. Otherwise, Event.What is set to evNothing.
19May98 LdB
---------------------------------------------------------------------}
PROCEDURE GetKeyEvent (Var Event: TEvent);

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          MOUSE CONTROL ROUTINES                           }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-ShowMouse----------------------------------------------------------
Decrements the hide counter and if zero the mouse is shown on screen.
30Jun98 LdB
---------------------------------------------------------------------}
PROCEDURE ShowMouse;

{-HideMouse----------------------------------------------------------
If mouse hide counter is zero it removes the cursor from the screen.
The hide counter is then incremented by one count.
30Jun98 LdB
---------------------------------------------------------------------}
PROCEDURE HideMouse;

{-GetMouseEvent------------------------------------------------------
Checks whether a mouse event is available. If a mouse event has occurred,
Event.What is set to evMouseDown, evMouseUp, evMouseMove, or evMouseAuto
and the button and double click variables are set appropriately.
06Jan97 LdB
---------------------------------------------------------------------}
PROCEDURE GetMouseEvent (Var Event: TEvent);

{-GetSystemEvent------------------------------------------------------
Checks whether a system event is available. If a system event has occurred,
Event.What is set to evCommand appropriately
10Oct2000 PM
---------------------------------------------------------------------}
procedure GetSystemEvent (Var Event: TEvent);

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                      EVENT HANDLER CONTROL ROUTINES                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-InitEvents---------------------------------------------------------
Initializes the event manager, enabling the mouse handler routine and
under DOS/DPMI shows the mouse on screen. It is called automatically
by TApplication.Init.
02May98 LdB
---------------------------------------------------------------------}
PROCEDURE InitEvents;

{-DoneEvents---------------------------------------------------------
Terminates event manager and disables the mouse and under DOS hides
the mouse. It is called automatically by TApplication.Done.
02May98 LdB
---------------------------------------------------------------------}
PROCEDURE DoneEvents;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           VIDEO CONTROL ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-Initkeyboard-------------------------------------------------------
Initializes the keyboard. Before it is called read(ln)/write(ln)
are functional, after it is called FV's keyboard routines are
functional.
---------------------------------------------------------------------}

procedure initkeyboard;

{-Donekeyboard-------------------------------------------------------
Restores keyboard to original state. FV's keyboard routines may not
be used after a call to this. Read(ln)/write(ln) can be used again.
---------------------------------------------------------------------}

procedure donekeyboard;

<<<<<<< HEAD
<<<<<<< HEAD
{-DetectVideo---------------------------------------------------------
Detects the current video mode without initializing or otherwise
changing the current screen.
---------------------------------------------------------------------}
procedure DetectVideo;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{-InitVideo---------------------------------------------------------
Initializes the video manager, Saves the current screen mode in
StartupMode, and switches to the mode indicated by ScreenMode.
19May98 LdB
---------------------------------------------------------------------}
function InitVideo:boolean;

{-DoneVideo---------------------------------------------------------
Terminates the video manager by restoring the initial screen mode
(given by StartupMode), clearing the screen, and restoring the cursor.
Called automatically by TApplication.Done.
03Jan97 LdB
---------------------------------------------------------------------}
PROCEDURE DoneVideo;

{-ClearScreen--------------------------------------------------------
Does nothing provided for compatability purposes only.
04Jan97 LdB
---------------------------------------------------------------------}
PROCEDURE ClearScreen;

{-SetVideoMode-------------------------------------------------------
Does nothing provided for compatability purposes only.
04Jan97 LdB
---------------------------------------------------------------------}
PROCEDURE SetVideoMode (Mode: Sw_Word);

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           ERROR CONTROL ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-InitSysError-------------------------------------------------------
Error handling is not yet implemented so this simply sets
SysErrActive=True (ie it lies) and exits.
20May98 LdB
---------------------------------------------------------------------}
PROCEDURE InitSysError;

{-DoneSysError-------------------------------------------------------
Error handling is not yet implemented so this simply sets
SysErrActive=False and exits.
20May98 LdB
---------------------------------------------------------------------}
PROCEDURE DoneSysError;

{-SystemError---------------------------------------------------------
Error handling is not yet implemented so this simply drops through.
20May98 LdB
---------------------------------------------------------------------}
FUNCTION SystemError (ErrorCode: Sw_Integer; Drive: Byte): Sw_Integer;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           STRING FORMAT ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-PrintStr-----------------------------------------------------------
Does nothing provided for compatability purposes only.
30Jun98 LdB
---------------------------------------------------------------------}
PROCEDURE PrintStr (CONST S: String);

{-FormatStr----------------------------------------------------------
A string formatting routine that given a string that includes format
specifiers and a list of parameters in Params, FormatStr produces a
formatted output string in Result.
18Feb99 LdB
---------------------------------------------------------------------}
PROCEDURE FormatStr (Var Result: String; CONST Format: String; Var Params);

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                 >> NEW QUEUED EVENT HANDLER ROUTINES <<                   }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-PutEventInQueue-----------------------------------------------------
If there is room in the queue the event is placed in the next vacant
position in the queue manager.
17Mar98 LdB
---------------------------------------------------------------------}
FUNCTION PutEventInQueue (Var Event: TEvent): Boolean;

{-NextQueuedEvent----------------------------------------------------
If there are queued events the next event is loaded into event else
evNothing is returned.
17Mar98 LdB
---------------------------------------------------------------------}
PROCEDURE NextQueuedEvent(Var Event: TEvent);

{***************************************************************************}
{                        INITIALIZED PUBLIC VARIABLES                       }
{***************************************************************************}

PROCEDURE HideMouseCursor;
PROCEDURE ShowMouseCursor;


{---------------------------------------------------------------------------}
{                INITIALIZED DOS/DPMI/WIN/NT/OS2 VARIABLES                  }
{---------------------------------------------------------------------------}
CONST
   CheckSnow    : Boolean = False;                    { Compatability only }
   MouseEvents  : Boolean = False;                    { Mouse event state }
   MouseReverse : Boolean = False;                    { Mouse reversed }
   HiResScreen  : Boolean = False;                    { Compatability only }
   CtrlBreakHit : Boolean = False;                    { Compatability only }
   SaveCtrlBreak: Boolean = False;                    { Compatability only }
   SysErrActive : Boolean = False;                    { Compatability only }
   FailSysErrors: Boolean = False;                    { Compatability only }
   ButtonCount  : Byte = 0;                           { Mouse button count }
   DoubleDelay  : Sw_Word = 8;                           { Double click delay }
   RepeatDelay  : Sw_Word = 8;                           { Auto mouse delay }
   SysColorAttr : Sw_Word = $4E4F;                       { System colour attr }
   SysMonoAttr  : Sw_Word = $7070;                       { System mono attr }
   StartupMode  : Sw_Word = $FFFF;                       { Compatability only }
   CursorLines  : Sw_Word = $FFFF;                       { Compatability only }
   ScreenBuffer : Pointer = Nil;                      { Compatability only }
   SaveInt09    : Pointer = Nil;                      { Compatability only }
   SysErrorFunc : TSysErrorFunc = {$ifdef FPC}@{$endif}SystemError; { System error ptr }


{***************************************************************************}
{                      UNINITIALIZED PUBLIC VARIABLES                       }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                UNINITIALIZED DOS/DPMI/WIN/NT/OS2 VARIABLES                }
{---------------------------------------------------------------------------}
VAR
   MouseIntFlag: Byte;                                { Mouse in int flag }
   MouseButtons: Byte;                                { Mouse button state }
   ScreenWidth : Byte;                                { Screen text width }
   ScreenHeight: Byte;                                { Screen text height }
   ScreenMode  : TVideoMode;                         { Screen mode }
   MouseWhere  : TPoint;                              { Mouse position }

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                               IMPLEMENTATION
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
{ API Units }
  USES
  FVConsts,
  Keyboard,Mouse;

{***************************************************************************}
{                        PRIVATE INTERNAL CONSTANTS                         }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                 DOS/DPMI MOUSE INTERRUPT EVENT QUEUE SIZE                 }
{---------------------------------------------------------------------------}
CONST EventQSize = 16;                                { Default int bufsize }

{---------------------------------------------------------------------------}
{                DOS/DPMI/WIN/NT/OS2 NEW EVENT QUEUE MAX SIZE               }
{---------------------------------------------------------------------------}
CONST QueueMax = 64;                                  { Max new queue size }

{---------------------------------------------------------------------------}
{   MAX WIEW WIDTH to avoid TDrawBuffer overrun in views unit               }
{---------------------------------------------------------------------------}
CONST MaxViewWidth = 255;                                  { Max view width }

{***************************************************************************}
{                          PRIVATE INTERNAL TYPES                           }
{***************************************************************************}

{***************************************************************************}
{                  PRIVATE INTERNAL INITIALIZED VARIABLES                   }
{***************************************************************************}

{---------------------------------------------------------------------------}
{          DOS/DPMI/WIN/NT/OS2 ALT KEY SCANCODES FROM KEYS (0-127)          }
{---------------------------------------------------------------------------}
CONST AltCodes: Array [0..127] Of Byte = (
      $00, $00, $00, $00, $00, $00, $00, $00,         { $00 - $07 }
      $00, $00, $00, $00, $00, $00, $00, $00,         { $08 - $0F }
      $00, $00, $00, $00, $00, $00, $00, $00,         { $10 - $17 }
      $00, $00, $00, $00, $00, $00, $00, $00,         { $18 - $1F }
      $00, $00, $00, $00, $00, $00, $00, $00,         { $20 - $27 }
      $00, $00, $00, $00, $00, $82, $00, $00,         { $28 - $2F }
      $81, $78, $79, $7A, $7B, $7C, $7D, $7E,         { $30 - $37 }
      $7F, $80, $00, $00, $00, $83, $00, $00,         { $38 - $3F }
      $00, $1E, $30, $2E, $20, $12, $21, $22,         { $40 - $47 }
      $23, $17, $24, $25, $26, $32, $31, $18,         { $48 - $4F }
      $19, $10, $13, $1F, $14, $16, $2F, $11,         { $50 - $57 }
      $2D, $15, $2C, $00, $00, $00, $00, $00,         { $58 - $5F }
      $00, $00, $00, $00, $00, $00, $00, $00,         { $60 - $67 }
      $00, $00, $00, $00, $00, $00, $00, $00,         { $68 - $6F }
      $00, $00, $00, $00, $00, $00, $00, $00,         { $70 - $77 }
      $00, $00, $00, $00, $00, $00, $00, $00);        { $78 - $7F }

{***************************************************************************}
{                  PRIVATE INTERNAL INITIALIZED VARIABLES                   }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                           NEW CONTROL VARIABLES                           }
{---------------------------------------------------------------------------}
CONST
   HideCount : Sw_Integer = 0;                           { Cursor hide count }
   QueueCount: Sw_Word = 0;                              { Queued message count }
   QueueHead : Sw_Word = 0;                              { Queue head pointer }
   QueueTail : Sw_Word = 0;                              { Queue tail pointer }

{***************************************************************************}
{                 PRIVATE INTERNAL UNINITIALIZED VARIABLES                  }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                     UNINITIALIZED DOS/DPMI/API VARIABLES                      }
{---------------------------------------------------------------------------}
VAR
   LastDouble : Boolean;                              { Last double buttons }
   LastButtons: Byte;                                 { Last button state }
   DownButtons: Byte;                                 { Last down buttons }
   EventCount : Sw_Word;                                 { Events in queue }
   AutoDelay  : Sw_Word;                                 { Delay time count }
   DownTicks  : Sw_Word;                                 { Down key tick count }
   AutoTicks  : Sw_Word;                                 { Held key tick count }
   LastWhereX : Sw_Word;                                 { Last x position }
   LastWhereY : Sw_Word;                                 { Last y position }
   DownWhereX : Sw_Word;                                 { Last x position }
   DownWhereY : Sw_Word;                                 { Last y position }
   LastWhere  : TPoint;                               { Last mouse position }
   DownWhere  : TPoint;                               { Last down position }
   EventQHead : Pointer;                              { Head of queue }
   EventQTail : Pointer;                              { Tail of queue }
   EventQueue : Array [0..EventQSize - 1] Of TEvent;  { Event queue }
   EventQLast : RECORD END;                           { Simple end marker }
   StartupScreenMode : TVideoMode;
<<<<<<< HEAD
<<<<<<< HEAD
   {$ifdef OS_AMIGA}
   StartupTicks: Int64; // ticks at Startup for GetDOSTicks
   {$endif}
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
{---------------------------------------------------------------------------}
{  GetDosTicks (18.2 Hz)                                                    }
{---------------------------------------------------------------------------}

Function GetDosTicks:longint; { returns ticks at 18.2 Hz, just like DOS }
{$IFDEF OS_OS2}
  const
    QSV_MS_COUNT = 14;
  var
    L: longint;
  begin
    DosQuerySysInfo (QSV_MS_COUNT, QSV_MS_COUNT, L, 4);
    GetDosTicks := L div 55;
  end;
{$ENDIF}
{$IFDEF OS_UNIX}
  var
     tv : TimeVal;
  {  tz : TimeZone;}
  begin
    FPGetTimeOfDay(@tv,nil{,tz});
    GetDosTicks:=((tv.tv_Sec mod 86400) div 60)*1092+((tv.tv_Sec mod 60)*1000000+tv.tv_USec) div 54945;
  end;
{$ENDIF OS_UNIX}
{$IFDEF OS_WINDOWS}
  begin
     GetDosTicks:=GetTickCount div 55;
  end;
{$ENDIF OS_WINDOWS}
{$IFDEF OS_DOS}
  begin
    GetDosTicks:=MemL[$40:$6c];
  end;
{$ENDIF OS_DOS}
{$IFDEF OS_NETWARE_LIBC}
var
  tv : TTimeVal;
  tz : TTimeZone;
  begin
    fpGetTimeOfDay(tv,tz);
    GetDosTicks:=((tv.tv_sec mod 86400) div 60)*1092+((tv.tv_Sec mod 60)*1000000+tv.tv_USec) div 549
  end;
{$ENDIF}
{$IFDEF OS_NETWARE_CLIB}
  begin
    GetDosTicks := Nwserv.GetCurrentTicks;
  end;
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF OS_AMIGA}
  begin
    GetDosTicks:= ((dos.GetMsCount div 55) - StartupTicks) and $7FFFFFFF;
  end;
{$ENDIF OS_AMIGA}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


procedure GiveUpTimeSlice;
{$IFDEF OS_DOS}
var r: registers;
begin
  Intr ($28, R); (* This is supported everywhere. *)
  r.ax:=$1680;
  intr($2f,r);
end;
{$ENDIF}
{$IFDEF OS_UNIX}
  var
    req,rem : timespec;
begin
  req.tv_sec:=0;
  req.tv_nsec:=10000000;{ 10 ms }
  fpnanosleep(@req,@rem);
end;
{$ENDIF}
{$IFDEF OS_OS2}
begin
 DosSleep (5);
end;
{$ENDIF}
{$IFDEF OS_WINDOWS}
begin
  { if the return value of this call is non zero then
    it means that a ReadFileEx or WriteFileEx have completed
    unused for now ! }
  { wait for 10 ms }
  if SleepEx(10,true)=WAIT_IO_COMPLETION then
    begin
      { here we should handle the completion of the routines
        if we use them }
    end;
end;
{$ENDIF}
{$IFDEF OS_NETWARE_LIBC}
  begin
    Delay (10);
  end;
{$ENDIF}
{$IFDEF OS_NETWARE_CLIB}
  begin
    Delay (10);
  end;
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF OS_AMIGA}
  begin
    { AmigaOS Delay() wait's argument in 1/50 seconds }
    { DOSDelay(2); // the old solution... }
    Keyboard.WaitForSystemEvent(150);
  end;
{$ENDIF OS_AMIGA}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{---------------------------------------------------------------------------}
{                UNINITIALIZED DOS/DPMI/WIN/NT/OS2 VARIABLES                }
{---------------------------------------------------------------------------}
VAR
   SaveExit: Pointer;                                 { Saved exit pointer }
   Queue   : Array [0..QueueMax-1] Of TEvent;         { New message queue }

{***************************************************************************}
{                         PRIVATE INTERNAL ROUTINES                         }
{***************************************************************************}

PROCEDURE ShowMouseCursor;inline;
BEGIN
  ShowMouse;
END;

PROCEDURE HideMouseCursor;inline;
BEGIN
  HideMouse;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{               DOS/DPMI/WIN/NT/OS2 PRIVATE INTERNAL ROUTINES               }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  ExitDrivers -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 08Jun98 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE ExitDrivers; {$IFNDEF PPC_FPC}{$IFNDEF OS_UNIX} FAR; {$ENDIF}{$ENDIF}
BEGIN
   DoneSysError;                                      { Relase error trap }
   DoneEvents;                                        { Close event driver }
{   DoneKeyboard;}
   DoneVideo;
   ExitProc := SaveExit;                              { Restore old exit }
END;

{---------------------------------------------------------------------------}
{  DetectVideo -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19May98 LdB       }
{---------------------------------------------------------------------------}

procedure DetectVideo;
VAR
  CurrMode : TVideoMode;
begin
  { Video.InitVideo; Incompatible with BP
    and forces a screen clear which is often a bad thing PM }
  GetVideoMode(CurrMode);
  ScreenMode:=CurrMode;
end;

{---------------------------------------------------------------------------}
{  DetectMouse -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19May98 LdB       }
FUNCTION DetectMouse: Byte;inline;
begin
  DetectMouse:=Mouse.DetectMouse;
end;

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           BUFFER MOVE ROUTINES                            }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  CStrLen -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25May96 LdB           }
{---------------------------------------------------------------------------}
FUNCTION CStrLen (Const S: String): Sw_Integer;
VAR I, J: Sw_Integer;
BEGIN
   J := 0;                                            { Set result to zero }
   For I := 1 To Length(S) Do
     If (S[I] <> '~') Then Inc(J);                    { Inc count if not ~ }
   CStrLen := J;                                      { Return length }
END;

{---------------------------------------------------------------------------}
{  MoveStr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 10Jul99 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE MoveStr (Var Dest; Const Str: String; Attr: Byte);
VAR I: Word; P: PWord;
BEGIN
   For I := 1 To Length(Str) Do Begin                 { For each character }
     P := @TWordArray(Dest)[I-1];                     { Pointer to Sw_Word }
     If (Attr <> 0) Then WordRec(P^).Hi := Attr;      { Copy attribute }
     WordRec(P^).Lo := Byte(Str[I]);                  { Copy string char }
   End;
END;

{---------------------------------------------------------------------------}
{  MoveCStr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 10Jul99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE MoveCStr (Var Dest; Const Str: String; Attrs: Word);
VAR B: Byte; I, J: Sw_Word; P: PWord;
BEGIN
   J := 0;                                            { Start position }
   For I := 1 To Length(Str) Do Begin                 { For each character }
     If (Str[I] <> '~') Then Begin                    { Not tilde character }
       P := @TWordArray(Dest)[J];                     { Pointer to Sw_Word }
       If (Lo(Attrs) <> 0) Then
         WordRec(P^).Hi := Lo(Attrs);                 { Copy attribute }
       WordRec(P^).Lo := Byte(Str[I]);                { Copy string char }
       Inc(J);                                        { Next position }
     End Else Begin
       B := Hi(Attrs);                                { Hold attribute }
       WordRec(Attrs).Hi := Lo(Attrs);                { Copy low to high }
       WordRec(Attrs).Lo := B;                        { Complete exchange }
     End;
   End;
END;

{---------------------------------------------------------------------------}
{  MoveBuf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 10Jul99 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE MoveBuf (Var Dest, Source; Attr: Byte; Count: Sw_Word);
VAR I: Word; P: PWord;
BEGIN
   For I := 1 To Count Do Begin
     P := @TWordArray(Dest)[I-1];                     { Pointer to Sw_Word }
     If (Attr <> 0) Then WordRec(P^).Hi := Attr;      { Copy attribute }
     WordRec(P^).Lo := TByteArray(Source)[I-1];       { Copy source data }
   End;
END;

{---------------------------------------------------------------------------}
{  MoveChar -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 10Jul99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE MoveChar (Var Dest; C: Char; Attr: Byte; Count: Sw_Word);
VAR I: Word; P: PWord;
BEGIN
   For I := 1 To Count Do Begin
     P := @TWordArray(Dest)[I-1];                     { Pointer to Sw_Word }
     If (Attr <> 0) Then WordRec(P^).Hi := Attr;      { Copy attribute }
     If (Ord(C) <> 0) Then WordRec(P^).Lo := Byte(C); { Copy character }
   End;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        KEYBOARD SUPPORT ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  GetAltCode -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25May96 LdB        }
{---------------------------------------------------------------------------}
FUNCTION GetAltCode (Ch: Char): Word;
BEGIN
   GetAltCode := 0;                                   { Preset zero return }
   Ch := UpCase(Ch);                                  { Convert upper case }
   If (Ch < #128) Then
     GetAltCode := AltCodes[Ord(Ch)] SHL 8            { Return code }
     Else If (Ch = #240) Then GetAltCode := $0200     { Return code }
       Else GetAltCode := 0;                          { Return zero }
END;

{---------------------------------------------------------------------------}
{  GetCtrlCode -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25May96 LdB       }
{---------------------------------------------------------------------------}
FUNCTION GetCtrlCode (Ch: Char): Word;
BEGIN
   GetCtrlCode := GetAltCode(Ch) OR (Ord(Ch) - $40);  { Ctrl+key code }
END;

{---------------------------------------------------------------------------}
{  GetAltChar -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25May96 LdB        }
{---------------------------------------------------------------------------}
FUNCTION GetAltChar (KeyCode: Word): Char;
VAR I: Sw_Integer;
BEGIN
   GetAltChar := #0;                                  { Preset fail return }
   If (Lo(KeyCode) = 0) Then Begin                    { Extended key }
     If (Hi(KeyCode) <= $83) Then Begin               { Highest value in list }
       I := 0;                                        { Start at first }
       While (I < 128) AND (Hi(KeyCode) <> AltCodes[I])
         Do Inc(I);                                   { Search for match }
       If (I < 128) Then GetAltChar := Chr(I);        { Return character }
     End Else
       If (Hi(KeyCode)=$02) Then GetAltChar := #240;  { Return char }
   End;
END;

{---------------------------------------------------------------------------}
{  GetCtrlChar -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25May96 LdB       }
{---------------------------------------------------------------------------}
FUNCTION GetCtrlChar (KeyCode: Word): Char;
VAR C: Char;
BEGIN
  C := #0;                                            { Preset #0 return }
  If (Lo(KeyCode) > 0) AND (Lo(KeyCode) <= 26) Then   { Between 1-26 }
    C := Chr(Lo(KeyCode) + $40);                      { Return char A-Z }
  GetCtrlChar := C;                                   { Return result }
END;

{---------------------------------------------------------------------------}
{  CtrlToArrow -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 25May96 LdB       }
{---------------------------------------------------------------------------}
FUNCTION CtrlToArrow (KeyCode: Word): Word;
CONST NumCodes = 11;
      CtrlCodes : Array [0..NumCodes-1] Of Char =
        (#19, #4, #5, #24, #1, #6, #7, #22, #18, #3, #8);
      ArrowCodes: Array [0..NumCodes-1] Of Sw_Word =
       (kbLeft, kbRight, kbUp, kbDown, kbHome, kbEnd, kbDel, kbIns,
        kbPgUp, kbPgDn, kbBack);
VAR I: Sw_Integer;
BEGIN
   CtrlToArrow := KeyCode;                            { Preset key return }
   For I := 0 To NumCodes - 1 Do
     If WordRec(KeyCode).Lo = Byte(CtrlCodes[I])      { Matches a code }
     Then Begin
       CtrlToArrow := ArrowCodes[I];                  { Return key stroke }
       Exit;                                          { Now exit }
     End;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        KEYBOARD CONTROL ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  GetShiftState -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 08Jul96 LdB     }
{---------------------------------------------------------------------------}
FUNCTION GetShiftState: Byte;
begin
  GetShiftState:=Keyboard.GetKeyEventShiftState(Keyboard.PollShiftStateEvent);
end;

{---------------------------------------------------------------------------}
{  GetKeyEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19May98 LdB       }
{---------------------------------------------------------------------------}
procedure GetKeyEvent (Var Event: TEvent);
var
  key      : TKeyEvent;
  keycode  : Word;
  keyshift : byte;
begin
  if Keyboard.PollKeyEvent<>0 then
   begin
     key:=Keyboard.GetKeyEvent;
     keycode:=Keyboard.GetKeyEventCode(key);
     keyshift:=KeyBoard.GetKeyEventShiftState(key);
     // some kbds still honour old XT E0 prefix. (org IBM ps/2, win98?) bug #8978
     if (keycode and $FF = $E0) and
<<<<<<< HEAD
<<<<<<< HEAD
        (byte(keycode shr 8) in
              [$1C,$1D,$2A,$35..$38,$46..$49,$4b,$4d,$4f,$50..$53]) Then
          keycode := keycode and $FF00;

=======
=======
>>>>>>> origin/fixes_2_2
        (byte(keycode shr 8) in  
              [$1C,$1D,$2A,$35..$38,$46..$49,$4b,$4d,$4f,$50..$53]) Then
          keycode := keycode and $FF00;
     
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     { fixup shift-keys }
     if keyshift and kbShift<>0 then
       begin
         case keycode of
           $5200 : keycode:=kbShiftIns;
           $5300 : keycode:=kbShiftDel;
           $8500 : keycode:=kbShiftF1;
           $8600 : keycode:=kbShiftF2;
         end;
       end
     { fixup ctrl-keys }
     else if keyshift and kbCtrl<>0 then
       begin
         case keycode of
           $5200,
           $9200 : keycode:=kbCtrlIns;
           $5300,
           $9300 : keycode:=kbCtrlDel;
         end;
       end
     { fixup alt-keys }
     else if keyshift and kbAlt<>0 then
       begin
         case keycode of
           $0e08,
           $0e00 : keycode:=kbAltBack;
         end;
       end
     { fixup normal keys }
     else
       begin
         case keycode of
           $e00d : keycode:=kbEnter;
         end;
       end;
     Event.What:=evKeyDown;
     Event.KeyCode:=keycode;
{$ifdef ENDIAN_LITTLE}
     Event.CharCode:=chr(keycode and $ff);
     Event.ScanCode:=keycode shr 8;
{$endif ENDIAN_LITTLE}
     Event.KeyShift:=keyshift;
   end
  else
   Event.What:=evNothing;
end;


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          MOUSE CONTROL ROUTINES                           }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  HideMouse -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jun98 LdB         }
{---------------------------------------------------------------------------}
procedure HideMouse;
begin
{ Is mouse hidden yet?
  If (HideCount = 0) Then}
    Mouse.HideMouse;
{  Inc(HideCount);}
end;

{---------------------------------------------------------------------------}
{  ShowMouse -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jun98 LdB         }
{---------------------------------------------------------------------------}
procedure ShowMouse;
begin
{  if HideCount>0 then
    dec(HideCount);
  if (HideCount=0) then}
   Mouse.ShowMouse;
end;

{---------------------------------------------------------------------------}
{  GetMouseEvent -> Platforms DOS/DPMI/WINDOWS/OS2 - Updated 30Jun98 LdB    }
{---------------------------------------------------------------------------}
procedure GetMouseEvent (Var Event: TEvent);
var
  e : Mouse.TMouseEvent;
begin
  if Mouse.PollMouseEvent(e) then
   begin
     Mouse.GetMouseEvent(e);
     MouseWhere.X:=e.x;
     MouseWhere.Y:=e.y;
     Event.Double:=false;
     case e.Action of
       MouseActionMove :
         Event.What:=evMouseMove;
       MouseActionDown :
         begin
           Event.What:=evMouseDown;
           if (DownButtons=e.Buttons) and (LastWhere.X=MouseWhere.X) and (LastWhere.Y=MouseWhere.Y) and
              (GetDosTicks-DownTicks<=DoubleDelay) then
             Event.Double:=true;
           DownButtons:=e.Buttons;
           DownWhere.X:=MouseWhere.x;
           DownWhere.Y:=MouseWhere.y;
           DownTicks:=GetDosTicks;
           AutoTicks:=GetDosTicks;
           if AutoTicks=0 then
             AutoTicks:=1;
           AutoDelay:=RepeatDelay;
         end;
       MouseActionUp :
         begin
           AutoTicks:=0;
           Event.What:=evMouseUp;
           AutoTicks:=0;
         end;
     end;
     Event.Buttons:=e.Buttons;
     Event.Where.X:=MouseWhere.x;
     Event.Where.Y:=MouseWhere.y;
     LastButtons:=Event.Buttons;
     LastWhere.x:=Event.Where.x;
     LastWhere.y:=Event.Where.y;
   end
  else if (AutoTicks <> 0) and (GetDosTicks >= AutoTicks + AutoDelay) then
   begin
     Event.What:=evMouseAuto;
     Event.Buttons:=LastButtons;
     Event.Where.X:=LastWhere.x;
     Event.Where.Y:=LastWhere.y;
     AutoTicks:=GetDosTicks;
     AutoDelay:=1;
   end
  else
   FillChar(Event,sizeof(TEvent),0);
  if MouseReverse and ((Event.Buttons and 3) in [1,2]) then
    Event.Buttons := Event.Buttons xor 3;
end;

{---------------------------------------------------------------------------}
{  GetSystemEvent                                                           }
{---------------------------------------------------------------------------}
procedure GetSystemEvent (Var Event: TEvent);
var
  SysEvent : TsystemEvent;
begin
  if PollSystemEvent(SysEvent) then
    begin
      SysMsg.GetSystemEvent(SysEvent);
      case SysEvent.typ of
      SysNothing :
        Event.What:=evNothing;
      SysSetFocus :
        begin
          Event.What:=evBroadcast;
          Event.Command:=cmReceivedFocus;
        end;
      SysReleaseFocus :
        begin
          Event.What:=evBroadcast;
          Event.Command:=cmReleasedFocus;
        end;
      SysClose :
        begin
          Event.What:=evCommand;
          Event.Command:=cmQuitApp;
        end;
      SysResize :
        begin
          Event.What:=evCommand;
          Event.Command:=cmResizeApp;
          Event.Id:=SysEvent.x;
          Event.InfoWord:=SysEvent.y;
        end;
      else
        Event.What:=evNothing;
      end;
    end
  else
    Event.What:=evNothing;
end;


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                      EVENT HANDLER CONTROL ROUTINES                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  InitEvents -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 07Sep99 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE InitEvents;
BEGIN
  If (ButtonCount <> 0) Then
    begin                   { Mouse is available }
     Mouse.InitMouse;                                 { Hook the mouse }
     { this is required by the use of HideCount variable }
     Mouse.ShowMouse;                                 { visible by default }
     { HideCount:=0;  }
     LastButtons := 0;                                { Clear last buttons }
     DownButtons := 0;                                { Clear down buttons }
     MouseWhere.X:=Mouse.GetMouseX;
     MouseWhere.Y:=Mouse.GetMouseY;                   { Get mouse position }
     LastWhere.x:=MouseWhere.x;
     LastWhereX:=MouseWhere.x;
     LastWhere.y:=MouseWhere.y;
     LastWhereY:=MouseWhere.y;
     MouseEvents := True;                             { Set initialized flag }
    end;
  InitSystemMsg;
END;

{---------------------------------------------------------------------------}
{  DoneEvents -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 30Jul99 LdB        }
{---------------------------------------------------------------------------}
PROCEDURE DoneEvents;
BEGIN
  DoneSystemMsg;
  Mouse.DoneMouse;
  MouseEvents:=false;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           VIDEO CONTROL ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

const
  VideoInitialized : boolean = false;

{---------------------------------------------------------------------------}
{  InitKeyboard -> Platforms ALL - 07May06 DM                               }
{---------------------------------------------------------------------------}

procedure initkeyboard;inline;

begin
  keyboard.initkeyboard;
end;

{---------------------------------------------------------------------------}
{  DoneKeyboard -> Platforms ALL - 07May06 DM                               }
{---------------------------------------------------------------------------}

procedure donekeyboard;inline;

begin
  keyboard.donekeyboard;
end;

{---------------------------------------------------------------------------}
{  InitVideo -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 26Nov99 LdB         }
{---------------------------------------------------------------------------}
function InitVideo:boolean;

var StoreScreenMode : TVideoMode;

begin
  initvideo:=false;
  if VideoInitialized then
    begin
      StoreScreenMode:=ScreenMode;
      DoneVideo;
    end
  else
    StoreScreenMode.Col:=0;

  Video.InitVideo;
  if video.errorcode<>viook then
    exit;
  GetVideoMode(StartupScreenMode);
  GetVideoMode(ScreenMode);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef OS_WINDOWS}
  { Force the console to the current screen mode }
  Video.SetVideoMode(ScreenMode);
{$endif OS_WINDOWS}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef win32}
  { Force the console to the current screen mode }
  Video.SetVideoMode(ScreenMode);
{$endif win32}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  If (StoreScreenMode.Col<>0) and
     ((StoreScreenMode.color<>ScreenMode.color) or
     (StoreScreenMode.row<>ScreenMode.row) or
     (StoreScreenMode.col<>ScreenMode.col)) then
    begin
      Video.SetVideoMode(StoreScreenMode);
      GetVideoMode(ScreenMode);
    end;

  if ScreenWidth > MaxViewWidth then
    ScreenWidth := MaxViewWidth;
  ScreenWidth:=Video.ScreenWidth;
  ScreenHeight:=Video.ScreenHeight;
  VideoInitialized:=true;
  initvideo:=true;
end;

{---------------------------------------------------------------------------}
{  DoneVideo -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19May98 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE DoneVideo;
BEGIN
  if not VideoInitialized then
    exit;
  Video.SetVideoMode(StartupScreenMode);
  Video.ClearScreen;
  Video.SetCursorPos(0,0);
  Video.DoneVideo;
  VideoInitialized:=false;
END;

{---------------------------------------------------------------------------}
{  ClearScreen -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 04Jan97 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE ClearScreen;
BEGIN
  Video.ClearScreen;
END;

{---------------------------------------------------------------------------}
{  SetVideoMode -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 10Nov99 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE SetVideoMode (Mode: Sw_Word);
BEGIN
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           ERROR CONTROL ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  InitSysError -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 20May98 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE InitSysError;
BEGIN
   SysErrActive := True;                              { Set active flag }
END;

{---------------------------------------------------------------------------}
{  DoneSysError -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 20May98 LdB      }
{---------------------------------------------------------------------------}
PROCEDURE DoneSysError;
BEGIN
   SysErrActive := False;                             { Clear active flag }
END;

{---------------------------------------------------------------------------}
{  SystemError -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 20May98 LdB       }
{---------------------------------------------------------------------------}
FUNCTION SystemError (ErrorCode: Sw_Integer; Drive: Byte): Sw_Integer;
BEGIN
   If (FailSysErrors = False) Then Begin              { Check error ignore }

   End Else SystemError := 1;                         { Return 1 for ignored }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           STRING FORMAT ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  PrintStr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 18Feb99 LdB          }
{---------------------------------------------------------------------------}
PROCEDURE PrintStr (CONST S: String);
BEGIN
   Write(S);                                          { Write to screen }
END;

{---------------------------------------------------------------------------}
{  FormatStr -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 13Jul99 LdB         }
{---------------------------------------------------------------------------}
procedure FormatStr (Var Result: String; CONST Format: String; Var Params);
TYPE TLongArray = Array[0..0] Of PtrInt;
VAR W, ResultLength : integer;
    FormatIndex, Justify, Wth: Byte;
    Fill: Char; S: String;

   FUNCTION LongToStr (L: Longint; Radix: Byte): String;
   CONST HexChars: Array[0..15] Of Char =
    ('0', '1', '2', '3', '4', '5', '6', '7',
     '8', '9', 'A', 'B', 'C', 'D', 'E', 'F');
   VAR I: LongInt; S: String; Sign: String[1];
   begin
     LongToStr := '';                                 { Preset empty return }
     If (L < 0) Then begin                            { If L is negative }
       Sign := '-';                                   { Sign is negative }
       L := Abs(L);                                   { Convert to positive }
     end Else Sign := '';                             { Sign is empty }
     S := '';                                         { Preset empty string }
     Repeat
       I := L MOD Radix;                              { Radix mod of value }
       S := HexChars[I] + S;                          { Add char to string }
       L := L DIV Radix;                              { Divid by radix }
     Until (L = 0);                                   { Until no remainder }
     LongToStr := Sign + S;                           { Return result }
   end;

   procedure HandleParameter (I : LongInt);
   begin
     While (FormatIndex <= Length(Format)) Do begin   { While length valid }
       if ResultLength>=High(Result) then
         exit;
       While (FormatIndex <= Length(Format)) and
             (Format[FormatIndex] <> '%')          { Param char not found }
        Do begin
         Result[ResultLength+1] := Format[FormatIndex]; { Transfer character }
         Inc(ResultLength);                           { One character added }
         Inc(FormatIndex);                            { Next param char }
       end;
       If (FormatIndex < Length(Format)) and          { Not last char and }
       (Format[FormatIndex] = '%') Then begin         { '%' char found }
         Fill := ' ';                                 { Default fill char }
         Justify := 0;                                { Default justify }
         Wth := 0;                                    { Default 0=no width }
         Inc(FormatIndex);                            { Next character }
         If (Format[FormatIndex] = '0') Then
           Fill := '0';                               { Fill char to zero }
         If (Format[FormatIndex] = '-') Then begin    { Optional just char }
           Justify := 1;                              { Right justify }
           Inc(FormatIndex);                          { Next character }
         end;
         While ((FormatIndex <= Length(Format)) and   { Length still valid }
         (Format[FormatIndex] >= '0') and
         (Format[FormatIndex] <= '9')) Do begin       { Numeric character }
           Wth := Wth * 10;                           { Multiply x10 }
           Wth := Wth + Ord(Format[FormatIndex])-$30; { Add numeric value }
           Inc(FormatIndex);                          { Next character }
         end;
         If ((FormatIndex <= Length(Format)) and      { Length still valid }
         (Format[FormatIndex] = '#')) Then begin      { Parameter marker }
           Inc(FormatIndex);                          { Next character }
           HandleParameter(Wth);                      { Width is param idx }
         end;
         If (FormatIndex <= Length(Format)) Then begin{ Length still valid }
           Case Format[FormatIndex] Of
           '%': begin                               { Literal % }
             S := '%';
             Inc(FormatIndex);
             Move(S[1], Result[ResultLength+1], 1);
             Inc(ResultLength,Length(S));
             Continue;
           end;
           'c': S := Char(TLongArray(Params)[I]);  { Character parameter }
             'd': S := LongToStr(TLongArray(Params)[I],
               10);                                   { Decimal parameter }
             's': S := PString(TLongArray(Params)[I])^;{ String parameter }
             'x': S := LongToStr(TLongArray(Params)[I],
               16);                                   { Hex parameter }
           end;
           Inc(FormatIndex);                          { Next character }
           If (Wth > 0) Then begin                    { Width control active }
             If (Length(S) > Wth) Then begin          { We must shorten S }
               If (Justify=1) Then                    { Check right justify }
                 S := Copy(S, Length(S)-Wth+1, Wth)   { Take right side data }
                 Else S := Copy(S, 1, Wth);           { Take left side data }
             end Else begin                           { We must pad out S }
               If (Justify=1) Then                    { Right justify }
                 While (Length(S) < Wth) Do
                   S := S+Fill Else                   { Right justify fill }
                 While (Length(S) < Wth) Do
                   S := Fill + S;                     { Left justify fill }
             end;
           end;
           W:=Length(S);
           if W+ResultLength+1>High(Result) then
             W:=High(Result)-ResultLength;
           Move(S[1], Result[ResultLength+1],
             W);                                      { Move data to result }
           Inc(ResultLength,W);                       { Adj result length }
           Inc(I);
         end;
       end;
     end;
   end;

begin
   ResultLength := 0;                                 { Zero result length }
   FormatIndex := 1;                                  { Format index to 1 }
   HandleParameter(0);                                { Handle parameter }
   Result[0] := Chr(ResultLength);                    { Set string length }
end;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                    NEW QUEUED EVENT HANDLER ROUTINES                      }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  PutEventInQueue -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Mar98 LdB   }
{---------------------------------------------------------------------------}
FUNCTION PutEventInQueue (Var Event: TEvent): Boolean;
BEGIN
   If (QueueCount < QueueMax) Then Begin              { Check room in queue }
     Queue[QueueHead] := Event;                       { Store event }
     Inc(QueueHead);                                  { Inc head position }
     If (QueueHead = QueueMax) Then QueueHead := 0;   { Roll to start check }
     Inc(QueueCount);                                 { Inc queue count }
     PutEventInQueue := True;                         { Return successful }
   End Else PutEventInQueue := False;                 { Return failure }
END;

{---------------------------------------------------------------------------}
{  NextQueuedEvent -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 17Mar98 LdB   }
{---------------------------------------------------------------------------}
PROCEDURE NextQueuedEvent(Var Event: TEvent);
BEGIN
   If (QueueCount > 0) Then Begin                     { Check queued event }
     Event := Queue[QueueTail];                       { Fetch next event }
     Inc(QueueTail);                                  { Inc tail position }
     If (QueueTail = QueueMax) Then QueueTail := 0;   { Roll to start check }
     Dec(QueueCount);                                 { Dec queue count }
   End Else Event.What := evNothing;                  { Return empty event }
END;

{***************************************************************************}
{                      UNIT INITIALIZATION ROUTINE                          }
{***************************************************************************}
BEGIN
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF OS_AMIGA}
  StartupTicks := (dos.GetMsCount div 55);
{$ENDIF}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   ButtonCount := DetectMouse;                        { Detect mouse }
   DetectVideo;                                       { Detect video }
{   InitKeyboard;}
   InitSystemMsg;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef OS_WINDOWS}
=======
{$ifdef win32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef win32}
>>>>>>> origin/fixes_2_2
   SetFileApisToOEM;
{$endif}

   SaveExit := ExitProc;                              { Save old exit }
   ExitProc := @ExitDrivers;                          { Set new exit }
END.
