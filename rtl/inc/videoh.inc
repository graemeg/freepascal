{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
  PVideoMode = ^TVideoMode;
  TVideoMode = record
    Col,Row : Word;
    Color   : Boolean;
  end;
  TVideoModeSelector = function (const VideoMode: TVideoMode; Params: Longint): Boolean;

  TVideoCell = Word;
  PVideoCell = ^TVideoCell;

  TVideoBuf = array[0..32759] of TVideoCell;
  PVideoBuf = ^TVideoBuf;

  TVideoDriver = Record
    InitDriver        : Procedure;
    DoneDriver        : Procedure;
    UpdateScreen      : Procedure(Force : Boolean);
    ClearScreen       : Procedure;
    SetVideoMode      : Function (Const Mode : TVideoMode) : Boolean;
    GetVideoModeCount : Function : Word;
    GetVideoModeData  : Function(Index : Word; Var Data : TVideoMode) : Boolean;
    SetCursorPos      : procedure (NewCursorX, NewCursorY: Word);
    GetCursorType     : function : Word;
    SetCursorType     : procedure (NewType: Word);
    GetCapabilities   : Function : Word;
  end;

const
  { Foreground and background color constants }
  Black         = 0;
  Blue          = 1;
  Green         = 2;
  Cyan          = 3;
  Red           = 4;
  Magenta       = 5;
  Brown         = 6;
  LightGray     = 7;

  { Foreground color constants }
  DarkGray      = 8;
  LightBlue     = 9;
  LightGreen    = 10;
  LightCyan     = 11;
  LightRed      = 12;
  LightMagenta  = 13;
  Yellow        = 14;
  White         = 15;

  { Add-in for blinking }
  Blink         = 128;

  { Capabilities bitmask }
  cpUnderLine     = $0001;
  cpBlink         = $0002;
  cpColor         = $0004;
  cpChangeFont    = $0008;
  cpChangeMode    = $0010;
  cpChangeCursor  = $0020;

  { Possible cursor types }
  crHidden        = 0;
  crUnderLine     = 1;
  crBlock         = 2;
  crHalfBlock     = 3;

  { Possible error codes }
  vioOK              = 0;
  errVioBase         = 1000;
  errVioInit         = errVioBase + 1; { Initialization error, shouldn't occur on DOS, but may
                         on Linux }
  errVioNotSupported = errVioBase + 2; { call to an unsupported function }
  errVioNoSuchMode   = errVioBase + 3; { No such video mode }

const
  ScreenWidth  : Word = 0;
  ScreenHeight : Word = 0;

var
  ScreenColor  : Boolean;
  CursorX,
  CursorY      : Word;
  VideoBuf,
  OldVideoBuf  : PVideoBuf;
  VideoBufSize : Longint;
  CursorLines  : Byte;

const {The following constants were variables in the past.
       - Lowascii was set to true if ASCII characters < 32 were available
       - NoExtendedFrame was set to true if the double with line drawing
         characters were set to true.

      These variables did exist because of VT100 limitations on Unix. However,
      only part of the character set problem was solved this way. Nowadays, the
      video unit converts characters to the output character set (which might be
      VT100) automatically, so the user does not need to worry about it anymore.}
      LowAscii = true;
      NoExtendedFrame = false;

      FVMaxWidth = 240;

Procedure LockScreenUpdate;
{ Increments the screen update lock count with one.}
Procedure UnlockScreenUpdate;
{ Decrements the screen update lock count with one.}
Function GetLockScreenCount : integer;
{ Gets the current lock level }
Function SetVideoDriver (Const Driver : TVideoDriver) : Boolean;
{ Sets the videodriver to be used }
Procedure GetVideoDriver (Var Driver : TVideoDriver);
{ Retrieves the current videodriver }

procedure InitVideo;
{ Initializes the video subsystem }
procedure DoneVideo;
{ Deinitializes the video subsystem }
function GetCapabilities: Word;
{ Return the capabilities of the current environment }
procedure ClearScreen;
{ Clears the screen }
procedure UpdateScreen(Force: Boolean);
{ Force specifies whether the whole screen has to be redrawn, or (if target
  platform supports it) its parts only }
procedure SetCursorPos(NewCursorX, NewCursorY: Word);
{ Position the cursor to the given position }
function GetCursorType: Word;
{ Return the cursor type: Hidden, UnderLine or Block }
procedure SetCursorType(NewType: Word);
{ Set the cursor to the given type }

procedure GetVideoMode(var Mode: TVideoMode);
{ Return dimensions of the current video mode }
Function SetVideoMode(Const Mode: TVideoMode) : Boolean;
{ Set video-mode to have Mode dimensions, may return errVioNoSuchMode }
Function GetVideoModeCount : Word;
{ Get the number of video modes supported by this driver }
Function GetVideoModeData(Index : Word; Var Data: TVideoMode) : Boolean;
{ Get the data for Video mode Index. Index is zero based. }

type
  TErrorHandlerReturnValue = (errRetry, errAbort, errContinue);
  { errRetry = retry the operation,
    errAbort = abort, return error code,
    errContinue = abort, without returning errorcode }

  TErrorHandler = function (Code: Longint; Info: Pointer): TErrorHandlerReturnValue;
    { ErrorHandler is the standard procedural interface for all error functions.
      Info may contain any data type specific to the error code passed to the
      function. }

function DefaultErrorHandler(AErrorCode: Longint; AErrorInfo: Pointer): TErrorHandlerReturnValue;
{ Default error handler, simply sets error code, and returns errContinue }

const
  errOk              = 0;
  ErrorCode: Longint = ErrOK;
  ErrorInfo: Pointer = nil;
  ErrorHandler: TErrorHandler = @DefaultErrorHandler;

