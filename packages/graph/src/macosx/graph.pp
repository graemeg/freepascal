{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl

    This file implements the linux GGI support for the graph unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Graph;
interface

uses
  { in the interface so the graphh definitions of moveto etc override }
  { the ones in the universal interfaces                              }
<<<<<<< HEAD
<<<<<<< HEAD
  MacOSAll;
=======
  FPCMacOSAll;
>>>>>>> graemeg/fixes_2_2
=======
  FPCMacOSAll;
>>>>>>> origin/fixes_2_2

{$pascalmainname FPCMacOSXGraphMain}

{$i graphh.inc}

Const
  { Supported modes }
  G320x200x16       = 1;
  G640x200x16       = 2;
  G640x350x16       = 3;
  G640x480x16       = 4;
  G320x200x256      = 5;
  G320x240x256      = 6;
  G320x400x256      = 7;
  G360x480x256      = 8;
  G640x480x2        = 9;

  G640x480x256      = 10;
  G800x600x256      = 11;
  G1024x768x256     = 12;

  G1280x1024x256    = 13;   { Additional modes. }

  G320x200x32K      = 14;
  G320x200x64K      = 15;
  G320x200x16M      = 16;
  G640x480x32K      = 17;
  G640x480x64K      = 18;
  G640x480x16M      = 19;
  G800x600x32K      = 20;
  G800x600x64K      = 21;
  G800x600x16M      = 22;
  G1024x768x32K     = 23;
  G1024x768x64K     = 24;
  G1024x768x16M     = 25;
  G1280x1024x32K    = 26;
  G1280x1024x64K    = 27;
  G1280x1024x16M    = 28;

  G800x600x16       = 29;
  G1024x768x16      = 30;
  G1280x1024x16     = 31;

  G720x348x2        = 32;               { Hercules emulation mode }

  G320x200x16M32    = 33;       { 32-bit per pixel modes. }
  G640x480x16M32    = 34;
  G800x600x16M32    = 35;
  G1024x768x16M32   = 36;
  G1280x1024x16M32  = 37;

  { additional resolutions }
  G1152x864x16      = 38;
  G1152x864x256     = 39;
  G1152x864x32K     = 40;
  G1152x864x64K     = 41;
  G1152x864x16M     = 42;
  G1152x864x16M32   = 43;

  G1600x1200x16     = 44;
  G1600x1200x256    = 45;
  G1600x1200x32K    = 46;
  G1600x1200x64K    = 47;
  G1600x1200x16M    = 48;
  G1600x1200x16M32  = 49;


implementation

uses
  { for FOUR_CHAR_CODE }
  macpas,
  baseunix,
  unix,
  ctypes,
  pthreads;

const
  InternalDriverName = 'Quartz';

  kEventClassFPCGraph = $46504367; // 'FPCg'
  kEventInitGraph     = $496E6974; // 'Init'
  kEventFlush         = $466c7368; // 'Flsh'
  kEventCloseGraph    = $446f6e65; // 'Done'
  kEventQuit          = $51756974; // 'Quit'
  
  kEventGraphInited   = $49746564 ; // Ited;
  kEventGraphClosed   = $436c6564 ; // Cled;

//  initGraphSpec  : EventTypeSpec = (eventClass: kEventClassFPCGraph; eventKind: kEventInitGraph);
//  flushGraphSpec : EventTypeSpec = (eventClass: kEventClassFPCGraph; eventKind: kEventFlush);
//  closeGraphSpec  : EventTypeSpec = (eventClass: kEventClassFPCGraph; eventKind: kEventCloseGraph);
  allGraphSpec: array[0..3] of EventTypeSpec = ((eventClass: kEventClassFPCGraph; eventKind: kEventInitGraph),
                                                (eventClass: kEventClassFPCGraph; eventKind: kEventFlush),
                                                (eventClass: kEventClassFPCGraph; eventKind: kEventCloseGraph),
                                                (eventClass: kEventClassFPCGraph; eventKind: kEventQuit));

  GraphInitedSpec: array[0..0] of EventTypeSpec = ((eventClass: kEventClassFPCGraph; eventKind: kEventGraphInited));
  GraphClosedSpec: array[0..0] of EventTypeSpec = ((eventClass: kEventClassFPCGraph; eventKind: kEventGraphClosed));

{$i graph.inc}

  type
    PByte = ^Byte;
    PLongInt = ^LongInt;

    PByteArray = ^TByteArray;
    TByteArray = array [0..MAXINT - 1] of Byte;

  var
    graphdrawing: TRTLCriticalSection;

{ ---------------------------------------------------------------------
   SVGA bindings.

  ---------------------------------------------------------------------}

Const
  { Text }

  WRITEMODE_OVERWRITE = 0;
  WRITEMODE_MASKED    = 1;
  FONT_EXPANDED       = 0;
  FONT_COMPRESSED     = 2;

 { Types }
 type
  PGraphicsContext = ^TGraphicsContext;
  TGraphicsContext = record
                       ModeType: Byte;
                       ModeFlags: Byte;
                       Dummy: Byte;
                       FlipPage: Byte;
                       Width: LongInt;
                       Height: LongInt;
                       BytesPerPixel: LongInt;
                       Colors: LongInt;
                       BitsPerPixel: LongInt;
                       ByteWidth: LongInt;
                       VBuf: pointer;
                       Clip: LongInt;
                       ClipX1: LongInt;
                       ClipY1: LongInt;
                       ClipX2: LongInt;
                       ClipY2: LongInt;
                       ff: pointer;
                     end;

Const
  GLASTMODE         = 49;
  ModeNames : Array[0..GLastMode] of string [18] =
   ('Text',
    'G320x200x16',
    'G640x200x16',
    'G640x350x16',
    'G640x480x16',
    'G320x200x256',
    'G320x240x256',
    'G320x400x256',
    'G360x480x256',
    'G640x480x2',
    'G640x480x256',
    'G800x600x256',
    'G1024x768x256',
    'G1280x1024x256',
    'G320x200x32K',
    'G320x200x64K',
    'G320x200x16M',
    'G640x480x32K',
    'G640x480x64K',
    'G640x480x16M',
    'G800x600x32K',
    'G800x600x64K',
    'G800x600x16M',
    'G1024x768x32K',
    'G1024x768x64K',
    'G1024x768x16M',
    'G1280x1024x32K',
    'G1280x1024x64K',
    'G1280x1024x16M',
    'G800x600x16',
    '1024x768x16',
    '1280x1024x16',
    'G720x348x2',
    'G320x200x16M32',
    'G640x480x16M32',
    'G800x600x16M32',
    'G1024x768x16M32',
    'G1280x1024x16M32',
    'G1152x864x16',
    'G1152x864x256',
    'G1152x864x32K',
    'G1152x864x64K',
    'G1152x864x16M',
    'G1152x864x16M32',
    'G1600x1200x16',
    'G1600x1200x256',
    'G1600x1200x32K',
    'G1600x1200x64K',
    'G1600x1200x16M',
    'G1600x1200x16M32');


{ ---------------------------------------------------------------------
    Mac OS X - specific stuff
  ---------------------------------------------------------------------}


var
  { where all the drawing occurs }
  offscreen: CGContextRef;
  { the drawing window's contents to which offscreen is flushed }
  graphHIView: HIViewRef;
  { the drawing window itself }
  myMainWindow: WindowRef;
  maineventqueue: EventQueueRef;
  updatepending: boolean;

  colorpalette: array[0..255,1..3] of single;


{ create a new offscreen bitmap context in which we can draw (and from }
{ which we can read again)                                             }
function CreateBitmapContext (pixelsWide, pixelsHigh: SInt32) : CGContextRef;
var
    colorSpace        : CGColorSpaceRef;
    bitmapData        : Pointer;
    bitmapByteCount   : SInt32;
    bitmapBytesPerRow : SInt32;
begin
  CreateBitmapContext := nil;

  bitmapBytesPerRow   := (pixelsWide * 4);// always draw in 24 bit colour (+ 8 bit alpha)
  bitmapByteCount     := (bitmapBytesPerRow * pixelsHigh);

  colorSpace := CGColorSpaceCreateDeviceRGB;// 2
  bitmapData := getmem ( bitmapByteCount );// 3
  if (bitmapData = nil) then
    exit;

  CreateBitmapContext := CGBitmapContextCreate (bitmapData,
                                  pixelsWide,
                                  pixelsHigh,
                                  8,      // bits per component
                                  bitmapBytesPerRow,
                                  colorSpace,
                                  kCGImageAlphaPremultipliedLast);
  if (CreateBitmapContext = nil) then
    begin
      system.freemem (bitmapData);
      writeln (stderr, 'Could not create graphics context!');
      exit;
    end;
    CGColorSpaceRelease( colorSpace );
    { disable anti-aliasing }
    CGContextTranslateCTM(CreateBitmapContext,0.5,0.5);
end;


{ dispose the offscreen bitmap context }
procedure DisposeBitmapContext(var bmContext: CGContextRef);
begin
  system.freemem(CGBitmapContextGetData(bmContext));
  CGContextRelease(bmContext);
  bmContext:=nil;
end;


{ create a HIView to add to a window, in which we can then draw }
function CreateHIView (inWindow: WindowRef; const inBounds: Rect; var outControl: HIObjectRef): OSStatus;
  var
    root  : ControlRef;
    event : EventRef;
    err   : OSStatus;
  label
    CantCreate, CantGetRootControl, CantSetParameter, CantCreateEvent{, CantRegister};
  begin
    // Make an initialization event
    err := CreateEvent( nil, kEventClassHIObject, kEventHIObjectInitialize,
                        GetCurrentEventTime(), 0, event );
    if (err <> noErr) then
      goto CantCreateEvent;
 
    // If bounds were specified, push the them into the initialization event
    // so that they can be used in the initialization handler.
    err := SetEventParameter( event, FOUR_CHAR_CODE('boun'), typeQDRectangle,
           sizeof( Rect ), @inBounds );
    if (err <> noErr) then
      goto CantSetParameter;

    err := HIObjectCreate( { kHIViewClassID } CFSTR('com.apple.hiview'), event, outControl );
    assert(err = noErr);
 
    // If a parent window was specified, place the new view into the
    // parent window.
    err := GetRootControl( inWindow, root );
    if (err <> noErr) then
      goto CantGetRootControl;
    err := HIViewAddSubview( root, outControl );
    if (err <> noErr) then
      goto CantGetRootControl;

    err := HIViewSetVisible(outControl, true);
 
CantCreate:
CantGetRootControl:
CantSetParameter:
CantCreateEvent:
    ReleaseEvent( event );
 
    CreateHIView := err;
  end;


{ Event handler which does the actual drawing by copying the offscreen to }
{ the HIView of the drawing window                                        }
function MyDrawEventHandler (myHandler: EventHandlerCallRef; 
                        event: EventRef; userData: pointer): OSStatus; mwpascal;
  var
    myContext: CGContextRef;
    bounds: HIRect;
    img: CGImageRef;
  begin
//      writeln('event');
      MyDrawEventHandler := GetEventParameter (event, // 1
                              kEventParamCGContextRef, 
                              typeCGContextRef, 
                              nil, 
                              sizeof (CGContextRef),
                              nil,
                              @myContext);
    if (MyDrawEventHandler <> noErr) then
      exit;
    MyDrawEventHandler := HIViewGetBounds (HIViewRef(userData), bounds);
    if (MyDrawEventHandler <> noErr) then
      exit;
    EnterCriticalSection(graphdrawing);
    img:=CGBitmapContextCreateImage(offscreen);
    CGContextDrawImage(myContext,
                       bounds,
                       img);
    updatepending:=false;
    LeaveCriticalSection(graphdrawing);
    CGImageRelease(img);
end;


{ force the draw event handler to fire }
procedure UpdateScreen;
var
  event : EventRef;
begin
  if (updatepending) then
    exit;

  if (CreateEvent(nil, kEventClassFPCGraph, kEventFlush, GetCurrentEventTime(), 0, event) <> noErr) then
    exit;

  if (PostEventToQueue(MainEventQueue,event,kEventPriorityLow) <> noErr) then
    begin
      ReleaseEvent(event);
      exit;
    end;
  updatepending:=true;
end;


{ ---------------------------------------------------------------------
    Required procedures
  ---------------------------------------------------------------------}
var
  LastColor: smallint;   {Cache the last set color to improve speed}

procedure q_SetColor(color: smallint);
begin
  if color <> LastColor then
    begin
//      writeln('setting color to ',color);
      EnterCriticalSection(graphdrawing);
      case maxcolor of
        16:
          begin
            CGContextSetRGBFillColor(offscreen,colorpalette[color,1],colorpalette[color,2],colorpalette[color,3],1);
            CGContextSetRGBStrokeColor(offscreen,colorpalette[color,1],colorpalette[color,2],colorpalette[color,3],1);
          end;
        256:
          begin
            CGContextSetRGBFillColor(offscreen,colorpalette[color,1],colorpalette[color,2],colorpalette[color,3],1);
            CGContextSetRGBStrokeColor(offscreen,colorpalette[color,1],colorpalette[color,2],colorpalette[color,3],1);
          end;
        32678:
          begin
            CGContextSetRGBFillColor(offscreen,((color and $7ffff) shr 10)/31.0,((color shr 5) and 31)/31.0,(color and 31)/31.0,1);
            CGContextSetRGBStrokeColor(offscreen,((color and $7ffff) shr 10)/31.0,((color shr 5) and 31)/31.0,(color and 31)/31.0,1);
          end;
        65536:
          begin
            CGContextSetRGBFillColor(offscreen,(word(color) shr 11)/31.0,((word(color) shr 5) and 63)/63.0,(color and 31)/31.0,1);
            CGContextSetRGBStrokeColor(offscreen,(word(color) shr 11)/31.0,((word(color) shr 5) and 63)/63.0,(color and 31)/31.0,1);
          end;
        else
          runerror(218);
      end;
      LeaveCriticalSection(graphdrawing);
      lastcolor:=color;
    end
end;


procedure q_savevideostate;
begin
end;

procedure q_restorevideostate;
begin
end;


function CGRectMake(x,y, width, height: single): CGRect; inline;
begin
  CGRectMake.origin.x:=x;
  CGRectMake.origin.y:=y;
  CGRectMake.size.width:=width;
  CGRectMake.size.height:=height;
end;


Function ClipCoords (Var X,Y : smallint) : Boolean;
{ Adapt to viewport, return TRUE if still in viewport,
  false if outside viewport}

begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  ClipCoords:=Not ClipPixels;
  if ClipPixels then
    Begin
    ClipCoords:=(X < StartXViewPort) or (X > (StartXViewPort + ViewWidth));
    ClipCoords:=ClipCoords or
               ((Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)));
    ClipCoords:=Not ClipCoords;
    end;
end;


procedure q_directpixelproc(X,Y: smallint);

Var Color : Word;

begin
  case CurrentWriteMode of
    XORPut:
      begin
        { getpixel wants local/relative coordinates }
        Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
        Color := CurrentColor Xor Color;
      end;
    OrPut:
      begin
        { getpixel wants local/relative coordinates }
        Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
        Color := CurrentColor Or Color;
      end;
    AndPut:
      begin
        { getpixel wants local/relative coordinates }
        Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
        Color := CurrentColor And Color;
      end;
    NotPut:
      begin
        Color := Not CurrentColor;
      end
  else
    Color:=CurrentColor;
  end;
  q_SetColor(Color);
  EnterCriticalSection(graphdrawing);
  CGContextBeginPath(offscreen);
  CGContextMoveToPoint(offscreen,x,y);
  CGContextAddLineToPoint(offscreen,x,y);
  CGContextClosePath(offscreen);
  CGContextStrokePath(offscreen);
  UpdateScreen;
  LeaveCriticalSection(graphdrawing);
end;

procedure q_putpixelproc(X,Y: smallint; Color: Word);
begin
  if Not ClipCoords(X,Y) Then
    exit;
  q_setcolor(Color);
  EnterCriticalSection(graphdrawing);
  CGContextBeginPath(offscreen);
  CGContextMoveToPoint(offscreen,x,y);
  CGContextAddLineToPoint(offscreen,x,y);
  CGContextClosePath(offscreen);
  CGContextStrokePath(offscreen);
  UpdateScreen;
  LeaveCriticalSection(graphdrawing);
end;

function q_getpixelproc (X,Y: smallint): word;
type
  pbyte = ^byte;
var
  p: pbyte;
  rsingle, gsingle, bsingle, dist, closest: single;
  count: longint;
  red, green, blue: byte;
begin
 if not ClipCoords(X,Y) then
   exit;
 p := pbyte(CGBitmapContextGetData(offscreen));
 y:=maxy-y;
 inc(p,(y*(maxx+1)+x)*4);
 red:=p^;
 green:=(p+1)^;
 blue:=(p+2)^;
 case maxcolor of
   16, 256:
     begin
       { find closest color using least squares }
       rsingle:=red/255.0;
       gsingle:=green/255.0;
       bsingle:=blue/255.0;
       closest:=255.0;
       q_getpixelproc:=0;
       for count := 0 to maxcolor-1 do
         begin
           dist:=sqr(colorpalette[count,1]-rsingle) +
                sqr(colorpalette[count,2]-gsingle) +
                sqr(colorpalette[count,3]-bsingle);
           if (dist < closest) then
             begin
               closest:=dist;
               q_getpixelproc:=count;
             end;
         end;
       exit;
     end;
   32678:
     q_getpixelproc:=((red div 8) shl 7) or ((green div 8) shl 2) or (blue div 8);
   65536:
     q_getpixelproc:=((red div 8) shl 8) or ((green div 4) shl 3) or (blue div 8);
 end;
end;

procedure q_clrviewproc;

begin
  q_SetColor(CurrentBkColor);
  EnterCriticalSection(graphdrawing);
  CGContextFillRect(offscreen,CGRectMake(StartXViewPort,StartYViewPort,ViewWidth+1,ViewHeight+1));
  UpdateScreen;
  LeaveCriticalSection(graphdrawing);
  { reset coordinates }
  CurrentX := 0;
  CurrentY := 0;
end;

procedure q_putimageproc (X,Y: smallint; var Bitmap; BitBlt: Word);
begin
{
  With TBitMap(BitMap) do
    gl_putbox(x, y, width, height, @Data);
}
end;

procedure q_getimageproc (X1,Y1,X2,Y2: smallint; Var Bitmap);
begin
{  with TBitmap(Bitmap) do
    begin
    Width := x2 - x1 + 1;
    Height := y2 - y1 + 1;
    gl_getbox(x1,y1, x2 - x1 + 1, y2 - y1 + 1, @Data);
    end;
}
end;

{
function  q_imagesizeproc (X1,Y1,X2,Y2: smallint): longint;
begin
 q_imagesizeproc := SizeOf(TBitmap) + (x2 - x1 + 1) * (y2 - y1 + 1) * PhysicalScreen^.BytesPerPixel;

end;
}

procedure q_lineproc_intern (X1, Y1, X2, Y2 : smallint);
begin
  if (CurrentWriteMode in [OrPut,AndPut,XorPut]) then
    begin
      LineDefault(X1,Y1,X2,Y2);
      exit
    end
  else
    begin
      { Convert to global coordinates. }
      x1 := x1 + StartXViewPort;
      x2 := x2 + StartXViewPort;
      y1 := y1 + StartYViewPort;
      y2 := y2 + StartYViewPort;
      if ClipPixels then
        if LineClipped(x1,y1,x2,y2,StartXViewPort,StartYViewPort,
                       StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
           exit;
      if (CurrentWriteMode = NotPut) then
        q_SetColor(not(currentcolor))
      else
        q_SetColor(currentcolor);
    end;
  EnterCriticalSection(graphdrawing);
  CGContextBeginPath(offscreen);
  CGContextMoveToPoint(offscreen,x1,y1);
  CGContextAddLineToPoint(offscreen,x2,y2);
  CGContextClosePath(offscreen);
  CGContextStrokePath(offscreen);
  UpdateScreen;
  LeaveCriticalSection(graphdrawing);
end;


procedure q_lineproc (X1, Y1, X2, Y2 : smallint);
begin
  if (CurrentWriteMode in [OrPut,AndPut,XorPut]) or
     (lineinfo.LineStyle <> SolidLn) or
     (lineinfo.Thickness<>NormWidth) then
    begin
      LineDefault(X1,Y1,X2,Y2);
      exit
    end
  else
    begin
      { Convert to global coordinates. }
      x1 := x1 + StartXViewPort;
      x2 := x2 + StartXViewPort;
      y1 := y1 + StartYViewPort;
      y2 := y2 + StartYViewPort;
      if ClipPixels then
        if LineClipped(x1,y1,x2,y2,StartXViewPort,StartYViewPort,
                       StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
           exit;
      if (CurrentWriteMode = NotPut) then
        q_SetColor(not(currentcolor))
      else
        q_SetColor(currentcolor);
    end;
  EnterCriticalSection(graphdrawing);
  CGContextBeginPath(offscreen);
  CGContextMoveToPoint(offscreen,x1,y1);
  CGContextAddLineToPoint(offscreen,x2,y2);
  CGContextClosePath(offscreen);
  CGContextStrokePath(offscreen);
  UpdateScreen;
  LeaveCriticalSection(graphdrawing);
end;


procedure q_hlineproc (x, x2,y : smallint);
begin
  if (CurrentWriteMode in [OrPut,AndPut,XorPut]) then
    HLineDefault(X,X2,Y)
  else
    q_lineproc_intern(x,y,x2,y);
end;

procedure q_vlineproc (x,y,y2: smallint);
begin
  if (CurrentWriteMode in [OrPut,AndPut,XorPut]) then
    VLineDefault(x,y,y2)
  else
    q_lineproc_intern(x,y,x,y2);
end;

procedure q_patternlineproc (x1,x2,y: smallint);
begin
end;

procedure q_ellipseproc  (X,Y: smallint;XRadius: word;
  YRadius:word; stAngle,EndAngle: word; fp: PatternLineProc);
begin
end;

procedure q_getscanlineproc (X1,X2,Y : smallint; var data);
begin
end;

procedure q_setactivepageproc (page: word);
begin
end;

procedure q_setvisualpageproc (page: word);
begin
end;


procedure q_savestateproc;
begin
end;

procedure q_restorestateproc;
begin
end;

procedure q_setrgbpaletteproc(ColorNum, RedValue, GreenValue, BlueValue: smallint);
begin
  { vga is only 6 bits per channel, palette values go from 0 to 252 }
  colorpalette[ColorNum,1]:=RedValue * (1.0/252.0);
  colorpalette[ColorNum,2]:=GreenValue * (1.0/252.0);
  colorpalette[ColorNum,3]:=BlueValue * (1.0/252.0);
end;

procedure q_getrgbpaletteproc (ColorNum: smallint; var RedValue, GreenValue, BlueValue: smallint);
begin
  RedValue:=trunc(colorpalette[ColorNum,1]*252.0);
  GreenValue:=trunc(colorpalette[ColorNum,2]*252.0);
  BlueValue:=trunc(colorpalette[ColorNum,3]*252.0);
end;


procedure InitColors(nrColors: longint);

var
  i: smallint;
begin
  for i:=0 to nrColors-1 do
    q_setrgbpaletteproc(I,DefaultColors[i].red,
      DefaultColors[i].green,DefaultColors[i].blue)
end;

procedure q_initmodeproc;
const
  myHIViewSpec  : EventTypeSpec = (eventClass: kEventClassControl; eventKind: kEventControlDraw);
var
 windowAttrs:   WindowAttributes;
 contentRect:   Rect; 
 titleKey:      CFStringRef;
 windowTitle:   CFStringRef; 
 err:           OSStatus;
 hiviewbounds : HIRect;
 b: boolean;
begin
  windowAttrs := kWindowStandardDocumentAttributes // 1
                        or kWindowStandardHandlerAttribute 
                        or kWindowInWindowMenuAttribute
                        or kWindowCompositingAttribute
                        or kWindowLiveResizeAttribute
                        or kWindowNoUpdatesAttribute; 

  SetRect (contentRect, 0,  0,
                         MaxX+1, MaxY+1);
  
  CreateNewWindow (kDocumentWindowClass, windowAttrs,// 3
                         contentRect, myMainWindow);
  
  SetRect (contentRect, 0,  50,
                         MaxX+1, 51+MaxY);
  
  SetWindowBounds(myMainWindow,kWindowContentRgn,contentrect);
  titleKey    := CFSTR('Graph Window'); // 4
  windowTitle := CFCopyLocalizedString(titleKey, nil); // 5
  err := SetWindowTitleWithCFString (myMainWindow, windowTitle); // 6
  CFRelease (titleKey); // 7
  CFRelease (windowTitle);

  with contentRect do
    begin
      top:=0;
      left:=0;
      bottom:=MaxY+1;
      right:=MaxX+1;
    end;
    
  offscreen:=CreateBitmapContext(MaxX+1,MaxY+1);
  if (offscreen = nil) then
    begin
      _GraphResult:=grNoLoadMem;
      exit;
    end;
  CGContextSetShouldAntialias(offscreen,0);

  if (CreateHIView(myMainWindow,contentRect,graphHIView) <> noErr) then
    begin
      DisposeBitmapContext(offscreen);
      _GraphResult:=grError;
      exit;
    end;


//   HIViewFindByID( HIViewGetRoot( myMainWindow ), kHIViewWindowContentID, graphHIView );

  if InstallEventHandler (GetControlEventTarget (graphHIView),
                          NewEventHandlerUPP (@MyDrawEventHandler), 
                          { GetEventTypeCount (myHIViewSpec)} 1,
                          @myHIViewSpec, 
                          pointer(graphHIView),
                          Nil) <> noErr then
    begin
      DisposeWindow(myMainWindow);
      DisposeBitmapContext(offscreen);
      _GraphResult:=grError;
      exit;
    end;

  LastColor:=-1;
  if (maxcolor=16) or (maxcolor=256) then
    InitColors(maxcolor);

  CGContextSetLineWidth(offscreen,1.0);

  { start with a black background }
  CGContextSetRGBStrokeColor(offscreen,0.0,0.0,0.0,1);
  CGContextFillRect(offscreen,CGRectMake(0,0,MaxX+1,MaxY+1));
  HIViewSetNeedsDisplay(graphHIView, true);

  ShowWindow (myMainWindow);  

{
  write('view is active: ',HIViewIsActive(graphHIView,@b));
  writeln(', latent: ',b);
  writeln('compositing enabled: ',HIViewIsCompositingEnabled(graphHIView));
  writeln('visible before: ',HIViewIsVisible(graphHIView));
  write('drawing enabled: ',HIViewIsDrawingEnabled(graphHIView));
  writeln(', latent: ',b);
  write('view is enabled: ',HIViewIsEnabled(graphHIView,@b));
  writeln(', latent: ',b);

  err := HIViewGetBounds(graphHIView,hiviewbounds);
  writeln('err, ',err,' (',hiviewbounds.origin.x:0:2,',',hiviewbounds.origin.y:0:2,'),(',hiviewbounds.size.width:0:2,',',hiviewbounds.size.height:0:2,')');
}
end;


{************************************************************************}
{*                       General routines                               *}
{************************************************************************}

procedure q_donegraph;
begin
  If not isgraphmode then
    begin
      _graphresult := grnoinitgraph;
      exit
    end;
  RestoreVideoState;
  DisposeWindow(myMainWindow);
  DisposeBitmapContext(offscreen);
  isgraphmode := false;
end;


procedure CloseGraph;
var
  event : EventRef;
  myQueue: EventQueueRef;
begin
  if (CreateEvent(nil, kEventClassFPCGraph, kEventCloseGraph, GetCurrentEventTime(), 0, event) <> noErr) then
    begin
      _GraphResult:=grError;
      exit;
    end;

  myQueue := GetCurrentEventQueue;
  if (SetEventParameter(event, FOUR_CHAR_CODE('Src '), typeVoidPtr, sizeof(EventQueueRef), @myQueue) <> noErr) then
    begin
      ReleaseEvent(event);
      _GraphResult:=grError;
    end;

  if (PostEventToQueue(MainEventQueue,event,kEventPriorityStandard) <> noErr) then
    begin
      ReleaseEvent(event);
      _GraphResult:=grError;
      exit;
    end;
    
  if (ReceiveNextEvent(length(GraphClosedSpec),@GraphClosedSpec,kEventDurationForever,true,event) <> noErr) then
    runerror(218);
  ReleaseEvent(event);
end;


procedure SendInitGraph;
var
  event : EventRef;
  myQueue: EventQueueRef;
begin
  if (CreateEvent(nil, kEventClassFPCGraph, kEventInitGraph, GetCurrentEventTime(), 0, event) <> noErr) then
    begin
      _GraphResult:=grError;
      exit;
    end;

  myQueue := GetCurrentEventQueue;
  if (SetEventParameter(event, FOUR_CHAR_CODE('Src '), typeVoidPtr, sizeof(EventQueueRef), @myQueue) <> noErr) then
    begin
      ReleaseEvent(event);
      _GraphResult:=grError;
      exit;
    end;

  if (PostEventToQueue(MainEventQueue,event,kEventPriorityStandard) <> noErr) then
    begin
      ReleaseEvent(event);
      _GraphResult:=grError;
      exit;
    end;

  if (ReceiveNextEvent(length(GraphInitedSpec),@GraphInitedSpec,kEventDurationForever,true,event) <> noErr) then
    runerror(218);
  ReleaseEvent(event);
end;


   procedure qaddmode(modenr,xres,yres,colors: longint);
   var
     mode: TModeInfo;
   begin
     InitMode(Mode);
     With Mode do
       begin
         ModeNumber := modenr;
         ModeName := ModeNames[modenr];
         // Always pretend we are VGA.
         DriverNumber := VGA;
         // MaxX is number of pixels in X direction - 1
         MaxX := xres-1;
         // same for MaxY
         MaxY := yres-1;
         YAspect := 10000;
         XAspect := 10000;
         MaxColor := colors;
         PaletteSize := MaxColor;
         directcolor := colors>256;
         HardwarePages := 0;
         // necessary hooks ...
         DirectPutPixel := @q_DirectPixelProc;
         GetPixel       := @q_GetPixelProc;
         PutPixel       := @q_PutPixelProc;
         { May be implemented later: }
         HLine          := @q_HLineProc;
         VLine          := @q_VLineProc;
  {           GetScanLine    := @q_GetScanLineProc;}
         ClearViewPort  := @q_ClrViewProc;
         SetRGBPalette  := @q_SetRGBPaletteProc;
         GetRGBPalette  := @q_GetRGBPaletteProc;
         { These are not really implemented yet:
         PutImage       := @q_PutImageProc;
         GetImage       := @q_GetImageProc;}
  {          If you use the default getimage/putimage, you also need the default
         imagesize! (JM)
          ImageSize      := @q_ImageSizeProc; }
         { Add later maybe ?
         SetVisualPage  := SetVisualPageProc;
         SetActivePage  := SetActivePageProc; }
         Line           := @q_LineProc;
  {
         InternalEllipse:= @q_EllipseProc;
         PatternLine    := @q_PatternLineProc;
         }
         InitMode       := @SendInitGraph;
       end;
     AddMode(Mode);
   end;


  function toval(const s: string): size_t;
    var
      err: longint;
    begin
      val(s,toval,err);
      if (err<>0) then
        begin
          writeln('Error decoding mode: ',s,' ',err);
          runerror(218);
        end;
    end;


  function QueryAdapterInfo:PModeInfo;
  { This routine returns the head pointer to the list }
  { of supported graphics modes.                      }
  { Returns nil if no graphics mode supported.        }
  { This list is READ ONLY!                           }
   var
     colorstr: string;
     i, hpos, cpos : longint;
     xres, yres, colors,
     dispxres, dispyres: longint;
     dispcolors: int64;
   begin
     QueryAdapterInfo := ModeList;
     { If the mode listing already exists... }
     { simply return it, without changing    }
     { anything...                           }
     if assigned(ModeList) then
       exit;
     dispxres:=CGDisplayPixelsWide(kCGDirectMainDisplay);
     { adjust for the menu bar and window title height }
     { (the latter approximated to the same as the menu bar) }
     dispyres:=CGDisplayPixelsHigh(kCGDirectMainDisplay)-GetMBarHeight*2;
     dispcolors:=int64(1) shl CGDisplayBitsPerPixel(kCGDirectMainDisplay);
     SaveVideoState:=@q_savevideostate;
     RestoreVideoState:=@q_restorevideostate;
     for i := 1 to GLASTMODE do
       begin
         { get the mode info from the names }
         hpos:=2;
         while modenames[i][hpos]<>'x' do
           inc(hpos);
         inc(hpos);
         cpos:=hpos;
         while modenames[i][cpos]<>'x' do
           inc(cpos);
         inc(cpos);
         xres:=toval(copy(modenames[i],2,hpos-3));
         yres:=toval(copy(modenames[i],hpos,cpos-hpos-1));
         colorstr:=copy(modenames[i],cpos,255);
         if (colorstr='16') then
           colors:=16
         else if (colorstr='256') then
           colors:=256
{
         These don't work very well 
         else if (colorstr='32K') then
           colors:=32768
         else if (colorstr='64K') then
           colors:=65536
}
         else 
//           1/24/32 bit not supported
           continue;
         if (xres <= dispxres) and
            (yres <= dispyres) and
            (colors <= dispcolors) then
           qaddmode(i,xres,yres,colors);
       end;
   end;


{ ************************************************* }

function GraphEventHandler (myHandler: EventHandlerCallRef; 
                        event: EventRef; userData: pointer): OSStatus; mwpascal;
var
  source: EventQueueRef;
  newEvent: EventRef;
begin
//  writeln('in GraphEventHandler, event: ',FourCharArray(GetEventKind(event)));
  newEvent := nil;
  case GetEventKind(event) of
    kEventInitGraph:
      begin
        q_initmodeproc;
        if (GetEventParameter(event,FOUR_CHAR_CODE('Src '), typeVoidPtr, nil, sizeof(EventQueueRef), nil, @source) <> noErr) then
          runerror(218);
        if (CreateEvent(nil, kEventClassFPCGraph, kEventGraphInited, GetCurrentEventTime(), 0, newEvent) <> noErr) then
          runerror(218);
      end;
    kEventCloseGraph:
      begin
        q_donegraph;
        if (GetEventParameter(event,FOUR_CHAR_CODE('Src '), typeVoidPtr, nil, sizeof(EventQueueRef), nil, @source) <> noErr) then
          runerror(218);
        if (CreateEvent(nil, kEventClassFPCGraph, kEventGraphClosed, GetCurrentEventTime(), 0, newEvent) <> noErr) then
          runerror(218);
      end;
    kEventFlush:
      begin
        HIViewSetNeedsDisplay(graphHIView, true);
      end;
    kEventQuit:
      begin
        QuitApplicationEventLoop;
      end;
  end;
  if assigned(newEvent) then
    if PostEventToQueue(source,newEvent,kEventPriorityStandard) <> noErr then
      runerror(218);
  GraphEventHandler := noErr;
  ReleaseEvent(event);
end;


type
  pmainparas = ^tmainparas;
  tmainparas = record
    argc: cint;
    argv: ppchar;
    envp: ppchar;
  end;

procedure FPCMacOSXGraphMain(argcpara: cint; argvpara, envppara: ppchar); cdecl; external;

function wrapper(p: pointer): pointer; cdecl;
  var
    mainparas: pmainparas absolute p;
  begin 
    FPCMacOSXGraphMain(mainparas^.argc, mainparas^.argv, mainparas^.envp);
    wrapper:=nil;
    { the main program should exit }
    fpexit(1);
  end;


{ this routine runs before the rtl is initialised, so don't call any }
{ rtl routines in it                                                 }
procedure main(argcpara: cint; argvpara, envppara: ppchar); cdecl; [public];
  var
    eventRec: eventrecord;
    graphmainthread: TThreadID;
    attr: TThreadAttr;
    ret: cint;
    mainparas: tmainparas;
  begin
    if InstallEventHandler (GetApplicationEventTarget,
                            NewEventHandlerUPP (@GraphEventHandler), 
                            length(allGraphSpec),
                            @allGraphSpec, 
                            nil,
                            nil) <> noErr then
      fpexit(1);
  
    { main program has to be the first one to access the event queue, see }
    { http://lists.apple.com/archives/carbon-dev/2007/Jun/msg00612.html   }
    eventavail(0,eventRec);
    maineventqueue:=GetMainEventQueue;
    ret:=pthread_attr_init(@attr);
    if (ret<>0) then
      fpexit(1);
    ret:=pthread_attr_setdetachstate(@attr,1);
    if (ret<>0) then
      fpexit(1);
    mainparas.argc:=argcpara;
    mainparas.argv:=argvpara;
    mainparas.envp:=envppara;
    ret:=pthread_create(@graphmainthread,@attr,@wrapper,@mainparas);
    if (ret<>0) then
      fpexit(1);
    RunApplicationEventLoop;
  end;


initialization
  initcriticalsection(graphdrawing);
  InitializeGraph;
finalization
  donecriticalsection(graphdrawing);
end.
