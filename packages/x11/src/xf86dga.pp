{
   Copyright (c) 1999  XFree86 Inc
}
{ $XFree86: xc/include/extensions/xf86dga.h,v 3.20 1999/10/13 04:20:48 dawes Exp $ }

Unit xf86dga;

{$PACKRECORDS C}

Interface

Uses
  ctypes, x, xlib;

Const
  libXxf86dga='Xxf86dga';

Type
  cfloat = Single;

{$linklib Xext}

{$INCLUDE xf86dga1.inc}

Const
  X_XDGAQueryVersion           = 0;

{ 1 through 9 are in xf86dga1.pp }

{ 10 and 11 are reserved to avoid conflicts with rogue DGA extensions }

  X_XDGAQueryModes            = 12;
  X_XDGASetMode               = 13;
  X_XDGASetViewport           = 14;
  X_XDGAInstallColormap       = 15;
  X_XDGASelectInput           = 16;
  X_XDGAFillRectangle         = 17;
  X_XDGACopyArea              = 18;
  X_XDGACopyTransparentArea   = 19;
  X_XDGAGetViewportStatus     = 20;
  X_XDGASync                  = 21;
  X_XDGAOpenFramebuffer       = 22;
  X_XDGACloseFramebuffer      = 23;
  X_XDGASetClientVersion      = 24;
  X_XDGAChangePixmapMode      = 25;
  X_XDGACreateColormap        = 26;


  XDGAConcurrentAccess = $00000001;
  XDGASolidFillRect    = $00000002;
  XDGABlitRect         = $00000004;
  XDGABlitTransRect    = $00000008;
  XDGAPixmap           = $00000010;

  XDGAInterlaced       = $00010000;
  XDGADoublescan       = $00020000;

  XDGAFlipImmediate    = $00000001;
  XDGAFlipRetrace      = $00000002;

  XDGANeedRoot         = $00000001;

  XF86DGANumberEvents          = 7;

  XDGAPixmapModeLarge          = 0;
  XDGAPixmapModeSmall          = 1;

  XF86DGAClientNotLocal        = 0;
  XF86DGANoDirectVideoMode     = 1;
  XF86DGAScreenNotActive       = 2;
  XF86DGADirectNotActivated    = 3;
  XF86DGAOperationNotSupported = 4;
  XF86DGANumberErrors          = (XF86DGAOperationNotSupported + 1);

Type
  PXDGAMode = ^TXDGAMode;
  TXDGAMode = Record
    num : cint;                 { A unique identifier for the mode (num > 0) }
    name : PChar;               { name of mode given in the XF86Config }
    verticalRefresh : cfloat;
    flags : cint;               { DGA_CONCURRENT_ACCESS, etc... }
    imageWidth : cint;          { linear accessible portion (pixels) }
    imageHeight : cint;
    pixmapWidth : cint;         { Xlib accessible portion (pixels) }
    pixmapHeight : cint;        { both fields ignored if no concurrent access }
    bytesPerScanline : cint;
    byteOrder : cint;           { MSBFirst, LSBFirst }
    depth : cint;
    bitsPerPixel : cint;
    redMask : culong;
    greenMask : culong;
    blueMask : culong;
    visualClass : cshort;
    viewportWidth : cint;
    viewportHeight : cint;
    xViewportStep : cint;       { viewport position granularity }
    yViewportStep : cint;
    maxViewportX : cint;        { max viewport origin }
    maxViewportY : cint;
    viewportFlags : cint;       { types of page flipping possible }
    reserved1 : cint;
    reserved2 : cint;
  End;

  PXDGADevice = ^TXDGADevice;
  TXDGADevice = Record
    mode : TXDGAMode;
    data : Pcuchar;
    pixmap : TPixmap;
  End;

  PXDGAButtonEvent = ^TXDGAButtonEvent;
  TXDGAButtonEvent = Record
    _type : cint;
    serial : culong;
    display : PDisplay;
    screen : cint;
    time : TTime;
    state : cuint;
    button : cuint;
  End;

  PXDGAKeyEvent = ^TXDGAKeyEvent;
  TXDGAKeyEvent = Record
    _type : cint;
    serial : culong;
    display : PDisplay;
    screen : cint;
    time : TTime;
    state : cuint;
    keycode : cuint;
  End;

  PXDGAMotionEvent = ^TXDGAMotionEvent;
  TXDGAMotionEvent = Record
    _type : cint;
    serial : culong;
    display : PDisplay;
    screen : cint;
    time : TTime;
    state : cuint;
    dx : cint;
    dy : cint;
  End;

  PXDGAEvent = ^TXDGAEvent;
  TXDGAEvent = Record
    Case LongInt Of
      0 : (_type : cint);
      1 : (xbutton : TXDGAButtonEvent);
      2 : (xkey : TXDGAKeyEvent);
      3 : (xmotion : TXDGAMotionEvent);
      4 : (pad : Array[0..23] Of clong);
  End;

Function XDGAQueryExtension(
    dpy : PDisplay;
    eventBase : Pcint;
    erroBase : Pcint
  ) : TBool; CDecl; External libXxf86dga;

Function XDGAQueryVersion(
    dpy : PDisplay;
    majorVersion : Pcint;
    minorVersion : Pcint
  ) : TBool; CDecl; External libXxf86dga;

Function XDGAQueryModes(
    dpy : PDisplay;
    screen : cint;
    num : Pcint
  ) : PXDGAMode; CDecl; External libXxf86dga;

Function XDGASetMode(
    dpy : PDisplay;
    screen : cint;
    mode : cint
  ) : PXDGADevice; CDecl; External libXxf86dga;

Function XDGAOpenFramebuffer(
    dpy : PDisplay;
    screen : cint
  ) : TBool; CDecl; External libXxf86dga;

Procedure XDGACloseFramebuffer(
    dpy : PDisplay;
    screen : cint
  ); CDecl; External libXxf86dga;

Procedure XDGASetViewport(
    dpy : PDisplay;
    screen : cint;
    x : cint;
    y : cint;
    flags : cint
  ); CDecl; External libXxf86dga;

Procedure XDGAInstallColormap(
    dpy : PDisplay;
    screen : cint;
    cmap : TColormap
  ); CDecl; External libXxf86dga;

Function XDGACreateColormap(
    dpy : PDisplay;
    screen : cint;
    device : PXDGADevice;
    alloc : cint
  ) : TColormap; CDecl; External libXxf86dga;

Procedure XDGASelectInput(
    dpy : PDisplay;
    screen : cint;
    event_mask : clong
  ); CDecl; External libXxf86dga;

Procedure XDGAFillRectangle(
    dpy : PDisplay;
    screen : cint;
    x : cint;
    y : cint;
    width : cuint;
    height : cuint;
    color : culong
  ); CDecl; External libXxf86dga;

Procedure XDGACopyArea(
    dpy : PDisplay;
    screen : cint;
    srcx : cint;
    srcy : cint;
    width : cuint;
    height : cuint;
    dstx : cint;
    dsty : cint
  ); CDecl; External libXxf86dga;

Procedure XDGACopyTransparentArea(
    dpy : PDisplay;
    screen : cint;
    srcx : cint;
    srcy : cint;
    width : cuint;
    height : cuint;
    dstx : cint;
    dsty : cint;
    key : culong
  ); CDecl; External libXxf86dga;

Function XDGAGetViewportStatus(
    dpy : PDisplay;
    screen : cint
  ) : cint; CDecl; External libXxf86dga;

Procedure XDGASync(
    dpy : PDisplay;
    screen : cint
  ); CDecl; External libXxf86dga;

Function XDGASetClientVersion(
    dpy : PDisplay
  ) : TBool; CDecl; External libXxf86dga;

Procedure XDGAChangePixmapMode(
    dpy : PDisplay;
    screen : cint;
    x : Pcint;
    y : Pcint;
    mode : cint
  ); CDecl; External libXxf86dga;

Procedure XDGAKeyEventToXKeyEvent(
    dk : PXDGAKeyEvent;
    xk : PXKeyEvent
  ); CDecl; External libXxf86dga;

Implementation

End.
