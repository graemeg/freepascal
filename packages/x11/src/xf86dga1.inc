{ $XFree86: xc/include/extensions/xf86dga1.h,v 1.2 1999/04/17 07:05:41 dawes Exp $ }
{

Copyright (c) 1995  Jon Tombs
Copyright (c) 1995  XFree86 Inc

}

{************************************************************************

   THIS IS THE OLD DGA API AND IS OBSOLETE.  PLEASE DO NOT USE IT ANYMORE

************************************************************************}

Type
  PPcchar = ^Pcchar;

Const
  X_XF86DGAQueryVersion     = 0;
  X_XF86DGAGetVideoLL       = 1;
  X_XF86DGADirectVideo      = 2;
  X_XF86DGAGetViewPortSize  = 3;
  X_XF86DGASetViewPort      = 4;
  X_XF86DGAGetVidPage       = 5;
  X_XF86DGASetVidPage       = 6;
  X_XF86DGAInstallColormap  = 7;
  X_XF86DGAQueryDirectVideo = 8;
  X_XF86DGAViewPortChanged  = 9;

  XF86DGADirectPresent      = $0001;
  XF86DGADirectGraphics     = $0002;
  XF86DGADirectMouse        = $0004;
  XF86DGADirectKeyb         = $0008;
  XF86DGAHasColormap        = $0100;
  XF86DGADirectColormap     = $0200;

Function XF86DGAQueryVersion(
    dpy : PDisplay;
    majorVersion : Pcint;
    minorVersion : Pcint
  ) : TBool; CDecl; External libXxf86dga;

Function XF86DGAQueryExtension(
    dpy : PDisplay;
    event_base : Pcint;
    error_base : Pcint
  ) : TBool; CDecl; External libXxf86dga;

Function XF86DGAGetVideoLL(
    dpy : PDisplay;
    screen : cint;
    base_addr : Pcint;
    width : Pcint;
    bank_size : Pcint;
    ram_size : Pcint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGAGetVideo(
    dpy : PDisplay;
    screen : cint;
    base_addr : PPcchar;
    width : Pcint;
    bank_size : Pcint;
    ram_size : Pcint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGADirectVideo(
    dpy : PDisplay;
    screen : cint;
    enable : cint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGADirectVideoLL(
    dpy : PDisplay;
    screen : cint;
    enable : cint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGAGetViewPortSize(
    dpy : PDisplay;
    screen : cint;
    width : Pcint;
    height : Pcint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGASetViewPort(
    dpy : PDisplay;
    screen : cint;
    x : cint;
    y : cint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGAGetVidPage(
    dpy : PDisplay;
    screen : cint;
    vid_page : Pcint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGASetVidPage(
    dpy : PDisplay;
    screen : cint;
    vid_page : cint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGAInstallColormap(
    dpy : PDisplay;
    screen : cint;
    Colormap : TColormap
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGAForkApp(
    screen : cint
  ) : cint; CDecl; External libXxf86dga;

Function XF86DGAQueryDirectVideo(
    dpy : PDisplay;
    screen : cint;
    flags : Pcint
  ) : TStatus; CDecl; External libXxf86dga;

Function XF86DGAViewPortChanged(
    dpy : PDisplay;
    screen : cint;
    n : cint
  ) : TBool; CDecl; External libXxf86dga;
