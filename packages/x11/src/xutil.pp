unit xutil;
interface
uses
  ctypes,x,xlib,keysym;

{$define MACROS}

{$ifndef os2}
  {$LinkLib c}
  {$LinkLib X11}
const
  libX11='X11';
{$else}
const
  libX11='X11';
{$endif}

{
  Automatically converted by H2Pas 0.99.15 from xutil.h
  The following command line parameters were used:
    -p
    -T
    -S
    -d
    -c
    xutil.h
}

{$PACKRECORDS C}

const
   NoValue = $0000;
   XValue = $0001;
   YValue = $0002;
   WidthValue = $0004;
   HeightValue = $0008;
   AllValues = $000F;
   XNegative = $0010;
   YNegative = $0020;
type

   PXSizeHints = ^TXSizeHints;
   TXSizeHints = record
        flags : clong;
        x, y : cint;
        width, height : cint;
        min_width, min_height : cint;
        max_width, max_height : cint;
        width_inc, height_inc : cint;
        min_aspect, max_aspect : record
             x : cint;
             y : cint;
          end;
        base_width, base_height : cint;
        win_gravity : cint;
     end;

const
   USPosition = 1 shl 0;
   USSize = 1 shl 1;
   PPosition = 1 shl 2;
   PSize = 1 shl 3;
   PMinSize = 1 shl 4;
   PMaxSize = 1 shl 5;
   PResizeInc = 1 shl 6;
   PAspect = 1 shl 7;
   PBaseSize = 1 shl 8;
   PWinGravity = 1 shl 9;
   PAllHints = PPosition or PSize or PMinSize or PMaxSize or PResizeInc or PAspect;
type

   PXWMHints = ^TXWMHints;
   TXWMHints = record
        flags : clong;
        input : TBool;
        initial_state : cint;
        icon_pixmap : TPixmap;
        icon_window : TWindow;
        icon_x, icon_y : cint;
        icon_mask : TPixmap;
        window_group : TXID;
     end;

const
   InputHint = 1 shl 0;
   StateHint = 1 shl 1;
   IconPixmapHint = 1 shl 2;
   IconWindowHint = 1 shl 3;
   IconPositionHint = 1 shl 4;
   IconMaskHint = 1 shl 5;
   WindowGroupHint = 1 shl 6;
   AllHints = InputHint or StateHint or IconPixmapHint or IconWindowHint or IconPositionHint or IconMaskHint or WindowGroupHint;
   XUrgencyHint = 1 shl 8;
   WithdrawnState = 0;
   NormalState = 1;
   IconicState = 3;
   DontCareState = 0;
   ZoomState = 2;
   InactiveState = 4;
type

   PXTextProperty = ^TXTextProperty;
   TXTextProperty = record
        value : pcuchar;
        encoding : TAtom;
        format : cint;
        nitems : culong;
     end;

const
   XNoMemory = -1;
   XLocaleNotSupported = -2;
   XConverterNotFound = -3;
type

   PXICCEncodingStyle = ^TXICCEncodingStyle;
   TXICCEncodingStyle = (XStringStyle,XCompoundTextStyle,XTextStyle,
     XStdICCTextStyle,XUTF8StringStyle);

   PPXIconSize = ^PXIconSize;
   PXIconSize = ^TXIconSize;
   TXIconSize = record
        min_width, min_height : cint;
        max_width, max_height : cint;
        width_inc, height_inc : cint;
     end;

   PXClassHint = ^TXClassHint;
   TXClassHint = record
        res_name : Pchar;
        res_class : Pchar;
     end;

type

   PXComposeStatus = ^TXComposeStatus;
   TXComposeStatus = record
        compose_ptr : TXPointer;
        chars_matched : cint;
     end;

type

   PXRegion = ^TXRegion;
   TXRegion = record
     end;
   TRegion = PXRegion;
   PRegion = ^TRegion;

const
   RectangleOut = 0;
   RectangleIn = 1;
   RectanglePart = 2;
type

   PXVisualInfo = ^TXVisualInfo;
   TXVisualInfo = record
        visual : PVisual;
        visualid : TVisualID;
        screen : cint;
        depth : cint;
        _class : cint;
        red_mask : culong;
        green_mask : culong;
        blue_mask : culong;
        colormap_size : cint;
        bits_per_rgb : cint;
     end;

const
   VisualNoMask = $0;
   VisualIDMask = $1;
   VisualScreenMask = $2;
   VisualDepthMask = $4;
   VisualClassMask = $8;
   VisualRedMaskMask = $10;
   VisualGreenMaskMask = $20;
   VisualBlueMaskMask = $40;
   VisualColormapSizeMask = $80;
   VisualBitsPerRGBMask = $100;
   VisualAllMask = $1FF;
type

   PPXStandardColormap = ^PXStandardColormap;
   PXStandardColormap = ^TXStandardColormap;
   TXStandardColormap = record
        colormap : TColormap;
        red_max : culong;
        red_mult : culong;
        green_max : culong;
        green_mult : culong;
        blue_max : culong;
        blue_mult : culong;
        base_pixel : culong;
        visualid : TVisualID;
        killid : TXID;
     end;

const
   BitmapSuccess = 0;
   BitmapOpenFailed = 1;
   BitmapFileInvalid = 2;
   BitmapNoMemory = 3;
   XCSUCCESS = 0;
   XCNOMEM = 1;
   XCNOENT = 2;
   ReleaseByFreeingColormap : TXID = TXID(1);

type
   PXContext = ^TXContext;
   TXContext = cint;

function XAllocClassHint:PXClassHint;cdecl;external libX11;
function XAllocIconSize:PXIconSize;cdecl;external libX11;
function XAllocSizeHints:PXSizeHints;cdecl;external libX11;
function XAllocStandardColormap:PXStandardColormap;cdecl;external libX11;
function XAllocWMHints:PXWMHints;cdecl;external libX11;
function XClipBox(para1:TRegion; para2:PXRectangle):cint;cdecl;external libX11;
function XCreateRegion:TRegion;cdecl;external libX11;
function XDefaultString:Pchar;cdecl;external libX11;
function XDeleteContext(para1:PDisplay; para2:TXID; para3:TXContext):cint;cdecl;external libX11;
function XDestroyRegion(para1:TRegion):cint;cdecl;external libX11;
function XEmptyRegion(para1:TRegion):cint;cdecl;external libX11;
function XEqualRegion(para1:TRegion; para2:TRegion):cint;cdecl;external libX11;
function XFindContext(para1:PDisplay; para2:TXID; para3:TXContext; para4:PXPointer):cint;cdecl;external libX11;
function XGetClassHint(para1:PDisplay; para2:TWindow; para3:PXClassHint):TStatus;cdecl;external libX11;
function XGetIconSizes(para1:PDisplay; para2:TWindow; para3:PPXIconSize; para4:Pcint):TStatus;cdecl;external libX11;
function XGetNormalHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints):TStatus;cdecl;external libX11;
function XGetRGBColormaps(para1:PDisplay; para2:TWindow; para3:PPXStandardColormap; para4:Pcint; para5:TAtom):TStatus;cdecl;external libX11;
function XGetSizeHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints; para4:TAtom):TStatus;cdecl;external libX11;
function XGetStandardColormap(para1:PDisplay; para2:TWindow; para3:PXStandardColormap; para4:TAtom):TStatus;cdecl;external libX11;
function XGetTextProperty(para1:PDisplay; para2:TWindow; para3:PXTextProperty; para4:TAtom):TStatus;cdecl;external libX11;
function XGetVisualInfo(para1:PDisplay; para2:clong; para3:PXVisualInfo; para4:Pcint):PXVisualInfo;cdecl;external libX11;
function XGetWMClientMachine(para1:PDisplay; para2:TWindow; para3:PXTextProperty):TStatus;cdecl;external libX11;
function XGetWMHints(para1:PDisplay; para2:TWindow):PXWMHints;cdecl;external libX11;
function XGetWMIconName(para1:PDisplay; para2:TWindow; para3:PXTextProperty):TStatus;cdecl;external libX11;
function XGetWMName(para1:PDisplay; para2:TWindow; para3:PXTextProperty):TStatus;cdecl;external libX11;
function XGetWMNormalHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints; para4:Pclong):TStatus;cdecl;external libX11;
function XGetWMSizeHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints; para4:Pclong; para5:TAtom):TStatus;cdecl;external libX11;
function XGetZoomHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints):TStatus;cdecl;external libX11;
function XIntersectRegion(para1:TRegion; para2:TRegion; para3:TRegion):cint;cdecl;external libX11;
procedure XConvertCase(para1:TKeySym; para2:PKeySym; para3:PKeySym);cdecl;external libX11;
function XLookupString(para1:PXKeyEvent; para2:Pchar; para3:cint; para4:PKeySym; para5:PXComposeStatus):cint;cdecl;external libX11;
function XMatchVisualInfo(para1:PDisplay; para2:cint; para3:cint; para4:cint; para5:PXVisualInfo):TStatus;cdecl;external libX11;
function XOffsetRegion(para1:TRegion; para2:cint; para3:cint):cint;cdecl;external libX11;
<<<<<<< HEAD
<<<<<<< HEAD
function XPointInRegion(para1:TRegion; para2:cint; para3:cint):TBoolResult;cdecl;external libX11;
=======
function XPointInRegion(para1:TRegion; para2:cint; para3:cint):TBool;cdecl;external libX11;
>>>>>>> graemeg/fixes_2_2
=======
function XPointInRegion(para1:TRegion; para2:cint; para3:cint):TBool;cdecl;external libX11;
>>>>>>> origin/fixes_2_2
function XPolygonRegion(para1:PXPoint; para2:cint; para3:cint):TRegion;cdecl;external libX11;
function XRectInRegion(para1:TRegion; para2:cint; para3:cint; para4:cuint; para5:cuint):cint;cdecl;external libX11;
function XSaveContext(para1:PDisplay; para2:TXID; para3:TXContext; para4:Pchar):cint;cdecl;external libX11;
function XSetClassHint(para1:PDisplay; para2:TWindow; para3:PXClassHint):cint;cdecl;external libX11;
function XSetIconSizes(para1:PDisplay; para2:TWindow; para3:PXIconSize; para4:cint):cint;cdecl;external libX11;
function XSetNormalHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints):cint;cdecl;external libX11;
procedure XSetRGBColormaps(para1:PDisplay; para2:TWindow; para3:PXStandardColormap; para4:cint; para5:TAtom);cdecl;external libX11;
function XSetSizeHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints; para4:TAtom):cint;cdecl;external libX11;
function XSetStandardProperties(para1:PDisplay; para2:TWindow; para3:Pchar; para4:Pchar; para5:TPixmap;
           para6:PPchar; para7:cint; para8:PXSizeHints):cint;cdecl;external libX11;
procedure XSetTextProperty(para1:PDisplay; para2:TWindow; para3:PXTextProperty; para4:TAtom);cdecl;external libX11;
procedure XSetWMClientMachine(para1:PDisplay; para2:TWindow; para3:PXTextProperty);cdecl;external libX11;
function XSetWMHints(para1:PDisplay; para2:TWindow; para3:PXWMHints):cint;cdecl;external libX11;
procedure XSetWMIconName(para1:PDisplay; para2:TWindow; para3:PXTextProperty);cdecl;external libX11;
procedure XSetWMName(para1:PDisplay; para2:TWindow; para3:PXTextProperty);cdecl;external libX11;
procedure XSetWMNormalHints(ADisplay:PDisplay; AWindow:TWindow; AHints:PXSizeHints);cdecl;external libX11;
procedure XSetWMProperties(ADisplay:PDisplay; AWindow:TWindow; AWindowName:PXTextProperty; AIconName:PXTextProperty; AArgv:PPchar;
            AArgc:cint; ANormalHints:PXSizeHints; AWMHints:PXWMHints; AClassHints:PXClassHint);cdecl;external libX11;
procedure XmbSetWMProperties(para1:PDisplay; para2:TWindow; para3:Pchar; para4:Pchar; para5:PPchar;
            para6:cint; para7:PXSizeHints; para8:PXWMHints; para9:PXClassHint);cdecl;external libX11;
procedure Xutf8SetWMProperties(para1:PDisplay; para2:TWindow; para3:Pchar; para4:Pchar; para5:PPchar;
            para6:cint; para7:PXSizeHints; para8:PXWMHints; para9:PXClassHint);cdecl;external libX11;
procedure XSetWMSizeHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints; para4:TAtom);cdecl;external libX11;
function XSetRegion(para1:PDisplay; para2:TGC; para3:TRegion):cint;cdecl;external libX11;
procedure XSetStandardColormap(para1:PDisplay; para2:TWindow; para3:PXStandardColormap; para4:TAtom);cdecl;external libX11;
function XSetZoomHints(para1:PDisplay; para2:TWindow; para3:PXSizeHints):cint;cdecl;external libX11;
function XShrinkRegion(para1:TRegion; para2:cint; para3:cint):cint;cdecl;external libX11;
function XStringListToTextProperty(para1:PPchar; para2:cint; para3:PXTextProperty):TStatus;cdecl;external libX11;
function XSubtractRegion(para1:TRegion; para2:TRegion; para3:TRegion):cint;cdecl;external libX11;
function XmbTextListToTextProperty(para1:PDisplay; para2:PPchar; para3:cint; para4:TXICCEncodingStyle; para5:PXTextProperty):cint;cdecl;external libX11;
function XwcTextListToTextProperty(para1:PDisplay; para2:PPWideChar; para3:cint; para4:TXICCEncodingStyle; para5:PXTextProperty):cint;cdecl;external libX11;
function Xutf8TextListToTextProperty(para1:PDisplay; para2:PPchar; para3:cint; para4:TXICCEncodingStyle; para5:PXTextProperty):cint;cdecl;external libX11;
procedure XwcFreeStringList(para1:PPWideChar);cdecl;external libX11;
function XTextPropertyToStringList(para1:PXTextProperty; para2:PPPchar; para3:Pcint):TStatus;cdecl;external libX11;
function XmbTextPropertyToTextList(para1:PDisplay; para2:PXTextProperty; para3:PPPchar; para4:Pcint):cint;cdecl;external libX11;
function XwcTextPropertyToTextList(para1:PDisplay; para2:PXTextProperty; para3:PPPWideChar; para4:Pcint):cint;cdecl;external libX11;
function Xutf8TextPropertyToTextList(para1:PDisplay; para2:PXTextProperty; para3:PPPchar; para4:Pcint):cint;cdecl;external libX11;
function XUnionRectWithRegion(para1:PXRectangle; para2:TRegion; para3:TRegion):cint;cdecl;external libX11;
function XUnionRegion(para1:TRegion; para2:TRegion; para3:TRegion):cint;cdecl;external libX11;
function XWMGeometry(para1:PDisplay; para2:cint; para3:Pchar; para4:Pchar; para5:cuint;
           para6:PXSizeHints; para7:Pcint; para8:Pcint; para9:Pcint; para10:Pcint;
           para11:Pcint):cint;cdecl;external libX11;
function XXorRegion(para1:TRegion; para2:TRegion; para3:TRegion):cint;cdecl;external libX11;

{$ifdef MACROS}
function XDestroyImage(ximage : PXImage) : cint;
function XGetPixel(ximage : PXImage; x, y : cint) : culong;
function XPutPixel(ximage : PXImage; x, y : cint; pixel : culong) : cint;
function XSubImage(ximage : PXImage; x, y : cint; width, height : cuint) : PXImage;
function XAddPixel(ximage : PXImage; value : clong) : cint;
function IsKeypadKey(keysym : TKeySym) : Boolean;
function IsPrivateKeypadKey(keysym : TKeySym) : Boolean;
function IsCursorKey(keysym : TKeySym) : Boolean;
function IsPFKey(keysym : TKeySym) : Boolean;
function IsFunctionKey(keysym : TKeySym) : Boolean;
function IsMiscFunctionKey(keysym : TKeySym) : Boolean;
function IsModifierKey(keysym : TKeySym) : Boolean;
{function XUniqueContext : TXContext;
function XStringToContext(_string : Pchar) : TXContext;}
{$endif MACROS}

implementation

{$ifdef MACROS}

function XDestroyImage(ximage : PXImage) : cint;

begin
  XDestroyImage := ximage^.f.destroy_image(ximage);
end;

function XGetPixel(ximage : PXImage; x, y : cint) : culong;
begin
   XGetPixel:=ximage^.f.get_pixel(ximage, x, y);
end;

function XPutPixel(ximage : PXImage; x, y : cint; pixel : culong) : cint;
begin
   XPutPixel:=ximage^.f.put_pixel(ximage, x, y, pixel);
end;

function XSubImage(ximage : PXImage; x, y : cint; width, height : cuint) : PXImage;
begin
   XSubImage:=ximage^.f.sub_image(ximage, x, y, width, height);
end;

function XAddPixel(ximage : PXImage; value : clong) : cint;
begin
   XAddPixel:=ximage^.f.add_pixel(ximage, value);
end;

function IsKeypadKey(keysym : TKeySym) : Boolean;
begin
   IsKeypadKey:=(keysym >= XK_KP_Space) and (keysym <= XK_KP_Equal);
end;

function IsPrivateKeypadKey(keysym : TKeySym) : Boolean;
begin
   IsPrivateKeypadKey:=(keysym >= $11000000) and (keysym <= $1100FFFF);
end;

function IsCursorKey(keysym : TKeySym) : Boolean;
begin
   IsCursorKey:=(keysym >= XK_Home) and (keysym < XK_Select);
end;

function IsPFKey(keysym : TKeySym) : Boolean;
begin
   IsPFKey:=(keysym >= XK_KP_F1) and (keysym <= XK_KP_F4);
end;

function IsFunctionKey(keysym : TKeySym) : Boolean;
begin
   IsFunctionKey:=(keysym >= XK_F1) and (keysym <= XK_F35);
end;

function IsMiscFunctionKey(keysym : TKeySym) : Boolean;
begin
   IsMiscFunctionKey:=(keysym >= XK_Select) and (keysym <= XK_Break);
end;

function IsModifierKey(keysym : TKeySym) : Boolean;
begin
  IsModifierKey := ((keysym >= XK_Shift_L) And (keysym <= XK_Hyper_R)) Or
                   (keysym = XK_Mode_switch) Or (keysym = XK_Num_Lock);
end;

{...needs xresource
function XUniqueContext : TXContext;
begin
   XUniqueContext:=TXContext(XrmUniqueQuark);
end;

function XStringToContext(_string : Pchar) : TXContext;
begin
   XStringToContext:=TXContext(XrmStringToQuark(_string));
end;}
{$endif MACROS}

end.
