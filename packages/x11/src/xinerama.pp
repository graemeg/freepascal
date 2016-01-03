unit xinerama;
interface
uses
 ctypes, xlib;
{ Converted from X11/Xinerama.h }

{$PACKRECORDS C}


type

   PXineramaScreenInfo = ^TXineramaScreenInfo;
   TXineramaScreenInfo = record
        screen_number : cint;
        x_org         : cshort;
        y_org         : cshort;
        width         : cshort;
        height        : cshort;
     end;

function XineramaQueryExtension(dpy:PDisplay; event_base:Pcint; error_base:Pcint):TBool;cdecl;external 'Xinerama';
function XineramaQueryVersion(dpy:PDisplay; major:Pcint; minor:Pcint):TStatus;cdecl;external 'Xinerama';
function XineramaIsActive(dpy:PDisplay):TBool;cdecl;external 'Xinerama';
function XineramaQueryScreens(dpy:PDisplay; number:Pcint):PXineramaScreenInfo;cdecl;external 'Xinerama';

implementation

end.
