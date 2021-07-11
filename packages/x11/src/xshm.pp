unit xshm;
interface
uses
  ctypes,x,xlib;

{$ifndef os2}
  {$LinkLib c}
  {$LinkLib X11}
  {$LinkLib Xext}
const
  libX11='X11';
{$else}
const
  libX11='X11';
{$endif}

{
  Automatically converted by H2Pas 0.99.15 from xshm.h
  The following command line parameters were used:
    -p
    -T
    -S
    -d
    -c
    xshm.h
}

{$PACKRECORDS C}


const
   X_ShmQueryVersion = 0;
   X_ShmAttach = 1;
   X_ShmDetach = 2;
   X_ShmPutImage = 3;
   X_ShmGetImage = 4;
   X_ShmCreatePixmap = 5;
   ShmCompletion = 0;
   ShmNumberEvents = ShmCompletion + 1;
   BadShmSeg = 0;
   ShmNumberErrors = BadShmSeg + 1;
type

   PShmSeg = ^TShmSeg;
   TShmSeg = culong;

   PXShmCompletionEvent = ^TXShmCompletionEvent;
   TXShmCompletionEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        drawable : TDrawable;
        major_code : cint;
        minor_code : cint;
        shmseg : TShmSeg;
        offset : culong;
     end;

   PXShmSegmentInfo = ^TXShmSegmentInfo;
   TXShmSegmentInfo = record
        shmseg : TShmSeg;
        shmid : cint;
        shmaddr : Pchar;
        readOnly : TBool;
     end;

function XShmQueryExtension(dpy:PDisplay):TBoolResult;cdecl;external libX11;
function XShmGetEventBase(dpy:PDisplay):cint;cdecl;external libX11;
function XShmQueryVersion(dpy:PDisplay;majorVersion,minorVersion:Pcint;sharedPixmaps:PBool):TBoolResult;cdecl;external libX11;
function XShmPixmapFormat(dpy:PDisplay):cint;cdecl;external libX11;
function XShmAttach(dpy:PDisplay;shminfo:PXShmSegmentInfo):TStatus;cdecl;external libX11;
function XShmDetach(dpy:PDisplay;shminfo:PXShmSegmentInfo):TStatus;cdecl;external libX11;
function XShmPutImage(dpy:PDisplay;d:TDrawable;gc:TGC;image:PXImage;
           src_x,src_y,dst_x,dst_y:cint;src_width,src_height:cuint;
           send_event:TBool):TStatus;cdecl;external libX11;
function XShmGetImage(dpy:PDisplay;d:TDrawable;image:PXImage;x,y:cint;
           plane_mask:culong):TStatus;cdecl;external libX11;
function XShmCreateImage(dpy:PDisplay;visual:PVisual;depth:cuint;format:cint;data:Pchar;
           shminfo:PXShmSegmentInfo;width,height:cuint):PXImage;cdecl;external libX11;
function XShmCreatePixmap(dpy:PDisplay;d:TDrawable;data:Pchar;shminfo:PXShmSegmentInfo;
           width,height,depth:cuint):TPixmap;cdecl;external libX11;

{ overloaded functions to handle TBool parameters as actual booleans }
function XShmPutImage(dpy:PDisplay;d:TDrawable;gc:TGC;image:PXImage;
           src_x,src_y,dst_x,dst_y:cint;src_width,src_height:cuint;
           send_event:Boolean):TStatus;

implementation

function XShmPutImage(dpy:PDisplay;d:TDrawable;gc:TGC;image:PXImage;
           src_x,src_y,dst_x,dst_y:cint;src_width,src_height:cuint;
           send_event:Boolean):TStatus;
begin
  XShmPutImage := XShmPutImage(dpy,d,gc,image,src_x,src_y,dst_x,dst_y,src_width,src_height,Ord(send_event));
end;

end.
