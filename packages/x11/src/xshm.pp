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

function XShmQueryExtension(para1:PDisplay):TBool;cdecl;external libX11;
function XShmGetEventBase(para1:PDisplay):cint;cdecl;external libX11;
function XShmQueryVersion(para1:PDisplay; para2:Pcint; para3:Pcint; para4:PBool):TBool;cdecl;external libX11;
function XShmPixmapFormat(para1:PDisplay):cint;cdecl;external libX11;
function XShmAttach(para1:PDisplay; para2:PXShmSegmentInfo):TStatus;cdecl;external libX11;
function XShmDetach(para1:PDisplay; para2:PXShmSegmentInfo):TStatus;cdecl;external libX11;
function XShmPutImage(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXImage; para5:cint;
           para6:cint; para7:cint; para8:cint; para9:cuint; para10:cuint;
           para11:TBool):TStatus;cdecl;external libX11;
function XShmGetImage(para1:PDisplay; para2:TDrawable; para3:PXImage; para4:cint; para5:cint;
           para6:culong):TStatus;cdecl;external libX11;
function XShmCreateImage(para1:PDisplay; para2:PVisual; para3:cuint; para4:cint; para5:Pchar;
           para6:PXShmSegmentInfo; para7:cuint; para8:cuint):PXImage;cdecl;external libX11;
function XShmCreatePixmap(para1:PDisplay; para2:TDrawable; para3:Pchar; para4:PXShmSegmentInfo; para5:cuint;
           para6:cuint; para7:cuint):TPixmap;cdecl;external libX11;

implementation


end.
