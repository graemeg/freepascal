{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Peter Vreman

    GPM (>v1.17) mouse Interface for linux

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY;without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit gpm;

{Note: Libgpm is *the* interface for Linux text-mode programs.
       Unfortunately it isn't suitable for anything else besides a blocky
       cursor on a text mode interface. The GPM protocol suffers from serious
       defficiencies and ideally, gpm is abolished as quickly as possible.

       With lack of a good alternative, GPM deserves good support. But
       please keep this in mind while coding.}

{*****************************************************************************}
                                    interface
{*****************************************************************************}

uses
  baseUnix;

{$ifdef use_external}
{$linklib gpm}
{$linklib c}
{$endif}

{$inline on}
{$goto on}

const
  _PATH_VARRUN = '/var/run/';
  _PATH_DEV    = '/dev/';
  GPM_NODE_DIR = _PATH_VARRUN;
  GPM_NODE_DIR_MODE = 0775;
  GPM_NODE_PID  = '/var/run/gpm.pid';
  GPM_NODE_DEV  = '/dev/gpmctl';
  GPM_NODE_CTL  = GPM_NODE_DEV;
  GPM_NODE_FIFO = '/dev/gpmdata';

  GPM_B_LEFT   = 4;
  GPM_B_MIDDLE = 2;
  GPM_B_RIGHT  = 1;

type
  TGpmEtype = longint;
  TGpmMargin = longint;

const
  GPM_MOVE = 1;
  GPM_DRAG = 2;
  GPM_DOWN = 4;
  GPM_UP = 8;
  GPM_SINGLE = 16;
  GPM_DOUBLE = 32;
  GPM_TRIPLE = 64;
  GPM_MFLAG = 128;
  GPM_HARD = 256;
  GPM_ENTER = 512;
  GPM_LEAVE = 1024;

  GPM_TOP = 1;
  GPM_BOT = 2;
  GPM_LFT = 4;
  GPM_RGT = 8;

type
{$PACKRECORDS c}
     Pgpm_event=^Tgpm_event;
     Tgpm_event=packed record
          buttons : byte;
          modifiers : byte;
          vc : word;
          dx : word;
          dy : word;
          x,y : word;
          EventType : TGpmEType;
          clicks : longint;
          margin : TGpmMargin;
          wdx,wdy : word;
     end;

     Pgpmevent=Pgpm_event;
     Tgpmevent=Tgpm_event;

     TGpmHandler=function(var event:TGpmEvent;clientdata:pointer):longint;cdecl;

  const
     GPM_MAGIC = $47706D4C;

  type
     Pgpm_connect = ^TGpm_connect;
     Tgpm_connect = packed record
          eventMask : word;
          defaultMask : word;
          minMod : word;
          maxMod : word;
          pid : longint;
          vc : longint;
       end;

     Pgpmconnect=Pgpm_connect;
     Tgpmconnect=Tgpm_connect;

     Pgpm_roi=^Tgpm_roi;
     Tgpm_roi=packed record
       xmin,xmax:integer;
       ymin,ymax:integer;
       minmod,maxmod:word;
       eventmask:word;
       owned:word;
       handler:Tgpmhandler;
       clientdata:pointer;
       prev,next:Pgpm_roi;
     end;

     Pgpmroi=Pgpm_roi;
     Tgpmroi=Tgpm_roi;

{$ifdef external}
var
  gpm_flag           : longint;cvar;external;
  gpm_fd             : longint;cvar;external;
  gpm_hflag          : longint;cvar;external;
  gpm_morekeys       : Longbool;cvar;external;
  gpm_zerobased      : Longbool;cvar;external;
  gpm_visiblepointer : Longbool;cvar;external;
  gpm_mx             : longint;cvar;external;
  gpm_my             : longint;cvar;external;
  gpm_timeout        : TTimeVal;cvar;external;
  _gpm_buf           : array[0..0] of char;cvar;external;
  _gpm_arg           : ^word;cvar;external;
  gpm_handler        : TGpmHandler;cvar;external;
  gpm_data           : pointer;cvar;external;
  gpm_roi_handler    : TGpmHandler;cvar;external;
  gpm_roi_data       : pointer;cvar;external;
  gpm_roi            : PGpmRoi;cvar;external;
  gpm_current_roi    : PGpmRoi;cvar;external;
  gpm_consolefd      : longint;cvar;external;
  Gpm_HandleRoi      : TGpmHandler;cvar;external;
{$else}
var gpm_roi:Pgpm_roi;
    gpm_handler,gpm_roi_handler:Tgpmhandler;
    gpm_current_roi:Pgpm_roi;
    gpm_roi_data:pointer;
{$endif}

function Gpm_StrictSingle(EventType : longint) : boolean;
function Gpm_AnySingle(EventType : longint) : boolean;
function Gpm_StrictDouble(EventType : longint) : boolean;
function Gpm_AnyDouble(EventType : longint) : boolean;
function Gpm_StrictTriple(EventType : longint) : boolean;
function Gpm_AnyTriple(EventType : longint) : boolean;

{$ifdef use_external}
function Gpm_Open(var _para1:TGpmConnect; _para2:longint):longint;cdecl;external name 'Gpm_Open';
function Gpm_Close:longint;cdecl;external name 'Gpm_Close';
function Gpm_GetEvent(var _para1:TGpmEvent):longint;cdecl;external name 'Gpm_GetEvent';
{function Gpm_Getc(_para1:pFILE):longint;cdecl;external;
function Gpm_Getchar : longint;}
function Gpm_Repeat(millisec:longint):longint;cdecl;external name 'Gpm_Repeat';
function Gpm_FitValuesM(var x,y:longint; margin:longint):longint;cdecl;external name 'Gpm_FitValuesM';
function Gpm_FitValues(var x,y:longint):longint;cdecl;external name 'Gpm_FitValues';
{function GPM_DRAWPOINTER(ePtr : longint) : longint;}
function Gpm_PushRoi(x1:longint; y1:longint; X2:longint; Y2:longint; mask:longint; fun:TGpmHandler; xtradata:pointer):PGpmRoi;cdecl;external name 'Gpm_PushRoi';
function Gpm_PopRoi(which:PGpmRoi):PGpmRoi;cdecl;external name 'Gpm_PopRoi';
function Gpm_RaiseRoi(which:PGpmRoi; before:PGpmRoi):PGpmRoi;cdecl;external name 'Gpm_RaiseRoi';
function Gpm_LowerRoi(which:PGpmRoi; after:PGpmRoi):PGpmRoi;cdecl;external name 'Gpm_LowerRoi';
{function Gpm_Wgetch:longint;cdecl;external;
function Gpm_Getch:longint;}
function Gpm_GetLibVersion(var where:longint):pchar;cdecl;external name 'Gpm_GetLibVersion';
function Gpm_GetServerVersion(var where:longint):pchar;cdecl;external name 'Gpm_GetServerVersion';
function gpm_getsnapshot(eptr:Pgpmevent):longint;cdecl;external name 'Gpm_GetSnapshot';
function Gpm_GetSnapshot(var ePtr:TGpmEvent):longint;cdecl;external name 'Gpm_GetSnapshot';
{$else}
function gpm_open(var conn:Tgpm_connect;flag:longint):longint;
function gpm_close:longint;
function gpm_getevent(var event:Tgpm_event):longint;
{function Gpm_Getc(_para1:pFILE):longint;cdecl;external;
function Gpm_Getchar : longint;}
function gpm_repeat(millisec:longint):longint;
function gpm_fitvaluesM(var x,y:longint; margin:longint):longint;
function gpm_fitvalues(var x,y:longint):longint;inline;
function gpm_pushroi(x1:longint;y1:longint;x2:longint;y2:longint;
                     mask:longint;fun:Tgpmhandler;xtradata:pointer):Pgpm_roi;
function gpm_poproi(which:Pgpm_roi):Pgpm_roi;
function gpm_raiseroi(which:Pgpm_roi;before:Pgpm_roi):Pgpm_roi;
function gpm_lowerroi(which:Pgpm_roi;after:Pgpm_roi):Pgpm_roi;
{Should be pointer because proc accepts nil.}
function gpm_getsnapshot(eptr:Pgpmevent):longint;
{Overload for compatibility.}
function gpm_getsnapshot(var eptr:Tgpmevent):longint;inline;
{$endif}


{*****************************************************************************}
                                 implementation
{*****************************************************************************}

{$ifndef use_external}
uses  termio,sockets,strings,unix;

type  Pgpm_stst=^Tgpm_stst;
      Tgpm_stst=record
        info:Tgpmconnect;
        next:Pgpm_stst;
      end;

      Pmicetab=^Tmicetab;
      Tmicetab=record
        next:Pmicetab;
        device,protocol,options:Pchar;
      end;

      string63=string[63];

      Toptions=record
        autodetect:longint;
        mice_count:longint;
        repeater:longint;
        repeater_type:Pchar;
        run_status:longint;
        micelist:Pmicetab;
        progname,
        consolename:string63;
      end;

var options:Toptions;
    gpm_stack:Pgpm_stst;
    gpm_mx,gpm_my:longint;
    gpm_saved_winch_hook,gpm_saved_suspend_hook:sigactionrec;

const gpm_flag:boolean=false; {almost unuseful now -- where was it used for ? can
                               we remove it now ? FIXME}
      gpm_tried:boolean=false;
      gpm_hflag:boolean=false;
      gpm_fd:longint=-1;
      gpm_consolefd:longint=-1;
      gpm_zerobased:longint=0;

const GPM_DEVFS_CONSOLE='/dev/vc/0';
      GPM_OLD_CONSOLE='/dev/tty0';

      GPM_REQ_SNAPSHOT=0;
      GPM_REQ_BUTTONS=1;
      GPM_REQ_CONFIG=2;
      GPM_REQ_NOPASTE=3;
{$endif}

function Gpm_StrictSingle(EventType : longint) : boolean;
begin
  Gpm_StrictSingle:=(EventType and GPM_SINGLE<>0) and not(EventType and GPM_MFLAG<>0);
end;

function Gpm_AnySingle(EventType : longint) : boolean;
begin
  Gpm_AnySingle:=(EventType and GPM_SINGLE<>0);
end;

function Gpm_StrictDouble(EventType : longint) : boolean;
begin
  Gpm_StrictDouble:=(EventType and GPM_DOUBLE<>0) and not(EventType and GPM_MFLAG<>0);
end;

function Gpm_AnyDouble(EventType : longint) : boolean;
begin
  Gpm_AnyDouble:=(EventType and GPM_DOUBLE<>0);
end;

function Gpm_StrictTriple(EventType : longint) : boolean;
begin
  Gpm_StrictTriple:=(EventType and GPM_TRIPLE<>0) and not(EventType and GPM_MFLAG<>0);
end;

function Gpm_AnyTriple(EventType : longint) : boolean;
begin
  Gpm_AnyTriple:=(EventType and GPM_TRIPLE<>0);
end;

{$ifdef use_external}
procedure Gpm_CheckVersion;
var
  l : longint;
begin
  Gpm_GetLibVersion(l);
  if l<11700 then
   begin
     writeln('You need at least gpm 1.17');
     halt(1);
   end;
end;

{$else}

const checked_con:boolean=false;

function putdata(where:longint;const what:Tgpmconnect):boolean;
var
  res: cint;
begin
  putdata:=true;
  repeat
    res:=fpwrite(where,what,sizeof(Tgpmconnect));
  until (res<>-1) or (fpgeterrno<>ESysEINTR);
  if res<>sizeof(Tgpmconnect) then
    begin
{      gpm_report(GPM_PR_ERR,GPM_MESS_WRITE_ERR,strerror(errno));}
      putdata:=false;
    end;
end;

function gpm_get_console:string63;

var buf:stat;

begin
  {First try the devfs device, because in the next time this will be
   the preferred one. If that fails, take the old console.}

  {Check for open new console.}
  if fpstat(GPM_DEVFS_CONSOLE,buf)=0 then
    gpm_get_console:=GPM_DEVFS_CONSOLE
  {Failed, try OLD console.}
  else if fpstat(GPM_OLD_CONSOLE,buf)=0 then
    gpm_get_console:=GPM_OLD_CONSOLE
  else
    gpm_get_console:='';
end;

procedure gpm_winch_hook(signum:longint;SigInfo: PSigInfo; SigContext: PSigContext);cdecl;

var win:winsize;

begin
  if (sigactionhandler(SIG_IGN)<>gpm_saved_winch_hook.sa_handler) and
     (sigactionhandler(SIG_DFL)<>gpm_saved_winch_hook.sa_handler) then
    gpm_saved_winch_hook.sa_handler(signum,nil,nil);
  if fpioctl(gpm_consolefd,TIOCGWINSZ,@win)=-1 then
    exit;
  if (win.ws_col=0) or (win.ws_row=0) then
    begin
      win.ws_col:=80;
      win.ws_row:=25;
    end;
  gpm_mx:=win.ws_col - gpm_zerobased;
  gpm_my:=win.ws_row - gpm_zerobased;
end;

procedure gpm_suspend_hook(signum:longint;SigInfo: PSigInfo; SigContext: PSigContext);cdecl;

var conn:Tgpmconnect;
    old_sigset,new_sigset:Tsigset;
    sa:sigactionrec;
    success:boolean;

begin
  fpsigemptyset(new_sigset);
  fpsigaddset(new_sigset,SIGTSTP);
  fpsigprocmask(SIG_BLOCK,new_sigset,old_sigset);

  {Open a completely transparent gpm connection.}
  conn.eventmask:=0;
  conn.defaultMask:=$ffff;
  conn.minmod:=$ffff;
  conn.maxmod:=0;
  {cannot do this under xterm, tough}
  success:=gpm_open(conn,0)>=0;

  {take the default action, whatever it is (probably a stop :)}
  fpsigprocmask(SIG_SETMASK,@old_sigset,nil);
  fpsigaction(SIGTSTP,@gpm_saved_suspend_hook,nil);
  fpkill(fpgetpid,SIGTSTP);

  { in bardo here }

  { Reincarnation. Prepare for another death early. }
  fpsigemptyset(sa.sa_mask);
  sa.sa_handler:=@gpm_suspend_hook;
  sa.sa_flags:=SA_NOMASK;
  fpsigaction(SIGTSTP,@sa,nil);

  { Pop the gpm stack by closing the useless connection }
  { but do it only when we know we opened one.. }
  if success then
    gpm_close;
end;

function gpm_open(var conn:Tgpmconnect;flag:longint):longint;

var tty:string;
    flagstr:string[10];
    term:Pchar;
    i:cardinal;
    addr:Tunixsockaddr;
    win:Twinsize;
    n:Pgpm_stst;
    l:byte;
    p:byte; {there max 256 console ttys}
    buf:stat;
    sa:sigactionrec;
    res: cint;

label err;

begin
  tty:='';
  options.consolename:='';

{   gpm_report(GPM_PR_DEBUG,"VC: %d",flag);}

  {....................................... First of all, check xterm}

(*
  term:=fpgetenv('TERM');
  if (term<>nil) and (strcomp(term,'xterm')=0) then
    begin
      if gpm_tried then
        begin
          gpm_open:=gpm_fd; { no stack }
          exit;
        end;
      gpm_fd:=-2;
      {save old hilit tracking and enable mouse tracking}
      write(#27'[?1001s'#27'[?1000h');
      flush(output);

      gpm_flag:=true;
      gpm_open:=gpm_fd;
      exit;
    end;
*)
  {....................................... No xterm, go on}

  { check whether we know what name the console is: what's with the lib??? }
  if not checked_con then
    begin
      options.consolename:=gpm_get_console;
      checked_con:=true;
    end;

  { So I chose to use the current tty, instead of /dev/console, which
    has permission problems. (I am fool, and my console is
    readable/writeable by everybody.

    However, making this piece of code work has been a real hassle.}

  if not gpm_flag and gpm_tried then
    begin
      gpm_open:=-1;
      exit;
    end;
  gpm_tried:=true; {do or die}

  new(n);
  n^.next:=gpm_stack;
  gpm_stack:=n;

  conn.pid:=fpgetpid; { fill obvious values }
  if n^.next<>nil then
    conn.vc:=n^.next^.info.vc {inherit}
  else
    begin
      conn.vc:=0;                 { default handler }
      if (flag>0) then
        begin { forced vc number }
          conn.vc:=flag;
          str(flag,flagstr);
          tty:=options.consolename+flagstr;
        end
      else
        begin {use your current vc}
          if isatty(0)<>0 then
            tty:=ttyname(0);     { stdin }
          if (tty='') and (isatty(1)<>0) then
            tty:=ttyname(1);     { stdout }
          if (tty='') and (isatty(2)<>0) then
            tty:=ttyname(2);     { stderr }
          if (tty='') then
            begin
{               gpm_report(GPM_PR_ERR,"checking tty name failed");}
              goto err;
            end;
          conn.vc:=0;
          l:=length(tty);
          p:=1;
          while tty[l] in ['0'..'9'] do
            begin
              inc(conn.vc,p*(byte(tty[l])-byte('0')));
              p:=p*10;
              dec(l);
            end;
        end;

      if (gpm_consolefd=-1) then
        begin
          repeat
            gpm_consolefd:=fpopen(tty,O_WRONLY);
          until (gpm_consolefd<>-1) or (fpgeterrno<>ESysEINTR);
          if gpm_consolefd<0 then
            begin
{              gpm_report(GPM_PR_ERR,GPM_MESS_DOUBLE_S,tty,strerror(errno));}
              goto err;
            end;
        end;
    end;
  n^.info:=conn;

  {....................................... Get screen dimensions }

  fpioctl(gpm_consolefd, TIOCGWINSZ, @win);

  if (win.ws_col or win.ws_row)=0 then
    begin
      {Hmmmm. The mad terminal didn't return it's size :/ }
{       fprintf(stderr, "libgpm: zero screen dimension, assuming 80x25.\n");}
      win.ws_col:=80;
      win.ws_row:=25;
    end;
  gpm_mx:=win.ws_col-gpm_zerobased;
  gpm_my:=win.ws_row-gpm_zerobased;

  {....................................... Connect to the control socket}
  if not gpm_flag then
    begin
      gpm_fd:=fpsocket(AF_UNIX,SOCK_STREAM,0);
      if gpm_fd<0 then
        begin
{           gpm_report(GPM_PR_ERR,GPM_MESS_SOCKET,strerror(errno));}
          goto err;
        end;
    end;

  fillchar(addr,sizeof(addr),0);
  addr.family:=PF_UNIX;
  strcopy(addr.path, GPM_NODE_CTL);
  i:=sizeof(addr.family)+length(GPM_NODE_CTL);

  repeat
    res:=fpconnect(gpm_fd,psockaddr(@addr),i);
  until (res<>-1) or (fpgeterrno<>ESysEINTR);
  if res<0 then
    begin
{         gpm_report(GPM_PR_INFO,GPM_MESS_DOUBLE_S,GPM_NODE_CTL,strerror(errno));}
      {Well, try to open a chr device called /dev/gpmctl. This should
       be forward-compatible with a kernel server.}
      repeat
        res:=fpclose(gpm_fd); {the socket}
      until (res<>-1) or (fpgeterrno<>ESysEINTR);
      repeat
        gpm_fd:=fpopen(GPM_NODE_DEV,O_RDWR);
      until (gpm_fd<>-1) or (fpgeterrno<>ESysEINTR);
      if gpm_fd=-1 then
        begin
{              gpm_report(GPM_PR_ERR,GPM_MESS_DOUBLE_S,GPM_NODE_DEV
                                                     ,strerror(errno));}
          goto err;
        end;
      if (fpfstat(gpm_fd,buf)=-1) or (buf.st_mode and STAT_IFMT<>STAT_IFCHR) then
        goto err;
    end;
  {....................................... Put your data}
  if putdata(gpm_fd,conn) then
    begin
      { itz Wed Dec 16 23:22:16 PST 1998 use sigaction, the old
        code caused a signal loop under XEmacs }
      fpsigemptyset(sa.sa_mask);

      { And the winch (window-resize) hook .. }
      sa.sa_handler:=@gpm_winch_hook;
      sa.sa_flags:=0;
      fpsigaction(SIGWINCH,@sa,@gpm_saved_winch_hook);

      if gpm_flag then
        begin
         { Install suspend hook }
         sa.sa_handler:=sigactionhandler(SIG_IGN);
         fpsigaction(SIGTSTP,@sa,@gpm_saved_suspend_hook);

         {if signal was originally ignored, job control is not supported}
         if gpm_saved_suspend_hook.sa_handler<>sigactionhandler(SIG_IGN) then
           begin
            sa.sa_flags:=SA_NOMASK;
            sa.sa_handler:=@gpm_suspend_hook;
            fpsigaction(SIGTSTP,@sa,nil);
           end;
        end;
     end;
  gpm_open:=gpm_fd;
  exit;
  {....................................... Error: free all memory}
err:
{   gpm_report(GPM_PR_ERR,'Oh, oh, it''s an error! possibly I die! ');}
   repeat
      n:=gpm_stack^.next;
      dispose(gpm_stack);
      gpm_stack:=n;
   until gpm_stack=nil;
   if gpm_fd>=0 then
     begin
       repeat
         res:=fpclose(gpm_fd);
       until (res<>-1) or (fpgeterrno<>ESysEINTR);
     end;
   gpm_flag:=false;
   gpm_open:=-1;
end;

function gpm_close:longint;

var
  next:Pgpm_stst;
  res: cint;


begin
  gpm_tried:=false; { reset the error flag for next time }
(*
  if gpm_fd=-2 then { xterm }
    begin
      write(#27'[?1000l'#27'[?1001r');
      flush(output);
    end
  else            { linux }
*)
    begin
      if not gpm_flag then
        gpm_close:=0
      else
        begin
          next:=gpm_stack^.next;
          dispose(gpm_stack);
          gpm_stack:=next;
          if next<>nil then
            putdata(gpm_fd,next^.info);

          gpm_flag:=false;
        end;
    end;

  if gpm_fd>=0 then
    begin
      repeat
        res:=fpclose(gpm_fd);
      until (res<>-1) or (fpgeterrno<>ESysEINTR);
    end;
  gpm_fd:=-1;
  fpsigaction(SIGTSTP,@gpm_saved_suspend_hook,nil);
  fpsigaction(SIGWINCH,@gpm_saved_winch_hook,nil);
  fpclose(gpm_consolefd);
  gpm_consolefd:=-1;
  gpm_close:=0;
end;

function gpm_getevent(var event:Tgpm_event):longint;

var count:cint;

begin
  gpm_getevent:=0;
  if gpm_fd=-1 then
    exit;

  repeat
    count:=fpread(gpm_fd,event,sizeof(Tgpm_event));
  until (count<>-1) or (fpgeterrno<>ESysEINTR);
  if count<>sizeof(Tgpm_event) then
    begin
       {avoid to send the message if there is no data; sometimes it makes
        sense to poll the mouse descriptor any now an then using a
        non-blocking descriptor}
{      if (count<>-1) or (errno<>EAGAIN)
          gpm_report(GPM_PR_INFO,"Read too few bytes (%i) at %s:%d",
                        count,__FILE__,__LINE__);}
      gpm_getevent:=-1;
      exit;
    end;

  dec(event.x,gpm_zerobased);
  dec(event.y,gpm_zerobased);
  gpm_getevent:=1;
end;

function gpm_repeat(millisec:longint):longint;

var fd:longint;
    selset:Tfdset;

begin
  fd:=0;    {Default to stdin (xterm).}
  if gpm_fd>=0 then
    fd:=gpm_fd;

  fpFD_ZERO(selset);
  fpFD_SET(fd,selset);
  gpm_repeat:=fpselect(fd+1,@selset,nil,nil,millisec);
end;

function gpm_fitvaluesM(var x,y:longint;margin:longint):longint;

begin
  gpm_fitvaluesM:=0;
  if margin=-1 then
    begin
      if x<gpm_zerobased then
        x:=gpm_zerobased
      else if x>gpm_mx then
        x:=gpm_mx;
      if y<gpm_zerobased then
        y:=gpm_zerobased
      else if y>gpm_my then
        y:=gpm_my;
    end
  else
    case margin of
      GPM_TOP:
        inc(y);
      GPM_BOT:
        dec(y);
      GPM_RGT:
        dec(x);
      GPM_LFT:
        inc(x);
    end;
end;

function gpm_fitvalues(var x,y:longint):longint;inline;

begin
  gpm_fitvalues:=gpm_fitvaluesm(x,y,-1);
end;

function gpm_handle_roi(var eptr:Tgpm_event;clientdata:pointer):longint;cdecl;

var backevent:Tgpm_event;
    roi:Pgpm_roi;

begin
  roi:=gpm_current_roi;

  {If motion or press, look for the interested roi.
   Drag and release will be reported to the old roi.}

  if eptr.eventtype and (GPM_MOVE or GPM_DOWN)<>0 then
    begin
      roi:=gpm_roi;
      while roi<>nil do
        begin
          if not ((roi^.xmin>eptr.x) or (roi^.xmax<eptr.x)) and
             not ((roi^.ymin>eptr.y) or (roi^.ymax<eptr.y)) and
             not ((roi^.minmod and eptr.modifiers)<roi^.minmod) and
             not ((roi^.maxmod and eptr.modifiers)<eptr.modifiers) then
            break;
          roi:=roi^.next;
        end;
    end;

  {Now generate the leave/enter events}

  if roi<>gpm_current_roi then
    begin
      if (gpm_current_roi<>nil) and (gpm_current_roi^.eventmask and GPM_LEAVE<>0) then
        begin
          backevent.eventtype:=GPM_LEAVE;
          gpm_current_roi^.handler(backevent,gpm_current_roi^.clientdata);
        end;
      if (roi<>nil) and (roi^.eventmask and GPM_ENTER<>0) then
        begin
          backevent.eventtype:=GPM_ENTER;
          roi^.handler(backevent,roi^.clientdata);
        end;
    end;
  gpm_current_roi:=roi;

  {events not requested are discarded}
  if (roi<>nil) and (eptr.eventtype and ($0f or GPM_ENTER or GPM_LEAVE) and roi^.eventmask=0) then
    gpm_handle_roi:=0
  else
    begin
      backevent:=eptr; {copy it, so the main one is unchanged}
      if roi=nil then
        if gpm_roi_handler<>nil then
          gpm_handle_roi:=gpm_roi_handler(backevent,gpm_roi_data)
        else
          gpm_handle_roi:=0
      else
        begin
          {Ok, now report the event as it is, after modifying x and y}
          dec(backevent.x,roi^.xmin);
          dec(backevent.y,roi^.ymin);
          roi^.handler(backevent,roi^.clientdata);
        end;
    end;
end;

function gpm_pushroi(x1:longint;y1:longint;x2:longint;y2:longint;
                     mask:longint;fun:Tgpmhandler;xtradata:pointer):Pgpm_roi;

var n:Pgpm_roi;

begin
  {create a roi and push it}
  new(n);
  {use the roi handler, if still null}
  if (gpm_roi<>nil) and (gpm_handler<>nil) then
    gpm_handler:=@gpm_handle_roi;

  n^.xmin:=x1;        n^.xmax:=x2;
  n^.ymin:=y1;        n^.ymax:=y2;
  n^.minmod:=0;       n^.maxmod:=$ffff;
  n^.prev:=nil;       n^.next:=nil;
  n^.eventmask:=mask;
  n^.owned:=0;        { use dispose }
  n^.handler:=fun;
  if xtradata=nil then
    n^.clientdata:=n
  else
    n^.clientdata:=xtradata;
  gpm_pushroi:=gpm_raiseroi(n,nil);
end;

function gpm_useroi(n:Pgpm_roi):Pgpm_roi;

begin
  { use a Roi by pushing it }
  n^.prev:=nil;
  n^.next:=nil;
  n^.owned:=1;

  { use the roi handler, if still nil }
  if (gpm_roi=nil) and (gpm_handler=nil) then
    gpm_handler:=@gpm_handle_roi;

  gpm_useroi:=gpm_raiseroi(n,nil);
end;

function gpm_poproi(which:Pgpmroi):Pgpmroi;

begin
  {extract the Roi and remove it}
  if which^.prev<>nil then
    which^.prev^.next:=which^.next;
  if which^.next<>nil then
    which^.next^.prev:=which^.prev;
  if gpm_roi=which then
    gpm_roi:=which^.next;

  if which^.owned=0 then
    dispose(which);
  if gpm_current_roi=which then
        gpm_current_roi:=nil;

  gpm_poproi:=gpm_roi; {return the new top-of-stack}
end;


function gpm_raiseroi(which:Pgpmroi;before:Pgpmroi):Pgpmroi;

begin
  {raise a Roi above another, or to top-of-stack}
  if gpm_roi=nil then
    begin
      gpm_roi:=which;
      gpm_raiseroi:=which;
      exit;
    end;
  if before=nil then
    before:=gpm_roi;
  if before=which then
    begin
      gpm_raiseroi:=gpm_roi;
      exit;
    end;

  if which^.prev<>nil then
    which^.prev^.next:=which^.next;
  if which^.next<>nil then
    which^.next^.prev:=which^.prev;
  if gpm_roi=which then
    gpm_roi:=which^.next;

  which^.prev:=before^.prev;
  before^.prev:=which;
  which^.next:=before;

  if which^.prev<>nil then
    which^.prev^.next:=which
  else
    gpm_roi:=which;

  gpm_raiseroi:=gpm_roi; { return the new top-of-stack }
end;

function gpm_lowerroi(which:Pgpmroi;after:Pgpmroi):Pgpmroi;

begin
  {lower a Roi below another, or to bottom-of-stack}
  if after=nil then
    begin
      after:=gpm_roi;
      while after^.next<>nil do
        after:=after^.next;
    end;
  if after=which then
    begin
      gpm_lowerroi:=gpm_roi;
      exit;
    end;
  if which^.prev<>nil then
    which^.prev^.next:=which^.next;
  if which^.next<>nil then
    which^.next^.prev:=which^.prev;
  if gpm_roi=which then
    gpm_roi:=which^.next;

  which^.next:=after^.next;
  after^.next:=which;
  which^.prev:=after;

  if which^.next<>nil then
    which^.next^.prev:=which;

  gpm_lowerroi:=gpm_roi; {return the new top-of-stack}
end;

function gpm_getsnapshot(eptr:Pgpm_event):longint;

var conn:Tgpm_connect;
    event:Tgpm_event;
    sillyset:Tfdset;
    i:longint;

begin
  fillchar(conn,sizeof(conn),0);
  if eptr<>nil then
    conn.vc:=GPM_REQ_SNAPSHOT
  else
    begin
      conn.vc:=GPM_REQ_BUTTONS;
      eptr:=@event;
    end;
  if gpm_fd=-1 then
    begin
      gpm_getsnapshot:=-1;
      exit;
    end;
  fpFD_ZERO(sillyset);
  fpFD_SET(gpm_fd,sillyset);
  if fpselect(gpm_fd+1,@sillyset,nil,nil,0)=1 then
    gpm_getsnapshot:=0
  else
    begin
      fpwrite(gpm_fd,conn,sizeof(Tgpm_connect));

      i:=gpm_getevent(eptr^);
      if i<>1 then
        gpm_getsnapshot:=-1
      else
        begin
          gpm_getsnapshot:=eptr^.eventtype; { number of buttons }
          if eptr^.eventtype=0 then
            gpm_getsnapshot:=15;
          eptr^.eventtype:=0;
        end;
    end;
end;

function gpm_getsnapshot(var eptr:Tgpmevent):longint;inline;

begin
    gpm_getsnapshot:=gpm_getsnapshot(@eptr);
end;

{$endif}

end.
