{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Peter Vreman
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This file contains the implementation of several termio(s) functions

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{******************************************************************************
                         IOCtl and Termios calls
******************************************************************************}

Function TCGetAttr(fd:cint;var tios:TermIOS):cint; {$ifdef VER2_0}inline;{$endif}
begin
  TCGETAttr:=fpIoCtl(Fd,TIOCGETA,@tios);
end;

Function TCSetAttr(fd:cint;OptAct:cint;const tios:TermIOS):cint;
var
  nr:culong;
begin
  case OptAct of
   TCSANOW   : nr:=TIOCSETA;
   TCSADRAIN : nr:=TIOCSETAW;
   TCSAFLUSH : nr:=TIOCSETAF;
  else
   begin
     fpsetErrNo(ESysEINVAL);
     TCSetAttr:=-1;
     exit;
   end;
  end;
  TCSetAttr:=fpIOCtl(fd,nr,@Tios);
end;

Procedure CFSetISpeed(var tios:TermIOS;speed:Cardinal); {$ifdef VER2_0}inline;{$endif}
begin
  tios.c_ispeed:=speed; {Probably the Bxxxx speed constants}
end;


Procedure CFSetOSpeed(var tios:TermIOS;speed:Cardinal); {$ifdef VER2_0}inline;{$endif}
begin
   tios.c_ospeed:=speed;
end;



Procedure CFMakeRaw(var tios:TermIOS);
begin
  with tios do
   begin
     c_iflag:=c_iflag and (not (IMAXBEL or IXOFF or INPCK or BRKINT or
                PARMRK or ISTRIP or INLCR or IGNCR or ICRNL or IXON or
                IGNPAR));
     c_iflag:=c_iflag OR IGNBRK;
     c_oflag:=c_oflag and (not OPOST);
     c_lflag:=c_lflag and (not (ECHO or ECHOE or ECHOK or ECHONL or ICANON or
                                ISIG or IEXTEN or NOFLSH or TOSTOP or PENDIN));
     c_cflag:=(c_cflag and (not (CSIZE or PARENB))) or (CS8 OR cread);
     c_cc[VMIN]:=1;
     c_cc[VTIME]:=0;
   end;
end;

//Function TCGetAttr(fd:cint;var tios:TermIOS):cint; cdecl; external 'c' name 'tcgetattr';
//Function TCSetAttr(fd:cint;OptAct:cint;const tios:TermIOS):cint; cdecl; external 'c' name 'tcsetattr';
//Procedure CFSetISpeed(var tios:TermIOS;speed:Cardinal); cdecl; external 'c' name 'cfsetispeed';
//Procedure CFSetOSpeed(var tios:TermIOS;speed:Cardinal); cdecl; external 'c' name 'cfsetospeed';
//Procedure CFMakeRaw(var tios:TermIOS); cdecl; external 'c' name 'cfmakeraw';

function real_tcsendbreak(fd,duration: cint): cint; cdecl; external name 'tcsendbreak';

Function TCSendBreak(fd,duration:cint):cint;{$ifdef VER2_0}inline;{$endif}
begin
  TCSendBreak:=real_tcsendbreak(fd,duration);
end;


function real_tcsetpgrp(fd: cint; pgrp: pid_t): cint; cdecl; external name 'tcsetpgrp';

Function TCSetPGrp(fd,id:cint):cint;{$ifdef VER2_0}inline;{$endif}
begin
<<<<<<< HEAD
<<<<<<< HEAD
  TCSetPGrp:=real_tcsetpgrp(fd,id);;
=======
  TCSetPGrp:=fpIOCtl(fd,TIOCSPGRP,pointer(ptrint(id)));
>>>>>>> graemeg/fixes_2_2
=======
  TCSetPGrp:=fpIOCtl(fd,TIOCSPGRP,pointer(ptrint(id)));
>>>>>>> origin/fixes_2_2
end;


Function TCGetPGrp(fd:cint;var id:cint):cint;{$ifdef VER2_0}inline;{$endif}
var
  pid: pid_t;
begin
  if isatty(fd)=0 then
    exit(-1);
  TCGetPGrp:=fpIOCtl(fd,TIOCGPGRP,@pid);
  if TCGetPGrp>=0 then
    id:=pid;
end;

function real_tcdrain(fd: cint): cint; cdecl; external name 'tcdrain';

Function TCDrain(fd:cint):cint;{$ifdef VER2_0}inline;{$endif}
begin
  TCDrain:=fpIOCtl(fd,TIOCDRAIN,nil); {Should set timeout to 1 first?}
end;


function real_tcflow(fd,act:cint): cint; cdecl; external name 'tcflow';

Function TCFlow(fd,act:cint):cint; {$ifdef VER2_0}inline;{$endif}
begin
  TCFlow:=real_tcflow(fd,act);
end;

function real_tcflush(fd,qsel: cint): cint; cdecl; external name 'tcflush';

Function TCFlush(fd,qsel:cint):cint;  {$ifdef VER2_0}inline;{$endif}
begin
<<<<<<< HEAD
<<<<<<< HEAD
  TCFlush:=real_tcflush(fd,qsel);
=======
  TCFlush:=fpIOCtl(fd,TIOCFLUSH,pointer(ptrint(qsel)));
>>>>>>> graemeg/fixes_2_2
=======
  TCFlush:=fpIOCtl(fd,TIOCFLUSH,pointer(ptrint(qsel)));
>>>>>>> origin/fixes_2_2
end;

Function IsATTY (Handle:cint):cint;
{
  Check if the filehandle described by 'handle' is a TTY (Terminal)
}
var
  t : Termios;
begin
 IsAtty:=ord(TCGetAttr(Handle,t) <> -1);
end;


Function IsATTY(var f: text):cint;  {$ifdef VER2_0}inline;{$endif}
{
  Idem as previous, only now for text variables.
}
begin
  IsATTY:=IsaTTY(textrec(f).handle);
end;

