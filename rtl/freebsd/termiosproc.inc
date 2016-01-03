{
   This file is part of the Free Pascal run time library.
   (c) 2000-2003 by Marco van de Voort
   member of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   Termios implementation for FreeBSD

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}


{******************************************************************************
                         IOCtl and Termios calls
******************************************************************************}

Function TCGetAttr(fd:cint;var tios:TermIOS):cint; {$ifdef VER2_0}inline;{$endif}
begin
  TCGETAttr:=fpIoCtl(Fd,TIOCGETA,@tios);
end;


Function TCSetAttr(fd:cint;OptAct:cint;const tios:TermIOS):cint;
var
  nr: TIoCtlRequest;
begin
  case OptAct of
  {the three constants TIOCSETA, TIOCSETAW and TIOCSETAF are
   unsigned values above $80000000, so that they give range check errors
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   on 32-bit systems 
   Solved by using TIoCtlRequest type for FpIOCtl second parameter }
   TCSANOW   : nr:=TIOCSETA;
   TCSADRAIN : nr:=TIOCSETAW;
   TCSAFLUSH : nr:=TIOCSETAF;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   on 32-bit systems }
   TCSANOW   : nr:=cint(TIOCSETA);
   TCSADRAIN : nr:=cint(TIOCSETAW);
   TCSAFLUSH : nr:=cint(TIOCSETAF);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

Function TCSendBreak(fd,duration:cint):cint; {$ifdef VER2_0}inline;{$endif}
var
    sleepytime : ttimeval;
begin
 sleepytime.tv_sec := 0;
 sleepytime.tv_usec := 400000;
 if fpioctl(fd, TIOCSBRK, nil) = -1 then
    exit(-1);
 fpselect(0, nil, nil, nil, @sleepytime);
 if fpioctl(fd, TIOCCBRK, nil) = -1 then
   exit(-1);
 TCSendBreak:=0;
end;

Function TCSetPGrp(fd,id:cint):cint; {$ifdef VER2_0}inline;{$endif}
begin
  TCSetPGrp:=fpIOCtl(fd,TIOCSPGRP,pointer(id));
end;


Function TCGetPGrp(fd:cint;var id:cint):cint; {$ifdef VER2_0}inline;{$endif}
begin
  TCGetPGrp:=fpIOCtl(fd,TIOCGPGRP,@id);
end;

Function TCDrain(fd:cint):cint; {$ifdef VER2_0}inline;{$endif}
begin
  TCDrain:=fpIOCtl(fd,TIOCDRAIN,nil); {Should set timeout to 1 first?}
end;

const 
   _POSIX_VDISABLE = $ff;

Function TCFlow(fd,act:cint):cint; {$ifdef VER2_0}inline;{$endif}
var
  term:Termios;
  c : cuchar;
  tmp : cint;
begin
    case act OF
     TCOOFF :  TCFlow:=fpIoctl(fd,TIOCSTOP,nil);
     TCOOn  :  TCFlow:=fpIOctl(Fd,TIOCStart,nil);
     TCION,
     TCIOFF :  begin
	         if tcgetattr(fd, term) = -1 then
                        exit(-1);
                 if act=TCIOFF then  
                   tmp:=VSTOP
                 else
                   tmp:=VSTART;
                 c:=term.c_cc[tmp];
                 if (c <> _POSIX_VDISABLE) and 
			(fpwrite(fd, c, sizeof(c)) = -1) then
                   exit (-1);
                TCFlow:=0;
              end;
          else
            begin
              errno := esysEINVAL;
              exit(-1);
            end;
      end;
end;

const FREAD  = 1; // marked "BSD visible"
      FWRITE = 2;

Function TCFlush(fd,qsel:cint):cint;  {$ifdef VER2_0}inline;{$endif}
var comval : cint;
begin
  case qsel of
     TCIFlush  : comval:=FREAD;
     TCOFlush  : comval:=FWRITE;
     TCIOFlush : comval:=FREAD or FWRITE;
    else
     begin
       errno:=ESysEINVAL;
       exit(-1);
     end;
   end;
  TCFlush:=fpIOCtl(fd,TIOCFLUSH,pointer(@comval));
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


Function IsATTY(var f: text):cint; {$ifdef VER2_0}inline;{$endif}
{
  Idem as previous, only now for text variables.
}
begin
  IsATTY:=IsaTTY(textrec(f).handle);
end;

