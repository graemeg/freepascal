{
   This file is part of the Free Pascal run time library.
   Copyright (c) 2000 by Marco van de Voort
     member of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

{$ifndef HAS_LIBC_PIPING}
Function PClose(Var F:file) : cint;
var
  pl : ^cint;
  res : cint;
  pid: cint;

begin
  fpclose(filerec(F).Handle);
{ closed our side, Now wait for the other - this appears to be needed ?? }
  pl:=@(filerec(f).userdata[2]);
  { avoid alignment error on sparc }
  move(pl^,pid,sizeof(pid));
  fpwaitpid(pid,@res,0);
  pclose:=res shr 8;
end;

Function PClose(Var F:text) :cint;
var
  pl  : ^cint;
  res : cint;
  pid: cint;

begin
  fpclose(Textrec(F).Handle);
{ closed our side, Now wait for the other - this appears to be needed ?? }
  pl:=@(textrec(f).userdata[2]);
  { avoid alignment error on sparc }
  move(pl^,pid,sizeof(pid));
  fpwaitpid(pid,@res,0);
  pclose:=res shr 8;
end;
{$ENDIF}


Function AssignPipe(var pipe_in,pipe_out:cint):cint; [public, alias : 'FPC_SYSC_ASSIGNPIPE'];
{
  Sets up a pair of file variables, which act as a pipe. The first one can
  be read from, the second one can be written to.
  If the operation was unsuccesful, linuxerror is set.
}
var
  ret  : longint;
  errn : cint;
  {$ifdef FPC_USE_LIBC}
   fdis : array[0..1] of cint;
  {$endif}
begin
{$ifndef FPC_USE_LIBC}
 ret:=intAssignPipe(pipe_in,pipe_out,errn);
 if ret=-1 Then
  fpseterrno(errn);
{$ELSE}
 fdis[0]:=pipe_in;
 fdis[1]:=pipe_out;
 ret:=pipe(fdis);
 pipe_in:=fdis[0];
 pipe_out:=fdis[1];
{$ENDIF}
 AssignPipe:=ret;
end;

{ should probably be defined in ostypes.inc for all OSes }
const
  F_RDLCK =        01;      (* Read lock *)
  F_WRLCK =        02;      (* Write lock *)
  F_UNLCK =        03;      (* Remove lock(s) *)


Function fpFlock (fd,mode : longint) : cint;
  var
    fl : flock;
    cmd : cint;
  begin
	  { initialize the flock struct to set lock on entire file }
    fillchar(fl,sizeof(fl),0);

		{ In non-blocking lock, use F_SETLK for cmd, F_SETLKW otherwise }
		if (mode and LOCK_NB)<>0 then
		  begin
			  cmd:=F_SETLK;
			  { turn off this bit }
			  mode:=mode and not(LOCK_NB);	
			end
		else
			cmd:=F_SETLKW;
	
		case mode of
		  LOCK_UN:
			  fl.l_type:=fl.l_type or F_UNLCK;
  		LOCK_SH:
			  fl.l_type:=fl.l_type or F_RDLCK;
		  LOCK_EX:
			  fl.l_type:=fl.l_type or F_WRLCK;
		  else
		    begin
			    errno:=ESysEINVAL;
			    fpFlock:=-1;
			    exit;
			  end;
		end;
	
		fpFlock:=fpFcntl(fd,cmd,fl);
	
		if (fpFlock=-1) and (errno=ESysEACCES) then
			errno:=ESysEWOULDBLOCK;
  end;

const
  SolarisTimeZoneFile = '/etc/default/init';
  SolarisTimeZoneDir = '/usr/share/lib/zoneinfo/';

{$define FPC_HAS_GETTIMEZONEFILE}  

function GetTimezoneFile:shortstring;
var
  s : shortstring;
  p : longint;
  ft : text;
begin
  GetTimezoneFile:='';
  assign(ft,SolarisTimeZoneFile);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
=======
{$ifopt I+}
  {$define OPT_I}
{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$ifopt I+}
  {$define OPT_I}
{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$ifopt I+}
  {$define OPT_I}
{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$ifopt I+}
  {$define OPT_I}
{$endif}
>>>>>>> origin/cpstrnew
{$I-}  
  reset(ft);
  if IOResult=0 then
    begin
      while not eof(ft) do
        begin
          readln(ft,s);
          p:=pos('#',s);
          if p>0 then
            s[0]:=chr(p-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          p:=pos('TZ=',s);
          if p>0 then
            begin
              GetTimeZoneFile:=SolarisTimeZoneDir+copy(s,p+3,length(s));
              close(ft);
              exit;
            end;
         end;
      close(ft);
    end;
{$pop}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	  p:=pos('TZ=',s);
          if p>0 then
            begin
	      GetTimeZoneFile:=SolarisTimeZoneDir+copy(s,p+3,length(s));
              close(ft);
              exit;
	    end;
	end;
      close(ft);	
    end;
{$ifdef OPT_I}    
  {$I+}
{$endif}  
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
end;

