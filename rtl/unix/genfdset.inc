{
   This file is part of the Free Pascal run time library.
   (c) 2002 by Marco van de Voort
   members of the Free Pascal development team.

   Generic POSIX signal functions draft. Based on a few constants.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

function fpFD_SET(fdno:cint;var nset : TFDSet): cint;

Begin
   if (fdno<0) or (fdno >= FD_MAXFDSET) Then
       exit(-1);
   nset[fdno shr ln2bitsinword]:=nset[(fdno) shr ln2bitsinword] OR (TFDSetEl(1) shl ((fdno) and ln2bitmask));
   fpFD_SET:=0;
End;

function fpFD_CLR(fdno:cint;var nset : TFDSet): cint;

Begin
   if (fdno<0) or (fdno >=  FD_MAXFDSET) Then
       exit(-1);
   nset[(fdno) shr ln2bitsinword]:=nset[(fdno) shr ln2bitsinword] AND TFDSetEl(NOT (culong(1) shl ((fdno) and ln2bitmask)));
   fpFD_CLR:=0;
End;

function fpFD_ZERO(out nset : TFDSet):cint;

var i :longint;

Begin
  for i:=0 to wordsinfdset-1 DO nset[i]:=0;
  fpFD_ZERO:=0;
End;

function fpfdfillset(var nset : TFDSet):cint;

var i :longint;

Begin
  for i:=0 to wordsinfdset-1 DO 
    nset[i]:=TFDSetEl(NOT 0);
  fpfdfillset:=0;
End;

function fpFD_ISSET(fdno:cint;const nset : TFDSet): cint;

Begin
   if (fdno<0) or (fdno >=  FD_MAXFDSET) Then
       exit(-1);
   if ((nset[fdno shr ln2bitsinword]) and (TFDSetEl(1) shl ((fdno) and ln2bitmask)))>0 Then
     fpFD_ISSET:=1
   else
     fpFD_ISSET:=0;
End;

