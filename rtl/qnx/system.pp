{
    $Id: system.pp,v 1.1.2.2 2002/05/01 14:10:36 carl Exp $
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    QNX system unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{ no stack check in system }

{$S-}
unit System;

interface

{$define FPC_IS_SYSTEM}
{ include system-independent routine headers }

{$I systemh.inc}

{ include heap support headers }

{$I heaph.inc}

var
  argc : longint; public name 'operatingsystem_parameter_argc';
  argv : ppchar;public name 'operatingsystem_parameter_argv';
  envp : ppchar;public name 'operatingsystem_parameter_envp';
  
  
var
  Errno : longint; external name 'errno';  { declared in libc } 
  

var
  UnusedHandle:longint;
  StdInputHandle:longint;
  StdOutputHandle:longint;
  StdErrorHandle:longint;
  
{Platform specific information}
const
 LineEnding = #10;
 LFNSupport = true;
 DirectorySeparator = '/';
 DriveSeparator = '';
 PathSeparator = ':';
 FileNameCaseSensitive  = True;
  

implementation


{$I system.inc}
{$i errno.inc}          { Error numbers                   }
{$I osposixh.inc}       { include POSIX types / constants }
{$I osposix.inc}        { include POSIX system calls      }

{$i sysposix.inc}



{*****************************************************************************
                              Executable filename
*****************************************************************************}
Function FileSearch(const path:shortstring;dirlist:shortstring):shortstring;
{
  Searches for a file 'path' in the list of direcories in 'dirlist'.
  returns an empty string if not found. Wildcards are NOT allowed.
  If dirlist is empty, it is set to '.'
}
Var
  NewDir : shortstring;
  p1     : Longint;
  Info   : Stat;
  buffer : array[0..PATH_MAX+1] of char;
Begin
  Move(path[1], Buffer, Length(path));
  Buffer[Length(path)]:=#0;
  if (length(Path)>0) and (path[1]='/') and (sys_stat(pchar(@Buffer),info)=0) then
  begin
    FileSearch:=path;
    exit;
  end;
{Replace ':' with ';'}
  for p1:=1to length(dirlist) do
   if dirlist[p1]=':' then
    dirlist[p1]:=';';
{Check for WildCards}
  If (Pos('?',Path) <> 0) or (Pos('*',Path) <> 0) Then
   FileSearch:='' {No wildcards allowed in these things.}
  Else
   Begin
     Dirlist:='.;'+dirlist;{Make sure current dir is first to be searched.}
     Repeat
       p1:=Pos(';',DirList);
       If p1=0 Then
        p1:=255;
       NewDir:=Copy(DirList,1,P1 - 1);
       if NewDir[Length(NewDir)]<>'/' then
        NewDir:=NewDir+'/';
       NewDir:=NewDir+Path;
       Delete(DirList,1,p1);
       Move(NewDir[1], Buffer, Length(NewDir));
       Buffer[Length(NewDir)]:=#0;
       if sys_stat(pchar(@Buffer),Info)=0 then
        Begin
          If Pos('./',NewDir)=1 Then
           Delete(NewDir,1,2);
        {DOS strips off an initial .\}
        End
       Else
        NewDir:='';
     Until (DirList='') or (Length(NewDir) > 0);
     FileSearch:=NewDir;
   End;
End;

Function GetEnv(EnvVar:shortstring):shortstring;
{
  Searches the environment for a string with name p and
  returns a pchar to it's value.
  A pchar is used to accomodate for strings of length > 255
}
var
  ep    : ppchar;
  found : boolean;
  p1 : pchar;
Begin
  EnvVar:=EnvVar+'=';            {Else HOST will also find HOSTNAME, etc}
  ep:=envp;
  found:=false;
  if ep<>nil then
   begin
     while (not found) and (ep^<>nil) do
      begin
        if (pos(EnvVar,strpas(ep^))=1) then
         found:=true
        else
         inc(ep);
      end;
   end;
  if found then
   p1:=ep^+length(EnvVar)
  else
   p1:=nil;
  if p1 = nil then
    GetEnv := ''
  else
    GetEnv := StrPas(p1);
end;

{ this routine sets up the paramstr(0) string at startup }
procedure setupexecname;
var
  fstr: shortstring;
begin
  execpathstr := strpas(argv[0]);
  fstr:=filesearch(strpas(argv[0]), getenv('PATH'));
  if fstr<>'' then
    execpathstr:=fstr;
end;



{*****************************************************************************
                              Heap Management
*****************************************************************************}

function malloc(size: size_t): pointer; cdecl; external name 'malloc';
{ IMPORTANT SOLARIS PORT NOTE: mmap() cannot be used, since ANONYMOUS      }
{ requests are only available starting from Solaris 8. sbrk() cannot       }
{ be used either since C libraries  linked in with the runtime library may }
{ use malloc(), and the man pages of Solaris indicate that mixing both     }
{ sbrk() and malloc() is a no-no.                                          }
function Sbrk(size : longint):longint;
var ptr : pointer;
begin
  ptr := malloc(size_t(size));
  if ptr = nil then
    sbrk := -1
  else
    begin
      sbrk := longint(ptr);
      errno := 0;
    end;
end;


{ include standard heap management }
{$I heap.inc}


{****************************************************************************
                        Low level File Routines
       All these functions can set InOutRes on errors
 ****************************************************************************}



function do_isdevice(handle:longint):boolean;
begin
  do_isdevice:= (handle=StdInputHandle) or
                (handle=StdOutputHandle) or
                (handle=StdErrorHandle);
end;


{*****************************************************************************
                           UnTyped File Handling
*****************************************************************************}

{$i file.inc}

{*****************************************************************************
                           Typed File Handling
*****************************************************************************}

{$i typefile.inc}

{*****************************************************************************
                           Text File Handling
*****************************************************************************}
{$DEFINE SHORT_LINEBREAK}
{ DEFINE EXTENDED_EOF}

{$i text.inc}

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}
var
  stacklength : longint;external name '__stklen';

begin
  { setup lowest value of stack pointer }
  StackBottom := SPtr - StackLength;
  InitHeap;
{ Set up signals handlers }
  InstallSignals;
{ Setup heap }
  InitExceptions;
{ Arguments }
  SetupCmdLine;
{ Setup IO }
  StdInputHandle:=0;
  StdOutputHandle:=1;
  StdErrorHandle:=2;

  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
  
{ Reset IO Error }
  InOutRes:=0;
  setupexecname;
end.


{
 $Log: system.pp,v $
 Revision 1.1.2.2  2002/05/01 14:10:36  carl
 * Correct structures for stat and dirent
 * correct some compilation problems
 * change types according to 80x86 version

 Revision 1.1.2.1  2001/12/20 02:55:01  carl
 + QNX versions (still untested)


}
