{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Marco van de Voort

    Some generic overloads for stringfunctions in the baseunix unit.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function  FpLink (const existing : RawByteString; const newone : RawByteString): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemExistingFileName, SystemNewOneFileName: RawByteString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{$I textrec.inc}
{$I filerec.inc}

Function  FpLink (existing : AnsiString; newone : AnsiString): cInt; {$ifdef VER2_0}inline;{$endif}
>>>>>>> graemeg/cpstrnew
Begin
  SystemExistingFileName:=ToSingleByteFileSystemEncodedFileName(existing);
  SystemNewOneFileName:=ToSingleByteFileSystemEncodedFileName(newone);
  FpLink:=FpLink(pchar(SystemExistingFileName),pchar(SystemNewOneFileName));
End;

Function  FpMkfifo (const path : RawByteString; Mode : TMode): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpMkfifo:=FpMkfifo(pchar(SystemPath),mode);
End;

Function  FpChmod (const path : RawByteString; Mode : TMode): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpChmod:=FpChmod(pchar(SystemPath),mode);
End;

Function  FpChown (const path : RawByteString; owner : TUid; group : TGid): cInt;{$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpChown:=FpChown(pchar(SystemPath),owner,group);
End;

Function  FpUtime (const path : RawByteString; times : putimbuf): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpUtime:=FpUtime(pchar(SystemPath),times);
End;

{
Function  FpGetcwd (const path:RawByteString; siz:TSize):RawByteString; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpGetcwd:=RawByteString(pchar(FpGetcwd(pchar(SystemPath),siz)));
  SetCodePage(FpGetcwd,DefaultFileSystemCodePage,false);
End;
}
Function  FpGetcwd: RawByteString;

Var
  Buf : Array[0..PATH_MAX+1]  of char;
Begin
  Buf[PATH_MAX+1]:=#0;
  If FpGetcwd(@Buf[0],PATH_MAX)=Nil then
    FpGetcwd:=''
  else
    begin
      FpGetcwd:=Buf;
      SetCodePage(FpGetcwd,DefaultFileSystemCodePage,false);
    end;
End;

Function  FpExecve (const path : RawByteString; argv : ppchar; envp: ppchar): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpExecve:=FpExecve (pchar(SystemPath),argv,envp);
End;

Function  FpExecv (const path : RawByteString; argv : ppchar): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpExecv:=FpExecve (pchar(SystemPath),argv,envp);
End;


Function  FpChdir (const path : RawByteString): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpChDir:=FpChdir(pchar(SystemPath));
End;

Function  FpOpen (const path : RawByteString; flags : cInt; Mode: TMode):cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpOpen:=FpOpen(pchar(SystemPath),flags,mode);
End;


Function  FpMkdir (const path : RawByteString; Mode: TMode):cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpMkdir:=FpMkdir(pchar(SystemPath),mode);
End;

Function  FpUnlink (const path : RawByteString): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpUnlink:=FpUnlink(pchar(SystemPath));
End;

Function  FpRmdir (const path : RawByteString): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpRmdir:=FpRmdir(pchar(SystemPath));
End;

Function  FpRename (const old  : RawByteString; const newpath: RawByteString): cInt; {$ifdef VER2_0}inline;{$endif}
var
  OldSystemPath, NewSystemPath: RawByteString;
Begin
  OldSystemPath:=ToSingleByteFileSystemEncodedFileName(old);
  NewSystemPath:=ToSingleByteFileSystemEncodedFileName(newpath);
  FpRename:=FpRename(pchar(OldSystemPath),pchar(NewSystemPath));
End;

Function  FpStat (const path: RawByteString; var buf : stat): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpStat:=FpStat(pchar(SystemPath),buf);
End;

<<<<<<< HEAD
<<<<<<< HEAD
Function  fpLstat   (const path: RawByteString; Info: pstat):cint; inline;
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  fplstat:=fplstat(pchar(SystemPath), info);
=======
Function  fpLstat   (path: Ansistring; Info: pstat):cint; inline;
begin
  fplstat:=fplstat(pchar(path), info);
>>>>>>> graemeg/fixes_2_2
=======
Function  fpLstat   (path: Ansistring; Info: pstat):cint; inline;
begin
  fplstat:=fplstat(pchar(path), info);
>>>>>>> origin/fixes_2_2
end;

Function  fpLstat   (path:pchar;var Info:stat):cint; inline;

begin
  fpLstat:=fplstat(path,@info);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function  fpLstat   (const Filename: RawByteString;var Info:stat):cint; inline;
=======
=======
>>>>>>> origin/fixes_2_2
Function  fpLstat   (Filename: ansistring;var Info:stat):cint; inline;

begin
  fpLstat:=fplstat(filename,@info);
end;

Function FpAccess (pathname : AnsiString; aMode : cInt): cInt; {$ifdef VER2_0}inline;{$endif}
>>>>>>> graemeg/fixes_2_2
Begin
  fpLstat:=fplstat(filename,@info);
end;

Function FpAccess (const pathname : RawByteString; aMode : cInt): cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPathName: RawByteString;
Begin
  SystemPathName:=ToSingleByteFileSystemEncodedFileName(pathname);
  FpAccess:=FpAccess(pchar(SystemPathName),amode);
End;

Function  FPFStat(var F:Text;Var Info:stat):Boolean; {$ifdef VER2_0}inline;{$endif}
{
  Get all information on a text file, and return it in info.
}
begin
  FPFStat:=FPFstat(TextRec(F).Handle,INfo)=0;
end;

Function  FPFStat(var F:File;Var Info:stat):Boolean; {$ifdef VER2_0}inline;{$endif}
{
  Get all information on a untyped file, and return it in info.
}
begin
  FPFStat:=FPFstat(FileRec(F).Handle,Info)=0;
end;

Function FpSignal(signum:longint;Handler:signalhandler):signalhandler;
// should be moved out of generic files. Too specific.

var sa,osa : sigactionrec;

begin
     sa.sa_handler:=SigActionHandler(handler);
     FillChar(sa.sa_mask,sizeof(sa.sa_mask),#0);
     sa.sa_flags := 0;
{     if (sigintr and signum) =0 then
 {restart behaviour needs libc}
      sa.sa_flags :=sa.sa_flags or SA_RESTART;
}
     FPSigaction(signum,@sa,@osa);
     if fpgetErrNo<>0 then
       fpsignal:=NIL
     else
       fpsignal:=signalhandler(osa.sa_handler);
end;

{$ifdef FPC_USE_LIBC} // can't remember why this is the case. Might be legacy.
function xFpread(fd: cint; buf: pchar; nbytes : size_t): ssize_t; cdecl; external clib name 'read';
{$else}
function xFpread(fd: cint; buf: pchar; nbytes : size_t): ssize_t; external name 'FPC_SYSC_READ';
{$endif}

Function  FpRead           (fd : cInt;var buf; nbytes : TSize): TSsize; {$ifdef VER2_0}inline;{$endif}

begin
  FPRead:=xFpRead(fd,pchar(@buf),nbytes);
end;

Function  FpWrite          (fd : cInt;const buf; nbytes : TSize): TSsize; {$ifdef VER2_0}inline;{$endif}
begin
 FpWrite:=FpWrite(fd,pchar(@buf),nbytes);
end;

{$ifdef linux}
function  FppRead           (fd : cInt;var buf; nbytes : TSize; offset:Toff): TSsize; {$ifdef VER2_0}inline;{$endif}

begin
  FppRead:=FppRead(fd,pchar(@buf),nbytes,offset);
end;

function  FppWrite          (fd : cInt;const buf; nbytes : TSize; offset:Toff): TSsize; {$ifdef VER2_0}inline;{$endif}

begin
  FppWrite:=FppWrite(fd,pchar(@buf),nbytes,offset);
end;
{$endif}

Function  FpOpen    (path : pChar; flags : cInt):cInt; {$ifdef VER2_0}inline;{$endif}

begin
 FpOpen:=FpOpen(path,flags,438);
end;

Function  FpOpen    (const path : RawByteString; flags : cInt):cInt; {$ifdef VER2_0}inline;{$endif}
var
  SystemPath: RawByteString;
Begin
  SystemPath:=ToSingleByteFileSystemEncodedFileName(path);
  FpOpen:=FpOpen(pchar(SystemPath),flags,438);
end;

Function  FpOpen    (path : String; flags : cInt):cInt;

begin
 path:=path+#0;
 FpOpen:=FpOpen(@path[1],flags,438);
end;

Function  FpOpen    (path : String; flags : cInt; Mode: TMode):cInt;

begin
 path:=path+#0;
 FpOpen:=FpOpen(@path[1],flags,Mode);
end;

Function  FpOpendir (const dirname : RawByteString): pDir; {$ifdef VER2_0}inline;{$endif}
var
  SystemDirName: RawByteString;
Begin
  SystemDirName:=ToSingleByteFileSystemEncodedFileName(dirname);
  FpOpenDir:=FpOpenDir(pchar(SystemDirName));
End;


Function  FpOpendir (dirname : shortString): pDir; {$ifdef VER2_0}inline;{$endif}
Begin
  dirname:=dirname+#0;
  FpOpenDir:=FpOpenDir(pchar(@dirname[1]));
End;


Function  FpStat (path: String; var buf : stat): cInt;

begin
 path:=path+#0;
 FpStat:=FpStat(pchar(@path[1]),buf);
end;

Function fpDup(var oldfile,newfile:text):cint;
{
  Copies the filedescriptor oldfile to newfile, after flushing the buffer of
  oldfile.
  After which the two textfiles are, in effect, the same, except
  that they don't share the same buffer, and don't share the same
  close_on_exit flag.
}
begin
  flush(oldfile);{ We cannot share buffers, so we flush them. }
  textrec(newfile):=textrec(oldfile);
  textrec(newfile).bufptr:=@(textrec(newfile).buffer);{ No shared buffer. }
  textrec(newfile).handle:=fpDup(textrec(oldfile).handle);
  fpdup:=textrec(newfile).handle;
end;

Function fpDup(var oldfile,newfile:file):cint;
{
  Copies the filedescriptor oldfile to newfile
}
begin
  filerec(newfile):=filerec(oldfile);
  filerec(newfile).handle:=fpDup(filerec(oldfile).handle);
  fpdup:=  filerec(newfile).handle;
end;


Function FpDup2(var oldfile,newfile:text):cint;
{
  Copies the filedescriptor oldfile to newfile, after flushing the buffer of
  oldfile. It closes newfile if it was still open.
  After which the two textfiles are, in effect, the same, except
  that they don't share the same buffer, and don't share the same
  close_on_exit flag.
}
var
  tmphandle : word;
begin
  case TextRec(oldfile).mode of
    fmOutput, fmInOut, fmAppend :
      flush(oldfile);{ We cannot share buffers, so we flush them. }
  end;
  case TextRec(newfile).mode of
    fmOutput, fmInOut, fmAppend :
      flush(newfile);
  end;
  tmphandle:=textrec(newfile).handle;
  textrec(newfile):=textrec(oldfile);
  textrec(newfile).handle:=tmphandle;
  textrec(newfile).bufptr:=@(textrec(newfile).buffer);{ No shared buffer. }
  fpDup2:=fpDup2(textrec(oldfile).handle,textrec(newfile).handle);
end;

Function FpDup2(var oldfile,newfile:file):cint;
{
  Copies the filedescriptor oldfile to newfile
}
var
  tmphandle : word;
begin
  tmphandle := filerec(newfile).handle;
  filerec(newfile):=filerec(oldfile);
  filerec(newfile).handle := tmphandle;
  fpDup2:=fpDup2(filerec(oldfile).handle,filerec(newfile).handle);
end;

function  fptime    :time_t;  {$ifdef VER2_0}inline;{$endif}
var t:time_t;
begin
  fptime:=fptime(t);
end;

Function fpSelect(N:cint;readfds,writefds,exceptfds:pfdset;TimeOut:cint):cint;
{
  Select checks whether the file descriptor sets in readfs/writefs/exceptfs
  have changed.
  This function allows specification of a timeout as a longint.
}
var
  p  : PTimeVal;
  tv : TimeVal;
begin
  if TimeOut=-1 then
   p:=nil
  else
   begin
     tv.tv_Sec:=Timeout div 1000;
     tv.tv_Usec:=(Timeout mod 1000)*1000;
     p:=@tv;
   end;
  fpSelect:=fpSelect(N,Readfds,WriteFds,ExceptFds,p);
end;

Function fpSelect(var T:Text;TimeOut :PTimeval):cint;
Var
  F:TfdSet;
begin
  if textrec(t).mode=fmclosed then
   begin
     fpSetErrNo(ESysEBADF);
     exit(-1);
   end;
  FpFD_ZERO(f);
  fpFD_SET(textrec(T).handle,f);
  if textrec(T).mode=fminput then
   fpselect:=fpselect(textrec(T).handle+1,@f,nil,nil,TimeOut)
  else
   fpSelect:=fpselect(textrec(T).handle+1,nil,@f,nil,TimeOut);
end;

Function fpSelect(var T:Text;TimeOut :time_t):cint;
var
  p  : PTimeVal;
  tv : TimeVal;
begin
  if TimeOut=-1 then
   p:=nil
  else
   begin
     tv.tv_Sec:=Timeout div 1000;
     tv.tv_Usec:=(Timeout mod 1000)*1000;
     p:=@tv;
   end;
  fpSelect:=fpSelect(T,p);
end;

function  FpWaitPid (pid : TPid; Var Status : cInt; Options : cint) : TPid;

begin
  fpWaitPID:=fpWaitPID(Pid,@Status,Options);
end;

Function fpReadLink(const Name: RawByteString): RawByteString;
{
  Read a link (where it points to)
}
var
  SystemFileName : RawByteString;
  i : cint;
begin
  SetLength(fpReadLink,PATH_MAX);
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Name);
  i:=fpReadLink(pchar(SystemFileName),pchar(fpReadLink),PATH_MAX);
  if i>0 then
   begin
     SetLength(fpReadLink,i);
     SetCodePage(fpReadLink,DefaultFileSystemCodePage,false);
   end
  else
    fpReadLink:='';
end;


