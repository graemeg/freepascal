{
    This file is part of the Free Pascal run time library.

    Main OS dependant body of the system unit, loosely modelled
    after POSIX.  *BSD version (Linux version is near identical)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{*****************************************************************************
                           Directory Handling
*****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
>>>>>>> origin/cpstrnew
const
  { read/write search permission for everyone }
  MODE_MKDIR = S_IWUSR OR S_IRUSR OR
               S_IWGRP OR S_IRGRP OR
               S_IWOTH OR S_IROTH OR
               S_IXUSR OR S_IXGRP OR S_IXOTH;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure Do_MkDir(s: rawbytestring);

Begin
  If Fpmkdir(pchar(s), MODE_MKDIR)<0 Then
=======
// len is not passed to the *nix functions because the unix API doesn't 
// use length safeguards for these functions. (probably because there
// already is a length limit due to PATH_MAX)

Begin
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  If Fpmkdir(s, MODE_MKDIR)<0 Then
>>>>>>> graemeg/cpstrnew
=======
// len is not passed to the *nix functions because the unix API doesn't 
// use length safeguards for these functions. (probably because there
// already is a length limit due to PATH_MAX)

Begin
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  If Fpmkdir(s, MODE_MKDIR)<0 Then
>>>>>>> graemeg/cpstrnew
=======
// len is not passed to the *nix functions because the unix API doesn't 
// use length safeguards for these functions. (probably because there
// already is a length limit due to PATH_MAX)

Begin
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  If Fpmkdir(s, MODE_MKDIR)<0 Then
>>>>>>> graemeg/cpstrnew
=======
// len is not passed to the *nix functions because the unix API doesn't 
// use length safeguards for these functions. (probably because there
// already is a length limit due to PATH_MAX)

Begin
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  If Fpmkdir(s, MODE_MKDIR)<0 Then
>>>>>>> origin/cpstrnew
   Errno2Inoutres
End;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

Procedure Do_RmDir(s: rawbytestring);

begin
  if (s='.') then
    begin
      InOutRes := 16;
      exit;
    end;
  If Fprmdir(pchar(S))<0 Then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Procedure RmDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
Begin
  if (len=1) and (s^ = '.') then
    InOutRes := 16;
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  If Fprmdir(s)<0 Then
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
   Errno2Inoutres
End;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

Procedure do_ChDir(s: rawbytestring);

Begin
  If Fpchdir(pchar(s))<0 Then
   Errno2Inoutres;
=======
Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
Begin
=======
Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
Begin
>>>>>>> graemeg/cpstrnew
=======
Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
Begin
>>>>>>> graemeg/cpstrnew
=======
Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
Begin
>>>>>>> origin/cpstrnew
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
   exit;
  If Fpchdir(s)<0 Then
   Errno2Inoutres
  Else
   InOutRes:=0;
>>>>>>> graemeg/cpstrnew
  { file not exists is path not found under tp7 }
  if InOutRes=2 then
   InOutRes:=3;
End;

// !! for now we use getcwd, unless we are fpc_use_libc.
// !! the old code  is _still needed_ since the syscall sometimes doesn't work
// !! on special filesystems like NFS etc.
// !! In the libc versions, the alt code is already integrated in the libc code.
// !! Also significantly boosted buffersize. This will make failure of the 
// !! dos legacy api's better visibile due to cut-off path, instead of "empty"


procedure do_getdir(drivenr : byte;var dir : rawbytestring);
var
  buf          : array[0..2047] of char;
{$ifndef FPC_USE_LIBC}
  cwdinfo      : stat;
  rootinfo     : stat;
  thedir,dummy : rawbytestring;
  dirstream    : pdir;
  d            : pdirent;
  thisdir      : stat;
  tmp          : rawbytestring;
{$endif FPC_USE_LIBC}
begin
  dir:='';
 if Fpgetcwd(@buf[0],sizeof(buf))<>nil then
   begin
     dir:=buf;
     { the returned result by the OS is in the DefaultFileSystemCodePage ->
       no conversion }
     setcodepage(dir,DefaultFileSystemCodePage,false);
   end
{$ifndef FPC_USE_LIBC}
 else 
  begin
  dummy:='';

  { get root directory information }
  tmp := '/'+#0;
  if Fpstat(@tmp[1],rootinfo)<0 then
    Exit;
  repeat
    tmp := dummy+'.'+#0;
    { get current directory information }
    if Fpstat(@tmp[1],cwdinfo)<0 then
      Exit;
    tmp:=dummy+'..'+#0;
    { open directory stream }
    { try to find the current inode number of the cwd }
    dirstream:=Fpopendir(@tmp[1]);
    if dirstream=nil then
       exit;
    repeat
      thedir:='';
      d:=Fpreaddir(dirstream);
      { no more entries to read ... }
      if not assigned(d) then
        break;
      tmp:=dummy+'../'+d^.d_name + #0;
      if (Fpstat(@tmp[1],thisdir)=0) then
       begin
         { found the entry for this directory name }
         if (cwdinfo.st_dev=thisdir.st_dev) and (cwdinfo.st_ino=thisdir.st_ino) then
          begin
            { are the filenames of type '.' or '..' ? }
            { then do not set the name.               }
            if (not ((d^.d_name[0]='.') and ((d^.d_name[1]=#0) or
                    ((d^.d_name[1]='.') and (d^.d_name[2]=#0))))) then
              { d^.d_name is an array[0..x] of char -> will be assigned the
                ansi code page on conversion to ansistring -> also typecast
                '/' to ansistring rather than rawbytestring so code pages match
                (will be unconditionally set to DefaultFileSystemCodePage at
                 the end without conversion) }
              thedir:=ansistring('/')+d^.d_name;
          end;
       end;
    until (thedir<>'');
    if Fpclosedir(dirstream)<0 then
      Exit;
    dummy:=dummy+'../';
    if ((cwdinfo.st_dev=rootinfo.st_dev) and (cwdinfo.st_ino=rootinfo.st_ino)) then
      begin
        if thedir='' then
          dir:='/'
        else
          begin
            dir:=thedir;
            { try to ensure that "dir" has a refcount of 1, so that setcodepage
              doesn't have to create a deep copy }
            thedir:='';
          end;
        { the returned result by the OS is in the DefaultFileSystemCodePage ->
          no conversion }
        setcodepage(dir,DefaultFileSystemCodePage,false);
        exit;
      end;
  until false;
  end;
 {$endif}
end;

