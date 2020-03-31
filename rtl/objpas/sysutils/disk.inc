{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2012 by the Free Pascal development team

    Disk calls

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function GetDriveIDFromLetter(Const ADrive : PathStr) : Byte;

begin
  if length(ADrive)=0 then
    Result:=0
  else
    Result := Ord(UpCase(ADrive[1]))-64;
end;

{$push}
{$i-}
Function SetCurrentDir(Const NewDir: PathStr): Boolean;
var
  PInOutRes: ^Word;
  OrigInOutRes: Word;
begin
  { inoutres is a threadvar -> cache address }
  PInOutRes:=@InOutRes;
  OrigInOutRes:=PInOutRes^;
  PInOutRes^:=0;
  ChDir(NewDir);
  Result:=PInOutRes^=0;
  InOutRes:=OrigInOutRes;
end;


Function CreateDir (Const NewDir: PathStr): Boolean;
var
  PInOutRes: ^Word;
  OrigInOutRes: Word;
begin
  { inoutres is a threadvar -> cache address }
  PInOutRes:=@InOutRes;
  OrigInOutRes:=PInOutRes^;
  PInOutRes^:=0;
  MkDir(NewDir);
  Result:=PInOutRes^=0;
  InOutRes:=OrigInOutRes;
end;


Function RemoveDir (Const Dir: PathStr): Boolean;
var
  PInOutRes: ^Word;
  OrigInOutRes: Word;
begin
  { inoutres is a threadvar -> cache address }
  PInOutRes:=@InOutRes;
  OrigInOutRes:=PInOutRes^;
  PInOutRes^:=0;
  RmDir(Dir);
  Result:=PInOutRes^=0;
  InOutRes:=OrigInOutRes;
end;
{$pop}


function ForceDirectories(Const Dir: PathStr): Boolean;
var
  E: EInOutError;
  ADrv: PathStr;

  function DoForceDirectories(Const Dir: PathStr): Boolean;
  var
    ADir: PathStr;
    APath: PathStr;
  begin
    Result:=True;
    ADir:=ExcludeTrailingPathDelimiter(Dir);
    if (ADir='') then Exit;
    if Not DirectoryExists(ADir) then
      begin
        APath:=ExtractFilePath(ADir);
        //this can happen on Windows if user specifies Dir like \user\name/test/
        //and would, if not checked for, cause an infinite recusrsion and a stack overflow
        if (APath=ADir) then
          Result:=False
        else
          Result:=DoForceDirectories(APath);
      if Result then
        Result:=CreateDir(ADir);
      end;
  end;

  function IsUncDrive(const Drv: PathStr): Boolean;
  begin
    Result:=
      (Length(Drv)>2) and
      (Drv[1]=PathDelim) and
      (Drv[2]=PathDelim);
  end;

begin
  Result:=False;
  ADrv:=ExtractFileDrive(Dir);
  if (ADrv<>'') and
     (not DirectoryExists(ADrv))
     {$IFNDEF FORCEDIR_NO_UNC_SUPPORT} and (not IsUncDrive(ADrv)){$ENDIF} then
    Exit;
  if Dir='' then
    begin
      E:=EInOutError.Create(SCannotCreateEmptyDir);
      E.ErrorCode:=3;
      Raise E;
    end;
  Result:=DoForceDirectories(SetDirSeparators(Dir));
end;


