{
    *********************************************************************
    Copyright (C) 1997, 1998 Gertjan Schouten

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

    System Utilities For Free Pascal
}

function ChangeFileExt(const FileName, Extension: PathStr): PathStr;
var
  i : longint;
  EndSep : Set of Char;
  SOF : Boolean;
  
begin
  i := Length(FileName);
  EndSep:=AllowDirectorySeparators+AllowDriveSeparators+[ExtensionSeparator];
  while (I > 0) and not(FileName[I] in EndSep) do
    Dec(I);
  if (I = 0) or (FileName[I] <> ExtensionSeparator) then
    I := Length(FileName)+1
  else
    begin
	SOF:=(I=1) or (FileName[i-1] in AllowDirectorySeparators);
  	if (SOF) and not FirstDotAtFileNameStartIsExtension then
	  I:=Length(FileName)+1;
	end;  
  Result := Copy(FileName, 1, I - 1) + Extension;
end;

function ExtractFilePath(const FileName: PathStr): PathStr;
var
  i : longint;
  EndSep : Set of Char;
begin
  i := Length(FileName);
  EndSep:=AllowDirectorySeparators+AllowDriveSeparators;
  while (i > 0) and not CharInSet(FileName[i],EndSep) do
    Dec(i);
  If I>0 then
    Result := Copy(FileName, 1, i)
  else
    Result:='';
end;

function ExtractFileDir(const FileName: PathStr): PathStr;
var
  i : longint;
  EndSep : Set of Char;
begin
  I := Length(FileName);
  EndSep:=AllowDirectorySeparators+AllowDriveSeparators;
  while (I > 0) and not CharInSet(FileName[I],EndSep) do
    Dec(I);
  if (I > 1) and CharInSet(FileName[I],AllowDirectorySeparators) and
     not CharInSet(FileName[I - 1],EndSep) then
    Dec(I);
  Result := Copy(FileName, 1, I);
end;

function ExtractFileDrive(const FileName: PathStr): PathStr;
var
  i,l: longint;
begin
  Result := '';
  l:=Length(FileName);
  if (l<2) then
    exit;
{$IFDEF HASAMIGA}
  i:=Pos(DriveSeparator,FileName);
  if (i > 0) then Result:=Copy(FileName,1,i); 
{$ELSE}
  If CharInSet(FileName[2],AllowDriveSeparators) then
    result:=Copy(FileName,1,2)
  else if CharInSet(FileName[1],AllowDirectorySeparators) and
          CharInSet(FileName[2],AllowDirectorySeparators) then
    begin
      i := 2;
      
      { skip share }
      While (i<l) and Not CharInSet(Filename[i+1],AllowDirectorySeparators) do
        inc(i);
      inc(i);

      While (i<l) and Not CharInSet(Filename[i+1],AllowDirectorySeparators) do
        inc(i);
      Result:=Copy(FileName,1,i);
    end;
{$ENDIF}
end;

function ExtractFileName(const FileName: PathStr): PathStr;
var
  i : longint;
  EndSep : Set of Char;
begin
  I := Length(FileName);
  EndSep:=AllowDirectorySeparators+AllowDriveSeparators;
  while (I > 0) and not CharInSet(FileName[I],EndSep) do
    Dec(I);
  Result := Copy(FileName, I + 1, MaxInt);
end;

function ExtractFileExt(const FileName: PathStr): PathStr;
var
  i : longint;
  EndSep : Set of Char;
  SOF : Boolean; // Dot at Start of filename ?
  
begin
  Result:='';
  I := Length(FileName);
  EndSep:=AllowDirectorySeparators+AllowDriveSeparators+[ExtensionSeparator];
  while (I > 0) and not CharInSet(FileName[I],EndSep) do
    Dec(I);
  if (I > 0) and (FileName[I] = ExtensionSeparator) then
    begin
	SOF:=(I=1) or (FileName[i-1] in AllowDirectorySeparators);
	if (Not SOF) or FirstDotAtFileNameStartIsExtension then
	  Result := Copy(FileName, I, MaxInt);
	end  
  else
    Result := '';
end;

{$ifndef HASEXTRACTSHORTPATHNAME}
function ExtractShortPathName(Const FileName : PathStr) : PathStr;
{$if defined(MSWINDOWS) and not defined(SYSUTILSUNICODE)}
var
  TempFile, TempResult: UnicodeString;
{$endif}
begin
{$ifdef MSWINDOWS}
{$if not defined(SYSUTILSUNICODE)}
  TempFile:=FileName;
  SetLength(TempResult,Max_Path);
  SetLength(TempResult,GetShortPathNameW(PWideChar(TempFile), PWideChar(TempResult),Length(TempResult)));
  widestringmanager.Unicode2AnsiMoveProc(PWideChar(TempResult),Result,DefaultRTLFileSystemCodePage,Length(TempResult));
{$else not SYSUTILSUNICODE}
  SetLength(Result,Max_Path);
  SetLength(Result,GetShortPathNameW(PWideChar(FileName), PWideChar(Result),Length(Result)));
{$endif not SYSUTILSUNICODE}
{$else MSWindows}
  Result:=FileName;
{$endif MSWindows}
end;
{$endif HASEXTRACTSHORTPATHNAME}

{$DEFINE FPC_FEXPAND_SYSUTILS}
{$I fexpand.inc}

function ExpandFileName (Const FileName : PathStr): PathStr;

Var S : PathStr;

Begin
 S:=FileName;
 DoDirSeparators(S);
 Result:=Fexpand(S);
end;

function ExpandFileName (Const FileName, BasePath : PathStr): PathStr;

Var S : PathStr;

Begin
 S:=FileName;
 DoDirSeparators(S);
 Result:=Fexpand(S,BasePath);
end;

{$ifndef HASEXPANDUNCFILENAME}
function ExpandUNCFileName (Const FileName : PathStr): PathStr;
begin
  Result:=ExpandFileName (FileName);
  //!! Here should follow code to replace the drive: part with UNC...
end;
{$endif HASEXPANDUNCFILENAME}


function ExpandFileNameCase (const FileName: PathStr; out MatchFound: TFilenameCaseMatch): PathStr;
var
{$ifdef SYSUTILSUNICODE}
  SR: TUnicodeSearchRec;
{$else SYSUTILSUNICODE}
  SR: TRawByteSearchRec;
{$endif SYSUTILSUNICODE}
  ItemsFound: byte;
  FoundPath: PathStr;
  RestPos: SizeUInt;
  Root: PathStr;

  procedure TryCase (const Base, Rest: PathStr);
  var
{$ifdef SYSUTILSUNICODE}
    SR: TUnicodeSearchRec;
{$else SYSUTILSUNICODE}
    SR: TRawByteSearchRec;
{$endif SYSUTILSUNICODE}
    RC: longint;
    NextDirPos: SizeUInt;
    NextPart: PathStr;
    NextRest: PathStr;
    SearchBase: PathStr;
  begin
    NextDirPos := 1;
    while (NextDirPos <= Length (Rest)) and
                       not CharInSet(Rest[NextDirPos],(AllowDirectorySeparators)) do
     Inc (NextDirPos);
    NextPart := Copy (Rest, 1, Pred (NextDirPos));
{$IFDEF FPC_FEXPAND_DIRSEP_IS_UPDIR}
    if (Length (Rest) >= NextDirPos) and
                           CharInSet(Rest[NextDirPos],AllowDirectorySeparators) then
{$ELSE FPC_FEXPAND_DIRSEP_IS_UPDIR}
    while (Length (Rest) >= NextDirPos) and
                             CharInSet(Rest[NextDirPos],AllowDirectorySeparators) do
{$ENDIF FPC_FEXPAND_DIRSEP_IS_UPDIR}
     Inc (NextDirPos);
    NextRest := Copy (Rest, NextDirPos, Length (Rest) - Pred (NextDirPos));
    if (Base = '') or CharInSet(Base[Length (Base)],AllowDirectorySeparators) then
     SearchBase := Base
    else
{$ifdef SYSUTILSUNICODE}
     SearchBase := Base + DirectorySeparator;
    RC := FindFirst (SearchBase + AllFilesMask, faAnyFile, SR);
{$else SYSUTILSUNICODE}
     SearchBase := Base + ToSingleByteFileSystemEncodedFileName(DirectorySeparator);
    RC := FindFirst (SearchBase + ToSingleByteFileSystemEncodedFileName(AllFilesMask), faAnyFile, SR);
{$endif SYSUTILSUNICODE}
    while (RC = 0) and (ItemsFound < 2) do
     begin
      if UpCase (NextPart) = UpCase (SR.Name) then
       begin
        if Length (NextPart) = Length (Rest) then
         begin
          Inc (ItemsFound);
          if ItemsFound = 1 then
           FoundPath := SearchBase + SR.Name;
         end
        else if SR.Attr and faDirectory = faDirectory then
{$ifdef SYSUTILSUNICODE}
          TryCase (SearchBase + SR.Name + DirectorySeparator, NextRest);
{$else SYSUTILSUNICODE}
          TryCase (SearchBase + SR.Name + ToSingleByteFileSystemEncodedFileName(DirectorySeparator), NextRest);
{$endif SYSUTILSUNICODE}
       end;
      if ItemsFound < 2 then
       RC := FindNext (SR);
     end;
    FindClose (SR);
  end;

begin
  Result := ExpandFileName (FileName);
  if FileName = '' then
   MatchFound := mkExactMatch
  else
   if (FindFirst (FileName, faAnyFile, SR) = 0) or
(* Special check for a root directory or a directory with a trailing slash *)
(* which are not found using FindFirst. *)
                                                DirectoryExists (FileName) then
    begin
     MatchFound := mkExactMatch;
     Result := ExtractFilePath (Result) + SR.Name;
     FindClose (SR);
    end
   else
    begin
(* Better close the search handle here before starting the recursive search *)
     FindClose (SR);
     MatchFound := mkNone;
     if FileNameCaseSensitive then
      begin
       ItemsFound := 0;
       FoundPath := '';
       RestPos := Length (ExtractFileDrive (FileName)) + 1;
       if (Length (FileName) > RestPos) then
        begin
{$IFDEF FPC_FEXPAND_DIRSEP_IS_UPDIR}
         if (Length (FileName) >= RestPos) and
                CharInSet(FileName[RestPos],AllowDirectorySeparators) then
{$ELSE FPC_FEXPAND_DIRSEP_IS_UPDIR}
         while (Length (FileName) >= RestPos) and
                  CharInSet(FileName[RestPos],AllowDirectorySeparators) do
{$ENDIF FPC_FEXPAND_DIRSEP_IS_UPDIR}
          Inc (RestPos);
         Root := Copy (FileName, 1, Pred (RestPos));
         TryCase (Root, Copy (FileName, RestPos, Length (FileName) - Length (Root)));
         if ItemsFound > 0 then
          begin
           Result := ExpandFileName (FoundPath);
           if ItemsFound = 1 then
            MatchFound := mkSingleMatch
           else
            MatchFound := mkAmbiguous;
          end;
        end;
      end;
    end;
end;


{$if not declared(MaxDirs)}
Const
  MaxDirs = 129;
{$endif}

function ExtractRelativepath (Const BaseName,DestName : PathStr): PathStr;

Var Source, Dest   : PathStr;
    Sc,Dc,I,J
{$ifndef SYSUTILSUNICODE}
    ,Len, NewLen
{$endif not SYSUTILSUNICODE}
                   : Longint;
    SD,DD          : Array[1..MaxDirs] of PathPChar;

Const OneLevelBack = '..'+DirectorySeparator;

begin
  If Uppercase(ExtractFileDrive(BaseName))<>Uppercase(ExtractFileDrive(DestName)) Then
    begin
    Result:=DestName;
    exit;
    end;
  Source:=ExcludeTrailingPathDelimiter(ExtractFilePath(BaseName));
  Dest:=ExcludeTrailingPathDelimiter(ExtractFilePath(DestName));
  SC:=GetDirs (Source,SD);
  DC:=GetDirs (Dest,DD);
  I:=1;
  While (I<=DC) and (I<=SC) do
    begin
    If StrIcomp(DD[i],SD[i])=0 then
      Inc(i)
    else
      Break;
    end;
  Result:='';
{$ifdef SYSUTILSUNICODE}
  For J:=I to SC do Result:=Result+OneLevelBack;
  For J:=I to DC do Result:=Result+DD[J]+DirectorySeparator;
{$else SYSUTILSUNICODE}
  { prevent conversion to DefaultSystemCodePage due to concatenation of
    constant string -- and optimise a little by reducing the numher of
    setlength cals }
  if SC>=I then
    begin
      Len:=Length(Result);
      SetLength(Result,Len+(SC-I+1)*Length(OneLevelBack));
      For J:=0 to SC-I do
        move(shortstring(OneLevelBack)[1],Result[Len+1+J*Length(OneLevelBack)],Length(OneLevelBack));
    end;

  if DC>=I then
    begin
      Len:=Length(Result);
      NewLen:=Len+(DC-I+1)*sizeof(ansichar);
      For J:=I to DC do
        Inc(NewLen,Length(DD[J]));
      SetLength(Result,NewLen);
      For J:=I to DC do
        begin
          NewLen:=Length(DD[J]);
          Move(DD[J][0],Result[Len+1],NewLen);
          inc(Len,NewLen);
          Result[Len+1]:=DirectorySeparator;
          Inc(Len);
        end;
    end;
{$endif SYSUTILSUNICODE}
  Result:=Result+ExtractFileName(DestName);
end;

Procedure DoDirSeparators (Var FileName : PathStr); {$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif}

VAr I : longint;

begin
  For I:=1 to Length(FileName) do
    If CharInSet(FileName[I],AllowDirectorySeparators) then
      FileName[i]:=DirectorySeparator;
end;


Function SetDirSeparators (Const FileName : PathStr) : PathStr;

begin
  Result:=FileName;
  DoDirSeparators (Result);
end;

{
  DirName is split in a #0 separated list of directory names,
  Dirs is an array of pchars, pointing to these directory names.
  The function returns the number of directories found, or -1
  if none were found.
}

Function GetDirs (Var DirName : PathStr; Var Dirs : Array of PathPChar) : Longint; {$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif}

Var I : Longint;

begin
  I:=1;
  Result:=-1;
  While I<=Length(DirName) do
    begin
    If (CharInSet(DirName[i],AllowDirectorySeparators)
    {$ifdef HASAMIGA}
      or (DirName[i] = DriveSeparator)
    {$endif}
      ) and
       { avoid error in case last char=pathdelim }
       (length(dirname)>i) then
      begin
        DirName[i]:=#0;
        Inc(Result);
        Dirs[Result]:=@DirName[I+1];
      end;
    Inc(I);
    end;
  If Result>-1 then inc(Result);
end;

function IncludeTrailingPathDelimiter(Const Path : PathStr) : PathStr;

Var
  l : Integer;

begin
  Result:=Path;
  l:=Length(Result);
  If (L=0) or not CharInSet(Result[l],AllowDirectorySeparators) then
{$ifdef HASAMIGA}
    If (L>0) and (Result[l] <> DriveSeparator) then
{$endif}
{$ifdef SYSUTILSUNICODE}
    Result:=Result+DirectorySeparator;
{$else SYSUTILSUNICODE}
    begin
      SetLength(Result,l+1);
      Result[l+1]:=DirectorySeparator;
    end;
{$endif SYSUTILSUNICODE}
end;

function IncludeTrailingBackslash(Const Path : PathStr) : PathStr;

begin
  Result:=IncludeTrailingPathDelimiter(Path);
end;

function ExcludeTrailingBackslash(Const Path: PathStr): PathStr;

begin
  Result:=ExcludeTrailingPathDelimiter(Path);
end;

function ExcludeTrailingPathDelimiter(Const Path: PathStr): PathStr;

Var
  L : Integer;

begin
  L:=Length(Path);
  If (L>0) and CharInSet(Path[L],AllowDirectorySeparators) then
    Dec(L);
  Result:=Copy(Path,1,L);
end;

function IncludeLeadingPathDelimiter(Const Path : PathStr) : PathStr;

Var
  l : Integer;

begin
  Result:=Path;
  l:=Length(Result);
  If (L=0) or not CharInSet(Result[1],AllowDirectorySeparators) then
{$ifdef SYSUTILSUNICODE}
    Result:=DirectorySeparator+Result;
{$else SYSUTILSUNICODE}
    begin
      SetLength(Result,l+1);
      Move(Result[1],Result[2],l);
      Result[1]:=DirectorySeparator;
    end;
{$endif SYSUTILSUNICODE}
end;

function ExcludeLeadingPathDelimiter(Const Path: PathStr): PathStr;

Var
  L : Integer;

begin
  Result:=Path;
  L:=Length(Result);
  If (L>0) and CharInSet(Result[1],AllowDirectorySeparators) then
    Delete(Result,1,1);
end;

function IsPathDelimiter(Const Path: PathStr; Index: Integer): Boolean;

begin
  Result:=(Index>0) and (Index<=Length(Path)) and CharInSet(Path[Index],AllowDirectorySeparators);
end;

function ConcatPaths(const Paths: array of PathStr): PathStr;
var
  I: Integer;
begin
  if Length(Paths) > 0 then
  begin
    Result := Paths[0];
    for I := 1 to Length(Paths) - 1 do
      Result := IncludeTrailingPathDelimiter(Result) + ExcludeLeadingPathDelimiter(Paths[I]);
  end else
    Result := '';
end;

