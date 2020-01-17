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

{$if not declared(TFilenameCaseMatch)}
type
  TFilenameCaseMatch = (mkNone, mkExactMatch, mkSingleMatch, mkAmbiguous);
{$endif}

function ChangeFileExt(const FileName, Extension: PathStr): PathStr;
function ExtractFilePath(const FileName: PathStr): PathStr;
function ExtractFileDrive(const FileName: PathStr): PathStr;
function ExtractFileName(const FileName: PathStr): PathStr;
function ExtractFileExt(const FileName: PathStr): PathStr;
function ExtractFileDir(Const FileName : PathStr): PathStr;
function ExtractShortPathName(Const FileName : PathStr) : PathStr;
function ExpandFileName (Const FileName : PathStr): PathStr;
function ExpandFileName (Const FileName, BasePath : PathStr): PathStr;
function ExpandFileNameCase (const FileName: PathStr; out MatchFound: TFilenameCaseMatch): PathStr;
function ExpandUNCFileName (Const FileName : PathStr): PathStr;
function ExtractRelativePath (Const BaseName,DestName : PathStr): PathStr;
function IncludeTrailingPathDelimiter(Const Path : PathStr) : PathStr;
function IncludeTrailingBackslash(Const Path : PathStr) : PathStr;
function ExcludeTrailingBackslash(Const Path: PathStr): PathStr;
function ExcludeTrailingPathDelimiter(Const Path: PathStr): PathStr;
function IncludeLeadingPathDelimiter(Const Path : PathStr) : PathStr;
function ExcludeLeadingPathDelimiter(Const Path: PathStr): PathStr;
function IsPathDelimiter(Const Path: PathStr; Index: Integer): Boolean;
Procedure DoDirSeparators (Var FileName : PathStr); {$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif}
Function SetDirSeparators (Const FileName : PathStr) : PathStr;
Function GetDirs (Var DirName : PathStr; Var Dirs : Array of PathPChar) : Longint; {$ifdef FPC_HAS_CPSTRING}rtlproc;{$endif}
function ConcatPaths(const Paths: array of PathStr): PathStr;

