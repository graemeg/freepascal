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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
function ExpandFileNameCase (const FileName: PathStr; out MatchFound: TFilenameCaseMatch): PathStr;
function ExpandUNCFileName (Const FileName : PathStr): PathStr;
function ExtractRelativepath (Const BaseName,DestNAme : PathStr): PathStr;
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
=======
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function ChangeFileExt(const FileName, Extension: string): string;
function ExtractFilePath(const FileName: string): string;
function ExtractFileDrive(const FileName: string): string;
function ExtractFileName(const FileName: string): string;
function ExtractFileExt(const FileName: string): string;
function ExtractFileDir(Const FileName : string): string;
function ExtractShortPathName(Const FileName : String) : String;
function ExpandFileName (Const FileName : string): String;
function ExpandUNCFileName (Const FileName : string): String;
function ExtractRelativepath (Const BaseName,DestNAme : String): String;
function IncludeTrailingPathDelimiter(Const Path : String) : String;
function IncludeTrailingBackslash(Const Path : String) : String;
function ExcludeTrailingBackslash(Const Path: string): string;
function ExcludeTrailingPathDelimiter(Const Path: string): string;
function IncludeLeadingPathDelimiter(Const Path : String) : String;
function ExcludeLeadingPathDelimiter(Const Path: string): string;
function IsPathDelimiter(Const Path: string; Index: Integer): Boolean;
Procedure DoDirSeparators (Var FileName : String);
Function SetDirSeparators (Const FileName : String) : String;
Function GetDirs (Var DirName : String; Var Dirs : Array of pchar) : Longint;
Function SameFileName(const S1, S2: string): Boolean;
function ConcatPaths(const Paths: array of String): String;
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

