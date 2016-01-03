{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    File utility calls

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Type


  // Some operating systems need FindHandle to be a Pointer
{$if defined(unix) or defined(msdos)}
    {$define FINDHANDLE_IS_POINTER}
{$endif}

  // Some operating systems need extra find data.
{$if defined(Win32) or defined(WinCE) or defined(Win64)}
    {$define USEFINDDATA}
    TFindData = TWin32FindDataW;
{$endif}
{$ifdef netware_clib}
    TFindData = TNetwareFindData;
    {$define USEFINDDATA}
{$endif}
{$ifdef netware_libc}
    TFindData = TNetwareLibcFindData;
    {$define USEFINDDATA}
{$endif}
{$ifdef MacOS}
    TFindData = TMacOSFindData;
    {$define USEFINDDATA}
{$endif}
{$ifdef nativent}
    TFindData = TNativeNTFindData;
    {$define USEFINDDATA}
{$endif}

  // The actual unicode search record
  TUnicodeSearchRec = Record
    Time : Longint;
    Size : Int64;
    Attr : Longint;
    Name : UnicodeString;
    ExcludeAttr : Longint;
    FindHandle : {$ifdef FINDHANDLE_IS_POINTER}Pointer{$else}THandle{$endif};
{$ifdef unix}
    Mode : TMode;
{$endif unix}
{$ifdef USEFINDDATA}
    FindData : TFindData;
{$endif}
  end;

  TRawbyteSearchRec = Record
    Time : Longint;
    Size : Int64;
    Attr : Longint;
    Name : RawByteString;
    ExcludeAttr : Longint;
    FindHandle : {$ifdef FINDHANDLE_IS_POINTER}Pointer{$else}THandle{$endif};
{$ifdef unix}
    Mode : TMode;
{$endif unix}
{$IFDEF USEFINDDATA}
    FindData : TFindData;
{$ENDIF}
  end;

{$IFDEF FPC_UNICODE_RTL}
  TSearchRec = TUnicodeSearchRec;
{$ELSE}
  TSearchRec = TRawbyteSearchRec;
{$ENDIF}

Const
  { File attributes }
  faReadOnly  = $00000001;
  faHidden    = $00000002;
  faSysFile   = $00000004;
  faVolumeId  = $00000008;
  faDirectory = $00000010;
  faArchive   = $00000020;
  faSymLink   = $00000040;
  faAnyFile   = $0000003f;

  { File open modes }
  fmOpenRead       = $0000;
  fmOpenWrite      = $0001;
  fmOpenReadWrite  = $0002;
  { Share modes}
  fmShareCompat    = $0000;
  fmShareExclusive = $0010;
  fmShareDenyWrite = $0020;
  fmShareDenyRead  = $0030;
  fmShareDenyNone  = $0040;

  { File seek origins }
  fsFromBeginning = 0;
  fsFromCurrent   = 1;
  fsFromEnd       = 2;

  { File errors }
  feInvalidHandle : THandle = THandle(-1);  //return value on FileOpen error

<<<<<<< HEAD
Type
  TFileSearchOption = (sfoImplicitCurrentDir,sfoStripQuotes);
  TFileSearchOptions = set of TFileSearchOption;

Function FileOpen (Const FileName : unicodestring; Mode : Integer) : THandle;
Function FileCreate (Const FileName : UnicodeString) : THandle;
Function FileCreate (Const FileName : UnicodeString; Rights : Integer) : THandle;
Function FileCreate (Const FileName : UnicodeString; ShareMode : Integer; Rights : Integer) : THandle;
{$IFNDEF FPUNONE}
Function FileAge (Const FileName : UnicodeString): Longint;
{$ENDIF}
Function FileExists (Const FileName : UnicodeString) : Boolean;
Function DirectoryExists (Const Directory : UnicodeString) : Boolean;
Function FileSetDate (Const FileName : UnicodeString;Age : Longint) : Longint;
Function FileGetAttr (Const FileName : UnicodeString) : Longint;
Function FileSetAttr (Const Filename : UnicodeString; Attr: longint) : Longint;
Function DeleteFile (Const FileName : UnicodeString) : Boolean;
Function RenameFile (Const OldName, NewName : UnicodeString) : Boolean;
Function FindFirst (Const Path : UnicodeString; Attr : Longint; out Rslt : TUnicodeSearchRec) : Longint;
Function FindNext (Var Rslt : TUnicodeSearchRec) : Longint;
Procedure FindClose (Var F : TUnicodeSearchrec);
Function FileSearch (Const Name, DirList : UnicodeString; Options : TFileSearchoptions = [sfoImplicitCurrentDir]) : UnicodeString;
Function FileSearch (Const Name, DirList : UnicodeString; ImplicitCurrentDir : Boolean) : UnicodeString;
Function ExeSearch  (Const Name : UnicodeString; Const DirList : UnicodeString = '') : UnicodeString;
Function FileIsReadOnly(const FileName : UnicodeString): Boolean;
function FileAge(const FileName: UnicodeString; out FileDateTime: TDateTime; FollowLink: Boolean = True): Boolean;

Function FileOpen (Const FileName : RawByteString; Mode : Integer) : THandle;
Function FileCreate (Const FileName : RawByteString) : THandle;
Function FileCreate (Const FileName : RawByteString; Rights : Integer) : THandle;
Function FileCreate (Const FileName : RawByteString; ShareMode : Integer; Rights : Integer) : THandle;
Function FileExists (Const FileName : RawByteString) : Boolean;
Function DirectoryExists (Const Directory : RawByteString) : Boolean;
Function FileSetDate (Const FileName : RawByteString;Age : Longint) : Longint;
Function FileGetAttr (Const FileName : RawByteString) : Longint;
Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
Function DeleteFile (Const FileName : RawByteString) : Boolean;
Function RenameFile (Const OldName, NewName : RawByteString) : Boolean;
Function FindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TRawByteSearchRec) : Longint;
Function FileSearch (Const Name, DirList : RawByteString; Options : TFileSearchoptions = [sfoImplicitCurrentDir]) : RawByteString;
Function FileSearch (Const Name, DirList : RawByteString; ImplicitCurrentDir : Boolean) : RawByteString;
Function ExeSearch  (Const Name : RawByteString; Const DirList : RawByteString = '') : RawByteString;
Function FileIsReadOnly(const FileName: RawByteString): Boolean;
function FileAge(const FileName: RawByteString; out FileDateTime: TDateTime; FollowLink: Boolean = True): Boolean;
{$ifndef FPUNONE}
Function FileAge (Const FileName : RawByteString): Longint;
{$endif}

=======
Function FileOpen (Const FileName : string; Mode : Integer) : THandle;
Function FileCreate (Const FileName : String) : THandle;
Function FileCreate (Const FileName : String; Mode : Integer) : THandle;
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
Function FileRead (Handle : THandle; out Buffer; Count : longint) : Longint;
Function FileWrite (Handle : THandle; const Buffer; Count : Longint) : Longint;
Function FileSeek (Handle : THandle; FOffset, Origin: Longint) : Longint;
Function FileSeek (Handle : THandle; FOffset: Int64; Origin: Longint) : Int64;
Procedure FileClose (Handle : THandle);
Function FileTruncate (Handle : THandle;Size: Int64) : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
Function FindNext (Var Rslt : TRawByteSearchRec) : Longint;
Procedure FindClose (Var F : TRawByteSearchrec);
=======
=======
>>>>>>> origin/fixes_2_2
Function FileAge (Const FileName : String): Longint;
Function FileExists (Const FileName : String) : Boolean;
Function DirectoryExists (Const Directory : String) : Boolean;
Function FindFirst (Const Path : String; Attr : Longint; out Rslt : TSearchRec) : Longint;
Function FindNext (Var Rslt : TSearchRec) : Longint;
Procedure FindClose (Var F : TSearchrec);
>>>>>>> graemeg/fixes_2_2
Function FileGetDate (Handle : THandle) : Longint;
Function FileSetDate (Handle : THandle;Age : Longint) : Longint;
<<<<<<< HEAD
Function GetFileHandle(var f : File):THandle;
Function GetFileHandle(var f : Text):THandle;
=======
Function FileSetDate (Const FileName : String;Age : Longint) : Longint;
Function FileGetAttr (Const FileName : String) : Longint;
Function FileSetAttr (Const Filename : String; Attr: longint) : Longint;
Function DeleteFile (Const FileName : String) : Boolean;
Function RenameFile (Const OldName, NewName : String) : Boolean;
Function FileSearch (Const Name, DirList : String; ImplicitCurrentDir : Boolean = True) : String;
Function ExeSearch  (Const Name, DirList : String) : String;
Function FileIsReadOnly(const FileName: String): Boolean;

Function GetFileHandle(var f : File):THandle;
Function GetFileHandle(var f : Text):THandle;

>>>>>>> graemeg/cpstrnew
