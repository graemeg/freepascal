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
{$if defined(unix) or defined(msdos) or defined(hasamiga) or defined(atari) or defined(win16)}
    {$define FINDHANDLE_IS_POINTER}
{$endif}

  // Some operating systems need extra find data.
{$if defined(Win32) or defined(WinCE) or defined(Win64)}
    {$define SEARCHREC_USEFINDDATA}
    {$define SYMLINKREC_USEFINDDATA}
    TFindData = TWin32FindDataW;
{$endif}
{$ifdef netware_clib}
    TFindData = TNetwareFindData;
    {$define SEARCHREC_USEFINDDATA}
{$endif}
{$ifdef netware_libc}
    TFindData = TNetwareLibcFindData;
    {$define SEARCHREC_USEFINDDATA}
{$endif}
{$ifdef MacOS}
    TFindData = TMacOSFindData;
    {$define SEARCHREC_USEFINDDATA}
{$endif}
{$ifdef nativent}
    TFindData = TNativeNTFindData;
    {$define SEARCHREC_USEFINDDATA}
{$endif}

  // The actual unicode search record
  TUnicodeSearchRec = Record
    Time : Int64;
    Size : Int64;
    Attr : Longint;
    Name : UnicodeString;
    ExcludeAttr : Longint;
    FindHandle : {$ifdef FINDHANDLE_IS_POINTER}Pointer{$else}THandle{$endif};
{$ifdef unix}
    Mode : TMode;
{$endif unix}
{$ifdef SEARCHREC_USEFINDDATA}
    FindData : TFindData;
{$endif}
  private
    function GetTimeStamp: TDateTime;
  public
    property TimeStamp: TDateTime read GetTimeStamp;
  end;

  TRawbyteSearchRec = Record
    Time : Int64;
    Size : Int64;
    Attr : Longint;
    Name : RawByteString;
    ExcludeAttr : Longint;
    FindHandle : {$ifdef FINDHANDLE_IS_POINTER}Pointer{$else}THandle{$endif};
{$ifdef unix}
    Mode : TMode;
{$endif unix}
{$IFDEF SEARCHREC_USEFINDDATA}
    FindData : TFindData;
{$ENDIF}
  private
    function GetTimeStamp: TDateTime;
  public
    property TimeStamp: TDateTime read GetTimeStamp;
  end;

{$IFDEF FPC_UNICODE_RTL}
  TSearchRec = TUnicodeSearchRec;
{$ELSE}
  TSearchRec = TRawbyteSearchRec;
{$ENDIF}

  TUnicodeSymLinkRec = Record
    TargetName : UnicodeString;
    Attr : Longint;
    Size : Int64;
{$ifdef unix}
    Mode : TMode;
{$endif unix}
{$ifdef SYMLINKREC_USEFINDDATA}
    FindData : TFindData;
{$endif}
  private
    function GetTimeStamp: TDateTime;
  public
    property TimeStamp: TDateTime read GetTimeStamp;
  end;

  TRawbyteSymLinkRec = Record
    TargetName : RawByteString;
    Size : Int64;
    Attr : Longint;
{$ifdef unix}
    Mode : TMode;
{$endif unix}
{$IFDEF SYMLINKREC_USEFINDDATA}
    FindData : TFindData;
{$ENDIF}
  private
    function GetTimeStamp: TDateTime;
  public
    property TimeStamp: TDateTime read GetTimeStamp;
  end;

{$IFDEF FPC_UNICODE_RTL}
  TSymLinkRec = TUnicodeSymLinkRec;
{$ELSE}
  TSymLinkRec = TRawbyteSymLinkRec;
{$ENDIF}


Const
  { File attributes }
  faReadOnly   = $00000001;
  faHidden     = $00000002 platform;
  faSysFile    = $00000004 platform;
  faVolumeId   = $00000008 platform deprecated;
  faDirectory  = $00000010;
  faArchive    = $00000020;
  faNormal     = $00000080;
  faTemporary  = $00000100 platform;
  faSymLink    = $00000400 platform;
  faCompressed = $00000800 platform;
  faEncrypted  = $00004000 platform;
  faVirtual    = $00010000 platform;
  faAnyFile    = $000001FF;
             

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

Type
  TFileSearchOption = (sfoImplicitCurrentDir,sfoStripQuotes);
  TFileSearchOptions = set of TFileSearchOption;

Function FileOpen (Const FileName : unicodestring; Mode : Integer) : THandle;
Function FileCreate (Const FileName : UnicodeString) : THandle;
Function FileCreate (Const FileName : UnicodeString; Rights : Integer) : THandle;
Function FileCreate (Const FileName : UnicodeString; ShareMode : Integer; Rights : Integer) : THandle;
{$IFNDEF FPUNONE}
Function FileAge (Const FileName : UnicodeString): Int64;
{$ENDIF}
Function FileExists (Const FileName : UnicodeString; FollowLink : Boolean = True) : Boolean;
Function DirectoryExists (Const Directory : UnicodeString; FollowLink : Boolean = True) : Boolean;
Function FileSetDate (Const FileName : UnicodeString;Age : Int64) : Longint;
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
function FileGetSymLinkTarget(const FileName: UnicodeString; out SymLinkRec: TUnicodeSymLinkRec): Boolean;
function FileGetSymLinkTarget(const FileName: UnicodeString; out TargetName: UnicodeString): Boolean; inline;

Function FileOpen (Const FileName : RawByteString; Mode : Integer) : THandle;
Function FileCreate (Const FileName : RawByteString) : THandle;
Function FileCreate (Const FileName : RawByteString; Rights : Integer) : THandle;
Function FileCreate (Const FileName : RawByteString; ShareMode : Integer; Rights : Integer) : THandle;
Function FileExists (Const FileName : RawByteString; FollowLink : Boolean = True) : Boolean;
Function DirectoryExists (Const Directory : RawByteString; FollowLink: Boolean = True) : Boolean;
Function FileSetDate (Const FileName : RawByteString;Age : Int64) : Longint;
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
Function FileAge (Const FileName : RawByteString): Int64;
{$endif}
function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
function FileGetSymLinkTarget(const FileName: RawByteString; out TargetName: RawByteString): Boolean; inline;

Function FileRead (Handle : THandle; out Buffer; Count : longint) : Longint;
Function FileWrite (Handle : THandle; const Buffer; Count : Longint) : Longint;
Function FileSeek (Handle : THandle; FOffset, Origin: Longint) : Longint;
Function FileSeek (Handle : THandle; FOffset: Int64; Origin: Longint) : Int64;
function FileFlush(Handle: THandle): Boolean; 
Procedure FileClose (Handle : THandle);
Function FileTruncate (Handle : THandle;Size: Int64) : boolean;
Function FindNext (Var Rslt : TRawByteSearchRec) : Longint;
Procedure FindClose (Var F : TRawByteSearchrec);
Function FileGetDate (Handle : THandle) : Int64;
Function FileSetDate (Handle : THandle;Age : Int64) : Longint;
Function GetFileHandle(var f : File):THandle;
Function GetFileHandle(var f : Text):THandle;
