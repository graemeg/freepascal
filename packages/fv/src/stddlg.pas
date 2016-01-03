{*******************************************************}
{ Free Vision Runtime Library                           }
{ StdDlg Unit                                           }
{ Version: 0.1.0                                        }
{ Release Date: July 23, 1998                           }
{                                                       }
{*******************************************************}
{                                                       }
{ This unit is a port of Borland International's        }
{ StdDlg.pas unit.  It is for distribution with the     }
{ Free Pascal (FPK) Compiler as part of the 32-bit      }
{ Free Vision library.  The unit is still fully         }
{ functional under BP7 by using the tp compiler         }
{ directive when rebuilding the library.                }
{                                                       }
{*******************************************************}

{ Revision History

1.1a   (97/12/29)
  - fixed bug in TFileDialog.HandleEvent that prevented the user from being
    able to have an action taken automatically when the FileList was
    selected and kbEnter pressed

1.1
  - modified OpenNewFile to take a history list ID
  - implemented OpenNewFile

1.0   (1992)
  - original implementation }

unit StdDlg;

{
  This unit has been modified to make some functions global, apply patches
  from version 3.1 of the TVBUGS list, added TEditChDirDialog, and added
  several new global functions and procedures.
}

{$i platform.inc}

{$ifdef PPC_FPC}
  {$H-}
{$else}
  {$F+,O+,E+,N+}
{$endif}
{$X+,R-,I-,Q-,V-}
{$ifndef OS_UNIX}
  {$S-}
{$endif}
{$ifdef OS_DOS}
  {$define HAS_DOS_DRIVES}
{$endif}
{$ifdef OS_WINDOWS}
  {$define HAS_DOS_DRIVES}
{$endif}
{$ifdef OS_OS2}
  {$define HAS_DOS_DRIVES}
{$endif}

{2.0 compatibility}
{$ifdef VER2_0}
  {$macro on}
  {$define resourcestring := const}
{$endif}

interface

uses
  FVConsts, Objects, Drivers, Views, Dialogs, Validate, Dos;

const
  MaxDir   = 255;   { Maximum length of a DirStr. }
  MaxFName = 255; { Maximum length of a FNameStr. }

  DirSeparator : Char = system.DirectorySeparator;

{$ifdef Unix}
  AllFiles = '*';
{$else}
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef OS_AMIGA}
    AllFiles = '*';
  {$else}
    AllFiles = '*.*';
  {$endif}
=======
  AllFiles = '*.*';
>>>>>>> graemeg/fixes_2_2
=======
  AllFiles = '*.*';
>>>>>>> origin/fixes_2_2
{$endif}

type
  { TSearchRec }

  {  Record used to store directory information by TFileDialog
     This is a part of Dos.Searchrec for Bp !! }

  TSearchRec =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
  packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
  record
    Attr: Longint;
    Time: Longint;
    Size: Longint;
    Name: string[MaxFName];
  end;
  PSearchRec = ^TSearchRec;

type

  { TFileInputLine is a special input line that is used by      }
  { TFileDialog that will update its contents in response to a  }
  { cmFileFocused command from a TFileList.          }

  PFileInputLine = ^TFileInputLine;
  TFileInputLine = object(TInputLine)
    constructor Init(var Bounds: TRect; AMaxLen: Sw_Integer);
    procedure HandleEvent(var Event: TEvent); virtual;
  end;

  { TFileCollection is a collection of TSearchRec's. }

  PFileCollection = ^TFileCollection;
  TFileCollection = object(TSortedCollection)
    function Compare(Key1, Key2: Pointer): Sw_Integer; virtual;
    procedure FreeItem(Item: Pointer); virtual;
    function GetItem(var S: TStream): Pointer; virtual;
    procedure PutItem(var S: TStream; Item: Pointer); virtual;
  end;

  {#Z+}
  PFileValidator = ^TFileValidator;
  {#Z-}
  TFileValidator = Object(TValidator)
  end;  { of TFileValidator }

  { TSortedListBox is a TListBox that assumes it has a     }
  { TStoredCollection instead of just a TCollection.  It will   }
  { perform an incremental search on the contents.       }

  PSortedListBox = ^TSortedListBox;
  TSortedListBox = object(TListBox)
    SearchPos: Byte;
    {ShiftState: Byte;}
    HandleDir : boolean;
    constructor Init(var Bounds: TRect; ANumCols: Sw_Word;
      AScrollBar: PScrollBar);
    procedure HandleEvent(var Event: TEvent); virtual;
    function GetKey(var S: String): Pointer; virtual;
    procedure NewList(AList: PCollection); virtual;
  end;

  { TFileList is a TSortedList box that assumes it contains     }
  { a TFileCollection as its collection.  It also communicates  }
  { through broadcast messages to TFileInput and TInfoPane      }
  { what file is currently selected.             }

  PFileList = ^TFileList;
  TFileList = object(TSortedListBox)
    constructor Init(var Bounds: TRect; AScrollBar: PScrollBar);
    destructor Done; virtual;
    function DataSize: Sw_Word; virtual;
    procedure FocusItem(Item: Sw_Integer); virtual;
    procedure GetData(var Rec); virtual;
    function GetText(Item,MaxLen: Sw_Integer): String; virtual;
    function GetKey(var S: String): Pointer; virtual;
    procedure HandleEvent(var Event: TEvent); virtual;
    procedure ReadDirectory(AWildCard: PathStr);
    procedure SetData(var Rec); virtual;
  end;

  { TFileInfoPane is a TView that displays the information      }
  { about the currently selected file in the TFileList     }
  { of a TFileDialog.                  }

  PFileInfoPane = ^TFileInfoPane;
  TFileInfoPane = object(TView)
    S: TSearchRec;
    constructor Init(var Bounds: TRect);
    procedure Draw; virtual;
    function GetPalette: PPalette; virtual;
    procedure HandleEvent(var Event: TEvent); virtual;
  end;

  { TFileDialog is a standard file name input dialog      }

  TWildStr = PathStr;

const
  fdOkButton      = $0001;      { Put an OK button in the dialog }
  fdOpenButton    = $0002;      { Put an Open button in the dialog }
  fdReplaceButton = $0004;      { Put a Replace button in the dialog }
  fdClearButton   = $0008;      { Put a Clear button in the dialog }
  fdHelpButton    = $0010;      { Put a Help button in the dialog }
  fdNoLoadDir     = $0100;      { Do not load the current directory }
            { contents into the dialog at Init. }
            { This means you intend to change the }
            { WildCard by using SetData or store }
            { the dialog on a stream. }

type

  PFileHistory = ^TFileHistory;
  TFileHistory = object(THistory)
    CurDir : PString;
    procedure HandleEvent(var Event: TEvent);virtual;
    destructor Done; virtual;
    procedure AdaptHistoryToDir(Dir : string);
  end;

  PFileDialog = ^TFileDialog;
  TFileDialog = object(TDialog)
    FileName: PFileInputLine;
    FileList: PFileList;
    FileHistory: PFileHistory;
    WildCard: TWildStr;
    Directory: PString;
    constructor Init(AWildCard: TWildStr; const ATitle,
      InputName: String; AOptions: Word; HistoryId: Byte);
    constructor Load(var S: TStream);
    destructor Done; virtual;
    procedure GetData(var Rec); virtual;
    procedure GetFileName(var S: PathStr);
    procedure HandleEvent(var Event: TEvent); virtual;
    procedure SetData(var Rec); virtual;
    procedure Store(var S: TStream);
    function Valid(Command: Word): Boolean; virtual;
  private
    procedure ReadDirectory;
  end;

  { TDirEntry }

  PDirEntry = ^TDirEntry;
  TDirEntry = record
    DisplayText: PString;
    Directory: PString;
  end;  { of TDirEntry }

  { TDirCollection is a collection of TDirEntry's used by       }
  { TDirListBox.                 }

  PDirCollection = ^TDirCollection;
  TDirCollection = object(TCollection)
    function GetItem(var S: TStream): Pointer; virtual;
    procedure FreeItem(Item: Pointer); virtual;
    procedure PutItem(var S: TStream; Item: Pointer); virtual;
  end;

  { TDirListBox displays a tree of directories for use in the }
  { TChDirDialog.                    }

  PDirListBox = ^TDirListBox;
  TDirListBox = object(TListBox)
    Dir: DirStr;
    Cur: Word;
    constructor Init(var Bounds: TRect; AScrollBar: PScrollBar);
    destructor Done; virtual;
    function GetText(Item,MaxLen: Sw_Integer): String; virtual;
    procedure HandleEvent(var Event: TEvent); virtual;
    function IsSelected(Item: Sw_Integer): Boolean; virtual;
    procedure NewDirectory(var ADir: DirStr);
    procedure SetState(AState: Word; Enable: Boolean); virtual;
  end;

  { TChDirDialog is a standard change directory dialog. }

const
  cdNormal     = $0000; { Option to use dialog immediately }
  cdNoLoadDir  = $0001; { Option to init the dialog to store on a stream }
  cdHelpButton = $0002; { Put a help button in the dialog }

type

  PChDirDialog = ^TChDirDialog;
  TChDirDialog = object(TDialog)
    DirInput: PInputLine;
    DirList: PDirListBox;
    OkButton: PButton;
    ChDirButton: PButton;
    constructor Init(AOptions: Word; HistoryId: Sw_Word);
    constructor Load(var S: TStream);
    function DataSize: Sw_Word; virtual;
    procedure GetData(var Rec); virtual;
    procedure HandleEvent(var Event: TEvent); virtual;
    procedure SetData(var Rec); virtual;
    procedure Store(var S: TStream);
    function Valid(Command: Word): Boolean; virtual;
  private
    procedure SetUpDialog;
  end;

  PEditChDirDialog = ^TEditChDirDialog;
  TEditChDirDialog = Object(TChDirDialog)
    { TEditChDirDialog allows setting/getting the starting directory.  The
      transfer record is a DirStr. }
    function DataSize : Sw_Word; virtual;
    procedure GetData (var Rec); virtual;
    procedure SetData (var Rec); virtual;
  end;  { of TEditChDirDialog }


  {#Z+}
  PDirValidator = ^TDirValidator;
  {#Z-}
  TDirValidator = Object(TFilterValidator)
    constructor Init;
    function IsValid(const S: string): Boolean; virtual;
    function IsValidInput(var S: string; SuppressFill: Boolean): Boolean;
      virtual;
  end;  { of TDirValidator }


  FileConfirmFunc = function (AFile : FNameStr) : Boolean;
    { Functions of type FileConfirmFunc's are used to prompt the end user for
      confirmation of an operation.

      FileConfirmFunc's should ask the user whether to perform the desired
      action on the file named AFile.  If the user elects to perform the
      function FileConfirmFunc's return True, otherwise they return False.

      Using FileConfirmFunc's allows routines to be coded independant of the
      user interface implemented.  OWL and TurboVision are supported through
      conditional defines.  If you do not use either user interface you must
      compile this unit with the conditional define cdNoMessages and set all
      FileConfirmFunc variables to a valid function prior to calling any
      routines in this unit. }
    {#X ReplaceFile DeleteFile }


var

  ReplaceFile : FileConfirmFunc;
    { ReplaceFile returns True if the end user elects to replace the existing
      file with the new file, otherwise it returns False.

      ReplaceFile is only called when #CheckOnReplace# is True. }
    {#X DeleteFile }

  DeleteFile : FileConfirmFunc;
    { DeleteFile returns True if the end user elects to delete the file,
      otherwise it returns False.

       DeleteFile is only called when #CheckOnDelete# is True. }
    {#X ReplaceFile }


const

  CInfoPane = #30;

  { TStream registration records }

function Contains(S1, S2: String): Boolean;
  { Contains returns true if S1 contains any characters in S2. }

function DriveValid(Drive: Char): Boolean;
  { DriveValid returns True if Drive is a valid DOS drive.  Drive valid works
    by attempting to change the current directory to Drive, then restoring
    the original directory. }

function ExtractDir(AFile: FNameStr): DirStr;
  { ExtractDir returns the path of AFile terminated with a trailing '\'.  If
    AFile contains no directory information, an empty string is returned. }

function ExtractFileName(AFile: FNameStr): NameStr;
  { ExtractFileName returns the file name without any directory or file
    extension information. }

function Equal(const S1, S2: String; Count: Sw_word): Boolean;
  { Equal returns True if S1 equals S2 for up to Count characters.  Equal is
    case-insensitive. }

function FileExists (AFile : FNameStr) : Boolean;
  { FileExists looks for the file specified in AFile.  If AFile is present
    FileExists returns true, otherwise FileExists returns False.

    The search is performed relative to the current system directory, but
    other directories may be searched by prefacing a file name with a valid
    directory path.

    There is no check for a vaild file name or drive.  Errrors are handled
    internally and not reported in DosError.  Critical errors are left to
    the system's critical error handler. }
  {#X OpenFile }

function GetCurDir: DirStr;
  { GetCurDir returns the current directory.  The directory returned always
    ends with a trailing backslash '\'. }

function GetCurDrive: Char;
  { GetCurDrive returns the letter of the current drive as reported by the
    operating system. }

function IsWild(const S: String): Boolean;
  { IsWild returns True if S contains a question mark (?) or asterix (*). }

function IsList(const S: String): Boolean;
  { IsList returns True if S contains list separator (;) char }

function IsDir(const S: String): Boolean;
  { IsDir returns True if S is a valid DOS directory. }

{procedure MakeResources;}
  { MakeResources places a language specific version of all resources
    needed for the StdDlg unit to function on the RezFile using the string
    constants and variables in the Resource unit.  The Resource unit and the
    appropriate string lists must be initialized prior to calling this
    procedure. }

function NoWildChars(S: String): String;
  { NoWildChars deletes the wild card characters ? and * from the string S
    and returns the result. }

function OpenFile (var AFile : FNameStr; HistoryID : Byte) : Boolean;
  { OpenFile prompts the user to select a file using the file specifications
    in AFile as the starting file and path.  Wildcards are accepted.  If the
    user accepts a file OpenFile returns True, otherwise OpenFile returns
    False.

    Note: The file returned may or may not exist. }

function OpenNewFile (var AFile: FNameStr; HistoryID: Byte): Boolean;
  { OpenNewFile allows the user to select a directory from disk and enter a
    new file name.  If the file name entered is an existing file the user is
    optionally prompted for confirmation of replacing the file based on the
    value in #CheckOnReplace#.  If a file name is successfully entered,
    OpenNewFile returns True. }
  {#X OpenFile }

function PathValid(var Path: PathStr): Boolean;
  { PathValid returns True if Path is a valid DOS path name.  Path may be a
    file or directory name.  Trailing '\'s are removed. }

procedure RegisterStdDlg;
  { RegisterStdDlg registers all objects in the StdDlg unit for stream
    usage. }

function SaveAs (var AFile : FNameStr; HistoryID : Word) : Boolean;
  { SaveAs prompts the user for a file name using AFile as a template.  If
    AFile already exists and CheckOnReplace is True, the user is prompted
    to replace the file.

    If a valid file name is entered SaveAs returns True, other SaveAs returns
    False. }

function SelectDir (var ADir : DirStr; HistoryID : Byte) : Boolean;
  { SelectDir prompts the user to select a directory using ADir as the
    starting directory.  If a directory is selected, SelectDir returns True.
    The directory returned is gauranteed to exist. }

function ShrinkPath (AFile : FNameStr; MaxLen : Byte) : FNameStr;
  { ShrinkPath returns a file name with a maximu length of MaxLen.
    Internal directories are removed and replaced with elipses as needed to
    make the file name fit in MaxLen.

    AFile must be a valid path name. }

function StdDeleteFile (AFile : FNameStr) : Boolean;
  { StdDeleteFile returns True if the end user elects to delete the file,
    otherwise it returns False.

    DeleteFile is only called when CheckOnDelete is True. }

function StdReplaceFile (AFile : FNameStr) : Boolean;
  { StdReplaceFile returns True if the end user elects to replace the existing
    AFile with the new AFile, otherwise it returns False.

    ReplaceFile is only called when CheckOnReplace is True. }

function ValidFileName(var FileName: PathStr): Boolean;
  { ValidFileName returns True if FileName is a valid DOS file name. }


const
  CheckOnReplace : Boolean = True;
    { CheckOnReplace is used by file functions.  If a file exists, it is
      optionally replaced based on the value of CheckOnReplace.

      If CheckOnReplace is False the file is replaced without asking the
      user.  If CheckOnReplace is True, the end user is asked to replace the
      file using a call to ReplaceFile.

      CheckOnReplace is set to True by default. }

  CheckOnDelete : Boolean = True;
    { CheckOnDelete is used by file and directory functions.  If a file
      exists, it is optionally deleted based on the value of CheckOnDelete.

      If CheckOnDelete is False the file or directory is deleted without
      asking the user.  If CheckOnDelete is True, the end user is asked to
      delete the file/directory using a call to DeleteFile.

      CheckOnDelete is set to True by default. }



const
  RFileInputLine: TStreamRec = (
     ObjType: idFileInputLine;
     VmtLink: Ofs(TypeOf(TFileInputLine)^);
     Load:    @TFileInputLine.Load;
     Store:   @TFileInputLine.Store
  );

  RFileCollection: TStreamRec = (
     ObjType: idFileCollection;
     VmtLink: Ofs(TypeOf(TFileCollection)^);
     Load:    @TFileCollection.Load;
     Store:   @TFileCollection.Store
  );

  RFileList: TStreamRec = (
     ObjType: idFileList;
     VmtLink: Ofs(TypeOf(TFileList)^);
     Load:    @TFileList.Load;
     Store:   @TFileList.Store
  );

  RFileInfoPane: TStreamRec = (
     ObjType: idFileInfoPane;
     VmtLink: Ofs(TypeOf(TFileInfoPane)^);
     Load:    @TFileInfoPane.Load;
     Store:   @TFileInfoPane.Store
  );

  RFileDialog: TStreamRec = (
     ObjType: idFileDialog;
     VmtLink: Ofs(TypeOf(TFileDialog)^);
     Load:    @TFileDialog.Load;
     Store:   @TFileDialog.Store
  );

  RDirCollection: TStreamRec = (
     ObjType: idDirCollection;
     VmtLink: Ofs(TypeOf(TDirCollection)^);
     Load:    @TDirCollection.Load;
     Store:   @TDirCollection.Store
  );

  RDirListBox: TStreamRec = (
     ObjType: idDirListBox;
     VmtLink: Ofs(TypeOf(TDirListBox)^);
     Load:    @TDirListBox.Load;
     Store:   @TDirListBox.Store
  );

  RChDirDialog: TStreamRec = (
     ObjType: idChDirDialog;
     VmtLink: Ofs(TypeOf(TChDirDialog)^);
     Load:    @TChDirDialog.Load;
     Store:   @TChDirDialog.Store
  );

  RSortedListBox: TStreamRec = (
     ObjType: idSortedListBox;
     VmtLink: Ofs(TypeOf(TSortedListBox)^);
     Load:    @TSortedListBox.Load;
     Store:   @TSortedListBox.Store
  );

  REditChDirDialog : TStreamRec = (
    ObjType : idEditChDirDialog;
    VmtLink : Ofs(TypeOf(TEditChDirDialog)^);
    Load    : @TEditChDirDialog.Load;
    Store   : @TEditChDirDialog.Store);


implementation

{****************************************************************************}
{            Local Declarations              }
{****************************************************************************}

uses
  App, {Memory,} HistList, MsgBox{, Resource};

type

  PStringRec = record
    { PStringRec is needed for properly displaying PStrings using
      MessageBox. }
    AString : PString;
  end;

resourcestring  sChangeDirectory='Change Directory';
                sDeleteFile='Delete file?'#13#10#13#3'%s';
                sDirectory='Directory';
                sDrives='Drives';
                sInvalidDirectory='Invalid directory.';
                sInvalidDriveOrDir='Invalid drive or directory.';
                sInvalidFileName='Invalid file name.';
                sOpen='Open';
                sReplaceFile='Replace file?'#13#10#13#3'%s';
                sSaveAs='Save As';
                sTooManyFiles='Too many files.';

                smApr='Apr';
                smAug='Aug';
                smDec='Dec';
                smFeb='Feb';
                smJan='Jan';
                smJul='Jul';
                smJun='Jun';
                smMar='Mar';
                smMay='May';
                smNov='Nov';
                smOct='Oct';
                smSep='Sep';

                slChDir='~C~hdir';
                slClear='C~l~ear';
                slDirectoryName='Directory ~n~ame';
                slDirectoryTree='Directory ~t~ree';
                slFiles='~F~iles';
                slReplace='~R~eplace';
                slRevert='~R~evert';

{****************************************************************************}
{ PathValid                        }
{****************************************************************************}
{$ifdef go32v2}
{$define NetDrive}
{$endif go32v2}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef OS_WINDOWS}
{$define NetDrive}
{$endif OS_WINDOWS}
=======
{$ifdef win32}
{$define NetDrive}
{$endif win32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef win32}
{$define NetDrive}
{$endif win32}
>>>>>>> origin/fixes_2_2

procedure RemoveDoubleDirSep(var ExpPath : PathStr);
var
  p: longint;
{$ifdef NetDrive}
  OneDirSepRemoved: boolean;
{$endif NetDrive}
begin
  p:=pos(DirSeparator+DirSeparator,ExpPath);
{$ifdef NetDrive}
  if p=1 then
    begin
      ExpPath:=Copy(ExpPath,1,high(ExpPath));
      OneDirSepRemoved:=true;
      p:=pos(DirSeparator+DirSeparator,ExpPath);
    end
  else
    OneDirSepRemoved:=false;
{$endif NetDrive}
  while p>0 do
    begin
      ExpPath:=Copy(ExpPath,1,p)+Copy(ExpPath,p+2,high(ExpPath));
      p:=pos(DirSeparator+DirSeparator,ExpPath);
    end;
{$ifdef NetDrive}
  if OneDirSepRemoved then
    ExpPath:=DirSeparator+ExpPath;
{$endif NetDrive}
end;

function PathValid (var Path: PathStr): Boolean;
var
  ExpPath: PathStr;
  SR: SearchRec;
begin
  RemoveDoubleDirSep(Path);
  ExpPath := FExpand(Path);
{$ifdef HAS_DOS_DRIVES}
  if (Length(ExpPath) <= 3) then
    PathValid := DriveValid(ExpPath[1])
  else
{$endif}
  begin
    { do not change '/' into '' }
    if (Length(ExpPath)>1) and (ExpPath[Length(ExpPath)] = DirSeparator) then
      Dec(ExpPath[0]);
    // This function is called on current directories.
    // If the current dir starts with a . on Linux it is is hidden.
    // That's why we allow hidden dirs below (bug 6173)
<<<<<<< HEAD
<<<<<<< HEAD
    FindFirst(ExpPath, Directory+hidden, SR);
=======
    FindFirst(ExpPath, Directory+hidden, SR); 
>>>>>>> graemeg/fixes_2_2
=======
    FindFirst(ExpPath, Directory+hidden, SR); 
>>>>>>> origin/fixes_2_2
    PathValid := (DosError = 0) and (SR.Attr and Directory <> 0);
{$ifdef NetDrive}
    if (DosError<>0) and (length(ExpPath)>2) and
       (ExpPath[1]='\') and (ExpPath[2]='\')then
      begin
        { Checking '\\machine\sharedfolder' directly always fails..
          rather try '\\machine\sharedfolder\*' PM }
      {$ifdef fpc}
        FindClose(SR);
      {$endif}
        FindFirst(ExpPath+'\*',AnyFile,SR);
        PathValid:=(DosError = 0);
      end;
{$endif NetDrive}
    {$ifdef fpc}
    FindClose(SR);
   {$endif}
  end;
end;

{****************************************************************************}
{ TDirValidator Object                        }
{****************************************************************************}
{****************************************************************************}
{ TDirValidator.Init                    }
{****************************************************************************}
constructor TDirValidator.Init;
const   { What should this list be?  The commented one doesn't allow home,
  end, right arrow, left arrow, Ctrl+XXXX, etc. }
  Chars: TCharSet = ['A'..'Z','a'..'z','.','~',':','_','-'];
{  Chars: TCharSet = [#0..#255]; }
begin
  Chars := Chars + [DirSeparator];
  if not inherited Init(Chars) then
    Fail;
end;

{****************************************************************************}
{ TDirValidator.IsValid                      }
{****************************************************************************}
function TDirValidator.IsValid(const S: string): Boolean;
begin
{  IsValid := False; }
  IsValid := True;
end;

{****************************************************************************}
{ TDirValidator.IsValidInput                  }
{****************************************************************************}
function TDirValidator.IsValidInput(var S: string; SuppressFill: Boolean): Boolean;
begin
{  IsValid := False; }
  IsValidInput := True;
end;

{****************************************************************************}
{ TFileInputLine Object                      }
{****************************************************************************}
{****************************************************************************}
{ TFileInputLine.Init                     }
{****************************************************************************}
constructor TFileInputLine.Init(var Bounds: TRect; AMaxLen: Sw_Integer);
begin
  TInputLine.Init(Bounds, AMaxLen);
  EventMask := EventMask or evBroadcast;
end;

{****************************************************************************}
{ TFileInputLine.HandleEvent                  }
{****************************************************************************}
procedure TFileInputLine.HandleEvent(var Event: TEvent);
begin
  TInputLine.HandleEvent(Event);
  if (Event.What = evBroadcast) and (Event.Command = cmFileFocused) and
    (State and sfSelected = 0) then
  begin
     if PSearchRec(Event.InfoPtr)^.Attr and Directory <> 0 then
       begin
          Data^ := PSearchRec(Event.InfoPtr)^.Name + DirSeparator +
            PFileDialog(Owner)^.WildCard;
          { PFileDialog(Owner)^.FileHistory^.AdaptHistoryToDir(
              PSearchRec(Event.InfoPtr)^.Name+DirSeparator);}
       end
     else Data^ := PSearchRec(Event.InfoPtr)^.Name;
     DrawView;
  end;
end;

{****************************************************************************}
{ TFileCollection Object                       }
{****************************************************************************}
{****************************************************************************}
{ TFileCollection.Compare                     }
{****************************************************************************}
  function uppername(const s : string) : string;
  var
    i  : Sw_integer;
    in_name : boolean;
  begin
     in_name:=true;
     for i:=length(s) downto 1 do
      if in_name and (s[i] in ['a'..'z']) then
        uppername[i]:=char(byte(s[i])-32)
      else
       begin
          uppername[i]:=s[i];
          if s[i] = DirSeparator then
            in_name:=false;
       end;
     uppername[0]:=s[0];
  end;

function TFileCollection.Compare(Key1, Key2: Pointer): Sw_Integer;
begin
  if PSearchRec(Key1)^.Name = PSearchRec(Key2)^.Name then Compare := 0
  else if PSearchRec(Key1)^.Name = '..' then Compare := 1
  else if PSearchRec(Key2)^.Name = '..' then Compare := -1
  else if (PSearchRec(Key1)^.Attr and Directory <> 0) and
     (PSearchRec(Key2)^.Attr and Directory = 0) then Compare := 1
  else if (PSearchRec(Key2)^.Attr and Directory <> 0) and
     (PSearchRec(Key1)^.Attr and Directory = 0) then Compare := -1
  else if UpperName(PSearchRec(Key1)^.Name) > UpperName(PSearchRec(Key2)^.Name) then
    Compare := 1
{$ifdef unix}
  else if UpperName(PSearchRec(Key1)^.Name) < UpperName(PSearchRec(Key2)^.Name) then
    Compare := -1
  else if PSearchRec(Key1)^.Name > PSearchRec(Key2)^.Name then
    Compare := 1
{$endif def unix}
  else
    Compare := -1;
end;

{****************************************************************************}
{ TFileCollection.FreeItem                   }
{****************************************************************************}
procedure TFileCollection.FreeItem(Item: Pointer);
begin
  Dispose(PSearchRec(Item));
end;

{****************************************************************************}
{ TFileCollection.GetItem                     }
{****************************************************************************}
function TFileCollection.GetItem(var S: TStream): Pointer;
var
  Item: PSearchRec;
begin
  New(Item);
  S.Read(Item^, SizeOf(TSearchRec));
  GetItem := Item;
end;

{****************************************************************************}
{ TFileCollection.PutItem                     }
{****************************************************************************}
procedure TFileCollection.PutItem(var S: TStream; Item: Pointer);
begin
  S.Write(Item^, SizeOf(TSearchRec));
end;


{*****************************************************************************
               TFileList
*****************************************************************************}

const
  ListSeparator=';';

function MatchesMask(What, Mask: string): boolean;

  function upper(const s : string) : string;
  var
    i  : Sw_integer;
  begin
     for i:=1 to length(s) do
      if s[i] in ['a'..'z'] then
       upper[i]:=char(byte(s[i])-32)
      else
       upper[i]:=s[i];
     upper[0]:=s[0];
  end;

  Function CmpStr(const hstr1,hstr2:string):boolean;
  var
    found : boolean;
    i1,i2 : Sw_integer;
  begin
    i1:=0;
    i2:=0;
    if hstr1='' then
      begin
        CmpStr:=(hstr2='');
        exit;
      end;
    found:=true;
    repeat
      inc(i1);
      if (i1>length(hstr1)) then
        break;
      inc(i2);
      if (i2>length(hstr2)) then
        break;
      case hstr1[i1] of
        '?' :
          found:=true;
        '*' :
          begin
            found:=true;
            if (i1=length(hstr1)) then
             i2:=length(hstr2)
            else
             if (i1<length(hstr1)) and (hstr1[i1+1]<>hstr2[i2]) then
              begin
                if i2<length(hstr2) then
                 dec(i1)
              end
            else
             if i2>1 then
              dec(i2);
          end;
        else
          found:=(hstr1[i1]=hstr2[i2]) or (hstr2[i2]='?');
      end;
    until not found;
    if found then
      begin
        found:=(i2>=length(hstr2)) and
               (
                (i1>length(hstr1)) or
                ((i1=length(hstr1)) and
                 (hstr1[i1]='*'))
               );
      end;
    CmpStr:=found;
  end;

var
  D1,D2 : DirStr;
  N1,N2 : NameStr;
  E1,E2 : Extstr;
begin
{$ifdef Unix}
  FSplit(What,D1,N1,E1);
  FSplit(Mask,D2,N2,E2);
{$else}
  FSplit(Upper(What),D1,N1,E1);
  FSplit(Upper(Mask),D2,N2,E2);
{$endif}
  MatchesMask:=CmpStr(N2,N1) and CmpStr(E2,E1);
end;

function MatchesMaskList(What, MaskList: string): boolean;
var P: integer;
    Match: boolean;
begin
  Match:=false;
  if What<>'' then
  repeat
    P:=Pos(ListSeparator, MaskList);
    if P=0 then P:=length(MaskList)+1;
    Match:=MatchesMask(What,copy(MaskList,1,P-1));
    Delete(MaskList,1,P);
  until Match or (MaskList='');
  MatchesMaskList:=Match;
end;

constructor TFileList.Init(var Bounds: TRect; AScrollBar: PScrollBar);
begin
  TSortedListBox.Init(Bounds, 2, AScrollBar);
end;

destructor TFileList.Done;
begin
  if List <> nil then Dispose(List, Done);
  TListBox.Done;
end;

function TFileList.DataSize: Sw_Word;
begin
  DataSize := 0;
end;

procedure TFileList.FocusItem(Item: Sw_Integer);
begin
  TSortedListBox.FocusItem(Item);
  if (List^.Count > 0) then
    Message(Owner, evBroadcast, cmFileFocused, List^.At(Item));
end;

procedure TFileList.GetData(var Rec);
begin
end;

function TFileList.GetKey(var S: String): Pointer;
const
  SR: TSearchRec = ();

procedure UpStr(var S: String);
var
  I: Sw_Integer;
begin
  for I := 1 to Length(S) do S[I] := UpCase(S[I]);
end;

begin
  if (HandleDir{ShiftState and $03 <> 0}) or ((S <> '') and (S[1]='.')) then
    SR.Attr := Directory
  else SR.Attr := 0;
  SR.Name := S;
{$ifndef Unix}
  UpStr(SR.Name);
{$endif Unix}
  GetKey := @SR;
end;

function TFileList.GetText(Item,MaxLen: Sw_Integer): String;
var
  S: String;
  SR: PSearchRec;
begin
  SR := PSearchRec(List^.At(Item));
  S := SR^.Name;
  if SR^.Attr and Directory <> 0 then
  begin
    S[Length(S)+1] := DirSeparator;
    Inc(S[0]);
  end;
  GetText := S;
end;

procedure TFileList.HandleEvent(var Event: TEvent);
var
  S : String;
  K : pointer;
  Value : Sw_integer;
begin
  if (Event.What = evMouseDown) and (Event.Double) then
  begin
    Event.What := evCommand;
    Event.Command := cmOK;
    PutEvent(Event);
    ClearEvent(Event);
  end
  else if (Event.What = evKeyDown) and (Event.CharCode='<') then
  begin
    { select '..' }
      S := '..';
      K := GetKey(S);
      If PSortedCollection(List)^.Search(K, Value) then
        FocusItem(Value);
  end
  else TSortedListBox.HandleEvent(Event);
end;

procedure TFileList.ReadDirectory(AWildCard: PathStr);
const
  FindAttr = ReadOnly + Archive;
  PrevDir  = '..';
var
  S: SearchRec;
  P: PSearchRec;
  FileList: PFileCollection;
  NumFiles: Word;
  FindStr,
  WildName : string;
  Dir: DirStr;
  Ext: ExtStr;
  Name: NameStr;
  Event : TEvent;
  Tmp: PathStr;
begin
  NumFiles := 0;
  FileList := New(PFileCollection, Init(5, 5));
  AWildCard := FExpand(AWildCard);
  FSplit(AWildCard, Dir, Name, Ext);
  if pos(ListSeparator,AWildCard)>0 then
   begin
     WildName:=Copy(AWildCard,length(Dir)+1,255);
     FindStr:=Dir+AllFiles;
   end
  else
   begin
     WildName:=Name+Ext;
     FindStr:=AWildCard;
   end;
  FindFirst(FindStr, FindAttr, S);
  P := PSearchRec(@P);
  while assigned(P) and (DosError = 0) do
   begin
     if (S.Attr and Directory = 0) and
        MatchesMaskList(S.Name,WildName) then
     begin
{       P := MemAlloc(SizeOf(P^));
       if assigned(P) then
       begin}
         new(P);
         P^.Attr:=S.Attr;
         P^.Time:=S.Time;
         P^.Size:=S.Size;
         P^.Name:=S.Name;
         FileList^.Insert(P);
{       end;}
     end;
     FindNext(S);
   end;
 {$ifdef fpc}
  FindClose(S);
 {$endif}

  Tmp := Dir + AllFiles;
  FindFirst(Tmp, Directory, S);
  while (P <> nil) and (DosError = 0) do
  begin
    if (S.Attr and Directory <> 0) and (S.Name <> '.') and (S.Name <> '..') then
    begin
{      P := MemAlloc(SizeOf(P^));
      if P <> nil then
      begin}
        new(p);
        P^.Attr:=S.Attr;
        P^.Time:=S.Time;
        P^.Size:=S.Size;
        P^.Name:=S.Name;
        FileList^.Insert(P);
{      end;}
    end;
    FindNext(S);
  end;
 {$ifdef fpc}
  FindClose(S);
 {$endif}
 {$ifndef Unix}
  if Length(Dir) > 4 then
 {$endif not Unix}
  begin
{
    P := MemAlloc(SizeOf(P^));
    if P <> nil then
    begin}
      new(p);
      FindFirst(Tmp, Directory, S);
      FindNext(S);
      if (DosError = 0) and (S.Name = PrevDir) then
       begin
         P^.Attr:=S.Attr;
         P^.Time:=S.Time;
         P^.Size:=S.Size;
         P^.Name:=S.Name;
       end
      else
       begin
         P^.Name := PrevDir;
         P^.Size := 0;
         P^.Time := $210000;
         P^.Attr := Directory;
       end;
      FileList^.Insert(PSearchRec(P));
     {$ifdef fpc}
      FindClose(S);
     {$endif}
{    end;}
  end;
  if P = nil then
    MessageBox(sTooManyFiles, nil, mfOkButton + mfWarning);
  NewList(FileList);
  if List^.Count > 0 then
  begin
    Event.What := evBroadcast;
    Event.Command := cmFileFocused;
    Event.InfoPtr := List^.At(0);
    Owner^.HandleEvent(Event);
  end;
end;

procedure TFileList.SetData(var Rec);
begin
  with PFileDialog(Owner)^ do
    Self.ReadDirectory(Directory^ + WildCard);
end;

{****************************************************************************}
{ TFileInfoPane Object                        }
{****************************************************************************}
{****************************************************************************}
{ TFileInfoPane.Init                    }
{****************************************************************************}
constructor TFileInfoPane.Init(var Bounds: TRect);
begin
  TView.Init(Bounds);
  FillChar(S,SizeOf(S),#0);
  EventMask := EventMask or evBroadcast;
end;

{****************************************************************************}
{ TFileInfoPane.Draw                    }
{****************************************************************************}
procedure TFileInfoPane.Draw;
var
  B: TDrawBuffer;
  D: String[9];
  M: String[3];
  PM: Boolean;
  Color: Word;
  Time: DateTime;
  Path: PathStr;
  FmtId: String;
  Params: array[0..7] of PtruInt;
  Str: String[80];
const
  sDirectoryLine = ' %-12s %-9s %3s %2d, %4d  %2d:%02d%cm';
  sFileLine      = ' %-12s %-9d %3s %2d, %4d  %2d:%02d%cm';
  InValidFiles : array[0..2] of string[12] = ('','.','..');
var
  Month: array[1..12] of String[3];
begin
  Month[1] := smJan;
  Month[2] := smFeb;
  Month[3] := smMar;
  Month[4] := smApr;
  Month[5] := smMay;
  Month[6] := smJun;
  Month[7] := smJul;
  Month[8] := smAug;
  Month[9] := smSep;
  Month[10] := smOct;
  Month[11] := smNov;
  Month[12] := smDec;
  { Display path }
  if (PFileDialog(Owner)^.Directory <> nil) then
    Path := PFileDialog(Owner)^.Directory^
  else Path := '';
  Path := FExpand(Path+PFileDialog(Owner)^.WildCard);
  { avoid B Buffer overflow PM }
  Path := ShrinkPath(Path, Size.X - 1);
  Color := GetColor($01);
  MoveChar(B, ' ', Color, Size.X); { fill with empty spaces }
  WriteLine(0, 0, Size.X, Size.Y, B);
  MoveStr(B[1], Path, Color);
  WriteLine(0, 0, Size.X, 1, B);
  if (S.Name = InValidFiles[0]) or (S.Name = InValidFiles[1]) or
     (S.Name = InValidFiles[2]) then
    Exit;

  { Display file }
  Params[0] := ptruint(@S.Name);
  if S.Attr and Directory <> 0 then
  begin
    FmtId := sDirectoryLine;
    D := sDirectory;
    Params[1] := ptruint(@D);
  end else
  begin
    FmtId := sFileLine;
    Params[1] := S.Size;
  end;
  UnpackTime(S.Time, Time);
  M := Month[Time.Month];
  Params[2] := ptruint(@M);
  Params[3] := Time.Day;
  Params[4] := Time.Year;
  PM := Time.Hour >= 12;
  Time.Hour := Time.Hour mod 12;
  if Time.Hour = 0 then Time.Hour := 12;
  Params[5] := Time.Hour;
  Params[6] := Time.Min;
  if PM then
    Params[7] := Byte('p')
  else Params[7] := Byte('a');
  FormatStr(Str, FmtId, Params);
  MoveStr(B, Str, Color);
  WriteLine(0, 1, Size.X, 1, B);

  { Fill in rest of rectangle }
  MoveChar(B, ' ', Color, Size.X);
  WriteLine(0, 2, Size.X, Size.Y-2, B);
end;

function TFileInfoPane.GetPalette: PPalette;
const
  P: String[Length(CInfoPane)] = CInfoPane;
begin
  GetPalette := PPalette(@P);
end;

procedure TFileInfoPane.HandleEvent(var Event: TEvent);
begin
  TView.HandleEvent(Event);
  if (Event.What = evBroadcast) and (Event.Command = cmFileFocused) then
  begin
    S := PSearchRec(Event.InfoPtr)^;
    DrawView;
  end;
end;

{****************************************************************************
              TFileHistory
****************************************************************************}

  function LTrim(const S: String): String;
  var
    I: Sw_Integer;
  begin
    I := 1;
    while (I < Length(S)) and (S[I] = ' ') do Inc(I);
    LTrim := Copy(S, I, 255);
  end;

  function RTrim(const S: String): String;
  var
    I: Sw_Integer;
  begin
    I := Length(S);
    while S[I] = ' ' do Dec(I);
    RTrim := Copy(S, 1, I);
  end;

  function RelativePath(var S: PathStr): Boolean;
  begin
    S := LTrim(RTrim(S));
<<<<<<< HEAD
<<<<<<< HEAD
    {$ifdef HASAMIGA}
    RelativePath := Pos(DriveSeparator, S) = 0;
    {$ELSE}
    RelativePath := not ((S <> '') and ((S[1] = DirSeparator) or (S[2] = ':')));
    {$ENDIF}
=======
    RelativePath := not ((S <> '') and ((S[1] = DirSeparator) or (S[2] = ':')));
>>>>>>> graemeg/fixes_2_2
=======
    RelativePath := not ((S <> '') and ((S[1] = DirSeparator) or (S[2] = ':')));
>>>>>>> origin/fixes_2_2
  end;

{ try to reduce the length of S+dir as a file path+pattern }

  function Simplify (var S,Dir : string) : string;
    var i : sw_integer;
  begin
   if RelativePath(Dir) then
     begin
        if (S<>'') and (Copy(Dir,1,3)='..'+DirSeparator) then
          begin
             i:=Length(S);
             for i:=Length(S)-1 downto 1 do
               if S[i]=DirSeparator then
                 break;
             if S[i]=DirSeparator then
               Simplify:=Copy(S,1,i)+Copy(Dir,4,255)
             else
               Simplify:=S+Dir;
          end
        else
          Simplify:=S+Dir;
     end
   else
      Simplify:=Dir;
  end;

{****************************************************************************}
{ TFileHistory.HandleEvent                                                       }
{****************************************************************************}

procedure TFileHistory.HandleEvent(var Event: TEvent);
var
  HistoryWindow: PHistoryWindow;
  R,P: TRect;
  C: Word;
  Rslt: String;
begin
  TView.HandleEvent(Event);
  if (Event.What = evMouseDown) or
     ((Event.What = evKeyDown) and (CtrlToArrow(Event.KeyCode) = kbDown) and
      (Link^.State and sfFocused <> 0)) then
  begin
    if not Link^.Focus then
    begin
      ClearEvent(Event);
      Exit;
    end;
    if assigned(CurDir) then
     Rslt:=CurDir^
    else
     Rslt:='';
    Rslt:=Simplify(Rslt,Link^.Data^);
    RemoveDoubleDirSep(Rslt);
    If IsWild(Rslt) then
      RecordHistory(Rslt);
    Link^.GetBounds(R);
    Dec(R.A.X); Inc(R.B.X); Inc(R.B.Y,7); Dec(R.A.Y,1);
    Owner^.GetExtent(P);
    R.Intersect(P);
    Dec(R.B.Y,1);
    HistoryWindow := InitHistoryWindow(R);
    if HistoryWindow <> nil then
    begin
      C := Owner^.ExecView(HistoryWindow);
      if C = cmOk then
      begin
        Rslt := HistoryWindow^.GetSelection;
        if Length(Rslt) > Link^.MaxLen then Rslt[0] := Char(Link^.MaxLen);
        Link^.Data^ := Rslt;
        Link^.SelectAll(True);
        Link^.DrawView;
      end;
      Dispose(HistoryWindow, Done);
    end;
    ClearEvent(Event);
  end
  else if (Event.What = evBroadcast) then
    if ((Event.Command = cmReleasedFocus) and (Event.InfoPtr = Link))
      or (Event.Command = cmRecordHistory) then
    begin
      if assigned(CurDir) then
       Rslt:=CurDir^
      else
       Rslt:='';
      Rslt:=Simplify(Rslt,Link^.Data^);
      RemoveDoubleDirSep(Rslt);
      If IsWild(Rslt) then
        RecordHistory(Rslt);
    end;
end;

procedure TFileHistory.AdaptHistoryToDir(Dir : string);
  var S,S2 : String;
      i,Count : Sw_word;
begin
   if assigned(CurDir) then
     begin
        S:=CurDir^;
        if S=Dir then
          exit;
        DisposeStr(CurDir);
     end
   else
     S:='';
   CurDir:=NewStr(Simplify(S,Dir));

   Count:=HistoryCount(HistoryId);
   for i:=1 to count do
     begin
        S2:=HistoryStr(HistoryId,1);
        HistoryRemove(HistoryId,1);
        if RelativePath(S2) then
          if S<>'' then
            S2:=S+S2
          else
            S2:=FExpand(S2);
        { simply full path
          we should simplify relative to Dir ! }
        HistoryAdd(HistoryId,S2);
     end;

end;

destructor TFileHistory.Done;
begin
  If assigned(CurDir) then
    DisposeStr(CurDir);
  Inherited Done;
end;

{****************************************************************************
              TFileDialog
****************************************************************************}

constructor TFileDialog.Init(AWildCard: TWildStr; const ATitle,
  InputName: String; AOptions: Word; HistoryId: Byte);
var
  Control: PView;
  R: TRect;
  Opt: Word;
begin
  R.Assign(15,1,64,20);
  TDialog.Init(R, ATitle);
  Options := Options or ofCentered;
  WildCard := AWildCard;

  R.Assign(3,3,31,4);
  FileName := New(PFileInputLine, Init(R, 79));
  FileName^.Data^ := WildCard;
  Insert(FileName);
  R.Assign(2,2,3+CStrLen(InputName),3);
  Control := New(PLabel, Init(R, InputName, FileName));
  Insert(Control);
  R.Assign(31,3,34,4);
  FileHistory := New(PFileHistory, Init(R, FileName, HistoryId));
  Insert(FileHistory);

  R.Assign(3,14,34,15);
  Control := New(PScrollBar, Init(R));
  Insert(Control);
  R.Assign(3,6,34,14);
  FileList := New(PFileList, Init(R, PScrollBar(Control)));
  Insert(FileList);
  R.Assign(2,5,8,6);
  Control := New(PLabel, Init(R, slFiles, FileList));
  Insert(Control);

  R.Assign(35,3,46,5);
  Opt := bfDefault;
  if AOptions and fdOpenButton <> 0 then
  begin
    Insert(New(PButton, Init(R,slOpen, cmFileOpen, Opt)));
    Opt := bfNormal;
    Inc(R.A.Y,3); Inc(R.B.Y,3);
  end;
  if AOptions and fdOkButton <> 0 then
  begin
    Insert(New(PButton, Init(R,slOk, cmFileOpen, Opt)));
    Opt := bfNormal;
    Inc(R.A.Y,3); Inc(R.B.Y,3);
  end;
  if AOptions and fdReplaceButton <> 0 then
  begin
    Insert(New(PButton, Init(R, slReplace,cmFileReplace, Opt)));
    Opt := bfNormal;
    Inc(R.A.Y,3); Inc(R.B.Y,3);
  end;
  if AOptions and fdClearButton <> 0 then
  begin
    Insert(New(PButton, Init(R, slClear,cmFileClear, Opt)));
    Opt := bfNormal;
    Inc(R.A.Y,3); Inc(R.B.Y,3);
  end;
  Insert(New(PButton, Init(R, slCancel, cmCancel, bfNormal)));
  Inc(R.A.Y,3); Inc(R.B.Y,3);
  if AOptions and fdHelpButton <> 0 then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    //Insert(New(PButton, Init(R,slHelp,cmHelp, bfNormal)));
    //Inc(R.A.Y,3); Inc(R.B.Y,3);
=======
    Insert(New(PButton, Init(R,slHelp,cmHelp, bfNormal)));
    Inc(R.A.Y,3); Inc(R.B.Y,3);
>>>>>>> graemeg/fixes_2_2
=======
    Insert(New(PButton, Init(R,slHelp,cmHelp, bfNormal)));
    Inc(R.A.Y,3); Inc(R.B.Y,3);
>>>>>>> origin/fixes_2_2
  end;

  R.Assign(1,16,48,18);
  Control := New(PFileInfoPane, Init(R));
  Insert(Control);

  SelectNext(False);

  if AOptions and fdNoLoadDir = 0 then ReadDirectory;
end;

constructor TFileDialog.Load(var S: TStream);
begin
  if not TDialog.Load(S) then
    Fail;
  S.Read(WildCard, SizeOf(WildCard));
  if (S.Status <> stOk) then
  begin
    TDialog.Done;
    Fail;
  end;
  GetSubViewPtr(S, FileName);
  GetSubViewPtr(S, FileList);
  GetSubViewPtr(S, FileHistory);
  ReadDirectory;
  if (DosError <> 0) then
  begin
    TDialog.Done;
    Fail;
  end;
end;

destructor TFileDialog.Done;
begin
  DisposeStr(Directory);
  TDialog.Done;
end;

procedure TFileDialog.GetData(var Rec);
begin
  GetFilename(PathStr(Rec));
end;

procedure TFileDialog.GetFileName(var S: PathStr);

var
  Path: PathStr;
  Name: NameStr;
  Ext: ExtStr;
  TWild : string;
  TPath: PathStr;
  TName: NameStr;
  TExt: NameStr;
  i : Sw_integer;
begin
  S := FileName^.Data^;
  if RelativePath(S) then
    begin
      if (Directory <> nil) then
   S := FExpand(Directory^ + S);
    end
  else
    S := FExpand(S);
  if Pos(ListSeparator,S)=0 then
   begin
     If FileExists(S) then
       exit;
     FSplit(S, Path, Name, Ext);
     if ((Name = '') or (Ext = '')) and not IsDir(S) then
     begin
       TWild:=WildCard;
       repeat
    i:=Pos(ListSeparator,TWild);
    if i=0 then
     i:=length(TWild)+1;
    FSplit(Copy(TWild,1,i-1), TPath, TName, TExt);
    if ((Name = '') and (Ext = '')) then
      S := Path + TName + TExt
    else
      if Name = '' then
        S := Path + TName + Ext
      else
        if Ext = '' then
          begin
       if IsWild(Name) then
         S := Path + Name + TExt
       else
         S := Path + Name + NoWildChars(TExt);
          end;
    if FileExists(S) then
     break;
    System.Delete(TWild,1,i);
       until TWild='';
       if TWild='' then
         S := Path + Name + Ext;
     end;
   end;
end;

procedure TFileDialog.HandleEvent(var Event: TEvent);
begin
  if (Event.What and evBroadcast <> 0) and
     (Event.Command = cmListItemSelected) then
  begin
    EndModal(cmFileOpen);
    ClearEvent(Event);
  end;
  TDialog.HandleEvent(Event);
  if Event.What = evCommand then
    case Event.Command of
      cmFileOpen, cmFileReplace, cmFileClear:
   begin
     EndModal(Event.Command);
     ClearEvent(Event);
   end;
    end;
end;

procedure TFileDialog.SetData(var Rec);
begin
  TDialog.SetData(Rec);
  if (PathStr(Rec) <> '') and (IsWild(TWildStr(Rec))) then
  begin
    Valid(cmFileInit);
    FileName^.Select;
  end;
end;

procedure TFileDialog.ReadDirectory;
begin
  FileList^.ReadDirectory(WildCard);
  FileHistory^.AdaptHistoryToDir(GetCurDir);
  Directory := NewStr(GetCurDir);
end;

procedure TFileDialog.Store(var S: TStream);
begin
  TDialog.Store(S);
  S.Write(WildCard, SizeOf(WildCard));
  PutSubViewPtr(S, FileName);
  PutSubViewPtr(S, FileList);
  PutSubViewPtr(S, FileHistory);
end;

function TFileDialog.Valid(Command: Word): Boolean;
var
  FName: PathStr;
  Dir: DirStr;
  Name: NameStr;
  Ext: ExtStr;

  function CheckDirectory(var S: PathStr): Boolean;
  begin
    if not PathValid(S) then
    begin
      MessageBox(sInvalidDriveOrDir, nil, mfError + mfOkButton);
      FileName^.Select;
      CheckDirectory := False;
    end else CheckDirectory := True;
  end;

  function CompleteDir(const Path: string): string;
  begin
    { keep c: untouched PM }
    if (Path<>'') and (Path[Length(Path)]<>DirSeparator) and
       (Path[Length(Path)]<>':') then
     CompleteDir:=Path+DirSeparator
    else
     CompleteDir:=Path;
  end;

  function NormalizeDir(const Path: string): string;
  var Root: boolean;
  begin
    Root:=false;
    {$ifdef Unix}
    if Path=DirSeparator then Root:=true;
    {$else}
<<<<<<< HEAD
<<<<<<< HEAD
    {$ifdef HASAMIGA}
    if Length(Path) > 0 then Root := Path[Length(Path)] = DriveSeparator;
    {$else}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    if (length(Path)=3) and (Upcase(Path[1]) in['A'..'Z']) and
       (Path[2]=':') and (Path[3]=DirSeparator) then
         Root:=true;
    {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
    {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    if (Root=false) and (copy(Path,length(Path),1)=DirSeparator) then
      NormalizeDir:=copy(Path,1,length(Path)-1)
    else
      NormalizeDir:=Path;
  end;
function NormalizeDirF(var S: openstring): boolean;
begin
  S:=NormalizeDir(S);
  NormalizeDirF:=true;
end;

begin
  if Command = 0 then
  begin
    Valid := True;
    Exit;
  end
  else Valid := False;
  if TDialog.Valid(Command) then
  begin
    GetFileName(FName);
    if (Command <> cmCancel) and (Command <> cmFileClear) then
    begin
      if IsWild(FName) or IsList(FName) then
      begin
        FSplit(FName, Dir, Name, Ext);
        if CheckDirectory(Dir) then
        begin
          FileHistory^.AdaptHistoryToDir(Dir);
          DisposeStr(Directory);
          Directory := NewStr(Dir);
          if Pos(ListSeparator,FName)>0 then
           WildCard:=Copy(FName,length(Dir)+1,255)
          else
           WildCard := Name+Ext;
          if Command <> cmFileInit then
            FileList^.Select;
          FileList^.ReadDirectory(Directory^+WildCard);
        end;
      end
    else
      if NormalizeDirF(FName) then
      { ^^ this is just a dummy if construct (the func always returns true,
        it's just there, 'coz I don't want to rearrange the following "if"s... }
      if IsDir(FName) then
        begin
          if CheckDirectory(FName) then
          begin
            FileHistory^.AdaptHistoryToDir(CompleteDir(FName));
            DisposeStr(Directory);
            Directory := NewSTr(CompleteDir(FName));
            if Command <> cmFileInit then FileList^.Select;
            FileList^.ReadDirectory(Directory^+WildCard);
          end
        end
      else
        if ValidFileName(FName) then
          Valid := True
        else
          begin
            MessageBox(^C + sInvalidFileName, nil, mfError + mfOkButton);
            Valid := False;
          end;
    end
    else Valid := True;
  end;
end;

{ TDirCollection }

function TDirCollection.GetItem(var S: TStream): Pointer;
var
  DirItem: PDirEntry;
begin
  New(DirItem);
  DirItem^.DisplayText := S.ReadStr;
  DirItem^.Directory := S.ReadStr;
  GetItem := DirItem;
end;

procedure TDirCollection.FreeItem(Item: Pointer);
var
  DirItem: PDirEntry absolute Item;
begin
  DisposeStr(DirItem^.DisplayText);
  DisposeStr(DirItem^.Directory);
  Dispose(DirItem);
end;

procedure TDirCollection.PutItem(var S: TStream; Item: Pointer);
var
  DirItem: PDirEntry absolute Item;
begin
  S.WriteStr(DirItem^.DisplayText);
  S.WriteStr(DirItem^.Directory);
end;

{ TDirListBox }

const
  DrivesS: String = '';
  Drives: PString = @DrivesS;

constructor TDirListBox.Init(var Bounds: TRect; AScrollBar:
  PScrollBar);
begin
  DrivesS := sDrives;
  TListBox.Init(Bounds, 1, AScrollBar);
  Dir := '';
end;

destructor TDirListBox.Done;
begin
  if (List <> nil) then
    Dispose(List,Done);
  TListBox.Done;
end;

function TDirListBox.GetText(Item,MaxLen: Sw_Integer): String;
begin
  GetText := PDirEntry(List^.At(Item))^.DisplayText^;
end;

procedure TDirListBox.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evMouseDown:
      if Event.Double then
      begin
   Event.What := evCommand;
   Event.Command := cmChangeDir;
   PutEvent(Event);
   ClearEvent(Event);
      end;
    evKeyboard:
      if (Event.CharCode = ' ') and
    (PSearchRec(List^.At(Focused))^.Name = '..') then
   NewDirectory(PSearchRec(List^.At(Focused))^.Name);
  end;
  TListBox.HandleEvent(Event);
end;

function TDirListBox.IsSelected(Item: Sw_Integer): Boolean;
begin
{  IsSelected := Item = Cur; }
  IsSelected := Inherited IsSelected(Item);
end;

procedure TDirListBox.NewDirectory(var ADir: DirStr);
const
  PathDir       = 'ÀÄÂ';
  FirstDir     =   'ÀÂÄ';
  MiddleDir   =   ' ÃÄ';
  LastDir       =   ' ÀÄ';
  IndentSize    = '  ';
var
  AList: PCollection;
  NewDir, Dirct: DirStr;
  C, OldC: Char;
  S, Indent: String[80];
  P: PString;
  NewCur: Word;
  isFirst: Boolean;
  SR: SearchRec;
  I: Sw_Integer;

  function NewDirEntry(const DisplayText, Directory: String): PDirEntry;{$ifdef PPC_BP}near;{$endif}
  var
    DirEntry: PDirEntry;
  begin
    New(DirEntry);
    DirEntry^.DisplayText := NewStr(DisplayText);
    If Directory='' then
      DirEntry^.Directory := NewStr(DirSeparator)
    else
      DirEntry^.Directory := NewStr(Directory);
    NewDirEntry := DirEntry;
  end;

begin
  Dir := ADir;
  AList := New(PDirCollection, Init(5,5));
{$ifdef HAS_DOS_DRIVES}
  AList^.Insert(NewDirEntry(Drives^,Drives^));
  if Dir = Drives^ then
  begin
    isFirst := True;
    OldC := ' ';
    for C := 'A' to 'Z' do
    begin
      if (C < 'C') or DriveValid(C) then
      begin
   if OldC <> ' ' then
   begin
     if isFirst then
     begin
       S := FirstDir + OldC;
       isFirst := False;
     end
     else S := MiddleDir + OldC;
     AList^.Insert(NewDirEntry(S, OldC + ':' + DirSeparator));
   end;
   if C = GetCurDrive then NewCur := AList^.Count;
   OldC := C;
      end;
    end;
    if OldC <> ' ' then
      AList^.Insert(NewDirEntry(LastDir + OldC, OldC + ':' + DirSeparator));
  end
  else
{$endif HAS_DOS_DRIVES}
  begin
    Indent := IndentSize;
    NewDir := Dir;
{$ifdef HAS_DOS_DRIVES}
    Dirct := Copy(NewDir,1,3);
    AList^.Insert(NewDirEntry(PathDir + Dirct, Dirct));
    NewDir := Copy(NewDir,4,255);
{$else HAS_DOS_DRIVES}
    Dirct := '';
{$endif HAS_DOS_DRIVES}
    while NewDir <> '' do
    begin
      I := Pos(DirSeparator,NewDir);
      if I <> 0 then
      begin
   S := Copy(NewDir,1,I-1);
   Dirct := Dirct + S;
   AList^.Insert(NewDirEntry(Indent + PathDir + S, Dirct));
   NewDir := Copy(NewDir,I+1,255);
      end
      else
      begin
   Dirct := Dirct + NewDir;
   AList^.Insert(NewDirEntry(Indent + PathDir + NewDir, Dirct));
   NewDir := '';
      end;
      Indent := Indent + IndentSize;
      Dirct := Dirct + DirSeparator;
    end;
    NewCur := AList^.Count-1;
    isFirst := True;
    NewDir := Dirct + AllFiles;
    FindFirst(NewDir, Directory, SR);
    while DosError = 0 do
    begin
      if (SR.Attr and Directory <> 0) and
         (SR.Name <> '.') and (SR.Name <> '..') then
      begin
   if isFirst then
   begin
     S := FirstDir;
     isFirst := False;
   end else S := MiddleDir;
   AList^.Insert(NewDirEntry(Indent + S + SR.Name, Dirct + SR.Name));
      end;
      FindNext(SR);
    end;
  FindClose(SR);
    P := PDirEntry(AList^.At(AList^.Count-1))^.DisplayText;
    I := Pos('À',P^);
    if I = 0 then
    begin
      I := Pos('Ã',P^);
      if I <> 0 then P^[I] := 'À';
    end else
    begin
      P^[I+1] := 'Ä';
      P^[I+2] := 'Ä';
    end;
  end;
  NewList(AList);
  FocusItem(NewCur);
  Cur:=NewCur;
end;

procedure TDirListBox.SetState(AState: Word; Enable: Boolean);
begin
  TListBox.SetState(AState, Enable);
  if AState and sfFocused <> 0 then
    PChDirDialog(Owner)^.ChDirButton^.MakeDefault(Enable);
end;

{****************************************************************************}
{ TChDirDialog Object                     }
{****************************************************************************}
{****************************************************************************}
{ TChDirDialog.Init                      }
{****************************************************************************}
constructor TChDirDialog.Init(AOptions: Word; HistoryId: Sw_Word);
var
  R: TRect;
  Control: PView;
begin
  R.Assign(16, 2, 64, 20);
  TDialog.Init(R,sChangeDirectory);

  Options := Options or ofCentered;

  R.Assign(3, 3, 30, 4);
  DirInput := New(PInputLine, Init(R, FileNameLen+4));
  Insert(DirInput);
  R.Assign(2, 2, 17, 3);
  Control := New(PLabel, Init(R,slDirectoryName, DirInput));
  Insert(Control);
  R.Assign(30, 3, 33, 4);
  Control := New(PHistory, Init(R, DirInput, HistoryId));
  Insert(Control);

  R.Assign(32, 6, 33, 16);
  Control := New(PScrollBar, Init(R));
  Insert(Control);
  R.Assign(3, 6, 32, 16);
  DirList := New(PDirListBox, Init(R, PScrollBar(Control)));
  Insert(DirList);
  R.Assign(2, 5, 17, 6);
  Control := New(PLabel, Init(R, slDirectoryTree, DirList));
  Insert(Control);

  R.Assign(35, 6, 45, 8);
  OkButton := New(PButton, Init(R, slOk, cmOK, bfDefault));
  Insert(OkButton);
  Inc(R.A.Y,3); Inc(R.B.Y,3);
  ChDirButton := New(PButton,Init(R,slChDir,cmChangeDir,
           bfNormal));
  Insert(ChDirButton);
  Inc(R.A.Y,3); Inc(R.B.Y,3);
  Insert(New(PButton, Init(R,slRevert, cmRevert, bfNormal)));
  if AOptions and cdHelpButton <> 0 then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    //Inc(R.A.Y,3); Inc(R.B.Y,3);
    //Insert(New(PButton, Init(R,slHelp, cmHelp, bfNormal)));
=======
    Inc(R.A.Y,3); Inc(R.B.Y,3);
    Insert(New(PButton, Init(R,slHelp, cmHelp, bfNormal)));
>>>>>>> graemeg/fixes_2_2
=======
    Inc(R.A.Y,3); Inc(R.B.Y,3);
    Insert(New(PButton, Init(R,slHelp, cmHelp, bfNormal)));
>>>>>>> origin/fixes_2_2
  end;

  if AOptions and cdNoLoadDir = 0 then SetUpDialog;

  SelectNext(False);
end;

{****************************************************************************}
{ TChDirDialog.Load                      }
{****************************************************************************}
constructor TChDirDialog.Load(var S: TStream);
begin
  TDialog.Load(S);
  GetSubViewPtr(S, DirList);
  GetSubViewPtr(S, DirInput);
  GetSubViewPtr(S, OkButton);
  GetSubViewPtr(S, ChDirbutton);
  SetUpDialog;
end;

{****************************************************************************}
{ TChDirDialog.DataSize                      }
{****************************************************************************}
function TChDirDialog.DataSize: Sw_Word;
begin
  DataSize := 0;
end;

{****************************************************************************}
{ TChDirDialog.GetData                        }
{****************************************************************************}
procedure TChDirDialog.GetData(var Rec);
begin
end;

{****************************************************************************}
{ TChDirDialog.HandleEvent                   }
{****************************************************************************}
procedure TChDirDialog.HandleEvent(var Event: TEvent);
var
  CurDir: DirStr;
  P: PDirEntry;
begin
  TDialog.HandleEvent(Event);
  case Event.What of
    evCommand:
      begin
   case Event.Command of
     cmRevert: GetDir(0,CurDir);
     cmChangeDir:
       begin
         P := DirList^.List^.At(DirList^.Focused);
         if (P^.Directory^ = Drives^)
            or DriveValid(P^.Directory^[1]) then
           CurDir := P^.Directory^
         else Exit;
       end;
   else
     Exit;
   end;
   if (Length(CurDir) > 3) and
      (CurDir[Length(CurDir)] = DirSeparator) then
     CurDir := Copy(CurDir,1,Length(CurDir)-1);
   DirList^.NewDirectory(CurDir);
   DirInput^.Data^ := CurDir;
   DirInput^.DrawView;
   DirList^.Select;
   ClearEvent(Event);
      end;
  end;
end;

{****************************************************************************}
{ TChDirDialog.SetData                        }
{****************************************************************************}
procedure TChDirDialog.SetData(var Rec);
begin
end;

{****************************************************************************}
{ TChDirDialog.SetUpDialog                   }
{****************************************************************************}
procedure TChDirDialog.SetUpDialog;
var
  CurDir: DirStr;
begin
  if DirList <> nil then
  begin
    CurDir := GetCurDir;
    DirList^.NewDirectory(CurDir);
    if (Length(CurDir) > 3) and (CurDir[Length(CurDir)] = DirSeparator) then
      CurDir := Copy(CurDir,1,Length(CurDir)-1);
    if DirInput <> nil then
    begin
      DirInput^.Data^ := CurDir;
      DirInput^.DrawView;
    end;
  end;
end;

{****************************************************************************}
{ TChDirDialog.Store                    }
{****************************************************************************}
procedure TChDirDialog.Store(var S: TStream);
begin
  TDialog.Store(S);
  PutSubViewPtr(S, DirList);
  PutSubViewPtr(S, DirInput);
  PutSubViewPtr(S, OkButton);
  PutSubViewPtr(S, ChDirButton);
end;

{****************************************************************************}
{ TChDirDialog.Valid                    }
{****************************************************************************}
function TChDirDialog.Valid(Command: Word): Boolean;
var
  P: PathStr;
begin
  Valid := True;
  if Command = cmOk then
  begin
    P := FExpand(DirInput^.Data^);
    if (Length(P) > 3) and (P[Length(P)] = DirSeparator) then
      Dec(P[0]);
<<<<<<< HEAD
<<<<<<< HEAD
    {$push}{$I-}
=======
    {$I-}
>>>>>>> graemeg/fixes_2_2
=======
    {$I-}
>>>>>>> origin/fixes_2_2
    ChDir(P);
    if (IOResult <> 0) then
    begin
      MessageBox(sInvalidDirectory, nil, mfError + mfOkButton);
      Valid := False;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
    {$pop}
=======
    {$I+}
>>>>>>> graemeg/fixes_2_2
=======
    {$I+}
>>>>>>> origin/fixes_2_2
  end;
end;

{****************************************************************************}
{ TEditChDirDialog Object                     }
{****************************************************************************}
{****************************************************************************}
{ TEditChDirDialog.DataSize                    }
{****************************************************************************}
function TEditChDirDialog.DataSize : Sw_Word;
begin
  DataSize := SizeOf(DirStr);
end;

{****************************************************************************}
{ TEditChDirDialog.GetData                   }
{****************************************************************************}
procedure TEditChDirDialog.GetData (var Rec);
var
  CurDir : DirStr absolute Rec;
begin
  if (DirInput = nil) then
    CurDir := ''
  else begin
    CurDir := DirInput^.Data^;
<<<<<<< HEAD
<<<<<<< HEAD
    
    if (CurDir[Length(CurDir)] <> DirSeparator) 
    {$IFDEF HASAMIGA}
      and (CurDir[Length(CurDir)] <> DriveSeparator) 
    {$endif}
    then
=======
    if (CurDir[Length(CurDir)] <> DirSeparator) then
>>>>>>> graemeg/fixes_2_2
=======
    if (CurDir[Length(CurDir)] <> DirSeparator) then
>>>>>>> origin/fixes_2_2
      CurDir := CurDir + DirSeparator;
  end;
end;

{****************************************************************************}
{ TEditChDirDialog.SetData                   }
{****************************************************************************}
procedure TEditChDirDialog.SetData (var Rec);
var
  CurDir : DirStr absolute Rec;
begin
  if DirList <> nil then
  begin
    DirList^.NewDirectory(CurDir);
    if DirInput <> nil then
    begin
      if (Length(CurDir) > 3) and (CurDir[Length(CurDir)] = DirSeparator) then
   DirInput^.Data^ := Copy(CurDir,1,Length(CurDir)-1)
      else DirInput^.Data^ := CurDir;
      DirInput^.DrawView;
    end;
  end;
end;

{****************************************************************************}
{ TSortedListBox Object                      }
{****************************************************************************}
{****************************************************************************}
{ TSortedListBox.Init                     }
{****************************************************************************}
constructor TSortedListBox.Init(var Bounds: TRect; ANumCols: Sw_Word;
  AScrollBar: PScrollBar);
begin
  TListBox.Init(Bounds, ANumCols, AScrollBar);
  SearchPos := 0;
  ShowCursor;
  SetCursor(1,0);
end;

{****************************************************************************}
{ TSortedListBox.HandleEvent                  }
{****************************************************************************}
procedure TSortedListBox.HandleEvent(var Event: TEvent);
const
  SpecialChars: set of Char = [#0,#9,#27];
var
  CurString, NewString: String;
  K: Pointer;
  Value : Sw_integer;
  OldPos, OldValue: Sw_Integer;
  T: Boolean;
begin
  OldValue := Focused;
  TListBox.HandleEvent(Event);
  if (OldValue <> Focused) or
     ((Event.What = evBroadcast) and (Event.InfoPtr = @Self) and
      (Event.Command = cmReleasedFocus)) then
    SearchPos := 0;
  if Event.What = evKeyDown then
  begin
    { patched to prevent error when no or empty list or Escape pressed }
    if (not (Event.CharCode in SpecialChars)) and
       (List <> nil) and (List^.Count > 0) then
    begin
      Value := Focused;
      if Value < Range then
        CurString := GetText(Value, 255)
      else
        CurString := '';
      OldPos := SearchPos;
      if Event.KeyCode = kbBack then
      begin
   if SearchPos = 0 then Exit;
   Dec(SearchPos);
          if SearchPos = 0 then
            HandleDir:= ((GetShiftState and $3) <> 0) or (Event.CharCode in ['A'..'Z']);
   CurString[0] := Char(SearchPos);
      end
      else if (Event.CharCode = '.') then
        SearchPos := Pos('.',CurString)
      else
      begin
   Inc(SearchPos);
          if SearchPos = 1 then
            HandleDir := ((GetShiftState and 3) <> 0) or (Event.CharCode in ['A'..'Z']);
   CurString[0] := Char(SearchPos);
   CurString[SearchPos] := Event.CharCode;
      end;
      K := GetKey(CurString);
      T := PSortedCollection(List)^.Search(K, Value);
      if Value < Range then
      begin
          if Value < Range then
            NewString := GetText(Value, 255)
          else
            NewString := '';
   if Equal(NewString, CurString, SearchPos) then
   begin
     if Value <> OldValue then
     begin
       FocusItem(Value);
       { Assumes ListControl will set the cursor to the first character }
       { of the sfFocused item }
       SetCursor(Cursor.X+SearchPos, Cursor.Y);
     end
              else
                SetCursor(Cursor.X+(SearchPos-OldPos), Cursor.Y);
   end
          else
            SearchPos := OldPos;
      end
      else SearchPos := OldPos;
      if (SearchPos <> OldPos) or (Event.CharCode in ['A'..'Z','a'..'z']) then
   ClearEvent(Event);
    end;
  end;
end;

function TSortedListBox.GetKey(var S: String): Pointer;
begin
  GetKey := @S;
end;

procedure TSortedListBox.NewList(AList: PCollection);
begin
  TListBox.NewList(AList);
  SearchPos := 0;
end;

{****************************************************************************}
{            Global Procedures and Functions          }
{****************************************************************************}

{****************************************************************************}
{ Contains                          }
{****************************************************************************}
function Contains(S1, S2: String): Boolean;
  { Contains returns true if S1 contains any characters in S2. }
var
  i : Byte;
begin
  Contains := True;
  i := 1;
  while ((i < Length(S2)) and (i < Length(S1))) do
    if (Upcase(S1[i]) = Upcase(S2[i])) then
      Exit
    else Inc(i);
  Contains := False;
end;

{****************************************************************************}
{ StdDeleteFile                           }
{****************************************************************************}
function StdDeleteFile (AFile : FNameStr) : Boolean;
var
  Rec : PStringRec;
begin
  if CheckOnDelete then
  begin
    AFile := ShrinkPath(AFile,33);
    Rec.AString := PString(@AFile);
    StdDeleteFile := (MessageBox(^C + sDeleteFile,
               @Rec,mfConfirmation or mfOkCancel) = cmOk);
  end
  else StdDeleteFile := False;
end;

{****************************************************************************}
{ DriveValid                         }
{****************************************************************************}
function DriveValid(Drive: Char): Boolean;
{$ifdef HAS_DOS_DRIVES}
var
  D: Char;
begin
  D := GetCurDrive;
<<<<<<< HEAD
<<<<<<< HEAD
  {$push}{$I-}
=======
  {$I-}
>>>>>>> graemeg/fixes_2_2
=======
  {$I-}
>>>>>>> origin/fixes_2_2
  ChDir(Drive+':');
  if (IOResult = 0) then
  begin
    DriveValid := True;
    ChDir(D+':')
  end
  else DriveValid := False;
<<<<<<< HEAD
<<<<<<< HEAD
  {$pop}
=======
  {$I+}
>>>>>>> graemeg/fixes_2_2
=======
  {$I+}
>>>>>>> origin/fixes_2_2
end;
{$else HAS_DOS_DRIVES}
begin
  DriveValid:=true;
end;
{$endif HAS_DOS_DRIVES}

{****************************************************************************}
{ Equal                             }
{****************************************************************************}
function Equal(const S1, S2: String; Count: Sw_word): Boolean;
var
  i: Sw_Word;
begin
  Equal := False;
  if (Length(S1) < Count) or (Length(S2) < Count) then
    Exit;
  for i := 1 to Count do
    if UpCase(S1[I]) <> UpCase(S2[I]) then
      Exit;
  Equal := True;
end;

{****************************************************************************}
{ ExtractDir                         }
{****************************************************************************}
function ExtractDir(AFile: FNameStr): DirStr;
  { ExtractDir returns the path of AFile terminated with a trailing '\'.  If
    AFile contains no directory information, an empty string is returned. }
var
  D: DirStr;
  N: NameStr;
  E: ExtStr;
begin
  FSplit(AFile,D,N,E);
  if D = '' then
  begin
    ExtractDir := '';
    Exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  if (D[Byte(D[0])] <> DirSeparator)
  {$ifdef HASAMIGA}
    and (D[Byte(D[0])] <> DriveSeparator)
  {$endif}
  then
=======
  if D[Byte(D[0])] <> DirSeparator then
>>>>>>> graemeg/fixes_2_2
=======
  if D[Byte(D[0])] <> DirSeparator then
>>>>>>> origin/fixes_2_2
    D := D + DirSeparator;
  ExtractDir := D;
end;

{****************************************************************************}
{ ExtractFileName                       }
{****************************************************************************}
function ExtractFileName(AFile: FNameStr): NameStr;
var
  D: DirStr;
  N: NameStr;
  E: ExtStr;
begin
  FSplit(AFile,D,N,E);
  ExtractFileName := N;
end;

{****************************************************************************}
{ FileExists                         }
{****************************************************************************}
function FileExists (AFile : FNameStr) : Boolean;
begin
  FileExists := (FSearch(AFile,'') <> '');
end;

{****************************************************************************}
{ GetCurDir                        }
{****************************************************************************}
function GetCurDir: DirStr;
var
  CurDir: DirStr;
begin
  GetDir(0, CurDir);
  if (Length(CurDir) > 3) then
  begin
    Inc(CurDir[0]);
    CurDir[Length(CurDir)] := DirSeparator;
  end;
  GetCurDir := CurDir;
end;

{****************************************************************************}
{ GetCurDrive                       }
{****************************************************************************}
function GetCurDrive: Char;
{$ifdef go32v2}
var
  Regs : Registers;
begin
  Regs.AH := $19;
  Intr($21,Regs);
  GetCurDrive := Char(Regs.AL + Byte('A'));
end;
{$else not go32v2}
var
  D : DirStr;
begin
  D:=GetCurDir;
  if (Length(D)>1) and (D[2]=':') then
    begin
      if (D[1]>='a') and (D[1]<='z') then
        GetCurDrive:=Char(Byte(D[1])+Byte('A')-Byte('a'))
      else
        GetCurDrive:=D[1];
    end
  else
    GetCurDrive:='C';
end;
{$endif not go32v2}

{****************************************************************************}
{ IsDir                             }
{****************************************************************************}
function IsDir(const S: String): Boolean;
var
  SR: SearchRec;
  Is: boolean;
begin
  Is:=false;
{$ifdef Unix}
  Is:=(S=DirSeparator); { handle root }
{$else}
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef HASAMIGA}
  Is := (Length(S) > 0) and (S[Length(S)] = DriveSeparator);
  {$else}
  Is:=(length(S)=3) and (Upcase(S[1]) in['A'..'Z']) and (S[2]=':') and (S[3]=DirSeparator);
  {$endif}
=======
  Is:=(length(S)=3) and (Upcase(S[1]) in['A'..'Z']) and (S[2]=':') and (S[3]=DirSeparator);
>>>>>>> graemeg/fixes_2_2
=======
  Is:=(length(S)=3) and (Upcase(S[1]) in['A'..'Z']) and (S[2]=':') and (S[3]=DirSeparator);
>>>>>>> origin/fixes_2_2
  { handle root dirs }
{$endif}
  if Is=false then
  begin
    FindFirst(S, Directory, SR);
    if DosError = 0 then
      Is := (SR.Attr and Directory) <> 0
    else
      Is := False;
   {$ifdef fpc}
    FindClose(SR);
   {$endif}
  end;
  IsDir:=Is;
end;

{****************************************************************************}
{ IsWild                           }
{****************************************************************************}
function IsWild(const S: String): Boolean;
begin
  IsWild := (Pos('?',S) > 0) or (Pos('*',S) > 0);
end;

{****************************************************************************}
{ IsList                           }
{****************************************************************************}
function IsList(const S: String): Boolean;
begin
  IsList := (Pos(ListSeparator,S) > 0);
end;

{****************************************************************************}
{ MakeResources                           }
{****************************************************************************}
(*
procedure MakeResources;
var
  Dlg : PDialog;
  Key : String;
  i : Word;
begin
  for i := 0 to 1 do
  begin
    case i of
      0 : begin
       Key := reOpenDlg;
       Dlg := New(PFileDialog,Init('*.*',sOpen,slName,
             fdOkButton or fdHelpButton or fdNoLoadDir,0));
     end;
      1 : begin
       Key := reSaveAsDlg;
       Dlg := New(PFileDialog,Init('*.*',sSaveAs,slName,
             fdOkButton or fdHelpButton or fdNoLoadDir,0));
     end;
      2 : begin
       Key := reEditChDirDialog;
       Dlg := New(PEditChDirDialog,Init(cdHelpButton,
             hiCurrentDirectories));
     end;
    end;
    if Dlg = nil then
    begin
       PrintStr('Error initializing dialog ' + Key);
       Halt;
    end
    else begin
      RezFile^.Put(Dlg,Key);
      if (RezFile^.Stream^.Status <> stOk) then
      begin
   PrintStr('Error writing dialog ' + Key + ' to the resource file.');
   Halt;
      end;
    end;
  end;
end;
*)
{****************************************************************************}
{ NoWildChars                       }
{****************************************************************************}
function NoWildChars(S: String): String;
const
  WildChars : array[0..1] of Char = ('?','*');
var
  i : Sw_Word;
begin
  repeat
    i := Pos('?',S);
    if (i > 0) then
      System.Delete(S,i,1);
  until (i = 0);
  repeat
    i := Pos('*',S);
    if (i > 0) then
      System.Delete(S,i,1);
  until (i = 0);
  NoWildChars:=S;
end;

{****************************************************************************}
{ OpenFile                          }
{****************************************************************************}
function OpenFile (var AFile : FNameStr; HistoryID : Byte) : Boolean;
var
  Dlg : PFileDialog;
begin
  {$ifdef cdResource}
  Dlg := PFileDialog(RezFile^.Get(reOpenDlg));
  {$else}
  Dlg := New(PFileDialog,Init('*.*',sOpen,slName,
        fdOkButton or fdHelpButton,0));
  {$endif cdResource}
    { this might not work }
  PHistory(Dlg^.FileName^.Next^.Next)^.HistoryID := HistoryID;
  OpenFile := (Application^.ExecuteDialog(Dlg,@AFile) = cmFileOpen);
end;

{****************************************************************************}
{ OpenNewFile                       }
{****************************************************************************}
function OpenNewFile (var AFile: FNameStr; HistoryID: Byte): Boolean;
  { OpenNewFile allows the user to select a directory from disk and enter a
    new file name.  If the file name entered is an existing file the user is
    optionally prompted for confirmation of replacing the file based on the
    value in #CheckOnReplace#.  If a file name is successfully entered,
    OpenNewFile returns True. }
  {#X OpenFile }
begin
  OpenNewFile := False;
  if OpenFile(AFile,HistoryID) then
  begin
    if not ValidFileName(AFile) then
      Exit;
    if FileExists(AFile) then
      if (not CheckOnReplace) or (not ReplaceFile(AFile)) then
   Exit;
    OpenNewFile := True;
  end;
end;

{****************************************************************************}
{ RegisterStdDlg                         }
{****************************************************************************}
procedure RegisterStdDlg;
begin
  RegisterType(RFileInputLine);
  RegisterType(RFileCollection);
  RegisterType(RFileList);
  RegisterType(RFileInfoPane);
  RegisterType(RFileDialog);
  RegisterType(RDirCollection);
  RegisterType(RDirListBox);
  RegisterType(RSortedListBox);
  RegisterType(RChDirDialog);
end;

{****************************************************************************}
{ StdReplaceFile                         }
{****************************************************************************}
function StdReplaceFile (AFile : FNameStr) : Boolean;
var
  Rec : PStringRec;
begin
  if CheckOnReplace then
  begin
    AFile := ShrinkPath(AFile,33);
    Rec.AString := PString(@AFile);
    StdReplaceFile :=
       (MessageBox(^C + sReplaceFile,
         @Rec,mfConfirmation or mfOkCancel) = cmOk);
  end
  else StdReplaceFile := True;
end;

{****************************************************************************}
{ SaveAs                           }
{****************************************************************************}
function SaveAs (var AFile : FNameStr; HistoryID : Word) : Boolean;
var
  Dlg : PFileDialog;
begin
  SaveAs := False;
  Dlg := New(PFileDialog,Init('*.*',sSaveAs,slSaveAs,
        fdOkButton or fdHelpButton,0));
    { this might not work }
  PHistory(Dlg^.FileName^.Next^.Next)^.HistoryID := HistoryID;
  Dlg^.HelpCtx := hcSaveAs;
  if (Application^.ExecuteDialog(Dlg,@AFile) = cmFileOpen) and
     ((not FileExists(AFile)) or ReplaceFile(AFile)) then
    SaveAs := True;
end;

{****************************************************************************}
{ SelectDir                        }
{****************************************************************************}
function SelectDir (var ADir : DirStr; HistoryID : Byte) : Boolean;
var
  Dir: DirStr;
  Dlg : PEditChDirDialog;
  Rec : DirStr;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  {$push}{$I-}
  GetDir(0,Dir);
  {$pop}
=======
  {$I-}
  GetDir(0,Dir);
  {$I+}
>>>>>>> graemeg/fixes_2_2
=======
  {$I-}
  GetDir(0,Dir);
  {$I+}
>>>>>>> origin/fixes_2_2
  Rec := FExpand(ADir);
  Dlg := New(PEditChDirDialog,Init(cdHelpButton,HistoryID));
  if (Application^.ExecuteDialog(Dlg,@Rec) = cmOk) then
  begin
    SelectDir := True;
    ADir := Rec;
  end
  else SelectDir := False;
<<<<<<< HEAD
<<<<<<< HEAD
  {$push}{$I-}
  ChDir(Dir);
  {$pop}
=======
  {$I-}
  ChDir(Dir);
  {$I+}
>>>>>>> graemeg/fixes_2_2
=======
  {$I-}
  ChDir(Dir);
  {$I+}
>>>>>>> origin/fixes_2_2
end;

{****************************************************************************}
{ ShrinkPath                         }
{****************************************************************************}
function ShrinkPath (AFile : FNameStr; MaxLen : Byte) : FNameStr;
var
  Filler: string;
  D1 : DirStr;
  N1 : NameStr;
  E1 : ExtStr;
  i  : Sw_Word;

begin
  if Length(AFile) > MaxLen then
  begin
    FSplit(FExpand(AFile),D1,N1,E1);
    AFile := Copy(D1,1,3) + '..' + DirSeparator;
    i := Pred(Length(D1));
    while (i > 0) and (D1[i] <> DirSeparator) do
      Dec(i);
    if (i = 0) then
      AFile := AFile + D1
    else AFile := AFile + Copy(D1,Succ(i),Length(D1)-i);
<<<<<<< HEAD
<<<<<<< HEAD
    if (AFile[Length(AFile)] <> DirSeparator)
    {$ifdef HASAMIGA}
      and (AFile[Length(AFile)] <> DriveSeparator)
    {$endif}
    then
=======
    if AFile[Length(AFile)] <> DirSeparator then
>>>>>>> graemeg/fixes_2_2
=======
    if AFile[Length(AFile)] <> DirSeparator then
>>>>>>> origin/fixes_2_2
      AFile := AFile + DirSeparator;
    if Length(AFile)+Length(N1)+Length(E1) <= MaxLen then
      AFile := AFile + N1 + E1
    else
      begin
        Filler := '...' + DirSeparator;
        AFile:=Copy(Afile,1,MaxLen-Length(Filler)-Length(N1)-Length(E1))
                +Filler+N1+E1;
      end;
  end;
  ShrinkPath := AFile;
end;

{****************************************************************************}
{ ValidFileName                           }
{****************************************************************************}
function ValidFileName(var FileName: PathStr): Boolean;
var
  IllegalChars: string[12];
  Dir: DirStr;
  Name: NameStr;
  Ext: ExtStr;
begin
{$ifdef PPC_FPC}
{$ifdef go32v2}
  { spaces are allowed if LFN is supported }
  if LFNSupport then
    IllegalChars := ';,=+<>|"[]'+DirSeparator
  else
    IllegalChars := ';,=+<>|"[] '+DirSeparator;
{$else not go32v2}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef OS_WINDOWS}
    IllegalChars := ';,=+<>|"[]'+DirSeparator;
{$else not go32v2 and not OS_WINDOWS }
    IllegalChars := ';,=+<>|"[] '+DirSeparator;
{$endif not OS_WINDOWS}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef win32}
    IllegalChars := ';,=+<>|"[]'+DirSeparator;
{$else not go32v2 and not win32 }
    IllegalChars := ';,=+<>|"[] '+DirSeparator;
{$endif not win32}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif not go32v2}
{$else not PPC_FPC}
  IllegalChars := ';,=+<>|"[] '+DirSeparator;
{$endif PPC_FPC}
  ValidFileName := True;
  FSplit(FileName, Dir, Name, Ext);
  if not ((Dir = '') or PathValid(Dir)) or
     Contains(Name, IllegalChars) or
     Contains(Dir, IllegalChars) then
    ValidFileName := False;
end;

{****************************************************************************}
{        Unit Initialization Section                                         }
{****************************************************************************}
begin
{$ifdef PPC_BP}
  ReplaceFile := StdReplaceFile;
  DeleteFile := StdDeleteFile;
{$else}
  ReplaceFile := @StdReplaceFile;
  DeleteFile := @StdDeleteFile;
{$endif PPC_BP}
end.
