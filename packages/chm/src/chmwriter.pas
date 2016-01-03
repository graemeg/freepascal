{ Copyright (C) <2005> <Andrew Haines> chmwriter.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
<<<<<<< HEAD
<<<<<<< HEAD
  along with this library; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02111-1301, USA.
=======
  along with this library; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
  along with this library; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
{
  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.
}
unit chmwriter;
{$MODE OBJFPC}{$H+}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ $DEFINE LZX_USETHREADS}

interface
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses Classes, ChmBase, chmtypes, chmspecialfiles, HtmlIndexer, chmsitemap, contnrs, StreamEx, Avl_Tree, lzxcompressthread;
=======
uses Classes, ChmBase, chmtypes, chmspecialfiles, HtmlIndexer, chmsitemap, contnrs, Avl_Tree{$IFDEF LZX_USETHREADS}, lzxcompressthread{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
uses Classes, ChmBase, chmtypes, chmspecialfiles, HtmlIndexer, chmsitemap, contnrs, Avl_Tree{$IFDEF LZX_USETHREADS}, lzxcompressthread{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
uses Classes, ChmBase, chmtypes, chmspecialfiles, HtmlIndexer, chmsitemap, contnrs, Avl_Tree{$IFDEF LZX_USETHREADS}, lzxcompressthread{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
uses Classes, ChmBase, chmtypes, chmspecialfiles, HtmlIndexer, chmsitemap, contnrs, Avl_Tree{$IFDEF LZX_USETHREADS}, lzxcompressthread{$ENDIF};
>>>>>>> origin/cpstrnew

Const
   DefaultHHC = 'Default.hhc';
   DefaultHHK = 'Default.hhk';

Type
=======
=======
>>>>>>> origin/fixes_2_2
=======
{ $DEFINE LZX_USETHREADS}
>>>>>>> origin/fixes_2.4

interface
uses Classes, ChmBase, chmtypes, chmspecialfiles, HtmlIndexer, chmsitemap, Avl_Tree{$IFDEF LZX_USETHREADS}, lzxcompressthread{$ENDIF};

type

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  TGetDataFunc = function (const DataName: String; out PathInChm: String; out FileName: String; var Stream: TStream): Boolean of object;
  //  DataName :  A FileName or whatever so that the getter can find and open the file to add
  //  PathInChm:  This is the absolute path in the archive. i.e. /home/user/helpstuff/
  //              becomes '/' and /home/user/helpstuff/subfolder/ > /subfolder/
  //  FileName :  /home/user/helpstuff/index.html > index.html
  //  Stream   :  the file opened with DataName should be written to this stream

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Type
   TStringIndex = Class    // AVLTree needs wrapping in non automated reference type also used in filewriter.
=======
Type
   TStringIndex = Class    // AVLTree needs wrapping in non automated reference type
>>>>>>> origin/fixes_2.4
                      TheString : String;
                      StrId     : Integer;
                    end;
   TUrlStrIndex = Class
                      UrlStr    : String;
                      UrlStrId  : Integer;
                    end;
<<<<<<< HEAD

  { TITSFWriter }

  TITSFWriter = class(TObject)
    FOnLastFile: TNotifyEvent;
  private
    ForceExit: Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

  { TChmWriter }

  TChmWriter = class(TObject)
    FOnLastFile: TNotifyEvent;
  private
    FHasBinaryTOC: Boolean;
    FHasBinaryIndex: Boolean;
    ForceExit: Boolean;

    FDefaultFont: String;
    FDefaultPage: String;
    FFullTextSearch: Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FInternalFiles: TFileEntryList; // Contains a complete list of files in the chm including
    FFrameSize: LongWord;           // uncompressed files and special internal files of the chm
    FCurrentStream: TStream; // used to buffer the files that are to be compressed
    FCurrentIndex: Integer;
    FOnGetFileData: TGetDataFunc;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
    FSearchTitlesOnly: Boolean;
    FStringsStream: TMemoryStream; // the #STRINGS file
    FTopicsStream: TMemoryStream;  // the #TOPICS file
    FURLTBLStream: TMemoryStream;  // the #URLTBL file. has offsets of strings in URLSTR
    FURLSTRStream: TMemoryStream;  // the #URLSTR file
    FFiftiMainStream: TMemoryStream;
    FContextStream: TMemoryStream; // the #IVB file
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FSection0: TMemoryStream;
    FSection1: TStream; // Compressed Stream
    FSection1Size: QWord;
    FSection1ResetTable: TMemoryStream; // has a list of frame positions NOT window positions
    FDirectoryListings: TStream;
    FOutStream: TStream;
    FFileNames: TStrings;
    FDestroyStream: Boolean;
    FTempStream: TStream;
    FPostStream: TStream;
<<<<<<< HEAD
<<<<<<< HEAD
    FWindowSize: LongWord;
    FReadCompressedSize: QWord; // Current Size of Uncompressed data that went in Section1 (compressed)
    FPostStreamActive: Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
    FTitle: String;
    FHasTOC: Boolean;
    FHasIndex: Boolean;
    FWindowSize: LongWord;
    FReadCompressedSize: QWord; // Current Size of Uncompressed data that went in Section1 (compressed)
    FIndexedFiles: TIndexedWordList;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    FPostStreamActive: Boolean;
>>>>>>> origin/fixes_2.4
    // Linear order of file
    ITSFHeader: TITSFHeader;
    HeaderSection0Table: TITSFHeaderEntry;  // points to HeaderSection0
    HeaderSection1Table: TITSFHeaderEntry; // points to HeaderSection1
    HeaderSuffix: TITSFHeaderSuffix; //contains the offset of CONTENTSection0 from zero
    HeaderSection0: TITSPHeaderPrefix;
    HeaderSection1: TITSPHeader; // DirectoryListings header
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FReadmeMessage : String;
    FCores	: integer;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    FAvlStrings   : TAVLTree;    // dedupe strings
    FAvlURLStr    : TAVLTree;    // dedupe urltbl + binindex must resolve URL to topicid
    SpareString   : TStringIndex;
    SpareUrlStr   : TUrlStrIndex;
>>>>>>> origin/fixes_2.4
    // DirectoryListings
    // CONTENT Section 0 (section 1 is contained in section 0)
    // EOF
    // end linear header parts
    procedure InitITSFHeader;
    procedure InitHeaderSectionTable;
    procedure SetTempRawStream(const AValue: TStream);
    procedure WriteHeader(Stream: TStream);
    procedure CreateDirectoryListings;
    procedure WriteDirectoryListings(Stream: TStream);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure WriteInternalFilesBefore; virtual;
    procedure WriteInternalFilesAfter; virtual;
    procedure StartCompressingStream;
    procedure WriteREADMEFile;
    procedure WriteFinalCompressedFiles; virtual;
    procedure WriteSection0;
    procedure WriteSection1;
    procedure WriteDataSpaceFiles(const AStream: TStream);

    procedure FileAdded(AStream: TStream; const AEntry: TFileEntryRec); virtual;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure StartCompressingStream;
    procedure WriteSYSTEM;
    procedure WriteITBITS;
    procedure WriteSTRINGS;
    procedure WriteTOPICS;
    procedure WriteIVB; // context ids
    procedure WriteURL_STR_TBL;
    procedure WriteOBJINST;
    procedure WriteFiftiMain;
    procedure WriteREADMEFile;
    procedure WriteFinalCompressedFiles;
    procedure WriteSection0;
    procedure WriteSection1;
    procedure WriteDataSpaceFiles(const AStream: TStream);
    function AddString(AString: String): LongWord;
    function AddURL(AURL: String; TopicsIndex: DWord): LongWord;
    procedure CheckFileMakeSearchable(AStream: TStream; AFileEntry: TFileEntryRec);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    function AddTopic(ATitle,AnUrl:AnsiString):integer;
    function NextTopicIndex: Integer;
>>>>>>> origin/fixes_2.4
    // callbacks for lzxcomp
    function  AtEndOfData: Longbool;
    function  GetData(Count: LongInt; Buffer: PByte): LongInt;
    function  WriteCompressedData(Count: Longint; Buffer: Pointer): LongInt;
    procedure MarkFrame(UnCompressedTotal, CompressedTotal: LongWord);
    // end callbacks
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    {$IFDEF LZX_USETHREADS}
>>>>>>> graemeg/cpstrnew
=======
    {$IFDEF LZX_USETHREADS}
>>>>>>> graemeg/cpstrnew
=======
    {$IFDEF LZX_USETHREADS}
>>>>>>> graemeg/cpstrnew
=======
    {$IFDEF LZX_USETHREADS}
>>>>>>> origin/cpstrnew
=======
    {$IFDEF LZX_USETHREADS}
>>>>>>> origin/fixes_2.4
    // callbacks for lzx compress threads
    function  LTGetData(Sender: TLZXCompressor; WantedByteCount: Integer; Buffer: Pointer): Integer;
    function  LTIsEndOfFile(Sender: TLZXCompressor): Boolean;
    procedure LTChunkDone(Sender: TLZXCompressor; CompressedSize: Integer; UncompressedSize: Integer; Buffer: Pointer);
    procedure LTMarkFrame(Sender: TLZXCompressor; CompressedTotal: Integer; UncompressedTotal: Integer);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    {$ENDIF}
>>>>>>> graemeg/cpstrnew
=======
    {$ENDIF}
>>>>>>> graemeg/cpstrnew
=======
    {$ENDIF}
>>>>>>> graemeg/cpstrnew
=======
    {$ENDIF}
>>>>>>> origin/cpstrnew
    // end callbacks
  public
    constructor Create(AOutStream: TStream; FreeStreamOnDestroy: Boolean); virtual;
    destructor Destroy; override;
    procedure Execute;
    procedure AddStreamToArchive(AFileName, APath: String; AStream: TStream; Compress: Boolean = True);
    procedure PostAddStreamToArchive(AFileName, APath: String; AStream: TStream; Compress: Boolean = True);
=======
=======
>>>>>>> origin/fixes_2_2
=======
    {$ENDIF}
    // end callbacks
>>>>>>> origin/fixes_2.4
  public
    constructor Create(OutStream: TStream; FreeStreamOnDestroy: Boolean);
    destructor Destroy; override;
    procedure Execute;
    procedure AppendTOC(AStream: TStream);
    procedure AppendBinaryTOCFromSiteMap(ASiteMap: TChmSiteMap);
    procedure AppendBinaryIndexFromSiteMap(ASiteMap: TChmSiteMap;chw:boolean);
    procedure AppendBinaryTOCStream(AStream: TStream);
    procedure AppendBinaryIndexStream(IndexStream,DataStream,MapStream,Propertystream: TStream;chw:boolean);
    procedure AppendIndex(AStream: TStream);
    procedure AppendSearchDB(AName: String; AStream: TStream);
    procedure AddStreamToArchive(AFileName, APath: String; AStream: TStream; Compress: Boolean = True);
    procedure PostAddStreamToArchive(AFileName, APath: String; AStream: TStream; Compress: Boolean = True);
    procedure AddContext(AContext: DWord; ATopic: String);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property WindowSize: LongWord read FWindowSize write FWindowSize default 2; // in $8000 blocks
    property FrameSize: LongWord read FFrameSize write FFrameSize default 1; // in $8000 blocks
    property FilesToCompress: TStrings read FFileNames;
    property OnGetFileData: TGetDataFunc read FOnGetFileData write FOnGetFileData;
    property OnLastFile: TNotifyEvent read FOnLastFile write FOnLastFile;
    property OutStream: TStream read FOutStream;
<<<<<<< HEAD
<<<<<<< HEAD
    property TempRawStream: TStream read FTempStream write SetTempRawStream;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property ReadmeMessage : String read fReadmeMessage write fReadmeMessage;
    property Cores : integer read fcores write fcores;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    //property LocaleID: dword read ITSFHeader.LanguageID write ITSFHeader.LanguageID;
  end;

  { TChmWriter }

  TChmWriter = class(TITSFWriter)
  private
    FHasBinaryTOC: Boolean;
    FHasBinaryIndex: Boolean;
    FDefaultFont: String;
    FDefaultPage: String;
    FFullTextSearch: Boolean;
    FFullTextSearchAvailable: Boolean;
    FSearchTitlesOnly: Boolean;
    FStringsStream: TMemoryStream; // the #STRINGS file
    FTopicsStream: TMemoryStream;  // the #TOPICS file
    FURLTBLStream: TMemoryStream;  // the #URLTBL file. has offsets of strings in URLSTR
    FURLSTRStream: TMemoryStream;  // the #URLSTR file
    FFiftiMainStream: TMemoryStream;
    FContextStream: TMemoryStream; // the #IVB file
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FIDXHdrStream : TMemoryStream; // the #IDXHDR and chunk 13 in #SYSTEM
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FTitle: String;
    FHasTOC: Boolean;
    FHasIndex: Boolean;
    FIndexedFiles: TIndexedWordList;
    FAvlStrings   : TAVLTree;    // dedupe strings
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FAVLTopicdedupe : TAVlTree;  // Topic deduping, if we load it both from hhp and TOC
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FAvlURLStr    : TAVLTree;    // dedupe urltbl + binindex must resolve URL to topicid
    SpareString   : TStringIndex;
    SpareUrlStr   : TUrlStrIndex;
    FWindows      : TObjectList;
    FDefaultWindow: String;
    FTocName      : String;
    FIndexName    : String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FMergeFiles   : TStringList;
    FTocSM        : TCHMSitemap;
    FHasKLinks    : Boolean;
    FNrTopics     : Integer;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  protected
    procedure FileAdded(AStream: TStream; const AEntry: TFileEntryRec); override;
  private
    procedure WriteInternalFilesBefore; override;
    procedure WriteInternalFilesAfter; override;
    procedure WriteFinalCompressedFiles; override;
    procedure WriteSYSTEM;
    procedure WriteITBITS;
    procedure WriteSTRINGS;
    procedure WriteTOPICS;
    procedure WriteIVB; // context ids
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure CreateIDXHDRStream;
    procedure WriteIDXHDR;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure WriteURL_STR_TBL;
    procedure WriteOBJINST;
    procedure WriteFiftiMain;
    procedure WriteWindows;

    function AddString(AString: String): LongWord;
    function AddURL(AURL: String; TopicsIndex: DWord): LongWord;
    procedure CheckFileMakeSearchable(AStream: TStream; AFileEntry: TFileEntryRec);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function AddTopic(ATitle,AnUrl:AnsiString;code:integer=-1):integer;
    procedure ScanSitemap(asitemap:TCHMSiteMap);
    function NextTopicIndex: Integer;
    procedure Setwindows (AWindowList:TObjectList);
    procedure SetMergefiles(src:TStringList);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function AddTopic(ATitle,AnUrl:AnsiString):integer;
    function NextTopicIndex: Integer;
    procedure Setwindows (AWindowList:TObjectList);

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
  public
    constructor Create(AOutStream: TStream; FreeStreamOnDestroy: Boolean); override;
    destructor Destroy; override;
    procedure AppendTOC(AStream: TStream);
    procedure AppendBinaryTOCFromSiteMap(ASiteMap: TChmSiteMap);
    procedure AppendBinaryIndexFromSiteMap(ASiteMap: TChmSiteMap;chw:boolean);
    procedure AppendBinaryTOCStream(AStream: TStream);
    procedure AppendBinaryIndexStream(IndexStream,DataStream,MapStream,Propertystream: TStream;chw:boolean);
    procedure AppendIndex(AStream: TStream);
    procedure AppendSearchDB(AName: String; AStream: TStream);
    procedure AddContext(AContext: DWord; ATopic: String);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure AddDummyALink;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    property Title: String read FTitle write FTitle;
    property FullTextSearch: Boolean read FFullTextSearch write FFullTextSearch;
    property SearchTitlesOnly: Boolean read FSearchTitlesOnly write FSearchTitlesOnly;
    property HasBinaryTOC: Boolean read FHasBinaryTOC write FHasBinaryTOC;
    property HasBinaryIndex: Boolean read FHasBinaryIndex write FHasBinaryIndex;
    property DefaultFont: String read FDefaultFont write FDefaultFont;
    property DefaultPage: String read FDefaultPage write FDefaultPage;
    property Windows : TObjectlist read fwindows write setwindows;
    property TOCName : String read FTocName write FTocName;
    property IndexName : String read FIndexName write FIndexName;
    property DefaultWindow : string read fdefaultwindow write fdefaultwindow;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property MergeFiles :TStringList read FMergeFiles write setmergefiles;
    property Tocsitemap :TChmSitemap read ftocsm write ftocsm;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;

Function CompareStrings(Node1, Node2: Pointer): integer; // also used in filewriter

=======
=======
>>>>>>> origin/fixes_2_2
    property Title: String read FTitle write FTitle;
    property FullTextSearch: Boolean read FFullTextSearch write FFullTextSearch;
    property SearchTitlesOnly: Boolean read FSearchTitlesOnly write FSearchTitlesOnly;
    property HasBinaryTOC: Boolean read FHasBinaryTOC write FHasBinaryTOC;
    property HasBinaryIndex: Boolean read FHasBinaryIndex write FHasBinaryIndex;
    property DefaultFont: String read FDefaultFont write FDefaultFont;
    property DefaultPage: String read FDefaultPage write FDefaultPage;
    property TempRawStream: TStream read FTempStream write SetTempRawStream;
    //property LocaleID: dword read ITSFHeader.LanguageID write ITSFHeader.LanguageID;
  end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
implementation
uses dateutils, sysutils, paslzxcomp, chmFiftiMain;

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  LZX_WINDOW_SIZE = 16; // 16 = 2 frames = 1 shl 16
  LZX_FRAME_SIZE = $8000;

  {$ifdef binindex}
    procedure logentry(s:string);
    begin
      Writeln(s);
      flush(stdout);
    end;
  {$endif}
{$I chmobjinstconst.inc}


Function CompareStrings(Node1, Node2: Pointer): integer;
var n1,n2 : TStringIndex;
begin
  n1:=TStringIndex(Node1); n2:=TStringIndex(Node2);
  Result := CompareText(n1.TheString, n2.TheString);
  if Result < 0 then Result := -1
  else if Result > 0 then Result := 1;
end;


Function CompareUrlStrs(Node1, Node2: Pointer): integer;
var n1,n2 : TUrlStrIndex;
begin
  n1:=TUrlStrIndex(Node1); n2:=TUrlStrIndex(Node2);
  Result := CompareText(n1.UrlStr, n2.UrlStr);
  if Result < 0 then Result := -1
  else if Result > 0 then Result := 1;
end;

{ TChmWriter }

procedure TITSFWriter.InitITSFHeader;
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
  LZX_WINDOW_SIZE = 16; // 16 = 2 frames = 1 shl 16
  LZX_FRAME_SIZE = $8000;

  {$ifdef binindex}
    procedure logentry(s:string);
    begin
      Writeln(s);
      flush(stdout);     
    end;
  {$endif}
{$I chmobjinstconst.inc}


Function CompareStrings(Node1, Node2: Pointer): integer;
var n1,n2 : TStringIndex;
begin
  n1:=TStringIndex(Node1); n2:=TStringIndex(Node2);
  Result := CompareText(n1.TheString, n2.TheString);
  if Result < 0 then Result := -1
  else if Result > 0 then Result := 1;
end;


Function CompareUrlStrs(Node1, Node2: Pointer): integer;
var n1,n2 : TUrlStrIndex;
begin
  n1:=TUrlStrIndex(Node1); n2:=TUrlStrIndex(Node2);
  Result := CompareText(n1.UrlStr, n2.UrlStr);
  if Result < 0 then Result := -1
  else if Result > 0 then Result := 1;
end;

{ TChmWriter }

procedure TChmWriter.InitITSFHeader;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  with ITSFHeader do begin
    ITSFsig := ITSFFileSig;
    Version := NToLE(DWord(3));
    // we fix endian order when this is written to the stream
<<<<<<< HEAD
<<<<<<< HEAD
    HeaderLength := NToLE(DWord(SizeOf(TITSFHeader) + (SizeOf(TGuid)*2)+ (SizeOf(TITSFHeaderEntry)*2) + SizeOf(TITSFHeaderSuffix)));
    Unknown_1 := NToLE(DWord(1));
    TimeStamp:= NToBE(MilliSecondOfTheDay(Now)); //bigendian
    LanguageID := NToLE(DWord($0409)); // English / English_US
  end;
end;

procedure TITSFWriter.InitHeaderSectionTable;
=======
=======
>>>>>>> origin/fixes_2_2
    HeaderLength := NToLE(DWord(SizeOf(TITSFHeader) + (SizeOf(TITSFHeaderEntry)*2) + SizeOf(TITSFHeaderSuffix)));
    Unknown_1 := NToLE(DWord(1));
    TimeStamp:= NToBE(MilliSecondOfTheDay(Now)); //bigendian
    LanguageID := NToLE(DWord($0409)); // English / English_US
    Guid1 := ITSFHeaderGUID;
    Guid2 := ITSFHeaderGUID;
  end;
end;

procedure TChmWriter.InitHeaderSectionTable;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  // header section 0
  HeaderSection0Table.PosFromZero := LEToN(ITSFHeader.HeaderLength);
  HeaderSection0Table.Length := SizeOf(TITSPHeaderPrefix);
  // header section 1
  HeaderSection1Table.PosFromZero := HeaderSection0Table.PosFromZero + HeaderSection0Table.Length;
  HeaderSection1Table.Length := SizeOf(TITSPHeader)+FDirectoryListings.Size;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  //contains the offset of CONTENT Section0 from zero
  HeaderSuffix.Offset := HeaderSection1Table.PosFromZero + HeaderSection1Table.Length;

=======
=======
>>>>>>> origin/fixes_2_2
  
  //contains the offset of CONTENT Section0 from zero
  HeaderSuffix.Offset := HeaderSection1Table.PosFromZero + HeaderSection1Table.Length;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

  //contains the offset of CONTENT Section0 from zero
  HeaderSuffix.Offset := HeaderSection1Table.PosFromZero + HeaderSection1Table.Length;

>>>>>>> origin/fixes_2.4
  // now fix endian stuff
  HeaderSection0Table.PosFromZero := NToLE(HeaderSection0Table.PosFromZero);
  HeaderSection0Table.Length := NToLE(HeaderSection0Table.Length);
  HeaderSection1Table.PosFromZero := NToLE(HeaderSection1Table.PosFromZero);
  HeaderSection1Table.Length := NToLE(HeaderSection1Table.Length);

  with HeaderSection0 do begin // TITSPHeaderPrefix;
    Unknown1 := NToLE(DWord($01FE));
    Unknown2 := 0;
    // at this point we are putting together the headers. content sections 0 and 1 are complete
    FileSize := NToLE(HeaderSuffix.Offset + FSection0.Size + FSection1Size);
    Unknown3 := 0;
    Unknown4 := 0;
  end;
  with HeaderSection1 do begin // TITSPHeader; // DirectoryListings header
    ITSPsig := ITSPHeaderSig;
    Version := NToLE(DWord(1));
    DirHeaderLength := NToLE(DWord(SizeOf(TITSPHeader)));  // Length of the directory header
    Unknown1 := NToLE(DWord($0A));
    ChunkSize := NToLE(DWord($1000));
    Density := NToLE(DWord(2));
    // updated when directory listings were created
    //IndexTreeDepth := 1 ; // 1 if there is no index 2 if there is one level of PMGI chunks. will update as
    //IndexOfRootChunk := -1;// if no root chunk
    //FirstPMGLChunkIndex,
    //LastPMGLChunkIndex: LongWord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
    
>>>>>>> graemeg/fixes_2_2
=======
    
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
    Unknown2 := NToLE(Longint(-1));
    //DirectoryChunkCount: LongWord;
    LanguageID := NToLE(DWord($0409));
    GUID := ITSPHeaderGUID;
    LengthAgain := NToLE(DWord($54));
    Unknown3 := NToLE(Longint(-1));
    Unknown4 := NToLE(Longint(-1));
    Unknown5 := NToLE(Longint(-1));
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  // more endian stuff
  HeaderSuffix.Offset := NToLE(HeaderSuffix.Offset);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TITSFWriter.SetTempRawStream(const AValue: TStream);
=======
procedure TChmWriter.SetTempRawStream(const AValue: TStream);
>>>>>>> graemeg/fixes_2_2
=======
procedure TChmWriter.SetTempRawStream(const AValue: TStream);
>>>>>>> origin/fixes_2_2
begin
  if (FCurrentStream.Size > 0) or (FSection1.Size > 0) then
    raise Exception.Create('Cannot set the TempRawStream once data has been written to it!');
  if AValue = nil then
    raise Exception.Create('TempRawStream cannot be nil!');
  if FCurrentStream = AValue then
    exit;
  FCurrentStream.Free;
  FCurrentStream := AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TITSFWriter.WriteHeader(Stream: TStream);
begin
  Stream.Write(ITSFHeader, SizeOf(TITSFHeader));

  if ITSFHeader.Version < 4 then
  begin
    Stream.Write(ITSFHeaderGUID, SizeOf(TGuid));
    Stream.Write(ITSFHeaderGUID, SizeOf(TGuid));
  end;
=======
procedure TChmWriter.WriteHeader(Stream: TStream);
begin
  Stream.Write(ITSFHeader, SizeOf(TITSFHeader));
>>>>>>> graemeg/fixes_2_2
=======
procedure TChmWriter.WriteHeader(Stream: TStream);
begin
  Stream.Write(ITSFHeader, SizeOf(TITSFHeader));
>>>>>>> origin/fixes_2_2
  Stream.Write(HeaderSection0Table, SizeOf(TITSFHeaderEntry));
  Stream.Write(HeaderSection1Table, SizeOf(TITSFHeaderEntry));
  Stream.Write(HeaderSuffix, SizeOf(TITSFHeaderSuffix));
  Stream.Write(HeaderSection0, SizeOf(TITSPHeaderPrefix));

end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TITSFWriter.CreateDirectoryListings;
=======
procedure TChmWriter.CreateDirectoryListings;
>>>>>>> graemeg/fixes_2_2
=======
procedure TChmWriter.CreateDirectoryListings;
>>>>>>> origin/fixes_2_2
type
  TFirstListEntry = record
    Entry: array[0..511] of byte;
    Size: Integer;
  end;
var
  Buffer: array [0..511] of Byte;
  IndexBlock: TPMGIDirectoryChunk;
  ListingBlock: TDirectoryChunk;
  I: Integer;
  Size: Integer;
  FESize: Integer;
  FileName: String;
  FileNameSize: Integer;
  LastListIndex: Integer;
  FirstListEntry: TFirstListEntry;
  ChunkIndex: Integer;
  ListHeader: TPMGListChunk;
const
  PMGL = 'PMGL';
  PMGI = 'PMGI';
  procedure UpdateLastListChunk;
  var
    Tmp: QWord;
  begin
    if ChunkIndex < 1 then begin
      Exit;
    end;
    Tmp := FDirectoryListings.Position;
    FDirectoryListings.Position := (LastListIndex) * $1000;
    FDirectoryListings.Read(ListHeader, SizeOf(TPMGListChunk));
    FDirectoryListings.Position := (LastListIndex) * $1000;
    ListHeader.NextChunkIndex := NToLE(ChunkIndex);
    FDirectoryListings.Write(ListHeader, SizeOf(TPMGListChunk));
    FDirectoryListings.Position := Tmp;
  end;
  procedure WriteIndexChunk(ShouldFinish: Boolean = False);
  var
    IndexHeader: TPMGIIndexChunk;
    ParentIndex,
    TmpIndex: TPMGIDirectoryChunk;
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    with IndexHeader do
    begin
=======
    with IndexHeader do begin
>>>>>>> graemeg/fixes_2_2
=======
    with IndexHeader do begin
>>>>>>> origin/fixes_2_2
      PMGIsig := PMGI;
      UnusedSpace := NToLE(IndexBlock.FreeSpace);
    end;
    IndexBlock.WriteHeader(@IndexHeader);
    IndexBlock.WriteChunkToStream(FDirectoryListings, ChunkIndex, ShouldFinish);
    IndexBlock.Clear;
    if HeaderSection1.IndexOfRootChunk < 0 then HeaderSection1.IndexOfRootChunk := ChunkIndex;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    if ShouldFinish then
    begin
      HeaderSection1.IndexTreeDepth := 2;
      ParentIndex := IndexBlock.ParentChunk;
      if ParentIndex <> nil then
      repeat // the parent index is notified by our child index when to write
=======
=======
>>>>>>> origin/fixes_2_2
    if ShouldFinish then begin;
      HeaderSection1.IndexTreeDepth := 2;
      ParentIndex := IndexBlock.ParentChunk;
      if ParentIndex <> nil then repeat // the parent index is notified by our child index when to write
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        HeaderSection1.IndexOfRootChunk := ChunkIndex;
        TmpIndex := ParentIndex;
        ParentIndex := ParentIndex.ParentChunk;
        TmpIndex.Free;
        Inc(HeaderSection1.IndexTreeDepth);
        Inc(ChunkIndex);
      until ParentIndex = nil;
    end;
    Inc(ChunkIndex);

  end;
  procedure WriteListChunk;
  begin
    with ListHeader do begin
      PMGLsig := PMGL;
      UnusedSpace := NToLE(ListingBlock.FreeSpace);
      Unknown1 :=  0;
      PreviousChunkIndex := NToLE(LastListIndex);
      NextChunkIndex := NToLE(Longint(-1)); // we update this when we write the next chunk
    end;
    if HeaderSection1.FirstPMGLChunkIndex <= 0 then
      HeaderSection1.FirstPMGLChunkIndex := NToLE(ChunkIndex);
    HeaderSection1.LastPMGLChunkIndex := NToLE(ChunkIndex);
    ListingBlock.WriteHeader(@ListHeader);
    ListingBlock.WriteChunkToStream(FDirectoryListings);
    ListingBlock.Clear;
    UpdateLastListChunk;

    LastListIndex := ChunkIndex;
    Inc(ChunkIndex);
    // now add to index
    if not IndexBlock.CanHold(FirstListEntry.Size) then
      WriteIndexChunk;
    IndexBlock.WriteEntry(FirstListEntry.Size, @FirstListEntry.Entry[0])
  end;
begin
  // first sort the listings
  FInternalFiles.Sort;
  HeaderSection1.IndexTreeDepth := 1;
  HeaderSection1.IndexOfRootChunk := -1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  ChunkIndex := 0;

  IndexBlock := TPMGIDirectoryChunk.Create(SizeOf(TPMGIIndexChunk));
  ListingBlock := TDirectoryChunk.Create(SizeOf(TPMGListChunk));

  LastListIndex  := -1;

  // add files to a pmgl block until it is full.
  // after the block is full make a pmgi block and add the first entry of the pmgl block
  // repeat until the index block is full and start another.
  // the pmgi chunks take care of needed parent chunks in the tree
  for I := 0 to FInternalFiles.Count-1 do begin
    Size := 0;
    FileName := FInternalFiles.FileEntry[I].Path + FInternalFiles.FileEntry[I].Name;
    FileNameSize := Length(FileName);
    // filename length
    Inc(Size, WriteCompressedInteger(@Buffer[Size], FileNameSize));
    // filename
    Move(FileName[1], Buffer[Size], FileNameSize);
    Inc(Size, FileNameSize);
    FESize := Size;
    // File is compressed...
    Inc(Size, WriteCompressedInteger(@Buffer[Size], Ord(FInternalFiles.FileEntry[I].Compressed)));
    // Offset from section start
    Inc(Size, WriteCompressedInteger(@Buffer[Size], FInternalFiles.FileEntry[I].DecompressedOffset));
    // Size when uncompressed
    Inc(Size, WriteCompressedInteger(@Buffer[Size], FInternalFiles.FileEntry[I].DecompressedSize));

    if not ListingBlock.CanHold(Size) then
      WriteListChunk;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    ListingBlock.WriteEntry(Size, @Buffer[0]);

=======
    
    ListingBlock.WriteEntry(Size, @Buffer[0]);
    
>>>>>>> graemeg/fixes_2_2
=======
    
    ListingBlock.WriteEntry(Size, @Buffer[0]);
    
>>>>>>> origin/fixes_2_2
=======

    ListingBlock.WriteEntry(Size, @Buffer[0]);

>>>>>>> origin/fixes_2.4
    if ListingBlock.ItemCount = 1 then begin // add the first list item to the index
      Move(Buffer[0], FirstListEntry.Entry[0], FESize);
      FirstListEntry.Size := FESize + WriteCompressedInteger(@FirstListEntry.Entry[FESize], ChunkIndex);
    end;
  end;
  if ListingBlock.ItemCount > 0 then WriteListChunk;

  if ChunkIndex > 1 then begin
    if (IndexBlock.ItemCount > 1)
    or ( (IndexBlock.ItemCount > 0) and (HeaderSection1.IndexOfRootChunk > -1) )
    then WriteIndexChunk(True);
  end;

  HeaderSection1.DirectoryChunkCount := NToLE(DWord(FDirectoryListings.Size div $1000));

  IndexBlock.Free;
  ListingBlock.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  //now fix some endian stuff
  HeaderSection1.IndexOfRootChunk := NToLE(HeaderSection1.IndexOfRootChunk);
  HeaderSection1.IndexTreeDepth := NtoLE(HeaderSection1.IndexTreeDepth);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TITSFWriter.WriteDirectoryListings(Stream: TStream);
=======
procedure TChmWriter.WriteDirectoryListings(Stream: TStream);
>>>>>>> graemeg/fixes_2_2
=======
procedure TChmWriter.WriteDirectoryListings(Stream: TStream);
>>>>>>> origin/fixes_2_2
begin
  Stream.Write(HeaderSection1, SizeOf(HeaderSection1));
  FDirectoryListings.Position := 0;
  Stream.CopyFrom(FDirectoryListings, FDirectoryListings.Size);
  FDirectoryListings.Position := 0;
  //TMemoryStream(FDirectoryListings).SaveToFile('dirlistings.pmg');
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TITSFWriter.WriteInternalFilesBefore;
begin
  // written to Section0 (uncompressed)
  WriteREADMEFile;
end;

procedure TITSFWriter.WriteInternalFilesAfter;
begin
end;

procedure IterateWord(aword:TIndexedWord;State:pointer);
var i,cnt : integer;
begin
  cnt:=pinteger(state)^;
  for i := 0 to AWord.DocumentCount-1 do
    Inc(cnt, AWord.GetLogicalDocument(i).NumberOfIndexEntries);
          // was commented in original procedure, seems to list index entries per doc.
            //WriteLn(AWord.TheWord,'             documents = ', AWord.DocumentCount, ' h
  pinteger(state)^:=cnt;
end;

procedure TITSFWriter.WriteREADMEFile;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const DISCLAIMER_STR = 'This archive was not made by the MS HTML Help Workshop(r)(tm) program, but by Free Pascal''s chm package '+chmpackageversion+'.'#13#10;
=======
const DISCLAIMER_STR = 'This archive was not made by the MS HTML Help Workshop(r)(tm) program.';
>>>>>>> graemeg/cpstrnew
=======
const DISCLAIMER_STR = 'This archive was not made by the MS HTML Help Workshop(r)(tm) program.';
>>>>>>> graemeg/cpstrnew
=======
const DISCLAIMER_STR = 'This archive was not made by the MS HTML Help Workshop(r)(tm) program.';
>>>>>>> graemeg/cpstrnew
=======
const DISCLAIMER_STR = 'This archive was not made by the MS HTML Help Workshop(r)(tm) program.';
>>>>>>> origin/cpstrnew
var
  Entry: TFileEntryRec;
begin
  // This procedure puts a file in the archive that says it wasn't compiled with the MS compiler
  Entry.Compressed := False;
  Entry.DecompressedOffset := FSection0.Position;
  FSection0.Write(DISCLAIMER_STR, SizeOf(DISCLAIMER_STR));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if length(FReadmeMessage)>0 then
    FSection0.Write(FReadmeMessage[1], length(FReadmeMessage));
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Entry.DecompressedSize := FSection0.Position - Entry.DecompressedOffset;
  Entry.Path := '/';
  Entry.Name := '_#_README_#_'; //try to use a name that won't conflict with normal names
  FInternalFiles.AddEntry(Entry);
end;

procedure TITSFWriter.WriteFinalCompressedFiles;
begin

end;


procedure TITSFWriter.WriteSection0;
begin
  FSection0.Position := 0;
  FOutStream.CopyFrom(FSection0, FSection0.Size);
end;

procedure TITSFWriter.WriteSection1;
begin
  WriteContentToStream(FOutStream, FSection1);
end;

procedure TITSFWriter.WriteDataSpaceFiles(const AStream: TStream);
var
  Entry: TFileEntryRec;
begin
  // This procedure will write all files starting with ::
  Entry.Compressed := False; // None of these files are compressed

  //  ::DataSpace/NameList
=======
=======
>>>>>>> origin/fixes_2_2
procedure TChmWriter.WriteSystem;
var
  Entry: TFileEntryRec;
  TmpStr: String;
  TmpTitle: String;
const
  VersionStr = 'HHA Version 4.74.8702'; // does this matter?
begin


  // this creates the /#SYSTEM file
  Entry.Name := '#SYSTEM';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset := FSection0.Position;

 { if FileExists('#SYSTEM') then
  begin
    TmpStream := TMemoryStream.Create;
    TmpStream.LoadFromFile('#SYSTEM');
    TmpStream.Position := 0;
    FSection0.CopyFrom(TmpStream, TmpStream.Size);
  end;                                    }
  // EntryCodeOrder: 10 9 4 2 3 16 6 0 1 5
  FSection0.WriteDWord(NToLE(Word(3))); // Version
  if Title <> '' then
    TmpTitle := Title
  else
    TmpTitle := 'default';

  // Code -> Length -> Data
  // 10
  FSection0.WriteWord(NToLE(Word(10)));
  FSection0.WriteWord(NToLE(Word(SizeOf(DWord))));
  FSection0.WriteDWord(NToLE(MilliSecondOfTheDay(Now)));
  // 9
  FSection0.WriteWord(NToLE(Word(9)));
  FSection0.WriteWord(NToLE(Word(SizeOf(VersionStr)+1)));
  FSection0.Write(VersionStr, SizeOf(VersionStr));
  FSection0.WriteByte(0);
  // 4 A struct that is only needed to set if full text search is on.
  FSection0.WriteWord(NToLE(Word(4)));
  FSection0.WriteWord(NToLE(Word(36))); // size

  FSection0.WriteDWord(NToLE(DWord($0409)));
  FSection0.WriteDWord(1);
  FSection0.WriteDWord(NToLE(DWord(Ord(FFullTextSearch))));
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  // two for a QWord
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);




  ////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // 2  default page to load
  if FDefaultPage <> '' then begin
    FSection0.WriteWord(NToLE(Word(2)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultPage)+1)));
    FSection0.Write(FDefaultPage[1], Length(FDefaultPage));
    FSection0.WriteByte(0);
  end;
  // 3  Title
  if FTitle <> '' then begin
    FSection0.WriteWord(NToLE(Word(3)));
    FSection0.WriteWord(NToLE(Word(Length(FTitle)+1)));
    FSection0.Write(FTitle[1], Length(FTitle));
    FSection0.WriteByte(0);
  end;

  // 16 Default Font
  if FDefaultFont <> '' then begin
    FSection0.WriteWord(NToLE(Word(16)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultFont)+1)));
    FSection0.Write(FDefaultFont[1], Length(FDefaultFont));
    FSection0.WriteByte(0);
  end;

  // 6
  // unneeded. if output file is :  /somepath/OutFile.chm the value here is outfile(lowercase)
  {FSection0.WriteWord(6);
  FSection0.WriteWord(Length('test1')+1);
  Fsection0.Write('test1', 5);
  FSection0.WriteByte(0);}

  // 0 Table of contents filename
  if FHasTOC then begin
    TmpStr := 'default.hhc';
    FSection0.WriteWord(0);
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 1
  // hhk Index
  if FHasIndex then begin
    TmpStr := 'default.hhk';
    FSection0.WriteWord(NToLE(Word(1)));
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 5 Default Window.
  // Not likely needed
// }
  Entry.DecompressedSize := FSection0.Position - Entry.DecompressedOffset;
  FInternalFiles.AddEntry(Entry);

  // 7 Binary Index
  if FHasBinaryIndex then
  begin
    {$ifdef binindex}
      logentry('binary index!');
    {$endif}
    FSection0.WriteWord(NToLE(Word(7)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;

  // 11 Binary TOC
  if FHasBinaryTOC then
  begin
    FSection0.WriteWord(NToLE(Word(11)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;
end;

procedure TChmWriter.WriteITBITS;
var
  Entry: TFileEntryRec;
begin
  // This is an empty and useless file
  Entry.Name := '#ITBITS';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset :=0;// FSection0.Position;
  Entry.DecompressedSize := 0;

  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteSTRINGS;
begin
  if FStringsStream.Size = 0 then;
    FStringsStream.WriteByte(0);
  FStringsStream.Position := 0;
  PostAddStreamToArchive('#STRINGS', '/', FStringsStream);
end;

<<<<<<< HEAD
=======
procedure IterateWord(aword:TIndexedWord;State:pointer);
var i,cnt : integer;
begin
  cnt:=pinteger(state)^;
  for i := 0 to AWord.DocumentCount-1 do
    Inc(cnt, AWord.GetLogicalDocument(i).NumberOfIndexEntries);
          // was commented in original procedure, seems to list index entries per doc.
            //WriteLn(AWord.TheWord,'             documents = ', AWord.DocumentCount, ' h
  pinteger(state)^:=cnt;
end;

>>>>>>> origin/fixes_2.4
procedure TChmWriter.WriteTOPICS;
var
  FHits: Integer;
  i: Integer;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  FTopicsStream.Position := 0;
  PostAddStreamToArchive('#TOPICS', '/', FTopicsStream);
<<<<<<< HEAD

  AWord := FIndexedFiles.FirstWord;
  while AWord <> nil do
  begin
    FHits := 0;
    for i := 0 to AWord.DocumentCount-1 do
    begin
      Inc(FHits, Length(AWord.GetLogicalDocument(i).WordIndex));
    //if AWord.IsTitle then

    end;
    //WriteLn(AWord.TheWord,'             documents = ', AWord.DocumentCount, ' hits = ', FHits, ' is title = ', AWord.IsTitle);
    AWord := AWord.NextWord;
  end;


=======
 // I commented the code below since the result seemed unused
 // FHits:=0;
 //   FIndexedFiles.ForEach(@IterateWord,FHits);
>>>>>>> origin/fixes_2.4
end;

procedure TChmWriter.WriteIVB;
begin
  if FContextStream = nil then exit;

  FContextStream.Position := 0;
  // the size of all the entries
  FContextStream.WriteDWord(NToLE(DWord(FContextStream.Size-SizeOf(dword))));

  FContextStream.Position := 0;
  AddStreamToArchive('#IVB', '/', FContextStream);
end;

procedure TChmWriter.WriteURL_STR_TBL;
begin
  if FURLSTRStream.Size <> 0 then begin
    FURLSTRStream.Position := 0;
    PostAddStreamToArchive('#URLSTR', '/', FURLSTRStream);
  end;
  if FURLTBLStream.Size <> 0 then begin
    FURLTBLStream.Position := 0;
    PostAddStreamToArchive('#URLTBL', '/', FURLTBLStream);
  end;
end;

procedure TChmWriter.WriteOBJINST;
var
  i: Integer;
  ObjStream: TMemoryStream;
  //Flags: Word;
begin
  ObjStream := TMemorystream.Create;
  // this file is needed to enable searches for the ms reader
  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(Dword(2))); // two entries

  ObjStream.WriteDWord(NtoLE(DWord(24))); // offset into file of entry
  ObjStream.WriteDWord(NtoLE(DWord(2691))); // size

  ObjStream.WriteDWord(NtoLE(DWord(2715))); // offset into file of entry
  ObjStream.WriteDWord(NtoLE(DWord(36))); // size

  // first entry
  // write guid 4662DAAF-D393-11D0-9A56-00C04FB68BF7
  ObjStream.WriteDWord(NtoLE($4662DAAF));
  ObjStream.WriteWord(NtoLE($D393));
  ObjStream.WriteWord(NtoLE($11D0));
  ObjStream.WriteWord(NtoLE($569A));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($F7);

  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(11));  // bit flags
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE($00000000));
  ObjStream.WriteDWord(NtoLE($00000000));
  ObjStream.WriteDWord(NtoLE($00145555));
  ObjStream.WriteDWord(NtoLE($00000A0F));
  ObjStream.WriteWord(NtoLE($0100));
  ObjStream.WriteDWord(NtoLE($00030005));
  for i := 0 to 5 do
    ObjStream.WriteDWord($00000000);
  ObjStream.WriteWord($0000);
  // okay now the fun stuff
  for i := 0 to $FF do
  ObjStream.Write(ObjInstEntries[i], SizeOF(TObjInstEntry));
  {begin
    if i = 1 then
      Flags := 7
    else
      Flags := 0;
    if (i >= $41) and (i <= $5A) then
      Flags := Flags or 2;
    if (i >= $61) and (i <= $7A) then
      Flags := Flags or 1;
    if i = $27 then
      Flags := Flags or 6;
    ObjStream.WriteWord(NtoLE(Flags));
    ObjStream.WriteWord(NtoLE(Word(i)));
    if (i >= $41) and (i <= $5A) then
      ObjStream.WriteByte(NtoLE(i+$20))
    else
      ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteWord(NtoLE($0000));
  end;}
  ObjStream.WriteDWord(NtoLE($E66561C6));
  ObjStream.WriteDWord(NtoLE($73DF6561));
  ObjStream.WriteDWord(NtoLE($656F8C73));
  ObjStream.WriteWord(NtoLE($6F9C));
  ObjStream.WriteByte(NtoLE($65));
  // third bit of second entry
  // write guid 8FA0D5A8-DEDF-11D0-9A61-00C04FB68BF7
  ObjStream.WriteDWord(NtoLE($8FA0D5A8));
  ObjStream.WriteWord(NtoLE($DEDF));
  ObjStream.WriteWord(NtoLE($11D0));
  ObjStream.WriteWord(NtoLE($619A));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($F7);

  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(DWord(1)));
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE(DWord(0)));

  // second entry
  // write guid 4662DAB0-D393-11D0-9A56-00C04FB68B66
  ObjStream.WriteDWord(NtoLE($4662DAB0));
  ObjStream.WriteWord(NtoLE($D393));
  ObjStream.WriteWord(NtoLE($11D0));
  ObjStream.WriteWord(NtoLE($569A));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($66);

  ObjStream.WriteDWord(NtoLE(DWord(666))); // not kidding
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE(DWord(10031)));
  ObjStream.WriteDWord(NtoLE(DWord(0)));

  ObjStream.Position := 0;
  AddStreamToArchive('$OBJINST', '/', ObjStream, True);
  ObjStream.Free;

end;

procedure TChmWriter.WriteFiftiMain;
var
  SearchWriter: TChmSearchWriter;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  SearchWriter := TChmSearchWriter.Create(FFiftiMainStream, FIndexedFiles);
  SearchWriter.WriteToStream;
  SearchWriter.Free;

  if FFiftiMainStream.Size = 0 then
    Exit;

  FFiftiMainStream.Position := 0;
  PostAddStreamToArchive('$FIftiMain', '/', FFiftiMainStream);
end;

procedure TChmWriter.WriteREADMEFile;
const DISCLAIMER_STR = 'This archive was not made by the MS HTML Help Workshop(r)(tm) program.';
var
  Entry: TFileEntryRec;
begin
  // This procedure puts a file in the archive that says it wasn't compiled with the MS compiler
  Entry.Compressed := False;
  Entry.DecompressedOffset := FSection0.Position;
  FSection0.Write(DISCLAIMER_STR, SizeOf(DISCLAIMER_STR));
  Entry.DecompressedSize := FSection0.Position - Entry.DecompressedOffset;
  Entry.Path := '/';
  Entry.Name := '_#_README_#_'; //try to use a name that won't conflict with normal names
  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteFinalCompressedFiles;
begin
  WriteTOPICS;
  WriteURL_STR_TBL;
  WriteSTRINGS;
  WriteFiftiMain;
end;


procedure TChmWriter.WriteSection0;
begin
  FSection0.Position := 0;
  FOutStream.CopyFrom(FSection0, FSection0.Size);
end;

procedure TChmWriter.WriteSection1;
begin
  WriteContentToStream(FOutStream, FSection1);
end;

procedure TChmWriter.WriteDataSpaceFiles(const AStream: TStream);
var
  Entry: TFileEntryRec;
begin
  // This procedure will write all files starting with ::
  Entry.Compressed := False; // None of these files are compressed

  //  ::DataSpace/NameList
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Entry.DecompressedOffset := FSection0.Position;
  Entry.DecompressedSize := WriteNameListToStream(FSection0, [snUnCompressed,snMSCompressed]);
  Entry.Path := '::DataSpace/';
  Entry.Name := 'NameList';
  FInternalFiles.AddEntry(Entry, False);

  //  ::DataSpace/Storage/MSCompressed/ControlData
  Entry.DecompressedOffset := FSection0.Position;
  Entry.DecompressedSize := WriteControlDataToStream(FSection0, 2, 2, 1);
  Entry.Path := '::DataSpace/Storage/MSCompressed/';
  Entry.Name := 'ControlData';
  FInternalFiles.AddEntry(Entry, False);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  //  ::DataSpace/Storage/MSCompressed/SpanInfo
  Entry.DecompressedOffset := FSection0.Position;
  Entry.DecompressedSize := WriteSpanInfoToStream(FSection0, FReadCompressedSize);
  Entry.Path := '::DataSpace/Storage/MSCompressed/';
  Entry.Name := 'SpanInfo';
  FInternalFiles.AddEntry(Entry, False);

  //  ::DataSpace/Storage/MSCompressed/Transform/List
  Entry.DecompressedOffset := FSection0.Position;
  Entry.DecompressedSize := WriteTransformListToStream(FSection0);
  Entry.Path := '::DataSpace/Storage/MSCompressed/Transform/';
  Entry.Name := 'List';
  FInternalFiles.AddEntry(Entry, False);

  //  ::DataSpace/Storage/MSCompressed/Transform/{7FC28940-9D31-11D0-9B27-00A0C91E9C7C}/
  //  ::DataSpace/Storage/MSCompressed/Transform/{7FC28940-9D31-11D0-9B27-00A0C91E9C7C}/InstanceData/ResetTable
  Entry.DecompressedOffset := FSection0.Position;
  Entry.DecompressedSize := WriteResetTableToStream(FSection0, FSection1ResetTable);
  Entry.Path := '::DataSpace/Storage/MSCompressed/Transform/{7FC28940-9D31-11D0-9B27-00A0C91E9C7C}/InstanceData/';
  Entry.Name := 'ResetTable';
  FInternalFiles.AddEntry(Entry, True);


  //  ::DataSpace/Storage/MSCompressed/Content do this last
  Entry.DecompressedOffset := FSection0.Position;
  Entry.DecompressedSize := FSection1Size; // we will write it directly to FOutStream later
  Entry.Path := '::DataSpace/Storage/MSCompressed/';
  Entry.Name := 'Content';
  FInternalFiles.AddEntry(Entry, False);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

end;

procedure TITSFWriter.FileAdded(AStream: TStream; const AEntry: TFileEntryRec);
begin
  // do nothing here
=======
=======
>>>>>>> origin/fixes_2_2
  
=======

>>>>>>> origin/fixes_2.4
end;

function TChmWriter.AddString(AString: String): LongWord;
var
  NextBlock: DWord;
  Pos: DWord;
  n  : TAVLTreeNode;
  StrRec : TStringIndex;
begin
  // #STRINGS starts with a null char
  if FStringsStream.Size = 0 then FStringsStream.WriteByte(0);

  SpareString.TheString:=AString;
  n:=fAvlStrings.FindKey(SpareString,@CompareStrings);
  if assigned(n) then
   exit(TStringIndex(n.data).strid);

  // each entry is a null terminated string
  Pos := DWord(FStringsStream.Position);

  // Strings are contained in $1000 byte blocks and cannot cross blocks
  NextBlock := ($0000F000 and Pos) + $00001000;
  if Length(AString) + 1 > NextBlock then
  begin
    FStringsStream.Size:= NextBlock;
    FStringsStream.Position := NextBlock;
  end;

  Result := FStringsStream.Position;
  FStringsStream.WriteBuffer(AString[1], Length(AString));
  FStringsStream.WriteByte(0);

  StrRec:=TStringIndex.Create;
  StrRec.TheString:=AString;
  StrRec.Strid    :=Result;
  fAvlStrings.Add(StrRec);
end;

function TChmWriter.AddURL ( AURL: String; TopicsIndex: DWord ) : LongWord;

  procedure CheckURLStrBlockCanHold(Const AString: String);
  var
    Rem: LongWord;
    Len: LongWord;
  begin
    Rem := $4000 - (FURLSTRStream.Size mod $4000);
    Len := 9 + Length(AString);  // 2 dwords the string and NT
    if Rem < Len then
      while Rem > 0 do
      begin
        FURLSTRStream.WriteByte(0);
        Dec(Rem);
      end;
  end;

  function AddURLString(Const AString: String): DWord;
  var urlstrrec : TUrlStrIndex;
  begin
    CheckURLStrBlockCanHold(AString);
    if FURLSTRStream.Size mod $4000 = 0 then
      FURLSTRStream.WriteByte(0);
      Result := FURLSTRStream.Position;
      UrlStrRec:=TUrlStrIndex.Create;
      UrlStrRec.UrlStr:=AString;
      UrlStrRec.UrlStrid:=result;
      FAvlUrlStr.Add(UrlStrRec);
      FURLSTRStream.WriteDWord(NToLE(DWord(0))); // URL Offset for topic after the the "Local" value
      FURLSTRStream.WriteDWord(NToLE(DWord(0))); // Offset of FrameName??
      FURLSTRStream.Write(AString[1], Length(AString));
      FURLSTRStream.WriteByte(0); //NT
  end;

  function LookupUrlString(const AUrl : String):DWord;
  var n :TAvlTreeNode;
  begin
    SpareUrlStr.UrlStr:=AUrl;
    n:=FAvlUrlStr.FindKey(SpareUrlStr,@CompareUrlStrs);
    if assigned(n) Then
      result:=TUrlStrIndex(n.data).UrlStrId
    else
      result:=AddUrlString(AUrl);
  end;


var UrlIndex : Integer;

begin
  if AURL[1] = '/' then Delete(AURL,1,1);
  UrlIndex:=LookupUrlString(AUrl);

  //if $1000 - (FURLTBLStream.Size mod $1000) = 4 then // we are at 4092
  if FURLTBLStream.Size and $FFC = $FFC then // faster :)
    FURLTBLStream.WriteDWord(0);
  Result := FURLTBLStream.Position;
  FURLTBLStream.WriteDWord(0);//($231e9f5c); //unknown
  FURLTBLStream.WriteDWord(NtoLE(TopicsIndex)); // Index of topic in #TOPICS
<<<<<<< HEAD
  FURLTBLStream.WriteDWord(NtoLE(AddURLString(AURL)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  FURLTBLStream.WriteDWord(NtoLE(UrlIndex));
>>>>>>> origin/fixes_2.4
end;

function _AtEndOfData(arg: pointer): LongBool; cdecl;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result := TITSFWriter(arg).AtEndOfData;
end;

function TITSFWriter.AtEndOfData: LongBool;
=======
=======
>>>>>>> origin/fixes_2_2
  Result := TChmWriter(arg).AtEndOfData;
end;

function TChmWriter.AtEndOfData: LongBool;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Result := ForceExit or (FCurrentIndex >= FFileNames.Count-1);
  if Result then
    Result := Integer(FCurrentStream.Position) >= Integer(FCurrentStream.Size)-1;
end;

function _GetData(arg: pointer; Count: LongInt; Buffer: Pointer): LongInt; cdecl;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result := TITSFWriter(arg).GetData(Count, PByte(Buffer));
end;

function TITSFWriter.GetData(Count: LongInt; Buffer: PByte): LongInt;
=======
=======
>>>>>>> origin/fixes_2_2
  Result := TChmWriter(arg).GetData(Count, PByte(Buffer));
end;

function TChmWriter.GetData(Count: LongInt; Buffer: PByte): LongInt;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  FileEntry: TFileEntryRec;
begin
  Result := 0;
  while (Result < Count) and (not AtEndOfData) do begin
    Inc(Result, FCurrentStream.Read(Buffer[Result], Count-Result));
    if (Result < Count) and (not AtEndOfData)
    then begin
      // the current file has been read. move to the next file in the list
      FCurrentStream.Position := 0;
<<<<<<< HEAD
<<<<<<< HEAD
      FCurrentStream.Size:=0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      Inc(FCurrentIndex);
      ForceExit := OnGetFileData(FFileNames[FCurrentIndex], FileEntry.Path, FileEntry.Name, FCurrentStream);
      FileEntry.DecompressedSize := FCurrentStream.Size;
      FileEntry.DecompressedOffset := FReadCompressedSize; //269047723;//to test writing really large numbers
      FileEntry.Compressed := True;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

      FileAdded(FCurrentStream, FileEntry);
=======
      
      if FullTextSearch then
        CheckFileMakeSearchable(FCurrentStream, FileEntry);
>>>>>>> graemeg/fixes_2_2
=======
      
=======

>>>>>>> origin/fixes_2.4
      if FullTextSearch then
        CheckFileMakeSearchable(FCurrentStream, FileEntry);
>>>>>>> origin/fixes_2_2

      FInternalFiles.AddEntry(FileEntry);
      // So the next file knows it's offset
      Inc(FReadCompressedSize,  FileEntry.DecompressedSize);
      FCurrentStream.Position := 0;
    end;

    // this is intended for programs to add perhaps a file
    // after all the other files have been added.
    if (AtEndOfData)
    and (FCurrentStream <> FPostStream) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FPostStreamActive := True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
      FPostStreamActive := True;
>>>>>>> origin/fixes_2.4
      if Assigned(FOnLastFile) then
        FOnLastFile(Self);
      FCurrentStream.Free;
      WriteFinalCompressedFiles;
      FCurrentStream := FPostStream;
      FCurrentStream.Position := 0;
      Inc(FReadCompressedSize, FCurrentStream.Size);
    end;
  end;
end;

function _WriteCompressedData(arg: pointer; Count: LongInt; Buffer: Pointer): LongInt; cdecl;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result := TITSFWriter(arg).WriteCompressedData(Count, Buffer);
end;

function TITSFWriter.WriteCompressedData(Count: Longint; Buffer: Pointer): LongInt;
=======
=======
>>>>>>> origin/fixes_2_2
  Result := TChmWriter(arg).WriteCompressedData(Count, Buffer);
end;

function TChmWriter.WriteCompressedData(Count: Longint; Buffer: Pointer): LongInt;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  // we allocate a MB at a time to limit memory reallocation since this
  // writes usually 2 bytes at a time
  if (FSection1 is TMemoryStream) and (FSection1.Position >= FSection1.Size-1) then begin
    FSection1.Size := FSection1.Size+$100000;
  end;
  Result := FSection1.Write(Buffer^, Count);
  Inc(FSection1Size, Result);
end;

procedure _MarkFrame(arg: pointer; UncompressedTotal, CompressedTotal: LongWord); cdecl;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  TITSFWriter(arg).MarkFrame(UncompressedTotal, CompressedTotal);
end;

procedure TITSFWriter.MarkFrame(UnCompressedTotal, CompressedTotal: LongWord);
=======
=======
>>>>>>> origin/fixes_2_2
  TChmWriter(arg).MarkFrame(UncompressedTotal, CompressedTotal);
end;

procedure TChmWriter.MarkFrame(UnCompressedTotal, CompressedTotal: LongWord);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  procedure WriteQWord(Value: QWord);
  begin
    FSection1ResetTable.Write(NToLE(Value), 8);
  end;
  procedure IncEntryCount;
  var
    OldPos: QWord;
    Value: DWord;
  begin
    OldPos := FSection1ResetTable.Position;
    FSection1ResetTable.Position := $4;
    Value := LeToN(FSection1ResetTable.ReadDWord)+1;
    FSection1ResetTable.Position := $4;
    FSection1ResetTable.WriteDWord(NToLE(Value));
    FSection1ResetTable.Position := OldPos;
  end;
  procedure UpdateTotalSizes;
  var
    OldPos: QWord;
  begin
    OldPos := FSection1ResetTable.Position;
    FSection1ResetTable.Position := $10;
    WriteQWord(FReadCompressedSize); // size of read data that has been compressed
    WriteQWord(CompressedTotal);
    FSection1ResetTable.Position := OldPos;
  end;
begin
  if FSection1ResetTable.Size = 0 then begin
    // Write the header
    FSection1ResetTable.WriteDWord(NtoLE(DWord(2)));
    FSection1ResetTable.WriteDWord(0); // number of entries. we will correct this with IncEntryCount
    FSection1ResetTable.WriteDWord(NtoLE(DWord(8))); // Size of Entries (qword)
    FSection1ResetTable.WriteDWord(NtoLE(DWord($28))); // Size of this header
    WriteQWord(0); // Total Uncompressed Size
    WriteQWord(0); // Total Compressed Size
    WriteQWord(NtoLE($8000)); // Block Size
    WriteQWord(0); // First Block start
  end;
  IncEntryCount;
  UpdateTotalSizes;
  WriteQWord(CompressedTotal); // Next Block Start
  // We have to trim the last entry off when we are done because there is no next block in that case
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$IFDEF LZX_USETHREADS}
>>>>>>> graemeg/cpstrnew
=======
{$IFDEF LZX_USETHREADS}
>>>>>>> graemeg/cpstrnew
=======
{$IFDEF LZX_USETHREADS}
>>>>>>> graemeg/cpstrnew
=======
{$IFDEF LZX_USETHREADS}
>>>>>>> origin/cpstrnew
function TITSFWriter.LTGetData(Sender: TLZXCompressor; WantedByteCount: Integer;
=======
{$IFDEF LZX_USETHREADS}
function TChmWriter.LTGetData(Sender: TLZXCompressor; WantedByteCount: Integer;
>>>>>>> origin/fixes_2.4
  Buffer: Pointer): Integer;
begin
  Result := GetData(WantedByteCount, Buffer);
  //WriteLn('Wanted ', WantedByteCount, ' got ', Result);
end;

<<<<<<< HEAD
function TITSFWriter.LTIsEndOfFile(Sender: TLZXCompressor): Boolean;
=======
function TChmWriter.LTIsEndOfFile(Sender: TLZXCompressor): Boolean;
>>>>>>> origin/fixes_2.4
begin
  Result := AtEndOfData;
end;

<<<<<<< HEAD
procedure TITSFWriter.LTChunkDone(Sender: TLZXCompressor;
=======
procedure TChmWriter.LTChunkDone(Sender: TLZXCompressor;
>>>>>>> origin/fixes_2.4
  CompressedSize: Integer; UncompressedSize: Integer; Buffer: Pointer);
begin
  WriteCompressedData(CompressedSize, Buffer);
end;

<<<<<<< HEAD
procedure TITSFWriter.LTMarkFrame(Sender: TLZXCompressor;
=======
procedure TChmWriter.LTMarkFrame(Sender: TLZXCompressor;
>>>>>>> origin/fixes_2.4
  CompressedTotal: Integer; UncompressedTotal: Integer);
begin
  MarkFrame(UncompressedTotal, CompressedTotal);
  //WriteLn('Mark Frame C = ', CompressedTotal, ' U = ', UncompressedTotal);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ENDIF}

>>>>>>> graemeg/cpstrnew
=======
{$ENDIF}

>>>>>>> graemeg/cpstrnew
=======
{$ENDIF}

>>>>>>> graemeg/cpstrnew
=======
{$ENDIF}

>>>>>>> origin/cpstrnew

constructor TITSFWriter.Create(AOutStream: TStream; FreeStreamOnDestroy: Boolean);
begin
  if AOutStream = nil then Raise Exception.Create('TITSFWriter.OutStream Cannot be nil!');
  FOutStream := AOutStream;
  FCurrentIndex := -1;
  FCurrentStream := TMemoryStream.Create;
  FInternalFiles := TFileEntryList.Create;
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$ENDIF}

>>>>>>> origin/fixes_2.4
procedure TChmWriter.CheckFileMakeSearchable(AStream: TStream; AFileEntry: TFileEntryRec);

  var
    TopicEntry: TTopicEntry;
    ATitle: String;
begin
  if Pos('.ht', AFileEntry.Name) > 0 then
  begin
    ATitle := FIndexedFiles.IndexFile(AStream, NextTopicIndex, FSearchTitlesOnly);
    if ATitle <> '' then
      TopicEntry.StringsOffset := AddString(ATitle)
    else
      TopicEntry.StringsOffset := $FFFFFFFF;
    TopicEntry.URLTableOffset := AddURL(AFileEntry.Path+AFileEntry.Name, NextTopicIndex);
    TopicEntry.InContents := 2;
    TopicEntry.Unknown := 0;
    TopicEntry.TocOffset := 0;
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.TocOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.StringsOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.URLTableOffset));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.InContents));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.Unknown));
  end;
end;

function TChmWriter.AddTopic(ATitle,AnUrl:AnsiString):integer;

var
    TopicEntry: TTopicEntry;

begin
    if ATitle <> '' then
      TopicEntry.StringsOffset := AddString(ATitle)
    else
      TopicEntry.StringsOffset := $FFFFFFFF;
    result:=NextTopicIndex;
    TopicEntry.URLTableOffset := AddURL(AnUrl, Result);
    TopicEntry.InContents := 2;
    TopicEntry.Unknown := 0;
    TopicEntry.TocOffset := 0;
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.TocOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.StringsOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.URLTableOffset));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.InContents));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.Unknown));
end;

function TChmWriter.NextTopicIndex: Integer;
begin
  Result := FTopicsStream.Size div 16;
end;

constructor TChmWriter.Create(OutStream: TStream; FreeStreamOnDestroy: Boolean);
begin
  if OutStream = nil then Raise Exception.Create('TChmWriter.OutStream Cannot be nil!');
  FCurrentStream := TMemoryStream.Create;
  FCurrentIndex := -1;
  FOutStream := OutStream;
  FInternalFiles := TFileEntryList.Create;
  FStringsStream := TmemoryStream.Create;
  FTopicsStream := TMemoryStream.Create;
  FURLSTRStream := TMemoryStream.Create;
  FURLTBLStream := TMemoryStream.Create;
  FFiftiMainStream := TMemoryStream.Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FSection0 := TMemoryStream.Create;
  FSection1 := TMemoryStream.Create;
  FSection1ResetTable := TMemoryStream.Create;
  FDirectoryListings := TMemoryStream.Create;
  FPostStream := TMemoryStream.Create;;
  FDestroyStream := FreeStreamOnDestroy;
  FFileNames := TStringList.Create;
<<<<<<< HEAD
<<<<<<< HEAD
end;

destructor TITSFWriter.Destroy;
begin
  if FDestroyStream then FOutStream.Free;
  FInternalFiles.Free;
  FCurrentStream.Free;
=======
=======
>>>>>>> origin/fixes_2_2
  FIndexedFiles := TIndexedWordList.Create;
  FAvlStrings   := TAVLTree.Create(@CompareStrings);    // dedupe strings
  FAvlURLStr    := TAVLTree.Create(@CompareUrlStrs);    // dedupe urltbl + binindex must resolve URL to topicid
  SpareString   := TStringIndex.Create;                 // We need an object to search in avltree
  SpareUrlStr   := TUrlStrIndex.Create;                 //    to avoid create/free circles we keep one in spare
                                                        //    for searching purposes
end;

destructor TChmWriter.Destroy;
begin
  if FDestroyStream then FOutStream.Free;
  if Assigned(FContextStream) then FContextStream.Free;
  FInternalFiles.Free;
  FCurrentStream.Free;
  FStringsStream.Free;
  FTopicsStream.Free;
  FURLSTRStream.Free;
  FURLTBLStream.Free;
  FFiftiMainStream.Free;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FSection0.Free;
  FSection1.Free;
  FSection1ResetTable.Free;
  FDirectoryListings.Free;
  FFileNames.Free;
<<<<<<< HEAD
<<<<<<< HEAD
  inherited Destroy;
end;

procedure TITSFWriter.Execute;
=======
=======
>>>>>>> origin/fixes_2_2
  FIndexedFiles.Free;
  SpareString.free;
  SpareUrlStr.free;
  FAvlUrlStr.FreeAndClear;
  FAvlUrlStr.Free;
  FAvlStrings.FreeAndClear;
  FAvlStrings.Free;
  inherited Destroy;
end;

procedure TChmWriter.Execute;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  InitITSFHeader;
  FOutStream.Position := 0;
  FSection1Size := 0;

  // write any internal files to FCurrentStream that we want in the compressed section
<<<<<<< HEAD
<<<<<<< HEAD
  WriteInternalFilesBefore;

=======
=======
>>>>>>> origin/fixes_2_2
  WriteIVB;

  // written to Section0 (uncompressed)
  WriteREADMEFile;

  WriteOBJINST;
<<<<<<< HEAD
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  // move back to zero so that we can start reading from zero :)
  FReadCompressedSize := FCurrentStream.Size;
  FCurrentStream.Position := 0;  // when compressing happens, first the FCurrentStream is read
                                 // before loading user files. So we can fill FCurrentStream with
                                 // internal files first.

  // this gathers ALL files that should be in section1 (the compressed section)
  StartCompressingStream;
  FSection1.Size := FSection1Size;

<<<<<<< HEAD
<<<<<<< HEAD
  WriteInternalFilesAfter;

  //this creates all special files in the archive that start with ::DataSpace
  WriteDataSpaceFiles(FSection0);

=======
=======
>>>>>>> origin/fixes_2_2
  // This creates and writes the #ITBITS (empty) file to section0
  WriteITBITS;
  // This creates and writes the #SYSTEM file to section0
  WriteSystem;


  //this creates all special files in the archive that start with ::DataSpace
  WriteDataSpaceFiles(FSection0);
<<<<<<< HEAD
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  // creates all directory listings including header
  CreateDirectoryListings;

  // do this after we have compressed everything so that we know the values that must be written
  InitHeaderSectionTable;

  // Now we can write everything to FOutStream
  WriteHeader(FOutStream);
  WriteDirectoryListings(FOutStream);
  WriteSection0; //does NOT include section 1 even though section0.content IS section1
  WriteSection1; // writes section 1 to FOutStream
end;

<<<<<<< HEAD
<<<<<<< HEAD
// this procedure is used to manually add files to compress to an internal stream that is
// processed before FileToCompress is called. Files added this way should not be
// duplicated in the FilesToCompress property.
procedure TITSFWriter.AddStreamToArchive(AFileName, APath: String; AStream: TStream; Compress: Boolean = True);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TChmWriter.AppendTOC(AStream: TStream);
begin
  FHasTOC := True;
  PostAddStreamToArchive('default.hhc', '/', AStream, True);
end;

procedure TChmWriter.AppendBinaryTOCFromSiteMap(ASiteMap: TChmSiteMap);
var
  Header: TTOCIdxHeader;
  Entry: TTocEntry;
  EntryInfo: TTOCEntryPageBookInfo;


  EntryInfoStream,
  EntryTopicOffsetStream,
  EntryStream: TMemoryStream;

  TOCIDXStream: TMemoryStream;

  NextLevelItems,
  CurrentLevelItems: TFPList;
  i,j: Integer;
  MenuItem: TChmSiteMapItem;
  MenuItems: TChmSiteMapItems;
  TopicEntry: TTopicEntry;
  EntryCount: DWord = $29A;
  procedure FixParentBookFirstChildOffset(AChildOffset: DWord);
  var
    ParentEntry: TTOCEntryPageBookInfo;
  begin
    // read parent entry
    EntryInfoStream.Position := MenuItems.InternalData;
    EntryInfoStream.Read(ParentEntry, SizeOf(ParentEntry));
    // update child offset
    ParentEntry.FirstChildOffset:= NtoLE(DWord(4096 + AChildOffset));
    // write back to stream
    EntryInfoStream.Position := MenuItems.InternalData;
    EntryInfoStream.Write(ParentEntry, SizeOf(ParentEntry));
    // move to end of stream
    EntryInfoStream.Position := AChildOffset;
  end;

begin
  FillChar(Header, 4096, 0);
  // create streams
  TOCIDXStream := TMemoryStream.Create;
  EntryInfoStream := TMemoryStream.Create;
  EntryTopicOffsetStream := TMemoryStream.Create;
  EntryStream := TMemoryStream.Create;

  NextLevelItems := TFPList.Create;

  NextLevelItems.Add(ASiteMap.Items);

  if NextLevelItems.Count = 0 then
      FreeAndNil(NextLevelItems);

  while NextLevelItems <> nil do
  begin
    CurrentLevelItems := NextLevelItems;
    NextLevelItems := TFPList.Create;

    for i := 0 to CurrentLevelItems.Count-1 do
    begin
      MenuItems := TChmSiteMapItems(CurrentLevelItems.Items[i]);

      for j := 0 to MenuItems.Count-1 do
      begin
        MenuItem := MenuItems.Item[j];
        // first figure out the props
        EntryInfo.Props := 0;
        if MenuItem.Children.Count > 0 then
          EntryInfo.Props := EntryInfo.Props or TOC_ENTRY_HAS_CHILDREN;
        if Length(MenuItem.Local) > 0 then
          EntryInfo.Props := EntryInfo.Props or TOC_ENTRY_HAS_LOCAL;


      if EntryInfo.Props and TOC_ENTRY_HAS_LOCAL > 0 then
      begin
        // Write #TOPICS entry
        TopicEntry.TocOffset      := NtoLE(DWord(4096 + EntryInfoStream.Position));
        TopicEntry.StringsOffset  := NtoLE(AddString(MenuItem.Text));
        TopicEntry.URLTableOffset := NtoLE(AddURL(MenuItem.Local, NextTopicIndex));
        TopicEntry.InContents     := NtoLE(Word( 2 ));
        TopicEntry.Unknown        := 0;
        EntryInfo.TopicsIndexOrStringsOffset := NtoLE(Dword(NextTopicIndex));;
        FTopicsStream.Write(TopicEntry, SizeOf(TopicEntry));
        EntryTopicOffsetStream.WriteDWord(EntryInfo.TopicsIndexOrStringsOffset);

        // write TOCEntry
        Entry.PageBookInfoOffset:= NtoLE(4096 + EntryInfoStream.Position);
        Entry.IncrementedInt  := NtoLE(EntryCount);
        EntryStream.Write(Entry, SizeOf(Entry));
        Inc(EntryCount);

      end
      else
      begin
        EntryInfo.TopicsIndexOrStringsOffset := NtoLE(AddString(MenuItem.Text));
      end;


        // write TOCEntryInfo

        EntryInfo.Unknown1 := 0;
        EntryInfo.EntryIndex := NtoLE(Word(EntryCount - $29A)); //who knows how useful any of this is

        if MenuItems.InternalData <> maxLongint then
          EntryInfo.ParentPageBookInfoOffset := MenuItems.InternalData
        else
          EntryInfo.ParentPageBookInfoOffset := 0;

        if j = MenuItems.Count-1 then
          EntryInfo.NextPageBookOffset := 0
        else if (EntryInfo.Props and TOC_ENTRY_HAS_CHILDREN) > 0 then
          EntryInfo.NextPageBookOffset := 4096 + EntryInfoStream.Position + 28
        else
          EntryInfo.NextPageBookOffset := 4096 + EntryInfoStream.Position + 20;

        // Only if TOC_ENTRY_HAS_CHILDREN is set are these written
        EntryInfo.FirstChildOffset := 0; // we will update this when the child is written
        // in fact lets update the *parent* of this item now if needed
        if (j = 0) and (MenuItems.InternalData <> maxLongint) then
          FixParentBookFirstChildOffset(EntryInfoStream.Position);

        EntryInfo.Unknown3 := 0;

        // fix endian order
        EntryInfo.Props := NtoLE(EntryInfo.Props);
        EntryInfo.ParentPageBookInfoOffset := NtoLE(EntryInfo.ParentPageBookInfoOffset);
        EntryInfo.NextPageBookOffset := NtoLE(EntryInfo.NextPageBookOffset);

        if MenuItem.Children.Count > 0 then
        begin
          NextLevelItems.Add(MenuItem.Children);
          MenuItem.Children.InternalData := EntryInfoStream.Position;
        end;

        // write to stream
        EntryInfoStream.Write(EntryInfo, PageBookInfoRecordSize(@EntryInfo));
      end;
    end;

    FreeAndNil(CurrentLevelItems);
    if NextLevelItems.Count = 0 then
      FreeAndNil(NextLevelItems);
  end;

  // write all streams to TOCIdxStream and free everything
  EntryInfoStream.Position:=0;
  EntryTopicOffsetStream.Position:=0;
  EntryStream.Position:=0;

  Header.BlockSize := NtoLE(DWord(4096));
  Header.EntriesCount := NtoLE(DWord(EntryCount - $29A));
  Header.EntriesOffset := NtoLE(DWord(4096 + EntryInfoStream.Size + EntryTopicOffsetStream.Size));
  Header.TopicsOffset := NtoLE(DWord(4096 + EntryInfoStream.Size));

  TOCIDXStream.Write(Header, SizeOf(Header));

  TOCIDXStream.CopyFrom(EntryInfoStream, EntryInfoStream.Size);
  EntryInfoStream.Free;

  TOCIDXStream.CopyFrom(EntryTopicOffsetStream, EntryTopicOffsetStream.Size);
  EntryTopicOffsetStream.Free;

  TOCIDXStream.CopyFrom(EntryStream, EntryStream.Size);
  EntryStream.Free;

  TOCIDXStream.Position := 0;
  AppendBinaryTOCStream(TOCIDXStream);
  TOCIDXStream.Free;
end;

Const
      BinIndexIdent : array[0..1] of char = (CHR($3B),CHR($29));
      AlwaysX44     : Array[0..15] of char = ('X','4','4',#0,#0,#0,#0,#0,
                                              #0,#0,#0,#0,#0,#0,#0,#0);
      DataEntry     : Array[0..12] of Byte = ($00,$00,$00,$00,$05,$00,$00,$00,$80,$00,$00,$00,$00);
{
  IndexStream:=TMemoryStream.Create;
  IndexStream.Write(BinIndexIdent,2);
  IndexStream.Write(NToLE(word(2)),2);
  IndexStream.Write(NToLE(word(2048)),2);
  IndexStream.Write(AlwaysX44,sizeof(AlwaysX44));
  IndexStrem.Write (dword(0),2);
}

Const DefBlockSize  = 2048;

Type TIndexBlock = Array[0..DefBlockSize-1] of Byte;

procedure writeword(var p:pbyte;w:word); inline;

begin
  pword(p)^:=NToLE(w);
  inc(pword(p));
end;

procedure writedword(var p:pbyte;d:dword); inline;

begin
  pdword(p)^:=NToLE(d);
  inc(pdword(p));
end;

procedure TChmWriter.AppendBinaryIndexFromSiteMap(ASiteMap: TChmSiteMap;chw:boolean);

Var
  IndexStream : TMemoryStream;
  //n           : Integer;
  curblock    : TIndexBlock;    // current listing block being built
  TestBlock   : TIndexBlock;    // each entry is first built here. then moved to curblock
  curind      : integer;        // next byte to write in testblock.
  blocknr     : Integer;        // blocknr of block in testblock;
  lastblock   : Integer;        // blocknr of last block.
  Entries     : Integer;        // Number of entries in this block so far
  TotalEntries: Integer;        // Total number of entries
  MapEntries  : Integer;
  MapIndex    : Integer;
  indexblocknr: Integer;
  blockind    : Integer;        // next byte to write in blockn[blocknr]
  blockentries: Integer;        // entries so far ins blockn[blocknr]
  blockn      : Array Of TIndexBlock;
  BlockNPlus1 : Array of TIndexBlock;
  Mod13value  : integer;        // A value that is increased by 13 for each entry. (?!?!)
  EntryToIndex: boolean;        // helper var to make sure the first block is always indexed.
  blocknplusindex   : Integer;  // blocks in level n+1 (second part)
  blocknplusentries : Integer;  // The other blocks indexed on creation.
  datastream,mapstream,propertystream : TMemoryStream;

procedure preparecurrentblock;

var p: PBTreeBlockHeader;

begin
  p:=@curblock[0];
  p^.Length:=NToLE(Defblocksize-curind);
  p^.NumberOfEntries:=Entries;
  p^.IndexOfPrevBlock:=lastblock;
  p^.IndexOfNextBlock:=Blocknr;
  IndexStream.Write(curblock[0],Defblocksize);
  MapStream.Write(NToLE(MapEntries),sizeof(dword));
  MapStream.Write(NToLE(BlockNr),Sizeof(DWord));
  MapEntries:=TotalEntries;
  curind:=sizeof(TBtreeBlockHeader);   // index into current block;
  lastblock:=blocknr;
  inc(blocknr);
end;

procedure prepareindexblockn(listingblocknr:integer);
var p:PBTreeIndexBlockHeader;
begin
  p:=@Blockn[IndexBlockNr];
  p^.Length:=defblocksize-BlockInd;
  p^.NumberOfEntries:=BlockEntries;

// p^.IndexOfChildBlock  // already entered on block creation, since of first entry, not last.
  inc(Indexblocknr);
  BlockEntries:=0;
  BlockInd:=0;
  if Indexblocknr>=length(blockn) then
    setlength(blockn,length(blockn)+1);  // larger increments also possible. #blocks is kept independantly.
  p:=@Blockn[IndexBlockNr];
  p^.IndexOfChildBlock:=ListingBlockNr;
  blockind:=sizeof(TBTreeIndexBlockHeader);
end;

procedure finalizeindexblockn(p:pbyte;var ind:integer;Entries:integer);
var ph:PBTreeIndexBlockHeader;
begin
  ph:=PBTreeIndexBlockHeader(p);
  ph^.Length:=defblocksize-Ind;
  ph^.NumberOfEntries:=Entries;
// p^.IndexOfChildBlock  // already entered on block creation, since of first entry, not last.
//  inc(Ind);
end;

procedure CurEntryToIndex(entrysize:integer);
var p,pentry : pbyte;
    indexentrysize : integer;
begin
  indexentrysize:=entrysize-sizeof(dword);         // index entry is 4 bytes shorter, and only the last dword differs
  if (blockind+indexentrysize)>=Defblocksize then
    prepareindexblockn(blocknr);
  p:=@blockn[Indexblocknr][blockind];
  move(testblock[0],p^,indexentrysize);
  pentry:=@p[indexentrysize-sizeof(dword)];         // ptr to last dword
  writedword(pentry,blocknr);                      // patch up the "index of child field"
  inc(blockind,indexentrysize);
end;

procedure CreateEntry(Item:TChmSiteMapItem;Str:WideString;commaatposition:integer);

var p      : pbyte;
    topicid: integer;
    seealso: Integer;
    entrysize:Integer;
    i      : Integer;
begin
  inc(TotalEntries);
  p:=@TestBlock[0];
  for i:=1 to Length(str) do
    WriteWord(p,Word(str[i]));   // write the wstr in little endian
  WriteWord(p,0);                // NT
//  if item.seealso='' then    // no seealso for now
    seealso:=0;
 // else
//    seealso:=2;
  WriteWord(p,seealso);          // =0 not a see also 2 =seealso
  WriteWord(p,2);                // Entrydepth.  We can't know it, so write 2.
  WriteDword(p,commaatposition); // position of the comma
  WriteDword(p,0);               // unused 0
  WriteDword(p,1);               // for now only local pair.
  TopicId:=AddTopic(Item.Text,item.Local);
  WriteDword(p,TopicId);
  // if seealso then _here_ a wchar NT string with seealso?
  WriteDword(p,1);               // always 1 (unknown);
  WriteDword(p,mod13value);      //a value that increments with 13.
  mod13value:=mod13value+13;
  entrysize:=p-pbyte(@testblock[0]);
  if (curind+entrysize)>=Defblocksize then
    begin
      preparecurrentblock;
      EntrytoIndex:=true;
    end;
  if EntryToIndex Then
    begin
      CurEntryToIndex(entrysize);
      EntryToIndex:=False;
    end;
  move(testblock[0],curblock[curind],entrysize);
  inc(curind,entrysize);
  datastream.write(DataEntry,Sizeof(DataEntry));
end;

procedure MoveIndexEntry(nr:integer;bytes:integer;childblock:integer);
var
  pscr,pdest : pbyte;
begin
  {$ifdef binindex}
    writeln(' moveindexentry ',nr,' bytes:',bytes,' childblock:',childblock);
    flush(stdout);
  {$endif}

  if ((blockind+bytes)>=defblocksize) then
    begin
      {$ifdef binindex}
      writeln(' in scalecheck  ',blockind);
      flush(stdout);
      {$endif}

      FinalizeIndexBlockn(@blocknplus1[blocknplusindex][0],blockind,blocknplusentries);
      inc(blocknplusindex);
      if blocknplusindex>=length(blocknplus1) then
        setlength(blocknplus1,length(blocknplus1)+1);
      blockInd:=Sizeof(TBTreeIndexBlockHeader);
      pdword(@blocknplus1[blocknplusindex][0])[4]:=NToLE(ChildBlock);  /// init 2nd level index to first 1st level index block
      end;
  {$ifdef binindex}
    writeln(' len:',length(blocknplus1),' blockind:',blockind,' index:',blocknplusindex);
    flush(stdout);
  {$endif}

  // copy entry from one indexblock to another
  pscr:=@blockn[nr][sizeof(TBtreeIndexBlockHeader)];
  pdest:=@blocknplus1[blocknplusindex][blockind];
  move(pscr^,pdest^,bytes);
  pdword(@pdest[bytes-sizeof(dword)])^:=NToLE(childblock);    // correcting the childindex
  inc (blockind,bytes);
  inc(blocknplusentries); // not needed for writing, but used to check if something has been written. End condition
end;

function ScanIndexBlock(blk:Pbyte):Integer;

var start : pbyte;
    n     : Integer;
    i     : Integer;
begin
  start:=@blk[sizeof(TBtreeIndexBlockHeader)];
  blk:=start;
  while pword(blk)^<>0 do   // skip wchar
    inc(pword(blk));
  inc(pword(blk));          // skip NT
  inc(pword(blk));          // skip see also
  inc(pword(blk));          // skip depth
  inc(pdword(blk));         // skip Character Index.
  inc(pdword(blk));          // skip always  0
  n:=LEToN(pdword(blk)^);
  inc(pdword(blk));          // skip nr of pairs.
  for i:= 1 to n do
      inc(pdword(blk));          // skip <n> topicids
  inc(pdword(blk));          // skip childindex
  Result:=blk-start;
end;

procedure CombineWithChildren(ParentItem:TChmSiteMapItem;Str:WideString;commaatposition:integer;first:boolean);
var i    : Integer;
    Item : TChmSiteMapItem;
begin
  if ParentItem.Children.Count = 0 Then
    Begin
     // comment/fix next
     //   if commatposition=length(str) then commaatposition:=0;
       if first then
        CreateEntry(ParentItem,Str,0)
       else
        CreateEntry(ParentItem,Str,commaatposition);
    End
  Else
    for i:=0 to ParentItem.Children.Count-1 do
      begin
        item := TChmSiteMapItem(ParentItem.Children.Item[i]);
        if first Then
          CombineWithChildren(Item,Str+', '+item.text,commaatposition+2,false)
        else
          CombineWithChildren(Item,Str+', '+item.text,commaatposition,false);
      end;
end;

Var i             : Integer;
    Key           : WideString;
    Item          : TChmSiteMapItem;
    ListingBlocks : Integer;
    EntryBytes    : Integer;
    Hdr           : TBTreeHeader;
    TreeDepth     : Integer;
   
{$ifdef binindex}
procedure printloopvars(i:integer);

begin   
  Writeln('location :' ,i, ' blocknr :', blocknr,' level:',TreeDepth);
  Writeln('blockn      length: ',length(blockn),' indexblocknr: ',indexblocknr,' blockind ',blockind);
  Writeln('blocknplus1 length: ',length(blocknplus1),' blocknplusindex:',blocknplusindex,' entries:',blocknplusentries);
  flush(stdout);
end;
{$endif}   
begin
  IndexStream:=TMemoryStream.Create;
  indexstream.size:=sizeof(TBTreeHeader);
  IndexStream.position:=Sizeof(TBTreeHeader);
  datastream:=TMemoryStream.Create;
  mapstream :=TMemoryStream.Create;
  mapstream.size:=2;
  mapstream.position:=2;
  propertystream :=TMemoryStream.Create;
  propertystream.write(NToLE(0),sizeof(4));
  // we iterate over all entries and write listingblocks directly to the stream.
  // and the first (and maybe last) level is written to blockn.
  // we can't do higher levels yet because we don't know how many listblocks we get
  BlockNr     :=0;   // current block number
  Lastblock   :=-1;  // previous block nr or -1 if none.
  Entries     :=0;   // entries in this block
  TotalEntries:=0;   // entries so far.
  Mod13value  :=0;   // value that increments by 13 entirely.
  indexblocknr:=0;   // nr of first index block.
  BlockEntries:=0;   // entries into current block;
  MapEntries  :=0;   // entries before the current listing block, for MAP file

  curind      :=sizeof(TBTreeBlockHeader);      // index into current listing block;
  blockind    :=sizeof(TBtreeIndexBlockHeader); // index into current index block

  Setlength(blockn,1);
  pdword(@blockn[0][4])^:=NToLE(0);  /// init first listingblock nr to 0 in the first index block
  EntryToIndex   := True;
  for i:=0 to ASiteMap.Items.Count-1 do
    begin
      item := TChmSiteMapItem(ASiteMap.Items.Item[i]);
      key  :=Item.Text;
      {$ifdef chm_windowsbinindex}
      // append 2 to all index level 0 entries. This
      // so we can see if Windows loads the binary or textual index.
      CombineWithChildren(Item,Key+'2',length(key)+1,true);
      {$else}
      CombineWithChildren(Item,Key,length(key),true);
      {$endif}
    end;
  PrepareCurrentBlock;     // flush last listing block.
  Listingblocks:=blocknr;   // blocknr is from now on the number of the first block in blockn.
                            // we still need the # of listingblocks for the header though

  {$ifdef binindex}
    writeln('binindex: listingblocks : '+inttostr(listingblocks),' indexblocks: ',indexblocknr,' entries:',blockentries);
  {$endif}

  // we have now created and written the listing blocks, and created the first level of index in <blockn>
  // the following loop uses <blockn> to calculate the next level (in blocknplus1), then write out blockn,
  // and repeat until we have no entries left.

  // First we finalize the current set of blocks

  if  Blockind<>sizeof(TBtreeIndexBlockHeader) Then
    begin
      {$ifdef binindex}
        writeln('finalizing level 1 index');
      {$endif}
      FinalizeIndexBlockN(@blockn[indexblocknr][0],blockind,blockentries); // also increasing indexblocknr
      inc(IndexBlockNr);
    end; 
  {$ifdef binindex}
    writeln('binindex: listingblocks : '+inttostr(listingblocks),' indexblocks: ',indexblocknr,' entries:',blockentries);
  {$endif}


  while (Indexblocknr>1) do
    begin
      {$ifdef binindex}
        printloopvars(1);
      {$endif}

      blockind      :=sizeof(TBtreeIndexBlockHeader);
      pdword(@blockn[0][4])^:=NToLE(Listingblocks);  /// init 2nd level index to first 1st level index block
      blocknplusindex     :=0;
      blocknplusentries   :=0;
      if length(blocknplus1)<1 then
        Setlength(blocknplus1,1);

      EntryToIndex        :=True;
      {$ifdef binindex}
        printloopvars(2);
      {$endif}
      for i:=0 to Indexblocknr-1 do
        begin
          Entrybytes:=ScanIndexBlock(@blockn[i][0]);
//          writeln('after scan ,',i, ' bytes: ',entrybytes,' blocknr:',blocknr,' indexblocknr:',indexblocknr,' to:',blocknr+i);
          MoveIndexEntry(i,Entrybytes,blocknr+i);
          indexStream.Write(blockn[i][0],defblocksize);
        end;

      {$ifdef binindex}
        printloopvars(3);
      {$endif}

      If Blockind<>sizeof(TBtreeIndexBlockHeader) Then
        begin
          {$ifdef binindex}
            logentry('finalizing');
          {$endif}
          FinalizeIndexBlockn(@blocknplus1[blocknplusindex][0],blockind,blocknplusentries);
          inc(blocknplusindex);
        end;

      inc(blocknr,indexblocknr);

      indexblocknr:=blocknplusindex;
      blockn:=copy(blocknplus1); setlength(blocknplus1,1);
      {$ifdef binindex}
        printloopvars(5);
      {$endif}

      inc(TreeDepth);
    end;
  indexStream.Write(blockn[0][0],defblocksize);
  inc(blocknr);
  // Fixup header.
  hdr.ident[0]:=chr($3B); hdr.ident[1]:=chr($29);
  hdr.flags          :=NToLE($2);           // bit $2 is always 1, bit $0400 1 if dir? (always on)
  hdr.blocksize      :=NToLE(defblocksize); // size of blocks (2048)
  hdr.dataformat     :=AlwaysX44;           // "X44" always the same, see specs.
  hdr.unknown0       :=NToLE(0);            // always 0
  hdr.lastlstblock   :=NToLE(ListingBlocks-1); // index of last listing block in the file;
  hdr.indexrootblock :=NToLE(blocknr-1);    // Index of the root block in the file.
  hdr.unknown1       :=NToLE(-1);           // always -1
  hdr.nrblock        :=NToLE(blocknr);      // Number of blocks
  hdr.treedepth      :=NToLE(TreeDepth);    // The depth of the tree of blocks (1 if no index blocks, 2 one level of index blocks, ...)
  hdr.nrkeywords     :=NToLE(Totalentries); // number of keywords in the file.
  hdr.codepage       :=NToLE(1252);         // Windows code page identifier (usually 1252 - Windows 3.1 US (ANSI))
  hdr.lcid           :=NToLE(0);            //  ???? LCID from the HHP file.
  if not chw then
    hdr.ischm        :=NToLE(1)             // 0 if this a BTREE and is part of a CHW file, 1 if it is a BTree and is part of a CHI or CHM file
  else
    hdr.ischm        :=NToLE(0);
  hdr.unknown2       :=NToLE(10031);        // Unknown. Almost always 10031. Also 66631 (accessib.chm, ieeula.chm, iesupp.chm, iexplore.chm, msoe.chm, mstask.chm, ratings.chm, wab.chm).
  hdr.unknown3       :=NToLE(0);            // unknown 0
  hdr.unknown4       :=NToLE(0);            // unknown 0
  hdr.unknown5       :=NToLE(0);            // unknown 0

  IndexStream.Position:=0;
  IndexStream.write(hdr,sizeof(hdr));
  {$ifdef binindex}
    logentry('before append');
  {$endif}

  AppendBinaryIndexStream(IndexStream,datastream,MapStream,PropertyStream,chw);
  IndexStream.Free;
  PropertyStream.Free;
  MapStream.Free;
  DataStream.Free;
end;

procedure TChmWriter.AppendBinaryTOCStream(AStream: TStream);
begin
  AddStreamToArchive('#TOCIDX', '/', AStream, True);
end;

procedure TChmWriter.AppendBinaryIndexStream(IndexStream,DataStream,MapStream,Propertystream: TStream;chw:boolean);

procedure stadd(fn:string;stream:TStream);

begin
  Stream.Position:=0;
  if CHW then 
    fn:=uppercase(fn);
  {$ifdef binindex}
    logentry('before append '+fn);
  {$endif}  
  AddStreamToArchive(fn,'/$WWKeywordLinks/',stream,True);
end;

begin
  stadd('BTree',IndexStream);
  stadd('Data', DataStream);
  stadd('Map' , MapStream);
  stadd('Property', PropertyStream);
end;

procedure TChmWriter.AppendIndex(AStream: TStream);
begin
  FHasIndex := True;
  PostAddStreamToArchive('default.hhk', '/', AStream, True);
end;

procedure TChmWriter.AppendSearchDB(AName: String; AStream: TStream);
begin
  PostAddStreamToArchive(AName, '/', AStream);
end;


// this procedure is used to manually add files to compress to an internal stream that is
// processed before FileToCompress is called. Files added this way should not be
// duplicated in the FilesToCompress property.
procedure TChmWriter.AddStreamToArchive(AFileName, APath: String; AStream: TStream; Compress: Boolean = True);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  TargetStream: TStream;
  Entry: TFileEntryRec;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  // in case AddStreamToArchive is used after we should be writing to the post stream
  if FPostStreamActive then
  begin
    PostAddStreamToArchive(AFileName, APath, AStream, Compress);
    Exit;
  end;
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
  if AStream = nil then Exit;
  if Compress then
    TargetStream := FCurrentStream
  else
    TargetStream := FSection0;

  Entry.Name := AFileName;
  Entry.Path := APath;
  Entry.Compressed :=  Compress;
  Entry.DecompressedOffset := TargetStream.Position;
  Entry.DecompressedSize := AStream.Size;
<<<<<<< HEAD
<<<<<<< HEAD
  FileAdded(AStream,Entry);
=======
  if FullTextSearch then
    CheckFileMakeSearchable(AStream, Entry); // Must check before we add it to the list so we know if the name needs to be added to #STRINGS
>>>>>>> graemeg/fixes_2_2
=======
  if FullTextSearch then
    CheckFileMakeSearchable(AStream, Entry); // Must check before we add it to the list so we know if the name needs to be added to #STRINGS
>>>>>>> origin/fixes_2_2
  FInternalFiles.AddEntry(Entry);
  AStream.Position := 0;
  TargetStream.CopyFrom(AStream, AStream.Size);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TITSFWriter.PostAddStreamToArchive(AFileName, APath: String;
=======
procedure TChmWriter.PostAddStreamToArchive(AFileName, APath: String;
>>>>>>> graemeg/fixes_2_2
=======
procedure TChmWriter.PostAddStreamToArchive(AFileName, APath: String;
>>>>>>> origin/fixes_2_2
  AStream: TStream; Compress: Boolean);
var
  TargetStream: TStream;
  Entry: TFileEntryRec;
begin
  if AStream = nil then Exit;
  if Compress then
    TargetStream := FPostStream
  else
    TargetStream := FSection0;

  Entry.Name := AFileName;
  Entry.Path := APath;
  Entry.Compressed :=  Compress;
  if not Compress then
    Entry.DecompressedOffset := TargetStream.Position
  else
    Entry.DecompressedOffset := FReadCompressedSize + TargetStream.Position;
  Entry.DecompressedSize := AStream.Size;
  FInternalFiles.AddEntry(Entry);
  AStream.Position := 0;
  TargetStream.CopyFrom(AStream, AStream.Size);
<<<<<<< HEAD
<<<<<<< HEAD
  FileAdded(AStream, Entry);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TITSFWriter.StartCompressingStream;
var
  LZXdata: Plzx_data;
  WSize: LongInt;
  Compressor: TLZXCompressor;
begin
 if fcores=0 then
   begin
     lzx_init(@LZXdata, LZX_WINDOW_SIZE, @_GetData, Self, @_AtEndOfData,
              @_WriteCompressedData, Self, @_MarkFrame, Self);

     WSize := 1 shl LZX_WINDOW_SIZE;
     while not AtEndOfData do begin
       lzx_reset(LZXdata);
       lzx_compress_block(LZXdata, WSize, True);
     end;

     //we have to mark the last frame manually
     MarkFrame(LZXdata^.len_uncompressed_input, LZXdata^.len_compressed_output);

     lzx_finish(LZXdata, nil);
   end
 else
   begin
     if fcores=0 then fcores:=4;
     Compressor := TLZXCompressor.Create(fcores);
     Compressor.OnChunkDone  :=@LTChunkDone;
     Compressor.OnGetData    :=@LTGetData;
     Compressor.OnIsEndOfFile:=@LTIsEndOfFile;
     Compressor.OnMarkFrame  :=@LTMarkFrame;
     Compressor.Execute(True);
     //Sleep(20000);
     Compressor.Free;
   end;
end;


procedure TChmWriter.WriteSYSTEM;
var
  Entry: TFileEntryRec;
  TmpStr: String;
  TmpTitle: String;
const
  VersionStr = 'HHA Version 4.74.8702'; // does this matter?
begin

  // this creates the /#SYSTEM file
  Entry.Name := '#SYSTEM';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset := FSection0.Position;

 { if FileExists('#SYSTEM') then
  begin
    TmpStream := TMemoryStream.Create;
    TmpStream.LoadFromFile('#SYSTEM');
    TmpStream.Position := 0;
    FSection0.CopyFrom(TmpStream, TmpStream.Size);
  end;                                    }
  // EntryCodeOrder: 10 9 4 2 3 16 6 0 1 5
  FSection0.WriteDWord(NToLE(Word(3))); // Version
  if Title <> '' then
    TmpTitle := Title
  else
    TmpTitle := 'default';

  // Code -> Length -> Data
  // 10
  FSection0.WriteWord(NToLE(Word(10)));
  FSection0.WriteWord(NToLE(Word(SizeOf(DWord))));
  FSection0.WriteDWord(NToLE(MilliSecondOfTheDay(Now)));
  // 9
  FSection0.WriteWord(NToLE(Word(9)));
  FSection0.WriteWord(NToLE(Word(SizeOf(VersionStr)+1)));
  FSection0.Write(VersionStr, SizeOf(VersionStr));
  FSection0.WriteByte(0);
  // 4 A struct that is only needed to set if full text search is on.
  FSection0.WriteWord(NToLE(Word(4)));
  FSection0.WriteWord(NToLE(Word(36))); // size

  FSection0.WriteDWord(NToLE(DWord($0409)));
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(NToLE(DWord(Ord(FFullTextSearch and FFullTextSearchAvailable))));

  FSection0.WriteDWord(NToLE(Dword(Ord(FHasKLinks))) ); // klinks
  FSection0.WriteDWord(0); // alinks

  // two for a QWord
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);


  ////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // 2  default page to load
  if FDefaultPage <> '' then begin
    FSection0.WriteWord(NToLE(Word(2)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultPage)+1)));
    FSection0.Write(FDefaultPage[1], Length(FDefaultPage));
    FSection0.WriteByte(0);
  end;
  // 3  Title
  if FTitle <> '' then begin
    FSection0.WriteWord(NToLE(Word(3)));
    FSection0.WriteWord(NToLE(Word(Length(FTitle)+1)));
    FSection0.Write(FTitle[1], Length(FTitle));
    FSection0.WriteByte(0);
  end;

  // 16 Default Font
  if FDefaultFont <> '' then begin
    FSection0.WriteWord(NToLE(Word(16)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultFont)+1)));
    FSection0.Write(FDefaultFont[1], Length(FDefaultFont));
    FSection0.WriteByte(0);
  end;

  // 6
  // unneeded. if output file is :  /somepath/OutFile.chm the value here is outfile(lowercase)
  {FSection0.WriteWord(6);
  FSection0.WriteWord(Length('test1')+1);
  Fsection0.Write('test1', 5);
  FSection0.WriteByte(0);}

  // 0 Table of contents filename
  if FHasTOC then begin
    if fTocName ='' then
      TmpStr := DefaultHHC
    else
      TmpStr := fTocName;
    FSection0.WriteWord(0);
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 1
  // hhk Index
  if FHasIndex then begin
    if fIndexName='' then
      TmpStr := DefaultHHK
    else
      TmpStr := fIndexName;
    FSection0.WriteWord(NToLE(Word(1)));
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 5 Default Window

  if FDefaultWindow<>'' then
    begin
      FSection0.WriteWord(NTOLE(Word(5)));
      tmpstr:=FDefaultWindow;
      FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
      FSection0.Write(TmpStr[1], Length(TmpStr));
      FSection0.WriteByte(0);
    end;

  // 7 Binary Index
  if FHasBinaryIndex then
  begin
    {$ifdef binindex}
      logentry('binary index!');
    {$endif}
    FSection0.WriteWord(NToLE(Word(7)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;

  // 11 Binary TOC
  if FHasBinaryTOC then
  begin
    FSection0.WriteWord(NToLE(Word(11)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;


  // 13
  if FIDXHdrStream.size>0 then
  begin
    FSection0.WriteWord(NToLE(Word(13)));
    FSection0.WriteWord(NToLE(Word(FIDXHdrStream.size)));
    FSection0.copyfrom(FIDXHdrStream,0);
  end;

  Entry.DecompressedSize := FSection0.Position - Entry.DecompressedOffset;
  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteITBITS;
var
  Entry: TFileEntryRec;
begin
  // This is an empty and useless file
  Entry.Name := '#ITBITS';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset :=0;// FSection0.Position;
  Entry.DecompressedSize := 0;

  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteSTRINGS;
begin
  if FStringsStream.Size = 0 then;
    FStringsStream.WriteByte(0);
  FStringsStream.Position := 0;
  PostAddStreamToArchive('#STRINGS', '/', FStringsStream);
end;

procedure TChmWriter.WriteTOPICS;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  if tocname<>'' then
    AddTopic('',self.TOCName,2);
  if indexname<>'' then
    AddTopic('',self.IndexName,2);

  FTopicsStream.Position := 0;
  PostAddStreamToArchive('#TOPICS', '/', FTopicsStream);
 // I commented the code below since the result seemed unused
 // FHits:=0;
 //   FIndexedFiles.ForEach(@IterateWord,FHits);
end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TITSFWriter.StartCompressingStream;
var
  {$IFNDEF LZX_USETHREADS}
  LZXdata: Plzx_data;
  WSize: LongInt;
  {$ELSE}
  Compressor: TLZXCompressor;
  {$ENDIF}
begin
 {$IFNDEF LZX_USETHREADS}
  lzx_init(@LZXdata, LZX_WINDOW_SIZE, @_GetData, Self, @_AtEndOfData,
              @_WriteCompressedData, Self, @_MarkFrame, Self);
>>>>>>> graemeg/cpstrnew

procedure TChmWriter.WriteIDXHDR;
begin
  if FIDXHdrStream.Size = 0 then
    Exit;
  FIDXHdrStream.Position := 0;
  PostAddStreamToArchive('#IDXHDR', '/', FIDXHdrStream);
end;

procedure TChmWriter.WriteIVB;
begin
  if FContextStream = nil then exit;

<<<<<<< HEAD
=======
  lzx_finish(LZXdata, nil);
  {$ELSE}
  Compressor := TLZXCompressor.Create(4);
  Compressor.OnChunkDone  :=@LTChunkDone;
  Compressor.OnGetData    :=@LTGetData;
  Compressor.OnIsEndOfFile:=@LTIsEndOfFile;
  Compressor.OnMarkFrame  :=@LTMarkFrame;
  Compressor.Execute(True);
  //Sleep(20000);
  Compressor.Free;
  {$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
end;


procedure TChmWriter.WriteSystem;
var
  Entry: TFileEntryRec;
  TmpStr: String;
  TmpTitle: String;
const
  VersionStr = 'HHA Version 4.74.8702'; // does this matter?
begin


  // this creates the /#SYSTEM file
  Entry.Name := '#SYSTEM';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset := FSection0.Position;

 { if FileExists('#SYSTEM') then
  begin
    TmpStream := TMemoryStream.Create;
    TmpStream.LoadFromFile('#SYSTEM');
    TmpStream.Position := 0;
    FSection0.CopyFrom(TmpStream, TmpStream.Size);
  end;                                    }
  // EntryCodeOrder: 10 9 4 2 3 16 6 0 1 5
  FSection0.WriteDWord(NToLE(Word(3))); // Version
  if Title <> '' then
    TmpTitle := Title
  else
    TmpTitle := 'default';

  // Code -> Length -> Data
  // 10
  FSection0.WriteWord(NToLE(Word(10)));
  FSection0.WriteWord(NToLE(Word(SizeOf(DWord))));
  FSection0.WriteDWord(NToLE(MilliSecondOfTheDay(Now)));
  // 9
  FSection0.WriteWord(NToLE(Word(9)));
  FSection0.WriteWord(NToLE(Word(SizeOf(VersionStr)+1)));
  FSection0.Write(VersionStr, SizeOf(VersionStr));
  FSection0.WriteByte(0);
  // 4 A struct that is only needed to set if full text search is on.
  FSection0.WriteWord(NToLE(Word(4)));
  FSection0.WriteWord(NToLE(Word(36))); // size

  FSection0.WriteDWord(NToLE(DWord($0409)));
  FSection0.WriteDWord(1);
  FSection0.WriteDWord(NToLE(DWord(Ord(FFullTextSearch and FFullTextSearchAvailable))));
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  // two for a QWord
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);




  ////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // 2  default page to load
  if FDefaultPage <> '' then begin
    FSection0.WriteWord(NToLE(Word(2)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultPage)+1)));
    FSection0.Write(FDefaultPage[1], Length(FDefaultPage));
    FSection0.WriteByte(0);
  end;
  // 3  Title
  if FTitle <> '' then begin
    FSection0.WriteWord(NToLE(Word(3)));
    FSection0.WriteWord(NToLE(Word(Length(FTitle)+1)));
    FSection0.Write(FTitle[1], Length(FTitle));
    FSection0.WriteByte(0);
  end;

  // 16 Default Font
  if FDefaultFont <> '' then begin
    FSection0.WriteWord(NToLE(Word(16)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultFont)+1)));
    FSection0.Write(FDefaultFont[1], Length(FDefaultFont));
    FSection0.WriteByte(0);
  end;

  // 6
  // unneeded. if output file is :  /somepath/OutFile.chm the value here is outfile(lowercase)
  {FSection0.WriteWord(6);
  FSection0.WriteWord(Length('test1')+1);
  Fsection0.Write('test1', 5);
  FSection0.WriteByte(0);}

  // 0 Table of contents filename
  if FHasTOC then begin
    if fTocName ='' then
      TmpStr := DefaultHHC
    else
      TmpStr := fTocName;
    FSection0.WriteWord(0);
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 1
  // hhk Index
  if FHasIndex then begin
    if fIndexName='' then
      TmpStr := DefaultHHK
    else
      TmpStr := fIndexName;
    FSection0.WriteWord(NToLE(Word(1)));
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 5 Default Window

  if FDefaultWindow<>'' then
    begin
      FSection0.WriteWord(NTOLE(Word(5)));
      tmpstr:=FDefaultWindow;
      FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
      FSection0.Write(TmpStr[1], Length(TmpStr));
      FSection0.WriteByte(0);
    end;

  // 7 Binary Index
  if FHasBinaryIndex then
  begin
    {$ifdef binindex}
      logentry('binary index!');
    {$endif}
    FSection0.WriteWord(NToLE(Word(7)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;

  // 11 Binary TOC
  if FHasBinaryTOC then
  begin
    FSection0.WriteWord(NToLE(Word(11)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;


  Entry.DecompressedSize := FSection0.Position - Entry.DecompressedOffset;
  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteITBITS;
var
  Entry: TFileEntryRec;
begin
  // This is an empty and useless file
  Entry.Name := '#ITBITS';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset :=0;// FSection0.Position;
  Entry.DecompressedSize := 0;

  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteSTRINGS;
begin
  if FStringsStream.Size = 0 then;
    FStringsStream.WriteByte(0);
  FStringsStream.Position := 0;
  PostAddStreamToArchive('#STRINGS', '/', FStringsStream);
end;

procedure TChmWriter.WriteTOPICS;
//var
  //FHits: Integer;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  FTopicsStream.Position := 0;
  PostAddStreamToArchive('#TOPICS', '/', FTopicsStream);
 // I commented the code below since the result seemed unused
 // FHits:=0;
 //   FIndexedFiles.ForEach(@IterateWord,FHits);
end;

procedure TChmWriter.WriteIVB;
begin
  if FContextStream = nil then exit;

>>>>>>> graemeg/cpstrnew
  FContextStream.Position := 0;
  // the size of all the entries
  FContextStream.WriteDWord(NToLE(DWord(FContextStream.Size-SizeOf(dword))));

  FContextStream.Position := 0;
  AddStreamToArchive('#IVB', '/', FContextStream);
end;

<<<<<<< HEAD
const idxhdrmagic ='T#SM';

procedure TChmWriter.CreateIDXHDRStream;
var i : Integer;
begin
   if fmergefiles.count=0 then  // I assume text/site properties could also trigger idxhdr
     exit;

   FIDXHdrStream.setsize(4096);
   FIDXHdrStream.position:=0;
   FIDXHdrStream.write(idxhdrmagic[1],4);     //  0 Magic
   FIDXHdrStream.writedword(ntole(1));        //  4 Unknown timestamp/checksum
   FIDXHdrStream.writedword(ntole(1));        //  8 1 (unknown)
   FIDXHdrStream.writedword(ntole(FNrTopics));        //  C Number of topic nodes including the contents & index files
   FIDXHdrStream.writedword(ntole(0));        // 10 0 (unknown)

   // 14 Offset in the #STRINGS file of the ImageList param of the "text/site properties" object of the sitemap contents (0/-1 = none)
   if assigned(ftocsm) and (ftocsm.ImageList<>'') then
     FIDXHdrStream.writedwordLE(addstring(ftocsm.ImageList))
   else
     FIDXHdrStream.writedwordLE(0);

   // 18 0 (unknown)
   FIDXHdrStream.writedwordLE(0);

   // 1C 1 if the value of the ImageType param of the "text/site properties" object of the sitemap contents is Folder. 0 otherwise.
   if assigned(ftocsm) and (ftocsm.UseFolderImages) then
     FIDXHdrStream.writedwordLE(1)
   else
     FIDXHdrStream.writedwordLE(0);

   // 20 The value of the Background param of the "text/site properties" object of the sitemap contents
   if assigned(ftocsm) then
     FIDXHdrStream.writedwordLE(ftocsm.Backgroundcolor)
   else
     FIDXHdrStream.writedwordLE(0);

   // 24 The value of the Foreground param of the "text/site properties" object of the sitemap contents
   if assigned(ftocsm) then
     FIDXHdrStream.writedwordLE(ftocsm.Foregroundcolor)
   else
     FIDXHdrStream.writedwordLE(0);

   // 28 Offset in the #STRINGS file of the Font param of the "text/site properties" object of the sitemap contents (0/-1 = none)
   if assigned(ftocsm) and (ftocsm.Font<>'') then
     FIDXHdrStream.writedwordLE(addstring(ftocsm.font))
   else
     FIDXHdrStream.writedwordLE(0);

   // 2C The value of the Window Styles param of the "text/site properties" object of the sitemap contents
   if assigned(ftocsm) then
     FIDXHdrStream.writedwordLE(FTocsm.WindowStyles)
   else
     FIDXHdrStream.writedwordLE(0);

   // 30 The value of the EXWindow Styles param of the "text/site properties" object of the sitemap contents
   if assigned(ftocsm) then
     FIDXHdrStream.writedwordLE(FTocSm.ExWindowStyles)
   else
     FIDXHdrStream.writedwordLE(0);

   // 34 Unknown. Often -1. Sometimes 0.
   FIDXHdrStream.writedwordLE(0);

   // 38 Offset in the #STRINGS file of the FrameName param of the "text/site properties" object of the sitemap contents (0/-1 = none)
   if assigned(ftocsm) and (ftocsm.framename<>'') then
     FIDXHdrStream.writedwordLE(addstring(FTocsm.Framename))
   else
     FIDXHdrStream.writedwordLE(0);

   // 3C Offset in the #STRINGS file of the WindowName param of the "text/site properties" object of the sitemap contents (0/-1 = none)
   if assigned(ftocsm) and (ftocsm.windowname<>'') then
     FIDXHdrStream.writedwordLE(addstring(FTocsm.windowname))
   else
     FIDXHdrStream.writedwordLE(0);
   FIDXHdrStream.writedword(ntole(0));        // 40 Number of information types.
   FIDXHdrStream.writedword(ntole(0));        // 44 Unknown. Often 1. Also 0, 3.
   FIDXHdrStream.writedword(ntole(fmergefiles.count));        // 48 Number of files in the [MERGE FILES] list.

   // 4C Unknown. Often 0. Non-zero mostly in files with some files in the merge files list.
   if fmergefiles.count>0 then
     FIDXHdrStream.writedwordLE(1)
   else
     FIDXHdrStream.writedwordLE(0);

   for i:=0 to FMergefiles.count-1 do
     FIDXHdrStream.WriteDword(addstring(fmergefiles[i]));

   for i:=0 to 1004-fmergefiles.count-1 do
    FIDXHdrStream.WriteDword(0);
end;

=======
>>>>>>> graemeg/cpstrnew
procedure TChmWriter.WriteURL_STR_TBL;
begin
  if FURLSTRStream.Size <> 0 then begin
    FURLSTRStream.Position := 0;
    PostAddStreamToArchive('#URLSTR', '/', FURLSTRStream);
  end;
  if FURLTBLStream.Size <> 0 then begin
    FURLTBLStream.Position := 0;
    PostAddStreamToArchive('#URLTBL', '/', FURLTBLStream);
  end;
end;

procedure TChmWriter.WriteOBJINST;
var
  i: Integer;
  ObjStream: TMemoryStream;
  //Flags: Word;
begin
  ObjStream := TMemorystream.Create;
  // this file is needed to enable searches for the ms reader
  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(Dword(2))); // two entries

  ObjStream.WriteDWord(NtoLE(DWord(24))); // offset into file of entry
  ObjStream.WriteDWord(NtoLE(DWord(2691))); // size

  ObjStream.WriteDWord(NtoLE(DWord(2715))); // offset into file of entry
  ObjStream.WriteDWord(NtoLE(DWord(36))); // size

  // first entry
  // write guid 4662DAAF-D393-11D0-9A56-00C04FB68BF7
  ObjStream.WriteDWord(NtoLE($4662DAAF));
  ObjStream.WriteWord(NtoLE($D393));
<<<<<<< HEAD
  ObjStream.WriteWord(NtoLE(word($11D0)));
  ObjStream.WriteWord(NtoLE(word($569A)));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($F7);

<<<<<<< HEAD
  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(11));  // bit flags
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE($00000000));
  ObjStream.WriteDWord(NtoLE($00000000));
  ObjStream.WriteDWord(NtoLE($00145555));
  ObjStream.WriteDWord(NtoLE($00000A0F));
  ObjStream.WriteWord(NtoLE($0100));
  ObjStream.WriteDWord(NtoLE($00030005));
  for i := 0 to 5 do
    ObjStream.WriteDWord($00000000);
  ObjStream.WriteWord($0000);
  // okay now the fun stuff
  for i := 0 to $FF do
  ObjStream.Write(ObjInstEntries[i], SizeOF(TObjInstEntry));
  {begin
    if i = 1 then
      Flags := 7
    else
      Flags := 0;
    if (i >= $41) and (i <= $5A) then
      Flags := Flags or 2;
    if (i >= $61) and (i <= $7A) then
      Flags := Flags or 1;
    if i = $27 then
      Flags := Flags or 6;
    ObjStream.WriteWord(NtoLE(Flags));
    ObjStream.WriteWord(NtoLE(Word(i)));
    if (i >= $41) and (i <= $5A) then
      ObjStream.WriteByte(NtoLE(i+$20))
    else
      ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteWord(NtoLE($0000));
  end;}
  ObjStream.WriteDWord(NtoLE($E66561C6));
  ObjStream.WriteDWord(NtoLE($73DF6561));
  ObjStream.WriteDWord(NtoLE($656F8C73));
  ObjStream.WriteWord(NtoLE(word($6F9C)));
  ObjStream.WriteByte($65);
  // third bit of second entry
  // write guid 8FA0D5A8-DEDF-11D0-9A61-00C04FB68BF7
  ObjStream.WriteDWord(NtoLE($8FA0D5A8));
  ObjStream.WriteWord(NtoLE($DEDF));
  ObjStream.WriteWord(NtoLE(word($11D0)));
  ObjStream.WriteWord(NtoLE(word($619A)));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($F7);

  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(DWord(1)));
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE(DWord(0)));

  // second entry
  // write guid 4662DAB0-D393-11D0-9A56-00C04FB68B66
  ObjStream.WriteDWord(NtoLE($4662DAB0));
  ObjStream.WriteWord(NtoLE($D393));
  ObjStream.WriteWord(NtoLE(word($11D0)));
  ObjStream.WriteWord(NtoLE(word($569A)));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($66);

  ObjStream.WriteDWord(NtoLE(DWord(666))); // not kidding
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE(DWord(10031)));
  ObjStream.WriteDWord(NtoLE(DWord(0)));

  ObjStream.Position := 0;
  AddStreamToArchive('$OBJINST', '/', ObjStream, True);
  ObjStream.Free;

end;

procedure TChmWriter.WriteFiftiMain;
var
  SearchWriter: TChmSearchWriter;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  SearchWriter := TChmSearchWriter.Create(FFiftiMainStream, FIndexedFiles);
  // do not add an empty $FIftiMain
  if not SearchWriter.HasData then
  begin
    FFullTextSearchAvailable := False;
    SearchWriter.Free;
    Exit;
  end;
  FFullTextSearchAvailable := True;
  SearchWriter.WriteToStream;
  SearchWriter.Free;

  if FFiftiMainStream.Size = 0 then
    Exit;

  FFiftiMainStream.Position := 0;
  PostAddStreamToArchive('$FIftiMain', '/', FFiftiMainStream);
end;

procedure TChmWriter.WriteWindows;
Var WindowStream : TMemoryStream;
    i,j          : Integer;
    win          : TChmWindow;
begin
  if FWindows.Count>0 then
    begin
      WindowStream:=TMemoryStream.Create;
      WindowStream.WriteDword(NToLE(dword(FWindows.Count)));
      WindowStream.WriteDword(NToLE(dword(196))); // 1.1 or later. 188 is old style.
      for i:=0 to FWindows.Count-1 Do
        begin
          Win:=TChmWindow(FWindows[i]);
          WindowStream.WriteDwordLE (196);                               //  0 size of entry.
          WindowStream.WriteDwordLE (0);                                 //  4 unknown (bool Unicodestrings?)
          WindowStream.WriteDword(NToLE(addstring(win.window_type )));   //  8 Arg 0, name of window
          WindowStream.WriteDword(NToLE(dword(win.flags )));             //  C valid fields
          WindowStream.WriteDword(NToLE(dword(win.nav_style)));          // 10 arg 10 navigation pane style
          WindowStream.WriteDword(NToLE(addstring(win.title_bar_text))); // 14 Arg 1,  title bar text
          WindowStream.WriteDword(NToLE(dword(win.styleflags)));         // 18 Arg 14, style flags
          WindowStream.WriteDword(NToLE(dword(win.xtdstyleflags)));      // 1C Arg 15, xtd style flags
          WindowStream.WriteDword(NToLE(dword(win.left)));               // 20 Arg 13, rect.left
          WindowStream.WriteDword(NToLE(dword(win.top)));                // 24 Arg 13, rect.top
          WindowStream.WriteDword(NToLE(dword(win.right)));              // 28 Arg 13, rect.right
          WindowStream.WriteDword(NToLE(dword(win.bottom)));             // 2C Arg 13, rect.bottom
          WindowStream.WriteDword(NToLE(dword(win.window_show_state)));  // 30 Arg 16, window show state
          WindowStream.WriteDword(NToLE(dword(0)));                      // 34  -    , HWND hwndhelp                OUT: window handle"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 38  -    , HWND hwndcaller              OUT: who called this window"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 3C  -    , HH_INFO_TYPE paINFO_TYPES    IN: Pointer to an array of Information Types"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 40  -    , HWND hwndtoolbar             OUT: toolbar window in tri-pane window"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 44  -    , HWND hwndnavigation          OUT: navigation window in tri-pane window"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 48  -    , HWND hwndhtml                OUT: window displaying HTML in tri-pane window"
          WindowStream.WriteDword(NToLE(dword(win.navpanewidth)));       // 4C Arg 11, width of nav pane
          WindowStream.WriteDword(NToLE(dword(0)));                      // 50  -    , rect.left,   OUT:Specifies the coordinates of the Topic pane
          WindowStream.WriteDword(NToLE(dword(0)));                      // 54  -    , rect.top ,   OUT:Specifies the coordinates of the Topic pane
          WindowStream.WriteDword(NToLE(dword(0)));                      // 58  -    , rect.right,  OUT:Specifies the coordinates of the Topic pane
          WindowStream.WriteDword(NToLE(dword(0)));                      // 5C  -    , rect.bottom, OUT:Specifies the coordinates of the Topic pane
          WindowStream.WriteDword(NToLE(addstring(win.toc_file)));       // 60 Arg 2,  toc file
          WindowStream.WriteDword(NToLE(addstring(win.index_file)));     // 64 Arg 3,  index file
          WindowStream.WriteDword(NToLE(addstring(win.default_file)));   // 68 Arg 4,  default file
          WindowStream.WriteDword(NToLE(addstring(win.home_button_file))); // 6c Arg 5,  home button file.
          WindowStream.WriteDword(NToLE(dword(win.buttons)));            // 70 arg 12,
          WindowStream.WriteDword(NToLE(dword(win.navpane_initially_closed))); // 74 arg 17
          WindowStream.WriteDword(NToLE(dword(win.navpane_default)));    // 78 arg 18,
          WindowStream.WriteDword(NToLE(dword(win.navpane_location)));   // 7C arg 19,
          WindowStream.WriteDword(NToLE(dword(win.wm_notify_id)));       // 80 arg 20,
          for j:=0 to 4 do
            WindowStream.WriteDword(NToLE(dword(0)));                    // 84  -      byte[20] unknown -  "BYTE tabOrder[HH_MAX_TABS + 1]; // IN/OUT: tab order: Contents, Index, Search, History, Favorites, Reserved 1-5, Custom tabs"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 94  -      int cHistory; // IN/OUT: number of history items to keep (default is 30)
          WindowStream.WriteDword(NToLE(addstring(win.Jumpbutton_1_Text)));  // 9C Arg 7,  The text of the Jump 1 button.
          WindowStream.WriteDword(NToLE(addstring(win.Jumpbutton_2_Text)));  // A0 Arg 9,  The text of the Jump 2 button.
          WindowStream.WriteDword(NToLE(addstring(win.Jumpbutton_1_File)));  // A4 Arg 6,  The file shown for Jump 1 button.
          WindowStream.WriteDword(NToLE(addstring(win.Jumpbutton_2_File)));  // A8 Arg 8,  The file shown for Jump 1 button.
          for j:=0 to 3 do
            WindowStream.WriteDword(NToLE(dword(0)));                    // AA  -      byte[16] (TRECT) "RECT rcMinSize; // Minimum size for window (ignored in version 1)"
          //   1.1+ fields
          WindowStream.WriteDword(NToLE(dword(0)));                      // BC -       int cbInfoTypes; // size of paInfoTypes;
          WindowStream.WriteDword(NToLE(dword(0)));                      // C0  -      LPCTSTR pszCustomTabs; // multiple zero-terminated strings
        end;
      WindowStream.Position := 0;
      AddStreamToArchive('#WINDOWS', '/', WindowStream, True);
      WindowStream.Free;
    end;
end;

procedure TChmWriter.WriteInternalFilesAfter;
begin
  // This creates and writes the #ITBITS (empty) file to section0
  WriteITBITS;
  // This creates and writes the #SYSTEM file to section0
  WriteSystem;
  if Assigned(FTocSM)  then
   Scansitemap(FTocSM);
end;

procedure TChmWriter.WriteFinalCompressedFiles;
begin
  inherited WriteFinalCompressedFiles;
  WriteTOPICS;
  WriteURL_STR_TBL;
  WriteWINDOWS;
  CreateIDXHDRStream;
  WriteIDXHDR;
  WriteSTRINGS;
  WriteFiftiMain;
end;

procedure TChmWriter.FileAdded(AStream: TStream; const AEntry: TFileEntryRec);
begin
  inherited FileAdded(AStream, AEntry);
  if FullTextSearch then
    CheckFileMakeSearchable(AStream, AEntry);
end;

procedure TChmWriter.WriteInternalFilesBefore;
begin
  inherited WriteInternalFilesBefore;
  WriteIVB;
  WriteOBJINST;
end;

constructor TChmWriter.Create(AOutStream: TStream; FreeStreamOnDestroy: Boolean);
begin
  inherited Create(AOutStream, FreeStreamOnDestroy);
  FStringsStream := TmemoryStream.Create;
  FTopicsStream := TMemoryStream.Create;
  FURLSTRStream := TMemoryStream.Create;
  FURLTBLStream := TMemoryStream.Create;
  FFiftiMainStream := TMemoryStream.Create;
  FIndexedFiles := TIndexedWordList.Create;
  FAVLTopicdedupe  :=TAVLTree.Create(@CompareStrings);  // dedupe filenames in topics.
  FAvlStrings   := TAVLTree.Create(@CompareStrings);    // dedupe strings
  FAvlURLStr    := TAVLTree.Create(@CompareUrlStrs);    // dedupe urltbl + binindex must resolve URL to topicid
  SpareString   := TStringIndex.Create;                 // We need an object to search in avltree
  SpareUrlStr   := TUrlStrIndex.Create;                 //    to avoid create/free circles we keep one in spare
  FIDXHdrStream := TMemoryStream.Create;                // the #IDXHDR and chunk 13 in #SYSTEM
                                                        //    for searching purposes
  FWindows      := TObjectlist.Create(True);
  FDefaultWindow:= '';
  FMergeFiles   :=TStringList.Create;
  FNrTopics     :=0;
end;

destructor TChmWriter.Destroy;
begin
  if Assigned(FContextStream) then FContextStream.Free;
  FMergeFiles.Free;
  FIndexedFiles.Free;
  FStringsStream.Free;
  FTopicsStream.Free;
  FURLSTRStream.Free;
  FURLTBLStream.Free;
  FFiftiMainStream.Free;
  FIDXHdrStream.Free;
  SpareString.free;
  SpareUrlStr.free;
  FAvlUrlStr.FreeAndClear;
  FAvlUrlStr.Free;
  FAvlStrings.FreeAndClear;
  FAvlStrings.Free;
  FAVLTopicdedupe.FreeAndClear;
  FAVLTopicdedupe.free;
  FWindows.Free;

  inherited Destroy;
end;


function TChmWriter.AddString(AString: String): LongWord;
var
  NextBlock: DWord;
  Pos: DWord;
  n  : TAVLTreeNode;
  StrRec : TStringIndex;
begin
  // #STRINGS starts with a null char
  if FStringsStream.Size = 0 then FStringsStream.WriteByte(0);

  SpareString.TheString:=AString;
  n:=fAvlStrings.FindKey(SpareString,@CompareStrings);
  if assigned(n) then
    exit(TStringIndex(n.data).strid);

  // each entry is a null terminated string
  Pos := DWord(FStringsStream.Position);

  // Strings are contained in $1000 byte blocks and cannot cross blocks
  NextBlock := ($0000F000 and Pos) + $00001000;
  if Length(AString) + 1 > NextBlock then
  begin
    FStringsStream.Size:= NextBlock;
    FStringsStream.Position := NextBlock;
  end;

  Result := FStringsStream.Position;
  if length(AString)>0 Then
    FStringsStream.WriteBuffer(AString[1], Length(AString));
  FStringsStream.WriteByte(0);
  StrRec:=TStringIndex.Create;
  StrRec.TheString:=AString;
  StrRec.Strid    :=Result;
  fAvlStrings.Add(StrRec);
end;

function TChmWriter.AddURL ( AURL: String; TopicsIndex: DWord ) : LongWord;

  procedure CheckURLStrBlockCanHold(Const AString: String);
  var
    Rem: LongWord;
    Len: LongWord;
  begin
    Rem := $4000 - (FURLSTRStream.Size mod $4000);
    Len := 9 + Length(AString);  // 2 dwords the string and NT
    if Rem < Len then
      while Rem > 0 do
      begin
        FURLSTRStream.WriteByte(0);
        Dec(Rem);
      end;
  end;

  function AddURLString(Const AString: String): DWord;
  var urlstrrec : TUrlStrIndex;
  begin
    CheckURLStrBlockCanHold(AString);
    if FURLSTRStream.Size mod $4000 = 0 then
      FURLSTRStream.WriteByte(0);
      Result := FURLSTRStream.Position;
      UrlStrRec:=TUrlStrIndex.Create;
      UrlStrRec.UrlStr:=AString;
      UrlStrRec.UrlStrid:=result;
      FAvlUrlStr.Add(UrlStrRec);
      FURLSTRStream.WriteDWord(NToLE(DWord(0))); // URL Offset for topic after the the "Local" value
      FURLSTRStream.WriteDWord(NToLE(DWord(0))); // Offset of FrameName??
      if Length(AString) > 0 then
        FURLSTRStream.Write(AString[1], Length(AString));
      FURLSTRStream.WriteByte(0); //NT
  end;

  function LookupUrlString(const AUrl : String):DWord;
  var n :TAvlTreeNode;
  begin
    SpareUrlStr.UrlStr:=AUrl;
    n:=FAvlUrlStr.FindKey(SpareUrlStr,@CompareUrlStrs);
    if assigned(n) Then
      result:=TUrlStrIndex(n.data).UrlStrId
    else
      result:=AddUrlString(AUrl);
  end;


var UrlIndex : Integer;

begin
  if (Length(AURL) > 0) and (AURL[1] = '/') then Delete(AURL,1,1);
  UrlIndex:=LookupUrlString(AUrl);

  //if $1000 - (FURLTBLStream.Size mod $1000) = 4 then // we are at 4092
  if FURLTBLStream.Size and $FFC = $FFC then // faster :)
    FURLTBLStream.WriteDWord(0);
  Result := FURLTBLStream.Position;
  FURLTBLStream.WriteDWord(0);//($231e9f5c); //unknown
  FURLTBLStream.WriteDWord(NtoLE(TopicsIndex)); // Index of topic in #TOPICS
  FURLTBLStream.WriteDWord(NtoLE(UrlIndex));
end;

procedure TChmWriter.CheckFileMakeSearchable(AStream: TStream; AFileEntry: TFileEntryRec);
var
    TopicEntry: TTopicEntry;
    ATitle: String;
begin
  if Pos('.ht', AFileEntry.Name) > 0 then
  begin
    ATitle := FIndexedFiles.IndexFile(AStream, NextTopicIndex, FSearchTitlesOnly);
    AddTopic(ATitle,AFileEntry.Path+AFileEntry.Name,-1);
 end;
end;

function TChmWriter.AddTopic(ATitle,AnUrl:AnsiString;code:integer=-1):integer;

var
    TopicEntry: TTopicEntry;

begin
    anurl:=StringReplace(anurl, '\', '/', [rfReplaceAll]);
    if ATitle <> '' then
      TopicEntry.StringsOffset := AddString(ATitle)
    else
      TopicEntry.StringsOffset := $FFFFFFFF;
    result:=NextTopicIndex;
    TopicEntry.URLTableOffset := AddURL(AnUrl, Result);
    if code=-1 then
      begin
       if ATitle<>'' then
         TopicEntry.InContents := 6
       else
         TopicEntry.InContents := 2;
       if pos('#',AnUrl)>0 then
         TopicEntry.InContents := 0;
      end
     else
       TopicEntry.InContents := code;

    inc(FNrTopics);
    TopicEntry.Unknown := 0;
    TopicEntry.TocOffset := 0;
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.TocOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.StringsOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.URLTableOffset));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.InContents));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.Unknown));
end;

procedure TChmWriter.ScanSitemap(asitemap:TCHMSiteMap);
procedure scanitems(it:TChmSiteMapItems);

var i : integer;
    x : TChmSiteMapItem;
    s : string;
    strrec : TStringIndex;

begin
  for i:=0 to it.count -1 do
    begin
      x:=it.item[i];
//      if sanitizeurl(fbasepath,x.local,S) then   // sanitize, remove stuff etc.
//        begin
//          writeln(x.text,' : ',x.local,' ',x.url,' ' ,x.merge);

      if assigned(x.children) and (x.children.count>0) then
        scanitems(x.children);
    end;
end;
begin
 scanitems(asitemap.items);
end;

function TChmWriter.NextTopicIndex: Integer;
begin
  Result := FTopicsStream.Size div 16;
end;

procedure TChmWriter.AppendTOC(AStream: TStream);

var tmpstr : string;
begin
  fHasTOC := True;
  if fTocName = '' then
    tmpstr := defaulthhc
  else
    tmpstr := fTocName;
  PostAddStreamToArchive(tmpstr, '/', AStream, True);
end;

procedure TChmWriter.AppendBinaryTOCFromSiteMap(ASiteMap: TChmSiteMap);
var
  Header: TTOCIdxHeader;
  Entry: TTocEntry;
  EntryInfo: TTOCEntryPageBookInfo;


  EntryInfoStream,
  EntryTopicOffsetStream,
  EntryStream: TMemoryStream;

  TOCIDXStream: TMemoryStream;

  NextLevelItems,
  CurrentLevelItems: TFPList;
  i,j: Integer;
  MenuItem: TChmSiteMapItem;
  MenuItems: TChmSiteMapItems;
  TopicEntry: TTopicEntry;
  EntryCount: DWord = $29A;
  procedure FixParentBookFirstChildOffset(AChildOffset: DWord);
  var
    ParentEntry: TTOCEntryPageBookInfo;
  begin
    // read parent entry
    EntryInfoStream.Position := MenuItems.InternalData;
    EntryInfoStream.Read(ParentEntry, SizeOf(ParentEntry));
    // update child offset
    ParentEntry.FirstChildOffset:= NtoLE(DWord(4096 + AChildOffset));
    // write back to stream
    EntryInfoStream.Position := MenuItems.InternalData;
    EntryInfoStream.Write(ParentEntry, SizeOf(ParentEntry));
    // move to end of stream
    EntryInfoStream.Position := AChildOffset;
  end;

begin
  FillChar(Header, 4096, 0);
  // create streams
  TOCIDXStream := TMemoryStream.Create;
  EntryInfoStream := TMemoryStream.Create;
  EntryTopicOffsetStream := TMemoryStream.Create;
  EntryStream := TMemoryStream.Create;

  NextLevelItems := TFPList.Create;

  NextLevelItems.Add(ASiteMap.Items);

  if NextLevelItems.Count = 0 then
      FreeAndNil(NextLevelItems);

  while NextLevelItems <> nil do
  begin
    CurrentLevelItems := NextLevelItems;
    NextLevelItems := TFPList.Create;

    for i := 0 to CurrentLevelItems.Count-1 do
    begin
      MenuItems := TChmSiteMapItems(CurrentLevelItems.Items[i]);

      for j := 0 to MenuItems.Count-1 do
      begin
        MenuItem := MenuItems.Item[j];
        // first figure out the props
        EntryInfo.Props := 0;
        if MenuItem.Children.Count > 0 then
          EntryInfo.Props := EntryInfo.Props or TOC_ENTRY_HAS_CHILDREN;
        if Length(MenuItem.Local) > 0 then
          EntryInfo.Props := EntryInfo.Props or TOC_ENTRY_HAS_LOCAL;


      if EntryInfo.Props and TOC_ENTRY_HAS_LOCAL > 0 then
      begin
        // Write #TOPICS entry
        TopicEntry.TocOffset      := NtoLE(DWord(4096 + EntryInfoStream.Position));
        TopicEntry.StringsOffset  := NtoLE(AddString(MenuItem.Text));
        TopicEntry.URLTableOffset := NtoLE(AddURL(MenuItem.Local, NextTopicIndex));
        TopicEntry.InContents     := NtoLE(Word( 2 ));
        TopicEntry.Unknown        := 0;
        EntryInfo.TopicsIndexOrStringsOffset := NtoLE(Dword(NextTopicIndex));;
        FTopicsStream.Write(TopicEntry, SizeOf(TopicEntry));
        EntryTopicOffsetStream.WriteDWord(EntryInfo.TopicsIndexOrStringsOffset);

        // write TOCEntry
        Entry.PageBookInfoOffset:= NtoLE(4096 + EntryInfoStream.Position);
        Entry.IncrementedInt  := NtoLE(EntryCount);
        EntryStream.Write(Entry, SizeOf(Entry));
        Inc(EntryCount);

      end
      else
      begin
        EntryInfo.TopicsIndexOrStringsOffset := NtoLE(AddString(MenuItem.Text));
      end;


        // write TOCEntryInfo

        EntryInfo.Unknown1 := 0;
        EntryInfo.EntryIndex := NtoLE(Word(EntryCount - $29A)); //who knows how useful any of this is

        if MenuItems.InternalData <> maxLongint then
          EntryInfo.ParentPageBookInfoOffset := MenuItems.InternalData
        else
          EntryInfo.ParentPageBookInfoOffset := 0;

        if j = MenuItems.Count-1 then
          EntryInfo.NextPageBookOffset := 0
        else if (EntryInfo.Props and TOC_ENTRY_HAS_CHILDREN) > 0 then
          EntryInfo.NextPageBookOffset := 4096 + EntryInfoStream.Position + 28
        else
          EntryInfo.NextPageBookOffset := 4096 + EntryInfoStream.Position + 20;

        // Only if TOC_ENTRY_HAS_CHILDREN is set are these written
        EntryInfo.FirstChildOffset := 0; // we will update this when the child is written
        // in fact lets update the *parent* of this item now if needed
        if (j = 0) and (MenuItems.InternalData <> maxLongint) then
          FixParentBookFirstChildOffset(EntryInfoStream.Position);

        EntryInfo.Unknown3 := 0;

        // fix endian order
        EntryInfo.Props := NtoLE(EntryInfo.Props);
        EntryInfo.ParentPageBookInfoOffset := NtoLE(EntryInfo.ParentPageBookInfoOffset);
        EntryInfo.NextPageBookOffset := NtoLE(EntryInfo.NextPageBookOffset);

        if MenuItem.Children.Count > 0 then
        begin
          NextLevelItems.Add(MenuItem.Children);
          MenuItem.Children.InternalData := EntryInfoStream.Position;
        end;

        // write to stream
        EntryInfoStream.Write(EntryInfo, PageBookInfoRecordSize(@EntryInfo));
      end;
    end;

    FreeAndNil(CurrentLevelItems);
    if NextLevelItems.Count = 0 then
      FreeAndNil(NextLevelItems);
  end;

  // write all streams to TOCIdxStream and free everything
  EntryInfoStream.Position:=0;
  EntryTopicOffsetStream.Position:=0;
  EntryStream.Position:=0;

  Header.BlockSize := NtoLE(DWord(4096));
  Header.EntriesCount := NtoLE(DWord(EntryCount - $29A));
  Header.EntriesOffset := NtoLE(DWord(4096 + EntryInfoStream.Size + EntryTopicOffsetStream.Size));
  Header.TopicsOffset := NtoLE(DWord(4096 + EntryInfoStream.Size));

  TOCIDXStream.Write(Header, SizeOf(Header));

  TOCIDXStream.CopyFrom(EntryInfoStream, EntryInfoStream.Size);
  EntryInfoStream.Free;

  TOCIDXStream.CopyFrom(EntryTopicOffsetStream, EntryTopicOffsetStream.Size);
  EntryTopicOffsetStream.Free;

  TOCIDXStream.CopyFrom(EntryStream, EntryStream.Size);
  EntryStream.Free;

  TOCIDXStream.Position := 0;
  AppendBinaryTOCStream(TOCIDXStream);
  TOCIDXStream.Free;
end;

Const
      BinIndexIdent : array[0..1] of char = (CHR($3B),CHR($29));
      AlwaysX44     : Array[0..15] of char = ('X','4','4',#0,#0,#0,#0,#0,
                                              #0,#0,#0,#0,#0,#0,#0,#0);
      DataEntry     : Array[0..12] of Byte = ($00,$00,$00,$00,$05,$00,$00,$00,$80,$00,$00,$00,$00);
{
  IndexStream:=TMemoryStream.Create;
  IndexStream.Write(BinIndexIdent,2);
  IndexStream.Write(NToLE(word(2)),2);
  IndexStream.Write(NToLE(word(2048)),2);
  IndexStream.Write(AlwaysX44,sizeof(AlwaysX44));
  IndexStrem.Write (dword(0),2);
}

Const DefBlockSize  = 2048;

Type TIndexBlock = Array[0..DefBlockSize-1] of Byte;

procedure writeword(var p:pbyte;w:word); inline;

begin
  pword(p)^:=NToLE(w);
  inc(pword(p));
end;

procedure writedword(var p:pbyte;d:dword); inline;

begin
  pdword(p)^:=NToLE(d);
  inc(pdword(p));
end;

procedure TChmWriter.AppendBinaryIndexFromSiteMap(ASiteMap: TChmSiteMap;chw:boolean);

Var
  IndexStream : TMemoryStream;
  //n           : Integer;
  curblock    : TIndexBlock;    // current listing block being built
  TestBlock   : TIndexBlock;    // each entry is first built here. then moved to curblock
  curind      : integer;        // next byte to write in testblock.
  blocknr     : Integer;        // blocknr of block in testblock;
  lastblock   : Integer;        // blocknr of last block.
  Entries     : Integer;        // Number of entries in this block so far
  TotalEntries: Integer;        // Total number of entries
  MapEntries  : Integer;
  MapIndex    : Integer;
  indexblocknr: Integer;
  blockind    : Integer;        // next byte to write in blockn[blocknr]
  blockentries: Integer;        // entries so far ins blockn[blocknr]
  blockn      : Array Of TIndexBlock;
  BlockNPlus1 : Array of TIndexBlock;
  Mod13value  : integer;        // A value that is increased by 13 for each entry. (?!?!)
  EntryToIndex: boolean;        // helper var to make sure the first block is always indexed.
  blocknplusindex   : Integer;  // blocks in level n+1 (second part)
  blocknplusentries : Integer;  // The other blocks indexed on creation.
  datastream,mapstream,propertystream : TMemoryStream;

procedure preparecurrentblock(force:boolean);
var p: PBTreeBlockHeader;
begin
  {$ifdef binindex}
  writeln('prepcurblock ' ,Entries,' ',lastblock,' ' ,blocknr,' ',indexstream.position);
  {$endif}
  p:=@curblock[0];
  fillchar(p^,sizeof(TBtreeBlockHeader),#0);
  p^.Length:=NToLE(Defblocksize-curind);
  p^.NumberOfEntries:=Entries;
  p^.IndexOfPrevBlock:=cardinal(lastblock); // lastblock can be -1, avoid rangecheck
  p^.IndexOfNextBlock:=Blocknr;
  if force and (blocknr=0) then   // only one listblock -> no indexblocks.
    p^.IndexOfNextBlock:=dword(-1);
  IndexStream.Write(curblock[0],Defblocksize);
  fillchar(curblock[0],DefBlockSize,#0);
  MapStream.Write(NToLE(MapEntries),sizeof(dword));
  MapStream.Write(NToLE(BlockNr),Sizeof(DWord));
  MapEntries:=TotalEntries;
  curind:=sizeof(TBtreeBlockHeader);   // index into current block;
  lastblock:=blocknr;
  inc(blocknr);
  Entries:=0;
  {$ifdef binindex}
  writeln('prepcurblock post' , indexstream.position);
  {$endif}
end;

procedure prepareindexblockn(listingblocknr:integer);
var p:PBTreeIndexBlockHeader;
begin
  {$ifdef binindex}
  writeln('prepindexblockn');
  {$endif}
  p:=@Blockn[IndexBlockNr];
  p^.Length:=defblocksize-BlockInd;
  p^.NumberOfEntries:=BlockEntries;

// p^.IndexOfChildBlock  // already entered on block creation, since of first entry, not last.
  inc(Indexblocknr);
  BlockEntries:=0;
  BlockInd:=0;
  if Indexblocknr>=length(blockn) then
    begin
      setlength(blockn,length(blockn)+1);  // larger increments also possible. #blocks is kept independantly.
      fillchar(blockn[0][0],sizeof(blockn[0]),#0);
    end;
  p:=@Blockn[IndexBlockNr];
  p^.IndexOfChildBlock:=ListingBlockNr;
  blockind:=sizeof(TBTreeIndexBlockHeader);
end;

procedure finalizeindexblockn(p:pbyte;var ind:integer;xEntries:integer);
var ph:PBTreeIndexBlockHeader;
begin
  ph:=PBTreeIndexBlockHeader(p);
  ph^.Length:=defblocksize-Ind;
  ph^.NumberOfEntries:=xEntries;
// p^.IndexOfChildBlock  // already entered on block creation, since of first entry, not last.
//  inc(Ind);
end;

procedure CurEntryToIndex(entrysize:integer);
var p,pentry : pbyte;
    indexentrysize : integer;
begin
  {$ifdef binindex}
  writeln('curentrytoindex ', entrysize);
  {$endif}

  indexentrysize:=entrysize-sizeof(dword);         // index entry is 4 bytes shorter, and only the last dword differs
  if (blockind+indexentrysize)>=Defblocksize then
    prepareindexblockn(blocknr);
  p:=@blockn[Indexblocknr][blockind];
  move(testblock[0],p^,indexentrysize);
  pentry:=@p[indexentrysize-sizeof(dword)];         // ptr to last dword
  writedword(pentry,blocknr);                      // patch up the "index of child field"
  inc(blockind,indexentrysize);
end;

procedure CreateEntry(Item:TChmSiteMapItem;Str:WideString;commaatposition:integer);

var p      : pbyte;
    topicid: integer;
    seealso: Integer;
    entrysize:Integer;
    i      : Integer;
begin
  inc(TotalEntries);
  fillchar(testblock[0],DefBlockSize,#0);
  p:=@TestBlock[0];
  for i:=1 to Length(str) do
    WriteWord(p,Word(str[i]));   // write the wstr in little endian
  WriteWord(p,0);                // NT
//  if item.seealso='' then    // no seealso for now
    seealso:=0;
 // else
//    seealso:=2;
  WriteWord(p,seealso);          // =0 not a see also 2 =seealso
  WriteWord(p,0);                // Entrydepth.  We can't know it, so write 2.
  WriteDword(p,commaatposition); // position of the comma
  WriteDword(p,0);               // unused 0
  WriteDword(p,1);               // for now only local pair.
  TopicId:=AddTopic(Item.Text,item.Local);
  WriteDword(p,TopicId);
  // if seealso then _here_ a wchar NT string with seealso?
  WriteDword(p,1);               // always 1 (unknown);
  WriteDword(p,mod13value);      //a value that increments with 13.
  mod13value:=mod13value+13;
  entrysize:=p-pbyte(@testblock[0]);
  {$ifdef binindex}
    writeln(curind, ' ',entrysize, ' ',defblocksize);
    writeln('curstr ',str,' ',commaatposition);
  {$endif}
  if (curind+entrysize)>=Defblocksize then
    begin
      {$ifdef binindex}
      writeln('larger!');
      {$endif}
      preparecurrentblock(False);
      EntrytoIndex:=true;
    end;
  if EntryToIndex Then
    begin
      {$ifdef binindex}
      writeln('entrytoindex');
      {$endif}
      CurEntryToIndex(entrysize);
      EntryToIndex:=False;
    end;
  move(testblock[0],curblock[curind],entrysize);
  inc(curind,entrysize);
  datastream.write(DataEntry,Sizeof(DataEntry));
  inc(Entries);
end;

procedure MoveIndexEntry(nr:integer;bytes:integer;childblock:integer);
var
  pscr,pdest : pbyte;
begin
  {$ifdef binindex}
    writeln(' moveindexentry ',nr,' bytes:',bytes,' childblock:',childblock);
    flush(stdout);
  {$endif}

  if ((blockind+bytes)>=defblocksize) then
    begin
      {$ifdef binindex}
      writeln(' in scalecheck  ',blockind);
      flush(stdout);
      {$endif}

      FinalizeIndexBlockn(@blocknplus1[blocknplusindex][0],blockind,blocknplusentries);
      inc(blocknplusindex);
      if blocknplusindex>=length(blocknplus1) then
        begin
          setlength(blocknplus1,length(blocknplus1)+1);
          fillchar(blocknplus1[length(blocknplus1)-1][0],sizeof(blocknplus1[0]),#0);
        end;
      blockInd:=Sizeof(TBTreeIndexBlockHeader);
      pdword(@blocknplus1[blocknplusindex][0])[4]:=NToLE(ChildBlock);  /// init 2nd level index to first 1st level index block
      end;
  {$ifdef binindex}
    writeln(' len:',length(blocknplus1),' blockind:',blockind,' index:',blocknplusindex);
    flush(stdout);
  {$endif}

  // copy entry from one indexblock to another
  pscr:=@blockn[nr][sizeof(TBtreeIndexBlockHeader)];
  pdest:=@blocknplus1[blocknplusindex][blockind];
  move(pscr^,pdest^,bytes);
  pdword(@pdest[bytes-sizeof(dword)])^:=NToLE(childblock);    // correcting the childindex
  inc (blockind,bytes);
  inc(blocknplusentries); // not needed for writing, but used to check if something has been written. End condition
end;

function ScanIndexBlock(blk:Pbyte):Integer;

var start : pbyte;
    n     : Integer;
    i     : Integer;
begin
  start:=@blk[sizeof(TBtreeIndexBlockHeader)];
  blk:=start;
  while pword(blk)^<>0 do   // skip wchar
    inc(pword(blk));
  inc(pword(blk));          // skip NT
  inc(pword(blk));          // skip see also
  inc(pword(blk));          // skip depth
  inc(pdword(blk));         // skip Character Index.
  inc(pdword(blk));          // skip always  0
  n:=LEToN(pdword(blk)^);
  inc(pdword(blk));          // skip nr of pairs.
  for i:= 1 to n do
      inc(pdword(blk));          // skip <n> topicids
  inc(pdword(blk));          // skip childindex
  Result:=blk-start;
end;

procedure CombineWithChildren(ParentItem:TChmSiteMapItem;Str:WideString;commaatposition:integer;first:boolean);
var i    : Integer;
    Item : TChmSiteMapItem;
begin
  if ParentItem.Children.Count = 0 Then
    Begin
     // comment/fix next
     //   if commatposition=length(str) then commaatposition:=0;
       if first then
        CreateEntry(ParentItem,Str,0)
       else
        CreateEntry(ParentItem,Str,commaatposition);
    End
  Else
    for i:=0 to ParentItem.Children.Count-1 do
      begin
        item := TChmSiteMapItem(ParentItem.Children.Item[i]);
        if first Then
          CombineWithChildren(Item,Str+', '+item.text,commaatposition+2,false)
        else
          CombineWithChildren(Item,Str+', '+item.text,commaatposition,false);
      end;
end;

Var i             : Integer;
    Key           : WideString;
    Item          : TChmSiteMapItem;
    ListingBlocks : Integer;
    EntryBytes    : Integer;
    Hdr           : TBTreeHeader;
    TreeDepth     : Integer;

{$ifdef binindex}
procedure printloopvars(i:integer);

begin
  Writeln('location :' ,i, ' blocknr :', blocknr,' level:',TreeDepth);
  Writeln('blockn      length: ',length(blockn),' indexblocknr: ',indexblocknr,' blockind ',blockind);
  Writeln('blocknplus1 length: ',length(blocknplus1),' blocknplusindex:',blocknplusindex,' entries:',blocknplusentries);
  flush(stdout);
end;
{$endif}
begin
  {$ifdef binindex}
    writeln('starting index');
  {$endif}
  IndexStream:=TMemoryStream.Create;
  indexstream.size:=sizeof(TBTreeHeader);
  IndexStream.position:=Sizeof(TBTreeHeader);
  datastream:=TMemoryStream.Create;
  mapstream :=TMemoryStream.Create;
  mapstream.size:=2;
  mapstream.position:=2;
  propertystream :=TMemoryStream.Create;
  propertystream.write(NToLE(0),sizeof(4));
  // we iterate over all entries and write listingblocks directly to the stream.
  // and the first (and maybe last) level is written to blockn.
  // we can't do higher levels yet because we don't know how many listblocks we get
  BlockNr     :=0;   // current block number
  Lastblock   :=-1;  // previous block nr or -1 if none.
  Entries     :=0;   // entries in this block
  TotalEntries:=0;   // entries so far.
  Mod13value  :=0;   // value that increments by 13 entirely.
  indexblocknr:=0;   // nr of first index block.
  BlockEntries:=0;   // entries into current block;
  MapEntries  :=0;   // entries before the current listing block, for MAP file
  TreeDepth   :=0;

  fillchar(testblock[0],DefBlockSize,#0);
  fillchar(curblock[0],DefBlockSize,#0);
  curind      :=sizeof(TBTreeBlockHeader);      // index into current listing block;
  blockind    :=sizeof(TBtreeIndexBlockHeader); // index into current index block

  Setlength(blockn,1);
  fillchar(blockn[0][0],sizeof(blockn[0]),#0);
  pdword(@blockn[0][4])^:=NToLE(0);  /// init first listingblock nr to 0 in the first index block
  EntryToIndex   := True;
  {$ifdef binindex}
  writeln('items:',asitemap.items.count);
  {$endif}
  for i:=0 to ASiteMap.Items.Count-1 do
    begin
      item := TChmSiteMapItem(ASiteMap.Items.Item[i]);
      key  :=Item.Text;
       {$ifdef binindex}
        writeln('item: ',i,' ',key);
       {$endif}

      {$ifdef chm_windowsbinindex}
      // append 2 to all index level 0 entries. This
      // so we can see if Windows loads the binary or textual index.
      CombineWithChildren(Item,Key+'2',length(key)+1,true);
      {$else}
      CombineWithChildren(Item,Key,length(key),true);
      {$endif}
    end;
  PrepareCurrentBlock(True);     // flush last listing block.

  Listingblocks:=blocknr;   // blocknr is from now on the number of the first block in blockn.
                            // we still need the # of listingblocks for the header though
  {$ifdef binindex}
    writeln('binindex: listingblocks : '+inttostr(listingblocks),' indexblocks: ',indexblocknr,' entries:',blockentries);
  {$endif}

  // we have now created and written the listing blocks, and created the first level of index in <blockn>
  // the following loop uses <blockn> to calculate the next level (in blocknplus1), then write out blockn,
  // and repeat until we have no entries left.

  // First we finalize the current set of blocks
  if blocknr>1 then
    begin
      if  Blockind<>sizeof(TBtreeIndexBlockHeader) Then
        begin
          {$ifdef binindex}
            writeln('finalizing level 1 index');
          {$endif}
          FinalizeIndexBlockN(@blockn[indexblocknr][0],blockind,blockentries); // also increasing indexblocknr
          inc(IndexBlockNr);
        end;
      {$ifdef binindex}
        writeln('binindex: listingblocks : '+inttostr(listingblocks),' indexblocks: ',indexblocknr,' entries:',blockentries);
      {$endif}


      while (Indexblocknr>1) do
        begin
          {$ifdef binindex}
            printloopvars(1);
          {$endif}

          blockind      :=sizeof(TBtreeIndexBlockHeader);
          pdword(@blockn[0][4])^:=NToLE(Listingblocks);  /// init 2nd level index to first 1st level index block
          blocknplusindex     :=0;
          blocknplusentries   :=0;
          if length(blocknplus1)<1 then
            begin
              Setlength(blocknplus1,1);
              fillchar(blocknplus1[0][0],sizeof(blocknplus1[0]),#0);
            end;

          EntryToIndex        :=True;
          {$ifdef binindex}
            printloopvars(2);
          {$endif}
          for i:=0 to Indexblocknr-1 do
            begin
              Entrybytes:=ScanIndexBlock(@blockn[i][0]);
    //          writeln('after scan ,',i, ' bytes: ',entrybytes,' blocknr:',blocknr,' indexblocknr:',indexblocknr,' to:',blocknr+i);
              MoveIndexEntry(i,Entrybytes,blocknr+i);
              indexStream.Write(blockn[i][0],defblocksize);
            end;

          {$ifdef binindex}
            printloopvars(3);
          {$endif}

          If Blockind<>sizeof(TBtreeIndexBlockHeader) Then
            begin
              {$ifdef binindex}
                logentry('finalizing');
              {$endif}
              FinalizeIndexBlockn(@blocknplus1[blocknplusindex][0],blockind,blocknplusentries);
              inc(blocknplusindex);
            end;

          inc(blocknr,indexblocknr);

          indexblocknr:=blocknplusindex;
          blockn:=copy(blocknplus1); setlength(blocknplus1,1);
          {$ifdef binindex}
            printloopvars(5);
          {$endif}

          inc(TreeDepth);
        end;
      indexStream.Write(blockn[0][0],defblocksize);
      inc(blocknr);
    end;
  // Fixup header.
  hdr.ident[0]:=chr($3B); hdr.ident[1]:=chr($29);
  hdr.flags          :=NToLE(word($2));           // bit $2 is always 1, bit $0400 1 if dir? (always on)
  hdr.blocksize      :=NToLE(word(defblocksize)); // size of blocks (2048)
  hdr.dataformat     :=AlwaysX44;           // "X44" always the same, see specs.
  hdr.unknown0       :=NToLE(0);            // always 0
  hdr.lastlstblock   :=NToLE(dword(ListingBlocks-1)); // index of last listing block in the file;
  hdr.indexrootblock :=NToLE(dword(blocknr-1));    // Index of the root block in the file.
  hdr.unknown1       :=NToLE(dword(-1));           // always -1
  hdr.nrblock        :=NToLE(blocknr);      // Number of blocks
  hdr.treedepth      :=NToLE(word(TreeDepth));    // The depth of the tree of blocks (1 if no index blocks, 2 one level of index blocks, ...)
  hdr.nrkeywords     :=NToLE(Totalentries); // number of keywords in the file.
  hdr.codepage       :=NToLE(dword(1252));         // Windows code page identifier (usually 1252 - Windows 3.1 US (ANSI))
  hdr.lcid           :=NToLE(0);            //  ???? LCID from the HHP file.
  if not chw then
    hdr.ischm        :=NToLE(dword(1))             // 0 if this a BTREE and is part of a CHW file, 1 if it is a BTree and is part of a CHI or CHM file
  else
    hdr.ischm        :=NToLE(0);
  hdr.unknown2       :=NToLE(dword(10031));        // Unknown. Almost always 10031. Also 66631 (accessib.chm, ieeula.chm, iesupp.chm, iexplore.chm, msoe.chm, mstask.chm, ratings.chm, wab.chm).
  hdr.unknown3       :=NToLE(0);            // unknown 0
  hdr.unknown4       :=NToLE(0);            // unknown 0
  hdr.unknown5       :=NToLE(0);            // unknown 0

  IndexStream.Position:=0;
  IndexStream.write(hdr,sizeof(hdr));
  {$ifdef binindex}
    logentry('before append');
  {$endif}

  AppendBinaryIndexStream(IndexStream,datastream,MapStream,PropertyStream,chw);
  IndexStream.Free;
  PropertyStream.Free;
  MapStream.Free;
  DataStream.Free;
  FHasKLinks:=TotalEntries>0;
  {$ifdef binindex}
    writeln('end index');
  {$endif}

end;

procedure TChmWriter.AppendBinaryTOCStream(AStream: TStream);
begin
  AddStreamToArchive('#TOCIDX', '/', AStream, True);
end;

procedure TChmWriter.AppendBinaryIndexStream(IndexStream,DataStream,MapStream,Propertystream: TStream;chw:boolean);

procedure stadd(fn:string;stream:TStream);

begin
  Stream.Position:=0;
  if CHW then
    fn:=uppercase(fn);
  {$ifdef binindex}
    logentry('before append '+fn);
  {$endif}
  AddStreamToArchive(fn,'/$WWKeywordLinks/',stream,True);
end;

begin
  AddDummyALink;
  stadd('BTree',IndexStream);
  stadd('Data', DataStream);
  stadd('Map' , MapStream);
  stadd('Property', PropertyStream);
end;

procedure TChmWriter.AppendIndex(AStream: TStream);
var tmpstr : string;
begin
  FHasIndex := True;
  if fIndexName = '' then
    tmpstr:=defaulthhk
  else
    tmpstr:=fIndexName;
  PostAddStreamToArchive(tmpstr, '/', AStream, True);
end;

procedure TChmWriter.AppendSearchDB(AName: String; AStream: TStream);
begin
  PostAddStreamToArchive(AName, '/', AStream);
end;


procedure TChmWriter.AddContext(AContext: DWord; ATopic: String);
var
  Offset: DWord;
begin
  if FContextStream = nil then begin
    FContextStream:=TMemoryStream.Create;
    // #IVB starts with a dword which is the size of the stream - sizeof(dword)
    FContextStream.WriteDWord(0);
    // we will update this when we write the file to the final stream
  end;
  // an entry is a context id and then the offset of the name of the topic in the strings file
  FContextStream.WriteDWord(NToLE(AContext));
  Offset := NToLE(AddString(ATopic));
  FContextStream.WriteDWord(Offset);
end;

procedure TChmWriter.AddDummyALink;
var stream  : TMemoryStream;
begin
    stream:=tmemorystream.create;
    stream.WriteDWord(0);
    stream.position:=0;
    AddStreamToArchive('Property','/$WWAssociativeLinks/',stream,True);
    stream.free;
end;

procedure TChmWriter.Setwindows(AWindowList: TObjectList);

var i : integer;
    x : TCHMWindow;
begin
  FWindows.Clear;
  for i:=0 to AWindowList.count -1 do
    begin
      x:=TChmWindow.Create;
      x.assign(TChmWindow(AWindowList[i]));
      Fwindows.Add(x);
    end;
end;

procedure TChmWriter.SetMergefiles(src:TStringList);
var i : integer;
begin
  FMergeFiles.Clear;
  for i:=0 to Src.count -1 do
      FMergefiles.add(src[i]);
=======
  lzx_finish(LZXdata, nil);
  {$ELSE}
  Compressor := TLZXCompressor.Create(4);
  Compressor.OnChunkDone  :=@LTChunkDone;
  Compressor.OnGetData    :=@LTGetData;
  Compressor.OnIsEndOfFile:=@LTIsEndOfFile;
  Compressor.OnMarkFrame  :=@LTMarkFrame;
  Compressor.Execute(True);
  //Sleep(20000);
  Compressor.Free;
  {$ENDIF}
end;


procedure TChmWriter.WriteSystem;
var
  Entry: TFileEntryRec;
  TmpStr: String;
  TmpTitle: String;
const
  VersionStr = 'HHA Version 4.74.8702'; // does this matter?
begin


  // this creates the /#SYSTEM file
  Entry.Name := '#SYSTEM';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset := FSection0.Position;

 { if FileExists('#SYSTEM') then
  begin
    TmpStream := TMemoryStream.Create;
    TmpStream.LoadFromFile('#SYSTEM');
    TmpStream.Position := 0;
    FSection0.CopyFrom(TmpStream, TmpStream.Size);
  end;                                    }
  // EntryCodeOrder: 10 9 4 2 3 16 6 0 1 5
  FSection0.WriteDWord(NToLE(Word(3))); // Version
  if Title <> '' then
    TmpTitle := Title
  else
    TmpTitle := 'default';

  // Code -> Length -> Data
  // 10
  FSection0.WriteWord(NToLE(Word(10)));
  FSection0.WriteWord(NToLE(Word(SizeOf(DWord))));
  FSection0.WriteDWord(NToLE(MilliSecondOfTheDay(Now)));
  // 9
  FSection0.WriteWord(NToLE(Word(9)));
  FSection0.WriteWord(NToLE(Word(SizeOf(VersionStr)+1)));
  FSection0.Write(VersionStr, SizeOf(VersionStr));
  FSection0.WriteByte(0);
  // 4 A struct that is only needed to set if full text search is on.
  FSection0.WriteWord(NToLE(Word(4)));
  FSection0.WriteWord(NToLE(Word(36))); // size

  FSection0.WriteDWord(NToLE(DWord($0409)));
  FSection0.WriteDWord(1);
  FSection0.WriteDWord(NToLE(DWord(Ord(FFullTextSearch and FFullTextSearchAvailable))));
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  // two for a QWord
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);




  ////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // 2  default page to load
  if FDefaultPage <> '' then begin
    FSection0.WriteWord(NToLE(Word(2)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultPage)+1)));
    FSection0.Write(FDefaultPage[1], Length(FDefaultPage));
    FSection0.WriteByte(0);
  end;
  // 3  Title
  if FTitle <> '' then begin
    FSection0.WriteWord(NToLE(Word(3)));
    FSection0.WriteWord(NToLE(Word(Length(FTitle)+1)));
    FSection0.Write(FTitle[1], Length(FTitle));
    FSection0.WriteByte(0);
  end;

  // 16 Default Font
  if FDefaultFont <> '' then begin
    FSection0.WriteWord(NToLE(Word(16)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultFont)+1)));
    FSection0.Write(FDefaultFont[1], Length(FDefaultFont));
    FSection0.WriteByte(0);
  end;

  // 6
  // unneeded. if output file is :  /somepath/OutFile.chm the value here is outfile(lowercase)
  {FSection0.WriteWord(6);
  FSection0.WriteWord(Length('test1')+1);
  Fsection0.Write('test1', 5);
  FSection0.WriteByte(0);}

  // 0 Table of contents filename
  if FHasTOC then begin
    if fTocName ='' then
      TmpStr := DefaultHHC
    else
      TmpStr := fTocName;
    FSection0.WriteWord(0);
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 1
  // hhk Index
  if FHasIndex then begin
    if fIndexName='' then
      TmpStr := DefaultHHK
    else
      TmpStr := fIndexName;
    FSection0.WriteWord(NToLE(Word(1)));
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 5 Default Window

  if FDefaultWindow<>'' then
    begin
      FSection0.WriteWord(NTOLE(Word(5)));
      tmpstr:=FDefaultWindow;
      FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
      FSection0.Write(TmpStr[1], Length(TmpStr));
      FSection0.WriteByte(0);
    end;

  // 7 Binary Index
  if FHasBinaryIndex then
  begin
    {$ifdef binindex}
      logentry('binary index!');
    {$endif}
    FSection0.WriteWord(NToLE(Word(7)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;

  // 11 Binary TOC
  if FHasBinaryTOC then
  begin
    FSection0.WriteWord(NToLE(Word(11)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;


  Entry.DecompressedSize := FSection0.Position - Entry.DecompressedOffset;
  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteITBITS;
var
  Entry: TFileEntryRec;
begin
  // This is an empty and useless file
  Entry.Name := '#ITBITS';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset :=0;// FSection0.Position;
  Entry.DecompressedSize := 0;

  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteSTRINGS;
begin
  if FStringsStream.Size = 0 then;
    FStringsStream.WriteByte(0);
  FStringsStream.Position := 0;
  PostAddStreamToArchive('#STRINGS', '/', FStringsStream);
end;

procedure TChmWriter.WriteTOPICS;
//var
  //FHits: Integer;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  FTopicsStream.Position := 0;
  PostAddStreamToArchive('#TOPICS', '/', FTopicsStream);
 // I commented the code below since the result seemed unused
 // FHits:=0;
 //   FIndexedFiles.ForEach(@IterateWord,FHits);
end;

procedure TChmWriter.WriteIVB;
begin
  if FContextStream = nil then exit;

  FContextStream.Position := 0;
  // the size of all the entries
  FContextStream.WriteDWord(NToLE(DWord(FContextStream.Size-SizeOf(dword))));

  FContextStream.Position := 0;
  AddStreamToArchive('#IVB', '/', FContextStream);
end;

procedure TChmWriter.WriteURL_STR_TBL;
begin
  if FURLSTRStream.Size <> 0 then begin
    FURLSTRStream.Position := 0;
    PostAddStreamToArchive('#URLSTR', '/', FURLSTRStream);
  end;
  if FURLTBLStream.Size <> 0 then begin
    FURLTBLStream.Position := 0;
    PostAddStreamToArchive('#URLTBL', '/', FURLTBLStream);
  end;
>>>>>>> graemeg/cpstrnew
end;

=======
end;


procedure TChmWriter.WriteSystem;
var
  Entry: TFileEntryRec;
  TmpStr: String;
  TmpTitle: String;
const
  VersionStr = 'HHA Version 4.74.8702'; // does this matter?
begin


  // this creates the /#SYSTEM file
  Entry.Name := '#SYSTEM';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset := FSection0.Position;

 { if FileExists('#SYSTEM') then
  begin
    TmpStream := TMemoryStream.Create;
    TmpStream.LoadFromFile('#SYSTEM');
    TmpStream.Position := 0;
    FSection0.CopyFrom(TmpStream, TmpStream.Size);
  end;                                    }
  // EntryCodeOrder: 10 9 4 2 3 16 6 0 1 5
  FSection0.WriteDWord(NToLE(Word(3))); // Version
  if Title <> '' then
    TmpTitle := Title
  else
    TmpTitle := 'default';

  // Code -> Length -> Data
  // 10
  FSection0.WriteWord(NToLE(Word(10)));
  FSection0.WriteWord(NToLE(Word(SizeOf(DWord))));
  FSection0.WriteDWord(NToLE(MilliSecondOfTheDay(Now)));
  // 9
  FSection0.WriteWord(NToLE(Word(9)));
  FSection0.WriteWord(NToLE(Word(SizeOf(VersionStr)+1)));
  FSection0.Write(VersionStr, SizeOf(VersionStr));
  FSection0.WriteByte(0);
  // 4 A struct that is only needed to set if full text search is on.
  FSection0.WriteWord(NToLE(Word(4)));
  FSection0.WriteWord(NToLE(Word(36))); // size

  FSection0.WriteDWord(NToLE(DWord($0409)));
  FSection0.WriteDWord(1);
  FSection0.WriteDWord(NToLE(DWord(Ord(FFullTextSearch and FFullTextSearchAvailable))));
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  // two for a QWord
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);




  ////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // 2  default page to load
  if FDefaultPage <> '' then begin
    FSection0.WriteWord(NToLE(Word(2)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultPage)+1)));
    FSection0.Write(FDefaultPage[1], Length(FDefaultPage));
    FSection0.WriteByte(0);
  end;
  // 3  Title
  if FTitle <> '' then begin
    FSection0.WriteWord(NToLE(Word(3)));
    FSection0.WriteWord(NToLE(Word(Length(FTitle)+1)));
    FSection0.Write(FTitle[1], Length(FTitle));
    FSection0.WriteByte(0);
  end;

  // 16 Default Font
  if FDefaultFont <> '' then begin
    FSection0.WriteWord(NToLE(Word(16)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultFont)+1)));
    FSection0.Write(FDefaultFont[1], Length(FDefaultFont));
    FSection0.WriteByte(0);
  end;

  // 6
  // unneeded. if output file is :  /somepath/OutFile.chm the value here is outfile(lowercase)
  {FSection0.WriteWord(6);
  FSection0.WriteWord(Length('test1')+1);
  Fsection0.Write('test1', 5);
  FSection0.WriteByte(0);}

  // 0 Table of contents filename
  if FHasTOC then begin
    if fTocName ='' then
      TmpStr := DefaultHHC
    else
      TmpStr := fTocName;
    FSection0.WriteWord(0);
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 1
  // hhk Index
  if FHasIndex then begin
    if fIndexName='' then
      TmpStr := DefaultHHK
    else
      TmpStr := fIndexName;
    FSection0.WriteWord(NToLE(Word(1)));
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 5 Default Window

  if FDefaultWindow<>'' then
    begin
      FSection0.WriteWord(NTOLE(Word(5)));
      tmpstr:=FDefaultWindow;
      FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
      FSection0.Write(TmpStr[1], Length(TmpStr));
      FSection0.WriteByte(0);
    end;

  // 7 Binary Index
  if FHasBinaryIndex then
  begin
    {$ifdef binindex}
      logentry('binary index!');
    {$endif}
    FSection0.WriteWord(NToLE(Word(7)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;

  // 11 Binary TOC
  if FHasBinaryTOC then
  begin
    FSection0.WriteWord(NToLE(Word(11)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;


  Entry.DecompressedSize := FSection0.Position - Entry.DecompressedOffset;
  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteITBITS;
var
  Entry: TFileEntryRec;
begin
  // This is an empty and useless file
  Entry.Name := '#ITBITS';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset :=0;// FSection0.Position;
  Entry.DecompressedSize := 0;

  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteSTRINGS;
begin
  if FStringsStream.Size = 0 then;
    FStringsStream.WriteByte(0);
  FStringsStream.Position := 0;
  PostAddStreamToArchive('#STRINGS', '/', FStringsStream);
end;

procedure TChmWriter.WriteTOPICS;
//var
  //FHits: Integer;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  FTopicsStream.Position := 0;
  PostAddStreamToArchive('#TOPICS', '/', FTopicsStream);
 // I commented the code below since the result seemed unused
 // FHits:=0;
 //   FIndexedFiles.ForEach(@IterateWord,FHits);
end;

procedure TChmWriter.WriteIVB;
begin
  if FContextStream = nil then exit;

  FContextStream.Position := 0;
  // the size of all the entries
  FContextStream.WriteDWord(NToLE(DWord(FContextStream.Size-SizeOf(dword))));

  FContextStream.Position := 0;
  AddStreamToArchive('#IVB', '/', FContextStream);
end;

procedure TChmWriter.WriteURL_STR_TBL;
begin
  if FURLSTRStream.Size <> 0 then begin
    FURLSTRStream.Position := 0;
    PostAddStreamToArchive('#URLSTR', '/', FURLSTRStream);
  end;
  if FURLTBLStream.Size <> 0 then begin
    FURLTBLStream.Position := 0;
    PostAddStreamToArchive('#URLTBL', '/', FURLTBLStream);
  end;
end;

>>>>>>> graemeg/cpstrnew
=======
end;


procedure TChmWriter.WriteSystem;
var
  Entry: TFileEntryRec;
  TmpStr: String;
  TmpTitle: String;
const
  VersionStr = 'HHA Version 4.74.8702'; // does this matter?
begin


  // this creates the /#SYSTEM file
  Entry.Name := '#SYSTEM';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset := FSection0.Position;

 { if FileExists('#SYSTEM') then
  begin
    TmpStream := TMemoryStream.Create;
    TmpStream.LoadFromFile('#SYSTEM');
    TmpStream.Position := 0;
    FSection0.CopyFrom(TmpStream, TmpStream.Size);
  end;                                    }
  // EntryCodeOrder: 10 9 4 2 3 16 6 0 1 5
  FSection0.WriteDWord(NToLE(Word(3))); // Version
  if Title <> '' then
    TmpTitle := Title
  else
    TmpTitle := 'default';

  // Code -> Length -> Data
  // 10
  FSection0.WriteWord(NToLE(Word(10)));
  FSection0.WriteWord(NToLE(Word(SizeOf(DWord))));
  FSection0.WriteDWord(NToLE(MilliSecondOfTheDay(Now)));
  // 9
  FSection0.WriteWord(NToLE(Word(9)));
  FSection0.WriteWord(NToLE(Word(SizeOf(VersionStr)+1)));
  FSection0.Write(VersionStr, SizeOf(VersionStr));
  FSection0.WriteByte(0);
  // 4 A struct that is only needed to set if full text search is on.
  FSection0.WriteWord(NToLE(Word(4)));
  FSection0.WriteWord(NToLE(Word(36))); // size

  FSection0.WriteDWord(NToLE(DWord($0409)));
  FSection0.WriteDWord(1);
  FSection0.WriteDWord(NToLE(DWord(Ord(FFullTextSearch and FFullTextSearchAvailable))));
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  // two for a QWord
  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);

  FSection0.WriteDWord(0);
  FSection0.WriteDWord(0);




  ////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // 2  default page to load
  if FDefaultPage <> '' then begin
    FSection0.WriteWord(NToLE(Word(2)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultPage)+1)));
    FSection0.Write(FDefaultPage[1], Length(FDefaultPage));
    FSection0.WriteByte(0);
  end;
  // 3  Title
  if FTitle <> '' then begin
    FSection0.WriteWord(NToLE(Word(3)));
    FSection0.WriteWord(NToLE(Word(Length(FTitle)+1)));
    FSection0.Write(FTitle[1], Length(FTitle));
    FSection0.WriteByte(0);
  end;

  // 16 Default Font
  if FDefaultFont <> '' then begin
    FSection0.WriteWord(NToLE(Word(16)));
    FSection0.WriteWord(NToLE(Word(Length(FDefaultFont)+1)));
    FSection0.Write(FDefaultFont[1], Length(FDefaultFont));
    FSection0.WriteByte(0);
  end;

  // 6
  // unneeded. if output file is :  /somepath/OutFile.chm the value here is outfile(lowercase)
  {FSection0.WriteWord(6);
  FSection0.WriteWord(Length('test1')+1);
  Fsection0.Write('test1', 5);
  FSection0.WriteByte(0);}

  // 0 Table of contents filename
  if FHasTOC then begin
    if fTocName ='' then
      TmpStr := DefaultHHC
    else
      TmpStr := fTocName;
    FSection0.WriteWord(0);
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 1
  // hhk Index
  if FHasIndex then begin
    if fIndexName='' then
      TmpStr := DefaultHHK
    else
      TmpStr := fIndexName;
    FSection0.WriteWord(NToLE(Word(1)));
    FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
    FSection0.Write(TmpStr[1], Length(TmpStr));
    FSection0.WriteByte(0);
  end;
  // 5 Default Window

  if FDefaultWindow<>'' then
    begin
      FSection0.WriteWord(NTOLE(Word(5)));
      tmpstr:=FDefaultWindow;
      FSection0.WriteWord(NToLE(Word(Length(TmpStr)+1)));
      FSection0.Write(TmpStr[1], Length(TmpStr));
      FSection0.WriteByte(0);
    end;

  // 7 Binary Index
  if FHasBinaryIndex then
  begin
    {$ifdef binindex}
      logentry('binary index!');
    {$endif}
    FSection0.WriteWord(NToLE(Word(7)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;

  // 11 Binary TOC
  if FHasBinaryTOC then
  begin
    FSection0.WriteWord(NToLE(Word(11)));
    FSection0.WriteWord(NToLE(Word(4)));
    FSection0.WriteDWord(DWord(0)); // what is this number to be?
  end;


  Entry.DecompressedSize := FSection0.Position - Entry.DecompressedOffset;
  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteITBITS;
var
  Entry: TFileEntryRec;
begin
  // This is an empty and useless file
  Entry.Name := '#ITBITS';
  Entry.Path := '/';
  Entry.Compressed := False;
  Entry.DecompressedOffset :=0;// FSection0.Position;
  Entry.DecompressedSize := 0;

  FInternalFiles.AddEntry(Entry);
end;

procedure TChmWriter.WriteSTRINGS;
begin
  if FStringsStream.Size = 0 then;
    FStringsStream.WriteByte(0);
  FStringsStream.Position := 0;
  PostAddStreamToArchive('#STRINGS', '/', FStringsStream);
end;

procedure TChmWriter.WriteTOPICS;
//var
  //FHits: Integer;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  FTopicsStream.Position := 0;
  PostAddStreamToArchive('#TOPICS', '/', FTopicsStream);
 // I commented the code below since the result seemed unused
 // FHits:=0;
 //   FIndexedFiles.ForEach(@IterateWord,FHits);
end;

procedure TChmWriter.WriteIVB;
begin
  if FContextStream = nil then exit;

  FContextStream.Position := 0;
  // the size of all the entries
  FContextStream.WriteDWord(NToLE(DWord(FContextStream.Size-SizeOf(dword))));

  FContextStream.Position := 0;
  AddStreamToArchive('#IVB', '/', FContextStream);
end;

procedure TChmWriter.WriteURL_STR_TBL;
begin
  if FURLSTRStream.Size <> 0 then begin
    FURLSTRStream.Position := 0;
    PostAddStreamToArchive('#URLSTR', '/', FURLSTRStream);
  end;
  if FURLTBLStream.Size <> 0 then begin
    FURLTBLStream.Position := 0;
    PostAddStreamToArchive('#URLTBL', '/', FURLTBLStream);
  end;
end;

>>>>>>> origin/cpstrnew
procedure TChmWriter.WriteOBJINST;
var
  i: Integer;
  ObjStream: TMemoryStream;
  //Flags: Word;
begin
  ObjStream := TMemorystream.Create;
  // this file is needed to enable searches for the ms reader
  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(Dword(2))); // two entries

  ObjStream.WriteDWord(NtoLE(DWord(24))); // offset into file of entry
  ObjStream.WriteDWord(NtoLE(DWord(2691))); // size

  ObjStream.WriteDWord(NtoLE(DWord(2715))); // offset into file of entry
  ObjStream.WriteDWord(NtoLE(DWord(36))); // size

  // first entry
  // write guid 4662DAAF-D393-11D0-9A56-00C04FB68BF7
  ObjStream.WriteDWord(NtoLE($4662DAAF));
  ObjStream.WriteWord(NtoLE($D393));
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ObjStream.WriteWord(NtoLE($11D0));
  ObjStream.WriteWord(NtoLE($569A));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($F7);

  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(11));  // bit flags
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE($00000000));
  ObjStream.WriteDWord(NtoLE($00000000));
  ObjStream.WriteDWord(NtoLE($00145555));
  ObjStream.WriteDWord(NtoLE($00000A0F));
  ObjStream.WriteWord(NtoLE($0100));
  ObjStream.WriteDWord(NtoLE($00030005));
  for i := 0 to 5 do
    ObjStream.WriteDWord($00000000);
  ObjStream.WriteWord($0000);
  // okay now the fun stuff
  for i := 0 to $FF do
  ObjStream.Write(ObjInstEntries[i], SizeOF(TObjInstEntry));
  {begin
    if i = 1 then
      Flags := 7
    else
      Flags := 0;
    if (i >= $41) and (i <= $5A) then
      Flags := Flags or 2;
    if (i >= $61) and (i <= $7A) then
      Flags := Flags or 1;
    if i = $27 then
      Flags := Flags or 6;
    ObjStream.WriteWord(NtoLE(Flags));
    ObjStream.WriteWord(NtoLE(Word(i)));
    if (i >= $41) and (i <= $5A) then
      ObjStream.WriteByte(NtoLE(i+$20))
    else
      ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteByte(NtoLE(i));
    ObjStream.WriteWord(NtoLE($0000));
  end;}
  ObjStream.WriteDWord(NtoLE($E66561C6));
  ObjStream.WriteDWord(NtoLE($73DF6561));
  ObjStream.WriteDWord(NtoLE($656F8C73));
  ObjStream.WriteWord(NtoLE($6F9C));
  ObjStream.WriteByte(NtoLE($65));
  // third bit of second entry
  // write guid 8FA0D5A8-DEDF-11D0-9A61-00C04FB68BF7
  ObjStream.WriteDWord(NtoLE($8FA0D5A8));
  ObjStream.WriteWord(NtoLE($DEDF));
  ObjStream.WriteWord(NtoLE($11D0));
  ObjStream.WriteWord(NtoLE($619A));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($F7);

  ObjStream.WriteDWord(NtoLE($04000000));
  ObjStream.WriteDWord(NtoLE(DWord(1)));
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE(DWord(0)));

  // second entry
  // write guid 4662DAB0-D393-11D0-9A56-00C04FB68B66
  ObjStream.WriteDWord(NtoLE($4662DAB0));
  ObjStream.WriteWord(NtoLE($D393));
  ObjStream.WriteWord(NtoLE($11D0));
  ObjStream.WriteWord(NtoLE($569A));
  ObjStream.WriteByte($00);
  ObjStream.WriteByte($C0);
  ObjStream.WriteByte($4F);
  ObjStream.WriteByte($B6);
  ObjStream.WriteByte($8B);
  ObjStream.WriteByte($66);

  ObjStream.WriteDWord(NtoLE(DWord(666))); // not kidding
  ObjStream.WriteDWord(NtoLE(DWord(1252)));
  ObjStream.WriteDWord(NtoLE(DWord(1033)));
  ObjStream.WriteDWord(NtoLE(DWord(10031)));
  ObjStream.WriteDWord(NtoLE(DWord(0)));

  ObjStream.Position := 0;
  AddStreamToArchive('$OBJINST', '/', ObjStream, True);
  ObjStream.Free;

end;

procedure TChmWriter.WriteFiftiMain;
var
  SearchWriter: TChmSearchWriter;
begin
  if FTopicsStream.Size = 0 then
    Exit;
  SearchWriter := TChmSearchWriter.Create(FFiftiMainStream, FIndexedFiles);
  // do not add an empty $FIftiMain
  if not SearchWriter.HasData then
  begin
    FFullTextSearchAvailable := False;
    SearchWriter.Free;
    Exit;
  end;
  FFullTextSearchAvailable := True;
  SearchWriter.WriteToStream;
  SearchWriter.Free;

  if FFiftiMainStream.Size = 0 then
    Exit;

  FFiftiMainStream.Position := 0;
  PostAddStreamToArchive('$FIftiMain', '/', FFiftiMainStream);
end;

procedure TChmWriter.WriteWindows;
Var WindowStream : TMemoryStream;
    i,j          : Integer;
    win          : TChmWindow;
begin
  if FWindows.Count>0 then
    begin
      WindowStream:=TMemoryStream.Create;
      WindowStream.WriteDword(NToLE(dword(FWindows.Count)));
      WindowStream.WriteDword(NToLE(dword(196))); // 1.1 or later. 188 is old style.
      for i:=0 to FWindows.Count-1 Do
        begin
          Win:=TChmWindow(FWindows[i]);
          WindowStream.WriteDword(NToLE(dword(196 )));                   //  0 size of entry.
          WindowStream.WriteDword(NToLE(dword(0 )));                     //  4 unknown (bool Unicodestrings?)
          WindowStream.WriteDword(NToLE(addstring(win.window_type )));   //  8 Arg 0, name of window
          WindowStream.WriteDword(NToLE(dword(win.flags )));             //  C valid fields
          WindowStream.WriteDword(NToLE(dword(win.nav_style)));          // 10 arg 10 navigation pane style
          WindowStream.WriteDword(NToLE(addstring(win.title_bar_text))); // 14 Arg 1,  title bar text
          WindowStream.WriteDword(NToLE(dword(win.styleflags)));         // 18 Arg 14, style flags
          WindowStream.WriteDword(NToLE(dword(win.xtdstyleflags)));      // 1C Arg 15, xtd style flags
          WindowStream.WriteDword(NToLE(dword(win.left)));               // 20 Arg 13, rect.left
          WindowStream.WriteDword(NToLE(dword(win.top)));                // 24 Arg 13, rect.top
          WindowStream.WriteDword(NToLE(dword(win.right)));              // 28 Arg 13, rect.right
          WindowStream.WriteDword(NToLE(dword(win.bottom)));             // 2C Arg 13, rect.bottom
          WindowStream.WriteDword(NToLE(dword(win.window_show_state)));  // 30 Arg 16, window show state
          WindowStream.WriteDword(NToLE(dword(0)));                      // 34  -    , HWND hwndhelp                OUT: window handle"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 38  -    , HWND hwndcaller              OUT: who called this window"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 3C  -    , HH_INFO_TYPE paINFO_TYPES    IN: Pointer to an array of Information Types"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 40  -    , HWND hwndtoolbar             OUT: toolbar window in tri-pane window"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 44  -    , HWND hwndnavigation          OUT: navigation window in tri-pane window"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 48  -    , HWND hwndhtml                OUT: window displaying HTML in tri-pane window"
          WindowStream.WriteDword(NToLE(dword(win.navpanewidth)));       // 4C Arg 11, width of nav pane
          WindowStream.WriteDword(NToLE(dword(0)));                      // 50  -    , rect.left,   OUT:Specifies the coordinates of the Topic pane
          WindowStream.WriteDword(NToLE(dword(0)));                      // 54  -    , rect.top ,   OUT:Specifies the coordinates of the Topic pane
          WindowStream.WriteDword(NToLE(dword(0)));                      // 58  -    , rect.right,  OUT:Specifies the coordinates of the Topic pane
          WindowStream.WriteDword(NToLE(dword(0)));                      // 5C  -    , rect.bottom, OUT:Specifies the coordinates of the Topic pane
          WindowStream.WriteDword(NToLE(addstring(win.toc_file)));       // 60 Arg 2,  toc file
          WindowStream.WriteDword(NToLE(addstring(win.index_file)));     // 64 Arg 3,  index file
          WindowStream.WriteDword(NToLE(addstring(win.default_file)));   // 68 Arg 4,  default file
          WindowStream.WriteDword(NToLE(addstring(win.home_button_file))); // 6c Arg 5,  home button file.
          WindowStream.WriteDword(NToLE(dword(win.buttons)));            // 70 arg 12,
          WindowStream.WriteDword(NToLE(dword(win.navpane_initially_closed))); // 74 arg 17
          WindowStream.WriteDword(NToLE(dword(win.navpane_default)));    // 78 arg 18,
          WindowStream.WriteDword(NToLE(dword(win.navpane_location)));   // 7C arg 19,
          WindowStream.WriteDword(NToLE(dword(win.wm_notify_id)));       // 80 arg 20,
          for j:=0 to 4 do
            WindowStream.WriteDword(NToLE(dword(0)));                    // 84  -      byte[20] unknown -  "BYTE tabOrder[HH_MAX_TABS + 1]; // IN/OUT: tab order: Contents, Index, Search, History, Favorites, Reserved 1-5, Custom tabs"
          WindowStream.WriteDword(NToLE(dword(0)));                      // 94  -      int cHistory; // IN/OUT: number of history items to keep (default is 30)
          WindowStream.WriteDword(NToLE(addstring(win.Jumpbutton_1_Text)));  // 9C Arg 7,  The text of the Jump 1 button.
          WindowStream.WriteDword(NToLE(addstring(win.Jumpbutton_2_Text)));  // A0 Arg 9,  The text of the Jump 2 button.
          WindowStream.WriteDword(NToLE(addstring(win.Jumpbutton_1_File)));  // A4 Arg 6,  The file shown for Jump 1 button.
          WindowStream.WriteDword(NToLE(addstring(win.Jumpbutton_2_File)));  // A8 Arg 8,  The file shown for Jump 1 button.
          for j:=0 to 3 do
            WindowStream.WriteDword(NToLE(dword(0)));                    // AA  -      byte[16] (TRECT) "RECT rcMinSize; // Minimum size for window (ignored in version 1)"
          //   1.1+ fields
          WindowStream.WriteDword(NToLE(dword(0)));                      // BC -       int cbInfoTypes; // size of paInfoTypes;
          WindowStream.WriteDword(NToLE(dword(0)));                      // C0  -      LPCTSTR pszCustomTabs; // multiple zero-terminated strings
        end;
      WindowStream.Position := 0;
      AddStreamToArchive('#WINDOWS', '/', WindowStream, True);
      WindowStream.Free;
    end;
end;

procedure TChmWriter.WriteInternalFilesAfter;
begin
  // This creates and writes the #ITBITS (empty) file to section0
  WriteITBITS;
  // This creates and writes the #SYSTEM file to section0
  WriteSystem;
end;

procedure TChmWriter.WriteFinalCompressedFiles;
begin
  inherited WriteFinalCompressedFiles;
  WriteTOPICS;
  WriteURL_STR_TBL;
  WriteSTRINGS;
  WriteWINDOWS;
  WriteFiftiMain;
end;

procedure TChmWriter.FileAdded(AStream: TStream; const AEntry: TFileEntryRec);
begin
  inherited FileAdded(AStream, AEntry);
  if FullTextSearch then
    CheckFileMakeSearchable(AStream, AEntry);
end;

procedure TChmWriter.WriteInternalFilesBefore;
begin
  inherited WriteInternalFilesBefore;
  WriteIVB;
  WriteOBJINST;
end;

constructor TChmWriter.Create(AOutStream: TStream; FreeStreamOnDestroy: Boolean);
begin
  inherited Create(AOutStream, FreeStreamOnDestroy);
  FStringsStream := TmemoryStream.Create;
  FTopicsStream := TMemoryStream.Create;
  FURLSTRStream := TMemoryStream.Create;
  FURLTBLStream := TMemoryStream.Create;
  FFiftiMainStream := TMemoryStream.Create;
  FIndexedFiles := TIndexedWordList.Create;
  FAvlStrings   := TAVLTree.Create(@CompareStrings);    // dedupe strings
  FAvlURLStr    := TAVLTree.Create(@CompareUrlStrs);    // dedupe urltbl + binindex must resolve URL to topicid
  SpareString   := TStringIndex.Create;                 // We need an object to search in avltree
  SpareUrlStr   := TUrlStrIndex.Create;                 //    to avoid create/free circles we keep one in spare
                                                        //    for searching purposes
  FWindows      := TObjectlist.Create(True);
  FDefaultWindow:= '';
end;

destructor TChmWriter.Destroy;
begin
  if Assigned(FContextStream) then FContextStream.Free;
  FIndexedFiles.Free;
  FStringsStream.Free;
  FTopicsStream.Free;
  FURLSTRStream.Free;
  FURLTBLStream.Free;
  FFiftiMainStream.Free;
  SpareString.free;
  SpareUrlStr.free;
  FAvlUrlStr.FreeAndClear;
  FAvlUrlStr.Free;
  FAvlStrings.FreeAndClear;
  FAvlStrings.Free;
  FWindows.Free;

  inherited Destroy;
end;


function TChmWriter.AddString(AString: String): LongWord;
var
  NextBlock: DWord;
  Pos: DWord;
  n  : TAVLTreeNode;
  StrRec : TStringIndex;
begin
  // #STRINGS starts with a null char
  if FStringsStream.Size = 0 then FStringsStream.WriteByte(0);

  SpareString.TheString:=AString;
  n:=fAvlStrings.FindKey(SpareString,@CompareStrings);
  if assigned(n) then
   exit(TStringIndex(n.data).strid);

  // each entry is a null terminated string
  Pos := DWord(FStringsStream.Position);

  // Strings are contained in $1000 byte blocks and cannot cross blocks
  NextBlock := ($0000F000 and Pos) + $00001000;
  if Length(AString) + 1 > NextBlock then
  begin
    FStringsStream.Size:= NextBlock;
    FStringsStream.Position := NextBlock;
  end;

  Result := FStringsStream.Position;
  FStringsStream.WriteBuffer(AString[1], Length(AString));
  FStringsStream.WriteByte(0);

  StrRec:=TStringIndex.Create;
  StrRec.TheString:=AString;
  StrRec.Strid    :=Result;
  fAvlStrings.Add(StrRec);
end;

function TChmWriter.AddURL ( AURL: String; TopicsIndex: DWord ) : LongWord;

  procedure CheckURLStrBlockCanHold(Const AString: String);
  var
    Rem: LongWord;
    Len: LongWord;
  begin
    Rem := $4000 - (FURLSTRStream.Size mod $4000);
    Len := 9 + Length(AString);  // 2 dwords the string and NT
    if Rem < Len then
      while Rem > 0 do
      begin
        FURLSTRStream.WriteByte(0);
        Dec(Rem);
      end;
  end;

  function AddURLString(Const AString: String): DWord;
  var urlstrrec : TUrlStrIndex;
  begin
    CheckURLStrBlockCanHold(AString);
    if FURLSTRStream.Size mod $4000 = 0 then
      FURLSTRStream.WriteByte(0);
      Result := FURLSTRStream.Position;
      UrlStrRec:=TUrlStrIndex.Create;
      UrlStrRec.UrlStr:=AString;
      UrlStrRec.UrlStrid:=result;
      FAvlUrlStr.Add(UrlStrRec);
      FURLSTRStream.WriteDWord(NToLE(DWord(0))); // URL Offset for topic after the the "Local" value
      FURLSTRStream.WriteDWord(NToLE(DWord(0))); // Offset of FrameName??
      if Length(AString) > 0 then
        FURLSTRStream.Write(AString[1], Length(AString));
      FURLSTRStream.WriteByte(0); //NT
  end;

  function LookupUrlString(const AUrl : String):DWord;
  var n :TAvlTreeNode;
  begin
    SpareUrlStr.UrlStr:=AUrl;
    n:=FAvlUrlStr.FindKey(SpareUrlStr,@CompareUrlStrs);
    if assigned(n) Then
      result:=TUrlStrIndex(n.data).UrlStrId
    else
      result:=AddUrlString(AUrl);
  end;


var UrlIndex : Integer;

begin
  if (Length(AURL) > 0) and (AURL[1] = '/') then Delete(AURL,1,1);
  UrlIndex:=LookupUrlString(AUrl);

  //if $1000 - (FURLTBLStream.Size mod $1000) = 4 then // we are at 4092
  if FURLTBLStream.Size and $FFC = $FFC then // faster :)
    FURLTBLStream.WriteDWord(0);
  Result := FURLTBLStream.Position;
  FURLTBLStream.WriteDWord(0);//($231e9f5c); //unknown
  FURLTBLStream.WriteDWord(NtoLE(TopicsIndex)); // Index of topic in #TOPICS
  FURLTBLStream.WriteDWord(NtoLE(UrlIndex));
end;



procedure TChmWriter.CheckFileMakeSearchable(AStream: TStream; AFileEntry: TFileEntryRec);

  var
    TopicEntry: TTopicEntry;
    ATitle: String;
begin
  if Pos('.ht', AFileEntry.Name) > 0 then
  begin
    ATitle := FIndexedFiles.IndexFile(AStream, NextTopicIndex, FSearchTitlesOnly);
    if ATitle <> '' then
      TopicEntry.StringsOffset := AddString(ATitle)
    else
      TopicEntry.StringsOffset := $FFFFFFFF;
    TopicEntry.URLTableOffset := AddURL(AFileEntry.Path+AFileEntry.Name, NextTopicIndex);
    TopicEntry.InContents := 2;
    TopicEntry.Unknown := 0;
    TopicEntry.TocOffset := 0;
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.TocOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.StringsOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.URLTableOffset));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.InContents));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.Unknown));
  end;
end;

function TChmWriter.AddTopic(ATitle,AnUrl:AnsiString):integer;

var
    TopicEntry: TTopicEntry;

begin
    if ATitle <> '' then
      TopicEntry.StringsOffset := AddString(ATitle)
    else
      TopicEntry.StringsOffset := $FFFFFFFF;
    result:=NextTopicIndex;
    TopicEntry.URLTableOffset := AddURL(AnUrl, Result);
    TopicEntry.InContents := 2;
    TopicEntry.Unknown := 0;
    TopicEntry.TocOffset := 0;
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.TocOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.StringsOffset));
    FTopicsStream.WriteDWord(LEtoN(TopicEntry.URLTableOffset));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.InContents));
    FTopicsStream.WriteWord(LEtoN(TopicEntry.Unknown));
end;

function TChmWriter.NextTopicIndex: Integer;
begin
  Result := FTopicsStream.Size div 16;
end;

procedure TChmWriter.AppendTOC(AStream: TStream);

var tmpstr : string;
begin
  fHasTOC := True;
  if fTocName = '' then
    tmpstr := defaulthhc
  else
    tmpstr := fTocName;
  PostAddStreamToArchive(tmpstr, '/', AStream, True);
end;

procedure TChmWriter.AppendBinaryTOCFromSiteMap(ASiteMap: TChmSiteMap);
var
  Header: TTOCIdxHeader;
  Entry: TTocEntry;
  EntryInfo: TTOCEntryPageBookInfo;


  EntryInfoStream,
  EntryTopicOffsetStream,
  EntryStream: TMemoryStream;

  TOCIDXStream: TMemoryStream;

  NextLevelItems,
  CurrentLevelItems: TFPList;
  i,j: Integer;
  MenuItem: TChmSiteMapItem;
  MenuItems: TChmSiteMapItems;
  TopicEntry: TTopicEntry;
  EntryCount: DWord = $29A;
  procedure FixParentBookFirstChildOffset(AChildOffset: DWord);
  var
    ParentEntry: TTOCEntryPageBookInfo;
  begin
    // read parent entry
    EntryInfoStream.Position := MenuItems.InternalData;
    EntryInfoStream.Read(ParentEntry, SizeOf(ParentEntry));
    // update child offset
    ParentEntry.FirstChildOffset:= NtoLE(DWord(4096 + AChildOffset));
    // write back to stream
    EntryInfoStream.Position := MenuItems.InternalData;
    EntryInfoStream.Write(ParentEntry, SizeOf(ParentEntry));
    // move to end of stream
    EntryInfoStream.Position := AChildOffset;
  end;

begin
  FillChar(Header, 4096, 0);
  // create streams
  TOCIDXStream := TMemoryStream.Create;
  EntryInfoStream := TMemoryStream.Create;
  EntryTopicOffsetStream := TMemoryStream.Create;
  EntryStream := TMemoryStream.Create;

  NextLevelItems := TFPList.Create;

  NextLevelItems.Add(ASiteMap.Items);

  if NextLevelItems.Count = 0 then
      FreeAndNil(NextLevelItems);

  while NextLevelItems <> nil do
  begin
    CurrentLevelItems := NextLevelItems;
    NextLevelItems := TFPList.Create;

    for i := 0 to CurrentLevelItems.Count-1 do
    begin
      MenuItems := TChmSiteMapItems(CurrentLevelItems.Items[i]);

      for j := 0 to MenuItems.Count-1 do
      begin
        MenuItem := MenuItems.Item[j];
        // first figure out the props
        EntryInfo.Props := 0;
        if MenuItem.Children.Count > 0 then
          EntryInfo.Props := EntryInfo.Props or TOC_ENTRY_HAS_CHILDREN;
        if Length(MenuItem.Local) > 0 then
          EntryInfo.Props := EntryInfo.Props or TOC_ENTRY_HAS_LOCAL;


      if EntryInfo.Props and TOC_ENTRY_HAS_LOCAL > 0 then
      begin
        // Write #TOPICS entry
        TopicEntry.TocOffset      := NtoLE(DWord(4096 + EntryInfoStream.Position));
        TopicEntry.StringsOffset  := NtoLE(AddString(MenuItem.Text));
        TopicEntry.URLTableOffset := NtoLE(AddURL(MenuItem.Local, NextTopicIndex));
        TopicEntry.InContents     := NtoLE(Word( 2 ));
        TopicEntry.Unknown        := 0;
        EntryInfo.TopicsIndexOrStringsOffset := NtoLE(Dword(NextTopicIndex));;
        FTopicsStream.Write(TopicEntry, SizeOf(TopicEntry));
        EntryTopicOffsetStream.WriteDWord(EntryInfo.TopicsIndexOrStringsOffset);

        // write TOCEntry
        Entry.PageBookInfoOffset:= NtoLE(4096 + EntryInfoStream.Position);
        Entry.IncrementedInt  := NtoLE(EntryCount);
        EntryStream.Write(Entry, SizeOf(Entry));
        Inc(EntryCount);

      end
      else
      begin
        EntryInfo.TopicsIndexOrStringsOffset := NtoLE(AddString(MenuItem.Text));
      end;


        // write TOCEntryInfo

        EntryInfo.Unknown1 := 0;
        EntryInfo.EntryIndex := NtoLE(Word(EntryCount - $29A)); //who knows how useful any of this is

        if MenuItems.InternalData <> maxLongint then
          EntryInfo.ParentPageBookInfoOffset := MenuItems.InternalData
        else
          EntryInfo.ParentPageBookInfoOffset := 0;

        if j = MenuItems.Count-1 then
          EntryInfo.NextPageBookOffset := 0
        else if (EntryInfo.Props and TOC_ENTRY_HAS_CHILDREN) > 0 then
          EntryInfo.NextPageBookOffset := 4096 + EntryInfoStream.Position + 28
        else
          EntryInfo.NextPageBookOffset := 4096 + EntryInfoStream.Position + 20;

        // Only if TOC_ENTRY_HAS_CHILDREN is set are these written
        EntryInfo.FirstChildOffset := 0; // we will update this when the child is written
        // in fact lets update the *parent* of this item now if needed
        if (j = 0) and (MenuItems.InternalData <> maxLongint) then
          FixParentBookFirstChildOffset(EntryInfoStream.Position);

        EntryInfo.Unknown3 := 0;

        // fix endian order
        EntryInfo.Props := NtoLE(EntryInfo.Props);
        EntryInfo.ParentPageBookInfoOffset := NtoLE(EntryInfo.ParentPageBookInfoOffset);
        EntryInfo.NextPageBookOffset := NtoLE(EntryInfo.NextPageBookOffset);

        if MenuItem.Children.Count > 0 then
        begin
          NextLevelItems.Add(MenuItem.Children);
          MenuItem.Children.InternalData := EntryInfoStream.Position;
        end;

        // write to stream
        EntryInfoStream.Write(EntryInfo, PageBookInfoRecordSize(@EntryInfo));
      end;
    end;

    FreeAndNil(CurrentLevelItems);
    if NextLevelItems.Count = 0 then
      FreeAndNil(NextLevelItems);
  end;

  // write all streams to TOCIdxStream and free everything
  EntryInfoStream.Position:=0;
  EntryTopicOffsetStream.Position:=0;
  EntryStream.Position:=0;

  Header.BlockSize := NtoLE(DWord(4096));
  Header.EntriesCount := NtoLE(DWord(EntryCount - $29A));
  Header.EntriesOffset := NtoLE(DWord(4096 + EntryInfoStream.Size + EntryTopicOffsetStream.Size));
  Header.TopicsOffset := NtoLE(DWord(4096 + EntryInfoStream.Size));

  TOCIDXStream.Write(Header, SizeOf(Header));

  TOCIDXStream.CopyFrom(EntryInfoStream, EntryInfoStream.Size);
  EntryInfoStream.Free;

  TOCIDXStream.CopyFrom(EntryTopicOffsetStream, EntryTopicOffsetStream.Size);
  EntryTopicOffsetStream.Free;

  TOCIDXStream.CopyFrom(EntryStream, EntryStream.Size);
  EntryStream.Free;

  TOCIDXStream.Position := 0;
  AppendBinaryTOCStream(TOCIDXStream);
  TOCIDXStream.Free;
end;

Const
      BinIndexIdent : array[0..1] of char = (CHR($3B),CHR($29));
      AlwaysX44     : Array[0..15] of char = ('X','4','4',#0,#0,#0,#0,#0,
                                              #0,#0,#0,#0,#0,#0,#0,#0);
      DataEntry     : Array[0..12] of Byte = ($00,$00,$00,$00,$05,$00,$00,$00,$80,$00,$00,$00,$00);
{
  IndexStream:=TMemoryStream.Create;
  IndexStream.Write(BinIndexIdent,2);
  IndexStream.Write(NToLE(word(2)),2);
  IndexStream.Write(NToLE(word(2048)),2);
  IndexStream.Write(AlwaysX44,sizeof(AlwaysX44));
  IndexStrem.Write (dword(0),2);
}

Const DefBlockSize  = 2048;

Type TIndexBlock = Array[0..DefBlockSize-1] of Byte;

procedure writeword(var p:pbyte;w:word); inline;

begin
  pword(p)^:=NToLE(w);
  inc(pword(p));
end;

procedure writedword(var p:pbyte;d:dword); inline;

begin
  pdword(p)^:=NToLE(d);
  inc(pdword(p));
end;

procedure TChmWriter.AppendBinaryIndexFromSiteMap(ASiteMap: TChmSiteMap;chw:boolean);

Var
  IndexStream : TMemoryStream;
  //n           : Integer;
  curblock    : TIndexBlock;    // current listing block being built
  TestBlock   : TIndexBlock;    // each entry is first built here. then moved to curblock
  curind      : integer;        // next byte to write in testblock.
  blocknr     : Integer;        // blocknr of block in testblock;
  lastblock   : Integer;        // blocknr of last block.
  Entries     : Integer;        // Number of entries in this block so far
  TotalEntries: Integer;        // Total number of entries
  MapEntries  : Integer;
  MapIndex    : Integer;
  indexblocknr: Integer;
  blockind    : Integer;        // next byte to write in blockn[blocknr]
  blockentries: Integer;        // entries so far ins blockn[blocknr]
  blockn      : Array Of TIndexBlock;
  BlockNPlus1 : Array of TIndexBlock;
  Mod13value  : integer;        // A value that is increased by 13 for each entry. (?!?!)
  EntryToIndex: boolean;        // helper var to make sure the first block is always indexed.
  blocknplusindex   : Integer;  // blocks in level n+1 (second part)
  blocknplusentries : Integer;  // The other blocks indexed on creation.
  datastream,mapstream,propertystream : TMemoryStream;

procedure preparecurrentblock;

var p: PBTreeBlockHeader;

begin
  p:=@curblock[0];
  p^.Length:=NToLE(Defblocksize-curind);
  p^.NumberOfEntries:=Entries;
  p^.IndexOfPrevBlock:=lastblock;
  p^.IndexOfNextBlock:=Blocknr;
  IndexStream.Write(curblock[0],Defblocksize);
  MapStream.Write(NToLE(MapEntries),sizeof(dword));
  MapStream.Write(NToLE(BlockNr),Sizeof(DWord));
  MapEntries:=TotalEntries;
  curind:=sizeof(TBtreeBlockHeader);   // index into current block;
  lastblock:=blocknr;
  inc(blocknr);
end;

procedure prepareindexblockn(listingblocknr:integer);
var p:PBTreeIndexBlockHeader;
begin
  p:=@Blockn[IndexBlockNr];
  p^.Length:=defblocksize-BlockInd;
  p^.NumberOfEntries:=BlockEntries;

// p^.IndexOfChildBlock  // already entered on block creation, since of first entry, not last.
  inc(Indexblocknr);
  BlockEntries:=0;
  BlockInd:=0;
  if Indexblocknr>=length(blockn) then
    setlength(blockn,length(blockn)+1);  // larger increments also possible. #blocks is kept independantly.
  p:=@Blockn[IndexBlockNr];
  p^.IndexOfChildBlock:=ListingBlockNr;
  blockind:=sizeof(TBTreeIndexBlockHeader);
end;

procedure finalizeindexblockn(p:pbyte;var ind:integer;Entries:integer);
var ph:PBTreeIndexBlockHeader;
begin
  ph:=PBTreeIndexBlockHeader(p);
  ph^.Length:=defblocksize-Ind;
  ph^.NumberOfEntries:=Entries;
// p^.IndexOfChildBlock  // already entered on block creation, since of first entry, not last.
//  inc(Ind);
end;

procedure CurEntryToIndex(entrysize:integer);
var p,pentry : pbyte;
    indexentrysize : integer;
begin
  indexentrysize:=entrysize-sizeof(dword);         // index entry is 4 bytes shorter, and only the last dword differs
  if (blockind+indexentrysize)>=Defblocksize then
    prepareindexblockn(blocknr);
  p:=@blockn[Indexblocknr][blockind];
  move(testblock[0],p^,indexentrysize);
  pentry:=@p[indexentrysize-sizeof(dword)];         // ptr to last dword
  writedword(pentry,blocknr);                      // patch up the "index of child field"
  inc(blockind,indexentrysize);
end;

procedure CreateEntry(Item:TChmSiteMapItem;Str:WideString;commaatposition:integer);

var p      : pbyte;
    topicid: integer;
    seealso: Integer;
    entrysize:Integer;
    i      : Integer;
begin
  inc(TotalEntries);
  p:=@TestBlock[0];
  for i:=1 to Length(str) do
    WriteWord(p,Word(str[i]));   // write the wstr in little endian
  WriteWord(p,0);                // NT
//  if item.seealso='' then    // no seealso for now
    seealso:=0;
 // else
//    seealso:=2;
  WriteWord(p,seealso);          // =0 not a see also 2 =seealso
  WriteWord(p,2);                // Entrydepth.  We can't know it, so write 2.
  WriteDword(p,commaatposition); // position of the comma
  WriteDword(p,0);               // unused 0
  WriteDword(p,1);               // for now only local pair.
  TopicId:=AddTopic(Item.Text,item.Local);
  WriteDword(p,TopicId);
  // if seealso then _here_ a wchar NT string with seealso?
  WriteDword(p,1);               // always 1 (unknown);
  WriteDword(p,mod13value);      //a value that increments with 13.
  mod13value:=mod13value+13;
  entrysize:=p-pbyte(@testblock[0]);
  if (curind+entrysize)>=Defblocksize then
    begin
      preparecurrentblock;
      EntrytoIndex:=true;
    end;
  if EntryToIndex Then
    begin
      CurEntryToIndex(entrysize);
      EntryToIndex:=False;
    end;
  move(testblock[0],curblock[curind],entrysize);
  inc(curind,entrysize);
  datastream.write(DataEntry,Sizeof(DataEntry));
end;

procedure MoveIndexEntry(nr:integer;bytes:integer;childblock:integer);
var
  pscr,pdest : pbyte;
begin
  {$ifdef binindex}
    writeln(' moveindexentry ',nr,' bytes:',bytes,' childblock:',childblock);
    flush(stdout);
  {$endif}

  if ((blockind+bytes)>=defblocksize) then
    begin
      {$ifdef binindex}
      writeln(' in scalecheck  ',blockind);
      flush(stdout);
      {$endif}

      FinalizeIndexBlockn(@blocknplus1[blocknplusindex][0],blockind,blocknplusentries);
      inc(blocknplusindex);
      if blocknplusindex>=length(blocknplus1) then
        setlength(blocknplus1,length(blocknplus1)+1);
      blockInd:=Sizeof(TBTreeIndexBlockHeader);
      pdword(@blocknplus1[blocknplusindex][0])[4]:=NToLE(ChildBlock);  /// init 2nd level index to first 1st level index block
      end;
  {$ifdef binindex}
    writeln(' len:',length(blocknplus1),' blockind:',blockind,' index:',blocknplusindex);
    flush(stdout);
  {$endif}

  // copy entry from one indexblock to another
  pscr:=@blockn[nr][sizeof(TBtreeIndexBlockHeader)];
  pdest:=@blocknplus1[blocknplusindex][blockind];
  move(pscr^,pdest^,bytes);
  pdword(@pdest[bytes-sizeof(dword)])^:=NToLE(childblock);    // correcting the childindex
  inc (blockind,bytes);
  inc(blocknplusentries); // not needed for writing, but used to check if something has been written. End condition
end;

function ScanIndexBlock(blk:Pbyte):Integer;

var start : pbyte;
    n     : Integer;
    i     : Integer;
begin
  start:=@blk[sizeof(TBtreeIndexBlockHeader)];
  blk:=start;
  while pword(blk)^<>0 do   // skip wchar
    inc(pword(blk));
  inc(pword(blk));          // skip NT
  inc(pword(blk));          // skip see also
  inc(pword(blk));          // skip depth
  inc(pdword(blk));         // skip Character Index.
  inc(pdword(blk));          // skip always  0
  n:=LEToN(pdword(blk)^);
  inc(pdword(blk));          // skip nr of pairs.
  for i:= 1 to n do
      inc(pdword(blk));          // skip <n> topicids
  inc(pdword(blk));          // skip childindex
  Result:=blk-start;
end;

procedure CombineWithChildren(ParentItem:TChmSiteMapItem;Str:WideString;commaatposition:integer;first:boolean);
var i    : Integer;
    Item : TChmSiteMapItem;
begin
  if ParentItem.Children.Count = 0 Then
    Begin
     // comment/fix next
     //   if commatposition=length(str) then commaatposition:=0;
       if first then
        CreateEntry(ParentItem,Str,0)
       else
        CreateEntry(ParentItem,Str,commaatposition);
    End
  Else
    for i:=0 to ParentItem.Children.Count-1 do
      begin
        item := TChmSiteMapItem(ParentItem.Children.Item[i]);
        if first Then
          CombineWithChildren(Item,Str+', '+item.text,commaatposition+2,false)
        else
          CombineWithChildren(Item,Str+', '+item.text,commaatposition,false);
      end;
end;

Var i             : Integer;
    Key           : WideString;
    Item          : TChmSiteMapItem;
    ListingBlocks : Integer;
    EntryBytes    : Integer;
    Hdr           : TBTreeHeader;
    TreeDepth     : Integer;

{$ifdef binindex}
procedure printloopvars(i:integer);

begin
  Writeln('location :' ,i, ' blocknr :', blocknr,' level:',TreeDepth);
  Writeln('blockn      length: ',length(blockn),' indexblocknr: ',indexblocknr,' blockind ',blockind);
  Writeln('blocknplus1 length: ',length(blocknplus1),' blocknplusindex:',blocknplusindex,' entries:',blocknplusentries);
  flush(stdout);
end;
{$endif}
begin
  IndexStream:=TMemoryStream.Create;
  indexstream.size:=sizeof(TBTreeHeader);
  IndexStream.position:=Sizeof(TBTreeHeader);
  datastream:=TMemoryStream.Create;
  mapstream :=TMemoryStream.Create;
  mapstream.size:=2;
  mapstream.position:=2;
  propertystream :=TMemoryStream.Create;
  propertystream.write(NToLE(0),sizeof(4));
  // we iterate over all entries and write listingblocks directly to the stream.
  // and the first (and maybe last) level is written to blockn.
  // we can't do higher levels yet because we don't know how many listblocks we get
  BlockNr     :=0;   // current block number
  Lastblock   :=-1;  // previous block nr or -1 if none.
  Entries     :=0;   // entries in this block
  TotalEntries:=0;   // entries so far.
  Mod13value  :=0;   // value that increments by 13 entirely.
  indexblocknr:=0;   // nr of first index block.
  BlockEntries:=0;   // entries into current block;
  MapEntries  :=0;   // entries before the current listing block, for MAP file

  curind      :=sizeof(TBTreeBlockHeader);      // index into current listing block;
  blockind    :=sizeof(TBtreeIndexBlockHeader); // index into current index block

  Setlength(blockn,1);
  pdword(@blockn[0][4])^:=NToLE(0);  /// init first listingblock nr to 0 in the first index block
  EntryToIndex   := True;
  for i:=0 to ASiteMap.Items.Count-1 do
    begin
      item := TChmSiteMapItem(ASiteMap.Items.Item[i]);
      key  :=Item.Text;
      {$ifdef chm_windowsbinindex}
      // append 2 to all index level 0 entries. This
      // so we can see if Windows loads the binary or textual index.
      CombineWithChildren(Item,Key+'2',length(key)+1,true);
      {$else}
      CombineWithChildren(Item,Key,length(key),true);
      {$endif}
    end;
  PrepareCurrentBlock;     // flush last listing block.
  Listingblocks:=blocknr;   // blocknr is from now on the number of the first block in blockn.
                            // we still need the # of listingblocks for the header though

  {$ifdef binindex}
    writeln('binindex: listingblocks : '+inttostr(listingblocks),' indexblocks: ',indexblocknr,' entries:',blockentries);
  {$endif}

  // we have now created and written the listing blocks, and created the first level of index in <blockn>
  // the following loop uses <blockn> to calculate the next level (in blocknplus1), then write out blockn,
  // and repeat until we have no entries left.

  // First we finalize the current set of blocks

  if  Blockind<>sizeof(TBtreeIndexBlockHeader) Then
    begin
      {$ifdef binindex}
        writeln('finalizing level 1 index');
      {$endif}
      FinalizeIndexBlockN(@blockn[indexblocknr][0],blockind,blockentries); // also increasing indexblocknr
      inc(IndexBlockNr);
    end;
  {$ifdef binindex}
    writeln('binindex: listingblocks : '+inttostr(listingblocks),' indexblocks: ',indexblocknr,' entries:',blockentries);
  {$endif}


  while (Indexblocknr>1) do
    begin
      {$ifdef binindex}
        printloopvars(1);
      {$endif}

      blockind      :=sizeof(TBtreeIndexBlockHeader);
      pdword(@blockn[0][4])^:=NToLE(Listingblocks);  /// init 2nd level index to first 1st level index block
      blocknplusindex     :=0;
      blocknplusentries   :=0;
      if length(blocknplus1)<1 then
        Setlength(blocknplus1,1);

      EntryToIndex        :=True;
      {$ifdef binindex}
        printloopvars(2);
      {$endif}
      for i:=0 to Indexblocknr-1 do
        begin
          Entrybytes:=ScanIndexBlock(@blockn[i][0]);
//          writeln('after scan ,',i, ' bytes: ',entrybytes,' blocknr:',blocknr,' indexblocknr:',indexblocknr,' to:',blocknr+i);
          MoveIndexEntry(i,Entrybytes,blocknr+i);
          indexStream.Write(blockn[i][0],defblocksize);
        end;

      {$ifdef binindex}
        printloopvars(3);
      {$endif}

      If Blockind<>sizeof(TBtreeIndexBlockHeader) Then
        begin
          {$ifdef binindex}
            logentry('finalizing');
          {$endif}
          FinalizeIndexBlockn(@blocknplus1[blocknplusindex][0],blockind,blocknplusentries);
          inc(blocknplusindex);
        end;

      inc(blocknr,indexblocknr);

      indexblocknr:=blocknplusindex;
      blockn:=copy(blocknplus1); setlength(blocknplus1,1);
      {$ifdef binindex}
        printloopvars(5);
      {$endif}

      inc(TreeDepth);
    end;
  indexStream.Write(blockn[0][0],defblocksize);
  inc(blocknr);
  // Fixup header.
  hdr.ident[0]:=chr($3B); hdr.ident[1]:=chr($29);
  hdr.flags          :=NToLE(word($2));           // bit $2 is always 1, bit $0400 1 if dir? (always on)
  hdr.blocksize      :=NToLE(word(defblocksize)); // size of blocks (2048)
  hdr.dataformat     :=AlwaysX44;           // "X44" always the same, see specs.
  hdr.unknown0       :=NToLE(0);            // always 0
  hdr.lastlstblock   :=NToLE(dword(ListingBlocks-1)); // index of last listing block in the file;
  hdr.indexrootblock :=NToLE(dword(blocknr-1));    // Index of the root block in the file.
  hdr.unknown1       :=NToLE(dword(-1));           // always -1
  hdr.nrblock        :=NToLE(blocknr);      // Number of blocks
  hdr.treedepth      :=NToLE(TreeDepth);    // The depth of the tree of blocks (1 if no index blocks, 2 one level of index blocks, ...)
  hdr.nrkeywords     :=NToLE(Totalentries); // number of keywords in the file.
  hdr.codepage       :=NToLE(dword(1252));         // Windows code page identifier (usually 1252 - Windows 3.1 US (ANSI))
  hdr.lcid           :=NToLE(0);            //  ???? LCID from the HHP file.
  if not chw then
    hdr.ischm        :=NToLE(dword(1))             // 0 if this a BTREE and is part of a CHW file, 1 if it is a BTree and is part of a CHI or CHM file
  else
    hdr.ischm        :=NToLE(0);
  hdr.unknown2       :=NToLE(dword(10031));        // Unknown. Almost always 10031. Also 66631 (accessib.chm, ieeula.chm, iesupp.chm, iexplore.chm, msoe.chm, mstask.chm, ratings.chm, wab.chm).
  hdr.unknown3       :=NToLE(0);            // unknown 0
  hdr.unknown4       :=NToLE(0);            // unknown 0
  hdr.unknown5       :=NToLE(0);            // unknown 0

  IndexStream.Position:=0;
  IndexStream.write(hdr,sizeof(hdr));
  {$ifdef binindex}
    logentry('before append');
  {$endif}

  AppendBinaryIndexStream(IndexStream,datastream,MapStream,PropertyStream,chw);
  IndexStream.Free;
  PropertyStream.Free;
  MapStream.Free;
  DataStream.Free;
end;

procedure TChmWriter.AppendBinaryTOCStream(AStream: TStream);
begin
  AddStreamToArchive('#TOCIDX', '/', AStream, True);
end;

procedure TChmWriter.AppendBinaryIndexStream(IndexStream,DataStream,MapStream,Propertystream: TStream;chw:boolean);

procedure stadd(fn:string;stream:TStream);

begin
  Stream.Position:=0;
  if CHW then
    fn:=uppercase(fn);
  {$ifdef binindex}
    logentry('before append '+fn);
  {$endif}
  AddStreamToArchive(fn,'/$WWKeywordLinks/',stream,True);
end;

begin
  stadd('BTree',IndexStream);
  stadd('Data', DataStream);
  stadd('Map' , MapStream);
  stadd('Property', PropertyStream);
end;

procedure TChmWriter.AppendIndex(AStream: TStream);
var tmpstr : string;
begin
  FHasIndex := True;
  if fIndexName = '' then
    tmpstr:=defaulthhk
  else
    tmpstr:=fIndexName;
  PostAddStreamToArchive(tmpstr, '/', AStream, True);
end;

procedure TChmWriter.AppendSearchDB(AName: String; AStream: TStream);
begin
  PostAddStreamToArchive(AName, '/', AStream);
end;


procedure TChmWriter.AddContext(AContext: DWord; ATopic: String);
var
  Offset: DWord;
begin
  if FContextStream = nil then begin
    FContextStream:=TMemoryStream.Create;
    // #IVB starts with a dword which is the size of the stream - sizeof(dword)
    FContextStream.WriteDWord(0);
    // we will update this when we write the file to the final stream
  end;
  // an entry is a context id and then the offset of the name of the topic in the strings file
  FContextStream.WriteDWord(NToLE(AContext));
  Offset := NToLE(AddString(ATopic));
  FContextStream.WriteDWord(Offset);
end;

procedure TChmWriter.SetWindows(AWindowList:TObjectList);

var i : integer;
    x : TCHMWindow;
begin
  FWindows.Clear;
  for i:=0 to AWindowList.count -1 do
    begin
      x:=TChmWindow.Create;
      x.assign(TChmWindow(AWindowList[i]));
      Fwindows.Add(x);
    end;
end;


=======
=======
>>>>>>> origin/fixes_2_2
  if FullTextSearch then
    CheckFileMakeSearchable(AStream, Entry);
end;

procedure TChmWriter.AddContext(AContext: DWord; ATopic: String);
var
  Offset: DWord;
begin
  if FContextStream = nil then begin
    // #IVB starts with a dword which is the size of the stream - sizeof(dword)
    FContextStream.WriteDWord(0);
    // we will update this when we write the file to the final stream
  end;
  // an entry is a context id and then the offset of the name of the topic in the strings file
  FContextStream.WriteDWord(NToLE(AContext));
  Offset := NToLE(AddString(ATopic));
  FContextStream.WriteDWord(Offset);
end;

procedure TChmWriter.StartCompressingStream;
var
  {$IFNDEF LZX_USETHREADS}
  LZXdata: Plzx_data;
  WSize: LongInt;
  {$ELSE}
  Compressor: TLZXCompressor;
  {$ENDIF}
begin
 {$IFNDEF LZX_USETHREADS}
  lzx_init(@LZXdata, LZX_WINDOW_SIZE, @_GetData, Self, @_AtEndOfData,
              @_WriteCompressedData, Self, @_MarkFrame, Self);

  WSize := 1 shl LZX_WINDOW_SIZE;
  while not AtEndOfData do begin
    lzx_reset(LZXdata);
    lzx_compress_block(LZXdata, WSize, True);
  end;

  //we have to mark the last frame manually
  MarkFrame(LZXdata^.len_uncompressed_input, LZXdata^.len_compressed_output);

  lzx_finish(LZXdata, nil);
  {$ELSE}
  Compressor := TLZXCompressor.Create(10);
  Compressor.OnChunkDone  :=@LTChunkDone;
  Compressor.OnGetData    :=@LTGetData;
  Compressor.OnIsEndOfFile:=@LTIsEndOfFile;
  Compressor.OnMarkFrame  :=@LTMarkFrame;
  Compressor.Execute(True);
  //Sleep(20000);
  Compressor.Free;
  {$ENDIF}
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.

