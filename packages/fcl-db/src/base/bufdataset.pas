{
    This file is part of the Free Pascal run time library.
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 1999-2014 by Joost van der Sluis and other members of the
=======
    Copyright (c) 1999-2006 by Joost van der Sluis, member of the
>>>>>>> graemeg/fixes_2_2
=======
    Copyright (c) 1999-2006 by Joost van der Sluis, member of the
>>>>>>> origin/fixes_2_2
    Free Pascal development team

    BufDataset implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit BufDataset;

{$mode objfpc}
{$h+}

interface

uses Classes,Sysutils,db,bufdataset_parser;

type
<<<<<<< HEAD
<<<<<<< HEAD
  TCustomBufDataset = Class;

  TResolverErrorEvent = procedure(Sender: TObject; DataSet: TCustomBufDataset; E: EUpdateError;
    UpdateKind: TUpdateKind; var Response: TResolverResponse) of object;

  { TBlobBuffer }
=======
=======
>>>>>>> origin/fixes_2_2
  TBufDataset = Class;

  TResolverErrorEvent = procedure(Sender: TObject; DataSet: TBufDataset; E: EUpdateError;
    UpdateKind: TUpdateKind; var Response: TResolverResponse) of object;

  { TBufBlobStream }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  PBlobBuffer = ^TBlobBuffer;
  TBlobBuffer = record
    FieldNo : integer;
    OrgBufID: integer;
    Buffer  : pointer;
<<<<<<< HEAD
<<<<<<< HEAD
    Size    : PtrInt;
  end;

  PBufBlobField = ^TBufBlobField;
  TBufBlobField = record
    ConnBlobBuffer : array[0..11] of byte; // DB specific data is stored here
    BlobBuffer     : PBlobBuffer;
  end;

  { TBufBlobStream }

  TBufBlobStream = class(TStream)
  private
    FField      : TBlobField;
    FDataSet    : TCustomBufDataset;
    FBlobBuffer : PBlobBuffer;
<<<<<<< HEAD
    FPosition   : PtrInt;
    FModified   : boolean;
=======
    FPosition   : ptrint;
    FDataset    : TCustomBufDataset;
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
  protected
    function Seek(Offset: Longint; Origin: Word): Longint; override;
    function Read(var Buffer; Count: Longint): Longint; override;
    function Write(const Buffer; Count: Longint): Longint; override;
  public
    constructor Create(Field: TBlobField; Mode: TBlobStreamMode);
    destructor Destroy; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { TCustomBufDataset }
=======
=======
>>>>>>> origin/fixes_2_2
    Size    : ptrint;
  end;

   TBufBlobStream = class(TStream)
  private
    FBlobBuffer : PBlobBuffer;
    FPosition   : ptrint;
    FDataset    : TBufDataset;
  protected
    function Read(var Buffer; Count: Longint): Longint; override;
    function Write(const Buffer; Count: Longint): Longint; override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
  public
    constructor Create(Field: TBlobField; Mode: TBlobStreamMode);
  end;

  { TBufDataset }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  PBufRecLinkItem = ^TBufRecLinkItem;
  TBufRecLinkItem = record
    prior   : PBufRecLinkItem;
    next    : PBufRecLinkItem;
  end;

  PBufBookmark = ^TBufBookmark;
  TBufBookmark = record
    BookmarkData : PBufRecLinkItem;
<<<<<<< HEAD
<<<<<<< HEAD
    BookmarkInt  : integer; // was used by TArrayBufIndex
=======
    BookmarkInt  : integer;
>>>>>>> graemeg/fixes_2_2
=======
    BookmarkInt  : integer;
>>>>>>> origin/fixes_2_2
    BookmarkFlag : TBookmarkFlag;
  end;

  TRecUpdateBuffer = record
    UpdateKind         : TUpdateKind;
{  BookMarkData:
<<<<<<< HEAD
<<<<<<< HEAD
     - Is -1 if the update has canceled out. For example: an appended record has been deleted again
     - If UpdateKind is ukInsert, it contains a bookmark to the newly created record
     - If UpdateKind is ukModify, it contains a bookmark to the record with the new data
     - If UpdateKind is ukDelete, it contains a bookmark to the deleted record (ie: the record is still there)
}
    BookmarkData       : TBufBookmark;
{  NextBookMarkData:
     - If UpdateKind is ukDelete, it contains a bookmark to the record just after the deleted record
}
    NextBookmarkData   : TBufBookmark;
{  OldValuesBuffer:
     - If UpdateKind is ukModify, it contains a record buffer which contains the old data
     - If UpdateKind is ukDelete, it contains a record buffer with the data of the deleted record
}
    OldValuesBuffer    : TRecordBuffer;
  end;
  TRecordsUpdateBuffer = array of TRecUpdateBuffer;

  TCompareFunc = function(subValue, aValue: pointer; size: integer; options: TLocateOptions): int64;

  TDBCompareRec = record
<<<<<<< HEAD
                   CompareFunc : TCompareFunc;
                   Off         : PtrInt;
                   NullBOff    : PtrInt;
                   FieldInd    : longint;
                   Size        : integer;
=======
                   Comparefunc : TCompareFunc;
                   Off1,Off2   : PtrInt;
                   FieldInd1,
                   FieldInd2   : longint;
                   NullBOff1,
                   NullBOff2   : PtrInt;
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
=======
=======
>>>>>>> origin/fixes_2_2
     - Is -1 if the update has canceled out. For example: a appended record has been deleted again
     - If UpdateKind is ukInsert it contains a bookmark to the new created record
     - If UpdateKind is ukModify it contains a bookmark to the record with the new data
     - If UpdateKind is ukDelete it contains a bookmark to the deleted record (ie: the record is still there)
}
    BookmarkData       : TBufBookmark;
{  NextBookMarkData:
     - If UpdateKind is ukDelete it contains a bookmark to the record just after the deleted record
}
    NextBookmarkData   : TBufBookmark;
{  OldValuesBuffer:
     - If UpdateKind is ukModify it contains a record-buffer which contains the old data
     - If UpdateKind is ukDelete it contains a record-buffer with the data of the deleted record
}
    OldValuesBuffer    : pchar;
  end;

  PBufBlobField = ^TBufBlobField;
  TBufBlobField = record
    ConnBlobBuffer : array[0..11] of byte; // It's here where the db-specific data is stored
    BlobBuffer     : PBlobBuffer;
  end;

  TCompareFunc = function(subValue, aValue: pointer; options: TLocateOptions): int64;
  TRecordsUpdateBuffer = array of TRecUpdateBuffer;

  TDBCompareRec = record
                   Comparefunc : TCompareFunc;
                   Off1,Off2   : PtrInt;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                   Options     : TLocateOptions;
                   Desc        : Boolean;
                  end;
  TDBCompareStruct = array of TDBCompareRec;

  { TBufIndex }

  TBufIndex = class(TObject)
  private
<<<<<<< HEAD
<<<<<<< HEAD
    FDataset : TCustomBufDataset;
  protected
    function GetBookmarkSize: integer; virtual; abstract;
    function GetCurrentBuffer: Pointer; virtual; abstract;
    function GetCurrentRecord: TRecordBuffer; virtual; abstract;
    function GetIsInitialized: boolean; virtual; abstract;
    function GetSpareBuffer: TRecordBuffer; virtual; abstract;
    function GetSpareRecord: TRecordBuffer; virtual; abstract;
    function GetRecNo: Longint; virtual; abstract;
    procedure SetRecNo(ARecNo: Longint); virtual; abstract;
  public
    DBCompareStruct : TDBCompareStruct;
    Name            : String;
=======
=======
>>>>>>> origin/fixes_2_2
    FDataset : TBufDataset;
  protected
    function GetBookmarkSize: integer; virtual; abstract;
    function GetCurrentBuffer: Pointer; virtual; abstract;
    function GetCurrentRecord: PChar; virtual; abstract;
    function GetIsInitialized: boolean; virtual; abstract;
    function GetSpareBuffer: PChar; virtual; abstract;
    function GetSpareRecord: PChar; virtual; abstract;
  public
    DBCompareStruct : TDBCompareStruct;
    Name            : String;
    Fields          : TField;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FieldsName      : String;
    CaseinsFields   : String;
    DescFields      : String;
    Options         : TIndexOptions;
    IndNr           : integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    constructor Create(const ADataset : TCustomBufDataset); virtual;
=======
    constructor Create(const ADataset : TBufDataset); virtual;
>>>>>>> graemeg/fixes_2_2
=======
    constructor Create(const ADataset : TBufDataset); virtual;
>>>>>>> origin/fixes_2_2
    function ScrollBackward : TGetResult; virtual; abstract;
    function ScrollForward : TGetResult;  virtual; abstract;
    function GetCurrent : TGetResult;  virtual; abstract;
    function ScrollFirst : TGetResult;  virtual; abstract;
    procedure ScrollLast; virtual; abstract;
<<<<<<< HEAD
<<<<<<< HEAD
    // Gets prior/next record relative to given bookmark; does not change current record
    function GetRecord(ABookmark: PBufBookmark; GetMode: TGetMode): TGetResult; virtual;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    procedure SetToFirstRecord; virtual; abstract;
    procedure SetToLastRecord; virtual; abstract;

    procedure StoreCurrentRecord;  virtual; abstract;
    procedure RestoreCurrentRecord;  virtual; abstract;

    function CanScrollForward : Boolean;  virtual; abstract;
    procedure DoScrollForward;  virtual; abstract;

    procedure StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark);  virtual; abstract;
    procedure StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark);  virtual; abstract;
    procedure GotoBookmark(const ABookmark : PBufBookmark); virtual; abstract;
    function BookmarkValid(const ABookmark: PBufBookmark): boolean; virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    function CompareBookmarks(const ABookmark1, ABookmark2 : PBufBookmark) : integer; virtual;
    function SameBookmarks(const ABookmark1, ABookmark2 : PBufBookmark) : boolean; inline;

    procedure InitialiseIndex; virtual; abstract;

    procedure InitialiseSpareRecord(const ASpareRecord : TRecordBuffer); virtual; abstract;
=======
=======
>>>>>>> origin/fixes_2_2

    procedure InitialiseIndex; virtual; abstract;

    procedure InitialiseSpareRecord(const ASpareRecord : PChar); virtual; abstract;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure ReleaseSpareRecord; virtual; abstract;

    procedure BeginUpdate; virtual; abstract;
    // Adds a record to the end of the index as the new last record (spare record)
    // Normally only used in GetNextPacket
<<<<<<< HEAD
<<<<<<< HEAD
    procedure AddRecord; virtual; abstract;
    // Inserts a record before the current record, or if the record is sorted,
    // inserts it in the proper position
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : TRecordBuffer); virtual; abstract;
    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); virtual; abstract;
    procedure OrderCurrentRecord; virtual; abstract;
    procedure EndUpdate; virtual; abstract;

    property SpareRecord : TRecordBuffer read GetSpareRecord;
    property SpareBuffer : TRecordBuffer read GetSpareBuffer;
    property CurrentRecord : TRecordBuffer read GetCurrentRecord;
    property CurrentBuffer : Pointer read GetCurrentBuffer;
    property IsInitialized : boolean read GetIsInitialized;
    property BookmarkSize : integer read GetBookmarkSize;
    property RecNo : Longint read GetRecNo write SetRecNo;
  end;
  
=======
=======
>>>>>>> origin/fixes_2_2
    procedure AddRecord(Const ARecord : PChar); virtual; abstract;
    // Inserts a record before the current record, or if the record is sorted,
    // insert it to the proper position
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : PChar); virtual; abstract;
    procedure EndUpdate; virtual; abstract;
    
    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); virtual; abstract;
    
    function CompareBookmarks(const ABookmark1, ABookmark2 : PBufBookmark) : boolean; virtual;
    Function GetRecNo(const ABookmark : PBufBookmark) : integer; virtual; abstract;


    property SpareRecord : PChar read GetSpareRecord;
    property SpareBuffer : PChar read GetSpareBuffer;
    property CurrentRecord : PChar read GetCurrentRecord;
    property CurrentBuffer : Pointer read GetCurrentBuffer;
    property IsInitialized : boolean read GetIsInitialized;
    property BookmarkSize : integer read GetBookmarkSize;
  end;
  
  TDataPacketFormat = (dfBinary,dfXML,dfXMLUTF8,dfAny);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { TDoubleLinkedBufIndex }

  TDoubleLinkedBufIndex = class(TBufIndex)
  private
    FCursOnFirstRec : boolean;

    FStoredRecBuf  : PBufRecLinkItem;
    FCurrentRecBuf  : PBufRecLinkItem;
  protected
    function GetBookmarkSize: integer; override;
    function GetCurrentBuffer: Pointer; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetCurrentRecord: TRecordBuffer; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: TRecordBuffer; override;
    function GetSpareRecord: TRecordBuffer; override;
    function GetRecNo: Longint; override;
    procedure SetRecNo(ARecNo: Longint); override;
=======
    function GetCurrentRecord: PChar; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: PChar; override;
    function GetSpareRecord: PChar; override;
>>>>>>> origin/fixes_2_2
  public
    FLastRecBuf     : PBufRecLinkItem;
    FFirstRecBuf    : PBufRecLinkItem;
    FNeedScroll     : Boolean;
<<<<<<< HEAD

=======
>>>>>>> origin/fixes_2_2
    function ScrollBackward : TGetResult; override;
    function ScrollForward : TGetResult; override;
    function GetCurrent : TGetResult; override;
    function ScrollFirst : TGetResult; override;
    procedure ScrollLast; override;
<<<<<<< HEAD
    function GetRecord(ABookmark: PBufBookmark; GetMode: TGetMode): TGetResult; override;
=======
>>>>>>> origin/fixes_2_2

    procedure SetToFirstRecord; override;
    procedure SetToLastRecord; override;

    procedure StoreCurrentRecord; override;
    procedure RestoreCurrentRecord; override;

    function CanScrollForward : Boolean; override;
    procedure DoScrollForward; override;

    procedure StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure GotoBookmark(const ABookmark : PBufBookmark); override;
<<<<<<< HEAD
    function CompareBookmarks(const ABookmark1, ABookmark2: PBufBookmark): integer; override;
    procedure InitialiseIndex; override;

    procedure InitialiseSpareRecord(const ASpareRecord : TRecordBuffer); override;
    procedure ReleaseSpareRecord; override;

    procedure BeginUpdate; override;
    procedure AddRecord; override;
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : TRecordBuffer); override;
    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    procedure OrderCurrentRecord; override;
    procedure EndUpdate; override;
  end;

  { TUniDirectionalBufIndex }

  TUniDirectionalBufIndex = class(TBufIndex)
  private
    FSPareBuffer:  TRecordBuffer;
  protected
    function GetBookmarkSize: integer; override;
    function GetCurrentBuffer: Pointer; override;
    function GetCurrentRecord: TRecordBuffer; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: TRecordBuffer; override;
    function GetSpareRecord: TRecordBuffer; override;
    function GetRecNo: Longint; override;
    procedure SetRecNo(ARecNo: Longint); override;
  public
    function ScrollBackward : TGetResult; override;
    function ScrollForward : TGetResult; override;
    function GetCurrent : TGetResult; override;
    function ScrollFirst : TGetResult; override;
    procedure ScrollLast; override;

    procedure SetToFirstRecord; override;
    procedure SetToLastRecord; override;

    procedure StoreCurrentRecord; override;
    procedure RestoreCurrentRecord; override;

    function CanScrollForward : Boolean; override;
    procedure DoScrollForward; override;

    procedure StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure GotoBookmark(const ABookmark : PBufBookmark); override;

    procedure InitialiseIndex; override;
    procedure InitialiseSpareRecord(const ASpareRecord : TRecordBuffer); override;
    procedure ReleaseSpareRecord; override;

    procedure BeginUpdate; override;
    procedure AddRecord; override;
<<<<<<< HEAD
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : TRecordBuffer); override;
    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    procedure OrderCurrentRecord; override;
    procedure EndUpdate; override;
  end;

=======
=======

    procedure InitialiseIndex; override;

    procedure InitialiseSpareRecord(const ASpareRecord : PChar); override;
    procedure ReleaseSpareRecord; override;

    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    Function GetRecNo(const ABookmark : PBufBookmark) : integer; override;

    procedure BeginUpdate; override;
    procedure AddRecord(Const ARecord : PChar); override;
>>>>>>> origin/fixes_2_2
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : PChar); override;
    procedure EndUpdate; override;
  end;

<<<<<<< HEAD
  { TUniDirectionalBufIndex }

  TUniDirectionalBufIndex = class(TBufIndex)
  private
    FSPareBuffer: PChar;
=======
  { TArrayBufIndex }

  TArrayBufIndex = class(TBufIndex)
  private
    FStoredRecBuf  : integer;

    FInitialBuffers,
    FGrowBuffer     : integer;
    Function GetRecordFromBookmark(ABookmark: TBufBookmark) : integer;
>>>>>>> origin/fixes_2_2
  protected
    function GetBookmarkSize: integer; override;
    function GetCurrentBuffer: Pointer; override;
    function GetCurrentRecord: PChar; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: PChar; override;
    function GetSpareRecord: PChar; override;
  public
<<<<<<< HEAD
=======
    FCurrentRecInd  : integer;
    FRecordArray    : array of Pointer;
    FLastRecInd     : integer;
    FNeedScroll     : Boolean;
    constructor Create(const ADataset: TBufDataset); override;
>>>>>>> origin/fixes_2_2
    function ScrollBackward : TGetResult; override;
    function ScrollForward : TGetResult; override;
    function GetCurrent : TGetResult; override;
    function ScrollFirst : TGetResult; override;
    procedure ScrollLast; override;

    procedure SetToFirstRecord; override;
    procedure SetToLastRecord; override;

    procedure StoreCurrentRecord; override;
    procedure RestoreCurrentRecord; override;

    function CanScrollForward : Boolean; override;
    procedure DoScrollForward; override;

    procedure StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure GotoBookmark(const ABookmark : PBufBookmark); override;

    procedure InitialiseIndex; override;
<<<<<<< HEAD
    procedure InitialiseSpareRecord(const ASpareRecord : PChar); override;
    procedure ReleaseSpareRecord; override;

    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    Function GetRecNo(const ABookmark : PBufBookmark) : integer; override;

    procedure BeginUpdate; override;
    procedure AddRecord; override;
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : PChar); override;
    procedure EndUpdate; override;
  end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  { TUniDirectionalBufIndex }

  TUniDirectionalBufIndex = class(TBufIndex)
  private
    FSPareBuffer: PChar;
  protected
    function GetBookmarkSize: integer; override;
    function GetCurrentBuffer: Pointer; override;
    function GetCurrentRecord: PChar; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: PChar; override;
    function GetSpareRecord: PChar; override;
  public
    function ScrollBackward : TGetResult; override;
    function ScrollForward : TGetResult; override;
    function GetCurrent : TGetResult; override;
    function ScrollFirst : TGetResult; override;
    procedure ScrollLast; override;

    procedure SetToFirstRecord; override;
    procedure SetToLastRecord; override;

    procedure StoreCurrentRecord; override;
    procedure RestoreCurrentRecord; override;

    function CanScrollForward : Boolean; override;
    procedure DoScrollForward; override;

    procedure StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure GotoBookmark(const ABookmark : PBufBookmark); override;

    procedure InitialiseIndex; override;
    procedure InitialiseSpareRecord(const ASpareRecord : PChar); override;
    procedure ReleaseSpareRecord; override;

    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    Function GetRecNo(const ABookmark : PBufBookmark) : integer; override;

    procedure BeginUpdate; override;
    procedure AddRecord; override;
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : PChar); override;
    procedure EndUpdate; override;
  end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  { TUniDirectionalBufIndex }

  TUniDirectionalBufIndex = class(TBufIndex)
  private
    FSPareBuffer: PChar;
  protected
    function GetBookmarkSize: integer; override;
    function GetCurrentBuffer: Pointer; override;
=======
>>>>>>> graemeg/fixes_2_2
    function GetCurrentRecord: PChar; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: PChar; override;
    function GetSpareRecord: PChar; override;
  public
<<<<<<< HEAD
=======
    FLastRecBuf     : PBufRecLinkItem;
    FFirstRecBuf    : PBufRecLinkItem;
    FNeedScroll     : Boolean;
>>>>>>> graemeg/fixes_2_2
    function ScrollBackward : TGetResult; override;
    function ScrollForward : TGetResult; override;
    function GetCurrent : TGetResult; override;
    function ScrollFirst : TGetResult; override;
    procedure ScrollLast; override;

    procedure SetToFirstRecord; override;
    procedure SetToLastRecord; override;

    procedure StoreCurrentRecord; override;
    procedure RestoreCurrentRecord; override;

    function CanScrollForward : Boolean; override;
    procedure DoScrollForward; override;

    procedure StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure GotoBookmark(const ABookmark : PBufBookmark); override;

    procedure InitialiseIndex; override;
<<<<<<< HEAD
    procedure InitialiseSpareRecord(const ASpareRecord : PChar); override;
    procedure ReleaseSpareRecord; override;

    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    Function GetRecNo(const ABookmark : PBufBookmark) : integer; override;

    procedure BeginUpdate; override;
    procedure AddRecord; override;
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : PChar); override;
    procedure EndUpdate; override;
  end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  { TUniDirectionalBufIndex }

  TUniDirectionalBufIndex = class(TBufIndex)
  private
    FSPareBuffer: PChar;
  protected
    function GetBookmarkSize: integer; override;
    function GetCurrentBuffer: Pointer; override;
    function GetCurrentRecord: PChar; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: PChar; override;
    function GetSpareRecord: PChar; override;
  public
    function ScrollBackward : TGetResult; override;
    function ScrollForward : TGetResult; override;
    function GetCurrent : TGetResult; override;
    function ScrollFirst : TGetResult; override;
    procedure ScrollLast; override;

    procedure SetToFirstRecord; override;
    procedure SetToLastRecord; override;

    procedure StoreCurrentRecord; override;
    procedure RestoreCurrentRecord; override;

    function CanScrollForward : Boolean; override;
    procedure DoScrollForward; override;

    procedure StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure GotoBookmark(const ABookmark : PBufBookmark); override;

    procedure InitialiseIndex; override;
=======

>>>>>>> graemeg/fixes_2_2
    procedure InitialiseSpareRecord(const ASpareRecord : PChar); override;
    procedure ReleaseSpareRecord; override;

    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    Function GetRecNo(const ABookmark : PBufBookmark) : integer; override;

    procedure BeginUpdate; override;
<<<<<<< HEAD
    procedure AddRecord; override;
=======
    procedure AddRecord(Const ARecord : PChar); override;
>>>>>>> graemeg/fixes_2_2
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : PChar); override;
    procedure EndUpdate; override;
  end;

<<<<<<< HEAD
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/fixes_2_2
  { TArrayBufIndex }

  TArrayBufIndex = class(TBufIndex)
  private
    FStoredRecBuf  : integer;

    FInitialBuffers,
    FGrowBuffer     : integer;
    Function GetRecordFromBookmark(ABookmark: TBufBookmark) : integer;
  protected
    function GetBookmarkSize: integer; override;
    function GetCurrentBuffer: Pointer; override;
<<<<<<< HEAD
    function GetCurrentRecord: TRecordBuffer; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: TRecordBuffer; override;
    function GetSpareRecord: TRecordBuffer; override;
    function GetRecNo: Longint; override;
    procedure SetRecNo(ARecNo: Longint); override;
  public
    FRecordArray    : array of Pointer;
    FCurrentRecInd  : integer;
    FLastRecInd     : integer;
    FNeedScroll     : Boolean;
    constructor Create(const ADataset: TCustomBufDataset); override;
=======
    function GetCurrentRecord: PChar; override;
    function GetIsInitialized: boolean; override;
    function GetSpareBuffer: PChar; override;
    function GetSpareRecord: PChar; override;
  public
    FCurrentRecInd  : integer;
    FRecordArray    : array of Pointer;
    FLastRecInd     : integer;
    FNeedScroll     : Boolean;
    constructor Create(const ADataset: TBufDataset); override;
>>>>>>> graemeg/fixes_2_2
    function ScrollBackward : TGetResult; override;
    function ScrollForward : TGetResult; override;
    function GetCurrent : TGetResult; override;
    function ScrollFirst : TGetResult; override;
    procedure ScrollLast; override;

    procedure SetToFirstRecord; override;
    procedure SetToLastRecord; override;

    procedure StoreCurrentRecord; override;
    procedure RestoreCurrentRecord; override;

    function CanScrollForward : Boolean; override;
    procedure DoScrollForward; override;

    procedure StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark); override;
    procedure GotoBookmark(const ABookmark : PBufBookmark); override;

    procedure InitialiseIndex; override;

<<<<<<< HEAD
    procedure InitialiseSpareRecord(const ASpareRecord : TRecordBuffer); override;
    procedure ReleaseSpareRecord; override;

    procedure BeginUpdate; override;
    procedure AddRecord; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : TRecordBuffer); override;
    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
    procedure InitialiseSpareRecord(const ASpareRecord : PChar); override;
    procedure ReleaseSpareRecord; override;

    Function GetRecNo(const ABookmark : PBufBookmark) : integer; override;
    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : PChar); override;

    procedure BeginUpdate; override;
    procedure AddRecord(Const ARecord : PChar); override;
>>>>>>> graemeg/fixes_2_2
    procedure EndUpdate; override;
  end;


  { TBufDatasetReader }

type
  TRowStateValue = (rsvOriginal, rsvDeleted, rsvInserted, rsvUpdated, rsvDetailUpdates);
  TRowState = set of TRowStateValue;

type

  { TDataPacketReader }

<<<<<<< HEAD
  TDataPacketFormat = (dfBinary,dfXML,dfXMLUTF8,dfAny);

  TDatapacketReaderClass = class of TDatapacketReader;
  TDataPacketReader = class(TObject)
    FDataSet: TCustomBufDataset;
=======
  TDatapacketReaderClass = class of TDatapacketReader;
  TDataPacketReader = class(TObject)
>>>>>>> graemeg/fixes_2_2
    FStream : TStream;
  protected
    class function RowStateToByte(const ARowState : TRowState) : byte;
    class function ByteToRowState(const AByte : Byte) : TRowState;
<<<<<<< HEAD
    procedure RestoreBlobField(AField: TField; ASource: pointer; ASize: integer);
    property DataSet: TCustomBufDataset read FDataSet;
    property Stream: TStream read FStream;
  public
    constructor Create(ADataSet: TCustomBufDataset; AStream : TStream); virtual;
    // Load a dataset from stream:
    // Load the field definitions from a stream.
    procedure LoadFieldDefs(var AnAutoIncValue : integer); virtual; abstract;
    // Is called before the records are loaded
    procedure InitLoadRecords; virtual; abstract;
    // Returns if there is at least one more record available in the stream
    function GetCurrentRecord : boolean; virtual; abstract;
<<<<<<< HEAD
    // Return the RowState of the current record, and the order of the update
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; virtual; abstract;
    // Store a record from stream in the current record buffer
    procedure RestoreRecord; virtual; abstract;
=======
    // Store a record from stream in the current record-buffer
    procedure RestoreRecord(ADataset : TCustomBufDataset); virtual; abstract;
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
=======
  public
    constructor create(AStream : TStream); virtual;
    // Load a dataset from stream:
    // Load the field-definitions from a stream.
    procedure LoadFieldDefs(AFieldDefs : TFieldDefs); virtual; abstract;
    // Is called before the records are loaded
    procedure InitLoadRecords; virtual; abstract;
    // Return the RowState of the current record, and the order of the update
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; virtual; abstract;
    // Returns if there is at least one more record available in the stream
    function GetCurrentRecord : boolean; virtual; abstract;
    // Store a record from stream in the current record-buffer
    procedure RestoreRecord(ADataset : TBufDataset); virtual; abstract;
>>>>>>> graemeg/fixes_2_2
    // Move the stream to the next record
    procedure GotoNextRecord; virtual; abstract;

    // Store a dataset to stream:
<<<<<<< HEAD
<<<<<<< HEAD
    // Save the field definitions to a stream.
    procedure StoreFieldDefs(AnAutoIncValue : integer); virtual; abstract;
    // Save a record from the current record buffer to the stream
    procedure StoreRecord(ARowState : TRowState; AUpdOrder : integer = 0); virtual; abstract;
=======
    // Save the field-definitions to a stream.
    procedure StoreFieldDefs(AFieldDefs : TFieldDefs); virtual; abstract;
    // Save a record from the current record-buffer to the stream
    procedure StoreRecord(ADataset : TCustomBufDataset; ARowState : TRowState; AUpdOrder : integer = 0); virtual; abstract;
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
=======
    // Save the field-definitions to a stream.
    procedure StoreFieldDefs(AFieldDefs : TFieldDefs); virtual; abstract;
    // Save a record from the current record-buffer to the stream
    procedure StoreRecord(ADataset : TBufDataset; ARowState : TRowState; AUpdOrder : integer = 0); virtual; abstract;
>>>>>>> graemeg/fixes_2_2
    // Is called after all records are stored
    procedure FinalizeStoreRecords; virtual; abstract;
    // Checks if the provided stream is of the right format for this class
    class function RecognizeStream(AStream : TStream) : boolean; virtual; abstract;
<<<<<<< HEAD
=======
    property Stream: TStream read FStream;
>>>>>>> graemeg/fixes_2_2
  end;

  { TFpcBinaryDatapacketReader }

<<<<<<< HEAD
  { Data layout:
     Header section:
       Identification: 13 bytes: 'BinBufDataSet'
       Version: 1 byte
     Columns section:
       Number of Fields: 2 bytes
       For each FieldDef: Name, DisplayName, Size: 2 bytes, DataType: 2 bytes, ReadOnlyAttr: 1 byte
     Parameter section:
       AutoInc Value: 4 bytes
     Rows section:
       Row header: each row begins with $fe: 1 byte
                   row state: 1 byte (original, deleted, inserted, modified)
                   update order: 4 bytes
                   null bitmap: 1 byte per each 8 fields (if field is null corresponding bit is 1)
       Row data: variable length data are prefixed with 4 byte length indicator
                 null fields are not stored (see: null bitmap)
  }

  TFpcBinaryDatapacketReader = class(TDataPacketReader)
  private
    const
      FpcBinaryIdent1 = 'BinBufDataset'; // Old version 1; support for transient period;
      FpcBinaryIdent2 = 'BinBufDataSet';
      StringFieldTypes = [ftString,ftFixedChar,ftWideString,ftFixedWideChar];
      BlobFieldTypes = [ftBlob,ftMemo,ftGraphic,ftWideMemo];
      VarLenFieldTypes = StringFieldTypes + BlobFieldTypes + [ftBytes,ftVarBytes];
    var
      FNullBitmapSize: integer;
      FNullBitmap: TBytes;
  protected
    var
      FVersion: byte;
  public
    constructor Create(ADataSet: TCustomBufDataset; AStream : TStream); override;
    procedure LoadFieldDefs(var AnAutoIncValue : integer); override;
    procedure StoreFieldDefs(AnAutoIncValue : integer); override;
    procedure InitLoadRecords; override;
    function GetCurrentRecord : boolean; override;
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; override;
    procedure RestoreRecord; override;
    procedure GotoNextRecord; override;
<<<<<<< HEAD
    procedure StoreRecord(ARowState : TRowState; AUpdOrder : integer = 0); override;
    procedure FinalizeStoreRecords; override;
    class function RecognizeStream(AStream : TStream) : boolean; override;
  end;


=======
    procedure InitLoadRecords; override;
    procedure RestoreRecord(ADataset : TCustomBufDataset); override;
    procedure StoreRecord(ADataset : TCustomBufDataset; ARowState : TRowState; AUpdOrder : integer = 0); override;
    class function RecognizeStream(AStream : TStream) : boolean; override;
  end;

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
  TCustomBufDataset = class(TDBDataSet)
  private
    FFileName: string;
    FReadFromFile   : boolean;
    FFileStream     : TFileStream;
    FDatasetReader  : TDataPacketReader;

    FIndexes        : array of TBufIndex;
    FMaxIndexesCount: integer;
    FIndexesCount   : integer;
    FCurrentIndex   : TBufIndex;

    FFilterBuffer   : TRecordBuffer;
    FBRecordCount   : integer;
    FReadOnly       : Boolean;

    FSavedState     : TDatasetState;
=======
  TFpcBinaryDatapacketReader = class(TDataPacketReader)
  public
    procedure LoadFieldDefs(AFieldDefs : TFieldDefs); override;
    procedure StoreFieldDefs(AFieldDefs : TFieldDefs); override;
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; override;
    procedure FinalizeStoreRecords; override;
    function GetCurrentRecord : boolean; override;
    procedure GotoNextRecord; override;
    procedure InitLoadRecords; override;
    procedure RestoreRecord(ADataset : TBufDataset); override;
    procedure StoreRecord(ADataset : TBufDataset; ARowState : TRowState; AUpdOrder : integer = 0); override;
    class function RecognizeStream(AStream : TStream) : boolean; override;
  end;

  TBufDataset = class(TDBDataSet)
  private
    FFileName: string;
    FFileStream     : TFileStream;
    FDatasetReader  : TDataPacketReader;
    FIndexes        : array of TBufIndex;
    FMaxIndexesCount: integer;

    FIndexesCount   : integer;
    FCurrentIndex   : TBufIndex;

    FFilterBuffer   : pchar;
    FBRecordCount   : integer;

>>>>>>> graemeg/fixes_2_2
    FPacketRecords  : integer;
    FRecordSize     : Integer;
    FNullmaskSize   : byte;
    FOpen           : Boolean;
    FUpdateBuffer   : TRecordsUpdateBuffer;
    FCurrentUpdateBuffer : integer;
<<<<<<< HEAD
    FAutoIncValue   : longint;
    FAutoIncField   : TAutoIncField;

=======
    
>>>>>>> graemeg/fixes_2_2
    FIndexDefs      : TIndexDefs;

    FParser         : TBufDatasetParser;

    FFieldBufPositions : array of longint;

    FAllPacketsFetched : boolean;
    FOnUpdateError  : TResolverErrorEvent;

    FBlobBuffers      : array of PBlobBuffer;
    FUpdateBlobBuffers: array of PBlobBuffer;
<<<<<<< HEAD
    FManualMergeChangeLog : Boolean;
    
<<<<<<< HEAD
    procedure ProcessFieldsToCompareStruct(const AFields, ADescFields, ACInsFields: TList;
      const AIndexOptions: TIndexOptions; const ALocateOptions: TLocateOptions; out ACompareStruct: TDBCompareStruct);
    function BufferOffset: integer;
=======
=======
    
>>>>>>> graemeg/fixes_2_2
=======

    procedure InitialiseSpareRecord(const ASpareRecord : PChar); override;
    procedure ReleaseSpareRecord; override;

    Function GetRecNo(const ABookmark : PBufBookmark) : integer; override;
    procedure RemoveRecordFromIndex(const ABookmark : TBufBookmark); override;
    procedure InsertRecordBeforeCurrentRecord(Const ARecord : PChar); override;

    procedure BeginUpdate; override;
    procedure AddRecord(Const ARecord : PChar); override;
    procedure EndUpdate; override;
  end;


  { TBufDatasetReader }

type
  TRowStateValue = (rsvOriginal, rsvDeleted, rsvInserted, rsvUpdated, rsvDetailUpdates);
  TRowState = set of TRowStateValue;

type

  { TDataPacketReader }

  TDatapacketReaderClass = class of TDatapacketReader;
  TDataPacketReader = class(TObject)
    FStream : TStream;
  protected
    class function RowStateToByte(const ARowState : TRowState) : byte;
    class function ByteToRowState(const AByte : Byte) : TRowState;
  public
    constructor create(AStream : TStream); virtual;
    // Load a dataset from stream:
    // Load the field-definitions from a stream.
    procedure LoadFieldDefs(AFieldDefs : TFieldDefs); virtual; abstract;
    // Is called before the records are loaded
    procedure InitLoadRecords; virtual; abstract;
    // Return the RowState of the current record, and the order of the update
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; virtual; abstract;
    // Returns if there is at least one more record available in the stream
    function GetCurrentRecord : boolean; virtual; abstract;
    // Store a record from stream in the current record-buffer
    procedure RestoreRecord(ADataset : TBufDataset); virtual; abstract;
    // Move the stream to the next record
    procedure GotoNextRecord; virtual; abstract;

    // Store a dataset to stream:
    // Save the field-definitions to a stream.
    procedure StoreFieldDefs(AFieldDefs : TFieldDefs); virtual; abstract;
    // Save a record from the current record-buffer to the stream
    procedure StoreRecord(ADataset : TBufDataset; ARowState : TRowState; AUpdOrder : integer = 0); virtual; abstract;
    // Is called after all records are stored
    procedure FinalizeStoreRecords; virtual; abstract;
    // Checks if the provided stream is of the right format for this class
    class function RecognizeStream(AStream : TStream) : boolean; virtual; abstract;
    property Stream: TStream read FStream;
  end;

  { TFpcBinaryDatapacketReader }

  TFpcBinaryDatapacketReader = class(TDataPacketReader)
  public
    procedure LoadFieldDefs(AFieldDefs : TFieldDefs); override;
    procedure StoreFieldDefs(AFieldDefs : TFieldDefs); override;
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; override;
    procedure FinalizeStoreRecords; override;
    function GetCurrentRecord : boolean; override;
    procedure GotoNextRecord; override;
    procedure InitLoadRecords; override;
    procedure RestoreRecord(ADataset : TBufDataset); override;
    procedure StoreRecord(ADataset : TBufDataset; ARowState : TRowState; AUpdOrder : integer = 0); override;
    class function RecognizeStream(AStream : TStream) : boolean; override;
  end;

  TBufDataset = class(TDBDataSet)
  private
    FFileName: string;
    FFileStream     : TFileStream;
    FDatasetReader  : TDataPacketReader;
    FIndexes        : array of TBufIndex;
    FMaxIndexesCount: integer;

    FIndexesCount   : integer;
    FCurrentIndex   : TBufIndex;

    FFilterBuffer   : pchar;
    FBRecordCount   : integer;

    FPacketRecords  : integer;
    FRecordSize     : Integer;
    FNullmaskSize   : byte;
    FOpen           : Boolean;
    FUpdateBuffer   : TRecordsUpdateBuffer;
    FCurrentUpdateBuffer : integer;
    
    FIndexDefs      : TIndexDefs;

    FParser         : TBufDatasetParser;

    FFieldBufPositions : array of longint;

    FAllPacketsFetched : boolean;
    FOnUpdateError  : TResolverErrorEvent;

    FBlobBuffers      : array of PBlobBuffer;
    FUpdateBlobBuffers: array of PBlobBuffer;
    
>>>>>>> origin/fixes_2_2
    procedure FetchAll;
    procedure BuildIndex(var AIndex : TBufIndex);
    function GetIndexDefs : TIndexDefs;
    function  GetCurrentBuffer: PChar;
    procedure CalcRecordSize;
    function GetIndexFieldNames: String;
    function GetIndexName: String;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetBufUniDirectional: boolean;
    function LoadBuffer(Buffer : PChar): TGetResult;
>>>>>>> graemeg/cpstrnew
    function GetFieldSize(FieldDef : TFieldDef) : longint;
    procedure CalcRecordSize;
    function  IntAllocRecordBuffer: TRecordBuffer;
    procedure IntLoadFieldDefsFromFile;
    procedure IntLoadRecordsFromFile;
    function  GetCurrentBuffer: TRecordBuffer;
    procedure CurrentRecordToBuffer(Buffer: TRecordBuffer);
    function LoadBuffer(Buffer : TRecordBuffer): TGetResult;
    procedure FetchAll;
    function GetRecordUpdateBuffer(const ABookmark : TBufBookmark; IncludePrior : boolean = false; AFindNext : boolean = false) : boolean;
    function GetRecordUpdateBufferCached(const ABookmark : TBufBookmark; IncludePrior : boolean = false) : boolean;
    function GetActiveRecordUpdateBuffer : boolean;
    procedure ParseFilter(const AFilter: string);

    function GetIndexDefs : TIndexDefs;
    function GetIndexFieldNames: String;
    function GetIndexName: String;
    function GetBufUniDirectional: boolean;
    procedure SetIndexFieldNames(const AValue: String);
    procedure SetIndexName(AValue: String);
    procedure SetMaxIndexesCount(const AValue: Integer);
<<<<<<< HEAD
    procedure SetBufUniDirectional(const AValue: boolean);
    // indexes handling
    procedure InitDefaultIndexes;
    procedure BuildIndex(var AIndex : TBufIndex);
    procedure BuildIndexes;
    procedure RemoveRecordFromIndexes(const ABookmark : TBufBookmark);
=======
    procedure SetPacketRecords(aValue : integer);
    function  IntAllocRecordBuffer: PChar;
    procedure DoFilterRecord(out Acceptable: Boolean);
=======
=======
>>>>>>> origin/fixes_2_2
    function LoadBuffer(Buffer : PChar): TGetResult;
    function GetFieldSize(FieldDef : TFieldDef) : longint;
    function GetRecordUpdateBuffer(const ABookmark : TBufBookmark; IncludePrior : boolean = false; AFindNext : boolean = false) : boolean;
    function GetRecordUpdateBufferCached(const ABookmark : TBufBookmark; IncludePrior : boolean = false) : boolean;
    function GetActiveRecordUpdateBuffer : boolean;
    procedure ProcessFieldCompareStruct(AField: TField; var ACompareRec : TDBCompareRec);
    procedure SetIndexFieldNames(const AValue: String);
    procedure SetIndexName(AValue: String);
    procedure SetMaxIndexesCount(const AValue: Integer);
    procedure SetPacketRecords(aValue : integer);
    function  IntAllocRecordBuffer: PChar;
    procedure DoFilterRecord(var Acceptable: Boolean);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure ParseFilter(const AFilter: string);
    procedure IntLoadFielddefsFromFile;
    procedure IntLoadRecordsFromFile;
    procedure CurrentRecordToBuffer(Buffer: PChar);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SetBufUniDirectional(const AValue: boolean);
    procedure InitDefaultIndexes;
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
  protected
    // abstract & virtual methods of TDataset
    procedure SetPacketRecords(aValue : integer); virtual;
    procedure UpdateIndexDefs; override;
    procedure SetRecNo(Value: Longint); override;
    function  GetRecNo: Longint; override;
    function GetChangeCount: integer; virtual;
    function  AllocRecordBuffer: TRecordBuffer; override;
    procedure FreeRecordBuffer(var Buffer: TRecordBuffer); override;
    procedure ClearCalcFields(Buffer: TRecordBuffer); override;
    procedure InternalInitRecord(Buffer: TRecordBuffer); override;
    function  GetCanModify: Boolean; override;
    function GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult; override;
    procedure DoBeforeClose; override;
    procedure InternalInitFieldDefs; override;
    procedure InternalOpen; override;
    procedure InternalClose; override;
    function GetRecordSize: Word; override;
=======
=======
>>>>>>> origin/fixes_2_2
  protected
    procedure UpdateIndexDefs; override;
    function GetNewBlobBuffer : PBlobBuffer;
    function GetNewWriteBlobBuffer : PBlobBuffer;
    procedure FreeBlobBuffer(var ABlobBuffer: PBlobBuffer);
    procedure SetRecNo(Value: Longint); override;
    function  GetRecNo: Longint; override;
    function GetChangeCount: integer; virtual;
    function  AllocRecordBuffer: PChar; override;
    procedure FreeRecordBuffer(var Buffer: PChar); override;
    procedure ClearCalcFields(Buffer: PChar); override;
    procedure InternalInitRecord(Buffer: PChar); override;
    function  GetCanModify: Boolean; override;
    function GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult; override;
    procedure DoBeforeClose; override;
    procedure InternalOpen; override;
    procedure InternalClose; override;
    function getnextpacket : integer;
    function GetRecordSize: Word; override;
    procedure InternalAddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string;
      const ACaseInsFields: string); virtual;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure InternalPost; override;
    procedure InternalCancel; Override;
    procedure InternalDelete; override;
    procedure InternalFirst; override;
    procedure InternalLast; override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure InternalSetToRecord(Buffer: TRecordBuffer); override;
    procedure InternalGotoBookmark(ABookmark: Pointer); override;
    procedure SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    procedure SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag); override;
    procedure GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override;
    function GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag; override;
    function IsCursorOpen: Boolean; override;
    function  GetRecordCount: Longint; override;
    procedure ApplyRecUpdate(UpdateKind : TUpdateKind); virtual;
    procedure SetOnUpdateError(const AValue: TResolverErrorEvent);
    procedure SetFilterText(const Value: String); override; {virtual;}
    procedure SetFiltered(Value: Boolean); override; {virtual;}
    procedure InternalRefresh; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure DataEvent(Event: TDataEvent; Info: PtrInt); override;
    // virtual or methods, which can be used by descendants
    function GetNewBlobBuffer : PBlobBuffer;
    function GetNewWriteBlobBuffer : PBlobBuffer;
    procedure FreeBlobBuffer(var ABlobBuffer: PBlobBuffer);
    procedure InternalAddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string;
      const ACaseInsFields: string); virtual;
    procedure BeforeRefreshOpenCursor; virtual;
    procedure DoFilterRecord(out Acceptable: Boolean); virtual;
    procedure SetReadOnly(AValue: Boolean); virtual;
    function IsReadFromPacket : Boolean;
    function getnextpacket : integer;
    function GetPacketReader(const Format: TDataPacketFormat; const AStream: TStream): TDataPacketReader; virtual;
    // abstracts, must be overidden by descendents
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure BeforeRefreshOpenCursor; virtual;
  {abstracts, must be overidden by descendents}
>>>>>>> graemeg/cpstrnew
    function Fetch : boolean; virtual;
    function LoadField(FieldDef : TFieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; virtual;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField); virtual; abstract;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure InternalSetToRecord(Buffer: PChar); override;
    procedure InternalGotoBookmark(ABookmark: Pointer); override;
    procedure SetBookmarkData(Buffer: PChar; Data: Pointer); override;
    procedure SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag); override;
    procedure GetBookmarkData(Buffer: PChar; Data: Pointer); override;
    function GetBookmarkFlag(Buffer: PChar): TBookmarkFlag; override;
    function IsCursorOpen: Boolean; override;
    function  GetRecordCount: Longint; override;
    procedure ApplyRecUpdate(UpdateKind : TUpdateKind); virtual;
    procedure SetOnUpdateError(const aValue: TResolverErrorEvent);
    procedure SetFilterText(const Value: String); override; {virtual;}
    procedure SetFiltered(Value: Boolean); override; {virtual;}
  {abstracts, must be overidden by descendents}
    function Fetch : boolean; virtual;
    function LoadField(FieldDef : TFieldDef;buffer : pointer; out CreateBlob : boolean) : boolean; virtual;
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef;ABlobBuf: PBufBlobField); virtual; abstract;
    function IsReadFromPacket : Boolean;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  public
    constructor Create(AOwner: TComponent); override;
    function GetFieldData(Field: TField; Buffer: Pointer;
      NativeFormat: Boolean): Boolean; override;
    function GetFieldData(Field: TField; Buffer: Pointer): Boolean; override;
    procedure SetFieldData(Field: TField; Buffer: Pointer;
      NativeFormat: Boolean); override;
    procedure SetFieldData(Field: TField; Buffer: Pointer); override;
    procedure ApplyUpdates; virtual; overload;
    procedure ApplyUpdates(MaxErrors: Integer); virtual; overload;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure MergeChangeLog;
    procedure CancelUpdates; virtual;
    destructor Destroy; override;
    function Locate(const KeyFields: string; const KeyValues: Variant; Options: TLocateOptions) : boolean; override;
    function Lookup(const KeyFields: string; const KeyValues: Variant; const ResultFields: string): Variant; override;
=======
    procedure CancelUpdates; virtual;
    destructor Destroy; override;
    function Locate(const keyfields: string; const keyvalues: Variant; options: TLocateOptions) : boolean; override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure CancelUpdates; virtual;
    destructor Destroy; override;
    function Locate(const keyfields: string; const keyvalues: Variant; options: TLocateOptions) : boolean; override;
>>>>>>> origin/fixes_2_2
    function UpdateStatus: TUpdateStatus; override;
    function CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream; override;
    procedure AddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string = '';
      const ACaseInsFields: string = ''); virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure ClearIndexes;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    procedure SetDatasetPacket(AReader : TDataPacketReader);
    procedure GetDatasetPacket(AWriter : TDataPacketReader);
    procedure LoadFromStream(AStream : TStream; Format: TDataPacketFormat = dfAny);
    procedure SaveToStream(AStream : TStream; Format: TDataPacketFormat = dfBinary);
    procedure LoadFromFile(AFileName: string = ''; Format: TDataPacketFormat = dfAny);
    procedure SaveToFile(AFileName: string = ''; Format: TDataPacketFormat = dfBinary);
    procedure CreateDataset;
    function BookmarkValid(ABookmark: TBookmark): Boolean; override;
    function CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint; override;

    property ChangeCount : Integer read GetChangeCount;
<<<<<<< HEAD
<<<<<<< HEAD
    property MaxIndexesCount : Integer read FMaxIndexesCount write SetMaxIndexesCount default 2;
    property ReadOnly : Boolean read FReadOnly write SetReadOnly default false;
    property ManualMergeChangeLog : Boolean read FManualMergeChangeLog write FManualMergeChangeLog default False;
=======
    property MaxIndexesCount : Integer read FMaxIndexesCount write SetMaxIndexesCount;
>>>>>>> graemeg/fixes_2_2
  published
    property FileName : string read FFileName write FFileName;
    property PacketRecords : Integer read FPacketRecords write SetPacketRecords default 10;
    property OnUpdateError: TResolverErrorEvent read FOnUpdateError write SetOnUpdateError;
    property IndexDefs : TIndexDefs read GetIndexDefs;
    property IndexName : String read GetIndexName write SetIndexName;
    property IndexFieldNames : String read GetIndexFieldNames write SetIndexFieldNames;
<<<<<<< HEAD
    property UniDirectional: boolean read GetBufUniDirectional write SetBufUniDirectional default False;
  end;

  TBufDataset = class(TCustomBufDataset)
  published
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property MaxIndexesCount;
    // TDataset stuff
    property FieldDefs;
    Property Active;
    Property AutoCalcFields;
    Property Filter;
    Property Filtered;
    Property ReadOnly;
    Property AfterCancel;
    Property AfterClose;
    Property AfterDelete;
    Property AfterEdit;
    Property AfterInsert;
    Property AfterOpen;
    Property AfterPost;
    Property AfterScroll;
    Property BeforeCancel;
    Property BeforeClose;
    Property BeforeDelete;
    Property BeforeEdit;
    Property BeforeInsert;
    Property BeforeOpen;
    Property BeforePost;
    Property BeforeScroll;
    Property OnCalcFields;
    Property OnDeleteError;
    Property OnEditError;
    Property OnFilterRecord;
    Property OnNewRecord;
    Property OnPostError;
=======
    property FieldDefs;
>>>>>>> graemeg/cpstrnew
=======
    property FieldDefs;
>>>>>>> graemeg/cpstrnew
=======
    property FieldDefs;
>>>>>>> graemeg/cpstrnew
=======
    property FieldDefs;
>>>>>>> origin/cpstrnew
  end;


=======
  end;

>>>>>>> graemeg/fixes_2_2
procedure RegisterDatapacketReader(ADatapacketReaderClass : TDatapacketReaderClass; AFormat : TDataPacketFormat);

implementation

<<<<<<< HEAD
uses variants, dbconst, FmtBCD;
=======
uses variants, dbconst;
>>>>>>> graemeg/fixes_2_2

Type TDatapacketReaderRegistration = record
                                       ReaderClass : TDatapacketReaderClass;
                                       Format      : TDataPacketFormat;
                                     end;

var RegisteredDatapacketReaders : Array of TDatapacketReaderRegistration;

procedure RegisterDatapacketReader(ADatapacketReaderClass : TDatapacketReaderClass; AFormat : TDataPacketFormat);
begin
  setlength(RegisteredDatapacketReaders,length(RegisteredDatapacketReaders)+1);
  with RegisteredDatapacketReaders[length(RegisteredDatapacketReaders)-1] do
    begin
    Readerclass := ADatapacketReaderClass;
    Format      := AFormat;
    end;
end;

function GetRegisterDatapacketReader(AStream : TStream; AFormat : TDataPacketFormat; var ADataReaderClass : TDatapacketReaderRegistration) : boolean;
var i : integer;
begin
  Result := False;
  for i := 0 to length(RegisteredDatapacketReaders)-1 do if ((AFormat=dfAny) or (AFormat=RegisteredDatapacketReaders[i].Format)) then
    begin
    if (AStream=nil) or (RegisteredDatapacketReaders[i].ReaderClass.RecognizeStream(AStream)) then
      begin
      ADataReaderClass := RegisteredDatapacketReaders[i];
      Result := True;
      if (AStream <> nil) then AStream.Seek(0,soFromBeginning);
      break;
      end;
    AStream.Seek(0,soFromBeginning);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function DBCompareText(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
=======
function DBCompareText(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
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
begin
  if [loCaseInsensitive,loPartialKey]=options then
    Result := AnsiStrLIComp(pchar(subValue),pchar(aValue),length(pchar(subValue)))
  else if [loPartialKey] = options then
    Result := AnsiStrLComp(pchar(subValue),pchar(aValue),length(pchar(subValue)))
  else if [loCaseInsensitive] = options then
=======
function DBCompareText(subValue, aValue: pointer; options: TLocateOptions): LargeInt;

begin
  if loCaseInsensitive in options then
>>>>>>> graemeg/fixes_2_2
    Result := AnsiCompareText(pchar(subValue),pchar(aValue))
  else
    Result := AnsiCompareStr(pchar(subValue),pchar(aValue));
end;

<<<<<<< HEAD
function DBCompareWideText(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
begin
  if [loCaseInsensitive,loPartialKey]=options then
    Result := WideCompareText(pwidechar(subValue),LeftStr(pwidechar(aValue), Length(pwidechar(subValue))))
  else if [loPartialKey] = options then
      Result := WideCompareStr(pwidechar(subValue),LeftStr(pwidechar(aValue), Length(pwidechar(subValue))))
    else if [loCaseInsensitive] = options then
         Result := WideCompareText(pwidechar(subValue),pwidechar(aValue))
       else
         Result := WideCompareStr(pwidechar(subValue),pwidechar(aValue));
end;

function DBCompareByte(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
=======
function DBCompareByte(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
>>>>>>> graemeg/fixes_2_2

begin
  Result := PByte(subValue)^-PByte(aValue)^;
end;

<<<<<<< HEAD
function DBCompareSmallInt(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
=======
function DBCompareSmallInt(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
>>>>>>> graemeg/fixes_2_2

begin
  Result := PSmallInt(subValue)^-PSmallInt(aValue)^;
end;

<<<<<<< HEAD
function DBCompareInt(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
=======
function DBCompareInt(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
>>>>>>> graemeg/fixes_2_2

begin
  Result := PInteger(subValue)^-PInteger(aValue)^;
end;

<<<<<<< HEAD
function DBCompareLargeInt(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
=======
function DBCompareLargeInt(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
>>>>>>> graemeg/fixes_2_2

begin
  // A simple subtraction doesn't work, since it could be that the result
  // doesn't fit into a LargeInt
  if PLargeInt(subValue)^ < PLargeInt(aValue)^ then
    result := -1
  else if PLargeInt(subValue)^  > PLargeInt(aValue)^ then
    result := 1
  else
    result := 0;
end;

<<<<<<< HEAD
function DBCompareWord(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
=======
function DBCompareWord(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
>>>>>>> graemeg/fixes_2_2

begin
  Result := PWord(subValue)^-PWord(aValue)^;
end;

<<<<<<< HEAD
function DBCompareQWord(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
=======
function DBCompareQWord(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
>>>>>>> graemeg/fixes_2_2

begin
  // A simple subtraction doesn't work, since it could be that the result
  // doesn't fit into a LargeInt
  if PQWord(subValue)^ < PQWord(aValue)^ then
    result := -1
  else if PQWord(subValue)^  > PQWord(aValue)^ then
    result := 1
  else
    result := 0;
end;

<<<<<<< HEAD
function DBCompareDouble(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
=======
function DBCompareDouble(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
>>>>>>> graemeg/fixes_2_2
begin
  // A simple subtraction doesn't work, since it could be that the result
  // doesn't fit into a LargeInt
  if PDouble(subValue)^ < PDouble(aValue)^ then
    result := -1
  else if PDouble(subValue)^  > PDouble(aValue)^ then
    result := 1
  else
    result := 0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function DBCompareBCD(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
begin
  result:=BCDCompare(PBCD(subValue)^, PBCD(aValue)^);
end;

function DBCompareBytes(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
begin
  Result := CompareByte(subValue^, aValue^, size);
end;

function DBCompareVarBytes(subValue, aValue: pointer; size: integer; options: TLocateOptions): LargeInt;
var len1, len2: LongInt;
begin
  len1 := PWord(subValue)^;
  len2 := PWord(aValue)^;
  inc(subValue, sizeof(Word));
  inc(aValue, sizeof(Word));
  if len1 > len2 then
    Result := CompareByte(subValue^, aValue^, len2)
  else
    Result := CompareByte(subValue^, aValue^, len1);
  if Result = 0 then
    Result := len1 - len2;
=======
function DBCompareBCD(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
begin
=======
function DBCompareBCD(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
begin
>>>>>>> graemeg/cpstrnew
=======
function DBCompareBCD(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
begin
>>>>>>> graemeg/cpstrnew
=======
function DBCompareBCD(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
begin
>>>>>>> origin/cpstrnew
  // A simple subtraction doesn't work, since it could be that the result
  // doesn't fit into a LargeInt
  if PBCD(subValue)^ < PBCD(aValue)^ then
    result := -1
  else if PBCD(subValue)^  > PBCD(aValue)^ then
    result := 1
  else
    result := 0;
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
end;

procedure unSetFieldIsNull(NullMask : pbyte;x : longint); //inline;
begin
  NullMask[x div 8] := (NullMask[x div 8]) and not (1 shl (x mod 8));
end;

procedure SetFieldIsNull(NullMask : pbyte;x : longint); //inline;
begin
  NullMask[x div 8] := (NullMask[x div 8]) or (1 shl (x mod 8));
end;

function GetFieldIsNull(NullMask : pbyte;x : longint) : boolean; //inline;
begin
  result := ord(NullMask[x div 8]) and (1 shl (x mod 8)) > 0
end;

function IndexCompareRecords(Rec1,Rec2 : pointer; ADBCompareRecs : TDBCompareStruct) : LargeInt;
var IndexFieldNr : Integer;
    IsNull1, IsNull2 : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  for IndexFieldNr:=0 to length(ADBCompareRecs)-1 do with ADBCompareRecs[IndexFieldNr] do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    IsNull1:=GetFieldIsNull(rec1+NullBOff,FieldInd);
    IsNull2:=GetFieldIsNull(rec2+NullBOff,FieldInd);
    if IsNull1 and IsNull2 then
      Result := 0
    else if IsNull1 then
      Result := -1
    else if IsNull2 then
      Result := 1
    else
      Result := CompareFunc(Rec1+Off, Rec2+Off, Size, Options);
=======
=======
>>>>>>> graemeg/cpstrnew
    IsNull1:=GetFieldIsNull(rec1+NullBOff1,FieldInd1);
    IsNull2:=GetFieldIsNull(rec2+NullBOff2,FieldInd2);
    if IsNull1 and IsNull2 then
      result := 0
    else if IsNull1 then
      result := -1
    else if IsNull2 then
      result := 1
    else
      Result := Comparefunc(Rec1+Off1,Rec2+Off2,Options);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
begin
  for IndexFieldNr:=0 to length(ADBCompareRecs)-1 do with ADBCompareRecs[IndexFieldNr] do
    begin
    IsNull1:=GetFieldIsNull(rec1+NullBOff1,FieldInd1);
    IsNull2:=GetFieldIsNull(rec2+NullBOff2,FieldInd2);
    if IsNull1 and IsNull2 then
      result := 0
    else if IsNull1 then
      result := -1
    else if IsNull2 then
      result := 1
    else
      Result := Comparefunc(Rec1+Off1,Rec2+Off2,Options);

=======
function IndexCompareRecords(Rec1,Rec2 : pointer; ADBCompareRecs : TDBCompareStruct) : LargeInt;
var IndexFieldNr : Integer;
begin
  for IndexFieldNr:=0 to length(ADBCompareRecs)-1 do with ADBCompareRecs[IndexFieldNr] do
    begin
    Result := Comparefunc(Rec1+Off1,Rec2+Off2,Options);
>>>>>>> graemeg/fixes_2_2
    if Result <> 0 then
      begin
      if Desc then
        Result := -Result;
      break;
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
{ ---------------------------------------------------------------------
    TCustomBufDataset
  ---------------------------------------------------------------------}

constructor TCustomBufDataset.Create(AOwner : TComponent);
begin
  Inherited Create(AOwner);
  FMaxIndexesCount:=2;
  FIndexesCount:=0;

  FIndexDefs := TIndexDefs.Create(Self);
>>>>>>> graemeg/cpstrnew

    if Result <> 0 then
      begin
      if Desc then
        Result := -Result;
      break;
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ ---------------------------------------------------------------------
    TCustomBufDataset
  ---------------------------------------------------------------------}

constructor TCustomBufDataset.Create(AOwner : TComponent);
begin
  Inherited Create(AOwner);
  FManualMergeChangeLog := False;
  FMaxIndexesCount:=2;
  FIndexesCount:=0;

  FIndexDefs := TIndexDefs.Create(Self);
  FAutoIncValue:=-1;
=======
procedure unSetFieldIsNull(NullMask : pbyte;x : longint); //inline;
begin
  NullMask[x div 8] := (NullMask[x div 8]) and not (1 shl (x mod 8));
end;

procedure SetFieldIsNull(NullMask : pbyte;x : longint); //inline;
begin
  NullMask[x div 8] := (NullMask[x div 8]) or (1 shl (x mod 8));
end;

function GetFieldIsNull(NullMask : pbyte;x : longint) : boolean; //inline;
begin
  result := ord(NullMask[x div 8]) and (1 shl (x mod 8)) > 0
end;

{ ---------------------------------------------------------------------
    TBufDataSet
  ---------------------------------------------------------------------}

constructor TBufDataset.Create(AOwner : TComponent);
begin
  Inherited Create(AOwner);
  FMaxIndexesCount:=2;
  FIndexesCount:=0;
  InternalAddIndex('DEFAULT_ORDER','',[],'','');
  FCurrentIndex:=FIndexes[0];
  InternalAddIndex('','',[],'','');

  FIndexDefs := TIndexDefs.Create(Self);
>>>>>>> graemeg/fixes_2_2

  SetLength(FUpdateBuffer,0);
  SetLength(FBlobBuffers,0);
  SetLength(FUpdateBlobBuffers,0);
<<<<<<< HEAD
=======
  BookmarkSize := FCurrentIndex.BookmarkSize;
>>>>>>> graemeg/fixes_2_2
  FParser := nil;
  FPacketRecords := 10;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.SetPacketRecords(aValue : integer);
begin
  if (aValue = -1) or (aValue > 0) then
    begin
    if (IndexFieldNames='') then
      FPacketRecords := aValue
    else if AValue<>-1 then
      DatabaseError(SInvPacketRecordsValueFieldNames);
    end
  else
    DatabaseError(SInvPacketRecordsValue);
end;

destructor TCustomBufDataset.Destroy;

=======
procedure TBufDataset.SetPacketRecords(aValue : integer);
begin
  if (aValue = -1) or (aValue > 0) then FPacketRecords := aValue
    else DatabaseError(SInvPacketRecordsValue);
end;

destructor TBufDataset.Destroy;

Var
  I : Integer;
>>>>>>> graemeg/fixes_2_2
begin
  if Active then Close;
  SetLength(FUpdateBuffer,0);
  SetLength(FBlobBuffers,0);
  SetLength(FUpdateBlobBuffers,0);
<<<<<<< HEAD
  ClearIndexes;
=======
  For I:=0 to Length(FIndexes)-1 do
    FreeAndNil(Findexes[I]);
  SetLength(FIndexes,0);
>>>>>>> graemeg/fixes_2_2
  FreeAndNil(FIndexDefs);
  inherited destroy;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.FetchAll;
=======
procedure TBufDataset.FetchAll;
>>>>>>> graemeg/fixes_2_2
begin
  repeat
  until (getnextpacket < FPacketRecords) or (FPacketRecords = -1);
end;

<<<<<<< HEAD
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// Code to dump raw dataset data, including indexes information, useful for debugging
=======
// Code to dump raw dataset data, including indexes information, usefull for debugging
>>>>>>> graemeg/cpstrnew
=======
// Code to dump raw dataset data, including indexes information, usefull for debugging
>>>>>>> graemeg/cpstrnew
=======
// Code to dump raw dataset data, including indexes information, usefull for debugging
>>>>>>> graemeg/cpstrnew
=======
// Code to dump raw dataset data, including indexes information, usefull for debugging
>>>>>>> origin/cpstrnew
  procedure DumpRawMem(const Data: pointer; ALength: PtrInt);
  var
    b: integer;
    s1,s2: string;
  begin
    s1 := '';
    s2 := '';
    for b := 0 to ALength-1 do
      begin
      s1 := s1 + ' ' + hexStr(pbyte(Data)[b],2);
      if pchar(Data)[b] in ['a'..'z','A'..'Z','1'..'9',' '..'/',':'..'@'] then
        s2 := s2 + pchar(Data)[b]
      else
        s2 := s2 + '.';
      if length(s2)=16 then
        begin
        write('    ',s1,'    ');
        writeln(s2);
        s1 := '';
        s2 := '';
        end;
      end;
    write('    ',s1,'    ');
    writeln(s2);
  end;

  procedure DumpRecord(Dataset: TCustomBufDataset; RecBuf: PBufRecLinkItem; RawData: boolean = false);
  var ptr: pointer;
      NullMask: pointer;
      FieldData: pointer;
      NullMaskSize: integer;
      i: integer;
  begin
    if RawData then
      DumpRawMem(RecBuf,Dataset.RecordSize)
    else
      begin
      ptr := RecBuf;
      NullMask:= ptr + (sizeof(TBufRecLinkItem)*Dataset.MaxIndexesCount);
      NullMaskSize := 1+(Dataset.Fields.Count-1) div 8;
      FieldData:= ptr + (sizeof(TBufRecLinkItem)*Dataset.MaxIndexesCount) +NullMaskSize;
      write('record: $',hexstr(ptr),'  nullmask: $');
      for i := 0 to NullMaskSize-1 do
        write(hexStr(byte((NullMask+i)^),2));
      write('=');
      for i := 0 to NullMaskSize-1 do
        write(binStr(byte((NullMask+i)^),8));
      writeln('%');
      for i := 0 to Dataset.MaxIndexesCount-1 do
        writeln('  ','Index ',inttostr(i),' Prior rec: ' + hexstr(pointer((ptr+(i*2)*sizeof(ptr))^)) + ' Next rec: ' + hexstr(pointer((ptr+((i*2)+1)*sizeof(ptr))^)));
      DumpRawMem(FieldData,Dataset.RecordSize-((sizeof(TBufRecLinkItem)*Dataset.MaxIndexesCount) +NullMaskSize));
      end;
  end;

  procedure DumpDataset(AIndex: TBufIndex;RawData: boolean = false);
  var RecBuf: PBufRecLinkItem;
  begin
    writeln('Dump records, order based on index ',AIndex.IndNr);
    writeln('Current record:',hexstr(AIndex.CurrentRecord));

    RecBuf:=(AIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
    while RecBuf<>(AIndex as TDoubleLinkedBufIndex).FLastRecBuf do
      begin
      DumpRecord(AIndex.FDataset,RecBuf,RawData);
      RecBuf:=RecBuf[(AIndex as TDoubleLinkedBufIndex).IndNr].next;
      end;
  end;
}

procedure TCustomBufDataset.BuildIndex(var AIndex: TBufIndex);
=======
procedure TBufDataset.BuildIndex(var AIndex: TBufIndex);
>>>>>>> graemeg/fixes_2_2

var PCurRecLinkItem : PBufRecLinkItem;
    p,l,q           : PBufRecLinkItem;
    i,k,psize,qsize : integer;
    MergeAmount     : integer;
    PlaceQRec       : boolean;

    IndexFields     : TList;
    DescIndexFields : TList;
    CInsIndexFields : TList;
<<<<<<< HEAD

    Index0,
    DblLinkIndex    : TDoubleLinkedBufIndex;

  procedure PlaceNewRec(var e: PBufRecLinkItem; var esize: integer);
  begin
    if DblLinkIndex.FFirstRecBuf=nil then
     begin
     DblLinkIndex.FFirstRecBuf:=e;
     e[DblLinkIndex.IndNr].prior:=nil;
=======
    FieldsAmount    : Integer;
    FieldNr         : integer;
    AField          : TField;

  procedure PlaceNewRec(var e: PBufRecLinkItem; var esize: integer);
  begin
    if (AIndex as TDoubleLinkedBufIndex).FFirstRecBuf=nil then
     begin
     (AIndex as TDoubleLinkedBufIndex).FFirstRecBuf:=e;
     e[(AIndex as TDoubleLinkedBufIndex).IndNr].prior:=nil;
>>>>>>> graemeg/fixes_2_2
     l:=e;
     end
   else
     begin
<<<<<<< HEAD
     l[DblLinkIndex.IndNr].next:=e;
     e[DblLinkIndex.IndNr].prior:=l;
     l:=e;
     end;
   e := e[DblLinkIndex.IndNr].next;
=======
     l[(AIndex as TDoubleLinkedBufIndex).IndNr].next:=e;
     e[(AIndex as TDoubleLinkedBufIndex).IndNr].prior:=l;
     l:=e;
     end;
   e := e[(AIndex as TDoubleLinkedBufIndex).IndNr].next;
>>>>>>> graemeg/fixes_2_2
   dec(esize);
  end;

begin
  // Build the DBCompareStructure
<<<<<<< HEAD
  // One AS is enough, and makes debugging easier.
  DblLinkIndex:=(AIndex as TDoubleLinkedBufIndex);
  Index0:=(FIndexes[0] as TDoubleLinkedBufIndex);
  with DblLinkIndex do
=======
  with AIndex do
>>>>>>> graemeg/fixes_2_2
    begin
    IndexFields := TList.Create;
    DescIndexFields := TList.Create;
    CInsIndexFields := TList.Create;
    try
      GetFieldList(IndexFields,FieldsName);
<<<<<<< HEAD
      GetFieldList(DescIndexFields,DescFields);
      GetFieldList(CInsIndexFields,CaseinsFields);
      if IndexFields.Count=0 then
        DatabaseError(SNoIndexFieldNameGiven);
      ProcessFieldsToCompareStruct(IndexFields, DescIndexFields, CInsIndexFields, Options, [], DBCompareStruct);
=======
      FieldsAmount:=IndexFields.Count;
      GetFieldList(DescIndexFields,DescFields);
      GetFieldList(CInsIndexFields,CaseinsFields);
      if FieldsAmount=0 then
        DatabaseError(SNoIndexFieldNameGiven);
      SetLength(DBCompareStruct,FieldsAmount);
      for FieldNr:=0 to FieldsAmount-1 do
        begin
        AField := TField(IndexFields[FieldNr]);
        ProcessFieldCompareStruct(AField,DBCompareStruct[FieldNr]);

        DBCompareStruct[FieldNr].Desc := (DescIndexFields.IndexOf(AField)>-1);
        if (CInsIndexFields.IndexOf(AField)>-1) then
          DBCompareStruct[FieldNr].Options := [loCaseInsensitive]
        else
          DBCompareStruct[FieldNr].Options := [];

        end;
>>>>>>> graemeg/fixes_2_2
    finally
      CInsIndexFields.Free;
      DescIndexFields.Free;
      IndexFields.Free;
    end;
    end;

<<<<<<< HEAD
  // This simply copies the index...
  PCurRecLinkItem:=Index0.FFirstRecBuf;
  PCurRecLinkItem[DblLinkIndex.IndNr].next := PCurRecLinkItem[0].next;
  PCurRecLinkItem[DblLinkIndex.IndNr].prior := PCurRecLinkItem[0].prior;

  if PCurRecLinkItem <> Index0.FLastRecBuf then
    begin
    while PCurRecLinkItem^.next<>Index0.FLastRecBuf do
      begin
      PCurRecLinkItem:=PCurRecLinkItem^.next;

      PCurRecLinkItem[DblLinkIndex.IndNr].next := PCurRecLinkItem[0].next;
      PCurRecLinkItem[DblLinkIndex.IndNr].prior := PCurRecLinkItem[0].prior;
      end;
    end
  else
    // Empty dataset
    Exit;

  // Set FirstRecBuf and FCurrentRecBuf
  DblLinkIndex.FFirstRecBuf:=Index0.FFirstRecBuf;
  DblLinkIndex.FCurrentRecBuf:=DblLinkIndex.FFirstRecBuf;
  // Link in the FLastRecBuf that belongs to this index
  PCurRecLinkItem[DblLinkIndex.IndNr].next:=DblLinkIndex.FLastRecBuf;
  DblLinkIndex.FLastRecBuf[DblLinkIndex.IndNr].prior:=PCurRecLinkItem;

  // Mergesort. Used the algorithm as described here by Simon Tatham
  // http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
  // The comments in the code are from this website.

  // In each pass, we are merging lists of size K into lists of size 2K.
  // (Initially K equals 1.)
=======
// This simply copies the index...
  PCurRecLinkItem:=(FIndexes[0] as TDoubleLinkedBufIndex).FFirstRecBuf;
  PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].next := PCurRecLinkItem[0].next;
  PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].prior := PCurRecLinkItem[0].prior;

  if PCurRecLinkItem <> (FIndexes[0] as TDoubleLinkedBufIndex).FLastRecBuf then
    begin
    while PCurRecLinkItem^.next<>(FIndexes[0] as TDoubleLinkedBufIndex).FLastRecBuf do
      begin
      PCurRecLinkItem:=PCurRecLinkItem^.next;

      PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].next := PCurRecLinkItem[0].next;
      PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].prior := PCurRecLinkItem[0].prior;
      end;
    end;

// Set FirstRecBuf and FCurrentRecBuf
  (AIndex as TDoubleLinkedBufIndex).FFirstRecBuf:=(FIndexes[0] as TDoubleLinkedBufIndex).FFirstRecBuf;
  (FCurrentIndex as TDoubleLinkedBufIndex).FCurrentRecBuf:=(AIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
// Link in the FLastRecBuf that belongs to this index
  PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].next:=(AIndex as TDoubleLinkedBufIndex).FLastRecBuf;
  (AIndex as TDoubleLinkedBufIndex).FLastRecBuf[(AIndex as TDoubleLinkedBufIndex).IndNr].prior:=PCurRecLinkItem;

// Mergesort. Used the algorithm as described here by Simon Tatham
// http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
// The comments in the code are from this website.

// In each pass, we are merging lists of size K into lists of size 2K.
// (Initially K equals 1.)
>>>>>>> graemeg/fixes_2_2
  k:=1;

  repeat

<<<<<<< HEAD
  // So we start by pointing a temporary pointer p at the head of the list,
  // and also preparing an empty list L which we will add elements to the end
  // of as we finish dealing with them.
  p := DblLinkIndex.FFirstRecBuf;
  DblLinkIndex.FFirstRecBuf := nil;
  q := p;
  MergeAmount := 0;

  // Then:
  // * If p is null, terminate this pass.
  while p <> DblLinkIndex.FLastRecBuf do
    begin

    //  * Otherwise, there is at least one element in the next pair of length-K
    //    lists, so increment the number of merges performed in this pass.
    inc(MergeAmount);

    //  * Point another temporary pointer, q, at the same place as p. Step q along
    //    the list by K places, or until the end of the list, whichever comes
    //    first. Let psize be the number of elements you managed to step q past.
    i:=0;
    while (i<k) and (q<>DblLinkIndex.FLastRecBuf) do
      begin
      inc(i);
      q := q[DblLinkIndex.IndNr].next;
      end;
    psize :=i;

    //  * Let qsize equal K. Now we need to merge a list starting at p, of length
    //    psize, with a list starting at q of length at most qsize.
    qsize:=k;

    //  * So, as long as either the p-list is non-empty (psize > 0) or the q-list
    //    is non-empty (qsize > 0 and q points to something non-null):
    while (psize>0) or ((qsize>0) and (q <> DblLinkIndex.FLastRecBuf)) do
      begin
      //  * Choose which list to take the next element from. If either list
      //    is empty, we must choose from the other one. (By assumption, at
      //    least one is non-empty at this point.) If both lists are
      //    non-empty, compare the first element of each and choose the lower
      //    one. If the first elements compare equal, choose from the p-list.
      //    (This ensures that any two elements which compare equal are never
      //    swapped, so stability is guaranteed.)
      if (psize=0)  then
        PlaceQRec := true
      else if (qsize=0) or (q = DblLinkIndex.FLastRecBuf) then
        PlaceQRec := False
      else if IndexCompareRecords(p,q,DblLinkIndex.DBCompareStruct) <= 0 then
=======
// So we start by pointing a temporary pointer p at the head of the list,
// and also preparing an empty list L which we will add elements to the end
// of as we finish dealing with them.

  p := (AIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  (AIndex as TDoubleLinkedBufIndex).ffirstRecBuf := nil;
  q := p;
  MergeAmount := 0;

// Then:
//    * If p is null, terminate this pass.
  while p <> (AIndex as TDoubleLinkedBufIndex).FLastRecBuf do
    begin

//    * Otherwise, there is at least one element in the next pair of length-K
//      lists, so increment the number of merges performed in this pass.

    inc(MergeAmount);

//    * Point another temporary pointer, q, at the same place as p. Step q along
//      the list by K places, or until the end of the list, whichever comes
//      first. Let psize be the number of elements you managed to step q past.

    i:=0;
    while (i<k) and (q<>(AIndex as TDoubleLinkedBufIndex).FLastRecBuf) do
      begin
      inc(i);
      q := q[(AIndex as TDoubleLinkedBufIndex).IndNr].next;
      end;
    psize :=i;

//    * Let qsize equal K. Now we need to merge a list starting at p, of length
//      psize, with a list starting at q of length at most qsize.

    qsize:=k;

//    * So, as long as either the p-list is non-empty (psize > 0) or the q-list
//      is non-empty (qsize > 0 and q points to something non-null):

    while (psize>0) or ((qsize>0) and (q <> (AIndex as TDoubleLinkedBufIndex).FLastRecBuf)) do
      begin
//          o Choose which list to take the next element from. If either list
//            is empty, we must choose from the other one. (By assumption, at
//            least one is non-empty at this point.) If both lists are
//            non-empty, compare the first element of each and choose the lower
//            one. If the first elements compare equal, choose from the p-list.
//            (This ensures that any two elements which compare equal are never
//            swapped, so stability is guaranteed.)
      if (psize=0)  then
        PlaceQRec := true
      else if (qsize=0) or (q = (AIndex as TDoubleLinkedBufIndex).FLastRecBuf) then
        PlaceQRec := False
      else if IndexCompareRecords(p,q,aindex.DBCompareStruct) <= 0 then
>>>>>>> graemeg/fixes_2_2
        PlaceQRec := False
      else
        PlaceQRec := True;
        
<<<<<<< HEAD
      //  * Remove that element, e, from the start of its list, by advancing
      //    p or q to the next element along, and decrementing psize or qsize.
      //  * Add e to the end of the list L we are building up.
=======
//          o Remove that element, e, from the start of its list, by advancing
//            p or q to the next element along, and decrementing psize or qsize.
//          o Add e to the end of the list L we are building up.
>>>>>>> graemeg/fixes_2_2
      if PlaceQRec then
        PlaceNewRec(q,qsize)
      else
        PlaceNewRec(p,psize);
      end;
<<<<<<< HEAD
      
    //  * Now we have advanced p until it is where q started out, and we have
    //    advanced q until it is pointing at the next pair of length-K lists to
    //    merge. So set p to the value of q, and go back to the start of this loop.
    p:=q;
    end;

  // As soon as a pass like this is performed and only needs to do one merge, the
  // algorithm terminates, and the output list L is sorted. Otherwise, double the
  // value of K, and go back to the beginning.

  l[DblLinkIndex.IndNr].next:=DblLinkIndex.FLastRecBuf;
=======
//    * Now we have advanced p until it is where q started out, and we have
//      advanced q until it is pointing at the next pair of length-K lists to
//      merge. So set p to the value of q, and go back to the start of this loop.
    p:=q;
    end;

// As soon as a pass like this is performed and only needs to do one merge, the
// algorithm terminates, and the output list L is sorted. Otherwise, double the
// value of K, and go back to the beginning.

  l[(AIndex as TDoubleLinkedBufIndex).IndNr].next:=(AIndex as TDoubleLinkedBufIndex).FLastRecBuf;
>>>>>>> graemeg/fixes_2_2

  k:=k*2;

  until MergeAmount = 1;
<<<<<<< HEAD
  DblLinkIndex.FLastRecBuf[DblLinkIndex.IndNr].next:=DblLinkIndex.FFirstRecBuf;
  DblLinkIndex.FLastRecBuf[DblLinkIndex.IndNr].prior:=l;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.BuildIndexes;
var i: integer;
begin
  for i:=1 to FIndexesCount-1 do
    if (i<>1) or (FIndexes[i]=FCurrentIndex) then
      BuildIndex(FIndexes[i]);
end;

procedure TCustomBufDataset.ClearIndexes;
var
  i:integer;
begin
  CheckInactive;
  For I:=0 to Length(FIndexes)-1 do
    FreeAndNil(FIndexes[I]);
  SetLength(FIndexes,0);
  FIndexesCount:=0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.RemoveRecordFromIndexes(const ABookmark: TBufBookmark);
var i: integer;
begin
  for i:=0 to FIndexesCount-1 do
    if (i<>1) or (FIndexes[i]=FCurrentIndex) then
      FIndexes[i].RemoveRecordFromIndex(ABookmark);
end;

<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCustomBufDataset.GetIndexDefs : TIndexDefs;
=======
  (AIndex as TDoubleLinkedBufIndex).FLastRecBuf[(AIndex as TDoubleLinkedBufIndex).IndNr].next:=(AIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  (AIndex as TDoubleLinkedBufIndex).FLastRecBuf[(AIndex as TDoubleLinkedBufIndex).IndNr].prior:=l;
end;

function TBufDataset.GetIndexDefs : TIndexDefs;
>>>>>>> graemeg/fixes_2_2

begin
  Result := FIndexDefs;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.UpdateIndexDefs;
=======
procedure TBufDataset.UpdateIndexDefs;
>>>>>>> graemeg/fixes_2_2
var i : integer;
begin
  FIndexDefs.Clear;
  for i := 0 to high(FIndexes) do with FIndexDefs.AddIndexDef do
    begin
    Name := FIndexes[i].Name;
    Fields := FIndexes[i].FieldsName;
    DescFields:= FIndexes[i].DescFields;
    CaseInsFields:=FIndexes[i].CaseinsFields;
    Options:=FIndexes[i].Options;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.GetCanModify: Boolean;
=======
Function TCustomBufDataset.GetCanModify: Boolean;
>>>>>>> graemeg/cpstrnew
begin
  Result:=not (UniDirectional or ReadOnly);
end;

<<<<<<< HEAD
function TCustomBufDataset.BufferOffset: integer;
=======
Function TCustomBufDataset.GetCanModify: Boolean;
>>>>>>> graemeg/cpstrnew
=======
Function TCustomBufDataset.GetCanModify: Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.intAllocRecordBuffer: PChar;
>>>>>>> graemeg/cpstrnew
begin
  // Returns the offset of data buffer in bufdataset record
  Result := sizeof(TBufRecLinkItem) * FMaxIndexesCount;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.IntAllocRecordBuffer: TRecordBuffer;
=======
function TCustomBufDataset.intAllocRecordBuffer: PChar;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.intAllocRecordBuffer: PChar;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.AllocRecordBuffer: PChar;
>>>>>>> graemeg/cpstrnew
begin
  // Note: Only the internal buffers of TDataset provide bookmark information
  result := AllocMem(FRecordSize+BufferOffset);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.AllocRecordBuffer: TRecordBuffer;
=======
function TCustomBufDataset.AllocRecordBuffer: PChar;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.AllocRecordBuffer: PChar;
>>>>>>> graemeg/cpstrnew
begin
  result := AllocMem(FRecordSize + BookmarkSize + CalcFieldsSize);
  // The records are initialised, or else the fields of an empty, just-opened dataset
  // are not null
  InitRecord(result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.FreeRecordBuffer(var Buffer: TRecordBuffer);
=======
procedure TCustomBufDataset.FreeRecordBuffer(var Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.FreeRecordBuffer(var Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.FreeRecordBuffer(var Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
Function TCustomBufDataset.GetCanModify: Boolean;
=======
Function TBufDataset.GetCanModify: Boolean;
>>>>>>> graemeg/fixes_2_2
begin
  Result:= True;
end;

<<<<<<< HEAD
function TCustomBufDataset.intAllocRecordBuffer: PChar;
=======
function TBufDataset.intAllocRecordBuffer: PChar;
>>>>>>> graemeg/fixes_2_2
begin
  // Note: Only the internal buffers of TDataset provide bookmark information
  result := AllocMem(FRecordsize+sizeof(TBufRecLinkItem)*FMaxIndexesCount);
end;

<<<<<<< HEAD
function TCustomBufDataset.AllocRecordBuffer: PChar;
=======
function TBufDataset.AllocRecordBuffer: PChar;
>>>>>>> graemeg/fixes_2_2
begin
  result := AllocMem(FRecordsize + BookmarkSize + CalcfieldsSize);
// The records are initialised, or else the fields of an empty, just-opened dataset
// are not null
  InitRecord(result);
end;

<<<<<<< HEAD
procedure TCustomBufDataset.FreeRecordBuffer(var Buffer: PChar);
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.FreeRecordBuffer(var Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
begin
  ReAllocMem(Buffer,0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.ClearCalcFields(Buffer: TRecordBuffer);
=======
procedure TCustomBufDataset.ClearCalcFields(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.ClearCalcFields(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.ClearCalcFields(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.ClearCalcFields(Buffer: PChar);
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.ClearCalcFields(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
begin
  if CalcFieldsSize > 0 then
    FillByte((Buffer+RecordSize)^,CalcFieldsSize,0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.InternalInitFieldDefs;
begin
  if FileName<>'' then
    begin
    IntLoadFieldDefsFromFile;
    FreeAndNil(FDatasetReader);
    FreeAndNil(FFileStream);
    end;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomBufDataset.InternalOpen;

var IndexNr : integer;
    i : integer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if assigned(FDatasetReader) or (FileName<>'') then
    IntLoadFieldDefsFromFile;

  // This checks if the dataset is actually created (by calling CreateDataset,
  // or reading from a stream in some other way implemented by a descendent)
  // If there are less fields than FieldDefs we know for sure that the dataset
  // is not (correctly) created.

  // If there are constant expressions in the select statement (for PostgreSQL)
  // they are of type ftUnknown (in FieldDefs), and are not created (in Fields).
  // So Fields.Count < FieldDefs.Count in this case
  // See mantis #22030

  //  if Fields.Count<FieldDefs.Count then
  if Fields.Count = 0 then
    DatabaseError(SErrNoDataset);

  // search for autoinc field
  FAutoIncField:=nil;
  if FAutoIncValue>-1 then
  begin
    for i := 0 to Fields.Count-1 do
      if Fields[i] is TAutoIncField then
      begin
        FAutoIncField := TAutoIncField(Fields[i]);
        Break;
      end;
  end;

  InitDefaultIndexes;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  InitDefaultIndexes;
=======
procedure TBufDataset.InternalOpen;

var IndexNr : integer;

begin
>>>>>>> graemeg/fixes_2_2
  if not Assigned(FDatasetReader) and (FileName<>'') then
    begin
    FFileStream := TFileStream.Create(FileName,fmOpenRead);
    FDatasetReader := TFpcBinaryDatapacketReader.Create(FFileStream);
<<<<<<< HEAD
    FReadFromFile := True;
    end;
  if assigned(FDatasetReader) then IntLoadFielddefsFromFile;
>>>>>>> graemeg/cpstrnew
  CalcRecordSize;

  FBRecordCount := 0;
=======
    end;
  if assigned(FDatasetReader) then IntLoadFielddefsFromFile;
  CalcRecordSize;

  FBRecordcount := 0;
>>>>>>> graemeg/fixes_2_2

  for IndexNr:=0 to FIndexesCount-1 do with FIndexes[IndexNr] do
    InitialiseSpareRecord(IntAllocRecordBuffer);

  FAllPacketsFetched := False;

  FOpen:=True;

  // parse filter expression
<<<<<<< HEAD
  ParseFilter(Filter);
=======
  try
    ParseFilter(Filter);
  except
    // oops, a problem with parsing, clear filter for now
    on E: Exception do Filter := EmptyStr;
  end;
>>>>>>> graemeg/fixes_2_2

  if assigned(FDatasetReader) then IntLoadRecordsFromFile;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.DoBeforeClose;
begin
  inherited DoBeforeClose;
  if FFileName<>'' then
    SaveToFile(FFileName);
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomBufDataset.InternalClose;

var r  : integer;
    iGetResult : TGetResult;
    pc : TRecordBuffer;

begin
  FOpen:=False;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FReadFromFile:=False;
  FBRecordCount:=0;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if FIndexesCount>0 then with FIndexes[0] do if IsInitialized then
=======
procedure TBufDataset.InternalClose;

var r  : integer;
    iGetResult : TGetResult;
    pc : pchar;

begin
  FOpen:=False;
  with FIndexes[0] do if IsInitialized then
>>>>>>> graemeg/fixes_2_2
    begin
    iGetResult:=ScrollFirst;
    while iGetResult = grOK do
      begin
      pc := pointer(CurrentRecord);
      iGetResult:=ScrollForward;
      FreeRecordBuffer(pc);
      end;
    end;

  for r := 0 to FIndexesCount-1 do with FIndexes[r] do if IsInitialized then
    begin
    pc := SpareRecord;
    ReleaseSpareRecord;
    FreeRecordBuffer(pc);
    end;

  if Length(FUpdateBuffer) > 0 then
    begin
    for r := 0 to length(FUpdateBuffer)-1 do with FUpdateBuffer[r] do
      begin
      if assigned(OldValuesBuffer) then
        FreeRecordBuffer(OldValuesBuffer);
<<<<<<< HEAD
      if (UpdateKind = ukDelete) and assigned(BookmarkData.BookmarkData) then
        FreeRecordBuffer(TRecordBuffer(BookmarkData.BookmarkData));
=======
>>>>>>> graemeg/fixes_2_2
      end;
    end;
  SetLength(FUpdateBuffer,0);
  
  for r := 0 to High(FBlobBuffers) do
    FreeBlobBuffer(FBlobBuffers[r]);
  for r := 0 to High(FUpdateBlobBuffers) do
    FreeBlobBuffer(FUpdateBlobBuffers[r]);

  SetLength(FBlobBuffers,0);
  SetLength(FUpdateBlobBuffers,0);

  SetLength(FFieldBufPositions,0);

<<<<<<< HEAD
  if FAutoIncValue>-1 then FAutoIncValue:=1;

  if assigned(FParser) then FreeAndNil(FParser);
  FReadFromFile:=false;
end;

procedure TCustomBufDataset.InternalFirst;
begin
  with FCurrentIndex do
    // if FCurrentRecBuf = FLastRecBuf then the dataset is just opened and empty
    // in which case InternalFirst should do nothing (bug 7211)
    SetToFirstRecord;
end;

procedure TCustomBufDataset.InternalLast;
begin
  FetchAll;
  with FCurrentIndex do
    SetToLastRecord;
end;

{ TBufIndex }

constructor TBufIndex.Create(const ADataset: TCustomBufDataset);
begin
  inherited create;
  FDataset := ADataset;
end;

function TBufIndex.BookmarkValid(const ABookmark: PBufBookmark): boolean;
begin
  Result := assigned(ABookmark) and assigned(ABookmark^.BookmarkData);
end;

function TBufIndex.CompareBookmarks(const ABookmark1, ABookmark2: PBufBookmark): integer;
begin
  Result := 0;
end;

function TBufIndex.SameBookmarks(const ABookmark1, ABookmark2: PBufBookmark): boolean;
begin
  Result := Assigned(ABookmark1) and Assigned(ABookmark2) and (CompareBookmarks(ABookmark1, ABookmark2) = 0);
end;

function TBufIndex.GetRecord(ABookmark: PBufBookmark; GetMode: TGetMode): TGetResult;
begin
  Result := grError;
end;

{ TDoubleLinkedBufIndex }

function TDoubleLinkedBufIndex.GetBookmarkSize: integer;
begin
  Result:=sizeof(TBufBookmark);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDoubleLinkedBufIndex.GetCurrentBuffer: Pointer;
=======
constructor TBufIndex.Create(const ADataset: TCustomBufDataset);
>>>>>>> graemeg/cpstrnew
=======
constructor TBufIndex.Create(const ADataset: TCustomBufDataset);
>>>>>>> graemeg/cpstrnew
begin
  Result := pointer(FCurrentRecBuf) + FDataset.BufferOffset;
end;

function TDoubleLinkedBufIndex.GetCurrentRecord: TRecordBuffer;
begin
  Result := TRecordBuffer(FCurrentRecBuf);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDoubleLinkedBufIndex.GetIsInitialized: boolean;
=======
constructor TBufIndex.Create(const ADataset: TCustomBufDataset);
>>>>>>> graemeg/cpstrnew
=======
constructor TBufIndex.Create(const ADataset: TCustomBufDataset);
>>>>>>> origin/cpstrnew
begin
  Result := (FFirstRecBuf<>nil);
end;

function TDoubleLinkedBufIndex.GetSpareBuffer: TRecordBuffer;
begin
  Result := pointer(FLastRecBuf) + FDataset.BufferOffset;
end;

function TDoubleLinkedBufIndex.GetSpareRecord: TRecordBuffer;
begin
  Result := TRecordBuffer(FLastRecBuf);
=======
  if assigned(FParser) then FreeAndNil(FParser);
end;

procedure TBufDataset.InternalFirst;
begin
  with FCurrentIndex do
    begin
// if FCurrentRecBuf = FLastRecBuf then the dataset is just opened and empty
// in which case InternalFirst should do nothing (bug 7211)
    SetToFirstRecord;
    end;
end;

procedure TBufDataset.InternalLast;
begin
  FetchAll;
  with FCurrentIndex do
  SetToLastRecord;
end;

function TDoubleLinkedBufIndex.GetCurrentRecord: PChar;
begin
  Result := pchar(FCurrentRecBuf);
end;

function TDoubleLinkedBufIndex.GetBookmarkSize: integer;
begin
  Result:=sizeof(TBufBookmark);
end;

function TDoubleLinkedBufIndex.GetCurrentBuffer: Pointer;
begin
  Result := pointer(FCurrentRecBuf)+(sizeof(TBufRecLinkItem)*FDataset.MaxIndexesCount);
end;

function TDoubleLinkedBufIndex.GetIsInitialized: boolean;
begin
  Result := (FFirstRecBuf<>nil);
end;

function TDoubleLinkedBufIndex.GetSpareBuffer: PChar;
begin
  Result := pointer(FLastRecBuf)+(sizeof(TBufRecLinkItem)*FDataset.MaxIndexesCount);
end;

function TDoubleLinkedBufIndex.GetSpareRecord: PChar;
begin
  Result := pchar(FLastRecBuf);
end;

constructor TBufIndex.Create(const ADataset: TBufDataset);
begin
  inherited create;
  FDataset := ADataset;
end;

function TBufIndex.BookmarkValid(const ABookmark: PBufBookmark): boolean;
begin
  Result := assigned(ABookmark) and assigned(ABookmark^.BookmarkData);
end;

function TBufIndex.CompareBookmarks(const ABookmark1, ABookmark2: PBufBookmark): boolean;
begin
  result := (ABookmark1^.BookmarkData=ABookmark2^.BookmarkData);
>>>>>>> graemeg/fixes_2_2
end;

function TDoubleLinkedBufIndex.ScrollBackward: TGetResult;
begin
  if not assigned(FCurrentRecBuf[IndNr].prior) then
    begin
    Result := grBOF;
    end
  else
    begin
    Result := grOK;
    FCurrentRecBuf := FCurrentRecBuf[IndNr].prior;
    end;
end;

function TDoubleLinkedBufIndex.ScrollForward: TGetResult;
begin
  if (FCurrentRecBuf = FLastRecBuf) or // just opened
     (FCurrentRecBuf[IndNr].next = FLastRecBuf) then
    result := grEOF
  else
    begin
    FCurrentRecBuf := FCurrentRecBuf[IndNr].next;
    Result := grOK;
    end;
end;

function TDoubleLinkedBufIndex.GetCurrent: TGetResult;
begin
  if FFirstRecBuf = FLastRecBuf then
    Result := grError
  else
    begin
    Result := grOK;
    if FCurrentRecBuf = FLastRecBuf then
      FCurrentRecBuf:=FLastRecBuf[IndNr].prior;
    end;
end;

function TDoubleLinkedBufIndex.ScrollFirst: TGetResult;
begin
  FCurrentRecBuf:=FFirstRecBuf;
  if (FCurrentRecBuf = FLastRecBuf) then
    result := grEOF
  else
    result := grOK;
end;

procedure TDoubleLinkedBufIndex.ScrollLast;
begin
  FCurrentRecBuf:=FLastRecBuf;
end;

<<<<<<< HEAD
function TDoubleLinkedBufIndex.GetRecord(ABookmark: PBufBookmark; GetMode: TGetMode): TGetResult;
var ARecord : PBufRecLinkItem;
begin
  Result := grOK;
  case GetMode of
    gmPrior:
      begin
      if assigned(ABookmark^.BookmarkData) then
        ARecord := ABookmark^.BookmarkData[IndNr].prior
      else
        ARecord := nil;
      if not assigned(ARecord) then
        Result := grBOF;
      end;
    gmNext:
      begin
      if assigned(ABookmark^.BookmarkData) then
        ARecord := ABookmark^.BookmarkData[IndNr].next
      else
        ARecord := FFirstRecBuf;
      end;
    else
      Result := grError;
  end;

  if ARecord = FLastRecBuf then
    Result := grEOF;

  // store into BookmarkData pointer to prior/next record
  ABookmark^.BookmarkData:=ARecord;
end;

=======
>>>>>>> graemeg/fixes_2_2
procedure TDoubleLinkedBufIndex.SetToFirstRecord;
begin
  FLastRecBuf[IndNr].next:=FFirstRecBuf;
  FCurrentRecBuf := FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.SetToLastRecord;
begin
  if FLastRecBuf <> FFirstRecBuf then FCurrentRecBuf := FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.StoreCurrentRecord;
begin
  FStoredRecBuf:=FCurrentRecBuf;
end;

procedure TDoubleLinkedBufIndex.RestoreCurrentRecord;
begin
  FCurrentRecBuf:=FStoredRecBuf;
end;

procedure TDoubleLinkedBufIndex.DoScrollForward;
begin
  FCurrentRecBuf := FCurrentRecBuf[IndNr].next;
end;

procedure TDoubleLinkedBufIndex.StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark);
begin
  ABookmark^.BookmarkData:=FCurrentRecBuf;
end;

procedure TDoubleLinkedBufIndex.StoreSpareRecIntoBookmark(
  const ABookmark: PBufBookmark);
begin
  ABookmark^.BookmarkData:=FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.GotoBookmark(const ABookmark : PBufBookmark);
begin
  FCurrentRecBuf := ABookmark^.BookmarkData;
end;

<<<<<<< HEAD
function TDoubleLinkedBufIndex.CompareBookmarks(const ABookmark1,ABookmark2: PBufBookmark): integer;
var ARecord1, ARecord2 : PBufRecLinkItem;
begin
  // valid bookmarks expected
  // estimate result using memory addresses of records
  Result := ABookmark1^.BookmarkData - ABookmark2^.BookmarkData;
  if Result = 0 then
    Exit
  else if Result < 0 then
  begin
    Result   := -1;
    ARecord1 := ABookmark1^.BookmarkData;
    ARecord2 := ABookmark2^.BookmarkData;
  end
  else
  begin
    Result   := +1;
    ARecord1 := ABookmark2^.BookmarkData;
    ARecord2 := ABookmark1^.BookmarkData;
  end;
  // if we need relative position of records with given bookmarks we must
  // traverse through index until we reach lower bookmark or 1st record
  while assigned(ARecord2) and (ARecord2 <> ARecord1) and (ARecord2 <> FFirstRecBuf) do
    ARecord2 := ARecord2[IndNr].prior;
  // if we found lower bookmark as first, then estimated position is correct
  if ARecord1 <> ARecord2 then
    Result := -Result;
end;

procedure TDoubleLinkedBufIndex.InitialiseIndex;
begin
  // Do nothing
=======
procedure TDoubleLinkedBufIndex.InitialiseIndex;
begin
// Do nothing
>>>>>>> graemeg/fixes_2_2
end;

function TDoubleLinkedBufIndex.CanScrollForward: Boolean;
begin
  if (FCurrentRecBuf[IndNr].next = FLastRecBuf) then
    Result := False
  else
    Result := True;
end;

<<<<<<< HEAD
procedure TDoubleLinkedBufIndex.InitialiseSpareRecord(const ASpareRecord : TRecordBuffer);
begin
  FFirstRecBuf := pointer(ASpareRecord);
  FLastRecBuf := FFirstRecBuf;
  FLastRecBuf[IndNr].prior:=nil;
=======
procedure TDoubleLinkedBufIndex.InitialiseSpareRecord(const ASpareRecord : PChar);
begin
  FFirstRecBuf := pointer(ASpareRecord);
  FLastRecBuf := FFirstRecBuf;
>>>>>>> graemeg/fixes_2_2
  FLastRecBuf[IndNr].next:=FLastRecBuf;
  FCurrentRecBuf := FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.ReleaseSpareRecord;
begin
  FFirstRecBuf:= nil;
end;

<<<<<<< HEAD
function TDoubleLinkedBufIndex.GetRecNo: Longint;
var ARecord : PBufRecLinkItem;
begin
  ARecord := FCurrentRecBuf;
  Result := 1;
  while ARecord <> FFirstRecBuf do
    begin
    inc(Result);
    ARecord := ARecord[IndNr].prior;
    end;
end;

procedure TDoubleLinkedBufIndex.SetRecNo(ARecNo: Longint);
var ARecord : PBufRecLinkItem;
begin
  ARecord := FFirstRecBuf;
  while (ARecNo > 1) and (ARecord <> FLastRecBuf) do
    begin
<<<<<<< HEAD
    dec(ARecNo);
    ARecord := ARecord[IndNr].next;
=======
    inc(recnr);
    TmpRecBuffer := TmpRecBuffer[IndNr].next;
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
    end;
  FCurrentRecBuf := ARecord;
=======
procedure TDoubleLinkedBufIndex.RemoveRecordFromIndex(const ABookmark : TBufBookmark);
var ARecord : PBufRecLinkItem;
begin
  ARecord := ABookmark.BookmarkData;
  if ARecord = FCurrentRecBuf then DoScrollForward;
  if ARecord <> FFirstRecBuf then
    ARecord[IndNr].prior[IndNr].next := ARecord[IndNr].next
  else
    begin
    FFirstRecBuf := ARecord[IndNr].next;
    FLastRecBuf[IndNr].next := FFirstRecBuf;
    end;
  ARecord[IndNr].next[IndNr].prior := ARecord[IndNr].prior;
end;

function TDoubleLinkedBufIndex.GetRecNo(const ABookmark: PBufBookmark): integer;
Var TmpRecBuffer    : PBufRecLinkItem;
    recnr           : integer;
begin
  TmpRecBuffer := FFirstRecBuf;
  recnr := 1;
  while TmpRecBuffer <> ABookmark^.BookmarkData do
    begin
    inc(recnr);
    TmpRecBuffer := TmpRecBuffer^.next;
    end;
  Result := recnr;
>>>>>>> graemeg/fixes_2_2
end;

procedure TDoubleLinkedBufIndex.BeginUpdate;
begin
  if FCurrentRecBuf = FLastRecBuf then
    FCursOnFirstRec := True
  else
    FCursOnFirstRec := False;
end;

<<<<<<< HEAD
procedure TDoubleLinkedBufIndex.AddRecord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var ARecord: TRecordBuffer;
=======
var ARecord: PChar;
>>>>>>> graemeg/cpstrnew
=======
var ARecord: PChar;
>>>>>>> graemeg/cpstrnew
=======
var ARecord: PChar;
>>>>>>> graemeg/cpstrnew
=======
var ARecord: PChar;
>>>>>>> origin/cpstrnew
begin
  ARecord := FDataset.IntAllocRecordBuffer;
=======
procedure TDoubleLinkedBufIndex.AddRecord(Const ARecord : PChar);
begin
>>>>>>> graemeg/fixes_2_2
  FLastRecBuf[IndNr].next := pointer(ARecord);
  FLastRecBuf[IndNr].next[IndNr].prior := FLastRecBuf;

  FLastRecBuf := FLastRecBuf[IndNr].next;
end;

<<<<<<< HEAD
procedure TDoubleLinkedBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: TRecordBuffer);
=======
procedure TDoubleLinkedBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: PChar);
>>>>>>> graemeg/fixes_2_2
var ANewRecord : PBufRecLinkItem;
begin
  ANewRecord:=PBufRecLinkItem(ARecord);
  ANewRecord[IndNr].prior:=FCurrentRecBuf[IndNr].prior;
  ANewRecord[IndNr].Next:=FCurrentRecBuf;

  if FCurrentRecBuf=FFirstRecBuf then
    begin
    FFirstRecBuf:=ANewRecord;
    ANewRecord[IndNr].prior:=nil;
    end
  else
    ANewRecord[IndNr].Prior[IndNr].next:=ANewRecord;
  ANewRecord[IndNr].next[IndNr].prior:=ANewRecord;
end;

<<<<<<< HEAD
procedure TDoubleLinkedBufIndex.RemoveRecordFromIndex(const ABookmark : TBufBookmark);
var ARecord : PBufRecLinkItem;
begin
  ARecord := ABookmark.BookmarkData;
  if ARecord = FCurrentRecBuf then DoScrollForward;
  if ARecord <> FFirstRecBuf then
    ARecord[IndNr].prior[IndNr].next := ARecord[IndNr].next
  else
    begin
    FFirstRecBuf := ARecord[IndNr].next;
    FLastRecBuf[IndNr].next := FFirstRecBuf;
    end;
  ARecord[IndNr].next[IndNr].prior := ARecord[IndNr].prior;
end;

procedure TDoubleLinkedBufIndex.OrderCurrentRecord;
var ARecord: PBufRecLinkItem;
    ABookmark: TBufBookmark;
begin
  // all records except current are already sorted
  // check prior records
  ARecord := FCurrentRecBuf;
  repeat
    ARecord := ARecord[IndNr].prior;
  until not assigned(ARecord) or (IndexCompareRecords(ARecord, FCurrentRecBuf, DBCompareStruct) <= 0);
  if assigned(ARecord) then
    ARecord := ARecord[IndNr].next
  else
    ARecord := FFirstRecBuf;
  if ARecord = FCurrentRecBuf then
  begin
    // prior record is less equal than current
    // check next records
    repeat
      ARecord := ARecord[IndNr].next;
    until (ARecord=FLastRecBuf) or (IndexCompareRecords(ARecord, FCurrentRecBuf, DBCompareStruct) >= 0);
    if ARecord = FCurrentRecBuf[IndNr].next then
      Exit; // current record is on proper position
  end;
  StoreCurrentRecIntoBookmark(@ABookmark);
  RemoveRecordFromIndex(ABookmark);
  FCurrentRecBuf := ARecord;
  InsertRecordBeforeCurrentRecord(TRecordBuffer(ABookmark.BookmarkData));
  GotoBookmark(@ABookmark);
end;

=======
>>>>>>> graemeg/fixes_2_2
procedure TDoubleLinkedBufIndex.EndUpdate;
begin
  FLastRecBuf[IndNr].next := FFirstRecBuf;
  if FCursOnFirstRec then FCurrentRecBuf:=FLastRecBuf;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.CurrentRecordToBuffer(Buffer: TRecordBuffer);
=======
procedure TCustomBufDataset.CurrentRecordToBuffer(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.CurrentRecordToBuffer(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.CurrentRecordToBuffer(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.CurrentRecordToBuffer(Buffer: PChar);
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.CurrentRecordToBuffer(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
var ABookMark : PBufBookmark;
begin
  with FCurrentIndex do
    begin
    move(CurrentBuffer^,buffer^,FRecordSize);
    ABookMark:=PBufBookmark(Buffer + FRecordSize);
    ABookmark^.BookmarkFlag:=bfCurrent;
    StoreCurrentRecIntoBookmark(ABookMark);
    end;

  GetCalcFields(Buffer);
end;

<<<<<<< HEAD
procedure TCustomBufDataset.SetBufUniDirectional(const AValue: boolean);
begin
  CheckInactive;
  if (AValue<>IsUniDirectional) then
    begin
    SetUniDirectional(AValue);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ClearIndexes;
    FPacketRecords := 1; // temporary
    end;
end;
=======
    SetLength(FIndexes,0);
    FPacketRecords := 1; // temporary
    FIndexesCount:=0;
    end;
end;

procedure TCustomBufDataset.InitDefaultIndexes;
begin
  if FIndexesCount=0 then
    begin
    InternalAddIndex('DEFAULT_ORDER','',[],'','');
    FCurrentIndex:=FIndexes[0];
    if not IsUniDirectional then
      InternalAddIndex('','',[],'','');
    BookmarkSize := FCurrentIndex.BookmarkSize;
    end;
end;

function TCustomBufDataset.GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
>>>>>>> origin/cpstrnew

procedure TCustomBufDataset.SetReadOnly(AValue: Boolean);
begin
  FReadOnly:=AValue;
end;

function TCustomBufDataset.GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    SetLength(FIndexes,0);
    FPacketRecords := 1; // temporary
    FIndexesCount:=0;
    end;
end;

procedure TCustomBufDataset.InitDefaultIndexes;
begin
  if FIndexesCount=0 then
    begin
    InternalAddIndex('DEFAULT_ORDER','',[],'','');
    FCurrentIndex:=FIndexes[0];
    if not IsUniDirectional then
      InternalAddIndex('','',[],'','');
    BookmarkSize := FCurrentIndex.BookmarkSize;
    end;
end;

function TCustomBufDataset.GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

var Acceptable : Boolean;
    SavedState : TDataSetState;
=======
function TBufDataset.GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult;

var Acceptable : Boolean;
    SaveState : TDataSetState;
>>>>>>> graemeg/fixes_2_2

begin
  Result := grOK;
  with FCurrentIndex do
<<<<<<< HEAD
=======
    begin
>>>>>>> graemeg/fixes_2_2
    repeat
    Acceptable := True;
    case GetMode of
      gmPrior : Result := ScrollBackward;
      gmCurrent : Result := GetCurrent;
      gmNext : begin
<<<<<<< HEAD
               if not CanScrollForward and (getnextpacket = 0) then
                 Result := grEOF
               else
                 begin
                 Result := grOK;
=======
               if not CanScrollForward and (getnextpacket = 0) then result := grEOF
               else
                 begin
                 result := grOK;
>>>>>>> graemeg/fixes_2_2
                 DoScrollForward;
                 end;
               end;
    end;

    if Result = grOK then
      begin
<<<<<<< HEAD
      CurrentRecordToBuffer(Buffer);
=======
      CurrentRecordToBuffer(buffer);
>>>>>>> graemeg/fixes_2_2

      if Filtered then
        begin
        FFilterBuffer := Buffer;
<<<<<<< HEAD
        SavedState := SetTempState(dsFilter);
=======
        SaveState := SetTempState(dsFilter);
>>>>>>> graemeg/fixes_2_2
        DoFilterRecord(Acceptable);
        if (GetMode = gmCurrent) and not Acceptable then
          begin
          Acceptable := True;
          Result := grError;
          end;
<<<<<<< HEAD
        RestoreState(SavedState);
        end;
      end
    else if (Result = grError) and DoCheck then
      DatabaseError('No record');
    until Acceptable;
<<<<<<< HEAD
=======
  end;
end;

procedure TCustomBufDataset.DoBeforeClose;
=======
        RestoreState(SaveState);
        end;
      end
    else if (Result = grError) and doCheck then
      DatabaseError('No record');
    until Acceptable;
  end;
end;

procedure TBufDataset.DoBeforeClose;
>>>>>>> graemeg/fixes_2_2
begin
  inherited DoBeforeClose;
  if FFileName<>'' then
    SaveToFile(FFileName);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
end;

function TCustomBufDataset.GetActiveRecordUpdateBuffer : boolean;
=======
end;

function TBufDataset.GetActiveRecordUpdateBuffer : boolean;
>>>>>>> graemeg/fixes_2_2

var ABookmark : TBufBookmark;

begin
  GetBookmarkData(ActiveBuffer,@ABookmark);
  result := GetRecordUpdateBufferCached(ABookmark);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.ProcessFieldsToCompareStruct(const AFields, ADescFields, ACInsFields: TList;
      const AIndexOptions: TIndexOptions; const ALocateOptions: TLocateOptions; out ACompareStruct: TDBCompareStruct);
var i: integer;
    AField: TField;
    ACompareRec: TDBCompareRec;
begin
  SetLength(ACompareStruct, AFields.Count);
  for i:=0 to high(ACompareStruct) do
    begin
    AField := TField(AFields[i]);

    case AField.DataType of
      ftString, ftFixedChar, ftGuid:
        ACompareRec.CompareFunc := @DBCompareText;
      ftWideString, ftFixedWideChar:
        ACompareRec.CompareFunc := @DBCompareWideText;
      ftSmallint:
        ACompareRec.CompareFunc := @DBCompareSmallInt;
      ftInteger, ftAutoInc:
        ACompareRec.CompareFunc := @DBCompareInt;
      ftLargeint, ftBCD:
        ACompareRec.CompareFunc := @DBCompareLargeInt;
      ftWord:
        ACompareRec.CompareFunc := @DBCompareWord;
      ftBoolean:
        ACompareRec.CompareFunc := @DBCompareByte;
      ftDate, ftTime, ftDateTime,
      ftFloat, ftCurrency:
        ACompareRec.CompareFunc := @DBCompareDouble;
      ftFmtBCD:
        ACompareRec.CompareFunc := @DBCompareBCD;
      ftVarBytes:
        ACompareRec.CompareFunc := @DBCompareVarBytes;
      ftBytes:
        ACompareRec.CompareFunc := @DBCompareBytes;
    else
      DatabaseErrorFmt(SErrIndexBasedOnInvField, [AField.FieldName,Fieldtypenames[AField.DataType]]);
    end;

    ACompareRec.Off:=BufferOffset + FFieldBufPositions[AField.FieldNo-1];
    ACompareRec.NullBOff:=BufferOffset;

    ACompareRec.FieldInd:=AField.FieldNo-1;
    ACompareRec.Size:=GetFieldSize(FieldDefs[ACompareRec.FieldInd]);

    ACompareRec.Desc := ixDescending in AIndexOptions;
    if assigned(ADescFields) then
      ACompareRec.Desc := ACompareRec.Desc or (ADescFields.IndexOf(AField)>-1);

    ACompareRec.Options := ALocateOptions;
    if assigned(ACInsFields) and (ACInsFields.IndexOf(AField)>-1) then
      ACompareRec.Options := ACompareRec.Options + [loCaseInsensitive];

    ACompareStruct[i] := ACompareRec;
    end;
end;

procedure TCustomBufDataset.InitDefaultIndexes;
=======
=======
>>>>>>> origin/cpstrnew
procedure TCustomBufDataset.ProcessFieldCompareStruct(AField: TField; var ACompareRec : TDBCompareRec);
begin
  case AField.DataType of
    ftString, ftFixedChar : ACompareRec.Comparefunc := @DBCompareText;
    ftSmallint : ACompareRec.Comparefunc := @DBCompareSmallInt;
    ftInteger, ftBCD, ftAutoInc : ACompareRec.Comparefunc :=
      @DBCompareInt;
    ftWord : ACompareRec.Comparefunc := @DBCompareWord;
    ftBoolean : ACompareRec.Comparefunc := @DBCompareByte;
    ftFloat, ftCurrency : ACompareRec.Comparefunc := @DBCompareDouble;
    ftDateTime, ftDate, ftTime : ACompareRec.Comparefunc :=
      @DBCompareDouble;
    ftLargeint : ACompareRec.Comparefunc := @DBCompareLargeInt;
    ftFmtBCD : ACompareRec.Comparefunc := @DBCompareBCD;
  else
    DatabaseErrorFmt(SErrIndexBasedOnInvField, [AField.FieldName,Fieldtypenames[AField.DataType]]);
  end;

  ACompareRec.Off1:=sizeof(TBufRecLinkItem)*FMaxIndexesCount+
    FFieldBufPositions[AField.FieldNo-1];
  ACompareRec.Off2:=ACompareRec.Off1;

  ACompareRec.FieldInd1:=AField.FieldNo-1;
  ACompareRec.FieldInd2:=ACompareRec.FieldInd1;

  ACompareRec.NullBOff1:=sizeof(TBufRecLinkItem)*MaxIndexesCount;
  ACompareRec.NullBOff2:=ACompareRec.NullBOff1;
end;

procedure TCustomBufDataset.SetIndexFieldNames(const AValue: String);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  if FIndexesCount=0 then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    InternalAddIndex('DEFAULT_ORDER','',[],'','');
    FCurrentIndex:=FIndexes[0];
    if not IsUniDirectional then
      InternalAddIndex('','',[],'','');
    BookmarkSize := FCurrentIndex.BookmarkSize;
    end;
end;

procedure TCustomBufDataset.AddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string = '';
  const ACaseInsFields: string = '');
=======
=======
>>>>>>> origin/cpstrnew
    if FIndexesCount=0 then
      InitDefaultIndexes;
    FIndexes[1].FieldsName:=AValue;
    FCurrentIndex:=FIndexes[1];
    if active then
      begin
      FetchAll;
      BuildIndex(FIndexes[1]);
      Resync([rmCenter]);
      end;
    end
  else
    SetIndexName('');
end;

procedure TCustomBufDataset.SetIndexName(AValue: String);
<<<<<<< HEAD
var i : integer;
>>>>>>> graemeg/cpstrnew
begin
  CheckBiDirectional;
  if AFields='' then DatabaseError(SNoIndexFieldNameGiven);

<<<<<<< HEAD
  if FIndexesCount=0 then
    InitDefaultIndexes;

  if Active and (FIndexesCount=FMaxIndexesCount) then
    DatabaseError(SMaxIndexes);

  // If not all packets are fetched, you can not sort properly.
  if not Active then
    FPacketRecords:=-1;
  InternalAddIndex(AName,AFields,AOptions,ADescFields,ACaseInsFields);
end;

procedure TCustomBufDataset.InternalAddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string;
                                       const ACaseInsFields: string);
var StoreIndNr : Integer;
begin
  if Active then FetchAll;
  if FIndexesCount>0 then
    StoreIndNr:=FCurrentIndex.IndNr
  else
    StoreIndNr:=0;
  inc(FIndexesCount);
  setlength(FIndexes,FIndexesCount); // This invalidates the currentindex! -> not anymore
  FCurrentIndex:=FIndexes[StoreIndNr];

  if IsUniDirectional then
    FIndexes[FIndexesCount-1] := TUniDirectionalBufIndex.Create(self)
  else
    FIndexes[FIndexesCount-1] := TDoubleLinkedBufIndex.Create(self);
//  FIndexes[FIndexesCount-1] := TArrayBufIndex.Create(self);
  with FIndexes[FIndexesCount-1] do
    begin
    InitialiseIndex;
    IndNr:=FIndexesCount-1;
    Name:=AName;
    FieldsName:=AFields;
    DescFields:=ADescFields;
    CaseinsFields:=ACaseInsFields;
    Options:=AOptions;
    end;

  if Active then
    begin
    FIndexes[FIndexesCount-1].InitialiseSpareRecord(IntAllocRecordBuffer);
    BuildIndex(FIndexes[FIndexesCount-1]);
    end
  else if FIndexesCount>FMaxIndexesCount then
    FMaxIndexesCount := FIndexesCount;

  FIndexDefs.Updated:=false;
=======
=======
>>>>>>> graemeg/cpstrnew
procedure TCustomBufDataset.ProcessFieldCompareStruct(AField: TField; var ACompareRec : TDBCompareRec);
begin
  case AField.DataType of
    ftString, ftFixedChar : ACompareRec.Comparefunc := @DBCompareText;
    ftSmallint : ACompareRec.Comparefunc := @DBCompareSmallInt;
    ftInteger, ftBCD, ftAutoInc : ACompareRec.Comparefunc :=
=======
procedure TBufDataset.ProcessFieldCompareStruct(AField: TField; var ACompareRec : TDBCompareRec);
begin
  case AField.DataType of
    ftString : ACompareRec.Comparefunc := @DBCompareText;
    ftSmallint : ACompareRec.Comparefunc := @DBCompareSmallInt;
    ftInteger, ftBCD : ACompareRec.Comparefunc :=
>>>>>>> graemeg/fixes_2_2
      @DBCompareInt;
    ftWord : ACompareRec.Comparefunc := @DBCompareWord;
    ftBoolean : ACompareRec.Comparefunc := @DBCompareByte;
    ftFloat, ftCurrency : ACompareRec.Comparefunc := @DBCompareDouble;
    ftDateTime, ftDate, ftTime : ACompareRec.Comparefunc :=
      @DBCompareDouble;
    ftLargeint : ACompareRec.Comparefunc := @DBCompareLargeInt;
<<<<<<< HEAD
    ftFmtBCD : ACompareRec.Comparefunc := @DBCompareBCD;
  else
    DatabaseErrorFmt(SErrIndexBasedOnInvField, [AField.FieldName,Fieldtypenames[AField.DataType]]);
=======
  else
    DatabaseErrorFmt(SErrIndexBasedOnInvField, [Fieldtypenames[AField.DataType]]);
>>>>>>> graemeg/fixes_2_2
  end;

  ACompareRec.Off1:=sizeof(TBufRecLinkItem)*FMaxIndexesCount+
    FFieldBufPositions[AField.FieldNo-1];
  ACompareRec.Off2:=ACompareRec.Off1;
<<<<<<< HEAD

  ACompareRec.FieldInd1:=AField.FieldNo-1;
  ACompareRec.FieldInd2:=ACompareRec.FieldInd1;

  ACompareRec.NullBOff1:=sizeof(TBufRecLinkItem)*MaxIndexesCount;
  ACompareRec.NullBOff2:=ACompareRec.NullBOff1;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;

procedure TCustomBufDataset.SetIndexFieldNames(const AValue: String);
begin
  if AValue<>'' then
    begin
    if FIndexesCount=0 then
      InitDefaultIndexes;
    FIndexes[1].FieldsName:=AValue;
    FCurrentIndex:=FIndexes[1];
    if Active then
=======
end;

procedure TBufDataset.SetIndexFieldNames(const AValue: String);
begin
  if AValue<>'' then
    begin
    FIndexes[1].FieldsName:=AValue;
    FCurrentIndex:=FIndexes[1];
    if active then
>>>>>>> graemeg/fixes_2_2
      begin
      FetchAll;
      BuildIndex(FIndexes[1]);
      Resync([rmCenter]);
      end;
<<<<<<< HEAD
    FPacketRecords:=-1;
    FIndexDefs.Updated:=false;
=======
>>>>>>> graemeg/fixes_2_2
    end
  else
    SetIndexName('');
end;

<<<<<<< HEAD
procedure TCustomBufDataset.SetIndexName(AValue: String);
=======
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.SetIndexName(AValue: String);
>>>>>>> graemeg/fixes_2_2
var i : integer;
begin
  if AValue='' then AValue := 'DEFAULT_ORDER';
  for i := 0 to FIndexesCount-1 do
    if SameText(FIndexes[i].Name,AValue) then
      begin
      (FIndexes[i] as TDoubleLinkedBufIndex).FCurrentRecBuf:=(FCurrentIndex as TDoubleLinkedBufIndex).FCurrentRecBuf;
      FCurrentIndex:=FIndexes[i];
<<<<<<< HEAD
      if Active then Resync([rmCenter]);
=======
      if active then Resync([rmCenter]);
>>>>>>> graemeg/fixes_2_2
      exit;
      end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomBufDataset.SetMaxIndexesCount(const AValue: Integer);
=======
procedure TBufDataset.SetMaxIndexesCount(const AValue: Integer);
>>>>>>> graemeg/fixes_2_2
begin
  CheckInactive;
  if AValue > 1 then
    FMaxIndexesCount:=AValue
  else
    DatabaseError(SMinIndexes);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.InternalSetToRecord(Buffer: TRecordBuffer);
=======
procedure TCustomBufDataset.InternalSetToRecord(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.InternalSetToRecord(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.InternalSetToRecord(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.InternalSetToRecord(Buffer: PChar);
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.InternalSetToRecord(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2
begin
  FCurrentIndex.GotoBookmark(PBufBookmark(Buffer+FRecordSize));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
=======
procedure TCustomBufDataset.SetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.SetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.SetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.SetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> origin/cpstrnew
begin
  PBufBookmark(Buffer + FRecordSize)^ := PBufBookmark(Data)^;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag);
=======
procedure TCustomBufDataset.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.SetBookmarkData(Buffer: PChar; Data: Pointer);
begin
  PBufBookmark(Buffer + FRecordSize)^.BookmarkData := pointer(Data^);
end;

procedure TBufDataset.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
>>>>>>> graemeg/fixes_2_2
begin
  PBufBookmark(Buffer + FRecordSize)^.BookmarkFlag := Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer);
=======
procedure TCustomBufDataset.GetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.GetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.GetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.GetBookmarkData(Buffer: PChar; Data: Pointer);
>>>>>>> origin/cpstrnew
begin
  PBufBookmark(Data)^ := PBufBookmark(Buffer + FRecordSize)^;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag;
=======
function TCustomBufDataset.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.GetBookmarkData(Buffer: PChar; Data: Pointer);
begin
  pointer(Data^) := PBufBookmark(Buffer + FRecordSize)^.BookmarkData;
end;

function TBufDataset.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
>>>>>>> graemeg/fixes_2_2
begin
  Result := PBufBookmark(Buffer + FRecordSize)^.BookmarkFlag;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.InternalGotoBookmark(ABookmark: Pointer);
=======
procedure TBufDataset.InternalGotoBookmark(ABookmark: Pointer);
>>>>>>> graemeg/fixes_2_2
begin
  // note that ABookMark should be a PBufBookmark. But this way it can also be
  // a pointer to a TBufRecLinkItem
  FCurrentIndex.GotoBookmark(ABookmark);
end;

<<<<<<< HEAD
function TCustomBufDataset.getnextpacket : integer;

var i : integer;
    pb : TRecordBuffer;
=======
function TBufDataset.getnextpacket : integer;

var i : integer;
    pb : pchar;
>>>>>>> graemeg/fixes_2_2

begin
  if FAllPacketsFetched then
    begin
    result := 0;
    exit;
    end;

  FCurrentIndex.BeginUpdate;

  i := 0;
  pb := FIndexes[0].SpareBuffer;
<<<<<<< HEAD
  while ((i < FPacketRecords) or (FPacketRecords = -1)) and (LoadBuffer(pb) = grOk) do
    begin
    with FIndexes[0] do
      begin
      AddRecord;
=======
  while ((i < FPacketRecords) or (FPacketRecords = -1)) and (loadbuffer(pb) = grOk) do
    begin
    with FIndexes[0] do
      begin
      AddRecord(IntAllocRecordBuffer);
>>>>>>> graemeg/fixes_2_2
      pb := SpareBuffer;
      end;
    inc(i);
    end;

  FCurrentIndex.EndUpdate;
  FBRecordCount := FBRecordCount + i;
  result := i;
end;

<<<<<<< HEAD
function TCustomBufDataset.GetFieldSize(FieldDef : TFieldDef) : longint;

begin
  case FieldDef.DataType of
    ftUnknown    : result := 0;
=======
function TBufDataset.GetFieldSize(FieldDef : TFieldDef) : longint;

begin
  case FieldDef.DataType of
>>>>>>> graemeg/fixes_2_2
    ftString,
      ftGuid,
      ftFixedChar: result := FieldDef.Size + 1;
    ftFixedWideChar,
      ftWideString:result := (FieldDef.Size + 1)*2;
    ftSmallint,
      ftInteger,
<<<<<<< HEAD
      ftAutoInc,
      ftword     : result := sizeof(longint);
    ftBoolean    : result := sizeof(wordbool);
    ftBCD        : result := sizeof(currency);
    ftFmtBCD     : result := sizeof(TBCD);
=======
      ftword     : result := sizeof(longint);
    ftBoolean    : result := sizeof(wordbool);
    ftBCD        : result := sizeof(currency);
>>>>>>> graemeg/fixes_2_2
    ftFloat,
      ftCurrency : result := sizeof(double);
    ftLargeInt   : result := sizeof(largeint);
    ftTime,
      ftDate,
      ftDateTime : result := sizeof(TDateTime);
<<<<<<< HEAD
    ftBytes      : result := FieldDef.Size;
    ftVarBytes   : result := FieldDef.Size + 2;
    ftVariant    : result := sizeof(variant);
=======
>>>>>>> graemeg/fixes_2_2
    ftBlob,
      ftMemo,
      ftGraphic,
      ftFmtMemo,
      ftParadoxOle,
      ftDBaseOle,
      ftTypedBinary,
      ftOraBlob,
      ftOraClob,
      ftWideMemo : result := sizeof(TBufBlobField)
<<<<<<< HEAD
  else
    DatabaseErrorFmt(SUnsupportedFieldType,[Fieldtypenames[FieldDef.DataType]]);
=======
  else Result := 10
>>>>>>> graemeg/fixes_2_2
  end;
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
  result:=Align(result,4);
{$ENDIF}
end;

<<<<<<< HEAD
function TCustomBufDataset.GetRecordUpdateBuffer(const ABookmark : TBufBookmark; IncludePrior : boolean = false; AFindNext : boolean = false): boolean;
=======
function TBufDataset.GetRecordUpdateBuffer(const ABookmark : TBufBookmark; IncludePrior : boolean = false; AFindNext : boolean = false): boolean;
>>>>>>> graemeg/fixes_2_2

var x        : integer;
    StartBuf : integer;

begin
  if AFindNext then
<<<<<<< HEAD
    StartBuf := FCurrentUpdateBuffer + 1
=======
    StartBuf:=FCurrentUpdateBuffer+1
>>>>>>> graemeg/fixes_2_2
  else
    StartBuf := 0;
  Result := False;
  for x := StartBuf to high(FUpdateBuffer) do
<<<<<<< HEAD
   if FCurrentIndex.SameBookmarks(@FUpdateBuffer[x].BookmarkData,@ABookmark) or
      (IncludePrior and (FUpdateBuffer[x].UpdateKind=ukDelete) and FCurrentIndex.SameBookmarks(@FUpdateBuffer[x].NextBookmarkData,@ABookmark)) then
=======
   if FCurrentIndex.CompareBookmarks(@FUpdateBuffer[x].BookmarkData,@ABookmark) or
      (IncludePrior and (FUpdateBuffer[x].UpdateKind=ukDelete) and FCurrentIndex.CompareBookmarks(@FUpdateBuffer[x].NextBookmarkData,@ABookmark)) then
>>>>>>> graemeg/fixes_2_2
    begin
    FCurrentUpdateBuffer := x;
    Result := True;
    break;
    end;
end;

<<<<<<< HEAD
function TCustomBufDataset.GetRecordUpdateBufferCached(const ABookmark: TBufBookmark;
  IncludePrior: boolean): boolean;
begin
  // if the current update buffer matches, immediately return true
  if (FCurrentUpdateBuffer < length(FUpdateBuffer)) and (
      FCurrentIndex.SameBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData,@ABookmark) or
      (IncludePrior
        and (FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind=ukDelete)
        and  FCurrentIndex.SameBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData,@ABookmark))) then
     begin
     Result := True;
     end
=======
function TBufDataset.GetRecordUpdateBufferCached(const ABookmark: TBufBookmark;
  IncludePrior: boolean): boolean;
begin
  // if the current update buffer complies, immediately return true
  if (FCurrentUpdateBuffer < length(FUpdateBuffer)) and (
      FCurrentIndex.CompareBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData,@ABookmark) or
      (IncludePrior and (FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind=ukDelete) and FCurrentIndex.CompareBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData,@ABookmark))) then
    Result := True
>>>>>>> graemeg/fixes_2_2
  else
    Result := GetRecordUpdateBuffer(ABookmark,IncludePrior);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.LoadBuffer(Buffer : TRecordBuffer): TGetResult;
=======
function TCustomBufDataset.LoadBuffer(Buffer : PChar): TGetResult;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.LoadBuffer(Buffer : PChar): TGetResult;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.LoadBuffer(Buffer : PChar): TGetResult;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.LoadBuffer(Buffer : PChar): TGetResult;
>>>>>>> origin/cpstrnew

var NullMask        : pbyte;
    x               : longint;
    CreateBlobField : boolean;
=======
function TBufDataset.LoadBuffer(Buffer : PChar): TGetResult;

var NullMask        : pbyte;
    x               : longint;
    CreateblobField : boolean;
>>>>>>> graemeg/fixes_2_2
    BufBlob         : PBufBlobField;

begin
  if not Fetch then
    begin
    Result := grEOF;
    FAllPacketsFetched := True;
    // This code has to be placed elsewhere. At least it should also run when
<<<<<<< HEAD
    // the datapacket is loaded from file ... see IntLoadRecordsFromFile
    BuildIndexes;
=======
    // the datapacket is loaded from file
    if FIndexesCount>0 then for x := 1 to FIndexesCount-1 do
      begin
      if not ((x=1) and (FIndexes[1].FieldsName='')) then
        begin
        BuildIndex(FIndexes[x]);
        (FCurrentIndex as TDoubleLinkedBufIndex).FCurrentRecBuf:=(FCurrentIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
        end;
      end;
>>>>>>> graemeg/fixes_2_2
    Exit;
    end;

  NullMask := pointer(buffer);
  fillchar(Nullmask^,FNullmaskSize,0);
  inc(buffer,FNullmaskSize);

<<<<<<< HEAD
  for x := 0 to FieldDefs.Count-1 do
    begin
    if not LoadField(FieldDefs[x],buffer,CreateBlobField) then
      SetFieldIsNull(NullMask,x)
    else if CreateBlobField then
=======
  for x := 0 to FieldDefs.count-1 do
    begin
    if not LoadField(FieldDefs[x],buffer,CreateblobField) then
      SetFieldIsNull(NullMask,x)
    else if CreateblobField then
>>>>>>> graemeg/fixes_2_2
      begin
      BufBlob := PBufBlobField(Buffer);
      BufBlob^.BlobBuffer := GetNewBlobBuffer;
      LoadBlobIntoBuffer(FieldDefs[x],BufBlob);
      end;
    inc(buffer,GetFieldSize(FieldDefs[x]));
    end;
  Result := grOK;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.GetCurrentBuffer: TRecordBuffer;
=======
function TCustomBufDataset.GetCurrentBuffer: PChar;
>>>>>>> graemeg/cpstrnew
=======
function TCustomBufDataset.GetCurrentBuffer: PChar;
>>>>>>> graemeg/cpstrnew
begin
  case State of
    dsFilter:        Result := FFilterBuffer;
    dsCalcFields:    Result := CalcBuffer;
    dsRefreshFields: Result := FCurrentIndex.CurrentBuffer
    else             Result := ActiveBuffer;
  end;
=======
=======
>>>>>>> origin/cpstrnew
function TCustomBufDataset.GetCurrentBuffer: PChar;
=======
function TBufDataset.GetCurrentBuffer: PChar;
>>>>>>> graemeg/fixes_2_2
begin
  if State = dsFilter then Result := FFilterBuffer
  else if state = dsCalcFields then Result := CalcBuffer
  else Result := ActiveBuffer;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
end;


function TCustomBufDataset.GetFieldData(Field: TField; Buffer: Pointer;
=======
end;


function TBufDataset.GetFieldData(Field: TField; Buffer: Pointer;
>>>>>>> graemeg/fixes_2_2
  NativeFormat: Boolean): Boolean;
begin
  Result := GetFieldData(Field, Buffer);
end;

<<<<<<< HEAD
function TCustomBufDataset.GetFieldData(Field: TField; Buffer: Pointer): Boolean;

var
  CurrBuff : TRecordBuffer;

begin
  Result := False;
  if State = dsOldValue then
  begin
    if FSavedState = dsInsert then
      CurrBuff := nil // old values = null
    else if GetActiveRecordUpdateBuffer then
      CurrBuff := FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer
    else
      // There is no UpdateBuffer for ActiveRecord, so there are no explicit old values available
      // then we can assume, that old values = current values
      CurrBuff := FCurrentIndex.CurrentBuffer;
  end
  else
    CurrBuff := GetCurrentBuffer;

  if not assigned(CurrBuff) then Exit; //Null value

  If Field.FieldNo > 0 then // If =-1, then calculated/lookup field or =0 unbound field
    begin
    if GetFieldIsNull(pbyte(CurrBuff),Field.FieldNo-1) then
      Exit;
    if assigned(Buffer) then
=======
function TBufDataset.GetFieldData(Field: TField; Buffer: Pointer): Boolean;

var CurrBuff : pchar;

begin
  Result := False;
  if state = dsOldValue then
    begin
    if not GetActiveRecordUpdateBuffer then
      begin
      // There is no old value available
      result := false;
      exit;
      end;
    currbuff := FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer;
    end
  else
    begin
    CurrBuff := GetCurrentBuffer;
    if not assigned(CurrBuff) then
      begin
      result := false;
      exit;
      end;
    end;

  If Field.Fieldno > 0 then // If = 0, then calculated field or something similar
    begin
    if GetFieldIsnull(pbyte(CurrBuff),Field.Fieldno-1) then
      begin
      result := false;
      exit;
      end;
    if assigned(buffer) then
>>>>>>> graemeg/fixes_2_2
      begin
      inc(CurrBuff,FFieldBufPositions[Field.FieldNo-1]);
      Move(CurrBuff^, Buffer^, GetFieldSize(FieldDefs[Field.FieldNo-1]));
      end;
    Result := True;
    end
  else
    begin
    Inc(CurrBuff, GetRecordSize + Field.Offset);
    Result := Boolean(CurrBuff^);
<<<<<<< HEAD
    if Result and assigned(Buffer) then
      begin
      inc(CurrBuff);
      Move(CurrBuff^, Buffer^, Field.DataSize);
=======
    if result and assigned(Buffer) then
      begin
      inc(CurrBuff);
      Move(CurrBuff^, Buffer^, Field.Datasize);
>>>>>>> graemeg/fixes_2_2
=======
    property MaxIndexesCount : Integer read FMaxIndexesCount write SetMaxIndexesCount;
  published
    property FileName : string read FFileName write FFileName;
    property PacketRecords : Integer read FPacketRecords write SetPacketRecords default 10;
    property OnUpdateError: TResolverErrorEvent read FOnUpdateError write SetOnUpdateError;
    property IndexDefs : TIndexDefs read GetIndexDefs;
    property IndexName : String read GetIndexName write SetIndexName;
    property IndexFieldNames : String read GetIndexFieldNames write SetIndexFieldNames;
  end;

procedure RegisterDatapacketReader(ADatapacketReaderClass : TDatapacketReaderClass; AFormat : TDataPacketFormat);

implementation

uses variants, dbconst;

Type TDatapacketReaderRegistration = record
                                       ReaderClass : TDatapacketReaderClass;
                                       Format      : TDataPacketFormat;
                                     end;

var RegisteredDatapacketReaders : Array of TDatapacketReaderRegistration;

procedure RegisterDatapacketReader(ADatapacketReaderClass : TDatapacketReaderClass; AFormat : TDataPacketFormat);
begin
  setlength(RegisteredDatapacketReaders,length(RegisteredDatapacketReaders)+1);
  with RegisteredDatapacketReaders[length(RegisteredDatapacketReaders)-1] do
    begin
    Readerclass := ADatapacketReaderClass;
    Format      := AFormat;
    end;
end;

function GetRegisterDatapacketReader(AStream : TStream; AFormat : TDataPacketFormat; var ADataReaderClass : TDatapacketReaderRegistration) : boolean;
var i : integer;
begin
  Result := False;
  for i := 0 to length(RegisteredDatapacketReaders)-1 do if ((AFormat=dfAny) or (AFormat=RegisteredDatapacketReaders[i].Format)) then
    begin
    if (AStream=nil) or (RegisteredDatapacketReaders[i].ReaderClass.RecognizeStream(AStream)) then
      begin
      ADataReaderClass := RegisteredDatapacketReaders[i];
      Result := True;
      if (AStream <> nil) then AStream.Seek(0,soFromBeginning);
      break;
      end;
    AStream.Seek(0,soFromBeginning);
    end;
end;

function DBCompareText(subValue, aValue: pointer; options: TLocateOptions): LargeInt;

begin
  if loCaseInsensitive in options then
    Result := AnsiCompareText(pchar(subValue),pchar(aValue))
  else
    Result := AnsiCompareStr(pchar(subValue),pchar(aValue));
end;

function DBCompareByte(subValue, aValue: pointer; options: TLocateOptions): LargeInt;

begin
  Result := PByte(subValue)^-PByte(aValue)^;
end;

function DBCompareSmallInt(subValue, aValue: pointer; options: TLocateOptions): LargeInt;

begin
  Result := PSmallInt(subValue)^-PSmallInt(aValue)^;
end;

function DBCompareInt(subValue, aValue: pointer; options: TLocateOptions): LargeInt;

begin
  Result := PInteger(subValue)^-PInteger(aValue)^;
end;

function DBCompareLargeInt(subValue, aValue: pointer; options: TLocateOptions): LargeInt;

begin
  // A simple subtraction doesn't work, since it could be that the result
  // doesn't fit into a LargeInt
  if PLargeInt(subValue)^ < PLargeInt(aValue)^ then
    result := -1
  else if PLargeInt(subValue)^  > PLargeInt(aValue)^ then
    result := 1
  else
    result := 0;
end;

function DBCompareWord(subValue, aValue: pointer; options: TLocateOptions): LargeInt;

begin
  Result := PWord(subValue)^-PWord(aValue)^;
end;

function DBCompareQWord(subValue, aValue: pointer; options: TLocateOptions): LargeInt;

begin
  // A simple subtraction doesn't work, since it could be that the result
  // doesn't fit into a LargeInt
  if PQWord(subValue)^ < PQWord(aValue)^ then
    result := -1
  else if PQWord(subValue)^  > PQWord(aValue)^ then
    result := 1
  else
    result := 0;
end;

function DBCompareDouble(subValue, aValue: pointer; options: TLocateOptions): LargeInt;
begin
  // A simple subtraction doesn't work, since it could be that the result
  // doesn't fit into a LargeInt
  if PDouble(subValue)^ < PDouble(aValue)^ then
    result := -1
  else if PDouble(subValue)^  > PDouble(aValue)^ then
    result := 1
  else
    result := 0;
end;

function IndexCompareRecords(Rec1,Rec2 : pointer; ADBCompareRecs : TDBCompareStruct) : LargeInt;
var IndexFieldNr : Integer;
begin
  for IndexFieldNr:=0 to length(ADBCompareRecs)-1 do with ADBCompareRecs[IndexFieldNr] do
    begin
    Result := Comparefunc(Rec1+Off1,Rec2+Off2,Options);
    if Result <> 0 then
      begin
      if Desc then
        Result := -Result;
      break;
>>>>>>> origin/fixes_2_2
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.SetFieldData(Field: TField; Buffer: Pointer;
=======
procedure TBufDataset.SetFieldData(Field: TField; Buffer: Pointer;
>>>>>>> graemeg/fixes_2_2
  NativeFormat: Boolean);
begin
  SetFieldData(Field,Buffer);
end;

<<<<<<< HEAD
procedure TCustomBufDataset.SetFieldData(Field: TField; Buffer: Pointer);
=======
procedure TBufDataset.SetFieldData(Field: TField; Buffer: Pointer);
>>>>>>> graemeg/fixes_2_2

var CurrBuff : pointer;
    NullMask : pbyte;

begin
<<<<<<< HEAD
  if not (State in dsWriteModes) then
    DatabaseErrorFmt(SNotEditing, [Name], Self);
  CurrBuff := GetCurrentBuffer;
  If Field.FieldNo > 0 then // If =-1, then calculated/lookup field or =0 unbound field
    begin
    if Field.ReadOnly and not (State in [dsSetKey, dsFilter, dsRefreshFields]) then
      DatabaseErrorFmt(SReadOnlyField, [Field.DisplayName]);	
    if State in [dsEdit, dsInsert, dsNewValue] then
      Field.Validate(Buffer);	
=======
  if not (state in [dsEdit, dsInsert, dsFilter, dsCalcFields]) then
    begin
    DatabaseErrorFmt(SNotEditing,[Name],self);
    exit;
    end;
  CurrBuff := GetCurrentBuffer;
  If Field.Fieldno > 0 then // If = 0, then calculated field or something
    begin
>>>>>>> graemeg/fixes_2_2
    NullMask := CurrBuff;

    inc(CurrBuff,FFieldBufPositions[Field.FieldNo-1]);
    if assigned(buffer) then
      begin
      Move(Buffer^, CurrBuff^, GetFieldSize(FieldDefs[Field.FieldNo-1]));
      unSetFieldIsNull(NullMask,Field.FieldNo-1);
      end
    else
      SetFieldIsNull(NullMask,Field.FieldNo-1);
    end
  else
    begin
    Inc(CurrBuff, GetRecordSize + Field.Offset);
    Boolean(CurrBuff^) := Buffer <> nil;
    inc(CurrBuff);
    if assigned(Buffer) then
<<<<<<< HEAD
      Move(Buffer^, CurrBuff^, Field.DataSize);
    end;
  if not (State in [dsCalcFields, dsFilter, dsNewValue]) then
<<<<<<< HEAD
    DataEvent(deFieldChange, PtrInt(Field));
end;

procedure TCustomBufDataset.InternalDelete;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var RemRec : pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    DataEvent(deFieldChange, Ptrint(Field));
end;

procedure TCustomBufDataset.InternalDelete;
>>>>>>> origin/cpstrnew
var i         : Integer;
    StartInd  : Integer;
    RemRec    : pointer;
>>>>>>> graemeg/cpstrnew
=======
      Move(Buffer^, CurrBuff^, Field.Datasize);
    end;
  if not (State in [dsCalcFields, dsFilter, dsNewValue]) then
    DataEvent(deFieldChange, Ptrint(Field));
end;

procedure TBufDataset.InternalDelete;
var i         : Integer;
    StartInd  : Integer;
    RemRec    : pointer;
>>>>>>> graemeg/fixes_2_2
    RemRecBookmrk : TBufBookmark;
begin
  InternalSetToRecord(ActiveBuffer);
  // Remove the record from all active indexes
  FCurrentIndex.StoreCurrentRecIntoBookmark(@RemRecBookmrk);
  RemRec := FCurrentIndex.CurrentBuffer;
<<<<<<< HEAD
  RemoveRecordFromIndexes(RemRecBookmrk);
=======
  FIndexes[0].RemoveRecordFromIndex(RemRecBookmrk);
  if FCurrentIndex=FIndexes[1] then StartInd := 1 else StartInd := 2;
  for i := StartInd to FIndexesCount-1 do
    findexes[i].RemoveRecordFromIndex(RemRecBookmrk);
>>>>>>> graemeg/fixes_2_2

  if not GetActiveRecordUpdateBuffer then
    begin
    FCurrentUpdateBuffer := length(FUpdateBuffer);
    SetLength(FUpdateBuffer,FCurrentUpdateBuffer+1);
<<<<<<< HEAD
    FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := IntAllocRecordBuffer;
    move(RemRec^, FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer^,FRecordSize);
    end
  else
    begin
    if FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind <> ukModify then
      begin
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := nil;  //this 'disables' the updatebuffer
      // Do NOT release record buffer (pointed to by RemRecBookmrk.BookmarkData) here
      //  - When record is inserted and deleted (and memory released) and again inserted then the same memory block can be returned
      //    which leads to confusion, because we get the same BookmarkData for distinct records
      //  - In CancelUpdates when records are restored, it is expected that deleted records still exist in memory
      // There also could be record(s) in the update buffer that is linked to this record.
      end;
=======

    FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := IntAllocRecordBuffer;
    move(RemRec^, FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer^,FRecordSize);
    end
  else //with FIndexes[0] do
    begin
    if FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind <> ukModify then
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := nil;  //this 'disables' the updatebuffer
>>>>>>> graemeg/fixes_2_2
    end;
  FCurrentIndex.StoreCurrentRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData);
  FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData := RemRecBookmrk;
  FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind := ukDelete;
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
  dec(FBRecordCount);
end;


<<<<<<< HEAD
procedure TCustomBufDataset.ApplyRecUpdate(UpdateKind : TUpdateKind);
=======
procedure TBufDataset.ApplyRecUpdate(UpdateKind : TUpdateKind);
>>>>>>> graemeg/fixes_2_2

begin
  raise EDatabaseError.Create(SApplyRecNotSupported);
end;

<<<<<<< HEAD
procedure TCustomBufDataset.CancelUpdates;
var StoreRecBM     : TBufBookmark;
  procedure CancelUpdBuffer(var AUpdBuffer : TRecUpdateBuffer);
  var
    TmpBuf         : TRecordBuffer;
    StoreUpdBuf    : integer;
    Bm             : TBufBookmark;
  begin
    with AUpdBuffer do
      begin
      if Not assigned(BookmarkData.BookmarkData) then
        exit;// this is used to exclude buffers which are already handled
      Case UpdateKind of
      ukModify:
        begin
        FCurrentIndex.GotoBookmark(@BookmarkData);
        move(TRecordBuffer(OldValuesBuffer)^,TRecordBuffer(FCurrentIndex.CurrentBuffer)^,FRecordSize);
        FreeRecordBuffer(OldValuesBuffer);
        end;
      ukDelete:
        if (assigned(OldValuesBuffer)) then
          begin
          FCurrentIndex.GotoBookmark(@NextBookmarkData);
          FCurrentIndex.InsertRecordBeforeCurrentRecord(TRecordBuffer(BookmarkData.BookmarkData));
          FCurrentIndex.ScrollBackward;
          move(TRecordBuffer(OldValuesBuffer)^,TRecordBuffer(FCurrentIndex.CurrentBuffer)^,FRecordSize);

          {for x := length(FUpdateBuffer)-1 downto 0 do
            begin
            if (FUpdateBuffer[x].UpdateKind=ukDelete) and FCurrentIndex.SameBookmarks(@FUpdateBuffer[x].NextBookmarkData,@BookmarkData) then
              CancelUpdBuffer(FUpdateBuffer[x]);
            end;}
          FreeRecordBuffer(OldValuesBuffer);
          inc(FBRecordCount);
          end  ;
      ukInsert:
        begin
        // Process all update buffers linked to this record before this record is removed
=======
procedure TBufDataset.CancelUpdates;
var StoreRecBM     : TBufBookmark;
  procedure CancelUpdBuffer(var AUpdBuffer : TRecUpdateBuffer);
  var
    TmpBuf         : PChar;
    StoreUpdBuf    : integer;
    Bm             : TBufBookmark;
    x              : Integer;
  begin
    with AUpdBuffer do if assigned(BookmarkData.BookmarkData) then // this is used to exclude buffers which are already handled
      begin
      if (UpdateKind = ukModify) then
        begin
        FCurrentIndex.GotoBookmark(@BookmarkData);
        move(pchar(OldValuesBuffer)^,pchar(FCurrentIndex.CurrentBuffer)^,FRecordSize);
        FreeRecordBuffer(OldValuesBuffer);
        end
      else if (UpdateKind = ukDelete) and (assigned(OldValuesBuffer)) then
        begin
        FCurrentIndex.GotoBookmark(@NextBookmarkData);
        FCurrentIndex.InsertRecordBeforeCurrentRecord(PChar(BookmarkData.BookmarkData));
        FCurrentIndex.ScrollBackward;
        move(pchar(OldValuesBuffer)^,pchar(FCurrentIndex.CurrentBuffer)^,FRecordSize);

{        for x := length(FUpdateBuffer)-1 downto 0 do
          begin
          if (FUpdateBuffer[x].UpdateKind=ukDelete) and FCurrentIndex.CompareBookmarks(@FUpdateBuffer[x].NextBookmarkData,@BookmarkData) then
            CancelUpdBuffer(FUpdateBuffer[x]);
          end;}
        FreeRecordBuffer(OldValuesBuffer);
        inc(FBRecordCount);
        end
      else if (UpdateKind = ukInsert) then
        begin
        // Process all upd-buffers linked to this record before this record is removed
>>>>>>> graemeg/fixes_2_2
        StoreUpdBuf:=FCurrentUpdateBuffer;
        Bm := BookmarkData;
        BookmarkData.BookmarkData:=nil; // Avoid infinite recursion...
        if GetRecordUpdateBuffer(Bm,True,False) then
          begin
          repeat
<<<<<<< HEAD
            if (FCurrentUpdateBuffer<>StoreUpdBuf) then
              CancelUpdBuffer(FUpdateBuffer[FCurrentUpdateBuffer]);
=======
          if (FCurrentUpdateBuffer<>StoreUpdBuf) then CancelUpdBuffer(FUpdateBuffer[FCurrentUpdateBuffer]);
>>>>>>> graemeg/fixes_2_2
          until not GetRecordUpdateBuffer(Bm,True,True);
          end;
        FCurrentUpdateBuffer:=StoreUpdBuf;

        FCurrentIndex.GotoBookmark(@Bm);
        TmpBuf:=FCurrentIndex.CurrentRecord;
        // resync won't work if the currentbuffer is freed...
<<<<<<< HEAD
        if FCurrentIndex.SameBookmarks(@Bm,@StoreRecBM) then with FCurrentIndex do
          begin
          GotoBookmark(@StoreRecBM);
          if ScrollForward = grEOF then
            if ScrollBackward = grBOF then
              ScrollLast;  // last record will be removed from index, so move to spare record
          StoreCurrentRecIntoBookmark(@StoreRecBM);
          end;
        RemoveRecordFromIndexes(Bm);
        FreeRecordBuffer(TmpBuf);
        dec(FBRecordCount);
        end;
      end;
=======
        if FCurrentIndex.CompareBookmarks(@Bm,@StoreRecBM) then with FCurrentIndex do
          begin
          GotoBookmark(@StoreRecBM);
          if ScrollForward = grEOF then
            ScrollBackward;
          StoreCurrentRecIntoBookmark(@StoreRecBM);
          end;
        FCurrentIndex.RemoveRecordFromIndex(Bm);
        FreeRecordBuffer(TmpBuf);
        dec(FBRecordCount);
        end;
>>>>>>> graemeg/fixes_2_2
      BookmarkData.BookmarkData:=nil;
      end;
  end;

var r              : Integer;

begin
  CheckBrowseMode;

  if Length(FUpdateBuffer) > 0 then
    begin
    FCurrentIndex.StoreCurrentRecIntoBookmark(@StoreRecBM);
<<<<<<< HEAD
    for r := Length(FUpdateBuffer) - 1 downto 0 do
      CancelUpdBuffer(FUpdateBuffer[r]);
=======
    r := Length(FUpdateBuffer) -1;
    while r > -1 do
      begin
      CancelUpdBuffer(FUpdateBuffer[r]);
      dec(r)
      end;
>>>>>>> graemeg/fixes_2_2

    SetLength(FUpdateBuffer,0);
    
    FCurrentIndex.GotoBookmark(@StoreRecBM);
    
    Resync([]);
    end;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.SetOnUpdateError(const AValue: TResolverErrorEvent);
=======
procedure TBufDataset.SetOnUpdateError(const AValue: TResolverErrorEvent);
>>>>>>> graemeg/fixes_2_2

begin
  FOnUpdateError := AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.ApplyUpdates; // For backward compatibility
=======
procedure TCustomBufDataset.ApplyUpdates; // For backwards-compatibility
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.ApplyUpdates; // For backwards-compatibility
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.ApplyUpdates; // For backwards-compatibility
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.ApplyUpdates; // For backwards-compatibility
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.ApplyUpdates; // For backwards-compatibility
>>>>>>> graemeg/fixes_2_2

begin
  ApplyUpdates(0);
end;

<<<<<<< HEAD
procedure TCustomBufDataset.ApplyUpdates(MaxErrors: Integer);
=======
procedure TBufDataset.ApplyUpdates(MaxErrors: Integer);
>>>>>>> graemeg/fixes_2_2

var r            : Integer;
    FailedCount  : integer;
    Response     : TResolverResponse;
    StoreCurrRec : TBufBookmark;
<<<<<<< HEAD
    AUpdateError : EUpdateError;
=======
    AUpdateErr   : EUpdateError;
>>>>>>> graemeg/fixes_2_2

begin
  CheckBrowseMode;

  FCurrentIndex.StoreCurrentRecIntoBookmark(@StoreCurrRec);

  r := 0;
  FailedCount := 0;
  Response := rrApply;
  DisableControls;
  try
    while (r < Length(FUpdateBuffer)) and (Response <> rrAbort) do
      begin
      // If the record is first inserted and afterwards deleted, do nothing
      if not ((FUpdateBuffer[r].UpdateKind=ukDelete) and not (assigned(FUpdateBuffer[r].OldValuesBuffer))) then
        begin
        FCurrentIndex.GotoBookmark(@FUpdateBuffer[r].BookmarkData);
<<<<<<< HEAD
        // Synchronise the CurrentBuffer to the ActiveBuffer
=======
        // Synchronise the Currentbuffer to the ActiveBuffer
>>>>>>> graemeg/fixes_2_2
        CurrentRecordToBuffer(ActiveBuffer);
        Response := rrApply;
        try
          ApplyRecUpdate(FUpdateBuffer[r].UpdateKind);
        except
          on E: EDatabaseError do
            begin
            Inc(FailedCount);
<<<<<<< HEAD
            if FailedCount > word(MaxErrors) then
              Response := rrAbort
            else
              Response := rrSkip;
            if assigned(FOnUpdateError) then
              begin
              AUpdateError := PSGetUpdateException(Exception(AcquireExceptionObject), nil);
              FOnUpdateError(Self, Self, AUpdateError, FUpdateBuffer[r].UpdateKind, Response);
              AUpdateError.Free;
=======
            if failedcount > word(MaxErrors) then Response := rrAbort
            else Response := rrSkip;
            if assigned(FOnUpdateError) then
              begin
              AUpdateErr := EUpdateError.Create(SOnUpdateError,E.Message,0,0,Exception(AcquireExceptionObject));
              FOnUpdateError(Self,Self,AUpdateErr,FUpdateBuffer[r].UpdateKind,Response);
              AUpdateErr.Free;
>>>>>>> graemeg/fixes_2_2
              if Response in [rrApply, rrIgnore] then dec(FailedCount);
              if Response = rrApply then dec(r);
              end
            else if Response = rrAbort then
<<<<<<< HEAD
              begin
              AUpdateError := PSGetUpdateException(Exception(AcquireExceptionObject), nil);
              raise AUpdateError;
              end;
=======
              Raise EUpdateError.Create(SOnUpdateError,E.Message,0,0,Exception(AcquireExceptionObject));
>>>>>>> graemeg/fixes_2_2
            end
          else
            raise;
        end;
<<<<<<< HEAD
        if Response in [rrApply, rrIgnore] then
          begin
          FreeRecordBuffer(FUpdateBuffer[r].OldValuesBuffer);
          if FUpdateBuffer[r].UpdateKind = ukDelete then
            FreeRecordBuffer( TRecordBuffer(FUpdateBuffer[r].BookmarkData.BookmarkData));
=======
        if response in [rrApply, rrIgnore] then
          begin
          FreeRecordBuffer(FUpdateBuffer[r].OldValuesBuffer);
          if FUpdateBuffer[r].UpdateKind = ukDelete then
            FreeRecordBuffer(PChar(FUpdateBuffer[r].BookmarkData.BookmarkData));
>>>>>>> graemeg/fixes_2_2
          FUpdateBuffer[r].BookmarkData.BookmarkData := nil;
          end
        end;
      inc(r);
      end;
  finally
<<<<<<< HEAD
    if (FailedCount=0) and Not ManualMergeChangeLog then
      MergeChangeLog;
    InternalGotoBookmark(@StoreCurrRec);
    Resync([]);
    EnableControls;
  end;
end;

procedure TCustomBufDataset.MergeChangeLog;

var r            : Integer;

begin
  for r:=0 to length(FUpdateBuffer)-1 do
    if assigned(FUpdateBuffer[r].OldValuesBuffer) then
      FreeMem(FUpdateBuffer[r].OldValuesBuffer);
  SetLength(FUpdateBuffer,0);

  if assigned(FUpdateBlobBuffers) then for r:=0 to length(FUpdateBlobBuffers)-1 do
    if assigned(FUpdateBlobBuffers[r]) then
      begin
      // update blob buffer is already referenced from record buffer (see InternalPost)
      if FUpdateBlobBuffers[r]^.OrgBufID >= 0 then
        begin
        FreeBlobBuffer(FBlobBuffers[FUpdateBlobBuffers[r]^.OrgBufID]);
        FBlobBuffers[FUpdateBlobBuffers[r]^.OrgBufID] := FUpdateBlobBuffers[r];
        end
      else
        begin
        setlength(FBlobBuffers,length(FBlobBuffers)+1);
        FUpdateBlobBuffers[r]^.OrgBufID := high(FBlobBuffers);
        FBlobBuffers[high(FBlobBuffers)] := FUpdateBlobBuffers[r];
        end;
      end;
  SetLength(FUpdateBlobBuffers,0);
end;

<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomBufDataset.InternalCancel;
=======
    if failedcount = 0 then
      begin
      SetLength(FUpdateBuffer,0);

      if assigned(FUpdateBlobBuffers) then for r:=0 to length(FUpdateBlobBuffers)-1 do
       if assigned(FUpdateBlobBuffers[r]) then
        begin
        if FUpdateBlobBuffers[r]^.OrgBufID >= 0 then
          begin
          Freemem(FBlobBuffers[FUpdateBlobBuffers[r]^.OrgBufID]^.Buffer);
          Dispose(FBlobBuffers[FUpdateBlobBuffers[r]^.OrgBufID]);
          FBlobBuffers[FUpdateBlobBuffers[r]^.OrgBufID] :=FUpdateBlobBuffers[r];
          end
        else
          begin
          setlength(FBlobBuffers,length(FBlobBuffers)+1);
          FUpdateBlobBuffers[r]^.OrgBufID := high(FBlobBuffers);
          FBlobBuffers[high(FBlobBuffers)] := FUpdateBlobBuffers[r];
          
          end;
        end;
      SetLength(FUpdateBlobBuffers,0);
      end;

    InternalGotoBookmark(@StoreCurrRec);
    Resync([]);
    EnableControls;
  end;
end;


procedure TBufDataset.InternalCancel;
>>>>>>> graemeg/fixes_2_2

Var i            : integer;

begin
<<<<<<< HEAD
  if assigned(FUpdateBlobBuffers) then for i:=0 to high(FUpdateBlobBuffers) do
    if assigned(FUpdateBlobBuffers[i]) and (FUpdateBlobBuffers[i]^.FieldNo>0) then
      FreeBlobBuffer(FUpdateBlobBuffers[i]);
end;

procedure TCustomBufDataset.InternalPost;

Var ABuff        : TRecordBuffer;
    i            : integer;
    ABookmark    : PBufBookmark;

begin
  inherited InternalPost;

  if assigned(FUpdateBlobBuffers) then for i:=0 to high(FUpdateBlobBuffers) do
   if assigned(FUpdateBlobBuffers[i]) and (FUpdateBlobBuffers[i]^.FieldNo>0) then
    FUpdateBlobBuffers[i]^.FieldNo := -1;

  if State = dsInsert then
    begin
    if assigned(FAutoIncField) then
      begin
      FAutoIncField.AsInteger := FAutoIncValue;
      inc(FAutoIncValue);
      end;
    // The active buffer is the newly created TDataset record,
    // from which the bookmark is set to the record where the new record should be
    // inserted
    ABookmark := PBufBookmark(ActiveBuffer + FRecordSize);
    // Create the new record buffer
    ABuff := IntAllocRecordBuffer;

    // Add new record to all active indexes
    for i := 0 to FIndexesCount-1 do
      if (i<>1) or (FIndexes[i]=FCurrentIndex) then
      begin
        if ABookmark^.BookmarkFlag = bfEOF then
          // append (at end)
          FIndexes[i].ScrollLast
        else
          // insert (before current record)
          FIndexes[i].GotoBookmark(ABookmark);

        FIndexes[i].InsertRecordBeforeCurrentRecord(ABuff);
        // newly inserted record becomes current record
        FIndexes[i].ScrollBackward;
      end;

    // Link the newly created record buffer to the newly created TDataset record
    FCurrentIndex.StoreCurrentRecIntoBookmark(ABookmark);
    ABookmark^.BookmarkFlag := bfInserted;

=======
  if assigned(FUpdateBlobBuffers) then for i:=0 to length(FUpdateBlobBuffers)-1 do
   if assigned(FUpdateBlobBuffers[i]) and (FUpdateBlobBuffers[i]^.FieldNo>0) then
    begin
    Reallocmem(FUpdateBlobBuffers[i]^.Buffer,0);
    Dispose(FUpdateBlobBuffers[i]);
    FUpdateBlobBuffers[i] := nil;
    end;
end;

procedure TBufDataset.InternalPost;

Var CurrBuff     : PChar;
    i            : integer;
    blobbuf      : tbufblobfield;
    NullMask     : pbyte;

begin
  inherited InternalPost;
  if assigned(FUpdateBlobBuffers) then for i:=0 to length(FUpdateBlobBuffers)-1 do
   if assigned(FUpdateBlobBuffers[i]) and (FUpdateBlobBuffers[i]^.FieldNo>0) then
    begin
    blobbuf.BlobBuffer := FUpdateBlobBuffers[i];
    CurrBuff := ActiveBuffer;
    NullMask := pbyte(CurrBuff);

    inc(CurrBuff,FFieldBufPositions[FUpdateBlobBuffers[i]^.FieldNo-1]);
    Move(blobbuf, CurrBuff^, GetFieldSize(FieldDefs[FUpdateBlobBuffers[i]^.FieldNo-1]));
    unSetFieldIsNull(NullMask,FUpdateBlobBuffers[i]^.FieldNo-1);
    
    FUpdateBlobBuffers[i]^.FieldNo := -1;
    end;

  if state = dsInsert then
    begin
    if GetBookmarkFlag(ActiveBuffer) = bfEOF then
      FIndexes[0].ScrollLast
    else
      // The active buffer is the newly created TDataset record,
      // from which the bookmark is set to the record where the new record should be
      // inserted
      InternalSetToRecord(ActiveBuffer);

    with FIndexes[0] do
      begin
      // Create the new record buffer
      FCurrentIndex.InsertRecordBeforeCurrentRecord(IntAllocRecordBuffer);
      ScrollBackward;
      // Add the record to the other indexes
      for i := 1 to FIndexesCount-1 do if ((i>1) or (FIndexes[i]=FCurrentIndex)) then
        FIndexes[i].InsertRecordBeforeCurrentRecord(CurrentRecord);
      end;

    // Link the newly created record buffer to the newly created TDataset record
    with PBufBookmark(ActiveBuffer + FRecordSize)^ do
      begin
      FCurrentIndex.StoreCurrentRecIntoBookmark(@BookmarkData);
      BookmarkFlag := bfInserted;
      end;
      
>>>>>>> graemeg/fixes_2_2
    inc(FBRecordCount);
    end
  else
    InternalSetToRecord(ActiveBuffer);

<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
  // If there is no updatebuffer already, add one
  if not GetActiveRecordUpdateBuffer then
    begin
    // Add a new updatebuffer
    FCurrentUpdateBuffer := length(FUpdateBuffer);
    SetLength(FUpdateBuffer,FCurrentUpdateBuffer+1);

    // Store a bookmark of the current record into the updatebuffer's bookmark
    FCurrentIndex.StoreCurrentRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);

<<<<<<< HEAD
    if State = dsEdit then
      begin
      // Create an oldvalues buffer with the old values of the record
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := IntAllocRecordBuffer;
=======
    if state = dsEdit then
      begin
      // Create an oldvalues buffer with the old values of the record
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := intAllocRecordBuffer;
>>>>>>> graemeg/fixes_2_2
      with FCurrentIndex do
        // Move only the real data
        move(CurrentBuffer^,FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer^,FRecordSize);
      FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind := ukModify;
      end
    else
      begin
      FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind := ukInsert;
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := nil;
      end;
    end;

<<<<<<< HEAD
  Move(ActiveBuffer^, FCurrentIndex.CurrentBuffer^, FRecordSize);

  // new data are now in current record so reorder current record if needed
  for i := 1 to FIndexesCount-1 do
    if (i<>1) or (FIndexes[i]=FCurrentIndex) then
      FIndexes[i].OrderCurrentRecord;
end;

procedure TCustomBufDataset.CalcRecordSize;
=======
  move(ActiveBuffer^,FCurrentIndex.CurrentBuffer^,FRecordSize);
end;

procedure TBufDataset.CalcRecordSize;
>>>>>>> graemeg/fixes_2_2

var x : longint;

begin
<<<<<<< HEAD
  FNullmaskSize := (FieldDefs.Count+7) div 8;
=======
  FNullmaskSize := 1+((FieldDefs.count-1) div 8);
>>>>>>> graemeg/fixes_2_2
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
  FNullmaskSize:=Align(FNullmaskSize,4);
{$ENDIF}
  FRecordSize := FNullmaskSize;
  SetLength(FFieldBufPositions,FieldDefs.count);
  for x := 0 to FieldDefs.count-1 do
    begin
    FFieldBufPositions[x] := FRecordSize;
    inc(FRecordSize, GetFieldSize(FieldDefs[x]));
    end;
end;

<<<<<<< HEAD
function TCustomBufDataset.GetIndexFieldNames: String;
begin
  if (FIndexesCount=0) or (FCurrentIndex<>FIndexes[1]) then
=======
function TBufDataset.GetIndexFieldNames: String;
begin
  if FCurrentIndex<>FIndexes[1] then
>>>>>>> graemeg/fixes_2_2
    result := ''
  else
    result := FCurrentIndex.FieldsName;
end;

<<<<<<< HEAD
function TCustomBufDataset.GetIndexName: String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  if FIndexesCount>0 then
    result := FCurrentIndex.Name
  else
    result := '';
end;

function TCustomBufDataset.GetBufUniDirectional: boolean;
begin
  result := IsUniDirectional;
end;

function TCustomBufDataset.GetPacketReader(const Format: TDataPacketFormat; const AStream: TStream): TDataPacketReader;

var APacketReader: TDataPacketReader;
    APacketReaderReg: TDatapacketReaderRegistration;

begin
  if GetRegisterDatapacketReader(AStream, format, APacketReaderReg) then
    APacketReader := APacketReaderReg.ReaderClass.Create(Self, AStream)
  else if TFpcBinaryDatapacketReader.RecognizeStream(AStream) then
    begin
    AStream.Seek(0, soFromBeginning);
    APacketReader := TFpcBinaryDatapacketReader.Create(Self, AStream)
    end
  else
    DatabaseError(SStreamNotRecognised);
  Result:=APacketReader;
end;

=======
=======
>>>>>>> graemeg/cpstrnew
begin
  if FIndexesCount>0 then
    result := FCurrentIndex.Name
  else
    result := '';
end;

function TCustomBufDataset.GetBufUniDirectional: boolean;
begin
  result := IsUniDirectional;
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
begin
  if FIndexesCount>0 then
    result := FCurrentIndex.Name
  else
    result := '';
end;

function TCustomBufDataset.GetBufUniDirectional: boolean;
begin
  result := IsUniDirectional;
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCustomBufDataset.GetRecordSize : Word;

begin
  result := FRecordSize + BookmarkSize;
end;

function TCustomBufDataset.GetChangeCount: integer;
=======
function TBufDataset.GetIndexName: String;
begin
  result := FCurrentIndex.Name;
end;

function TBufDataset.GetRecordSize : Word;

begin
  result := FRecordSize + BookmarkSize;
end;

function TBufDataset.GetChangeCount: integer;
>>>>>>> graemeg/fixes_2_2

begin
  result := length(FUpdateBuffer);
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.InternalInitRecord(Buffer:  TRecordBuffer);
=======
procedure TCustomBufDataset.InternalInitRecord(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.InternalInitRecord(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.InternalInitRecord(Buffer: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.InternalInitRecord(Buffer: PChar);
>>>>>>> origin/cpstrnew
=======
procedure TBufDataset.InternalInitRecord(Buffer: PChar);
>>>>>>> graemeg/fixes_2_2

begin
  FillChar(Buffer^, FRecordSize, #0);

  fillchar(Buffer^,FNullmaskSize,255);
end;

<<<<<<< HEAD
procedure TCustomBufDataset.SetRecNo(Value: Longint);

var ABookmark : TBufBookmark;

begin
  CheckBrowseMode;
  if Value > RecordCount then
    repeat until (getnextpacket < FPacketRecords) or (Value <= RecordCount) or (FPacketRecords = -1);

  if (Value > RecordCount) or (Value < 1) then
    begin
    DatabaseError(SNoSuchRecord, Self);
    exit;
    end;
<<<<<<< HEAD

  FCurrentIndex.RecNo:=Value;
  FCurrentIndex.StoreCurrentRecIntoBookmark(@ABookmark);
  GotoBookmark(@ABookmark);
end;
=======
  TmpRecBuffer := (FCurrentIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  for recnr := 1 to value-1 do
    TmpRecBuffer := TmpRecBuffer[FCurrentIndex.IndNr].next;
  GotoBookmark(@TmpRecBuffer);
end;

function TCustomBufDataset.GetRecNo: Longint;
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

function TCustomBufDataset.GetRecNo: Longint;

begin
  if IsUniDirectional then
    Result := -1
  else if (FBRecordCount = 0) or (State = dsInsert) then
    Result := 0
  else
    begin
    InternalSetToRecord(ActiveBuffer);
    Result := FCurrentIndex.RecNo;
    end;
end;

function TCustomBufDataset.IsCursorOpen: Boolean;
=======
procedure TBufDataset.SetRecNo(Value: Longint);

var
    recnr        : integer;
    TmpRecBuffer : PBufRecLinkItem;

begin
  checkbrowsemode;
  if value > RecordCount then
    begin
    repeat until (getnextpacket < FPacketRecords) or (value <= RecordCount) or (FPacketRecords = -1);
    if value > RecordCount then
      begin
      DatabaseError(SNoSuchRecord,self);
      exit;
      end;
    end;
  TmpRecBuffer := (FCurrentIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  for recnr := 1 to value-1 do
    TmpRecBuffer := TmpRecBuffer^.next;
  GotoBookmark(@TmpRecBuffer);
end;

function TBufDataset.GetRecNo: Longint;

Var abuf            : PChar;

begin
  abuf := GetCurrentBuffer;
  // If abuf isn't assigned, the recordset probably isn't opened.
  if assigned(abuf) and (FBRecordCount>0) and (state <> dsInsert) then
    Result:=FCurrentIndex.GetRecNo(PBufBookmark(abuf+FRecordSize))
  else
    result := 0;
end;

function TBufDataset.IsCursorOpen: Boolean;
>>>>>>> graemeg/fixes_2_2

begin
  Result := FOpen;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.GetRecordCount: Longint;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Function TCustomBufDataset.GetRecordCount: Longint;

>>>>>>> graemeg/cpstrnew
begin
  if Active then
    Result := FBRecordCount
  else
    Result:=0;  
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.UpdateStatus: TUpdateStatus;
=======
Function TCustomBufDataset.UpdateStatus: TUpdateStatus;
>>>>>>> graemeg/cpstrnew
=======
Function TCustomBufDataset.UpdateStatus: TUpdateStatus;
>>>>>>> graemeg/cpstrnew
=======
Function TCustomBufDataset.UpdateStatus: TUpdateStatus;
>>>>>>> graemeg/cpstrnew
=======
Function TCustomBufDataset.UpdateStatus: TUpdateStatus;
>>>>>>> origin/cpstrnew
=======
Function TBufDataset.GetRecordCount: Longint;

begin
  Result := FBRecordCount;
end;

Function TBufDataSet.UpdateStatus: TUpdateStatus;
>>>>>>> graemeg/fixes_2_2

begin
  Result:=usUnmodified;
  if GetActiveRecordUpdateBuffer then
    case FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind of
      ukModify : Result := usModified;
      ukInsert : Result := usInserted;
      ukDelete : Result := usDeleted;
    end;
end;

<<<<<<< HEAD
function TCustomBufDataset.GetNewBlobBuffer : PBlobBuffer;
=======
function TbufDataset.GetNewBlobBuffer : PBlobBuffer;
>>>>>>> graemeg/fixes_2_2

var ABlobBuffer : PBlobBuffer;

begin
  setlength(FBlobBuffers,length(FBlobBuffers)+1);
  new(ABlobBuffer);
  fillbyte(ABlobBuffer^,sizeof(ABlobBuffer^),0);
<<<<<<< HEAD
  ABlobBuffer^.OrgBufID := high(FBlobBuffers);
=======
  ABlobBuffer^.OrgBufID := high(FUpdateBlobBuffers);
>>>>>>> graemeg/fixes_2_2
  FBlobBuffers[high(FBlobBuffers)] := ABlobBuffer;
  result := ABlobBuffer;
end;

<<<<<<< HEAD
function TCustomBufDataset.GetNewWriteBlobBuffer : PBlobBuffer;
=======
function TbufDataset.GetNewWriteBlobBuffer : PBlobBuffer;
>>>>>>> graemeg/fixes_2_2

var ABlobBuffer : PBlobBuffer;

begin
  setlength(FUpdateBlobBuffers,length(FUpdateBlobBuffers)+1);
  new(ABlobBuffer);
  fillbyte(ABlobBuffer^,sizeof(ABlobBuffer^),0);
  FUpdateBlobBuffers[high(FUpdateBlobBuffers)] := ABlobBuffer;
  result := ABlobBuffer;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.FreeBlobBuffer(var ABlobBuffer: PBlobBuffer);
=======
procedure TBufDataset.FreeBlobBuffer(var ABlobBuffer: PBlobBuffer);
>>>>>>> graemeg/fixes_2_2

begin
  if not Assigned(ABlobBuffer) then Exit;
  FreeMem(ABlobBuffer^.Buffer, ABlobBuffer^.Size);
  Dispose(ABlobBuffer);
  ABlobBuffer := Nil;
end;

<<<<<<< HEAD
{ TBufBlobStream }

=======
>>>>>>> graemeg/fixes_2_2
function TBufBlobStream.Seek(Offset: Longint; Origin: Word): Longint;

begin
  Case Origin of
    soFromBeginning : FPosition:=Offset;
    soFromEnd       : FPosition:=FBlobBuffer^.Size+Offset;
<<<<<<< HEAD
    soFromCurrent   : FPosition:=FPosition+Offset;
=======
    soFromCurrent   : FpoSition:=FPosition+Offset;
>>>>>>> graemeg/fixes_2_2
  end;
  Result:=FPosition;
end;


function TBufBlobStream.Read(var Buffer; Count: Longint): Longint;

var ptr : pointer;

begin
<<<<<<< HEAD
  if FPosition + Count > FBlobBuffer^.Size then
    Count := FBlobBuffer^.Size-FPosition;
  ptr := FBlobBuffer^.Buffer+FPosition;
  move(ptr^, Buffer, Count);
  inc(FPosition, Count);
  result := Count;
=======
  if FPosition + count > FBlobBuffer^.Size then
    count := FBlobBuffer^.Size-FPosition;
  ptr := FBlobBuffer^.Buffer+FPosition;
  move(ptr^,buffer,count);
  inc(FPosition,count);
  result := count;
>>>>>>> graemeg/fixes_2_2
end;

function TBufBlobStream.Write(const Buffer; Count: Longint): Longint;

var ptr : pointer;

begin
<<<<<<< HEAD
  ReAllocMem(FBlobBuffer^.Buffer, FPosition+Count);
  ptr := FBlobBuffer^.Buffer+FPosition;
  move(buffer, ptr^, Count);
  inc(FBlobBuffer^.Size, Count);
  inc(FPosition, Count);
  FModified := True;
  Result := Count;
=======
  ReAllocMem(FBlobBuffer^.Buffer,FPosition+Count);
  ptr := FBlobBuffer^.Buffer+FPosition;
  move(buffer,ptr^,count);
  inc(FBlobBuffer^.Size,count);
  inc(FPosition,count);
  Result := count;
>>>>>>> graemeg/fixes_2_2
end;

constructor TBufBlobStream.Create(Field: TBlobField; Mode: TBlobStreamMode);

var bufblob : TBufBlobField;
<<<<<<< HEAD
<<<<<<< HEAD
    CurrBuff : TRecordBuffer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  FField := Field;
  FDataSet := Field.DataSet as TCustomBufDataset;
  with FDataSet do
    if Mode = bmRead then
=======
=======
>>>>>>> graemeg/cpstrnew
=======

begin
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FDataset := Field.DataSet as TCustomBufDataset;
=======

begin
  FDataset := Field.DataSet as TBufDataset;
>>>>>>> graemeg/fixes_2_2
  if mode = bmread then
    begin
    if not field.getData(@bufblob) then
      DatabaseError(SFieldIsNull);
    if not assigned(bufblob.BlobBuffer) then with FDataSet do
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
      begin
      if not Field.GetData(@bufblob) then
        DatabaseError(SFieldIsNull);
      if not assigned(bufblob.BlobBuffer) then
        begin
        bufblob.BlobBuffer := GetNewBlobBuffer;
        LoadBlobIntoBuffer(FieldDefs[Field.FieldNo-1], @bufblob);
        end;
      FBlobBuffer := bufblob.BlobBuffer;
<<<<<<< HEAD
      end
    else if Mode=bmWrite then
      begin
      FBlobBuffer := GetNewWriteBlobBuffer;
      FBlobBuffer^.FieldNo := Field.FieldNo;
      if Field.GetData(@bufblob) and assigned(bufblob.BlobBuffer) then
        FBlobBuffer^.OrgBufID := bufblob.BlobBuffer^.OrgBufID
      else
        FBlobBuffer^.OrgBufID := -1;
      bufblob.BlobBuffer := FBlobBuffer;
=======
    end
  else if mode=bmWrite then with FDataSet as TCustomBufDataset do
=======
      begin
      FBlobBuffer := GetNewBlobBuffer;
      bufblob.BlobBuffer := FBlobBuffer;
      LoadBlobIntoBuffer(FieldDefs[field.FieldNo-1],@bufblob);
      end
    else
      FBlobBuffer := bufblob.BlobBuffer;
    end
  else if mode=bmWrite then with FDataSet as TBufDataset do
>>>>>>> graemeg/fixes_2_2
    begin
    FBlobBuffer := GetNewWriteBlobBuffer;
    FBlobBuffer^.FieldNo := Field.FieldNo;
    if (field.getData(@bufblob)) and assigned(bufblob.BlobBuffer) then
      FBlobBuffer^.OrgBufID := bufblob.BlobBuffer^.OrgBufID
    else
      FBlobBuffer^.OrgBufID := -1;
    end;
end;

<<<<<<< HEAD
function TCustomBufDataset.CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

      CurrBuff := GetCurrentBuffer;
      // unset null flag for blob field
      unSetFieldIsNull(PByte(CurrBuff), Field.FieldNo-1);
      // redirect pointer in current record buffer to new write blob buffer
      inc(CurrBuff, FDataSet.FFieldBufPositions[Field.FieldNo-1]);
      Move(bufblob, CurrBuff^, FDataSet.GetFieldSize(FDataSet.FieldDefs[Field.FieldNo-1]));
      FModified := True;
      end;
end;
=======

var bufblob : TBufBlobField;
>>>>>>> origin/cpstrnew

destructor TBufBlobStream.Destroy;
begin
  if FModified then
    begin
    // if TBufBlobStream was requested, but no data was written, then Size = 0;
    //  used by TBlobField.Clear, so in this case set Field to null
    //FField.Modified := True; // should be set to True, but TBlobField.Modified is never reset

    if not (FDataSet.State in [dsFilter, dsCalcFields, dsNewValue]) then
      begin
      if FBlobBuffer^.Size = 0 then // empty blob = IsNull
        // blob stream should be destroyed while DataSet is in write state
        SetFieldIsNull(PByte(FDataSet.GetCurrentBuffer), FField.FieldNo-1);
      FDataSet.DataEvent(deFieldChange, PtrInt(FField));
      end;
    end;
<<<<<<< HEAD
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCustomBufDataset.CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
end;

>>>>>>> origin/cpstrnew
procedure TCustomBufDataset.AddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string = '';
                               const ACaseInsFields: string = '');
begin
  CheckBiDirectional;
  if AFields='' then DatabaseError(SNoIndexFieldNameGiven);

  if FIndexesCount=0 then
    InitDefaultIndexes;
  
  if active and (FIndexesCount=FMaxIndexesCount) then
    DatabaseError(SMaxIndexes);
>>>>>>> graemeg/cpstrnew

var bufblob : TBufBlobField;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomBufDataset.SaveToFile(AFileName: string;
  Format: TDataPacketFormat);
var AFileStream : TFileStream;
>>>>>>> graemeg/cpstrnew
begin
  Result := nil;
  case Mode of
    bmRead:
      if not Field.GetData(@bufblob) then Exit;
    bmWrite:
      begin
      if not (State in [dsEdit, dsInsert, dsFilter, dsCalcFields]) then
        DatabaseErrorFmt(SNotEditing, [Name], Self);
      if Field.ReadOnly and not (State in [dsSetKey, dsFilter]) then
        DatabaseErrorFmt(SReadOnlyField, [Field.DisplayName]);
      end;
  end;
  Result := TBufBlobStream.Create(Field as TBlobField, Mode);
end;

procedure TCustomBufDataset.SetDatasetPacket(AReader: TDataPacketReader);
=======
function TBufDataset.CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream;

var bufblob : TBufBlobField;

begin
  result := nil;
  if mode=bmread then
    begin
    if not field.getData(@bufblob) then
      exit;

    result := TBufBlobStream.Create(Field as tblobfield,bmread);
    end
  else if mode=bmWrite then
    begin
    if not (state in [dsEdit, dsInsert, dsFilter, dsCalcFields]) then
      begin
      DatabaseErrorFmt(SNotEditing,[Name],self);
      exit;
      end;

    result := TBufBlobStream.Create(Field as tblobfield,bmWrite);

    if not (State in [dsCalcFields, dsFilter, dsNewValue]) then
      DataEvent(deFieldChange, Ptrint(Field));
    end;
end;

procedure TBufDataset.AddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string = '';
                               const ACaseInsFields: string = '');
begin
  if AFields='' then DatabaseError(SNoIndexFieldNameGiven);
  
  if active and (FIndexesCount=FMaxIndexesCount) then
    DatabaseError(SMaxIndexes);

  // If not all packets are fetched, you can not sort properly.
  if not active then
    FPacketRecords:=-1;
  InternalAddIndex(AName,AFields,AOptions,ADescFields,ACaseInsFields);
end;

procedure TBufDataset.SaveToFile(AFileName: string;
  Format: TDataPacketFormat);
var AFileStream : TFileStream;
begin
  if AFileName='' then AFileName := FFileName;
  AFileStream := TFileStream.Create(AFileName,fmCreate);
  try
    SaveToStream(AFileStream, Format);
  finally
    AFileStream.Free;
  end;
end;

procedure TBufDataset.SetDatasetPacket(AReader: TDataPacketReader);
>>>>>>> graemeg/fixes_2_2
begin
  FDatasetReader := AReader;
  try
    Open;
  finally
    FDatasetReader := nil;
  end;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.GetDatasetPacket(AWriter: TDataPacketReader);
=======
procedure TBufDataset.GetDatasetPacket(AWriter: TDataPacketReader);
>>>>>>> graemeg/fixes_2_2

  procedure StoreUpdateBuffer(AUpdBuffer : TRecUpdateBuffer; var ARowState: TRowState);
  var AThisRowState : TRowState;
      AStoreUpdBuf  : Integer;
  begin
    if AUpdBuffer.UpdateKind = ukModify then
      begin
      AThisRowState := [rsvOriginal];
      ARowState:=[rsvUpdated];
      end
    else if AUpdBuffer.UpdateKind = ukDelete then
      begin
      AStoreUpdBuf:=FCurrentUpdateBuffer;
<<<<<<< HEAD
      if GetRecordUpdateBuffer(AUpdBuffer.BookmarkData,True,False) then
        begin
        repeat
          if FCurrentIndex.SameBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData, @AUpdBuffer.BookmarkData) then
            StoreUpdateBuffer(FUpdateBuffer[FCurrentUpdateBuffer], ARowState);
=======
      if GetRecordUpdateBuffer(AUpdBuffer.BookmarkData,True) then
        begin
        repeat
        if FCurrentIndex.CompareBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData, @AUpdBuffer.BookmarkData) then
          StoreUpdateBuffer(FUpdateBuffer[FCurrentUpdateBuffer], ARowState);
>>>>>>> graemeg/fixes_2_2
        until not GetRecordUpdateBuffer(AUpdBuffer.BookmarkData,True,True)
        end;
      FCurrentUpdateBuffer:=AStoreUpdBuf;
      AThisRowState := [rsvDeleted];
      end
<<<<<<< HEAD
    else // ie: UpdateKind = ukInsert
      ARowState := [rsvInserted];

    FFilterBuffer:=AUpdBuffer.OldValuesBuffer;
    // OldValuesBuffer is nil if the record is either inserted or inserted and then deleted
    if assigned(FFilterBuffer) then
      FDatasetReader.StoreRecord(AThisRowState,FCurrentUpdateBuffer);
=======
    else // ie: updatekind = ukInsert
      begin
      ARowState := [rsvInserted];
      Exit;
      end;
    FFilterBuffer:=AUpdBuffer.OldValuesBuffer;
    FDatasetReader.StoreRecord(Self,AThisRowState,FCurrentUpdateBuffer);
>>>>>>> graemeg/fixes_2_2
  end;

  procedure HandleUpdateBuffersFromRecord(AFirstCall : boolean;ARecBookmark : TBufBookmark; var ARowState: TRowState);
  var StoreUpdBuf1,StoreUpdBuf2 : Integer;
  begin
    if AFirstCall then ARowState:=[];
    if GetRecordUpdateBuffer(ARecBookmark,True,not AFirstCall) then
      begin
      if FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind=ukDelete then
        begin
        StoreUpdBuf1:=FCurrentUpdateBuffer;
        HandleUpdateBuffersFromRecord(False,ARecBookmark,ARowState);
        StoreUpdBuf2:=FCurrentUpdateBuffer;
        FCurrentUpdateBuffer:=StoreUpdBuf1;
        StoreUpdateBuffer(FUpdateBuffer[StoreUpdBuf1], ARowState);
        FCurrentUpdateBuffer:=StoreUpdBuf2;
        end
      else
        begin
        StoreUpdateBuffer(FUpdateBuffer[FCurrentUpdateBuffer], ARowState);
        HandleUpdateBuffersFromRecord(False,ARecBookmark,ARowState);
        end;
      end
  end;

var ScrollResult   : TGetResult;
<<<<<<< HEAD
    SavedState     : TDataSetState;
=======
    StoreDSState   : TDataSetState;
>>>>>>> graemeg/fixes_2_2
    ABookMark      : PBufBookmark;
    ATBookmark     : TBufBookmark;
    RowState       : TRowState;

begin
  FDatasetReader := AWriter;
  try
<<<<<<< HEAD
    //  CheckActive;
    ABookMark:=@ATBookmark;
    FDatasetReader.StoreFieldDefs(FAutoIncValue);

    SavedState:=SetTempState(dsFilter);
=======
    //CheckActive;
    ABookMark:=@ATBookmark;
    FDatasetReader.StoreFieldDefs(FieldDefs);

    StoreDSState:=State;
    SetTempState(dsFilter);
>>>>>>> graemeg/fixes_2_2
    ScrollResult:=FCurrentIndex.ScrollFirst;
    while ScrollResult=grOK do
      begin
      RowState:=[];
      FCurrentIndex.StoreCurrentRecIntoBookmark(ABookmark);
      HandleUpdateBuffersFromRecord(True,ABookmark^,RowState);
      FFilterBuffer:=FCurrentIndex.CurrentBuffer;
      if RowState=[] then
<<<<<<< HEAD
        FDatasetReader.StoreRecord([])
      else
        FDatasetReader.StoreRecord(RowState,FCurrentUpdateBuffer);
=======
        FDatasetReader.StoreRecord(Self,[])
      else
        FDatasetReader.StoreRecord(Self,RowState,FCurrentUpdateBuffer);
>>>>>>> graemeg/fixes_2_2

      ScrollResult:=FCurrentIndex.ScrollForward;
      if ScrollResult<>grOK then
        begin
        if getnextpacket>0 then
          ScrollResult := FCurrentIndex.ScrollForward;
        end;
      end;
<<<<<<< HEAD
    // There could be an update buffer linked to the last (spare) record
    FCurrentIndex.StoreSpareRecIntoBookmark(ABookmark);
    HandleUpdateBuffersFromRecord(True,ABookmark^,RowState);

    RestoreState(SavedState);
=======
    // There could be a update-buffer linked to the last (spare) record
    FCurrentIndex.StoreSpareRecIntoBookmark(ABookmark);
    HandleUpdateBuffersFromRecord(True,ABookmark^,RowState);

    RestoreState(StoreDSState);
>>>>>>> graemeg/fixes_2_2

    FDatasetReader.FinalizeStoreRecords;
  finally
    FDatasetReader := nil;
  end;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.LoadFromStream(AStream: TStream; Format: TDataPacketFormat);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var APacketReader : TDataPacketReader;
begin
  CheckBiDirectional;
  APacketReader:=GetPacketReader(Format, AStream);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var APacketReaderReg : TDatapacketReaderRegistration;
    APacketReader : TDataPacketReader;
begin
  CheckBiDirectional;
=======
procedure TBufDataset.LoadFromStream(AStream: TStream; Format: TDataPacketFormat);
var APacketReaderReg : TDatapacketReaderRegistration;
    APacketReader : TDataPacketReader;
begin
>>>>>>> graemeg/fixes_2_2
  if GetRegisterDatapacketReader(AStream,format,APacketReaderReg) then
    APacketReader := APacketReaderReg.ReaderClass.create(AStream)
  else if TFpcBinaryDatapacketReader.RecognizeStream(AStream) then
    begin
    AStream.Seek(0,soFromBeginning);
    APacketReader := TFpcBinaryDatapacketReader.create(AStream)
    end
  else
    DatabaseError(SStreamNotRecognised);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
  try
    SetDatasetPacket(APacketReader);
  finally
    APacketReader.Free;
  end;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.SaveToStream(AStream: TStream; Format: TDataPacketFormat);
var APacketReaderReg : TDatapacketReaderRegistration;
    APacketWriter : TDataPacketReader;
begin
  CheckBiDirectional;
  if GetRegisterDatapacketReader(Nil,format,APacketReaderReg) then
    APacketWriter := APacketReaderReg.ReaderClass.Create(Self, AStream)
  else if Format = dfBinary then
    APacketWriter := TFpcBinaryDatapacketReader.Create(Self, AStream)
=======
procedure TBufDataset.SaveToStream(AStream: TStream; Format: TDataPacketFormat);
var APacketReaderReg : TDatapacketReaderRegistration;
    APacketWriter : TDataPacketReader;
begin
  if GetRegisterDatapacketReader(Nil,format,APacketReaderReg) then
    APacketWriter := APacketReaderReg.ReaderClass.create(AStream)
  else if Format = dfBinary then
    APacketWriter := TFpcBinaryDatapacketReader.create(AStream)
>>>>>>> graemeg/fixes_2_2
  else
    DatabaseError(SNoReaderClassRegistered);
  try
    GetDatasetPacket(APacketWriter);
  finally
    APacketWriter.Free;
  end;
end;

<<<<<<< HEAD
procedure TCustomBufDataset.LoadFromFile(AFileName: string; Format: TDataPacketFormat);
=======
procedure TBufDataset.LoadFromFile(AFileName: string; Format: TDataPacketFormat);
>>>>>>> graemeg/fixes_2_2
var AFileStream : TFileStream;
begin
  if AFileName='' then AFileName := FFileName;
  AFileStream := TFileStream.Create(AFileName,fmOpenRead);
  try
    LoadFromStream(AFileStream, Format);
  finally
    AFileStream.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.SaveToFile(AFileName: string;
  Format: TDataPacketFormat);
var AFileStream : TFileStream;
begin
  if AFileName='' then AFileName := FFileName;
  AFileStream := TFileStream.Create(AFileName,fmCreate);
  try
    SaveToStream(AFileStream, Format);
  finally
    AFileStream.Free;
  end;
end;

procedure TCustomBufDataset.CreateDataset;
var AStoreFileName: string;
=======
procedure TCustomBufDataset.CreateDataset;
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.CreateDataset;
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.CreateDataset;
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.CreateDataset;
>>>>>>> origin/cpstrnew
begin
  CheckInactive;
  if ((Fields.Count=0) or (FieldDefs.Count=0)) then
    begin
    if (FieldDefs.Count>0) then
      CreateFields
    else if (Fields.Count>0) then
      begin
      InitFieldDefsFromFields;
      BindFields(True);
      end
    else
      raise Exception.Create(SErrNoFieldsDefined);
    FAutoIncValue:=1;
    end;
  // When a FileName is set, do not read from this file; we want empty dataset
  AStoreFileName:=FFileName;
  FFileName := '';
  try
    Open;
  finally
    FFileName:=AStoreFileName;
  end;
end;

function TCustomBufDataset.BookmarkValid(ABookmark: TBookmark): Boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=assigned(FCurrentIndex) and  FCurrentIndex.BookmarkValid(pointer(ABookmark));
end;

<<<<<<< HEAD
function TCustomBufDataset.CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint;
begin
  if Bookmark1 = Bookmark2 then
=======
=======
function TCustomBufDataset.BookmarkValid(ABookmark: TBookmark): Boolean;
begin
>>>>>>> graemeg/cpstrnew
  Result:=assigned(FCurrentIndex) and  FCurrentIndex.BookmarkValid(ABookmark);
end;

=======
  Result:=assigned(FCurrentIndex) and  FCurrentIndex.BookmarkValid(ABookmark);
end;

>>>>>>> graemeg/cpstrnew
=======
  Result:=assigned(FCurrentIndex) and  FCurrentIndex.BookmarkValid(ABookmark);
end;

>>>>>>> origin/cpstrnew
function TCustomBufDataset.CompareBookmarks(Bookmark1, Bookmark2: TBookmark
  ): Longint;
begin
  if Assigned(FCurrentIndex) and FCurrentIndex.CompareBookmarks(Bookmark1,Bookmark2) then
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
    Result := 0
  else if not assigned(Bookmark1) then
    Result := 1
  else if not assigned(Bookmark2) then
    Result := -1
  else if assigned(FCurrentIndex) then
    Result := FCurrentIndex.CompareBookmarks(pointer(Bookmark1),pointer(Bookmark2))
=======
procedure TBufDataset.CreateDataset;
begin
  CheckInactive;
  CreateFields;
end;

function TBufDataset.BookmarkValid(ABookmark: TBookmark): Boolean;
begin
  Result:=FCurrentIndex.BookmarkValid(ABookmark);
end;

function TBufDataset.CompareBookmarks(Bookmark1, Bookmark2: TBookmark
  ): Longint;
begin
  if FCurrentIndex.CompareBookmarks(Bookmark1,Bookmark2) then
    Result := 0
>>>>>>> graemeg/fixes_2_2
  else
    Result := -1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomBufDataset.IntLoadFieldDefsFromFile;
=======
procedure TCustomBufDataset.IntLoadFielddefsFromFile;
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.IntLoadFielddefsFromFile;
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.IntLoadFielddefsFromFile;
>>>>>>> graemeg/cpstrnew
=======
procedure TCustomBufDataset.IntLoadFielddefsFromFile;
>>>>>>> origin/cpstrnew

begin
  FReadFromFile := True;
  if not assigned(FDatasetReader) then
    begin
    FFileStream := TFileStream.Create(FileName, fmOpenRead);
    FDatasetReader := GetPacketReader(dfAny, FFileStream);
    end;

  FieldDefs.Clear;
  FDatasetReader.LoadFieldDefs(FAutoIncValue);
  if DefaultFields then
    CreateFields
  else
    BindFields(true);
end;

procedure TCustomBufDataset.IntLoadRecordsFromFile;

var SavedState      : TDataSetState;
=======
procedure TBufDataset.IntLoadFielddefsFromFile;

begin
  FDatasetReader.LoadFielddefs(FieldDefs);
  if DefaultFields then CreateFields;
end;

procedure TBufDataset.IntLoadRecordsFromFile;

var StoreState      : TDataSetState;
>>>>>>> graemeg/fixes_2_2
    AddRecordBuffer : boolean;
    ARowState       : TRowState;
    AUpdOrder       : integer;
    x               : integer;

begin
<<<<<<< HEAD
  CheckBiDirectional;
  FDatasetReader.InitLoadRecords;
  SavedState:=SetTempState(dsFilter);
=======
  FDatasetReader.InitLoadRecords;
  StoreState:=SetTempState(dsFilter);
>>>>>>> graemeg/fixes_2_2

  while FDatasetReader.GetCurrentRecord do
    begin
    ARowState := FDatasetReader.GetRecordRowState(AUpdOrder);
    if rsvOriginal in ARowState then
      begin
      if length(FUpdateBuffer) < (AUpdOrder+1) then
        SetLength(FUpdateBuffer,AUpdOrder+1);

      FCurrentUpdateBuffer:=AUpdOrder;

      FFilterBuffer:=IntAllocRecordBuffer;
      fillchar(FFilterBuffer^,FNullmaskSize,0);
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := FFilterBuffer;
<<<<<<< HEAD
      FDatasetReader.RestoreRecord;
=======
      FDatasetReader.RestoreRecord(self);
>>>>>>> graemeg/fixes_2_2

      FDatasetReader.GotoNextRecord;
      if not FDatasetReader.GetCurrentRecord then
        DatabaseError(SStreamNotRecognised);
      ARowState := FDatasetReader.GetRecordRowState(AUpdOrder);
      if rsvUpdated in ARowState then
        FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind:= ukModify
      else
        DatabaseError(SStreamNotRecognised);

      FFilterBuffer:=FIndexes[0].SpareBuffer;
      FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
      fillchar(FFilterBuffer^,FNullmaskSize,0);

<<<<<<< HEAD
<<<<<<< HEAD
      FDatasetReader.RestoreRecord;
=======
      FDatasetReader.RestoreRecord(self);
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
      FIndexes[0].AddRecord;
=======
      FDatasetReader.RestoreRecord(self);
      FIndexes[0].AddRecord(IntAllocRecordBuffer);
>>>>>>> graemeg/fixes_2_2
      inc(FBRecordCount);

      AddRecordBuffer:=False;

      end
    else if rsvDeleted in ARowState then
      begin
      if length(FUpdateBuffer) < (AUpdOrder+1) then
        SetLength(FUpdateBuffer,AUpdOrder+1);

      FCurrentUpdateBuffer:=AUpdOrder;

      FFilterBuffer:=IntAllocRecordBuffer;
      fillchar(FFilterBuffer^,FNullmaskSize,0);
<<<<<<< HEAD

      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := FFilterBuffer;
      FDatasetReader.RestoreRecord;

      FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind:= ukDelete;
      FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
      FIndexes[0].AddRecord;
=======
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := FFilterBuffer;
      FDatasetReader.RestoreRecord(self);

      FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind:= ukDelete;
      FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
      FIndexes[0].AddRecord(IntAllocRecordBuffer);
>>>>>>> graemeg/fixes_2_2
      FIndexes[0].RemoveRecordFromIndex(FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
      FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData);

      for x := FCurrentUpdateBuffer+1 to length(FUpdateBuffer)-1 do
<<<<<<< HEAD
        if FIndexes[0].SameBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData,@FUpdateBuffer[x].NextBookmarkData) then
=======
        if Findexes[0].CompareBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData,@FUpdateBuffer[x].NextBookmarkData) then
>>>>>>> graemeg/fixes_2_2
          FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[x].NextBookmarkData);

      AddRecordBuffer:=False;
      end
    else
      AddRecordBuffer:=True;

    if AddRecordBuffer then
      begin
      FFilterBuffer:=FIndexes[0].SpareBuffer;
      fillchar(FFilterBuffer^,FNullmaskSize,0);

<<<<<<< HEAD
      FDatasetReader.RestoreRecord;
=======
      FDatasetReader.RestoreRecord(self);
>>>>>>> graemeg/fixes_2_2

      if rsvInserted in ARowState then
        begin
        if length(FUpdateBuffer) < (AUpdOrder+1) then
          SetLength(FUpdateBuffer,AUpdOrder+1);
        FCurrentUpdateBuffer:=AUpdOrder;
        FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind:= ukInsert;
<<<<<<< HEAD
        FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
        end;

      FIndexes[0].AddRecord;
=======
        FCurrentIndex.StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
        end;

      FIndexes[0].AddRecord(IntAllocRecordBuffer);
>>>>>>> graemeg/fixes_2_2
      inc(FBRecordCount);
      end;

    FDatasetReader.GotoNextRecord;
    end;

<<<<<<< HEAD
  RestoreState(SavedState);
=======
  RestoreState(StoreState);
>>>>>>> graemeg/fixes_2_2
  FIndexes[0].SetToFirstRecord;
  FAllPacketsFetched:=True;
  if assigned(FFileStream) then
    begin
    FreeAndNil(FFileStream);
    FreeAndNil(FDatasetReader);
    end;
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // rebuild indexes
  BuildIndexes;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCustomBufDataset.InternalAddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string;
=======
end;

procedure TBufDataset.InternalAddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string;
>>>>>>> graemeg/fixes_2_2
                                       const ACaseInsFields: string);
var StoreIndNr : Integer;
begin
  if Active then FetchAll;
  if FIndexesCount>0 then
    StoreIndNr:=FCurrentIndex.IndNr
  else
    StoreIndNr:=0;
  inc(FIndexesCount);
  setlength(FIndexes,FIndexesCount); // This invalidates the currentindex! -> not anymore
  FCurrentIndex:=FIndexes[StoreIndNr];
<<<<<<< HEAD
  if IsUniDirectional then
    FIndexes[FIndexesCount-1] := TUniDirectionalBufIndex.Create(self)
  else
    FIndexes[FIndexesCount-1] := TDoubleLinkedBufIndex.Create(self);
=======
  FIndexes[FIndexesCount-1] := TDoubleLinkedBufIndex.Create(self);
>>>>>>> graemeg/fixes_2_2
//  FIndexes[FIndexesCount-1] := TArrayBufIndex.Create(self);
  FIndexes[FIndexesCount-1].InitialiseIndex;
  with (FIndexes[FIndexesCount-1] as TBufIndex) do
    begin
    Name:=AName;
    FieldsName:=AFields;
    DescFields:=ADescFields;
    CaseinsFields:=ACaseInsFields;
    Options:=AOptions;
    IndNr:=FIndexesCount-1;
    end;

  if Active then
    begin
    (FIndexes[FIndexesCount-1] as TDoubleLinkedBufIndex).FFirstRecBuf := pointer(IntAllocRecordBuffer);
    (FIndexes[FIndexesCount-1] as TDoubleLinkedBufIndex).FLastRecBuf := (FIndexes[FIndexesCount-1] as TDoubleLinkedBufIndex).FFirstRecBuf;
    (FCurrentIndex as TDoubleLinkedBufIndex).FCurrentRecBuf := (FIndexes[FIndexesCount-1] as TDoubleLinkedBufIndex).FLastRecBuf;
    BuildIndex(FIndexes[FIndexesCount-1]);
    end
  else if FIndexesCount>FMaxIndexesCount then
    FMaxIndexesCount := FIndexesCount;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
end;

procedure TCustomBufDataset.DoFilterRecord(out Acceptable: Boolean);
=======
end;

procedure TBufDataset.DoFilterRecord(var Acceptable: Boolean);
>>>>>>> graemeg/fixes_2_2
begin
  Acceptable := true;
  // check user filter
  if Assigned(OnFilterRecord) then
    OnFilterRecord(Self, Acceptable);

  // check filtertext
  if Acceptable and (Length(Filter) > 0) then
    Acceptable := Boolean((FParser.ExtractFromBuffer(GetCurrentBuffer))^);
<<<<<<< HEAD
end;

procedure TCustomBufDataset.SetFilterText(const Value: String);
=======

end;

procedure TBufDataset.SetFilterText(const Value: String);
>>>>>>> graemeg/fixes_2_2
begin
  if Value = Filter then
    exit;

  // parse
  ParseFilter(Value);

  // call dataset method
  inherited;

  // refilter dataset if filtered
<<<<<<< HEAD
  if IsCursorOpen and Filtered then Resync([]);
end;

procedure TCustomBufDataset.SetFiltered(Value: Boolean); {override;}
=======
  if IsCursorOpen and Filtered then Refresh;
end;

procedure TBufDataset.SetFiltered(Value: Boolean); {override;}
>>>>>>> graemeg/fixes_2_2
begin
  if Value = Filtered then
    exit;

  // pass on to ancestor
  inherited;

  // only refresh if active
  if IsCursorOpen then
<<<<<<< HEAD
    Resync([]);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TCustomBufDataset.InternalRefresh;
var StoreDefaultFields: boolean;
begin
  if length(FUpdateBuffer)>0 then
    DatabaseError(SErrApplyUpdBeforeRefresh);
  StoreDefaultFields:=DefaultFields;
  SetDefaultFields(False);
  FreeFieldBuffers;
  ClearBuffers;
  InternalClose;
  BeforeRefreshOpenCursor;
  InternalOpen;
  SetDefaultFields(StoreDefaultFields);
end;

procedure TCustomBufDataset.BeforeRefreshOpenCursor;
begin
  // Do nothing
end;

procedure TCustomBufDataset.DataEvent(Event: TDataEvent; Info: PtrInt);
begin
  if Event = deUpdateState then
    // Save DataSet.State set by DataSet.SetState (filter out State set by DataSet.SetTempState)
    FSavedState := State;
  inherited;
end;

function TCustomBufDataset.Fetch: boolean;
begin
=======
end;

procedure TCustomBufDataset.InternalRefresh;
var StoreDefaultFields: boolean;
begin
=======
end;

procedure TCustomBufDataset.InternalRefresh;
var StoreDefaultFields: boolean;
begin
>>>>>>> graemeg/cpstrnew
  StoreDefaultFields:=DefaultFields;
  SetDefaultFields(False);
  FreeFieldBuffers;
  ClearBuffers;
  InternalClose;
  BeforeRefreshOpenCursor;
  InternalOpen;
  SetDefaultFields(StoreDefaultFields);
end;

procedure TCustomBufDataset.BeforeRefreshOpenCursor;
begin
  // Do nothing
end;

function TCustomBufDataset.Fetch: boolean;
begin
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
end;

procedure TCustomBufDataset.InternalRefresh;
var StoreDefaultFields: boolean;
begin
  StoreDefaultFields:=DefaultFields;
  SetDefaultFields(False);
  FreeFieldBuffers;
  ClearBuffers;
  InternalClose;
  BeforeRefreshOpenCursor;
  InternalOpen;
  SetDefaultFields(StoreDefaultFields);
end;

procedure TCustomBufDataset.BeforeRefreshOpenCursor;
begin
  // Do nothing
end;

function TCustomBufDataset.Fetch: boolean;
begin
>>>>>>> graemeg/cpstrnew
=======
end;

procedure TCustomBufDataset.InternalRefresh;
var StoreDefaultFields: boolean;
begin
  StoreDefaultFields:=DefaultFields;
  SetDefaultFields(False);
  FreeFieldBuffers;
  ClearBuffers;
  InternalClose;
  BeforeRefreshOpenCursor;
  InternalOpen;
  SetDefaultFields(StoreDefaultFields);
end;

procedure TCustomBufDataset.BeforeRefreshOpenCursor;
begin
  // Do nothing
end;

function TCustomBufDataset.Fetch: boolean;
begin
>>>>>>> origin/cpstrnew
  // Empty procedure to make it possible to use TCustomBufDataset as a memory dataset
  Result := False;
end;

function TCustomBufDataset.LoadField(FieldDef: TFieldDef; buffer: pointer; out
  CreateBlob: boolean): boolean;
begin
  // Empty procedure to make it possible to use TCustomBufDataset as a memory dataset
=======
    Refresh;
end;

function TBufDataset.Fetch: boolean;
begin
  // Empty procedure to make it possible to use TBufDataset as a memory dataset
  Result := False;
end;

function TBufDataset.LoadField(FieldDef: TFieldDef; buffer: pointer; out
  CreateBlob: boolean): boolean;
begin
  // Empty procedure to make it possible to use TBufDataset as a memory dataset
>>>>>>> graemeg/fixes_2_2
  CreateBlob := False;
  Result := False;
end;

<<<<<<< HEAD
function TCustomBufDataset.IsReadFromPacket: Boolean;
begin
  Result := (FDatasetReader<>nil) or (FFileName<>'') or FReadFromFile;
end;

procedure TCustomBufDataset.ParseFilter(const AFilter: string);
=======
function TBufDataset.IsReadFromPacket: Boolean;
begin
  Result := (FDatasetReader<>nil) or (FFileName<>'');
end;

procedure TBufDataset.ParseFilter(const AFilter: string);
>>>>>>> graemeg/fixes_2_2
begin
  // parser created?
  if Length(AFilter) > 0 then
  begin
    if (FParser = nil) and IsCursorOpen then
    begin
      FParser := TBufDatasetParser.Create(Self);
    end;
<<<<<<< HEAD
    // is there a parser now?
=======
    // have a parser now?
>>>>>>> graemeg/fixes_2_2
    if FParser <> nil then
    begin
      // set options
      FParser.PartialMatch := not (foNoPartialCompare in FilterOptions);
      FParser.CaseInsensitive := foCaseInsensitive in FilterOptions;
      // parse expression
      FParser.ParseExpression(AFilter);
    end;
  end;
end;

<<<<<<< HEAD
function TCustomBufDataset.Locate(const KeyFields: string;
  const KeyValues: Variant; Options: TLocateOptions): boolean;

<<<<<<< HEAD
var SearchFields    : TList;
    DBCompareStruct : TDBCompareStruct;
    ABookmark       : TBufBookmark;
    SavedState      : TDataSetState;
    FilterRecord    : TRecordBuffer;
    FilterAcceptable: boolean;

begin
  // Call inherited to make sure the dataset is bi-directional
  Result := inherited;
=======
=======
function TArrayBufIndex.GetRecordFromBookmark(ABookMark: TBufBookmark) : integer;
begin
  // ABookmark.BookMarkBuf is nil if SetRecNo calls GotoBookmark
  if (ABookmark.BookmarkData<>nil) and (FRecordArray[ABookmark.BookmarkInt]<>ABookmark.BookmarkData) then
    begin
    // Start searching two records before the expected record
    if ABookmark.BookmarkInt > 2 then
      Result := ABookmark.BookmarkInt-2
    else
      Result := 0;

    while (Result<FLastRecInd) do
      begin
      if (FRecordArray[Result] = ABookmark.BookmarkData) then exit;
      inc(Result);
      end;

    Result:=0;
    while (Result<ABookmark.BookmarkInt) do
      begin
      if (FRecordArray[Result] = ABookmark.BookmarkData) then exit;
      inc(Result);
      end;

>>>>>>> graemeg/fixes_2_2
    DatabaseError(SInvalidBookmark)
    end
  else
    Result := ABookmark.BookmarkInt;
end;

<<<<<<< HEAD
Function TCustomBufDataset.Locate(const KeyFields: string; const KeyValues: Variant; options: TLocateOptions) : boolean;
=======
Function TBufDataset.Locate(const KeyFields: string; const KeyValues: Variant; options: TLocateOptions) : boolean;
>>>>>>> graemeg/fixes_2_2

var CurrLinkItem    : PBufRecLinkItem;
    bm              : TBufBookmark;
    SearchFields    : TList;
    FieldsAmount    : Integer;
    DBCompareStruct : TDBCompareStruct;
    FieldNr         : Integer;
    StoreDSState    : TDataSetState;
    FilterBuffer    : PChar;
<<<<<<< HEAD
    FiltAcceptable  : boolean;

begin
  Result := False;
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
  CheckActive;
=======


begin
  Result := False;
>>>>>>> graemeg/fixes_2_2
  if IsEmpty then exit;

  // Build the DBCompare structure
  SearchFields := TList.Create;
  try
    GetFieldList(SearchFields,KeyFields);
<<<<<<< HEAD
<<<<<<< HEAD
    if SearchFields.Count=0 then exit;
    ProcessFieldsToCompareStruct(SearchFields, nil, nil, [], Options, DBCompareStruct);
=======
=======
>>>>>>> graemeg/fixes_2_2
    FieldsAmount:=SearchFields.Count;
    if FieldsAmount=0 then exit;

    SetLength(DBCompareStruct,FieldsAmount);
    for FieldNr:=0 to FieldsAmount-1 do
<<<<<<< HEAD
      begin
      ProcessFieldCompareStruct(TField(SearchFields[FieldNr]),DBCompareStruct[FieldNr]);
      DBCompareStruct[FieldNr].Options:=options;
      end;
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
=======
      ProcessFieldCompareStruct(TField(SearchFields[FieldNr]),DBCompareStruct[FieldNr]);
>>>>>>> graemeg/fixes_2_2
  finally
    SearchFields.Free;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  // Set the filter buffer
  SavedState:=SetTempState(dsFilter);
  FilterRecord:=IntAllocRecordBuffer;
  FFilterBuffer:=FilterRecord + BufferOffset;
  SetFieldValues(KeyFields,KeyValues);
=======
=======
>>>>>>> graemeg/fixes_2_2
  // Set The filter-buffer
  StoreDSState:=State;
  FFilterBuffer:=FCurrentIndex.SpareBuffer;
  SetTempState(dsFilter);
  SetFieldValues(keyfields,KeyValues);
  CurrLinkItem := (FCurrentIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  FilterBuffer:=IntAllocRecordBuffer;
  move((FCurrentIndex as TDoubleLinkedBufIndex).FLastRecBuf^,FilterBuffer^,FRecordsize+sizeof(TBufRecLinkItem)*FMaxIndexesCount);
<<<<<<< HEAD
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

  // Iterate through the records until a match is found
  ABookmark.BookmarkData:=nil;
  while true do
    begin
    // try get next record
    if FCurrentIndex.GetRecord(@ABookmark, gmNext) <> grOK then
      // for grEOF ABookmark points to SpareRecord, which is used for storing next record(s)
      if getnextpacket = 0 then
        break;
    if IndexCompareRecords(FilterRecord, ABookmark.BookmarkData, DBCompareStruct) = 0 then
      begin
      if Filtered then
        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FFilterBuffer:=pointer(ABookmark.BookmarkData) + BufferOffset;
        // The dataset state is still dsFilter at this point, so we don't have to set it.
        DoFilterRecord(FilterAcceptable);
        if FilterAcceptable then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        FFilterBuffer:=pointer(CurrLinkItem)+(sizeof(TBufRecLinkItem)*MaxIndexesCount);
        // The dataset-state is still dsFilter at this point, so we don't have to set it.
        DoFilterRecord(FiltAcceptable);
        if FiltAcceptable then
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
          begin
          Result := True;
          break;
          end;
        end
      else
        begin
        Result := True;
        break;
        end;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    end;

  RestoreState(SavedState);
  FreeRecordBuffer(FilterRecord);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    CurrLinkItem := CurrLinkItem[(FCurrentIndex as TDoubleLinkedBufIndex).IndNr].next;
    if CurrLinkItem = (FCurrentIndex as TDoubleLinkedBufIndex).FLastRecBuf then
      getnextpacket;
    end;

  RestoreState(StoreDSState);
  FreeRecordBuffer(FilterBuffer);
>>>>>>> graemeg/cpstrnew
=======
  RestoreState(StoreDSState);

  // Iterate through the records until a match is found
  while (CurrLinkItem <> (FCurrentIndex as TDoubleLinkedBufIndex).FLastRecBuf) do
    begin
    if (IndexCompareRecords(FilterBuffer,CurrLinkItem,DBCompareStruct) = 0) then
      begin
      Result := True;
      break;
      end;
    CurrLinkItem := CurrLinkItem^.next;
    if CurrLinkItem = (FCurrentIndex as TDoubleLinkedBufIndex).FLastRecBuf then
      getnextpacket;
    end;
    
  FreeRecordBuffer(FilterBuffer);
>>>>>>> graemeg/fixes_2_2

  // If a match is found, jump to the found record
  if Result then
    begin
<<<<<<< HEAD
    ABookmark.BookmarkFlag := bfCurrent;
    GotoBookmark(@ABookmark);
    end;
end;

function TCustomBufDataset.Lookup(const KeyFields: string;
  const KeyValues: Variant; const ResultFields: string): Variant;
var
  bm:TBookmark;
begin
  result:=Null;
  bm:=GetBookmark;
  DisableControls;
  try
    if Locate(KeyFields,KeyValues,[]) then
      begin
      //  CalculateFields(ActiveBuffer); // not needed, done by Locate more than once
      result:=FieldValues[ResultFields];
      end;
    GotoBookmark(bm);
    FreeBookmark(bm);
  finally
    EnableControls;
  end;
end;

=======
    bm.BookmarkData := CurrLinkItem;
    bm.BookmarkFlag := bfCurrent;
    GotoBookmark(@bm);
    end;
end;

>>>>>>> graemeg/fixes_2_2
{ TArrayBufIndex }

function TArrayBufIndex.GetBookmarkSize: integer;
begin
  Result:=Sizeof(TBufBookmark);
end;

function TArrayBufIndex.GetCurrentBuffer: Pointer;
begin
<<<<<<< HEAD
  Result:=TRecordBuffer(FRecordArray[FCurrentRecInd]);
end;

function TArrayBufIndex.GetCurrentRecord:  TRecordBuffer;
=======
  Result:=pchar(FRecordArray[FCurrentRecInd]);
end;

function TArrayBufIndex.GetCurrentRecord: PChar;
>>>>>>> graemeg/fixes_2_2
begin
  Result:=GetCurrentBuffer;
end;

function TArrayBufIndex.GetIsInitialized: boolean;
begin
  Result:=Length(FRecordArray)>0;
end;

<<<<<<< HEAD
function TArrayBufIndex.GetSpareBuffer:  TRecordBuffer;
begin
  if FLastRecInd>-1 then
    Result:= TRecordBuffer(FRecordArray[FLastRecInd])
=======
function TArrayBufIndex.GetSpareBuffer: PChar;
begin
  if FLastRecInd>-1 then
    Result:=pchar(FRecordArray[FLastRecInd])
>>>>>>> graemeg/fixes_2_2
  else
    Result := nil;
end;

<<<<<<< HEAD
function TArrayBufIndex.GetSpareRecord:  TRecordBuffer;
=======
function TArrayBufIndex.GetSpareRecord: PChar;
>>>>>>> graemeg/fixes_2_2
begin
  Result := GetSpareBuffer;
end;

<<<<<<< HEAD
constructor TArrayBufIndex.Create(const ADataset: TCustomBufDataset);
=======
constructor TArrayBufIndex.Create(const ADataset: TBufDataset);
>>>>>>> graemeg/fixes_2_2
begin
  Inherited create(ADataset);
  FInitialBuffers:=10000;
  FGrowBuffer:=1000;
end;

function TArrayBufIndex.ScrollBackward: TGetResult;
begin
  if FCurrentRecInd>0 then
    begin
    dec(FCurrentRecInd);
    Result := grOK;
    end
  else
    Result := grBOF;
end;

function TArrayBufIndex.ScrollForward: TGetResult;
begin
  if FCurrentRecInd = FLastRecInd-1 then
    result := grEOF
  else
    begin
    Result:=grOK;
    inc(FCurrentRecInd);
    end;
end;

function TArrayBufIndex.GetCurrent: TGetResult;
begin
  if FLastRecInd=0 then
    Result := grError
  else
    begin
    Result := grOK;
    if FCurrentRecInd = FLastRecInd then
      dec(FCurrentRecInd);
    end;
end;

function TArrayBufIndex.ScrollFirst: TGetResult;
begin
  FCurrentRecInd:=0;
  if (FCurrentRecInd = FLastRecInd) then
    result := grEOF
  else
    result := grOk;
end;

procedure TArrayBufIndex.ScrollLast;
begin
  FCurrentRecInd:=FLastRecInd;
end;

procedure TArrayBufIndex.SetToFirstRecord;
begin
<<<<<<< HEAD
  // if FCurrentRecBuf = FLastRecBuf then the dataset is just opened and empty
  // in which case InternalFirst should do nothing (bug 7211)
=======
// if FCurrentRecBuf = FLastRecBuf then the dataset is just opened and empty
// in which case InternalFirst should do nothing (bug 7211)
>>>>>>> graemeg/fixes_2_2
  if FCurrentRecInd <> FLastRecInd then
    FCurrentRecInd := -1;
end;

procedure TArrayBufIndex.SetToLastRecord;
begin
  if FLastRecInd <> 0 then FCurrentRecInd := FLastRecInd;
end;

procedure TArrayBufIndex.StoreCurrentRecord;
begin
  FStoredRecBuf := FCurrentRecInd;
end;

procedure TArrayBufIndex.RestoreCurrentRecord;
begin
  FCurrentRecInd := FStoredRecBuf;
end;

function TArrayBufIndex.CanScrollForward: Boolean;
begin
  Result := (FCurrentRecInd < FLastRecInd-1);
end;

procedure TArrayBufIndex.DoScrollForward;
begin
  inc(FCurrentRecInd);
end;

procedure TArrayBufIndex.StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark);
begin
  with ABookmark^ do
    begin
    BookmarkInt := FCurrentRecInd;
    BookmarkData := FRecordArray[FCurrentRecInd];
    end;
end;

procedure TArrayBufIndex.StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark
  );
begin
  with ABookmark^ do
    begin
    BookmarkInt := FLastRecInd;
    BookmarkData := FRecordArray[FLastRecInd];
    end;
end;

<<<<<<< HEAD
function TArrayBufIndex.GetRecordFromBookmark(ABookmark: TBufBookmark): integer;
begin
  // ABookmark.BookMarkBuf is nil if SetRecNo calls GotoBookmark
  if (ABookmark.BookmarkData<>nil) and (FRecordArray[ABookmark.BookmarkInt]<>ABookmark.BookmarkData) then
    begin
    // Start searching two records before the expected record
    if ABookmark.BookmarkInt > 2 then
      Result := ABookmark.BookmarkInt-2
    else
      Result := 0;

    while (Result<FLastRecInd) do
      begin
      if (FRecordArray[Result] = ABookmark.BookmarkData) then exit;
      inc(Result);
      end;

    Result:=0;
    while (Result<ABookmark.BookmarkInt) do
      begin
      if (FRecordArray[Result] = ABookmark.BookmarkData) then exit;
      inc(Result);
      end;

    DatabaseError(SInvalidBookmark)
    end
  else
    Result := ABookmark.BookmarkInt;
end;

=======
>>>>>>> graemeg/fixes_2_2
procedure TArrayBufIndex.GotoBookmark(const ABookmark : PBufBookmark);
begin
  FCurrentRecInd:=GetRecordFromBookmark(ABookmark^);
end;

procedure TArrayBufIndex.InitialiseIndex;
begin
<<<<<<< HEAD
  //  FRecordArray:=nil;
=======
//  FRecordArray:=nil;
>>>>>>> graemeg/fixes_2_2
  setlength(FRecordArray,FInitialBuffers);
  FCurrentRecInd:=-1;
  FLastRecInd:=-1;
end;

<<<<<<< HEAD
procedure TArrayBufIndex.InitialiseSpareRecord(const ASpareRecord:  TRecordBuffer);
begin
  FLastRecInd := 0;
  // FCurrentRecInd := 0;
=======
procedure TArrayBufIndex.InitialiseSpareRecord(const ASpareRecord: PChar);
begin
  FLastRecInd := 0;
 // FCurrentRecInd := 0;
>>>>>>> graemeg/fixes_2_2
  FRecordArray[0] := ASpareRecord;
end;

procedure TArrayBufIndex.ReleaseSpareRecord;
begin
  SetLength(FRecordArray,FInitialBuffers);
end;

<<<<<<< HEAD
function TArrayBufIndex.GetRecNo: integer;
begin
  Result := FCurrentRecInd+1;
end;

procedure TArrayBufIndex.SetRecNo(ARecNo: Longint);
begin
  FCurrentRecInd := ARecNo-1;
end;

procedure TArrayBufIndex.InsertRecordBeforeCurrentRecord(const ARecord:  TRecordBuffer);
=======
function TArrayBufIndex.GetRecNo(const ABookmark: PBufBookmark): integer;
begin
  Result := GetRecordFromBookmark(ABookmark^)+1;
end;

procedure TArrayBufIndex.RemoveRecordFromIndex(const ABookmark : TBufBookmark);
var ARecordInd : integer;
begin
  ARecordInd:=GetRecordFromBookmark(ABookmark);
  Move(FRecordArray[ARecordInd+1],FRecordArray[ARecordInd],sizeof(Pointer)*(FLastRecInd-ARecordInd));
  dec(FLastRecInd);
end;

procedure TArrayBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: PChar);
>>>>>>> graemeg/fixes_2_2
begin
  inc(FLastRecInd);
  if FLastRecInd >= length(FRecordArray) then
    SetLength(FRecordArray,length(FRecordArray)+FGrowBuffer);

  Move(FRecordArray[FCurrentRecInd],FRecordArray[FCurrentRecInd+1],sizeof(Pointer)*(FLastRecInd-FCurrentRecInd));
  FRecordArray[FCurrentRecInd]:=ARecord;
  inc(FCurrentRecInd);
end;

<<<<<<< HEAD
procedure TArrayBufIndex.RemoveRecordFromIndex(const ABookmark : TBufBookmark);
var ARecordInd : integer;
begin
  ARecordInd:=GetRecordFromBookmark(ABookmark);
  Move(FRecordArray[ARecordInd+1],FRecordArray[ARecordInd],sizeof(Pointer)*(FLastRecInd-ARecordInd));
  dec(FLastRecInd);
end;

<<<<<<< HEAD
procedure TArrayBufIndex.BeginUpdate;
begin
  //  inherited BeginUpdate;
end;

procedure TArrayBufIndex.AddRecord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var ARecord:  TRecordBuffer;
=======
var ARecord: PChar;
>>>>>>> graemeg/cpstrnew
=======
var ARecord: PChar;
>>>>>>> graemeg/cpstrnew
=======
var ARecord: PChar;
>>>>>>> graemeg/cpstrnew
begin
=======
procedure TArrayBufIndex.AddRecord;
var ARecord: PChar;
begin
>>>>>>> origin/cpstrnew
  ARecord := FDataset.IntAllocRecordBuffer;
=======
procedure TArrayBufIndex.BeginUpdate;
begin
//  inherited BeginUpdate;
end;

procedure TArrayBufIndex.AddRecord(const ARecord: PChar);
begin
>>>>>>> graemeg/fixes_2_2
  inc(FLastRecInd);
  if FLastRecInd >= length(FRecordArray) then
    SetLength(FRecordArray,length(FRecordArray)+FGrowBuffer);
  FRecordArray[FLastRecInd]:=ARecord;
end;

procedure TArrayBufIndex.EndUpdate;
begin
<<<<<<< HEAD
  //  inherited EndUpdate;
end;


=======
//  inherited EndUpdate;
end;

>>>>>>> graemeg/fixes_2_2
{ TDataPacketReader }

class function TDataPacketReader.RowStateToByte(const ARowState: TRowState
  ): byte;
var RowStateInt : Byte;
begin
  RowStateInt:=0;
  if rsvOriginal in ARowState then RowStateInt := RowStateInt+1;
  if rsvDeleted in ARowState then RowStateInt := RowStateInt+2;
  if rsvInserted in ARowState then RowStateInt := RowStateInt+4;
  if rsvUpdated in ARowState then RowStateInt := RowStateInt+8;
  Result := RowStateInt;
end;

class function TDataPacketReader.ByteToRowState(const AByte: Byte): TRowState;
begin
  result := [];
  if (AByte and 1)=1 then Result := Result+[rsvOriginal];
  if (AByte and 2)=2 then Result := Result+[rsvDeleted];
  if (AByte and 4)=4 then Result := Result+[rsvInserted];
  if (AByte and 8)=8 then Result := Result+[rsvUpdated];
end;

<<<<<<< HEAD
procedure TDataPacketReader.RestoreBlobField(AField: TField; ASource: pointer; ASize: integer);
var
  ABufBlobField: TBufBlobField;
begin
  ABufBlobField.BlobBuffer:=FDataSet.GetNewBlobBuffer;
  ABufBlobField.BlobBuffer^.Size:=ASize;
  ReAllocMem(ABufBlobField.BlobBuffer^.Buffer, ASize);
  move(ASource^, ABufBlobField.BlobBuffer^.Buffer^, ASize);
  AField.SetData(@ABufBlobField);
end;

constructor TDataPacketReader.Create(ADataSet: TCustomBufDataset; AStream: TStream);
begin
  FDataSet := ADataSet;
  FStream := AStream;
end;


{ TFpcBinaryDatapacketReader }

constructor TFpcBinaryDatapacketReader.Create(ADataSet: TCustomBufDataset; AStream: TStream);
begin
  inherited;
  FVersion := 20; // default version 2.0
end;

procedure TFpcBinaryDatapacketReader.LoadFieldDefs(var AnAutoIncValue: integer);

var FldCount : word;
    i        : integer;
    s        : string;

begin
  // Identify version
  SetLength(s, 13);
  if (Stream.Read(s[1], 13) = 13) then
    case s of
      FpcBinaryIdent1:
        FVersion := 10;
      FpcBinaryIdent2:
        FVersion := Stream.ReadByte;
      else
        DatabaseError(SStreamNotRecognised);
    end;

  // Read FieldDefs
  FldCount := Stream.ReadWord;
  DataSet.FieldDefs.Clear;
  for i := 0 to FldCount - 1 do with DataSet.FieldDefs.AddFieldDef do
=======
constructor TDataPacketReader.create(AStream: TStream);
begin
  FStream := AStream;
end;

{ TFpcBinaryDatapacketReader }

const FpcBinaryIdent = 'BinBufDataset';

procedure TFpcBinaryDatapacketReader.LoadFieldDefs(AFieldDefs: TFieldDefs);

var FldCount : word;
    i        : integer;

begin
  if not RecognizeStream(Stream) then
    DatabaseError(SStreamNotRecognised);

  FldCount:=Stream.ReadWord;
  for i := 0 to FldCount -1 do with TFieldDef.create(AFieldDefs) do
>>>>>>> graemeg/fixes_2_2
    begin
    Name := Stream.ReadAnsiString;
    Displayname := Stream.ReadAnsiString;
    Size := Stream.ReadWord;
    DataType := TFieldType(Stream.ReadWord);

    if Stream.ReadByte = 1 then
      Attributes := Attributes + [faReadonly];
    end;
<<<<<<< HEAD
  Stream.ReadBuffer(i,sizeof(i));
  AnAutoIncValue := i;

  FNullBitmapSize := (FldCount + 7) div 8;
  SetLength(FNullBitmap, FNullBitmapSize);
end;

procedure TFpcBinaryDatapacketReader.StoreFieldDefs(AnAutoIncValue: integer);
var i : integer;
begin
  Stream.Write(FpcBinaryIdent2[1], length(FpcBinaryIdent2));
  Stream.WriteByte(FVersion);

  Stream.WriteWord(DataSet.FieldDefs.Count);
  for i := 0 to DataSet.FieldDefs.Count - 1 do with DataSet.FieldDefs[i] do
    begin
    Stream.WriteAnsiString(Name);
    Stream.WriteAnsiString(DisplayName);
    Stream.WriteWord(Size);
=======
end;

procedure TFpcBinaryDatapacketReader.StoreFieldDefs(AFieldDefs: TFieldDefs);
var i : integer;
begin
  Stream.Write(FpcBinaryIdent[1],length(FpcBinaryIdent));

  Stream.WriteWord(AFieldDefs.Count);
  for i := 0 to AFieldDefs.Count -1 do with AFieldDefs[i] do
    begin
    Stream.WriteAnsiString(Name);
    Stream.WriteAnsiString(DisplayName);
    Stream.WriteWord(size);
>>>>>>> graemeg/fixes_2_2
    Stream.WriteWord(ord(DataType));

    if faReadonly in Attributes then
      Stream.WriteByte(1)
    else
      Stream.WriteByte(0);
    end;
<<<<<<< HEAD
  i := AnAutoIncValue;
  Stream.WriteBuffer(i,sizeof(i));

  FNullBitmapSize := (DataSet.FieldDefs.Count + 7) div 8;
  SetLength(FNullBitmap, FNullBitmapSize);
end;

procedure TFpcBinaryDatapacketReader.InitLoadRecords;
begin
  //  Do nothing
end;

function TFpcBinaryDatapacketReader.GetCurrentRecord: boolean;
var Buf : byte;
begin
  Result := (Stream.Read(Buf,1)=1) and (Buf=$fe);
=======
>>>>>>> graemeg/fixes_2_2
end;

function TFpcBinaryDatapacketReader.GetRecordRowState(out AUpdOrder : Integer) : TRowState;
var Buf : byte;
begin
  Stream.Read(Buf,1);
  Result := ByteToRowState(Buf);
  if Result<>[] then
    Stream.ReadBuffer(AUpdOrder,sizeof(integer))
  else
    AUpdOrder := 0;
end;

<<<<<<< HEAD
procedure TFpcBinaryDatapacketReader.GotoNextRecord;
begin
  //  Do Nothing
end;

procedure TFpcBinaryDatapacketReader.RestoreRecord;
var
  AField: TField;
  i: integer;
  L: cardinal;
  B: TBytes;
begin
  with DataSet do
    case FVersion of
      10:
        Stream.ReadBuffer(GetCurrentBuffer^, FRecordSize);  // Ugly because private members of ADataset are used...
      20:
        begin
        // Restore field's Null bitmap
        Stream.ReadBuffer(FNullBitmap[0], FNullBitmapSize);
        // Restore field's data
        for i:=0 to FieldDefs.Count-1 do
          begin
          AField := Fields.FieldByNumber(FieldDefs[i].FieldNo);
          if AField=nil then continue;
          if GetFieldIsNull(PByte(FNullBitmap), i) then
            AField.SetData(nil)
          else if AField.DataType in StringFieldTypes then
            AField.AsString := Stream.ReadAnsiString
          else
            begin
            if AField.DataType in VarLenFieldTypes then
              L := Stream.ReadDWord
            else
              L := AField.DataSize;
            SetLength(B, L);
            if L > 0 then
              Stream.ReadBuffer(B[0], L);
            if AField.DataType in BlobFieldTypes then
              RestoreBlobField(AField, @B[0], L)
            else
              AField.SetData(@B[0], False);  // set it to the FilterBuffer
            end;
          end;
        end;
    end;
end;

procedure TFpcBinaryDatapacketReader.StoreRecord(ARowState: TRowState; AUpdOrder : integer);
var
  AField: TField;
  i: integer;
  L: cardinal;
  B: TBytes;
begin
  // Record header
  Stream.WriteByte($fe);
  Stream.WriteByte(RowStateToByte(ARowState));
  if ARowState<>[] then
    Stream.WriteBuffer(AUpdOrder,sizeof(integer));

  // Record data
  with DataSet do
    case FVersion of
      10:
        Stream.WriteBuffer(GetCurrentBuffer^, FRecordSize); // Old 1.0 version
      20:
        begin
        // store fields Null bitmap
        FillByte(FNullBitmap[0], FNullBitmapSize, 0);
        for i:=0 to FieldDefs.Count-1 do
          begin
          AField := Fields.FieldByNumber(FieldDefs[i].FieldNo);
          if assigned(AField) and AField.IsNull then
            SetFieldIsNull(PByte(FNullBitmap), i);
          end;
        Stream.WriteBuffer(FNullBitmap[0], FNullBitmapSize);

        for i:=0 to FieldDefs.Count-1 do
          begin
          AField := Fields.FieldByNumber(FieldDefs[i].FieldNo);
          if not assigned(AField) or AField.IsNull then continue;
          if AField.DataType in StringFieldTypes then
            Stream.WriteAnsiString(AField.AsString)
          else
            begin
            B := AField.AsBytes;
            L := length(B);
            if AField.DataType in VarLenFieldTypes then
              Stream.WriteDWord(L);
            if L > 0 then
              Stream.WriteBuffer(B[0], L);
            end;
          end;
        end;
    end;
end;

procedure TFpcBinaryDatapacketReader.FinalizeStoreRecords;
begin
  //  Do nothing
end;

class function TFpcBinaryDatapacketReader.RecognizeStream(AStream: TStream): boolean;
var s : string;
begin
  SetLength(s, 13);
  if (AStream.Read(s[1], 13) = 13) then
    case s of
      FpcBinaryIdent1,
      FpcBinaryIdent2:
        Result := True;
      else
        Result := False;
    end;
end;

{ TUniDirectionalBufIndex }

function TUniDirectionalBufIndex.GetBookmarkSize: integer;
begin
  // In principle there are no bookmarks, and the size should be 0.
  // But there is quite some code in TCustomBufDataset that relies on
  // an existing bookmark of the TBufBookmark type.
  // This code could be moved to the TBufIndex but that would make things
  // more complicated and probably slower. So use a 'fake' bookmark of
  // size TBufBookmark.
  // When there are other TBufIndexes which also need special bookmark code
  // this can be adapted.
  Result:=sizeof(TBufBookmark);
end;

function TUniDirectionalBufIndex.GetCurrentBuffer: Pointer;
begin
  result := FSPareBuffer;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TUniDirectionalBufIndex.GetCurrentRecord:  TRecordBuffer;
=======
procedure TFpcBinaryDatapacketReader.RestoreRecord(ADataset: TCustomBufDataset);
>>>>>>> graemeg/cpstrnew
=======
procedure TFpcBinaryDatapacketReader.RestoreRecord(ADataset: TCustomBufDataset);
>>>>>>> graemeg/cpstrnew
=======
procedure TFpcBinaryDatapacketReader.RestoreRecord(ADataset: TCustomBufDataset);
>>>>>>> graemeg/cpstrnew
=======
procedure TFpcBinaryDatapacketReader.RestoreRecord(ADataset: TCustomBufDataset);
>>>>>>> origin/cpstrnew
begin
  //  Result:=inherited GetCurrentRecord;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TUniDirectionalBufIndex.GetIsInitialized: boolean;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TFpcBinaryDatapacketReader.StoreRecord(ADataset: TCustomBufDataset;
  ARowState: TRowState; AUpdOrder : integer);
>>>>>>> graemeg/cpstrnew
begin
  Result := Assigned(FSPareBuffer);
end;

function TUniDirectionalBufIndex.GetSpareBuffer:  TRecordBuffer;
begin
  result := FSPareBuffer;
end;

function TUniDirectionalBufIndex.GetSpareRecord:  TRecordBuffer;
begin
  result := FSPareBuffer;
end;

function TUniDirectionalBufIndex.ScrollBackward: TGetResult;
begin
  result := grError;
end;

function TUniDirectionalBufIndex.ScrollForward: TGetResult;
begin
  result := grOk;
end;

<<<<<<< HEAD
=======
{ TUniDirectionalBufIndex }

function TUniDirectionalBufIndex.GetBookmarkSize: integer;
begin
  // In principle there are no bookmarks, and the size should be 0.
  // But there is quite some code in TCustomBufDataset that relies on
  // an existing bookmark of the TBufBookmark type.
  // This code could be moved to the TBufIndex but that would make things
  // more complicated and probably slower. So use a 'fake' bookmark of
  // size TBufBookmark.
  // When there are other TBufIndexes which also need special bookmark-code
  // this can be adapted.
  Result:=sizeof(TBufBookmark);
end;

function TUniDirectionalBufIndex.GetCurrentBuffer: Pointer;
begin
  result := FSPareBuffer;
end;

function TUniDirectionalBufIndex.GetCurrentRecord: PChar;
begin
//  Result:=inherited GetCurrentRecord;
end;

function TUniDirectionalBufIndex.GetIsInitialized: boolean;
begin
  Result := Assigned(FSPareBuffer);
end;

<<<<<<< HEAD
function TUniDirectionalBufIndex.GetSpareBuffer: PChar;
begin
  result := FSPareBuffer;
end;

function TUniDirectionalBufIndex.GetSpareRecord: PChar;
begin
  result := FSPareBuffer;
end;

function TUniDirectionalBufIndex.ScrollBackward: TGetResult;
begin
  result := grError;
end;

function TUniDirectionalBufIndex.ScrollForward: TGetResult;
begin
  result := grOk;
end;

>>>>>>> graemeg/cpstrnew
function TUniDirectionalBufIndex.GetCurrent: TGetResult;
begin
  result := grOk;
end;

function TUniDirectionalBufIndex.ScrollFirst: TGetResult;
begin
  Result:=grError;
end;

procedure TUniDirectionalBufIndex.ScrollLast;
begin
  DatabaseError(SUniDirectional);
end;

procedure TUniDirectionalBufIndex.SetToFirstRecord;
begin
<<<<<<< HEAD
  // for UniDirectional datasets should be [Internal]First valid method call
  // do nothing
=======
  DatabaseError(SUniDirectional);
>>>>>>> graemeg/cpstrnew
end;

procedure TUniDirectionalBufIndex.SetToLastRecord;
begin
  DatabaseError(SUniDirectional);
end;

procedure TUniDirectionalBufIndex.StoreCurrentRecord;
begin
  DatabaseError(SUniDirectional);
end;

<<<<<<< HEAD
procedure TUniDirectionalBufIndex.RestoreCurrentRecord;
begin
  DatabaseError(SUniDirectional);
end;

<<<<<<< HEAD
function TUniDirectionalBufIndex.CanScrollForward: Boolean;
begin
<<<<<<< HEAD
  // should return true if next record is already fetched
=======
  // should return true if a next record is already fetched
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ TUniDirectionalBufIndex }

function TUniDirectionalBufIndex.GetBookmarkSize: integer;
begin
  // In principle there are no bookmarks, and the size should be 0.
  // But there is quite some code in TCustomBufDataset that relies on
  // an existing bookmark of the TBufBookmark type.
  // This code could be moved to the TBufIndex but that would make things
  // more complicated and probably slower. So use a 'fake' bookmark of
  // size TBufBookmark.
  // When there are other TBufIndexes which also need special bookmark-code
  // this can be adapted.
  Result:=sizeof(TBufBookmark);
end;

function TUniDirectionalBufIndex.GetCurrentBuffer: Pointer;
begin
  result := FSPareBuffer;
end;

function TUniDirectionalBufIndex.GetCurrentRecord: PChar;
begin
//  Result:=inherited GetCurrentRecord;
end;

function TUniDirectionalBufIndex.GetIsInitialized: boolean;
begin
  Result := Assigned(FSPareBuffer);
end;

function TUniDirectionalBufIndex.GetSpareBuffer: PChar;
begin
  result := FSPareBuffer;
end;

function TUniDirectionalBufIndex.GetSpareRecord: PChar;
begin
  result := FSPareBuffer;
end;

function TUniDirectionalBufIndex.ScrollBackward: TGetResult;
begin
  result := grError;
end;

function TUniDirectionalBufIndex.ScrollForward: TGetResult;
begin
  result := grOk;
end;

function TUniDirectionalBufIndex.GetCurrent: TGetResult;
begin
  result := grOk;
end;

function TUniDirectionalBufIndex.ScrollFirst: TGetResult;
begin
  Result:=grError;
end;

procedure TUniDirectionalBufIndex.ScrollLast;
begin
  DatabaseError(SUniDirectional);
end;

procedure TUniDirectionalBufIndex.SetToFirstRecord;
begin
  DatabaseError(SUniDirectional);
end;

procedure TUniDirectionalBufIndex.SetToLastRecord;
begin
  DatabaseError(SUniDirectional);
end;

procedure TUniDirectionalBufIndex.StoreCurrentRecord;
begin
  DatabaseError(SUniDirectional);
end;

procedure TUniDirectionalBufIndex.RestoreCurrentRecord;
begin
  DatabaseError(SUniDirectional);
end;

function TUniDirectionalBufIndex.CanScrollForward: Boolean;
begin
  // should return true if a next record is already fetched
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  result := false;
end;

procedure TUniDirectionalBufIndex.DoScrollForward;
begin
  // do nothing
end;

procedure TUniDirectionalBufIndex.StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark);
begin
  // do nothing
end;

procedure TUniDirectionalBufIndex.StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark);
begin
  // do nothing
end;

procedure TUniDirectionalBufIndex.GotoBookmark(const ABookmark: PBufBookmark);
begin
  DatabaseError(SUniDirectional);
end;

procedure TUniDirectionalBufIndex.InitialiseIndex;
begin
  // do nothing
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TUniDirectionalBufIndex.InitialiseSpareRecord(const ASpareRecord:  TRecordBuffer);
=======
procedure TUniDirectionalBufIndex.InitialiseSpareRecord(const ASpareRecord: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TUniDirectionalBufIndex.InitialiseSpareRecord(const ASpareRecord: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TUniDirectionalBufIndex.InitialiseSpareRecord(const ASpareRecord: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TUniDirectionalBufIndex.InitialiseSpareRecord(const ASpareRecord: PChar);
>>>>>>> origin/cpstrnew
begin
  FSPareBuffer:=ASpareRecord;
end;

procedure TUniDirectionalBufIndex.ReleaseSpareRecord;
begin
  FSPareBuffer:=nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TUniDirectionalBufIndex.GetRecNo: Longint;
begin
  Result := -1;
end;

procedure TUniDirectionalBufIndex.SetRecNo(ARecNo: Longint);
begin
  DatabaseError(SUniDirectional);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TUniDirectionalBufIndex.RemoveRecordFromIndex(const ABookmark: TBufBookmark);
begin
  DatabaseError(SUniDirectional);
end;

function TUniDirectionalBufIndex.GetRecNo(const ABookmark: PBufBookmark): integer;
begin
  result := -1;
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
end;

procedure TUniDirectionalBufIndex.BeginUpdate;
begin
  // Do nothing
end;

procedure TUniDirectionalBufIndex.AddRecord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  h,i: integer;
begin
  // Release unneeded blob buffers, in order to save memory
  // TDataSet has own buffer of records, so do not release blobs until they can be referenced
  with FDataSet do
    begin
    h := high(FBlobBuffers) - BufferCount*BlobFieldCount;
    if h > 10 then //Free in batches, starting with oldest (at beginning)
      begin
      for i := 0 to h do
        FreeBlobBuffer(FBlobBuffers[i]);
      FBlobBuffers := Copy(FBlobBuffers, h+1, high(FBlobBuffers)-h);
      end;
    end;
end;

procedure TUniDirectionalBufIndex.InsertRecordBeforeCurrentRecord(const ARecord:  TRecordBuffer);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  // Do nothing
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TUniDirectionalBufIndex.RemoveRecordFromIndex(const ABookmark: TBufBookmark);
begin
  DatabaseError(SUniDirectional);
end;

procedure TUniDirectionalBufIndex.OrderCurrentRecord;
=======
procedure TUniDirectionalBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TUniDirectionalBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TUniDirectionalBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: PChar);
>>>>>>> graemeg/cpstrnew
=======
procedure TUniDirectionalBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: PChar);
>>>>>>> origin/cpstrnew
begin
  // Do nothing
end;

procedure TUniDirectionalBufIndex.EndUpdate;
begin
  // Do nothing
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
procedure unSetFieldIsNull(NullMask : pbyte;x : longint); //inline;
begin
  NullMask[x div 8] := (NullMask[x div 8]) and not (1 shl (x mod 8));
end;

procedure SetFieldIsNull(NullMask : pbyte;x : longint); //inline;
begin
  NullMask[x div 8] := (NullMask[x div 8]) or (1 shl (x mod 8));
end;

function GetFieldIsNull(NullMask : pbyte;x : longint) : boolean; //inline;
begin
  result := ord(NullMask[x div 8]) and (1 shl (x mod 8)) > 0
end;

{ ---------------------------------------------------------------------
    TBufDataSet
  ---------------------------------------------------------------------}

constructor TBufDataset.Create(AOwner : TComponent);
begin
  Inherited Create(AOwner);
  FMaxIndexesCount:=2;
  FIndexesCount:=0;
  InternalAddIndex('DEFAULT_ORDER','',[],'','');
  FCurrentIndex:=FIndexes[0];
  InternalAddIndex('','',[],'','');

  FIndexDefs := TIndexDefs.Create(Self);

  SetLength(FUpdateBuffer,0);
  SetLength(FBlobBuffers,0);
  SetLength(FUpdateBlobBuffers,0);
  BookmarkSize := FCurrentIndex.BookmarkSize;
  FParser := nil;
  FPacketRecords := 10;
end;

procedure TBufDataset.SetPacketRecords(aValue : integer);
begin
  if (aValue = -1) or (aValue > 0) then FPacketRecords := aValue
    else DatabaseError(SInvPacketRecordsValue);
end;

destructor TBufDataset.Destroy;

Var
  I : Integer;
begin
  if Active then Close;
  SetLength(FUpdateBuffer,0);
  SetLength(FBlobBuffers,0);
  SetLength(FUpdateBlobBuffers,0);
  For I:=0 to Length(FIndexes)-1 do
    FreeAndNil(Findexes[I]);
  SetLength(FIndexes,0);
  FreeAndNil(FIndexDefs);
  inherited destroy;
end;

procedure TBufDataset.FetchAll;
begin
  repeat
  until (getnextpacket < FPacketRecords) or (FPacketRecords = -1);
end;

procedure TBufDataset.BuildIndex(var AIndex: TBufIndex);

var PCurRecLinkItem : PBufRecLinkItem;
    p,l,q           : PBufRecLinkItem;
    i,k,psize,qsize : integer;
    MergeAmount     : integer;
    PlaceQRec       : boolean;

    IndexFields     : TList;
    DescIndexFields : TList;
    CInsIndexFields : TList;
    FieldsAmount    : Integer;
    FieldNr         : integer;
    AField          : TField;

  procedure PlaceNewRec(var e: PBufRecLinkItem; var esize: integer);
  begin
    if (AIndex as TDoubleLinkedBufIndex).FFirstRecBuf=nil then
     begin
     (AIndex as TDoubleLinkedBufIndex).FFirstRecBuf:=e;
     e[(AIndex as TDoubleLinkedBufIndex).IndNr].prior:=nil;
     l:=e;
     end
   else
     begin
     l[(AIndex as TDoubleLinkedBufIndex).IndNr].next:=e;
     e[(AIndex as TDoubleLinkedBufIndex).IndNr].prior:=l;
     l:=e;
     end;
   e := e[(AIndex as TDoubleLinkedBufIndex).IndNr].next;
   dec(esize);
  end;

begin
  // Build the DBCompareStructure
  with AIndex do
    begin
    IndexFields := TList.Create;
    DescIndexFields := TList.Create;
    CInsIndexFields := TList.Create;
    try
      GetFieldList(IndexFields,FieldsName);
      FieldsAmount:=IndexFields.Count;
      GetFieldList(DescIndexFields,DescFields);
      GetFieldList(CInsIndexFields,CaseinsFields);
      if FieldsAmount=0 then
        DatabaseError(SNoIndexFieldNameGiven);
      SetLength(DBCompareStruct,FieldsAmount);
      for FieldNr:=0 to FieldsAmount-1 do
        begin
        AField := TField(IndexFields[FieldNr]);
        ProcessFieldCompareStruct(AField,DBCompareStruct[FieldNr]);

        DBCompareStruct[FieldNr].Desc := (DescIndexFields.IndexOf(AField)>-1);
        if (CInsIndexFields.IndexOf(AField)>-1) then
          DBCompareStruct[FieldNr].Options := [loCaseInsensitive]
        else
          DBCompareStruct[FieldNr].Options := [];

        end;
    finally
      CInsIndexFields.Free;
      DescIndexFields.Free;
      IndexFields.Free;
    end;
    end;

// This simply copies the index...
  PCurRecLinkItem:=(FIndexes[0] as TDoubleLinkedBufIndex).FFirstRecBuf;
  PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].next := PCurRecLinkItem[0].next;
  PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].prior := PCurRecLinkItem[0].prior;

  if PCurRecLinkItem <> (FIndexes[0] as TDoubleLinkedBufIndex).FLastRecBuf then
    begin
    while PCurRecLinkItem^.next<>(FIndexes[0] as TDoubleLinkedBufIndex).FLastRecBuf do
      begin
      PCurRecLinkItem:=PCurRecLinkItem^.next;

      PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].next := PCurRecLinkItem[0].next;
      PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].prior := PCurRecLinkItem[0].prior;
      end;
    end;

// Set FirstRecBuf and FCurrentRecBuf
  (AIndex as TDoubleLinkedBufIndex).FFirstRecBuf:=(FIndexes[0] as TDoubleLinkedBufIndex).FFirstRecBuf;
  (FCurrentIndex as TDoubleLinkedBufIndex).FCurrentRecBuf:=(AIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
// Link in the FLastRecBuf that belongs to this index
  PCurRecLinkItem[(AIndex as TDoubleLinkedBufIndex).IndNr].next:=(AIndex as TDoubleLinkedBufIndex).FLastRecBuf;
  (AIndex as TDoubleLinkedBufIndex).FLastRecBuf[(AIndex as TDoubleLinkedBufIndex).IndNr].prior:=PCurRecLinkItem;

// Mergesort. Used the algorithm as described here by Simon Tatham
// http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
// The comments in the code are from this website.

// In each pass, we are merging lists of size K into lists of size 2K.
// (Initially K equals 1.)
  k:=1;

  repeat

// So we start by pointing a temporary pointer p at the head of the list,
// and also preparing an empty list L which we will add elements to the end
// of as we finish dealing with them.

  p := (AIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  (AIndex as TDoubleLinkedBufIndex).ffirstRecBuf := nil;
  q := p;
  MergeAmount := 0;

// Then:
//    * If p is null, terminate this pass.
  while p <> (AIndex as TDoubleLinkedBufIndex).FLastRecBuf do
    begin

//    * Otherwise, there is at least one element in the next pair of length-K
//      lists, so increment the number of merges performed in this pass.

    inc(MergeAmount);

//    * Point another temporary pointer, q, at the same place as p. Step q along
//      the list by K places, or until the end of the list, whichever comes
//      first. Let psize be the number of elements you managed to step q past.

    i:=0;
    while (i<k) and (q<>(AIndex as TDoubleLinkedBufIndex).FLastRecBuf) do
      begin
      inc(i);
      q := q[(AIndex as TDoubleLinkedBufIndex).IndNr].next;
      end;
    psize :=i;

//    * Let qsize equal K. Now we need to merge a list starting at p, of length
//      psize, with a list starting at q of length at most qsize.

    qsize:=k;

//    * So, as long as either the p-list is non-empty (psize > 0) or the q-list
//      is non-empty (qsize > 0 and q points to something non-null):

    while (psize>0) or ((qsize>0) and (q <> (AIndex as TDoubleLinkedBufIndex).FLastRecBuf)) do
      begin
//          o Choose which list to take the next element from. If either list
//            is empty, we must choose from the other one. (By assumption, at
//            least one is non-empty at this point.) If both lists are
//            non-empty, compare the first element of each and choose the lower
//            one. If the first elements compare equal, choose from the p-list.
//            (This ensures that any two elements which compare equal are never
//            swapped, so stability is guaranteed.)
      if (psize=0)  then
        PlaceQRec := true
      else if (qsize=0) or (q = (AIndex as TDoubleLinkedBufIndex).FLastRecBuf) then
        PlaceQRec := False
      else if IndexCompareRecords(p,q,aindex.DBCompareStruct) <= 0 then
        PlaceQRec := False
      else
        PlaceQRec := True;
        
//          o Remove that element, e, from the start of its list, by advancing
//            p or q to the next element along, and decrementing psize or qsize.
//          o Add e to the end of the list L we are building up.
      if PlaceQRec then
        PlaceNewRec(q,qsize)
      else
        PlaceNewRec(p,psize);
      end;
//    * Now we have advanced p until it is where q started out, and we have
//      advanced q until it is pointing at the next pair of length-K lists to
//      merge. So set p to the value of q, and go back to the start of this loop.
    p:=q;
    end;

// As soon as a pass like this is performed and only needs to do one merge, the
// algorithm terminates, and the output list L is sorted. Otherwise, double the
// value of K, and go back to the beginning.

  l[(AIndex as TDoubleLinkedBufIndex).IndNr].next:=(AIndex as TDoubleLinkedBufIndex).FLastRecBuf;

  k:=k*2;

  until MergeAmount = 1;
  (AIndex as TDoubleLinkedBufIndex).FLastRecBuf[(AIndex as TDoubleLinkedBufIndex).IndNr].next:=(AIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  (AIndex as TDoubleLinkedBufIndex).FLastRecBuf[(AIndex as TDoubleLinkedBufIndex).IndNr].prior:=l;
end;

function TBufDataset.GetIndexDefs : TIndexDefs;

begin
  Result := FIndexDefs;
end;

procedure TBufDataset.UpdateIndexDefs;
var i : integer;
begin
  FIndexDefs.Clear;
  for i := 0 to high(FIndexes) do with FIndexDefs.AddIndexDef do
    begin
    Name := FIndexes[i].Name;
    Fields := FIndexes[i].FieldsName;
    DescFields:= FIndexes[i].DescFields;
    CaseInsFields:=FIndexes[i].CaseinsFields;
    Options:=FIndexes[i].Options;
    end;
end;

Function TBufDataset.GetCanModify: Boolean;
begin
  Result:= True;
end;

function TBufDataset.intAllocRecordBuffer: PChar;
begin
  // Note: Only the internal buffers of TDataset provide bookmark information
  result := AllocMem(FRecordsize+sizeof(TBufRecLinkItem)*FMaxIndexesCount);
end;

function TBufDataset.AllocRecordBuffer: PChar;
begin
  result := AllocMem(FRecordsize + BookmarkSize + CalcfieldsSize);
// The records are initialised, or else the fields of an empty, just-opened dataset
// are not null
  InitRecord(result);
end;

procedure TBufDataset.FreeRecordBuffer(var Buffer: PChar);
begin
  ReAllocMem(Buffer,0);
end;

procedure TBufDataset.ClearCalcFields(Buffer: PChar);
begin
  if CalcFieldsSize > 0 then
    FillByte((Buffer+RecordSize)^,CalcFieldsSize,0);
end;

procedure TBufDataset.InternalOpen;

var IndexNr : integer;

begin
  if not Assigned(FDatasetReader) and (FileName<>'') then
    begin
    FFileStream := TFileStream.Create(FileName,fmOpenRead);
    FDatasetReader := TFpcBinaryDatapacketReader.Create(FFileStream);
    end;
  if assigned(FDatasetReader) then IntLoadFielddefsFromFile;
  CalcRecordSize;

  FBRecordcount := 0;

  for IndexNr:=0 to FIndexesCount-1 do with FIndexes[IndexNr] do
    InitialiseSpareRecord(IntAllocRecordBuffer);

  FAllPacketsFetched := False;

  FOpen:=True;

  // parse filter expression
  try
    ParseFilter(Filter);
  except
    // oops, a problem with parsing, clear filter for now
    on E: Exception do Filter := EmptyStr;
  end;

  if assigned(FDatasetReader) then IntLoadRecordsFromFile;
end;

procedure TBufDataset.InternalClose;

var r  : integer;
    iGetResult : TGetResult;
    pc : pchar;

begin
  FOpen:=False;
  with FIndexes[0] do if IsInitialized then
    begin
    iGetResult:=ScrollFirst;
    while iGetResult = grOK do
      begin
      pc := pointer(CurrentRecord);
      iGetResult:=ScrollForward;
      FreeRecordBuffer(pc);
      end;
    end;

  for r := 0 to FIndexesCount-1 do with FIndexes[r] do if IsInitialized then
    begin
    pc := SpareRecord;
    ReleaseSpareRecord;
    FreeRecordBuffer(pc);
    end;

  if Length(FUpdateBuffer) > 0 then
    begin
    for r := 0 to length(FUpdateBuffer)-1 do with FUpdateBuffer[r] do
      begin
      if assigned(OldValuesBuffer) then
        FreeRecordBuffer(OldValuesBuffer);
      end;
    end;
  SetLength(FUpdateBuffer,0);
  
  for r := 0 to High(FBlobBuffers) do
    FreeBlobBuffer(FBlobBuffers[r]);
  for r := 0 to High(FUpdateBlobBuffers) do
    FreeBlobBuffer(FUpdateBlobBuffers[r]);

  SetLength(FBlobBuffers,0);
  SetLength(FUpdateBlobBuffers,0);

  SetLength(FFieldBufPositions,0);

  if assigned(FParser) then FreeAndNil(FParser);
end;

procedure TBufDataset.InternalFirst;
begin
  with FCurrentIndex do
    begin
// if FCurrentRecBuf = FLastRecBuf then the dataset is just opened and empty
// in which case InternalFirst should do nothing (bug 7211)
    SetToFirstRecord;
    end;
end;

procedure TBufDataset.InternalLast;
begin
  FetchAll;
  with FCurrentIndex do
  SetToLastRecord;
end;

function TDoubleLinkedBufIndex.GetCurrentRecord: PChar;
begin
  Result := pchar(FCurrentRecBuf);
end;

function TDoubleLinkedBufIndex.GetBookmarkSize: integer;
begin
  Result:=sizeof(TBufBookmark);
end;

function TDoubleLinkedBufIndex.GetCurrentBuffer: Pointer;
begin
  Result := pointer(FCurrentRecBuf)+(sizeof(TBufRecLinkItem)*FDataset.MaxIndexesCount);
end;

function TDoubleLinkedBufIndex.GetIsInitialized: boolean;
begin
  Result := (FFirstRecBuf<>nil);
end;

function TDoubleLinkedBufIndex.GetSpareBuffer: PChar;
begin
  Result := pointer(FLastRecBuf)+(sizeof(TBufRecLinkItem)*FDataset.MaxIndexesCount);
end;

function TDoubleLinkedBufIndex.GetSpareRecord: PChar;
begin
  Result := pchar(FLastRecBuf);
end;

constructor TBufIndex.Create(const ADataset: TBufDataset);
begin
  inherited create;
  FDataset := ADataset;
end;

function TBufIndex.BookmarkValid(const ABookmark: PBufBookmark): boolean;
begin
  Result := assigned(ABookmark) and assigned(ABookmark^.BookmarkData);
end;

function TBufIndex.CompareBookmarks(const ABookmark1, ABookmark2: PBufBookmark): boolean;
begin
  result := (ABookmark1^.BookmarkData=ABookmark2^.BookmarkData);
end;

function TDoubleLinkedBufIndex.ScrollBackward: TGetResult;
begin
  if not assigned(FCurrentRecBuf[IndNr].prior) then
    begin
    Result := grBOF;
    end
  else
    begin
    Result := grOK;
    FCurrentRecBuf := FCurrentRecBuf[IndNr].prior;
    end;
end;

function TDoubleLinkedBufIndex.ScrollForward: TGetResult;
begin
  if (FCurrentRecBuf = FLastRecBuf) or // just opened
     (FCurrentRecBuf[IndNr].next = FLastRecBuf) then
    result := grEOF
  else
    begin
    FCurrentRecBuf := FCurrentRecBuf[IndNr].next;
    Result := grOK;
    end;
end;

function TDoubleLinkedBufIndex.GetCurrent: TGetResult;
begin
  if FFirstRecBuf = FLastRecBuf then
    Result := grError
  else
    begin
    Result := grOK;
    if FCurrentRecBuf = FLastRecBuf then
      FCurrentRecBuf:=FLastRecBuf[IndNr].prior;
    end;
end;

function TDoubleLinkedBufIndex.ScrollFirst: TGetResult;
begin
  FCurrentRecBuf:=FFirstRecBuf;
  if (FCurrentRecBuf = FLastRecBuf) then
    result := grEOF
  else
    result := grOK;
end;

procedure TDoubleLinkedBufIndex.ScrollLast;
begin
  FCurrentRecBuf:=FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.SetToFirstRecord;
begin
  FLastRecBuf[IndNr].next:=FFirstRecBuf;
  FCurrentRecBuf := FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.SetToLastRecord;
begin
  if FLastRecBuf <> FFirstRecBuf then FCurrentRecBuf := FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.StoreCurrentRecord;
begin
  FStoredRecBuf:=FCurrentRecBuf;
end;

procedure TDoubleLinkedBufIndex.RestoreCurrentRecord;
begin
  FCurrentRecBuf:=FStoredRecBuf;
end;

procedure TDoubleLinkedBufIndex.DoScrollForward;
begin
  FCurrentRecBuf := FCurrentRecBuf[IndNr].next;
end;

procedure TDoubleLinkedBufIndex.StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark);
begin
  ABookmark^.BookmarkData:=FCurrentRecBuf;
end;

procedure TDoubleLinkedBufIndex.StoreSpareRecIntoBookmark(
  const ABookmark: PBufBookmark);
begin
  ABookmark^.BookmarkData:=FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.GotoBookmark(const ABookmark : PBufBookmark);
begin
  FCurrentRecBuf := ABookmark^.BookmarkData;
end;

procedure TDoubleLinkedBufIndex.InitialiseIndex;
begin
// Do nothing
end;

function TDoubleLinkedBufIndex.CanScrollForward: Boolean;
begin
  if (FCurrentRecBuf[IndNr].next = FLastRecBuf) then
    Result := False
  else
    Result := True;
end;

procedure TDoubleLinkedBufIndex.InitialiseSpareRecord(const ASpareRecord : PChar);
begin
  FFirstRecBuf := pointer(ASpareRecord);
  FLastRecBuf := FFirstRecBuf;
  FLastRecBuf[IndNr].next:=FLastRecBuf;
  FCurrentRecBuf := FLastRecBuf;
end;

procedure TDoubleLinkedBufIndex.ReleaseSpareRecord;
begin
  FFirstRecBuf:= nil;
end;

procedure TDoubleLinkedBufIndex.RemoveRecordFromIndex(const ABookmark : TBufBookmark);
var ARecord : PBufRecLinkItem;
begin
  ARecord := ABookmark.BookmarkData;
  if ARecord = FCurrentRecBuf then DoScrollForward;
  if ARecord <> FFirstRecBuf then
    ARecord[IndNr].prior[IndNr].next := ARecord[IndNr].next
  else
    begin
    FFirstRecBuf := ARecord[IndNr].next;
    FLastRecBuf[IndNr].next := FFirstRecBuf;
    end;
  ARecord[IndNr].next[IndNr].prior := ARecord[IndNr].prior;
end;

function TDoubleLinkedBufIndex.GetRecNo(const ABookmark: PBufBookmark): integer;
Var TmpRecBuffer    : PBufRecLinkItem;
    recnr           : integer;
begin
  TmpRecBuffer := FFirstRecBuf;
  recnr := 1;
  while TmpRecBuffer <> ABookmark^.BookmarkData do
    begin
    inc(recnr);
    TmpRecBuffer := TmpRecBuffer^.next;
    end;
  Result := recnr;
end;

procedure TDoubleLinkedBufIndex.BeginUpdate;
begin
  if FCurrentRecBuf = FLastRecBuf then
    FCursOnFirstRec := True
  else
    FCursOnFirstRec := False;
end;

procedure TDoubleLinkedBufIndex.AddRecord(Const ARecord : PChar);
begin
  FLastRecBuf[IndNr].next := pointer(ARecord);
  FLastRecBuf[IndNr].next[IndNr].prior := FLastRecBuf;

  FLastRecBuf := FLastRecBuf[IndNr].next;
end;

procedure TDoubleLinkedBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: PChar);
var ANewRecord : PBufRecLinkItem;
begin
  ANewRecord:=PBufRecLinkItem(ARecord);
  ANewRecord[IndNr].prior:=FCurrentRecBuf[IndNr].prior;
  ANewRecord[IndNr].Next:=FCurrentRecBuf;

  if FCurrentRecBuf=FFirstRecBuf then
    begin
    FFirstRecBuf:=ANewRecord;
    ANewRecord[IndNr].prior:=nil;
    end
  else
    ANewRecord[IndNr].Prior[IndNr].next:=ANewRecord;
  ANewRecord[IndNr].next[IndNr].prior:=ANewRecord;
end;

procedure TDoubleLinkedBufIndex.EndUpdate;
begin
  FLastRecBuf[IndNr].next := FFirstRecBuf;
  if FCursOnFirstRec then FCurrentRecBuf:=FLastRecBuf;
end;

procedure TBufDataset.CurrentRecordToBuffer(Buffer: PChar);
var ABookMark : PBufBookmark;
begin
  with FCurrentIndex do
    begin
    move(CurrentBuffer^,buffer^,FRecordSize);
    ABookMark:=PBufBookmark(Buffer + FRecordSize);
    ABookmark^.BookmarkFlag:=bfCurrent;
    StoreCurrentRecIntoBookmark(ABookMark);
    end;

  GetCalcFields(Buffer);
end;

function TBufDataset.GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult;

var Acceptable : Boolean;
    SaveState : TDataSetState;

begin
  Result := grOK;
  with FCurrentIndex do
    begin
    repeat
    Acceptable := True;
    case GetMode of
      gmPrior : Result := ScrollBackward;
      gmCurrent : Result := GetCurrent;
      gmNext : begin
               if not CanScrollForward and (getnextpacket = 0) then result := grEOF
               else
                 begin
                 result := grOK;
                 DoScrollForward;
                 end;
               end;
    end;

    if Result = grOK then
      begin
      CurrentRecordToBuffer(buffer);

      if Filtered then
        begin
        FFilterBuffer := Buffer;
        SaveState := SetTempState(dsFilter);
        DoFilterRecord(Acceptable);
        if (GetMode = gmCurrent) and not Acceptable then
          begin
          Acceptable := True;
          Result := grError;
          end;
        RestoreState(SaveState);
        end;
      end
    else if (Result = grError) and doCheck then
      DatabaseError('No record');
    until Acceptable;
  end;
end;

procedure TBufDataset.DoBeforeClose;
begin
  inherited DoBeforeClose;
  if FFileName<>'' then
    SaveToFile(FFileName);
end;

function TBufDataset.GetActiveRecordUpdateBuffer : boolean;

var ABookmark : TBufBookmark;

begin
  GetBookmarkData(ActiveBuffer,@ABookmark);
  result := GetRecordUpdateBufferCached(ABookmark);
end;

procedure TBufDataset.ProcessFieldCompareStruct(AField: TField; var ACompareRec : TDBCompareRec);
begin
  case AField.DataType of
    ftString : ACompareRec.Comparefunc := @DBCompareText;
    ftSmallint : ACompareRec.Comparefunc := @DBCompareSmallInt;
    ftInteger, ftBCD : ACompareRec.Comparefunc :=
      @DBCompareInt;
    ftWord : ACompareRec.Comparefunc := @DBCompareWord;
    ftBoolean : ACompareRec.Comparefunc := @DBCompareByte;
    ftFloat, ftCurrency : ACompareRec.Comparefunc := @DBCompareDouble;
    ftDateTime, ftDate, ftTime : ACompareRec.Comparefunc :=
      @DBCompareDouble;
    ftLargeint : ACompareRec.Comparefunc := @DBCompareLargeInt;
  else
    DatabaseErrorFmt(SErrIndexBasedOnInvField, [Fieldtypenames[AField.DataType]]);
  end;

  ACompareRec.Off1:=sizeof(TBufRecLinkItem)*FMaxIndexesCount+
    FFieldBufPositions[AField.FieldNo-1];
  ACompareRec.Off2:=ACompareRec.Off1;
end;

procedure TBufDataset.SetIndexFieldNames(const AValue: String);
begin
  if AValue<>'' then
    begin
    FIndexes[1].FieldsName:=AValue;
    FCurrentIndex:=FIndexes[1];
    if active then
      begin
      FetchAll;
      BuildIndex(FIndexes[1]);
      Resync([rmCenter]);
      end;
    end
  else
    SetIndexName('');
end;

procedure TBufDataset.SetIndexName(AValue: String);
var i : integer;
begin
  if AValue='' then AValue := 'DEFAULT_ORDER';
  for i := 0 to FIndexesCount-1 do
    if SameText(FIndexes[i].Name,AValue) then
      begin
      (FIndexes[i] as TDoubleLinkedBufIndex).FCurrentRecBuf:=(FCurrentIndex as TDoubleLinkedBufIndex).FCurrentRecBuf;
      FCurrentIndex:=FIndexes[i];
      if active then Resync([rmCenter]);
      exit;
      end;
end;

procedure TBufDataset.SetMaxIndexesCount(const AValue: Integer);
begin
  CheckInactive;
  if AValue > 1 then
    FMaxIndexesCount:=AValue
  else
    DatabaseError(SMinIndexes);
end;

procedure TBufDataset.InternalSetToRecord(Buffer: PChar);
begin
  FCurrentIndex.GotoBookmark(PBufBookmark(Buffer+FRecordSize));
end;

procedure TBufDataset.SetBookmarkData(Buffer: PChar; Data: Pointer);
begin
  PBufBookmark(Buffer + FRecordSize)^.BookmarkData := pointer(Data^);
end;

procedure TBufDataset.SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag);
begin
  PBufBookmark(Buffer + FRecordSize)^.BookmarkFlag := Value;
end;

procedure TBufDataset.GetBookmarkData(Buffer: PChar; Data: Pointer);
begin
  pointer(Data^) := PBufBookmark(Buffer + FRecordSize)^.BookmarkData;
end;

function TBufDataset.GetBookmarkFlag(Buffer: PChar): TBookmarkFlag;
begin
  Result := PBufBookmark(Buffer + FRecordSize)^.BookmarkFlag;
end;

procedure TBufDataset.InternalGotoBookmark(ABookmark: Pointer);
begin
  // note that ABookMark should be a PBufBookmark. But this way it can also be
  // a pointer to a TBufRecLinkItem
  FCurrentIndex.GotoBookmark(ABookmark);
end;

function TBufDataset.getnextpacket : integer;

var i : integer;
    pb : pchar;

begin
  if FAllPacketsFetched then
    begin
    result := 0;
    exit;
    end;

  FCurrentIndex.BeginUpdate;

  i := 0;
  pb := FIndexes[0].SpareBuffer;
  while ((i < FPacketRecords) or (FPacketRecords = -1)) and (loadbuffer(pb) = grOk) do
    begin
    with FIndexes[0] do
      begin
      AddRecord(IntAllocRecordBuffer);
      pb := SpareBuffer;
      end;
    inc(i);
    end;

  FCurrentIndex.EndUpdate;
  FBRecordCount := FBRecordCount + i;
  result := i;
end;

function TBufDataset.GetFieldSize(FieldDef : TFieldDef) : longint;

begin
  case FieldDef.DataType of
    ftString,
      ftGuid,
      ftFixedChar: result := FieldDef.Size + 1;
    ftFixedWideChar,
      ftWideString:result := (FieldDef.Size + 1)*2;
    ftSmallint,
      ftInteger,
      ftword     : result := sizeof(longint);
    ftBoolean    : result := sizeof(wordbool);
    ftBCD        : result := sizeof(currency);
    ftFloat,
      ftCurrency : result := sizeof(double);
    ftLargeInt   : result := sizeof(largeint);
    ftTime,
      ftDate,
      ftDateTime : result := sizeof(TDateTime);
    ftBlob,
      ftMemo,
      ftGraphic,
      ftFmtMemo,
      ftParadoxOle,
      ftDBaseOle,
      ftTypedBinary,
      ftOraBlob,
      ftOraClob,
      ftWideMemo : result := sizeof(TBufBlobField)
  else Result := 10
  end;
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
  result:=Align(result,4);
{$ENDIF}
end;

function TBufDataset.GetRecordUpdateBuffer(const ABookmark : TBufBookmark; IncludePrior : boolean = false; AFindNext : boolean = false): boolean;

var x        : integer;
    StartBuf : integer;

begin
  if AFindNext then
    StartBuf:=FCurrentUpdateBuffer+1
  else
    StartBuf := 0;
  Result := False;
  for x := StartBuf to high(FUpdateBuffer) do
   if FCurrentIndex.CompareBookmarks(@FUpdateBuffer[x].BookmarkData,@ABookmark) or
      (IncludePrior and (FUpdateBuffer[x].UpdateKind=ukDelete) and FCurrentIndex.CompareBookmarks(@FUpdateBuffer[x].NextBookmarkData,@ABookmark)) then
    begin
    FCurrentUpdateBuffer := x;
    Result := True;
    break;
    end;
end;

function TBufDataset.GetRecordUpdateBufferCached(const ABookmark: TBufBookmark;
  IncludePrior: boolean): boolean;
begin
  // if the current update buffer complies, immediately return true
  if (FCurrentUpdateBuffer < length(FUpdateBuffer)) and (
      FCurrentIndex.CompareBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData,@ABookmark) or
      (IncludePrior and (FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind=ukDelete) and FCurrentIndex.CompareBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData,@ABookmark))) then
    Result := True
  else
    Result := GetRecordUpdateBuffer(ABookmark,IncludePrior);
end;

function TBufDataset.LoadBuffer(Buffer : PChar): TGetResult;

var NullMask        : pbyte;
    x               : longint;
    CreateblobField : boolean;
    BufBlob         : PBufBlobField;

begin
  if not Fetch then
    begin
    Result := grEOF;
    FAllPacketsFetched := True;
    // This code has to be placed elsewhere. At least it should also run when
    // the datapacket is loaded from file
    if FIndexesCount>0 then for x := 1 to FIndexesCount-1 do
      begin
      if not ((x=1) and (FIndexes[1].FieldsName='')) then
        begin
        BuildIndex(FIndexes[x]);
        (FCurrentIndex as TDoubleLinkedBufIndex).FCurrentRecBuf:=(FCurrentIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
        end;
      end;
    Exit;
    end;

  NullMask := pointer(buffer);
  fillchar(Nullmask^,FNullmaskSize,0);
  inc(buffer,FNullmaskSize);

  for x := 0 to FieldDefs.count-1 do
    begin
    if not LoadField(FieldDefs[x],buffer,CreateblobField) then
      SetFieldIsNull(NullMask,x)
    else if CreateblobField then
      begin
      BufBlob := PBufBlobField(Buffer);
      BufBlob^.BlobBuffer := GetNewBlobBuffer;
      LoadBlobIntoBuffer(FieldDefs[x],BufBlob);
      end;
    inc(buffer,GetFieldSize(FieldDefs[x]));
    end;
  Result := grOK;
end;

function TBufDataset.GetCurrentBuffer: PChar;
begin
  if State = dsFilter then Result := FFilterBuffer
  else if state = dsCalcFields then Result := CalcBuffer
  else Result := ActiveBuffer;
end;


function TBufDataset.GetFieldData(Field: TField; Buffer: Pointer;
  NativeFormat: Boolean): Boolean;
begin
  Result := GetFieldData(Field, Buffer);
end;

function TBufDataset.GetFieldData(Field: TField; Buffer: Pointer): Boolean;

var CurrBuff : pchar;

begin
  Result := False;
  if state = dsOldValue then
    begin
    if not GetActiveRecordUpdateBuffer then
      begin
      // There is no old value available
      result := false;
      exit;
      end;
    currbuff := FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer;
    end
  else
    begin
    CurrBuff := GetCurrentBuffer;
    if not assigned(CurrBuff) then
      begin
      result := false;
      exit;
      end;
    end;

  If Field.Fieldno > 0 then // If = 0, then calculated field or something similar
    begin
    if GetFieldIsnull(pbyte(CurrBuff),Field.Fieldno-1) then
      begin
      result := false;
      exit;
      end;
    if assigned(buffer) then
      begin
      inc(CurrBuff,FFieldBufPositions[Field.FieldNo-1]);
      Move(CurrBuff^, Buffer^, GetFieldSize(FieldDefs[Field.FieldNo-1]));
      end;
    Result := True;
    end
  else
    begin
    Inc(CurrBuff, GetRecordSize + Field.Offset);
    Result := Boolean(CurrBuff^);
    if result and assigned(Buffer) then
      begin
      inc(CurrBuff);
      Move(CurrBuff^, Buffer^, Field.Datasize);
      end;
    end;
end;

procedure TBufDataset.SetFieldData(Field: TField; Buffer: Pointer;
  NativeFormat: Boolean);
begin
  SetFieldData(Field,Buffer);
end;

procedure TBufDataset.SetFieldData(Field: TField; Buffer: Pointer);

var CurrBuff : pointer;
    NullMask : pbyte;

begin
  if not (state in [dsEdit, dsInsert, dsFilter, dsCalcFields]) then
    begin
    DatabaseErrorFmt(SNotEditing,[Name],self);
    exit;
    end;
  CurrBuff := GetCurrentBuffer;
  If Field.Fieldno > 0 then // If = 0, then calculated field or something
    begin
    NullMask := CurrBuff;

    inc(CurrBuff,FFieldBufPositions[Field.FieldNo-1]);
    if assigned(buffer) then
      begin
      Move(Buffer^, CurrBuff^, GetFieldSize(FieldDefs[Field.FieldNo-1]));
      unSetFieldIsNull(NullMask,Field.FieldNo-1);
      end
    else
      SetFieldIsNull(NullMask,Field.FieldNo-1);
    end
  else
    begin
    Inc(CurrBuff, GetRecordSize + Field.Offset);
    Boolean(CurrBuff^) := Buffer <> nil;
    inc(CurrBuff);
    if assigned(Buffer) then
      Move(Buffer^, CurrBuff^, Field.Datasize);
    end;
  if not (State in [dsCalcFields, dsFilter, dsNewValue]) then
    DataEvent(deFieldChange, Ptrint(Field));
end;

procedure TBufDataset.InternalDelete;
var i         : Integer;
    StartInd  : Integer;
    RemRec    : pointer;
    RemRecBookmrk : TBufBookmark;
begin
  InternalSetToRecord(ActiveBuffer);
  // Remove the record from all active indexes
  FCurrentIndex.StoreCurrentRecIntoBookmark(@RemRecBookmrk);
  RemRec := FCurrentIndex.CurrentBuffer;
  FIndexes[0].RemoveRecordFromIndex(RemRecBookmrk);
  if FCurrentIndex=FIndexes[1] then StartInd := 1 else StartInd := 2;
  for i := StartInd to FIndexesCount-1 do
    findexes[i].RemoveRecordFromIndex(RemRecBookmrk);

  if not GetActiveRecordUpdateBuffer then
    begin
    FCurrentUpdateBuffer := length(FUpdateBuffer);
    SetLength(FUpdateBuffer,FCurrentUpdateBuffer+1);

    FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := IntAllocRecordBuffer;
    move(RemRec^, FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer^,FRecordSize);
    end
  else //with FIndexes[0] do
    begin
    if FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind <> ukModify then
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := nil;  //this 'disables' the updatebuffer
    end;
  FCurrentIndex.StoreCurrentRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData);
  FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData := RemRecBookmrk;
  FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind := ukDelete;

  dec(FBRecordCount);
end;


procedure TBufDataset.ApplyRecUpdate(UpdateKind : TUpdateKind);

begin
  raise EDatabaseError.Create(SApplyRecNotSupported);
end;

procedure TBufDataset.CancelUpdates;
var StoreRecBM     : TBufBookmark;
  procedure CancelUpdBuffer(var AUpdBuffer : TRecUpdateBuffer);
  var
    TmpBuf         : PChar;
    StoreUpdBuf    : integer;
    Bm             : TBufBookmark;
    x              : Integer;
  begin
    with AUpdBuffer do if assigned(BookmarkData.BookmarkData) then // this is used to exclude buffers which are already handled
      begin
      if (UpdateKind = ukModify) then
        begin
        FCurrentIndex.GotoBookmark(@BookmarkData);
        move(pchar(OldValuesBuffer)^,pchar(FCurrentIndex.CurrentBuffer)^,FRecordSize);
        FreeRecordBuffer(OldValuesBuffer);
        end
      else if (UpdateKind = ukDelete) and (assigned(OldValuesBuffer)) then
        begin
        FCurrentIndex.GotoBookmark(@NextBookmarkData);
        FCurrentIndex.InsertRecordBeforeCurrentRecord(PChar(BookmarkData.BookmarkData));
        FCurrentIndex.ScrollBackward;
        move(pchar(OldValuesBuffer)^,pchar(FCurrentIndex.CurrentBuffer)^,FRecordSize);

{        for x := length(FUpdateBuffer)-1 downto 0 do
          begin
          if (FUpdateBuffer[x].UpdateKind=ukDelete) and FCurrentIndex.CompareBookmarks(@FUpdateBuffer[x].NextBookmarkData,@BookmarkData) then
            CancelUpdBuffer(FUpdateBuffer[x]);
          end;}
        FreeRecordBuffer(OldValuesBuffer);
        inc(FBRecordCount);
        end
      else if (UpdateKind = ukInsert) then
        begin
        // Process all upd-buffers linked to this record before this record is removed
        StoreUpdBuf:=FCurrentUpdateBuffer;
        Bm := BookmarkData;
        BookmarkData.BookmarkData:=nil; // Avoid infinite recursion...
        if GetRecordUpdateBuffer(Bm,True,False) then
          begin
          repeat
          if (FCurrentUpdateBuffer<>StoreUpdBuf) then CancelUpdBuffer(FUpdateBuffer[FCurrentUpdateBuffer]);
          until not GetRecordUpdateBuffer(Bm,True,True);
          end;
        FCurrentUpdateBuffer:=StoreUpdBuf;

        FCurrentIndex.GotoBookmark(@Bm);
        TmpBuf:=FCurrentIndex.CurrentRecord;
        // resync won't work if the currentbuffer is freed...
        if FCurrentIndex.CompareBookmarks(@Bm,@StoreRecBM) then with FCurrentIndex do
          begin
          GotoBookmark(@StoreRecBM);
          if ScrollForward = grEOF then
            ScrollBackward;
          StoreCurrentRecIntoBookmark(@StoreRecBM);
          end;
        FCurrentIndex.RemoveRecordFromIndex(Bm);
        FreeRecordBuffer(TmpBuf);
        dec(FBRecordCount);
        end;
      BookmarkData.BookmarkData:=nil;
      end;
  end;

var r              : Integer;

begin
  CheckBrowseMode;

  if Length(FUpdateBuffer) > 0 then
    begin
    FCurrentIndex.StoreCurrentRecIntoBookmark(@StoreRecBM);
    r := Length(FUpdateBuffer) -1;
    while r > -1 do
      begin
      CancelUpdBuffer(FUpdateBuffer[r]);
      dec(r)
      end;

    SetLength(FUpdateBuffer,0);
    
    FCurrentIndex.GotoBookmark(@StoreRecBM);
    
    Resync([]);
    end;
end;

procedure TBufDataset.SetOnUpdateError(const AValue: TResolverErrorEvent);

begin
  FOnUpdateError := AValue;
end;

procedure TBufDataset.ApplyUpdates; // For backwards-compatibility

begin
  ApplyUpdates(0);
end;

procedure TBufDataset.ApplyUpdates(MaxErrors: Integer);

var r            : Integer;
    FailedCount  : integer;
    Response     : TResolverResponse;
    StoreCurrRec : TBufBookmark;
    AUpdateErr   : EUpdateError;

begin
  CheckBrowseMode;

  FCurrentIndex.StoreCurrentRecIntoBookmark(@StoreCurrRec);

  r := 0;
  FailedCount := 0;
  Response := rrApply;
  DisableControls;
  try
    while (r < Length(FUpdateBuffer)) and (Response <> rrAbort) do
      begin
      // If the record is first inserted and afterwards deleted, do nothing
      if not ((FUpdateBuffer[r].UpdateKind=ukDelete) and not (assigned(FUpdateBuffer[r].OldValuesBuffer))) then
        begin
        FCurrentIndex.GotoBookmark(@FUpdateBuffer[r].BookmarkData);
        // Synchronise the Currentbuffer to the ActiveBuffer
        CurrentRecordToBuffer(ActiveBuffer);
        Response := rrApply;
        try
          ApplyRecUpdate(FUpdateBuffer[r].UpdateKind);
        except
          on E: EDatabaseError do
            begin
            Inc(FailedCount);
            if failedcount > word(MaxErrors) then Response := rrAbort
            else Response := rrSkip;
            if assigned(FOnUpdateError) then
              begin
              AUpdateErr := EUpdateError.Create(SOnUpdateError,E.Message,0,0,Exception(AcquireExceptionObject));
              FOnUpdateError(Self,Self,AUpdateErr,FUpdateBuffer[r].UpdateKind,Response);
              AUpdateErr.Free;
              if Response in [rrApply, rrIgnore] then dec(FailedCount);
              if Response = rrApply then dec(r);
              end
            else if Response = rrAbort then
              Raise EUpdateError.Create(SOnUpdateError,E.Message,0,0,Exception(AcquireExceptionObject));
            end
          else
            raise;
        end;
        if response in [rrApply, rrIgnore] then
          begin
          FreeRecordBuffer(FUpdateBuffer[r].OldValuesBuffer);
          if FUpdateBuffer[r].UpdateKind = ukDelete then
            FreeRecordBuffer(PChar(FUpdateBuffer[r].BookmarkData.BookmarkData));
          FUpdateBuffer[r].BookmarkData.BookmarkData := nil;
          end
        end;
      inc(r);
      end;
  finally
    if failedcount = 0 then
      begin
      SetLength(FUpdateBuffer,0);

      if assigned(FUpdateBlobBuffers) then for r:=0 to length(FUpdateBlobBuffers)-1 do
       if assigned(FUpdateBlobBuffers[r]) then
        begin
        if FUpdateBlobBuffers[r]^.OrgBufID >= 0 then
          begin
          Freemem(FBlobBuffers[FUpdateBlobBuffers[r]^.OrgBufID]^.Buffer);
          Dispose(FBlobBuffers[FUpdateBlobBuffers[r]^.OrgBufID]);
          FBlobBuffers[FUpdateBlobBuffers[r]^.OrgBufID] :=FUpdateBlobBuffers[r];
          end
        else
          begin
          setlength(FBlobBuffers,length(FBlobBuffers)+1);
          FUpdateBlobBuffers[r]^.OrgBufID := high(FBlobBuffers);
          FBlobBuffers[high(FBlobBuffers)] := FUpdateBlobBuffers[r];
          
          end;
        end;
      SetLength(FUpdateBlobBuffers,0);
      end;

    InternalGotoBookmark(@StoreCurrRec);
    Resync([]);
    EnableControls;
  end;
end;


procedure TBufDataset.InternalCancel;

Var i            : integer;

begin
  if assigned(FUpdateBlobBuffers) then for i:=0 to length(FUpdateBlobBuffers)-1 do
   if assigned(FUpdateBlobBuffers[i]) and (FUpdateBlobBuffers[i]^.FieldNo>0) then
    begin
    Reallocmem(FUpdateBlobBuffers[i]^.Buffer,0);
    Dispose(FUpdateBlobBuffers[i]);
    FUpdateBlobBuffers[i] := nil;
    end;
end;

procedure TBufDataset.InternalPost;

Var CurrBuff     : PChar;
    i            : integer;
    blobbuf      : tbufblobfield;
    NullMask     : pbyte;

begin
  inherited InternalPost;
  if assigned(FUpdateBlobBuffers) then for i:=0 to length(FUpdateBlobBuffers)-1 do
   if assigned(FUpdateBlobBuffers[i]) and (FUpdateBlobBuffers[i]^.FieldNo>0) then
    begin
    blobbuf.BlobBuffer := FUpdateBlobBuffers[i];
    CurrBuff := ActiveBuffer;
    NullMask := pbyte(CurrBuff);

    inc(CurrBuff,FFieldBufPositions[FUpdateBlobBuffers[i]^.FieldNo-1]);
    Move(blobbuf, CurrBuff^, GetFieldSize(FieldDefs[FUpdateBlobBuffers[i]^.FieldNo-1]));
    unSetFieldIsNull(NullMask,FUpdateBlobBuffers[i]^.FieldNo-1);
    
    FUpdateBlobBuffers[i]^.FieldNo := -1;
    end;

  if state = dsInsert then
    begin
    if GetBookmarkFlag(ActiveBuffer) = bfEOF then
      FIndexes[0].ScrollLast
    else
      // The active buffer is the newly created TDataset record,
      // from which the bookmark is set to the record where the new record should be
      // inserted
      InternalSetToRecord(ActiveBuffer);

    with FIndexes[0] do
      begin
      // Create the new record buffer
      FCurrentIndex.InsertRecordBeforeCurrentRecord(IntAllocRecordBuffer);
      ScrollBackward;
      // Add the record to the other indexes
      for i := 1 to FIndexesCount-1 do if ((i>1) or (FIndexes[i]=FCurrentIndex)) then
        FIndexes[i].InsertRecordBeforeCurrentRecord(CurrentRecord);
      end;

    // Link the newly created record buffer to the newly created TDataset record
    with PBufBookmark(ActiveBuffer + FRecordSize)^ do
      begin
      FCurrentIndex.StoreCurrentRecIntoBookmark(@BookmarkData);
      BookmarkFlag := bfInserted;
      end;
      
    inc(FBRecordCount);
    end
  else
    InternalSetToRecord(ActiveBuffer);


  // If there is no updatebuffer already, add one
  if not GetActiveRecordUpdateBuffer then
    begin
    // Add a new updatebuffer
    FCurrentUpdateBuffer := length(FUpdateBuffer);
    SetLength(FUpdateBuffer,FCurrentUpdateBuffer+1);

    // Store a bookmark of the current record into the updatebuffer's bookmark
    FCurrentIndex.StoreCurrentRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);

    if state = dsEdit then
      begin
      // Create an oldvalues buffer with the old values of the record
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := intAllocRecordBuffer;
      with FCurrentIndex do
        // Move only the real data
        move(CurrentBuffer^,FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer^,FRecordSize);
      FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind := ukModify;
      end
    else
      begin
      FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind := ukInsert;
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := nil;
      end;
    end;

  move(ActiveBuffer^,FCurrentIndex.CurrentBuffer^,FRecordSize);
end;

procedure TBufDataset.CalcRecordSize;

var x : longint;

begin
  FNullmaskSize := 1+((FieldDefs.count-1) div 8);
{$IFDEF FPC_REQUIRES_PROPER_ALIGNMENT}
  FNullmaskSize:=Align(FNullmaskSize,4);
{$ENDIF}
  FRecordSize := FNullmaskSize;
  SetLength(FFieldBufPositions,FieldDefs.count);
  for x := 0 to FieldDefs.count-1 do
    begin
    FFieldBufPositions[x] := FRecordSize;
    inc(FRecordSize, GetFieldSize(FieldDefs[x]));
    end;
end;

function TBufDataset.GetIndexFieldNames: String;
begin
  if FCurrentIndex<>FIndexes[1] then
    result := ''
  else
    result := FCurrentIndex.FieldsName;
end;

function TBufDataset.GetIndexName: String;
begin
  result := FCurrentIndex.Name;
end;

function TBufDataset.GetRecordSize : Word;

begin
  result := FRecordSize + BookmarkSize;
end;

function TBufDataset.GetChangeCount: integer;

begin
  result := length(FUpdateBuffer);
end;


procedure TBufDataset.InternalInitRecord(Buffer: PChar);

begin
  FillChar(Buffer^, FRecordSize, #0);

  fillchar(Buffer^,FNullmaskSize,255);
end;

procedure TBufDataset.SetRecNo(Value: Longint);

var
    recnr        : integer;
    TmpRecBuffer : PBufRecLinkItem;

begin
  checkbrowsemode;
  if value > RecordCount then
    begin
    repeat until (getnextpacket < FPacketRecords) or (value <= RecordCount) or (FPacketRecords = -1);
    if value > RecordCount then
      begin
      DatabaseError(SNoSuchRecord,self);
      exit;
      end;
    end;
  TmpRecBuffer := (FCurrentIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  for recnr := 1 to value-1 do
    TmpRecBuffer := TmpRecBuffer^.next;
  GotoBookmark(@TmpRecBuffer);
end;

function TBufDataset.GetRecNo: Longint;

Var abuf            : PChar;

begin
  abuf := GetCurrentBuffer;
  // If abuf isn't assigned, the recordset probably isn't opened.
  if assigned(abuf) and (FBRecordCount>0) and (state <> dsInsert) then
    Result:=FCurrentIndex.GetRecNo(PBufBookmark(abuf+FRecordSize))
  else
    result := 0;
end;

function TBufDataset.IsCursorOpen: Boolean;

begin
  Result := FOpen;
end;

Function TBufDataset.GetRecordCount: Longint;

begin
  Result := FBRecordCount;
end;

Function TBufDataSet.UpdateStatus: TUpdateStatus;

begin
  Result:=usUnmodified;
  if GetActiveRecordUpdateBuffer then
    case FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind of
      ukModify : Result := usModified;
      ukInsert : Result := usInserted;
      ukDelete : Result := usDeleted;
    end;
end;

function TbufDataset.GetNewBlobBuffer : PBlobBuffer;

var ABlobBuffer : PBlobBuffer;

begin
  setlength(FBlobBuffers,length(FBlobBuffers)+1);
  new(ABlobBuffer);
  fillbyte(ABlobBuffer^,sizeof(ABlobBuffer^),0);
  ABlobBuffer^.OrgBufID := high(FUpdateBlobBuffers);
  FBlobBuffers[high(FBlobBuffers)] := ABlobBuffer;
  result := ABlobBuffer;
end;

function TbufDataset.GetNewWriteBlobBuffer : PBlobBuffer;

var ABlobBuffer : PBlobBuffer;

begin
  setlength(FUpdateBlobBuffers,length(FUpdateBlobBuffers)+1);
  new(ABlobBuffer);
  fillbyte(ABlobBuffer^,sizeof(ABlobBuffer^),0);
  FUpdateBlobBuffers[high(FUpdateBlobBuffers)] := ABlobBuffer;
  result := ABlobBuffer;
end;

procedure TBufDataset.FreeBlobBuffer(var ABlobBuffer: PBlobBuffer);

begin
  if not Assigned(ABlobBuffer) then Exit;
  FreeMem(ABlobBuffer^.Buffer, ABlobBuffer^.Size);
  Dispose(ABlobBuffer);
  ABlobBuffer := Nil;
end;

function TBufBlobStream.Seek(Offset: Longint; Origin: Word): Longint;

begin
  Case Origin of
    soFromBeginning : FPosition:=Offset;
    soFromEnd       : FPosition:=FBlobBuffer^.Size+Offset;
    soFromCurrent   : FpoSition:=FPosition+Offset;
  end;
  Result:=FPosition;
end;


function TBufBlobStream.Read(var Buffer; Count: Longint): Longint;

var ptr : pointer;

begin
  if FPosition + count > FBlobBuffer^.Size then
    count := FBlobBuffer^.Size-FPosition;
  ptr := FBlobBuffer^.Buffer+FPosition;
  move(ptr^,buffer,count);
  inc(FPosition,count);
  result := count;
end;

function TBufBlobStream.Write(const Buffer; Count: Longint): Longint;

var ptr : pointer;

begin
  ReAllocMem(FBlobBuffer^.Buffer,FPosition+Count);
  ptr := FBlobBuffer^.Buffer+FPosition;
  move(buffer,ptr^,count);
  inc(FBlobBuffer^.Size,count);
  inc(FPosition,count);
  Result := count;
end;

constructor TBufBlobStream.Create(Field: TBlobField; Mode: TBlobStreamMode);

var bufblob : TBufBlobField;

begin
  FDataset := Field.DataSet as TBufDataset;
  if mode = bmread then
    begin
    if not field.getData(@bufblob) then
      DatabaseError(SFieldIsNull);
    if not assigned(bufblob.BlobBuffer) then with FDataSet do
      begin
      FBlobBuffer := GetNewBlobBuffer;
      bufblob.BlobBuffer := FBlobBuffer;
      LoadBlobIntoBuffer(FieldDefs[field.FieldNo-1],@bufblob);
      end
    else
      FBlobBuffer := bufblob.BlobBuffer;
    end
  else if mode=bmWrite then with FDataSet as TBufDataset do
    begin
    FBlobBuffer := GetNewWriteBlobBuffer;
    FBlobBuffer^.FieldNo := Field.FieldNo;
    if (field.getData(@bufblob)) and assigned(bufblob.BlobBuffer) then
      FBlobBuffer^.OrgBufID := bufblob.BlobBuffer^.OrgBufID
    else
      FBlobBuffer^.OrgBufID := -1;
    end;
end;

function TBufDataset.CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream;

var bufblob : TBufBlobField;

begin
  result := nil;
  if mode=bmread then
    begin
    if not field.getData(@bufblob) then
      exit;

    result := TBufBlobStream.Create(Field as tblobfield,bmread);
    end
  else if mode=bmWrite then
    begin
    if not (state in [dsEdit, dsInsert, dsFilter, dsCalcFields]) then
      begin
      DatabaseErrorFmt(SNotEditing,[Name],self);
      exit;
      end;

    result := TBufBlobStream.Create(Field as tblobfield,bmWrite);

    if not (State in [dsCalcFields, dsFilter, dsNewValue]) then
      DataEvent(deFieldChange, Ptrint(Field));
    end;
end;

procedure TBufDataset.AddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string = '';
                               const ACaseInsFields: string = '');
begin
  if AFields='' then DatabaseError(SNoIndexFieldNameGiven);
  
  if active and (FIndexesCount=FMaxIndexesCount) then
    DatabaseError(SMaxIndexes);

  // If not all packets are fetched, you can not sort properly.
  if not active then
    FPacketRecords:=-1;
  InternalAddIndex(AName,AFields,AOptions,ADescFields,ACaseInsFields);
end;

procedure TBufDataset.SaveToFile(AFileName: string;
  Format: TDataPacketFormat);
var AFileStream : TFileStream;
begin
  if AFileName='' then AFileName := FFileName;
  AFileStream := TFileStream.Create(AFileName,fmCreate);
  try
    SaveToStream(AFileStream, Format);
  finally
    AFileStream.Free;
  end;
end;

procedure TBufDataset.SetDatasetPacket(AReader: TDataPacketReader);
begin
  FDatasetReader := AReader;
  try
    Open;
  finally
    FDatasetReader := nil;
  end;
end;

procedure TBufDataset.GetDatasetPacket(AWriter: TDataPacketReader);

  procedure StoreUpdateBuffer(AUpdBuffer : TRecUpdateBuffer; var ARowState: TRowState);
  var AThisRowState : TRowState;
      AStoreUpdBuf  : Integer;
  begin
    if AUpdBuffer.UpdateKind = ukModify then
      begin
      AThisRowState := [rsvOriginal];
      ARowState:=[rsvUpdated];
      end
    else if AUpdBuffer.UpdateKind = ukDelete then
      begin
      AStoreUpdBuf:=FCurrentUpdateBuffer;
      if GetRecordUpdateBuffer(AUpdBuffer.BookmarkData,True) then
        begin
        repeat
        if FCurrentIndex.CompareBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData, @AUpdBuffer.BookmarkData) then
          StoreUpdateBuffer(FUpdateBuffer[FCurrentUpdateBuffer], ARowState);
        until not GetRecordUpdateBuffer(AUpdBuffer.BookmarkData,True,True)
        end;
      FCurrentUpdateBuffer:=AStoreUpdBuf;
      AThisRowState := [rsvDeleted];
      end
    else // ie: updatekind = ukInsert
      begin
      ARowState := [rsvInserted];
      Exit;
      end;
    FFilterBuffer:=AUpdBuffer.OldValuesBuffer;
    FDatasetReader.StoreRecord(Self,AThisRowState,FCurrentUpdateBuffer);
  end;

  procedure HandleUpdateBuffersFromRecord(AFirstCall : boolean;ARecBookmark : TBufBookmark; var ARowState: TRowState);
  var StoreUpdBuf1,StoreUpdBuf2 : Integer;
  begin
    if AFirstCall then ARowState:=[];
    if GetRecordUpdateBuffer(ARecBookmark,True,not AFirstCall) then
      begin
      if FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind=ukDelete then
        begin
        StoreUpdBuf1:=FCurrentUpdateBuffer;
        HandleUpdateBuffersFromRecord(False,ARecBookmark,ARowState);
        StoreUpdBuf2:=FCurrentUpdateBuffer;
        FCurrentUpdateBuffer:=StoreUpdBuf1;
        StoreUpdateBuffer(FUpdateBuffer[StoreUpdBuf1], ARowState);
        FCurrentUpdateBuffer:=StoreUpdBuf2;
        end
      else
        begin
        StoreUpdateBuffer(FUpdateBuffer[FCurrentUpdateBuffer], ARowState);
        HandleUpdateBuffersFromRecord(False,ARecBookmark,ARowState);
        end;
      end
  end;

var ScrollResult   : TGetResult;
    StoreDSState   : TDataSetState;
    ABookMark      : PBufBookmark;
    ATBookmark     : TBufBookmark;
    RowState       : TRowState;

begin
  FDatasetReader := AWriter;
  try
    //CheckActive;
    ABookMark:=@ATBookmark;
    FDatasetReader.StoreFieldDefs(FieldDefs);

    StoreDSState:=State;
    SetTempState(dsFilter);
    ScrollResult:=FCurrentIndex.ScrollFirst;
    while ScrollResult=grOK do
      begin
      RowState:=[];
      FCurrentIndex.StoreCurrentRecIntoBookmark(ABookmark);
      HandleUpdateBuffersFromRecord(True,ABookmark^,RowState);
      FFilterBuffer:=FCurrentIndex.CurrentBuffer;
      if RowState=[] then
        FDatasetReader.StoreRecord(Self,[])
      else
        FDatasetReader.StoreRecord(Self,RowState,FCurrentUpdateBuffer);

      ScrollResult:=FCurrentIndex.ScrollForward;
      if ScrollResult<>grOK then
        begin
        if getnextpacket>0 then
          ScrollResult := FCurrentIndex.ScrollForward;
        end;
      end;
    // There could be a update-buffer linked to the last (spare) record
    FCurrentIndex.StoreSpareRecIntoBookmark(ABookmark);
    HandleUpdateBuffersFromRecord(True,ABookmark^,RowState);

    RestoreState(StoreDSState);

    FDatasetReader.FinalizeStoreRecords;
  finally
    FDatasetReader := nil;
  end;
end;

procedure TBufDataset.LoadFromStream(AStream: TStream; Format: TDataPacketFormat);
var APacketReaderReg : TDatapacketReaderRegistration;
    APacketReader : TDataPacketReader;
begin
  if GetRegisterDatapacketReader(AStream,format,APacketReaderReg) then
    APacketReader := APacketReaderReg.ReaderClass.create(AStream)
  else if TFpcBinaryDatapacketReader.RecognizeStream(AStream) then
    begin
    AStream.Seek(0,soFromBeginning);
    APacketReader := TFpcBinaryDatapacketReader.create(AStream)
    end
  else
    DatabaseError(SStreamNotRecognised);
  try
    SetDatasetPacket(APacketReader);
  finally
    APacketReader.Free;
  end;
end;

procedure TBufDataset.SaveToStream(AStream: TStream; Format: TDataPacketFormat);
var APacketReaderReg : TDatapacketReaderRegistration;
    APacketWriter : TDataPacketReader;
begin
  if GetRegisterDatapacketReader(Nil,format,APacketReaderReg) then
    APacketWriter := APacketReaderReg.ReaderClass.create(AStream)
  else if Format = dfBinary then
    APacketWriter := TFpcBinaryDatapacketReader.create(AStream)
  else
    DatabaseError(SNoReaderClassRegistered);
  try
    GetDatasetPacket(APacketWriter);
  finally
    APacketWriter.Free;
  end;
end;

procedure TBufDataset.LoadFromFile(AFileName: string; Format: TDataPacketFormat);
var AFileStream : TFileStream;
begin
  if AFileName='' then AFileName := FFileName;
  AFileStream := TFileStream.Create(AFileName,fmOpenRead);
  try
    LoadFromStream(AFileStream, Format);
  finally
    AFileStream.Free;
  end;
end;

procedure TBufDataset.CreateDataset;
begin
  CheckInactive;
  CreateFields;
end;

function TBufDataset.BookmarkValid(ABookmark: TBookmark): Boolean;
begin
  Result:=FCurrentIndex.BookmarkValid(ABookmark);
end;

function TBufDataset.CompareBookmarks(Bookmark1, Bookmark2: TBookmark
  ): Longint;
begin
  if FCurrentIndex.CompareBookmarks(Bookmark1,Bookmark2) then
    Result := 0
  else
    Result := -1;
end;

procedure TBufDataset.IntLoadFielddefsFromFile;

begin
  FDatasetReader.LoadFielddefs(FieldDefs);
  if DefaultFields then CreateFields;
end;

procedure TBufDataset.IntLoadRecordsFromFile;

var StoreState      : TDataSetState;
    AddRecordBuffer : boolean;
    ARowState       : TRowState;
    AUpdOrder       : integer;
    x               : integer;

begin
  FDatasetReader.InitLoadRecords;
  StoreState:=SetTempState(dsFilter);

  while FDatasetReader.GetCurrentRecord do
    begin
    ARowState := FDatasetReader.GetRecordRowState(AUpdOrder);
    if rsvOriginal in ARowState then
      begin
      if length(FUpdateBuffer) < (AUpdOrder+1) then
        SetLength(FUpdateBuffer,AUpdOrder+1);

      FCurrentUpdateBuffer:=AUpdOrder;

      FFilterBuffer:=IntAllocRecordBuffer;
      fillchar(FFilterBuffer^,FNullmaskSize,0);
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := FFilterBuffer;
      FDatasetReader.RestoreRecord(self);

      FDatasetReader.GotoNextRecord;
      if not FDatasetReader.GetCurrentRecord then
        DatabaseError(SStreamNotRecognised);
      ARowState := FDatasetReader.GetRecordRowState(AUpdOrder);
      if rsvUpdated in ARowState then
        FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind:= ukModify
      else
        DatabaseError(SStreamNotRecognised);

      FFilterBuffer:=FIndexes[0].SpareBuffer;
      FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
      fillchar(FFilterBuffer^,FNullmaskSize,0);

      FDatasetReader.RestoreRecord(self);
      FIndexes[0].AddRecord(IntAllocRecordBuffer);
      inc(FBRecordCount);

      AddRecordBuffer:=False;

      end
    else if rsvDeleted in ARowState then
      begin
      if length(FUpdateBuffer) < (AUpdOrder+1) then
        SetLength(FUpdateBuffer,AUpdOrder+1);

      FCurrentUpdateBuffer:=AUpdOrder;

      FFilterBuffer:=IntAllocRecordBuffer;
      fillchar(FFilterBuffer^,FNullmaskSize,0);
      FUpdateBuffer[FCurrentUpdateBuffer].OldValuesBuffer := FFilterBuffer;
      FDatasetReader.RestoreRecord(self);

      FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind:= ukDelete;
      FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
      FIndexes[0].AddRecord(IntAllocRecordBuffer);
      FIndexes[0].RemoveRecordFromIndex(FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
      FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].NextBookmarkData);

      for x := FCurrentUpdateBuffer+1 to length(FUpdateBuffer)-1 do
        if Findexes[0].CompareBookmarks(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData,@FUpdateBuffer[x].NextBookmarkData) then
          FIndexes[0].StoreSpareRecIntoBookmark(@FUpdateBuffer[x].NextBookmarkData);

      AddRecordBuffer:=False;
      end
    else
      AddRecordBuffer:=True;

    if AddRecordBuffer then
      begin
      FFilterBuffer:=FIndexes[0].SpareBuffer;
      fillchar(FFilterBuffer^,FNullmaskSize,0);

      FDatasetReader.RestoreRecord(self);

      if rsvInserted in ARowState then
        begin
        if length(FUpdateBuffer) < (AUpdOrder+1) then
          SetLength(FUpdateBuffer,AUpdOrder+1);
        FCurrentUpdateBuffer:=AUpdOrder;
        FUpdateBuffer[FCurrentUpdateBuffer].UpdateKind:= ukInsert;
        FCurrentIndex.StoreSpareRecIntoBookmark(@FUpdateBuffer[FCurrentUpdateBuffer].BookmarkData);
        end;

      FIndexes[0].AddRecord(IntAllocRecordBuffer);
      inc(FBRecordCount);
      end;

    FDatasetReader.GotoNextRecord;
    end;

  RestoreState(StoreState);
  FIndexes[0].SetToFirstRecord;
  FAllPacketsFetched:=True;
  if assigned(FFileStream) then
    begin
    FreeAndNil(FFileStream);
    FreeAndNil(FDatasetReader);
    end;
end;

procedure TBufDataset.InternalAddIndex(const AName, AFields : string; AOptions : TIndexOptions; const ADescFields: string;
                                       const ACaseInsFields: string);
var StoreIndNr : Integer;
begin
  if Active then FetchAll;
  if FIndexesCount>0 then
    StoreIndNr:=FCurrentIndex.IndNr
  else
    StoreIndNr:=0;
  inc(FIndexesCount);
  setlength(FIndexes,FIndexesCount); // This invalidates the currentindex! -> not anymore
  FCurrentIndex:=FIndexes[StoreIndNr];
  FIndexes[FIndexesCount-1] := TDoubleLinkedBufIndex.Create(self);
//  FIndexes[FIndexesCount-1] := TArrayBufIndex.Create(self);
  FIndexes[FIndexesCount-1].InitialiseIndex;
  with (FIndexes[FIndexesCount-1] as TBufIndex) do
    begin
    Name:=AName;
    FieldsName:=AFields;
    DescFields:=ADescFields;
    CaseinsFields:=ACaseInsFields;
    Options:=AOptions;
    IndNr:=FIndexesCount-1;
    end;

  if Active then
    begin
    (FIndexes[FIndexesCount-1] as TDoubleLinkedBufIndex).FFirstRecBuf := pointer(IntAllocRecordBuffer);
    (FIndexes[FIndexesCount-1] as TDoubleLinkedBufIndex).FLastRecBuf := (FIndexes[FIndexesCount-1] as TDoubleLinkedBufIndex).FFirstRecBuf;
    (FCurrentIndex as TDoubleLinkedBufIndex).FCurrentRecBuf := (FIndexes[FIndexesCount-1] as TDoubleLinkedBufIndex).FLastRecBuf;
    BuildIndex(FIndexes[FIndexesCount-1]);
    end
  else if FIndexesCount>FMaxIndexesCount then
    FMaxIndexesCount := FIndexesCount;
end;

procedure TBufDataset.DoFilterRecord(var Acceptable: Boolean);
begin
  Acceptable := true;
  // check user filter
  if Assigned(OnFilterRecord) then
    OnFilterRecord(Self, Acceptable);

  // check filtertext
  if Acceptable and (Length(Filter) > 0) then
    Acceptable := Boolean((FParser.ExtractFromBuffer(GetCurrentBuffer))^);

end;

procedure TBufDataset.SetFilterText(const Value: String);
begin
  if Value = Filter then
    exit;

  // parse
  ParseFilter(Value);

  // call dataset method
  inherited;

  // refilter dataset if filtered
  if IsCursorOpen and Filtered then Refresh;
end;

procedure TBufDataset.SetFiltered(Value: Boolean); {override;}
begin
  if Value = Filtered then
    exit;

  // pass on to ancestor
  inherited;

  // only refresh if active
  if IsCursorOpen then
    Refresh;
end;

function TBufDataset.Fetch: boolean;
begin
  // Empty procedure to make it possible to use TBufDataset as a memory dataset
  Result := False;
end;

function TBufDataset.LoadField(FieldDef: TFieldDef; buffer: pointer; out
  CreateBlob: boolean): boolean;
begin
  // Empty procedure to make it possible to use TBufDataset as a memory dataset
  CreateBlob := False;
  Result := False;
end;

function TBufDataset.IsReadFromPacket: Boolean;
begin
  Result := (FDatasetReader<>nil) or (FFileName<>'');
end;

procedure TBufDataset.ParseFilter(const AFilter: string);
begin
  // parser created?
  if Length(AFilter) > 0 then
  begin
    if (FParser = nil) and IsCursorOpen then
    begin
      FParser := TBufDatasetParser.Create(Self);
    end;
    // have a parser now?
    if FParser <> nil then
    begin
      // set options
      FParser.PartialMatch := not (foNoPartialCompare in FilterOptions);
      FParser.CaseInsensitive := foCaseInsensitive in FilterOptions;
      // parse expression
      FParser.ParseExpression(AFilter);
    end;
  end;
end;

function TArrayBufIndex.GetRecordFromBookmark(ABookMark: TBufBookmark) : integer;
begin
  // ABookmark.BookMarkBuf is nil if SetRecNo calls GotoBookmark
  if (ABookmark.BookmarkData<>nil) and (FRecordArray[ABookmark.BookmarkInt]<>ABookmark.BookmarkData) then
    begin
    // Start searching two records before the expected record
    if ABookmark.BookmarkInt > 2 then
      Result := ABookmark.BookmarkInt-2
    else
      Result := 0;

    while (Result<FLastRecInd) do
      begin
      if (FRecordArray[Result] = ABookmark.BookmarkData) then exit;
      inc(Result);
      end;

    Result:=0;
    while (Result<ABookmark.BookmarkInt) do
      begin
      if (FRecordArray[Result] = ABookmark.BookmarkData) then exit;
      inc(Result);
      end;

    DatabaseError(SInvalidBookmark)
    end
  else
    Result := ABookmark.BookmarkInt;
end;

Function TBufDataset.Locate(const KeyFields: string; const KeyValues: Variant; options: TLocateOptions) : boolean;

var CurrLinkItem    : PBufRecLinkItem;
    bm              : TBufBookmark;
    SearchFields    : TList;
    FieldsAmount    : Integer;
    DBCompareStruct : TDBCompareStruct;
    FieldNr         : Integer;
    StoreDSState    : TDataSetState;
    FilterBuffer    : PChar;


begin
  Result := False;
  if IsEmpty then exit;

  // Build the DBCompare structure
  SearchFields := TList.Create;
  try
    GetFieldList(SearchFields,KeyFields);
    FieldsAmount:=SearchFields.Count;
    if FieldsAmount=0 then exit;

    SetLength(DBCompareStruct,FieldsAmount);
    for FieldNr:=0 to FieldsAmount-1 do
      ProcessFieldCompareStruct(TField(SearchFields[FieldNr]),DBCompareStruct[FieldNr]);
  finally
    SearchFields.Free;
  end;

  // Set The filter-buffer
  StoreDSState:=State;
  FFilterBuffer:=FCurrentIndex.SpareBuffer;
  SetTempState(dsFilter);
  SetFieldValues(keyfields,KeyValues);
  CurrLinkItem := (FCurrentIndex as TDoubleLinkedBufIndex).FFirstRecBuf;
  FilterBuffer:=IntAllocRecordBuffer;
  move((FCurrentIndex as TDoubleLinkedBufIndex).FLastRecBuf^,FilterBuffer^,FRecordsize+sizeof(TBufRecLinkItem)*FMaxIndexesCount);
  RestoreState(StoreDSState);

  // Iterate through the records until a match is found
  while (CurrLinkItem <> (FCurrentIndex as TDoubleLinkedBufIndex).FLastRecBuf) do
    begin
    if (IndexCompareRecords(FilterBuffer,CurrLinkItem,DBCompareStruct) = 0) then
      begin
      Result := True;
      break;
      end;
    CurrLinkItem := CurrLinkItem^.next;
    if CurrLinkItem = (FCurrentIndex as TDoubleLinkedBufIndex).FLastRecBuf then
      getnextpacket;
    end;
    
  FreeRecordBuffer(FilterBuffer);

  // If a match is found, jump to the found record
  if Result then
    begin
    bm.BookmarkData := CurrLinkItem;
    bm.BookmarkFlag := bfCurrent;
    GotoBookmark(@bm);
    end;
end;

{ TArrayBufIndex }

function TArrayBufIndex.GetBookmarkSize: integer;
begin
  Result:=Sizeof(TBufBookmark);
end;

function TArrayBufIndex.GetCurrentBuffer: Pointer;
begin
  Result:=pchar(FRecordArray[FCurrentRecInd]);
end;

function TArrayBufIndex.GetCurrentRecord: PChar;
begin
  Result:=GetCurrentBuffer;
end;

function TArrayBufIndex.GetIsInitialized: boolean;
begin
  Result:=Length(FRecordArray)>0;
end;

function TArrayBufIndex.GetSpareBuffer: PChar;
begin
  if FLastRecInd>-1 then
    Result:=pchar(FRecordArray[FLastRecInd])
  else
    Result := nil;
end;

function TArrayBufIndex.GetSpareRecord: PChar;
begin
  Result := GetSpareBuffer;
end;

constructor TArrayBufIndex.Create(const ADataset: TBufDataset);
begin
  Inherited create(ADataset);
  FInitialBuffers:=10000;
  FGrowBuffer:=1000;
end;

function TArrayBufIndex.ScrollBackward: TGetResult;
begin
  if FCurrentRecInd>0 then
    begin
    dec(FCurrentRecInd);
    Result := grOK;
    end
  else
    Result := grBOF;
end;

function TArrayBufIndex.ScrollForward: TGetResult;
begin
  if FCurrentRecInd = FLastRecInd-1 then
    result := grEOF
  else
    begin
    Result:=grOK;
    inc(FCurrentRecInd);
    end;
end;

function TArrayBufIndex.GetCurrent: TGetResult;
begin
  if FLastRecInd=0 then
    Result := grError
  else
    begin
    Result := grOK;
    if FCurrentRecInd = FLastRecInd then
      dec(FCurrentRecInd);
    end;
end;

function TArrayBufIndex.ScrollFirst: TGetResult;
begin
  FCurrentRecInd:=0;
  if (FCurrentRecInd = FLastRecInd) then
    result := grEOF
  else
    result := grOk;
end;

procedure TArrayBufIndex.ScrollLast;
begin
  FCurrentRecInd:=FLastRecInd;
end;

procedure TArrayBufIndex.SetToFirstRecord;
begin
// if FCurrentRecBuf = FLastRecBuf then the dataset is just opened and empty
// in which case InternalFirst should do nothing (bug 7211)
  if FCurrentRecInd <> FLastRecInd then
    FCurrentRecInd := -1;
end;

procedure TArrayBufIndex.SetToLastRecord;
begin
  if FLastRecInd <> 0 then FCurrentRecInd := FLastRecInd;
end;

procedure TArrayBufIndex.StoreCurrentRecord;
begin
  FStoredRecBuf := FCurrentRecInd;
end;

procedure TArrayBufIndex.RestoreCurrentRecord;
begin
  FCurrentRecInd := FStoredRecBuf;
end;

function TArrayBufIndex.CanScrollForward: Boolean;
begin
  Result := (FCurrentRecInd < FLastRecInd-1);
end;

procedure TArrayBufIndex.DoScrollForward;
begin
  inc(FCurrentRecInd);
end;

procedure TArrayBufIndex.StoreCurrentRecIntoBookmark(const ABookmark: PBufBookmark);
begin
  with ABookmark^ do
    begin
    BookmarkInt := FCurrentRecInd;
    BookmarkData := FRecordArray[FCurrentRecInd];
    end;
end;

procedure TArrayBufIndex.StoreSpareRecIntoBookmark(const ABookmark: PBufBookmark
  );
begin
  with ABookmark^ do
    begin
    BookmarkInt := FLastRecInd;
    BookmarkData := FRecordArray[FLastRecInd];
    end;
end;

procedure TArrayBufIndex.GotoBookmark(const ABookmark : PBufBookmark);
begin
  FCurrentRecInd:=GetRecordFromBookmark(ABookmark^);
end;

procedure TArrayBufIndex.InitialiseIndex;
begin
//  FRecordArray:=nil;
  setlength(FRecordArray,FInitialBuffers);
  FCurrentRecInd:=-1;
  FLastRecInd:=-1;
end;

procedure TArrayBufIndex.InitialiseSpareRecord(const ASpareRecord: PChar);
begin
  FLastRecInd := 0;
 // FCurrentRecInd := 0;
  FRecordArray[0] := ASpareRecord;
end;

procedure TArrayBufIndex.ReleaseSpareRecord;
begin
  SetLength(FRecordArray,FInitialBuffers);
end;

function TArrayBufIndex.GetRecNo(const ABookmark: PBufBookmark): integer;
begin
  Result := GetRecordFromBookmark(ABookmark^)+1;
end;

procedure TArrayBufIndex.RemoveRecordFromIndex(const ABookmark : TBufBookmark);
var ARecordInd : integer;
begin
  ARecordInd:=GetRecordFromBookmark(ABookmark);
  Move(FRecordArray[ARecordInd+1],FRecordArray[ARecordInd],sizeof(Pointer)*(FLastRecInd-ARecordInd));
  dec(FLastRecInd);
end;

procedure TArrayBufIndex.InsertRecordBeforeCurrentRecord(const ARecord: PChar);
begin
  inc(FLastRecInd);
  if FLastRecInd >= length(FRecordArray) then
    SetLength(FRecordArray,length(FRecordArray)+FGrowBuffer);

  Move(FRecordArray[FCurrentRecInd],FRecordArray[FCurrentRecInd+1],sizeof(Pointer)*(FLastRecInd-FCurrentRecInd));
  FRecordArray[FCurrentRecInd]:=ARecord;
  inc(FCurrentRecInd);
end;

procedure TArrayBufIndex.BeginUpdate;
begin
//  inherited BeginUpdate;
end;

procedure TArrayBufIndex.AddRecord(const ARecord: PChar);
begin
  inc(FLastRecInd);
  if FLastRecInd >= length(FRecordArray) then
    SetLength(FRecordArray,length(FRecordArray)+FGrowBuffer);
  FRecordArray[FLastRecInd]:=ARecord;
end;

procedure TArrayBufIndex.EndUpdate;
begin
//  inherited EndUpdate;
end;

{ TDataPacketReader }

class function TDataPacketReader.RowStateToByte(const ARowState: TRowState
  ): byte;
var RowStateInt : Byte;
begin
  RowStateInt:=0;
  if rsvOriginal in ARowState then RowStateInt := RowStateInt+1;
  if rsvDeleted in ARowState then RowStateInt := RowStateInt+2;
  if rsvInserted in ARowState then RowStateInt := RowStateInt+4;
  if rsvUpdated in ARowState then RowStateInt := RowStateInt+8;
  Result := RowStateInt;
end;

class function TDataPacketReader.ByteToRowState(const AByte: Byte): TRowState;
begin
  result := [];
  if (AByte and 1)=1 then Result := Result+[rsvOriginal];
  if (AByte and 2)=2 then Result := Result+[rsvDeleted];
  if (AByte and 4)=4 then Result := Result+[rsvInserted];
  if (AByte and 8)=8 then Result := Result+[rsvUpdated];
end;

constructor TDataPacketReader.create(AStream: TStream);
begin
  FStream := AStream;
end;

{ TFpcBinaryDatapacketReader }

const FpcBinaryIdent = 'BinBufDataset';

procedure TFpcBinaryDatapacketReader.LoadFieldDefs(AFieldDefs: TFieldDefs);

var FldCount : word;
    i        : integer;

begin
  if not RecognizeStream(Stream) then
    DatabaseError(SStreamNotRecognised);

  FldCount:=Stream.ReadWord;
  for i := 0 to FldCount -1 do with TFieldDef.create(AFieldDefs) do
    begin
    Name := Stream.ReadAnsiString;
    Displayname := Stream.ReadAnsiString;
    Size := Stream.ReadWord;
    DataType := TFieldType(Stream.ReadWord);

    if Stream.ReadByte = 1 then
      Attributes := Attributes + [faReadonly];
    end;
end;

procedure TFpcBinaryDatapacketReader.StoreFieldDefs(AFieldDefs: TFieldDefs);
var i : integer;
begin
  Stream.Write(FpcBinaryIdent[1],length(FpcBinaryIdent));

  Stream.WriteWord(AFieldDefs.Count);
  for i := 0 to AFieldDefs.Count -1 do with AFieldDefs[i] do
    begin
    Stream.WriteAnsiString(Name);
    Stream.WriteAnsiString(DisplayName);
    Stream.WriteWord(size);
    Stream.WriteWord(ord(DataType));

    if faReadonly in Attributes then
      Stream.WriteByte(1)
    else
      Stream.WriteByte(0);
    end;
end;

function TFpcBinaryDatapacketReader.GetRecordRowState(out AUpdOrder : Integer) : TRowState;
var Buf : byte;
begin
  Stream.Read(Buf,1);
  Result := ByteToRowState(Buf);
  if Result<>[] then
    Stream.ReadBuffer(AUpdOrder,sizeof(integer))
  else
    AUpdOrder := 0;
end;

>>>>>>> origin/fixes_2_2
procedure TFpcBinaryDatapacketReader.FinalizeStoreRecords;
begin
//  Do nothing
end;

function TFpcBinaryDatapacketReader.GetCurrentRecord: boolean;
var Buf : byte;
begin
  Result := (Stream.Read(Buf,1)=1) and (Buf=$fe);
end;

procedure TFpcBinaryDatapacketReader.GotoNextRecord;
begin
//  Do Nothing
end;

procedure TFpcBinaryDatapacketReader.InitLoadRecords;
begin
//  SetLength(AChangeLog,0);
end;

procedure TFpcBinaryDatapacketReader.RestoreRecord(ADataset: TBufDataset);
begin
  Stream.ReadBuffer(ADataset.GetCurrentBuffer^,ADataset.FRecordSize);
end;

procedure TFpcBinaryDatapacketReader.StoreRecord(ADataset: TBufDataset;
  ARowState: TRowState; AUpdOrder : integer);
begin
  // Ugly because private members of ADataset are used...
  Stream.WriteByte($fe);
  Stream.WriteByte(RowStateToByte(ARowState));
  if ARowState<>[] then
    Stream.WriteBuffer(AUpdOrder,sizeof(integer));
  Stream.WriteBuffer(ADataset.GetCurrentBuffer^,ADataset.FRecordSize);
end;

class function TFpcBinaryDatapacketReader.RecognizeStream(AStream: TStream
  ): boolean;
var s        : string;
    len      : integer;
begin
  Len := length(FpcBinaryIdent);
  setlength(s,len);
  if (AStream.Read (s[1],len) = len)
  and (s=FpcBinaryIdent) then
    Result := True
  else
    Result := False;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
initialization
  setlength(RegisteredDatapacketReaders,0);
finalization
  setlength(RegisteredDatapacketReaders,0);
end.
