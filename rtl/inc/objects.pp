{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    Objects.pas clone for Free Pascal

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{************[ SOURCE FILE OF FREE VISION ]****************}
{                                                          }
{    System independent clone of objects.pas               }
{                                                          }
{    Interface Copyright (c) 1992 Borland International    }
{                                                          }
{    Parts Copyright (c) 1999-2000 by Florian Klaempfl     }
{    fnklaemp@cip.ft.uni-erlangen.de                       }
{                                                          }
{    Parts Copyright (c) 1999-2000 by Frank ZAGO           }
{    zago@ecoledoc.ipc.fr                                  }
{                                                          }
{    Parts Copyright (c) 1999-2000 by MH Spiegel           }
{                                                          }
{    Parts Copyright (c) 1996, 1999-2000 by Leon de Boer   }
{    ldeboer@ibm.net                                       }
{                                                          }
{    Free Vision project coordinator Balazs Scheidler      }
{    bazsi@tas.vein.hu                                     }
{                                                          }
UNIT Objects;

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                  INTERFACE
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

{==== Select assembler ==============================================}
{$IFDEF CPU86}
  {$ASMMODE ATT}
{$ENDIF}


{==== Compiler directives ===========================================}
{$H-} { No ansistrings }
{$X+} { Extended syntax is ok }
{$R-} { Disable range checking }
{$ifndef Unix}
  {$S-} { Disable Stack Checking }
{$endif}
{$I-} { Disable IO Checking }
{$Q-} { Disable Overflow Checking }
{$V-} { Turn off strict VAR strings }
{$INLINE ON} {Turn on inlining.}
{====================================================================}

{$ifdef win32}
  uses
    Windows;
{$endif}

{***************************************************************************}
{                             PUBLIC CONSTANTS                              }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                          STREAM ERROR STATE MASKS                         }
{---------------------------------------------------------------------------}
CONST
   stOk         =  0;                                 { No stream error }
   stError      = -1;                                 { Access error }
   stInitError  = -2;                                 { Initialize error }
   stReadError  = -3;                                 { Stream read error }
   stWriteError = -4;                                 { Stream write error }
   stGetError   = -5;                                 { Get object error }
   stPutError   = -6;                                 { Put object error }
   stSeekError  = -7;                                 { Seek error in stream }
   stOpenError  = -8;                                 { Error opening stream }

{---------------------------------------------------------------------------}
{                        STREAM ACCESS MODE CONSTANTS                       }
{---------------------------------------------------------------------------}
CONST
   stCreate    = $3C00;                               { Create new file }
   stOpenRead  = $3D00;                               { Read access only }
   stOpenWrite = $3D01;                               { Write access only }
   stOpen      = $3D02;                               { Read/write access }

{---------------------------------------------------------------------------}
{                          TCollection ERROR CODES                          }
{---------------------------------------------------------------------------}
CONST
   coIndexError = -1;                                 { Index out of range }
   coOverflow   = -2;                                 { Overflow }

{---------------------------------------------------------------------------}
{         VMT HEADER CONSTANT - HOPEFULLY WE CAN DROP THIS LATER            }
{---------------------------------------------------------------------------}
CONST
   vmtHeaderSize = 8;                                 { VMT header size }

CONST
{---------------------------------------------------------------------------}
{                            MAXIUM DATA SIZES                              }
{---------------------------------------------------------------------------}
{$IFDEF FPC}
   MaxBytes = 128*1024*128;                               { Maximum data size }
{$ELSE}
   MaxBytes = 16384;
{$ENDIF}
   MaxWords = MaxBytes DIV SizeOf(Word);              { Max word data size }
   MaxPtrs = MaxBytes DIV SizeOf(Pointer);            { Max ptr data size }
   MaxCollectionSize = MaxBytes DIV SizeOf(Pointer);  { Max collection size }
   MaxTPCompatibleCollectionSize = 65520 div 4;

{***************************************************************************}
{                          PUBLIC TYPE DEFINITIONS                          }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                               CHARACTER SET                               }
{---------------------------------------------------------------------------}
TYPE
   TCharSet = SET Of Char;                            { Character set }
   PCharSet = ^TCharSet;                              { Character set ptr }

{---------------------------------------------------------------------------}
{                               GENERAL ARRAYS                              }
{---------------------------------------------------------------------------}
TYPE
   TByteArray = ARRAY [0..MaxBytes-1] Of Byte;        { Byte array }
   PByteArray = ^TByteArray;                          { Byte array pointer }

   TWordArray = ARRAY [0..MaxWords-1] Of Word;        { Word array }
   PWordArray = ^TWordArray;                          { Word array pointer }

   TPointerArray = Array [0..MaxPtrs-1] Of Pointer;   { Pointer array }
   PPointerArray = ^TPointerArray;                    { Pointer array ptr }

{---------------------------------------------------------------------------}
{                             POINTER TO STRING                             }
{---------------------------------------------------------------------------}
TYPE
   PString = PShortString;                            { String pointer }

{---------------------------------------------------------------------------}
{                    OS dependent File type / consts                        }
{---------------------------------------------------------------------------}
type
   FNameStr = String;
const
   MaxReadBytes = $7fffffff;

var
   invalidhandle : THandle;


{---------------------------------------------------------------------------}
{                            DOS ASCIIZ FILENAME                            }
{---------------------------------------------------------------------------}
TYPE
   AsciiZ = Array [0..255] Of Char;                   { Filename array }

{---------------------------------------------------------------------------}
{                        BIT SWITCHED TYPE CONSTANTS                        }
{---------------------------------------------------------------------------}
TYPE
   Sw_Word    = Cardinal;                             { Long Word now }
   Sw_Integer = LongInt;                              { Long integer now }

{***************************************************************************}
{                        PUBLIC RECORD DEFINITIONS                          }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                          TYPE CONVERSION RECORDS                          }
{---------------------------------------------------------------------------}
TYPE
   WordRec = packed RECORD
{$ifdef ENDIAN_LITTLE}
     Lo, Hi: Byte;                                    { Word to bytes }
{$else}
     Hi,Lo: Byte;
{$endif}
   END;

   LongRec = packed RECORD
{$ifdef ENDIAN_LITTLE}
     Lo, Hi: Word;                                    { LongInt to words }
{$else}
     Hi,Lo: Word;                                    { LongInt to words }
{$endif}
   END;

   PtrRec = packed RECORD
     Ofs, Seg: Word;                                  { Pointer to words }
   END;

{---------------------------------------------------------------------------}
{                  TStreamRec RECORD - STREAM OBJECT RECORD                 }
{---------------------------------------------------------------------------}
TYPE
   PStreamRec = ^TStreamRec;                          { Stream record ptr }
   TStreamRec = Packed RECORD
      ObjType: Sw_Word;                               { Object type id }
      VmtLink: pointer;                               { VMT link }
      Load : Pointer;                                 { Object load code }
      Store: Pointer;                                 { Object store code }
      Next : PStreamRec;                              { Next stream record }
   END;

{***************************************************************************}
{                        PUBLIC OBJECT DEFINITIONS                          }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                        TPoint OBJECT - POINT OBJECT                       }
{---------------------------------------------------------------------------}
TYPE
   PPoint = ^TPoint;
   TPoint = OBJECT
      X, Y: Sw_Integer;
   END;

{---------------------------------------------------------------------------}
{                      TRect OBJECT - RECTANGLE OBJECT                      }
{---------------------------------------------------------------------------}
   PRect = ^TRect;
   TRect = OBJECT
      A, B: TPoint;                                { Corner points }
      FUNCTION Empty: Boolean;
      FUNCTION Equals (R: TRect): Boolean;
      FUNCTION Contains (P: TPoint): Boolean;
      PROCEDURE Copy (R: TRect);
      PROCEDURE Union (R: TRect);
      PROCEDURE Intersect (R: TRect);
      PROCEDURE Move (ADX, ADY: Sw_Integer);
      PROCEDURE Grow (ADX, ADY: Sw_Integer);
      PROCEDURE Assign (XA, YA, XB, YB: Sw_Integer);
   END;

{---------------------------------------------------------------------------}
{                  TObject OBJECT - BASE ANCESTOR OBJECT                    }
{---------------------------------------------------------------------------}
TYPE
   TObject = OBJECT
      CONSTRUCTOR Init;
      PROCEDURE Free;
      FUNCTION Is_Object(P:Pointer):Boolean;
      DESTRUCTOR Done;                                               Virtual;
   END;
   PObject = ^TObject;

{ ******************************* REMARK ****************************** }
{  Two new virtual methods have been added to the object in the form of }
{  Close and Open. The main use here is in the Disk Based Descendants   }
{  the calls open and close the given file so these objects can be      }
{  used like standard files. Two new fields have also been added to     }
{  speed up seeks on descendants. All existing code will compile and    }
{  work completely normally oblivious to these new methods and fields.  }
{ ****************************** END REMARK *** Leon de Boer, 15May96 * }

{---------------------------------------------------------------------------}
{                 TStream OBJECT - STREAM ANCESTOR OBJECT                   }
{---------------------------------------------------------------------------}
TYPE
   TStream = OBJECT (TObject)
         Status    : Integer;                         { Stream status }
         ErrorInfo : Integer;                         { Stream error info }
         StreamSize: LongInt;                         { Stream current size }
         Position  : LongInt;                         { Current position }
         TPCompatible : Boolean;
      CONSTRUCTOR Init;
      FUNCTION Get: PObject;
      FUNCTION StrRead: PChar;
      FUNCTION GetPos: Longint;                                      Virtual;
      FUNCTION GetSize: Longint;                                     Virtual;
      FUNCTION ReadStr: PString;
      PROCEDURE Open (OpenMode: Word);                               Virtual;
      PROCEDURE Close;                                               Virtual;
      PROCEDURE Reset;
      PROCEDURE Flush;                                               Virtual;
      PROCEDURE Truncate;                                            Virtual;
      PROCEDURE Put (P: PObject);
      PROCEDURE StrWrite (P: PChar);
      PROCEDURE WriteStr (P: PString);
      PROCEDURE Seek (Pos: LongInt);                                 Virtual;
      PROCEDURE Error (Code, Info: Integer);                         Virtual;
      PROCEDURE Read (Var Buf; Count: LongInt);                      Virtual;
      PROCEDURE Write (Var Buf; Count: LongInt);                     Virtual;
      PROCEDURE CopyFrom (Var S: TStream; Count: Longint);
   END;
   PStream = ^TStream;

{ ******************************* REMARK ****************************** }
{   A few minor changes to this object and an extra field added called  }
{  FName which holds an AsciiZ array of the filename this allows the    }
{  streams file to be opened and closed like a normal text file. All    }
{  existing code should work without any changes.                       }
{ ****************************** END REMARK *** Leon de Boer, 19May96 * }

{---------------------------------------------------------------------------}
{                TDosStream OBJECT - DOS FILE STREAM OBJECT                 }
{---------------------------------------------------------------------------}
TYPE
   TDosStream = OBJECT (TStream)
         Handle: THandle;                             { DOS file handle }
         FName : AsciiZ;                              { AsciiZ filename }
      CONSTRUCTOR Init (FileName: FNameStr; Mode: Word);
      DESTRUCTOR Done;                                               Virtual;
      PROCEDURE Close;                                               Virtual;
      PROCEDURE Truncate;                                            Virtual;
      PROCEDURE Seek (Pos: LongInt);                                 Virtual;
      PROCEDURE Open (OpenMode: Word);                               Virtual;
      PROCEDURE Read (Var Buf; Count: Longint);                      Virtual;
      PROCEDURE Write (Var Buf; Count: Longint);                     Virtual;
   private
      FileInfo : File;
   END;
   PDosStream = ^TDosStream;

{ ******************************* REMARK ****************************** }
{   A few minor changes to this object and an extra field added called  }
{  lastmode which holds the read or write condition last using the      }
{  speed up buffer which helps speed up the flush, position and size    }
{  functions. All existing code should work without any changes.        }
{ ****************************** END REMARK *** Leon de Boer, 19May96 * }

{---------------------------------------------------------------------------}
{                TBufStream OBJECT - BUFFERED DOS FILE STREAM               }
{---------------------------------------------------------------------------}
TYPE
   TBufStream = OBJECT (TDosStream)
         LastMode: Byte;                              { Last buffer mode }
         BufSize : Longint;                           { Buffer size }
         BufPtr  : Longint;                           { Buffer start }
         BufEnd  : Longint;                           { Buffer end }
         Buffer  : PByteArray;                        { Buffer allocated }
      CONSTRUCTOR Init (FileName: FNameStr; Mode, Size: Word);
      DESTRUCTOR Done;                                               Virtual;
      PROCEDURE Close;                                               Virtual;
      PROCEDURE Flush;                                               Virtual;
      PROCEDURE Truncate;                                            Virtual;
      PROCEDURE Seek (Pos: LongInt);                                 Virtual;
      PROCEDURE Open (OpenMode: Word);                               Virtual;
      PROCEDURE Read (Var Buf; Count: Longint);                      Virtual;
      PROCEDURE Write (Var Buf; Count: Longint);                     Virtual;
   END;
   PBufStream = ^TBufStream;

{ ******************************* REMARK ****************************** }
{  All the changes here should be completely transparent to existing    }
{  code. Basically the memory blocks do not have to be base segments    }
{  but this means our list becomes memory blocks rather than segments.  }
{  The stream will also expand like the other standard streams!!        }
{ ****************************** END REMARK *** Leon de Boer, 19May96 * }

{---------------------------------------------------------------------------}
{               TMemoryStream OBJECT - MEMORY STREAM OBJECT                 }
{---------------------------------------------------------------------------}
TYPE
   TMemoryStream = OBJECT (TStream)
         BlkCount: Longint;                           { Number of segments }
         BlkSize : Word;                              { Memory block size }
         MemSize : LongInt;                           { Memory alloc size }
         BlkList : PPointerArray;                     { Memory block list }
      CONSTRUCTOR Init (ALimit: Longint; ABlockSize: Word);
      DESTRUCTOR Done;                                               Virtual;
      PROCEDURE Truncate;                                            Virtual;
      PROCEDURE Read (Var Buf; Count: Longint);                      Virtual;
      PROCEDURE Write (Var Buf; Count: Longint);                     Virtual;
      PRIVATE
      FUNCTION ChangeListSize (ALimit: Longint): Boolean;
   END;
   PMemoryStream = ^TMemoryStream;


TYPE
  TItemList = Array [0..MaxCollectionSize - 1] Of Pointer;
  PItemList = ^TItemList;

{ ******************************* REMARK ****************************** }
{    The changes here look worse than they are. The Sw_Integer simply   }
{  switches between Integers and LongInts if switched between 16 and 32 }
{  bit code. All existing code will compile without any changes.        }
{ ****************************** END REMARK *** Leon de Boer, 10May96 * }

{---------------------------------------------------------------------------}
{              TCollection OBJECT - COLLECTION ANCESTOR OBJECT              }
{---------------------------------------------------------------------------}
   TCollection = OBJECT (TObject)
         Items: PItemList;                            { Item list pointer }
         Count: Sw_Integer;                           { Item count }
         Limit: Sw_Integer;                           { Item limit count }
         Delta: Sw_Integer;                           { Inc delta size }
      CONSTRUCTOR Init (ALimit, ADelta: Sw_Integer);
      CONSTRUCTOR Load (Var S: TStream);
      DESTRUCTOR Done;                                               Virtual;
      FUNCTION At (Index: Sw_Integer): Pointer;
      FUNCTION IndexOf (Item: Pointer): Sw_Integer;                  Virtual;
      FUNCTION GetItem (Var S: TStream): Pointer;                    Virtual;
      FUNCTION LastThat (Test: Pointer): Pointer;
      FUNCTION FirstThat (Test: Pointer): Pointer;
      PROCEDURE Pack;
      PROCEDURE FreeAll;
      PROCEDURE DeleteAll;
      PROCEDURE Free (Item: Pointer);
      PROCEDURE Insert (Item: Pointer);                              Virtual;
      PROCEDURE Delete (Item: Pointer);
      PROCEDURE AtFree (Index: Sw_Integer);
      PROCEDURE FreeItem (Item: Pointer);                            Virtual;
      PROCEDURE AtDelete (Index: Sw_Integer);
      PROCEDURE ForEach (Action: Pointer);
      PROCEDURE SetLimit (ALimit: Sw_Integer);                       Virtual;
      PROCEDURE Error (Code, Info: Integer);                         Virtual;
      PROCEDURE AtPut (Index: Sw_Integer; Item: Pointer);
      PROCEDURE AtInsert (Index: Sw_Integer; Item: Pointer);
      PROCEDURE Store (Var S: TStream);
      PROCEDURE PutItem (Var S: TStream; Item: Pointer);             Virtual;
   END;
   PCollection = ^TCollection;

{---------------------------------------------------------------------------}
{          TSortedCollection OBJECT - SORTED COLLECTION ANCESTOR            }
{---------------------------------------------------------------------------}
TYPE
   TSortedCollection = OBJECT (TCollection)
         Duplicates: Boolean;                         { Duplicates flag }
      CONSTRUCTOR Init (ALimit, ADelta: Sw_Integer);
      CONSTRUCTOR Load (Var S: TStream);
      FUNCTION KeyOf (Item: Pointer): Pointer;                       Virtual;
      FUNCTION IndexOf (Item: Pointer): Sw_Integer;                  Virtual;
      FUNCTION Compare (Key1, Key2: Pointer): Sw_Integer;            Virtual;
      FUNCTION Search (Key: Pointer; Var Index: Sw_Integer): Boolean;Virtual;
      PROCEDURE Insert (Item: Pointer);                              Virtual;
      PROCEDURE Store (Var S: TStream);
   END;
   PSortedCollection = ^TSortedCollection;

{---------------------------------------------------------------------------}
{           TStringCollection OBJECT - STRING COLLECTION OBJECT             }
{---------------------------------------------------------------------------}
TYPE
   TStringCollection = OBJECT (TSortedCollection)
      FUNCTION GetItem (Var S: TStream): Pointer;                    Virtual;
      FUNCTION Compare (Key1, Key2: Pointer): Sw_Integer;            Virtual;
      PROCEDURE FreeItem (Item: Pointer);                            Virtual;
      PROCEDURE PutItem (Var S: TStream; Item: Pointer);             Virtual;
   END;
   PStringCollection = ^TStringCollection;

{---------------------------------------------------------------------------}
{             TStrCollection OBJECT - STRING COLLECTION OBJECT              }
{---------------------------------------------------------------------------}
TYPE
   TStrCollection = OBJECT (TSortedCollection)
      FUNCTION Compare (Key1, Key2: Pointer): Sw_Integer;            Virtual;
      FUNCTION GetItem (Var S: TStream): Pointer;                    Virtual;
      PROCEDURE FreeItem (Item: Pointer);                            Virtual;
      PROCEDURE PutItem (Var S: TStream; Item: Pointer);             Virtual;
   END;
   PStrCollection = ^TStrCollection;

{ ******************************* REMARK ****************************** }
{    This is a completely >> NEW << object which holds a collection of  }
{  strings but does not alphabetically sort them. It is a very useful   }
{  object for insert ordered list boxes!                                }
{ ****************************** END REMARK *** Leon de Boer, 15May96 * }

{---------------------------------------------------------------------------}
{        TUnSortedStrCollection - UNSORTED STRING COLLECTION OBJECT         }
{---------------------------------------------------------------------------}
TYPE
   TUnSortedStrCollection = OBJECT (TStringCollection)
      PROCEDURE Insert (Item: Pointer);                              Virtual;
   END;
   PUnSortedStrCollection = ^TUnSortedStrCollection;

{---------------------------------------------------------------------------}
{         TResourceCollection OBJECT - RESOURCE COLLECTION OBJECT           }
{---------------------------------------------------------------------------}
TYPE
   TResourceCollection = OBJECT (TStringCollection)
      FUNCTION KeyOf (Item: Pointer): Pointer;                       Virtual;
      FUNCTION GetItem (Var S: TStream): Pointer;                    Virtual;
      PROCEDURE FreeItem (Item: Pointer);                            Virtual;
      PROCEDURE PutItem (Var S: TStream; Item: Pointer);             Virtual;
   END;
   PResourceCollection = ^TResourceCollection;

{---------------------------------------------------------------------------}
{                 TResourceFile OBJECT - RESOURCE FILE OBJECT               }
{---------------------------------------------------------------------------}
TYPE
   TResourceFile = OBJECT (TObject)
         Stream  : PStream;                           { File as a stream }
         Modified: Boolean;                           { Modified flag }
      CONSTRUCTOR Init (AStream: PStream);
      DESTRUCTOR Done;                                               Virtual;
      FUNCTION Count: Sw_Integer;
      FUNCTION KeyAt (I: Sw_Integer): String;
      FUNCTION Get (Key: String): PObject;
      FUNCTION SwitchTo (AStream: PStream; Pack: Boolean): PStream;
      PROCEDURE Flush;
      PROCEDURE Delete (Key: String);
      PROCEDURE Put (Item: PObject; Key: String);
      PRIVATE
         BasePos: LongInt;                            { Base position }
         IndexPos: LongInt;                           { Index position }
         Index: TResourceCollection;                  { Index collection }
   END;
   PResourceFile = ^TResourceFile;

TYPE
   TStrIndexRec = Packed RECORD
      Key : Sw_word;
      Count, Offset: Word;
   END;

   TStrIndex = Array [0..9999] Of TStrIndexRec;
   PStrIndex = ^TStrIndex;

{---------------------------------------------------------------------------}
{                 TStringList OBJECT - STRING LIST OBJECT                   }
{---------------------------------------------------------------------------}
   TStringList = OBJECT (TObject)
      CONSTRUCTOR Load (Var S: TStream);
      DESTRUCTOR Done;                                               Virtual;
      FUNCTION Get (Key: Sw_Word): String;
      PRIVATE
         Stream   : PStream;
         BasePos  : Longint;
         IndexSize: Longint;
         Index    : PStrIndex;
      PROCEDURE ReadStr (Var S: String; Offset, Skip: Longint);
   END;
   PStringList = ^TStringList;

{---------------------------------------------------------------------------}
{                 TStrListMaker OBJECT - RESOURCE FILE OBJECT               }
{---------------------------------------------------------------------------}
TYPE
   TStrListMaker = OBJECT (TObject)
      CONSTRUCTOR Init (AStrSize, AIndexSize: Sw_Word);
      DESTRUCTOR Done;                                               Virtual;
      PROCEDURE Put (Key: Sw_Word; S: String);
      PROCEDURE Store (Var S: TStream);
      PRIVATE
         StrPos   : Sw_Word;
         StrSize  : Sw_Word;
         Strings  : PByteArray;
         IndexPos : Sw_Word;
         IndexSize: Sw_Word;
         Index    : PStrIndex;
         Cur      : TStrIndexRec;
      PROCEDURE CloseCurrent;
   END;
   PStrListMaker = ^TStrListMaker;

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                      CALL HELPERS INTERFACE ROUTINES                      }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{ Constructor calls.

  Ctor     Pointer to the constructor.
  Obj      Pointer to the instance. NIL if new instance to be allocated.
  VMT      Pointer to the VMT (obtained by TypeOf()).
  returns  Pointer to the instance.
}
function CallVoidConstructor(Ctor: pointer; Obj: pointer; VMT: pointer): pointer;inline;
function CallPointerConstructor(Ctor: pointer; Obj: pointer; VMT: pointer; Param1: pointer): pointer;inline;

{ Method calls.

  Method   Pointer to the method.
  Obj      Pointer to the instance. NIL if new instance to be allocated.
  returns  Pointer to the instance.
}
function CallVoidMethod(Method: pointer; Obj: pointer): pointer;inline;
function CallPointerMethod(Method: pointer; Obj: pointer; Param1: pointer): pointer;inline;

{ Local-function/procedure calls.

  Func     Pointer to the local function (which must be far-coded).
  Frame    Frame pointer of the wrapping function.
}

function CallVoidLocal(Func: pointer; Frame: Pointer): pointer;inline;
function CallPointerLocal(Func: pointer; Frame: Pointer; Param1: pointer): pointer;inline;

{ Calls of functions/procedures local to methods.

  Func     Pointer to the local function (which must be far-coded).
  Frame    Frame pointer of the wrapping method.
  Obj      Pointer to the object that the method belongs to.
}
function CallVoidMethodLocal(Func: pointer; Frame: Pointer; Obj: pointer): pointer;inline;
function CallPointerMethodLocal(Func: pointer; Frame: Pointer; Obj: pointer; Param1: pointer): pointer;inline;


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                    DYNAMIC STRING INTERFACE ROUTINES                      }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-NewStr-------------------------------------------------------------
Allocates a dynamic string into memory. If S is nil, NewStr returns
a nil pointer, otherwise NewStr allocates Length(S)+1 bytes of memory
containing a copy of S, and returns a pointer to the string.
12Jun96 LdB
---------------------------------------------------------------------}
FUNCTION NewStr (Const S: String): PString;

{-DisposeStr---------------------------------------------------------
Disposes of a PString allocated by the function NewStr.
12Jun96 LdB
---------------------------------------------------------------------}
PROCEDURE DisposeStr (P: PString);

PROCEDURE SetStr(VAR p:pString; CONST s:STRING);


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        STREAM INTERFACE ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-Abstract-----------------------------------------------------------
Terminates program with a run-time error 211. When implementing
an abstract object type, call Abstract in those virtual methods that
must be overridden in descendant types. This ensures that any
attempt to use instances of the abstract object type will fail.
12Jun96 LdB
---------------------------------------------------------------------}
PROCEDURE Abstract;

{-RegisterObjects----------------------------------------------------
Registers the three standard objects TCollection, TStringCollection
and TStrCollection.
02Sep97 LdB
---------------------------------------------------------------------}
PROCEDURE RegisterObjects;

{-RegisterType-------------------------------------------------------
Registers the given object type with Free Vision's streams, creating
a list of known objects. Streams can only store and return these known
object types. Each registered object needs a unique stream registration
record, of type TStreamRec.
02Sep97 LdB
---------------------------------------------------------------------}
PROCEDURE RegisterType (Var S: TStreamRec);

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                    GENERAL FUNCTION INTERFACE ROUTINES                    }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{-LongMul------------------------------------------------------------
Returns the long integer value of X * Y integer values.
04Sep97 LdB
---------------------------------------------------------------------}
FUNCTION LongMul (X, Y: Integer): LongInt;

{-LongDiv------------------------------------------------------------
Returns the integer value of long integer X divided by integer Y.
04Sep97 LdB
---------------------------------------------------------------------}
FUNCTION LongDiv (X: Longint; Y: Integer): Integer;


{***************************************************************************}
{                         PUBLIC INITIALIZED VARIABLES                      }
{***************************************************************************}


CONST
{---------------------------------------------------------------------------}
{              INITIALIZED DOS/DPMI/WIN/OS2 PUBLIC VARIABLES                }
{---------------------------------------------------------------------------}
   StreamError: Pointer = Nil;                        { Stream error ptr }
   DefaultTPCompatible: Boolean = false;

{---------------------------------------------------------------------------}
{                        STREAM REGISTRATION RECORDS                        }
{---------------------------------------------------------------------------}

CONST
   RCollection: TStreamRec = (
     ObjType: 50;
     VmtLink: Ofs(TypeOf(TCollection)^);
     Load: @TCollection.Load;
     Store: @TCollection.Store;
     Next: Nil);

   RStringCollection: TStreamRec = (
     ObjType: 51;
     VmtLink: Ofs(TypeOf(TStringCollection)^);
     Load: @TStringCollection.Load;
     Store: @TStringCollection.Store;
     Next: Nil);

   RStrCollection: TStreamRec = (
     ObjType: 69;
     VmtLink: Ofs(TypeOf(TStrCollection)^);
     Load:    @TStrCollection.Load;
     Store:   @TStrCollection.Store;
     Next: Nil);

   RStringList: TStreamRec = (
     ObjType: 52;
     VmtLink: Ofs(TypeOf(TStringList)^);
     Load: @TStringList.Load;
     Store: Nil;
     Next: Nil);

   RStrListMaker: TStreamRec = (
     ObjType: 52;
     VmtLink: Ofs(TypeOf(TStrListMaker)^);
     Load: Nil;
     Store: @TStrListMaker.Store;
     Next: Nil);

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                IMPLEMENTATION
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

Uses dos;

{***************************************************************************}
{                      HELPER ROUTINES FOR CALLING                          }
{***************************************************************************}

type
  VoidLocal = function(_EBP: Pointer): pointer;
  PointerLocal = function(_EBP: Pointer; Param1: pointer): pointer;
  VoidMethodLocal = function(_EBP: Pointer): pointer;
  PointerMethodLocal = function(_EBP: Pointer; Param1: pointer): pointer;
  VoidConstructor = function(VMT: pointer; Obj: pointer): pointer;
  PointerConstructor = function(VMT: pointer; Obj: pointer; Param1: pointer): pointer;
  VoidMethod = function(Obj: pointer): pointer;
  PointerMethod = function(Obj: pointer; Param1: pointer): pointer;


function CallVoidConstructor(Ctor: pointer; Obj: pointer; VMT: pointer): pointer;inline;
begin
  CallVoidConstructor := VoidConstructor(Ctor)(Obj, VMT);
end;


function CallPointerConstructor(Ctor: pointer; Obj: pointer; VMT: pointer; Param1: pointer): pointer;inline;
{$undef FPC_CallPointerConstructor_Implemented}
begin
  {$define FPC_CallPointerConstructor_Implemented}
  CallPointerConstructor := PointerConstructor(Ctor)(Obj, VMT, Param1)
end;
{$ifndef FPC_CallPointerConstructor_Implemented}
{$error CallPointerConstructor function not implemented}
{$endif not FPC_CallPointerConstructor_Implemented}


function CallVoidMethod(Method: pointer; Obj: pointer): pointer;inline;
begin
  CallVoidMethod := VoidMethod(Method)(Obj)
end;


function CallPointerMethod(Method: pointer; Obj: pointer; Param1: pointer): pointer;inline;
{$undef FPC_CallPointerMethod_Implemented}
begin
{$define FPC_CallPointerMethod_Implemented}
  CallPointerMethod := PointerMethod(Method)(Obj, Param1)
end;
{$ifndef FPC_CallPointerMethod_Implemented}
{$error CallPointerMethod function not implemented}
{$endif not FPC_CallPointerMethod_Implemented}


function CallVoidLocal(Func: pointer; Frame: Pointer): pointer;inline;
begin
  CallVoidLocal := VoidLocal(Func)(Frame)
end;


function CallPointerLocal(Func: pointer; Frame: Pointer; Param1: pointer): pointer;inline;
begin
  CallPointerLocal := PointerLocal(Func)(Frame, Param1)
end;


function CallVoidMethodLocal(Func: pointer; Frame: Pointer; Obj: pointer): pointer;inline;
begin
  CallVoidMethodLocal := VoidMethodLocal(Func)(Frame)
end;


function CallPointerMethodLocal(Func: pointer; Frame: Pointer; Obj: pointer; Param1: pointer): pointer;inline;
begin
  CallPointerMethodLocal := PointerMethodLocal(Func)(Frame, Param1)
end;




{***************************************************************************}
{                      PRIVATE INITIALIZED VARIABLES                        }
{***************************************************************************}

{---------------------------------------------------------------------------}
{               INITIALIZED DOS/DPMI/WIN/OS2 PRIVATE VARIABLES              }
{---------------------------------------------------------------------------}
CONST
   StreamTypes: PStreamRec = Nil;                     { Stream types reg }

{***************************************************************************}
{                          PRIVATE INTERNAL ROUTINES                        }
{***************************************************************************}
{---------------------------------------------------------------------------}
{  RegisterError -> Platforms DOS/DPMI/WINDOWS/OS2 - Checked 12Jun96 LdB    }
{---------------------------------------------------------------------------}
PROCEDURE RegisterError;
BEGIN
   RunError(212);                                     { Register error }
END;


{***************************************************************************}
{                               OBJECT METHODS                              }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           TRect OBJECT METHODS                            }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
PROCEDURE CheckEmpty (Var Rect: TRect);
BEGIN
   With Rect Do Begin
     If (A.X >= B.X) OR (A.Y >= B.Y) Then Begin       { Zero or reversed }
       A.X := 0;                                      { Clear a.x }
       A.Y := 0;                                      { Clear a.y }
       B.X := 0;                                      { Clear b.x }
       B.Y := 0;                                      { Clear b.y }
     End;
   End;
END;

{--TRect--------------------------------------------------------------------}
{  Empty -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                }
{---------------------------------------------------------------------------}
FUNCTION TRect.Empty: Boolean;
BEGIN
   Empty := (A.X >= B.X) OR (A.Y >= B.Y);             { Empty result }
END;

{--TRect--------------------------------------------------------------------}
{  Equals -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB               }
{---------------------------------------------------------------------------}
FUNCTION TRect.Equals (R: TRect): Boolean;
BEGIN
   Equals := (A.X = R.A.X) AND (A.Y = R.A.Y) AND
   (B.X = R.B.X) AND (B.Y = R.B.Y);                   { Equals result }
END;

{--TRect--------------------------------------------------------------------}
{  Contains -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB             }
{---------------------------------------------------------------------------}
FUNCTION TRect.Contains (P: TPoint): Boolean;
BEGIN
   Contains := (P.X >= A.X) AND (P.X < B.X) AND
     (P.Y >= A.Y) AND (P.Y < B.Y);                    { Contains result }
END;

{--TRect--------------------------------------------------------------------}
{  Copy -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TRect.Copy (R: TRect);
BEGIN
   A := R.A;                                          { Copy point a }
   B := R.B;                                          { Copy point b }
END;

{--TRect--------------------------------------------------------------------}
{  Union -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TRect.Union (R: TRect);
BEGIN
   If (R.A.X < A.X) Then A.X := R.A.X;                { Take if smaller }
   If (R.A.Y < A.Y) Then A.Y := R.A.Y;                { Take if smaller }
   If (R.B.X > B.X) Then B.X := R.B.X;                { Take if larger }
   If (R.B.Y > B.Y) Then B.Y := R.B.Y;                { Take if larger }
END;

{--TRect--------------------------------------------------------------------}
{  Intersect -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TRect.Intersect (R: TRect);
BEGIN
   If (R.A.X > A.X) Then A.X := R.A.X;                { Take if larger }
   If (R.A.Y > A.Y) Then A.Y := R.A.Y;                { Take if larger }
   If (R.B.X < B.X) Then B.X := R.B.X;                { Take if smaller }
   If (R.B.Y < B.Y) Then B.Y := R.B.Y;                { Take if smaller }
   CheckEmpty(Self);                                  { Check if empty }
END;

{--TRect--------------------------------------------------------------------}
{  Move -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TRect.Move (ADX, ADY: Sw_Integer);
BEGIN
   Inc(A.X, ADX);                                     { Adjust A.X }
   Inc(A.Y, ADY);                                     { Adjust A.Y }
   Inc(B.X, ADX);                                     { Adjust B.X }
   Inc(B.Y, ADY);                                     { Adjust B.Y }
END;

{--TRect--------------------------------------------------------------------}
{  Grow -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TRect.Grow (ADX, ADY: Sw_Integer);
BEGIN
   Dec(A.X, ADX);                                     { Adjust A.X }
   Dec(A.Y, ADY);                                     { Adjust A.Y }
   Inc(B.X, ADX);                                     { Adjust B.X }
   Inc(B.Y, ADY);                                     { Adjust B.Y }
   CheckEmpty(Self);                                  { Check if empty }
END;

{--TRect--------------------------------------------------------------------}
{  Assign -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TRect.Assign (XA, YA, XB, YB: Sw_Integer);
BEGIN
   A.X := XA;                                         { Hold A.X value }
   A.Y := YA;                                         { Hold A.Y value }
   B.X := XB;                                         { Hold B.X value }
   B.Y := YB;                                         { Hold B.Y value }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           TObject OBJECT METHODS                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

TYPE
   DummyObject = OBJECT (TObject)                     { Internal object }
     Data: RECORD END;                                { Helps size VMT link }
   END;

{ ******************************* REMARK ****************************** }
{ I Prefer this code because it self sizes VMT link rather than using a }
{ fixed record structure thus it should work on all compilers without a }
{ specific record to match each compiler.                               }
{ ****************************** END REMARK *** Leon de Boer, 10May96 * }

{--TObject------------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TObject.Init;
VAR LinkSize: LongInt; Dummy: DummyObject;
BEGIN
   LinkSize := PtrInt(@Dummy.Data)-PtrInt(@Dummy);  { Calc VMT link size }
   FillChar(Pointer(PtrInt(@Self)+LinkSize)^,
     SizeOf(Self)-LinkSize, #0);                      { Clear data fields }
END;

{--TObject------------------------------------------------------------------}
{  Free -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TObject.Free;
BEGIN
   Dispose(PObject(@Self), Done);                     { Dispose of self }
END;

{--TObject------------------------------------------------------------------}
{  Is_Object -> Platforms DOS/DPMI/WIN/OS2 - Checked 5Mar00 DM              }
{---------------------------------------------------------------------------}
FUNCTION TObject.Is_Object(P:Pointer):Boolean;
TYPE
   PVMT=^VMT;
   PPVMT=^PVMT;
   VMT=RECORD
     Size,NegSize:Longint;
     ParentLink:PVMT;
   END;
VAR SP:PPVMT; Q:PVMT;
BEGIN
   SP:=PPVMT(@SELF);
   Q:=SP^;
   Is_Object:=False;
   While Q<>Nil Do Begin
     IF Q=P THEN Begin
       Is_Object:=True;
       Break;
     End;
     Q:=Q^.Parentlink;
   End;
END;

{--TObject------------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
DESTRUCTOR TObject.Done;
BEGIN                                                 { Abstract method }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           TStream OBJECT METHODS                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

CONSTRUCTOR TStream.Init;
BEGIN
  Status := StOK;
  ErrorInfo := 0;
  StreamSize := 0;
  Position := 0;
  TPCompatible := DefaultTPCompatible;
END;

{--TStream------------------------------------------------------------------}
{  Get -> Platforms DOS/DPMI/WIN/OS2 - Checked 02Sep97 LdB                  }
{---------------------------------------------------------------------------}
FUNCTION TStream.Get: PObject;
VAR ObjType: Sw_Word; P: PStreamRec; ObjTypeWord: Word;
BEGIN
   If TPCompatible Then Begin
     { Read 16-bit word for TP compatibility. }
     Read(ObjTypeWord, SizeOf(ObjTypeWord));
     ObjType := ObjTypeWord
   End
   else
     Read(ObjType, SizeOf(ObjType));                  { Read object type }
   If (ObjType<>0) Then Begin                         { Object registered }
     P := StreamTypes;                                { Current reg list }
     While (P <> Nil) AND (P^.ObjType <> ObjType)     { Find object type OR }
       Do P := P^.Next;                               { Find end of chain }
     If (P=Nil) Then Begin                            { Not registered }
       Error(stGetError, ObjType);                    { Obj not registered }
       Get := Nil;                                    { Return nil pointer }
     End Else
       Get :=PObject(
         CallPointerConstructor(P^.Load,Nil,P^.VMTLink, @Self))  { Call constructor }
   End Else Get := Nil;                               { Return nil pointer }
END;

{--TStream------------------------------------------------------------------}
{  StrRead -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TStream.StrRead: PChar;
VAR L: Word; P: PChar;
BEGIN
   Read(L, SizeOf(L));                                { Read length }
   If (L = 0) Then StrRead := Nil Else Begin          { Check for empty }
     GetMem(P, L + 1);                                { Allocate memory }
     If (P <> Nil) Then Begin                         { Check allocate okay }
       Read(P[0], L);                                 { Read the data }
       P[L] := #0;                                    { Terminate with #0 }
     End;
     StrRead := P;                                    { Return PChar }
   End;
END;

{--TStream------------------------------------------------------------------}
{  ReadStr -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TStream.ReadStr: PString;
VAR L: Byte; P: PString;
BEGIN
   Read(L, 1);                                        { Read string length }
   If (L > 0) Then Begin
     GetMem(P, L + 1);                                { Allocate memory }
     If (P <> Nil) Then Begin                         { Check allocate okay }
       P^[0] := Char(L);                              { Hold length }
       Read(P^[1], L);                                { Read string data }
     End;
     ReadStr := P;                                    { Return string ptr }
   End Else ReadStr := Nil;
END;

{--TStream------------------------------------------------------------------}
{  GetPos -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB               }
{---------------------------------------------------------------------------}
FUNCTION TStream.GetPos: LongInt;
BEGIN
   If (Status=stOk) Then GetPos := Position           { Return position }
     Else GetPos := -1;                               { Stream in error }
END;

{--TStream------------------------------------------------------------------}
{  GetSize -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TStream.GetSize: LongInt;
BEGIN
   If (Status=stOk) Then GetSize := StreamSize        { Return stream size }
     Else GetSize := -1;                              { Stream in error }
END;

{--TStream------------------------------------------------------------------}
{  Close -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Close;
BEGIN                                                 { Abstract method }
END;

{--TStream------------------------------------------------------------------}
{  Reset -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Reset;
BEGIN
   Status := stOK;                                       { Clear status }
   ErrorInfo := 0;                                    { Clear error info }
END;

{--TStream------------------------------------------------------------------}
{  Flush -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Flush;
BEGIN                                                 { Abstract method }
END;

{--TStream------------------------------------------------------------------}
{  Truncate -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Truncate;
BEGIN
   Abstract;                                          { Abstract error }
END;

{--TStream------------------------------------------------------------------}
{  Put -> Platforms DOS/DPMI/WIN/OS2 - Checked 02Sep97 LdB                  }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Put (P: PObject);
VAR ObjType: Sw_Word; Link: pointer; Q: PStreamRec; VmtPtr: ^pointer;
    ObjTypeWord: Word;
BEGIN
   VmtPtr := Pointer(P);                              { Xfer object to ptr }
   if assigned(vmtptr) then
    Link := VmtPtr^                                   { VMT link }
   else
    Link:=nil;
   ObjType := 0;                                      { Set objtype to zero }
   If (P<>Nil) AND (Link<>Nil) Then Begin               { We have a VMT link }
     Q := StreamTypes;                                { Current reg list }
     While (Q <> Nil) AND (Q^.VMTLink <> Link)        { Find link match OR }
       Do Q := Q^.Next;                               { Find end of chain }
     If (Q=Nil) Then Begin                            { End of chain found }
       Error(stPutError, 0);                          { Not registered error }
       Exit;                                          { Now exit }
     End Else ObjType := Q^.ObjType;                  { Update object type }
   End;
   If TPCompatible Then Begin
     ObjTypeWord := word(ObjType);
     Write(ObjTypeWord, SizeOf(ObjTypeWord))
   end
   else
     Write(ObjType, SizeOf(ObjType));                 { Write object type }
   If (ObjType<>0) Then                               { Registered object }
     CallPointerMethod(Q^.Store, P, @Self);
END;

{--TStream------------------------------------------------------------------}
{  Seek -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Seek (Pos: LongInt);
BEGIN
   If (Status = stOk) Then Begin                      { Check status }
     If (Pos < 0) Then Pos := 0;                      { Remove negatives }
     If (Pos <= StreamSize) Then Position := Pos      { If valid set pos }
       Else Error(stSeekError, Pos);                  { Position error }
   End;
END;

{--TStream------------------------------------------------------------------}
{  StrWrite -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TStream.StrWrite (P: PChar);
VAR L: Word; Q: PByteArray;
BEGIN
   L := 0;                                            { Preset zero size }
   Q := PByteArray(P);                                { Transfer type }
   If (Q <> Nil) Then While (Q^[L] <> 0) Do Inc(L);   { PChar length }
   Write(L, SizeOf(L));                               { Store length }
   If (P <> Nil) Then Write(P[0], L);                 { Write data }
END;

{--TStream------------------------------------------------------------------}
{  WriteStr -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TStream.WriteStr (P: PString);
CONST Empty: String[1] = '';
BEGIN
   If (P <> Nil) Then Write(P^, Length(P^) + 1)       { Write string }
     Else Write(Empty, 1);                            { Write empty string }
END;

{--TStream------------------------------------------------------------------}
{  Open -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Open (OpenMode: Word);
BEGIN                                                 { Abstract method }
END;

{--TStream------------------------------------------------------------------}
{  Error -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Error (Code, Info: Integer);
TYPE TErrorProc = Procedure(Var S: TStream);
BEGIN
   Status := Code;                                    { Hold error code }
   ErrorInfo := Info;                                 { Hold error info }
   If (StreamError <> Nil) Then
     TErrorProc(StreamError)(Self);                   { Call error ptr }
END;

{--TStream------------------------------------------------------------------}
{  Read -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Read (Var Buf; Count: Longint);
BEGIN
   Abstract;                                          { Abstract error }
END;

{--TStream------------------------------------------------------------------}
{  Write -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TStream.Write (Var Buf; Count: Longint);
BEGIN
   Abstract;                                          { Abstract error }
END;

{--TStream------------------------------------------------------------------}
{  CopyFrom -> Platforms DOS/DPMI/WIN/OS2 - Checked 10May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TStream.CopyFrom (Var S: TStream; Count: Longint);
VAR W: Word; Buffer: Array[0..1023] of Byte;
BEGIN
   While (Count > 0) Do Begin
     If (Count > SizeOf(Buffer)) Then                 { To much data }
       W := SizeOf(Buffer) Else W := Count;           { Size to transfer }
     S.Read(Buffer, W);                               { Read from stream }
     Write(Buffer, W);                                { Write to stream }
     Dec(Count, W);                                   { Dec write count }
   End;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                         TDosStream OBJECT METHODS                         }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{$IFOPT I+}
{$DEFINE IO_CHECK_ON}
{$I-}
{$ENDIF}

{--TDosStream---------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/OS2 - Checked 16May96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TDosStream.Init (FileName: FNameStr; Mode: Word);
VAR OldFileMode : Byte;
    DosStreamError : Word;
BEGIN
   Inherited Init;                                    { Call ancestor }
   FileName := FileName+#0;                           { Make asciiz }
   Move(FileName[1], FName, Length(FileName));        { Create asciiz name }
   Handle := InvalidHandle;
   Assign(FileInfo,FileName);
   { Handle the mode }
   if Mode =stCreate then
     Begin
       Rewrite(FileInfo,1);
     end
   else
     Begin
       OldFileMode := FileMode;
       { Keep sharing modes! }
       FileMode := Mode and $FF;
       System.Reset(FileInfo,1);
       FileMode := OldFileMode;
       { To use the correct mode we must reclose the file
         and open it again
       }
     end;
   Handle := FileRec(FileInfo).Handle;                { Set handle value   }
   DosStreamError := IOResult;
   If DosStreamError = 0 then
     Begin
       StreamSize := System.FileSize(FileInfo);
     end;
   If DosStreamError = 0 then
       DosStreamError := IOResult;
   If (DosStreamError <> 0) Then
     Error(stInitError, DosStreamError)              { Call stream error }
   else
     Status := StOK;
END;

{--TDosStream---------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/OS2 - Checked 16May96 LdB                 }
{---------------------------------------------------------------------------}
DESTRUCTOR TDosStream.Done;
var
  DosStreamError : Word;
BEGIN
   if Handle <> InvalidHandle then
     Begin
       System.Close(FileInfo);
       DosStreamError := IOResult;
       If DosStreamError = 0 then
          Status := stOk
       else
          Error(stError, DosStreamError);
     end;
   Position := 0;                                     { Zero the position }
   Handle := InvalidHandle;
   Inherited Done;                                    { Call ancestor }
END;

{--TDosStream---------------------------------------------------------------}
{  Close -> Platforms DOS/DPMI/WIN/OS2 - Checked 16May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TDosStream.Close;
var
  DosStreamError : Word;
BEGIN
   if Handle <> InvalidHandle then                    { Is file closed ?   }
     Begin
       System.Close(FileInfo);                        { Close file         }
       DosStreamError := IOResult;                    { Check for error    }
       If DosStreamError = 0 then
          Status := stOk
       else
          Error(stError, DosStreamError);             { Call error routine }
     end;
   Position := 0;                                     { Zero the position }
   Handle := InvalidHandle;                           { Handle invalid    }
END;

{--TDosStream---------------------------------------------------------------}
{  Truncate -> Platforms DOS/DPMI/WIN/OS2 - Checked 16May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TDosStream.Truncate;
 var
  DosStreamError : Word;
BEGIN
   If Status = stOk then
     Begin
       System.Truncate(FileInfo);
       DosStreamError := IOResult;
       If DosStreamError = 0 then
           { Status is already = stOK }
           StreamSize := Position
       else
         Error(stError, DosStreamError);
     end;
END;

{--TDosStream---------------------------------------------------------------}
{  Seek -> Platforms DOS/DPMI/WIN/OS2 - Checked 16May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TDosStream.Seek (Pos: Longint);
var
 DosStreamError : Word;
BEGIN
   If (Status=stOk) Then
     Begin                                  { Check status okay }
       If (Pos < 0) Then
             Pos := 0;                      { Negatives removed }
       System.Seek(FileInfo, Pos);
       DosStreamError := IOResult;
       if DosStreamError <> 0 then
          Error(stSeekError, DosStreamError){ Specific seek error }
       Else Position := Pos;                 { Adjust position     }
       { Status is already = stOK }
    End;
END;

{--TDosStream---------------------------------------------------------------}
{  Open -> Platforms DOS/DPMI/WIN/OS2 - Checked 16May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TDosStream.Open (OpenMode: Word);
VAR OldFileMode : Byte;
    DosStreamError : Word;
BEGIN
   If (Status=stOk) Then
     Begin                        { Check status okay }
     If (Handle = InvalidHandle) Then
        Begin                      { File not open }
          Assign(FileInfo,FName);
          { Handle the mode }
          if OpenMode =stCreate then
            Begin
              System.Rewrite(FileInfo,1);
            end
           else
             Begin
               OldFileMode := FileMode;
               FileMode := OpenMode and 3;
               System.Reset(FileInfo,1);
               FileMode := OldFileMode;
               { To use the correct mode we must reclose the file
                 and open it again
               }
           end;
           Handle := FileRec(FileInfo).Handle;                { Set handle value   }
           DosStreamError := IOResult;
           If DosStreamError = 0 then
              StreamSize := System.FileSize(FileInfo);
           If DosStreamError = 0 then
              DosStreamError := IOResult;
           If (DosStreamError <> 0) Then
              Error(stOpenError, DosStreamError)              { Call stream error }
           else
              Status := StOK;
           Position := 0;
         end
      Else
          Error(stOpenError, 104);                { File already open }
   End;
END;

{--TDosStream---------------------------------------------------------------}
{  Read -> Platforms DOS/DPMI/WIN/OS2 - Checked 16May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TDosStream.Read (Var Buf; Count: Longint);
VAR BytesMoved: Longint;
    DosStreamError : Word;
BEGIN
   If Status = StOK then
     Begin
        If (Position + Count > StreamSize) Then            { Insufficient data }
          Error(stReadError, 0);                           { Read beyond end!!! }
        If (Handle = InvalidHandle) Then
          Error(stReadError, 103);                         { File not open }
        BlockRead(FileInfo, Buf, Count, BytesMoved);       { Read from file }
        DosStreamError := IOResult;
        If ((DosStreamError<>0) OR (BytesMoved<>Count)) Then
           Begin  { Error was detected }
             BytesMoved := 0;                              { Clear bytes moved }
             If (DosStreamError <> 0) Then
                Error(stReadError, DosStreamError)         { Specific read error }
             Else
                Error(stReadError, 0);                     { Non specific error }
           End;
        Inc(Position, BytesMoved);                         { Adjust position }
     End;
   { If there was already an error, or an error was just
     generated, fill the vuffer with NULL
   }
   If Status <> StOK then
     FillChar(Buf, Count, #0);                        { Error clear buffer }
END;

{--TDosStream---------------------------------------------------------------}
{  Write -> Platforms DOS/DPMI/WIN/OS2 - Checked 16May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TDosStream.Write (Var Buf; Count: Longint);
VAR BytesMoved: Longint;
    DosStreamError : Word;
BEGIN
   { If status is not OK, simply exit }
   if Status <> StOK then
     exit;
   If (Handle = InvalidHandle) Then
    Error(stWriteError, 103);                    { File not open }
   BlockWrite(FileInfo, Buf, Count, BytesMoved); { Write to file }
   DosStreamError := IOResult;
   If ((DosStreamError<>0) OR (BytesMoved<>Count)) Then
      Begin  { Error was detected }
          BytesMoved := 0;                               { Clear bytes moved }
          If (DosStreamError<>0) Then
             Error(stWriteError, DOSStreamError)     { Specific write error }
          Else
             Error(stWriteError, 0);                 { Non specific error }
      End;
   Inc(Position, BytesMoved);                       { Adjust position }
   If (Position > StreamSize) Then                  { File expanded }
     StreamSize := Position;                        { Adjust stream size }
END;



{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                         TBufStream OBJECT METHODS                         }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TBufStream---------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TBufStream.Init (FileName: FNameStr; Mode, Size: Word);
BEGIN
   Inherited Init(FileName, Mode);                    { Call ancestor }
   BufSize := Size;                                   { Hold buffer size }
   If (Size<>0) Then GetMem(Buffer, Size);            { Allocate buffer }
   If (Buffer=Nil) Then Error(stInitError, 0);        { Buffer allocate fail }
END;

{--TBufStream---------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB                 }
{---------------------------------------------------------------------------}
DESTRUCTOR TBufStream.Done;
BEGIN
   Flush;                                             { Flush the file }
   Inherited Done;                                    { Call ancestor }
   If (Buffer<>Nil) Then FreeMem(Buffer, BufSize);    { Release buffer }
END;

{--TBufStream---------------------------------------------------------------}
{  Close -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TBufStream.Close;
BEGIN
   Flush;                                             { Flush the buffer }
   Inherited Close;                                   { Call ancestor }
END;

{--TBufStream---------------------------------------------------------------}
{  Flush -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TBufStream.Flush;
VAR W: Longint;
    DosStreamError : Word;
BEGIN
   If Status <> StOK then
      exit;
   If (LastMode=2) AND (BufPtr<>0) Then Begin         { Must update file }
     If (Handle = InvalidHandle) Then DosStreamError := 103  { File is not open }
       Else
         Begin
           BlockWrite(FileInfo, Buffer^,BufPtr, W);     { Write to file }
           DosStreamError := IOResult;
         End;
     If (DosStreamError<>0) OR (W<>BufPtr) Then       { We have an error }
       If (DosStreamError=0) Then Error(stWriteError, 0){ Unknown write error }
         Else Error(stError, DosStreamError);         { Specific write error }
   End;
   BufPtr := 0;                                       { Reset buffer ptr }
   BufEnd := 0;                                       { Reset buffer end }
END;

{--TBufStream---------------------------------------------------------------}
{  Truncate -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TBufStream.Truncate;
BEGIN
   Flush;                                             { Flush buffer }
   Inherited Truncate;                                { Truncate file }
END;

{--TBufStream---------------------------------------------------------------}
{  Seek -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TBufStream.Seek (Pos: LongInt);
BEGIN
   If (Status=stOk) Then Begin                        { Check status okay }
     If (Position<>Pos) Then Begin                    { Move required }
       Flush;                                         { Flush the buffer }
       Inherited Seek(Pos);                           { Call ancestor }
     End;
   End;
END;

{--TBufStream---------------------------------------------------------------}
{  Open -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TBufStream.Open (OpenMode: Word);
BEGIN
   If (Status=stOk) Then Begin                        { Check status okay }
     BufPtr := 0;                                     { Clear buffer start }
     BufEnd := 0;                                     { Clear buffer end }
     Inherited Open(OpenMode);                        { Call ancestor }
   End;
END;

{--TBufStream---------------------------------------------------------------}
{  Read -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TBufStream.Read (Var Buf; Count: Longint);
VAR W, Bw: Longint; P: PByte;
    DosStreamError : Word;
BEGIN
   If Status <> StOk then
     begin
       FillChar(Buf, Count, #0);                       { Error clear buffer }
       exit;
     end;
   If (Position + Count > StreamSize) Then            { Read pas stream end }
     Error(stReadError, 0);                           { Call stream error }
   If (Handle = InvalidHandle) Then Error(stReadError, 103);     { File not open }
   P := @Buf;                                         { Transfer address }
   If (LastMode=2) Then Flush;                        { Flush write buffer }
   LastMode := 1;                                     { Now set read mode }
   While (Count>0) AND (Status=stOk) Do Begin         { Check status & count }
     If (BufPtr=BufEnd) Then Begin                    { Buffer is empty }
       If (Position + BufSize > StreamSize) Then
         Bw := StreamSize - Position                  { Amount of file left }
         Else Bw := BufSize;                          { Full buffer size }
       BlockRead(FileInfo, Buffer^, Bw, W);
       DosStreamError := IOResult;                    { Read from file }
       If ((DosStreamError<>0) OR (Bw<>W)) Then Begin { Error was detected }
       If (DosStreamError<>0) Then
         Error(stReadError, DosStreamError)           { Specific read error }
         Else Error(stReadError, 0);                  { Non specific error }
       End Else Begin
         BufPtr := 0;                                 { Reset BufPtr }
         BufEnd := W;                                 { End of buffer }
       End;
     End;
     If (Status=stOk) Then Begin                      { Status still okay }
       W := BufEnd - BufPtr;                          { Space in buffer }
       If (Count < W) Then W := Count;                { Set transfer size }
       Move(Buffer^[BufPtr], P^, W);                  { Data from buffer }
       Dec(Count, W);                                 { Reduce count }
       Inc(BufPtr, W);                                { Advance buffer ptr }
       Inc(P, W);                                     { Transfer address }
       Inc(Position, W);                              { Advance position }
     End;
   End;
   If (Status<>stOk) AND (Count>0) Then
     FillChar(P^, Count, #0);                         { Error clear buffer }
END;

{--TBufStream---------------------------------------------------------------}
{  Write -> Platforms DOS/DPMI/WIN/OS2 - Checked 17May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TBufStream.Write (Var Buf; Count: Longint);
VAR W: Longint;
    P: PByte;
    DosStreamError : Word;
BEGIN
   if Status <> StOK then exit;                       { Exit if error     }
   If (Handle = InvalidHandle) Then Error(stWriteError, 103);    { File not open }
   If (LastMode=1) Then Flush;                        { Flush read buffer }
   LastMode := 2;                                     { Now set write mode }
   P := @Buf;                                         { Transfer address }
   While (Count>0) AND (Status=stOk) Do Begin         { Check status & count }
     If (BufPtr=BufSize) Then Begin                   { Buffer is full }
       BlockWrite(FileInfo, Buffer^, BufSize,W);      { Write to file }
       DosStreamError := IOResult;
       If (DosStreamError<>0) OR (W<>BufSize) Then           { We have an error }
         If (DosStreamError=0) Then Error(stWriteError, 0)   { Unknown write error }
           Else Error(stError, DosStreamError);              { Specific write error }
       BufPtr := 0;                                   { Reset BufPtr }
     End;
     If (Status=stOk) Then Begin                      { Status still okay }
       W := BufSize - BufPtr;                         { Space in buffer }
       If (Count < W) Then W := Count;                { Transfer size }
       Move(P^, Buffer^[BufPtr], W);                  { Data to buffer }
       Dec(Count, W);                                 { Reduce count }
       Inc(BufPtr, W);                                { Advance buffer ptr }
       Inc(P,W);                                      { Transfer address }
       Inc(Position, W);                              { Advance position }
       If (Position > StreamSize) Then                { File has expanded }
         StreamSize := Position;                      { Update new size }
     End;
   End;
END;

{$IFDEF IO_CHECK_ON}
{$UNDEF IO_CHECK_ON}
{$I+}
{$ENDIF}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        TMemoryStream OBJECT METHODS                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TMemoryStream------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/OS2 - Checked 19May96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TMemoryStream.Init (ALimit: LongInt; ABlockSize: Word);
VAR W: Word;
BEGIN
   Inherited Init;                                    { Call ancestor }
   If (ABlockSize=0) Then BlkSize := 8192 Else        { Default blocksize }
     BlkSize := ABlockSize;                           { Set blocksize }
   If (ALimit = 0) Then W := 1 Else                   { At least 1 block }
     W := (ALimit + BlkSize - 1) DIV BlkSize;         { Blocks needed }
   If NOT ChangeListSize(W) Then                      { Try allocate blocks }
      Error(stInitError, 0);                          { Initialize error }
END;

{--TMemoryStream------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/OS2 - Checked 19May96 LdB                 }
{---------------------------------------------------------------------------}
DESTRUCTOR TMemoryStream.Done;
BEGIN
   ChangeListSize(0);                                 { Release all memory }
   Inherited Done;                                    { Call ancestor }
END;

{--TMemoryStream------------------------------------------------------------}
{  Truncate -> Platforms DOS/DPMI/WIN/OS2 - Checked 19May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TMemoryStream.Truncate;
VAR W: Word;
BEGIN
   If (Status=stOk) Then Begin                        { Check status okay }
     If (Position = 0) Then W := 1 Else               { At least one block }
       W := (Position + BlkSize - 1) DIV BlkSize;     { Blocks needed }
     If ChangeListSize(W) Then StreamSize := Position { Set stream size }
       Else Error(stError, 0);                        { Error truncating }
   End;
END;

{--TMemoryStream------------------------------------------------------------}
{  Read -> Platforms DOS/DPMI/WIN/OS2 - Checked 19May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TMemoryStream.Read (Var Buf; Count: Longint);
VAR W, CurBlock, BlockPos: Word; Li: LongInt; P, Q: PByte;
BEGIN
   If (Position + Count > StreamSize) Then            { Insufficient data }
     Error(stReadError, 0);                           { Read beyond end!!! }
   P := @Buf;                                         { Transfer address }
   While (Count>0) AND (Status=stOk) Do Begin         { Check status & count }
     CurBlock := Position DIV BlkSize;                { Current block }
     { * REMARK * - Do not shorten this, result can be > 64K }
     Li := CurBlock;                                  { Transfer current block }
     Li := Li * BlkSize;                              { Current position }
     { * REMARK END * - Leon de Boer }
     BlockPos := Position - Li;                       { Current position }
     W := BlkSize - BlockPos;                         { Current block space }
     If (W > Count) Then W := Count;                  { Adjust read size }
     Q := BlkList^[CurBlock] + BlockPos;                                     { Calc pointer }
     Move(Q^, P^, W);                                 { Move data to buffer }
     Inc(Position, W);                                { Adjust position }
     Inc(P, W);
     Dec(Count, W);                                   { Adjust count left }
   End;
   If (Count<>0) Then FillChar(P^, Count, #0);        { Error clear buffer }
END;

{--TMemoryStream------------------------------------------------------------}
{  Write -> Platforms DOS/DPMI/WIN/OS2 - Checked 19May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TMemoryStream.Write (Var Buf; Count: Longint);
VAR
  W, CurBlock, BlockPos: Word;
  Li: LongInt;
  P, Q: PByte;
BEGIN
   If (Position + Count > MemSize) Then Begin         { Expansion needed }
     If (Position + Count = 0) Then W := 1 Else       { At least 1 block }
       W := (Position+Count+BlkSize-1) DIV BlkSize;   { Blocks needed }
     If NOT ChangeListSize(W) Then
       Error(stWriteError, 0);                        { Expansion failed!!! }
   End;
   P := @Buf;                                         { Transfer address }
   While (Count>0) AND (Status=stOk) Do Begin         { Check status & count }
     CurBlock := Position DIV BlkSize;                { Current segment }
     { * REMARK * - Do not shorten this, result can be > 64K }
     Li := CurBlock;                                  { Transfer current block }
     Li := Li * BlkSize;                              { Current position }
     { * REMARK END * - Leon de Boer }
     BlockPos := Position - Li;                       { Current position }
     W := BlkSize - BlockPos;                         { Current block space }
     If (W > Count) Then W := Count;                  { Adjust write size }
     Q := BlkList^[CurBlock] + BlockPos;                                     { Calc pointer }
     Move(P^, Q^, W);                                 { Transfer data }
     Inc(Position, W);                                { Adjust position }
     Inc(P, W);
     Dec(Count, W);                                   { Adjust count left }
     If (Position > StreamSize) Then                  { File expanded }
       StreamSize := Position;                        { Adjust stream size }
   End;
END;

{***************************************************************************}
{                      TMemoryStream PRIVATE METHODS                        }
{***************************************************************************}

{--TMemoryStream------------------------------------------------------------}
{  ChangeListSize -> Platforms DOS/DPMI/WIN/OS2 - Checked 19May96 LdB       }
{---------------------------------------------------------------------------}
FUNCTION TMemoryStream.ChangeListSize (ALimit: Longint): Boolean;
VAR
  W: Longint;
  Li: LongInt;
  P: PPointerArray;
BEGIN
   If (ALimit <> BlkCount) Then Begin                 { Change is needed }
     ChangeListSize := False;                         { Preset failure }
     If (ALimit > MaxPtrs) Then Exit;                 { To many blocks req }
     If (ALimit <> 0) Then Begin                      { Create segment list }
       Li := ALimit * SizeOf(Pointer);                { Block array size }
       GetMem(P, Li);                               { Allocate memory }
       FillChar(P^, Li, #0);                        { Clear the memory }
       If (BlkCount <> 0) AND (BlkList <> Nil) Then   { Current list valid }
         If (BlkCount <= ALimit) Then Move(BlkList^,
           P^, BlkCount * SizeOf(Pointer)) Else       { Move whole old list }
           Move(BlkList^, P^, Li);                    { Move partial list }
     End Else P := Nil;                               { No new block list }
     If (ALimit < BlkCount) Then                      { Shrink stream size }
       For W := BlkCount-1 DownTo ALimit Do
         FreeMem(BlkList^[W], BlkSize);               { Release memory block }
     If (P <> Nil) AND (ALimit > BlkCount) Then Begin { Expand stream size }
       For W := BlkCount To ALimit-1 Do Begin
         GetMem(P^[W], BlkSize);             { Allocate memory }
       End;
     End;
     If (BlkCount <> 0) AND (BlkList<>Nil) Then
       FreeMem(BlkList, BlkCount * SizeOf(Pointer));  { Release old list }
     BlkList := P;                                    { Hold new block list }
     BlkCount := ALimit;                              { Hold new count }
     { * REMARK * - Do not shorten this, result can be > 64K }
     MemSize := BlkCount;                             { Block count }
     MemSize := MemSize * BlkSize;                    { Current position }
     { * REMARK END * - Leon de Boer }
   End;
   ChangeListSize := True;                            { Successful }
END;


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                       TCollection OBJECT METHODS                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TCollection--------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TCollection.Init (ALimit, ADelta: Sw_Integer);
BEGIN
   Inherited Init;                                    { Call ancestor }
   Delta := ADelta;                                   { Set increment }
   SetLimit(ALimit);                                  { Set limit }
END;

{--TCollection--------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TCollection.Load (Var S: TStream);
VAR C, I: Sw_Integer;
BEGIN
   If S.TPCompatible Then Begin
     { I ignore endianness issues here. If endianness is different,
       you can't expect binary compatible resources anyway. }
     Count := 0; S.Read(Count, Sizeof(Word));
     Limit := 0; S.Read(Limit, Sizeof(Word));
     Delta := 0; S.Read(Delta, Sizeof(Word))
   End
   Else Begin
     S.Read(Count, Sizeof(Count));                    { Read count }
     S.Read(Limit, Sizeof(Limit));                    { Read limit }
     S.Read(Delta, Sizeof(Delta));                    { Read delta }
   End;
   Items := Nil;                                      { Clear item pointer }
   C := Count;                                        { Hold count }
   I := Limit;                                        { Hold limit }
   Count := 0;                                        { Clear count }
   Limit := 0;                                        { Clear limit }
   SetLimit(I);                                       { Set requested limit }
   Count := C;                                        { Set count }
   For I := 0 To C-1 Do AtPut(I, GetItem(S));         { Get each item }
END;

{--TCollection--------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                 }
{---------------------------------------------------------------------------}
DESTRUCTOR TCollection.Done;
BEGIN
   FreeAll;                                           { Free all items }
   SetLimit(0);                                       { Release all memory }
END;

{--TCollection--------------------------------------------------------------}
{  At -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                   }
{---------------------------------------------------------------------------}
FUNCTION TCollection.At (Index: Sw_Integer): Pointer;
BEGIN
   If (Index < 0) OR (Index >= Count) Then Begin      { Invalid index }
     Error(coIndexError, Index);                      { Call error }
     At := Nil;                                       { Return nil }
   End Else At := Items^[Index];                      { Return item }
END;

{--TCollection--------------------------------------------------------------}
{  IndexOf -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TCollection.IndexOf (Item: Pointer): Sw_Integer;
VAR I: Sw_Integer;
BEGIN
   If (Count>0) Then Begin                            { Count is positive }
     For I := 0 To Count-1 Do                         { For each item }
       If (Items^[I]=Item) Then Begin                 { Look for match }
         IndexOf := I;                                { Return index }
         Exit;                                        { Now exit }
       End;
   End;
   IndexOf := -1;                                     { Return index }
END;

{--TCollection--------------------------------------------------------------}
{  GetItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TCollection.GetItem (Var S: TStream): Pointer;
BEGIN
   GetItem := S.Get;                                  { Item off stream }
END;

{--TCollection--------------------------------------------------------------}
{  LastThat -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB             }
{---------------------------------------------------------------------------}

{$IFOPT W-}
{$DEFINE STACKFRAME_OFF}
{$W+}
{$ENDIF}

FUNCTION TCollection.LastThat (Test: Pointer): Pointer;
VAR I: LongInt;

BEGIN
   For I := Count DownTo 1 Do
     Begin                   { Down from last item }
       IF Boolean(Byte(ptrint(CallPointerLocal(Test,get_caller_frame(get_frame),Items^[I-1])))) THEN
       Begin          { Test each item }
         LastThat := Items^[I-1];                     { Return item }
         Exit;                                        { Now exit }
       End;
     End;
   LastThat := Nil;                                   { None passed test }
END;


{--TCollection--------------------------------------------------------------}
{  FirstThat -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB            }
{---------------------------------------------------------------------------}
FUNCTION TCollection.FirstThat (Test: Pointer): Pointer;
VAR I: LongInt;
BEGIN
   For I := 1 To Count Do Begin                       { Up from first item }
     IF Boolean(Byte(ptrint(CallPointerLocal(Test,get_caller_frame(get_frame),Items^[I-1])))) THEN
       Begin          { Test each item }
       FirstThat := Items^[I-1];                      { Return item }
       Exit;                                          { Now exit }
     End;
   End;
   FirstThat := Nil;                                  { None passed test }
END;

{$IFDEF STACKFRAME_OFF}
{$UNDEF STACKFRAME_OFF}
{$W-}
{$ENDIF}

{--TCollection--------------------------------------------------------------}
{  Pack -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.Pack;
VAR I, J: Sw_Integer;
BEGIN
   I := 0;                                            { Initialize dest }
   J := 0;                                            { Intialize test }
   While (I<Count) AND (J<Limit) Do Begin             { Check fully packed }
     If (Items^[J]<>Nil) Then Begin                   { Found a valid item }
       If (I<>J) Then Begin
         Items^[I] := Items^[J];                      { Transfer item }
         Items^[J] := Nil;                            { Now clear old item }
       End;
       Inc(I);                                        { One item packed }
     End;
     Inc(J);                                          { Next item to test }
   End;
   If (I<Count) Then Count := I;                      { New packed count }
END;

{--TCollection--------------------------------------------------------------}
{  FreeAll -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.FreeAll;
VAR I: Sw_Integer;
BEGIN
   for I := Count-1 downto 0 do
    FreeItem(At(I));
   Count := 0;                                        { Clear item count }
END;

{--TCollection--------------------------------------------------------------}
{  DeleteAll -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB            }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.DeleteAll;
BEGIN
   Count := 0;                                        { Clear item count }
END;

{--TCollection--------------------------------------------------------------}
{  Free -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                 }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.Free (Item: Pointer);
BEGIN
   Delete(Item);                                      { Delete from list }
   FreeItem(Item);                                    { Free the item }
END;

{--TCollection--------------------------------------------------------------}
{  Insert -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.Insert (Item: Pointer);
BEGIN
   AtInsert(Count, Item);                             { Insert item }
END;

{--TCollection--------------------------------------------------------------}
{  Delete -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.Delete (Item: Pointer);
BEGIN
   AtDelete(IndexOf(Item));                           { Delete from list }
END;

{--TCollection--------------------------------------------------------------}
{  AtFree -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.AtFree (Index: Sw_Integer);
VAR Item: Pointer;
BEGIN
   Item := At(Index);                                 { Retreive item ptr }
   AtDelete(Index);                                   { Delete item }
   FreeItem(Item);                                    { Free the item }
END;

{--TCollection--------------------------------------------------------------}
{  FreeItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.FreeItem (Item: Pointer);
VAR P: PObject;
BEGIN
   P := PObject(Item);                                { Convert pointer }
   If (P<>Nil) Then Dispose(P, Done);                 { Dispose of object }
END;

{--TCollection--------------------------------------------------------------}
{  AtDelete -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.AtDelete (Index: Sw_Integer);
BEGIN
   If (Index >= 0) AND (Index < Count) Then Begin     { Valid index }
     Dec(Count);                                      { One less item }
     If (Count>Index) Then Move(Items^[Index+1],
      Items^[Index], (Count-Index)*Sizeof(Pointer));  { Shuffle items down }
   End Else Error(coIndexError, Index);               { Index error }
END;

{--TCollection--------------------------------------------------------------}
{  ForEach -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}

{$IFOPT W-}
{$DEFINE STACKFRAME_OFF}
{$W+}
{$ENDIF}
PROCEDURE TCollection.ForEach (Action: Pointer);
VAR I: LongInt;
BEGIN
   For I := 1 To Count Do                             { Up from first item }
    CallPointerLocal(Action,get_caller_frame(get_frame),Items^[I-1]);   { Call with each item }
END;
{$IFDEF STACKFRAME_OFF}
{$UNDEF STACKFRAME_OFF}
{$W-}
{$ENDIF}


{--TCollection--------------------------------------------------------------}
{  SetLimit -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.SetLimit (ALimit: Sw_Integer);
VAR
  AItems: PItemList;
BEGIN
   If (ALimit < Count) Then
     ALimit := Count;
   If (ALimit > MaxCollectionSize) Then
     ALimit := MaxCollectionSize;
   If (ALimit <> Limit) Then
     Begin
       If (ALimit = 0) Then
         AItems := Nil
       Else
         Begin
           GetMem(AItems, ALimit * SizeOf(Pointer));
           If (AItems<>Nil) Then
             FillChar(AItems^,ALimit * SizeOf(Pointer), #0);
         End;
       If (AItems<>Nil) OR (ALimit=0) Then
         Begin
           If (AItems <>Nil) AND (Items <> Nil) Then
             Move(Items^, AItems^, Count*SizeOf(Pointer));
           If (Limit <> 0) AND (Items <> Nil) Then
             FreeMem(Items, Limit * SizeOf(Pointer));
         end;
       Items := AItems;
       Limit := ALimit;
     End;
END;

{--TCollection--------------------------------------------------------------}
{  Error -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.Error (Code, Info: Integer);
BEGIN
   RunError(212 - Code);                              { Run error }
END;

{--TCollection--------------------------------------------------------------}
{  AtPut -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.AtPut (Index: Sw_Integer; Item: Pointer);
BEGIN
   If (Index >= 0) AND (Index < Count) Then           { Index valid }
     Items^[Index] := Item                            { Put item in index }
     Else Error(coIndexError, Index);                 { Index error }
END;

{--TCollection--------------------------------------------------------------}
{  AtInsert -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.AtInsert (Index: Sw_Integer; Item: Pointer);
VAR I: Sw_Integer;
BEGIN
   If (Index >= 0) AND (Index <= Count) Then Begin    { Valid index }
     If (Count=Limit) Then SetLimit(Limit+Delta);     { Expand size if able }
     If (Limit>Count) Then Begin
       If (Index < Count) Then Begin                  { Not last item }
         For I := Count-1 DownTo Index Do               { Start from back }
           Items^[I+1] := Items^[I];                  { Move each item }
       End;
       Items^[Index] := Item;                         { Put item in list }
       Inc(Count);                                    { Inc count }
     End Else Error(coOverflow, Index);               { Expand failed }
   End Else Error(coIndexError, Index);               { Index error }
END;

{--TCollection--------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.Store (Var S: TStream);
var
  LimitWord, DeltaWord: Word;

   PROCEDURE DoPutItem (P: Pointer);{$IFNDEF FPC}FAR;{$ENDIF}
   BEGIN
     PutItem(S, P);                                   { Put item on stream }
   END;

BEGIN
   If S.TPCompatible Then Begin
    { Check if it is safe to write in TP-compatible stream.
      If Count is too big, signal an error.
      If Limit or Delta are too big, write smaller values. }
     If (Count > MaxTPCompatibleCollectionSize)
       Then S.Error(stWriteError, 0)
     Else Begin
       S.Write(Count, Sizeof(Word));
       if Limit > MaxTPCompatibleCollectionSize
       then LimitWord := MaxTPCompatibleCollectionSize
       else LimitWord := Limit;
       S.Write(LimitWord, Sizeof(Word));
       if Delta > MaxTPCompatibleCollectionSize
       then DeltaWord := MaxTPCompatibleCollectionSize
       else DeltaWord := Delta;
       S.Write(DeltaWord, Sizeof(Word));
     End
   End
   Else Begin
     S.Write(Count, Sizeof(Count));                   { Write count }
     S.Write(Limit, Sizeof(Limit));                   { Write limit }
     S.Write(Delta, Sizeof(Delta));                   { Write delta }
   End;
   ForEach(@DoPutItem);                               { Each item to stream }
END;

{--TCollection--------------------------------------------------------------}
{  PutItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TCollection.PutItem (Var S: TStream; Item: Pointer);
BEGIN
   S.Put(Item);                                       { Put item on stream }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                       TSortedCollection OBJECT METHODS                    }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TSortedCollection--------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TSortedCollection.Init (ALimit, ADelta: Sw_Integer);
BEGIN
   Inherited Init(ALimit, ADelta);                    { Call ancestor }
   Duplicates := False;                               { Clear flag }
END;

{--TSortedCollection--------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TSortedCollection.Load (Var S: TStream);
BEGIN
   Inherited Load(S);                                 { Call ancestor }
   S.Read(Duplicates, SizeOf(Duplicates));            { Read duplicate flag }
END;

{--TSortedCollection--------------------------------------------------------}
{  KeyOf -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                }
{---------------------------------------------------------------------------}
FUNCTION TSortedCollection.KeyOf (Item: Pointer): Pointer;
BEGIN
   KeyOf := Item;                                     { Return item as key }
END;

{--TSortedCollection--------------------------------------------------------}
{  IndexOf -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TSortedCollection.IndexOf (Item: Pointer): Sw_Integer;
VAR I, J: Sw_Integer;
BEGIN
   J := -1;                                           { Preset result }
   If Search(KeyOf(Item), I) Then Begin               { Search for item }
     If Duplicates Then                               { Duplicates allowed }
       While (I < Count) AND (Item <> Items^[I]) Do
         Inc(I);                                      { Count duplicates }
     If (I < Count) Then J := I;                      { Index result }
   End;
   IndexOf := J;                                      { Return result }
END;

{--TSortedCollection--------------------------------------------------------}
{  Compare -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TSortedCollection.Compare (Key1, Key2: Pointer): Sw_Integer;
BEGIN
   Abstract;                                          { Abstract method }
   Compare:=0;
END;

{--TSortedCollection--------------------------------------------------------}
{  Search -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB               }
{---------------------------------------------------------------------------}
FUNCTION TSortedCollection.Search (Key: Pointer; Var Index: Sw_Integer): Boolean;
VAR L, H, I, C: Sw_Integer;
BEGIN
   Search := False;                                   { Preset failure }
   L := 0;                                            { Start count }
   H := Count - 1;                                    { End count }
   Index := 0;
   if Count=0 then
     exit;
   While (L <= H) Do Begin
     I := (L + H) SHR 1;                              { Mid point }
     C := Compare(KeyOf(Items^[I]), Key);             { Compare with key }
     If (C < 0) Then L := I + 1 Else Begin            { Item to left }
       H := I - 1;                                    { Item to right }
       If C = 0 Then Begin                            { Item match found }
         Search := True;                              { Result true }
         If NOT Duplicates Then L := I;               { Force kick out }
       End;
     End;
   End;
   Index := L;                                        { Return result }
END;

{--TSortedCollection--------------------------------------------------------}
{  Insert -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TSortedCollection.Insert (Item: Pointer);
VAR I: Sw_Integer;
BEGIN
   If NOT Search(KeyOf(Item), I) OR Duplicates Then   { Item valid }
     AtInsert(I, Item);                               { Insert the item }
END;

{--TSortedCollection--------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TSortedCollection.Store (Var S: TStream);
BEGIN
   TCollection.Store(S);                              { Call ancestor }
   S.Write(Duplicates, SizeOf(Duplicates));           { Write duplicate flag }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                     TStringCollection OBJECT METHODS                      }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TStringCollection--------------------------------------------------------}
{  GetItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TStringCollection.GetItem (Var S: TStream): Pointer;
BEGIN
   GetItem := S.ReadStr;                              { Get new item }
END;

{--TStringCollection--------------------------------------------------------}
{  Compare -> Platforms DOS/DPMI/WIN/OS2 - Checked 21Aug97 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TStringCollection.Compare (Key1, Key2: Pointer): Sw_Integer;
VAR I, J: Sw_Integer; P1, P2: PString;
BEGIN
   P1 := PString(Key1);                               { String 1 pointer }
   P2 := PString(Key2);                               { String 2 pointer }
   If (Length(P1^)<Length(P2^)) Then J := Length(P1^)
     Else J := Length(P2^);                           { Shortest length }
   I := 1;                                            { First character }
   While (I<J) AND (P1^[I]=P2^[I]) Do Inc(I);         { Scan till fail }
   If (I=J) Then Begin                                { Possible match }
   { * REMARK * - Bug fix   21 August 1997 }
     If (P1^[I]<P2^[I]) Then Compare := -1 Else       { String1 < String2 }
       If (P1^[I]>P2^[I]) Then Compare := 1 Else      { String1 > String2 }
       If (Length(P1^)>Length(P2^)) Then Compare := 1 { String1 > String2 }
         Else If (Length(P1^)<Length(P2^)) Then       { String1 < String2 }
           Compare := -1 Else Compare := 0;           { String1 = String2 }
   { * REMARK END * - Leon de Boer }
   End Else If (P1^[I]<P2^[I]) Then Compare := -1     { String1 < String2 }
     Else Compare := 1;                               { String1 > String2 }
END;

{--TStringCollection--------------------------------------------------------}
{  FreeItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TStringCollection.FreeItem (Item: Pointer);
BEGIN
   DisposeStr(Item);                                  { Dispose item }
END;

{--TStringCollection--------------------------------------------------------}
{  PutItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 22May96 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TStringCollection.PutItem (Var S: TStream; Item: Pointer);
BEGIN
   S.WriteStr(Item);                                  { Write string }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                       TStrCollection OBJECT METHODS                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TStrCollection-----------------------------------------------------------}
{  Compare -> Platforms DOS/DPMI/WIN/OS2 - Checked 23May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TStrCollection.Compare (Key1, Key2: Pointer): Sw_Integer;
VAR I, J: Sw_Integer; P1, P2: PByteArray;
BEGIN
   P1 := PByteArray(Key1);                            { PChar 1 pointer }
   P2 := PByteArray(Key2);                            { PChar 2 pointer }
   I := 0;                                            { Preset no size }
   If (P1<>Nil) Then While (P1^[I]<>0) Do Inc(I);     { PChar 1 length }
   J := 0;                                            { Preset no size }
   If (P2<>Nil) Then While (P2^[J]<>0) Do Inc(J);     { PChar 2 length }
   If (I < J) Then J := I;                            { Shortest length }
   I := 0;                                            { First character }
   While (I<J) AND (P1^[I]=P2^[I]) Do Inc(I);         { Scan till fail }
   If (P1^[I]=P2^[I]) Then Compare := 0 Else          { Strings matched }
     If (P1^[I]<P2^[I]) Then Compare := -1 Else       { String1 < String2 }
        Compare := 1;                                 { String1 > String2 }
END;

{--TStrCollection-----------------------------------------------------------}
{  GetItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 23May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TStrCollection.GetItem (Var S: TStream): Pointer;
BEGIN
   GetItem := S.StrRead;                              { Get string item }
END;

{--TStrCollection-----------------------------------------------------------}
{  FreeItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 23May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TStrCollection.FreeItem (Item: Pointer);
VAR I: Sw_Integer; P: PByteArray;
BEGIN
   If (Item<>Nil) Then Begin                          { Item is valid }
     P := PByteArray(Item);                           { Create byte pointer }
     I := 0;                                          { Preset no size }
     While (P^[I]<>0) Do Inc(I);                      { Find PChar end }
     FreeMem(Item, I+1);                              { Release memory }
   End;
END;

{--TStrCollection-----------------------------------------------------------}
{  PutItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 23May96 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TStrCollection.PutItem (Var S: TStream; Item: Pointer);
BEGIN
   S.StrWrite(Item);                                  { Write the string }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                   TUnSortedStrCollection OBJECT METHODS                   }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TUnSortedCollection------------------------------------------------------}
{  Insert -> Platforms DOS/DPMI/WIN/OS2 - Checked 23May96 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TUnSortedStrCollection.Insert (Item: Pointer);
BEGIN
   AtInsert(Count, Item);                             { Insert - NO sorting }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                           TResourceItem RECORD                            }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
TYPE
   TResourceItem = packed RECORD
      Posn: LongInt;                                  { Resource position }
      Size: LongInt;                                  { Resource size }
      Key : String;                                   { Resource key }
   End;
   PResourceItem = ^TResourceItem;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                    TResourceCollection OBJECT METHODS                     }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TResourceCollection------------------------------------------------------}
{  KeyOf -> Platforms DOS/DPMI/WIN/OS2 - Checked 24May96 LdB                }
{---------------------------------------------------------------------------}
FUNCTION TResourceCollection.KeyOf (Item: Pointer): Pointer;
BEGIN
   KeyOf := @PResourceItem(Item)^.Key;                { Pointer to key }
END;

{--TResourceCollection------------------------------------------------------}
{  GetItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 24May96 LdB              }
{---------------------------------------------------------------------------}
FUNCTION TResourceCollection.GetItem (Var S: TStream): Pointer;
VAR B: Byte; Pos: Longint; Size: Longint; Ts: String; P: PResourceItem;
BEGIN
   S.Read(Pos, SizeOf(Pos));                          { Read position }
   S.Read(Size, SizeOf(Size));                        { Read size }
   S.Read(B, 1);                                      { Read key length }
   GetMem(P, B + (SizeOf(TResourceItem) -
     SizeOf(Ts) + 1));                                { Allocate min memory }
   If (P<>Nil) Then Begin                             { If allocate works }
     P^.Posn := Pos;                                  { Xfer position }
     P^.Size := Size;                                 { Xfer size }
     P^.Key[0] := Char(B);                            { Xfer string length }
     S.Read(P^.Key[1], B);                            { Xfer string data }
   End;
   GetItem := P;                                      { Return pointer }
END;

{--TResourceCollection------------------------------------------------------}
{  FreeItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 24May96 LdB             }
{---------------------------------------------------------------------------}
PROCEDURE TResourceCollection.FreeItem (Item: Pointer);
VAR Ts: String;
BEGIN
   If (Item<>Nil) Then FreeMem(Item,
     SizeOf(TResourceItem) - SizeOf(Ts) +
     Length(PResourceItem(Item)^.Key) + 1);           { Release memory }
END;

{--TResourceCollection------------------------------------------------------}
{  PutItem -> Platforms DOS/DPMI/WIN/OS2 - Checked 24May96 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TResourceCollection.PutItem (Var S: TStream; Item: Pointer);
VAR Ts: String;
BEGIN
   If (Item<>Nil) Then S.Write(PResourceItem(Item)^,
    SizeOf(TResourceItem) - SizeOf(Ts) +
    Length(PResourceItem(Item)^.Key) + 1);            { Write to stream }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                  PRIVATE RESOURCE MANAGER CONSTANTS                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
CONST
   RStreamMagic: LongInt = $52504246;                 { 'FBPR' }
   RStreamBackLink: LongInt = $4C424246;              { 'FBBL' }

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                    PRIVATE RESOURCE MANAGER TYPES                         }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
TYPE
{$IFDEF NewExeFormat}                                 { New EXE format }
   TExeHeader = packed RECORD
     eHdrSize:   Word;
     eMinAbove:  Word;
     eMaxAbove:  Word;
     eInitSS:    Word;
     eInitSP:    Word;
     eCheckSum:  Word;
     eInitPC:    Word;
     eInitCS:    Word;
     eRelocOfs:  Word;
     eOvlyNum:   Word;
     eRelocTab:  Word;
     eSpace:     Array[1..30] of Byte;
     eNewHeader: Word;
   END;
{$ENDIF}

   THeader = packed RECORD
     Signature: Word;
     Case Integer Of
       0: (
         LastCount: Word;
         PageCount: Word;
         ReloCount: Word);
       1: (
         InfoType: Word;
         InfoSize: Longint);
   End;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                       TResourceFile OBJECT METHODS                        }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TResourceFile------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TResourceFile.Init(AStream: PStream);
VAR Found, Stop: Boolean; Header: THeader;
    {$IFDEF NewExeFormat} ExeHeader: TExeHeader; {$ENDIF}
BEGIN
   TObject.Init;                                      { Initialize object }
   Found := False;                                    { Preset false }
   If (AStream<>Nil) Then Begin
     Stream := AStream;                               { Hold stream }
     BasePos := Stream^.GetPos;                       { Get position }
     Repeat
       Stop := True;                                  { Preset stop }
       If (BasePos <= Stream^.GetSize-SizeOf(THeader))
       Then Begin                                     { Valid file header }
         Stream^.Seek(BasePos);                       { Seek to position }
         Stream^.Read(Header, SizeOf(THeader));       { Read header }
         Case Header.Signature Of
         {$IFDEF NewExeFormat}                        { New format file }
           $5A4D: Begin
             Stream^.Read(ExeHeader, SizeOf(TExeHeader));
             BasePos := ExeHeader.eNewHeader;         { Hold position }
             Stop := False;                           { Clear stop flag }
           End;
           $454E: Begin
             BasePos := Stream^.GetSize - 8;          { Hold position }
             Stop := False;                           { Clear stop flag }
           End;
           $4246: Begin
             Stop := False;                           { Clear stop flag }
             Case Header.Infotype Of
               $5250: Begin                           { Found Resource }
                   Found := True;                     { Found flag is true }
                   Stop := True;                      { Set stop flag }
                 End;
               $4C42: Dec(BasePos, Header.InfoSize-8);{ Found BackLink }
               $4648: Dec(BasePos, SizeOf(THeader)*2);{ Found HelpFile }
               Else Stop := True;                     { Set stop flag }
             End;
           End;
           $424E: If Header.InfoType = $3230          { Found Debug Info }
           Then Begin
             Dec(BasePos, Header.InfoSize);           { Adjust position }
             Stop := False;                           { Clear stop flag }
           End;
         {$ELSE}
           $5A4D: Begin
             Inc(BasePos, LongInt(Header.PageCount)*512
               - (-Header.LastCount AND 511));        { Calc position }
             Stop := False;                           { Clear stop flag }
           End;
           $4246: If Header.InfoType = $5250 Then     { Header was found }
             Found := True Else Begin
               Inc(BasePos, Header.InfoSize + 8);     { Adjust position }
               Stop := False;                         { Clear stop flag }
             End;
         {$ENDIF}
         End;
       End;
     Until Stop;                                      { Until flag is set }
   End;
   If Found Then Begin                                { Resource was found }
     Stream^.Seek(BasePos + SizeOf(LongInt) * 2);     { Seek to position }
     Stream^.Read(IndexPos, SizeOf(LongInt));         { Read index position }
     Stream^.Seek(BasePos + IndexPos);                { Seek to resource }
     Index.Load(Stream^);                             { Load resource }
   End Else Begin
     IndexPos := SizeOf(LongInt) * 3;                 { Set index position }
     Index.Init(0, 8);                                { Set index }
   End;
END;

{--TResourceFile------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB                 }
{---------------------------------------------------------------------------}
DESTRUCTOR TResourceFile.Done;
BEGIN
   Flush;                                             { Flush the file }
   Index.Done;                                        { Dispose of index }
   If (Stream<>Nil) Then Dispose(Stream, Done);       { Dispose of stream }
END;

{--TResourceFile------------------------------------------------------------}
{  Count -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB                }
{---------------------------------------------------------------------------}
FUNCTION TResourceFile.Count: Sw_Integer;
BEGIN
   Count := Index.Count;                              { Return index count }
END;

{--TResourceFile------------------------------------------------------------}
{  KeyAt -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB                }
{---------------------------------------------------------------------------}
FUNCTION TResourceFile.KeyAt (I: Sw_Integer): String;
BEGIN
   KeyAt := PResourceItem(Index.At(I))^.Key;          { Return key }
END;

{--TResourceFile------------------------------------------------------------}
{  Get -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB                  }
{---------------------------------------------------------------------------}
FUNCTION TResourceFile.Get (Key: String): PObject;
VAR I: Sw_Integer;
BEGIN
   If (Stream = Nil) OR (NOT Index.Search(@Key, I))   { No match on key }
   Then Get := Nil Else Begin
     Stream^.Seek(BasePos +
       PResourceItem(Index.At(I))^.Posn);             { Seek to position }
     Get := Stream^.Get;                              { Get item }
   End;
END;

{--TResourceFile------------------------------------------------------------}
{  SwitchTo -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB             }
{---------------------------------------------------------------------------}
FUNCTION TResourceFile.SwitchTo (AStream: PStream; Pack: Boolean): PStream;
VAR NewBasePos: LongInt;

   PROCEDURE DoCopyResource (Item: PResourceItem);{$IFNDEF FPC}FAR;{$ENDIF}
   BEGIN
     Stream^.Seek(BasePos + Item^.Posn);              { Move stream position }
     Item^.Posn := AStream^.GetPos - NewBasePos;      { Hold new position }
     AStream^.CopyFrom(Stream^, Item^.Size);          { Copy the item }
   END;

BEGIN
   SwitchTo := Stream;                                { Preset return }
   If (AStream<>Nil) AND (Stream<>Nil) Then Begin     { Both streams valid }
     NewBasePos := AStream^.GetPos;                   { Get position }
     If Pack Then Begin
       AStream^.Seek(NewBasePos + SizeOf(LongInt)*3); { Seek to position }
       Index.ForEach(@DoCopyResource);                { Copy each resource }
       IndexPos := AStream^.GetPos - NewBasePos;      { Hold index position }
     End Else Begin
       Stream^.Seek(BasePos);                         { Seek to position }
       AStream^.CopyFrom(Stream^, IndexPos);          { Copy the resource }
     End;
     Stream := AStream;                               { Hold new stream }
     BasePos := NewBasePos;                           { New base position }
     Modified := True;                                { Set modified flag }
   End;
END;

{--TResourceFile------------------------------------------------------------}
{  Flush -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TResourceFile.Flush;
VAR ResSize: LongInt; LinkSize: LongInt;
BEGIN
   If (Modified) AND (Stream<>Nil) Then Begin         { We have modification }
     Stream^.Seek(BasePos + IndexPos);                { Seek to position }
     Index.Store(Stream^);                            { Store the item }
     ResSize := Stream^.GetPos - BasePos;             { Hold position }
     LinkSize := ResSize + SizeOf(LongInt) * 2;       { Hold link size }
     Stream^.Write(RStreamBackLink, SizeOf(LongInt)); { Write link back }
     Stream^.Write(LinkSize, SizeOf(LongInt));        { Write link size }
     Stream^.Seek(BasePos);                           { Move stream position }
     Stream^.Write(RStreamMagic, SizeOf(LongInt));    { Write number }
     Stream^.Write(ResSize, SizeOf(LongInt));         { Write record size }
     Stream^.Write(IndexPos, SizeOf(LongInt));        { Write index position }
     Stream^.Flush;                                   { Flush the stream }
   End;
   Modified := False;                                 { Clear modified flag }
END;

{--TResourceFile------------------------------------------------------------}
{  Delete -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB               }
{---------------------------------------------------------------------------}
PROCEDURE TResourceFile.Delete (Key: String);
VAR I: Sw_Integer;
BEGIN
   If Index.Search(@Key, I) Then Begin                { Search for key }
     Index.Free(Index.At(I));                         { Delete from index }
     Modified := True;                                { Set modified flag }
   End;
END;

{--TResourceFile------------------------------------------------------------}
{  Put -> Platforms DOS/DPMI/WIN/OS2 - Checked 18Jun96 LdB                  }
{---------------------------------------------------------------------------}
PROCEDURE TResourceFile.Put (Item: PObject; Key: String);
VAR I: Sw_Integer; Ts: String; P: PResourceItem;
BEGIN
   If (Stream=Nil) Then Exit;                         { Stream not valid }
   If Index.Search(@Key, I) Then P := Index.At(I)     { Search for item }
   Else Begin
     GetMem(P, Length(Key) + (SizeOf(TResourceItem) -
       SizeOf(Ts) + 1));                              { Allocate memory }
     If (P<>Nil) Then Begin
       P^.Key := Key;                                 { Store key }
       Index.AtInsert(I, P);                          { Insert item }
     End;
   End;
   If (P<>Nil) Then Begin
     P^.Posn := IndexPos;                             { Set index position }
     Stream^.Seek(BasePos + IndexPos);                { Seek file position }
     Stream^.Put(Item);                               { Put item on stream }
     IndexPos := Stream^.GetPos - BasePos;            { Hold index position }
     P^.Size := IndexPos - P^.Posn;                   { Calc size }
     Modified := True;                                { Set modified flag }
   End;
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          TStringList OBJECT METHODS                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TStringList--------------------------------------------------------------}
{  Load -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TStringList.Load (Var S: TStream);
VAR Size: Word;
BEGIN
   Stream := @S;                                      { Hold stream pointer }
   S.Read(Size, SizeOf(Word));                        { Read size }
   BasePos := S.GetPos;                               { Hold position }
   S.Seek(BasePos + Size);                            { Seek to position }
   S.Read(IndexSize, SizeOf(Integer));                { Read index size }
   GetMem(Index, IndexSize * SizeOf(TStrIndexRec));   { Allocate memory }
   S.Read(Index^, IndexSize * SizeOf(TStrIndexRec));  { Read indexes }
END;

{--TStringList--------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB                 }
{---------------------------------------------------------------------------}
DESTRUCTOR TStringList.Done;
BEGIN
   FreeMem(Index, IndexSize * SizeOf(TStrIndexRec));  { Release memory }
END;

{--TStringList--------------------------------------------------------------}
{  Get -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB                  }
{---------------------------------------------------------------------------}
FUNCTION TStringList.Get (Key: Sw_Word): String;
VAR I: Word; S: String;
BEGIN
   S := '';                                           { Preset empty string }
   If (IndexSize>0) Then Begin                        { We must have strings }
     I := 0;                                          { First entry }
     While (I<IndexSize) AND (S='') Do Begin
       If ((Key - Index^[I].Key)<Index^[I].Count)     { Diff less than count }
         Then ReadStr(S, Index^[I].Offset,
           Key-Index^[I].Key);                        { Read the string }
       Inc(I);                                        { Next entry }
     End;
   End;
   Get := S;                                          { Return empty string }
END;

{***************************************************************************}
{                       TStringList PRIVATE METHODS                         }
{***************************************************************************}

{--TStringLis---------------------------------------------------------------}
{  ReadStr -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB              }
{---------------------------------------------------------------------------}
PROCEDURE TStringList.ReadStr (Var S: String; Offset, Skip: Longint);
BEGIN
   Stream^.Seek(BasePos + Offset);                    { Seek to position }
   Inc(Skip);                                         { Adjust skip }
   Repeat
     Stream^.Read(S[0], 1);                           { Read string size }
     Stream^.Read(S[1], Ord(S[0]));                   { Read string data }
     Dec(Skip);                                       { One string read }
   Until (Skip = 0);
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                         TStrListMaker OBJECT METHODS                      }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{--TStrListMaker------------------------------------------------------------}
{  Init -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB                 }
{---------------------------------------------------------------------------}
CONSTRUCTOR TStrListMaker.Init (AStrSize, AIndexSize: Sw_Word);
BEGIN
   Inherited Init;                                    { Call ancestor }
   StrSize := AStrSize;                               { Hold size }
   IndexSize := AIndexSize;                           { Hold index size }
   GetMem(Strings, AStrSize);                         { Allocate memory }
   GetMem(Index, AIndexSize * SizeOf(TStrIndexRec));  { Allocate memory }
END;

{--TStrListMaker------------------------------------------------------------}
{  Done -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB                 }
{---------------------------------------------------------------------------}
DESTRUCTOR TStrListMaker.Done;
BEGIN
   FreeMem(Index, IndexSize * SizeOf(TStrIndexRec));  { Free index memory }
   FreeMem(Strings, StrSize);                         { Free data memory }
END;

{--TStrListMaker------------------------------------------------------------}
{  Put -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB                  }
{---------------------------------------------------------------------------}
PROCEDURE TStrListMaker.Put (Key: Sw_Word; S: String);
BEGIN
   If (Cur.Count = 16) OR (Key <> Cur.Key + Cur.Count)
     Then CloseCurrent;                               { Close current }
   If (Cur.Count = 0) Then Begin
     Cur.Key := Key;                                  { Set key }
     Cur.Offset := StrPos;                            { Set offset }
   End;
   Inc(Cur.Count);                                    { Inc count }
   Move(S, Strings^[StrPos], Length(S) + 1);          { Move string data }
   Inc(StrPos, Length(S) + 1);                        { Adjust position }
END;

{--TStrListMaker------------------------------------------------------------}
{  Store -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB                }
{---------------------------------------------------------------------------}
PROCEDURE TStrListMaker.Store (Var S: TStream);
BEGIN
   CloseCurrent;                                      { Close all current }
   S.Write(StrPos, SizeOf(Word));                     { Write position }
   S.Write(Strings^, StrPos);                         { Write string data }
   S.Write(IndexPos, SizeOf(Word));                   { Write index position }
   S.Write(Index^, IndexPos * SizeOf(TStrIndexRec));  { Write indexes }
END;

{***************************************************************************}
{                      TStrListMaker PRIVATE METHODS                        }
{***************************************************************************}

{--TStrListMaker------------------------------------------------------------}
{  CloseCurrent -> Platforms DOS/DPMI/WIN/OS2 - Checked 30Jun97 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE TStrListMaker.CloseCurrent;
BEGIN
   If (Cur.Count <> 0) Then Begin
     Index^[IndexPos] := Cur;                         { Hold index position }
     Inc(IndexPos);                                   { Next index }
     Cur.Count := 0;                                  { Adjust count }
   End;
END;

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                    DYNAMIC STRING INTERFACE ROUTINES                      }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  NewStr -> Platforms DOS/DPMI/WINDOWS/OS2 - Checked 12Jun96 LdB           }
{---------------------------------------------------------------------------}
FUNCTION NewStr (Const S: String): PString;
VAR P: PString;
BEGIN
   If (S = '') Then P := Nil Else Begin               { Return nil }
     GetMem(P, Length(S) + 1);                        { Allocate memory }
     If (P<>Nil) Then P^ := S;                        { Hold string }
   End;
   NewStr := P;                                       { Return result }
END;

{---------------------------------------------------------------------------}
{  DisposeStr -> Platforms DOS/DPMI/WINDOWS/OS2 - Checked 12Jun96 LdB       }
{---------------------------------------------------------------------------}
PROCEDURE DisposeStr (P: PString);
BEGIN
   If (P <> Nil) Then FreeMem(P, Length(P^) + 1);     { Release memory }
END;


PROCEDURE SetStr(VAR p:pString; CONST s:STRING);
BEGIN
  IF p<>NIL THEN
    FreeMem(P, Length(P^) + 1);
  GetMem(p,LENGTH(s)+1);
  pSTRING(p)^ := s
END;


{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        STREAM INTERFACE ROUTINES                          }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  Abstract -> Platforms DOS/DPMI/WINDOWS/OS2 - Checked 12Jun96 LdB         }
{---------------------------------------------------------------------------}
PROCEDURE Abstract;
BEGIN
   RunError(211);                                     { Abstract error }
END;

{---------------------------------------------------------------------------}
{  RegisterObjects -> Platforms DOS/DPMI/WINDOWS/OS2 - Checked 02Sep97 LdB  }
{---------------------------------------------------------------------------}
PROCEDURE RegisterObjects;
BEGIN
   RegisterType(RCollection);                         { Register object }
   RegisterType(RStringCollection);                   { Register object }
   RegisterType(RStrCollection);                      { Register object }
END;

{---------------------------------------------------------------------------}
{  RegisterType -> Platforms DOS/DPMI/WINDOWS/OS2 - Checked 02Sep97 LdB     }
{---------------------------------------------------------------------------}
PROCEDURE RegisterType (Var S: TStreamRec);
VAR P: PStreamRec;
BEGIN
   P := StreamTypes;                                  { Current reg list }
   While (P <> Nil) AND (P^.ObjType <> S.ObjType)
     Do P := P^.Next;                                 { Find end of chain }
   If (P = Nil) AND (S.ObjType <> 0) Then Begin       { Valid end found }
     S.Next := StreamTypes;                           { Chain the list }
     StreamTypes := @S;                               { We are now first }
   End Else RegisterError;                            { Register the error }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                    GENERAL FUNCTION INTERFACE ROUTINES                    }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  LongMul -> Platforms DOS/DPMI/WINDOWS/OS2 - Checked 04Sep97 LdB          }
{---------------------------------------------------------------------------}
FUNCTION LongMul (X, Y: Integer): LongInt;
  BEGIN
    LongMul:=Longint(X*Y);
  END;
{---------------------------------------------------------------------------}
{  LongDiv -> Platforms DOS/DPMI/WINDOWS/OS2 - Checked 04Sep97 LdB          }
{---------------------------------------------------------------------------}
FUNCTION LongDiv (X: LongInt; Y: Integer): Integer;
BEGIN
  LongDiv := Integer(X DIV Y);
END;


BEGIN
  invalidhandle:=UnusedHandle;
END.
