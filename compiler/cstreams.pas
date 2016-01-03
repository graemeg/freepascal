{
    Copyright (c) 1998-2002 by Florian Klaempfl and Peter Vreman

    This module provides stream classes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit cstreams;

{$i fpcdefs.inc}

interface

   uses
     cutils;


{****************************************************************************
                                  TCStream
****************************************************************************}

    {
      TCStream is copied directly from classesh.inc from the FCL so
      it's compatible with the normal Classes.TStream.

      TCFileStream is a merge of THandleStream and TFileStream and updated
      to have a 'file' type instead of Handle.

      TCCustomMemoryStream and TCMemoryStream are direct copies.
    }
    const
       { TCStream seek origins }
       soFromBeginning = 0;
       soFromCurrent = 1;
       soFromEnd = 2;

       { TCFileStream create mode }
       fmCreate        = $FFFF;
       fmOpenRead      = 0;
       fmOpenWrite     = 1;
       fmOpenReadWrite = 2;

var
{ Used for Error reporting instead of exceptions }
  CStreamError : longint;

type
{ Fake TComponent class, it isn't used any futher }
  TCComponent = class(TObject)
  end;

{ TCStream abstract class }

  TCStream = class(TObject)
  private
    function GetPosition: Longint;
    procedure SetPosition(Pos: Longint);
    function GetSize: Longint;
  protected
    procedure SetSize(NewSize: Longint); virtual;
  public
    function Read(var Buffer; Count: Longint): Longint; virtual; abstract;
    function Write(const Buffer; Count: Longint): Longint; virtual; abstract;
    function Seek(Offset: Longint; Origin: Word): Longint; virtual; abstract;
    procedure ReadBuffer(var Buffer; Count: Longint);
    procedure WriteBuffer(const Buffer; Count: Longint);
    function CopyFrom(Source: TCStream; Count: Longint): Longint;
    function ReadComponent(Instance: TCComponent): TCComponent;
    function ReadComponentRes(Instance: TCComponent): TCComponent;
    procedure WriteComponent(Instance: TCComponent);
    procedure WriteComponentRes(const ResName: string; Instance: TCComponent);
    procedure WriteDescendent(Instance, Ancestor: TCComponent);
    procedure WriteDescendentRes(const ResName: string; Instance, Ancestor: TCComponent);
    procedure WriteResourceHeader(const ResName: string; {!!!:out} var FixupInfo: Integer);
    procedure FixupResourceHeader(FixupInfo: Integer);
    procedure ReadResHeader;
    function ReadByte : Byte;
    function ReadWord : Word;
    function ReadDWord : Cardinal;
    function ReadAnsiString : AnsiString;
    procedure WriteByte(b : Byte);
    procedure WriteWord(w : Word);
    procedure WriteDWord(d : Cardinal);
    Procedure WriteAnsiString (S : AnsiString);
    property Position: Longint read GetPosition write SetPosition;
    property Size: Longint read GetSize write SetSize;
  end;

{ TCCustomFileStream class }

  TCCustomFileStream = class(TCStream)
  protected
    FFileName : String;
  public
    constructor Create(const AFileName: string;{shortstring!} Mode: Word); virtual; abstract;
    function EOF: boolean; virtual; abstract;
    property FileName : String Read FFilename;
  end;

{ TFileStream class }

  TCFileStream = class(TCCustomFileStream)
  Private
    FHandle: File;
  protected
    procedure SetSize(NewSize: Longint); override;
  public
    constructor Create(const AFileName: string; Mode: Word); override;
    destructor Destroy; override;
    function Read(var Buffer; Count: Longint): Longint; override;
    function Write(const Buffer; Count: Longint): Longint; override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
    function EOF: boolean; override;
  end;

  TCFileStreamClass = class of TCCustomFileStream;
var
  CFileStreamClass: TCFileStreamClass = TCFileStream;

type
{ TCustomMemoryStream abstract class }

  TCCustomMemoryStream = class(TCStream)
  private
    FMemory: Pointer;
    FSize, FPosition: Longint;
  protected
    procedure SetPointer(Ptr: Pointer; ASize: Longint);
  public
    function Read(var Buffer; Count: Longint): Longint; override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
    procedure SaveToStream(Stream: TCStream);
    procedure SaveToFile(const FileName: string);
    property Memory: Pointer read FMemory;
  end;

{ TCMemoryStream }

  TCMemoryStream = class(TCCustomMemoryStream)
  private
    FCapacity: Longint;
    procedure SetCapacity(NewCapacity: Longint);
  protected
    function Realloc(var NewCapacity: Longint): Pointer; virtual;
    property Capacity: Longint read FCapacity write SetCapacity;
  public
    destructor Destroy; override;
    procedure Clear;
    procedure LoadFromStream(Stream: TCStream);
    procedure LoadFromFile(const FileName: string);
    procedure SetSize(NewSize: Longint); override;
    function Write(const Buffer; Count: Longint): Longint; override;
  end;


implementation

  Type
    PByte = ^Byte;

{*****************************************************************************
                                   TCStream
*****************************************************************************}

  function TCStream.GetPosition: Longint;

    begin
       Result:=Seek(0,soFromCurrent);
    end;

  procedure TCStream.SetPosition(Pos: Longint);

    begin
       Seek(pos,soFromBeginning);
    end;

  function TCStream.GetSize: Longint;

    var
       p : longint;

    begin
       p:=GetPosition;
       GetSize:=Seek(0,soFromEnd);
       Seek(p,soFromBeginning);
    end;

  procedure TCStream.SetSize(NewSize: Longint);

    begin
    // We do nothing. Pipe streams don't support this
    // As well as possible read-ony streams !!
    end;

  procedure TCStream.ReadBuffer(var Buffer; Count: Longint);

    begin
       CStreamError:=0;
       if Read(Buffer,Count)<Count then
         CStreamError:=102;
    end;

  procedure TCStream.WriteBuffer(const Buffer; Count: Longint);

    begin
       CStreamError:=0;
       if Write(Buffer,Count)<Count then
         CStreamError:=103;
    end;

  function TCStream.CopyFrom(Source: TCStream; Count: Longint): Longint;

    var
       i : longint;
       buffer : array[0..1023] of byte;

    begin
       CStreamError:=0;
       Result:=0;
       while Count>0 do
         begin
            if (Count>sizeof(buffer)) then
              i:=sizeof(Buffer)
            else
              i:=Count;
            i:=Source.Read(buffer,i);
            i:=Write(buffer,i);
            dec(count,i);
            inc(Result,i);
            if i=0 then
              exit;
         end;
    end;

  function TCStream.ReadComponent(Instance: TCComponent): TCComponent;
    begin
      Result:=nil;
    end;

  function TCStream.ReadComponentRes(Instance: TCComponent): TCComponent;
    begin
      Result:=nil;
    end;

  procedure TCStream.WriteComponent(Instance: TCComponent);
    begin
    end;

  procedure TCStream.WriteComponentRes(const ResName: string; Instance: TCComponent);
    begin
    end;

  procedure TCStream.WriteDescendent(Instance, Ancestor: TCComponent);
    begin
    end;

  procedure TCStream.WriteDescendentRes(const ResName: string; Instance, Ancestor: TCComponent);
    begin
    end;

  procedure TCStream.WriteResourceHeader(const ResName: string; {!!!: out} var FixupInfo: Integer);
    begin
    end;

  procedure TCStream.FixupResourceHeader(FixupInfo: Integer);
    begin
    end;

  procedure TCStream.ReadResHeader;
    begin
    end;

  function TCStream.ReadByte : Byte;

    var
       b : Byte;

    begin
       ReadBuffer(b,1);
       ReadByte:=b;
    end;

  function TCStream.ReadWord : Word;

    var
       w : Word;

    begin
       ReadBuffer(w,2);
       ReadWord:=w;
    end;

  function TCStream.ReadDWord : Cardinal;

    var
       d : Cardinal;

    begin
       ReadBuffer(d,4);
       ReadDWord:=d;
    end;

  Function TCStream.ReadAnsiString : AnsiString;
  Var
    TheSize : Longint;
    P : PByte ;
  begin
    ReadBuffer (TheSize,SizeOf(TheSize));
    SetLength(Result,TheSize);
    // Illegal typecast if no AnsiStrings defined.
    if TheSize>0 then
     begin
       ReadBuffer (Pointer(Result)^,TheSize);
       P:=PByte(PtrInt(Result)+TheSize);
       p^:=0;
     end;
   end;

  Procedure TCStream.WriteAnsiString (S : AnsiString);

  Var L : Longint;

  begin
    L:=Length(S);
    WriteBuffer (L,SizeOf(L));
    WriteBuffer (Pointer(S)^,L);
  end;

  procedure TCStream.WriteByte(b : Byte);

    begin
       WriteBuffer(b,1);
    end;

  procedure TCStream.WriteWord(w : Word);

    begin
       WriteBuffer(w,2);
    end;

  procedure TCStream.WriteDWord(d : Cardinal);

    begin
       WriteBuffer(d,4);
    end;


{****************************************************************************}
{*                             TCFileStream                                  *}
{****************************************************************************}

constructor TCFileStream.Create(const AFileName: string; Mode: Word);
var
  oldfilemode : byte;
begin
  FFileName:=AFileName;
  If Mode=fmcreate then
    begin
      system.assign(FHandle,AFileName);
      {$push} {$I-}
       system.rewrite(FHandle,1);
      {$pop}
      CStreamError:=IOResult;
    end
  else
    begin
      oldfilemode:=filemode;
      filemode:=$40 or Mode;
      system.assign(FHandle,AFileName);
      {$push} {$I-}
       system.reset(FHandle,1);
      {$pop}
      CStreamError:=IOResult;
      filemode:=oldfilemode;
    end;
end;


destructor TCFileStream.Destroy;
begin
  {$push} {$I-}
   System.Close(FHandle);
  {$pop}
  CStreamError:=IOResult;
end;


function TCFileStream.Read(var Buffer; Count: Longint): Longint;
begin
  CStreamError:=0;
  BlockRead(FHandle,Buffer,Count,Result);
  If Result=-1 then Result:=0;
end;


function TCFileStream.Write(const Buffer; Count: Longint): Longint;
begin
  CStreamError:=0;
  BlockWrite (FHandle,(@Buffer)^,Count,Result);
  If Result=-1 then Result:=0;
end;


Procedure TCFileStream.SetSize(NewSize: Longint);
begin
  {$push} {$I-}
   System.Seek(FHandle,NewSize);
   System.Truncate(FHandle);
  {$pop}
  CStreamError:=IOResult;
end;


function TCFileStream.Seek(Offset: Longint; Origin: Word): Longint;
var
  l : longint;
begin
  {$push} {$I-}
   case Origin of
     soFromBeginning :
       begin
         System.Seek(FHandle,Offset);
         l:=Offset;
       end;
     soFromCurrent :
       begin
         l:=System.FilePos(FHandle);
         inc(l,Offset);
         System.Seek(FHandle,l);
       end;
     soFromEnd :
       begin
         l:=System.FileSize(FHandle);
         dec(l,Offset);
         if l<0 then
          l:=0;
         System.Seek(FHandle,l);
       end;
     else
       begin
         CStreamError:=103;
         l:=Offset;
       end;
   end;
  {$pop}
  CStreamError:=IOResult;
  Result:=l;
<<<<<<< HEAD
<<<<<<< HEAD
end;

function TCFileStream.EOF: boolean;
begin
  EOF:=system.eof(FHandle);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


{****************************************************************************}
{*                             TCustomMemoryStream                          *}
{****************************************************************************}

procedure TCCustomMemoryStream.SetPointer(Ptr: Pointer; ASize: Longint);

begin
  FMemory:=Ptr;
  FSize:=ASize;
end;


function TCCustomMemoryStream.Read(var Buffer; Count: Longint): Longint;

begin
  Result:=0;
  If (FSize>0) and (FPosition<Fsize) then
    begin
    Result:=FSize-FPosition;
    If Result>Count then Result:=Count;
    Move (Pointer(PtrUInt(FMemory)+PtrUInt(FPosition))^,Buffer,Result);
    FPosition:=Fposition+Result;
    end;
end;


function TCCustomMemoryStream.Seek(Offset: Longint; Origin: Word): Longint;

begin
  Case Origin of
    soFromBeginning : FPosition:=Offset;
    soFromEnd       : FPosition:=FSize+Offset;
    soFromCurrent   : FpoSition:=FPosition+Offset;
  end;
  Result:=FPosition;
end;


procedure TCCustomMemoryStream.SaveToStream(Stream: TCStream);

begin
  if FSize>0 then Stream.WriteBuffer (FMemory^,FSize);
end;


procedure TCCustomMemoryStream.SaveToFile(const FileName: string);

Var S : TCCustomFileStream;

begin
  Try
    S:=CFileStreamClass.Create (FileName,fmCreate);
    SaveToStream(S);
  finally
    S.free;
  end;
end;


{****************************************************************************}
{*                             TCMemoryStream                                *}
{****************************************************************************}


Const TMSGrow = 4096; { Use 4k blocks. }

procedure TCMemoryStream.SetCapacity(NewCapacity: Longint);

begin
  SetPointer (Realloc(NewCapacity),Fsize);
  FCapacity:=NewCapacity;
end;


function TCMemoryStream.Realloc(var NewCapacity: Longint): Pointer;

Var MoveSize : Longint;

begin
  CStreamError:=0;
  If NewCapacity>0 Then // round off to block size.
    NewCapacity := (NewCapacity + (TMSGrow-1)) and not (TMSGROW-1);
  // Only now check !
  If NewCapacity=FCapacity then
    Result:=FMemory
  else
    If NewCapacity=0 then
      begin
        FreeMem (FMemory,Fcapacity);
        Result:=nil;
      end
    else
      begin
      GetMem (Result,NewCapacity);
      If Result=Nil then
        CStreamError:=204;
      If FCapacity>0 then
        begin
        MoveSize:=FSize;
        If MoveSize>NewCapacity then MoveSize:=NewCapacity;
        Move (Fmemory^,Result^,MoveSize);
        FreeMem (FMemory,FCapacity);
        end;
      end;
end;


destructor TCMemoryStream.Destroy;

begin
  Clear;
  Inherited Destroy;
end;


procedure TCMemoryStream.Clear;

begin
  FSize:=0;
  FPosition:=0;
  SetCapacity (0);
end;


procedure TCMemoryStream.LoadFromStream(Stream: TCStream);

begin
  Stream.Position:=0;
  SetSize(Stream.Size);
  If FSize>0 then Stream.ReadBuffer(FMemory^,FSize);
end;


procedure TCMemoryStream.LoadFromFile(const FileName: string);

Var S : TCCustomFileStream;

begin
  Try
    S:=CFileStreamClass.Create (FileName,fmOpenRead);
    LoadFromStream(S);
  finally
    S.free;
  end;
end;


procedure TCMemoryStream.SetSize(NewSize: Longint);

begin
  SetCapacity (NewSize);
  FSize:=NewSize;
  IF FPosition>FSize then
    FPosition:=FSize;
end;


function TCMemoryStream.Write(const Buffer; Count: Longint): Longint;

Var NewPos : Longint;

begin
  If Count=0 then
   begin
     Result:=0;
     exit;
   end;
  NewPos:=FPosition+Count;
  If NewPos>Fsize then
    begin
    IF NewPos>FCapacity then
      SetCapacity (NewPos);
    FSize:=Newpos;
    end;
  System.Move (Buffer,Pointer(Ptruint(FMemory)+PtrUInt(FPosition))^,Count);
  FPosition:=NewPos;
  Result:=Count;
end;

end.
