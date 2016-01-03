{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{****************************************************************************}
{*                             TStream                                      *}
{****************************************************************************}

procedure TStream.ReadNotImplemented;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  raise EStreamError.CreateFmt(SStreamNoReading, [ClassName]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
=======
  raise EStreamError.CreateFmt(SStreamNoReading, [ClassName]) at get_caller_addr(get_frame);
>>>>>>> graemeg/fixes_2_2
=======
  raise EStreamError.CreateFmt(SStreamNoReading, [ClassName]) at get_caller_addr(get_frame);
>>>>>>> origin/fixes_2_2
end;

procedure TStream.WriteNotImplemented;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  raise EStreamError.CreateFmt(SStreamNoWriting, [ClassName]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
=======
  raise EStreamError.CreateFmt(SStreamNoWriting, [ClassName]) at get_caller_addr(get_frame);
>>>>>>> graemeg/fixes_2_2
=======
  raise EStreamError.CreateFmt(SStreamNoWriting, [ClassName]) at get_caller_addr(get_frame);
>>>>>>> origin/fixes_2_2
end;

function TStream.Read(var Buffer; Count: Longint): Longint;
begin
  ReadNotImplemented;
  Result := 0;
end;

function TStream.Write(const Buffer; Count: Longint): Longint;
begin
  WriteNotImplemented;
  Result := 0;
end;


  function TStream.GetPosition: Int64;

    begin
       Result:=Seek(0,soCurrent);
    end;

  procedure TStream.SetPosition(const Pos: Int64);

    begin
       Seek(pos,soBeginning);
    end;

  procedure TStream.SetSize64(const NewSize: Int64);

    begin
      // Required because can't use overloaded functions in properties
      SetSize(NewSize);
    end;

  function TStream.GetSize: Int64;

    var
       p : int64;

    begin
       p:=Seek(0,soCurrent);
       GetSize:=Seek(0,soEnd);
       Seek(p,soBeginning);
    end;

  procedure TStream.SetSize(NewSize: Longint);

    begin
    // We do nothing. Pipe streams don't support this
    // As wel as possible read-ony streams !!
    end;

  procedure TStream.SetSize(const NewSize: Int64);

    begin
      // Backwards compatibility that calls the longint SetSize
      if (NewSize<Low(longint)) or
         (NewSize>High(longint)) then
        raise ERangeError.Create(SRangeError);
      SetSize(longint(NewSize));
    end;

  function TStream.Seek(Offset: Longint; Origin: Word): Longint;

    type
      TSeek64 = function(const offset:Int64;Origin:TSeekorigin):Int64 of object;
    var
      CurrSeek,
      TStreamSeek : TSeek64;
      CurrClass   : TClass;
    begin
      // Redirect calls to 64bit Seek, but we can't call the 64bit Seek
      // from TStream, because then we end up in an infinite loop
      CurrSeek:=nil;
      CurrClass:=Classtype;
      while (CurrClass<>nil) and
            (CurrClass<>TStream) do
       CurrClass:=CurrClass.Classparent;
      if CurrClass<>nil then
       begin
         CurrSeek:=@Self.Seek;
         TStreamSeek:=@TStream(@CurrClass).Seek;
         if TMethod(TStreamSeek).Code=TMethod(CurrSeek).Code then
          CurrSeek:=nil;
       end;
      if CurrSeek<>nil then
       Result:=Seek(Int64(offset),TSeekOrigin(origin))
      else
       raise EStreamError.CreateFmt(SSeekNotImplemented,[ClassName]);
    end;

  procedure TStream.Discard(const Count: Int64);

  const
    CSmallSize      =255;
    CLargeMaxBuffer =32*1024; // 32 KiB
  var
    Buffer: array[1..CSmallSize] of Byte;

  begin
    if Count=0 then
      Exit;
    if Count<=SizeOf(Buffer) then
      ReadBuffer(Buffer,Count)
    else
      DiscardLarge(Count,CLargeMaxBuffer);
  end;

  procedure TStream.DiscardLarge(Count: int64; const MaxBufferSize: Longint);

  var
    Buffer: array of Byte;

  begin
    if Count=0 then
       Exit;
    if Count>MaxBufferSize then
      SetLength(Buffer,MaxBufferSize)
    else
      SetLength(Buffer,Count);
    while (Count>=Length(Buffer)) do
      begin
      ReadBuffer(Buffer[0],Length(Buffer));
      Dec(Count,Length(Buffer));
      end;
    if Count>0 then
      ReadBuffer(Buffer[0],Count);
  end;

  procedure TStream.InvalidSeek;

  begin
    raise EStreamError.CreateFmt(SStreamInvalidSeek, [ClassName]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
  end;

  procedure TStream.FakeSeekForward(Offset: Int64;  const Origin: TSeekOrigin; const Pos: Int64);

  begin
    if Origin=soBeginning then
       Dec(Offset,Pos);
    if (Offset<0) or (Origin=soEnd) then
      InvalidSeek;
    if Offset>0 then
      Discard(Offset);
   end;

 function TStream.Seek(const Offset: Int64; Origin: TSeekorigin): Int64;

    begin
      // Backwards compatibility that calls the longint Seek
      if (Offset<Low(longint)) or
         (Offset>High(longint)) then
        raise ERangeError.Create(SRangeError);
      Result:=Seek(longint(Offset),ord(Origin));
    end;

  procedure TStream.ReadBuffer(var Buffer; Count: Longint);

  Var
    r,t : longint;

  begin
    t:=0;
    repeat
      r:=Read(PByte(@Buffer)[t],Count-t);
      inc(t,r);
    until (t=Count) or (r<=0);
    if (t<Count) then
      Raise EReadError.Create(SReadError);
  end;

  procedure TStream.WriteBuffer(const Buffer; Count: Longint);

  var
    r,t : Longint;

    begin
      T:=0;
      Repeat
         r:=Write(PByte(@Buffer)[t],Count-t);
         inc(t,r);
      Until (t=count) or (r<=0);
      if (t<Count) then
         Raise EWriteError.Create(SWriteError);
    end;

  function TStream.CopyFrom(Source: TStream; Count: Int64): Int64;

    var
       Buffer: Pointer;
       BufferSize, i: LongInt;

    const
       MaxSize = $20000;
    begin

       Result:=0;
       if Count=0 then
         Source.Position:=0;   // This WILL fail for non-seekable streams...
       BufferSize:=MaxSize;
       if (Count>0) and (Count<BufferSize) then
         BufferSize:=Count;    // do not allocate more than needed

       GetMem(Buffer,BufferSize);
       try
         if Count=0 then
         repeat
           i:=Source.Read(buffer^,BufferSize);
           if i>0 then
             WriteBuffer(buffer^,i);
           Inc(Result,i);
         until i<BufferSize
         else
         while Count>0 do
         begin
           if Count>BufferSize then
             i:=BufferSize
           else
             i:=Count;
           Source.ReadBuffer(buffer^,i);
           WriteBuffer(buffer^,i);
           Dec(count,i);
           Inc(Result,i);
         end;
       finally
         FreeMem(Buffer);
       end;

    end;

  function TStream.ReadComponent(Instance: TComponent): TComponent;

    var
      Reader: TReader;

    begin

      Reader := TReader.Create(Self, 4096);
      try
        Result := Reader.ReadRootComponent(Instance);
      finally
        Reader.Free;
      end;

    end;

  function TStream.ReadComponentRes(Instance: TComponent): TComponent;

    begin

      ReadResHeader;
      Result := ReadComponent(Instance);

    end;

  procedure TStream.WriteComponent(Instance: TComponent);

    begin

      WriteDescendent(Instance, nil);

    end;

  procedure TStream.WriteComponentRes(const ResName: string; Instance: TComponent);

    begin

      WriteDescendentRes(ResName, Instance, nil);

    end;

  procedure TStream.WriteDescendent(Instance, Ancestor: TComponent);

    var
       Driver : TAbstractObjectWriter;
       Writer : TWriter;

    begin

       Driver := TBinaryObjectWriter.Create(Self, 4096);
       Try
         Writer := TWriter.Create(Driver);
         Try
           Writer.WriteDescendent(Instance, Ancestor);
         Finally
           Writer.Destroy;
         end;
       Finally
         Driver.Free;
       end;

    end;

  procedure TStream.WriteDescendentRes(const ResName: string; Instance, Ancestor: TComponent);

    var
      FixupInfo: Longint;

    begin

      { Write a resource header }
      WriteResourceHeader(ResName, FixupInfo);
      { Write the instance itself }
      WriteDescendent(Instance, Ancestor);
      { Insert the correct resource size into the resource header }
      FixupResourceHeader(FixupInfo);

    end;

<<<<<<< HEAD
  procedure TStream.WriteResourceHeader(const ResName: string; {!!!: out} var FixupInfo: Longint);
=======
  procedure TStream.WriteResourceHeader(const ResName: string; {!!!: out} var FixupInfo: Integer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    var
      ResType, Flags : word;
    begin
       ResType:=NtoLE(word($000A));
       Flags:=NtoLE(word($1030));
       { Note: This is a Windows 16 bit resource }
       { Numeric resource type }
       WriteByte($ff);
       { Application defined data }
       WriteWord(ResType);
       { write the name as asciiz }
       WriteBuffer(ResName[1],length(ResName));
       WriteByte(0);
       { Movable, Pure and Discardable }
       WriteWord(Flags);
       { Placeholder for the resource size }
       WriteDWord(0);
       { Return current stream position so that the resource size can be
         inserted later }
       FixupInfo := Position;
    end;

  procedure TStream.FixupResourceHeader(FixupInfo: Longint);

    var
<<<<<<< HEAD
<<<<<<< HEAD
       ResSize,TmpResSize : Longint;
=======
       ResSize,TmpResSize : Integer;
>>>>>>> graemeg/fixes_2_2
=======
       ResSize,TmpResSize : Integer;
>>>>>>> origin/fixes_2_2

    begin

      ResSize := Position - FixupInfo;
      TmpResSize := NtoLE(longword(ResSize));

      { Insert the correct resource size into the placeholder written by
        WriteResourceHeader }
      Position := FixupInfo - 4;
      WriteDWord(TmpResSize);
      { Seek back to the end of the resource }
      Position := FixupInfo + ResSize;

    end;

  procedure TStream.ReadResHeader;
    var
      ResType, Flags : word;
    begin
       try
         { Note: This is a Windows 16 bit resource }
         { application specific resource ? }
         if ReadByte<>$ff then
           raise EInvalidImage.Create(SInvalidImage);
         ResType:=LEtoN(ReadWord);
         if ResType<>$000a then
           raise EInvalidImage.Create(SInvalidImage);
         { read name }
         while ReadByte<>0 do
           ;
         { check the access specifier }
         Flags:=LEtoN(ReadWord);
         if Flags<>$1030 then
           raise EInvalidImage.Create(SInvalidImage);
         { ignore the size }
         ReadDWord;
       except
         on EInvalidImage do
           raise;
         else
           raise EInvalidImage.create(SInvalidImage);
       end;
    end;

  function TStream.ReadByte : Byte;

    var
       b : Byte;

    begin
       ReadBuffer(b,1);
       ReadByte:=b;
    end;

  function TStream.ReadWord : Word;

    var
       w : Word;

    begin
       ReadBuffer(w,2);
       ReadWord:=w;
    end;

  function TStream.ReadDWord : Cardinal;

    var
       d : Cardinal;

    begin
       ReadBuffer(d,4);
       ReadDWord:=d;
    end;

  function TStream.ReadQWord: QWord;
    var
       q: QWord;
    begin
      ReadBuffer(q,8);
      ReadQWord:=q;

    end;

  Function TStream.ReadAnsiString : String;

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
       P:=Pointer(Result)+TheSize;
       p^:=0;
     end;
   end;

  Procedure TStream.WriteAnsiString (const S : String);

  Var L : Longint;

  begin
    L:=Length(S);
    WriteBuffer (L,SizeOf(L));
    WriteBuffer (Pointer(S)^,L);
  end;

  procedure TStream.WriteByte(b : Byte);

    begin
       WriteBuffer(b,1);
    end;

  procedure TStream.WriteWord(w : Word);

    begin
       WriteBuffer(w,2);
    end;

  procedure TStream.WriteDWord(d : Cardinal);

    begin
       WriteBuffer(d,4);
    end;

  procedure TStream.WriteQWord(q: QWord);
    begin
      WriteBuffer(q,8);
    end;


{****************************************************************************}
{*                             THandleStream                                *}
{****************************************************************************}

Constructor THandleStream.Create(AHandle: THandle);

begin
  Inherited Create;
  FHandle:=AHandle;
end;


function THandleStream.Read(var Buffer; Count: Longint): Longint;

begin
  Result:=FileRead(FHandle,Buffer,Count);
  If Result=-1 then Result:=0;
end;


function THandleStream.Write(const Buffer; Count: Longint): Longint;

begin
  Result:=FileWrite (FHandle,Buffer,Count);
  If Result=-1 then Result:=0;
end;

Procedure THandleStream.SetSize(NewSize: Longint);

begin
  SetSize(Int64(NewSize));
end;


Procedure THandleStream.SetSize(const NewSize: Int64);

begin
  FileTruncate(FHandle,NewSize);
end;


function THandleStream.Seek(const Offset: Int64; Origin: TSeekOrigin): Int64;

begin
  Result:=FileSeek(FHandle,Offset,ord(Origin));
end;


{****************************************************************************}
{*                             TFileStream                                  *}
{****************************************************************************}

constructor TFileStream.Create(const AFileName: string; Mode: Word);

begin
  Create(AFileName,Mode,438);
end;


constructor TFileStream.Create(const AFileName: string; Mode: Word; Rights: Cardinal);

begin
  FFileName:=AFileName;
  If (Mode and fmCreate) > 0 then
    FHandle:=FileCreate(AFileName,Mode,Rights)
  else
    FHAndle:=FileOpen(AFileName,Mode);

  If (THandle(FHandle)=feInvalidHandle) then
    If Mode=fmcreate then
      raise EFCreateError.createfmt(SFCreateError,[AFileName])
    else
      raise EFOpenError.Createfmt(SFOpenError,[AFilename]);
end;


destructor TFileStream.Destroy;

begin
  FileClose(FHandle);
end;

{****************************************************************************}
{*                             TCustomMemoryStream                          *}
{****************************************************************************}

procedure TCustomMemoryStream.SetPointer(Ptr: Pointer; ASize: PtrInt);

begin
  FMemory:=Ptr;
  FSize:=ASize;
end;


function TCustomMemoryStream.GetSize: Int64;

begin
  Result:=FSize;
end;

function TCustomMemoryStream.GetPosition: Int64;
begin
  Result:=FPosition;
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
function TCustomMemoryStream.Read(var Buffer; Count: LongInt): LongInt;

begin
  Result:=0;
  If (FSize>0) and (FPosition<Fsize) and (FPosition>=0) then
    begin
    Result:=Count;
    If (Result>(FSize-FPosition)) then
      Result:=(FSize-FPosition);
    Move ((FMemory+FPosition)^,Buffer,Result);
    FPosition:=Fposition+Result;
    end;
end;


function TCustomMemoryStream.Seek(const Offset: Int64; Origin: TSeekOrigin): Int64;

begin
  Case Word(Origin) of
    soFromBeginning : FPosition:=Offset;
    soFromEnd       : FPosition:=FSize+Offset;
    soFromCurrent   : FPosition:=FPosition+Offset;
  end;
  Result:=FPosition;
  {$IFDEF DEBUG}
  if Result < 0 then
    raise Exception.Create('TCustomMemoryStream');
  {$ENDIF}
end;


procedure TCustomMemoryStream.SaveToStream(Stream: TStream);

begin
  if FSize>0 then Stream.WriteBuffer (FMemory^,FSize);
end;


procedure TCustomMemoryStream.SaveToFile(const FileName: string);

Var S : TFileStream;

begin
  S:=TFileStream.Create (FileName,fmCreate);
  Try
    SaveToStream(S);
  finally
    S.free;
  end;
end;


{****************************************************************************}
{*                             TMemoryStream                                *}
{****************************************************************************}


Const TMSGrow = 4096; { Use 4k blocks. }

procedure TMemoryStream.SetCapacity(NewCapacity: PtrInt);

begin
  SetPointer (Realloc(NewCapacity),Fsize);
  FCapacity:=NewCapacity;
end;


function TMemoryStream.Realloc(var NewCapacity: PtrInt): Pointer;

begin
  If NewCapacity<0 Then
    NewCapacity:=0
  else
    begin
      // if growing, grow at least a quarter
      if (NewCapacity>FCapacity) and (NewCapacity < (5*FCapacity) div 4) then
        NewCapacity := (5*FCapacity) div 4;
      // round off to block size.
      NewCapacity := (NewCapacity + (TMSGrow-1)) and not (TMSGROW-1);
    end;
  // Only now check !
  If NewCapacity=FCapacity then
    Result:=FMemory
  else
    begin
      Result:=Reallocmem(FMemory,Newcapacity);
      If (Result=Nil) and (Newcapacity>0) then
        Raise EStreamError.Create(SMemoryStreamError);
    end;
end;


destructor TMemoryStream.Destroy;

begin
  Clear;
  Inherited Destroy;
end;


procedure TMemoryStream.Clear;

begin
  FSize:=0;
  FPosition:=0;
  SetCapacity (0);
end;


procedure TMemoryStream.LoadFromStream(Stream: TStream);

begin
  Stream.Position:=0;
  SetSize(Stream.Size);
  If FSize>0 then Stream.ReadBuffer(FMemory^,FSize);
end;


procedure TMemoryStream.LoadFromFile(const FileName: string);

Var S : TFileStream;

begin
  S:=TFileStream.Create (FileName,fmOpenRead or fmShareDenyWrite);
  Try
    LoadFromStream(S);
  finally
    S.free;
  end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TMemoryStream.SetSize({$ifdef CPU64}const NewSize: Int64{$else}NewSize: LongInt{$endif});
=======
procedure TMemoryStream.SetSize({$ifdef CPU64}const{$endif CPU64} NewSize: PtrInt);
>>>>>>> graemeg/cpstrnew
=======
procedure TMemoryStream.SetSize({$ifdef CPU64}const{$endif CPU64} NewSize: PtrInt);
>>>>>>> graemeg/cpstrnew
=======
procedure TMemoryStream.SetSize({$ifdef CPU64}const{$endif CPU64} NewSize: PtrInt);
>>>>>>> graemeg/cpstrnew
=======
procedure TMemoryStream.SetSize({$ifdef CPU64}const{$endif CPU64} NewSize: PtrInt);
>>>>>>> origin/cpstrnew

begin
  SetCapacity (NewSize);
  FSize:=NewSize;
  IF FPosition>FSize then
    FPosition:=FSize;
end;

function TMemoryStream.Write(const Buffer; Count: LongInt): LongInt;

Var NewPos : PtrInt;

begin
  If (Count=0) or (FPosition<0) then
    exit(0);
  NewPos:=FPosition+Count;
  If NewPos>Fsize then
    begin
    IF NewPos>FCapacity then
      SetCapacity (NewPos);
    FSize:=Newpos;
    end;
  System.Move (Buffer,(FMemory+FPosition)^,Count);
  FPosition:=NewPos;
  Result:=Count;
end;

{****************************************************************************}
{*                              TBytesStream                                *}
{****************************************************************************}

constructor TBytesStream.Create(const ABytes: TBytes);
begin
  inherited Create;
  FBytes:=ABytes;
  SetPointer(Pointer(FBytes),Length(FBytes));
  FCapacity:=Length(FBytes);
end;

function TBytesStream.Realloc(var NewCapacity: PtrInt): Pointer;
begin
  // adapt TMemoryStream code to use with dynamic array
  if NewCapacity<0 Then
    NewCapacity:=0
  else
    begin
      if (NewCapacity>Capacity) and (NewCapacity < (5*Capacity) div 4) then
        NewCapacity := (5*Capacity) div 4;
      NewCapacity := (NewCapacity + (TMSGrow-1)) and not (TMSGROW-1);
    end;
  if NewCapacity=Capacity then
    Result:=Pointer(FBytes)
  else
    begin
      SetLength(FBytes,Newcapacity);
      Result:=Pointer(FBytes);
      if (Result=nil) and (Newcapacity>0) then
        raise EStreamError.Create(SMemoryStreamError);
    end;
end;

{****************************************************************************}
{*                             TStringStream                                *}
{****************************************************************************}

function TStringStream.GetSize: Int64;
begin
  Result:=Length(FDataString);
end;

function TStringStream.GetPosition: Int64;
begin
  Result:=FPosition;
end;

procedure TStringStream.SetSize(NewSize: Longint);

begin
 Setlength(FDataString,NewSize);
 If FPosition>NewSize then FPosition:=NewSize;
end;


constructor TStringStream.Create(const AString: string = '');

begin
  Inherited create;
  FDataString:=AString;
  UniqueString(FDataString);
end;


function TStringStream.Read(var Buffer; Count: Longint): Longint;

begin
  Result:=Length(FDataString)-FPosition;
  If Result>Count then Result:=Count;
  // This supposes FDataString to be of type AnsiString !
  if Result>0 then
   begin
   Move (Pchar(FDataString)[FPosition],Buffer,Result);
   FPosition:=FPosition+Result;
   end;
end;


function TStringStream.ReadString(Count: Longint): string;

Var NewLen : Longint;

begin
  NewLen:=Length(FDataString)-FPosition;
  If NewLen>Count then NewLen:=Count;
  SetLength(Result,NewLen);
  Read (Pointer(Result)^,NewLen);
end;


function TStringStream.Seek(Offset: Longint; Origin: Word): Longint;

begin
  Case Origin of
    soFromBeginning : FPosition:=Offset;
    soFromEnd       : FPosition:=Length(FDataString)+Offset;
    soFromCurrent   : FPosition:=FPosition+Offset;
  end;
  If FPosition>Length(FDataString) then
    FPosition:=Length(FDataString)
  else If FPosition<0 then
    FPosition:=0;
  Result:=FPosition;
end;


function TStringStream.Write(const Buffer; Count: Longint): Longint;

begin
  Result:=Count;
  if FPosition+Count>Size then
    SetSize(FPosition+Count);
  // This supposes that FDataString is of type AnsiString)
  Move (Buffer,PChar(FDataString)[Fposition],Count);
  FPosition:=FPosition+Count;
end;


procedure TStringStream.WriteString(const AString: string);

begin
  Write (PChar(Astring)[0],Length(AString));
end;



{****************************************************************************}
{*                             TResourceStream                              *}
{****************************************************************************}

<<<<<<< HEAD
{$ifdef FPC_OS_UNICODE}
procedure TResourceStream.Initialize(Instance: TFPResourceHMODULE; Name, ResType: PWideChar; NameIsID: Boolean);
=======
{$ifdef UNICODE}
procedure TResourceStream.Initialize(Instance: THandle; Name, ResType: PWideChar; NameIsID: Boolean);
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
    Res:=FindResource(Instance, Name, ResType);
    if Res=0 then
      if NameIsID then
        raise EResNotFound.CreateFmt(SResNotFound,[IntToStr(PtrInt(Name))])
      else
        raise EResNotFound.CreateFmt(SResNotFound,[Name]);
    Handle:=LoadResource(Instance,Res);
    if Handle=0 then
      if NameIsID then
        raise EResNotFound.CreateFmt(SResNotFound,[IntToStr(PtrInt(Name))])
      else
        raise EResNotFound.CreateFmt(SResNotFound,[Name]);
    SetPointer(LockResource(Handle),SizeOfResource(Instance,Res));
  end;

constructor TResourceStream.Create(Instance: TFPResourceHMODULE; const ResName: WideString; ResType: PWideChar);
  begin
    inherited create;
    Initialize(Instance,PWideChar(ResName),ResType,False);
  end;
constructor TResourceStream.CreateFromID(Instance: TFPResourceHMODULE; ResID: Integer; ResType: PWideChar);
  begin
    inherited create;
    Initialize(Instance,PWideChar(ResID),ResType,True);
  end;
<<<<<<< HEAD
{$else FPC_OS_UNICODE}

<<<<<<< HEAD
<<<<<<< HEAD
procedure TResourceStream.Initialize(Instance: TFPResourceHMODULE; Name, ResType: PChar; NameIsID: Boolean);
=======
procedure TResourceStream.Initialize(Instance: THandle; Name, ResType: PChar; NameIsID: Boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure TResourceStream.Initialize(Instance: THandle; Name, ResType: PChar; NameIsID: Boolean);
>>>>>>> graemeg/cpstrnew
=======
{$else UNICODE}

procedure TResourceStream.Initialize(Instance: THandle; Name, ResType: PChar; NameIsID: Boolean);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  begin
    Res:=FindResource(Instance, Name, ResType);
    if Res=0 then
      if NameIsID then
        raise EResNotFound.CreateFmt(SResNotFound,[IntToStr(PtrInt(Name))])
      else
        raise EResNotFound.CreateFmt(SResNotFound,[Name]);
    Handle:=LoadResource(Instance,Res);
    if Handle=0 then
      if NameIsID then
        raise EResNotFound.CreateFmt(SResNotFound,[IntToStr(PtrInt(Name))])
      else
        raise EResNotFound.CreateFmt(SResNotFound,[Name]);
    SetPointer(LockResource(Handle),SizeOfResource(Instance,Res));
  end;

constructor TResourceStream.Create(Instance: TFPResourceHMODULE; const ResName: string; ResType: PChar);
  begin
    inherited create;
    Initialize(Instance,pchar(ResName),ResType,False);
  end;
constructor TResourceStream.CreateFromID(Instance: TFPResourceHMODULE; ResID: Integer; ResType: PChar);
  begin
    inherited create;
    Initialize(Instance,pchar(PtrInt(ResID)),ResType,True);
  end;
{$endif FPC_OS_UNICODE}


destructor TResourceStream.Destroy;
  begin
    UnlockResource(Handle);
    FreeResource(Handle);
    inherited destroy;
  end;

{****************************************************************************}
{*                             TOwnerStream                                 *}
{****************************************************************************}

constructor TOwnerStream.Create(ASource: TStream);
begin
  FSource:=ASource;
end;

destructor TOwnerStream.Destroy;
begin
  If FOwner then
    FreeAndNil(FSource);
  inherited Destroy;
end;

{****************************************************************************}
{*                             TStreamAdapter                               *}
{****************************************************************************}
constructor TStreamAdapter.Create(Stream: TStream; Ownership: TStreamOwnership = soReference);
begin
  inherited Create;
  FStream:=Stream;
  FOwnership:=Ownership;
<<<<<<< HEAD
<<<<<<< HEAD
  m_bReverted:=false;   // mantis 15003
			// http://www.tech-archive.net/Archive/German/microsoft.public.de.vc/2005-08/msg00791.html
			// http://code.google.com/p/ddab-lib/wiki/TPJIStreamWrapper
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


destructor TStreamAdapter.Destroy;
begin
  if StreamOwnership=soOwned then
    FreeAndNil(FStream);
  inherited Destroy;
end;
<<<<<<< HEAD
<<<<<<< HEAD

{$warnings off}
function TStreamAdapter.Read(pv: Pointer; cb: DWORD; pcbRead: PDWORD): HResult; stdcall;
var
  readcount: Longint;
begin
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  if pv = nil then
    begin
      Result := STG_E_INVALIDPOINTER;
      Exit;
    end;

  readcount := FStream.Read(pv^, cb);
  if pcbRead <> nil then pcbRead^ := readcount;
  Result := S_OK;
end;

function TStreamAdapter.Write(pv: Pointer; cb: DWORD; pcbWritten: PDWORD): HResult; stdcall;
var
  writecount: Longint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
begin
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  if pv = nil then
    begin
      Result := STG_E_INVALIDPOINTER;
      Exit;
    end;
<<<<<<< HEAD
<<<<<<< HEAD

  writecount := FStream.Write(pv^, cb);
  if pcbWritten <> nil then pcbWritten^ := writecount;
  Result := S_OK;
end;

function TStreamAdapter.Seek(dlibMove: LargeUint; dwOrigin: Longint; out libNewPosition: LargeUint): HResult; stdcall;
var
  newpos: QWord;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  case dwOrigin of
    STREAM_SEEK_SET: newpos := FStream.Seek(dlibMove, soBeginning);
    STREAM_SEEK_CUR: newpos := FStream.Seek(dlibMove, soCurrent);
    STREAM_SEEK_END: newpos := FStream.Seek(dlibMove, soEnd);
    else
      begin
        Result := STG_E_INVALIDFUNCTION;
        Exit;
      end;
  end;
  if @libNewPosition <> nil then
    libNewPosition := newpos;
  Result := S_OK;
end;

function TStreamAdapter.SetSize(libNewSize: LargeUint): HResult; stdcall;
=======

=======
  if pv = nil then
    begin
      Result := STG_E_INVALIDPOINTER;
      Exit;
    end;

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======
  if pv = nil then
    begin
      Result := STG_E_INVALIDPOINTER;
      Exit;
    end;

>>>>>>> origin/cpstrnew
  writecount := FStream.Write(pv^, cb);
  if pcbWritten <> nil then pcbWritten^ := writecount;
  Result := S_OK;
end;

function TStreamAdapter.Seek(dlibMove: Largeint; dwOrigin: Longint; out libNewPosition: Largeint): HResult; stdcall;
var
  newpos: Int64;
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
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if libNewSize<0 then
    begin
      Result := STG_E_INVALIDFUNCTION;
      Exit;
    end;
  try
    FStream.Size := libNewSize;
    Result := S_OK;
  except
    // TODO: return different error value according to exception like STG_E_MEDIUMFULL
    Result := E_FAIL;
  end;
end;


function TStreamAdapter.CopyTo(stm: IStream; cb: LargeUint; out cbRead: LargeUint; out cbWritten: Largeuint): HResult; stdcall;
var
  sz: dword;
  buffer : array[0..1023] of byte;
=======
  case dwOrigin of
    STREAM_SEEK_SET: newpos := FStream.Seek(dlibMove, soBeginning);
    STREAM_SEEK_CUR: newpos := FStream.Seek(dlibMove, soCurrent);
    STREAM_SEEK_END: newpos := FStream.Seek(dlibMove, soEnd);
    else 
      begin 
        Result := STG_E_INVALIDFUNCTION; 
        Exit; 
      end;
  end;
  if @libNewPosition <> nil then
    libNewPosition := newpos;
  Result := S_OK;
end;

=======
  case dwOrigin of
    STREAM_SEEK_SET: newpos := FStream.Seek(dlibMove, soBeginning);
    STREAM_SEEK_CUR: newpos := FStream.Seek(dlibMove, soCurrent);
    STREAM_SEEK_END: newpos := FStream.Seek(dlibMove, soEnd);
    else 
      begin 
        Result := STG_E_INVALIDFUNCTION; 
        Exit; 
      end;
  end;
  if @libNewPosition <> nil then
    libNewPosition := newpos;
  Result := S_OK;
end;

>>>>>>> graemeg/cpstrnew
=======
  case dwOrigin of
    STREAM_SEEK_SET: newpos := FStream.Seek(dlibMove, soBeginning);
    STREAM_SEEK_CUR: newpos := FStream.Seek(dlibMove, soCurrent);
    STREAM_SEEK_END: newpos := FStream.Seek(dlibMove, soEnd);
    else 
      begin 
        Result := STG_E_INVALIDFUNCTION; 
        Exit; 
      end;
  end;
  if @libNewPosition <> nil then
    libNewPosition := newpos;
  Result := S_OK;
end;

>>>>>>> graemeg/cpstrnew
=======
  case dwOrigin of
    STREAM_SEEK_SET: newpos := FStream.Seek(dlibMove, soBeginning);
    STREAM_SEEK_CUR: newpos := FStream.Seek(dlibMove, soCurrent);
    STREAM_SEEK_END: newpos := FStream.Seek(dlibMove, soEnd);
    else 
      begin 
        Result := STG_E_INVALIDFUNCTION; 
        Exit; 
      end;
  end;
  if @libNewPosition <> nil then
    libNewPosition := newpos;
  Result := S_OK;
end;

>>>>>>> origin/cpstrnew
function TStreamAdapter.SetSize(libNewSize: Largeint): HResult; stdcall;
>>>>>>> graemeg/cpstrnew
begin
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  // the method is similar to TStream.CopyFrom => use CopyFrom implementation
  cbWritten := 0;
  cbRead := 0;
  while cb > 0 do
    begin
      if (cb > sizeof(buffer)) then
        sz := sizeof(Buffer)
      else
        sz := cb;
      sz := FStream.Read(buffer, sz);
      inc(cbRead, sz);
      stm.Write(@buffer[0], sz, @sz);
      inc(cbWritten, sz);
      if sz = 0 then
        begin
          Result := E_FAIL;
          Exit;
        end;
      dec(cb, sz);
    end;
  Result := S_OK;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if libNewSize<0 then
    begin
      Result := STG_E_INVALIDFUNCTION;
      Exit;
    end;
  try
    FStream.Size := libNewSize;
    Result := S_OK;
  except
    // TODO: return different error value according to exception like STG_E_MEDIUMFULL
    Result := E_FAIL;
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
end;

function TStreamAdapter.Commit(grfCommitFlags: Longint): HResult; stdcall;
begin
  if m_bReverted then
    Result := STG_E_REVERTED
  else
    Result := S_OK;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TStreamAdapter.Revert: HResult; stdcall;
begin
  m_bReverted := True;
  Result := S_OK;
end;
=======
function TStreamAdapter.CopyTo(stm: IStream; cb: Largeint; out cbRead: Largeint; out cbWritten: Largeint): HResult; stdcall;
var
  sz: dword;
  buffer : array[0..1023] of byte;
begin
=======
function TStreamAdapter.CopyTo(stm: IStream; cb: Largeint; out cbRead: Largeint; out cbWritten: Largeint): HResult; stdcall;
var
  sz: dword;
  buffer : array[0..1023] of byte;
begin
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

  // the method is similar to TStream.CopyFrom => use CopyFrom implementation
  cbWritten := 0;
  cbRead := 0;
  while cb > 0 do
    begin
      if (cb > sizeof(buffer)) then
        sz := sizeof(Buffer)
      else
        sz := cb;
      sz := FStream.Read(buffer, sz);
      inc(cbRead, sz);
      stm.Write(@buffer[0], sz, @sz);
      inc(cbWritten, sz);
      if sz = 0 then
        begin
          Result := E_FAIL;
          Exit;
        end;
      dec(cb, sz);
    end;
  Result := S_OK;
end;

function TStreamAdapter.LockRegion(libOffset: LargeUint; cb: LargeUint; dwLockType: Longint): HResult; stdcall;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result := STG_E_INVALIDFUNCTION;
=======

=======

>>>>>>> origin/cpstrnew
  // the method is similar to TStream.CopyFrom => use CopyFrom implementation
  cbWritten := 0;
  cbRead := 0;
  while cb > 0 do
    begin
      if (cb > sizeof(buffer)) then
        sz := sizeof(Buffer)
      else
        sz := cb;
      sz := FStream.Read(buffer, sz);
      inc(cbRead, sz);
      stm.Write(@buffer[0], sz, @sz);
      inc(cbWritten, sz);
      if sz = 0 then
        begin
          Result := E_FAIL;
          Exit;
        end;
      dec(cb, sz);
    end;
  Result := S_OK;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


function TStreamAdapter.UnlockRegion(libOffset: LargeUint; cb: LargeUint; dwLockType: Longint): HResult; stdcall;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result := STG_E_INVALIDFUNCTION;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if m_bReverted then
    Result := STG_E_REVERTED
  else
    Result := S_OK;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
  if m_bReverted then
    Result := STG_E_REVERTED
  else
    Result := S_OK;
end;

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TStreamAdapter.Revert: HResult; stdcall;
begin
  m_bReverted := True;
  Result := S_OK;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


function TStreamAdapter.Stat(out statstg: TStatStg; grfStatFlag: Longint): HResult; stdcall;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  if grfStatFlag in [STATFLAG_DEFAULT,STATFLAG_NOOPEN,STATFLAG_NONAME] then
  begin
    if @statstg <> nil then
    begin
      fillchar(statstg, sizeof(TStatStg),#0);

      { //TODO handle pwcsName
        if grfStatFlag = STATFLAG_DEFAULT then
          runerror(217) //Result :={$ifdef windows} STG_E_INVALIDFLAG{$else}E_INVALID_FLAG{$endif}
      }

      statstg.dwType := STGTY_STREAM;
      statstg.cbSize := FStream.Size;
      statstg.grfLocksSupported := LOCK_WRITE;
    end;
    Result := S_OK;
  end else
    Result := STG_E_INVALIDFLAG
=======
  Result := STG_E_INVALIDFUNCTION;
>>>>>>> graemeg/cpstrnew
=======
  Result := STG_E_INVALIDFUNCTION;
>>>>>>> origin/cpstrnew
end;

function TStreamAdapter.Clone(out stm: IStream): HResult; stdcall;
begin
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  // don't raise an exception here return error value that function is not implemented
  // to implement this we need a clone method for TStream class
  Result := STG_E_UNIMPLEMENTEDFUNCTION;
=======
  Result := STG_E_INVALIDFUNCTION;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;

constructor TProxyStream.Create(const Stream: IStream);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FStream := Stream;
=======
  Result := STG_E_INVALIDFUNCTION;
>>>>>>> graemeg/cpstrnew
=======
  Result := STG_E_INVALIDFUNCTION;
>>>>>>> origin/cpstrnew
end;

function TProxyStream.Read(var Buffer; Count: Longint): Longint;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Check(FStream.Read(@Buffer, Count, @Result));
=======
  Result := STG_E_INVALIDFUNCTION;
>>>>>>> graemeg/cpstrnew
=======
  Result := STG_E_INVALIDFUNCTION;
>>>>>>> graemeg/cpstrnew
end;

function TProxyStream.Seek(const Offset: int64; Origin: TSeekOrigin): int64;
begin
<<<<<<< HEAD
  Check(FStream.Seek(Offset, ord(Origin), QWord(result)));
end;

function TProxyStream.Write(const Buffer; Count: Longint): Longint;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Check(FStream.Write(@Buffer, Count, @Result));
=======
  Result := STG_E_INVALIDFUNCTION;
>>>>>>> graemeg/cpstrnew
end;

function TProxyStream.GetIStream: IStream;
begin
<<<<<<< HEAD
  Result := FStream;
end;

procedure TProxyStream.Check(err:integer);
var e : EInOutError;
begin
<<<<<<< HEAD
  e:= EInOutError.Create('Proxystream.Check');
  e.Errorcode:=err;
  raise e;
end;

=======
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  if grfStatFlag in [STATFLAG_DEFAULT,STATFLAG_NOOPEN,STATFLAG_NONAME] then
  begin
    if @statstg <> nil then
    begin
      fillchar(statstg, sizeof(TStatStg),#0);
      
      { //TODO handle pwcsName
        if grfStatFlag = STATFLAG_DEFAULT then
          runerror(217) //Result :={$ifdef windows} STG_E_INVALIDFLAG{$else}E_INVALID_FLAG{$endif}
      }

      statstg.dwType := STGTY_STREAM;
      statstg.cbSize := FStream.Size;
      statstg.grfLocksSupported := LOCK_WRITE;
    end;
    Result := S_OK;
  end else
    Result := STG_E_INVALIDFLAG
end; 
=======
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  if grfStatFlag in [STATFLAG_DEFAULT,STATFLAG_NOOPEN,STATFLAG_NONAME] then
  begin
    if @statstg <> nil then
    begin
      fillchar(statstg, sizeof(TStatStg),#0);
      
      { //TODO handle pwcsName
        if grfStatFlag = STATFLAG_DEFAULT then
          runerror(217) //Result :={$ifdef windows} STG_E_INVALIDFLAG{$else}E_INVALID_FLAG{$endif}
      }

      statstg.dwType := STGTY_STREAM;
      statstg.cbSize := FStream.Size;
      statstg.grfLocksSupported := LOCK_WRITE;
    end;
    Result := S_OK;
  end else
    Result := STG_E_INVALIDFLAG
end; 
=======
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  if grfStatFlag in [STATFLAG_DEFAULT,STATFLAG_NOOPEN,STATFLAG_NONAME] then
  begin
    if @statstg <> nil then
    begin
      fillchar(statstg, sizeof(TStatStg),#0);
      
      { //TODO handle pwcsName
        if grfStatFlag = STATFLAG_DEFAULT then
          runerror(217) //Result :={$ifdef windows} STG_E_INVALIDFLAG{$else}E_INVALID_FLAG{$endif}
      }

      statstg.dwType := STGTY_STREAM;
      statstg.cbSize := FStream.Size;
      statstg.grfLocksSupported := LOCK_WRITE;
    end;
    Result := S_OK;
  end else
    Result := STG_E_INVALIDFLAG
end; 
=======
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  if grfStatFlag in [STATFLAG_DEFAULT,STATFLAG_NOOPEN,STATFLAG_NONAME] then
  begin
    if @statstg <> nil then
    begin
      fillchar(statstg, sizeof(TStatStg),#0);
      
      { //TODO handle pwcsName
        if grfStatFlag = STATFLAG_DEFAULT then
          runerror(217) //Result :={$ifdef windows} STG_E_INVALIDFLAG{$else}E_INVALID_FLAG{$endif}
      }

      statstg.dwType := STGTY_STREAM;
      statstg.cbSize := FStream.Size;
      statstg.grfLocksSupported := LOCK_WRITE;
    end;
    Result := S_OK;
  end else
    Result := STG_E_INVALIDFLAG
end; 

function TStreamAdapter.Clone(out stm: IStream): HResult; stdcall;
begin
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  // don't raise an exception here return error value that function is not implemented
  // to implement this we need a clone method for TStream class
  Result := STG_E_UNIMPLEMENTEDFUNCTION;
end;

constructor TProxyStream.Create(const Stream: IStream);
begin
  FStream := Stream;
end;
>>>>>>> origin/cpstrnew

function TProxyStream.Read(var Buffer; Count: Longint): Longint;
begin
<<<<<<< HEAD
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  // don't raise an exception here return error value that function is not implemented
  // to implement this we need a clone method for TStream class
  Result := STG_E_UNIMPLEMENTEDFUNCTION;
end;

constructor TProxyStream.Create(const Stream: IStream);
begin
  FStream := Stream;
end;
>>>>>>> graemeg/cpstrnew

function TProxyStream.Read(var Buffer; Count: Longint): Longint;
begin
<<<<<<< HEAD
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  // don't raise an exception here return error value that function is not implemented
  // to implement this we need a clone method for TStream class
  Result := STG_E_UNIMPLEMENTEDFUNCTION;
end;

constructor TProxyStream.Create(const Stream: IStream);
begin
  FStream := Stream;
end;
>>>>>>> graemeg/cpstrnew

function TProxyStream.Read(var Buffer; Count: Longint): Longint;
begin
<<<<<<< HEAD
  if m_bReverted then
    begin
      Result := STG_E_REVERTED;
      Exit;
    end;
  // don't raise an exception here return error value that function is not implemented
  // to implement this we need a clone method for TStream class
  Result := STG_E_UNIMPLEMENTEDFUNCTION;
end;

constructor TProxyStream.Create(const Stream: IStream);
begin
  FStream := Stream;
end;

function TProxyStream.Read(var Buffer; Count: Longint): Longint;
begin
=======
>>>>>>> graemeg/cpstrnew
  Check(FStream.Read(@Buffer, Count, @Result));
end;

function TProxyStream. Seek(const Offset: int64; Origin: TSeekOrigin): int64;
begin
  Check(FStream.Seek(Offset, ord(Origin), result));
end;

function TProxyStream.Write(const Buffer; Count: Longint): Longint;
begin
  Check(FStream.Write(@Buffer, Count, @Result));
end;

function TProxyStream.GetIStream: IStream;
begin
  Result := FStream;
end; 

procedure TProxyStream.Check(err:integer);
var e : EInOutError;
begin 
  e:= EInOutError.Create('Proxystream.Check');
  e.Errorcode:=err;
  raise e;
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  Check(FStream.Read(@Buffer, Count, @Result));
end;

=======
  Check(FStream.Read(@Buffer, Count, @Result));
end;

>>>>>>> origin/cpstrnew
function TProxyStream. Seek(const Offset: int64; Origin: TSeekOrigin): int64;
begin
  Check(FStream.Seek(Offset, ord(Origin), result));
end;

function TProxyStream.Write(const Buffer; Count: Longint): Longint;
begin
  Check(FStream.Write(@Buffer, Count, @Result));
end;

function TProxyStream.GetIStream: IStream;
begin
  Result := FStream;
end; 

procedure TProxyStream.Check(err:integer);
var e : EInOutError;
begin 
  e:= EInOutError.Create('Proxystream.Check');
  e.Errorcode:=err;
  raise e;
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  
{$warnings off}
function TStreamAdapter.Read(pv: Pointer; cb: DWORD; pcbRead: PDWORD): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.Write(pv: Pointer; cb: DWORD; pcbWritten: PDWORD): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.Seek(dlibMove: Largeint; dwOrigin: Longint; out libNewPosition: Largeint): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.SetSize(libNewSize: Largeint): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.CopyTo(stm: IStream; cb: Largeint; out cbRead: Largeint; out cbWritten: Largeint): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.Commit(grfCommitFlags: Longint): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.Revert: HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.LockRegion(libOffset: Largeint; cb: Largeint; dwLockType: Longint): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.UnlockRegion(libOffset: Largeint; cb: Largeint; dwLockType: Longint): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.Stat(out statstg: TStatStg; grfStatFlag: Longint): HResult; stdcall;
begin
  runerror(217);
end;


function TStreamAdapter.Clone(out stm: IStream): HResult; stdcall;
begin
  runerror(217);
end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$warnings on}
