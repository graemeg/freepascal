{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{****************************************************************************
                    subroutines For TextFile handling
****************************************************************************}

Procedure FileCloseFunc(Var t:TextRec);
Begin
  Do_Close(t.Handle);
  t.Handle:=UnusedHandle;
End;

Procedure FileReadFunc(var t:TextRec);
Begin
  t.BufEnd:=Do_Read(t.Handle,t.Bufptr,t.BufSize);
  t.BufPos:=0;
End;


Procedure FileWriteFunc(var t:TextRec);
var
  i : longint;
Begin
  { prevent unecessary system call }
  if t.BufPos=0 then
    exit;
  i:=Do_Write(t.Handle,t.Bufptr,t.BufPos);
  if i<>t.BufPos then
    InOutRes:=101;
  t.BufPos:=0;
End;


Procedure FileOpenFunc(var t:TextRec);
var
  Flags : Longint;
Begin
  Case t.mode Of
    fmInput : Flags:=$10000;
    fmOutput : Flags:=$11001;
    fmAppend : Flags:=$10101;
  else
   begin
     InOutRes:=102;
     exit;
   end;
  End;
{$ifdef USE_FILEREC_FULLNAME}
  if Assigned(t.FullName) then
    Do_Open(t,PFileTextRecChar(t.FullName),Flags,False)
  else
{$endif USE_FILEREC_FULLNAME}
    Do_Open(t,PFileTextRecChar(@t.Name),Flags,False);
  t.CloseFunc:=@FileCloseFunc;
  t.FlushFunc:=nil;
  if t.Mode=fmInput then
   t.InOutFunc:=@FileReadFunc
  else
   begin
     t.InOutFunc:=@FileWriteFunc;
     { Only install flushing if its a NOT a file, and only check if there
       was no error opening the file, because else we always get a bad
       file handle error 6 (PFV) }
     if (InOutRes=0) and
        Do_Isdevice(t.Handle) then
      t.FlushFunc:=@FileWriteFunc;
   end;
End;

Procedure InitText(Var t : Text);

begin
  FillChar(t,SizeOf(TextRec),0);
{ only set things that are not zero }
  TextRec(t).Handle:=UnusedHandle;
  TextRec(t).mode:=fmClosed;
  TextRec(t).BufSize:=TextRecBufSize;
  TextRec(t).Bufptr:=@TextRec(t).Buffer;
  TextRec(t).OpenFunc:=@FileOpenFunc;
  Case DefaultTextLineBreakStyle Of
    tlbsLF: TextRec(t).LineEnd := #10;
    tlbsCRLF: TextRec(t).LineEnd := #13#10;
    tlbsCR: TextRec(t).LineEnd := #13;
  End;
end;


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure Assign(out t:Text;const s : UnicodeString);
begin
  InitText(t);
{$ifdef FPC_ANSI_TEXTFILEREC}
  TextRec(t).Name:=ToSingleByteFileSystemEncodedFileName(S);
{$ifdef USE_FILEREC_FULLNAME}
  if length(s)>255 then
    RawByteString(TextRec(t).FullName):=ToSingleByteFileSystemEncodedFileName(S);
{$endif USE_FILEREC_FULLNAME}
{$else FPC_ANSI_TEXTFILEREC}
  TextRec(t).Name:=S;
{$ifdef USE_FILEREC_FULLNAME}
  if length(s)>255 then
    UnicodeString(TextRec(t).FullName):=S;
{$endif USE_FILEREC_FULLNAME}
{$endif FPC_ANSI_TEXTFILEREC}
  { null terminate, since the name array is regularly used as p(wide)char }
  TextRec(t).Name[high(TextRec(t).Name)]:=#0;
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure Assign(out t:Text;const s: RawByteString);
Begin
  InitText(t);
{$ifdef FPC_ANSI_TEXTFILEREC}
  { ensure the characters in the record's filename are encoded correctly }
  TextRec(t).Name:=ToSingleByteFileSystemEncodedFileName(S);
{$ifdef USE_FILEREC_FULLNAME}
  if length(s)>255 then
    RawByteString(TextRec(t).FullName):=ToSingleByteFileSystemEncodedFileName(S);
{$endif USE_FILEREC_FULLNAME}
{$else FPC_ANSI_TEXTFILEREC}
  TextRec(t).Name:=S;
{$ifdef USE_FILEREC_FULLNAME}
  if length(s)>255 then
    UnicodeString(TextRec(t).FullName):=S;
{$endif USE_FILEREC_FULLNAME}
{$endif FPC_ANSI_TEXTFILEREC}
  { null terminate, since the name array is regularly used as p(wide)char }
  TextRec(t).Name[high(TextRec(t).Name)]:=#0;
End;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


Procedure Assign(out t:Text;const s: ShortString);
Begin
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  Assign(t,AnsiString(s));
{$else FPC_HAS_FEATURE_ANSISTRINGS}
  InitText(t);
  { warning: no encoding support }
  TextRec(t).Name:=s;
  { null terminate, since the name array is regularly used as p(wide)char }
  TextRec(t).Name[high(TextRec(t).Name)]:=#0;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
End;


Procedure Assign(out t:Text;const p: PAnsiChar);
Begin
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  Assign(t,AnsiString(p));
{$else FPC_HAS_FEATURE_ANSISTRINGS}
  { no use in making this the one that does the work, since the name field is
    limited to 255 characters anyway }
  Assign(t,strpas(p));
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
End;


Procedure Assign(out t:Text;const c: AnsiChar);
Begin
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  Assign(t,AnsiString(c));
{$else FPC_HAS_FEATURE_ANSISTRINGS}
  Assign(t,ShortString(c));
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
End;


Procedure Close(var t : Text);[IOCheck];
Begin
  if InOutRes<>0 then
   Exit;
  case TextRec(t).mode of
    fmInput,fmOutput,fmAppend:
      Begin
        { Write pending buffer }
        If Textrec(t).Mode=fmoutput then
          FileFunc(TextRec(t).InOutFunc)(TextRec(t));
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
        { Only close functions not connected to stdout.}
        If ((TextRec(t).Handle<>StdInputHandle) and
            (TextRec(t).Handle<>StdOutputHandle) and
            (TextRec(t).Handle<>StdErrorHandle)) Then
{$endif FPC_HAS_FEATURE_CONSOLEIO}
          FileFunc(TextRec(t).CloseFunc)(TextRec(t));
        TextRec(t).mode := fmClosed;
        { Reset buffer for safety }
        TextRec(t).BufPos:=0;
        TextRec(t).BufEnd:=0;
      End
    else inOutRes := 103;
  End;
{$ifdef USE_FILEREC_FULLNAME}
{$ifdef FPC_ANSI_TEXTFILEREC}
  RawByteString(TextRec(t).FullName):='';
{$else FPC_ANSI_TEXTFILEREC}
  UnicodeString(TextRec(t).FullName):='';
{$endif FPC_ANSI_TEXTFILEREC}
{$endif USE_FILEREC_FULLNAME}
End;


Procedure OpenText(var t : Text;mode,defHdl:Longint);
Begin
  Case TextRec(t).mode Of {This gives the fastest code}
   fmInput,fmOutput,fmInOut : Close(t);
   fmClosed : ;
  else
   Begin
     InOutRes:=102;
     exit;
   End;
  End;
  TextRec(t).mode:=mode;
  TextRec(t).bufpos:=0;
  TextRec(t).bufend:=0;

{$ifdef FPC_HAS_CPSTRING}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  { if no codepage is yet assigned then assign default ansi codepage }
  TextRec(t).CodePage:=TranslatePlaceholderCP(TextRec(t).CodePage);
{$else FPC_HAS_FEATURE_ANSISTRINGS}
  TextRec(t).CodePage:=0;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$endif FPC_HAS_CPSTRING}
  FileFunc(TextRec(t).OpenFunc)(TextRec(t));
  { reset the mode to closed when an error has occurred }
  if InOutRes<>0 then
   TextRec(t).mode:=fmClosed;
End;


Procedure Rewrite(var t : Text);[IOCheck];
Begin
  If InOutRes<>0 then
   exit;
  OpenText(t,fmOutput,1);
End;


Procedure Reset(var t : Text);[IOCheck];
Begin
  If InOutRes<>0 then
   exit;
  OpenText(t,fmInput,0);
End;


Procedure Append(var t : Text);[IOCheck];
Begin
  If InOutRes<>0 then
   exit;
  OpenText(t,fmAppend,1);
End;


Procedure Flush(var t : Text);[IOCheck];
Begin
  If InOutRes<>0 then
   exit;
  if TextRec(t).mode<>fmOutput then
   begin
     if TextRec(t).mode=fmInput then
      InOutRes:=105
     else
      InOutRes:=103;
     exit;
   end;
{ Not the flushfunc but the inoutfunc should be used, because that
  writes the data, flushfunc doesn't need to be assigned }
  FileFunc(TextRec(t).InOutFunc)(TextRec(t));
End;


Procedure Erase(var t:Text);[IOCheck];
Begin
  if InOutRes<>0 then
    exit;
  if TextRec(t).mode<>fmClosed then
    begin
      InOutRes:=102;
      exit;
    end;
  Do_Erase(PFileTextRecChar(@TextRec(t).Name),false);
End;


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure Rename(var t : Text;const s : unicodestring);[IOCheck];
{$ifdef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
var
  fs: RawByteString;
{$endif FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
Begin
  if InOutRes<>0 then
    exit;
  if TextRec(t).mode<>fmClosed then
    begin
      InOutRes:=102;
      exit;
    end;
{$ifdef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  { it's slightly faster to convert the unicodestring here to rawbytestring
    than doing it in do_rename(), because here we still know the length }
  fs:=ToSingleByteFileSystemEncodedFileName(s);
  Do_Rename(PFileTextRecChar(@TextRec(t).Name),PAnsiChar(fs),false,true);
  If InOutRes=0 then
     TextRec(t).Name:=fs
{$else FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  Do_Rename(PFileTextRecChar(@TextRec(t).Name),PUnicodeChar(S),false,false);
  If InOutRes=0 then
{$ifdef FPC_ANSI_TEXTTextRec}
    TextRec(t).Name:=ToSingleByteFileSystemEncodedFileName(s);
{$else FPC_ANSI_TEXTFILEREC}
    TextRec(t).Name:=s
{$endif FPC_ANSI_TEXTFILEREC}
{$endif FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
End;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}



{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure Rename(var t : Text;const s : rawbytestring);[IOCheck];
var
{$ifdef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  fs: RawByteString;
  pdst: PAnsiChar;
{$else FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  fs: UnicodeString;
  pdst: PUnicodeChar;
{$endif FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  dstchangeable: boolean;
Begin
  if InOutRes<>0 then
    exit;
  if TextRec(t).mode<>fmClosed then
    begin
      InOutRes:=102;
      exit;
    end;
{$ifdef FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  dstchangeable:=false;
  pdst:=PAnsiChar(s);
  if TranslatePlaceholderCP(StringCodePage(s))<>DefaultFileSystemCodePage then
    begin
      fs:=ToSingleByteFileSystemEncodedFileName(s);
      pdst:=PAnsiChar(fs);
      dstchangeable:=true;
    end
  else
    fs:=s;
{$else FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
   { it's slightly faster to convert the rawbytestring here to unicodestring
     than doing it in do_rename, because here we still know the length }
   fs:=unicodestring(s);
   pdst:=PUnicodeChar(fs);
   dstchangeable:=true;
{$endif FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
  Do_Rename(PFileTextRecChar(@TextRec(t).Name),pdst,false,dstchangeable);
  If InOutRes=0 then
{$if defined(FPC_ANSI_TEXTTextRec) and not defined(FPCRTL_FILESYSTEM_SINGLE_BYTE_API)}
    TextRec(t).Name:=ToSingleByteFileSystemEncodedFileName(fs)
{$else FPC_ANSI_TEXTTextRec and not FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
    TextRec(t).Name:=fs
{$endif FPC_ANSI_TEXTTextRec and not FPCRTL_FILESYSTEM_SINGLE_BYTE_API}
End;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


Procedure Rename(var t : Text;const s : ShortString);[IOCheck];
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Begin
  Rename(t,AnsiString(s));
End;
{$else FPC_HAS_FEATURE_ANSISTRINGS}
var
  p : array[0..255] Of Char;
Begin
  Move(s[1],p,Length(s));
  p[Length(s)]:=#0;
  Rename(t,Pchar(@p));
End;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

Procedure Rename(var t:Text;const p:PAnsiChar);
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Begin
  Rename(t,AnsiString(p));
End;
{$else FPC_HAS_FEATURE_ANSISTRINGS}
var
  len: SizeInt;
Begin
  if InOutRes<>0 then
    exit;
  if TextRec(t).mode<>fmClosed then
    begin
      InOutRes:=102;
      exit;
    end;
  Do_Rename(PFileTextRecChar(@TextRec(t).Name),p,false,false);
  { check error code of do_rename }
  if InOutRes=0 then
    begin
      len:=min(StrLen(p),high(TextRec(t).Name));
      Move(p^,TextRec(t).Name,len);
      TextRec(t).Name[len]:=#0;
    end;
End;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

Procedure Rename(var t : Text;const c : AnsiChar);[IOCheck];
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Begin
  Rename(t,AnsiString(c));
End;
{$else FPC_HAS_FEATURE_ANSISTRINGS}
var
  p : array[0..1] Of AnsiChar;
Begin
  p[0]:=c;
  p[1]:=#0;
  Rename(t,PAnsiChar(@p));
End;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

Function Eof(Var t: Text): Boolean;[IOCheck];
Begin
  If (InOutRes<>0) then
   exit(true);
  if (TextRec(t).mode<>fmInput) Then
   begin
     if TextRec(t).mode=fmOutput then
      InOutRes:=104
     else
      InOutRes:=103;
     exit(true);
   end;
  If TextRec(t).BufPos>=TextRec(t).BufEnd Then
   begin
     FileFunc(TextRec(t).InOutFunc)(TextRec(t));
     If TextRec(t).BufPos>=TextRec(t).BufEnd Then
      exit(true);
   end;
  Eof:=CtrlZMarksEOF and (TextRec(t).Bufptr^[TextRec(t).BufPos]=#26);
end;


Function Eof:Boolean;
Begin
  Eof:=Eof(Input);
End;


Function SeekEof (Var t : Text) : Boolean;
Begin
  If (InOutRes<>0) then
   exit(true);
  if (TextRec(t).mode<>fmInput) Then
   begin
     if TextRec(t).mode=fmOutPut then
      InOutRes:=104
     else
      InOutRes:=103;
     exit(true);
   end;
  repeat
    If TextRec(t).BufPos>=TextRec(t).BufEnd Then
     begin
       FileFunc(TextRec(t).InOutFunc)(TextRec(t));
       If TextRec(t).BufPos>=TextRec(t).BufEnd Then
        begin
          SeekEof := true;
          break;
        end;
     end;
    case TextRec(t).Bufptr^[TextRec(t).BufPos] of
      #26 :
        if CtrlZMarksEOF then
          begin
            SeekEof := true;
            break;
          end;
     #10,#13,#9,' ' :
       ;
    else
     begin
       SeekEof := false;
       break;
     end;
    end;
   inc(TextRec(t).BufPos);
  until false;
End;


Function SeekEof : Boolean;
Begin
  SeekEof:=SeekEof(Input);
End;


Function Eoln(var t:Text) : Boolean;
Begin
  If (InOutRes<>0) then
   exit(true);
  if (TextRec(t).mode<>fmInput) Then
   begin
     if TextRec(t).mode=fmOutPut then
      InOutRes:=104
     else
      InOutRes:=103;
     exit(true);
   end;
  If TextRec(t).BufPos>=TextRec(t).BufEnd Then
   begin
     FileFunc(TextRec(t).InOutFunc)(TextRec(t));
     If TextRec(t).BufPos>=TextRec(t).BufEnd Then
      exit(true);
   end;
  if CtrlZMarksEOF and (TextRec (T).BufPtr^[TextRec (T).BufPos] = #26) then
   exit (true);
  Eoln:=(TextRec(t).Bufptr^[TextRec(t).BufPos] in [#10,#13]);
End;


Function Eoln : Boolean;
Begin
  Eoln:=Eoln(Input);
End;


Function SeekEoln (Var t : Text) : Boolean;
Begin
  If (InOutRes<>0) then
   exit(true);
  if (TextRec(t).mode<>fmInput) Then
   begin
     if TextRec(t).mode=fmOutput then
      InOutRes:=104
     else
      InOutRes:=103;
     exit(true);
   end;
  repeat
    If TextRec(t).BufPos>=TextRec(t).BufEnd Then
     begin
       FileFunc(TextRec(t).InOutFunc)(TextRec(t));
       If TextRec(t).BufPos>=TextRec(t).BufEnd Then
        exit(true);
     end;
    case TextRec(t).Bufptr^[TextRec(t).BufPos] of
         #26: if CtrlZMarksEOF then
               exit (true);
     #10,#13 : exit(true);
      #9,' ' : ;
    else
     exit(false);
    end;
    inc(TextRec(t).BufPos);
  until false;
End;


Function SeekEoln : Boolean;
Begin
  SeekEoln:=SeekEoln(Input);
End;


Procedure SetTextBuf(Var F : Text; Var Buf; Size : SizeInt);
Begin
  TextRec(f).BufPtr:=@Buf;
  TextRec(f).BufSize:=Size;
  TextRec(f).BufPos:=0;
  TextRec(f).BufEnd:=0;
End;

Procedure SetTextLineEnding(Var f:Text; Ending:string);
Begin
  TextRec(F).LineEnd:=Ending;
End;

function GetTextCodePage(var T: Text): TSystemCodePage;
begin
{$if defined(FPC_HAS_CPSTRING) and defined(FPC_HAS_FEATURE_ANSISTRINGS)}
  GetTextCodePage:=TextRec(T).CodePage;
{$else}
  GetTextCodePage:=0;
{$endif}
end;


procedure SetTextCodePage(var T: Text; CodePage: TSystemCodePage);
begin
{$if defined(FPC_HAS_CPSTRING) and defined(FPC_HAS_FEATURE_ANSISTRINGS)}
  TextRec(T).CodePage:=TranslatePlaceholderCP(CodePage);
{$endif}
end;


procedure SetTextAutoFlush (var T: Text; AutoFlush: boolean);[IOCheck];
Begin
  If InOutRes<>0 then
   exit;
  if TextRec(T).mode<>fmOutput then
   begin
     if TextRec(T).mode=fmInput then
      InOutRes:=105
     else
      InOutRes:=103;
     exit;
   end;
  if AutoFlush then
   TextRec(T).FlushFunc := TextRec(T).InOutFunc
  else
   TextRec(T).FlushFunc := nil;
End;


function GetTextAutoFlush (var T: Text): boolean;[IOCheck];
Begin
  GetTextAutoFlush := false;
  If InOutRes<>0 then
   exit;
  if TextRec(t).mode<>fmOutput then
   begin
     if TextRec(t).mode=fmInput then
      InOutRes:=105
     else
      InOutRes:=103;
     exit;
   end;
  GetTextAutoFlush := Assigned (TextRec(T).FlushFunc);
End;


Function fpc_get_input:PText;compilerproc;
begin
  fpc_get_input:=@Input;
end;


Function fpc_get_output:PText;compilerproc;
begin
  fpc_get_output:=@Output;
end;


Procedure fpc_textinit_iso(var t : Text;nr : DWord);compilerproc;
begin
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
  assign(t,paramstr(nr));
{$else FPC_HAS_FEATURE_COMMANDARGS}
  { primitive workaround for targets supporting no command line arguments,
    invent some file name, try to avoid complex procedures like concating strings which might
    pull-in bigger parts of the rtl }
  assign(t,chr((nr mod 16)+65));
{$endif FPC_HAS_FEATURE_COMMANDARGS}
end;


Procedure fpc_textinit_filename_iso(var t : Text;nr : DWord;const filename : string);compilerproc;
begin
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
  if paramstr(nr)='' then
    assign(t,filename+'.txt')
  else
    assign(t,paramstr(nr));
{$else FPC_HAS_FEATURE_COMMANDARGS}
  { primitive workaround for targets supporting no command line arguments,
    invent some file name, try to avoid complex procedures like concating strings which might
    pull-in bigger parts of the rtl }
  assign(t,chr((nr mod 16)+65));
{$endif FPC_HAS_FEATURE_COMMANDARGS}
end;



Procedure fpc_textclose_iso(var t : Text);compilerproc;
begin
  { reset inout result as this procedure is only called by the compiler and no I/O checking is carried out,
    so further I/O does not fail }
  inoutres:=0;
  close(t);
  inoutres:=0;
end;


{*****************************************************************************
                               Write(Ln)
*****************************************************************************}

Procedure fpc_WriteBuffer(var f:Text;const b;len:SizeInt);
var
  p   : pchar;
  left,
  idx : SizeInt;
begin
  p:=pchar(@b);
  idx:=0;
  left:=TextRec(f).BufSize-TextRec(f).BufPos;
  while len>left do
   begin
     move(p[idx],TextRec(f).Bufptr^[TextRec(f).BufPos],left);
     dec(len,left);
     inc(idx,left);
     inc(TextRec(f).BufPos,left);
     FileFunc(TextRec(f).InOutFunc)(TextRec(f));
     left:=TextRec(f).BufSize-TextRec(f).BufPos;
   end;
  move(p[idx],TextRec(f).Bufptr^[TextRec(f).BufPos],len);
  inc(TextRec(f).BufPos,len);
end;


Procedure fpc_WriteBlanks(var f:Text;len:longint);
var
  left : longint;
begin
  left:=TextRec(f).BufSize-TextRec(f).BufPos;
  while len>left do
   begin
     FillChar(TextRec(f).Bufptr^[TextRec(f).BufPos],left,' ');
     dec(len,left);
     inc(TextRec(f).BufPos,left);
     FileFunc(TextRec(f).InOutFunc)(TextRec(f));
     left:=TextRec(f).BufSize-TextRec(f).BufPos;
   end;
  FillChar(TextRec(f).Bufptr^[TextRec(f).BufPos],len,' ');
  inc(TextRec(f).BufPos,len);
end;


Procedure fpc_Write_End(var f:Text); iocheck; compilerproc;
begin
  if TextRec(f).FlushFunc<>nil then
   FileFunc(TextRec(f).FlushFunc)(TextRec(f));
end;


Procedure fpc_Writeln_End(var f:Text); iocheck; compilerproc;
begin
  If InOutRes <> 0 then exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        { Write EOL }
        fpc_WriteBuffer(f,TextRec(f).LineEnd[1],length(TextRec(f).LineEnd));
        { Flush }
        if TextRec(f).FlushFunc<>nil then
          FileFunc(TextRec(f).FlushFunc)(TextRec(f));
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
end;


Procedure fpc_Write_Text_ShortStr(Len : Longint;var f : Text;const s : String); iocheck; [Public,Alias:'FPC_WRITE_TEXT_SHORTSTR']; compilerproc;
Begin
  If (InOutRes<>0) then
   exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        If Len>Length(s) Then
          fpc_WriteBlanks(f,Len-Length(s));
        fpc_WriteBuffer(f,s[1],Length(s));
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
End;


Procedure fpc_Write_Text_ShortStr_Iso(Len : Longint;var f : Text;const s : String); iocheck; [Public,Alias:'FPC_WRITE_TEXT_SHORTSTR_ISO']; compilerproc;
Begin
  If (InOutRes<>0) then
   exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        { default value? }
        If Len=-1 then
          Len:=length(s);

        If Len>Length(s) Then
          begin
            fpc_WriteBlanks(f,Len-Length(s));
            fpc_WriteBuffer(f,s[1],Length(s));
          end
        else
          fpc_WriteBuffer(f,s[1],Len);
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
End;


{ provide local access to write_str }
procedure Write_Str(Len : Longint;var f : Text;const s : String); iocheck; [external name 'FPC_WRITE_TEXT_SHORTSTR'];

{ provide local access to write_str_iso }
procedure Write_Str_Iso(Len : Longint;var f : Text;const s : String); iocheck; [external name 'FPC_WRITE_TEXT_SHORTSTR_ISO'];

Procedure fpc_Write_Text_Pchar_as_Array(Len : Longint;var f : Text;const s : array of char; zerobased: boolean = true); iocheck; compilerproc;
var
  ArrayLen : longint;
  p : pchar;
Begin
  If (InOutRes<>0) then
   exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        p:=pchar(@s);
        if zerobased then
          begin
            { can't use StrLen, since that one could try to read past the end }
            { of the heap (JM)                                                }
            ArrayLen:=IndexByte(p^,high(s)+1,0);
            { IndexByte returns -1 if not found (JM) }
            if ArrayLen = -1 then
              ArrayLen := high(s)+1;
          end
        else
          ArrayLen := high(s)+1;
        If Len>ArrayLen Then
          fpc_WriteBlanks(f,Len-ArrayLen);
        fpc_WriteBuffer(f,p^,ArrayLen);
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
End;


Procedure fpc_Write_Text_Pchar_as_Array_Iso(Len : Longint;var f : Text;const s : array of char; zerobased: boolean = true); iocheck; compilerproc;
var
  ArrayLen : longint;
  p : pchar;
Begin
  If (InOutRes<>0) then
   exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        p:=pchar(@s);
        if zerobased then
          begin
            { can't use StrLen, since that one could try to read past the end }
            { of the heap (JM)                                                }
            ArrayLen:=IndexByte(p^,high(s)+1,0);
            { IndexByte returns -1 if not found (JM) }
            if ArrayLen = -1 then
              ArrayLen := high(s)+1;
          end
        else
          ArrayLen := high(s)+1;

        { default value? }
        If Len=-1 then
          Len:=ArrayLen;

        If Len>ArrayLen Then
          begin
            fpc_WriteBlanks(f,Len-ArrayLen);
            fpc_WriteBuffer(f,p^,ArrayLen);
          end
        else
          fpc_WriteBuffer(f,p^,Len);
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
End;


Procedure fpc_Write_Text_PChar_As_Pointer(Len : Longint;var f : Text;p : PChar); iocheck; compilerproc;
var
  PCharLen : longint;
Begin
  If (p=nil) or (InOutRes<>0) then
   exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        PCharLen:=StrLen(p);
        If Len>PCharLen Then
          fpc_WriteBlanks(f,Len-PCharLen);
        fpc_WriteBuffer(f,p^,PCharLen);
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
End;


Procedure fpc_Write_Text_AnsiStr (Len : Longint; Var f : Text; const S : RawByteString); iocheck; [Public,alias:'FPC_WRITE_TEXT_ANSISTR']; compilerproc;
{
 Writes a AnsiString to the Text file T
}
var
  SLen: longint;
  a: RawByteString;
begin
  If (InOutRes<>0) then
   exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        SLen:=Length(s);
        If Len>SLen Then
          fpc_WriteBlanks(f,Len-SLen);
        if SLen > 0 then
          begin
            {$if defined(FPC_HAS_CPSTRING) and defined(FPC_HAS_FEATURE_ANSISTRINGS)}
            if TextRec(f).CodePage<>TranslatePlaceholderCP(StringCodePage(S)) then
              begin
                a:=fpc_AnsiStr_To_AnsiStr(S,TextRec(f).CodePage);
                fpc_WriteBuffer(f,PAnsiChar(a)^,Length(a));
              end
            else
            {$endif}
            fpc_WriteBuffer(f,PAnsiChar(s)^,SLen);
          end;
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
end;


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_Write_Text_UnicodeStr (Len : Longint; Var f : Text; const S : UnicodeString); iocheck; compilerproc;
{
 Writes a UnicodeString to the Text file T
}
var
  SLen: longint;
  a: RawByteString;
begin
  If (pointer(S)=nil) or (InOutRes<>0) then
   exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        SLen:=Length(s);
        If Len>SLen Then
          fpc_WriteBlanks(f,Len-SLen);
        {$ifdef FPC_HAS_CPSTRING}
        WideStringManager.Unicode2AnsiMoveProc(PUnicodeChar(S),a,TextRec(f).CodePage,SLen);
        {$else}
        a:=s;
        {$endif FPC_HAS_CPSTRING}
        { length(a) can be > slen, e.g. after utf-16 -> utf-8 }
        fpc_WriteBuffer(f,PAnsiChar(a)^,Length(a));
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Procedure fpc_Write_Text_WideStr (Len : Longint; Var f : Text; const S : WideString); iocheck; compilerproc;
{
 Writes a WideString to the Text file T
}
var
  SLen: longint;
  a: RawByteString;
begin
  If (pointer(S)=nil) or (InOutRes<>0) then
   exit;
  case TextRec(f).mode of
    fmOutput { fmAppend gets changed to fmOutPut in do_open (JM) }:
      begin
        SLen:=Length(s);
        If Len>SLen Then
          fpc_WriteBlanks(f,Len-SLen);
        {$ifdef FPC_HAS_CPSTRING}
        widestringmanager.Wide2AnsiMoveProc(PWideChar(s), a, TextRec(f).CodePage, SLen);
        {$else}
        a:=s;
        {$endif}
        { length(a) can be > slen, e.g. after utf-16 -> utf-8 }
        fpc_WriteBuffer(f,PAnsiChar(a)^,Length(a));
      end;
    fmInput: InOutRes:=105
    else InOutRes:=103;
  end;
end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

Procedure fpc_Write_Text_SInt(Len : Longint;var t : Text;l : ValSInt); iocheck; compilerproc;
var
  s : String;
Begin
  If (InOutRes<>0) then
   exit;
  Str(l,s);
  Write_Str(Len,t,s);
End;


Procedure fpc_Write_Text_UInt(Len : Longint;var t : Text;l : ValUInt); iocheck; compilerproc;
var
  s : String;
Begin
  If (InOutRes<>0) then
   exit;
  Str(L,s);
  Write_Str(Len,t,s);
End;


Procedure fpc_Write_Text_SInt_Iso(Len : Longint;var t : Text;l : ValSInt); iocheck; compilerproc;
var
  s : String;
Begin
  If (InOutRes<>0) then
   exit;
  Str(l,s);
  { default value? }
  if len=-1 then
    len:=11
  else if len<length(s) then
    len:=length(s);
  Write_Str_Iso(Len,t,s);
End;


Procedure fpc_Write_Text_UInt_Iso(Len : Longint;var t : Text;l : ValUInt); iocheck; compilerproc;
var
  s : String;
Begin
  If (InOutRes<>0) then
   exit;
  Str(L,s);
  { default value? }
  if len=-1 then
    len:=11
  else if len<length(s) then
    len:=length(s);
  Write_Str_Iso(Len,t,s);
End;

{$ifndef CPU64}
procedure fpc_write_text_qword(len : longint;var t : text;q : qword); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(q,s);
  write_str(len,t,s);
end;


procedure fpc_write_text_int64(len : longint;var t : text;i : int64); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(i,s);
  write_str(len,t,s);
end;


procedure fpc_write_text_qword_iso(len : longint;var t : text;q : qword); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
    exit;
  str(q,s);
  { default value? }
  if len=-1 then
    len:=20
  else if len<length(s) then
    len:=length(s);
  write_str_iso(len,t,s);
end;


procedure fpc_write_text_int64_iso(len : longint;var t : text;i : int64); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(i,s);
  { default value? }
  if len=-1 then
    len:=20
  else if len<length(s) then
    len:=length(s);
  write_str_iso(len,t,s);
end;

{$endif CPU64}

{$if defined(CPU16) or defined(CPU8)}
procedure fpc_write_text_longword(len : longint;var t : text;q : longword); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(q,s);
  write_str(len,t,s);
end;


procedure fpc_write_text_longint(len : longint;var t : text;i : longint); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(i,s);
  write_str(len,t,s);
end;


procedure fpc_write_text_longword_iso(len : longint;var t : text;q : longword); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
    exit;
  str(q,s);
  { default value? }
  if len=-1 then
    len:=11
  else if len<length(s) then
    len:=length(s);
  write_str_iso(len,t,s);
end;


procedure fpc_write_text_longint_iso(len : longint;var t : text;i : longint); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(i,s);
  { default value? }
  if len=-1 then
    len:=11
  else if len<length(s) then
    len:=length(s);
  write_str_iso(len,t,s);
end;


procedure fpc_write_text_word(len : longint;var t : text;q : word); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(q,s);
  write_str(len,t,s);
end;


procedure fpc_write_text_smallint(len : longint;var t : text;i : smallint); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(i,s);
  write_str(len,t,s);
end;


procedure fpc_write_text_word_iso(len : longint;var t : text;q : word); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
    exit;
  str(q,s);
  { default value? }
  if len=-1 then
    len:=11
  else if len<length(s) then
    len:=length(s);
  write_str_iso(len,t,s);
end;


procedure fpc_write_text_smallint_iso(len : longint;var t : text;i : smallint); iocheck; compilerproc;
var
  s : string;
begin
  if (InOutRes<>0) then
   exit;
  str(i,s);
  { default value? }
  if len=-1 then
    len:=11
  else if len<length(s) then
    len:=length(s);
  write_str_iso(len,t,s);
end;
{$endif CPU16 or CPU8}

{$ifndef FPUNONE}
Procedure fpc_Write_Text_Float(rt,fixkomma,Len : Longint;var t : Text;r : ValReal); iocheck; compilerproc;
var
  s : String;
Begin
  If (InOutRes<>0) then
   exit;
  Str_real(Len,fixkomma,r,treal_type(rt),s);
  Write_Str(Len,t,s);
End;


Procedure fpc_Write_Text_Float_iso(rt,fixkomma,Len : Longint;var t : Text;r : ValReal); iocheck; compilerproc;
var
  s : String;
Begin
  If (InOutRes<>0) then
   exit;
  Str_real_iso(Len,fixkomma,r,treal_type(rt),s);
  Write_Str(Len,t,s);
End;
{$endif}

procedure fpc_write_text_enum(typinfo,ord2strindex:pointer;len:sizeint;var t:text;ordinal:longint); iocheck; compilerproc;

var
    s:string;

begin
{$ifdef EXCLUDE_COMPLEX_PROCS}
  runerror(219);
{$else EXCLUDE_COMPLEX_PROCS}
  if textrec(t).mode<>fmoutput then
    begin
      if textrec(t).mode=fminput then
        inoutres:=105
      else
        inoutres:=103;
      exit;
    end;
  inoutres := fpc_shortstr_enum_intern(ordinal, len, typinfo, ord2strindex, s);
  if (inoutres <> 0) then
    exit;
  fpc_writeBuffer(t,s[1],length(s));
{$endif EXCLUDE_COMPLEX_PROCS}
end;

Procedure fpc_Write_Text_Currency(fixkomma,Len : Longint;var t : Text;c : Currency); iocheck; compilerproc;
{$ifdef EXCLUDE_COMPLEX_PROCS}
      begin
        runerror(217);
      end;
{$else EXCLUDE_COMPLEX_PROCS}
var
  s : String;
Begin
  If (InOutRes<>0) then
   exit;
  str(c:Len:fixkomma,s);
  Write_Str(Len,t,s);
End;
{$endif EXCLUDE_COMPLEX_PROCS}

Procedure fpc_Write_Text_Boolean(Len : Longint;var t : Text;b : Boolean); iocheck; compilerproc;
Begin
  If (InOutRes<>0) then
   exit;
{ Can't use array[boolean] because b can be >0 ! }
  if b then
    Write_Str(Len,t,'TRUE')
  else
    Write_Str(Len,t,'FALSE');
End;


Procedure fpc_Write_Text_Boolean_Iso(Len : Longint;var t : Text;b : Boolean); iocheck; compilerproc;
Begin
  If (InOutRes<>0) then
   exit;
  { Can't use array[boolean] because b can be >0 ! }
  { default value? }
  If Len=-1 then
    Len:=5;
  if b then
    Write_Str_Iso(Len,t,'true')
  else
    Write_Str_Iso(Len,t,'false');
End;


Procedure fpc_Write_Text_Char(Len : Longint;var t : Text;c : Char); iocheck; compilerproc;
Begin
  If (InOutRes<>0) then
    exit;
  if (TextRec(t).mode<>fmOutput) Then
   begin
     if TextRec(t).mode=fmClosed then
      InOutRes:=103
     else
      InOutRes:=105;
     exit;
   end;
  If Len>1 Then
    fpc_WriteBlanks(t,Len-1);
  If TextRec(t).BufPos>=TextRec(t).BufSize Then
    FileFunc(TextRec(t).InOutFunc)(TextRec(t));
  TextRec(t).Bufptr^[TextRec(t).BufPos]:=c;
  Inc(TextRec(t).BufPos);
End;


Procedure fpc_Write_Text_Char_Iso(Len : Longint;var t : Text;c : Char); iocheck; compilerproc;
Begin
  If (InOutRes<>0) then
    exit;
  if (TextRec(t).mode<>fmOutput) Then
   begin
     if TextRec(t).mode=fmClosed then
      InOutRes:=103
     else
      InOutRes:=105;
     exit;
   end;
  { default value? }
  If Len=-1 then
    Len:=1;
  If Len>1 Then
    fpc_WriteBlanks(t,Len-1)
  else If Len<1 Then
    exit;
  If TextRec(t).BufPos>=TextRec(t).BufSize Then
    FileFunc(TextRec(t).InOutFunc)(TextRec(t));
  TextRec(t).Bufptr^[TextRec(t).BufPos]:=c;
  Inc(TextRec(t).BufPos);
End;


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_Write_Text_WideChar(Len : Longint;var t : Text;c : WideChar); iocheck; compilerproc;
var
  a: RawByteString;
Begin
  If (InOutRes<>0) then
    exit;
  if (TextRec(t).mode<>fmOutput) Then
   begin
     if TextRec(t).mode=fmClosed then
      InOutRes:=103
     else
      InOutRes:=105;
     exit;
   end;
  If Len>1 Then
    fpc_WriteBlanks(t,Len-1);
  If TextRec(t).BufPos>=TextRec(t).BufSize Then
    FileFunc(TextRec(t).InOutFunc)(TextRec(t));
  { a widechar can be translated into more than a single ansichar }
  {$ifdef FPC_HAS_CPSTRING}
  widestringmanager.Wide2AnsiMoveProc(@c,a,TextRec(t).CodePage,1);
  {$else}
  a:=c;
  {$endif}
  fpc_WriteBuffer(t,PAnsiChar(a)^,Length(a));
End;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


{*****************************************************************************
                                Read(Ln)
*****************************************************************************}

Function NextChar(var f:Text;var s:string):Boolean;
begin
  NextChar:=false;
  if (TextRec(f).BufPos<TextRec(f).BufEnd) then
   if not (CtrlZMarksEOF) or (TextRec(f).Bufptr^[TextRec(f).BufPos]<>#26) then
    begin
     if length(s)<high(s) then
      begin
        inc(s[0]);
        s[length(s)]:=TextRec(f).BufPtr^[TextRec(f).BufPos];
      end;
     Inc(TextRec(f).BufPos);
     If TextRec(f).BufPos>=TextRec(f).BufEnd Then
      FileFunc(TextRec(f).InOutFunc)(TextRec(f));
     NextChar:=true;
   end;
end;


Function IgnoreSpaces(var f:Text):Boolean;
{
  Removes all leading spaces,tab,eols from the input buffer, returns true if
  the buffer is empty
}
var
  s : string;
begin
  s:='';
  IgnoreSpaces:=false;
  { Return false when already at EOF }
  if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
   exit;
(* Check performed separately to avoid accessing memory outside buffer *)
  if CtrlZMarksEOF and (TextRec(f).Bufptr^[TextRec(f).BufPos]=#26) then
   exit;
  while (TextRec(f).Bufptr^[TextRec(f).BufPos] <= ' ') do
   begin
     if not NextChar(f,s) then
      exit;
     { EOF? }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      break;
     if CtrlZMarksEOF and (TextRec(f).Bufptr^[TextRec(f).BufPos]=#26) then
      break;
   end;
  IgnoreSpaces:=true;
end;


procedure ReadNumeric(var f:Text;var s:string);
{
  Read numeric input, if buffer is empty then return True
}
begin
  repeat
    if not NextChar(f,s) then
      exit;
  until (length(s)=high(s)) or (TextRec(f).BufPtr^[TextRec(f).BufPos] <= ' ');
end;


function CheckRead(var f:Text):Boolean;
begin
  CheckRead:=False;
{ Check error and if file is open and load buf if empty }
  If (InOutRes<>0) then
    exit;
  if (TextRec(f).mode<>fmInput) Then
    begin
      case TextRec(f).mode of
        fmOutPut,fmAppend:
          InOutRes:=104;
        else
          InOutRes:=103;
      end;
      exit;
    end;
  if TextRec(f).BufPos>=TextRec(f).BufEnd Then
    FileFunc(TextRec(f).InOutFunc)(TextRec(f));
  CheckRead:=InOutRes=0;
end;


procedure ReadInteger(var f:Text;var s:string);
{
 Ignore leading blanks (incl. EOF) and return the first characters matching
 an integer in the format recognized by the Val procedure:
      [+-]?[0-9]+
   or [+-]?(0x|0X|x|X)[0-9A-Za-z]+
   or [+-]?&[0-7]+
   or [+-]?%[0-1]+
 A partial match may be returned, e.g.: '' or '+' or '0x'.
 Used by some fpc_Read_Text_*_Iso functions which implement the read()
 standard function in ISO mode.
}
var
  Base: Integer;
begin
    s := '';
    with TextRec(f) do begin
        if not CheckRead(f) then Exit;

        IgnoreSpaces(f);

        if BufPos >= BufEnd then Exit;
        if BufPtr^[BufPos] in ['+','-'] then
            NextChar(f,s);

        Base := 10;

        if BufPos >= BufEnd then Exit;
        if BufPtr^[BufPos] in ['$','x','X','%','&'] then
        begin
            case BufPtr^[BufPos] of
              '$','x','X': Base := 16;
	      '%': Base := 2;
              '&': Base := 8;
	    end;
            NextChar(f,s);
        end else if BufPtr^[BufPos] = '0' then
        begin
            NextChar(f,s);
            if BufPos >= BufEnd then Exit;
            if BufPtr^[BufPos] in ['x','X'] then
            begin
                Base := 16;
                NextChar(f,s);
            end;
        end;

        while (BufPos < BufEnd) and (Length(s) < High(s)) do
            if (((Base = 2) and (BufPtr^[BufPos] in ['0'..'1']))
	      or ((Base = 8) and (BufPtr^[BufPos] in ['0'..'7']))
              or ((Base = 10) and (BufPtr^[BufPos] in ['0'..'9']))
              or ((Base = 16) and (BufPtr^[BufPos] in ['0'..'9','a'..'f','A'..'F']))) then
                 NextChar(f,s)
	    else Exit;
   end;
end;


procedure ReadReal(var f:Text;var s:string);
{
 Ignore leading blanks (incl. EOF) and return the first characters matching
 a float number in the format recognized by the Val procedure:
      [+-]?([0-9]+)?\.[0-9]+([eE][+-]?[0-9]+)?
   or [+-]?[0-9]+\.([0-9]+)?([eE][+-]?[0-9]+)?
 A partial match may be returned, e.g.: '' or '+' or '.' or '1e' or even '+.'.
 Used by some fpc_Read_Text_*_Iso functions which implement the read()
 standard function in ISO mode.
}
var digit: Boolean;
begin
    s := '';
    with TextRec(f) do begin
        if not CheckRead(f) then Exit;

        IgnoreSpaces(f);

        if BufPos >= BufEnd then Exit;
        if BufPtr^[BufPos] in ['+','-'] then
            NextChar(f,s);

        digit := false;
        if BufPos >= BufEnd then Exit;
	if BufPtr^[BufPos] in ['0'..'9'] then
        begin
            digit := true;
            repeat
                NextChar(f,s);
                if (BufPos >= BufEnd) or (Length(s) >= High(s)) then Exit;
            until not (BufPtr^[BufPos] in ['0'..'9']);
        end;

        if BufPtr^[BufPos] = '.' then
        begin
            NextChar(f,s);

            if (BufPos >= BufEnd) or (Length(s) >= High(s)) then Exit;
	    if BufPtr^[BufPos] in ['0'..'9'] then
            begin
                digit := true;
                repeat
                    NextChar(f,s);
                    if (BufPos >= BufEnd) or (Length(s) >= High(s)) then Exit;
                until not (BufPtr^[BufPos] in ['0'..'9']);
            end;
        end;

        {at least one digit is required on the left of the exponent}
        if digit and (BufPtr^[BufPos] in ['e','E']) then
        begin
            NextChar(f,s);

            if (BufPos >= BufEnd) or (Length(s) >= High(s)) then Exit;
            if BufPtr^[BufPos] in ['+','-'] then
                NextChar(f,s);

	    while (BufPos < BufEnd) and (Length(s) < High(s)) do
                if BufPtr^[BufPos] in ['0'..'9'] then
                    NextChar(f,s)
                else break;
        end;
    end;
end;


Procedure fpc_Read_End(var f:Text);[Public,Alias:'FPC_READ_END']; iocheck; compilerproc;
begin
  if TextRec(f).FlushFunc<>nil then
   FileFunc(TextRec(f).FlushFunc)(TextRec(f));
end;


Procedure fpc_ReadLn_End(var f : Text);[Public,Alias:'FPC_READLN_END']; iocheck; compilerproc;
var prev: char;
Begin
  If not CheckRead(f) then
    exit;
  if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
    { Flush if set }
    begin
      if (TextRec(f).FlushFunc<>nil) then
        FileFunc(TextRec(f).FlushFunc)(TextRec(f));
      exit;
    end;
  if CtrlZMarksEOF and (TextRec (F).BufPtr^ [TextRec (F).BufPos] = #26) then
   Exit;
  repeat
    prev := TextRec(f).BufPtr^[TextRec(f).BufPos];
    inc(TextRec(f).BufPos);
{ no system uses #10#13 as line seperator (#10 = *nix, #13 = Mac, }
{ #13#10 = Dos), so if we've got #10, we can safely exit          }
    if prev = #10 then
      exit;
    {$ifdef MACOS}
    if prev = #13 then
      {StdInput on macos never have dos line ending, so this is safe.}
      if TextRec(f).Handle = StdInputHandle then
        exit;
    {$endif MACOS}
    if TextRec(f).BufPos>=TextRec(f).BufEnd Then
      begin
        FileFunc(TextRec(f).InOutFunc)(TextRec(f));
        if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
          { Flush if set }
          begin
           if (TextRec(f).FlushFunc<>nil) then
             FileFunc(TextRec(f).FlushFunc)(TextRec(f));
           exit;
         end;
      end;
   if CtrlZMarksEOF and (TextRec (F).BufPtr^ [TextRec (F).BufPos] = #26) then
    Exit;
   if (prev=#13) then
     { is there also a #10 after it? }
     begin
       if (TextRec(f).BufPtr^[TextRec(f).BufPos]=#10) then
         { yes, skip that one as well }
         inc(TextRec(f).BufPos);
       exit;
     end;
  until false;
End;


Procedure fpc_ReadLn_End_Iso(var f : Text);[Public,Alias:'FPC_READLN_END_ISO']; iocheck; compilerproc;
var prev: char;
Begin
  If not CheckRead(f) then
    exit;
  if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
    { Flush if set }
    begin
      if (TextRec(f).FlushFunc<>nil) then
        FileFunc(TextRec(f).FlushFunc)(TextRec(f));
      exit;
    end;
  if TextRec (F).BufPtr^ [TextRec (F).BufPos] = #26 then
    begin
      inc(TextRec(f).BufPos);
      Exit;
    end;
  repeat
    prev := TextRec(f).BufPtr^[TextRec(f).BufPos];
    inc(TextRec(f).BufPos);
{ no system uses #10#13 as line seperator (#10 = *nix, #13 = Mac, }
{ #13#10 = Dos), so if we've got #10, we can safely exit          }
    if prev = #10 then
      exit;
    {$ifdef MACOS}
    if prev = #13 then
      {StdInput on macos never have dos line ending, so this is safe.}
      if TextRec(f).Handle = StdInputHandle then
        exit;
    {$endif MACOS}
    if TextRec(f).BufPos>=TextRec(f).BufEnd Then
      begin
        FileFunc(TextRec(f).InOutFunc)(TextRec(f));
        if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
          { Flush if set }
          begin
           if (TextRec(f).FlushFunc<>nil) then
             FileFunc(TextRec(f).FlushFunc)(TextRec(f));
           exit;
         end;
      end;
   if TextRec (F).BufPtr^ [TextRec (F).BufPos] = #26 then
     begin
       inc(TextRec(f).BufPos);
       Exit;
     end;
   if (prev=#13) then
     { is there also a #10 after it? }
     begin
       if (TextRec(f).BufPtr^[TextRec(f).BufPos]=#10) then
         { yes, skip that one as well }
         inc(TextRec(f).BufPos);
       exit;
     end;
  until false;
End;


Function ReadPCharLen(var f:Text;s:pchar;maxlen:longint):longint;
var
  sPos,len : Longint;
  p,startp,maxp : pchar;
  end_of_string:boolean;
Begin
{$ifdef EXCLUDE_COMPLEX_PROCS}
  runerror(219);
{$else EXCLUDE_COMPLEX_PROCS}
  ReadPCharLen:=0;
  If not CheckRead(f) then
    exit;
{ Read maximal until Maxlen is reached }
  sPos:=0;
  end_of_string:=false;
  repeat
    If TextRec(f).BufPos>=TextRec(f).BufEnd Then
     begin
       FileFunc(TextRec(f).InOutFunc)(TextRec(f));
       If TextRec(f).BufPos>=TextRec(f).BufEnd Then
         break;
     end;
    p:=@TextRec(f).Bufptr^[TextRec(f).BufPos];
    if SPos+TextRec(f).BufEnd-TextRec(f).BufPos>MaxLen then
     maxp:=@TextRec(f).BufPtr^[TextRec(f).BufPos+MaxLen-SPos]
    else
     maxp:=@TextRec(f).Bufptr^[TextRec(f).BufEnd];
    startp:=p;
  { find stop character }
    while p<maxp do
      begin
        { Optimization: Do a quick check for a control character first }
        if (p^<' ') then
          begin
            if (p^ in [#10,#13]) or
               (ctrlZmarkseof and (p^=#26)) then
              begin
                end_of_string:=true;
                break;
              end;
          end;
        inc(p);
      end;
  { calculate read bytes }
    len:=p-startp;
    inc(TextRec(f).BufPos,Len);
    Move(startp^,s[sPos],Len);
    inc(sPos,Len);
  until (spos=MaxLen) or end_of_string;
  ReadPCharLen:=spos;
{$endif EXCLUDE_COMPLEX_PROCS}
End;


Procedure fpc_Read_Text_ShortStr(var f : Text;out s : String); iocheck; compilerproc;
Begin
  s[0]:=chr(ReadPCharLen(f,pchar(@s[1]),high(s)));
End;


Procedure fpc_Read_Text_PChar_As_Pointer(var f : Text; const s : PChar); iocheck; compilerproc;
Begin
  pchar(s+ReadPCharLen(f,s,$7fffffff))^:=#0;
End;


Procedure fpc_Read_Text_PChar_As_Array(var f : Text;out s : array of char; zerobased: boolean = false); iocheck; compilerproc;
var
  len: longint;
Begin
  len := ReadPCharLen(f,pchar(@s),high(s)+1);
  if zerobased and
     (len > high(s)) then
    len := high(s);
  if (len <= high(s)) then
    s[len] := #0;
End;


{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure fpc_Read_Text_AnsiStr(var f : Text;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); [public, alias: 'FPC_READ_TEXT_ANSISTR']; iocheck; compilerproc;
var
  slen,len : SizeInt;
Begin
  slen:=0;
  Repeat
    // SetLength will reallocate the length.
    SetLength(s,slen+255);
    len:=ReadPCharLen(f,pchar(Pointer(s)+slen),255);
    inc(slen,len);
  Until len<255;
  // Set actual length
  SetLength(s,Slen);
  {$ifdef FPC_HAS_CPSTRING}
  SetCodePage(s,TextRec(f).CodePage,false);
  if cp<>TextRec(f).CodePage then
    s:=fpc_AnsiStr_To_AnsiStr(s,cp);
  {$endif FPC_HAS_CPSTRING}
End;

Procedure fpc_Read_Text_AnsiStr_Intern(var f : Text;out s : RawByteString{$ifdef FPC_HAS_CPSTRING};cp : TSystemCodePage{$endif FPC_HAS_CPSTRING}); [external name 'FPC_READ_TEXT_ANSISTR'];
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure fpc_Read_Text_UnicodeStr(var f : Text;out us : UnicodeString); iocheck; compilerproc;
var
  s: RawByteString;
Begin
  // all standard input is assumed to be ansi-encoded
  fpc_Read_Text_AnsiStr_Intern(f,s{$ifdef FPC_HAS_CPSTRING},DefaultSystemCodePage{$endif FPC_HAS_CPSTRING});
  // Convert to unicodestring
  {$ifdef FPC_HAS_CPSTRING}
  widestringmanager.Ansi2UnicodeMoveProc(PAnsiChar(s),StringCodePage(s),us,Length(s));
  {$else}
  us:=s;
  {$endif}
End;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
Procedure fpc_Read_Text_WideStr(var f : Text;out ws : WideString); iocheck; compilerproc;
var
  s: RawByteString;
Begin
  // all standard input is assumed to be ansi-encoded
  fpc_Read_Text_AnsiStr_Intern(f,s{$ifdef FPC_HAS_CPSTRING},DefaultSystemCodePage{$endif FPC_HAS_CPSTRING});
  // Convert to widestring
  {$ifdef FPC_HAS_CPSTRING}
  widestringmanager.Ansi2WideMoveProc(PAnsiChar(s),StringCodePage(s),ws,Length(s));
  {$else}
  ws:=s;
  {$endif}
End;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

procedure fpc_Read_Text_Char(var f : Text; out c: char); [public, alias: 'FPC_READ_TEXT_CHAR']; iocheck; compilerproc;
Begin
  c:=#0;
  If not CheckRead(f) then
    exit;
  If TextRec(f).BufPos>=TextRec(f).BufEnd Then
    begin
      c := #26;
      exit;
    end;
  c:=TextRec(f).Bufptr^[TextRec(f).BufPos];
  inc(TextRec(f).BufPos);
end;

procedure fpc_Read_Text_Char_intern(var f : Text; out c: char); iocheck; [external name 'FPC_READ_TEXT_CHAR'];


function fpc_GetBuf_Text(var f : Text) : pchar; iocheck; compilerproc;
Begin
  Result:=@TextRec(f).Bufptr^[TextRec(f).BufEnd];
  if TextRec(f).mode=fmOutput then
    exit;
  If not CheckRead(f) then
    exit;
  If TextRec(f).BufPos>=TextRec(f).BufEnd Then
    exit;
  Result:=@TextRec(f).Bufptr^[TextRec(f).BufPos];
end;


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
procedure fpc_Read_Text_WideChar(var f : Text; out wc: widechar); iocheck;compilerproc;
var
  ws: widestring;
  i: longint;
  { maximum code point length is 6 characters (with UTF-8) }
  str: array[0..5] of char;
Begin
  fillchar(str[0],sizeof(str),0);
  for i:=low(str) to high(str) do
    begin
      fpc_Read_Text_Char_intern(f,str[i]);
      case widestringmanager.CodePointLengthProc(@str[0],i+1) of
        -1: { possibly incomplete code point, try with an extra character }
           ;
        0: { null character }
          begin
            wc:=#0;
            exit;
          end;
        else
          begin
            { valid code point -> convert to widestring}
            {$ifdef FPC_HAS_CPSTRING}
            widestringmanager.Ansi2WideMoveProc(@str[0],TextRec(f).CodePage,ws,i+1);
            {$else}
            widestringmanager.Ansi2WideMoveProc(@str[0],DefaultSystemCodePage,ws,i+1);
            {$endif}
            { has to be exactly one widechar }
            if length(ws)=1 then
              begin
                wc:=ws[1];
                exit
              end
            else
              break;
          end;
      end;
    end;
  { invalid widechar input }
  inoutres:=106;
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


procedure fpc_Read_Text_Char_Iso(var f : Text; out c: char); iocheck;compilerproc;
Begin
  c:=' ';
  If not CheckRead(f) then
    exit;
  If TextRec(f).BufPos>=TextRec(f).BufEnd Then
    begin
      c:=' ';
      exit;
    end;
  c:=TextRec(f).Bufptr^[TextRec(f).BufPos];
  inc(TextRec(f).BufPos);
  if c=#13 then
    begin
      c:=' ';
      If not CheckRead(f) or
        (TextRec(f).BufPos>=TextRec(f).BufEnd) then
        exit;
      If TextRec(f).Bufptr^[TextRec(f).BufPos]=#10 then
        inc(TextRec(f).BufPos);

      { ignore #26 following a new line }
      If not CheckRead(f) or
        (TextRec(f).BufPos>=TextRec(f).BufEnd) then
        exit;
      If TextRec(f).Bufptr^[TextRec(f).BufPos]=#26 then
        inc(TextRec(f).BufPos);
    end
  else if c=#10 then
    begin
      c:=' ';
      { ignore #26 following a new line }
      If not CheckRead(f) or
        (TextRec(f).BufPos>=TextRec(f).BufEnd) then
        exit;
      If TextRec(f).Bufptr^[TextRec(f).BufPos]=#26 then
        inc(TextRec(f).BufPos);
      end
  else if c=#26 then
    c:=' ';
end;


Procedure fpc_Read_Text_SInt(var f : Text; out l : ValSInt); iocheck; compilerproc;
var
  hs   : String;
  code : ValSInt;
Begin
  l:=0;
  If not CheckRead(f) then
    exit;
  hs:='';
  if IgnoreSpaces(f) then
   begin
     { When spaces were found and we are now at EOF,
       then we return 0 }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      exit;
     if CtrlZMarksEOF and (TextRec(f).Bufptr^[TextRec(f).BufPos]=#26) then
      exit;
     ReadNumeric(f,hs);
   end;
   if (hs = '') then
    L := 0
   else
    begin
     Val(hs,l,code);
     if Code <> 0 then
      InOutRes:=106;
    end;
End;


Procedure fpc_Read_Text_SInt_Iso(var f : Text; out l : ValSInt); iocheck; compilerproc;
var
  hs   : String;
  code : ValSInt;
Begin
  l:=0;
  if not CheckRead(f) then
    Exit;

  ReadInteger(f,hs);

  Val(hs,l,code);
  if Code <> 0 then
    InOutRes:=106;
End;


Procedure fpc_Read_Text_UInt(var f : Text; out u : ValUInt);  iocheck; compilerproc;
var
  hs   : String;
  code : ValSInt;
Begin
  u:=0;
  If not CheckRead(f) then
    exit;
  hs:='';
  if IgnoreSpaces(f) then
   begin
     { When spaces were found and we are now at EOF,
       then we return 0 }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      exit;
     ReadNumeric(f,hs);
   end;
   if (hs = '') then
    u := 0
   else
    begin
      val(hs,u,code);
      If code<>0 Then
        InOutRes:=106;
    end;
End;

Procedure fpc_Read_Text_UInt_Iso(var f : Text; out u : ValUInt);  iocheck; compilerproc;
var
  hs   : String;
  code : ValSInt;
Begin
  u:=0;
  if not CheckRead(f) then
    Exit;

  ReadInteger(f,hs);
  Val(hs,u,code);
  If code<>0 Then
     InOutRes:=106;
End;


{$ifndef FPUNONE}
procedure fpc_Read_Text_Float(var f : Text; out v : ValReal); iocheck; compilerproc;
var
  hs : string;
  code : Word;
begin
  v:=0.0;
  If not CheckRead(f) then
    exit;
  hs:='';
  if IgnoreSpaces(f) then
   begin
     { When spaces were found and we are now at EOF,
       then we return 0 }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      exit;
     ReadNumeric(f,hs);
   end;
  val(hs,v,code);
  If code<>0 Then
   InOutRes:=106;
end;


procedure fpc_Read_Text_Float_Iso(var f : Text; out v : ValReal); iocheck; compilerproc;
var
  hs : string;
  code : Word;
begin
  v:=0.0;
  if not CheckRead(f) then
    Exit;

  ReadReal(f,hs);
  Val(hs,v,code);
  If code<>0 Then
    InOutRes:=106;
end;
{$endif}

procedure fpc_read_text_enum(str2ordindex:pointer;var t:text;out ordinal:longint); iocheck;compilerproc;

var s:string;
    code:valsint;

begin
  if not checkread(t) then
    exit;
  s:='';
  if ignorespaces(t) then
    begin
      { When spaces were found and we are now at EOF, then we return 0 }
      if (TextRec(t).BufPos>=TextRec(t).BufEnd) then
        exit;
      ReadNumeric(t,s);
    end;
  ordinal:=fpc_val_enum_shortstr(str2ordindex,s,code);
  if code<>0 then
   InOutRes:=106;
end;

procedure fpc_Read_Text_Currency(var f : Text; out v : Currency); iocheck; compilerproc;
var
  hs : string;
  code : ValSInt;
begin
{$ifdef FPUNONE}
  v:=0;
{$else}
  v:=0.0;
{$endif}
  If not CheckRead(f) then
    exit;
  hs:='';
  if IgnoreSpaces(f) then
   begin
     { When spaces were found and we are now at EOF,
       then we return 0 }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      exit;
     ReadNumeric(f,hs);
   end;
  val(hs,v,code);
  If code<>0 Then
   InOutRes:=106;
end;


procedure fpc_Read_Text_Currency_Iso(var f : Text; out v : Currency); iocheck; compilerproc;
var
  hs : string;
  code : ValSInt;
begin
  v:=0;
  if not CheckRead(f) then
    Exit;

  ReadReal(f,hs);
  Val(hs,v,code);
  If code<>0 Then
   InOutRes:=106;
end;


{$ifndef cpu64}

procedure fpc_Read_Text_QWord(var f : text; out q : qword); iocheck; compilerproc;
var
  hs   : String;
  code : longint;
Begin
  q:=0;
  If not CheckRead(f) then
    exit;
  hs:='';
  if IgnoreSpaces(f) then
   begin
     { When spaces were found and we are now at EOF,
       then we return 0 }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      exit;
     ReadNumeric(f,hs);
   end;
  val(hs,q,code);
  If code<>0 Then
   InOutRes:=106;
End;

procedure fpc_Read_Text_QWord_Iso(var f : text; out q : qword); iocheck; compilerproc;
var
  hs   : String;
  code : longint;
Begin
  q:=0;
  if not CheckRead(f) then
    Exit;

  ReadInteger(f,hs);
  Val(hs,q,code);
  If code<>0 Then
    InOutRes:=106;
End;

procedure fpc_Read_Text_Int64(var f : text; out i : int64); iocheck; compilerproc;
var
  hs   : String;
  code : Longint;
Begin
  i:=0;
  If not CheckRead(f) then
    exit;
  hs:='';
  if IgnoreSpaces(f) then
   begin
     { When spaces were found and we are now at EOF,
       then we return 0 }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      exit;
     ReadNumeric(f,hs);
   end;
  Val(hs,i,code);
  If code<>0 Then
   InOutRes:=106;
End;

procedure fpc_Read_Text_Int64_Iso(var f : text; out i : int64); iocheck; compilerproc;
var
  hs   : String;
  code : Longint;
Begin
  i:=0;
  if not CheckRead(f) then
    Exit;

  ReadInteger(f,hs);
  Val(hs,i,code);
  If code<>0 Then
    InOutRes:=106;
End;


{$endif CPU64}

{$if defined(CPU16) or defined(CPU8)}
procedure fpc_Read_Text_LongWord(var f : text; out q : longword); iocheck; compilerproc;
var
  hs   : String;
  code : longint;
Begin
  q:=0;
  If not CheckRead(f) then
    exit;
  hs:='';
  if IgnoreSpaces(f) then
   begin
     { When spaces were found and we are now at EOF,
       then we return 0 }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      exit;
     ReadNumeric(f,hs);
   end;
  val(hs,q,code);
  If code<>0 Then
   InOutRes:=106;
End;

procedure fpc_Read_Text_LongInt(var f : text; out i : longint); iocheck; compilerproc;
var
  hs   : String;
  code : Longint;
Begin
  i:=0;
  If not CheckRead(f) then
    exit;
  hs:='';
  if IgnoreSpaces(f) then
   begin
     { When spaces were found and we are now at EOF,
       then we return 0 }
     if (TextRec(f).BufPos>=TextRec(f).BufEnd) then
      exit;
     ReadNumeric(f,hs);
   end;
  Val(hs,i,code);
  If code<>0 Then
   InOutRes:=106;
End;
{$endif CPU16 or CPU8}



{*****************************************************************************
                              WriteStr/ReadStr
*****************************************************************************}

const
  { pointer to target string }
  StrPtrIndex = 1;
  { temporary destination for writerstr, because the original value of the
    destination may be used in the writestr expression }
  TempWriteStrDestIndex = 9;
  ShortStrLenIndex = 17;
  { how many bytes of the string have been processed already (used for readstr) }
  BytesReadIndex = 17;

procedure WriteStrShort(var t: textrec);
var
  str: pshortstring;
  newbytes,
  oldlen: longint;
begin
  if (t.bufpos=0) then
    exit;
  str:=pshortstring(ppointer(@t.userdata[TempWriteStrDestIndex])^);
  newbytes:=t.BufPos;
  oldlen:=length(str^);
  if (oldlen+t.bufpos > t.userdata[ShortStrLenIndex]) then
    begin
      newbytes:=t.userdata[ShortStrLenIndex]-oldlen;
{$ifdef writestr_iolencheck}
      // GPC only gives an io error if {$no-truncate-strings} is active
      // FPC does not have this setting (it never gives errors when a
      // a string expression is truncated)

      { "disk full" }
      inoutres:=101;
{$endif}
    end;
  setlength(str^,length(str^)+newbytes);
  move(t.bufptr^,str^[oldlen+1],newbytes);
  t.bufpos:=0;
end;


procedure WriteStrShortFlush(var t: textrec);
begin
  { move written data from internal buffer to temporary string (don't move
    directly from buffer to final string, because the temporary string may
    already contain data in case the textbuf was smaller than the string
    length) }
  WriteStrShort(t);
  { move written data to original string }
  move(PPointer(@t.userdata[TempWriteStrDestIndex])^^,
       PPointer(@t.userdata[StrPtrIndex])^^,
       t.userdata[ShortStrLenIndex]+1);
  { free temporary buffer }
  freemem(PPointer(@t.userdata[TempWriteStrDestIndex])^);
end;



{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
procedure WriteStrAnsi(var t: textrec);
var
  str: pansistring;
  oldlen: longint;
begin
  if (t.bufpos=0) then
    exit;
  str:=pansistring(@t.userdata[TempWriteStrDestIndex]);
  oldlen:=length(str^);
  setlength(str^,oldlen+t.bufpos);
  move(t.bufptr^,str^[oldlen+1],t.bufpos);
  t.bufpos:=0;
end;


procedure WriteStrAnsiFlush(var t: textrec);
begin
  { see comment in WriteStrShortFlush }
  WriteStrAnsi(t);
  pansistring(ppointer(@t.userdata[StrPtrIndex])^)^:=
    pansistring(@t.userdata[TempWriteStrDestIndex])^;
  { free memory/finalize temp }
  pansistring(@t.userdata[TempWriteStrDestIndex])^:='';
end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
function EndOfLastCompleteUTF8CodePoint(var t: textrec): SizeInt;
var
  i, codepointlen: sizeint;
begin
  for i:=t.bufpos-1 downto 0 do
    begin
      { we don't care about combining diacritical marks here: we just want a
        valid UTF-8 codepoint that we can translate to UTF-16. The combining
        diacritical marks can be translated separately }
      codepointlen:=Utf8CodePointLen(pchar(@t.bufptr^[i]),(t.bufpos-1-i)+1,false);
      { complete codepoint -> flush till here }
      if codepointlen>0 then
        begin
          result:=i+codepointlen;
          exit;
        end
    end;
  { all invalid data, or the buffer is too small to be able to deal with the
    complete utf8char -> nothing else to do but to handle the entire buffer
    (and end up with a partial/invalid character) }
  result:=t.bufpos;
end;


procedure WriteStrUnicodeIntern(var t: textrec; flush: boolean);
var
  temp: unicodestring;
  str: punicodestring;
  validend: SizeInt;
begin
  if (t.bufpos=0) then
    exit;
  str:=punicodestring(@t.userdata[TempWriteStrDestIndex]);
  if not flush then
    validend:=EndOfLastCompleteUTF8CodePoint(t)
  else
    validend:=t.bufpos;
  widestringmanager.Ansi2UnicodeMoveProc(@t.bufptr^[0],CP_UTF8,temp,validend);
  str^:=str^+temp;
  dec(t.bufpos,validend);
  { move remainder to the start }
  if t.bufpos<>0 then
    move(t.bufptr^[validend],t.bufptr^[0],t.bufpos);
end;


procedure WriteStrUnicode(var t: textrec);
begin
  WriteStrUnicodeIntern(t,false);
end;


procedure WriteStrUnicodeFlush(var t: textrec);
begin
  { see comment in WriteStrShortFlush }
  WriteStrUnicodeIntern(t,true);
  punicodestring(ppointer(@t.userdata[StrPtrIndex])^)^:=
    punicodestring(@t.userdata[TempWriteStrDestIndex])^;
  { free memory/finalize temp }
  punicodestring(@t.userdata[TempWriteStrDestIndex])^:='';
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
procedure WriteStrWideIntern(var t: textrec; flush: boolean);
var
  temp: unicodestring;
  str: pwidestring;
  validend: SizeInt;
begin
  if (t.bufpos=0) then
    exit;
  str:=pwidestring(@t.userdata[TempWriteStrDestIndex]);
  if not flush then
    validend:=EndOfLastCompleteUTF8CodePoint(t)
  else
    validend:=t.bufpos;
  widestringmanager.Ansi2UnicodeMoveProc(@t.bufptr^[0],CP_UTF8,temp,validend);
  str^:=str^+temp;
  dec(t.bufpos,validend);
  { move remainder to the start }
  if t.bufpos<>0 then
    move(t.bufptr^[validend],t.bufptr^[0],t.bufpos);
end;


procedure WriteStrWide(var t: textrec);
begin
  WriteStrUnicodeIntern(t,false);
end;


procedure WriteStrWideFlush(var t: textrec);
begin
  { see comment in WriteStrShortFlush }
  WriteStrWideIntern(t,true);
  pwidestring(ppointer(@t.userdata[StrPtrIndex])^)^:=
    pwidestring(@t.userdata[TempWriteStrDestIndex])^;
  { free memory/finalize temp }
  finalize(pwidestring(@t.userdata[TempWriteStrDestIndex])^);
end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

procedure SetupWriteStrCommon(out t: textrec; cp: TSystemCodePage);
begin
  // initialise
  Assign(text(t),'');
  t.mode:=fmOutput;
  t.OpenFunc:=nil;
  t.CloseFunc:=nil;
{$if defined(FPC_HAS_CPSTRING) and defined(FPC_HAS_FEATURE_ANSISTRINGS)}
  t.CodePage:=TranslatePlaceholderCP(cp);
{$endif}
end;


procedure fpc_SetupWriteStr_Shortstr(var ReadWriteStrText: text; var s: shortstring); compilerproc;
begin
  SetupWriteStrCommon(TextRec(ReadWriteStrText),DefaultSystemCodePage);
  PPointer(@TextRec(ReadWriteStrText).userdata[StrPtrIndex])^:=@s;

  { temporary destination (see comments for TempWriteStrDestIndex) }
  getmem(PPointer(@TextRec(ReadWriteStrText).userdata[TempWriteStrDestIndex])^,high(s)+1);
  setlength(pshortstring(ppointer(@TextRec(ReadWriteStrText).userdata[TempWriteStrDestIndex])^)^,0);

  TextRec(ReadWriteStrText).userdata[ShortStrLenIndex]:=high(s);
  TextRec(ReadWriteStrText).InOutFunc:=@WriteStrShort;
  TextRec(ReadWriteStrText).FlushFunc:=@WriteStrShortFlush;
end;


{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
procedure fpc_SetupWriteStr_Ansistr(var ReadWriteStrText: text; var s: ansistring; cp: TSystemCodePage); compilerproc;
begin
  { destination rawbytestring -> use CP_ACP }
  if cp=CP_NONE then
    cp:=CP_ACP;
  SetupWriteStrCommon(TextRec(ReadWriteStrText),cp);
  PPointer(@TextRec(ReadWriteStrText).userdata[StrPtrIndex])^:=@s;

  { temp destination ansistring, nil = empty string }
  PPointer(@TextRec(ReadWriteStrText).userdata[TempWriteStrDestIndex])^:=nil;

  TextRec(ReadWriteStrText).InOutFunc:=@WriteStrAnsi;
  TextRec(ReadWriteStrText).FlushFunc:=@WriteStrAnsiFlush;
end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
procedure fpc_SetupWriteStr_Unicodestr(var ReadWriteStrText: text; var s: unicodestring); compilerproc;
begin
  SetupWriteStrCommon(TextRec(ReadWriteStrText),CP_UTF8);
  PPointer(@TextRec(ReadWriteStrText).userdata[StrPtrIndex])^:=@s;

  { temp destination unicodestring, nil = empty string }
  PPointer(@TextRec(ReadWriteStrText).userdata[TempWriteStrDestIndex])^:=nil;

  TextRec(ReadWriteStrText).InOutFunc:=@WriteStrUnicode;
  TextRec(ReadWriteStrText).FlushFunc:=@WriteStrUnicodeFlush;
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
procedure fpc_SetupWriteStr_Widestr(var ReadWriteStrText: text; var s: widestring); compilerproc;
begin
  SetupWriteStrCommon(TextRec(ReadWriteStrText),CP_UTF8);
  PPointer(@TextRec(ReadWriteStrText).userdata[StrPtrIndex])^:=@s;

  { temp destination widestring }
  PWideString(@TextRec(ReadWriteStrText).userdata[TempWriteStrDestIndex])^:='';

  TextRec(ReadWriteStrText).InOutFunc:=@WriteStrWide;
  TextRec(ReadWriteStrText).FlushFunc:=@WriteStrWideFlush;
end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}


{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
procedure ReadAnsiStrFinal(var t: textrec);
begin
  { finalise the temp ansistring }
  PAnsiString(@t.userdata[StrPtrIndex])^ := '';
end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


procedure ReadStrCommon(var t: textrec; strdata: pchar; len: sizeint);
var
  newbytes: sizeint;
begin
  newbytes := len - PSizeInt(@t.userdata[BytesReadIndex])^;
  if (t.BufSize <= newbytes) then
    newbytes := t.BufSize;
  if (newbytes > 0) then
    begin
      move(strdata[PSizeInt(@t.userdata[BytesReadIndex])^],t.BufPtr^,newbytes);
      inc(PSizeInt(@t.userdata[BytesReadIndex])^,newbytes);
    end;
  t.BufEnd:=newbytes;
  t.BufPos:=0;
end;


procedure ReadStrAnsi(var t: textrec);
var
  str: pansistring;
begin
  str:=pansistring(@t.userdata[StrPtrIndex]);
  ReadStrCommon(t,@str^[1],length(str^));
end;


procedure SetupReadStrCommon(out t: textrec; cp: TSystemCodePage);
begin
  // initialise
  Assign(text(t),'');
  t.mode:=fmInput;
  t.OpenFunc:=nil;
  t.CloseFunc:=nil;
{$if defined(FPC_HAS_CPSTRING) and defined(FPC_HAS_FEATURE_ANSISTRINGS)}
  t.CodePage:=TranslatePlaceholderCP(cp);
  {$endif}
  PSizeInt(@t.userdata[BytesReadIndex])^:=0;
end;


{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
procedure fpc_SetupReadStr_Ansistr(var ReadWriteStrText: text; const s: ansistring); [public, alias: 'FPC_SETUPREADSTR_ANSISTR']; compilerproc;
begin
  SetupReadStrCommon(TextRec(ReadWriteStrText),StringCodePage(s));
  { we need a reference, because 's' may be a temporary expression }
  PAnsiString(@TextRec(ReadWriteStrText).userdata[StrPtrIndex])^:=s;
  TextRec(ReadWriteStrText).InOutFunc:=@ReadStrAnsi;
  { this is called at the end, by fpc_read_end }
  TextRec(ReadWriteStrText).FlushFunc:=@ReadAnsiStrFinal;
end;

procedure fpc_SetupReadStr_Ansistr_Intern(var ReadWriteStrText: text; const s: rawbytestring); [external name 'FPC_SETUPREADSTR_ANSISTR'];
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


procedure fpc_SetupReadStr_Shortstr(var ReadWriteStrText: text; const s: shortstring); compilerproc;
begin
  { the reason we convert the short string to ansistring, is because the semantics of
    readstr are defined as:

    *********************
    Apart from the restrictions imposed by requirements given in this clause,
    the execution of readstr(e,v 1 ,...,v n ) where e denotes a
    string-expression and v 1 ,...,v n denote variable-accesses possessing the
    char-type (or a subrange of char-type), the integer-type (or a subrange of
    integer-type), the real-type, a fixed-string-type, or a
    variable-string-type, shall be equivalent to

            begin
            rewrite(f);
            writeln(f, e);
            reset(f);
            read(f, v 1 ,...,v n )
            end
    *********************

    This means that any side effects caused by the evaluation of v 1 .. v n
    must not affect the value of e (= our argument s) -> we need a copy of it.
    An ansistring is the easiest way to get a threadsafe copy, and allows us
    to use the other ansistring readstr helpers too.
  }
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  fpc_SetupReadStr_Ansistr_Intern(ReadWriteStrText,s);
{$else FPC_HAS_FEATURE_ANSISTRINGS}
  runerror(217);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
end;


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
procedure fpc_SetupReadStr_Unicodestr(var ReadWriteStrText: text; const s: unicodestring); compilerproc;
begin
  { we use an utf8string to avoid code duplication }
  fpc_SetupReadStr_Ansistr_Intern(ReadWriteStrText,utf8string(s));
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


{$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
procedure fpc_SetupReadStr_Widestr(var ReadWriteStrText: text; const s: widestring); compilerproc;
begin
  { we use an utf8string to avoid code duplication }
  fpc_SetupReadStr_Ansistr_Intern(ReadWriteStrText,utf8string(s));
end;
{$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}

{$ifdef FPC_HAS_FEATURE_UNICODESTRINGS}
Function GetFullName(var t:Text) : UnicodeString;
  begin
{$ifdef USE_FILEREC_FULLNAME}
  if Assigned(TextRec(t).FullName) then
    Result:=UnicodeString(TextRec(t).FullName)
  else
{$endif USE_FILEREC_FULLNAME}
    Result:=PFileTextRecChar(@TextRec(t).Name);
  end;
{$endif FPC_HAS_FEATURE_UNICODESTRINGS}

{*****************************************************************************
                               Initializing
*****************************************************************************}

procedure OpenStdIO(var f:text;mode:longint;hdl:thandle);
begin
  Assign(f,'');
  TextRec(f).Handle:=hdl;
  TextRec(f).Mode:=mode;
  TextRec(f).Closefunc:=@FileCloseFunc;
  case mode of
    fmInput :
      begin
        TextRec(f).InOutFunc:=@FileReadFunc;
      {$if defined(FPC_HAS_CPSTRING) and defined(FPC_HAS_FEATURE_WIDESTRINGS)}
        TextRec(f).CodePage:=WideStringManager.GetStandardCodePageProc(scpConsoleInput);
      {$endif}
      end;
    fmOutput :
      begin
        TextRec(f).InOutFunc:=@FileWriteFunc;
        {$if defined(FPC_HAS_CPSTRING) and defined(FPC_HAS_FEATURE_WIDESTRINGS)}
        TextRec(f).CodePage:=WideStringManager.GetStandardCodePageProc(scpConsoleOutput);
      {$endif}
        if Do_Isdevice(hdl) then
          TextRec(f).FlushFunc:=@FileWriteFunc;
      end;
  else
   HandleError(102);
  end;
end;


