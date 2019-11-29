{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    This unit converts a stream to a regular text file.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
{$H+}

unit StreamIO;

interface

uses Classes,SysUtils;

Procedure AssignStream(var F: Textfile; Stream: TStream);
Function GetStream(var F: TTextRec) : TStream;

implementation

ResourceString
  SErrNilStream = 'Can not assign file to Nil stream';

Type
  PStream = ^TStream;

{ ---------------------------------------------------------------------
    Text IO functions
  ---------------------------------------------------------------------}


procedure StreamRead(var F: TTextRec);

begin
  InOutRes:=0;
  With F do
    Try
      Bufend:=GetStream(F).Read(BufPtr^,BufSize);
      BufPos:=0;
    except
      InOutRes:=100;
    end;
end;


procedure StreamWrite(var F: TTextRec );
begin
  InOutRes:=0;
  with F do
    if (BufPos>0) then
      try
        GetStream(F).WriteBuffer(BufPtr^,BufPos);
        BufPos:=0;
      except
        InOutRes:=101;
      end;
end;


{$PUSH}
{$WARN 5024 OFF : Parameter "$1" not used}
Procedure StreamFlush(var F: TTextRec);

begin
  InOutRes:=0;
end;


procedure StreamClose(var F: TTextRec);
begin
  InOutRes:=0;
end;
{$POP}

Procedure StreamOpen(var F: TTextRec );

begin
  InOutRes:=0;
  with F do
    begin
    BufPos:=0;
    Bufend:=0;
    case Mode of
      fmInput:
        begin
        InOutFunc:=@StreamRead;
        FlushFunc:=@StreamFlush;
        end;
      fmOutput,fmAppend:
        begin
        InOutFunc:=@StreamWrite;
        FlushFunc:=@StreamWrite;
        if mode=fmAppend then
          Try
            GetStream(F).Seek(0,soFromEnd);
          except
            InOutRes:=156;
          end;
        end;
    end;
    end;
end;


{ ---------------------------------------------------------------------
    Public functions
  ---------------------------------------------------------------------}


Procedure AssignStream(var F: Textfile; Stream : TStream);

Var
  E : EInoutError;

begin
  if (Stream=Nil) then
    begin
    E:=EInOutError.Create(SErrNilStream);
    E.ErrorCode:=6;
    Raise E;
    end;
  with TTextRec(F) do
    begin
    
    OpenFunc:=@StreamOpen;
    CloseFunc:=@StreamClose;
    Case DefaultTextLineBreakStyle Of
      tlbsLF: LineEnd:=#10;
      tlbsCRLF: LineEnd:=#13#10;
      tlbsCR: LineEnd:=#13;
    End;
    PStream(@UserData)^:=Stream;
    Mode:=fmClosed;
    BufSize:=SizeOf(Buffer);
    BufPtr:=@Buffer;
    Name[0]:=#0;
    end;
   SetTextCodePage(F,CP_ACP); 
end;


Function GetStream(var F: TTextRec) : TStream;

begin
  Result:=PStream(@F.Userdata)^;
end;

end.
