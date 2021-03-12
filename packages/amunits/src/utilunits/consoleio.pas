{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit consoleio;

{
    History:
    First version of ConsoleIO.
    This is an translation of consoleio from PCQ Pascal.
    Just AttachConsole to a window and you have your
    own console.
    12 Sep 2000.

    Added the define use_amiga_smartlink.
    13 Jan 2003.

    Changed integer > smallint.
    10 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se

}

interface

uses exec, intuition, console, amigalib, conunit;

TYPE
    tConsoleSet = record
                     WritePort,
                     ReadPort   : pMsgPort;
                     WriteRequest,
                     ReadRequest : pIOStdReq;
                     Window     : pWindow; { not yet used }
                     Buffer     : Char;
                 end;
    pConsoleSet = ^tConsoleSet;

{
        ConsoleIO.p

        This file implements all the normal console.device stuff for
dealing with windows.  They are pulled from the ROM Kernel Manual.
See ConsoleTest.p for an example of using these routines.
}

Procedure ConPutChar(Request : pIOStdReq; Character : Char);
Procedure ConWrite(Request : pIOStdReq; Str : pchar; length : longint);
Procedure ConPutStr(Request : pIOStdReq; Str : pchar);
Procedure QueueRead(Request : pIOStdReq; Where : pchar);
Function ConGetChar(consolePort : pMsgPort; Request : pIOStdReq;
                        WhereTo : pchar) : Char;
Procedure CleanSet(con : pConsoleSet);
Function AttachConsole(w : pWindow) : pConsoleSet;
Function ReadKey(con : pConsoleSet) : Char;
Function KeyPressed(con : pConsoleSet) : Boolean;
Procedure WriteString(con : pConsoleSet; Str : Pchar);
Procedure WriteString(con : pConsoleSet; Str : string);
Function MaxX(con : pConsoleSet) : smallint;
Function MaxY(con : pConsoleSet) : smallint;
Function WhereX(con : pConsoleSet) : smallint;
Function WhereY(con : pConsoleSet) : smallint;
Procedure TextColor(con : pConsoleSet; pen : Byte);
Procedure TextBackground(con : pConsoleSet; pen : Byte);
Procedure DetachConsole(con : pConsoleSet);
Procedure ClrEOL(con : pConsoleSet);
Procedure ClrScr(con : pConsoleSet);
Procedure CursOff(con : pConsoleSet);
Procedure CursOn(con : pConsoleSet);
Procedure DelLine(con : pConsoleSet);
Function LongToStr (I : smallint) : String;
Procedure GotoXY(con : pConsoleSet; x,y : smallint);
Procedure InsLine(con : pConsoleSet);
Procedure OpenConsoleDevice;
Procedure CloseConsoleDevice;

implementation

Procedure ConPutChar(Request : pIOStdReq; Character : Char);
var
    Error : longint;
begin
    Request^.io_Command := CMD_WRITE;
    Request^.io_Data := Addr(Character);
    Request^.io_Length := 1;
    Error := DoIO(pIORequest(Request));
end;

Procedure ConWrite(Request : pIOStdReq; Str : pchar; length : longint);
var
   Error : longint;
begin
    Request^.io_Command := CMD_WRITE;
    Request^.io_Data := Str;
    Request^.io_Length := Length;
    Error := DoIO(pIORequest(Request));
end;

Procedure ConPutStr(Request : pIOStdReq; Str : pchar);
var
    Error : longint;
begin
    Request^.io_Command := CMD_WRITE;
    Request^.io_Data := Str;
    Request^.io_Length := -1;
    Error := DoIO(pIORequest(Request));
end;

Procedure QueueRead(Request : pIOStdReq; Where : pchar);
begin
    Request^.io_Command := CMD_READ;
    Request^.io_Data := Where;
    Request^.io_Length := 1;
    SendIO(pIORequest(Request));
end;

Function ConGetChar(consolePort : pMsgPort; Request : pIOStdReq;
                        WhereTo : pchar) : Char;
var
    Temp : Char;
    TempMsg : pMessage;
begin
    if GetMsg(consolePort) = Nil then begin
        TempMsg := WaitPort(consolePort);
        TempMsg := GetMsg(consolePort);
    end;
    Temp := WhereTo^;
    QueueRead(Request, WhereTo);
    ConGetChar := Temp;
end;

Procedure CleanSet(con : pConsoleSet);
begin
    with con^ do begin
        if ReadRequest <> Nil then
            DeleteStdIO(ReadRequest);
        if WriteRequest <> Nil then
            DeleteStdIO(WriteRequest);
        if ReadPort <> Nil then
            DeletePort(ReadPort);
        if WritePort <> Nil then
            DeletePort(WritePort);
    end;
end;

Function AttachConsole(w : pWindow) : pConsoleSet;
var
    con : pConsoleSet;
    Error : Boolean;
begin
    New(con);
    if con = Nil then
        AttachConsole := Nil;
    with Con^ do begin
        WritePort := CreatePort(Nil, 0);
        Error := WritePort = Nil;
        ReadPort  := CreatePort(Nil, 0);
        Error := Error or (ReadPort = Nil);
        if not Error then begin
            WriteRequest := CreateStdIO(WritePort);
            Error := Error or (WriteRequest = Nil);
            ReadRequest := CreateStdIO(ReadPort);
            Error := Error or (ReadRequest = Nil);
        end;
        if Error then begin
            CleanSet(con);
            Dispose(con);
            AttachConsole := Nil;
        end;
        Window := w;
    end;
    with con^.WriteRequest^ do begin
        io_Data := pointer(w);
        io_Length := SizeOf(tWindow);
    end;
    Error := OpenDevice('console.device', 0,
                        pIORequest(con^.WriteRequest), 0) <> 0;
    if Error then begin
        CleanSet(con);
        Dispose(con);
        AttachConsole := Nil;
    end;
    with con^ do begin
        ReadRequest^.io_Device := WriteRequest^.io_Device;
        ReadRequest^.io_Unit := WriteRequest^.io_Unit;
    end;
    QueueRead(con^.ReadRequest, Addr(con^.Buffer));
    AttachConsole := Con;
end;

Function ReadKey(con : pConsoleSet) : Char;
begin
    with con^ do
        ReadKey := ConGetChar(ReadPort, ReadRequest, Addr(Buffer));
end;

Function KeyPressed(con : pConsoleSet) : Boolean;
begin
    with con^ do
        KeyPressed := CheckIO(pIORequest(ReadRequest)) <> Nil;
end;

Procedure WriteString(con : pConsoleSet; Str : Pchar);
begin
    ConPutStr(con^.WriteRequest, Str);
end;

Procedure WriteString(con : pConsoleSet; Str : string);
var
    temp : string;
begin
    temp := Str;
    temp := temp + #0;
    ConPutStr(con^.WriteRequest, @temp[1]);
end;

Function MaxX(con : pConsoleSet) : smallint;
var
    CU : pConUnit;
begin
    CU := pConUnit(con^.WriteRequest^.io_Unit);
    MaxX := CU^.cu_XMax;
end;

Function MaxY(con : pConsoleSet) : smallint;
var
    CU : pConUnit;
begin
    CU := pConUnit(con^.WriteRequest^.io_Unit);
    MaxY := CU^.cu_YMax;
end;

Function WhereX(con : pConsoleSet) : smallint;
var
    CU : pConUnit;
begin
    CU := pConUnit(con^.WriteRequest^.io_Unit);
    WhereX := CU^.cu_XCP;
end;

Function WhereY(con : pConsoleSet) : smallint;
var
    CU : pConUnit;
begin
    CU := pConUnit(con^.WriteRequest^.io_Unit);
    WhereY := CU^.cu_YCP;
end;

Procedure TextColor(con : pConsoleSet; pen : Byte);
var
    CU : pConUnit;
begin
    CU := pConUnit(con^.WriteRequest^.io_Unit);
    CU^.cu_FgPen := pen;
end;

Procedure TextBackground(con : pConsoleSet; pen : Byte);
var
    CU : pConUnit;
begin
    CU := pConUnit(con^.WriteRequest^.io_Unit);
    CU^.cu_BgPen := pen;
end;

Procedure DetachConsole(con : pConsoleSet);
var
    TempMsg : pMessage;
begin
    with con^ do begin
        Forbid;
        if CheckIO(pIORequest(ReadRequest)) = Nil then begin
            AbortIO(pIORequest(ReadRequest));
            Permit;
            TempMsg := WaitPort(ReadPort);
            TempMsg := GetMsg(ReadPort);
        end else
            Permit;
        CloseDevice(pIORequest(WriteRequest));
    end;
    CleanSet(con);
    Dispose(con);
end;

const
    CSI = #27 + '[';

Procedure ClrEOL(con : pConsoleSet);
{
    Clear to the end of the line
}
begin
    WriteString(con, CSI + 'K');
end;

Procedure ClrScr(con : pConsoleSet);
{
    Clear the text area of the window
}
begin
    WriteString(con, CSI + '1;1H\cJ');
end;

Procedure CursOff(con : pConsoleSet);
{
    Turn the console device's text cursor off
}
begin
    WriteString(con, CSI + '0 p');
end;

Procedure CursOn(con : pConsoleSet);
{
    Turn the text cursor on
}
begin
    WriteString(con, CSI + ' p');
end;


{ Delete the current line, moving all the lines below it  }
{ up one.  The bottom line is cleared.                    }

Procedure DelLine(con : pConsoleSet);
begin
    WriteString(con, CSI + 'M');
end;

Function LongToStr (I : smallint) : String;
Var
    S : String;
begin
    Str (I,S);
    LongToStr:=S;
end;

Procedure GotoXY(con : pConsoleSet; x,y : smallint);
{
    Move the text cursor to the x,y position.  This routine uses
    the ANSI CUP command.
}
var
    XRep : string[7];
    YRep : string[7];
begin
    XRep := LongToStr(x);
    YRep := LongToStr(y);
    WriteString(con,CSI);
    WriteString(con,(YRep));
    WriteString(con,string(';'));
    WriteString(con,(XRep));
    WriteString(con,string('H'));
end;


{  Insert a line at the current text position.  The current line and  }
{  all those below it are moved down one.                             }

Procedure InsLine(con : pConsoleSet);
begin
    WriteString(con, CSI + 'L');
end;



{
        These routines just open and close the Console device without
attaching it to any window.  They update ConsoleBase, and are thus required
for RawKeyConvert and DeadKeyConvert.
}



var

    ConsoleRequest : tIOStdReq;

Procedure OpenConsoleDevice;
{
        This procedure initializes ConsoleDevice, which is required for
    CDInputHandler and RawKeyConvert.
}
var
    Error : longint;
begin
    Error := OpenDevice('console.device', -1, Addr(ConsoleRequest), 0);
    ConsoleDevice := ConsoleRequest.io_Device;
end;

Procedure CloseConsoleDevice;
begin
    CloseDevice(Addr(ConsoleRequest));
end;

end.
