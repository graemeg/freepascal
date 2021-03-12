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

unit timerutils;

{
   History:

   First version of this unit.
   06 Sep 2000.

   Added the define use_amiga_smartlink.
   13 Jan 2003.
   nils.sjoholm@mailbox.swipnet.se

}


interface

uses exec, timer, amigalib;

Function CreateTimer(theUnit : longint) : pTimeRequest;
Function SetTimer(WhichTimer : pTimeRequest;
                        Seconds, Microseconds : longint) : pMsgPort;
Procedure WaitTimer(WhichTimer : pTimeRequest;
                        Seconds, Microseconds : longint);
Procedure DeleteTimer(WhichTimer : pTimeRequest);

implementation

Function CreateTimer(theUnit : longint) : pTimeRequest;
var
    Error : longint;
    TimerPort : pMsgPort;
    TimeReq : pTimeRequest;
begin
    TimerPort := CreatePort(Nil, 0);
    if TimerPort = Nil then
        CreateTimer := Nil;
    TimeReq := pTimeRequest(CreateExtIO(TimerPort,sizeof(tTimeRequest)));
    if TimeReq = Nil then begin
        DeletePort(TimerPort);
        CreateTimer := Nil;
    end;
    Error := OpenDevice(TIMERNAME, theUnit, pIORequest(TimeReq), 0);
    if Error <> 0 then begin
        DeleteExtIO(pIORequest(TimeReq));
        DeletePort(TimerPort);
        CreateTimer := Nil;
    end;
    TimerBase := pointer(TimeReq^.tr_Node.io_Device);
    CreateTimer := pTimeRequest(TimeReq);
end;

Function SetTimer(WhichTimer : pTimeRequest; Seconds, Microseconds : longint) : pMsgPort;
var
    TempPort : pMsgPort;
begin
    with WhichTimer^ do begin
        TempPort := tr_Node.io_Message.mn_ReplyPort;
        tr_Node.io_Command := TR_ADDREQUEST;    { add a new timer request }
        tr_Time.tv_Secs := Seconds;             { seconds }
        tr_Time.tv_Micro := Microseconds;               { microseconds }
        SendIO(pIORequest(WhichTimer));
        SetTimer := TempPort;
    end;
end;

Procedure WaitTimer(WhichTimer : pTimeRequest;
                        Seconds, Microseconds : longint);
var
    Error : Integer;
begin
    with WhichTimer^ do begin
        tr_Node.io_Command := TR_ADDREQUEST;    { add a new timer request }
        tr_Time.tv_Secs := Seconds;             { seconds }
        tr_Time.tv_Micro := Microseconds;               { microseconds }
        Error := DoIO(pIORequest(WhichTimer));
    end;
end;

Procedure DeleteTimer(WhichTimer : pTimeRequest);
var
    WhichPort : pMsgPort;
begin

    WhichPort := WhichTimer^.tr_Node.io_Message.mn_ReplyPort;
    if assigned(WhichTimer) then begin
        CloseDevice(pIORequest(WhichTimer));
        DeleteExtIO(pIORequest(WhichTimer));
    end;
    if assigned(WhichPort) then
        DeletePort(WhichPort);
end;

end.
