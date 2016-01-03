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

{
    History:

    Added DoMethodA, DoSuperMethodA, CoerceMethodA and SetSuperAttrsA.

    I've translated those from amigae. I'm not sure that they are
    correct but it's a start. Now you can try to make some tests
    with mui.
    30 Jul 2000.

    Added stuff for commodities.
    FreeIEvents
    CxCustom
    CxDebug
    CxFilter
    CxSender
    CxSignal
    CxTranslate
    19 Aug 2000.

    Rewrote Createport and DeletePort.
    06 Sep 2000.

    Added two printf, one with pchar and one with string.
    They use array of const so this unit compiles with
    mode objfpc.
    05 Nov 2002.

    Added the define use_amiga_smartlink
    13 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit amigalib;


INTERFACE

uses exec,intuition,utility,commodities,inputevent,amigados;

{*  Exec support functions from amiga.lib  *}

procedure BeginIO (ioRequest: pIORequest);
function CreateExtIO (port: pMsgPort; size: Longint): pIORequest;
procedure DeleteExtIO (ioReq: pIORequest);
function CreateStdIO (port: pMsgPort): pIOStdReq;
procedure DeleteStdIO (ioReq: pIOStdReq);
function CreatePort (name: PChar; pri: longint): pMsgPort;
procedure DeletePort (port: pMsgPort);
function CreateTask (name: STRPTR; pri: longint;
                     initPC : Pointer;
             stackSize : ULONG): pTask;
procedure DeleteTask (task: pTask);
procedure NewList (list: pList);

{* Commodities support functions from amiga.lib *}
procedure FreeIEvents (events: pInputEvent);
function CxCustom
                (action: pointer;
                id: longint): pCxObj;

function CxDebug (id: long): pCxObj;
function CxFilter (d: STRPTR): pCxObj;
function CxSender
                (port: pMsgPort;
                id: longint): pCxObj;

function CxSignal
                (task: pTask;
                sig: byte): pCxObj;

function CxTranslate (ie: pInputEvent): pCxObj;


function DoMethodA(obj : pObject_; msg : APTR): ulong;
function DoSuperMethodA(cl : pIClass; obj : pObject_; msg : APTR): ulong;
function CoerceMethodA(cl : pIClass; obj : pObject_; msg : APTR): ulong;
function SetSuperAttrsA(cl : pIClass; obj: pObject_; msg : APTR): ulong;

{

   NAME
        printf - print a formatted output line to the standard output.

   SYNOPSIS
        printf(formatstring [,value [,values] ] );

   FUNCTION
        Format the output in accordance with specifications in the format
        string.

   INPUTS
        formatString - a C-language-like NULL-terminated format string,
                       with the following supported % options:

          %[flags][width][.limit][length]type

            $     - must follow the arg_pos value, if specified
          flags   - only one allowed. '-' specifies left justification.
          width   - field width. If the first character is a '0', the
                    field is padded with leading 0s.
            .     - must precede the field width value, if specified
          limit   - maximum number of characters to output from a string.
                    (only valid for %s or %b).
          length  - size of input data defaults to word (16-bit) for types c,
                    d, u and x, 'l' changes this to long (32-bit).
          type    - supported types are:
                          b - BSTR, data is 32-bit BPTR to byte count followed
                              by a byte string. A NULL BPTR is treated as an
                              empty string. (V36)
                          d - signed decimal
                          u - unsigned decimal
                          x - hexadecimal with hex digits in uppercase
                          X - hexadecimal with hex digits in lowercase
                          s - string, a 32-bit pointer to a NULL-terminated
                              byte string. A NULL pointer is treated
                              as an empty string.
                          c - character

        value(s) - numeric variables or addresses of null-terminated strings
                   to be added to the format information.

   NOTE
        The global "_stdout" must be defined, and contain a pointer to
        a legal AmigaDOS file handle. Using the standard Amiga startup
        module sets this up. In other cases you will need to define
        stdout, and assign it to some reasonable value (like what the
        dos.library/Output() call returns). This code would set it up:

                ULONG stdout;
                stdout=Output();

   BUGS
        This function will crash if the resulting stream after
        parameter substitution is longer than 140 bytes.

}

procedure printf(Fmtstr : pchar; Args : array of const);
procedure printf(Fmtstr : string; Args : array of const);

IMPLEMENTATION

uses pastoc;

{*  Exec support functions from amiga.lib  *}

procedure BeginIO (ioRequest: pIORequest);
begin
   asm
      move.l  a6,-(a7)
      move.l  ioRequest,a1    ; get IO Request
      move.l  20(a1),a6      ; extract Device ptr
      jsr     -30(a6)        ; call BEGINIO directly
      move.l  (a7)+,a6
   end;
end;

function CreateExtIO (port: pMsgPort; size: Longint): pIORequest;
var
   IOReq: pIORequest;
begin
    IOReq := NIL;
    if port <> NIL then
    begin
        IOReq := AllocMem(size, MEMF_CLEAR or MEMF_PUBLIC);
        if IOReq <> NIL then
        begin
            IOReq^.io_Message.mn_Node.ln_Type   := NT_REPLYMSG;
            IOReq^.io_Message.mn_Length    := size;
            IOReq^.io_Message.mn_ReplyPort := port;
        end;
    end;
    CreateExtIO := IOReq;
end;


procedure DeleteExtIO (ioReq: pIORequest);
begin
    if ioReq <> NIL then
    begin
        ioReq^.io_Message.mn_Node.ln_Type := $FF;
        ioReq^.io_Message.mn_ReplyPort    := pMsgPort(-1);
        ioReq^.io_Device                  := pDevice(-1);
        ExecFreeMem(ioReq, ioReq^.io_Message.mn_Length);
    end
end;


function CreateStdIO (port: pMsgPort): pIOStdReq;
begin
    CreateStdIO := pIOStdReq(CreateExtIO(port, sizeof(tIOStdReq)))
end;


procedure DeleteStdIO (ioReq: pIOStdReq);
begin
    DeleteExtIO(pIORequest(ioReq))
end;


function Createport(name : PChar; pri : longint): pMsgPort;
var
   sigbit : Byte;
   port    : pMsgPort;
begin
   sigbit := AllocSignal(-1);
   if sigbit = -1 then CreatePort := nil;
   port := Allocmem(sizeof(tMsgPort),MEMF_CLEAR or MEMF_PUBLIC);
   if port = nil then begin
      FreeSignal(sigbit);
      CreatePort := nil;
   end;
   with port^ do begin
       if assigned(name) then
       mp_Node.ln_Name := name
       else mp_Node.ln_Name := nil;
       mp_Node.ln_Pri := pri;
       mp_Node.ln_Type := NT_MsgPort;
       mp_Flags := PA_Signal;
       mp_SigBit := sigbit;
       mp_SigTask := FindTask(nil);
   end;
   if assigned(name) then AddPort(port)
   else NewList(addr(port^.mp_MsgList));
   CreatePort := port;
end;

procedure DeletePort (port: pMsgPort);
begin
    if port <> NIL then
    begin
        if port^.mp_Node.ln_Name <> NIL then
            RemPort(port);

        port^.mp_Node.ln_Type     := $FF;
        port^.mp_MsgList.lh_Head  := pNode(-1);
        FreeSignal(port^.mp_SigBit);
        ExecFreeMem(port, sizeof(tMsgPort));
    end;
end;


function CreateTask (name: STRPTR; pri: longint;
        initPC: pointer; stackSize: ULONG): pTask;
var
   memlist : pMemList;
   task    : pTask;
   totalsize : Longint;
begin
    task  := NIL;
    stackSize   := (stackSize + 3) and not 3;
    totalsize := sizeof(tMemList) + sizeof(tTask) + stackSize;

    memlist := AllocMem(totalsize, MEMF_PUBLIC + MEMF_CLEAR);
    if memlist <> NIL then begin
       memlist^.ml_NumEntries := 1;
       memlist^.ml_ME[0].me_Un.meu_Addr := Pointer(memlist + 1);
       memlist^.ml_ME[0].me_Length := totalsize - sizeof(tMemList);

       task := pTask(memlist + sizeof(tMemList) + stackSize);
       task^.tc_Node.ln_Pri := pri;
       task^.tc_Node.ln_Type := NT_TASK;
       task^.tc_Node.ln_Name := name;
       task^.tc_SPLower := Pointer(memlist + sizeof(tMemList));
       task^.tc_SPUpper := Pointer(task^.tc_SPLower + stackSize);
       task^.tc_SPReg := task^.tc_SPUpper;

       NewList(@task^.tc_MemEntry);
       AddTail(@task^.tc_MemEntry,@memlist^.ml_Node);

       AddTask(task,initPC,NIL)
    end;
    CreateTask := task;
end;


procedure DeleteTask (task: pTask);
begin
    RemTask(task)
end;


procedure NewList (list: pList);
begin
    with list^ do
    begin
        lh_Head     := pNode(@lh_Tail);
        lh_Tail     := NIL;
        lh_TailPred := pNode(@lh_Head)
    end
end;

procedure FreeIEvents (events: pInputEvent);
begin
        while events <> NIL do
        begin
                FreeMem (events, sizeof (tInputEvent));
                events := events^.ie_NextEvent
        end
end;

function CxCustom
                (action: pointer;
                id: longint): pCxObj;
begin
        CxCustom := CreateCxObj(CX_CUSTOM, longint(action), id)
end;

function CxDebug (id: long): pCxObj;
begin
        CxDebug := CreateCxObj(CX_DEBUG, id, 0)
end;

function CxFilter (d: STRPTR): pCxObj;
begin
        CxFilter := CreateCxObj(CX_FILTER, longint(d), 0)
end;

function CxSender
                (port: pMsgPort;
                id: longint): pCxObj;
begin
        CxSender := CreateCxObj(CX_SEND, longint(port), id)
end;

function CxSignal
                (task: pTask;
                sig: byte): pCxObj;
begin
        CxSignal:= CreateCxObj(CX_SIGNAL, longint(task), sig)
end;

function CxTranslate (ie: pInputEvent): pCxObj;
begin
        CxTranslate := CreateCxObj(CX_TRANSLATE, longint(ie), 0)
end;

function DoMethodA(obj : pObject_; msg : APTR): ulong;
var
    o : p_Object;
begin
    if assigned(obj) then begin
       o := p_Object(obj);
       DoMethodA := CallHookPkt(@o^.o_Class^.cl_Dispatcher, obj,msg);
    end else DoMethodA := 0;
end;

function DoSuperMethodA(cl : pIClass; obj : pObject_; msg : APTR): ulong;
begin
    if assigned(obj) and assigned(cl) then
       DoSuperMethodA := CallHookPkt(@cl^.cl_Super^.cl_Dispatcher,obj,msg)
    else DoSuperMethodA := 0;
end;

function CoerceMethodA(cl : pIClass; obj : pObject_; msg : APTR): ulong;
begin
    if assigned(cl) and assigned(obj) then
       CoerceMethodA := CallHookPkt(@cl^.cl_Dispatcher,obj,msg)
    else CoerceMethodA := 0;
end;

function SetSuperAttrsA(cl : pIClass; obj: pObject_; msg : APTR): ulong;
var
    arr : array[0..2] of longint;
begin
    arr[0] := OM_SET;
    arr[1] := longint(msg);
    arr[2] := 0;
    SetSuperAttrsA := DoSuperMethodA(cl, obj, @arr);
end;

var
  argarray : array [0..20] of longint;

function gettheconst(args : array of const): pointer;
var
   i : longint;

begin

    for i := 0 to High(args) do begin
        case args[i].vtype of
            vtinteger : argarray[i] := longint(args[i].vinteger);
            vtpchar   : argarray[i] := longint(args[i].vpchar);
            vtchar    : argarray[i] := longint(args[i].vchar);
            vtpointer : argarray[i] := longint(args[i].vpointer);
            vtstring  : argarray[i] := longint(pas2c(args[i].vstring^));
        end;
    end;
    gettheconst := @argarray;
end;

procedure printf(Fmtstr : pchar; Args : array of const);
begin
    VPrintf(Fmtstr,gettheconst(Args));
end;

procedure printf(Fmtstr : string; Args : array of const);
begin
    VPrintf(pas2c(Fmtstr) ,gettheconst(Args));
end;


end.
