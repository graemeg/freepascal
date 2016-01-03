{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2000 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  History:

  First translation of AMarquee.h to FPC pascal.
  No testing has been done so there could be bugs
  in here. If you find any bugs please let me know.
  25 Aug 2000.

  Added functions and procedures with array of const.
  For use with fpc 1.0.7
  30 Nov 2002.

  Added the defines use_amiga_smartlink and
  use_auto_openlib.
  12 Jan 2003.

  Changed startcode for unit.
  10 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se

}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}
unit amarquee;


interface


uses exec, utility;



  { This library contains definitions and structures necessary to use amarquee.library.  }
  { Different error types that can be returned to the client in QMessages  }
  { Everything is okay  }

  const
     AMARQUEENAME : PChar = 'amarquee.library';

     QERROR_NO_ERROR = 0;
  { Don't know what the error was.  }
     QERROR_UNKNOWN = -(1);
  { Keystring could not be parsed or "dir" does not exist  }
     QERROR_MALFORMED_KEY = -(2);
  { Server did not have enough mem available to complete request  }
     QERROR_NO_SERVER_MEM = -(3);
  { The connection to the server went south  }
     QERROR_NO_CONNECTION = -(4);
  { You're not allowed to do that!  }
     QERROR_UNPRIVILEGED = -(5);
  { Feature you requested does not exist yet  }
     QERROR_UNIMPLEMENTED = -(6);
  { Your computer didn't have enough memory available to complete an action  }
     QERROR_NO_CLIENT_MEM = -(7);
  { This message is a "wall" text message from a server admin  }
     QERROR_SYS_MESSAGE = -(8);
  { This message is a "send done" notification from your client TCP thread (new for v47)  }
     QERROR_SEND_DONE = -(9);
  { These error types may be returned by QFreeSession() or set to the error code
  variable (set with the QSESSION_ERRORCODEPTR tag), after a QSession
  allocation attempt fails.  }
  { TCP stack wasn't running  }
     QERROR_NO_TCP_STACK = -(20);
  { Hostname lookup of server failed  }
     QERROR_HOSTNAME_LOOKUP = -(21);
  { TCP thread got a control-C  }
     QERROR_ABORTED = -(22);
  { No AMarquee server at requested host/port  }
     QERROR_NO_SERVER = -(23);
  { The program wasn't launched by inetd  }
     QERROR_NO_INETD = -(24);
  { The server wouldn't accept our connection  }
     QERROR_ACCESS_DENIED = -(25);
  { The client is out of memory  }
     QERROR_NO_MEMORY = -(26);
  { No server at requested host/port  }
     QERROR_NO_TCPSERVER = -(27);
  { Flags to use with QGo()  }
  { Request notification via sync packet when all has settled on the server side  }
     QGOF_SYNC = 1 shl 0;
  { Request notification via QERROR_SEND_DONE message when the TCP thread has finished sending this transaction (new for v47)  }
     QGOF_NOTIFY = 1 shl 1;
  { Flags representing special privileges; used with QRequestPrivileges(), etc  }
  { license to kill!  }
     QPRIV_KILLCLIENTS = 1 shl 0;
  { Ability to set env vars, etc, on the server  }
     QPRIV_ADMIN = 1 shl 1;
  { Ability to receive special sysadmin messages  }
     QPRIV_GETSYSMESSAGES = 1 shl 2;
  { Ability to send special sysadmin messages  }
     QPRIV_SENDSYSMESSAGES = 1 shl 3;
  { all possible QPRIV_  bits  }
     QPRIV_ALL = $0F;
  { Wait() on this for QMessages!  }
  {                                                                   }
  { Invisible, private info is here... don't trust sizeof(QSession)!  }
  {                                                                   }

  type
     PQSession = ^TQSession;
     TQSession = record
          qMsgPort : PMsgPort;
       end;

  { All events from AMarquee come in this package!  }
  { Don't directly use the contents of qm_Msg!  }
  { Message ID # of transaction related to this opCode, or -1 if no ID is applicable.  }
  { One of the QERROR_  codes defined in AMarquee headers.  }
  { Pathname of a node, or NULL if not applicable.  }
  { Pointer to beginning of data buffer, or NULL if not applicable.  }
  { Length of qm_Data buffer, or 0 if not applicable.  }
  { Length of the data buffer stored on the AMarquee server.  }
  { Line # of the server source code that generated the error.  Useful for debugging.  }
  { Private info used by FreeQMessage()  }
  { Added for v48; more private info used by FreeQMessage()  }
  { Added for v50. The session this message comes from. Necessary with shared msgports   }
     PQMessage = ^TQMessage;
     TQMessage = record
          qm_Msg : tMessage;
          qm_ID : LONG;
          qm_Status : longint;
          qm_Path : Pchar;
          qm_Data : pointer;
          qm_DataLen : ULONG;
          qm_ActualLen : ULONG;
          qm_ErrorLine : ULONG;
          qm_Private : pointer;
          qm_Private2 : pointer;
          qm_Session : PQSession;
       end;

  { The theoretical maximum number of bytes your server may allocate.  }
  { The number of bytes currently allocated by your server.  }
  { The number of bytes that may actually be allocated by your server.  }
  { Bit-chord of QPRIV_  bits, showing what special privs you have.  }
  { Bit-chord of QPRIV_  bits, showing what special privs you could get if you asked.  }
     PQRunInfo = ^TQRunInfo;
     TQRunInfo = record
          qr_Allowed : LONG;
          qr_Alloced : LONG;
          qr_Avail : LONG;
          qr_CurrentPrivs : ULONG;
          qr_PossiblePrivs : ULONG;
       end;

  { Used internally  }
  { the message port to listen on  }
     PQSharedMessagePort = ^TQSharedMessagePort;
     TQSharedMessagePort = record
          qs_private : tSignalSemaphore;
          qs_mp : PMsgPort;
       end;

  { the QRAWSESSION_PROTOCOLSTOPHOOK Hook function receive a pointer to the Hook in A0, the QSession
  pointer in A2 and a pointer to a QProtocolStopMarkerMessage in A1. Return the length of the buffer
  to send to the client. Return a length of 0 if not enough data is present. With this function
  it's easy to make sure that no structs or other datatypes get stripped.  }
     PQProtocolStopMarkerMessage = ^TQProtocolStopMarkerMessage;
     TQProtocolStopMarkerMessage = record
          buffer : STRPTR;
          length : ULONG;
          userdata : ULONG;
          id : LONG;
       end;

  { Tags to use with QNewSocketSession(Async)  }
  { (ULONG maxBufSize) The maximum buffer size to use (same as QSetMaxRawBufSize())  }

  const
     QRAWSESSION_MAXBUFFERSIZE = $a0000001;
  { (struct Hook  stopmarkerfunc) A hook with the function which sets the stop marker for a block of data  }
     QRAWSESSION_PROTOCOLSTOPHOOK = $a0000002;
  { (BOOL b) default TRUE. Shall the last data before the connection is broken be sent to the client even though the PROTOCOLSTOPHOOK hook specify it shouldn't ? }
     QRAWSESSION_RECEIVE_EXCEEDING_DATA = $a0000003;
  { (ULONG u) The initial value of the userdata field in struct QProtocolStopMarkerMessage  }
     QRAWSESSION_PROTOCOLSTOPHOOK_USERDATA = $a0000004;
  { Tags to use with QNew Session(Async) and QNewSocket Session(Async)  }
  { (LONG  errorCode) A pointer to the variable that will hold the error code  }
     QSESSION_ERRORCODEPTR = $b0000001;
  { (struct QSharedMessagePort  mp) A shared message port created with QCreateSharedMessagePort()  }
     QSESSION_SHAREDMSGPORT = $b0000002;

VAR AMarqueeBase : pLibrary;

FUNCTION QFreeSession(session : pQSession) : LONGINT;
FUNCTION QDebugOp(session : pQSession; string_ : pCHar) : LONGINT;
FUNCTION QGetOp(session : pQSession; path : pCHar; maxBytes : LONGINT) : LONGINT;
FUNCTION QDeleteOp(session : pQSession; path : pCHar) : LONGINT;
FUNCTION QRenameOp(session : pQSession; path : pCHar; label_ : pCHar) : LONGINT;
FUNCTION QSubscribeOp(session : pQSession; path : pCHar; maxBytes : LONGINT) : LONGINT;
FUNCTION QSetOp(session : pQSession; path : pCHar; buf : POINTER; len : ULONG) : LONGINT;
FUNCTION QClearSubscriptionsOp(session : pQSession; which : LONGINT) : LONGINT;
FUNCTION QPingOp(session : pQSession) : LONGINT;
FUNCTION QInfoOp(session : pQSession) : LONGINT;
FUNCTION QSetAccessOp(session : pQSession; hosts : pCHar) : LONGINT;
PROCEDURE FreeQMessage(session : pQSession; qmsg : pQMessage);
FUNCTION QGo(session : pQSession; sync : ULONG) : LONGINT;
FUNCTION QStreamOp(session : pQSession; path : pCHar; buf : POINTER; len : ULONG) : LONGINT;
FUNCTION QSetMessageAccessOp(session : pQSession; access : pCHar; maxbytes : LONGINT) : LONGINT;
FUNCTION QMessageOp(session : pQSession; hosts : pCHar; buffer : POINTER; len : ULONG) : LONGINT;
FUNCTION QNumQueuedPackets(session : pQSession) : ULONG;
FUNCTION QNumQueuedBytes(session : pQSession) : ULONG;
FUNCTION QErrorName(session : LONGINT) : pCHar;
FUNCTION QRequestPrivilegesOp(session : pQSession; privBits : ULONG) : LONGINT;
FUNCTION QReleasePrivilegesOp(session : pQSession; privBits : ULONG) : LONGINT;
FUNCTION QKillClientsOp(session : pQSession; hosts : pCHar) : LONGINT;
FUNCTION QSetParameterOp(session : pQSession; paramName : pCHar; newValue : pCHar) : LONGINT;
FUNCTION QGetParameterOp(session : pQSession; paramName : pCHar) : LONGINT;
FUNCTION QSysMessageOp(session : pQSession; hosts : pCHar; message : pCHar) : LONGINT;
FUNCTION QGetAndSubscribeOp(session : pQSession; path : pCHar; maxBytes : LONGINT) : LONGINT;
FUNCTION QDetachSession(session : pQSession; flags : ULONG) : BOOLEAN;
FUNCTION QReattachSession(session : pQSession; flags : ULONG) : BOOLEAN;
FUNCTION QNewSocketSession(host : pCHar; port : LONGINT; tags : pTagItem) : pQSession;
FUNCTION QSendRawOp(session : pQSession; buf : POINTER; len : ULONG) : LONGINT;
FUNCTION QNewSocketSessionAsync(host : pCHar; port : LONGINT; tags : pTagItem) : pQSession;
FUNCTION QNewSocketServerSession( port : pLONGINT; tags : pTagItem) : pQSession;
FUNCTION QSetKeyAccessOp(session : pQSession; path : pCHar; hosts : pCHar) : LONGINT;
FUNCTION QGetHostName(session : pQSession) : pCHar;
FUNCTION QGetProgName(session : pQSession) : pCHar;
PROCEDURE QSetMaxRawBufSize(session : pQSession; maxBufSize : ULONG);
FUNCTION QNewSession(host : pCHar; port : LONGINT; name : pCHar; taglist : pTagItem) : pQSession;
FUNCTION QNewSessionAsync(host : pCHar; port : LONGINT; name : pCHar; taglist : pTagItem) : pQSession;
FUNCTION QNewHostSession(hostnames : pCHar; port : pLONGINT; names : pCHar; taglist : pTagItem) : pQSession;
FUNCTION QNewServerSession(hostNames : pCHar; progNames : pCHar; taglist : pTagItem) : pQSession;
FUNCTION QCreateSharedMessagePort : pQSharedMessagePort;
PROCEDURE QDeleteSharedMessagePort(mp : pQSharedMessagePort);
FUNCTION QGetLocalIP(session : pQSession) : pCHAR;

{
     This is functions and procedures with array of const.
     For use with fpc 1.0 and above.

}
FUNCTION QNewSocketSessiontags(host : pCHar; port : LONGINT; const argv : Array Of Const) : pQSession;
FUNCTION QNewSocketSessionAsyncTags(host : pCHar; port : LONGINT; const argv : Array Of Const) : pQSession;
FUNCTION QNewSocketServerSessionTags( port : pLONGINT; const argv : Array Of Const) : pQSession;
FUNCTION QNewSessionTags(host : pCHar; port : LONGINT; name : pCHar; const argv : Array Of Const) : pQSession;
FUNCTION QNewSessionAsyncTags(host : pCHar; port : LONGINT; name : pCHar; const argv : Array Of Const) : pQSession;
FUNCTION QNewHostSessionTags(hostnames : pCHar; port : pLONGINT; names : pCHar; const argv : Array Of Const) : pQSession;
FUNCTION QNewServerSessionTags(hostNames : pCHar; progNames : pCHar; const argv : Array Of Const) : pQSession;


FUNCTION QDebugOp(session : pQSession; string_ : string) : LONGINT;
FUNCTION QGetOp(session : pQSession; path : string; maxBytes : LONGINT) : LONGINT;
FUNCTION QDeleteOp(session : pQSession; path : string) : LONGINT;
FUNCTION QRenameOp(session : pQSession; path : string; label_ : string) : LONGINT;
FUNCTION QSubscribeOp(session : pQSession; path : string; maxBytes : LONGINT) : LONGINT;
FUNCTION QSetOp(session : pQSession; path : string; buf : POINTER; len : ULONG) : LONGINT;
FUNCTION QSetAccessOp(session : pQSession; hosts : string) : LONGINT;
FUNCTION QStreamOp(session : pQSession; path : string; buf : POINTER; len : ULONG) : LONGINT;
FUNCTION QSetMessageAccessOp(session : pQSession; access : string; maxbytes : LONGINT) : LONGINT;
FUNCTION QMessageOp(session : pQSession; hosts : string; buffer : POINTER; len : ULONG) : LONGINT;
FUNCTION QKillClientsOp(session : pQSession; hosts : string) : LONGINT;
FUNCTION QSetParameterOp(session : pQSession; paramName : string; newValue : string) : LONGINT;
FUNCTION QGetParameterOp(session : pQSession; paramName : string) : LONGINT;
FUNCTION QSysMessageOp(session : pQSession; hosts : string; message : string) : LONGINT;
FUNCTION QGetAndSubscribeOp(session : pQSession; path : string; maxBytes : LONGINT) : LONGINT;
FUNCTION QNewSocketSession(host : string; port : LONGINT; tags : pTagItem) : pQSession;
FUNCTION QNewSocketSessionAsync(host : string; port : LONGINT; tags : pTagItem) : pQSession;
FUNCTION QSetKeyAccessOp(session : pQSession; path : string; hosts : string) : LONGINT;
FUNCTION QNewSession(host : string; port : LONGINT; name : string; taglist : pTagItem) : pQSession;
FUNCTION QNewSessionAsync(host : string; port : LONGINT; name : string; taglist : pTagItem) : pQSession;
FUNCTION QNewHostSession(hostnames : string; port : pLONGINT; names : string; taglist : pTagItem) : pQSession;
FUNCTION QNewServerSession(hostNames : string; progNames : string; taglist : pTagItem) : pQSession;

{
     This is functions and procedures with array of const.
     For use with fpc 1.0 and above.
}

FUNCTION QNewSocketSessionTags(host : string; port : LONGINT; const argv : Array Of Const) : pQSession;
FUNCTION QNewSocketSessionAsyncTags(host : string; port : LONGINT; const argv : Array Of Const) : pQSession;
FUNCTION QNewSessionTags(host : string; port : LONGINT; name : string; const argv : Array Of Const) : pQSession;
FUNCTION QNewSessionAsyncTags(host : string; port : LONGINT; name : string; const argv : Array Of Const) : pQSession;
FUNCTION QNewHostSessionTags(hostnames : string; port : pLONGINT; names : string; const argv : Array Of Const) : pQSession;
FUNCTION QNewServerSessionTags(hostNames : string; progNames : string; const argv : Array Of Const) : pQSession;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitAMARQUEELibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    AMARQUEEIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
pastoc,tagsarray;


FUNCTION QFreeSession(session : pQSession) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QDebugOp(session : pQSession; string_ : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L string_,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QGetOp(session : pQSession; path : pCHar; maxBytes : LONGINT) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L path,A1
        MOVE.L  maxBytes,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QDeleteOp(session : pQSession; path : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L path,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QRenameOp(session : pQSession; path : pCHar; label_ : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L path,A1
        MOVE.L  label_,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QSubscribeOp(session : pQSession; path : pCHar; maxBytes : LONGINT) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L path,A1
        MOVE.L  maxBytes,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QSetOp(session : pQSession; path : pCHar; buf : POINTER; len : ULONG) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L path,A1
        MOVE.L  buf,D0
        MOVE.L  len,D1
        MOVEA.L AMarqueeBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QClearSubscriptionsOp(session : pQSession; which : LONGINT) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVE.L  which,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QPingOp(session : pQSession) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QInfoOp(session : pQSession) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QSetAccessOp(session : pQSession; hosts : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L hosts,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE FreeQMessage(session : pQSession; qmsg : pQMessage);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L qmsg,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION QGo(session : pQSession; sync : ULONG) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVE.L  sync,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QStreamOp(session : pQSession; path : pCHar; buf : POINTER; len : ULONG) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L path,A1
        MOVE.L  buf,D0
        MOVE.L  len,D1
        MOVEA.L AMarqueeBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QSetMessageAccessOp(session : pQSession; access : pCHar; maxbytes : LONGINT) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L access,A1
        MOVE.L  maxbytes,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QMessageOp(session : pQSession; hosts : pCHar; buffer : POINTER; len : ULONG) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L hosts,A1
        MOVE.L  buffer,D0
        MOVE.L  len,D1
        MOVEA.L AMarqueeBase,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QNumQueuedPackets(session : pQSession) : ULONG;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QNumQueuedBytes(session : pQSession) : ULONG;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QErrorName(session : LONGINT) : pCHar;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  session,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QRequestPrivilegesOp(session : pQSession; privBits : ULONG) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVE.L  privBits,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QReleasePrivilegesOp(session : pQSession; privBits : ULONG) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVE.L  privBits,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -174(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QKillClientsOp(session : pQSession; hosts : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L hosts,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -180(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QSetParameterOp(session : pQSession; paramName : pCHar; newValue : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L paramName,A1
        MOVE.L  newValue,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -186(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QGetParameterOp(session : pQSession; paramName : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L paramName,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -192(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QSysMessageOp(session : pQSession; hosts : pCHar; message : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L hosts,A1
        MOVE.L  message,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -198(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QGetAndSubscribeOp(session : pQSession; path : pCHar; maxBytes : LONGINT) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L path,A1
        MOVE.L  maxBytes,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -210(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QDetachSession(session : pQSession; flags : ULONG) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVE.L  flags,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -216(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION QReattachSession(session : pQSession; flags : ULONG) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVE.L  flags,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -222(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION QNewSocketSession(host : pCHar; port : LONGINT; tags : pTagItem) : pQSession;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L host,A0
        MOVE.L  port,D0
        MOVEA.L tags,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -228(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QSendRawOp(session : pQSession; buf : POINTER; len : ULONG) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L buf,A1
        MOVE.L  len,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -234(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QNewSocketSessionAsync(host : pCHar; port : LONGINT; tags : pTagItem) : pQSession;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L host,A0
        MOVE.L  port,D0
        MOVEA.L tags,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -240(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QNewSocketServerSession(port : pLONGINT; tags : pTagItem) : pQSession;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L port,A0
        MOVEA.L tags,A1
        MOVEA.L AMarqueeBase,A6
        JSR     -246(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QSetKeyAccessOp(session : pQSession; path : pCHar; hosts : pCHar) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L path,A1
        MOVE.L  hosts,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -252(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QGetHostName(session : pQSession) : pCHar;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -258(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QGetProgName(session : pQSession) : pCHar;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -264(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE QSetMaxRawBufSize(session : pQSession; maxBufSize : ULONG);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVE.L  maxBufSize,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -270(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION QNewSession(host : pCHar; port : LONGINT; name : pCHar; taglist : pTagItem) : pQSession;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L host,A0
        MOVE.L  port,D0
        MOVEA.L name,A1
        MOVE.L  taglist,D1
        MOVEA.L AMarqueeBase,A6
        JSR     -276(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QNewSessionAsync(host : pCHar; port : LONGINT; name : pCHar; taglist : pTagItem) : pQSession;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L host,A0
        MOVE.L  port,D0
        MOVEA.L name,A1
        MOVE.L  taglist,D1
        MOVEA.L AMarqueeBase,A6
        JSR     -282(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QNewHostSession(hostnames : pCHar; port : pLONGINT; names : pCHar; taglist : pTagItem) : pQSession;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L hostnames,A0
        MOVEA.L port,A1
        MOVE.L  names,D0
        MOVE.L  taglist,D1
        MOVEA.L AMarqueeBase,A6
        JSR     -288(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QNewServerSession(hostNames : pCHar; progNames : pCHar; taglist : pTagItem) : pQSession;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L hostNames,A0
        MOVEA.L progNames,A1
        MOVE.L  taglist,D0
        MOVEA.L AMarqueeBase,A6
        JSR     -294(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION QCreateSharedMessagePort : pQSharedMessagePort;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L AMarqueeBase,A6
        JSR     -300(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE QDeleteSharedMessagePort(mp : pQSharedMessagePort);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mp,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -306(A6)
        MOVEA.L (A7)+,A6
  END;
END;


FUNCTION QGetLocalIP(session : pQSession) : pCHAR;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L session,A0
        MOVEA.L AMarqueeBase,A6
        JSR     -312(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;


FUNCTION QDebugOp(session : pQSession; string_ : string) : LONGINT;
begin
    QDebugOp := QDebugOp(session,pas2c(string_));
end;

FUNCTION QGetOp(session : pQSession; path : string; maxBytes : LONGINT) : LONGINT;
begin
    QGetOp := QGetOp(session,pas2c(path),maxBytes);
end;

FUNCTION QDeleteOp(session : pQSession; path : string) : LONGINT;
begin
    QDeleteOp := QDeleteOp(session,pas2c(path));
end;

FUNCTION QRenameOp(session : pQSession; path : string; label_ : string) : LONGINT;
begin
    QRenameOp := QRenameOp(session,pas2c(path),pas2c(label_));
end;

FUNCTION QSubscribeOp(session : pQSession; path : string; maxBytes : LONGINT) : LONGINT;
begin
    QSubscribeOp := QSubscribeOp(session,pas2c(path),maxBytes);
end;

FUNCTION QSetOp(session : pQSession; path : string; buf : POINTER; len : ULONG) : LONGINT;
begin
    QSetOp := QSetOp(session,pas2c(path),buf,len);
end;

FUNCTION QSetAccessOp(session : pQSession; hosts : string) : LONGINT;
begin
    QSetAccessOp := QSetAccessOp(session,pas2c(hosts));
end;

FUNCTION QStreamOp(session : pQSession; path : string; buf : POINTER; len : ULONG) : LONGINT;
begin
    QStreamOp := QStreamOp(session,pas2c(path),buf,len);
end;

FUNCTION QSetMessageAccessOp(session : pQSession; access : string; maxbytes : LONGINT) : LONGINT;
begin
    QSetMessageAccessOp := QSetMessageAccessOp(session,pas2c(access),maxBytes);
end;

FUNCTION QMessageOp(session : pQSession; hosts : string; buffer : POINTER; len : ULONG) : LONGINT;
begin
    QMessageOp := QMessageOp(session,pas2c(hosts),buffer,len);
end;

FUNCTION QKillClientsOp(session : pQSession; hosts : string) : LONGINT;
begin
    QKillClientsOp := QKillClientsOp(session,pas2c(hosts));
end;

FUNCTION QSetParameterOp(session : pQSession; paramName : string; newValue : string) : LONGINT;
begin
    QSetParameterOp := QSetParameterOp(session,pas2c(paramName),pas2c(newValue));
end;

FUNCTION QGetParameterOp(session : pQSession; paramName : string) : LONGINT;
begin
    QGetParameterOp := QGetParameterOp(session,pas2c(paramName));
end;

FUNCTION QSysMessageOp(session : pQSession; hosts : string; message : string) : LONGINT;
begin
    QSysMessageOp := QSysMessageOp(session,pas2c(hosts),pas2c(message));
end;

FUNCTION QGetAndSubscribeOp(session : pQSession; path : string; maxBytes : LONGINT) : LONGINT;
begin
    QGetAndSubscribeOp := QGetAndSubscribeOp(session,pas2c(path),maxBytes);
end;

FUNCTION QNewSocketSession(host : string; port : LONGINT; tags : pTagItem) : pQSession;
begin
    QNewSocketSession := QNewSocketSession(pas2c(host),port,tags);
end;

FUNCTION QNewSocketSessionAsync(host : string; port : LONGINT; tags : pTagItem) : pQSession;
begin
    QNewSocketSessionAsync := QNewSocketSessionAsync(pas2c(host),port,tags);
end;

FUNCTION QSetKeyAccessOp(session : pQSession; path : string; hosts : string) : LONGINT;
begin
    QSetKeyAccessOp := QSetKeyAccessOp(session,pas2c(path),pas2c(hosts));
end;

FUNCTION QNewSession(host : string; port : LONGINT; name : string; taglist : pTagItem) : pQSession;
begin
    QNewSession := QNewSession(pas2c(host),port,pas2c(name),taglist);
end;

FUNCTION QNewSessionAsync(host : string; port : LONGINT; name : string; taglist : pTagItem) : pQSession;
begin
    QNewSessionAsync := QNewSessionAsync(pas2c(host),port,pas2c(name),taglist);
end;

FUNCTION QNewHostSession(hostnames : string; port : pLONGINT; names : string; taglist : pTagItem) : pQSession;
begin
    QNewHostSession := QNewHostSession(pas2c(hostnames),port,pas2c(names),taglist);
end;

FUNCTION QNewServerSession(hostNames : string; progNames : string; taglist : pTagItem) : pQSession;
begin
    QNewServerSession := QNewServerSession(pas2c(hostnames),pas2c(prognames),taglist);
end;

FUNCTION QNewSocketSessiontags(host : pCHar; port : LONGINT; const argv : Array Of Const) : pQSession;
begin
    QNewSocketSessiontags := QNewSocketSession(host,port,readintags(argv));
end;

FUNCTION QNewSocketSessionAsyncTags(host : pCHar; port : LONGINT; const argv : Array Of Const) : pQSession;
begin
    QNewSocketSessionAsyncTags := QNewSocketSessionAsync(host,port,readintags(argv));
end;

FUNCTION QNewSocketServerSessionTags( port : pLONGINT; const argv : Array Of Const) : pQSession;
begin
    QNewSocketServerSessionTags := QNewSocketServerSession(port,readintags(argv));
end;

FUNCTION QNewSessionTags(host : pCHar; port : LONGINT; name : pCHar; const argv : Array Of Const) : pQSession;
begin
    QNewSessionTags := QNewSession(host,port,name,readintags(argv));
end;

FUNCTION QNewSessionAsyncTags(host : pCHar; port : LONGINT; name : pCHar; const argv : Array Of Const) : pQSession;
begin
    QNewSessionAsyncTags := QNewSessionAsync(host,port,name,readintags(argv));
end;

FUNCTION QNewHostSessionTags(hostnames : pCHar; port : pLONGINT; names : pCHar; const argv : Array Of Const) : pQSession;
begin
    QNewHostSessionTags := QNewHostSession(hostnames,port,names,readintags(argv));
end;

FUNCTION QNewServerSessionTags(hostNames : pCHar; progNames : pCHar; const argv : Array Of Const) : pQSession;
begin
    QNewServerSessionTags := QNewServerSession(hostnames,prognames,readintags(argv));
end;


FUNCTION QNewSocketSessionTags(host : string; port : LONGINT; const argv : Array Of Const) : pQSession;
begin
    QNewSocketSessionTags := QNewSocketSession(host,port,readintags(argv));
end;

FUNCTION QNewSocketSessionAsyncTags(host : string; port : LONGINT; const argv : Array Of Const) : pQSession;
begin
    QNewSocketSessionAsyncTags := QNewSocketSessionAsync(host,port,readintags(argv));
end;

FUNCTION QNewSessionTags(host : string; port : LONGINT; name : string; const argv : Array Of Const) : pQSession;
begin
    QNewSessionTags := QNewSession(host,port,name,readintags(argv));
end;

FUNCTION QNewSessionAsyncTags(host : string; port : LONGINT; name : string; const argv : Array Of Const) : pQSession;
begin
    QNewSessionAsyncTags := QNewSessionAsync(host,port,name,readintags(argv));
end;

FUNCTION QNewHostSessionTags(hostnames : string; port : pLONGINT; names : string; const argv : Array Of Const) : pQSession;
begin
    QNewHostSessionTags := QNewHostSession(hostnames,port,names,readintags(argv));
end;

FUNCTION QNewServerSessionTags(hostNames : string; progNames : string; const argv : Array Of Const) : pQSession;
begin
    QNewServerSessionTags := QNewServerSession(hostnames,prognames,readintags(argv));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of amarquee.library}
  {$Info don't forget to use InitAMARQUEELibrary in the beginning of your program}

var
    amarquee_exit : Pointer;

procedure CloseamarqueeLibrary;
begin
    ExitProc := amarquee_exit;
    if AMarqueeBase <> nil then begin
        CloseLibrary(AMarqueeBase);
        AMarqueeBase := nil;
    end;
end;

procedure InitAMARQUEELibrary;
begin
    AMarqueeBase := nil;
    AMarqueeBase := OpenLibrary(AMARQUEENAME,LIBVERSION);
    if AMarqueeBase <> nil then begin
        amarquee_exit := ExitProc;
        ExitProc := @CloseamarqueeLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open amarquee.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    AMARQUEEIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of amarquee.library}

var
    amarquee_exit : Pointer;

procedure CloseamarqueeLibrary;
begin
    ExitProc := amarquee_exit;
    if AMarqueeBase <> nil then begin
        CloseLibrary(AMarqueeBase);
        AMarqueeBase := nil;
    end;
end;

begin
    AMarqueeBase := nil;
    AMarqueeBase := OpenLibrary(AMARQUEENAME,LIBVERSION);
    if AMarqueeBase <> nil then begin
        amarquee_exit := ExitProc;
        ExitProc := @CloseamarqueeLibrary;
        AMARQUEEIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open amarquee.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    AMARQUEEIsCompiledHow := 3;
   {$Warning No autoopening of amarquee.library compiled}
   {$Warning Make sure you open amarquee.library yourself}
{$endif dont_use_openlib}

END. (* UNIT AMARQUEE *)



