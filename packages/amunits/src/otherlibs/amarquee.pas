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

  Added functions and procedures with array of PtrUInt.
  For use with fpc 1.0.7
  30 Nov 2002.

  Added the defines use_amiga_smartlink and
  use_auto_openlib.
  12 Jan 2003.

  Changed startcode for unit.
  10 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se

}

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

VAR AMarqueeBase : pLibrary = nil;

FUNCTION QFreeSession(session : pQSession location 'a0') : LONGINT; syscall AMarqueeBase 36;
FUNCTION QDebugOp(session : pQSession location 'a0'; string_ : pCHar location 'a1') : LONGINT; syscall AMarqueeBase 42;
FUNCTION QGetOp(session : pQSession location 'a0'; path : pCHar location 'a1'; maxBytes : LONGINT location 'd0') : LONGINT; syscall AMarqueeBase 48;
FUNCTION QDeleteOp(session : pQSession location 'a0'; path : pCHar location 'a1') : LONGINT; syscall AMarqueeBase 54;
FUNCTION QRenameOp(session : pQSession location 'a0'; path : pCHar location 'a1'; label_ : pCHar location 'd0') : LONGINT; syscall AMarqueeBase 60;
FUNCTION QSubscribeOp(session : pQSession location 'a0'; path : pCHar location 'a1'; maxBytes : LONGINT location 'd0') : LONGINT; syscall AMarqueeBase 66;
FUNCTION QSetOp(session : pQSession location 'a0'; path : pCHar location 'a1'; buf : POINTER location 'd0'; len : ULONG location 'd1') : LONGINT; syscall AMarqueeBase 72;
FUNCTION QClearSubscriptionsOp(session : pQSession location 'a0'; which : LONGINT location 'd0') : LONGINT; syscall AMarqueeBase 78;
FUNCTION QPingOp(session : pQSession location 'a0') : LONGINT; syscall AMarqueeBase 84;
FUNCTION QInfoOp(session : pQSession location 'a0') : LONGINT; syscall AMarqueeBase 90;
FUNCTION QSetAccessOp(session : pQSession location 'a0'; hosts : pCHar location 'a1') : LONGINT; syscall AMarqueeBase 96;
PROCEDURE FreeQMessage(session : pQSession location 'a0'; qmsg : pQMessage location 'a1'); syscall AMarqueeBase 102;
FUNCTION QGo(session : pQSession location 'a0'; sync : ULONG location 'd0') : LONGINT; syscall AMarqueeBase 108;
FUNCTION QStreamOp(session : pQSession location 'a0'; path : pCHar location 'a1'; buf : POINTER location 'd0'; len : ULONG location 'd1') : LONGINT; syscall AMarqueeBase 120;
FUNCTION QSetMessageAccessOp(session : pQSession location 'a0'; access : pCHar location 'a1'; maxbytes : LONGINT location 'd0') : LONGINT; syscall AMarqueeBase 132;
FUNCTION QMessageOp(session : pQSession location 'a0'; hosts : pCHar location 'a1'; buffer : POINTER location 'd0'; len : ULONG location 'd1') : LONGINT; syscall AMarqueeBase 138;
FUNCTION QNumQueuedPackets(session : pQSession location 'a0') : ULONG; syscall AMarqueeBase 150;
FUNCTION QNumQueuedBytes(session : pQSession location 'a0') : ULONG; syscall AMarqueeBase 156;
FUNCTION QErrorName(session : LONGINT location 'd0') : pCHar; syscall AMarqueeBase 162;
FUNCTION QRequestPrivilegesOp(session : pQSession location 'a0'; privBits : ULONG location 'd0') : LONGINT; syscall AMarqueeBase 168;
FUNCTION QReleasePrivilegesOp(session : pQSession location 'a0'; privBits : ULONG location 'd0') : LONGINT; syscall AMarqueeBase 174;
FUNCTION QKillClientsOp(session : pQSession location 'a0'; hosts : pCHar location 'a1') : LONGINT; syscall AMarqueeBase 180;
FUNCTION QSetParameterOp(session : pQSession location 'a0'; paramName : pCHar location 'a1'; newValue : pCHar location 'd0') : LONGINT; syscall AMarqueeBase 186;
FUNCTION QGetParameterOp(session : pQSession location 'a0'; paramName : pCHar location 'a1') : LONGINT; syscall AMarqueeBase 192;
FUNCTION QSysMessageOp(session : pQSession location 'a0'; hosts : pCHar location 'a1'; message : pCHar location 'd0') : LONGINT; syscall AMarqueeBase 198;
FUNCTION QGetAndSubscribeOp(session : pQSession location 'a0'; path : pCHar location 'a1'; maxBytes : LONGINT location 'd0') : LONGINT; syscall AMarqueeBase 210;
FUNCTION QDetachSession(session : pQSession location 'a0'; flags : ULONG location 'd0') : BOOLEAN; syscall AMarqueeBase 216;
FUNCTION QReattachSession(session : pQSession location 'a0'; flags : ULONG location 'd0') : BOOLEAN; syscall AMarqueeBase 222;
FUNCTION QNewSocketSession(host : pCHar location 'a0'; port : LONGINT location 'd0'; tags : pTagItem location 'a1') : pQSession; syscall AMarqueeBase 228;
FUNCTION QSendRawOp(session : pQSession location 'a0'; buf : POINTER location 'a1'; len : ULONG location 'd0') : LONGINT; syscall AMarqueeBase 234;
FUNCTION QNewSocketSessionAsync(host : pCHar location 'a0'; port : LONGINT location 'd0'; tags : pTagItem location 'a1') : pQSession; syscall AMarqueeBase 240;
FUNCTION QNewSocketServerSession( port : pLONGINT location 'a0'; tags : pTagItem location 'a1') : pQSession; syscall AMarqueeBase 246;
FUNCTION QSetKeyAccessOp(session : pQSession location 'a0'; path : pCHar location 'a1'; hosts : pCHar location 'd0') : LONGINT; syscall AMarqueeBase 252;
FUNCTION QGetHostName(session : pQSession location 'a0') : pCHar; syscall AMarqueeBase 258;
FUNCTION QGetProgName(session : pQSession location 'a0') : pCHar; syscall AMarqueeBase 264;
PROCEDURE QSetMaxRawBufSize(session : pQSession location 'a0'; maxBufSize : ULONG location 'd0'); syscall AMarqueeBase 270;
FUNCTION QNewSession(host : pCHar location 'a0'; port : LONGINT location 'd0'; name : pCHar location 'a1'; taglist : pTagItem location 'd1') : pQSession; syscall AMarqueeBase 276;
FUNCTION QNewSessionAsync(host : pCHar location 'a0'; port : LONGINT location 'd0'; name : pCHar location 'a1'; taglist : pTagItem location 'd1') : pQSession; syscall AMarqueeBase 282;
FUNCTION QNewHostSession(hostnames : pCHar location 'a0'; port : pLONGINT location 'a1'; names : pCHar location 'd0'; taglist : pTagItem location 'd1') : pQSession; syscall AMarqueeBase 288;
FUNCTION QNewServerSession(hostNames : pCHar location 'a0'; progNames : pCHar location 'a1'; taglist : pTagItem location 'd0') : pQSession; syscall AMarqueeBase 294;
FUNCTION QCreateSharedMessagePort : pQSharedMessagePort; syscall AMarqueeBase 300;
PROCEDURE QDeleteSharedMessagePort(mp : pQSharedMessagePort location 'a0'); syscall AMarqueeBase 306;
FUNCTION QGetLocalIP(session : pQSession location 'a0') : pCHAR; syscall AMarqueeBase 312;

{
     This is functions and procedures with array of PtrUInt.
     For use with fpc 1.0 and above.

}
FUNCTION QNewSocketSessiontags(host : pCHar; port : LONGINT; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewSocketSessionAsyncTags(host : pCHar; port : LONGINT; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewSocketServerSessionTags( port : pLONGINT; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewSessionTags(host : pCHar; port : LONGINT; name : pCHar; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewSessionAsyncTags(host : pCHar; port : LONGINT; name : pCHar; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewHostSessionTags(hostnames : pCHar; port : pLONGINT; names : pCHar; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewServerSessionTags(hostNames : pCHar; progNames : pCHar; const argv : array of PtrUInt) : pQSession;


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
     This is functions and procedures with array of PtrUInt.
     For use with fpc 1.0 and above.
}

FUNCTION QNewSocketSessionTags(host : string; port : LONGINT; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewSocketSessionAsyncTags(host : string; port : LONGINT; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewSessionTags(host : string; port : LONGINT; name : string; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewSessionAsyncTags(host : string; port : LONGINT; name : string; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewHostSessionTags(hostnames : string; port : pLONGINT; names : string; const argv : array of PtrUInt) : pQSession;
FUNCTION QNewServerSessionTags(hostNames : string; progNames : string; const argv : array of PtrUInt) : pQSession;

IMPLEMENTATION

uses
  pastoc;

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

FUNCTION QNewSocketSessiontags(host : pCHar; port : LONGINT; const argv : array of PtrUInt) : pQSession;
begin
    QNewSocketSessiontags := QNewSocketSession(host,port,@argv);
end;

FUNCTION QNewSocketSessionAsyncTags(host : pCHar; port : LONGINT; const argv : array of PtrUInt) : pQSession;
begin
    QNewSocketSessionAsyncTags := QNewSocketSessionAsync(host,port,@argv);
end;

FUNCTION QNewSocketServerSessionTags( port : pLONGINT; const argv : array of PtrUInt) : pQSession;
begin
    QNewSocketServerSessionTags := QNewSocketServerSession(port,@argv);
end;

FUNCTION QNewSessionTags(host : pCHar; port : LONGINT; name : pCHar; const argv : array of PtrUInt) : pQSession;
begin
    QNewSessionTags := QNewSession(host,port,name,@argv);
end;

FUNCTION QNewSessionAsyncTags(host : pCHar; port : LONGINT; name : pCHar; const argv : array of PtrUInt) : pQSession;
begin
    QNewSessionAsyncTags := QNewSessionAsync(host,port,name,@argv);
end;

FUNCTION QNewHostSessionTags(hostnames : pCHar; port : pLONGINT; names : pCHar; const argv : array of PtrUInt) : pQSession;
begin
    QNewHostSessionTags := QNewHostSession(hostnames,port,names,@argv);
end;

FUNCTION QNewServerSessionTags(hostNames : pCHar; progNames : pCHar; const argv : array of PtrUInt) : pQSession;
begin
    QNewServerSessionTags := QNewServerSession(hostnames,prognames,@argv);
end;


FUNCTION QNewSocketSessionTags(host : string; port : LONGINT; const argv : array of PtrUInt) : pQSession;
begin
    QNewSocketSessionTags := QNewSocketSession(host,port,@argv);
end;

FUNCTION QNewSocketSessionAsyncTags(host : string; port : LONGINT; const argv : array of PtrUInt) : pQSession;
begin
    QNewSocketSessionAsyncTags := QNewSocketSessionAsync(host,port,@argv);
end;

FUNCTION QNewSessionTags(host : string; port : LONGINT; name : string; const argv : array of PtrUInt) : pQSession;
begin
    QNewSessionTags := QNewSession(host,port,name,@argv);
end;

FUNCTION QNewSessionAsyncTags(host : string; port : LONGINT; name : string; const argv : array of PtrUInt) : pQSession;
begin
    QNewSessionAsyncTags := QNewSessionAsync(host,port,name,@argv);
end;

FUNCTION QNewHostSessionTags(hostnames : string; port : pLONGINT; names : string; const argv : array of PtrUInt) : pQSession;
begin
    QNewHostSessionTags := QNewHostSession(hostnames,port,names,@argv);
end;

FUNCTION QNewServerSessionTags(hostNames : string; progNames : string; const argv : array of PtrUInt) : pQSession;
begin
    QNewServerSessionTags := QNewServerSession(hostnames,prognames,@argv);
end;

const
    { Change VERSION and LIBVERSION to proper values }
    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

initialization
  AMarqueeBase := OpenLibrary(AMARQUEENAME,LIBVERSION);
finalization
  if Assigned(AMarqueeBase) then
    CloseLibrary(AMarqueeBase);
END. (* UNIT AMARQUEE *)



