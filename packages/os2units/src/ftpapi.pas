{
    Copyright (c) 2002 by Yuri Prokushev (prokushev@freemail.ru).

    Functions from FTPAPI.DLL (part of standard OS/2 Warp 4/eCS installation).

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 **********************************************************************}

{
@abstract(a unit to handle FTP)
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(22 Jul 2002)
@lastmod(01 Oct 2002)
This is functions from FTPAPI.DLL. Goal is ftp manipulation.
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
@todo(Rework some functions to support strings longer then 255 chars)
@todo(Finish functions description)
}
unit FTPAPI;

{****************************************************************************

                             RTL configuration

****************************************************************************}


interface

uses
  OS2Def,
  PMWin,
  Strings;

Const
  // window message id for post xfer updates
  WM_FTPAPI_XFER_UPDATE = WM_USER + 1000;

  // Transfer types is ASCII
  T_ASCII  = 1;
  // Transfer types is EBCDIC
  T_EBCDIC = 2;
  // Transfer types is BINARY
  T_BINARY = 3;

  // command/reply trace file modes
  M_OVERLAY = 1;
  M_APPEND  = 2;

  // command/reply tracing error codes

  // invalid trace file open mode
  TRCMODE = 1;
  // unable to open trace file
  TRCOPEN = 2;

  // Common error codes (try Ftp_ErrNo, all other functions returns usually
  // 0 if all ok, -1 if all bad)

  // No any error
  FTPNOERROR    = 00;
  // Unknown service.
  FTPSERVICE    = 01;
  // Unknown host.
  FTPHOST       = 02;
  // Unable to obtain socket.
  FTPSOCKET     = 03;
  // Unable to connect to server.
  FTPCONNECT    = 04;
  // Login failed.
  FTPLOGIN      = 05;
  // Transfer aborted.
  FTPABORT      = 06;
  // Problem opening the local file.
  FTPLOCALFILE  = 07;
  // Problem initializing data connection.
  FTPDATACONN   = 08;
  // Command failed.
  FTPCOMMAND    = 09;
  // Proxy server does not support third party transfers.
  FTPPROXYTHIRD = 10;
  // No primary connection for proxy transfer.
  FTPNOPRIMARY  = 11;
  // No code page translation table was loaded
  FTPNOXLATETBL = 12;

  // ping error codes

  // All ok
  PINGOK        = 0;
  // Host does not reply
  PINGREPLY     = -1;
  // Unable to obtain socket
  PINGSOCKET    = -3;
  // Unknown protocol ICMP
  PINGPROTO     = -4;
  // Send failed
  PINGSEND      = -5;
  // Recv() failed
  PINGRECV      = -6;
  // Unknown host (can't resolve)
  PINGHOST      = -7;

  // Restart Specific
  REST_GET = 1;
  REST_PUT = 2;

Const
  // Short functions vars
  ShortHost: String='';
  ShortUserId: String='';
  ShortPasswd: String='';
  ShortAcct: String='';
  ShortTransferType: Integer = T_ASCII;


{****************************************************************************

                          Opening and Closing Functions

****************************************************************************}

// Defines Host, UserId, Passwd and Acct for short function calls
Function FtpSetUser(Host, UserId, Passwd, Acct: String): Integer;

// Defines TransferType for short function calls
Function FtpSetBinary(TransferType: Integer): Integer;

// Stores the string containing the FTP API version
//   Buf is the buffer to store version string
//   BufLen is length of the buffer
// Version string is null-terminated and truncated to buffer length
Function FtpVer(var Buf; BufLen: Integer): Integer; cdecl;

Function FtpVer(Var Buf: String): Integer;

// Closes all current connections
Procedure FtpLogoff; cdecl;

{****************************************************************************

                       File Action Functions

****************************************************************************}

// Appends information to a remote file
//   Host is hostname. Use 'hostname portnumber' to specify non-standard port
//   UserID is user ID
//   Passwd is password
//   Acct is account (can be nil)
//   Local is local filename
//   Remote is Remote filename
//   TransferType is type of transfer (T_* constants)
Function FTPAppend(Host, UserId, Passwd, Acct, Local, Remote: PChar;
                   Transfertype: Integer): Integer; cdecl;

Function FTPAppend(Host, UserId, Passwd, Acct, Local, Remote: String;
                   Transfertype: Integer): Integer;

Function FTPAppend(Local, Remote: PChar; Transfertype: Integer): Integer;
Function FTPAppend(Local, Remote: String; Transfertype: Integer): Integer;

Function FTPAppend(Local, Remote: PChar): Integer;
Function FTPAppend(Local, Remote: String): Integer;

// Deletes files on a remote host
Function FtpDelete(Host, UserId, Passwd, Acct, Name: PChar): Integer; cdecl;

Function FtpDelete(Host, UserId, Passwd, Acct, Name: String): Integer;

Function FtpDelete(Name: PChar): Integer;
Function FtpDelete(Name: String): Integer;

// Renames a file on a remote host
Function FtpRename(Host, UserId, Passwd, Acct, NameFrom, NameTo: PChar): Integer; cdecl;

Function FtpRename(Host, UserId, Passwd, Acct, NameFrom, NameTo: String): Integer;

Function FtpRename(NameFrom, NameTo: PChar): Integer;
Function FtpRename(NameFrom, NameTo: String): Integer;

// Gets a file from an FTP server
// Mode is either 'w' for re_w_rite, or 'a' for _a_ppend
Function FtpGet(Host, UserId, Passwd, Acct, Local, Remote, Mode: PChar; TransferType: integer): Integer; cdecl;

Function FtpGet(Host, UserId, Passwd, Acct, Local, Remote, Mode: String; TransferType: integer): Integer;

Function FtpGet(Local, Remote, Mode: PChar; TransferType: integer): Integer;
Function FtpGet(Local, Remote, Mode: String; TransferType: integer): Integer;

Function FtpGet(Local, Remote, Mode: PChar): Integer;
Function FtpGet(Local, Remote, Mode: String): Integer;

// Transfers a file to an FTP server
Function FtpPut(Host, UserId, Passwd, Acct, Local, Remote: PChar; TransferType: Integer): Integer; cdecl;

Function FtpPut(Host, UserId, Passwd, Acct, Local, Remote: String; TransferType: Integer): Integer;

Function FtpPut(Local, Remote: PChar; TransferType: Integer): Integer;
Function FtpPut(Local, Remote: String; TransferType: Integer): Integer;

Function FtpPut(Local, Remote: PChar): Integer;
Function FtpPut(Local, Remote: String): Integer;

// Transfers a file to a host and ensures it is created with a unique name
Function FtpPutUnique(Host, UserId, Passwd, Acct, Local, Remote: PChar; TransferType: Integer): Integer; cdecl;

Function FtpPutUnique(Host, UserId, Passwd, Acct, Local, Remote: String; TransferType: Integer): Integer;

Function FtpPutUnique(Local, Remote: PChar; TransferType: Integer): Integer;
Function FtpPutUnique(Local, Remote: String; TransferType: Integer): Integer;

Function FtpPutUnique(Local, Remote: PChar): Integer;
Function FtpPutUnique(Local, Remote: String): Integer;

// Restarts an aborted transaction from the point of interruption
Function FtpReStart(Host, UserId, Passwd, Acct, Local, Remote, Mode: PChar; TransferType, Rest: Integer): Longint; cdecl;

Function FtpReStart(Host, UserId, Passwd, Acct, Local, Remote, Mode: String; TransferType, Rest: Integer): Longint;

Function FtpReStart(Local, Remote, Mode: String; TransferType, Rest: Integer): Longint;
Function FtpReStart(Local, Remote, Mode: String; Rest: Integer): Longint;


{****************************************************************************

                          Directory Listing Functions

****************************************************************************}

// Gets directory information in short format from a remote host and stores it to a local file
// You can use named pipes here to avoid a need for creating a real file
Function FtpLs(Host, Userid, Passwd, Acct, Local, Pattern: PChar): Integer; cdecl;

Function FtpLs(Host, Userid, Passwd, Acct, Local, Pattern: String): Integer;

Function FtpLs(Local, Pattern: String): Integer;

// Gets a directory in wide format from a host and stores it in file Local
// See comment regarding named pipes above
Function FtpDir(Host, UserId, Passwd, Acct, Local, Pattern: PChar): Integer; cdecl;

Function FtpDir(Host, Userid, Passwd, Acct, Local, Pattern: String): Integer;

Function FtpDir(Local, Pattern: String): Integer;


{****************************************************************************

                          Directory Action Functions

****************************************************************************}

// Changes the current working directory on a host
Function FtpCd(Host, Userid, Passwd, Acct, Dir: PChar): Integer; cdecl;

Function FtpCd(Host, Userid, Passwd, Acct, Dir: String): Integer;

Function FtpCd(Dir: String): Integer;

// Creates a new directory on a target machine
Function FtpMkd(Host, Userid, Passwd, Acct, Dir: PChar): Integer; cdecl;

Function FtpMkd(Host, Userid, Passwd, Acct, Dir: String): Integer;

Function FtpMkd(Dir: String): Integer;

// Removes a directory on a target machine
Function FtpRmd(Host, UserId, Passwd, Acct, Dir: PChar): Integer; cdecl;

Function FtpRmd(Host, UserId, Passwd, Acct, Dir: String): Integer;

Function FtpRmd(Dir: String): Integer;

// Stores the string containing the FTP server description of the current
// working directory on the host to the buffer
Function FtpPwd(Host, UserId, Passwd, Acct, Buf: PChar; BufLen: Integer): Integer; cdecl;

Function FtpPwd(Host, UserId, Passwd, Acct: String; var Buf: String): Integer;

Function FtpPwd(var Buf: String): Integer;

{****************************************************************************

                             Remote Server Functions

****************************************************************************}

// Sends a string to the server verbatim
Function FtpQuote(Host, UserId, Passwd, Acct, QuoteStr: PChar): Integer; cdecl;

Function FtpQuote(Host, UserId, Passwd, Acct, QuoteStr: String): Integer;

Function FtpQuote(QuoteStr: String): Integer;

// Executes the site command
Function FtpSite(Host, UserId, Passwd, Acct, SiteStr: PChar): Integer; cdecl;

Function FtpSite(Host, UserId, Passwd, Acct, SiteStr: String): Integer;

Function FtpSite(SiteStr: String): Integer;

// Stores the string containing the FTP server description of the operating
// system running on the host in a buffer
Function FtpSys(Host, UserId, Passwd, Acct, Buf: PChar; BufLen: Integer): Integer; cdecl;

Function FtpSys(Host, UserId, Passwd, Acct: String; var Buf: String): Integer;

Function FtpSys(var Buf: String): Integer;


// Transfers a file between two remote servers without sending the file to
// the local host
Function FtpProxy(Host1, UserId1, Passwd1, Acct1,
                  Host2, UserId2, Passwd2, Acct2,
                  FN1, FN2: PChar; TransferType: Integer): Integer; cdecl;

Function FtpProxy(Host1, UserId1, Passwd1, Acct1,
                  Host2, UserId2, Passwd2, Acct2,
                  FN1, FN2: String; TransferType: Integer): Integer;

Function FtpProxy(Host1, UserId1, Passwd1, Acct1,
                  Host2, UserId2, Passwd2, Acct2,
                  FN1, FN2: String): Integer;

// Resolves a host name and sends a ping to the remote host to determine if the host is responding
Function FtpPing(Host: PChar; Len: Integer; var Addr: Longint): Integer; cdecl;

Function FtpPing(Host: String; Len: Integer; var Addr: Longint): Integer;

// Sends a ping to the remote host to determine if the host is responding
Function Ping(Addr: Longint; Len: Integer): Integer; cdecl;

// Returns the size of a file on the remote host
Function FtpRemSize(Host, UserId, Passwd, Acct, Local, Remote, Mode: Pchar; TransferType: Integer): Longint; cdecl;

Function FtpRemSize(Host, UserId, Passwd, Acct, Local, Remote, Mode: String; TransferType: Integer): Longint;

Function FtpRemSize(Local, Remote, Mode: String; TransferType: Integer): Longint;

Function FtpRemSize(Local, Remote, Mode: String): Longint;

// Maintain the original date/time of files received.
Function Keep_File_Date(LocalFile, RemoteFile: PChar): Boolean; cdecl;

Function Keep_File_Date(LocalFile, RemoteFile: String): Boolean;

{****************************************************************************

                                  Trace Functions

****************************************************************************}


// Opens the trace file specified and starts tracing
Function FtpTrcOn(FileSpec: PChar; Mode: Integer): Integer; cdecl;

Function FtpTrcOn(FileSpec: String; Mode: Integer): Integer; cdecl;

// Closes the trace file, and stops tracing of the command and reply sequences that
// were sent over the control connection between the local and remote hosts
Function FtpTrcOff: Integer; cdecl;

{****************************************************************************

                                  Other Functions

****************************************************************************}

// FTP error No
Function Ftp_ErrNo: Integer; cdecl;


(* Undocumented / unimplemented functions:
Function FtpXLate(Dig: Longint; St:PChar): Longint; cdecl;
Procedure FtpXferWnd(var _hwnd: HWND); cdecl;
Procedure FtpSetConvertMode(var code: integer); cdecl;
Procedure FtpSetEncodeMode(var code: integer); cdecl;
Procedure FtpSetDecodeMode(var code: integer); cdecl;
Function FtpSetActiveMode(var UseActiveOnly: integer): integer; cdecl;
*)

implementation

const
  FTPAPIDLL = 'FTPAPI';

Function FTPAppend(Host, UserId, Passwd, Acct, Local, Remote: PChar;
                   Transfertype: Integer): Integer; cdecl;
    external FTPAPIDLL index 1;

Function FTPAppend(Host, UserId, Passwd, Acct, Local, Remote: String;
                   Transfertype: Integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, Host);
  StrPCopy(@_UserId, UserId);
  StrPCopy(@_Passwd, Passwd);
  StrPCopy(@_Acct, Acct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpAppend:=FtpAppend(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, TransferType);
End;

Function FTPAppend(Local, Remote: PChar; TransferType: Integer): Integer;
Var
  Host, UserId, Passwd, Acct: Array[0..255] of Char;
Begin
  StrPCopy(@Host, ShortHost);
  StrPCopy(@UserId, ShortUserId);
  StrPCopy(@Passwd, ShortPasswd);
  StrPCopy(@Acct, ShortAcct);
  FtpAppend:=FtpAppend(@Host, @UserId, @Passwd, @Acct, Local, Remote, TransferType);
End;

Function FTPAppend(Local, Remote: PChar): Integer;
Begin
  FtpAppend:=FtpAppend(Local, Remote, ShortTransferType);
End;

Function FTPAppend(Local, Remote: String; TransferType: Integer): Integer;
Var
  _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpAppend:=FtpAppend(@_Local, @_Remote, TransferType);
End;

Function FTPAppend(Local, Remote: String): Integer;
Var
  _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpAppend:=FtpAppend(@_Local, @_Remote);
End;

Function FtpCd(Host, Userid, Passwd, Acct, Dir: PChar): Integer; cdecl;
    external FTPAPIDLL index 2;

Function FtpCd(Host, Userid, Passwd, Acct, Dir: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Dir: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  GetMem(_Dir, Length(Dir)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  StrPCopy(_Dir, Dir);
  FtpCd:=FtpCd(_Host, _Userid, _Passwd, _Acct, _Dir);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
  FreeMem(_Dir, Length(Dir)+1);
End;

Function FtpCd(Dir: String): Integer;
Begin
  FtpCd:=FtpCd(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Dir);
End;

Function FtpDelete(Host, UserId, Passwd, Acct, Name: PChar): Integer; cdecl;
    external FTPAPIDLL index 3;

Function FtpDelete(Host, UserId, Passwd, Acct, Name: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Name: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, Host);
  StrPCopy(@_UserId, UserId);
  StrPCopy(@_Passwd, Passwd);
  StrPCopy(@_Acct, Acct);
  StrPCopy(@_Name, Name);
  FtpDelete:=FtpDelete(@_Host, @_UserId, @_Passwd, @_Acct, @_Name);
End;

Function FtpDelete(Name: PChar): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  FtpDelete:=FtpDelete(@_Host, @_UserId, @_Passwd, @_Acct, Name);
End;

Function FtpDelete(Name: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Name: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  StrPCopy(@_Name, Name);
  FtpDelete:=FtpDelete(@_Host, @_UserId, @_Passwd, @_Acct, @_Name);
End;

Function FtpDir(Host, UserId, Passwd, Acct, Local, Pattern: PChar): Integer; cdecl;
    external FTPAPIDLL index 4;

Function FtpDir(Host, Userid, Passwd, Acct, Local, Pattern: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Pattern: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  GetMem(_Local, Length(Local)+1);
  GetMem(_Pattern, Length(Pattern)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  StrPCopy(_Local, Local);
  StrPCopy(_Pattern, Pattern);
  FtpDir:=FtpDir(_Host, _Userid, _Passwd, _Acct, _Local, _Pattern);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
  FreeMem(_Local, Length(Local)+1);
  FreeMem(_Pattern, Length(Pattern)+1);
End;

Function FtpDir(Local, Pattern: String): Integer;
Begin
  FtpDir:=FtpDir(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Local, Pattern);
End;

Function FtpLs(Host, Userid, Passwd, Acct, Local, Pattern: PChar): Integer; cdecl;
    external FTPAPIDLL index 7;

Function FtpLs(Host, Userid, Passwd, Acct, Local, Pattern: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Pattern: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  GetMem(_Local, Length(Local)+1);
  GetMem(_Pattern, Length(Pattern)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  StrPCopy(_Local, Local);
  StrPCopy(_Pattern, Pattern);
  FtpLs:=FtpLs(_Host, _Userid, _Passwd, _Acct, _Local, _Pattern);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
  FreeMem(_Local, Length(Local)+1);
  FreeMem(_Pattern, Length(Pattern)+1);
End;

Function FtpLs(Local, Pattern: String): Integer;
Begin
  FtpLs:=FtpLs(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Local, Pattern);
End;

Function FtpGet(Host, UserId, Passwd, Acct, Local, Remote, Mode: PChar; TransferType: integer): Integer; cdecl;
    external FTPAPIDLL index 5;

Function FtpGet(Host, UserId, Passwd, Acct, Local, Remote, Mode: String; TransferType: integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote, _Mode: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, Host);
  StrPCopy(@_UserId, UserId);
  StrPCopy(@_Passwd, Passwd);
  StrPCopy(@_Acct, Acct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  StrPCopy(@_Mode, Mode);
  FtpGet:=FtpGet(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, @_Mode, TransferType);
End;

Function FtpGet(Local, Remote, Mode: PChar; TransferType: integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  FtpGet:=FtpGet(@_Host, @_UserId, @_Passwd, @_Acct, Local, Remote, Mode, TransferType);
End;

Function FtpGet(Local, Remote, Mode: String; TransferType: integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote, _Mode: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  StrPCopy(@_Mode, Mode);
  FtpGet:=FtpGet(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, @_Mode, TransferType);
End;

Function FtpGet(Local, Remote, Mode: PChar): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  FtpGet:=FtpGet(@_Host, @_UserId, @_Passwd, @_Acct, Local, Remote, Mode, ShortTransferType);
End;

Function FtpGet(Local, Remote, Mode: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote, _Mode: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  StrPCopy(@_Mode, Mode);
  FtpGet:=FtpGet(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, @_Mode, ShortTransferType);
End;

Procedure FtpLogoff; cdecl;
    external FTPAPIDLL index 6;

Function FtpMkd(Host, Userid, Passwd, Acct, Dir: PChar): Integer; cdecl;
    external FTPAPIDLL index 8;

Function FtpMkD(Host, Userid, Passwd, Acct, Dir: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Dir: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  GetMem(_Dir, Length(Dir)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  StrPCopy(_Dir, Dir);
  FtpMkD:=FtpMkD(_Host, _Userid, _Passwd, _Acct, _Dir);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
  FreeMem(_Dir, Length(Dir)+1);
End;

Function FtpMkD(Dir: String): Integer;
Begin
  FtpMkD:=FtpMkD(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Dir);
End;

Function FtpPing(Host: PChar; Len: Integer; var Addr: Longint): Integer; cdecl;
    external FTPAPIDLL index 9;

Function FtpPing(Host: String; Len: Integer; var Addr: Longint): Integer;
var
  _Host: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  StrPCopy(_Host, Host);
  FtpPing:=FtpPing(_Host, Len, Addr);
  FreeMem(_Host, Length(Host)+1);
End;

Function FtpProxy(Host1, UserId1, Passwd1, Acct1,
                  Host2, UserId2, Passwd2, Acct2,
                  FN1, FN2: PChar; TransferType: Integer): Integer; cdecl;
    external FTPAPIDLL index 10;

Function FtpProxy(Host1, UserId1, Passwd1, Acct1,
                  Host2, UserId2, Passwd2, Acct2,
                  FN1, FN2: String; TransferType: Integer): Integer;
Begin
  Host1:=Host2+#0;
  Host2:=Host2+#0;
  UserId1:=UserId1+#0;
  UserId2:=UserId2+#0;
  Passwd1:=Passwd1+#0;
  Passwd2:=Passwd2+#0;
  Acct1:=Acct1+#0;
  Acct2:=Acct2+#0;
  FN1:=FN1+#0;
  FN2:=FN2+#0;
  FtpProxy:=FtpProxy(@Host1[1], @UserId1[1], @Passwd1[1], @Acct1[1],
                  @Host2[1], @UserId2[1], @Passwd2[1], @Acct2[1],
                  @FN1[1], @FN2[1], TransferType);
End;

Function FtpProxy(Host1, UserId1, Passwd1, Acct1,
                  Host2, UserId2, Passwd2, Acct2,
                  FN1, FN2: String): Integer;
Begin
  FtpProxy:=FtpProxy(Host1, UserId1, Passwd1, Acct1,
                  Host2, UserId2, Passwd2, Acct2,
                  FN1, FN2, ShortTransferType);
End;

Function FtpPut(Host, UserId, Passwd, Acct, Local, Remote: PChar; TransferType: Integer): Integer; cdecl;
    external FTPAPIDLL index 11;

Function FtpPut(Host, UserId, Passwd, Acct, Local, Remote: String; TransferType: Integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, Host);
  StrPCopy(@_UserId, UserId);
  StrPCopy(@_Passwd, Passwd);
  StrPCopy(@_Acct, Acct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpPut:=FtpPut(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, TransferType);
End;

Function FtpPut(Local, Remote: PChar; TransferType: Integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  FtpPut:=FtpPut(@_Host, @_UserId, @_Passwd, @_Acct, Local, Remote, TransferType);
End;

Function FtpPut(Local, Remote: String; TransferType: Integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpPut:=FtpPut(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, TransferType);
End;

Function FtpPut(Local, Remote: PChar): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  FtpPut:=FtpPut(@_Host, @_UserId, @_Passwd, @_Acct, Local, Remote, ShortTransferType);
End;

Function FtpPut(Local, Remote: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpPut:=FtpPut(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, ShortTransferType);
End;

Function FtpPutUnique(Host, UserId, Passwd, Acct, Local, Remote: PChar; TransferType: Integer): Integer; cdecl;
    external FTPAPIDLL index 12;

Function FtpPutUnique(Host, UserId, Passwd, Acct, Local, Remote: String; TransferType: Integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, Host);
  StrPCopy(@_UserId, UserId);
  StrPCopy(@_Passwd, Passwd);
  StrPCopy(@_Acct, Acct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpPutUnique:=FtpPutUnique(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, TransferType);
End;

Function FtpPutUnique(Local, Remote: PChar; TransferType: Integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  FtpPutUnique:=FtpPutUnique(@_Host, @_UserId, @_Passwd, @_Acct, Local, Remote, TransferType);
End;

Function FtpPutUnique(Local, Remote: String; TransferType: Integer): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpPutUnique:=FtpPutUnique(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, TransferType);
End;

Function FtpPutUnique(Local, Remote: PChar): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  FtpPutUnique:=FtpPutUnique(@_Host, @_UserId, @_Passwd, @_Acct, Local, Remote, ShortTransferType);
End;

Function FtpPutUnique(Local, Remote: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  StrPCopy(@_Local, Local);
  StrPCopy(@_Remote, Remote);
  FtpPutUnique:=FtpPutUnique(@_Host, @_UserId, @_Passwd, @_Acct, @_Local, @_Remote, ShortTransferType);
End;

Function FtpPwd(Host, UserId, Passwd, Acct, Buf: PChar; BufLen: Integer): Integer; cdecl;
    external FTPAPIDLL index 13;

Function FtpPwd(Host, UserId, Passwd, Acct: String; var Buf: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: PChar;
  _Buf: Array[0..255] of Char;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  FtpPwd:=FtpPwd(_Host, _UserId, _Passwd, _Acct, @_Buf, SizeOf(_Buf));
  Buf:=StrPas(@_Buf);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
End;

Function FtpPwd(var Buf: String): Integer;
Begin
  FtpPwd:=FtpPwd(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Buf);
End;


Function FtpQuote(Host, UserId, Passwd, Acct, QuoteStr: PChar): Integer; cdecl;
    external FTPAPIDLL index 14;

Function FtpQuote(Host, UserId, Passwd, Acct, QuoteStr: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _QuoteStr: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  GetMem(_QuoteStr, Length(QuoteStr)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  StrPCopy(_QuoteStr, QuoteStr);
  FtpQuote:=FtpQuote(_Host, _UserId, _Passwd, _Acct, _QuoteStr);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
  FreeMem(_QuoteStr, Length(QuoteStr)+1);
End;

Function FtpQuote(QuoteStr: String): Integer;
Begin
  FtpQuote:=FtpQuote(ShortHost, ShortUserId, ShortPasswd, ShortAcct, QuoteStr);
End;

Function FtpRename(Host, UserId, Passwd, Acct, NameFrom, NameTo: PChar): Integer; cdecl;
    external FTPAPIDLL index 15;

Function FtpRename(Host, UserId, Passwd, Acct, NameFrom, NameTo: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _NameFrom, _NameTo: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, Host);
  StrPCopy(@_UserId, UserId);
  StrPCopy(@_Passwd, Passwd);
  StrPCopy(@_Acct, Acct);
  StrPCopy(@_NameTo, NameTo);
  StrPCopy(@_NameFrom, NameFrom);
  FtpRename:=FtpRename(@_Host, @_UserId, @_Passwd, @_Acct, @_NameFrom, @_NameTo);
End;

Function FtpRename(NameFrom, NameTo: PChar): Integer;
Var
  _Host, _UserId, _Passwd, _Acct: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  FtpRename:=FtpRename(@_Host, @_UserId, @_Passwd, @_Acct, NameFrom, NameTo);
End;

Function FtpRename(NameFrom, NameTo: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _NameFrom, _NameTo: Array[0..255] of Char;
Begin
  StrPCopy(@_Host, ShortHost);
  StrPCopy(@_UserId, ShortUserId);
  StrPCopy(@_Passwd, ShortPasswd);
  StrPCopy(@_Acct, ShortAcct);
  StrPCopy(@_NameTo, NameTo);
  StrPCopy(@_NameFrom, NameFrom);
  FtpRename:=FtpRename(@_Host, @_UserId, @_Passwd, @_Acct, @_NameFrom, @_NameTo);
End;

Function FtpRmd(Host, UserId, Passwd, Acct, Dir: PChar): Integer; cdecl;
    external FTPAPIDLL index 16;

Function FtpRmD(Host, Userid, Passwd, Acct, Dir: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _Dir: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  GetMem(_Dir, Length(Dir)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  StrPCopy(_Dir, Dir);
  FtpRmD:=FtpRmD(_Host, _Userid, _Passwd, _Acct, _Dir);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
  FreeMem(_Dir, Length(Dir)+1);
End;

Function FtpRmD(Dir: String): Integer;
Begin
  FtpRmD:=FtpRmD(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Dir);
End;

Function FtpSite(Host, UserId, Passwd, Acct, SiteStr: PChar): Integer; cdecl;
    external FTPAPIDLL index 17;

Function FtpSite(Host, UserId, Passwd, Acct, SiteStr: String): Integer;
Var
  _Host, _UserId, _Passwd, _Acct, _SiteStr: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  GetMem(_SiteStr, Length(SiteStr)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  StrPCopy(_SiteStr, SiteStr);
  FtpSite:=FtpSite(_Host, _Userid, _Passwd, _Acct, _SiteStr);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
  FreeMem(_SiteStr, Length(SiteStr)+1);
End;

Function FtpSite(SiteStr: String): Integer;
Begin
  FtpSite:=FtpSite(ShortHost, ShortUserid, ShortPasswd, ShortAcct, SiteStr);
End;

Function FtpSys(Host, UserId, Passwd, Acct, Buf: PChar; BufLen: Integer): Integer; cdecl;
    external FTPAPIDLL index 18;

Function FtpSys(Host, UserId, Passwd, Acct: String; var Buf: String): Integer;
var
  _Buf: Array[0..255] of char;
Begin
  Host:=Host+#0;
  UserId:=UserId+#0;
  Passwd:=Passwd+#0;
  Acct:=Acct+#0;
  FtpSys:=FtpSys(@Host[1], @UserId[1], @Passwd[1], @Acct[1], @_Buf, SizeOf(Buf));
  Buf:=StrPas(@_Buf);
End;

Function FtpSys(var Buf: String): Integer;
Begin
  FtpSys:=FtpSys(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Buf);
End;

Function Ping(Addr: Longint; Len: Integer): Integer; cdecl;
    external FTPAPIDLL index 19;

Function Ftp_ErrNo: Integer; cdecl;
    external FTPAPIDLL index 21;

Function FtpVer(var Buf; BufLen: Integer): Integer; cdecl;
    external FTPAPIDLL index 23;

Function FtpVer(var Buf: String): Integer;
var
  T:array[0..255] of char;
begin
  FtpVer:=FtpVer(T, SizeOf(T));
  Buf:=StrPas(T);
end;

Function FtpTrcOn(FileSpec: PChar; Mode: Integer): Integer; cdecl;
    external FTPAPIDLL index 24;

Function FtpTrcOn(FileSpec: String; Mode: Integer): Integer; cdecl;
Begin
  FileSpec:=FileSpec+#0;
  FtpTrcOn:=FtpTrcOn(@FileSpec[1], Mode);
End;

Function FtpTrcOff: Integer; cdecl;
    external FTPAPIDLL index 25;

Function Keep_File_Date(LocalFile, RemoteFile: PChar): Boolean; cdecl;
    external FTPAPIDLL index 30;

Function Keep_File_Date(LocalFile, RemoteFile: String): Boolean;
Begin
  LocalFile:=LocalFile+#0;
  RemoteFile:=RemoteFile+#0;
  Keep_File_Date:=Keep_File_Date(@LocalFile[1], @RemoteFile[1]);
End;

Function FtpReStart(Host, UserId, Passwd, Acct, Local, Remote, Mode: PChar; TransferType, Rest: Integer): Longint; cdecl;
    external FTPAPIDLL index 31;

Function FtpReStart(Host, UserId, Passwd, Acct, Local, Remote, Mode: String; TransferType, Rest: Integer): Longint;
Var
  _Host, _UserId, _Passwd, _Acct, _Local, _Remote, _Mode: PChar;
Begin
  GetMem(_Host, Length(Host)+1);
  GetMem(_UserId, Length(UserId)+1);
  GetMem(_Passwd, Length(Passwd)+1);
  GetMem(_Acct, Length(Acct)+1);
  GetMem(_Local, Length(Local)+1);
  GetMem(_Remote, Length(Remote)+1);
  GetMem(_Mode, Length(Mode)+1);
  StrPCopy(_Host, Host);
  StrPCopy(_UserId, UserId);
  StrPCopy(_Passwd, Passwd);
  StrPCopy(_Acct, Acct);
  StrPCopy(_Local, Local);
  StrPCopy(_Remote, Remote);
  StrPCopy(_Mode, Mode);
  FtpReStart:=FtpReStart(_Host, _UserId, _Passwd, _Acct, _Local, _Remote, _Mode, TransferType, Rest);
  FreeMem(_Host, Length(Host)+1);
  FreeMem(_UserId, Length(UserId)+1);
  FreeMem(_Passwd, Length(Passwd)+1);
  FreeMem(_Acct, Length(Acct)+1);
  FreeMem(_Local, Length(Local)+1);
  FreeMem(_Remote, Length(Remote)+1);
  FreeMem(_Mode, Length(Mode)+1);
End;

Function FtpReStart(Local, Remote, Mode: String; TransferType, Rest: Integer): Longint;
Begin
  FtpReStart:=FtpReStart(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Local, Remote, Mode, TransferType, Rest);
End;

Function FtpReStart(Local, Remote, Mode: String; Rest: Integer): Longint;
Begin
  FtpReStart:=FtpReStart(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Local, Remote, Mode, ShortTransferType, Rest);
End;

Function FtpRemSize(Host, UserId, Passwd, Acct, Local, Remote, Mode: Pchar; TransferType: Integer): Longint; cdecl;
    external FTPAPIDLL index 32;

Function FtpRemSize(Host, UserId, Passwd, Acct, Local, Remote, Mode: String; TransferType: Integer): Longint;
Begin
  Host:=Host+#0;
  UserId:=UserId+#0;
  Passwd:=Passwd+#0;
  Acct:=Acct+#0;
  Local:=Local+#0;
  Remote:=Remote+#0;
  Mode:=Mode+#0;
  FtpRemSize:=FtpRemSize(@Host[1], @UserId[1], @Passwd[1], @Acct[1], @Local[1], @Remote[1], @Mode[1], TransferType);
End;

Function FtpRemSize(Local, Remote, Mode: String; TransferType: Integer): Longint;
Begin
  FtpRemSize:=FtpRemSize(ShortHost, ShortUserId, ShortPasswd, ShortAcct, Local, Remote, Mode, TransferType);
End;

Function FtpRemSize(Local, Remote, Mode: String): Longint;
Begin
  FtpRemSize:=FtpRemSize(Local, Remote, Mode, ShortTransferType);
End;

Function FtpSetUser(Host, UserId, Passwd, Acct: String): Integer;
Begin
  ShortHost:=Host;
  ShortUserId:=UserId;
  ShortPasswd:=Passwd;
  ShortAcct:=Acct;
  FtpSetUser:=0;
  If (Host='') or (UserId='') then FtpSetUser:=-1;
End;

Function FtpSetBinary(TransferType: Integer): Integer;
Begin
  ShortTransferType:=TransferType;
  FtpSetBinary:=0;
End;

(* Undocumented functions follow
Function FtpXLate(Dig: Longint; St:PChar): Longint; cdecl;
                                                   external FTPAPIDLL index 22;
Procedure FtpXferWnd(var _hwnd: HWND); cdecl; external FTPAPIDLL index 26;
Procedure FtpSetConvertMode(var code: integer); cdecl;
                                                   external FTPAPIDLL index 27;
Procedure FtpSetEncodeMode(var code: integer); cdecl;
                                                   external FTPAPIDLL index 28;
Procedure FtpSetDecodeMode(var code: integer); cdecl;
                                                   external FTPAPIDLL index 29;

Absolutely no information about following functions:

var FtpErrNo: integer; cdecl; external FTPAPIDLL index 20; // seems to be a copy of ftp_errno
//³ 00033 ³ FTPQUOTEREPLY     // Seems to be direct command send (reply to ftpquote)
//³ 00034 ³ FtpSetActiveMode

*)

End.
