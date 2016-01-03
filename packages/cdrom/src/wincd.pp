{
}
unit wincd;

{$mode objfpc}
{$h+}

interface

uses Windows,SysUtils;

Type
  TCDAccessMethod = (camNone,camASPI,camSPTI,camIOCTL);
{$packrecords c}
TTOCTrack = packed record
  rsvd,
  ADR,
  trackNumber,
  rsvd2 : Byte;
  addr : Array[0..3] of byte;
end;

TTOC = packed Record
  toclen : word;
  firsttrack,
  lastTrack  : byte;
  toctrack:  Array[0..99] of TTocTrack;
end;

Const
  AccessMethodNames : Array[TCDAccessMethod] of string
                    = ('None','ASPI','SPTI','IOCTL');

Function  GetCDAccessMethod : TCDAccessMethod;
Procedure SetCDAccessMethod (Value : TCDAccessMethod);
Function  ReadTOC(Device : String; Var TOC : TTOc) : Integer;
Function  EnumCDDrives(Var Drives : Array of String) : Integer;
Function  GetNumDrives : Integer;

implementation

uses cdromioctl,wnaspi32,scsidefs;

Var
  CurrentAccessMethod : TCDAccessMethod;
  CDOSVer : Integer;

{ ---------------------------------------------------------------------
  SPTI Defines.
  ---------------------------------------------------------------------}

Type

{$packrecords C}
 SCSI_PASS_THROUGH = record
   Length : USHORT;
   ScsiStatus : UCHAR;
   PathId : UCHAR;
   TargetId : UCHAR;
   Lun : UCHAR;
   CdbLength : UCHAR;
   SenseInfoLength : UCHAR;
   DataIn : UCHAR;
   DataTransferLength : ULONG;
   TimeOutValue : ULONG;
   DataBufferOffset : ULONG;
   SenseInfoOffset : ULONG;
   Cdb : array[0..15] of UCHAR;
 end;
 TSCSI_PASS_THROUGH = SCSI_PASS_THROUGH;
 PSCSI_PASS_THROUGH = ^TSCSI_PASS_THROUGH;

 SCSI_PASS_THROUGH_DIRECT = record
   Length : USHORT;
   ScsiStatus : UCHAR;
   PathId : UCHAR;
   TargetId : UCHAR;
   Lun : UCHAR;
   CdbLength : UCHAR;
   SenseInfoLength : UCHAR;
   DataIn : UCHAR;
   DataTransferLength : ULONG;
   TimeOutValue : ULONG;
   DataBuffer : PVOID;
   SenseInfoOffset : ULONG;
   Cdb : array[0..15] of UCHAR;
 end;
 TSCSI_PASS_THROUGH_DIRECT = SCSI_PASS_THROUGH_DIRECT;
 PSCSI_PASS_THROUGH_DIRECT = ^SCSI_PASS_THROUGH_DIRECT;

 SCSI_PASS_THROUGH_DIRECT_WITH_BUFFER = record
    spt : SCSI_PASS_THROUGH_DIRECT;
    Filler : ULONG;
    ucSenseBuf : array[0..31] of UCHAR;
 end;
 TSCSI_PASS_THROUGH_DIRECT_WITH_BUFFER = SCSI_PASS_THROUGH_DIRECT_WITH_BUFFER;
 PSCSI_PASS_THROUGH_DIRECT_WITH_BUFFER = ^SCSI_PASS_THROUGH_DIRECT_WITH_BUFFER;


const
 METHOD_BUFFERED   = 0;
 METHOD_IN_DIRECT  = 1;
 METHOD_OUT_DIRECT = 2;
 METHOD_NEITHER    = 3;

 FILE_ANY_ACCESS   = 0;
 FILE_READ_ACCESS  = $0001;
 FILE_WRITE_ACCESS = $0002;
 IOCTL_CDROM_BASE  = $00000002;
 IOCTL_SCSI_BASE   = $00000004;

 SCSI_IOCTL_DATA_OUT         = 0;
 SCSI_IOCTL_DATA_IN          = 1;
 SCSI_IOCTL_DATA_UNSPECIFIED = 2;

{ ---------------------------------------------------------------------
  Initialization code.
  ---------------------------------------------------------------------}

procedure InitWinCD;

Var
  TheCDOSVER : TOSVersionInfo;

begin
  TheCDOSVer.dwOSVersionInfoSize:=SizeOf(TheCDOSver);
  GetVersionEx(TheCDOSVer);
  CDOSVer:=TheCDOSVer.dwMajorVersion;
  If AspiLoaded then
    CurrentAccessMethod := camASPI
  else
    begin
    if (CDOSver<1) then
      CurrentAccessMethod := camNone
    else
      {
        It is better to use SPTI on windows, but the problem with that
        is that administrative priviledges are needed. A detection
        algorithm for these priviledges here would be nice.
      }
      CurrentAccessMethod := camSPTI;
    end;
end;

{ ---------------------------------------------------------------------
  Actual reading of table of contents.
  ---------------------------------------------------------------------}

{ ---------------------------------------------------------------------
  1. SPTI
  ---------------------------------------------------------------------}

Function sptiReadTOC(Device : String; var TOC: TToC) : Integer;

Var
  DriveHandle : THandle;
  len : Cardinal;
  buf : Array[0..31] of char;
  ID,retVal : Integer;
  Returned,Flags : Cardinal;
  swb : TSCSI_PASS_THROUGH_DIRECT_WITH_BUFFER;
begin
  Flags := Cardinal(GENERIC_READ);
  if (CDOSVer>4) then
    Flags:=Flags or Cardinal(GENERIC_WRITE);
  Device:=Upcase('\\.\'+Device);
  DriveHandle:=CreateFile(pchar(Device),Flags,FILE_SHARE_READ,
                          nil,OPEN_EXISTING, 0, 0 );
  if (DriveHandle=INVALID_HANDLE_VALUE) then
    begin
    Result:=-1;
    Exit;
    end;
  Try
    Returned:= 0;
    len:= sizeof(SCSI_PASS_THROUGH_DIRECT_WITH_BUFFER);
    FillChar(swb, len ,0);
    With swb.spt do
      begin
      Length := sizeof(swb.spt); // SCSI_PASS_THROUGH);
      CdbLength       := 10;
      DataIn          := SCSI_IOCTL_DATA_IN;
      DataTransferLength := SizeOf(tOC);
      TimeOutValue    := 5;
      DataBuffer      := @TOC;
      SenseInfoOffset := @swb.ucSenseBuf[0]-pbyte(@swb.spt);
      Cdb[0]          := $43; // read TOC
      Cdb[1]          := $02; // MSF mode
      Cdb[7]          := $03;
      Cdb[8]          := $24;
      end;
    if (Not DeviceIoControl(DriveHandle,
                            IOCTL_SCSI_PASS_THROUGH_DIRECT,
                            @swb,
                            len,
                            @swb,
                            len,
                            @Returned,
                            Nil)) then
      begin
      Result:=-1;
      Exit;
      end;
    With TOC do
      Result:=LastTrack-FirstTrack+1;
  finally
    CloseHandle(DriveHandle);
  end;
end;

{ ---------------------------------------------------------------------
  2. ASPI
  ---------------------------------------------------------------------}

Function AspiGetNumAdapters : Integer;

Var
  D : DWORD;
  Count, Status : Byte;

begin
  d:= GetASPI32SupportInfo();
  Count:=D and $FF;
  Status:=(D shr 8) and $ff;
  if (Status<>SS_COMP) and (Status<>SS_NO_ADAPTERS) then
    Result:=-1
  else
    Result:=Count;
end;

Function DriveToSCSIParm (Device : String; Var HID,TGT,LUN : Byte) : Boolean;

Var
  Code : Integer;

begin
  Result:=False;
  Code:=Pos('[',Device);
  if Code<>0 then
    begin
    Delete(Device,1,Code);
    Code:=Pos(';',Device);
    HID:=StrToIntDef(Copy(Device,1,Code-1),-1);
    Result:=HID<>-1;
    If result then
      begin
      Delete(DEvice,1,Code);
      Code:=Pos(';',Device);
      Tgt:=StrToIntDef(Copy(Device,1,Code-1),-1);
      Result:=tgt<>-1;
      If result then
        begin
        Delete(DEvice,1,Code);
        Code:=Pos(']',Device);
        Lun:=StrToIntDef(Copy(Device,1,Code-1),-1);
        Result:=Lun<>-1;
        end;
      end;
    end;
end;

Var
  Atoc : TTOc;

Function AspiReadTOC(Device : String; Var TOC : TTOC) : Integer;

Var
  HAID,TGT,LUN : Byte;
  Status : DWord;
  S,T : SRB_ExecSCSICmd;
  HEvent : THANDLE;

begin
  If Not DriveToSCSIParm(Device,HAID,TGT,lun) then
    begin
    Result:=-1;
    Exit;
    end;
  Writeln('About to read toc from ',haid,' ',tgt,' ',lun);
  hEvent:=CreateEvent( nil, TRUE, FALSE, nil );
  Writeln('Resetting event');
  ResetEvent(hEvent);
  Writeln('Reset event');
  Try
   FillChar(S,sizeof(s),0);
   s.SRB_Cmd      := SC_EXEC_SCSI_CMD;
   s.SRB_HaID     := HaID;
   s.SRB_Target   := Tgt;
   s.SRB_Lun      := lun;
   s.SRB_Flags    := SRB_DIR_IN or SRB_EVENT_NOTIFY;
   s.SRB_BufLen   := SizeOf(Toc);
   s.SRB_BufPointer := @TOC;
   s.SRB_SenseLen := SENSE_LEN;
   s.SRB_CDBLen   := $0A;
   s.SRB_PostProc := LPVOID(hEvent);
   s.CDBByte[0]   := SCSI_READ_TOC;   // read TOC command
   s.CDBByte[1]   := $02;   // MSF mode
   s.CDBByte[7]   := HiByte(Word(S.SRB_BufLen));   // high-order byte of buffer len
   s.CDBByte[8]   := LoByte(Word(S.SRB_BUFLEN));   // low-order byte of buffer len
   Writeln('Sending Command');
   SendASPI32Command(LPSRB(@s));
   Writeln('Sent Command');
   Status:=S.SRB_STATUS;
   Writeln('Command status,',Status);
   if (Status=SS_PENDING ) then
     begin
     Writeln('Waiting for object');
     WaitForSingleObject( hEvent, 10000 );  // wait up to 10 secs
     Writeln('Waiting ended');
     end;
  Finally
    CloseHandle( hEvent );
  end;
  if (S.SRB_Status<>SS_COMP ) then
    begin
    Result:=-1;
    Exit;
    end;
  Writeln('Command completed');
  With TOC do
    Result:=LastTrack-FirstTrack+1;
end;

{ ---------------------------------------------------------------------
  3. IOCTL
  ---------------------------------------------------------------------}

Function ioctlReadTOC(Device : String; Var TOC : TTOC) : Integer;

Var
  DriveHandle : Thandle;
  Retval : Longint;
  Returned : DWord;
  Flags : Cardinal;

begin
  Flags:=Cardinal(GENERIC_READ);
  device:=Upcase('\\.\'+device);
  DriveHandle:=CreateFile(PChar(Device), Flags,
                          FILE_SHARE_READ, nil, OPEN_EXISTING, 0, 0 );
  if (DriveHandle = INVALID_HANDLE_VALUE) then
    begin
    result:=-1;
    exit;
    end;
  Try
    Returned := 0;
    FillChar(Toc, sizeof(TOC),0 );
    if Not DeviceIoControl(DriveHandle,
                           IOCTL_CDROM_READ_TOC,
                           Nil,
                           0,
                           @TOC,
                           sizeof(TTOC),
                           @Returned,
                           NiL) then
       begin
       Result:=-1;
       exit;
       end;
    With TOC do
      Result:=LastTrack-FirstTrack+1;
  Finally
    CloseHandle(DriveHandle);
  end;
end;


Function NtDriveInfo(CopyDrives : Boolean;Var CDDrives : Array of string): Integer;

var
  I : Integer;
  Drives : Array[0..105] of char;
  P : PChar;

begin
  FillChar(Drives,SizeOf(Drives),0);
  GetLogicalDriveStrings(105,Drives);
  P:=@Drives[0];
  Result:=0;
  While P[0]<>#0 do
    begin
    If GetDriveType(p)=DRIVE_CDROM then
      begin
      If CopyDrives and (Result<High(CDDrives)) then
        CDDrives[Result]:=Upcase(P[0])+':';
      Inc(Result);
      end;
    P:=P+Strlen(P)+1;
    end;
end;

Function NTGetNumDrives: Integer;

Var A : Array[1..1] of string;

begin
  Result:=NTDriveInfo(False,A);
end;

Function ioctlEnumDrives(Var Drives : Array of string) : Integer;

begin
  result:=NTDriveInfo(True,Drives);
end;

{ ---------------------------------------------------------------------
  3. Generic
  ---------------------------------------------------------------------}

Function ReadTOC(Device : String; Var TOC : TTOc) : Integer;

begin
  Case CurrentAccessMethod of
    camNone  : Result:=-1;
    camASPI  : Result:=AspiReadTOC(Device,TOC);
    camSPTI  : Result:=SptiReadTOC(Device,TOC);
    camIOCTL : Result:=IOCTLReadTOC(Device,TOC);
  end;
end;

Function  GetCDAccessMethod : TCDAccessMethod;

begin
  Result:=CurrentAccessMethod;
end;

Procedure SetCDAccessMethod (Value : TCDAccessMethod);

begin
  CurrentAccessMethod:=Value;
end;


Function ASPIDriveInfo(CopyInfo : Boolean; Var Drives : Array of string) : Integer;

var
  sh : SRB_HAInquiry;
  sd : SRB_GDEVBlock;
  numAdapters, maxTgt : Byte;
  i, j, k : byte;
  idx : Integer;

begin
  Result:=0;
  numAdapters := AspiGetNumAdapters;
  if (numAdapters=0) then
    exit;
  For I:=0 to NumAdapters-1 do
    begin
    FillChar(sh,sizeof(sh),0);
    sh.SRB_Cmd := SC_HA_INQUIRY;
    sh.SRB_HaID := i;
    SendASPI32Command(LPSRB(@sh));
    if (sh.SRB_Status=SS_COMP) then
      begin
      maxTgt:=sh.HA_Unique[3];
      if (maxTgt=0) then
        maxTgt:=MAXTARG;
      For J:=0 to Maxtgt-1 do
        For k:=0 to MAXLUN-1 do
          begin
          FillChar(sd,sizeof(sd),0);
          sd.SRB_Cmd    := SC_GET_DEV_TYPE;
          sd.SRB_HaID   := i;
          sd.SRB_Target := j;
          sd.SRB_Lun    := k;
          SendASPI32Command(LPSRB(@sd));
          if (sd.SRB_Status=SS_COMP) and
             (sd.SRB_DeviceType=DTYPE_CDROM) then
            begin
            If CopyInfo and (Result<High(Drives)) then
              Drives[Result]:=Format('ASPI[%d;%d;%d]',[I,J,K]);
            Inc(Result);
            end;
          end;
      end;
    end;
end;

Function ASPIGetNumDrives: Integer;

Var
  A : Array[1..1] of string;

begin
  Result:=AspiDriveInfo(False,A);
end;


Function GetNumDrives : Integer;

begin
 If CurrenTAccessMethod=camASPI then
   Result:=AspiGetNumDrives
 else
   Result:=NTGetNumDrives;
end;

Function EnumCDDrives(Var Drives : Array of String) : Integer;

begin
 If CurrenTAccessMethod=camASPI then
   Result:=AspiDriveInfo(True,Drives)
 else
   Result:=ioctlEnumDrives(Drives);
end;

Initialization
  InitWinCD;
end.
