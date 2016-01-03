{$mode objfpc}
{$H+}
unit macuuid;

Interface

uses SysUtils;

Function CreateMacGUID(Out GUID : TGUID) : Integer;


Implementation

uses unixtype, sockets, baseunix, unix;

Const 
  MAX_ADJUSTMENT = 10;
  IPPROTO_IP     = 0;
//  AF_INET        = 2;
//  SOCK_DGRAM     = 2; 
  IF_NAMESIZE    = 16;
  SIOCGIFCONF    = $8912;
  SIOCGIFHWADDR  = $8927;
  
Type
  {$packrecords c}
  tifr_ifrn = record
    case integer of
      0 : (ifrn_name: array [0..IF_NAMESIZE-1] of char);
  end;
  tifmap = record
    mem_start : cardinal;
    mem_end   : cardinal;
    base_addr : word;
    irq       : byte;
    dma       : byte;
    port      : byte;
  end;
  PIFrec = ^TIFrec;
  TIFrec = record
    ifr_ifrn : tifr_ifrn;
    case integer of
      0 : (ifru_addr      : TSockAddr);
      1 : (ifru_dstaddr   : TSockAddr);
      2 : (ifru_broadaddr : TSockAddr);
      3 : (ifru_netmask   : TSockAddr);
      4 : (ifru_hwaddr    : TSockAddr);
      5 : (ifru_flags     : word); 
      6 : (ifru_ivalue    : longint);
      7 : (ifru_mtu       : longint);
      8 : (ifru_map       : tifmap);
      9 : (ifru_slave     : Array[0..IF_NAMESIZE-1] of char);
      10 : (ifru_newname  : Array[0..IF_NAMESIZE-1] of char);
      11 : (ifru_data     : pointer);
  end; 
  TIFConf = record
    ifc_len : longint;
    case integer of
      0 : (ifcu_buf : pointer);
      1 : (ifcu_req : ^tifrec);
  end;

  tuuid = record 
    time_low : cardinal;
    time_mid : Word;
    time_hi_and_version : Word;
    clock_seq : Word;
    node : Array[0..5] of byte;
  end;

Var
  MacAddr      : Packed Array[1..6] of byte = (0,0,0,0,0,0);
  MacAddrTried : Byte = 0 ;
  Last   : TTimeVal = (tv_sec:0;tv_usec:0);
  ClockSeq   : Word = 0;
  AdjustMent : Integer = 0;

Procedure GetRandomBytes(Var Buf; NBytes : Integer);

Var
  I : Integer;
  P : PByte;

begin
  P:=@Buf;
  Randomize;
  For I:=0 to NBytes-1 do
    P[i]:=Random(256);
end;
  
Function GetMacAddr : Boolean;

var
  i,j,n,Sd : Integer;
  buf : Array[0..1023] of byte;
  ifc : TIfConf;
  ifr : TIFRec;
  ifp : PIFRec;
  p   : PChar;
begin
  Result:=MacAddrTried>0;
  If Result then
    Result:=MacAddrTried>1
  else  
    begin
    MacAddrTried:=1;
    sd:=fpSocket(AF_INET,SOCK_DGRAM,IPPROTO_IP);
    if (sd<0) then 
      exit;
    Try
      ifc.ifc_len:=Sizeof(Buf);
      ifc.ifcu_buf:=@buf;
      if fpioctl(sd, SIOCGIFCONF, @ifc)<0 then
        Exit;
      n:= ifc.ifc_len;  
      i:=0;
      While (Not Result) and (I<N) do
        begin
        ifp:=PIFRec(PByte(ifc.ifcu_buf)+i);
        move(ifp^.ifr_ifrn.ifrn_name,ifr.ifr_ifrn.ifrn_name,IF_NAMESIZE);
        if (fpioctl(sd, SIOCGIFHWADDR, @ifr) >= 0) then
          begin
          P:=Pchar(@ifr.ifru_hwaddr.sa_data);
          Result:=(p[0]<>#0) or (p[1]<>#0) or (p[2]<>#0) 
                  or (p[3]<>#0) or (p[4]<>#0) or (p[5]<>#0);
          If Result Then
            begin
            Move(P^,MacAddr,SizeOf(MacAddr));  
            MacAddrTried:=2;
            // DumpMacAddr;
            end;
          end;
        I:=I+sizeof(tifrec);
        end;
    Finally  
      fileClose(sd);
    end;
    end;
end;

  
Function GetClock(Var ClockHigh,ClockLow : Cardinal; Var RetClockSeq : Word) : boolean;

Var
  TV       : TTImeVal;
  ClockReg : QWord;  
  OK       : Boolean; 

begin
  OK:=True;
  Repeat
    FPGetTimeOfDay(@Tv,Nil);
    If (Last.tv_sec=0) and (last.tv_sec=0) then
      begin
      GetRandomBytes(ClockSeq,SizeOf(ClockSeq));
      ClockSeq:=ClockSeq and $1FFF;
      last:=TV;
      Dec(last.tv_sec);
      end;
    if (tv.tv_sec<last.tv_sec) or 
        ((tv.tv_sec=last.tv_sec) and (tv.tv_usec<last.tv_usec)) then
      begin
      ClockSeq:=(ClockSeq+1) and $1FFF;
      Adjustment:=0;
      Last:=Tv;
      end
    else if (tv.tv_sec=last.tv_sec) and (tv.tv_usec=last.tv_usec) then
      begin
      If Adjustment>=MAX_ADJUSTMENT then
        OK:=False
      else  
        inc(AdjustMent);
      end
    else
      begin
      AdjustMent:=0;
      Last:=tv;
      end;
  Until OK;  
  ClockReg:=tv.tv_usec*10+adjustment;
  Inc(ClockReg,tv.tv_sec*10000000);
  Inc(ClockReg,($01B21DD2 shl 32) + $13814000);
  ClockHigh   :=Hi(ClockReg);
  ClockLow    :=Lo(ClockReg);
  RetClockSeq :=ClockSeq;
  Result      :=True;                  
end;

Procedure UUIDPack(Const UU : TUUID; Var GUID : TGUID);

Var
  tmp : Cardinal;
  P   : PByte;
  
begin
  P:=PByte(@GUID);
  
  tmp:=uu.time_low;
  P[3]:=tmp and $FF;
  tmp:=tmp shr 8;
  P[2]:=tmp and $FF;
  tmp:=tmp shr 8;
  P[1]:=tmp and $FF;
  tmp:=tmp shr 8;
  P[0]:=tmp and $FF;
  
  tmp:=uu.time_mid;
  P[5]:=tmp and $FF;
  tmp:=tmp shr 8;
  P[4]:=tmp and $FF;
  
  tmp:=uu.time_hi_and_version;
  P[7]:=tmp and $FF;
  tmp:=tmp shr 8;
  P[6]:=tmp and $FF;
  
  tmp:=uu.clock_seq;
  P[9]:=tmp and $FF;
  tmp:=tmp shr 8;
  P[8]:=tmp and $FF;
  
  Move(uu.node,P[10],6);
end;

Procedure DumpMacAddr;

var
  I : Integer;
begin
  Write('Mac Addr: ');
  For i:=1 to 6 do
    write(hexstr(MacAddr[i],2),':');
end;

Function CreateMacGUID(Out GUID : TGUID) : Integer;

Var
  UU       : TUUId;
  ClockMid : Cardinal;

begin
  Result:=Ord(not GetMacAddr);
  If (Result=0) then
    begin
    // DumpMacAddr;
    // Writeln;
    GetClock(ClockMid,uu.time_low,uu.clock_seq);
    uu.Clock_seq:=uu.Clock_seq or $8000;
    uu.time_mid:=lo(clockMid);
    uu.time_hi_and_version:=hi(ClockMid) or $1000;
    move(MacAddr,uu.node,sizeof(MacAddr));
    UUIDPack(UU,GUID);
    end;
end;

initialization
  OnCreateGUID:=@CreateMacGUID;
end.
