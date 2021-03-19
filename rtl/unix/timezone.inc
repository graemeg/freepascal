{

  Support for timezone info in /usr/share/timezone
}

type

  ttzhead=packed record
    tzh_identifier : array[0..3] of AnsiChar;
    tzh_version : AnsiChar;
    tzh_reserved : array[0..14] of byte;
    tzh_ttisgmtcnt,
    tzh_ttisstdcnt,
    tzh_leapcnt,
    tzh_timecnt,
    tzh_typecnt,
    tzh_charcnt  : longint;
  end;

  pttinfo=^tttinfo;
  tttinfo=packed record
    offset : longint;
    isdst  : boolean;
    idx    : byte;
    isstd  : byte;
    isgmt  : byte;
  end;

  pleap=^tleap;
  tleap=record
    transition : int64;
    change     : longint;
  end;

var
  num_transitions,
  num_leaps,
  num_types    : longint;

  transitions  : PInt64 = nil;
  type_idxs    : pbyte = Nil;
  types        : pttinfo = Nil;
  zone_names   : pchar = Nil;
  leaps        : pleap = Nil;

function find_transition(timer:int64;timerIsUTC:Boolean;var trans_start,trans_end:int64):pttinfo;
var
  i,L,R,CompareRes : longint;
  found : boolean;

  function DoCompare: longint;
  var
    timerUTC: int64;
  begin
    if not timerIsUTC then
      timerUTC:=timer-types[type_idxs[i-1]].offset
    else
      timerUTC:=timer;
    if timerUTC<transitions[i-1] then
      Exit(-1)
    else
    if timerUTC>=transitions[i] then
      Exit(1)
    else
      Exit(0);
  end;
var
  timerLoUTC, timerHiUTC: int64;
begin
  if (num_transitions>0) and not timerIsUTC then
   begin
     timerLoUTC:=timer-types[type_idxs[0]].offset;
     timerHiUTC:=timer-types[type_idxs[num_transitions-1]].offset;
   end
  else
   begin
     timerLoUTC:=timer;
     timerHiUTC:=timer;
   end;

  if (num_transitions=0) or (timerLoUTC<transitions[0]) then
   { timer is before the first transition }
   begin
     i:=0;
     while (i<num_types) and (types[i].isdst) do
      inc(i);
     if (i=num_types) then
      i:=0;
     { unknown transition boundaries }
     trans_start:=low(trans_start);
     trans_end:=high(trans_end);
   end
  else
  if (num_transitions>0) and (timerHiUTC>=transitions[num_transitions-1]) then
   { timer is after the last transition }
   begin
     i:=type_idxs[num_transitions-1];
     trans_start:=transitions[num_transitions-1];
     trans_end:=high(trans_end);
   end
  else
   { timer inbetween }
   begin
      // Use binary search.
      L := 1;
      R := num_transitions-1;
      found := false;
      while not found and (L<=R) do
      begin
        I := L + (R - L) div 2;
        CompareRes := DoCompare;
        if (CompareRes>0) then
          L := I+1
        else begin
          R := I-1;
          if (CompareRes=0) then
             found:=true; // break cycle
        end;
      end;
     if not found then
       Exit(nil);
     trans_start:=transitions[i-1];
     trans_end:=transitions[i];
     i:=type_idxs[i-1];
   end;
  find_transition:=@types[i];
end;


procedure DoGetLocalTimezone(info:pttinfo;const trans_start,trans_end:int64;var ATZInfo:TTZInfo);
begin
  ATZInfo.validsince:=trans_start;
  ATZInfo.validuntil:=trans_end;
  ATZInfo.Daylight:=info^.isdst;
  ATZInfo.Seconds:=info^.offset;
end;

procedure DoGetLocalTimezoneEx(timer:int64;info:pttinfo;var ATZInfoEx:TTZInfoEx);
var
  i : longint;
  names: array[Boolean] of pchar;
begin
  names[true]:=nil;
  names[false]:=nil;
  ATZInfoEx.leap_hit:=0;
  ATZInfoEx.leap_correct:=0;

  i:=0;
  while (i<num_types) do
   begin
     names[types[i].isdst]:=@zone_names[types[i].idx];
     inc(i);
   end;
  names[info^.isdst]:=@zone_names[info^.idx];
  ATZInfoEx.name[true]:=names[true];
  ATZInfoEx.name[false]:=names[false];
  i:=num_leaps;
  repeat
    if i=0 then
     exit;
    dec(i);
  until (timer>leaps[i].transition);
  ATZInfoEx.leap_correct:=leaps[i].change;
  if (timer=leaps[i].transition) and
     (((i=0) and (leaps[i].change>0)) or
      (leaps[i].change>leaps[i-1].change)) then
   begin
     ATZInfoEx.leap_hit:=1;
     while (i>0) and
           (leaps[i].transition=leaps[i-1].transition+1) and
           (leaps[i].change=leaps[i-1].change+1) do
      begin
        inc(ATZInfoEx.leap_hit);
        dec(i);
      end;
   end;
end;

function GetLocalTimezone(timer:int64;timerIsUTC:Boolean;var ATZInfo:TTZInfo):Boolean;
var
  info: pttinfo;
  trans_start,trans_end,timerUTC: int64;
begin
  { check if time is in current global Tzinfo }
  ATZInfo:=CurrentTZinfo[InterlockedExchangeAdd(CurrentTZindex, 0)];
  if not timerIsUTC then
    timerUTC:=timer-ATZInfo.seconds
  else
    timerUTC:=timer;
  if (ATZInfo.validsince<=timerUTC) and (timerUTC<ATZInfo.validuntil) then
    Exit(True);

  LockTZInfo;
  info:=find_transition(timer,timerIsUTC,trans_start,trans_end);
  GetLocalTimezone:=assigned(info);
  if GetLocalTimezone then
    DoGetLocalTimezone(info,trans_start,trans_end,ATZInfo);
  UnlockTZInfo;
end;

function GetLocalTimezone(timer:int64;timerIsUTC:Boolean;var ATZInfo:TTZInfo;var ATZInfoEx:TTZInfoEx):Boolean;
var
  info: pttinfo;
  trans_start,trans_end,timerUTC: int64;
begin
  { check if time is in current global Tzinfo }
  ATZInfo:=CurrentTZinfo[InterlockedExchangeAdd(CurrentTZindex, 0)];
  if not timerIsUTC then
    timerUTC:=timer-ATZInfo.seconds
  else
    timerUTC:=timer;
  if (ATZInfo.validsince<=timerUTC) and (timerUTC<ATZInfo.validuntil) then
    begin
    ATZInfoEx:=TZInfoEx;
    Exit(True);
    end;

  { not current - search through all }
  LockTZInfo;
  info:=find_transition(timer,timerIsUTC,trans_start,trans_end);
  GetLocalTimezone:=assigned(info);
  if GetLocalTimezone then
    begin
    DoGetLocalTimezone(info,trans_start,trans_end,ATZInfo);
    DoGetLocalTimezoneEx(timer,info,ATZInfoEx);
    end;
  UnlockTZInfo;
end;

procedure RefreshTZInfo;
var
  NewTZInfo: TTZInfo;
  NewTZInfoEx: TTZInfoEx;
begin
  LockTZInfo;
  if GetLocalTimezone(fptime,true,NewTZInfo,NewTZInfoEx) then
    SetTZInfo(NewTZInfo,NewTZInfoEx);
  UnlockTZInfo;
end;

Const
  DefaultTimeZoneDir = '/usr/share/zoneinfo';

function TimeZoneDir : ShortString;

begin
  // Observe TZDIR environment variable.
  TimeZoneDir:=fpgetenv('TZDIR');
  if TimeZoneDir='' then
    TimeZoneDir:=DefaultTimeZoneDir;
  if TimeZoneDir[length(TimeZoneDir)]<>'/' then
    TimeZoneDir:=TimeZoneDir+'/';
end;

function ReadTimezoneFile(fn:string) : Boolean;

  function decode(const l:longint):longint;
  begin
    {$IFDEF ENDIAN_LITTLE}
    decode:=SwapEndian(l);
    {$ELSE}
    decode:=l;
    {$ENDIF}
  end;

  function decode(const l:int64):int64;
  begin
    {$IFDEF ENDIAN_LITTLE}
    decode:=SwapEndian(l);
    {$ELSE}
    decode:=l;
    {$ENDIF}
  end;

const
  bufsize = 2048;
var
  buf    : array[0..bufsize-1] of byte;
  bufptr : pbyte;
  bufbytes : tsSize;
  bufoverflow : boolean;
  f      : longint;
  tzhead : ttzhead;

  function readfilebuf : TsSize;
  begin
    bufptr := @buf[0];
    bufbytes:=fpread(f, buf, bufsize);
    readfilebuf:=bufbytes;
  end;

  Procedure checkbufptr(asize : integer);
  
  var
    a : tssize;
    
  begin
    a:=bufptr-@buf+asize;
    if (a>bufbytes) then
      bufoverflow:=true;
  end;

  function readbufbyte: byte;
  begin
    if bufptr > @buf[bufsize-1] then
      readfilebuf;
    checkbufptr(1);
    readbufbyte := bufptr^;
    inc(bufptr);
  end;

  function readbuf(dest:pointer; count: integer): integer;
  var
    numbytes: integer;
  begin
    readbuf := 0;
    repeat
      numbytes := (@buf[bufsize-1] + 1) - bufptr;
      if numbytes > count then
        numbytes := count;
      if numbytes > 0 then
      begin
        checkbufptr(numbytes);
        if assigned(dest) then
          move(bufptr^, dest^, numbytes);
        inc(bufptr, numbytes);
        dec(count, numbytes);
        inc(readbuf, numbytes);
        inc(dest, numbytes);
      end;
      if count > 0 then
        readfilebuf
      else
        break;
    until false;
  end;

  function readheader: boolean;
  var
    i      : longint;
  begin
    i:=readbuf(@tzhead,sizeof(tzhead));
    if i<>sizeof(tzhead) then
      exit(False);
    tzhead.tzh_timecnt:=decode(tzhead.tzh_timecnt);
    tzhead.tzh_typecnt:=decode(tzhead.tzh_typecnt);
    tzhead.tzh_charcnt:=decode(tzhead.tzh_charcnt);
    tzhead.tzh_leapcnt:=decode(tzhead.tzh_leapcnt);
    tzhead.tzh_ttisstdcnt:=decode(tzhead.tzh_ttisstdcnt);
    tzhead.tzh_ttisgmtcnt:=decode(tzhead.tzh_ttisgmtcnt);
    readheader:=(tzhead.tzh_identifier[0]='T') and (tzhead.tzh_identifier[1]='Z')
      and (tzhead.tzh_identifier[2]='i') and (tzhead.tzh_identifier[3]='f');
  end;

  procedure AllocFields;
  begin
    num_transitions:=tzhead.tzh_timecnt;
    num_types:=tzhead.tzh_typecnt;
    num_leaps:=tzhead.tzh_leapcnt;
    reallocmem(transitions,num_transitions*sizeof(int64));
    reallocmem(type_idxs,num_transitions);
    reallocmem(types,num_types*sizeof(tttinfo));
    reallocmem(zone_names,tzhead.tzh_charcnt);
    reallocmem(leaps,num_leaps*sizeof(tleap));
  end;

  function readdata: boolean;
  var
    i      : longint;
    longval: longint;
    version: longint;
  begin
    if tzhead.tzh_version='2' then
      begin
        version:=2;
        // skip version 0
        readbuf(nil,
           tzhead.tzh_timecnt*4  // transitions
          +tzhead.tzh_timecnt    // type_idxs
          +tzhead.tzh_typecnt*6  // types
          +tzhead.tzh_charcnt    // zone_names
          +tzhead.tzh_leapcnt*8  // leaps
          +tzhead.tzh_ttisstdcnt // isstd
          +tzhead.tzh_ttisgmtcnt // isgmt
          );
        readheader; // read version 2 header
        if tzhead.tzh_version<>'2' then
          Exit(False);
      end
    else
      version:=0;

    AllocFields;

    if version=2 then
      begin // read 64bit values
        readbuf(transitions,num_transitions*sizeof(int64));
        for i:=0 to num_transitions-1 do
          transitions[i]:=decode(transitions[i]);
      end
    else
      begin // read 32bit values
        for i:=0 to num_transitions-1 do
         begin
           readbuf(@longval,sizeof(longval));
           transitions[i]:=decode(longval);
         end;
      end;
    readbuf(type_idxs,num_transitions);

    for i:=0 to num_types-1 do
     begin
       readbuf(@types[i].offset,sizeof(LongInt));
       types[i].offset:=decode(types[i].offset);
       readbuf(@types[i].isdst,1);
       readbuf(@types[i].idx,1);
       types[i].isstd:=0;
       types[i].isgmt:=0;
     end;

    readbuf(zone_names,tzhead.tzh_charcnt);
    if version=2 then
      begin // read 64bit values
        for i:=0 to num_leaps-1 do
         begin
           readbuf(@leaps[i].transition,sizeof(int64));
           readbuf(@leaps[i].change,sizeof(longint));
           leaps[i].transition:=decode(leaps[i].transition);
           leaps[i].change:=decode(leaps[i].change);
         end;
      end
    else
      begin
        for i:=0 to num_leaps-1 do
         begin
           readbuf(@longval,sizeof(longval));
           leaps[i].transition:=decode(longval);
           readbuf(@longval,sizeof(longval));
           leaps[i].change:=decode(longval);
         end;
      end;

    for i:=0 to tzhead.tzh_ttisstdcnt-1 do
     types[i].isstd:=byte(readbufbyte<>0);

    for i:=0 to tzhead.tzh_ttisgmtcnt-1 do
     types[i].isgmt:=byte(readbufbyte<>0);

    readdata:=true;
  end;
  procedure ClearCurrentTZinfo;
  var
    i:integer;
  begin
    for i:=low(CurrentTZinfo) to high(CurrentTZinfo) do
      CurrentTZinfo[i] := Default(TTZInfo);
  end;
begin
  if fn='' then
   fn:='localtime';
  if fn[1]<>'/' then
    fn:='/usr/share/zoneinfo/'+fn;
  f:=fpopen(fn,Open_RdOnly);
  if f<0 then
   exit(False);
  bufoverflow:=false;
  bufptr := @buf[bufsize-1]+1;
  tzhead:=default(ttzhead);
  LockTZInfo;
  ReadTimezoneFile:=(readheader() and readdata()) and not BufOverflow;
  ClearCurrentTZinfo;
  UnlockTZInfo;
  fpclose(f);
end;

Const
  // Debian system; contains location of timezone file.
  TimeZoneLocationFile = '/etc/timezone';
  // SuSE has link in /usr/lib/zoneinfo/localtime to /etc/localtime
  // RedHat uses /etc/localtime

  TimeZoneFile = '/etc/localtime';                      // POSIX
  AltTimeZoneFile = '/usr/lib/zoneinfo/localtime';      // Other
  iOSTimeZoneFile = '/var/db/timezone/localtime';      // iOS
{$ifdef BSD}
  BSDTimeZonefile = DefaultTimeZoneDir;      // BSD usually is POSIX
                                                // compliant though
{$ENDIF}

{$ifndef FPC_HAS_GETTIMEZONEFILE}

function GetTimezoneFile:shortstring;
var
  f,len : longint;
  fn,s : shortstring;
  info : stat;
begin
  GetTimezoneFile:='';
  // Observe TZ variable.
  fn:=fpgetenv('TZ');
  if (fn<>'') then
    if (fn[1]=':') then
      begin
      Delete(fn,1,1);
      if (fn<>'') then
        begin
        if (fn[1]<>'/') then
          Exit(TimeZoneDir+fn);
        Exit(fn);
        end;
      end;
  if (fn='') then
    fn:=TimeZoneLocationFile;
  f:=fpopen(TimeZoneLocationFile,Open_RdOnly);
  if f>0 then
    begin
    len:=fpread(f,s[1],high(s));
    s[0]:=chr(len);
    len:=pos(#10,s);
    if len<>0 then
     s[0]:=chr(len-1);
    fpclose(f);
    GetTimezoneFile:=s;
    end
  // Try SuSE
  else if fpstat(TimeZoneFile,{$ifdef oldlinuxstat}baseunix.stat(info){$else}info{$endif})>=0 then
    GetTimeZoneFile:=TimeZoneFile
  // Try RedHat
  else If fpstat(AltTimeZoneFile,{$ifdef oldlinuxstat}baseunix.stat(info){$else}info{$endif})>=0 then
      GetTimeZoneFile:=AltTimeZoneFile
{$ifdef BSD}
//  else
//       If fpstat(BSDTimeZoneFile,{$ifdef oldlinuxstat}baseunix.stat(info){$else}info{$endif})>=0 then
// GetTimeZoneFile:=BSDTimeZoneFile
{$ENDIF}
{$if (defined(darwin) and defined(arm)) or defined(iphonesim)}
  else If fpstat(iOSTimeZoneFile,info)>=0 then
      GetTimeZoneFile:=iOSTimeZoneFile
{$endif}
end;

{$endif ndef FPC_HAS_GETTIMEZONEFILE}

procedure InitLocalTime;
begin
  ReadTimezoneFile(GetTimezoneFile);
  RefreshTZInfo;
end;


procedure DoneLocalTime;
begin
  if assigned(transitions) then
   freemem(transitions);
  transitions:=nil;
  if assigned(type_idxs) then
   freemem(type_idxs);
  type_idxs:=nil;
  if assigned(types) then
   freemem(types);
  types:=nil;
  if assigned(zone_names) then
   freemem(zone_names);
  zone_names:=Nil;
  if assigned(leaps) then
   freemem(leaps);
  leaps:=nil;
  num_transitions:=0;
  num_leaps:=0;
  num_types:=0;
end;

Procedure ReReadLocalTime;

begin
  LockTZInfo;
  DoneLocalTime;
  InitLocalTime;
  UnlockTZInfo;
end;
