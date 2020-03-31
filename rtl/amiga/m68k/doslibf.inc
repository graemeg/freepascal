{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 Free Pascal development team

    dos functions (V40) for Amiga/68k

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{ dos.library functions }

function Open(fname     : PChar   location 'd1';
              accessMode: LongInt location 'd2'): LongInt;
SysCall AOS_DOSBase 30;

function dosClose(fileh: LongInt location 'd1'): LongBool;
SysCall AOS_DOSBase 36;

function dosRead(fileh : LongInt location 'd1';
                 buffer: Pointer location 'd2';
                 length: LongInt location 'd3'): LongInt;
SysCall AOS_DOSBase 42;

function dosWrite(fileh : LongInt location 'd1';
                  buffer: Pointer location 'd2';
                  length: LongInt location 'd3'): LongInt;
SysCall AOS_DOSBase 48;

function dosInput: LongInt;
SysCall AOS_DOSBase 54;

function dosOutput: LongInt;
SysCall AOS_DOSBase 60;

function dosSeek(fileh   : LongInt location 'd1';
                 position: LongInt location 'd2';
                 posmode : LongInt location 'd3'): LongInt;
SysCall AOS_DOSBase 66;

function dosDeleteFile(fname: PChar location 'd1'): LongBool;
SysCall AOS_DOSBase 72;

function dosRename(oldName: PChar location 'd1';
                   newName: PChar location 'd2'): LongInt;
SysCall AOS_DOSBase 78;

function Lock(lname     : PChar   location 'd1';
              accessMode: LongInt location 'd2'): LongInt;
SysCall AOS_DOSBase 84;

procedure Unlock(lock: LongInt location 'd1');
SysCall AOS_DOSBase 90;

function DupLock(lock: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 096;

function Examine(lock         : LongInt        location 'd1';
                 fileInfoBlock: PFileInfoBlock location 'd2'): LongInt;
SysCall AOS_DOSBase 102;

function ExNext(lock         : LongInt        location 'd1';
                fileInfoBlock: PFileInfoBlock location 'd2'): LongInt;
SysCall AOS_DOSBase 108;

function Info(lock          : LongInt   location 'd1';
              parameterBlock: PInfoData location 'd2'): LongInt;
SysCall AOS_DOSBase 114;

function dosCreateDir(dname: PChar location 'd1'): LongInt;
SysCall AOS_DOSBase 120;

function CurrentDir(lock: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 126;

function IoErr: LongInt;
SysCall AOS_DOSBase 132;

function CreateProc(name     : PChar   location 'd1';
                    pri      : LongInt location 'd2';
                    segList  : LongInt location 'd3';
                    stackSize: LongInt location 'd4'): PMsgPort;
SysCall AOS_DOSBase 138;

procedure dosExit(returnCode: LongInt location 'd1');
SysCall AOS_DOSBase 144;

function LoadSeg(name: PChar location 'd1'): LongInt;
SysCall AOS_DOSBase 150;

procedure UnLoadSeg(seglist: LongInt location 'd1');
SysCall AOS_DOSBase 156;

function DeviceProc(name: PChar location 'd1'): PMsgPort;
SysCall AOS_DOSBase 174;

function SetComment(name   : PChar location 'd1';
                    comment: PChar location 'd2'): LongBool;
SysCall AOS_DOSBase 180;

function SetProtection(name: PChar   location 'd1';
                       mask: LongInt location 'd2'): LongInt;
SysCall AOS_DOSBase 186;

function DateStamp(date: PDateStamp location 'd1'): PDateStamp;
SysCall AOS_DOSBase 192;

procedure DOSDelay(timeout: LongInt location 'd1');
SysCall AOS_DOSBase 198;

function WaitForChar(file1  : LongInt location 'd1';
                     timeout: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 204;

function ParentDir(lock: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 210;

function IsInteractive(file1: LongInt location 'd1'): LongBool;
SysCall AOS_DOSBase 216;

function Execute(string1: PChar   location 'd1';
                 file1  : LongInt location 'd2';
                 file2  : LongInt location 'd3'): LongBool;
SysCall AOS_DOSBase 222;

// these functions are only available v36+ (OS Release 2.0+)
{$IFNDEF AMIGA_V1_0_ONLY}
{$IFNDEF AMIGA_V1_2_ONLY}

function AllocDosObject(type1: Cardinal location 'd1';
                        tags : PTagItem location 'd2'): Pointer;
SysCall AOS_DOSBase 228;

function AllocDosObjectTagList(type1: Cardinal location 'd1';
                               tags : PTagItem location 'd2'): Pointer;
SysCall AOS_DOSBase 228;

procedure FreeDosObject(type1: Cardinal location 'd1';
                        ptr  : Pointer  location 'd2');
SysCall AOS_DOSBase 234;

function DoPkt(port  : PMsgPort location 'd1';
               action: LongInt  location 'd2';
               arg1  : LongInt  location 'd3';
               arg2  : LongInt  location 'd4';
               arg3  : LongInt  location 'd5';
               arg4  : LongInt  location 'd6';
               arg5  : LongInt  location 'd7'): LongInt;
SysCall AOS_DOSBase 240;

function DoPkt0(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2'): LongInt;
SysCall AOS_DOSBase 240;

function DoPkt1(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2';
                arg1  : LongInt  location 'd3'): LongInt;
SysCall AOS_DOSBase 240;

function DoPkt2(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2';
                arg1  : LongInt  location 'd3';
                arg2  : LongInt  location 'd4'): LongInt;
SysCall AOS_DOSBase 240;

function DoPkt3(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2';
                arg1  : LongInt  location 'd3';
                arg2  : LongInt  location 'd4';
                arg3  : LongInt  location 'd5'): LongInt;
SysCall AOS_DOSBase 240;

function DoPkt4(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2';
                arg1  : LongInt  location 'd3';
                arg2  : LongInt  location 'd4';
                arg3  : LongInt  location 'd5';
                arg4  : LongInt  location 'd6'): LongInt;
SysCall AOS_DOSBase 240;

procedure SendPkt(dp       : PDosPacket location 'd1';
                  port     : PMsgPort   location 'd2';
                  replyport: PMsgPort   location 'd3');
SysCall AOS_DOSBase 246;

function WaitPkt: PDosPacket;
SysCall AOS_DOSBase 252;

procedure ReplyPkt(dp  : PDosPacket location 'd1';
                   res1: LongInt    location 'd2';
                   res2: LongInt    location 'd3');
SysCall AOS_DOSBase 258;

procedure AbortPkt(port: PMsgPort   location 'd1';
                   pkt : PDosPacket location 'd2');
SysCall AOS_DOSBase 264;

function LockRecord(fh     : LongInt  location 'd1';
                    offset : Cardinal location 'd2';
                    length : Cardinal location 'd3';
                    mode   : Cardinal location 'd4';
                    timeout: Cardinal location 'd5'): LongBool;
SysCall AOS_DOSBase 270;

function LockRecords(recArray: PRecordLock location 'd1';
                     timeout : Cardinal    location 'd2'): LongBool;
SysCall AOS_DOSBase 276;

function UnLockRecord(fh    : LongInt  location 'd1';
                      offset: Cardinal location 'd2';
                      length: Cardinal location 'd3'): LongBool;
SysCall AOS_DOSBase 282;

function UnLockRecords(recArray: PRecordLock location 'd1'): LongBool;
SysCall AOS_DOSBase 288;

function SelectInput(fh: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 294;

function SelectOutput(fh: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 300;

function FGetC(fh: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 306;

function FPutC(fh: LongInt location 'd1';
               ch: LongInt location 'd2'): LongInt;
SysCall AOS_DOSBase 312;

function UnGetC(fh       : LongInt location 'd1';
                character: LongInt location 'd2'): LongInt;
SysCall AOS_DOSBase 318;

function FRead(fh      : LongInt  location 'd1';
               block   : Pointer  location 'd2';
               blocklen: Cardinal location 'd3';
               number  : Cardinal location 'd4'): LongInt;
SysCall AOS_DOSBase 324;

function FWrite(fh      : LongInt  location 'd1';
                block   : Pointer  location 'd2';
                blocklen: Cardinal location 'd3';
                number  : Cardinal location 'd4'): LongInt;
SysCall AOS_DOSBase 330;

function FGets(fh    : LongInt  location 'd1';
               buf   : PChar    location 'd2';
               buflen: Cardinal location 'd3'): PChar;
SysCall AOS_DOSBase 336;

function FPuts(fh : LongInt location 'd1';
               str: PChar   location 'd2'): LongInt;
SysCall AOS_DOSBase 342;

procedure VFWritef(fh      : LongInt location 'd1';
                   format  : PChar   location 'd2';
                   argarray: Pointer location 'd3');
SysCall AOS_DOSBase 348;

function VFPrintf(fh      : LongInt location 'd1';
                  format  : PChar   location 'd2';
                  argarray: Pointer location 'd3'): LongInt;
SysCall AOS_DOSBase 354;

function dosFlush(fh: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 360;

function SetVBuf(fh   : LongInt location 'd1';
                 buff : PChar   location 'd2';
                 type1: LongInt location 'd3';
                 size : LongInt location 'd4'): LongInt;
SysCall AOS_DOSBase 366;

function DupLockFromFH(fh: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 372;

function OpenFromLock(lock: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 378;

function ParentOfFH(fh: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 384;

function ExamineFH(fh : BPTR           location 'd1';
                   fib: PFileInfoBlock location 'd2'): LongBool;
SysCall AOS_DOSBase 390;

function SetFileDate(name: PChar      location 'd1';
                     date: PDateStamp location 'd2'): LongBool;
SysCall AOS_DOSBase 396;

function NameFromLock(lock  : LongInt location 'd1';
                      buffer: PChar   location 'd2';
                      len   : LongInt location 'd3'): LongBool;
SysCall AOS_DOSBase 402;

function NameFromFH(fh    : BPTR    location 'd1';
                    buffer: PChar   location 'd2';
                    len   : LongInt location 'd3'): LongBool;
SysCall AOS_DOSBase 408;

function SplitName(name     : PChar    location 'd1';
                   separator: Cardinal location 'd2';
                   buf      : PChar    location 'd3';
                   oldpos   : LongInt  location 'd4';
                   size     : LongInt  location 'd5'): SmallInt;
SysCall AOS_DOSBase 414;

function SameLock(lock1: LongInt location 'd1';
                  lock2: LongInt location 'd2'): LongInt;
SysCall AOS_DOSBase 420;

function SetMode(fh  : LongInt location 'd1';
                 mode: LongInt location 'd2'): LongInt;
SysCall AOS_DOSBase 426;

function ExAll(lock   : LongInt       location 'd1';
               buffer : PExAllData    location 'd2';
               size   : LongInt       location 'd3';
               data   : LongInt       location 'd4';
               control: PExAllControl location 'd5'): LongBool;
SysCall AOS_DOSBase 432;

function ReadLink(port  : PMsgPort location 'd1';
                  lock  : LongInt  location 'd2';
                  path  : PChar    location 'd3';
                  buffer: PChar    location 'd4';
                  size  : Cardinal location 'd5'): LongBool;
SysCall AOS_DOSBase 438;

function MakeLink(name: PChar   location 'd1';
                  dest: LongInt location 'd2';
                  soft: LongInt location 'd3'): LongBool;
SysCall AOS_DOSBase 444;

function ChangeMode(type1  : LongInt location 'd1';
                    fh     : LongInt location 'd2';
                    newmode: LongInt location 'd3'): LongBool;
SysCall AOS_DOSBase 450;

function SetFileSize(fh  : LongInt location 'd1';
                     pos : LongInt location 'd2';
                     mode: LongInt location 'd3'): LongInt;
SysCall AOS_DOSBase 456;

function SetIoErr(result: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 462;

function Fault(code  : LongInt location 'd1';
               header: PChar   location 'd2';
               buffer: PChar   location 'd3';
               len   : LongInt location 'd4'): LongBool;
SysCall AOS_DOSBase 468;

function PrintFault(code  : LongInt location 'd1';
                    header: PChar   location 'd2'): LongBool;
SysCall AOS_DOSBase 474;

function ErrorReport(code  : LongInt  location 'd1';
                     type1 : LongInt  location 'd2';
                     arg1  : Cardinal location 'd3';
                     device: PMsgPort location 'd4'): LongBool;
SysCall AOS_DOSBase 480;

function Cli: PCommandLineInterface;
SysCall AOS_DOSBase 492;

function CreateNewProc(tags: PTagItem location 'd1'): PProcess;
SysCall AOS_DOSBase 498;

function CreateNewProcTagList(tags: PTagItem location 'd1'): PProcess;
SysCall AOS_DOSBase 498;

function RunCommand(seg     : LongInt location 'd1';
                    stack   : LongInt location 'd2';
                    paramptr: PChar   location 'd3';
                    paramlen: LongInt location 'd4'): LongInt;
SysCall AOS_DOSBase 504;

function GetConsoleTask: PMsgPort;
SysCall AOS_DOSBase 510;

function SetConsoleTask(task: PMsgPort location 'd1'): PMsgPort;
SysCall AOS_DOSBase 516;

function GetFileSysTask: PMsgPort;
SysCall AOS_DOSBase 522;

function SetFileSysTask(task: PMsgPort location 'd1'): PMsgPort;
SysCall AOS_DOSBase 528;

function GetArgStr: PChar;
SysCall AOS_DOSBase 534;

function SetArgStr(str: PChar location 'd1'): LongBool;
SysCall AOS_DOSBase 540;

function FindCliProc(num: Cardinal location 'd1'): PProcess;
SysCall AOS_DOSBase 546;

function MaxCli: Cardinal;
SysCall AOS_DOSBase 552;

function SetCurrentDirName(name: PChar location 'd1'): LongBool;
SysCall AOS_DOSBase 558;

function GetCurrentDirName(buf: PChar   location 'd1';
                           len: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 564;

function SetProgramName(name: PChar location 'd1'): LongBool;
SysCall AOS_DOSBase 570;

function GetProgramName(buf: PChar   location 'd1';
                        len: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 576;

function SetPrompt(name: PChar location 'd1'): LongBool;
SysCall AOS_DOSBase 582;

function GetPrompt(buf: PChar   location 'd1';
                   len: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 588;

function SetProgramDir(lock: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 594;

function GetProgramDir: LongInt;
SysCall AOS_DOSBase 600;

function SystemTagList(command: PChar    location 'd1';
                       tags   : PTagItem location 'd2'): LongInt;
SysCall AOS_DOSBase 606;

function dosSystem(command: PChar    location 'd1';
                   tags   : PTagItem location 'd2'): LongInt;
SysCall AOS_DOSBase 606;

function AssignLock(name: PChar   location 'd1';
                    lock: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 612;

function AssignLate(name: PChar location 'd1';
                    path: PChar location 'd2'): LongBool;
SysCall AOS_DOSBase 618;

function AssignPath(name: PChar location 'd1';
                    path: PChar location 'd2'): LongBool;
SysCall AOS_DOSBase 624;

function AssignAdd(name: PChar   location 'd1';
                   lock: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 630;

function RemAssignList(name: PChar   location 'd1';
                       lock: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 636;

function GetDeviceProc(name: PChar    location 'd1';
                       dp  : PDevProc location 'd2'): PDevProc;
SysCall AOS_DOSBase 642;

procedure FreeDeviceProc(dp: PDevProc location 'd1');
SysCall AOS_DOSBase 648;

function LockDosList(flags: Cardinal location 'd1'): PDosList;
SysCall AOS_DOSBase 654;

procedure UnLockDosList(flags: Cardinal location 'd1');
SysCall AOS_DOSBase 660;

function AttemptLockDosList(flags: Cardinal location 'd1'): PDosList;
SysCall AOS_DOSBase 666;

function RemDosEntry(dlist: PDosList location 'd1'): LongBool;
SysCall AOS_DOSBase 672;

function AddDosEntry(dlist: PDosList location 'd1'): LongInt;
SysCall AOS_DOSBase 678;

function FindDosEntry(dlist: PDosList location 'd1';
                      name : PChar    location 'd2';
                      flags: Cardinal location 'd3'): PDosList;
SysCall AOS_DOSBase 684;

function NextDosEntry(dlist: PDosList location 'd1';
                      flags: Cardinal location 'd2'): PDosList;
SysCall AOS_DOSBase 690;

function MakeDosEntry(name : PChar   location 'd1';
                      type1: LongInt location 'd2'): PDosList;
SysCall AOS_DOSBase 696;

procedure FreeDosEntry(dlist: PDosList location 'd1');
SysCall AOS_DOSBase 702;

function IsFileSystem(name: PChar location 'd1'): LongBool;
SysCall AOS_DOSBase 708;

function Format(filesystem: PChar    location 'd1';
                volumename: PChar    location 'd2';
                dostype   : Cardinal location 'd3'): LongBool;
SysCall AOS_DOSBase 714;

function Relabel(drive  : PChar location 'd1';
                 newname: PChar location 'd2'): LongBool;
SysCall AOS_DOSBase 720;

function Inhibit(name : PChar   location 'd1';
                 onoff: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 726;

function AddBuffers(name  : PChar   location 'd1';
                    number: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 732;

function CompareDates(date1: PDateStamp location 'd1';
                      date2: PDateStamp location 'd2'): LongInt;
SysCall AOS_DOSBase 738;

function DateToStr(datetime: _PDateTime location 'd1'): LongBool;
SysCall AOS_DOSBase 744;

function StrToDate(datetime: _PDateTime location 'd1'): LongBool;
SysCall AOS_DOSBase 750;

function InternalLoadSeg(fh           : LongInt location 'd0';
                         table        : LongInt location 'a0';
                         var funcarray: LongInt location 'a1';
                         var stack    : LongInt location 'a2'): LongInt;
SysCall AOS_DOSBase 756;

function NewLoadSeg(file1: PChar    location 'd1';
                    tags : PTagItem location 'd2'): LongInt;
SysCall AOS_DOSBase 768;

function NewLoadSegTagList(file1: PChar    location 'd1';
                           tags : PTagItem location 'd2'): LongInt;
SysCall AOS_DOSBase 768;

function AddSegment(name  : PChar   location 'd1';
                    seg   : LongInt location 'd2';
                    system: LongInt location 'd3'): LongBool;
SysCall AOS_DOSBase 774;

function FindSegment(name  : PChar    location 'd1';
                     seg   : PSegment location 'd2';
                     system: LongInt  location 'd3'): PSegment;
SysCall AOS_DOSBase 780;

function RemSegment(seg: PSegment location 'd1'): LongBool;
SysCall AOS_DOSBase 786;

function CheckSignal(mask: LongInt location 'd1'): LongInt;
SysCall AOS_DOSBase 792;

function ReadArgs(arg_template: PChar   location 'd1';
                  var array1  : LongInt location 'd2';
                  args        : PRDArgs location 'd3'): PRDArgs;
SysCall AOS_DOSBase 798;

function FindArg(keyword     : PChar location 'd1';
                 arg_template: PChar location 'd2'): LongInt;
SysCall AOS_DOSBase 804;

function ReadItem(name    : PChar    location 'd1';
                  maxchars: LongInt  location 'd2';
                  cSource : PCSource location 'd3'): LongInt;
SysCall AOS_DOSBase 810;

function StrToLong(string1  : PChar   location 'd1';
                   var value: LongInt location 'd2'): LongInt;
SysCall AOS_DOSBase 816;

function MatchFirst(pat   : PChar       location 'd1';
                    anchor: PAnchorPath location 'd2'): LongInt;
SysCall AOS_DOSBase 822;

function MatchNext(anchor: PAnchorPath location 'd1'): LongInt;
SysCall AOS_DOSBase 828;

procedure MatchEnd(anchor: PAnchorPath location 'd1');
SysCall AOS_DOSBase 834;

function ParsePattern(pat   : PChar   location 'd1';
                      buf   : PChar   location 'd2';
                      buflen: LongInt location 'd3'): LongInt;
SysCall AOS_DOSBase 840;

function MatchPattern(pat: PChar location 'd1';
                      str: PChar location 'd2'): LongBool;
SysCall AOS_DOSBase 846;

procedure FreeArgs(args: pRDArgs location 'd1');
SysCall AOS_DOSBase 858;

function FilePart(path: PChar location 'd1'): PChar;
SysCall AOS_DOSBase 870;

function PathPart(path: PChar location 'd1'): PChar;
SysCall AOS_DOSBase 876;

function AddPart(dirname: PChar    location 'd1';
                filename: PChar    location 'd2';
                size    : Cardinal location 'd3'): LongBool;
SysCall AOS_DOSBase 882;

function StartNotify(notify: PNotifyRequest location 'd1'): LongBool;
SysCall AOS_DOSBase 888;

procedure EndNotify(notify: PNotifyRequest location 'd1');
SysCall AOS_DOSBase 894;

function SetVar(name  : PChar   location 'd1';
                buffer: PChar   location 'd2';
                size  : LongInt location 'd3';
                flags : LongInt location 'd4'): LongBool;
SysCall AOS_DOSBase 900;

function GetVar(name  : PChar   location 'd1';
                buffer: PChar   location 'd2';
                size  : LongInt location 'd3';
                flags : LongInt location 'd4'): LongInt;
SysCall AOS_DOSBase 906;

function DeleteVar(name : PChar    location 'd1';
                   flags: Cardinal location 'd2'): LongBool;
SysCall AOS_DOSBase 912;

function FindVar(name : PChar    location 'd1';
                 type1: Cardinal location 'd2'): PLocalVar;
SysCall AOS_DOSBase 918;

function CliInitNewcli(dp: PDosPacket location 'a0'): LongInt;
SysCall AOS_DOSBase 930;

function CliInitRun(dp: PDosPacket location 'a0'): LongInt;
SysCall AOS_DOSBase 936;

function WriteChars(buf   : PChar    location 'd1';
                    buflen: Cardinal location 'd2'): LongInt;
SysCall AOS_DOSBase 942;

function PutStr(str: PChar location 'd1'): LongInt;
SysCall AOS_DOSBase 948;

function VPrintf(format  : PChar   location 'd1';
                 argarray: Pointer location 'd2'): LongInt;
SysCall AOS_DOSBase 954;

function ParsePatternNoCase(pat   : PChar   location 'd1';
                            buf   : PChar   location 'd2';
                            buflen: LongInt location 'd3'): LongInt;
SysCall AOS_DOSBase 966;

function MatchPatternNoCase(pat: PChar location 'd1';
                            str: PChar location 'd2'): LongBool;
SysCall AOS_DOSBase 972;

function SameDevice(lock1: LongInt location 'd1';
                    lock2: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 984;

procedure ExAllEnd(lock   : LongInt       location 'd1';
                   buffer : PExAllData    location 'd2';
                   size   : LongInt       location 'd3';
                   data   : LongInt       location 'd4';
                   control: PExAllControl location 'd5');
SysCall AOS_DOSBase 990;

function SetOwner(name      : PChar   location 'd1';
                  owner_info: LongInt location 'd2'): LongBool;
SysCall AOS_DOSBase 996;

function AddSegmentTagList(tags: PTagItem location 'a0'): LongInt;
SysCall AOS_DOSBase 1002;

function FindSegmentTagList(tags: PTagItem location 'a0'): PSegment;
SysCall AOS_DOSBase 1008;

{$ENDIF AMIGA_V1_2_ONLY}
{$ENDIF AMIGA_V1_0_ONLY}

