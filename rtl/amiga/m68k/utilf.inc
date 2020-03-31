{
    This file is part of the Free Pascal run time library.

    utility functions for AmigaOS/m68k
    Copyright (c) 2007 Karoly Balogh

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ these functions are only available v36+ (OS Release 2.0+) }
{$IFNDEF AMIGA_V1_0_ONLY}
{$IFNDEF AMIGA_V1_2_ONLY}

function FindTagItem(tagVal : Cardinal location 'd0';
                     tagList: PTagItem location 'a0'): PTagItem;
SysCall AOS_UtilityBase 030;

function GetTagData(tagValue  : Cardinal location 'd0';
                    defaultVal: Cardinal location 'd1';
                    tagList   : PTagItem location 'a0'): Cardinal;
SysCall AOS_UtilityBase 036;

function PackBoolTags(initialFlags: Cardinal location 'd0';
                      tagList     : PTagItem location 'a0';
                      boolMap     : PTagItem location 'a1'): Cardinal;
SysCall AOS_UtilityBase 042;

function NextTagItem(tagListPtr: pPTagItem location 'a0'): PTagItem;
SysCall AOS_UtilityBase 048;

procedure FilterTagChanges(changeList  : PTagItem location 'a0';
                           originalList: PTagItem location 'a1';
                           apply       : Cardinal location 'd0');
SysCall AOS_UtilityBase 054;

procedure MapTags(tagList: PTagItem location 'a0';
                  mapList: PTagItem location 'a1';
                  mapType: Cardinal location 'd0');
SysCall AOS_UtilityBase 060;

function AllocateTagItems(numTags: Cardinal location 'd0'): PTagItem;
SysCall AOS_UtilityBase 066;

function CloneTagItems(tagList: PTagItem location 'a0'): PTagItem;
SysCall AOS_UtilityBase 072;

procedure FreeTagItems(tagList: PTagItem location 'a0');
SysCall AOS_UtilityBase 078;

procedure RefreshTagItemClones(clone   : PTagItem location 'a0';
                               original: PTagItem location 'a1');
SysCall AOS_UtilityBase 084;

function TagInArray(tagValue    : Cardinal location 'd0';
                    var tagArray: Cardinal location 'a0'): LongBool;
SysCall AOS_UtilityBase 090;

function FilterTagItems(tagList        : PTagItem location 'a0';
                        var filterArray: Cardinal location 'a1';
                        logic          : Cardinal location 'd0'): Cardinal;
SysCall AOS_UtilityBase 096;

function CallHookPkt(hook       : PHook   location 'a0';
                     hobject    : Pointer location 'a2';
                     paramPacket: Pointer location 'a1'): Cardinal;
SysCall AOS_UtilityBase 102;

procedure Amiga2Date(seconds: Cardinal   location 'd0';
                     result : PClockData location 'a0');
SysCall AOS_UtilityBase 120;

function Date2Amiga(date: PClockData location 'a0'): Cardinal;
SysCall AOS_UtilityBase 126;

function CheckDate(date: PClockData location 'a0'): Cardinal;
SysCall AOS_UtilityBase 132;

function SMult32(arg1: LongInt location 'd0';
                 arg2: LongInt location 'd1'): LongInt;
SysCall AOS_UtilityBase 138;

function UMult32(arg1: Cardinal location 'd0';
                 arg2: Cardinal location 'd1'): Cardinal;
SysCall AOS_UtilityBase 144;

function SDivMod32(dividend: LongInt location 'd0';
                    divisor: LongInt location 'd1'): LongInt;
SysCall AOS_UtilityBase 150;

function UDivMod32(dividend: Cardinal location 'd0';
                   divisor : Cardinal location 'd1'): Cardinal;
SysCall AOS_UtilityBase 156;

function Stricmp(string1: PChar location 'a0';
                 string2: PChar location 'a1'): LongInt;
SysCall AOS_UtilityBase 162;

function Strnicmp(string1: PChar   location 'a0';
                  string2: PChar   location 'a1';
                  length : LongInt location 'd0'): LongInt;
SysCall AOS_UtilityBase 168;

function ToUpper(character: Cardinal location 'd0'): Char;
SysCall AOS_UtilityBase 174;

function ToLower(character: Cardinal location 'd0'): Char;
SysCall AOS_UtilityBase 180;

procedure ApplyTagChanges(list      : PTagItem location 'a0';
                          changeList: PTagItem location 'a1');
SysCall AOS_UtilityBase 186;

function SMult64(arg1: LongInt location 'd0';
                 arg2: LongInt location 'd1'): LongInt;
SysCall AOS_UtilityBase 198;

function UMult64(arg1: Cardinal location 'd0';
                 arg2: Cardinal location 'd1'): Cardinal;
SysCall AOS_UtilityBase 204;

function PackStructureTags(pack         : Pointer  location 'a0';
                           var packTable: Cardinal location 'a1';
                           tagList      : PTagItem location 'a2'): Cardinal;
SysCall AOS_UtilityBase 210;

function UnpackStructureTags(pack         : Pointer  location 'a0';
                             var packTable: Cardinal location 'a1';
                             tagList      : PTagItem location 'a2'): Cardinal;
SysCall AOS_UtilityBase 216;

function AddNamedObject(nameSpace: PNamedObject location 'a0';
                        nobject  : PNamedObject location 'a1'): LongBool;
SysCall AOS_UtilityBase 222;

function AllocNamedObjectA(name   : PChar    location 'a0';
                           tagList: PTagItem location 'a1'): PNamedObject;
SysCall AOS_UtilityBase 228;

function AttemptRemNamedObject(nobject: PNamedObject location 'a0'): LongInt;
SysCall AOS_UtilityBase 234;

function FindNamedObject(nameSpace : PNamedObject location 'a0';
                         name      : PChar        location 'a1';
                         lastObject: PNamedObject location 'a2'): PNamedObject;
SysCall AOS_UtilityBase 240;

procedure FreeNamedObject(nobject: PNamedObject location 'a0');
SysCall AOS_UtilityBase 246;

function NamedObjectName(nobject: PNamedObject location 'a0'): PChar;
SysCall AOS_UtilityBase 252;

procedure ReleaseNamedObject(nobject: pNamedObject location 'a0');
SysCall AOS_UtilityBase 258;

procedure RemNamedObject(nobject: PNamedObject location 'a0';
                         message: PMessage     location 'a1');
SysCall AOS_UtilityBase 264;

function GetUniqueID: Cardinal;
SysCall AOS_UtilityBase 270;

{$ENDIF AMIGA_V1_2_ONLY}
{$ENDIF AMIGA_V1_0_ONLY}
