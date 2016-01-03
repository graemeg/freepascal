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

    Added functions and procedures with array of const.
    For use with fpc 1.0.7. They are in systemvartags.
    11 Nov 2002.

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening
    of the library.
    14 Jan 2003.

    Changed the start code for unit.
    01 Feb 2003.

    Changed integer > smallint.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT locale;

INTERFACE
USES exec, amigados, utility;

const
{ constants for GetLocaleStr() }
 DAY_1          = 1;       { Sunday    }
 DAY_2          = 2;       { Monday    }
 DAY_3          = 3;       { Tuesday   }
 DAY_4          = 4;       { Wednesday }
 DAY_5          = 5;       { Thursday  }
 DAY_6          = 6;       { Friday    }
 DAY_7          = 7;       { Saturday  }

 ABDAY_1        = 8 ;      { Sun }
 ABDAY_2        = 9 ;      { Mon }
 ABDAY_3        = 10;      { Tue }
 ABDAY_4        = 11;      { Wed }
 ABDAY_5        = 12;      { Thu }
 ABDAY_6        = 13;      { Fri }
 ABDAY_7        = 14;      { Sat }

 MON_1          = 15;      { January   }
 MON_2          = 16;      { February  }
 MON_3          = 17;      { March     }
 MON_4          = 18;      { April     }
 MON_5          = 19;      { May       }
 MON_6          = 20;      { June      }
 MON_7          = 21;      { July      }
 MON_8          = 22;      { August    }
 MON_9          = 23;      { September }
 MON_10         = 24;      { October   }
 MON_11         = 25;      { November  }
 MON_12         = 26;      { December  }

 ABMON_1        = 27;      { Jan }
 ABMON_2        = 28;      { Feb }
 ABMON_3        = 29;      { Mar }
 ABMON_4        = 30;      { Apr }
 ABMON_5        = 31;      { May }
 ABMON_6        = 32;      { Jun }
 ABMON_7        = 33;      { Jul }
 ABMON_8        = 34;      { Aug }
 ABMON_9        = 35;      { Sep }
 ABMON_10       = 36;      { Oct }
 ABMON_11       = 37;      { Nov }
 ABMON_12       = 38;      { Dec }

 YESSTR         = 39;      { affirmative response for yes/no queries }
 NOSTR          = 40;      { negative response for yes/no queries    }

 AM_STR         = 41;      { AM }
 PM_STR         = 42;      { PM }

 SOFTHYPHEN     = 43;      { soft hyphenation }
 HARDHYPHEN     = 44;      { hard hyphenation }

 OPENQUOTE      = 45;      { start of quoted block }
 CLOSEQUOTE     = 46;      { end of quoted block   }

 YESTERDAYSTR   = 47;      { Yesterday }
 TODAYSTR       = 48;      { Today     }
 TOMORROWSTR    = 49;      { Tomorrow  }
 FUTURESTR      = 50;      { Future    }

 MAXSTRMSG      = 51;      { current number of defined strings }


{***************************************************************************}

Type
{ OpenLibrary("locale.library",0) returns a pointer to this structure }
 pLocaleBase = ^tLocaleBase;
 tLocaleBase = record
    lb_LibNode      : tLibrary;
    lb_SysPatches   : Boolean;   { TRUE if locale installed its patches }
 end;

{***************************************************************************}

Type
{ This structure must only be allocated by locale.library and is READ-ONLY! }
 pLocale = ^tLocale;
 tLocale = record
    loc_LocaleName,                         { locale's name               }
    loc_LanguageName    : STRPTR;           { language of this locale     }
    loc_PrefLanguages   : Array[0..9] of STRPTR;    { preferred languages }
    loc_Flags           : ULONG;            { always 0 for now            }

    loc_CodeSet,                            { always 0 for now            }
    loc_CountryCode,                        { user's country code         }
    loc_TelephoneCode,                      { country's telephone code    }
    loc_GMTOffset       : ULONG;            { minutes from GMT            }
    loc_MeasuringSystem,                    { what measuring system?      }
    loc_CalendarType    : Byte;             { what calendar type?         }
    loc_Reserved0       : Array[0..1] of Byte;

    loc_DateTimeFormat,                     { regular date & time format  }
    loc_DateFormat,                         { date format by itself       }
    loc_TimeFormat,                         { time format by itself       }

    loc_ShortDateTimeFormat,                { short date & time format    }
    loc_ShortDateFormat,                    { short date format by itself }
    loc_ShortTimeFormat,                    { short time format by itself }

    { for numeric values }
    loc_DecimalPoint,                       { character before the decimals }
    loc_GroupSeparator,                     { separates groups of digits    }
    loc_FracGroupSeparator : STRPTR;        { separates groups of digits    }
    loc_Grouping,                           { size of each group            }
    loc_FracGrouping    : Pointer;          { size of each group            }

    { for monetary values }
    loc_MonDecimalPoint,
    loc_MonGroupSeparator,
    loc_MonFracGroupSeparator : STRPTR;
    loc_MonGrouping,
    loc_MonFracGrouping : Pointer;
    loc_MonFracDigits,                      { digits after the decimal point   }
    loc_MonIntFracDigits : Byte;            { for international representation }
    loc_Reserved1       : Array[0..1] of Byte;

    { for currency symbols }
    loc_MonCS,                              { currency symbol              }
    loc_MonSmallCS,                         { symbol for small amounts     }
    loc_MonIntCS        : STRPTR;           { internationl (ISO 4217) code }

    { for positive monetary values }
    loc_MonPositiveSign : STRPTR;           { indicate positive money value   }
    loc_MonPositiveSpaceSep,                { determine if separated by space }
    loc_MonPositiveSignPos,                 { position of positive sign       }
    loc_MonPositiveCSPos,                   { position of currency symbol     }
    loc_Reserved2       : Byte;

    { for negative monetary values }
    loc_MonNegativeSign : STRPTR;           { indicate negative money value   }
    loc_MonNegativeSpaceSep,                { determine if separated by space }
    loc_MonNegativeSignPos,                 { position of negative sign       }
    loc_MonNegativeCSPos,                   { position of currency symbol     }
    loc_Reserved3       : Byte;
 end;

const
{ constants for Locale.loc_MeasuringSystem }
 MS_ISO         = 0;       { international metric system }
 MS_AMERICAN    = 1;       { american system             }
 MS_IMPERIAL    = 2;       { imperial system             }
 MS_BRITISH     = 3;       { british system              }

{ constants for Locale.loc_CalendarType }
 CT_7SUN        = 0;       { 7 days a week, Sunday is the first day    }
 CT_7MON        = 1;       { 7 days a week, Monday is the first day    }
 CT_7TUE        = 2;       { 7 days a week, Tuesday is the first day   }
 CT_7WED        = 3;       { 7 days a week, Wednesday is the first day }
 CT_7THU        = 4;       { 7 days a week, Thursday is the first day  }
 CT_7FRI        = 5;       { 7 days a week, Friday is the first day    }
 CT_7SAT        = 6;       { 7 days a week, Saturday is the first day  }

{ constants for Locale.loc_MonPositiveSpaceSep and Locale.loc_MonNegativeSpaceSep }
 SS_NOSPACE     = 0;       { cur. symbol is NOT separated from value with a space }
 SS_SPACE       = 1;       { cur. symbol IS separated from value with a space     }

{ constants for Locale.loc_MonPositiveSignPos and Locale.loc_MonNegativeSignPos }
 SP_PARENS      = 0;       { () surround the quantity and currency_symbol   }
 SP_PREC_ALL    = 1;       { sign string comes before amount and symbol     }
 SP_SUCC_ALL    = 2;       { sign string comes after amount and symbol      }
 SP_PREC_CURR   = 3;       { sign string comes right before currency symbol }
 SP_SUCC_CURR   = 4;       { sign string comes right after currency symbol  }

{ constants for Locale.loc_MonPositiveCSPos and Locale.loc_MonNegativeCSPos }
 CSP_PRECEDES   = 0;  { currency symbol comes before value }
 CSP_SUCCEEDS   = 1;  { currency symbol comes after value  }

{ elements of the byte arrays pointed to by:
 *   Locale.loc_Grouping
 *   Locale.loc_FracGrouping
 *   Locale.loc_MonGrouping
 *   Locale.loc_MonFracGrouping
 * are interpreted as follows:
 *
 *    255     indicates that no further grouping is to be performed
 *    0       indicates that the previous element is to be repeatedly used
 *            for the remainder of the digits
 *    <other> the number of digits that comprises the current group
 }


{***************************************************************************}


{ Tags for OpenCatalog() }
 OC_TagBase         = (TAG_USER + $90000);
 OC_BuiltInLanguage = OC_TagBase+1;   { language of built-in strings    }
 OC_BuiltInCodeSet  = OC_TagBase+2;   { code set of built-in strings    }
 OC_Version         = OC_TagBase+3;   { catalog version number required }
 OC_Language        = OC_TagBase+4;   { preferred language of catalog   }


{***************************************************************************}


{ Comparison types for StrnCmp() }
 SC_ASCII    = 0;
 SC_COLLATE1 = 1;
 SC_COLLATE2 = 2;


{***************************************************************************}

Type
{ This structure must only be allocated by locale.library and is READ-ONLY! }
 pCatalog = ^tCatalog;
 tCatalog = record
    cat_Link    : tNode;        { for internal linkage    }
    cat_Pad     : WORD;         { to longword align       }
    cat_Language: STRPTR;       { language of the catalog }
    cat_CodeSet : ULONG;        { currently always 0      }
    cat_Version : WORD;         { version of the catalog  }
    cat_Revision: WORD;         { revision of the catalog }
 end;

{***************************************************************************}

{ --- functions in V38 or higher (Release 2.1) --- }

VAR LocaleBase : pLocaleBase;

const
    LOCALENAME : PChar = 'locale.library';

PROCEDURE CloseCatalog(catalog : pCatalog);
PROCEDURE CloseLocale(locale : pLocale);
FUNCTION ConvToLower(locale : pLocale; character : ULONG) : ULONG;
FUNCTION ConvToUpper(locale : pLocale; character : ULONG) : ULONG;
PROCEDURE FormatDate(locale : pLocale; fmtTemplate : pCHAR; date : pDateStamp; putCharFunc : pHook);
FUNCTION FormatString(locale : pLocale; fmtTemplate : pCHAR; dataStream : POINTER; putCharFunc : pHook) : POINTER;
FUNCTION GetCatalogStr(catalog : pCatalog; stringNum : LONGINT; defaultString : pCHAR) : pCHAR;
FUNCTION GetLocaleStr(locale : pLocale; stringNum : ULONG) : pCHAR;
FUNCTION IsAlNum(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsAlpha(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsCntrl(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsDigit(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsGraph(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsLower(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsPrint(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsPunct(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsSpace(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsUpper(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION IsXDigit(locale : pLocale; character : ULONG) : BOOLEAN;
FUNCTION OpenCatalogA(locale : pLocale; name : pCHAR; tags : pTagItem) : pCatalog;
FUNCTION OpenLocale(name : pCHAR) : pLocale;
FUNCTION ParseDate(locale : pLocale; date : pDateStamp; fmtTemplate : pCHAR; getCharFunc : pHook) : BOOLEAN;
FUNCTION StrConvert(locale : pLocale; string1 : pCHAR; buffer : POINTER; bufferSize : ULONG; typ : ULONG) : ULONG;
FUNCTION StrnCmp(locale : pLocale; string1 : pCHAR; string2 : pCHAR; length : LONGINT; typ : ULONG) : LONGINT;

{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitLOCALELibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    LOCALEIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox;
{$endif dont_use_openlib}

PROCEDURE CloseCatalog(catalog : pCatalog);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L catalog,A0
    MOVEA.L LocaleBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE CloseLocale(locale : pLocale);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVEA.L LocaleBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ConvToLower(locale : pLocale; character : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ConvToUpper(locale : pLocale; character : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE FormatDate(locale : pLocale; fmtTemplate : pCHAR; date : pDateStamp; putCharFunc : pHook);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVEA.L fmtTemplate,A1
    MOVEA.L date,A2
    MOVEA.L putCharFunc,A3
    MOVEA.L LocaleBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION FormatString(locale : pLocale; fmtTemplate : pCHAR; dataStream : POINTER; putCharFunc : pHook) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVEA.L fmtTemplate,A1
    MOVEA.L dataStream,A2
    MOVEA.L putCharFunc,A3
    MOVEA.L LocaleBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetCatalogStr(catalog : pCatalog; stringNum : LONGINT; defaultString : pCHAR) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L catalog,A0
    MOVE.L  stringNum,D0
    MOVEA.L defaultString,A1
    MOVEA.L LocaleBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetLocaleStr(locale : pLocale; stringNum : ULONG) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  stringNum,D0
    MOVEA.L LocaleBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IsAlNum(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsAlpha(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsCntrl(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsDigit(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsGraph(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsLower(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsPrint(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsPunct(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsSpace(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsUpper(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsXDigit(locale : pLocale; character : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVE.L  character,D0
    MOVEA.L LocaleBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION OpenCatalogA(locale : pLocale; name : pCHAR; tags : pTagItem) : pCatalog;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVEA.L name,A1
    MOVEA.L tags,A2
    MOVEA.L LocaleBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenLocale(name : pCHAR) : pLocale;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L name,A0
    MOVEA.L LocaleBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ParseDate(locale : pLocale; date : pDateStamp; fmtTemplate : pCHAR; getCharFunc : pHook) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVEA.L date,A1
    MOVEA.L fmtTemplate,A2
    MOVEA.L getCharFunc,A3
    MOVEA.L LocaleBase,A6
    JSR -162(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION StrConvert(locale : pLocale; string1 : pCHAR; buffer : POINTER; bufferSize : ULONG; typ : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVEA.L string1,A1
    MOVEA.L buffer,A2
    MOVE.L  bufferSize,D0
    MOVE.L  typ,D1
    MOVEA.L LocaleBase,A6
    JSR -174(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION StrnCmp(locale : pLocale; string1 : pCHAR; string2 : pCHAR; length : LONGINT; typ : ULONG) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L locale,A0
    MOVEA.L string1,A1
    MOVEA.L string2,A2
    MOVE.L  length,D0
    MOVE.L  typ,D1
    MOVEA.L LocaleBase,A6
    JSR -180(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of locale.library}
  {$Info don't forget to use InitLOCALELibrary in the beginning of your program}

var
    locale_exit : Pointer;

procedure CloselocaleLibrary;
begin
    ExitProc := locale_exit;
    if LocaleBase <> nil then begin
        CloseLibrary(pLibrary(LocaleBase));
        LocaleBase := nil;
    end;
end;

procedure InitLOCALELibrary;
begin
    LocaleBase := nil;
    LocaleBase := pLocaleBase(OpenLibrary(LOCALENAME,LIBVERSION));
    if LocaleBase <> nil then begin
        locale_exit := ExitProc;
        ExitProc := @CloselocaleLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open locale.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    LOCALEIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of locale.library}

var
    locale_exit : Pointer;

procedure CloselocaleLibrary;
begin
    ExitProc := locale_exit;
    if LocaleBase <> nil then begin
        CloseLibrary(pLibrary(LocaleBase));
        LocaleBase := nil;
    end;
end;

begin
    LocaleBase := nil;
    LocaleBase := pLocaleBase(OpenLibrary(LOCALENAME,LIBVERSION));
    if LocaleBase <> nil then begin
        locale_exit := ExitProc;
        ExitProc := @CloselocaleLibrary;
        LOCALEIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open locale.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    LOCALEIsCompiledHow := 3;
   {$Warning No autoopening of locale.library compiled}
   {$Warning Make sure you open locale.library yourself}
{$endif dont_use_openlib}


END. (* UNIT LOCALE *)



