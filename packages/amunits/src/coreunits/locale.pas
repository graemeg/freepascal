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
{$PACKRECORDS 2}

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
    lb_SysPatches   : WordBool;   { TRUE if locale installed its patches }
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

VAR LocaleBase : pLocaleBase = nil;

const
    LOCALENAME : PChar = 'locale.library';

PROCEDURE CloseCatalog(catalog : pCatalog location 'a0'); syscall LocaleBase 036;
PROCEDURE CloseLocale(locale : pLocale location 'a0'); syscall LocaleBase 042;
FUNCTION ConvToLower(locale : pLocale location 'a0'; character : ULONG location 'd0') : ULONG; syscall LocaleBase 048;
FUNCTION ConvToUpper(locale : pLocale location 'a0'; character : ULONG location 'd0') : ULONG; syscall LocaleBase 054;
PROCEDURE FormatDate(locale : pLocale location 'a0'; fmtTemplate : pCHAR location 'a1'; date : pDateStamp location 'a2'; putCharFunc : pHook location 'a3'); syscall LocaleBase 060;
FUNCTION FormatString(locale : pLocale location 'a0'; fmtTemplate : pCHAR location 'a1'; dataStream : POINTER location 'a2'; putCharFunc : pHook location 'a3') : POINTER; syscall LocaleBase 066;
FUNCTION GetCatalogStr(catalog : pCatalog location 'a0'; stringNum : LONGINT location 'd0'; defaultString : pCHAR location 'a1') : pCHAR; syscall LocaleBase 072;
FUNCTION GetLocaleStr(locale : pLocale location 'a0'; stringNum : ULONG location 'd0') : pCHAR; syscall LocaleBase 078;
FUNCTION IsAlNum(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 084;
FUNCTION IsAlpha(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 090;
FUNCTION IsCntrl(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 096;
FUNCTION IsDigit(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 102;
FUNCTION IsGraph(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 108;
FUNCTION IsLower(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 114;
FUNCTION IsPrint(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 120;
FUNCTION IsPunct(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 126;
FUNCTION IsSpace(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 132;
FUNCTION IsUpper(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 138;
FUNCTION IsXDigit(locale : pLocale location 'a0'; character : ULONG location 'd0') : LongBool; syscall LocaleBase 144;
FUNCTION OpenCatalogA(locale : pLocale location 'a0'; name : pCHAR location 'a1'; tags : pTagItem location 'a2') : pCatalog; syscall LocaleBase 150;
FUNCTION OpenLocale(name : pCHAR location 'a0') : pLocale; syscall LocaleBase 156;
FUNCTION ParseDate(locale : pLocale location 'a0'; date : pDateStamp location 'a1'; fmtTemplate : pCHAR location 'a2'; getCharFunc : pHook location 'a3') : LongBool; syscall LocaleBase 162;
FUNCTION StrConvert(locale : pLocale location 'a0'; string1 : pCHAR location 'a1'; buffer : POINTER location 'a2'; bufferSize : ULONG location 'd0'; typ : ULONG location 'd1') : ULONG; syscall LocaleBase 174;
FUNCTION StrnCmp(locale : pLocale location 'a0'; string1 : pCHAR location 'a1'; string2 : pCHAR location 'a2'; length : LONGINT location 'd0'; typ : ULONG location 'd1') : LONGINT; syscall LocaleBase 180;

function OpenCatalog(locale : pLocale; name : pCHAR; Const argv : array of PtrUInt) : pCatalog;

IMPLEMENTATION

function OpenCatalog(locale : pLocale; name : pCHAR; Const argv : array of PtrUInt) : pCatalog;
begin
    OpenCatalog := OpenCatalogA(locale,name,@argv);
end;

const
    { Change VERSION and LIBVERSION to proper values }
    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

initialization
  LocaleBase := pLocaleBase(OpenLibrary(LOCALENAME,LIBVERSION));
finalization
  if Assigned(LocaleBase) then
    CloseLibrary(pLibrary(LocaleBase));
END. (* UNIT LOCALE *)



