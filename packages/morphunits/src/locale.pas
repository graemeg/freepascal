{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by Free Pascal development team

    locale.library functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}

unit locale;

interface

uses
  exec, amigados, utility;

const
  // Internal String-Numbers GetLocaleStr()
  // Days of Week
  DAY_1   = 1; // Sunday
  DAY_2   = 2; // Monday
  DAY_3   = 3; // Tuesday
  DAY_4   = 4; // Wednesday
  DAY_5   = 5; // Thursday
  DAY_6   = 6; // Friday
  DAY_7   = 7; // Saturday
  // Abbreviated Days of Week
  ABDAY_1 = 8;  // Sun
  ABDAY_2 = 9;  // Mon
  ABDAY_3 = 10; // Tue
  ABDAY_4 = 11; // Wed
  ABDAY_5 = 12; // Thu
  ABDAY_6 = 13; // Fri
  ABDAY_7 = 14; // Sat
  // Months
  MON_1    = 15; // January
  MON_2    = 16; // February
  MON_3    = 17; // March
  MON_4    = 18; // April
  MON_5    = 19; // May
  MON_6    = 20; // June
  MON_7    = 21; // July
  MON_8    = 22; // August
  MON_9    = 23; // September
  MON_10   = 24; // October
  MON_11   = 25; // November
  MON_12   = 26; // December
  // Abbreviated Months
  ABMON_1  = 27; // Jan
  ABMON_2  = 28; // Feb
  ABMON_3  = 29; // Mar
  ABMON_4  = 30; // Apr
  ABMON_5  = 31; // May
  ABMON_6  = 32; // Jun
  ABMON_7  = 33; // Jul
  ABMON_8  = 34; // Aug
  ABMON_9  = 35; // Sep
  ABMON_10 = 36; // Oct
  ABMON_11 = 37; // Nov
  ABMON_12 = 38; // Dec
  // Question
  YESSTR = 39; // affirmative response for yes/no queries
  NOSTR  = 40; // negative response for yes/no queries
  // 12 hour clock
  AM_STR = 41;
  PM_STR = 42;
  // Hyphen
  SOFTHYPHEN = 43;
  HARDHYPHEN = 44;
  // Quotes
  OPENQUOTE  = 45; // Open quote
  CLOSEQUOTE = 46; // Close quote
  // relative day names
  YESTERDAYSTR = 47; // Yesterday
  TODAYSTR     = 48; // Today
  TOMORROWSTR  = 49; // Tomorrow
  FUTURESTR    = 50; // Future

  MAXSTRMSG = 51; // current number of defined strings

type
  PLocaleBase = ^TLocaleBase;
  TLocaleBase = record
    lb_LibNode: TLibrary;
    lb_SysPatches: WordBool;
  end;

{ This structure must only be allocated by locale.library and is READ-ONLY! }
  PLocale = ^TLocale;
  TLocale = record
    // Basics
    loc_LocaleName: STRPTR;                   // Name of locale
    loc_LanguageName: STRPTR;                 // Language of locale
    loc_PrefLanguages: array[0..9] of STRPTR; // Preferred languages as array
    loc_Flags: LongWord;                      // Locale Flags
    // Country
    loc_CodeSet: LongWord;
    loc_CountryCode: LongWord;         // country code
    loc_TelephoneCode: LongWord;       // telephone code of country
    loc_GMTOffset: LongInt;            // distance to GMT in minutes
    loc_MeasuringSystem: Byte;         // Used measuring system (MS_*)
    loc_CalendarType: Byte;            // Define start of the Week (CT_*)
    loc_Reserved0: array[0..1] of Byte;
    // DateTime
    loc_DateTimeFormat,                     // Long date, time format
    loc_DateFormat,                         // Long date format
    loc_TimeFormat,                         // Long time format

    loc_ShortDateTimeFormat,                // Short date, time format
    loc_ShortDateFormat,                    // Short date format
    loc_ShortTimeFormat,                    // Short time format
    // Numerics
    loc_DecimalPoint: STRPTR;               // Decimalseparator
    loc_GroupSeparator: STRPTR;             // Thousandseparator
    loc_FracGroupSeparator: STRPTR;         // Additional separator
    loc_Grouping: PByte;                    // Size of loc_FracGroupSeparator group
    loc_FracGrouping: PByte;                // Size of loc_FracGroupSeparator group
    // Currency Format
    loc_MonDecimalPoint: STRPTR;
    loc_MonGroupSeparator: STRPTR;
    loc_MonFracGroupSeparator: STRPTR;
    loc_MonGrouping: PByte;
    loc_MonFracGrouping: PByte;
    loc_MonFracDigits: Byte;       // Numbers of places after loc_MonDecimalPoint
    loc_MonIntFracDigits: Byte;
    loc_Reserved1: array[0..1] of Byte;
    // Currency symbol
    loc_MonCS: STRPTR;             // Currency symbol (e.g. $)
    loc_MonSmallCS: STRPTR;        // Symbol for small amounts (eg. cent)
    loc_MonIntCS: STRPTR;          // Internationl (ISO 4217) code (e.g. EUR/USD/GBP)

    loc_MonPositiveSign: STRPTR;   // for positive money value
    loc_MonPositiveSpaceSep: Byte; // Separated by space (SS_*)
    loc_MonPositiveSignPos: Byte;  // Position of positive sign (SP_*)
    loc_MonPositiveCSPos: Byte;    // Position of currency symbol (CSP_*)
    loc_Reserved2: Byte;

    loc_MonNegativeSign: STRPTR;   // for negative money values
    loc_MonNegativeSpaceSep: Byte; // Separated by space (SS_*)
    loc_MonNegativeSignPos: Byte;  // Position of negative sign (SP_*)
    loc_MonNegativeCSPos: Byte;    // Position of currency symbol (CSP_*)
    loc_Reserved3: Byte;
 end;

const
  // Values for loc_MeasuringSystem
  MS_ISO      = 0;
  MS_AMERICAN = 1;
  MS_IMPERIAL = 2;
  MS_BRITISH  = 3;

  // Values for loc_CalendarType
  CT_7SUN = 0; // First day is Sunday
  CT_7MON = 1; // First day is Monday
  CT_7TUE = 2; // First day is Tuesday
  CT_7WED = 3; // First day is Wednesday
  CT_7THU = 4; // First day is Thursday
  CT_7FRI = 5; // First day is Friday
  CT_7SAT = 6; // First day is Saturday

  // Values for loc_MonPositiveSpaceSep and loc_MonNegativeSpaceSep
  SS_NOSPACE = 0; // No space between currency symbol and value
  SS_SPACE   = 1; // Set a space between currency symbol and value

  // Values for loc_MonPositiveSignPos and loc_MonNegativeSignPos
  SP_PARENS    = 0; // () surround the quantity and currency_symbol
  SP_PREC_ALL  = 1; // sign before value and symbol
  SP_SUCC_ALL  = 2; // sign after value and symbol
  SP_PREC_CURR = 3; // sign right before currency symbol
  SP_SUCC_CURR = 4; // sign right after currency symbol

  // Values for loc_MonPositiveCSPos and loc_MonNegativeCSPos
  CSP_PRECEDES = 0; // currency symbol before value
  CSP_SUCCEEDS = 1; // currency symbol after value

  // Tags for OpenCatalog()
  OC_TagBase         = TAG_USER + $90000;
  OC_BuiltInLanguage = OC_TagBase + 1; // language of built-in strings
  OC_BuiltInCodeSet  = OC_TagBase + 2; // code set of built-in strings
  OC_Version         = OC_TagBase + 3; // catalog version number required
  OC_Language        = OC_TagBase + 4; // preferred language of catalog
  OC_CodeSet         = OC_TagBase + 5; // V51
  OC_BuiltInStrings  = OC_TagBase + 6; // V52.3

  // Comparison types for StrnCmp()
  SC_ASCII    = 0;
  SC_COLLATE1 = 1;
  SC_COLLATE2 = 2;
  SC_UNICODE  = SC_ASCII;

  UCF_IGNORE_CASE = 1 shl 0;

  UNICODE_NFD  = 0;
  UNICODE_NFKD = 1;

type
// This structure must only be allocated by locale.library and is READ-ONLY!
  PCatalog = ^TCatalog;
  TCatalog = record
    cat_Link: TNode;       // for internal linkage
    cat_Pad: Word;         // to longword align
    cat_Language: STRPTR;  // language of the catalog
    cat_CodeSet: LongWord; // currently always 0
    cat_Version: Word;     // version of the catalog
    cat_Revision: Word;    // revision of the catalog
 end;

const
// cat_CodeSet values
  CODESET_LEGACY = 0;
  CODESET_UTF8   = 1;
  CODESET_UTF32  = 2;
  CODESET_COUNT  = 3;

  CODESET_LATIN1 = CODESET_LEGACY;
  CODESET_UCS4   = CODESET_UTF32;

// Values returned by IsUnicode()
  UNICODE_INVALID = 0; // ASCII or ISO-8859-1
  UNICODE_UTF8    = 1; // UTF-8
  UNICODE_16_BE   = 2; // UCS-2/UTF-16 big endian
  UNICODE_16_LE   = 3; // UCS-2/UTF-16 little endian
  UNICODE_32_BE   = 4; // UCS-4/UTF-32 big endian
  UNICODE_32_LE   = 5; // UCS-4/UTF-32 little endian

var
  LocaleBase: PLocaleBase = nil;

const
  LOCALENAME: PChar = 'locale.library';

procedure CloseCatalog(Catalog: PCatalog location 'a0'); syscall LocaleBase 36;
procedure CloseLocale(Locale: PLocale location 'a0'); syscall LocaleBase 42;
function ConvToLower(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongWord; syscall LocaleBase 48;
function ConvToUpper(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongWord; syscall LocaleBase 54;
procedure FormatDate(Locale: PLocale location 'a0'; FmtTemplate: STRPTR location 'a1'; Date: PDateStamp location 'a2'; PutCharFunc: PHook location 'a3'); syscall LocaleBase 60;
function FormatString(Locale: PLocale location 'a0'; FmtTemplate: STRPTR location 'a1'; DataStream: APTR location 'a2'; PutCharFunc: PHook location 'a3'): APTR; syscall LocaleBase 66;
function GetCatalogStr(Catalog: PCatalog location 'a0'; StringNum: LongInt location 'd0'; DefaultString: STRPTR location 'a1'): STRPTR; syscall LocaleBase 72;
function GetLocaleStr(Locale: PLocale location 'a0'; StringNum: LongWord location 'd0'): STRPTR; syscall LocaleBase 78;
function IsAlNum(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 84;
function IsAlpha(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 90;
function IsCntrl(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 96;
function IsDigit(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 102;
function IsGraph(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 108;
function IsLower(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 114;
function IsPrint(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 120;
function IsPunct(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 126;
function IsSpace(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 132;
function IsUpper(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 138;
function IsXDigit(Locale: PLocale location 'a0'; Character: LongWord location 'd0'): LongBool; syscall LocaleBase 144;
function OpenCatalogA(Locale: PLocale location 'a0'; Name: STRPTR location 'a1'; Tags: PTagItem location 'a2'): PCatalog; syscall LocaleBase 150;
function OpenLocale(Name: STRPTR location 'a0'): PLocale; syscall LocaleBase 156;
function ParseDate(Locale: PLocale location 'a0'; Date: PDateStamp location 'a1'; FmtTemplate: STRPTR location 'a2'; GetCharFunc: PHook location 'a3'): LongBool; syscall LocaleBase 162;
function StrConvert(Locale: PLocale location 'a0'; String1: STRPTR location 'a1'; Buffer: APTR location 'a2'; BufferSize: LongWord location 'd0'; Typ: LongWord location 'd1'): LongWord; syscall LocaleBase 29;
function StrnCmp(Locale: PLocale location 'a0'; String1: STRPTR location 'a2'; String2: STRPTR location 'a2'; Length: LongInt location 'd0'; Typ: LongWord location 'd1'): LongInt; syscall LocaleBase 30;
// MorphOS specific
function UCS4_ConvToLower(UCharacter: WideChar): WideChar; syscall sysv LocaleBase 232;
function UCS4_ConvToUpper(UCharacter: WideChar): WideChar; syscall sysv LocaleBase 238;
function UTF8_Decode(UTF8: STRPTR; UCharacter: PWideChar): LongWord; syscall sysv LocaleBase 244;
function UTF8_Encode(UCharacter: WideChar; UTF8: STRPTR): LongWord; syscall sysv LocaleBase 250;
function UCS4_GetCatalogStr(Catalog: PCatalog; StringNum: LongWord; defaultString: WSTRPTR): WSTRPTR; syscall sysvbase LocaleBase 256;
function UCS4_IsAlNum(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 262;
function UCS4_IsAlpha(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 268;
function UCS4_IsCntrl(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 274;
function UCS4_IsDigit(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 280;
function UCS4_IsGraph(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 286;
function UCS4_IsLower(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 292;
function UCS4_IsPrint(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 298;
function UCS4_IsPunct(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 304;
function UCS4_IsSpace(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 310;
function UCS4_IsUpper(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 316;
function UCS4_IsXDigit(Locale: PLocale; Character: WideChar): LongBool; syscall sysv LocaleBase 322;
procedure UCS4_FormatDate(Locale: PLocale; FmtTemplate: WSTRPTR; Date: PDateStamp; PutCharFunc: PHook); syscall sysvbase LocaleBase 328;
function UCS4_FormatString(Locale: PLocale; FmtTemplate: WSTRPTR; DataStream: APTR; PutCharFunc: PHook): APTR; syscall sysvbase LocaleBase 334;
function UCS4_GetLocaleStr(Locale: PLocale; StringNum: LongWord): WSTRPTR; syscall sysvbase LocaleBase 340;
function UCS4_StrnCmp(Locale: PLocale; String1: WSTRPTR; String2: WSTRPTR; Length: LongInt; Typ: LongWord): LongInt; syscall sysvbase LocaleBase 346;
function UCS4_StrToLower(Locale: PLocale; String1: WSTRPTR; buffer: WSTRPTR; BufferSize: LongWord; Typ: LongWord): LongInt; syscall sysvbase LocaleBase 352;
function UCS4_StrToUpper(Locale: PLocale; String1: WSTRPTR; buffer: WSTRPTR; BufferSize: LongWord; Typ: LongWord): LongInt; syscall sysvbase LocaleBase 358;
function UCS4_Decompose(Ch: WideChar): WSTRPTR; syscall sysvbase LocaleBase 364;
function UCS4_IsNSM(UCharacter: WideChar): LongBool; syscall sysv LocaleBase 370;
function UCS4_CanonicalDecompose(Ch: WideChar): WSTRPTR; syscall sysvbase LocaleBase 376;
procedure UCS4_Normalize(Src: WSTRPTR; Dst: WSTRPTR; Length: LongInt; Typ: LongWord); syscall sysvbase LocaleBase 382;
function ConvertUTF8ToUCS4(Src: STRPTR; Dst: WSTRPTR; Length: LongInt): LongWord; syscall sysv LocaleBase 388;
function ConvertUCS4ToUTF8(Src: WSTRPTR; Dst: STRPTR; Length: LongInt): LongWord; syscall sysv LocaleBase 394;
function UCS4_IsCombining(UCharacter: WideChar): LongWord; syscall sysv LocaleBase 400;
function UCS4_Compare(Locale: PLocale; String1: WSTRPTR; String2: WSTRPTR; Length: LongInt; Flags: LongWord): LongInt; syscall sysvbase LocaleBase 406;
function UCS4_GetCombiningClass(UCharacter: WideChar): LongWord; syscall sysv LocaleBase 412;
function UCS4_NormalizedLength(String1: WSTRPTR; Lenght: LongInt; Typ: LongWord): LongWord; syscall sysvbase LocaleBase 418;
function UTF8_CheckEncoding(String1: STRPTR; Length: LongInt): LongInt; syscall sysvbase LocaleBase 424;
function IsUnicode(Buffer: APTR; Length: LongWord): LongWord; syscall sysvbase LocaleBase 430;
function UTF8_EncodingLength(Utf32: WideChar): LongWord; syscall sysv LocaleBase 436;
function UTF8_DecodeSafe(Src: STRPTR; Dest: PWideChar; Length: LongWord): LongWord; syscall sysvbase LocaleBase 442;
procedure FormatClockData(Locale: PLocale; FormatString: STRPTR; CData: PClockData; Hook: PHook); syscall sysvbase LocaleBase 448;
procedure UCS4_FormatClockData(Locale: PLocale; FormatString: WSTRPTR; CData: PClockData; Hook: PHook); syscall sysvbase LocaleBase 460;

function OpenCatalog(Locale: PLocale; Name: STRPTR; const Tags: array of PtrUInt): PCatalog;

implementation

function OpenCatalog(Locale: PLocale; Name: STRPTR; const Tags: array of PtrUInt): PCatalog;
begin
  OpenCatalog := OpenCatalogA(Locale, Name, @Tags);
end;

const
  LIBVERSION = 0;

initialization
  LocaleBase := PLocaleBase(OpenLibrary(LOCALENAME, LIBVERSION));
finalization
  if Assigned(LocaleBase) then
    CloseLibrary(PLibrary(LocaleBase));
end.



