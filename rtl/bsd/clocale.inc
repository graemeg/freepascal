{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team.

    Init rtl formating variables based on BSD like libc locales

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 **********************************************************************}

{ Initial FreeBSD Implementation by Marco van de Voort.
  Crosschecked with OS X 10.4/ppc headers.
  To some symbols leading underscores were added. If these values are
   ever exposed in an interface this has to be reevaluated.
}

{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}

  __LC_ALL          = 0;
  __LC_COLLATE      = 1;
  __LC_CTYPE        = 2;
  __LC_MONETARY     = 3;
  __LC_NUMERIC      = 4;
  __LC_TIME         = 5;
  __LC_MESSAGES     = 6;
  _LC_LAST         = 7;              { marks end }


Type
     lconv = record
          decimal_point     : pchar;
          thousands_sep     : pchar;
          grouping          : pchar;
          int_curr_symbol   : pchar;
          currency_symbol   : pchar;
          mon_decimal_point : pchar;
          mon_thousands_sep : pchar;
          mon_grouping      : pchar;
          positive_sign     : pchar;
          negative_sign     : pchar;
          int_frac_digits   : char;
          frac_digits       : char;
          p_cs_precedes     : char;
          p_sep_by_space    : char;
          n_cs_precedes     : char;
          n_sep_by_space    : char;
          p_sign_posn       : char;
          n_sign_posn       : char;
          int_p_cs_precedes : char;
          int_n_cs_precedes : char;
          int_p_sep_by_space: char;
          int_n_sep_by_space: char;
          int_p_sign_posn   : char;
          int_n_sign_posn   : char;
       end;
     Tlconv = lconv;
     plconv = ^lconv;

Const
        CODESET         = 0;     { codeset name }
        D_T_FMT         = 1;     { string for formatting date and time }
        D_FMT           = 2;     { date format string }
        T_FMT           = 3;     { time format string }
        T_FMT_AMPM      = 4;     { a.m. or p.m. time formatting string }
        AM_STR          = 5;     { Ante Meridian affix }
        PM_STR          = 6;     { Post Meridian affix }

{ week day names }
        DAY_1           = 7;
        DAY_2           = 8;
        DAY_3           = 9;
        DAY_4           = 10;
        DAY_5           = 11;
        DAY_6           = 12;
        DAY_7           = 13;

{ abbreviated week day names }
        ABDAY_1         = 14;
        ABDAY_2         = 15;
        ABDAY_3         = 16;
        ABDAY_4         = 17;
        ABDAY_5         = 18;
        ABDAY_6         = 19;
        ABDAY_7         = 20;

{ month names }
        MON_1           = 21;
        MON_2           = 22;
        MON_3           = 23;
        MON_4           = 24;
        MON_5           = 25;
        MON_6           = 26;
        MON_7           = 27;
        MON_8           = 28;
        MON_9           = 29;
        MON_10          = 30;
        MON_11          = 31;
        MON_12          = 32;

{ abbreviated month names }
        ABMON_1         = 33;
        ABMON_2         = 34;
        ABMON_3         = 35;
        ABMON_4         = 36;
        ABMON_5         = 37;
        ABMON_6         = 38;
        ABMON_7         = 39;
        ABMON_8         = 40;
        ABMON_9         = 41;
        ABMON_10        = 42;
        ABMON_11        = 43;
        ABMON_12        = 44;
                        
        ERA             = 45;      { era description segments }
        ERA_D_FMT       = 46;      { era date format string }
        ERA_D_T_FMT     = 47;      { era date and time format string }
        ERA_T_FMT       = 48;      { era time format string }
        ALT_DIGITS      = 49;      { alternative symbols for digits }
        RADIXCHAR       = 50;      { radix char }
        THOUSEP         = 51;      { separator for thousands }

        YESEXPR         = 52;      { affirmative response expression }
        NOEXPR          = 53;      { negative response expression }
        YESSTR          = 54;      { affirmative response for yes/no queries }
        NOSTR           = 55;      { negative response for yes/no queries }
        CRNCYSTR        = 56;      { currency symbol }
        D_MD_ORDER      = 57;      { month/day order (local extension) }

function localeconv:plconv; cdecl; external clib name 'localeconv';
