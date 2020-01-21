#
#   This file is part of the Free Pascal Compiler
#   Copyright (c) 1999-2018 by the Free Pascal Development team
#
#   Hebrew (CP1255) language file for Free Pascal Compiler
#   Contributed by Ido Kanner <idokan at gmail.com> and Dotan Kamber <kamberd at yahoo.com>
#   Based on errore.msg of SVN revision 8988
#
#   See the file COPYING.v2, included in this distribution,
#   for details about the copyright.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#

# CodePage 1255

#
# The constants are build in the following order:
# <part>_<type>_<txtidentifier>
#
# <part> is the part of the compiler the message is used
#   asmr_     assembler parsing
#   asmw_     assembler writing/binary writers
#   unit_     unit handling
#   scan_     scanner
#   parser_   parser
#   type_     type checking
#   general_  general info
#   exec_     calls to assembler, linker, binder
#
# <type> the type of the message it should normally used for
#   f_   fatal error
#   e_   error
#   w_   warning
#   n_   note
#   h_   hint
#   i_   info
#   l_   add linenumber
#   u_   used
#   t_   tried
#   c_   conditional
#   d_   debug message
#   x_   executable informations
#

#
# General
#
# 01023 is the last used one
#
# BeginOfTeX
% \section{General compiler messages}
% This section gives the compiler messages which are not fatal, but which
% display useful information. The number of such messages can be
% controlled with the various verbosity level \var{-v} switches.
% \begin{description}
general_t_compilername=01000_T_מהדר: $1
% When the \var{-vt} switch is used, this line tells you what compiler
% is used.
general_d_sourceos=01001_D_מערכת הפעלה: $1
% When the \var{-vd} switch is used, this line tells you what the source
% operating system is.
general_i_targetos=01002_I_מערכת הפעלה מיועדת: $1
% When the \var{-vd} switch is used, this line tells you what the target
% operating system is.
general_t_exepath=01003_T_משתמש בנתיב הרצה: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its binaries.
general_t_unitpath=01004_T_משתמש בנתיב היחידות: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for compiled units. You can set this path with the \var{-Fu}
general_t_includepath=01005_T_משתמש בנתיב המוסף: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its include files (files used in \var{\{\$I xxx\}} statements).
% You can set this path with the \var{-I} option.
general_t_librarypath=01006_T_משתמש בנתיב הספריה: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for the libraries. You can set this path with the \var{-Fl} option.
general_t_objectpath=01007_T_Using object path: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for object files you link in (files used in \var{\{\$L xxx\}} statements).
% You can set this path with the \var{-Fo} option.
general_i_abslines_compiled=01008_I_$1 שורות הודרו, ב$2 שניות
% When the \var{-vi} switch is used, the compiler reports the number
% of lines compiled, and the time it took to compile them (real time,
% not program time).
general_f_no_memory_left=01009_F_לא נשאר זכרון
% The compiler doesn't have enough memory to compile your program. There are
% several remedies for this:
% \begin{itemize}
% \item If you're using the build option of the compiler, try compiling the
% different units manually.
% \item If you're compiling a huge program, split it up in units, and compile
% these separately.
% \item If the previous two don't work, recompile the compiler with a bigger
% heap (you can use the \var{-Ch} option for this, \seeo{Ch})
% \end{itemize}
general_i_writingresourcefile=01010_I_כותב את קובץ משאבי טבלת המחרוזות: $1
% This message is shown when the compiler writes the Resource String Table
% file containing all the resource strings for a program.
general_e_errorwritingresourcefile=01011_E_כותב את קובץ משאבי טבלת המחרוזות: $1
% This message is shown when the compiler encountered an error when writing
% the Resource String Table file
general_i_fatal=01012_I_שגיאה קריטית:
% Prefix for Fatal Errors
general_i_error=01013_I_שגיאה:
% Prefix for Errors
general_i_warning=01014_I_התראה:
% Prefix for Warnings
general_i_note=01015_I_הערה:
% Prefix for Notes
general_i_hint=01016_I_רמז:
% Prefix for Hints
general_e_path_does_not_exist=01017_E_הנתיב "$1" אינו קיים
% The specified path does not exist.
general_f_compilation_aborted=01018_F_ההידור בוטל
% Compilation was aborted.
general_text_bytes_code=01019_bytes code
general_text_bytes_data=01020_bytes data
general_i_number_of_warnings=01021_I_אזהרות $1 הונפקו
% Total number of warnings issued during compilation.
general_i_number_of_hints=01022_I_הונפקו  $1 רמזים
% Total number of hints issued during compilation.
general_i_number_of_notes=01023_I_הונפקו $1 הערות
% Total number of notes issued during compilation.
% \end{description}
#
# Scanner
#
# 02084 is the last used one
#
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compiling handling.
% \begin{description}
scan_f_end_of_file=02000_F_סוף קובץ לא צפוי
% this typically happens in one of the following cases :
% \begin{itemize}
% \item The source file ends before the final \var{end.} statement. This
% happens mostly when the \var{begin} and \var{end} statements aren't
% balanced;
% \item An include file ends in the middle of a statement.
% \item A comment was not closed
% \end{itemize}
scan_f_string_exceeds_line=02001_F_מחרוזת גולשת משורה
% There is a missing closing ' in a string, so it occupies
% multiple lines.
scan_f_illegal_char=02002_F_תו לא חוקי "$1" ($2)
% An illegal character was encountered in the input file.
scan_f_syn_expected=02003_F_שגיאת תחביר: מצפה ל "$1" אבל "$2" נמצא
% This indicates that the compiler expected a different token than
% the one you typed. It can occur almost everywhere where you make a
% mistake against the pascal language.
scan_t_start_include_file=02004_TL_מתחיל קריאה של קובץ תוספות $1
% When you provide the \var{-vt} switch, the compiler tells you
% when it starts reading an included file.
scan_w_comment_level=02005_W_רמת הערות $1 נמצאה
% When the \var{-vw} switch is used, then the compiler warns you if
% it finds nested comments. Nested comments are not allowed in Turbo Pascal
% and can be a possible source of errors.
scan_n_ignored_switch=02008_N_מתעלם ממתג מהדר "$1"
% With \var{-vn} on, the compiler warns if it ignores a switch
scan_w_illegal_switch=02009_W_מתג מהדר "$1" אינו חוקי
% You included a compiler switch (i.e. \var{\{\$... \}}) which the compiler
% does not recognise
scan_w_switch_is_global=02010_W_מיקום שגוי למתג מהדר גלובלי
% The compiler switch is misplaced, and should be located at
% the start of the unit or program.
scan_e_illegal_char_const=02011_E_תו קבוע לא חוקי
% This happens when you specify a character with its ASCII code, as in
% \var{\#96}, but the number is either illegal, or out of range.
scan_f_cannot_open_input=02012_F_לא ניתן לפתוח את קובץ "$1"
% \fpc cannot find the program or unit source file you specified on the
% command line.
scan_f_cannot_open_includefile=02013_F_לא ניתן לפתוח את קובץ התוספות "$1"
% \fpc cannot find the source file you specified in a \var{\{\$include ..\}}
% statement.
scan_e_illegal_pack_records=02015_E_יישור רשומה לא חוקי מייצג "$1"
% You are specifying the \var{\{\$PACKRECORDS n\} } or \var{\{\$ALIGN n\} }
% with an illegal value for \var{n}. For \$PACKRECORDS valid alignments are 1, 2, 4, 8, 16, 32, C,
% NORMAL, DEFAULT, and for \$ALIGN valid alignment are 1, 2, 4, 8, 16, 32, ON,
% OFF. Under mode MacPas \$ALIGN also supports MAC68K, POWER and RESET.
scan_e_illegal_pack_enum=02016_E_מספר מינימלי של מנייה המייצג "$1" אינו חוקי
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2,4, NORMAL or DEFAULT are valid here.
scan_e_endif_expected=02017_E_מצפה ל$ENDIF בהגדרה $1 $2 הנמצאת ב$3 שורה $4
% Your conditional compilation statements are unbalanced.
scan_e_preproc_syntax_error=02018_E_שגיאת תחביר בעת ניתוח משפט תנאי
% There is an error in the expression following the \var{\{\$if ..\}}, $ifc or $setc compiler
% directives.
scan_e_error_in_preproc_expr=02019_E_הערכת משפט תנאי
% There is an error in the expression following the \var{\{\$if ..\}}, $ifc or $setc compiler
% directives.
scan_w_macro_cut_after_255_chars=02020_W_תוכן המקרו מוגבל לאורך של 255 תווים
% The contents of macros cannot be longer than 255 characters.
scan_e_endif_without_if=02021_E_ENDIF ללא  IF(N)DEF
% Your \var{\{\$IFDEF ..\}} and {\{\$ENDIF\}} statements aren't balanced.
scan_f_user_defined=02022_F_הגדרת משתמש: $1
% A user defined fatal error occurred. see also the \progref
scan_e_user_defined=02023_E_הגדרת משתמש: $1
% A user defined error occurred. see also the \progref
scan_w_user_defined=02024_W_הגדרת משתמש: $1
% A user defined warning occurred. see also the \progref
scan_n_user_defined=02025_N_הגדרת משתמש: $1
% A user defined note was encountered. see also the \progref
scan_h_user_defined=02026_H_הגדרת משתמש: $1
% A user defined hint was encountered. see also the \progref
scan_i_user_defined=02027_I_הגדרת משתמש: $1
% User defined information was encountered. see also the \progref
scan_e_keyword_cant_be_a_macro=02028_E_מילת מפתח המוגדרת מחדש כמקרו אינה מכילה השפעה
% You cannot redefine keywords with macros.
scan_f_macro_buffer_overflow=02029_F_גלישת מאגר מקרו בעת קריאה או הרחבה של מקרו
% Your macro or its result was too long for the compiler.
scan_w_macro_too_deep=02030_W_הרחבת מקרו גולשת מעומק של 16.
% When expanding a macro, macros have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_w_wrong_styled_switch=02031_W_ מתגי פעולת מהדר אינם נתמכים בהערות מסוג //
% Compiler switches should be in normal pascal style comments.
scan_d_handling_switch=02032_DL_מטפל במתג "$1"
% When you set debugging info on (\var{-vd}) the compiler tells you when it
% is evaluating conditional compile statements.
scan_c_endif_found=02033_CL_נמצא ENDIF $1
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifdef_found=02034_CL_נמצא IFDEF $1, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifopt_found=02035_CL_נמצא IFOPT $1, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_if_found=02036_CL_נמצא IF $1, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifndef_found=02037_CL_נמצא IFNDEF $1, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_else_found=02038_CL_נמצא ELSE $1, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_skipping_until=02039_CL_מדלג...
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements, and whether it is skipping or
% compiling parts.
scan_i_press_enter=02040_I_לחץ על <return> להמשיך
% When the \var{-vi} switch is used, the compiler stops compilation
% and waits for the \var{Enter} key to be pressed when it encounters
% a \var{\{\$STOP\}} directive.
scan_w_unsupported_switch=02041_W_מתג "$1" אינו נתמך
% When warnings are turned on (\var{-vw}) the compiler warns you about
% unsupported switches. This means that the switch is used in Delphi or
% Turbo Pascal, but not in \fpc
scan_w_illegal_directive=02042_W_הוראת המהדר "$1" אינה חוקית
% When warings are turned on (\var{-vw}) the compiler warns you about
% unrecognised switches. For a list of recognised switches, \progref
scan_t_back_in=02043_TL_חזרה ב$1
% When you use (\var{-vt}) the compiler tells you when it has finished
% reading an include file.
scan_w_unsupported_app_type=02044_W_אין תמיכה לסוג האפליקציה: "$1"
% You get this warning, if you specify an unknown application type
% with the directive \var{\{\$APPTYPE\}}
scan_w_app_type_not_support=02045_W_APPTYPE אינו נתמך ע"י ההמערכת הפעלה המבוקשת
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
scan_w_description_not_support=02046_W_DESCRIPTION אינו נתמך ע"י מערכת ההפעלה המבוקשת
% The \var{\{\$DESCRIPTION\}} directive is not supported on this target OS
scan_n_version_not_support=02047_N_VERSION אינו נתמך ע"י מערכת הפעלה המבוקשת
% The \var{\{\$VERSION\}} directive is not supported on this target OS
scan_n_only_exe_version=02048_N_VERSION רק לקבצי exe או DLL
% The \var{\{\$VERSION\}} directive is only used for executable or DLL sources.
scan_w_wrong_version_ignored=02049_W_פורמט שגוי להוראת VERSION: "$1"
% The \var{\{\$VERSION\}} directive format is majorversion.minorversion
% where majorversion and minorversion are words.
scan_e_illegal_asmmode_specifier=02050_E_סגנון האסמבלר שצויין "$1" אינו חוקי
% When you specify an assembler mode with the \var{\{\$ASMMODE xxx\}}
% the compiler didn't recognize the mode you specified.
scan_w_no_asm_reader_switch_inside_asm=02051_W_מתג קורא הASM אינו אפשרי בתוך משפט אסמבלי. "$1" יהיה ישפיע רק בהכרזה הבאה/
% It is not possible to switch from one assembler reader to another
% inside an assembler block. The new reader will be used for next
% assembler statements only.
scan_e_wrong_switch_toggle=02052_E_שימוש שגוי במתג. השתמש בON/OFF או ב +/-
% You need to use ON or OFF or a + or - to toggle the switch
scan_e_resourcefiles_not_supported=02053_E_קובץ המשאבים אינו נתמך ביעד הנבחר
% The target you are compiling for doesn't support resource files.
scan_w_include_env_not_found=02054_W_משתנה הסביבה "$1" לא נמצא בסביבה
% The included environment variable can't be found in the environment, it will
% be replaced by an empty string instead.
scan_e_invalid_maxfpureg_value=02055_E_ערך לא חוקי להגבלות אוגר הFPC
% Valid values for this directive are 0..8 and NORMAL/DEFAULT
scan_w_only_one_resourcefile_supported=02056_W_רק קובץ משאבים אחד נתמך ביעד זה
% The target you are compiling for supports only one resource file.
% The first resource file found is used, the others are discarded.
scan_w_macro_support_turned_off=02057_W_התמיכה במקרו כובה
% A macro declaration has been found, but macro support is currently off,
% so the declaration will be ignored. To turn macro support on compile with
% -Sm on the commandline or add \{\$MACRO ON\} in the source
scan_e_invalid_interface_type=02058_E_הוכרז סוג ממשק לא חוקי. הגדרות חוקיות הם COM, CORBA או DEFAULT
% The interface type that was specified is not supported
scan_w_appid_not_support=02059_W_APPID נתמך רק בPALMOS
% The \var{\{\$APPID\}} directive is only supported for the PalmOS target.
scan_w_appname_not_support=02060_W_APPNAME נתמך רק בPALMOS
% The \var{\{\$APPNAME\}} directive is only supported for the PalmOS target.
scan_e_string_exceeds_255_chars=02061_E_מחרוזת קבועה לא יכולה להיות מעל 255 תווים
% A single string constant can contain at most 255 chars. Try splitting up the
% string in multiple smaller parts and concatenate them with a + operator.
scan_f_include_deep_ten=02062_F_הוספת קבצי הinclude גלש מעומק ה16
% When including include files the files have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_e_too_many_push=02063_F_יותר מידי שכבות PUSH
% A maximum of 20 levels is allowed. This error occurs only in mode MacPas.
scan_e_too_many_pop=02064_E_POP ללא שחרור של PUSH
% This error occurs only in mode MacPas.
scan_e_error_macro_lacks_value=02065_E_מקרו "$1" לא מכיל ערכים
% Thus the conditional compile time expression cannot be evaluated.
scan_e_wrong_switch_toggle_default=02066_E_שימוש במתג שגוי, השתמש ב ON/OFF/DEFAULT או ב+/-/*
% You need to use ON or OFF or DEFAULT or a + or - or * to toggle the switch
scan_e_mode_switch_not_allowed=02067_E_מצב מתג "$1" לא מורשה
% A mode switch has already been encountered, or, in case of option -Mmacpas,
% a mode switch occur after UNIT.
scan_e_error_macro_undefined=02068_E_משתנה זמן הידור "$1" לא הוגדר.
% Thus the conditional compile time expression cannot be evaluated. Only in mode MacPas.
scan_e_utf8_bigger_than_65535=02069_E_נמצא קוד UTF-8 גבוהה יותר מ65535
% \fpc handles utf-8 strings internally as widestrings e.g. the char codes are limited to 65535
scan_e_utf8_malformed=02070_E_מחרוזת UTF-8 תקולה
% The given string isn't a valid UTF-8 string
scan_c_switching_to_utf8=02071_C_UTF-8 signature found, using UTF-8 encoding
% The compiler found an UTF-8 encoding signature (\$ef, \$bb, \$bf) at the beginning of a file,
% so it interprets it as an UTF-8 file
scan_e_compile_time_typeerror=02072_E_ביטוי בזמן הידור: נדרש $1 אבל התקבל $2 ב$3
% Type check of a compile time expression failed.
scan_n_app_type_not_support=02073_N_APPTYPE לא נתמך במערכת ההפעלה המבוקשת
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
scan_e_illegal_optimization_specifier=02074_E_סופק מיטוב לא חוקי "$1"
% When you specify an optimization with the \var{\{\$OPTIMIZATION xxx\}}
% the compiler didn't recognize the optimization you specified.
scan_w_setpeflags_not_support=02075_W_SETPEFLAGS אינו נתמך במערכת ההפעלה המבוקשת
% The \var{\{\$SETPEFLAGS\}} directive is not supported by the target OS
scan_w_imagebase_not_support=02076_W_IMAGEBASE אינו נתמך על ידי מערכת ההפעלה המבוקשת
% The \var{\{\$IMAGEBASE\}} directive is not supported by the target OS
scan_w_minstacksize_not_support=02077_W_MINSTACKSIZE אינו נתמך על ידי מערכת ההפעלה המבוקשת
% The \var{\{\$MINSTACKSIZE\}} directive is not supported by the target OS
scan_w_maxstacksize_not_support=02078_W_MAXSTACKSIZE אינו נתמך על ידי מערכת ההפעלה המבוקשת
% The \var{\{\$MAXSTACKSIZE\}} directive is not supported by the target OS
scanner_e_illegal_warn_state=02079_E_שימוש לא חוקי בהנחית $WARN
% Only ON and OFF can be used as state with a \$warn compiler directive
scan_e_only_packset=02080_E_ערך של אריזה לא חוקי
% Only 0, 1, 2, 4, 8, DEFAULT and NORMAL are allowed as packset parameter
scan_w_pic_ignored=02081_W_מתעלם מהוראת PIC
% Several targets like windows do not support neither need PIC so the PIC directive and switch are
% ignored.
scan_w_unsupported_switch_by_target=02082_W_המתג "$1" אינו נתמך בסוג תוצאה שנבחר
% Some compiler switches like \$E are not supported by all targets.
scan_w_frameworks_darwin_only=02084_W_אפשרויות מבוססות מסגרת נתמכות רק עבור Darwin/Mac OS X
% Frameworks are not a known concept, or at least not supported by FPC, on operating systems other than Darwin/Mac OS X.
scan_e_illegal_minfpconstprec=02085_E_דיוק הערך הקבוע של הנקודה העשרונית המינימלית "$1" אינו חוקי
% Valid minimal precisions for floating point constants are default, 32 and 64, which mean respectively minimal (usually 32 bit), 32 bit and 64 bit precision.
% \end{description}
#
# Parser
#
# 03192 is the last used one
#
% \section{Parser messages}
% This section lists all parser messages. The parser takes care of the
% semantics of you language, i.e. it determines if your pascal constructs
% are correct.
% \begin{description}
parser_e_syntax_error=03000_E_מפרש - שגיאה תחבירית
% An error against the Turbo Pascal language was encountered. This happens
% typically when an illegal character is found in the sources file.
parser_e_dont_nest_interrupt=03004_E_פרוצדורת פסיקה אינה יכולה להיות מקוננת
% An \var{INTERRUPT} procedure must be global.
parser_w_proc_directive_ignored=03005_W_מתעלם מסוג פרוצדורה "$1"
% The specified procedure directive is ignored by FPC programs.
parser_e_no_overload_for_all_procs=03006_E_לא כל ההכזרות של "$1" בעלות הכרזת OVERLOAD
% When you want to use overloading using the \var{OVERLOAD} directive, then
% all declarations need to have \var{OVERLOAD} specified.
parser_e_export_name_double=03008_E_כפילויות בייצוא שם הפונקציה "$1"
% Exported function names inside a specific DLL must all be different
parser_e_export_ordinal_double=03009_E_כפילויות בייצוא אינדקס הפונקציה "$1"
% Exported function names inside a specific DLL must all be different
parser_e_export_invalid_index=03010_E_אינדקס לא תקין לייצוא הפונקציה
% DLL function index must be in the range \var{1..\$FFFF}
parser_w_parser_reloc_no_debug=03011_W_שינוי מיקום נתוני הניפוי לא עובדים לקבצי DLL או קבצי הרצה. התבטל.
parser_w_parser_win32_debug_needs_WN=03012_W_לאפשר ניפוי שגיאות לwin32 יש לבטל את שינוי המיקום ע"י האפשרות -WN
% Stabs info is wrong for relocatable DLL or EXES use -WN
% if you want to debug win32 executables.
parser_e_constructorname_must_be_init=03013_E_שם יוצר חייב להיות INIT
% You are declaring an object constructor with a name which is not \var{init}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_destructorname_must_be_done=03014_E_שם הורס חייב להיות DONE
% You are declaring an object destructor with a name which is not \var{done}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_proc_inline_not_supported=03016_E_פרוצדורת INLINE לא נתמכת
% You tried to compile a program with C++ style inlining, and forgot to
% specify the \var{-Si} option (\seeo{Si}). The compiler doesn't support C++
% styled inlining by default.
parser_w_constructor_should_be_public=03018_W_יוצר צריך להיות ציבורי
% Constructors must be in the 'public' part of an object (class) declaration.
parser_w_destructor_should_be_public=03019_W_הורס צריך להיות ציבורי
% Destructors must be in the 'public' part of an object (class) declaration.
parser_n_only_one_destructor=03020_N_מחלקה צריכה הורס אחד בלבד
% You can declare only one destructor for a class.
parser_e_no_local_objects=03021_E_הגדרות מחלקה מקומיות אינם מורשות
% Classes must be defined globally. They cannot be defined inside a
% procedure or function
parser_f_no_anonym_objects=03022_F_הגדרות מחלקה אנונימיות אינם מורשות
% An invalid object (class) declaration was encountered, i.e. an
% object or class without methods that isn't derived from another object or
% class. For example:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% will trigger this error.
parser_n_object_has_no_vmt=03023_N_האובייקט "$1" ללא VMT
% This is a note indicating that the declared object has no
% virtual method table.
parser_e_illegal_parameter_list=03024_E_רשימת פרמטרים לא חוקית
% You are calling a function with parameters that are of a different type than
% the declared parameters of the function.
parser_e_wrong_parameter_size=03026_E_מספר שגוי של פרמטרים צוין עבור הקריאה ל "$1"
% There is an error in the parameter list of the function or procedure,
% the number of parameters is not correct.
parser_e_overloaded_no_procedure=03027_E_מזהה המרובה שימוש "$1" אינו פונקציה
% The compiler encountered a symbol with the same name as an overloaded
% function, but it is not a function it can overload.
parser_e_overloaded_have_same_parameters=03028_E_פונקציות מרובות שימוש מכילות את אותם רשימת פרמטרים
% You're declaring overloaded functions, but with the same parameter list.
% Overloaded function must have at least 1 different parameter in their
% declaration.
parser_e_header_dont_match_forward=03029_E_הגדרת הפונקציה אינה תואמת את ההגדרה הקודמת "$1"
% You declared a function with same parameters but
% different result type or function modifiers.
parser_e_header_different_var_names=03030_E_הגדרת הפונקציה "$1" אינה תואמת את המימוש: שם משתנה השתנה $2 => $3
% You declared the function in the \var{interface} part, or with the
% \var{forward} directive, but define it with a different parameter list.
parser_n_duplicate_enum=03031_N_ערכים בסוג מניה חייבים להיות בסדר עולה
% \fpc allows enumeration constructions as in C. Given the following
% declaration two declarations:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% The second declaration would produce an error. \var{A\_UAS} needs to have a
% value higher than \var{A\_E}, i.e. at least 7.
parser_e_no_with_for_variable_in_other_segments=03033_E_With אינו מסוגל להשתמש במשתנים ממקטע שונה
% With stores a variable locally on the stack,
% but this is not possible if the variable belongs to another segment.
parser_e_too_much_lexlevel=03034_E_פונקציה מקוננת > 31
% You can nest function definitions only 31 times.
parser_e_range_check_error=03035_E_שגיאה בבדיקת טווח בעת הערכת קבועים
% The constants are out of their allowed range.
parser_w_range_check_error=03036_W_שגיאה בבדיקת טווח בעת הערכת קבועים
% The constants are out of their allowed range.
parser_e_double_caselabel=03037_E_הכזרת תווית כפולה בcase
% You are specifying the same label 2 times in a \var{case} statement.
parser_e_case_lower_less_than_upper_bound=03038_E_הטווח הסופי קטן מהטווח ההתחלתי
% The upper bound of a \var{case} label is less than the lower bound and this
% is useless
parser_e_type_const_not_possible=03039_E_הכרזת טיפוס קבועים של מחלקות או ממשקים אסורה
% You cannot declare a constant of type class or object.
parser_e_no_overloaded_procvars=03040_E_משתנים של פונקציות מרובות שימוש אינם מורשות
% You are trying to assign an overloaded function to a procedural variable.
% This is not allowed
parser_e_invalid_string_size=03041_E_אורך מחרוזת חייב להיות עם ערכים מ1 ל255
% The length of a shortstring in Pascal is limited to 255 characters. You are
% trying to declare a string with length lower than 1 or greater than 255
parser_w_use_extended_syntax_for_objects=03042_W_השתמש בתחביר מורחב של NEW וDISPOSE לעותקים של אובייקט
% If you have a pointer \var{a} to a class type, then the statement
% \var{new(a)} will not initialize the class (i.e. the constructor isn't
% called), although space will be allocated. you should issue the
% \var{new(a,init)} statement. This will allocate space, and call the
% constructor of the object
parser_w_no_new_dispose_on_void_pointers=03043_W_השימוש במצביע לא מוגדר עם NEW ו DISPOSE חסר משמעות
parser_e_no_new_dispose_on_void_pointers=03044_E_בלתי אפשרי להשתמש במצביע לא מוגדר עם NEW או DISPOSE
% You cannot use \var{new(p)} or \var{dispose(p)} if \var{p} is an untyped pointer
% because no size is associated to an untyped pointer.
% Accepted for compatibility in \var{tp} and \var{delphi} modes.
parser_e_class_id_expected=03045_E_מצפה למזהה מחלקה
% This happens when the compiler scans a procedure declaration that contains
% a dot,
% i.e., a object or class method, but the type in front of the dot is not
% a known type.
parser_e_no_type_not_allowed_here=03046_E_סוג מזהה לא מורשה במיקום הנוכחי
% You cannot use a type inside an expression.
parser_e_methode_id_expected=03047_E_מצפה למזהה מתודי
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_e_header_dont_match_any_member=03048_E_הגדרת הפונקציה לא זהה למתודות של המחלקה "$1"
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_d_procedure_start=03049_DL_פרוצדורה/פונקציה $1
% When using the \var{-vd} switch, the compiler tells you when it starts
% processing a procedure or function implementation.
parser_e_error_in_real=03050_E_קבוע של מספר ממשי אינו חוקי
% The compiler expects a floating point expression, and gets something else.
parser_e_fail_only_in_constructor=03051_E_FAIL יכול להיות בשימוש רק בתוך יוצר
% You are using the \var{fail} keyword outside a constructor method.
parser_e_no_paras_for_destructor=03052_E_הורסים לא יכולים להכיל פרמטרים
% You are declaring a destructor with a parameter list. Destructor methods
% cannot have parameters.
parser_e_only_class_methods_via_class_ref=03053_E_רק מתודות מחלקה יכולות להיות מוגדרות בהתייחסות מחלקה
% This error occurs in a situation like the following:
% \begin{verbatim}
% Type :
%    Tclass = Class of Tobject;
%
% Var C : TClass;
%
% begin
% ...
% C.free
% \end{verbatim}
% \var{Free} is not a class method and hence cannot be called with a class
% reference.
parser_e_only_class_methods=03054_E_רק מתודת מחלקה יכולה לגשת למתודת מחלקה
% This is related to the previous error. You cannot call a method of an object
% from a inside a class method. The following code would produce this error:
% \begin{verbatim}
% class procedure tobject.x;
%
% begin
%   free
% \end{verbatim}
% Because free is a normal method of a class it cannot be called from a class
% method.
parser_e_case_mismatch=03055_E_קבועים וסוג CASE אינם מתאימים
% One of the labels is not of the same type as the case variable.
parser_e_illegal_symbol_exported=03056_E_לא ניתן לייצא את הסמל מספרייה
% You can only export procedures and functions when you write a library. You
% cannot export variables or constants.
parser_w_should_use_override=03057_W_מתודת ירושה נסתרת ע"י "$1"
% A method that is declared \var{virtual} in a parent class, should be
% overridden in the descendent class with the \var{override} directive. If you
% don't specify the \var{override} directive, you will hide the parent method;
% you will not override it.
parser_e_nothing_to_be_overridden=03058_E_אין מתודה  במחלקה של האב אשר ניתן לשכתב: "$1"
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_no_procedure_to_access_property=03059_E_לא ניתן מספר לגשת למאפיין
% You specified no \var{read} directive for a property.
parser_w_stored_not_implemented=03060_W_מאפיין שמור אינו מבוצע עדיין
% The \var{stored} directive is not yet implemented
parser_e_ill_property_access_sym=03061_E_סימן לא חוקי לגישת המאפיין
% There is an error in the \var{read} or \var{write} directives for an array
% property. When you declare an array property, you can only access it with
% procedures and functions. The following code would cause such an error.
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_לא ניתן לגשת לשדה מוגן באובייקט הנוכחי
% Fields that are declared in a \var{protected} section of an object or class
% declaration cannot be accessed outside the module where the object is
% defined, or outside descendent object methods.
parser_e_cant_access_private_member=03063_E_לא ניתן לגשת לשדה פרטי באובקייט הנוכחי
% Fields that are declared in a \var{private} section of an object or class
% declaration cannot be accessed outside the module where the class is
% defined.
parser_e_overridden_methods_not_same_ret=03066_E_מתודה מרובת שימוש חייבת להיות זהה בסוג המשתנה: "$2" בשימוש ע"י "$1", אשר מכילה סוג משתנה אחר
% If you declare overridden methods in a class definition, they must
% have the same return type.
parser_e_dont_nest_export=03067_E_הגדדרת פונקציות EXPORT לא יכולות להיות מקוננות
% You cannot declare a function or procedure within a function or procedure
% that was declared as an export procedure.
parser_e_methods_dont_be_export=03068_E_אי אפשר לייצא את המתודה
% You cannot declare a procedure that is a method for an object as
% \var{export}ed.
parser_e_call_by_ref_without_typeconv=03069_E_השימוש בפרמטר של משתנה חייב להיות זהה: מכיל "$1" מצפה ל"$2"
% When calling a function declared with \var{var} parameters, the variables in
% the function call must be of exactly the same type. There is no automatic
% type conversion.
parser_e_no_super_class=03070_E_המחלקה איננה מחלקת אב של המחלקה הנוכחית
% When calling inherited methods, you are trying to call a method of a non-related
% class. You can only call an inherited method of a parent class.
parser_e_self_not_in_method=03071_E_SELF מורשה רק במתודות
% You are trying to use the \var{self} parameter outside an object's method.
% Only methods get passed the \var{self} parameters.
parser_e_generic_methods_only_in_methods=03072_E_מתודות יכולות להיקרא ישירות רק בתוך מתודות אחרות עם סוג מזהה שלהמחלקה
% A construction like \var{sometype.somemethod} is only allowed in a method.
parser_e_illegal_colon_qualifier=03073_E_שימוש לא חוקי של ':'
% You are using the format \var{:} (colon) 2 times on an expression that
% is not a real expression.
parser_e_illegal_set_expr=03074_E_שגיאה בבדיקת טווח בסדרת יוצרים או שכפול סדרה
% The declaration of a set contains an error. Either one of the elements is
% outside the range of the set type, either two of the elements are in fact
% the same.
parser_e_pointer_to_class_expected=03075_E_מצפה למצביע אובייקט
% You specified an illegal type in a \var{new} statement.
% The extended syntax of \var{new} needs an object as a parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_הביטוי חייב להיות קריאת יוצר
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the object you are trying to create. The procedure you specified
% is not a constructor.
parser_e_expr_have_to_be_destructor_call=03077_E_הביטוי חייב להיות קריאת הורס
% When using the extended syntax of \var{dispose}, you must specify the
% destructor method of the object you are trying to dispose of.
% The procedure you specified is not a destructor.
parser_e_invalid_record_const=03078_E_סדר לא חוקי של אלמנטים ברשומה
% When declaring a constant record, you specified the fields in the wrong
% order.
parser_e_false_with_expr=03079_E_סוג ביטוי חייב להיות ביטוי רשומה או מחלקה
% A \var{with} statement needs an argument that is of the type \var{record}
% or \var{class}. You are using \var{with} on an expression that is not of
% this type.
parser_e_void_function=03080_E_פרוצדורה לא יכולה להחזיר ערך
% In \fpc, you can specify a return value for a function when using
% the \var{exit} statement. This error occurs when you try to do this with a
% procedure. Procedures cannot return a value.
parser_e_only_methods_allowed=03081_E_היוצרים וההורסים חייבים להיות מתודיים
% You're declaring a procedure as destructor, constructor or class operator, when the
% procedure isn't a class method.
parser_e_operator_not_overloaded=03082_E_האופריטור לא מרובה שימוש
% You're trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_no_such_assignment=03083_E_לא ניתן להציב ערכים לסוגים מרובי שימוש
% You can not overload assignment for types
% that the compiler considers as equal.
parser_e_overload_impossible=03084_E_בלתי אפשרי להשתמש באופרטור מרובה שימוש
% The combination of operator, arguments and return type are
% incompatible.
parser_e_no_reraise_possible=03085_E_לא ניתן להרים חריגה
% You are trying to raise an exception where it is not allowed. You can only
% raise exceptions in an \var{except} block.
parser_e_no_new_or_dispose_for_classes=03086_E_התחביר המורחב של NEW או DISPOSE לא מורשה למחלקות
% You cannot generate an instance of a class with the extended syntax of
% \var{new}. The constructor must be used for that. For the same reason, you
% cannot call \var{dispose} to de-allocate an instance of a class, the
% destructor must be used for that.
parser_e_procedure_overloading_is_off=03088_E_האפשרות לפרוצדורות מרובות שימוש כבוי
% When using the \var{-So} switch, procedure overloading is switched off.
% Turbo Pascal does not support function overloading.
parser_e_overload_operator_failed=03089_E_לא ניתן להעמיס על סוג האופרטור הנוכחי. העמסת האופרטורים הקשורים לפעולה (אם בכלל) הם: $1
% You are trying to overload an operator which cannot be overloaded.
% The following operators can be overloaded :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_אופרטור השוואתי חייב להחזיר ערך בוליאני
% When overloading the \var{=} operator, the function must return a boolean
% value.
parser_e_only_virtual_methods_abstract=03091_E_רק שיטה ווירטואלית יכולה להיות מופשט
% You are declaring a method as abstract, when it is not declared to be
% virtual.
parser_f_unsupported_feature=03092_F_שימוש באופציה לא נתמכת!
% You're trying to force the compiler into doing something it cannot do yet.
parser_e_mix_of_classes_and_objects=03093_E_הערבוב של עצמים שונים (מחלקות, עצם, ממשק וכו') אינו מורשה
% You cannot derive \var{objects}, \var{classes}, \var{cppclasses} and \var{interfaces} interttwined . E.g.
% a class cannot have an object as parent and vice versa.
parser_w_unknown_proc_directive_ignored=03094_W_מתעלם מהנחית השגרה: "$1"
% The procedure directive you specified is unknown.
parser_e_absolute_only_one_var=03095_E_absolute ניתן לשימוש רק עם משתנה אחת
% You cannot specify more than one variable before the \var{absolute} directive.
% Thus, the following construct will provide this error:
% \begin{verbatim}
% Var Z : Longint;
%     X,Y : Longint absolute Z;
% \end{verbatim}
% \item [ absolute can only be associated a var or const ]
% The address of a \var{absolute} directive can only point to a variable or a
% typed constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_absolute_only_to_var_or_const=03096_E_ניתן להשתמש בabsolute רק עם משתנה או קבוע
% The address of a \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_initialized_only_one_var=03097_E_ניתן לאתחל רק משתנה אחד
% You cannot specify more than one variable with a initial value
% in Delphi mode.
parser_e_abstract_no_definition=03098_E_מתודה מופשטת אינה זקוקה להגדרה (עם גוף של פונקציה)
% Abstract methods can only be declared, you cannot implement them. They
% should be overridden by a descendant class.
parser_e_overloaded_must_be_all_global=03099_E_העמסת היתר של הפונקציה אינה יכולה להיות מקומית (חייבת להיות מיוצאת)
% You are defining a overloaded function in the implementation part of a unit,
% but there is no corresponding declaration in the interface part of the unit.
parser_w_virtual_without_constructor=03100_W_שימוש במתודות ווירטואליות ללא יוצר ב "$1"
% If you declare objects or classes that contain virtual methods, you need
% to have a constructor and destructor to initialize them. The compiler
% encountered an object or class with virtual methods that doesn't have
% a constructor/destructor pair.
parser_c_macro_defined=03101_CL_הגדרת מאקרו: $1
% When \var{-vc} is used, the compiler tells you when it defines macros.
parser_c_macro_undefined=03102_CL_מאקרו לא מוגדר: $1
% When \var{-vc} is used, the compiler tells you when it undefines macros.
parser_c_macro_set_to=03103_CL_מאקרו $1 מוגדר כ $2
% When \var{-vc} is used, the compiler tells you what values macros get.
parser_i_compiling=03104_I_מהדר $1
% When you turn on information messages (\var{-vi}), the compiler tells you
% what units it is recompiling.
parser_u_parsing_interface=03105_UL_מפרש את הממשק של היחידה $1
% This tells you that the reading of the interface
% of the current unit starts
parser_u_parsing_implementation=03106_UL_מפרש את הביצוע של $1
% This tells you that the code reading of the implementation
% of the current unit, library or program starts
parser_d_compiling_second_time=03107_DL_מהדר את $1 בפעם השנייה
% When you request debug messages (\var{-vd}) the compiler tells you what
% units it recompiles for the second time.
parser_e_no_property_found_to_override=03109_E_לא נמצאה תכונה לעקיפה
% You want to override a property of a parent class, when there is, in fact,
% no such property in the parent class.
parser_e_only_one_default_property=03110_E_רק תכונה אחת עם ברירת מחדל מורשת
% You specified a property as \var{Default}, but the class already has a
% default property, and a class can have only one default property.
parser_e_property_need_paras=03111_E_התונה עם ברירת מחדל חייבת להיות תכונה של מערך
% Only array properties of classes can be made \var{default} properties.
parser_e_constructor_cannot_be_not_virtual=03112_E_יוצרים ווירטואלים נתמכים רק במחלקה
% You cannot have virtual constructors in objects. You can only have them
% in classes.
parser_e_no_default_property_available=03113_E_ערך ברירת מחדל אינו קיים לתכונה
% You are trying to access a default property of a class, but this class (or one of
% its ancestors) doesn't have a default property.
parser_e_cant_have_published=03114_E_המחלקה אינה יכולה להכיל את חתך published, השתמש במתג {$M+}
% If you want a \var{published} section in a class definition, you must
% use the \var{\{\$M+\}} switch, whch turns on generation of type
% information.
parser_e_forward_declaration_must_be_resolved=03115_E_נדרשת הגדרה ראשונית של המחלקה "$1" לפני השימוש בה כמחלקה יורשת
% To be able to use an object as an ancestor object, it must be defined
% first. This error occurs in the following situation:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% Where \var{ParentClass} is declared but not defined.
parser_e_no_local_operator=03116_E_השימוש באופריטור מקומי אינו נתמך
% You cannot overload locally, i.e. inside procedures or function
% definitions.
parser_e_proc_dir_not_allowed_in_interface=03117_E_מקדם שיגרה "$1" אינו מורשה בשימוש בתוך הממשק
% This procedure directive is not allowed in the \var{interface} section of
% a unit. You can only use it in the \var{implementation} section.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_מקדם שיגרה "$1" אינו מורשה בחלק הביצועי
% This procedure directive is not defined in the \var{implementation} section of
% a unit. You can only use it in the \var{interface} section.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_מקדם שיגרה "$1" אינו מורשה כחלק מהגדרת השיגרה
% This procedure directive cannot be part of a procedural or function
% type declaration.
parser_e_function_already_declared_public_forward=03120_E_הגדרת הפונקציה "$1" כבר בוצעה
% You will get this error if a function is defined as \var{forward} twice.
% Or it is once in the \var{interface} section, and once as a \var{forward}
% declaration in the \var{implmentation} section.
parser_e_not_external_and_export=03121_E_לא ניתן להשתמש גם ב EXPORT וגם EXTERNAL
% These two procedure directives are mutually exclusive
parser_n_not_supported_for_inline=03123_N_"$1" אינו נתמך עדיין בתוך פונקצית/שגרת inline
% Inline procedures don't support this declaration.
parser_h_inlining_disabled=03124_H_השימוש הישיר מבוטל
% Inlining of procedures is disabled.
parser_i_writing_browser_log=03125_I_כותב יומן דפדפן $1
% When information messages are on, the compiler warns you when it
% writes the browser log (generated with the \var{\{\$Y+ \}} switch).
parser_h_maybe_deref_caret_missing=03126_H_ייתכן כי חסר תוכן למצביע
% The compiler thinks that a pointer may need a dereference.
parser_f_assembler_reader_not_supported=03127_F_אין תמיכה במאסף הנבחר
% The selected assembler reader (with \var{\{\$ASMMODE xxx\}} is not
% supported. The compiler can be compiled with or without support for a
% particular assembler reader.
parser_e_proc_dir_conflict=03128_E_מקדם השיגרה "$1" מתנגש עם מקדמים אחרים
% You specified a procedure directive that conflicts with other directives.
% for instance \var{cdecl} and \var{pascal} are mutually exclusive.
parser_e_call_convention_dont_match_forward=03129_E_השימוש בהגדרה הנוכחית אינו זהה להגדרה הראשונית
% This error happens when you declare a function or procedure with
% e.g. \var{cdecl;} but omit this directive in the implementation, or vice
% versa. The calling convention is part of the function declaration, and
% must be repeated in the function definition.
parser_e_property_cant_have_a_default_value=03131_E_תכונות אינן יכולות להכיל ערך ברירת מחדל
% Set properties or indexed properties cannot have a default value.
parser_e_property_default_value_must_const=03132_E_ערך ברירת המחדלק שת התכונה חייב להיות קבוע
% The value of a \var{default} declared property must be known at compile
% time. The value you specified is only known at run time. This happens
% .e.g. if you specify a variable name as a default value.
parser_e_cant_publish_that=03133_E_הסמל אינו יכול להיות בשימוש בתוך מחלקה ללא הגדרתו בתוך המחלקה
% Only class type variables can be in a \var{published} section of a class
% if they are not declared as a property.
parser_e_cant_publish_that_property=03134_E_סוג זה של תכונה אינו יכול להיות בשימוש
% Properties in a \var{published} section cannot be array properties.
% they must be moved to public sections. Properties in a \var{published}
% section must be an ordinal type, a real type, strings or sets.
parser_e_empty_import_name=03136_E_נדרש שם ייבוא
% Some targets need a name for the imported procedure or a \var{cdecl} specifier
parser_e_division_by_zero=03138_E_חלוק באפס
% There is a division by zero encounted
parser_e_invalid_float_operation=03139_E_פעולת עשרונית לא חוקית
% An operation on two real type values produced an overflow or a division
% by zero.
parser_e_array_lower_less_than_upper_bound=03140_E_הגבול העליון נמוך יותר מהגבול התחתון
% The upper bound of a an array declaration is less than the lower bound and this
% is not possible
parser_w_string_too_long=03141_W_אורך המחרוזת של "$1" ארוך יותר מ "$1"
% The size of the constant string is larger than the size you specified in
% string type definition
parser_e_string_larger_array=03142_E_אורך המחרוזת ארוך יותר מאורך מערך התווים
% The size of the constant string is larger than the size you specified in
% the array[x..y] of char definition
parser_e_ill_msg_expr=03143_E_התוכן של הביטוי לאחר ההודעה אינו חוקי
% \fpc supports only integer or string values as message constants
parser_e_ill_msg_param=03144_E_טיפול בהודעות חייב להיות במבנה קבוע של פרמטרים
% A method declared with the \var{message}-directive as message handler
% can take only one parameter which must be declared as call by reference
% Parameters are declared as call by reference using the \var{var}-directive
parser_e_duplicate_message_label=03145_E_תווית הודעה כפולה: "$1"
% A label for a message is used twice in one object/class
parser_e_self_in_non_message_handler=03146_E_Self חייב להיות פרמטר במתודה של מטפל ההודעות
% The self parameter can only be passed explicitly to a method which
% is declared as message handler.
parser_e_threadvars_only_sg=03147_E_Threadvars חייבים להיות סטטיים או גלובליים
% Threadvars must be static or global, you can't declare a thread
% local to a procedure. Local variables are always local to a thread,
% because every thread has its own stack and local variables
% are stored on the stack
parser_f_direct_assembler_not_allowed=03148_F_השימוש במאסף הנוכחי אינו תומך בוסג התוצאה הבינארית
% You can't use direct assembler when using a binary writer, choose an
% other outputformat or use another assembler reader
parser_w_no_objpas_use_mode=03149_W_אסור לקרוא ליחידת OBJPAS ישירות, יש להשתמש ב \{\$mode objfpc\} או ב \{\$mode delphi\} במקום
% You are trying to load the ObjPas unit manually from a uses clause. This is
% not a good idea. Use the \var{\{\$mode objfpc\}} or
% \var{\{\$mode delphi\}}
% directives which load the unit automatically
parser_e_no_object_override=03150_E_OVERRIDE אינו יכול להיות בשימוש באובייקטים
% Override is not supported for objects, use \var{virtual} instead to override
% a method of a parent object
parser_e_cant_use_inittable_here=03151_E_טיפוסי נתונים אשר דורשים אתחול או סיכום אינם יכולים להיות ברשומות variant
% Some data type (e.g. \var{ansistring}) needs initialization/finalization
% code which is implicitly generated by the compiler. Such data types
% can't be used in the variant part of a record.
parser_e_resourcestring_only_sg=03152_E_לא ניתן להגדיר Resourcestring בתור הגדרה מקומית, רק הגדרה גלובלית או סטטית
% Resourcestring can not be declared local, only global or using the static
% directive.
parser_e_exit_with_argument_not__possible=03153_E_השימוש בשיגרה exit אינו יכול להתבצע במיקום הנוכחי
% an exit statement with an argument for the return value can't be used here, this
% can happen e.g. in \var{try..except} or \var{try..finally} blocks
parser_e_stored_property_must_be_boolean=03154_E_טיפוס הנתונים בסימול האחסון חייב להיות בוליאני
% If you specify a storage symbol in a property declaration, it must be of
% the type boolean
parser_e_ill_property_storage_sym=03155_E_הסימול אינו יכול להשמש כסימול לאחסון
% You can't use this type of symbol as storage specifier in property
% declaration. You can use only methods with the result type boolean,
% boolean class fields or boolean constants
parser_e_only_publishable_classes_can_be_published=03156_E_רק מחלקה המהודרת במצב $M+ יכולה להיות איזור ה published
% In the published section of a class can be only class as fields used which
% are compiled in \var{\{\$M+\}} or which are derived from such a class. Normally
% such a class should be derived from TPersitent
parser_e_proc_directive_expected=03157_E_מצפה להנחיית שיגרה
% This error is triggered when you have a \var{\{\$Calling\}} directive without
% a calling convention specified.
% It also happens when declaring a procedure in a const block and you
% used a ; after a procedure declaration which must be followed by a
% procedure directive.
% Correct declarations are:
% \begin{verbatim}
% const
%   p : procedure;stdcall=nil;
%   p : procedure stdcall=nil;
% \end{verbatim}
parser_e_invalid_property_index_value=03158_E_הערך האינדקס של התכונה חייב להיות טיפוס מסודר
% The value you use to index a property must be of an ordinal type, for
% example an integer or enumerated type.
parser_e_procname_to_short_for_export=03159_E_שם השגרה קצר מידי לייצוא
% The length of the procedure/function name must be at least 2 characters
% long. This is because of a bug in dlltool which doesn't parse the .def
% file correct with a name of length 1.
parser_e_dlltool_unit_var_problem=03160_E_לא ניתן לייצר חלק DEFFILE למשתנים גלובליים ביחידה
parser_e_dlltool_unit_var_problem2=03161_E_הדר ללא שימוש ב -WD
% You need to compile this file without the -WD switch on the
% commandline
parser_f_need_objfpc_or_delphi_mode=03162_F_יש להשתמש במצב ObjFpc (-S2) או במצב Delphi (-Sd) להדר את המודול
% You need to use \{\$mode objfpc\} or \{\$mode delphi\} to compile this file.
% Or use the equivalent commandline switches -S2 or -Sd.
parser_e_no_export_with_index_for_target=03163_E_אי אפשר לייצא עם אינדקס תחת $1
% Exporting of functions or procedures with a specified index is not
% supported on this target.
parser_e_no_export_of_variables_for_target=03164_E_ייצוא של משתמש אינו נתמך ב$1
% Exporting of variables is not supported on this target.
parser_e_improper_guid_syntax=03165_E_תחביר GUID אינו חוקי
% The GUID indication does not have the proper syntax. It should be of the form
% \begin{verbatim}
% {XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX}
% \end{verbatim}
% Where each \var{X} represents a hexadecimal digit.
parser_w_interface_mapping_notfound=03168_W_השגרה "$1" נמצאה אך ללא הפרמטרים המבוקשים של $2.$3
% The compiler cannot find a suitable procedure which implements the given method of an interface.
% A procedure with the same name is found, but the arguments do not match.
parser_e_interface_id_expected=03169_E_מצפה למזהה ממשק
% This happens when the compiler scans a \var{class} declaration that contains
% \var{interface} function name mapping code like this:
% \begin{verbatim}
% type
%   TMyObject = class(TObject, IDispatch)
%     function IUnknown.QueryInterface=MyQueryInterface;
%     ....
% \end{verbatim}
% and the \var{interface} before the dot not listed in the inheritance list.
parser_e_type_cant_be_used_in_array_index=03170_E_הסוג "$1" אינו יכול להיות בשימוש בתור סוג אינדקס למערך
% Types like \var{qword} or \var{int64} aren't allowed as array index type
parser_e_no_con_des_in_interfaces=03171_E_יוצר והורס אינם מורשים בממשקים
% Constructor and destructor declarations aren't allowed in interface
% In the most cases the method \var{QueryInterface} of \var{IUnknown} can
% be used to create a new interface.
parser_e_no_access_specifier_in_interfaces=03172_E_מזהה כניסה אינו יכול להיות בשימוש עם ממשקים
% The access specifiers \var{public}, \var{private}, \var{protected} and
% \var{pusblished} can't be used in interfaces because all methods
% of an interfaces must be public.
parser_e_no_vars_in_interfaces=03173_E_ממשק אינו יכול להכיל שדות
% Declarations of fields aren't allowed in interfaces. An interface
% can contain only methods
parser_e_no_local_proc_external=03174_E_לא ניתן להגדיר שגרה מקומית כחיצונית
% Declaring local procedures as external is not possible. Local procedures
% get hidden parameters that will make the chance of errors very high
parser_w_skipped_fields_before=03175_W_חלק מהשדות הבאים לפני "$1" לא אותחלו
% In Delphi mode, not all fields of a typed constant record have to be
% initialized, but the compiler warns you when it detects such situations.
parser_e_skipped_fields_before=03176_E_חלק מהשדות הבאים לפני "$1" לא אותחלו
% In all syntax modes but Delphi mode, you can't leave some fields uninitialized
% in the middle of a typed constant record
parser_w_skipped_fields_after=03177_W_חלק מהשדות הבאים אחרי "$1" לא אותחלו
% You can leave some fields at the end of a type constant record uninitialized
% (the compiler will initialize them to zero automatically). This may be the cause
% of subtle problems.
parser_e_varargs_need_cdecl_and_external=03178_E_השימוש בVarArgs (או '...' בMacPas) חייב להיות בשימוש עם CDecl/CPPDecl/MWPascal ו External
% The varargs directive (or the ``...'' varargs parameter in MacPas mode) can only be used with procedures or functions
% that are declared with \var{external} and one of \var{cdecl}, \var{cppdecl} and \var{mwpascal}. This functionality
% is only supported to provide a compatible interface to C functions like printf.
parser_e_self_call_by_value=03179_E_Self חייב להיות פרמטר רגיל (call-by-value)
% You can't declare self as a const or var parameter, it must always be
% a call-by-value parameter
parser_e_interface_has_no_guid=03180_E_הממשק "$1" אינו מכיל מזהה GUID
% When you want to assign an interface to a constant, then the interface
% must have a GUID value set.
parser_e_illegal_field_or_method=03181_E_המזהה של שדה או מתודה של מחלקה "$1" אינם ידועים
% Properties must refer to a field or method in the same class.
parser_w_proc_overriding_calling=03182_W_עולה על מוסכמות הפעלה "$1" עם "$2"
% There are two directives in the procedure declaration that specify a calling
% convention. Only the last directive will be used
parser_e_no_procvarobj_const=03183_E_ניתן לאתחל טיפוס קבוע של "procedure of object" רק עם NIL
% You can't assign the address of a method to a typed constant which has a
% 'procedure of object' type, because such a constant requires two addresses:
% that of the method (which is known at compile time) and that of the object or
% class instance it operates on (which can not be known at compile time).
parser_e_default_value_only_one_para=03184_E_ערך ברירת מחדל יכול להיות בשימוש רק עם פרמטר אחד
% It is not possible to specify a default value for several parameters at once. 
% The following is invalid:
% \begin{verbatim}
% Procedure MyProcedure (A,B : Integer = 0);
% \end{verbatim}
% Instead, this should be declared as
% \begin{verbatim}
% Procedure MyProcedure (A : Integer = 0; B : Integer = 0);
% \end{verbatim}
parser_e_default_value_expected_for_para=03185_E_פרמטר ברירת מחדל דרוש עבור "$1"
% The specified parameter requires a default value.
parser_w_unsupported_feature=03186_W_שימוש במאפיין לא נתמך !
% You're trying to force the compiler into doing something it cannot do yet.
parser_h_c_arrays_are_references=03187_H_מערכים של C מועברים כהפניה
% Any array passed to a C functions is passed
% by a pointer (i.e. by reference).
parser_e_C_array_of_const_must_be_last=03188_E_מערך C של קבוע חייב להיות הארגומנט האחרון
% You can not add any other argument after an \var{array of const} for
% \var{cdecl} functions, as the size pushed on stack for this argument is
% not known.
parser_h_type_redef=03189_H_הגדרה מחודשת של הטיפוס "$1"
% This is an indicator that a previously declared type is
% being redefined as something else. This may, or may not
% be, a cause for errors.
parser_w_cdecl_has_no_high=03190_W_אין לפונקציות cdecl פרמטר גבוהה
% Functions declared with cdecl modifier do not pass an extra implicit parameter.
parser_w_cdecl_no_openstring=03191_W_פונקציות cdecel אינן תומכות ב open string
% Openstring is not supported for cdecl'ared functions.
parser_e_initialized_not_for_threadvar=03192_E_אין אפשרות לאתחל משתנה המוגדר כthreadvar
% Variables declared as threadvar can not be initialized with a default value.
% The variables will always be filled with zero at the start of a new thread.
parser_e_msg_only_for_classes=03193_E_ניתן להשתמש בהנחיית message רק בתוך מחלקה
% The message directive is only supported for Class types.
parser_e_procedure_or_function_expected=03194_E_מצפה לשיגרה או פונקציה
% A class method can only be specified for procedures and functions.
parser_e_illegal_calling_convention=03195_W_מתעלם ממוסכמת ההפעלה: "$1"
% Some calling conventions are supported only by certain CPUs. I.e. most non-i386 ports support
% only the standard ABI calling convention of the CPU.
parser_e_no_object_reintroduce=03196_E_לא ניתן להשתמש בREINTRODUCE בתוך אובייקט
% \var{reintroduce} is not supported for objects.
parser_e_paraloc_only_one_para=03197_E_כל ארגומנט חייב להכיל מיקום עצמאי.
% If locations for arguments are specified explicitly as it is required by
% some syscall conventions, each argument must have it's only location, things
% like \var{procedure p(i,j : longint 'r1');} aren't allowed
parser_e_paraloc_all_paras=03198_E_כל ארגומנט חייב להכיל מיקום מוגדר
% If one argument has an explicit argument location, all arguments of a procedure
% must have one.
parser_e_illegal_explicit_paraloc=03199_E_מיקום ארגומנט לא ידוע
% The location specified for an argument isn't recognized by the compiler
parser_e_32bitint_or_pointer_variable_expected=03200_E_מצפה לסוג משתנה של מספר שלם 32-Bit או מצביע
% The libbase for MorphOS/AmigaOS can be give only as \var{longint}, \var{dword} or any pointer variable.
parser_e_goto_outside_proc=03201_E_לא ניתן להשמש בgoto בין שני שיגרות
% It isn't allowed to use the \var{goto} statements referencing labels outside the
% current procedure. The following example shows the problem:
% \begin{verbatim}
% ...
%   procedure p1;
%   label
%     l1;
%
%     procedure p2;
%     begin
%       goto l1; // This goto ISN'T allowed
%     end;
%
%   begin
%     p2
%   l1:
%   end;
% ...
%
% \end{verbatim}
parser_f_too_complex_proc=03202_F_השיגרה מסובכת מידי ודורשת שימוש של אוגרים רבים מידי
% Your procedure body is too long for the compiler. You should split the
% procedure into multiple smaller procedures.
parser_e_illegal_expression=03203_E_ביטוי לא חוקי
% This can occur under many circumstances. Mostly when trying to evaluate
% constant expressions.
parser_e_invalid_integer=03204_E_הביטוי של המספר השלם אינו חוקי
% You made an expression which isn't an integer, and the compiler expects the
% result to be an integer.
parser_e_invalid_qualifier=03205_E_שימוש לא חוקי במבחין
% One of the following is happening :
% \begin{itemize}
% \item You're trying to access a field of a variable that is not a record.
% \item You're indexing a variable that is not an array.
% \item You're dereferencing a variable that is not a pointer.
% \end{itemize}
parser_e_upper_lower_than_lower=03206_E_טווח ההגבלה הגבוהה קטן מטווח ההגבלה הנמוכה
% You are declaring a subrange, and the lower limit is higher than the high
% limit of the range.
parser_e_macpas_exit_wrong_param=03207_E_הפרמטר של Exit חייב להיות שם השיגרה תחתיו הוא נמצא
% Non local exit is not allowed. This error occurs only in mode MacPas.
parser_e_illegal_assignment_to_count_var=03208_E_הצבה לא חוקית של המשתנה בלולאת ה for "$1"
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings. You can also not assign values to
% loop variables inside the loop (except in Delphi and TP modes). Use a while or
% repeat loop instead if you need to do something like that, since those
% constructs were built for that.
parser_e_no_local_var_external=03209_E_לא ניתן להגדיר משתנים מקומיים כחיצוניים
% Declaring local variables as external is not allowed. Only global variables can reference
% to external variables.
parser_e_proc_already_external=03210_E_השיגרה כבר מוגדרת כחיצונית
% The procedure is already declared with the EXTERNAL directive in an interface or
% forward declaration.
parser_w_implicit_uses_of_variants_unit=03211_W_שימוש משתמע ביחידת Variants
% The Variant type is used in the unit without any used unit using the Variants unit. The
% compiler has implicitly added the Variants unit to the uses list. To remove this warning
% the Variants unit needs to be added to the uses statement.
parser_e_no_static_method_in_interfaces=03212_E_השימוש במחלקה ומתודות סטטיים אינם יכוליות להיות בשימוש בממשק
% The specifier \var{class} and directive \var{static} can't be used in interfaces
% because all methods of an interfaces must be public.
parser_e_arithmetic_operation_overflow=03213_E_גלישה בפעולה מתמטית
% An operation on two integers values produced an overflow
parser_e_protected_or_private_expected=03214_E_מצפה לאיזור Protected או Private
% \var{strict} can be only used together with \var{protected} or \var{private}.
parser_e_illegal_slice=03215_E_SLICE אינ יכול להיות בשימוש מחוץ לרשימת פרמטרים
% \var{slice} can be used only for arguments accepting an open array parameter
parser_e_dispinterface_cant_have_parent=03216_E_מחלקת DISPINTERFACE אינה יכולה להכיל הורה
% A DISPINMTERFACE is a special type of interface which can't have a parent class
parser_e_dispinterface_needs_a_guid=03217_E_DISPINTERFACE חייב GUID
% A DISPINMTERFACE always needs an interface identification
parser_w_overridden_methods_not_same_ret=03218_W_המתודות החדשות חייבות להחזיר את אותו טיפוס נתונים. הקוד הנוכחי יכול לגרום לקריסה בעקבות באג של המפרש של דלפי (“$2” עולה על “$1” אשר מכיל טיפוס נתונים אחר בהחזרה)
% If you declare overridden methods in a class definition, they must
% have the same return type. Some versions of Delphi allow you to change the
% return type of interface methods, and even to change procedures into
% functions, but the resulting code may crash depending on the types used
% and the way the methods are called.
parser_e_dispid_must_be_ord_const=03219_E_מזהה Dispatch חייב להיות קבוע בעל ערך סידורי
parser_e_array_range_out_of_bounds=03220_E_הטווח של המערך גבוהה מידי
% Regardless of the size taken up by its elements, an array cannot have more
% than high(ptrint) elements. Additionally, the range type must be a subrange
% of ptrint.
parser_e_packed_element_no_var_addr=03221_E_לא ניתן למצוא את הכתובת של ביט המערך ארוז, אלמנטים או שדות של המערך
% If you declare an array or record as \var{packed} in Mac Pascal mode (or as \var{packed} in any mode with \var{\{\$bitpacking on\}}), it will
% be packed at the bit level. This means it becomes impossible to take addresses
% of individual array elements or record fields. The only exception to this rule is in case of packed arrays elements
% whose packed size is a multple of 8 bits.
parser_e_packed_dynamic_open_array=03222_E_לא ניתן לארוז מערך דינאמי
% Only regular (and possibly in the future also open) arrays can be packed
parser_e_packed_element_no_loop=03223_E_אלמנטים ושדות של מערכי ביט ארוזים אינם יכולים לשמש כמשתנים ללולאות
% If you declare an array or record as \var{packed} in Mac Pascal mode (or as \var{packed} in any mode with \var{\{\$bitpacking on\}}), it will
% be packed at the bit level. For performance reasons, they cannot be
% used as loop variables.
parser_e_type_and_var_only_in_generics=03224_E_ניתן להשתמש ב VAR ו TYPE רק עם generics
% The usage of VAR and TYPE to declare new types inside an object is allowed only inside
% generics.
parser_e_cant_create_generics_of_this_type=03225_E_הטיפוס אינו יכול להיות generic
% Only Classes, Objects, Interfaces and Records are allowed to be used as generic
parser_w_no_lineinfo_use_switch=03226_W_אין לטעון את הספרייה LINEINFO בצורה ידנית. על מנת להשתמש בספרייה יש להשתמש במתג -gl במקום
% Do not use the LINEINFO unit directly, Use the \var{-gl} switch which automatically adds the
% unit for reading the selected type of debugging information instead.
parser_e_no_funcret_specified=03227_E_לא צויין טיפוס החזרה עבור הפונקציה "$1"
% The first time you declare a function you have to declare it completely,
% including all parameters and the result type.
parser_e_special_onlygenerics=03228_E_השימוש בSpecialization מורשה רק כאשר משתמשים בטיפוסי generic
% Types not being generics can't be specialized
parser_e_no_generics_as_params=03229_E_לא ניתן להשתמש בgenerics בתור פרמטרים כאשר יש שימוש ב spezializing generics
% When specializing a generic, only non-generic types can be used as parameters.
parser_e_type_object_constants=03230_E_השימוש בקבועים של אובייקטים המכילים VMT אינו מורשה
% If an object requires a VMT either because it contains a constructor or virtual methods,
% it's not allowed to create constants of it. In TP and Delphi mode this is allowed
% for compatibility reasons.
parser_e_label_outside_proc=03231_E_שימוש בכתובת של תוויות המוכרזות מחוץ למתחם הנוכחי אינו מורשה
% It isn't allowed to take the addresss of labels outside the
% current procedure.
parser_e_initialized_not_for_external=03233_E_לא ניתן לאתחל ערך ברירת מחדל למשתנים המוגדרים כexternal
% Variables declared as external can not be initialized with a default value.
parser_e_illegal_function_result=03234_E_טיפוס החזרה של הפונקציה אינו חוקי
% Some types like file types can not be used as function result
parser_e_no_common_type=03235_E_אין טיפוס משותף ל "$1" ו "$2"
% To perform an operation beween integers, the compiler converts both operands
% to their common type, which appears to be an invalid type. To determine the
% common type of the operands, the compiler takes the minimum of the minimal values
% of both types, and the maximum of the maximal values of both types. The common
% type is then minimum..maximum.
parser_e_no_generics_as_types=03236_E_Generics ללא specialization אינו יכול להיות בשימוש בתור טיפוס למשתנה
% Generics must be always specialized before being used as variable type
parser_w_register_list_ignored=03237_W_מתעלם מרשימת האוגרים עבור שגרות assemblter טהורות
% When using pure assembler routines, the list with modified registers is ignored.

% \end{description}
#
# Type Checking
#
# 04049 is the last used one
#
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
type_e_mismatch=04000_E_טיפוס נתונים לא חופף
% This can happen in many cases:
% \begin{itemize}
% \item The variable you're assigning to is of a different type than the
% expression in the assignment.
% \item You are calling a function or procedure with parameters that are
% incompatible with the parameters in the function or procedure definition.
% \end{itemize}
type_e_incompatible_types=04001_E_טיפוס נתונים לא תואם: נעשה שימוש ב "$1" מצפה ל"$2"
% There is no conversion possible between the two types
% Another possiblity is that they are declared in different
% declarations:
% \begin{verbatim}
% Var
%    A1 : Array[1..10] Of Integer;
%    A2 : Array[1..10] Of Integer;
%
% Begin
%    A1:=A2; { This statement gives also this error, it
%              is due the strict type checking of pascal }
% End.
% \end{verbatim}
type_e_not_equal_types=04002_E_אי התאמה בין טיפוסי הנתונים "$1" ו "$2"
% The types are not equal
type_e_type_id_expected=04003_E_מצפה לטיפוס נתונים
% The identifier is not a type, or you forgot to supply a type identifier.
type_e_variable_id_expected=04004_E_מצפה למזהה של משתנה
% This happens when you pass a constant to a routine (such as \var{Inc} var or \var{Dec})
% when it expects a variable. You can only pass variables as arguments to these functions.
type_e_integer_expr_expected=04005_E_מצפה למספר שלם, אך התקבל "$1"
% The compiler expects an expression of type integer, but gets a different
% type.
type_e_boolean_expr_expected=04006_E_מצפה לביטוי בוליאני, אך התקבל "$1"
% The expression must be a boolean type, it should be return true or
% false.
type_e_ordinal_expr_expected=04007_E_מצפה לטיפוס סודר
% The expression must be of ordinal type, i.e., maximum a \var{Longint}.
% This happens, for instance, when you specify a second argument
% to \var{Inc} or \var{Dec} that doesn't evaluate to an ordinal value.
type_e_pointer_type_expected=04008_E_מצפה למצביע, אך התקבל "$1"
% The variable or expression isn't of the type \var{pointer}. This
% happens when you pass a variable that isn't a pointer to \var{New}
% or \var{Dispose}.
type_e_class_type_expected=04009_E_מצפה למחלקה, אך התקבל "$1"
% The variable of expression isn't of the type \var{class}. This happens
% typically when
% \begin{enumerate}
% \item The parent class in a class declaration isn't a class.
% \item An exception handler (\var{On}) contains a type identifier that
% isn't a class.
% \end{enumerate}
type_e_cant_eval_constant_expr=04011_E_לא ניתן לנתח את הביטוי הקבוע
% This error can occur when the bounds of an array you declared does
% not evaluate to ordinal constants
type_e_set_element_are_not_comp=04012_E_האלמנטים של הקבוצה אינם תואמים
% You are trying to make an operation on two sets, when the set element types
% are not the same. The base type of a set must be the same when taking the
% union
type_e_set_operation_unknown=04013_E_הפעולה אינה בשימוש עם קבוצות
% several binary operations are not defined for sets
% like div mod ** (also >= <= for now)
type_w_convert_real_2_comp=04014_W_המרה אוטומטית של טיפוס עשרוני ל COMP אשר הוא מספר שלם
% An implicit type conversion from a real type to a \var{comp} is
% encountered. Since \var{comp} is a 64 bit integer type, this may indicate
% an error.
type_h_use_div_for_int=04015_H_השתמש בDIV במקום על מנת לקבל תוצאה של מספר שלם
% When hints are on, then an integer division with the '/' operator will
% procuce this message, because the result will then be of type real
type_e_strict_var_string_violation=04016_E_טיפוס המחרוזת אינם תואם בגלל השימוש במתג $V+
% When compiling in \var{\{\$V+\}} mode, the string you pass as a parameter
% should be of the exact same type as the declared parameter of the procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_לא ניתן להציב ערכים למניות תוך שימוש בsucc או pred
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use the \var{Succ} or \var{Pred} functions on them.
type_e_cant_read_write_type=04018_E_לא ניתן לקרוא או לכתוב משתנים מהסוג הנוכחי
% You are trying to \var{read} or \var{write} a variable from or to a
% file of type text, which doesn't support that. Only integer types,
% reals, pchars and strings can be read from/written to a text file.
% Booleans can only be written to text files.
type_e_no_readln_writeln_for_typed_file=04019_E_לא ניתן להשתמש בreadln וwriteln על טיפוס נתונים מסוג file
% \var{readln} and \var{writeln} are only allowed for text files.
type_e_no_read_write_for_untyped_file=04020_E_לא ניתן לקרוא או לכתוב טיפוס לא מוגר של קבצים
% \var{read} and \var{write} are only allowed for text or typed files.
type_e_typeconflict_in_set=04021_E_התנגשות של טיפוס נתונים בתוך איברים של קבוצה
% There is at least one set element which is of the wrong type, i.e. not of
% the set type.
type_w_maybe_wrong_hi_lo=04022_W_lo/hi(dword/qword) מחזיר את הערך העליון/תחתון של word/dword
% \fpc supports an overloaded version of \var{lo/hi} for \var{longint/dword/int64/qword}
% which returns the lower/upper word/dword of the argument. TP always uses
% a 16 bit \var{lo/hi} which returns always bits 0..7 for \var{lo} and the
% bits 8..15 for \var{hi}. If you want the TP behavior you have
% to type cast the argument to \var{word/integer}
type_e_integer_or_real_expr_expected=04023_E_מצפה לביטוי של מספר שלם או מספר ממשי
% The first argument to \var{str} must a real or integer type.
type_e_wrong_type_in_array_constructor=04024_E_שימוש שגוי בטיפוס "$1" בתוך יוצר המערך
% You are trying to use a type in an array constructor which is not
% allowed.
type_e_wrong_parameter_type=04025_E_טיפוס לא מתאים לארגומנט מספר $1: התקבל "$2", מצפה ל"$3"
% You are trying to pass an invalid type for the specified parameter.
type_e_no_method_and_procedure_not_compatible=04026_E_המשתנה של המתודה והשגרה אינם תואמים
% You can't assign a method to a procedure variable or a procedure to a
% method pointer.
type_e_wrong_math_argument=04027_E_ביטוי קבוע לא חוקי הוזן לפונקציה מתמטית פנימית
% The constant argument passed to a ln or sqrt function is out of
% the definition range of these functions.
type_e_no_addr_of_constant=04028_E_לא ניתן לקבל את הכתובת של הקבוע
% It is not possible to get the address of a constant expression, because they
% aren't stored in memory. You can try making it a typed constant. This error
% can also be displayed if you try to pass a property to a var parameter.
type_e_argument_cant_be_assigned=04029_E_לא ניתן להשים את הארגומנט
% Only expressions which can be on the left side of an
% assignment can be passed as call by reference argument
% Remark: Properties can be only
% used on the left side of an assignment, but they cannot be used as arguments
type_e_cannot_local_proc_to_procvar=04030_E_לא ניתן לשייך פונקציה/שיגרה מקומית למשתנה מסוג שיגרה
% It's not allowed to assign a local procedure/function to a
% procedure variable, because the calling of local procedure/function is
% different. You can only assign local procedure/function to a void pointer.
type_e_no_assign_to_addr=04031_E_לא ניתן לשייך ערך לכתובת
% It is not allowed to assign a value to an address of a variable,constant,
% procedure or function. You can try compiling with -So if the identifier
% is a procedure variable.
type_e_no_assign_to_const=04032_E_לא ניתן לשייך ערך למשתנה קבוע
% It's not allowed to assign a value to a variable which is declared
% as a const. This is normally a parameter declared as const, to allow
% changing the value make the parameter as a value parameter or a var.
type_e_array_required=04033_E_נדרש משתנה מסוג מערך
% If you are accessing a variable using an index '[<x>]' then
% the type must be an array. In FPC mode also a pointer is allowed.
type_e_interface_type_expected=04034_E_מצפה לטיפוס מסוג ממשק, אבל התקבל "$1"
% The compiler expected to encounter an interface type name, but got something else.
% The following code would provoke this error:
% \begin{verbatim}
% Type
%   TMyStream = Class(TStream,Integer)
% \end{verbatim}
type_w_mixed_signed_unsigned=04035_W_עירבוב ביטויי סימנים וlongwords מספקים תוצאה של 64bit
% If you divide (or calculate the modulus of) a signed expression by a longword (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetic
% expression (+, -, *, div, mod) in which both signed numbers and longwords appear,
% then everything has to be evaluated in 64bit which is slower than normal
% 32bit arithmetic. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_w_mixed_signed_unsigned2=04036_W_ערבוב ביטויי סימנים ומספרים שלמים גבוהים עלול לגרום לשגיאת טווח מספרים
% If you use a binary operator (and, or, xor) and one of
% the operands is a longword while the other one is a signed expression, then,
% if range checking is turned on, you may get a range check error because in
% such a case both operands are converted to longword before the operation is
% carried out. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_e_typecast_wrong_size_for_assignment=04037_E_ישנו הבדל בגודל בהצבה של typecast ($1-> $2) 
% Type casting to a type with a different size is not allowed when the variable is
% used for assigning.
type_e_array_index_enums_with_assign_not_possible=04038_E_מניות עם תוכן של הצבה אינם יכולים לשמש בתור אינדקס למערך
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use it as index of an array.
type_e_classes_not_related=04039_E_אין קירבה בין טיפוסי מחלקות או אובייקטים של "$1" ו "$2"
% There is a typecast from one class or object to another while the class/object
% are not related. This will probably lead to errors
type_w_classes_not_related=04040_W_אין קירבה בין טיפוסי המחלקות של "$1" ו "$2"
% There is a typecast from one class or object to another while the class/object
% are not related. This will probably lead to errors
type_e_class_or_interface_type_expected=04041_E_מצפה לטיפוסי מחלקות או ממשקים, אבל "$1" התקבל
% The compiler expected a class or interface name, but got another type or identifier.
type_e_type_is_not_completly_defined=04042_E_הטיפוס "$1" לא הוגדר במלואו
% This error occurs when a type is not complete: i.e. a pointer type which points to
% an undefined type.
type_w_string_too_long=04043_W_התוכן של המחרוזת מכיל יותר תווים ממה שניתן להכיל באורך של מחרוזת קצרה
% The size of the constant string, which is assigned to a shortstring,
% is longer than the maximum size of the shortstring
type_w_signed_unsigned_always_false=04044_W_ההשוואה תמיד תחזיר ערך של false בגלל טווח הערכים
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% false. Exlicitly typecast the constant to the correct range to avoid this problem.
type_w_signed_unsigned_always_true=04045_W_ההשוואה תמיד תחזיר ערך של true  בגלל טווח הערכים
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% true. Exlicitly typecast the constant to the correct range to avoid this problem.
type_w_instance_with_abstract=04046_W_מאתחל את המחלקה "$1" עם מתודות לא ממומשות
% An instance of a class is created which contains non-implemented abstract
% methods. This will probably lead to a runtime error 211 in the code if that
% routine is ever called. All abstract methods should be overridden.
type_h_in_range_check=04047_H_הערך השמאלי של האופרנד IN צריך להיות בגודל של בית
% The left operand of the \var{in} operator is not an ordinal or enumeration which fits
% within 8-bits, this may lead to range check errors. The \var{in} operator
% currently only supports a left operand which fits within a byte. In the case of
% enumerations, the size of an element of an enumeration can be controlled with
% the \var{\{\$PACKENUM\}} or \var{\{\$Zn\}} switches.
type_w_smaller_possible_range_check=04048_W_גודל הטיפוס אינו מתאים, ישנה אפשרות לאיבוד מידע או שגיאה בבדיקת הטווח
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_h_smaller_possible_range_check=04049_H_גודל הטיפוס אינו מתאים, ישנה אפשרות לאיבוד מידע או שגיאה בבדיקת הטווח
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_e_cant_take_address_of_abstract_method=04050_E_לא ניתן לספק את כתובת המתודה המוגדרת כ abstract
% An abstract method has no body, so the address of an abstract method can't be taken.
type_e_assignment_not_allowed=04051_E_לא ניתן לשייך את הערך לפרמטרים רשמיים ומערכים פתוחים
% You are trying to assign a value to a formal (untyped var, const or out)
% parameter, or to an open array.
type_e_constant_expr_expected=04052_E_מצפה לביטוי קבוע
% The compiler expects an constant expression, but gets a variable expression.
type_e_operator_not_supported_for_types=04053_E_הפעולה "$1" אינה נתמכת לטיפוסים "$1" ו "$3"
% The operation is not allowed for the supplied types
type_e_illegal_type_conversion=04054_E_המרה לא חוקית של הטיפוס של "$1" ל "$2"
% When doing a type-cast, you must take care that the sizes of the variable and
% the destination type are the same.
type_h_pointer_to_longint_conv_not_portable=04055_H_ההמרה בין מספר למצביע אינה אפשרית בכל המערכות
% If you typecast a pointer to a longint (or vice-versa), this code will not compile
% on a machine using 64-bit for pointer storage.
type_w_pointer_to_longint_conv_not_portable=04056_W_ההמרה בין מספר למצביע אינה אפשרית בכל המערכות
% If you typecast a pointer to a ordinal type of a different size (or vice-versa), this can
% cause problems. This is a warning to help finding the 32bit specific code where cardinal/longint is used
% to typecast pointers to ordinals. A solution is to use the ptrint/ptruint types instead.
type_e_cant_choose_overload_function=04057_E_לא ניתן להחליט באיזו פונקצית עמוסת יתר להשתמש
% You're calling overloaded functions with a parameter that doesn't correspond
% to any of the declared function parameter lists. e.g. when you have declared
% a function with parameters \var{word} and \var{longint}, and then you call
% it with a parameter which is of type \var{integer}.
type_e_illegal_count_var=04058_E_משתנה ספירה לא חוקי
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
type_w_double_c_varargs=04059_W_ממיר את הטיפוס real לטיפוס double למשתנה C. הוסף typecast ספציפי למנוע את המצב.
% In C, constant real values are double by default. For this reason, if you
% pass a constant real value to a variable argument part of a C function, FPC
% by default converts this constant to double as well. If you want to prevent
% this from happening, add an explicit typecast around the constant.
type_e_class_or_cominterface_type_expected=04060_E_מצפה לטיפוס מחלקה או ממשק COM, אבל התקבל "$1"
% Some operators like the AS operator are only appliable to classes or COM interfaces.
type_e_no_const_packed_array=04061_E_אין תמיכה במערך קבוע דחוס
% You cannot declare a (bit)packed array as a typed constant.
type_e_got_expected_packed_array=04062_E_חוסר תאימות לטיפוס הנתונים של ארגומנט $1. התקבל: "$2" מצא ל "(bit)packed array" 
% The compiler expects a (bit)packed array as the specified parameter
type_e_got_expected_unpacked_array=04063_E_חוסר תאימות טיפוס הנתונים של ארגומנט $1. התקבל "$2" מצפה ל "(not packed) Array"
% The compiler expects a regular (i.e., not packed) array as the specified parameter
type_e_no_packed_inittable=04064_E_אלמטים של מערך דחוס אינם יכולים להיות מטיפוס נתונים אשר דורש אתחול
% Support for packed arrays of types that need initialization (such as ansistrings, or records which contain ansistrings) is not yet implemented.
type_e_no_const_packed_record=04065_E_אין תמיכה לרשומות ואבייקטים קבועים ודחוסים
% You cannot declare a (bit)packed array as a typed constant at this time.
type_w_untyped_arithmetic_unportable=04066_W_חישוב "$1" על טיפוס לא מוגדר של מצביע אינו נתמך במצב {$T+}, אך ניתן להשתמש בtypecast
% Addition/subtraction from untyped pointer may work differently in \var{\{\$T+\}}, use typecast to typed pointer
type_e_cant_take_address_of_local_subroutine=04076_E_לא ניתן לקחת את כתובת השיגרה המסומנת כמקומית
% The address of a subroutine marked as local can't be taken.
type_e_cant_export_local=04077_E_לא ניתן לייצא שיגרה המסומנת כמקומית בתוך יחידה
% A subroutine marked as local can't be export from a unit.
type_e_not_automatable=04078_E_הטיפוס אינו אוטומט: "$1"
% Only byte, integer, longint, smallint, currency, single, double, ansistring,
% widestring, tdatetime, variant, olevariant, wordbool and all interfaces are automatable.
type_h_convert_add_operands_to_prevent_overflow=04079_H_המרת האופרנד ל"$1" לפני פעולת החיבור, יכול למנוע שגיאות גלישה.
% Adding two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the addition.
type_h_convert_sub_operands_to_prevent_overflow=04080_H_המרת האופרנד ל"$1" לפני פעולת החיבור יכולה למנוע שגיאות גלישה.
% Subtracting two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the subtraction.
type_h_convert_mul_operands_to_prevent_overflow=04081_H_המרת האופרנד "$1" לפני פעולת הכפל יכולה למנוע שגיאות גלישה.
% Multiplying two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the multiplication.
type_w_pointer_to_signed=04082_W_המרת מצביעים למספרים שלמים עם סימן עלולה לגרום לשגיאות בתוצאות השוואה ובטווחים. יש להשתמש במקום זאת בטיפוס ללא סימן.
% The virtual address space on 32-bit machines runs from \$00000000 to \$ffffffff. Many operating systems allow you to
% allocate memory above \$80000000, for example both Windows and Linux allow pointers in the range \$0000000 to \$bfffffff.
% If you convert pointers to signed types, this can cause overflow and range check errors, but also \$80000000 < \$7fffffff.
% This can cause random errors in code like "if p>q".
% \end{description}
#
# Symtable
#
# 05060 is the last used one
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_המזהה לא נמצא "$1"
% The compiler doesn't know this symbol. Usually happens when you misspel
% the name of a variable or procedure, or when you forgot to declare a
% variable.
sym_f_internal_error_in_symtablestack=05001_F_שגיאה פנימית בSymTableStack()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_e_duplicate_id=05002_E_מזהה כפול "$1"
% The identifier was already declared in the current scope.
sym_h_duplicate_id_where=05003_H_המזהה כבר מוגדר ב$1 בשורה $2
% The identifier was already declared in a previous scope.
sym_e_unknown_id=05004_E_מזהה לא מוכר "$1"
% The identifier encountered has not been declared, or is used outside the
% scope where it is defined.
sym_e_forward_not_resolved=05005_E_הגדרת Forward לא ממומשת
% This can happen in two cases:
% \begin{itemize}
% \item This happens when you declare a function (in the \var{interface} part, or
% with a \var{forward} directive, but do not implement it.
% \item You reference a type which isn't declared in the current \var{type}
% block.
% \end{itemize}
sym_e_error_in_type_def=05007_E_שגיאה בהגדרת טיפוס
% There is an error in your definition of a new array type:
% \item One of the range delimiters in an array declaration is erroneous.
% For example, \var{Array [1..1.25]} will trigger this error.
sym_e_forward_type_not_resolved=05009_E_הגדרת הטיפוס לא הושלה "$1"
% A symbol was forward defined, but no declaration was encountered.
sym_e_only_static_in_static=05010_E_רק משתנים סטטיים יכולים להיות בשימוש עם מתודות סטטיות או מחוץ למטודות
% A static method of an object can only access static variables.
sym_e_type_must_be_rec_or_object_or_class=05012_E_Record or object or class type expected
% The variable or expression isn't of the type \var{record} or \var{object} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_אדגם של מחלקות או אובייקטים עם מתודות abstract אינם מורשים
% You are trying to generate an instance of a class which has an abstract
% method that wasn't overridden.
sym_w_label_not_defined=05014_W_התווית "$1" אינה מוגדרת
% A label was declared, but not defined.
sym_e_label_used_and_not_defined=05015_E_התווית "$1" בשימוש, אבל לא מגדרת
% A label was declared and used, but not defined.
sym_e_ill_label_decl=05016_E_הגדרת תווית לא חוקית
% This error should never happen; it occurs if a label is defined outside a
% procedure or function.
sym_e_goto_and_label_not_supported=05017_E_GOTO ו LABEL אינם נתמכים (השתמש במתג -Sg)
% You must compile a program which has \var{label}s and \var{goto} statements
% with the \var{-Sg} switch. By default, \var{label} and \var{goto} aren't
% supported.
sym_e_label_not_found=05018_E_התווית לא נמצאה
% A \var{goto label} was encountered, but the label isn't declared.
sym_e_id_is_no_label_id=05019_E_המזהה אינו תווית
% The identifier specified after the \var{goto} isn't of type label.
sym_e_label_already_defined=05020_E_התווית כבר הוגדרה
% You are defining a label twice. You can define a label only once.
sym_e_ill_type_decl_set=05021_E_הגדרת טיפוס סידרה לא חוקית
% The declaration of a set contains an invalid type definition.
sym_e_class_forward_not_resolved=05022_E_הגדרה מקדימה של מחלקה "$1" לא ממומשת
% You declared a class, but you did not implement it.
sym_n_unit_not_used=05023_H_היחידה "$1" אינה בשימוש ב $2
% The unit referenced in the \var{uses} clause is not used.
sym_h_para_identifier_not_used=05024_H_הפרמטר "$1" אינו בשימוש
% The identifier was declared (locally or globally) but
% was not used (locally or globally).
sym_n_local_identifier_not_used=05025_N_המשתנה המקומי "$1" אינו בשימוש
% You have declared, but not used a variable in a procedure or function
% implementation.
sym_h_para_identifier_only_set=05026_H_הערך של הפרמטר "$1" הוזן אך הוא לא בשימוש
% The identifier was declared (locally or globally)
% set but not used (locally or globally).
sym_n_local_identifier_only_set=05027_N_הערך של הפרמטר "$1" הוזן אך הוא לא בשימוש
% The variable in a procedure or function
% implementation is declared, set but never used.
sym_h_local_symbol_not_used=05028_H_$1 "$2" מקומי לא בשימוש
% A local symbol is never used.
sym_n_private_identifier_not_used=05029_N_שדה פרטי "$1.$2" לא בשימוש
% The indicated private field is defined, but is never used in the code.
sym_n_private_identifier_only_set=05030_N_השדה הפרטי "$1.$2" בעל ערך, אך לא בשימוש
% The indicated private field is declared, assigned but never read.
sym_n_private_method_not_used=05031_N_המתודה הפרטית "$1.$2" אינה בשימוש
% The indicated private method is declared but is never used in the code.
sym_e_set_expected=05032_E_מצפה לטיפוס סדרה
% The variable or expression is not of type \var{set}. This happens in an
% \var{in} statement.
sym_w_function_result_not_set=05033_W_הפונקציה כנראה לא מחזירה ערך
% You can get this warning if the compiler thinks that a function return
% value is not set. This will not be displayed for assembler procedures,
% or procedures that contain assembler blocks.
sym_w_wrong_C_pack=05034_W_הטיפוס "$1" אינו מייושר נכון לרשומה של שפת C
% Arrays with sizes not multiples of 4 will be wrongly aligned
% for C structures.
sym_e_illegal_field=05035_E_שדה של רשומה לא חוקי "$1"
% The field doesn't exist in the record/object definition.
sym_w_uninitialized_local_variable=05036_W_המשתנה המקומי "$1" אינו נראה מאותחל
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_w_uninitialized_variable=05037_W_המשתנה "$1" אינו נראה מאותחל
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_e_id_no_member=05038_E_המזהה בשימוש אינו חבר ב"$1"
% This error is generated when an identifier of a record,
% field, or method is accessed while it is not defined.
sym_h_param_list=05039_H_נמצאה ההכרזה: $1
% You get this when you use the \var{-vh} switch. In case an overloaded
% procedure is not found, then all candidate overloaded procedures are
% listed, with their parameter lists.
sym_e_segment_too_large=05040_E_גודל המידע של האלמנט גדול מידי
% You get this when you declare a data element whose size exceeds the
% prescribed limit (2 Gb on 80386+/68020+ processors)
sym_e_no_matching_implementation_found=05042_E_לא נמצא ביצוע למתודה "$1" של הממשק
% There was no matching method found which could implement the interface
% method. Check argument types and result type of the methods.
sym_w_deprecated_symbol=05043_W_הסימול "$1" מיושן
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Usage of this symbol
% should be avoided as much as possible.
sym_w_non_portable_symbol=05044_W_הסימול "$1" אינו נייד
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{platform} is used. This symbol's value, usage
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_non_implemented_symbol=05055_W_הסימול "$1" אינו מבוצע
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{unimplemented} is used. This symbol is defined,
% but is not yet implemented on this specific platform.
sym_e_cant_create_unique_type=05056_E_לא ניתן ליצור טיפוס ייחודי מהטיפוס הנוכחי
% Only simple types like ordinal, float and string types are supported when
% redefining a type with \var{type newtype = type oldtype;}.
sym_h_uninitialized_local_variable=05057_H_המשתנה המקומי "$1" אינו נראה מאותחל
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_h_uninitialized_variable=05058_H_המשתנה "$1" אינו נראה מאותחל
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_w_function_result_uninitialized=05059_W_הערך המוחזר מהפונקציה אינו נראה כמאותחל
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. appears in the right-hand-side of an expression)
% before it is initialized (i.e. appeared in the left-hand side of an
% assigment)
sym_h_function_result_uninitialized=05060_H_הערך המוחזר מהפונקציה אינו נראה כמאותחל
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. appears in the right-hand-side of an expression)
% before it is initialized (i.e. appeared in the left-hand side of an
% assigment)
sym_w_identifier_only_read=05061_W_המשתנה "$1" נקרא, אך מעולם לא קיבל תוכן
% You have read the value of a variable, but nowhere assigned a value to
% it.
sym_h_abstract_method_list=05062_H_נמצאה מתודה מופשטת: $1
% When getting a warning about constructing a class/object with abstract methods
% you get this hint to find the affected method.
% \end{description}
#
# Codegenerator
#
# 06040 is the last used one
#
% \section{Code generator messages}
% This section lists all messages that can be displayed if the code
% generator encounters an error condition.
% \begin{description}
cg_e_parasize_too_big=06009_E_הגודר של רשימת פרמטרים גדולה מ 65535 בתים
% The I386 processor limits the parameter list to 65535 bytes (the \var{RET}
% instruction causes this)
cg_e_file_must_call_by_reference=06012_E_טיפוס של קובץ חייב להיות משתנה מוגדר
% You cannot specify files as value parameters, i.e. they must always be
% declared \var{var} parameters.
cg_e_cant_use_far_pointer_there=06013_E_השימוש במצביע far אינו מורשה במיקום הנוכחי
% Free Pascal doesn't support far pointers, so you cannot take the address of
% an expression which has a far reference as a result. The \var{mem} construct
% has a far reference as a result, so the following code will produce this
% error:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_dont_call_exported_direct=06015_E_לא ניתן לקרוא לפונקציות עם הגדרת EXPORT
% No longer in use.
cg_w_member_cd_call_from_method=06016_W_כנראה קריאה לא חוקית של יוצר או הורס
% The compiler detected that a constructor or destructor is called within a
% a method. This will probably lead to problems, since constructors / destructors
% require parameters on entry.
cg_n_inefficient_code=06017_N_חסר קוד
% Your statement seems dubious to the compiler.
cg_w_unreachable_code=06018_W_לא ניתן להריץ את קטע הקוד
% You specified a construct which will never be executed. Example:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_cant_call_abstract_method=06020_E_לא ניתן לקרוא למתודות Abstract בצורה ישירה
% You cannot call an abstract method directy, instead you must call a
% overriding child method, because an abstract method isn't implemented.
cg_d_register_weight=06027_DL_Register $1 משקל $2 $3
% Debugging message. Shown when the compiler considers a variable for
% keeping in the registers.
cg_d_stackframe_omited=06029_DL_משמיט את מסגרת המחסנית
% Some procedure/functions do not need a complete stack-frame, so it is omitted.
% This message will be displayed when the {-vd} switch is used.
cg_e_unable_inline_object_methods=06031_E_מתודות של אובייקט או מחלקה אינם יכולים להיות inline
% You cannot have inlined object methods.
cg_e_unable_inline_procvar=06032_E_קריאות Procvar אינם יכולים להיות inline
% A procedure with a procedural variable call cannot be inlined.
cg_e_no_code_for_inline_stored=06033_E_אין קוד לפרוצדורות inline
% The compiler couldn't store code for the inline procedure.
cg_e_can_access_element_zero=06035_E_אלמנט האפס של ansi/wide- או longstring לא נגיש, השתמש ב (set)length במקום
% You should use \var{setlength} to set the length of an ansi/wide/longstring
% and \var{length} to get the length of such a string types
cg_e_cannot_call_cons_dest_inside_with=06037_E_לא ניתן לקרוא ליוצרים או הורסים בתוך חלק של 'with'
% Inside a \var{with} clause you cannot call a constructor or destructor for the
% object you have in the \var{with} clause.
cg_e_cannot_call_message_direct=06038_E_לא ניתן לקרוא למתודות של מטפל הודאות בצורה ישירה
% A message method handler method cannot be called directly if it contains an
% explicit self argument
cg_e_goto_inout_of_exception_block=06039_E_קפיצה אל תוך או מחוץ לבלוק של exception
% It is not allowed to jump in or outside of an exception block like \var{try..finally..end;}:
% \begin{verbatim}
% label 1;
%
% ...
%
% try
%    if not(final) then
%      goto 1;   // this line will cause an error
% finally
%   ...
% end;
% 1:
% ...
% \end{verbatim}
cg_e_control_flow_outside_finally=06040_E_השימוש בביטויים השולטים בזרימת הקוד אינם מורשים בחלק ה finally
% It isn't allowed to use the control flow statements \var{break},
% \var{continue} and \var{exit}
% inside a finally statement. The following example shows the problem:
% \begin{verbatim}
% ...
%   try
%      p;
%   finally
%      ...
%      exit;  // This exit ISN'T allowed
%   end;
% ...
%
% \end{verbatim}
% If the procedure \var{p} raises an exception the finally block is
% executed. If the execution reaches the exit, it's unclear what to do:
% exiting the procedure or searching for another exception handler
cg_w_parasize_too_big=06041_W_גודל הפרמטים גולש את ההגבלות של חלק מהמעבדים
% This indicates that you are declaring more than 64K of parameters, which
% might not be supported on other processor targets.
cg_w_localsize_too_big=06042_W_הגודל של המשתנה המקומי גולש מהגבלות של חלק מהמעבדים
% This indicates that you are declaring more than 32K of local variables, which
% might not be supported on other processor targets.
cg_e_localsize_too_big=06043_E_גודל משתנים מקומיים גולש מהגבלות הנתמכות
% This indicates that you are declaring more than 32K of local variables, which
% is not supported by this processor.
cg_e_break_not_allowed=06044_E_BREAK אינו מורשה
% You're trying to use \var{break} outside a loop construction.
cg_e_continue_not_allowed=06045_E_CONTINUE אינו מורשה
% You're trying to use \var{continue} outside a loop construction.
cg_f_unknown_compilerproc=06046_F_compilerproc "$1" לא ידוע. בדוק אם השתמש בספריית זמן הריצה הנכונה.
% The compiler expects that the runtime library contains certain subroutines. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the used compiler. If you changed the runtime library this error means
% that you removed a subroutine which the compiler needs for internal use.
cg_f_unknown_system_type=06047_F_לא ניתן למצוא טיפוס מערכת "$1". בדוק האם אתה משתמש בספריית זמן הריצה עדכנית.
% The compiler expects that the runtime library contains certain type definitions. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the used compiler. If you changed the runtime library this error means
% that you removed a type which the compiler needs for internal use.
cg_h_inherited_ignored=06048_H_מתעלם מהשימוש בinherited במתודה מסוג abstract
% This messages appears only in Delphi mode when you call an abstract method
% of a parent class via \var{inherited;}. The call is then ignored.
cg_e_goto_label_not_found=06049_E_תווית ה Goto "$1" לא הוגדרה או לא עברה אופטימיזציה
% The label used in the goto definition is not defined or optimized away by the
% unreachable code elemination.
% \end{description}
# EndOfTeX

#
# Assembler reader
#
# 07105 is the last used one
#
asmr_d_start_reading=07000_DL_מתחיל בסגנון מפרש אסמבלר $1
% This informs you that an assembler block is being parsed
asmr_d_finish_reading=07001_DL_הסתיים ניתוח סגנון אסמבלר $1
% This informs you that an assembler block has finished.
asmr_e_none_label_contain_at=07002_E_תבנית ללא תווית מכילה את התו @
% A identifier which isn't a label can't contain a @.
asmr_e_building_record_offset=07004_E_שגיאה בבניית היסט הרשומה
% There has an error occured while building the offset of a record/object
% structure, this can happend when there is no field specified at all or
% an unknown field identifier is used.
asmr_e_offset_without_identifier=07005_E_נעשה שימוש בOFFSET ללא מזהה
% You can only use OFFSET with an identifier. Other syntaxes aren't
% supported
asmr_e_type_without_identifier=07006_E_נעשה שימוש בTYPE ללא מזהה
% You can only use TYPE with an identifier. Other syntaxes aren't
% supported
asmr_e_no_local_or_para_allowed=07007_E_לא ניתן להשתמש במשתמה מקומי או פרמטר במיקום הנוכחי
% You can't use a local variable or parameter here, mostly because the
% addressing of locals and parameters is done using the frame pointer register so the
% address can't be obtained directly.
asmr_e_need_offset=07008_E_יש צורך להשתמש ב OFFSET
% You need to use OFFSET <id> here to get the address of the identifier.
asmr_e_need_dollar=07009_E_יש צורך להשתמש ב$
% You need to use $<id> here to get the address of the identifier.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_לא ניתן להשתמש ב relocatable symbols מרובים
% You can't have more than one relocatable symbol (variable/typed constant)
% in one argument.
asmr_e_only_add_relocatable_symbol=07011_E_ניתן להוסיף רק relocatable symbol
% Relocatable symbols (variable/typed constant) can't be used with other
% operators. Only addition is allowed.
asmr_e_invalid_constant_expression=07012_E_ביטוי קבוע לא חוקי
% There is an error in the constant expression.
asmr_e_relocatable_symbol_not_allowed=07013_E_Relocatable symbol אינו מורשה
% You can't use a relocatable symbol (variable/typed constant) here.
asmr_e_invalid_reference_syntax=07014_E_תחביר הפניות לא חוקי
% There is an error in the reference.
asmr_e_local_para_unreachable=07015_E_לא ניתן לגשת ל $1 מהקוד
% You can not read directly the value of a local variable or parameter
% of a higher level procedure in assembler code (except for
% local assembler code without parameter nor locals).
asmr_e_local_label_not_allowed_as_ref=07016_E_סימולים/תוויות מקומיות אינם מורשים כהפניות
% You can't use local symbols/labels as references
asmr_e_wrong_base_index=07017_E_שימוש שגוי בבסיס ואינדקס של אוגר
% There is an error with the base and index register, they are
% probably incorrect
asmr_w_possible_object_field_bug=07018_W_יתכן כי יש שגיאה בטיפול השדות של אובייקט
% Fields of objects or classes can be reached directly in normal or objfpc
% modes but TP and Delphi modes treat the field name as a simple offset.
asmr_e_wrong_scale_factor=07019_E_צויין טווח סולם שגוי
% The scale factor given is wrong, only 1,2,4 and 8 are allowed
asmr_e_multiple_index=07020_E_שימוש מרובה באינקס האוגר
% You are trying to use more than one index register
asmr_e_invalid_operand_type=07021_E_טיפוס אופרנד שגוי
% The operand type doesn't match with the opcode used
asmr_e_invalid_string_as_opcode_operand=07022_E_אשימוש במחרוזת כאופרנד opcode שגוי: $1 
% The string specified as operand is not correct with this opcode
asmr_w_CODE_and_DATA_not_supported=07023_W_השימוש ב@CODE ו@DATA לא נתמך
% @CODE and @DATA are unsupported and are ignored.
asmr_e_null_label_ref_not_allowed=07024_E_התייחסות לתווית ריקה אינה מורשת
asmr_e_expr_zero_divide=07025_E_חילוק באפס בהערכת asm
% There is a division by zero in a constant expression
asmr_e_expr_illegal=07026_E_ביטוי לא חוקי
% There is an illegal expression in a constant expression
asmr_e_escape_seq_ignored=07027_E_מתעלם מרצף קידוד: $1
% There is a C-styled string, but the escape sequence in the string
% is unknown, and is therefore ignored
asmr_e_invalid_symbol_ref=07028_E_התייחסות סימול לא תקינה
asmr_w_fwait_emu_prob=07029_W_FWAIT מסוגל לגרום לבעיות חיקוי בemu387
asmr_w_fadd_to_faddp=07030_W_$1 ללא תרגום אופרנד ל$1P
asmr_w_enter_not_supported_by_linux=07031_W_הוראת ENTER אינה נתמכת ע"י ליבת Linux
% ENTER instruction can generate a stack page fault that is not
% caught correctly by the i386 Linux page handler.
asmr_w_calling_overload_func=07032_W_מבצע קריאה לפונקציה משוכתבת בassembler
% There is a call to an overloaded method in the assembler block,
% this might be the sign there is a problem
asmr_e_unsupported_symbol_type=07033_E_טיפוס הסימול של האופרנד אינו נתמך
asmr_e_constant_out_of_bounds=07034_E_תוכן הקבוע מחוץ לגבולות
asmr_e_error_converting_decimal=07035_E_שגיאה בזמן המרה דצימלית $1
% A constant decimal value does not have the correct syntax
asmr_e_error_converting_octal=07036_E_שגיאה בזמן המרה אוקטלית $1
% A constant octal value does not have the correct syntax
asmr_e_error_converting_binary=07037_E_שגיאה בזמן המרה בינרית $1
% A constant binary value does not have the correct syntax
asmr_e_error_converting_hexadecimal=07038_E_שגיאה בזמן המרה הקסה-דצימלית $1
% A constant hexadecimal value does not have the correct syntax
asmr_h_direct_global_to_mangled=07039_H_$1 תורגם ל $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 משוייך לפונקציה משוכתבת
asmr_e_cannot_use_SELF_outside_a_method=07041_E_לא ניתן להשתמש ב SELF מחוץ למתודה
% There is a reference to the \var{self} symbol while it is not
% allowed. \var{self} can only be referenced inside methods
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_לא ניתן להשתמש ב OLDEBP מחוץ לפרוצדורה מקוננת
% There is a reference to the \var{oldebp} symbol while it is not
% allowed. \var{oldebp} can only be referenced inside nested routines
asmr_e_void_function=07043_W_פרוצדורות אינם יכולים להחזיר ערך בתוך קוד assembly
% Trying to return a value while in a procedure. A procedure
% does not have any return value
asmr_e_SEG_not_supported=07044_E_SEG אינו נתמך
asmr_e_size_suffix_and_dest_dont_match=07045_E_גודל סופי ויעד, או מקור הגודל אינם תואמים
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_w_size_suffix_and_dest_dont_match=07046_W_גודל סופי ויעד, או מקור הגודל אינם תואמים
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_e_syntax_error=07047_E_שגיאה בתחביר Assembler
% There is an assembler syntax error
asmr_e_invalid_opcode_and_operand=07048_E_שילוב לא חוקי בין opcode ואופרנד
% The opcode cannot be used with this type of operand
asmr_e_syn_operand=07049_E_שגיאה בתחביר אופרנד בAssembler
asmr_e_syn_constant=07050_E_שגיאה בתחביר קבוע בAssembler
asmr_e_invalid_string_expression=07051_E_ביטוי מחרוזת לא תקין
asmr_w_const32bit_for_address=07052_W_הקבוע בעל הסימול $1 לכתובת, הוא לא מצביע
% A constant expression represents an address which does not fit
% into a pointer. The address is probably incorrect
asmr_e_unknown_opcode=07053_E_opcode $1 אינו מוכר
% This opcode is not known
asmr_e_invalid_or_missing_opcode=07054_E_opcode חסר או לא תקין
asmr_e_invalid_prefix_and_opcode=07055_E_שילוב של תחילית וopcode לא תקין : $1
asmr_e_invalid_override_and_opcode=07056_E_השילוב של שכתוב וopcode לא תקין: $1
asmr_e_too_many_operands=07057_E_Too many operands on line
% There are too many operands for this opcode. Check your
% assembler syntax
asmr_w_near_ignored=07058_W_מתעלם מNEAR
asmr_w_far_ignored=07059_W_מתעלם מ FAR
asmr_e_dup_local_sym=07060_E_שיכפול של הסימול המקומי $1
asmr_e_unknown_local_sym=07061_E_הסימול המקומי $1 אינו מוגדר
asmr_e_unknown_label_identifier=07062_E_מזהה תווית $1 לא ידוע
asmr_e_invalid_register=07063_E_שם אוגר שגוי
% There is an unknown register name used as operand.
asmr_e_invalid_fpu_register=07064_E_שם אוגר לנקודה צפה לא תקין
% There is an unknown register name used as operand.
asmr_w_modulo_not_supported=07066_W_מודולו לא נתמך
asmr_e_invalid_float_const=07067_E_קבוע של נקודה צפה לא תקין $1
% The floating point constant declared in an assembler block is
% invalid.
asmr_e_invalid_float_expr=07068_E_ביטוי לא תקין של נקודה צפה
% The floating point expression declared in an assembler block is
% invalid.
asmr_e_wrong_sym_type=07069_E_סוג סימול שגוי
asmr_e_cannot_index_relative_var=07070_E_לא ניתן לשמור ציון של פרמטר או משתנה מקומי עם אוגר
% Trying to index using a base register a symbol which is already relative
% to a register. This is not possible, and will probably lead to crashes.
asmr_e_invalid_seg_override=07071_E_ביטוי המיקטע המשוכתב אינו תקין
asmr_w_id_supposed_external=07072_W_המזהה $1 אינו נמצא, מניח שהמזהה חיצוני
% There is a reference to an undefined symbol. This will not result
% in an error, since the symbol might be external, but may cause
% problems at link time if the symbol is not defined anywhere.
asmr_e_string_not_allowed_as_const=07073_E_המחרוזות אינן מורשות להיות קבועות
% Character strings are not allowed as constants.
asmr_e_no_var_type_specified=07074_לא צויין טיפוס המשתנה
% The syntax expects a type idenfitifer after the dot, but
% none was found.
asmr_w_assembler_code_not_returned_to_text=07075_E_קוד הassembler לא חזר לאיזור הטקסט
% There was a directive in the assembler block to change sections,
% but there is a missing return to the text section at the end
% of the assembler block. This might cause errors during link time.
asmr_e_not_directive_or_local_symbol=07076_E_$1 הוא לא הנחיה או סימול מקומי
% This symbol is unknown.
asmr_w_using_defined_as_local=07077_E_משתמש בהגדרת השם כתווית מקומית
asmr_e_dollar_without_identifier=07078_E_השימוש בתו הדולר מתבצע ללא מזהה
% A constant expression has an identifier which does not start with
% the $ symbol.
asmr_w_32bit_const_for_address=07079_W_מספק כתובת של 32 סיביות לקבוע
% A constant was used as an address. This is probably an error,
% since using absolute addresses will probably not work.
asmr_n_align_is_target_specific=07080_N_השימוש ב.align מבוסס על סוג יעד, השתמש ב.balign או .p2align
% Using the .align directive is platform specific, and its meaning will vary
% from one platform to another.
asmr_e_cannot_access_field_directly_for_parameters=07081_E_לא ניתן להשתמש בצורה ישירה בשדות של פרמטרים
% You should load the parameter first into a register and then access the
% fields using that register.
asmr_e_cannot_access_object_field_directly=07082_E_לא ניתן לגשת בצורה ישירה לשדות של אובייקטים/מחלקות
% You should load the self pointer first into a register and then access the
% fields using the register as base. By default the self pointer is available
% in the esi register on i386.
asmr_e_unable_to_determine_reference_size=07083_E_לא ניתן לקבוע את גודל האופרנדים ללא ציון של הגודל
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference.
asmr_e_cannot_use_RESULT_here=07084_E_לא ניתן להשתמש בRESULT בפונקציה הנוכחית
% Some functions which return complex types cannot use the \var{result}
% keyword.
asmr_w_adding_explicit_args_fXX=07086_W_"$1" ללא תרגום של אופרנדים ל "$1 %st,%st(1)"
asmr_w_adding_explicit_first_arg_fXX=07087_W_"$1 %st(n)" תורגם ל "$1 %st,%st(n)"
asmr_w_adding_explicit_second_arg_fXX=07088_W_"$1 %st(n)" תורגם ל "$1 %st(n),%st"
asmr_e_invalid_char_smaller=07089_E_התו > אינו מורשה כאן
% The shift operator requires the << characters. Only one
% of those characters was found.
asmr_e_invalid_char_greater=07090_E_התו < אינו מורשה כאן
% The shift operator requires the >> characters. Only one
% of those characters was found.
asmr_w_align_not_supported=07093_W_ALIGN אינו נתמך
asmr_e_no_inc_and_dec_together=07094_E_השימוש INC ו DEC אינם יכולים להגיע בייחד
% Trying to use an increment and a decrement within the same
% opcode on the 680x0. This is impossible.
asmr_e_invalid_reg_list_in_movem=07095_E_שימוש בreglist ל movem לא תקין
% Trying to use the \var{movem} opcode with invalid registers
% to save or restore.
asmr_e_invalid_reg_list_for_opcode=07096_E_שימוש שגוי בreglist לopcode
asmr_e_higher_cpu_mode_required=07097_E_זקוק למצב מעבד גבוהה יותר ($1)
% Trying to use an instruction which is not supported in the current
% cpu mode. Use a higher cpu generation to be able to use this
% opcode in your assembler block
asmr_w_unable_to_determine_reference_size_using_dword=07098_W_לא צויין גודל של אופרנד ולא ניתן לנחש את הגודל, משתמש בDWORD כברירת מחדל
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use DWORD as default.
asmr_e_illegal_shifterop_syntax=07099_E_שגיאת תחביר בעת ניסיון לפרש הסטת אופרנד
% ARM only; ARM assembler supports a so called shifter operand. The used syntax isn't
% a valid shifter operand. Example for an operation with shifter operand:
% \begin{verbatim}
% asm
%   orr     r2,r2,r2,lsl #8
% end;
% \end{verbatim}
asmr_e_packed_element=07100_E_הכתובת של רכיב ארוז אינו נמצא בטווח של בית
% Packed components (record fields and array elements) may start at an arbitrary
% bit inside a byte. On CPU which do not support bit-addressable memory (which
% includes all currently supported CPUs by FPC) you will therefore get an error
% message when trying to index arrays with elements whose size is not a multiple
% of 8 bits. The same goes for accessing record fields with such an address.
% multiple of 8 bits.
asmr_w_unable_to_determine_reference_size_using_byte=07101_W_לא צויין גודל, והמהדר אינו מצליח לקבוע את גודל האופרנד, משתמש בגודל BYTE בתור ברירת מחדל
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use BYTE as default.
asmr_w_no_direct_ebp_for_parameter=07102_W_לא ניתן להשתמש ב+הסט(%ebp) במיקום הנוכי עבור הפרמטרים
% Using direct 8(%ebp) reference for function/procedure parameters is invalid
% if parameters are in registers.
asmr_w_direct_ebp_for_parameter_regcall=07103_W_השימוש ב+הסט(%ebp) אינו תואם עם מוסכמות ה regcall
% Using direct 8(%ebp) reference for function/procedure parameters is invalid
% if parameters are in registers.
asmr_w_direct_ebp_neg_offset=07104_W_השימוש ב-הסט(%ebp) אינו מומלץ עבור גישה למשתנים מקומיים
% Using -8(%ebp) to access a local variable is not recommended
asmr_w_direct_esp_neg_offset=07105_W_השימוש ב-הסט(%ebp) עלול לגרום לקריסה של התוכנית או לאיבוד המידע
% Using -8(%esp) to access a local stack is not recommended, as
% this stack portion can be overwritten by any function calls or interrupts.
asmr_e_no_vmtoffset_possible=07106_E_השימוש ב VMTOffset חייב להגיע בשילוב של מתודות ווירטואליות ו "$1" אינו ווירטואלי
% \end{verbatim}
#
# Assembler/binary writers
#
# 08018 is the last used one
#
asmw_f_too_many_asm_files=08000_F_יותר מידי קבצי assembler
% With smartlinking enabled, there are too many assembler
% files generated. Disable smartlinking.
asmw_f_assembler_output_not_supported=08001_F_סוג הפלט של הassembler אינו נתמך
asmw_f_comp_not_supported=08002_F_Comp אינו נתמך
asmw_f_direct_not_supported=08003_F_מצב ישיר של assembler אינו נתמך ע"י כותבים בינאריים
% Direct assembler mode is not supported for binary writers.
asmw_e_alloc_data_only_in_bss=08004_E_אתחול המידע מורשה רק בחלק הbss
asmw_f_no_binary_writer_selected=08005_F_לא נבחר כותב בינארי
asmw_e_opcode_not_in_table=08006_E_Asm: Opcode $1 הוא לא טבלה
asmw_e_invalid_opcode_and_operands=08007_E_Asm: $1 הינו שילוב שגוי בין opcode לבין אופרנד
asmw_e_16bit_not_supported=08008_E_Asm: התייחסות של 16 סיביות אינו נתמך
asmw_e_invalid_effective_address=08009_E_Asm: מען בפעול לא תקין
asmw_e_immediate_or_reference_expected=08010_E_Asm: מצפה להתייחסות לאופרנד או אופרנד מידי
asmw_e_value_exceeds_bounds=08011_E_Asm: $1 גולש מהגבולות $2
asmw_e_short_jmp_out_of_range=08012_E_Asm: Short jump מחוץ לטווח $1
asmw_e_undefined_label=08013_E_Asm: תווית לא מוגדרת $1
asmw_e_comp_not_supported=08014_E_Asm: הטיפוס COMP לא נתמך ביעד המבוקש
asmw_e_extended_not_supported=08015_E_Asm: הטיפוס Extended אינו נתמך ביעד המבוקש
asmw_e_duplicate_label=08016_E_Asm: תוויות כפולה $1
asmw_e_redefined_label=08017_E_Asm: מגדיר מחדש את התווית $1
asmw_e_first_defined_label=08018_E_Asm: מוגדר כאן לראשונה
asmw_e_invalid_register=08019_E_Asm: אוגר שגוי $1
asmw_e_16bit_32bit_not_supported=08020_E_Asm: ההתייחסות ל16 או 32 סיביות אינה נתמכת
asmw_e_64bit_not_supported=08021_E_Asm: האופרנט 64 סיביות אינו נתמך

#
# Executing linker/assembler
#
# 09034 is the last used one
#
# BeginOfTeX
%
% \section{Errors of assembling/linking stage}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
exec_w_source_os_redefined=09000_W_מקור מערכת ההפעלה מוגדר מחדש
% The source operating system is redefined.
exec_i_assembling_pipe=09001_I_מרכיב (צינור) $1
% Assembling using a pipe to an external assembler.
exec_d_cant_create_asmfile=09002_E_לא ניתן ליצור את קובץ הassembler $1
% The mentioned file can't be created. Check if you have got
% access permissions to create this file
exec_e_cant_create_objectfile=09003_E_לא ניתן ליצור את קובץ אובייקט $1
% The mentioned file can't be created. Check if you've
% got access permissions to create this file
exec_e_cant_create_archivefile=09004_E_לא ניתן ליצור את קובץ הארכיון $1
% The mentioned file can't be created. Check if you've
% access permissions to create this file
exec_e_assembler_not_found=09005_E_המקשר $1 אינו נמצא, עובר למאסף חיצוני
% The assembler program was not found. The compiler will produce a script that
% can be used to assemble and link the program.
exec_t_using_assembler=09006_T_משתמש במאסף: $1
% Information message saying which assembler is being used.
exec_e_error_while_assembling=09007_E_שגיאה בזמן איסוף קוד יציאה $1
% There was an error while assembling the file using an external assembler.
% Consult the documentation of the assembler tool to find out more information
% on this error.
exec_e_cant_call_assembler=09008_E_לא ניתן להריץ את המאסף, שגיאה $1, מריץ מאסף חיצוני
% An error occurred when calling an external assembler, The compiler will produce a script that
% can be used to assemble and link the program.
exec_i_assembling=09009_I_מקשר $1
% An informational message stating which file is being assembled.
exec_i_assembling_smart=09010_I_אוסף עם smartlinking $1
% An informational message stating which file is being assembled using smartlinking.
exec_w_objfile_not_found=09011_W_האובייקט $1 לא נמצא, נראה שהאיסוף יכשל !
% One of the object file is missing, and linking will probably fail.
% Check your paths.
exec_w_libfile_not_found=09012_W_הספרייה $1 לא נמצאה, נראה שהאיסוף יכשל !
% One of the library file is missing, and linking will probably fail.
% Check your paths.
exec_e_error_while_linking=09013_E_שגיאה בזמן קישור
% Generic error while linking.
exec_e_cant_call_linker=09014_E_לא ניתן להריץ את המקשר, עובר למקשר חיצוני
% An error occurred when calling an external linker, The compiler will produce a script that
% can be used to assemble and link the program.
exec_i_linking=09015_I_מקשר $1
% An informational message, showing which program or library is being linked.
exec_e_util_not_found=09016_E_הכלי $1 לא נמצא, עובר למקשר חיצוני
% An external tool was not found, the compiler will produce a script that
% can be used to assemble and link or postprocess the program.
exec_t_using_util=09017_T_משתמש בכלי $1
% An informational message, showing which external program (usually a postprocessor) is being used.
exec_e_exe_not_supported=09018_E_אין תמיכה ביצירת קבצי ריצה
% Creating executable programs is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_e_dll_not_supported=09019_E_אין תמיכה ביצירת ספריות דינמיות/משותפות
% Creating dynamically loadable libraries is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_i_closing_script=09020_I_סוגר את התסריט $1
% Informational message showing when the external assembling an linking script is finished.
exec_e_res_not_found=09021_E_מהדר משאבים לא נמצא, עובר למצב חיצוני
% An external resource compiler was not found, the compiler will produce a script that
% can be used to assemble, compile resources and link or postprocess the program.
exec_i_compilingresource=09022_I_מהדר את המשאב $1
% An informational message, showing which resource is being compiled.
exec_t_unit_not_static_linkable_switch_to_smart=09023_T_לא ניתן לקשר את היחידה $1 בצורה סטטית, עובר לקישור חכם
% Statical linking was requested, but a unit which is not statically linkable was used.
exec_t_unit_not_smart_linkable_switch_to_static=09024_T_לא ניתן לקשר בצורה חכמה את היחידה $1, עובר לקישור סטטי
% Smart linking was requested, but a unit which is not smart-linkable was used.
exec_t_unit_not_shared_linkable_switch_to_static=09025_T_לא ניתן לקשר את היחידה $1 בקישור משותף, עובר לקישור סטטי
% Shared linking was requested, but a unit which is not shared-linkable  was used.
exec_e_unit_not_smart_or_static_linkable=09026_E_לא ניתן לקשר את היחידה $1 בקישור חכם או בקישור סטטי
% Smart or static linking was requested, but a unit which cannot be used for either  was used.
exec_e_unit_not_shared_or_static_linkable=09027_E_לא ניתן לקשר את היחידה $1 בקישור משותף או סטטי
% Shared or static linking was requested, but a unit which cannot be used for either  was used.
exec_d_resbin_params=09028_D_מריץ את מהדר המשאבים "$1" עם "$2" בתור שורת פקודה
% An informational message showing which command-line is used for the resource compiler.
%\end{description}
# EndOfTeX

#
# Executable information
#
# BeginOfTeX
% \section{Executable information messages.}
% This section lists all messages that the compiler emits when an executable program is produced,
% and only when the internal linker is used.
% \begin{description}
execinfo_f_cant_process_executable=09128_F_לא מצליח לבצע מעבר סופי על קובץ ההרצה $1
% Fatal error when the compiler is unable to post-process an executable.
execinfo_f_cant_open_executable=09129_F_לא מצליח לפתוח את קובץ הריצה $1
% Fatal error when the compiler cannot open the file for the executable.
execinfo_x_codesize=09130_X_גודל הקוד: $1 בתים
% Informational message showing the size of the produced code section.
execinfo_x_initdatasize=09131_X_הגודל של מידע מאותחל: $1 בתים
% Informational message showing the size of the initialized data section.
execinfo_x_uninitdatasize=09132_X_מידע של מידע לא מאותחל: $1 בתים
% Informational message showing the size of the uninitialized data section.
execinfo_x_stackreserve=09133_X_גודל המחסנית השמורה: $1 בתים
% Informational message showing the stack size that the compiler reserved for the executable.
execinfo_x_stackcommit=09134_X_גודל המחסנית בשימוש: $1 בתים
% Informational message showing the stack size that the compiler commites for the executable.
%\end{description}
# EndOfTeX

#
# Unit loading
#
# 10041 is the last used one
#
# BeginOfTeX
% \section{Unit loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a unit from disk into memory. Many of these messages are
% informational messages.
% \begin{description}
unit_t_unitsearch=10000_T_מחפש את היחידה: $1
% When you use the \var{-vt}, the compiler tells you where it tries to find
% unit files.
unit_t_ppu_loading=10001_T_טוען את קובץ ה PPU $1
% When the \var{-vt} switch is used, the compiler tells you
% what units it loads.
unit_u_ppu_name=10002_U_שם ה PPU: $1
% When you use the \var{-vu} flag, the unit name is shown.
unit_u_ppu_flags=10003_U_דגלי PPU: $1
% When you use the \var{-vu} flag, the unit flags are shown.
unit_u_ppu_crc=10004_U_PPU Crc: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_time=10005_U_זמן PPU: $1
% When you use the \var{-vu} flag, the time the unit was compiled is shown.
unit_u_ppu_file_too_short=10006_U_קובץ הPPU קצר מידי
% The ppufile is too short, not all declarations are present.
unit_u_ppu_invalid_header=10007_U_הראש של קובץ ה PPU אינו תקין (הקובץ אינו מכיל PPU בהתחלה)
% A unit file contains as the first three bytes the ascii codes of \var{PPU}
unit_u_ppu_invalid_version=10008_U_גרסה לא תקינה של הPPU $1
% This unit file was compiled with a different version of the compiler, and
% cannot be read.
unit_u_ppu_invalid_processor=10009_U_קובץ הPPU הודר למעבד אחר
% This unit file was compiled for a different processor type, and
% cannot be read
unit_u_ppu_invalid_target=10010_U_קובץ הPPU הודר למטרה אחרת
% This unit file was compiled for a different target, and
% cannot be read
unit_u_ppu_source=10011_U_מקור הPPU: $1
% When you use the \var{-vu} flag, the unit source file name is shown.
unit_u_ppu_write=10012_U_כותב $1
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the unit file.
unit_f_ppu_cannot_write=10013_F_לא כותב את קובץ ה PPU
% An error occurred when writing the unit file.
unit_f_ppu_read_error=10014_F_שגיאה בזמן קריאת קובץ ה PPU
% This means that the unit file was corrupted, and contains invalid
% information. Recompilation will be necessary.
unit_f_ppu_read_unexpected_end=10015_F_סוף קובץ לא צפוי (קובץ PPU)
% Unexpected end of file. This may mean that the PPU file is
% corrupted.
unit_f_ppu_invalid_entry=10016_F_כניסה לא תקינה בקובץ ה PPU: $1
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
unit_f_ppu_dbx_count_problem=10017_F_בעיה בספירת Dbx בקובץ PPU
% There is an inconsistency in the debugging information of the unit.
unit_e_illegal_unit_name=10018_E_שם יחידה לא תקין: $1
% The name of the unit does not match the file name.
unit_f_too_much_units=10019_F_יותר מידי יחידות
% \fpc has a limit of 1024 units in a program. You can change this behavior
% by changing the \var{maxunits} constant in the \file{fmodule.pas} file of the
% compiler, and recompiling the compiler.
unit_f_circular_unit_reference=10020_F_קריאה מעגלית בין היחידות $1 ו $2
% Two units are using each other in the interface part. This is only allowed
% in the \var{implementation} part. At least one unit must contain the other one
% in the \var{implementation} section.
unit_f_cant_compile_unit=10021_F_אין מספיק משאבים על מנת להדר את היחידה $1
% A unit was found that needs to be recompiled, but no sources are
% available.
unit_f_cant_find_ppu=10022_F_לא מוצא את היחידה $1
% You tried to use a unit of which the PPU file isn't found by the
% compiler. Check your configuration file for the unit paths
unit_w_unit_name_error=10023_W_היחידה $1 לא נמצאה, אבל $2 קיים
% This error message is no longer used.
unit_f_unit_name_error=10024_F_מחפש את היחידה $1, אבל $2 נמצא
% Dos truncation of 8 letters for unit PPU files
% may lead to problems when unit name is longer than 8 letters.
unit_w_switch_us_missed=10025_W_הידור יחידת system דורשת את המתג -Us
% When recompiling the system unit (it needs special treatment), the
% \var{-Us} must be specified.
unit_f_errors_in_unit=10026_F_עוצר לאחר שנמצאו $1 שגיאות בזמן הידור המודול
% When the compiler encounters a fatal error or too many errors in a module
% then it stops with this message.
unit_u_load_unit=10027_U_טוען $1 ($2) יחידה $3
% When you use the \var{-vu} flag, which unit is loaded from which unit is
% shown.
unit_u_recompile_crc_change=10028_U_מהדר את $1 לאחר שהחתימה שונתה ל $2
% The unit is recompiled because the checksum of a unit it depends on has 
% changed.
unit_u_recompile_source_found_alone=10029_U_נמצא רק קוד מקור, מהדר מחדש את $1
% When you use the \var{-vu} flag, these messages tell you why the current
% unit is recompiled.
unit_u_recompile_staticlib_is_older=10030_U_מהדר מחדש את היחידה. הספרייה הסטטית ישנה יותר מקובץ ה PPU
% When you use the \var{-vu} flag, the compiler warns if the static library
% of the unit are older than the unit file itself.
unit_u_recompile_sharedlib_is_older=10031_U_מהדר מחדש את היחידה. הספרייה המשותפת ישנה יותר מקובץ ה PPU
% When you use the \var{-vu} flag, the compiler warns if the shared library
% of the unit are older than the unit file itself.
unit_u_recompile_obj_and_asm_older=10032_U_מהדר מחדש את ה היחידה. קבצי ה obj והasm ישנים יותר מקובץ ה PPU
% When you use the \var{-vu} flag, the compiler warns if the assembler or
% object file of the unit are older than the unit file itself.
unit_u_recompile_obj_older_than_asm=10033_U_מהדר מחדש את היחידה. קובץ הobj ישן יותר מקובץ הasm
% When you use the \var{-vu} flag, the compiler warns if the assembler
% file of the unit is older than the object file of the unit.
unit_u_parsing_interface=10034_U_מפרש את חלק הממשק של $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the interface part of the unit
unit_u_parsing_implementation=10035_U_מפרש את חלק הביצועי של $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the implementation part of the unit
unit_u_second_load_unit=10036_U_טעינה שנייה של היחידה $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_check_time=10037_U_בדיקת קובץ PPU $1 זמן $2
% When you use the \var{-vu} flag, the compiler show the filename and
% date and time of the file which a recompile depends on
### The following two error msgs is currently disabled.
#unit_h_cond_not_set_in_last_compile=10038_H_התנאים של $1 לא הוגדרו בהתחלת הריצה בהידור האחרון של $2
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompiliation. The compiler has
#% found a conditional that currently is defined, but was not used the last
#% time the unit was compiled.
#unit_h_cond_set_in_last_compile=10039_H_התנאים של $1 הוגדרו בהתחלת הריצה בהידור האחרון של $2
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompiliation. The compiler has
#% found a conditional that was used the last time the unit was compiled, but
#% the conditional is currently not defined.
unit_w_cant_compile_unit_with_changed_incfile=10040_W_לא ניתן להדר את היחידה $1 למרות שקבצי הinclude שונו
% A unit was found to have modified include files, but
% some source files were not found, so recompilation is impossible.
unit_u_source_modified=10041_U_הקובץ $1 חדש יותר מהקובץ PPU $2
% A modified source file for a compiler unit was found.
unit_u_ppu_invalid_fpumode=10042_U_מנסה להשתמש ביחידה אשר הודרה למצב FPU שונה
% Trying to compile code while using units which were not compiled with
% the same floating point format mode. Either all code should be compiled
% with FPU emulation on, or with FPU emulation off.
unit_u_loading_interface_units=10043_U_טוען את חלק הממשק של היחידות מ $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the units defined in the interface part of the unit.
unit_u_loading_implementation_units=10044_U_טוען את החלק הביצועי של היחידות מ $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the units defined in the implementation part of the unit.
unit_u_interface_crc_changed=10045_U_חתימת ה CRC של חלק הממשק השתנה ביחידה $1
% When you use the \var{-vu} flag, the compiler warns that it the
% CRC calculated for the interface has been changed after the implementation
% has been parsed.
unit_u_implementation_crc_changed=10046_U_חתימת הCRC של החלק הביצועי השתנה ליחידה $1
% When you use the \var{-vu} flag, the compiler warns that it the
% CRC calculated has been changed after the implementation
% has been parsed.
unit_u_finished_compiling=10047_U_הסתיים הידור היחידה $1
% When you use the \var{-vu} flag, the compiler warns that it
% has finished compiling the unit.
unit_u_add_depend_to=10048_U_מוסיף את התלות של $1 ל$2
% When you use the \var{-vu} flag, the compiler warns that it
% has added a dependency between the two units.
unit_u_no_reload_is_caller=10049_U_אין קריאה מחודשת של הקובץ בשביל $1
% When you use the \var{-vu} flag, the compiler warns that it
% has will not reload the unit because it is the unit that wants
% to load this unit
unit_u_no_reload_in_second_compile=10050_U_אין טעינה מחדש, בזמן הידור שני של היחידה $1
% When you use the \var{-vu} flag, the compiler warns that it
% has will not reload the unit because it is already in a second recompile
unit_u_flag_for_reload=10051_U_דגל לטעינה מחודשת: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to reload the unit
unit_u_forced_reload=10052_U_כופה על טעינה מחודשת
% When you use the \var{-vu} flag, the compiler warns that it
% has is reloading the unit because it was required
unit_u_previous_state=10053_U_המצב הישן של $1: $2
% When you use the \var{-vu} flag, the compiler shows the
% previous state of the unit
unit_u_second_compile_unit=10054_U_נמצא כבר בתהליך הידור $1, מיישם הידור שני
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_loading_unit=10055_U_טוען את היחידה $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the unit.
unit_u_finished_loading_unit=10056_U_מסיים לטעון את היחידה $1
% When you use the \var{-vu} flag, the compiler warns that it finished
% loading the unit.
unit_u_registering_new_unit=10057_U_רושם יחידה חדשה $1
% When you use the \var{-vu} flag, the compiler warns that it has
% found a new unit and registers it in the internal lists.
unit_u_reresolving_unit=10058_U_מחשב מחדש את היחידה $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to recalculate the internal data of the unit
unit_u_skipping_reresolving_unit=10059_U_מדלג על טעינה מחודשת של היחידה $1, עדיין בזמן טעינת יחידות בשימוש
% When you use the \var{-vu} flag, the compiler warns that it
% skips to recalculate the internal data of the unit because there
% is no data to recalculate
% \end{description}
# EndOfTeX

#
#  Options
#
# 11041 is the last used one
#
option_usage=11000_O_$1 [options] <inputfile> [options]
# BeginOfTeX
%
% \section{Command-line handling errors}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
option_only_one_source_support=11001_W_תומך רק בקובץ מקור אחד בשורת הפקודה
% You can specify only one source file on the command line. The first
% one will be compiled, others will be ignored. This may indicate that
% you forgot a \var{'-'} sign.
option_def_only_for_os2=11002_W_ניתן ליצור את קובץ ה DEF רק לOS/2
% This option can only be specified when you're compiling for OS/2
option_no_nested_response_file=11003_E_אין תמיכה בקבצי תגובה מקוננים
% you cannot nest response files with the \var{@file} command-line option.
option_no_source_found=11004_F_לא צויין קובץ מקור בשורת הפקודה
% The compiler expects a source file name on the command line.
option_no_option_found=11005_N_לא צוינו אפשרויות בתוך קובץ ההגדרות $1
% The compiler didn't find any option in that config file.
option_illegal_para=11006_E_פרמטר לא חוקי: $1
% You specified an unknown option.
option_help_pages_para=11007_H_-? כותב דפי עזרה
% When an unknown option is given, this message is diplayed.
option_too_many_cfg_files=11008_F_יותר מידי קבצי הגדרות מקוננים
% You can only nest up to 16 config files.
option_unable_open_file=11009_F_לא ניתן לפתוח את הקובץ $1
% The option file cannot be found.
option_reading_further_from=11010_D_קורא הגדרות נוספות מ $1
% Displayed when you have notes turned on, and the compiler switches
% to another options file.
option_target_is_already_set=11011_W_המטרה כבר הוגדרה ל: $1
% Displayed if more than one \var{-T} option is specified.
option_no_shared_lib_under_dos=11012_W_DOS אינו תומך בספריות משותפות, חוזר לספריות סטטיות
% If you specify \var{-CD} for the \dos platform, this message is displayed.
% The compiler supports only static libraries under \dos
option_too_many_ifdef=11013_F_IF(N)DEF רבים מידי
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_many_endif=11014_F_ENDIF רבים מידי
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_less_endif=11015_F_תנאי פתוח בסוף הקובץ
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_no_debug_support=11016_W_מידע ניפוי שגיאות אינו נתמך בסוג קובץ הריצה המבוקש
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_no_debug_support_recompile_fpc=11017_H_נסה להדר מחדש עם המתג -dGDB
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_obsolete_switch=11018_W_אתה משתמש במתג המיושן $1
% this warns you when you use a switch that is not needed/supported anymore.
% It is recommended that you remove the switch to overcome problems in the
% future, when the switch meaning may change.
option_obsolete_switch_use_new=11019_W_אתה משתמש במתג המיושן $1, השתמש במתג $2 במקום
% this warns you when you use a switch that is not supported anymore. You
% must now use the second switch instead.
% It is recommended that you change the switch to overcome problems in the
% future, when the switch meaning may change.
option_switch_bin_to_src_assembler=11020_N_מעביר את האוסף למצב ברירת מחדל של כתיבת מאסף
% this notifies you that the assembler has been changed because you used the
% -a switch which can't be used with a binary assembler writer.
option_incompatible_asm=11021_W_התוכן הנבחר של הassembler "$1" אינו מתאים ל "$2"
option_asm_forced=11022_W_התוכן של המאסף "$1" לא מסוגל ליצור קבצי אובייקט, משתמש במאסף ברירת המחדל במקום
% The assembler output selected can not generate
% object files with the correct format. Therefore, the
% default assembler for this target is used instead.
option_using_file=11026_T_קורא אפשרויות מהקובץ $1
% Options are also read from this file
option_using_env=11027_T_קורא אפשרויות מהסביבה $1
% Options are also read from this environment string
option_handling_option=11028_D_מטפל באפשרות "$1"
% Debug info that an option is found and will be handled
option_help_press_enter=11029__*** לחץ על enter ***
option_start_reading_configfile=11030_H_מתחיל בקריאת קובץ ההגדרות $1
% Starting of config file parsing.
option_end_reading_configfile=11031_H_סיום קריאת קובץ ההגדרות $1
% End of config file parsing.
option_interpreting_option=11032_D_מפרש את האפשרות "$1"
option_interpreting_firstpass_option=11036_D_מפרש את המעבר הראשון על האפשרות "$1"
option_interpreting_file_option=11033_D_מפרש את הגדרת הקובץ "$1"
option_read_config_file=11034_D_קורא את קובץ ההגדרות "$1"
option_found_file=11035_D_נמצא קובץ מקור בשם "$1"
% Additional infos about options, displayed
% when you have debug option turned on.
option_code_page_not_available=11039_E_קוד דף לא ידוע
option_config_is_dir=11040_F_נמצאה ספרייה במקום קובץ ההגדרות $1
% Directories can not be used as configuration files.
option_confict_asm_debug=11041_W_סוג הפלט של המאסף שנבחר "$1" אינו יכול ליצור מידע עבור ניפוי שגיאות. מבטל ניפוי שגיאות
% The assembler output selected can not generate
% debugging information, debugging option is therefore disabled.
%\end{description}
# EndOfTeX

#
# Logo (option -l)
#
option_logo=11023_[
Free Pascal Compiler version $FPCFULLVERSION [$FPCDATE] for $FPCCPU
Copyright (c) 1993-2020 by Florian Klaempfl and others
]

#
# Info (option -i)
#
option_info=11024_[
Free Pascal Compiler version $FPCVERSION

תאריך מהדר : $FPCDATE
מהדר למעבד: $FPCCPU

מערכות הפעלה נתמכות:
  $OSTARGETS

הוראות מעבד נתמכות:
  $INSTRUCTIONSETS

הוראות נתמכות של יחידת נקודה צפה:
  $FPUINSTRUCTIONSETS

תמיכת המיטוב:
  $OPTIMIZATIONS

התוכנה מוגשת תחת רישיון GNU General Public License
למידע נוסף יש לקרוא את COPYING.v2

דיווח על תקלות (באגים), הצעות וכו':
                 http://bugs.freepascal.org
]

#
# Help pages (option -? and -h)
#
# The first character on the line indicates who will display this
# line, the current possibilities are :
#    * = every target
#    3 = 80x86 targets
#    6 = 680x0 targets
#    e = in extended debug mode only
#    P = PowerPC targets
#    S = Sparc targets
#    V = Virtual machine targets
# The second character also indicates who will display this line,
# (if the above character was TRUE) the current possibilities are :
#    * = everyone
#    g = with GDB info supported by the compiler
#    O = OS/2
#    L = UNIX systems
#  The third character represents the indentation level.
#
option_help_pages=11025_[
**0*_הוסף את הסימן + לאחר אפשרות מתג בוליאני לאפשר את האפשרות, הוסף את הסימן - לבטל את האפשרות
**1a_המהדר אינו מוחק את קובץ הassembler שנוצר
**2al_רושם רשימה של שורות קוד מקור בקובץ הassembler
**2an_רשימת מידע של צמתים בקובץ הassembler
*L2ap_השתמש בצינורות במקום קבצי assembler זמניים
**2ar_רשימה אודות רישום של הקצאה/שחרור בקובץ הassembler
**2at_רשימה זמנית אודות הקצאה/שחרור בקובץ הassembler
**1A<x>_תסדירי פלט:
**2Adefault_השתמש במאסף ברירת מחדל
3*2Aas_אסוף עם GNU AS
3*2Anasmcoff_קובץ COFF (Go32v2) בשימוש עם Nasm
3*2Anasmelf_קובץ ELF32 (Linux) בשימוש עם Nasm
3*2Anasmwin32_קובץ אובייקט Win32 בשימוש עם Nasm
3*2Anasmwdosx_קובץ אובייקט Win32/WDOSX בשימוש עם Nasm
3*2Awasm_קובץ obj בשימוש עם Wasm (Watcom)
3*2Anasmobj_קובץ obj בשימוש עם Nasm
3*2Amasm_קובץ obj בשימוש עם Masm (Microsoft)
3*2Atasm_קובץ obj בשימוש Tasm (Borland)
3*2Aelf_קובץ ELF32 (Linux) בשימוש כותב פנימי
3*2Acoff_קובץ OFF (Go32v2) בשימוש כותב פנימי
3*2Apecoff_קובץ pecoff (Win32) בשימוש כותב פנימי
4*2Aas_אסוף עם GNU AS
6*2Aas_קובץ-o Unix בשימוש GNU AS
6*2Agas_GNU Motorola assembler
6*2Amit_MIT Syntax (GAS ישן)
6*2Amot_Motorola assembler תקני
A*2Aas_אסוף עם GNU AS
P*2Aas_אסוף עם GNU AS
S*2Aas_אסוף עם GNU AS
**1b_ייצר מידע לדפדוף
**2bl_ייצר מידע על סמלים מקומיים
**1B_בנה את כל המודולים
**1C<x>_אפשרויות יצירת קוד:
**2Cc<x>_קובע מוסכמות הפעלה ברירת מחדל ל <ס>
**2CD_צור גם ספרייה דינאמית (לא נתמך)
**2Ce_מהדר עם הדמייה של opcodes השייכים לנקודה צפה
**2Cf<x>_בחר הוראות FPU לשימוש, ראה fpc -i לערכים אפשריים
**2CF<x>_דיוק נקודה עשרונית קבועה מינימלית (default, 32, 64)
**2Cg_צור קוד PIC
**2Ch<n>_כמות של <n> בתים ל מצבור (בין 1023 ו67107840)
**2Ci_בדיקת IO
**2Cn_השמט מצב קישור
**2Co_בדוק גלישה של פעולות מספר שלם
**2CO_בדוק עבור פעולות גלישה אפשרויות של מספר שלם
**2Cp<x>_בחר קבוצת הוראות, ראה fpc -i לערכים אפשריים
**2CP<x>=<y>_הגדרות אריזה
**3CPPACKSET=<y>_ <y> מסדר הקצאה: 0, 1 או DEFAULT או NORMAL, 2, 4 ו 8
**2Cr_בדיקת טווח
**2CR_וודא קריאה תקינה למתודה
**2Cs<n>_קבע גודל מחסנית ל <n>
**2Ct_בדיקת מחסנית
**2CX_צור גם ספרייה של קישור חכם
**1d<x>_מגדיר את הסמל <x>
**1D_יוצר קובץ DEF
**2Dd<x>_יוצר תאור ל <x>
**2Dv<x>_יוצר גרסת DLL ל <x>
*O2Dw_ישום PM
**1e<x>_קובע נתיב לקובץ ריצה
**1E_זהה ל -Cn
**1fPIC_זהה ל -Cg
**1F<x>_קובע שם קבצים ומיקומים:
**2Fa<x>[,y]_קודם טוען את היחידות <x> ו [y] ראשונות, לפני ניתוח שורת ה uses
**2Fc<x>_קובע קוד דף של קלט ל<x>
**2FC<x>_קבע שם מהדר RC בינארי ל <x>
**2FD<x>_קובע את הספרייה בה אפשר לחפש את כלי העזר של המהדר
**2Fe<x>_הפנה הודעות שגיאה ל<x>
**2Ff<x>_הוסף את <x> לנתיב המסגרת (רק ב Darwin)
**2FE<x>_הפנה פלט של exe/unit לנתיב <x>
**2Fi<x>_מוסיף את <x> לרשימת הנתיבים
**2Fl<x>_מוסיף את <x> לרשימת הנתיבים של הספריה
**2FL<x>_משתמש ב<x> כמקשר דינאמי
**2Fm<x>_טוען טבלת המרה של יוניקוד מהקובץ x>.txt> מספריית המהדר
**2Fo<x>_מוסיף את <x> לרשינת הנתיבים של אובייקט
**2Fr<x>_טוען קובץ הודעות שגיאה <x>
**2FR<x>_קבע מקשר לקובץ res ל <x>
**2Fu<x>_מוסיף את <x> לרשימת הנתיבים של יחידה
**2FU<x>_קובע את המיקום הפלט של היחידות ל <x> ומשכתב את -FE
*g1g_יוצר מידע לניפוי שגיאות
*g2gc_יוצר בדיקות למצביעים
*g2gh_השתמש ביחידה heaptrace (עבור דליפות זכרון/בעיות בניפוי שגיאות)
*g2gl_השתמש ביחידת מידע של שורה להציג מידע נוסף לbacktraces
*g2go<x>_צור אפשרויות לניפוי שגיאות
*g3godwarfsets_אפשר מידע לניהול מידע עבור ניפוי שגיאות של Dwarf (שובר את gdb < 6.5)
*g2gp_משמר גודל שמות סמלי הstabs
*g2gs_מייצר מידע לניפוי שגיאות stub
*g2gt_לכלך משתנים מקומיים (לזיהוי מידע לא מאותחל)
*g2gv_מייצר תוכנות עם יכולת מעקב של valgrind
*g2gw_מייצר מידע לניפוי שגיאות לdwarf
*g2gw2_מיצר מידע לניפוי שגיאות dwarf-2
*g2gw3_מיצר מידע לניפוי שגיאות dwarf-3
**1i_מידע
**2iD_הצג תאריך המהדר
**2iV_הצג גרסת המהדר
**2iW_הצג גרסה מלאה של המהדר
**2iSO_הצג מערכת הפעלה של המהדר
**2iSP_הצג גרסת מעבד של המהדר
**2iTO_הצג את המטרה של מערכת ההפעלה 
**2iTP_הצג את המטרה של המעבד
**1I<x>_הוסף את <x> לרשימת הנתיבים להוספה
**1k<x>_העבר את <x> למקשר
**1l_כתוב סמליל
**1M<x>_הגדר מצב שפה ל <x>
**2Mfpc_דיאלקט של Free Pascal (ברירת מחדל)
**2Mobjfpc_אפשר כמה תוספות של Delphi 2
**2Mdelphi_מנסה להיות תואם Delphi
**2Mtp_מנסה להיות תואם ל TP/BP 7.0
**2Mmacpas_מנסה להיות תואם ללהג של Macintosh Pascal
**1n_אל תקרא את קובץ הגדרות ברירת המחדל
**1o<x>_שנה את שם קובץ הריצה שהתקבל ל <x>
**1O<x>_מיטובים:
**2O-_בטל מיטוב
**2O1_מיטוב רמה 1 (מהיר וטוב לניפוי שגיאות)
**2O2_מיטוב רמה 2 (-O1 + מיטוב מהיר)
**2O3_מיטוב רמה 3 (-O2 + מיטוב איטי)
**2Oa<x>=<y>_קבע יישור
**2Oo[NO]<x>_ע"מ לאפשר או לאפשר מיטובים, ראה fpc -i לאפשרויות
**2Op<x>_לקביעת מיטוב למעבד נבחר, ראה fpc -i לאפשרויות
**2Os_צור קוד קטן
**1pg_צור קוד פרופיל עבור gprof (מגדיר את FPC_PROFILE)
**1R<x>_סגנון קריאת האסף:
**2Rdefault_השתמש במאסף ברירת מחדל
3*2Ratt_קרא סגנון מקשר של AT&T
3*2Rintel_קרא סגנון מקשר של Intel
4*2Ratt_קרא סגנון מקשר של AT&T
4*2Rintel_קרא סגנון מקשר של Intel
8*2Ratt_קרא סגנון מקשר של AT&T
8*2Rintel_קרא סגנון מקשר של Intel
6*2RMOT_קרא סגנון מקשר של Motorola
**1S<x>_אפשרויות תחביר:
**2S2_זהה ל -Mobjfpc
**2Sc_תומך אופרטורים בסגנון C (*=, +=, /= ו -=)
**2Sa_הוסף קוד טענת תנאי קביעה (assertion)
**2Sd_זהה ל -Mdelphi
**2Se<x>_אפשרויות שגיאה. <x> הינו השילוב הבא:
**3*_<n> : המהדר עוצר לאחר <n> שגיאות (ברירת מחדל היא 1)
**3*_w : המהדר עוצר גם לאחר אזהרות
**3*_n : המהדר עוצר לאחר הערות
**3*_h : המהדר עוצר לאחר רמזים
**2Sg_אפשר LABEL ו GOTO
**2Sh_השתמש ב ansistrings
**2Si_תמוך בסגנון C++ של INLINE
**2Sk_טען את היחידה fpcylix
**2SI<x>_קבע סגנון ממשק ל<x>
**3SIcom_תואם ממשקי COM (ברירת מחדל)
**3SIcorba_תואם ממשקי CORBA
**2Sm_תמוך במקרו דומים לC (גלובליים)
**2So_זהה ל -Mtp
**2Ss_שם יוצר חייב להיות init (שם הורס חייב להיות done)
**2Sx_אפשר מילות מפתח לexception (ברירת מחדל במצבי Delphi/ObjFPC)
**1s_אל תקרא למאסף והמקשר
**2sh_צור תסריט לקישור במארח
**2st_צור תסריט לקישור במטרה
**2sr_דלג על רישום הקצאת מופע (השתמש בייחד עם -alr)
**1T<x>_מטרת מערכת ההפעלה:
3*2Temx_OS/2 בשימוש EMX ( בייחד עם הרחבת EMX/RSX)
3*2Tfreebsd_FreeBSD
3*2Tgo32v2_גרסה 2 של הרחבת DJ Delorie DOS 
3*2Tlinux_Linux
3*2Tnetbsd_NetBSD
3*2Tnetware_Novell Netware Module (clib)
3*2Tnetwlibc_Novell Netware Module (libc)
3*2Topenbsd_OpenBSD
3*2Tos2_OS/2 / eComStation
3*2Twatcom_Watcom תואם הרחבת DOS
3*2Twdosx_WDOSX הרחבת DOS
3*2Twin32_Windows 32 סיביות
3*2Twince_Windows CE
4*2Tlinux_Linux
6*2Tamiga_Commodore Amiga
6*2Tatari_Atari ST/STe/TT
6*2Tlinux_Linux/m68k
6*2Tmacos_Macintosh m68k (לא נתמך)
6*2Tpalmos_PalmOS
A*2Tlinux_Linux
A*2Twince_Windows CE
P*2Tamiga_AmigaOS על PowerPC
P*2Tdarwin_Darwin ו Mac OS X על PowerPC
P*2Tlinux_Linux על PowerPC
P*2Tmacos_Mac OS (קלאסי) על PowerPC
P*2Tmorphos_MorphOS
S*2Tlinux_Linux
**1u<x>_מסיר את ההגדרה של הסמל <x>
**1U_הגדרות יחידה:
**2Un_אל תבדוק את שם היחידה
**2Ur_צור קבצי שחרור של יחידות
**2Us_הדר את יחידת הsystem
**1v<x>_תהיה מפורט יותר. <x> הינו שילוב של התווים הבאים:
**2*_e : הצג הודעות שגיאה (ברירת מחדל)
**2*_0 : אל תציג כלום (למעט הודעות שגיאה)
**2*_w : הצג אזהרות
**2*_u : הצג מידע על יחידה
**2*_n : הצג הערות
**2*_t : הצג קבצים שהיו בשימוש
**2*_h : הצג רמזים
**2*_c : הצג תנאים
**2*_i : הצג מידע כללי
**2*_d : הצג מידע לנפוי שגיאות
**2*_l : הצג מספרי שורות
**2*_r : מצב תואם Rhide/GCC
**2*_a : הצג הכול
**2*_x : הצג מידע על קובץ הריצה (רק בWin32)
**2*_b : כתוב הודעות עם שמות קבצים ונתיבים מלאים
**2*_v : כתוב את הקובץ fpcdebug.txt עם הרבה מידע על ניפוי שגיאות
**2*_p : כתוב את הקובץ tree.log עם ניתוח עץ
3*1W<x>_אפשרות מבוססת מטרה (מטרות)
A*1W<x>_אפשרות מבוססת מטרה (מטרות)
P*1W<x>_אפשרות מבוססת מטרה (מטרות)
3*2Wb_צור חבילה במקום ספרייה (Darwin)
P*2Wb_צור חבילה במקום ספרייה (Darwin)
p*2Wb_צור חבילה במקום ספרייה (Darwin)
3*2WB_צור תמונה הניתנת למיקום מחודש (Windows)
A*2WB_צור תמונה הניתנת למיקום מחודש (Windows, Symbian)
3*2WC_מציין אפליקציה מסוג מסוף (EMX, OS/2, Windows)
A*2WC_מציין אפליקציה מסוג מסוף (Windows)
P*2WC_מציין אפליקציה מסוג מסוף (Classic Mac OS)
3*2WD_השתמש ב DEFFILE לייצא פנקציות של DLL או EXE (Windows)
A*2WD_השתמש ב DEFFILE לייצא פנקציות של DLL או EXE (Windows)
3*2WF_מציין אפליקציה מסוג מסך מלא (EMX, OS/2)
3*2WG_מציין אפליקציה מסוג גרפי (EMX, OS/2, Windows)
A*2WG_מציין אפליקציה מסוג גרפי (Windows)
P*2WG_מציין אפליקציה מסוג גרפי (Classic Mac OS)
3*2WN_אל תצור קוד משנה מיקום, נדרש עבור ניפוי שגיאות (Windows)
A*2WN_אל תצור קוד משנה מיקום, נדרש עבור ניפוי שגיאות (Windows)
3*2WR_תיצור קוד משנה מיקום (Windows)
A*2WR_תיצור קוד משנה מיקום (Windows)
P*2WT_ציין אפליקציה מסוג כלי MPW (Classic Mac OS)
**1X_הגדרות ריצה:
**2Xc_העבר --shared/-dynamic למקשר (BeOS, Darwin, FreeBSD, Linux)
**2Xd_אל תשתמש בנתיב חיפוש הספריות הסטנדרטיות (נדרש עבור הידור לארכיטקטורות אחרות)
**2Xe_השתמש במקשר חיצוני
**2XD_נסה לקשר יחידות בצורה דינאמית (מגדיר FPC_LINK_DYNAMIC)
**2Xi_השתמש במקשר פנימי
**2Xm_צור מפת קישורים
**2XM<x>_הגדר את השם של רוטינה ה'עיקרית' של התוכנה (ברירת מחדל הוא 'main')
**2XP<x>_צרף את השמות המגיעים עם התחילית <x> מbinutils
**2Xr<x>_הגדר את נתיב חיפוש הספריות ל <x> (נדרש עבור הידור לארכיטקטורות אחרות) (BeOS, Linux)
**2XR<x>_צרף את <x> לכל נתיבי החיפוש של המקשר (BeOS, Darwin, FreeBSD, Linux, Mac OS, Solaris)
**2Xs_נקה את כל הסמלים מקבצי ההרצה
**2XS_נסה לקשר יחידות בצורה סטטית (ברירת מחדל, מגדיר FPC_LINK_STATIC)
**2Xt_קשר עם ספריות סטטיות (מעביר -static למקשר)
**2XX_תנסה לבצע smartlink ליחידות (מגדיר FPC_LINK_SMART)
**1*_
**1?_הצג את עזרה זו
**1h_הצג עזרה זו ללא לחכות

]

#
# The End...
